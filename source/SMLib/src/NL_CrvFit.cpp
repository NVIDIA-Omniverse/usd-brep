// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvFit.c: Curve fitting routines                                   */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#define ROUND(x)  (NL_INDEX)(x + 0.5)

/* Prototypes only referenced within CrvFit.c */

static NL_VOID ST_CalcCPtsCircle( NL_POINT, NL_POINT, NL_POINT, NL_CPOINT * );

#if NLIB_UNUSED

/**********************************************************************/
/* N_FitArcToPts: Best fitting circle or circular arc to a set of points   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine  computes a  least-squares circle or circular
     arc  to a  set of  2-D points. An initial  least-squares  circle is
     computed, and  then it is  improved  by  Gauss-Newton  iteration to
     minimize the  true distances from  the circle. If the least-squares
     line is better suited, i.e., the average error for the line is less
     than  that of  the circle, a  line segment  is returned. A  typical 
     calling example is:

       NL_FLAG    cut;
       NL_POINT   *P;
       NL_INDEX   n;
       NL_REAL    tol, era, erm;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get P and tol);
       ...
       N_CrvInitArrays(&cur);
       N_FitArcToPts(P,n,tol,NL_MAXIMUM,NL_YES,NL_QUADRATIC,&cur,&cut,&era,&erm,&SG);


   ACCESS:
   
     P     , input  ,  Points in 2-D
     n     , input  ,  Highest index in P
     tol   , input  ,  Tolerance to check  collinearity; the  maximum or
                       the  average  error from  the least-squares  line 
                       must be less than "tol"
     etp   , input  ,  Flag:
                         NL_MAXIMUM: tol is maximum deviation
                         NL_AVERAGE: tol is average deviation
     arc   , input  ,  Flag:
                         NL_YES: find the best arc representing the points
                         NL_NO : output the full circle
     cit   , input  ,  Flag:
                         NL_QUADRATIC: degree  2  circle  is created  with
                                    double internal knots
                         NL_QUARTIC  : degree 4  circle is created, possi-
                                    ibly with  quadruple internal knots
                                    (but better  parameterization  than
                                    NL_QUADRATIC)
                         NL_QUINTIC  : degree 5  circle  is  created  with 
                                    no internal knots
     cur   , output ,  Best fitting NURBS circle
     cut   , output ,  Flag:
                         NL_NLINE  : a line is returned
                         NL_NCIRCLE: a circle/arc is returned
     era   , output ,  Average absolute error
     erm   , output ,  Maximum absolute error
     SG    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitArcToPts( NL_POINT *P, NL_INDEX n, NL_REAL tol, NL_FLAG etp, NL_FLAG arc, NL_FLAG cit, NL_CURVE *cur, NL_FLAG *cut, NL_REAL *era, NL_REAL *erm, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitArcToPts");

    NL_FLAG pli, lfi, cfi, cfl, error = NL_NO;

    NL_INDEX i, imax=0, imin=0, ig=0;

    NL_REAL *alf, lea, lem, cea = 0.0, cem = 0.0, cx = 0.0, cy = 0.0, rr = 0.0, tmax, tmin, t, as, ae, gap, dal;

    NL_POINT A, B, CC, DD;

    NL_VECTOR VV, XX, YY;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    if( n LT 1 )
        NL_ERROR( NL_INP_ERR );

    /* Fit a line first and check if points are collinear */

    pli = NL_NO;

    error = N_LineFitPts( P, n, &CC, &VV, &lea, &lem );

    if( error EQ NL_YES )
        NL_OUT;

    switch( etp )
    {
        case NL_MAXIMUM:
            if( lem LT tol )
                pli = NL_YES;
            break;

        case NL_AVERAGE:
            if( lea LT tol )
                pli = NL_YES;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* If the points are not collinear, attempt to fit a circle */

    lfi = cfi = NL_NO;

    if( pli EQ NL_NO )
    {
        error = N_CreateCircFrom2dPts( P, n, tol, etp, &cx, &cy, &rr, &cea, &cem, &cfl );

        if( error EQ NL_YES )
            NL_OUT;

        if( cfl EQ NL_TRUE )
        {
            if( lea LT cea )
                lfi = NL_YES;
            else
                cfi = NL_YES;
        }
        else
        {
            lfi = NL_YES;
        }
    }
    else
    {
        lfi = NL_YES;
    }

    /* Line fit was chosen either because the points are collinear, or */
    /* because the NL_AVERAGE error for the line is smaller than the one  */
    /* for the circle                                                  */

    if( lfi EQ NL_YES )
    {
        /* Get end points of line fit */

        N_VectorSum( CC, VV, &DD );

        tmin = NL_BIGD;
        tmax = -1.0;

        for ( i = 0; i <= n; i++ )
        {
            error = N_ProjectPtLineParam( P[i], CC, DD, &A, &t );

            if( error EQ NL_YES )
                NL_OUT;

            if( t LT tmin )
            {
                tmin = t;
                imin = i;
            }

            if( t GT tmax )
            {
                tmax = t;
                imax = i;
            }
        }

        error = N_ProjectPtLineParam( P[imin], CC, DD, &A, &t );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_ProjectPtLineParam( P[imax], CC, DD, &B, &t );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( B, A, &VV );

        error = N_CrvLineFromPtAndVector( A, VV, cur, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *cut = NL_NLINE;
        *era = lea;
        *erm = lem;

        NL_OUT;
    }

    /* Circle fit was chosen because the average error is smaller for */
    /* the circle than the one for the line                           */

    if( cfi EQ NL_YES )
    {
        N_VectorCreate( 1.0, 0.0, 0.0, &XX );
        N_VectorCreate( 0.0, 1.0, 0.0, &YY );
        N_VectorCreate( cx, cy, 0.0, &CC );

        if( arc EQ NL_NO )
        {
            as = 0.0;
            ae = 360.0;
        }
        else
        {
            alf = N_AllocReal1dArray( n + 1, &SL );

            if( alf EQ NULL )
                NL_QUIT;

            /* Get sweep angles */

            for ( i = 0; i <= n; i++ )
            {
                N_VectorDiff( P[i], CC, &VV );

                error = N_VectorDirectedAngle( XX, VV, &alf[i] );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* Sort the angles */

            N_ShellSortReal( alf, n );
            alf[n + 1] = 360.0 + alf[0];

            /* Find the largest gap */

            gap = -1.0;

            for ( i = 0; i <= n; i++ )
            {
                dal = fabs( alf[i + 1] - alf[i] );

                if( dal GT gap )
                {
                    gap = dal;
                    ig = i;
                }
            }

            as = alf[ig + 1];
            ae = alf[ig];

            if( as GT 360.0 )
                as -= 360.0;

            if( ae GT 360.0 )
                ae -= 360.0;
        }

        error = N_CreateCircArc( CC, XX, YY, rr, as, ae, cit, cur, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *cut = NL_NCIRCLE;
        *era = cea;
        *erm = cem;

        NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FitArcToEndPtsAndTangents: Compute a biarc to given end points and end tangents     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a biarc to given end points and end
     tangents. The biarc is NL_G1 continuous at the junction  point(s).
     The points and tangents must be 2d, i.e. in the xy plane with all 
     z coordinates set to 0.
      
     A typical calling example is:

       NL_POINT   Ps, Pe;
       NL_VECTOR  Ts, Te;
       NL_CPOINT  Pw[18];
       NL_INDEX   *n;
       ...
       (get Ps, Ts, and Pe, Te);
       ...
       N_FitArcToEndPtsAndTangents(Ps,Ts,Pe,Te,Pw,&n);

     IT IS  ASSUMED THAT  MEMORY FOR  PW IS  ALLOCATED  IN THE CALLING 
     ROUTINE TO  HOLD UP TO  Pw[17].

     This routine has problems when the tangents are parallel
     See preferred routine: N_FitBiArc
   ACCESS:
   
     Ps,Ts , input  ,  Start point and tangent
     Pe,Te , input  ,  End point and tangent
     Pw    , output ,  Control points of NURBS biarc
     n     , output ,  Highest index in Pw

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_FitArcToEndPtsAndTangents  /* eff: make CPts of a deg 2 piecewise bezier rational BSpline fitting pts and tangents */
                                     /*    : usually 2 segments, can be more. */ 
  (NL_POINT    Ps,                   /* in : start point                      */
   NL_VECTOR   Ts,                   /* in : start tangent                    */
   NL_POINT    Pe,                   /* in : end point                        */
   NL_VECTOR   Te,                   /* in : end tangent                      */
   NL_CPOINT  *Pw,                   /* out: ControlPoints of Nurbs biarc     */
   NL_INDEX   *n )                   /* out: highest index in Pw              */
{
    NL_FLAG error = NL_NO;

    NL_INDEX k, l;

    NL_REAL d, dis, dot, alf, vv, vts, vte, tste, a, b, c, disc;

    NL_POINT Pm, A, B, J, K;

    NL_CPOINT Qw[3];

    NL_VECTOR V;

    NL_FLAG bZeroTangent = NL_NO;

    /* Initialize and compute some parameters */

    /* define chord vector, V = Pe - Ps */
    N_VectorDiff( Pe,   /* a of c = a - b         */ 
                  Ps,   /* b of c = a - b         */ 
                  &V ); /* c = vector from b to a */
                   
    /* define chord mid point, Pm = (Pe+Ps)/2 */ 
    N_Combine2Pts( 0.5, Ps, 0.5, Pe, &Pm );

    /* normalize chord vector */ 
    error = N_VectorNormalize( V, &V, &d );

    if( error EQ NL_YES )
        NL_OUT;

    /* normalize start tangent */ 
    error = N_VectorNormalizeRef( &Ts );

    if( error EQ NL_YES )
      { bZeroTangent = NL_YES; }   /* NL_OUT; */

    /* normalize end tangent */ 
    error = N_VectorNormalizeRef( &Te );

    if( error EQ NL_YES )
      { bZeroTangent = NL_YES; }   /* NL_OUT; */

    if ( bZeroTangent == NL_YES )
    {
        /* The best solution for this case is the same as the anitparallel-tangent case,   */
        /* but instead of trying to direct execution down to that block, it's much easier  */
        /* just to copy that block up here.   [B541] */
        error = NL_NO;

        N_VectorPtAlongVector( Ps, 0.5 *d, Ts, &A );     /* A = half chord distance from Start along Start tangent */ 
        N_VectorPtAlongVector( Pm, 0.5 *d, Ts, &J );     /* J = half chord distance from Mid along Start tangent */
        N_VectorPtAlongVector( Pe, -0.5 *d, Te, &B );    /* B = half chord distance back from end along end tangent */
        /* define 3 cpts for circular arc from Ps to J via A */
        ST_CalcCPtsCircle( Ps,    /* in : start control point position - gets weight = 1.0 */                                        
        A,    /* in : mid control point position   - gets weight = 1/2 * (dist(P0,P1)+dist(P1,P2))/dist(P0,P2) */
        J,    /* in : end point point position     - gets weight = 1.0 */                                        
        Qw ); /* out: control points Pw[0], Pw[1], Pw[2] */                                                      
        Pw[0] = Qw[0];
        Pw[1] = Qw[1];
        Pw[2] = Qw[2];

        /* define 3 cpts for circular arc from J to Pe via B */
        ST_CalcCPtsCircle( J, B, Pe, Qw );
        Pw[3] = Qw[1];
        Pw[4] = Qw[2];

        *n = 4;

        NL_OUT;
    }

    /* Check for parallel tangents - don't have to be parallel to the chord vector */
    N_VectorDot( Ts, Te, &tste );
    N_VectorCross( Ts, Te, &A ); /* gwc: this presumes Ts and Te are in the XY plane */ 
    N_PtToXYZ( A, &a, &b, &c );

    /* when Start and End tangents are parallel */ 
    if( fabs( c ) LT NL_LTOL OR fabs( tste - 1.0 ) LT NL_LTOL ) /* NL_LTOL = parallel line tolerance */ 
    {
        /* End tangents are parallel */

        /* check for parallel Start Tangent and chord vector */ 
        N_VectorCross( Ts, V, &A );
        N_PtToXYZ( A, &a, &b, &c );

        /* When StartTangent, EndTangent, and chordVec are all parallel */ 
        if( fabs( c ) LT NL_LTOL )
        {
           /* A = point half a chord length along the start vector */ 
                                             /* d = chord segment length */
            N_VectorPtAlongVector( Ps,       /* in : P     of  Q = P + alpha * V */
                                   0.5 *d,   /* in : alpha of  Q = P + alpha * V */
                                   Ts,       /* in : V     of  Q = P + alpha * V */
                                   &A );     /* out: Q     of  Q = P + alpha * V */

            /* B = point half a chord length along the end vector */ 
            N_VectorPtAlongVector( Pe, -0.5 *d, Te, &B );
            N_DistPtPt( A, B, &dis );

            /* when startTangent points to endPoint and End Tangent points the same way */
            /* gwc: I think the case where startTangent points away from endPoint and EndTangent points the same way was ignored */  
			/* gwc: make tolerance check proportional to size of span between end points: bug 569 */
			if (dis LT((1.0 + dis) * NL_MTOL))
              {
                /* End tangents are collinear */
                /* output a set of 3 colinear control points */ 

                N_Weight( Ps, 1.0, &Pw[0] );
                N_Weight( Pm, 1.0, &Pw[1] );
                N_Weight( Pe, 1.0, &Pw[2] );

                *n = 2; /* hihgest control point index */ 

                NL_OUT;
              }
            else /* for all other tangent direction combinations */ 
              {
                /* check Ts and Te for same/opposite V direction */

                N_VectorDot( V, Ts, &vts );  /* vts: pos = startTangent towards end, neg = away from end */ 
                N_VectorDot( V, Te, &vte );  /* vte: neg = endTangent towards start, pos = away from start */ 

                                             /* V = chord dir */ 
                N_VectorPerpendicular( V,    /* in : 2d vector (z == 0.0)              */
                                      &A );  /* out: CCW 90 degree rotation (z == 0.0) */
                a = 1.0;

                /* when tangents point away from one another */ 
                if( vts LT 0.0 AND vte GT 0.0 )
                  {
                    /* switch A to a CW rotation */ 
                    N_VectorReverse( A, &A );
                    a = -1.0;
                  }

                /* define point J half a chord length off the chord line from mid point */
                /* points Ps, J, and Pe are now all an equal 1/2 chord distance from point Pm */ 
                N_VectorPtAlongVector( Pm, a * 0.5 *d, A, &J );


                /* gwc: how odd, an arc from Ps to J and another from J to Pe but
                        tangents into a kind of S shape - is that's what's desired? */ 
                error = N_FitArcToEndPtsAndTangents( Ps, Ts, J, A, Pw, &k );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_FitArcToEndPtsAndTangents( J, A, Pe, Te, &Pw[k], &l );

                if( error EQ NL_YES )
                    NL_OUT;

                *n = k + l;

                NL_OUT;
              }
          } /* end start and end parallel tangents are parallel to chord vec branch */ 
        else /* start and end parallel tangents are not parallel to chord vec branch */ 
          {
            /* End tangents are not collinear */

            /* dot: pos=in same direction, neg=in opposite directions */ 
            N_VectorDot( Ts, Te, &dot );

            /* when end tangents are in opposite directions */ 
            if( dot LT 0.0 )
              {
                /* Tangents are antiparallel */                  /*  A,J,B are equally spaced colinear */
                N_VectorPtAlongVector( Ps, 0.5 *d, Ts, &A );     /* A = half chord distance from Start along Start tangent */ 
                N_VectorPtAlongVector( Pm, 0.5 *d, Ts, &J );     /* J = half chord distance from Mid along Start tangent */
                N_VectorPtAlongVector( Pe, -0.5 *d, Te, &B );    /* B = half chord distance back from end along end tangent */
                
                /* define 3 cpts for circular arc from Ps to J via A */
                ST_CalcCPtsCircle( Ps,    /* in : start control point position - gets weight = 1.0 */                                        
                                    A,    /* in : mid control point position   - gets weight = 1/2 * (dist(P0,P1)+dist(P1,P2))/dist(P0,P2) */
                                    J,    /* in : end point point position     - gets weight = 1.0 */                                        
                                    Qw ); /* out: control points Pw[0], Pw[1], Pw[2] */                                                      
                Pw[0] = Qw[0];
                Pw[1] = Qw[1];
                Pw[2] = Qw[2];

                /* define 3 cpts for circular arc from J to Pe via B */
                ST_CalcCPtsCircle( J, B, Pe, Qw );
                Pw[3] = Qw[1];
                Pw[4] = Qw[2];

                *n = 4;

                NL_OUT;
              }
            else /* end tangents are in same direction */ 
              {
                /* Tangents are parallel  fill shape with an 'S' shape */

                N_Combine2Pts        ( 0.5, Ps, 0.5, Pm, &K ); /* K = point on chord mid way between start and midpoint */ 
                N_VectorPtAlongVector( Ps, 0.25 *d, Ts, &A );  /* A = quarter chord from start along start vector */ 
                N_VectorPtAlongVector( K, 0.25 *d, Ts, &J );   /* J = quarter chord from K along start vector */ 
                N_VectorPtAlongVector( Pm, 0.25 *d, Ts, &B );  /* B = quarter chord from mid point along start  */ 

                /* define 3 cpts for circular arc from Ps to J via A */
                ST_CalcCPtsCircle( Ps, A, J, Qw );   /* GWC: A,J,B are equally spaced colinear */ 
                Pw[0] = Qw[0];
                Pw[1] = Qw[1];
                Pw[2] = Qw[2];
                
                /* define 3 cpts for circular arc from J to Pm via B */
                ST_CalcCPtsCircle( J, B, Pm, Qw );
                Pw[3] = Qw[1];
                Pw[4] = Qw[2];

                N_Combine2Pts        ( 0.5, Pm, 0.5, Pe, &K );  /* K = point on chord mid way between end and midpoint */
                N_VectorPtAlongVector( Pm, -0.25 *d, Te, &A );  /* A = quarter chord from midPoint back end vector */      
                N_VectorPtAlongVector( K, -0.25 *d, Te, &J );   /* J = quarter chord from K back end vector */          
                N_VectorPtAlongVector( Pe, -0.25 *d, Te, &B );  /* B = quarter chord from EndPoint back end vector  */        

                /* define 3 cpts for circular arc from Pm to J via A */
                ST_CalcCPtsCircle( Pm, A, J, Qw );      /* GWC: oldB,Pm,A are equally spaced colinear */ 
                Pw[5] = Qw[1];                          /* GWC: A,J,B are equally spaced colinear */ 
                Pw[6] = Qw[2];
                
                /* define 3 cpts for circular arc from J to Pe via A */
                ST_CalcCPtsCircle( J, B, Pe, Qw );
                Pw[7] = Qw[1];
                Pw[8] = Qw[2];

                *n = 8;

                NL_OUT;
              } /* end tangents are in same direction */ 
          } /* end start and end parallel tangents are not parallel to chord vec branch */ 
      } /* end when end tangents are parallel check */ 

    /* else start and end tangents are not parallel */

    N_VectorSum( Ts, Te, &A );  /* A = Ts + Te */ 
    N_VectorDot( A, V, &dot );  /* V = chord unit-vector */ 

    /* When tangent sum is perp to chord vector */ 
    if( fabs( dot )LT NL_LTOL )
      {
        /* split the interval into two sub intervals */ 
        N_VectorReverse( A, &A );

        error = N_FitArcToEndPtsAndTangents( Ps, Ts, Pm, A, Pw, &k );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitArcToEndPtsAndTangents( Pm, A, Pe, Te, &Pw[k], &l );

        if( error EQ NL_YES )
            NL_OUT;

        *n = k + l;

        NL_OUT;
      } /* end tangent sum is perp to chord vector check */

    /* next: compute control points to define two bezier deg 2 same-radius circular-arc segments that connect         */
    /*       C1 to one another and match the given position and tangent values at the start and end points            */
    /*                                                                                                                */
    /*       To do that we need to define 5 control points:                                                           */
    /*         Start = given start point                                                                              */
    /*         A     = Ps + alf * Ts, CPnt on start tangent line setting bi arc start tangent = input start tangent   */
    /*         J     = (A + B) / 2.0, CPnt at join between the two biarcs, colinear and evenly spaced between A and B */
    /*         B     = Pe - alf * Te, CPnt on end tangent line setting bi arc end tangent = input end tangent         */
    /*         End   = given end point                                                                                */
    /*                                                                                                                */
    /*        To make C1 connecting circular biarcs of the same radius (using CPnts [Start, A, J] and [J, B, End]),   */
    /*           1. Control points A, J, and B have to be colinear                                                    */
    /*           2. The spacing between all 5 control points (Start, A, J, B, End) has to be constant.                */
    /*        So, given above control point definitions, find CPnts A, J, and B by solving for alf in                 */
    /*                                                                                                                */
    /*           (A-B).(A-B) = 4*alf*alf                                                                              */
    /*              with v     = Pe - Ps,                                                                             */
    /*                   Ts.Ts = 1,                                                                                   */
    /*                   Te.Te = 1                                                                                    */
    /*                                                                                                                */
    /*           (v + (Te+Ts)*alf).(v + (Te+Ts)*alf) = 4*alf*alf                                                      */
    /*                                                                                                                */
    /*           0 =   2 * (Te.Ts - 1)   * alf*alf                                                                    */
    /*               + 2 * (v.Te + v.Ts) * alf                                                                        */
    /*               + v.v                                                                                            */

    N_VectorDiff( Ps, Pe, &V );  /* define chord vector, V = Pe - Ps */
    N_VectorDot( V, V, &vv );
    N_VectorDot( V, Ts, &vts );  
    N_VectorDot( V, Te, &vte );   

    a = 2.0 *( tste - 1.0 );   
    b = 2.0 *( vts + vte );    
    c = vv;                    

    /* gwc: Solve for alf in 0 =   2.0 *( tste - 1.0 ) * alf**2  (with te and ts being unit vectors) */
    /*                           + 2.0 *( vts + vte )  * alf     */  
    /*                           + vv                            */ 

    disc = b * b - 4.0 *a * c;   /* this is positive definite = VSq * (TeSq + TsSq + 2) [Te and Ts are unit vecs] */ 
    disc = sqrt( disc );
    alf = (-b - disc) / (2.0 *a);

    /* define points A and B offset from Ps and Pe by alf*endTangent */ 
    N_VectorPtAlongVector( Ps,  alf, Ts, &A );
    N_VectorPtAlongVector( Pe, -alf, Te, &B );
    N_Combine2Pts( 0.5, A, 0.5, B, &J );         /* make points A,J,B equally spaced colinear */ 

    /* reuse a,b,c to store distances from points A,B,J to line Ps,Pe */ 
    error = N_DistPtLineSeg( A,    /* in : point                                */
                             Ps,   /* in : start line segment                   */
                             Pe,   /* in : end line segment                     */
                             &a ); /* out: min dist between point P and line AB */

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtLineSeg( B, Ps, Pe, &b );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtLineSeg( J, Ps, Pe, &c );

    if( error EQ NL_YES )
        NL_OUT;
    
    /* when mid-points are within tol of being colinear of line Ps,Pe */ 
    if( a LT NL_MTOL AND b LT NL_MTOL AND c LT NL_MTOL )
    {
        /* output a line segment */ 
        N_Weight( Ps, 1.0, &Pw[0] );
        N_Weight( Pm, 1.0, &Pw[1] );
        N_Weight( Pe, 1.0, &Pw[2] );

        *n = 2;

        NL_OUT;
    }

    /* define 3 cpts for circular arc from Ps to J via A */ 
    ST_CalcCPtsCircle( Ps, A, J, Qw );
    Pw[0] = Qw[0];
    Pw[1] = Qw[1];
    Pw[2] = Qw[2];

    /* define 3 cpts from circular arc from J to Pe via B */ 
    ST_CalcCPtsCircle( J, B, Pe, Qw );
    Pw[3] = Qw[1];
    Pw[4] = Qw[2];

    *n = 4;

    /* End NURBS and Exit */

    EXIT:

    return (error);

} /* end N_FitArcToEndPtsAndTangents */

#if NLIB_UNUSED

/**********************************************************************/
/* N_FitBiArc: Compute a biarc to given end points and end tangents   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a biarc to given end points and end
     tangents. The end point, Pe, must not be behind the start point, Ps, 
     ie, (Pe - Ps) dot Ts >= 0, and the start point must not be in front of
     the end point, ie(Pe - Ps) dot Te >= 0. The biarc is NL_G1 continuous at  
     the junction  point(s).
     A typical calling example is:

       NL_POINT   Ps, Pe;
       NL_VECTOR  Ts, Te;
       NL_CPOINT  Pw[5];
       NL_INDEX   *n;
       ...
       (get Ps, Ts, and Pe, Te);
       ...
       N_FitBiArc(Ps, Ts, Pe, Te, Pw, &n);

     IT IS  ASSUMED THAT  MEMORY FOR  PW IS  ALLOCATED  IN THE CALLING 
     ROUTINE TO  HOLD UP TO  Pw[4].


   ACCESS:
   
     Ps, Ts , input  ,  Start point and tangent with(Pe - Ps) dot Ts >= 0
     Pe, Te , input  ,  End point and tangent with(Pe - Ps) dot Te >= 0
     Pw     , output ,  Control points of NURBS biarc
     n      , output ,  Highest index in Pw


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitBiArc( NL_POINT Ps, NL_VECTOR Ts, NL_POINT Pe, NL_VECTOR Te, NL_CPOINT *Pw, NL_INDEX *n )
{
    NL_FLAG error = NL_NO;

    NL_REAL d, alf, vv, vts, vte, tste, a, b, c, disc;

    NL_POINT Pm, A, B, J;

    NL_VECTOR V;

    NL_CPOINT Qw[3];

    /* Initialize and compute some parameters */

    N_VectorDiff( Pe, Ps, &V );
    N_Combine2Pts( 0.5, Ps, 0.5, Pe, &Pm );

    error = N_VectorNormalize( V, &V, &d );

    if( d < NL_MTOL )
        error = NL_YES;

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDot( V, Ts, &vts ); /* Pe must be in front of Ps, Ts */

    if( vts < -NL_MTOL )
    {
        error = NL_YES;
        NL_OUT;
    }

    N_VectorDot( V, Te, &vte ); /* Ps must be behind Pe, Te */

    if( vte < -NL_MTOL )
    {
        error = NL_YES;
        NL_OUT;
    }

    /* Given P0, P1 and P2 if |P1 - P0| = |P2 - P1| then by         */
    /* setting the appropriate weight, w1, then P0, w1*P1, P2 can   */
    /* define a Bezier circular arc from P0 to P2                   */
    /* Hence given Ps, Ts and Pe, Te with |Ts| = |Te| = 1           */
    /* Let A = Ps + alf*Ts and B = Pe - alf*Te  for alf > 0         */
    /* so that B - A = Pe - Ps - alf*Te - alf*Ts                    */
    /* Then if alf is such that |B - A| = 2*alf                     */
    /* we can construct two circular arcs from Ps, Ts to Pe, Te     */
    /* (B - A) o (B - A) = 4*alf*alf                                */
    /* Let V = Pe - Ps  and                                         */
    /*    vv   = V o V                                              */
    /*    vts  = V o Ts      >= 0                                   */
    /*    vte  = V o Te      >= 0                                   */
    /*    tste = Ts o Te     <= 1                                   */
    /* then  2*alf*alf = vv - 2*alf*(vts+vte) + 2*alf*alf*tste      */
    /*    alf*alf*(1 - tste) + 2*alf*(vts+vte) - vv = 0             */
    /* seting a = 2*(1 - tste), b = 2*(vts + vte), c = -vv          */
    /*    alf = (-b + sqrt(b*b - 4*a*c))/2*a                        */
    /* note alf > 0 since sqrt(b*b - 4*a*c) > b                     */

    N_VectorDiff( Pe, Ps, &V ); /* V = Pe - Ps  */
    N_VectorDot( V, V, &vv );
    N_VectorDot( V, Ts, &vts );
    N_VectorDot( V, Te, &vte );
    N_VectorDot( Ts, Te, &tste );

    a = 2.0 *( 1.0 - tste ); /* Ts dot Te <= 1 hence a >= 0   */
    b = 2.0 *( vts + vte );  /* V dot Ts > 0 and V dot Te > 0 hence b > 0 */
    c = -vv;                 /* c < 0    */

    if( tste > (1.0 - NL_LTOL) )
    {
        alf = vv / b;
    }
    else                           /* a < 0  */
    {
        disc = b * b - 4.0 *a * c; /* a > 0 and c < 0 hence disc > 0  */
        disc = sqrt( disc );
        alf = (-b + disc) / (2.0 *a);

        if( alf < NL_MTOL )
        {
            error = NL_YES;
            NL_OUT;
        }
    }

    N_VectorPtAlongVector( Ps, alf, Ts, &A );
    N_VectorPtAlongVector( Pe, -alf, Te, &B );
    N_Combine2Pts( 0.5, A, 0.5, B, &J );

    /* check for a line from Ps,Ts to Pe,Te */
    error = N_DistPtLineSeg( A, Ps, Pe, &a );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtLineSeg( B, Ps, Pe, &b );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtLineSeg( J, Ps, Pe, &c );

    if( error EQ NL_YES )
        NL_OUT;

    if( a LT NL_MTOL AND b LT NL_MTOL AND c LT NL_MTOL )
    { /* A, B and J are all on the line from Ps to Pe */
        N_Weight( Ps, 1.0, &Pw[0] );
        N_Weight( Pm, 1.0, &Pw[1] );
        N_Weight( Pe, 1.0, &Pw[2] );
        *n = 2;
        NL_OUT;
    }

    ST_CalcCPtsCircle( Ps, A, J, Qw ); /* construct the CPOINTs for the circle defined by Ps, A, J */
    Pw[0] = Qw[0];
    Pw[1] = Qw[1];
    Pw[2] = Qw[2];
    ST_CalcCPtsCircle( J, B, Pe, Qw ); /* construct the CPOINTs for the circle defined by J, B, Pe */
    Pw[3] = Qw[1];
    Pw[4] = Qw[2];

    *n = 4;

    /* End NURBS and Exit */
    EXIT:

    return (error);
} /* end N_FitBiArc */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FitRemoveKnotsAndDerivs: Remove all removable knots with derivative constraints   */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This  fitting  routine  removes all  removable knots  from a NURBS 
     curve approximating, at supplied parameter values, a  given set of  
     points. An  error vector (displacement size at given set of points)
     is updated as knots are removed in place, 
     i.e. the  original  curve is  destroyed.  This  version  maintains 
     derivatives at  either  end or  both  ends of the curve. A typical 
     calling example is:

       NL_CURVE  cur;
       NL_INDEX  mu, der;
       NL_REAL   *u, *er, E;
       ...
       (define cur and der, get arrays u and er);
       ...
       N_FitRemoveKnotsAndDerivs(&cur,u,er,mu,E,NL_BOTH,der);

     This  routine is used in N_FITCRVAPPROXKNOTSANDTANGENTSTOL to  clean approximating  curves.

   ACCESS:
   
     cur , in/out ,  NURBS curve
     u   , input  ,  Parameter values at which error is to be checked
     er  , in/out ,  Error vector to be updated
     mu  , input  ,  Highest index in u and er
     E   , input  ,  Error tolerance (max dist modified curve allowed to move from original pos) 
     whr , input  ,  Flag:
                       NL_NO   : don't constrain end derivatives
                       NL_START: constrain at the start point
                       NL_END  : constrain at the end point
                       NL_BOTH : constrain at both ends
     der , input  ,  Highest  derivative  constraint, i.e. the  routine
                     maintains the  1-st to the  der-th  derivatives at 
                     the specified end point(s)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitRemoveKnotsAndDerivs( NL_CURVE *cur, NL_REAL *u, NL_REAL *er, NL_INDEX mu, NL_REAL E, NL_FLAG whr, NL_INDEX der )
{
    NL_FLAG rmf, error = NL_NO;

    NL_INDEX *left, *right, *sr, i, j, k, l, ii, jj, first, last, off, fout, n, m, r, s;

    NL_DEGREE p;

    NL_REAL *U, *br, *te, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, N, N1;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Rw;

    NL_STACKS SL;

    NL_REAL NOREM = 1.0e+25;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur, &knt );

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

    te = N_AllocReal1dArray( mu, &SL );

    if( te EQ NULL )
        NL_QUIT;

    left = N_AllocInt1dArray( n, &SL );

    if( left EQ NULL )
        NL_QUIT;

    right = N_AllocInt1dArray( n, &SL );

    if( right EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( i = 0; i <= m; i++ )
    {
        br[i] = NL_BIGD;
        sr[i] = 0;
    }

    /* Compute knot removal errors for each distinct knot */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;
        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( cur, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Get ranges of parameter indexes */

    j = 1;

    for ( i = 0; i <= n; i++ )
    {
        left[i] = j;

        while( j LT mu AND u[j]GT U[i]AND u[j]LE U[i + 1] )
            j++;

        for ( k = j; k < mu; k++ )
        {
            if( u[k]GE U[i + p + 1] )
                break;
        }
        right[i] = k - 1;
    }

    /* Try to remove knots until cumulative error < E */

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
            k = (p + s + 1) / 2;
            i = r - k;
            alf = (U[r] - U[i]) / (U[i + p + 1] - U[i]);
            bet = (U[r] - U[i + 1]) / (U[i + p + 2] - U[i + 1]);
            omb = 1.0 - bet;
            lam = alf / (alf + bet);
            oml = 1.0 - lam;

            for ( j = left[i]; j <= right[i + 1]; j++ )
            {
                error = N_BasisIEval( knt, i, p, u[j], NL_LEFT, &N );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_BasisIEval( knt, i + 1, p, u[j], NL_LEFT, &N1 );

                if( error EQ NL_YES )
                    NL_OUT;

                te[j] = er[j] + fabs( lam * alf * N - oml * omb * N1 ) * b;

                if( te[j]GT E )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }
        else
        {
            k = (p + s) / 2;
            i = r - k;

            for ( j = left[i]; j <= right[i]; j++ )
            {
                error = N_BasisIEval( knt, i, p, u[j], NL_LEFT, &N );

                if( error EQ NL_YES )
                    NL_OUT;

                te[j] = er[j] + N * b;

                if( te[j]GT E )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }

        /* If knot is removable, update error and remove knot */

        if( rmf EQ NL_TRUE )
        {
            if( (p + s) % 2 )
                l = right[i + 1];
            else
                l = right[i];

            for ( j = left[i]; j <= l; j++ )
                er[j] = te[j];

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;
            i = first;
            j = last;
            ii = 1;
            jj = last - off;

            N_CopyCPt( Pw[off], &Rw[0] );
            N_CopyCPt( Pw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                oma = 1.0 - alf;
                bet = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Pw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Pw[j], omb, Rw[jj + 1], &Rw[jj] );
                i++;
                j--;
                ii++;
                jj--;
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
                N_CopyCPt( Rw[i - off], &Pw[i] );
                N_CopyCPt( Rw[j - off], &Pw[j] );
                i++;
                j--;
            }

            /* Shift down some parameters */

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( i = r + 1; i <= m; i++ )
            {
                br[i - 1] = br[i];
                sr[i - 1] = sr[i];
            }

            /* Shift down knots and control points */

            for ( i = r + 1; i <= m; i++ )
                U[i - 1] = U[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Pw[i], &Pw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( cur, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( U[i]NEQ U[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( cur, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            /* Update index ranges */

            for ( i = r - p - 1; i <= r - s; i++ )
            {
                for ( k = right[i] + 1; k <= mu; k++ )
                {
                    if( u[k]GE U[i + p + 1] )
                        break;
                }
                right[i] = k - 1;
            }

            for ( i = r - s + 1; i <= n; i++ )
            {
                left[i] = left[i + 1];
                right[i] = right[i + 1];
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCALCCRVPARAMVALUES: Parametrization for global curve interpolation           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes  parameter values for global curve 
     interpolation.  The  data  points are  assumed at  those parameter 
     values. In order to facilitate with both NL_POINT and CPOINT objects,
     the routine employes a VOID pointer as input parameter and assumes
     that the calling routine typecasts the given pointer to the appro-
     priate type. The calling mechanism works as follows:

       CPOINT     *Pw;
       NL_POINT   *P;
       NL_INDEX       n;
       NL_PARAMETER  *u;
       ...
       (define control point/point arrays and allocate memory for u);
       ...
       N_FitCalcCrvParamValues((VOID *)P ,n,EPOINT,CHORDLENGTH,u);
       N_FitCalcCrvParamValues((VOID *)Pw,n,HPOINT,CENTRIPETAL,u);

     It is assumed that memory to store the  parameters is allocated in
     the calling routine. A recommended default for the parametrization 
     type is NL_CHORDLENGTH.


   ACCESS:
   
     A   , input  ,  VOID pointer representing either NL_POINT or CPOINT
     n   , input  ,  Highest index in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     u   , output ,  Parameters corresponding to data points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcCrvParamValues( NL_VOID *A, NL_INDEX n, NL_FLAG ptp, NL_FLAG par, NL_PARAMETER *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcCrvParamValues");

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL *dist, sum, fact;

    NL_POINT *P;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute uniform parametrization */

    u[0] = 0.0;
    u[n] = 1.0;

    if( par EQ NL_UNIFORM )
    {
        fact = 1.0 / n;

        for ( i = 1; i < n; i++ )
            u[i] = i * fact;

        NL_OUT;
    }

    /* Compute chord length or centripetal parametrization */

    dist = N_AllocReal1dArray( n, &SL );

    if( dist EQ NULL )
        NL_QUIT;

    sum = 0.0;

    switch( ptp )
    {
        case NL_EPOINT:

            P = (NL_POINT *)A;

            switch( par )
            {
                case NL_CHORDLENGTH:
                    for ( i = 1; i <= n; i++ )
                    {
                        N_DistPtPt( P[i - 1], P[i], &dist[i] );
                        /*  suggested change to allow for non-uniform param 'speed' at ends         
                        if (n > 8) 
                        {
                        if  (i == 1 || i == n) dist[i] = dist[i]*0.65;
                        if  (i == 2 || i == n-1) dist[i] = dist[i]*0.8;
                        if  (i == 3 || i == n-2) dist[i] = dist[i]*0.9;
                        }
                        */
                        sum += dist[i];
                    }
                    break;

                case NL_CENTRIPETAL:
                    for ( i = 1; i <= n; i++ )
                    {
                        N_DistPtPt( P[i - 1], P[i], &dist[i] );
                        /*  suggested change to allow for non-uniform param 'speed' at ends         
                        if (n > 8) 
                        {
                        if  (i == 1 || i == n) dist[i] = dist[i]*0.65;
                        if  (i == 2 || i == n-1) dist[i] = dist[i]*0.8;
                        if  (i == 3 || i == n-2) dist[i] = dist[i]*0.9;
                        }
                        */

                        dist[i] = sqrt( dist[i] );
                        sum += dist[i];
                    }
                    break;

                default:

                    NL_ERROR( NL_CAL_ERR );
            }
            break;

        case NL_HPOINT:

            Pw = (NL_CPOINT *)A;

            switch( par )
            {
                case NL_CHORDLENGTH:
                    for ( i = 1; i <= n; i++ )
                    {
                        N_DistCptCptHomo( Pw[i - 1], Pw[i], &dist[i] );
                        sum += dist[i];
                    }
                    break;

                case NL_CENTRIPETAL:
                    for ( i = 1; i <= n; i++ )
                    {
                        N_DistCptCptHomo( Pw[i - 1], Pw[i], &dist[i] );
                        dist[i] = sqrt( dist[i] );
                        sum += dist[i];
                    }
                    break;

                default:

                    NL_ERROR( NL_CAL_ERR );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    if( sum > NL_PTOL )
        for ( i = 1; i < n; i++ )
            u[i] = u[i - 1] + dist[i] / sum;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/*********************************************************************************/
/* N_FitCalcFuncParamValues: Parametrization for global functional interpolation */
/*********************************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes parameter values for global function 
     interpolation.  The  data  values are  assumed at  those parameter 
     values. A typical calling example is:

       NL_REAL       *f;
       NL_INDEX      n;
       NL_PARAMETER  *u;
       ...
       (get f and allocate memory for u);
       ...
       N_FitCalcFuncParamValues(f,n,CHORDLENGTH,u);

     It is assumed that memory to store the  parameters is allocated in
     the calling routine. A recommended default for the parametrization 
     type is NL_CHORDLENGTH.


   ACCESS:
   
     f   , input  ,  Data values
     n   , input  ,  Highest index in f
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     u   , output ,  Parameters corresponding to data values


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_FitCalcFuncParamValues( NL_REAL *f, NL_INDEX n, NL_FLAG par, NL_PARAMETER *u )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL *dist, sum, fact;

    NL_STACKS SL;

    NL_PRIVATE NL_STRING rname = _T("N_FitCalcFuncParamValues");

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute uniform parametrization */

    u[0] = 0.0;
    u[n] = 1.0;

    if( par EQ NL_UNIFORM )
    {
        fact = 1.0 / n;

        for ( i = 1; i < n; i++ )
            u[i] = i * fact;

        NL_OUT;
    }

    /* Compute chord length or centripetal parametrization */

    dist = N_AllocReal1dArray( n, &SL );

    if( dist EQ NULL )
        NL_QUIT;

    sum = 0.0;

    switch( par )
    {
        case NL_CHORDLENGTH:
            for ( i = 1; i <= n; i++ )
            {
                dist[i] = fabs( f[i - 1] - f[i] );
                sum += dist[i];
            }
            break;

        case NL_CENTRIPETAL:
            for ( i = 1; i <= n; i++ )
            {
                dist[i] = sqrt( fabs( f[i - 1] - f[i] ) );
                sum += dist[i];
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 1; i < n; i++ )
        u[i] = u[i - 1] + dist[i] / sum;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITFUNCINTP: Global function interpolation with arbitrary degree      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a  NURBS function interpolating a 
     given set of values. The degree  is arbitrary and  the type  of 
     parametrization  can  be  chosen. If  the  output  function  is  
     initialized to NULL, memory  is allocated locally. Otherwise it 
     is  checked if  enough memory  is passed in. A  typical calling 
     example is:

       NL_REAL   *f;
       NL_INDEX   n;
       NL_DEGREE  p;
       NL_CFUN    cfn;
       NL_STACKS  SG;
       ...
       (define array f and choose degree);
       ...
       N_CFuncInitArrays(&cfn);
       N_FitFuncInterp(f,n,p,NL_CHORDLENGTH,&cfn,&SG);

     A  recommended   default   for  the   parametrization  type  is 
     NL_CHORDLENGTH.


   ACCESS:
   
     f   , input  ,  Values to be interpolated
     n   , input  ,  Highest index in f
     p   , input  ,  Degree of interpolating function
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     cfn , output ,  Interpolating function
     SG  , input  ,  cfn's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_FitFuncInterp( NL_REAL *f, NL_INDEX n, NL_DEGREE p, NL_FLAG par, NL_CFUN *cfn, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitFuncInterp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, bw, sbw;

    NL_REAL ** A, *N, *fu;

    NL_PARAMETER *u;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for function memory */

    if( p GT n )
        NL_ERROR( NL_INP_ERR );

    error = N_CFuncSizeArrays( cfn, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnotVector( cfn, &fu, &knt, &N );

    /* Get parameters and compute knot vetor */

    u = N_AllocReal1dArray( n, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcFuncParamValues( f, n, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    N_FitCrvCalcKnotVector( u, n, p, knt );

    /* Linear splines */

    if( p EQ 1 )
    {
        for ( i = 0; i <= n; i++ )
            fu[i] = f[i];

        NL_OUT;
    }

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[n][sbw] = 1.0;

    for ( i = 1; i < n; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - i - 1;

        for ( k = 0; k <= p; k++ )
            A[i][l + k] = N[k];
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    error = N_RealMatrixRightForBack( &cm, f, fu );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}


/**********************************************************************/
/* N_FITFUNCINTPGIVENPARAMS: Function interpolation with given parameters             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a  NURBS function interpolating a 
     given set of values. The degree  is arbitrary and  the type  of 
     parametrization  can  be  chosen. If  the  output  function  is  
     initialized to NULL, memory  is allocated locally. Otherwise it 
     is  checked if  enough memory  is passed in. A  typical calling 
     example is:

       NL_REAL       *f;
       NL_PARAMETER  *u;
       NL_INDEX      n;
       NL_DEGREE     p;
       NL_CFUN       cfn;
       NL_STACKS     SG;
       ...
       (define arrays f and u; choose degree);
       ...
       N_CFuncInitArrays(&cfn);
       N_FitFuncInterpGivenParams(f,n,p,u,&cfn,&SG);

     This version requires the input of parameters where the function
     values are assumed.


   ACCESS:
   
     f   , input  ,  Values to be interpolated
     n   , input  ,  Highest index in f
     p   , input  ,  Degree of interpolating function
     u   , input  ,  Parameters where function values are assumed
     cfn , output ,  Interpolating function
     SG  , input  ,  cfn's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitFuncInterpGivenParams( NL_REAL *f, NL_INDEX n, NL_DEGREE p, NL_PARAMETER *u, NL_CFUN *cfn, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitFuncInterpGivenParams");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, bw, sbw;

    NL_REAL ** A, *N, *fu;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for function memory */

    if( p GT n )
        NL_ERROR( NL_INP_ERR );

    error = N_CFuncSizeArrays( cfn, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnotVector( cfn, &fu, &knt, &N );

    /* Compute knot vetor */

    N_FitCrvCalcKnotVector( u, n, p, knt );

    /* Linear splines */

    if( p EQ 1 )
    {
        for ( i = 0; i <= n; i++ )
            fu[i] = f[i];

        NL_OUT;
    }

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[n][sbw] = 1.0;

    for ( i = 1; i < n; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - i - 1;

        for ( k = 0; k <= p; k++ )
            A[i][l + k] = N[k];
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    error = N_RealMatrixRightForBack( &cm, f, fu );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCRVCALCKNOTVECTOR: Compute knot vector for global curve interpolation       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes the knot vector for global curve  
     interpolation. The output knot vector has p+1 multiple knots
     equal to u[0] and u[1] at its ends and a sequence of single knots
     in the interior.  The output interior knots, U[i] are computed 
     as averages of p parameter values starting at u[i] as
        U[i] = Avg(u[i],u[i+1]...u[i+p-1]).

     A typical calling example is:

       NL_REAL        *u;
       NL_INDEX       n;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose p and define knt);
       ...
       N_FitCrvCalcKnotVector(u,n,p,&knt);

     IT IS ASSUMED THAT MEMORY FOR knt IS  ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     u   , input  ,  Parameters
     n   , input  ,  Highest index in u
     p   , input  ,  Degree of interpolating curve
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FitCrvCalcKnotVector
  (NL_REAL       *u,     /* in : parameter array, sized:[n+1]  */
   NL_INDEX       n,     /* in : highest index in U            */
   NL_DEGREE      p,     /* in : degree of interpolating curve */
   NL_KNOTVECTOR *knt )  /* out: knot vector - multiple end knots and interior single knots */
{

    NL_INDEX i, j;

    NL_REAL *U, sum;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute knot vector */

    N_KnotVectorGetKnots( knt, &i, &U );

    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[n + i + 1] = u[n];
    }

    for ( i = 1; i <= n - p; i++ )
    {
        sum = 0.0;

        for ( j = i; j <= i + p - 1; j++ )
            sum += u[j];
        U[i + p] = sum / p;
    }

    N_SetKnotIndex( knt, n + p + 1 );

    /* End NURBS */

    N_EndNurbs( &SL );
}

/**********************************************************************/
/* N_FITCURVEINTP: Global curve interpolation with arbitrary degree         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of points. The degree is arbitrary and the type  of parametri-
     zation can be  chosen. If the  output curve is  initialized to the
     NULL curve, memory  is allocated  locally. Otherwise it is checked
     if enough memory is passed in. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   n;
       NL_DEGREE  p;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define array P and choose degree);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvInterp(P,n,p,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P   , input  ,  Points to be interpolated
     n   , input  ,  Highest index in P
     p   , input  ,  Degree of interpolating curve
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     cur , output ,  Interpolating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvInterp( NL_POINT *P, NL_INDEX n, NL_DEGREE p, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvInterp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, bw, sbw;

    NL_REAL ** A, *N;

    NL_PARAMETER *u;

    NL_CPOINT *Pw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( p GT n )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsKnotVectorAndKnots( cur, &Pw, &knt, &N );

    /* Get parameters and compute knot vetor */

    u = N_AllocReal1dArray( n, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, n, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    N_FitCrvCalcKnotVector( u, n, p, knt );

    /* Control points are the data points in case of linear curves */

    if( p EQ 1 )
    {
        for ( i = 0; i <= n; i++ )
            N_PtToCPt( P[i], &Pw[i] );

        NL_OUT;
    }

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[n][sbw] = 1.0;

    for ( i = 1; i < n; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - i - 1;

        for ( k = 0; k <= p; k++ )
            A[i][l + k] = N[k];
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    error = N_RealMatrixForBack( &cm, (NL_VOID *)P, NL_EPOINT, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCURVEINTPGIVENPARAMS: Curve interpolation with specified knot vector           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of points. This  is a general routine requiring the input of a
     specified knot vector, parameters at interpolation points, and the
     input  of NL_POINT or NL_CPOINT data. If the output curve is initialized 
     to the NULL  curve, memory  is allocated  locally. Otherwise it is 
     checked if  enough memory  is passed in. A typical calling example 
     is as follows:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  knt;
       NL_INDEX       n;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get arrays P/Pw and u, define knt, and choose p);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvInterpGivenParams((NL_VOID *)P ,n,NL_EPOINT,u,&knt,p,&cur,&SG);
       N_FitCrvInterpGivenParams((NL_VOID *)Pw,n,NL_HPOINT,u,&knt,p,&cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT NL_VECTOR ARE COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     A   , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     n   , input  ,  Highest index in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     u   , input  ,  Parameters corresponding to data points
     knt , input  ,  Knot vector of interpolating curve
     p   , input  ,  Degree of interpolating curve
     cur , output ,  Interpolating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvInterpGivenParams( NL_VOID *A, NL_INDEX n, NL_FLAG ptp, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvInterpGivenParams");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, m, mc, bw, sbw, sp;

    NL_REAL ** B, *N, *U, *UC;

    NL_POINT *Q;

    NL_CPOINT *Pw, *Qw;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    N_KnotVectorGetKnots( knt, &m, &U );
    mc = n + p + 1;

    if( m NEQ mc )
        NL_ERROR( NL_INP_ERR );
    /* if( p GT  n  )  NL_ERROR(NL_INP_ERR);  RCLxx */

    error = N_CrvSizeArrays( cur, n, p, mc, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= m; i++ )
        UC[i] = U[i];

    /* Control points are the data points in case of linear curves */

    if( p EQ 1 )
    {
        switch( ptp )
        {
            case NL_EPOINT:

                Q = (NL_POINT *)A;

                for ( i = 0; i <= n; i++ )
                    N_PtToCPt( Q[i], &Pw[i] );
                break;

            case NL_HPOINT:

                Qw = (NL_CPOINT *)A;

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Qw[i], &Pw[i] );
                break;

            default:

                NL_ERROR( NL_CAL_ERR );
        }

        NL_OUT;
    }

    /* Compute coefficient matrix */

    if( 2 *p LE n )
    {
        bw = 2 * p + 1;
        sbw = p;
    }
    else
    {
        bw = 2 * p - 1;
        sbw = p - 1;
    }
    sp = sbw - p; /* this must be non-positive */

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &B );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            B[i][j] = 0.0;
    }

    B[0][sbw] = 1.0;
    B[n][sbw] = 1.0;

    for ( i = 1; i < n; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = sp - i + j;

        if( l LT 0 OR l + p GE bw )
            NL_ERROR( NL_INP_ERR );

        for ( k = 0; k <= p; k++ )
            B[i][l + k] = N[k];
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    error = N_RealMatrixForBack( &cm, A, ptp, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCALCKNOTVECTORENDDERIVS: Knot vector for curve interpolation with end derivatives */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes the knot vector for global curve  
     interpolation with end derivatives specified. A typical calling 
     example is:

       NL_REAL        *u;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose p and define knt);
       ...
       N_FitCalcKnotVectorEndDerivs(u,k,p,&knt);

     IT IS ASSUMED THAT  MEMORY FOR knt IS  ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     u   , input  ,  Parameters
     k   , input  ,  Highest index in u
     p   , input  ,  Degree of interpolating curve
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FitCalcKnotVectorEndDerivs( NL_REAL *u, NL_INDEX k, NL_DEGREE p, NL_KNOTVECTOR *knt )
{

    NL_INDEX i, j, n;

    NL_REAL *U, sum;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute knot vector */

    n = k + 2;
    N_KnotVectorGetKnots( knt, &i, &U );

    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[n + i + 1] = u[k];
    }

    if( p EQ 1 )
    {
        if( k GT 1 )
        {
            U[2] = 0.5 *( u[0] + u[1] );
            U[n] = 0.5 *( u[k] + u[k - 1] );
        }
        else
        {
            U[2] = 0.75 *u[0] + 0.25 *u[1];
            U[3] = 0.25 *u[0] + 0.75 *u[1];
        }

        for ( i = 1; i < k; i++ )
            U[i + 2] = u[i];
    }
    else
    {
        for ( i = 0; i <= k - p + 1; i++ )
        {
            sum = 0.0;

            for ( j = i; j <= i + p - 1; j++ )
                sum += u[j];
            U[i + p + 1] = sum / p;
        }
    }

    N_SetKnotIndex( knt, n + p + 1 );

    /* End NURBS */

    N_EndNurbs( &SL );
}


/**********************************************************************/
/* N_FITCALCKNOTVECTORDERIV: Knot vector for curve interpolation with end derivative  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes the knot vector for global curve  
     interpolation with end derivative specified. A  typical calling 
     example is:

       NL_REAL        *u;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose p and define knt);
       ...
       N_FitCalcKnotVectorDeriv(u,k,p,NL_START,&knt);

     IT IS ASSUMED THAT  MEMORY FOR knt IS  ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     u   , input  ,  Parameters
     k   , input  ,  Highest index in u
     p   , input  ,  Degree of interpolating curve
     whr , input  ,  Flag:
                       NL_START: derivative specified at the start 
                       NL_END  : derivative specified at the end
     knt , output ,  Knot vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcKnotVectorDeriv( NL_REAL *u, NL_INDEX k, NL_DEGREE p, NL_FLAG whr, NL_KNOTVECTOR *knt )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcKnotVectorDeriv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n;

    NL_REAL *U, sum;

    /* Check error */

    if( (p GT k + 1)OR( (whr NEQ NL_START)AND( whr NEQ NL_END ) ) )
        NL_ERROR( NL_INP_ERR );

    /* Compute knot vector */

    n = k + 1;
    N_KnotVectorGetKnots( knt, &i, &U );

    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[n + i + 1] = u[k];
    }

    if( p EQ 1 ) /* special case out */
    {
        if( k EQ 1 )
            U[2] = 0.5 *( u[0] + u[1] );
        else
        {
            if( whr EQ NL_START )
            {
                U[2] = 0.5 *( u[0] + u[1] );

                for ( i = 1; i < k; i++ )
                    U[i + 2] = u[i];
            }

            if( whr EQ NL_END )
            {
                U[n] = 0.5 *( u[k] + u[k - 1] );

                for ( i = 1; i < k; i++ )
                    U[i + 1] = u[i];
            }
        }
    }

    if( p GT 1 AND p LT n )
    {
        if( whr EQ NL_START )
        {
            for ( i = 1; i <= k - p; i++ )
            {
                sum = 0.0;

                for ( j = i; j <= i + p - 1; j++ )
                    sum += u[j];
                U[i + p + 1] = sum / p;
            }

            sum = 0.0;

            for ( j = 0; j <= p - 1; j++ )
                sum += u[j];
            U[p + 1] = sum / p;
        }

        if( whr EQ NL_END )
        {
            for ( i = 1; i <= k - p; i++ )
            {
                sum = 0.0;

                for ( j = i; j <= i + p - 1; j++ )
                    sum += u[j];
                U[i + p] = sum / p;
            }

            sum = 0.0;

            for ( j = k - p + 1; j <= k; j++ )
                sum += u[j];
            U[n] = sum / p;
        }
    }

    N_SetKnotIndex( knt, n + p + 1 );

    EXIT:

    return (error);
}


/**********************************************************************/
/* N_FITCRVDERIVS: Curve interpolation with end derivatives specified       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of points and two end derivatives. The degree is arbitrary and  
     the type  of parametrization can be chosen. If the output curve is  
     initialized  to  the  NULL  curve, memory  is  allocated  locally. 
     Otherwise  it is  checked if enough memory is passed in. A typical 
     calling example is:

       NL_POINT   *P;
       NL_INDEX   kk;
       NL_DEGREE  p;
       NL_VECTOR  Ds, De;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P, the degree and the end derivatives Ds and De);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvDerivs(P,kk,p,Ds,De,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P     , input  ,  Points to be interpolated
     kk    , input  ,  Highest index in P
     p     , input  ,  Degree of  interpolating curve (must be <= kk+2)
     Ds,De , input  ,  Start and end derivatives (see NOTE)
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization
                         NL_CENTRIPETAL: Centripetal parametrization
                         NL_INHERITED    from Input curve
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   NOTE:
     Ds,De derivatives should be scaled by arclength distance between 
     points. Alternate version N_FitCrvTangents does this from tangent input. 
 
   ***********************************************************************/

NL_FLAG N_FitCrvDerivs( NL_POINT *P, NL_INDEX kk, NL_DEGREE p, NL_VECTOR Ds, NL_VECTOR De, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, bw, sbw;

    NL_REAL ** A, *U, *N, fact;

    NL_PARAMETER *u;

    NL_POINT *Q;

    NL_CPOINT *Pw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* neded for computing Ds, De if not supplied */
    NL_CURVE CubicCur;
    NL_POINT *Pset4;
    NL_POINT CD[2];
    NL_REAL len;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( p LT 2 OR p GT kk + 2 )
        NL_ERROR( NL_INP_ERR );

    /* If derivatives are missing we will estimate them */
    if( Ds.x == 0.0 && Ds.y == 0.0 && Ds.z == 0.0 )
    {
        N_CrvInitArrays( &CubicCur );

        /* Use cubic Bezier to get start and end tangent directions */
        error = N_FitCrvInterp( P, 3, 3, NL_CHORDLENGTH, &CubicCur, &SL );

        N_CrvDerivs( &CubicCur, 0.0, NL_LEFT, 1, CD );
        N_VectorCopy( CD[1], &Ds );
        N_DistPolygon( P, kk, &len );
        error = N_VectorNormalizeRef( &Ds );
        N_VectorScale( Ds, len, &Ds );
    }

    if( De.x == 0.0 && De.y == 0.0 && De.z == 0.0 )
    {
        N_CrvInitArrays( &CubicCur );
        Pset4 = &P[kk - 3];
        error = N_FitCrvInterp( Pset4, 3, 3, NL_CHORDLENGTH, &CubicCur, &SL );

        N_CrvDerivs( &CubicCur, 1.0, NL_RIGHT, 1, CD );
        N_VectorCopy( CD[1], &De );
        N_DistPolygon( P, kk, &len );
        error = N_VectorNormalizeRef( &De );
        N_VectorScale( De, len, &De );
    }

    n = kk + 2;

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsKnotVectorAndKnots( cur, &Pw, &knt, &U );

    /* Get parameters and compute knot vector */

    u = N_AllocReal1dArray( kk, &SL );

    if( u EQ NULL )
        NL_QUIT;

    if( par == NL_INHERITED && U != NULL )
    {
        for ( i = 0; i <= kk; i++ )
            u[i] = U[i + p];
    }
    else
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)P, kk, NL_EPOINT, par, u );

        if( error EQ NL_YES )
            NL_OUT;
        N_FitCalcKnotVectorEndDerivs( u, kk, p, knt ); /* This modifies the knots */
    }

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[1][sbw - 1] = -1.0;
    A[1][sbw] = 1.0;
    A[n - 1][sbw] = -1.0;
    A[n - 1][sbw + 1] = 1.0;
    A[n][sbw] = 1.0;

    for ( i = 2; i <= kk; i++ )
    {
        error = N_BasisEval( knt, p, u[i - 1], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - i - 1;

        for ( k = 0; k <= p; k++ )
            A[i][l + k] = N[k];
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    Q = N_AllocPt1dArray( n, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    /* this may be where the routine gives poor results ( /p ??)  */
    fact = U[p + 1] / (NL_REAL)p;
    N_CopyPt( P[0], &Q[0] );
    N_ScalePt( fact, Ds, &Q[1] );

    for ( i = 2; i <= kk; i++ )
        N_CopyPt( P[i - 1], &Q[i] );

    fact = (1.0 - U[n]) / p;
    N_ScalePt( fact, De, &Q[n - 1] );
    N_CopyPt( P[kk], &Q[n] );

    error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED


/**********************************************************************/
/* N_FITCRVTANGENTS: Curve interpolation with end tangents specified       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of points and two end tangents. The degree is arbitrary and  
     the type  of parametrization can be chosen. If the output curve is  
     initialized  to  the  NULL  curve, memory  is  allocated  locally. 
     Otherwise  it is  checked if enough memory is passed in. A typical 
     calling example is:

       NL_POINT   *P;
       NL_INDEX   kk;
       NL_DEGREE  p;
       NL_VECTOR  Ts, Te;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P, the degree and the end derivatives Ds and De);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvTangents(P,kk,p,Ts,Te,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P     , input  ,  Points to be interpolated
     kk    , input  ,  Highest index in P
     p     , input  ,  Degree of  interpolating curve (must be <= kk+2)
     Ts,Te , input  ,  Start and end tangent directions
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization
                         NL_CENTRIPETAL: Centripetal parametrization
                         NL_INHERITED  passed in from cur
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   NOTE:
     If you have derivatives (already scaled to arclength of points)
     you should call derivative version N_FitCrvDerivs.
     If Ts and Te are zero, they will be estimated from the point data.
 

   ***********************************************************************/

NL_FLAG N_FitCrvTangents( NL_POINT *P, NL_INDEX kk, NL_DEGREE p, NL_VECTOR Ts, NL_VECTOR Te, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_VECTOR Ds, De;

    NL_REAL len;

    /* Now scale Ts, Te by arclength of points */
    NL_CURVE CubicCur;
    NL_POINT *Pset4;
    NL_POINT CD[2];

    /* If derivatives are missing we will estimate them */
    if( Ts.x == 0.0 && Ts.y == 0.0 && Ts.z == 0.0 )
    {
        N_CrvInitArrays( &CubicCur );

        /* Use cubic Bezier to get start and end tangent directions */
        error = N_FitCrvInterp( P, 3, 3, NL_CHORDLENGTH, &CubicCur, SG );
        N_CrvDerivs( &CubicCur, 0.0, NL_LEFT, 1, CD );
        N_VectorCopy( CD[1], &Ts );
    }

    if( Te.x == 0.0 && Te.y == 0.0 && Te.z == 0.0 )
    {
        N_CrvInitArrays( &CubicCur );
        Pset4 = &P[kk - 3];
        error = N_FitCrvInterp( Pset4, 3, 3, NL_CHORDLENGTH, &CubicCur, SG );
        N_CrvDerivs( &CubicCur, 1.0, NL_RIGHT, 1, CD );
        N_VectorCopy( CD[1], &Te );
    }
    N_DistPolygon( P, kk, &len );
    error = N_VectorNormalizeRef( &Ts );
    N_VectorScale( Ts, len, &Ds );
    error = N_VectorNormalizeRef( &Te );
    N_VectorScale( Te, len, &De );

    error = N_FitCrvDerivs( P, kk, p, Ds, De, par, cur, SG );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVKNOTSANDDERIVS: Curve interpolation with end derivatives and knot vector */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of  points and two  end derivatives. This is a general routine
     requiring parameters where data points are assumed, a knot vector,
     and  NL_POINT or  NL_CPOINT data. If the  output curve is initialized to 
     the  NULL  curve, memory  is  allocated  locally. Otherwise  it is  
     checked if enough memory is  passed in. A typical calling example:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  knt;
       NL_INDEX       kk;
       NL_DEGREE      p;
       NL_VECTOR      Vs, Ve;
       NL_CVECTOR     Ws, We;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get arrays P/Pw; get Vs, Ve, Ws, We; define knt and get u);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvKnotsAndDerivs((NL_VOID *)P ,kk,NL_EPOINT,u,&knt,p,(NL_VOID *)&Vs,(NL_VOID *)&Ve,&cur,&SG);
       N_FitCrvKnotsAndDerivs((NL_VOID *)Pw,kk,NL_HPOINT,u,&knt,p,(NL_VOID *)&Ws,(NL_VOID *)&We,&cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT NL_VECTOR ARE COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     A     , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     kk    , input  ,  Highest index in A
     ptp   , input  ,  Flag:
                         NL_EPOINT: Euclidean point pointer passed in
                         NL_HPOINT: Homogeneous point pointer passed in
     u     , input  ,  Parameters corresponding to data points
     knt   , input  ,  Knot vector of interpolating curve
     p     , input  ,  Degree of  interpolating curve (must be <= kk+2)
     As,Ae , input  ,  NL_VOID pointer   representing NL_VECTOR or NL_CVECTOR as 
                       start and end  derivatives. THE  NL_START  AND  NL_END 
                       DERIVATIVES MUST  BE PASSED  IN BY  REFERENCE AS
                       THIS IS THE ONLY WAY TO CAST THEM TO NL_VOID.
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvKnotsAndDerivs( NL_VOID *A, NL_INDEX kk, NL_FLAG ptp, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_DEGREE p, NL_VOID *As, NL_VOID *Ae, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvKnotsAndDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, mc, bw, sbw, sp;

    NL_REAL ** B, *U, *UC, *N, fact;

    NL_POINT *Q, *R;

    NL_VECTOR Vs, Ve;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_CVECTOR Ws, We;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors and see if curve memory is needed */

    N_KnotVectorGetKnots( knt, &m, &U );

    n = kk + 2;
    mc = n + p + 1;

    if( p LT 2 OR p GT kk + 2 )
        NL_ERROR( NL_INP_ERR );

    if( m NEQ mc )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= m; i++ )
        UC[i] = U[i];

    /* Compute coefficient matrix */

    if( 2 *p LE n )
    {
        bw = 2 * p + 1;
        sbw = p;
    }
    else
    {
        bw = 2 * p - 1;
        sbw = p - 1;
    }
    sp = sbw - p; /* this must be non-positive */

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &B );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            B[i][j] = 0.0;
    }

    B[0][sbw] = 1.0;
    B[1][sbw - 1] = -1.0;
    B[1][sbw] = 1.0;
    B[n - 1][sbw] = -1.0;
    B[n - 1][sbw + 1] = 1.0;
    B[n][sbw] = 1.0;

    for ( i = 2; i <= kk; i++ )
    {
        error = N_BasisEval( knt, p, u[i - 1], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = sp - i + j;

        if( l LT 0 OR l + p GE bw )
            NL_ERROR( NL_INP_ERR );

        for ( k = 0; k <= p; k++ )
            B[i][l + k] = N[k];
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)A;
            Vs = *(NL_VECTOR *)As;
            Ve = *(NL_VECTOR *)Ae;

            Q = N_AllocPt1dArray( n, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyPt( R[0], &Q[0] );
            N_ScalePt( fact, Vs, &Q[1] );

            for ( i = 2; i <= kk; i++ )
                N_CopyPt( R[i - 1], &Q[i] );

            fact = (U[n + p] - U[n]) / p;
            N_ScalePt( fact, Ve, &Q[n - 1] );
            N_CopyPt( R[kk], &Q[n] );

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            Ws = *(NL_CVECTOR *)As;
            We = *(NL_CVECTOR *)Ae;

            Qw = N_AllocCPt1dArray( n, &SL );

            if( Qw EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyCPt( Rw[0], &Qw[0] );
            N_ScaleCPt( fact, Ws, &Qw[1] );

            for ( i = 2; i <= kk; i++ )
                N_CopyCPt( Rw[i - 1], &Qw[i] );

            fact = (U[n + p] - U[n]) / p;
            N_ScaleCPt( fact, We, &Qw[n - 1] );
            N_CopyCPt( Rw[kk], &Qw[n] );

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVKNOTSANDDERIV: Curve interpolation with end derivative and knot vector  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes a NURBS curve  interpolating a given
     set of points and  one end  derivative.  This is  a general routine
     requiring parameters where data  points are assumed, a knot vector,
     and  NL_POINT or  NL_CPOINT  data. If the  output curve is initialized to 
     the  NULL  curve,  memory  is  allocated  locally. Otherwise  it is  
     checked if enough  memory is  passed in. A typical calling example:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  knt;
       NL_INDEX       kk;
       NL_DEGREE      p;
       NL_VECTOR      V;
       NL_CVECTOR     W;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get arrays P/Pw; get V, W; define knt and get u);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvKnotsAndDeriv((NL_VOID *)P ,kk,NL_EPOINT,u,&knt,p,(NL_VOID *)&V,NL_START,&cur,&SG);
       N_FitCrvKnotsAndDeriv((NL_VOID *)Pw,kk,NL_HPOINT,u,&knt,p,(NL_VOID *)&W,NL_END  ,&cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT NL_VECTOR ARE  COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     A     , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     kk    , input  ,  Highest index in A
     ptp   , input  ,  Flag:
                         NL_EPOINT: Euclidean point pointer passed in
                         NL_HPOINT: Homogeneous point pointer passed in
     u     , input  ,  Parameters corresponding to data points
     knt   , input  ,  Knot vector of interpolating curve
     p     , input  ,  Degree of  interpolating curve (must be <= kk+1)
     T     , input  ,  NL_VOID pointer   representing NL_VECTOR or NL_CVECTOR as 
                       start or  end  derivative.  THE  NL_START  OR   NL_END 
                       NL_DERIVATIVE  MUST  BE PASSED  IN BY  REFERENCE AS
                       THIS IS THE ONLY WAY TO CAST THEM TO NL_VOID.
     whr   , input  ,  Flag:
                         NL_START: T is start derivative
                         NL_END  : T is end derivative
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvKnotsAndDeriv( NL_VOID *A, NL_INDEX kk, NL_FLAG ptp, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_DEGREE p, NL_VOID *T, NL_FLAG whr, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvKnotsAndDeriv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, mc, bw, sbw;

    NL_REAL ** B, *U, *UC, *N, fact;

    NL_POINT *Q, *R;

    NL_VECTOR V;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_CVECTOR W;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors and see if curve memory is needed */

    N_KnotVectorGetKnots( knt, &m, &U );

    n = kk + 1;
    mc = n + p + 1;

    if( p LT 2 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    if( m NEQ mc )
        NL_ERROR( NL_INP_ERR );

    switch( ptp )
    {
        case NL_EPOINT:
            break;

        case NL_HPOINT:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    switch( whr )
    {
        case NL_START:
            break;

        case NL_END:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= m; i++ )
        UC[i] = U[i];

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &B );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            B[i][j] = 0.0;
    }

    switch( whr )
    {
        case NL_START:

            B[0][sbw] = 1.0;
            B[1][sbw - 1] = -1.0;
            B[1][sbw] = 1.0;
            B[n][sbw] = 1.0;

            for ( i = 2; i <= kk; i++ )
            {
                error = N_BasisEval( knt, p, u[i - 1], NL_LEFT, N, &j );

                if( error EQ NL_YES )
                    NL_OUT;

                if( p EQ 2 )
                    l = 0;
                else
                    l = j - i - 1;

                if( l LT 0 OR l + p GE bw )
                    NL_ERROR( NL_INP_ERR );

                for ( k = 0; k <= p; k++ )
                    B[i][l + k] = N[k];
            }
            break;

        case NL_END:

            B[0][sbw] = 1.0;
            B[n - 1][sbw] = -1.0;
            B[n - 1][sbw + 1] = 1.0;
            B[n][sbw] = 1.0;

            for ( i = 1; i <= kk - 1; i++ )
            {
                error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

                if( error EQ NL_YES )
                    NL_OUT;

                if( p EQ 2 )
                    l = 0;
                else
                    l = j - i - 1;

                if( l LT 0 OR l + p GE bw )
                    NL_ERROR( NL_INP_ERR );

                for ( k = 0; k <= p; k++ )
                    B[i][l + k] = N[k];
            }
            break;
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)A;
            V = *(NL_VECTOR *)T;

            Q = N_AllocPt1dArray( n, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            switch( whr )
            {
                case NL_START:

                    fact = (U[p + 1] - U[1]) / p;
                    N_CopyPt( R[0], &Q[0] );
                    N_ScalePt( fact, V, &Q[1] );

                    for ( i = 2; i <= n; i++ )
                        N_CopyPt( R[i - 1], &Q[i] );
                    break;

                case NL_END:

                    fact = (U[n + p] - U[n]) / p;
                    N_ScalePt( fact, V, &Q[n - 1] );
                    N_CopyPt( R[kk], &Q[n] );

                    for ( i = 0; i <= n - 2; i++ )
                        N_CopyPt( R[i], &Q[i] );
                    break;
            }

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            W = *(NL_CVECTOR *)T;

            Qw = N_AllocCPt1dArray( n, &SL );

            if( Qw EQ NULL )
                NL_QUIT;

            switch( whr )
            {
                case NL_START:

                    fact = (U[p + 1] - U[1]) / p;
                    N_CopyCPt( Rw[0], &Qw[0] );
                    N_ScaleCPt( fact, W, &Qw[1] );

                    for ( i = 2; i <= n; i++ )
                        N_CopyCPt( Rw[i - 1], &Qw[i] );
                    break;

                case NL_END:

                    fact = (U[n + p] - U[n]) / p;
                    N_ScaleCPt( fact, W, &Qw[n - 1] );
                    N_CopyCPt( Rw[kk], &Qw[n] );

                    for ( i = 0; i <= n - 2; i++ )
                        N_CopyCPt( Rw[i], &Qw[i] );
                    break;
            }

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}


/**********************************************************************/
/* N_FITCUBICSPLINEINTP: Cubic spline interpolation                               */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a cubic spline curve interpolating a 
     given  set  of  points  and  two  end  derivatives.  The  type  of 
     parametrization can be chosen. If the output curve is  initialized  
     to the  NULL curve, memory  is allocated locally. Otherwise  it is  
     checked if  enough memory is passed in. A typical calling  example
     is as follows:

       NL_POINT   *P;
       NL_INDEX   k;
       NL_VECTOR  Ds, De;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P and the end derivatives Ds and De);
       ...
       N_FitCubicSplineInterp(P,k,Ds,De,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P     , input  ,  Points to be interpolated
     k     , input  ,  Highest index in P
     Ds,De , input  ,  Start and end derivatives
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization
                         NL_CENTRIPETAL: Centripetal parametrization
     cur   , output ,  Cubic spline
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCubicSplineInterp( NL_POINT *P, NL_INDEX k, NL_VECTOR Ds, NL_VECTOR De, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCubicSplineInterp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n;

    NL_REAL *dd, *U, abc[4], den, alf, bet, gam;

    NL_PARAMETER *u;

    NL_POINT A;

    NL_CPOINT *Pw, *Rw, Aw;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    NL_CURVE CubicCur;
    NL_POINT *Pset4;
    NL_POINT CD[2];
    NL_REAL len;

    /* Start NURBS */

    N_InitNurbs( &SL );

    if( k LE 0 )
        NL_ERROR( NL_INP_ERR );

    /* If derivatives are missing we will estimate them */
    if( Ds.x == 0.0 && Ds.y == 0.0 && Ds.z == 0.0 )
    {
        N_CrvInitArrays( &CubicCur );

        /* Use cubic Bezier to get start and end tangent directions */
        error = N_FitCrvInterp( P, 3, 3, NL_CHORDLENGTH, &CubicCur, &SL );

        N_CrvDerivs( &CubicCur, 0.0, NL_LEFT, 1, CD );
        N_VectorCopy( CD[1], &Ds );
        N_DistPolygon( P, k, &len );
        error = N_VectorNormalizeRef( &Ds );
        N_VectorScale( Ds, len, &Ds );
    }

    if( De.x == 0.0 && De.y == 0.0 && De.z == 0.0 )
    {
        N_CrvInitArrays( &CubicCur );
        Pset4 = &P[k - 3];
        error = N_FitCrvInterp( Pset4, 3, 3, NL_CHORDLENGTH, &CubicCur, &SL );

        N_CrvDerivs( &CubicCur, 1.0, NL_RIGHT, 1, CD );
        N_VectorCopy( CD[1], &De );
        N_DistPolygon( P, k, &len );
        error = N_VectorNormalizeRef( &De );
        N_VectorScale( De, len, &De );
    }

    /* Check for curve memory */

    n = k + 2;

    error = N_CrvSizeArrays( cur, n, 3, n + 4, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsKnotVectorAndKnots( cur, &Pw, &knt, &U );

    /* Get parameters */

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute knot vector */

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = 0.0;
        U[n + i + 1] = 1.0;
    }

    for ( i = 1; i < k; i++ )
        U[i + 3] = u[i];

    /* Compute first two and last two control points */

    alf = 1.0;
    bet = U[4] / 3.0;

    N_PtToCPt( P[0], &Pw[0] );
    N_Combine2Pts( alf, P[0], bet, Ds, &A );
    N_PtToCPt( A, &Pw[1] );

    bet = (U[n] - 1.0) / 3.0;

    N_PtToCPt( P[k], &Pw[n] );
    N_Combine2Pts( alf, P[k], bet, De, &A );
    N_PtToCPt( A, &Pw[n - 1] );

    if( k EQ 1 )
        NL_OUT;

    /* Handle case k equal 2 special */

    if( k EQ 2 )
    {
        error = N_BasisEval( knt, 3, U[4], NL_LEFT, abc, &j );

        if( error EQ NL_YES )
            NL_OUT;

        den = abc[1];
        alf = 1.0 / den;
        bet = -abc[0] / den;

        N_PtToCPt( P[1], &Aw );
        N_Combine2CPts( alf, Aw, bet, Pw[1], &Pw[2] );

        alf = 1.0;
        bet = -abc[2] / den;

        N_Combine2CPts( alf, Pw[2], bet, Pw[3], &Pw[2] );

        NL_OUT;
    }

    /* Solve tridiagonal system */

    Rw = N_AllocCPt1dArray( k, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    dd = N_AllocReal1dArray( k, &SL );

    if( dd EQ NULL )
        NL_QUIT;

    for ( i = 3; i < k; i++ )
        N_PtToCPt( P[i - 1], &Rw[i] );

    error = N_BasisEval( knt, 3, U[4], NL_LEFT, abc, &j );

    if( error EQ NL_YES )
        NL_OUT;

    den = abc[1];
    alf = 1.0 / den;
    bet = -abc[0] / den;

    N_PtToCPt( P[1], &Aw );
    N_Combine2CPts( alf, Aw, bet, Pw[1], &Pw[2] );

    for ( i = 3; i < k; i++ )
    {
        dd[i] = abc[2] / den;

        error = N_BasisEval( knt, 3, U[i + 2], NL_LEFT, abc, &j );

        if( error EQ NL_YES )
            NL_OUT;

        den = abc[1] - abc[0] * dd[i];
        alf = 1.0 / den;
        bet = -abc[0] / den;

        N_Combine2CPts( alf, Rw[i], bet, Pw[i - 1], &Pw[i] );
    }

    dd[k] = abc[2] / den;

    error = N_BasisEval( knt, 3, U[n], NL_LEFT, abc, &j );

    if( error EQ NL_YES )
        NL_OUT;

    den = abc[1] - abc[0] * dd[k];
    alf = 1.0 / den;
    bet = -abc[2] / den;
    gam = -abc[0] / den;

    N_PtToCPt( P[k - 1], &Aw );
    N_Combine2CPts( alf, Aw, bet, Pw[k + 1], &Pw[k] );
    N_VectorBlendCPt( gam, Pw[k - 1], &Pw[k] );

    for ( i = k - 1; i >= 2; i-- )
        N_VectorBlendCPt( -dd[i + 1], Pw[i + 1], &Pw[i] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitCrvCPtsFromSrfData: Curve interpolation through row/column of surface data   */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes the control points of curves inter-
     polating rows/columns of surface data points. In order to facili-
     tate with both  NL_POINT and  NL_CPOINT objects, the routine  employs a 
     NL_VOID  pointer as input  parameter, and assumes  that  the calling 
     routine typecasts the given pointer to the  appropriate type. The 
     calling mechanism works as follows:

       NL_CPOINT   **Pw, *Qw;
       NL_POINT    **P;
       NL_INDEX    n, m, k;
       NL_RMATRIX  cm;
       ...
       (define data point array, compute matrix cm and allocate memory
        for Qw);
       ...
       N_FitCrvCPtsFromSrfData((NL_VOID **)P ,n,m,NL_EPOINT,k,NL_UDIR,&cm,Qw);
       N_FitCrvCPtsFromSrfData((NL_VOID **)Pw,n,m,NL_HPOINT,k,NL_VDIR,&cm,Qw);

     It is  assumed that memory  to store the output control points is
     allocated  in the calling  routine. IT  IS  ALSO ASSUMED THAT THE 
     MATRIX  cm  IS THE LU DECOMPOSED INTERPOLATION COEFFICIENT MATRIX 
     COMPUTED IN THE CALLING ROUTINE.


   ACCESS:
   
     A   , input  ,  NL_VOID pointer representing either NL_POINT or NL_CPOINT
     n,m , input  ,  Highest indexes in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     k   , input  ,  Index of row/column to be considered
     dir , input  ,  Flag:
                       NL_UDIR: Interpolate in u-direction (row)
                       NL_VDIR: Interpolate in v-direction (column)
     cm  , input  ,  LU decomposed coefficient matrix
     Qw  , output ,  Control points of curve  interpolating row/column
                     of surface data points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvCPtsFromSrfData( NL_VOID ** A, NL_INDEX n, NL_INDEX m, NL_FLAG ptp, NL_INDEX k, NL_FLAG dir, NL_RMATRIX *cm, NL_CPOINT *Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvCPtsFromSrfData");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_POINT ** P = NULL, *R = NULL;

    NL_CPOINT ** Pw = NULL, *Rw = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Typecast input data */

    switch( ptp )
    {
        case NL_EPOINT:

            P = (NL_POINT ** )A;

            R = N_AllocPt1dArray( NL_MAX( n, m ), &SL );

            if( R EQ NULL )
                NL_OUT;
            break;

        case NL_HPOINT:

            Pw = (NL_CPOINT ** )A;

            Rw = N_AllocCPt1dArray( NL_MAX( n, m ), &SL );

            if( Rw EQ NULL )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Compute control points */

    switch( dir )
    {
        case NL_UDIR:
            for ( i = 0; i <= n; i++ )
            {
                if( ptp EQ NL_EPOINT )
                    N_CopyPt( P[i][k], &R[i] );
                else
                    N_CopyCPt( Pw[i][k], &Rw[i] );
            }
            break;

        case NL_VDIR:
            for ( j = 0; j <= m; j++ )
            {
                if( ptp EQ NL_EPOINT )
                    N_CopyPt( P[k][j], &R[j] );
                else
                    N_CopyCPt( Pw[k][j], &Rw[j] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    if( ptp EQ NL_EPOINT )
    {
        error = N_RealMatrixForBack( cm, (NL_VOID *)R, NL_EPOINT, Qw, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_RealMatrixForBack( cm, (NL_VOID *)Rw, NL_HPOINT, Qw, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCALCKNOTVECTORCRVAPPROX: Compute knot vector for global curve approximation       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes the knot vector for global curve 
     approximation. It selects U[i] values near sample points.
     If the sample points are evenly distributed U[i] will be
     evenly distributed.  If the sample points are heavily clustered
     around one or a few values, U[i] will be clustered.
     A typical calling example is:

       NL_REAL        *u;
       NL_INDEX       m, n;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose n and p, and define knt);
       ...
       N_FitCalcKnotVectorCrvApprox(u,m,n,p,&knt);

     IT IS ASSUMED THAT MEMORY FOR  knt IS ALLOCATED IN THE CALLING
     ROUTINE. 


   ACCESS:
   
     u   , input  ,  Parameters in ascending order
     m   , input  ,  Highest index in u
     n   , input  ,  Highest   index  of  control  point  array  of 
                     approximating curve (n <= m)
     p   , input  ,  Degree of approximating curve (p<n) 
     knt , output ,  Knot vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcKnotVectorCrvApprox
  (NL_REAL       *u,     /* in : parameters, sized:[m+1], ascending order                */
   NL_INDEX       m,     /* in : highest index in u                                      */
   NL_INDEX       n,     /* in : output curve highest control point index, note:(n <= m) */
   NL_DEGREE      p,     /* in : output curve degree, note:(p < n)                       */
   NL_KNOTVECTOR *knt )  /* i/o: in : preallocated with knt->m >= n+p+1                  */       
                         /*      out: knt->U[i] = Avg(cluster[i].u, cluster[i+p].u)      */
                         /*           knt->m = n+p+1                                     */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcKnotVectorCrvApprox");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, il, ih;

    NL_REAL *U, *uk, l, d, sum;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize */

    /* check state: as many or more sample points than control points */
    if( n GT m )
      { NL_ERROR( NL_INP_ERR ); }

    /* i = knt->m, U = knt->U */
    N_KnotVectorGetKnots( knt, &i, &U );

    /* set 1st and last p+1 U values to 1st and last sample u values */
    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[n + i + 1] = u[m];
    }

    /* Compute representatives of clusters of parameters - one cluster for every one control point */

    uk = N_AllocReal1dArray( n, &SL );

    if( uk EQ NULL )
        NL_QUIT;

    d = (m + 1.0) / (n + 1.0); /* number of samples/number of control points (always greater than 1) */
    il = 0;                    /* il = 1st index for current cluster of sample points */
    ih = il;                   /* ih = last index for current cluster of sample points */
    l = -1.0;

    /* for every control point - get a typical u value for its sample cluster */
    for ( i = 0; i <= n; i++ )
    {
        l = l + d;         /* l  = index of last sample clustered together for control point n */
        ih = ROUND( l );   /* ih = (NL_INDEX)(l + .5) */

        /* When only one sample is available for this control point */
        if( il EQ ih )    
        {
            uk[i] = u[il];  /* set uk value to sample u value */
        }
        else  /* set uk = Avg(clustered[i] sample u values) */
        {
            sum = 0.0;

            for ( j = il; j <= ih; j++ )
                sum += u[j];
            uk[i] = sum / ((NL_REAL)ih - (NL_REAL)il + (NL_REAL)1);
        }

        il = ih + 1;
    } /* end iter every control point */

    /* Now compute the knot vector */

    /* for every internal knot value */
    for ( i = 1; i <= n - p; i++ )
    {
        sum = 0.0;

        /* let internal knot value = Avg(next p cluster[i] U values) */
        for ( j = i; j <= i + p - 1; j++ )
            sum += uk[j];
        U[i + p] = sum / p;
    }

    N_SetKnotIndex( knt, n + p + 1 );

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_FitCalcKnotVectorCrvApprox */



/**********************************************************************/
/* N_FITCRVAPPROXLSTSQ: Global curve approximation with arbitrary degree         */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes a least-squares NURBS curve approxi-
     mation to a given set of  points. The degree is  arbitrary and the 
     type of parametrization and the number of control  points  can  be  
     chosen. If the  output curve  is  initialized  to the  NULL curve, 
     memory is allocated  locally. Otherwise  it is  checked  if enough  
     memory is passed in. A typical calling example is:
 
       NL_POINT   *P;
       NL_INDEX   m, n;
       NL_DEGREE  p;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define array P and choose degree and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxLstSq(P,m,n,p,NL_CHORDLENGTH,&cur,&SG);
 
     A recommended default for the parametrization type is NL_CHORDLENGTH.
 
 
   ACCESS:
   
     P   , input  ,  Points to be approximated
     m   , input  ,  Highest index in P
     n   , input  ,  Highest   index   of   control   point   array  of 
                     approximating curve (must satisfy n <= m);
     p   , input  ,  Degree of approximating curve (must satisfy p <= n)
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCrvApproxLstSq( NL_POINT *P, NL_INDEX m, NL_INDEX n, NL_DEGREE p, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxLstSq");

    NL_FLAG error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, l, bw, sbw, rj, sj, ej, lk, hk, lj, hj;

    NL_REAL ** A, ** NTN, *N, n0, np;

    NL_PARAMETER *u;

    NL_POINT *Rk, *R;

    NL_CPOINT *Pw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( n LT 2 OR p GT n OR m LT n )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &N );
    N_CrvGetKnotVector( cur, &knt );

    N_PtToCPt( P[0], &Pw[0] );
    N_PtToCPt( P[m], &Pw[n] );

    /* Get parameters and the knot vector */

    u = N_AllocReal1dArray( m, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_FitCalcKnotVectorCrvApprox( u, m, n, p, knt );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get the Rk's */

    Rk = N_AllocPt1dArray( m - 2, &SL );

    if( Rk EQ NULL )
        NL_QUIT;

    for ( i = 1; i <= m - 1; i++ )
    {
        error = N_BasisIEval( knt, 0, p, u[i], NL_LEFT, &n0 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knt, n, p, u[i], NL_LEFT, &np );

        if( error EQ NL_YES )
            NL_OUT;

        N_TranslateSum2Pts( P[i], -n0, P[0], -np, P[m], &Rk[i - 1] );
    }

    /* Compute coefficient matrix */

    bw = 2 * p + 1;
    sbw = p;
    rj = p;
    sj = p - 1;
    ej = -2;

    A = N_AllocReal2dArray( m - 2, p, &SL );

    if( A EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    index = N_AllocInt1dArray( m - 2, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( n - 2, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( n - 2, &SL );

    if( end EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n - 2, n - 2, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &NTN );

    for ( i = 0; i <= n - 2; i++ )
    {
        for ( j = 0; j < bw; j++ )
            NTN[i][j] = 0.0;
    }

    for ( i = 0; i <= m - 2; i++ )
        A[i][p] = 0.0;

    for ( i = 0; i <= NL_MIN( p - 1, n - 2 ); i++ )
        start[i] = 0;

    end[0] = -2;

    for ( i = 1; i <= m - 1; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        if( j EQ p )
            l = 1;
        else
            l = 0;

        if( j EQ p OR j EQ n )
            hk = p - 1;
        else
            hk = p;

        for ( k = 0; k <= hk; k++ )
            A[i - 1][k] = N[l + k];

        index[i - 1] = NL_MAX( 0, j - p - 1 );

        if( j GT rj )
        {
            for ( k = 1; k <= j - rj; k++ )
            {
                sj++;
                ej++;

                if( sj LE n - 2 )
                    start[sj] = i - 1;

                if( ej GE 0 )
                    end[ej] = i - 2;
            }
            rj = j;
        }
    }

    if( sj LT n - 2 OR end[0]EQ - 1 )
        NL_ERROR( NL_INP_ERR );

    for ( i = NL_MAX( 0, ej + 1 ); i <= n - 2; i++ )
        end[i] = m - 2;

    for ( i = 0; i <= n - 2; i++ )
    {
        lj = NL_MAX( 0, i - p );
        hj = NL_MIN( n - 2, i + p );

        for ( j = lj; j <= hj; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( k = lk; k <= hk; k++ )
            {
                NTN[i][j - i + sbw] += A[k][i - index[k]] * A[k][j - index[k]];
            }
        }
    }

    /* Get right hand side of equation */

    R = N_AllocPt1dArray( n - 2, &SL );

    if( R EQ NULL )
        NL_QUIT;

    for ( i = 1; i <= n - 1; i++ )
    {
        N_CopyPt( NL_ZERO, &R[i - 1] );

        lk = start[i - 1];
        hk = end[i - 1];

        for ( k = lk; k <= hk; k++ )
        {
            N_VectorBlendPt( A[k][i - index[k] - 1], Rk[k], &R[i - 1] );
        }
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, &Pw[1], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROX: Curve approximation with error bound specified           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS  curve approximating a given
     set of points to a user given tolerance. The method starts with an
     interpolating curve of given degree and one control point per sample
     point and works its way up to the 
     required degree by repeating the following steps:
       (1) remove as many knots as possible;
       (2) least-squares fit with a higher degree curve; and
       (3) adjust parameter values and error vector.
     If one of the  above steps fails,  the process is  re-started with 
     another  interpolating  curve  of  one  degree  higher. A  typical 
     calling example is as follows:

       NL_POINT   *P;
       NL_INDEX   m;
       NL_DEGREE  ps, pr;
       NL_REAL    E;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P, and choose ps, pr and E);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApprox(P,m,NL_YES,ps,pr,E,NL_SINGLE,&cur,&SG);

     The  algorithm can  use single or multiple knots for data approxi-
     mation. The  single-knot  approximation  smooths  out  the  data,
     whereas  the  multiple-knot  approximant captures  local phenomena 
     such as straight edges and cusps.


   ACCESS:
   
     P   , input  ,  Point array to be approximated
     m   , input  ,  Highest index in P
     cld , input  ,  Flag:
                       NL_YES: consider data set as closed if P[0]=P[m]
                       NL_NO : data set is open even if P[0]=P[m]
     ps  , input  ,  Start iterations with degree ps (if  cusps are  to 
                     be preserved, ps=1 is recommended. Otherwise  ps=2
                     is a good  default). IF  CLOSED NL_CURVE IS REQUIRED, 
                     ps MUST BE >= 2!
     pr  , input  ,  Required degree of approximating curve
     E   , input  ,  Error tolerance
     ktp , input  ,  Flag:
                       NL_SINGLE  : use single knots for approximation
                       NL_MULTIPLE: use multiple knots for approximation
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvApprox( NL_POINT *P, NL_INDEX m, NL_FLAG cld, NL_DEGREE ps, NL_DEGREE pr, NL_REAL E, NL_FLAG ktp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApprox");

    NL_FLAG apr, reset = NL_NO, closed = NL_NO, error = NL_NO;

    NL_INDEX I[2], k, l, nc, mc, nh, mh, nsp, mlt;

    NL_DEGREE i, j;

    NL_REAL *U, *er, *wp = NULL, wd[2], d, ltop, ltoc;

    NL_PARAMETER *u, *us;

    NL_POINT *Q, R;

    NL_VECTOR D[2], *Vs = NULL, *Ve = NULL;

    NL_KNOTVECTOR *knt, *kna;

    NL_CURVE curA;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL top = 1.0e-04;
    NL_PRIVATE NL_REAL toc = 1.0e-06;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Adjust tolerance */

    N_Pts1dGetMaxExtent( P, m, &d );

    ltop = d * top;
    ltoc = toc;

    /* Check for curve memory */

    if( pr GT m )
        NL_ERROR( NL_INP_ERR );

    if( pr LT ps )
        NL_ERROR( NL_INP_ERR );

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    error = N_CrvSizeArrays( cur, m + 2, pr, m + pr + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnotVector( cur, &knt );
    N_KnotVectorGetKnots( knt, &k, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( m, &SL );

    if( u EQ NULL )
        NL_QUIT;

    us = N_AllocReal1dArray( m, &SL );

    if( us EQ NULL )
        NL_QUIT;

    er = N_AllocReal1dArray( m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    /* See if data set is closed */

    if( N_1dPtSetIsClosed( (NL_VOID *)P, m, NL_EPOINT, NL_MTOL ) )
    {
        if( cld EQ NL_YES )
            closed = NL_YES;
    }

    if( closed EQ NL_YES )
    {
        if( ps EQ 1 )
            NL_ERROR( NL_INP_ERR );

        /* Get derivative at the end points */

        Q = N_AllocPt1dArray( m, &SL );

        if( Q EQ NULL )
            NL_QUIT;

        wp = N_AllocReal1dArray( m, &SL );

        if( wp EQ NULL )
            NL_QUIT;

        nc = m / 2;

        for ( k = 0; k <= m; k++ )
        {
            l = (nc + k) % m;

            if( l EQ 0 )
                l = m;

            N_CopyPt( P[l], &Q[k] );
            wp[k] = 1.0;
            er[k] = 0.0;
        }

        error = N_FitCalcCrvParamValues( (NL_VOID *)Q, m, NL_EPOINT, NL_CHORDLENGTH, u );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_AllocCrvArrays( &curA, m, pr, m + pr + 1, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnotVector( &curA, &kna );
        N_FitCrvCalcKnotVector( u, m, pr, kna );

        error = N_FitCrvInterpGivenParams( (NL_VOID *)Q, m, NL_EPOINT, u, kna, pr, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitRemoveKnots( &curA, u, er, m, E );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetArraySizes( &curA, &nc, &mc );
        error = N_FitCrvApproxKnots( (NL_VOID *)Q, m, NL_EPOINT, u, kna, nc, pr, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        i = (NL_DEGREE)(m + 1) / 2;
        error = N_CrvDerivs( &curA, u[i], NL_LEFT, 1, D );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( D[1], &D[0] );
        Vs = &D[0];
        Ve = &D[1];

        /* Prepare for constrained fitting */

        wp[0] = -1.0;
        wp[m] = -1.0;
        wd[0] = -1.0;
        wd[1] = -1.0;
        I[0] = 0;
        I[1] = m;
    }

    /* Get parameters */

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, NL_CHORDLENGTH, us );

    if( error EQ NL_YES )
        NL_OUT;

    /*********************************************************/
    /*  Fit curve as follows:                                */
    /*    for i=ps to pr do                                  */
    /*       interpolate with degree i                       */
    /*       for j=i to pr do                                */
    /*          remove knots                                 */
    /*          compute degree elevated curve's knot vector  */
    /*          least-squares approximate                    */
    /*          if approximation fails break                 */
    /*          adjust parameters and error vector           */
    /*********************************************************/

    for ( i = ps; i <= pr; i++ )
    {
        /* Interpolate with degree i */

        for ( k = 0; k <= m; k++ )
        {
            u[k] = us[k];
            er[k] = 0.0;
        }

        if( closed EQ NL_YES )
        {
            N_FitCalcKnotVectorEndDerivs( u, m, i, knt );
            N_CrvSetSizeIndices( cur, m + 2, i, m + i + 3 );
            error = N_FitCrvKnotsAndDerivs( (NL_VOID *)P, m, NL_EPOINT, u, knt, i, (NL_VOID *)Vs, (NL_VOID *)Ve, cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            N_FitCrvCalcKnotVector( u, m, i, knt );
            N_CrvSetSizeIndices( cur, m, i, m + i + 1 );
            error = N_FitCrvInterpGivenParams( (NL_VOID *)P, m, NL_EPOINT, u, knt, i, cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( j = i; j <= pr; j++ )
        {
            /* Remove as many knots as possible */

            error = N_FitRemoveKnots( cur, u, er, m, E );

            if( error EQ NL_YES )
                NL_OUT;

            if( j EQ pr )
                break;

            N_CrvGetArraySizes( cur, &nc, &mc );

            /* Get new knot vector */

            if( ktp EQ NL_SINGLE )
            {
                nh = nc + 1;
                mh = mc + 2;

                if( nh GT m )
                    break;

                for ( k = 0; k <= j + 1; k++ )
                    U[mh--] = U[mc];

                for ( k = nc; k >= j + 1; k-- )
                    U[mh--] = U[k];

                for ( k = 0; k <= j + 1; k++ )
                    U[mh--] = U[0];
            }
            else
            {
                N_BasisGetSpanCount( knt, j, &nsp );

                nh = nc + nsp;
                mh = mc + nsp + 1;

                if( nh GT m )
                    break;

                while( mc GT 0 )
                {
                    k = mc;

                    while( mc GT 0 AND U[mc]EQ U[mc - 1] )
                        mc--;
                    mlt = k - mc + 1;

                    for ( l = 1; l <= mlt + 1; l++ )
                        U[mh--] = U[mc];
                    mc--;
                }
            }

            /* Approximate by least-squares */

            N_CrvSetSizeIndices( cur, nh, (NL_DEGREE)(j + 1), nh + j + 2 );

            if( closed EQ NL_YES )
            {
                error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, m, (NL_VOID *)D, wd, I, 1, NL_EPOINT, u, knt, nh, (NL_DEGREE)(j + 1), cur, &SL );

                if( error EQ NL_YES )
                    break;
            }
            else
            {
                error = N_FitCrvApproxKnots( (NL_VOID *)P, m, NL_EPOINT, u, knt, nh, (NL_DEGREE)(j + 1), cur, &SL );

                if( error EQ NL_YES )
                    break;
            }

            /* Adjust parameters and error vector */

            apr = NL_TRUE;

            for ( k = 1; k < m; k++ )
            {
                error = N_CrvClosestPt( cur, P[k], u[k], ltop, ltoc, &u[k], &R );

                if( error EQ NL_YES )
                {
                    apr = NL_FALSE;
                    break;
                }

                N_DistPtPt( P[k], R, &er[k] );

                if( er[k]GT E )
                {
                    apr = NL_FALSE;
                    break;
                }
            }

            if( apr EQ NL_FALSE )
                break;
        }

        if( j EQ pr )
            break;
    }

    /* Compact curve */

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCRVAPPROXKNOTSTANGENTS: Curve approximation with knots and end tangents          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve approximating  a given
     set of points. Optionally end tangents, parameters and knots may be
     passed in. The  routine  uses a  subset of  the knots  if they are
     suitable  for approximation. These  knots  must  lie in  intervals
     defined by  knots computed by averaging the parameters. If some of 
     these intervals do not contain knots from  the input  knot vector, 
     the  knots  defining the  intervals are  added to  the  input knot 
     vector. If the  output  curve is  initialized  to the NULL  curve, 
     memory is  allocated  locally. Otherwise it is  checked if  enough 
     memory is passed in. A typical calling example is as follows:

       NL_POINT       *P;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  *knt;
       NL_VECTOR      Ts, Te;
       NL_INDEX       k, n;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SC, SK;
       ...
       (get arrays P and u, define knt, choose n, p, Ts and Te);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxKnotsTangents(P,k,u   ,&Ts ,&Te ,NL_TANGENT,&knt,0.7,n,p,&cur,&SC,&SK);
       knt = NULL;
       N_FitCrvApproxKnotsTangents(P,k,NULL,NULL,NULL,NL_TANGENT,&knt,1.0,n,p,&cur,&SC,&SK);


   ACCESS:
   
     P   , input  ,  Points to be approximated
     k   , input  ,  Highest index in P
     u   , input  ,  Parameters corresponding to data points
                       !NULL: parameters passed in
                        NULL: must be computed locally
     Ts  , in/out ,  Start tangent/derivative
                       !NULL: approximate with start tangent
                        NULL: approximate without start tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE WILL BE RESCALED!
     Te  , in/out ,  End tangent/derivative
                       !NULL: approximate with end tangent
                        NULL: approximate without end tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE WILL BE RESCALED!
     der , input  ,  Flag:
                       NL_TANGENT   : Ts and/or Te are  tangent directions
                                   only and must be scaled internally
                       NL_DERIVATIVE: Ts and/or Te are derivatives and are
                                   used as passed in
     knt , in/out ,  Knot vector.  The  double  pointer knt must not be 
                     NULL.  The  single  pointer *knt may or may not be
                     NULL on input.
                       !NULL: choose the knots from this  knot vetor if
                              possible. If  in  certain  positions  new 
                              knots are required, add these to knt.
                        NULL: compute knt internally
     per , input  ,  Percentage of interval usage
                       1.0: the entire interval is  used to  find knots 
                        |   in knt.  This  gives  maximum  use  of knt,
                        |   however,  the approximant  may  not  be  as 
                        |   pleasing as desired.
                       \|/
                       0.0: only one knot may be used. This  gives zero
                            flexibility and produces a  new knt that is 
                            the old knt plus a new knot vector obtained 
                            by parameter averaging.
                     For most data sets per = 1.0 is quite adequate.
     n   , input  ,  Highest index of control  point array of cur. Must
                     satisfy:
                       No  tangent : n < k-3
                       One tangent : n < k-4
                       Two tangents: n < k-5
     p   , input  ,  Degree of approximating curve (p>=2)
     cur , output ,  Approximating curve
     SC  , input  ,  cur's memory stack
     SK  , input  ,  knt's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvApproxKnotsTangents( NL_POINT *P, NL_INDEX k, NL_PARAMETER *u, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG der, NL_KNOTVECTOR ** knt, NL_REAL per, NL_INDEX n, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SC, NL_STACKS *SK )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxKnotsTangents");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jc, mt = 0, mo = 0, ms = 0;

    NL_REAL *ul, *UT = NULL, *UI, *UA, *UP, *US = NULL, *UO, a, b, d, fac;

    NL_DEGREE pt = 0;

    NL_KNOTVECTOR *kni, *kna, *knp, *kno;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check incoming data */

    if( *knt NEQ NULL )
    {
        N_KnotVectorGetKnots( *knt, &mt, &UT );

        for ( pt = 0; pt < mt; pt++ )
        {
            if( UT[pt]NEQ UT[pt + 1] )
                break;
        }

        if( pt GE mt )
            NL_ERROR( NL_INP_ERR );
        error = N_KnotVectorIsValid( *knt, pt, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( per LT 0.0 OR per GT 1.0 )
        NL_ERROR( NL_INP_ERR );

    if( u NEQ NULL AND *knt NEQ NULL )
    {
        if( u[0]NEQ UT[0]OR u[k]NEQ UT[mt] )
            NL_ERROR( NL_INP_ERR );
    }

    /* Compute parameters if not passed in */

    if( u EQ NULL )
    {
        ul = N_AllocReal1dArray( k, &SL );

        if( ul EQ NULL )
            NL_QUIT;

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, ul );

        if( error EQ NL_YES )
            NL_OUT;

        if( *knt NEQ NULL )
        {
            if( ul[0]NEQ UT[0]OR ul[k]NEQ UT[mt] )
            {
                fac = (UT[mt] - UT[0]) / (ul[k] - ul[0]);

                for ( i = 1; i < k; i++ )
                {
                    ul[i] = UT[0] + fac * (ul[i] - ul[0]);
                }

                ul[0] = UT[0];
                ul[k] = UT[mt];
            }
        }
    }
    else
    {
        ul = u;
    }

    /* Get knot vector if none passed in */

    kna = N_AllocKnotVectorAndArray( n + p + 1, &SL );

    if( kna EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( kna, &i, &UA );

    if( *knt EQ NULL )
    {
        error = N_FitCalcKnotVectorCrvApprox( ul, k, n, p, kna );

        if( error EQ NL_YES )
            NL_OUT;

        mo = n + p + 1;
    }

    /* Select knots for approximation */

    if( *knt NEQ NULL )
    {
        US = N_AllocReal1dArray( n + p + 1, &SL );

        if( US EQ NULL )
            NL_QUIT;

        kni = N_AllocKnotVectorAndArray( n + p + 1, &SL );

        if( kni EQ NULL )
            NL_QUIT;

        knp = N_AllocKnotVectorAndArray( n + p, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knp, &i, &UP );
        N_KnotVectorGetKnots( kni, &i, &UI );

        for ( i = 0; i <= p; i++ )
        {
            UA[i] = ul[0];
            UA[n + i + 1] = ul[k];
        }

        /* Get ideal knot vector for degree p */

        error = N_FitCalcKnotVectorCrvApprox( ul, k, n, p, kni );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get ideal knot vector for degree p-1 */

        error = N_FitCalcKnotVectorCrvApprox( ul, k, n, (NL_DEGREE)(p - 1), knp );

        if( error EQ NL_YES )
            NL_OUT;

        /* Select knots from knt */

        j = pt + 1;
        ms = -1;

        for ( i = p + 1; i <= n; i++ )
        {
            a = (1.0 - per) * UI[i] + per * UP[i - 1];
            b = (1.0 - per) * UI[i] + per * UP[i];

            /* See if there are knots in (a,b). If yes, select the */
            /* one closest to UI[i]. If not, add UI[i] to knt.     */

            while( UT[j]LE a )
                j++;

            fac = b - a;
            jc = -1;

            while( UT[j]GT a AND UT[j]LT b )
            {
                d = fabs( UI[i] - UT[j] );

                if( d LT fac )
                {
                    fac = d;
                    jc = j;
                }
                j++;
            }

            if( jc EQ - 1 )
            {
                UA[i] = UI[i];
                US[++ms] = UI[i];
            }
            else
            {
                UA[i] = UT[jc];
            }
        }
    }

    /* Now approximate data points */

    error = N_FitCrvTangentsKnotsParams( (NL_VOID *)P, k, n, p, (NL_VOID *)Ts, (NL_VOID *)Te, NL_EPOINT, der, ul, kna, cur, SC );

    if( error EQ NL_YES )
        NL_OUT;

    /* Update input knot vector */
    if( *knt EQ NULL )
    {
        kno = N_AllocKnotVectorAndArray( mo, SK );

        if( kno EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( kno, &mo, &UO );

        for ( i = 0; i <= mo; i++ )
            UO[i] = UA[i];

        *knt = kno;
    }
    else if( ms GT - 1 )
    {
        mo = mt + ms + 1;

        kno = N_AllocKnotVectorAndArray( mo, SK );

        if( kno EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( kno, &mo, &UO );

        mo = -1;
        i = 0;
        j = 0;

        while( i LE mt AND j LE ms )
        {
            if( UT[i]LT US[j] )
            {
                UO[++mo] = UT[i];
                i++;
            }
            else
            {
                UO[++mo] = US[j];
                j++;
            }
        }

        if( i LE mt )
        {
            while( i LE mt )
            {
                UO[++mo] = UT[i];
                i++;
            }
        }

        N_FreeKnotVector( *knt, SK );

        *knt = kno;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROXTANGENTS: Global curve approximation with end tangents             */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes a least-squares NURBS curve approxi-
     mation to a given set of points.  Optionally end tangents or end
     derivatives may be specified.  If the output curve is initialized
     to the NULL curve, memory is allocated locally.  Otherwise it is
     checked if enough memory is passed in.  A typical calling example:
 
       NL_POINT   *P;
       NL_INDEX   m, n;
       NL_DEGREE  p;
       NL_VECTOR  Ts, Te;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define array P, choose p, n, Ts and Te);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxTangents(P,m,n,p,&Ts ,&Te ,NL_TANGENT   ,NL_CHORDLENGTH,&cur,&SG);
       N_FitCrvApproxTangents(P,m,n,p,NULL,NULL,NL_DERIVATIVE,NL_CHORDLENGTH,&cur,&SG);
 
     A recommended default for the parametrization type is NL_CHORDLENGTH.
 
 
   ACCESS:

     P   , input  ,  Points to be approximated
     m   , input  ,  Highest index in P
     n   , input  ,  Highest index of control point array of cur.  Must satisfy:
                       No  tangent : n < m-3
                       One tangent : n < m-4
                       Two tangents: n < m-5
     p   , input  ,  Degree of approximating curve (must satisfy p<=n)
     Ts  , in/out ,  Start tangent/derivative:
                       !NULL: interpolate with start tangent
                        NULL: interpolate without start tangent
                     IF IT IS A DIRECTION VECTOR AND NOT A DERIVATIVE,
                     ITS MAGNITUDE MAY BE RESCALED!
     Te  , in/out ,  End tangent/derivative
                       !NULL: interpolate with end tangent
                        NULL: interpolate without end tangent
                     IF IT IS A DIRECTION VECTOR AND NOT A DERIVATIVE,
                     ITS MAGNITUDE MAY BE RESCALED!
     der , input  ,  Flag:
                       NL_TANGENT   : Ts and/or Te are  tangent directions
                                      only and must be scaled internally
                       NL_DERIVATIVE: Ts and/or Te are derivatives and are
                                      used as passed in
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCrvApproxTangents( NL_POINT *P, NL_INDEX m, NL_INDEX n, NL_DEGREE p, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG der, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxTangents");

    NL_FLAG error = NL_NO;

    NL_INDEX I[2], i, h;

    NL_REAL *wp, wd[2], len;

    NL_PARAMETER *u;

    NL_VECTOR D[2];

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    if ( n < p )
      { NL_ERROR( NL_INP_ERR ); }

    /* Get parameters and knot vector */

    if( n GT m - 2 )
        NL_ERROR( NL_INP_ERR );

    u = N_AllocReal1dArray( m, &SL );

    if( u EQ NULL )
        NL_QUIT;

    wp = N_AllocReal1dArray( m, &SL );

    if( wp EQ NULL )
        NL_QUIT;

    knt = N_AllocKnotVectorAndArray( n + p + 1, &SL );

    if( knt EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_FitCalcKnotVectorCrvApprox( u, m, n, p, knt );

    if( error EQ NL_YES )
        NL_OUT;

    /* Set up constraint data */

    wp[0] = -1.0;
    wp[m] = -1.0;
    h = -1;

    for ( i = 1; i < m; i++ )
        wp[i] = 1.0;

    if( Ts NEQ NULL OR Te NEQ NULL )
    {
        /* Scale tangents if neccesary */

        if( der EQ NL_TANGENT )
        {
            N_DistPolygon( P, m, &len );
            len = len * (u[m] - u[0]);

            if( Ts NEQ NULL )
            {
                error = N_VectorNormalizeRef( Ts );

                if( error EQ NL_YES )
                    NL_OUT;

                N_VectorScale( *Ts, len, Ts );
            }

            if( Te NEQ NULL )
            {
                error = N_VectorNormalizeRef( Te );

                if( error EQ NL_YES )
                    NL_OUT;

                N_VectorScale( *Te, len, Te );
            }
        }

        /* Set up constraint data */

        if( Ts NEQ NULL AND Te EQ NULL )
        {
            if( n GT m - 3 )
                NL_ERROR( NL_INP_ERR );

            wd[0] = -1.0;
            I[0] = 0;
            h = 0;

            N_VectorCopy( *Ts, &D[0] );
        }
        else if( Ts EQ NULL AND Te NEQ NULL )
        {
            if( n GT m - 3 )
                NL_ERROR( NL_INP_ERR );

            wd[0] = -1.0;
            I[0] = m;
            h = 0;

            N_VectorCopy( *Te, &D[0] );
        }
        else if( Ts NEQ NULL AND Te NEQ NULL )
        {
            if( n GT m - 4 )
                NL_ERROR( NL_INP_ERR );

            wd[0] = -1.0;
            wd[1] = -1.0;
            I[0] = 0;
            I[1] = m;
            h = 1;

            N_VectorCopy( *Ts, &D[0] );
            N_VectorCopy( *Te, &D[1] );
        }
    }

    /* Now fit data */

    error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, m, (NL_VOID *)D, wd, I, h, NL_EPOINT, u, knt, n, p, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCRVAPPROXKNOTS: Curve approximation with specified knot vector           */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes a  NURBS curve approximating a given
     set of points. This  is a general routine requiring the input of a
     specified knot vector, parameters at interpolation points, and the
     choice of NL_POINT or NL_CPOINT data. If the output curve is initialized 
     to the NULL  curve, memory  is allocated  locally. Otherwise it is 
     checked if  enough memory  is passed in. A typical calling example 
     is as follows:
 
       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  knt;
       NL_INDEX       m, n;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get arrays P/Pw and u, define knt, and choose p and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxKnots((NL_VOID *)P ,m,NL_EPOINT,u,&knt,n,p,&cur,&SG);
       N_FitCrvApproxKnots((NL_VOID *)Pw,m,NL_HPOINT,u,&knt,n,p,&cur,&SG);
 
     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT NL_VECTOR ARE COMPUTED
     IN  THE  CALLING   ROUTINE. This  routine  computes  least-squares 
     approximation to data.
 
 
   ACCESS:
   
     A   , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     m   , input  ,  Highest index in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     u   , input  ,  Parameters corresponding to data points
     knt , input  ,  Knot vector of approximating curve
     n   , input  ,  Highest index of approximating curve : n >= p.
     p   , input  ,  Degree of approximating curve
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCrvApproxKnots( NL_VOID *A, NL_INDEX m, NL_FLAG ptp, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_INDEX n, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, l, bw, sbw, rj, sj, ej, lk, hk, lj, hj, mk, mc;

    NL_REAL ** B, ** NTN, *N, *UC, *UK, n0, np;

    NL_POINT *Rk, *R, *Q;

    NL_CPOINT *Pw, *Qw, *Rkw, *Rw;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    if ( n < p )
      { NL_ERROR( NL_INP_ERR ); }

    N_KnotVectorGetKnots( knt, &mk, &UK );

    /* Check curve memory */
    mc = n + p + 1;

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= mc; i++ )
        UC[i] = UK[i];

    if( n < 2 ) /* just a line between 2 points */
    {
        if( ptp == NL_EPOINT )
        {
            Q = (NL_POINT *)A;
            N_PtToCPt( Q[0], &Pw[0] ); /* gwc - switch argument order form N_CPtToPt(Pw[0], &Q[0]); */
            N_PtToCPt( Q[1], &Pw[1] ); /*                                  N_CPtToPt(Pw[1], &Q[1]); */
        }
        else
        {
            Qw = (NL_CPOINT *)A;
            Pw[0] = Qw[0];
            Pw[1] = Qw[1];
        }
        NL_OUT;
    }

    /* Check error */

    if( mk NEQ mc )
        NL_ERROR( NL_INP_ERR );

    if( n LT 2 OR p GT n OR m LT n )
        NL_ERROR( NL_INP_ERR );

    /* Compute coefficient matrix */

    bw = 2 * p + 1;
    sbw = p;
    rj = p;
    sj = p - 1;
    ej = -2;

    B = N_AllocReal2dArray( m - 2, p, &SL );

    if( B EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    index = N_AllocInt1dArray( m - 2, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( n - 2, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( n - 2, &SL );

    if( end EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n - 2, n - 2, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &NTN );

    for ( i = 0; i <= n - 2; i++ )
    {
        for ( j = 0; j < bw; j++ )
            NTN[i][j] = 0.0;
    }

    for ( i = 0; i <= m - 2; i++ )
        B[i][p] = 0.0;

    for ( i = 0; i <= NL_MIN( p - 1, n - 2 ); i++ )
        start[i] = 0;

    end[0] = -2;

    for ( i = 1; i <= m - 1; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        if( j EQ p )
            l = 1;
        else
            l = 0;

        if( j EQ p OR j EQ n )
            hk = p - 1;
        else
            hk = p;

        for ( k = 0; k <= hk; k++ )
            B[i - 1][k] = N[l + k];

        index[i - 1] = NL_MAX( 0, j - p - 1 );

        if( j GT rj )
        {
            for ( k = 1; k <= j - rj; k++ )
            {
                sj++;
                ej++;

                if( sj LE n - 2 )
                    start[sj] = i - 1;

                if( ej GE 0 )
                    end[ej] = i - 2;
            }
            rj = j;
        }
    }

    if( sj LT n - 2 OR end[0]EQ - 1 )
        NL_ERROR( NL_INP_ERR );

    for ( i = NL_MAX( 0, ej + 1 ); i <= n - 2; i++ )
        end[i] = m - 2;

    for ( i = 0; i <= n - 2; i++ )
    {
        lj = NL_MAX( 0, i - p );
        hj = NL_MIN( n - 2, i + p );

        for ( j = lj; j <= hj; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( k = lk; k <= hk; k++ )
            {
                NTN[i][j - i + sbw] += B[k][i - index[k]] * B[k][j - index[k]];
            }
        }
    }

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Setup and solve system of equations */

    switch( ptp )
    {
        case NL_EPOINT:

            Q = (NL_POINT *)A;

            N_PtToCPt( Q[0], &Pw[0] );
            N_PtToCPt( Q[m], &Pw[n] );

            Rk = N_AllocPt1dArray( m - 2, &SL );

            if( Rk EQ NULL )
                NL_QUIT;

            for ( i = 1; i <= m - 1; i++ )
            {
                error = N_BasisIEval( knt, 0, p, u[i], NL_LEFT, &n0 );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_BasisIEval( knt, n, p, u[i], NL_LEFT, &np );

                if( error EQ NL_YES )
                    NL_OUT;

                N_TranslateSum2Pts( Q[i], -n0, Q[0], -np, Q[m], &Rk[i - 1] );
            }

            R = N_AllocPt1dArray( n - 2, &SL );

            if( R EQ NULL )
                NL_QUIT;

            for ( i = 1; i <= n - 1; i++ )
            {
                N_CopyPt( NL_ZERO, &R[i - 1] );

                lk = start[i - 1];
                hk = end[i - 1];

                for ( k = lk; k <= hk; k++ )
                {
                    N_VectorBlendPt( B[k][i - index[k] - 1], Rk[k], &R[i - 1] );
                }
            }

            error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, &Pw[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        case NL_HPOINT:

            Qw = (NL_CPOINT *)A;

            N_CopyCPt( Qw[0], &Pw[0] );
            N_CopyCPt( Qw[m], &Pw[n] );

            Rkw = N_AllocCPt1dArray( m - 2, &SL );

            if( Rkw EQ NULL )
                NL_QUIT;

            for ( i = 1; i <= m - 1; i++ )
            {
                error = N_BasisIEval( knt, 0, p, u[i], NL_LEFT, &n0 );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_BasisIEval( knt, n, p, u[i], NL_LEFT, &np );

                if( error EQ NL_YES )
                    NL_OUT;

                N_TranslateSum2CPts( Qw[i], -n0, Qw[0], -np, Qw[m], &Rkw[i - 1] );
            }

            Rw = N_AllocCPt1dArray( n - 2, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( i = 1; i <= n - 1; i++ )
            {
                N_CopyCPt( NL_CZERO, &Rw[i - 1] );

                lk = start[i - 1];
                hk = end[i - 1];

                for ( k = lk; k <= hk; k++ )
                {
                    N_VectorBlendCPt( B[k][i - index[k] - 1], Rkw[k], &Rw[i - 1] );
                }
            }

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Rw, NL_HPOINT, &Pw[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVARCAPPROX: Data approximation with piecewise biarc segments         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes a quadratic NURBS biarc approxima-
     ting a given set of points. This is a local method producing a NL_G1
     continuous piecewise circular curve. If tangent data is available 
     at each  data point, they  can  be passed in. Otherwise  tangents 
     are computed  either locally  or via a base  curve. If the output 
     curve  is  initialized  to the  NULL  curve, memory  is allocated 
     locally. Otherwise  it is  checked if enough memory is passed in. 
     A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k, maxk;
       NL_REAL    E;
       NL_VECTOR  *T,
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P, T, and choose maxk and E);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvArcApprox(P,k,T,NL_BASECURVE,maxk,E,&cur,&SG);

     If cur is initialized to NULL, memory is  allocated twice. First,
     enough memory is allocated for worst-case approximation, i.e. for
     interpolation.  After  approximation  is   completed,  memory  is 
     reallocated using the proper number of control  points and knots.


   ACCESS:
   
     P    , input  ,  Points to be approximated
     k    , input  ,  Highest index in P
     T    , input  ,  Tangents  at each  point (optional, used only if
                      tan=NL_PASSEDIN)
     tan  , input  ,  Flag:
                        NL_PASSEDIN:    Tangents are used as passed in
                        NL_LOCALMETHOD: A local method is used to compute
                                     tangents at each point
                        NL_BASECURVE:   A base curve  is fitted to obtain
                                     the tangents
                      IF A BASE NL_CURVE IS NL_USED, THE CIRCLE SEGMENTS ARE
                      NOT  ANCHORED AT  THE DATA  NL_POINTS, HOWEVER, THE
                      NL_CIRCULAR ARCS  PASS NEAR  P[i]. IF  TANGENTS ARE
                      PASSED  IN OR  COMPUTED  BY A LOCAL  METHOD, THE 
                      CIRCLE SEGMENTS  PASS THROUGH  A SUBSET  OF DATA 
                      NL_POINTS!
     maxk , input  ,  Maximum number of points to be  approximated by
                      one segment
     E    , input  ,  Approximation tolerance (NL_ABSOLUTE TOLERANCE!)
     cur  , output ,  Approximating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvArcApprox( NL_POINT *P, NL_INDEX k, NL_VECTOR *T, NL_FLAG tan, NL_INDEX maxk, NL_REAL E, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvArcApprox");

    NL_FLAG reset = NL_NO, error = NL_NO, closed, apr, rer, aef;

    NL_INDEX i, j, l, le=0, n, m, ks, ke, kl, kr, nu, nv, dn, ITLIM_SAV = 0;

    NL_REAL *U, *er, alf, oma, a, b, d, xd, yd, zd, lp, pp, Ed, maxe, top;

    NL_PARAMETER *u;

    NL_POINT *B, *Q, C, CD[2];

    NL_CPOINT *Pw, Bw[18];

    NL_VECTOR *S, A;

    NL_KNOTVECTOR *knt;

    NL_MINMAXBOX box;

    NL_CURVE curB;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL toc = 1.0e-06;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    n = 8 * k;
    error = N_CrvSizeArrays( cur, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get some memory and set parameters */

    B = N_AllocPt1dArray( k, &SL );

    if( B EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    if( tan NEQ NL_PASSEDIN )
    {
        T = N_AllocPt1dArray( k, &SL );

        if( T EQ NULL )
            NL_QUIT;
    }

    if( tan NEQ NL_BASECURVE )
    {
        for ( i = 0; i <= k; i++ )
            N_VectorCopy( P[i], &B[i] );
    }

    N_DistPtPt( P[0], P[k], &d );

    if( d LT NL_MTOL )
        closed = NL_YES;
    else
        closed = NL_NO;

    ITLIM_SAV = NL_ITLIM;
    NL_ITLIM = 10;

    /* Compute the tangents if needed */
    if( tan EQ NL_LOCALMETHOD )
    {
        S = N_AllocPt1dArray( k + 3, &SL );

        if( S EQ NULL )
            NL_QUIT;

        for ( i = 1; i <= k; i++ )
            N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

        if( closed EQ NL_YES )
        {
            N_VectorCopy( S[k + 1], &S[1] );
            N_VectorCopy( S[k], &S[0] );
            N_VectorCopy( S[2], &S[k + 2] );
            N_VectorCopy( S[3], &S[k + 3] );
        }
        else
        {
            N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
            N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
            N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
            N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
        }

        for ( i = 0; i <= k; i++ )
        {
            N_VectorCross( S[i], S[i + 1], &A );
            N_VectorMagnitude( A, &a );
            N_VectorCross( S[i + 2], S[i + 3], &A );
            N_VectorMagnitude( A, &b );

            if( (a + b)GT NL_LTOL )
                alf = a / (a + b);
            else
                alf = 0.5;

            oma = 1.0 - alf;
            N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
        }
    }

    if( tan EQ NL_BASECURVE )
    {
        /* Get min-max box data and point set length */

        N_Pts1dCalcBBox( P, k, &box );
        N_BBoxGetDiagonal( &box, &d );
        N_BBoxGetDimensions( &box, &xd, &yd, &zd );
        N_DistPolygon( P, k, &lp );

        /* Estimate n */

        pp = 2.0 *( xd + yd );
        nu = 4;
        nv = 4;

        if( xd GT yd )
            nu = (NL_INDEX)(4.0 *xd / yd);
        else
            nv = (NL_INDEX)(4.0 *yd / xd);

        a = NL_MAX( 1.0, log( d / E ) );
        b = NL_MAX( 16.0, (2 * lp * ((NL_REAL)nu + (NL_REAL)nv)) / pp );
        n = NL_MIN( k, (NL_INDEX)(a * b) );
        dn = NL_MAX( 1, n / 2 );

        /* Fit curve to the data points */

        knt = N_AllocKnotVectorAndArray( k + 4, &SL );

        if( knt EQ NULL )
            NL_QUIT;

        er = N_AllocReal1dArray( k, &SL );

        if( er EQ NULL )
            NL_QUIT;

        error = N_AllocCrvArrays( &curB, k, 3, k + 4, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        top = 0.01 *E;

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, u );

        if( error EQ NL_YES )
            NL_OUT;

        do
        {
            /* Get a fit */

            if( closed EQ NL_YES )
            {
                error = N_FitCalcKnotsEndDerivs( u, k, n, 3, 2, 2, knt );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_FitCrvClosedDerivsParamsKnots( (NL_VOID *)P, k, NL_EPOINT, NULL, 2, u, knt, n, 3, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                error = N_FitCalcKnotsEndDerivs( u, k, n, 3, 0, 0, knt );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_FitCrvDerivsKnots( (NL_VOID *)P, k, NL_EPOINT, NULL, 0, NULL, 0, u, knt, n, 3, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* Do quick error check */

            apr = NL_YES;
            maxe = 0.0;

            for ( i = 1; i < k; i++ )
            {
                error = N_CrvEval( &curB, u[i], NL_LEFT, &C );

                if( error EQ NL_YES )
                    NL_OUT;

                N_DistPtPt( C, P[i], &er[i] );

                if( er[i]GT E )
                {
                    apr = NL_NO;
                    break;
                }

                if( er[i]GT maxe )
                    maxe = er[i];
            }

            /* If approximation is acceptable */

            if( apr EQ NL_YES )
            {
                /* Remove unnecessary knots */

                Ed = E - maxe;

                if( closed EQ NL_YES )
                {
                    error = N_FitRemoveKnotsAndDerivs( &curB, u, er, k, Ed, NL_BOTH, 2 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitRemoveKnots( &curB, u, er, k, Ed );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                /* Do parameter correction */

                for ( i = 1; i < k; i++ )
                {
                    error = N_CrvClosestPt( &curB, P[i], u[i], top, toc, &u[i], &C );

                    if( error EQ NL_YES )
                    {
                        rer = (NL_FLAG)N_ErrGetType();

                        if( rer EQ NL_CON_ERR )
                            N_ErrClear();
                        else
                            NL_OUT;
                    }
                }

                /* Done - break out of the loop */

                break;
            }
            else
            {
                /* Approximation is not acceptable - increase */
                /* number of control points and reapproximate */

                n = NL_MIN( n + dn, (n + k + 1) / 2 );

                N_CrvSetSizeIndices( &curB, k, 3, k + 4 );
                N_SetKnotIndex( knt, k + 4 );
            }
        } while ( n LT k );

        /* Got base curve - compute points and tangents */

        for ( i = 0; i <= k; i++ )
        {
            error = N_CrvDerivs( &curB, u[i], NL_LEFT, 1, CD );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorCopy( CD[0], &B[i] );
            N_VectorCopy( CD[1], &T[i] );
        }
    }

    /* Approximate data */

    ks = 0;
    n = 0;

    while( ks LT k )
    {
        if( closed EQ NL_NO )
            ke = NL_MIN( (ks + maxk), k );
        else
            ke = k / 2;

        kl = ks;
        kr = ke;

        while( kl NEQ ke )
        {
            /* Get a biarc */

            error = N_FitArcToEndPtsAndTangents( B[ks], T[ks], B[ke], T[ke], Bw, &l );

            if( error EQ NL_YES )
                NL_OUT;

            /* Check error of approximation */

            error = N_CrvArePtsWithinTol( Bw, l, &P[ks], ke - ks, E, &aef );

            if( error EQ NL_YES )
                NL_OUT;

            if( aef EQ NL_YES )
            {
                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Bw[j], &Pw[n + j] );

                kl = ke;
                le = l;
            }
            else
                kr = ke;

            ke = (kl + kr) / 2;
        }

        n += le;
        ks = ke;
        closed = NL_NO;
    }

    N_CrvSetSizeIndices( cur, n, 2, n + 3 );

    /* Compute the knot vector */

    l = n / 2;

    u = N_AllocReal1dArray( l, &SL );

    if( u EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( l, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= l; i++ )
    {
        j = 2 * i;

        N_CPtToPtEuclid( Pw[j], &Q[i] );
    }

    error = N_FitCalcCrvParamValues( (NL_VOID *)Q, l, NL_EPOINT, NL_CHORDLENGTH, u );

    if( error EQ NL_YES )
        NL_OUT;

    m = -1;

    for ( i = 0; i <= 2; i++ )
        U[++m] = u[0];

    for ( i = 1; i < l; i++ )
    {
        U[++m] = u[i];
        U[++m] = u[i];
    }

    for ( i = 0; i <= 2; i++ )
        U[++m] = u[l];

    /* Compact curve */

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    NL_ITLIM = ITLIM_SAV;

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVCUBICAPPROX: Data approximation with piecewise cubic segments         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes a cubic NURBS curve  approximating
     a  given set  of points. This  is a local  method  producing a NL_G1
     or NL_C1 continuous piecewise cubic curve. If tangent data is avail-
     able at each  data point, they can  be passed in. Otherwise  tan-
     gents are computed locally. If the output curve is initialized to
     the NULL  curve, memory  is allocated  locally.  Otherwise  it is
     checked if enough memory is passed in.  A typical calling example
     is:

       NL_POINT   *P;
       NL_INDEX   k, maxk;
       NL_REAL    E;
       NL_VECTOR  *T,
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P, T, and choose maxk and E);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvCubicApprox(P,k,T,NL_YES,NL_G1,maxk,E,&cur,&SG);

     If cur is initialized to NULL, memory is  allocated twice. First,
     enough memory is allocated for worst-case approximation, i.e. for
     interpolation.  After  approximation  is   completed,  memory  is 
     reallocated using the proper number of control  points and knots.
     THE SPEED OF THE  SEARCH PROCESS DEPENDS ON maxk. A GOOD ESTIMATE
     IS TO SET maxk = 20% OF THE NUMBER OF NL_POINTS which typically 
     yields an NL_ERROR of about 1% TOLERANCE. FOR  EXAMPLE, IF  THE  
     NL_CURVE  IS IN THE [0,1] BOX, THE NUMBER OF DATA NL_POINTS IS 1000, 
     AND THE DESIRED NL_ERROR IS 0.01, set maxk = 200 as a good estimate. 
     FOR  SMALLER NL_ERROR TOLERANCES maxk decrease maxk PROPORTIONALLY.


   ACCESS:
   
     P    , input  ,  Points to be approximated
     k    , input  ,  Highest index in P
     T    , input  ,  Tangents at each point
     tan  , input  ,  Flag:
                        NL_YES: Compute tangents
                        NL_NO : Tangents are passed in
     cont , input  ,  Flag:
                        NL_G1 : the curve will  be  NL_G1-continuous  (with
                             triple knots).  A  relative  chordlength
                             based parameterization is used
                        NL_C1 : the curve will  be  NL_C1-continuous  (with
                             double knots)
     maxk , input  ,  Maximum number of points to be  approximated by
                      one segment
     E    , input  ,  Approximation tolerance
     cur  , output ,  Approximating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvCubicApprox( NL_POINT *P, NL_INDEX k, NL_VECTOR *T, NL_FLAG tan, NL_FLAG cont, NL_INDEX maxk, NL_REAL E, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvCubicApprox");

    NL_FLAG reset = NL_NO, error = NL_NO, closed = NL_NO;

    NL_INDEX i, l, n, m, ks, ke, kl, kr, ii, j1, j2;

    NL_REAL *U, alf, oma, a, b, d, uu;

    NL_PARAMETER *u;

    NL_CPOINT *Pw;

    NL_POINT P1, P2, P3;

    NL_VECTOR *S, A;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL top = 1.0e-04;
    NL_PRIVATE NL_REAL toc = 1.0e-06;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    n = 3 * k;
    error = N_CrvSizeArrays( cur, n, 3, n + 4, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Compute parameters, and tangents if needed */

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, u );

    if( error EQ NL_YES )
        NL_OUT;

    if( tan EQ NL_YES )
    {
        S = N_AllocPt1dArray( k + 3, &SL );

        if( S EQ NULL )
            NL_QUIT;

        T = N_AllocPt1dArray( k, &SL );

        if( T EQ NULL )
            NL_QUIT;

        N_DistPtPt( P[0], P[k], &d );

        if( d LT NL_MTOL )
            closed = NL_YES;
        else
            closed = NL_NO;

        for ( i = 1; i <= k; i++ )
            N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

        if( closed EQ NL_YES )
        {
            N_VectorCopy( S[k + 1], &S[1] );
            N_VectorCopy( S[k], &S[0] );
            N_VectorCopy( S[2], &S[k + 2] );
            N_VectorCopy( S[3], &S[k + 3] );
        }
        else
        {
            N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
            N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
            N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
            N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
        }

        for ( i = 0; i <= k; i++ )
        {
            N_VectorCross( S[i], S[i + 1], &A );
            N_VectorMagnitude( A, &a );
            N_VectorCross( S[i + 2], S[i + 3], &A );
            N_VectorMagnitude( A, &b );

            if( (a + b)GT NL_LTOL )
                alf = a / (a + b);
            else
                alf = 0.5;

            oma = 1.0 - alf;
            N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );

            error = N_VectorNormalizeRef( &T[i] );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Approximate data */

    ks = 0;
    n = 0;
    m = -1;

    for ( i = 0; i <= 3; i++ )
        U[++m] = 0.0;

    while( ks LT k )
    {
        if( maxk LT k )
            ke = NL_MIN( (ks + maxk), k );

        else if( closed EQ NL_NO )
            ke = NL_MIN( (ks + maxk), k );

        else
            ke = k / 2;

        kl = ks;
        kr = ke;

        while( kl NEQ ke )
        {
            error = N_FitLocalCubicApprox( P, k, T, ks, ke, u, E, top, toc, &Pw[n], &l );

            if( error EQ NL_YES )
                NL_OUT;

            if( l GT 0 )
                kl = ke;
            else
                kr = ke;

            ke = (kl + kr) / 2;
        }

        if( ke LT k )
        {
            U[++m] = u[ke];
            U[++m] = u[ke];
            U[++m] = u[ke];
        }

        n += 3;
        ks = ke;
        closed = NL_NO;
    }

    N_CrvSetSizeIndices( cur, n, 3, n + 4 );

    for ( i = 0; i <= 3; i++ )
        U[++m] = 1.0;

    if( cont EQ NL_C1 AND n GT 3 )
    {
        U[4] = U[5] = U[6] = 1.0;

        ii = 5; /* new knot index */
        j1 = 2; /* new ctrl pnt index */

        j2 = 3; /* old ctrl pnt index */

        while( j2 LT n )
        {
            N_CPtToPtEuclid( Pw[j2 - 1], &P1 );
            N_CPtToPtEuclid( Pw[j2], &P2 );
            N_CPtToPtEuclid( Pw[j2 + 1], &P3 );

            N_DistPtPt( P1, P2, &a );
            N_DistPtPt( P2, P3, &b );

            if( N_FloatOpIsBad( b, a, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            uu = U[ii] + (U[ii] - U[ii - 2]) * (b / a);

            Pw[++j1] = Pw[++j2];
            Pw[++j1] = Pw[++j2];

            j2 += 1;

            U[++ii] = uu;
            U[++ii] = uu;
        }

        Pw[++j1] = Pw[n];

        U[ii + 1] = U[ii];
        U[ii + 2] = U[ii];

        N_CrvSetSizeIndices( cur, j1, 3, j1 + 4 );

        N_CrvReparam( cur, 0.0, 1.0 );
    }

    /* Compact curve */

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

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
/* N_FITCRVDERIVSMATRIX: Curve interpolation with end derivatives and matrix      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes the control points of a NURBS curve  
     interpolating a given set of points and two end derivatives. This 
     is a special routine  requiring a knot  vector, the LU-decomposed
     interpolation matrix, and NL_POINT or NL_CPOINT data. A typical calling 
     example:

       NL_POINT       *Q;
       NL_CPOINT      *Qw, *Pw;
       NL_KNOTVECTOR  knt;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_VECTOR      *Vs, *Ve;
       NL_CVECTOR     *Ws, *We;
       NL_RMATRIX     cm;
       ...
       (get Q/Qw, Vs, Ve, Ws, We, define knt and get cm);
       ...
       N_FitCrvDerivsMatrix((NL_VOID *)Q ,k,NL_EPOINT,p,&knt,(NL_VOID *)&Vs,(NL_VOID *)&Ve,Pw);
       N_FitCrvDerivsMatrix((NL_VOID *)Qw,k,NL_HPOINT,p,&knt,(NL_VOID *)&Ws,(NL_VOID *)&We,Pw);

     IT IS ASSUMED THAT MEMORY TO STORE Pw IS ALLOCATED IN THE CALLING
     ROUTINE!!


   ACCESS:
   
     A     , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     k     , input  ,  Highest index in A
     ptp   , input  ,  Flag:
                         NL_EPOINT: Euclidean point pointer passed in
                         NL_HPOINT: Homogeneous point pointer passed in
     p     , input  ,  Degree of  interpolating curve (must be <= k+2)
     knt   , input  ,  Knot vector of interpolating curve
     As,Ae , input  ,  NL_VOID pointer  representing NL_VECTOR or NL_CVECTOR as 
                       start and end  derivatives. THEY MUST BE PASSED
                       IN BY REFERENCE AS THIS IS THE ONLY WAY TO CAST 
                       THEM TO NL_VOID.
     cm    , input  ,  LU-decomposed interpolation matrix
     Pw    , output ,  Control  points  of  interpolating  curve. MUST 
                       HAVE ENOUGH MEMORY TO HOLD Pw[0],...,Pw[k+2]!!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvDerivsMatrix( NL_VOID *A, NL_INDEX k, NL_FLAG ptp, NL_DEGREE p, NL_KNOTVECTOR *knt, NL_VOID *As, NL_VOID *Ae, NL_RMATRIX *cm, NL_CPOINT *Pw )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvDerivsMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m;

    NL_REAL *U, fact;

    NL_POINT *Q, *R;

    NL_CPOINT *Qw, *Rw;

    NL_VECTOR Vs, Ve;

    NL_CVECTOR Ws, We;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get knot vector data */

    N_KnotVectorGetKnots( knt, &m, &U );
    n = k + 2;

    /* Solve system of linear equations */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)A;
            Vs = *(NL_VECTOR *)As;
            Ve = *(NL_VECTOR *)Ae;

            Q = N_AllocPt1dArray( n, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyPt( R[0], &Q[0] );
            N_ScalePt( fact, Vs, &Q[1] );

            for ( i = 2; i <= k; i++ )
                N_CopyPt( R[i - 1], &Q[i] );

            fact = (U[n + p] - U[n]) / p;
            N_ScalePt( fact, Ve, &Q[n - 1] );
            N_CopyPt( R[k], &Q[n] );

            error = N_RealMatrixForBack( cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            Ws = *(NL_CVECTOR *)As;
            We = *(NL_CVECTOR *)Ae;

            Qw = N_AllocCPt1dArray( n, &SL );

            if( Qw EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyCPt( Rw[0], &Qw[0] );
            N_ScaleCPt( fact, Ws, &Qw[1] );

            for ( i = 2; i <= k; i++ )
                N_CopyCPt( Rw[i - 1], &Qw[i] );

            fact = (U[n + p] - U[n]) / p;
            N_ScaleCPt( fact, We, &Qw[n - 1] );
            N_CopyCPt( Rw[k], &Qw[n] );

            error = N_RealMatrixForBack( cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROXKNOTSTOL: Curve approximation with error bound and knot vector     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes a NURBS  curve approximating a given
     set of points to  user given tolerance. Optionally end tangents and 
     and a knot vector knt may be passed in. The routine attempts to use 
     as many knots of  knt as possible. A typical calling example is  as 
     follows:

       NL_POINT       *P;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_REAL        eps;
       NL_VECTOR      Ts, Te;
       NL_KNOTVECTOR  *knt;
       NL_CURVE       cur;
       NL_STACKS      SC, SK;
       ...
       (get array P, choose p, eps, Ts, Te and knt);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxKnotsTol(P,k,p,&Ts ,&Te ,NL_TANGENT   ,eps,&knt,NL_YES,&cur,&SC,&SK);
       knt = NULL;
       N_FitCrvApproxKnotsTol(P,k,p,NULL,NULL,NL_DERIVATIVE,eps,&knt,NL_NO ,&cur,&SC,&SK);

     IF THE NL_POINT DATA IS RELATIVELY SMOOTH, THE  ROUTINE MAY START WITH
     INTERPOLATING  P[0],...,P[k], FOLLOWED BY  REMOVING KNOTS. HOWEVER, 
     IF THE DATA IS NOISY, AN APPROXIMATING  NL_CURVE IS COMPUTED FIRST AND
     ALL KNOTS  ARE REMOVED  NEXT. THESE OPTIONS  ARE CONTROLLED  BY THE 
     NL_FLAG ffl (SEE BELOW).


   ACCESS:
   
     P   , input  ,  Point array to be approximated
     k   , input  ,  Highest index in P
     p   , input  ,  Degree of approximating curve (2<=p<=k)
     Ts  , in/out ,  Start tangent/derivative
                       !NULL: approximate with start tangent
                        NULL: approximate without start tangent
                     IF IT IS A DIRECTION  NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE WILL BE RESCALED!
     Te  , in/out ,  End tangent/derivative
                       !NULL: approximate with end tangent
                        NULL: approximate without end tangent
                     IF IT IS A DIRECTION  NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE WILL BE RESCALED!
     tfl , input  ,  Flag:
                       NL_TANGENT   : Ts  and/or Te are  tangent directions
                                   only and must be scaled internally
                       NL_DERIVATIVE: Ts  and/or Te are derivatives and are
                                   used as passed in
     eps , input  ,  Error tolerance
     knt , in/out ,  Knot vector.  The  double  pointer knt must  not be 
                     NULL.  The  single  pointer *knt may or may  not be
                     NULL on input.
                       !NULL: choose  the knots from this  knot vetor if
                              possible.  If  in  certain  positions  new 
                              knots are required, add these to knt.
                        NULL: compute knt internally
     ffl , input  ,  Flag:
                       NL_YES: fair point  data,  i.e.  approximate  before
                            knot removal.  This option is recommended if
                            the point data is "noisy"
                       NL_NO : do not fair data,  i.e.  interpolate  before 
                            knot removal.  This option is recommended if
                            the point data is precise
     cur , output ,  Approximating curve
     SC  , input  ,  cur's memory stack
     SK  , input  ,  knt's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvApproxKnotsTol( NL_POINT *P, NL_INDEX k, NL_DEGREE p, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG tfl, NL_REAL eps, NL_KNOTVECTOR ** knt, NL_FLAG ffl, NL_CURVE *cur, NL_STACKS *SC, NL_STACKS *SK )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxKnotsTol");

    NL_FLAG apr, whr = NL_NO, ink, error = NL_NO;

    NL_INDEX *pr, i, j, jh, m, ml, mt = 0, n, nh = 0, nl = 0, dn, etp, der, ITLIM_SAV = 0;

    NL_REAL *U, *UT = NULL, *UH = NULL, *UL, *er, top, fac;

    NL_PARAMETER *u, uc;

    NL_POINT Q;

    NL_KNOTVECTOR *knl;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL toc = 1.0e-05;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input data */

    if( p LT 2 OR p GT k )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, k + 2, p, k + p + 3, rname, SC );

    if( error EQ NL_YES )
        NL_OUT;

    ink = NL_NO;

    if( *knt NEQ NULL )
    {
        error = N_KnotVectorIsValid( *knt, p, rname );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( *knt, &mt, &UT );

        UH = N_AllocReal1dArray( mt, &SL );

        if( UH EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= mt; i++ )
            UH[i] = UT[i];

        knl = N_AllocKnotVectorAndArray( mt, &SL );

        if( knl EQ NULL )
            NL_QUIT;

        error = N_KnotsCopy( *knt, &knl, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        ink = NL_YES;
    }
    else
    {
        knl = NULL;
    }

    /* Initialize and allocate memory */

    ITLIM_SAV = NL_ITLIM;
    NL_ITLIM = 10;
    top = 0.01 *eps;

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    er = N_AllocReal1dArray( k, &SL );

    if( er EQ NULL )
        NL_QUIT;

    pr = N_AllocInt1dArray( k + p + 3, &SL );

    if( pr EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, u );

    if( error EQ NL_YES )
        NL_OUT;

    if( ink EQ NL_YES )
    {
        if( u[0]NEQ UT[0]OR u[k]NEQ UT[mt] )
        {
            fac = (UT[mt] - UT[0]) / (u[k] - u[0]);

            for ( i = 1; i < k; i++ )
            {
                u[i] = UT[0] + fac * (u[i] - u[0]);
            }

            u[0] = UT[0];
            u[k] = UT[mt];
        }
    }

    for ( i = 0; i <= k; i++ )
        er[i] = 0.0;

    if( Ts EQ NULL AND Te EQ NULL )
    {
        whr = NL_NO;
        nh = k;
        nl = k - 3;
    }
    else if( Ts NEQ NULL AND Te EQ NULL )
    {
        whr = NL_START;
        nh = k + 1;
        nl = k - 4;
    }
    else if( Ts EQ NULL AND Te NEQ NULL )
    {
        whr = NL_END;
        nh = k + 1;
        nl = k - 4;
    }
    else if( Ts NEQ NULL AND Te NEQ NULL )
    {
        whr = NL_BOTH;
        nh = k + 2;
        nl = k - 5;
    }

    if( whr EQ NL_NO )
        der = 0;
    else
        der = 1;

    /* Interpolate and knot remove first */

    error = N_FitCrvKnotsAndTangents( P, k, u, Ts, Te, tfl, &knl, 1.0, p, cur, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( cur, &m, &U );
    N_CrvGetArraySizes( cur, &n, &m );

    for ( i = 0; i <= p; i++ )
        pr[i] = pr[n + i + 1] = 0;

    if( ink EQ NL_NO )
    {
        for ( i = p + 1; i <= n; i++ )
            pr[i] = 1;
    }
    else
    {
        j = jh = p + 1;

        while( j LE n )
        {
            if( UH[jh]EQ U[j] )
                pr[j] = 2;
            else
                pr[j] = 1;

            if( UH[jh]EQ U[j] )
            {
                jh++;
                j++;
            }

            else if( UH[jh]LT U[j] )
                jh++;

            else if( UH[jh]GT U[j] )
                j++;
        }
    }

    error = N_FitRemoveKnotsPriorities( cur, pr, u, er, k, eps, whr, der );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetArraySizes( cur, &n, &m );

    if( ffl EQ NL_NO OR n EQ nh )
        goto OUTPUT;

    /* Start with approximation and then remove knots */
    N_CrvSetSizeIndices( cur, k + 2, p, k + p + 3 );

    dn = NL_MAX( 1, n / 2 );

    if( ink EQ NL_YES )
    {
        error = N_KnotsCopy( *knt, &knl, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        knl = NULL;
    }

    do
    {
        /* Approximate with n+1 number of control points */

        if( n GE nl )
        {
            error = N_FitCrvKnotsAndTangents( P, k, u, Ts, Te, tfl, &knl, 1.0, p, cur, SC, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            error = N_FitCrvApproxKnotsTangents( P, k, u, Ts, Te, tfl, &knl, 1.0, n, p, cur, SC, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* Check error */

        apr = NL_YES;

        for ( i = 1; i < k; i++ )
        {
            error = N_CrvClosestPt( cur, P[i], u[i], top, toc, &uc, &Q );

            if( error EQ NL_YES )
            {
                etp = N_ErrGetType();

                if( etp EQ NL_CON_ERR )
                    N_ErrClear();
                else
                    NL_OUT;
            }

            N_DistPtPt( P[i], Q, &er[i] );

            if( er[i]GT eps )
            {
                apr = NL_NO;
                break;
            }
        }

        if( apr EQ NL_YES )
            break;

        /* Error test did not pass. Increase number of control points */

        if( n GE nl )
            break;

        n = NL_MIN( n + dn, (n + k + 1) / 2 );
        n = NL_MIN( n, nl );

        for ( i = 1; i < k; i++ )
            er[i] = 0.0;

        N_CrvSetSizeIndices( cur, k + 2, p, k + p + 3 );

        if( ink EQ NL_YES )
        {
            error = N_KnotsCopy( *knt, &knl, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            N_FreeKnotVector( knl, &SL );
            knl = NULL;
        }
    } while ( n LT k );

    /* Remove knots */

    N_CrvGetKnots( cur, &m, &U );

    for ( i = 0; i <= p; i++ )
        pr[i] = pr[n + i + 1] = 0;

    if( ink EQ NL_NO )
    {
        for ( i = p + 1; i <= n; i++ )
            pr[i] = 1;
    }
    else
    {
        j = jh = p + 1;

        while( j LE n )
        {
            if( UH[jh]EQ U[j] )
                pr[j] = 2;
            else
                pr[j] = 1;

            if( UH[jh]EQ U[j] )
            {
                jh++;
                j++;
            }

            else if( UH[jh]LT U[j] )
                jh++;

            else if( UH[jh]GT U[j] )
                j++;
        }
    }

    nh = n;

    error = N_FitRemoveKnotsPriorities( cur, pr, u, er, k, eps, whr, der );

    if( error EQ NL_YES )
        NL_OUT;

    OUTPUT:

    N_CrvGetArraySizes( cur, &n, &m );
    N_CrvGetKnots( cur, &m, &U );

    /* Merge knot vectors */
    if( ink EQ NL_NO )
    {
        knl = N_AllocKnotVectorAndArray( m, SK );

        if( knl EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knl, &ml, &UL );

        for ( i = 0; i <= ml; i++ )
            UL[i] = U[i];

        *knt = knl;
    }
    else
    {
        knl = N_AllocKnotVectorAndArray( mt + m, SK );

        if( knl EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knl, &ml, &UL );

        ml = -1;
        i = j = 0;

        while( i LE mt AND j LE m )
        {
            if( UH[i]LT U[j] )
            {
                UL[++ml] = UH[i];
                i++;
            }
            else if( UH[i]EQ U[j] )
            {
                UL[++ml] = UH[i];
                i++;
                j++;
            }
            else if( UH[i]GT U[j] )
            {
                UL[++ml] = U[j];
                j++;
            }
        }

        N_SetKnotIndex( knl, ml );
        N_FreeKnotVector( *knt, SK );

        *knt = knl;
    }

    if( n LT nh )
    {
        error = N_CrvCompress( cur, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    NL_ITLIM = ITLIM_SAV;

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROXKNOTSANDTANGENTSTOL: Curve approximation with error bound & end constraints   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS  curve approximating a given
     set of  points to  a user  given  tolerance. End  points  and  end 
     tangents or derivatives  may be  constrained or unconstrained. The 
     method  starts with  an  interpolating  curve of given degree, and 
     works its way up to the required degree by repeating the following 
     steps:
       (1) remove as many knots as possible;
       (2) least-squares fit with a higher degree curve; and
       (3) adjust parameter values and error vector.
     If one of the  above steps fails,  the process is  re-started with 
     another  interpolating  curve  of  one  degree  higher. A  typical 
     calling example is as follows:

       NL_POINT   *P;
       NL_INDEX   m;
       NL_DEGREE  ps, pr;
       NL_REAL    E;
       NL_VECTOR  Ts;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P, and choose ps, pr, Ts and E);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxKnotsAndTangentsTol(P,m,NL_START,Ts,NULL,NL_YES,ps,pr,E,NL_SINGLE,&cur,&SG);

     The  algorithm can  use single or multiple knots for data approxi-
     mation. The  single-knot  approximation  smooths  out  the  data,
     whereas  the  multiple-knot  approximant captures  local phenomena 
     such as straight edges and cusps.


   ACCESS:
   
     P   , input  ,  Point array to be approximated
     m   , input  ,  Highest index in P
     cst , input  ,  Flag:
                       NL_NO   : neither end point is constrained
                       NL_START: start point is constrained
                       NL_END  : end point is constrained
                       NL_BOTH : start and end points are constrained
     Ts  , input  ,  Start tangent:
                       !NULL: tangent constraint applied at start
                        NULL: no tangent constraint applied at start
     Te  , input  ,  End tangent:
                       !NULL: tangent constraint applied at the end
                        NULL: no tangent constraint applied at the end
     mag , input  ,  Flag:
                       NL_YES: keep tangent magnitudes
                       NL_NO : adjust magnitudes as appropriate
     ps  , input  ,  Start iterations with degree ps (if  cusps are  to 
                     be preserved, ps=1 is recommended. Otherwise  ps=2
                     is a good  default).
     pr  , input  ,  Required degree of approximating curve
     E   , input  ,  Error tolerance
     ktp , input  ,  Flag:
                       NL_SINGLE  : use single knots for approximation
                       NL_MULTIPLE: use multiple knots for approximation
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvApproxKnotsAndTangentsTol( NL_POINT *P, NL_INDEX m, NL_FLAG cst, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG mag, NL_DEGREE ps, NL_DEGREE pr, NL_REAL E, NL_FLAG ktp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxKnotsAndTangentsTol");

    NL_FLAG apr, error = NL_NO;

    NL_INDEX I[2], i, k, l, nc, mc, nh, mh, nsp, mlt, c;

    NL_DEGREE ii, jj;

    NL_REAL *U, *er, *wp, wd[2], d, ltop, ltoc;

    NL_PARAMETER *u, *us;

    NL_POINT R;

    NL_VECTOR D[2], *Vs = NULL, *Ve = NULL, As = { 0,0,0 }, Ae = { 0,0,0 };

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL top = 1.0e-02;
    NL_PRIVATE NL_REAL toc = 1.0e-03;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Adjust tolerance */

    if( pr GT m )
        NL_ERROR( NL_INP_ERR );

    if( pr LT ps )
        NL_ERROR( NL_INP_ERR );

    N_DistPolygon( P, m, &d );

    ltop = d * top;
    ltoc = toc;
    c = -1;

    /* Get tangents or derivatives */

    if( Ts NEQ NULL )
    {
        As = *Ts;

        if( mag EQ NL_NO )
        {
            error = N_VectorNormalizeRef( &As );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( As, d, &As );
        }
    }

    if( Te NEQ NULL )
    {
        Ae = *Te;

        if( mag EQ NL_NO )
        {
            error = N_VectorNormalizeRef( &Ae );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( Ae, d, &Ae );
        }
    }

    if( Ts NEQ NULL AND Te NEQ NULL )
    {
        N_VectorCopy( As, &D[0] );
        N_VectorCopy( Ae, &D[1] );

        Vs = &As;
        Ve = &Ae;
        wd[0] = -1.0;
        wd[1] = -1.0;
        I[0] = 0;
        I[1] = m;
        c = 1;
    }
    else if( Ts NEQ NULL AND Te EQ NULL )
    {
        N_VectorCopy( As, &D[0] );

        Vs = &As;
        wd[0] = -1.0;
        I[0] = 0;
        c = 0;
    }
    else if( Ts EQ NULL AND Te NEQ NULL )
    {
        N_VectorCopy( Ae, &D[0] );

        Ve = &Ae;
        wd[0] = -1.0;
        I[0] = m;
        c = 0;
    }

    /* Check for curve memory */

    error = N_CrvSizeArrays( cur, m + 2, pr, m + pr + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnotVector( cur, &knt );
    N_KnotVectorGetKnots( knt, &k, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( m, &SL );

    if( u EQ NULL )
        NL_QUIT;

    us = N_AllocReal1dArray( m, &SL );

    if( us EQ NULL )
        NL_QUIT;

    er = N_AllocReal1dArray( m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    wp = N_AllocReal1dArray( m, &SL );

    if( wp EQ NULL )
        NL_QUIT;

    /* Set up constraint array */

    for ( i = 0; i <= m; i++ )
        wp[i] = 1.0;

    if( cst EQ NL_START )
        wp[0] = -1.0;

    else if( cst EQ NL_END )
        wp[m] = -1.0;

    else if( cst EQ NL_BOTH )
        wp[0] = wp[m] = -1.0;

    /* Get parameters */

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, NL_CHORDLENGTH, us );

    if( error EQ NL_YES )
        NL_OUT;

    /*********************************************************/
    /*  Fit curve as follows:                                */
    /*    for ii=ps to pr do                                 */
    /*       interpolate with degree ii                      */
    /*       for jj=ii to pr do                              */
    /*          remove knots                                 */
    /*          compute degree elevated curve's knot vector  */
    /*          least-squares approximate                    */
    /*          if approximation fails break                 */
    /*          adjust parameters and error vector           */
    /*********************************************************/

    for ( ii = ps; ii <= pr; ii++ )
    {
        /* Interpolate with degree ii */

        for ( k = 0; k <= m; k++ )
        {
            u[k] = us[k];
            er[k] = 0.0;
        }

        if( Ts EQ NULL AND Te EQ NULL )
        {
            N_FitCrvCalcKnotVector( u, m, ii, knt );
            N_CrvSetSizeIndices( cur, m, ii, m + ii + 1 );

            error = N_FitCrvInterpGivenParams( (NL_VOID *)P, m, NL_EPOINT, u, knt, ii, cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( Ts NEQ NULL AND Te EQ NULL )
        {
            N_FitCalcKnotVectorDeriv( u, m, ii, NL_START, knt );
            N_CrvSetSizeIndices( cur, m + 1, ii, m + ii + 2 );

            error = N_FitCrvKnotsAndDeriv( (NL_VOID *)P, m, NL_EPOINT, u, knt, ii, (NL_VOID *)Vs, NL_START, cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( Ts EQ NULL AND Te NEQ NULL )
        {
            N_FitCalcKnotVectorDeriv( u, m, ii, NL_END, knt );
            N_CrvSetSizeIndices( cur, m + 1, ii, m + ii + 2 );

            error = N_FitCrvKnotsAndDeriv( (NL_VOID *)P, m, NL_EPOINT, u, knt, ii, (NL_VOID *)Ve, NL_END, cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( Ts NEQ NULL AND Te NEQ NULL )
        {
            N_FitCalcKnotVectorEndDerivs( u, m, ii, knt );
            N_CrvSetSizeIndices( cur, m + 2, ii, m + ii + 3 );

            error = N_FitCrvKnotsAndDerivs( (NL_VOID *)P, m, NL_EPOINT, u, knt, ii, (NL_VOID *)Vs, (NL_VOID *)Ve, cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( jj = ii; jj <= pr; jj++ )
        {
            /* Remove as many knots as possible */

            if( pr LE 2 )
            {
                error = N_FitRemoveKnotsAndDerivs( cur, u, er, m, E, cst, 1 );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                error = N_FitRemoveKnots( cur, u, er, m, E );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( jj EQ pr )
                break;

            N_CrvGetArraySizes( cur, &nc, &mc );

            /* Get new knot vector */

            if( ktp EQ NL_SINGLE )
            {
                nh = nc + 1;
                mh = mc + 2;

                if( nh GT m )
                    break;

                for ( k = 0; k <= jj + 1; k++ )
                    U[mh--] = U[mc];

                for ( k = nc; k >= jj + 1; k-- )
                    U[mh--] = U[k];

                for ( k = 0; k <= jj + 1; k++ )
                    U[mh--] = U[0];
            }
            else
            {
                N_BasisGetSpanCount( knt, jj, &nsp );

                nh = nc + nsp;
                mh = mc + nsp + 1;

                if( nh GT m )
                    break;

                while( mc GT 0 )
                {
                    k = mc;

                    while( mc GT 0 AND U[mc]EQ U[mc - 1] )
                        mc--;
                    mlt = k - mc + 1;

                    for ( l = 1; l <= mlt + 1; l++ )
                        U[mh--] = U[mc];
                    mc--;
                }
            }

            /* Approximate by least-squares */

            N_CrvSetSizeIndices( cur, nh, (NL_DEGREE)(jj + 1), nh + jj + 2 );

            error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, m, (NL_VOID *)D, wd, I, c, NL_EPOINT, u, knt, nh, (NL_DEGREE)(jj + 1), cur, &SL );

            if( error EQ NL_YES )
                break;

            /* Adjust parameters and error vector */

            apr = NL_TRUE;

            for ( k = 1; k < m; k++ )
            {
                error = N_CrvClosestPt( cur, P[k], u[k], ltop, ltoc, &u[k], &R );

                if( error EQ NL_YES )
                {
                    apr = NL_FALSE;
                    break;
                }

                N_DistPtPt( P[k], R, &er[k] );

                if( er[k]GT E )
                {
                    apr = NL_FALSE;
                    break;
                }
            }

            if( apr EQ NL_FALSE )
                break;
        }

        if( jj EQ pr )
            break;
    }

    /* Compact curve */

    error = N_CrvCompress( cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVFIRSTDERIV: Curve interpolation with first derivatives specified     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of points and first derivatives at those points. The degree is 
     2  or  3  and the  type of  parametrization  can be chosen. If the 
     output curve is initialized to the NULL curve, memory is allocated  
     locally. Otherwise  it is checked if enough memory is passed in. A 
     typical calling example is:

       NL_POINT   *P;
       NL_VECTOR  *D;
       NL_INDEX   kk;
       NL_DEGREE  p;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P and D, and choose the degree);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvFirstDeriv(P,D,kk,p,NL_TANGENT,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.
     THE  ROUTINE AUTOMATICALLY  COMPUTES NL_DERIVATIVE  MAGNITUDES IF NOT
     PASSED IN. It  first normalizes  each tangent, then scales them by 
     the chord length of the data set.


   ACCESS:
   
     P     , input  ,  Points to be interpolated
     D     , input  ,  Derivatives to be interpolated
     kk    , input  ,  Highest index in P and D
     p     , input  ,  Degree of  interpolating curve (must be 2 or 3!)
     vec   , input  ,  Flag:
                         NL_TANGENT   : Tangent directions passed in. Need
                                     to compute proper magnitudes.
                         NL_DERIVATIVE: Derivatives  passed   in  (D's are 
                                     taken as is)
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization 
                         NL_CENTRIPETAL: Centripetal parametrization 
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvFirstDeriv( NL_POINT *P, NL_VECTOR *D, NL_INDEX kk, NL_DEGREE p, NL_FLAG vec, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvFirstDeriv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, ii, j, k, l, n, bw, sbw, hp;

    NL_REAL ** A, ** ND, *U, fact, alf;

    NL_PARAMETER *u;

    NL_POINT *Q;

    NL_CPOINT *Pw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( p LT 2 OR p GT 3 )
        NL_ERROR( NL_DEG_ERR );

    n = 2 * kk + 1;

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );
    N_CrvGetKnotVector( cur, &knt );

    /* Get parameters */

    u = N_AllocReal1dArray( kk, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, kk, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute knot vector */

    N_FitCalcKnotsDerivs( u, kk, p, knt );

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    ND = N_AllocReal2dArray( 1, p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[1][sbw - 1] = -1.0;
    A[1][sbw] = 1.0;
    A[n - 1][sbw] = -1.0;
    A[n - 1][sbw + 1] = 1.0;
    A[n][sbw] = 1.0;

    if( p EQ 3 )
        hp = p;
    else
        hp = p - 1;

    for ( i = 1; i < kk; i++ )
    {
        error = N_BasisDerivs( knt, p, u[i], NL_LEFT, 1, ND, &j );

        if( error EQ NL_YES )
            NL_OUT;

        ii = 2 * i;
        l = j - ii - 1;

        for ( k = 0; k <= hp; k++ )
        {
            A[ii][l + k] = ND[0][k];
            A[ii + 1][l + k - 1] = ND[1][k];
        }
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute magnitudes if needed */

    if( vec EQ NL_TANGENT )
    {
        N_DistPolygon( P, kk, &alf );

        for ( i = 0; i <= kk; i++ )
        {
            error = N_VectorNormalizeRef( &D[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( D[i], alf, &D[i] );
        }
    }

    /* Solve system of linear equations */

    Q = N_AllocPt1dArray( n, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    fact = U[p + 1] / p;
    N_CopyPt( P[0], &Q[0] );
    N_ScalePt( fact, D[0], &Q[1] );

    for ( i = 1; i < kk; i++ )
    {
        ii = 2 * i;
        N_CopyPt( P[i], &Q[ii] );
        N_CopyPt( D[i], &Q[ii + 1] );
    }

    fact = (1.0 - U[n]) / p;
    N_ScalePt( fact, D[kk], &Q[n - 1] );
    N_CopyPt( P[kk], &Q[n] );

    error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCRVFIRSTDERIVANDKNOTS: Curve interpolation with first derivatives & knot vector */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set  of points  and first  derivatives  at  those points. It  is a 
     general  routine  requiring  parameters  where  data   points  are 
     assumed, a knot vector, and  NL_POINT or NL_CPOINT point data and NL_VECTOR
     or  NL_CVECTOR derivative  data. The degree  must be  2 or 3.  If the 
     output curve is initialized to the NULL curve, memory is allocated  
     locally. Otherwise  it is checked if enough memory is passed in. A 
     typical calling example is:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_VECTOR      *D;
       NL_CVECTOR     *Dw;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  *knt;
       NL_INDEX       kk;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get P/Pw, D/Dw, u and knt, and choose the degree);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvFirstDerivAndKnots((NL_VOID *)P ,(NL_VOID *)D ,kk,NL_EPOINT,u,knt,p,&cur,&SG);
       N_FitCrvFirstDerivAndKnots((NL_VOID *)Pw,(NL_VOID *)Dw,kk,NL_HPOINT,u,knt,p,&cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT NL_VECTOR ARE COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     A    , input  ,  NL_VOID pointer representing  NL_POINT or  NL_CPOINT data
                      as interpolation points
     B    , input  ,  NL_VOID pointer representing NL_VECTOR or NL_CVECTOR data
                      as derivatives at interpolation points
     kk   , input  ,  Highest index in A and B
     ptp  , input  ,  Flag:
                        NL_EPOINT: Euclidean pointer passed in
                        NL_HPOINT: Homogeneous pointer passed in
     u    , input  ,  Parameters; A[i] and B[i] are assumed at u[i]
     knt  , input  ,  Knot vector of interpolating curve
     p    , input  ,  Degree of  interpolating curve (MUST BE 2 OR 3!)
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvFirstDerivAndKnots( NL_VOID *A, NL_VOID *B, NL_INDEX kk, NL_FLAG ptp, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvFirstDerivAndKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX i, ii, j, k, l, n, m, mc, bw, sbw, hp;

    NL_REAL ** M, ** ND, *U, *UC, fact;

    NL_POINT *Q, *R;

    NL_VECTOR *D;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_CVECTOR *Dw;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error and curve memory */

    N_KnotVectorGetKnots( knt, &m, &U );
    n = 2 * kk + 1;
    mc = n + p + 1;

    if( p LT 2 OR p GT 3 )
        NL_ERROR( NL_DEG_ERR );

    if( m NEQ mc )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= m; i++ )
        UC[i] = U[i];

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    ND = N_AllocReal2dArray( 1, p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &M );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            M[i][j] = 0.0;
    }

    M[0][sbw] = 1.0;
    M[1][sbw - 1] = -1.0;
    M[1][sbw] = 1.0;
    M[n - 1][sbw] = -1.0;
    M[n - 1][sbw + 1] = 1.0;
    M[n][sbw] = 1.0;

    if( p EQ 3 )
        hp = p;
    else
        hp = p - 1;

    for ( i = 1; i < kk; i++ )
    {
        error = N_BasisDerivs( knt, p, u[i], NL_LEFT, 1, ND, &j );

        if( error EQ NL_YES )
            NL_OUT;

        ii = 2 * i;
        l = j - ii - 1;

        if( l LT 0 OR l + hp GE bw )
            NL_ERROR( NL_INP_ERR );

        for ( k = 0; k <= hp; k++ )
        {
            M[ii][l + k] = ND[0][k];
            M[ii + 1][l + k - 1] = ND[1][k];
        }
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)A;
            D = (NL_VECTOR *)B;

            Q = N_AllocPt1dArray( n, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyPt( R[0], &Q[0] );
            N_ScalePt( fact, D[0], &Q[1] );

            for ( i = 1; i < kk; i++ )
            {
                ii = 2 * i;
                N_CopyPt( R[i], &Q[ii] );
                N_CopyPt( D[i], &Q[ii + 1] );
            }

            fact = (U[n + p] - U[n]) / p;
            N_ScalePt( fact, D[kk], &Q[n - 1] );
            N_CopyPt( R[kk], &Q[n] );

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            Dw = (NL_CVECTOR *)B;

            Qw = N_AllocCPt1dArray( n, &SL );

            if( Qw EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyCPt( Rw[0], &Qw[0] );
            N_ScaleCPt( fact, Dw[0], &Qw[1] );

            for ( i = 1; i < kk; i++ )
            {
                ii = 2 * i;
                N_CopyCPt( Rw[i], &Qw[ii] );
                N_CopyCPt( Dw[i], &Qw[ii + 1] );
            }

            fact = (U[n + p] - U[n]) / p;
            N_ScaleCPt( fact, Dw[kk], &Qw[n - 1] );
            N_CopyCPt( Rw[kk], &Qw[n] );

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVKNOTSANDTANGENTS: Curve interpolation with knots and end tangents          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set of points. Optionally end tangents, parameters and knots may be
     passed in. The  routine  uses a  subset of  the knots  if they are
     suitable  for intepolation. These  knots  must  lie  in  intervals
     defined by  knots computed by averaging the parameters. If some of 
     these intervals do not contain knots from  the input  knot vector, 
     the  knots  defining the  intervals are  added to  the  input knot 
     vector. If the  output  curve is  initialized  to the NULL  curve, 
     memory is  allocated  locally. Otherwise it is  checked if  enough 
     memory is passed in. A typical calling example is as follows:

       NL_POINT       *P;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  *knt;
       NL_VECTOR      Ts, Te;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SC, SK;
       ...
       (get arrays P and u, define knt, choose p, and select Ts, Te);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvKnotsAndTangents(P,k,u   ,&Ts ,&Te ,NL_TANGENT   ,&knt,0.7,p,&cur,&SC,&SK);
       knt = NULL;
       N_FitCrvKnotsAndTangents(P,k,NULL,NULL,NULL,NL_DERIVATIVE,&knt,0.7,p,&cur,&SC,&SK);


   ACCESS:
   
     P   , input  ,  Points to be interpolated
     k   , input  ,  Highest index in P
     u   , input  ,  Parameters corresponding to data points
                       !NULL: parameters passed in
                        NULL: must be computed locally
     Ts  , in/out ,  Start tangent/derivative
                       !NULL: interpolate with start tangent
                        NULL: interpolate without start tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE WILL BE RESCALED!
     Te  , in/out ,  End tangent/derivative
                       !NULL: interpolate with end tangent
                        NULL: interpolate without end tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE WILL BE RESCALED!
     der , input  ,  Flag:
                       NL_TANGENT   : Ts and/or Te are  tangent directions
                                   only and must be scaled internally
                       NL_DERIVATIVE: Ts and/or Te are derivatives and are
                                   used as passed in
     knt , in/out ,  Knot vector.  The  double  pointer knt must not be 
                     NULL.  The  single  pointer *knt may or may not be
                     NULL on input.
                       !NULL: choose the knots from this  knot vetor if
                              possible. If  in  certain  positions  new 
                              knots are required, add these to knt.
                        NULL: compute knt internally
     per , input  ,  Percentage of interval usage
                       1.0: the entire interval is  used to  find knots 
                        |   in knt.  This  gives  maximum  use  of knt,
                        |   however,  the  interpolant may  not  be  as 
                        |   pleasing as desired.
                       \|/
                       0.0: only one knot may be used. This  gives zero
                            flexibility and produces a  new knt that is 
                            the old knt plus a new knot vector obtained 
                            by parameter averaging.
                     For most data sets per = 1.0 is quite adequate. If
                     the data changes rapidly, the suggested values are
                       p = 2: --> per = 0.6 (or higher)
                           3: --> per = 0.8 (or higher)
                         > 3: --> per = 1.0
     p   , input  ,  Degree of interpolating curve (p>=2)
     cur , output ,  Interpolating curve
     SC  , input  ,  cur's memory stack
     SK  , input  ,  knt's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvKnotsAndTangents( NL_POINT *P, NL_INDEX k, NL_PARAMETER *u, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG der, NL_KNOTVECTOR ** knt, NL_REAL per, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SC, NL_STACKS *SK )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvKnotsAndTangents");

    NL_FLAG tan = NL_NO, error = NL_NO;

    NL_INDEX i, j, jc, n = 0, mt = 0, mo = 0, ms = 0;

    NL_REAL *ul, *UT = NULL, *UI, *UA, *UP, *US = NULL, *UO, a, b, d, fac;

    NL_DEGREE pt = 0;

    NL_KNOTVECTOR *kni, *kna, *knp, *kno;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check incoming data */

    if( *knt NEQ NULL )
    {
        N_KnotVectorGetKnots( *knt, &mt, &UT );

        for ( pt = 0; pt < mt; pt++ )
        {
            if( UT[pt]NEQ UT[pt + 1] )
                break;
        }

        if( pt GE mt )
            NL_ERROR( NL_INP_ERR );
        error = N_KnotVectorIsValid( *knt, pt, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( per LT 0.0 OR per GT 1.0 )
        NL_ERROR( NL_INP_ERR );

    if( u NEQ NULL AND *knt NEQ NULL )
    {
        if( u[0]NEQ UT[0]OR u[k]NEQ UT[mt] )
            NL_ERROR( NL_INP_ERR );
    }

    /* Check tangency conditions */

    if( Ts EQ NULL AND Te EQ NULL )
    {
        tan = NL_NO;
        n = k;
    }
    else if( Ts NEQ NULL AND Te EQ NULL )
    {
        tan = NL_START;
        n = k + 1;
    }
    else if( Ts EQ NULL AND Te NEQ NULL )
    {
        tan = NL_END;
        n = k + 1;
    }
    else if( Ts NEQ NULL AND Te NEQ NULL )
    {
        tan = NL_BOTH;
        n = k + 2;
    }

    /* Compute parameters if not passed in */
    if( u EQ NULL )
    {
        ul = N_AllocReal1dArray( k, &SL );

        if( ul EQ NULL )
            NL_QUIT;

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, ul );

        if( error EQ NL_YES )
            NL_OUT;

        if( *knt NEQ NULL )
        {
            if( ul[0]NEQ UT[0]OR ul[k]NEQ UT[mt] )
            {
                fac = (UT[mt] - UT[0]) / (ul[k] - ul[0]);

                for ( i = 1; i < k; i++ )
                {
                    ul[i] = UT[0] + fac * (ul[i] - ul[0]);
                }

                ul[0] = UT[0];
                ul[k] = UT[mt];
            }
        }
    }
    else
    {
        ul = u;
    }

    /* Get knot vector if none passed in */

    kni = N_AllocKnotVectorAndArray( n + p + 1, &SL );

    if( kni EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( kni, &i, &UI );

    if( *knt EQ NULL )
    {
        if( tan EQ NL_NO )
            N_FitCrvCalcKnotVector( ul, k, p, kni );

        else if( tan EQ NL_START )
            N_FitCalcKnotVectorDeriv( ul, k, p, NL_START, kni );

        else if( tan EQ NL_END )
            N_FitCalcKnotVectorDeriv( ul, k, p, NL_END, kni );

        else if( tan EQ NL_BOTH )
            N_FitCalcKnotVectorEndDerivs( ul, k, p, kni );

        mo = n + p + 1;
    }

    /* Get tangent magnitudes */

    if( tan NEQ NL_NO AND der EQ NL_TANGENT )
    {
        N_DistPolygon( P, k, &fac );
        fac = fac * (ul[k] - ul[0]);

        if( Ts NEQ NULL )
        {
            error = N_VectorNormalizeRef( Ts );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( *Ts, fac, Ts );
        }

        if( Te NEQ NULL )
        {
            error = N_VectorNormalizeRef( Te );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( *Te, fac, Te );
        }
    }

    /* Select knots for interpolation */

    if( *knt NEQ NULL )
    {
        US = N_AllocReal1dArray( n + p + 1, &SL );

        if( US EQ NULL )
            NL_QUIT;

        kna = N_AllocKnotVectorAndArray( n + p + 1, &SL );

        if( kna EQ NULL )
            NL_QUIT;

        knp = N_AllocKnotVectorAndArray( n + p, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knp, &i, &UP );
        N_KnotVectorGetKnots( kna, &i, &UA );

        for ( i = 0; i <= p; i++ )
        {
            UI[i] = ul[0];
            UI[n + i + 1] = ul[k];
        }

        /* Get ideal knot vector for degree p */

        if( tan EQ NL_NO )
            N_FitCrvCalcKnotVector( ul, k, p, kna );

        else if( tan EQ NL_START )
            N_FitCalcKnotVectorDeriv( ul, k, p, NL_START, kna );

        else if( tan EQ NL_END )
            N_FitCalcKnotVectorDeriv( ul, k, p, NL_END, kna );

        else if( tan EQ NL_BOTH )
            N_FitCalcKnotVectorEndDerivs( ul, k, p, kna );

        /* Get ideal knot vector for degree p-1 */
        if( tan EQ NL_NO )
            N_FitCrvCalcKnotVector( ul, k, (NL_DEGREE)(p - 1), knp );

        else if( tan EQ NL_START )
            N_FitCalcKnotVectorDeriv( ul, k, (NL_DEGREE)(p - 1), NL_START, knp );

        else if( tan EQ NL_END )
            N_FitCalcKnotVectorDeriv( ul, k, (NL_DEGREE)(p - 1), NL_END, knp );

        else if( tan EQ NL_BOTH )
            N_FitCalcKnotVectorEndDerivs( ul, k, (NL_DEGREE)(p - 1), knp );

        /* Select knots from knt */

        j = pt + 1;
        ms = -1;

        for ( i = p + 1; i <= n; i++ )
        {
            a = (1.0 - per) * UA[i] + per * UP[i - 1];
            b = (1.0 - per) * UA[i] + per * UP[i];

            /* See if there are knots in (a,b). If yes, select the */
            /* one closest to UA[i]. If not, add UA[i] to knt.     */

            while( UT[j]LE a )
                j++;

            fac = b - a;
            jc = -1;

            while( UT[j]GT a AND UT[j]LT b )
            {
                d = fabs( UA[i] - UT[j] );

                if( d LT fac )
                {
                    fac = d;
                    jc = j;
                }
                j++;
            }

            if( jc EQ - 1 )
            {
                UI[i] = UA[i];
                US[++ms] = UA[i];
            }
            else
            {
                UI[i] = UT[jc];
            }
        }
    }

    /* Now interpolate data points */

    if( tan EQ NL_NO )
    {
        error = N_FitCrvInterpGivenParams( (NL_VOID *)P, k, NL_EPOINT, ul, kni, p, cur, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else if( tan EQ NL_START )
    {
        error = N_FitCrvKnotsAndDeriv( (NL_VOID *)P, k, NL_EPOINT, ul, kni, p, (NL_VOID *)Ts, NL_START, cur, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else if( tan EQ NL_END )
    {
        error = N_FitCrvKnotsAndDeriv( (NL_VOID *)P, k, NL_EPOINT, ul, kni, p, (NL_VOID *)Te, NL_END, cur, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else if( tan EQ NL_BOTH )
    {
        error = N_FitCrvKnotsAndDerivs( (NL_VOID *)P, k, NL_EPOINT, ul, kni, p, (NL_VOID *)Ts, (NL_VOID *)Te, cur, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Update input knot vector */

    if( *knt EQ NULL )
    {
        kno = N_AllocKnotVectorAndArray( mo, SK );

        if( kno EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( kno, &mo, &UO );

        for ( i = 0; i <= mo; i++ )
            UO[i] = UI[i];

        *knt = kno;
    }
    else if( ms GT - 1 )
    {
        mo = mt + ms + 1;

        kno = N_AllocKnotVectorAndArray( mo, SK );

        if( kno EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( kno, &mo, &UO );

        mo = -1;
        i = 0;
        j = 0;

        while( i LE mt AND j LE ms )
        {
            if( UT[i]LT US[j] )
            {
                UO[++mo] = UT[i];
                i++;
            }
            else
            {
                UO[++mo] = US[j];
                j++;
            }
        }

        if( i LE mt )
        {
            while( i LE mt )
            {
                UO[++mo] = UT[i];
                i++;
            }
        }

        N_FreeKnotVector( *knt, SK );

        *knt = kno;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCRVMATRIX: Curve interpolation with specified matrix                */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes the  control points of a  NURBS curve  
     interpolating a  given  set of  points. This  is a special  routine 
     requiring the  input of the  LU-decomposed interpolation matrix and 
     NL_POINT or NL_CPOINT data. A typical calling example is as follows:

       NL_POINT    *Q;
       NL_CPOINT   *Qw, *Pw;
       NL_INDEX    k;
       NL_DEGREE   p;
       NL_RMATRIX  cm;
       ...
       (get arrays Q/Qw, choose p, get cm and allocate memory for Pw);
       ...
       N_FitCrvMatrix((NL_VOID *)Q ,k,NL_EPOINT,p,&cm,Pw);
       N_FitCrvMatrix((NL_VOID *)Qw,k,NL_HPOINT,p,&cm,Pw);

     IT IS  ASSUMED  THAT  MEMORY FOR  Pw  IS ALLOCATED  IN  THE CALLING 
     ROUTINE!! IT IS  ALSO ASSUMED THAT THE INTERPOLATION MATRIX  cm  IS 
     LU-DECOMPOSED!!


   ACCESS:
   
     A   , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     k   , input  ,  Highest index in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     p   , input  ,  Degree of interpolating curve (must be <= k)
     cm  , input  ,  LU-decomposed interpolation matrix
     Pw  , output ,  Control  points of  interpolating curve.  MUST HAVE 
                     ENOUGH MEMORY TO HOLD Pw[0],...,Pw[k]!!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvMatrix( NL_VOID *A, NL_INDEX k, NL_FLAG ptp, NL_DEGREE p, NL_RMATRIX *cm, NL_CPOINT *Pw )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_POINT *Q;

    NL_CPOINT *Qw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Control points are the data points in case of linear curves */

    if( p EQ 1 )
    {
        switch( ptp )
        {
            case NL_EPOINT:

                Q = (NL_POINT *)A;

                for ( i = 0; i <= k; i++ )
                    N_PtToCPt( Q[i], &Pw[i] );
                break;

            case NL_HPOINT:

                Qw = (NL_CPOINT *)A;

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Qw[i], &Pw[i] );
                break;

            default:

                NL_ERROR( NL_CAL_ERR );
        }

        NL_OUT;
    }

    /* Solve system of linear equations */

    error = N_RealMatrixForBack( cm, A, ptp, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVARCS: Curve interpolation with piecewise circular arcs         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes a NURBS  piecewise  circular curve 
     interpolating a given set of points. The parametrization is NL_G1. If 
     the  output  curve is  initialized  to the  NULL  curve, memory is 
     allocated  locally. Otherwise  it  is checked if  enough memory is 
     passed in. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvArcs(P,k,NL_CHORDLENGTH,NL_AKIMA,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.
     The method employs biarcs fitted between neighboring points.


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k    , input  ,  Highest index in P
     par  , input  ,  Flag:
                        NL_UNIFORM    : Uniform parametrization
                        NL_CHORDLENGTH: Chord length parametrization 
                        NL_CENTRIPETAL: Centripetal parametrization 
     tan  , input  ,  Flag:
                        NL_BESSEL: Tangents computed by Bessel's method
                        NL_AKIMA : Tangents computed by Akima's method
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvArcs( NL_POINT *PP, NL_INDEX kk, NL_FLAG par, NL_FLAG tan, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvArcs");

    NL_FLAG closed, type, reset = NL_NO, error = NL_NO;

    NL_INDEX i, j, l, n, m, k;

    NL_REAL *U, d, dui, dui1, alf, oma, a, b;

    NL_PARAMETER *u;

    NL_POINT *Q, *P;

    NL_CPOINT *Pw;

    NL_VECTOR *S, *SU, *T, A, B;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make copy of point array, since we may have to overwrite */

    k = kk;
    P = N_AllocPt1dArray( kk, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
        N_CopyPt( PP[i], &P[i] );

    /* Check for curve memory */

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    n = 16 * k;
    error = N_CrvSizeArrays( cur, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( 4 * k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( 4 * k, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( k + 3, &SL );

    if( S EQ NULL )
        NL_QUIT;

    SU = N_AllocPt1dArray( k + 3, &SL );

    if( SU EQ NULL )
        NL_QUIT;

    T = N_AllocPt1dArray( k, &SL );

    if( T EQ NULL )
        NL_QUIT;

    /* Compute the chords and normalize them */

    N_DistPtPt( P[0], P[k], &d );

    if( d LT NL_MTOL )
        closed = NL_YES;
    else
        closed = NL_NO;

    for ( i = 1; i <= k; i++ )
        N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

    if( closed EQ NL_YES )
    {
        N_VectorCopy( S[k + 1], &S[1] );
        N_VectorCopy( S[k], &S[0] );
        N_VectorCopy( S[2], &S[k + 2] );
        N_VectorCopy( S[3], &S[k + 3] );
    }
    else
    {
        N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
        N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
        N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
        N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
    }

    for ( i = 2; i <= k + 1; i++ )
    {
        error = N_VectorNormalize( S[i], &SU[i], &a );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute the unit tangents */

    switch( tan )
    {
        case NL_BESSEL:

            error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, par, u );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 1; i < k; i++ )
            {
                dui = u[i] - u[i - 1];
                dui1 = u[i + 1] - u[i];

                if( N_FloatOpIsBad( dui, dui + dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = dui / (dui + dui1);
                oma = 1.0 - alf;

                if( N_FloatOpIsBad( oma, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( alf, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = oma / dui;
                alf = alf / dui1;

                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }

            dui = u[1] - u[0];
            dui1 = u[k] - u[k - 1];

            if( closed EQ NL_YES )
            {
                if( N_FloatOpIsBad( 1.0, 2.0 *dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 1.0, 2.0 *dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 1.0 / (2.0 *dui);
                alf = 1.0 / (2.0 *dui1);

                N_VectorCombine( oma, S[2], alf, S[k + 1], &T[0] );
                N_VectorCopy( T[0], &T[k] );
            }
            else
            {
                if( N_FloatOpIsBad( 2.0, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 2.0, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 2.0 / dui;
                alf = 2.0 / dui1;

                N_VectorCombine( oma, S[2], -1.0, T[1], &T[0] );
                N_VectorCombine( alf, S[k + 1], -1.0, T[k - 1], &T[k] );
            }
            break;

        case NL_AKIMA:
            for ( i = 0; i <= k; i++ )
            {
                N_VectorCross( S[i], S[i + 1], &A );
                N_VectorMagnitude( A, &a );
                N_VectorCross( S[i + 2], S[i + 3], &B );
                N_VectorMagnitude( B, &b );

                if( (a + b)LT NL_LTOL )
                {
                    alf = 0.5;
                }
                else
                {
                    alf = a / (a + b);
                }

                oma = 1.0 - alf;
                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= k; i++ )
    {
        error = N_VectorNormalizeRef( &T[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Interpolate segment by segment */

    n = 0;

    for ( i = 1; i <= k; i++ )
    {
        if( i EQ k )
        {
            type = 2;
        }
        else
        {
            N_VectorCross( SU[i + 1], T[i - 1], &A );
            N_VectorMagnitude( A, &a );
            N_VectorCross( SU[i + 1], T[i], &B );
            N_VectorMagnitude( B, &b );

            if( a LT NL_LTOL AND b LT NL_LTOL )
            {
                N_VectorCross( SU[i + 2], T[i], &A );
                N_VectorMagnitude( A, &a );
                N_VectorCross( SU[i + 2], T[i + 1], &B );
                N_VectorMagnitude( B, &b );

                if( a LT NL_LTOL AND b LT NL_LTOL )
                {
                    type = 1;
                }
                else
                {
                    type = 2;
                }
            }
            else
            {
                type = 2;
            }
        }

        switch( type )
        {
            case 1: /* Discard point - shift down data */
                for ( j = i; j < k; j++ )
                {
                    N_CopyPt( P[j + 1], &P[j] );
                    N_VectorCopy( T[j + 1], &T[j] );
                }

                for ( j = i; j <= k + 1; j++ )
                    N_VectorCopy( SU[j + 2], &SU[j + 1] );

                k--;
                i--;
                break;

            case 2: /* Compute biarc */

                error = N_FitArcToEndPtsAndTangents( P[i - 1], T[i - 1], P[i], T[i], &Pw[n], &l );

                if( error EQ NL_YES )
                    NL_OUT;

                n += l;
                break;

            default:

                NL_ERROR( NL_NUM_ERR );
        } /* End of switch */
    }     /* End of loop */

    /* Compute knot vector */

    l = n / 2;
    m = -1;

    for ( i = 0; i <= l; i++ )
    {
        j = 2 * i;
        N_CPtToPtEuclid( Pw[j], &Q[++m] );
    }

    m = 2;

    for ( i = 0; i <= 2; i++ )
        U[i] = 0.0;

    error = N_FitCalcCrvParamValues( (NL_VOID *)Q, l, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 1; i < l; i++ )
    {
        U[++m] = u[i];
        U[++m] = u[i];
    }

    for ( i = 0; i <= 2; i++ )
        U[++m] = 1.0;

    /* Set curve indexes and compact curve */

    N_CrvSetSizeIndices( cur, n, 2, m );

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

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
/* N_FITCRVSHAPE: Interpolate points based on curve shaping                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine  interpolates a  set of  points by  shaping a
     base  curve so  that  it  passes  through  the  given  points. This 
     routine works best if  the points represent a  simple curve like in 
     reverse  engineering  after the  point set has been  segmented. The
     points can be very  irregularly spaced as long as they  represent a
     simple shape. The method  first computes a base curve which then is
     forced to  pass through the  given points  via constrained shaping.
     The  base curve is a  cubic  Hermite  curve or a  C^1  local  cubic 
     interpolant  properly  approximated to  eliminate multiple knots. A 
     typical calling example is:

       NL_CURVE   cur;
       NL_POINT   *P;
       NL_INDEX   k, nit;
       NL_VECTOR  *Ts, *Te;
       NL_STACKS  SG;
       ...
       (get points P, and possible tangents Ts and/or Te);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvShape(P,k,Ts,Te  ,nit,NL_YES,NL_AKIMA ,NL_HERMITE,&cur,&SG);
       N_FitCrvShape(P,k,Ts,NULL,nit,NL_NO ,NL_BESSEL,NL_INTAPPR,&cur,&SG);

     The base curve can be refined before shaping. If it is refined, the
     effect is  more local. If a more  round effect is  desired, set the
     flag to NL_NO.


   ACCESS:
   
     P   , input  ,  Points  output  curve  must  interpolate. P  can be 
                     randomly given with the following assumption:
                       P[0] is the start point of the final curve
                       P[k] is the end point of the final curve
                       P[1..k-1] may be randomly listed
     k   , input  ,  Highest index in P
     Ts  , input  ,  Start tangent direction:
                       Ts NEQ NULL: tangent passed in
                       Ts  EQ NULL: tangent must be computed
     Te  , input  ,  End tangent direction:
                       Te NEQ NULL: tangent passed in
                       Te  EQ NULL: tangent must be computed
     nit , input  ,  Number of  iterations to  reach  the  most  distant 
                     point. The method modifies the base curve nit times
                     to achieve interpolation of all points. nit = 1 can
                     be safely set for most applications. If the data is
                     highly irregular, nit = 2-4 is a good choice.
     rfl , input  ,  Flag:
                       NL_YES: refine base curve bafore shaping
                       NL_NO : do not refine
     tfl , input  ,  Flag:
                       NL_AKIMA   : compute tangent by Akima method
                       NL_BESSEL  : compute tangent by Bessel method
                       NL_CIRCULAR: get tangent by  fitting a circle to the
                                 first three or last three points
     bfl , input  ,  Flag:
                       NL_HERMITE: use cubic Hermite as base curve
                       NL_INTAPPR: use  cubic  interpolant   reapproximated
                                to eliminate multiple knots
     cur , output ,  Curve interpolating P[i], i=0,...,k
     SG  , input  ,  cur' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvShape( NL_POINT *P, NL_INDEX k, NL_VECTOR *Ts, NL_VECTOR *Te, NL_INDEX nit, NL_FLAG rfl, NL_FLAG tfl, NL_FLAG bfl, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvShape");

    NL_FLAG cfl, error = NL_NO;

    NL_INDEX i, j, m, n, np;

    NL_DEGREE p;

    NL_REAL *U, *t, *x, *y, *up, oneth, twoth, len, d, gs, ge, cs, ce, num, den, sum, alf, oma, lam, qu;

    NL_POINT *Q, *PP, R[5], Q1, Q2, C, N;

    NL_VECTOR Vs = { 0,0,0 }, Ve = { 0,0,0 } , Vd, Vu, Tu, Tv;

    NL_CPOINT *Pw;

    NL_CURVE curB, curI;

    NL_KNOTVECTOR *kni, knx;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL dtl = 0.0001;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Handle special case of no internal points */

    if( k LT 1 )
        NL_ERROR( NL_INP_ERR );

    oneth = 1.0 / 3.0;
    twoth = 2.0 / 3.0;

    if( k EQ 1 )
    {
        if( Ts EQ NULL OR Te EQ NULL )
            p = 1;
        else
            p = 3;

        if( p EQ 1 )
        {
            error = N_AllocCrvArrays( cur, 1, 1, 3, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

            for ( i = 0; i <= 1; i++ )
            {
                U[i] = 0.0;
                U[2 + i] = 1.0;
            }

            N_PtToCPt( P[0], &Pw[0] );
            N_PtToCPt( P[k], &Pw[1] );
        }
        else
        {
            error = N_AllocCrvArrays( cur, 3, 3, 7, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

            for ( i = 0; i <= 3; i++ )
            {
                U[i] = 0.0;
                U[4 + i] = 1.0;
            }

            N_PtToCPt( P[0], &Pw[0] );
            N_PtToCPt( P[1], &Pw[3] );

            N_DistPtPt( P[0], P[1], &len );
            N_VectorCopy( *Ts, &Vs );
            N_VectorCopy( *Te, &Ve );
            N_VectorDiff( P[1], P[0], &Vd );

            error = N_VectorsCosAngle( Vd, Vs, &cs );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorsCosAngle( Vd, Ve, &ce );

            if( error EQ NL_YES )
                NL_OUT;

            num = 0.25 *len;
            den = twoth * ce + oneth * cs;

            if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            gs = num / den;

            den = twoth * cs + oneth * ce;

            if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            ge = num / den;

            error = N_VectorNormalizeRef( &Vs );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &Ve );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( Vs, gs, &Vs );
            N_VectorScale( Ve, ge, &Ve );

            N_VectorPtAlongVector( P[0], oneth, Vs, &Q1 );
            N_VectorPtAlongVector( P[k], -oneth, Ve, &Q2 );

            N_PtToCPt( Q1, &Pw[1] );
            N_PtToCPt( Q2, &Pw[2] );
        }

        NL_OUT;
    }

    /* Get internal points */

    t = N_AllocReal1dArray( k, &SL );

    if( t EQ NULL )
        NL_QUIT;

    x = N_AllocReal1dArray( k, &SL );

    if( x EQ NULL )
        NL_QUIT;

    y = N_AllocReal1dArray( 2 * k, &SL );

    if( y EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( k - 2, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    for ( i = 1; i < k; i++ )
        N_VectorCopy( P[i], &Q[i - 1] );

    /* Compute end tangents */

    if( Ts NEQ NULL AND Te NEQ NULL )
    {
        /* Both end tangents are given */

        N_VectorCopy( *Ts, &Vs );
        N_VectorCopy( *Te, &Ve );
    }
    else
    {
        /* Compute end tangents */

        if( Ts EQ NULL )
        {
            if( tfl EQ NL_AKIMA )
            {
                N_VectorCopy( P[0], &R[0] );
                N_VectorCopy( P[1], &R[1] );
                N_VectorCopy( P[2], &R[2] );
                N_VectorCopy( NL_ZERO, &R[3] );
                N_VectorCopy( NL_ZERO, &R[4] );

                error = N_TangentVectorAkima( R, 0, NL_NO, &Vs );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( tfl EQ NL_BESSEL )
            {
                N_VectorCopy( P[0], &R[0] );
                N_VectorCopy( P[1], &R[1] );
                N_VectorCopy( P[2], &R[2] );

                error = N_TangentVectorBessel( R, 0, NULL, NL_NO, &Vs );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( tfl EQ NL_CIRCULAR )
            {
                error = N_CalcCircCenterAndRadius( P[0], P[1], P[2], &C, &d, &cfl );

                if( error EQ NL_YES )
                    NL_OUT;

                if( cfl EQ NL_TRUE )
                {
                    N_VectorDiff( P[2], C, &Vu );
                    N_VectorDiff( P[0], C, &Vd );
                    N_VectorCross( Vu, Vd, &N );
                    N_VectorCross( Vd, N, &Vs );
                }
                else
                {
                    N_VectorDiff( P[2], P[0], &Vd );
                    N_VectorDiff( P[1], P[0], &Vu );
                    N_VectorCombine( 0.5, Vd, 0.5, Vu, &Vs );
                }
            }
        }

        if( Te EQ NULL )
        {
            if( tfl EQ NL_AKIMA )
            {
                N_VectorCopy( P[k], &R[4] );
                N_VectorCopy( P[k - 1], &R[3] );
                N_VectorCopy( P[k - 2], &R[2] );
                N_VectorCopy( NL_ZERO, &R[1] );
                N_VectorCopy( NL_ZERO, &R[0] );

                error = N_TangentVectorAkima( R, 4, NL_NO, &Ve );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( tfl EQ NL_BESSEL )
            {
                N_VectorCopy( P[k], &R[2] );
                N_VectorCopy( P[k - 1], &R[1] );
                N_VectorCopy( P[k - 2], &R[0] );

                error = N_TangentVectorBessel( R, 2, NULL, NL_NO, &Ve );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( tfl EQ NL_CIRCULAR )
            {
                error = N_CalcCircCenterAndRadius( P[k], P[k - 1], P[k - 2], &C, &d, &cfl );

                if( error EQ NL_YES )
                    NL_OUT;

                if( cfl EQ NL_TRUE )
                {
                    N_VectorDiff( P[k - 2], C, &Vu );
                    N_VectorDiff( P[k], C, &Vd );
                    N_VectorCross( Vd, Vu, &N );
                    N_VectorCross( Vd, N, &Ve );
                }
                else
                {
                    N_VectorDiff( P[k], P[k - 2], &Vd );
                    N_VectorDiff( P[k], P[k - 1], &Vu );
                    N_VectorCombine( 0.5, Vd, 0.5, Vu, &Ve );
                }
            }
        }
    }

    /* Get base curve */

    if( bfl EQ NL_HERMITE )
    {
        error = N_AllocCrvArrays( &curB, 3, 3, 7, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsDegreeAndKnots( &curB, &n, &Pw, &p, &m, &U );

        for ( i = 0; i <= 3; i++ )
        {
            U[i] = 0.0;
            U[4 + i] = 1.0;
        }

        N_PtToCPt( P[0], &Pw[0] );
        N_PtToCPt( P[k], &Pw[3] );

        /* Get appropriate end tangent magnitude ratios */

        N_DistPtPt( P[0], P[k], &len );
        N_VectorDiff( P[k], P[0], &Vd );

        error = N_VectorsCosAngle( Vd, Vs, &cs );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorsCosAngle( Vd, Ve, &ce );

        if( error EQ NL_YES )
            NL_OUT;

        num = 0.25 *len;

        den = twoth * ce + oneth * cs;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        gs = num / den;

        den = twoth * cs + oneth * ce;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        ge = num / den;

        error = N_VectorNormalizeRef( &Vs );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( &Ve );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorScale( Vs, gs, &Vs );
        N_VectorScale( Ve, ge, &Ve );

        /* Uniformly scale tangents to approximate internal points */

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, t );

        if( error EQ NL_YES )
            NL_OUT;

        sum = 0.0;

        for ( i = 1; i < k; i++ )
        {
            alf = t[i] * t[i] * (3.0 - 2.0 *t[i]);
            oma = 1.0 - alf;
            N_VectorCombine( oma, P[0], alf, P[k], &Vu );

            alf = t[i];
            oma = 1.0 - alf;
            N_VectorCombine( oma, Vs, -alf, Ve, &Tu );
            qu = oma * alf;

            N_VectorDiff( P[i], Vu, &Vd );
            N_VectorScale( Tu, qu, &Tv );
            N_VectorMagnitude( Vd, &num );
            N_VectorMagnitude( Tv, &den );

            if( fabs( num - den )LT NL_MTOL )
            {
                lam = 1.0;
            }
            else
            {
                if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );
                lam = num / den;
            }

            sum += lam;
        }

        lam = sum / (k - 1.0);

        N_VectorScale( Vs, lam, &Vs );
        N_VectorScale( Ve, lam, &Ve );

        N_VectorPtAlongVector( P[0], oneth, Vs, &Q1 );
        N_VectorPtAlongVector( P[k], -oneth, Ve, &Q2 );

        N_PtToCPt( Q1, &Pw[1] );
        N_PtToCPt( Q2, &Pw[2] );

        /* Estimate initial knot vector */

        x[0] = t[0];
        x[k] = t[k];

        for ( i = 1; i <= k - 1; i++ )
        {
            sum = 0.0;

            for ( j = i - 1; j <= i + 1; j++ )
                sum += t[j];
            x[i] = oneth * sum;
        }

        if( rfl EQ NL_YES )
        {
            y[0] = x[0];

            for ( i = 1; i <= k; i++ )
            {
                j = 2 * i;
                y[j - 1] = 0.5 *( x[i - 1] + x[i] );
                y[j] = x[i];
            }
        }

        if( rfl EQ NL_NO )
        {
            N_KnotVectorFromRealArray( &knx, &x[1], k - 2 );
        }
        else
        {
            N_KnotVectorFromRealArray( &knx, &y[1], 2 * k - 2 );
        }

        error = N_CrvRefine( &curB, &knx, &curB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( bfl EQ NL_INTAPPR )
    {
        /* Interpolate with a local method */

        N_CrvInitArrays( &curI );
        error = N_FitCrvCubicTangents( P, k, &Vs, &Ve, NL_BESSEL, &curI, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Sample curve */

        error = N_ApproxCrvWithPolyline( &curI, dtl, NL_RELATIVE, NL_BOTH, &PP, &up, &np, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Now approximate points */

        kni = NULL;
        N_CrvInitArrays( &curB );
        error = N_FitCrvApproxKnotsTangents( PP, np, up, &Vs, &Ve, NL_TANGENT, &kni, 1.0, 2 * k, 3, &curB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Shape cubic curve to pass through the given points */

    error = N_CrvShapeInterp( &curB, Q, k - 2, 1, 1, nit, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVCUBIC: Curve interpolation with NL_C1 non-rational cubic curves    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes a C1 continuous non-rational cubic
     NURBS curve interpolating  a given set  of points. If  the output 
     curve is  initialized to  the NULL  curve,  memory  is  allocated 
     locally. Otherwise it is checked if enough memory is passed in. A 
     typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvCubic(P,k,NL_AKIMA,&cur,&SG);

     A recommended  default for the  tangent is NL_AKIMA.


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k    , input  ,  Highest index in P
     tan  , input  ,  Flag:
                        NL_BESSEL: Tangents computed by Bessel's method
                        NL_AKIMA : Tangents computed by Akima's method
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvCubic( NL_POINT *P, NL_INDEX k, NL_FLAG tan, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvCubic");

    NL_FLAG closed, error = NL_NO;

    NL_INDEX i, n, m;

    NL_REAL *U, dui, dui1, alf, oma, a, b, c, disc;

    NL_PARAMETER *u;

    NL_POINT P1, P2;

    NL_CPOINT *Pw;

    NL_VECTOR *S, *T, A;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for too few input points */

    if( k LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Check for curve memory */

    n = 2 * k + 1;

    error = N_CrvSizeArrays( cur, n, 3, n + 4, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( k + 3, &SL );

    if( S EQ NULL )
        NL_QUIT;

    T = N_AllocPt1dArray( k, &SL );

    if( T EQ NULL )
        NL_QUIT;

    /* Compute the chords */

    N_DistPtPt( P[0], P[k], &a );

    if( a LT NL_MTOL )
        closed = NL_YES;
    else
        closed = NL_NO;

    for ( i = 1; i <= k; i++ )
        N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

    if( closed EQ NL_YES )
    {
        N_VectorCopy( S[k + 1], &S[1] );
        N_VectorCopy( S[k], &S[0] );
        N_VectorCopy( S[2], &S[k + 2] );
        N_VectorCopy( S[3], &S[k + 3] );
    }
    else
    {
        N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
        N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
        N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
        N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
    }

    /* Compute the unit tangents */

    switch( tan )
    {
        case NL_BESSEL:

            error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, u );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 1; i < k; i++ )
            {
                dui = u[i] - u[i - 1];
                dui1 = u[i + 1] - u[i];

                if( N_FloatOpIsBad( dui, dui + dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = dui / (dui + dui1);
                oma = 1.0 - alf;

                if( N_FloatOpIsBad( oma, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( alf, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = oma / dui;
                alf = alf / dui1;

                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }

            dui = u[1] - u[0];
            dui1 = u[k] - u[k - 1];

            if( closed EQ NL_YES )
            {
                if( N_FloatOpIsBad( 1.0, 2.0 *dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 1.0, 2.0 *dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 1.0 / (2.0 *dui);
                alf = 1.0 / (2.0 *dui1);

                N_VectorCombine( oma, S[2], alf, S[k + 1], &T[0] );
                N_VectorCopy( T[0], &T[k] );
            }
            else
            {
                if( N_FloatOpIsBad( 2.0, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 2.0, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 2.0 / dui;
                alf = 2.0 / dui1;

                N_VectorCombine( oma, S[2], -1.0, T[1], &T[0] );
                N_VectorCombine( alf, S[k + 1], -1.0, T[k - 1], &T[k] );
            }
            break;

        case NL_AKIMA:
            for ( i = 0; i <= k; i++ )
            {
                N_VectorCross( S[i], S[i + 1], &A );
                N_VectorMagnitude( A, &a );
                N_VectorCross( S[i + 2], S[i + 3], &A );
                N_VectorMagnitude( A, &b );

                if( (a + b)GT NL_LTOL )
                    alf = a / (a + b);
                else
                    alf = 0.5;
                oma = 1.0 - alf;
                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= k; i++ )
    {
        error = N_VectorNormalizeRef( &T[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Interpolate segment by segment */

    n = 0;
    u[0] = 0.0;

    N_PtToCPt( P[0], &Pw[0] );

    for ( i = 1; i <= k; i++ )
    {
        N_VectorSum( T[i - 1], T[i], &A );
        N_VectorDot( A, A, &a );
        N_VectorDot( S[i + 1], A, &b );
        N_VectorDot( S[i + 1], S[i + 1], &c );

        a = 16.0 - a;
        b = 12.0 *b;
        c = -36.0 *c;

        disc = b * b - 4.0 *a * c;

        if( disc LT 0.0 )
            NL_ERROR( NL_NUM_ERR );
        disc = sqrt( disc );

        if( N_FloatOpIsBad( 0.5 *( -b + disc ), a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        alf = 0.5 *( -b + disc ) / a;

        if( alf LT 0.0 )
        {
            if( N_FloatOpIsBad( 0.5 *( -b - disc ), a, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            alf = 0.5 *( -b - disc ) / a;
        }
        alf = alf / 3.0;

        N_VectorPtAlongVector( P[i - 1], alf, T[i - 1], &P1 );
        N_VectorPtAlongVector( P[i], -alf, T[i], &P2 );

        N_PtToCPt( P1, &Pw[++n] );
        N_PtToCPt( P2, &Pw[++n] );

        N_DistPtPt( P1, P[i - 1], &a );
        u[i] = u[i - 1] + 3.0 *a;
    }

    N_PtToCPt( P[k], &Pw[++n] );

    /* Compute the knot vector */

    m = -1;

    for ( i = 0; i <= 3; i++ )
        U[++m] = 0.0;

    for ( i = 1; i < k; i++ )
    {
        a = u[i] / u[k];
        U[++m] = a;
        U[++m] = a;
    }

    for ( i = 0; i <= 3; i++ )
        U[++m] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVCUBICTANGENTS: Local NL_C1 cubic curve interpolation with end tangents     */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes a C1 continuous non-rational cubic
     NURBS curve interpolating  a given set  of points, and tangential
     to given end tangents. If  the output curve is initialized to the 
     NULL  curve, memory is allocated locally. Otherwise it is checked 
     if enough memory is passed in. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k;
       NL_VECTOR  Ts, Te;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvCubicTangents(P,k,&Ts ,&Te ,NL_AKIMA ,&cur,&SG);
       N_FitCrvCubicTangents(P,k,NULL,NULL,NL_BESSEL,&cur,&SG);


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k    , input  ,  Highest index in P
     Ts   , input  ,  Start tangent:
                       !NULL: interpolate with start tangent Ts
                        NULL: compute start tangent internally
     Te   , input  ,  End tangent:
                       !NULL: interpolate with end tangent Te
                        NULL: compute end tangent internally
     tan  , input  ,  Flag:
                        NL_BESSEL: Unspecified  tangents are  computed by 
                                Bessel's method
                        NL_AKIMA : Unspecified  tangents are  computed by 
                                Akima's method
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvCubicTangents( NL_POINT *P, NL_INDEX k, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG tan, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvCubicTangents");

    NL_FLAG closed, error = NL_NO;

    NL_INDEX i, n, m;

    NL_REAL *U, dui, dui1, alf, oma, a, b, c, disc;

    NL_PARAMETER *u;

    NL_POINT P1, P2;

    NL_CPOINT *Pw;

    NL_VECTOR *S, *T, A;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for too few input points */

    if( k LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Check for curve memory */

    n = 2 * k + 1;

    error = N_CrvSizeArrays( cur, n, 3, n + 4, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( k + 3, &SL );

    if( S EQ NULL )
        NL_QUIT;

    T = N_AllocPt1dArray( k, &SL );

    if( T EQ NULL )
        NL_QUIT;

    /* Compute the chords */

    N_DistPtPt( P[0], P[k], &a );

    if( a LT NL_MTOL )
        closed = NL_YES;
    else
        closed = NL_NO;

    for ( i = 1; i <= k; i++ )
        N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

    if( closed EQ NL_YES )
    {
        N_VectorCopy( S[k + 1], &S[1] );
        N_VectorCopy( S[k], &S[0] );
        N_VectorCopy( S[2], &S[k + 2] );
        N_VectorCopy( S[3], &S[k + 3] );
    }
    else
    {
        N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
        N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
        N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
        N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
    }

    /* Compute the unit tangents */

    switch( tan )
    {
        case NL_BESSEL:

            error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, NL_CHORDLENGTH, u );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 1; i < k; i++ )
            {
                dui = u[i] - u[i - 1];
                dui1 = u[i + 1] - u[i];

                if( N_FloatOpIsBad( dui, dui + dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = dui / (dui + dui1);
                oma = 1.0 - alf;

                if( N_FloatOpIsBad( oma, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( alf, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = oma / dui;
                alf = alf / dui1;

                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }

            dui = u[1] - u[0];
            dui1 = u[k] - u[k - 1];

            if( closed EQ NL_YES )
            {
                if( N_FloatOpIsBad( 1.0, 2.0 *dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 1.0, 2.0 *dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 1.0 / (2.0 *dui);
                alf = 1.0 / (2.0 *dui1);

                N_VectorCombine( oma, S[2], alf, S[k + 1], &T[0] );
                N_VectorCopy( T[0], &T[k] );
            }
            else
            {
                if( N_FloatOpIsBad( 2.0, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 2.0, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 2.0 / dui;
                alf = 2.0 / dui1;

                N_VectorCombine( oma, S[2], -1.0, T[1], &T[0] );
                N_VectorCombine( alf, S[k + 1], -1.0, T[k - 1], &T[k] );
            }
            break;

        case NL_AKIMA:
            for ( i = 0; i <= k; i++ )
            {
                N_VectorCross( S[i], S[i + 1], &A );
                N_VectorMagnitude( A, &a );
                N_VectorCross( S[i + 2], S[i + 3], &A );
                N_VectorMagnitude( A, &b );

                if( (a + b)GT NL_LTOL )
                    alf = a / (a + b);
                else
                    alf = 0.5;
                oma = 1.0 - alf;
                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Enforce end tangents and normalize all tangents */

    if( Ts NEQ NULL )
        N_VectorCopy( *Ts, &T[0] );

    if( Te NEQ NULL )
        N_VectorCopy( *Te, &T[k] );

    for ( i = 0; i <= k; i++ )
    {
        error = N_VectorNormalizeRef( &T[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Interpolate segment by segment */

    n = 0;
    u[0] = 0.0;

    N_PtToCPt( P[0], &Pw[0] );

    for ( i = 1; i <= k; i++ )
    {
        N_VectorSum( T[i - 1], T[i], &A );
        N_VectorDot( A, A, &a );
        N_VectorDot( S[i + 1], A, &b );
        N_VectorDot( S[i + 1], S[i + 1], &c );

        a = 16.0 - a;
        b = 12.0 *b;
        c = -36.0 *c;

        disc = b * b - 4.0 *a * c;

        if( disc LT 0.0 )
            NL_ERROR( NL_NUM_ERR );
        disc = sqrt( disc );

        if( N_FloatOpIsBad( 0.5 *( -b + disc ), a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        alf = 0.5 *( -b + disc ) / a;

        if( alf LT 0.0 )
        {
            if( N_FloatOpIsBad( 0.5 *( -b - disc ), a, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            alf = 0.5 *( -b - disc ) / a;
        }
        alf = alf / 3.0;

        N_VectorPtAlongVector( P[i - 1], alf, T[i - 1], &P1 );
        N_VectorPtAlongVector( P[i], -alf, T[i], &P2 );

        N_PtToCPt( P1, &Pw[++n] );
        N_PtToCPt( P2, &Pw[++n] );

        N_DistPtPt( P1, P[i - 1], &a );
        u[i] = u[i - 1] + 3.0 *a;
    }

    N_PtToCPt( P[k], &Pw[++n] );

    /* Compute the knot vector */

    m = -1;

    for ( i = 0; i <= 3; i++ )
        U[++m] = 0.0;

    for ( i = 1; i < k; i++ )
    {
        a = u[i] / u[k];
        U[++m] = a;
        U[++m] = a;
    }

    for ( i = 0; i <= 3; i++ )
        U[++m] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVCONICS: Curve interpolation with piecewise conic arcs            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS piecewise conic curve inter-
     polating a given set  of points. The parametrization is G1. If the 
     output curve is initialized to the NULL curve, memory is allocated  
     locally. Otherwise  it is checked if enough memory is passed in. A 
     typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvConics(P,k,NL_CHORDLENGTH,NL_AKIMA,NL_CUSP,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k    , input  ,  Highest index in P
     par  , input  ,  Flag:
                        NL_UNIFORM    : Uniform parametrization
                        NL_CHORDLENGTH: Chord length parametrization 
                        NL_CENTRIPETAL: Centripetal parametrization 
     tan  , input  ,  Flag:
                        NL_BESSEL: Tangents computed by Bessel's method
                        NL_AKIMA : Tangents computed by Akima's method
     csp  , input  ,  Flag:
                        NL_CUSP  : Maintain cusps/corners 
                        NL_NOCUSP: Smooth out corners
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvConics( NL_POINT *PP, NL_INDEX kk, NL_FLAG par, NL_FLAG tan, NL_FLAG csp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvConics");

    NL_FLAG closed, its, type, reset = NL_NO, error = NL_NO;

    NL_INDEX i, j, l, n, m, k;

    NL_REAL *U, d, dui, dui1, alf, oma, a, b, cosi, cosi1, gam, gam1, w, cw;

    NL_PARAMETER *u;

    NL_POINT *Q, R, M, Ri, Ri1, *P;

    NL_CPOINT *Pw;

    NL_VECTOR *S, *SU, *T, A;

    NL_LINESEG li1, li;

    NL_STACKS SL;

    NL_REAL ALFA = 0.66;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for too few input points */

    if( kk LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Make copy of point array, since we may have to overwrite */

    k = kk;
    P = N_AllocPt1dArray( kk, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
        N_CopyPt( PP[i], &P[i] );

    /* Check for curve memory */

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    cw = 0.5 *sqrt( 2.0 );
    n = 4 * k;

    error = N_CrvSizeArrays( cur, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( 2 * k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( 2 * k, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( k + 3, &SL );

    if( S EQ NULL )
        NL_QUIT;

    SU = N_AllocPt1dArray( k + 3, &SL );

    if( SU EQ NULL )
        NL_QUIT;

    T = N_AllocPt1dArray( k, &SL );

    if( T EQ NULL )
        NL_QUIT;

    /* Compute the chords and normalize them */

    N_DistPtPt( P[0], P[k], &d );

    if( d LT NL_MTOL )
        closed = NL_YES;
    else
        closed = NL_NO;

    for ( i = 1; i <= k; i++ )
        N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

    if( closed EQ NL_YES )
    {
        N_VectorCopy( S[k + 1], &S[1] );
        N_VectorCopy( S[k], &S[0] );
        N_VectorCopy( S[2], &S[k + 2] );
        N_VectorCopy( S[3], &S[k + 3] );
    }
    else
    {
        N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
        N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
        N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
        N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
    }

    for ( i = 2; i <= k + 1; i++ )
    {
        error = N_VectorNormalize( S[i], &SU[i], &a );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute the unit tangents */

    switch( tan )
    {
        case NL_BESSEL:

            error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, par, u );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 1; i < k; i++ )
            {
                dui = u[i] - u[i - 1];
                dui1 = u[i + 1] - u[i];

                if( N_FloatOpIsBad( dui, dui + dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = dui / (dui + dui1);
                oma = 1.0 - alf;

                if( N_FloatOpIsBad( oma, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( alf, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = oma / dui;
                alf = alf / dui1;

                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }

            dui = u[1] - u[0];
            dui1 = u[k] - u[k - 1];

            if( closed EQ NL_YES )
            {
                if( N_FloatOpIsBad( 1.0, 2.0 *dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 1.0, 2.0 *dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 1.0 / (2.0 *dui);
                alf = 1.0 / (2.0 *dui1);

                N_VectorCombine( oma, S[2], alf, S[k + 1], &T[0] );
                N_VectorCopy( T[0], &T[k] );
            }
            else
            {
                if( N_FloatOpIsBad( 2.0, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 2.0, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 2.0 / dui;
                alf = 2.0 / dui1;

                N_VectorCombine( oma, S[2], -1.0, T[1], &T[0] );
                N_VectorCombine( alf, S[k + 1], -1.0, T[k - 1], &T[k] );
            }
            break;

        case NL_AKIMA:
            for ( i = 0; i <= k; i++ )
            {
                N_VectorCross( S[i], S[i + 1], &A );
                N_VectorMagnitude( A, &a );
                N_VectorCross( S[i + 2], S[i + 3], &A );
                N_VectorMagnitude( A, &b );

                if( (a + b)LT NL_LTOL )
                {
                    if( csp EQ NL_CUSP )
                        alf = 1.0;
                    else
                        alf = 0.5;
                }
                else
                {
                    alf = a / (a + b);
                }

                oma = 1.0 - alf;
                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= k; i++ )
    {
        error = N_VectorNormalizeRef( &T[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Interpolate segment by segment */

    n = 0;
    l = 0;

    N_Weight( P[0], 1.0, &Pw[0] );
    N_CopyPt( P[0], &Q[0] );

    for ( i = 1; i <= k; i++ )
    {
        /* Establish the various cases */

        N_CreateLineStartDirVector( &li1, P[i - 1], T[i - 1], NL_UNBOUNDED );
        N_CreateLineStartDirVector( &li, P[i], T[i], NL_UNBOUNDED );

        error = N_IsectLineLine( li1, li, &R, &a, &b, &its );

        if( error EQ NL_YES )
            NL_OUT;

        if( its EQ NL_TRUE ) /* Lines intersect */
        {
            if( a GT 0.0 AND b LT 0.0 )
            {
                type = 1; /* One arc segment */
            }
            else if( fabs( a )LT NL_PTOL OR fabs( b )LT NL_PTOL )   /* gwc: was NL_LTOL - but this is not an aangle - maybe NL_PTOL is not right */
            {
                if( csp EQ NL_CUSP )
                    type = 2;
                else          /* Corner needed */
                    type = 3; /* Split arc     */
            }
            else
            {
                type = 3; /* Split arc */
            }
        }
        else /* Lines do not intersect */
        {
            N_VectorDot( T[i - 1], T[i], &a );
            N_VectorCross( T[i - 1], SU[i + 1], &A );
            N_VectorMagnitude( A, &b );

            if( b GT NL_LTOL )
            {
                if( a GT 0.0 )
                    type = 3;
                else          /* Parallel tangents     */
                    type = 4; /* Antiparallel tangents */
            }
            else              /* Collinear points */
            {
                if( i EQ k )
                {
                    type = 6; /* No points discarded */
                }
                else
                {
                    N_VectorCross( T[i], T[i + 1], &A );
                    N_VectorMagnitude( A, &b );

                    if( b GT NL_LTOL )
                    {
                        type = 6; /* No points discarded */
                    }
                    else
                    {
                        N_VectorCross( SU[i + 1], SU[i + 2], &A );
                        N_VectorMagnitude( A, &b );

                        if( b LT NL_LTOL )
                            type = 5;
                        else          /* Points discarded    */
                            type = 6; /* No points discarded */
                    }
                }
            }
        }

        /* Switch to different types of segments */

        switch( type )
        {
            case 1: /* Single arc */

                N_BezSetConicWeight( P[i - 1], R, P[i], &w );

                N_Weight( R, w, &Pw[++n] );
                N_Weight( P[i], 1.0, &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 2: /* Corner needed */

                N_Combine2Pts( 0.5, P[i - 1], 0.5, P[i], &R );

                N_Weight( R, 1.0, &Pw[++n] );
                N_Weight( P[i], 1.0, &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 3: /* Split segment */

                N_DistPtPt( P[i - 1], P[i], &d );

                N_VectorDot( T[i - 1], SU[i + 1], &cosi1 );
                N_VectorDot( T[i], SU[i + 1], &cosi );

                gam1 = 0.25 *d / (1.0 + ALFA * fabs( cosi ) + (1.0 - ALFA) * fabs( cosi1 ));
                gam = 0.25 *d / (1.0 + ALFA * fabs( cosi1 ) + (1.0 - ALFA) * fabs( cosi ));
                alf = gam1 / (gam + gam1);
                oma = 1.0 - alf;

                N_VectorPtAlongVector( P[i - 1], gam1, T[i - 1], &Ri1 );
                N_VectorPtAlongVector( P[i], -gam, T[i], &Ri );
                N_Combine2Pts( oma, Ri1, alf, Ri, &M );

                N_BezSetConicWeight( P[i - 1], Ri1, M, &w );

                N_Weight( Ri1, w, &Pw[++n] );
                N_Weight( M, 1.0, &Pw[++n] );
                N_CopyPt( M, &Q[++l] );

                N_BezSetConicWeight( M, Ri, P[i], &w );

                N_Weight( Ri, w, &Pw[++n] );
                N_Weight( P[i], 1.0, &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 4: /* Antiparallel tangents */

                N_Combine2Pts( 0.5, P[i - 1], 0.5, P[i], &M );
                N_DistPtPt( P[i - 1], M, &gam );

                N_VectorPtAlongVector( P[i - 1], gam, T[i - 1], &Ri1 );
                N_VectorPtAlongVector( M, gam, T[i - 1], &M );
                N_VectorPtAlongVector( P[i], gam, T[i - 1], &Ri );

                N_Weight( Ri1, cw, &Pw[++n] );
                N_Weight( M, 1.0, &Pw[++n] );
                N_CopyPt( M, &Q[++l] );

                N_Weight( Ri, cw, &Pw[++n] );
                N_Weight( P[i], 1.0, &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 5: /* Discard point - shift down data */
                for ( j = i; j < k; j++ )
                {
                    N_CopyPt( P[j + 1], &P[j] );
                    N_VectorCopy( T[j + 1], &T[j] );
                }

                for ( j = i; j <= k + 1; j++ )
                    N_VectorCopy( SU[j + 2], &SU[j + 1] );

                k--;
                i--;
                break;

            case 6: /* Collinear data - output line segment */

                N_Combine2Pts( 0.5, P[i - 1], 0.5, P[i], &R );

                N_Weight( R, 1.0, &Pw[++n] );
                N_Weight( P[i], 1.0, &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            default:

                NL_ERROR( NL_NUM_ERR );
        } /* End of switch */
    }     /* End of loop */

    /* Compute knot vector */

    m = 2;

    for ( i = 0; i <= 2; i++ )
        U[i] = 0.0;

    error = N_FitCalcCrvParamValues( (NL_VOID *)Q, l, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 1; i < l; i++ )
    {
        U[++m] = u[i];
        U[++m] = u[i];
    }

    for ( i = 0; i <= 2; i++ )
        U[++m] = 1.0;

    /* Set curve indexes and compact curve */

    N_CrvSetSizeIndices( cur, n, 2, m );

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVCONICSAPPROX: Data approximation with piecewise conic segments         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes a quadratic NURBS curve approxima-
     ting a given set of points. This is a local method producing a G1
     continuous piecewise conic curve. If tangent data is available at
     each  data point, they can  be passed in. Otherwise  tangents are
     computed locally. If the output curve is initialized to  the NULL 
     curve, memory  is allocated  locally. Otherwise  it is checked if 
     enough memory is passed in. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k, maxk;
       NL_REAL    E;
       NL_VECTOR  *T,
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P, T, and choose maxk and E);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvConicsApprox(P,k,T,NL_YES,maxk,E,&cur,&SG);

     If cur is initialized to NULL, memory is  allocated twice. First,
     enough memory is allocated for worst-case approximation, i.e. for
     interpolation.  After  approximation  is   completed,  memory  is 
     reallocated using the proper number of control  points and knots.
     THE SPEED OF THE  SEARCH PROCESS DEPENDS ON maxk. A GOOD ESTIMATE
     IS TO SET  maxk = 20%  OF THE  NUMBER OF  NL_POINTS  FOR 1% OF NL_ERROR
     TOLERANCE. FOR  EXAMPLE, IF  THE  NL_CURVE  IS IN THE [0,1] BOX, THE 
     NUMBER OF DATA NL_POINTS IS 1000, AND  THE NL_ERROR IS 0.01, maxk = 200 
     IS A GOOD ESTIMATE. FOR  SMALLER NL_ERROR TOLERANCES maxk  SHOULD BE 
     DECREASED  PROPORTIONALLY. THIS  ROUTINE  WORKS  WELL WITH PLANAR 
     DATA (NOT NECESSARILY X-Y PLANAR). FOR  3-D DATA THE NL_CURVE CANNOT
     BE GUARANTEED TO BE NL_G1 CONTINUOUS.


   ACCESS:
   
     P    , input  ,  Points to be approximated
     k    , input  ,  Highest index in P
     T    , input  ,  Tangents at each point
     tan  , input  ,  Flag:
                        NL_YES: Compute tangents
                        NL_NO : Tangents are passed in
     maxk , input  ,  Maximum number of points to be  approximated by
                      one segment
     E    , input  ,  Approximation tolerance
     cur  , output ,  Approximating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvConicsApprox( NL_POINT *P, NL_INDEX k, NL_VECTOR *T, NL_FLAG tan, NL_INDEX maxk, NL_REAL E, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvConicsApprox");

    NL_FLAG reset = NL_NO, error = NL_NO, closed = NL_NO;

    NL_INDEX i, j, l, le = 0, n, m, ks, ke, kl, kr;

    NL_REAL *U, alf, oma, a, b, d;

    NL_PARAMETER *u;

    NL_POINT *Q;

    NL_CPOINT *Pw;

    NL_VECTOR *S, A;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL top = 1.0e-04;
    NL_PRIVATE NL_REAL toc = 1.0e-06;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    n = 4 * k;
    error = N_CrvSizeArrays( cur, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Compute the tangents if needed */

    if( tan EQ NL_YES )
    {
        S = N_AllocPt1dArray( k + 3, &SL );

        if( S EQ NULL )
            NL_QUIT;

        T = N_AllocPt1dArray( k, &SL );

        if( T EQ NULL )
            NL_QUIT;

        N_DistPtPt( P[0], P[k], &d );

        if( d LT NL_MTOL )
            closed = NL_YES;
        else
            closed = NL_NO;

        for ( i = 1; i <= k; i++ )
            N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

        if( closed EQ NL_YES )
        {
            N_VectorCopy( S[k + 1], &S[1] );
            N_VectorCopy( S[k], &S[0] );
            N_VectorCopy( S[2], &S[k + 2] );
            N_VectorCopy( S[3], &S[k + 3] );
        }
        else
        {
            N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
            N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
            N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
            N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
        }

        for ( i = 0; i <= k; i++ )
        {
            N_VectorCross( S[i], S[i + 1], &A );
            N_VectorMagnitude( A, &a );
            N_VectorCross( S[i + 2], S[i + 3], &A );
            N_VectorMagnitude( A, &b );

            if( (a + b)GT NL_LTOL )
                alf = a / (a + b);
            else
                alf = 0.5;

            oma = 1.0 - alf;
            N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );

            error = N_VectorNormalizeRef( &T[i] );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Approximate data */

    ks = 0;
    n = 0;
    m = -1;

    while( ks LT k )
    {
        if( maxk LT k )
            ke = NL_MIN( (ks + maxk), k );

        else if( closed EQ NL_NO )
            ke = NL_MIN( (ks + maxk), k );

        else
            ke = k / 2;

        kl = ks;
        kr = ke;

        while( kl NEQ ke )
        {
            error = N_FitConicToPts( P, ks, ke, T[ks], T[ke], E, top, toc, &Pw[n], &l );

            if( error EQ NL_YES )
                NL_OUT;

            if( l GT 0 )
            {
                kl = ke;
                le = l;
            }
            else
                kr = ke;

            ke = (kl + kr) / 2;
        }

        n += le;
        ks = ke;
        closed = NL_NO;
    }

    N_CrvSetSizeIndices( cur, n, 2, n + 3 );

    /* Compute knot vector */

    l = n / 2;

    u = N_AllocReal1dArray( l, &SL );

    if( u EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( l, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= l; i++ )
    {
        j = 2 * i;

        N_CPtToPtEuclid( Pw[j], &Q[i] );
    }

    error = N_FitCalcCrvParamValues( (NL_VOID *)Q, l, NL_EPOINT, NL_CHORDLENGTH, u );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= 2; i++ )
        U[++m] = 0.0;

    for ( i = 1; i < l; i++ )
    {
        U[++m] = u[i];
        U[++m] = u[i];
    }

    for ( i = 0; i <= 2; i++ )
        U[++m] = 1.0;

    /* Compact curve */

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

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
/* N_FITREMOVEKNOTS: Remove all removable knots from an approximating curve   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting  routine  removes all  removable knots  from a NURBS 
     curve approximating, at supplied parameter values, a  given set of  
     points. An  error vector is updated as knots are removed in place, 
     ie the  original curve is destroyed. A typical calling example is:

       NL_CURVE  cur;
       NL_INDEX  mu;
       NL_REAL   *u, *er, E;
       ...
       (define cur, get arrays u and er);
       ...
       N_FitRemoveKnots(&cur,u,er,mu,E);

     This  routine is used in N_FITCRVAPPROX to  clean approximating  curves.

   ACCESS:
   
     cur , in/out ,  NURBS curve
     u   , input  ,  Parameter values at which error is to be checked
     er  , in/out ,  Error vector to be updated
     mu  , input  ,  Highest index in u and er
     E   , input  ,  Error tolerance


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitRemoveKnots( NL_CURVE *cur, NL_REAL *u, NL_REAL *er, NL_INDEX mu, NL_REAL E )
{

    NL_FLAG rmf, error = NL_NO;

    NL_INDEX *left, *right, *sr, i, j, k, l, ii, jj, first, last, off, fout, n, m, r, s;

    NL_DEGREE p;

    NL_REAL *U, *br, *te, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, N, N1;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Rw;

    NL_STACKS SL;

    NL_REAL NOREM = 1.0e+25;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur, &knt );

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

    te = N_AllocReal1dArray( mu, &SL );

    if( te EQ NULL )
        NL_QUIT;

    left = N_AllocInt1dArray( n, &SL );

    if( left EQ NULL )
        NL_QUIT;

    right = N_AllocInt1dArray( n, &SL );

    if( right EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( i = 0; i <= m; i++ )
    {
        br[i] = NL_BIGD;
        sr[i] = 0;
    }

    /* Compute knot removal errors for each distinct knot */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;
        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( cur, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Get ranges of parameter indexes */

    j = 1;

    for ( i = 0; i <= n; i++ )
    {
        left[i] = j;

        while( j LT mu AND u[j]GT U[i]AND u[j]LE U[i + 1] )
            j++;

        for ( k = j; k < mu; k++ )
        {
            if( u[k]GE U[i + p + 1] )
                break;
        }
        right[i] = k - 1;
    }

    /* Try to remove knots until cumulative error < E */

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

        rmf = NL_TRUE;

        if( (p + s) % 2 )
        {
            k = (p + s + 1) / 2;
            i = r - k;
            alf = (U[r] - U[i]) / (U[i + p + 1] - U[i]);
            bet = (U[r] - U[i + 1]) / (U[i + p + 2] - U[i + 1]);
            omb = 1.0 - bet;
            lam = alf / (alf + bet);
            oml = 1.0 - lam;

            for ( j = left[i]; j <= right[i + 1]; j++ )
            {
                error = N_BasisIEval( knt, i, p, u[j], NL_LEFT, &N );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_BasisIEval( knt, i + 1, p, u[j], NL_LEFT, &N1 );

                if( error EQ NL_YES )
                    NL_OUT;

                te[j] = er[j] + fabs( lam * alf * N - oml * omb * N1 ) * b;

                if( te[j]GT E )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }
        else
        {
            k = (p + s) / 2;
            i = r - k;

            for ( j = left[i]; j <= right[i]; j++ )
            {
                error = N_BasisIEval( knt, i, p, u[j], NL_LEFT, &N );

                if( error EQ NL_YES )
                    NL_OUT;

                te[j] = er[j] + N * b;

                if( te[j]GT E )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }

        /* If knot is removable, update error and remove knot */

        if( rmf EQ NL_TRUE )
        {
            if( (p + s) % 2 )
                l = right[i + 1];
            else
                l = right[i];

            for ( j = left[i]; j <= l; j++ )
                er[j] = te[j];

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;
            i = first;
            j = last;
            ii = 1;
            jj = last - off;

            N_CopyCPt( Pw[off], &Rw[0] );
            N_CopyCPt( Pw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                oma = 1.0 - alf;
                bet = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Pw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Pw[j], omb, Rw[jj + 1], &Rw[jj] );
                i++;
                j--;
                ii++;
                jj--;
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
                N_CopyCPt( Rw[i - off], &Pw[i] );
                N_CopyCPt( Rw[j - off], &Pw[j] );
                i++;
                j--;
            }

            /* Shift down some parameters */

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( i = r + 1; i <= m; i++ )
            {
                br[i - 1] = br[i];
                sr[i - 1] = sr[i];
            }

            /* Shift down knots and control points */

            for ( i = r + 1; i <= m; i++ )
                U[i - 1] = U[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Pw[i], &Pw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( cur, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( U[i]NEQ U[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( cur, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            /* Update index ranges */

            for ( i = r - p - 1; i <= r - s; i++ )
            {
                for ( k = right[i] + 1; k <= mu; k++ )
                {
                    if( u[k]GE U[i + p + 1] )
                        break;
                }
                right[i] = k - 1;
            }

            for ( i = r - s + 1; i <= n; i++ )
            {
                left[i] = left[i + 1];
                right[i] = right[i + 1];
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}


/**********************************************************************/
/* N_FITREMOVEKNOTSPRIORITIES: Remove knots with derivative constraints and priorities  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting  routine  removes all  removable knots  from a NURBS 
     curve approximating, at supplied parameter values, a  given set of  
     points. An  error vector is updated as knots are removed in place, 
     i.e. the  original  curve is  destroyed.  This  version  maintains 
     derivatives at  either end or both ends of the curve, and  removes
     the  knots in the  order specified by the  user. A typical calling 
     example is:

       NL_CURVE  cur;
       NL_INDEX  *pr, mu, der;
       NL_REAL   *u, *er, E;
       ...
       (define cur and der, get arrays u, pr and er);
       ...
       N_FitRemoveKnotsPriorities(&cur,pr,u,er,mu,E,NL_BOTH,der);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     pr  , input  ,  Priority array, i.e. pr[i] is the  priority of the
                     knot U[i]. The following restrictions apply:
                     1. each knot must have a priority
                     2. for  multiple knots the  right most  knots must 
                        have  higher  priorities. That is, for a triple
                        knot the multiplicities  from left to right may 
                        be 4 3 1
                     The interpretation of priorities is as follows:
                     pr = 0: do not remove knots with zero priority
                          1: try to remove  ALL knots with priority one
                             first
                          2: try to remove  ALL knots with priority two
                             second
                          etc...
     u   , input  ,  Parameter values at which error is to be checked
     er  , in/out ,  Error vector to be updated
     mu  , input  ,  Highest index in u and er
     E   , input  ,  Error tolerance
     whr , input  ,  Flag:
                       NL_NO   : don't constrain end derivatives
                       NL_START: constrain at the start point
                       NL_END  : constrain at the end point
                       NL_BOTH : constrain at both ends
     der , input  ,  Highest  derivative  constraint, i.e. the  routine
                     maintains the  1-st to the  der-th  derivatives at 
                     the specified end point(s)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitRemoveKnotsPriorities( NL_CURVE *cur, NL_INDEX *pr, NL_REAL *u, NL_REAL *er, NL_INDEX mu, NL_REAL E, NL_FLAG whr, NL_INDEX der )
{

    NL_FLAG rmf, error = NL_NO;

    NL_INDEX *left, *right, *sr, i, j, k, l, ih, ii, jj, first, last, off, fout, n, m, r, s = 0, t;

    NL_DEGREE p;

    NL_REAL *U, *br, *te, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, N, N1;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Rw;

    NL_STACKS SL;

    NL_REAL NOREM = 1.0e+25;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur, &knt );

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

    te = N_AllocReal1dArray( mu, &SL );

    if( te EQ NULL )
        NL_QUIT;

    left = N_AllocInt1dArray( n, &SL );

    if( left EQ NULL )
        NL_QUIT;

    right = N_AllocInt1dArray( n, &SL );

    if( right EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( i = 0; i <= m; i++ )
    {
        br[i] = NL_BIGD;
        sr[i] = 0;
    }

    /* Compute knot removal errors for each distinct knot */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;
        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( cur, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Get ranges of parameter indexes */

    j = 1;

    for ( i = 0; i <= n; i++ )
    {
        left[i] = j;

        while( j LT mu AND u[j]GT U[i]AND u[j]LE U[i + 1] )
            j++;

        for ( k = j; k < mu; k++ )
        {
            if( u[k]GE U[i + p + 1] )
                break;
        }
        right[i] = k - 1;
    }

    /* Adjust error bounds based on priorities */

    ih = 0;

    for ( i = 0; i <= m; i++ )
    {
        if( pr[i]GT ih )
            ih = pr[i];

        if( pr[i]EQ 0 )
            br[i] = NOREM;
    }

    /* Try to remove knots until cumulative error < E */

    t = 1;

    while( NL_TRUE )
    {
        /* Find knot with priority t and smallest error */

        b = NOREM;

        for ( i = p + 1; i <= n; i++ )
        {
            if( pr[i]EQ t AND br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot with priority t, consider next priority */

        if( b EQ NL_BIGD OR b EQ NOREM )
        {
            if( t GE ih )
                break;
            else
            {
                t++;
                continue;
            }
        }

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
            k = (p + s + 1) / 2;
            i = r - k;
            alf = (U[r] - U[i]) / (U[i + p + 1] - U[i]);
            bet = (U[r] - U[i + 1]) / (U[i + p + 2] - U[i + 1]);
            omb = 1.0 - bet;
            lam = alf / (alf + bet);
            oml = 1.0 - lam;

            for ( j = left[i]; j <= right[i + 1]; j++ )
            {
                error = N_BasisIEval( knt, i, p, u[j], NL_LEFT, &N );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_BasisIEval( knt, i + 1, p, u[j], NL_LEFT, &N1 );

                if( error EQ NL_YES )
                    NL_OUT;

                te[j] = er[j] + fabs( lam * alf * N - oml * omb * N1 ) * b;

                if( te[j]GT E )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }
        else
        {
            k = (p + s) / 2;
            i = r - k;

            for ( j = left[i]; j <= right[i]; j++ )
            {
                error = N_BasisIEval( knt, i, p, u[j], NL_LEFT, &N );

                if( error EQ NL_YES )
                    NL_OUT;

                te[j] = er[j] + N * b;

                if( te[j]GT E )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }

        /* If knot is removable, update error and remove knot */

        if( rmf EQ NL_TRUE )
        {
            if( (p + s) % 2 )
                l = right[i + 1];
            else
                l = right[i];

            for ( j = left[i]; j <= l; j++ )
                er[j] = te[j];

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;
            i = first;
            j = last;
            ii = 1;
            jj = last - off;

            N_CopyCPt( Pw[off], &Rw[0] );
            N_CopyCPt( Pw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                oma = 1.0 - alf;
                bet = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Pw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Pw[j], omb, Rw[jj + 1], &Rw[jj] );
                i++;
                j--;
                ii++;
                jj--;
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
                N_CopyCPt( Rw[i - off], &Pw[i] );
                N_CopyCPt( Rw[j - off], &Pw[j] );
                i++;
                j--;
            }

            /* Shift down some parameters */

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( i = r + 1; i <= m; i++ )
            {
                br[i - 1] = br[i];
                sr[i - 1] = sr[i];
                pr[i - 1] = pr[i];
            }

            /* Shift down knots and control points */

            for ( i = r + 1; i <= m; i++ )
                U[i - 1] = U[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Pw[i], &Pw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( cur, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( U[i]NEQ U[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( cur, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            /* Update index ranges */

            for ( i = r - p - 1; i <= r - s; i++ )
            {
                for ( k = right[i] + 1; k <= mu; k++ )
                {
                    if( u[k]GE U[i + p + 1] )
                        break;
                }
                right[i] = k - 1;
            }

            for ( i = r - s + 1; i <= n; i++ )
            {
                left[i] = left[i + 1];
                right[i] = right[i + 1];
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITCRVDERIVMATRIX: Curve interpolation with end derivative and matrix       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes the control  points of a NURBS curve  
     interpolating a given set of points and one end derivative. This is 
     a special routine requiring a knot vector, an LU-decomposed matrix,
     and NL_POINT or NL_CPOINT data. A typical calling example:

       NL_POINT       *Q;
       NL_CPOINT      *Qw, *Pw;
       NL_KNOTVECTOR  knt;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_VECTOR      V;
       NL_CVECTOR     W;
       NL_RMATRIX     cm;
       ...
       (get Q/Qw,V, W, knt and cm);
       ...
       N_FitCrvDerivMatrix((NL_VOID *)Q ,k,NL_EPOINT,p,&knt,(NL_VOID *)&V,NL_START,&cm,Pw);
       N_FitCrvDerivMatrix((NL_VOID *)Qw,k,NL_HPOINT,p,&knt,(NL_VOID *)&W,NL_END  ,&cm,Pw);

     IT IS ASSUMED THAT MEMORY TO STORE Pw IS ALLOCATED IN THE CALLING ROUTINE!!


   ACCESS:
   
     A     , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     k     , input  ,  Highest index in A
     ptp   , input  ,  Flag:
                         NL_EPOINT: Euclidean point pointer passed in
                         NL_HPOINT: Homogeneous point pointer passed in
     p     , input  ,  Degree of  interpolating curve (must be <= k+1)
     knt   , input  ,  Knot vector of interpolating curve
     T     , input  ,  NL_VOID pointer   representing NL_VECTOR or  NL_CVECTOR as 
                       start or  end  derivative.  THEY  MUST  BE PASSED
                       IN BY REFERENCE AS THIS IS THE  ONLY WAY  TO CAST 
                       THEM TO NL_VOID.
     whr   , input  ,  Flag:
                         NL_START: T is start derivative
                         NL_END  : T is end derivative
     cm    , input  ,  LU-decomposed matrix
     Pw    , output ,  Control points of  interpolating curve. MUST HAVE
                       ENOUGH MEMORY TO HOLD Pw[0],...,Pw[k+1]!!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvDerivMatrix( NL_VOID *A, NL_INDEX k, NL_FLAG ptp, NL_DEGREE p, NL_KNOTVECTOR *knt, NL_VOID *T, NL_FLAG whr, NL_RMATRIX *cm, NL_CPOINT *Pw )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvDerivMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m;

    NL_REAL *U, fact;

    NL_POINT *Q, *R;

    NL_CPOINT *Qw, *Rw;

    NL_VECTOR V;

    NL_CVECTOR W;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get knot vector data */

    N_KnotVectorGetKnots( knt, &m, &U );
    n = k + 1;

    /* Solve system of linear equations */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)A;
            V = *(NL_VECTOR *)T;

            Q = N_AllocPt1dArray( n, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            switch( whr )
            {
                case NL_START:

                    fact = (U[p + 1] - U[1]) / p;
                    N_CopyPt( R[0], &Q[0] );
                    N_ScalePt( fact, V, &Q[1] );

                    for ( i = 2; i <= n; i++ )
                        N_CopyPt( R[i - 1], &Q[i] );
                    break;

                case NL_END:

                    fact = (U[n + p] - U[n]) / p;
                    N_ScalePt( fact, V, &Q[n - 1] );
                    N_CopyPt( R[k], &Q[n] );

                    for ( i = 0; i <= n - 2; i++ )
                        N_CopyPt( R[i], &Q[i] );
                    break;

                default:
                    NL_ERROR( NL_CAL_ERR );
            }

            error = N_RealMatrixForBack( cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            W = *(NL_CVECTOR *)T;

            Qw = N_AllocCPt1dArray( n, &SL );

            if( Qw EQ NULL )
                NL_QUIT;

            switch( whr )
            {
                case NL_START:

                    fact = (U[p + 1] - U[1]) / p;
                    N_CopyCPt( Rw[0], &Qw[0] );
                    N_ScaleCPt( fact, W, &Qw[1] );

                    for ( i = 2; i <= n; i++ )
                        N_CopyCPt( Rw[i - 1], &Qw[i] );
                    break;

                case NL_END:

                    fact = (U[n + p] - U[n]) / p;
                    N_ScaleCPt( fact, W, &Qw[n - 1] );
                    N_CopyCPt( Rw[k], &Qw[n] );

                    for ( i = 0; i <= n - 2; i++ )
                        N_CopyCPt( Rw[i], &Qw[i] );
                    break;

                default:
                    NL_ERROR( NL_CAL_ERR );
            }

            error = N_RealMatrixForBack( cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVWEIGHTEDLSTSQ: Weighted & constrained least-squares curve approximation */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes a weighted  and constrained least-
     squares NURBS curve  approximation to a  given set of  points. The 
     degree is arbitrary and the type of parametrization and the number 
     of control  points can  be chosen  so as to  satisfy the following 
     conditions:
       (1) mc   < n,    where mc  is the  highest index  of constrained 
                        data,  and n  is the  highest index  of control 
                        points; and
       (2) mc+n < mu+1, where  mu is the highest index of unconstrained
                        data. 
     If  the output  curve is  initialized to the NULL curve, memory is 
     allocated  locally. Otherwise  it is checked  if enough  memory is 
     passed in. A typical calling example is:

       NL_POINT   *P;
       NL_VECTOR  *D;
       NL_REAL    *wp, *wd;
       NL_INDEX   *I, r, s, n;
       NL_DEGREE  p;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define arrays P, D, wp, wd and I, and choose p and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvWeightedLstSq(P,wp,r,D,wd,I,s,n,p,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P   , input  ,  Points to be approximated/interpolated
     wp  , input  ,  Array of point weights:
                       wp[i] > 0.0: P[i] is unconstrained
                       wp[i] < 0.0: P[i] is constrained
     r   , input  ,  Highest index in P
     D   , input  ,  Derivatives to be approximated/interpolated
     wd  , input  ,  Array of derivative weights:
                       wd[j] > 0.0: D[j] is unconstrained
                       wd[j] < 0.0: D[j] is constrained
     I   , input  ,  Index array; the derivative at P[I[j]] is D[j]
     s   , input  ,  Highest index of D, wd and I
     n   , input  ,  Highest   index   of   control   point   array  of 
                     approximating curve 
     p   , input  ,  Degree of approximating curve 
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvWeightedLstSq( NL_POINT *P, NL_REAL *wp, NL_INDEX r, NL_VECTOR *D, NL_REAL *wd, NL_INDEX *I, NL_INDEX s, NL_INDEX n, NL_DEGREE p, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvWeightedLstSq");

    NL_FLAG der = NL_NO, error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, rj, sj, ej, lk, hk, ru, su, rc, sc, mu, mc, mu2, mc2, jd;

    NL_REAL ** ND, ** N, ** M = NULL, ** WN, ** NTWN, *W;

    NL_PARAMETER *u;

    NL_POINT *S, *T = NULL, *NTWS, *INTWS, *V, *A = NULL, B;

    NL_CPOINT *Pw;

    NL_RMATRIX m, mt, mit, mi, imit, ntwn, intwn;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize some parameters */

    rj = p;
    sj = p;
    ej = -1;
    ru = -1;
    su = -1;
    rc = -1;
    sc = -1;
    mu2 = 0;
    mc2 = 0;
    jd = 0;

    /* Get constrained and unconstrained indexes */

    for ( i = 0; i <= r; i++ )
    {
        if( wp[i]GT 0.0 )
            ru++;
        else
            rc++;
    }

    for ( j = 0; j <= s; j++ )
    {
        if( wd[j]GT 0.0 )
            su++;
        else
            sc++;
    }

    mu = ru + su + 1;
    mc = rc + sc + 1;

    if( mc GE n OR( mc + n )GE( mu + 1 ) )
        NL_ERROR( NL_INP_ERR );

    if( n LT 1 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    /* Check for curve memory */

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &W );
    N_CrvGetKnotVector( cur, &knt );

    /* Allocate memory */

    index = N_AllocInt1dArray( mu, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( n, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( n, &SL );

    if( end EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( r, &SL );

    if( u EQ NULL )
        NL_QUIT;

    W = N_AllocReal1dArray( mu, &SL );

    if( W EQ NULL )
        NL_QUIT;

    ND = N_AllocReal2dArray( 1, p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    N = N_AllocReal2dArray( mu, p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    WN = N_AllocReal2dArray( mu, p, &SL );

    if( WN EQ NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( mu, &SL );

    if( S EQ NULL )
        NL_QUIT;

    NTWS = N_AllocPt1dArray( n, &SL );

    if( NTWS EQ NULL )
        NL_QUIT;

    INTWS = N_AllocPt1dArray( n, &SL );

    if( INTWS EQ NULL )
        NL_QUIT;

    V = N_AllocPt1dArray( NL_MAX( n, mc ), &SL );

    if( V EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &ntwn, n, n, NL_MT_FULL, n, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &ntwn, &NTWN );

    if( mc GE 0 )
    {
        A = N_AllocPt1dArray( mc, &SL );

        if( A EQ NULL )
            NL_QUIT;

        T = N_AllocPt1dArray( mc, &SL );

        if( T EQ NULL )
            NL_QUIT;

        error = N_SetRealMatrix( &m, mc, n, NL_MT_FULL, n, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &m, &M );
    }

    /* Initialize matrices */

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            N[i][j] = 0.0;
            WN[i][j] = 0.0;
        }
    }

    if( mc GE 0 )
    {
        for ( i = 0; i <= mc; i++ )
        {
            for ( j = 0; j <= n; j++ )
                M[i][j] = 0.0;
        }
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
            NTWN[i][j] = 0.0;
    }

    /* Compute parameters and knot vector */

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, r, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_FitCalcKnotVectorCrvApprox( u, r, n, p, knt );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute N, M, W, S and T */

    for ( i = 0; i <= n; i++ )
    {
        start[i] = 0;
        end[i] = mu;
    }

    for ( i = 0; i <= r; i++ )
    {
        error = N_BasisDerivs( knt, p, u[i], NL_LEFT, 1, ND, &j );

        if( error EQ NL_YES )
            NL_OUT;

        der = NL_NO;

        if( jd LE s AND i EQ I[jd] )
            der = NL_YES;

        if( wp[i]GT 0.0 ) /* Unconstrained point data */
        {
            W[mu2] = wp[i];
            N_ScalePt( W[mu2], P[i], &S[mu2] );

            for ( k = 0; k <= p; k++ )
                N[mu2][k] = ND[0][k];

            index[mu2] = j - p;

            if( j GT rj )
            {
                for ( k = 1; k <= j - rj; k++ )
                {
                    sj++;
                    ej++;
                    start[sj] = mu2;
                    end[ej] = mu2 - 1;
                }
                rj = j;
            }
            mu2++;
        }
        else /* Constrained point data */
        {
            N_CopyPt( P[i], &T[mc2] );

            for ( k = 0; k <= p; k++ )
                M[mc2][j - p + k] = ND[0][k];

            mc2++;
        }

        if( der EQ NL_YES )
        {
            if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
            {
                W[mu2] = wd[jd];
                N_ScalePt( W[mu2], D[jd], &S[mu2] );

                for ( k = 0; k <= p; k++ )
                    N[mu2][k] = ND[1][k];

                index[mu2] = j - p;

                if( j GT rj )
                {
                    for ( k = 1; k <= j - rj; k++ )
                    {
                        sj++;
                        ej++;
                        start[sj] = mu2;
                        end[ej] = mu2 - 1;
                    }
                    rj = j;
                }
                mu2++;
            }
            else /* Constrained derivative data */
            {
                N_CopyPt( D[jd], &T[mc2] );

                for ( k = 0; k <= p; k++ )
                    M[mc2][j - p + k] = ND[1][k];

                mc2++;
            }
            jd++;
        }
    }

    if( sj LT n OR end[0]EQ - 1 )
        NL_ERROR( NL_INP_ERR );

    /* Compute WN, NTWN and NTWS */

    for ( j = 0; j <= n; j++ )
    {
        lk = start[j];
        hk = end[j];

        for ( k = lk; k <= hk; k++ )
        {
            WN[k][j - index[k]] = W[k] * N[k][j - index[k]];
        }
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( k = lk; k <= hk; k++ )
            {
                NTWN[i][j] += N[k][i - index[k]] * WN[k][j - index[k]];
            }
        }
    }

    error = N_RealMatrixLuDecompose( &ntwn );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= n; i++ )
    {
        N_CopyPt( NL_ZERO, &NTWS[i] );

        lk = start[i];
        hk = end[i];

        for ( k = lk; k <= hk; k++ )
        {
            N_VectorBlendPt( N[k][i - index[k]], S[k], &NTWS[i] );
        }
    }

    /* In no constraints, do weighted approximation */

    if( mc LT 0 )
    {
        error = N_RealMatrixForBack( &ntwn, (NL_VOID *)NTWS, NL_EPOINT, Pw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Compute Lagrange multipliers */

    N_InitRealMatrix( &intwn );
    error = N_RealMatrixInverse( &ntwn, NL_YES, &intwn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &intwn, NTWS, INTWS );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &m, INTWS, V );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= mc; i++ )
        N_Diff2Pts( V[i], T[i], &V[i] );

    N_InitRealMatrix( &mt );
    error = N_RealMatrixTranspose( &m, &mt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mit );
    error = N_RealMatrixMultiply( &intwn, &mt, &mit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mi );
    error = N_RealMatrixMultiply( &m, &mit, &mi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &imit );
    error = N_RealMatrixInverse( &mi, NL_NO, &imit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &imit, V, A );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get control points */

    error = N_RealMatrixMultiplyPtArray( &mit, A, V );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= n; i++ )
    {
        N_Diff2Pts( INTWS[i], V[i], &B );
        N_PtToCPt( B, &Pw[i] );
    }

    if( wp[0]LT 0.0 )
        N_PtToCPt( P[0], &Pw[0] );

    if( wp[r]LT 0.0 )
        N_PtToCPt( P[r], &Pw[n] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVWEIGHTEDLSTSQPERIODIC: Weighted/constrained periodic least-squares curve fit    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes a weighted  and constrained least-
     squares NURBS curve approximation to a  given  set  of  points.  A
     periodic curve fit is done; i.e. the resulting curve  is  degree-1
     continuous at its start/end. This function should be  called  only
     for closed sets of points. This routine requires parameters and  a 
     knot vector, and  NL_POINT or NL_CPOINT  data. The knot vector must be a
     standard Nlib clamped knot vector  (e.g.  as created by N_FitCalcKnotVectorCrvApprox). 
     However,  the  fitted  curve  can be returned as either clamped or
     unclamped  (unclamped should be  used  only  for export,  as  Nlib 
     functions are not  designed  to  process  unclamped  curves).  The
     degree is arbitrary (> 1) and the number of control  points can be 
     chosen so as to satisfy the following conditions:
       (1) mc+p   < n,    where mc is the highest index  of constrained 
                          data,  and n is the highest index  of control 
                          points; and
       (2) mc+n+p < mu+1, where mu is the highest index of unconstrain-
                          ed data. 
     If  the output  curve is  initialized to the NULL curve, memory is 
     allocated  locally. Otherwise  it is checked  if enough  memory is 
     passed in. A typical calling example is:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_VECTOR      *D;
       NL_CVECTOR     *Dw;
       NL_REAL        *wp, *wd;
       NL_INDEX       *I, r, s, n;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_FLAG        clp;
       NL_STACKS      SG;
       ...
       (get P/Pw, D/Dw, wp, wd and I; choose p and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvWeightedLstSqPeriodic((NL_VOID *)P ,wp,r,(NL_VOID *)D ,wd,I,s,NL_EPOINT,u,&knt,n,p,
                NL_YES,&cur,&SG);
       N_FitCrvWeightedLstSqPeriodic((NL_VOID *)Pw,wp,r,(NL_VOID *)Dw,wd,I,s,NL_HPOINT,u,&knt,n,p,
                NL_NO,&cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT NL_VECTOR ARE COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     E   , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data.
                     Note: the equality E[0] = E[r] must hold
     wp  , input  ,  Array of point weights:
                       wp[i] > 0.0: E[i] is unconstrained
                       wp[i] < 0.0: E[i] is constrained
                     Note: if the curve is to be constrained to pass
                     through E[0] (=E[r]), set wp[0] = -1 and wp[r] = 1
     r   , input  ,  Highest index in E
     F   , input  ,  NL_VOID pointer representing  NL_VECTOR or NL_CVECTOR deri-
                     vative data
     wd  , input  ,  Array of derivative weights:
                       wd[j] > 0.0: F[j] is unconstrained
                       wd[j] < 0.0: F[j] is constrained
                     Note: if the first derivative is to be constrained
                     to a specific value at E[0] (=E[r]), then constrain
                     it at E[0], but not at E[r].
     I   , input  ,  Index array; the derivative at E[I[j]] is F[j]
     s   , input  ,  Highest index in F, wd and I
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     u   , input  ,  Parameters corresponding to data points
     knt , input  ,  Knot vector of approximating curve. Must be
                     clamped.
     n   , input  ,  Highest   index   of   control   point   array  of 
                     approximating curve 
     p   , input  ,  Degree of approximating curve ( p > 1 )
     clp , input  ,  Clamped flag:
                       = NL_YES : Output curve should be clamped
                       = NL_NO  : Output curve unclamped (for export only)
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvWeightedLstSqPeriodic( NL_VOID *E, NL_REAL *wp, NL_INDEX r, NL_VOID *F, NL_REAL *wd, NL_INDEX *I, NL_INDEX s, NL_FLAG ptp, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_INDEX n, NL_DEGREE p, NL_FLAG clp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvWeightedLstSqPeriodic");

    NL_FLAG der = NL_NO, error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, rj, sj, ej, lk, hk, ru, su, rc, sc, mu, mc, mu2, mc2, jd, mknt, mcur;

    NL_REAL ** ND, ** N, ** M, ** WN, ** NTWN, *W, *U, *UC;

    NL_POINT *S, *T = NULL, *NTWS = NULL, *INTWS = NULL, *V = NULL, *A = NULL, *R = NULL, B;

    NL_CPOINT *Pw = NULL, *Sw = NULL, *Tw = NULL, *NTWSw = NULL, *INTWSw = NULL, *Vw = NULL, *Aw = NULL, *Rw = NULL;

    NL_VECTOR *D;

    NL_CVECTOR *Dw;

    NL_RMATRIX m, mt, mit, mi, imit, ntwn, intwn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize some parameters */

    rj = p;
    sj = p;
    ej = -1;
    ru = -1;
    su = -1;
    rc = -1;
    sc = -1;

    /* Get constrained and unconstrained indexes and check error */

    for ( i = 0; i <= r; i++ )
    {
        if( wp[i]GT 0.0 )
            ru++;
        else
            rc++;
    }

    for ( j = 0; j <= s; j++ )
    {
        if( wd[j]GT 0.0 )
            su++;
        else
            sc++;
    }

    N_KnotVectorGetKnots( knt, &mknt, &U );

    mu = ru + su + 1;
    mc = rc + sc + 1;
    mcur = n + p + 1;

    if( mc + p GE n OR( mc + n + p )GE( mu + 1 ) )
        NL_ERROR( NL_INP_ERR );

    if( n LT 1 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    if( mknt NEQ mcur )
        NL_ERROR( NL_INP_ERR );

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Check for curve memory */

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= mknt; i++ )
        UC[i] = U[i];

    /* Allocate memory */

    index = N_AllocInt1dArray( mu, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( n, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( n, &SL );

    if( end EQ NULL )
        NL_QUIT;

    W = N_AllocReal1dArray( mu, &SL );

    if( W EQ NULL )
        NL_QUIT;

    ND = N_AllocReal2dArray( p - 1, p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    N = N_AllocReal2dArray( mu, p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    WN = N_AllocReal2dArray( mu, p, &SL );

    if( WN EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &ntwn, n, n, NL_MT_FULL, n, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &ntwn, &NTWN );

    error = N_SetRealMatrix( &m, mc + p, n, NL_MT_FULL, n, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &m, &M );

    /* Initialize matrices */

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            N[i][j] = 0.0;
            WN[i][j] = 0.0;
        }
    }

    for ( i = 0; i <= mc + p; i++ )
    {
        for ( j = 0; j <= n; j++ )
            M[i][j] = 0.0;
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
            NTWN[i][j] = 0.0;
    }

    /* Compute N, M and W */

    for ( i = 0; i <= n; i++ )
    {
        start[i] = 0;
        end[i] = mu;
    }

    mu2 = 0;
    mc2 = 0;
    jd = 0;

    for ( i = 0; i <= r; i++ )
    {
        error = N_BasisDerivs( knt, p, u[i], NL_LEFT, 1, ND, &j );

        if( error EQ NL_YES )
            NL_OUT;

        der = NL_NO;

        if( jd LE s AND i EQ I[jd] )
            der = NL_YES;

        if( wp[i]GT 0.0 ) /* Unconstrained point data */
        {
            W[mu2] = wp[i];

            for ( k = 0; k <= p; k++ )
                N[mu2][k] = ND[0][k];

            index[mu2] = j - p;

            if( j GT rj )
            {
                for ( k = 1; k <= j - rj; k++ )
                {
                    sj++;
                    ej++;
                    start[sj] = mu2;
                    end[ej] = mu2 - 1;
                }
                rj = j;
            }
            mu2++;
        }
        else /* Constrained point data */
        {
            for ( k = 0; k <= p; k++ )
                M[mc2][j - p + k] = ND[0][k];
            mc2++;
        }

        if( der EQ NL_YES )
        {
            if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
            {
                W[mu2] = wd[jd];

                for ( k = 0; k <= p; k++ )
                    N[mu2][k] = ND[1][k];

                index[mu2] = j - p;

                if( j GT rj )
                {
                    for ( k = 1; k <= j - rj; k++ )
                    {
                        sj++;
                        ej++;
                        start[sj] = mu2;
                        end[ej] = mu2 - 1;
                    }
                    rj = j;
                }
                mu2++;
            }
            else /* Constrained derivative data */
            {
                for ( k = 0; k <= p; k++ )
                    M[mc2][j - p + k] = ND[1][k];
                mc2++;
            }
            jd++;
        }
    }

    if( sj LT n OR end[0]EQ - 1 )
        NL_ERROR( NL_INP_ERR );

    /* Add in the periodic constraints */

    error = N_BasisDerivs( knt, p, u[0], NL_LEFT, p - 1, ND, &j );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i < p; i++ )
    {
        for ( k = 0; k <= p; k++ )
            M[mc2][j - p + k] = ND[i][k];
        mc2++;
    }

    error = N_BasisDerivs( knt, p, u[r], NL_RIGHT, p - 1, ND, &j );

    if( error EQ NL_YES )
        NL_OUT;

    mc2 -= p;

    for ( i = 0; i < p; i++ )
    {
        for ( k = 0; k <= p; k++ )
            M[mc2][j - p + k] = -ND[i][k];
        mc2++;
    }

    /* Compute WN, NTWN and LU-decompose NTWN */

    for ( j = 0; j <= n; j++ )
    {
        lk = start[j];
        hk = end[j];

        for ( k = lk; k <= hk; k++ )
        {
            WN[k][j - index[k]] = W[k] * N[k][j - index[k]];
        }
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( k = lk; k <= hk; k++ )
            {
                NTWN[i][j] += N[k][i - index[k]] * WN[k][j - index[k]];
            }
        }
    }

    error = N_RealMatrixLuDecompose( &ntwn );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute input dependent data */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)E;
            D = (NL_VECTOR *)F;

            S = N_AllocPt1dArray( mu, &SL );

            if( S EQ NULL )
                NL_QUIT;

            NTWS = N_AllocPt1dArray( n, &SL );

            if( NTWS EQ NULL )
                NL_QUIT;

            INTWS = N_AllocPt1dArray( n, &SL );

            if( INTWS EQ NULL )
                NL_QUIT;

            V = N_AllocPt1dArray( NL_MAX( n, mc + p ), &SL );

            if( V EQ NULL )
                NL_QUIT;

            A = N_AllocPt1dArray( mc + p, &SL );

            if( A EQ NULL )
                NL_QUIT;

            T = N_AllocPt1dArray( mc + p, &SL );

            if( T EQ NULL )
                NL_QUIT;

            mu2 = 0;
            mc2 = 0;
            jd = 0;

            for ( i = 0; i <= r; i++ )
            {
                der = NL_NO;

                if( jd LE s AND i EQ I[jd] )
                    der = NL_YES;

                if( wp[i]GT 0.0 ) /* Unconstrained point data */
                {
                    N_ScalePt( W[mu2], R[i], &S[mu2] );
                    mu2++;
                }
                else /* Constrained point data */
                {
                    N_CopyPt( R[i], &T[mc2] );
                    mc2++;
                }

                if( der EQ NL_YES )
                {
                    if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
                    {
                        N_ScalePt( W[mu2], D[jd], &S[mu2] );
                        mu2++;
                    }
                    else /* Constrained derivative data */
                    {
                        N_CopyPt( D[jd], &T[mc2] );
                        mc2++;
                    }
                    jd++;
                }
            }

            /* Add in the periodic constraints to T */

            for ( i = 0; i < p; i++ )
            {
                N_CopyPt( NL_ZERO, &T[mc2] );
                mc2++;
            }

            for ( i = 0; i <= n; i++ )
            {
                N_CopyPt( NL_ZERO, &NTWS[i] );

                lk = start[i];
                hk = end[i];

                for ( k = lk; k <= hk; k++ )
                {
                    N_VectorBlendPt( N[k][i - index[k]], S[k], &NTWS[i] );
                }
            }

            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)E;
            Dw = (NL_CVECTOR *)F;

            Sw = N_AllocCPt1dArray( mu, &SL );

            if( Sw EQ NULL )
                NL_QUIT;

            NTWSw = N_AllocCPt1dArray( n, &SL );

            if( NTWSw EQ NULL )
                NL_QUIT;

            INTWSw = N_AllocCPt1dArray( n, &SL );

            if( INTWSw EQ NULL )
                NL_QUIT;

            Vw = N_AllocCPt1dArray( NL_MAX( n, mc + p ), &SL );

            if( Vw EQ NULL )
                NL_QUIT;

            Aw = N_AllocCPt1dArray( mc + p, &SL );

            if( Aw EQ NULL )
                NL_QUIT;

            Tw = N_AllocCPt1dArray( mc + p, &SL );

            if( Tw EQ NULL )
                NL_QUIT;

            mu2 = 0;
            mc2 = 0;
            jd = 0;

            for ( i = 0; i <= r; i++ )
            {
                der = NL_NO;

                if( jd LE s AND i EQ I[jd] )
                    der = NL_YES;

                if( wp[i]GT 0.0 ) /* Unconstrained point data */
                {
                    N_ScaleCPt( W[mu2], Rw[i], &Sw[mu2] );
                    mu2++;
                }
                else /* Constrained point data */
                {
                    N_CopyCPt( Rw[i], &Tw[mc2] );
                    mc2++;
                }

                if( der EQ NL_YES )
                {
                    if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
                    {
                        N_ScaleCPt( W[mu2], Dw[jd], &Sw[mu2] );
                        mu2++;
                    }
                    else /* Constrained derivative data */
                    {
                        N_CopyCPt( Dw[jd], &Tw[mc2] );
                        mc2++;
                    }
                    jd++;
                }
            }

            /* Add in the periodic constraints to Tw */

            for ( i = 0; i < p; i++ )
            {
                N_CopyCPt( NL_CZERO, &Tw[mc2] );
                mc2++;
            }

            for ( i = 0; i <= n; i++ )
            {
                N_CopyCPt( NL_CZERO, &NTWSw[i] );

                lk = start[i];
                hk = end[i];

                for ( k = lk; k <= hk; k++ )
                {
                    N_VectorBlendCPt( N[k][i - index[k]], Sw[k], &NTWSw[i] );
                }
            }

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Get matrices to compute Lagrange multipliers */

    N_InitRealMatrix( &intwn );
    error = N_RealMatrixInverse( &ntwn, NL_YES, &intwn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mt );
    error = N_RealMatrixTranspose( &m, &mt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mit );
    error = N_RealMatrixMultiply( &intwn, &mt, &mit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mi );
    error = N_RealMatrixMultiply( &m, &mit, &mi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &imit );
    error = N_RealMatrixInversePivot( &mi, &imit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute control points */

    switch( ptp )
    {
        case NL_EPOINT:

            error = N_RealMatrixMultiplyPtArray( &intwn, NTWS, INTWS );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyPtArray( &m, INTWS, V );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mc + p; i++ )
                N_Diff2Pts( V[i], T[i], &V[i] );

            error = N_RealMatrixMultiplyPtArray( &imit, V, A );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyPtArray( &mit, A, V );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
            {
                N_Diff2Pts( INTWS[i], V[i], &B );
                N_PtToCPt( B, &Pw[i] );
            }

            if( wp[0]LT 0.0 )
                N_PtToCPt( R[0], &Pw[0] );

            if( wp[r]LT 0.0 )
                N_PtToCPt( R[r], &Pw[n] );
            break;

        case NL_HPOINT:

            error = N_RealMatrixMultiplyCPtArray( &intwn, NTWSw, INTWSw );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyCPtArray( &m, INTWSw, Vw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mc + p; i++ )
                N_Diff2CPts( Vw[i], Tw[i], &Vw[i] );

            error = N_RealMatrixMultiplyCPtArray( &imit, Vw, Aw );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyCPtArray( &mit, Aw, Vw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
            {
                N_Diff2CPts( INTWSw[i], Vw[i], &Pw[i] );
            }

            if( wp[0]LT 0.0 )
                N_CopyCPt( Rw[0], &Pw[0] );

            if( wp[r]LT 0.0 )
                N_CopyCPt( Rw[r], &Pw[n] );
            break;
    }

    /* Unclamp if necessary */

    if( clp EQ NL_NO )
        N_CrvUnclamp( cur );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVWEIGHTEDLSTSQKNOTS: Weighted/constrained least-squares with knot vector      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes a weighted  and constrained least-
     squares NURBS curve approximation to a  given set of  points. This
     is a  general routine  requiring  parameters where data points are
     assumed, a knot vector, and  NL_POINT or  NL_CPOINT  data. The degree is 
     arbitrary and the number of control  points can be chosen so as to  
     satisfy the following conditions:
       (1) mc   < n,    where mc  is the  highest index  of constrained 
                        data,  and n  is the  highest index  of control 
                        points; and
       (2) mc+n < mu+1, where  mu is the highest index of unconstrained
                        data. 
     If  the output  curve is  initialized to the NULL curve, memory is 
     allocated  locally. Otherwise  it is checked  if enough  memory is 
     passed in. A typical calling example is:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_VECTOR      *D;
       NL_CVECTOR     *Dw;
       NL_REAL        *wp, *wd;
       NL_INDEX       *I, r, s, n;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get P/Pw, D/Dw, wp, wd and I; choose p and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvWeightedLstSqKnots((NL_VOID *)P ,wp,r,(NL_VOID *)D ,wd,I,s,NL_EPOINT,u,&knt,n,p,
                &cur,&SG);
       N_FitCrvWeightedLstSqKnots((NL_VOID *)Pw,wp,r,(NL_VOID *)Dw,wd,I,s,NL_HPOINT,u,&knt,n,p,
                &cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT VECTOR ARE COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     E   , input  ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     wp  , input  ,  Array of point weights:
                       wp[i] > 0.0: E[i] is unconstrained
                       wp[i] < 0.0: E[i] is constrained
     r   , input  ,  Highest index in E
     F   , input  ,  NL_VOID pointer representing  NL_VECTOR or NL_CVECTOR deri-
                     vative data
     wd  , input  ,  Array of derivative weights:
                       wd[j] > 0.0: F[j] is unconstrained
                       wd[j] < 0.0: F[j] is constrained
     I   , input  ,  Index array; the derivative at E[I[j]] is F[j]
     s   , input  ,  Highest index in F, wd and I
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     u   , input  ,  Parameters corresponding to data points
     knt , input  ,  Knot vector of approximating curve
     n   , input  ,  Highest   index   of   control   point   array  of 
                     approximating curve 
     p   , input  ,  Degree of approximating curve 
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_FitCrvWeightedLstSqKnots
 (NL_VOID       * E,        // in : TgtPoint position constraint values to be approximated by points on the ApproxCrv
                            //      when ptp == NL_EPOINT, ptr to NL_POINT array
                            //      when ptp == NL_HPOINT, ptr to NL_CPOINT array
  NL_REAL       * wp,       // in : Array of TgtPt weights for TgtPt-to-PtOnCrv dist penalty method 
                            //      (weight is stiffness of a spring between TgtPt and PtOnCrv - bigger weights tighter fits and less smoothness)
                            //      when wp[i] > 0.0: E[i] is unconstrained
                            //      when wp[i] < 0.0: E[i] is constrained
  NL_INDEX        r,        // in : highest index in TgtPoint array, (TgtPtArray Size = Highest Index + 1)
  NL_VOID       * F,        // in : Tgt 1st derivative constraint values to be approximated by points on the ApproxCrv
                            //      when ptp == NL_EPOINT, ptr to NL_VECTOR array
                            //      when ptp == NL_HPOINT, ptr to NL_CVECTOR array
  NL_REAL       * wd,       // in : Array of Tgt1stDeriv weights for TgtPt1stDeriv-to-PtOnCrv1stDeriv penalty method 
                            //       (weight of stiffness of a rotational spring between Tgt1stDreriv and 1stDeriv val at PtOnCrv.
                            //        bigger weights force the direction of the ApproxCrv closer to the Tgt1stDeriv value.
                            //        can be used to add a 'damping' effect to prevent the polynomial ApproxCurve from deviating between TgtPts when
                            //        constraint 1st Deriv values point from one TgtPoint position to the next.)
                            //      when wd[i] > 0.0: F[i] is unconstrained
                            //      when wd[i] < 0.0: F[i] is constrained
  NL_INDEX      * I,        // in : Index array, the Tgt 1st derivative value at E[I[j]] is F[j]
  NL_INDEX        s,        // in : Highest index in arrays F, wd, and I, (Array Size = HIghest Index + 1)
  NL_FLAG         ptp,      // in : NL_EPOINT: TgtPts and Tgt1stDerivs are Euclidean pts and vectors.
                            //      NL_HPOINT: TgtPts and Tgt1stDerivs are Homogeneous pts and vectors.
  NL_PARAMETER  * u,        // in : Associated array of TgtCrv param values for each TgtPt value 
                            //      (the point on the curve that is made to approximate the associated TgtPt pos value) 
  NL_KNOTVECTOR * knt,      // in : Knot vector of approximating curve 
  NL_INDEX        n,        // in : Highest index of approximating curve CPts, (ApproxCrv CPT count = Highest Index + 1)
  NL_DEGREE       p,        // in : approximating curve degree (between 3 and 5 works well for pt sets representing sampled curve sequences)
  NL_CURVE      * cur,      // out: Approximating Curve
  NL_STACKS     * SG )      // i/o: approximating curve's memory stack
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvWeightedLstSqKnots");

    NL_FLAG der = NL_NO, error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, rj, sj, ej, lk, hk, ru, su, rc, sc, mu, mc, mu2, mc2, jd, mknt, mcur;

    NL_REAL ** ND, ** N, ** M = NULL, ** WN, ** NTWN, *W, *U, *UC;

    NL_POINT *S, *T = NULL, *NTWS = NULL, *INTWS = NULL, *V = NULL, *A = NULL, *R = NULL, B;

    NL_CPOINT *Pw, *Sw, *Tw = NULL, *NTWSw = NULL, *INTWSw = NULL, *Vw = NULL, *Aw = NULL, *Rw = NULL;

    NL_VECTOR *D;

    NL_CVECTOR *Dw;

    NL_RMATRIX m, mt, mit, mi, imit, ntwn, intwn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize some parameters */

    rj = p;
    sj = p;
    ej = -1;
    ru = -1;
    su = -1;
    rc = -1;
    sc = -1;

    /* Get constrained and unconstrained indexes and check error */

    for ( i = 0; i <= r; i++ )
    {
        if( wp[i]GT 0.0 )
            ru++;
        else
            rc++;
    }

    for ( j = 0; j <= s; j++ )
    {
        if( wd[j]GT 0.0 )
            su++;
        else
            sc++;
    }

    N_KnotVectorGetKnots( knt, &mknt, &U );

    mu = ru + su + 1;
    mc = rc + sc + 1;
    mcur = n + p + 1;

    if( mc GE n OR( mc + n )GE( mu + 1 ) )
        NL_ERROR( NL_INP_ERR );

    if( n LT 1 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    if( mknt NEQ mcur )
        NL_ERROR( NL_INP_ERR );

    /* Check for curve memory */

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= mknt; i++ )
        UC[i] = U[i];

    /* Allocate memory */

    index = N_AllocInt1dArray( mu, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( n, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( n, &SL );

    if( end EQ NULL )
        NL_QUIT;

    W = N_AllocReal1dArray( mu, &SL );

    if( W EQ NULL )
        NL_QUIT;

    ND = N_AllocReal2dArray( 1, p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    N = N_AllocReal2dArray( mu, p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    WN = N_AllocReal2dArray( mu, p, &SL );

    if( WN EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &ntwn, n, n, NL_MT_FULL, n, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &ntwn, &NTWN );

    if( mc GE 0 )
    {
        error = N_SetRealMatrix( &m, mc, n, NL_MT_FULL, n, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &m, &M );
    }

    /* Initialize matrices */

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            N[i][j] = 0.0;
            WN[i][j] = 0.0;
        }
    }

    if( mc GE 0 )
    {
        for ( i = 0; i <= mc; i++ )
        {
            for ( j = 0; j <= n; j++ )
                M[i][j] = 0.0;
        }
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
            NTWN[i][j] = 0.0;
    }

    /* Compute N, M and W */

    for ( i = 0; i <= n; i++ )
    {
        start[i] = 0;
        end[i] = mu;
    }

    mu2 = 0;
    mc2 = 0;
    jd = 0;

    for ( i = 0; i <= r; i++ )
    {
        error = N_BasisDerivs( knt, p, u[i], NL_LEFT, 1, ND, &j );

        if( error EQ NL_YES )
            NL_OUT;

        der = NL_NO;

        if( jd LE s AND i EQ I[jd] )
            der = NL_YES;

        if( wp[i]GT 0.0 ) /* Unconstrained point data */
        {
            W[mu2] = wp[i];

            for ( k = 0; k <= p; k++ )
                N[mu2][k] = ND[0][k];

            index[mu2] = j - p;

            if( j GT rj )
            {
                for ( k = 1; k <= j - rj; k++ )
                {
                    sj++;
                    ej++;
                    start[sj] = mu2;
                    end[ej] = mu2 - 1;
                }
                rj = j;
            }
            mu2++;
        }
        else /* Constrained point data */
        {
            for ( k = 0; k <= p; k++ )
                M[mc2][j - p + k] = ND[0][k];
            mc2++;
        }

        if( der EQ NL_YES )
        {
            if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
            {
                W[mu2] = wd[jd];

                for ( k = 0; k <= p; k++ )
                    N[mu2][k] = ND[1][k];

                index[mu2] = j - p;

                if( j GT rj )
                {
                    for ( k = 1; k <= j - rj; k++ )
                    {
                        sj++;
                        ej++;
                        start[sj] = mu2;
                        end[ej] = mu2 - 1;
                    }
                    rj = j;
                }
                mu2++;
            }
            else /* Constrained derivative data */
            {
                for ( k = 0; k <= p; k++ )
                    M[mc2][j - p + k] = ND[1][k];
                mc2++;
            }
            jd++;
        }
    }

    if( sj LT n OR end[0]EQ - 1 )
        NL_ERROR( NL_INP_ERR );

    /* Compute WN, NTWN and LU-decompose NTWN */

    for ( j = 0; j <= n; j++ )
    {
        lk = start[j];
        hk = end[j];

        for ( k = lk; k <= hk; k++ )
        {
            WN[k][j - index[k]] = W[k] * N[k][j - index[k]];
        }
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( k = lk; k <= hk; k++ )
            {
                NTWN[i][j] += N[k][i - index[k]] * WN[k][j - index[k]];
            }
        }
    }

    error = N_RealMatrixLuDecompose( &ntwn );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute input dependent data */

    switch( ptp )
    {
        case NL_EPOINT:

            R = (NL_POINT *)E;
            D = (NL_VECTOR *)F;

            S = N_AllocPt1dArray( mu, &SL );

            if( S EQ NULL )
                NL_QUIT;

            NTWS = N_AllocPt1dArray( n, &SL );

            if( NTWS EQ NULL )
                NL_QUIT;

            INTWS = N_AllocPt1dArray( n, &SL );

            if( INTWS EQ NULL )
                NL_QUIT;

            V = N_AllocPt1dArray( NL_MAX( n, mc ), &SL );

            if( V EQ NULL )
                NL_QUIT;

            if( mc GE 0 )
            {
                A = N_AllocPt1dArray( mc, &SL );

                if( A EQ NULL )
                    NL_QUIT;

                T = N_AllocPt1dArray( mc, &SL );

                if( T EQ NULL )
                    NL_QUIT;
            }

            mu2 = 0;
            mc2 = 0;
            jd = 0;

            for ( i = 0; i <= r; i++ )
            {
                der = NL_NO;

                if( jd LE s AND i EQ I[jd] )
                    der = NL_YES;

                if( wp[i]GT 0.0 ) /* Unconstrained point data */
                {
                    N_ScalePt( W[mu2], R[i], &S[mu2] );
                    mu2++;
                }
                else /* Constrained point data */
                {
                    N_CopyPt( R[i], &T[mc2] );
                    mc2++;
                }

                if( der EQ NL_YES )
                {
                    if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
                    {
                        N_ScalePt( W[mu2], D[jd], &S[mu2] );
                        mu2++;
                    }
                    else /* Constrained derivative data */
                    {
                        N_CopyPt( D[jd], &T[mc2] );
                        mc2++;
                    }
                    jd++;
                }
            }

            for ( i = 0; i <= n; i++ )
            {
                N_CopyPt( NL_ZERO, &NTWS[i] );

                lk = start[i];
                hk = end[i];

                for ( k = lk; k <= hk; k++ )
                {
                    N_VectorBlendPt( N[k][i - index[k]], S[k], &NTWS[i] );
                }
            }

            /* In no constraints, do weighted approximation */

            if( mc LT 0 )
            {
                error = N_RealMatrixForBack( &ntwn, (NL_VOID *)NTWS, NL_EPOINT, Pw, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                NL_OUT;
            }
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)E;
            Dw = (NL_CVECTOR *)F;

            Sw = N_AllocCPt1dArray( mu, &SL );

            if( Sw EQ NULL )
                NL_QUIT;

            NTWSw = N_AllocCPt1dArray( n, &SL );

            if( NTWSw EQ NULL )
                NL_QUIT;

            INTWSw = N_AllocCPt1dArray( n, &SL );

            if( INTWSw EQ NULL )
                NL_QUIT;

            Vw = N_AllocCPt1dArray( NL_MAX( n, mc ), &SL );

            if( Vw EQ NULL )
                NL_QUIT;

            if( mc GE 0 )
            {
                Aw = N_AllocCPt1dArray( mc, &SL );

                if( Aw EQ NULL )
                    NL_QUIT;

                Tw = N_AllocCPt1dArray( mc, &SL );

                if( Tw EQ NULL )
                    NL_QUIT;
            }

            mu2 = 0;
            mc2 = 0;
            jd = 0;

            for ( i = 0; i <= r; i++ )
            {
                der = NL_NO;

                if( jd LE s AND i EQ I[jd] )
                    der = NL_YES;

                if( wp[i]GT 0.0 ) /* Unconstrained point data */
                {
                    N_ScaleCPt( W[mu2], Rw[i], &Sw[mu2] );
                    mu2++;
                }
                else /* Constrained point data */
                {
                    N_CopyCPt( Rw[i], &Tw[mc2] );
                    mc2++;
                }

                if( der EQ NL_YES )
                {
                    if( wd[jd]GT 0.0 ) /* Unconstrained derivative data */
                    {
                        N_ScaleCPt( W[mu2], Dw[jd], &Sw[mu2] );
                        mu2++;
                    }
                    else /* Constrained derivative data */
                    {
                        N_CopyCPt( Dw[jd], &Tw[mc2] );
                        mc2++;
                    }
                    jd++;
                }
            }

            for ( i = 0; i <= n; i++ )
            {
                N_CopyCPt( NL_CZERO, &NTWSw[i] );

                lk = start[i];
                hk = end[i];

                for ( k = lk; k <= hk; k++ )
                {
                    N_VectorBlendCPt( N[k][i - index[k]], Sw[k], &NTWSw[i] );
                }
            }

            /* In no constraints, do weighted approximation */

            if( mc LT 0 )
            {
                error = N_RealMatrixForBack( &ntwn, (NL_VOID *)NTWSw, NL_HPOINT, Pw, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                NL_OUT;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Get matrices to compute Lagrange multipliers */

    N_InitRealMatrix( &intwn );
    error = N_RealMatrixInverse( &ntwn, NL_YES, &intwn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mt );
    error = N_RealMatrixTranspose( &m, &mt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mit );
    error = N_RealMatrixMultiply( &intwn, &mt, &mit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mi );
    error = N_RealMatrixMultiply( &m, &mit, &mi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &imit );
    error = N_RealMatrixInverse( &mi, NL_NO, &imit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute control points */

    switch( ptp )
    {
        case NL_EPOINT:

            error = N_RealMatrixMultiplyPtArray( &intwn, NTWS, INTWS );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyPtArray( &m, INTWS, V );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mc; i++ )
                N_Diff2Pts( V[i], T[i], &V[i] );

            error = N_RealMatrixMultiplyPtArray( &imit, V, A );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyPtArray( &mit, A, V );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
            {
                N_Diff2Pts( INTWS[i], V[i], &B );
                N_PtToCPt( B, &Pw[i] );
            }

            if( wp[0]LT 0.0 )
                N_PtToCPt( R[0], &Pw[0] );

            if( wp[r]LT 0.0 )
                N_PtToCPt( R[r], &Pw[n] );
            break;

        case NL_HPOINT:

            error = N_RealMatrixMultiplyCPtArray( &intwn, NTWSw, INTWSw );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyCPtArray( &m, INTWSw, Vw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mc; i++ )
                N_Diff2CPts( Vw[i], Tw[i], &Vw[i] );

            error = N_RealMatrixMultiplyCPtArray( &imit, Vw, Aw );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_RealMatrixMultiplyCPtArray( &mit, Aw, Vw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
            {
                N_Diff2CPts( INTWSw[i], Vw[i], &Pw[i] );
            }

            if( wp[0]LT 0.0 )
                N_CopyCPt( Rw[0], &Pw[0] );

            if( wp[r]LT 0.0 )
                N_CopyCPt( Rw[r], &Pw[n] );
            break;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROXCLOSED: Curve approximation to closed data with end conditions   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve approximating a closed
     point set.  The degree can be as  high as the number of conditions 
     required, and  the type  of  parametrization can be chosen. If the 
     output curve is initialized to the NULL curve, memory is allocated  
     locally.  Otherwise  it is  checked if enough memory is passed in. 
     A typical calling example:

       NL_POINT   *P;
       NL_INDEX   m, n, k;
       NL_DEGREE  p;
       NL_VECTOR  *D;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P, and D, and select n and p);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxClosed(P,m,D,k,n,p,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization  is  NL_CHORDLENGTH. If 
     no derivative is wanted at the ends, set k=0. THE BEST USE OF THIS
     ROUTINE IS TO COMPUTE A C^p-1 CONTINUOUS CLOSED NL_CURVE APPROXIMANT.


   ACCESS:
   
     P     , input  ,  Points to be interpolated, P[0]=P[n]
     m     , input  ,  Highest index in P
     D     , input  ,  Start and end derivatives:
                          NULL: no derivatives passed in. Compute  them internally
                         !NULL: derivatives passed in
     k     , input  ,  Highest index in D;  D[1],...,D[k] are  given at
                       each  end, i.e., the  curve is  closed with  C^k 
                       continuity. k  MUST BE  SPECIFIED EVEN IF NL_NO DE-
                       RIVATIVES ARE PASSED IN! k < p!
     n     , input  ,  Highest control point index of approximating curve: n >= p.
     p     , input  ,  Degree of approximating curve
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization
                         NL_CENTRIPETAL: Centripetal parametrization
     cur   , output ,  Approximating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvApproxClosed( NL_POINT *P, NL_INDEX m, NL_VECTOR *D, NL_INDEX k, NL_INDEX n, NL_DEGREE p, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxClosed");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, is, ie, id = 0, nl, ml, nm;

    NL_REAL *u, *v, fr;

    NL_POINT *Q;

    NL_VECTOR *Ds, *De, *Dl;

    NL_KNOTVECTOR *knt;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    if ( n < p )
      { NL_ERROR( NL_INP_ERR ); }

    /* Allocate memory for end derivatives */

    Ds = N_AllocPt1dArray( k, &SL );

    if( Ds EQ NULL )
        NL_QUIT;

    De = N_AllocPt1dArray( k, &SL );

    if( De EQ NULL )
        NL_QUIT;

    /* Derivatives passed in */

    if( D NEQ NULL )
    {
        for ( i = 1; i <= k; i++ )
        {
            N_VectorCopy( D[i], &Ds[i] );
            N_VectorCopy( D[i], &De[i] );
        }
    }
    else
    {

        /* No derivatives are passed in; compute them internally */

        nm = m / 2;
        fr = (NL_REAL)((m + 1.0) / (n + 1.0));
        ml = (NL_INDEX)((p + 1.0) * fr + 0.5);
        is = NL_MAX( nm, m - ml );
        ie = NL_MIN( nm, ml );
        ml = m - is + ie;
        fr = (NL_REAL)( ml ) / (NL_REAL)( m );
        nl = (NL_INDEX)(fr * n + 0.5);

        Q = N_AllocPt1dArray( ml, &SL );

        if( Q EQ NULL )
            NL_QUIT;

        Dl = N_AllocPt1dArray( k, &SL );

        if( Dl EQ NULL )
            NL_QUIT;

        u = N_AllocReal1dArray( ml, &SL );

        if( u EQ NULL )
            NL_QUIT;

        v = N_AllocReal1dArray( m, &SL );

        if( v EQ NULL )
            NL_QUIT;

        knt = N_AllocKnotVectorAndArray( nl + p + 1, &SL );

        if( knt EQ NULL )
            NL_QUIT;

        /* Get data points for local fitting */

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, par, v );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( P[is], &Q[0] );
        u[0] = 0.0;

        i = is + 1;
        j = 1;

        while( j LE ml )
        {
            N_VectorCopy( P[i], &Q[j] );
            u[j] = u[j - 1] + (v[i] - v[i - 1]);

            if( i EQ m )
            {
                i = 0;
                id = j;
            }

            i++;
            j++;
        }

        /* Approximate local data */

        N_FitCalcKnotVectorCrvApprox( u, ml, nl, p, knt );

        N_CrvInitArrays( &curA );
        error = N_FitCrvApproxKnots( (NL_VOID *)Q, ml, NL_EPOINT, u, knt, nl, p, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get derivatives off the local interpolant */

        error = N_CrvDerivs( &curA, u[id], NL_LEFT, k, Dl );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 1; i <= k; i++ )
        {
            N_VectorCopy( Dl[i], &Ds[i] );
            N_VectorCopy( Dl[i], &De[i] );
        }
    }

    /* Approximate data set */

    error = N_FitCrvApproxDerivs( P, m, Ds, k, De, k, n, p, par, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROXDERIVS: Global curve approximation with end derivatives          */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine computes a least-squares NURBS curve approxi-
     mation to a  given set of  points  with end derivative constraints. 
     The  degree is  arbitrary and the type of  parametrization  and the 
     number of  control  points  can be chosen. If the  output curve  is  
     initialized  to  the  NULL  curve,  memory  is  allocated  locally. 
     Otherwise it is  checked if  enough memory is  passed in. A typical 
     calling example is:
 
       NL_POINT   *P;
       NL_VECTOR  *Ds, *De;
       NL_INDEX   m, n, k, l;
       NL_DEGREE  p;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define array P, get Ds and De, and choose p and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxDerivs(P,m,Ds,k,De,l,n,p,NL_CHORDLENGTH,&cur,&SG);
 
     A recommended default for the parametrization is NL_CHORDLENGTH.
 
 
   ACCESS:
   
     P   , input  ,  Points to be approximated
     m   , input  ,  Highest index in P
     Ds  , input  ,  Start derivatives
     k   , input  ,  Highest start derivative, i.e., at the start Ds[1],
                     ...,Ds[k] are specified. If k=0, no  derivative is 
                     needed. Must satisfy k < p!
     De  , input  ,  End derivatives
     l   , input  ,  Highest end derivative, i.e., at the end De[1],...,
                     De[l] are  specified. l=0  means no  derivative  is 
                     needed. Must satisfy l < p!
     n   , input  ,  Highest index of control points. The following must
                     hold: n <= m  and n > k+l!
     p   , input  ,  Degree of approximating curve (p <= n!)
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCrvApproxDerivs( NL_POINT *P, NL_INDEX m, NL_VECTOR *Ds, NL_INDEX k, NL_VECTOR *De, NL_INDEX l, NL_INDEX n, NL_DEGREE p, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, kk, ll, bw, sbw, rj, sj, ej, lk, hk, lj, hj, nd;

    NL_REAL ** A, ** NTN, ** ND, *N, *u, fac, nk, nl;

    NL_POINT *Rk, *R, Sk, Sl, D;

    NL_CPOINT *Pw, Sw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error and curve memory */

    if( p GT n )
        NL_ERROR( NL_INP_ERR );

    if( k GE p OR l GE p )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR l LT 0 )
        NL_ERROR( NL_INP_ERR );

    if( m LT n OR n LE k + l )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &N );
    N_CrvGetKnotVector( cur, &knt );

    /* Get parameters and the knot vector */

    u = N_AllocReal1dArray( m, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_FitCalcKnotsEndDerivs( u, m, n, p, k, l, knt );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute end control points based on end derivatives */

    ND = N_AllocReal2dArray( NL_MAX( k, l ), p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knt, p, u[0], NL_LEFT, k, ND, &j );

    if( error EQ NL_YES )
        NL_OUT;

    N_PtToCPt( P[0], &Pw[0] );

    for ( i = 1; i <= k; i++ )
    {
        N_CopyCPt( NL_CZERO, &Sw );

        for ( j = 0; j <= i - 1; j++ )
            N_VectorBlendCPt( ND[i][j], Pw[j], &Sw );

        N_CPtToPtEuclid( Sw, &Sk );
        N_Diff2Pts( Ds[i], Sk, &D );

        if( N_FloatOpIsBad( 1.0, ND[i][i], NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        fac = 1.0 / ND[i][i];

        N_ScalePt( fac, D, &D );
        N_PtToCPt( D, &Pw[i] );
    }

    error = N_BasisDerivs( knt, p, u[m], NL_LEFT, l, ND, &j );

    if( error EQ NL_YES )
        NL_OUT;

    N_PtToCPt( P[m], &Pw[n] );

    for ( i = 1; i <= l; i++ )
    {
        N_CopyCPt( NL_CZERO, &Sw );

        for ( j = 0; j <= i - 1; j++ )
            N_VectorBlendCPt( ND[i][p - i + j + 1], Pw[n - i + j + 1], &Sw );

        N_CPtToPtEuclid( Sw, &Sl );
        N_Diff2Pts( De[i], Sl, &D );

        if( N_FloatOpIsBad( 1.0, ND[i][p - i], NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        fac = 1.0 / ND[i][p - i];

        N_ScalePt( fac, D, &D );
        N_PtToCPt( D, &Pw[n - i] );
    }

    if( n EQ k + l + 1 )
        NL_OUT;

    /* Get the Rk's */

    Rk = N_AllocPt1dArray( m - 2, &SL );

    if( Rk EQ NULL )
        NL_QUIT;

    for ( i = 1; i <= m - 1; i++ )
    {
        N_CopyPt( NL_ZERO, &Sk );

        for ( j = 0; j <= k; j++ )
        {
            error = N_BasisIEval( knt, j, p, u[i], NL_LEFT, &nk );

            if( error EQ NL_YES )
                NL_OUT;

            N_CPtToPtEuclid( Pw[j], &D );
            N_VectorBlendPt( nk, D, &Sk );
        }

        N_CopyPt( NL_ZERO, &Sl );

        for ( j = 0; j <= l; j++ )
        {
            error = N_BasisIEval( knt, n - j, p, u[i], NL_LEFT, &nl );

            if( error EQ NL_YES )
                NL_OUT;

            N_CPtToPtEuclid( Pw[n - j], &D );
            N_VectorBlendPt( nl, D, &Sl );
        }

        N_Diff2Pts( P[i], Sk, &D );
        N_Diff2Pts( D, Sl, &Rk[i - 1] );
    }

    /* Compute coefficient matrix */

    bw = 2 * p + 1;
    sbw = p;
    rj = p;
    sj = p - k - 1;
    ej = -2 - k;
    nd = n - k - l - 2;

    A = N_AllocReal2dArray( m - 2, p, &SL );

    if( A EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    index = N_AllocInt1dArray( m - 2, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( nd, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( nd, &SL );

    if( end EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, nd, nd, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &NTN );

    for ( i = 0; i <= nd; i++ )
    {
        for ( j = 0; j < bw; j++ )
            NTN[i][j] = 0.0;
    }

    for ( i = 0; i <= m - 2; i++ )
    {
        for ( j = 0; j <= p; j++ )
            A[i][j] = 0.0;
    }

    for ( i = 0; i <= nd; i++ )
    {
        start[i] = 0;
        end[i] = m - 2;
    }

    for ( i = 1; i <= m - 1; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        ll = NL_MAX( 0, k + 1 - j + p );
        hk = NL_MIN( p, n - l - 1 - j + p ) - ll;

        for ( kk = 0; kk <= hk; kk++ )
            A[i - 1][kk] = N[ll + kk];

        index[i - 1] = NL_MAX( 0, j - p - k - 1 );

        if( j GT rj )
        {
            for ( kk = 1; kk <= j - rj; kk++ )
            {
                sj++;
                ej++;

                if( sj LE nd )
                    start[sj] = i - 1;

                if( ej GE 0 )
                    end[ej] = i - 2;
            }
            rj = j;
        }
    }

    for ( i = 0; i <= nd; i++ )
    {
        lj = NL_MAX( 0, i - p );
        hj = NL_MIN( nd, i + p );

        for ( j = lj; j <= hj; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( kk = lk; kk <= hk; kk++ )
            {
                NTN[i][j - i + sbw] += A[kk][i - index[kk]] * A[kk][j - index[kk]];
            }
        }
    }

    /* Get right hand side of equation */

    R = N_AllocPt1dArray( nd, &SL );

    if( R EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= nd; j++ )
    {
        N_CopyPt( NL_ZERO, &R[j] );

        lk = start[j];
        hk = end[j];

        for ( kk = lk; kk <= hk; kk++ )
        {
            N_VectorBlendPt( A[kk][j - index[kk]], Rk[kk], &R[j] );
        }
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, &Pw[k + 1], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVAPPROXCLOSEDCONDITIONS: Curve interpolation to closed data with end conditions   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve interpolating a closed
     point set.  The degree can be as  high as the number of conditions 
     required, and  the type  of  parametrization can be chosen. If the 
     output curve is initialized to the NULL curve, memory is allocated  
     locally.  Otherwise  it is  checked if enough memory is passed in. 
     A typical calling example:

       NL_POINT   *P;
       NL_INDEX   n, k;
       NL_DEGREE  p;
       NL_VECTOR  *D;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P, and D, and select p);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvApproxClosedConditions(P,n,p,D,k,NL_CHORDLENGTH,&cur,&SG);

     A recommended default for the parametrization  is  NL_CHORDLENGTH. If 
     no derivative is wanted at the ends, set k=0. THE BEST USE OF THIS
     ROUTINE IS TO COMPUTE A C^p-1 CONTINUOUS CLOSED NL_CURVE INTERPOLANT.


   ACCESS:
   
     P     , input  ,  Points to be interpolated, P[0]=P[n]
     n     , input  ,  Highest index in P
     p     , input  ,  Degree of interpolating curve (p <= n+2*k)
     D     , input  ,  Start and end derivatives:
                          NULL: no derivatives passed in. Compute  them
                                internally
                         !NULL: derivatives passed in
     k     , input  ,  Highest index in D;  D[1],...,D[k] are  given at
                       each  end, i.e., the  curve is  closed with  C^k 
                       continuity. k  MUST BE  SPECIFIED EVEN IF NL_NO DE-
                       RIVATIVES ARE PASSED IN!
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization
                         NL_CENTRIPETAL: Centripetal parametrization
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvApproxClosedConditions( NL_POINT *P, NL_INDEX n, NL_DEGREE p, NL_VECTOR *D, NL_INDEX k, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, is, ie, id = 0, nl, nm;

    NL_PARAMETER *u, *v;

    NL_POINT *Q;

    NL_VECTOR *Ds, *De, *Dl;

    NL_KNOTVECTOR *knt;

    NL_CURVE curI;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Allocate memory for end derivatives */

    Ds = N_AllocPt1dArray( k, &SL );

    if( Ds EQ NULL )
        NL_QUIT;

    De = N_AllocPt1dArray( k, &SL );

    if( De EQ NULL )
        NL_QUIT;

    /* Derivatives passed in */

    if( D NEQ NULL )
    {
        for ( i = 1; i <= k; i++ )
        {
            N_VectorCopy( D[i], &Ds[i] );
            N_VectorCopy( D[i], &De[i] );
        }
    }
    else
    {

        /* No derivatives are passed in; compute them internally */

        nm = n / 2;
        is = NL_MAX( nm, n - p - 1 );
        ie = NL_MIN( nm, p + 1 );
        nl = n - is + ie;

        Q = N_AllocPt1dArray( nl, &SL );

        if( Q EQ NULL )
            NL_QUIT;

        Dl = N_AllocPt1dArray( k, &SL );

        if( Dl EQ NULL )
            NL_QUIT;

        u = N_AllocReal1dArray( nl, &SL );

        if( u EQ NULL )
            NL_QUIT;

        v = N_AllocReal1dArray( n, &SL );

        if( v EQ NULL )
            NL_QUIT;

        knt = N_AllocKnotVectorAndArray( nl + p + 1, &SL );

        if( knt EQ NULL )
            NL_QUIT;

        /* Get data points for local fitting */

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, n, NL_EPOINT, par, v );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( P[is], &Q[0] );
        u[0] = 0.0;

        i = is + 1;
        j = 1;

        while( j LE nl )
        {
            N_VectorCopy( P[i], &Q[j] );
            u[j] = u[j - 1] + (v[i] - v[i - 1]);

            if( i EQ n )
            {
                i = 0;
                id = j;
            }

            i++;
            j++;
        }

        /* Interpolate local data through the start point */

        N_FitCrvCalcKnotVector( u, nl, p, knt );

        N_CrvInitArrays( &curI );
        error = N_FitCrvInterpGivenParams( (NL_VOID *)Q, nl, NL_EPOINT, u, knt, p, &curI, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get derivatives off the local interpolant */

        error = N_CrvDerivs( &curI, u[id], NL_LEFT, k, Dl );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 1; i <= k; i++ )
        {
            N_VectorCopy( Dl[i], &Ds[i] );
            N_VectorCopy( Dl[i], &De[i] );
        }
    }

    /* Interpolate data set */

    error = N_FitCrvHighDerivs( P, n, p, Ds, k, De, k, par, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVHIGHDERIVS: Curve interpolation with given higher end derivatives    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve interpolating a given
     set of  points and  specified  end derivatives. The degree can be 
     as high as the number of  conditions specified, and  the type  of 
     parametrization can be chosen. If the output curve is initialized  
     to the NULL curve, memory is allocated  locally.  Otherwise it is  
     checked if enough memory is passed in. A typical calling example:

       NL_POINT   *P;
       NL_INDEX   nn, kk, ll;
       NL_DEGREE  p;
       NL_VECTOR  *Ds, *De;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get arrays P, Ds, De, and select p);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvHighDerivs(P,nn,p,Ds,kk,De,ll,NL_CHORDLENGTH,&cur,&SG);

     A recommended  default for  the parametrization  is  NL_CHORDLENGTH.
     If no derivative is  wanted at either end,  set kk=0 and/or ll=0.


   ACCESS:
   
     P     , input  ,  Points to be interpolated
     nn    , input  ,  Highest index in P
     p     , input  ,  Degree of interpolating curve (p <= nn+kk+ll)
     Ds    , input  ,  Start derivatives
     kk    , input  ,  Highest  start  derivative, i.e.,  at the start 
                       Ds[1],...,Ds[kk] are specified. If kk=0, no de-
                       rivative is computed.
     De    , input  ,  End derivatives
     ll    , input  ,  Highest end derivative, i.e., at the end De[1],
                       ...,De[ll] are specified. ll=0 means no deriva-
                       tive is needed.
     par   , input  ,  Flag:
                         NL_UNIFORM    : Uniform parametrization
                         NL_CHORDLENGTH: Chord length parametrization
                         NL_CENTRIPETAL: Centripetal parametrization
     cur   , output ,  Interpolating curve
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvHighDerivs( NL_POINT *P, NL_INDEX nn, NL_DEGREE p, NL_VECTOR *Ds, NL_INDEX kk, NL_VECTOR *De, NL_INDEX ll, NL_FLAG par, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvHighDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, bw, sbw, row;

    NL_REAL ** A, ** ND, *U, *N;

    NL_PARAMETER *u;

    NL_POINT *Q;

    NL_CPOINT *Pw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for curve memory */

    n = nn + kk + ll;

    if( p LT 1 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    if( kk GE p OR ll GE p )
        NL_ERROR( NL_INP_ERR );

    if( kk LT 0 OR ll LT 0 )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsKnotVectorAndKnots( cur, &Pw, &knt, &U );

    /* Get parameters and compute knot vector */

    u = N_AllocReal1dArray( nn, &SL );

    if( u EQ NULL )
        NL_QUIT;

    error = N_FitCalcCrvParamValues( (NL_VOID *)P, nn, NL_EPOINT, par, u );

    if( error EQ NL_YES )
        NL_OUT;

    N_FitCalcKnotsHighEndDerivs( u, nn, p, kk, ll, knt );

    /* Compute coefficient matrix */

    bw = 2 * p - 1;
    sbw = p - 1;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    ND = N_AllocReal2dArray( NL_MAX( kk, ll ), p, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;

    row = 1;

    error = N_BasisDerivs( knt, p, u[0], NL_LEFT, kk, ND, &j );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 1; i <= kk; i++ )
    {
        for ( j = 0; j <= i; j++ )
            A[row][sbw - i + j] = ND[i][j];

        row++;
    }

    for ( i = 1; i < nn; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - row - 1;

        for ( k = 0; k <= p; k++ )
            A[row][l + k] = N[k];

        row++;
    }

    error = N_BasisDerivs( knt, p, u[nn], NL_LEFT, ll, ND, &j );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = ll; i >= 1; i-- )
    {
        for ( j = 0; j <= i; j++ )
            A[row][sbw + j] = ND[i][p - i + j];

        row++;
    }

    A[row][sbw] = 1.0;

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve system of linear equations */

    Q = N_AllocPt1dArray( n, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    N_CopyPt( P[0], &Q[0] );

    row = 1;

    for ( i = 1; i <= kk; i++ )
        N_CopyPt( Ds[i], &Q[row++] );

    for ( i = 1; i < nn; i++ )
        N_CopyPt( P[i], &Q[row++] );

    for ( i = ll; i >= 1; i-- )
        N_CopyPt( De[i], &Q[row++] );

    N_CopyPt( P[nn], &Q[row] );

    error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVCLOSEDDERIVSPARAMSKNOTS: Curve appr. to closed data with ders pars and knots      */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve approximating a closed
     point set.  The degree can be as  high as the number of conditions 
     required, and  the parameters as  well as the  knot vector must be 
     passed in. If the  output curve is  initialized to the NULL curve, 
     memory is  allocated locally.  Otherwise  it is  checked if enough 
     memory is passed in. A typical calling example:

       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_VECTOR      *D;
       NL_CVECTOR     *Dw;
       NL_INDEX       m, n, k;
       NL_DEGREE      p;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  *knt;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (get P/Pw, D/Dw, u, knt, and select n and p);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvClosedDerivsParamsKnots((NL_VOID *)P, m,NL_EPOINT,(NL_VOID *)D, k,u,knt,n,p,&cur,&SG);
       N_FitCrvClosedDerivsParamsKnots((NL_VOID *)Pw,m,NL_HPOINT,(NL_VOID *)Dw,k,u,knt,n,p,&cur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS  u  AND THE KNOT NL_VECTOR  knt ARE
     PASSED IN.


   ACCESS:
   
     A    , input  ,  NL_VOID pointer to NL_POINT or NL_CPOINT data; A[0]=A[m]!
     m    , input  ,  Highest index in A
     ptp  , input  ,  Flag:
                       NL_EPOINT: Euclidean point passed in
                       NL_HPOINT: Homogeneous point passed in
     B    , input  ,  NL_VOID pointer to NL_VECTOR or NL_CVECTOR data for start 
                      and end derivatives:
                         NULL: no derivatives passed in. Compute  them
                               internally
                        !NULL: derivatives passed in
     k    , input  ,  Highest index in B;  B[1],...,B[k] are  given at
                      each  end, i.e., the  curve is  closed with  C^k 
                      continuity. k  MUST BE  SPECIFIED EVEN IF NL_NO DE-
                      RIVATIVES ARE PASSED IN! k < p!
     u    , input  ,  Parameters where data points are assumed
     knt  , input  ,  Knot vector of approximating curve
     n    , input  ,  Highest  control  point  index of  approximating 
                      curve
     p    , input  ,  Degree of approximating curve
     cur  , output ,  Approximating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvClosedDerivsParamsKnots( NL_VOID *A, NL_INDEX m, NL_FLAG ptp, NL_VOID *B, NL_INDEX k, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_INDEX n, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvClosedDerivsParamsKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, is = 0, ie, id = 0, nl = 0, ml = 0, nm;

    NL_REAL *ul = NULL, *vl = NULL, fr;

    NL_POINT *P, *Q;

    NL_CPOINT *Pw, *Qw;

    NL_VECTOR *D, *Ds, *De, *Dl;

    NL_CVECTOR *Dw, *Dws, *Dwe, *Dwl;

    NL_KNOTVECTOR *knl = NULL;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute some entities */

    if( B EQ NULL )
    {
        nm = m / 2;
        fr = (NL_REAL)((m + 1.0) / (n + 1.0));
        ml = (NL_INDEX)((p + 1.0) * fr + 0.5);
        is = NL_MAX( nm, m - ml );
        ie = NL_MIN( nm, ml );
        ml = m - is + ie;
        fr = (NL_REAL)( ml ) / (NL_REAL)( m );
        nl = (NL_INDEX)(fr * n + 0.5);

        ul = N_AllocReal1dArray( ml, &SL );

        if( ul EQ NULL )
            NL_QUIT;

        vl = N_AllocReal1dArray( m, &SL );

        if( vl EQ NULL )
            NL_QUIT;

        knl = N_AllocKnotVectorAndArray( nl + p + 1, &SL );

        if( knl EQ NULL )
            NL_QUIT;
    }

    /* Get end derivatives and approximate */

    switch( ptp )
    {
        case NL_EPOINT:

            P = (NL_POINT *)A;

            Ds = N_AllocPt1dArray( k, &SL );

            if( Ds EQ NULL )
                NL_QUIT;

            De = N_AllocPt1dArray( k, &SL );

            if( De EQ NULL )
                NL_QUIT;

            if( B NEQ NULL )
            {
                D = (NL_VECTOR *)B;

                for ( i = 1; i <= k; i++ )
                {
                    N_VectorCopy( D[i], &Ds[i] );
                    N_VectorCopy( D[i], &De[i] );
                }
            }
            else
            {
                Q = N_AllocPt1dArray( ml, &SL );

                if( Q EQ NULL )
                    NL_QUIT;

                Dl = N_AllocPt1dArray( k, &SL );

                if( Dl EQ NULL )
                    NL_QUIT;

                error = N_FitCalcCrvParamValues( (NL_VOID *)P, m, NL_EPOINT, NL_CHORDLENGTH, vl );

                if( error EQ NL_YES )
                    NL_OUT;

                N_VectorCopy( P[is], &Q[0] );
                ul[0] = 0.0;

                i = is + 1;
                j = 1;

                while( j LE ml )
                {
                    N_VectorCopy( P[i], &Q[j] );
                    ul[j] = ul[j - 1] + (vl[i] - vl[i - 1]);

                    if( i EQ m )
                    {
                        i = 0;
                        id = j;
                    }

                    i++;
                    j++;
                }

                N_FitCalcKnotVectorCrvApprox( ul, ml, nl, p, knl );

                N_CrvInitArrays( &curA );
                error = N_FitCrvApproxKnots( (NL_VOID *)Q, ml, NL_EPOINT, ul, knl, nl, p, &curA, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_CrvDerivs( &curA, ul[id], NL_LEFT, k, Dl );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 1; i <= k; i++ )
                {
                    N_VectorCopy( Dl[i], &Ds[i] );
                    N_VectorCopy( Dl[i], &De[i] );
                }
            }

            error = N_FitCrvDerivsKnots( (NL_VOID *)P, m, NL_EPOINT, (NL_VOID *)Ds, k, (NL_VOID *)De, k, u, knt, n, p, cur, SG );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        case NL_HPOINT:

            Pw = (NL_CPOINT *)A;

            Dws = N_AllocCPt1dArray( k, &SL );

            if( Dws EQ NULL )
                NL_QUIT;

            Dwe = N_AllocCPt1dArray( k, &SL );

            if( Dwe EQ NULL )
                NL_QUIT;

            if( B NEQ NULL )
            {
                Dw = (NL_CVECTOR *)B;

                for ( i = 1; i <= k; i++ )
                {
                    N_CopyCPt( Dw[i], &Dws[i] );
                    N_CopyCPt( Dw[i], &Dwe[i] );
                }
            }
            else
            {
                Qw = N_AllocCPt1dArray( ml, &SL );

                if( Qw EQ NULL )
                    NL_QUIT;

                Dwl = N_AllocCPt1dArray( k, &SL );

                if( Dwl EQ NULL )
                    NL_QUIT;

                error = N_FitCalcCrvParamValues( (NL_VOID *)Pw, m, NL_HPOINT, NL_CHORDLENGTH, vl );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CopyCPt( Pw[is], &Qw[0] );
                ul[0] = 0.0;

                i = is + 1;
                j = 1;

                while( j LE ml )
                {
                    N_CopyCPt( Pw[i], &Qw[j] );
                    ul[j] = ul[j - 1] + (vl[i] - vl[i - 1]);

                    if( i EQ m )
                    {
                        i = 0;
                        id = j;
                    }

                    i++;
                    j++;
                }

                N_FitCalcKnotVectorCrvApprox( ul, ml, nl, p, knl );

                N_CrvInitArrays( &curA );
                error = N_FitCrvApproxKnots( (NL_VOID *)Qw, ml, NL_HPOINT, ul, knl, nl, p, &curA, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_CrvDerivsAtKnot( &curA, ul[id], NL_LEFT, k, Dwl );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 1; i <= k; i++ )
                {
                    N_CopyCPt( Dwl[i], &Dws[i] );
                    N_CopyCPt( Dwl[i], &Dwe[i] );
                }
            }

            error = N_FitCrvDerivsKnots( (NL_VOID *)Pw, m, NL_HPOINT, (NL_VOID *)Dws, k, (NL_VOID *)Dwe, k, u, knt, n, p, cur, SG );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVDERIVSKNOTS: Curve appoximation with end derivatives and knots        */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine computes a least-squares NURBS curve approxi-
     mation to a  given set of  points  with end derivative constraints. 
     The  degree is  arbitrary, the  parameters and  the  knots must  be 
     passed in, and the number of  control  points can be chosen. If the 
     output curve is initialized to the NULL curve, memory is  allocated  
     locally. Otherwise it is checked if  enough memory is  passed in. A 
     typical calling example is:
 
       NL_POINT       *B;
       NL_CPOINT      *Bw;
       NL_VECTOR      *Bs, *Be;
       NL_CVECTOR     *Bws, *Bwe;
       NL_PARAMETER   *u;
       NL_KNOTVECTOR  *knt;
       NL_INDEX       m, n, k, l;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (define B/Bw, Bs/Bws, Be/Bwe, u, knt, and choose p and n);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvDerivsKnots((NL_VOID *)B, m,NL_EPOINT,(NL_VOID *)Bs, k,(NL_VOID *)Be, l,u,knt,
                n,p,&cur,&SG);
       N_FitCrvDerivsKnots((NL_VOID *)Bw,m,NL_HPOINT,(NL_VOID *)Bws,k,(NL_VOID *)Bwe,l,u,knt,
                n,p,&cur,&SG);
 
     IT IS ASSUMED THAT THE NL_PARAMETERS  u  AND THE KNOT NL_VECTOR  knt  ARE 
     COMPUTED IN THE CALLING ROUTINE.
 
 
   ACCESS:
   
     B   , input  ,  NL_VOID pointer to NL_POINT or NL_CPOINT data points
     m   , input  ,  Highest index in B
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point passed in
                       NL_HPOINT: Homogeneous point passed in
     Bs  , input  ,  NL_VOID pointer to NL_VECTOR or NL_CVECTOR derivative data
     k   , input  ,  Highest start derivative, i.e., at the start Bs[1],
                     ...,Bs[k] are specified. If k=0, no  derivative is 
                     needed. Must satisfy k < p!
     Be  , input  ,  NL_VOID pointer to NL_VECTOR or NL_CVECTOR derivative data
     l   , input  ,  Highest end derivative, i.e., at the end Be[1],...,
                     Be[l] are  specified. l=0  means no  derivative  is 
                     needed. Must satisfy l < p!
     u   , input  ,  Parameters corresponding to data points
     knt , input  ,  knot vector of approximating curve
     n   , input  ,  Highest index of control points. The following must
                     hold: n <= m  and n > k+l!
     p   , input  ,  Degree of approximating curve (p <= n!)
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCrvDerivsKnots( NL_VOID *B, NL_INDEX m, NL_FLAG ptp, NL_VOID *Bs, NL_INDEX k, NL_VOID *Be, NL_INDEX l, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_INDEX n, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvDerivsKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, kk, ll, bw, sbw, rj, sj, ej, lk, hk, lj, hj, nd, mk, mc;

    NL_REAL ** A, ** NTN, ** NDs, ** NDe, *N, *UC, *UK, fac, nk, nl;

    NL_POINT *Q, *Rk, *R, Sk, Sl, D;

    NL_VECTOR *Ds, *De;

    NL_CPOINT *Qw, *Pw, *Rwk, *Rw, Dw, Sw, Swk, Swl;

    NL_CVECTOR *Dws, *Dwe;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error and curve memory */

    N_KnotVectorGetKnots( knt, &mk, &UK );

    mc = n + p + 1;

    if( mc NEQ mk )
        NL_ERROR( NL_INP_ERR );

    if( p GT n )
        NL_ERROR( NL_INP_ERR );

    if( k GE p OR l GE p )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR l LT 0 )
        NL_ERROR( NL_INP_ERR );

    if( m LT n OR n LE k + l )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    for ( i = 0; i <= mc; i++ )
        UC[i] = UK[i];

    /* Compute end control points based on end derivatives */

    NDs = N_AllocReal2dArray( k, p, &SL );

    if( NDs EQ NULL )
        NL_QUIT;

    NDe = N_AllocReal2dArray( l, p, &SL );

    if( NDe EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knt, p, u[0], NL_LEFT, k, NDs, &j );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisDerivs( knt, p, u[m], NL_LEFT, l, NDe, &j );

    if( error EQ NL_YES )
        NL_OUT;

    switch( ptp )
    {
        case NL_EPOINT:

            Q = (NL_POINT *)B;
            Ds = (NL_VECTOR *)Bs;
            De = (NL_VECTOR *)Be;

            N_PtToCPt( Q[0], &Pw[0] );
            N_PtToCPt( Q[m], &Pw[n] );

            for ( i = 1; i <= k; i++ )
            {
                N_CopyCPt( NL_CZERO, &Sw );

                for ( j = 0; j <= i - 1; j++ )
                    N_VectorBlendCPt( NDs[i][j], Pw[j], &Sw );

                N_CPtToPtEuclid( Sw, &Sk );
                N_Diff2Pts( Ds[i], Sk, &D );

                if( N_FloatOpIsBad( 1.0, NDs[i][i], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                fac = 1.0 / NDs[i][i];

                N_ScalePt( fac, D, &D );
                N_PtToCPt( D, &Pw[i] );
            }

            for ( i = 1; i <= l; i++ )
            {
                N_CopyCPt( NL_CZERO, &Sw );

                for ( j = 0; j <= i - 1; j++ )
                    N_VectorBlendCPt( NDe[i][p - i + j + 1], Pw[n - i + j + 1], &Sw );

                N_CPtToPtEuclid( Sw, &Sl );
                N_Diff2Pts( De[i], Sl, &D );

                if( N_FloatOpIsBad( 1.0, NDe[i][p - i], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                fac = 1.0 / NDe[i][p - i];

                N_ScalePt( fac, D, &D );
                N_PtToCPt( D, &Pw[n - i] );
            }

            break;

        case NL_HPOINT:

            Qw = (NL_CPOINT *)B;
            Dws = (NL_CVECTOR *)Bs;
            Dwe = (NL_CVECTOR *)Be;

            N_CopyCPt( Qw[0], &Pw[0] );
            N_CopyCPt( Qw[m], &Pw[n] );

            for ( i = 1; i <= k; i++ )
            {
                N_CopyCPt( NL_CZERO, &Sw );

                for ( j = 0; j <= i - 1; j++ )
                    N_VectorBlendCPt( NDs[i][j], Pw[j], &Sw );

                N_Diff2CPts( Dws[i], Sw, &Dw );

                if( N_FloatOpIsBad( 1.0, NDs[i][i], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                fac = 1.0 / NDs[i][i];

                N_ScaleCPt( fac, Dw, &Pw[i] );
            }

            for ( i = 1; i <= l; i++ )
            {
                N_CopyCPt( NL_CZERO, &Sw );

                for ( j = 0; j <= i - 1; j++ )
                    N_VectorBlendCPt( NDe[i][p - i + j + 1], Pw[n - i + j + 1], &Sw );

                N_Diff2CPts( Dwe[i], Sw, &Dw );

                if( N_FloatOpIsBad( 1.0, NDe[i][p - i], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                fac = 1.0 / NDe[i][p - i];

                N_ScaleCPt( fac, Dw, &Pw[n - i] );
            }

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    if( n EQ k + l + 1 )
        NL_OUT;

    /* Compute coefficient matrix */

    bw = 2 * p + 1;
    sbw = p;
    rj = p;
    sj = p - k - 1;
    ej = -2 - k;
    nd = n - k - l - 2;

    A = N_AllocReal2dArray( m - 2, p, &SL );

    if( A EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    index = N_AllocInt1dArray( m - 2, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( nd, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( nd, &SL );

    if( end EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, nd, nd, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &NTN );

    for ( i = 0; i <= nd; i++ )
    {
        for ( j = 0; j < bw; j++ )
            NTN[i][j] = 0.0;
    }

    for ( i = 0; i <= m - 2; i++ )
    {
        for ( j = 0; j <= p; j++ )
            A[i][j] = 0.0;
    }

    for ( i = 0; i <= nd; i++ )
    {
        start[i] = 0;
        end[i] = m - 2;
    }

    for ( i = 1; i <= m - 1; i++ )
    {
        error = N_BasisEval( knt, p, u[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        ll = NL_MAX( 0, k + 1 - j + p );
        hk = NL_MIN( p, n - l - 1 - j + p ) - ll;

        for ( kk = 0; kk <= hk; kk++ )
            A[i - 1][kk] = N[ll + kk];

        index[i - 1] = NL_MAX( 0, j - p - k - 1 );

        if( j GT rj )
        {
            for ( kk = 1; kk <= j - rj; kk++ )
            {
                sj++;
                ej++;

                if( sj LE nd )
                    start[sj] = i - 1;

                if( ej GE 0 )
                    end[ej] = i - 2;
            }
            rj = j;
        }
    }

    for ( i = 0; i <= nd; i++ )
    {
        lj = NL_MAX( 0, i - p );
        hj = NL_MIN( nd, i + p );

        for ( j = lj; j <= hj; j++ )
        {
            lk = NL_MAX( start[i], start[j] );
            hk = NL_MIN( end[i], end[j] );

            for ( kk = lk; kk <= hk; kk++ )
            {
                NTN[i][j - i + sbw] += A[kk][i - index[kk]] * A[kk][j - index[kk]];
            }
        }
    }

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Set up and solve system of equations */

    switch( ptp )
    {
        case NL_EPOINT:

            Q = (NL_POINT *)B;

            Rk = N_AllocPt1dArray( m - 2, &SL );

            if( Rk EQ NULL )
                NL_QUIT;

            for ( i = 1; i <= m - 1; i++ )
            {
                N_CopyPt( NL_ZERO, &Sk );
                N_CopyPt( NL_ZERO, &Sl );

                for ( j = 0; j <= k; j++ )
                {
                    error = N_BasisIEval( knt, j, p, u[i], NL_LEFT, &nk );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CPtToPtEuclid( Pw[j], &D );
                    N_VectorBlendPt( nk, D, &Sk );
                }

                for ( j = 0; j <= l; j++ )
                {
                    error = N_BasisIEval( knt, n - j, p, u[i], NL_LEFT, &nl );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CPtToPtEuclid( Pw[n - j], &D );
                    N_VectorBlendPt( nl, D, &Sl );
                }

                N_Diff2Pts( Q[i], Sk, &D );
                N_Diff2Pts( D, Sl, &Rk[i - 1] );
            }

            R = N_AllocPt1dArray( nd, &SL );

            if( R EQ NULL )
                NL_QUIT;

            for ( j = 0; j <= nd; j++ )
            {
                N_CopyPt( NL_ZERO, &R[j] );

                lk = start[j];
                hk = end[j];

                for ( kk = lk; kk <= hk; kk++ )
                {
                    N_VectorBlendPt( A[kk][j - index[kk]], Rk[kk], &R[j] );
                }
            }

            error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, &Pw[k + 1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        case NL_HPOINT:

            Qw = (NL_CPOINT *)B;

            Rwk = N_AllocCPt1dArray( m - 2, &SL );

            if( Rwk EQ NULL )
                NL_QUIT;

            for ( i = 1; i <= m - 1; i++ )
            {
                N_CopyCPt( NL_CZERO, &Swk );
                N_CopyCPt( NL_CZERO, &Swl );

                for ( j = 0; j <= k; j++ )
                {
                    error = N_BasisIEval( knt, j, p, u[i], NL_LEFT, &nk );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorBlendCPt( nk, Pw[j], &Swk );
                }

                for ( j = 0; j <= l; j++ )
                {
                    error = N_BasisIEval( knt, n - j, p, u[i], NL_LEFT, &nl );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorBlendCPt( nl, Pw[n - j], &Swl );
                }

                N_Diff2CPts( Qw[i], Swk, &Dw );
                N_Diff2CPts( Dw, Swl, &Rwk[i - 1] );
            }

            Rw = N_AllocCPt1dArray( nd, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( j = 0; j <= nd; j++ )
            {
                N_CopyCPt( NL_CZERO, &Rw[j] );

                lk = start[j];
                hk = end[j];

                for ( kk = lk; kk <= hk; kk++ )
                {
                    N_VectorBlendCPt( A[kk][j - index[kk]], Rwk[kk], &Rw[j] );
                }
            }

            error = N_RealMatrixForBack( &cm, (NL_VOID *)Rw, NL_HPOINT, &Pw[k + 1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITHERMITE: Fit Hermite curve to end kth and (k+1)th derivatives     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine generates a cubic Hermite curve given the k-th 
     and the (k+1)th derivatives at the ends. That is, if end points and
     end tangents are given, a regular Hermite curve is computed. If end
     derivatives and twists are given, a cross-boundary devivative field
     is computed, etc. A typical calling example is:

       NL_CURVE   cur;
       NL_POINT   Ps, Pe;
       NL_VECTOR  Ds, De;
       NL_STACKS  SG;
       ...
       (get Ps, Pe, Ds, De);
       ...
       N_CrvInitArrays(&cur);
       N_FitHermite(Ps,Pe,Ds,De,&cur,&SG);


   ACCESS:
   
     Ps,Pe , input  ,  End k-th  derivatives,  e.g.  end  points  or end 
                       derivatives
     Ds,De , input  ,  End (k+1)th derivatives, e.g. end tangents or end
                       twists
     cur   , output ,  Cubic Hermite interpolating all end derivatives
     SG    , input  ,  cur' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitHermite( NL_POINT Ps, NL_POINT Pe, NL_VECTOR Ds, NL_VECTOR De, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitHermite");

    NL_FLAG error = NL_NO;

    NL_INDEX i, m, n;

    NL_DEGREE p;

    NL_REAL *U, oneth;

    NL_POINT Q1, Q2;

    NL_CPOINT *Pw;

    /* Compute Hermite cubic */

    oneth = 1.0 / 3.0;

    error = N_CrvSizeArrays( cur, 3, 3, 7, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = 0.0;
        U[4 + i] = 1.0;
    }

    N_PtToCPt( Ps, &Pw[0] );
    N_PtToCPt( Pe, &Pw[3] );

    N_VectorPtAlongVector( Ps, oneth, Ds, &Q1 );
    N_VectorPtAlongVector( Pe, -oneth, De, &Q2 );

    N_PtToCPt( Q1, &Pw[1] );
    N_PtToCPt( Q2, &Pw[2] );

    /* End NURBS and Exit */

    EXIT:

    return (error);
}


/**********************************************************************/
/* N_FITCALCKNOTSDERIVS: Knot vector for curve interpolation with all derivatives */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes the knot vector for global curve  
     interpolation with all derivatives specified. A typical calling 
     example is:

       NL_REAL        *u;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose p and define knt);
       ...
       N_FitCalcKnotsDerivs(u,k,p,&knt);

     IT IS ASSUMED THAT  MEMORY FOR knt IS  ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     u   , input  ,  Parameters
     k   , input  ,  Highest index in u
     p   , input  ,  Degree of interpolating curve (MUST BE 2 OR 3!)
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FitCalcKnotsDerivs( NL_REAL *u, NL_INDEX k, NL_DEGREE p, NL_KNOTVECTOR *knt )
{

    NL_INDEX i, j, n;

    NL_REAL *U, alf, bet;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute knot vector */

    n = 2 * k + 1;
    N_KnotVectorGetKnots( knt, &i, &U );

    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[n + i + 1] = u[k];
    }

    j = p + 1;

    switch( p )
    {
        case 2:

            U[p + 1] = 0.5 *( u[1] + u[0] );

            for ( i = 1; i < k; i++ )
            {
                U[++j] = u[i];
                U[++j] = 0.5 *( u[i] + u[i + 1] );
            }
            break;

        case 3:
            if( k GT 1 )
            {
                U[p + 1] = 0.5 *( u[1] + u[0] );
                U[n] = 0.5 *( u[k] + u[k - 1] );
            }

            alf = 1.0 / 3.0;
            bet = 2.0 / 3.0;

            for ( i = 1; i < k - 1; i++ )
            {
                U[++j] = bet * u[i] + alf * u[i + 1];
                U[++j] = alf * u[i] + bet * u[i + 1];
            }
            break;
    }

    N_SetKnotIndex( knt, n + p + 1 );

    /* End NURBS */

    N_EndNurbs( &SL );
}

/**********************************************************************/
/* N_FITCALCKNOTSENDDERIVS: Knot vector for curve approximation with end derivatives */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes the knot  vector for global curve 
     approximation with arbitrary end derivatives specified. A typical 
     calling example is:

       NL_REAL        *u;
       NL_INDEX       m, n, k, l;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose n, p, k, and l, and define knt);
       ...
       N_FitCalcKnotsEndDerivs(u,m,n,p,k,l,&knt);

     IT IS ASSUMED THAT MEMORY FOR  knt IS  ALLOCATED  IN THE  CALLING
     ROUTINE. 


   ACCESS:
   
     u   , input  ,  Parameters
     m   , input  ,  Highest index in u
     n   , input  ,  Highest index of control point array (n <= m)
     k,l , input  ,  Highest end derivatives: at the  start up to  the
                     k-th and at the end up to the l-th are  specified
                     (n > k+l)
     p   , input  ,  Degree of approximating curve (p<n) 
     knt , output ,  Knot vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcKnotsEndDerivs( NL_REAL *u, NL_INDEX m, NL_INDEX n, NL_DEGREE p, NL_INDEX k, NL_INDEX l, NL_KNOTVECTOR *knt )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcKnotsEndDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, il, ih, is, ie, js, je, r, nc;

    NL_REAL *U, *uk, le, d, sum;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize */

    if( n GT m )
        NL_ERROR( NL_INP_ERR );

    if( n LE k + l )
        NL_ERROR( NL_INP_ERR );

    nc = n - k - l;

    N_KnotVectorGetKnots( knt, &i, &U );

    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[n + i + 1] = u[m];
    }

    /* Compute representatives of clusters of parameters */

    uk = N_AllocReal1dArray( nc, &SL );

    if( uk EQ NULL )
        NL_QUIT;

    d = (m + 1.0) / (nc + 1.0);
    il = 0;
    ih = il;
    le = -1.0;

    for ( i = 0; i <= nc; i++ )
    {
        le = le + d;
        ih = ROUND( le );

        if( il EQ ih )
        {
            uk[i] = u[il];
        }
        else
        {
            sum = 0.0;

            for ( j = il; j <= ih; j++ )
                sum += u[j];
            uk[i] = sum / ((NL_REAL)ih - (NL_REAL)il + (NL_REAL)1);
        }

        il = ih + 1;
    }

    /* Now compute the knot vector */

    is = 1 - k;
    ie = nc - p + l;
    r = p;

    for ( i = is; i <= ie; i++ )
    {
        js = NL_MAX( 0, i );
        je = NL_MIN( nc, i + p - 1 );

        sum = 0.0;

        for ( j = js; j <= je; j++ )
            sum += uk[j];
        U[++r] = sum / ((NL_REAL)je - (NL_REAL)js + (NL_REAL)1);
    }

    N_SetKnotIndex( knt, n + p + 1 );

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCALCKNOTSHIGHENDDERIVS: Knot vector for interpolation with high end derivatives  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes the knot  vector for  global curve  
     interpolation with end derivatives specified up to any derivative
     less than the degree. A typical calling example is:

       NL_REAL        *u;
       NL_INDEX       n, k, l;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (get array u, choose p, k and l, and allocate memory for knt);
       ...
       N_FitCalcKnotsHighEndDerivs(u,n,p,k,l,&knt);

     IT IS ASSUMED THAT  MEMORY FOR knt IS  ALLOCATED  IN THE  CALLING
     ROUTINE (THE HIGHEST NL_INDEX MUST BE AT LEAST n+k+l+p+1)!


   ACCESS:
   
     u   , input  ,  Parameters
     n   , input  ,  Highest index in u
     p   , input  ,  Degree of interpolating curve
     k,l , input  ,  Highest derivatives at the ends:
                       1st thru the kth are given at the start
                       1st thru the lth are given at the end
                       k,l less than p must hold!
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FitCalcKnotsHighEndDerivs( NL_REAL *u, NL_INDEX n, NL_DEGREE p, NL_INDEX k, NL_INDEX l, NL_KNOTVECTOR *knt )
{

    NL_INDEX i, j, m, r, is, ie, js, je;

    NL_REAL *U, sum;

    /* Compute knot vector */

    m = n + k + l;
    N_KnotVectorGetKnots( knt, &i, &U );

    for ( i = 0; i <= p; i++ )
    {
        U[i] = u[0];
        U[m + i + 1] = u[n];
    }

    is = 1 - k;
    ie = n - p + l;
    r = p;

    for ( i = is; i <= ie; i++ )
    {
        js = NL_MAX( 0, i );
        je = NL_MIN( n, i + p - 1 );

        sum = 0.0;

        for ( j = js; j <= je; j++ )
            sum += u[j];
        U[++r] = sum / ((NL_REAL)je - (NL_REAL)js + (NL_REAL)1);
    }

    N_SetKnotIndex( knt, m + p + 1 );
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITLOCALCUBICAPPROX: Check scatter and compute local cubic approximant        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  computes a best fitting NURBS cubic arc to a
     set of  points and checks if  the error of approximation is within 
     a given tolerance. A typical calling example is:

       NL_POINT      *P;
       NL_INDEX      ks, ke, n, m;
       NL_VECTOR     *T;
       NL_PARAMETER  *u;       
       NL_REAL       E, top, toc;
       NL_CPOINT     Pw[4];
       ...
       (get arrays P, T and u; get ks, ke, E, top, toc);
       ...
       N_FitLocalCubicApprox(P,m,T,ks,ke,u,E,top,toc,Pw,&n);

     This  routine  is  the  working  bee  used by  N_FITCRVCUBICAPPROX  to  fit a 
     piecewise cubic curve to data.


   ACCESS:
   
     P     , input  ,  Points to be approximated
     m     , input  ,  Highest index in P
     T     , input  ,  Unit tangents at each data point
     ks,ke , input  ,  Start   and  end   indexes   of   points  to  be 
                       approximated
     u     , input  ,  Parameter values the P[i]s are assumed at
     E     , input  ,  Error tolerance
     top   , input  ,  Point coincidence tolerance for Newton (relative
                       tolerance)
     toc   , input  ,  Cosine tolerance for Newton
     Pw    , output ,  Control points of approximating curve
     n     , output ,  Highest index in Pw


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitLocalCubicApprox( NL_POINT *P, NL_INDEX m, NL_VECTOR *T, NL_INDEX ks, NL_INDEX ke, NL_PARAMETER *u, NL_REAL E, NL_REAL top, NL_REAL toc, NL_CPOINT *Pw, NL_INDEX *n )
{

    NL_FLAG its, type, prj, apr, newton = NL_NO, error = NL_NO;

    NL_INDEX i, j, k;

    NL_REAL *alfs, *bets, U[8], a[7][3], b[7], A[3][3], B[3], alf, bet, gam, sd, d, ts, te, fact, s, s2, s3, t, t2, t3, dena, denb, numa, numb, salf, sbet, aa, bb, d2, d3;

    NL_PARAMETER *uh;

    NL_POINT R, P1, P2, Pc, Pd;

    NL_CPOINT Qw[4];

    NL_VECTOR S, SU, V, N = { 0,0,0 }, Ns, Ne, N2;

    NL_LINESEG ls, le, lc, ld, lse;

    NL_PLANE pst;

    NL_CURVE cur;

    NL_STACKS SL;

    NL_PRIVATE_TLS NL_REAL c[4] =
        {
        1.0, 0.0, -3.0, 2.0
        };

    NL_PRIVATE NL_STRING rname = _T("N_FitLocalCubicApprox");
    NL_PRIVATE NL_REAL EP = 1.0e-05;
    NL_PRIVATE NL_REAL G[101] =
        {
        1.000000, 0.941096, 0.915962, 0.896352, 0.879596, 0.864650, 0.850983, 0.838281, 0.826339, 0.815013, 0.804191, 0.793816, 0.783813, 0.774133, 0.764739, 0.755597, 0.746681, 0.737968, 0.729437, 0.721073, 0.712859, 0.704784, 0.696835, 0.689002, 0.681276, 0.673648, 0.666111, 0.658658, 0.651283, 0.643980, 0.636736, 0.629562, 0.622445, 0.615379, 0.608362, 0.601389, 0.594457, 0.587562, 0.580701, 0.573871, 0.567069, 0.560292, 0.553538, 0.546803, 0.540086, 0.533383, 0.526692, 0.520011, 0.513333, 0.506667, 0.500000, 0.493333, 0.486667, 0.479989, 0.473308, 0.466617, 0.459914, 0.453197, 0.446462, 0.439708, 0.432931, 0.426129, 0.419299, 0.412438, 0.405543, 0.398611, 0.391638, 0.384621, 0.377555, 0.370438, 0.363264, 0.356020, 0.348717, 0.341342, 0.333889, 0.326352, 0.318724, 0.310998, 0.303165, 0.295216, 0.287141, 0.278927, 0.270563, 0.262032, 0.253319, 0.244403, 0.235261, 0.225867, 0.216187, 0.206184, 0.195809, 0.184987, 0.173661, 0.161719, 0.149017, 0.135350, 0.120404, 0.103648, 0.084038,
            0.058904, 0.000000
        };

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize certain entities */

    if( ks GE ke )
        NL_ERROR( NL_INP_ERR );

    *n = 0;
    type = 0;

    N_VectorDiff( P[ks], P[ke], &S );

    error = N_VectorNormalize( S, &SU, &sd );

    if( error EQ NL_YES OR sd LT NL_MTOL )
        NL_OUT;

    N_Pts1dGetMaxExtent( &P[ks], ke - ks, &d );

    top *= d;

    /* Get different cases */

    N_CreateLineStartDirVector( &ls, P[ks], T[ks], NL_UNBOUNDED );
    N_CreateLineStartDirVector( &le, P[ke], T[ke], NL_UNBOUNDED );

    if( ke - ks EQ 1 )
    {
        type = 1; /* Interpolatory segment */
    }
    else
    {
        error = N_IsectLineLine( ls, le, &R, &ts, &te, &its );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCross( T[ks], SU, &N );
        N_VectorMagnitude( N, &d );
        N_VectorCross( T[ke], SU, &N2 );
        N_VectorMagnitude( N2, &d2 );

        type = -1;

        if( d LT NL_LTOL OR d2 LT NL_LTOL )
        {
            N_CreateLinePtPt( &lse, P[ks], P[ke], NL_BOUNDED );

            for ( i = ks + 1; i < ke; i++ )
            {
                error = N_ProjectPtLine( lse, P[i], &R, &alf, &prj );

                if( error EQ NL_YES OR prj EQ NL_FALSE )
                    break;

                N_DistPtPt( P[i], R, &d3 );

                if( d3 GT E )
                    break;
            }

            if( i GE ke )
                type = 2; /* Colinear points */
            else if( d LE NL_LTOL )
            {
                if( d2 GT NL_LTOL )
                    N_CopyPt( N2, &N );
                else
                    NL_OUT;
            }
        }

        if( type EQ - 1 )
        {
            if( its EQ NL_TRUE ) /* Tangents are not parallel */
            {
                N_VectorPtAlongVector( P[ks], ts, T[ks], &P1 );
                N_VectorPtAlongVector( P[ke], te, T[ke], &P2 );
                N_DistPtPt( P1, P2, &d );

                if( d LT NL_MTOL )
                    type = 3;
                else          /* Coplanar case */
                    type = 4; /* 3-D case      */
            }
            else              /* Tangents are parallel */
            {
                type = 3;     /* Coplanar points  */
            }
        }
    }

    /* Switch to the different cases */

    switch( type )
    {
        case 1: /* Interpolatory curve required */

            N_DistPolygon( P, m, &d );

            d = d * (u[ke] - u[ks]);
            alf = d / 3.0;
            bet = -d / 3.0;

            N_VectorPtAlongVector( P[ks], alf, T[ks], &P1 );
            N_VectorPtAlongVector( P[ke], bet, T[ke], &P2 );

            N_PtToCPt( P[ks], &Pw[0] );
            N_PtToCPt( P1, &Pw[1] );
            N_PtToCPt( P2, &Pw[2] );
            N_PtToCPt( P[ke], &Pw[3] );

            *n = 3;
            NL_OUT;

        case 2: /* Collinear points */

            alf = 2.0 / 3.0;
            bet = 1.0 / 3.0;

            N_Combine2Pts( alf, P[ks], bet, P[ke], &P1 );
            N_Combine2Pts( bet, P[ks], alf, P[ke], &P2 );

            N_PtToCPt( P[ks], &Pw[0] );
            N_PtToCPt( P1, &Pw[1] );
            N_PtToCPt( P2, &Pw[2] );
            N_PtToCPt( P[ke], &Pw[3] );

            *n = 3;
            NL_OUT;

        case 3: /* Coplanar points */

            N_CreatePlanePtNormal( &pst, P[ks], N );

            for ( i = ks + 1; i < ke; i++ )
            {
                error = N_ProjectPtPlane( pst, P[i], &R );

                if( error EQ NL_YES )
                    NL_OUT;

                N_DistPtPt( P[i], R, &d );

                if( d GT E )
                    NL_OUT;
            }

            uh = N_AllocReal1dArray( ke - ks, &SL );

            if( uh EQ NULL )
                NL_QUIT;

            alfs = N_AllocReal1dArray( ke - ks, &SL );

            if( alfs EQ NULL )
                NL_QUIT;

            bets = N_AllocReal1dArray( ke - ks, &SL );

            if( bets EQ NULL )
                NL_QUIT;

            salf = 0.0;
            sbet = 0.0;
            uh[0] = 0.0;
            fact = 1.0 / (u[ke] - u[ks]);

            for ( i = ks + 1; i < ke; i++ )
            {
                /* Get parameters */

                j = i - ks;
                uh[j] = uh[j - 1] + fact * (u[i] - u[i - 1]);

                s = 1.0 - uh[j];
                s2 = s * s;
                s3 = s * s2;
                t = uh[j];
                t2 = t * t;
                t3 = t * t2;
                gam = 2.0 *s * t;

                /* Set up overdetermined system */

                N_VectorCross( T[i], T[ks], &Ns );
                N_VectorCross( T[i], T[ke], &Ne );
                N_VectorCross( T[i], S, &N );

                alf = 3.0 *s2 * t;
                N_VectorScale( T[ks], alf, &V );
                N_PtToXYZ( V, &a[1][1], &a[2][1], &a[3][1] );

                alf = 3.0 *s * t2;
                N_VectorScale( T[ke], alf, &V );
                N_PtToXYZ( V, &a[1][2], &a[2][2], &a[3][2] );

                alf = s2 - gam;
                N_VectorScale( Ns, alf, &V );
                N_PtToXYZ( V, &a[4][1], &a[5][1], &a[6][1] );

                alf = gam - t2;
                N_VectorScale( Ne, alf, &V );
                N_PtToXYZ( V, &a[4][2], &a[5][2], &a[6][2] );

                alf = -(s3 + 3.0 *s2 * t);
                bet = -(t3 + 3.0 *s * t2);

                N_TranslateSum2Pts( P[i], alf, P[ks], bet, P[ke], &V );
                N_PtToXYZ( V, &b[1], &b[2], &b[3] );

                N_VectorScale( N, gam, &V );
                N_PtToXYZ( V, &b[4], &b[5], &b[6] );

                /* Solve by least squares */

                A[1][1] = a[1][1] * a[1][1] + a[2][1] * a[2][1] + a[3][1] * a[3][1] + a[4][1] * a[4][1] + a[5][1] * a[5][1] + a[6][1] * a[6][1];
                A[1][2] = a[1][1] * a[1][2] + a[2][1] * a[2][2] + a[3][1] * a[3][2] + a[4][1] * a[4][2] + a[5][1] * a[5][2] + a[6][1] * a[6][2];
                A[2][1] = A[1][2];
                A[2][2] = a[1][2] * a[1][2] + a[2][2] * a[2][2] + a[3][2] * a[3][2] + a[4][2] * a[4][2] + a[5][2] * a[5][2] + a[6][2] * a[6][2];

                B[1] = a[1][1] * b[1] + a[2][1] * b[2] + a[3][1] * b[3] + a[4][1] * b[4] + a[5][1] * b[5] + a[6][1] * b[6];
                B[2] = a[1][2] * b[1] + a[2][2] * b[2] + a[3][2] * b[3] + a[4][2] * b[4] + a[5][2] * b[5] + a[6][2] * b[6];

                dena = A[1][1] * A[2][2] - A[1][2] * A[2][1];
                numa = B[1] * A[2][2] - B[2] * A[1][2];
                numb = B[2] * A[1][1] - B[1] * A[2][1];

                if( N_FloatOpIsBad( numa, dena, NL_DIVISION ) )
                    NL_OUT;

                if( N_FloatOpIsBad( numb, dena, NL_DIVISION ) )
                    NL_OUT;

                alfs[j] = numa / dena;
                bets[j] = numb / dena;

                if( alfs[j]LT NL_MTOL OR bets[j]GT - NL_MTOL )
                    NL_OUT;

                salf += alfs[j];
                sbet += bets[j];
            }
            break;

        case 4: /* 3-D case */

            N_CreatePlanePtNormal( &pst, P[ks], N );
            N_CreateLinePtPt( &lse, P[ks], P[ke], NL_BOUNDED );

            uh = N_AllocReal1dArray( ke - ks, &SL );

            if( uh EQ NULL )
                NL_QUIT;

            alfs = N_AllocReal1dArray( ke - ks, &SL );

            if( alfs EQ NULL )
                NL_QUIT;

            bets = N_AllocReal1dArray( ke - ks, &SL );

            if( bets EQ NULL )
                NL_QUIT;

            salf = 0.0;
            sbet = 0.0;

            for ( i = ks + 1; i < ke; i++ )
            {
                j = i - ks;

                /* Get Pc and Pd */

                N_CreateLineStartDirVector( &ld, P[i], T[ke], NL_UNBOUNDED );

                error = N_IsectLinePlane( ld, pst, &Pd, &alf, &its );

                if( error EQ NL_YES OR its EQ NL_FALSE )
                    NL_OUT;

                N_CreateLineStartDirVector( &lc, Pd, T[ks], NL_UNBOUNDED );

                error = N_IsectLineLine( lc, lse, &Pc, &alf, &bet, &its );

                if( error EQ NL_YES OR its EQ NL_FALSE )
                    NL_OUT;

                /* Get uh */

                N_DistPtPt( Pc, Pd, &aa );
                N_DistPtPt( Pd, P[i], &bb );
                N_DistPtPt( Pc, P[ke], &alf );

                gam = alf / sd;

                if( gam LT 0.0 OR gam GT 1.0 )
                    NL_OUT;

                c[0] = 1.0 - gam;
                s = 100.0 *gam;
                k = (NL_INTEGER)s;
                alf = s - floor( s );
                uh[j] = (1.0 - alf) * G[k] + alf * G[k + 1];

                newton = N_PowerBasisRootNewton( c, 3, uh[j], 0.0, 1.0, EP, &uh[j] );

                if( newton EQ NL_YES )
                    NL_OUT;

                /* Get alfas and betas */

                fact = 3.0 *uh[j] * (1.0 - uh[j]);
                dena = fact * (1.0 - uh[j]);
                denb = fact * uh[j];

                if( N_FloatOpIsBad( aa, dena, NL_DIVISION ) )
                    NL_OUT;

                if( N_FloatOpIsBad( bb, denb, NL_DIVISION ) )
                    NL_OUT;

                alfs[j] = aa / dena;
                bets[j] = -bb / denb;

                if( alfs[j]LT NL_MTOL OR bets[j]GT - NL_MTOL )
                    NL_OUT;

                salf += alfs[j];
                sbet += bets[j];
            }
            break;

        default:

            NL_ERROR( NL_NUM_ERR );
    } /* End of switch */

    /* Check approximation error */

    alf = salf / ((NL_REAL)ke - (NL_REAL)ks - (NL_REAL)1);
    bet = sbet / ((NL_REAL)ke - (NL_REAL)ks - (NL_REAL)1);

    N_VectorPtAlongVector( P[ks], alf, T[ks], &P1 );
    N_VectorPtAlongVector( P[ke], bet, T[ke], &P2 );

    /* Check parametric error first */

    if( ke - ks GT 2 )
    {
        apr = NL_TRUE;

        for ( i = ks + 1; i < ke; i++ )
        {
            j = i - ks;
            fact = 3.0 *uh[j] * (1.0 - uh[j]);
            aa = fact * (1.0 - uh[j]) * (alfs[j] - alf);
            bb = -fact * uh[j] * (bets[j] - bet);

            N_VectorCombine( aa, T[ks], bb, T[ke], &V );
            N_VectorMagnitude( V, &d );

            if( d GT E )
            {
                apr = NL_FALSE;
                break;
            }
        }

        if( apr EQ NL_TRUE )
        {
            N_PtToCPt( P[ks], &Pw[0] );
            N_PtToCPt( P1, &Pw[1] );
            N_PtToCPt( P2, &Pw[2] );
            N_PtToCPt( P[ke], &Pw[3] );

            *n = 3;
            NL_OUT;
        }
    }

    /* Check perpendicular error */

    N_PtToCPt( P[ks], &Qw[0] );
    N_PtToCPt( P1, &Qw[1] );
    N_PtToCPt( P2, &Qw[2] );
    N_PtToCPt( P[ke], &Qw[3] );

    U[0] = 0.0;
    U[1] = 0.0;
    U[2] = 0.0;
    U[3] = 0.0;
    U[4] = 1.0;
    U[5] = 1.0;
    U[6] = 1.0;
    U[7] = 1.0;

    error = N_CrvFromCPtsAndKnots( &cur, Qw, 3, 3, U, 7, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = ks + 1; i < ke; i++ )
    {
        j = i - ks;

        newton = N_CrvClosestPt( &cur, P[i], uh[j], top, toc, &aa, &R );

        if( newton EQ NL_YES )
            NL_OUT;

        N_DistPtPt( P[i], R, &d );

        if( d GT E )
            NL_OUT;
    }

    for ( i = 0; i <= 3; i++ )
        N_CopyCPt( Qw[i], &Pw[i] );

    *n = 3;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitLineToPts: Best fitting line segment to a set of random points      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a best  fitting line segment to a set
     of 2-D or  3-D points. First an infinite line is fit, which is then
     cut back so that the projections of all points lie on the segmented
     line. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   n;
       NL_REAL    era, erm;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get P);
       ...
       N_CrvInitArrays(&cur);
       N_FitLineToPts(P,n,&cur,&era,&erm,&SG);


   ACCESS:
   
     P     , input  ,  Points 
     n     , input  ,  Highest index in P
     cur   , output ,  Best fitting NURBS line
     era   , output ,  Average absolute error
     erm   , output ,  Maximum absolute error
     SG    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitLineToPts( NL_POINT *P, NL_INDEX n, NL_CURVE *cur, NL_REAL *era, NL_REAL *erm, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitLineToPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, imax = 0, imin = 0;

    NL_REAL tmax, tmin, t;

    NL_POINT A, B, C, D;

    NL_VECTOR V;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    if( n LT 1 )
        NL_ERROR( NL_INP_ERR );

    /* Fit an infinite line first */

    error = N_LineFitPts( P, n, &C, &V, era, erm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now segment the line */

    N_VectorSum( C, V, &D );

    tmin = NL_BIGD;
    tmax = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        error = N_ProjectPtLineParam( P[i], C, D, &A, &t );

        if( error EQ NL_YES )
            NL_OUT;

        if( t LT tmin )
        {
            tmin = t;
            imin = i;
        }

        if( t GT tmax )
        {
            tmax = t;
            imax = i;
        }
    }

    error = N_ProjectPtLineParam( P[imin], C, D, &A, &t );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_ProjectPtLineParam( P[imax], C, D, &B, &t );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create NURBS line */

    N_VectorDiff( B, A, &V );

    error = N_CrvLineFromPtAndVector( A, V, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitConicToPts: Check scatter and compute local conic approximant        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a  best fitting NURBS conic arc to a
     set of  points and checks if  the error of approximation is within 
     a given tolerance. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   ks, ke, n;
       NL_VECTOR  Ts, Te;
       NL_REAL    E, top, toc;
       NL_CPOINT  Pw[5];
       ...
       (get array P, ks, ke, Ts, Te, E, top, toc);
       ...
       N_FitConicToPts(P,ks,ke,Ts,Te,E,top,toc,Pw,&n);

     This  routine  is  the  working  bee  used by  N_FITCRVCONICSAPPROX  to  fit a 
     piecewise quadratic curve to data.


   ACCESS:
   
     P     , input  ,  Points to be approximated
     ks,ke , input  ,  Start   and   end   indexes  of   points  to  be 
                       approximated
     Ts,Te , input  ,  Unit tangents at P[ks] and P[ke]
     E     , input  ,  Error tolerance
     top   , input  ,  Point coincidence tolerance for Newton (relative
                       tolerance)
     toc   , input  ,  Cosine tolerance for Newton
     Pw    , output ,  Control points of approximating curve
     n     , output ,  Highest index in Pw


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitConicToPts( NL_POINT *P, NL_INDEX ks, NL_INDEX ke, NL_VECTOR Ts, NL_VECTOR Te, NL_REAL E, NL_REAL top, NL_REAL toc, NL_CPOINT *Pw, NL_INDEX *n )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitConicToPts");

    NL_FLAG its, itype, ftype, prj, newton = NL_NO, error = NL_NO;

    NL_INDEX i, j, k;

    NL_REAL U[6], d, alf, oma, a, b, c, coss, cose, gams, game, w, cw, s, num, den;

    NL_PARAMETER *u;

    NL_POINT R, Rs, Re, M;

    NL_CPOINT Qw[3];

    NL_VECTOR S, A, B, C;

    NL_LINESEG ls, le, lc, li;

    NL_CURVE cur;

    NL_STACKS SL;

    NL_REAL ALFA = 0.66;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize certain entities */

    if( ks GE ke )
        NL_ERROR( NL_INP_ERR );

    *n = 0;
    k = 0;
    itype = 0;
    ftype = 0;

    N_VectorDiff( P[ke], P[ks], &S );

    error = N_VectorNormalizeRef( &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_Pts1dGetMaxExtent( &P[ks], ke - ks, &d );

    top *= d;

    /* Get different cases */

    N_CreateLineStartDirVector( &ls, P[ks], Ts, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &le, P[ke], Te, NL_UNBOUNDED );

    error = N_IsectLineLine( ls, le, &R, &a, &b, &its );

    if( error EQ NL_YES )
        NL_OUT;

    if( ke - ks EQ 1 )
        ftype = 1;

    else /* Interpolation     */
    if( its EQ NL_FALSE )
        ftype = 2;

    else           /* Parallel tangents */
        ftype = 3; /* Approximation     */

    /* Switch to the different cases */

    switch( ftype )
    {
        case 1: /* Interpolatory curve required */

            N_Weight( P[ks], 1.0, &Pw[0] );

            if( its EQ NL_TRUE )
            {
                if( a GT 0.0 AND b LT 0.0 )
                    itype = 1;
                else           /* One arc   */
                    itype = 2; /* Split arc */
            }
            else
            {
                N_VectorDot( Ts, Te, &a );
                N_VectorCross( Ts, S, &A );
                N_VectorMagnitude( A, &b );

                if( b GT NL_LTOL )
                {
                    if( a GT 0.0 )
                        itype = 2;
                    else           /* Parallel tangents     */
                        itype = 3; /* Antiparallel tangents */
                }
                else
                {
                    itype = 4; /* Collinear tangents */
                }
            }

            /* Switch to different interpolation cases */

            switch( itype )
            {
                case 1: /* Single arc */

                    N_BezSetConicWeight( P[ks], R, P[ke], &w );

                    N_Weight( R, w, &Pw[++k] );
                    N_Weight( P[ke], 1.0, &Pw[++k] );
                    break;

                case 2: /* Split segment */

                    N_DistPtPt( P[ks], P[ke], &d );

                    N_VectorDot( Ts, S, &coss );
                    N_VectorDot( Te, S, &cose );

                    gams = 0.25 *d / (1.0 + ALFA * fabs( cose ) + (1.0 - ALFA) * fabs( coss ));
                    game = 0.25 *d / (1.0 + ALFA * fabs( coss ) + (1.0 - ALFA) * fabs( cose ));
                    alf = gams / (gams + game);
                    oma = 1.0 - alf;

                    N_VectorPtAlongVector( P[ks], gams, Ts, &Rs );
                    N_VectorPtAlongVector( P[ke], -game, Te, &Re );
                    N_Combine2Pts( oma, Rs, alf, Re, &M );

                    N_BezSetConicWeight( P[ks], Rs, M, &w );

                    N_Weight( Rs, w, &Pw[++k] );
                    N_Weight( M, 1.0, &Pw[++k] );

                    N_BezSetConicWeight( M, Re, P[ke], &w );

                    N_Weight( Re, w, &Pw[++k] );
                    N_Weight( P[ke], 1.0, &Pw[++k] );
                    break;

                case 3: /* Antiparallel tangents */

                    cw = 0.5 *sqrt( 2.0 );

                    N_Combine2Pts( 0.5, P[ks], 0.5, P[ke], &M );
                    N_DistPtPt( P[ks], M, &alf );

                    N_VectorPtAlongVector( P[ks], alf, Ts, &Rs );
                    N_VectorPtAlongVector( M, alf, Ts, &M );
                    N_VectorPtAlongVector( P[ke], alf, Ts, &Re );

                    N_Weight( Rs, cw, &Pw[++k] );
                    N_Weight( M, 1.0, &Pw[++k] );
                    N_Weight( Re, cw, &Pw[++k] );
                    N_Weight( P[ke], 1.0, &Pw[++k] );
                    break;

                case 4: /* Collinear tangents */

                    N_Combine2Pts( 0.5, P[ks], 0.5, P[ke], &R );

                    N_Weight( R, 1.0, &Pw[++k] );
                    N_Weight( P[ke], 1.0, &Pw[++k] );
                    break;

                default:

                    NL_ERROR( NL_NUM_ERR );
            } /* End of switch for interpolation */
            break;

        case 2: /* Parallel tangents: 
           check scatter */

            N_Weight( P[ks], 1.0, &Pw[0] );
            N_VectorCross( Ts, S, &A );
            N_VectorMagnitude( A, &a );

            if( a GT NL_LTOL )
                NL_OUT;

            N_CreateLinePtPt( &lc, P[ks], P[ke], NL_BOUNDED );

            for ( i = ks + 1; i < ke; i++ )
            {
                error = N_ProjectPtLine( lc, P[i], &R, &a, &prj );

                if( error EQ NL_YES OR prj EQ NL_FALSE )
                    NL_OUT;

                N_DistPtPt( P[i], R, &d );

                if( d GT E )
                    NL_OUT;
            }

            N_Combine2Pts( 0.5, P[ks], 0.5, P[ke], &R );
            N_Weight( R, 1.0, &Pw[++k] );
            N_Weight( P[ke], 1.0, &Pw[++k] );
            break;

        case 3: /* Fit conic arc to data */

            u = N_AllocReal1dArray( ke - ks, &SL );

            if( u EQ NULL )
                NL_QUIT;

            N_CreateLinePtPt( &lc, P[ks], P[ke], NL_BOUNDED );

            s = 0.0;

            for ( i = ks + 1; i < ke; i++ )
            {
                j = i - ks;

                N_CreateLinePtPt( &li, R, P[i], NL_UNBOUNDED );

                error = N_IsectLineLine( lc, li, &M, &alf, &oma, &its );

                if( error EQ NL_YES OR its EQ NL_FALSE )
                    NL_OUT;

                if( N_FloatOpIsBad( alf, 1.0 - alf, NL_DIVISION ) )
                    NL_OUT;

                a = sqrt( alf / (1.0 - alf) );
                u[j] = a / (1.0 + a);

                N_VectorDiff( P[i], P[ks], &A );
                N_VectorDiff( P[i], P[ke], &B );
                N_VectorDiff( R, P[i], &C );

                N_VectorDot( A, C, &a );
                N_VectorDot( B, C, &b );
                N_VectorDot( C, C, &c );

                num = (1.0 - u[j]) * (1.0 - u[j]) * a + u[j] * u[j] * b;
                den = 2.0 *u[j] * (1.0 - u[j]) * c;

                if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                    NL_OUT;

                w = num / den;

                if( w LT 0.0 )
                    NL_OUT;
                s += (w / (1.0 + w));
            }

            s = s / ((NL_REAL)ke - (NL_REAL)ks - (NL_REAL)1);

            if( N_FloatOpIsBad( s, 1.0 - s, NL_DIVISION ) )
                NL_OUT;
            w = s / (1.0 - s);

            if( w LT NL_WMIN OR w GT NL_WMAX )
                NL_OUT;

            N_Weight( P[ks], 1.0, &Qw[0] );
            N_Weight( R, w, &Qw[1] );
            N_Weight( P[ke], 1.0, &Qw[2] );

            U[0] = 0.0;
            U[1] = 0.0;
            U[2] = 0.0;
            U[3] = 1.0;
            U[4] = 1.0;
            U[5] = 1.0;

            error = N_CrvFromCPtsAndKnots( &cur, Qw, 2, 2, U, 5, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = ks + 1; i < ke; i++ )
            {
                j = i - ks;

                newton = N_CrvClosestPt( &cur, P[i], u[j], top, toc, &a, &R );

                if( newton EQ NL_YES )
                    NL_OUT;

                N_DistPtPt( P[i], R, &d );

                if( d GT E )
                    NL_OUT;
            }

            k = 2;

            for ( i = 0; i <= 2; i++ )
            {
                N_CopyCPt( Qw[i], &Pw[i] );
            }
            break;

        default:

            NL_ERROR( NL_NUM_ERR );
    } /* End of switch for approximation */

    *n = k;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITCRVPARABARCS: Curve interpolation with piecewise parabolic arcs        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes  a NURBS piecewise  parabolic curve  
     interpolating a given set of points. The parametrization is either
     NL_C1 or NL_G1. If the output  curve is  initialized to  the NULL curve, 
     memory  is allocated  locally. Otherwise  it is  checked if enough 
     memory is passed in. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   k;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get array P);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvParabArcs(P,k,NL_CHORDLENGTH,NL_AKIMA,NL_G1,NL_CUSP,&cur,&SG);

     A recommended default for the parametrization type is NL_CHORDLENGTH.


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k    , input  ,  Highest index in P
     par  , input  ,  Flag:
                        NL_UNIFORM    : Uniform parametrization
                        NL_CHORDLENGTH: Chord length parametrization 
                        NL_CENTRIPETAL: Centripetal parametrization 
     tan  , input  ,  Flag:
                        NL_BESSEL: Tangents computed by Bessel's method
                        NL_AKIMA : Tangents computed by Akima's method
     cnt  , input  ,  Flag:
                        NL_C1: NL_C1 parametrization
                        NL_G1: NL_G1 parametrization
     csp  , input  ,  Flag:
                        NL_CUSP  : Maintain cusps/corners 
                        NL_NOCUSP: Smooth out corners
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvParabArcs( NL_POINT *PP, NL_INDEX k, NL_FLAG par, NL_FLAG tan, NL_FLAG cnt, NL_FLAG csp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvParabArcs");

    NL_FLAG closed, its, type, reset = NL_NO, error = NL_NO;

    NL_INDEX i, j, l, n, m;

    NL_REAL *U, d, dui, dui1, alf, oma, a, b, cosi, cosi1, gam, gam1;

    NL_PARAMETER *u;

    NL_POINT *Q, R, M, Ri, Ri1, *P;

    NL_CPOINT *Pw;

    NL_VECTOR *S, *SU, *T, A;

    NL_LINESEG li1, li;

    NL_STACKS SL;

    NL_REAL ALFA = 0.66;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for too few input points */

    if( k LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Make local copy of points, as some may be discarded */

    P = N_AllocPt1dArray( k, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
        N_CopyPt( PP[i], &P[i] );

    /* Check for curve memory */

    if( N_CrvAreArraysNULL( cur ) )
        reset = NL_YES;

    n = 4 * k;
    error = N_CrvSizeArrays( cur, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Allocate memory */

    u = N_AllocReal1dArray( 2 * k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( 2 * k, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( k + 3, &SL );

    if( S EQ NULL )
        NL_QUIT;

    SU = N_AllocPt1dArray( k + 3, &SL );

    if( SU EQ NULL )
        NL_QUIT;

    T = N_AllocPt1dArray( k, &SL );

    if( T EQ NULL )
        NL_QUIT;

    /* Compute the chords and normalize them */

    N_DistPtPt( P[0], P[k], &d );

    if( d LT NL_MTOL )
        closed = NL_YES;
    else
        closed = NL_NO;

    for ( i = 1; i <= k; i++ )
        N_VectorDiff( P[i], P[i - 1], &S[i + 1] );

    if( closed EQ NL_YES )
    {
        N_VectorCopy( S[k + 1], &S[1] );
        N_VectorCopy( S[k], &S[0] );
        N_VectorCopy( S[2], &S[k + 2] );
        N_VectorCopy( S[3], &S[k + 3] );
    }
    else
    {
        N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
        N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
        N_VectorCombine( 2.0, S[k + 1], -1.0, S[k], &S[k + 2] );
        N_VectorCombine( 2.0, S[k + 2], -1.0, S[k + 1], &S[k + 3] );
    }

    for ( i = 2; i <= k + 1; i++ )
    {
        error = N_VectorNormalize( S[i], &SU[i], &a );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute the unit tangents */

    switch( tan )
    {
        case NL_BESSEL:

            error = N_FitCalcCrvParamValues( (NL_VOID *)P, k, NL_EPOINT, par, u );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 1; i < k; i++ )
            {
                dui = u[i] - u[i - 1];
                dui1 = u[i + 1] - u[i];

                if( N_FloatOpIsBad( dui, dui + dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = dui / (dui + dui1);
                oma = 1.0 - alf;

                if( N_FloatOpIsBad( oma, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( alf, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = oma / dui;
                alf = alf / dui1;

                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }

            dui = u[1] - u[0];
            dui1 = u[k] - u[k - 1];

            if( closed EQ NL_YES )
            {
                if( N_FloatOpIsBad( 1.0, 2.0 *dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 1.0, 2.0 *dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 1.0 / (2.0 *dui);
                alf = 1.0 / (2.0 *dui1);

                N_VectorCombine( oma, S[2], alf, S[k + 1], &T[0] );
                N_VectorCopy( T[0], &T[k] );
            }
            else
            {
                if( N_FloatOpIsBad( 2.0, dui, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( N_FloatOpIsBad( 2.0, dui1, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                oma = 2.0 / dui;
                alf = 2.0 / dui1;

                N_VectorCombine( oma, S[2], -1.0, T[1], &T[0] );
                N_VectorCombine( alf, S[k + 1], -1.0, T[k - 1], &T[k] );
            }
            break;

        case NL_AKIMA:
            for ( i = 0; i <= k; i++ )
            {
                N_VectorCross( S[i], S[i + 1], &A );
                N_VectorMagnitude( A, &a );
                N_VectorCross( S[i + 2], S[i + 3], &A );
                N_VectorMagnitude( A, &b );

                if( (a + b)LT NL_LTOL )
                {
                    if( csp EQ NL_CUSP )
                        alf = 1.0;
                    else
                        alf = 0.5;
                }
                else
                {
                    alf = a / (a + b);
                }

                oma = 1.0 - alf;
                N_VectorCombine( oma, S[i + 1], alf, S[i + 2], &T[i] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= k; i++ )
    {
        error = N_VectorNormalizeRef( &T[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Interpolate segment by segment */

    n = 0;
    l = 0;

    N_PtToCPt( P[0], &Pw[0] );
    N_CopyPt( P[0], &Q[0] );

    for ( i = 1; i <= k; i++ )
    {
        /* Establish the various cases */

        N_CreateLineStartDirVector( &li1, P[i - 1], T[i - 1], NL_UNBOUNDED );
        N_CreateLineStartDirVector( &li, P[i], T[i], NL_UNBOUNDED );

        error = N_IsectLineLine( li1, li, &R, &a, &b, &its );

        if( error EQ NL_YES )
            NL_OUT;

        if( its EQ NL_TRUE ) /* Lines intersect */
        {
            if( a GT 0.0 AND b LT 0.0 )
            {
                type = 1; /* One arc segment */
            }
            else if( fabs( a )LT NL_PTOL OR fabs( b )LT NL_PTOL )  /* gwc: was NL_LTOL - but this is not an aangle - maybe NL_PTOL is not right */
            {
                if( csp EQ NL_CUSP )
                    type = 2;
                else          /* Corner needed */
                    type = 3; /* Split arc     */
            }
            else
            {
                type = 3; /* Split arc */
            }
        }
        else /* Lines do not intersect */
        {
            N_VectorDot( T[i - 1], T[i], &a );
            N_VectorCross( T[i - 1], SU[i + 1], &A );
            N_VectorMagnitude( A, &b );

            if( b GT NL_LTOL )
            {
                if( a GT 0.0 )
                    type = 3;
                else          /* Parallel tangents     */
                    type = 4; /* Antiparallel tangents */
            }
            else              /* Collinear points */
            {
                if( i EQ k )
                {
                    type = 6; /* No points discarded */
                }
                else
                {
                    N_VectorCross( T[i], T[i + 1], &A );
                    N_VectorMagnitude( A, &b );

                    if( b GT NL_LTOL )
                    {
                        type = 6; /* No points discarded */
                    }
                    else
                    {
                        N_VectorCross( SU[i + 1], SU[i + 2], &A );
                        N_VectorMagnitude( A, &b );

                        if( b LT NL_LTOL )
                            type = 5;
                        else          /* Points discarded    */
                            type = 6; /* No points discarded */
                    }
                }
            }
        }

        /* Switch to different types of segments */

        switch( type )
        {
            case 1: /* Single arc */

                N_PtToCPt( R, &Pw[++n] );
                N_PtToCPt( P[i], &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 2: /* Corner needed */

                N_Combine2Pts( 0.5, P[i - 1], 0.5, P[i], &R );
                N_PtToCPt( R, &Pw[++n] );
                N_PtToCPt( P[i], &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 3: /* Split segment */

                N_DistPtPt( P[i - 1], P[i], &d );

                N_VectorDot( T[i - 1], SU[i + 1], &cosi1 );
                N_VectorDot( T[i], SU[i + 1], &cosi );

                gam1 = 0.125 *d / (ALFA * fabs( cosi ) + (1.0 - ALFA) * fabs( cosi1 ));
                gam = 0.125 *d / (ALFA * fabs( cosi1 ) + (1.0 - ALFA) * fabs( cosi ));
                alf = gam1 / (gam + gam1);
                oma = 1.0 - alf;

                N_VectorPtAlongVector( P[i - 1], gam1, T[i - 1], &Ri1 );
                N_VectorPtAlongVector( P[i], -gam, T[i], &Ri );
                N_Combine2Pts( oma, Ri1, alf, Ri, &M );

                N_PtToCPt( Ri1, &Pw[++n] );
                N_PtToCPt( M, &Pw[++n] );
                N_CopyPt( M, &Q[++l] );

                N_PtToCPt( Ri, &Pw[++n] );
                N_PtToCPt( P[i], &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 4: /* Antiparallel tangents */

                N_Combine2Pts( 0.5, P[i - 1], 0.5, P[i], &M );
                N_DistPtPt( P[i - 1], M, &gam );

                N_VectorPtAlongVector( P[i - 1], gam, T[i - 1], &Ri1 );
                N_VectorPtAlongVector( M, gam, T[i - 1], &M );
                N_VectorPtAlongVector( P[i], gam, T[i - 1], &Ri );

                N_PtToCPt( Ri1, &Pw[++n] );
                N_PtToCPt( M, &Pw[++n] );
                N_CopyPt( M, &Q[++l] );

                N_PtToCPt( Ri, &Pw[++n] );
                N_PtToCPt( P[i], &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            case 5: /* Discard point - shift down data */
                for ( j = i; j < k; j++ )
                {
                    N_CopyPt( P[j + 1], &P[j] );
                    N_VectorCopy( T[j + 1], &T[j] );
                }

                for ( j = i; j <= k + 1; j++ )
                    N_VectorCopy( SU[j + 2], &SU[j + 1] );

                k--;
                i--;
                break;

            case 6: /* Collinear data - output line segment */

                N_Combine2Pts( 0.5, P[i - 1], 0.5, P[i], &R );
                N_PtToCPt( R, &Pw[++n] );
                N_PtToCPt( P[i], &Pw[++n] );
                N_CopyPt( P[i], &Q[++l] );
                break;

            default:

                NL_ERROR( NL_NUM_ERR );
        } /* End of switch */
    }     /* End of loop */

    /* Compute knot vector */

    if( csp EQ NL_CUSP )
        cnt = NL_G1;

    m = 2;

    for ( i = 0; i <= 2; i++ )
        U[i] = 0.0;

    switch( cnt )
    {
        case NL_G1: /* Geometric continuity required */

            error = N_FitCalcCrvParamValues( (NL_VOID *)Q, l, NL_EPOINT, par, u );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 1; i < l; i++ )
            {
                U[++m] = u[i];
                U[++m] = u[i];
            }
            break;

        case NL_C1: /* Parametric continuity required */

            u[0] = 0.0;
            u[1] = 1.0;

            for ( i = 2; i <= l; i++ )
            {
                j = 2 * (i - 1);

                N_DistCptCpt( Pw[j + 1], Pw[j], &a );
                N_DistCptCpt( Pw[j], Pw[j - 1], &b );

                u[i] = u[i - 1] + (a * (u[i - 1] - u[i - 2])) / b;
            }

            for ( i = 1; i < l; i++ )
                U[++m] = u[i] / u[l];

            n = 1;

            for ( i = 2; i <= l; i++ )
            {
                j = 2 * i - 1;
                N_CopyCPt( Pw[j], &Pw[++n] );
            }
            N_PtToCPt( P[k], &Pw[++n] );
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= 2; i++ )
        U[++m] = 1.0;

    /* Set curve indexes and compact curve */

    N_CrvSetSizeIndices( cur, n, 2, m );

    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( cur, SG );

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
/* N_FITCRVLSTSQENDS: Least Squares Curve Fit to Points, with End Conditions   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine  computes a least-squares NURBS curve approximation to 
     a  given set of  points. The degree is arbitrary.
     End Conditions are optional (not NULL), and can be given as TANGENTs
     or DERIVATIVEs. The knot vector (which therefore defines the output number
     of spans) and degree are given.
     If  the output  curve is  initialized to the NULL curve, memory is 
     allocated  locally. Otherwise  it is checked  if enough  memory is 
     passed in. 
     
     This routine uses N_FitCrvWeightedLstSqKnots().  
     A typical calling example is:

       NL_POINT       *P;
       NL_INDEX       k;
       NL_VECTOR      Ts, Te;
       NL_FLAG         der;
       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvLstSqEnds(P, k, &Ts, &Te, NL_TANGENT, &knt, p, &cur, &SG);

   ACCESS:
   
     P   , input  ,  3D Euclidean NL_POINT data
     k   , input  ,  Highest index in P
     Ts  , input  ,  Start tangent/derivative
                       !NULL: interpolate with start tangent
                        NULL: interpolate without start tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                        ITS MAGNITUDE WILL BE RESCALED!
     Te  , input ,  End tangent/derivative
                       !NULL: interpolate with end tangent
                        NULL: interpolate without end tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                        ITS MAGNITUDE WILL BE RESCALED!
     der , input  ,  Flag:
                       NL_TANGENT   : Ts and/or Te are  tangent directions
                                   only and must be scaled internally
                       NL_DERIVATIVE: Ts and/or Te are derivatives and are
                                   used as passed in
     knt , input  ,  Knot vector of approximating curve
     p   , input  ,  Degree of approximating curve 
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvLstSqEnds( NL_POINT *P, NL_INDEX k, NL_VECTOR *Ts, NL_VECTOR *Te, NL_FLAG der, NL_KNOTVECTOR *knt, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_STACKS SL;
    NL_PARAMETER *U, *u, *wp, wd[2];
    NL_INDEX mt;
    NL_FLAG error, tan = NL_NO;
    NL_INDEX i, s = 0, nCP, I[2];
    NL_VECTOR *pF = NULL;
    NL_VECTOR F[2];

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Compute parameter values for each P in u */
    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;
    error = N_FitCalcCrvParamValues( (void *)P, k, NL_EPOINT, NL_CHORDLENGTH, u );

    if( error EQ NL_YES )
        NL_OUT;

    /* reset domain of u to correspond with knt */
    N_KnotVectorGetKnots( knt, &mt, &U );

    if( u[0]NEQ U[0]OR u[k]NEQ U[mt] )
    {
        NL_REAL fac;
        fac = (U[mt] - U[0]) / (u[k] - u[0]);

        for ( i = 1; i < k; i++ )
        {
            u[i] = U[0] + fac * (u[i] - u[0]);
        }

        u[0] = U[0];
        u[k] = U[mt];
    }

    /* Check tangency conditions */
    if( Ts EQ NULL AND Te EQ NULL )
    {
        tan = NL_NO;
    }
    else if( Ts NEQ NULL AND Te EQ NULL )
    {
        tan = NL_START;
    }
    else if( Ts EQ NULL AND Te NEQ NULL )
    {
        tan = NL_END;
    }
    else if( Ts NEQ NULL AND Te NEQ NULL )
    {
        tan = NL_BOTH;
    }

    /* Convert tangent magnitudes to derivatives */
    if( tan NEQ NL_NO AND der EQ NL_TANGENT )
    {
        NL_REAL fac;
        N_DistPolygon( P, k, &fac );
        fac = fac * (u[k] - u[0]);

        if( Ts NEQ NULL )
        {
            error = N_VectorNormalizeRef( Ts );

            if( error EQ NL_YES )
                NL_OUT;
            N_VectorScale( *Ts, fac, Ts );
        }

        if( Te NEQ NULL )
        {
            error = N_VectorNormalizeRef( Te );

            if( error EQ NL_YES )
                NL_OUT;
            N_VectorScale( *Te, fac, Te );
        }
    }

    /* Set up arguments for interpolating start,end points and start,end derivatives */
    wp = N_AllocReal1dArray( k, &SL );

    for ( i = 1; i < k; i++ )
        wp[i] = 1.0;
    wp[0] = -1.0;
    wp[k] = -1.0;
    I[0] = 0;
    I[1] = k;
    wd[0] = -1.0;
    wd[1] = -1.0;

    if( tan EQ NL_NO )
    {
        pF = NULL;
        s = -1;
    }

    if( tan EQ NL_START )
    {
        pF = Ts;
        s = 0;
    }

    if( tan EQ NL_END )
    {
        pF = Te;
        s = 0;
        I[0] = k;
    }

    if( tan EQ NL_BOTH )
    {
        F[0] = *Ts;
        F[1] = *Te;
        pF = F;
        s = 1;
    }

    nCP = mt - p - 1;
    error = N_FitCrvWeightedLstSqKnots( P, wp, k, pF, wd, I, s, NL_EPOINT, u, knt, nCP, p, cur, SG );

    /* End NURBS and Exit */
    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FitCrvPtsNormals: Curve interpolation from points and normals              */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve  interpolating a given
     set  of points  and normals  at  those points. It  is a 
     general  routine  requiring  parameters  where  data   points  are 
     assumed, and  NL_POINT point data and NL_VECTOR normal data. The degree  
     must be  2 or 3.  If the output curve is initialized to the NULL 
     curve, memory is allocated locally. Otherwise  it is checked if 
     enough memory is passed in.
     A typical calling example is:

       NL_POINT       *pts;
       NL_VECTOR      *nrm;
       NL_INDEX       kk;
       NL_DEGREE       p;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
(get points, normals, and choose the degree);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvPtsNormals(pts , nrm , kk, p, &cur, &SG);

   ACCESS:
   
     pts  , input  ,  NL_POINT  data as interpolation points
     nrm  , input  ,  NL_VECTOR as normal at interpolation points
     kk   , input  ,  Highest index in pts and nrm
     p    , input  ,  Degree, 2 or 3
     cur  , output ,  Interpolating curve
     SG   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCrvPtsNormals( NL_POINT *pts, NL_VECTOR *nrm, NL_INDEX kk, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;
    NL_VECTOR *Tdir, M;
    NL_CURVE crv;
    NL_INDEX i;
    NL_POINT PntDer[2];
    NL_PARAMETER u0, u1, ui = 0.0;
    NL_REAL tol0 = 1.0E-4;
    NL_REAL tol1 = 1.0E-4;
    NL_STACKS SL;

    if( p < 2 || p > 3 )
        NL_OUT;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Build a curve interpolating the points */

    N_CrvInitArrays( &crv );
    error = N_FitCrvInterp( pts, kk, p, NL_CHORDLENGTH, &crv, &SL );

    if( error )
        NL_OUT;

    N_CrvGetParamBounds( &crv, &u0, &u1 );

    /* get tangent direction array, one for each point */
    Tdir = N_AllocPt1dArray( kk, &SL );

    /* For each point */

    for ( i = 0; i <= kk; i++ )
    {
        if( i EQ 0 )
            ui = u0; /* first point */

        else if( i EQ kk )
            ui = u1; /* last point */

        else
        {
            /* Obtain parameter value for this point */
            u0 = ui; /* starting u is previous ui */
            error = N_CrvClosestPt( &crv, pts[i], u0, tol0, tol1, &ui, &PntDer[0] );

            if( error )
                NL_OUT;
        }

        /* evaluate curve, getting point and one derivative */
        error = N_CrvDerivs( &crv, ui, NL_LEFT, 1, PntDer );

        if( error )
            NL_OUT;

        /* project curve derivative into plane giving tangent direction */
        N_VectorCross( nrm[i], PntDer[1], &M );
        N_VectorCross( M, nrm[i], &Tdir[i] );
    }

    /* Interpolate points and tangent direction */
    error = N_FitCrvFirstDeriv( pts, Tdir, kk, p, NL_TANGENT, NL_CENTRIPETAL, cur, SG );

    /* End NURBS and Exit */

    EXIT:
    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITCALCMATRIX: Compute interpolation matrix for curve fitting           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine computes the  LU-decomposed matrix necessary
     for curve interpolation with or without end tangent constraints. A  
     typical calling example is:

       NL_KNOTVECTOR  *knt;
       NL_DEGREE      p;
       NL_PARAMETER   *t;
       NL_INDEX       n;
       NL_RMATRIX     *cm;
       NL_STACKS      S;
       ...
       (get knt, p and t);
       ...
       N_FitCalcMatrix(knt,p,t,n,NL_START,cm,&S);

     MEMORY FOR cm IS ALLOCATED INSIDE THE ROUTINE!


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree
     t   , input  ,  Parameters
     n   , input  ,  Highest index in t
     itp , input  ,  Flag:
                       NL_NO   : no derivatives are specified
                       NL_START: derivative specified at the start 
                       NL_END  : derivative specified at the end
                       NL_BOTH : derivatives specified at start and end
     cm  , output ,  LU-decomposed matrix
     SM  , input  ,  cm's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcMatrix( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER *t, NL_INDEX n, NL_FLAG itp, NL_RMATRIX *cm, NL_STACKS *SM )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, bw, sbw, sp;

    NL_REAL ** A, *N;

    NL_STACKS SL;

    N_InitNurbs( &SL );

    /* Allocate memory */

    switch( itp )
    {
        case NL_NO:
            k = n;
            break;

        case NL_START:
            k = n + 1;
            break;

        case NL_END:
            k = n + 1;
            break;

        case NL_BOTH:
            k = n + 2;
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    if( 2 *p LE k )
    {
        bw = 2 * p + 1;
        sbw = p;
    }
    else
    {
        bw = 2 * p - 1;
        sbw = p - 1;
    }
    sp = sbw - p;

    error = N_SetRealMatrix( cm, k, k, NL_MT_BANDED, bw, SM );

    if( error EQ NL_YES )
        NL_OUT;

    N = N_AllocReal1dArray( p, &SL );

    if( N EQ NULL )
        NL_QUIT;

    N_GetRealMatrixPtr( cm, &A );

    /* Compute matrix entries */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    switch( itp )
    {
        case NL_NO:

            A[0][sbw] = 1.0;
            A[n][sbw] = 1.0;

            for ( i = 1; i < n; i++ )
            {
                error = N_BasisEval( knt, p, t[i], NL_LEFT, N, &j );

                if( error EQ NL_YES )
                    NL_OUT;

                l = sp - i + j;

                if( l LT 0 OR l + p GE bw )
                    NL_ERROR( NL_SEQ_ERR );

                for ( k = 0; k <= p; k++ )
                    A[i][l + k] = N[k];
            }
            break;

        case NL_START:

            A[0][sbw] = 1.0;
            A[1][sbw - 1] = -1.0;
            A[1][sbw] = 1.0;
            A[n + 1][sbw] = 1.0;

            for ( i = 2; i <= n; i++ )
            {
                error = N_BasisEval( knt, p, t[i - 1], NL_LEFT, N, &j );

                if( error EQ NL_YES )
                    NL_OUT;

                l = sp - i + j;

                if( l LT 0 OR l + p GE bw )
                    NL_ERROR( NL_SEQ_ERR );

                for ( k = 0; k <= p; k++ )
                    A[i][l + k] = N[k];
            }
            break;

        case NL_END:

            A[0][sbw] = 1.0;
            A[n][sbw] = -1.0;
            A[n][sbw + 1] = 1.0;
            A[n + 1][sbw] = 1.0;

            for ( i = 1; i <= n - 1; i++ )
            {
                error = N_BasisEval( knt, p, t[i], NL_LEFT, N, &j );

                if( error EQ NL_YES )
                    NL_OUT;

                l = sp - i + j;

                if( l LT 0 OR l + p GE bw )
                    NL_ERROR( NL_SEQ_ERR );

                for ( k = 0; k <= p; k++ )
                    A[i][l + k] = N[k];
            }
            break;

        case NL_BOTH:

            A[0][sbw] = 1.0;
            A[1][sbw - 1] = -1.0;
            A[1][sbw] = 1.0;
            A[n + 1][sbw] = -1.0;
            A[n + 1][sbw + 1] = 1.0;
            A[n + 2][sbw] = 1.0;

            for ( i = 2; i <= n; i++ )
            {
                error = N_BasisEval( knt, p, t[i - 1], NL_LEFT, N, &j );

                if( error EQ NL_YES )
                    NL_OUT;

                l = sp - i + j;

                if( l LT 0 OR l + p GE bw )
                    NL_ERROR( NL_SEQ_ERR );

                for ( k = 0; k <= p; k++ )
                    A[i][l + k] = N[k];
            }
    }

    /* LU decompose matrix */

    error = N_RealMatrixLuDecompose( cm );

    if( error EQ NL_YES )
        NL_OUT;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FitSmoothPts: Smooth 3D points in preparation for curve fitting        */
/**********************************************************************/

/*******************************************************************//**
   DESCRIPTION:

     This routine smoothes a sequence of 3D points.  It  can  be used to
     smooth a set of points prior to curve fitting.  It  sweeps  through
     the points, from left to right, fitting curve segments through sub-
     sets of the points,  then projecting the points to these curve seg-
     ments. A typical calling example is:

       NL_POINT    *P, *Q;
       NL_INDEX    np, nc, nn;
       NL_DEGREE   p;
       ...
       (allocate P and Q, load P, and set np,p,nc,nn);
       ...
       N_FitSmoothPts(P,np,p,nc,nn,NL_NO,P);

   ACCESS:

     P    , input  ,  The points to be smoothed
     np   , input  ,  High index of points in P.  The  following is  re-
                      quired:
                        np >= nn    if eflg = NL_NO
                        np >= nn+2  if eflg = NL_YES
     p    , input  ,  Degree of the curve segments used  for  smoothing.
                      p > 1 is required; p = 2 or p = 3 is recommended
     nc   , input  ,  High index of control points for  the  curve  seg-
                      ments. nc >= p is required;  nc = p is recommended
     nn   , input  ,  High index of data points to be used to  fit  each
                      segment. nn > nc is required. nn is data dependent
                      and therefore difficult to set. The larger nn, the
                      more smoothing is done; nn should increase with np
                      nn=0.2*np  is  a  reasonable  choice for small np,  
                      nn=0.1*np  or larger is appropriate for larger np
     eflg , input  ,  Flag for end constraints:
                       = NL_YES : constrain end points (they aren't smooth-
                               ed)
                       = NL_NO  : smooth all points, including end points
     Q    , output ,  Smoothed points.  This memory must be allocated in
                      the calling routine; it may be the same memory lo-
                      cation as P

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_FitSmoothPts( NL_POINT *P, NL_INDEX np, NL_DEGREE p, NL_INDEX nc, NL_INDEX nn, NL_FLAG eflg, NL_POINT *Q )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSmoothPts");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, r, s, *I, n, *idx, k1, k2;

    NL_REAL dd, *wp, *wd, top, toc, *uu, *U;

    NL_CURVE curv;

    NL_POINT QQ, *pnts, *D, *PP;

    NL_KNOTVECTOR knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors in input */

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( nc LT p )
        NL_ERROR( NL_INP_ERR );

    if( nn LE nc )
        NL_ERROR( NL_INP_ERR );

    if( (eflg EQ NL_YES AND np LT nn + 2)OR( eflg EQ NL_NO AND np LT nn ) )
        NL_ERROR( NL_INP_ERR );

    /* Allocate some memory and initialize some variables */

    top = NL_MTOL;
    toc = 1.e-5;

    D = NULL;
    wd = NULL;
    I = NULL;
    s = -1; /* no derivatives */

    r = nc + p + 1;

    wp = N_AllocReal1dArray( 2 * (nn + 3) + r + 2, &SL );

    if( wp EQ NULL )
        NL_QUIT;

    uu = &wp[nn + 3];
    U = &uu[nn + 3];

    for ( ii = 0; ii <= nn + 2; ii++ )
        wp[ii] = 1.0;

    for ( ii = 0; ii <= p; ii++ )
    {
        U[ii] = 0.0;
        U[r - ii] = 1.0;
    }

    n = r - 2 * p - 1;
    dd = 1.0 / ((NL_REAL)n + (NL_REAL)1);

    for ( ii = 1; ii <= n; ii++ )
        U[p + ii] = ii * dd;

    N_KnotVectorFromRealArray( &knt, U, r );

    /* Handle special case of small np (one fit) */

    if( (eflg EQ NL_YES AND np EQ nn + 2)OR( eflg EQ NL_NO AND np EQ nn ) )
    {
        N_CrvInitArrays( &curv );

        error = N_FitCalcCrvParamValues( (NL_VOID *)P, np, NL_EPOINT, NL_CHORDLENGTH, uu );

        if( error EQ NL_YES )
            NL_OUT;

        if( eflg EQ NL_YES )
        {
            wp[0] = wp[np] = -1;
            error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, np, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            N_CopyPt( P[0], &Q[0] );
            N_CopyPt( P[np], &Q[np] );
            jj = 1;
            kk = np - 1;
        }
        else
        {
            error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, np, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            jj = 0;
            kk = np;
        }

        for ( ii = jj; ii <= kk; ii++ )
            error = N_CrvClosestPt( &curv, P[ii], uu[ii], top, toc, &uu[ii], &Q[ii] );

        error = NL_NO;
        NL_OUT;
    }

    /* Now the general case. */

    pnts = N_AllocPt1dArray( np + nn + 2, &SL );
    idx = N_AllocInt1dArray( np, &SL );

    if( pnts EQ NULL OR idx EQ NULL )
        NL_QUIT;

    PP = &pnts[nn + 1];

    for ( ii = 0; ii <= np; ii++ )
    {
        N_CopyPt( NL_ZERO, &PP[ii] );
        idx[ii] = 0;
    }

    if( eflg EQ NL_YES )
    {
        N_CopyPt( P[0], &PP[0] );
        N_CopyPt( P[np], &PP[np] );
    }

    /* Now do the segment fitting and projecting points back to */
    /* the segments. Do the interior first, then the two ends.  */

    kk = 0;
    k1 = 3;
    k2 = NL_MIN( nn, np - 3 );

    while( kk + nn LE np )
    {
        N_CrvInitArrays( &curv );

        for ( ii = 0; ii <= nn; ii++ )
            N_CopyPt( P[kk + ii], &pnts[ii] );

        error = N_FitCalcCrvParamValues( (NL_VOID *)pnts, nn, NL_EPOINT, NL_CHORDLENGTH, uu );

        if( error EQ NL_YES )
            NL_OUT;

        wp[0] = wp[nn] = 1.0;

        if( eflg EQ NL_YES )
        {
            if( kk EQ 0 )
                wp[0] = -1.0;

            if( kk + nn EQ np )
                wp[nn] = -1.0;
        }

        error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)pnts, wp, nn, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        for ( ii = k1; ii <= k2; ii++ )
        {
            error = N_CrvClosestPt( &curv, P[ii], uu[ii - kk], top, toc, &uu[ii - kk], &QQ );

            if( error EQ NL_NO )
            {
                N_VectorBlendPt( 1.0, QQ, &PP[ii] );
                idx[ii] += 1;
            }
        }

        kk += 1;

        if( k2 LT np - 3 )
            k2 += 1;

        if( k1 LT kk )
            k1 += 1;

        N_FreeCrv( &curv, &SL );
    }

    /* Finish off the interior points by scaling (form averages */
    /* of how many times each was projected).                   */

    for ( ii = 3; ii <= np - 3; ii++ )
    {
        dd = 1.0 / idx[ii];
        N_ScalePt( dd, PP[ii], &PP[ii] );
    }

    /* Now smooth first and last three points. */

    N_CrvInitArrays( &curv );

    N_CopyPt( P[0], &PP[0] );
    N_CopyPt( P[1], &PP[1] );
    N_CopyPt( P[2], &PP[2] );

    N_CopyPt( P[np], &PP[np] );
    N_CopyPt( P[np - 1], &PP[np - 1] );
    N_CopyPt( P[np - 2], &PP[np - 2] );

    if( eflg EQ NL_YES )
    {
        error = N_FitCrvApproxLstSq( PP, 3, 2, 2, NL_CHORDLENGTH, &curv, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvClosestPt( &curv, PP[1], 0.333, top, toc, &uu[1], &PP[1] );
        error = N_CrvClosestPt( &curv, PP[2], 0.666, top, toc, &uu[2], &PP[2] );

        N_FreeCrv( &curv, &SL );
        N_CrvInitArrays( &curv );

        error = N_FitCrvApproxLstSq( &PP[np - 3], 3, 2, 2, NL_CHORDLENGTH, &curv, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvClosestPt( &curv, PP[np - 2], 0.333, top, toc, &uu[1], &PP[np - 2] );
        error = N_CrvClosestPt( &curv, PP[np - 1], 0.666, top, toc, &uu[2], &PP[np - 1] );

        N_FreeCrv( &curv, &SL );
        N_CrvInitArrays( &curv );

        wp[0] = -1.0;
        wp[nn] = 1.0;

        error = N_FitCalcCrvParamValues( (NL_VOID *)PP, nn, NL_EPOINT, NL_CHORDLENGTH, uu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)PP, wp, nn, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvClosestPt( &curv, PP[1], uu[1], top, toc, &uu[1], &PP[1] );
        error = N_CrvClosestPt( &curv, PP[2], uu[2], top, toc, &uu[2], &PP[2] );

        N_FreeCrv( &curv, &SL );
        N_CrvInitArrays( &curv );

        wp[0] = 1.0;
        wp[nn] = -1.0;

        error = N_FitCalcCrvParamValues( (NL_VOID *)( &PP[np - nn] ), nn, NL_EPOINT, NL_CHORDLENGTH, uu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)( &PP[np - nn] ), wp, nn, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvClosestPt( &curv, PP[np - 1], uu[nn - 1], top, toc, &uu[nn - 1], &PP[np - 1] );
        error = N_CrvClosestPt( &curv, PP[np - 2], uu[nn - 2], top, toc, &uu[nn - 2], &PP[np - 2] );
    }
    else
    {
        wp[0] = 1.0;
        wp[nn] = 1.0;

        N_CopyPt( NL_ZERO, &pnts[0] );
        N_CopyPt( NL_ZERO, &pnts[1] );
        N_CopyPt( NL_ZERO, &pnts[2] );

        k1 = NL_MIN( 2, np - nn );

        for ( ii = 0; ii <= k1; ii++ )
        {
            error = N_FitCalcCrvParamValues( (NL_VOID *)( &PP[ii] ), nn, NL_EPOINT, NL_CHORDLENGTH, uu );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)( &PP[ii] ), wp, nn, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = ii; jj <= 2; jj++ )
            {
                error = N_CrvClosestPt( &curv, PP[jj], uu[jj - ii], top, toc, &uu[jj - ii], &QQ );
                N_VectorBlendPt( 1.0, QQ, &pnts[jj] );
            }

            N_FreeCrv( &curv, &SL );
            N_CrvInitArrays( &curv );
        }

        N_CopyPt( pnts[0], &PP[0] );
        N_ScalePt( 0.5, pnts[1], &PP[1] );

        if( k1 EQ 2 )
            dd = 1.0 / 3.0;
        else
            dd = 0.5;
        N_ScalePt( dd, pnts[2], &PP[2] );

        N_CopyPt( NL_ZERO, &pnts[nn - 2] );
        N_CopyPt( NL_ZERO, &pnts[nn - 1] );
        N_CopyPt( NL_ZERO, &pnts[nn] );

        k2 = np - nn;

        for ( ii = 0; ii <= k1; ii++ )
        {
            error = N_FitCalcCrvParamValues( (NL_VOID *)( &PP[k2 - ii] ), nn, NL_EPOINT, NL_CHORDLENGTH, uu );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)( &PP[k2 - ii] ), wp, nn, (NL_VOID *)D, wd, I, s, NL_EPOINT, uu, &knt, nc, p, &curv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = ii; jj <= 2; jj++ )
            {
                error = N_CrvClosestPt( &curv, PP[np - jj], uu[nn - jj + ii], top, toc, &uu[nn - jj + ii], &QQ );
                N_VectorBlendPt( 1.0, QQ, &pnts[nn - jj] );
            }

            N_FreeCrv( &curv, &SL );
            N_CrvInitArrays( &curv );
        }

        N_CopyPt( pnts[nn], &PP[np] );
        N_ScalePt( 0.5, pnts[nn - 1], &PP[np - 1] );

        if( k1 EQ 2 )
            dd = 1.0 / 3.0;
        else
            dd = 0.5;
        N_ScalePt( dd, pnts[nn - 2], &PP[np - 2] );
    }

    /* Load output array */

    for ( ii = 0; ii <= np; ii++ )
        N_CopyPt( PP[ii], &Q[ii] );

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITCRVTANGENTSKNOTSPARAMS: Curve approximation with end tangents, knots and pars    */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes a least-squares NURBS curve approxi-
     mation to a  given set of  points. Optionally end  tangents or end
     derivatives may be specified. The knot  vector and the  parameters
     where the  points are  assumed  must be  passed  in. If the output 
     curve  is  initialized  to  the  NULL  curve, memory is  allocated 
     locally. Otherwise  it is checked if enough memory is passed in. A 
     typical calling example:
 
       NL_POINT       *P;
       NL_CPOINT      *Pw;
       NL_INDEX       m, n;
       NL_DEGREE      p;
       NL_REAL        *u;
       NL_VECTOR      *Vs, *Ve;
       NL_CVECTOR     *Ws, *We;
       NL_KNOTVECTOR  *knt;
       NL_CURVE       cur;
       NL_STACKS      SG;
       ...
       (define arrays P or Pw, define p, n, Vs/Ws, Ve/We, u and knt);
       ...
       N_CrvInitArrays(&cur);
       N_FitCrvTangentsKnotsParams((NL_VOID *)P ,m,n,p,(NL_VOID *)Vs,(NL_VOID *)Ve,NL_EPOINT,NL_TANGENT,
                u,knt,&cur,&SG);
       N_FitCrvTangentsKnotsParams((NL_VOID *)Pw,m,n,p,(NL_VOID *)Ws,(NL_VOID *)We,NL_HPOINT,NL_TANGENT,
                u,knt,&cur,&SG);
 
     THE NL_PARAMETERS u AND THE KNOT NL_VECTOR knt MUST BE PASSED IN!!
 
 
   ACCESS:
   
     A   , input  ,  Points/control points to be approximated
     m   , input  ,  Highest index in A
     n   , input  ,  Highest index of control  point array of cur. Must 
                     satisfy:
                       No  tangent : n < m-3
                       One tangent : n < m-4
                       Two tangents: n < m-5
     p   , input  ,  Degree of approximating curve (must satisfy p<=n)
     As  , in/out ,  Start tangent/derivative:
                       !NULL: interpolate with start tangent
                        NULL: interpolate without start tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE MAY BE RESCALED!
     Ae  , in/out ,  End tangent/derivative
                       !NULL: interpolate with end tangent
                        NULL: interpolate without end tangent
                     IF IT IS A DIRECTION NL_VECTOR AND NOT A  NL_DERIVATIVE,
                     ITS MAGNITUDE MAY BE RESCALED!
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean data passed in
                       NL_HPOINT: Homogeneous data passed in
     der , input  ,  Flag:
                       NL_TANGENT   : As and/or Ae are  tangent directions
                                   only and must be scaled internally
                       NL_DERIVATIVE: As and/or Ae are derivatives and are
                                   used as passed in
     u   , input  ,  Parameters where data points are assumed
     knt , input  ,  Knot vector of the approximating curve
     cur , output ,  Approximating curve
     SG  , input  ,  cur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCrvTangentsKnotsParams( NL_VOID *A, NL_INDEX m, NL_INDEX n, NL_DEGREE p, NL_VOID *As, NL_VOID *Ae, NL_FLAG ptp, NL_FLAG der, NL_PARAMETER *u, NL_KNOTVECTOR *knt, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvTangentsKnotsParams");

    NL_FLAG error = NL_NO;

    NL_INDEX I[2], i, h;

    NL_REAL *wp, wd[2], len = 0.0, mag;

    NL_VECTOR D[2], *Vs = NULL, *Ve = NULL;

    NL_CVECTOR Dw[2], *Ws = NULL, *We = NULL;

    NL_POINT *P = NULL;

    NL_CPOINT *Pw = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Typecast input data */

    if( n GT m - 3 )
        NL_ERROR( NL_INP_ERR );

    if( ptp EQ NL_EPOINT )
    {
        P = (NL_POINT *)A;

        if( As NEQ NULL )
            Vs = (NL_VECTOR *)As;

        if( Ae NEQ NULL )
            Ve = (NL_VECTOR *)Ae;
    }
    else if( ptp EQ NL_HPOINT )
    {
        Pw = (NL_CPOINT *)A;

        if( As NEQ NULL )
            Ws = (NL_CVECTOR *)As;

        if( Ae NEQ NULL )
            We = (NL_CVECTOR *)Ae;
    }

    /* Set up constraint data */

    wp = N_AllocReal1dArray( m, &SL );

    if( wp EQ NULL )
        NL_QUIT;

    wp[0] = -1.0;
    wp[m] = -1.0;
    h = -1;

    for ( i = 1; i < m; i++ )
        wp[i] = 1.0;

    if( As NEQ NULL OR Ae NEQ NULL )
    {
        /* Scale tangents if neccesary */

        if( der EQ NL_TANGENT )
        {
            if( ptp EQ NL_EPOINT )
                N_DistPolygon( P, m, &len );

            else if( ptp EQ NL_HPOINT )
                N_DistCPolygonHomo( Pw, m, &len );

            len = len * (u[m] - u[0]);

            if( As NEQ NULL )
            {
                if( ptp EQ NL_EPOINT )
                {
                    N_PtMagnitude( *Vs, &mag );

                    if( N_FloatOpIsBad( len, mag, NL_DIVISION ) )
                        NL_ERROR( NL_INP_ERR );
                    mag = len / mag;
                    N_ScalePt( mag, *Vs, Vs );
                }
                else if( ptp EQ NL_HPOINT )
                {
                    N_CPtMagnitude( *Ws, &mag );

                    if( N_FloatOpIsBad( len, mag, NL_DIVISION ) )
                        NL_ERROR( NL_INP_ERR );
                    mag = len / mag;
                    N_ScaleCPt( mag, *Ws, Ws );
                }
            }

            if( Ae NEQ NULL )
            {
                if( ptp EQ NL_EPOINT )
                {
                    N_PtMagnitude( *Ve, &mag );

                    if( N_FloatOpIsBad( len, mag, NL_DIVISION ) )
                        NL_ERROR( NL_INP_ERR );
                    mag = len / mag;
                    N_ScalePt( mag, *Ve, Ve );
                }
                else if( ptp EQ NL_HPOINT )
                {
                    N_CPtMagnitude( *We, &mag );

                    if( N_FloatOpIsBad( len, mag, NL_DIVISION ) )
                        NL_ERROR( NL_INP_ERR );
                    mag = len / mag;
                    N_ScaleCPt( mag, *We, We );
                }
            }
        }

        /* Set up constraint data */

        if( As NEQ NULL AND Ae EQ NULL )
        {
            if( n GT m - 4 )
                NL_ERROR( NL_INP_ERR );

            wd[0] = -1.0;
            I[0] = 0;
            h = 0;

            if( ptp EQ NL_EPOINT )
                N_CopyPt( *Vs, &D[0] );

            else if( ptp EQ NL_HPOINT )
                N_CopyCPt( *Ws, &Dw[0] );
        }
        else if( As EQ NULL AND Ae NEQ NULL )
        {
            if( n GT m - 4 )
                NL_ERROR( NL_INP_ERR );

            wd[0] = -1.0;
            I[0] = m;
            h = 0;

            if( ptp EQ NL_EPOINT )
                N_CopyPt( *Ve, &D[0] );

            else if( ptp EQ NL_HPOINT )
                N_CopyCPt( *We, &Dw[0] );
        }
        else if( As NEQ NULL AND Ae NEQ NULL )
        {
            if( n GT m - 5 )
                NL_ERROR( NL_INP_ERR );

            wd[0] = -1.0;
            wd[1] = -1.0;
            I[0] = 0;
            I[1] = m;
            h = 1;

            if( ptp EQ NL_EPOINT )
                N_CopyPt( *Vs, &D[0] );

            else if( ptp EQ NL_HPOINT )
                N_CopyCPt( *Ws, &Dw[0] );

            if( ptp EQ NL_EPOINT )
                N_CopyPt( *Ve, &D[1] );

            else if( ptp EQ NL_HPOINT )
                N_CopyCPt( *We, &Dw[1] );
        }
    }

    /* Now fit data */

    if( ptp EQ NL_EPOINT )
    {
        error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, m, (NL_VOID *)D, wd, I, h, NL_EPOINT, u, knt, n, p, cur, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else if( ptp EQ NL_HPOINT )
    {
        error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)Pw, wp, m, (NL_VOID *)Dw, wd, I, h, NL_HPOINT, u, knt, n, p, cur, SG );

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
/* N_FitPeriodicCubic: Periodic cubic spline interpolation            */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a periodic cubic spline curve 
     interpolating a given set of periodic points. The parameter values 
     at each given point is given. If the output curve is  initialized  
     to the  NULL curve, memory  is allocated locally. Otherwise  it is  
     checked if  enough memory is passed in. A typical calling  example
     is as follows:

       NL_POINT  *Q;
       NL_REAL      *u;
       NL_INDEX      k;
       NL_FLAG   clamp;
       NL_CURVE    cur;
       NL_STACKS    SG;
       ...
       (get array P and set parameters u;
       ...
       N_FitPeriodicCubic(Q, u, k, &cur, &SG);

     Note: Q[k] = Q[0] and u[i] < u[i + 1]

   ACCESS:
   
     Q     , input  ,  Points to be interpolated
     u     , input  ,  Parameter values for each point
     k     , input  ,  Highest index in Q and u
     clamp , input  ,  Clamp the end knots, TRUE or FALSE
     cur   , output ,  Cubic spline
     SG    , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitPeriodicCubic( NL_POINT *Q, NL_REAL *u, NL_INDEX k, NL_FLAG clamp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitPeriodicCubic");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n;

    NL_REAL *a, *b, *c; /* the diagonals for the tridiagonal system */

    NL_REAL *t, t10, t11, t01, e, f;

    NL_REAL dist, alf, oma;

    NL_PARAMETER *U;

    NL_CPOINT *Pw;

    NL_KNOTVECTOR *knt;

    NL_POINT *P;

    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    if( k LE 1 )
        NL_ERROR( INP_ERR );

    /* Make sure the last point and first point are the same */
    N_DistPtPt( Q[0], Q[k], &dist );

    if( dist > MTOL )
        NL_ERROR( INP_ERR );

    /* Check for curve memory */
    n = k + 2;

    error = N_CrvSizeArrays( cur, n, 3, n + 4, rname, SG );

    if( error EQ YES )
        NL_OUT;

    N_CrvGetCPtsKnotVectorAndKnots( cur, &Pw, &knt, &U );

    /* Compute knot vector */

    U[3] = u[0];

    for ( i = 1; i <= k; i++ )
    { /* make sure the knots are distinct */
        if( u[i - 1] < u[i] - NL_PTOL )
            U[i + 3] = u[i];
        else
        {
            error = YES;
            NL_OUT;
        }
    }

    /* Must use periodic knots */
    for ( i = 1; i <= 3; i++ )
    {
        U[k + 3 + i] = U[k + 3 + i - 1] + (u[i] - u[i - 1]); /* ie U[k + 4] = U[k + 3] +(u[1] - u[0]) */
        U[3 - i] = U[4 - i] - (u[k + 1 - i] - u[k - i]);     /*   U[2] = U[3] -(u[k] - u[k - 1])  */
    }

    /* Allocate and set the diagonals a, b, c */

    a = N_AllocReal1dArray( k, &SL );
    b = N_AllocReal1dArray( k, &SL );
    c = N_AllocReal1dArray( k, &SL );

    if( a == NULL || b == NULL || c == NULL )
        NL_OUT;

    /* Fill in the a, b, c arrays */
    for ( i = 0; i < k; i++ )
    { /* Use the knots to evaluate the basis functions */
        t = &U[3 + i];
        t10 = t[1] - t[0];
        t11 = t[1] - t[-1];
        t01 = t[0] - t[-1];
        e = t10 / ((t[1] - t[-2]) * t11);
        f = t01 / ((t[2] - t[-1]) * t11);
        a[i] = t10 * e;
        b[i] = (t[0] - t[-2]) * e + (t[2] - t[0]) * f;
        c[i] = t01 * f;
    }

    /* Solve the cornered tridiagonal system */

    P = N_AllocPt1dArray( k + 2, &SL );

    if( P == NULL )
        NL_OUT;

    error = M_CorneredTridiagonalSystem( a, b, c, c[k - 1], a[0], Q, k, &P[1] );

    if( error )
        NL_OUT;

    /* For periodic, coefficients overlap */
    /* P[0] is same as P[k] */
    N_CopyPt( P[k], &P[0] );

    /* P[k + 1] is the same as P[1] */
    N_CopyPt( P[1], &P[k + 1] );

    /* And P[k + 2] is the same as P[2] */
    N_CopyPt( P[2], &P[k + 2] );

    /* Copy P to the Pw of cur */
    for ( i = 0; i <= n; i++ )
    {
        N_PtToCPt( P[i], &Pw[i] );
    }

    /* Clamp the cur if requested */
    if( clamp )
    {
        /* At the start, U[3] */
        alf = (U[3] - U[1]) / (U[4] - U[1]);
        oma = 1.0 - alf;
        N_Combine2CPts( oma, Pw[0], alf, Pw[1], &Pw[0] );
        alf = (U[3] - U[2]) / (U[5] - U[2]);
        oma = 1.0 - alf;
        N_Combine2CPts( oma, Pw[1], alf, Pw[2], &Pw[1] );
        alf = (U[3] - U[2]) / (U[4] - U[2]);
        oma = 1.0 - alf;
        N_Combine2CPts( oma, Pw[0], alf, Pw[1], &Pw[0] );

        U[2] = U[1] = U[0] = U[3];

        /* At the end, U[n + 1] */
        alf = (U[n + 1] - U[n]) / (U[n + 3] - U[n]);
        oma = 1.0 - alf;
        N_Combine2CPts( oma, Pw[n - 1], alf, Pw[n], &Pw[n] );
        alf = (U[n + 1] - U[n - 1]) / (U[n + 2] - U[n - 1]);
        oma = 1.0 - alf;
        N_Combine2CPts( oma, Pw[n - 2], alf, Pw[n - 1], &Pw[n - 1] );
        alf = (U[n + 1] - U[n]) / (U[n + 2] - U[n]);
        oma = 1.0 - alf;
        N_Combine2CPts( oma, Pw[n - 1], alf, Pw[n], &Pw[n] );

        U[n + 2] = U[n + 3] = U[n + 4] = U[n + 1];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

/***********************************************************************/
/* ST_CalcCPtsCircle: Compute weighted control points of Bezier circle */
/***********************************************************************/
NL_VOID ST_CalcCPtsCircle
  ( NL_POINT P0,      /* in : start control point position - gets weight = 1.0 */ 
    NL_POINT P1,      /* in : mid control point position   - gets weight = 1/2 * (dist(P0,P1)+dist(P1,P2))/dist(P0,P2) */ 
    NL_POINT P2,      /* in : end point point position     - gets weight = 1.0 */ 
    NL_CPOINT *Pw )   /* out: control points Pw[0], Pw[1], Pw[2] */ 
{

    NL_REAL e, fl, fr, fa, w;

    N_DistPtPt( P0, P2, &e );
    N_DistPtPt( P0, P1, &fl );
    N_DistPtPt( P1, P2, &fr );

    fa = 0.5 *( fl + fr );
    w = (0.5 *e) / fa;

    N_Weight( P0, 1.0, &Pw[0] );
    N_Weight( P1, w, &Pw[1] );
    N_Weight( P2, 1.0, &Pw[2] );
}

#if NLIB_UNUSED


/**********************************************************************/
/* N_FITSRFLSTSQDERIVS: Least squares surface approximation to random points     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  computes  a  least squares b-spline  curve 
     approximation to a sequenced set of points.  First  partial
     derivative vectors may (optionally) be input at each point.  
     When the number of sample points is large the
     algorithm can not interpolate every input point, it will find the
     best approximation to those points.
     
     If  the  output  curve is initialized to the NULL 
     surface,  memory is allocated locally.  Otherwise  it is checked if  
     enough memory is passed in. A typical calling example is:
 
       NL_POINT      *P;
       NL_VECTOR     *DU;
       NL_PARAMETER  *uu;
       NL_REAL       *wts, *uwts;
       NL_INDEX      n, np, nu, *Iu;
       NL_DEGREE     p;
       NL_CURVE      cur;
       NL_STACKS     SG;

       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqDerivs(P,wts,uu,np,DU,uwts,nu,Iu,n,p,U,&cur,&SG);
 
 
   ACCESS:
   
     P    , input  ,  Points to be approximated
     wts  , input  ,  Least squares point weights (wts[i] >= 0 for all i).
                      The  larger  wts[i],  the  closer the curve
                      comes to P[i]. wts[i] < 1 lessens the influence of
                      P[i]. If wts[i] = 0, then P[i] is not used.  If no
                      weighting is desired, set wts = NULL
     uu   , input  ,  The u-parameters of the points in P
     np   , input  ,  The high index of arrays: P, wts, and uu
     DU   , input  ,  First derivative vectors wrt u (optional). DU=NULL
                      means no u-derivatives specified
     uwts , input  ,  u-derivative weights (uwts[i] >= 0 for all i). The
                      larger uwts[i],  the  closer  the curve comes to 
                      assuming DU[i].  uwts[i] < 1 lessens the influence 
                      of DU[i].  If uwts[i] = 0, then DU[i] is not used.  
                      If no weighting is desired, set uwts = NULL
     nu   , input  ,  The high index of the arrays: DU, uwts, and Iu 
                      (set to -1 if no u-derivatives specified)
     Iu   , input  ,  DU[i] is the u-derivative at (uu[Iu[i]],vv[Iu[i]])
     n    , input  ,  High indexes of the curve  control  points 
                      (the curve will have (n+1) control points).  
     p    , input  ,  curve Degree
     U    , input  ,  Knots for the curve. If U=NULL, compute the knots
                      in this routine. If given, there must be n+p+2 u-knots
                      The values in the uu array must be contained by the 
                      given knot range (unless wts[i] = 0)
     cur  , output ,  Approximating curve
     SG   , input  ,  cur's memory stack

 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR

    Sample code including SMLib routines:
    - compute UV values 

    N_FitSrfCalcParams(P, m, NL_NO, Origin, X, Y, Z, 0.0, 1.0, 0.0, 1.0, uu, vv);
    
    - Fit surface to (xyz uv) points
    error = N_FitSrfLstSqDerivs(P, NULL, uu, vv, m, NULL, NULL, -1, NULL, NULL, NULL, 
            -1, NULL, 4, 4, 3, 3, NULL, NULL, &sur, &S);

    SmBSplineSurface *pSrf = new(sContext) SmBSplineSurface(&sur);   

   ***********************************************************************/
#ifdef OBSOLETE
   NL_FLAG N_FitCrvLstSqDerivs
     (NL_POINT   *P,       /* in : Points to be approximated,              sized:[np+1]  */ 
      NL_REAL    *wts,     /* in : Associated Least Squares Point weights, sized:[np+1]  */ 
      NL_REAL    *uu,      /* in : The u-parameters of the points in P,    sized:[np+1]  */ 
      NL_INDEX    np,      /* in : size parameter                                        */ 
      NL_VECTOR  *DU,      /* in : First derivative vectors wrt u (optional),     NULL to ignore, sized:[nu+1] */ 
      NL_REAL    *uwts,    /* in : Associated Least Squares u-derivative weights, NULL to ignore, sized:[nu+1] */ 
      NL_INDEX    nu,      /* in : size parameter                                        */ 
      NL_INDEX    *Iu,     /* in : DU and uwts index map,                  sized;[nu+1]  */
                           /*        e.g. DU[i] is u-derivative at (uu[Iu[i]],vv[Iu[i]]) */ 
      NL_INDEX    n,       /* in : Output Curve U ControlPoint count = (n+1)             */ 
      NL_DEGREE   p,       /* in : Output Curve U Degree                                 */ 
      NL_REAL    *U,       /* in : Optional Output Curve U Knot Vector, if given must have n+p+2 u-knots, Null to ignore */ 
      NL_CURVE   *cur,     /* out: Approximating Curve   */ 
      NL_STACKS  *SG )     /* in : cur's memory stack    */ 
   {
       NL_PRIVATE NL_STRING rname = _T("N_FitCrvLstSqDerivs");
   
       NL_FLAG error = NL_NO, pflg;
   
       NL_INDEX ii, jj, kk, me, uspan, vspan, j1, k1, *perm, ** uvsp, i2;
   
       NL_REAL *US, *VS, ** a_nunu, w, u1, u2, ** a_pq, u0, ** ND;
       
       NL_INDEX i0, j0, i1, USpan, mStart, mm, ll ;
   
       NL_REAL *pBu, *pBv, dBm, scl, off ; 
   
       NL_CPOINT *Qw, ** Sw;
   
       NL_POINT P1, *rhs, *nrhs;
   
       NL_KNOTVECTOR knt, *knu, *knv, *knus, *knvs;
   
       NL_RMATRIX rma;
   
       /* these A matrix gain terms are kind of arbitrary and can be tuned by experiment */
       NL_REAL alpha, beta, PtStiffness ;
   
       NL_STACKS SL;
   
       /* Start NURBS */
   
       N_InitNurbs( &SL );
   
       /* Check for input errors */
   
       if(n LT 1)                             /* no U Control Points */
           NL_ERROR( NL_INP_ERR );
   
       if(n LT p)                             /* fewer output ControlPoints than degree+1 in U dir */
           NL_ERROR( NL_INP_ERR );
   
       /* Check validity of option input U Knotvector */
       if( U NEQ NULL )
       {
           N_KnotVectorFromRealArray( &knt, U, n + p + 1 );
           error = N_KnotVectorIsValid( &knt, p, rname );
   
           if( error EQ NL_YES )
               NL_OUT;
       }
   
       /* remember NO input u-derivative data */
       if( nu LT 0 OR DU EQ NULL )
           nu = -1;
   
       /* initialise just to avoid compiler complaint */
       u1 = 0.0;
       u2 = 1.0;
   
       /* Check memory for output curve */
       /* allocate memory when cur arrays are NULL (via N_CrvInitArrays()) */
   
       error = N_CrvSizeArrays( cur, n, p, n + p + 1, rname, SG );
   
       if( error EQ NL_YES )
           NL_OUT;
   
       N_CrvGetCPtsAndKnots( cur, &Sw, &US);
       N_CrvGetKnotVectors( cur, &knu);
   
       /* Get the surface u--knots */
   
       u1 = u2 = 0.0;
   
       pflg = NL_NO;
   
       /* when given Opt U KnotVector - set knus as given */
       if( U NEQ NULL )
       {
           kk = n + p + 1;
   
           for ( ii = 0; ii <= kk; ii++ )
               US[ii] = U[ii];
           knus = knu;
       }
       else /* set knus to NULL and get max and min u values from input uu array */
       {
           knus = NULL;
   
           /* when given weights */
           if( wts NEQ NULL )
           {
               pflg = NL_YES;
               u1 =  1.0e+20;
               u2 = -1.0e+20;
   
               /* get max and min uu array values */
               for ( ii = 0; ii <= np; ii++ )
                   if( wts[ii]NEQ 0.0 )
                   {
                       if( u1 GT uu[ii] )
                           u1 = uu[ii];
   
                       if( u2 LT uu[ii] )
                           u2 = uu[ii];
                   }
           }
       }
   
       /* when curve knot vectors have not been specified */
       if( knus NEQ knu OR knvs NEQ knv )
       {
           /* construct surface knot vectors */
           error = N_FitCrvCalcKnotVectors
             ( uu,       /* in : u parameter values, sized:[nn+1] */                                               
               np,       /* in : highest u index */                                                          
               p,        /* in : approximating curve degree U */                                                 
               pflg,     /* in : NL_YES = use us,ue values to set max/min u knot values */             
                         /*      NL_NO  = find max/min u knot values in u arrays        */             
               u1,       /* in : min U param value, overridden by any input knu value   */                         
               u2,       /* in : max U param value, overridden by any input knu value   */                         
               knus,     /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */
               knu );    /* i/o: sized knot vector whose knot values are to be determined */                       
   
           if( error EQ NL_YES )
               NL_OUT;
   
           /* scale the knots so that the sample points won't lie exactly on the boundary */
           scl = 1.125 ;
           off = knu->U[0] - (knu->U[knu->m]-knu->U[0])*(scl-1)/2.0 - knu->U[0]*scl ; 
           for(ii=0;ii<=knu->m;ii++) { knu->U[ii] = scl * knu->U[ii] + off ; }
       }
   
#if 0
     /* with change to curves - map[ii] = ii = cind[ii] NO NEED FOR THEM ANY LONGER */
     { /* scope to remove */
         /* Allocate memory for the index map */
         map = N_AllocInt1dArray( n, &SL );
    
         if( map EQ NULL )
             NL_QUIT;
    
         cind = N_AllocInt1dArray( n, 1, &SL );
    
         if( cind EQ NULL )
             NL_QUIT;
    
         /* Define the map from Sw(i,j) to svd_a's column index                           */
         /* map[n+1]             = global index for every output surface control point    */
         /* glo_cind = cind[nuk+1] = Control Point ii indices for every global index   */
         /*     cind[kk] = ii the control point's ii index for global index kk            */
         /* gwc note: these maps could be replaced by indexing functions as                    */
         /*   map[ii][jj] = (jj*(n+1))+ii ;                                                    */
         /*   cind[kk][0] =  kk-((kk%(n+1))*n+1)                                               */
         /*   cind[kk][1] =  kk%(n+1)                                                          */
    
         nuk = -1; /* nuk+1 unknowns */
    
         for ( ii = 0; ii <= n; ii++ )
         {
             nuk += 1;
             map[ii] = nuk;
    
             cind[nuk] = jj;
         }
     } /* end scope to remove */
#endif

       Allocate memory for the right hand side and solution vector
   
       Qw = N_AllocCPt1dArray( n, &SL );
   
       if( Qw EQ NULL )
           NL_QUIT;
   
       me = np;          /* number of point positions */
       me += (nu + 1);   /* number of 1st derivatives */
   
       rhs = N_AllocPt1dArray( me, &SL );
   
       if( rhs EQ NULL )
           NL_QUIT;
   
       /* Now set up the overdetermined system of equations */
   
       u1 = US[0];
       u2 = US[n + 1];
   
       w = 1.0;
       u0 = u1 - 1.0;
   
       a_pq = N_AllocReal2dArray( me, p + 1, &SL );
   
       if( a_pq EQ NULL )
           NL_QUIT;
   
       uvsp = N_AllocInt2dArray( me, 1, &SL );
   
       if( uvsp EQ NULL )
           NL_QUIT;
   
       me = -1; /* me+1 equations to solve */
   
       glo_a = a_pq;
       glo_cind = cind;
       glo_uvsp = uvsp;
   
       /* set up system */
   
       /* first, the equations for the sample point positions */
   
       /* for every sample point position - build                                                             */
       /*  glo_a = a_pq[me+1][p+1]   = basis functions values for sample point uu,vv values                   */
       /*                              a_pq[ii][0:p]       = uu basis values for sample point ii              */
       /*  glo_uvsp = uvsp[me+1][1]  = knot spans for sample point uu, vv values                              */
       /*                              uvsp[ii][0] = max u knot index LE to uu value for sample point ii      */
       /*  rhs[me+1]                 = sample point positions                                                 */
       /*                              rhs[ii] = xyz position for sample point ii                             */
       /* when given optional wts, each rhs[ii] and a_pq[ii][0:p] uu basis values                             */
       /*   are multiplied by the given wts[ii] value        */
       /* ignore sample points when                          */
       /*       o. uu value is out of specified bounds       */
       /*       o. optional weight is LE 0.0                 */
       for ( ii = 0; ii <= np; ii++ )
       {
           if( uu[ii]LT u1 OR uu[ii]GT u2 )
               continue;
   
           if( wts NEQ NULL )
           {
               if( wts[ii]LE 0.0 )
                   continue;
               else
               {
                   if( wts[ii]EQ 1.0 )
                       w = 1.0;
                   else
                       w = sqrt( wts[ii] );
               }
           }
   
           me += 1;
   
           if( uu[ii] NEQ u0 )
           {
               u0 = uu[ii];
               error = N_BasisEval( knu, p, uu[ii], NL_LEFT, &a_pq[me][0], &uspan );
   
               if( error EQ NL_YES )
                   NL_OUT;
           }
           else
           {
               for ( jj = 0; jj <= p; jj++ )
                   a_pq[me][jj] = a_pq[me - 1][jj];
           }
   
   
           uvsp[me][0] = uspan;
   
           N_CopyPt( P[ii], &rhs[me] );
   
           if( w NEQ 1.0 )
           {
               u0 = u1 - 1.0;
   
               for ( jj = 0; jj <= p; jj++ )      /* <=== gwc: why isn't this jj <= p+q+1 */ 
                   a_pq[me][jj] *= w;             /* because all left side terms are p*q products and this puts just 1 w on the equation left and right */
               N_ScalePt( w, rhs[me], &rhs[me] );
           }
   
           for ( jj = 0; jj <= p; jj++ )
           {
               j1 = uspan - p + jj;
   
               for ( kk = 0; kk <= q; kk++ )
               {
                   k1 = vspan - q + kk;
   
                   if( map[j1][k1]LT 0 ) /* gwc: I think this is never true */
                   {
                       N_CPtToPtEuclid( Sw[j1][k1], &P1 );   /* gwc: I think this is uninitialized when sur allocated in this function */ 
                       N_VectorBlendPt( -(a_pq[me][jj] * a_pq[me][(p + 1) + kk]), P1, &rhs[me] );
                   }
               } /* end iter every sur DegreeV value */
           } /* end iter every sur DegreeU value */
       } /* end iter every sample point - building a matrix position equations */
   
       /* now the equations for the u-derivatives  get appended to same arrays used for positions */
       /* for every sample point u-derivative, build */
       /*  glo_a = a_pq[(np+1)+0:nu][p+q+2] = 1st u deriv and v basis functions values for sample point uu,vv values */
       /*              a_pq[np+1+ii][0:p]       = 1st deriv uu basis values for sample u deriv ii  */
       /*              a_pq[np+1+ii][(p+1)+0:q] = vv basis values for sample point ii              */
       /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point u-deriv uu, vv values                      */
       /*                              uvsp[np+1+ii][0] = max u knot index LE to uu value for sample point ii */
       /*                              uvsp[np+1+ii][1] = max v knot index LE to vv value for sample point ii */
       /*  rhs[me+1]                 = sample point u-derivs                                                  */
       /*                              rhs[np+1+ii] = DU vector for sample point u-deriv ii                   */
   
       ii = NL_MAX( p, q );
       ND = N_AllocReal2dArray( 1, ii, &SL );
   
       if( ND EQ NULL )
           NL_QUIT;
   
       w = 1.0;
       u0 = u1 - 1.0;
       v0 = v1 - 1.0;
   
       for ( ii = 0; ii <= nu; ii++ )
       {
           i2 = Iu[ii];
   
           if( uu[i2]LT u1 OR uu[i2]GT u2 )
               continue;
   
           if( vv[i2]LT v1 OR vv[i2]GT v2 )
               continue;
   
           if( uwts NEQ NULL )
           {
               if( uwts[ii]LE 0.0 )
                   continue;
               else
               {
                   if( uwts[ii]EQ 1.0 )
                       w = 1.0;
                   else
                       w = sqrt( uwts[ii] );
               }
           }
   
           me += 1;
   
           if( uu[i2]NEQ u0 )
           {
               u0 = uu[i2];
               error = N_BasisDerivs( knu, p, uu[i2], NL_LEFT, 1, ND, &uspan );
   
               if( error EQ NL_YES )
                   NL_OUT;
   
               for ( jj = 0; jj <= p; jj++ )
                   a_pq[me][jj] = ND[1][jj];
           }
           else
           {
               for ( jj = 0; jj <= p; jj++ )
                   a_pq[me][jj] = a_pq[me - 1][jj];
           }
   
           if( vv[i2]NEQ v0 )
           {
               v0 = vv[i2];
               error = N_BasisEval( knv, q, vv[i2], NL_LEFT, &a_pq[me][p + 1], &vspan );
   
               if( error EQ NL_YES )
                   NL_OUT;
           }
           else
           {
               for ( jj = 0; jj <= q; jj++ )
                   a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
           }
   
           uvsp[me][0] = uspan;
           uvsp[me][1] = vspan;
   
           N_CopyPt( DU[ii], &rhs[me] );
   
           if( w NEQ 1.0 )
           {
               u0 = u1 - 1.0;
   
               for ( jj = 0; jj <= p; jj++ )
                   a_pq[me][jj] *= w;
               N_ScalePt( w, rhs[me], &rhs[me] );
           }
       }
   
       /* at this point 3 arrays have been built from the sample data and 2 arrays for indexing */
       /* DATA                                                                                  */
       /*   glo_a = a_pq    = every row stores the u and v basis values for a sample position, u-deriv, or v-deriv. */
       /*                     rows 0:np                   = [u basis values, v basis values]                        */
       /*                     rows np+1:np+1+nu           = [u 1st deriv basis values, v basis values]              */
       /*                     rows np+1+nu+1:np+1+nu+1+nv = [u basis values, v 1st deriv basis values]              */
       /*   rhs             = every row stores a sample point xyz pos, u-deriv or v-deriv value                     */
       /*                     rows 0:np                   = [ P[i]  ]                           */
       /*                     rows np+1:np+1+nu           = [ DU[i] ]                           */
       /*                     rows np+1+nu+1:np+1+nu+1+nv = [ DV[i] ]                           */
       /*   glo_uvsp = uvsp = every row store the u and v span index for every sample           */ 
       /* NL_INDEXING                                                                              */
       /*   map[n+1][m+1]             = global index for every output surface control point     */
       /*   glo_cind = cind[nuk+1][2] = Control Point ii,jj indices for every global index      */
       /*       cind[kk][0] = ii the control point's ii index for global index kk               */
       /*       cind[kk][1] = jj the control point's jj index for global index kk               */
   
       /* check state more sample points than control points - no longer needed */
       /* if( me LE nuk )              */
       /*     NL_ERROR( NL_INP_ERR );  */
   
       /* set global size variables */
       glo_me = me;
       glo_nu = nuk;
       glo_p  = p;
       glo_q  = q;
   
       /* Now build and solve the system of equations                                  */
       /* let W(u,v) = Sum_l(w_l * B_l(u,v))                                           */
       /*    where w_l is the lth control point                                        */
       /*          B_l is the lth basis function                                       */
       /* for a tensor product surface B_l is separable as                             */
       /*     B_l(u,v) = B_i(u) * B_j(v)                                               */
       /*    where subscripts i and j are reserved for single direction                */
       /*          basis functions                                                     */
       /* Minimize 1/2 * (  Sum_ko(wt_k0*(W (u_k0,v_k0) - P_k0)**2       - positions   */
       /*                 + Sum_k1(wt_k1*(Wu(u_k1,v_k1) - DU_k1)**2      - u-derivs    */
       /*                 + Sum_k2(wt_k2*(Wv(u_k2,v_k2) - DV_k2)**2)     - v-derivs    */
       /*    Where Wu = d(W)/du and Wv = d(W)/dv                                       */
       /* To Solve take the partial of the cost function with respect to               */
       /*   each control point, w_l, and set those equations yielding a                */
       /*   matrix equation of the form                                                */
       /*    AX = B                                                                    */
       /*     where A[m,l] =   Sum_k0(B_m(u_k0,v_k0)  * B_l(u_k0,v_k0))                */
       /*                    + Sum_k1(Bu_m(u_k1,v_k1) * Bu_l(u_k1,v_k1))               */
       /*                    + Sum_k2(Bv_m(u_k2,v_k2) * Bv_l(u_k2,v_k2)) ;             */
       /*            X[m]  =  w_m, the mth control point                               */
       /*            B[m]  =   Sum_k0(B_m(u_k0,v_k0)  * P_k0)                          */
       /*                    + Sum_k1(Bu_m(u_k1,v_k1) * DU_k1)                         */
       /*                    + Sum_k2(Bv_m(u_k2,v_k2) * DV_k2) ;                       */
       /*     where Bu = d(B)/du and Bv = d(B)/dv                                      */
       /*     and   B_m (u,v) = B_mi(u)  * B_mj(v)                                     */
       /*           Bu_m(u,v) = Bu_mi(u) * B_mj(v)                                     */
       /*           Bv_m(u,v) = B_mi(u)  * Bv_mj(v)                                    */
       /*                                                                              */
       /* Solve AX = B via LU Decomposition                                            */
   
       a_nunu = N_AllocReal2dArray( nuk, nuk, &SL );
   
       if( a_nunu EQ NULL )
           NL_QUIT;
   
       /* Set up Normal Equations and nrhs */
   
       nrhs = N_AllocPt1dArray( nuk, &SL );
   
       if( nrhs EQ NULL )
           NL_QUIT;
   
       /* build A and B matrices */
   
       /* init A and B to zero - these clears are tuned to memory layout done */
       /* in  N_AllocPt1dArray() and N_AllocReal2dArray()                     */
       N_MemSet(nrhs,0,(nuk + 1) * sizeof( NL_POINT )) ;
       N_MemSet(a_nunu[0], 0, (nuk + 1) * (nuk + 1) * sizeof( NL_REAL )  );
   
       /* add stiffness terms to A matrix */
       alpha       = 1.0 ;
       beta        = 10.0 ; 
       PtStiffness = 1.0E10 ;
       N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, NULL, NULL, NULL, NULL) ;
   
       /* for every data point */
       for(kk=0;kk<=me;kk++)
         {
           /* u and v basis values for kkth data point */
           pBu = a_pq[kk] ;
           pBv = pBu + p + 1 ;
   
           /* u and v spans for the kkth data point    */
           USpan = uvsp[kk][0] ;
           VSpan = uvsp[kk][1] ;
   
           /* smallest global dof number for USpan/VSpan element */
           mStart = ((VSpan - q) * (n + 1)) + (USpan - p) ;
   
           /* for every V Basis function */
           for(j0=0;j0<=q;j0++)
             {
               /* map i0,j0 to global mm index */
               mm = mStart + (j0 * (n + 1)) ;
   
               /* for every U Basis function */
               for(i0=0;i0<=p;i0++, mm++)
                 {
                   /* Basis function B_m */
                   dBm = PtStiffness * pBu[i0] * pBv[j0] ;
   
                   /* add terms to B matrix B[mm] += rhs[kk] * B_m(u_k,v_k) */
                   N_VectorBlendPt(dBm , rhs[kk], &nrhs[mm] );
   
                   /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                   for(j1=0;j1<=q;j1++)
                     {
                       /* map i1,j1 to global ll index */
                       ll = mStart + (j1 * (n + 1)) ;
   
                       for(i1=0;i1<=p;i1++,ll++)
                         {
                           /* add terms to A matrix */
                           a_nunu[mm][ll] +=  dBm * pBu[i1] * pBv[j1] ;
                           
                         } /* end iter every U Basis function */
                     } /* end iter every V Basis function - making A terms */
   
                 } /* end iter every U Basis function */
             } /* end iter every V Basis function - making B and A terms */
         } /* end iter every pos, u-deriv and v-deriv input point */
   
       /* test new method of building a and b by comparing it to the old          */
       /*                                                                         */
       /*      a_nunu_test = N_AllocReal2dArray( nuk, nuk, &SL );                 */
       /*      if( a_nunu_test EQ NULL )                                          */
       /*          NL_QUIT;                                                       */
       /*                                                                         */
       /*      nrhs_test = N_AllocPt1dArray( nuk, &SL );                          */
       /*      if( nrhs_test EQ NULL )                                            */
       /*          NL_QUIT;                                                       */
       /*                                                                         */
       /*      col1 = N_AllocReal1dArray( me, &SL );                              */
       /*      if( col1 EQ NULL )                                                 */
       /*          NL_QUIT;                                                       */
       /*                                                                         */
       /*      col2 = N_AllocReal1dArray( me, &SL );                              */
       /*      if( col2 EQ NULL )                                                 */
       /*          NL_QUIT;                                                       */
       /*                                                                         */
       /*      for ( ii = 0; ii <= nuk; ii++ )                                    */
       /*      {                                                                  */
       /*          error = ST_GetRowsColMatrix2( -1, ii, col1 );                  */
       /*                                                                         */
       /*          if( error EQ NL_YES )                                          */
       /*              NL_OUT;                                                    */
       /*                                                                         */
       /*          N_CopyPt( NL_ZERO, &nrhs_test[ii] );                           */
       /*                                                                         */
       /*          for ( jj = 0; jj <= me; jj++ )                                 */
       /*              N_VectorBlendPt( col1[jj], rhs[jj], &nrhs_test[ii] );      */
       /*                                                                         */
       /*          for ( jj = ii; jj <= nuk; jj++ )                               */
       /*          {                                                              */
       /*              error = ST_GetRowsColMatrix2( -1, jj, col2 );              */
       /*                                                                         */
       /*              if( error EQ NL_YES )                                      */
       /*                  NL_OUT;                                                */
       /*                                                                         */
       /*              dd = 0.0;                                                  */
       /*                                                                         */
       /*              for ( kk = 0; kk <= me; kk++ )                             */
       /*                  dd += col1[kk] * col2[kk];                             */
       /*              a_nunu_test[ii][jj] = dd;                                  */
       /*          }                                                              */
       /*                                                                         */
       /*          for ( jj = ii + 1; jj <= nuk; jj++ )                           */
       /*              a_nunu_test[jj][ii] = a_nunu_test[ii][jj];                 */
       /*      }                                                                  */
       /*                                                                         */
       /*        now compare                                                      */
       /*      lTestCnt=0 ;                                                       */
       /*      dTestMax=0.0 ;                                                     */
       /*      for(ii=0;ii<=nuk;ii++)                                             */
       /*        {                                                                */
       /*          if(  fabs(nrhs[ii].x - nrhs_test[ii].x)                        */
       /*             + fabs(nrhs[ii].y - nrhs_test[ii].y)                        */
       /*             + fabs(nrhs[ii].z - nrhs_test[ii].z) > 1.0E-6)              */
       /*            lTestCnt++ ;                                                 */
       /*          if(  fabs(nrhs[ii].x - nrhs_test[ii].x)                        */
       /*             + fabs(nrhs[ii].y - nrhs_test[ii].y)                        */
       /*             + fabs(nrhs[ii].z - nrhs_test[ii].z) > dTestMax)            */
       /*            dTestMax = fabs(  fabs(nrhs[ii].x - nrhs_test[ii].x)         */
       /*                            + fabs(nrhs[ii].y - nrhs_test[ii].y)         */
       /*                            + fabs(nrhs[ii].z - nrhs_test[ii].z)) ;      */
       /*                                                                         */
       /*          for(jj=0;jj<=nuk;jj++)                                         */
       /*            {                                                            */
       /*              if(fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) > 1.0E-6)    */
       /*                lTestCnt++ ;                                             */
       /*              if(fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) > dTestMax)  */
       /*                dTestMax = fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) ;  */
       /*            }                                                            */
       /*        }                                                                */
       /*        end change temp test code                                        */
                                                            
   
       /* Now solve via LU decomposition (Crout with partial piv) */
   
       perm = N_AllocInt1dArray( nuk, &SL );
   
       if( perm EQ NULL )
           NL_QUIT;
   
       N_CreateRealMatrix( &rma, nuk, nuk, a_nunu, NL_MT_FULL, nuk );
   
       /* Failure to solve occurs here if data not ok.                                     */
       /*   One kind of problem: Every control point has to be near sample points.         */
       /*      When no sample points are within the domain of the basis functions of a     */
       /*      surface point, the equation for that point will be all zero and the solver  */
       /*      will fail.                                                                  */
       error = N_RealMatrixLuDecomposePivot( &rma, perm );
   
       if( error EQ NL_YES )
           NL_OUT;
   
       error = N_RealMatrixRightForBackPivot( &rma, perm, nrhs, Qw );
   
       if( error EQ NL_YES )
           NL_OUT;
   
       /* Load the control points into Sw[][] via the index map */
   
       for ( ii = 0; ii <= n; ii++ )
         {
           for ( jj = 0; jj <= m; jj++ )
             {
               if( map[ii][jj]GE 0 )
                   N_CopyCPt( Qw[map[ii][jj]], &Sw[ii][jj] );
             }
         }
   
       /* End NURBS and Exit */
   
       EXIT:
   
       N_EndNurbs( &SL );
   
       return (error);
   
   } /* end N_FitCrvLstSqDerivs */
#endif /* OBSOLETE */

#endif // NLIB_UNUSED
