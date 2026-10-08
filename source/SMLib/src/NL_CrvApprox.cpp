// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvApproximate.c: Curve approximation routines                           */
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

static NL_FLAG ST_ApproxCrvOnSrf( NL_PARAMETER, NL_POINT * );
static NL_FLAG ST_ApproxNca( NL_PARAMETER, NL_POINT * );
static NL_FLAG ST_CheckApproxErr( NL_CURVE *, NL_POINT, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL * );

/**********************************************************************/
/* N_ApproxCrvWithArcs: Approximate NURBS curve with circular arcs (biarcs)      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  approximation routine  approximates a given 2-D NURBS curve 
     with a piecewise circular curve, i.e., with biarcs. If the output 
     curve is  initialized  to  NULL,  memory  is  allocated  locally. 
     Otherwise it is checked if enough memory is passed in. A  typical 
     calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define curP and get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxCrvWithArcs(&curP,tol,&curQ,&SG);

     The output is a NURBS curve which is, in fact, a piecewise Bezier 
     curve given in  NURBS  form. If the individual  circular arcs are 
     wanted in  geometric  form, i.e., center,  radius, start and  end 
     angles, then do the following:

       k=n/2
       for i=0 to k-1
         j=2*i;
         call N_BezCenterRadiusFrom3CPts with control points Pw[j],Pw[j+1],Pw[j+2]
       end

     where n is the  highest  index of  control points in  curQ. IT IS
     RECOMMENDED TO LET THE  NL_FUNCTION ALLOCATE  MEMORY FOR curQ INSIDE 
     THE ROUTINE!


   ACCESS:
   
     curP  , input  ,  NURBS curve (must be x-y planar!)
     tol   , input  ,  Absolute error tolerance, taken as is!!
     curQ  , output ,  Approximating piecewise biarc curve
     SG    , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxCrvWithArcs( NL_CURVE *curP, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxCrvWithArcs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, m, n, r, s;

    NL_REAL *UB, *UQ, us, ul, ur;

    NL_CPOINT *Bw, *Qw;

    NL_CURVE ** curB, ** curD;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get smooth pieces */

    error = N_CrvDecomposeContinuity( curP, &curD, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get biarc approximation for each piece */

    curB = N_AllocArrayCrvPtrs( k, &SL );

    if( curB EQ NULL )
        NL_QUIT;

    n = 0;

    for ( i = 0; i <= k; i++ )
    {
        curB[i] = N_AllocCrv( &SL );

        if( curB[i]EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( curB[i] );
        error = N_ApproxContinuousCrvWithArcs( curD[i], tol, curB[i], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetArraySizes( curB[i], &r, &s );
        n += r;
    }

    /* Merge biarcs */

    error = N_CrvSizeArrays( curQ, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    N_CrvGetCPtsAndKnots( curB[0], &Bw, &UB );

    UQ[0] = UQ[1] = UQ[2] = UB[0];

    n = 0;
    us = UB[0];
    m = 2;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetCPtsAndKnots( curB[i], &Bw, &UB );
        N_CrvGetArraySizes( curB[i], &r, &s );

        for ( j = 0; j <= r; j++ )
            N_CopyCPt( Bw[j], &Qw[n + j] );

        for ( j = 3; j < s; j++ )
            UQ[++m] = us + UB[j];

        n += r;
        us = UQ[m];
    }

    UQ[++m] = us;

    N_CrvGetParamBounds( curP, &ul, &ur );
    N_CrvReparam( curQ, ul, ur );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_ApproxContinuousCrvWithArcs: Approximate continuous NURBS curve with biarcs           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  approximation routine  approximates a given 2-D NURBS curve 
     with a piecewise  circular curve, i.e., with biarcs. The curve is
     assumed to be at  least C^1 continuous, i.e. all  knots must have
     multiplicities less than the degree. This function is the working
     bee for N_ApproxCrvWithArcs that handles issues of knot multiplicity. If the 
     output curve is initialized to NULL, memory is allocated locally. 
     Otherwise it is checked if enough memory is passed in. A  typical 
     calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define curP and get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxContinuousCrvWithArcs(&curP,tol,&curQ,&SG);

     The output is a NURBS curve which is, in fact, a piecewise Bezier 
     curve given in  NURBS  form. If the individual  circular arcs are 
     wanted in  geometric  form, i.e., center,  radius, start and  end 
     angles, then do the following:

       k=n/2
       for i=0 to k-1
         j=2*i;
         call N_BezCenterRadiusFrom3CPts with control points Pw[j],Pw[j+1],Pw[j+2]
       end

     where n is the highest index of control points in curQ. 


   ACCESS:
   
     curP  , input  ,  NURBS curve; must be x-y planar and C^1 smooth!
     tol   , input  ,  Absolute error tolerance, taken as is!!
     curQ  , output ,  Approximating piecewise biarc curve
     SG    , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxContinuousCrvWithArcs
  ( NL_CURVE  * curP,  /* in : NURBS curve; must be x-y planar and C^1 smooth  */
    NL_REAL     tol,   /* in : Absolute error tolerance                        */
    NL_CURVE  * curQ,  /* out: Approximating piecewise biarc curve             */
    NL_STACKS * SG )   /* in : new object stack                                */
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxContinuousCrvWithArcs");

    NL_FLAG bClosed, aef, frt, sbd, reset = NL_NO, error = NL_NO;

    NL_DEGREE p;

    NL_INDEX i, j, k, l, m, n, ks, ke, kl, kr, kd, le=0, top;

    NL_INDEX cur_n, cur_m ;

    NL_REAL *U = NULL, *u = NULL, *ul = NULL, *ur = NULL, *t = NULL, um = 0.0, u1 = 0.0, u2 = 0.0, eps = 0.0, d = 0.0;

    NL_POINT *P, *R, CL[2], CR[2], Q[2], Pm;

    NL_CPOINT *Pw, Bw[18];

    NL_VECTOR *T;

    NL_STACKS SL;

    NL_INDEX std = 20;  /* length of sample point arrays - lengthened in the case that subdivision is needed */

    /* Start NURBS */

    N_InitNurbs( &SL );

    N_CrvGetDegree( curP, &p );

    /* Get linear decomposition of curve */

    eps = 0.5 *tol;

    error = N_ApproxCrvWithPolyline /* eff: build polyline (pts on crv) to chord height tol with  */
                                    /*      at least a point at every original knot param value   */
             ( curP,                /* in : target curve                                          */
               eps,                 /* in : max chord height                                      */
               NL_ABSOLUTE,         /* in : NL_ABSOLUTE = use tol as is                           */
                                    /*      NL_RELATIVE = multiply tol by arc length of cur       */
               NL_BOTH,             /* in : NL_POINTS     = output points only                    */
                                    /*      NL_PARAMETERS = output params only                    */
                                    /*      NL_BOTH       = output both                           */
               &P,                  /* out: output polyline points                                */
               &u,                  /* out: output polyline params                                */
               &k,                  /* out: highest index in output arrays                        */
               &SL );               /* in : new object stack */

    if( error EQ NL_YES )
        NL_OUT;

    /* Handle p=1 (input CurP degree = 1 ) as special case */

    if( p EQ 1 )
    {                /* k = highest index in polyline approx */ 
        n = 2 * k;   /* n = highest curQ control point index */

        /* output a degree 2 curve whose control points are the */ 
        /* polyline end and mid points                          */ 
        error = N_CrvSizeArrays( curQ,  /* in : NURBS curve to be created    */
                                 n,     /* in : highest control point index  */
                                 2,     /* in : degree                       */
                                 n + 3, /* in : highest knot index           */
                                 rname, /* in : name of calling routine      */
                                 SG );  /* in : new object stack             */

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, 
                             &Pw,    /* Pw = output curve control points */
                             &U );   /* U  = output curve knots */

        /* set first output control point */ 
        N_Weight( P[0],    /* in : euclidean point                     */
                  1.0,     /* in : weight or NL_NOW                    */
                 &Pw[0] ); /* out: euclidean/homogeneous control point */

        j = 0;

        /* build control point array as [p0 p01 p1 p12 p2 ... pn]      */
        /* where pi = point from polyline approx                       */
        /*       pij = mid point between polyline points p[i] and p[j] */ 
        for ( i = 0; i < k; i++ )
        {
            N_Combine2Pts( 0.5, P[i], 0.5, P[i + 1], &Pm );

            N_Weight( Pm, 1.0, &Pw[++j] );
            N_Weight( P[i + 1], 1.0, &Pw[++j] );
        }

        goto FINISH; /* to compute the knot vector, clean up, and return */ 
    } /* end degree p = 1 special case check */

    /* arrive here when input curP degree is higher than 1 */

    /* Compute derivatives at all the linear decomposition points */
    T = N_AllocPt1dArray( k+1, &SL );
    if( T EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
		if (i == k) {
			error = N_CrvDerivs(curP, u[i], NL_LEFT, 1, CL);
		}
		else {
			error = N_CrvDerivs(curP, u[i], NL_LEFT, 1, CL);
		}

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( CL[1], &T[i] );
    } /* end iter all linear decomposition points - evaluating tangents */

    /* locals */
    ul = N_AllocReal1dArray( std, &SL );   /* start ul array sized 20 */

    if( ul EQ NULL )
        NL_QUIT;

    ur = N_AllocReal1dArray( std, &SL );   /* start ur array sized 20 */

    if( ur EQ NULL )
        NL_QUIT;

    /* Fit biarcs to the polygon P[0],...,P[k] */

    if( N_CrvAreArraysNULL( curQ ) )
        reset = NL_YES;

    /* initial size estimate for output curQ */
    n = 8 * k;        /* as a guess assume control points for 2 biarcs per linear decomposition */
    error = N_CrvSizeArrays( curQ, n, 2, n + 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Output ControlPoint and Knot arrays */ 
    N_CrvGetCPtsAndKnots( curQ, &Pw, &U );

    /* check polyline to see if it's open or closed */
    N_DistPtPt( P[0], P[k], &d );  
    if( d LT NL_MTOL )
        bClosed = NL_YES;  /* gwc: when yes, stays yes until the 1st subdivision - then no after that */
    else                   /*      when su
						   bdivided, more than two arc segments will be output */
        bClosed = NL_NO;   /* gwc: when no, always no.  So it's both a closed bit and a subdivided bit */

    /* while markers */
    ks = 0;     /* working start index into PolyLine approximation */
    kd = k;     /* k = last polyline point */
    n = 0;      /* working index into output curQ->Pw array to write polyarc arrays */

    /* while there are polyline segments left to approximate                  */
    /* gwc: come back to the outer loop when subdivision is required,         */
    /*      ks will be set to be the 1st last point for which a arc was found */
    while( ks LT k ) /* working PolyLine index is less than last Polyline point */
      {
        /* set ke = to be the end of the working polyline interval being approximated by arcs */
        if( bClosed EQ NL_NO )
            ke = NL_MIN( (ks + kd), k );
        else
            ke = k / 2;

        /* kl and kr define a working polyline interval that can get subdivided into the ks/ke range as needed */
        /* The ks/ke polyline interval may have to be approximated by more than one birac. */
        /* when that happens the kl and kr indices are narrowed down until a curve segment is found that can be approximated */
        kl = ks;
        kr = ke;
        frt = NL_YES; /* frt = */
        sbd = NL_NO;  /* sbd: set to NL_YES if a segment had to be subdivided */

        /* while the working polyline segment still has pieces to approximate */
        while( kl NEQ ke )
          {
            /* Get a biarc from ks to ke */ 
            
            /* get controlPoints for a piecewise bezier deg 2 BSpline (usually 5 control points can be more) */
            /*           or 3 points when end points and tangents make a line */
            error = N_FitArcToEndPtsAndTangents( P[ks], T[ks], P[ke], T[ke], Bw, &l );

            if( error EQ NL_YES )
                NL_OUT;

            /* Check approx polygon is within tol of approx arc over current curve interval */
            error = N_CrvIsPolygonWithinTol( Bw,      /* in : Control points of biarc                       */           
                                             l,       /* in : Highest index in Pw (must be an even number)  */
                                            &P[ks],   /* in : Vertices of the polygon                       */
                                             ke - ks, /* in : Highest index in P                            */
                                             eps,     /* in : Error tolerance                               */
                                            &aef );   /* out: NL_YES: approximation is acceptable           */
                                                      /*      NL_NO : approximation is NOT acceptable       */
            if( error EQ NL_YES )
                NL_OUT;

            /* when approx arc is within tol of workingInterval of approx polygon */
            if( aef EQ NL_YES )
              {
                /* Approximation is successful */

                /* make sure curP arrays are large enough */
                N_CrvGetArraySizes( curQ, &cur_n, &cur_m );
                if(n+l > cur_n)
                  {
                    /* size the output curQ - it's up to caller to make sure the sizes can only get smaller */
                                
                    N_CrvExpand( curQ, 2*cur_n, 2*cur_n+3, SG ) ;
                    N_CrvGetCPtsAndKnots( curQ, &Pw, &U );

                  }

                /* append the biarc control points into the curQ->Pw array */
                /* the first biarc control point overwrites (should be equal) the last biArc control point */
                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Bw[j], &Pw[n + j] );

                /* move working start index to end of this fitted interval */
                kl = ke;
                le = l;

                /* normally extend working interval to the next interval       */
                /* but don't extend it until a closed curve is subdivided once */
                if(    frt EQ NL_YES 
                   AND ke LT k 
                   AND bClosed NEQ NL_YES)   /* polyline is closed and we have yet to subdivide */
                  {
                    ke = NL_MIN( (ke + kd), k );
                    kr = ke;
                  }

              } /* end approx polygon is within tol of approx arc for current curve interval */
            else /* approx polygon is not within tol of approx arc for current curve interval */
              {
                /* Approximation is not successful */

                /* when we have made current circArc as long as possible */
                if( (ke - ks)EQ 1 )
                  {
                    /* No internal points -> must subdivide piece */

                    um = 0.5 *( u[ks] + u[ke] );
                    sbd = NL_YES;

                    /* start subdivision stack with current interval [u[ks] u[ke]]  */
                    /* split in two as [ (um u[ke])     <== stack bottom */
                    /*                   (u[ks] um) ]   <== stack top */
                    /*    note: stack top intervals are processed first */
                    ul[0] = um;
                    ur[0] = u[ke];
                    ul[1] = u[ks];
                    ur[1] = um;
                    top = 1;

                    /* while there are subdvision parameter values on the subdivision stack */
                    /*  make biarc approx to subdivided curve segments */
                    while( top GE 0 )
                      {
                        /* Pop the stack */

                        u1 = ul[top];
                        u2 = ur[top];
                        top--;

                        /* Get biarc approximation of current subdvision segment */

                        error = N_CrvDerivs( curP, u1, NL_LEFT, 1, CL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_CrvDerivs( curP, u2, NL_LEFT, 1, CR );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_FitArcToEndPtsAndTangents( CL[0], CL[1], CR[0], CR[1], Bw, &l );

                        if( error EQ NL_YES )
                            NL_OUT;

                        /* Check error */

                        Q[0] = CL[0];
                        Q[1] = CR[0];

                        error = N_CrvIsPolygonWithinTol( Bw, l, Q, 1, eps, &aef );

                        if( error EQ NL_YES )
                            NL_OUT;

                        if( aef EQ NL_YES )
                          {
                            /* Error OK -> output segment */

                            /* make sure curP arrays are large enough */
                            N_CrvGetArraySizes( curQ, &cur_n, &cur_m );
                            if(n+l > cur_n)
                              {
                                /* size the output curQ - it's up to caller to make sure the sizes can only get smaller */
                                
                                N_CrvExpand( curQ, 2*cur_n, 2*cur_n+3, SG ) ;
                                N_CrvGetCPtsAndKnots( curQ, &Pw, &U );

                              }

                            /*  control points are being added to the Pw array without increasing its size */
                            for ( j = 0; j <= l; j++ )
                                N_CopyCPt( Bw[j], &Pw[n + j] );
                            n += l;
                          } /* end subdivided interval adequately approximated by biarc branch */
                        else
                          {
                            /* Error is not acceptable -> must subdivide again */ 

                            um = 0.5 *( u1 + u2 );

                            /* when subdivision is about to exceed the curve value memory storage - increase the storage size */
                            if( top + 2 GT std )
                              {
                                /* lengthen the ul array to an ever growing std size */
                                error = N_Realloc1dRealArray( &ul, std, std + std, &SL );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                /* lengthen the ur array to an ever growing std size */
                                error = N_Realloc1dRealArray( &ur, std, std + std, &SL );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                std += std;
                              }

                            /* push new subdivision parameters onto the subdivision parameter stack */
                            ul[top + 1] = um;
                            ur[top + 1] = u2;
                            ul[top + 2] = u1;
                            ur[top + 2] = um;
                            top += 2;
                          } /* end subdivided interval not adequately approximated by biarc branch */
                      } /* end while there are subdvision parameter values on the subdivision stack */

                    /* arrive here when subdivision has adequately approximated the polyline segment interval */
                    /* move the left working index to the end of this segment */
                    kl = ke;
                  } /* end working interval is equal to one piece of the polyline approx branc */
                else  /* the working interval has more than one polyline segment */
                      /* or the working interval has no more polyline segments   */
                  {
                    /* move the right working index to the end of this segment */
                    kr = ke;

                    /* remember that . .  */
                    frt = NL_NO;
                  } /* end else the remaining polygon segment is not exactly 1 branch */
              } /* end else approx polygon is not within tol of approx arc for current curve interval branch */

            if( frt EQ NL_NO )
                ke = (kl + kr) / 2;
          } /* end while the working polyline segment still has pieces to approximate */

        if( sbd EQ NL_NO )
            n += le;
        kd = NL_MAX( 1, (ke - ks) );
        ks = ke;
        bClosed = NL_NO;
      } /* end while there are polyline segments left to approximate */

    /* size the output curQ - it's up to caller to make sure the sizes can only get smaller */
    N_CrvSetSizeIndices( curQ, n, 2, n + 3 );

    /* Compute the knot vector */

    FINISH:

    l = n / 2;

    t = N_AllocReal1dArray( l, &SL );

    if( t EQ NULL )
        NL_QUIT;

    R = N_AllocPt1dArray( l, &SL );

    if( R EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= l; i++ )
    {
        j = 2 * i;

        N_CPtToPtEuclid( Pw[j], &R[i] );
    }

    /* calc parameter values for every control point */
    error = N_FitCalcCrvParamValues( (NL_VOID *)R, l, NL_EPOINT, NL_CHORDLENGTH, t );

    if( error EQ NL_YES )
        NL_OUT;

    m = -1;

    /* build knot vector from param value array */
    for ( i = 0; i <= 2; i++ )
        U[++m] = t[0];

    for ( i = 1; i < l; i++ )
    {
        U[++m] = t[i];
        U[++m] = t[i];
    }

    for ( i = 0; i <= 2; i++ )
        U[++m] = t[l];

    /* Compact curve */
    /* when control point and knot arrays are larger than required - set arrays to needed sizes */
    if( reset EQ NL_YES )
    {
        error = N_CrvCompress( curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ApproxContinuousCrvWithArcs */

#if NLIB_UNUSED

/**********************************************************************/
/* N_CrvArePtsWithinTol: Check error of biac approximation                        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation  routine checks the error  between a biarc curve
     and  a  point  set. For each  point  in  the  set it  computes  the 
     distance of that  point from the biarc. If all  distances are below
     the input error threshold, the approximation is accepted. A typical 
     calling example is:

       NL_CPOINT  *Pw;
       NL_POINT   *P;
       NL_INDEX   n, k;
       NL_REAL    eps;
       NL_FLAG    aef;
       ...
       (get Pw, P, n, k and eps);
       ...
       N_CrvArePtsWithinTol(Pw,n,P,k,eps,&aef)


   ACCESS:
   
     Pw  , input  ,  Control points of biarc
     n   , input  ,  Highest index in Pw (must be even number)
     P   , input  ,  Point set
     k   , input  ,  Highest index in P
     aef , output ,  Flag:
                       NL_YES: approximation is acceptable
                       NL_NO : approximation is NOT acceptable


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvArePtsWithinTol( NL_CPOINT *Pw, NL_INDEX n, NL_POINT *P, NL_INDEX k, NL_REAL eps, NL_FLAG *aef )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n2;

    NL_REAL *r, *alf, *bet, d, dmin;

    NL_POINT *C, *Ps, *Pe;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get circle data */

    *aef = NL_YES;
    n2 = n / 2;

    C = N_AllocPt1dArray( n2 - 1, &SL );

    if( C EQ NULL )
        NL_QUIT;

    Ps = N_AllocPt1dArray( n2 - 1, &SL );

    if( Ps EQ NULL )
        NL_QUIT;

    Pe = N_AllocPt1dArray( n2 - 1, &SL );

    if( Pe EQ NULL )
        NL_QUIT;

    r = N_AllocReal1dArray( n2 - 1, &SL );

    if( r EQ NULL )
        NL_QUIT;

    alf = N_AllocReal1dArray( n2 - 1, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( n2 - 1, &SL );

    if( bet EQ NULL )
        NL_QUIT;

    for ( i = 0; i < n2; i++ )
    {
        j = 2 * i;

        error = N_BezCenterRadiusFrom3CPts( Pw[j], Pw[j + 1], Pw[j + 2], &C[i], &r[i], &alf[i], &bet[i], &Ps[i], &Pe[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check error */

    for ( i = 1; i < k; i++ )
    {
        dmin = NL_BIGD;

        for ( j = 0; j < n2; j++ )
        {
            error = N_DistPtArc( P[i], C[j], r[j], alf[j], bet[j], Ps[j], Pe[j], &d );

            if( error EQ NL_YES )
                NL_OUT;

            if( d NEQ NL_UNDEFINED )
            {
                if( d LT dmin )
                    dmin = d;
            }
        }

        if( dmin GT eps )
        {
            *aef = NL_NO;
            break;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_CrvIsPolygonWithinTol: Check error of biarc approximation of NURBS curves        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation  routine checks the error  between a biarc curve
     and a polygon, representing a  decomposition of a  NURBS curve. For 
     each polygon leg, the  routine computes the  maximum  distance from 
     the biarc. If all  distances are below the  input error  threshold, 
     the approximation is accepted. A typical calling example is:

       NL_CPOINT  *Pw;
       NL_POINT   *P;
       NL_INDEX   n, k;
       NL_REAL    eps;
       NL_FLAG    aef;
       ...
       (get Pw, P, n, k and eps);
       ...
       N_CrvIsPolygonWithinTol(Pw,n,P,k,eps,&aef)


   ACCESS:
   
     Pw  , input  ,  Control points of biarc
     n   , input  ,  Highest index in Pw (must be an even number)
     P   , input  ,  Vertices of the polygon
     k   , input  ,  Highest index in P
     eps , input  ,  Error tolerance
     aef , output ,  Flag:
                       NL_YES: approximation is acceptable
                       NL_NO : approximation is NOT acceptable


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsPolygonWithinTol /* eff: check polyline is within tol of a biarc curve */
 ( NL_CPOINT *Pw,               /* in : Control points of biarc                       */ 
   NL_INDEX   n,                /* in : Highest index in Pw (must be an even number)  */
   NL_POINT * P,                /* in : Vertices of the polygon                       */
   NL_INDEX   k,                /* in : Highest index in P                            */
   NL_REAL    eps,              /* in : Error tolerance                               */
   NL_FLAG  * aef )             /* out: NL_YES: approximation is acceptable           */
                                /*      NL_NO : approximation is NOT acceptable       */ 
{
    NL_FLAG error = NL_NO;

    NL_INDEX *ind, i, j, n2;

    NL_REAL *r, *alf, *bet, d, dmin, dmax;

    NL_POINT *C, *Ps, *Pe;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get circle data */

    *aef = NL_YES;
    n2 = n / 2;

    C = N_AllocPt1dArray( 3 * n2, &SL );

    if( C EQ NULL )
        NL_QUIT;

    r = N_AllocReal1dArray( 3 * n2, &SL );

    if( r EQ NULL )
        NL_QUIT;

    Ps = &C[n2];
    Pe = &Ps[n2];
    alf = &r[n2];
    bet = &alf[n2];

    ind = N_AllocInt1dArray( k, &SL );

    if( ind EQ NULL )
        NL_QUIT;

    /* get center and radius for every biarc - linear segments set r[i] = NL_INFINITE */
    for ( i = 0; i < n2; i++ )
    {
        j = 2 * i;

        /* gwc: when 3 cpts are colinear, r is set to NL_INFINITE. (Not an error condition)                     */
        /*      Upcoming N_DistPtArc call catches r == NL_INFINITE and passes call along to N_DistPerpPtLineSeg */
        error = N_BezCenterRadiusFrom3CPts( Pw[j], Pw[j + 1], Pw[j + 2], &C[i], &r[i], &alf[i], &bet[i], &Ps[i], &Pe[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /***************/
    /* Check error */
    /***************/

    /* For each polygon point, find closest biarc */
    for ( i = 0; i <= k; i++ )
    {
        dmin = NL_BIGD;
        ind[i] = -1;

        /* for every bi arc in Pw */
        for ( j = 0; j < n2; j++ )
        {
            /* gwc: when r[j] = NL_INFINITE, call passed to N_DistPerpPtLineSeg */
            error = N_DistPtArc( P[i], C[j], r[j], alf[j], bet[j], Ps[j], Pe[j], &d );

            if( error EQ NL_YES )
                NL_OUT;

            if( d NEQ NL_UNDEFINED )
            {
                if( d LT dmin )
                {
                    dmin = d;
                    ind[i] = j;
                }
            }
        }  /* end iter j, every biarc in Pw */

        if( ind[i]LT 0 OR dmin GT eps )
        {
            *aef = NL_NO;
            NL_OUT;
        }
    } /* end iter i, every polygon point */

    /* For each polygon line segment, get distance from biarc */
    for ( i = 0; i < k; i++ )
    {
        dmax = -1.0;

        /* for every biarc in Pw */
        for ( j = 0; j < n2; j++ )
        {
            if( j NEQ ind[i]AND j NEQ ind[i + 1] )
                continue;

            error = N_DistMaxLineArc( C[j], r[j], alf[j], bet[j], Ps[j], Pe[j], P[i], P[i + 1], &d );

            if( error EQ NL_YES )
                NL_OUT;

            if( d NEQ NL_UNDEFINED )
            {
                if( d GT dmax )
                    dmax = d;
            }
        }  /* end iter j, every biarc in Pw */

        if( dmax LT 0.0 OR dmax GT eps )
        {
            *aef = NL_NO;
            break;
        }
    }  /* end iter i, every polyline line segment */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_CrvIsPolygonWithinTol */

/**********************************************************************/
/* N_ApproxCircArcWithCrv: Approximate circle (full or arc) with non-rational curve */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine approximates a given circle or  circular
     arc with a non-rational  curve of degree 2,3, or 4.  The  error  is 
     measured in terms of the deviation from center  (error in radius) ,
     and the desired error tolerance is specified either relative to the 
     length of the radius, or as an absolute distance. A typical calling 
     example is:

       NL_CURVE   cur;
       NL_DEGREE  deg;
       NL_POINT   cen, Ps, Pe;
       NL_VECTOR  X, Y;
       NL_REAL    tol, rad, as, ae;
       NL_STACKS  SG;
       ...
       (define tol and deg, and get circle defining data);
       ...
       N_CrvInitArrays(&cur);
       N_ApproxCircArcWithCrv(cen,X,Y,rad,as,ae,&Ps,&Pe,deg,tol,NL_RELATIVE,&cur,&SG);

     MEMORY TO STORE THE  OUTPUT NL_CURVE  cur  IS ALLOCATED  INSIDE THE
     ROUTINE! 


   ACCESS:
   
     cen   , input  ,  Center of circle
     X,Y   , in/out ,  Local coordinate system of circle
     rad   , input  ,  Radius of circle
     as,ae , input  ,  Start and end angles of circle
     Ps,Pe , input  ,  Start and end points of circle  (provided  to en-
                       sure strict continuity with neighboring  curves).
                       Not used if equal to NULL
     deg   , input  ,  Degree of approximating non-rational circle (2, 3
                       or 4)
     tol   , input  ,  Radial error tolerance (see eflg)
     eflg  , input  ,  Error tolerance flag:
                         NL_ABSOLUTE: tol is an absolute distance
                         NL_RELATIVE: tol is a percentage of the radius
                       If rad=2,  then  (tol,eflg)=(0.002,NL_ABSOLUTE)  and
                       (tol,eflg)=(0.1,NL_RELATIVE)  are  equivalent  error
                       tolerances. tol may have any value, however, this
                       routine achieves tolerances  only  in  the  range
                       1.e-6 % to 1 %  (NL_RELATIVE tol) for deg = 3 or 4 ,
                       or 1.e-5 % to 1 % for deg = 2
     cur   , output ,  Approximating non-rational curve
     SG    , input  ,  cur's stack

 
   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_ApproxCircArcWithCrv( NL_POINT cen, NL_VECTOR X, NL_VECTOR Y, NL_REAL rad, NL_REAL as, NL_REAL ae, NL_POINT *Ps, NL_POINT *Pe, NL_DEGREE deg, NL_REAL tol, NL_FLAG eflg, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxCircArcWithCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX nc;

    NL_REAL dd, sweep;

    NL_POINT Rs, Re;

    NL_SFUN sfn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors on input */

    if( deg LT 2 OR deg GT 4 )
        NL_ERROR( NL_INP_ERR );

    if( rad LE 0.0 OR as EQ ae )
        NL_ERROR( NL_INP_ERR );

    /* Get number of control points required for fit */

    if( ae LT as )
        ae = ae + 360.0;
    sweep = ae - as;

    if( eflg EQ NL_ABSOLUTE )
        tol = 100.0 *( tol / rad );

    N_SFuncInitArrays( &sfn );
    error = N_CalcNumCPtsToApproxArc( tol, sweep, deg, NL_NO, &nc, &sfn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now fit nonrational curve to circle data */

    N_CrvInitArrays( cur );

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_ERROR( NL_INP_ERR );
    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_ERROR( NL_INP_ERR );

    if( Ps EQ NULL )
    {
        dd = as * (NL_PI / 180.0);
        N_TranslateSum2Pts( cen, rad * cos( dd ), X, rad * sin( dd ), Y, &Rs );
    }
    else
        N_CopyPt( *Ps, &Rs );

    if( Pe EQ NULL )
    {
        if( sweep EQ 360.0 )
            N_CopyPt( Rs, &Re );
        else
        {
            dd = ae * (NL_PI / 180.0);
            N_TranslateSum2Pts( cen, rad * cos( dd ), X, rad * sin( dd ), Y, &Re );
        }
    }
    else
        N_CopyPt( *Pe, &Re );

    error = N_ApproxCircArcWithCrvData( cen, X, Y, rad, as, ae, Rs, Re, deg, nc, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#ifdef NLIB_UNUSED

/**********************************************************************/
/* N_ApproxCrvOnSrfWithCrv: Approximate curve on surface from uv-curve               */
/**********************************************************************/

NL_PRIVATE_TLS NL_CURVE *gloC;
NL_PRIVATE_TLS NL_SURFACE *gloS;
/*******************************************************************//**


   DESCRIPTION:

     This approximation  routine computes a NURBS curve, Q(t) ,  on the
     surface, S(u,v), which is an approximation of the image of the uv-
     domain curve, C(s).  Both C(s) and S(u,v) must be NL_G1 continuous. A
     global  method is used, consisting of interpolation, least squares
     approximation, and knot removal.  Q(t)  is  at least NL_C1 continuous
     and nonrational. Various options exist for the parameterization of
     Q(t). A typical calling example is as follows:

       NL_DEGREE     p;
       NL_REAL       Eg, Ep;
       NL_CURVE      curC, curQ;
       NL_SURFACE    sur;
       NL_STACKS     SQ;
       ...
       (define function:  NL_FLAG  evnF (NL_PARAMETER s, NL_PARAMETER *t);
       (choose p, Eg, and Ep);
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxCrvOnSrfWithCrv(&curC,&sur,p,NL_NO,NL_FUNCTION,evnF,Eg,Ep,&curQ,&SQ);

       curQ must be initialized to NULL for this function.


   ACCESS:
   
     curC , input  ,  The curve, C(s), in the uv-domain of the surface.
     sur  , input  ,  The surface in whose parameter domain C(s) lies.
     p    , input  ,  Degree of  the  approximating curve, Q(t).  p > 1
                      must hold. p = 3 is recommended
     tans , input  ,  Flag:
                       NL_YES: maintain the precise end tangent directions 
                       NL_NO : do not require precise end tangent  direct-
                            ions 
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(s)
                       NL_FUNCTION   : parameterization defined by evnF
     evnF , input  ,  Reparameterization function, t = f(s). Used  only
                      if par = NL_FUNCTION
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from Q(t) to
                      S(u(s),v(s)) to not be greater than Eg
     Ep   , input  ,  Parametric error tolerance.  For  par = NL_INHERITED
                      and  par = NL_FUNCTION ,  this  function  attemps to 
                      maintain  |S(u(s),v(s))-Q(t)| <= Ep  for  corres-
                      ponding s and t.  Ep  is  also used for the par = 
                      NL_CHORDLENGTH and par = NL_CENTRIPETAL cases, but it's
                      meaning is less precise. 
     curQ , output ,  Approximating curve (initialized to NULL)
     SQ   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_ApproxCrvOnSrfWithCrv( NL_CURVE *curC, NL_SURFACE *sur, NL_DEGREE p, NL_FLAG tans, NL_FLAG par, NL_FLAG(*evnF)( NL_PARAMETER, NL_PARAMETER * ), NL_REAL Eg, NL_REAL Ep, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxCrvOnSrfWithCrv");

    NL_FLAG error = NL_NO;

    NL_PARAMETER s0, s1, u, v;

    NL_VECTOR *Ts, *Te, D[4];

    NL_POINT ** Suv;

    NL_REAL du, dv, dummy;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvAreArraysNULL( curQ ) )
        NL_ERROR( NL_INP_ERR );

    /* Assign pointers to globals */

    gloC = curC;
    gloS = sur;

    /* get start/end parameters of curve */

    N_CrvGetParamBounds( curC, &s0, &s1 );

    /* Supply end tangents if required */

    Ts = NULL;
    Te = NULL;

    if( tans )
    {
        Suv = N_AllocPt2dArray( 1, 1, &SL );

        if( Suv EQ NULL )
            NL_OUT;

        error = N_CrvDerivs( curC, s0, NL_LEFT, 1, &D[0] );

        if( error EQ NL_YES )
            NL_OUT;
        N_PtToXYZ( D[0], &u, &v, &dummy );
        N_PtToXYZ( D[1], &du, &dv, &dummy );

        if( u LT gloS->knu->U[0] )
        {
            if( u + NL_PTOL LT gloS->knu->U[0] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                u = gloS->knu->U[0];
        }

        if( u GT gloS->knu->U[gloS->knu->m] )
        {
            if( u - NL_PTOL GT gloS->knu->U[gloS->knu->m] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                u = gloS->knu->U[gloS->knu->m];
        }

        if( v LT gloS->knv->U[0] )
        {
            if( v + NL_PTOL LT gloS->knv->U[0] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                v = gloS->knv->U[0];
        }

        if( v GT gloS->knv->U[gloS->knv->m] )
        {
            if( v - NL_PTOL GT gloS->knv->U[gloS->knv->m] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                v = gloS->knv->U[gloS->knv->m];
        }

        error = N_SrfDerivs( sur, u, v, NL_LEFT, NL_LEFT, NL_TRUE, 1, 1, Suv );

        if( error EQ NL_YES )
            NL_OUT;
        N_Combine2Pts( du, Suv[1][0], dv, Suv[0][1], &D[1] );
        Ts = &D[1];

        error = N_CrvDerivs( curC, s1, NL_RIGHT, 1, &D[2] );

        if( error EQ NL_YES )
            NL_OUT;
        N_PtToXYZ( D[2], &u, &v, &dummy );
        N_PtToXYZ( D[3], &du, &dv, &dummy );

        if( u LT gloS->knu->U[0] )
        {
            if( u + NL_PTOL LT gloS->knu->U[0] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                u = gloS->knu->U[0];
        }

        if( u GT gloS->knu->U[gloS->knu->m] )
        {
            if( u - NL_PTOL GT gloS->knu->U[gloS->knu->m] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                u = gloS->knu->U[gloS->knu->m];
        }

        if( v LT gloS->knv->U[0] )
        {
            if( v + NL_PTOL LT gloS->knv->U[0] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                v = gloS->knv->U[0];
        }

        if( v GT gloS->knv->U[gloS->knv->m] )
        {
            if( v - NL_PTOL GT gloS->knv->U[gloS->knv->m] )
            {
                NL_ERROR( NL_PAR_ERR );
            }
            else
                v = gloS->knv->U[gloS->knv->m];
        }

        error = N_SrfDerivs( sur, u, v, NL_LEFT, NL_LEFT, NL_TRUE, 1, 1, Suv );

        if( error EQ NL_YES )
            NL_OUT;
        N_Combine2Pts( du, Suv[1][0], dv, Suv[0][1], &D[3] );
        Te = &D[3];
    }

    /* Call the procedural curve approximator */

    error = N_ApproxProcCrvWithCrv( ST_ApproxCrvOnSrf, s0, s1, Ts, Te, p, par, evnF, Eg, Ep, NL_MTOL, curQ, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_ApproxArcWithCrv: Non-rational B-spline approximation of circle/arc        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine approximates a given circle or  circular
     arc with a non-rational curve of any degree and any level of conti-
     nuity. The  error is measured in  terms of the  deviation from  the 
     center (error in radius), and the desired error tolerance is speci-
     fied either relative to the length of the radius, or as an absolute 
     distance. A typical calling example is:

       NL_CURVE   cur;
       NL_DEGREE  p;
       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    tol, rad, as, ae;
       NL_INDEX   der, n;
       NL_STACKS  SG;
       ...
       (define tol, p, der, n, and get circle defining data);
       ...
       N_CrvInitArrays(&cur);
       N_ApproxArcWithCrv(C,X,Y,rad,as,ae,p,der,n,tol,NL_RELATIVE,&cur,&SG);

     MEMORY  TO  STORE THE  OUTPUT  NL_CURVE  cur  IS ALLOCATED  INSIDE THE
     ROUTINE! 


   ACCESS:
   
     C,X,Y , input  ,  Local coordinate system of circle
     rad   , input  ,  Radius of circle
     as,ae , input  ,  Start and end angles of circle. ae-as <= 360!
     p     , input  ,  Degree of approximating non-rational circle
     der   , input  ,  Highest  start  and  end  derivatives used in the
                       approximating curve. Recommended  value is der=2,
                       i.e. the approximating curve is  C^2  continuous.
                       der less than p must always hold!
     n     , input  ,  Highest index  of data  points to be sampled from 
                       the precise circle. Recommended n=NL_MAX(2,p-1). n>0
                       must always hold!
     tol   , input  ,  Radial error tolerance (see tfl)
     tfl   , input  ,  Error tolerance flag:
                         NL_ABSOLUTE: tol is taken as is
                         NL_RELATIVE: tol is a percentage of the radius
                       If  rad=2,  then  (tol,tfl)=(0.002,NL_ABSOLUTE)  and
                       (tol,tfl)=(0.1,NL_RELATIVE)  are   equivalent  error
                       tolerances.
     cur   , output ,  Approximating non-rational curve
     SG    , input  ,  cur's stack

 
   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxArcWithCrv( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL rad, NL_REAL as, NL_REAL ae, NL_DEGREE p, NL_INDEX der, NL_INDEX n, NL_REAL tol, NL_FLAG tfl, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxArcWithCrv");

    NL_FLAG app, error = NL_NO, fci = NL_NO;

    NL_INDEX i, j, k, l, ns, na, ma, m4;

    NL_REAL *U4, *UA, *UB, ltol, ktol, ainc, us, ang, dang, erc, astr, aend, toc;

    NL_POINT *P;

    NL_VECTOR *Ds, *De, *Db = NULL;

    NL_CPOINT *Aw, *Bw;

    NL_CURVE cur4, curA, curB;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error */

    while( ae LE as )
        ae = 360.0 + ae;

    while( (ae - as)GE 360.1 )
        ae = ae - 360.0;

    if( (ae - as)GT 360.0 )
        ae = as + 360.0;

    na = n + der + der;
    ma = na + p + 1;
    dang = ae - as;

    if( p LT 2 OR p GT na )
        NL_ERROR( NL_INP_ERR );

    if( der GE p OR der LT 0 )
        NL_ERROR( NL_INP_ERR );

    if( n LT 1 OR dang GT 360.0 )
        NL_ERROR( NL_INP_ERR );

    /* Adjust tolerance and coordinate vectors */

    if( tfl EQ NL_RELATIVE )
        ltol = 0.01 *rad * tol;
    else
        ltol = tol;

    ktol = 0.1 *ltol;
    ltol = 0.9 *ltol;
    toc = NL_MIN( 1.0e-06, 0.1 *ltol );
    astr = as;
    aend = ae;

    if( dang EQ 360.0 )
        fci = NL_YES;

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get memory */

    P = N_AllocPt1dArray( n, &SL );

    if( P EQ NULL )
        NL_QUIT;

    Ds = N_AllocPt1dArray( der, &SL );

    if( Ds EQ NULL )
        NL_QUIT;

    De = N_AllocPt1dArray( der, &SL );

    if( De EQ NULL )
        NL_QUIT;

    if( fci EQ NL_YES )
    {
        Db = N_AllocPt1dArray( der, &SL );

        if( Db EQ NULL )
            NL_QUIT;
    }

    error = N_AllocCrvArrays( &cur4, 12, 4, 17, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocCrvArrays( &curA, na, p, ma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /**********************/
    /* Approximate circle */
    /**********************/

    ns = 0;

    while( 1 )
    {
        ns++;

        if( ns GT 1 )
            ae = astr + (aend - astr) / ns;

        N_CrvSetSizeIndices( &cur4, 12, 4, 17 );
        error = N_CreateQuarticArc( C, X, Y, rad, astr, ae, &cur4, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnots( &cur4, &m4, &U4 );

        /* Get end derivatives */

        error = N_CrvDerivs( &cur4, U4[0], NL_LEFT, der, Ds );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( &cur4, U4[m4], NL_LEFT, der, De );

        if( error EQ NL_YES )
            NL_OUT;

        if( fci EQ NL_YES )
            for ( i = 0; i <= der; i++ )
                N_VectorCopy( Ds[i], &Db[i] );

        /* Sample points uniformly */

        ainc = (ae - astr) / n;

        for ( i = 0; i <= n; i++ )
        {
            if( i EQ 0 )
                ang = astr;
            else
            {
                if( i EQ n )
                    ang = ae;
                else
                    ang = astr + i * ainc;
            }

            N_CalcCircArcDerivs( C, X, Y, rad, ang, 0, &P[i] );
        }

        /* Get approximating curve */

        error = N_FitCrvHighDerivs( P, n, p, Ds, der, De, der, NL_CHORDLENGTH, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Check error of approximation */

        error = ST_CheckApproxErr( &curA, C, rad, astr, ae, toc, &erc );

        if( error EQ NL_YES )
            NL_OUT;

        /* If approximation is not acceptable, continue */

        if( erc GT ltol )
            continue;

        /* Otherwise, try to piece the circle together */

        Bw = N_AllocCPt1dArray( ns * na, &SL );

        if( Bw EQ NULL )
            NL_QUIT;

        UB = N_AllocReal1dArray( ns * ma, &SL );

        if( UB EQ NULL )
            NL_QUIT;

        N_CrvGetCPtsDegreeAndKnots( &curA, &na, &Aw, &p, &ma, &UA );

        dang = ae - astr;
        us = UA[0];
        k = -1;
        l = p;

        for ( i = 0; i <= p; i++ )
            UB[i] = UA[i];

        for ( i = 0; i <= na; i++ )
            N_CopyCPt( Aw[i], &Bw[++k] );

        for ( i = p + 1; i <= na; i++ )
            UB[++l] = us + UA[i];

        for ( i = 1; i <= p; i++ )
            UB[++l] = us + UA[ma];

        us = UB[l];
        app = NL_YES;

        /* Get rest of the arcs */

        for ( i = 2; i <= ns; i++ )
        {
            as = ae;
            ae = as + dang;

            /* Get next circle */

            N_CrvSetSizeIndices( &cur4, 12, 4, 17 );
            error = N_CreateQuarticArc( C, X, Y, rad, as, ae, &cur4, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetKnots( &cur4, &m4, &U4 );

            for ( j = 0; j <= der; j++ )
                N_VectorCopy( De[j], &Ds[j] );

            /* Sample points */

            ainc = (ae - as) / n;

            for ( j = 0; j <= n; j++ )
            {
                if( j EQ 0 )
                    ang = as;
                else
                {
                    if( j EQ n )
                        ang = ae;
                    else
                        ang = as + j * ainc;
                }

                N_CalcCircArcDerivs( C, X, Y, rad, ang, 0, &P[j] );
            }

            /* Get end derivatives */

            if( i EQ ns AND fci EQ NL_YES ) /* Last arc */
            {
                for ( j = 0; j <= der; j++ )
                    N_VectorCopy( Db[j], &De[j] );
            }
            else
            {
                error = N_CrvDerivs( &cur4, U4[m4], NL_LEFT, der, De );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* Get approximating curve */

            error = N_FitCrvHighDerivs( P, n, p, Ds, der, De, der, NL_CHORDLENGTH, &curA, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Check error of approximation */

            error = ST_CheckApproxErr( &curA, C, rad, as, ae, toc, &erc );

            if( error EQ NL_YES )
                NL_OUT;

            /* If approximation is not acceptable, get a smaller arc and restart */

            if( erc GT ltol )
            {
                app = NL_NO;
                break;
            }

            /* Load control points and knots */

            N_CrvGetCPtsDegreeAndKnots( &curA, &na, &Aw, &p, &ma, &UA );

            for ( j = 1; j <= na; j++ )
                N_CopyCPt( Aw[j], &Bw[++k] );

            for ( j = p + 1; j <= na; j++ )
                UB[++l] = us + UA[j];

            for ( j = 1; j <= p; j++ )
                UB[++l] = us + UA[ma];

            us = UB[l];
        }

        if( app EQ NL_NO )
        {
            N_FreeCPt1dArray( Bw, &SL );
            N_FreeReal1dArray( UB, &SL );
        }
        else
        {
            break;
        }
    }

    UB[++l] = us;

    error = N_CrvFromCPtsAndKnots( &curB, Bw, k, p, UB, l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Rescale knot vector and remove all removable knots */

    N_CrvReparamToInterval( &curB, NL_UNITSPAN );

    N_CrvInitArrays( cur );
    error = N_CrvRemoveKnots( &curB, ktol, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_CalcNumCPtsToApproxArc: Compute number of ctrl pts for circle approximation      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation utility computes the number of  control  points
     required to approximate a given NURBS circle or circular arc  with
     a non-rational curve of degree 2,3, or 4.  Approximation error  is  
     measured in terms of the deviation from center  (error in the rad-
     ius),  and the desired error tolerance is specified as a  percent-
     age of the length  of  the  radius.  This  function  assumes  that 
     N_ApproxCircArcWithCrvDATA will be used to fit the non-rational approximation to the
     circle data.  This  function uses a surface function,  with  sweep 
     angle and error tolerance as the two parameters,  to  compute  the
     number of control points. This surface function may also be passed 
     in and out of this routine. A typical calling example is:

       NL_SFUN    sfn;
       NL_DEGREE  deg;
       NL_INDEX   nc;
       NL_REAL    tol, ang;
       NL_STACKS  SG;
       ...
       (get tol, ang and deg);
       ...
       N_SFuncInitArrays(&sfn);
       N_CalcNumCPtsToApproxArc(tol,ang,deg,NL_YES,&nc,&sfn,&SG);


   ACCESS:
   
     tol  , input  ,  Desired error tolerance, expressed as a percent-
                      age of radius; e.g. 0.1 means the absolute error
                      in radial distance is <= 0.001*radius. This rou-
                      tine truncates tol to the range 1.0 to 1.e-6  if
                      deg = 3 or 4, or 1.0 to 1.e-5 if deg = 2
     ang  , input  ,  Sweep angle of arc, in degrees. Must be in range
                      0 < ang <= 360
     deg  , input  ,  Degree of approximating non-rational circle  (2,
                      3 or 4)
     flg  , input  ,  Flag:
                        NL_YES: create sfn on global stack and return it
                        NL_NO : create sfn on local stack, (not returned)
                      This flag is ignored if sfn is passed in
     nc   , output ,  The high index of  control  points  required  by
                      N_ApproxCircArcWithCrvDATA to fit the arc to within tol error
     sfn  , output ,  The surface function used to compute nc from tol
                      and ang. If sfn is initialized to NULL on input,
                      it is created in this routine either  on  SG  or
                      this routine's local stack (see flg).  If  it is
                      not initialized to NULL on input,  this  routine
                      assumes it is a valid function and uses it
     SG   , input  ,  sfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CalcNumCPtsToApproxArc( NL_REAL tol, NL_REAL ang, NL_DEGREE deg, NL_FLAG flg, NL_INDEX *nc, NL_SFUN *sfn, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CalcNumCPtsToApproxArc");

    NL_FLAG error = NL_NO;

    NL_INDEX kill = 0, nu, nv;

    NL_REAL dd, uu[9], vv[7], ** fv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors */

    if( deg LT 2 OR deg GT 4 )
        NL_ERROR( NL_INP_ERR );

    if( ang LE 0.0 OR ang GT 360.0 )
        NL_ERROR( NL_INP_ERR );

    if( tol LT 1.e-6 )
        tol = 1.e-6;

    if( tol GT 1.0 )
        tol = 1.0;

    if( tol LT 1.e-5 AND deg EQ 2 )
        tol = 1.e-5;

    /* Create the surface function, sfn, if necessary */

    if( N_SrfAreFuncArraysNULL( sfn ) )
    {
        nu = 8;
        nv = 6; /* high indexes for interpolation */

        uu[0] = 0.0;
        uu[1] = 45.0;
        uu[2] = 90.0;
        uu[3] = 135.0;
        uu[4] = 180.0;
        uu[5] = 225.0;
        uu[6] = 270.0;
        uu[7] = 315.0;
        uu[8] = 360.0;

        vv[0] = 0.000001;
        vv[1] = 0.00001;
        vv[2] = 0.0001;
        vv[3] = 0.001;
        vv[4] = 0.01;
        vv[5] = 0.1;
        vv[6] = 1.0;

        fv = N_AllocReal2dArray( nu, nv, &SL );

        if( fv EQ NULL )
            NL_QUIT;

        /* depends on degree */

        switch( deg )
        {
            case 2:

                fv[0][0] = 8.0;
                fv[1][0] = 38.0;
                fv[2][0] = 68.0;
                fv[3][0] = 98.0;
                fv[4][0] = 128.0;
                fv[5][0] = 158.0;
                fv[6][0] = 188.0;
                fv[7][0] = 218.0;
                fv[8][0] = 248.0;

                fv[0][1] = 7.0;
                fv[1][1] = 30.0;
                fv[2][1] = 56.0;
                fv[3][1] = 81.0;
                fv[4][1] = 109.0;
                fv[5][1] = 135.0;
                fv[6][1] = 161.0;
                fv[7][1] = 185.0;
                fv[8][1] = 211.0;

                fv[0][2] = 6.0;
                fv[1][2] = 20.0;
                fv[2][2] = 34.0;
                fv[3][2] = 48.0;
                fv[4][2] = 62.0;
                fv[5][2] = 76.0;
                fv[6][2] = 90.0;
                fv[7][2] = 105.0;
                fv[8][2] = 120.0;

                fv[0][3] = 5.0;
                fv[1][3] = 13.0;
                fv[2][3] = 21.0;
                fv[3][3] = 29.0;
                fv[4][3] = 37.0;
                fv[5][3] = 45.0;
                fv[6][3] = 53.0;
                fv[7][3] = 61.0;
                fv[8][3] = 69.0;

                fv[0][4] = 4.0;
                fv[1][4] = 8.0;
                fv[2][4] = 13.0;
                fv[3][4] = 17.0;
                fv[4][4] = 22.0;
                fv[5][4] = 26.0;
                fv[6][4] = 31.0;
                fv[7][4] = 35.0;
                fv[8][4] = 39.0;

                fv[0][5] = 3.0;
                fv[1][5] = 5.0;
                fv[2][5] = 8.0;
                fv[3][5] = 10.0;
                fv[4][5] = 13.0;
                fv[5][5] = 15.0;
                fv[6][5] = 18.0;
                fv[7][5] = 20.0;
                fv[8][5] = 23.0;

                fv[0][6] = 2.0;
                fv[1][6] = 4.0;
                fv[2][6] = 5.0;
                fv[3][6] = 7.0;
                fv[4][6] = 9.0;
                fv[5][6] = 10.0;
                fv[6][6] = 12.0;
                fv[7][6] = 13.0;
                fv[8][6] = 15.0;

                break;

            case 3:

                fv[0][0] = 4.0;
                fv[1][0] = 21.0;
                fv[2][0] = 39.0;
                fv[3][0] = 56.0;
                fv[4][0] = 74.0;
                fv[5][0] = 92.0;
                fv[6][0] = 108.0;
                fv[7][0] = 124.0;
                fv[8][0] = 140.0;

                fv[0][1] = 3.0;
                fv[1][1] = 12.0;
                fv[2][1] = 22.0;
                fv[3][1] = 31.0;
                fv[4][1] = 40.0;
                fv[5][1] = 49.0;
                fv[6][1] = 59.0;
                fv[7][1] = 67.0;
                fv[8][1] = 74.0;

                fv[0][2] = 3.0;
                fv[1][2] = 8.0;
                fv[2][2] = 13.0;
                fv[3][2] = 18.0;
                fv[4][2] = 23.0;
                fv[5][2] = 29.0;
                fv[6][2] = 34.0;
                fv[7][2] = 38.0;
                fv[8][2] = 43.0;

                fv[0][3] = 3.0;
                fv[1][3] = 5.0;
                fv[2][3] = 8.0;
                fv[3][3] = 11.0;
                fv[4][3] = 14.0;
                fv[5][3] = 17.0;
                fv[6][3] = 20.0;
                fv[7][3] = 23.0;
                fv[8][3] = 26.0;

                fv[0][4] = 3.0;
                fv[1][4] = 4.0;
                fv[2][4] = 6.0;
                fv[3][4] = 7.0;
                fv[4][4] = 9.0;
                fv[5][4] = 11.0;
                fv[6][4] = 12.0;
                fv[7][4] = 14.0;
                fv[8][4] = 16.0;

                fv[0][5] = 3.0;
                fv[1][5] = 3.0;
                fv[2][5] = 4.0;
                fv[3][5] = 5.0;
                fv[4][5] = 6.0;
                fv[5][5] = 7.0;
                fv[6][5] = 8.0;
                fv[7][5] = 9.0;
                fv[8][5] = 10.0;

                fv[0][6] = 3.0;
                fv[1][6] = 3.0;
                fv[2][6] = 4.0;
                fv[3][6] = 4.0;
                fv[4][6] = 5.0;
                fv[5][6] = 5.0;
                fv[6][6] = 6.0;
                fv[7][6] = 6.0;
                fv[8][6] = 7.0;

                break;

            case 4:

                fv[0][0] = 4.0;
                fv[1][0] = 8.0;
                fv[2][0] = 12.0;
                fv[3][0] = 16.0;
                fv[4][0] = 20.0;
                fv[5][0] = 24.0;
                fv[6][0] = 28.0;
                fv[7][0] = 32.0;
                fv[8][0] = 36.0;

                fv[0][1] = 4.0;
                fv[1][1] = 6.0;
                fv[2][1] = 9.0;
                fv[3][1] = 12.0;
                fv[4][1] = 15.0;
                fv[5][1] = 18.0;
                fv[6][1] = 20.0;
                fv[7][1] = 23.0;
                fv[8][1] = 26.0;

                fv[0][2] = 4.0;
                fv[1][2] = 5.0;
                fv[2][2] = 7.0;
                fv[3][2] = 9.0;
                fv[4][2] = 11.0;
                fv[5][2] = 13.0;
                fv[6][2] = 15.0;
                fv[7][2] = 17.0;
                fv[8][2] = 19.0;

                fv[0][3] = 4.0;
                fv[1][3] = 4.0;
                fv[2][3] = 6.0;
                fv[3][3] = 8.0;
                fv[4][3] = 9.0;
                fv[5][3] = 10.0;
                fv[6][3] = 11.0;
                fv[7][3] = 13.0;
                fv[8][3] = 14.0;

                fv[0][4] = 4.0;
                fv[1][4] = 4.0;
                fv[2][4] = 5.0;
                fv[3][4] = 6.0;
                fv[4][4] = 7.0;
                fv[5][4] = 8.0;
                fv[6][4] = 9.0;
                fv[7][4] = 10.0;
                fv[8][4] = 11.0;

                fv[0][5] = 4.0;
                fv[1][5] = 4.0;
                fv[2][5] = 4.0;
                fv[3][5] = 5.0;
                fv[4][5] = 6.0;
                fv[5][5] = 7.0;
                fv[6][5] = 7.0;
                fv[7][5] = 8.0;
                fv[8][5] = 9.0;

                fv[0][6] = 4.0;
                fv[1][6] = 4.0;
                fv[2][6] = 4.0;
                fv[3][6] = 4.0;
                fv[4][6] = 5.0;
                fv[5][6] = 5.0;
                fv[6][6] = 6.0;
                fv[7][6] = 6.0;
                fv[8][6] = 7.0;

                break;
        }

        /* now create the function */

        if( flg EQ NL_YES )
            error = N_FitSrfFuncInterp( fv, nu, nv, 1, 1, uu, vv, sfn, SG );
        else
        {
            error = N_FitSrfFuncInterp( fv, nu, nv, 1, 1, uu, vv, sfn, &SL );
            kill = 1;
        }

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Evaluate the function to get the number of control points */

    error = N_SrfFuncEvalPt( sfn, ang, tol, NL_LEFT, NL_LEFT, &dd );

    if( error EQ NL_YES )
        NL_OUT;

    dd = ceil( dd );
    *nc = (NL_INDEX)dd;

    /* End NURBS and Exit */

    EXIT:
    if( kill EQ 1 )
        N_SFuncInitArrays( sfn );

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_ApproxCircArcWithCrvDATA: Perform nonrational curve fit to circle data             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation utility approximates a  given  NURBS  circle  or
     circular arc with a  non-rational  curve  of degree 2,3, or 4.  The 
     number of control points to be used must be passed  into  this rou-
     tine. If used in conjunction with N_CalcNumCPtsToApproxArc, this routine guarantees
     the error tolerance as described in N_CalcNumCPtsToApproxArc. A typical calling ex-
     ample is:

       NL_CURVE   cur;
       NL_DEGREE  deg;
       NL_INDEX   nc;
       NL_REAL    as, ae, rad;
       NL_POINT   cen, Ps, Pe;
       NL_VECTOR  X, Y;
       NL_STACKS  SG;
       ...
       (define cur, and get circle defining data);
       ...
       N_CrvInitArrays(&cur);
       N_ApproxCircArcWithCrvData(cen,X,Y,rad,as,ae,Ps,Pe,deg,nc,&cur,&SG);

     MEMORY TO STORE THE  OUTPUT NL_CURVE  cur  IS  ALLOCATED  INSIDE  THIS
     ROUTINE! 


   ACCESS:
   
     cen   , input  ,  Center of circle
     X,Y   , in/out ,  Local coordinate system of circle
     rad   , input  ,  Radius of circle
     as,ae , input  ,  Start and end angles of circle
     Ps,Pe , input  ,  Start and end points of circle  (provided  to en-
                       sure strict continuity with neighboring curves)
     deg   , input  ,  Degree of approximating non-rational circle (2,
                       3 or 4)
     nc    , input  ,  High index of control points for the fitting
     cur   , output ,  Approximating non-rational curve
     SG    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxCircArcWithCrvData( NL_POINT cen, NL_VECTOR X, NL_VECTOR Y, NL_REAL rad, NL_REAL as, NL_REAL ae, NL_POINT Ps, NL_POINT Pe, NL_DEGREE deg, NL_INDEX nc, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxCircArcWithCrvData");

    NL_FLAG error = NL_NO, flg;

    NL_INDEX closed, np, I[2], nk, ii, kk;

    NL_REAL *wp, wd[2], mag, theta, *up, *U, du, u, t;

    NL_POINT *P, Q, R;

    NL_CPOINT *Pw;

    NL_VECTOR D[2];

    NL_KNOTVECTOR knt;

    NL_LINESEG lsg1, lsg2;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors */

    if( deg LT 2 OR deg GT 4 )
        NL_ERROR( NL_INP_ERR );

    if( nc LT deg )
        NL_ERROR( NL_INP_ERR );

    if( rad LE 0.0 )
        NL_ERROR( NL_INP_ERR );

    /* Normalize axis vectors and convert angles */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_ERROR( NL_INP_ERR );
    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_ERROR( NL_INP_ERR );

    closed = 0;

    if( ae LE as )
        ae = 360.0 + ae;

    if( (ae - as)GT 359.9999 )
    {
        ae = as + 360.0;
        closed = 1;
    }

    theta = (ae - as) * (NL_PI / 180.0);
    as = as * (NL_PI / 180.0);
    ae = ae * (NL_PI / 180.0);

    /* Compute start/end first derivative vector */

    mag = rad * theta;

    N_Combine2Pts( -mag * sin( as ), X, mag * cos( as ), Y, &D[0] );

    if( closed )
        N_VectorCopy( D[0], &D[1] );
    else
        N_Combine2Pts( -mag *sin( ae ), X, mag *cos( ae ), Y, &D[1] );

    /* Now fit a nonrational curve. method depends on */
    /* degree and number of control points            */

    N_CrvInitArrays( cur );

    switch( deg )
    {
        case 2:

            N_CreateLineStartDirVector( &lsg1, Ps, D[0], NL_UNBOUNDED ); /* make tangent lines */
            N_CreateLineStartDirVector( &lsg2, Pe, D[1], NL_UNBOUNDED );

            if( nc LT 3 )
            { /* Can't least squares fit; construct parabolic arc */
                error = N_AllocCrvArrays( cur, 2, 2, 5, SG );

                if( error EQ NL_YES )
                    NL_OUT;
                N_CrvGetCPtsAndKnots( cur, &Pw, &U );

                for ( ii = 0; ii <= 2; ii++ )
                {
                    U[ii] = 0.0;
                    U[ii + 3] = 1.0;
                }

                N_Weight( Ps, NL_NOW, &Pw[0] );
                N_Weight( Pe, NL_NOW, &Pw[2] );

                error = N_IsectLineLine( lsg1, lsg2, &R, &t, &u, &flg );

                if( error EQ NL_YES OR flg EQ NL_FALSE )
                    NL_OUT;

                N_Weight( R, NL_NOW, &Pw[1] );
            }
            else
            { /* Least squares fit (without end tangent constraints) */
                nk = nc + 3;
                t = 4.4 *nc;
                np = (NL_INDEX)t;

                P = N_AllocPt1dArray( np, &SL );

                if( P EQ NULL )
                    NL_QUIT;

                du = theta / np;
                N_CopyPt( Ps, &P[0] );
                N_CopyPt( Pe, &P[np] );

                for ( ii = 1; ii < np; ii++ )
                {
                    u = as + ii * du;
                    N_TranslateSum2Pts( cen, rad * cos( u ), X, rad * sin( u ), Y, &P[ii] );
                }

                error = N_FitCrvApproxLstSq( P, np, nc, 2, NL_UNIFORM, cur, SG );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Now adjust end tangents */

                N_CrvGetCPts( cur, &kk, &Pw );

                N_CPtToPtEuclid( Pw[1], &Q );
                error = N_ProjectPtLine( lsg1, Q, &R, &t, &flg );
                N_Weight( R, NL_NOW, &Pw[1] );

                N_CPtToPtEuclid( Pw[kk - 1], &Q );
                error = N_ProjectPtLine( lsg2, Q, &R, &t, &flg );
                N_Weight( R, NL_NOW, &Pw[kk - 1] );
            }

            break;

        case 3:
            if( nc LT 4 )
            { /* Can't least squares fit; use interpolation */
                P = N_AllocPt1dArray( 1, &SL );

                if( P EQ NULL )
                    NL_QUIT;

                N_CopyPt( Ps, &P[0] );
                N_CopyPt( Pe, &P[1] );

                error = N_FitCrvDerivs( P, 1, 3, D[0], D[1], NL_UNIFORM, cur, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            { /* Least squares fit */
                nk = nc + 4;
                np = 4 * nc;

                U = N_AllocReal1dArray( nk, &SL );

                if( U EQ NULL )
                    NL_QUIT;

                for ( ii = 0; ii <= 3; ii++ )
                {
                    U[ii] = 0.0;
                    U[nk - ii] = 1.0;
                }

                kk = nk - 8;

                if( kk >= 0 )
                {
                    du = 1.0 / ((NL_REAL)kk + (NL_REAL)2);

                    for ( ii = 0; ii <= kk; ii++ )
                        U[ii + 4] = ((NL_REAL)ii + (NL_REAL)1) * du;
                }

                N_KnotVectorFromRealArray( &knt, U, nk );

                up = N_AllocReal1dArray( np, &SL );
                wp = N_AllocReal1dArray( np, &SL );
                P = N_AllocPt1dArray( np, &SL );

                if( up EQ NULL OR wp EQ NULL OR P EQ NULL )
                    NL_QUIT;

                du = 1.0 / np;
                up[0] = 0.0;
                up[np] = 1.0;
                wp[0] = wp[np] = -1.0;
                N_CopyPt( Ps, &P[0] );
                N_CopyPt( Pe, &P[np] );

                for ( ii = 1; ii < np; ii++ )
                {
                    up[ii] = ii * du;
                    wp[ii] = 1.0;
                    u = as + up[ii] * theta;

                    N_TranslateSum2Pts( cen, rad * cos( u ), X, rad * sin( u ), Y, &P[ii] );
                }

                wd[0] = wd[1] = -1.0;
                I[0] = 0;
                I[1] = np;

                error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, np, (NL_VOID *)D, wd, I, 1, NL_EPOINT, up, &knt, nc, 3, cur, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            break;

        case 4:
            if( nc LT 5 )
            { /* Can't least squares fit; use interpolation */
                P = N_AllocPt1dArray( 2, &SL );

                if( P EQ NULL )
                    NL_QUIT;

                N_CopyPt( Ps, &P[0] );
                N_CopyPt( Pe, &P[2] );

                u = as + 0.5 *theta;
                N_TranslateSum2Pts( cen, rad * cos( u ), X, rad * sin( u ), Y, &P[1] );

                error = N_FitCrvDerivs( P, 2, 4, D[0], D[1], NL_UNIFORM, cur, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            { /* Least squares fit */
                nk = nc + 5;
                np = 4 * nc;

                U = N_AllocReal1dArray( nk, &SL );

                if( U EQ NULL )
                    NL_QUIT;

                for ( ii = 0; ii <= 4; ii++ )
                {
                    U[ii] = 0.0;
                    U[nk - ii] = 1.0;
                }

                kk = nk - 10;

                if( kk >= 0 )
                {
                    du = 1.0 / ((NL_REAL)kk + (NL_REAL)2);

                    for ( ii = 0; ii <= kk; ii++ )
                        U[ii + 5] = ((NL_REAL)ii + (NL_REAL)1) * du;
                }

                N_KnotVectorFromRealArray( &knt, U, nk );

                up = N_AllocReal1dArray( np, &SL );
                wp = N_AllocReal1dArray( np, &SL );
                P = N_AllocPt1dArray( np, &SL );

                if( up EQ NULL OR wp EQ NULL OR P EQ NULL )
                    NL_QUIT;

                du = 1.0 / np;
                up[0] = 0.0;
                up[np] = 1.0;
                wp[0] = wp[np] = -1.0;
                N_CopyPt( Ps, &P[0] );
                N_CopyPt( Pe, &P[np] );

                for ( ii = 1; ii < np; ii++ )
                {
                    up[ii] = ii * du;
                    wp[ii] = 1.0;
                    u = as + up[ii] * theta;

                    N_TranslateSum2Pts( cen, rad * cos( u ), X, rad * sin( u ), Y, &P[ii] );
                }

                wd[0] = wd[1] = -1.0;
                I[0] = 0;
                I[1] = np;

                error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)P, wp, np, (NL_VOID *)D, wd, I, 1, NL_EPOINT, up, &knt, nc, 4, cur, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            break;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_ApproxNurbsWithNonRatCrv: Approximate NURBS curve with non-rational curve          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine approximates a given NURBS curve with a
     non-rational  curve of selected degree. The method is global, i.e. 
     the  approximating curve  does not deviate from the original curve 
     more than the  tolerance  anywhere on the curve. A typical calling 
     example is:

       NL_CURVE   curP, curQ;
       NL_DEGREE  deg;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define curP, get tol and deg);
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxNurbsWithNonRatCrv(&curP,tol,deg,NL_INHERITED,NL_YES,&curQ,&SG);

     MEMORY TO STORE THE  OUTPUT NL_CURVE  curQ  IS ALLOCATED  INSIDE THE
     ROUTINE! 


   ACCESS:
   
     curP  , input  ,  NURBS curve
     tol   , input  ,  Error tolerance
     deg   , input  ,  Degree of approximating curve
     par   , input  ,  Parametrization flag:
                         NL_UNIFORM    : uniform parametrization
                         NL_CHORDLENGTH: chordlength parametrization
                         NL_CENTRIPETAL: centipetal parametrization
                         NL_INHERITED  : curP's parametrization (see note below)
     tan   , input  ,  Flag:
                         NL_YES: maintain end tangents
                         NL_NO : do not maintain end tangents
     curQ  , output ,  Approximating curve
     SG    , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR
   USER NOTE:
     An example for curves with fully multiple internal knots:

     The curve is really two Bezier segments and the 
     following knots would give the exact same curve:
        0  0  0  0
        1  1  1 
        2  2  2  2

     But so would these knots:
        0  0  0  0
        1  1  1 
        100  100  100  100

     The algorithm used in N_ApproxNurbsWithNonRatCrv determines the number 
     of points for each span and comes up with a u[i] array
     and evaluates the curve at each u[i] to get the array of 
     points P[i].

     These points are then interpolated to give curQ.
     If the par flag is set to NL_INHERITED, curQ interpolates 
     each P[i] at u[i] but this will give extremely poor results
     whenever the knot spacing is not related to the length
     of each Bezier span.  Suppose the length of the straight
     segment of your curve is 5 and the length of the curved
     segment is 1, if the knots were then
        0  0  0  0
        5  5  5 
        6  6  6  6  
     the results would be quite good.

    So, anytime a curve his repeated internal knots you should not use 
    par = NL_INHERITED without first doing the following:

    For curves with Bezier pieces (fully multiple internal knots)
    find domain of old curve
       NL_PARAMETER ul, ur;
       N_CrvGetParamBounds(&curP, &ul, &ur);

    reparam old curve by chordlength (first by Bezier span)
       N_SrfReparamMultKnots(&curP, tol, &curP, &S);

    reset new curve domain to original
       N_CrvReparam(&curP, ul, ur);

    Approximate NURBS curve with non-rational curve and desired degree
       error =  N_ApproxNurbsWithNonRatCrv(&curP,tol,2,NL_INHERITED,NL_YES, &curQ, &S);

   ***********************************************************************/

NL_FLAG N_ApproxNurbsWithNonRatCrv( NL_CURVE *curP, NL_REAL tol, NL_DEGREE deg, NL_FLAG par, NL_FLAG tan, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxNurbsWithNonRatCrv");

    NL_FLAG twod = NL_NO, error = NL_NO;

    NL_INDEX *nu, i, j, k, l, n, mp, mq, mu, der;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *u, Muu, ul, ur, uinc, f1, f2, del, num, gro, exp;

    NL_POINT *P, CD[2];

    NL_VECTOR *Vs, *Ve, Ts, Te;

    NL_KNOTVECTOR *knu;

    NL_INTERVAL I;

    NL_CURVE ** bez;

    NL_STACKS SL;

    NL_REAL pct = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

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
    f2 = (NL_REAL)deg;

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

        error = N_CrvGetMax2ndDeriv( bez[i], &Muu );

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
        error = N_CrvEval( curP, u[i], NL_LEFT, &P[i] );

        if( error EQ NL_YES )
            NL_OUT;
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

            error = N_CrvArcLength( curP, UP[0], UP[mp], pct, NL_RELATIVE, &del );

            if( error EQ NL_YES )
                NL_OUT;

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

/**********************************************************************/
/* N_ApproxNurbsWithCrvKnots: Curve approximation with knot vector passed in           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine approximates a given NURBS curve with a
     non-rational  curve of selected degree. The method is global, i.e. 
     the  approximating curve  does not deviate from the original curve 
     more than the  tolerance  anywhere on the curve. A knot vector may
     be passed in to select knots for approximation. A  typical calling 
     example is:

       NL_CURVE       curP, curQ;
       NL_DEGREE      deg;
       NL_REAL        tol;
       NL_KNOTVECTOR  *knt;
       NL_STACKS      SC, SK;
       ...
       (define curP, get tol, deg and knt);
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxNurbsWithCrvKnots(&curP,tol,deg,NL_YES,NL_TANGENT   ,NL_YES,&knt,&curQ,&SC,&SK);
       N_ApproxNurbsWithCrvKnots(&curP,tol,deg,NL_NO ,NL_DERIVATIVE,NL_NO ,NULL,&curQ,&SC,&SK);

     MEMORY TO  STORE THE  OUTPUT NL_CURVE  curQ  IS ALLOCATED  INSIDE THE
     ROUTINE! 


   ACCESS:
   
     curP  , input  ,  NURBS curve
     tol   , input  ,  Error tolerance
     deg   , input  ,  Degree of approximating curve
     tfl   , input  ,  Flag:
                         NL_YES: maintain end tangents
                         NL_NO : do not maintain end tangents
     dfl   , input  ,  Flag:
                         NL_TANGENT   : maintain  end  tangent  directions
                                     only. Rescale magnitudes.
                         NL_DERIVATIVE: maintain end derivatives
     ffl   , input  ,  Flag:
                         NL_YES: evaluate sample points, approximate them,
                              then do knot removal.  This option is re-
                              commended  if  the  curvature  along  the 
                              curve does not  change  alot  (relatively
                              smooth and fair curve)
                         NL_NO : evaluate sample points, interpolate them,
                              then do knot removal.  This option is re-
                              commended  if  the  curvature  along  the 
                              curve changes alot 
     knt   , in/out ,  Knot vector
                         !NULL: choose the knots  from this knot vector 
                                if possible.  If  in  certain positions  
                                new  knots are  required, add  these to 
                                knt.
                          NULL: compute knt internally
     curQ  , output ,  Approximating curve
     SC    , input  ,  curQ's stack
     SK    , input  ,  knt's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxNurbsWithCrvKnots( NL_CURVE *curP, NL_REAL tol, NL_DEGREE deg, NL_FLAG tfl, NL_FLAG dfl, NL_FLAG ffl, NL_KNOTVECTOR ** knt, NL_CURVE *curQ, NL_STACKS *SC, NL_STACKS *SK )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxNurbsWithCrvKnots");

    NL_FLAG twod = NL_NO, error = NL_NO;

    NL_INDEX *nu, i, j, k, l, n, mp, mu;

    NL_DEGREE p;

    NL_REAL *UP, *u, Muu, ul, ur, uinc, f1, f2, del, num, gro, exp;

    NL_POINT *P, CD[2];

    NL_VECTOR *Vs, *Ve, Ts, Te;

    NL_CURVE ** bez;

    NL_STACKS SL;

    NL_REAL pct = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

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
    f2 = (NL_REAL)deg;

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

        error = N_CrvGetMax2ndDeriv( bez[i], &Muu );

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
        error = N_CrvEval( curP, u[i], NL_LEFT, &P[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get end tangents */

    if( tfl EQ NL_YES )
    {
        /* Get end derivatives */

        error = N_CrvDerivs( curP, UP[0], NL_LEFT, 1, CD );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( CD[1], &Ts );

        error = N_CrvDerivs( curP, UP[mp], NL_LEFT, 1, CD );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( CD[1], &Te );

        if( dfl EQ NL_TANGENT )
        {
            /* Adjust magnitudes */

            error = N_CrvArcLength( curP, UP[0], UP[mp], pct, NL_RELATIVE, &del );

            if( error EQ NL_YES )
                NL_OUT;

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
    }
    else
    {
        Vs = NULL;
        Ve = NULL;
    }

    /* Approximate point set */

    N_CrvInitArrays( curQ );
    error = N_FitCrvApproxKnotsTol( P, mu, deg, Vs, Ve, NL_DERIVATIVE, tol, knt, ffl, curQ, SC, SK );

    if( error EQ NL_YES )
        NL_OUT;

    if( twod EQ NL_YES )
        N_Crv3dTo2d( curQ );

    /* Compact output curve */

    N_CrvGetArraySizes( curQ, &n, &i );

    if( n LT mu )
    {
        error = N_CrvCompress( curQ, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_ApproxG1CrvWithCrv: Approximate any NL_G1 Nlib curve with nonrational curve     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a NURBS curve, Q(v) , approxi-
     mating a NL_G1 continuous Nlib curve, C(u).  A global method is used,
     consisting of interpolation, least squares approximation, and knot
     removal.  Q(v) is at least NL_C1 continuous and nonrational.  Various
     options exist for the parameterization of Q(v). Uses of this func-
     tion  include  approximate  degree  reduction  and  conversion  of
     rational to nonrational. A typical calling example is as follows:

       NL_DEGREE     p;
       NL_REAL       Eg, Ep, ptol;
       NL_CURVE      curQ;
       NL_STACKS     SQ;
       ...
       (define function:  NL_FLAG  evnF (NL_PARAMETER u, NL_PARAMETER *v);
       (choose p, Eg, Ep, and ptol);
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxG1CrvWithCrv(curP,p,NL_NO,NL_FUNCTION,evnF,Eg,Ep,ptol,&curQ,&SQ);

       curQ must be initialized to NULL for this function.


   ACCESS:
   
     curP , input  ,  Curve to be approximated. Assumed to be NL_G1
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
                      to bound the perpendicular distance  from C(u) to
                      Q(v) to not be greater than Eg
     Ep   , input  ,  Parametric error tolerance.  For  par = NL_INHERITED
                      and  par = NL_FUNCTION ,  this  function  attemps to 
                      maintain |C(u)-Q(v)| <= Ep  for  corresponding  u
                      and v.  Ep is also used for the par = NL_CHORDLENGTH
                      and par = NL_CENTRIPETAL cases, but it's  meaning is
                      less precise. 
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

NL_FLAG N_ApproxG1CrvWithCrv( NL_CURVE *curP, NL_DEGREE p, NL_FLAG tans, NL_FLAG par, NL_FLAG(*evnF)( NL_PARAMETER, NL_PARAMETER * ), NL_REAL Eg, NL_REAL Ep, NL_REAL ptol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxG1CrvWithCrv");

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

    /* Assign pointer to global */

    gloC = curP;

    /* Evaluate a set of sample points from input curve */

    error = N_GetPtsForCrvApprox( curP, ST_ApproxNca, par, Eg, ptol, &P, &m, &up, &uq, &SL );

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

    error = N_ApproxProcCrvWithCrvFit( P, m, ST_ApproxNca, Ts, Te, up, par, uq, evnF, p, Eg, Ep, ptol, curQ, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_GetPtsForCrvApprox: Get points for approximation of Nurbs-based curves       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a set of sample points  to  be
     used for the approximation of a NL_G1  continuous Nurbs  curve  or  a
     curve which is derived from a base curve which is  in  Nurbs  form
     (such as the offset of a Nurbs curve).  Denote  the  curve  to  be
     approximated by C(u). It is assumed that C(u)  is  NL_G1  continuous.
     The approximation of C(u), denoted by  Q(v), is NL_C1  continuous and
     nonrational. A typical calling example is as follows:

       NL_CURVE      cur;
       NL_PARAMETER  *up, *uq;
       NL_REAL       Eg, ptol;
       NL_POINT      *P;
       NL_INDEX      m;
       NL_STACKS     SG;
       ...
       (define function:  NL_FLAG  evnC (NL_PARAMETER u, NL_POINT *Q);
       (define the base Nurbs curve: cur);
       (choose p, Eg, and ptol);
       ...
       N_GetPtsForCrvApprox(&cur,evnC,NL_FUNCTION,Eg,ptol,&P,&m,&up,&uq,&SG);


   ACCESS:
   
     cur  , input  ,  The base curve of C(u)
     evnC , input  ,  Function which  evaluates  a  point on the curve,
                      C(u)
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(u)
                       NL_FUNCTION   : parameterization defined by  funct-
                                    ion
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from C(u) to
                      Q(v) to not be greater than Eg
     ptol , input  ,  Point coincidence tolerance.  Two points are con-
                      sidered to be equal if the  distance between them
                      is less than or equal to ptol.  ptol < Eg  should
                      hold
     P    , output ,  Points computed on C(u).  P  is allocated in this
                      function
     m    , output ,  Highest index in P, up, and uq arrays
     up   , output ,  Parameters of C(u) corresponding to the points in
                      array P. up is allocated in this function
     uq   , output ,  If par = NL_CHORDLENGTH or par = NL_CENTRIPETAL ,  then
                      uq is  the  array  of parameters corresponding to
                      the points in P. Otherwise, uq is not allocated
     SG   , input  ,  Memory stack for P, up, and uq


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_GetPtsForCrvApprox( NL_CURVE *cur, NL_FLAG(*evnC)( NL_PARAMETER, NL_POINT * ), NL_FLAG par, NL_REAL Eg, NL_REAL ptol, NL_POINT ** P, NL_INDEX *m, NL_PARAMETER ** up, NL_PARAMETER ** uq, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, jj, kk, np1, np2, k1, k2, k3, k4, vjumps, bet, toss, mb;

    NL_DEGREE pb;

    NL_PARAMETER u, *up1, *up2, *uqq=NULL;

    NL_REAL *Ub, cosa, mincos, du, length, d1 = 0.0, d2, d3, v1 = 0.0, v2;

    NL_KNOTVECTOR *kntb;

    NL_BOOLEAN sseg;

    NL_POINT *PP1, *PP2;

    NL_VECTOR VV[2];

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Compute an initial sample of points and analyze curve     */
    /* complexity. Use points from each knot span of base curve. */

    N_CrvGetDegree( cur, &pb );
    N_CrvGetKnotVector( cur, &kntb );
    N_KnotVectorGetKnots( kntb, &mb, &Ub );
    N_BasisGetSpanCount( kntb, pb, &k1 ); /* k1 is number of knot spans */

    k2 = 1;                               /* number of initial sample points interior to a span */

    if( k1 LT 25 )
    {
        if( k1 GE 10 )
            k2 = 3;
        else
        {
            k2 = 60 / k1;

            if( k2 % 2 EQ 0 )
                k2 = k2 - 1;
        }
    }

    np1 = k1 * (k2 + 1);
    PP1 = N_AllocPt1dArray( np1, &SL );

    if( PP1 EQ NULL )
        NL_QUIT;
    up1 = N_AllocReal1dArray( np1, &SL );

    if( up1 EQ NULL )
        NL_QUIT;

    /* Loop and compute points, parameters, and several complexity measures */

    k3 = 1;
    k4 = 0;
    length = 0.0;
    mincos = 1.0;    /* minimum turning cosine */
    sseg = NL_FALSE; /* possible straight segment in curve */
    vjumps = 0;      /* number of big velocity jumps */
    kk = 0;

    j = pb;
    up1[0] = Ub[j];
    error = evnC( up1[0], &PP1[0] );

    if( error EQ NL_YES )
        NL_OUT;
    np1 = 1;
    k2 += 1;

    for ( i = 0; i < k1; i++ )
    {
        du = (Ub[j + 1] - Ub[j]) / ((NL_REAL)k2 + (NL_REAL)1);

        for ( k = 1; k <= k2; k++ )
        {
            if( k LT k2 )
                up1[np1] = Ub[j] + k * du;
            else
                up1[np1] = Ub[j + 1];
            error = evnC( up1[np1], &PP1[np1] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDir( PP1[np1 - 1], PP1[np1], &VV[k4] );
            N_VectorMagnitude( VV[k4], &d2 );
            length += d2;
            v2 = d2 / du;

            if( np1 GT 1 AND d1 *d2 GT ptol )
            {
                if( 7.5 *v1 LT v2 )
                    vjumps += 1;

                else if( 7.5 *v2 LT v1 )
                    vjumps += 1;
                N_VectorDot( VV[0], VV[1], &d3 );
                cosa = d3 / (d1 * d2);

                if( cosa LT mincos )
                    mincos = cosa;

                if( (NOT sseg)AND cosa GT 0.9999 )
                {
                    if( kk EQ 0 )
                        kk = 1;
                    else
                        sseg = NL_TRUE;
                }
                else
                    kk = 0;
            }
            else
                kk = 0;

            jj = k3;
            k3 = k4;
            k4 = jj;
            d1 = d2;
            v1 = v2;
            np1 += 1;
        }

        if( i LT k1 - 1 )
        {
            j += 1;

            while( Ub[j]EQ Ub[j + 1]AND j LT mb - 1 )
                j += 1;
        }
    }

    np1 -= 1;

    /* Handle the degenerate case */

    if( length LE ptol )
    {
        np2 = 2; /* pass back 3 points */
        PP2 = N_AllocPt1dArray( np2, SG );

        if( PP2 EQ NULL )
            NL_QUIT;
        up2 = N_AllocReal1dArray( np2, SG );

        if( up2 EQ NULL )
            NL_QUIT;

        if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
        {
            *uq = N_AllocReal1dArray( np2, SG );

            if( *uq EQ NULL )
                NL_QUIT;
            ( *uq)[0] = 0.0;
            ( *uq)[1] = 0.5;
            ( *uq)[2] = 1.0;
        }

        up2[0] = up1[0];
        N_CopyPt( PP1[0], &PP2[0] );
        up2[2] = up1[np1];
        N_CopyPt( PP1[np1], &PP2[2] );
        i = np1 / 2;
        up2[1] = up1[i];
        N_CopyPt( PP1[i], &PP2[1] );

        *m = np2;
        *P = PP2;
        *up = up2;
        NL_OUT;
    }

    /* Now the general case. We insert more points between */
    /* the existing ones. First determine how many.        */

    bet = -1; /* number of additional points between existing */

    if( length LT 10.0 *Eg )
        bet = 0;
    else if( length LT 100.0 *Eg )
    {
        bet = 1;

        if( mincos LT 0.9975 AND np1 LT 50 )
            if( sseg OR vjumps GT 0 )
                bet = 2;
    }

    if( bet EQ - 1 )
    {
        d1 = 4.0 *Eg;
        d2 = 1.0 - 2.0 *NL_PI * d1 / length; /* based on circle */

        if( d2 LT - 1.0 )
            d2 = -1.0;
        d3 = acos( d2 );

        if( 2.0 *NL_PI GT 1400.0 *d3 )
            k = 1400;                            /* roughly total number of */
        else
            k = (NL_INDEX)(2.0 *NL_PI / d3) + 2; /* points */

        bet = (k - np1) / np1 + 1;

        if( bet LT 2 )
            bet = 2;

        k = vjumps; /* add points for possible complexities */

        if( mincos LE 0.9999 )
        {
            if( sseg )
                k += 3;

            if( mincos LT 0.55 )
                k += 3;

            else if( mincos LT 0.75 )
                k += 1;
        }

        if( k GT 0 )
        {
            k = k + (k % 2);

            if( k GT 10 )
                k = 10;

            if( bet LT 10 )
                bet += k;
            else
                bet += (3 * k / 2);
            d3 = 50.0 / np1;
            bet = (NL_INDEX)(bet * d3);
        }
    }

    /* Now compute points and parameters. */
    k = np1 * (bet + 1) + 1;
    PP2 = N_AllocPt1dArray( k, SG );

    if( PP2 EQ NULL )
        NL_QUIT;
    up2 = N_AllocReal1dArray( k, SG );

    if( up2 EQ NULL )
        NL_QUIT

    if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
    {
        uqq = N_AllocReal1dArray( k, SG );

        if( uqq EQ NULL )
            NL_QUIT;
        uqq[0] = 0.0;
        *uq = uqq;
    }

    length = 0.0;
    np2 = 0;
    up2[0] = up1[0];
    N_CopyPt( PP1[0], &PP2[0] );

    for ( i = 1; i <= np1; i++ )
    {
        u = up1[i - 1];
        du = (up1[i] - u) / ((NL_REAL)bet + (NL_REAL)1);
        toss = 0;

        for ( j = 1; j <= bet; j++ )
        {
            np2 += 1;
            up2[np2] = u + j * du;
            error = evnC( up2[np2], &PP2[np2] );

            if( error EQ NL_YES )
                NL_OUT;
            N_DistPtPt( PP2[np2 - 1], PP2[np2], &d1 );

            if( d1 LE ptol )
            {
                np2 -= 1;
                toss += 1;
            }
            else if( par EQ NL_CHORDLENGTH )
            {
                length += d1;
                uqq[np2] = length;
            }
        }
        np2 += 1;
        up2[np2] = up1[i];
        error = evnC( up2[np2], &PP2[np2] );

        if( error EQ NL_YES )
            NL_OUT;
        N_DistPtPt( PP2[np2 - 1], PP2[np2], &d1 );

        if( d1 LE ptol AND bet - toss GT 0 )
        {
            up2[np2 - 1] = up2[np2];
            N_CopyPt( PP2[np2], &PP2[np2 - 1] );
            np2 -= 1;
            N_DistPtPt( PP2[np2 - 1], PP2[np2], &d1 );
        }

        if( par EQ NL_CHORDLENGTH )
        {
            length += d1;
            uqq[np2] = length;
        }
    }

    if( par EQ NL_CENTRIPETAL )
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)PP2, np2, NL_EPOINT, par, uqq );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else if( par EQ NL_CHORDLENGTH )
    {
        uqq[np2] = 1.0;

        for ( i = 1; i < np2; i++ )
            uqq[i] = uqq[i] / length;
    }

    *m = np2;
    *P = PP2;
    *up = up2;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_ApproxProcCrvWithCrv: Approximate procedural curve with error bound specified  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a NURBS curve, Q(v) , approxi-
     mating a procedurally defined, NL_G1 continuous curve, C(u). A global
     method is used, consisting of interpolation, least squares approx-
     imation, and knot removal. Q(v) is at least NL_C1 continuous and non-
     rational. Various options exist for the parameterization of  Q(v).
     A typical calling example is as follows:

       NL_PARAMETER  u0, u1;
       NL_VECTOR     Ts, Te;
       NL_DEGREE     p;
       NL_REAL       Eg, Ep, ptol;
       NL_CURVE      curQ;
       NL_STACKS     SQ;
       ...
       (define function:  NL_FLAG  evnC (NL_PARAMETER u, NL_POINT *Q);
       (define function:  NL_FLAG  evnF (NL_PARAMETER u, NL_PARAMETER *v);
       (choose u0, u1, p, Eg, Ep, and ptol);
       (optionally, choose Ts, Te)
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxProcCrvWithCrv(evnC,u0,u1,Ts,Te,p,NL_FUNCTION,evnF,Eg,Ep,ptol,&curQ,&SQ);

       curQ must be initialized to NULL for this function.


   ACCESS:
   
     evnC , input  ,  Function which  evaluates  a  point on the curve,
                      C(u)
     u0   , input  ,  Start parameter. The segment [u0,u1] of C(u) will
                      be approximated
     u1   , input  ,  End parameter
     Ts   , input  ,  Optional tangent direction vector at C(u0).
     Te   , input  ,  Optional tangent direction vector at C(u1).
                      Ts and Te are specified by non-NULL  pointers  to
                      vectors. Either both Ts and Te must be  non-NULL,
                      or they must both be NULL (no end tangents speci-
                      fied
     p    , input  ,  Degree of  the  approximating curve, Q(v).  p > 1
                      must hold. p = 3 is recommended
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(u)
                       NL_FUNCTION   : parameterization defined by evnF
     evnF , input  ,  Reparameterization function, v = f(u). Used  only
                      if par = NL_FUNCTION
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from C(u) to
                      Q(v) to not be greater than Eg
     Ep   , input  ,  Parametric error tolerance.  For  par = NL_INHERITED
                      and  par = NL_FUNCTION ,  this  function  attemps to 
                      maintain |C(u)-Q(v)| <= Ep  for  corresponding  u
                      and v.  Ep is also used for the par = NL_CHORDLENGTH
                      and par = NL_CENTRIPETAL cases, but it's  meaning is
                      less precise. 
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

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_ApproxProcCrvWithCrv( NL_FLAG(*evnC)( NL_PARAMETER, NL_POINT * ), NL_PARAMETER u0, NL_PARAMETER u1, NL_VECTOR *Ts, NL_VECTOR *Te, NL_DEGREE p, NL_FLAG par, NL_FLAG(*evnF)( NL_PARAMETER, NL_PARAMETER * ), NL_REAL Eg, NL_REAL Ep, NL_REAL ptol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxProcCrvWithCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX m;

    NL_PARAMETER *up, *uq;

    NL_POINT *P;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvAreArraysNULL( curQ ) )
        NL_ERROR( NL_INP_ERR );

    if( u0 GE u1 )
        NL_ERROR( NL_INP_ERR );

    /* Evaluate a set of sample points from input curve */

    error = N_GetPtsForCrvApproxProc( evnC, u0, u1, p, par, Eg, ptol, &P, &m, &up, &uq, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now fit the point set with a NL_C1 Nurbs curve */

    error = N_ApproxProcCrvWithCrvFit( P, m, evnC, Ts, Te, up, par, uq, evnF, p, Eg, Ep, ptol, curQ, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_GetPtsForCrvApproxPROC: Get points for approximation of procedural curve         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a set of sample points  to  be
     used for the approximation of a  procedurally  defined, NL_G1 contin-
     uous curve, C(u),  by a NL_C1 continuous nonrational curve,  Q(v).  A
     typical calling example is as follows:

       NL_PARAMETER  u0, u1, *up, *uq;
       NL_DEGREE     p;
       NL_REAL       Eg, ptol;
       NL_POINT      *P;
       NL_INDEX      m;
       NL_STACKS     SG;
       ...
       (define function:  NL_FLAG  evnC (NL_PARAMETER u, NL_POINT *Q);
       (choose u0, u1, p, Eg, and ptol);
       ...
       N_GetPtsForCrvApproxProc(evnC,u0,u1,p,NL_FUNCTION,Eg,ptol,&P,&m,&up,&uq,&SG);


   ACCESS:
   
     evnC , input  ,  Function which  evaluates  a  point on the curve,
                      C(u)
     u0   , input  ,  Start parameter. The segment [u0,u1] of C(u) will
                      be approximated
     u1   , input  ,  End parameter
     p    , input  ,  Degree of  the  approximating curve, Q(v).  p > 1
                      must hold. p = 3 is recommended
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(u)
                       NL_FUNCTION   : parameterization defined by  funct-
                                    ion
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from C(u) to
                      Q(v) to not be greater than Eg
     ptol , input  ,  Point coincidence tolerance.  Two points are con-
                      sidered to be equal if the  distance between them
                      is less than or equal to ptol.  ptol < Eg  should
                      hold
     P    , output ,  Points computed on C(u).  P  is allocated in this
                      function
     m    , output ,  Highest index in P, up, and uq arrays
     up   , output ,  Parameters of C(u) corresponding to the points in
                      array P. up is allocated in this function
     uq   , output ,  If par = NL_CHORDLENGTH or par = NL_CENTRIPETAL ,  then
                      uq is  the  array  of parameters corresponding to
                      the points in P. Otherwise, uq is not allocated
     SG   , input  ,  Memory stack for P, up, and uq


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_GetPtsForCrvApproxProc( NL_FLAG(*evnC)( NL_PARAMETER, NL_POINT * ), NL_PARAMETER u0, NL_PARAMETER u1, NL_DEGREE p, NL_FLAG par, NL_REAL Eg, NL_REAL ptol, NL_POINT ** P, NL_INDEX *m, NL_PARAMETER ** up, NL_PARAMETER ** uq, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, np1, np2, k1, k2, vjumps, bet, toss;

    NL_PARAMETER u, *up1, *up2, *uqq=NULL;

    NL_REAL cosa, mincos, du, length, d1, d2, d3;

    NL_BOOLEAN sseg;

    NL_POINT *PP1, *PP2;

    NL_VECTOR VV[2];

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Compute an initial sample of points and analyze curve complexity */

    np1 = NL_MAX( 40, 2 * (p + 1) );
    PP1 = N_AllocPt1dArray( np1, &SL );

    if( PP1 EQ NULL )
        NL_QUIT;
    up1 = N_AllocReal1dArray( np1, &SL );

    if( up1 EQ NULL )
        NL_QUIT;

    /* Loop and compute points, parameters, and several complexity measures */

    k1 = 0;
    k2 = 1;
    length = 0.0;
    du = (u1 - u0) / np1;
    mincos = 1.0;    /* minimum turning cosine */
    sseg = NL_FALSE; /* possible straight segment in curve */
    vjumps = 0;      /* number of big velocity jumps */
    k = 0;
    d1 = d2 = d3 = 0.0;

    for ( i = 0; i <= np1; i++ )
    {
        if( i LT np1 )
            up1[i] = u0 + i * du;
        else
            up1[i] = u1;
        error = evnC( up1[i], &PP1[i] );

        if( error EQ NL_YES )
            NL_OUT;

        if( i GT 0 )
        {
            N_VectorDir( PP1[i - 1], PP1[i], &VV[k2] );
            N_VectorMagnitude( VV[k2], &d2 );
            length += d2;

            if( i GT 1 AND d1 *d2 GT ptol )
            {
                if( 8.0 *d1 LT d2 )
                    vjumps += 1;

                else if( 8.0 *d2 LT d1 )
                    vjumps += 1;
                N_VectorDot( VV[0], VV[1], &d3 );
                cosa = d3 / (d1 * d2);

                if( cosa LT mincos )
                    mincos = cosa;

                if( (NOT sseg)AND cosa GT 0.9999 )
                {
                    if( k EQ 0 )
                        k = 1;
                    else
                        sseg = NL_TRUE;
                }
                else
                    k = 0;
            }
            else
                k = 0;
        }
        j = k1;
        k1 = k2;
        k2 = j;
        d1 = d2;
    }

    /* Handle the degenerate case */

    if( length LE ptol )
    {
        np2 = 2; /* pass back 3 points */
        PP2 = N_AllocPt1dArray( np2, SG );

        if( PP2 EQ NULL )
            NL_QUIT;
        up2 = N_AllocReal1dArray( np2, SG );

        if( up2 EQ NULL )
            NL_QUIT;

        if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
        {
            *uq = N_AllocReal1dArray( np2, SG );

            if( *uq EQ NULL )
                NL_QUIT;
            ( *uq)[0] = 0.0;
            ( *uq)[1] = 0.5;
            ( *uq)[2] = 1.0;
        }

        up2[0] = up1[0];
        N_CopyPt( PP1[0], &PP2[0] );
        up2[2] = up1[np1];
        N_CopyPt( PP1[np1], &PP2[2] );
        i = np1 / 2;
        up2[1] = up1[i];
        N_CopyPt( PP1[i], &PP2[1] );

        *m = np2;
        *P = PP2;
        *up = up2;
        NL_OUT;
    }

    /* Now the general case. We insert more points between */
    /* the existing ones. First determine how many.        */

    bet = -1; /* number of additional points between existing */

    if( length LT 10.0 *Eg )
        bet = 0;
    else if( length LT 100.0 *Eg )
    {
        bet = 1;

        if( mincos LT 0.9975 )
            if( sseg OR vjumps GT 0 )
                bet = 2;
    }

    if( bet EQ - 1 )
    {
        d1 = 4.0 *Eg;
        d2 = 1.0 - 2.0 *NL_PI * d1 / length; /* based on circle */

        if( d2 LT - 1.0 )
            d2 = -1.0;
        d3 = acos( d2 );

        if( 2.0 *NL_PI GT 1400.0 *d3 )
            k = 1400;                            /* roughly total number of */
        else
            k = (NL_INDEX)(2.0 *NL_PI / d3) + 2; /* points */

        bet = (k - np1) / np1 + 1;

        if( bet LT 2 )
            bet = 2;

        k = vjumps; /* add points for possible complexities */

        if( mincos LE 0.9999 )
        {
            if( sseg )
                k += 3;

            if( mincos LT 0.50 )
                k += 3;

            else if( mincos LT 0.70 )
                k += 1;
        }

        if( k GT 0 )
        {
            k = k + (k % 2);

            if( k GT 10 )
                k = 10;

            if( bet LT 10 )
                bet += k;
            else
                bet += (3 * k / 2);
        }
    }

    /* Now compute points and parameters. */

    k = np1 * (bet + 1) + 1;
    PP2 = N_AllocPt1dArray( k, SG );

    if( PP2 EQ NULL )
        NL_QUIT;
    up2 = N_AllocReal1dArray( k, SG );

    if( up2 EQ NULL )
        NL_QUIT

    if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
    {
        uqq = N_AllocReal1dArray( k, SG );

        if( uqq EQ NULL )
            NL_QUIT;
        uqq[0] = 0.0;
        *uq = uqq;
    }

    length = 0.0;
    np2 = 0;
    up2[0] = up1[0];
    N_CopyPt( PP1[0], &PP2[0] );

    for ( i = 1; i <= np1; i++ )
    {
        u = up1[i - 1];
        du = (up1[i] - u) / ((NL_REAL)bet + (NL_REAL)1);
        toss = 0;

        for ( j = 1; j <= bet; j++ )
        {
            np2 += 1;
            up2[np2] = u + j * du;
            error = evnC( up2[np2], &PP2[np2] );

            if( error EQ NL_YES )
                NL_OUT;
            N_DistPtPt( PP2[np2 - 1], PP2[np2], &d1 );

            if( d1 LE ptol )
            {
                np2 -= 1;
                toss += 1;
            }
            else if( par EQ NL_CHORDLENGTH )
            {
                length += d1;
                uqq[np2] = length;
            }
        }
        np2 += 1;
        up2[np2] = up1[i];
        error = evnC( up2[np2], &PP2[np2] );

        if( error EQ NL_YES )
            NL_OUT;
        N_DistPtPt( PP2[np2 - 1], PP2[np2], &d1 );

        if( d1 LE ptol AND bet - toss GT 0 )
        {
            up2[np2 - 1] = up2[np2];
            N_CopyPt( PP2[np2], &PP2[np2 - 1] );
            np2 -= 1;
            N_DistPtPt( PP2[np2 - 1], PP2[np2], &d1 );
        }

        if( par EQ NL_CHORDLENGTH )
        {
            length += d1;
            uqq[np2] = length;
        }
    }

    if( par EQ NL_CENTRIPETAL )
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)PP2, np2, NL_EPOINT, par, uqq );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else if( par EQ NL_CHORDLENGTH )
    {
        uqq[np2] = 1.0;

        for ( i = 1; i < np2; i++ )
            uqq[i] = uqq[i] / length;
    }

    *m = np2;
    *P = PP2;
    *up = up2;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_ApproxProcCrvWithCrvFIT: Fit procedural curve points with error bound specified   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a NURBS curve, Q(v),  approxi-
     mating a given set of points to a user given tolerance. The points
     are computed from a procedurally  defined,  G1  continuous  curve, 
     C(u). This function uses a global method, consisting of interpola-
     tion,  least  squares approximation, and knot removal.  Q(v) is at
     least C1 continuous and nonrational. A typical calling example  is 
     as follows:

       NL_POINT      *P;
       NL_INDEX      m;
       NL_VECTOR     Ts, Te;
       NL_PARAMETER  *up, *uq;
       NL_DEGREE     p;
       NL_REAL       Eg, Ep, ptol;
       NL_CURVE      curQ;
       NL_STACKS     SQ;
       ...
       (define function:  NL_FLAG  evnC (NL_PARAMETER u, NL_POINT *Q);
       (define function:  NL_FLAG  evnF (NL_PARAMETER u, NL_PARAMETER *v);
       (get arrays P and up, and choose p, Eg, Ep, and ptol);
       (optionally, choose Ts, Te)
       ...
       N_CrvInitArrays(&curQ);
       N_ApproxProcCrvWithCrvFit(P,m,evnC,Ts,Te,up,FUNCTION,NULL,evnF,p,Eg,Ep,ptol,
                &curQ,&SQ);

     curQ MUST be initialized to NULL for this function.


   ACCESS:
   
     P    , input  ,  Point array to be approximated
     m    , input  ,  Highest index in P, up, and uq. m > 1 must hold
     evnC , input  ,  Function which  evaluates  a  point on the curve,
                      C(u)
     Ts   , input  ,  Optional tangent direction vector at first  point
                      in array P
     Te   , input  ,  Optional tangent direction vector at  last  point
                      in array P
                      Ts and Te are specified by non-NULL  pointers  to
                      vectors. Either both Ts and Te must be  non-NULL,
                      or they must both be NULL (no end tangents speci-
                      fied
     up   , input  ,  Parameters of C(u) corresponding to the points in
                      array P
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(u)
                       FUNCTION   : parameterization defined by evnF
     uq   , input  ,  If par = NL_CHORDLENGTH or par = NL_CENTRIPETAL ,  then
                      uq is the array  of  parameters  corresponding to
                      the points in P. Otherwise, uq is not used
     evnF , input  ,  Reparameterization function, v = f(u). Used  only
                      if par == FUNCTION
     p    , input  ,  Degree of  the  approximating curve, Q(v).  p > 1
                      must hold. p = 3 is recommended
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from C(u) to
                      Q(v) to not be greater than Eg
     Ep   , input  ,  Parametric error tolerance.  For  par = NL_INHERITED
                      and  par = FUNCTION ,  this  function  attemps to 
                      maintain |C(u)-Q(v)| <= Ep  for  corresponding  u
                      and v.  Ep is also used for the par = NL_CHORDLENGTH
                      and par = NL_CENTRIPETAL cases, but it's  meaning is
                      less precise. 
     ptol , input  ,  Point coincidence tolerance.  Two points are con-
                      sidered to be equal if the  distance between them
                      is less than or equal to ptol.  ptol < Eg  should
                      hold
     curQ , output ,  Approximating curve
     SQ   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

// ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_ApproxProcCrvWithCrvFit( NL_POINT *P, NL_INDEX m, NL_FLAG(*evnC)( NL_PARAMETER, NL_POINT * ), NL_VECTOR *Ts, NL_VECTOR *Te, NL_PARAMETER *up, NL_FLAG par, NL_PARAMETER *uq, NL_FLAG(*evnF)( NL_PARAMETER, NL_PARAMETER * ), NL_DEGREE p, NL_REAL Eg, NL_REAL Ep, NL_REAL ptol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxProcCrvWithCrvFIT");

    NL_FLAG closed = NL_NO, tangents = NL_NO, error = NL_NO, dumf;

    NL_INDEX nn, mm, i, j, k, km, kn, jj, kk, np, I[2], k1, k2, k3, mmm;

    NL_INTEGER *pmk = NULL, save = NL_ITLIM;

    NL_REAL *U1, *U2, mintol, d1, d2, d3, *err, du, wd[2], *wp, top, toc, dv, Ebnd, ktol, *temp, maxtol;  /* bigerr unused */

    NL_CPOINT *Pw, Qw;

    NL_VECTOR Vs = { 0,0,0 }, Ve = { 0,0,0 }, V1, V2, D[2];

    NL_PARAMETER *up1, *uq1 = NULL, *uq2, u, v, ul, ur, umid;

    NL_KNOTVECTOR *knt1, *knt2, knt3;

    NL_DEGREE pp;

    NL_POINT *R1, *R2, Q;

    NL_CURVE cur1, cur2, *cur3;

    NL_LINESEG line;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( m LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Initialize some variables */

    NL_ITLIM = 4;
    top = ptol;
    toc = 1.e-6;
    ktol = 0.0001;

    /* Create degree 1 curve from input points */

    error = N_AllocCrvArrays( &cur1, m, 1, m + 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    N_CrvGetCPtsKnotVectorAndKnots( &cur1, &Pw, &knt1, &U1 );

    for ( i = 0; i <= m; i++ )
    {
        N_PtToCPt( P[i], &Pw[i] );
    }

    switch( par )
    {
        case NL_CHORDLENGTH:
        case NL_CENTRIPETAL:

            U1[0] = U1[1] = 0.0;
            U1[m + 1] = U1[m + 2] = 1.0;

            for ( i = 2; i <= m; i++ )
                U1[i] = uq[i - 1];
            break;

        case NL_INHERITED:

            U1[0] = U1[1] = up[0];
            U1[m + 1] = U1[m + 2] = up[m];

            for ( i = 2; i <= m; i++ )
                U1[i] = up[i - 1];
            break;

        case NL_FUNCTION:

            error = evnF( up[0], &U1[0] );

            if( error EQ NL_YES )
                NL_OUT;
            U1[1] = U1[0];
            error = evnF( up[m], &U1[m + 1] );

            if( error EQ NL_YES )
                NL_OUT;
            U1[m + 2] = U1[m + 1];

            for ( i = 2; i <= m; i++ )
            {
                error = evnF( up[i - 1], &U1[i] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            break;

        default:

            NL_ERROR( NL_INP_ERR );
    }

    /* Do knot removal on degree 1 curve */

    mintol = NL_MIN( Eg, Ep );
    /* bigerr = 10.0 *mintol; */

    N_CrvInitArrays( &cur2 );
    error = N_CrvRemoveAllKnots( &cur1, 60.0 *mintol, &cur2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Check for smooth closure, and get end derivatives if indicated */

    d1 = ptol + 1.0;

    if( Ts NEQ NULL AND Te NEQ NULL )
        tangents = NL_YES;
    else
        N_DistPtPt( P[0], P[m], &d1 );

    if( tangents OR( d1 LE ptol AND m GT 2 ) )
    {
        d1 = U1[2] - U1[1]; /* use 3-point Bessel to get end derivatives */
        d2 = U1[3] - U1[2];
        d3 = U1[3] - U1[1];
        N_VectorDir( P[0], P[1], &V1 );
        N_VectorDir( P[1], P[2], &V2 );
        N_VectorCombine( (d1 + d3) / (d1 * d3), V1, -d1 / (d2 * d3), V2, &Vs );
        d1 = U1[m + 1] - U1[m];
        d2 = U1[m] - U1[m - 1];
        d3 = U1[m + 1] - U1[m - 1];
        N_VectorDir( P[m - 1], P[m], &V1 );
        N_VectorDir( P[m - 2], P[m - 1], &V2 );
        N_VectorCombine( (d1 + d3) / (d1 * d3), V1, -d1 / (d2 * d3), V2, &Ve );

        N_VectorMagnitude( Vs, &d1 );
        N_VectorMagnitude( Ve, &d2 );

        if( tangents )
        { /* use magnitudes from Vs and Ve */
            N_VectorCopy( *Ts, &Vs );
            N_VectorCopy( *Te, &Ve );
            error = N_VectorNormalizeRef( &Vs );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_VectorNormalizeRef( &Ve );

            if( error EQ NL_YES )
                NL_OUT;
            N_VectorScaleRef( &Vs, d1, &Vs );
            N_VectorScaleRef( &Ve, d2, &Ve );
        }
        else
        {
            if( d1 GT ptol AND d2 GT ptol )
            {                        /* get cosine of angle between end derivatives */
                N_VectorDot( Vs, Ve, &d3 );
                d3 = d3 / (d1 * d2); /* cosine */

                if( d3 GT 0.9986 )   /* 3 degrees */
                {
                    closed = NL_YES;
                    N_VectorCombine( 0.5 / d1, Vs, 0.5 / d2, Ve, &V1 ); /* average unit direction */

                    N_VectorScale( V1, d1, &Vs );                       /* the end derivatives */
                    N_VectorScale( V1, d2, &Ve );
                }
            }
        }
        N_VectorCopy( Vs, &D[0] );
        N_VectorCopy( Ve, &D[1] );
    }

    /* Compute points for quadratic fit. Ensure that there are */
    /* at least np points in interior of each quadratic span. */

    N_CrvGetCPtsDegreeAndKnots( &cur2, &kn, &Pw, &pp, &km, &U2 );

    np = 3;
    nn = m + np * km; /* maximum size required */

    R1 = N_AllocPt1dArray( nn, &SL );

    if( R1 EQ NULL )
        NL_QUIT;
    up1 = N_AllocReal1dArray( nn, &SL );

    if( up1 EQ NULL )
        NL_QUIT;

    N_CopyPt( P[0], &R1[0] );
    up1[0] = up[0];
    nn = 0;
    j = 1;

    for ( i = 2; i < km; i++ )
    {
        k = j + 1;

        while( U1[k]LT U2[i] )
            k = k + 1;
        U2[i] = up[k - 1]; /* so I know where to put double knots below */

        if( k - j GT np )
        {
            for ( jj = j; jj < k - 1; jj++ ) /* copy points over */
            {
                nn = nn + 1;
                N_CopyPt( P[jj], &R1[nn] );
                up1[nn] = up[jj];
            }
        }
        else
        {
            du = (up[k - 1] - up[j - 1]) / ((NL_REAL)np + (NL_REAL)1); /* compute np new points */

            for ( jj = 1; jj <= np; jj++ )
            {
                nn = nn + 1;
                up1[nn] = up[j - 1] + jj * du;
                error = evnC( up1[nn], &R1[nn] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
        nn = nn + 1;
        N_CopyPt( P[k - 1], &R1[nn] );
        up1[nn] = up[k - 1];
        j = k;
    }

    /* There are nn+1 fit points. Now get new parameters for fit. */
    /* Also get error vector for knot removal. */

    err = N_AllocReal1dArray( nn, &SL );

    if( err EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        uq1 = N_AllocReal1dArray( nn, &SL );

        if( uq1 EQ NULL )
            NL_QUIT;
    }

    for ( i = 0; i <= nn; i++ )
        err[i] = 0.0;

    if( par EQ NL_INHERITED )
    {
        uq1 = up1;
    }
    else
    {
        if( nn EQ m )
        { /* same parameters as for degree 1 fit */
            for ( i = 0; i <= nn; i++ )
                uq1[i] = U1[i + 1];
        }
        else
        { /* compute new parameters */
            if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
            {
                error = N_FitCalcCrvParamValues( (NL_VOID *)R1, nn, NL_EPOINT, par, uq1 );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                for ( i = 0; i <= nn; i++ )
                {
                    error = evnF( up1[i], &uq1[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
    }

    mm = nn;

    if( closed OR tangents )
    {
        err[1] = err[nn - 1] = 0.1 *mintol;
        nn = nn + 2;
    }

    /* Kill current cur1 and prepare it for the quadratic fit */

    N_FreeCrv( &cur1, &SL );
    error = N_AllocCrvArrays( &cur1, nn, 2, nn + 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    N_CrvGetCPtsKnotVectorAndKnots( &cur1, &Pw, &knt1, &U1 );

    /* Now compute the knot vector. Put a double knot at each knot */
    /* remaining from the degree 1 curve after knot removal. Also  */
    /* insert a knot at each end if derivatives are called for.    */
    /* Also insert small error at double knots so that they are    */
    /* not real easy to remove.                                    */

    j = nn + 3;

    for ( i = 0; i < 3; i++ )
    {
        U1[i] = uq1[0];
        U1[j--] = uq1[mm];
    }

    km = 3; /* index in U1 */
    kn = 2; /* index in U2 */
    kk = 1; /* index in up1 and uq1 */

    if( closed OR tangents )
    {
        U1[3] = 0.5 *( uq1[0] + uq1[1] );
        U1[nn] = 0.5 *( uq1[mm] + uq1[mm - 1] );
        km = 4;
    }

    while( 1 )
    {
        k = kk;

        while( up1[k]LT U2[kn] )
            k = k + 1;
        k = k - 1;

        for ( i = kk; i < k; i++ )
            U1[km++] = 0.5 *( uq1[i] + uq1[i + 1] );
        k = k + 1;

        if( k GE mm )
            break;
        else
        {
            U1[km++] = uq1[k];
            U1[km++] = uq1[k];
            kk = k + 1;
            kn = kn + 1;
            err[k] = 0.15 *mintol;
        }
    }

    /* Now kill cur2 and do the quadratic fit */

    N_FreeCrv( &cur2, &SL );

    if( closed OR tangents )
    {
        error = N_FitCrvKnotsAndDerivs( (NL_VOID *)R1, mm, NL_EPOINT, uq1, knt1, 2, (NL_VOID *) &Vs, (NL_VOID *) &Ve, &cur1, &SL );
    }
    else
    {
        error = N_FitCrvInterpGivenParams( (NL_VOID *)R1, mm, NL_EPOINT, uq1, knt1, 2, &cur1, &SL );
    }

    if( error EQ NL_YES )
        NL_OUT;

    /* Now remove knots. If error = NO, then cur1 is adequate, */
    /* and we save it in cur3. */

    error = N_FitRemoveKnots( &cur1, uq1, err, mm, 0.95 *mintol );

    if( error EQ NL_YES )
        NL_OUT;
    cur3 = &cur1; /* saved curve */
    R2 = R1;
    uq2 = uq1;

    /* Least squares approximate */

    N_CrvInitArrays( &cur2 );
    error = NL_NO;

    if( p EQ 2 )
    {
        N_CrvGetKnotVector( &cur1, &knt1 );
        N_CrvGetArraySizes( &cur1, &nn, &kk );

        if( closed OR tangents )
        {
            wp = N_AllocReal1dArray( mm, &SL );

            if( wp EQ NULL )
                NL_QUIT;
            wp[0] = wp[mm] = -1.0;

            for ( i = 1; i < mm; i++ )
                wp[i] = 1.0;
            wd[0] = wd[1] = -1.0;
            I[0] = 0;
            I[1] = mm;

            error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)R1, wp, mm, (NL_VOID *)D, wd, I, 1, NL_EPOINT, uq1, knt1, nn, 2, &cur2, &SL );
        }
        else
        {
            error = N_FitCrvApproxKnots( (NL_VOID *)R1, mm, NL_EPOINT, uq1, knt1, nn, 2, &cur2, &SL );
        }

        if( error EQ  NL_NO)
        {
            N_FreeCrv( &cur1, &SL );
            cur3 = &cur2; /* save this one */
        }
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( cur3, &nn, &Pw, &pp, &km, &U1 );

        /* Build the p-th degree knot vector */

        U2 = N_AllocReal1dArray( km * (p - 2 + 1), &SL );

        if( U2 EQ NULL )
            NL_QUIT;
        i = j = 0;
        km = km - 2;

        while( 1 )
        {
            u = U1[i];

            do
            {
                U2[j++] = U1[i++];
            } while ( U1[i]EQ u );

            for ( k = 2; k < p; k++ )
                U2[j++] = u;

            if( i GE km )
            {
                for ( k = 0; k <= p; k++ )
                    U2[j++] = U1[km];
                km = j - 1; /* new high index of knots */
                break;
            }
        }
        N_KnotVectorFromRealArray( &knt3, U2, km );

        /* Build the new parameter values. May have to increase    */
        /* number of points. Want at least np points in each span. */

        np = p + 1;
        uq2 = N_AllocReal1dArray( mm + nn * np, &SL );

        if( uq2 EQ NULL )
            NL_QUIT;

        if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
        {
            pmk = N_AllocInt1dArray( km / 2, &SL ); /* to map old knots to new ones */

            if( pmk EQ NULL )
                NL_QUIT;
        }

        R2 = N_AllocPt1dArray( mm + nn * np, &SL ); /* compute points and parameters */

        if( R2 EQ NULL )
            NL_QUIT;
        N_CopyPt( R1[0], &R2[0] );
        uq2[0] = uq1[0];
        k1 = k2 = 1; /* indices into uq1 and uq2 */
        kk = p + 1;  /* index into U2 */
        k3 = -1;     /* index into pmk (location in uq2 of p-mult knots) */

        while( k1 LE mm )
        {
            for ( k = k1; k <= mm; k++ )
                if( uq1[k]GE U2[kk] )
                    break;
            jj = k - k1; /* number of parameters in interior of span */

            if( jj GE np )
            {
                NL_INDEX k1Start = k1;
                for ( k1 = k1Start; k1 < k; k1++ )
                {
                    uq2[k2] = uq1[k1];
                    N_CopyPt( R1[k1], &R2[k2++] );
                }
            }
            else
            {
                dv = (U2[kk] - U2[kk - 1]) / ((NL_REAL)np + (NL_REAL)1);
                j = k1 - 1;
                k1 = k;

                for ( i = 1; i <= np; i++ )
                {
                    v = U2[kk - 1] + i * dv;

                    while( 1 )
                    {
                        if( v GE uq1[j]AND v LE uq1[j + 1] )
                            break;
                        else
                            j = j + 1;
                    }
                    u = up1[j] + ((up1[j + 1] - up1[j]) / (uq1[j + 1] - uq1[j])) * (v - uq1[j]);
                    error = evnC( u, &R2[k2] );

                    if( error EQ NL_YES )
                        break;

                    if( par EQ NL_INHERITED )
                        uq2[k2] = u;
                    else if( par EQ NL_FUNCTION )
                    {
                        error = evnF( u, &uq2[k2] );

                        if( error EQ NL_YES )
                            break;
                    }

                    k2 = k2 + 1;
                }

                if( error EQ NL_YES )
                    break;
            }

            if( uq1[k1]EQ U2[kk] )
            {
                if( U2[kk]EQ U2[kk + p - 1] )
                    if( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL )
                        pmk[++k3] = k2;
                uq2[k2] = uq1[k1];
                N_CopyPt( R1[k1++], &R2[k2++] );
            }

            if( kk GE km - p OR U2[kk]GE uq1[mm] )
                break;

            while( U2[kk]EQ U2[kk + 1] )
                kk = kk + 1;
            kk = kk + 1;
        }

        mmm = mm;        /* save it */
        mm = k2 - 1;     /* new high index for number of points and parameters */
        nn = km - p - 1; /* new high index of control points */

        if( error EQ  NL_NO AND( par EQ NL_CHORDLENGTH OR par EQ NL_CENTRIPETAL ) )
        {
            error = N_FitCalcCrvParamValues( (NL_VOID *)R2, mm, NL_EPOINT, par, uq2 );

            if( error EQ  NL_NO AND k3 GE 0 )
            {              /* map old knot values to new knot values */
                pmk[++k3] = mm;
                k1 = p;    /* last knot of mult GE p */
                u = U2[p]; /* old knot value at U2[k1] */
                k2 = 0;    /* index in uq2 corresponding to k1 (new value) */
                k3 = 0;
                i = p + 1;

                while( i LE km )
                {
                    j = 1;

                    while( U2[i]EQ U2[i + j]AND i + j LT km )
                        j += 1;

                    if( j LT p )
                    {
                        i = i + j;
                        continue;
                    }

                    if( i + j EQ km )
                        j += 1;       /* j is multiplicity */
                    v = uq2[pmk[k3]]; /* new knot value at U2[i] */
                    d1 = (uq2[k2] - v) / (U2[i] - u);
                    u = U2[i];

                    for ( k = k1 + 1; k < i; k++ )
                        U2[k] = d1 * (u - U2[k]) + v;

                    for ( k = 0; k < j; k++ )
                        U2[i + k] = v;
                    k2 = pmk[k3];
                    k3 += 1;
                    i = i + j;
                    k1 = i - 1;
                }
            }
        }

        /* Ready now to least squares fit */

        if( error EQ  NL_NO)
        {
            if( closed OR tangents )
            {
                wp = N_AllocReal1dArray( mm, &SL );

                if( wp EQ NULL )
                    NL_QUIT;
                wp[0] = wp[mm] = -1.0;

                for ( i = 1; i < mm; i++ )
                    wp[i] = 1.0;
                wd[0] = wd[1] = -1.0;
                I[0] = 0;
                I[1] = mm;

                error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)R2, wp, mm, (NL_VOID *)D, wd, I, 1, NL_EPOINT, uq2, &knt3, nn, p, &cur2, &SL );
            }
            else
            {
                error = N_FitCrvApproxKnots( (NL_VOID *)R2, mm, NL_EPOINT, uq2, &knt3, nn, p, &cur2, &SL );
            }

            if( error EQ  NL_NO)
                cur3 = &cur2;
        }

        if( error EQ NL_YES )
        {
            mm = mmm;
            R2 = R1;
            uq2 = uq1;
        }
    }

    /* cur3 contains the current best approximation. uq2 and R2 */
    /* are the corresponding mm parameters and points. */

    err = N_AllocReal1dArray( mm, &SL );

    if( err EQ NULL )
        NL_QUIT;

    /* Compute errors */

    maxtol = NL_MAX( Eg, Ep );
    err[0] = err[mm] = 0.0;

    for ( i = 1; i < mm; i++ )
    {
        d1 = 0.0;
        error = N_CrvEval( cur3, uq2[i], NL_LEFT, &Q );

        if( error EQ NL_YES )
            NL_OUT;
        else
            N_DistPtPt( R2[i], Q, &d1 );
        error = N_CrvClosestPt( cur3, R2[i], uq2[i], top, toc, &v, &Q );
        N_DistPtPt( R2[i], Q, &d2 );
        d1 = maxtol - Ep + d1;
        d2 = maxtol - Eg + d2;
        err[i] = NL_MAX( d1, d2 );
    }

    if( closed OR tangents )
    {
        N_CrvGetDegree( cur3, &pp );

        if( pp EQ 2 )
        {
            err[1] = 1.1 *err[1];
            err[mm - 1] = 1.1 *err[mm - 1];
        }
    }

    /* Now force C1 */

    ktol = ktol * (uq2[mm] - uq2[0]);
    N_CrvGetCPtsDegreeAndKnots( cur3, &nn, &Pw, &pp, &km, &U2 );
    N_CrvGetKnotVector( cur3, &knt2 );
    U1 = N_AllocReal1dArray( nn + 2, &SL ); /* enough space for the refinement vector */

    if( U1 EQ NULL )
        NL_QUIT;
    temp = N_AllocReal1dArray( mm, &SL ); /* enough space for temporary errors */

    if( temp EQ NULL )
        NL_QUIT;

    np = 0;          /* total number of pp-multiplicity knots found */
    km = km - pp;
    kk = -1;         /* high index of refinement knots */
    k = 0;           /* index into uq2 */
    i = pp + 1;

    while( i LT km ) /* find pp-multiplicity knots */
    {
        j = 1;

        while( U2[i]EQ U2[i + j] )
            j += 1;

        if( j LT pp )
        {
            i = i + j;
            continue;
        }

        i = i + j - 1; /* pp-multiplicity */
        np += 1;

        while( uq2[k]LT U2[i] )
            k += 1;
        k1 = k2 = k;
        jj = i - pp;                                  /* index of basis function corres. to pp-mult knot */

        while( uq2[k1 - 1]GT U2[jj] )
            k1 -= 1;                                  /* get range of parameters */

        while( uq2[k2 + 1]LT U2[i + 1] )
            k2 += 1;                                  /* in neighbor knot spans  */

        d1 = (U2[i] - U2[jj]) / (U2[i + 1] - U2[jj]); /* get knt removal err bnd */
        N_Combine2CPts( d1, Pw[jj + 1], 1.0 - d1, Pw[jj - 1], &Qw );
        N_DistCptCptHomo( Pw[jj], Qw, &Ebnd );

        if( Ebnd LT ptol ) /* can remove without insertion */
        {
            i += 1;
            continue;
        }

        for ( j = k1; j <= k2; j++ ) /* check if can remove without insertion */
        {
            error = N_BasisIEval( knt2, jj, pp, uq2[j], NL_LEFT, &d2 );
            temp[j] = d2 * Ebnd + err[j];

            if( error EQ NL_YES OR temp[j]GT maxtol )
                break;
        }

        if( j GT k2 ) /* can remove without insertion */
        {
            for ( j = k1; j <= k2; j++ )
                err[j] = temp[j];
            i += 1;
            continue;
        }

        d1 = (maxtol - err[k]) / Ebnd; /* must insert a knot to each */

        if( d1 LT 0.0 )
            d1 = 0.0;                  /* side of U2[i]              */

        if( d1 GT 1.0 )
            d1 = 1.0;
        v = d1 * U2[jj] + (1.0 - d1) * U2[i];

        if( v LT uq2[k - 1] )
            v = uq2[k - 1];

        else if( v GT U2[i] - ktol )
            v = U2[i] - ktol;
        kk += 1;
        U1[kk] = v;
        v = d1 * U2[i + 1] + (1.0 - d1) * U2[i];

        if( v GT uq2[k + 1] )
            v = uq2[k + 1];

        else if( v LT U2[i] + ktol )
            v = U2[i] + ktol;
        kk += 1;
        U1[kk] = v;
        err[k] = maxtol; /* remove only once */

        i += 1;
    }

    if( kk GE 0 ) /* must refine knot vector */
    {
        N_KnotVectorFromRealArray( &knt3, U1, kk );
        error = N_CrvRefine( cur3, &knt3, cur3, &SL, &SL );
    }

    if( np GT 0 ) /* now remove one occurrence of pp-mult knots */
    {
        i = pp + 1;
        N_CrvGetKnots( cur3, &km, &U2 );
        km = km - pp;

        while( i LT km ) /* find pp-multiplicity knots */
        {
            j = 1;

            while( U2[i]EQ U2[i + j] )
                j += 1;

            if( j LT pp )
            {
                i = i + j;
                continue;
            }

            error = N_CrvRemoveKnot( cur3, U2[i], 1, NL_BIGD, &k1, cur3, &SL );
            i = i + j - k1;
            N_CrvGetKnots( cur3, &km, &U2 );
            km = km - pp;
        }
    }

    /* Now knot remove one final time over entire curve */

    error = N_FitRemoveKnots( cur3, uq2, err, mm, maxtol );

    if( error EQ NL_YES )
        NL_OUT;

    /* If degree is 2, end tangents might be slightly off; correct them */

    if( pp EQ 2 AND( closed OR tangents ) )
    {
        N_CrvGetCPts( cur3, &nn, &Pw );

        /* cur3 has degree 2, Must have more than one span
        so that the start and end tangents can be reset */
        if( nn EQ 2 )
        {
            /* insert a mid knot so that cur3 has two spans */
            N_CrvGetParamBounds( cur3, &ul, &ur );
            umid = 0.5 *( ul + ur );
            error = N_CrvInsertKnot( cur3, umid, 1, cur3, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            N_CrvGetCPts( cur3, &nn, &Pw );
        }

        N_CreateLineStartDirVector( &line, R2[0], Vs, NL_UNBOUNDED );
        N_CPtToPtEuclid( Pw[1], &Q );
        error = N_ProjectPtLine( line, Q, &Q, &d1, &dumf );

        if( error EQ  NL_NO)
            N_PtToCPt( Q, &Pw[1] );

        N_CreateLineStartDirVector( &line, R2[mm], Ve, NL_UNBOUNDED );
        N_CPtToPtEuclid( Pw[nn - 1], &Q );
        error = N_ProjectPtLine( line, Q, &Q, &d1, &dumf );

        if( error EQ  NL_NO)
            N_PtToCPt( Q, &Pw[nn - 1] );
    }

    /* Must check degree; it could be quadratic and less than p. */
    /* And copy over to curQ. */

    if( pp LT p )
        error = N_CrvElevateDegree( cur3, p - pp, curQ, &SL, SQ );
    else
        error = N_CrvCopy( cur3, curQ, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    NL_ITLIM = save;

    N_EndNurbs( &SL );

    return (error);
}


#ifdef NLIB_UNUSED

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

/*****************************************************************/
/* ST_ApproxCrvOnSrf: Check error of approximation               */
/*****************************************************************/

NL_FLAG ST_ApproxCrvOnSrf( NL_PARAMETER s, NL_POINT *P )
{
    NL_PRIVATE NL_STRING rname = _T("ST_ApproxCrvOnSrf");

    NL_FLAG error;

    NL_POINT R;

    error = N_CrvEval( gloC, s, NL_LEFT, &R );

    if( error EQ NL_YES )
        return (error);

    if( R.x LT gloS->knu->U[0] )
    {
        if( R.x + NL_PTOL LT gloS->knu->U[0] )
        {
            N_ErrSet( NL_PAR_ERR, rname );
            return (NL_YES);
        }
        else
            R.x = gloS->knu->U[0];
    }

    if( R.y LT gloS->knv->U[0] )
    {
        if( R.y + NL_PTOL LT gloS->knv->U[0] )
        {
            N_ErrSet( NL_PAR_ERR, rname );
            return (NL_YES);
        }
        else
            R.y = gloS->knv->U[0];
    }

    if( R.x GT gloS->knu->U[gloS->knu->m] )
    {
        if( R.x - NL_PTOL GT gloS->knu->U[gloS->knu->m] )
        {
            N_ErrSet( NL_PAR_ERR, rname );
            return (NL_YES);
        }
        else
            R.x = gloS->knu->U[gloS->knu->m];
    }

    if( R.y GT gloS->knv->U[gloS->knv->m] )
    {
        if( R.y - NL_PTOL GT gloS->knv->U[gloS->knv->m] )
        {
            N_ErrSet( NL_PAR_ERR, rname );
            return (NL_YES);
        }
        else
            R.y = gloS->knv->U[gloS->knv->m];
    }

    error = N_SrfEvalPt( gloS, (NL_PARAMETER)R.x, (NL_PARAMETER)R.y, NL_LEFT, NL_LEFT, P );

    return (error);
}

/*****************************************************************/
/* ST_ApproxNca: Check error of approximation                    */
/*****************************************************************/

NL_FLAG ST_ApproxNca( NL_PARAMETER u, NL_POINT *P )
{
    NL_FLAG error = NL_NO;

    error = N_CrvEval( gloC, u, NL_LEFT, P );

    return (error);
}

/**********************************************************************/
/* ST_CheckApproxErr: Check error of approximation                    */
/**********************************************************************/

NL_FLAG ST_CheckApproxErr( NL_CURVE *cur, NL_POINT C, NL_REAL rad, NL_REAL as, NL_REAL ae, NL_REAL toc, NL_REAL *erc )
{
    NL_PRIVATE NL_STRING rname = _T("ST_CheckApproxErr");

    NL_FLAG error = NL_NO;

    NL_INDEX i, m, s;

    NL_REAL *U, ang, emax, umax, len, u, uinc, dot, zco, uold, unew, num, den, drv, dis;

    NL_POINT Q;

	NL_VECTOR D[3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } ;
	
	NL_VECTOR V;

    /* Get start parameter */

    N_CrvGetKnots( cur, &m, &U );

    ang = (ae - as) / 180.0;
    len = ang * NL_PI * rad;
    s = (NL_INDEX)(100 * len);
    s = NL_MAX( 100, s );
    uinc = (U[m] - U[0]) / s;
    emax = 0.0;
    umax = 0.0;

    for ( i = 1; i < s; i++ )
    {
        u = U[0] + i * uinc;

        error = N_CrvEval( cur, u, NL_LEFT, &Q );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( C, Q, &len );
        len = fabs( rad - len );

        if( len GT emax )
        {
            emax = len;
            umax = u;
        }
    }

    /* Use Newton to find precise error */

    unew = umax;
    zco = NL_BIGD;

    while( zco GT toc )
    {
        error = N_CrvDerivs( cur, unew, NL_LEFT, 2, D );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( D[0], C, &V );
        N_VectorDot( D[1], V, &num );
        N_VectorMagnitude( D[1], &drv );
        N_VectorMagnitude( V, &dis );

        if( N_FloatOpIsBad( num, drv * dis, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        zco = fabs( num ) / (drv * dis);

        if( zco LE toc )
            break;

        N_VectorDot( D[2], V, &dot );
        den = dot + drv * drv;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        uold = unew;
        unew = uold - num / den;

        if( unew LT U[0] )
            unew = U[0];

        if( unew GT U[m] )
            unew = U[m];
    }

    N_DistPtPt( D[0], C, &len );
    *erc = fabs( len - rad );

    EXIT:

    return (error);
}

#endif // NLIB_UNUSED
