// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* Spiral.c: Curve and surface spiral routines                        */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

static NL_REAL ST_GetSpiralAngle( NL_REAL Theta );
static int ST_GetPointAtAngle( NL_REAL Theta, NL_POINT *xyz );
static int ST_GetPointAtAngleD( NL_REAL Theta, NL_POINT *xyz );

/**********************************************************************/
/* N_CrvApproxSpiral: Cubic approximation of a spiral with error specified */
/**********************************************************************/

static constexpr NL_REAL TWOPI = 6.2831853071795864;
static int RL = 1;
static NL_INDEX nSpans = 8;
static NL_REAL dTurns = 1.0;
static NL_REAL h = 0.5;
static NL_REAL r0 = 1.0;
static NL_REAL r1 = 1.0;
static NL_REAL Rdfun = 0.0; /*((-r0 + r1)/(TWOPI*dTurns)); */

/*******************************************************************//**

   DESCRIPTION:

     Construct a cubic NURBS approximation to a spiral around the z axis. 
     The axis of the spiral is the z axis from(0, 0, 0) to(0, 0, height).
     rad0 is the radius at z = 0 and rad1 is the radius at z = height 
        with rad0 >= 0  and rad1 >= 0 and rad0 or rad1 must be > 0.
     NumTurns is the number of revolutions, eg NumTurns = 2.5.
     The spiral starts at (rad0, 0, 0) and revolves to (x, y, height) 
        with sqrt(x2 + y2) = rad1.
     The spiral is a right hand spiral if RorL = 1 and 
        a left hand spiral if RorL = 0
     If NumPoints > 0 then this is the number of points to interpolate,
        if NumPoints is 0 then then NumPoints is calculated and the 
        curve is an approximation to the given tol.

       NL_REAL        height;
       NL_REAL        rad0;
       NL_REAL        rad1;
       NL_REAL        NumTurns;
       NL_INDEX       RorL;
       NL_INDEX       *NumPoints;
       NL_REAL        *tol
       NL_CURVE       *Crv;
       NL_STACKS&      S;
       ....
       ....
       get height, rad0, rad1, NumTurns, RorL, deg
       set NumPoints and tol
       ...
       N_CrvInitArrays(&cur);
       N_CrvApproxSpiral(height, rad0, rad1, NumTurns, 
            RorL, *NumPoints, *tol, &Crv, &S);

   ACCESS:
   
     height     input     The height (>0) of the spiral along the z axis 
     rad0       input     Initial radius at z = 0
     rad1       input     Final radius at z = height
     NumTurns   input     Number of turns (1 = 360 degrees) in the spiral
     RorL       input     Right or left spiral direction
     *NumPoints input     if zero, the routine will compute and 
                           Crv will be a fit to the tolerance tol
                output    number of points required for this fit
                           max allowed is determined by NL_CCPLIM
     *tol       input     Cubic Crv is an approximation to within tol
                           tol > 10*NL_MTOL
                
     Crv        output    Spiral cubic curve
     S          input     Crv's memory stack

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CrvApproxSpiral( NL_REAL height, NL_REAL rad0, NL_REAL rad1, NL_REAL NumTurns, NL_INDEX RorL, NL_INDEX *NumPoints, NL_REAL *tol, NL_CURVE *Crv, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvApproxSpiral");

    NL_STACKS SL;
    NL_REAL dTheta, angle;
    NL_VECTOR dP0, dP1;
    NL_POINT *Pnts;
    NL_INDEX m, deg = 3;
    NL_REAL *UU;
    NL_REAL t, dist, Theta, distCxy, distDxy, factor;
    NL_REAL dmax = 0.0;
    NL_POINT C, D;
    NL_INDEX i, nSpans0, nSpansMax, deltaSpans;
    NL_FLAG error;
    NL_BOOLEAN doFit, inTol;

    /* check for input errors */

    if( height < NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    if( (rad0 < -NL_MTOL) || (rad1 < -NL_MTOL) )
        NL_ERROR( NL_INP_ERR );

    if( rad0 < NL_MTOL )
        rad0 = 0.;

    if( rad1 < NL_MTOL )
        rad1 = 0.;

    if( (rad0 + rad1) < NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    if( NumTurns < NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    if( *tol < 10 * NL_MTOL )
        *tol = 10*NL_MTOL;

    nSpansMax = NL_CCPLIM - 6; /* max number of knots for cubic*/

    /* set up static values */
    RL = (RorL == 1) ? 1 : -1;
    h = height;
    r0 = rad0;
    r1 = rad1;

    dTurns = NumTurns;
    nSpans = *NumPoints - 1;

    doFit = NL_FALSE;

    if( nSpans <= 0 )
    {
        doFit = NL_TRUE;
        nSpans = (int)(8.0 *dTurns + 0.99);
    }

    inTol = NL_FALSE;
    N_InitNurbs( &SL ); /* initialize the local stacks */

    do
    {
        *NumPoints = nSpans + 1;
        Pnts = N_AllocPt1dArray( nSpans + 1, &SL );
        dTheta = TWOPI * dTurns / nSpans;

        Pnts[0].x = r0;
        Pnts[0].y = 0.0;
        Pnts[0].z = 0.0;

        angle = 0.0;

        for ( i = 1; i <= nSpans; i++ )
        {
            angle = angle + dTheta;
            ST_GetPointAtAngle( angle, &Pnts[i] );
        }

        Rdfun = ((-r0 + r1) / (TWOPI * dTurns));
        ST_GetPointAtAngleD( 0.0, &dP0 );
        ST_GetPointAtAngleD( TWOPI * dTurns, &dP1 );

        /* End tangents are the exact analytic derivative dP/dTheta scaled
           uniformly into the fit's [0,1] parameterization (Theta = 2*pi*dTurns
           * u, so dP/du = dP/dTheta * 2*pi*dTurns). Scaling every component by
           the same factor preserves the true tangent direction. The previous
           code normalized dP0/dP1 and then scaled XY by r*2*pi*dTurns and Z by
           h -- different per-axis factors that corrupted the direction (e.g.
           the start-tangent y/z ratio of a constant-radius helix came out as
           r^2*(2*pi*turns)^2/h^2 instead of the correct r*2*pi*turns/h). The
           uniform factor below reproduces the intended magnitudes (XY ~
           r*2*pi*dTurns, Z ~ h) while keeping the direction analytic. */
        dP0.x *= TWOPI * dTurns;
        dP0.y *= TWOPI * dTurns;
        dP0.z *= TWOPI * dTurns;
        dP1.x *= TWOPI * dTurns;
        dP1.y *= TWOPI * dTurns;
        dP1.z *= TWOPI * dTurns;

        /* Use cubic interpolation at the knots */

        error = N_FitCubicSplineInterp( Pnts, nSpans, dP0, dP1, NL_UNIFORM, Crv, S );

        if( error )
            NL_OUT;

        /* Test goodness of fit */

        N_CrvGetKnots( Crv, &m, &UU );
        dmax = 0.0;

        for ( i = 0; i < nSpans; i++ ) /* check midpoint of each span */
        {
            t = 0.5 *( UU[deg + i] + UU[deg + i + 1] );
            N_CrvEval( Crv, t, NL_LEFT, &C );
            Theta = (C.z * TWOPI * dTurns) / h;
            ST_GetPointAtAngle( Theta, &D );
            distCxy = sqrt( C.x * C.x + C.y * C.y );
            distDxy = sqrt( D.x * D.x + D.y * D.y );
            dist = fabs( distCxy - distDxy );

            if( dist > dmax )
                dmax = dist;
        }

        if( doFit )
        {
            inTol = (dmax < *tol);
        }
        else
            inTol = NL_TRUE;

        /* if not intol, reset nSpans and repeat */
        if( !inTol )
        {
            nSpans0 = nSpans;
            factor = dmax / (*tol);

            if( factor > 1.0 )
                factor = 1.0;

            deltaSpans = (int)(nSpans * factor);

            if( deltaSpans < 1 )
                deltaSpans = 1;
            nSpans = nSpans + deltaSpans;

            if( nSpans0 < nSpansMax )
            {
                if( nSpans > nSpansMax )
                    nSpans = nSpansMax;
                Crv->pol = NULL;
                Crv->p = -1;
                Crv->knt = NULL;
                N_EndNurbs( &SL );
            }
            else
            { /* we hit the max number of allowed spans */
                inTol = NL_TRUE;
            }
        }
    } while ( !inTol );

    /* in all cases return numPoints and dmax */

    *tol = dmax;
    *NumPoints = nSpans + 1;

    /* End NURBS and Exit */
    EXIT:
    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_CreateSpiralSrf: Sweep a curve along a spiral                         */
/**********************************************************************/

/*******************************************************************//**

   DESCRIPTION:

     Sweep a curve along  a spiral around the z axis.
     The curve Crv must be g1 continuous, to make a valid surface, and
     be in the x,z plane
     The Crv origin (0,0,0) is at the center of the helix at height=0, radius0.
     The axis of the spiral is the z axis from(0, 0, 0) to(0, 0, height).
     radius0 is the radius at z = 0 and radius1 is the radius at z = height 
        with radius0 >= 0 and radius1 >=0;
     NumTurns is the number of revolutions, eg NumTurns = 2.5.
     The spiral starts at (radius, 0, 0) and revolves to (x, y, height) 
        with sqrt(x2 + y2) = radius.
     The spiral is a right hand spiral if RorL = 1 and 
        a left hand spiral if RorL = 0
     If NumPoints > 0 then this is the number of points to interpolate,
        if NumPoints is 0 then then NumPoints is calculated and the 
        spiral curve is an approximation to the given tol.
     The radius0 and radius1 values should be large enough so that the swept
       curve always has a positive radius about the z axis ( radius - Pw.x >= 0)

       NL_CURVE       *Crv
       NL_REAL        height;
       NL_REAL        radius0;
       NL_REAL        radius1;
       NL_REAL        NumTurns;
       NL_INDEX       RorL;
       NL_INDEX       *NumPoints;
       NL_REAL        *tol
       NL_SURFACE     *Srf;
       NL_STACKS&      S;
       ....
       ....
       get height, radius, taper, NumTurns, RorL, deg
       set NumPoints and tol
       ...
       N_CrvInitArrays(&Crv);
       ....get curve to sweep in X,Z plane
       N_SrfInitArrays(&Srf);
       N_CrvApproxSpiral(&Crv. height, radius, taper, NumTurns, 
            RorL, *NumPoints, *tol, &Srf, &S);

   ACCESS:
   
     *Crv       input     Curve to sweep along helix
     height     input     The height (>0) of the spiral along the z axis 
     radius0    input     Initial radius at z = 0
     radius1    input     Initial radius at z = height
     NumTurns   input     Number of turns (1 = 360 degrees) in the spiral
     RorL       input     Right or left spiral direction
     *NumPoints input     if zero, the routine will compute and 
                           Crv will be a fit to the tolerance tol
                output    number of points required for this fit
                           max allowed is determined by NL_CCPLIM
     *tol       input     Cubic Crv is an approximation to within tol
                           tol > 10*NL_MTOL
                
     Srf        output    Output Surface
     S          input     Crv's memory stack

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CreateSpiralSrf
 (NL_CURVE   * Crv,        /* in : curve in x-y plane to be swept (not modified) */
  NL_REAL      height,     /* in : height of helix sweep                                               */
  NL_REAL      radius0,    /* in : defines taperAngle as: tan(TaperAngle) = (radius1 - radius0)/height */
  NL_REAL      radius1,    /* in : defines taperAngle as: tan(TaperAngle) = (radius1 - radius0)/height */
  NL_REAL      NumTurns,   /* in : 1 turn = 360 degrees                                                */
  NL_INDEX     RorL,       /* in : 1 = right hand spiral                                               */
                           /*      0 = left hand spiral                                                */
  NL_INDEX   * numPoints,  /* i/o: 0 = fit helix sweep to tolerance                                    */
                           /*      else number of control points in sweep direction.                   */
                           /*      Set to number of sweep direction control points on exit.            */
  NL_REAL    * tol,        /* i/o: tol of swept helix curves from true helix, range: tol >= 1.0E-6     */
  NL_SURFACE * Srf,        /* out: Output surface                                                      */
  NL_STACKS  * S )         /* in : Crv's memory stack                                                  */
{
    NL_STACKS SL;

    NL_CURVE CrvH;
    NL_FLAG error = NL_NO;

    NL_REAL w;
    NL_REAL rad0, rad1, taper;
    NL_KNOTVECTOR *hKnt;
    NL_BOOLEAN rat = 0;

    /* get knot data off section curve */
    NL_CPOINT *cPw, *hPw, ** Qw, *Cp;
    NL_REAL *cU, *hU, *UQ, *VQ;
    NL_DEGREE p, q;
    NL_INDEX i, ii, mU, n, m;
    NL_VECTOR Trans;
    NL_KNOTVECTOR *knu, *knv;

    N_InitNurbs( &SL );
    N_CrvInitArrays( &CrvH );
    N_CrvGetCPtsDegreeAndKnots( Crv, &n, &cPw, &p, &mU, &cU );

    if( cPw[0].w != NL_NOW )
        rat = 1;
    taper = radius1 - radius0;

    /* for each control point in section curve */
    for ( ii = 0; ii <= n; ii++ )
    {
        /* the helix */
        w = (rat) ? cPw[ii].w : 1.0; /* weight of Pw from section curve  */
        rad0 = radius0 + cPw[ii].x / w;
        /* rad0 = sqrt(cPw[ii].x/w * cPw[ii].x/w + cPw[ii].y/w * cPw[ii].y/w) ; */
        if( rad0 < 0.0 )
        {
            error = 1;
            NL_OUT;
        }
        rad1 = rad0 + taper;

        if( rad1 < 0.0 )
        {
            error = 1;
            NL_OUT;
        }

        error = N_CrvApproxSpiral( height, rad0, rad1, NumTurns, RorL, numPoints, tol, &CrvH, &SL );

        if( error )
            NL_OUT;

        m = *numPoints + 1; /* must now be constant for all helixes */

        /*  Translate CrvH to coeff point */
        Trans.x = 0.0;
        Trans.y = cPw[ii].y / w;
        Trans.z = cPw[ii].z / w;

        N_CrvTranslate( &CrvH, Trans );

        if( ii == 0 )
        {
            /* Build surface */

            q = 3; /* degree from helix = cubic */
            N_SrfSizeArrays( Srf, n, m, p, q, n + p + 1, m + q + 1, _T("N_CreateSpiralSrf"), S );
            N_SrfGetCPtsKnotVectorAndKnots( Srf, &Qw, &knu, &knv, &UQ, &VQ );

            /* get knot data off section curve */
            for ( i = 0; i <= n + p + 1; i++ )
                UQ[i] = cU[i];

            /* get knot data off helix */
            N_CrvGetCPtsKnotVectorAndKnots( &CrvH, &hPw, &hKnt, &hU );

            for ( i = 0; i <= m + q + 1; i++ )
                VQ[i] = hU[i];
        }
        else
        { /* get control points from helix  */
            N_CrvGetCPtsKnotVectorAndKnots( &CrvH, &hPw, &hKnt, &hU );
        }

        /* load  coeffs from helix */
        for ( i = 0; i <= m; i++ )
        {
            Cp = &( Srf->net->Pw[ii][i] );
            Cp->x = hPw[i].x * w;
            Cp->y = hPw[i].y * w;
            Cp->z = hPw[i].z * w;
            Cp->w = (rat) ? w : NL_NOW;
        }
    }

    /* End NURBS and Exit */
    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSpiralSrf */


/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

static NL_REAL ST_GetSpiralAngle( NL_REAL Theta )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CrvApproxSpiral"); */
    return ((r0 * (TWOPI * dTurns - Theta) + r1 * Theta) / (TWOPI * dTurns));
}

/* --------------------------------------------------------------- */
/* --------------------------------------------------------------- */

static int ST_GetPointAtAngle( NL_REAL Theta, NL_POINT *xyz )
{
    NL_REAL cosTheta, sinTheta;

    if( Theta > TWOPI * dTurns )
        Theta = TWOPI * dTurns;
    cosTheta = cos( Theta );
    sinTheta = sin( Theta );
    xyz->x = ST_GetSpiralAngle( Theta ) * cosTheta;
    xyz->y = RL * ST_GetSpiralAngle( Theta ) * sinTheta;
    xyz->z = h * Theta / (TWOPI * dTurns);
    return (1);
}

/* --------------------------------------------------------------- */
/* --------------------------------------------------------------- */

static int ST_GetPointAtAngleD( NL_REAL Theta, NL_POINT *xyz )
{
    NL_REAL cosTheta;
    NL_REAL sinTheta;

    if( Theta > TWOPI * dTurns )
        Theta = TWOPI * dTurns;

    cosTheta = cos( Theta );
    sinTheta = sin( Theta );
    xyz->x = -ST_GetSpiralAngle( Theta ) * sinTheta + Rdfun * cosTheta;
    xyz->y = RL * (ST_GetSpiralAngle( Theta ) * cosTheta + Rdfun * sinTheta);
    xyz->z = h / (TWOPI * dTurns);
    return (1);
}
