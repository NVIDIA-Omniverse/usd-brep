// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***************************************************************************************/
/* SrfGeom.c : Geometry Processing Function Definitions that act on NL_SURFACE objects */
/***************************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */


/**********************************************************************/
/* N_SrfGetClosestPt: Surface point inversion/projection using Newton's method */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine  finds the  point Q on a NURBS
     surface closest to a  given point P. The  parameters corresponding 
     to  P  are  also  returned. The  point  P  can be  on  the surface 
     (inversion)  or  off  the surface  (projection). The  routine uses  
     Newton iteration  with  start  parameters passed in. The iteration 
     limit  NL_ITLIM is  defined as a global parameter in "globals.h". The 
     returned  u,v,Q represent the  best solution found,  regardless of
     the returned error value. A typical calling example is:

       NL_SURFACE    sur;
       NL_POINT      P, Q;
       NL_REAL       top, toc;
       NL_PARAMETER  u0, v0, u, v;
       ...
       (define sur, get P, u0, v0, top and toc);
       ...
       N_SrfGetClosestPt(&sur,P,u0,v0,top,toc,&u,&v,&Q);

     If convergence is not reached, the predefined NL_CON_ERR (convergence 
     error) is returned. In any case, the best values of u,v and Q  are 
     returned.  


   ACCESS:
   
     sur   , input  ,  NURBS surface
     P     , input  ,  Point to be inverted/projected
     u0,v0 , input  ,  Guess parameters
     top   , input  ,  Point coincidence tolerance
     toc   , input  ,  Zero cosine tolerance
     u,v   , output ,  Parameters  corresponding to Q, i.e. Q = S(u,v)
     Q     , output ,  Point on the surface closest to P


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfGetClosestPt( NL_SURFACE *sur, NL_POINT P, NL_PARAMETER u0, NL_PARAMETER v0, NL_REAL top, NL_REAL toc, NL_PARAMETER *u, NL_PARAMETER *v, NL_POINT *Q )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfGetClosestPt"); */

    NL_FLAG error = NL_NO;

    NL_POINT ** SD;

    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_OUT;

    error = N_GetClosestPtOnSrf( sur, P, u0, v0, top, toc, u, v, Q, SD );

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfGetClosestPt */


/**********************************************************************/
/* N_GetClosestPtOnSrf: Surface point inversion/projection using Newton's method */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine  finds the  point Q on a NURBS
     surface closest to a  given point P. The  parameters corresponding 
     to  P  are  also  returned. The  point  P  can be  on  the surface 
     (inversion)  or  off  the surface  (projection). The  routine uses  
     Newton iteration  with  start  parameters passed in. The iteration 
     limit  NL_ITLIM is  defined as a global parameter in "globals.h". The 
     returned  u,v,Q represent the  best solution found,  regardless of
     the returned error value. A typical calling example is:

       NL_SURFACE    sur;
       NL_POINT      P, Q;
       NL_REAL       top, toc;
       NL_PARAMETER  u0, v0, u, v;
       NL_POINT      **P;
       ...
       (define sur, get P, u0, v0, top and toc);
       ...
       N_SrfGetClosestPt(&sur,P,u0,v0,top,toc,&u,&v,&Q);

     If convergence is not reached, the predefined NL_CON_ERR (convergence 
     error) is returned. In any case, the best values of u,v and Q  are 
     returned.  


   ACCESS:
   
     sur   , input  ,  NURBS surface
     P     , input  ,  Point to be inverted/projected
     u0,v0 , input  ,  Guess parameters
     top   , input  ,  Point coincidence tolerance
     toc   , input  ,  Zero cosine tolerance
     u,v   , output ,  Parameters  corresponding to Q, i.e. Q = S(u,v)
     Q     , output ,  Point on the surface closest to P


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_GetClosestPtOnSrf
 (NL_SURFACE   *sur,     /* Surface                     */
  NL_POINT      P,       /* Point                       */
  NL_PARAMETER  u0,      /* Guess u value               */
  NL_PARAMETER  v0,      /* Guess v value               */
  NL_REAL       top,     /* Point Coincidence Tolerance */
  NL_REAL       toc,     /* Zero cosine tolerance       */
  NL_PARAMETER *u,       /* Output: u value             */
  NL_PARAMETER *v,       /* Output: v Value             */
  NL_POINT     *Q,       /* Output: Surface Point closest to P */
  NL_POINT    **D )      /* Output: SD[0][0]=Pos, SD[0][1]=Dv,   SD[0][2]=Dvv, sized:[3][3] */
                         /*         SD[1][0]=Du,  SD[1][1]=Duv,  SD[1][2]=Duvv              */
                         /*         SD[2][0]=Duu, SD[2][1]=Duuv, SD[2][2]=Duuvv             */
{
    NL_PRIVATE NL_STRING rname = _T("N_GetClosestPtOnSrf");

    NL_FLAG error = NL_NO, uclosed = NL_NO, vclosed = NL_NO;

    NL_INDEX k, r, s, obndy, ucount, vcount;

    NL_REAL *U, *V;
    NL_REAL FVal, GVal, dis, zcu, zcv, Fu, Fv, Gu, Gv;
    NL_REAL GapDotSuv, SuDotSv, num, den, deltaU, deltaV, mag, best_dis, d1, d2;
    NL_REAL Fu_1, Fu_2, Gv_1, Gv_2;

    static constexpr NL_REAL d2ndDerivTermLimit = 0.50; /* See comments below. */

    NL_PARAMETER uold, vold, unew, vnew, ub = 0.0, vb = 0.0;

    NL_POINT A = { 0,0,0 }, B;

    NL_VECTOR Gap;

    /* We changed from using surface first derivatives in the objective function  */
    /* to using the normalized derivatives: works much better.  [B146]            */
    /* That's more involved programming however, as the derivatives of            */
    /* unitized vectors get kind of involved.  But when you do it                 */
    /* step by step, it's not too bad.  For example, the derivative m' of the     */
    /* magnitude of a unitized vector V comes out to U.V', where U is V unitized. */
    /* Then the derivative of the unitized vector U' is ( V'' - U*m' ) / m.          */
    /* So here we go.  */
    /* NL_POINT  sS; */                         /* Surface position */
    NL_VECTOR sSu,   sSv,   sSuu, sSuv, sSvv; /* Ordinary surface derivatives */
    NL_VECTOR sSuN,  sSvN;                    /* Normalized 1st derivs: sSu / dMU, sSv / dMV */
    NL_VECTOR sSuNu, sSuNv, sSvNu, sSvNv;     /* derivs of sSuN and sSvN */
    NL_REAL dMU,  dMV;                        /* magnitudes of 1st derivs, sSu.Length(), sSv.Length() */
    NL_REAL dMUu,  dMUv,  dMVu,  dMVv;        /* derivs of dMU and dMV */
    NL_REAL dTemp1, dTemp2;
    NL_VECTOR sTempVec1, sTempVec2;

    /* Get local notation */

    N_SrfGetKnots( sur, &r, &s, &U, &V );

    /* Perform Newton interations till convergence reached */

    if( N_SrfIsClosed( sur, NL_UDIR ) )
        uclosed = NL_YES;

    if( N_SrfIsClosed( sur, NL_VDIR ) )
        vclosed = NL_YES;

    k = 0;
    unew = u0;
    vnew = v0;
    best_dis = NL_BIGD;

    ucount = vcount = 0;

    while( k LT NL_ITLIM )
    {
        /* Get derivatives */

        error = N_SrfDerivs( sur, unew, vnew, NL_LEFT, NL_LEFT, NL_TRUE, 2, 2, D );

        if( error EQ NL_YES )
            NL_OUT;

        /* Rename vectors in D */
        /* sS   = D[0][0]; */
        sSu  = D[1][0];
        sSv  = D[0][1];
        sSuu = D[2][0];
        sSuv = D[1][1];
        sSvv = D[0][2];

        /* Prepare for convergence test */

        N_VectorDiff( D[0][0], P,   &Gap  );

        /* Convergence test #1: point coincidence */

        N_VectorMagnitude( Gap, &dis );

        if( dis LT best_dis )
        {
            *u = unew;
            *v = vnew;
            N_CopyPt( D[0][0], Q );
            best_dis = dis;
        }

        if( dis LE top )
            break;

        /* N_VectorDot(  sSu, Gap, &FVal ); */
        /* N_VectorDot(  sSv, Gap, &GVal ); */
        N_VectorMagnitude( sSu, &dMU );
        N_VectorMagnitude( sSv, &dMV );
        /* Note, this test is more stringent than we have to be, comparing against 1.0.
         * But if the magnitude is smaller than the machine limits,
         * we know it's not going to work.
         */
        if( N_FloatOpIsBad( 1.0, dMU, NL_DIVISION ) ) { NL_ERROR( NL_CON_ERR ); }
        if( N_FloatOpIsBad( 1.0, dMV, NL_DIVISION ) ) { NL_ERROR( NL_CON_ERR ); }

        N_VectorScale( sSu, 1/dMU, &sSuN );
        N_VectorScale( sSv, 1/dMV, &sSvN );
        N_VectorDot( Gap, sSuN, &FVal );
        N_VectorDot( Gap, sSvN, &GVal );

        /* Convergence test #2: zero cosine: gap vector perp to both 1st derivs. */

        if( N_FloatOpIsBad( FVal, dMU * dis, NL_DIVISION ) )
          { NL_ERROR( NL_CON_ERR ); }

        if( N_FloatOpIsBad( GVal, dMV * dis, NL_DIVISION ) )
          { NL_ERROR( NL_CON_ERR ); }

        zcu = fabs( FVal ) / (dMU * dis);
        zcv = fabs( GVal ) / (dMV * dis);

        if( zcu LE toc AND zcv LE toc AND dis EQ best_dis )
            break;


        /* No convergence, compute new parameters */
        /* Calculate derivatives of objective functions. */


        /*
         * Sometimes the 2nd derivative can dominate things, and lead to
         * divergence.  Geometrically, this can happen if the first deriv
         * is pointing towards the test point (for example), but its
         * magnitude is increasing rapidly in that direction.  The function
         * (FVal or GVal) is Gap dot Deriv, but if Deriv increases more than Gap
         * decreases, then the function value will decrease by moving in the
         * wrong direction: increasing Gap but decreasing Deriv even more.
         * This will also happen on a cylinder (for example), where the test
         * point is beyond the center of curvature from the current surface
         * trial point.  In that case, Gap is big, and it lines up closely with
         * the 2nd deriv, so their dot product is big.  This will push the
         * surface point to the far side of the cylinder, which is a solution
         * to the objective functions used, but is not what is desired.  (We're
         * looking for the geometrically closest point; hence the name of this
         * routine.)  Note that neither of these situations will arise if Gap is
         * small -- so this situation implies that the guess point wasn't really
         * good enough ... we're trying to work around that.
         * One way to fix this is to change the objective functions to use
         * unitized derivatives.  This gets a bit hairy, mathematically (e.g.,
         * derivatives of a unitized vector), but is not all that bad and is
         * certainly doable.
         * (Note: implemented the unitized-derivative method, 12/30/11.)
         * However, there's an easier way that works very
         * well in practice: simply limit the magnitude of the second-deriv
         * term in Fu and Gv.  This will guarantee that the step will move
         * closer to the test point, geometrically, even if it increases the
         * objective function values.  Specifically, Fu and Gv must be positive.
         * In practice, limiting the 2nd deriv term to (negative) half of the
         * 1st deriv term works well.  (The 1st deriv term is always positive.)
         * [B45]
         */
        /* Fu_1 = dMU * dMU;  / * Su dot Su */
        /* N_VectorDot( Gap, D[2][0], &Fu_2 ); */

        /* Calculate dMUu and sSuNu */
        N_VectorDot  ( sSuN, sSuu, &dMUu );
        N_VectorScale( sSuN, dMUu, &sTempVec1 );
        N_VectorDiff ( sSuu, sTempVec1, &sTempVec2 );
        N_VectorScale( sTempVec2, 1/dMU, &sSuNu );

        N_VectorDot( sSu, sSuN,  &Fu_1 );
        N_VectorDot( Gap, sSuNu, &Fu_2 );

        if ( Fu_2 < -Fu_1 * d2ndDerivTermLimit )
          {  Fu_2 = -Fu_1 * d2ndDerivTermLimit; }
        Fu = Fu_1 + Fu_2;

        /* Gv_1 = dMV * dMV;  / * Sv dot Sv */
        /* N_VectorDot( Gap, D[0][2], &Gv_2 ); */
        /* Calculate dMVv and sSvNv */
        N_VectorDot  ( sSvN, sSvv, &dMVv );
        N_VectorScale( sSvN, dMVv, &sTempVec1 );
        N_VectorDiff ( sSvv, sTempVec1, &sTempVec2 );
        N_VectorScale( sTempVec2, 1/dMV, &sSvNv );

        N_VectorDot( sSv, sSvN,  &Gv_1 );
        N_VectorDot( Gap, sSvNv, &Gv_2 );

        if ( Gv_2 < -Gv_1 * d2ndDerivTermLimit )
          {  Gv_2 = -Gv_1 * d2ndDerivTermLimit; }
        Gv = Gv_1 + Gv_2;

        N_VectorDot(   Gap,   D[1][1], &GapDotSuv );
        N_VectorDot( D[1][0], D[0][1], &SuDotSv   );

        /* Fv = SuDotSv + GapDotSuv; */
        /* Gu = Fv; */
        /* Calculate dMUv and sSuNv */
        N_VectorDot  ( sSuN, sSuv, &dMUv );
        N_VectorScale( sSuN, dMUv, &sTempVec1 );
        N_VectorDiff ( sSuv, sTempVec1, &sTempVec2 );
        N_VectorScale( sTempVec2, 1/dMU, &sSuNv );

        N_VectorDot( sSv, sSuN,  &dTemp1 );
        N_VectorDot( Gap, sSuNv, &dTemp2 );
        Fv = dTemp1 + dTemp2;

        /* Calculate dMVu and sSvNu */
        N_VectorDot  ( sSvN, sSuv, &dMVu );
        N_VectorScale( sSvN, dMVu, &sTempVec1 );
        N_VectorDiff ( sSuv, sTempVec1, &sTempVec2 );
        N_VectorScale( sTempVec2, 1/dMV, &sSvNu );

        N_VectorDot( sSu, sSvN,  &dTemp1 );
        N_VectorDot( Gap, sSvNu, &dTemp2 );
        Gu = dTemp1 + dTemp2;


        /* Solve 2x2 for shifts using functions and derivatives: Cramer's rule. */

        den =  Fu  * Gv - Fv *  Gu;

        num = FVal * Gv - Fv * GVal;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_CON_ERR );
        deltaU = - num / den;

        num = Fu * GVal - FVal * Gu;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_CON_ERR );
        deltaV = - num / den;

        uold = unew;
        vold = vnew;
        unew = uold + deltaU;
        vnew = vold + deltaV;

        /* Check #3: parameter range */

        obndy = -1;

        if( uclosed )
        {
            if( unew LT U[0] )
            {
                if( (U[0] - unew)LT( 100.0 *( U[r] - U[0] ) ) )
                {
                    while( unew LT U[0] )
                        unew = U[r] - U[0] + unew;
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
            else if( unew GT U[r] )
            {
                if( ( unew - U[r] ) LT ( 100.0 *( U[r] - U[0] ) ) )
                {
                    while( unew GT U[r] )
                        unew = U[0] - U[r] + unew;
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
            if( unew LT U[0] OR unew GT U[r] )
            {
                if( unew LT U[0] )
                    unew = U[0];

                if( unew GT U[r] )
                    unew = U[r];

                /* Calculate the 1-dimensional shift in the other parameter. */ 
                if( NOT( N_FloatOpIsBad( GVal, Gv, NL_DIVISION ) ) )
                {
                    vb = vold - GVal / Gv;

                    if( vb GE V[0] AND vb LE V[s] )
                        obndy = 1;
                }
            }
        }

        if( vclosed )
        {
            if( vnew LT V[0] )
            {
                if( ( V[0] - vnew ) LT ( 100.0 *( V[s] - V[0] ) ) )
                {
                    while( vnew LT V[0] )
                        vnew = V[s] - V[0] + vnew;
                    vcount = 0;
                }
                else
                {
                    if( vcount GE 2 )
                        NL_ERROR( NL_CON_ERR );
                    vnew = vold;
                    vcount += 1;
                }
            }
            else if( vnew GT V[s] )
            {
                if( ( vnew - V[s] ) LT ( 100.0 * ( V[s] - V[0] ) ) )
                {
                    while( vnew GT V[s] )
                        vnew = V[0] - V[s] + vnew;
                    vcount = 0;
                }
                else
                {
                    if( vcount GE 2 )
                        NL_ERROR( NL_CON_ERR );
                    vnew = vold;
                    vcount += 1;
                }
            }
            else
                vcount = 0;
        }
        else
        {
            if( vnew LT V[0] OR vnew GT V[s] )
            {
                if( vnew LT V[0] )
                    vnew = V[0];

                if( vnew GT V[s] )
                    vnew = V[s];

                /* Calculate the 1-dimensional shift in the other parameter. */ 
                if( NOT( N_FloatOpIsBad( FVal, Fu, NL_DIVISION ) ) )
                {
                    ub = uold - FVal / Fu;

                    if( ub GE U[0] AND ub LE U[r] )
                    {
                        if( obndy GT 0 )
                            obndy = 3;
                        else
                            obndy = 2;
                    }
                }
            }
        }

        /* check which of two possibilities is better */

        if( obndy GT 0 )
        {
            if( obndy LT 2 )
                ub = unew;

            if( obndy EQ 2 )
                vb = vnew;

            error = N_SrfEvalPt( sur, unew, vnew, NL_LEFT, NL_LEFT, &A );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_SrfEvalPt( sur, ub, vb, NL_LEFT, NL_LEFT, &B );

            if( error EQ NL_YES )
                NL_OUT;

            /* Check geometric distance. */
            N_DistSqPtPt( A, P, &d1 );
            N_DistSqPtPt( B, P, &d2 );

            if( d2 LT d1 )
            {
                unew = ub;
                vnew = vb;
            }
        }

        /* Check #4: parameter change */

        deltaU = unew - uold;
        deltaV = vnew - vold;

        /* This linear combination gives the 3d shift (A). */
        N_Combine2Pts( deltaU, D[1][0], deltaV, D[0][1], &A );
        N_PtMagnitude( A, &mag );

        if( mag LE top AND dis EQ best_dis )
            break;

        k++;
    }

    /* If no convergence, error out */

    if( k GE NL_ITLIM )
        NL_ERROR( NL_CON_ERR );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_GetClosestPtOnSrf */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine inverts the tangent  of a surface
     curve, i.e. it  computes the  tangent to the parameter space curve
     that is the inverse  image of the surface curve. A typical calling 
     example is:

       NL_SURFACE    sur;
       NL_VECTOR     T, V;
       NL_PARAMETER  u, v;
       ...
       (define sur, get T, u and v);
       ...
       N_InvertTangentSrfCrv(&sur,T,u,v,&V);

     IT IS ASSUMED THE THE NL_SURFACE IS AT LEAST ONCE DIFFERENTIABLE.


   ACCESS:
   
     sur   , input  ,  NURBS surface
     T     , input  ,  Tangent of surface curve at (u,v)
     u,v   , input  ,  Parameters at which tangent is computed
     V     , output ,  Tangent of the parameter space curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_InvertTangentSrfCrv( NL_SURFACE *sur, NL_VECTOR T, NL_PARAMETER u, NL_PARAMETER v, NL_VECTOR *V )
{
    NL_PRIVATE NL_STRING rname = _T("N_InvertTangentSrfCrv");

    NL_FLAG error = NL_NO;

    NL_REAL fu, fv, gu, gv, f, g, nu, nv, den, du, dv;

    NL_POINT ** D;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get surface derivatives */

    D = N_AllocPt2dArray( 1, 1, &SL );

    if( D EQ NULL )
        NL_OUT;

    error = N_SrfDerivs( sur, u, v, NL_LEFT, NL_LEFT, NL_TRUE, 1, 1, D );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve linear system */

    N_VectorDot( D[1][0], D[1][0], &fu );
    N_VectorDot( D[1][0], D[0][1], &fv );
    gu = fv;
    N_VectorDot( D[0][1], D[0][1], &gv );

    N_VectorDot( T, D[1][0], &f );
    N_VectorDot( T, D[0][1], &g );

    den = fu * gv - fv * gu;

    nu = f * gv - g * fv;
    nv = fu * g - gu * f;

    if( N_FloatOpIsBad( nu, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( nv, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    du = nu / den;
    dv = nv / den;

    N_VectorCreate( du, dv, 0.0, V );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_InvertTangentSrfCrv */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine  converts a  NURBS surface into 
     piecewise  power basis form. MEMORY TO STORE THE OUTPUT SURFACES 
     IS  ALLOCATED  INSIDE THE ROUTINE. A typical calling example is: 

       NL_SURFACE sur, ***spl;
       NL_INDEX   i, j, k, l;
       NL_CPOINT  **aw;
       NL_STACKS  SG;
       ...
       (define sur);
       ...
       N_ConvertNurbsToPowerBasis(&sur,NL_NORMALIZED,&spl,&k,&l,&SG);
       ...
       aw = spl[i][j]->net->Pw;
       ...

     spl[i][j], 0<=i<=k, 0<=j<=l is a  pointer to  the (i,j)-th power 
     basis  surface. The power  basis surface  patch is  defined as a  
     NL_SURFACE object; the vector coefficients  aw[i][j] are  stored in  
     place of  the control points, and the knots U[0], U[1] and V[0],
     V[1]  represent  the  bounds  of  the  rectangle  over which the 
     surface is defined.


   ACCESS:
   
     sur   , input  ,  NURBS surface to be converted
     pflg  , input  ,  Flag, indicating desired parameterization of  the
                       power basis patches (U[0],U[1],V[0],V[1] values):
                        = NL_INHERITED  : inherited knot span bounds
                        = NL_NORMALIZED : [0,1] x [0,1]
     spl   , output ,  2-D array of power basis surfaces
     kk,ll , output ,  Highest indexes in spl
     SG    , input  ,  spl's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ConvertNurbsToPowerBasis( NL_SURFACE *sur, NL_FLAG pflg, NL_SURFACE **** spl, NL_INDEX *kk, NL_INDEX *ll, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, ru, su, rv, sv, nsu, nsv, mlu, mlv, isu, ieu, isv, iev, row, col, save;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *A, *B, *uals, *vals, *omus, *omvs, num;

    NL_KNOTVECTOR *knu, *knv;

    NL_RMATRIX pum, pvm;

    NL_CPOINT ** Pw, ** Sw, ** NSw, ** Bw, ** NBw, ** aw;

    NL_SURFACE *** spa;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Allocate memory */

    N_BasisGetSpanCount( knu, p, &nsu );
    N_BasisGetSpanCount( knv, q, &nsv );

    spa = N_Alloc2dArraySrfPtrsParameters( p, q, p, q, 1, 1, nsu - 1, nsv - 1, SG );

    if( spa EQ NULL )
        NL_QUIT;

    Sw = N_AllocCPt2dArray( p, m, &SL );

    if( Sw EQ NULL )
        NL_QUIT;

    NSw = N_AllocCPt2dArray( p, m, &SL );

    if( NSw EQ NULL )
        NL_QUIT;

    Bw = N_AllocCPt2dArray( p, q, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    NBw = N_AllocCPt2dArray( p, q, &SL );

    if( NBw EQ NULL )
        NL_QUIT;

    uals = N_AllocReal1dArray( p, &SL );

    if( uals EQ NULL )
        NL_QUIT;

    omus = N_AllocReal1dArray( p, &SL );

    if( omus EQ NULL )
        NL_QUIT;

    vals = N_AllocReal1dArray( q, &SL );

    if( vals EQ NULL )
        NL_QUIT;

    omvs = N_AllocReal1dArray( q, &SL );

    if( omvs EQ NULL )
        NL_QUIT;

    /* Get power basis conversion matrices */

    N_InitRealMatrix( &pum );
    error = N_BezGetPowerConversionMatrix( p, &pum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &pvm );
    error = N_BezGetPowerConversionMatrix( q, &pvm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize for u-directional decomposition */

    isu = p;
    ieu = p + 1;
    k = -1;

    for ( i = 0; i <= p; i++ )
    {
        for ( col = 0; col <= m; col++ )
        {
            N_CopyCPt( Pw[i][col], &Sw[i][col] );
        }
    }

    /* Decompose in u-direction into Bezier strips */

    while( ieu LT r )
    {
        k = k + 1;

        /* Get knot multiplicity */

        i = ieu;

        while( ieu LT r AND U[ieu]EQ U[ieu + 1] )
            ieu++;
        mlu = ieu - i + 1;
        ru = p - mlu;

        /* Insert the knot */

        if( mlu LT p )
        {
            num = U[ieu] - U[isu];

            for ( i = p; i > mlu; i-- )
            {
                uals[i - mlu - 1] = num / (U[isu + i] - U[isu]);
                omus[i - mlu - 1] = 1.0 - uals[i - mlu - 1];
            }

            for ( i = 1; i <= ru; i++ )
            {
                su = mlu + i;
                save = ru - i;

                for ( j = p; j >= su; j-- )
                {
                    for ( col = 0; col <= m; col++ )
                    {
                        N_Combine2CPts( uals[j - su], Sw[j][col], omus[j - su], Sw[j - 1][col], &Sw[j][col] );
                    }
                }

                if( ieu LT r )
                {
                    for ( col = 0; col <= m; col++ )
                    {
                        N_CopyCPt( Sw[p][col], &NSw[save][col] );
                    }
                }
            }
        }

        /* Bezier strip completed. Initialize for */
        /* v-directional decomposition            */

        isv = q;
        iev = q + 1;
        l = -1;

        for ( i = 0; i <= p; i++ )
        {
            for ( j = 0; j <= q; j++ )
            {
                N_CopyCPt( Sw[i][j], &Bw[i][j] );
            }
        }

        /* Decompose in v-direction into Bezier patches */

        while( iev LT s )
        {
            l = l + 1;

            N_SrfGetCPtsAndKnots( spa[k][l], &aw, &A, &B );

            /* Get knot multiplicity */

            i = iev;

            while( iev LT s AND V[iev]EQ V[iev + 1] )
                iev++;
            mlv = iev - i + 1;
            rv = q - mlv;

            /* Insert the knot */

            if( mlv LT q )
            {
                num = V[iev] - V[isv];

                for ( i = q; i > mlv; i-- )
                {
                    vals[i - mlv - 1] = num / (V[isv + i] - V[isv]);
                    omvs[i - mlv - 1] = 1.0 - vals[i - mlv - 1];
                }

                for ( i = 1; i <= rv; i++ )
                {
                    sv = mlv + i;
                    save = rv - i;

                    for ( j = q; j >= sv; j-- )
                    {
                        for ( row = 0; row <= p; row++ )
                        {
                            N_Combine2CPts( vals[j - sv], Bw[row][j], omvs[j - sv], Bw[row][j - 1], &Bw[row][j] );
                        }
                    }

                    if( iev LT s )
                    {
                        for ( row = 0; row <= p; row++ )
                        {
                            N_CopyCPt( Bw[row][q], &NBw[row][save] );
                        }
                    }
                }
            }

            /* Patch completed - convert to power basis form */

            if( pflg EQ NL_NORMALIZED )
            {
                A[0] = 0.0;
                A[1] = 1.0;
                B[0] = 0.0;
                B[1] = 1.0;
            }
            else
            {
                A[0] = U[isu];
                A[1] = U[ieu];
                B[0] = V[isv];
                B[1] = V[iev];
            }

            error = N_BezSrfToPower( Bw,    /* in : Bezier surface control points                       */
                                     p,     /* in : Bezier surface degrees                              */
                                     q,     /* in : Bezier surface degrees                              */
                                     &pum,  /* in : Power basis conversion  matrices (in  u-directions) */
                                     &pvm,  /* in : Power basis conversion  matrices (in  v-directions) */
                                     A[0],  /* in : a of Bezier surface definition over [a,b] x [c,d]   */
                                     A[1],  /* in : b of Bezier surface definition over [a,b] x [c,d]   */
                                     B[0],  /* in : c of Bezier surface definition over [a,b] x [c,d]   */
                                     B[1],  /* in : d of Bezier surface definition over [a,b] x [c,d]   */
                                     aw) ;  /* out: Power basis coefficients                            */

            if( error EQ NL_YES )
                NL_OUT;

            /* Conversion completed - prepare for next patch */

            if( iev LT s )
            {
                for ( j = 0; j < rv; j++ )
                {
                    for ( row = 0; row <= p; row++ )
                    {
                        N_CopyCPt( NBw[row][j], &Bw[row][j] );
                    }
                }

                for ( j = rv; j <= q; j++ )
                {
                    for ( row = 0; row <= p; row++ )
                    {
                        N_CopyCPt( Sw[row][iev - q + j], &Bw[row][j] );
                    }
                }
            }
            isv = iev;
            iev = iev + 1;
        } /* End while for Bezier patches */

        /* Bezier strip decomposed - prepare for next strip */

        if( ieu LT r )
        {
            for ( i = 0; i < ru; i++ )
            {
                for ( col = 0; col <= m; col++ )
                {
                    N_CopyCPt( NSw[i][col], &Sw[i][col] );
                }
            }

            for ( i = ru; i <= p; i++ )
            {
                for ( col = 0; col <= m; col++ )
                {
                    N_CopyCPt( Pw[ieu - p + i][col], &Sw[i][col] );
                }
            }
        }
        isu = ieu;
        ieu = ieu + 1;
    } /* End while for Bezier strips */

    *kk = nsu - 1;
    *ll = nsv - 1;
    *spl = spa;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ConvertNurbsToPowerBasis */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine  converts  a  piecewise power 
     basis surface to NURBS form. If the output surface is initialized 
     to the NULL surface, memory is allocated. Otherwise it is checked 
     if enough memory is passed in. A typical calling example is:

       NL_SURFACE  ***spl, sur;
       NL_INDEX    k, l;
       NL_STACKS   SG;
       ...
       (define spl, k and l);
       ...
       N_SrfInitArrays(&sur);
       N_ConvertPiecesToNurbs(spl,k,l,&sur,&SG);

     spl[i][j], 0<=i<=k,  0<=j<=l is a  pointer to the  (i,j)-th power 
     basis  surface. The  power  basis  surface patch is  defined as a 
     NL_SURFACE  object; the  vector coefficients aw[i][j] are  stored in  
     place of the control points, and the knots  U[0], U[1]  and V[0], 
     V[1] represent the bounds of the rectangle over which the surface 
     is defined. IT  IS  ASSUMED  THAT  THE  NL_SURFACE  STRUCTURE sur IS 
     DEFINED IN THE CALLING ROUTINE.


   ACCESS:
   
     spl , input  ,  Piecewise  power basis  surface (array of surface 
                     pointers)
     k,l , input  ,  Highest  indexes in spl  array (number  of pieces
                     minus one in each direction)
     sur , output ,  NURBS surface
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ConvertPiecesToNurbs( NL_SURFACE *** spl, NL_INDEX k, NL_INDEX l, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ConvertPiecesToNurbs");

    NL_FLAG error = NL_NO, nov = NL_YES;

    NL_INDEX i, j, n, m, r, s, a, b, row, col, u_glo_par, v_glo_par, ii, jj;

    NL_DEGREE p, q;

    NL_REAL *A = NULL, *B, *U, *V, u0, ur, v0, vs, uu, vv;

    NL_RMATRIX ipu, ipv;

    NL_CPOINT ** aw, ** Pw, ** Bw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetDegrees( spl[0][0], &p, &q );

    n = (k + 1) * p;
    m = (l + 1) * q;
    r = (k + 2) * p + 1;
    s = (l + 2) * q + 1;

    /* Determine parameterization of surface patches (global or local) */

    N_SrfGetKnots( spl[0][0], &i, &j, &U, &V );
    u0 = U[0];
    v0 = V[0];
    ur = U[1];
    vs = V[1];

    u_glo_par = 1; /* global */

    for ( i = 1; i <= k; i++ )
    {
        N_SrfGetKnots( spl[i][0], &ii, &jj, &U, &V );

        if( U[0]NEQ ur )
        {
            u_glo_par = 0; /* local */
            break;
        }
        ur = U[1];
    }

    v_glo_par = 1; /* global */

    for ( j = 1; j <= l; j++ )
    {
        N_SrfGetKnots( spl[0][j], &ii, &jj, &U, &V );

        if( V[0]NEQ vs )
        {
            v_glo_par = 0; /* local */
            break;
        }
        vs = V[1];
    }

    /* Check for memory */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    Bw = N_AllocCPt2dArray( p, q, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    /* Get inverse of power basis conversion matrices */

    N_InitRealMatrix( &ipu );
    error = N_BezInversePowerMatrix( p, &ipu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &ipv );
    error = N_BezInversePowerMatrix( q, &ipv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize */

    row = 0;
    r = p;
    s = q;

    for ( i = 0; i <= p; i++ )
        U[i] = u0;

    for ( j = 0; j <= q; j++ )
        V[j] = v0;

    /********************************************************/
    /* Convert surface to NURBS form as follows:            */
    /*  (1) stitch patches together into piecewise Bezier,  */
    /*  (2) remove all removable knots.                     */
    /********************************************************/

    for ( i = 0; i <= k; i++ )
    {
        col = 0;

        for ( j = 0; j <= l; j++ )
        {
            N_SrfGetCPtsAndKnots( spl[i][j], &aw, &A, &B );

            error = N_BezSrfPowerToBez( aw, p, q, &ipu, &ipv, A[0], A[1], B[0], B[1], Bw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( a = 0; a <= p; a++ )
            {
                for ( b = 0; b <= q; b++ )
                {
                    N_CopyCPt( Bw[a][b], &Pw[row + a][col + b] );
                }
            }

            if( nov EQ NL_YES )
            {
                if( v_glo_par EQ 1 )
                {
                    for ( b = 1; b <= q; b++ )
                        V[s + b] = B[1];
                }
                else
                {
                    vv = B[1] - B[0];

                    for ( b = 1; b <= q; b++ )
                        V[s + b] = V[s] + vv;
                }

                s = s + q;
            }

            col = col + q;
        }

        nov = NL_NO;

        if( u_glo_par EQ 1 )
        {
            for ( a = 1; a <= p; a++ )
                U[r + a] = A[1];
        }
        else
        {
            uu = A[1] - A[0];

            for ( a = 1; a <= p; a++ )
                U[r + a] = U[r] + uu;
        }

        row = row + p;
        r = r + p;
    }

    U[r + 1] = U[r];
    V[s + 1] = V[s];

    /* Remove all removable knots */

    error = N_SrfRemoveAllKnots( sur, NL_MTOL, NL_UVDIR, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ConvertPiecesToNurbs */


/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine estimates  average  length  of  a
     surface in both u- and v-directions,  and  it  computes a bound on
     surface area by taking the product of the maximums  of  length  in
     both u- and v-directions. A typical calling example is:

       NL_SURFACE  sur;
       NL_REAL     A, lu, lv;
       ...
       (define sur);
       ...
       N_SrfGetAverageLen(&sur,&A,&lu,&lv);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     A     , output ,  Estimated bound on surface area
     lu,lv , output ,  Average lengths in u- and v-directions 
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfGetAverageLen( NL_SURFACE *sur, NL_REAL *A, NL_REAL *lu, NL_REAL *lv )
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

    /* Get maximum polygon lengths in u- and v-directions */

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
} /* end N_SrfGetAverageLen */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes a piecewise quadrilateral approximation to a 
     NURBS surface. An  absolute or relative planar deviation tolerance 
     can be specified. Points on the surface and/or their corresponding  
     parameter values can be returned. A typical calling example is:

       NL_SURFACE  sur;
       NL_POINT    **P;
       NL_REAL     *u, *v, tol;
       NL_INDEX    n, m;
       NL_FLAG     tfl, rfl;
       NL_STACKS   SG;
       ...
       (define sur, get tol);
       ...
       N_ApproxSrfWithQuadSrf(&sur,tol,NL_RELATIVE,NL_POINTS,&P,&u,&v,&n,&m,&SG);

     MEMORY FOR P, u AND v IS ALLOCATED INSIDE THE ROUTINE!


   ACCESS:
   
     sur  , input  ,  NURBS surface
     tol  , input  ,  Planar deviation tolerance; the quadrilaterals do
                      not deviate from a plane more than tol
     tfl  , input  ,  Tolerance flag:
                        NL_ABSOLUTE: tol is taken as is
                        NL_RELATIVE: tol is scaled by the  diagonal of the
                                  bounding box of sur
     rfl  , input  ,  Return flag:
                        NL_POINTS    : return points on the surface
                        NL_PARAMETERS: return parameters only
                        NL_BOTH      : return both
     P    , output ,  Vertices of the quadrilaterals
     u,v  , output ,  Parameters  corresponding to P[i][j]
     n,m  , output ,  Highest inds in P[0..n][0..m], u[0..n], v[0..m]
     SG   , input  ,  P's, u's and v's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxSrfWithQuadSrf( NL_SURFACE *sur, NL_REAL tol, NL_FLAG tfl, NL_FLAG rfl, NL_POINT *** P, NL_REAL ** u, NL_REAL ** v, NL_INDEX *n, NL_INDEX *m, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_ApproxSrfWithQuadSrf"); */
    NL_PRIVATE NL_INDEX ch = 40;
    NL_PRIVATE NL_INDEX uh = 100;
    NL_PRIVATE NL_INDEX vh = 100;

    NL_FLAG flt, dir, error = NL_NO;

    NL_INDEX ns, ms, rs, ss, i, j, ts, tu, tv, first, last, mid;

    NL_DEGREE ps, qs;

    NL_REAL *S_ul, *S_ur, *S_vb, *S_vt, *US, *VS, *uu, *vv, *uo, *vo, ul, ur, vb, vt, um, vm, dg;

    NL_POINT ** Q;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &ns, &ms, &rs, &ss );
    N_SrfGetDegrees( sur, &ps, &qs );
    N_SrfGetKnots( sur, &rs, &ss, &US, &VS );

    /* Adjust tolerance */

    if( tfl EQ NL_RELATIVE )
    {
        error = N_SrfMaxDiagDistBBox( sur, &dg );

        if( error EQ NL_YES )
            NL_OUT;

        tol = dg * tol;
    }

    /* Get memory */

    S_ul = N_AllocReal1dArray( ch, &SL );

    if( S_ul EQ NULL )
        NL_QUIT;

    S_ur = N_AllocReal1dArray( ch, &SL );

    if( S_ur EQ NULL )
        NL_QUIT;

    S_vb = N_AllocReal1dArray( ch, &SL );

    if( S_vb EQ NULL )
        NL_QUIT;

    S_vt = N_AllocReal1dArray( ch, &SL );

    if( S_vt EQ NULL )
        NL_QUIT;

    uu = N_AllocReal1dArray( uh, &SL );

    if( uu EQ NULL )
        NL_QUIT;

    vv = N_AllocReal1dArray( vh, &SL );

    if( vv EQ NULL )
        NL_QUIT;

    error = N_AllocSrfArrays( &surA, ns, ms, ps, qs, rs, ss, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize */

    S_ul[0] = US[0];
    S_ur[0] = US[rs];
    S_vb[0] = VS[0];
    S_vt[0] = VS[ss];

    uu[0] = US[0];
    uu[1] = US[rs];
    vv[0] = VS[0];
    vv[1] = VS[ss];

    ts = 0;
    tu = 1;
    tv = 1;

    /* Subdivide surface */

    while( ts GE 0 )
    {
        /* Pop stack */

        ul = S_ul[ts];
        ur = S_ur[ts];
        vb = S_vb[ts];
        vt = S_vt[ts];

        ts--;

        /* Extract surface patch */

        N_SrfSetSizeIndices( &surA, ns, ms, ps, qs, rs, ss );
        error = N_SrfExtractPatch( sur, ul, ur, vb, vt, &surA, SG, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Check flatness */

        error = N_SrfIsFlatCheap( &surA, tol, &flt, &dir );

        if( error EQ NL_YES )
            NL_OUT;

        /* If not flat, subdivide */

        if( flt EQ NL_NO )
        {
            /* Check for array overflow */

            if( ts + 2 GT ch )
            {
                error = N_Realloc1dRealArray( &S_ul, ch, ch + ch, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_Realloc1dRealArray( &S_ur, ch, ch + ch, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_Realloc1dRealArray( &S_vb, ch, ch + ch, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_Realloc1dRealArray( &S_vt, ch, ch + ch, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                ch += ch;
            }

            if( dir EQ NL_UDIR AND tv + 1 GT vh )
            {
                error = N_Realloc1dRealArray( &vv, vh, vh + vh, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                vh += vh;
            }

            if( dir EQ NL_VDIR AND tu + 1 GT uh )
            {
                error = N_Realloc1dRealArray( &uu, uh, uh + uh, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                uh += uh;
            }

            /* Compute subdivision points and put new rectangles on the stack */

            if( dir EQ NL_UDIR )
            {
                vm = 0.5 *( vb + vt );

                first = 0;
                last = tv;

                while( first LE last )
                {
                    mid = (first + last) / 2;

                    if( fabs( vv[mid] - vm )LT NL_PTOL )
                        break;

                    if( vm LT vv[mid] )
                        last = mid - 1;
                    else
                        first = mid + 1;
                }

                if( first GT last )
                {
                    for ( j = tv; j >= last + 1; j-- )
                        vv[j + 1] = vv[j];
                    vv[last + 1] = vm;
                    tv++;
                }

                S_ul[ts + 1] = ul;
                S_ur[ts + 1] = ur;
                S_vb[ts + 1] = vb;
                S_vt[ts + 1] = vm;
                S_ul[ts + 2] = ul;
                S_ur[ts + 2] = ur;
                S_vb[ts + 2] = vm;
                S_vt[ts + 2] = vt;
            }
            else
            {
                um = 0.5 *( ul + ur );

                first = 0;
                last = tu;

                while( first LE last )
                {
                    mid = (first + last) / 2;

                    if( fabs( uu[mid] - um )LT NL_PTOL )
                        break;

                    if( um LT uu[mid] )
                        last = mid - 1;
                    else
                        first = mid + 1;
                }

                if( first GT last )
                {
                    for ( i = tu; i >= last + 1; i-- )
                        uu[i + 1] = uu[i];
                    uu[last + 1] = um;
                    tu++;
                }

                S_ul[ts + 1] = ul;
                S_ur[ts + 1] = um;
                S_vb[ts + 1] = vb;
                S_vt[ts + 1] = vt;
                S_ul[ts + 2] = um;
                S_ur[ts + 2] = ur;
                S_vb[ts + 2] = vb;
                S_vt[ts + 2] = vt;
            }

            ts += 2;
        }
    }

    /* Create output */

    if( rfl EQ NL_POINTS OR rfl EQ NL_BOTH )
    {
        Q = N_AllocPt2dArray( tu, tv, SG );

        if( Q EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= tu; i++ )
        {
            for ( j = 0; j <= tv; j++ )
            {
                error = N_SrfEvalPt( sur, uu[i], vv[j], NL_LEFT, NL_LEFT, &Q[i][j] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }

        *P = Q;
    }

    if( rfl EQ NL_PARAMETERS OR rfl EQ NL_BOTH )
    {
        uo = N_AllocReal1dArray( tu, SG );

        if( uo EQ NULL )
            NL_QUIT;

        vo = N_AllocReal1dArray( tv, SG );

        if( vo EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= tu; i++ )
            uo[i] = uu[i];

        for ( j = 0; j <= tv; j++ )
            vo[j] = vv[j];

        *u = uo;
        *v = vo;
    }

    *n = tu;
    *m = tv;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ApproxSrfWithQuadSrf */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  geometry processing  routine  reparametrizes a  NURBS surface   
     using a rational linear function of the form:

                            alf*u+bet
                 s = g(u) = ---------
                            gam*u+del
 
     It is assumed that the following conditions hold:

           (1) alf*del-bet*gam >  0
           (2) gam*u+del       != 0
           (3) gam*s-alf       != 0

     where u and s are the old and new parameters, respectively. If  the 
     output surface is initialized to NULL, memory to store new  control 
     points and knots is allocated. If the output surface is the same as 
     the input surface, reparametrization is  in place  and the original 
     surface is destroyed. A typical calling example is: 

       NL_SURFACE  surP, surQ;
       NL_REAL     alf, bet, gam, del;
       NL_STACKS   SG;
       ...
       (define surP, get alf, bet, gam & del);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfReparmRat(&surP,alf,bet,gam,del,NL_UDIR,&surQ,&SG);
       N_SrfReparmRat(&surP,alf,bet,gam,del,NL_VDIR,&surP,&SG);

     If memory is  available, surQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in surQ's  
     knot vectors  and control net objects. IF THE INPUT NL_SURFACE IS NON 
     RATIONAL, AFTER REPARAMETRIZATION IT BECOMES A RATIONAL NL_SURFACE.


   ACCESS:
   
     surP    , input  ,  NURBS surface
     alf,bet , input  ,  Numerator's coefficients
     gam,del , input  ,  Denominator's coefficients
     dir     , input  ,  Flag:
                           NL_UDIR: reparametrize in u-direction
                           NL_VDIR: reparametrize in v-direction
     surQ    , output ,  Surface after reparametrization
     SG      , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReparmRat( NL_SURFACE *surP, NL_REAL alf, NL_REAL bet, NL_REAL gam, NL_REAL del, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReparmRat");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, ** w, num, den, fact;

    NL_CPOINT ** Pw, ** Qw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check coefficients */

    if( (alf * del - bet * gam)LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    /* Get local notation */

    if( NOT N_IsSrfRat( surP ) )
        N_SrfNonRatToRat( surP );

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Get scaled control points and weights */

    if( surP NEQ surQ )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
            {
                N_CopyCPt( Pw[i][j], &Qw[i][j] );
            }
        }
    }

    w = N_AllocReal2dArray( n, m, &SL );

    if( w EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtGetW( Qw[i][j], &w[i][j] );
            fact = 1.0 / w[i][j];
            N_ScaleCPt( fact, Qw[i][j], &Qw[i][j] );
        }
    }

    /* Compute new weights and knot vectors */

    switch( dir )
    {
        case NL_UDIR:
            for ( i = 0; i <= r; i++ )
            {
                num = alf * UP[i] + bet;
                den = gam * UP[i] + del;

                if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                    NL_ERROR( NL_INP_ERR );

                UQ[i] = num / den;
            }

            if( surP NEQ surQ )
            {
                for ( j = 0; j <= s; j++ )
                    VQ[j] = VP[j];
            }

            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= n; i++ )
                {
                    fact = 1.0;

                    for ( k = 1; k <= p; k++ )
                        fact = fact * (gam * UQ[i + k] - alf);

                    if( fact LT 0.0 )
                        fact = -fact;

                    w[i][j] = w[i][j] * fact;
                    N_ScaleCPt( w[i][j], Qw[i][j], &Qw[i][j] );
                }
            }
            break;

        case NL_VDIR:
            for ( j = 0; j <= s; j++ )
            {
                num = alf * VP[j] + bet;
                den = gam * VP[j] + del;

                if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                    NL_ERROR( NL_INP_ERR );

                VQ[j] = num / den;
            }

            if( surP NEQ surQ )
            {
                for ( i = 0; i <= r; i++ )
                    UQ[i] = UP[i];
            }

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    fact = 1.0;

                    for ( k = 1; k <= q; k++ )
                        fact = fact * (gam * VQ[j + k] - alf);

                    if( fact LT 0.0 )
                        fact = -fact;

                    w[i][j] = w[i][j] * fact;
                    N_ScaleCPt( w[i][j], Qw[i][j], &Qw[i][j] );
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReparmRat */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine reparametrizes NURBS surfaces with
     respect to the arc  length. That is, in each  direction the routine 
     computes the arc  length of iso-curves  extracted at the nodes, and 
     averages the  results of the  arc length  parametrizations of these
     curves. The reparametrization does not change the surface geometri-
     cally, however, it changes the  derivatives. If  the output surface 
     is  initialized to  NULL, memory  to store  new control  points and 
     knots is  allocated. If the output surface is the same as the input 
     surface,  reparametrization is  in place and the original knots are 
     destroyed. A typical calling example is: 

       NL_SURFACE  surP, surQ;
       NL_REAL     tol;
       NL_STACKS   SG;
       ...
       (define surP and get tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfReparamArcLength(&surP,tol,&surQ,&SG);
       N_SrfReparamArcLength(&surP,tol,&surP,&SG);

     If memory is  available, surQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in surQ's  
     knot vector and control net objects. This routine is  particularly 
     suitable to reparametrize piecewise Bezier surfaces given in NURBS
     form with degree-fold  multiple knots.


   ACCESS:
   
     surP , input  ,  NURBS surface
     tol  , input  ,  Relative tolerance for arc length computation   
     surQ , output ,  Surface after reparametrization
     SG   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReparamArcLength( NL_SURFACE *surP, NL_REAL tol, NL_SURFACE *surQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReparamArcLength");

    NL_FLAG error = NL_NO;

    NL_INDEX *mlu, *mlv, i, j, k, l, n, m, r, s, nsu, nsv;

    NL_DEGREE p, q, pq;

    NL_REAL *UP, *VP, *UQ, *VQ, *UA, *u, *v, *un, *vn, t;

    NL_CPOINT ** Pw, ** Qw;

    NL_KNOTVECTOR *knu, *knv;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Allocate memory */

    N_BasisGetSpanCount( knu, p, &nsu );
    N_BasisGetSpanCount( knv, q, &nsv );

    pq = NL_MAX( p, q );
    error = N_AllocCrvArrays( &curA, NL_MAX( n, m ), pq, NL_MAX( r, s ), &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( &curA, &k, &UA );

    u = N_AllocReal1dArray( nsu - 1, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( nsv - 1, &SL );

    if( v EQ NULL )
        NL_QUIT;

    un = N_AllocReal1dArray( n, &SL );

    if( un EQ NULL )
        NL_QUIT;

    vn = N_AllocReal1dArray( m, &SL );

    if( vn EQ NULL )
        NL_QUIT;

    mlu = N_AllocInt1dArray( nsu - 1, &SL );

    if( mlu EQ NULL )
        NL_QUIT;

    mlv = N_AllocInt1dArray( nsv - 1, &SL );

    if( mlv EQ NULL )
        NL_QUIT;

    /********************************/
    /* Reparametrize in u-direction */
    /********************************/

    /* Get v-nodes */

    N_BasisFindIndexNodeArray( knv, q, vn );

    /* Get multiplicities of distinct u-knots */

    k = p + 1;
    i = 0;

    while( k LT r )
    {
        l = k;

        while( k LT r AND UP[k]EQ UP[k + 1] )
            k++;

        mlu[i] = k - l + 1;

        i++;
        k++;
    }

    /* For each v-node, reparametrize iso-curve and sum up knots */

    for ( i = 0; i <= nsu - 1; i++ )
        u[i] = 0.0;

    for ( j = 0; j <= m; j++ )
    {
        error = N_SrfExtractIsoCrv( surP, vn[j], NL_UDIR, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfReparamMultKnots( &curA, tol, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        k = p + 1;
        i = 0;

        while( k LT r )
        {
            for ( l = 1; l <= mlu[i] - 1; l++ )
                k++;
            u[i] += UA[k];

            i++;
            k++;
        }
    }

    /********************************/
    /* Reparametrize in v-direction */
    /********************************/

    pq = NL_MAX( p, q );
    N_CrvSetSizeIndices( &curA, NL_MAX( n, m ), pq, NL_MAX( r, s ) );

    /* Get u-nodes */

    N_BasisFindIndexNodeArray( knu, p, un );

    /* Get multiplicities of distinct v-knots */

    k = q + 1;
    j = 0;

    while( k LT s )
    {
        l = k;

        while( k LT s AND VP[k]EQ VP[k + 1] )
            k++;

        mlv[j] = k - l + 1;

        j++;
        k++;
    }

    /* For each u-node, reparametrize iso-curve and sum up knots */

    for ( j = 0; j <= nsv - 1; j++ )
        v[j] = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        error = N_SrfExtractIsoCrv( surP, un[i], NL_VDIR, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfReparamMultKnots( &curA, tol, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        k = q + 1;
        j = 0;

        while( k LT s )
        {
            for ( l = 1; l <= mlv[j] - 1; l++ )
                k++;
            v[j] += UA[k];

            j++;
            k++;
        }
    }

    /* Compute output knot vectors */

    for ( i = 0; i <= p; i++ )
        UQ[i] = 0.0;

    i = p + 1;

    for ( k = 0; k <= nsu - 1; k++ )
    {
        t = u[k] / ((NL_REAL)m + 1.0);

        for ( l = 1; l <= mlu[k]; l++ )
            UQ[i++] = t;
    }

    for ( j = 0; j <= q; j++ )
        VQ[j] = 0.0;

    j = q + 1;

    for ( k = 0; k <= nsv - 1; k++ )
    {
        t = v[k] / ((NL_REAL)n + 1.0);

        for ( l = 1; l <= mlv[k]; l++ )
            VQ[j++] = t;
    }

    /* Output control points if necessary */

    if( surP NEQ surQ )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_CopyCPt( Pw[i][j], &Qw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReparamArcLength */

/*******************************************************************//**


   DESCRIPTION:

     Given a  set of  points  and a  surface, this geometry  processing 
     routine  projects each of  them onto  the surface and  returns the
     coresponding parameters. The projection is done by decomposing the
     surface into  rectangular regions  and projecting  the points onto 
     small  quadrilaterals. The  routine can  also be  used to thin the 
     data set, i.e. points closest to the corners of the quadrilaterals
     are used. This routine is designed primarily for use in parameter-
     izing unstructured point clouds (e.g. called by N_FitCalcSrfParamsBoundarySrf), and the
     surface is assumed to be simple, i.e. projectable onto a plane. In
     order to use it for more complicated surfaces, they should be sub-
     divided  prior to calling this routine. A typical calling example:


       NL_SURFACE  sur;
       NL_POINT    *P, *T;
       NL_REAL     *u, *v, tol;
       NL_INDEX    n, m;
       NL_STACKS   SG;
       ...
       (define sur, get P and tol);
       ...
       N_SrfProjectPts(&sur,P,n,NL_YES,NL_NO,tol,&T,&u,&v,&m,&SG);

     MEMORY FOR T, u AND v IS ALLOCATED  INSIDE ROUTINE! IF  THE FILTER
     NL_FLAG IS ON (fif=NL_YES), THE NL_PARAMETER  DOMAIN IS OFFSET  INWARD BY A
     TOLERANCE  EQUALS TO 0.5  PERCENT OF THE  RATIO OF  THE  NL_PARAMETER 
     SPANS AND THE NL_AVERAGE (u AND v) LENGTHS OF THE NL_SURFACE. THIS PEELS
     THE NL_POINT OFF THE BOUNDARIES AND THEIR IMMEDIATE VICINITY.

     IF FIF=NL_NO,  THE RETURNED U,V VALUES MAY LIE OUTSIDE THE U,V DOMAIN
     FOR NL_POINTS WHICH ARE NOT ON OR ABOVE THE NL_SURFACE, I.E. NL_POINTS THAT
     DO NOT PROJECT DOWN TO THE NL_SURFACE ALONG A NORMAL NL_VECTOR.


   ACCESS:
   
     sur  , input  ,  NURBS surface
     P    , input  ,  Points to be projected
     n    , input  ,  Highest index in array P
     thf  , input  ,  Flag:
                        NL_YES: thin  data set;  tol is  for decomposition
                             for thinning and for projection
                        NL_NO : do not thin data set; tol is for  decompo-
                             sition for projection
     fif  , input  ,  Flag:
                        NL_YES: points that project to the  outside of the
                             modified domain must be filterd out
                        NL_NO : do not filter  points out; all points must 
                             be projected
     tol  , input  ,  Surface  flatness tolerance;  MUST BE A RELATIVE 
                      TOLERANCE!! 0.1%, I.E. 0.001 IS SUGGESTED.
     T    , output ,  Thinned points or  subset of points P sucessfully 
                      projected onto sur
     u,v  , output ,  Parameters  corresponding to T[i], i=0,...,m
     m    , output ,  Highest index in T[0..m], u[0..m], v[0..m]
     SG   , input  ,  T's, u's and v's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfProjectPts( NL_SURFACE *sur, NL_POINT *P, NL_INDEX n, NL_FLAG thf, NL_FLAG fif, NL_REAL tol, NL_POINT ** T, NL_REAL ** u, NL_REAL ** v, NL_INDEX *m, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfProjectPts"); */
    NL_PRIVATE NL_REAL mat = 1.0;
    NL_PRIVATE NL_REAL btp = 0.005;
    NL_PRIVATE NL_INDEX ces = 10;

    NL_FLAG *vst, pfl, error = NL_NO;

    NL_INDEX *** cell, ** top, ** tm, i, j, k, l, r, s, kk, ll, xres, yres, ic, jc, il, ih, jl, jh, ip, t, rr, ss;

    NL_REAL *U, *V, *uq, *vq, *ua, *va, *ub, *vb, *uc, *vc, *uo, *vo, *da, *db, *dc, uh, vh, x, y, z, xl, xr, yb, yt, xd, yd, size, d, dsh2, dis2, dmin2, x1, y1, x2, y2, up, vp, u0, u1, v0, v1, u2, u3, v2, v3, lu, lv, tlu, tlv, dum;

    NL_POINT ** Q, ** QT, *PT, *PO, O, PP;

    NL_VECTOR X, Y, Z;

    NL_RMATRIX rma;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Decompose surface */

    error = N_ApproxSrfWithQuadSrf( sur, tol, NL_RELATIVE, NL_BOTH, &Q, &uq, &vq, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get maximum area plane */

    error = N_MaxAreaPlane3dPts( Q, k, l, mat, &Z );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get local coordinate system on maximum area plane */

    N_SrfGetKnots( sur, &rr, &ss, &U, &V );

    error = N_SrfGetAverageLen( sur, &dum, &lu, &lv );

    if( error EQ NL_YES )
        NL_OUT;

    tlu = btp * ((U[rr] - U[0]) / lu);
    tlv = btp * ((V[ss] - V[0]) / lv);

    uh = 0.5 *( U[rr] + U[0] );
    vh = 0.5 *( V[ss] + V[0] );

    u2 = U[0] + tlu;
    u3 = U[rr] - tlu;
    v2 = V[0] + tlv;
    v3 = V[ss] - tlv;

    error = N_SrfEvalPt( sur, uh, vh, NL_LEFT, NL_LEFT, &O );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfEvalPt( sur, uh, V[0], NL_LEFT, NL_LEFT, &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfEvalPt( sur, U[rr], vh, NL_LEFT, NL_LEFT, &Y );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDiff( X, O, &X );
    N_VectorDiff( Y, O, &Y );
    N_VectorMagnitude( X, &uh );
    N_VectorMagnitude( Y, &vh );

    if( uh GT vh )
    {
        N_VectorCross( Z, X, &Y );
        N_VectorCross( Y, Z, &X );
    }
    else
    {
        N_VectorCross( Y, Z, &X );
        N_VectorCross( Z, X, &Y );
    }

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    /* Transform maximum area plane to [x-y] plane */

    N_InitRealMatrix( &rma );
    error = N_CreateTransformMatrixFromAxes( O, X, Y, Z, NL_ZERO, NL_UNITX, NL_UNITY, NL_UNITZ, &rma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    PT = N_AllocPt1dArray( n, &SL );

    if( PT EQ NULL )
        NL_QUIT;

    QT = N_AllocPt2dArray( k, l, &SL );

    if( QT EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
        N_TransformPt( P[i], &rma, &PT[i] );

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
            N_TransformPt( Q[i][j], &rma, &QT[i][j] );
    }

    /**************************************************/
    /* Set up data structure for fast range searching */
    /**************************************************/

    /* Get min-max box */

    N_PtToXYZ( QT[0][0], &x, &y, &z );

    xl = xr = x;
    yb = yt = y;

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            N_PtToXYZ( QT[i][j], &x, &y, &z );

            if( x LT xl )
                xl = x;

            if( x GT xr )
                xr = x;

            if( y LT yb )
                yb = y;

            if( y GT yt )
                yt = y;
        }
    }

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( PT[i], &x, &y, &z );

        if( x LT xl )
            xl = x;

        if( x GT xr )
            xr = x;

        if( y LT yb )
            yb = y;

        if( y GT yt )
            yt = y;
    }

    /* Get cell size and allocate memory */

    xl -= NL_MTOL;
    xr += NL_MTOL;
    yb -= NL_MTOL;
    yt += NL_MTOL;

    xd = fabs( xr - xl );
    yd = fabs( yt - yb );

    size = sqrt( (xd * yd) / (((NL_REAL)k + 1.0) * ((NL_REAL)l + 1.0)) );
    xres = (NL_INDEX)(xd / size);
    yres = (NL_INDEX)(yd / size);

    top = N_AllocInt2dArray( xres, yres, &SL );

    if( top EQ NULL )
        NL_QUIT;

    tm = N_AllocInt2dArray( xres, yres, &SL );

    if( tm EQ NULL )
        NL_QUIT;

    cell = N_AllocIntPtr2dArray( xres, yres, &SL );

    if( cell EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= xres; i++ )
    {
        for ( j = 0; j <= yres; j++ )
        {
            top[i][j] = -1;
            tm[i][j] = ces;

            cell[i][j] = N_AllocInt1dArray( ces, &SL );

            if( cell[i][j]EQ NULL )
                NL_QUIT;
        }
    }

    /* Put each point in a cell */

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( PT[i], &x, &y, &z );

        ic = (NL_INDEX)((x - xl) / size);
        jc = (NL_INDEX)((y - yb) / size);
        ip = top[ic][jc] + 1;

        if( ip GT tm[ic][jc] )
        {
            error = N_Realloc1dIntArray( &cell[ic][jc], tm[ic][jc], tm[ic][jc] + ces, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            tm[ic][jc] += ces;
        }

        cell[ic][jc][ip] = i;
        top[ic][jc]++;
    }

    /******************************************/
    /* Now project points and find parameters */
    /******************************************/

    /* Allocate memory and initialize */

    da = N_AllocReal1dArray( n, &SL );

    if( da EQ NULL )
        NL_QUIT;

    db = N_AllocReal1dArray( n, &SL );

    if( db EQ NULL )
        NL_QUIT;

    dc = N_AllocReal1dArray( n, &SL );

    if( dc EQ NULL )
        NL_QUIT;

    ua = N_AllocReal1dArray( n, &SL );

    if( ua EQ NULL )
        NL_QUIT;

    va = N_AllocReal1dArray( n, &SL );

    if( va EQ NULL )
        NL_QUIT;

    ub = N_AllocReal1dArray( n, &SL );

    if( ub EQ NULL )
        NL_QUIT;

    vb = N_AllocReal1dArray( n, &SL );

    if( vb EQ NULL )
        NL_QUIT;

    uc = N_AllocReal1dArray( n, &SL );

    if( uc EQ NULL )
        NL_QUIT;

    vc = N_AllocReal1dArray( n, &SL );

    if( vc EQ NULL )
        NL_QUIT;

    vst = N_AllocFlag1dArray( n, &SL );

    if( vst EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        da[i] = db[i] = dc[i] = NL_BIGD;
        vst[i] = NL_NO;
    }

    /* Mark points to be used after thinning */

    if( thf EQ NL_YES )
    {
        for ( i = 0; i <= k; i++ )
        {
            for ( j = 0; j <= l; j++ )
            {
                /* Get cell indexes */

                N_PtToXYZ( QT[i][j], &x, &y, &z );
                ic = (NL_INDEX)((x - xl) / size);
                jc = (NL_INDEX)((y - yb) / size);

                x1 = xl + ic * size;
                x2 = x1 + size;
                y1 = yb + jc * size;
                y2 = y1 + size;

                /* Get shortest distance from point to cell walls */

                dsh2 = (x - x1) * (x - x1);

                if( (x2 - x) * (x2 - x)LT dsh2 )
                    dsh2 = (x2 - x) * (x2 - x);

                if( (y - y1) * (y - y1)LT dsh2 )
                    dsh2 = (y - y1) * (y - y1);

                if( (y2 - y) * (y2 - y)LT dsh2 )
                    dsh2 = (y2 - y) * (y2 - y);

                dmin2 = NL_BIGD;

                /* Now search for the nearest neighbor */

                ll = 0;
                kk = -1;

                while( dmin2 GT dsh2 )
                {
                    il = NL_MAX( 0, ic - ll );
                    jl = NL_MAX( 0, jc - ll );
                    ih = NL_MIN( xres, ic + ll );
                    jh = NL_MIN( yres, jc + ll );

                    for ( r = il; r <= ih; r++ )
                    {
                        for ( s = jl; s <= jh; s++ )
                        {
                            /* Check distances for all points in the cell */

                            for ( t = 0; t <= top[r][s]; t++ )
                            {
                                ip = cell[r][s][t];

                                N_DistSqPtPt2d( QT[i][j], PT[ip], &dis2 );

                                if( dis2 LT dmin2 )
                                {
                                    dmin2 = dis2;
                                    kk = ip;
                                }
                            }
                        }
                    }

                    dsh2 += size * size;
                    ll++;
                }

                if( kk GT - 1 )
                    vst[kk] = NL_YES;
            }
        }
    }

    /* For each quadrilateral do */

    for ( i = 0; i < k; i++ )
    {
        for ( j = 0; j < l; j++ )
        {
            /* Find covering cells */

            il = xres;
            ih = 0;
            jl = yres;
            jh = 0;

            u0 = uq[i];
            u1 = uq[i + 1];
            v0 = vq[j];
            v1 = vq[j + 1];

            N_PtToXYZ( QT[i][j], &x, &y, &z );
            ic = (NL_INDEX)((x - xl) / size);
            jc = (NL_INDEX)((y - yb) / size);

            il = NL_MIN( ic, il );
            ih = NL_MAX( ic, ih );
            jl = NL_MIN( jc, jl );
            jh = NL_MAX( jc, jh );

            N_PtToXYZ( QT[i + 1][j], &x, &y, &z );
            ic = (NL_INDEX)((x - xl) / size);
            jc = (NL_INDEX)((y - yb) / size);

            il = NL_MIN( ic, il );
            ih = NL_MAX( ic, ih );
            jl = NL_MIN( jc, jl );
            jh = NL_MAX( jc, jh );

            N_PtToXYZ( QT[i][j + 1], &x, &y, &z );
            ic = (NL_INDEX)((x - xl) / size);
            jc = (NL_INDEX)((y - yb) / size);

            il = NL_MIN( ic, il );
            ih = NL_MAX( ic, ih );
            jl = NL_MIN( jc, jl );
            jh = NL_MAX( jc, jh );

            N_PtToXYZ( QT[i + 1][j + 1], &x, &y, &z );
            ic = (NL_INDEX)((x - xl) / size);
            jc = (NL_INDEX)((y - yb) / size);

            il = NL_MIN( ic, il );
            ih = NL_MAX( ic, ih );
            jl = NL_MIN( jc, jl );
            jh = NL_MAX( jc, jh );

            /* For each cell do */

            for ( r = il; r <= ih; r++ )
            {
                for ( s = jl; s <= jh; s++ )
                {
                    /* Find points in the cell */

                    for ( t = 0; t <= top[r][s]; t++ )
                    {
                        ip = cell[r][s][t];

                        /* If not marked for thinning, skip it */

                        if( thf EQ NL_YES AND vst[ip]EQ NL_NO )
                            continue;

                        /* Project point onto the quadrilateral */

                        error = N_ProjectPtQuad( Q[i][j], Q[i + 1][j], Q[i][j + 1], Q[i + 1][j + 1], P[ip], NL_MTOL, u0, u1, v0, v1, &PP, &up, &vp, &d, &pfl );

                        if( error EQ NL_YES )
                            NL_OUT;

                        /* If projection is successful */

                        if( pfl EQ NL_YES )
                        {
                            if( up LE u2 OR up GE u3 OR vp LE v2 OR vp GE v3 )
                            {
                                if( d LT dc[ip] )
                                {
                                    uc[ip] = up;
                                    vc[ip] = vp;
                                    dc[ip] = d;
                                }
                            }
                            else if( up GE u0 AND up LE u1 AND vp GE v0 AND vp LE v1 )
                            {
                                if( d LT da[ip] )
                                {
                                    ua[ip] = up;
                                    va[ip] = vp;
                                    da[ip] = d;
                                }
                            }
                            else
                            {
                                if( d LT db[ip] )
                                {
                                    ub[ip] = up;
                                    vb[ip] = vp;
                                    db[ip] = d;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /* Filter on/out-side points */

    jh = -1;

    for ( i = 0; i <= n; i++ )
    {
        vst[i] = NL_YES;

        if( da[i]EQ NL_BIGD AND db[i]EQ NL_BIGD AND dc[i]EQ NL_BIGD )
            vst[i] = NL_NO;

        if( fif EQ NL_YES AND dc[i]NEQ NL_BIGD )
            vst[i] = NL_NO;

        if( vst[i]EQ NL_YES )
            jh++;
    }

    /* Create output */

    PO = N_AllocPt1dArray( jh, SG );

    if( PO EQ NULL )
        NL_QUIT;

    uo = N_AllocReal1dArray( jh, SG );

    if( uo EQ NULL )
        NL_QUIT;

    vo = N_AllocReal1dArray( jh, SG );

    if( vo EQ NULL )
        NL_QUIT;

    j = 0;

    for ( i = 0; i <= n; i++ )
    {
        if( vst[i]EQ NL_YES AND j LE jh )
        {
            if( da[i]NEQ NL_BIGD )
            {
                uo[j] = ua[i];
                vo[j] = va[i];
            }
            else if( db[i]NEQ NL_BIGD )
            {
                uo[j] = ub[i];
                vo[j] = vb[i];
            }
            else if( dc[i]NEQ NL_BIGD )
            {
                uo[j] = uc[i];
                vo[j] = vc[i];
            }

            N_VectorCopy( P[i], &PO[j] );

            j++;
        }
    }

    *T = PO;
    *u = uo;
    *v = vo;
    *m = jh;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfProjectPts */
