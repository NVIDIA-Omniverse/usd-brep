// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************/
/* SrfAdv.c : Advanced Function Definitions that act on NL_SURFACE objects */
/************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */
#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */
#include <NL_FuncsAdv.h>    /* Advanced NL_CFUN, NL_CVALUE, NL_SFUN, NL_SVALUE, NL_VFUN, and NL_VVALUE functions */



/* ------------------ */
/* File Local Globals */
/* ------------------ */
NL_PRIVATE NL_REAL KN_TOL = 1.0e-6;

/* ----------------------- */
/* File Local declarations */
/* ----------------------- */

NL_FLAG ST_surxsdr(NL_SURFACE* surP, NL_REAL dist, NL_FLAG dflg, NL_FLAG end, NL_FLAG cont, NL_SURFACE* surQ,
    NL_STACKS* SP, NL_STACKS* SQ, int depth);


/* NL_PRIVATE NL_REAL NOREM = 1.0e+25; */
/* NL_PRIVATE NL_REAL sto = 1.0e-05; */

/*      NL_PRIVATE NL_REAL cto          = 1.0e-05; */
/*      NL_PRIVATE NL_REAL eps          = 1.0e-03; */
#define BAD_INPUT  NL_ERROR(NL_INP_ERR);

NL_FLAG ST_GetRotationRateOfChange   /* eff: find [x'|y'|z'] for sweep trajectory*/
( NL_CURVE *traj,                    /* in : sweep curve trajectory  */
  NL_PARAMETER u,                    /* in : parameter being examined */
  NL_CURVE *curZ,                    /* in : optional curve to compute Z and dZ or NULL  */
  NL_VECTOR *Z,                      /* in : alternative: associated Z value for given u */
  NL_VECTOR *prevZ,                  /* in :              discrete Z prior to Z or NULL  */
  NL_PARAMETER prev_u,               /* in :              u associated with prevZ      */
  NL_VECTOR *nextZ,                  /* in :              discrete Z after Z or NULL  */
  NL_PARAMETER next_u,               /* in :              u associated with nextZ     */
  NL_RMATRIX *drma );                /* out: rotation rate of change - pre-allocated  */
                                     /*       ordered = [ dx[0] | dy[0] | dz[0] ]     */
                                     /*                 [ dx[1] | dy[1] | dz[1] ]     */
                                     /*                 [ dx[2] | dy[2] | dz[2] ]     */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a grid of points on a NURBS surface given a
     set of u- and v-values. A typical calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  *u, *v;
       NL_INDEX      nn, mm;
       NL_POINT      **S;
       ...
       (define sur; get u and v, allocate memory for S);
       ...
       N_SrfEvalPtGrid(&sur,u,v,nn,mm,NL_LEFT,NL_RIGHT,S);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Arrays of INCREASING parameters
     nn,mm     , input  ,  Highest indexes in u and v
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u or v)
     S       , output ,  Points  on the  surface.  S[i][j] is  a point 
                         computed at (u[i],v[j]). MEMORY FOR S MUST BE
                         ALLOCATED IN  THE CALLING  ROUTINE TO HOLD UP 
                         TO S[nn][mm].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtGrid( NL_SURFACE *sur, NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX nn, NL_INDEX mm, NL_FLAG ufl, NL_FLAG vfl, NL_POINT ** S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtGrid");
    NL_FLAG error = NL_NO;
    NL_INDEX *usp, *vsp, i, j, n, m, nu, mv, r, s;
    NL_DEGREE p, q;
    NL_REAL ** NU, ** NV, *U, *V;
    NL_KNOTVECTOR *knu, *knv;
    NL_CPOINT ** Pw, *Tw, Sw;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation */
    N_SrfGetCPtsDegreesAndKnots( sur, &nu, &mv, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Check parameters */
    if( nn LT 0 OR mm LT 0 )
        NL_ERROR( NL_IND_ERR );

    if( u[0]LT U[0]OR u[nn]GT U[r] )
        NL_ERROR( NL_PAR_ERR );

    if( v[0]LT V[0]OR v[mm]GT V[s] )
        NL_ERROR( NL_PAR_ERR );

    /* Compute non-vanishing B-splines */
    NU = N_AllocReal2dArray( nn, p, &SL );

    if( NU EQ NULL )
        NL_QUIT;

    NV = N_AllocReal2dArray( mm, q, &SL );

    if( NV EQ NULL )
        NL_QUIT;

    usp = N_AllocInt1dArray( nn, &SL );

    if( usp EQ NULL )
        NL_QUIT;

    vsp = N_AllocInt1dArray( mm, &SL );

    if( vsp EQ NULL )
        NL_QUIT;

    error = N_BasisEvalArray( knu, p, u, nn, ufl, NU, usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEvalArray( knv, q, v, mm, vfl, NV, vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute grid of points on the surface */

    Tw = N_AllocCPt1dArray( NL_MAX( nu, mv ), &SL );

    if( Tw EQ NULL )
        NL_QUIT;

    r = mm * (nu * q + nn * p);
    s = nn * (mv * p + mm * q);

    if( r LT s )
    {
        /* for every grid V point */
        for ( m = 0; m <= mm; m++ )
        {
            s = vsp[m] - q;

            /* build Tw[i] = Sum_j NV[m][j] * Pw[i][s+j] */
            for ( i = usp[0] - p; i <= usp[nn]; i++ )
            {
                N_CopyCPt( NL_CZERO, &Tw[i] );

                for ( j = 0; j <= q; j++ )
                    N_VectorBlendCPt( NV[m][j],     /* in : alpha of Bw = Bw + alpha*Aw */
                                      Pw[i][s + j], /* in : Aw    of Bw = Bw + alpha*Aw */
                                      &Tw[i] );     /* out: Bw    of Bw = Bw + alpha*Aw */
            }

            /* for every grid U point */
            for ( n = 0; n <= nn; n++ )
            {
                r = usp[n] - p;

                /* build S[n][m] = Sum_i NU[n][i] * Tw[r+i]                     */
                /*               = Sum_i Sum_j NU[n][i] * NV[m][j] * Pw[r+i][s+j] */
                N_CopyCPt( NL_CZERO, &Sw );

                for ( i = 0; i <= p; i++ )
                    N_VectorBlendCPt( NU[n][i],  /* in : alpha of Bw = Bw + alpha*Aw */
                                      Tw[r + i], /* in : Aw    of Bw = Bw + alpha*Aw */
                                      &Sw );     /* out: Bw    of Bw = Bw + alpha*Aw */
                N_CPtToPtEuclid( Sw, &S[n][m] );
            }
        } /* end iter every V grid Point */
    }
    else
    {
        /* for every grid U point */
        for ( n = 0; n <= nn; n++ )
        {
            r = usp[n] - p;

            for ( j = vsp[0] - q; j <= vsp[mm]; j++ )
            {
                N_CopyCPt( NL_CZERO, &Tw[j] );

                for ( i = 0; i <= p; i++ )
                    N_VectorBlendCPt( NU[n][i],     /* in : alpha of Bw = Bw + alpha*Aw */
                                      Pw[r + i][j], /* in : Aw    of Bw = Bw + alpha*Aw */
                                      &Tw[j] );     /* out: Bw    of Bw = Bw + alpha*Aw */
            }

            for ( m = 0; m <= mm; m++ )
            {
                s = vsp[m] - q;

                N_CopyCPt( NL_CZERO, &Sw );

                for ( j = 0; j <= q; j++ )
                    N_VectorBlendCPt( NV[m][j],  /* in : alpha of Bw = Bw + alpha*Aw */
                                      Tw[s + j], /* in : Aw    of Bw = Bw + alpha*Aw */
                                      &Sw );     /* out: Bw    of Bw = Bw + alpha*Aw */
                N_CPtToPtEuclid( Sw, &S[n][m] );
            }
        } /* end iter every U Grid Point */
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfEvalPtGrid */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This surface  routine scales  weighted  control  points of a NURBS 
     surface,  i.e.  if   Pw[i][j] = (w[i][j]*x[i][j], w[i][j]*y[i][j],
     w[i][j]*z[i][j],w[i][j]) and the scaling factor is f, then the new 
     control points are  Qw[i][j]=(w[i][j]*f*x[i][j],w[i][j]*f*y[i][j],
     w[i][j]*f*z[i][j],w[i][j]*f). A typical calling example is:

       NL_REAL     f;
       NL_SURFACE  sur;
       ...
       (define sur and get f);
       ...
       N_SrfScaleWeights(&sur,f);

     THE  SCALING  IS  DONE  IN-PLACE, I.E.  THE  ORIGINAL  NL_SURFACE  IS 
     DESTROYED.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     f   , input  ,  Scaling factor


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfScaleWeights( NL_SURFACE *sur, NL_REAL f )
{

    NL_INDEX i, j, n, m;
    NL_CPOINT ** Pw;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation */
    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Get scaled control points */
    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_ScaleCPt( f, Pw[i][j], &Pw[i][j] );
        }
    }

    N_EndNurbs( &SL );
} /* end N_SrfScaleWeights */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes  (n+1)x(m+1)  approximately  evenly  spaced 
     points on a surface. Optionally, the corresponding parameter values 
     as well as the points may be returned. A typical calling example:

       NL_SURFACE    sur;
       NL_PARAMETER  u0, u1, v0, v1;
       NL_INDEX      n, m;
       NL_REAL       *u, *v, tol;
       NL_POINT      **P;
       ...
       (define sur, get u0,..., tol; get memory for P, u and v);
       ...
       N_SrfEvenSpacedPts(&sur,u0,u1,v0,v1,n,m,tol,P   ,u   ,v   ); or
       N_SrfEvenSpacedPts(&sur,u0,u1,v0,v1,n,m,tol,P   ,NULL,NULL); or
       N_SrfEvenSpacedPts(&sur,u0,u1,v0,v1,n,m,tol,NULL,u   ,v   ); 


   ACCESS:
   
     sur   , input  ,  NURBS surface
     u0,u1 , input  ,  The  n+1 points will be taken at parameter values
                       between u0 and u1
     v0,v1 , input  ,  The  m+1 points will be taken at parameter values
                       between v0 and v1
     n,m   , input  ,  (n+1)x(m+1) points are  to be generated (n,m > 1)
     tol   , input  ,  Tolerance used to place points equally along iso-
                       parametric  lines (see N_CrvEvalEvenSpacedPts). A  good default
                       is  1% of the  diagonal of the  surface's min-max
                       box.
     P     , output ,  Point array:
                         != NULL: (n+1)x(m+1) points generated
                          = NULL: no points are computed
                       MEMORY FOR  P  MUST BE  ALLOCATED IN  THE CALLING
                       ROUTINE TO HOLD UP TO P[n][m]!
     u,v   , output ,  Parameter arrays:
                         != NULL: (n+1)/(m+1) parameters are computed
                          = NULL: no parameters are computed
                       MEMORY  FOR  u  AND  v  MUST BE  ALLOCATED IN THE 
                       CALLING ROUTINE TO HOLD UP TO u[n] AND v[m]!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvenSpacedPts( NL_SURFACE *sur, NL_PARAMETER u0, NL_PARAMETER u1, NL_PARAMETER v0, NL_PARAMETER v1, NL_INDEX n, NL_INDEX m, NL_REAL tol, NL_POINT ** P, NL_PARAMETER *u, NL_PARAMETER *v )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfEvenSpacedPts");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, ns, ms, rs, ss, nc, mc;
    NL_DEGREE ps, qs, pc;
    NL_REAL *t, *ua, *va, *ue, *ve, du, dv, uv;
    NL_KNOTVECTOR *knu, *knv;
    NL_CURVE cur;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation */
    N_SrfGetArraySizes( sur, &ns, &ms, &rs, &ss );
    N_SrfGetKnotVectors( sur, &knu, &knv );
    N_SrfGetDegrees( sur, &ps, &qs );

    /* Check input */
    error = N_KnotVectorIsParamOutOfBounds( knu, u0, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knu, u1, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v0, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v1, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( n LE 1 OR m LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( u NEQ NULL AND v EQ NULL )
        NL_ERROR( NL_INP_ERR );

    if( v NEQ NULL AND u EQ NULL )
        NL_ERROR( NL_INP_ERR );

    /* Get local memory */
    t = N_AllocReal1dArray( NL_MAX( n, m ), &S );

    if( t EQ NULL )
        NL_QUIT;

    ua = N_AllocReal1dArray( n, &S );

    if( ua EQ NULL )
        NL_QUIT;

    va = N_AllocReal1dArray( m, &S );

    if( va EQ NULL )
        NL_QUIT;

    ue = N_AllocReal1dArray( n, &S );

    if( ue EQ NULL )
        NL_QUIT;

    ve = N_AllocReal1dArray( m, &S );

    if( ve EQ NULL )
        NL_QUIT;

    nc = NL_MAX( ns, ms );
    pc = NL_MAX( ps, qs );
    mc = NL_MAX( rs, ss );

    error = N_AllocCrvArrays( &cur, nc, pc, mc, &S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get average v parameters using uniform u's */
    N_CrvSetSizeIndices( &cur, nc, pc, mc );

    du = (u1 - u0) / n;

    for ( j = 0; j <= m; j++ )
        va[j] = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        uv = u0 + i * du;

        if( uv LT u0 )
            uv = u0;

        if( uv GT u1 )
            uv = u1;

        error = N_SrfExtractIsoCrv( sur, uv, NL_VDIR, &cur, &S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEvalEvenSpacedPts( &cur, v0, v1, m, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 1; j < m; j++ )
            va[j] += t[j];
    }

    va[0] = v0;
    va[m] = v1;

    for ( j = 1; j < m; j++ )
        va[j] /= ((NL_REAL)n + 1.0);

    /* Get average u parameters using uniform v's */
    N_CrvSetSizeIndices( &cur, nc, pc, mc );

    dv = (v1 - v0) / m;

    for ( i = 0; i <= n; i++ )
        ua[i] = 0.0;

    for ( j = 0; j <= m; j++ )
    {
        uv = v0 + j * dv;

        if( uv LT v0 )
            uv = v0;

        if( uv GT v1 )
            uv = v1;

        error = N_SrfExtractIsoCrv( sur, uv, NL_UDIR, &cur, &S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEvalEvenSpacedPts( &cur, u0, u1, n, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 1; i < n; i++ )
            ua[i] += t[i];
    }

    ua[0] = u0;
    ua[n] = u1;

    for ( i = 1; i < n; i++ )
        ua[i] /= ((NL_REAL)m + 1.0);

    /* Get equally spaced points in v-direction using average u's */
    N_CrvSetSizeIndices( &cur, nc, pc, mc );

    for ( j = 0; j <= m; j++ )
        ve[j] = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        error = N_SrfExtractIsoCrv( sur, ua[i], NL_VDIR, &cur, &S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEvalEvenSpacedPts( &cur, v0, v1, m, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 1; j < m; j++ )
            ve[j] += t[j];
    }

    ve[0] = v0;
    ve[m] = v1;

    for ( j = 1; j < m; j++ )
        ve[j] /= ((NL_REAL)n + 1.0);

    /* Get equally spaced points in u-direction using average v's */
    N_CrvSetSizeIndices( &cur, nc, pc, mc );

    for ( i = 0; i <= n; i++ )
        ue[i] = 0.0;

    for ( j = 0; j <= m; j++ )
    {
        error = N_SrfExtractIsoCrv( sur, va[j], NL_UDIR, &cur, &S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEvalEvenSpacedPts( &cur, u0, u1, n, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 1; i < n; i++ )
            ue[i] += t[i];
    }

    ue[0] = u0;
    ue[n] = u1;

    for ( i = 1; i < n; i++ )
        ue[i] /= ((NL_REAL)m + 1.0);

    /* Output parameters and/or points */
    if( u NEQ NULL OR v NEQ NULL )
    {
        for ( i = 0; i <= n; i++ )
            u[i] = ue[i];

        for ( j = 0; j <= m; j++ )
            v[j] = ve[j];
    }

    if( P NEQ NULL )
    {
        error = N_SrfEvalPtGrid( sur, ue, ve, n, m, NL_LEFT, NL_LEFT, P );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */
    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfEvenSpacedPts */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This surface routine extends a Nurbs surface a given (approximate)
     distance.  The start or end in either the u- or v-direction can be 
     extended.  Continuity can be controlled via options.  After exten-
     sion,  the original parameter domain still  maps  to  the original 
     surface  (i.e. the parameter domain is also extended).  A  typical 
     calling example is:

       NL_SURFACE   surP, surQ;
       NL_REAL      dist;
       NL_STACKS    SP, SQ;
       ...
       (define surP and choose dist);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfExtendByDist(&surP,dist,NL_UDIR,NL_START,NL_CMAX,&surQ,&SP,&SQ);
       N_SrfExtendByDist(&surP,dist,NL_VDIR,NL_END,NL_G1,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in surQ's  
     knot vector and polygon objects.

     Routine extended to limit recursionto 10 levels.
   ACCESS:
   
     surP , input  ,  NURBS surface
     dist , input  ,  The length of the extension is approximately dist
                      all along the extended boundary
     dflg , input  ,  Flag:
                       NL_UDIR : The surface is extended across either the  
                              u=umin or u=umax boundary
                       NL_VDIR : The surface is extended across either the  
                              v=vmin or v=vmax boundary
     end  , input  ,  Flag:
                       NL_START: The surface is extended across either the 
                              u=umin or v=vmin boundary (see dflg)
                       NL_END  : The surface is extended across either the 
                              u=umax or v=vmax boundary (see dflg)
     cont , input  ,  Flag:
                       NL_G1  : surP is  extended with tangent plane cont-
                             inuity.  surQ  is  NL_G1 continuous where the  
                             extension joins the original surface.
                             Use method by Shetty & White, CAD, Sept. 1991.
                       NL_G1R : surP is extended with a ruled surface that
                             connects with NL_G1-continuity
                       NL_G2  : surP is extended by  reflection,  yielding 
                             a NL_G2 (curvature) continuous extension.
                             Use method by Shetty & White, CAD, Sept. 1991.
                       NL_CMAX: the extension yields infinite C-continuity
                             along the boundary (no knot there).
     surQ , output ,  Surface after extension
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_SrfExtendByDist
 (NL_SURFACE *surP,  /* in : tgt surface to extend */
  NL_REAL     dist,  /* in : requested extension distance of the specified boundary - approximately enforced */
  NL_FLAG     dflg,  /* in : NL_UDIR  = extend U dir by either decreasing uMin or increasing uMax value */
                     /*      NL_VDir  = extend V dir by either decreasing vMin or increasing vMax value */
  NL_FLAG     end,   /* in : NL_START = decrease Min knot value */
                     /*      NL_END   = increase Max knot value */
  NL_FLAG     cont,  /* in : NL_G1    = surP is extended with tangent plane continuity. */
                     /*                 surQ is NL_G1 continuous where the */ 
                     /*                 extension joins the original surface */
                     /*      NL_G1R   = surP is extended with a ruled surface that */
                     /*                 connects with NL_G1-continuity. */
                     /*      NL_G2    = surP is extended by reflection, yielding */
                     /*                 a NL_G2 (curvature) continuous extension. */
                     /*      NL_CMAX  = the extension yields infinite C-continuity */
                     /*                 along the boundary (no knot there) */
  NL_SURFACE *surQ,  /* out: the extended surface */
  NL_STACKS  *SP,    /* out: stack containing surP */
  NL_STACKS  *SQ )   /* out: stack to contain all surQ newly constructed objects */
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfExtendByDist"); */

    /* pass the call along to the resursive function */
    return (ST_surxsdr( surP, dist, dflg, end, cont, surQ, SP, SQ, 0 ));

} /* end N_SrfExtendByDist */


/*******************************************************************//**


   DESCRIPTION:

     Recursive base method to N_SrfExtendByDist() interface method.
   ***********************************************************************/
NL_FLAG ST_surxsdr
 (NL_SURFACE *surP,  /* in : tgt surface to extend */
  NL_REAL     dist,  /* in : requested extension distance of the specified boundary - approximately enforced */
  NL_FLAG     dflg,  /* in : NL_UDIR  = extend U dir by either decreasing uMin or increasing uMax value */
                     /*      NL_VDir  = extend V dir by either decreasing vMin or increasing vMax value */
  NL_FLAG     end,   /* in : NL_START = decrease Min knot value */
                     /*      NL_END   = increase Max knot value */
  NL_FLAG     cont,  /* in : NL_G1    = surP is extended with tangent plane continuity. */
                     /*                 surQ is NL_G1 continuous where the */ 
                     /*                 extension joins the original surface */
                     /*      NL_G1R   = surP is extended with a ruled surface that */
                     /*                 connects with NL_G1-continuity. */
                     /*      NL_G2    = surP is extended by reflection, yielding */
                     /*                 a NL_G2 (curvature) continuous extension. */
                     /*      NL_CMAX  = the extension yields infinite C-continuity */
                     /*                 along the boundary (no knot there) */
  NL_SURFACE *surQ,  /* out: the extended surface */
  NL_STACKS  *SP,    /* out: stack containing surP */
  NL_STACKS  *SQ,    /* out: stack to contain all surQ newly constructed objects */
  int         depth) /* in : recursion depth */
{
    NL_PRIVATE NL_STRING rname = _T("ST_surxsdr");
    NL_FLAG error = NL_NO, lcont;
    NL_CURVE curA, curB;
    NL_SURFACE surA, surB;

    NL_INDEX ii, jj, kk, n, m, r, s, nq, mq, rq, sq, span = 0, mult = 0, nb, mb, rb, sb, kby, kkt, nn, ktal = 0, rem, kp;
    NL_INDEX iNumSamples;

    NL_DEGREE p, q, pq;
    NL_CPOINT ** Pw, ** Qw, ** Bw, ** Rw = NULL, *Dw;
    NL_POINT P1, P2, P3;
    NL_VECTOR V;
    NL_PLANE pln;
    NL_MATRIXTYPE mtp;
    NL_RMATRIX rma;
    NL_KNOTVECTOR *knu, *knv;
    NL_BOOLEAN rat;
    NL_REAL *UP, *VP, *UQ, *VQ, uv[5], uuu = 0.0, vvv = 0.0, d1, du, dv, d2 = 0.0, d3, d4 = 0.0, *UB, *VB, tal, w0, w1, ** RM, rhs[4], sol[2], hh[5][4], dr, dl, w;
    NL_REAL dExtPrmU = 0.0, dExtPrmV = 0.0, dReflPrmU = 0.0, dReflPrmV = 0.0, dEndPrmU = 0.0, dEndPrmV = 0.0;
    NL_STACKS SL;

static int siDebug=0;
static int cbi=0;

    /* Limit recursion to 10 levels */
    if( depth > 10 )
        return (1);

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation */
    N_SrfGetCPtsDegreesAndKnots( surP,   /* in : tgt surface */
                                 &n,     /* out: max 1st index in Pw - number of U control points = n + 1 */
                                 &m,     /* out: max 2nd index in Pw - number of V control points = m + 1 */
                                 &Pw,    /* out: Pw sized:[n+1][m+1]  */
                                 &p,     /* out: u degree */
                                 &q,     /* out: v degree */
                                 &r,     /* out: max knot index in U array - number of u knots = r + 1 */
                                 &s,     /* out: max knot index in V array - number of v knots = s + 1 */
                                 &UP,    /* out: U knot vector sized:[r+1] */
                                 &VP );  /* out: V knot vector sized:[s+1] */

    /* Determine u or v value that corresponds roughly to dist */
    /* Do this by averaging results for curve extensions       */
    N_SrfGetKnotVectors( surP, &knu, &knv );

    iNumSamples = 5;  /* this many curves will be used to compute the average */

    uv[0] = 0.07; /* must change array declaration if change iNumSamples */
    uv[1] = 0.19;
    uv[2] = 0.45;
    uv[3] = 0.73;
    uv[4] = 0.92;

    if( cont EQ NL_CMAX )     { lcont = NL_CMAX; }
    else if( cont EQ NL_G1R ) { lcont = NL_G1; }
    else                      { lcont = NL_G2; }
    /*  Note: why does NL_G1 map to NL_G2? Because the choice drives different algorithms */
    /*        lcont = NL_G2: extend surface by reflection, this works for both NL_G1 and NL_G2 input requests */
    /*        lcont = NL_G1: extend surface with a ruled surface, this works for NL_G1R input requests */
    /*        lcont = NL_CMAX: extend surface leaving no knot at original boundary (infinite C-continuity */

    /* when dflg == NL_UDIR branch */
    if( dflg EQ NL_UDIR )
    {
        N_CrvInitArrays( &curA );
        error = N_CrvSizeArrays( &curA, n, p, r, rname, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get average extension parameter of iNumSamples surface iso-curve extensions to our extension dist. */
        kk = 0;
        dExtPrmU = 0.0;

        /* iter every sample point making and extending an isocurve to get an average target extension parameter value */
        for ( ii = 0; ii < iNumSamples; ii++ )
        {
            /* pick a varying sample V value on the line between the first and last Knot V values */
            vvv = (1.0 - uv[ii]) * VP[0] + uv[ii] * VP[s];

            /* extract const V=vvv (u param varying) isoCurve from surf surP */ 
            error = N_SrfExtractIsoCrv( surP, vvv, NL_UDIR, &curA, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* extend the isoCurve by the requested dist */
            N_CrvInitArrays( &curB );
            error = N_CrvExtendByDist( &curA, dist, NL_ABSOLUTE, end, lcont, &curB, &SL, &SL );

            /* debug */
            if ( siDebug>0 ) 
              {
                N_CrvPrint( &curA );
                N_CrvPrint( &curB );
              }

            /* accumulate the extended curve natural interval bound values - for the upcoming average */
            if( error EQ NL_NO )
            {
                kk += 1;
                N_CrvGetParamBounds(&curB,  /* in : tgt curve */
                                    &d1,    /* out: cur's NaturalInterval min value */
                                    &d2 );  /* out: cur's NaturalInterval max value */

                /* accumulate the appropriate NaturalInterval bound value */
                if( end EQ NL_START ) { dExtPrmU += d1; }
                else                  { dExtPrmU += d2; }
                N_FreeCrv( &curB, &SL ); /* sometimes crashes here when dist is big */
            }
        } /* end iter every sample point making and extending an isocurve to get an average target extension parameter value */

        if( kk EQ 0 )
          { NL_ERROR( NL_NUM_ERR ); }

        /* let target extension param = Avg(extension found for sampled extended isocurves to meet the dist input request) */
        dExtPrmU /= kk;

        /* Check if need to recurse because distance too big */
        if( cont EQ NL_G1 OR cont EQ NL_G2 )
        {
            /* when param extension is as big or bigger than the current Natural interval - break the extension up into steps */
            if(    ( end EQ NL_START AND UP[0] - dExtPrmU GT 0.9313 *( UP[r] - UP[0] ))
                OR ( end EQ NL_END   AND dExtPrmU - UP[r] GT 0.9313 *( UP[r] - UP[0] )) )
            { /* must recurse */
                if( end EQ NL_START ) { d2 = (dist * (UP[r] - UP[0])) / (2.0 *( UP[0] - dExtPrmU )); }
                else                  { d2 = (dist * (UP[r] - UP[0])) / (2.0 *( dExtPrmU - UP[r] )); }

                /* 1st Recursive call to Extend first half of dist into surQ: (a very large extension can be broken up by many recursive steps) */
                error = ST_surxsdr( surP, dist - d2, dflg, end, cont, surQ, SP, SQ, depth + 1 );

                if( error EQ NL_YES )
                  { NL_OUT; }

                /* 2nd Recursive call to Extend surQ by second half of dist: */
                error = ST_surxsdr( surQ, d2, dflg, end, cont, surQ, SQ, SQ, depth + 1 );
                NL_OUT;
            }
        } /* end need to recurse because param extension is too big check */

        /* Set reflection parameter from extension parameter. */
        if( end EQ NL_START ) { dReflPrmU = 2.0 * UP[0] - dExtPrmU; } /* = UP[0] + (UP[0] - dExtPrmU) */ 
        else                  { dReflPrmU = 2.0 * UP[r] - dExtPrmU; } /* = UP[r] - (dExtPrmU - UP[r]) */ 

        /* Find the closest knot to dReflPrm and snap to it if close enough. */
        d1 = NL_BIGD;

        jj = p + 1;  /* Just to silence a compiler warning... */

        /* iter all knot values */
        for ( ii = p + 1; ii < r - p; ii++ )
        {
            /* when current knot is closer to the ReflPrmU value */
            if( fabs( dReflPrmU - UP[ii] )LT d1 )
            {
                /* save the smallest dist to an existing knot value and the knot's index */
                d1 = fabs( dReflPrmU - UP[ii] );
                jj = ii;
            }
        } /* end iter all knots looking for the knot value closest to the RelPrmU value */

        /* when nearest existing knot is close enough - snap to current knot value */
        if( d1 LT KN_TOL * (UP[r] - UP[0]) )  
          { dReflPrmU = UP[jj]; }

    } /* end branch dflg == NL_UDIR */

    else  /* when dflg == NL_VDIR branch */
    {
        N_CrvInitArrays( &curA );
        error = N_CrvSizeArrays( &curA, m, q, s, rname, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get average extension parameter of iNumSamples surface iso-curve extensions to our extension dist. */
        kk = 0;
        dExtPrmV = 0.0;

        /* iter every sample point making and extending an isocurve to get an average target extension parameter value */
        for ( ii = 0; ii < iNumSamples; ii++ )
        {
            /* pick a varying sample U value on the line between the first and last Knot U values */
            uuu = (1.0 - uv[ii]) * UP[0] + uv[ii] * UP[r];

            /* extract const U=uuu (v param varying) isoCurve from surf surP */ 
            error = N_SrfExtractIsoCrv( surP, uuu, NL_VDIR, &curA, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* extend the isoCurve by the requested dist */
            N_CrvInitArrays( &curB );
            error = N_CrvExtendByDist( &curA, dist, NL_ABSOLUTE, end, lcont, &curB, &SL, &SL );

            /* accumulate the extended curve natural interval bound values - for the upcoming average */
            if( error EQ NL_NO )
            {
                kk += 1;
                N_CrvGetParamBounds( &curB, &d1, &d2 );

                if( end EQ NL_START ) { dExtPrmV += d1; }
                else                  { dExtPrmV += d2; }
                N_FreeCrv( &curB, &SL );
            }
        } /* end iter every sample point making and extending an isocurve to get an average target extension parameter value */

        if( kk EQ 0 )
          { NL_ERROR( NL_NUM_ERR ); }

        /* let target extension param = Avg(extension found for sampled extended isocurves to meet the dist input request) */
        dExtPrmV /= kk;

        /* Check if need to recurse because distance too big */
        if( cont EQ NL_G1 OR cont EQ NL_G2 )
        {
            /* when param extension is as big or bigger than the current Natural interval - break the extension up into steps */
            if(   ( end EQ NL_START AND VP[0] - dExtPrmV GT 0.9313 *( VP[s] - VP[0] ))
                OR( end EQ NL_END   AND dExtPrmV - VP[s] GT 0.9313 *( VP[s] - VP[0] )) )
            { /* must recurse */
                if( end EQ NL_START ) { d2 = (dist * (VP[s] - VP[0])) / (2.0 *( VP[0] - dExtPrmV )); }
                else                  { d2 = (dist * (VP[s] - VP[0])) / (2.0 *( dExtPrmV - VP[s] )); }

                /* 1st Recursive call to Extend first half of dist into surQ: (a very large extension can be broken up by many recursive steps) */
                error = ST_surxsdr( surP, dist - d2, dflg, end, cont, surQ, SP, SQ, depth + 1 );

                if( error EQ NL_YES )
                  { NL_OUT; }

                /* 2nd Recursive call to Extend surQ by second half of dist: */
                error = ST_surxsdr( surQ, d2, dflg, end, cont, surQ, SQ, SQ, depth + 1 );
                NL_OUT;
            }
        } /* end need to recurse because param extension is too big check */

        /* Set reflection parameter from extension parameter. */
        if( end EQ NL_START ) { dReflPrmV = 2.0 *VP[0] - dExtPrmV; } /* = VP[0] + (VP[0] - dExtPrmU) */ 
        else                  { dReflPrmV = 2.0 *VP[s] - dExtPrmV; } /* = VP[s] - (dExtPrmU - VP[s]) */ 

        /* Find the closest knot to dReflPrm and snap to it if close enough. */
        d1 = NL_BIGD;

        jj = q + 1;  /* Just to silence a compiler warning... */

        /* iter all knot values */
        for ( ii = q + 1; ii < s - q; ii++ )
        {
            /* when current knot is closer to the ReflPrmV value */
            if( fabs( dReflPrmV - VP[ii] )LT d1 )
            {
                /* save the smallest dist to an existing knot value and the knot's index */
                d1 = fabs( dReflPrmV - VP[ii] );
                jj = ii;
            }
        } /* end iter all knots looking for the knot value closest to the RelPrmV value */

        /* when nearest existing knot is close enough - snap to current knot value */
        if( d1 LT KN_TOL * (VP[s] - VP[0]) )  
          { dReflPrmV = VP[jj]; }

    } /* end branch dflg == NL_VDIR */

/* --------------------------------
 *   cbi Note: done with 'dist' here already.
 * --------------------------------
 */

if ( cbi>0 ) {
  double dExtendParam = ( dflg == NL_UDIR ) ? dExtPrmU : dExtPrmV;

  /* pass the call along */
  return N_SrfExtendByParamDist( surP,         /* in : tgt surface to extend */
                                 dExtendParam, /* in : desired new u or v parameter value */
                                 dflg,         /* in : NL_UDIR  = extend U dir by either decreasing uMin or increasing uMax value */
                                               /*      NL_VDir  = extend V dir by either decreasing vMin or increasing vMax value */
                                 end,          /* in : NL_START = decrease Min knot value */
                                               /*      NL_END   = increase Max knot value */
                                 cont,         /* in : NL_G1    = surP is extended with tangent plane continuity. */
                                               /*                 surQ is NL_G1 continuous where the */ 
                                               /*                 extension joins the original surface */
                                               /*      NL_G1R   = surP is extended with a ruled surface that */
                                               /*                 connects with NL_G1-continuity. */
                                               /*      NL_G2    = surP is extended by reflection, yielding */
                                               /*                 a NL_G2 (curvature) continuous extension. */
                                               /*      NL_CMAX  = the extension yields infinite C-continuity */
                                               /*                 along the boundary (no knot there) */
                                 surQ,         /* out: the extended surface */
                                 SP,           /* out: stack containing surP */
                                 SQ );         /* out: stack to contain all surQ newly constructed objects */
}

    /* Determine nq = max index of U knots to be in output surQ surface,      */
    /*           mq = max index of V knots to be in output surQ surface,      */
    /*           rq = max index of U ControlPts to be in output surQ surface, */
    /*           sq = max index of V ControlPts to be in output surQ surface, */
    /* and allocate working memory */

    /* when dFlg == NL_UDIR branch */
    if( dflg EQ NL_UDIR )
    {
        /* when end EQ NL_START branch */
        if( end EQ NL_START )
        {
            /* when cont EQ NL_CMAX branch - for boundary interval extension */
            if( cont EQ NL_CMAX )
            {
                mult = 1;

                while( p + mult + 1 LE r )
                {
                    if( UP[p + mult + 1]EQ UP[p + 1] )
                      { mult += 1; }
                    else
                      { break; }
                }

                if( mult GT p )
                    mult = p;
                nq = n + 2 * p - mult;
                kk = 2 * p;
            } /* end cont == NL_CMAX branch */

            /* when cont == NL_G1R branch - for ruled surface extension */
            else if( cont EQ NL_G1R )
              { nq = n + p; }

            else /* when cont == NL_G1 or NL_G2 branch (for reflective surface extension) */
            {
                error = N_BasisFindSpanAndMult(knu,       /* in : knot vector                              */
                                               p,         /* in : curve degree                             */
                                               dReflPrmU, /* in : parameter value                          */
                                               NL_RIGHT,  /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                          /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,     /* out: index of span containing u               */
                                               &mult );   /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */
                if( error EQ NL_YES )
                  { NL_OUT; }
                nq = n + span + p - mult;
                kk = 2 * span;
            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end end EQ NL_START branch */

        else /* when (end EQ NL_END) branch */
        {
            /* when cont EQ NL_CMAX branch - for boundary interval extension */
            if( cont EQ NL_CMAX )
            {
                mult = 1;

                while( r - p - mult - 1 GE 0 )
                {
                    if( UP[r - p - mult - 1]EQ UP[r - p - 1] )
                      { mult += 1; }
                    else
                      { break; }
                }

                if( mult GT p )
                    mult = p;
                nq = n + 2 * p - mult;
                kk = 2 * p;
            } /* end cont == NL_CMAX branch */

            /* when cont == NL_G1R branch - for ruled surface extension */
            else if( cont EQ NL_G1R )
              { nq = n + p; }

            else /* when cont == NL_G1 or NL_G2 branch (for reflective surface extension) */
            {
                error = N_BasisFindSpanAndMult(knu,       /* in : knot vector                              */
                                               p,         /* in : curve degree                             */
                                               dReflPrmU, /* in : parameter value                          */
                                               NL_LEFT,   /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                          /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,     /* out: index of span containing u               */
                                               &mult );   /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */
                if( error EQ NL_YES )
                  { NL_OUT; }
                nq = n + r - span - 1 + p - mult;
                kk = 2 * (r - span - 1);
            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end == NL_END) branch */

        mq = m;
        sq = s;
        rq = nq + p + 1;

        /* allocate new memory for ruled surface and reflection extensions - not needed for last interval extension */
        if( cont NEQ NL_G1R )
        {
            Rw = N_AllocCPt2dArray( mq, kk, &SL );

            if( Rw EQ NULL )
              { NL_QUIT; }
        }
    } /* end dflg == NL_UDIR branch */

    else  /* when dFlg == NL_VDIR branch */
    {
        /* when end == NL_START branch */
        if( end EQ NL_START )
        {
            /* when cont == NL_CMAX branch - for boundary interval extension */
            if( cont EQ NL_CMAX )
            {
                mult = 1;

                while( q + mult + 1 LE s )
                {
                    if( VP[q + mult + 1]EQ VP[q + 1] )
                      { mult += 1; }
                    else
                      { break; }
                }

                if( mult GT q )
                  { mult = q; }
                mq = m + 2 * q - mult;
                kk = 2 * q;
            } /* end cont == NL_CMAX branch */

            /* when cont == NL_G1R branch - for ruled surface extension */
            else if( cont EQ NL_G1R )
              { mq = m + q; }

            else /* when cont == NL_G1 or NL_G2 branch (for reflective surface extension) */
            {
                error = N_BasisFindSpanAndMult(knv,       /* in : knot vector                              */
                                               q,         /* in : curve degree                             */
                                               dReflPrmV, /* in : parameter value                          */
                                               NL_RIGHT,  /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                          /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,     /* out: index of span containing u               */
                                               &mult );   /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */
                if( error EQ NL_YES )
                  { NL_OUT; }
                mq = m + span + q - mult;
                kk = 2 * span;
            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end == NL_START) branch */

        else /* when (end EQ NL_END) branch */
        {
            /* when cont EQ NL_CMAX branch - for boundary interval extension */
            if( cont EQ NL_CMAX )
            {
                mult = 1;

                while( s - q - mult - 1 GE 0 )
                {
                    if( VP[s - q - mult - 1]EQ VP[s - q - 1] )
                      { mult += 1; }
                    else
                      { break; }
                }

                if( mult GT q )
                  { mult = q; }
                mq = m + 2 * q - mult;
                kk = 2 * q;
            } /* end cont == NL_CMAX branch */

            /* when cont == NL_G1R branch - for ruled surface extension */
            else if( cont EQ NL_G1R )
              { mq = m + q; }

            else /* when cont == NL_G1 or NL_G2) branch for reflective surface extension) */
            {
                error = N_BasisFindSpanAndMult(knv,       /* in : knot vector                              */
                                               q,         /* in : curve degree                             */
                                               dReflPrmV, /* in : parameter value                          */
                                               NL_LEFT,   /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                          /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,     /* out: index of span containing u               */
                                               &mult );   /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */

                if( error EQ NL_YES )
                  { NL_OUT; }
                mq = m + s - span - 1 + q - mult;
                kk = 2 * (s - span - 1);
            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end EQ NL_END) branch */

        nq = n;
        rq = r;
        sq = mq + q + 1;

        /* allocate new memory for ruled surface and reflection extensions - not needed for last interval extension */
        if( cont NEQ NL_G1R )
        {
            Rw = N_AllocCPt2dArray( nq, kk, &SL );

            if( Rw EQ NULL )
              { NL_QUIT; }
        }
    }  /* end dflg == NL_VDIR branch */

    /* arrive here when */
    /*   nq = max index of U knots to be in output surQ surface,      */
    /*   mq = max index of V knots to be in output surQ surface,      */
    /*   rq = max index of U ControlPts to be in output surQ surface, */
    /*   sq = max index of V ControlPts to be in output surQ surface, */

    /* See if memory is needed for surQ */
    surA = *surP;

    /* when doing a ruled surface or reflection extension */
    if( cont NEQ NL_G1R )
    {
        /* copy surP */
        N_SrfInitArrays( &surB );
        error = N_SrfCopy( surP, &surB, &SL );

        if( error EQ NL_YES )
          { NL_OUT; }
    }

    /* when asked to extend in place */
    if( surP EQ surQ )
    {
        error = N_AllocSrfArrays(surP,  /* in : target surface */
                                 nq,    /* in : highest U control point index */
                                 mq,    /* in : highest V control point index */
                                 p,     /* in : u degree */
                                 q,     /* in : v degree */
                                 rq,    /* in : highest U knot index */
                                 sq,    /* in : highest V knot index */
                                 SP );  /* in : sur's stack */

        if( error EQ NL_YES )
          { NL_OUT; }

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else /* do the extension in surQ struct */
    {
        error = N_SrfSizeArrays(surQ,  /* in : target sur */
                                nq,    /* in : highest U control point index */
                                mq,    /* in : highest V control point index */
                                p,     /* in : u degree */
                                q,     /* in : v degree */
                                rq,    /* in : highest U knot index */
                                sq,    /* in : highest V knot index */
                                rname, /* in : Routine making request's name */
                                SQ );  /* in : sur's stack */

        if( error EQ NL_YES )
          { NL_OUT; }

        N_SrfGetCPtsAndKnots(surQ,   /* in : target surface       */
                             &Qw,    /* out: Control Point Arrays */
                             &UQ,    /* out: U Knots */
                             &VQ );  /* out: V knots */
    }

    /* Do the RuledSurface extension for NL_G1R case and exit */
    if( cont EQ NL_G1R )
    {
        if( dflg EQ NL_UDIR )
        {
            /* load v-knot vector */

            for ( ii = 0; ii <= s; ii++ )
              { VQ[ii] = VP[ii]; }

            /* rest depends on start/end case */

            d2 = (NL_REAL)p;

            if( end EQ NL_START )
            {
                /* load u-knot vector */

                /* Recompute Extension param in case we snapped. */
                dExtPrmU = 2.0 * UP[0] - dReflPrmU;

                for ( ii = 0; ii <= p; ii++ )
                  { UQ[ii] = dExtPrmU; }

                for ( ii = 1; ii <= r; ii++ )
                  { UQ[ii + p] = UP[ii]; }

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                {
                  for ( jj = 0; jj <= m; jj++ )
                      { N_CopyCPt( Pw[ii][jj], &Qw[ii + p][jj] ); }
                }

                /* compute new Qw[0][jj], i.e. boundary control points */

                du = -d2 * ((UP[0] - dExtPrmU) / (UP[p + 1] - UP[0]));

                /* Save the end knot, to be (partially) removed. */
                dEndPrmU = UP[0];

                for ( jj = 0; jj <= m; jj++ )
                {
                    N_CPtToPtEuclid( Pw[0][jj], &P1 );
                    N_CPtToPtEuclid( Pw[1][jj], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector(P1,    /* in : P     of  Q = P + alpha * V */
                                          du,    /* in : alpha of  Q = P + alpha * V */
                                          V,     /* in : V     of  Q = P + alpha * V */
                                          &P3 ); /* out: Q     of  Q = P + alpha * V */

                    N_CPtGetW( Pw[1][jj], &w );
                    N_Weight( P3,            /* in : euclidean point                     */
                               w,            /* in : weight or NL_NOW                    */
                              &Qw[0][jj] );  /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[1][jj],...,Qw[p-1][jj]  (degree elevation) */

                for ( jj = 0; jj <= m; jj++ )
                {
                    for ( ii = 1; ii < p; ii++ )
                    {
                        d1 = (NL_REAL)ii / d2;
                        N_Combine2CPts( 1.0 - d1,      /* alpha of Cw = alpha * Aw + beta * Bw */
                                        Qw[0][jj],     /* Aw    of Cw = alpha * Aw + beta * Bw */
                                        d1,            /* beta  of Cw = alpha * Aw + beta * Bw */
                                        Pw[0][jj],     /* Bw    of Cw = alpha * Aw + beta * Bw */
                                        &Qw[ii][jj] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
                }
            }
            else  /* end == NL_END */
            {
                /* load u-knot vector */

                /* Recompute Extension param in case we snapped. */
                dExtPrmU = 2.0 * UP[r] - dReflPrmU;

                for ( ii = 0; ii <= p; ii++ )
                  { UQ[ii + r] = dExtPrmU; }

                for ( ii = 0; ii < r; ii++ )
                  { UQ[ii] = UP[ii]; }

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                {
                    for ( jj = 0; jj <= m; jj++ )
                      { N_CopyCPt( Pw[ii][jj], &Qw[ii][jj] ); }
                }

                /* compute new Qw[nq][jj], i.e. boundary control points */

                du = d2 * ((dExtPrmU - UP[r]) / (UP[r] - UP[r - p - 1]));

                /* Save the end knot, to be (partially) removed. */
                dEndPrmU = UP[r];

                for ( jj = 0; jj <= m; jj++ )
                {
                    N_CPtToPtEuclid( Pw[n - 1][jj], &P1 );
                    N_CPtToPtEuclid( Pw[n][jj], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector(P2,     /* in : P     of  Q = P + alpha * V */
                                          du,     /* in : alpha of  Q = P + alpha * V */
                                          V,      /* in : V     of  Q = P + alpha * V */
                                          &P3 );  /* out: Q     of  Q = P + alpha * V */

                    N_CPtGetW( Pw[n - 1][jj], &w );
                    N_Weight( P3,           /* in : euclidean point                     */
                               w,           /* in : weight or NL_NOW                    */
                             &Qw[nq][jj] ); /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[n+1][jj],...,Qw[nq-1][jj]  (degree elevation) */

                for ( jj = 0; jj <= m; jj++ )
                {
                    for ( ii = 1; ii < p; ii++ )
                    {
                        d1 = (NL_REAL)ii / d2;
                        N_Combine2CPts( 1.0 - d1,         /* alpha of Cw = alpha * Aw + beta * Bw */
                                       Qw[n][jj],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                       d1,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                       Qw[nq][jj],        /* Bw    of Cw = alpha * Aw + beta * Bw */
                                       &Qw[n + ii][jj] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
                }
            }

            /* Try to remove one occurrence of dEndPrmU */

            error = N_SrfRemoveKnotConditional(surQ,     /* in : target surface */
                                               dEndPrmU, /* in : knot to be removed */
                                               1,        /* in : number of times to be removed */
                                               NL_MTOL,  /* in : tol to check removability */
                                               NL_UDIR,  /* in : NL_UDIR: Remove in u-direction */
                                                         /*      NL_VDIR: Remove in v-direction */
                                               &ii,      /* out: number of knots actually removed */
                                               surQ,     /* out: Surface after knot removal */
                                               SQ );     /* in : surQ's stack */
            if( error EQ NL_YES )
              { NL_OUT; }
        }
        else  /* dflg == NL_VDIR ) */
        {
            /* load u-knot vector */

            for ( ii = 0; ii <= r; ii++ )
              { UQ[ii] = UP[ii]; }

            /* rest depends on start/end case */

            d2 = (NL_REAL)q;

            if( end EQ NL_START )
            {
                /* load v-knot vector */

                dExtPrmV = 2.0 * VP[0] - dReflPrmV;

                for ( ii = 0; ii <= q; ii++ )
                  { VQ[ii] = dExtPrmV; }

                for ( ii = 1; ii <= s; ii++ )
                  { VQ[ii + q] = VP[ii]; }

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                {   for ( jj = 0; jj <= m; jj++ )
                      { N_CopyCPt( Pw[ii][jj], &Qw[ii][jj + q] ); }
                }

                /* compute new Qw[ii][0], i.e. boundary control points */

                dv = -d2 * ((VP[0] - dExtPrmV) / (VP[q + 1] - VP[0]));

                /* Save the end knot, to be (partially) removed. */
                dEndPrmV = VP[0];

                for ( ii = 0; ii <= n; ii++ )
                {
                    N_CPtToPtEuclid( Pw[ii][0], &P1 );
                    N_CPtToPtEuclid( Pw[ii][1], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector(P1,    /* in : P     of  Q = P + alpha * V */
                                          dv,    /* in : alpha of  Q = P + alpha * V */
                                          V,     /* in : V     of  Q = P + alpha * V */
                                          &P3 ); /* out: Q     of  Q = P + alpha * V */

                    N_CPtGetW( Pw[ii][1], &w );
                    N_Weight( P3,          /* in : euclidean point                     */
                               w,          /* in : weight or NL_NOW                    */
                             &Qw[ii][0] ); /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[ii][1],...,Qw[ii][q-1]  (degree elevation) */

                for ( ii = 0; ii <= n; ii++ )
                {
                    for ( jj = 1; jj < q; jj++ )
                    {
                        d1 = (NL_REAL)jj / d2;
                        N_Combine2CPts( 1.0 - d1,     /* alpha of Cw = alpha * Aw + beta * Bw */
                                       Qw[ii][0],     /* Aw    of Cw = alpha * Aw + beta * Bw */
                                       d1,            /* beta  of Cw = alpha * Aw + beta * Bw */
                                       Pw[ii][0],     /* Bw    of Cw = alpha * Aw + beta * Bw */
                                       &Qw[ii][jj] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
                }
            }
            else  /* end == NL_END */
            {
                /* load v-knot vector */

                dExtPrmV = 2.0 * VP[s] - dReflPrmV;

                for ( ii = 0; ii <= q; ii++ )
                  { VQ[ii + s] = dExtPrmV; }

                for ( ii = 0; ii < s; ii++ )
                  { VQ[ii] = VP[ii]; }

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                {
                    for ( jj = 0; jj <= m; jj++ )
                      { N_CopyCPt( Pw[ii][jj], &Qw[ii][jj] ); }
                }

                /* compute new Qw[ii][mq], i.e. boundary control points */

                dv = d2 * ((dExtPrmV - VP[s]) / (VP[s] - VP[s - q - 1]));

                /* Save the end knot, to be (partially) removed. */
                dEndPrmV = VP[s];

                for ( ii = 0; ii <= n; ii++ )
                {
                    N_CPtToPtEuclid( Pw[ii][m - 1], &P1 );
                    N_CPtToPtEuclid( Pw[ii][m], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector(P2,    /* in : P     of  Q = P + alpha * V */
                                          dv,    /* in : alpha of  Q = P + alpha * V */
                                          V,     /* in : V     of  Q = P + alpha * V */
                                          &P3 ); /* out: Q     of  Q = P + alpha * V */

                    N_CPtGetW( Pw[ii][m - 1], &w );
                    N_Weight( P3,           /* in : euclidean point                     */
                               w,           /* in : weight or NL_NOW                    */
                             &Qw[ii][mq] ); /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[ii][m+1],...,Qw[ii][mq-1]  (degree elevation) */

                for ( ii = 0; ii <= n; ii++ )
                {
                    for ( jj = 1; jj < q; jj++ )
                    {
                        d1 = (NL_REAL)jj / d2;
                        N_Combine2CPts( 1.0 - d1,         /* alpha of Cw = alpha * Aw + beta * Bw */
                                       Qw[ii][m],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                       d1,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                       Qw[ii][mq],        /* Bw    of Cw = alpha * Aw + beta * Bw */
                                       &Qw[ii][m + jj] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
                }
            }

            /* Try to remove one occurrence of dEndPrmV */

            error = N_SrfRemoveKnotConditional(surQ,     /* in : target surface */
                                               dEndPrmV, /* in : knot to be removed */
                                               1,        /* in : number of times to be removed */
                                               NL_MTOL,  /* in : tol to check removability */
                                               NL_VDIR,  /* in : NL_UDIR: Remove in u-direction */
                                                         /*      NL_VDIR: Remove in v-direction */
                                               &ii,      /* out: number of knots actually removed */
                                               surQ,     /* out: Surface after knot removal */
                                               SQ );     /* in : surQ's stack */
            if( error EQ NL_YES )
              { NL_OUT; }

        }  /* end  ( else  dflg == NL_VDIR ) */

        /* If extension is in place, kill old surface */

        if( surP EQ surQ )
          { N_FreeSrf( &surA, SP ); }

        NL_OUT; /* exit */

    } /* end special case for cont == NL_G1R */

    /* arrive here for reflection or last interval extension cases (cont == NL_G1, NL_G2, or NL_CMAX) */
    /*   nq = max index of U knots to be in output surQ surface,      */
    /*   mq = max index of V knots to be in output surQ surface,      */
    /*   rq = max index of U ControlPts to be in output surQ surface, */
    /*   sq = max index of V ControlPts to be in output surQ surface, */

    /* Next: Insert knot into surB,                        */
    /*       load UQ and VQ,                               */
    /*       partially load Qw, and                        */
    /*       load Rw with the required ctrl pts from surB  */

    rem = 0;

    /* when dflg == NL_UDIR branch */
    if( dflg EQ NL_UDIR )
    {
        /* pick new param bound value */
        if     (cont NEQ NL_CMAX)  { uuu = dReflPrmU; }
        else if(end  EQ  NL_START) { uuu = UP[p + 1]; }
        else                       { uuu = UP[r - p - 1]; }

        /* make new knot full multiplicity */
        if( mult LT p )
        {
            rem = p - mult;
            error = N_SrfInsertKnot(&surB,   /* in : tgt surface */
                                    uuu,     /* in : new knot to be inserted */
                                    rem,     /* in : number of times to insert new knot t */
                                    NL_UDIR, /* in : NL_UDIR: Insert in u-direction */
                                             /*      NL_VDIR: Insert in v-direction */
                                    &surB,   /* out: Surface after knot inserion */
                                    &SL,     /* in : surP's stack */
                                    &SL );   /* in : surQ's stack */

            if( error EQ NL_YES )
              { NL_OUT; }
        }

        /* copy non-extending direction knot values */
        for ( ii = 0; ii <= s; ii++ )
          { VQ[ii] = VP[ii]; }
        
        /* get surB internals */
        N_SrfGetCPtsDegreesAndKnots(&surB,  /* in : tgt surface */
                                    &nb,    /* out: max 1st index in Pw - number of U control points = n + 1 */
                                    &mb,    /* out: max 2nd index in Pw - number of V control points = m + 1 */
                                    &Bw,    /* out: Pw sized:[n+1][m+1]  */
                                    &p,     /* out: u degree */
                                    &q,     /* out: v degree */
                                    &rb,    /* out: max knot index in U array - number of u knots = r + 1 */
                                    &sb,    /* out: max knot index in V array - number of v knots = s + 1 */
                                    &UB,    /* out: U knot vector sized:[r+1] */
                                    &VB );  /* out: V knot vector sized:[s+1] */

        /* when end == NL_START branch */
        if( end EQ NL_START )
        {
            /* number of new surQ control points to be generated for the extension */
            kby = nq - nb;

            /* for every non-extending direction control point */
            for ( ii = 0; ii <= mb; ii++ )
            {
                /* for every surB extending dir control point */
                for ( jj = 0; jj <= nb; jj++ )
                  { 
                    /* copy all surB ctrlPt into surQ leaving room for new Extend CPts */
                    N_CopyCPt( Bw[jj][ii], &Qw[kby + jj][ii] ); 
                  }

                /* for every to-be-mirrored surB extending dir control point */
                for ( jj = 0; jj <= kby; jj++ )
                  { 
                    /* copy to-be-mirrored surB ctrlPt into Rw array - leaving room for new Extend CPts*/
                    N_CopyCPt( Bw[jj][ii], &Rw[ii][jj + kby] ); 
                  }
            }

            /* when cont == NL_CMAX branch (last ivl extension) */
            if( cont EQ NL_CMAX )
            {
                kkt = p + 1;

                for ( ii = 0; ii <= p; ii++ )
                  { UQ[ii] = 2.0 *UB[0] - dReflPrmU; }

                for ( ii = 1; ii <= rb; ii++ )
                  { UQ[p + ii] = UB[ii]; }
            } /* end cont == NL_CMAX branch */

            else /* when cont == NL_G1 or NL_G2 branch (reflection) */
            {
                kkt = span + 1;
                jj = kkt - 1;

                /* copy surB extend-dir knots that are the same into VQ for surQ */
                for ( ii = 1; ii <= rb; ii++ )
                  { UQ[span + ii] = UB[ii]; }

                /* set VQ knots between old knots and new boundary */
                for ( ii = p + 1; ii <= span; ii++ )
                {
                    UQ[jj] = UQ[jj + 1] - (UB[ii] - UB[ii - 1]);
                    jj -= 1;
                }
                d1 = UQ[jj + 1] - (UB[kkt] - UB[span]);

                /* set VQ new boundary knot values */
                for ( ii = 0; ii <= p; ii++ )
                  { UQ[ii] = d1; }
            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end == NL_START) branch */

        else  /* when end == NL_END branch*/ 
        {
            kby = nq - nb;
            kk = nb - kby;

            for ( ii = 0; ii <= mb; ii++ )
            {
                for ( jj = 0; jj <= nb; jj++ )
                  { N_CopyCPt( Bw[jj][ii], &Qw[jj][ii] ); }

                for ( jj = 0; jj <= kby; jj++ )
                  { N_CopyCPt( Bw[jj + kk][ii], &Rw[ii][jj] ); }
            }

            kkt = rb - 1;

            for ( ii = 0; ii < rb; ii++ )
              { UQ[ii] = UB[ii]; }

            /* when cont == NL_CMAX branch (last ivl extension) */
            if( cont EQ NL_CMAX )
            {
                for ( ii = 0; ii <= p; ii++ )
                  { UQ[rb + ii] = 2.0 *UB[rb] - dReflPrmU; }
            } /* end cont == NL_CMAX branch */

            else /* when cont == NL_G1 or NL_G2 branch (reflection) */
            {
                jj = rb;

                for ( ii = r - p - 1; ii > span; ii-- )
                {
                    UQ[jj] = UQ[jj - 1] + (UP[ii + 1] - UP[ii]);
                    jj += 1;
                }
                d1 = UQ[jj - 1] + (UP[span + 1] - dReflPrmU);

                for ( ii = 0; ii <= p; ii++ )
                  { UQ[jj + ii] = d1; }

            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end == NL_END) branch */
    } /* end dflg == NL_UDIR branch */

    else  /* dflg == NL_VDIR branch */
    {
        /* pick new param bound value */
        if     (cont NEQ NL_CMAX) { vvv = dReflPrmV; }
        else if(end EQ NL_START)  { vvv = VP[q + 1]; }
        else                      { vvv = VP[s - q - 1]; }

        /* make new knot full multiplicity */
        if( mult LT q )
        {
            rem = q - mult;
            error = N_SrfInsertKnot(&surB,   /* in : tgt surface */
                                    vvv,     /* in : new knot to be inserted */
                                    rem,     /* in : number of times to insert new knot t */
                                    NL_VDIR, /* in : NL_UDIR: Insert in u-direction */
                                             /*      NL_VDIR: Insert in v-direction */
                                    &surB,   /* out: Surface after knot inserion */
                                    &SL,     /* in : surP's stack */
                                    &SL );   /* in : surQ's stack */

            if( error EQ NL_YES )
              { NL_OUT; }
        }

        /* copy non-extending directiong knot values */
        for ( ii = 0; ii <= r; ii++ )
          { UQ[ii] = UP[ii]; }

        N_SrfGetCPtsDegreesAndKnots(&surB,  /* in : tgt surface */
                                    &nb,    /* out: max 1st index in Pw - number of U control points = n + 1 */
                                    &mb,    /* out: max 2nd index in Pw - number of V control points = m + 1 */
                                    &Bw,    /* out: Pw sized:[n+1][m+1]  */
                                    &p,     /* out: u degree */
                                    &q,     /* out: v degree */
                                    &rb,    /* out: max knot index in U array - number of u knots = r + 1 */
                                    &sb,    /* out: max knot index in V array - number of v knots = s + 1 */
                                    &UB,    /* out: U knot vector sized:[r+1] */
                                    &VB );  /* out: V knot vector sized:[s+1] */

        /* when end == NL_START branch */
        if( end EQ NL_START )
        {
            /* number of new surQ control points to be generated for the extension */
            kby = mq - mb; 

            /* for every non-extending direction control point */
            for ( ii = 0; ii <= nb; ii++ ) 
            {
                /* for every surB extending dir control point */
                for ( jj = 0; jj <= mb; jj++ )  
                  { 
                    /* copy all surB ctrlPt into surQ leaving room for new Extend CPts */
                    N_CopyCPt( Bw[ii][jj], &Qw[ii][kby + jj] ); 
                  }
                   
                /* for every to-be-mirrored surB extending dir control point */
                for ( jj = 0; jj <= kby; jj++ ) 
                  { 
                    /* copy to-be-mirrored surB ctrlPt into Rw array - leaving room for new Extend CPts*/
                    N_CopyCPt( Bw[ii][jj], &Rw[ii][jj + kby] ); 
                  } 
            } /* end copying surB CPts into surQ and Rw */

            /* when cont == NL_CMAX branch (last ivl extension) */
            if( cont EQ NL_CMAX )
            {
                kkt = q + 1;

                for ( ii = 0; ii <= q; ii++ )
                  { VQ[ii] = 2.0 *VB[0] - dReflPrmV; }

                for ( ii = 1; ii <= sb; ii++ )
                  { VQ[q + ii] = VB[ii]; }
            } /* end cont == NL_CMAX branch */

            else /* when cont == NL_G1 or NL_G2 branch (reflection) */
            {
                kkt = span + 1;
                jj = kkt - 1;

                /* copy surB extend-dir knots that are the same into VQ for surQ */
                for ( ii = 1; ii <= sb; ii++ ) 
                  { VQ[span + ii] = VB[ii]; }

                /* set VQ knots between old knots and new boundary */
                for ( ii = q + 1; ii <= span; ii++ ) 
                {
                    VQ[jj] = VQ[jj + 1] - (VB[ii] - VB[ii - 1]);
                    jj -= 1;
                }
                d1 = VQ[jj + 1] - (VB[kkt] - VB[span]);

                /* set VQ new boundary knot values */
                for ( ii = 0; ii <= q; ii++ )  
                  { VQ[ii] = d1; }
            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end == NL_START) branch */

        else  /* when end == NL_END branch*/ 
        {
            kby = mq - mb;
            kk = mb - kby;

            for ( ii = 0; ii <= nb; ii++ )
            {
                for ( jj = 0; jj <= mb; jj++ )
                  { N_CopyCPt( Bw[ii][jj], &Qw[ii][jj] ); }

                for ( jj = 0; jj <= kby; jj++ )
                  { N_CopyCPt( Bw[ii][jj + kk], &Rw[ii][jj] ); }
            }

            kkt = sb - 1;

            for ( ii = 0; ii < sb; ii++ )
              { VQ[ii] = VB[ii]; }

            /* when cont == NL_CMAX branch (last ivl extension) */
            if( cont EQ NL_CMAX )
            {
                for ( ii = 0; ii <= q; ii++ )
                  { VQ[sb + ii] = 2.0 *VB[sb] - dReflPrmV; }
            } /* end cont == NL_CMAX branch */

            else /* when cont == NL_G1 or NL_G2 branch (reflection) */
            {
                jj = sb;

                for ( ii = s - q - 1; ii > span; ii-- )
                {
                    VQ[jj] = VQ[jj - 1] + (VP[ii + 1] - VP[ii]);
                    jj += 1;
                }
                d1 = VQ[jj - 1] + (VP[span + 1] - dReflPrmV);

                for ( ii = 0; ii <= q; ii++ )
                  { VQ[jj + ii] = d1; }

            } /* end cont == NL_G1 or NL_G2 branch */
        } /* end (end == NL_END) branch */
    } /* end dflg == NL_VDIR branch */

    /* arrive here for reflection or last interval extension cases (cont == NL_G1, NL_G2, or NL_CMAX) */
    /*   nq = max index of U knots to be in output surQ surface,      */
    /*   mq = max index of V knots to be in output surQ surface,      */
    /*   rq = max index of U ControlPts to be in output surQ surface, */
    /*   sq = max index of V ControlPts to be in output surQ surface, */

    /* surQ sized for output */ /* SurP copied to SurB, then  Inserted knot into surB for the reflection parameter */
    /*   finished surQ->knu and surQ->knv knot arrays                               */
    /*   partially loaded surQ->Pw = Qw with all surP->CPts that stay the same - still needs new Extension CPts*/
    /*   partially loaded Rw with surB->CPts that will be used to compute the extension CPts (through mirroring)  */

    /* Next: Compute the new control points defining the extension.             */
    /*       for NL_G1 & NL_G2 - Use method by Shetty & White, CAD, Sept. 1991. */
    /*       for NL_CMAX       - Use derivative expressions.                    */

    rat = N_IsSrfRat( &surA );   /* note: surA = *surP */

    /* output sur max knot index and degree */
    if(dflg EQ NL_UDIR) { nn = mq;  /* mq = max index of extend dir CPts to be in output surQ surface */
                          pq = p;   /* extend dir degree */
                        }
    else                { nn = nq;  /* nq = max index of extend dir CPts to be in output surQ surface */
                          pq = q;   /* extend dir degree */
                        }

    /* Initialize pln in any case to silence warning */
    P1.x = 0.0; P1.y = 0.0; P1.z = 0.0;
    V.x = 0; V.y = 0; V.z = 1.0;
    N_CreatePlanePtNormal( &pln, P1, V ); 

    /* when cont == NL_G1 & NL_G2 branch */
    if( cont NEQ NL_CMAX )
    {
        /*  Use Shetty & White.  */

        /* when (end == NL_START) branch */
        if( end EQ NL_START )
        {
            /* for every extend dir CPt row - mirror row CPts about last row CPt refelction plane */
            for ( ii = 0; ii <= nn; ii++ )  
            {
                /* get reflection plane in euclidean space */
                N_CPtToPtEuclid( Rw[ii][kby], &P1 ); 
                N_CPtToPtEuclid( Rw[ii][kby + 1], &P2 );
                N_VectorDir( P2,   /* in : Ps  of Vec = Pe - Ps */
                             P1,   /* in : Pe  of Vec = Pe - Ps */
                             &V ); /* out: Vec of Vec = Pe - Ps */
                error = N_VectorNormalize(V,     /* in : Given vector */
                                          &V,    /* out: Normalized vector */
                                          &d1 ); /* out: Magnitude of given vector */

                if( error )
                {
                    N_CPtToPtEuclid( Rw[ii][kby + 2], &P2 );
                    N_VectorDir( P2,   /* in : Ps  of Vec = Pe - Ps */
                                 P1,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    error = N_VectorNormalize(V,     /* in : Given vector */
                                              &V,    /* out: Normalized vector */
                                              &d1 ); /* out: Magnitude of given vector */
                }

                /* If we hit singularity here, just do straight copying.*/
                if( error NEQ NL_YES )
                {
                    N_CreatePlanePtNormal(&pln, /* out: Plane */
                                          P1,   /* in : Point lying on the plane */
                                          V );  /* in : Normal vector */
                }
                
                /* load reflected control points */
                kk = kby - 1; 

                /* for every reflected extend dir row - reflect control point into appropriate Rw slot */
                for ( jj = 1; jj <= kby; jj++ )
                {
                    if( error NEQ NL_YES )
                    {
                        N_CptReflect( Rw[ii][kby + jj], /* in : Homogeneous point to be reflected */
                                      pln,              /* in : The reflection plane */
                                      &Rw[ii][kk] );    /* out: Reflection of Pw */
                    }
                    else
                    {
                        Rw[ii][kk] = Rw[ii][kby + jj];
                    }
                    kk -= 1;
                }
            } /* end iter every extend-dir row mirroring CPts - For nonrat surf, this already gives NL_G1 continuity */

            if( (NOT rat) AND cont EQ NL_G2 AND pq GT 1 )
            { 
                /* recompute second row using NL_C2 condition */
                if( dflg EQ NL_UDIR )
                  { d1 = 2.0 + (UB[p + 2] - UB[p + 1]) / (UB[p + 1] - UB[0]); }
                else
                  { d1 = 2.0 + (VB[q + 2] - VB[q + 1]) / (VB[q + 1] - VB[0]); }

                for ( ii = 0; ii <= nn; ii++ )
                  { N_TranslateSum2CPts(Rw[ii][kby + 2],    /* in : Cw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        d1,                 /* in : alpha of Dw = Cw + alpha*Aw + beta*Bw */
                                        Rw[ii][kby - 1],    /* in : Aw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        -d1,                /* in : beta  of Dw = Cw + alpha*Aw + beta*Bw */
                                        Rw[ii][kby + 1],    /* in : Bw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        &Rw[ii][kby - 2] ); /* out: Dw    of Dw = Cw + alpha*Aw + beta*Bw */
                  }
            } /* For nonrat surf, this gives NL_G2 continuity */

            /* for rational surfs - compute the first 1 or 2 reflection rows differently to get NL_G1 or NL_G2 */
            if( rat ) 
            {
                if( dflg EQ NL_UDIR ) { d1 = 2.0 *p / (UB[p + 1] - UB[0]); }
                else                  { d1 = 2.0 *q / (VB[q + 1] - VB[0]); }
                tal = -1.0e+20;

                /* for every extend dir CPt row - find greatest d2 value */
                for ( ii = 0; ii <= nn; ii++ ) 
                {
                    N_CPtGetW( Rw[ii][kby], &w0 );
                    N_CPtGetW( Rw[ii][kby + 1], &w1 );
                    d2 = d1 * (w1 - w0) / w0;

                    /* find greatest d2 value - ktal = row with greatest d2 value */
                    if( d2 GT tal ) 
                    {
                        tal = d2;
                        ktal = ii;
                    }
                }

                /* when cont == NL_G2 */
                if( cont EQ NL_G2 AND pq GT 1 )
                { 
                    /* compute theta and e0 (Shetty & White) */
                    N_InitRealMatrix( &rma );
                    error = N_SetRealMatrix(&rma,       /* i/o: target real matrix */
                                            3,          /* in : highest 1st index */
                                            1,          /* in : highest 2nd index */
                                            NL_MT_FULL, /* in : Matrix Type: NL_MT_FULL      */
                                                        /*                   NL_MT_LOWERLEFT */
                                                        /*                   NL_MT_UPPERRIGH */
                                                        /*                   NL_MT_BANDED    */
                                            3,          /* in : Bandwidth */
                                            &SL );      /* in : Memory stacks pointer */
                    N_GetRealMatrixData( &rma, &ii, &jj, &RM, &mtp, &jj );

                    if( dflg EQ NL_UDIR )
                    {
                        d3 = UB[p + 1] - UB[0];
                        d4 = UB[p + 2] - UB[p + 1];
                        d1 = p / d3;
                        d2 = (p * ((NL_REAL)p - 1.0)) / (d3 * (d3 + d4));
                    }
                    else
                    {
                        d3 = VB[q + 1] - VB[0];
                        d4 = VB[q + 2] - VB[q + 1];
                        d1 = q / d3;
                        d2 = (q * ((NL_REAL)q - 1.0)) / (d3 * (d3 + d4));
                    }
                    d4 = (2.0 *d3 + d4) / d3;
                    d3 = -2.0 *tal * d1;

                    for ( ii = 0; ii <= 4; ii++ )
                      { N_CPtToWxWyWz(Rw[ktal][kby - 2 + ii], /* in : Tgt Control point */
                                      &hh[ii][0],             /* out: x of Cpt[x y z w] */
                                      &hh[ii][1],             /* out: y of Cpt[x y z w] */
                                      &hh[ii][2],             /* out: z of Cpt[x y z w] */
                                      &hh[ii][3] ); }         /* out: w of Cpt[x y z w] */

                    for ( ii = 0; ii <= 3; ii++ )
                    { /* set up system of 4 eqs in 2 unknowns */
                        RM[ii][0] = d1 * (hh[2][ii] - hh[3][ii]);
                        RM[ii][1] = hh[2][ii];
                        rhs[ii]   =   d2 * (d4 * (hh[3][ii] - hh[1][ii]) 
                                               + (hh[0][ii] - hh[4][ii])) 
                                    + d3 * (hh[2][ii] - hh[3][ii]);
                    }             

                    /* solve system */
                    error = N_RealMatrixLstSqSolve(&rma,  /* in : NL_RMATRIX *A of Ax=b, with m+1 rows and n+1 columns (m>n) */
                                                   rhs,   /* in : NL_REAL    *b of Ax=b, sized:[m+1] */
                                                   sol ); /* out: NL_REAL    *x of Ax=b, sized:[n+1] */

                    if( error EQ NL_YES )
                      { NL_OUT; }
                } /* end continuity == NL_G2 on degree > 1 surfs - so must compute theta and e0 values check */

                if( dflg EQ NL_UDIR ) { d1 = 2.0 + tal * (UQ[kkt] - UQ[kkt - 1]) / pq; }
                else                  { d1 = 2.0 + tal * (VQ[kkt] - VQ[kkt - 1]) / pq; }

                /* Note: These coefficients don't add up to 1.
                * That's ok because we're in homogeneous space.
                * d1 times the first pt is the same point in 3d space, just with a different weight.
                * d1 is actually set up so that the result will have the same weight as the second point.
                */
                /* recompute first row for NL_G1 */
                for ( ii = 0; ii <= nn; ii++ ) 
                  { N_Combine2CPts(d1,                    /* alpha of Cw = alpha * Aw + beta * Bw */
                                   Rw[ii][kby],           /* Aw    of Cw = alpha * Aw + beta * Bw */
                                   -1.0,                  /* beta  of Cw = alpha * Aw + beta * Bw */
                                   Rw[ii][kby + 1],       /* Bw    of Cw = alpha * Aw + beta * Bw */
                                  &Rw[ii][kby - 1] );     /* Cw    of Cw = alpha * Aw + beta * Bw */
                  }

                /* when NL_G2 continuity */
                if( cont EQ NL_G2 AND pq GT 1 )
                { 
                    /* recompute second row for NL_G2 */
                    d2 = 1.0 / d2;
                    d1 = d1 * (sol[0] + 2.0 *tal);

                    /* for every extend dir row */
                    for ( ii = 0; ii <= nn; ii++ )
                    {
                        /* for every affected CPt in current extend dir row */
                        for ( jj = 1; jj <= 4; jj++ )
                          { N_CPtToWxWyWz(Rw[ii][kby - 2 + jj], /* in : Tgt Control point */
                                          &hh[jj][0],           /* out: x of Cpt[x y z w] */
                                          &hh[jj][1],           /* out: y of Cpt[x y z w] */
                                          &hh[jj][2],           /* out: z of Cpt[x y z w] */
                                          &hh[jj][3] ); }       /* out: w of Cpt[x y z w] */

                        for ( jj = 0; jj <= 3; jj++ )
                          { hh[0][jj] =  hh[4][jj] 
                                       + d4 * (hh[1][jj] - hh[3][jj]) 
                                       + d2 * (  d1 * (hh[2][jj] - hh[3][jj]) 
                                               + sol[1] * hh[2][jj]); 
                          }

                        /* and for the final point in the current extend dir row */
                        N_CPtFromWxWyWz(hh[0][0],           /* in :  x  of CPt[x y z w] */
                                        hh[0][1],           /* in :  y  of CPt[x y z w] */
                                        hh[0][2],           /* in :  z  of CPt[x y z w] */
                                        hh[0][3],           /* in :  w  of CPt[x y z w] */
                                        &Rw[ii][kby - 2] ); /* out: CPt of CPt[x y z w] */
                    }
                } /* end need to recompute 2nd row due to NL_G2 check */
            } /* end Is Rational check */
        } /* end (end == NL_START) branch */

        else /* when (end == NL_END) branch */
        {
            /* for every extend dir CPt row - mirror row CPts about last row CPt refelction plane */
            for ( ii = 0; ii <= nn; ii++ )
            {
                /* get reflection plane in euclidean space */
                N_CPtToPtEuclid( Rw[ii][kby], &P1 ); /* get reflection plane */
                N_CPtToPtEuclid( Rw[ii][kby - 1], &P2 );
                N_VectorDir(P2,    /* in : Ps  of Vec = Pe - Ps */
                            P1,    /* in : Pe  of Vec = Pe - Ps */
                            &V );  /* out: Vec of Vec = Pe - Ps */
                error = N_VectorNormalize( V, &V, &d1 );

                if( error )
                { /* try for prev point */
                    N_CPtToPtEuclid( Rw[ii][kby - 2], &P2 );
                    N_VectorDir( P2,   /* in : Ps  of Vec = Pe - Ps */
                                 P1,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    error = N_VectorNormalize( V, &V, &d1 );
                }

                /* if you have no reflection plane
                 just do straight copying */
                if( error NEQ NL_YES )
                {
                    N_CreatePlanePtNormal( &pln, /* out: Plane */
                                           P1,   /* in : Point lying on the plane */
                                           V );  /* in : Normal vector */
                }
                
                /* load reflected control points */
                kk = 2 * kby; /* load reflected control points */

                /* for every reflected extend dir row - reflect control point into appropriate Rw slot */
                for ( jj = 0; jj < kby; jj++ )
                {
                    if( error NEQ NL_YES )
                    {
                        N_CptReflect( Rw[ii][jj],    /* in : Homogeneous point to be reflecte */
                                      pln,           /* in : The reflection plane */
                                      &Rw[ii][kk] ); /* out: Reflection of Pw */
                    }
                    else
                    {
                        Rw[ii][kk] = Rw[ii][jj];
                    }
                    kk -= 1;
                }
            } /* end iter every extend-dir row mirroring CPts - For nonrat surf, this already gives NL_G1 continuity */

            if( (NOT rat)AND cont EQ NL_G2 AND pq GT 1 )
            { 
                /* recompute second row using NL_C2 condition */
                if( dflg EQ NL_UDIR )
                  { d1 = 2.0 + (UB[rb - p - 1] - UB[rb - p - 2]) / (UB[rb] - UB[rb - p - 1]); }
                else
                  { d1 = 2.0 + (VB[sb - q - 1] - VB[sb - q - 2]) / (VB[sb] - VB[sb - q - 1]); }

                for ( ii = 0; ii <= nn; ii++ )
                  { N_TranslateSum2CPts(Rw[ii][kby - 2],    /* in : Cw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        d1,                 /* in : alpha of Dw = Cw + alpha*Aw + beta*Bw */
                                        Rw[ii][kby + 1],    /* in : Aw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        -d1,                /* in : beta  of Dw = Cw + alpha*Aw + beta*Bw */
                                        Rw[ii][kby - 1],    /* in : Bw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        &Rw[ii][kby + 2] ); /* out: Dw    of Dw = Cw + alpha*Aw + beta*Bw */
                  }
            } /* For nonrat surf, this gives NL_G2 continuity */

            /* when rat surf - must compute 1st and 2nd reflected rows differentlyt to get NL_G1 or NL_G2 */
            if( rat ) 
            {
                if( dflg EQ NL_UDIR )
                  { d1 = 2.0 *p / (UB[rb] - UB[rb - p - 1]); }
                else
                  { d1 = 2.0 *q / (VB[sb] - VB[sb - q - 1]); }
                tal = -1.0e+20;

                /* for every extend dir CPt row - find greatest d2 value */
                for ( ii = 0; ii <= nn; ii++ )
                {
                    N_CPtGetW( Rw[ii][kby], &w0 );
                    N_CPtGetW( Rw[ii][kby - 1], &w1 );
                    d2 = d1 * (w1 - w0) / w0;

                    /* find greatest d2 value - ktal = row with greatest d2 value */
                    if( d2 GT tal )
                    {
                        tal = d2;
                        ktal = ii;
                    }
                }

                /* when cont == NL_G2 */
                if( cont EQ NL_G2 AND pq GT 1 )
                { 
                    /* compute theta and e0 (Shetty & White) */
                    N_InitRealMatrix( &rma );
                    error = N_SetRealMatrix(&rma,       /* i/o: target real matrix */
                                            3,          /* in : highest 1st index */
                                            1,          /* in : highest 2nd index */
                                            NL_MT_FULL, /* in : Matrix Type: NL_MT_FULL      */
                                                        /*                   NL_MT_LOWERLEFT */
                                                        /*                   NL_MT_UPPERRIGH */
                                                        /*                   NL_MT_BANDED    */
                                            3,          /* in : Bandwidth */
                                            &SL );      /* in : Memory stacks pointer */
                    N_GetRealMatrixData( &rma, &ii, &jj, &RM, &mtp, &jj );

                    if( dflg EQ NL_UDIR )
                    {
                        d3 = UB[rb] - UB[rb - p - 1];
                        d4 = UB[rb - p - 1] - UB[rb - p - 2];
                        d1 = p / d3;
                        d2 = (p * ((NL_REAL)p - 1.0)) / (d3 * (d3 + d4));
                    }
                    else
                    {
                        d3 = VB[sb] - VB[sb - q - 1];
                        d4 = VB[sb - q - 1] - VB[sb - q - 2];
                        d1 = q / d3;
                        d2 = (q * ((NL_REAL)q - 1.0)) / (d3 * (d3 + d4));
                    }
                    d4 = (2.0 *d3 + d4) / d3;
                    d3 = -2.0 *tal * d1;

                    for ( ii = 0; ii <= 4; ii++ )
                      { N_CPtToWxWyWz(Rw[ktal][kby - 2 + ii], /* in : Tgt Control point */
                                      &hh[ii][0],             /* out: x of Cpt[x y z w] */
                                      &hh[ii][1],             /* out: y of Cpt[x y z w] */
                                      &hh[ii][2],             /* out: z of Cpt[x y z w] */
                                      &hh[ii][3] ); }         /* out: w of Cpt[x y z w] */

                    for ( ii = 0; ii <= 3; ii++ )
                    { /* set up system of 4 eqs in 2 unknowns */
                        RM[ii][0] = d1 * (hh[2][ii] - hh[3][ii]);
                        RM[ii][1] = hh[2][ii];
                        rhs[ii] = d2 * (d4 * (hh[3][ii] - hh[1][ii]) + (hh[0][ii] - hh[4][ii])) + d3 * (hh[2][ii] - hh[3][ii]);
                    }

                    /* solve system */
                    error = N_RealMatrixLstSqSolve(&rma,  /* in : NL_RMATRIX *A of Ax=b, with m+1 rows and n+1 columns (m>n) */
                                                   rhs,   /* in : NL_REAL    *b of Ax=b, sized:[m+1] */
                                                   sol ); /* out: NL_REAL    *x of Ax=b, sized:[n+1] */

                    if( error EQ NL_YES )
                      { NL_OUT; }
                } /* end continuity == NL_G2 on degree > 1 surfs - so must compute theta and e0 values check */

                if( dflg EQ NL_UDIR ) { d1 = 2.0 + tal * (UQ[kkt + 1] - UQ[kkt]) / pq; }
                else                  { d1 = 2.0 + tal * (VQ[kkt + 1] - VQ[kkt]) / pq; }

                for ( ii = 0; ii <= nn; ii++ ) /* recompute first row for NL_G1 */
                  { N_Combine2CPts(d1,                 /* alpha of Cw = alpha * Aw + beta * Bw */
                                   Rw[ii][kby],        /* Aw    of Cw = alpha * Aw + beta * Bw */
                                   -1.0,               /* beta  of Cw = alpha * Aw + beta * Bw */
                                   Rw[ii][kby - 1],    /* Bw    of Cw = alpha * Aw + beta * Bw */
                                   &Rw[ii][kby + 1] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                  }

                /* when NL_G2 continuity */
                if( cont EQ NL_G2 AND pq GT 1 )
                { 
                    /* recompute second row for NL_G2 */
                    d2 = 1.0 / d2;
                    d1 = d1 * (sol[0] + 2.0 *tal);

                    /* for every extend dir row */
                    for ( ii = 0; ii <= nn; ii++ )
                    {
                        /* for every affected CPt in current extend dir row */
                        for ( jj = 0; jj <= 3; jj++ )
                          { N_CPtToWxWyWz(Rw[ii][kby - 2 + jj], /* in : Tgt Control point */ 
                                          &hh[jj][0],           /* out: x of Cpt[x y z w] */
                                          &hh[jj][1],           /* out: y of Cpt[x y z w] */
                                          &hh[jj][2],           /* out: z of Cpt[x y z w] */
                                          &hh[jj][3] );         /* out: w of Cpt[x y z w] */
                           }

                        for ( jj = 0; jj <= 3; jj++ )
                          { hh[4][jj] =   hh[0][jj] 
                                        - d4 * (hh[1][jj] - hh[3][jj]) 
                                        + d2 * (  d1 * (  hh[2][jj] 
                                                        - hh[3][jj]) 
                                                + sol[1] * hh[2][jj]); 
                          }

                        /* and for the final point in the current extend dir row */
                        N_CPtFromWxWyWz(hh[4][0],           /* in :  x  of CPt[x y z w] */
                                        hh[4][1],           /* in :  y  of CPt[x y z w] */
                                        hh[4][2],           /* in :  z  of CPt[x y z w] */
                                        hh[4][3],           /* in :  w  of CPt[x y z w] */
                                        &Rw[ii][kby + 2] ); /* out: CPt of CPt[x y z w] */
                    }
                } /* end need to recompute 2nd row for NL_G2 check */
            } /* end rational surface check */
        } /* end (end == NL_END) branch */
    } /* end cont == NL_G1 & NL_G2 branch */
    
    else /* when cont == NL_CMAX branch  */
    {
        kk = pq;

        /*          bin = N_AllocInt2dArray(kk,kk,&SL);    */
        /*          if ( bin EQ NULL )  NL_QUIT;     */
        /*          N_PascalTriRow(bin,kk);             */

        Dw = N_AllocCPt1dArray( pq, &SL );

        if( Dw EQ NULL )
          { NL_QUIT; }

        /* when (end == NL_START) branch */
        if( end EQ NL_START )
        {
            if( dflg EQ NL_UDIR ) { d2 = dReflPrmU - UB[0];
                                    d3 = UB[p + 1] - UB[0];
                                  }
            else                  { d2 = dReflPrmV - VB[0];
                                    d3 = VB[q + 1] - VB[0];
                                  }
            d4 = 1.0 / d3;

            /* for every knot in the extension direction */
            for ( ii = 0; ii <= nn; ii++ )
            {
                dr = pq * d4;

                /* for every controlPoint affecting this span */
                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes derivatives */
                {                              /* from the right                 */
                    N_CopyCPt( Rw[ii][kby + jj], &Dw[jj] );

                    if( jj % 2 EQ 0 )
                      { kp = 1; }
                    else
                      { kp = -1; }

                    for ( kk = 0; kk < jj; kk++ ) /* compute the jj-th derivative */
                    {
                        N_VectorBlendCPt((NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], /* in : alpha of Bw = Bw + alpha*Aw */
                                         Rw[ii][kby + kk],          /* in : Aw    of Bw = Bw + alpha*Aw */
                                         &Dw[jj] );                 /* out: Bw    of Bw = Bw + alpha*Aw */
                        kp *= -1;
                    }
                    N_ScaleCPt(dr,        /* in : alpha of Bw = alpha*Aw */
                               Dw[jj],    /* in : Aw    of Bw = alpha*Aw */
                               &Dw[jj] ); /* out: Bw    of Bw = alpha*Aw */
                    dr = dr * ((NL_REAL)pq - (NL_REAL)jj) * d4;
                } /* end iter every controlPoint affecting this span */

                dl = d2 / pq;

                /* for every controlPoint affecting this span */
                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes */
                {                              /* new control points */
                    if( jj % 2 EQ 0 )
                      { kp = -1; }
                    else
                      { kp = 1; }

                    N_CopyCPt( Dw[jj], &Rw[ii][pq - jj] );
                    N_ScaleCPt( -kp * dl,            /* in : alpha of Bw = alpha*Aw */
                                Rw[ii][pq - jj],     /* in : Aw    of Bw = alpha*Aw */
                                &Rw[ii][pq - jj] );  /* out: Bw    of Bw = alpha*Aw */

                    if( jj LT pq )
                      { dl = dl * d2 / ((NL_REAL)pq - (NL_REAL)jj); }

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (p-jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], /* in : alpha of Bw = Bw + alpha*Aw */
                                          Rw[ii][pq - kk],           /* in : Aw    of Bw = Bw + alpha*Aw */
                                          &Rw[ii][pq - jj] );        /* out: Bw    of Bw = Bw + alpha*Aw */
                        kp *= -1;
                    }
                } /* end iter every controlPoint affecting this span */

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 0; jj < pq; jj++ )
                    {
                        N_CPtGetW( Rw[ii][jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                          { NL_ERROR( NL_WEI_ERR ); }
                    }
                }

            }  /* end iter every knot in the extension direction */
        } /* end (end == NL_START) branch */

        else /* when (end == NL_END) branch */
        {
            if( dflg EQ NL_UDIR ) { d2 = UB[rb] - dReflPrmU;
                                    d3 = UB[rb] - UB[rb - p - 1];
                                  }
            else                  { d2 = VB[sb] - dReflPrmV;
                                    d3 = VB[sb] - VB[sb - q - 1];
                                  }
            d4 = 1.0 / d3;

            /* for every knot in the extension direction */
            for ( ii = 0; ii <= nn; ii++ )
            {
                dl = -pq * d4;

                /* for every controlPoint affecting this span */
                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes derivatives */
                {                              /* from the left                  */
                    N_CopyCPt( Rw[ii][kby - jj], &Dw[jj] );

                    if( jj % 2 EQ 0 )
                      { kp = 1; }
                    else
                      { kp = -1; }

                    for ( kk = 0; kk < jj; kk++ ) /* compute the jj-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], /* in : alpha of Bw = Bw + alpha*Aw */
                                          Rw[ii][kby - kk],          /* in : Aw    of Bw = Bw + alpha*Aw */
                                          &Dw[jj] );                 /* out: Bw    of Bw = Bw + alpha*Aw */
                        kp *= -1;
                    }
                    N_ScaleCPt( dl,         /* in : alpha of Bw = alpha*Aw */
                                Dw[jj],     /* in : Aw    of Bw = alpha*Aw */
                                &Dw[jj] );  /* out: Bw    of Bw = alpha*Aw */
                    dl = -dl * ((NL_REAL)pq - (NL_REAL)jj) * d4;
                } /* end iter every controlPoint affecting this span */

                dr = d2 / pq;

                /* for every controlPoint affecting this span */
                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes */
                {                              /* new control points */
                    if( jj % 2 EQ 0 )
                      { kp = -1; }
                    else
                      { kp = 1; }

                    N_CopyCPt( Dw[jj], &Rw[ii][kby + jj] );
                    N_ScaleCPt( dr,                  /* in : alpha of Bw = alpha*Aw */
                                Rw[ii][kby + jj],    /* in : Aw    of Bw = alpha*Aw */
                                &Rw[ii][kby + jj] ); /* out: Bw    of Bw = alpha*Aw */

                    if( jj LT pq )
                      { dr = dr * d2 / ((NL_REAL)pq - (NL_REAL)jj); }

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (kby+jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], /* in : alpha of Bw = Bw + alpha*Aw */
                                          Rw[ii][kby + kk],          /* in : Aw    of Bw = Bw + alpha*Aw */
                                          &Rw[ii][kby + jj] );       /* out: Bw    of Bw = Bw + alpha*Aw */
                        kp *= -1;
                    }
                } /* end iter every controlPoint affecting this span */

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj <= pq; jj++ )
                    {
                        N_CPtGetW( Rw[ii][kby + jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                          { NL_ERROR( NL_WEI_ERR ); }
                    }
                }

            }  /* end iter every knot in the extension direction */
        } /* end (end == NL_END) branch */
    } /* end cont == NL_CMAX branch  */

    /* Now load the new extend dir CPts from Rw (where they were computed) into surQ->Qw (its a complete surface) */

    /* when dflg == NL_UDir branch */
    if( dflg EQ NL_UDIR )
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ ) 
            {
                for ( jj = 0; jj < kby; jj++ )
                  { N_CopyCPt( Rw[ii][jj], &Qw[jj][ii] ); }
            }
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ ) 
            {
                for ( jj = 1; jj <= kby; jj++ )
                  { N_CopyCPt( Rw[ii][kby + jj], &Qw[nb + jj][ii] ); }
            }
        }
    } /* end dflg == NL_UDir branch */

    else /* when dflg == NL_VDir branch */
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ ) 
            {
                for ( jj = 0; jj < kby; jj++ )
                  { N_CopyCPt( Rw[ii][jj], &Qw[ii][jj] ); }
            }
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ ) 
            {
                for ( jj = 1; jj <= kby; jj++ )
                  { N_CopyCPt( Rw[ii][kby + jj], &Qw[ii][mb + jj] ); }
            }
        }
    } /* end dflg == NL_VDir branch */

    /* Remove unnecessary knots */
    if( dflg EQ NL_UDIR )
    {
        if( rem GT 0 )
        {
            /* see if added reflection Param knots can be removed - they should only have been needed temporarily */
            error = N_SrfRemoveKnotConditional(surQ,           /* in : target surface */
                                               uuu,            /* in : knot to be removed */
                                               rem,            /* in : number of times to be removed */
                                               100.0 *NL_MTOL, /* in : tol to check removability */
                                               NL_UDIR,        /* in : NL_UDIR: Remove in u-direction */
                                                               /*      NL_VDIR: Remove in v-direction */
                                               &ii,            /* out: number of knots actually removed */
                                               surQ,           /* out: Surface after knot removal */
                                               SQ );           /* in : surQ's stack */

            if( error EQ NL_YES )
              { NL_OUT; }
        }

        if( end EQ NL_START )
          { uuu = UP[0]; }
        else
          { uuu = UP[r]; }

        if( cont EQ NL_G1 )
          { rem = 1; }

        else if( cont EQ NL_G2 )
          { rem = 2; }

        else
          { rem = p; }

        /* see if any knots at the reflection boundary can be removed */
        error = N_SrfRemoveKnotConditional(surQ,    /* in : target surface */
                                           uuu,     /* in : knot to be removed */
                                           rem,     /* in : number of times to be removed */
                                           NL_MTOL, /* in : tol to check removability */
                                           NL_UDIR, /* in : NL_UDIR: Remove in u-direction */
                                                    /*      NL_VDIR: Remove in v-direction */
                                           &ii,     /* out: number of knots actually removed */
                                           surQ,    /* out: Surface after knot removal */
                                           SQ );    /* in : surQ's stack */

        if( error EQ NL_YES )
          { NL_OUT; }

    } /* end NL_UDir branch */
    else /* NL_VDir branch */
    {
        if( rem GT 0 )
        {   
            /* see if added reflection Param knots can be removed - they should only have been needed temporarily */
            error = N_SrfRemoveKnotConditional(surQ,           /* in : target surface */
                                               vvv,            /* in : knot to be removed */
                                               rem,            /* in : number of times to be removed */
                                               100.0 *NL_MTOL, /* in : tol to check removability */
                                               NL_VDIR,        /* in : NL_UDIR: Remove in u-direction */
                                               &ii,            /*      NL_VDIR: Remove in v-direction */
                                               surQ,           /* out: number of knots actually removed */
                                               SQ );           /* out: Surface after knot removal */
                                                               /* in : surQ's stack */
            if( error EQ NL_YES )
              { NL_OUT; }
        }

        if( end EQ NL_START )
          { vvv = VP[0]; }
        else
          { vvv = VP[s]; }

        if( cont EQ NL_G1 )
          { rem = 1; }

        else if( cont EQ NL_G2 )
          { rem = 2; }

        else
          { rem = q; }

        /* see if any knots at the reflection boundary can be removed */
        error = N_SrfRemoveKnotConditional(surQ,    /* in : target surface */
                                           vvv,     /* in : knot to be removed */
                                           rem,     /* in : number of times to be removed */
                                           NL_MTOL, /* in : tol to check removability */
                                           NL_VDIR, /* in : NL_UDIR: Remove in u-direction */
                                                    /*      NL_VDIR: Remove in v-direction */
                                           &ii,     /* out: number of knots actually removed */
                                           surQ,    /* out: Surface after knot removal */
                                           SQ );    /* in : surQ's stack */

        if( error EQ NL_YES )
          { NL_OUT; }
    } /* end NL_VDir branch */

    /* If extension is in place, kill old surface */

    if( surP EQ surQ )
      { N_FreeSrf( &surA, SP ); }

    /* End NURBS and Exit */

    EXIT:
    N_EndNurbs( &SL );

    return (error);

} /* end ST_surxsdr */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This surface routine extends a Nurbs surface to a curve. That is,
     the given curve becomes the new boundary of  the  surface. Either 
     the start or end of either the u- or v-direction can be extended.  
     Continuity can  be  controlled via options. After extension,  the 
     original parameter domain still maps to the original surface (ie. 
     the parameter domain is also  extended.  For  best  results,  the
     parameterizations of the old and new boundary curves  should  not
     vary greatly from one another. A typical calling example is: 

       NL_SURFACE   surP, surQ;
       NL_CURVE     newbdy;
       NL_STACKS    SP, SQ;
       ...
       (define surP and newbdy);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfExtendToCrv(&surP,&newbdy,NL_UDIR,NL_START,NL_CMAX,&surQ,&SP,&SQ);
       N_SrfExtendToCrv(&surP,&newbdy,NL_VDIR,NL_END,NL_G1,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in surQ's  
     knot vector and polygon objects.


   ACCESS:
   
     surP   , input  ,  NURBS surface
     newbdy , input  ,  New boundary curve
     dflg   , input  ,  Flag:
                         NL_UDIR : The surface is extended across either the
                                u=umin or u=umax boundary
                         NL_VDIR : The surface is extended across either the  
                                v=vmin or v=vmax boundary
     end    , input  ,  Flag:
                         NL_START: The surface is extended across either the 
                                u=umin or v=vmin boundary (see dflg)
                         NL_END  : The surface is extended across either the 
                                u=umax or v=vmax boundary (see dflg)
     cont   , input  ,  Flag:
                         NL_G1  :  surP is extended quadratically (unless it 
                                is degree 1,  in which case NL_G1 is not at-
                                tained).  surQ is NL_G1 where the  extension 
                                joins the original surface
                         NL_CMAX:  the extension yields  (C)  continuity  of 
                                order degree-1 (a single knot at the ori-
                                ginal boundary)
     surQ , output ,  Surface after extension
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfExtendToCrv( NL_SURFACE *surP, NL_CURVE *newbdy, NL_FLAG dflg, NL_FLAG end, NL_FLAG cont, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfExtendToCrv");

    NL_FLAG error = NL_NO;

    NL_CURVE curA, curB, curC;

    NL_SURFACE surA, surB;

    /*        NL_INTEGER     **bin;            */

    NL_INDEX ii, jj, kk, k1, n, m, r, s, nq, mq, rq, sq, mult = 0, rem, nn, kp;

    NL_DEGREE p, q, pq;

    NL_CPOINT ** Pw, ** Qw, ** Rw, *Dw, *Cw;

    NL_POINT P1;

    NL_BOOLEAN rat;

    NL_REAL *UP, *VP, *UQ, *VQ, du, dv, u, v, uu, vv, uuu = 0.0, vvv = 0.0, d1, d2, d3, d4, tal, w0, w1, dr, dl, w;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make copies of the curve and surface */

    N_CrvInitArrays( &curA );
    error = N_CrvCopy( newbdy, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    N_SrfInitArrays( &surA );
    error = N_SrfCopy( surP, &surA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Ensure that the curve and surface directions are compatible */

    N_SrfGetParameterBounds( &surA, &u, &uu, &v, &vv );

    if( dflg EQ NL_UDIR )
        error = N_CrvMakeCompatibleWithSrf( &curA, &surA, NL_VDIR, v, vv, &SL );
    else
        error = N_CrvMakeCompatibleWithSrf( &curA, &surA, NL_UDIR, u, uu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( &surA, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_CrvGetCPts( &curA, &ii, &Cw );

    /* Determine du or dv parameter extension value.      */
    /* Do this by averaging results for curve extensions. */

    k1 = 5; /* this many curves will be used to compute the average */

    if( dflg EQ NL_UDIR )
    {
        N_CrvInitArrays( &curB );
        error = N_CrvSizeArrays( &curB, n, p, r, rname, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        du = 0.0;

        dv = (VP[s] - VP[0]) / ((NL_REAL)k1 - 1.0);
        kk = 0;
        du = 0.0;

        for ( ii = 0; ii < k1; ii++ )
        {
            if( ii EQ k1 - 1 )
                v = VP[s];
            else
                v = VP[0] + ii * dv;

            error = N_SrfExtractIsoCrv( &surA, v, NL_UDIR, &curB, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &curC );
            error = N_CrvEval( &curA, v, NL_LEFT, &P1 );
            error = N_CrvExtendToPt( &curB, P1, end, cont, &curC, &SL, &SL );

            if( error EQ NL_NO )
            {
                kk += 1;
                N_CrvGetParamBounds( &curC, &d1, &d2 );

                if( end EQ NL_START )
                    du += (UP[0] - d1);
                else
                    du += (d2 - UP[r]);
                N_FreeCrv( &curC, &SL );
            }
        }

        if( kk EQ 0 )
            NL_ERROR( NL_NUM_ERR );
        du /= kk;
    }
    else
    {
        N_CrvInitArrays( &curB );
        error = N_CrvSizeArrays( &curB, m, q, s, rname, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        du = (UP[r] - UP[0]) / ((NL_REAL)k1 - 1.0);
        kk = 0;
        dv = 0.0;

        for ( ii = 0; ii < k1; ii++ )
        {
            if( ii EQ k1 - 1 )
                u = UP[r];
            else
                u = UP[0] + ii * du;

            error = N_SrfExtractIsoCrv( &surA, u, NL_VDIR, &curB, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &curC );
            error = N_CrvEval( &curA, u, NL_LEFT, &P1 );
            error = N_CrvExtendToPt( &curB, P1, end, cont, &curC, &SL, &SL );

            if( error EQ NL_NO )
            {
                kk += 1;
                N_CrvGetParamBounds( &curC, &d1, &d2 );

                if( end EQ NL_START )
                    dv += (VP[0] - d1);
                else
                    dv += (d2 - VP[s]);
                N_FreeCrv( &curC, &SL );
            }
        }

        if( kk EQ 0 )
            NL_ERROR( NL_NUM_ERR );
        dv /= kk;
    }

    /* Determine nq,mq,rq,sq values, and allocate working memory. */
    /* If necessary, insert interior knot in NL_CMAX case.           */

    if( dflg EQ NL_UDIR )
    {
        if( cont EQ NL_CMAX )
        {
            mult = 1;

            if( end EQ NL_START )
            {
                while( p + mult + 1 LE r )
                    if( UP[p + mult + 1]EQ UP[p + 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT p )
                    mult = p;

                if( mult LT p )
                {
                    uuu = UP[p + 1];
                    error = N_SrfInsertKnot( &surA, uuu, p - mult, NL_UDIR, &surA, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
            else
            {
                while( r - p - mult - 1 GE 0 )
                    if( UP[r - p - mult - 1]EQ UP[r - p - 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT p )
                    mult = p;

                if( mult LT p )
                {
                    uuu = UP[r - p - 1];
                    error = N_SrfInsertKnot( &surA, uuu, p - mult, NL_UDIR, &surA, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            N_SrfGetCPtsDegreesAndKnots( &surA, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
        }

        mq = m;
        sq = s;
        nq = n + p;
        rq = r + p;

        Rw = N_AllocCPt2dArray( mq, 2 * p, &SL );

        if( Rw EQ NULL )
            NL_QUIT;
    }
    else
    {
        if( cont EQ NL_CMAX )
        {
            mult = 1;

            if( end EQ NL_START )
            {
                while( q + mult + 1 LE s )
                    if( VP[q + mult + 1]EQ VP[q + 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT q )
                    mult = q;

                if( mult LT q )
                {
                    vvv = VP[q + 1];
                    error = N_SrfInsertKnot( &surA, vvv, q - mult, NL_VDIR, &surA, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
            else
            {
                while( s - q - mult - 1 GE 0 )
                    if( VP[s - q - mult - 1]EQ VP[s - q - 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT q )
                    mult = q;

                if( mult LT q )
                {
                    vvv = VP[s - q - 1];
                    error = N_SrfInsertKnot( &surA, vvv, q - mult, NL_VDIR, &surA, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            N_SrfGetCPtsDegreesAndKnots( &surA, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
        }

        nq = n;
        rq = r;
        mq = m + q;
        sq = s + q;

        Rw = N_AllocCPt2dArray( nq, 2 * q, &SL );

        if( Rw EQ NULL )
            NL_QUIT;
    }

    /* See if memory is needed for surQ */

    surB = *surP;

    if( surP EQ surQ )
    {
        error = N_AllocSrfArrays( surP, nq, mq, p, q, rq, sq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, nq, mq, p, q, rq, sq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Load UQ and VQ, partially load Qw, and        */
    /* load Rw with the required ctrl pts from surA  */

    if( dflg EQ NL_UDIR )
    {
        for ( ii = 0; ii <= s; ii++ )
            VQ[ii] = VP[ii];

        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= mq; ii++ )
            {
                for ( jj = 0; jj <= n; jj++ )
                    N_CopyCPt( Pw[jj][ii], &Qw[p + jj][ii] );
                N_CopyCPt( Cw[ii], &Qw[0][ii] );

                if( cont EQ NL_G1 AND p GT 1 )
                    N_CopyCPt( Cw[ii], &Rw[ii][p - 2] );

                for ( jj = 0; jj <= p; jj++ )
                    N_CopyCPt( Pw[jj][ii], &Rw[ii][jj + p] );
            }

            for ( ii = 0; ii <= p; ii++ )
                UQ[ii] = UP[0] - du;

            for ( ii = 1; ii <= r; ii++ )
                UQ[p + ii] = UP[ii];
        }
        else
        {
            kk = n - p;

            for ( ii = 0; ii <= mq; ii++ )
            {
                for ( jj = 0; jj <= n; jj++ )
                    N_CopyCPt( Pw[jj][ii], &Qw[jj][ii] );
                N_CopyCPt( Cw[ii], &Qw[nq][ii] );

                if( cont EQ NL_G1 AND p GT 1 )
                    N_CopyCPt( Cw[ii], &Rw[ii][p + 2] );

                for ( jj = 0; jj <= p; jj++ )
                    N_CopyCPt( Pw[jj + kk][ii], &Rw[ii][jj] );
            }

            for ( ii = 0; ii < r; ii++ )
                UQ[ii] = UP[ii];

            for ( ii = 0; ii <= p; ii++ )
                UQ[r + ii] = UP[r] + du;
        }
    }
    else
    {
        for ( ii = 0; ii <= r; ii++ )
            UQ[ii] = UP[ii];

        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nq; ii++ )
            {
                for ( jj = 0; jj <= m; jj++ )
                    N_CopyCPt( Pw[ii][jj], &Qw[ii][q + jj] );
                N_CopyCPt( Cw[ii], &Qw[ii][0] );

                if( cont EQ NL_G1 AND q GT 1 )
                    N_CopyCPt( Cw[ii], &Rw[ii][q - 2] );

                for ( jj = 0; jj <= q; jj++ )
                    N_CopyCPt( Pw[ii][jj], &Rw[ii][jj + q] );
            }

            for ( ii = 0; ii <= q; ii++ )
                VQ[ii] = VP[0] - dv;

            for ( ii = 1; ii <= s; ii++ )
                VQ[q + ii] = VP[ii];
        }
        else
        {
            kk = m - q;

            for ( ii = 0; ii <= nq; ii++ )
            {
                for ( jj = 0; jj <= m; jj++ )
                    N_CopyCPt( Pw[ii][jj], &Qw[ii][jj] );
                N_CopyCPt( Cw[ii], &Qw[ii][mq] );

                if( cont EQ NL_G1 AND q GT 1 )
                    N_CopyCPt( Cw[ii], &Rw[ii][q + 2] );

                for ( jj = 0; jj <= q; jj++ )
                    N_CopyCPt( Pw[ii][jj + kk], &Rw[ii][jj] );
            }

            for ( ii = 0; ii < s; ii++ )
                VQ[ii] = VP[ii];

            for ( ii = 0; ii <= q; ii++ )
                VQ[s + ii] = VP[s] + dv;
        }
    }

    /* Compute the new control points defining the extension. */
    /* Use method by Shetty & White, CAD, Sept. 1991 for NL_G1.  */
    /* Use derivative expressions for NL_CMAX.                   */

    rat = N_IsSrfRat( &surA );

    if( dflg EQ NL_UDIR )
    {
        nn = mq;
        pq = p;
    }
    else
    {
        nn = nq;
        pq = q;
    }

    if( cont EQ NL_G1 AND pq GT 1 )
    {
        if( end EQ NL_START )
        {
            if( rat ) /* compute tal (Shetty & White) */
            {
                if( dflg EQ NL_UDIR )
                    d1 = 2.0 *p / (UP[p + 1] - UP[0]);
                else
                    d1 = 2.0 *q / (VP[q + 1] - VP[0]);
                tal = -1.0e+20;

                for ( ii = 0; ii <= nn; ii++ )
                {
                    N_CPtGetW( Rw[ii][pq], &w0 );
                    N_CPtGetW( Rw[ii][pq + 1], &w1 );
                    d2 = d1 * (w1 - w0) / w0;

                    if( d2 GT tal )
                        tal = d2;
                }
            }
            else
                tal = 0.0;

            if( dflg EQ NL_UDIR ) /* initial extension is quadratic */
            {
                d1 = 1.0 + tal * du / 2.0;
                d2 = (pq * du) / (2.0 *( UP[p + 1] - UP[0] ));
            }
            else
            {
                d1 = 1.0 + tal * dv / 2.0;
                d2 = (pq * dv) / (2.0 *( VP[q + 1] - VP[0] ));
            }
            d3 = d1 + d2;

            for ( ii = 0; ii <= nn; ii++ ) /* recompute first row for NL_G1 */
            {
                N_Combine2CPts( d3,                 /* alpha of Cw = alpha * Aw + beta * Bw */
                                Rw[ii][pq],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                -d2,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                Rw[ii][pq + 1],     /* Bw    of Cw = alpha * Aw + beta * Bw */
                                &Rw[ii][pq - 1] );  /* Cw    of Cw = alpha * Aw + beta * Bw */
            }

            if( pq GT 2 ) /* now degree elevate */
            {             /* rmb  correction based on N_CrvExtendToPt  9/16/05 */
                for ( ii = 3; ii <= pq; ii++ )
                {
                    k1 = pq - ii;
                    d1 = (NL_REAL)ii;

                    for ( kk = 0; kk <= nn; kk++ )
                        N_CopyCPt( Rw[kk][k1 + 1], &Rw[kk][k1] );

                    for ( jj = 1; jj < ii; jj++ )
                    {
                        d2 = (NL_REAL)jj / d1;

                        for ( kk = 0; kk <= nn; kk++ )
                            N_Combine2CPts( 1.0 - d2,             /* alpha of Cw = alpha * Aw + beta * Bw */
                                            Rw[kk][k1 + jj + 1],  /* Aw    of Cw = alpha * Aw + beta * Bw */
                                            d2,                   /* beta  of Cw = alpha * Aw + beta * Bw */
                                            Rw[kk][k1 + jj],      /* Bw    of Cw = alpha * Aw + beta * Bw */
                                            &Rw[kk][k1 + jj] );   /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
                }
            }
        }
        else          /* NL_UDIR extension is at end */
        {
            if( rat ) /* compute tal (Shetty & White) */
            {
                if( dflg EQ NL_UDIR )
                    d1 = 2.0 *p / (UP[r] - UP[r - p - 1]);
                else
                    d1 = 2.0 *q / (VP[s] - VP[s - q - 1]);
                tal = -1.0e+20;

                for ( ii = 0; ii <= nn; ii++ )
                {
                    N_CPtGetW( Rw[ii][pq], &w0 );
                    N_CPtGetW( Rw[ii][pq - 1], &w1 );
                    d2 = d1 * (w1 - w0) / w0;

                    if( d2 GT tal )
                        tal = d2;
                }
            }
            else
                tal = 0.0;

            if( dflg EQ NL_UDIR ) /* initial extension is quadratic */
            {
                d1 = 1.0 + tal * du / 2.0;
                d2 = (pq * du) / (2.0 *( UP[r] - UP[r - p - 1] ));
            }
            else
            {
                d1 = 1.0 + tal * dv / 2.0;
                d2 = (pq * dv) / (2.0 *( VP[s] - VP[s - q - 1] ));
            }
            d3 = d1 + d2;

            for ( ii = 0; ii <= nn; ii++ ) /* recompute first row for NL_G1 */
            {
                N_Combine2CPts( d3,                 /* alpha of Cw = alpha * Aw + beta * Bw */
                                Rw[ii][pq],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                -d2,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                Rw[ii][pq - 1],     /* Bw    of Cw = alpha * Aw + beta * Bw */
                                &Rw[ii][pq + 1] );  /* Cw    of Cw = alpha * Aw + beta * Bw */
            }

            if( pq GT 2 ) /* now degree elevate */
            {             /* rmb correction based on N_CrvExtendToPt 9/16/05  */
                for ( ii = 3; ii <= pq; ii++ )
                {
                    k1 = pq + ii;
                    d1 = (NL_REAL)ii;

                    for ( kk = 0; kk <= nn; kk++ )
                        N_CopyCPt( Rw[kk][k1 - 1], &Rw[kk][k1] ); /* Rw[kk][k1] = Rw[kk][k1-1];*/

                    for ( jj = k1 - 1; jj > pq; jj-- )
                    {
                        d2 = ((NL_REAL)jj - (NL_REAL)pq) / d1;

                        for ( kk = 0; kk <= nn; kk++ )
                            N_Combine2CPts( d2,               /* alpha of Cw = alpha * Aw + beta * Bw */
                                            Rw[kk][jj - 1],   /* Aw    of Cw = alpha * Aw + beta * Bw */
                                            1.0 - d2,         /* beta  of Cw = alpha * Aw + beta * Bw */
                                            Rw[kk][jj],       /* Bw    of Cw = alpha * Aw + beta * Bw */
                                            &Rw[kk][jj] );    /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
                }
            }
        }
    }

    if( cont EQ NL_CMAX AND pq GT 1 )
    {
        kk = pq;
        /*          bin = N_AllocInt2dArray(kk,kk,&SL);  */
        /*          if ( bin EQ NULL )  NL_QUIT;   */
        /*          N_PascalTriRow(bin,kk);           */

        Dw = N_AllocCPt1dArray( pq, &SL );

        if( Dw EQ NULL )
            NL_QUIT;

        if( end EQ NL_START )
        {
            if( dflg EQ NL_UDIR )
            {
                d2 = du;
                d3 = UP[p + 1] - UP[0];
            }
            else
            {
                d2 = dv;
                d3 = VP[q + 1] - VP[0];
            }
            d4 = 1.0 / d3;

            for ( ii = 0; ii <= nn; ii++ )
            {
                dr = pq * d4;

                for ( jj = 1; jj < pq; jj++ ) /* this loop computes derivatives */
                {                             /* from the right                 */
                    N_CopyCPt( Rw[ii][pq + jj], &Dw[jj] );

                    if( jj % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < jj; kk++ ) /* compute the jj-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], /* in : alpha of Bw = Bw + alpha*Aw */
                                          Rw[ii][pq + kk],           /* in : Aw    of Bw = Bw + alpha*Aw */
                                          &Dw[jj] );                 /* out: Bw    of Bw = Bw + alpha*Aw */
                        kp *= -1;
                    }
                    N_ScaleCPt( dr, Dw[jj], &Dw[jj] );
                    dr = dr * ((NL_REAL)pq - (NL_REAL)jj) * d4;
                }

                dl = d2 / pq;

                for ( jj = 1; jj < pq; jj++ ) /* this loop computes */
                {                             /* new control points */
                    if( jj % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw[jj], &Rw[ii][pq - jj] );
                    N_ScaleCPt( -kp * dl, Rw[ii][pq - jj], &Rw[ii][pq - jj] );

                    if( jj LT pq )
                        dl = dl * d2 / ((NL_REAL)pq - (NL_REAL)jj);

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (p-jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], /* in : alpha of Bw = Bw + alpha*Aw */
                                          Rw[ii][pq - kk],           /* in : Aw    of Bw = Bw + alpha*Aw */
                                          &Rw[ii][pq - jj] );        /* out: Bw    of Bw = Bw + alpha*Aw */
                        kp *= -1;
                    }
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj < pq; jj++ )
                    {
                        N_CPtGetW( Rw[ii][jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }
                }
            }
        }
        else
        {
            if( dflg EQ NL_UDIR )
            {
                d2 = du;
                d3 = UP[r] - UP[r - p - 1];
            }
            else
            {
                d2 = dv;
                d3 = VP[s] - VP[s - q - 1];
            }
            d4 = 1.0 / d3;

            for ( ii = 0; ii <= nn; ii++ )
            {
                dl = -pq * d4;

                for ( jj = 1; jj < pq; jj++ ) /* this loop computes derivatives */
                {                             /* from the left                  */
                    N_CopyCPt( Rw[ii][pq - jj], &Dw[jj] );

                    if( jj % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < jj; kk++ ) /* compute the jj-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], Rw[ii][pq - kk], &Dw[jj] );
                        kp *= -1;
                    }
                    N_ScaleCPt( dl, Dw[jj], &Dw[jj] );
                    dl = -dl * ((NL_REAL)pq - (NL_REAL)jj) * d4;
                }

                dr = d2 / pq;

                for ( jj = 1; jj < pq; jj++ ) /* this loop computes */
                {                             /* new control points */
                    if( jj % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw[jj], &Rw[ii][pq + jj] );
                    N_ScaleCPt( dr, Rw[ii][pq + jj], &Rw[ii][pq + jj] );

                    if( jj LT pq )
                        dr = dr * d2 / ((NL_REAL)pq - (NL_REAL)jj);

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (kby+jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], Rw[ii][pq + kk], &Rw[ii][pq + jj] );
                        kp *= -1;
                    }
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj < pq; jj++ )
                    {
                        N_CPtGetW( Rw[ii][pq + jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }
                }
            }
        }
    }

    /* Now load the control points from Rw into Qw */

    if( dflg EQ NL_UDIR )
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 1; jj < pq; jj++ )
                    N_CopyCPt( Rw[ii][jj], &Qw[jj][ii] );
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 1; jj < pq; jj++ )
                    N_CopyCPt( Rw[ii][pq + jj], &Qw[n + jj][ii] );
        }
    }
    else
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 1; jj < pq; jj++ )
                    N_CopyCPt( Rw[ii][jj], &Qw[ii][jj] );
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 1; jj < pq; jj++ )
                    N_CopyCPt( Rw[ii][pq + jj], &Qw[ii][m + jj] );
        }
    }

    /* Remove unnecessary knots */

    if( dflg EQ NL_UDIR )
    {
        if( cont EQ NL_CMAX AND p GT mult )
        {
            error = N_SrfRemoveKnotConditional( surQ, uuu, p - mult, 100.0 *NL_MTOL, NL_UDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( end EQ NL_START )
            uuu = UP[0];
        else
            uuu = UP[r];

        if( cont EQ NL_G1 )
            rem = 1;
        else
            rem = p - 1;

        if( rem GT 0 )
        {
            error = N_SrfRemoveKnotConditional( surQ, uuu, rem, NL_MTOL, NL_UDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }
    else
    {
        if( cont EQ NL_CMAX AND q GT mult )
        {
            error = N_SrfRemoveKnotConditional( surQ, vvv, q - mult, 100.0 *NL_MTOL, NL_VDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( end EQ NL_START )
            vvv = VP[0];
        else
            vvv = VP[s];

        if( cont EQ NL_G1 )
            rem = 1;
        else
            rem = q - 1;

        if( rem GT 0 )
        {
            error = N_SrfRemoveKnotConditional( surQ, vvv, rem, NL_MTOL, NL_VDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* If extension is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surB, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfExtendToCrv */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes  the  derivative of a  NURBS  surface  with 
     respect to a knot. A typical calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_INDEX      k;
       NL_POINT      Sk;
       ...
       (define sur, get u, v and k);
       ...
       N_SrfDerivKnot(&sur,k,u,v,NL_UDIR,NL_LEFT,NL_RIGHT,NL_LEFT,&Sk);


   ACCESS:
   
     sur , input  ,  NURBS surface
     k   , input  ,  Index of knot, i.e. the derivative  with respect to
                     t_k is computed (t is either u or v)
     u,v , input  ,  Parameter values 
     dir , input  ,  NL_FLAG:
                       NL_UDIR: u-derivative required
                       NL_VDIR: v-derivative required
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              t_(k) != t_(k-1)
                       NL_RIGHT: right  derivative. NL_INDEX  k  MUST  SATISFY
                              t_(k) != t_(k+1)
                              (t is either u or v)
     ulp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     vlp , input  ,  Flag:
                       NL_LEFT : v is in [v[j],v[j+1])
                       NL_RIGHT: v is in (v[j],v[j+1]]
     Sk  , output ,  Derivative computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfDerivKnot( NL_SURFACE *sur, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG dir, NL_FLAG flk, NL_FLAG ulp, NL_FLAG vlp, NL_POINT *Sk )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, kk, spn;

    NL_DEGREE p, q;

    NL_REAL ** Bk, *U, *V;

    NL_KNOTVECTOR *knu, *knv;

    NL_ENET ntl;

    NL_POINT ** P;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnotVectors( sur, &knu, &knv );
    N_SrfGetKnots( sur, &i, &j, &U, &V );

    /* Compute basis function derivatives */

    Bk = N_AllocReal2dArray( p + 1, q + 1, &SL );

    if( Bk EQ NULL )
        NL_QUIT;

    if( N_IsSrfRat( sur ) )
    {
        error = N_SrfRatBasisKnotDeriv( sur, k, u, v, dir, flk, ulp, vlp, Bk, &spn );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_BiBasisKnotDeriv( knu, knv, p, q, k, u, v, dir, flk, ulp, vlp, Bk, &spn );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute derivatives */

    kk = k;
    N_CopyPt( NL_ZERO, Sk );

    if( dir EQ NL_UDIR )
    {
        if( N_IsSrfRat( sur ) )
        {
            if( flk EQ NL_LEFT AND u GT U[k] )
                while( U[kk]EQ U[kk + 1] )
                    kk++;

            if( flk EQ NL_RIGHT AND u LT U[k] )
                while( U[kk]EQ U[kk - 1] )
                    kk--;
        }

        error = N_SrfGetENet( sur, kk - p - 1, kk, spn - q, spn, &ntl, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_ENetGetPts( &ntl, &i, &j, &P );

        for ( i = 0; i <= p + 1; i++ )
        {
            for ( j = 0; j <= q; j++ )
                N_VectorBlendPt( Bk[i][j], P[i][j], Sk );
        }
    }
    else if( dir EQ NL_VDIR )
    {
        if( N_IsSrfRat( sur ) )
        {
            if( flk EQ NL_LEFT AND v GT V[k] )
                while( V[kk]EQ V[kk + 1] )
                    kk++;

            if( flk EQ NL_RIGHT AND v LT V[k] )
                while( V[kk]EQ V[kk - 1] )
                    kk--;
        }

        error = N_SrfGetENet( sur, spn - p, spn, kk - q - 1, kk, &ntl, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_ENetGetPts( &ntl, &i, &j, &P );

        for ( i = 0; i <= p; i++ )
        {
            for ( j = 0; j <= q + 1; j++ )
                N_VectorBlendPt( Bk[i][j], P[i][j], Sk );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfDerivKnot */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This surface routine modifies a surface so that a given curve be-
     comes a new boundary of the surface.  That is,  the  old boundary
     curve is replaced with the new curve, and the surface is modified
     inward a small amount and transitions smoothly into the  original
     surface. Any of the four boundaries may be  modified.  Continuity 
     is controlled via options. The largest deviation between original 
     and modified surfaces is at the boundary. A typical calling exam-
     ple is:

       NL_SURFACE   surP, surQ;
       NL_CURVE     newbdy;
       NL_REAL      uv;
       NL_STACKS    SP, SQ;
       ...
       (define surP and newbdy, and choose uv);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfModifyBoundaryCrv(&surP,&newbdy,NL_UDIR,NL_START,uv,NL_C1,&surQ,&SP,&SQ);
       N_SrfModifyBoundaryCrv(&surP,&newbdy,NL_VDIR,NL_END,uv,NL_CMAX,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in surQ's  
     knot vector and polygon objects.


   ACCESS:
   
     surP   , input  ,  NURBS surface
     newbdy , input  ,  New boundary curve
     dirflg , input  ,  Flag:
                         NL_UDIR : newbdy replaces  either  the  u=umin or 
                                u=umax boundary, and uv is a u-value
                         NL_VDIR : newbdy replaces  either  the  v=vmin or 
                                v=vmax boundary, and uv is a v-value
     end    , input  ,  Flag:
                         NL_START: newbdy replaces either  the  u=umin  or 
                                v=vmin boundary (see dirflg above)
                         NL_END  : newbdy replaces either  the  u=umax  or 
                                v=vmax boundary (see dirflg above)
     uv     , input  ,  A parameter value (u or v) which  controls  how
                        far inward from the affected boundary the surf-
                        is modified (see dirflg above)
     cont   , input  ,  Flag:
                         NL_C1  : The modified segment will join the unmod-
                               ified segment with at least NL_C1 continuity
                               (assuming degree > 1  and  the continuity
                               there was originally at least NL_C1)
                         NL_C2  : The modified segment will join the unmod-
                               ified segment with at least NL_C2 continuity
                               (assuming degree > 2  and  the continuity
                               there was originally at least NL_C2)
                         NL_CMAX: The modified segment will join the unmod-
                               ified segment  with  degree-1  continuity
                               (assuming the continuity there was origi-
                               nally at least degree-1)

     surQ   , output ,  Surface after modification
     SP     , input  ,  surP's stack
     SQ     , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfModifyBoundaryCrv( NL_SURFACE *surP, NL_CURVE *newbdy, NL_FLAG dirflg, NL_FLAG end, NL_REAL uv, NL_FLAG cont, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfModifyBoundaryCrv");

    NL_FLAG error = NL_NO;

    NL_CURVE curC;

    NL_SURFACE surA, surB, surC, *surs[2];

    NL_INDEX ii, jj, kk, n, m, r, s, nc, mc;

    NL_DEGREE p, q, pq, pc;

    NL_CPOINT ** Pw, ** Qw, ** Bw, *Cw, Rw;

    NL_REAL *UP, *VP, *UQ, *VQ, *UB, *VB, *UC, u, v, uu, vv;

    NL_RECTANGLE R;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make copies of the curve and surface */

    N_CrvInitArrays( &curC );
    error = N_CrvCopy( newbdy, &curC, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    N_SrfInitArrays( &surC );
    error = N_SrfCopy( surP, &surC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Ensure that the curve and surface directions are compatible */

    N_SrfGetParameterBounds( &surC, &u, &uu, &v, &vv );

    if( dirflg EQ NL_UDIR )
        error = N_CrvMakeCompatibleWithSrf( &curC, &surC, NL_VDIR, v, vv, &SL );
    else
        error = N_CrvMakeCompatibleWithSrf( &curC, &surC, NL_UDIR, u, uu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get local notation and initialize surface object */

    N_SrfGetCPtsDegreesAndKnots( &surC, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_CrvGetCPtsDegreeAndKnots( &curC, &nc, &Cw, &pc, &mc, &UC );
    N_SrfInitArrays( &surB );

    /* Set uv */

    if( dirflg EQ NL_UDIR )
    {
        /* check that uv not too close to knot or equal to end knots */

        if( uv LT UP[0]OR uv GT UP[r] )
            NL_ERROR( NL_PAR_ERR );

        for ( ii = p + 1; ii <= r; ii++ )
            if( uv LE UP[ii] )
                break;

        if( uv - UP[ii - 1]LT KN_TOL * (UP[r] - UP[0]) )
            uv = UP[ii - 1];

        else if( UP[ii] - uv LT KN_TOL *( UP[r] - UP[0] ) )
            uv = UP[ii];

        if( uv EQ UP[0] )
            uv = UP[0] + KN_TOL * (UP[r] - UP[0]);

        if( uv EQ UP[r] )
            uv = UP[r] - KN_TOL * (UP[r] - UP[0]);
    }
    else
    {
        /* check that uv not too close to knot or equal to end knots */

        if( uv LT VP[0]OR uv GT VP[s] )
            NL_ERROR( NL_PAR_ERR );

        for ( ii = q + 1; ii <= s; ii++ )
            if( uv LE VP[ii] )
                break;

        if( uv - VP[ii - 1]LT KN_TOL * (VP[s] - VP[0]) )
            uv = VP[ii - 1];

        else if( VP[ii] - uv LT KN_TOL *( VP[s] - VP[0] ) )
            uv = VP[ii];

        if( uv EQ VP[0] )
            uv = VP[0] + KN_TOL * (VP[s] - VP[0]);

        if( uv EQ VP[s] )
            uv = VP[s] - KN_TOL * (VP[s] - VP[0]);
    }

    /* A new surface is formed which is zero to uv, and then transitions */
    /* to newbdy-curC (or analogous if end == NL_START). This surface is    */
    /* then added to the original.                                       */

    if( N_IsCrvRat( &curC ) )
        N_CPtFromWxWyWz( 0.0, 0.0, 0.0, 0.0, &Rw ); /* initialize zero ctrl pt */
    else
        N_CPtFromWxWyWz( 0.0, 0.0, 0.0, NL_NOW, &Rw );

    /* Form the surface to be added to surP */

    if( dirflg EQ NL_UDIR )
    {
        if( cont EQ NL_C1 )
            pq = 2; /* pq is degree of surB */

        else if( cont EQ NL_C2 )
            pq = 3;

        else
            pq = p;

        if( pq GT p )
            pq = p;

        error = N_SrfSizeArrays( &surB, pq + 1, m, pq, q, 2 * (pq + 1), s, rname, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        N_SrfGetCPtsAndKnots( &surB, &Bw, &UB, &VB );

        /* Load knots */

        for ( ii = 0; ii <= s; ii++ )
            VB[ii] = VP[ii];
        UB[pq + 1] = uv;

        for ( ii = 0; ii <= pq; ii++ )
        {
            UB[ii] = UP[0];
            UB[ii + pq + 2] = UP[r];
        }

        /* Load control points */

        kk = pq + 1;

        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= m; ii++ )
            {
                N_Diff2CPts( Cw[ii], Pw[0][ii], &Bw[0][ii] );

                for ( jj = 1; jj <= kk; jj++ )
                    N_CopyCPt( Rw, &Bw[jj][ii] );
            }
        }
        else
        {
            for ( ii = 0; ii <= m; ii++ )
            {
                N_Diff2CPts( Cw[ii], Pw[n][ii], &Bw[kk][ii] );

                for ( jj = 0; jj <= pq; jj++ )
                    N_CopyCPt( Rw, &Bw[jj][ii] );
            }
        }
    }
    else
    {
        if( cont EQ NL_C1 )
            pq = 2; /* pq is degree of surB */

        else if( cont EQ NL_C2 )
            pq = 3;

        else
            pq = q;

        if( pq GT q )
            pq = q;

        error = N_SrfSizeArrays( &surB, n, pq + 1, p, pq, r, 2 * (pq + 1), rname, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        N_SrfGetCPtsAndKnots( &surB, &Bw, &UB, &VB );

        /* Load knots */

        for ( ii = 0; ii <= r; ii++ )
            UB[ii] = UP[ii];
        VB[pq + 1] = uv;

        for ( ii = 0; ii <= pq; ii++ )
        {
            VB[ii] = VP[0];
            VB[ii + pq + 2] = VP[s];
        }

        /* Load control points */

        kk = pq + 1;

        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= n; ii++ )
            {
                N_Diff2CPts( Cw[ii], Pw[ii][0], &Bw[ii][0] );

                for ( jj = 1; jj <= kk; jj++ )
                    N_CopyCPt( Rw, &Bw[ii][jj] );
            }
        }
        else
        {
            for ( ii = 0; ii <= n; ii++ )
            {
                N_Diff2CPts( Cw[ii], Pw[ii][m], &Bw[ii][kk] );

                for ( jj = 0; jj <= pq; jj++ )
                    N_CopyCPt( Rw, &Bw[ii][jj] );
            }
        }
    }

    /* Make surC and surB compatible */

    surs[0] = &surC;
    surs[1] = &surB;

    error = N_MakeSrfsCompatibleUV( surs, 1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsDegreesAndKnots( &surC, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetCPtsDegreesAndKnots( &surB, &n, &m, &Bw, &p, &q, &r, &s, &UB, &VB );

    /* See if memory is needed for surQ */

    surA = *surP;

    if( surP EQ surQ )
    {
        error = N_AllocSrfArrays( surP, n, m, p, q, r, s, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Add the two surfaces together */

    for ( ii = 0; ii <= n; ii++ )
        for ( jj = 0; jj <= m; jj++ )
            N_Sum2CPts( Pw[ii][jj], Bw[ii][jj], &Qw[ii][jj] );

    /* Load the knots */

    for ( ii = 0; ii <= r; ii++ )
        UQ[ii] = UP[ii];

    for ( ii = 0; ii <= s; ii++ )
        VQ[ii] = VP[ii];

    /* Reparameterize back to the original parm rectangle if necessary */

    if( u NEQ UQ[0]OR uu NEQ UQ[r]OR v NEQ VQ[0]OR vv NEQ VQ[s] )
    {
        N_CreateRectangle( &R, u, uu, v, vv );
        N_SrfReparamToInterval( surQ, R, NL_UVDIR );
    }

    /* If extension is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfModifyBoundaryCrv */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a NURBS surface  by evaluating 
     all   non-vanishing  basis  functions  and  their  derivatives, and 
     multiplying  them  by  appropriate  control  points.  Discontinuous 
     surfaces  can also  be  handled by  passing  NL_LEFT/NL_RIGHT flags. This 
     version computes homogeneous derivatives. A typical calling example 
     is:

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_INDEX      udr, vdr;
       NL_CPOINT     **SDw;
       ...
       (define sur, get u, v, udr, vdr, and allocate memory for SDw);
       ...
       N_SrfEvalPtDerivs(&sur,u,v,NL_LEFT,NL_RIGHT,NL_TRUE,udr,vdr,SDw);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     mfl     , input  ,  Flag: 
                           NL_TRUE : compute  upper  half  only  of the 
                                  derivative matrix
                           NL_FALSE: compute full derivative matrix
                         (APPLICABLE ONLY IF udr=vdr!)
     udr,vdr , input  ,  Highest derivatives required
     SDw     , output ,  Derivatives;   SDw[k][l]  is    the   (k,l)-th 
                         derivative.  MEMORY  FOR SDw MUST BE ALLOCATED 
                         IN THE CALLING ROUTINE TO HOLD SDw[udr][vdr].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtDerivs( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_CPOINT ** SDw )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtDerivs");

    NL_FLAG jump, error = NL_NO;

    NL_INDEX i, j, k, l, usp, vsp;

    NL_DEGREE p, q;

    NL_REAL **** BD;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetCPts( sur, &i, &j, &Pw );
    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute basis function derivatives */

    BD = N_AllocReal4dArray( udr, vdr, p, q, &S );

    if( BD EQ NULL )
        NL_QUIT;

    error = N_SrfNonRatBasisDerivs( sur, u, v, ufl, vfl, mfl, udr, vdr, BD, &usp, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives */

    if( mfl EQ NL_TRUE AND udr EQ vdr )
        jump = 1;
    else
        jump = 2;

    switch( jump )
    {
        case 1: /* Compute upper half of derivative matrix */
            for ( k = 0; k <= udr; k++ )
            {
                for ( l = 0; l <= udr - k; l++ )
                {
                    N_CopyCPt( NL_CZERO, &SDw[k][l] );

                    for ( i = 0; i <= p; i++ )
                    {
                        for ( j = 0; j <= q; j++ )
                        {
                            N_VectorBlendCPt( BD[k][l][i][j], Pw[usp - p + i][vsp - q + j], &SDw[k][l] );
                        }
                    }
                }
            }
            break;

        case 2: /* Compute full derivative matrix */
            for ( k = 0; k <= udr; k++ )
            {
                for ( l = 0; l <= vdr; l++ )
                {
                    N_CopyCPt( NL_CZERO, &SDw[k][l] );

                    for ( i = 0; i <= p; i++ )
                    {
                        for ( j = 0; j <= q; j++ )
                        {
                            N_VectorBlendCPt( BD[k][l][i][j], Pw[usp - p + i][vsp - q + j], &SDw[k][l] );
                        }
                    }
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfEvalPtDerivs */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes an upper bound on the second derivative of a 
     surface. The  approach is based on  sampling the Bezier patches at
     the nodes. A typical calling example is:

       NL_SURFACE  sur;
       NL_REAL     Muu, Muv, Mvv; 
       ...
       (define sur);
       ...
       N_SrfMaxSecondDeriv(&sur,&Muu,&Muv,&Mvv);

     THIS ROUTINE PROVIDES A REASONABLE APPROXIMATION TO THE NL_MAXIMUM OF
     THE SECOND NL_DERIVATIVE. FOR A PRECISE ALTHOUGH SIGNIFICANTLY SLOWER
     APPROACH SEE N_SrfMax2ndDeriv.


   ACCESS:
   
     sur , input  ,  NURBS surface
     Muu , output ,  Maximum 2nd derivative magnitude in u- direction
     Muv , output ,  Maximum 2nd derivative magnitude in uv-direction
     Mvv , output ,  Maximum 2nd derivative magnitude in v- direction


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfMaxSecondDeriv( NL_SURFACE *sur, NL_REAL *Muu, NL_REAL *Muv, NL_REAL *Mvv )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INDEX i, j, k, l, r, s, a, b, al, bl;

    NL_REAL *U, *V, *u, *v, ui, vi, duu, duv, dvv, muu, muv, mvv;

    NL_POINT ** SD;

    NL_SURFACE *** bez;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and decompose into Bezier patches */

    N_SrfGetDegrees( sur, &p, &q );

    error = N_SrfDecomposeToBez( sur, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( p, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( q, &SL );

    if( v EQ NULL )
        NL_QUIT;

    duu = duv = dvv = 0.0;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            al = 0;
        else
            al = 1;

        for ( j = 0; j <= l; j++ )
        {
            if( j EQ 0 )
                bl = 0;
            else
                bl = 1;

            N_SrfGetKnots( bez[i][j], &r, &s, &U, &V );

            ui = (U[r] - U[0]) / p;
            vi = (V[s] - V[0]) / q;

            u[0] = U[0];
            u[p] = U[r];
            v[0] = V[0];
            v[q] = V[s];

            for ( a = 1; a <= p - 1; a++ )
                u[a] = U[0] + a * ui;

            for ( b = 1; b <= q - 1; b++ )
                v[b] = V[0] + b * vi;

            for ( a = al; a <= p; a++ )
            {
                for ( b = bl; b <= q; b++ )
                {
                    error = N_SrfDerivs( bez[i][j], u[a], v[b], NL_LEFT, NL_LEFT, NL_TRUE, 2, 2, SD );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_PtMagnitude( SD[2][0], &muu );
                    N_PtMagnitude( SD[1][1], &muv );
                    N_PtMagnitude( SD[0][2], &mvv );

                    if( muu GT duu )
                        duu = muu;

                    if( muv GT duv )
                        duv = muv;

                    if( mvv GT dvv )
                        dvv = mvv;
                }
            }
        }
    }

    *Muu = duu;
    *Muv = duv;
    *Mvv = dvv;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfMaxSecondDeriv */



/*******************************************************************//**


   DESCRIPTION:

     This routine computes the first and second derivatives of the UNIT
     surface normal at  given parameter  values. Discontinuous surfaces 
     can be  handled by  passing  NL_LEFT/NL_RIGHT  flags. A typical  calling 
     example:

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_POINT      **ND;
       ...
       (define sur, get u, v, and allocate memory for ND);
       ...
       N_SrfEvalPtNormalDeriv(&sur,u,v,NL_LEFT,NL_LEFT,ND);

     MEMORY FOR ND MUST BE ALLOCATED IN THE CALLING  ROUTINE TO HOLD UP
     TO ND[2][2]! 


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) (NL_RIGHT deriva-
                                  tives are used)
                           NL_RIGHT: t is in (t[j],t[j+1]] (NL_LEFT  deriva-
                                  tives used)
                           (t is either u or v)
     ND      , output ,  Derivatives of the unit normal:
                           ND[1][0]: N_u(u,v)
                           ND[0][1]: N_v(u,v)
                           ND[2][0]: N_uu(u,v)
                           ND[1][1]: N_uv(u,v)
                           ND[0][2]: N_vv(u,v)
                         where N(u,v) is the unit normal at (u,v) 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtNormalDeriv( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT ** ND )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtNormalDeriv");

    NL_FLAG error = NL_NO;

    NL_REAL su, sv, suu, suv, svv, s, ss, a, b, c, os, fac, mlt;

    NL_POINT ** SD, Su, Sv, Suu, Suv, Svv, S;

    NL_VECTOR A, B, C, D, V;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get surface derivatives */

    SD = N_AllocPt2dArray( 3, 3, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    error = N_SrfDerivs( sur, u, v, ufl, vfl, NL_TRUE, 3, 3, SD );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get derivatives of the cross product S_u x S_v */

    N_VectorCross( SD[1][0], SD[0][1], &S );

    N_VectorCross( SD[2][0], SD[0][1], &A );
    N_VectorCross( SD[1][0], SD[1][1], &B );
    N_VectorSum( A, B, &Su );

    N_VectorCross( SD[1][1], SD[0][1], &A );
    N_VectorCross( SD[1][0], SD[0][2], &B );
    N_VectorSum( A, B, &Sv );

    N_VectorCross( SD[3][0], SD[0][1], &A );
    N_VectorCross( SD[2][0], SD[1][1], &B );
    N_VectorScale( B, 2.0, &B );
    N_VectorCross( SD[1][0], SD[2][1], &C );
    N_VectorSum( A, B, &V );
    N_VectorSum( V, C, &Suu );

    N_VectorCross( SD[2][1], SD[0][1], &A );
    N_VectorCross( SD[2][0], SD[0][2], &B );
    N_VectorCross( SD[1][0], SD[1][2], &C );
    N_VectorSum( A, B, &V );
    N_VectorSum( V, C, &Suv );

    N_VectorCross( SD[1][2], SD[0][1], &A );
    N_VectorCross( SD[1][1], SD[0][2], &B );
    N_VectorScale( B, 2.0, &B );
    N_VectorCross( SD[1][0], SD[0][3], &C );
    N_VectorSum( A, B, &V );
    N_VectorSum( V, C, &Svv );

    /* Get derivatives of |S_u x S_v| */

    N_VectorMagnitude( S, &s );

    if( N_FloatOpIsBad( 1.0, s, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    os = 1.0 / s;
    mlt = os;

    N_VectorDot( S, Su, &a );
    su = mlt * a;

    N_VectorDot( S, Sv, &a );
    sv = mlt * a;

    N_VectorDot( S, Suu, &a );
    N_VectorDot( Su, Su, &b );
    c = su * su;
    suu = mlt * (a + b - c);

    N_VectorDot( S, Suv, &a );
    N_VectorDot( Su, Sv, &b );
    c = su * sv;
    suv = mlt * (a + b - c);

    N_VectorDot( S, Svv, &a );
    N_VectorDot( Sv, Sv, &b );
    c = sv * sv;
    svv = mlt * (a + b - c);

    /* Now get derivatives of the unit normal */

    mlt = os * os;

    N_VectorScale( Su, s, &A );
    N_VectorScale( S, su, &B );
    N_VectorDiff( A, B, &C );
    N_VectorScale( C, mlt, &ND[1][0] );

    N_VectorScale( Sv, s, &A );
    N_VectorScale( S, sv, &B );
    N_VectorDiff( A, B, &C );
    N_VectorScale( C, mlt, &ND[0][1] );

    ss = s * s;
    mlt = mlt * os;

    N_VectorScale( Suu, ss, &A );
    fac = 2.0 *s * su;
    N_VectorScale( Su, fac, &B );
    fac = 2.0 *su * su - s * suu;
    N_VectorScale( S, fac, &C );
    N_VectorDiff( A, B, &D );
    N_VectorSum( D, C, &V );
    N_VectorScale( V, mlt, &ND[2][0] );

    N_VectorScale( Suv, ss, &A );
    fac = s * sv;
    N_VectorScale( Su, fac, &B );
    fac = s * su;
    N_VectorScale( Sv, fac, &C );
    fac = 2.0 *su * sv - s * suv;
    N_VectorScale( S, fac, &D );
    N_VectorDiff( A, B, &V );
    N_VectorDiff( V, C, &V );
    N_VectorSum( V, D, &V );
    N_VectorScale( V, mlt, &ND[1][1] );

    N_VectorScale( Svv, ss, &A );
    fac = 2.0 *s * sv;
    N_VectorScale( Sv, fac, &B );
    fac = 2.0 *sv * sv - s * svv;
    N_VectorScale( S, fac, &C );
    N_VectorDiff( A, B, &V );
    N_VectorSum( V, C, &V );
    N_VectorScale( V, mlt, &ND[0][2] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfEvalPtNormalDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes an upper  bound on the second derivatives of 
     the offset surface of a given NURBS surface. The approach is based 
     on  sampling the  Bezier  patches at  the nodes. A typical calling 
     example is:

       NL_SURFACE  sur;
       NL_REAL     d, Muu, Muv, Mvv; 
       ...
       (define sur);
       ...
       N_SrfOffsetGetMaxDeriv(&sur,d,&Muu,&Muv,&Mvv);

     THIS ROUTINE PROVIDES A REASONABLE APPROXIMATION TO THE NL_MAXIMUM OF
     THE SECOND DERIVATIVES. A PRECISE, ALTHOUGH  SIGNIFICANTLY  SLOWER
     APPROACH, IS TO USE SYMBOLIC OPERATORS.


   ACCESS:
   
     sur , input  ,  NURBS surface
     d   , input  ,  Offset distance
     Muu , output ,  Maximum 2nd derivative magnitude in u- direction
     Muv , output ,  Maximum 2nd derivative magnitude in uv-direction
     Mvv , output ,  Maximum 2nd derivative magnitude in v- direction


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfOffsetGetMaxDeriv( NL_SURFACE *sur, NL_REAL d, NL_REAL *Muu, NL_REAL *Muv, NL_REAL *Mvv )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INDEX i, j, k, l, r, s, a, b, al, bl;

    NL_REAL *U, *V, *u, *v, ui, vi, duu, duv, dvv, muu, muv, mvv, nuu, nuv, nvv, luu, luv, lvv;

    NL_POINT ** SD, ** ND;

    NL_SURFACE *** bez;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and decompose into Bezier patches */

    N_SrfGetDegrees( sur, &p, &q );

    error = N_SrfDecomposeToBez( sur, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    ND = N_AllocPt2dArray( 2, 2, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( p, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( q, &SL );

    if( v EQ NULL )
        NL_QUIT;

    luu = luv = lvv = 0.0;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            al = 0;
        else
            al = 1;

        for ( j = 0; j <= l; j++ )
        {
            if( j EQ 0 )
                bl = 0;
            else
                bl = 1;

            N_SrfGetKnots( bez[i][j], &r, &s, &U, &V );

            ui = (U[r] - U[0]) / p;
            vi = (V[s] - V[0]) / q;

            u[0] = U[0];
            u[p] = U[r];
            v[0] = V[0];
            v[q] = V[s];

            for ( a = 1; a <= p - 1; a++ )
                u[a] = U[0] + a * ui;

            for ( b = 1; b <= q - 1; b++ )
                v[b] = V[0] + b * vi;

            for ( a = al; a <= p; a++ )
            {
                for ( b = bl; b <= q; b++ )
                {
                    error = N_SrfDerivs( bez[i][j], u[a], v[b], NL_LEFT, NL_LEFT, NL_TRUE, 2, 2, SD );

                    if( error EQ NL_YES )
                        NL_OUT;

                    error = N_SrfEvalPtNormalDeriv( bez[i][j], u[a], v[b], NL_LEFT, NL_LEFT, ND );

                    if( error EQ NL_YES )
                    {
                        N_ErrClear();
                        error = NL_NO;
                        continue;
                    }

                    N_PtMagnitude( SD[2][0], &muu );
                    N_PtMagnitude( SD[1][1], &muv );
                    N_PtMagnitude( SD[0][2], &mvv );

                    N_PtMagnitude( ND[2][0], &nuu );
                    N_PtMagnitude( ND[1][1], &nuv );
                    N_PtMagnitude( ND[0][2], &nvv );

                    duu = muu + d * nuu;
                    duv = muv + d * nuv;
                    dvv = mvv + d * nvv;

                    if( duu GT luu )
                        luu = duu;

                    if( duv GT luv )
                        luv = duv;

                    if( dvv GT lvv )
                        lvv = dvv;
                }
            }
        }
    }

    *Muu = luu;
    *Muv = luv;
    *Mvv = lvv;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfOffsetGetMaxDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a grid of points and the unit normal at these
     points on a NURBS surface given a set of u- and v-values. A typical 
     calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  *u, *v;
       NL_INDEX      mu, mv;
       NL_POINT      **P;
       NL_VECTOR     **N
       ...
       (define sur; get u and v, allocate memory for P and N);
       ...
       N_SrfGetGridPtsNormals(&sur,u,v,mu,mv,NL_LEFT,NL_RIGHT,P,N);

     MEMORY TO STORE P AND N MUST BE ALLOCATED IN THE CALLING ROUTINE TO
     HOLD UP TO  P[mu][mv] AND  N[mu][mv]!


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Arrays of INCREASING parameters
     mu,mv   , input  ,  Highest indexes in u and v
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u or v)
     P,N     , output ,  Points and UNIT normals on the surface. P[i][j]
                         and N[i][j] are computed at (u[i],v[j])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfGetGridPtsNormals( NL_SURFACE *sur, NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX mu, NL_INDEX mv, NL_FLAG ufl, NL_FLAG vfl, NL_POINT ** P, NL_VECTOR ** N )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfGetGridPtsNormals");

    NL_FLAG error = NL_NO;

    NL_INDEX *usp, *vsp, i, j, k, l, n, m, r, s, a, b, c, d;

    NL_DEGREE p, q;

    NL_REAL *** NU, *** NV, ** wd, ** w, *U, *V, t, fac, cross_tol;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw;

    NL_POINT ** Q, ** AD, ** SD, T;

    NL_VECTOR A, BB, CC;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Check parameters */

    if( mu LT 0 OR mv LT 0 )
        NL_ERROR( NL_IND_ERR );

    if( u[0]LT U[0]OR u[mu]GT U[r] )
        NL_ERROR( NL_PAR_ERR );

    if( v[0]LT V[0]OR v[mv]GT V[s] )
        NL_ERROR( NL_PAR_ERR );

    cross_tol = 0.001 *NL_MTOL;

    /* Compute non-vanishing B-splines and map cp's to 3-D */

    NU = N_AllocReal3dArray( 1, p, mu, &SL );

    if( NU EQ NULL )
        NL_QUIT;

    NV = N_AllocReal3dArray( 1, q, mv, &SL );

    if( NV EQ NULL )
        NL_QUIT;

    usp = N_AllocInt1dArray( mu, &SL );

    if( usp EQ NULL )
        NL_QUIT;

    vsp = N_AllocInt1dArray( mv, &SL );

    if( vsp EQ NULL )
        NL_QUIT;

    error = N_BasisDerivsArray( knu, p, u, mu, ufl, 1, NU, usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisDerivsArray( knv, q, v, mv, vfl, 1, NV, vsp );

    if( error EQ NL_YES )
        NL_OUT;

    SD = N_AllocPt2dArray( 1, 1, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    Q = N_AllocPt2dArray( n, m, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            N_CPtToPt( Pw[i][j], &Q[i][j] );
    }

    /* Compute grid of points and normals */

    if( NOT N_IsSrfRat( sur ) )
    {
        /* Non-rational case */

        for ( a = 0; a <= mu; a++ )
        {
            for ( b = 0; b <= mv; b++ )
            {
                for ( k = 0; k <= 1; k++ )
                {
                    for ( l = 0; l <= 1 - k; l++ )
                    {
                        N_CopyPt( NL_ZERO, &SD[k][l] );

                        for ( i = 0; i <= p; i++ )
                        {
                            N_CopyPt( NL_ZERO, &T );
                            c = usp[a] - p + i;

                            for ( j = 0; j <= q; j++ )
                            {
                                d = vsp[b] - q + j;
                                N_VectorBlendPt( NV[l][j][b], Q[c][d], &T );
                            }
                            N_VectorBlendPt( NU[k][i][a], T, &SD[k][l] );
                        }
                    }
                }

                N_CopyPt( SD[0][0], &P[a][b] );
                N_VectorCross( SD[1][0], SD[0][1], &A );

                error = N_VectorNormalize( A, &N[a][b], &fac );

                if( error EQ NL_YES OR fac LE cross_tol )
                {
                    error = N_SrfEvalPtNormalAtPole( sur, u[a], v[b], ufl, vfl, &P[a][b], &BB, &CC, &N[a][b], SD );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
    }
    else
    {
        /* Rational case */

        w = N_AllocReal2dArray( n, m, &SL );

        if( w EQ NULL )
            NL_QUIT;

        wd = N_AllocReal2dArray( 1, 1, &SL );

        if( wd EQ NULL )
            NL_QUIT;

        AD = N_AllocPt2dArray( 1, 1, &SL );

        if( AD EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_CPtGetW( Pw[i][j], &w[i][j] );
        }

        for ( a = 0; a <= mu; a++ )
        {
            for ( b = 0; b <= mv; b++ )
            {
                for ( k = 0; k <= 1; k++ )
                {
                    for ( l = 0; l <= 1 - k; l++ )
                    {
                        N_CopyPt( NL_ZERO, &AD[k][l] );
                        wd[k][l] = 0.0;

                        for ( i = 0; i <= p; i++ )
                        {
                            N_CopyPt( NL_ZERO, &T );
                            t = 0.0;
                            c = usp[a] - p + i;

                            for ( j = 0; j <= q; j++ )
                            {
                                d = vsp[b] - q + j;
                                t += w[c][d] * NV[l][j][b];
                                N_VectorBlendPt( NV[l][j][b], Q[c][d], &T );
                            }
                            N_VectorBlendPt( NU[k][i][a], T, &AD[k][l] );
                            wd[k][l] += t * NU[k][i][a];
                        }
                    }
                }

                fac = 1.0 / wd[0][0];
                N_ScalePt( fac, AD[0][0], &SD[0][0] );

                N_ScalePt( wd[1][0], SD[0][0], &A );
                N_Diff2Pts( AD[1][0], A, &A );
                N_ScalePt( fac, A, &SD[1][0] );

                N_ScalePt( wd[0][1], SD[0][0], &A );
                N_Diff2Pts( AD[0][1], A, &A );
                N_ScalePt( fac, A, &SD[0][1] );

                N_CopyPt( SD[0][0], &P[a][b] );
                N_VectorCross( SD[1][0], SD[0][1], &A );

                error = N_VectorNormalize( A, &N[a][b], &fac );

                if( error EQ NL_YES OR fac LE cross_tol )
                {
                    error = N_SrfEvalPtNormalAtPole( sur, u[a], v[b], ufl, vfl, &P[a][b], &BB, &CC, &N[a][b], SD );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfGetGridPtsNormals */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes an upper bound on the second derivative of a 
     unit normal surface. The  approach is based on sampling the Bezier 
     patches at the nodes. A typical calling example is:

       NL_SURFACE  sur;
       NL_REAL     Nuu, Nuv, Nvv; 
       ...
       (define sur);
       ...
       N_SrfGetMaxSecondDeriv(&sur,&Nuu,&Nuv,&Nvv);

     THIS ROUTINE PROVIDES A REASONABLE APPROXIMATION TO THE NL_MAXIMUM OF
     THE SECOND NL_DERIVATIVE. FOR A PRECISE ALTHOUGH SIGNIFICANTLY SLOWER
     APPROACH SYMBOLIC OPERATORS CAN BE TRIED.


   ACCESS:
   
     sur , input  ,  NURBS surface
     Nuu , output ,  Maximum 2nd derivative magnitude in u- direction
     Nuv , output ,  Maximum 2nd derivative magnitude in uv-direction
     Nvv , output ,  Maximum 2nd derivative magnitude in v- direction


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfGetMaxSecondDeriv( NL_SURFACE *sur, NL_REAL *Nuu, NL_REAL *Nuv, NL_REAL *Nvv )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INDEX i, j, k, l, r, s, a, b, al, bl, mu, mv;

    NL_REAL *U, *V, *u, *v, ui, vi, duu, duv, dvv, muu, muv, mvv;

    NL_POINT ** ND;

    NL_SURFACE *** bez;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and decompose into Bezier patches */

    N_SrfGetDegrees( sur, &p, &q );

    error = N_SrfDecomposeToBez( sur, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    ND = N_AllocPt2dArray( 2, 2, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    mu = 2 * p;
    mv = 2 * q;

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mv, &SL );

    if( v EQ NULL )
        NL_QUIT;

    duu = duv = dvv = 0.0;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            al = 0;
        else
            al = 1;

        for ( j = 0; j <= l; j++ )
        {
            if( j EQ 0 )
                bl = 0;
            else
                bl = 1;

            N_SrfGetKnots( bez[i][j], &r, &s, &U, &V );

            ui = (U[r] - U[0]) / mu;
            vi = (V[s] - V[0]) / mv;

            u[0] = U[0];
            u[mu] = U[r];
            v[0] = V[0];
            v[mv] = V[s];

            for ( a = 1; a <= mu - 1; a++ )
                u[a] = U[0] + a * ui;

            for ( b = 1; b <= mv - 1; b++ )
                v[b] = V[0] + b * vi;

            for ( a = al; a <= mu; a++ )
            {
                for ( b = bl; b <= mv; b++ )
                {
                    error = N_SrfEvalPtNormalDeriv( bez[i][j], u[a], v[b], NL_LEFT, NL_LEFT, ND );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_PtMagnitude( ND[2][0], &muu );
                    N_PtMagnitude( ND[1][1], &muv );
                    N_PtMagnitude( ND[0][2], &mvv );

                    if( muu GT duu )
                        duu = muu;

                    if( muv GT duv )
                        duv = muv;

                    if( mvv GT dvv )
                        dvv = mvv;
                }
            }
        }
    }

    *Nuu = duu;
    *Nuv = duv;
    *Nvv = dvv;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfGetMaxSecondDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the 1st and the 2nd derivatives of the offset 
     surface of a given NURBS surface. The offset is computed as

         S_o(u,v) = S(u,v) + d(u,v)*V(u,v)

     where  S(u,v) is the base  surface,  d(u,v) is a distance function, 
     and V(u,v) is a direction surface. A typical calling example:

       NL_SURFACE    sur, surV;
       NL_SFUN       sfnd
       NL_PARAMETER  u, v;
       NL_POINT      **OD;
       ...
       (define sur; get sfnd and surV, and allocate memory for OD);
       ...
       N_SrfOffsetGetFirstSecondDerivs(&sur,u,v,&sfnd,&surV,NL_LEFT,NL_LEFT,OD);

     THE  DERIVATIVES  ARE  STORED  IN   OD[1][0],  OD[0][1],  OD[2][0], 
     OD[1][1]  AND  OD[0][2]. MEMORY  MUST BE  ALLOCATED IN  THE CALLING 
     ROUTINE! sur, sfnd AND surV MUST BE DEFINED NL_OVER THE SAME NL_PARAMETER 
     NL_RECTANGLE!


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values
     sfnd    , input  ,  Distance  function;  sfnd(u,v)  is  the  offset 
                         distance at (u,v)
     surV    , input  ,  Direction  surface;  surV(u,v) is the direction 
                         in which sur is to be offset at (u,v)
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t  is  in [t[j],t[j+1]) (NL_RIGHT deriva-
                                  tives are needed)
                           NL_RIGHT: t  is  in (t[j],t[j+1]] (NL_LEFT  deriva-
                                  tives are needed)
                           (t is u or v)
     OD      , output ,  Offset derivatives at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfOffsetGetFirstSecondDerivs( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_SFUN *sfnd, NL_SURFACE *surV, NL_FLAG ufl, NL_FLAG vfl, NL_POINT ** OD )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfOffsetGetFirstSecondDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX rs, ss, rf, sf, rv, sv;

    NL_REAL ** fd, *US, *VS, *UF, *VF, *UV, *VV;

    NL_POINT ** SD, ** VD;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get derivatives of base entities */

    N_SrfGetKnots( sur, &rs, &ss, &US, &VS );
    N_SrfFuncGetKnots( sfnd, &rf, &sf, &UF, &VF );
    N_SrfGetKnots( surV, &rv, &sv, &UV, &VV );

    if( US[0]NEQ UF[0]OR US[0]NEQ UV[0] )
        NL_ERROR( NL_INP_ERR );

    if( US[rs]NEQ UF[rf]OR US[rs]NEQ UV[rv] )
        NL_ERROR( NL_INP_ERR );

    if( VS[0]NEQ VF[0]OR VS[0]NEQ VV[0] )
        NL_ERROR( NL_INP_ERR );

    if( VS[ss]NEQ VF[sf]OR VS[ss]NEQ VV[sv] )
        NL_ERROR( NL_INP_ERR );

    fd = N_AllocReal2dArray( 2, 2, &SL );

    if( fd EQ NULL )
        NL_QUIT;

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    VD = N_AllocPt2dArray( 2, 2, &SL );

    if( VD EQ NULL )
        NL_QUIT;

    error = N_SrfDerivs( sur, u, v, ufl, vfl, NL_TRUE, 2, 2, SD );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SFuncDerivs( sfnd, u, v, ufl, vfl, 2, 2, fd );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfDerivs( surV, u, v, ufl, vfl, NL_TRUE, 2, 2, VD );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get offset derivatives */

    N_VectorPtAlongVector( SD[0][0], fd[0][0], VD[0][0], &OD[0][0] );

    N_VectorPtAlongVector( SD[1][0], fd[1][0], VD[0][0], &OD[1][0] );
    N_VectorPtAlongVector( OD[1][0], fd[0][0], VD[1][0], &OD[1][0] );

    N_VectorPtAlongVector( SD[0][1], fd[0][1], VD[0][0], &OD[0][1] );
    N_VectorPtAlongVector( OD[0][1], fd[0][0], VD[0][1], &OD[0][1] );

    N_VectorPtAlongVector( SD[2][0], fd[2][0], VD[0][0], &OD[2][0] );
    N_VectorPtAlongVector( OD[2][0], fd[1][0], VD[1][0], &OD[2][0] );
    N_VectorPtAlongVector( OD[2][0], fd[1][0], VD[1][0], &OD[2][0] );
    N_VectorPtAlongVector( OD[2][0], fd[0][0], VD[2][0], &OD[2][0] );

    N_VectorPtAlongVector( SD[1][1], fd[1][1], VD[0][0], &OD[1][1] );
    N_VectorPtAlongVector( OD[1][1], fd[1][0], VD[0][1], &OD[1][1] );
    N_VectorPtAlongVector( OD[1][1], fd[0][1], VD[1][0], &OD[1][1] );
    N_VectorPtAlongVector( OD[1][1], fd[0][0], VD[1][1], &OD[1][1] );

    N_VectorPtAlongVector( SD[0][2], fd[0][2], VD[0][0], &OD[0][2] );
    N_VectorPtAlongVector( OD[0][2], fd[0][1], VD[0][1], &OD[0][2] );
    N_VectorPtAlongVector( OD[0][2], fd[0][1], VD[0][1], &OD[0][2] );
    N_VectorPtAlongVector( OD[0][2], fd[0][0], VD[0][2], &OD[0][2] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfOffsetGetFirstSecondDerivs */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes  points  and  derivatives  of a NURBS surface
     without regard to its uv-domain bounds. That is, if the given (u,v)
     is out of bounds, then the appropriately extended surface is evalu-
     ated. If the surface is NL_G1-smoothly closed in a direction,  then it
     is not extended, but rather it is treated and evaluated as periodic 
     in that direction. A typical calling example is:

       NL_SURFACE    sur, esur;
       NL_PARAMETER  u, v;
       NL_FLAG       gflg;
       NL_INDEX      ndr;
       NL_POINT      P, **D;
       NL_STACKS     SG;
       ...
       (define sur, get u, v, ndr, cflg, and allocate D if required);
       ...
       N_SrfInitArrays(&esur);    (initialize before first call to N_SrfEvalPtDerivsUnbounded)
       ...
       N_SrfEvalPtDerivsUnbounded(&sur,u,v,ndr,cflg,&esur,
                &gflg,&P,D,&SG);             (multiple calls)


   ACCESS:
   
     sur  , input  ,  NURBS surface
     u,v  , input  ,  Parameter values.  These do not have to lie within
                      the uv-domain. 
     ndr  , input  ,  Maximum order of the  derivatives to compute. Only
                      the upper portion of the derivative matrix is com-
                      puted,  i.e.  derivatives whose total order is  <= 
                      ndr.  For example,  if ndr=2, then S, Su, Sv, Suu, 
                      Suv, and Svv are computed
     cflg , input  ,  Flag:
                       NL_G1  : sur is extended with tangent plane  contin-
                             uity where necessary for evaluation
                       NL_G1R : surP is extended with a ruled surface that
                             connects with NL_G1-continuity
                       NL_G2  : sur is extended by  reflection,  yielding a
                             NL_G2 (curvature) continuous  extension  where
                             necessary
                       NL_CMAX: the extension yields infinite  C-continuity
                             where necessary
     esur , in/out ,  The surface's extension is stored here.  esur must
                      be initialized to the NULL object before the first
                      call to N_SrfEvalPtDerivsUnbounded with the surface, sur.  Upon sub-
                      sequent calls to N_SrfEvalPtDerivsUnbounded, sur will be extended in
                      esur if required.  The  calling  routine  must not 
                      modify this surface until evaluations of  sur  are
                      finished.  If esur was used ( can be determined by
                      calling N_SrfAreArraysNULL),  then  it  can  be reused after
                      calls to N_FreeSrf and N_SrfInitArrays
     gflg , in/out ,  Flag:
                       This flag indicates whether sur is  NL_G1-closed  in
                       no, one, or  both  directions  (NL_NO, NL_UDIR, NL_VDIR or
                       NL_UVDIR, respectively).  This flag is computed only
                       if sur requires extension.  The  calling  routine
                       must not modify this flag  until  evaluations  of
                       sur are finished
     P    , output ,  Evaluated point on the surface. If ndr>0, P is not
                      used (point is in D[0][0])
     D    , output ,  Point and derivatives:  D[i][j]  is  the  (i,j)-th 
                      derivative,  e.g. D[2][0] is Suu.  If ndr=0,  D is 
                      not used (point is in P).  MEMORY  FOR  D  MUST BE 
                      ALLOCATED IN THE CALLING ROUTINE
     SG   , input  ,  Global stack.  sur's stack.  If required,  esur is 
                      also created on this stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtDerivsUnbounded( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_INDEX ndr, NL_FLAG cflg, NL_SURFACE *esur, NL_FLAG *gflg, NL_POINT *P, NL_POINT ** D, NL_STACKS *SG )
{

    /* NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtDerivsUnbounded"); */
    NL_PRIVATE NL_REAL g1tol = 0.1; /* g1 angular  tolerance */

    NL_FLAG error = NL_NO;

    NL_SURFACE *surptr;

    NL_FLAG g1_flag = NL_NO;

    NL_REAL us, ue, vs, ve, uu, vv, tols[2];

    /* Assign pointer to surface to be evaluated and get u,v - bounds */

    if( N_SrfAreArraysNULL( esur ) )
        surptr = sur;
    else
    {
        surptr = esur;
        g1_flag = *gflg;
    }

    N_SrfGetParameterBounds( surptr, &us, &ue, &vs, &ve );

    /* Check if (u,v) is in bounds */
    error = ((fabs( u ) > 1.0e12) || (fabs( v ) > 1.0e12)) ? 1 : 0;

    if( error )
        NL_OUT;

    if( u LT us OR u GT ue OR v LT vs OR v GT ve )
    { /* must extend or adjust periodic parameter */
        if( surptr EQ sur )
        {
            error = N_SrfCopy( sur, esur, SG );

            if( error EQ NL_YES )
                NL_OUT;

            tols[0] = NL_MTOL;
            tols[1] = g1tol;

            error = N_SrfIsClosedSmooth( surptr, 1, tols, NL_UVDIR, gflg );

            if( error EQ NL_YES )
                NL_OUT;

            surptr = esur;
            g1_flag = *gflg;
        }

        /* Now check each direction; either extend or adjust parm */

        if( u LT us OR u GT ue )
        {
            if( g1_flag EQ NL_UDIR OR g1_flag EQ NL_UVDIR )
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

                        error = N_SrfExtendByParamDist( surptr, uu, NL_UDIR, NL_START, cflg, surptr, SG, SG );

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

                        error = N_SrfExtendByParamDist( surptr, uu, NL_UDIR, NL_END, cflg, surptr, SG, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ue = uu;
                    } while ( u GT ue );
                }
            }
        }

        if( v LT vs OR v GT ve )
        {
            if( g1_flag EQ NL_VDIR OR g1_flag EQ NL_UVDIR )
            {
                while( v LT vs )
                    v = ve - vs + v; /* adjust parm */

                while( v GT ve )
                    v = vs - ve + v;
            }
            else
            { /* extend */
                if( v LT vs )
                {
                    do
                    {
                        if( vs - v LE 0.9 *( ve - vs ) )
                            vv = v;
                        else
                            vv = vs - (0.9 *( ve - vs ));

                        error = N_SrfExtendByParamDist( surptr, vv, NL_VDIR, NL_START, cflg, surptr, SG, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        vs = vv;
                    } while ( v LT vs );
                }
                else
                {
                    do
                    {
                        if( v - ve LE 0.9 *( ve - vs ) )
                            vv = v;
                        else
                            vv = ve + (0.9 *( ve - vs ));

                        error = N_SrfExtendByParamDist( surptr, vv, NL_VDIR, NL_END, cflg, surptr, SG, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ve = vv;
                    } while ( v GT ve );
                }
            }
        }
    }

    /* Now evaluate */

    if( ndr EQ 0 )
        error = N_SrfEvalPt( surptr, u, v, NL_LEFT, NL_LEFT, P );
    else
        error = N_SrfDerivs( surptr, u, v, NL_LEFT, NL_LEFT, NL_TRUE, ndr, ndr, D );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_SrfEvalPtDerivsUnbounded */

                                                                                                                                                            

/*******************************************************************//**


   DESCRIPTION:

     This surface routine extends a Nurbs surface  a  given  parametric
     distance.  The start or end in either the u- or v-direction can be 
     extended.  Continuity can be controlled via options.  After exten-
     sion,  the  original  parameter  domain still maps to the original 
     surface (i.e. the parameter domain is simply extended).  A typical 
     calling example is:

       NL_SURFACE   surP, surQ;
       NL_REAL      par;
       NL_STACKS    SP, SQ;
       ...
       (define surP and choose par);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfExtendByParamDist(&surP,par,NL_UDIR,NL_START,NL_CMAX,&surQ,&SP,&SQ);
       N_SrfExtendByParamDist(&surP,par,NL_VDIR,NL_END,NL_G1,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks
     for the proper amount  by looking at the highest indexes in surQ's  
     knot vector and polygon objects.


   ACCESS:
   
     surP , input  ,  NURBS surface
     par  , input  ,  The u- or v-parameter value defining the  new  u-
                      or v-extended boundary
     dflg , input  ,  Flag:
                       NL_UDIR : The surface is extended across either the  
                              u=umin or u=umax boundary
                       NL_VDIR : The surface is extended across either the  
                              v=vmin or v=vmax boundary
     end  , input  ,  Flag:
                       NL_START: The surface is extended across either the 
                              u=umin or v=vmin boundary (see dflg)
                       NL_END  : The surface is extended across either the 
                              u=umax or v=vmax boundary (see dflg)
     cont , input  ,  Flag:
                       NL_G1  : surP is  extended with tangent plane cont-
                             inuity.  surQ  is  NL_G1 continuous where the  
                             extension joins the original surface
                       NL_G1R : surP is extended with a ruled surface that
                             connects with NL_G1-continuity
                       NL_G2  : surP is extended by  reflection,  yielding 
                             a NL_G2 (curvature) continuous extension
                       NL_CMAX: the extension yields infinite C-continuity
                             along the boundary (no knot there)
     surQ , output ,  Surface after extension
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   METHOD
    The extension method depends on the continuity value
    cont == NL_CMAX ;
          1. find the 1st interior knot value next to the end being extended.
             Insert knots to make it a fully multiple (mult == degree) knot.
             (adding upt to p-1 knots).
             Changing knot array as:
          2. Make new extended knot array - add clamped knot at input par - remove 1 knot
             from old clamped end.  (adding p knots in extension direction)
          3. Set surQ controlPoints by mirroring ControlPoint Values
          4. Remove extra knots.
         Knot Vector changes
              |             #
         in:         *--*-----*--*       * = knot              
                     *           *   ==> | = given param value 
                     *           *       # = mirror param value
                     *           *       
              |                             
          1.         *--*-----*--*        
                     *  *        *  ==>   
                     *  *        *        
                     *           *        
              |              
          2.  *------*--*-----*--*                                    
              *      *  *        *
              *      *  *        *
              *                  *

              |              
          4.  *---------*-----*--*                                    
              *         *        *
              *         *        *
              *                  *

    cont == NL_G1R ; Add one span to end increasing that direction knotCount by degree
                  span is clamped on external end (knot multiplicity = degree + 1)
                  and fully multiple (knot multiplicity = degree) on the internal end.
                  The internal end is the curve's endPoint prior to extension.
                  An attempt to remove one knot from the old endPoint is made
                  so the returned surface may have have either degree or 
                  degree-1 more knots in the extension direction.
    else  1. find the interior param value an equal distance from the end point
             that the input extension param point is from the end point.
             Insert knots to make it a fully multiple (mult == degree) knot.
             (adding up to p new knots).
          2. Make new extended knot array - add clamped knot at input par - mirror
             all knots between old start knot and mirror knot about old start knot -
             remove 1 knot from old clamped end.  (adding span-1 knots in extension direction)
          3. Set surQ controlPoints by mirroring ControlPoint Values
          4. Remove extra knots.

         Knot Vector changes
               |             # 
         in:          *--*------*--*       * = knot
                      *            *   ==> | = given param value
                      *            *       # = mirror param value
                      *            *       
               |             #               
          1.          *--*---*--*--*        
                      *      *     *  ==>   
                      *      *     *        
                      *            *        
               |             #
          2.   *---*--*--*---*--*--*                                    
               *      *      *     *
               *      *      *     *
               *                   *
               |             #
          4.   *---*--*--*---*--*--*                                    
               *      *      *     *
               *      *      *     *
               *                   *

   ***********************************************************************/

NL_FLAG N_SrfExtendByParamDist
 (NL_SURFACE *surP,  /* in : tgt surface to extend */
  NL_REAL     par,   /* in : desired new u or v parameter value */
  NL_FLAG     dflg,  /* in : NL_UDIR  = extend U dir by either decreasing uMin or increasing uMax value */
                     /*      NL_VDir  = extend V dir by either decreasing vMin or increasing vMax value */
  NL_FLAG     end,   /* in : NL_START = decrease Min knot value */
                     /*      NL_END   = increase Max knot value */
  NL_FLAG     cont,  /* in : NL_G1    = surP is extended with tangent plane continuity. */
                     /*                 surQ is NL_G1 continuous where the */ 
                     /*                 extension joins the original surface */
                     /*      NL_G1R   = surP is extended with a ruled surface that */
                     /*                 connects with NL_G1-continuity. */
                     /*      NL_G2    = surP is extended by reflection, yielding */
                     /*                 a NL_G2 (curvature) continuous extension. */
                     /*      NL_CMAX  = the extension yields infinite C-continuity */
                     /*                 along the boundary (no knot there) */
  NL_SURFACE *surQ,  /* out: the extended surface */
  NL_STACKS  *SP,    /* out: stack containing surP */
  NL_STACKS  *SQ )   /* out: stack to contain all surQ newly constructed objects */
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfExtendByParamDist");

    NL_FLAG error = NL_NO;

    NL_SURFACE surA, surB;

    /*        NL_INTEGER     **bin;       */

    NL_INDEX ii, jj = 0, kk = 0, n, m, r, s, nq, mq, rq, sq, span = 0, mult = 0, nb, mb, rb, sb, kby, kkt, nn, ktal = 0, rem, kp;

    NL_DEGREE p, q, pq;

    NL_CPOINT ** Pw, ** Qw, ** Bw, ** Rw = NULL, *Dw;

    NL_POINT P1, P2, P3;

    NL_VECTOR V;

    NL_PLANE pln;

    NL_MATRIXTYPE mtp;

    NL_RMATRIX rma;

    NL_KNOTVECTOR *knu, *knv;

    NL_BOOLEAN rat;

    NL_REAL *UP, *VP, *UQ, *VQ, uu = 0.0, vv = 0.0, uuu = 0.0, vvv = 0.0, d1, d2 = 0.0, d3, d4 = 0.0, *UB, *VB, tal, w0, w1, ** RM, rhs[4], sol[2], hh[5][4], dr, dl, w, du, dv;

    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation for input surP */
    /* n = NL_UDIR CPoint max index,   p     = NL_UDIR degree           */
    /* m = NL_VDIR CPoint max index,   q     = NL_VDIR degree           */
    /* r = NL_UDIR max knot index,     Pw    = ControlPoint array    */
    /* s = NL_VDIR max knot index,     UP/VP = NL_UDIR/NL_VDIR knot arrays */
    /* knu/knv = NL_UDIR/NL_VDIR knot structs                           */
    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* set uu = point in range same distance from end as given par value */
    if( dflg EQ NL_UDIR )
    {
        /* Check if need to recurse because par is too big */
        if( cont EQ NL_G1 OR cont EQ NL_G2 )
            /* when extension is bigger than 7% of knot range */
            if( (end EQ NL_START AND UP[0] - par GT 0.9313 *( UP[r] - UP[0] ))OR( end EQ NL_END AND par - UP[r]GT 0.9313 *( UP[r] - UP[0] ) ) )
            { /* break big extension up into 2 smaller steps */
                if( end EQ NL_START )
                    d2 = par + 0.4671 *( UP[r] - UP[0] );
                else
                    d2 = par - 0.4671 *( UP[r] - UP[0] );

                /* make first part of extension */
                error = N_SrfExtendByParamDist( surP, d2, dflg, end, cont, surQ, SP, SQ );

                if( error EQ NL_YES )
                    NL_OUT;

                /* finish whole step with 2nd extension */
                error = N_SrfExtendByParamDist( surQ, par, dflg, end, cont, surQ, SQ, SQ );
                NL_OUT;
            }

        /* let uu = point in range same distance from end as given par value */
        if( end EQ NL_START )
            uu = 2.0 *UP[0] - par;
        else
            uu = 2.0 *UP[r] - par;

        /* snap uu to closest knot if within tolerance */
        d1 = NL_BIGD;
        /* find closest knot to uu */
        for ( ii = p + 1; ii < r - p; ii++ )
            if( fabs( uu - UP[ii] )LT d1 )
            {
                d1 = fabs( uu - UP[ii] );
                jj = ii;
            }
        /* snap uu to knot if within tolerance */
        if( d1 LT KN_TOL * (UP[r] - UP[0]) )
            uu = UP[jj];
    }
    else /* dflg == NL_VDIR */
    {
        /* set vv = point in range same distance from end as given par value */

        /* Check if need to recurse because par is too big */
        if( cont EQ NL_G1 OR cont EQ NL_G2 )
            /* when extension is bigger than 7% of knot range */
            if( (end EQ NL_START AND VP[0] - par GT 0.9313 *( VP[s] - VP[0] ))OR( end EQ NL_END AND par - VP[s]GT 0.9313 *( VP[s] - VP[0] ) ) )
            { /* break big extension up into 2 smaller steps */
                if( end EQ NL_START )
                    d2 = par + 0.4671 *( VP[s] - VP[0] );
                else
                    d2 = par - 0.4671 *( VP[s] - VP[0] );

                /* make first part of extension */
                error = N_SrfExtendByParamDist( surP, d2, dflg, end, cont, surQ, SP, SQ );

                if( error EQ NL_YES )
                    NL_OUT;

                /* finish whole step with 2nd extension */
                error = N_SrfExtendByParamDist( surQ, par, dflg, end, cont, surQ, SQ, SQ );
                NL_OUT;
            }

        /* let vv = point in range same distance from end as given par value */
        if( end EQ NL_START )
            vv = 2.0 *VP[0] - par;
        else
            vv = 2.0 *VP[s] - par;

        /* snap vv to closest knot if within tolerance */
        d1 = NL_BIGD;

        /* find closest knot to vv */
        for ( ii = q + 1; ii < s - q; ii++ )
            if( fabs( vv - VP[ii] )LT d1 )
            {
                d1 = fabs( vv - VP[ii] );
                jj = ii;
            }

        /* snap vv to knot if within tolerance */
        if( d1 LT KN_TOL * (VP[s] - VP[0]) )
            vv = VP[jj];
    } /* end setting uu or vv branches */

    /* Determine nq,mq,rq,sq values, and allocate working memory */
    /* nq = output curve NL_UDIR CPoint max index  */
    /* mq = output curve NL_VDIR CPoint max index  */
    /* rq = output curve NL_UDIR knot max index    */
    /* sq = output curve NL_VDIR knot max index    */
    /* kk = */
    /* Rw = */

    if( dflg EQ NL_UDIR )
    {
        /* first get nq and kk values */
        if( end EQ NL_START )
        {
            if( cont EQ NL_CMAX )
            {
                /* count multiplicity of 1st interior knot - limit mult to degree*/
                mult = 1;

                while( p + mult + 1 LE r )
                    if( UP[p + mult + 1]EQ UP[p + 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT p )
                    mult = p;

                /* new CPoint max index = */
                nq = n + 2 * p - mult;
                kk = 2 * p;
            }
            else if( cont EQ NL_G1R )
            {
                nq = n + p;
                kk = 0;
            }
            else
            {
                /* get span and multiplicity for uu value */
                error = N_BasisFindSpanAndMult(knu,      /* in : knot vector                              */
                                               p,        /* in : curve degree                             */
                                               uu,       /* in : parameter value                          */
                                               NL_RIGHT, /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                         /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,    /* out: index of span containing u               */
                                               &mult );  /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */
                if( error EQ NL_YES )
                    NL_OUT;
                nq = n + span + p - mult;
                kk = 2 * span;
            }
        }
        else /* (end EQ NL_END) branch */
        {
            if( cont EQ NL_CMAX )
            {
                /* count multiplicity of 1st interior knot - limit mult to degree*/
                mult = 1;

                while( r - p - mult - 1 GE 0 )
                    if( UP[r - p - mult - 1]EQ UP[r - p - 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT p )
                    mult = p;

                /* new CPoint max index = */
                nq = n + 2 * p - mult;
                kk = 2 * p;
            }

            else if( cont EQ NL_G1R )
                nq = n + p;

            else
            {
                error = N_BasisFindSpanAndMult(knu,     /* in : knot vector                              */
                                               p,       /* in : curve degree                             */
                                               uu,      /* in : parameter value                          */
                                               NL_LEFT, /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                        /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,   /* out: index of span containing u               */
                                               &mult ); /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */

                if( error EQ NL_YES )
                    NL_OUT;
                nq = n + r - span - 1 + p - mult;
                kk = 2 * (r - span - 1);
            }
        } /* end branching for nq and kk values */

        /* NL_VDIR values remain constant */
        mq = m;
        sq = s;

        /* get Uknot index from U CPoint index and degree */
        rq = nq + p + 1;

        /* allocate an [mq X kk] array of control points */
        if( cont NEQ NL_G1R )
        {
            Rw = N_AllocCPt2dArray( mq, kk, &SL );

            if( Rw EQ NULL )
                NL_QUIT;
        }
    }
    else /* dflag == NL_VDIR branch to determine nq,mq,rq,sq values */
    {
        if( end EQ NL_START )
        {
            if( cont EQ NL_CMAX )
            {
                /* count multiplicity of 1st interior knot - limit mult to degree*/
                mult = 1;

                while( q + mult + 1 LE s )
                    if( VP[q + mult + 1]EQ VP[q + 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT q )
                    mult = q;

                /* new CPoint max index = */
                mq = m + 2 * q - mult;
                kk = 2 * q;
            }
            else if( cont EQ NL_G1R )
            {
                mq = m + q;
                kk = 0;
            }
            else
            {
                /* get span and multiplicity for vv value */
                error = N_BasisFindSpanAndMult(knv,      /* in : knot vector                              */
                                               q,        /* in : curve degree                             */
                                               vv,       /* in : parameter value                          */
                                               NL_RIGHT, /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                         /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,    /* out: index of span containing u               */
                                               &mult );  /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */

                if( error EQ NL_YES )
                    NL_OUT;
                mq = m + span + q - mult;
                kk = 2 * span;
            }
        }
        else /* (end EQ NL_END) branch to compute mq and kk */
        {
            if( cont EQ NL_CMAX )
            {
                /* count multiplicity of 1st interior knot - limit mult to degree*/
                mult = 1;

                while( s - q - mult - 1 GE 0 )
                    if( VP[s - q - mult - 1]EQ VP[s - q - 1] )
                        mult += 1;
                    else
                        break;

                if( mult GT q )
                    mult = q;

                /* new CPoint max index = */
                mq = m + 2 * q - mult;
                kk = 2 * q;
            }
            else if( cont EQ NL_G1R )
            {
                mq = m + q;
                kk = 0;
            }
            else
            {
                error = N_BasisFindSpanAndMult(knv,     /* in : knot vector                              */
                                               q,       /* in : curve degree                             */
                                               vv,      /* in : parameter value                          */
                                               NL_LEFT, /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                                                        /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
                                               &span,   /* out: index of span containing u               */
                                               &mult ); /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */

                if( error EQ NL_YES )
                    NL_OUT;
                mq = m + s - span - 1 + q - mult;
                kk = 2 * (s - span - 1);
            }
        } /* end branches to compute mq and kk */

        /* NL_UDIR values remain constant */
        nq = n;
        rq = r;

        /* get Vknot index from V CPoint index and degree */
        sq = mq + q + 1;

        /* allocate an [nq X kk] array of control points */
        if( cont NEQ NL_G1R )
        {
            Rw = N_AllocCPt2dArray( nq, kk, &SL );

            if( Rw EQ NULL )
                NL_QUIT;
        }
    } /* end branches to compute nq,mq,rq,sq, and kk and Rw */

    /* See if memory is needed for surQ */
    surA = *surP;

    if( cont NEQ NL_G1R )
    {
        /* deep copy surP into local surB */
        N_SrfInitArrays( &surB );
        error = N_SrfCopy( surP, &surB, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* get pointers to output curve ControlPoint and knot arrays */
    if( surP EQ surQ )
    {
        /* allocate new CPoint and knot memory in surP */
        /* old surP values are available through surA  */
        error = N_AllocSrfArrays( surP, nq, mq, p, q, rq, sq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        /* get pointers to surQ ControlPoints and knots */
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        /* allocate or verify CPoint and knot memory in surQ */
        error = N_SrfSizeArrays( surQ, nq, mq, p, q, rq, sq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        /* get pointers to surQ ControlPoints and knots */
        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Patched NL_G1R case in as special case */
    if( cont EQ NL_G1R )
    {
        /* branch on extension direction */
        if( dflg EQ NL_UDIR )
        {
            /* load v-knot vector */
            for ( ii = 0; ii <= s; ii++ )
                VQ[ii] = VP[ii];

            /* rest depends on start/end case */

            d2 = (NL_REAL)p;
            uu = par;

            if( end EQ NL_START )
            {
                /* load u-knot vector */

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = uu;

                for ( ii = 1; ii <= r; ii++ )
                    UQ[ii + p] = UP[ii];

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                    for ( jj = 0; jj <= m; jj++ )
                        N_CopyCPt( Pw[ii][jj], &Qw[ii + p][jj] );

                /* compute new Qw[0][jj], i.e. boundary control points */

                du = -d2 * ((UP[0] - uu) / (UP[p + 1] - UP[0]));
                uu = UP[0];

                for ( jj = 0; jj <= m; jj++ )
                {
                    N_CPtToPtEuclid( Pw[0][jj], &P1 );
                    N_CPtToPtEuclid( Pw[1][jj], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector( P1, du, V, &P3 );

                    N_CPtGetW( Pw[1][jj], &w );
                    N_Weight( P3,           /* in : euclidean point                     */
                               w,           /* in : weight or NL_NOW                    */
                             &Qw[0][jj] );  /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[1][jj],...,Qw[p-1][jj]  (degree elevation) */

                for ( jj = 0; jj <= m; jj++ )
                    for ( ii = 1; ii < p; ii++ )
                    {
                        d1 = (NL_REAL)ii / d2;
                        N_Combine2CPts( 1.0 - d1,       /* alpha of Cw = alpha * Aw + beta * Bw */
                                        Qw[0][jj],      /* Aw    of Cw = alpha * Aw + beta * Bw */
                                        d1,             /* beta  of Cw = alpha * Aw + beta * Bw */
                                        Pw[0][jj],      /* Bw    of Cw = alpha * Aw + beta * Bw */
                                        &Qw[ii][jj] );  /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
            }
            else /* ( end EQ NL_START ) branch */
            {
                /* load u-knot vector */

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii + r] = uu;

                for ( ii = 0; ii < r; ii++ )
                    UQ[ii] = UP[ii];

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                    for ( jj = 0; jj <= m; jj++ )
                        N_CopyCPt( Pw[ii][jj], &Qw[ii][jj] );

                /* compute new Qw[nq][jj], i.e. boundary control points */

                du = d2 * ((uu - UP[r]) / (UP[r] - UP[r - p - 1]));
                uu = UP[r];

                for ( jj = 0; jj <= m; jj++ )
                {
                    N_CPtToPtEuclid( Pw[n - 1][jj], &P1 );
                    N_CPtToPtEuclid( Pw[n][jj], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector( P2, du, V, &P3 );

                    N_CPtGetW( Pw[n - 1][jj], &w );
                    N_Weight( P3,           /* in : euclidean point                     */
                               w,           /* in : weight or NL_NOW                    */
                             &Qw[nq][jj] ); /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[n+1][jj],...,Qw[nq-1][jj]  (degree elevation) */

                for ( jj = 0; jj <= m; jj++ )
                    for ( ii = 1; ii < p; ii++ )
                    {
                        d1 = (NL_REAL)ii / d2;
                        N_Combine2CPts( 1.0 - d1,          /* alpha of Cw = alpha * Aw + beta * Bw */
                                        Qw[n][jj],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                        d1,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                        Qw[nq][jj],        /* Bw    of Cw = alpha * Aw + beta * Bw */
                                        &Qw[n + ii][jj] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
            } /* end NL_START/NL_END branching for NL_G1R extensions */

            /* Try to remove one knot from curve's old endPoint */
            error = N_SrfRemoveKnotConditional( surQ, uu, 1, NL_MTOL, NL_UDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else /* ( dflg EQ NL_VDIR ) */
        {
            /* load u-knot vector */
            for ( ii = 0; ii <= r; ii++ )
                UQ[ii] = UP[ii];

            /* rest depends on start/end case */
            d2 = (NL_REAL)q;
            vv = par;

            if( end EQ NL_START )
            {
                /* load v-knot vector */

                for ( ii = 0; ii <= q; ii++ )
                    VQ[ii] = vv;

                for ( ii = 1; ii <= s; ii++ )
                    VQ[ii + q] = VP[ii];

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                    for ( jj = 0; jj <= m; jj++ )
                        N_CopyCPt( Pw[ii][jj], &Qw[ii][jj + q] );

                /* compute new Qw[ii][0], i.e. boundary control points */

                dv = -d2 * ((VP[0] - vv) / (VP[q + 1] - VP[0]));
                vv = VP[0];

                for ( ii = 0; ii <= n; ii++ )
                {
                    N_CPtToPtEuclid( Pw[ii][0], &P1 );
                    N_CPtToPtEuclid( Pw[ii][1], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector( P1, dv, V, &P3 );

                    N_CPtGetW( Pw[ii][1], &w );
                    N_Weight( P3,          /* in : euclidean point                     */
                               w,          /* in : weight or NL_NOW                    */
                             &Qw[ii][0] ); /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[ii][1],...,Qw[ii][q-1]  (degree elevation) */

                for ( ii = 0; ii <= n; ii++ )
                    for ( jj = 1; jj < q; jj++ )
                    {
                        d1 = (NL_REAL)jj / d2;
                        N_Combine2CPts( 1.0 - d1,       /* alpha of Cw = alpha * Aw + beta * Bw */
                                        Qw[ii][0],      /* Aw    of Cw = alpha * Aw + beta * Bw */
                                        d1,             /* beta  of Cw = alpha * Aw + beta * Bw */
                                        Pw[ii][0],      /* Bw    of Cw = alpha * Aw + beta * Bw */
                                        &Qw[ii][jj] );  /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
            }
            else
            {
                /* load v-knot vector */

                for ( ii = 0; ii <= q; ii++ )
                    VQ[ii + s] = vv;

                for ( ii = 0; ii < s; ii++ )
                    VQ[ii] = VP[ii];

                /* load control points from surP */

                for ( ii = 0; ii <= n; ii++ )
                    for ( jj = 0; jj <= m; jj++ )
                        N_CopyCPt( Pw[ii][jj], &Qw[ii][jj] );

                /* compute new Qw[ii][mq], i.e. boundary control points */

                dv = d2 * ((vv - VP[s]) / (VP[s] - VP[s - q - 1]));
                vv = VP[s];

                for ( ii = 0; ii <= n; ii++ )
                {
                    N_CPtToPtEuclid( Pw[ii][m - 1], &P1 );
                    N_CPtToPtEuclid( Pw[ii][m], &P2 );
                    N_VectorDir( P1,   /* in : Ps  of Vec = Pe - Ps */
                                 P2,   /* in : Pe  of Vec = Pe - Ps */
                                 &V ); /* out: Vec of Vec = Pe - Ps */
                    N_VectorPtAlongVector( P2, dv, V, &P3 );

                    N_CPtGetW( Pw[ii][m - 1], &w );
                    N_Weight( P3,           /* in : euclidean point                     */
                               w,           /* in : weight or NL_NOW                    */
                             &Qw[ii][mq] ); /* out: euclidean/homogeneous control point */
                }

                /* compute Qw[ii][m+1],...,Qw[ii][mq-1]  (degree elevation) */

                for ( ii = 0; ii <= n; ii++ )
                    for ( jj = 1; jj < q; jj++ )
                    {
                        d1 = (NL_REAL)jj / d2;
                        N_Combine2CPts( 1.0 - d1,          /* alpha of Cw = alpha * Aw + beta * Bw */
                                        Qw[ii][m],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                        d1,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                        Qw[ii][mq],        /* Bw    of Cw = alpha * Aw + beta * Bw */
                                        &Qw[ii][m + jj] ); /* Cw    of Cw = alpha * Aw + beta * Bw */
                    }
            }

            /* Try to remove one occurrence of vv */

            error = N_SrfRemoveKnotConditional( surQ, vv, 1, NL_MTOL, NL_VDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        } /* end on extension direction branching */

        /* If extension is in place, kill old surface */

        if( surP EQ surQ )
            N_FreeSrf( &surA, SP );

        NL_OUT; /* exit */
    }           /* end cont == NL_G1R case */

    /* Insert knot into surB, load UQ and VQ, partially load */
    /* Qw, and load Rw with the required ctrl pts from surB  */

    rem = 0;

    if( dflg EQ NL_UDIR )
    {
        /* let uuu = target param value for knot insertions */
        /* case NL_CMAX  = 1st interior knot                                */
        /* case other = uu, the interior mirror point to extension param */
        if( cont NEQ NL_CMAX )
            uuu = uu;

        else if( end EQ NL_START )
            uuu = UP[p + 1];

        else
            uuu = UP[r - p - 1];

        /* make uuu a fully multiple knot (mult==degree) using knot insertion */
        if( mult LT p )
        {
            /* make it a fully multiple knot using knot insertion */
            rem = p - mult;
            error = N_SrfInsertKnot( &surB, uuu, rem, NL_UDIR, &surB, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* set NL_VDIR Knots - they don't change */
        for ( ii = 0; ii <= s; ii++ )
            VQ[ii] = VP[ii];

        /* refresh surB size, index, and array pointer values */
        N_SrfGetCPtsDegreesAndKnots( &surB, &nb, &mb, &Bw, &p, &q, &rb, &sb, &UB, &VB );

        /* Init surQ extended ControlPoint and knot arrays */
        if( end EQ NL_START )
        {
            /* kby = number of knots inserted to make uuu fully multiple */
            kby = nq - nb;

            /* copy surB controlPoint values into surQ and Rw controlPoint arrays */
            for ( ii = 0; ii <= mb; ii++ )
            {
                for ( jj = 0; jj <= nb; jj++ )
                    N_CopyCPt( Bw[jj][ii], &Qw[kby + jj][ii] );

                for ( jj = 0; jj <= kby; jj++ )
                    N_CopyCPt( Bw[jj][ii], &Rw[ii][jj + kby] );
            }

            /* set surQ U knot values */
            if( cont EQ NL_CMAX )
            {
                kkt = p + 1;
                /* 1st knot = clamped at input par value */
                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = 2.0 *UB[0] - uu;

                /* next knot = old clamped knot - 1 knot */
                /* subsequent knots = original knots     */
                for ( ii = 1; ii <= rb; ii++ )
                    UQ[p + ii] = UB[ii];
            }
            else
            {
                kkt = span + 1;
                jj = kkt - 1;
                /* set surQ knots from span + 1 to end                  */
                /* let surQ knots after span = surB knots starting at 1 */
                for ( ii = 1; ii <= rb; ii++ )
                    UQ[span + ii] = UB[ii];

                /* set surQ knots from p+1 to span */
                /* mirror UB knots between old end and uuu across old end boundary */
                for ( ii = p + 1; ii <= span; ii++ )
                {
                    UQ[jj] = UQ[jj + 1] - (UB[ii] - UB[ii - 1]);
                    jj -= 1;
                }

                /* set surQ knots from 0 to p */
                /* clamp UQ end to par value  (d1 should equal par) */
                d1 = UQ[jj + 1] - (UB[kkt] - UB[span]);

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = d1;
            }
        }
        else /* ( end EQ NL_END ) */
        {
            kby = nq - nb;
            kk = nb - kby;

            for ( ii = 0; ii <= mb; ii++ )
            {
                for ( jj = 0; jj <= nb; jj++ )
                    N_CopyCPt( Bw[jj][ii], &Qw[jj][ii] );

                for ( jj = 0; jj <= kby; jj++ )
                    N_CopyCPt( Bw[jj + kk][ii], &Rw[ii][jj] );
            }

            kkt = rb - 1;

            for ( ii = 0; ii < rb; ii++ )
                UQ[ii] = UB[ii];

            if( cont EQ NL_CMAX )
            {
                for ( ii = 0; ii <= p; ii++ )
                    UQ[rb + ii] = 2.0 *UB[rb] - uu;
            }
            else
            {
                jj = rb;

                for ( ii = r - p - 1; ii > span; ii-- )
                {
                    UQ[jj] = UQ[jj - 1] + (UP[ii + 1] - UP[ii]);
                    jj += 1;
                }
                d1 = UQ[jj - 1] + (UP[span + 1] - uu);

                for ( ii = 0; ii <= p; ii++ )
                    UQ[jj + ii] = d1;
            }
        } /* end initializing curQ knot and controlPoint arrays from surB NL_START/NL_END branches */
    }
    else  /* ( dflg EQ NL_VDIR ) */
    {
        /* let vvv = target param value for knot insertions */
        /* case NL_CMAX  = 1st interior knot                                */
        /* case other = vv, the interior mirror point to extension param */
        if( cont NEQ NL_CMAX )
            vvv = vv;

        else if( end EQ NL_START )
            vvv = VP[q + 1];

        else
            vvv = VP[s - q - 1];

        /* make vvv a fully multiple knot (mult==degree) using knot insertion */
        if( mult LT q )
        {
            rem = q - mult;
            error = N_SrfInsertKnot( &surB, vvv, rem, NL_VDIR, &surB, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* set NL_UDIR Knots - they don't change */
        for ( ii = 0; ii <= r; ii++ )
            UQ[ii] = UP[ii];

        /* refresh surB size, index, and array pointer values */
        N_SrfGetCPtsDegreesAndKnots( &surB, &nb, &mb, &Bw, &p, &q, &rb, &sb, &UB, &VB );

        /* Init surQ extended ControlPoint and knot arrays */
        if( end EQ NL_START )
        {
            /* kby = number of knots inserted to make uuu fully multiple */
            kby = mq - mb;

            /* copy surB controlPoint values into surQ and Rw controlPoint arrays */
            for ( ii = 0; ii <= nb; ii++ )
            {
                for ( jj = 0; jj <= mb; jj++ )
                    N_CopyCPt( Bw[ii][jj], &Qw[ii][kby + jj] );

                for ( jj = 0; jj <= kby; jj++ )
                    N_CopyCPt( Bw[ii][jj], &Rw[ii][jj + kby] );
            }

            /* set surQ V knot values */
            if( cont EQ NL_CMAX )
            {
                kkt = q + 1;

                /* 1st knot = clamped at input par value */
                for ( ii = 0; ii <= q; ii++ )
                    VQ[ii] = 2.0 *VB[0] - vv;

                /* next knot = old clamped knot - 1 knot */
                /* subsequent knots = original knots     */
                for ( ii = 1; ii <= sb; ii++ )
                    VQ[q + ii] = VB[ii];
            }
            else
            {
                kkt = span + 1;
                jj = kkt - 1;
                /* set surQ knots from span + 1 to end                  */
                /* let surQ knots after span = surB knots starting at 1 */
                for ( ii = 1; ii <= sb; ii++ )
                    VQ[span + ii] = VB[ii];

                /* set surQ knots from p+1 to span */
                /* mirror UB knots between old end and uuu across old end boundary */
                for ( ii = q + 1; ii <= span; ii++ )
                {
                    VQ[jj] = VQ[jj + 1] - (VB[ii] - VB[ii - 1]);
                    jj -= 1;
                }

                /* set surQ knots from 0 to p */
                /* clamp UQ end to par value  (d1 should equal par) */
                d1 = VQ[jj + 1] - (VB[kkt] - VB[span]);

                for ( ii = 0; ii <= q; ii++ )
                    VQ[ii] = d1;
            }
        }
        else /* ( end EQ NL_END ) */
        {
            kby = mq - mb;
            kk = mb - kby;

            for ( ii = 0; ii <= nb; ii++ )
            {
                for ( jj = 0; jj <= mb; jj++ )
                    N_CopyCPt( Bw[ii][jj], &Qw[ii][jj] );

                for ( jj = 0; jj <= kby; jj++ )
                    N_CopyCPt( Bw[ii][jj + kk], &Rw[ii][jj] );
            }

            kkt = sb - 1;

            for ( ii = 0; ii < sb; ii++ )
                VQ[ii] = VB[ii];

            if( cont EQ NL_CMAX )
            {
                for ( ii = 0; ii <= q; ii++ )
                    VQ[sb + ii] = 2.0 *VB[sb] - vv;
            }
            else
            {
                jj = sb;

                for ( ii = s - q - 1; ii > span; ii-- )
                {
                    VQ[jj] = VQ[jj - 1] + (VP[ii + 1] - VP[ii]);
                    jj += 1;
                }
                d1 = VQ[jj - 1] + (VP[span + 1] - vv);

                for ( ii = 0; ii <= q; ii++ )
                    VQ[jj + ii] = d1;
            }
        } /* end initializing curQ knot and controlPoint arrays from surB NL_START/NL_END branches */
    }     /* end initializing surQ knot and ControlPoint array NL_UDIR/NL_VDIR branches */

    /* surQ ControlPoints are initialized */
    /* surQ knots are set                 */

    /* Compute the new control points defining the extension.     */
    /* Use method by Shetty & White, CAD, Sept. 1991 for NL_G1 & NL_G2. */
    /* Use derivative expressions for NL_CMAX.                       */

    rat = N_IsSrfRat( &surA );

    if( dflg EQ NL_UDIR )
    {
        nn = mq;
        pq = p;
    }
    else
    {
        nn = nq;
        pq = q;
    }

    if( cont NEQ NL_CMAX )
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ )
            {
                N_CPtToPtEuclid( Rw[ii][kby], &P1 ); /* get reflection plane */
                N_CPtToPtEuclid( Rw[ii][kby + 1], &P2 );
                N_VectorDir( P2,   /* in : Ps  of Vec = Pe - Ps */
                             P1,   /* in : Pe  of Vec = Pe - Ps */
                             &V ); /* out: Vec of Vec = Pe - Ps */
                error = N_VectorNormalize( V, &V, &d1 );

                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );
                N_CreatePlanePtNormal( &pln, /* out: Plane */
                                       P1,   /* in : Point lying on the plane */
                                       V );  /* in : Normal vector */

                kk = kby - 1; /* load reflected control points */

                for ( jj = 1; jj <= kby; jj++ )
                {
                    N_CptReflect( Rw[ii][kby + jj], /* in : Homogeneous point to be reflecte */
                                  pln,              /* in : The reflection plane */
                                  &Rw[ii][kk] );    /* out: Reflection of Pw */
                    kk -= 1;
                }
            } /* For nonrat surf, this already gives NL_G1 continuity */

            if( (NOT rat)AND cont EQ NL_G2 AND pq GT 1 )
            { /* recompute second row using NL_C2 condition */
                if( dflg EQ NL_UDIR )
                    d1 = 2.0 + (UB[p + 2] - UB[p + 1]) / (UB[p + 1] - UB[0]);
                else
                    d1 = 2.0 + (VB[q + 2] - VB[q + 1]) / (VB[q + 1] - VB[0]);

                for ( ii = 0; ii <= nn; ii++ )
                    N_TranslateSum2CPts(Rw[ii][kby + 2],    /* in : Cw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        d1,                 /* in : alpha of Dw = Cw + alpha*Aw + beta*Bw */
                                        Rw[ii][kby - 1],    /* in : Aw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        -d1,                /* in : beta  of Dw = Cw + alpha*Aw + beta*Bw */
                                        Rw[ii][kby + 1],    /* in : Bw    of Dw = Cw + alpha*Aw + beta*Bw */
                                        &Rw[ii][kby - 2] ); /* out: Dw    of Dw = Cw + alpha*Aw + beta*Bw */
            }         /* For nonrat surf, this gives NL_G2 continuity */

            if( rat ) /* must do more to get NL_G1 or NL_G2 */
            {
                if( dflg EQ NL_UDIR )
                    d1 = 2.0 *p / (UB[p + 1] - UB[0]);
                else
                    d1 = 2.0 *q / (VB[q + 1] - VB[0]);
                tal = -1.0e+20;

                for ( ii = 0; ii <= nn; ii++ )
                {
                    N_CPtGetW( Rw[ii][kby], &w0 );
                    N_CPtGetW( Rw[ii][kby + 1], &w1 );
                    d2 = d1 * (w1 - w0) / w0;

                    if( d2 GT tal )
                    {
                        tal = d2;
                        ktal = ii;
                    }
                }

                if( cont EQ NL_G2 AND pq GT 1 )
                { /* compute theta and e0 (Shetty & White) */
                    N_InitRealMatrix( &rma );
                    error = N_SetRealMatrix( &rma, 3, 1, NL_MT_FULL, 3, &SL );
                    N_GetRealMatrixData( &rma, &ii, &jj, &RM, &mtp, &jj );

                    if( dflg EQ NL_UDIR )
                    {
                        d3 = UB[p + 1] - UB[0];
                        d4 = UB[p + 2] - UB[p + 1];
                        d1 = p / d3;
                        d2 = ((NL_REAL)p * ((NL_REAL)p - 1.0)) / (d3 * (d3 + d4));
                    }
                    else
                    {
                        d3 = VB[q + 1] - VB[0];
                        d4 = VB[q + 2] - VB[q + 1];
                        d1 = q / d3;
                        d2 = ((NL_REAL)q * ((NL_REAL)q - 1.0)) / (d3 * (d3 + d4));
                    }
                    d4 = (2.0 *d3 + d4) / d3;
                    d3 = -2.0 *tal * d1;

                    for ( ii = 0; ii <= 4; ii++ )
                        N_CPtToWxWyWz( Rw[ktal][kby - 2 + ii], &hh[ii][0], &hh[ii][1], &hh[ii][2], &hh[ii][3] );

                    for ( ii = 0; ii <= 3; ii++ )
                    { /* set up system of 4 eqs in 2 unknowns */
                        RM[ii][0] = d1 * (hh[2][ii] - hh[3][ii]);
                        RM[ii][1] = hh[2][ii];
                        rhs[ii] = d2 * (d4 * (hh[3][ii] - hh[1][ii]) + (hh[0][ii] - hh[4][ii])) + d3 * (hh[2][ii] - hh[3][ii]);
                    }

                    /* solve system */ 
                    error = N_RealMatrixLstSqSolve( &rma,  /* in : NL_RMATRIX *A of Ax=b, with m+1 rows and n+1 columns (m>n) */
                                                    rhs,   /* in : NL_REAL    *b of Ax=b, sized:[m+1] */
                                                    sol ); /* out: NL_REAL    *x of Ax=b, sized:[n+1] */

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( dflg EQ NL_UDIR )
                    d1 = 2.0 + tal * (UQ[kkt] - UQ[kkt - 1]) / pq;
                else
                    d1 = 2.0 + tal * (VQ[kkt] - VQ[kkt - 1]) / pq;

                for ( ii = 0; ii <= nn; ii++ ) /* recompute first row for NL_G1 */
                    N_Combine2CPts( d1,                  /* alpha of Cw = alpha * Aw + beta * Bw */
                                    Rw[ii][kby],         /* Aw    of Cw = alpha * Aw + beta * Bw */
                                    -1.0,                /* beta  of Cw = alpha * Aw + beta * Bw */
                                    Rw[ii][kby + 1],     /* Bw    of Cw = alpha * Aw + beta * Bw */
                                    &Rw[ii][kby - 1] );  /* Cw    of Cw = alpha * Aw + beta * Bw */

                if( cont EQ NL_G2 AND pq GT 1 )
                { /* recompute second row for NL_G2 */
                    d2 = 1.0 / d2;
                    d1 = d1 * (sol[0] + 2.0 *tal);

                    for ( ii = 0; ii <= nn; ii++ )
                    {
                        for ( jj = 1; jj <= 4; jj++ )
                            N_CPtToWxWyWz( Rw[ii][kby - 2 + jj], &hh[jj][0], &hh[jj][1], &hh[jj][2], &hh[jj][3] );

                        for ( jj = 0; jj <= 3; jj++ )
                            hh[0][jj] = hh[4][jj] + d4 * (hh[1][jj] - hh[3][jj]) + d2 * (d1 * (hh[2][jj] - hh[3][jj]) + sol[1] * hh[2][jj]);
                        N_CPtFromWxWyWz( hh[0][0], hh[0][1], hh[0][2], hh[0][3], &Rw[ii][kby - 2] );
                    }
                }
            }
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ )
            {
                N_CPtToPtEuclid( Rw[ii][kby], &P1 ); /* get reflection plane */
                N_CPtToPtEuclid( Rw[ii][kby - 1], &P2 );
                N_VectorDir( P2,   /* in : Ps  of Vec = Pe - Ps */
                             P1,   /* in : Pe  of Vec = Pe - Ps */
                             &V ); /* out: Vec of Vec = Pe - Ps */
                error = N_VectorNormalize( V, &V, &d1 );

                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );
                N_CreatePlanePtNormal( &pln, /* out: Plane */
                                        P1,  /* in : Point lying on the plane */
                                        V ); /* in : Normal vector */

                kk = 2 * kby; /* load reflected control points */

                for ( jj = 0; jj < kby; jj++ )
                {
                    N_CptReflect( Rw[ii][jj],    /* in : Homogeneous point to be reflecte */
                                  pln,           /* in : The reflection plane */
                                  &Rw[ii][kk] ); /* out: Reflection of Pw */
                    kk -= 1;
                }
            } /* For nonrat surf, this already gives NL_G1 continuity */

            if( (NOT rat)AND cont EQ NL_G2 AND pq GT 1 )
            { /* recompute second row using NL_C2 condition */
                if( dflg EQ NL_UDIR )
                    d1 = 2.0 + (UB[rb - p - 1] - UB[rb - p - 2]) / (UB[rb] - UB[rb - p - 1]);
                else
                    d1 = 2.0 + (VB[sb - q - 1] - VB[sb - q - 2]) / (VB[sb] - VB[sb - q - 1]);

                for ( ii = 0; ii <= nn; ii++ )
                    N_TranslateSum2CPts( Rw[ii][kby - 2],    /* in : Cw    of Dw = Cw + alpha*Aw + beta*Bw */
                                         d1,                 /* in : alpha of Dw = Cw + alpha*Aw + beta*Bw */
                                         Rw[ii][kby + 1],    /* in : Aw    of Dw = Cw + alpha*Aw + beta*Bw */
                                         -d1,                /* in : beta  of Dw = Cw + alpha*Aw + beta*Bw */
                                         Rw[ii][kby - 1],    /* in : Bw    of Dw = Cw + alpha*Aw + beta*Bw */
                                         &Rw[ii][kby + 2] ); /* out: Dw    of Dw = Cw + alpha*Aw + beta*Bw */
            }         /* For nonrat surf, this gives NL_G2 continuity */

            if( rat ) /* must do more to get NL_G1 or NL_G2 */
            {
                if( dflg EQ NL_UDIR )
                    d1 = 2.0 *p / (UB[rb] - UB[rb - p - 1]);
                else
                    d1 = 2.0 *q / (VB[sb] - VB[sb - q - 1]);
                tal = -1.0e+20;

                for ( ii = 0; ii <= nn; ii++ )
                {
                    N_CPtGetW( Rw[ii][kby], &w0 );
                    N_CPtGetW( Rw[ii][kby - 1], &w1 );
                    d2 = d1 * (w1 - w0) / w0;

                    if( d2 GT tal )
                    {
                        tal = d2;
                        ktal = ii;
                    }
                }

                if( cont EQ NL_G2 AND pq GT 1 )
                { /* compute theta and e0 (Shetty & White) */
                    N_InitRealMatrix( &rma );
                    error = N_SetRealMatrix( &rma, 3, 1, NL_MT_FULL, 3, &SL );
                    N_GetRealMatrixData( &rma, &ii, &jj, &RM, &mtp, &jj );

                    if( dflg EQ NL_UDIR )
                    {
                        d3 = UB[rb] - UB[rb - p - 1];
                        d4 = UB[rb - p - 1] - UB[rb - p - 2];
                        d1 = p / d3;
                        d2 = ((NL_REAL)p * ((NL_REAL)p - 1.0)) / (d3 * (d3 + d4));
                    }
                    else
                    {
                        d3 = VB[sb] - VB[sb - q - 1];
                        d4 = VB[sb - q - 1] - VB[sb - q - 2];
                        d1 = q / d3;
                        d2 = ((NL_REAL)q * ((NL_REAL)q - 1.0)) / (d3 * (d3 + d4));
                    }
                    d4 = (2.0 *d3 + d4) / d3;
                    d3 = -2.0 *tal * d1;

                    for ( ii = 0; ii <= 4; ii++ )
                        N_CPtToWxWyWz( Rw[ktal][kby - 2 + ii], &hh[ii][0], &hh[ii][1], &hh[ii][2], &hh[ii][3] );

                    for ( ii = 0; ii <= 3; ii++ )
                    { /* set up system of 4 eqs in 2 unknowns */
                        RM[ii][0] = d1 * (hh[2][ii] - hh[3][ii]);
                        RM[ii][1] = hh[2][ii];
                        rhs[ii] = d2 * (d4 * (hh[3][ii] - hh[1][ii]) + (hh[0][ii] - hh[4][ii])) + d3 * (hh[2][ii] - hh[3][ii]);
                    }

                    /* solve system */
                    error = N_RealMatrixLstSqSolve( &rma,   /* in : NL_RMATRIX *A of Ax=b, with m+1 rows and n+1 columns (m>n) */
                                                     rhs,   /* in : NL_REAL    *b of Ax=b, sized:[m+1] */
                                                     sol ); /* out: NL_REAL    *x of Ax=b, sized:[n+1] */

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( dflg EQ NL_UDIR )
                    d1 = 2.0 + tal * (UQ[kkt + 1] - UQ[kkt]) / pq;
                else
                    d1 = 2.0 + tal * (VQ[kkt + 1] - VQ[kkt]) / pq;

                for ( ii = 0; ii <= nn; ii++ ) /* recompute first row for NL_G1 */
                    N_Combine2CPts( d1,                 /* alpha of Cw = alpha * Aw + beta * Bw */
                                    Rw[ii][kby],        /* Aw    of Cw = alpha * Aw + beta * Bw */
                                    -1.0,               /* beta  of Cw = alpha * Aw + beta * Bw */
                                    Rw[ii][kby - 1],    /* Bw    of Cw = alpha * Aw + beta * Bw */
                                    &Rw[ii][kby + 1] ); /* Cw    of Cw = alpha * Aw + beta * Bw */

                if( cont EQ NL_G2 AND pq GT 1 )
                { /* recompute second row for NL_G2 */
                    d2 = 1.0 / d2;
                    d1 = d1 * (sol[0] + 2.0 *tal);

                    for ( ii = 0; ii <= nn; ii++ )
                    {
                        for ( jj = 0; jj <= 3; jj++ )
                            N_CPtToWxWyWz( Rw[ii][kby - 2 + jj], &hh[jj][0], &hh[jj][1], &hh[jj][2], &hh[jj][3] );

                        for ( jj = 0; jj <= 3; jj++ )
                            hh[4][jj] = hh[0][jj] - d4 * (hh[1][jj] - hh[3][jj]) + d2 * (d1 * (hh[2][jj] - hh[3][jj]) + sol[1] * hh[2][jj]);
                        N_CPtFromWxWyWz( hh[4][0], hh[4][1], hh[4][2], hh[4][3], &Rw[ii][kby + 2] );
                    }
                }
            }
        }
    }
    else /* cont == NL_CMAX */
    {
        kk = pq;
        /*          bin = N_AllocInt2dArray(kk,kk,&SL);    */
        /*          if ( bin EQ NULL )  NL_QUIT;     */
        /*          N_PascalTriRow(bin,kk);             */

        Dw = N_AllocCPt1dArray( pq, &SL );

        if( Dw EQ NULL )
            NL_QUIT;

        if( end EQ NL_START )
        {
            if( dflg EQ NL_UDIR )
            {
                d2 = uu - UB[0];
                d3 = UB[p + 1] - UB[0];
            }
            else
            {
                d2 = vv - VB[0];
                d3 = VB[q + 1] - VB[0];
            }
            d4 = 1.0 / d3;

            for ( ii = 0; ii <= nn; ii++ )
            {
                dr = pq * d4;

                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes derivatives */
                {                              /* from the right                 */
                    N_CopyCPt( Rw[ii][kby + jj], &Dw[jj] );

                    if( jj % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < jj; kk++ ) /* compute the jj-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], Rw[ii][kby + kk], &Dw[jj] );
                        kp *= -1;
                    }
                    N_ScaleCPt( dr, Dw[jj], &Dw[jj] );
                    dr = dr * ((NL_REAL)pq - (NL_REAL)jj) * d4;
                }

                dl = d2 / pq;

                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes */
                {                              /* new control points */
                    if( jj % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw[jj], &Rw[ii][pq - jj] );
                    N_ScaleCPt( -kp * dl, Rw[ii][pq - jj], &Rw[ii][pq - jj] );

                    if( jj LT pq )
                        dl = dl * d2 / ((NL_REAL)pq - (NL_REAL)jj);

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (p-jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], Rw[ii][pq - kk], &Rw[ii][pq - jj] );
                        kp *= -1;
                    }
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 0; jj < pq; jj++ )
                    {
                        N_CPtGetW( Rw[ii][jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }
                }
            }
        }
        else
        {
            if( dflg EQ NL_UDIR )
            {
                d2 = UB[rb] - uu;
                d3 = UB[rb] - UB[rb - p - 1];
            }
            else
            {
                d2 = VB[sb] - vv;
                d3 = VB[sb] - VB[sb - q - 1];
            }
            d4 = 1.0 / d3;

            for ( ii = 0; ii <= nn; ii++ )
            {
                dl = -pq * d4;

                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes derivatives */
                {                              /* from the left                  */
                    N_CopyCPt( Rw[ii][kby - jj], &Dw[jj] );

                    if( jj % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < jj; kk++ ) /* compute the jj-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], Rw[ii][kby - kk], &Dw[jj] );
                        kp *= -1;
                    }
                    N_ScaleCPt( dl, Dw[jj], &Dw[jj] );
                    dl = -dl * ((NL_REAL)pq - (NL_REAL)jj) * d4;
                }

                dr = d2 / pq;

                for ( jj = 1; jj <= pq; jj++ ) /* this loop computes */
                {                              /* new control points */
                    if( jj % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw[jj], &Rw[ii][kby + jj] );
                    N_ScaleCPt( dr, Rw[ii][kby + jj], &Rw[ii][kby + jj] );

                    if( jj LT pq )
                        dr = dr * d2 / ((NL_REAL)pq - (NL_REAL)jj);

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (kby+jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[jj][kk], Rw[ii][kby + kk], &Rw[ii][kby + jj] );
                        kp *= -1;
                    }
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj <= pq; jj++ )
                    {
                        N_CPtGetW( Rw[ii][kby + jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }
                }
            }
        }
    } /* end cont branching for setting surQ ControlPoint array */

    /* Now load the control points from Rw into Qw */

    if( dflg EQ NL_UDIR )
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 0; jj < kby; jj++ )
                    N_CopyCPt( Rw[ii][jj], &Qw[jj][ii] );
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 1; jj <= kby; jj++ )
                    N_CopyCPt( Rw[ii][kby + jj], &Qw[nb + jj][ii] );
        }
    }
    else
    {
        if( end EQ NL_START )
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 0; jj < kby; jj++ )
                    N_CopyCPt( Rw[ii][jj], &Qw[ii][jj] );
        }
        else
        {
            for ( ii = 0; ii <= nn; ii++ )
                for ( jj = 1; jj <= kby; jj++ )
                    N_CopyCPt( Rw[ii][kby + jj], &Qw[ii][mb + jj] );
        }
    }

    /* Remove unnecessary knots */
    if( dflg EQ NL_UDIR )
    {
        if( rem GT 0 )
        {
            error = N_SrfRemoveKnotConditional( surQ, uuu, rem, 100.0 *NL_MTOL, NL_UDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( end EQ NL_START )
            uuu = UP[0];
        else
            uuu = UP[r];

        if( cont EQ NL_G1 )
            rem = 1;

        else if( cont EQ NL_G2 )
            rem = 2;

        else
            rem = p;

        error = N_SrfRemoveKnotConditional( surQ, uuu, rem, NL_MTOL, NL_UDIR, &ii, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        if( rem GT 0 )
        {
            error = N_SrfRemoveKnotConditional( surQ, vvv, rem, 100.0 *NL_MTOL, NL_VDIR, &ii, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( end EQ NL_START )
            vvv = VP[0];
        else
            vvv = VP[s];

        if( cont EQ NL_G1 )
            rem = 1;

        else if( cont EQ NL_G2 )
            rem = 2;

        else
            rem = q;

        error = N_SrfRemoveKnotConditional( surQ, vvv, rem, NL_MTOL, NL_VDIR, &ii, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* If extension is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}  /* end N_SrfExtendByParamDist */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine evaluates the rational or non  rational basis function 
     of a surface corresponding to a given index. It is assumed that the 
     end knots  are  repeated with  multiplicity = degree + 1. A typical 
     calling example is:

       NL_SURFACE    sur;
       NL_INDEX      k, l;
       NL_PARAMETER  u, v;
       NL_REAL       R;
       ...
       (define sur, get k, l, u and v);
       ...
       N_SrfBasisIEval(&sur,k,l,u,v,NL_LEFT,NL_RIGHT,&R);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     k,l     , input  ,  Indexes of basis function
     u,v     , input  ,  Parameter values
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) 
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u or v) 
     R       , output ,  Basis function computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfBasisIEval */
NL_FLAG N_SrfBasisIEval( NL_SURFACE *sur, NL_INDEX k, NL_INDEX l, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_REAL *R )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_REAL NU, NV;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute basis function */

    if( N_IsSrfRat( sur ) )
    {
        error = N_SrfRatBasisIEval( sur, k, l, u, v, ufl, vfl, R );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        N_SrfGetKnotVectors( sur, &knu, &knv );
        N_SrfGetDegrees( sur, &p, &q );

        error = N_BasisIEval( knu, k, p, u, ufl, &NU );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knv, l, q, v, vfl, &NV );

        if( error EQ NL_YES )
            NL_OUT;

        *R = NU * NV;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfBasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivative of a bivariate  rational basis 
     function with respect to a knot. A typical calling example is:

       NL_SURFACE    sur;
       NL_INDEX      i, j, k;
       NL_PARAMETER  u, v;
       NL_REAL       R;
       ...
       (define sur, get i, j, u, v and k);
       ...
       N_SrfRatBasisIKnotDeriv(&sur,i,j,k,u,v,NL_UDIR,NL_LEFT,NL_RIGHT,NL_LEFT,&Rd);
       

   ACCESS:
   
     sur , input  ,  NURBS surface
     i,j , input  ,  Indeces of rational basis function
     k   , input  ,  Index of  knot, i.e. the derivative with respect to 
                     t_k is computed (t is either u or v)
     u,v , input  ,  Parameter values
     dir , input  ,  NL_FLAG:
                       NL_UDIR: u-derivative required
                       NL_VDIR: v-derivative required
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              t_(k) != t_(k-1)
                       NL_RIGHT: right derivative.  NL_INDEX  k  MUST  SATISFY
                              t_(k) != t_(k+1)
                              (t is either u or v) 
     ulp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     vlp , input  ,  Flag:
                       NL_LEFT : v is in [v[j],v[j+1])
                       NL_RIGHT: v is in (v[j],v[j+1]]
     Rd  , output ,  Derivative computed  at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfRatBasisIKnotDeriv */
NL_FLAG N_SrfRatBasisIKnotDeriv( NL_SURFACE *sur, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG dir, NL_FLAG flk, NL_FLAG ulp, NL_FLAG vlp, NL_REAL *Rd )
{

    NL_FLAG error = NL_NO;

    NL_REAL ** w, *A, Nu, Nv, N, fu, fv, den, R;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Extract denominator */

    N_SFuncInitArrays( &sfn );
    error = N_SrfGetDenominatorFunc( sur, &sfn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( &sfn, &w, &A, &A );

    /* Compute the derivative */

    error = N_SrfFuncEvalPt( &sfn, u, v, ulp, vlp, &den );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfRatBasisIEval( sur, i, j, u, v, ulp, vlp, &R );

    if( error EQ NL_YES )
        NL_OUT;

    if( dir EQ NL_UDIR )
    {
        error = N_BasisIKnotDeriv( knu, i, k, p, u, flk, ulp, &Nu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knv, j, q, v, vlp, &N );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfFuncDerivFuncAtKnot( &sfn, k, u, v, NL_UDIR, flk, ulp, vlp, &fu );

        if( error EQ NL_YES )
            NL_OUT;

        *Rd = (w[i][j] * Nu * N - fu * R) / den;
    }
    else if( dir EQ NL_VDIR )
    {
        error = N_BasisIEval( knu, i, p, u, ulp, &N );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIKnotDeriv( knv, j, k, q, v, flk, vlp, &Nv );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfFuncDerivFuncAtKnot( &sfn, k, u, v, dir, flk, ulp, vlp, &fv );

        if( error EQ NL_YES )
            NL_OUT;

        *Rd = (w[i][j] * N * Nv - fv * R) / den;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRatBasisIKnotDeriv */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advance surface  construction routine  reparametrizes the two
     sets of curves, C_j(u) and  C_i(v), to force  compatible parameter
     values at the intersection points. This is a preprocessing step to
     calling N_CreateGordonSrf. The two sets of  curves must be NONRATIONAL  and
     already  compatible in the  B-spline sense. This  routine produces
     parameters u[i] and v[j] such that:

       C_j(u[i]) = C_i(v[j]) for all i and j

     The curves  are  REPARAMETRIZED  IN  PLACE and memory to store the
     output  parameters  is  ALLOCATED  INSIDE THE  ROUTINE. A  typical 
     calling example:

       NL_CURVE      **curU, **curV;
       NL_INDEX      k, l;
       NL_PARAMETER  **uu, **vv, *u, *v;
       NL_STACKS     SC, SP;
       ...
       (get array of curves, make them compatible, compute 
        intersections and get **uu and **vv)
       ...
       N_ReparmCrvsIsectPt(curU,curV,k,l,uu,vv,NL_LINEAR,&u,&v,&SC,&SP);

     ALL CROSS-SECTIONAL CURVES MUST BE ON THE  SAME STACK "SC". 


   ACCESS:
   
     curU  , in/out ,  U-curves in v-direction (C_j(u))
     curV  , in/out ,  V-curves in u-direction (C_i(v))
     k,l   , input  ,  Highest indeces in curU and curV
     uu    , input  ,  u parameters at intersection
     vv    , input  ,  v   parameters   at   intersection;   note  that 
                       C_j(uu[i][j]) = C_i(vv[i][j]), i=0,...,l; j=0,..
                       .,k
     rep   , input  ,  Flag:
                         NL_LINEAR    : use a  piecewise  linear  function
                                     for    reparametrization;   Gordon 
                                     surface  will be C0 continuous
                         NL_QUADRATIC: use  a   quadrartic  function  for 
                                     reparametrization; Gordon  surface
                                     will be NL_C1, however, the degree is
                                     doubled.
     u,v   , output ,  compatible u- and  v-parameters at  intersection
                       points, i.e. C_j(u[i]) = C_i(v[j]) for all i,j
     SC    , input  ,  curs' memory stack
     SP    , input  ,  u's and v's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ReparmCrvsIsectPt( NL_CURVE ** curU, NL_CURVE ** curV, NL_INDEX k, NL_INDEX l, NL_PARAMETER ** uu, NL_PARAMETER ** vv, NL_FLAG rep, NL_PARAMETER ** u, NL_PARAMETER ** v, NL_STACKS *SC, NL_STACKS *SP )
{
    NL_PRIVATE NL_STRING rname = _T("N_ReparmCrvsIsectPt");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r;

    NL_DEGREE p;

    NL_REAL *thisCurveParams, *U, fd[2], sum;

    NL_PARAMETER *uParamAverages, *vParamAverages;

    NL_CFUN cfn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local memory */

    switch( rep )
    {
        case NL_LINEAR:
            p = 1;
            break;

        case NL_QUADRATIC:
            p = 2;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    n = NL_MAX( k, l );
    m = n + p + 1;

    uParamAverages = N_AllocReal1dArray( l, SP );

    if( uParamAverages EQ NULL )
        NL_QUIT;

    vParamAverages = N_AllocReal1dArray( k, SP );

    if( vParamAverages EQ NULL )
        NL_QUIT;

    thisCurveParams = N_AllocReal1dArray( n, &SL );

    if( thisCurveParams EQ NULL )
        NL_QUIT;

    error = N_AllocCFuncArrays( &cfn, n, p, m, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get new parameters.
     * For each curve, they are the average of all of the other
     * curves' intersection parameters at that curve.
     * They should be nearly the same.
     */

    for ( i = 0; i <= l; i++ )  /* For each v-curve */
    {
        sum = 0.0;

        for ( j = 0; j <= k; j++ )  /* Average int param with each u-curve. */
          { sum += uu[i][j]; }
        uParamAverages[i] = sum / (k + 1.0);
    }

    for ( j = 0; j <= k; j++ )  /* For each u-curve */
    {
        sum = 0.0;

        for ( i = 0; i <= l; i++ )  /* Average int param with each v-curve. */
          { sum += vv[i][j]; }
        vParamAverages[j] = sum / (l + 1.0);
    }

    /* Reparametrize u-curves */

    N_CFuncGetKnots( &cfn, &r, &U );  /* cfn: not set yet, we set it here. */

    for ( j = 0; j <= k; j++ )  /* along each u-curve */
    {
        for ( i = 0; i <= l; i++ )  /* pick out its intersections, into 'thisCurveParams' array. */
          { thisCurveParams[i] = uu[i][j]; }

        error = N_FitFuncInterpGivenParams( thisCurveParams, l, p, uParamAverages, &cfn, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( p EQ 2 )
        {
            for ( r = p + 1; r <= l; r++ )
            {
                error = N_CFuncDerivs( &cfn, U[r], NL_LEFT, 1, fd );

                if( error EQ NL_YES )
                    NL_OUT;

                if( fd[1]LE 0.0 )
                    NL_ERROR( NL_DER_ERR );
            }
        }

        error = N_SrfReparamFunc( curU[j], &cfn, curU[j], SC, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Reparametrize v-curves */

    N_CFuncSetSizeIndices( &cfn, n, p, m );

    for ( i = 0; i <= l; i++ )
    {
        for ( j = 0; j <= k; j++ )
            thisCurveParams[j] = vv[i][j];

        error = N_FitFuncInterpGivenParams( thisCurveParams, k, p, vParamAverages, &cfn, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( p EQ 2 )
        {
            for ( r = p + 1; r <= k; r++ )
            {
                error = N_CFuncDerivs( &cfn, U[r], NL_LEFT, 1, fd );

                if( error EQ NL_YES )
                    NL_OUT;

                if( fd[1]LE 0.0 )
                    NL_ERROR( NL_DER_ERR );
            }
        }

        error = N_SrfReparamFunc( curV[i], &cfn, curV[i], SC, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make compatible again */

    error = N_CrvsMakeCompatible( curU, k, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvsMakeCompatible( curV, l, SC );

    if( error EQ NL_YES )
        NL_OUT;

    *u = uParamAverages;
    *v = vParamAverages;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ReparmCrvsIsectPt */


/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine creates a swung surface
     given a profile curve in the  [x,z] plane  and a trajectory curve 
     in the [x,y] plane.  The swung surface is a generalization of the 
     surface of rotation in which the circular sweep curve is replaced 
     by a general sweep curve. If the output surface is initialized to 
     NULL, memory to store new control points and knots is  allocated. 
     A typical calling example is:

       NL_REAL     alf;
       NL_CURVE    curP, curT;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get curP, curT and alf);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSwungSrf(&curP,&curT,alf,&sur,&SG);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector  and  polygon  objects. 


   ACCESS:
   
     curP , input  ,  Profile curve in the [x,z] plane
     curT , input  ,  Trajectory curve in the [x,y] plane
     alf  , input  ,  Scale factor
     sur  , output ,  Swung surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSwungSrf( NL_CURVE *curP, NL_CURVE *curT, NL_REAL alf, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSwungSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *US, *VS, xp, yp, zp, wp, xt, yt, zt, wt, xq, yq, zq;

    NL_POINT P, T, Q;

    NL_CPOINT *Pw, *Tw, ** Qw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &r, &U );
    N_CrvGetCPtsDegreeAndKnots( curT, &m, &Tw, &q, &s, &V );

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &US, &VS );

    /* Compute control points */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P );
        N_CPtGetW( Pw[i], &wp );
        N_PtToXYZ( P, &xp, &yp, &zp );

        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Tw[j], &T );
            N_CPtGetW( Tw[j], &wt );
            N_PtToXYZ( T, &xt, &yt, &zt );

            xq = alf * xp * xt;
            yq = alf * xp * yt;
            zq = zp;

            N_PtFromXYZ( xq, yq, zq, &Q );

            if( wp EQ NL_NOW AND wt EQ NL_NOW )
                N_PtToCPt( Q, &Qw[i][j] );

            else if( wp EQ NL_NOW AND wt NEQ NL_NOW )
                N_Weight( Q, wt, &Qw[i][j] );

            else if( wp NEQ NL_NOW AND wt EQ NL_NOW )
                N_Weight( Q, wp, &Qw[i][j] );

            else if( wp NEQ NL_NOW AND wt NEQ NL_NOW )
                N_Weight( Q, wp *wt, &Qw[i][j] );
        }
    }

    /* Compute the knot vectors */

    for ( i = 0; i <= r; i++ )
        US[i] = U[i];

    for ( j = 0; j <= s; j++ )
        VS[j] = V[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSwungSrf */



/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  a skinned 
     surface given a set of cross-sectional  curves. This  routine does
     not assume  compatibility of  these curves, however, if they are,
     compatibility  computations are skipped. If the output surface is 
     initialized to NULL, memory to store new control points and knots  
     is allocated. A typical calling example is:

       NL_CURVE    **cur;
       NL_INDEX    k;
       NL_DEGREE   deg;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       NL_REAL     knotTol;
       ...
       (get array of curves and choose degree);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrf(cur,k,NL_YES,NL_NO,deg,NL_UDIR,knotTol,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE SAME STACK "SC". If the surface weights are outside the 
     range [NL_WMIN,NL_WMAX], the error NL_WEI_ERR is returned even through the 
     NL_SURFACE IS  CONSTRUCTED. In  case the  surface is used, the error 
     can be cleared by N_ErrClear().


   ACCESS:
   
     cur , input  ,  Cross-sectional curves (array of  curve pointers)
     k   , input  ,  Highest index in curve array
     cmp , input  ,  Flag:
                       NL_YES: make curves compatible
                       NL_NO : curves are compatible already
     cld , input  ,  Flag:
                       NL_YES: create  NL_C1  smoothly  closed  surface   if 
                            cur[0]=cur[k]
                       NL_NO : do not force NL_C1 closure  if  cur[0]=cur[k]
     deg , input  ,  Degree of surface in the skinning direction
     dir , input  ,  Flag:
                       NL_UDIR: skin in the  u-direction (all  curves are 
                             v-curves)
                       NL_VDIR: skin in the  v-direction (all  curves are 
                             u-curves)
     knotTol, input, Compatibility   tolerance.  The   cross-sectional 
                     curves  must  not  deviate  from  the  compatible 
                     curves more than knotTol. If unknown, set knotTol = 0.0.
     sur , output ,  Skinned surface
     SC  , input  ,  curs' memory stack
     SS  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrf( NL_CURVE ** cur, NL_INDEX k, NL_FLAG cmp, NL_FLAG cld, NL_DEGREE deg, NL_FLAG dir, double knotTol, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrf");

    NL_FLAG closed = NL_NO, error = NL_NO;

    NL_INDEX i, j, l, n, m, nu, mv, r, s, nh, mh;

    NL_DEGREE p, q;

    NL_REAL *U = NULL, *V = NULL, *US, *VS, *u, *v, *uc = NULL, *vc = NULL, *tmp;

    NL_CPOINT ** Qw, ** Rw, ** Sw = NULL, *Pw, *Aw, *Bw, *Cw = NULL;

    NL_CVECTOR *Ws, *We, Dw[2];

    NL_KNOTVECTOR *knu, *knv, *knt = NULL;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curves compatible */

    if( cmp EQ NL_YES )
    {
        /* if knotTol == 0, N_CrvsMakeCompatibleKnotTol will determine a knotTol */
        error = N_CrvsMakeCompatibleKnotTol( cur, k, knotTol, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check if curve set is closed */

    if( N_CrvsAreCoincident( cur[0], cur[k], NL_MTOL, SC ) )
    {
        if( cld EQ NL_YES )
            closed = NL_YES;
    }

    /* Get indexes and control point data */

    switch( dir )
    {
        case NL_UDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &m, &Pw, &q, &s, &V );

            if( closed EQ NL_YES )
                n = k + 2;
            else
                n = k;
            nu = k;
            mv = m;
            p = deg;
            r = n + p + 1;

            Rw = N_AllocCPt2dArray( k, m, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= k; i++ )
            {
                N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Pw[j], &Rw[i][j] );
            }

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( k, m, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                nh = k / 2;

                for ( i = 0; i <= k; i++ )
                {
                    l = (nh + i) % k;

                    if( l EQ 0 )
                        l = k;

                    for ( j = 0; j <= m; j++ )
                        N_CopyCPt( Rw[l][j], &Sw[i][j] );
                }
            }
            break;

        case NL_VDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &n, &Pw, &p, &r, &U );

            if( closed EQ NL_YES )
                m = k + 2;
            else
                m = k;
            nu = n;
            mv = k;
            q = deg;
            s = m + q + 1;

            Rw = N_AllocCPt2dArray( n, k, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( j = 0; j <= k; j++ )
            {
                N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Pw[i], &Rw[i][j] );
            }

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( n, k, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                mh = k / 2;

                for ( j = 0; j <= k; j++ )
                {
                    l = (mh + j) % k;

                    if( l EQ 0 )
                        l = k;

                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Rw[i][l], &Sw[i][j] );
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &US, &VS );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    Aw = N_AllocCPt1dArray( k, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    /* Get parameters */

    u = N_AllocReal1dArray( nu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mv, &SL );

    if( v EQ NULL )
        NL_QUIT;

    error = N_FitCalcSrfParamValues( (NL_VOID ** )Rw, nu, mv, NL_HPOINT, NL_CHORDLENGTH, u, v );

    if( error EQ NL_YES )
        NL_OUT;

    if( closed EQ NL_YES )
    {
        uc = N_AllocReal1dArray( nu, &SL );

        if( uc EQ NULL )
            NL_QUIT;

        vc = N_AllocReal1dArray( mv, &SL );

        if( vc EQ NULL )
            NL_QUIT;

        error = N_FitCalcSrfParamValues( (NL_VOID ** )Sw, nu, mv, NL_HPOINT, NL_CHORDLENGTH, uc, vc );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute surface control points */

    switch( dir )
    {
        case NL_UDIR:

            error = N_AllocCrvArrays( &curA, n, p, r, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                Cw = N_AllocCPt1dArray( k, &SL );

                if( Cw EQ NULL )
                    NL_QUIT;

                knt = N_AllocKnotVectorAndArray( r, &SL );

                if( knt EQ NULL )
                    NL_QUIT;

                N_FitCrvCalcKnotVector( uc, k, p, knt );
                N_FitCalcKnotVectorEndDerivs( u, k, p, knu );
            }
            else
            {
                N_FitCrvCalcKnotVector( u, k, p, knu );
            }

            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= k; i++ )
                {
                    N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );
                    N_CopyCPt( Pw[j], &Aw[i] );
                }

                if( closed EQ NL_YES )
                {
                    for ( i = 0; i <= k; i++ )
                        N_CopyCPt( Sw[i][j], &Cw[i] );

                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Cw, k, NL_HPOINT, uc, knt, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, uc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    Ws = &Dw[1];
                    We = &Dw[1];

                    N_CrvSetSizeIndices( &curA, n, p, r );
                    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)Aw, k, NL_HPOINT, u, knu, p, (NL_VOID *)Ws, (NL_VOID *)We, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Aw, k, NL_HPOINT, u, knu, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Bw[i], &Qw[i][j] );
            }

            for ( j = 0; j <= s; j++ )
                VS[j] = V[j];
            break;

        case NL_VDIR:

            error = N_AllocCrvArrays( &curA, m, q, s, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                Cw = N_AllocCPt1dArray( k, &SL );

                if( Cw EQ NULL )
                    NL_QUIT;

                knt = N_AllocKnotVectorAndArray( s, &SL );

                if( knt EQ NULL )
                    NL_QUIT;

                N_FitCrvCalcKnotVector( vc, k, q, knt );
                N_FitCalcKnotVectorEndDerivs( v, k, q, knv );
            }
            else
            {
                N_FitCrvCalcKnotVector( v, k, q, knv );
            }

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= k; j++ )
                {
                    N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );
                    N_CopyCPt( Pw[i], &Aw[j] );
                }

                if( closed EQ NL_YES )
                {
                    for ( j = 0; j <= k; j++ )
                        N_CopyCPt( Sw[i][j], &Cw[j] );

                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Cw, k, NL_HPOINT, vc, knt, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, vc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    Ws = &Dw[1];
                    We = &Dw[1];

                    N_CrvSetSizeIndices( &curA, m, q, s );
                    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)Aw, k, NL_HPOINT, v, knv, q, (NL_VOID *)Ws, (NL_VOID *)We, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Aw, k, NL_HPOINT, v, knv, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Bw[j], &Qw[i][j] );
            }

            for ( i = 0; i <= r; i++ )
                US[i] = U[i];
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Check the weights */

    error = N_SrfWeightsAreValid( sur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrf */


/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  a skinned 
     surface approximating a given set of profile curves. This  
     routine does not assume  compatibility of  these curves, however, 
     if  they  are,  compatibility  computations  are skipped. If  the 
     output  surface  is  initialized  to  NULL, memory  to  store new 
     control points and knots is allocated. A typical calling example:

       NL_CURVE    **cur;
       NL_INDEX    k, hic;
       NL_DEGREE   deg;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (get array of curves, and choose deg and hic);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrfApprox(cur,k,NL_YES,NL_NO,hic,deg,NL_UDIR,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE SAME STACK "SC". If the surface weights are outside the 
     range [NL_WMIN,NL_WMAX], the error NL_WEI_ERR is returned even through the 
     NL_SURFACE IS  CONSTRUCTED. In  case the  surface is used, the error 
     can be cleared by N_ErrClear().


   ACCESS:
   
     cur , input  ,  Cross-sectional curves (array of  curve pointers)
     k   , input  ,  Highest index in curve array
     cmp , input  ,  Flag:
                       NL_YES: make curves compatible
                       NL_NO : curves are compatible already
     cld , input  ,  Flag:
                       NL_YES: create  NL_C1  smoothly  closed  surface   if 
                            cur[0]=cur[k]
                       NL_NO : do not force NL_C1 closure  if  cur[0]=cur[k]
     hic , input  ,  Highest index of control  points of approximating
                     surface
     deg , input  ,  Degree of surface in the skinning direction
     dir , input  ,  Flag:
                       NL_UDIR: skin in the  u-direction (all  curves are 
                             v-curves)
                       NL_VDIR: skin in the  v-direction (all  curves are 
                             u-curves)
     sur , output ,  Skinned approximating surface
     SC  , input  ,  curs' memory stack
     SS  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfApprox( NL_CURVE ** cur, NL_INDEX k, NL_FLAG cmp, NL_FLAG cld, NL_INDEX hic, NL_DEGREE deg, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrfApprox");

    NL_FLAG closed = NL_NO, error = NL_NO;

    NL_INDEX I[2], i, j, l, n, m, nc, mc, r, s, nh, mh;

    NL_DEGREE p, q;

    NL_REAL *U = NULL, *V = NULL, *US, *VS, *u, *v, *uc = NULL, *vc = NULL, *tmp, *wp = NULL, wd[2];

    NL_CPOINT ** Qw, ** Rw, ** Sw = NULL, *Pw, *Aw, *Bw, *Cw = NULL;

    NL_CVECTOR Dw[2];

    NL_KNOTVECTOR *knu, *knv, *knc = NULL;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curves compatible */

    if( cmp EQ NL_YES )
    {
        error = N_CrvsMakeCompatible( cur, k, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check if curve set is closed */

    if( N_CrvsAreCoincident( cur[0], cur[k], NL_MTOL, SC ) )
    {
        if( cld EQ NL_YES )
        {
            Cw = N_AllocCPt1dArray( k, &SL );

            if( Cw EQ NULL )
                NL_QUIT;

            wp = N_AllocReal1dArray( k, &SL );

            if( wp EQ NULL )
                NL_QUIT;

            for ( i = 1; i < k; i++ )
                wp[i] = 1.0;

            wp[0] = -1.0;
            wd[0] = -1.0;
            I[0] = 0;
            wp[k] = -1.0;
            wd[1] = -1.0;
            I[1] = k;

            closed = NL_YES;
        }
    }

    /* Get indexes and control point data */

    switch( dir )
    {
        case NL_UDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &m, &Pw, &q, &s, &V );
            n = hic;
            nc = k;
            p = deg;
            mc = m;
            r = n + p + 1;

            Rw = N_AllocCPt2dArray( k, m, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= k; i++ )
            {
                N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Pw[j], &Rw[i][j] );
            }

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( k, m, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                nh = k / 2;

                for ( i = 0; i <= k; i++ )
                {
                    l = (nh + i) % k;

                    if( l EQ 0 )
                        l = k;

                    for ( j = 0; j <= m; j++ )
                        N_CopyCPt( Rw[l][j], &Sw[i][j] );
                }
            }
            break;

        case NL_VDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &n, &Pw, &p, &r, &U );
            m = hic;
            nc = n;
            q = deg;
            mc = k;
            s = m + q + 1;

            Rw = N_AllocCPt2dArray( n, k, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( j = 0; j <= k; j++ )
            {
                N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Pw[i], &Rw[i][j] );
            }

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( n, k, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                mh = k / 2;

                for ( j = 0; j <= k; j++ )
                {
                    l = (mh + j) % k;

                    if( l EQ 0 )
                        l = k;

                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Rw[i][l], &Sw[i][j] );
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &US, &VS );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Get parameters */

    u = N_AllocReal1dArray( nc, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mc, &SL );

    if( v EQ NULL )
        NL_QUIT;

    error = N_FitCalcSrfParamValues( (NL_VOID ** )Rw, nc, mc, NL_HPOINT, NL_CHORDLENGTH, u, v );

    if( error EQ NL_YES )
        NL_OUT;

    if( closed EQ NL_YES )
    {
        uc = N_AllocReal1dArray( nc, &SL );

        if( uc EQ NULL )
            NL_QUIT;

        vc = N_AllocReal1dArray( mc, &SL );

        if( vc EQ NULL )
            NL_QUIT;

        error = N_FitCalcSrfParamValues( (NL_VOID ** )Sw, nc, mc, NL_HPOINT, NL_CHORDLENGTH, uc, vc );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute surface control points */

    Aw = N_AllocCPt1dArray( k, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    switch( dir )
    {
        case NL_UDIR:

            error = N_AllocCrvArrays( &curA, n, p, r, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_FitCalcKnotVectorCrvApprox( u, k, n, p, knu );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                knc = N_AllocKnotVectorAndArray( r, &SL );

                if( knc EQ NULL )
                    NL_QUIT;

                error = N_FitCalcKnotVectorCrvApprox( uc, k, n, p, knc );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= k; i++ )
                {
                    N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );
                    N_CopyCPt( Pw[j], &Aw[i] );
                }

                if( closed EQ NL_YES )
                {
                    for ( i = 0; i <= k; i++ )
                        N_CopyCPt( Sw[i][j], &Cw[i] );

                    error = N_FitCrvApproxKnots( (NL_VOID *)Cw, k, NL_HPOINT, uc, knc, n, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, uc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CopyCPt( Dw[1], &Dw[0] );

                    error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)Aw, wp, k, (NL_VOID *)Dw, wd, I, 1, NL_HPOINT, u, knu, n, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvApproxKnots( (NL_VOID *)Aw, k, NL_HPOINT, u, knu, n, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Bw[i], &Qw[i][j] );
            }

            for ( j = 0; j <= s; j++ )
                VS[j] = V[j];
            break;

        case NL_VDIR:

            error = N_AllocCrvArrays( &curA, m, q, s, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_FitCalcKnotVectorCrvApprox( v, k, m, q, knv );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                knc = N_AllocKnotVectorAndArray( s, &SL );

                if( knc EQ NULL )
                    NL_QUIT;

                error = N_FitCalcKnotVectorCrvApprox( vc, k, m, q, knc );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= k; j++ )
                {
                    N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );
                    N_CopyCPt( Pw[i], &Aw[j] );
                }

                if( closed EQ NL_YES )
                {
                    for ( j = 0; j <= k; j++ )
                        N_CopyCPt( Sw[i][j], &Cw[j] );

                    error = N_FitCrvApproxKnots( (NL_VOID *)Cw, k, NL_HPOINT, vc, knc, m, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, vc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CopyCPt( Dw[1], &Dw[0] );

                    error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)Aw, wp, k, (NL_VOID *)Dw, wd, I, 1, NL_HPOINT, v, knv, m, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvApproxKnots( (NL_VOID *)Aw, k, NL_HPOINT, v, knv, m, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Bw[j], &Qw[i][j] );
            }

            for ( i = 0; i <= r; i++ )
                US[i] = U[i];
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Check the weights */

    error = N_SrfWeightsAreValid( sur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfApprox */



/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  a skinned 
     surface  given  a set of  cross-setional  curves assumed at given
     parameters. The knot  vector in  the skinning  direction is  also 
     given. This routine doesn't assume compatibility of these curves, 
     however, if they are, compatibility  computations are skipped. If 
     the output  surface is  initialized  to NULL, memory to store new 
     control points and knots is allocated. A typical calling example:

       NL_CURVE       **cur;
       NL_INDEX       k;
       NL_DEGREE      deg;
       NL_PARAMETER   *t;
       NL_KNOTVECTOR  *knt;
       NL_SURFACE     sur;
       NL_STACKS      SC, SS;
       ...
       (get curves, parameters and knot vector; and choose degree);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrfParams(cur,k,NL_YES,NL_NO,deg,NL_UDIR,t,knt,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE SAME STACK "SC". If the surface weights are outside the 
     range [NL_WMIN,NL_WMAX], the error NL_WEI_ERR is returned even through the 
     NL_SURFACE IS  CONSTRUCTED. In  case the  surface is used, the error 
     can be cleared by N_ErrClear().


   ACCESS:
   
     cur , input  ,  Cross-sectional curves (array of  curve pointers)
     k   , input  ,  Highest index in curve array
     cmp , input  ,  Flag:
                       NL_YES: make curves compatible
                       NL_NO : curves are compatible already
     cld , input  ,  Flag:
                       NL_YES: create  NL_C1  smoothly  closed  surface   if 
                            cur[0]=cur[k]
                       NL_NO : do not force NL_C1 closure  if  cur[0]=cur[k]
     deg , input  ,  Degree of surface in the skinning direction
     dir , input  ,  Flag:
                       NL_UDIR: skin in the  u-direction (all  curves are 
                             v-curves)
                       NL_VDIR: skin in the  v-direction (all  curves are 
                             u-curves)
     t   , input  ,  Parameters; cur[i] is assumed at t[i]
     knt , input  ,  Knot vector in the skinning direction
     sur , output ,  Skinned surface
     SC  , input  ,  curs' memory stack
     SS  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfParams( NL_CURVE ** cur, NL_INDEX k, NL_FLAG cmp, NL_FLAG cld, NL_DEGREE deg, NL_FLAG dir, NL_PARAMETER *t, NL_KNOTVECTOR *knt, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrfParams");

    NL_FLAG closed = NL_NO, error = NL_NO;

    NL_INDEX i, j, l, n, m, nc, mc, nh, mh, mt, r, s;

    NL_DEGREE p, q;

    NL_REAL *U = NULL, *V = NULL, *US, *VS, *T, *uc = NULL, *vc = NULL, *tmp;

    NL_CPOINT ** Qw, ** Sw = NULL, *Pw, *Aw, *Bw, *Cw = NULL;

    NL_CVECTOR *Ws, *We, Dw[2];

    NL_KNOTVECTOR *knc = NULL;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curves compatible */

    if( cmp EQ NL_YES )
    {
        error = N_CrvsMakeCompatible( cur, k, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check if curve set is closed */

    if( N_CrvsAreCoincident( cur[0], cur[k], NL_MTOL, SC ) )
    {
        if( cld EQ NL_YES )
            closed = NL_YES;
    }

    /* Get indexes and control point data */

    N_KnotVectorGetKnots( knt, &mt, &T );

    switch( dir )
    {
        case NL_UDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &m, &Pw, &q, &s, &V );

            if( closed EQ NL_YES )
                n = k + 2;
            else
                n = k;
            nc = k;
            mc = m;
            p = deg;
            r = n + p + 1;

            if( r NEQ mt )
                NL_ERROR( NL_INP_ERR );

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( k, m, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                nh = k / 2;

                for ( i = 0; i <= k; i++ )
                {
                    l = (nh + i) % k;

                    if( l EQ 0 )
                        l = k;

                    N_CrvGetCPtsAndKnots( cur[l], &Pw, &tmp );

                    for ( j = 0; j <= m; j++ )
                        N_CopyCPt( Pw[j], &Sw[i][j] );
                }
            }
            break;

        case NL_VDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &n, &Pw, &p, &r, &U );

            if( closed EQ NL_YES )
                m = k + 2;
            else
                m = k;
            nc = n;
            mc = k;
            q = deg;
            s = m + q + 1;

            if( s NEQ mt )
                NL_ERROR( NL_INP_ERR );

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( n, k, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                mh = k / 2;

                for ( j = 0; j <= k; j++ )
                {
                    l = (mh + j) % k;

                    if( l EQ 0 )
                        l = k;

                    N_CrvGetCPtsAndKnots( cur[l], &Pw, &tmp );

                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Pw[i], &Sw[i][j] );
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &US, &VS );

    Aw = N_AllocCPt1dArray( k, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    if( closed EQ NL_YES )
    {
        Cw = N_AllocCPt1dArray( k, &SL );

        if( Cw EQ NULL )
            NL_QUIT;
    }

    /* Get parameters */

    if( closed EQ NL_YES )
    {
        uc = N_AllocReal1dArray( nc, &SL );

        if( uc EQ NULL )
            NL_QUIT;

        vc = N_AllocReal1dArray( mc, &SL );

        if( vc EQ NULL )
            NL_QUIT;

        error = N_FitCalcSrfParamValues( (NL_VOID ** )Sw, nc, mc, NL_HPOINT, NL_CHORDLENGTH, uc, vc );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute surface control points */

    switch( dir )
    {
        case NL_UDIR:

            error = N_AllocCrvArrays( &curA, n, p, r, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                knc = N_AllocKnotVectorAndArray( r, &SL );

                if( knc EQ NULL )
                    NL_QUIT;

                N_FitCrvCalcKnotVector( uc, k, p, knc );
            }

            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= k; i++ )
                {
                    N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );
                    N_CopyCPt( Pw[j], &Aw[i] );
                }

                if( closed EQ NL_YES )
                {
                    for ( i = 0; i <= k; i++ )
                        N_CopyCPt( Sw[i][j], &Cw[i] );

                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Cw, k, NL_HPOINT, uc, knc, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, uc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    Ws = &Dw[1];
                    We = &Dw[1];

                    N_CrvSetSizeIndices( &curA, n, p, r );
                    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)Aw, k, NL_HPOINT, t, knt, p, (NL_VOID *)Ws, (NL_VOID *)We, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Aw, k, NL_HPOINT, t, knt, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Bw[i], &Qw[i][j] );
            }

            for ( i = 0; i <= r; i++ )
                US[i] = T[i];

            for ( j = 0; j <= s; j++ )
                VS[j] = V[j];
            break;

        case NL_VDIR:

            error = N_AllocCrvArrays( &curA, m, q, s, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                knc = N_AllocKnotVectorAndArray( s, &SL );

                if( knc EQ NULL )
                    NL_QUIT;

                N_FitCrvCalcKnotVector( vc, k, q, knc );
            }

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= k; j++ )
                {
                    N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );
                    N_CopyCPt( Pw[i], &Aw[j] );
                }

                if( closed EQ NL_YES )
                {
                    for ( j = 0; j <= k; j++ )
                        N_CopyCPt( Sw[i][j], &Cw[j] );

                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Cw, k, NL_HPOINT, vc, knc, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, vc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    Ws = &Dw[1];
                    We = &Dw[1];

                    N_CrvSetSizeIndices( &curA, m, q, s );
                    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)Aw, k, NL_HPOINT, t, knt, q, (NL_VOID *)Ws, (NL_VOID *)We, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvInterpGivenParams( (NL_VOID *)Aw, k, NL_HPOINT, t, knt, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Bw[j], &Qw[i][j] );
            }

            for ( i = 0; i <= r; i++ )
                US[i] = U[i];

            for ( j = 0; j <= s; j++ )
                VS[j] = T[j];
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Check the weights */

    error = N_SrfWeightsAreValid( sur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfParams */


/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  a skinned 
     surface  approximating a given set of profile curves. The
     curves  are assumed at given parameter values. The knot vector in
     the  skinning  direction  is  also  input. This  routine does not 
     assume  compatibility  of  these  curves,  however, if they  are,  
     compatibility computations are skipped. If the output  surface is  
     initialized to NULL, memory to store new control points and knots 
     is allocated. A typical calling example is:

       NL_CURVE       **cur;
       NL_INDEX       k, hic;
       NL_DEGREE      deg;
       NL_PARAMETER   *t;
       NL_KNOTVECTOR  *knt;
       NL_SURFACE     sur;
       NL_STACKS      SC, SS;
       ...
       (get curves, t and knt; and choose deg and hic);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrfApproxParams(cur,k,NL_YES,NL_NO,hic,deg,NL_UDIR,t,knt,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE SAME STACK "SC". If the surface weights are outside the 
     range [NL_WMIN,NL_WMAX], the error NL_WEI_ERR is returned even through the 
     NL_SURFACE IS  CONSTRUCTED. In  case the  surface is used, the error 
     can be cleared by N_ErrClear().


   ACCESS:
   
     cur , input  ,  Cross-sectional curves (array of  curve pointers)
     k   , input  ,  Highest index in curve array
     cmp , input  ,  Flag:
                       NL_YES: make curves compatible
                       NL_NO : curves are compatible already
     cld , input  ,  Flag:
                       NL_YES: create  NL_C1  smoothly  closed  surface   if 
                            cur[0]=cur[k]
                       NL_NO : do not force NL_C1 closure  if  cur[0]=cur[k]
     hic , input  ,  Highest index of control  points of approximating
                     surface
     deg , input  ,  Degree of surface in the skinning direction
     dir , input  ,  Flag:
                       NL_UDIR: skin in the  u-direction (all  curves are 
                             v-curves)
                       NL_VDIR: skin in the  v-direction (all  curves are 
                             u-curves)
     t   , input  ,  Parameters; cur[i] is assumed at t[i]
     knt , input  ,  Knot  vector of  skinned  surface in the skinning 
                     direction
     sur , output ,  Skinned approximating surface
     SC  , input  ,  curs' memory stack
     SS  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfApproxParams( NL_CURVE ** cur, NL_INDEX k, NL_FLAG cmp, NL_FLAG cld, NL_INDEX hic, NL_DEGREE deg, NL_FLAG dir, NL_PARAMETER *t, NL_KNOTVECTOR *knt, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrfApproxParams");

    NL_FLAG closed = NL_NO, error = NL_NO;

    NL_INDEX I[2], i, j, l, n, m, nc, mc, r, s, nh, mh, mt;

    NL_DEGREE p, q;

    NL_REAL *U = NULL, *V = NULL, *US, *VS, *T, *uc = NULL, *vc = NULL, *tmp, *wp = NULL, wd[2];

    NL_CPOINT ** Qw, ** Sw = NULL, *Pw, *Aw, *Bw, *Cw = NULL;

    NL_CVECTOR Dw[2];

    NL_KNOTVECTOR *knc = NULL;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curves compatible */

    if( cmp EQ NL_YES )
    {
        error = N_CrvsMakeCompatible( cur, k, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check if curve set is closed */

    if( N_CrvsAreCoincident( cur[0], cur[k], NL_MTOL, SC ) )
    {
        if( cld EQ NL_YES )
        {
            Cw = N_AllocCPt1dArray( k, &SL );

            if( Cw EQ NULL )
                NL_QUIT;

            wp = N_AllocReal1dArray( k, &SL );

            if( wp EQ NULL )
                NL_QUIT;

            for ( i = 1; i < k; i++ )
                wp[i] = 1.0;

            wp[0] = -1.0;
            wd[0] = -1.0;
            I[0] = 0;
            wp[k] = -1.0;
            wd[1] = -1.0;
            I[1] = k;

            closed = NL_YES;
        }
    }

    /* Get indexes and control point data */

    N_KnotVectorGetKnots( knt, &mt, &T );

    switch( dir )
    {
        case NL_UDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &m, &Pw, &q, &s, &V );
            n = hic;
            nc = k;
            p = deg;
            mc = m;
            r = n + p + 1;

            if( r NEQ mt )
                NL_ERROR( NL_INP_ERR );

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( k, m, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                nh = k / 2;

                for ( i = 0; i <= k; i++ )
                {
                    l = (nh + i) % k;

                    if( l EQ 0 )
                        l = k;

                    N_CrvGetCPtsAndKnots( cur[l], &Pw, &tmp );

                    for ( j = 0; j <= m; j++ )
                        N_CopyCPt( Pw[j], &Sw[i][j] );
                }
            }
            break;

        case NL_VDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &n, &Pw, &p, &r, &U );
            m = hic;
            nc = n;
            q = deg;
            mc = k;
            s = m + q + 1;

            if( s NEQ mt )
                NL_ERROR( NL_INP_ERR );

            if( closed EQ NL_YES )
            {
                Sw = N_AllocCPt2dArray( n, k, &SL );

                if( Sw EQ NULL )
                    NL_QUIT;

                mh = k / 2;

                for ( j = 0; j <= k; j++ )
                {
                    l = (mh + j) % k;

                    if( l EQ 0 )
                        l = k;

                    N_CrvGetCPtsAndKnots( cur[l], &Pw, &tmp );

                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Pw[i], &Sw[i][j] );
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &US, &VS );

    /* Get parameters */

    if( closed EQ NL_YES )
    {
        uc = N_AllocReal1dArray( nc, &SL );

        if( uc EQ NULL )
            NL_QUIT;

        vc = N_AllocReal1dArray( mc, &SL );

        if( vc EQ NULL )
            NL_QUIT;

        error = N_FitCalcSrfParamValues( (NL_VOID ** )Sw, nc, mc, NL_HPOINT, NL_CHORDLENGTH, uc, vc );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute surface control points */

    Aw = N_AllocCPt1dArray( k, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    switch( dir )
    {
        case NL_UDIR:

            error = N_AllocCrvArrays( &curA, n, p, r, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                knc = N_AllocKnotVectorAndArray( r, &SL );

                if( knc EQ NULL )
                    NL_QUIT;

                error = N_FitCalcKnotVectorCrvApprox( uc, k, n, p, knc );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= k; i++ )
                {
                    N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );
                    N_CopyCPt( Pw[j], &Aw[i] );
                }

                if( closed EQ NL_YES )
                {
                    for ( i = 0; i <= k; i++ )
                        N_CopyCPt( Sw[i][j], &Cw[i] );

                    error = N_FitCrvApproxKnots( (NL_VOID *)Cw, k, NL_HPOINT, uc, knc, n, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, uc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CopyCPt( Dw[1], &Dw[0] );

                    error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)Aw, wp, k, (NL_VOID *)Dw, wd, I, 1, NL_HPOINT, t, knt, n, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvApproxKnots( (NL_VOID *)Aw, k, NL_HPOINT, t, knt, n, p, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Bw[i], &Qw[i][j] );
            }

            for ( i = 0; i <= r; i++ )
                US[i] = T[i];

            for ( j = 0; j <= s; j++ )
                VS[j] = V[j];
            break;

        case NL_VDIR:

            error = N_AllocCrvArrays( &curA, m, q, s, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( closed EQ NL_YES )
            {
                knc = N_AllocKnotVectorAndArray( s, &SL );

                if( knc EQ NULL )
                    NL_QUIT;

                error = N_FitCalcKnotVectorCrvApprox( vc, k, m, q, knc );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= k; j++ )
                {
                    N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );
                    N_CopyCPt( Pw[i], &Aw[j] );
                }

                if( closed EQ NL_YES )
                {
                    for ( j = 0; j <= k; j++ )
                        N_CopyCPt( Sw[i][j], &Cw[j] );

                    error = N_FitCrvApproxKnots( (NL_VOID *)Cw, k, NL_HPOINT, vc, knc, m, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l = (k + 1) / 2;
                    error = N_CrvDerivsAtKnot( &curA, vc[l], NL_LEFT, 1, Dw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CopyCPt( Dw[1], &Dw[0] );

                    error = N_FitCrvWeightedLstSqKnots( (NL_VOID *)Aw, wp, k, (NL_VOID *)Dw, wd, I, 1, NL_HPOINT, t, knt, m, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    error = N_FitCrvApproxKnots( (NL_VOID *)Aw, k, NL_HPOINT, t, knt, m, q, &curA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &curA, &l, &Bw );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Bw[j], &Qw[i][j] );
            }

            for ( i = 0; i <= r; i++ )
                US[i] = U[i];

            for ( j = 0; j <= s; j++ )
                VS[j] = T[j];
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Check the weights */

    error = N_SrfWeightsAreValid( sur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfApproxParams */



/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  a skinned 
     surface  approximating a given set of profile curves to a 
     specified error bound. The  cross-sectional  curves are made com-
     patible approximately as well. If the output surface is initiali-
     zed  to NULL,  memory to  store new  control points  and knots is 
     allocated. A typical calling example:

       NL_CURVE    **cur;
       NL_INDEX    k;
       NL_DEGREE   ps, pr;
       NL_REAL     EC, ES;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (get curves; choose ps, pr, EC and ES);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrfApproxTol(cur,k,NL_YES,NL_NO,ps,pr,EC,ES,NL_UDIR,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE SAME STACK "SC". If the surface weights are outside the 
     range [NL_WMIN,NL_WMAX], the error NL_WEI_ERR is returned even through the 
     NL_SURFACE IS  CONSTRUCTED. In  case the  surface is used, the error 
     can be cleared by N_ErrClear().


   ACCESS:
   
     cur , input  ,  Cross-sectional curves (array of  curve pointers)
     k   , input  ,  Highest index in curve array
     cmp , input  ,  Flag:
                       NL_YES: make curves compatible
                       NL_NO : curves are compatible already
     cld , input  ,  Flag:
                       NL_YES: create  NL_C1  smoothly  closed  surface   if 
                            cur[0]=cur[k]
                       NL_NO : do not force NL_C1 closure  if  cur[0]=cur[k]
     ps  , input  ,  Start iterations with degree ps. If C0 continuity
                     is  to be  preserved,  ps = 1  should  be chosen. 
                     Otherwise ps = 2 is a good default.
     pr  , input  ,  Required  degree in  the  direction  of  skinning 
                     (pr>=ps)
     EC  , input  ,  Compatibility   tolerance.  The   cross-sectional 
                     curves  must  not  deviate  from  the  compatible 
                     curves more than EC. If precise  compatibility is 
                     required, set EC = 0.0.
     ES  , input  ,  Skinning tolerance; the skinned  surface must not
                     deviate   from  the  compatible   cross-sectional 
                     curves more than ES.
     dir , input  ,  Flag:
                       NL_UDIR: skin in the  u-direction (all  curves are 
                             v-curves)
                       NL_VDIR: skin in the  v-direction (all  curves are 
                             u-curves)
     sur , output ,  Skinned approximating surface
     SC  , input  ,  curs' memory stack
     SS  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfApproxTol( NL_CURVE ** cur, NL_INDEX k, NL_FLAG cmp, NL_FLAG cld, NL_DEGREE ps, NL_DEGREE pr, NL_REAL EC, NL_REAL ES, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrfApproxTol");
    NL_PRIVATE NL_REAL top = 1.0e-05;
    NL_PRIVATE NL_REAL toc = 1.0e-07;

    NL_FLAG apr, reset = NL_NO, closed = NL_NO, error = NL_NO;

    NL_INDEX l, ll, n, m, r, s, ns = 0, ms = 0, rs, ss, nh, mh, rh, sh, nu, mv;

    NL_DEGREE i, j, p, q;

    NL_REAL ** er, *U = NULL, *V = NULL, *US, *VS, *u, *v, *us, *vs, *tmp, ltop, ltoc, d, diag, dev;

    NL_CPOINT ** Rw, *Pw;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check some parameters */

    if( pr GT k )
        NL_ERROR( NL_INP_ERR );

    if( pr LT ps )
        NL_ERROR( NL_INP_ERR );

    if( N_SrfAreArraysNULL( sur ) )
        reset = NL_YES;

    /* Make curves compatible */

    if( cmp EQ NL_YES )
    {
        error = N_CrvsMakeCompatibleApprox( cur, k, EC, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check if curve set is closed */

    if( N_CrvsAreCoincident( cur[0], cur[k], NL_MTOL, SC ) )
    {
        if( cld EQ NL_YES )
            closed = NL_YES;
    }

    /* Adjust tolerances */

    diag = -1.0;

    for ( l = 0; l <= k; l++ )
    {
        error = N_CrvGetBBoxMaxDiagDist( cur[l], &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT diag )
            diag = d;
    }

    ltop = top * diag;
    ltoc = toc;

    /* Get indexes and control point data */

    switch( dir )
    {
        case NL_UDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &m, &Pw, &q, &s, &V );

            if( closed EQ NL_YES )
                n = k + 2;
            else
                n = k;
            p = pr;
            nu = k;
            r = n + p + 1;
            mv = m;

            Rw = N_AllocCPt2dArray( k, m, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= k; i++ )
            {
                N_CrvGetCPtsAndKnots( cur[i], &Pw, &tmp );

                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Pw[j], &Rw[i][j] );
            }
            break;

        case NL_VDIR:

            N_CrvGetCPtsDegreeAndKnots( cur[0], &n, &Pw, &p, &r, &U );

            if( closed EQ NL_YES )
                m = k + 2;
            else
                m = k;
            q = pr;
            nu = n;
            s = m + q + 1;
            mv = k;

            Rw = N_AllocCPt2dArray( n, k, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( j = 0; j <= k; j++ )
            {
                N_CrvGetCPtsAndKnots( cur[j], &Pw, &tmp );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Pw[i], &Rw[i][j] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnotVectors( sur, &knu, &knv );
    N_KnotVectorGetKnots( knu, &r, &US );
    N_KnotVectorGetKnots( knv, &s, &VS );

    /* Get parameters */

    er = N_AllocReal2dArray( nu, mv, &SL );

    if( er EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( nu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mv, &SL );

    if( v EQ NULL )
        NL_QUIT;

    us = N_AllocReal1dArray( nu, &SL );

    if( us EQ NULL )
        NL_QUIT;

    vs = N_AllocReal1dArray( mv, &SL );

    if( vs EQ NULL )
        NL_QUIT;

    error = N_FitCalcSrfParamValues( (NL_VOID ** )Rw, nu, mv, NL_HPOINT, NL_CHORDLENGTH, us, vs );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute skinned surface */

    switch( dir )
    {
        case NL_UDIR:
            for ( i = ps; i <= pr; i++ )
            {
                /* Start with precise skinning */

                for ( l = 0; l <= nu; l++ )
                    u[l] = us[l];

                for ( l = 0; l <= mv; l++ )
                    v[l] = vs[l];

                for ( l = 0; l <= nu; l++ )
                {
                    for ( ll = 0; ll <= mv; ll++ )
                        er[l][ll] = 0.0;
                }

                if( closed EQ NL_YES )
                    N_FitCalcKnotVectorEndDerivs( u, k, i, knu );
                else
                    N_FitCrvCalcKnotVector( u, k, i, knu );

                N_SrfSetSizeIndices( sur, n, m, i, q, n + i + 1, m + q + 1 );

                error = N_CreateSkinSrfParams( cur, k, NL_NO, cld, i, NL_UDIR, u, knu, sur, SC, SS );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = i; j <= pr; j++ )
                {
                    /* Remove as many knots as possible */

                    error = N_FitSrfApproxRemoveKnots( sur, u, v, er, nu, mv, ES, NL_UDIR );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_SrfGetArraySizes( sur, &ns, &ms, &rs, &ss );

                    if( j EQ pr )
                        break;

                    /* Compute new knot vector */

                    nh = ns + 1;
                    rh = rs + 2;

                    if( nh GT k )
                        break;

                    for ( l = 0; l <= j + 1; l++ )
                        US[rh--] = US[rs];

                    for ( l = ns; l >= j + 1; l-- )
                        US[rh--] = US[l];

                    for ( l = 0; l <= j + 1; l++ )
                        US[rh--] = US[0];

                    /* Compute least-squares skinned surface */

                    N_SrfSetSizeIndices( sur, nh, ms, (NL_DEGREE)(j + 1), q, nh + j + 2, ms + q + 1 );

                    error = N_CreateSkinSrfApproxParams( cur, k, NL_NO, cld, nh, (NL_DEGREE)(j + 1), NL_UDIR, u, knu, sur, SC, SS );

                    if( error EQ NL_YES )
                        break;

                    /* Adjust u-parameters */

                    apr = NL_TRUE;

                    for ( l = 1; l < k; l++ )
                    {
                        error = N_GetIsoCrvClosestCrv( sur, cur[l], NL_VDIR, u[l], ltop, ltoc, v, mv, er, l, &u[l], &dev );

                        if( error EQ NL_YES )
                        {
                            apr = NL_FALSE;
                            break;
                        }

                        if( dev GT ES )
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

            for ( l = 0; l <= s; l++ )
                VS[l] = V[l];
            break;

        case NL_VDIR:
            for ( i = ps; i <= pr; i++ )
            {
                /* Start with precise skinning */

                for ( l = 0; l <= nu; l++ )
                    u[l] = us[l];

                for ( l = 0; l <= mv; l++ )
                    v[l] = vs[l];

                for ( l = 0; l <= mv; l++ )
                {
                    for ( ll = 0; ll <= nu; ll++ )
                        er[ll][l] = 0.0;
                }

                if( closed EQ NL_YES )
                    N_FitCalcKnotVectorEndDerivs( v, k, i, knv );
                else
                    N_FitCrvCalcKnotVector( v, k, i, knv );

                N_SrfSetSizeIndices( sur, n, m, p, i, n + p + 1, m + i + 1 );

                error = N_CreateSkinSrfParams( cur, k, NL_NO, cld, i, NL_VDIR, v, knv, sur, SC, SS );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = i; j <= pr; j++ )
                {
                    /* Remove as many knots as possible */

                    error = N_FitSrfApproxRemoveKnots( sur, u, v, er, nu, mv, ES, NL_VDIR );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_SrfGetArraySizes( sur, &ns, &ms, &rs, &ss );

                    if( j EQ pr )
                        break;

                    /* Compute new knot vector */

                    mh = ms + 1;
                    sh = ss + 2;

                    if( mh GT k )
                        break;

                    for ( l = 0; l <= j + 1; l++ )
                        VS[sh--] = VS[ss];

                    for ( l = ms; l >= j + 1; l-- )
                        VS[sh--] = VS[l];

                    for ( l = 0; l <= j + 1; l++ )
                        VS[sh--] = VS[0];

                    /* Compute least-squares skinned surface */

                    N_SrfSetSizeIndices( sur, ns, mh, p, (NL_DEGREE)(j + 1), ns + p + 1, mh + j + 2 );

                    error = N_CreateSkinSrfApproxParams( cur, k, NL_NO, cld, mh, (NL_DEGREE)(j + 1), NL_VDIR, v, knv, sur, SC, SS );

                    if( error EQ NL_YES )
                        break;

                    /* Adjust v-parameters */

                    apr = NL_TRUE;

                    for ( l = 1; l < k; l++ )
                    {
                        error = N_GetIsoCrvClosestCrv( sur, cur[l], NL_UDIR, v[l], ltop, ltoc, u, nu, er, l, &v[l], &dev );

                        if( error EQ NL_YES )
                        {
                            apr = NL_FALSE;
                            break;
                        }

                        if( dev GT ES )
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

            for ( l = 0; l <= r; l++ )
                US[l] = U[l];
            break;
    }

    /* Compact the surface */

    if( reset EQ NL_YES AND( ns LT n OR ms LT m ) )
    {
        error = N_SrfCompress( sur, SS );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check the weights */

    error = N_SrfWeightsAreValid( sur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfApproxTol */


/*******************************************************************//**


   DESCRIPTION:

     This advance surface contruction routine finds the  iso-parametric 
     curve on a NURBS surface closest to  a given curve. The curve must 
     be compatible with u- or v-directional iso-curves of the  surface. 
     The routine updates an error matrix and computes the maximum error
     along the iso-curve. A typical calling example is:

       NL_SURFACE    sur;
       NL_CURVE      cur;
       NL_INDEX      k, l;
       NL_REAL       **er, top, toc, dev;
       NL_PARAMETER  *uv, t0, tc;
       ...
       (define sur & cur; get uv, er, t0, top and toc);
       ...
       N_GetIsoCrvClosestCrv(&sur,&cur,NL_UDIR,t0,top,toc,uv,k,er,l,&tc,&dev);

     If convergence is reached, the parameter tc is returned. Otherwise 
     the  predefined  NL_CON_ERR  (convergence  error)  is  returned. THIS 
     VERSION  COMPUTES AN  APPROXIMATE SOLUTION  USING THE NODES OF THE
     GIVEN NL_CURVE.


   ACCESS:
   
     sur , input  ,  NURBS surface
     cur , input  ,  NURBS curve to be projected
     dir , input  ,  Flag:
                       NL_UDIR: find u-curve closest to cur
                       NL_VDIR: find v-curve closest to cur
     t0  , input  ,  Guess parameter
     top , input  ,  Point coincidence tolerance
     toc , input  ,  Zero cosine tolerance
     uv  , input  ,  Parameters where error is to be measured
     k   , input  ,  Highest index in uv
     er  , in/out ,  Error matrix
     l   , input  ,  Index of row/column to be  updated in error matrix
     tc  , output ,  Parameter corresponding to closest iso-curve
     dev , output ,  Maximum distance betwen cur and  closest iso-curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_GetIsoCrvClosestCrv( NL_SURFACE *sur, NL_CURVE *cur, NL_FLAG dir, NL_PARAMETER t0, NL_REAL top, NL_REAL toc, NL_PARAMETER *uv, NL_INDEX k, NL_REAL ** er, NL_INDEX l, NL_PARAMETER *tc, NL_REAL *dev )
{
    NL_PRIVATE NL_STRING rname = _T("N_GetIsoCrvClosestCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, nc, mc, ns, ms, rs, ss, nt, mt;

    NL_DEGREE pc, pt;

    NL_REAL *U, *t, u, v, usum, vsum, d, fact;

    NL_POINT P, Q;

    NL_CPOINT *Cw, *Tw;

    NL_CURVE curT;

    NL_CFUN cfn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &ns, &ms, &rs, &ss );
    N_CrvGetCPtsDegreeAndKnots( cur, &nc, &Cw, &pc, &mc, &U );
    nt = NL_MAX( ns, ms );
    pt = pc;
    mt = NL_MAX( rs, ss );

    /* Check input curve */

    switch( dir )
    {
        case NL_UDIR:
            if( nc NEQ ns OR mc NEQ rs )
                NL_ERROR( NL_INP_ERR );
            break;

        case NL_VDIR:
            if( nc NEQ ms OR mc NEQ ss )
                NL_ERROR( NL_INP_ERR );
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Allocate memory */

    t = N_AllocReal1dArray( nc, &SL );

    if( t EQ NULL )
        NL_QUIT;

    error = N_AllocCrvArrays( &curT, nt, pt, mt, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the nodes */

    t[0] = U[0];
    t[nc] = U[mc];
    fact = 1.0 / pc;

    for ( i = 1; i < nc; i++ )
    {
        d = 0.0;

        for ( j = 1; j <= pc; j++ )
            d += U[i + j];
        t[i] = fact * d;
    }

    /* Find parameter of closest iso-curve */

    switch( dir )
    {
        case NL_UDIR:

            vsum = 0.0;

            for ( i = 0; i <= nc; i++ )
            {
                error = N_SrfExtractIsoCrv( sur, t[i], NL_VDIR, &curT, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_CrvEval( cur, t[i], NL_LEFT, &P );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_CrvClosestPt( &curT, P, t0, top, toc, &v, &Q );

                if( error EQ NL_YES )
                    NL_OUT;

                vsum += v;
            }
            v = vsum / ((NL_REAL)nc + 1.0);

            N_CrvSetSizeIndices( &curT, nc, pc, mc );
            error = N_SrfExtractIsoCrv( sur, v, NL_UDIR, &curT, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            *tc = v;
            break;

        case NL_VDIR:

            usum = 0.0;

            for ( j = 0; j <= nc; j++ )
            {
                error = N_SrfExtractIsoCrv( sur, t[j], NL_UDIR, &curT, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_CrvEval( cur, t[j], NL_LEFT, &P );

                if( error EQ NL_YES )
                    NL_OUT;

                error = N_CrvClosestPt( &curT, P, t0, top, toc, &u, &Q );

                if( error EQ NL_YES )
                    NL_OUT;

                usum += u;
            }
            u = usum / ((NL_REAL)nc + 1.0);

            N_CrvSetSizeIndices( &curT, nc, pc, mc );
            error = N_SrfExtractIsoCrv( sur, u, NL_VDIR, &curT, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            *tc = u;
            break;
    }

    /* Update error and compute maximum deviation */

    N_CrvGetCPtsAndKnots( &curT, &Tw, &U );

    for ( i = 0; i <= nc; i++ )
        N_DistCptCptHomo( Cw[i], Tw[i], &t[i] );

    error = N_CFuncFromKnots( &cfn, t, nc, pc, U, mc, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    *dev = -1.0;

    switch( dir )
    {
        case NL_UDIR:
            for ( i = 0; i <= k; i++ )
            {
                error = N_CFuncEval( &cfn, uv[i], NL_LEFT, &er[i][l] );

                if( error EQ NL_YES )
                    NL_OUT;

                if( er[i][l]GT *dev )
                    *dev = er[i][l];
            }
            break;

        case NL_VDIR:
            for ( j = 0; j <= k; j++ )
            {
                error = N_CFuncEval( &cfn, uv[j], NL_LEFT, &er[l][j] );

                if( error EQ NL_YES )
                    NL_OUT;

                if( er[l][j]GT *dev )
                    *dev = er[l][j];
            }
            break;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_GetIsoCrvClosestCrv */


/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  a skinned 
     surface  given a set of  profile curves positioned along a
     spine  curve at  given parameter  values. This  routine  does not 
     assume  compatibility  of  these  curves,  however, if  they are,
     compatibility  computations are skipped. If the output surface is 
     initialized to NULL, memory to store new control points and knots  
     is allocated. A typical calling example is:

       NL_CURVE      **pProfiles, pSpine;
       NL_INDEX      iNumProfiles, iSubDivLevel;
       NL_PARAMETER  *pParams
       NL_VECTOR     sZ0;
       NL_DEGREE     iDegree;
       NL_SURFACE    sur;
       NL_STACKS     pCrvStack, pSrfStack;
       ...
       (get curves, spine, parameters, and choose iDegree, sZ0 and iSubDivLevel);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSpine(pProfiles,iNumProfiles,NL_YES,&pSpine,ts,NL_NO,Z0,iSubDivLevel,NL_ENDDER,iDegree,NL_UDIR,&sur,
                &pCrvStack,&pSrfStack);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE SAME STACK "pCrvStack". The cross-sectional curves are aligned 
     to the spine by mapping the global  coordinate system, where they
     are defined, to  the  spine's local  systems computed recursively 
     using sZ0  and  the tangents at each point  where instances of the 
     section curves are  placed. If placements  of the section  curves 
     are not satisfactory, they can be modified by editing them in the 
     global coordinate system. If  the surface weights are outside the 
     range [NL_WMIN,NL_WMAX], the error NL_WEI_ERR is returned even through the 
     NL_SURFACE IS  CONSTRUCTED. In  case the  surface is used, the error 
     can be cleared by N_ErrClear().


   ACCESS:
   
     pProfiles       , input  ,  Cross-sectional curves (array of  curve pointers)
     iNumProfiles    , input  ,  Highest index in pProfiles and pParams arrays
     bMakeCompatible , input  ,  Flag:
                                  NL_YES: make curves compatible
                                  NL_NO : curves are compatible already
     pSpine          , input  ,  Spine curve
     pSpineParams    , input  ,  Parameters; pProfiles[i] is positioned to  the spine at
                                 a point computed at pSpineParams[i] on the spine
     bAlignProfiles  , input  ,  Flag:
                                  NL_YES: align cross-sectional curves
                                  NL_NO : cross-sectional curves are already aligned
     sZ0             , input  ,  Start local z-axis of the spine
                                 Used only if bAlignProfiles is true.
     iSubDivLevel    , input  ,  Subdivision level; the higher the level, the more Z
                                 vectors are computed to  obtain a smoothly moving
                                 local frame  along the spine. If  iNumProfiles is large, iSubDivLevel
                                 should be small (1 or 2). If iNumProfiles is small, a larger
                                 iSubDivLevel, e.g. 5-10, is recommended.
     eDerivFlag      , input  ,  Flag:
                                  NL_NODER : no  derivatives  required; spine  curve 
                                             used only to position section-curves
                                  NL_ENDDER: only end  derivatives  are used  to fit 
                                             control points of section-curves
                                  NL_ALLDER: derivatives  are   specified  at   each 
                                             control point of each section-curve
     iDegree         , input  ,  Degree of surface in the skinning direction. MUST
                                 BE 2 OR 3 IF ALL OR ENDDIR DERIVATIVES ARE SPECIFIED
     eUVDir          , input  ,  Flag:
                                  NL_UDIR: skin in the  u-direction (all  curves are v-curves)
                                  NL_VDIR: skin in the  v-direction (all  curves are u-curves)
     pNewSurf        , output ,  Skinned surface
     pCrvStack       , input  ,  pProfiles' memory stack
     pSrfStack       , input  ,  pNewSurf's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSpine(
        NL_CURVE ** pProfiles, NL_INDEX iNumProfiles,
        NL_FLAG bMakeCompatible,
        NL_CURVE *pSpine,
        NL_PARAMETER *pSpineParams,
        NL_FLAG bAlignProfiles,
        NL_VECTOR sZ0,
        NL_INDEX iSubDivLevel,
        NL_FLAG eDerivFlag,
        NL_DEGREE iDegree,
        NL_FLAG eUVDir,
        NL_SURFACE *pNewSurf,
        NL_STACKS *pCrvStack, NL_STACKS *pSrfStack
    )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSpine");

    NL_FLAG bClosed = NL_NO, bRational = NL_NO, error = NL_NO;

    NL_INDEX ii, jj, ll, iNPtsU, iNPtsV, iNumSrfPtsU, iNumSrfPtsV, iNumKnotsU, iNumKnotsV;

    NL_DEGREE iDegU, iDegV, iDegFunc;

    NL_REAL *pProfKnotsU = NULL, *pProfKnotsV = NULL;
    NL_REAL *pNewSurfKnotsPtrU, *pNewSurfKnotsPtrV;
    NL_REAL *pInterpParamsU, *pInterpParamsV, *tmp;
    NL_REAL *pSpineDists = NULL, *pProfDists = NULL, *pAlignParams, *pWeights = NULL, pWeightValDeriv[2];
    NL_REAL  dIncr, dSpineDist1 = 0.0, dSpineDistK = 0.0, dProfDist1, dProfDistK, dFactor;

    NL_POINT *pSpinePtDer = NULL, *pSpDerScaled = NULL, T[2];
    NL_POINT  sSpPtDer0 = { 0,0,0 }, sSpPtDer1, sSpPtDerKm1, sSpPtDerK = { 0,0,0 };
    NL_POINT  sSpDerScaled0, sSpDerScaledK, sOrigin;

    NL_CPOINT ** pNewSurfCPtsPtr, ** pAllProfCPts, *pThisProfCPts, *pProfileCPts, *pSkinCurveCpts, *pRatDerivs = NULL, pRatDerivs0, pRatDerivsK;

    NL_VECTOR *pBVecs, sXAxis, sYAxis, sZAxis;

    NL_CVECTOR *pRatDerivStart, *pRatDerivEnd;

    NL_KNOTVECTOR *pKnotVecU, *pKnotVecV;

    NL_RMATRIX sAlignMatrix;

    NL_CURVE sSkinningCurve;

    NL_CFUN fWeightFcn;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check some parameters */

    switch( eDerivFlag )
    {
        case NL_NODER:
            if( iDegree GT iNumProfiles )
                NL_ERROR( NL_INP_ERR );
            break;

        case NL_ENDDER:
            if( iDegree LT 2 OR iDegree GT iNumProfiles + 2 )
                NL_ERROR( NL_INP_ERR );
            break;

        case NL_ALLDER:
            if( iDegree LT 2 OR iDegree GT 3 )
                NL_ERROR( NL_INP_ERR );
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Make curves compatible */

    if( bMakeCompatible EQ NL_YES )
    {
        error = N_CrvsMakeCompatible( pProfiles, iNumProfiles, pCrvStack );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* See if aligment is needed */

    if( bAlignProfiles EQ NL_YES )
    {
        error = N_SetRealMatrix( &sAlignMatrix, 3, 3, NL_MT_FULL, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( iSubDivLevel LE 0 )
            iSubDivLevel = 1;

        ll = iNumProfiles * (iSubDivLevel + 1);

        pAlignParams = N_AllocReal1dArray( ll, &SL );

        if( pAlignParams EQ NULL )
            NL_QUIT;

        pBVecs = N_AllocPt1dArray( ll, &SL );

        if( pBVecs EQ NULL )
            NL_QUIT;

        /* Set up pAlignParams, which are pSpineParams with iSubDivLevel interpolations */
        for ( ii = 0; ii < iNumProfiles; ii++ )
        {
            dIncr = (pSpineParams[ii + 1] - pSpineParams[ii]) / ((NL_REAL)iSubDivLevel + 1.0);
            ll = ii * (iSubDivLevel + 1);

            for ( jj = 0; jj <= iSubDivLevel; jj++ )
            {
                pAlignParams[ll + jj] = pSpineParams[ii] + jj * dIncr;
            }
        }

        ll = iNumProfiles * (iSubDivLevel + 1);
        pAlignParams[ll] = pSpineParams[iNumProfiles];

        /* Done setting up pAlignParams */

        error = N_CreateBVectors( pSpine, sZ0, pAlignParams, ll, pBVecs );

        if( error EQ NL_YES )
            NL_OUT;

        for ( ii = 0; ii <= iNumProfiles; ii++ )
        {
            error = N_CrvEvalTangent( pSpine, pSpineParams[ii], NL_LEFT, &sOrigin, &sXAxis );

            if( error EQ NL_YES )
                NL_OUT;

            ll = ii * (iSubDivLevel + 1);
            N_CopyPt( pBVecs[ll], &sZAxis );
            N_VectorCross( sZAxis, sXAxis, &sYAxis );

            error = N_CreateTransformMatrixFromAxes( NL_ZERO, NL_UNITX, NL_UNITY, NL_UNITZ, sOrigin, sXAxis, sYAxis, sZAxis, &sAlignMatrix, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvTransform( pProfiles[ii], &sAlignMatrix );
        }
    } /* end if bAlignProfiles */


    /* Do simple skinning if no derivative is specified */

    if( eDerivFlag EQ NL_NODER )
    {
        error = N_CreateSkinSrf( pProfiles, iNumProfiles, NL_NO, NL_NO, iDegree, eUVDir, 0.0, pNewSurf, pCrvStack, pSrfStack );

        NL_OUT;
    }


    /* Check if curve set is closed or rational */

    if( N_CrvsAreCoincident( pProfiles[0], pProfiles[iNumProfiles], NL_MTOL, pCrvStack ) )
        bClosed = NL_YES;

    if( N_IsCrvRat( pProfiles[0] ) )
        bRational = NL_YES;

    /* Get indexes and control point data */

    switch( eUVDir )
    {
        case NL_UDIR:

            N_CrvGetCPtsDegreeAndKnots( pProfiles[0], &iNPtsV, &pThisProfCPts, &iDegV, &iNumKnotsV, &pProfKnotsV );

            if( eDerivFlag EQ NL_ENDDER )
                iNPtsU = iNumProfiles + 2;
            else
                iNPtsU = 2 * iNumProfiles + 1;
            iNumSrfPtsU = iNumProfiles;
            iNumSrfPtsV = iNPtsV;
            iDegU = iDegree;
            iNumKnotsU = iNPtsU + iDegU + 1;

            pAllProfCPts = N_AllocCPt2dArray( iNumProfiles, iNPtsV, &SL );

            if( pAllProfCPts EQ NULL )
                NL_QUIT;

            for ( ii = 0; ii <= iNumProfiles; ii++ )
            {
                N_CrvGetCPtsAndKnots( pProfiles[ii], &pThisProfCPts, &tmp );

                for ( jj = 0; jj <= iNPtsV; jj++ )
                    N_CopyCPt( pThisProfCPts[jj], &pAllProfCPts[ii][jj] );
            }
            break;

        case NL_VDIR:

            N_CrvGetCPtsDegreeAndKnots( pProfiles[0], &iNPtsU, &pThisProfCPts, &iDegU, &iNumKnotsU, &pProfKnotsU );

            if( eDerivFlag EQ NL_ENDDER )
                iNPtsV = iNumProfiles + 2;
            else
                iNPtsV = 2 * iNumProfiles + 1;
            iNumSrfPtsU = iNPtsU;
            iNumSrfPtsV = iNumProfiles;
            iDegV = iDegree;
            iNumKnotsV = iNPtsV + iDegV + 1;

            pAllProfCPts = N_AllocCPt2dArray( iNPtsU, iNumProfiles, &SL );

            if( pAllProfCPts EQ NULL )
                NL_QUIT;

            for ( jj = 0; jj <= iNumProfiles; jj++ )
            {
                N_CrvGetCPtsAndKnots( pProfiles[jj], &pThisProfCPts, &tmp );

                for ( ii = 0; ii <= iNPtsU; ii++ )
                    N_CopyCPt( pThisProfCPts[ii], &pAllProfCPts[ii][jj] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( pNewSurf, iNPtsU, iNPtsV, iDegU, iDegV, iNumKnotsU, iNumKnotsV, rname, pSrfStack );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( pNewSurf, &pNewSurfCPtsPtr, &pNewSurfKnotsPtrU, &pNewSurfKnotsPtrV );
    N_SrfGetKnotVectors ( pNewSurf, &pKnotVecU, &pKnotVecV );

    /* Get parameters */

    pInterpParamsU = N_AllocReal1dArray( iNumSrfPtsU, &SL );

    if( pInterpParamsU EQ NULL )
        NL_QUIT;

    pInterpParamsV = N_AllocReal1dArray( iNumSrfPtsV, &SL );

    if( pInterpParamsV EQ NULL )
        NL_QUIT;

    error = N_FitCalcSrfParamValues( (NL_VOID ** )pAllProfCPts, iNumSrfPtsU, iNumSrfPtsV, NL_HPOINT, NL_CHORDLENGTH, pInterpParamsU, pInterpParamsV );

    if( error EQ NL_YES )
        NL_OUT;

    /* Allocate local memory */

    pProfileCPts = N_AllocCPt1dArray( iNumProfiles, &SL );

    if( pProfileCPts EQ NULL )
        NL_QUIT;

    if( bRational EQ NL_YES )
    {
        pWeights = N_AllocReal1dArray( iNumProfiles, &SL );

        if( pWeights EQ NULL )
            NL_QUIT;
    }

    if( eDerivFlag EQ NL_ALLDER )
    {
        pSpinePtDer = N_AllocPt1dArray( iNumProfiles, &SL );

        if( pSpinePtDer EQ NULL )
            NL_QUIT;

        pSpDerScaled = N_AllocPt1dArray( iNumProfiles, &SL );

        if( pSpDerScaled EQ NULL )
            NL_QUIT;

        pRatDerivs = N_AllocCPt1dArray( iNumProfiles, &SL );

        if( pRatDerivs EQ NULL )
            NL_QUIT;

        pSpineDists = N_AllocReal1dArray( iNumProfiles + 1, &SL );

        if( pSpineDists EQ NULL )
            NL_QUIT;

        pProfDists = N_AllocReal1dArray( iNumProfiles + 1, &SL );

        if( pProfDists EQ NULL )
            NL_QUIT;
    }

    /* Compute distances and derivatives along the spine */

    if( eDerivFlag EQ NL_ALLDER )
    {
        error = N_CrvEval( pSpine, pSpineParams[0], NL_LEFT, &pSpinePtDer[0] );

        if( error EQ NL_YES )
            NL_OUT;

        for ( ii = 1; ii <= iNumProfiles; ii++ )
        {
            error = N_CrvEval( pSpine, pSpineParams[ii], NL_LEFT, &pSpinePtDer[ii] );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( pSpinePtDer[ii], pSpinePtDer[ii - 1], &pSpineDists[ii] );
        }

        if( bClosed EQ NL_NO )
        {
            pSpineDists[ 0 ]              = 0.0;
            pSpineDists[ iNumProfiles+1 ] = 0.0;
        }
        else
        {
            pSpineDists[ 0 ]              = pSpineDists[iNumProfiles];
            pSpineDists[ iNumProfiles+1 ] = pSpineDists[1];
        }

        /* Overwrite pSpinePtDer[] with spine tangents at profiles */
        for ( ii = 0; ii <= iNumProfiles; ii++ )
        {
            error = N_CrvDerivs( pSpine, pSpineParams[ii], NL_LEFT, 1, T );

            if( error EQ NL_YES )
                NL_OUT;

            N_CopyPt( T[1], &pSpinePtDer[ii] );
        }
    }
    else
    {
        error = N_CrvEval( pSpine, pSpineParams[0], NL_LEFT, &sSpPtDer0 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( pSpine, pSpineParams[1], NL_LEFT, &sSpPtDer1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( pSpine, pSpineParams[iNumProfiles - 1], NL_LEFT, &sSpPtDerKm1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( pSpine, pSpineParams[iNumProfiles], NL_LEFT, &sSpPtDerK );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( sSpPtDer1, sSpPtDer0,   &dSpineDist1 );
        N_DistPtPt( sSpPtDerK, sSpPtDerKm1, &dSpineDistK );

        error = N_CrvDerivs( pSpine, pSpineParams[0], NL_LEFT, 1, T );

        if( error EQ NL_YES )
            NL_OUT;

        N_CopyPt( T[1], &sSpPtDer0 );

        error = N_CrvDerivs( pSpine, pSpineParams[iNumProfiles], NL_LEFT, 1, T );

        if( error EQ NL_YES )
            NL_OUT;

        N_CopyPt( T[1], &sSpPtDerK );
    }

    /* Compute surface control points */

    switch( eUVDir )
    {
        case NL_UDIR:
        {
            error = N_AllocCrvArrays( &sSkinningCurve, iNPtsU, iDegU, iNumKnotsU, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_AllocCFuncArrays( &fWeightFcn, iNumProfiles, iDegU, iNumProfiles + iDegU + 1, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( eDerivFlag EQ NL_ALLDER )
                N_FitCalcKnotsDerivs( pInterpParamsU, iNumProfiles, iDegU, pKnotVecU );
            else
                N_FitCalcKnotVectorEndDerivs( pInterpParamsU, iNumProfiles, iDegU, pKnotVecU );

            for ( jj = 0; jj <= iNPtsV; jj++ )
            {
                /* Extract control points for interpolation */

                for ( ii = 0; ii <= iNumProfiles; ii++ )
                {
                    N_CrvGetCPtsAndKnots( pProfiles[ii], &pThisProfCPts, &tmp );
                    N_CopyCPt( pThisProfCPts[jj], &pProfileCPts[ii] );

                    if( bRational EQ NL_YES )
                        N_CPtGetW( pProfileCPts[ii], &pWeights[ii] );
                }

                /* Get weight function */

                if( bRational EQ NL_YES )
                {
                    iDegFunc = NL_MIN( (NL_DEGREE)iNumProfiles, iDegU );
                    error = N_FitFuncInterpGivenParams( pWeights, iNumProfiles, iDegFunc, pInterpParamsU, &fWeightFcn, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                /* Get derivatives and interpolate */

                if( eDerivFlag EQ NL_ALLDER )
                {
                    for ( ii = 1; ii <= iNumProfiles; ii++ )
                        N_DistCptCpt( pProfileCPts[ii], pProfileCPts[ii - 1], &pProfDists[ii] );

                    if( bClosed EQ NL_NO )
                    {
                        pProfDists[0] = 0.0;
                        pProfDists[iNumProfiles + 1] = 0.0;
                    }
                    else
                    {
                        pProfDists[0] = pProfDists[iNumProfiles];
                        pProfDists[iNumProfiles + 1] = pProfDists[1];
                    }

                    for ( ii = 0; ii <= iNumProfiles; ii++ )
                    {
                        dFactor = ( pProfDists[ii + 1] + pProfDists[ii] ) / ( pSpineDists[ii + 1] + pSpineDists[ii] );
                        /* pSpinePtDer[] here is the spine tangent at this profile. */
                        N_ScalePt( dFactor, pSpinePtDer[ii], &pSpDerScaled[ii] );

                        if( bRational EQ NL_YES )
                        {
                            error = N_CFuncDerivs( &fWeightFcn, pInterpParamsU[ii], NL_LEFT, 1, pWeightValDeriv );

                            if( error EQ NL_YES )
                                NL_OUT;

                            N_CPtRatDeriv( pProfileCPts[ii], pSpDerScaled[ii], pWeightValDeriv[1], &pRatDerivs[ii] );
                        }
                        else
                        {
                            N_PtToCPt( pSpDerScaled[ii], &pRatDerivs[ii] );
                        }
                    }

                    error = N_FitCrvFirstDerivAndKnots( (NL_VOID *)pProfileCPts, (NL_VOID *)pRatDerivs, iNumProfiles, NL_HPOINT, pInterpParamsU, pKnotVecU, iDegU, &sSkinningCurve, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    N_DistCptCpt( pProfileCPts[1], pProfileCPts[0], &dProfDist1 );
                    N_DistCptCpt( pProfileCPts[iNumProfiles], pProfileCPts[iNumProfiles - 1], &dProfDistK );

                    if( bClosed EQ NL_NO )
                    {
                        dFactor = dProfDist1 / dSpineDist1;
                        N_ScalePt( dFactor, sSpPtDer0, &sSpDerScaled0 );
                        dFactor = dProfDistK / dSpineDistK;
                        N_ScalePt( dFactor, sSpPtDerK, &sSpDerScaledK );
                    }
                    else
                    {
                        dFactor = (dProfDist1 + dProfDistK) / (dSpineDist1 + dSpineDistK);
                        N_ScalePt( dFactor, sSpPtDer0, &sSpDerScaled0 );
                        N_ScalePt( dFactor, sSpPtDerK, &sSpDerScaledK );
                    }

                    if( bRational EQ NL_YES )
                    {
                        error = N_CFuncDerivs( &fWeightFcn, pInterpParamsU[0], NL_LEFT, 1, pWeightValDeriv );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_CPtRatDeriv( pProfileCPts[0], sSpDerScaled0, pWeightValDeriv[1], &pRatDerivs0 );

                        error = N_CFuncDerivs( &fWeightFcn, pInterpParamsU[iNumProfiles], NL_LEFT, 1, pWeightValDeriv );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_CPtRatDeriv( pProfileCPts[iNumProfiles], sSpDerScaledK, pWeightValDeriv[1], &pRatDerivsK );
                    }
                    else
                    {
                        N_PtToCPt( sSpDerScaled0, &pRatDerivs0 );
                        N_PtToCPt( sSpDerScaledK, &pRatDerivsK );
                    }

                    pRatDerivStart = &pRatDerivs0;
                    pRatDerivEnd = &pRatDerivsK;

                    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)pProfileCPts, iNumProfiles, NL_HPOINT, pInterpParamsU, pKnotVecU, iDegU, (NL_VOID *)pRatDerivStart, (NL_VOID *)pRatDerivEnd, &sSkinningCurve, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &sSkinningCurve, &ll, &pSkinCurveCpts );

                for ( ii = 0; ii <= iNPtsU; ii++ )
                    N_CopyCPt( pSkinCurveCpts[ii], &pNewSurfCPtsPtr[ii][jj] );

            } /* end for all iNPtsV */

            for ( jj = 0; jj <= iNumKnotsV; jj++ )
                pNewSurfKnotsPtrV[jj] = pProfKnotsV[jj];
            break;

            }  /* end case NL_UDIR: */

        case NL_VDIR:
            {
            error = N_AllocCrvArrays( &sSkinningCurve, iNPtsV, iDegV, iNumKnotsV, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_AllocCFuncArrays( &fWeightFcn, iNumProfiles, iDegV, iNumProfiles + iDegV + 1, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( eDerivFlag EQ NL_ALLDER )
                N_FitCalcKnotsDerivs( pInterpParamsV, iNumProfiles, iDegV, pKnotVecV );
            else
                N_FitCalcKnotVectorEndDerivs( pInterpParamsV, iNumProfiles, iDegV, pKnotVecV );

            for ( ii = 0; ii <= iNPtsU; ii++ )
            {
                /* Extract control points for interpolation */

                for ( jj = 0; jj <= iNumProfiles; jj++ )
                {
                    N_CrvGetCPtsAndKnots( pProfiles[jj], &pThisProfCPts, &tmp );
                    N_CopyCPt( pThisProfCPts[ii], &pProfileCPts[jj] );

                    if( bRational EQ NL_YES )
                        N_CPtGetW( pProfileCPts[jj], &pWeights[jj] );
                }

                /* Get weight function */

                if( bRational EQ NL_YES )
                {
                    iDegFunc = NL_MIN( (NL_DEGREE)iNumProfiles, iDegV );
                    error = N_FitFuncInterpGivenParams( pWeights, iNumProfiles, iDegFunc, pInterpParamsV, &fWeightFcn, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                /* Get derivatives and interpolate */

                if( eDerivFlag EQ NL_ALLDER )
                {
                    for ( jj = 1; jj <= iNumProfiles; jj++ )
                        N_DistCptCpt( pProfileCPts[jj], pProfileCPts[jj - 1], &pProfDists[jj] );

                    if( bClosed EQ NL_NO )
                    {
                        pProfDists[0] = 0.0;
                        pProfDists[iNumProfiles + 1] = 0.0;
                    }
                    else
                    {
                        pProfDists[0] = pProfDists[iNumProfiles];
                        pProfDists[iNumProfiles + 1] = pProfDists[1];
                    }

                    for ( jj = 0; jj <= iNumProfiles; jj++ )
                    {
                        dFactor = ( pProfDists[jj + 1] + pProfDists[jj] ) / ( pSpineDists[jj + 1] + pSpineDists[jj] );
                        /* pSpinePtDer[] here is the spine tangent at this profile. */
                        N_ScalePt( dFactor, pSpinePtDer[jj], &pSpDerScaled[jj] );

                        if( bRational EQ NL_YES )
                        {
                            error = N_CFuncDerivs( &fWeightFcn, pInterpParamsV[jj], NL_LEFT, 1, pWeightValDeriv );

                            if( error EQ NL_YES )
                                NL_OUT;

                            N_CPtRatDeriv( pProfileCPts[jj], pSpDerScaled[jj], pWeightValDeriv[1], &pRatDerivs[jj] );
                        }
                        else
                        {
                            N_PtToCPt( pSpDerScaled[jj], &pRatDerivs[jj] );
                        }
                    }

                    error = N_FitCrvFirstDerivAndKnots( (NL_VOID *)pProfileCPts, (NL_VOID *)pRatDerivs, iNumProfiles, NL_HPOINT, pInterpParamsV, pKnotVecV, iDegV, &sSkinningCurve, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    N_DistCptCpt( pProfileCPts[1], pProfileCPts[0], &dProfDist1 );
                    N_DistCptCpt( pProfileCPts[iNumProfiles], pProfileCPts[iNumProfiles - 1], &dProfDistK );

                    if( bClosed EQ NL_NO )
                    {
                        dFactor = dProfDist1 / dSpineDist1;
                        N_ScalePt( dFactor, sSpPtDer0, &sSpDerScaled0 );
                        dFactor = dProfDistK / dSpineDistK;
                        N_ScalePt( dFactor, sSpPtDerK, &sSpDerScaledK );
                    }
                    else
                    {
                        dFactor = (dProfDist1 + dProfDistK) / (dSpineDist1 + dSpineDistK);
                        N_ScalePt( dFactor, sSpPtDer0, &sSpDerScaled0 );
                        N_ScalePt( dFactor, sSpPtDerK, &sSpDerScaledK );
                    }

                    if( bRational EQ NL_YES )
                    {
                        error = N_CFuncDerivs( &fWeightFcn, pInterpParamsV[0], NL_LEFT, 1, pWeightValDeriv );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_CPtRatDeriv( pProfileCPts[0], sSpDerScaled0, pWeightValDeriv[1], &pRatDerivs0 );

                        error = N_CFuncDerivs( &fWeightFcn, pInterpParamsV[iNumProfiles], NL_LEFT, 1, pWeightValDeriv );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_CPtRatDeriv( pProfileCPts[iNumProfiles], sSpDerScaledK, pWeightValDeriv[1], &pRatDerivsK );
                    }
                    else
                    {
                        N_PtToCPt( sSpDerScaled0, &pRatDerivs0 );
                        N_PtToCPt( sSpDerScaledK, &pRatDerivsK );
                    }

                    pRatDerivStart = &pRatDerivs0;
                    pRatDerivEnd = &pRatDerivsK;

                    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)pProfileCPts, iNumProfiles, NL_HPOINT, pInterpParamsV, pKnotVecV, iDegV, (NL_VOID *)pRatDerivStart, (NL_VOID *)pRatDerivEnd, &sSkinningCurve, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvGetCPts( &sSkinningCurve, &ll, &pSkinCurveCpts );

                for ( jj = 0; jj <= iNPtsV; jj++ )
                    N_CopyCPt( pSkinCurveCpts[jj], &pNewSurfCPtsPtr[ii][jj] );

            } /* end for all iNPtsU */

            for ( ii = 0; ii <= iNumKnotsU; ii++ )
                pNewSurfKnotsPtrU[ii] = pProfKnotsU[ii];
            break;

        }  /* end case NL_VDIR: */

    } /* end switch on eUVDir */

    /* Check the weights */

    error = N_SrfWeightsAreValid( pNewSurf, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_CreateSkinSpine */



/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine creates a  translational
     sweep  surface  given  a trajectory  and a  section  curve. If the 
     output surface is initialized to NULL, memory to store new control 
     points and knots is allocated. A typical calling example is:

       NL_CURVE    curT, curS;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get curT, curS);
       ...
       N_SrfInitArrays(&sur);
       N_CreateTransSweepSrf(&curT,&curS,&sur,&SG);

     If  memory is  available, sur  is not  initialized and the routine
     assumes that  memory allocation has been done. However, it  checks  
     for the proper  amount by looking at the highest  indexes in sur's 
     knot vector  and  polygon  objects. 


   ACCESS:
   
     curT , input  ,  Trajectory curve (v-curve)
     curS , input  ,  Section curve (u-curve)
     sur  , output ,  Translational sweep surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTransSweepSrf( NL_CURVE *curT, NL_CURVE *curS, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTransSweepSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UT, *US, wt, ws;

    NL_POINT P, T, S;

    NL_CPOINT ** Pw, *Tw, *Sw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations */

    N_CrvGetCPtsDegreeAndKnots( curS, &n, &Sw, &p, &r, &US );
    N_CrvGetCPtsDegreeAndKnots( curT, &m, &Tw, &q, &s, &UT );

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Compute control points */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Sw[i], &S );
        N_CPtGetW( Sw[i], &ws );

        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Tw[j], &T );
            N_CPtGetW( Tw[j], &wt );

            N_Sum2Pts( S, T, &P );

            if( ws EQ NL_NOW AND wt EQ NL_NOW )
                N_PtToCPt( P, &Pw[i][j] );

            else if( ws EQ NL_NOW AND wt NEQ NL_NOW )
                N_Weight( P, wt, &Pw[i][j] );

            else if( ws NEQ NL_NOW AND wt EQ NL_NOW )
                N_Weight( P, ws, &Pw[i][j] );

            else if( ws NEQ NL_NOW AND wt NEQ NL_NOW )
                N_Weight( P, ws *wt, &Pw[i][j] );
        }
    }

    /* Compute the knot vectors */

    for ( i = 0; i <= r; i++ )
        U[i] = US[i];

    for ( j = 0; j <= s; j++ )
        V[j] = UT[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateTransSweepSrf */


/*******************************************************************//**


   DESCRIPTION:

     This advance surface construction routine  computes the "B" vectors
     of a  trajectory curve at given  parameter  values. If a "B" vector
     cannot be  computed  at a certain parameter  value, an error number 
     is returned. In this case it is recommended  to restart the process 
     with a larger number of sample parameter values or a better distri-
     bution of the same number of parameters. A typical calling example:

       NL_INDEX      k;
       NL_VECTOR     B0, *B;
       NL_PARAMETER  *t;
       NL_CURVE      cur;
       ...
       (get cur, B0 and the parameters);
       ...
       N_CreateBVectors(&cur,B0,t,k,B);

     IT IS ASSUMED THAT MEMORY TO STORE THE "B" VECTORS IS  ALLOCATED IN
     THE CALLING ROUTINE.


   ACCESS:
   
     cur , input  ,  NURBS curve
     B0  , input  ,  Start "B" vector at t[0]
     t   , input  ,  Parameters where the "B"s are to be computed
     k   , input  ,  Highest index in t
     B   , output ,  "B"s computed at t[i], i=0,...,k


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateBVectors( NL_CURVE *cur, NL_VECTOR B0, NL_PARAMETER *t, NL_INDEX k, NL_VECTOR *B )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL dot, dis;

    NL_VECTOR *T, A;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize */

    N_VectorCopy( B0, &B[0] );

    /* normal B[0] in place */
    error = N_VectorNormalizeRef( &B[0] );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute unit tangents and the B's in one direction */

    /* allocate memory for T */
    T = N_AllocPt1dArray( k, &SL );

    if( T EQ NULL )
        NL_QUIT;

    for ( i = 1; i <= k; i++ )
    {
        error = N_CrvEvalTangent( cur, t[i], NL_LEFT, &A, &T[i] );

        if( error EQ NL_YES )
            NL_OUT;

        /* let B[i] = B[i-1] - (B[i-1]*T[i])*T[i]  */
        N_VectorDot( B[i - 1], T[i], &dot );
        N_Combine2Pts( 1.0, B[i - 1], -dot, T[i], &B[i] );

        /* normalize B[i] */
        error = N_VectorNormalizeRef( &B[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Correct for closed curves */

    if( N_CrvIsClosed( cur ) )
    {
        N_DistPtPt( B[0], B[k], &dis );

        if( dis GT NL_MTOL )
        {
            N_CopyPt( B[0], &B[k] );

            for ( i = k; i >= 2; i-- )
            {
                N_VectorDot( B[i], T[i - 1], &dot );
                N_Combine2Pts( 1.0, B[i], -dot, T[i - 1], &A );

                error = N_VectorNormalizeRef( &A );

                if( error EQ NL_YES )
                    NL_OUT;

                N_Combine2Pts( 0.5, B[i - 1], 0.5, A, &B[i - 1] );

                error = N_VectorNormalizeRef( &B[i - 1] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateBVectors */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine creates a swept surface  
     given a trajectory  curve  and a section  curve to be swept along 
     the  trajectory. The  swept  surface  is computed  by positioning 
     scaled instances of the section curve  along the  trajectory, and
     skinning  across them. If  the output  surface is  initialized to 
     NULL, memory to store new control points and knots  is allocated. 
     A typical calling example is:

       NL_CURVE      curT, curC, curS;
       NL_INDEX      k, sub;
       NL_REAL       ES;
       NL_VECTOR     Z0;
       NL_DEGREE     deg;
       NL_SURFACE    sur;
       NL_STACKS     SG;
       ...
       (get curves, choose deg, Z0, sub and ES);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSweepSrf(&curT,&curC,k,Z0,sub,&curS,deg,NL_UDIR,ES,&sur,&SG);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon objects. The  section curve is aligned to
     the  trajectory by  mapping its  global coordinate  system to the 
     trajectory's local  systems computed recursively using Z0 and the
     tangents  at each point  where instances of the section curve are 
     placed. If placements of the section  curve are not satisfactory, 
     they  can be  modified by  editing them in  the global coordinate 
     system. The surface can be computed precisely (ES=0.0) or approx-
     imately (ES>0.0) using ES  as an error control tolerance. A  very 
     accurate  swept  surface with a low number  of  control points is  
     obtained  by  generating  a  large  number of section curves, and 
     approximately skinning across them.

     Note: We recommend using N_CreateSweepScale instead of this function.  This
           is an older sweep function,  left in the library for upward
           compatibility reasons.


   ACCESS:
   
     curT , input  ,  Trajectory curve
     curC , input  ,  Cross-sectional curve
     k    , input  ,  Highest  index of  instances of  curC positioned
                      along curT, i.e. the number of section curves is
                      k+1
     Z0   , input  ,  Start local z-axis of the trajectory
     sub  , input  ,  Subdivision level; the higher the sub the more Z
                      vectors are computed to obtain a smoothly moving
                      local frame along the trajectory. If k is large,
                      sub should  be small (1 or 2). If k  is small, a 
                      larger sub, e.g. 5-10, is recommended.
     curS , input  ,  Scaling curve:
                        = NULL  no scaling is desired.
                       != NULL  curS[t[i]]  is a vector used to scale
                                 the  section  curve  in   the   x-y-z 
                                 directions.
     deg  , input  ,  Degree of swept surface:
                        = 0  interpolate trajectory; surface degree =
                             trajectory degree
                       != 0  do  not  interpolate trajectory; surface
                             degree = deg 
     dir  , input  ,  Flag:
                        NL_UDIR: sweep  in  the  u-direction (all section 
                              curves are v-curves)
                        NL_VDIR: sweep  in  the  v-direction (all section 
                              curves are u-curves)
     ES   , input  ,  Skinning tolerance; set 0.0 for precise skinning
     sur  , output ,  Swept surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSweepSrf( NL_CURVE *curT, NL_CURVE *curC, NL_INDEX k, NL_VECTOR Z0, NL_INDEX sub, NL_CURVE *curS, NL_DEGREE deg, NL_FLAG dir, NL_REAL ES, NL_SURFACE *sur, NL_STACKS *SG )
{

    /* NL_PRIVATE NL_STRING rname = _T("N_CreateSweepSrf"); */
    NL_PRIVATE NL_REAL tol = 0.01;

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, l, nt, mt, m, ma, off;

    NL_DEGREE pt, ldeg;

    NL_REAL *UT, *UA, *ts, *t, tinc, sum, ltol, d;

    NL_POINT O;

    NL_CPOINT *Pw;

    NL_VECTOR *B, X, Y, Z, f;

    NL_KNOTVECTOR *knt, kna;

    NL_RMATRIX rma;

    NL_CURVE ** cur;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Adjust local tolerance */

    error = N_CrvGetBBoxMaxDiagDist( curT, &d );

    if( error EQ NL_YES )
        NL_OUT;

    ltol = d * tol;

    /* Compute parameters and knot vector */

    N_CrvGetCPtsDegreeAndKnots( curT, &nt, &Pw, &pt, &mt, &UT );
    ldeg = deg;

    if( deg EQ 0 )
    {
        ldeg = pt;

        if( nt GE k )
            k = nt;
    }

    if( N_CrvIsClosed( curT ) )
    {
        m = k + ldeg + 3;
        ma = k - nt + 2;
        off = 1;
    }
    else
    {
        m = k + ldeg + 1;
        ma = k - nt;
        off = 0;
    }

    if( deg EQ 0 )
    {
        /* Interpolate trajectory */

        N_CrvGetKnotVector( curT, &knt );

        /* Add knots */

        if( nt LT k )
        {
            UA = N_AllocReal1dArray( m, &SL );

            if( UA EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= mt; i++ )
                UA[i] = UT[i];

            N_KnotVectorFromRealArray( &kna, UA, mt );
            N_BasisSplitLongestSpan( &kna, ldeg, ma );
            N_KnotVectorGetKnots( &kna, &mt, &UT );

            knt = &kna;
        }

        /* Compute parameters */

        t = N_AllocReal1dArray( k, &SL );

        if( t EQ NULL )
            NL_QUIT;

        t[0] = UT[0];
        t[k] = UT[mt];

        for ( i = 1; i < k; i++ )
        {
            sum = 0.0;

            for ( j = 1; j <= ldeg; j++ )
                sum += UT[i + j + off];
            t[i] = sum / ldeg;
        }
    }
    else
    {
        /* Do not interpolate trajectory */

        t = N_AllocReal1dArray( k, &SL );

        if( t EQ NULL )
            NL_QUIT;

        knt = N_AllocKnotVectorAndArray( m, &SL );

        if( knt EQ NULL )
            NL_QUIT;

        error = N_CrvEvalEvenSpacedPts( curT, UT[0], UT[mt], k, ltol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        if( N_CrvIsClosed( curT ) )
            N_FitCalcKnotVectorEndDerivs( t, k, ldeg, knt );
        else
            N_FitCrvCalcKnotVector( t, k, ldeg, knt );
    }

    /* Get z-axes */

    if( sub LE 0 )
        sub = 1;

    l = k * (sub + 1);

    ts = N_AllocReal1dArray( l, &SL );

    if( ts EQ NULL )
        NL_QUIT;

    B = N_AllocPt1dArray( l, &SL );

    if( B EQ NULL )
        NL_QUIT;

    for ( i = 0; i < k; i++ )
    {
        tinc = (t[i + 1] - t[i]) / ((NL_REAL)sub + 1.0);
        l = i * (sub + 1);

        for ( j = 0; j <= sub; j++ )
            ts[l + j] = t[i] + j * tinc;
    }

    l = k * (sub + 1);
    ts[l] = t[k];

    error = N_CreateBVectors( curT, Z0, ts, l, B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Align curves */

    error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    cur = N_AllocArrayCrvPtrs( k, &SL );

    if( cur EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        cur[i] = N_AllocCrv( &SL );

        if( cur[i]EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( cur[i] );
        error = N_CrvCopy( curC, cur[i], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( curS NEQ NULL )
        {
            error = N_CrvEval( curS, t[i], NL_LEFT, &f );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvScale( cur[i], NL_ZERO, f );
        }

        error = N_CrvEvalTangent( curT, t[i], NL_LEFT, &O, &X );

        if( error EQ NL_YES )
            NL_OUT;

        l = i * (sub + 1);
        N_CopyPt( B[l], &Z );
        N_VectorCross( Z, X, &Y );

        error = N_CreateTransformMatrixFromAxes( NL_ZERO, NL_UNITX, NL_UNITY, NL_UNITZ, O, X, Y, Z, &rma, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvTransform( cur[i], &rma );
    }

    /* Skin across section curves */

    if( ES LT NL_PTOL )
    {
        error = N_CreateSkinSrfParams( cur, k, NL_NO, NL_YES, ldeg, dir, t, knt, sur, &SL, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_CreateSkinSrfApproxTol( cur, k, NL_NO, NL_YES, 2, ldeg, 0.0, ES, dir, sur, &SL, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSweepSrf */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advance surface construction routine creates a Gordon surface
     interpolating  two sets  of compatible  NON-RATIONAL  curves. More
     precisely, given C_i(u) at v[i], i=0,...,k, and C_j(v) at u[j], j=
     0,...,l. It is  assumed that  C_i(u) and  C_j(v) are compatible in 
     the B-spline sense, and that the following condition holds:

                  C_i(u[j]) = C_j(v[i]) = Q_j,i

     The Gordon surface interpolates both the u- and the v-curves, i.e.

                  S(u_j,v) = C_j(v), j=0,...,l
                  S(u,v_i) = C_i(u), i=0,...,k

     If the output surface is initialized to NULL, memory  to store new 
     control points and knots is allocated. A typical calling example:

       NL_CURVE      **curU, **curV;
       NL_INDEX      k, l;
       NL_DEGREE     pl, ql, pt,qt;
       NL_PARAMETER  *u, *v;
       NL_SURFACE    sur;
       NL_STACKS     SC, SS;
       ...
       (get array of curves, parameters and choose degrees);
       ...
       N_SrfInitArrays(&sur);
       N_CreateGordonSrf(curU,v,k,curV,u,l,pl,ql,pt,qt,NL_YES,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest  indexes in sur's 
     knot vector and  polygon  objects. ALL CROSS-SECTIONAL CURVES MUST
     BE ON THE  SAME STACK "SC". The  parametrizations for the  various 
     surface constructions  can be chosen  as follows: (1) maintain the
     supplied u- and v-parameters, or (2) let the  surface construction
     routines compute their  own parametrizations  independently of one
     another.  
     k > pl  .....you need at least 4 curves for a cubic, for example
     l > ql



   ACCESS:

     curU       , input  :  U-curves in v-direction
     vPrms      , input  :  Parameters where u-curves are assumed
     lNumU      , input  :  Highest index in curU and v
     curV       , input  :  V-curves in u-direction
     uPrms      , input  :  Parameters where v-curves are assumed
     lNumV      , input  :  Highest index in curV and u
     degLoftU   , degLoftV  , input  : Degrees for u- and v-directional lofting
     degTensorU , degTensorV, input  : Degrees for tensor product interpolation
     bPreserveParams , input  :  Flag:
                         NL_YES: preserve u- and v-parameters
                         NL_NO : let  lofting  and  interpolation routines
                              compute new parameters
     sur , output :  Gordon surface
     SC  , input  :  curs' memory stack
     SS  , input  :  sur's memory stack




   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateGordonSrf( NL_CURVE ** curU, NL_PARAMETER *vPrms, NL_INDEX lNumU, NL_CURVE ** curV, NL_PARAMETER *uPrms, NL_INDEX lNumV, NL_DEGREE degLoftU, NL_DEGREE degLoftV, NL_DEGREE degTensorU, NL_DEGREE degTensorV, NL_FLAG bPreserveParams, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateGordonSrf");

static int cbi=0;

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, ncptsU, ncptsV, r, s;

    NL_DEGREE p, q;

    NL_REAL *U = NULL, *V = NULL, *UA, *VA, *tmp;

    NL_POINT ** Q, CU, CV;

    NL_CPOINT ** Pw, ** Uw, ** Vw, ** Tw;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL;

    NL_SURFACE ** surA, surU, surV, surT;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check curves */

    if( NOT N_CrvsAreCombatible( curU, lNumU ) )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvsAreCombatible( curV, lNumV ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    /* Get local memory */

    Q = N_AllocPt2dArray( lNumV, lNumU, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    if( bPreserveParams EQ NL_YES )
    {
        p = NL_MAX( degLoftU, degTensorU );
        q = NL_MAX( degLoftV, degTensorV );

        knu = N_AllocKnotVectorAndArray( lNumV + p + 1, &SL );

        if( knu EQ NULL )
            NL_QUIT;

        knv = N_AllocKnotVectorAndArray( lNumU + q + 1, &SL );

        if( knv EQ NULL )
            NL_QUIT;
    }

    /* Get intersection points */

    for ( ii = 0; ii <= lNumU; ii++ )
    {
        for ( jj = 0; jj <= lNumV; jj++ )
        {
            error = N_CrvEval( curU[ii], uPrms[jj], NL_LEFT, &CU );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvEval( curV[jj], vPrms[ii], NL_LEFT, &CV );

            if( error EQ NL_YES )
                NL_OUT;

            N_Combine2Pts( 0.5, CU, 0.5, CV, &Q[jj][ii] );
        }
    }

    /* Compute the various surfaces */

    switch( bPreserveParams )
    {
        case NL_YES: /* Maintain u- and v-parameters */

            /* U-lofting */

            N_FitCrvCalcKnotVector( uPrms, lNumV, degLoftU, knu );
            N_SrfInitArrays( &surU );
            error = N_CreateSkinSrfParams( curV, lNumV, NL_NO, NL_NO, degLoftU, NL_UDIR, uPrms, knu, &surU, SC, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* V-lofting */

            N_FitCrvCalcKnotVector( vPrms, lNumU, degLoftV, knv );
            N_SrfInitArrays( &surV );
            error = N_CreateSkinSrfParams( curU, lNumU, NL_NO, NL_NO, degLoftV, NL_VDIR, vPrms, knv, &surV, SC, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Tensor product */

            N_FitCrvCalcKnotVector( uPrms, lNumV, degTensorU, knu );
            N_FitCrvCalcKnotVector( vPrms, lNumU, degTensorV, knv );
            N_SrfInitArrays( &surT );
            error = N_FitSrfToPtsKnots( (NL_VOID ** )Q, lNumV, lNumU, NL_EPOINT, uPrms, vPrms, knu, knv, degTensorU, degTensorV, &surT, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_NO: /* Compute u- and v-parameters */

            /* U-lofting */

            N_SrfInitArrays( &surU );
            error = N_CreateSkinSrf( curV, lNumV, NL_NO, NL_NO, degLoftU, NL_UDIR, 0.0, &surU, SC, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* V-lofting */

            N_SrfInitArrays( &surV );
            error = N_CreateSkinSrf( curU, lNumU, NL_NO, NL_NO, degLoftV, NL_VDIR, 0.0, &surV, SC, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Tensor product */

            N_SrfInitArrays( &surT );
            error = N_FitSrfToPts( Q, lNumV, lNumU, degTensorU, degTensorV, NL_CHORDLENGTH, &surT, &SL );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Make surfaces compatible */

    surA = N_AllocArraySrfPtrs( 2, &SL );

    if( surA EQ NULL )
        NL_QUIT;

    surA[0] = &surU;
    surA[1] = &surV;
    surA[2] = &surT;

    error = N_MakeSrfsCompatibleUV( surA, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute output surface */

    N_SrfGetCPtsDegreesAndKnots( surA[0], &ncptsU, &ncptsV, &Uw, &p, &q, &r, &s, &UA, &VA );

    error = N_SrfSizeArrays( sur, ncptsU, ncptsV, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( surA[1], &Vw, &tmp, &tmp );
    N_SrfGetCPtsAndKnots( surA[2], &Tw, &tmp, &tmp );
    N_SrfGetCPtsAndKnots( sur,     &Pw, &U,   &V   );

    for ( ii = 0; ii <= ncptsU; ii++ )
    {
        for ( jj = 0; jj <= ncptsV; jj++ )
        {

/*cbi: experiment: */
if ( cbi==1 )
  { Pw[ii][jj] = Uw[ii][jj]; }
else if ( cbi==2 )
  { Pw[ii][jj] = Vw[ii][jj]; }
else if ( cbi==3 )
  { Pw[ii][jj] = Tw[ii][jj]; }
else {

            N_TranslateSum2CPts( Uw[ii][jj],    /* in : Cw    of Dw = Cw + alpha*Aw + beta*Bw */
                                 1.0,           /* in : alpha of Dw = Cw + alpha*Aw + beta*Bw */
                                 Vw[ii][jj],    /* in : Aw    of Dw = Cw + alpha*Aw + beta*Bw */
                                 -1.0,          /* in : beta  of Dw = Cw + alpha*Aw + beta*Bw */
                                 Tw[ii][jj],    /* in : Bw    of Dw = Cw + alpha*Aw + beta*Bw */
                                 &Pw[ii][jj] ); /* out: Dw    of Dw = Cw + alpha*Aw + beta*Bw */
} /*cbi. */

        }
    }

    for ( ii = 0; ii <= r; ii++ )
        U[ii] = UA[ii];

    for ( jj = 0; jj <= s; jj++ )
        V[jj] = VA[jj];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateGordonSrf */


/*******************************************************************//**


   DESCRIPTION:

     This advance surface construction routine creates a bilinear Coons
     surface  interpolating four  boundary curves. The  boundary curves 
     are  assumed  to intersect at their  respective  end  points, e.g. 
     C_1(u=0) = C_0(v=1) = S01. If the output surface is initialized to 
     NULL, memory to store new control points and knots is allocated. A 
     typical calling example:

       NL_CURVE    **curU, **curV;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (get boundary curves);
       ...
       N_SrfInitArrays(&sur);
       N_CreateCoonsSrf(curU,curV,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest  indexes in sur's 
     knot vector and  polygon  objects.  ALL BOUNDARY CURVES MUST BE ON
     THE  SAME  STACK  "SC". If at least one of the curves is rational,
     then all  four of them  are  reparametrized  in-place  to make end
     weights equal, i.e. to satisfy  compatibility conditions in 4-D.


   ACCESS:
   
     curU  , in/out ,  U-boundaries (curU[0] and curU[1])
     curV  , in/out ,  V-boundaries (curV[0] and curV[1])
     sur   , output ,  Bilinear Coons surface
     SC    , input  ,  curs' memory stack
     SS    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCoonsSrf( NL_CURVE ** curU, NL_CURVE ** curV, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCoonsSrf");

    NL_FLAG allnra, error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UA, *VA, *tmp, w = 0.0, sum, us = 0.0, ue, vs, ve;

    NL_POINT P00, P01, P10, P11, CU, CV;

    NL_CPOINT ** Pw, ** Uw, ** Vw, ** Tw, *Aw;

    NL_SURFACE ** surA, surU, surV, surT;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get corner points */

    N_CrvGetKnots( curU[0], &r, &U );
    N_CrvGetKnots( curV[0], &s, &V );
    error = N_CrvEval( curU[0], U[0], NL_LEFT, &CU );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( curV[0], V[0], NL_LEFT, &CV );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, CU, 0.5, CV, &P00 );

    N_CrvGetKnots( curV[1], &s, &V );
    error = N_CrvEval( curU[0], U[r], NL_LEFT, &CU );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( curV[1], V[0], NL_LEFT, &CV );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, CU, 0.5, CV, &P10 );

    N_CrvGetKnots( curU[1], &r, &U );
    N_CrvGetKnots( curV[0], &s, &V );
    error = N_CrvEval( curU[1], U[0], NL_LEFT, &CU );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( curV[0], V[s], NL_LEFT, &CV );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, CU, 0.5, CV, &P01 );

    N_CrvGetKnots( curV[1], &s, &V );
    error = N_CrvEval( curU[1], U[r], NL_LEFT, &CU );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( curV[1], V[s], NL_LEFT, &CV );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, CU, 0.5, CV, &P11 );

    /* Check rationality of boundaries */

    allnra = NL_TRUE;

    for ( i = 0; i <= 1; i++ )
    {
        if( N_IsCrvRat( curU[i] ) )
            allnra = NL_FALSE;

        if( N_IsCrvRat( curV[i] ) )
            allnra = NL_FALSE;
    }

    if( allnra EQ NL_FALSE )
    {
        /* Make all curves rational */

        for ( i = 0; i <= 1; i++ )
        {
            if( NOT N_IsCrvRat( curU[i] ) )
                N_CrvNonRatToRat( curU[i] );

            if( NOT N_IsCrvRat( curV[i] ) )
                N_CrvNonRatToRat( curV[i] );
        }

        /* Average end weights */

        sum = 0.0;

        for ( i = 0; i <= 1; i++ )
        {
            N_CrvGetCPts( curU[i], &n, &Aw );
            N_CPtGetW( Aw[0], &w );
            sum += w;
            N_CPtGetW( Aw[n], &w );
            sum += w;

            N_CrvGetCPts( curV[i], &n, &Aw );
            N_CPtGetW( Aw[0], &w );
            sum += w;
            N_CPtGetW( Aw[n], &w );
            sum += w;
        }

        /* Reparametrize to get end weights = w */

        w = sum / 8.0;

        for ( i = 0; i <= 1; i++ )
        {
            N_CrvGetKnots( curU[i], &r, &U );
            error = N_CrvReparamWeights( curU[i], w, U[0], U[r] );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetKnots( curV[i], &s, &V );
            error = N_CrvReparamWeights( curV[i], w, V[0], V[s] );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /********************************/
    /* Compute the various surfaces */
    /********************************/

    /* U-ruling */

    N_SrfInitArrays( &surU );
    error = N_CreateRuledSrf( curV[0], curV[1], NL_UDIR, &surU, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* V-ruling */

    N_SrfInitArrays( &surV );
    error = N_CreateRuledSrf( curU[0], curU[1], NL_VDIR, &surV, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Bilinear surface */

    N_SrfInitArrays( &surT );
    error = N_CreateSrfCornerPts( P00, P10, P01, P11, &surT, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Ensure surface has same parameter domain as curves */

    N_CrvGetParamBounds( curU[0], &us, &ue );
    N_CrvGetParamBounds( curV[0], &vs, &ve );

    N_SrfReparam( &surU, us, ue, vs, ve );
    N_SrfReparam( &surV, us, ue, vs, ve );
    N_SrfReparam( &surT, us, ue, vs, ve );

    /* Scale control points of surT if necessary */

    if( allnra EQ NL_FALSE )
    {
        N_SrfGetCPts( &surT, &n, &m, &Pw );

        N_CPtSetW( 1.0, &Pw[0][0] );
        N_ScaleCPt( w, Pw[0][0], &Pw[0][0] );
        N_CPtSetW( 1.0, &Pw[0][1] );
        N_ScaleCPt( w, Pw[0][1], &Pw[0][1] );
        N_CPtSetW( 1.0, &Pw[1][0] );
        N_ScaleCPt( w, Pw[1][0], &Pw[1][0] );
        N_CPtSetW( 1.0, &Pw[1][1] );
        N_ScaleCPt( w, Pw[1][1], &Pw[1][1] );
    }

    /* Make surfaces compatible */

    surA = N_AllocArraySrfPtrs( 2, &SL );

    if( surA EQ NULL )
        NL_QUIT;

    surA[0] = &surU;
    surA[1] = &surV;
    surA[2] = &surT;

    error = N_MakeSrfsCompatibleUV( surA, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute output surface */

    N_SrfGetCPtsDegreesAndKnots( surA[0], &n, &m, &Uw, &p, &q, &r, &s, &UA, &VA );

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( surA[1], &Vw, &tmp, &tmp );
    N_SrfGetCPtsAndKnots( surA[2], &Tw, &tmp, &tmp );
    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TranslateSum2CPts( Uw[i][j], 1.0, Vw[i][j], -1.0, Tw[i][j], &Pw[i][j] );
        }
    }

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCoonsSrf */

#ifdef NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advance surface  construction routine creates a bicubic Coons
     surface  interpolating four  NON-RATIONAL boundary curves and four
     twist vectors. The  boundary  curves  are assumed  to intersect at 
     their respective end points, eg C_1(u=0) = C_0(v=1) = S01. The two
     curU curves must be defined on the same  u-parameter  domain,  and 
     the two curV curves must be defined on the same v-parameter domain 
     (not necessarily the same two domains).  The twist vectors must be
     defined with respect to these  parameterizations,  and  the  Coons 
     surface will have  the  parameterization  of  the  curU  and  curV 
     curves.  If the output surface is initialized to NULL,  memory  to 
     store new control points and knots is allocated. A typical calling
     example is:

       NL_CURVE    **curU, **curV;
       NL_VECTOR   **T;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (get boundary curves and twists);
       ...
       N_SrfInitArrays(&sur);
       N_CreateCoonsSrfTwist(curU,curV,T,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest  indexes in sur's 
     knot vector and  polygon  objects.  ALL BOUNDARY CURVES MUST BE ON
     THE SAME STACK "SC". 


   ACCESS:
   
     curU  , in/out ,  U-boundaries (curU[0] and curU[1])
     curV  , in/out ,  V-boundaries (curV[0] and curV[1])
     T     , input  ,  Corner  twists  (T[0][0],  T[1][0],  T[0][1] and 
                       T[1][1])
     sur   , output ,  Bicubic Coons surface
     SC    , input  ,  curs' memory stack
     SS    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCoonsSrfTwist( NL_CURVE ** curU, NL_CURVE ** curV, NL_VECTOR ** T, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCoonsSrfTwist");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UA, *VA, *tmp, one_3, one_9, du, dv, us, ue, vs, ve, d1, d2;

    NL_POINT D00[2], D01[2], D10[2], D11[2], A;

    NL_CPOINT ** Pw, ** Uw, ** Vw, ** Tw, *Dw, *Aw, T00w, T10w, T01w, T11w;

    NL_CURVE ** curA, ** curB;

    NL_SURFACE ** surA, surU, surV, surT;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check rationality and parameter domains, and initialize constants */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    one_3 = 1.0 / 3.0;
    one_9 = 1.0 / 9.0;

    N_CrvGetParamBounds( curU[0], &us, &ue );
    N_CrvGetParamBounds( curU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    du = ue - us;

    N_CrvGetParamBounds( curV[0], &vs, &ve );
    N_CrvGetParamBounds( curV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    dv = ve - vs;

    /*************************************************/
    /* Get cross-boundary derivatives for u-blending */
    /*************************************************/

    curA = N_AllocArrayCrvPtrs( 3, SC );

    if( curA EQ NULL )
        NL_QUIT;

    curA[0] = curV[0];
    curA[1] = curV[1];

    curA[2] = N_AllocCrvAndArrays( 3, 3, 7, SC );

    if( curA[2]EQ NULL )
        NL_QUIT;

    curA[3] = N_AllocCrvAndArrays( 3, 3, 7, SC );

    if( curA[3]EQ NULL )
        NL_QUIT;

    /* Get corner derivatives */

    N_CrvGetKnots( curU[0], &r, &U );

    error = N_CrvDerivs( curU[0], U[0], NL_LEFT, 1, D00 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curU[0], U[r], NL_LEFT, 1, D10 );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curU[1], &r, &U );

    error = N_CrvDerivs( curU[1], U[0], NL_LEFT, 1, D01 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curU[1], U[r], NL_LEFT, 1, D11 );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier representation of cross-boundary derivatives */

    N_CrvGetCPtsAndKnots( curA[2], &Dw, &V );

    N_PtToCPt( D00[1], &Dw[0] );
    N_Combine2Pts( 1.0, D00[1], one_3 * dv, T[0][0], &A );
    N_PtToCPt( A, &Dw[1] );
    N_Combine2Pts( 1.0, D01[1], -one_3 * dv, T[0][1], &A );
    N_PtToCPt( A, &Dw[2] );
    N_PtToCPt( D01[1], &Dw[3] );

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    N_CrvGetCPtsAndKnots( curA[3], &Dw, &V );

    N_PtToCPt( D10[1], &Dw[0] );
    N_Combine2Pts( 1.0, D10[1], one_3 * dv, T[1][0], &A );
    N_PtToCPt( A, &Dw[1] );
    N_Combine2Pts( 1.0, D11[1], -one_3 * dv, T[1][1], &A );
    N_PtToCPt( A, &Dw[2] );
    N_PtToCPt( D11[1], &Dw[3] );

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curA, 3, SC );

    if( error EQ NL_YES )
        NL_OUT;

    /*************************************************/
    /* Get cross-boundary derivatives for v-blending */
    /*************************************************/

    curB = N_AllocArrayCrvPtrs( 3, SC );

    if( curB EQ NULL )
        NL_QUIT;

    curB[0] = curU[0];
    curB[1] = curU[1];

    curB[2] = N_AllocCrvAndArrays( 3, 3, 7, SC );

    if( curB[2]EQ NULL )
        NL_QUIT;

    curB[3] = N_AllocCrvAndArrays( 3, 3, 7, SC );

    if( curB[3]EQ NULL )
        NL_QUIT;

    /* Get corner derivatives */

    N_CrvGetKnots( curV[0], &s, &V );

    error = N_CrvDerivs( curV[0], V[0], NL_LEFT, 1, D00 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV[0], V[s], NL_LEFT, 1, D01 );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curV[1], &s, &V );

    error = N_CrvDerivs( curV[1], V[0], NL_LEFT, 1, D10 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV[1], V[s], NL_LEFT, 1, D11 );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier representation of cross-boundary derivatives */

    N_CrvGetCPtsAndKnots( curB[2], &Dw, &U );

    N_PtToCPt( D00[1], &Dw[0] );
    N_Combine2Pts( 1.0, D00[1], one_3 * du, T[0][0], &A );
    N_PtToCPt( A, &Dw[1] );
    N_Combine2Pts( 1.0, D10[1], -one_3 * du, T[1][0], &A );
    N_PtToCPt( A, &Dw[2] );
    N_PtToCPt( D10[1], &Dw[3] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    N_CrvGetCPtsAndKnots( curB[3], &Dw, &U );

    N_PtToCPt( D01[1], &Dw[0] );
    N_Combine2Pts( 1.0, D01[1], one_3 * du, T[0][1], &A );
    N_PtToCPt( A, &Dw[1] );
    N_Combine2Pts( 1.0, D11[1], -one_3 * du, T[1][1], &A );
    N_PtToCPt( A, &Dw[2] );
    N_PtToCPt( D11[1], &Dw[3] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curB, 3, SC );

    if( error EQ NL_YES )
        NL_OUT;

    /********************************/
    /* Compute the various surfaces */
    /********************************/

    /* Allocate memory */

    N_CrvGetCPtsDegreeAndKnots( curA[0], &m, &Aw, &q, &s, &VA );
    N_CrvGetCPtsDegreeAndKnots( curB[0], &n, &Aw, &p, &r, &UA );

    error = N_AllocSrfArrays( &surU, 3, m, 3, q, 7, s, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocSrfArrays( &surV, n, 3, p, 3, r, 7, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocSrfArrays( &surT, 3, 3, 3, 3, 7, 7, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute u-blend */

    N_SrfGetCPtsAndKnots( &surU, &Uw, &U, &V );

    N_CrvGetCPts( curA[0], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_CopyCPt( Aw[j], &Uw[0][j] );

    N_CrvGetCPts( curA[1], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_CopyCPt( Aw[j], &Uw[3][j] );

    N_CrvGetCPts( curA[2], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_Combine2CPts( 1.0, Uw[0][j], one_3 * du, Aw[j], &Uw[1][j] );

    N_CrvGetCPts( curA[3], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_Combine2CPts( 1.0, Uw[3][j], -one_3 * du, Aw[j], &Uw[2][j] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* Compute v-blend */

    N_SrfGetCPtsAndKnots( &surV, &Vw, &U, &V );

    N_CrvGetCPts( curB[0], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Aw[i], &Vw[i][0] );

    N_CrvGetCPts( curB[1], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Aw[i], &Vw[i][3] );

    N_CrvGetCPts( curB[2], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_Combine2CPts( 1.0, Vw[i][0], one_3 * dv, Aw[i], &Vw[i][1] );

    N_CrvGetCPts( curB[3], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_Combine2CPts( 1.0, Vw[i][3], -one_3 * dv, Aw[i], &Vw[i][2] );

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    /* Compute tensor product surface */

    N_SrfGetCPtsAndKnots( &surT, &Tw, &U, &V );

    for ( i = 0; i <= 3; i++ )
    {
        N_CopyCPt( Uw[i][0], &Tw[i][0] );
        N_CopyCPt( Uw[i][m], &Tw[i][3] );
    }

    for ( j = 1; j <= 2; j++ )
    {
        N_CopyCPt( Vw[0][j], &Tw[0][j] );
        N_CopyCPt( Vw[n][j], &Tw[3][j] );
    }

    N_PtToCPt( T[0][0], &T00w );
    N_PtToCPt( T[1][0], &T10w );
    N_PtToCPt( T[0][1], &T01w );
    N_PtToCPt( T[1][1], &T11w );

    d1 = du * dv * one_9;

    N_Combine4CPts( d1, T00w, 1.0, Tw[1][0], 1.0, Tw[0][1], -1.0, Tw[0][0], &Tw[1][1] );
    N_Combine4CPts( -d1, T10w, 1.0, Tw[3][1], 1.0, Tw[2][0], -1.0, Tw[3][0], &Tw[2][1] );
    N_Combine4CPts( -d1, T01w, 1.0, Tw[1][3], 1.0, Tw[0][2], -1.0, Tw[0][3], &Tw[1][2] );
    N_Combine4CPts( d1, T11w, 1.0, Tw[2][3], 1.0, Tw[3][2], -1.0, Tw[3][3], &Tw[2][2] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    /* Make surfaces compatible */

    surA = N_AllocArraySrfPtrs( 2, &SL );

    if( surA EQ NULL )
        NL_QUIT;

    surA[0] = &surU;
    surA[1] = &surV;
    surA[2] = &surT;

    error = N_MakeSrfsCompatibleUV( surA, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute output surface */

    N_SrfGetCPtsDegreesAndKnots( surA[0], &n, &m, &Uw, &p, &q, &r, &s, &UA, &VA );

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( surA[1], &Vw, &tmp, &tmp );
    N_SrfGetCPtsAndKnots( surA[2], &Tw, &tmp, &tmp );
    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TranslateSum2CPts( Uw[i][j], 1.0, Vw[i][j], -1.0, Tw[i][j], &Pw[i][j] );
        }
    }

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCoonsSrfTwist */

/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface  construction  routine  creates  the  data 
     defining cross-boundary derivatives of a given surface. The cross-
     derivative  is  defined  as a  Bezier  strip  along  the specified 
     boundary. A typical calling example is:

       NL_SURFACE  sur, der;
       NL_STACKS   SG;
       ...
       (get surface);
       ...
       N_SrfInitArrays(&der);
       N_CreateDataBoundaryDerivs(&sur,NL_LEFT,&der,&SG);

     If memory is  available, der  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest  indexes in der's 
     knot vector and  polygon  objects.


   ACCESS:
   
     sur  , input  ,  NURBS surface
     bnd  , input  ,  Flag:
                        NL_LEFT  : extract boundary data along u=umin
                        NL_RIGHT : extract boundary data along u=umax
                        NL_BOTTOM: extract boundary data along v=vmin
                        NL_TOP   : extract boundary data along v=vmax
     der  , output ,  Derivative data
     SG   , input  ,  der's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateDataBoundaryDerivs( NL_SURFACE *sur, NL_FLAG bnd, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateDataBoundaryDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, ul, ur, vb, vt;

    NL_CPOINT ** Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* Get parameter bounds */

    switch( bnd )
    {
        case NL_LEFT:

            ul = U[0];
            ur = U[p + 1];
            vb = V[0];
            vt = V[s];
            break;

        case NL_RIGHT:

            ul = U[n];
            ur = U[r];
            vb = V[0];
            vt = V[s];
            break;

        case NL_BOTTOM:

            ul = U[0];
            ur = U[r];
            vb = V[0];
            vt = V[q + 1];
            break;

        case NL_TOP:

            ul = U[0];
            ur = U[r];
            vb = V[m];
            vt = V[s];
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Extract surface */

    error = N_SrfExtractPatch( sur,          /* target surface */
                               ul, ur,       /* U min and max patch boundaries */
                               vb, vt,       /* V min and max patch boundaries */
                               der,          /* extracted patch - made with knot insertion */
                               &SL, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateDataBoundaryDerivs */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  advanced   surface  construction  routine creates  the  data 
     defining  cross-boundary   derivatives.  The   cross-derivative is  
     defined  from a curve  and a vector (constant derivative of degree 
     0). It can be constructed as a boundary strip of type NL_LEFT, NL_RIGHT,
     NL_BOTTOM or NL_TOP. A typical calling example is:

       NL_CURVE    cur;
       NL_VECTOR   V;
       NL_SURFACE  der;
       NL_STACKS   SG;
       ...
       (get boundary curve cur and vector V);
       ...
       N_SrfInitArrays(&der);
       N_CreateSkinSrfBoundaryDerivs(&cur,V,NL_LEFT,&der,&SG);

     If memory is  available, der  is not  initialized and  the routine
     assumes that memory allocation  has been done. However, it  checks
     for the proper amount by looking  at the highest  indexes in der's 
     knot vector and  polygon  objects.


   ACCESS:
   
     cur  , input  ,  Boundary curve across which derivative applies
     V    , input  ,  Vector (degree zero derivative)
     bndy , input  ,  Boundary  flag  indicating  how  cross-derivative
                      strip should  be  constructed  (i.e.  from  which
                      boundary of an imaginary surface would cur be ex-
                      tracted):
                        NL_LEFT   : cur corresponds to u=umin boundary
                        NL_RIGHT  : cur corresponds to u=umax boundary
                        NL_BOTTOM : cur corresponds to v=vmin boundary
                        NL_TOP    : cur corresponds to v=vmax boundary
     der  , output ,  Derivative data
     SG   , input  ,  der's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfBoundaryDerivs( NL_CURVE *cur, NL_VECTOR V, NL_FLAG bndy, NL_SURFACE *der, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO, dir;

    NL_REAL mag;

    NL_INDEX ii, n, m;

    NL_CPOINT ** Pw, temp;

    NL_VECTOR V2;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Handle four cases */

    if( bndy EQ NL_LEFT OR bndy EQ NL_RIGHT )
        dir = NL_UDIR;
    else
        dir = NL_VDIR;

    if( bndy EQ NL_LEFT OR bndy EQ NL_BOTTOM )
        N_VectorCopy( V, &V2 );
    else
        N_VectorReverse( V, &V2 );

    /* Create derivative surface */

    N_VectorMagnitude( V2, &mag );

    /* create a cylinder by extruding cur in V2 directon */
    error = N_CreateSrfExtrudeCrv( cur, V2, mag, dir, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Reverse direction if bndy = NL_RIGHT or NL_TOP */

    if( bndy EQ NL_RIGHT OR bndy EQ NL_TOP )
    {
        /* locals: surface net indices, n and m, and control point array, Pw */
        N_SrfGetCPts( der, &n, &m, &Pw );

        if( bndy EQ NL_RIGHT )
        {
            for ( ii = 0; ii <= m; ii++ )
            {
                N_CopyCPt( Pw[0][ii], &temp );
                N_CopyCPt( Pw[1][ii], &Pw[0][ii] );
                N_CopyCPt( temp, &Pw[1][ii] );
            }
        }
        else
        {
            for ( ii = 0; ii <= n; ii++ )
            {
                N_CopyCPt( Pw[ii][0], &temp );
                N_CopyCPt( Pw[ii][1], &Pw[ii][0] );
                N_CopyCPt( temp, &Pw[ii][1] );
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfBoundaryDerivs */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes new knots to merge two knot vectors that have
     the  same end  knots  with the same  end  multiplicities. A typical 
     calling example is:

       NL_KNOTVECTOR  knr, kns;
       NL_DEGREE      p;
       NL_REAL        *XR, *XS;
       NL_INDEX       mr, ms;
       ...
       (define knr, kns; get p; allocate memory for XR and XS);
       ...
       N_MergeKnotVectors(&knr,&kns,p,XR,&mr,XS,&ms);

     IT  IS  ASSUMED  THAT MEMORY FOR  XR  AND  XS  IS  ALLOCATED IN THE 
     CALLING  ROUTINE. 
     

   ACCESS:
   
     knr , input  ,  Knot vector
     kns , input  ,  Knot vector
     p   , input  ,  Degree of knr and kns
     XR  , output ,  New knots to be inserted into knr
     rr  , output ,  Highest index in XR
     XS  , output ,  New knots to be inserted into kns
     ss  , output ,  Highest index in XS


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MergeKnotVectors( NL_KNOTVECTOR *knr, NL_KNOTVECTOR *kns, NL_DEGREE p, NL_REAL *XR, NL_INDEX *rr, NL_REAL *XS, NL_INDEX *ss )
{
    NL_PRIVATE NL_STRING rname = _T("N_MergeKnotVectors");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, mr, ms, mxr, mxs, ir, js, mult_r, mult_s;

    NL_REAL *R, *S, tol;

    /* Get local notation and initialize */

    N_KnotVectorGetKnots( knr, &mr, &R );
    N_KnotVectorGetKnots( kns, &ms, &S );

    if( fabs( R[0] - S[0] )GT NL_PTOL OR fabs( R[mr] - S[ms] )GT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    tol = NL_PTOL * (R[mr] - R[0]);
    mxr = -1;
    mxs = -1;
    ii = p + 1;
    jj = p + 1;

    /* Find knots to be inserted */

    while( ii LT mr - p OR jj LT ms - p )
    {
        /* Sanity check: avoid infinite loop.  */
        /* With bad data, these can slip past. */
        if ( ii GT mr  OR  jj GT ms )
          { NL_QUIT; }

        if( fabs( R[ii] - S[jj] ) LT tol )
        {
            ir = ii;

            while ( ii LT mr - p AND R[ii] EQ R[ii + 1] )
              { ii++; }
            mult_r = ii - ir + 1;

            js = jj;

            while ( jj LT ms - p AND S[jj] EQ S[jj + 1] )
              { jj++; }
            mult_s = jj - js + 1;

            for ( kk = mult_r + 1; kk <= mult_s; kk++ )
              { XR[++mxr] = S[jj]; }

            for ( kk = mult_s + 1; kk <= mult_r; kk++ )
              { XS[++mxs] = R[ii]; }

            ii++;
            jj++;
        }
        else if( R[ii] LT S[jj] )
        {
            ir = ii;

            while ( ii LT mr - p AND R[ii] EQ R[ii + 1] )
              { ii++; }
            mult_r = ii - ir + 1;

            for ( kk = 1; kk <= mult_r; kk++ )
              { XS[++mxs] = R[ii]; }

            ii++;
        }
        else if( R[ii] GT S[jj] )
        {
            js = jj;

            while ( jj LT ms - p AND S[jj] EQ S[jj + 1] )
              { jj++; }
            mult_s = jj - js + 1;

            for ( kk = 1; kk <= mult_s; kk++ )
              { XR[++mxr] = S[jj]; }

            jj++;
        }
    }

    *rr = mxr;
    *ss = mxs;

    /* Exit */

    EXIT:

    return (error);

} /* end N_MergeKnotVectors */


/*******************************************************************//**

   DESCRIPTION:

     This advanced surface construction routine creates a  skinned surf-
     ace interpolating or approximating a set of cross-sectional curves.
     Cross-boundary  continuity can be  specified in the skinning direc-
     tion, and rail curves can be given that  form the boundaries of the
     skinned  surface. The rails  must be  compatible, i.e. they must be
     defined over the same knot vector. A typical calling example is:

       NL_CURVE       **cur, **rail;
       NL_SURFACE     **der, sur;
       NL_PARAMETER   *ts;
       NL_REAL        ES;
       NL_INDEX       k;
       NL_DEGREE      deg;
       NL_FLAG        cnt[2], bnd[2];
       NL_KNOTVECTOR  *kns;
       NL_STACKS      SI, SS;
       ...
       (define various input data as described in ACCESS);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrfBoundaryContinuity(cur,k,der ,NL_YES,cnt,bnd,rail,deg,ES ,NL_UDIR,ts  ,kns ,&sur,
                &SI,&SS);
       N_CreateSkinSrfBoundaryContinuity(cur,k,NULL,NL_NO ,cnt,bnd,NULL,deg,0.0,NL_VDIR,NULL,NULL,&sur,
                &SI,&SS);

     If  the output  surface is  initialized to  NULL, memory  to  store
     control points  and knots is allocated. If memory is available, the 
   
     routine  checks for the proper  amount. If the  surface weights are
     out of the  range  [NL_WMIN,NL_WMAX], the error  NL_WEI_ERR is returned even
     though the  surface IS contructed. ALL  INPUT DATA, I.E. THE CROSS-
     SECTIONAL CURVES, THE RAILS AND THE  NL_DERIVATIVE SURFACES MUST BE ON
     THE SAME INPUT STACK SI!!



   ACCESS:
   
     cur  , in/out ,  Cross-sectional  curves. They  may  be degree ele-
                      vated in place and made compatible.
     lNumSects_1 , input  ,  Highest index in cur
     der  , in/out ,  HOMOGENEOUS  derivative  data  from  which  cross-
                      boundary continuity is computed:
                        der[0]: derivative across cur[0]
                        der[1]: derivative across cur[lNumSects_1]
                      If der = NULL, no derivative  constraint is  given
                      If der != NULL,  either  der[0] or der[1]  may  be
                      NULL, indicating no constraint for the correspond-
                      ing cross section.
                      IF NL_BOTH RAIL CURVES ARE PRESENT, THE RAILS AND THE
                      NL_DERIVATIVE SURFACES MUST BE NL_G1 CONTINUOUS IN HOMO-
                      GENEOUS SPACE!!! IF ONLY ONE RAIL CURVE IS GIVEN,
                      NL_G1 CONTINUITY IS REQUIRED IN EUCLIDEAN SPACE ONLY!
     cmp  , input  ,  Flag:
                        NL_YES: cur[0],...,cur[lNumSects_1], der[0], der[1]  are compatible
                        NL_NO : make cur[i] and der[j] compatible
     cnt  , input  ,  Flag:
                        cnt[0]: NL_G1 or NL_C1 for der[0]
                        cnt[1]: NL_G1 or NL_C1 for der[1]
     bnd  , input  ,  Flag:
                        bnd[0]: boundary flag indicating der[0]'s type:
                          NL_LEFT, NL_RIGHT, NL_BOTTOM or NL_TOP 
                        bnd[1]: boundary flag indicating der[1]'s type:
                          NL_LEFT, NL_RIGHT, NL_BOTTOM or NL_TOP
                      THESE FLAGS INDICATE HOW THE SURFACES WERE CREATED
                      (SEE N_CreateDataBoundaryDerivs AND N_CreateSkinSrfBoundaryDerivs!)
     rail , in/out ,  Rail  curves in the  skinning  direction, e.g. for
                      skinning in the NL_UDIR:
                        rail[0]: rail for v = vmin
                        rail[1]: rail for v = vmax
                      If rail = NULL, no rail is given. If rail != NULL,
                      rail[0] and  rail[1] can be individually  NULL. If
                      both rails are present, THEY MUST BE COMPATIBLE!!!
     deg  , input  ,  Required degree in the skinning direction. If rail
                      curves are present, degree = degree of rail(s).
     ES   , input  ,  Skinning tolerance:
                        ES > 0.0: the  skinning is approximate, i.e. the
                                  cur[i]  will  not  deviate  from   the 
                                  surface more then ES.
                        ES = 0.0: the skinned  surface interpolates  the
                                  cur[i].
     dir  , input  ,  Flag:
                       NL_UDIR: u-skin (cur[i] are v-curves)
                       NL_VDIR: v-skin (cur[i] are u-curves)
     ts   , input  ,  Parameters where cur[i] are assumed:
                        ts != NULL: parameters used for skinning.
                           rail != NULL: cur[i]   intersect  rail(s)  at 
                                         ts[i]. MUST  BE PASSED  IN!
                           rail  = NULL: parameters  used  for skinning
                        ts  = NULL: parameters are computed
                           rail != NULL: Error!   Parameters   must   be 
                                         passed in.
                           rail  = NULL: parameters are computed.
     kns  , input  ,  Knot vector in the skinning direction.
                        kns != NULL: kns passed in.
                            rail != NULL: kns must contain knots of rail
                            rail  = NULL: kns used for skinning
                        kns  = NULL: kns is computed internally
     sur  , output ,  Skinned surface
     SI   , input  ,  Memory stack for all input data, i.e. cur, der and
                      rail
     SS   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfBoundaryContinuity( NL_CURVE ** cur, NL_INDEX lNumSects_1, NL_SURFACE ** der, NL_FLAG cmp, NL_FLAG *cnt, NL_FLAG *bnd, NL_CURVE ** rail, NL_DEGREE deg, NL_REAL ES, NL_FLAG dir, NL_PARAMETER *ts, NL_KNOTVECTOR *kns, NL_SURFACE *sur, NL_STACKS *SI, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrfBoundaryContinuity");
    NL_PRIVATE NL_CPOINT ORIGIN = { 0.0, 0.0, 0.0, 1.0 };

    NL_PRIVATE NL_REAL dper = 0.92;
    NL_PRIVATE NL_REAL lper = 0.88;

    NL_FLAG reset, sdr, mll = NL_NO, mlr = NL_NO, lsm = NL_NO, opr = NL_NO, trl = NL_NO, rat = NL_NO, dfl = NL_NO, rfl = NL_NO, dec = NL_NO, error = NL_NO;
    NL_FLAG InBnd ;

    NL_INDEX i, j, l, js, je, nc, mc, ns = 0, ms, rs, ss, nr = 0, mr = 0, nd, mxr, mxt;

    NL_DEGREE pc, pr = 0, ps, qs;

    NL_REAL *UF, *UT, *UC, *UR = NULL, *XR = NULL, *XT = NULL, *US, *VS, *fu, wa, wb, ra, rb, dist, fact, mg0 = 0.0, mgn = 0.0, L00, L10, L01, L11, L0 = 0.0, L1 = 0.0, d1, d2, fac0, fac1, alf = 0.0;

    NL_PARAMETER *tl;

    NL_CVECTOR *Dws = NULL, *Dwe = NULL, *Ws, *We;

    NL_POINT A, B, Ca, Cb, T = { 0,0,0 };

    NL_CPOINT ** Sw, ** Rw, ** SUL = NULL, ** SUR = NULL, ** SVB = NULL, ** SVT = NULL, *Cw, *Dw, *R0w, *R1w, *Pw, *Qw, RDw[2], Aw, Bw;

    NL_KNOTVECTOR *knt, *kna, *knr = NULL, *knu, *knv, knx;

    NL_RMATRIX cm;

    NL_CURVE ** curM, *curDA = NULL, *curDB = NULL, curA, curB, curPA, curPB, curI;

    NL_CPOLYGON *pol;

    NL_CFUN cfnA, cfnB;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check boundary of derivative surfaces */

    /* array of cross derivative surfaces */
    if( der NEQ NULL )
    {
        /* cross-sections are U isoparameter curves */
        if( dir EQ NL_UDIR )
        {
            /* given a start crossDerivative and its not the u=umax */
            if( der[0] NEQ NULL AND bnd[0] NEQ NL_RIGHT )
            {
                /* replace deriv surface with one built for u=umax */
                InBnd = (bnd[0] == NL_BOTTOM || bnd[0] == NL_TOP) ? NL_VDIR : NL_UDIR ;
                error = N_AdjustDerivSrf( der[0], 
                                          InBnd,  
                                          bnd[0], 
                                          NL_RIGHT, 
                                          der[0], SI );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* given an end crossDerivative and its not the u=umin */
            if( der[1] NEQ NULL AND bnd[1] NEQ NL_LEFT )
            {
                /* replace deriv surface with one built for u = umin */
                InBnd = (bnd[1] == NL_BOTTOM || bnd[1] == NL_TOP) ? NL_VDIR : NL_UDIR ;
                error = N_AdjustDerivSrf( der[1], 
                                          InBnd, /* NL_UDIR, */
                                          bnd[1], 
                                          NL_LEFT, 
                                          der[1], SI );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        } /* end cross-sections are U isoparameter curves branch - adjusting Deriv Surfaces as needed */

        /* else cross-sections are V isoparameter curves */
        if( dir EQ NL_VDIR )
        {
            /* given a start cross-derivative surface and its not the  v=vmax */ 
            if( der[0] NEQ NULL AND bnd[0] NEQ NL_TOP )
            {
                /* replace deriv surface with one built for v = vmax */
                InBnd = (bnd[0] == NL_BOTTOM || bnd[0] == NL_TOP) ? NL_VDIR : NL_UDIR ;
                error = N_AdjustDerivSrf( der[0], 
                                         InBnd, /* NL_VDIR, */
                                          bnd[0], 
                                          NL_TOP, 
                                          der[0], SI );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* given an end cross-derivative surface and its not the  v=vmin */ 
            if( der[1] NEQ NULL AND bnd[1] NEQ NL_BOTTOM )
            {
                /* replace deriv surface with one built for v = vmin */
                InBnd = (bnd[1] == NL_BOTTOM || bnd[1] == NL_TOP) ? NL_VDIR : NL_UDIR ;
                error = N_AdjustDerivSrf( der[1], 
                                          InBnd, /*  NL_VDIR, */
                                          bnd[1], 
                                          NL_BOTTOM, 
                                          der[1], SI );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        } /* end cross-sections are V isoparameter curve check */
    } /* when given cross-derivative data - make sure it's oriented correctly check */

    /* Prune rational input */

    /* when using rails - must have rail parameters */
    if( rail NEQ NULL AND ts EQ NULL )
        BAD_INPUT;

    /* when specifying cross-derivative data - when possible make sure its nonRational */
    if( der NEQ NULL )
    {
        /* convert given cross-derivatives der surfaces that are rational with equal weights into non-rational curves  */
        if( der[0]NEQ NULL )
            if( N_IsSrfRat( der[0] ) )
                N_SrfPruneRat( der[0] );

        if( der[1]NEQ NULL )
            if( N_IsSrfRat( der[1] ) )
                N_SrfPruneRat( der[1] );
    }

    /* when given rail curves */
    if( rail NEQ NULL )
    {
        /* when given a min param rail curve */
        if( rail[0]NEQ NULL )
        {
            /* promote 2d curves to 3d curves */
            if( NOT N_CrvIs3d( rail[0] ) )
                N_Crv2dTo3d( rail[0] );

            /* convert rail-curve that are rational with equal weights into non-rational curves */
            if( N_IsCrvRat( rail[0] ) )
                N_CrvPruneRat( rail[0], NL_NO );
        }

        /* when given a max param rail curve */
        if( rail[1]NEQ NULL )
        {
            /* promote 2d curves to 3d curves */
            if( NOT N_CrvIs3d( rail[1] ) )
                N_Crv2dTo3d( rail[1] );

            /* convert rail-curve that are rational with equal weights into non-rational curves */
            if( N_IsCrvRat( rail[1] ) )
                N_CrvPruneRat( rail[1], NL_NO );
        }
    } /* end when given rail curves - try to ensure they are nonRational and 3d check */

    /* for every cross-section curve */
    for ( i = 0; i <= lNumSects_1; i++ )
    {
        /* convert 2d cross-section curves into 3d curves */
        if( NOT N_CrvIs3d( cur[i] ) )
            N_Crv2dTo3d( cur[i] );

        /* convert rational equal weight cross-section curves to non-rational curves */
        if( N_IsCrvRat( cur[i] ) )
            N_CrvPruneRat( cur[i], NL_NO );

    } /* end iter every cross-section curve */

    /* Check weights on input */

    /* when given cross-derivative surfaces */
    if( der NEQ NULL )
    {
        /* when given a min param cross-derivative surface */
        if( der[0]NEQ NULL )
        {
            /* if DerivSur is Rational */
            if( N_IsSrfRat( der[0] ) )
            { 
                /* check for illegal weight values - each weight is in the range <NL_WMIN, NL_WMAX> */
                error = N_SrfWeightsAreValid( der[0], rname );

                if( error EQ NL_YES )
                    NL_OUT;
                rat = NL_YES;
            } /* end  cross-deriv surface is rational check */
        } /* end given a min param cross-derivative surface check */

        /* when given a max param cross-derivative surface */
        if( der[1]NEQ NULL )
        {
            /* if DerivSur is Rational */
            if( N_IsSrfRat( der[1] ) )
            {
                /* check for illegal weight values - each weight is in the range <NL_WMIN, NL_WMAX> */
                error = N_SrfWeightsAreValid( der[1], rname );

                if( error EQ NL_YES )
                    NL_OUT;
                rat = NL_YES;
            } /* end  cross-deriv surface is rational check */
        } /* end given a max param cross-derivative surface check */
    } /* end given cross-derivative surfaces check */

    /* when given rail curves - check for rational curves - check for valid weight values */
    if( rail NEQ NULL )
    {
        if( rail[0]NEQ NULL )
        {
            if( N_IsCrvRat( rail[0] ) )
            {
                error = N_CrvWeightsAreValid( rail[0], rname );

                if( error EQ NL_YES )
                    NL_OUT;
                rat = NL_YES;
            }
        }

        if( rail[1]NEQ NULL )
        {
            if( N_IsCrvRat( rail[1] ) )
            {
                error = N_CrvWeightsAreValid( rail[1], rname );

                if( error EQ NL_YES )
                    NL_OUT;
                rat = NL_YES;
            }
        }
    } /* end when given rail curves - check for rational curves - check for valid weight values */

    /* for every cross-section curve - check for rational curves - check for valid weight values */
    for ( i = 0; i <= lNumSects_1; i++ )
    {
        if( N_IsCrvRat( cur[i] ) )
        {
            error = N_CrvWeightsAreValid( cur[i], rname );

            if( error EQ NL_YES )
                NL_OUT;
            rat = NL_YES;
        }
    }  /* end iter every cross-section curve - checking for rational curves - checking for valid weight values */

    /* when any input curve is rational - make all curves rational */
    if( rat EQ NL_YES )
    {
        /* Make non-rationals rational */

        /* when given cross-derivative curves - make them Rational */
        if( der NEQ NULL )
        {
            if( der[0]NEQ NULL )
                if( NOT N_IsSrfRat( der[0] ) )
                    N_SrfNonRatToRat( der[0] );

            if( der[1]NEQ NULL )
                if( NOT N_IsSrfRat( der[1] ) )
                    N_SrfNonRatToRat( der[1] );
        } /* end when given cross-derivative curves - make them Rational check */

        /* when given rail curves - make them Rational */
        if( rail NEQ NULL )
        {
            if( rail[0]NEQ NULL )
                if( NOT N_IsCrvRat( rail[0] ) )
                    N_CrvNonRatToRat( rail[0] );

            if( rail[1]NEQ NULL )
                if( NOT N_IsCrvRat( rail[1] ) )
                    N_CrvNonRatToRat( rail[1] );
        } /* end when given rail curves - make them Rational check */

        /* for every cross-section curve - make nonRational curves Rational */
        for ( i = 0; i <= lNumSects_1; i++ )
        {
            if( NOT N_IsCrvRat( cur[i] ) )
                N_CrvNonRatToRat( cur[i] );
        } /* end iter every cross-section curve - making nonRational curves Rational */

        /* when given rail curves - check intersections with rails in 4-D */
        if( rail NEQ NULL )
        {
            /* for every cross-section */
            for ( i = 0; i <= lNumSects_1; i++ )
            {
                reset = NL_NO;

                /* get cross-section curve start/end control point and weight */
                N_CrvGetCPts( cur[i], &nc, &Cw );
                N_CPtToPtAndW( Cw[0], &Ca, &wa );
                N_CPtToPtAndW( Cw[nc], &Cb, &wb );

                /* when given a start rail curve */
                if( rail[0]NEQ NULL )
                {
                    /* evalute rail curve at given matching param value for this cross-section curve */
                    error = N_CrvEvalPt( rail[0], ts[i], NL_LEFT, &Aw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CPtToPtAndW( Aw, &A, &ra );

                    /* compare the rail/sectionCurve point-to-point distance */
                    N_DistPtPt( Ca, A, &dist );

                    /* out of tolerance distances are an error */
                    if( dist GT NL_MTOL )
                        BAD_INPUT;

                    /* when weights are different */
                    if( fabs( wa - ra )GT NL_WTOL )
                    {
                        /* change cross-section curve end-point weight */
                        wa = ra;
                        reset = NL_YES;
                    }
                } /* end when given a start rail curve */

                /* when given an end rail curve */
                if( rail[1]NEQ NULL )
                {
                    /* evaluate rail curve at given mathcing param value for this cross-section curve */
                    error = N_CrvEvalPt( rail[1], ts[i], NL_LEFT, &Bw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CPtToPtAndW( Bw, &B, &rb );
                    
                    /* compare the rail/sectionCurve point-to-point distance */
                    N_DistPtPt( Cb, B, &dist );

                    /* out of tolerance distances are an error */
                    if( dist GT NL_MTOL )
                        BAD_INPUT;

                    /* when weights are different */
                    if( fabs( wb - rb )GT NL_WTOL )
                    {
                        /* change cross-section curve end-point weight */
                        wb = rb;
                        reset = NL_YES;
                    }
                } /* end when given a end rail curve */

                /* when either cross-section curve endPoint weight was changed */
                if( reset EQ NL_YES )
                {
                    /* rescale the weights of the cross-section curve to make them compatible with the rail curve weights */
                    error = N_SrfReparamWeights( cur[i], wa, wb );

                    if( error EQ NL_YES )
                        NL_OUT;
                } /* end when either cross-section curve endPoint weight was changed check */
            } /* end iter every cross-section curve checking rail/cross-section intersections for compatibility */

            /* Translate if rails' end tangents pass thru the global origin */

            /* when given a start rail curve - remember if start rail tangent points at the origin */
            if( rail[0]NEQ NULL )
            {
                N_CrvGetCPts( rail[0], &nr, &R0w );

                if( N_CPtsAreColinear( ORIGIN, R0w[0], R0w[1], NL_THREED ) )
                    trl = NL_YES;

                if( N_CPtsAreColinear( R0w[nr - 1], R0w[nr], ORIGIN, NL_THREED ) )
                    trl = NL_YES;
            }

            /* when given an end rail curve - remember if end rail tangent points at the origin */
            if( rail[1]NEQ NULL )
            {
                N_CrvGetCPts( rail[1], &nr, &R1w );

                if( N_CPtsAreColinear( ORIGIN, R1w[0], R1w[1], NL_THREED ) )
                    trl = NL_YES;

                if( N_CPtsAreColinear( R1w[nr - 1], R1w[nr], ORIGIN, NL_THREED ) )
                    trl = NL_YES;
            }

            /* when either rail curve's end tangent points at the origin */
            if( trl EQ NL_YES )
            {
                /* seek a translation vector */

                /* let transVec = average of the skinned surface corners */
                N_CrvGetCPts( cur[0], &nc, &Cw );

                N_CPtToPtEuclid( Cw[0], &T );
                N_CPtToPtEuclid( Cw[nc], &A );
                N_Sum2Pts( T, A, &T );

                N_CrvGetCPts( cur[lNumSects_1], &nc, &Cw );

                N_CPtToPtEuclid( Cw[0], &A );
                N_Sum2Pts( T, A, &T );
                N_CPtToPtEuclid( Cw[nc], &A );
                N_Sum2Pts( T, A, &T );

                N_ScalePt( 0.25, T, &T );

                N_DistPtPt( NL_ZERO, T, &dist );

                /* if translation by the the corner average point happens to be zero */
                if( dist LE NL_MTOL )
                {
                    /* let transVec = MaxPosition Vector of a given rail curve */
                    if( rail[0]NEQ NULL )
                        N_CrvGetMaxPosVector( rail[0], &T, &fact );

                    else if( rail[1]NEQ NULL )
                        N_CrvGetMaxPosVector( rail[1], &T, &fact );
                }

                /* translate the rail curves */
                if( rail[0] NEQ NULL )
                    N_CrvTranslate( rail[0], T );

                if( rail[1 ]NEQ NULL )
                    N_CrvTranslate( rail[1], T );

                /* translate the cross-derivative surfaces */
                if( der NEQ NULL )
                {
                    if( der[0] NEQ NULL )
                        N_SrfTranslate( der[0], T );

                    if( der[1] NEQ NULL )
                        N_SrfTranslate( der[1], T );
                }

                /* translate the cross-section curves */
                for ( i = 0; i <= lNumSects_1; i++ )
                    N_CrvTranslate( cur[i], T );

            } /* when need to translate the input check */

            /* Lift rows or columns of der surface control points */

            /* when given cross-deriv surfaces */
            if( der NEQ NULL )
            {
                /* when cross-section curves are U isoparameter curves */
                if( dir EQ NL_UDIR )
                {
                    /* for start cross-derivative surface */
                    if( der[0] NEQ NULL )
                    {
                        N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                        SUL = N_AllocCPt2dArray( 1, ms, &SL );

                        if( SUL EQ NULL )
                            NL_QUIT;

                        l = 0;

                        /* make an array of the last two U Control Points by every V control point */
                        /* This captures the Du(u=Umax,v) curve from the input cross-derivative surface */
                        for ( i = ns - 1; i <= ns; i++ )
                        {
                            for ( j = 0; j <= ms; j++ )
                                N_CopyCPt( Sw[i][j], &SUL[l][j] );
                            l++;
                        }
                    } /* end when given start cross-derivative - build the SUL array capturing Du(u=Umax,v) information check */

                    /* for end cross-derivative surface */
                    if( der[1] NEQ NULL )
                    {
                        N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                        SUR = N_AllocCPt2dArray( 1, ms, &SL );

                        if( SUR EQ NULL )
                            NL_QUIT;

                        /* make an array of the first two U Control Points by every V control point */
                        /* This captures the Du(u=Umin,v) curve from the input cross-derivative surface */
                        for ( i = 0; i <= 1; i++ )
                        {
                            for ( j = 0; j <= ms; j++ )
                                N_CopyCPt( Sw[i][j], &SUR[i][j] );
                        }
                    } /* end when given end cross-derivative - build the SUR array capturing Du(u=Umin,v) information check */
                } /* end when cross-section curves are U isoparameter curves check */

                /* when cross-section curves are V isoparameter curves */
                if( dir EQ NL_VDIR )
                {
                    /* for start cross-derivative surface */
                    if( der[0]NEQ NULL )
                    {
                        N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                        SVB = N_AllocCPt2dArray( ns, 1, &SL );

                        if( SVB EQ NULL )
                            NL_QUIT;

                        l = 0;

                        /* make an array of the last two V Control Points by every U control point */
                        /* This captures the Dv(u,v=Vmax) curve from the input cross-derivative surface */
                        for ( j = ms - 1; j <= ms; j++ )
                        {
                            for ( i = 0; i <= ns; i++ )
                                N_CopyCPt( Sw[i][j], &SVB[i][l] );
                            l++;
                        }
                    } /* end when given start cross-derivative - build the SVB array capturing Dv(u,v=Vmax) information check */

                    /* for end cross-derivative surface */
                    if( der[1]NEQ NULL )
                    {
                        N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                        SVT = N_AllocCPt2dArray( ns, 1, &SL );

                        if( SVT EQ NULL )
                            NL_QUIT;

                        /* make an array of the first two V Control Points by every U control point */
                        /* This captures the Dv(u,v=Vmin) curve from the input cross-derivative surface */
                        for ( j = 0; j <= 1; j++ )
                        {
                            for ( i = 0; i <= ns; i++ )
                                N_CopyCPt( Sw[i][j], &SVT[i][j] );
                        }
                    } /* end when given end cross-derivative - build the SVT array capturing Dv(u,v=Vmin) information check */
                } /* end when cross-section curves are V isoparameter curves check */

                /* Check NL_G1 continuity of rails and der surface */
                /* Two rails --> must be NL_G1 in 4-D              */

                /* when given both rail curves */
                if( rail[0] NEQ NULL AND rail[1] NEQ NULL )
                {
                    N_CrvGetCPts( rail[0], &nr, &R0w );
                    N_CrvGetCPts( rail[1], &nr, &R1w );

                    /* when cross-section curves are U isoparameters */
                    if( dir EQ NL_UDIR )
                    {
                        /* for start cross-derivative surface */
                        if( der[0] NEQ NULL )
                        {
                            N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                            /* check for G1 continuity between rail tangent and start cross-derivative end points */
                            /* G1 constraint is */
                            /*   Rail-EndPoint = DirevSurfCorner Point (already checked) and       */
                            /*   Rail-EndVec(Rail-EndPoint,RailNextToEndPoint) ColinearTo          */
                            /*      DerivSurfCornerVec(DerivEndPoint, DerivNextToEndPoint)        */
                            if( NOT N_CPtsAreColinear( SUL[0][0], SUL[1][0], R0w[1], NL_FOURD ) )
                                BAD_INPUT;

                            if( NOT N_CPtsAreColinear( SUL[0][ms], SUL[1][ms], R1w[1], NL_FOURD ) )
                                BAD_INPUT;
                        }

                        /* for end cross-derivative surface */
                        if( der[1] NEQ NULL )
                        {
                            N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                            /* check for G1 continuity between rail tangent and start cross-derivative end points */
                            if( NOT N_CPtsAreColinear( R0w[nr - 1], SUR[0][0], SUR[1][0], NL_FOURD ) )
                                BAD_INPUT;

                            if( NOT N_CPtsAreColinear( R1w[nr - 1], SUR[0][ms], SUR[1][ms], NL_FOURD ) )
                                BAD_INPUT;
                        }
                    } /* end when cross-section curves are U isoparameters check */

                    /* when cross-section curves are V isoparameters */
                    if( dir EQ NL_VDIR )
                    {
                        /* for start cross-derivaitve surface */
                        if( der[0]NEQ NULL )
                        {
                            N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                            /* check for G1 continuity between rail tangent and start cross-derivative end points */
                            if( NOT N_CPtsAreColinear( SVB[0][0], SVB[0][1], R0w[1], NL_FOURD ) )
                                BAD_INPUT;

                            if( NOT N_CPtsAreColinear( SVB[ns][0], SVB[ns][1], R1w[1], NL_FOURD ) )
                                BAD_INPUT;
                        }

                        /* for end cross-derivaitve surface */
                        if( der[1]NEQ NULL )
                        {
                            N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                            /* check for G1 continuity between rail tangent and start cross-derivative end points */
                            if( NOT N_CPtsAreColinear( R0w[nr - 1], SVT[0][0], SVT[0][1], NL_FOURD ) )
                                BAD_INPUT;

                            if( NOT N_CPtsAreColinear( R1w[nr - 1], SVT[ns][0], SVT[ns][1], NL_FOURD ) )
                                BAD_INPUT;
                        }
                    } /* when cross-section curves are V isoparameters check */
                } /* end when given both rail curves check */
                else

                /* One rail only --> if not NL_G1 in 4-D, will adjust second */
                /* row of der                                                */

                /* when only given a start rail curve */
                if( rail[0] NEQ NULL )
                {
                    N_CrvGetCPts( rail[0], &nr, &R0w );

                    /* when cross-section curves are U isoparameter curves */
                    if( dir EQ NL_UDIR )
                    {
                        /* when given a start cross-derivative surf and continuity is not C1 */
                        if( der[0] NEQ NULL AND cnt[0] NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                            /* check for euclidean colinearity */
                            if( NOT N_CPtsAreColinear( R0w[1], SUL[1][0], SUL[0][0], NL_THREED ) )
                                BAD_INPUT;

                            /* when end cpts are not colinear in 4D space */
                            if( NOT N_CPtsAreColinear( R0w[1], SUL[1][0], SUL[0][0], NL_FOURD ) )
                            {
                                /* compute a new weight for SUL[0][0], the cross derivative end control point */
                                error = N_AdjustWeightScale( R0w[1], SUL[1][0], SUL[0][0], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                /* apply the weight change to the rest of the derivative vectors */
                                for ( j = 0; j <= ms; j++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SUL[1][j], fact, SUL[0][j], &SUL[0][j] );
                                }

                                N_CPtGetW( SUL[0][0], &wb );
                                fact = wa / wb;

                                for ( j = 0; j <= ms; j++ )
                                    N_ScaleCPt( fact, SUL[0][j], &SUL[0][j] );
                            }
                        } /* end when given a start cross-derivative surf and continuity is not C1 check */

                        /* when given an end cross-derivative surface and continuity is not C1 check */
                        if( der[1] NEQ NULL AND cnt[1] NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                            /* check for colinear euclidean end vecs */
                            if( NOT N_CPtsAreColinear( R0w[nr - 1], SUR[0][0], SUR[1][0], NL_THREED ) )
                                BAD_INPUT;

                            /* when not colinear in 4D homogeneous space - adjust the cross derivative weight */
                            if( NOT N_CPtsAreColinear( R0w[nr - 1], SUR[0][0], SUR[1][0], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R0w[nr - 1], SUR[0][0], SUR[1][0], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( j = 0; j <= ms; j++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SUR[0][j], fact, SUR[1][j], &SUR[1][j] );
                                }

                                N_CPtGetW( SUR[1][0], &wb );
                                fact = wa / wb;

                                for ( j = 0; j <= ms; j++ )
                                    N_ScaleCPt( fact, SUR[1][j], &SUR[1][j] );
                            }
                        } /* end when given an end cross-derivative surface and continuity is not C1 check */
                    } /* end when cross-section curves are U isoparameter curves check */

                    /* when cross-section curves are V isoparameter curves */
                    if( dir EQ NL_VDIR )
                    {
                        if( der[0]NEQ NULL AND cnt[0]NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                            if( NOT N_CPtsAreColinear( R0w[1], SVB[0][1], SVB[0][0], NL_THREED ) )
                                BAD_INPUT;

                            if( NOT N_CPtsAreColinear( R0w[1], SVB[0][1], SVB[0][0], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R0w[1], SVB[0][1], SVB[0][0], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( i = 0; i <= ns; i++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SVB[i][1], fact, SVB[i][0], &SVB[i][0] );
                                }

                                N_CPtGetW( SVB[0][0], &wb );
                                fact = wa / wb;

                                for ( i = 0; i <= ns; i++ )
                                    N_ScaleCPt( fact, SVB[i][0], &SVB[i][0] );
                            }
                        }

                        /* when given an end cross-derivative and continuity is not C1 check */
                        if( der[1]NEQ NULL AND cnt[1] NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                            /* check euclidean colinearity */
                            if( NOT N_CPtsAreColinear( R0w[nr - 1], SVT[0][0], SVT[0][1], NL_THREED ) )
                                BAD_INPUT;

                            /* when end vecs are not colinear in homogeneous space - adjust cross-deriv weights */
                            if( NOT N_CPtsAreColinear( R0w[nr - 1], SVT[0][0], SVT[0][1], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R0w[nr - 1], SVT[0][0], SVT[0][1], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( i = 0; i <= ns; i++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SVT[i][0], fact, SVT[i][1], &SVT[i][1] );
                                }

                                N_CPtGetW( SVT[0][1], &wb );
                                fact = wa / wb;

                                for ( i = 0; i <= ns; i++ )
                                    N_ScaleCPt( fact, SVT[i][1], &SVT[i][1] );
                            }
                        } /* end when given an end cross-derivative surface and continuity is not C1 check */
                    } /* when cross-section curves are V isoparameter curves check */
                } /* end when only given a start rail curve check */
                else

                /* Check other rail now */

                /* when only given an end rail curve */
                if( rail[1]NEQ NULL )
                {
                    N_CrvGetCPts( rail[1], &nr, &R1w );

                    /* when cross-section curves are U isoparameter curves */
                    if( dir EQ NL_UDIR )
                    {
                        /* when given a start cross-derivative and continuity is not C1 check */
                        if( der[0]NEQ NULL AND cnt[0]NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                            /* check for euclidean colinearity */
                            if( NOT N_CPtsAreColinear( R1w[1], SUL[1][ms], SUL[0][ms], NL_THREED ) )
                                BAD_INPUT;

                            /* when end vexs are not colinaer in homogeneous space - adjust cross derivative weights */
                            if( NOT N_CPtsAreColinear( R1w[1], SUL[1][ms], SUL[0][ms], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R1w[1], SUL[1][ms], SUL[0][ms], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( j = 0; j <= ms; j++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SUL[1][j], fact, SUL[0][j], &SUL[0][j] );
                                }

                                N_CPtGetW( SUL[0][ms], &wb );
                                fact = wa / wb;

                                for ( j = 0; j <= ms; j++ )
                                    N_ScaleCPt( fact, SUL[0][j], &SUL[0][j] );
                            }
                        } /* end when given a start cross-derivative and continuity is not C1 check */

                        /* when given an end cross-derivative and continuity is not C1 check */
                        if( der[1]NEQ NULL AND cnt[1]NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                            /* check endVec euclidean colinearity */
                            if( NOT N_CPtsAreColinear( R1w[nr - 1], SUR[0][ms], SUR[1][ms], NL_THREED ) )
                                BAD_INPUT;

                            /* when end vecs arenot colinear in homogeneous space - adjust cross-derivative weights */
                            if( NOT N_CPtsAreColinear( R1w[nr - 1], SUR[0][ms], SUR[1][ms], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R1w[nr - 1], SUR[0][ms], SUR[1][ms], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( j = 0; j <= ms; j++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SUR[0][j], fact, SUR[1][j], &SUR[1][j] );
                                }

                                N_CPtGetW( SUR[1][ms], &wb );
                                fact = wa / wb;

                                for ( j = 0; j <= ms; j++ )
                                    N_ScaleCPt( fact, SUR[1][j], &SUR[1][j] );
                            }
                        } /* end when given an end cross-derivative and continuity is not C1 check */
                    } /* end cross-sections are U isoparamter curves check */

                    /* when cross-section curves are V isoparameter curves */
                    if( dir EQ NL_VDIR )
                    {
                        /* start cross-derivative surface and continuity is not C1 check */
                        if( der[0]NEQ NULL AND cnt[0]NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[0], &ns, &ms, &Sw );

                            /* check endVec euclidean colinearity */
                            if( NOT N_CPtsAreColinear( R1w[1], SVB[ns][1], SVB[ns][0], NL_THREED ) )
                                BAD_INPUT;

                            /* when endvecs are not colinear in homogeneous space - adjust derivSurface weights */
                            if( NOT N_CPtsAreColinear( R1w[1], SVB[ns][1], SVB[ns][0], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R1w[1], SVB[ns][1], SVB[ns][0], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( i = 0; i <= ns; i++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SVB[i][1], fact, SVB[i][0], &SVB[i][0] );
                                }

                                N_CPtGetW( SVB[ns][0], &wb );
                                fact = wa / wb;

                                for ( i = 0; i <= ns; i++ )
                                    N_ScaleCPt( fact, SVB[i][0], &SVB[i][0] );
                            } /* end adjusting derivSurface weights to make endVecs colinear in homogeneous space */
                        } /* end start cross-derivative surface and continuity is not C1 check */

                        /* end cross-derivative surface and continuity is not C1 check */
                        if( der[1]NEQ NULL AND cnt[1]NEQ NL_C1 )
                        {
                            N_SrfGetCPts( der[1], &ns, &ms, &Sw );

                            /* check end vecs are colinear in euclidean space */
                            if( NOT N_CPtsAreColinear( R1w[nr - 1], SVT[ns][0], SVT[ns][1], NL_THREED ) )
                                BAD_INPUT;

                            /* when endvecs are not colinear in homogeneous space - adjust derivSurface weights */
                            if( NOT N_CPtsAreColinear( R1w[nr - 1], SVT[ns][0], SVT[ns][1], NL_FOURD ) )
                            {
                                error = N_AdjustWeightScale( R1w[nr - 1], SVT[ns][0], SVT[ns][1], &wa, &fact );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                for ( i = 0; i <= ns; i++ )
                                {
                                    N_Combine2CPts( 1.0 - fact, SVT[i][0], fact, SVT[i][1], &SVT[i][1] );
                                }

                                N_CPtGetW( SVT[ns][1], &wb );
                                fact = wa / wb;

                                for ( i = 0; i <= ns; i++ )
                                    N_ScaleCPt( fact, SVT[i][1], &SVT[i][1] );
                            } /* end adjusting derivSurface weights to make endVecs colinear in homogeneous space */
                        } /* end start cross-derivative surface and continuity is not C1 check */

                    } /* end when cross-section curves are end iso-parameter curves */
                } /* when only given an end rail curve check */

            } /* end when given cross-deriv surfaces check - make sure cross derives and rails are G1 continuous */
        } /* end when given rail curves - need to check intersections with rails in 4-D check */
    } /* end any input curve is rational check */

    /* arrive here once rail curves and cross-section curves are fixed to be compatible at intersection points */

    /* Set some flags and get locals */

    /* rail locals and flags */
    if( rail NEQ NULL )
    {
        if( rail[0] NEQ NULL )
        {
            N_CrvGetCPtsDegreeAndKnots( rail[0], &nr, &R0w, &pr, &mr, &UR );
            N_CrvGetKnotVector( rail[0], &knr );
        }
        else if( rail[1] NEQ NULL )
        {
            N_CrvGetCPtsDegreeAndKnots( rail[1], &nr, &R1w, &pr, &mr, &UR );
            N_CrvGetKnotVector( rail[1], &knr );
        }

        if( rail[0] NEQ NULL AND rail[1] EQ NULL )
            rfl = NL_LEFT;

        else if( rail[0] EQ NULL AND rail[1] NEQ NULL )
            rfl = NL_RIGHT;

        else if( rail[0] NEQ NULL AND rail[1] NEQ NULL )
            rfl = NL_BOTH;
    }

    /* cross-derivative locals and flags */
    if( der NEQ NULL )
    {
        if( der[0]NEQ NULL AND der[1]EQ NULL )
            dfl = NL_LEFT;

        else if( der[0]EQ NULL AND der[1]NEQ NULL )
            dfl = NL_RIGHT;

        else if( der[0]NEQ NULL AND der[1]NEQ NULL )
            dfl = NL_BOTH;
    }

    /* Get cross-boundary derivatives */

    /* when given cross-derivatives */
    if( der NEQ NULL )
    {
        if( rat EQ NL_NO OR( rat EQ NL_YES AND rail EQ NULL ) )
        {
            if( der[0]NEQ NULL )
            {
                N_CrvInitArrays( &curA );

                if( dir EQ NL_UDIR )
                {
                    error = N_CrossBoundDerivCrvNonRatSrf( der[0], NL_RIGHT, &curA, SI );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else if( dir EQ NL_VDIR )
                {
                    /* let curA = cross boundary derivative of der[0]                    */
                    /*   the control points of curA are computed as differences          */
                    /*   of the der[0] control points running along the target boundary. */
                    error = N_CrossBoundDerivCrvNonRatSrf( der[0], NL_TOP, &curA, SI );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            if( der[1]NEQ NULL )
            {
                N_CrvInitArrays( &curB );

                if( dir EQ NL_UDIR )
                {
                    error = N_CrossBoundDerivCrvNonRatSrf( der[1], NL_LEFT, &curB, SI );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else if( dir EQ NL_VDIR )
                {
                    error = N_CrossBoundDerivCrvNonRatSrf( der[1], NL_BOTTOM, &curB, SI );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            if( der[0]NEQ NULL )
            {
                N_SrfGetCPts( der[0], &ns, &ms, &Sw );
                N_SrfGetDegrees( der[0], &ps, &qs );
                N_SrfGetKnots( der[0], &rs, &ss, &US, &VS );
                N_SrfGetKnotVectors( der[0], &knu, &knv );

                if( dir EQ NL_UDIR )
                {
                    pol = N_AllocCPolygonAndArray( ms, &SL );

                    if( pol EQ NULL )
                        NL_QUIT;

                    fact = ps / (US[ns + 1] - US[ns]);

                    N_CPolygonGetCPts( pol, &ms, &Pw );

                    for ( j = 0; j <= ms; j++ )
                    {
                        N_Diff2CPts( SUL[1][j], SUL[0][j], &Pw[j] );
                        N_ScaleCPt( fact, Pw[j], &Pw[j] );
                    }
                    N_CrvFromCPolygonAndKnotVector( &curA, pol, qs, knv );
                }

                if( dir EQ NL_VDIR )
                {
                    pol = N_AllocCPolygonAndArray( ns, &SL );

                    if( pol EQ NULL )
                        NL_QUIT;

                    fact = qs / (VS[ms + 1] - VS[ms]);

                    N_CPolygonGetCPts( pol, &ns, &Pw );

                    for ( i = 0; i <= ns; i++ )
                    {
                        N_Diff2CPts( SVB[i][1], SVB[i][0], &Pw[i] );
                        N_ScaleCPt( fact, Pw[i], &Pw[i] );
                    }
                    N_CrvFromCPolygonAndKnotVector( &curA, pol, ps, knu );
                }
            }

            if( der[1]NEQ NULL )
            {
                N_SrfGetCPts( der[1], &ns, &ms, &Sw );
                N_SrfGetDegrees( der[1], &ps, &qs );
                N_SrfGetKnots( der[1], &rs, &ss, &US, &VS );
                N_SrfGetKnotVectors( der[1], &knu, &knv );

                if( dir EQ NL_UDIR )
                {
                    pol = N_AllocCPolygonAndArray( ms, &SL );

                    if( pol EQ NULL )
                        NL_QUIT;

                    fact = ps / (US[ps + 1] - US[ps]);

                    N_CPolygonGetCPts( pol, &ms, &Pw );

                    for ( j = 0; j <= ms; j++ )
                    {
                        N_Diff2CPts( SUR[1][j], SUR[0][j], &Pw[j] );
                        N_ScaleCPt( fact, Pw[j], &Pw[j] );
                    }
                    N_CrvFromCPolygonAndKnotVector( &curB, pol, qs, knv );
                }

                if( dir EQ NL_VDIR )
                {
                    pol = N_AllocCPolygonAndArray( ns, &SL );

                    if( pol EQ NULL )
                        NL_QUIT;

                    fact = qs / (VS[qs + 1] - VS[qs]);

                    N_CPolygonGetCPts( pol, &ns, &Pw );

                    for ( i = 0; i <= ns; i++ )
                    {
                        N_Diff2CPts( SVT[i][1], SVT[i][0], &Pw[i] );
                        N_ScaleCPt( fact, Pw[i], &Pw[i] );
                    }
                    N_CrvFromCPolygonAndKnotVector( &curB, pol, ps, knu );
                }
            }
        }
    } /* end when given cross-derivatives check */

    /* Check NL_C1 continuity */

    /* when given both rails and cross-derivative surfaces */
    if( der NEQ NULL AND rail NEQ NULL )
    {
        if( rail[0]NEQ NULL )
        {
            if( der[0]NEQ NULL AND cnt[0]EQ NL_C1 )
            {
                error = N_CrvDerivsAtKnot( rail[0], UR[0], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPts( &curA, &nd, &Dw );

                if( NOT N_CPtsAreEqual( Dw[0], RDw[1] ) )
                    BAD_INPUT;
            }

            if( der[1]NEQ NULL AND cnt[1]EQ NL_C1 )
            {
                error = N_CrvDerivsAtKnot( rail[0], UR[mr], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPts( &curB, &nd, &Dw );

                if( NOT N_CPtsAreEqual( Dw[0], RDw[1] ) )
                    BAD_INPUT;
            }
        }

        if( rail[1]NEQ NULL )
        {
            if( der[0]NEQ NULL AND cnt[0]EQ NL_C1 )
            {
                error = N_CrvDerivsAtKnot( rail[1], UR[0], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPts( &curA, &nd, &Dw );

                if( NOT N_CPtsAreEqual( Dw[nd], RDw[1] ) )
                    BAD_INPUT;
            }

            if( der[1]NEQ NULL AND cnt[1]EQ NL_C1 )
            {
                error = N_CrvDerivsAtKnot( rail[1], UR[mr], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPts( &curB, &nd, &Dw );

                if( NOT N_CPtsAreEqual( Dw[nd], RDw[1] ) )
                    BAD_INPUT;
            }
        }
    } /* end when given both rails and cross-derivative surfaces check */

    /* Make all curves compatible */

    /* when cross-section curves are not compatible on input */
    if( cmp EQ NL_NO )
    {
        curM = N_AllocArrayCrvPtrs( lNumSects_1 + 2, SI );

        if( curM EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= lNumSects_1; i++ )
            curM[i] = cur[i];

        l = lNumSects_1;

        if( der NEQ NULL )
        {
            if( der[0]NEQ NULL )
                curM[++l] = &curA;

            if( der[1]NEQ NULL )
                curM[++l] = &curB;
        }

        error = N_CrvsMakeCompatibleKnotTol( curM, l, 0, SI );

        if( error EQ NL_YES )
            NL_OUT;
    } /* end when cross-section curves are not compatible on input */

    N_CrvGetCPtsDegreeAndKnots( cur[0], &nc, &Cw, &pc, &mc, &UC );

    /* Get derivative data */

    Qw = N_AllocCPt1dArray( lNumSects_1, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    /* when given cross-derivative end constraints */
    if( der NEQ NULL )
    {
        sdr = NL_NO;

        if( der[0] NEQ NULL AND cnt[0] NEQ NL_C1 )
            sdr = NL_YES;

        if( der[1] NEQ NULL AND cnt[1] NEQ NL_C1 )
            sdr = NL_YES;

        if( sdr EQ NL_NO )
        {
            /* No need to compute magnitudes */

            if( der[0]NEQ NULL )
                curDA = &curA;

            if( der[1]NEQ NULL )
                curDB = &curB;
        }
        else
        {
            /* Compute chordal lengths along the ends */

            if( rfl EQ NL_LEFT OR rfl EQ NL_BOTH )
            {
                error = N_CrvDerivsAtKnot( rail[0], UR[0], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;
                N_CPtMagnitude( RDw[1], &L00 );

                error = N_CrvDerivsAtKnot( rail[0], UR[mr], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;
                N_CPtMagnitude( RDw[1], &L01 );

                if( rfl EQ NL_LEFT )
                {
                    for ( i = 0; i <= lNumSects_1; i++ )
                    {
                        error = N_CrvEvalPt( rail[0], ts[i], NL_LEFT, &Qw[i] );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    N_DistCPolygon( Qw, lNumSects_1, &L0 );
                }
            }
            else
            {
                for ( i = 0; i <= lNumSects_1; i++ )
                {
                    N_CrvGetCPts( cur[i], &nc, &Cw );
                    N_CopyCPt( Cw[0], &Qw[i] );
                }
                N_DistCPolygon( Qw, lNumSects_1, &L00 );
                L01 = L00;
            }

            if( rfl EQ NL_RIGHT OR rfl EQ NL_BOTH )
            {
                error = N_CrvDerivsAtKnot( rail[1], UR[0], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;
                N_CPtMagnitude( RDw[1], &L10 );

                error = N_CrvDerivsAtKnot( rail[1], UR[mr], NL_LEFT, 1, RDw );

                if( error EQ NL_YES )
                    NL_OUT;
                N_CPtMagnitude( RDw[1], &L11 );

                if( rfl EQ NL_RIGHT )
                {
                    for ( i = 0; i <= lNumSects_1; i++ )
                    {
                        error = N_CrvEvalPt( rail[1], ts[i], NL_LEFT, &Qw[i] );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    N_DistCPolygon( Qw, lNumSects_1, &L1 );
                }
            }
            else
            {
                for ( i = 0; i <= lNumSects_1; i++ )
                {
                    N_CrvGetCPts( cur[i], &nc, &Cw );
                    N_CopyCPt( Cw[nc], &Qw[i] );
                }
                N_DistCPolygon( Qw, lNumSects_1, &L10 );
                L11 = L10;
            }

            /* Set magnitudes for one rail */

            if( rfl EQ NL_LEFT )
            {
                L10 = (L10 / L0) * L00;
                L11 = (L11 / L0) * L01;
            }
            else if( rfl EQ NL_RIGHT )
            {
                L00 = (L00 / L1) * L10;
                L01 = (L01 / L1) * L11;
            }

            /* Adjust magnitudes across cur[0] */

            mll = NL_NO;

            if( der[0]NEQ NULL )
            {
                /* Set some flags */

                if( cnt[0]NEQ NL_C1 )
                {
                    /* See if end magnitudes match up with der */

                    sdr = NL_YES;
                    N_CrvGetCPts( &curA, &nd, &Dw );

                    N_CPtMagnitude( Dw[0], &mg0 );

                    if( mg0 LT L00 )
                        fac0 = mg0 / L00;
                    else
                        fac0 = L00 / mg0;

                    if( fac0 LT dper )
                        sdr = NL_NO;

                    N_CPtMagnitude( Dw[nd], &mgn );

                    if( mgn LT L10 )
                        fac1 = mgn / L10;
                    else
                        fac1 = L10 / mgn;

                    if( fac1 LT dper )
                        sdr = NL_NO;

                    /* See if end magnitudes are the same */

                    lsm = NL_NO;

                    if( rfl EQ NL_NO )
                    {
                        if( L00 LT L10 )
                            fact = L00 / L10;
                        else
                            fact = L10 / L00;

                        if( fact GE lper )
                            lsm = NL_YES;
                    }

                    /* See if magnitude opposite to rail matches up with der */

                    opr = NL_NO;

                    if( rfl EQ NL_LEFT OR rfl EQ NL_RIGHT )
                    {
                        if( rfl EQ NL_LEFT AND fac1 GE dper )
                            opr = NL_YES;

                        else if( rfl EQ NL_RIGHT AND fac0 GE dper )
                            opr = NL_YES;
                    }
                }

                /* Get derivative curves */

                if( cnt[0]EQ NL_C1 OR( rfl EQ NL_NO AND sdr EQ NL_YES ) )
                {
                    /* No adjustment needed */

                    curDA = &curA;
                }
                else if( lsm EQ NL_YES OR opr EQ NL_YES )
                {
                    /* Derivative curve is scaled by a constant */

                    if( lsm EQ NL_YES )
                    {
                        d1 = L00 / mg0;
                        d2 = L10 / mgn;
                        alf = 0.5 *( d1 + d2 );
                    }
                    else if( opr EQ NL_YES )
                    {
                        if( rfl EQ NL_LEFT )
                            alf = L00 / mg0;

                        else if( rfl EQ NL_RIGHT )
                            alf = L10 / mgn;
                    }

                    N_CrvInitArrays( &curPA );
                    error = N_CrvCopy( &curA, &curPA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_ConstantMultiplyCrv4d( alf, &curPA );

                    curDA = &curPA;
                }
                else
                {
                    /* Derivative curve is scaled by a function */

                    error = N_AllocCFuncArrays( &cfnA, 1, 1, 3, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CrvFuncCntrlValKnots( &cfnA, &fu, &UF );

                    fu[0] = L00 / mg0;
                    fu[1] = L10 / mgn;
                    UF[0] = UF[1] = 0.0;
                    UF[2] = UF[3] = 1.0;

                    N_CrvInitArrays( &curPA );
                    error = N_CrvFuncMultiplyCrv4d( &cfnA, &curA, &curPA, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    curDA = &curPA;

                    if( NOT N_Crv4dIsDegen( &curA ) )
                        dec = NL_YES;

                    mll = NL_YES;
                }
            }

            /* Adjust magnitudes across cur[lNumSects_1] */

            mlr = NL_NO;

            if( der[1]NEQ NULL )
            {
                /* Set some flags */

                if( cnt[1]NEQ NL_C1 )
                {
                    /* See if end magnitudes match up with der */

                    sdr = NL_YES;
                    N_CrvGetCPts( &curB, &nd, &Dw );

                    N_CPtMagnitude( Dw[0], &mg0 );

                    if( mg0 LT L01 )
                        fac0 = mg0 / L01;
                    else
                        fac0 = L01 / mg0;

                    if( fac0 LT dper )
                        sdr = NL_NO;

                    N_CPtMagnitude( Dw[nd], &mgn );

                    if( mgn LT L11 )
                        fac1 = mgn / L11;
                    else
                        fac1 = L11 / mgn;

                    if( fac1 LT dper )
                        sdr = NL_NO;

                    /* See if end magnitudes are the same */

                    lsm = NL_NO;

                    if( rfl EQ NL_NO )
                    {
                        if( L01 LT L11 )
                            fact = L01 / L11;
                        else
                            fact = L11 / L01;

                        if( fact GE lper )
                            lsm = NL_YES;
                    }

                    /* See if magnitude opposite to rail matches up with der */

                    opr = NL_NO;

                    if( rfl EQ NL_LEFT OR rfl EQ NL_RIGHT )
                    {
                        if( rfl EQ NL_LEFT AND fac1 GE dper )
                            opr = NL_YES;

                        else if( rfl EQ NL_RIGHT AND fac0 GE dper )
                            opr = NL_YES;
                    }
                }

                /* Get derivative curves */

                if( cnt[1]EQ NL_C1 OR( rfl EQ NL_NO AND sdr EQ NL_YES ) )
                {
                    /* No adjustment needed */

                    curDB = &curB;
                }
                else if( lsm EQ NL_YES OR opr EQ NL_YES )
                {
                    /* Derivative curve is scaled by a constant */

                    if( lsm EQ NL_YES )
                    {
                        d1 = L01 / mg0;
                        d2 = L11 / mgn;
                        alf = 0.5 *( d1 + d2 );
                    }
                    else if( opr EQ NL_YES )
                    {
                        if( rfl EQ NL_LEFT )
                            alf = L01 / mg0;

                        else if( rfl EQ NL_RIGHT )
                            alf = L11 / mgn;
                    }

                    N_CrvInitArrays( &curPB );
                    error = N_CrvCopy( &curB, &curPB, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_ConstantMultiplyCrv4d( alf, &curPB );

                    curDB = &curPB;
                }
                else
                {
                    /* Derivative curve is scaled by a function */

                    error = N_AllocCFuncArrays( &cfnB, 1, 1, 3, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CrvFuncCntrlValKnots( &cfnB, &fu, &UF );

                    fu[0] = L01 / mg0;
                    fu[1] = L11 / mgn;
                    UF[0] = UF[1] = 0.0;
                    UF[2] = UF[3] = 1.0;

                    N_CrvInitArrays( &curPB );
                    error = N_CrvFuncMultiplyCrv4d( &cfnB, &curB, &curPB, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    curDB = &curPB;

                    if( NOT N_Crv4dIsDegen( &curB ) )
                        dec = NL_YES;

                    mlr = NL_YES;
                }
            }
        }
    } /* end when given cross-derivative end constraints check */

    /* Make derivative curves compatible */

    if( dfl EQ NL_BOTH AND( mll EQ NL_YES OR mlr EQ NL_YES ) )
    {
        if( mll EQ NL_NO )
        {
            error = N_CrvElevateDegree( curDA, 1, curDA, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( mlr EQ NL_NO )
        {
            error = N_CrvElevateDegree( curDB, 1, curDB, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    } /* end making cross deriviative curves compatible */

    /* Get derivative vectors */

    if( der NEQ NULL )
    {
        if( der[0]NEQ NULL )
            N_CrvGetCPts( curDA, &nd, &Dws );

        if( der[1]NEQ NULL )
            N_CrvGetCPts( curDB, &nd, &Dwe );
    }

    /* Degree elevate if necessary */

    if( dec EQ NL_YES )
    {
        for ( i = 0; i <= lNumSects_1; i++ )
        {
            error = N_CrvElevateDegree( cur[i], 1, cur[i], SI, SI );

            if( error EQ NL_YES )
                NL_OUT;
        }

        N_CrvGetCPtsDegreeAndKnots( cur[0], &nc, &Cw, &pc, &mc, &UC );
    }

    /* Get surface indices */

    if( rfl NEQ NL_NO )
        ps = pr;
    else
        ps = deg;
    qs = pc;

    if( dfl EQ NL_NO )
        ns = lNumSects_1;

    else if( dfl EQ NL_LEFT )
        ns = lNumSects_1 + 1;

    else if( dfl EQ NL_RIGHT )
        ns = lNumSects_1 + 1;

    else if( dfl EQ NL_BOTH )
        ns = lNumSects_1 + 2;

    if( ns LT ps )
        BAD_INPUT;

    ms = nc;
    rs = ns + ps + 1;
    ss = mc;

    /* Get parameters */

    tl = N_AllocReal1dArray( lNumSects_1, &SL );

    if( tl EQ NULL )
        NL_QUIT;

    /* When given no rail */
    if( rail EQ NULL )
    {
        if( ts EQ NULL )
        {
            Rw = N_AllocCPt2dArray( lNumSects_1, nc, &SL );

            if( Rw EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= lNumSects_1; i++ )
            {
                N_CrvGetCPts( cur[i], &nc, &Cw );

                for ( j = 0; j <= nc; j++ )
                    N_CopyCPt( Cw[j], &Rw[i][j] );
            }

            error = N_FitSrfInterpParams( (NL_VOID ** )Rw, lNumSects_1, nc, NL_HPOINT, NL_CHORDLENGTH, NL_UDIR, tl );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( i = 0; i <= lNumSects_1; i++ )
                tl[i] = ts[i];
        }
    }
    else
    {
        for ( i = 0; i <= lNumSects_1; i++ )
            tl[i] = ts[i];
    }

    /* Get knot vector for interpolation */

    if( kns NEQ NULL )
    {
        knt = kns;

        N_KnotVectorGetKnots( kns, &i, &UT );

        if( i NEQ rs )
            BAD_INPUT;
    }
    else
    {
        knt = N_AllocKnotVectorAndArray( rs, &SL );

        if( knt EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knt, &i, &UT );

        if( dfl EQ NL_NO )
            N_FitCrvCalcKnotVector( tl, lNumSects_1, ps, knt );

        else if( dfl EQ NL_LEFT )
            N_FitCalcKnotVectorDeriv( tl, lNumSects_1, ps, NL_START, knt );

        else if( dfl EQ NL_RIGHT )
            N_FitCalcKnotVectorDeriv( tl, lNumSects_1, ps, NL_END, knt );

        else if( dfl EQ NL_BOTH )
            N_FitCalcKnotVectorEndDerivs( tl, lNumSects_1, ps, knt );
    }

    /* Get working curve */

    pol = N_AllocCPolygonAndArray( ns, &SL );

    if( pol EQ NULL )
        NL_QUIT;

    N_CrvFromCPolygonAndKnotVector( &curI, pol, ps, knt );

    /* See if refinement is needed */

    mxr = mxt = -1;

    if( rail NEQ NULL )
    {
        XR = N_AllocReal1dArray( ns, &SL );

        if( XR EQ NULL )
            NL_QUIT;

        XT = N_AllocReal1dArray( nr, &SL );

        if( XT EQ NULL )
            NL_QUIT;

        error = N_MergeKnotVectors( knt, knr, ps, XT, &mxt, XR, &mxr );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Update surface indexes and check/allocate memory */

    if( mxt GE 0 )
    {
        ns += mxt + 1;
        rs += mxt + 1;
    }

    error = N_SrfSizeArrays( sur, ns, ms, ps, qs, rs, ss, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );

    Pw = N_AllocCPt1dArray( ns, &SL );

    if( Pw EQ NULL )
        NL_QUIT;

    /* Refine rail(s) and get surface boundaries     */
    /* If rails are given, copy their control points */
    /* directly into the surface boundary points,    */
    /* After possibly refining them.                 */

    if( rail NEQ NULL )
    {
        if( mxr GE 0 )
        {
            if( rail[0]NEQ NULL )
            {
                error = N_CrvRefineToKnotVector( rail[0], XR, mxr, Pw );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= ns; i++ )
                    N_CopyCPt( Pw[i], &Sw[i][0] );
            }

            if( rail[1]NEQ NULL )
            {
                error = N_CrvRefineToKnotVector( rail[1], XR, mxr, Pw );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= ns; i++ )
                    N_CopyCPt( Pw[i], &Sw[i][ms] );
            }
        }
        else
        {
            if( rail[0]NEQ NULL )
            {
                N_CrvGetCPts( rail[0], &nd, &Dw );

                for ( i = 0; i <= nd; i++ )
                    N_CopyCPt( Dw[i], &Sw[i][0] );
            }

            if( rail[1]NEQ NULL )
            {
                N_CrvGetCPts( rail[1], &nd, &Dw );

                for ( i = 0; i <= nd; i++ )
                    N_CopyCPt( Dw[i], &Sw[i][ms] );
            }
        }
    }

    /* Now interpolate cross-sectional curves */

    error = N_FitCalcMatrix( knt, ps, tl, lNumSects_1, dfl, &cm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    js = 0;
    je = nc;

    if( rfl EQ NL_LEFT OR rfl EQ NL_BOTH )
        js = 1;

    if( rfl EQ NL_RIGHT OR rfl EQ NL_BOTH )
        je = nc - 1;

    N_CrvGetCPts( &curI, &nd, &Dw );

    for ( j = js; j <= je; j++ )
    {
        for ( i = 0; i <= lNumSects_1; i++ )
        {
            N_CrvGetCPts( cur[i], &nc, &Cw );
            N_CopyCPt( Cw[j], &Qw[i] );
        }

        if( dfl EQ NL_NO )
        {
            error = N_FitCrvMatrix( (NL_VOID *)Qw, lNumSects_1, NL_HPOINT, ps, &cm, Dw );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( dfl EQ NL_LEFT )
        {
            Ws = &Dws[j];
            error = N_FitCrvDerivMatrix( (NL_VOID *)Qw, lNumSects_1, NL_HPOINT, ps, knt, (NL_VOID *)Ws, NL_START, &cm, Dw );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( dfl EQ NL_RIGHT )
        {
            We = &Dwe[j];
            error = N_FitCrvDerivMatrix( (NL_VOID *)Qw, lNumSects_1, NL_HPOINT, ps, knt, (NL_VOID *)We, NL_END, &cm, Dw );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( dfl EQ NL_BOTH )
        {
            Ws = &Dws[j];
            We = &Dwe[j];
            error = N_FitCrvDerivsMatrix( (NL_VOID *)Qw, lNumSects_1, NL_HPOINT, ps, knt, (NL_VOID *)Ws, (NL_VOID *)We, &cm, Dw );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( mxt GE 0 )
        {
            error = N_CrvRefineToKnotVector( &curI, XT, mxt, Pw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= ns; i++ )
                N_CopyCPt( Pw[i], &Sw[i][j] );
        }
        else
        {
            for ( i = 0; i <= ns; i++ )
                N_CopyCPt( Dw[i], &Sw[i][j] );
        }
    }

    /* Get surface knot vectors */

    if( mxt GE 0 )
    {
        N_KnotVectorFromRealArray( &knx, XT, mxt );

        kna = N_AllocKnotVectorAndArray( rs, &SL );

        if( kna EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( knt, ps, &knx, kna );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kna, &rs, &UT );
    }

    for ( i = 0; i <= rs; i++ )
        US[i] = UT[i];

    for ( j = 0; j <= ss; j++ )
        VS[j] = UC[j];

    /* Remove knots if required */

    if( ES GT 0.0 )
    {
        if( rail EQ NULL )
        {
            error = N_SrfRemoveKnotsConstraints( sur, NULL, 0, NULL, 0, ES, NL_UDIR, sur, SS );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            error = N_SrfRemoveKnotsConstraints( sur, UR, mr, NULL, 0, ES, NL_UDIR, sur, SS );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* If v-skinning is needed, swap parametrization */

    if( dir EQ NL_VDIR )
    {
        error = N_SwapUV( sur, SS );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Translate back if necessary */

    if( trl EQ NL_YES )
    {
        N_ScalePt( -1.0, T, &T );
        N_SrfTranslate( sur, T );
    }

    /* Check weights */

    error = N_SrfWeightsAreValid( sur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfBoundaryContinuity */



/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine creates a swept surface
     given a trajectory curve and a section curve to be swept along the
     trajectory as
     
     NewSurf(u,v) =   A(v) * ScaleCurve(v) * SectionCurve(u) 
                    + TrajectoryCurve(v) 
     
     CrossBoundary tangent directions in the sweep direction match
     those of the trajectory curve. 
     
     d(NewSurf(u,v))/dv = d(TrajectoryCurve(v))/dv
     
     The swept surface is computed by positioning scaled instances 
     of the section curve along the trajectory, and skinning across them. 
     
     The output Nurbs surface attempts to approximate the true swept  
     surface to within the given tolerance. 

     If desired, the surface can contain the trajectory as an isocurve.  
     The output surface must be initialized to NULL.  
     A typical calling example is:

       NL_CURVE      curT, curC, curZ, curS;
       NL_REAL       E;
       NL_VECTOR     Z0;
       NL_DEGREE     deg;
       NL_PARAMETER  tc;
       NL_SURFACE    sur;
       NL_STACKS     SG;
       ...
       (get curves, choose deg, Z0, and E, and compute tc);
       ...

       N_SrfInitArrays(&sur);
       N_CreateSweepScale(&curT, &curC, Z0, NULL, &curS, NL_YES, tc, deg, NL_UDIR, E, &sur, &SG);
     
     The  section curve is aligned to the  trajectory by  mapping  the
     global coordinate  system to the trajectory's local  systems com-
     puted recursively using Z0 and the tangents  at each point  where 
     instances of the section curve are placed. That is, curC and curT
     are defined in the global coordinate system(O, X, Y, Z) , and there
     is a local coordinate system(OL, XL, YL, ZL) that rides along curT,
     whose origin OL is always a point on curT, and whose XL vector is
     always given by the tangent to curT.  Z0 is the initial value  of 
     ZL, from which all subsequent values are computed(if curZ = NULL).
     The caller can specify the ZL vectors at every point on  the tra-
     jectory by inputting the function curZ(!= NULL).  In this  case,
     the parameter ranges of curZ and curT  must  correspond.  Vectors
     evaluated from curZ do not have to have unit length.

     A typical setup for curC, curT, Z0, and curZ is as follows:

     - position curT so that its start point is at the  global  origin
       (0, 0, 0), and align curT so that its tangent at its  start point 
       is in the X direction(1, 0, 0)
     - place curC in the desired start position  with the desired ini-
       tial orientation
     - set Z0 = Z =(0, 0, 1); hence, the initial local system  on  curT
       is the global system
     - pass in curZ = NULL


   ACCESS:
   
     curT , input  ,  Trajectory curve.  This curve is assumed to be NL_G1 
                      smooth
     curC , input  ,  Cross - sectional curve
     Z0   , input  ,  Start local z - axis of the trajectory
     curZ , input  ,  Curve giving local ZL vector at  every  point  on
                      curT:
                        = NULL   not used.
                       != NULL   used to compute the ZL vectors; Z0 not
                                 used  in  this case.  Parameter  range 
                                 must coincide with the range of curT's 
                                 parameter.
     curS , input  ,  Scaling curve:
                        = NULL   no scaling is desired.
                       != NULL   curS[t]  is a vector used to scale the
                                 section curve in the x - y - z directions.
                                 Parameter range must coincide with the
                                 range of curT's parameter.
     tflg , input  ,  Interpolate trajectory flag:
                       = NL_YES   interpolate trajectory. If  curC  passes
                               through the  global  origin,  then  curT 
                               will be an isocurve in the swept surface
                       = NL_NO    do  not  interpolate trajectory
     tc   , input  ,  Parameter value on curC where curC passes through
                      the global origin(ignored if tflg = NL_NO)
     deg  , input  ,  Degree of the surface in the sweep direction.  If
                      tflg = NL_YES,  then deg must be greater than or equal
                      to the degree of the trajectory
     dir  , input  ,  Flag:
                        NL_UDIR  sweep  in  the  u - direction(all section 
                              curves are v - curves)
                        NL_VDIR  sweep  in  the  v - direction(all section 
                              curves are u - curves)
     E    , input  ,  Sweeping tolerance(E>0).  The  resulting  Nurbs 
                      surface attempts to approximate the  true  swept
                      surface to within this tolerance.  WARNING:  ex-
                      tremely small values of E can produce a  surface
                      with an enormous number of control points
     sur  , output ,  Swept surface. Must be initialized to  the  null
                      object with N_SrfInitArrays().
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSweepScale( NL_CURVE *curT, NL_CURVE *curC, NL_VECTOR Z0, NL_CURVE *curZ, NL_CURVE *curS, NL_FLAG tflg, NL_PARAMETER tc, NL_DEGREE deg, NL_FLAG dir, NL_REAL E, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSweepScale");
    NL_PRIVATE NL_INDEX MAXPTS = 1000;

    NL_FLAG error = NL_NO, *seg_flags, cont[2], split, bndy[2], UseCrossTangent = 0;

    NL_CURVE ** segs, ** sects1, ** sects2 = NULL, curCs[2], ** rails, tempC;

    /* gwc:added for new cross-tangent computation */
    NL_CURVE sects1_cross_tangents[2], sects2_cross_tangents[2];

    NL_SURFACE *sur1 = NULL, *sur2, *sur3 = NULL, surA[2], ** ders, Ds[2], De[2];

    NL_INDEX ii, jj, kk, nn, mm, k1, n_segs, *n_pars, sub, ns, ms, ruled;

    NL_REAL d1, tol, ** seg_pars, ** seg_pbnds, *temp, du, wt, w, wx, wy, wz;

    NL_DEGREE pt, ps, pp;

    NL_POINT *dummy, scale, OL, derivs1[2], derivs2[2];

    /* gwc:added for new cross-tangent computation */
    NL_POINT dscale1[2]; /* dscale1[0] = scale, dscale1[1] = d(scale)/dv (beg segment) */
    NL_POINT dscale2[2]; /* dscale2[0] = scale, dscale2[1] = d(scale)/dv (end segment) */

    NL_CPOINT *Pw;

    NL_PARAMETER *tt = NULL, us = 0.0, ue;

    NL_INTEGER ZLvec_count = 0;
    NL_VECTOR XL, YL, ZL, *ZLvecs = NULL;

    NL_RMATRIX rma;

    /* gwc:added for new cross-tangent computation */
    NL_RMATRIX rma1, rma2;   /* rma  = rotation matrix (beg and end of segment) */
    NL_RMATRIX drma1, drma2; /* drma = derivatives of rma rotation matrix (beg and end of segment) */

    NL_BOOLEAN ratT = FALSE, ratC = FALSE;

    NL_CFUN wtfun;

    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* 1 = use cross_tangent function rather than a single tangent-vector  */
    UseCrossTangent = 1;

    /* Get local notation */
    N_CrvGetDegree( curT, &pt );
    ruled = 0;
    split = 0;

    /* Decompose trajectory into linear and curved pieces */

    /* when trajectory curve is degree 1 (linear) */
    if( pt EQ 1 )
    {
        /* error check: curT better be linear */
        if( NOT( N_CrvIsLine( curT, NL_MTOL ) ) )
            NL_ERROR( NL_INP_ERR );

        /* size seg_pbnds to [1,2] and seg_flags to [1] */
        seg_pbnds = N_AllocReal2dArray( 0, 1, &SL );
        seg_flags = N_AllocFlag1dArray( 0, &SL );

        if( seg_pbnds EQ NULL OR seg_flags EQ NULL )
            NL_QUIT;
        seg_flags[0] = NL_YES;

        /* set seg_pbnds[0][0,1] to start/end param bounds of curT */
        N_CrvGetParamBounds( curT, &seg_pbnds[0][0], &seg_pbnds[0][1] );
        n_segs = 0;

        /* gwc:bug? shouldn't this be a ruled surface when curS is NULL */
        /* when a scale function is given */
        if( curS )
        {     /* when curS is a line and output sweep degree is 1 */
            if( N_CrvIsLine( curS, NL_MTOL )AND deg EQ 1 )
            { /* then remember output is a ruled surface */
                ruled = 1;
            }
        }
        else
            ruled = 1; /* no curS but trajectory is linear */
    }
    else               /* when trajectory curve degree > 1 */
    {
        /* get a list of all linear segments in curT */
        error = N_CrvGetLinearSegs( curT, NL_MTOL, &seg_pbnds, &seg_flags, &n_segs, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* allocate memory for segs, seg_pars, and n_pars */
    /* set segs:size[n_segs+1], seg_pars:size[n_segs+1],n_pars:size[n_segs+1] */
    segs = N_AllocArrayCrvPtrsAndData( n_segs, NL_YES, &SL );

    if( segs EQ NULL )
        NL_QUIT;
    seg_pars = N_AllocRealPtr1dArray( n_segs, &SL );

    if( seg_pars EQ NULL )
        NL_QUIT;
    n_pars = N_AllocInt1dArray( n_segs, &SL );

    if( n_pars EQ NULL )
        NL_QUIT;

    /* place curT into segs array */
    if( n_segs EQ 0 )
    {
        /* copy all of curT into segs[0] */
        error = N_CrvCopy( curT, segs[0], &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        for ( ii = 0; ii <= n_segs; ii++ )
        {
            /* place curT segments into seg arrays */
            error = N_CrvExtractCrvSeg( curT, seg_pbnds[ii][0], seg_pbnds[ii][1], segs[ii], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Must degree elevate trajectory segments if deg > pt and tflg=NL_YES */
    /*  tflg == NL_YES: interpolate the trajectory curve and */
    /*  deg  = degree of surface in sweep direction   */
    /*  pt   = input degree of CurT  */
    if( tflg EQ NL_YES AND deg GT pt )
    {
        for ( ii = 0; ii <= n_segs; ii++ )
        {
            /* increment curve segs[ii] degree to the sweep surface's requested degree.*/
            error = N_CrvElevateDegree( segs[ii], deg - pt, segs[ii], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Compute the trajectory parameter locations where to */
    /* place the sections for skinning */

    tol = 0.5 *E; /* polygonalization tolerance (chordal deviation) */

    /* for every segment */
    for ( ii = 0; ii <= n_segs; ii++ )
    {
        d1 = tol;

        while( 1 )
        {
            /* compute piecewise linear approximation to each trajectory segment */
            error = N_ApproxCrvWithPolyline( segs[ii], d1, NL_ABSOLUTE, NL_PARAMETERS, &dummy, &temp, &kk, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* quit when linear approximation is less then MAXPTS */
            if( kk LE MAXPTS )
                break;

            /* else increase tolerance by factor of 10 - delete old answer and try again */
            d1 *= 10.0;
            N_FreeReal1dArray( temp, &SL );
        } /* end while curve is over-sampled */

        /* ensure we have enough placement locations */

        /* let jj = requested output surface sweep degree */
        jj = deg;

        if( curS NEQ NULL )
        {
            /* get curS control_point and knot highest indices.*/
            N_CrvGetArraySizes( curS, &nn, &mm );
            jj = nn + 1;
        }

        /* when linear sampling is good enough. */
        /*    jj is either the output surface sweep degree */
        /*       or the number of control points in the scale curve.*/
        /*    kk is the number of linear segments in the segment approximation.*/
        if( (kk GE jj)OR ruled ) /* test for simple ruled surface */
        {
            /* use the linear sampling of trajectory curve as given */
            seg_pars[ii] = temp;
            n_pars[ii] = kk;
        }
        else /* linear sampling is too coarse */
        {
            mm = jj - kk;
            nn = mm / kk + 1; /* add this many in each interval*/
            seg_pars[ii] = N_AllocReal1dArray( kk * (nn + 1), &SL );

            if( seg_pars[ii]EQ NULL )
                NL_QUIT;
            n_pars[ii] = kk * (nn + 1);

            seg_pars[ii][0] = temp[0];
            mm = 1;

            for ( jj = 1; jj <= kk; jj++ )
            {
                du = (temp[jj] - temp[jj - 1]) / ((NL_REAL)nn + 1.0);

                for ( k1 = 1; k1 <= nn; k1++ )
                    seg_pars[ii][mm++] = temp[jj - 1] + k1 * du;
                seg_pars[ii][mm++] = temp[jj];
            }
            N_FreeReal1dArray( temp, &SL );
        } /* end not a simple ruled surface case */
    }     /* end iter every trajectory segment */

    /* Compute the binormal ZL-vectors for each parameter location */
    /* if curZ is not given. Must do it here to handle closed curT */

    if( curZ EQ NULL )
    {
        kk = 1;

        for ( ii = 0; ii <= n_segs; ii++ )
        {
            if( seg_flags[ii]EQ NL_YES )
                sub = 0;

            else if( n_pars[ii]GT MAXPTS / 2 )
                sub = 0;

            else if( n_pars[ii]GT MAXPTS / 5 )
                sub = 1;

            else if( n_pars[ii]GT MAXPTS / 10 )
                sub = 2;

            else if( n_pars[ii]GT MAXPTS / 20 )
                sub = 3;

            else
                sub = 4;

            kk += (n_pars[ii] * (sub + 1));
        }

        /* allocate tt an array of Zlvec param values */
        tt = N_AllocReal1dArray( kk, &SL );

        if( tt EQ NULL )
            NL_QUIT;
        ZLvecs = N_AllocPt1dArray( kk, &SL );

        if( ZLvecs EQ NULL )
            NL_QUIT;

        tt[0] = seg_pars[0][0];
        kk = 1;

        for ( ii = 0; ii <= n_segs; ii++ )
        {
            /* set sub-sampling for this segment */
            if( seg_flags[ii]EQ NL_YES )
                sub = 0;

            else if( n_pars[ii]GT MAXPTS / 2 )
                sub = 0;

            else if( n_pars[ii]GT MAXPTS / 5 )
                sub = 1;

            else if( n_pars[ii]GT MAXPTS / 10 )
                sub = 2;

            else if( n_pars[ii]GT MAXPTS / 20 )
                sub = 3;

            else
                sub = 4;

            /* set tt values (with/without sub-sampling) for upcoming ZLvecs computation*/
            if( sub EQ 0 )
            {
                for ( jj = 1; jj <= n_pars[ii]; jj++ )
                    tt[kk++] = seg_pars[ii][jj];
            }
            else
            {
                for ( jj = 1; jj <= n_pars[ii]; jj++ )
                {
                    du = (seg_pars[ii][jj] - seg_pars[ii][jj - 1]) / ((NL_REAL)sub + 1.0);

                    for ( k1 = 1; k1 <= sub; k1++ )
                        tt[kk++] = seg_pars[ii][jj - 1] + k1 * du;
                    tt[kk++] = seg_pars[ii][jj];
                }
            }
        } /* end iter every segment */

        /* compute array of Zlvecs for curT at every tt param value given Z0 */
        ZLvec_count = kk;
        error = N_CreateBVectors( curT, Z0, tt, kk - 1, ZLvecs );

        if( error EQ NL_YES )
            NL_OUT;
    } /* end if curZ == NULL check */

    /* Handle trajectory interpolation (rail) and split curC if necessary */

    rails = NULL;
    N_CrvInitArrays( &curCs[0] );
    N_CrvInitArrays( &curCs[1] );

    if( tflg EQ NL_YES )
    {
        N_CrvGetParamBounds( curC, &us, &ue );
        rails = N_AllocArrayCrvPtrs( 1, &SL );

        if( rails EQ NULL )
            NL_QUIT;

        if( tc GT us AND tc LT ue )
        { /* trajectory is interior */
            /* split curC into two segments at tc parameter value */
            error = N_CrvSplit( curC, tc, &curCs[0], &curCs[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;
            split = NL_YES;

            cont[0] = cont[1] = NL_G1;
        }
        else
        { /* trajectory is at start or end (rail) */
            if( tc EQ us )
                rails[1] = NULL;
            else
                rails[0] = NULL;

            error = N_CrvCopy( curC, &curCs[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;
            split = NL_NO;

            cont[0] = cont[1] = NL_G1;
        }
    }
    else
    {
        error = N_CrvCopy( curC, &curCs[0], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        split = NL_NO;
        cont[0] = cont[1] = NL_G1;
    }

    /* Prepare surface objects (including cross-boundary derivatives) */

    /* When sur is NOT NULL init sur to NULL */
    if( NOT( N_SrfAreArraysNULL( sur ) ) )
        N_SrfInitArrays( sur );
    sur2 = sur;

    if( split EQ NL_YES )
    {
        sur3 = &surA[1];
        N_SrfInitArrays( sur3 );
        N_SrfInitArrays( &Ds[1] );
        N_SrfInitArrays( &De[1] );
    }

    /* allocate an array of 2 NL_SURFACE pointers for specifying cross-tangent constraints */
    ders = N_AllocArraySrfPtrs( 1, &SL );

    if( ders EQ NULL )
        NL_QUIT;

    N_SrfInitArrays( &Ds[0] );
    N_SrfInitArrays( &De[0] );

    /* Now loop over the segments of the trajectory, and: */
    /*   (1) scale and place the cross sections      */
    /*   (2) define the boundary cross-derivatives   */
    /*   (3) skin                                    */
    /*   (4) remove knots if did interpolatory skinning */
    /*   (5) merge (join) surface with previous surfaces */

    /* First allocate various memory */

    /* let kk = max n_pars[ii] value (number of cross-sections per trajectory segment) */
    kk = 0;

    for ( ii = 0; ii <= n_segs; ii++ )
    {
        if( n_pars[ii]GT kk )
            kk = n_pars[ii];
    }

    if( split EQ NL_YES )
    {
        N_CrvGetArraySizes( &curCs[0], &ns, &ms );
        N_CrvGetDegree( &curCs[0], &ps );

        /* allocate ns+1 array of cross-section curves and 2 cross-tangent curves */
        sects1 = N_Alloc1dArrayCrvs( ns, ps, ms, kk, &SL );

        /* allocate memory for Ds and De surfaces */
        error = N_AllocSrfArrays( &Ds[0], ns, 1, ps, 1, ms, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_AllocSrfArrays( &De[0], ns, 1, ps, 1, ms, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetArraySizes( &curCs[1], &ns, &ms );
        N_CrvGetDegree( &curCs[1], &ps );

        /* allocate ns+1 array of cross-section curves and 2 cross-tangent curves */
        sects2 = N_Alloc1dArrayCrvs( ns, ps, ms, kk, &SL );

        /* allocate memory for Ds and De surfaces */
        error = N_AllocSrfArrays( &Ds[1], ns, 1, ps, 1, ms, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_AllocSrfArrays( &De[1], ns, 1, ps, 1, ms, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }    /* end split case */
    else /* no-split case */
    {
        /* locals: ns = curC->pol->n,  ms = curC->knt->m,  ps = curC->p */
        N_CrvGetArraySizes( curC, &ns, &ms );
        N_CrvGetDegree( curC, &ps );

        /* allocate memory for kk+1 curves sharing ns, ps, ms values and 2 cross-tangent curves */
        sects1 = N_Alloc1dArrayCrvs( ns, ps, ms, kk, &SL );

        /* allocate memory for Ds[0] surfaces with ns+1 x 2 control points */
        error = N_AllocSrfArrays( &Ds[0], ns, 1, ps, 1, ms, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* allocate memory for De[0] surface to be same size as Ds[0] surfaces */
        error = N_AllocSrfArrays( &De[0], ns, 1, ps, 1, ms, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* allocate memory for 4x4 full matrix (bandwidth = 3) */
    error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_SetRealMatrix( &rma1, 2, 2, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_SetRealMatrix( &rma2, 2, 2, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_SetRealMatrix( &drma1, 2, 2, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_SetRealMatrix( &drma2, 2, 2, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Handle rationality correctly if interpolating trajectory */

    if( tflg EQ NL_YES )
    {
        ratT = N_IsCrvRat( curT );

        if( ratT )
        {
            N_CrvGetCPts( curT, &nn, &Pw ); /* check if it is really rational */

            for ( jj = 0; jj <= nn; jj++ )
            {
                N_CPtGetW( Pw[jj], &w );

                if( w NEQ 1.0 )
                    break;
            }

            if( jj GT nn )
                ratT = NL_FALSE;
            else
            {
                ratC = N_IsCrvRat( curC );
                N_CFuncInitArrays( &wtfun );
                N_CrvGetDenomCrvFunc( curT, &wtfun, &SL );
            }
        }
    }

    k1 = 0; /* index into tt and ZLvecs arrays */

    bndy[0] = NL_TOP;
    bndy[1] = NL_BOTTOM;

    /* Now the big loop */

    /* for every trajectory segment */
    for ( ii = 0; ii <= n_segs; ii++ )
    {
        /* scale and place the cross sections */
        /* for every parameter value in this trajectory segment */
        for ( jj = 0; jj <= n_pars[ii]; jj++ )
        {
            /* copy curCs[0] into sects1[jj]*/
            error = N_CrvCopy( &curCs[0], sects1[jj], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( split EQ NL_YES )
            {
                /* copy curCS[1] into sects2[jj] */
                error = N_CrvCopy( &curCs[1], sects2[jj], &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* when no scale function is given */
            if( curS EQ NULL )
            {
                /* on the first pass */
                if( ii EQ 0 AND jj EQ 0 )
                {
                    /* set scale = 1.0 and d(scale)/dv = 0.0 for segment begin */
                    dscale1[0].x = 1.0;
                    dscale1[1].x = 0.0;
                    dscale1[0].y = 1.0;
                    dscale1[1].y = 0.0;
                    dscale1[0].z = 1.0;
                    dscale1[1].z = 0.0;

                    /* set scale = 1.0 and d(scale)/dv = 0.0 for segment end */
                    dscale2[0].x = 1.0;
                    dscale2[1].x = 0.0;
                    dscale2[0].y = 1.0;
                    dscale2[1].y = 0.0;
                    dscale2[0].z = 1.0;
                    dscale2[1].z = 0.0;
                }
            }    /* end no scale case */
            else /* use the input scale function to compute scale and d(scale)/dv values */
            {
                /* let scale = curS(seg_pars[ii][jj]) */
                error = N_CrvEval( curS, seg_pars[ii][jj], NL_LEFT, &scale );

                if( error EQ NL_YES )
                    NL_OUT;

                /* gwc:added for new cross-tangent computation */
                /* when computing gain for 1st or last cross-section */
                if( jj EQ 0 )
                { /* compute the segment's beginning scale rate of change */
                    error = N_CrvDerivs( curS, seg_pars[ii][jj], NL_LEFT, 1, dscale1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( jj EQ n_pars[ii] )
                {
                    /* compute the segment's end scale rate of change */
                    error = N_CrvDerivs( curS, seg_pars[ii][jj], NL_LEFT, 1, dscale2 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( error EQ NL_YES )
                    NL_OUT;

                /* scale curve, sects1[jj]->cpt_x = scale_x * sects1[jj]->cpt_x */
                /*              sects1[jj]->cpt_y = scale_y * sects1[jj]->cpt_y */
                /*              sects1[jj]->cpt_z = scale_z * sects1[jj]->cpt_z */
                N_CrvScale( sects1[jj], NL_ZERO, scale );

                if( split EQ NL_YES )
                    N_CrvScale( sects2[jj], NL_ZERO, scale );
            }

            /* let OL = point_on_curve(seg_pars[ii][jj]) and */
            /*     XL = tang_on_curve(seg_pars[ii][jj]) */
            error = N_CrvEvalTangent( segs[ii], seg_pars[ii][jj], NL_LEFT, &OL, &XL );

            if( error EQ NL_YES )
                NL_OUT;

            if( curZ NEQ NULL )
            {
                /* evaluate given curZ for param = seg_pars[ii][jj] */
                error = N_CrvEval( curZ, seg_pars[ii][jj], NL_LEFT, &ZL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* normalize ZL */
                error = N_VectorNormalize( ZL, &ZL, &d1 );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                /* let ZL = curve B vector for param = seg_pars[ii][jj] */
                while( tt[k1]LT seg_pars[ii][jj] )
                    k1 += 1;
                N_CopyPt( ZLvecs[k1], &ZL );
            }

            /* let YL = ZL cross XL */
            N_VectorCross( ZL, XL, &YL );

            /* build rma transform to map current coordinates into new L coordinates */
            error = N_CreateTransformMatrixFromAxes( NL_ZERO, NL_UNITX, NL_UNITY, NL_UNITZ, OL, XL, YL, ZL, &rma, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* gwc:note for new cross-tangent computations
             sweep surface cross-tangent values in the direction of the sweep
             depend on the derivative of the sweep definition.  Sweep is defined as
              
               S(u,v) = T(v) + M(v)C(u)     Where: T(v) = trajectory
                                                   M(v) = rotation and scaling effects.
             The cross tangents are computed as
            
               Sv(u,v) = Tv(v) + Mv(v)C(u)  Where: a partial derivative with v is shown as Tv or Mv
            
             The M matrix is composed of
               M(v) = [scale_x(v)*x0(v)  scale_y(v)*y0(v)  scale_z(v)*z0(v)]
                       [scale_x(v)*x1(v)  scale_y(v)*y1(v)  scale_z(v)*z1(v)]
                       [scale_x(v)*x2(v)  scale_y(v)*y2(v)  scale_z(v)*z2(v)]
              where:
                   x(v)     = T'(v)/|T'(v)|              stored in XL
                   z(v)     = B(v)/|B(v)|                stored in ZL
                   y(v)     = cross_product(z(v), x(v))  stored in YL
            
             In this function 
                The M(v) scale is represented by a function or is a constant,
                The M(v) rotation terms are stored in the 3x3 upper left corner of rma,
                C(u) has been stored as one or two different curves in curCs, 
                T(v) has been stored as a set of segments in segs.
            
             For later cross-tangent constraint computation on the ends of each segment
             save XL(v), YL(v), ZL(v), XL'(v), YL'(v), ZL'(v), T'(v), scale(v), and scale'(v)
             for the beginning and end of each trajectory curve segment.
            
             save transforms to compute cross-tangent values later on */
            if( jj EQ 0 )
            {
                /*save the segment beg rotation matrix */
                rma1.RM[0][0] = XL.x;
                rma1.RM[0][1] = YL.x;
                rma1.RM[0][2] = ZL.x;
                rma1.RM[1][0] = XL.y;
                rma1.RM[1][1] = YL.y;
                rma1.RM[1][2] = ZL.y;
                rma1.RM[2][0] = XL.z;
                rma1.RM[2][1] = YL.z;
                rma1.RM[2][2] = ZL.z;

                /*compute and save the rotation matrix rate of change */
                error = ST_GetRotationRateOfChange( segs[ii], seg_pars[ii][jj], curZ, &ZL, (k1 > 0) ? &ZLvecs[k1 - 1] : NULL, (k1 > 0) ? tt[k1 - 1] : 0, (k1 < ZLvec_count - 1) ? &ZLvecs[k1 + 1] : NULL, (k1 < ZLvec_count - 1) ? tt[k1 + 1] : 0, &drma1 );
            }
            else if( jj EQ n_pars[ii] )
            {
                /* save the segment end rotation matrix */
                rma2.RM[0][0] = XL.x;
                rma2.RM[0][1] = YL.x;
                rma2.RM[0][2] = ZL.x;
                rma2.RM[1][0] = XL.y;
                rma2.RM[1][1] = YL.y;
                rma2.RM[1][2] = ZL.y;
                rma2.RM[2][0] = XL.z;
                rma2.RM[2][1] = YL.z;
                rma2.RM[2][2] = ZL.z;

                /* compute and save the rotation matrix rate of change */
                error = ST_GetRotationRateOfChange( segs[ii], seg_pars[ii][jj], curZ, &ZL, (k1 > 0) ? &ZLvecs[k1 - 1] : NULL, (k1 > 0) ? tt[k1 - 1] : 0, (k1 < ZLvec_count - 1) ? &ZLvecs[k1 + 1] : NULL, (k1 < ZLvec_count - 1) ? tt[k1 + 1] : 0, &drma2 );
            }

            /* transform curve in sects1[jj] by general 4x4 transformation rma */
            N_CrvTransform( sects1[jj], &rma );

            if( split EQ NL_YES )
                N_CrvTransform( sects2[jj], &rma );

            if( tflg EQ NL_YES ) /* adjust cross-section weights if necessary */
            {
                if( ratT )
                {
                    error = N_CFuncEval( &wtfun, seg_pars[ii][jj], NL_LEFT, &wt );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CrvGetCPts( sects1[jj], &ns, &Pw );

                    for ( kk = 0; kk <= ns; kk++ )
                    {
                        N_CPtToWxWyWz( Pw[kk], &wx, &wy, &wz, &w );

                        if( NOT ratC )
                            w = 1.0;
                        N_CPtFromWxWyWz( wt * wx, wt * wy, wt * wz, wt * w, &Pw[kk] );
                    }

                    if( split EQ NL_YES )
                    {
                        N_CrvGetCPts( sects2[jj], &ns, &Pw );

                        for ( kk = 0; kk <= ns; kk++ )
                        {
                            N_CPtToWxWyWz( Pw[kk], &wx, &wy, &wz, &w );

                            if( NOT ratC )
                                w = 1.0;
                            N_CPtFromWxWyWz( wt * wx, wt * wy, wt * wz, wt * w, &Pw[kk] );
                        } /* end every control point */
                    }     /* end split case */
                }         /* end ratT check - constructiong rational surface */
            }             /* end tflg EQ yes check - interpolating the trajectory curve */
        }                 /* end iter every param value for this trajectory segment */

        /* define the boundary cross derivatives */

        ders[0] = &Ds[0];
        ders[1] = &De[0];

        /* let derivs1[0] = pos, derivs1[1] = start tang of trajectory curve */
        /* where trajectory curve is now stored in pieces in segs[ii] with start-param value seg_pbnds[ii][0] */
        error = N_CrvDerivs( segs[ii], seg_pbnds[ii][0], NL_LEFT, 1, derivs1 );

        if( error EQ NL_YES )
            NL_OUT;

        /* let derivs2[0] = pos, derivs2[1] = end tang of trajectory curve */
        /* where trajectory curve is now stored in pieces in segs[ii] with end-param values seg_pbnds[ii][1] */
        error = N_CrvDerivs( segs[ii], seg_pbnds[ii][1], NL_LEFT, 1, derivs2 );

        if( error EQ NL_YES )
            NL_OUT;

        if( split EQ NL_YES )
        {
            N_CrvGetArraySizes( sects1[0], &ns, &ms );
            N_CrvGetDegree( sects1[0], &ps );
            N_SrfSetSizeIndices( &Ds[0], ns, 1, ps, 1, ms, 3 );

            /* gwc:added for new cross derivative computation */
            /* compute begin and end cross-tangent curves for curCs[0] part of sweep surface */
            error = N_SweepCrossTangentCrv( &derivs1[1], &dscale1[0], &dscale1[1], &rma1, &drma1, &curCs[0], &sects1_cross_tangents[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_SweepCrossTangentCrv( &derivs2[1], &dscale2[0], &dscale2[1], &rma2, &drma2, &curCs[0], &sects1_cross_tangents[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* compute begin and end cross-tangent curves for curCS[1] part of sweep surface */
            error = N_SweepCrossTangentCrv( &derivs1[1], &dscale1[0], &dscale1[1], &rma1, &drma1, &curCs[1], &sects2_cross_tangents[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_SweepCrossTangentCrv( &derivs2[1], &dscale2[0], &dscale2[1], &rma2, &drma2, &curCs[1], &sects2_cross_tangents[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* GWC:CHANGE modify computation of cross-derivatives */
            /* use cross_tangent function rather than a single tangent-vector */
            if( UseCrossTangent )
                error = N_CrossBoundaryDerivsVectorField( sects1[0], &sects1_cross_tangents[0], NL_TOP, &Ds[0], &SL );
            else
                error = N_CreateSkinSrfBoundaryDerivs( sects1[0], derivs1[1], NL_TOP, &Ds[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfSetSizeIndices( &De[0], ns, 1, ps, 1, ms, 3 );

            /* GWC:CHANGE modify computation of cross-derivatives */
            /* use cross_tangent function rather than a single tangent-vector */
            if( UseCrossTangent )
                error = N_CrossBoundaryDerivsVectorField( sects1[n_pars[ii]], &sects1_cross_tangents[1], NL_BOTTOM, &De[0], &SL );
            else
                error = N_CreateSkinSrfBoundaryDerivs( sects1[n_pars[ii]], derivs2[1], NL_BOTTOM, &De[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetArraySizes( sects2[0], &ns, &ms );
            N_CrvGetDegree( sects2[0], &ps );
            N_SrfSetSizeIndices( &Ds[1], ns, 1, ps, 1, ms, 3 );

            /* GWC:CHANGE modify computation of cross-derivatives */
            /* use cross_tangent function rather than a single tangent-vector */
            if( UseCrossTangent )
                error = N_CrossBoundaryDerivsVectorField( sects2[0], &sects2_cross_tangents[0], NL_TOP, &Ds[1], &SL );
            else
                error = N_CreateSkinSrfBoundaryDerivs( sects2[0], derivs1[1], NL_TOP, &Ds[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfSetSizeIndices( &De[1], ns, 1, ps, 1, ms, 3 );

            /* GWC:CHANGE modify computation of cross-derivatives */
            /* use cross_tangent function rather than a single tangent-vector */
            if( UseCrossTangent )
                error = N_CrossBoundaryDerivsVectorField( sects2[n_pars[ii]], &sects2_cross_tangents[1], NL_BOTTOM, &De[1], &SL );
            else
                error = N_CreateSkinSrfBoundaryDerivs( sects2[n_pars[ii]], derivs2[1], NL_BOTTOM, &De[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }    /* split case */
        else /* not-split case */
        {
            /* locals: ns, ms, ps for cross-curve */
            N_CrvGetArraySizes( sects1[0], &ns, &ms );
            N_CrvGetDegree( sects1[0], &ps );

            /* set surface DS[0] size, surf_size = cross_curve_size X 1_element */
            N_SrfSetSizeIndices( &Ds[0], ns, 1, ps, 1, ms, 3 );

            /* gwc:added for new cross derivative computation */
            /* compute begin and end cross-tangent curves for sweep surface */
            if( UseCrossTangent )
            {
                error = N_SweepCrossTangentCrv( &derivs1[1], &dscale1[0], &dscale1[1], &rma1, &drma1, &curCs[0], &sects1_cross_tangents[0], &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                error = N_SweepCrossTangentCrv( &derivs2[1], &dscale2[0], &dscale2[1], &rma2, &drma2, &curCs[0], &sects1_cross_tangents[1], &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* compute cross-derivative data */
                /* GWC:CHANGE modify computation of cross-derivatives */
                /* use cross_tangent function rather than a single tangent-vector */
                error = N_CrossBoundaryDerivsVectorField( sects1[0], &sects1_cross_tangents[0], NL_TOP, &Ds[0], &SL );
            }
            else
                error = N_CreateSkinSrfBoundaryDerivs( sects1[0], derivs1[1], NL_TOP, &Ds[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* set surface De[0] size, surf_size = cross_curve_size X 1_element */
            N_SrfSetSizeIndices( &De[0], ns, 1, ps, 1, ms, 3 );

            /* compute cross-derivative data */
            /* GWC:CHANGE modify computation of cross-derivatives */
            /* use cross_tangent function rather than a single tangent-vector */
            if( UseCrossTangent )
                error = N_CrossBoundaryDerivsVectorField( sects1[n_pars[ii]], &sects1_cross_tangents[1], NL_BOTTOM, &De[0], &SL );
            else
                error = N_CreateSkinSrfBoundaryDerivs( sects1[n_pars[ii]], derivs2[1], NL_BOTTOM, &De[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* now skin */

        if( split EQ NL_YES )
        {
            /* copy trajectory segment, segs[ii], into tempC */
            N_CrvInitArrays( &tempC );
            error = N_CrvCopy( segs[ii], &tempC, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* skin surf for this trajectory segment from cross-section start to trajectory curve */
            rails[0] = NULL;
            rails[1] = segs[ii];
            error = N_CreateSkinSrfBoundaryContinuity( sects1, n_pars[ii], ders, NL_YES, cont, bndy, rails, deg, 0.0, NL_VDIR, seg_pars[ii], NULL, sur2, &SL, SG );

            if( error EQ NL_YES )
                NL_OUT;

            /* skin surf for this trajectory segment from trajectory curve tp cross-section end */
            ders[0] = &Ds[1];
            ders[1] = &De[1];
            rails[1] = NULL;
            rails[0] = &tempC;
            error = N_CreateSkinSrfBoundaryContinuity( sects2, n_pars[ii], ders, NL_YES, cont, bndy, rails, deg, 0.0, NL_VDIR, seg_pars[ii], NULL, sur3, &SL, SG );

            if( error EQ NL_YES )
                NL_OUT;
            N_FreeCrv( &tempC, &SL );

            /* join sur2 and sur3 skins along trajectory curve and place in sur2 */
            error = N_SrfJoin( sur2, sur3, NL_UDIR, NL_MTOL, sur2, SG );

            if( error EQ NL_YES )
                NL_OUT;
            N_FreeSrf( sur3, SG );

        /* doctor across crease here to get NL_G1 ? */
        /*
        N_CrvGetKnots(segs[ii], &mr, &VR);
        error = N_SrfRemoveKnotsConstraints(sur2, NULL, 0, VR, mr, E, NL_VDIR, sur2, SG);
        if (error EQ NL_YES)
        NL_OUT;  */
        }    /* end split case */
        else /* not-split case */
        {
            if( tflg EQ NL_YES )
            {
                temp = seg_pars[ii];

                if( tc EQ us )
                    rails[0] = segs[ii];
                else
                    rails[1] = segs[ii];
            }
            else
                temp = NULL;

            /* If ruled ignore end derivatives */
            if( ruled )
                ders = NULL;

            /* let sur2 = skinning of profile set */
            error = N_CreateSkinSrfBoundaryContinuity( sects1, n_pars[ii], ders, NL_YES, cont, bndy, rails, deg, E, NL_VDIR, temp, NULL, sur2, &SL, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* merge surfaces and prepare for next pass thru loop */

        if( n_segs GT 0 )
        {
            if( ii EQ 0 )
            {
                sur1 = sur;
                sur2 = &surA[0];
            }
            else
            {
                /* get sur1 and sur2 degrees */
                N_SrfGetDegrees( sur1, &ps, &pp );
                N_SrfGetDegrees( sur2, &pt, &pp );
                error = NL_NO;

                /* use elevate to ensure sur1 and sur2 have same degrees */
                if( ps LT pt )
                    error = N_SrfElevateDegree( sur1, pt - ps, NL_UDIR, sur1, SG, SG );

                else if( pt LT ps )
                    error = N_SrfElevateDegree( sur2, ps - pt, NL_UDIR, sur2, SG, SG );

                if( error EQ NL_YES )
                    NL_OUT;

                /* join sur1 and sur2 in the vdir */
                error = N_SrfJoin( sur1, sur2, NL_VDIR, NL_MTOL, sur1, SG );

                if( error EQ NL_YES )
                    NL_OUT;

                N_FreeSrf( sur2, SG );
            }

            N_SrfInitArrays( &surA[0] );

            if( split EQ NL_YES )
                N_SrfInitArrays( &surA[1] );
        } /* end n_segs GT 0 check */
    }     /* end iter every segment */
    /* end of the big loop */

    /* Make sure sweep direction is correct */

    if( dir EQ NL_UDIR )
    {
        error = N_SwapUV( sur, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}  /* end N_CreateSweepScale*/



/*==========================================================================
 ST_GetRotationRateOfChange                                                  
==========================================================================*/

NL_FLAG ST_GetRotationRateOfChange /* eff: find [x'|y'|z'] for sweep trajectory*/
( NL_CURVE *traj,                  /* in : sweep curve trajectory  */
NL_PARAMETER u,                    /* in : parameter being examined */
NL_CURVE *curZ,                    /* in : optional curve to compute Z and dZ or NULL  */
NL_VECTOR *Z,                      /* in : alternative: associated Z value for given u */
NL_VECTOR *prevZ,                  /* in :              discrete Z prior to Z or NULL  */
NL_PARAMETER prev_u,               /* in :              u associated with prevZ      */
NL_VECTOR *nextZ,                  /* in :              discrete Z after Z or NULL  */
NL_PARAMETER next_u,               /* in :              u associated with nextZ     */
NL_RMATRIX *drma )                 /* out: rotation rate of change - pre-allocated  */
/*       ordered = [ dx[0] | dy[0] | dz[0] ]     */
/*                 [ dx[1] | dy[1] | dz[1] ]     */
/*                 [ dx[2] | dy[2] | dz[2] ]     */
/* modifies: drma
 effects : computes and sets drma with the rotation rate of change for the given
           sweep curve and and Z vector.  Stored as:

 method  : For Rotation M = [x | y | z ] where
           x = t'/|t'|  
           z = Z
           y = cross_product(z,x)

           with t'  = derivative of traj at param u and
                t'' = 2nd derivative of traj at param u.

           derivative of M = [x' | y' | z'] with
           x' = t'' * |t'| - t'* (t' * t'')
                ----------   --------------
                  |t'|**2       |t'|**3

           z' = dZ
           y' = cross_product(z',x) + cross_product(z,x')
           */
{
    /* locals */
    NL_FLAG error = NL_NO; /* return value*/
    NL_POINT t[3];         /* t[0] = traj(u), */
    /* t[1] = d(traj(u)/du, */
    /* t[2] = d2(traj(u))/du**2 */
    NL_POINT z[2]; /* z[0] = curZ(u)           */
    /* z[1] = d(curZ(u))/du     */
    NL_REAL mag;        /* mag  = sqrt(vec * vec)   */
    NL_REAL mag3;       /* mag3 = mag * mag * mag;  */
    NL_REAL d12;        /* d12  = vec * vec'        */
    NL_VECTOR X;        /* X    = rotation frame x axis */
    NL_VECTOR dX;       /* dX   = d(X(u))/du            */
    NL_VECTOR mZ;       /* mZ   = rotation frame z axis */
    NL_VECTOR dmZ;      /* dmZ  = d(mZ(u))/du */
    NL_VECTOR *F1, *F2; /* indirection for finite differences */
    NL_REAL du1, du2;   /* indirection for finite differences */

    /* check for required inputs*/
    if( traj == NULL || drma == NULL )
        return (1);

    /* user must supply either curZ */
    /* or Z and either nextZ or prevZ or both*/
    if( curZ == NULL && (Z == NULL || (prevZ == NULL && nextZ == NULL)) )
        return (1);

    /* currently no checks on u, prev_u, or next_u values */

    /* get Z and dZ values */
    if( curZ NEQ NULL )
    { /* evaluation ZL and z */
        error = N_CrvDerivs( curZ, u, NL_LEFT, 1, z );

        if( error EQ NL_YES )
            NL_OUT;

        /* get Z01, |ZL|, and mZL3 */
        N_Dot2Pts( z[0], z[1], &d12 );
        N_PtMagnitude( z[0], &mag );
        mag3 = mag * mag * mag;

        /* save normalized Z */
        mZ.x = z[0].x / mag;
        mZ.y = z[0].y / mag;
        mZ.z = z[0].z / mag;

        /* compute z for normalized dZ */
        /* with x = z/|z|, dx = z'/|z| - z*dot(z,z')/|z|**3 */
        /*  z  = ZL[0] and  */
        /*  z' = ZL[1]      */

        dmZ.x = z[1].x / mag - z[0].x * d12 / mag3;
        dmZ.y = z[1].y / mag - z[0].y * d12 / mag3;
        dmZ.z = z[1].z / mag - z[0].z * d12 / mag3;
    }    /* end curZ case*/
    else /* compute dZ with finite differences*/
    {
        /* copy Z*/
        N_CopyPt( *Z, &mZ );

        /* finite differences for dZ*/
        /* with prior_Z and next_Z     let dz = central difference = (nextZ - prevZ) / (du1 - du2)*/
        /* with just prior_Z or next_Z let dz = sided difference*/
        /* get two sample points*/
        if( prevZ )
        {
            F1 = prevZ;
            du1 = prev_u - u;
        }
        else
        {
            F1 = Z;
            du1 = 0;
        }

        if( nextZ )
        {
            F2 = nextZ;
            du2 = next_u - u;
        }
        else
        {
            F2 = Z;
            du2 = 0;
        }

        /* compute the finite difference*/
        dmZ.x = (F1->x - F2->x) / (du1 - du2);
        dmZ.y = (F1->y - F2->y) / (du1 - du2);
        dmZ.z = (F1->z - F2->z) / (du1 - du2);
    } /* end compute Z, dZ*/

    /* get sweep trajectory evaluations */
    error = N_CrvDerivs( traj, u, NL_LEFT, 2, t );

    if( error NEQ NL_NO )
        return (error);

    /* get mag of t[1]*/
    N_PtMagnitude( t[1], &mag );
    mag3 = mag * mag * mag;

    /* get d12 = dot_product(t[1],t[2])*/
    N_Dot2Pts( t[1], t[2], &d12 );

    /* set X = t'/|t'|*/
    X.x = t[1].x / mag;
    X.y = t[1].y / mag;
    X.z = t[1].z / mag;

    /* set dX = dx = t'' * |t'| - t'* (t' * t'')         */
    /*               ----------   --------------         */
    /*                 |t'|**2       |t'|**3             */
    dX.x = drma->RM[0][0] = t[2].x / mag - t[1].x * d12 / mag3;
    dX.y = drma->RM[1][0] = t[2].y / mag - t[1].y * d12 / mag3;
    dX.z = drma->RM[2][0] = t[2].z / mag - t[1].z * d12 / mag3;

    /* set dz = dZ*/
    drma->RM[0][2] = dmZ.x;
    drma->RM[1][2] = dmZ.y;
    drma->RM[2][2] = dmZ.z;

    /* set dy = cross_product(z',x) + cross_product(z,x')*/
    drma->RM[0][1] = (dmZ.y * X.z - dmZ.z * X.y) + (mZ.y * dX.z - mZ.z * dX.y);
    drma->RM[1][1] = (dmZ.z * X.x - dmZ.x * X.z) + (mZ.z * dX.x - mZ.x * dX.z);
    drma->RM[2][1] = (dmZ.x * X.y - dmZ.y * X.x) + (mZ.x * dX.y - mZ.y * dX.x);

    /* all done*/
    EXIT:

    return (error);
} /* end ST_GetRotationRateOfChange */




/*------------------------------------------------------------------------


   DESCRIPTION:

     This advanced surface construction routine creates a bspline 
     curve representing the start or end cross-tangent values for a 
     sweep surface, created by sweeping a cross-section curve along a trajectory 
     curve as done by N_CreateSweepScale().  The returned curves's knot vector 
     is the same as the cross-section curves. The cross tangents can
     be constructed for any cross-section.

     A typical calling example is:

     NL_VECTOR dtrajectory;       derivative of trajectory curve for selected param value
     NL_POINT  scale;             scale value for selected param value
     NL_VECTOR dscale;            derivative of scale value for selected param value
     NL_RMATRIX rma;              3x3 trajectory rotation matrix for selected param value
     NL_RMATRIX drma;             derivative of 3x3 trajectory rotation matrix for selected param value
     NL_CURVE *cross_section;     cross-section curve being swept
     NL_CURVE *cross_tangent;     cross-tangent curve to be computed
     NL_STACKS *SL;               stack to hold cross_tangent curve memory

     ...
     (compute dtrajectory, scale, dscale, rma, drma for common parameter value)
     ...

     N_SweepCrossTangentCrv(dtrajectory,
                                 scale,
                                 dscale,
                                 rma,
                                 drma,
                                 cross_section,
                                 cross_tangent,
                                 SL);


     The cross_tangent curve is a bspline approximation of the cross - tangent
     values for a swept surface constructed from the given cross_section curve.
     The cross_tangent bspline knot_vector is constructed to match the 
     input cross_section curve's knot_vector.

   ACCESS:

     dtrajectory   , input  , Derivative of the trajectory curve for a selected
                              parameter value.
     scale         , input  , sweep scale for selected parameter value
     dscale        , input  , derivative of sweep scale for selected parameter value
     rma           , input  , the sweep rotation for selected parameter value
                              ordered as:[x0   y0   z0]
                                         [x1   y1   z1]
                                         [x2   y2   z2]
     drma          , input  , derivative of the sweep rotation for selected parameter value
                              ordered as:[x0'  y0'  z0']
                                         [x1'  y1'  z1']
                                         [x2'  y2'  z2']
     cross_section , input  , input cross - section curve being swept(in its initial position)
     cross_tangent , output , cross - tangent curve for selected parameter values
     SL            , input  , cross_tangent's memory stack

   METHOD: 
     Cross - Tangent construction for swept surfaces:

     Using the notation from the NURBs book article 10.4,
     A sweep surface is defined as:
 
     S(u, v) = T(v) + M(v)C(u)

     where:  S(u, v) is the sweep surface,
             T(v) is the trajectory curve,
             M(v) is a scaled rotation matrix,
             C(u) is the cross - section curve being swept.
             u and v are independent parameters whose ranges are defined by curves
                T(v) and C(u).

     The tangents of S(u, v) in the v direction are found as:
 
     Sv(u, v) = Tv(v) + Mv(v)C(u)
 
     M(v) is built from the rotation and scaling portions of a coordinate system
     traveling along the curT as:

     M(v) =[scale_x(v)*x0(v)  scale_y(v)*y0(v)  scale_z(v)*z0(v)]
           [scale_x(v)*x1(v)  scale_y(v)*y1(v)  scale_z(v)*z1(v)]
           [scale_x(v)*x2(v)  scale_y(v)*y2(v)  scale_z(v)*z2(v)]
 
   
     The x, y, z coordinate system is not the Frenet frame, rather it is defined as:
 
     o(v)     = T(v)
     x(v)     = T'(v)/|T'(v)|
     z(v)     = B(v)/|B(v)|
     y(v)     = cross_product(z(v), x(v))
     scale(v) = curS(v) an input to the function.
 
     The construction of B(v) is given by Siltanen and Woodward's projection normal method.
 
        Given Bo a unit length vector perpendicular to T(v)
 
        bi = Bi - 1 - dot_product(Bi - 1, Ti) Ti
        Bi = bi/|bi|          note: this construction does not guarantee that dot_product(Ti, Bi) = 0
 
        for closed surfaces define B*i by letting 
        B*m = Bo
        b*i = Bi + 1 - dot_product(Bi + 1, Ti) Ti
        and
        Bi = 1/2(Bi + B*i)   note: this computation does not guarantee a unit vector for Bi
 
 
     The derivative of M(v) = Mv(v) is given as 

       Mv(v) =[scale_x(v)*x'0(v) + scale_x'*x(v)  scale_y(v)*y'0(v) + scale_y'*y0(v)  scale_z(v)*z'0(v) + scale_z'*z0(v)] 
              [scale_x(v)*x'1(v) + scale_x'*y(v)  scale_y(v)*y'1(v) + scale_y'*y1(v)  scale_z(v)*z'1(v) + scale_z'*z1(v)]
              [scale_x(v)*x'2(v) + scale_x'*z(v)  scale_y(v)*y'2(v) + scale_y'*y2(v)  scale_z(v)*z'2(v) + scale_z'*z2(v)]
 
       x'(v) = scale_x *(T''/|T'| - dot_product(T', T'') T' / |T'|**3) + scale_x' * x(v)
       z'(v) = scale_y *(B'/|B|   - dot_product(B, B')B/|B | **3)      + scale_y' * y(v)
       y'(v) = scale_z *(cross_product(z'(v), x(v)) + cross_product(z(v), x'(v))) + scale_z' * z(v)
 
       Given the discreet definition of B, the only way to define B' is by finite differences.
       So let's approximate all of  x', y', and z' by finite differences.
 
       For the start boundary:
       let x'(0) =(x(1) - x(0))/dv
       let y'(0) =(y(1) - y(0))/dv
       let z'(0) =(z(1) - z(0))/dv
 
       For the end boundary:
       let x'(n) =(x(n) - x(n - 1))/dv
       let y'(n) =(y(n) - y(n - 1))/dv
       let z'(n) =(z(n) - z(n - 1))/dv
 
  RETURN CODES:

    0 : No error
    1 : Error saved in NL_ERROR

  ---------------------------------------------------------------- */

NL_FLAG N_SweepCrossTangentCrv( NL_VECTOR *dtrajectory, NL_POINT *scale, NL_VECTOR *dscale, NL_RMATRIX *rma, NL_RMATRIX *drma, NL_CURVE *cross_section, NL_CURVE *cross_tangent, NL_STACKS *SL )
{
    /*
      NL_FLAG                          rtn: 0=success, 1=err in NL_ERROR                                              
      N_SweepCrossTangentCrv(  eff: calc and rtn cross_tangent curve for swept surface                    
      NL_VECTOR *dtrajectory,          in : derivative of trajectory curve for selected param value               
      NL_POINT  *scale,                in : scale value for selected param value                                  
      NL_VECTOR *dscale,               in : derivative of scale value for selected param value                    
      NL_RMATRIX *rma,                 in : derivative of 3x3 trajectory rotation matrix for selected param value 
                                         ordered: as: [ x0   y0   z0  ]                                        
                                                      [ x1   y1   z1  ]                                       
                                                      [ x2   y2   z2  ]                                        
      NL_RMATRIX *drma,                in : derivative of 3x3 trajectory rotation matrix for selected param value 
                                         ordered: as: [ x0'  y0'  z0' ]                                        
                                                      [ x1'  y1'  z1' ]                                        
                                                      [ x2'  y2'  z2' ]                                        
      NL_CURVE *cross_section,         in : cross-section curve being swept                                     
      NL_CURVE *cross_tangent,         out: cross-tangent curve to be computed                                   
      NL_STACKS *SL)                    in : stack to hold cross_tangent curve memory 
    */

    /* locals */
    NL_FLAG error = NL_NO;       /* return error value   */
    NL_INTEGER sample_count = 0; /* number of sample points   */
    NL_PARAMETER *params;        /* param value for every sample point, sized:[sample_count]  */
    NL_REAL du;                  /* param increment value  */
    NL_INDEX iSmp;               /* cross_tangent sample index */
    NL_POINT sample;             /* cross_section sample values  */
    NL_VECTOR *cross_tangents;   /* cross_tangent value for every sample point, sized:[sample_count] */
    NL_STACKS S1;                /* local memory manager */

    /* local input data */
    NL_INDEX i0, i1, jj, cnt; /* iterators */
    NL_DEGREE pc;             /* cross-section curve degree */
    NL_INTEGER nc, mc;        /* cross-section curve knot and control-point indices */
    NL_INDEX iSeg;            /* cross-section curve segment index */
    NL_INTEGER segment_count; /* cross-section curve segment count */
    NL_KNOTVECTOR *knt;       /* cross-section knot vector */

    /* set up local memory stack */
    N_InitNurbs( &S1 );

    /* get cross_section curve degree, indices, and knot_vector */
    N_CrvGetDegree( cross_section, &pc );
    N_CrvGetArraySizes( cross_section, &nc, &mc );
    N_CrvGetKnotVector( cross_section, &knt );

    /* count cross_section segments */
    segment_count = 0;

    /* for every knot pair */
    for ( i0 = 0, i1 = 1; i1 <= mc; i0++, i1++ )
    {
        /* when sequential knots are not equal - increment the segment_count */
        if( (knt->U[i1] - knt->U[i0])GT NL_PTOL )
            segment_count += 1;
    } /* end iter every knot pair */

    /* let sample_count be a function of curve degree and segment count */
    sample_count = pc * segment_count + 1;

    /* size local arrays: cross_tangents */
    cross_tangents = N_AllocPt1dArray( sample_count - 1, &S1 );

    if( cross_tangents EQ NULL )
        NL_QUIT;
    params = N_AllocReal1dArray( sample_count - 1, &S1 );

    if( params EQ NULL )
        NL_QUIT;

    /* compute an array of cross-tangent values */
    /* for every knot pair */
    for ( i0 = 0, i1 = 1, iSmp = 0, iSeg = 0; i1 <= mc; i0++, i1++ )
    {
        /* when sequential knots are not equal - found a segment */
        if( (knt->U[i1] - knt->U[i0])GT NL_PTOL )
        {
            /* sample each segment pc times (last segment gets pc+1) */
            /* +-----------+  for segment with pc = 3    */
            /* '   '   '      use these sample points for beginning and mid segments */
            /* '   '   '   '  use these sample points for the ending segment   */
            cnt = pc + ((iSeg == segment_count - 1) ? 1 : 0);

            /* let du = segment_width/samples_per_segment */
            du = (knt->U[i1] - knt->U[i0]) / pc;
            params[iSmp] = knt->U[i0];

            /* for every segment sample */
            for ( jj = 0; jj < cnt; jj++ )
            {
                /* sample cross_section curve */
                N_CrvEval( cross_section, params[iSmp], NL_LEFT, &sample );

                /* compute and save the cross_tangent value as */
                /* Sv(u,v) = Tv(v) + Mv(v)C(u) */
                /* with Tv(v) = dtrajectory  */
                /*      Mv(v) = [scale_x(v)*x'0(v)+scale_x'*x(v)  scale_y(v)*y'0(v)+scale_y'*y0(v)  scale_z(v)*z'0(v)+scale_z'*z0(v)] */
                /*              [scale_x(v)*x'1(v)+scale_x'*y(v)  scale_y(v)*y'1(v)+scale_y'*y1(v)  scale_z(v)*z'1(v)+scale_z'*z1(v)] */
                /*              [scale_x(v)*x'2(v)+scale_x'*z(v)  scale_y(v)*y'2(v)+scale_y'*y2(v)  scale_z(v)*z'2(v)+scale_z'*z2(v)] */
                /*       rma = [ x  y  z  ]  */
                /*      drma = [ x' y' z' ]  */
                cross_tangents[iSmp].x = dtrajectory->x + (scale->x * drma->RM[0][0] + dscale->x * rma->RM[0][0]) * sample.x + (scale->y * drma->RM[0][1] + dscale->y * rma->RM[0][1]) * sample.y + (scale->z * drma->RM[0][2] + dscale->z * rma->RM[0][2]) * sample.z;

                cross_tangents[iSmp].y = dtrajectory->y + (scale->x * drma->RM[1][0] + dscale->x * rma->RM[1][0]) * sample.x + (scale->y * drma->RM[1][1] + dscale->y * rma->RM[1][1]) * sample.y + (scale->z * drma->RM[1][2] + dscale->z * rma->RM[1][2]) * sample.z;

                cross_tangents[iSmp].z = dtrajectory->z + (scale->x * drma->RM[2][0] + dscale->x * rma->RM[2][0]) * sample.x + (scale->y * drma->RM[2][1] + dscale->y * rma->RM[2][1]) * sample.y + (scale->z * drma->RM[2][2] + dscale->z * rma->RM[2][2]) * sample.z;

                /* increment the sample count */
                iSmp += 1;

                /* set next param value */
                if( iSmp < sample_count )
                {
                    params[iSmp] = params[iSmp - 1] + du;
                }
            } /* end iter every sample in this segment */

            /* increment the segment count */
            iSeg += 1;
        } /* end found a segment check */
    }     /* end iter every knot pair */

    /* build cross-tangent curve to approximate the sample point cross-tangent values */
    N_CrvInitArrays( cross_tangent );
    error = N_FitCrvApproxKnots( (void *)cross_tangents, /* in : array of points to approximate, sized:[sample_count] */
    sample_count - 1,                                    /* in : Highest index in cross_tangents                      */
    NL_EPOINT,                                           /* in : using Euclidean (not Homogeneous) points            */
    params,                                              /* in : array of associated params, sized:[sample_count]   */
    knt,                                                 /* in : specified knot vector of approximating curve */
    nc,                                                  /* in : Highest index of approximating curve */
    pc,                                                  /* in : specified degree of approximating curve  */
    cross_tangent,                                       /* out: approximating curve  */
    SL );                                                /* i/o: cross_tangent curve memory stack */

    /* all done */
    EXIT:

    N_EndNurbs( &S1 );

    return (error);
} /* end N_SweepCrossTangentCrv */

/*******************************************************************//**


   DESCRIPTION:

     This  advanced  surface construction  routine  adjusts a derivative
     surface in homogeneous space to facilitate usage in cross-sectional 
     design. The derivative surface  must be Bezier in one  direction. A 
     typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_STACKS   SG;
       ...
       (get surP);
       ...
       N_AdjustDerivSrf(&surP,NL_UDIR,NL_LEFT,NL_BOTTOM,&surQ,&SG);

     MEMORY TO  HOLD THE  OUTPUT NL_SURFACE  surQ  IS ALLOCATED  INSIDE THE 
     ROUTINE!


   ACCESS:
   
     surP , input  ,  Bezier strip
     dir  , input  ,  Flag:
                        NL_UDIR: surP is Bezier in u-direction
                        NL_VDIR: surP is Bezier in v-direction
     cst  , input  ,  Construction flag:
                        NL_LEFT  : surP is constructed along u=umin
                        NL_RIGHT : surP is constructed along u=umax
                        NL_BOTTOM: surP is constructed along v=vmin
                        NL_TOP   : surP is constructed along v=vmax
     usg  , input  ,  Usage flag:
                        NL_LEFT  : surQ is used along u=umin
                        NL_RIGHT : surQ is used along u=umax
                        NL_BOTTOM: surQ is used along v=vmin
                        NL_TOP   : surQ is used along v=vmax
     surQ , output ,  New Bezier strip
     SG   , input  ,  Stack of surP and surQ


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AdjustDerivSrf( NL_SURFACE *surP, NL_FLAG dir, NL_FLAG cst, NL_FLAG usg, NL_SURFACE *surQ, NL_STACKS *SG )
{

    NL_FLAG sdr = NL_NO, error = NL_NO;

    /*        NL_INTEGER  **bin;                      */

    NL_DEGREE p, q;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Prepare */

    N_SrfGetDegrees( surP, &p, &q );

    if( dir EQ NL_UDIR )
        sdr = NL_VDIR;

    else if( dir EQ NL_VDIR )
        sdr = NL_UDIR;

    if( surP NEQ surQ )
        N_SrfInitArrays( surQ );

    /*        bin = N_AllocInt2dArray(pq,pq,&SL);           */
    /*        if( bin EQ NULL )  NL_QUIT;             */
    /*                                             */
    /*        N_PascalTriRow(bin,pq);                    */

    /* Create new surface */

    switch( cst )
    {
        case NL_LEFT:
            switch( usg )
            {
                case NL_LEFT:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_RIGHT:

                    error = N_BezSrfExtend4D( surP, dir, NL_START, NL_NO, surQ, SG );

                    if( error EQ NL_YES )
                        NL_OUT;
                    break;

                case NL_BOTTOM:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_TOP:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surQ, sdr, NL_START, NL_NO, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surP, sdr, NL_START, NL_NO, surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;
            }
            break;

        case NL_RIGHT:
            switch( usg )
            {
                case NL_LEFT:

                    error = N_BezSrfExtend4D( surP, dir, NL_END, NL_NO, surQ, SG );

                    if( error EQ NL_YES )
                        NL_OUT;
                    break;

                case NL_RIGHT:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_BOTTOM:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surQ, sdr, NL_END, NL_NO, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surP, sdr, NL_END, NL_NO, surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_TOP:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;
            }
            break;

        case NL_BOTTOM:
            switch( usg )
            {
                case NL_LEFT:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_RIGHT:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surQ, sdr, NL_START, NL_NO, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surP, sdr, NL_START, NL_NO, surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_BOTTOM:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_TOP:

                    error = N_BezSrfExtend4D( surP, dir, NL_START, NL_NO, surQ, SG );

                    if( error EQ NL_YES )
                        NL_OUT;
                    break;
            }
            break;

        case NL_TOP:
            switch( usg )
            {
                case NL_LEFT:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surQ, sdr, NL_END, NL_NO, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_BezSrfExtend4D( surP, sdr, NL_END, NL_NO, surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_RIGHT:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_SwapUV( surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SwapUV( surP, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;

                case NL_BOTTOM:

                    error = N_BezSrfExtend4D( surP, dir, NL_END, NL_NO, surQ, SG );

                    if( error EQ NL_YES )
                        NL_OUT;
                    break;

                case NL_TOP:
                    if( surP NEQ surQ )
                    {
                        error = N_SrfCopy( surP, surQ, SG );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    break;
            }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_AdjustDerivSrf */


/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine computes a  new weight 
     and a scaling factor for ensuring  NL_G1  continuity in  homogeneous 
     space. A typical calling example is:

       NL_CPOINT  Pw, Qw, Rw;
       NL_REAL    wr, alf;
       ...
       (get Pw, Qw and Rw);
       ...
       N_AdjustWeightScale(Pw,Qw,Rw,&wr,&alf);

     The points are arranged as Pw - Qw - Rw, the scaling is along the
     side  Qw - Rw, i.e. Rw  moves into a  new  position, and  the new 
     weight is for Rw.


   ACCESS:
   
     Pw  , input  ,  Control point
     Qw  , input  ,  Control point
     Rw  , input  ,  Control point
     wr  , output ,  New weight for Rw
     alf , output ,  Scale factor along Qw - Rw


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AdjustWeightScale( NL_CPOINT Pw, NL_CPOINT Qw, NL_CPOINT Rw, NL_REAL *wr, NL_REAL *alf )
{
    NL_PRIVATE NL_STRING rname = _T("N_AdjustWeightScale");

    NL_FLAG ifl, error = NL_NO;

    NL_REAL t1, t2, wp, wq, d1, d2;

    NL_POINT P, Q, Ps, Qs, Rs, As, Bs, Ds;

    NL_VECTOR V1, V2;

    NL_LINESEG ls1, ls2;

    /* Get new weight and scale factor */

    N_CPtToPtAndW( Pw, &P, &wp );
    N_CPtToPtAndW( Qw, &Q, &wq );

    N_CPtToPt( Pw, &Ps );
    N_CPtToPt( Qw, &Qs );
    N_CPtToPt( Rw, &Rs );

    if( fabs( wp - wq )LT NL_WTOL )
    {
        N_Combine2Pts( 2.0, Qs, -1.0, Ps, &Bs );

        *wr = 0.5 *( wp + wq );
    }
    else
    {
        *wr = (wq * wq) / wp;

        N_ScalePt( *wr, Q, &As );

        N_VectorDiff( Q, P, &V1 );
        N_CreateLineStartDirVector( &ls1, As, V1, NL_UNBOUNDED );
        N_VectorDiff( Qs, Ps, &V2 );
        N_CreateLineStartDirVector( &ls2, Ps, V2, NL_UNBOUNDED );

        error = N_IsectLineLine( ls1, ls2, &Bs, &t1, &t2, &ifl );

        if( error EQ NL_YES )
            NL_OUT;

        if( ifl EQ NL_FALSE )
            NL_ERROR( NL_GEO_ERR );
    }

    N_VectorDiff( Rs, Qs, &V1 );
    N_CreateLineStartDirVector( &ls1, Qs, V1, NL_UNBOUNDED );
    N_VectorDiff( Bs, NL_ZERO, &V2 );
    N_CreateLineStartDirVector( &ls2, NL_ZERO, V2, NL_UNBOUNDED );

    error = N_IsectLineLine( ls1, ls2, &Ds, &t1, &t2, &ifl );

    if( error EQ NL_YES )
        NL_OUT;

    if( ifl EQ NL_FALSE )
        NL_ERROR( NL_GEO_ERR );

    N_DistPtPt( Qs, Ds, &d1 );
    N_DistPtPt( Qs, Rs, &d2 );

    if( N_FloatOpIsBad( d1, d2, NL_DIVISION ) )
        NL_ERROR( NL_GEO_ERR );

    *alf = d1 / d2;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_AdjustWeightScale */



/*******************************************************************//**


   DESCRIPTION:

     This advanced surface  construction routine  approximates a cross-
     boundary  derivative field up to a  given angular  tolerance. That 
     is, given D(t), the approximation D_a(t) satisfies:

       (1) ANGLE<D(t_f),D_a(t_f)> < eps 
       (2) Ds = D_a'(tmin)
       (3) De = D_a'(tmax)

     where Ds and De are given end derivatives, t_f is any parameter in
     [tmin,tmax], and  eps is an  angular  tolerance. A typical calling 
     example is:

       NL_CURVE   derP, derQ;
       NL_VECTOR  Ds, De;
       NL_REAL    eps;
       NL_STACKS  SG;
       ...
       (get derP, Ds, De and eps);
       ...
       N_CrvInitArrays(&derQ);
       N_ApproxCrossBoundaryDerivs(&derP,Ds,De,eps,&derQ,&SG);
       N_ApproxCrossBoundaryDerivs(&derP,Ds,De,eps,&derP,&SG);

     THIS VERSION IS FOR NON-RATIONAL  CROSS-BOUNDARY DERIVATIVES ONLY!
     THE APPROXIMATION MAY BE DONE  IN-PLACE, I.E. THE INPUT NL_CURVE WILL
     BE DESTROYED.


   ACCESS:
   
     derP  , in/out ,  Cross-boundary derivative
     Ds,De , input  ,  Start and end derivatives
     eps   , input  ,  Angular tolerance in degrees
     derQ  , output ,  Approximation of derP
     SG    , input  ,  derQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxCrossBoundaryDerivs( NL_CURVE *derP, NL_VECTOR Ds, NL_VECTOR De, NL_REAL eps, NL_CURVE *derQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxCrossBoundaryDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m, mx;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *UX, dts, dte, fs, fe, als, ale, rad, sne, a, b, c, ts = 0.0, te = 0.0, deltat, tmid;

    NL_POINT CD[2], Ps, Pe, As, Ae, Bs, Be, A;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE curA;

    NL_KNOTVECTOR *knx;

    NL_BOOLEAN Repeat;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if angular tolerance is maintained */

    if( N_IsCrvRat( derP ) )
        NL_ERROR( NL_INP_ERR );

    Repeat = NL_FALSE;

    do
    {

        N_CrvGetCPtsDegreeAndKnots( derP, &n, &Pw, &p, &m, &UP );

        dts = UP[p + 1] - UP[p];
        dte = UP[n + 1] - UP[n];
        fs = dts / p;
        fe = dte / p;

        N_CPtToPtEuclid( Pw[0], &Ps );
        N_CPtToPtEuclid( Pw[1], &As );
        N_CPtToPtEuclid( Pw[n - 1], &Ae );
        N_CPtToPtEuclid( Pw[n], &Pe );

        N_VectorPtAlongVector( Ps, fs, Ds, &Bs );
        N_VectorPtAlongVector( Pe, -fe, De, &Be );

        N_VectorsAngle( As, Bs, &als );
        N_VectorsAngle( Ae, Be, &ale );

        if( als LE eps AND ale LE eps )   /* angle check in degrees */
        {
            if( derP EQ derQ )
            {
                N_PtToCPt( Bs, &Pw[1] );
                N_PtToCPt( Be, &Pw[n - 1] );
            }
            else
            {
                error = N_CrvSizeArrays( derQ, n, p, m, rname, SG );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPtsAndKnots( derQ, &Qw, &UQ );

                for ( i = 0; i <= n; i++ )
                    N_CopyCPt( Pw[i], &Qw[i] );

                for ( i = 0; i <= m; i++ )
                    UQ[i] = UP[i];

                N_PtToCPt( Bs, &Qw[1] );
                N_PtToCPt( Be, &Qw[n - 1] );
            }

            NL_OUT;
        }

        /* Insert knots to bring derivatives in compliance */

        rad = (eps * NL_PI) / 180.0;
        sne = sin( rad );
        mx = -1;

        if( als GT eps )
        {
            error = N_CrvDerivs( derP, UP[0], NL_LEFT, 1, CD );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorMagnitude( Ps, &a );
            N_VectorDiff( Ds, CD[1], &A );
            N_VectorMagnitude( A, &b );
            N_VectorMagnitude( CD[1], &c );

            dts = (sne * p * a) / (b + sne * c);
            ts = UP[p] + dts;

            if( ts GT UP[p + 1] )
                NL_ERROR( NL_KNT_ERR );

            mx++;
        }

        if( ale GT eps )
        {
            error = N_CrvDerivs( derP, UP[m], NL_LEFT, 1, CD );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorMagnitude( Pe, &a );
            N_VectorDiff( De, CD[1], &A );
            N_VectorMagnitude( A, &b );
            N_VectorMagnitude( CD[1], &c );

            dte = (sne * p * a) / (b + sne * c);
            te = UP[n + 1] - dte;

            if( te LT UP[n] )
                NL_ERROR( NL_KNT_ERR );

            mx++;
        }

        /* RMB This algorithm assumes that ts < te but     */
        /* when there is only one span (Bezier, n = p)     */
        /* it is possible that te < ts and this gives      */
        /* an error, the new knots are out of order        */

        if( Repeat EQ NL_FALSE AND n EQ p AND mx EQ 1 )
        { /* als and ale are both > eps  */
            deltat = UP[p + 1] - UP[p];

            if( (te - ts)LT 0.25 *deltat )
            { /* insert a mid knot and reset ts, te and mx  */
                tmid = UP[p] + 0.5 *deltat;
                error = N_CrvInsertKnot( derP, tmid, 1, derP, SG, SG );

                if( error EQ NL_YES )
                    NL_OUT;
                Repeat = NL_TRUE;
            }
        }
        else
            Repeat = NL_FALSE;
    } while ( Repeat EQ NL_TRUE );

    knx = N_AllocKnotVectorAndArray( mx, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( knx, &mx, &UX );

    if( als GT eps AND ale LT eps )
    {
        UX[0] = ts;
        n++;
        m++;
    }
    else if( als LT eps AND ale GT eps )
    {
        UX[0] = te;
        n++;
        m++;
    }
    else if( als GT eps AND ale GT eps )
    {
        UX[0] = ts;
        UX[1] = te;
        n += 2;
        m += 2;
    }

    if( derP EQ derQ )
    {
        error = N_CrvRefine( derP, knx, derP, SG, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( derP, &Pw, &UP );

        dts = UP[p + 1] - UP[p];
        dte = UP[n + 1] - UP[n];
        fs = dts / p;
        fe = dte / p;

        N_VectorPtAlongVector( Ps, fs, Ds, &Bs );
        N_VectorPtAlongVector( Pe, -fe, De, &Be );
        N_PtToCPt( Bs, &Pw[1] );
        N_PtToCPt( Be, &Pw[n - 1] );
    }
    else
    {
        N_CrvInitArrays( &curA );
        error = N_CrvRefine( derP, knx, &curA, SG, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvSizeArrays( derQ, n, p, m, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( &curA, &Pw, &UP );
        N_CrvGetCPtsAndKnots( derQ, &Qw, &UQ );

        for ( i = 0; i <= n; i++ )
            N_CopyCPt( Pw[i], &Qw[i] );

        for ( i = 0; i <= m; i++ )
            UQ[i] = UP[i];

        dts = UP[p + 1] - UP[p];
        dte = UP[n + 1] - UP[n];
        fs = dts / p;
        fe = dte / p;

        N_VectorPtAlongVector( Ps, fs, Ds, &Bs );
        N_VectorPtAlongVector( Pe, -fe, De, &Be );
        N_PtToCPt( Bs, &Qw[1] );
        N_PtToCPt( Be, &Qw[n - 1] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ApproxCrossBoundaryDerivs */


/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine creates a bicubic Coons
     surface interpolating four NON-RATIONAL boundary curves and cross-
     boundary derivatives. The boundary curves are assumed to intersect 
     at their  respective  end  points,  and the  cross-derivatives are 
     assumed to be compatible in the Coons sense  (including  their im-
     plied twists). curL,derL,curR,derR must be defined on the same  v-
     parameter  domain,  and curB,derB,curT,derT must be defined on the 
     same u-parameter domain  (not  necessarily  the same two domains).  
     The Coons surface will have the parameterization of these  curves.
     If the  output surface is initialized to NULL, memory to store new 
     control points and knots is allocated.  A  typical calling example
     is:

       NL_CURVE    curL, derL, curR, derR, curB, derB, curT, derT;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (get boundary curves and derivatives);
       ...
       N_SrfInitArrays(&sur);
       N_CreateCoonsBoundaryCrvs(&curL,&derL,&curR,&derR,&curB,&derB,&curT,&derT,NL_NO,
                &sur,&SC,&SS);

     If memory is  available, sur  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest  indexes in sur's 
     knot  vector and  polygon  objects.  ALL INPUT DATA MUST BE ON THE  
     SAME STACK  "SC". OPPOSITE INPUT  CURVES AND  DERIVATIVES  MUST BE
     COMPATIBLE IN  THE  B-SPLINE  SENSE. IF  THEY ARE NOT, THE ROUTINE 
     MAKES THEM COMPATIBLE, I.E. THE INPUT DATA MAY BE DESTROYED. NL_TWIST
     AND  NL_DERIVATIVE COMPATIBILITIES ARE  CHECKED UPON REQUEST. IF  THE 
     NL_NO CHECK OPTION IS WANTED, AND IF NL_NO COMPATIBILITY IS PRESENT, THE
     ROUTINE  CREATES A  NL_SURFACE  THAT VIOLATES  THE  COONS  CONDITION, 
     HOWEVER, IT INTERPOLATES THE BOUNDARY CURVES (AND MAYBE USEFUL FOR
     SUBSEQUENT OPERATIONS, E.G. NL_POINT PROJECTION FOR PARAMETRIZATION).


   ACCESS:
   
     curL  , in/out ,  Left boundary
     derL  , in/out ,  Cross-boundary derivative across curL
     curR  , in/out ,  Right boundary
     derR  , in/out ,  Cross-boundary derivative across curR
     curB  , in/out ,  Bottom boundary
     derB  , in/out ,  Cross-boundary derivative across curB
     curT  , in/out ,  Top boundary
     derT  , in/out ,  Cross-boundary derivative across curT
     cpf   , input  ,  Flag:
                         NL_YES: check twist & derivative compatibilities
                         NL_NO : do not check compatibilities
     sur   , output ,  Bicubic Coons surface
     SC    , input  ,  curs' and ders' memory stack
     SS    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCoonsBoundaryCrvs( NL_CURVE *curL, NL_CURVE *derL, NL_CURVE *curR, NL_CURVE *derR, NL_CURVE *curB, NL_CURVE *derB, NL_CURVE *curT, NL_CURVE *derT, NL_FLAG cpf, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCoonsBoundaryCrvs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UA, *VA, one_3, one_9, mag, da, db, d1, d2, us, ue, vs, ve, du, dv;

    NL_POINT CD1[2], CD2[2], T00, T01, T10, T11, A, B;

    NL_CPOINT ** Pw, ** Uw, ** Vw, ** Tw, *Aw, T00w, T10w, T01w, T11w;

    NL_CURVE ** curAA, ** curBB;

    NL_SURFACE ** surA, surU, surV, surT;

    NL_STACKS SL;

    NL_REAL dTwistTol = NL_MTOL * 1.0e2;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check rationality, parameter domains, and initialize constants */

    if( N_IsCrvRat( curL ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curR ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curB ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curT ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derL ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derR ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derB ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derT ) )
        NL_ERROR( NL_INP_ERR );

    one_3 = 1.0 / 3.0;
    one_9 = 1.0 / 9.0;

    N_CrvGetParamBounds( curB, &us, &ue );
    N_CrvGetParamBounds( curT, &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derB, &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derT, &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    du = ue - us;

    N_CrvGetParamBounds( curL, &vs, &ve );
    N_CrvGetParamBounds( curR, &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derL, &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derR, &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    dv = ve - vs;

    /* Make input data compatible in the B-spline sense */

    curAA = N_AllocArrayCrvPtrs( 3, SC );

    if( curAA EQ NULL )
        NL_QUIT;

    curBB = N_AllocArrayCrvPtrs( 3, SC );

    if( curBB EQ NULL )
        NL_QUIT;

    curAA[0] = curL;
    curAA[1] = curR;
    curAA[2] = derL;
    curAA[3] = derR;

    curBB[0] = curB;
    curBB[1] = curT;
    curBB[2] = derB;
    curBB[3] = derT;

    if( NOT N_CrvsAreCombatible( curAA, 3 ) )
    {
        error = N_CrvsMakeCompatible( curAA, 3, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( NOT N_CrvsAreCombatible( curBB, 3 ) )
    {
        error = N_CrvsMakeCompatible( curBB, 3, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check compatibility */

    N_CrvGetKnots( curBB[0], &r, &U );
    N_CrvGetKnots( curAA[0], &s, &V );

    if( cpf EQ NL_YES )
    {
        error = N_CrvEval( curAA[2], V[0], NL_LEFT, &A );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( curAA[2], V[s], NL_LEFT, &B );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curBB[0], U[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curBB[1], U[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( A, CD1[1], &A );
        N_VectorDiff( B, CD2[1], &B );
        N_VectorMagnitude( A, &da );
        N_VectorMagnitude( B, &db );

        if( da GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        if( db GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        error = N_CrvEval( curAA[3], V[0], NL_LEFT, &A );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( curAA[3], V[s], NL_LEFT, &B );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curBB[0], U[r], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curBB[1], U[r], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( A, CD1[1], &A );
        N_VectorDiff( B, CD2[1], &B );
        N_VectorMagnitude( A, &da );
        N_VectorMagnitude( B, &db );

        if( da GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        if( db GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        error = N_CrvEval( curBB[2], U[0], NL_LEFT, &A );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( curBB[2], U[r], NL_LEFT, &B );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curAA[0], V[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curAA[1], V[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( A, CD1[1], &A );
        N_VectorDiff( B, CD2[1], &B );
        N_VectorMagnitude( A, &da );
        N_VectorMagnitude( B, &db );

        if( da GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        if( db GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        error = N_CrvEval( curBB[3], U[0], NL_LEFT, &A );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( curBB[3], U[r], NL_LEFT, &B );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curAA[0], V[s], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curAA[1], V[s], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( A, CD1[1], &A );
        N_VectorDiff( B, CD2[1], &B );
        N_VectorMagnitude( A, &da );
        N_VectorMagnitude( B, &db );

        if( da GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );

        if( db GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );
    }

    error = N_CrvDerivs( curAA[2], V[0], NL_LEFT, 1, CD1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curBB[2], U[0], NL_LEFT, 1, CD2 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDiff( CD1[1], CD2[1], &A );
    N_VectorMagnitude( A, &mag );

    if( cpf EQ NL_YES AND mag GT dTwistTol )
        NL_ERROR( NL_INP_ERR );
    N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T00 );

    error = N_CrvDerivs( curBB[2], U[r], NL_LEFT, 1, CD1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curAA[3], V[0], NL_LEFT, 1, CD2 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDiff( CD1[1], CD2[1], &A );
    N_VectorMagnitude( A, &mag );

    if( cpf EQ NL_YES AND mag GT dTwistTol )
        NL_ERROR( NL_INP_ERR );
    N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T10 );

    error = N_CrvDerivs( curAA[3], V[s], NL_LEFT, 1, CD1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curBB[3], U[r], NL_LEFT, 1, CD2 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDiff( CD1[1], CD2[1], &A );
    N_VectorMagnitude( A, &mag );

    if( cpf EQ NL_YES AND mag GT dTwistTol )
        NL_ERROR( NL_INP_ERR );
    N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T11 );

    error = N_CrvDerivs( curBB[3], U[0], NL_LEFT, 1, CD1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curAA[2], V[s], NL_LEFT, 1, CD2 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDiff( CD1[1], CD2[1], &A );
    N_VectorMagnitude( A, &mag );

    if( cpf EQ NL_YES AND mag GT dTwistTol )
        NL_ERROR( NL_INP_ERR );
    N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T01 );

    /********************************/
    /* Compute the various surfaces */
    /********************************/

    /* Allocate memory */

    N_CrvGetCPtsDegreeAndKnots( curAA[0], &m, &Aw, &q, &s, &VA );
    N_CrvGetCPtsDegreeAndKnots( curBB[0], &n, &Aw, &p, &r, &UA );

    error = N_AllocSrfArrays( &surU, 3, m, 3, q, 7, s, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocSrfArrays( &surV, n, 3, p, 3, r, 7, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocSrfArrays( &surT, 3, 3, 3, 3, 7, 7, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute u-blend */

    N_SrfGetCPtsAndKnots( &surU, &Uw, &U, &V );

    N_CrvGetCPts( curAA[0], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_CopyCPt( Aw[j], &Uw[0][j] );

    N_CrvGetCPts( curAA[1], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_CopyCPt( Aw[j], &Uw[3][j] );

    N_CrvGetCPts( curAA[2], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_Combine2CPts( 1.0, Uw[0][j], du * one_3, Aw[j], &Uw[1][j] );

    N_CrvGetCPts( curAA[3], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_Combine2CPts( 1.0, Uw[3][j], -du * one_3, Aw[j], &Uw[2][j] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* Compute v-blend */

    N_SrfGetCPtsAndKnots( &surV, &Vw, &U, &V );

    N_CrvGetCPts( curBB[0], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Aw[i], &Vw[i][0] );

    N_CrvGetCPts( curBB[1], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Aw[i], &Vw[i][3] );

    N_CrvGetCPts( curBB[2], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_Combine2CPts( 1.0, Vw[i][0], dv * one_3, Aw[i], &Vw[i][1] );

    N_CrvGetCPts( curBB[3], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_Combine2CPts( 1.0, Vw[i][3], -dv * one_3, Aw[i], &Vw[i][2] );

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    /* Compute tensor product surface */

    N_SrfGetCPtsAndKnots( &surT, &Tw, &U, &V );

    for ( i = 0; i <= 3; i++ )
    {
        N_CopyCPt( Uw[i][0], &Tw[i][0] );
        N_CopyCPt( Uw[i][m], &Tw[i][3] );
    }

    for ( j = 1; j <= 2; j++ )
    {
        N_CopyCPt( Vw[0][j], &Tw[0][j] );
        N_CopyCPt( Vw[n][j], &Tw[3][j] );
    }

    N_PtToCPt( T00, &T00w );
    N_PtToCPt( T10, &T10w );
    N_PtToCPt( T01, &T01w );
    N_PtToCPt( T11, &T11w );

    d1 = du * dv * one_9;

    N_Combine4CPts( d1, T00w, 1.0, Tw[1][0], 1.0, Tw[0][1], -1.0, Tw[0][0], &Tw[1][1] );
    N_Combine4CPts( -d1, T10w, 1.0, Tw[3][1], 1.0, Tw[2][0], -1.0, Tw[3][0], &Tw[2][1] );
    N_Combine4CPts( -d1, T01w, 1.0, Tw[1][3], 1.0, Tw[0][2], -1.0, Tw[0][3], &Tw[1][2] );
    N_Combine4CPts( d1, T11w, 1.0, Tw[2][3], 1.0, Tw[3][2], -1.0, Tw[3][3], &Tw[2][2] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    /* Make surfaces compatible */

    surA = N_AllocArraySrfPtrs( 2, &SL );

    if( surA EQ NULL )
        NL_QUIT;

    surA[0] = &surU;
    surA[1] = &surV;
    surA[2] = &surT;

    error = N_MakeSrfsCompatibleUV( surA, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute output surface */

    N_SrfGetCPtsDegreesAndKnots( surA[0], &n, &m, &Uw, &p, &q, &r, &s, &UA, &VA );

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPts( surA[1], &n, &m, &Vw );
    N_SrfGetCPts( surA[2], &n, &m, &Tw );
    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TranslateSum2CPts( Uw[i][j], 1.0, Vw[i][j], -1.0, Tw[i][j], &Pw[i][j] );
        }
    }

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCoonsBoundaryCrvs */



/*******************************************************************//**


   DESCRIPTION:

     This advanced  surface  construction  routine  computes a  cross-
     boundary derivative curve  given the boundary, the normals at the 
     end points, and the start and the end cross-boundary derivatives.
     A typical calling example is:

       NL_CURVE   cur, der;
       NL_VECTOR  Ns, Ne, Bs, Be;
       NL_STACKS  SG;
       ...
       (get cur, Ns, Ne, Bs, Be);
       ...
       N_CreateCrossBoundaryDerivCrv(&cur,Ns,Ne,Bs,Be,&der,&SG);

     MEMORY TO HOLD  der  IS  ALLOCATED INSIDE THE ROUTINE!  der  MUST 
     BE DECLARED AS SHOWN ABOVE!


   ACCESS:
   
     cur   , input  ,  Boundary curve
     Ns,Ne , input  ,  UNIT normals at the end points of cur
     Bs,Be , input  ,  Cross-boundary derivatives at the end points of
                       cur
     der   , output ,  Cross-boundary derivative
     SG    , output ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCrossBoundaryDerivCrv( NL_CURVE *cur, NL_VECTOR Ns, NL_VECTOR Ne, NL_VECTOR Bs, NL_VECTOR Be, NL_CURVE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCrossBoundaryDerivCrv");

    NL_FLAG ifl, error = NL_NO;

    NL_INDEX n, m;

    NL_REAL *U, *fu, p0, p1, q0, q1, t1, t2, dot, mds, mde, w0, wn;

    NL_POINT CD[2], Ps, Pe, Vs, Ve, A, B;

    NL_VECTOR Ts, Te, Ds, De;

    NL_CPOINT *Pw;

    NL_LINESEG ls1, ls2;

    NL_CFUN *cfnP, *cfnQ;

    NL_CURVE *curT, curD, pd, qt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get start and end coordinates (p0,p1) and (q0,q1) so that  */
    /* Bs = p0*Ds + q0*Ts and Be = p1*De + q1*Te, where Ds and De */
    /* are the start and end derivatives of cur                   */

    N_CrvGetKnots( cur, &m, &U );

    error = N_CrvDerivs( cur, U[0], NL_LEFT, 1, CD );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( CD[0], &Ps );
    N_VectorCopy( CD[1], &Ds );
    N_VectorMagnitude( Ds, &mds );

    error = N_CrvDerivs( cur, U[m], NL_LEFT, 1, CD );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( CD[0], &Pe );
    N_VectorCopy( CD[1], &De );
    N_VectorMagnitude( De, &mde );

    N_VectorCross( Ns, Ds, &Ts );
    N_VectorCross( Ne, De, &Te );
    N_VectorSum( Ps, Bs, &Vs );
    N_VectorSum( Pe, Be, &Ve );

    error = N_VectorNormalizeRef( &Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreateLineStartDirVector( &ls1, Ps, Ds, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &ls2, Vs, Ts, NL_UNBOUNDED );

    error = N_IsectLineLine( ls1, ls2, &A, &t1, &t2, &ifl );

    if( error EQ NL_YES )
        NL_OUT;

    if( ifl EQ NL_FALSE )
        NL_ERROR( NL_GEO_ERR );

    N_CreateLineStartDirVector( &ls1, Ps, Ts, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &ls2, Vs, Ds, NL_UNBOUNDED );

    error = N_IsectLineLine( ls1, ls2, &B, &t1, &t2, &ifl );

    if( error EQ NL_YES )
        NL_OUT;

    if( ifl EQ NL_FALSE )
        NL_ERROR( NL_GEO_ERR );

    N_VectorDiff( A, Ps, &A );
    N_VectorDiff( B, Ps, &B );
    N_VectorMagnitude( A, &p0 );
    N_VectorMagnitude( B, &q0 );

    p0 = p0 / mds;

    N_VectorDot( A, Ds, &dot );

    if( dot LT 0.0 )
        p0 = -p0;

    N_VectorDot( B, Ts, &dot );

    if( dot LT 0.0 )
        q0 = -q0;

    N_CreateLineStartDirVector( &ls1, Pe, De, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &ls2, Ve, Te, NL_UNBOUNDED );

    error = N_IsectLineLine( ls1, ls2, &A, &t1, &t2, &ifl );

    if( error EQ NL_YES )
        NL_OUT;

    if( ifl EQ NL_FALSE )
        NL_ERROR( NL_GEO_ERR );

    N_CreateLineStartDirVector( &ls1, Pe, Te, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &ls2, Ve, De, NL_UNBOUNDED );

    error = N_IsectLineLine( ls1, ls2, &B, &t1, &t2, &ifl );

    if( error EQ NL_YES )
        NL_OUT;

    if( ifl EQ NL_FALSE )
        NL_ERROR( NL_GEO_ERR );

    N_VectorDiff( A, Pe, &A );
    N_VectorDiff( B, Pe, &B );
    N_VectorMagnitude( A, &p1 );
    N_VectorMagnitude( B, &q1 );

    p1 = p1 / mde;

    N_VectorDot( A, De, &dot );

    if( dot LT 0.0 )
        p1 = -p1;

    N_VectorDot( B, Te, &dot );

    if( dot LT 0.0 )
        q1 = -q1;

    /* Get linear functions */

    cfnP = N_AllocCrvFunc( 1, 1, 3, &SL );

    if( cfnP EQ NULL )
        NL_QUIT;

    cfnQ = N_AllocCrvFunc( 1, 1, 3, &SL );

    if( cfnQ EQ NULL )
        NL_QUIT;

    N_CrvFuncCntrlValKnots( cfnP, &fu, &U );

    fu[0] = p0;
    fu[1] = p1;
    U[0] = U[1] = 0.0;
    U[2] = U[3] = 1.0;

    N_CrvFuncCntrlValKnots( cfnQ, &fu, &U );

    fu[0] = q0;
    fu[1] = q1;
    U[0] = U[1] = 0.0;
    U[2] = U[3] = 1.0;

    /* Get derivative curve */

    N_CrvInitArrays( &curD );
    error = N_CrvRatGetFirstDeriv( cur, &curD, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get tangent curve */

    curT = N_AllocCrvAndArrays( 1, 1, 3, &SL );

    if( curT EQ NULL )
        NL_QUIT;

    N_CrvGetCPtsAndKnots( curT, &Pw, &U );

    N_PtToCPt( Ts, &Pw[0] );
    N_PtToCPt( Te, &Pw[1] );

    U[0] = U[1] = 0.0;
    U[2] = U[3] = 1.0;

    /* Compute cross-boundary curve B(v) = p(v)*D(v) + q(v)*T(v) */

    N_CrvInitArrays( &pd );
    error = N_CrvFuncMultiplyCrv( cfnP, &curD, &pd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &qt );
    error = N_CrvFuncMultiplyCrv( cfnQ, curT, &qt, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( der );
    error = N_CrvSumDiffCrv( &pd, &qt, NL_PLUS, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPts( der, &n, &Pw );

    if( N_IsCrvRat( der ) )
    {
        N_CPtGetW( Pw[0], &w0 );
        N_CPtGetW( Pw[n], &wn );

        N_Weight( Bs, w0, &Pw[0] );
        N_Weight( Be, wn, &Pw[n] );
    }
    else
    {
        N_PtToCPt( Bs, &Pw[0] );
        N_PtToCPt( Be, &Pw[n] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCrossBoundaryDerivCrv */

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface  construction  routine  computes a collection
     of patches to  fill an  n-sided hole. Given n  boundary curves  and
     (optionally) n  cross-boundary derivatives, n  NURBS  surfaces  are
     computed that join the given derivatives and each other  in approx-
     imate NL_G1 continuity. To  handle twist  incompatibility,  the cross-
     boundary derivatives are approximated to a given angular tolerance,
     i.e.  the unit normals along a boundary curve do  not  deviate more
     than the specified  tolerance. The inner  curves  run from the mid-
     point of each boundary to a center point where they share the  same
     end normal, the center normal. The center point and the center nor-
     mal may be passed in.  This version  requires that the input curves 
     and derivatives be NON-RATIONAL. They must be input in a consistent 
     manner:
     (1) the end point of cur[i] must be the start point of cur[i+1]; 
     (2) the end vectors of der[i] must be collinear (anti-collinear)
         with the end derivatives of cur[i-1] and cur[i+1]. 
     A typical calling example is:

       NL_CURVE    **cur, **der;
       NL_INDEX    k;
       NL_REAL     eps;
       NL_VECTOR   CI, NI;
       NL_SURFACE  **sur;
       NL_STACKS   SI, SS;
       ...
       (get cur, der, eps, and optionally CI and NI);
       ...
       N_FillNSidedHole(cur,der,k,eps,&CI,&NI,NL_NO,&sur,&SI,&SS);

     MEMORY TO HOLD  sur IS  ALLOCATED INSIDE THE ROUTINE! sur  MUST BE 
     BE DECLARED AS SHOWN ABOVE! THE  BOUNDARY NL_CURVE  cur[i] AND CROSS-
     BOUNDARY NL_DERIVATIVE  (IF PASSED IN) der[i] MUST BE  COMPATIBLE. IF 
     THEY ARE NOT,  THEY ARE MADE  COMPATIBLE INTERNALLY.  cur and  der 
     MUST BE ON THE SAME STACK SI!


   ACCESS:
   
     cur   , in/out ,  NON-RATIONAL  boundary curves; cur[0],...,cur[k] 
                       must be  input  consecutively along  the n-sided 
                       boundary
     der   , in/out ,  NON-RATIONAL  boundary  derivatives; der[0],...,
                       der[k] must be input  consecutively along the n-
                       sided boundary. If  der = NULL,  no  derivatives 
                       are passed in. If der != NULL, then:
                         der[i]  = NULL: no  derivative  passed in  for 
                                         cur[i],   must   be   computed 
                                         internally
                         der[i] != NULL: derivative passed in
                       der[i] MUST BE COMPATIBLE WITH ITS CORRESPONDING
                       BOUNDARY  cur[i], i.e. MUST BE  DEFINED OVER THE 
                       SAME KNOT VECTOR! IF THEY ARE NOT  COMPATIBLE ON
                       INPUT, THE ROUTINE  MAKES THEM  COMPATIBLE, I.E.
                       THEY WILL BE DESTROYED.
     k     , input  ,  Highest index in cur and der arrays
     eps   , input  ,  Angular  tolerance in degrees;  angles  between   
                       the normals along the common  boundaries of  
                       joining surfaces do not deviate more than eps
     CI,NI , in/out ,  Center  point and UNIT center  normal of tangent
                       plane at CI
     flg   , input  ,  Flag:
                         NL_YES: CI and NI are passed in
                         NL_NO : compute CI and NI internally
     sur   , output ,  (k+1) surfaces interpolating boundary curves and
                       approximating cross-boundary derivatives
     SI    , input  ,  cur's and der's stack
     SS    , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FillNSidedHole                                                                                       
 ( NL_CURVE    ** cur,       /* in : NON-RATIONAL  boundary curves; cur[0],...,cur[k]                          */
                             /*      must be  input  consecutively along  the n-sided                          */
                             /*      boundary                                                                  */
   NL_CURVE    ** der,       /* in : NON-RATIONAL compatible corresponding cross-derivative curves.            */
                             /*      der=NULL:No curves specified, else der[i]=NULL:just der[i] not specified. */
                             /*      specified der values approximated to eps angle in degrees                 */
   NL_INDEX       k,         /* in : Highest index in cur and der arrays                                       */
   NL_REAL        eps,       /* in : max angDeg allowed between NSidedPatch bndryCrossDerivs and der values    */
   NL_POINT     * CI,        /* in : CenterPoint of plane tangent to NSidedPatch at its center                 */
   NL_VECTOR    * NI,        /* in : UnitNormal  of plane tangent to NSidedPatch at its center                 */ 
   NL_FLAG        flg,       /* in : NL_YES = CI and NI are passed in                                          */
                             /*      NL_NO  = compute CI and NI internally                                     */
   NL_SURFACE *** sur,       /* out: (k+1) surfaces, interpolating boundary curves, and                        */
                             /*       approximating cross-boundary derivatives                                 */
   NL_STACKS    * SI,        /* in : cur's and der's stack                                                     */
   NL_STACKS    * SS )       /* in : sur's stack                                                               */
{
    NL_PRIVATE NL_STRING rname = _T("N_FillNSidedHole");
    NL_PRIVATE NL_REAL lto = 0.05;

    NL_FLAG le = NL_YES, ri = NL_YES, bo = NL_YES, to = NL_YES, error = NL_NO;

    NL_DEGREE p, pl, pr, ph;

    NL_INDEX i, j, n, m, ml, mr, mi, mx, a, b, r, s;

    NL_REAL *U, *V, *UL, *UR, *UI, *UX, *UF, *fu, um, dl, dr, fac, mag, la, alf, bet, gam, del;

    NL_POINT *M, *Q, *D, CD1[2], CD2[2], CC, A, B, ML, MR, DL, DR;

    NL_VECTOR *N, NN, F, T0, T3, D1, D2, D3, D4;

    NL_CPOINT *Pw;

    NL_PLANE pln;

    NL_KNOTVECTOR ** kna, ** knm, *knu, *knv, *knx;

    NL_CFUN *cfn;

    NL_CURVE ** curL, ** curR, ** derL, ** derR, ** curI, ** bdrL, ** bdrR, ** curC, *derA, *curA;

    NL_SURFACE ** surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( k LT 2 )
        NL_ERROR( NL_INP_ERR );

    for ( i = 0; i <= k; i++ )
    {
        if( N_IsCrvRat( cur[i] ) )
            NL_ERROR( NL_INP_ERR );

        if( NOT N_CrvIs3d( cur[i] ) )
            N_Crv2dTo3d( cur[i] );

        if( der NEQ NULL AND der[i]NEQ NULL )
        {
            if( N_IsCrvRat( der[i] ) )
                NL_ERROR( NL_INP_ERR );
        }
    }

    /* Check compatibility */

    curC = N_AllocArrayCrvPtrs( 1, &SL );

    if( curC EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        if( der NEQ NULL AND der[i]NEQ NULL )
        {
            curC[0] = cur[i];
            curC[1] = der[i];

            if( NOT N_CrvsAreCombatible( curC, 1 ) )
            {
                error = N_CrvsMakeCompatibleAdjKnots( curC, 1, NL_PTOL, SI );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    /* Check direction of parametrization */

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        N_CrvGetKnots( cur[i], &ml, &UL );
        N_CrvGetKnots( cur[b], &mr, &UR );

        error = N_CrvEval( cur[i], UL[ml], NL_LEFT, &ML );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( cur[b], UR[0], NL_LEFT, &MR );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( ML, MR, &A );
        N_VectorMagnitude( A, &mag );

        if( mag GT NL_MTOL )
            NL_ERROR( NL_INP_ERR );
    }

    /* Split boundary curves at their mid points */

    curL = N_AllocArrayCrvPtrs( k, &SL );

    if( curL EQ NULL )
        NL_QUIT;

    curR = N_AllocArrayCrvPtrs( k, &SL );

    if( curR EQ NULL )
        NL_QUIT;

    M = N_AllocPt1dArray( k, &SL );

    if( M EQ NULL )
        NL_QUIT;

    ph = 0;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetDegree( cur[i], &p );
        N_CrvGetKnots( cur[i], &m, &U );

        if( p GT ph )
            ph = p;

        /* Get mid point */

        um = 0.5 *( U[0] + U[m] );

        error = N_CrvEval( cur[i], um, NL_LEFT, &M[i] );

        if( error EQ NL_YES )
            NL_OUT;

        /* Split curve */

        curL[i] = N_AllocCrv( &SL );

        if( curL[i]EQ NULL )
            NL_QUIT;

        curR[i] = N_AllocCrv( &SL );

        if( curR[i]EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( curL[i] );
        N_CrvInitArrays( curR[i] );
        error = N_CrvSplit( cur[i], um, curL[i], curR[i], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvReverse( curR[i], curR[i], &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check degrees */

    ph = NL_MAX( 3, ph );

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetDegree( curL[i], &pl );
        N_CrvGetDegree( curR[i], &pr );

        if( pl LT ph )
        {
            error = N_CrvElevateDegree( curL[i], ph - pl, curL[i], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( pr LT ph )
        {
            error = N_CrvElevateDegree( curR[i], ph - pr, curR[i], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Split derivative curves at their mid points */

    derL = N_AllocArrayCrvPtrs( k, &SL );

    if( derL EQ NULL )
        NL_QUIT;

    derR = N_AllocArrayCrvPtrs( k, &SL );

    if( derR EQ NULL )
        NL_QUIT;

    D = N_AllocPt1dArray( k, &SL );

    if( D EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        /* Derivative passed in */

        if( der NEQ NULL AND der[i]NEQ NULL )
        {
            N_CrvGetKnots( der[i], &m, &U );

            /* Get mid derivative */

            um = 0.5 *( U[0] + U[m] );

            error = N_CrvEval( der[i], um, NL_LEFT, &D[i] );

            if( error EQ NL_YES )
                NL_OUT;

            /* Split derivative curve */

            derL[i] = N_AllocCrv( &SL );

            if( derL[i]EQ NULL )
                NL_QUIT;

            derR[i] = N_AllocCrv( &SL );

            if( derR[i]EQ NULL )
                NL_QUIT;

            N_CrvInitArrays( derL[i] );
            N_CrvInitArrays( derR[i] );
            error = N_CrvSplit( der[i], um, derL[i], derR[i], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvReverse( derR[i], derR[i], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            /* Compute derivative */

            derA = N_AllocCrvAndArrays( 1, 1, 3, &SL );

            if( derA EQ NULL )
                NL_QUIT;

            if( i EQ 0 )
                a = k;
            else
                a = i - 1;

            if( i EQ k )
                b = 0;
            else
                b = i + 1;

            N_CrvGetKnots( curR[a], &mr, &UR );
            N_CrvGetKnots( curL[b], &ml, &UL );
            N_CrvGetCPtsAndKnots( derA, &Pw, &U );

            error = N_CrvDerivs( curR[a], UR[0], NL_LEFT, 1, CD1 );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDerivs( curL[b], UL[0], NL_LEFT, 1, CD2 );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtToCPt( CD1[1], &Pw[0] );
            N_PtToCPt( CD2[1], &Pw[1] );

            U[0] = U[1] = 0.0;
            U[2] = U[3] = 1.0;

            N_CrvGetDegree( cur[i], &p );
            N_CrvGetArraySizes( cur[i], &n, &m );
            N_CrvGetKnots( cur[i], &m, &U );

            if( p GT 1 )
            {
                error = N_CrvElevateDegree( derA, p - 1, derA, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            mx = n - p - 1;

            if( mx GT 0 )
            {
                knx = N_AllocKnotVectorAndArray( mx, &SL );

                if( knx EQ NULL )
                    NL_QUIT;

                N_KnotVectorGetKnots( knx, &mx, &UX );

                for ( j = 0; j <= mx; j++ )
                    UX[j] = U[j + p + 1];

                error = N_CrvRefine( derA, knx, derA, &SL, &SL );
            /* if( error EQ NL_YES )  NL_OUT;  */
            }

            N_CrvGetKnots( derA, &m, &U );

            /* Get mid derivative */

            um = 0.5 *( U[0] + U[m] );

            error = N_CrvEval( derA, um, NL_LEFT, &D[i] );

            if( error EQ NL_YES )
                NL_OUT;

            /* Split derivative curve */

            derL[i] = N_AllocCrv( &SL );

            if( derL[i]EQ NULL )
                NL_QUIT;

            derR[i] = N_AllocCrv( &SL );

            if( derR[i]EQ NULL )
                NL_QUIT;

            N_CrvInitArrays( derL[i] );
            N_CrvInitArrays( derR[i] );
            error = N_CrvSplit( derA, um, derL[i], derR[i], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvReverse( derR[i], derR[i], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    for ( i = 0; i <= k; i++ )
    {
        /* Check degrees */

        N_CrvGetDegree( derL[i], &pl );
        N_CrvGetDegree( derR[i], &pr );

        if( pl LT ph )
        {
            error = N_CrvElevateDegree( derL[i], ph - pl, derL[i], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( pr LT ph )
        {
            error = N_CrvElevateDegree( derR[i], ph - pr, derR[i], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* Check direction of derivatives */

        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        N_CrvGetKnots( curR[a], &mr, &UR );
        N_CrvGetKnots( curL[b], &ml, &UL );

        error = N_CrvDerivs( curR[a], UR[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curL[b], UL[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnots( derL[i], &ml, &UL );
        N_CrvGetKnots( derR[i], &mr, &UR );

        error = N_CrvEval( derL[i], UL[0], NL_LEFT, &DL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( derR[i], UR[0], NL_LEFT, &DR );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDot( CD1[1], DL, &dl );
        N_VectorDot( CD2[1], DR, &dr );

        if( dl LT 0.0 )
        {
            N_CrvGetCPts( derL[i], &n, &Pw );

            for ( j = 0; j <= n; j++ )
                N_ScaleCPt( -1.0, Pw[j], &Pw[j] );
        }

        if( dr LT 0.0 )
        {
            N_CrvGetCPts( derR[i], &n, &Pw );

            for ( j = 0; j <= n; j++ )
                N_ScaleCPt( -1.0, Pw[j], &Pw[j] );
        }

        if( dl LT 0.0 AND dr LT 0.0 )
            N_VectorReverse( D[i], &D[i] );
    }

    /* Scale magnitude of middle derivative */

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        N_CrvGetKnots( cur[a], &ml, &UL );
        N_CrvGetKnots( cur[b], &mr, &UR );

        error = N_CrvArcLength( cur[a], UL[0], UL[ml], lto, NL_RELATIVE, &dl );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvArcLength( cur[b], UR[0], UR[mr], lto, NL_RELATIVE, &dr );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorMagnitude( D[i], &mag );

        la = 0.25 *( dl + dr );
        fac = la / mag;

        N_VectorCreate( fac, fac, fac, &F );
        N_VectorScale( D[i], fac, &D[i] );
    }

    /* Get center point and normal vector */

    if( flg EQ NL_NO )
    {
        Q = N_AllocPt1dArray( k, &SL );

        if( Q EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= k; i++ )
            N_VectorPtAlongVector( M[i], 0.5, D[i], &Q[i] );

        N_CopyPt( NL_ZERO, &CC );

        for ( i = 0; i <= k; i++ )
            N_Sum2Pts( CC, Q[i], &CC );

        fac = 1.0 / (k + 1.0);
        N_ScalePt( fac, CC, &CC );

        N_CopyPt( NL_ZERO, &NN );

        for ( i = 0; i <= k; i++ )
        {
            if( i EQ k )
                b = 0;
            else
                b = i + 1;

            N_VectorDiff( M[i], CC, &A );
            N_VectorDiff( M[b], CC, &B );
            N_VectorCross( A, B, &F );
            N_Sum2Pts( NN, F, &NN );
        }

        fac = 1.0 / (k + 1.0);
        N_ScalePt( fac, NN, &NN );
    }
    else
    {
        N_VectorCopy( *CI, &CC );
        N_VectorCopy( *NI, &NN );
    }

    error = N_VectorNormalizeRef( &NN );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreatePlanePtNormal( &pln,  /* out: Plane */
                            CC,   /* in : Point lying on the plane */
                            NN ); /* in : Normal vector */

    /* Get inner boundary curves */

    curI = N_AllocArrayCrvPtrs( k, &SL );

    if( curI EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        curI[i] = N_AllocCrvAndArrays( 3, 3, 7, &SL );

        if( curI[i]EQ NULL )
            NL_QUIT;

        N_CrvGetCPtsAndKnots( curI[i], &Pw, &U );

        N_PtToCPt( M[i], &Pw[0] );
        N_PtToCPt( CC, &Pw[3] );

        N_VectorCopy( D[i], &T0 );

        error = N_VectorNormalizeRef( &T0 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_ProjectPtPlane( pln, M[i], &A );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( CC, A, &T3 );

        error = N_VectorNormalizeRef( &T3 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorSum( T0, T3, &A );
        N_VectorDiff( CC, M[i], &B );
        N_VectorMagnitude( A, &mag );
        N_VectorMagnitude( B, &del );
        N_VectorDot( A, B, &dl );

        alf = 16.0 - mag * mag;
        bet = 12.0 *dl;
        gam = -36.0 *del * del;

        error = N_SolveQuadraticEq( alf, bet, gam, &mag, &fac, &n );

        if( error EQ NL_YES )
            NL_OUT;

        if( n NEQ 2 )
            NL_ERROR( NL_GEO_ERR );

        fac = fac / 3.0;

        N_VectorPtAlongVector( M[i], fac, T0, &A );
        N_VectorPtAlongVector( CC, -fac, T3, &B );

        N_PtToCPt( A, &Pw[1] );
        N_PtToCPt( B, &Pw[2] );

        U[0] = U[1] = U[2] = U[3] = 0.0;
        U[4] = U[5] = U[6] = U[7] = 1.0;
    }

    /* Get unit normals at the mid points */

    N = N_AllocPt1dArray( k, &SL );

    if( N EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetKnots( curL[i], &ml, &UL );
        N_CrvGetKnots( curI[i], &mi, &UI );

        error = N_CrvDerivs( curL[i], UL[ml], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[i], UI[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCross( CD1[1], CD2[1], &N[i] );

        error = N_VectorNormalizeRef( &N[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get cross-boundary derivatives across the inner curves */

    bdrL = N_AllocArrayCrvPtrs( k, &SL );

    if( bdrL EQ NULL )
        NL_QUIT;

    bdrR = N_AllocArrayCrvPtrs( k, &SL );

    if( bdrR EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        bdrL[i] = N_AllocCrv( &SL );

        if( bdrL[i]EQ NULL )
            NL_QUIT;

        bdrR[i] = N_AllocCrv( &SL );

        if( bdrR[i]EQ NULL )
            NL_QUIT;

        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        N_CrvGetKnots( curR[i], &mr, &UR );
        N_CrvGetKnots( curI[b], &mi, &UI );

        error = N_CrvDerivs( curR[i], UR[mr], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[b], UI[mi], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CreateCrossBoundaryDerivCrv( curI[i], N[i], NN, CD1[1], CD2[1], bdrL[i], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnots( curL[i], &ml, &UL );
        N_CrvGetKnots( curI[a], &mi, &UI );

        error = N_CrvDerivs( curL[i], UL[ml], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[a], UI[mi], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CreateCrossBoundaryDerivCrv( curI[i], N[i], NN, CD1[1], CD2[1], bdrR[i], &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make boundary and cross-derivatives compatible */

    curC = N_AllocArrayCrvPtrs( 6, &SL );

    if( curC EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        curC[0] = curR[a];
        curC[1] = curI[i];
        curC[2] = curL[b];

        curC[3] = derR[a];
        curC[4] = bdrL[i];
        curC[5] = bdrR[i];
        curC[6] = derL[b];

        if( NOT N_CrvsAreCombatible( curC, 6 ) )
        {
            error = N_CrvsMakeCompatibleAdjKnots( curC, 6, NL_PTOL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            curR[a] = curC[0];
            curI[i] = curC[1];
            curL[b] = curC[2];

            derR[a] = curC[3];
            bdrL[i] = curC[4];
            bdrR[i] = curC[5];
            derL[b] = curC[6];
        }
    }

    /* Adjust derivative magnitudes */

    cfn = N_AllocCrvFunc( 1, 1, 3, &SL );

    if( cfn EQ NULL )
        NL_QUIT;

    N_CrvFuncCntrlValKnots( cfn, &fu, &UF );

    UF[0] = UF[1] = 0.0;
    UF[2] = UF[3] = 1.0;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        N_CrvGetKnots( curL[i], &r, &U );
        N_CrvGetKnots( curR[a], &s, &V );

        error = N_CrvEval( derR[a], V[0], NL_LEFT, &DL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( derR[a], V[s], NL_LEFT, &DR );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curL[i], U[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[a], U[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorMagnitude( DL, &dl );
        N_VectorMagnitude( DR, &dr );
        N_VectorMagnitude( CD1[1], &alf );
        N_VectorMagnitude( CD2[1], &bet );

        fu[0] = alf / dl;
        fu[1] = bet / dr;

        curA = N_AllocCrv( &SL );

        if( curA EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( curA );
        error = N_CrvFuncMultiplyCrv( cfn, derR[a], curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_FreeCrv( derR[a], &SL );
        derR[a] = curA;

        error = N_CrvEval( derL[i], U[0], NL_LEFT, &DL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( derL[i], U[r], NL_LEFT, &DR );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curR[a], V[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[i], V[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorMagnitude( DL, &dl );
        N_VectorMagnitude( DR, &dr );
        N_VectorMagnitude( CD1[1], &alf );
        N_VectorMagnitude( CD2[1], &bet );

        fu[0] = alf / dl;
        fu[1] = bet / dr;

        curA = N_AllocCrv( &SL );

        if( curA EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( curA );
        error = N_CrvFuncMultiplyCrv( cfn, derL[i], curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_FreeCrv( derL[i], &SL );
        derL[i] = curA;

        error = N_CrvEval( bdrR[i], V[0], NL_LEFT, &DL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( bdrR[i], V[s], NL_LEFT, &DR );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curL[i], U[r], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[a], U[r], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorMagnitude( DL, &dl );
        N_VectorMagnitude( DR, &dr );
        N_VectorMagnitude( CD1[1], &alf );
        N_VectorMagnitude( CD2[1], &bet );

        fu[0] = alf / dl;
        fu[1] = bet / dr;

        curA = N_AllocCrv( &SL );

        if( curA EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( curA );
        error = N_CrvFuncMultiplyCrv( cfn, bdrR[i], curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_FreeCrv( bdrR[i], &SL );
        bdrR[i] = curA;

        error = N_CrvEval( bdrL[a], U[0], NL_LEFT, &DL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( bdrL[a], U[r], NL_LEFT, &DR );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curR[a], V[s], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( curI[i], V[s], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorMagnitude( DL, &dl );
        N_VectorMagnitude( DR, &dr );
        N_VectorMagnitude( CD1[1], &alf );
        N_VectorMagnitude( CD2[1], &bet );

        fu[0] = alf / dl;
        fu[1] = bet / dr;

        curA = N_AllocCrv( &SL );

        if( curA EQ NULL )
            NL_QUIT;

        N_CrvInitArrays( curA );
        error = N_CrvFuncMultiplyCrv( cfn, bdrL[a], curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_FreeCrv( bdrL[a], &SL );
        bdrL[a] = curA;
    }

    /* Approximate cross-boundary to ensure twist compatibility */

    for ( i = 0; i <= k; i++ )
    {
        /* Get average of corner derivatives */

        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        N_CrvGetKnots( derR[a], &s, &V );
        N_CrvGetKnots( derL[i], &r, &U );

        error = N_CrvDerivs( derR[a], V[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derL[i], U[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( CD1[1], CD2[1], &A );
        N_VectorMagnitude( A, &mag );

        if( mag GT NL_MTOL )
            le = bo = NL_NO;
        N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &D1 );

        error = N_CrvDerivs( derL[i], U[r], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( bdrR[i], V[0], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( CD1[1], CD2[1], &A );
        N_VectorMagnitude( A, &mag );

        if( mag GT NL_MTOL )
            bo = ri = NL_NO;
        N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &D2 );

        error = N_CrvDerivs( bdrR[i], V[s], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( bdrL[a], U[r], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( CD1[1], CD2[1], &A );
        N_VectorMagnitude( A, &mag );

        if( mag GT NL_MTOL )
            ri = to = NL_NO;
        N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &D3 );

        error = N_CrvDerivs( bdrL[a], U[0], NL_LEFT, 1, CD1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derR[a], V[s], NL_LEFT, 1, CD2 );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( CD1[1], CD2[1], &A );
        N_VectorMagnitude( A, &mag );

        if( mag GT NL_MTOL )
            to = le = NL_NO;
        N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &D4 );

        /* Now approximate cross-derivatives */

        if( bo EQ NL_NO )
        {
            error = N_ApproxCrossBoundaryDerivs( derL[i], D1, D2, eps, derL[i], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( ri EQ NL_NO )
        {
            error = N_ApproxCrossBoundaryDerivs( bdrR[i], D2, D3, 0.5 *eps, bdrR[i], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( to EQ NL_NO )
        {
            error = N_ApproxCrossBoundaryDerivs( bdrL[a], D4, D3, 0.5 *eps, bdrL[a], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( le EQ NL_NO )
        {
            error = N_ApproxCrossBoundaryDerivs( derR[a], D1, D4, eps, derR[a], &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Make boundary and approximated cross-derivatives compatible */

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        curC[0] = curR[a];
        curC[1] = curI[i];
        curC[2] = curL[b];

        curC[3] = derR[a];
        curC[4] = bdrL[i];
        curC[5] = bdrR[i];
        curC[6] = derL[b];

        if( NOT N_CrvsAreCombatible( curC, 6 ) )
        {
            error = N_CrvsMakeCompatibleAdjKnots( curC, 6, NL_PTOL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            curR[a] = curC[0];
            curI[i] = curC[1];
            curL[b] = curC[2];

            derR[a] = curC[3];
            bdrL[i] = curC[4];
            bdrR[i] = curC[5];
            derL[b] = curC[6];
        }
    }

    /* Fit bicubically blended Coons patches into each rectangular hole */

    surA = N_AllocArraySrfPtrs( k, SS );

    if( surA EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            a = k;
        else
            a = i - 1;

        surA[i] = N_AllocSrf( SS );

        if( surA[i]EQ NULL )
            NL_QUIT;

        N_SrfInitArrays( surA[i] );
        error = N_CreateCoonsBoundaryCrvs( curR[a], derR[a], curI[i], bdrR[i], curL[i], derL[i], curI[a], bdrL[a], NL_NO, surA[i], &SL, SS ); /* NL_YES changed to NL_NO */

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Merge knots along common boundaries */

    kna = N_Alloc1dArrayKnotVectPtrs( 1, &SL );

    if( kna EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ k )
            b = 0;
        else
            b = i + 1;

        N_SrfGetKnotVectors( surA[i], &knu, &kna[0] );
        N_SrfGetKnotVectors( surA[b], &kna[1], &knv );

        error = N_GetCompatibleKnotArray( kna, 1, &knm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( knm[0], &ml, &UL );
        N_KnotVectorGetKnots( knm[1], &mr, &UR );

        if( ml GE 0 )
        {
            error = N_SrfInsertKnots( surA[i], knm[0], NL_VDIR, surA[i], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( mr GE 0 )
        {
            error = N_SrfInsertKnots( surA[b], knm[1], NL_UDIR, surA[b], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    *sur = surA;

    N_VectorCopy( CC, CI );
    N_VectorCopy( NN, NI );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_FillNSidedHole */

#ifdef NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine  computes the four corn-
     er twist vectors required for the bicubic Coons surface (N_CreateCoonsSrfTwist).
     It requires as input the  four  boundary curves,  and additionally,
     four interior surface points that are to be interpolated by the bi-
     cubic Coons surface.  The four boundary curves must be nonrational.
     A typical calling example is:

       NL_CURVE    **curU, **curV;
       NL_POINT    *P;
       NL_REAL     *u, *v;
       NL_VECTOR   **T;
       ...
       (get boundary curves and 4 points together with their (u,v));
       ...
       N_Get4CornerTwistVectors(curU,curV,P,u,v,T);


   ACCESS:
   
     curU  , input  ,  U-boundaries (at v=vmin and v=vmax).  curU[0] and
                       curU[1] need not  be  compatible,  however,  they 
                       must be defined on the same parameter range
     curV  , input  ,  V-boundaries (at u=umin and u=umax).  curV[0] and
                       curV[1] need not  be  compatible,  however,  they 
                       must be defined on the same parameter range
     P     , input  ,  Four interior points to be  interpolated  by  the
                       bicubically blended Coons surface
     u,v   , input  ,  u[i],v[i] are the parameters corresponding to the
                       point P[i]
     T     , output ,  Corner  twists  (T[0][0],  T[1][0],  T[0][1]  and 
                       T[1][1])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Get4CornerTwistVectors( NL_CURVE ** curU, NL_CURVE ** curV, NL_POINT *P, NL_REAL *u, NL_REAL *v, NL_VECTOR ** T )
{
    NL_PRIVATE NL_STRING rname = _T("N_Get4CornerTwistVectors");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, ind[4];

    NL_REAL d1, d2, us, ue, vs, ve, h1s, h2s, h3s, h4s, h1t, h2t, h3t, h4t, du, dv, s, t, ** RM;

    NL_POINT P00, P01, P10, P11, A1, A2, B1, B2, rhs[4], tmp;

    NL_CPOINT sol[4];

    NL_RMATRIX rma;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors in input */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( curU[0], &us, &ue );
    N_CrvGetParamBounds( curU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( curV[0], &vs, &ve );
    N_CrvGetParamBounds( curV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    du = ue - us;
    dv = ve - vs;

    /* Get surface corner points */

    error = N_CrvEval( curU[0], us, NL_LEFT, &A1 );
    error = N_CrvEval( curV[0], vs, NL_LEFT, &A2 );
    N_Combine2Pts( 0.5, A1, 0.5, A2, &P00 );

    error = N_CrvEval( curU[0], ue, NL_LEFT, &A1 );
    error = N_CrvEval( curV[1], vs, NL_LEFT, &A2 );
    N_Combine2Pts( 0.5, A1, 0.5, A2, &P10 );

    error = N_CrvEval( curU[1], us, NL_LEFT, &A1 );
    error = N_CrvEval( curV[0], ve, NL_LEFT, &A2 );
    N_Combine2Pts( 0.5, A1, 0.5, A2, &P01 );

    error = N_CrvEval( curU[1], ue, NL_LEFT, &A1 );
    error = N_CrvEval( curV[1], ve, NL_LEFT, &A2 );
    N_Combine2Pts( 0.5, A1, 0.5, A2, &P11 );

    /* Allocate memory for the system of equations */

    error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixData( &rma, &ii, &jj, &RM, &mtp, &kk );

    /* Now loop and set up four equations */

    for ( ii = 0; ii <= 3; ii++ )
    {
        /* evaluate points on boundaries */

        error = N_CrvEval( curU[0], u[ii], NL_LEFT, &A1 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( curU[1], u[ii], NL_LEFT, &A2 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( curV[0], v[ii], NL_LEFT, &B1 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( curV[1], v[ii], NL_LEFT, &B2 );

        if( error EQ NL_YES )
            NL_OUT;

        /* transform u,v to [0,1] interval */

        s = (u[ii] - us) / du;
        t = (v[ii] - vs) / dv;

        /* evaluate 4 Hermite functions for both s and t */

        d1 = s * s;
        d2 = s * d1;

        h1s = 1.0 - 3.0 *d1 + 2.0 *d2;
        h2s = 3.0 *d1 - 2.0 *d2;
        h3s = s - 2.0 *d1 + d2;
        h4s = -d1 + d2;

        d1 = t * t;
        d2 = t * d1;

        h1t = 1.0 - 3.0 *d1 + 2.0 *d2;
        h2t = 3.0 *d1 - 2.0 *d2;
        h3t = t - 2.0 *d1 + d2;
        h4t = -d1 + d2;

        /* now load the ii-th equation */

        RM[ii][0] = h3s * h3t;
        RM[ii][1] = h3s * h4t;
        RM[ii][2] = h4s * h3t;
        RM[ii][3] = h4s * h4t;

        rhs[ii] = P[ii];

        N_Combine2Pts( h1t, P00, h2t, P01, &tmp );
        N_VectorBlendPt( h1s, tmp, &rhs[ii] );

        N_Combine2Pts( h1t, P10, h2t, P11, &tmp );
        N_VectorBlendPt( h2s, tmp, &rhs[ii] );

        N_VectorBlendPt( -h1t, A1, &rhs[ii] );
        N_VectorBlendPt( -h2t, A2, &rhs[ii] );
        N_VectorBlendPt( -h1s, B1, &rhs[ii] );
        N_VectorBlendPt( -h2s, B2, &rhs[ii] );
    }

    /* Now solve the system of equations */

    error = N_RealMatrixLuDecomposePivot( &rma, ind );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixRightForBackPivot( &rma, ind, rhs, sol );

    if( error EQ NL_YES )
        NL_OUT;

    /* Project the cpoints and scale for correct parameter intervals */

    d1 = 1.0 / (du * dv);

    N_CPtToPtEuclid( sol[0], &T[0][0] );
    N_ScalePt( d1, T[0][0], &T[0][0] );

    N_CPtToPtEuclid( sol[1], &T[0][1] );
    N_ScalePt( d1, T[0][1], &T[0][1] );

    N_CPtToPtEuclid( sol[2], &T[1][0] );
    N_ScalePt( d1, T[1][0], &T[1][0] );

    N_CPtToPtEuclid( sol[3], &T[1][1] );
    N_ScalePt( d1, T[1][1], &T[1][1] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Get4CornerTwistVectors */

/*******************************************************************//**


   DESCRIPTION:

     This advanced  surface  construction  routine  computes a  cross-
     boundary  derivative  curve  given  the  derivative curve  of the 
     boundary,  a cross-tangent field,  and cross-boundary derivatives
     at at least two locations on the boundary  curve.  Both  tan  and 
     cder must be nonrational curves. A typical calling example is:

       NL_CURVE   cder, crder, tan;
       NL_VECTOR  *B;
       NL_REAL    *u;
       NL_INDEX   nn;
       NL_STACKS  SG;
       ...
       (compute cder and tan, allocate and load B, and u);
       ...
       N_CrvInitArrays(&crder);
       N_CrossBoundaryDerivCrv(&cder,&tan,B,u,nn,&crder,&SG);

     MEMORY TO HOLD  crder IS ALLOCATED INSIDE THE ROUTINE! crder MUST 
     BE DECLARED AS SHOWN ABOVE!  Note  that  cder  and  tan  must  be 
     defined on the same parameter range.


   ACCESS:
   
     cder  , input  ,  The derivative  curve  of  the  boundary  curve. 
                       cder must be nonrational
     tan   , input  ,  A cross-tangent vector  field.  For  every  par-
                       ameter in the domain of cder and tan,  the  vec-
                       tor defined by crder will lie in the  plane  de-
                       fined by cder and tan. tan must be nonrational
     B     , input  ,  Cross-boundary derivatives at the nn+1 locations
                       in array u
     u     , input  ,  Parameters  where  cross-derivative  constraints
                       are specified in array B
     nn    , input  ,  High index for arrays B and u
     crder , output ,  Cross-boundary derivative
     SG    , output ,  crder's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrossBoundaryDerivCrv( NL_CURVE *cder, NL_CURVE *tan, NL_VECTOR *B, NL_REAL *u, NL_INDEX nn, NL_CURVE *crder, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrossBoundaryDerivCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, mc, mt;

    NL_REAL s1, s2, t1, t2, *pp, *qq, ** RM, rhs[3], sol[2], *U1, *U2;

    NL_VECTOR T, D;

    NL_RMATRIX rma;

    NL_MATRIXTYPE mtp;

    NL_DEGREE deg;

    NL_CFUN cfnP, cfnQ;

    NL_CURVE pd, qt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error conditions */

    if( N_IsCrvRat( cder ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( tan ) )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( cder, &s1, &s2 );
    N_CrvGetParamBounds( tan, &t1, &t2 );

    if( s1 NEQ t1 OR s2 NEQ t2 )
        NL_ERROR( NL_INP_ERR );

    if( nn LT 1 )
        NL_ERROR( NL_INP_ERR );

    if( s1 NEQ u[0]OR s2 NEQ u[nn] )
        NL_ERROR( NL_INP_ERR );

    /*  Get 2D coordinates (pp[i],qq[i]) , i=0,...,nn  so that  */
    /*  B[i] = pp[i]*cder(u[i]) + qq[i]*tan(u[i])               */

    pp = N_AllocReal1dArray( 2 * nn + 1, &SL );

    if( pp EQ NULL )
        NL_QUIT;
    qq = &pp[nn + 1];

    N_InitRealMatrix( &rma );
    error = N_SetRealMatrix( &rma, 2, 1, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    N_GetRealMatrixData( &rma, &ii, &jj, &RM, &mtp, &jj );

    for ( ii = 0; ii <= nn; ii++ )
    {
        /* Get vectors D and T which define a plane */

        error = N_CrvEval( cder, u[ii], NL_LEFT, &D );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( tan, u[ii], NL_LEFT, &T );

        if( error EQ NL_YES )
            NL_OUT;

        /* Solve for 2D coordinates of B[ii] in plane (D,T) */

        N_PtToXYZ( D, &RM[0][0], &RM[1][0], &RM[2][0] );
        N_PtToXYZ( T, &RM[0][1], &RM[1][1], &RM[2][1] );
        N_PtToXYZ( B[ii], &rhs[0], &rhs[1], &rhs[2] );

        error = N_RealMatrixLstSqSolve( &rma,   /* in : NL_RMATRIX *A of Ax=b, with m+1 rows and n+1 columns (m>n) */
                                         rhs,   /* in : NL_REAL    *b of Ax=b, sized:[m+1] */
                                         sol ); /* out: NL_REAL    *x of Ax=b, sized:[n+1] */

        if( error EQ NL_YES )
            NL_OUT;

        pp[ii] = sol[0];
        qq[ii] = sol[1];
    }

    /* Get coordinate functions, pp(t) and qq(t) */

    if( nn EQ 1 )
        deg = 1;
    else
        deg = 2;

    N_CFuncInitArrays( &cfnP );
    error = N_FitFuncInterpGivenParams( pp, nn, deg, u, &cfnP, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &cfnQ );
    error = N_FitFuncInterpGivenParams( qq, nn, deg, u, &cfnQ, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*  Compute cross-boundary derivative curve:     */
    /*  crder(t) = cfnP(t)*cder(t) + cfnQ(t)*tan(t)  */

    N_CrvGetArraySizes( cder, &ii, &mc ); /* N_CrvFuncMultiplyCrv rescales knots. */
    N_CrvGetArraySizes( tan, &ii, &mt );  /* Must restore originals.  */
    U2 = N_AllocReal1dArray( NL_MAX( mc, mt ), &SL );

    if( U2 EQ NULL )
        NL_QUIT;

    N_CrvGetKnots( cder, &mc, &U1 ); /* Save knots */

    for ( ii = 0; ii <= mc; ii++ )
        U2[ii] = U1[ii];

    N_CrvInitArrays( &pd );
    error = N_CrvFuncMultiplyCrv( &cfnP, cder, &pd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( ii = 0; ii <= mc; ii++ )
        U1[ii] = U2[ii];            /* Restore knots */

    N_CrvGetKnots( tan, &mt, &U1 ); /* Save knots */

    for ( ii = 0; ii <= mt; ii++ )
        U2[ii] = U1[ii];

    N_CrvInitArrays( &qt );
    error = N_CrvFuncMultiplyCrv( &cfnQ, tan, &qt, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( ii = 0; ii <= mt; ii++ )
        U1[ii] = U2[ii]; /* Restore knots */

    N_CrvInitArrays( crder );
    error = N_CrvSumDiffCrv( &pd, &qt, NL_PLUS, crder, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrossBoundaryDerivCrv */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine  computes the four cross
     boundary derivative curves required for the bicubic  Coons  surface 
     (N_CreateCoonsBoundaryCrvs). It requires as input the four boundary curves, and add-
     itionally,  some number of interior surface points that are  to  be 
     approximated by the bicubic Coons surface. The four boundary curves 
     must be nonrational. A typical calling example is:

       NL_CURVE     **curU, **curV, **derU, **derV;
       NL_POINT     *P;
       NL_INDEX     np;
       NL_REAL      *u, *v;
       NL_DEGREE    p, q;
       NL_INDEX     nu, nv;
       NL_REAL      *U, *V;
       NL_STACKS    SG;
       ...
       (get boundary curves and np+1 points together with their (u,v));
       (choose p, q, nu and nv, and optionally U and V);
       ...
       derU = N_AllocArrayCrvPtrsAndData(1,NL_YES,&SG);   
       if ( derU EQ NULL )  handle error case
       derV = N_AllocArrayCrvPtrsAndData(1,NL_YES,&SG);   
       if ( derV EQ NULL )  handle error case
       ...
       N_Create4CrossBoundaryDerivCrvs(curU,curV,P,np,u,v,p,q,nu,nv,U,V,derU,derV,&SG);


   ACCESS:
   
     curU  , input  ,  U-boundaries (at v=vmin and v=vmax).  curU[0] and
                       curU[1] need not  be  compatible,  however,  they 
                       must be defined on the same parameter range
     curV  , input  ,  V-boundaries (at u=umin and u=umax).  curV[0] and
                       curV[1] need not  be  compatible,  however,  they 
                       must be defined on the same parameter range
     P     , input  ,  np+1 interior points to be  approximated  by  the
                       bicubically blended Coons surface
     np    , input  ,  High index of points in P and parameters in u,v
     u,v   , input  ,  u[i],v[i] are the parameters corresponding to the
                       point P[i]
     p,q   , input  ,  Degree of the  cross  derivatives in derU (p) and 
                       derV (q). p >= 2 and q >= 2 must hold
     nu,nv , input  ,  High index of control points for the cross deriv-
                       ative curves in derU (nu) and derV (nv). 
                       NOTE: all of the following must hold:
                       nu >= p  and  nv >= q  and  np > 2*(nu+nv). It is
                       recommended to make np much larger than this min-
                       imum
     U,V   , input  ,  Knots for the derivative curves derU (U) and derV
                       (V):
                       != NULL : these knots will be used; there must be
                                 nu+p+2 knots in U (nv+q+2 in V).  Start
                                 and end knots must agree with those  in
                                 curU[i] and curV[i]
                        = NULL : this routine will compute the knots
     derU  , output ,  derU[0] is the derivative curve across the bottom 
                       (curU[0]) boundary, derU[1] the derivative across
                       the top boundary (curU[1])
     derV  , output ,  Derivative curves across the curV boundaries
     SG    , input  ,  Stack for the curves created in derU and derV


     NOTE:  On input it is assumed that derU[i] and derV[i] are initial-
     ized to NULL objects.  Memory for the knots and control points will
     be allocated in this routine on SG


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Create4CrossBoundaryDerivCrvs( NL_CURVE ** curU, NL_CURVE ** curV, NL_POINT *P, NL_INDEX np, NL_REAL *u, NL_REAL *v, NL_DEGREE p, NL_DEGREE q, NL_INDEX nu, NL_INDEX nv, NL_REAL *U, NL_REAL *V, NL_CURVE ** derU, NL_CURVE ** derV, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_Create4CrossBoundaryDerivCrvs");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, mu, mv, nuk, uspn, vspn, k1, k2, j1, j2;

    NL_REAL d1, d2, us, ue, vs, ve, *hs, *ht, *ndu, *ndv, *ntu, *ntv, du, dv, s, t, ** RM, ** RN, *U1, *U2, *V1, *V2, dus, due, dvs, dve, d3, *uu, *vv, *UVI, *UVP, *UVT1, *UVT2, fac1, fac2, per, UT[8], VT[8], *WW, ** VV, ** PM;

    NL_POINT S00[2][2], S01[2][2], S10[2][2], S11[2][2], tmp1[2], tmp2[2], *rhs, *tmprhs;

    NL_CPOINT *PUw1, *PUw2, *PVw1, *PVw2, *sol;

    NL_DEGREE pq;

    NL_KNOTVECTOR kntu, kntv, kndu, kndv, *kni, *knp;

    NL_RMATRIX rma, rmt, rmb;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors in input */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( p LT 2 OR q LT 2 )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( curU[0], &us, &ue );
    N_CrvGetParamBounds( curU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( curV[0], &vs, &ve );
    N_CrvGetParamBounds( curV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    du = ue - us;
    dv = ve - vs;

    /* Get surface corner points and partials */

    error = N_CrvDerivs( curU[0], us, NL_LEFT, 1, tmp1 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[0], vs, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &S00[0][0] );
    N_CopyPt( tmp1[1], &S00[1][0] );
    N_CopyPt( tmp2[1], &S00[0][1] );

    error = N_CrvDerivs( curU[0], ue, NL_LEFT, 1, tmp1 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[1], vs, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &S10[0][0] );
    N_CopyPt( tmp1[1], &S10[1][0] );
    N_CopyPt( tmp2[1], &S10[0][1] );

    error = N_CrvDerivs( curU[1], us, NL_LEFT, 1, tmp1 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[0], ve, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &S01[0][0] );
    N_CopyPt( tmp1[1], &S01[1][0] );
    N_CopyPt( tmp2[1], &S01[0][1] );

    error = N_CrvDerivs( curU[1], ue, NL_LEFT, 1, tmp1 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[1], ve, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &S11[0][0] );
    N_CopyPt( tmp1[1], &S11[1][0] );
    N_CopyPt( tmp2[1], &S11[0][1] );

    /* Allocate memory for derivative curves */

    mu = nu + p + 1;
    mv = nv + q + 1;

    for ( ii = 0; ii <= 1; ii++ )
    {
        error = N_AllocCrvArrays( derU[ii], nu, p, mu, SG );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_AllocCrvArrays( derV[ii], nv, q, mv, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute/load knots for derU and derV */

    per = 0.8;

    N_CrvGetKnots( derU[0], &ii, &U1 );
    N_CrvGetKnots( derU[1], &ii, &U2 );

    if( U NEQ NULL )
    {
        if( U[mu - 1]NEQ U[mu] )
            NL_ERROR( NL_INP_ERR );

        if( U[0]NEQ us OR U[mu]NEQ ue )
            NL_ERROR( NL_INP_ERR );

        for ( ii = 0; ii <= mu; ii++ )
        {
            if( ii LT mu )
                if( U[ii]GT U[ii + 1] )
                    NL_ERROR( NL_INP_ERR );

            U1[ii] = U[ii];
            U2[ii] = U[ii];
        }
    }
    else
    {
        /* first get sorted list of parameters */

        uu = N_AllocReal1dArray( np + 2, &SL );

        if( uu EQ NULL )
            NL_QUIT;

        uu[0] = us;
        uu[np + 2] = ue;

        for ( ii = 0; ii <= np; ii++ )
            uu[ii + 1] = u[ii];

        N_ShellSortReal( uu, np + 2 );

        kk = np + 2;

        for ( ii = 0; ii < kk; ii++ )
        {
            if( uu[ii]EQ ue )
            {
                kk = ii;
                break;
            }

            jj = 1;

            while( uu[ii]EQ uu[ii + jj] )
                jj += 1;

            if( jj GT 1 )
            {
                k1 = ii + 1;

                for ( k2 = ii + jj; k2 <= kk; k2++ )
                    uu[k1++] = uu[k2];
                kk -= (jj - 1);
            }
        }

        /* now get ideal knot vectors for degrees p and p-1 */

        kni = N_AllocKnotVectorAndArray( mu, &SL );

        if( kni EQ NULL )
            NL_QUIT;
        knp = N_AllocKnotVectorAndArray( mu - 1, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knp, &ii, &UVP );
        N_KnotVectorGetKnots( kni, &ii, &UVI );

        error = N_FitCalcKnotVectorCrvApprox( uu, kk, nu, p, kni ); /* for degree p */

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( uu, kk, nu, (NL_DEGREE)(p - 1), knp ); /* for degree p-1 */

        if( error EQ NL_YES )
            NL_OUT;

        /* now get U1 from ideals and candidate */

        N_CrvGetKnots( curU[0], &ii, &UVT1 ); /* candidate knots */
        N_CrvGetKnots( curU[1], &ii, &UVT2 );

        for ( ii = 0; ii <= p; ii++ )
        {
            U1[ii] = uu[0];
            U1[nu + ii + 1] = uu[kk];
        }

        N_CrvGetDegree( curU[0], &pq );
        j1 = pq + 1;
        N_CrvGetDegree( curU[1], &pq );
        j2 = pq + 1;

        for ( ii = p + 1; ii <= nu; ii++ )
        {
            d1 = (1.0 - per) * UVI[ii] + per * UVP[ii - 1];
            d2 = (1.0 - per) * UVI[ii] + per * UVP[ii];

            while( UVT1[j1]LE d1 )
                j1 += 1;

            while( UVT2[j2]LE d1 )
                j2 += 1;

            fac1 = d2 - d1;
            k1 = -1;
            fac2 = d2 - d1;
            k2 = -1;

            while( UVT1[j1]GT d1 AND UVT1[j1]LT d2 )
            {
                d3 = fabs( UVI[ii] - UVT1[j1] );

                if( d3 LT fac1 )
                {
                    fac1 = d3;
                    k1 = j1;
                }
                j1 += 1;
            }

            while( UVT2[j2]GT d1 AND UVT2[j2]LT d2 )
            {
                d3 = fabs( UVI[ii] - UVT2[j2] );

                if( d3 LT fac2 )
                {
                    fac2 = d3;
                    k2 = j2;
                }
                j2 += 1;
            }

            if( k1 EQ - 1 AND k2 EQ - 1 )
                U1[ii] = UVI[ii];

            else if( fac1 LT fac2 )
                U1[ii] = UVT1[k1];

            else
                U1[ii] = UVT2[k2];
        }

        for ( ii = 0; ii <= mu; ii++ )
            U2[ii] = U1[ii];
    }

    N_CrvGetKnots( derV[0], &ii, &V1 );
    N_CrvGetKnots( derV[1], &ii, &V2 );

    if( V NEQ NULL )
    {
        if( V[mv - 1]NEQ V[mv] )
            NL_ERROR( NL_INP_ERR );

        if( V[0]NEQ vs OR V[mv]NEQ ve )
            NL_ERROR( NL_INP_ERR );

        for ( ii = 0; ii <= mv; ii++ )
        {
            if( ii LT mv )
                if( V[ii]GT V[ii + 1] )
                    NL_ERROR( NL_INP_ERR );

            V1[ii] = V[ii];
            V2[ii] = V[ii];
        }
    }
    else
    {
        /* first get sorted list of parameters */

        vv = N_AllocReal1dArray( np + 2, &SL );

        if( vv EQ NULL )
            NL_QUIT;

        vv[0] = vs;
        vv[np + 2] = ve;

        for ( ii = 0; ii <= np; ii++ )
            vv[ii + 1] = v[ii];

        N_ShellSortReal( vv, np + 2 );

        kk = np + 2;

        for ( ii = 0; ii < kk; ii++ )
        {
            if( vv[ii]EQ ve )
            {
                kk = ii;
                break;
            }

            jj = 1;

            while( vv[ii]EQ vv[ii + jj] )
                jj += 1;

            if( jj GT 1 )
            {
                k1 = ii + 1;

                for ( k2 = ii + jj; k2 <= kk; k2++ )
                    vv[k1++] = vv[k2];
                kk -= (jj - 1);
            }
        }

        /* now get ideal knot vectors for degrees q and q-1 */

        kni = N_AllocKnotVectorAndArray( mv, &SL );

        if( kni EQ NULL )
            NL_QUIT;
        knp = N_AllocKnotVectorAndArray( mv - 1, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knp, &ii, &UVP );
        N_KnotVectorGetKnots( kni, &ii, &UVI );

        error = N_FitCalcKnotVectorCrvApprox( vv, kk, nv, q, kni ); /* for degree q */

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( vv, kk, nv, (NL_DEGREE)(q - 1), knp ); /* for degree q-1 */

        if( error EQ NL_YES )
            NL_OUT;

        /* now get V1 from ideals and candidate */

        N_CrvGetKnots( curV[0], &ii, &UVT1 ); /* candidate knots */
        N_CrvGetKnots( curV[1], &ii, &UVT2 );

        for ( ii = 0; ii <= q; ii++ )
        {
            V1[ii] = vv[0];
            V1[nv + ii + 1] = vv[kk];
        }

        N_CrvGetDegree( curV[0], &pq );
        j1 = pq + 1;
        N_CrvGetDegree( curV[1], &pq );
        j2 = pq + 1;

        for ( ii = q + 1; ii <= nv; ii++ )
        {
            d1 = (1.0 - per) * UVI[ii] + per * UVP[ii - 1];
            d2 = (1.0 - per) * UVI[ii] + per * UVP[ii];

            while( UVT1[j1]LE d1 )
                j1 += 1;

            while( UVT2[j2]LE d1 )
                j2 += 1;

            fac1 = d2 - d1;
            k1 = -1;
            fac2 = d2 - d1;
            k2 = -1;

            while( UVT1[j1]GT d1 AND UVT1[j1]LT d2 )
            {
                d3 = fabs( UVI[ii] - UVT1[j1] );

                if( d3 LT fac1 )
                {
                    fac1 = d3;
                    k1 = j1;
                }
                j1 += 1;
            }

            while( UVT2[j2]GT d1 AND UVT2[j2]LT d2 )
            {
                d3 = fabs( UVI[ii] - UVT2[j2] );

                if( d3 LT fac2 )
                {
                    fac2 = d3;
                    k2 = j2;
                }
                j2 += 1;
            }

            if( k1 EQ - 1 AND k2 EQ - 1 )
                V1[ii] = UVI[ii];

            else if( fac1 LT fac2 )
                V1[ii] = UVT1[k1];

            else
                V1[ii] = UVT2[k2];
        }

        for ( ii = 0; ii <= mv; ii++ )
            V2[ii] = V1[ii];
    }

    dus = U1[p + 1] - U1[0];
    due = U1[mu] - U1[mu - p - 1];
    dvs = V1[q + 1] - V1[0];
    dve = V1[mv] - V1[mv - q - 1];

    /* Load the first/last control points of each derivative */

    N_CrvGetCPts( derU[0], &ii, &PVw1 );
    N_CrvGetCPts( derU[1], &ii, &PVw2 );
    N_CrvGetCPts( derV[0], &ii, &PUw1 );
    N_CrvGetCPts( derV[1], &ii, &PUw2 );

    N_PtToCPt( S00[1][0], &PUw1[0] );
    N_PtToCPt( S00[0][1], &PVw1[0] );
    N_PtToCPt( S10[1][0], &PUw2[0] );
    N_PtToCPt( S10[0][1], &PVw1[nu] );
    N_PtToCPt( S01[1][0], &PUw1[nv] );
    N_PtToCPt( S01[0][1], &PVw2[0] );
    N_PtToCPt( S11[1][0], &PUw2[nv] );
    N_PtToCPt( S11[0][1], &PVw2[nu] );

    /*  Allocate memory for the system of equations.    */
    /*                                                  */
    /*  The unknowns are: PUw1/PUw2 , 1,...,nv-1 and    */
    /*  PVw1/PVw2 , 1,...,nu-1                          */
    /*                                                  */
    /*  There are also 4 constraint equations, coupling */
    /*  the 4 twists to their corresponding 8 unknown   */
    /*  control points. We use Lagrange multipliers to  */
    /*  solve the system.                               */

    nuk = 2 * (nu + nv - 2) - 1; /* high index of unknowns */

    error = N_SetRealMatrix( &rma, np, nuk, NL_MT_FULL, np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixData( &rma, &ii, &jj, &RN, &mtp, &kk );

    sol = N_AllocCPt1dArray( nuk + 4, &SL );

    if( sol EQ NULL )
        NL_QUIT;

    rhs = N_AllocPt1dArray( nuk + np + 7, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    tmprhs = &rhs[nuk + 5];

    for ( ii = 0; ii <= np; ii++ )
        for ( jj = 0; jj <= nuk; jj++ )
            RN[ii][jj] = 0.0;

    /*  Allocate memory for the various basis functions  */

    ii = p + q + 18;

    hs = N_AllocReal1dArray( ii, &SL ); /* hs,ht Hermite basis functions   */

    if( hs EQ NULL )
        NL_QUIT;
    ht = &hs[4];

    ntu = &ht[4];      /* ntu,ntv b-spline functions for  */
    ntv = &ntu[4];     /* T-surface in Coons construction */

    ndu = &ntv[4];     /* b-spline functions for derU[i]  */
    ndv = &ndu[p + 1]; /* b-spline functions for derV[i]  */

    /* Set up knot vector objects for computing basis functions */

    for ( ii = 0; ii <= 3; ii++ )
    {
        UT[ii] = us;
        UT[ii + 4] = ue;
        VT[ii] = vs;
        VT[ii + 4] = ve;
    }

    N_KnotVectorFromRealArray( &kntu, UT, 7 );
    N_KnotVectorFromRealArray( &kntv, VT, 7 );

    N_KnotVectorFromRealArray( &kndu, U1, mu );
    N_KnotVectorFromRealArray( &kndv, V1, mv );

    /* Now loop and set up the np+1 equations; very tedious. */

    for ( ii = 0; ii <= np; ii++ )
    {
        /* first evaluate all basis functions */

        error = N_BasisEval( &kntu, 3, u[ii], NL_LEFT, ntu, &uspn );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisEval( &kntv, 3, v[ii], NL_LEFT, ntv, &vspn );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisEval( &kndu, p, u[ii], NL_LEFT, ndu, &uspn );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisEval( &kndv, q, v[ii], NL_LEFT, ndv, &vspn );

        if( error EQ NL_YES )
            NL_OUT;

        /* transform u,v to [0,1] and evaluate Hermites */

        s = (u[ii] - us) / du;
        t = (v[ii] - vs) / dv;

        d1 = s * s;
        d2 = s * d1;

        hs[0] = 1.0 - 3.0 *d1 + 2.0 *d2;
        hs[1] = 3.0 *d1 - 2.0 *d2;
        hs[2] = s - 2.0 *d1 + d2;
        hs[3] = -d1 + d2;

        d1 = t * t;
        d2 = t * d1;

        ht[0] = 1.0 - 3.0 *d1 + 2.0 *d2;
        ht[1] = 3.0 *d1 - 2.0 *d2;
        ht[2] = t - 2.0 *d1 + d2;
        ht[3] = -d1 + d2;

        /* Now load the ii-th equation. Unknown control points */
        /* appear in the order:                                */
        /*                                                     */
        /*    PVw1[1],...,PVw1[nu-1],PVw2[1],...,PVw2[nu-1],   */
        /*    PUw1[1],...,PUw1[nv-1],PUw2[1],...,PUw2[nv-1]    */

        N_CopyPt( P[ii], &tmprhs[ii] );

        error = N_CrvEval( curU[0], u[ii], NL_LEFT, &tmp1[0] );

        if( error EQ NL_YES )
            NL_OUT;
        N_VectorBlendPt( -ht[0], tmp1[0], &tmprhs[ii] );

        error = N_CrvEval( curU[1], u[ii], NL_LEFT, &tmp1[0] );

        if( error EQ NL_YES )
            NL_OUT;
        N_VectorBlendPt( -ht[1], tmp1[0], &tmprhs[ii] );

        error = N_CrvEval( curV[0], v[ii], NL_LEFT, &tmp1[0] );

        if( error EQ NL_YES )
            NL_OUT;
        N_VectorBlendPt( -hs[0], tmp1[0], &tmprhs[ii] );

        error = N_CrvEval( curV[1], v[ii], NL_LEFT, &tmp1[0] );

        if( error EQ NL_YES )
            NL_OUT;
        N_VectorBlendPt( -hs[1], tmp1[0], &tmprhs[ii] );

        d1 = dv * ht[2];
        d2 = dv * ht[3];

        k2 = nu - 2;

        for ( kk = uspn - p, jj = 0; jj <= p; kk++, jj++ )
        {
            if( kk EQ 0 )
            {
                N_VectorBlendPt( -d1 * ndu[0], S00[0][1], &tmprhs[ii] );
                N_VectorBlendPt( -d2 * ndu[0], S01[0][1], &tmprhs[ii] );
            }
            else if( kk EQ nu )
            {
                N_VectorBlendPt( -d1 * ndu[p], S10[0][1], &tmprhs[ii] );
                N_VectorBlendPt( -d2 * ndu[p], S11[0][1], &tmprhs[ii] );
            }
            else
            {
                RN[ii][kk - 1] += (d1 * ndu[jj]);
                RN[ii][kk + k2] += (d2 * ndu[jj]);
            }
        }

        d1 = du * hs[2];
        d2 = du * hs[3];

        k1 = 2 * nu - 3;
        k2 = k1 + nv - 1;

        for ( kk = vspn - q, jj = 0; jj <= q; kk++, jj++ )
        {
            if( kk EQ 0 )
            {
                N_VectorBlendPt( -d1 * ndv[0], S00[1][0], &tmprhs[ii] );
                N_VectorBlendPt( -d2 * ndv[0], S10[1][0], &tmprhs[ii] );
            }
            else if( kk EQ nv )
            {
                N_VectorBlendPt( -d1 * ndv[q], S01[1][0], &tmprhs[ii] );
                N_VectorBlendPt( -d2 * ndv[q], S11[1][0], &tmprhs[ii] );
            }
            else
            {
                RN[ii][kk + k1] += (d1 * ndv[jj]);
                RN[ii][kk + k2] += (d2 * ndv[jj]);
            }
        }

        N_VectorBlendPt( (ntu[0] * (ntv[0] + ntv[1]) + ntu[1] * ntv[0]), S00[0][0], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[0] * (ntv[2] + ntv[3]) + ntu[1] * ntv[3]), S01[0][0], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[3] * (ntv[0] + ntv[1]) + ntu[2] * ntv[0]), S10[0][0], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[3] * (ntv[2] + ntv[3]) + ntu[2] * ntv[3]), S11[0][0], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[0] * ntv[1] * dv) / 3, S00[0][1], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[1] * ntv[0] * du) / 3, S00[1][0], &tmprhs[ii] );
        N_VectorBlendPt( -(ntu[0] * ntv[2] * dv) / 3, S01[0][1], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[1] * ntv[3] * du) / 3, S01[1][0], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[3] * ntv[1] * dv) / 3, S10[0][1], &tmprhs[ii] );
        N_VectorBlendPt( -(ntu[2] * ntv[0] * du) / 3, S10[1][0], &tmprhs[ii] );
        N_VectorBlendPt( -(ntu[3] * ntv[2] * dv) / 3, S11[0][1], &tmprhs[ii] );
        N_VectorBlendPt( -(ntu[2] * ntv[3] * du) / 3, S11[1][0], &tmprhs[ii] );

        d1 = (du * dv) / 18.0;
        d2 = du / 3.0;
        d3 = dv / 3.0;

        RN[ii][0] += (-(d1 * p * ntu[1] * ntv[1]) / dus);
        RN[ii][2 * nu - 2] += (-(d1 * q * ntu[1] * ntv[1]) / dvs);
        N_VectorBlendPt( -(ntu[1] * ntv[1]) * (((d1 * p) / dus) - d3), S00[0][1], &tmprhs[ii] );
        N_VectorBlendPt( -(ntu[1] * ntv[1]) * (((d1 * q) / dvs) - d2), S00[1][0], &tmprhs[ii] );
        N_VectorBlendPt( ntu[1] * ntv[1], S00[0][0], &tmprhs[ii] );

        RN[ii][2 * nu + nv - 4] += (-(d1 * q * ntu[1] * ntv[2]) / dve);
        RN[ii][nu - 1] += ((d1 * p * ntu[1] * ntv[2]) / dus);
        N_VectorBlendPt( (ntu[1] * ntv[2]) * (((d1 * p) / dus) - d3), S01[0][1], &tmprhs[ii] );
        N_VectorBlendPt( -(ntu[1] * ntv[2]) * (((d1 * q) / dve) - d2), S01[1][0], &tmprhs[ii] );
        N_VectorBlendPt( ntu[1] * ntv[2], S01[0][0], &tmprhs[ii] );

        RN[ii][nu - 2] += (-(d1 * p * ntu[2] * ntv[1]) / due);
        RN[ii][2 * nu + nv - 3] += ((d1 * q * ntu[2] * ntv[1]) / dvs);
        N_VectorBlendPt( -(ntu[2] * ntv[1]) * (((d1 * p) / due) - d3), S10[0][1], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[2] * ntv[1]) * (((d1 * q) / dvs) - d2), S10[1][0], &tmprhs[ii] );
        N_VectorBlendPt( ntu[2] * ntv[1], S10[0][0], &tmprhs[ii] );

        RN[ii][2 * nu - 3] += ((d1 * p * ntu[2] * ntv[2]) / due);
        RN[ii][2 *( nu + nv ) - 5] += ((d1 *q *ntu[2]*ntv[2]) / dve);
        N_VectorBlendPt( (ntu[2] * ntv[2]) * (((d1 * p) / due) - d3), S11[0][1], &tmprhs[ii] );
        N_VectorBlendPt( (ntu[2] * ntv[2]) * (((d1 * q) / dve) - d2), S11[1][0], &tmprhs[ii] );
        N_VectorBlendPt( ntu[2] * ntv[2], S11[0][0], &tmprhs[ii] );
    }

    /* Now the 4 constraint equations */

    RM = N_AllocReal2dArray( 3, nuk, &SL );

    if( RM EQ NULL )
        NL_QUIT;

    for ( ii = 0; ii <= 3; ii++ )
        for ( jj = 0; jj <= nuk; jj++ )
            RM[ii][jj] = 0.0;

    d1 = (p * dvs) / (q * dus);
    RM[0][2 * nu - 2] = 1.0;
    RM[0][0] = -d1;
    N_CopyPt( S00[1][0], &rhs[nuk + 1] );
    N_VectorBlendPt( -d1, S00[0][1], &rhs[nuk + 1] );

    d1 = (p * dve) / (q * dus);
    RM[1][2 * nu + nv - 4] = 1.0;
    RM[1][nu - 1] = d1;
    N_CopyPt( S01[1][0], &rhs[nuk + 2] );
    N_VectorBlendPt( d1, S01[0][1], &rhs[nuk + 2] );

    d1 = (p * dvs) / (q * due);
    RM[2][2 * nu + nv - 3] = 1.0;
    RM[2][nu - 2] = d1;
    N_CopyPt( S10[1][0], &rhs[nuk + 3] );
    N_VectorBlendPt( d1, S10[0][1], &rhs[nuk + 3] );

    d1 = (p * dve) / (q * due);
    RM[3][2 *( nu + nv ) - 5] = 1.0;
    RM[3][2 * nu - 3] = -d1;
    N_CopyPt( S11[1][0], &rhs[nuk + 4] );
    N_VectorBlendPt( -d1, S11[0][1], &rhs[nuk + 4] );

    /* Get the transpose of the (np+1)x(nuk+1) coefficient matrix. */
    /* Then multiply it by rma and tmprhs.                         */

    N_InitRealMatrix( &rmt );
    error = N_RealMatrixTranspose( &rma, &rmt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &rmb );
    error = N_RealMatrixMultiply( &rmt, &rma, &rmb, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &rmt, tmprhs, rhs );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now set up the (nuk+5)x(nuk+5) square system to solve */

    PM = N_AllocReal2dArray( nuk + 4, nuk + 4, &SL );

    if( PM EQ NULL )
        NL_QUIT;

    N_GetRealMatrixData( &rmb, &ii, &jj, &RN, &mtp, &kk );

    for ( ii = 0; ii <= nuk; ii++ )
    {
        for ( jj = 0; jj <= 3; jj++ )
        {
            PM[nuk + jj + 1][ii] = RM[jj][ii];
            PM[ii][nuk + jj + 1] = RM[jj][ii];
        }

        for ( jj = 0; jj <= nuk; jj++ )
            PM[ii][jj] = RN[ii][jj];
    }

    for ( ii = 1; ii <= 4; ii++ )
        for ( jj = 1; jj <= 4; jj++ )
            PM[nuk + ii][nuk + jj] = 0.0;

    /* Now solve via Single Value Decomposition */

    nuk += 4;

    WW = N_AllocReal1dArray( nuk, &SL );
    VV = N_AllocReal2dArray( nuk, nuk, &SL );

    if( WW EQ NULL OR VV EQ NULL )
        NL_QUIT;

    error = N_RealMatrixSingleValueDecompose( PM, WW, VV, nuk, nuk );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SingleValueDecomposeSolve( PM, WW, VV, nuk, nuk, (NL_VOID *)rhs, NL_EPOINT, NL_YES, (NL_VOID *)sol );

    if( error EQ NL_YES )
        NL_OUT;

    /* Load the control points into the proper locations */

    for ( ii = 1; ii < nu; ii++ )
    {
        N_CopyCPt( sol[ii - 1], &PVw1[ii] );
        N_CopyCPt( sol[nu + ii - 2], &PVw2[ii] );
    }

    k1 = 2 * nu - 3;
    k2 = k1 + nv - 1;

    for ( ii = 1; ii < nv; ii++ )
    {
        N_CopyCPt( sol[k1 + ii], &PUw1[ii] );
        N_CopyCPt( sol[k2 + ii], &PUw2[ii] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Create4CrossBoundaryDerivCrvs */

#ifdef NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface  construction  routine  computes  the  tensor
     product surface required for the bicubic Coons surface construction
     via N_CreateCoonsSrfCrossBoundaryDerivs.  It requires as input the four boundary curves,  four
     cross-boundary derivatives, and additionally, some number of inter-
     ior surface points that are to be approximated by the bicubic Coons
     surface.  The four boundary curves and  cross-derivatives  must  be 
     nonrational. The output surface must be initialized to the NULL ob-
     ject. A typical calling example is:

       NL_CURVE     **curU, **curV, **derU, **derV;
       NL_SURFACE   surT;
       NL_POINT     *P;
       NL_INDEX     np;
       NL_REAL      *u, *v;
       NL_INDEX     nu, nv;
       NL_REAL      *U, *V;
       NL_STACKS    SG;
       ...
       (get boundary curves, cross-derivatives, and np+1 points 
       (together with their (u,v));
       (choose nu and nv, and optionally U and V);
       ...
       N_SrfInitArrays(&surT);
       ...
       N_CreateTensorProductSrf(curU,derU,curV,derV,P,np,u,v,nu,nv,U,V,&surT,&SG);


   ACCESS:
   
     curU  , input  ,  U-boundaries (at v=vmin and v=vmax) 
     derU  , input  ,  Derivative curves across curU[0] and curU[1]
     curV  , input  ,  V-boundaries (at u=umin and u=umax)
     derV  , input  ,  Derivative curves across curV[0] and curV[1]
     P     , input  ,  np+1 interior points to be  approximated  by  the
                       bicubically blended Coons surface
     np    , input  ,  High index of points in P and parameters in u,v.
                       np >= (nu+1)*(nv+1)-4*(nu+nv)+7  must hold  (this
                       is a minimum;  more  may  be required to obtain a 
                       solvable system of equations)
     u,v   , input  ,  u[i],v[i] are the parameters corresponding to the
                       point P[i].  
     nu,nv , input  ,  High index of  control points for the tensor pro-
                       duct surface, surT. nu > 3 and nv > 3  must hold.
                       For best results, nu and nv should be larger than
                       the number of control points in the corresponding
                       derU and derV
     U,V   , input  ,  Knots for the bicubic tensor product surface surT
                       != NULL : these knots will be used; there must be
                                 nu+5 knots in U and nv+5  in  V.  Start
                                 and end knots must agree with those  in
                                 curU[i] and curV[i],  and  they must be
                                 repeated with multiplicity 4
                        = NULL : this routine will compute the knots
     surT  , output ,  The bicubic tensor product surface  required  for
                       the Coons construction (via N_CreateCoonsSrfCrossBoundaryDerivs)
     SG    , input  ,  Stack for surT


     NOTE: curU[0], curU[1], derU[0], derU[1] need not  be  compatible,
     however,  they must be defined on the same parameter domain. Anal-
     ogously, curV[0], curV[1], derV[0], derV[1] must be  defined  on a
     common parameter domain. 

     Although not checked in this routine,  it  is  expected  that  all 
     boundary curves and cross-derivatives are compatible in the  Coons 
     sense, i.e. all corner derivatives and twists are compatible


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTensorProductSrf( NL_CURVE ** curU, NL_CURVE ** derU, NL_CURVE ** curV, NL_CURVE ** derV, NL_POINT *P, NL_INDEX np, NL_REAL *u, NL_REAL *v, NL_INDEX nu, NL_INDEX nv, NL_REAL *U, NL_REAL *V, NL_SURFACE *surT, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTensorProductSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, mu, mv, nuk, uspn, vspn, j1, j2, rs, ind[4], k1, k2 = 0;

    NL_REAL d1, d2, us, ue = 0.0, vs, ve, *hs, *ht, *ntu, *ntv, du, dv, s, t, ** RN, *X, d3, *uu, *vv, *UVI, *UVP, *UVT[4], fac, per, *UT, *VT, *WW, ** VV;

    NL_POINT P1, P2, P3, P4, tmp1[2], tmp2[2], tmp3[2], tmp4[2], *rhs;

    NL_VECTOR vec;

    NL_CPOINT *sol, ** Tw;

    NL_DEGREE pq;

    NL_KNOTVECTOR *kntu, *kntv, *kni, *knp, knx;

    NL_RMATRIX rma;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors in input */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( nu LE 3 OR nv LE 3 )
        NL_ERROR( NL_INP_ERR );

    mu = nu + 4;
    mv = nv + 4;

    if( np LT( nu + 1 ) * (nv + 1) - 4 * (nu + nv) + 7 )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( curU[0], &us, &ue );
    N_CrvGetParamBounds( curU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derU[0], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetParamBounds( curV[0], &vs, &ve );
    N_CrvGetParamBounds( curV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derV[0], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    du = ue - us;
    dv = ve - vs;

    /* Allocate the base tensor product surface */

    N_SrfInitArrays( surT );

    error = N_AllocSrfArrays( surT, 3, 3, 3, 3, 7, 7, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( surT, &Tw, &UT, &VT );

    /* Load initial knots (will be refined later) */

    for ( ii = 0; ii <= 3; ii++ )
    {
        UT[ii] = us;
        UT[ii + 4] = ue;
        VT[ii] = vs;
        VT[ii + 4] = ve;
    }

    /* Get 16 control points of the base tensor product surface */

    error = N_CrvDerivs( curU[0], us, NL_LEFT, 1, tmp1 ); /* bottom left */

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[0], vs, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derU[0], us, NL_LEFT, 1, tmp3 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derV[0], vs, NL_LEFT, 1, tmp4 );

    if( error EQ NL_YES )
        NL_OUT;

    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &P1 );
    N_PtToCPt( P1, &Tw[0][0] );

    N_Combine2Pts( 0.5, tmp1[1], 0.5, tmp4[0], &vec );
    N_Combine2Pts( du / 3.0, vec, 1.0, P1, &P2 );
    N_PtToCPt( P2, &Tw[1][0] );

    N_Combine2Pts( 0.5, tmp2[1], 0.5, tmp3[0], &vec );
    N_Combine2Pts( dv / 3.0, vec, 1.0, P1, &P3 );
    N_PtToCPt( P3, &Tw[0][1] );

    N_Combine2Pts( 0.5, tmp3[1], 0.5, tmp4[1], &vec );
    N_Combine4Pts( du * dv / 9.0, vec, 1.0, P3, 1.0, P2, -1.0, P1, &P4 );
    N_PtToCPt( P4, &Tw[1][1] );

    error = N_CrvDerivs( curU[0], ue, NL_LEFT, 1, tmp1 ); /* bottom right */

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[1], vs, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derU[0], ue, NL_LEFT, 1, tmp3 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derV[1], vs, NL_LEFT, 1, tmp4 );

    if( error EQ NL_YES )
        NL_OUT;

    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &P1 );
    N_PtToCPt( P1, &Tw[3][0] );

    N_Combine2Pts( 0.5, tmp1[1], 0.5, tmp4[0], &vec );
    N_Combine2Pts( -du / 3.0, vec, 1.0, P1, &P2 );
    N_PtToCPt( P2, &Tw[2][0] );

    N_Combine2Pts( 0.5, tmp2[1], 0.5, tmp3[0], &vec );
    N_Combine2Pts( dv / 3.0, vec, 1.0, P1, &P3 );
    N_PtToCPt( P3, &Tw[3][1] );

    N_Combine2Pts( 0.5, tmp3[1], 0.5, tmp4[1], &vec );
    N_Combine4Pts( -du * dv / 9.0, vec, 1.0, P3, 1.0, P2, -1.0, P1, &P4 );
    N_PtToCPt( P4, &Tw[2][1] );

    error = N_CrvDerivs( curU[1], us, NL_LEFT, 1, tmp1 ); /* top left */

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[0], ve, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derU[1], us, NL_LEFT, 1, tmp3 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derV[0], ve, NL_LEFT, 1, tmp4 );

    if( error EQ NL_YES )
        NL_OUT;

    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &P1 );
    N_PtToCPt( P1, &Tw[0][3] );

    N_Combine2Pts( 0.5, tmp1[1], 0.5, tmp4[0], &vec );
    N_Combine2Pts( du / 3.0, vec, 1.0, P1, &P2 );
    N_PtToCPt( P2, &Tw[1][3] );

    N_Combine2Pts( 0.5, tmp2[1], 0.5, tmp3[0], &vec );
    N_Combine2Pts( -dv / 3.0, vec, 1.0, P1, &P3 );
    N_PtToCPt( P3, &Tw[0][2] );

    N_Combine2Pts( 0.5, tmp3[1], 0.5, tmp4[1], &vec );
    N_Combine4Pts( -du * dv / 9.0, vec, 1.0, P3, 1.0, P2, -1.0, P1, &P4 );
    N_PtToCPt( P4, &Tw[1][2] );

    error = N_CrvDerivs( curU[1], ue, NL_LEFT, 1, tmp1 ); /* top right */

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( curV[1], ve, NL_LEFT, 1, tmp2 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derU[1], ue, NL_LEFT, 1, tmp3 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( derV[1], ve, NL_LEFT, 1, tmp4 );

    if( error EQ NL_YES )
        NL_OUT;

    N_Combine2Pts( 0.5, tmp1[0], 0.5, tmp2[0], &P1 );
    N_PtToCPt( P1, &Tw[3][3] );

    N_Combine2Pts( 0.5, tmp1[1], 0.5, tmp4[0], &vec );
    N_Combine2Pts( -du / 3.0, vec, 1.0, P1, &P2 );
    N_PtToCPt( P2, &Tw[2][3] );

    N_Combine2Pts( 0.5, tmp2[1], 0.5, tmp3[0], &vec );
    N_Combine2Pts( -dv / 3.0, vec, 1.0, P1, &P3 );
    N_PtToCPt( P3, &Tw[3][2] );

    N_Combine2Pts( 0.5, tmp3[1], 0.5, tmp4[1], &vec );
    N_Combine4Pts( du * dv / 9.0, vec, 1.0, P3, 1.0, P2, -1.0, P1, &P4 );
    N_PtToCPt( P4, &Tw[2][2] );

    /* Compute/load knots required for refinement */

    per = 0.8;

    ii = NL_MAX( mu - 8, mv - 8 );
    X = N_AllocReal1dArray( ii, &SL );

    if( X EQ NULL )
        NL_QUIT;

    if( U NEQ NULL )
    {
        if( U[mu - 3]NEQ U[mu] )
            NL_ERROR( NL_INP_ERR );

        if( U[0]NEQ U[3] )
            NL_ERROR( NL_INP_ERR );

        if( U[0]NEQ us OR U[mu]NEQ ue )
            NL_ERROR( NL_INP_ERR );

        for ( rs = -1, ii = 4; ii <= nu; ii++ )
        {
            if( U[ii]GT U[ii + 1] )
                NL_ERROR( NL_INP_ERR );

            X[++rs] = U[ii];
        }
    }
    else
    {
        /* first get sorted list of parameters */

        uu = N_AllocReal1dArray( np + 2, &SL );

        if( uu EQ NULL )
            NL_QUIT;

        uu[0] = us;
        uu[np + 2] = ue;

        for ( ii = 0; ii <= np; ii++ )
            uu[ii + 1] = u[ii];

        N_ShellSortReal( uu, np + 2 );

        kk = np + 2;

        for ( ii = 0; ii < kk; ii++ )
        {
            if( uu[ii]EQ ue )
            {
                kk = ii;
                break;
            }

            jj = 1;

            while( uu[ii]EQ uu[ii + jj] )
                jj += 1;

            if( jj GT 1 )
            {
                k1 = ii + 1;

                for ( k2 = ii + jj; k2 <= kk; k2++ )
                    uu[k1++] = uu[k2];
                kk -= (jj - 1);
            }
        }

        /* now get ideal knot vectors for degrees 3 and 2 */

        kni = N_AllocKnotVectorAndArray( mu, &SL );

        if( kni EQ NULL )
            NL_QUIT;
        knp = N_AllocKnotVectorAndArray( mu - 1, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knp, &ii, &UVP );
        N_KnotVectorGetKnots( kni, &ii, &UVI );

        error = N_FitCalcKnotVectorCrvApprox( uu, kk, nu, 3, kni ); /* for degree 3 */

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( uu, kk, nu, 2, knp ); /* for degree 2 */

        if( error EQ NL_YES )
            NL_OUT;

        /* now get X from ideals and candidate */

        N_CrvGetKnots( curU[0], &ii, &( UVT[0] ) ); /* candidate knots */
        N_CrvGetKnots( curU[1], &ii, &( UVT[1] ) );
        N_CrvGetKnots( derU[0], &ii, &( UVT[2] ) );
        N_CrvGetKnots( derU[1], &ii, &( UVT[3] ) );

        N_CrvGetDegree( curU[0], &pq );
        ind[0] = pq + 1;
        N_CrvGetDegree( curU[1], &pq );
        ind[1] = pq + 1;
        N_CrvGetDegree( derU[0], &pq );
        ind[2] = pq + 1;
        N_CrvGetDegree( derU[1], &pq );
        ind[3] = pq + 1;

        for ( rs = -1, ii = 4; ii <= nu; ii++ )
        {
            d1 = (1.0 - per) * UVI[ii] + per * UVP[ii - 1];
            d2 = (1.0 - per) * UVI[ii] + per * UVP[ii];

            k1 = -1;
            fac = d2 - d1;

            for ( jj = 0; jj <= 3; jj++ )
            {
                while( UVT[jj][ind[jj]]LE d1 )
                    ind[jj] += 1;

                while( UVT[jj][ind[jj]]GT d1 AND UVT[jj][ind[jj]]LT d2 )
                {
                    d3 = fabs( UVI[ii] - UVT[jj][ind[jj]] );

                    if( d3 LT fac )
                    {
                        fac = d3;
                        k1 = ind[jj];
                        k2 = jj;
                    }
                    ind[jj] += 1;
                }
            }

            if( k1 EQ - 1 )
                X[++rs] = UVI[ii];
            else
                X[++rs] = UVT[k2][k1];
        }
    }

    N_KnotVectorFromRealArray( &knx, X, rs );

    error = N_SrfInsertKnots( surT, &knx, NL_UDIR, surT, SG, SG ); /* now refine */

    if( error EQ NL_YES )
        NL_OUT;

    if( V NEQ NULL )
    {
        if( V[mv - 3]NEQ V[mv] )
            NL_ERROR( NL_INP_ERR );

        if( V[0]NEQ V[3] )
            NL_ERROR( NL_INP_ERR );

        if( V[0]NEQ vs OR V[mv]NEQ ve )
            NL_ERROR( NL_INP_ERR );

        for ( rs = -1, ii = 4; ii <= nv; ii++ )
        {
            if( V[ii]GT V[ii + 1] )
                NL_ERROR( NL_INP_ERR );

            X[++rs] = V[ii];
        }
    }
    else
    {
        /* first get sorted list of parameters */

        vv = N_AllocReal1dArray( np + 2, &SL );

        if( vv EQ NULL )
            NL_QUIT;

        vv[0] = vs;
        vv[np + 2] = ve;

        for ( ii = 0; ii <= np; ii++ )
            vv[ii + 1] = v[ii];

        N_ShellSortReal( vv, np + 2 );

        kk = np + 2;

        for ( ii = 0; ii < kk; ii++ )
        {
            if( vv[ii]EQ ve )
            {
                kk = ii;
                break;
            }

            jj = 1;

            while( vv[ii]EQ vv[ii + jj] )
                jj += 1;

            if( jj GT 1 )
            {
                k1 = ii + 1;

                for ( k2 = ii + jj; k2 <= kk; k2++ )
                    vv[k1++] = vv[k2];
                kk -= (jj - 1);
            }
        }

        /* now get ideal knot vectors for degrees 3 and 2 */

        kni = N_AllocKnotVectorAndArray( mv, &SL );

        if( kni EQ NULL )
            NL_QUIT;
        knp = N_AllocKnotVectorAndArray( mv - 1, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knp, &ii, &UVP );
        N_KnotVectorGetKnots( kni, &ii, &UVI );

        error = N_FitCalcKnotVectorCrvApprox( vv, kk, nv, 3, kni ); /* for degree 3 */

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( vv, kk, nv, 2, knp ); /* for degree 2 */

        if( error EQ NL_YES )
            NL_OUT;

        /* now get X from ideals and candidate */

        N_CrvGetKnots( curV[0], &ii, &( UVT[0] ) ); /* candidate knots */
        N_CrvGetKnots( curV[1], &ii, &( UVT[1] ) );
        N_CrvGetKnots( derV[0], &ii, &( UVT[2] ) );
        N_CrvGetKnots( derV[1], &ii, &( UVT[3] ) );

        N_CrvGetDegree( curV[0], &pq );
        ind[0] = pq + 1;
        N_CrvGetDegree( curV[1], &pq );
        ind[1] = pq + 1;
        N_CrvGetDegree( derV[0], &pq );
        ind[2] = pq + 1;
        N_CrvGetDegree( derV[1], &pq );
        ind[3] = pq + 1;

        for ( rs = -1, ii = 4; ii <= nv; ii++ )
        {
            d1 = (1.0 - per) * UVI[ii] + per * UVP[ii - 1];
            d2 = (1.0 - per) * UVI[ii] + per * UVP[ii];

            k1 = -1;
            fac = d2 - d1;

            for ( jj = 0; jj <= 3; jj++ )
            {
                while( UVT[jj][ind[jj]]LE d1 )
                    ind[jj] += 1;

                while( UVT[jj][ind[jj]]GT d1 AND UVT[jj][ind[jj]]LT d2 )
                {
                    d3 = fabs( UVI[ii] - UVT[jj][ind[jj]] );

                    if( d3 LT fac )
                    {
                        fac = d3;
                        k1 = ind[jj];
                        k2 = jj;
                    }
                    ind[jj] += 1;
                }
            }

            if( k1 EQ - 1 )
                X[++rs] = UVI[ii];
            else
                X[++rs] = UVT[k2][k1];
        }
    }

    N_KnotVectorFromRealArray( &knx, X, rs );

    error = N_SrfInsertKnots( surT, &knx, NL_VDIR, surT, SG, SG ); /* now refine */

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( surT, &Tw, &UT, &VT );

    /*  Allocate memory for the system of equations.    */
    /*                                                  */
    /*  The unknowns are: Tw[i][j], i=2,...,nu-2 and    */
    /*  j=2,...,nv-2                                    */

    nuk = (nu - 3) * (nv - 3) - 1; /* high index of unknowns */

    error = N_SetRealMatrix( &rma, np, nuk, NL_MT_FULL, np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixData( &rma, &ii, &jj, &RN, &mtp, &kk );

    sol = N_AllocCPt1dArray( nuk, &SL );

    if( sol EQ NULL )
        NL_QUIT;

    rhs = N_AllocPt1dArray( np, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    for ( ii = 0; ii <= np; ii++ )
        for ( jj = 0; jj <= nuk; jj++ )
            RN[ii][jj] = 0.0;

    /*  Allocate memory for the basis functions  */

    hs = N_AllocReal1dArray( 15, &SL ); /* hs,ht Hermite basis functions   */

    if( hs EQ NULL )
        NL_QUIT;
    ht = &hs[4];

    ntu = &ht[4];  /* ntu,ntv b-spline functions for  */
    ntv = &ntu[4]; /* T-surface in Coons construction */

    /* Need knot vector objects for computing basis functions */

    N_SrfGetKnotVectors( surT, &kntu, &kntv );

    /* Now loop and set up the np+1 equations */

    for ( ii = 0; ii <= np; ii++ )
    {

        N_CopyPt( P[ii], &rhs[ii] );

        /* transform u,v to [0,1] and evaluate Hermites */

        s = (u[ii] - us) / du;
        t = (v[ii] - vs) / dv;

        d1 = s * s;
        d2 = s * d1;

        hs[0] = 1.0 - 3.0 *d1 + 2.0 *d2;
        hs[1] = 3.0 *d1 - 2.0 *d2;
        hs[2] = s - 2.0 *d1 + d2;
        hs[3] = -d1 + d2;

        d1 = t * t;
        d2 = t * d1;

        ht[0] = 1.0 - 3.0 *d1 + 2.0 *d2;
        ht[1] = 3.0 *d1 - 2.0 *d2;
        ht[2] = t - 2.0 *d1 + d2;
        ht[3] = -d1 + d2;

        /* Evaluate point on v-loft surface */

        error = N_CrvEval( curU[0], u[ii], NL_LEFT, &P1 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( curU[1], u[ii], NL_LEFT, &P2 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( derU[0], u[ii], NL_LEFT, &P3 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( derU[1], u[ii], NL_LEFT, &P4 );

        if( error EQ NL_YES )
            NL_OUT;

        ht[2] *= dv;
        ht[3] *= dv;

        N_Combine4Pts( ht[0], P1, ht[1], P2, ht[2], P3, ht[3], P4, &tmp1[0] );
        N_VectorBlendPt( -1.0, tmp1[0], &rhs[ii] );

        /* Evaluate point on u-loft surface */

        error = N_CrvEval( curV[0], v[ii], NL_LEFT, &P1 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( curV[1], v[ii], NL_LEFT, &P2 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( derV[0], v[ii], NL_LEFT, &P3 );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvEval( derV[1], v[ii], NL_LEFT, &P4 );

        if( error EQ NL_YES )
            NL_OUT;

        hs[2] *= du;
        hs[3] *= du;

        N_Combine4Pts( hs[0], P1, hs[1], P2, hs[2], P3, hs[3], P4, &tmp1[0] );
        N_VectorBlendPt( -1.0, tmp1[0], &rhs[ii] );

        /* now evaluate b-spline basis functions */

        error = N_BasisEval( kntu, 3, u[ii], NL_LEFT, ntu, &uspn );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisEval( kntv, 3, v[ii], NL_LEFT, ntv, &vspn );

        if( error EQ NL_YES )
            NL_OUT;

        /* There are 16 terms of the form:  N(j1,j2)*Tw[j1][j2].  */
        /* For known Tw[][], these get put over on the rhs of the */
        /* system. For unknowns, the N(j1,j2) gets accumulated in */
        /* the coefficient matrix.                                */

        for ( j1 = uspn - 3, jj = 0; jj <= 3; j1++, jj++ )
            for ( j2 = vspn - 3, kk = 0; kk <= 3; j2++, kk++ )
            {
                if( j1 LT 2 OR j1 GT nu - 2 OR j2 LT 2 OR j2 GT nv - 2 )
                {
                    N_CPtToPtEuclid( Tw[j1][j2], &P1 );
                    N_VectorBlendPt( ntu[jj] * ntv[kk], P1, &rhs[ii] );
                }
                else
                {
                    RN[ii][(j2 - 2)*( nu - 3 ) + j1 - 2] -= (ntu[jj]*ntv[kk]);
                }
            }
    }

    /* Now solve via Single Value Decomposition */

    WW = N_AllocReal1dArray( nuk, &SL );
    VV = N_AllocReal2dArray( np, nuk, &SL );

    if( WW EQ NULL OR VV EQ NULL )
        NL_QUIT;

    error = N_RealMatrixSingleValueDecompose( RN, WW, VV, np, nuk );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SingleValueDecomposeSolve( RN, WW, VV, np, nuk, (NL_VOID *)rhs, NL_EPOINT, NL_YES, (NL_VOID *)sol );

    if( error EQ NL_YES )
        NL_OUT;

    /* Load the control points into the proper locations */

    for ( kk = 0, ii = 2; ii < nv - 1; ii++ )
        for ( jj = 2; jj < nu - 1; jj++ )
            N_CopyCPt( sol[kk++], &Tw[jj][ii] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateTensorProductSrf */

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine creates a bicubic Coons
     surface interpolating four NON-RATIONAL boundary curves and cross-
     boundary derivatives.  The tensor product surface  which  is  sub-
     tracted out in the Coons construction is also input.  The boundary 
     curves are assumed to intersect at their  respective  end  points,
     and  the  cross-derivatives are  assumed  to be compatible in  the
     Coons sense  (including  their implied twists).  The curU and derU
     must be defined on the same u-parameter  domain,  and the curV and
     derV must be defined on the same v-parameter domain  (not  necess-
     arily the same two domains).  The  Coons  surface  will  have  the 
     parameterization of these curves.  The tensor product surface must
     also be defined with the same parameter domains as the  curves and 
     cross-derivatives,  and  it  must  be compatible with the boundary
     curves and derivatives in the Coons sense,  including  twists.  If 
     the  output surface is initialized to NULL, memory  to  store  new 
     control points and knots is allocated.  A  typical calling example
     is:

       NL_CURVE    **curU, **derU, **curV, **derV;
       NL_SURFACE  surT, sur;
       NL_STACKS   SC, SS;
       ...
       (get boundary curves, derivatives, and surT);
       (e.g.: see N_Create4CrossBoundaryDerivCrvs for derivatives and N_CreateTensorProductSrf for surT)
       ...
       N_SrfInitArrays(&sur);
       N_CreateCoonsSrfCrossBoundaryDerivs(curU,derU,curV,derV,&surT,&sur,&SC,&SS);

     If memory is  available, sur  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest  indexes in sur's 
     knot  vector and  polygon  objects.  ALL INPUT DATA MUST BE ON THE  
     SAME STACK  "SC". THE INPUT DATA MAY BE DESTROYED. 


   ACCESS:
   
     curU  , in/out ,  U-boundaries (at v=vmin and v=vmax)
     derU  , in/out ,  derU[0] is the derivative curve across the bottom 
                       (curU[0]) boundary, derU[1] the derivative across
                       the top boundary (curU[1])
     curV  , in/out ,  V-boundaries (at u=umin and u=umax)
     derV  , in/out ,  derV[0] is the derivative curve across  the  left
                       (curV[0]) boundary, derV[1] the derivative across
                       the right boundary (curV[1])
     surT  , input  ,  Tensor product surface to be subtracted out. Must
                       be  compatible  with  curU,derU,curV,derV  in the 
                       Coons sense
     sur   , output ,  Bicubic Coons surface
     SC    , input  ,  curs', ders' and surT memory stack
     SS    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCoonsSrfCrossBoundaryDerivs( NL_CURVE ** curU, NL_CURVE ** derU, NL_CURVE ** curV, NL_CURVE ** derV, NL_SURFACE *surT, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCoonsSrfCrossBoundaryDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UA, *VA, one_3, d1, d2, us, ue, vs, ve, du, dv; 

    NL_CPOINT ** Pw, ** Uw, ** Vw, ** Tw, *Aw;

    NL_CURVE ** curAA, ** curBB;

    NL_SURFACE ** surA, surU, surV, tens;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check rationality, parameter domains, and initialize constants */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( derV[1] ) )
        NL_ERROR( NL_INP_ERR );

    one_3 = 1.0 / 3.0;

    N_CrvGetParamBounds( curU[0], &us, &ue );
    N_CrvGetParamBounds( curU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derU[0], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derU[1], &d1, &d2 );

    if( us NEQ d1 OR ue NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    du = ue - us;

    N_CrvGetParamBounds( curV[0], &vs, &ve );
    N_CrvGetParamBounds( curV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derV[0], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );
    N_CrvGetParamBounds( derV[1], &d1, &d2 );

    if( vs NEQ d1 OR ve NEQ d2 )
        NL_ERROR( NL_INP_ERR );

    dv = ve - vs;

    /* Make input data compatible in the B-spline sense */

    curAA = N_AllocArrayCrvPtrs( 3, SC );

    if( curAA EQ NULL )
        NL_QUIT;

    curBB = N_AllocArrayCrvPtrs( 3, SC );

    if( curBB EQ NULL )
        NL_QUIT;

    curAA[0] = curV[0];
    curAA[1] = curV[1];
    curAA[2] = derV[0];
    curAA[3] = derV[1];

    curBB[0] = curU[0];
    curBB[1] = curU[1];
    curBB[2] = derU[0];
    curBB[3] = derU[1];

    if( NOT N_CrvsAreCombatible( curAA, 3 ) )
    {
        error = N_CrvsMakeCompatible( curAA, 3, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( NOT N_CrvsAreCombatible( curBB, 3 ) )
    {
        error = N_CrvsMakeCompatible( curBB, 3, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /********************************/
    /* Compute the various surfaces */
    /********************************/

    /* Allocate memory */

    N_CrvGetCPtsDegreeAndKnots( curAA[0], &m, &Aw, &q, &s, &VA );
    N_CrvGetCPtsDegreeAndKnots( curBB[0], &n, &Aw, &p, &r, &UA );

    error = N_AllocSrfArrays( &surU, 3, m, 3, q, 7, s, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocSrfArrays( &surV, n, 3, p, 3, r, 7, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute u-blend */

    N_SrfGetCPtsAndKnots( &surU, &Uw, &U, &V );

    N_CrvGetCPts( curAA[0], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_CopyCPt( Aw[j], &Uw[0][j] );

    N_CrvGetCPts( curAA[1], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_CopyCPt( Aw[j], &Uw[3][j] );

    N_CrvGetCPts( curAA[2], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_Combine2CPts( 1.0, Uw[0][j], du * one_3, Aw[j], &Uw[1][j] );

    N_CrvGetCPts( curAA[3], &m, &Aw );

    for ( j = 0; j <= m; j++ )
        N_Combine2CPts( 1.0, Uw[3][j], -du * one_3, Aw[j], &Uw[2][j] );

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = us;
        U[4 + i] = ue;
    }

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* Compute v-blend */

    N_SrfGetCPtsAndKnots( &surV, &Vw, &U, &V );

    N_CrvGetCPts( curBB[0], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Aw[i], &Vw[i][0] );

    N_CrvGetCPts( curBB[1], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Aw[i], &Vw[i][3] );

    N_CrvGetCPts( curBB[2], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_Combine2CPts( 1.0, Vw[i][0], dv * one_3, Aw[i], &Vw[i][1] );

    N_CrvGetCPts( curBB[3], &n, &Aw );

    for ( i = 0; i <= n; i++ )
        N_Combine2CPts( 1.0, Vw[i][3], -dv * one_3, Aw[i], &Vw[i][2] );

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = vs;
        V[4 + j] = ve;
    }

    /* Copy surT to structure on local stack */

    N_SrfInitArrays( &tens );
    error = N_SrfCopy( surT, &tens, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Make surfaces compatible */

    surA = N_AllocArraySrfPtrs( 2, &SL );

    if( surA EQ NULL )
        NL_QUIT;

    surA[0] = &surU;
    surA[1] = &surV;
    surA[2] = &tens;

    error = N_MakeSrfsCompatibleUV( surA, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute output surface */

    N_SrfGetCPtsDegreesAndKnots( surA[0], &n, &m, &Uw, &p, &q, &r, &s, &UA, &VA );

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPts( surA[1], &n, &m, &Vw );
    N_SrfGetCPts( surA[2], &n, &m, &Tw );
    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TranslateSum2CPts( Uw[i][j], 1.0, Vw[i][j], -1.0, Tw[i][j], &Pw[i][j] );
        }
    }

    for ( i = 0; i <= r; i++ )
        U[i] = UA[i];

    for ( j = 0; j <= s; j++ )
        V[j] = VA[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCoonsSrfCrossBoundaryDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This advanced  surface  construction  routine  constructs a skinned 
     surface through a set of cross-sectional curves. First, it approxi-
     mates each section curve up to given tolerances, then  interpolates
     thru these curves. The general  skinning operator has the following 
     shortcomings:

       1. If the curves are inconsistently parametrized, the surface may
          wiggle (or even loop) around  even though  the section  curves 
          are geometrically perfect.
       2. G-continuous rational curves may be discontinuous in 4-D space
          giving rise to  surfaces with  creases. For example, if one of
          the section curves is a circle with G^1 continuity, the resul-
          tant surface will only be C^0 continuous.
       3. If some of the cross-sectional curves are rational, then  even
          with reasonable differences in weights the skinned surface may
          have  control points with negative weights. This may result in 
          unwanted  bumps and  wiggles even  though the  curves may look 
          perfectly okay.

     This routine  avoids the above problems  by replacing  the original
     section curves with non-rational approximants that are consistently
     parametrized. The price of this  flexibility is an increased number
     of control points and the  extra time it takes to reapproximate the
     section curves. A typical calling example is:

       NL_CURVE    **cur;
       NL_INDEX    k;
       NL_DEGREE   p, q;
       NL_REAL     *eps;
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (get curves cur, choose p, q, eps);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSkinSrfInterp(cur,k,NL_YES,p,q,eps,&sur,&SC,&SS);


   ACCESS:
   
     cur  , input  ,  Cross-sectional  curves. THESE  ARE  ASSUMED TO BE 
                      THE V-CURVES OF THE SKINNED NL_SURFACE.
     k    , input  ,  Highest index in cur
     cld  , input  ,  Flag:
                        NL_YES: if cur[0]=cur[k] close the skinned  surface
                             with C^1 smoothness
                        NL_NO : do not force NL_C1  closure  if  cur[0]=cur[k]
     p,q  , input  ,  Degrees of the skinned surface:
                        p: degree in the skinning direction which is the
                           u-direction
                        q: degree of the  approximating curves in the v-
                           direction
     eps  , input  ,  Tolerances;  curves  approximating  cur[i]  do not 
                      deviate more than eps[i]
     sur  , output ,  Skinned surface
     SC   , input  ,  cur's memory stack
     SS   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSkinSrfInterp( NL_CURVE ** cur, NL_INDEX k, NL_FLAG cld, NL_DEGREE p, NL_DEGREE q, NL_REAL *eps, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSkinSrfInterp");
    NL_PRIVATE NL_REAL per = 1.0;

    NL_FLAG closed = NL_NO, error = NL_NO;

    NL_INDEX *m, *nu, i, j, ii, jj, id = 0, is, ie, rs, ss, ns, ms, k2, kl, l, b;

    NL_REAL *UT, *VT, *US, *VS, *ul, *up, *epl, us, ue = 0.0, mag, exp, f1, f2, num, del, Muu, uinc, uu, gro;

    NL_POINT ** P, *Q, *R, Ds[2], De[2];

    NL_VECTOR *T, D;

    NL_CPOINT ** Sw, *Cw;

    NL_KNOTVECTOR *knu, *knv;

    NL_CURVE ** curU, ** curV, ** curB, curI;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Allocate memory and prepare data */

    m = N_AllocInt1dArray( k, &SL );

    if( m EQ NULL )
        NL_QUIT;

    epl = N_AllocReal1dArray( k, &SL );

    if( epl EQ NULL )
        NL_QUIT;

    ul = N_AllocReal1dArray( k, &SL );

    if( ul EQ NULL )
        NL_QUIT;

    up = N_AllocReal1dArray( k, &SL );

    if( up EQ NULL )
        NL_QUIT;

    P = N_AllocPtPtr1dArray( k, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        epl[i] = 0.5 *eps[i];
        up[i] = 0.0;
    }

    /*************************************/
    /* Sample each cross-sectional curve */
    /*************************************/

    if( q EQ 1 )
        exp = 0.5;
    else
        exp = 0.34;

    for ( i = 0; i <= k; i++ )
    {
        if( N_FloatOpIsBad( 1.0, eps[i], NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        f1 = 1.0 / eps[i];
        f1 = pow( f1, exp );
        f2 = (NL_REAL)q;

        error = N_CrvDecomposeBez( cur[i], &curB, &b, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        nu = N_AllocInt1dArray( b, &SL );

        if( nu EQ NULL )
            NL_QUIT;

        m[i] = 0;

        for ( j = 0; j <= b; j++ )
        {
            N_CrvReparamToInterval( curB[j], NL_UNITSPAN );

            error = N_CrvGetMax2ndDeriv( curB[j], &Muu );

            if( error EQ NL_YES )
                NL_OUT;

            num = 0.125 *Muu;
            gro = NL_MAX( f2, sqrt( num ) );
            del = f1 * gro;
            del = ceil( del );
            nu[j] = (NL_INDEX)del;
            m[i] += nu[j];
        }

        P[i] = N_AllocPt1dArray( m[i], &SL );

        if( P[i]EQ NULL )
            NL_QUIT;

        l = -1;

        for ( j = 0; j <= b; j++ )
        {
            N_CrvGetParamBounds( curB[j], &us, &ue );
            uinc = (ue - us) / nu[j];

            for ( ii = 0; ii < nu[j]; ii++ )
            {
                uu = us + ii * uinc;

                error = N_CrvEval( curB[j], uu, NL_LEFT, &P[i][++l] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }

        error = N_CrvEval( curB[b], ue, NL_LEFT, &P[i][++l] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /**************************************/
    /* Approximate cross-sectional curves */
    /**************************************/

    curV = N_AllocArrayCrvPtrs( k, &SL );

    if( curV EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        curV[i] = N_AllocCrv( &SL );

        if( curV[i]EQ NULL )
            NL_QUIT;
    }

    knv = NULL;
    T = NULL;

    for ( i = 0; i <= k; i++ )
    {
        /* See if cur[i] is smoothly closed */

        if( N_CrvIsClosed( cur[i] ) )
        {
            N_CrvGetParamBounds( cur[i], &us, &ue );

            error = N_CrvDerivs( cur[i], us, NL_LEFT, 1, Ds );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDerivs( cur[i], ue, NL_LEFT, 1, De );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &Ds[1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &De[1] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDiff( De[1], Ds[1], &D );
            N_VectorMagnitude( D, &mag );

            if( mag LT NL_MTOL )
            {
                N_VectorCombine( 0.5, Ds[1], 0.5, De[1], &D );

                T = &D;
            }
        }

        N_CrvInitArrays( curV[i] );
        error = N_FitCrvApproxKnotsTol( P[i], m[i], q, T, T, NL_TANGENT, epl[i], &knv, NL_NO, curV[i], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curV, k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curV[0], &ss, &VT );
    N_CrvGetArraySizes( curV[0], &ms, &ss );

    /**********************************************/
    /* Now fit the cross-sectional curves curV[i] */
    /**********************************************/

    if( N_CrvsAreCoincident( cur[0], cur[k], NL_MTOL, SC ) )
    {
        if( cld EQ NL_YES )
            closed = NL_YES;
    }

    curU = N_AllocArrayCrvPtrs( ms, &SL );

    if( curV EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= ms; j++ )
    {
        curU[j] = N_AllocCrv( &SL );

        if( curU[j]EQ NULL )
            NL_QUIT;
    }

    Q = N_AllocPt1dArray( k, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    /* Get parametrization for u-directional fitting */

    for ( j = 0; j <= ms; j++ )
    {
        for ( i = 0; i <= k; i++ )
        {
            N_CrvGetCPts( curV[i], &ms, &Cw );
            N_CPtToPtEuclid( Cw[j], &Q[i] );
        }

        error = N_FitCalcCrvParamValues( (NL_VOID *)Q, k, NL_EPOINT, NL_CHORDLENGTH, ul );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= k; i++ )
            up[i] += ul[i];
    }

    for ( i = 0; i <= k; i++ )
        up[i] /= ((NL_REAL)ms + 1.0);

    knu = N_AllocKnotVectorAndArray( k + p + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    N_FitCrvCalcKnotVector( up, k, p, knu );

    T = NULL;

    for ( j = 0; j <= ms; j++ )
    {
        /* Get control point array */

        for ( i = 0; i <= k; i++ )
        {
            N_CrvGetCPts( curV[i], &ms, &Cw );
            N_CPtToPtEuclid( Cw[j], &Q[i] );
        }

        /* If closed surface wanted, need end derivatives */

        if( closed EQ NL_YES )
        {
            k2 = k / 2;
            is = NL_MAX( k2, k - p );
            ie = NL_MIN( k2, p );
            kl = k - is + ie;

            R = N_AllocPt1dArray( kl, &SL );

            if( R EQ NULL )
                NL_QUIT;

            N_VectorCopy( Q[is], &R[0] );

            ii = is + 1;
            jj = 1;

            while( jj LE kl )
            {
                N_VectorCopy( Q[ii], &R[jj] );

                if( ii EQ k )
                {
                    ii = 0;
                    id = jj;
                }
                ii++;
                jj++;
            }

            N_CrvInitArrays( &curI );
            error = N_FitCrvInterp( R, kl, p, NL_CHORDLENGTH, &curI, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_FitCalcCrvParamValues( (NL_VOID *)R, kl, NL_EPOINT, NL_CHORDLENGTH, ul );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDerivs( &curI, ul[id], NL_LEFT, 1, Ds );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &Ds[1] );

            if( error EQ NL_YES )
                NL_OUT;

            T = &Ds[1];
        }

        N_CrvInitArrays( curU[j] );
        error = N_FitCrvKnotsAndTangents( Q, k, up, T, T, NL_TANGENT, &knu, per, p, curU[j], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curU, ms, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curU[0], &rs, &UT );
    N_CrvGetArraySizes( curU[0], &ns, &rs );

    /* Get output surface */

    error = N_SrfSizeArrays( sur, ns, ms, p, q, rs, ss, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );

    for ( j = 0; j <= ms; j++ )
    {
        N_CrvGetCPts( curU[j], &ns, &Cw );

        for ( i = 0; i <= ns; i++ )
            N_CopyCPt( Cw[i], &Sw[i][j] );
    }

    for ( i = 0; i <= rs; i++ )
        US[i] = UT[i];

    for ( j = 0; j <= ss; j++ )
        VS[j] = VT[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSkinSrfInterp */


#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  advanced   surface  construction  routine creates  the  data 
     defining  cross-boundary   derivatives.  The   cross-derivative is  
     defined  from a curve  and a vector field.  The  curve  and vector
     field must be defined on the same knot vector  (must be compatible
     as b-spline curves).  It can be constructed as a boundary strip of
     type NL_LEFT, NL_RIGHT, NL_BOTTOM or NL_TOP. A typical calling example is:

       NL_CURVE    cur, vfield;
       NL_SURFACE  der;
       NL_STACKS   SG;
       ...
       (get boundary curve cur and vector field vfield);
       ...
       N_SrfInitArrays(&der);
       N_CrossBoundaryDerivsVectorField(&cur,&vfield,NL_LEFT,&der,&SG);

     If memory is  available, der  is not  initialized and  the routine
     assumes that memory allocation  has been done. However, it  checks
     for the proper amount by looking  at the highest  indexes in der's 
     knot vector and  polygon  objects.


   ACCESS:
   
     cur    , input  ,  Boundary curve across which derivative applies
     vfield , input  ,  Vector field (must be nonrational).  Must have
                        the same knots as cur
     bndy   , input  ,  Boundary flag indicating how  cross-derivative
                        strip should be constructed  (i.e. from  which
                        boundary of an imaginary surface would cur  be 
                        extracted):
                          NL_LEFT   : cur corresponds to u=umin boundary
                          NL_RIGHT  : cur corresponds to u=umax boundary
                          NL_BOTTOM : cur corresponds to v=vmin boundary
                          NL_TOP    : cur corresponds to v=vmax boundary
     der  , output ,  Derivative data
     SG   , input  ,  der's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrossBoundaryDerivsVectorField( NL_CURVE *cur, NL_CURVE *vfield, NL_FLAG bndy, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrossBoundaryDerivsVectorField");

    NL_FLAG error = NL_NO;

    NL_REAL *U, *V, *UVC;

    NL_INDEX ii, n, m, r, s;

    NL_DEGREE p, q;

    NL_CPOINT *Pw, *Qw, ** Sw;

    NL_VECTOR T;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /*  Check for error  */

    /* when vfield is rational - quit */
    /* if ( N_IsCrvRat(vfield)) NL_ERROR(NL_INP_ERR);    */
    /* if ( N_IsCrvRat(cur) )   NL_ERROR(NL_INP_ERR);    */

    /* when cur and vfield knot counts vary - quit*/
    N_CrvGetKnots( cur, &n, &U );
    N_CrvGetKnots( vfield, &m, &V );

    if( n NEQ m )
        NL_ERROR( NL_INP_ERR );

    /* when cur and vfield knot values vary - quit */
    for ( ii = 0; ii <= n; ii++ )
        if( U[ii]NEQ V[ii] )
            NL_ERROR( NL_INP_ERR );

    /* Handle cases determining direction */

    if( bndy EQ NL_LEFT OR bndy EQ NL_RIGHT )
    {
        /* get local values for cur basic components */
        N_CrvGetCPtsDegreeAndKnots( cur, &m, &Pw, &q, &s, &UVC );
        n = 1;
        p = 1;
        r = n + p + 1;
    }
    else
    {
        /* get local values for cur basic components */
        N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &r, &UVC );
        m = 1;
        q = 1;
        s = m + q + 1;
    }

    /* get local control polygon data for vfield */
    N_CrvGetCPts( vfield, &ii, &Qw );

    /*  See if memory is needed  */

    error = N_SrfSizeArrays( der, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* get local der surface control point and knot values */
    N_SrfGetCPtsAndKnots( der, &Sw, &U, &V );

    /*  Load knots  */

    if( bndy EQ NL_LEFT OR bndy EQ NL_RIGHT )
    {
        for ( ii = 0; ii <= s; ii++ )
            V[ii] = UVC[ii];

        U[0] = U[1] = 0.0;
        U[2] = U[3] = 1.0;
    }
    else
    {
        for ( ii = 0; ii <= r; ii++ )
            U[ii] = UVC[ii];

        V[0] = V[1] = 0.0;
        V[2] = V[3] = 1.0;
    }

    /*  Load control points  */

    if( bndy EQ NL_LEFT OR bndy EQ NL_RIGHT )
    {
        for ( ii = 0; ii <= m; ii++ )
        {
            if( bndy EQ NL_LEFT )
            {
                /* let Sw[0][ii] = Pw[ii] */
                N_CopyCPt( Pw[ii], &Sw[0][ii] );

                /* convert vector field control points to euclidean space */
                N_CPtToPtEuclid( Qw[ii], &T );

                /* let Sw[1][ii] = Pw[ii] + t */
                N_TranslateCPt( Pw[ii], T, &Sw[1][ii] );
            }
            else
            {
                /* let Sw[0][ii] = Pw[ii] */
                N_CopyCPt( Pw[ii], &Sw[1][ii] );

                /* convert vector field control points to euclidean space */
                N_CPtToPtEuclid( Qw[ii], &T );

                /* let Sw[1][ii] = Pw[ii] + t */
                N_VectorReverse( T, &T );
                N_TranslateCPt( Pw[ii], T, &Sw[0][ii] );
            }
        }
    }
    else
    {
        for ( ii = 0; ii <= n; ii++ )
        {
            if( bndy EQ NL_BOTTOM )
            {
                N_CopyCPt( Pw[ii], &Sw[ii][0] );
                N_CPtToPtEuclid( Qw[ii], &T );
                N_TranslateCPt( Pw[ii], T, &Sw[ii][1] );
            }
            else
            {
                N_CopyCPt( Pw[ii], &Sw[ii][1] );
                N_CPtToPtEuclid( Qw[ii], &T );
                N_VectorReverse( T, &T );
                N_TranslateCPt( Pw[ii], T, &Sw[ii][0] );
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrossBoundaryDerivsVectorField */

/*******************************************************************//**


   DESCRIPTION:

     This advanced surface construction routine computes a cross-deriva-
     tive field  given a base  surface, end  derivatives and end twists.
     The cross derivative  interpolates the end  derivatives and the end
     twists, plus  assumes  some  number of  internal  cross-derivatives 
     taken from  the base  surface. The output  cross-derivative has the
     same number of control points and the same  knots as the correspon-
     ding boundary curve, i.e. they are  compatible as  B-spline curves.
     This routine is useful for cross-derivative estimation with tangent
     or twist constraints  using a  base surface  such as a  bi-linearly
     blended  Coons patch. In  case no  tangent or twist  contraints are 
     available, they are taken from the base surface. A typical  calling 
     example is:

       NL_SURFACE  sur;
       NL_VECTOR   *Ds, *De, *Ts, *Te;
       NL_CURVE    der;
       NL_STACKS   SG;
       ...
       (get surface and derivatives and twists);
       ...
       N_CrvInitArrays(&der);
       N_CreateDerivField(&sur,NL_LEFT,Ds  ,De,Ts,Te  ,&der,&SG);
       N_CreateDerivField(&sur,NL_LEFT,NULL,De,Ts,NULL,&der,&SG);

     If memory is  available, der  is not  initialized  and the  routine
     assumes that memory  allocation  has been done. However, it  checks  
     for the proper amount  by looking  at the highest  indexes in der's 
     knot vector and  polygon  objects.


   ACCESS:
   
     sur   , input  ,  NURBS surface
     bfl   , input  ,  Flag:
                         NL_LEFT  : compute cross-derivative along u=umin
                         NL_RIGHT : compute cross-derivative along u=umax
                         NL_BOTTOM: compute cross-derivative along v=vmin
                         NL_TOP   : compute cross-derivative along v=vmax
     Ds,De , input  ,  Start and end derivatives:
                          = NULL: no start/end derivative given; take it
                                  from the base surface
                         != NULL: start/end derivative passed in
     Ts,Te , input  ,  Start and end twists:
                          = NULL: no start/end  twist is  given; take it
                                  from the base surface
                         != NULL: start/end twist passed in
     der   , output ,  Cross-boundary derivative
     SG    , input  ,  der's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateDerivField( NL_SURFACE *sur, NL_FLAG bfl, NL_VECTOR *Ds, NL_VECTOR *De, NL_VECTOR *Ts, NL_VECTOR *Te, NL_CURVE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateDerivField");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, rs, ss;

    NL_DEGREE p;

    NL_REAL *U, *t, *U2, *US, *VS, sum, op, fs = 0, fe;

    NL_POINT ** SD, *P, Q1, Q2;

    NL_CPOINT *Pw;

    NL_VECTOR *Vs, *Ve, DDs = { 0,0,0 }, DDe = { 0,0,0 }, TTs = { 0,0,0 }, TTe = { 0,0,0 };

    NL_CURVE cur;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get cross-derivative off the base surface */

    if( Ds EQ NULL AND De EQ NULL AND Ts EQ NULL AND Te EQ NULL )
    {
        /* There are no constraints */

        error = N_CrossBoundDerivCrvNonRatSrf( sur, bfl, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    N_CrvInitArrays( &cur );
    error = N_CrossBoundDerivCrvNonRatSrf( sur, bfl, &cur, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( &cur, &m, &U );
    N_CrvGetArraySizes( &cur, &n, &m );
    N_CrvGetDegree( &cur, &p );
    N_CrvGetKnotVector( &cur, &knt );
    N_SrfGetKnots( sur, &rs, &ss, &US, &VS );

    SD = N_AllocPt2dArray( 1, 1, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    if( bfl EQ NL_LEFT )
    {
        if( Ds EQ NULL OR Ts EQ NULL )
        {
            error = N_SrfDerivs( sur, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( Ds EQ NULL )
            N_VectorCopy( SD[1][0], &DDs );
        else
            N_VectorCopy( *Ds, &DDs );

        if( Ts EQ NULL )
            N_VectorCopy( SD[1][1], &TTs );
        else
            N_VectorCopy( *Ts, &TTs );

        if( De EQ NULL OR Te EQ NULL )
        {
            error = N_SrfDerivs( sur, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( De EQ NULL )
            N_VectorCopy( SD[1][0], &DDe );
        else
            N_VectorCopy( *De, &DDe );

        if( Te EQ NULL )
            N_VectorCopy( SD[1][1], &TTe );
        else
            N_VectorCopy( *Te, &TTe );
    }

    if( bfl EQ NL_RIGHT )
    {
        if( Ds EQ NULL OR Ts EQ NULL )
        {
            error = N_SrfDerivs( sur, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( Ds EQ NULL )
            N_VectorCopy( SD[1][0], &DDs );
        else
            N_VectorCopy( *Ds, &DDs );

        if( Ts EQ NULL )
            N_VectorCopy( SD[1][1], &TTs );
        else
            N_VectorCopy( *Ts, &TTs );

        if( De EQ NULL OR Te EQ NULL )
        {
            error = N_SrfDerivs( sur, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( De EQ NULL )
            N_VectorCopy( SD[1][0], &DDe );
        else
            N_VectorCopy( *De, &DDe );

        if( Te EQ NULL )
            N_VectorCopy( SD[1][1], &TTe );
        else
            N_VectorCopy( *Te, &TTe );
    }

    if( bfl EQ NL_BOTTOM )
    {
        if( Ds EQ NULL OR Ts EQ NULL )
        {
            error = N_SrfDerivs( sur, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( Ds EQ NULL )
            N_VectorCopy( SD[0][1], &DDs );
        else
            N_VectorCopy( *Ds, &DDs );

        if( Ts EQ NULL )
            N_VectorCopy( SD[1][1], &TTs );
        else
            N_VectorCopy( *Ts, &TTs );

        if( De EQ NULL OR Te EQ NULL )
        {
            error = N_SrfDerivs( sur, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( De EQ NULL )
            N_VectorCopy( SD[0][1], &DDe );
        else
            N_VectorCopy( *De, &DDe );

        if( Te EQ NULL )
            N_VectorCopy( SD[1][1], &TTe );
        else
            N_VectorCopy( *Te, &TTe );
    }

    if( bfl EQ NL_TOP )
    {
        if( Ds EQ NULL OR Ts EQ NULL )
        {
            error = N_SrfDerivs( sur, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( Ds EQ NULL )
            N_VectorCopy( SD[0][1], &DDs );
        else
            N_VectorCopy( *Ds, &DDs );

        if( Ts EQ NULL )
            N_VectorCopy( SD[1][1], &TTs );
        else
            N_VectorCopy( *Ts, &TTs );

        if( De EQ NULL OR Te EQ NULL )
        {
            error = N_SrfDerivs( sur, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( De EQ NULL )
            N_VectorCopy( SD[0][1], &DDe );
        else
            N_VectorCopy( *De, &DDe );

        if( Te EQ NULL )
            N_VectorCopy( SD[1][1], &TTe );
        else
            N_VectorCopy( *Te, &TTe );
    }

    /* Handle special cases */

    if( n LE 3 )
    {
        if( p EQ 3 )
        {
            error = N_FitHermite( DDs, DDe, TTs, TTe, der, SG );

            if( error EQ NL_YES )
                NL_OUT;

            NL_OUT;
        }

        if( p EQ 2 )
        {
            error = N_CrvSizeArrays( der, 3, 2, 6, rname, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPtsDegreeAndKnots( der, &i, &Pw, &p, &j, &U2 );

            U2[0] = U2[1] = U2[2] = U[0];
            U2[4] = U2[5] = U2[6] = U[m];

            if( n EQ 2 )
            {
                U2[3] = 0.5 *( U[0] + U[m] );

                fs = fe = 0.5 *( U2[3] - U2[0] );
            }
            else if( n EQ 3 )
            {
                U2[3] = U[3];

                fs = 0.5 *( U[3] - U[0] );
                fe = 0.5 *( U[m] - U[3] );
            }

            N_PtToCPt( DDs, &Pw[0] );
            N_PtToCPt( DDe, &Pw[3] );

            N_VectorPtAlongVector( DDs, fs, TTs, &Q1 );
            N_VectorPtAlongVector( DDe, -fs, TTe, &Q2 );

            N_PtToCPt( Q1, &Pw[1] );
            N_PtToCPt( Q2, &Pw[2] );
        }

        NL_OUT;
    }

    /* Handle the general case */

    P = N_AllocPt1dArray( n - 2, &SL );

    if( P EQ NULL )
        NL_QUIT;

    t = N_AllocReal1dArray( n - 2, &SL );

    if( t EQ NULL )
        NL_QUIT;

    t[0] = U[0];
    t[n - 2] = U[m];
    op = 1.0 / p;

    for ( i = 2; i <= n - 2; i++ )
    {
        sum = 0.0;

        for ( j = i + 1; j <= i + p; j++ )
            sum += U[j];
        t[i - 1] = op * sum;
    }

    N_VectorCopy( DDs, &P[0] );
    N_VectorCopy( DDe, &P[n - 2] );

    for ( i = 1; i <= n - 3; i++ )
    {
        error = N_CrvEval( &cur, t[i], NL_LEFT, &P[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    Vs = &TTs;
    Ve = &TTe;

    error = N_FitCrvKnotsAndDerivs( (NL_VOID *)P, n - 2, NL_EPOINT, t, knt, p, (NL_VOID *)Vs, (NL_VOID *)Ve, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateDerivField */
