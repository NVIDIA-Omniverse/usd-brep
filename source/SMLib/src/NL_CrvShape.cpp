// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvShape.c: Shape curve routines                                   */
/**********************************************************************/

#include "StdAfx.h"

#if NLIB_UNUSED

#include <nurbs.h>
#include <NL_Globals.h>

static NL_FLAG ST_refine( NL_CURVE *, NL_LINESEG *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER *, NL_FLAG * );

/**********************************************************************/
/* N_CrvShapeApproxPts: Shape curve to approximate given points                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping  routine  modifies the shape of a given base curve to
     approximate a  set of points. It performs  constrained  shaping to
     produce an approximating curve. A typical calling example is:

       NL_FLAG    ftl;
       NL_CURVE   curB, curS;
       NL_POINT   *P;
       NL_REAL    tol;
       NL_INDEX   np, ds, de;
       NL_STACKS  SG;
       ...
       (get base curve curB, points P, and tol);
       ...
       N_CrvInitArrays(&curS);
       N_CrvShapeApproxPts(&curB,P,np,ds,de,tol,&curS,&ftl,&SG);

     MEMORY FOR THE  NL_CURVE  STRUCTURE OF curS  MUST BE ALLOCATED IN THE
     CALLING ROUTINE (AS SHOWN  IN THE  EXAMPLE ABOVE). THIS VERSION OF 
     THE METHOD  COMPUTES THE NL_PARAMETERS  ONLY ONCE  BY  PROJECTING THE 
     NL_POINTS ONTO THE BASE NL_CURVE. PRO: IF THE BASE NL_CURVE IS WELL CHOSEN,
     THEN GOOD NL_PARAMETERS ARE NL_USED AND WHILE THE PROCESS ITERATES, THEY
     ARE NOT DESTROYED BY POSSIBLE WIGGLY APPROXIMANT. CON: IF THE BASE
     NL_CURVE IS NOT WELL CHOSEN THEN THE NL_PARAMETERS ARE NOT  SATISFACTORY
     FOR LEAST-SQAURES FITTING.


   ACCESS:
   
     curB , input  ,  Base curve
     P    , input  ,  Random points output curve must approximate
     np   , input  ,  Highest index in P
     ds   , input  ,  Start derivative constraint;  0,..,ds derivatives
                      not to change at the start
     de   , input  ,  End  derivative  constraint;  0,..,de derivatives 
                      not to change at the end
     tol  , input  ,  The filtered  average error of the  approximation
                      must be less than tol. The shaping iteration will
                      stop when:
                      1. the average error is less than tol;
                      2. there is  no  improvement in  two  consecutive
                         iterates; or
                      3. the error gets worse
     curS , output ,  Shaped curve approximating P[i], i=0,...,np
     ftl  , output ,  Flag:
                        NL_YES: approximant is within tol
                        NL_NO : error condition is not  satisfied, however, 
                             the best curve is returned
     SG   , input  ,  curS' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeApproxPts( NL_CURVE *curB, NL_POINT *P, NL_INDEX np, NL_INDEX ds, NL_INDEX de, NL_REAL tol, NL_CURVE *curS, NL_FLAG *ftl, NL_STACKS *SG )
{
    NL_FLAG fin, error = NL_NO;

    NL_INDEX ** K, *I, *nd, *id, nq, i, j, m, n, dn, nI, ni, lo, hi, nf;

    NL_DEGREE p;

    NL_REAL *U, *up, *uq, *us, *ui, *d, *eps, *z, *zv, ltl, dis, sum, emax, emp, emc, eav, op;

    NL_POINT *Q, R;

    NL_VECTOR ** DD;

    NL_KNOTVECTOR knx;

    NL_RMATRIX rma;

    NL_CURVE curA, curI;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL dtl = 0.001;
    NL_PRIVATE NL_REAL efp = 0.05;
    NL_PRIVATE NL_REAL cie = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Prepare for shaping and get local notation */

    N_CrvGetBBoxMaxDiagDist( curB, &dis );
    ltl = dis * dtl;

    N_CrvInitArrays( &curI );
    error = N_CrvCopy( curB, &curI, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curB, &m, &U );
    N_CrvGetArraySizes( curB, &n, &m );
    N_CrvGetDegree( curB, &p );

    nI = n + np;
    op = 1.0 / p;
    dn = NL_MAX( 1, n / 2 );
    nf = NL_MAX( 1, (NL_INDEX)(efp * np) );
    *ftl = NL_NO;
    emc = NL_BIGD;
    emp = NL_BIGD;
    fin = NL_NO;

    /* Allocate memory */

    I = N_AllocInt1dArray( nI, &SL );

    if( I EQ NULL )
        NL_QUIT;

    K = N_AllocInt2dArray( np, 0, &SL );

    if( K EQ NULL )
        NL_QUIT;

    nd = N_AllocInt1dArray( np, &SL );

    if( nd EQ NULL )
        NL_QUIT;

    id = N_AllocInt1dArray( nI, &SL );

    if( id EQ NULL )
        NL_QUIT;

    DD = N_AllocPt2dArray( np, 0, &SL );

    if( DD EQ NULL )
        NL_QUIT;

    up = N_AllocReal1dArray( np, &SL );

    if( up EQ NULL )
        NL_QUIT;

    us = N_AllocReal1dArray( np, &SL );

    if( us EQ NULL )
        NL_QUIT;

    d = N_AllocReal1dArray( np, &SL );

    if( d EQ NULL )
        NL_QUIT;

    eps = N_AllocReal1dArray( np, &SL );

    if( eps EQ NULL )
        NL_QUIT;

    ui = N_AllocReal1dArray( np, &SL );

    if( ui EQ NULL )
        NL_QUIT;

    z = N_AllocReal1dArray( nI, &SL );

    if( z EQ NULL )
        NL_QUIT;

    zv = N_AllocReal1dArray( nI, &SL );

    if( zv EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= np; i++ )
    {
        K[i][0] = 0;
        nd[i] = 0;
    }

    /* Get the parametrization and the difference vectors */

    error = N_ApproxCrvWithPolyline( curB, ltl, NL_ABSOLUTE, NL_BOTH, &Q, &uq, &nq, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= np; i++ )
    {
        error = N_ProjectPtClosePolyPt( Q, uq, nq, P[i], &R, &up[i], &d[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= np; i++ )
    {
        N_CrvEval( curB, up[i], NL_LEFT, &R );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( P[i], R, &DD[i][0] );
    }

    /*********************/
    /* Iteratively shape */
    /*********************/

    /* While tolerance requirement is not satisfied do */

    while( 1 )
    {
        /* Shape base curve */

        for ( j = 0; j <= ds; j++ )
            I[j] = 0;

        for ( j = 0; j <= de; j++ )
            I[n - j] = 0;

        for ( j = ds + 1; j <= n - de - 1; j++ )
            I[j] = 1;

        N_CrvInitArrays( &curA );
        error = N_CrvCopy( &curI, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvShapeDerivConstraintsOver( &curA, up, np, I, DD, K, nd, NL_PREPARE, &rma, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Compute filtered average error */

        error = N_ApproxCrvWithPolyline( &curA, ltl, NL_ABSOLUTE, NL_BOTH, &Q, &uq, &nq, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= np; i++ )
        {
            error = N_ProjectPtClosePolyPt( Q, uq, nq, P[i], &R, &dis, &d[i] );

            if( error EQ NL_YES )
                NL_OUT;

            us[i] = up[i];
        }

        emax = eav = 0.0;

        for ( i = 0; i <= np; i++ )
        {
            lo = NL_MAX( 0, i - nf );
            hi = NL_MIN( np, i + nf );

            sum = 0.0;

            for ( j = lo; j <= hi; j++ )
                sum += d[j];
            eps[i] = sum / ((NL_REAL)hi - (NL_REAL)lo + 1.0);
            eav += eps[i];

            if( eps[i]GT emax )
                emax = eps[i];
        }

        emc = emax;
        eav /= (np + 1.0);

        if( emax LT tol )
        {
            fin = NL_YES;
            *ftl = NL_YES;
        }
        else if( emp NEQ NL_BIGD )
        {
            dis = (fabs( emc - emp )) / eav;

            if( dis LT cie )
                fin = NL_YES;

            if( emc GT emp )
                fin = NL_YES;
        }

        if( fin EQ NL_YES )
        {
            N_CrvInitArrays( curS );
            error = N_CrvCopy( &curA, curS, SG );

            if( error EQ NL_YES )
                NL_OUT;

            break;
        }

        /* Add extra knots */

        j = NL_MIN( n + dn, (n + np) / 2 );
        ni = j - n;

        /* Get Greville abscissae */

        for ( i = 0; i <= n; i++ )
        {
            sum = 0.0;

            for ( j = i + 1; j <= i + p; j++ )
                sum += U[j];
            z[i] = op * sum;
        }

        /* Sort error vector */

        N_SortRealRealArrays( eps, us, np );

        /* Add priority to each Greville span based on highest errors */

        for ( i = 0; i <= n; i++ )
        {
            zv[i] = 0.0;
            id[i] = i;
        }

        for ( i = 0; i <= np; i++ )
        {
            /* Find interval parameter is in */

            error = N_FindIntervalRealArray( z, n, us[i], NL_LEFT, &j );

            if( error EQ NL_YES )
                NL_OUT;

            /* Increment priority */

            dis = ((NL_REAL)np - (NL_REAL)i) / ((NL_REAL)np);
            zv[j] += dis;
        }

        /* Sort intervals based on priority */

        N_SortRealIndexArrays( zv, id, n - 1 );

        /* Find knots to be inserted */

        hi = NL_MIN( ni - 1, n - 1 );

        for ( i = n - 1; i >= n - hi - 1; i-- )
        {
            j = id[i];
            ui[n - i - 1] = 0.5 *( U[j + 1] + U[j + p + 1] );
        }

        N_ShellSortReal( ui, hi );

        /* Insert knots */

        N_KnotVectorFromRealArray( &knx, ui, hi );

        error = N_CrvRefine( &curI, &knx, &curI, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnots( &curI, &m, &U );
        N_CrvGetArraySizes( &curI, &n, &m );

        if( n GT nI )
        {
            j = NL_MAX( n + nI, nI + nI );

            error = N_Realloc1dIntArray( &I, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dIntArray( &id, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &z, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &zv, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            nI = j;
        }

        emp = emc;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeApproxPtsUpdate: Shape curve to approximate given points                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping  routine  modifies the shape of a given base curve to
     approximate a  set of points. It performs  constrained  shaping to
     produce an approximating curve and at the same time it updates the
     parameters as the quality of the approximation improves. A typical 
     calling example is:

       NL_FLAG    ftl;
       NL_CURVE   curB, curS;
       NL_POINT   *P;
       NL_REAL    tol;
       NL_INDEX   np, ds, de;
       NL_STACKS  SG;
       ...
       (get base curve curB, points P and tol);
       ...
       N_CrvInitArrays(&curS);
       N_CrvShapeApproxPtsUpdate(&curB,P,np,ds,de,tol,&curS,&ftl,&SG);

     MEMORY FOR THE  NL_CURVE  STRUCTURE OF curS  MUST BE ALLOCATED IN THE
     CALLING  ROUTINE (AS SHOWN IN THE  EXAMPLE ABOVE). THIS VERSION OF
     THE METHOD  CONSTANTLY UPDATES THE NL_PARAMETERS AS THE APPROXIMATION
     IMPORVES.  PRO: FOR  GOOD  APPROXIMATION   BETTER  NL_PARAMETERS  ARE 
     OBTAINED.  CON: IF THE  APPROXIMATION  WORSENS OR  BEGINS  TO SHOW 
     WIGGLES, THE PARAMETRIZATION WORSENS AS WELL.


   ACCESS:
   
     curB , input  ,  Base curve
     P    , input  ,  Random points output curve must approximate
     np   , input  ,  Highest index in P
     ds   , input  ,  Start derivative constraint;  0,..,ds derivatives
                      not to change at the start
     de   , input  ,  End  derivative  constraint;  0,..,de derivatives 
                      not to change at the end
     tol  , input  ,  The filtered  average error of the  approximation
                      must be less than tol. The shaping iteration will
                      stop when:
                      1. the average error is less than tol;
                      2. there is  no  improvement in  two  consecutive
                         iterates; or
                      3. the error gets worse
     curS , output ,  Shaped curve approximating P[i], i=0,...,np
     ftl  , output ,  Flag:
                        NL_YES: approximant is within tol
                        NL_NO : error condition is not  satisfied, however, 
                             the best curve is returned
     SG   , input  ,  curS' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeApproxPtsUpdate( NL_CURVE *curB, NL_POINT *P, NL_INDEX np, NL_INDEX ds, NL_INDEX de, NL_REAL tol, NL_CURVE *curS, NL_FLAG *ftl, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX ** K, *I, *nd, *id, nq, i, j, m, n, dn, nI, ni, lo, hi, nf;

    NL_DEGREE p;

    NL_REAL *U, *up, *uq, *us, *ui, *d, *eps, *z, *zv, ltl, dis, sum, emax, emp, emc, eav = 0.0, op;

    NL_POINT *Q, R;

    NL_VECTOR ** DD;

    NL_KNOTVECTOR knx;

    NL_RMATRIX rma;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL dtl = 0.001;
    NL_PRIVATE NL_REAL efp = 0.05;
    NL_PRIVATE NL_REAL cie = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Prepare for shaping and get local notation */

    N_CrvGetBBoxMaxDiagDist( curB, &dis );
    ltl = dis * dtl;

    error = N_CrvCopy( curB, curS, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curS, &m, &U );
    N_CrvGetArraySizes( curS, &n, &m );
    N_CrvGetDegree( curS, &p );

    nI = n + np;
    op = 1.0 / p;
    dn = NL_MAX( 1, n / 2 );
    nf = NL_MAX( 1, (NL_INDEX)(efp * np) );
    *ftl = NL_NO;
    emc = NL_BIGD;
    emp = NL_BIGD;

    /* Allocate memory */

    I = N_AllocInt1dArray( nI, &SL );

    if( I EQ NULL )
        NL_QUIT;

    K = N_AllocInt2dArray( np, 0, &SL );

    if( K EQ NULL )
        NL_QUIT;

    nd = N_AllocInt1dArray( np, &SL );

    if( nd EQ NULL )
        NL_QUIT;

    id = N_AllocInt1dArray( nI, &SL );

    if( id EQ NULL )
        NL_QUIT;

    DD = N_AllocPt2dArray( np, 0, &SL );

    if( DD EQ NULL )
        NL_QUIT;

    up = N_AllocReal1dArray( np, &SL );

    if( up EQ NULL )
        NL_QUIT;

    us = N_AllocReal1dArray( np, &SL );

    if( us EQ NULL )
        NL_QUIT;

    d = N_AllocReal1dArray( np, &SL );

    if( d EQ NULL )
        NL_QUIT;

    eps = N_AllocReal1dArray( np, &SL );

    if( eps EQ NULL )
        NL_QUIT;

    ui = N_AllocReal1dArray( np, &SL );

    if( ui EQ NULL )
        NL_QUIT;

    z = N_AllocReal1dArray( nI, &SL );

    if( z EQ NULL )
        NL_QUIT;

    zv = N_AllocReal1dArray( nI, &SL );

    if( zv EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= np; i++ )
    {
        K[i][0] = 0;
        nd[i] = 0;
    }

    /*********************/
    /* Iteratively shape */
    /*********************/

    /* While tolerance requirement is not satisfied do */

    while( 1 )
    {
        /* Get new parameters */

        error = N_ApproxCrvWithPolyline( curS, ltl, NL_ABSOLUTE, NL_BOTH, &Q, &uq, &nq, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= np; i++ )
        {
            error = N_ProjectPtClosePolyPt( Q, uq, nq, P[i], &R, &up[i], &d[i] );

            if( error EQ NL_YES )
                NL_OUT;

            us[i] = up[i];
        }

        /* Set up for shaping */

        for ( i = 0; i <= np; i++ )
        {
            N_CrvEval( curS, up[i], NL_LEFT, &R );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDiff( P[i], R, &DD[i][0] );
        }

        for ( j = 0; j <= ds; j++ )
            I[j] = 0;

        for ( j = 0; j <= de; j++ )
            I[n - j] = 0;

        for ( j = ds + 1; j <= n - de - 1; j++ )
            I[j] = 1;

        /* Shape curve now */

        error = N_CrvShapeDerivConstraintsOver( curS, up, np, I, DD, K, nd, NL_PREPARE, &rma, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Compute filtered average error */

        emax = 0.0;

        for ( i = 0; i <= np; i++ )
        {
            lo = NL_MAX( 0, i - nf );
            hi = NL_MIN( np, i + nf );

            sum = 0.0;

            for ( j = lo; j <= hi; j++ )
                sum += d[j];
            eps[i] = sum / ((NL_REAL)hi - (NL_REAL)lo + 1.0);
            eav += eps[i];

            if( eps[i]GT emax )
                emax = eps[i];
        }

        emc = emax;
        eav /= ((NL_REAL)np + 1.0);

        if( emax LT tol )
        {
            *ftl = NL_YES;
            break;
        }

        if( emp NEQ NL_BIGD )
        {
            dis = (fabs( emc - emp )) / eav;

            if( dis LT cie OR emc GT emp )
                break;
        }

        /* Add extra knots */

        j = NL_MIN( n + dn, (n + np) / 2 );
        ni = j - n;

        /* Get Greville abscissae */

        for ( i = 0; i <= n; i++ )
        {
            sum = 0.0;

            for ( j = i + 1; j <= i + p; j++ )
                sum += U[j];
            z[i] = op * sum;
        }

        /* Sort error vector */

        N_SortRealRealArrays( eps, us, np );

        /* Add priority to each Greville span based on highest errors */

        for ( i = 0; i <= n; i++ )
        {
            zv[i] = 0.0;
            id[i] = i;
        }

        for ( i = 0; i <= np; i++ )
        {
            /* Find interval parameter is in */

            error = N_FindIntervalRealArray( z, n, us[i], NL_LEFT, &j );

            if( error EQ NL_YES )
                NL_OUT;

            /* Increment priority */

            dis = (((NL_REAL)np - (NL_REAL)i)) / ((NL_REAL)np);
            zv[j] += dis;
        }

        /* Sort intervals based on priority */

        N_SortRealIndexArrays( zv, id, n - 1 );

        /* Find knots to be inserted */

        hi = NL_MIN( ni - 1, n - 1 );

        for ( i = n - 1; i >= n - hi - 1; i-- )
        {
            j = id[i];
            ui[n - i - 1] = 0.5 *( U[j + 1] + U[j + p + 1] );
        }

        N_ShellSortReal( ui, hi );

        /* Insert knots */

        N_KnotVectorFromRealArray( &knx, ui, hi );

        error = N_CrvRefine( curS, &knx, curS, SG, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnots( curS, &m, &U );
        N_CrvGetArraySizes( curS, &n, &m );

        if( n GT nI )
        {
            j = NL_MAX( n + nI, nI + nI );

            error = N_Realloc1dIntArray( &I, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dIntArray( &id, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &z, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &zv, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            nI = j;
        }

        emp = emc;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CRVSHAPEAXIALDEFORM: Axial deformations of curves                             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:


     This shaping routine modifies the  shape of a curve by applying the  
     following axial deformations:

       NL_PINCH: {x|y|z} coordinate is scaled according to s({x|y|z})
       NL_TAPER: P is scaled according to s({x|y|z})
       NL_TWIST: P is rotated according to s({x|y|z})
       NL_SHEAR: {x|y|z} coordinate is translated according to s({x|y|z})

     where {x|y|z} means x, y or z coordinate, s({x|y|z}) is a function 
     of either  x, y or z, and P is a non-weighted control point. Also, 
     if the point P is scaled according to, say, s(y), then  only its x 
     and z  coordinates  change. The  scaling  function is  given  as a  
     B-spline function. A typical calling example is:
  
       NL_CURVE  cur;
       NL_REAL   a;
       NL_CFUN   cfn;
       ...
       (get cur, cfn and a);
       ...
       N_CrvShapeAxialDeform(&cur,&cfn,a,NL_TAPER,NL_XDIR,NL_YCRD);

     The  transformation is performed  in-place, i.e. the original curve
     is destroyed.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     cfn , input  ,  Shape function
     a   , input  ,  Scalar factor (amplitude)
     tra , input  ,  Flag:
                       NL_PINCH: curve is pinched
                       NL_TAPER: curve is tapered
                       NL_TWIST: curve is twisted
                       NL_SHEAR: curve is sheared
     dir , input  ,  Flag:
                       NL_XDIR: shape in the x-direction
                       NL_YDIR: shape in the y-direction
                       NL_ZDIR: shape in the z-direction
     cor , input  ,  Flag:
                       NL_XCRD: change x coordinate
                       NL_YCRD: change y coordinate
                       NL_ZCRD: change z coordinate


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeAxialDeform( NL_CURVE *cur, NL_CFUN *cfn, NL_REAL a, NL_FLAG tra, NL_FLAG dir, NL_FLAG cor )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, n;

    NL_REAL w;

    NL_POINT P;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notations */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Shape curve */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtAndW( Pw[i], &P, &w );

        error = N_TransformPtWithShapeFuncAndScale( &P, cfn, a, tra, dir, cor );

        if( error EQ NL_YES )
            NL_OUT;

        N_Weight( P, w, &Pw[i] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeBend: Bend a NURBS curve                                 */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine bends a NURBS curve toward a bend curve from a 
     bend center. The bend curve must be strictly concave from the view-
     point of the bend center which must be placed so that semi-infinite
     rays passing through it  intersect the bend curve only once. Only a 
     region  of the  curve that  is defined over  [us,ue]  is bent. When 
     interactive   change  is  required,  only   those  quantities   are 
     recomputed  that  directly affect the change in the bend. A typical 
     calling example is:

       NL_CURVE      curP, curB, curQ;
       NL_INDEX      npo, si, ei;
       NL_PARAMETER  us, ue;
       NL_POINT      O;
       NL_REAL       lam, tol;
       NL_STACKS     SP, SQ;
       ...
       (define curP; get us, ue, npo; get O, curB and lam; get tol);
       ...
       N_CrvShapeBend(&curP,us,ue,npo,&curB,O,lam,tol,NL_PREPARE,NL_YES,&si,&ei,
                &curQ,&SP,&SQ);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_CrvShapeBend");
...
         get new lam;
         N_CrvShapeBend(&curP,us,ue,npo,&curB,O,lam,tol,NL_INTERACT,NL_NO,&si,&ei,
                  &curQ,&SP,&SQ);
       }
       N_CrvShapeBend(&curP,us,ue,npo,&curB,O,lam,tol,NL_CLEANUP,NL_NO,&si,&ei,
                &curQ,&SP,&SQ);

     THE BENT NL_CURVE IS STORED IN curQ. ALTHOUGH THE INPUT NL_CURVE DOES NOT 
     CHANGE  EITHER  GEOMETIRICALLY OR PARAMETRICALLY, ITS DEFINITION IS  
     DESTROYED  IN THAT  ITS KNOT  NL_VECTOR IS  REFINED AND  ITS NL_DEGREE IS 
     RAISED IF IT  WAS A NL_LINEAR  NL_CURVE. IF THIS IS  UNACCEPTABLE, EITHER 
     SAVE THE ORIGINAL NL_CURVE OR  APPLY KNOT REMOVAL AND NL_DEGREE REDUCTION 
     AFTER  BENDING IS  COMPLETED. THE  NL_CURVE  CAN  BE REFINED  PRIOR TO 
     SHAPING IN WHICH CASE THE ROUTINE SKIPS REFINEMENT.


   ACCESS:      
   
     curP  , in/out ,  NURBS curve
     us,ue , input  ,  Bend curve over [us,ue]
     npo   , input  ,  Number of  knots to be  inserted. Depends  on the
                       bend; for large bends,  more  control  points are  
                       needed to  obtain a smooth  change in  shape. The 
                       default is  ns*(p+1),  where ns is  the number of
                       knot spans in [us,ue].
     curB  , input  ,  Bend curve
     O     , input  ,  Bend center
     lam   , input  ,  Cross ratio for bending. Some special values: 
                         = 1.0: maps poins onto curB
                         = 0.0: maps points to O
                         = INF: maps points to curP
     tol   , inpout ,  Knot removal tolerance. For  shaping accuracy, 1%
                       of the curve's size is a good default
     flg   , input  ,  Flag:
                         NL_PREPARE : get some entities and do first bend
                         NL_INTERACT: change bend by changing lam
                         NL_CLEANUP : remove unneccesary knots
     ref   , input  ,  Flag:
                         NL_YES: refine curve
                         NL_NO : curve is already refined
     si,ei , in/out ,  Indexes of local control points
     curQ  , output ,  Bent curve; if curP = curQ, only the NL_PREPARE  and 
                       NL_CLEANUP options are available
     SP    , input  ,  curP's stack
     SQ    , input  ,  curQ's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeBend( NL_CURVE *curP, NL_PARAMETER us, NL_PARAMETER ue, NL_INDEX npo, NL_CURVE *curB, NL_POINT O, NL_REAL lam, NL_REAL tol, NL_FLAG flg, NL_FLAG ref, NL_INDEX *si, NL_INDEX *ei, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeBend");

    NL_FLAG its, par = NL_YES, error = NL_NO;

    NL_INDEX i, n, m, s = 0, e = 0, nkt, nkx, fk, lk;

    NL_DEGREE p;

    NL_REAL *U, ug, uc, ltl, d, sb, tb, ts, te, w;

    NL_POINT T[2], D[2], Q, R;

    NL_VECTOR V, B, M, N;

    NL_CPOINT *Pw, *Qw = NULL;

    NL_KNOTVECTOR *knt = NULL, *knx;

    NL_LINESEG lin, tnP, tnB;

    NL_RMATRIX rma;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL itl = 1.0e-04;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    ug = 0.0; /* to avoid warning on some systems */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &U );

    if( us GE ue )
        NL_ERROR( NL_INP_ERR );

    if( us GE U[m]OR ue LE U[0] )
        NL_ERROR( NL_INP_ERR );

    if( us LT U[0] )
        us = U[0];

    if( ue GT U[m] )
        ue = U[m];

    switch( flg )
    {
        case NL_PREPARE:

            N_CrvGetKnotVector( curP, &knt );
            break;

        case NL_INTERACT:
            if( curP EQ curQ )
                NL_ERROR( NL_INP_ERR );

            s = *si;
            e = *ei;
            N_CrvGetCPts( curQ, &n, &Qw );
            break;

        case NL_CLEANUP:

            s = *si;
            e = *ei;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Adjust intersection tolerance */

    error = N_CrvGetBBoxMaxDiagDist( curP, &d );

    if( error EQ NL_YES )
        NL_OUT;

    ltl = d * itl;

    /* Prepare for bend */

    if( flg EQ NL_PREPARE )
    {
        /* Degree elevate if required */

        if( p EQ 1 )
        {
            error = N_CrvElevateDegree( curP, 1, curP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &U );
            N_CrvGetKnotVector( curP, &knt );
        }

        /* Refine curve */

        if( ref EQ NL_YES )
        {
            error = N_BasisFindSpan( knt, p, us, NL_LEFT, &s );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpan( knt, p, ue, NL_LEFT, &e );

            if( error EQ NL_YES )
                NL_OUT;

            nkt = (e - s + 1) * (p + 1);
            nkt = NL_MAX( npo, nkt );
            nkx = nkt + 2;

            knx = N_AllocKnotVectorAndArray( nkx, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            error = N_BasisSplitNLongestSpans( knt, p, us, ue, nkt, 1, 1, knx );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvRefine( curP, knx, curP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &U );
            N_CrvGetKnotVector( curP, &knt );
        }

        if( curP NEQ curQ )
        {
            N_CrvInitArrays( curQ );
            error = N_CrvCopy( curP, curQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPts( curQ, &n, &Qw );
        }
        else
        {
            N_CrvGetCPts( curP, &n, &Qw );
        }

        /* Get indexes */

        error = N_BasisFindSpan( knt, p, us, NL_LEFT, &s );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knt, p, ue, NL_LEFT, &e );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
            s = 0;

        if( ue EQ U[m] )
            e = n + p + 1;

        /* Output indexes for interactive step */

        *si = s;
        *ei = e;
    }

    /* Interactively bend */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Recover original control points */

        if( curP NEQ curQ )
        {
            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Pw[i], &Qw[i] );
        }

        /* Reposition control points for bend region */

        N_CrvGetKnots( curB, &m, &U );

        error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = s; i < e - p; i++ )
        {
            N_CPtToPtAndW( Qw[i], &Q, &w );

            N_CreateLinePtPt( &lin, O, Q, NL_UNBOUNDED );

            error = N_IsectCrvShapeLineCrv( curB, &lin, U[0], U[m], ug, par, ltl, &R, &uc, &its );

            if( error EQ NL_YES OR its EQ NL_FALSE )
                NL_OUT;

            N_DistPtPt( O, R, &sb );
            N_DistPtPt( O, Q, &tb );

            if( tb LT NL_MTOL )
                NL_ERROR( NL_INP_ERR );

            sb = sb / tb;
            tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);

            N_Combine2Pts( 1.0 - tb, O, tb, Q, &Q );
            N_Weight( Q, w, &Qw[i] );

            ug = uc;
            par = NL_NO;
        }

        /* Reattach control points on the left */

        if( s GT 0 )
        {
            error = N_CrvDerivs( curP, us, NL_LEFT, 1, T );

            if( error EQ NL_YES )
                NL_OUT;

            N_CreateLinePtPt( &lin, O, T[0], NL_UNBOUNDED );

            error = N_IsectCrvShapeLineCrv( curB, &lin, U[0], U[m], ug, NL_YES, ltl, &R, &ts, &its );

            if( error EQ NL_YES OR its EQ NL_FALSE )
                NL_OUT;

            error = N_CrvDerivs( curB, ts, NL_LEFT, 1, D );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( O, D[0], &sb );
            N_DistPtPt( O, T[0], &tb );

            if( tb LT NL_MTOL )
                NL_ERROR( NL_INP_ERR );

            sb = sb / tb;
            tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);
            N_Combine2Pts( 1.0 - tb, O, tb, T[0], &R );

            N_CreateLineStartDirVector( &tnP, T[0], T[1], NL_UNBOUNDED );
            N_CreateLineStartDirVector( &tnB, D[0], D[1], NL_UNBOUNDED );

            error = N_IsectLineLine( tnP, tnB, &Q, &sb, &tb, &its );

            if( error EQ NL_YES )
                NL_OUT;

            if( its EQ NL_TRUE )
            {
                N_VectorDiff( R, Q, &V );
                N_VectorDiff( T[0], O, &B );
                N_VectorCross( B, T[1], &N );
                N_VectorCross( B, V, &M );

                if( NOT N_VectorsAreParallel( N, M ) )
                    N_VectorReverseInPlace( &V );
            }
            else
            {
                N_VectorCopy( T[1], &V );
            }

            error = N_VectorNormalizeRef( &T[1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &V );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CreateTransformMatrixFromVectors( T[0], T[1], R, V, &rma, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i < s; i++ )
                N_TransformCPt( Qw[i], &rma, &Qw[i] );
        }

        /* Reattach control points on the right */

        if( e LT n + p + 1 )
        {
            error = N_CrvDerivs( curP, ue, NL_LEFT, 1, T );

            if( error EQ NL_YES )
                NL_OUT;

            N_CreateLinePtPt( &lin, O, T[0], NL_UNBOUNDED );

            error = N_IsectCrvShapeLineCrv( curB, &lin, U[0], U[m], ug, NL_YES, ltl, &R, &te, &its );

            if( error EQ NL_YES OR its EQ NL_FALSE )
                NL_OUT;

            error = N_CrvDerivs( curB, te, NL_LEFT, 1, D );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( O, D[0], &sb );
            N_DistPtPt( O, T[0], &tb );

            if( tb LT NL_MTOL )
                NL_ERROR( NL_INP_ERR );

            sb = sb / tb;
            tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);
            N_Combine2Pts( 1.0 - tb, O, tb, T[0], &R );

            N_CreateLineStartDirVector( &tnP, T[0], T[1], NL_UNBOUNDED );
            N_CreateLineStartDirVector( &tnB, D[0], D[1], NL_UNBOUNDED );

            error = N_IsectLineLine( tnP, tnB, &Q, &sb, &tb, &its );

            if( error EQ NL_YES )
                NL_OUT;

            if( its EQ NL_TRUE )
            {
                N_VectorDiff( R, Q, &V );
                N_VectorDiff( T[0], O, &B );
                N_VectorCross( B, T[1], &N );
                N_VectorCross( B, V, &M );

                if( NOT N_VectorsAreParallel( N, M ) )
                    N_VectorReverseInPlace( &V );
            }
            else
            {
                N_VectorCopy( T[1], &V );
            }

            error = N_VectorNormalizeRef( &T[1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &V );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CreateTransformMatrixFromVectors( T[0], T[1], R, V, &rma, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = e - p; i <= n; i++ )
                N_TransformCPt( Qw[i], &rma, &Qw[i] );
        }
    }

    /* Remove unnecessary knots */

    if( flg EQ NL_CLEANUP )
    {
        if( s GE p )
            fk = s + (p + 1) / 2;
        else
            fk = p + 1;

        if( e LE n )
            lk = e - (p + 2) / 2;
        else
            lk = n;

        error = N_CrvShapeRemoveKnots( curQ, tol, fk, lk, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check dimensionality */

    if( NOT N_CrvIs3d( curP ) )
        N_Crv3dTo2d( curQ );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeDerivConstraintsOver: Constraint-based curve shaping with int. or appr.        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies the  shape of a curve by satisfying a
     set of  derivative  constraints, or  approximating  them  with  the 
     minimum  amount of  change in the  position of  control points. The 
     derivative constraints are given at selected parameter values. When 
     interactive shaping is required, only those quantities are computed 
     that  directly  affect the  change in the  curve's shape. A typical 
     calling example is:

       NL_CURVE      cur;
       NL_INDEX      **K, *I, *nd, nu; 
       NL_PARAMETER  *u;
       NL_VECTOR     **DD;
       NL_RMATRIX    rma;
       NL_STACKS     SM;
       ...
       (define cur; get u, I, DD, K and nd);
       ...
       N_CrvShapeDerivConstraintsOver(&cur,u,nu,I,DD,K,nd,NL_PREPARE,&rma,&SM);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_CrvShapeDerivConstraintsOver");
...
         get new DD;
         N_CrvShapeDerivConstraintsOver(&cur,u,nu,I,DD,K,nd,NL_INTERACT,&rma,&SM);
       }

     THE SHAPING IS DONE IN-PLACE, I.E. THE ORIGINAL NL_CURVE IS DESTROYED.
     THE NL_CURVE IS UPDATED EITHER PRECISELY OR APPROXIMATELY. THAT IS, IF
     THE  SYSTEM IS  FULLY OR UNDER-CONSTRAINED, THE  CONDITIONS ARE MET
     PRECISELY. IF THE  SYSTEM IS  NL_OVER-CONSTRAINED, THE  CONDITIONS ARE 
     MET APPROXIMATELY IN THE LEAST-SQUARES SENSE. 


   ACCESS:
   
     cur  , in/out ,  NURBS curve
     u    , input  ,  Parameters where constraints are assumed
     nu   , input  ,  Highest index in u and nd
     I    , input  ,  Array: I[k] = 
                        1 : Pw[k] IS to change
                        0 : Pw[k] is NOT to change
     DD   , input  ,  Derivative differences (constraints):  DD[i][j] is
                      the j-th constraint  assumed at u[i].  DD does not
                      have to  be a  2-D array. It  can be  an  array of 
                      pointers pointing  to arrays of  different length.
                      THE  DERIVATIVES  ARE  ASSUMED  TO  BE  STORED  IN  
                      INCREASING ORDER. Example:
                        DD[i][0] = point (0-th derivative) change
                        DD[i][1] = second derivative change
                        DD[i][2] = fourth derivative change
     K    , input  ,  Types of  constraints:  K[i][j] specifies  that at 
                      u[i]  the  K[i][j]-th  derivative  is constrained.  
                      Example (see DD above):
                        K[i][0] = 0
                        K[i][1] = 2
                        K[i][2] = 4 
     nd   , input  ,  Highest indexes in DD and K, ie  the highest index
                      in the arrays pointed to by DD[i] or K[i] is nd[i]
     flg  , input  ,  Flag:
                        NL_PREPARE : get some entities and shape
                        NL_INTERACT: change shape by changing DD
     rma  , in/out ,  Matrix defining control point changes
     SM   , input  ,  rma's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeDerivConstraintsOver( NL_CURVE *cur, NL_PARAMETER *u, NL_INDEX nu, NL_INDEX *I, NL_VECTOR ** DD, NL_INDEX ** K, NL_INDEX *nd, NL_FLAG flg, NL_RMATRIX *rma, NL_STACKS *SM )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeDerivConstraintsOver");

    NL_FLAG twoD = NL_YES, error = NL_NO;

    NL_INDEX *O, i, j, k, l, r, s, t, n, m, spn, nb, mb, dh, row;

    NL_DEGREE p;

    NL_REAL ** ND, ** B, *U, w;

    NL_POINT *DP, P;

    NL_CPOINT *Pw;

    NL_VECTOR *DV;

    NL_KNOTVECTOR *knt;

    NL_RMATRIX rmb, rmt, rbt, rmi;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( u[0]LT U[0]OR u[nu]GT U[m] )
        NL_ERROR( NL_PAR_ERR );

    if( N_CrvIs3d( cur ) )
        twoD = NL_NO;

    switch( flg )
    {
        case NL_PREPARE:
            break;

        case NL_INTERACT:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Prepare for shaping */

    if( flg EQ NL_PREPARE )
    {
        N_CrvGetKnotVector( cur, &knt );

        /* Get offset array and adjust I array */

        O = N_AllocInt1dArray( n, &SL );

        if( O EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
            O[i] = 0;

        for ( i = 0; i <= nu; i++ )
        {
            error = N_BasisFindSpan( knt, p, u[i], NL_LEFT, &spn );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = spn - p; j <= spn; j++ )
                O[j] = 1;
        }

        mb = -1;

        for ( k = 0; k <= n; k++ )
        {
            I[k] = I[k] * O[k];

            if( I[k]EQ 1 )
                mb++;
        }

        O[0] = 0;

        for ( k = 1; k <= n; k++ )
        {
            O[k] = O[k - 1];

            if( I[k - 1]EQ 0 )
                O[k]++;
        }

        /* Get highest derivative, and number of constraints */

        nb = -1;
        dh = 0;

        for ( i = 0; i <= nu; i++ )
        {
            nb += nd[i] + 1;

            if( K[i][nd[i]]GT dh )
                dh = K[i][nd[i]];
        }

        /* Get matrix containing required derivatives */

        ND = N_AllocReal2dArray( dh, p, &SL );

        if( ND EQ NULL )
            NL_QUIT;

        error = N_SetRealMatrix( &rmb, nb, mb, NL_MT_FULL, nb, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &rmb, &B );

        for ( i = 0; i <= nb; i++ )
        {
            for ( j = 0; j <= mb; j++ )
                B[i][j] = 0.0;
        }

        row = -1;

        for ( i = 0; i <= nu; i++ )
        {
            error = N_CrvBasisDerivs( cur, u[i], NL_LEFT, K[i][nd[i]], ND, &spn );

            if( error EQ NL_YES )
                NL_OUT;

            for ( r = 0; r <= nd[i]; r++ )
            {
                row++;

                for ( s = 0; s <= p; s++ )
                {
                    j = spn - p + s;
                    t = j - O[j];

                    if( I[j]EQ 1 )
                        B[row][t] = ND[K[i][r]][s];
                }
            }
        }

        /* Compute matrix */

        if( nb EQ mb )
        {
            /* Fully-determined system -> precise solution */

            N_InitRealMatrix( rma );
            error = N_RealMatrixInversePivot( &rmb, rma, SM );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( nb LT mb )
        {
            /* Under-determined system -> minimum length precise solution */

            N_InitRealMatrix( &rmt );
            error = N_RealMatrixTranspose( &rmb, &rmt, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rbt );
            error = N_RealMatrixTransposeMultiply( &rmb, &rbt, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rmi );
            error = N_RealMatrixInversePivot( &rbt, &rmi, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( rma );
            error = N_RealMatrixMultiply( &rmt, &rmi, rma, SM );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            /* Over-determined system -> least-squares solution */

            N_InitRealMatrix( &rmt );
            error = N_RealMatrixTranspose( &rmb, &rmt, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rbt );
            error = N_RealMatrixMultiply( &rmt, &rmb, &rbt, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rmi );
            error = N_RealMatrixInversePivot( &rbt, &rmi, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( rma );
            error = N_RealMatrixMultiply( &rmi, &rmt, rma, SM );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Interactively shape */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Get right hand side of constraints */

        N_GetMaxIndexRealMatrix( rma, &mb, &nb );

        DV = N_AllocPt1dArray( nb, &SL );

        if( DV EQ NULL )
            NL_QUIT;

        k = 0;

        for ( i = 0; i <= nu; i++ )
        {
            for ( r = 0; r <= nd[i]; r++ )
            {
                N_CopyPt( DD[i][r], &DV[k] );
                k++;
            }
        }

        /* Get control point differences */

        DP = N_AllocPt1dArray( mb, &SL );

        if( DP EQ NULL )
            NL_QUIT;

        error = N_RealMatrixMultiplyPtArray( rma, DV, DP );

        if( error EQ NL_YES )
            NL_OUT;

        /* Update control points */

        l = 0;

        for ( k = 0; k <= n; k++ )
        {
            if( I[k]EQ 1 )
            {
                N_CPtToPtAndW( Pw[k], &P, &w );
                N_Sum2Pts( P, DP[l], &P );
                N_Weight( P, w, &Pw[k] );
                l++;
            }
        }
    }

    /* Check dimensionality */

    if( twoD EQ NL_YES )
        N_Crv3dTo2d( cur );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeDerivConstraints: Constraint-based curve modification                      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies the  shape of a curve by satisfying a
     set of  derivative  constraints, or  approximating  them  with  the 
     minimum  amount of  change in the  position of  control points. The 
     derivative constraints are given at selected parameter values. When 
     interactive shaping is required, only those quantities are computed 
     that  directly  affect the  change in the  curve's shape. A typical 
     calling example is:

       NL_FLAG       upd;
       NL_CURVE      cur;
       NL_INDEX      **K, *I, *nd, nu; 
       NL_PARAMETER  *u;
       NL_VECTOR     **DD;
       NL_RMATRIX    rma;
       NL_STACKS     SM;
       ...
       (define cur; get u, I, DD, K and nd);
       ...
       N_CrvShapeDerivConstraints(&cur,u,nu,I,DD,K,nd,NL_PREPARE,&rma,&upd,&SM);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_CrvShapeDerivConstraints");
...
         get new DD;
         N_CrvShapeDerivConstraints(&cur,u,nu,I,DD,K,nd,NL_INTERACT,&rma,&upd,&SM);
       }

     THE SHAPING IS DONE IN-PLACE, I.E. THE ORIGINAL NL_CURVE IS DESTROYED.
     THE NL_CURVE IS UPDATED IF IT IS UNDER- OR FULLY-CONSTRAINED. IF IT IS
     NL_OVER-CONSTRAINED, A  NL_FLAG IS  RETURNED. KNOT  REFINEMENT  CAN  HELP 
     AVOIDING MORE THAN p CONSTRAINTS PER SPAN, WHERE p IS THE NL_DEGREE OF
     THE NL_CURVE.


   ACCESS:
   
     cur  , in/out ,  NURBS curve
     u    , input  ,  Parameters where constraints are assumed
     nu   , input  ,  Highest index in u and nd
     I    , input  ,  Array: I[k] = 
                        1 : Pw[k] IS to change
                        0 : Pw[k] is NOT to change
     DD   , input  ,  Derivative differences (constraints):  DD[i][j] is
                      the j-th constraint  assumed at u[i].  DD does not
                      have to  be a  2-D array. It  can be  an  array of 
                      pointers pointing  to arrays of  different length.
                      THE  DERIVATIVES  ARE  ASSUMED  TO  BE  STORED  IN  
                      INCREASING ORDER. Example:
                        DD[i][0] = point (0-th derivative) change
                        DD[i][1] = second derivative change
                        DD[i][2] = fourth derivative change
     K    , input  ,  Types of  constraints:  K[i][j] specifies  that at 
                      u[i]  the  K[i][j]-th  derivative  is constrained.  
                      Example (see DD above):
                        K[i][0] = 0
                        K[i][1] = 2
                        K[i][2] = 4 
     nd   , input  ,  Highest indexes in DD and K, ie  the highest index
                      in the arrays pointed to by DD[i] or K[i] is nd[i]
     flg  , input  ,  Flag:
                        NL_PREPARE : get some entities and shape
                        NL_INTERACT: change shape by changing DD
     rma  , in/out ,  Matrix defining control point changes
     upd  , output ,  Flag:
                         NL_YES: curve updated, the curve is under or fully 
                              constrained
                         NL_NO : curve is  NOT  updated, the  curve is NL_OVER
                              constrained
     SM   , input  ,  rma's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeDerivConstraints( NL_CURVE *cur, NL_PARAMETER *u, NL_INDEX nu, NL_INDEX *I, NL_VECTOR ** DD, NL_INDEX ** K, NL_INDEX *nd, NL_FLAG flg, NL_RMATRIX *rma, NL_FLAG *upd, NL_STACKS *SM )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeDerivConstraints");

    NL_FLAG twoD = NL_YES, error = NL_NO;

    NL_INDEX *O, i, j, k, l, r, s, t, n, m, spn, nb, mb, dh, row;

    NL_DEGREE p;

    NL_REAL ** ND, ** B, *U, w;

    NL_POINT *DP, P;

    NL_CPOINT *Pw;

    NL_VECTOR *DV;

    NL_KNOTVECTOR *knt;

    NL_RMATRIX rmb, rmt, rbt, rmi;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( u[0]LT U[0]OR u[nu]GT U[m] )
        NL_ERROR( NL_PAR_ERR );

    if( N_CrvIs3d( cur ) )
        twoD = NL_NO;

    switch( flg )
    {
        case NL_PREPARE:
            break;

        case NL_INTERACT:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Prepare for shaping */

    if( flg EQ NL_PREPARE )
    {
        N_CrvGetKnotVector( cur, &knt );

        /* Get offset array and adjust I array */

        O = N_AllocInt1dArray( n, &SL );

        if( O EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
            O[i] = 0;

        for ( i = 0; i <= nu; i++ )
        {
            error = N_BasisFindSpan( knt, p, u[i], NL_LEFT, &spn );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = spn - p; j <= spn; j++ )
                O[j] = 1;
        }

        mb = -1;

        for ( k = 0; k <= n; k++ )
        {
            I[k] = I[k] * O[k];

            if( I[k]EQ 1 )
                mb++;
        }

        O[0] = 0;

        for ( k = 1; k <= n; k++ )
        {
            O[k] = O[k - 1];

            if( I[k - 1]EQ 0 )
                O[k]++;
        }

        /* Get highest derivative, and number of constraints */

        nb = -1;
        dh = 0;

        for ( i = 0; i <= nu; i++ )
        {
            nb += nd[i] + 1;

            if( K[i][nd[i]]GT dh )
                dh = K[i][nd[i]];
        }

        /* If over-constrained -> out */

        if( nb GT mb )
        {
            *upd = NL_NO;
            NL_OUT;
        }

        /* Get matrix containing required derivatives */

        ND = N_AllocReal2dArray( dh, p, &SL );

        if( ND EQ NULL )
            NL_QUIT;

        error = N_SetRealMatrix( &rmb, nb, mb, NL_MT_FULL, nb, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &rmb, &B );

        for ( i = 0; i <= nb; i++ )
        {
            for ( j = 0; j <= mb; j++ )
                B[i][j] = 0.0;
        }

        row = -1;

        for ( i = 0; i <= nu; i++ )
        {
            error = N_CrvBasisDerivs( cur, u[i], NL_LEFT, K[i][nd[i]], ND, &spn );

            if( error EQ NL_YES )
                NL_OUT;

            for ( r = 0; r <= nd[i]; r++ )
            {
                row++;

                for ( s = 0; s <= p; s++ )
                {
                    j = spn - p + s;
                    t = j - O[j];

                    if( I[j]EQ 1 )
                        B[row][t] = ND[K[i][r]][s];
                }
            }
        }

        /* Compute matrix */

        if( nb EQ mb )
        {
            /* Fully-determined system */

            N_InitRealMatrix( rma );
            error = N_RealMatrixInversePivot( &rmb, rma, SM );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            /* Under-determined system */

            N_InitRealMatrix( &rmt );
            error = N_RealMatrixTranspose( &rmb, &rmt, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rbt );
            error = N_RealMatrixTransposeMultiply( &rmb, &rbt, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rmi );
            error = N_RealMatrixInversePivot( &rbt, &rmi, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( rma );
            error = N_RealMatrixMultiply( &rmt, &rmi, rma, SM );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Interactively shape */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Get right hand side of constraints */

        N_GetMaxIndexRealMatrix( rma, &mb, &nb );

        DV = N_AllocPt1dArray( nb, &SL );

        if( DV EQ NULL )
            NL_QUIT;

        k = 0;

        for ( i = 0; i <= nu; i++ )
        {
            for ( r = 0; r <= nd[i]; r++ )
            {
                N_CopyPt( DD[i][r], &DV[k] );
                k++;
            }
        }

        /* Get control point differences */

        DP = N_AllocPt1dArray( mb, &SL );

        if( DP EQ NULL )
            NL_QUIT;

        error = N_RealMatrixMultiplyPtArray( rma, DV, DP );

        if( error EQ NL_YES )
            NL_OUT;

        /* Update control points */

        *upd = NL_YES;
        l = 0;

        for ( k = 0; k <= n; k++ )
        {
            if( I[k]EQ 1 )
            {
                N_CPtToPtAndW( Pw[k], &P, &w );
                N_Sum2Pts( P, DP[l], &P );
                N_Weight( P, w, &Pw[k] );
                l++;
            }
        }
    }

    /* Check dimensionality */

    if( twoD EQ NL_YES )
        N_Crv3dTo2d( cur );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeFlatten: Flatten a NURBS curve                           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  shaping  routine  flattens a  NURBS  curve  against a bounded 
     line  or a plane. Two  kinds of flattening are implemented: (1) map 
     all control points  lying on the same  side of the (2-D)line/plane, 
     and (2) map only  control  points local  to a given knot span. Only 
     those mapped control points are accepted that lie within the bounds 
     of the line. A typical calling example is:

       NL_CURVE      curP, curQ;
       NL_INDEX      npo, ks, ke;
       NL_PARAMETER  us, ue;
       NL_LINESEG    lsg;
       NL_PLANE      pln;
       NL_VECTOR     W;
       NL_REAL       tol;
       NL_STACKS     SP, SQ;
       ...
       (define curP; get us, ue, npo, ks, ke; get either line or plane;
        get W);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvShapeFlatten(&curP,us,ue,npo,ks,ke,NULL,&pln,W,tol,NL_YES,NL_LEFT,NL_YES,
                &curQ,&SP,&SQ);

     THE  NL_CURVE  CAN BE  REFINED  PRIOR  TO  SHAPING. IN  THIS CASE  THE 
     REFINEMENT IS  SKIPPED, HOWEVER, THE ROUTINE CHECKS IF AT LEAST p+1 
     CONTROL NL_POINTS ARE LOCAL TO [us,ue]. IF REFINEMENT IS REQUESTED, IT
     IS RESTRICTRED TO THE SPAN [us,ue]!


   ACCESS:
   
     curP  , in/out ,  NURBS curve
     us,ue , input  ,  Flatten curve over [us,ue]. Meaningful only if no
                       side test is required.
     npo   , input  ,  Number of  knots to  be inserted. The  default is 
                       3*degree.
     ks,ke , input  ,  Continuity  controls; the  flattened  curve is at 
                       most C^{p-ks} continuous at us and C^{p-ke} at ue 
                       NL_PRIVATE  NL_STRING  rname = "N_CrvShapeFlatten");
(1<=ks,ke<=p!). To obtain a cusp, set ks=ke=p.
     lsg   , input  ,  Line segment (bounded or unbounded):
                          = NULL: flatten against pln
                         != NULL: flatten against lsg
     pln   , input  ,  Plane:
                          = NULL: flatten against lsg
                         != NULL: flatten against pln
     W     , input  ,  Flatten direction or  normal vector for 3-D line,
                       i.e. points mapped to the 3-D lsg by intersecting 
                       the planes defined by Pw[i] and W
     tol   , inpout ,  Knot removal tolerance. For  shaping accuracy, 1%
                       of the curve's size is a good default
     stf   , input  ,  Side test flag:
                         NL_YES: map points on given side of line/plane
                         NL_NO : map points local to [us,ue]
     lor   , input  ,  Side indicator:
                         NL_LEFT : map points on the left
                         NL_RIGHT: map points on the right
     ref   , input  ,  Refinement indicator:
                         NL_YES: refine curve
                         NL_NO : do not refine curve
     curQ  , output ,  Flattened curve
     SP    , input  ,  curP's stack
     SQ    , input  ,  curQ's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeFlatten( NL_CURVE *curP, NL_PARAMETER us, NL_PARAMETER ue, NL_INDEX npo, NL_INDEX ks, NL_INDEX ke, NL_LINESEG *lsg, NL_PLANE *pln, NL_VECTOR W, NL_REAL tol, NL_FLAG stf, NL_FLAG lor, NL_FLAG ref, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeFlatten");

    NL_FLAG side, its, threed = NL_NO, error = NL_NO;

    NL_INDEX i, n, m, s, e, nkt, nkx, fk, lk;

    NL_DEGREE p;

    NL_REAL *U, w, t;

    NL_POINT Q;

    NL_CPOINT *Pw, *Qw;

    NL_KNOTVECTOR *knt, *knx;

    NL_LINESEG lin;

    NL_PLANE pla;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( curP, &knt );

    if( N_CrvIs3d( curP ) )
        threed = NL_YES;

    if( us GE ue )
        NL_ERROR( NL_INP_ERR );

    if( us GE U[m]OR ue LE U[0] )
        NL_ERROR( NL_INP_ERR );

    if( stf NEQ NL_YES AND stf NEQ NL_NO )
        NL_ERROR( NL_CAL_ERR );

    if( ref NEQ NL_YES AND ref NEQ NL_NO )
        NL_ERROR( NL_CAL_ERR );

    if( lor NEQ NL_LEFT AND lor NEQ NL_RIGHT )
        NL_ERROR( NL_CAL_ERR );

    if( lsg NEQ NULL AND pln NEQ NULL )
        NL_ERROR( NL_CAL_ERR );

    if( threed EQ NL_YES )
    {
        if( lsg NEQ NULL AND stf EQ NL_YES )
            NL_ERROR( NL_CAL_ERR );
    }

    /* Refine curve */

    if( us LT U[0] )
        us = U[0];

    if( ue GT U[m] )
        ue = U[m];

    if( ref EQ NL_YES )
    {
        nkt = NL_MAX( npo, 3 * p ) + 1;
        nkx = nkt + 2 * p + 1;

        knx = N_AllocKnotVectorAndArray( nkx, &SL );

        if( knx EQ NULL )
            NL_QUIT;

        error = N_BasisSplitNLongestSpans( knt, p, us, ue, nkt, ks, ke, knx );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvRefine( curP, knx, curQ, SP, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_CrvGetCPts( curQ, &n, &Qw );
    N_CrvGetKnotVector( curQ, &knt );

    /* Get indexes */

    if( stf EQ NL_NO )
    {
        error = N_BasisFindSpan( knt, p, us, NL_LEFT, &s );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knt, p, ue, NL_LEFT, &e );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
            s = 0;

        if( ue EQ U[m] )
            e = n + p + 1;

        if( ref EQ NL_NO AND e - p - s - 1 LT p )
            NL_ERROR( NL_INP_ERR );
    }
    else
    {
        s = 0;
        e = n + p + 1;
    }

    /* Flatten curve against line */

    if( lsg NEQ NULL )
    {
        for ( i = s; i < e - p; i++ )
        {
            N_CPtToPtAndW( Qw[i], &Q, &w );

            if( stf EQ NL_YES )
            {
                side = N_PtGetSideOfLine( lsg, Q );

                if( side NEQ lor )
                    continue;
            }

            if( threed EQ NL_YES )
            {
                N_CreatePlanePtNormal( &pla, Q, W );

                error = N_IsectLinePlane( *lsg, pla, &Q, &t, &its );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                N_CreateLineStartDirVector( &lin, Q, W, NL_UNBOUNDED );

                error = N_IsectLineLine( *lsg, lin, &Q, &t, &t, &its );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( its EQ NL_TRUE )
                N_Weight( Q, w, &Qw[i] );
        }
    }

    if( pln NEQ NULL )
    {
        for ( i = s; i < e - p; i++ )
        {
            N_CPtToPtAndW( Qw[i], &Q, &w );

            if( stf EQ NL_YES )
            {
                side = N_PtGetSideOfPlane( pln, Q );

                if( side NEQ lor )
                    continue;
            }

            N_CreateLineStartDirVector( &lin, Q, W, NL_UNBOUNDED );

            error = N_IsectLinePlane( lin, *pln, &Q, &t, &its );

            if( error EQ NL_YES )
                NL_OUT;

            if( its EQ NL_TRUE )
                N_Weight( Q, w, &Qw[i] );
        }
    }

    /* Remove unnecessary knots */

    if( s GT 0 )
        fk = s + (p + 1) / 2;
    else
        fk = p + 1;

    if( e LT n + p + 1 )
        lk = e - (p + 2) / 2;
    else
        lk = n;

    error = N_CrvShapeRemoveKnots( curQ, tol, fk, lk, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* Check dimensionality */

    if( threed EQ NL_NO )
        N_Crv3dTo2d( curQ );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CRVSHAPEINTP: Shape curve to interpolate given points                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping  routine modifies the shape of a given base curve to
     pass through a  set of points. It performs  constrained  shaping,
     satisfying  a  set of  interpolation  conditions at a time, while 
     inproves the parameters as the curve  approaches its final shape. 
     A typical calling example is:

       NL_CURVE   curB, curS;
       NL_POINT   *P;
       NL_INDEX   np, ds, de, nit;
       NL_STACKS  SG;
       ...
       (get base curve curB and points P);
       ...
       N_CrvInitArrays(&curS);
       N_CrvShapeInterp(&curB,P,np,ds,de,nit,&curS,&SG);

     MEMORY FOR THE  NL_CURVE STRUCTURE OF curS  MUST BE ALLOCATED IN THE
     CALLING ROUTINE (AS SHOWN IN THE EXAMPLE ABOVE).


   ACCESS:
   
     curB , input  ,  Base curve
     P    , input  ,  Random points output curve must interpolate
     np   , input  ,  Highest index in P
     ds   , input  ,  Start derivative constraint; 0,..,ds derivatives
                      not to change at the start
     de   , input  ,  End  derivative  constraint; 0,..,de derivatives 
                      not to change at the end
     nit  , input  ,  Number of iterations  required to reach the most
                      distant point. For most  applications nit = 1 is 
                      more than adequate!
     curS , output ,  Shaped curve interpolating P[i], i=0,...,np
     SG   , input  ,  curS' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeInterp( NL_CURVE *curB, NL_POINT *P, NL_INDEX np, NL_INDEX ds, NL_INDEX de, NL_INDEX nit, NL_CURVE *curS, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeInterp");

    NL_FLAG *used, upd, plt, error = NL_NO;

    NL_INDEX ** K, *I, *nd, nq, nc, i, j, l, m, n, nI;

    NL_DEGREE p;

    NL_REAL *U, *up, *uc, *uq, *ui, *d, ltl, dis, dmax, fac, uu, dinc;

    NL_POINT *Q, R;

    NL_VECTOR ** DD, V;

    NL_KNOTVECTOR knx;

    NL_RMATRIX rma;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL dtl = 0.001;
    NL_PRIVATE NL_REAL cdt = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Prepare for shaping and get local notation */

    N_CrvGetBBoxMaxDiagDist( curB, &dis );
    ltl = dis * dtl;

    error = N_CrvCopy( curB, curS, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curS, &m, &U );
    N_CrvGetArraySizes( curS, &n, &m );
    N_CrvGetDegree( curS, &p );

    nI = n + np + 2;

    /* Allocate memory */

    I = N_AllocInt1dArray( nI, &SL );

    if( I EQ NULL )
        NL_QUIT;

    K = N_AllocInt2dArray( np, 0, &SL );

    if( K EQ NULL )
        NL_QUIT;

    nd = N_AllocInt1dArray( np, &SL );

    if( nd EQ NULL )
        NL_QUIT;

    DD = N_AllocPt2dArray( np, 0, &SL );

    if( DD EQ NULL )
        NL_QUIT;

    up = N_AllocReal1dArray( np, &SL );

    if( up EQ NULL )
        NL_QUIT;

    d = N_AllocReal1dArray( np, &SL );

    if( d EQ NULL )
        NL_QUIT;

    used = N_AllocFlag1dArray( np, &SL );

    if( used EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= np; i++ )
    {
        K[i][0] = 0;
        nd[i] = 0;
        used[i] = NL_NO;
    }

    /* Get distance increment */

    error = N_ApproxCrvWithPolyline( curS, ltl, NL_ABSOLUTE, NL_BOTH, &Q, &uq, &nq, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    dmax = 0.0;

    for ( i = 0; i <= np; i++ )
    {
        error = N_ProjectPtClosePolyPt( Q, uq, nq, P[i], &R, &up[i], &dis );

        if( error EQ NL_YES )
            NL_OUT;

        if( up[i]LE uq[0]OR up[i]GE uq[nq] )
            NL_ERROR( NL_NUM_ERR );

        if( dis GT dmax )
            dmax = dis;
    }

    dinc = dmax / nit;

    if( dinc LT NL_MTOL )
        dinc = 0.0;

    /*********************/
    /* Iteratively shape */
    /*********************/

    /* While not all points have been interpolated do */

    while( 1 )
    {
        /* Get new parameters */

        error = N_ApproxCrvWithPolyline( curS, ltl, NL_ABSOLUTE, NL_BOTH, &Q, &uq, &nq, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        plt = NL_NO;

        for ( i = 0; i <= np; i++ )
        {
            if( used[i]EQ NL_NO )
            {
                error = N_ProjectPtClosePolyPt( Q, uq, nq, P[i], &R, &up[i], &dis );

                if( error EQ NL_YES )
                    NL_OUT;

                plt = NL_YES;
            }
        }

        if( plt EQ NL_NO )
            break;

        /* Eliminate clustering of points */

        error = N_FindClustersRealArray( up, np, cdt, NL_PARAMETERS, &uc, &nc, NULL, NULL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* See if additional degrees of freedom are needed */

        error = N_KnotsRefine( U, m, p, uc, nc, ds, de, &ui, &l, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( l GE 0 )
        {
            /* Must refine knot vector */

            N_KnotVectorFromRealArray( &knx, ui, l );

            error = N_CrvRefine( curS, &knx, curS, SG, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetKnots( curS, &m, &U );
            N_CrvGetArraySizes( curS, &n, &m );

            if( n GT nI )
            {
                l = NL_MAX( n + nI, nI + nI );

                error = N_Realloc1dIntArray( &I, nI, l, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                nI = l;
            }
        }

        /* Set up for shaping */

        for ( i = 0; i <= np; i++ )
        {
            if( used[i]EQ NL_YES )
            {
                N_VectorCopy( NL_ZERO, &DD[i][0] );
                d[i] = 0.0;
            }
            else
            {
                N_CrvEval( curS, up[i], NL_LEFT, &R );

                if( error EQ NL_YES )
                    NL_OUT;

                N_VectorDiff( P[i], R, &V );
                N_VectorMagnitude( V, &d[i] );

                if( d[i]LT NL_MTOL )
                    d[i] = 0.0;

                if( d[i]LE dinc )
                {
                    N_VectorDiff( P[i], R, &DD[i][0] );
                }
                else
                {
                    fac = dinc / d[i];
                    N_VectorScale( V, fac, &DD[i][0] );
                }
            }
        }

        for ( j = 0; j <= ds; j++ )
            I[j] = 0;

        for ( j = 0; j <= de; j++ )
            I[n - j] = 0;

        for ( j = ds + 1; j <= n - de - 1; j++ )
            I[j] = 1;

        /* Shape curve now */

        error = N_CrvShapeDerivConstraints( curS, up, np, I, DD, K, nd, NL_PREPARE, &rma, &upd, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( upd EQ NL_NO )
        {
            N_CrvGetKnots( curS, &m, &U );

            dmax = 0.0;

            for ( i = p; i < m - p; i++ )
            {
                if( U[i]NEQ U[i + 1] )
                {
                    dis = U[i + 1] - U[i];

                    if( dis GT dmax )
                    {
                        dmax = dis;
                        j = i;
                    }
                }
            }

            uu = 0.5 *( U[j] + U[j + 1] );

            error = N_CrvInsertKnot( curS, uu, 1, curS, SG, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetKnots( curS, &m, &U );
            N_CrvGetArraySizes( curS, &n, &m );
        }
        else
        {
            for ( i = 0; i <= np; i++ )
                if( d[i]LE dinc )
                    used[i] = NL_YES;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeModifyWeight: Modify one curve weight                                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies  one curve weight to  pull the curve
     toward a given control point  with a specified distance. The curve
     weight is allowed to vary in the range of [NL_WMIN,NL_WMAX] as specified
     in  "globals.h". The shaping is  done in place, i.e. the  original 
     curve is  destroyed. To facilitate  with interactive shape design,
     several quantities, needed to  compute the new weight, are output.
     A typical calling example is:

       NL_CURVE      cur;
       NL_INDEX      k;
       NL_PARAMETER  u;
       NL_REAL       d, wk, pkp, rku;
       ...
       (define cur; get u and d);
       ...
       N_CrvShapeModifyWeight(&cur,k,u,d,&wk,&pkp,&rku,NL_PREPARE);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_CrvShapeModifyWeight");
...
         d = d+dd;
         N_CrvShapeModifyWeight(&cur,k,u,d,&wk,&pkp,&rku,NL_INTERACT);
       }

     To prepare  for interactive  shape modification, several utilities 
     are  provided:  N_BasisFindIndexNode,  N_BasisComputeIndexNodeArray,  N_BasisFindNodeSpan and N_BasisFindKnotToTurnParamIntoNode. These 
     utilities  allow the  designer to  compute nodes, find node spans, 
     refine the  curve with  the appropriate knot, and  select the most
     suitable control point. To  insert a knot, see the  many utilities
     N_TOO*** in the tools directory. 


   ACCESS:
   
     cur , in/out ,  NURBS curve
     k   , input  ,  Index of weight to be changed
     u   , input  ,  Parameter where curve point is to be pulled/pushed
     d   , input  ,  Distance of pull/push
     wk  , in/out ,  Original weight of Pw[k]
     pkp , in/out ,  Distance between C(u) and Pw[k]
     rku , in/out ,  Value of rational basis function
     flg , input  ,  Flag:
                       NL_PREPARE : get pkp, rku and update weight
                       NL_INTERACT: update weight


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeModifyWeight( NL_CURVE *cur, NL_INDEX k, NL_PARAMETER u, NL_REAL d, NL_REAL *wk, NL_REAL *pkp, NL_REAL *rku, NL_FLAG flg )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeModifyWeight");

    NL_FLAG error = NL_NO, threed = NL_NO;

    NL_INDEX n;

    NL_REAL w, wh, den;

    NL_POINT Pk, P;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notations and check input */

    N_CrvGetCPts( cur, &n, &Pw );

    if( k LT 0 OR k GT n )
        NL_ERROR( NL_IND_ERR );

    if( NOT N_IsCrvRat( cur ) )
        NL_ERROR( NL_INP_ERR );

    if( N_CrvIs3d( cur ) )
        threed = NL_YES;

    switch( flg )
    {
        case NL_PREPARE:
            break;

        case NL_INTERACT:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Compute some quantities if first call */

    N_CPtToPtAndW( Pw[k], &Pk, &w );

    if( flg EQ NL_PREPARE )
    {
        error = N_CrvBasisIEval( cur, k, u, NL_LEFT, rku );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( cur, u, NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( Pk, P, pkp );

        *wk = w;
    }

    /* Compute new weight and reweight control point */

    if( flg EQ NL_PREPARE OR flg EQ NL_INTERACT )
    {
        w = *wk;

        den = (*rku) * (*pkp - d);

        if( N_FloatOpIsBad( d, den, NL_DIVISION ) )
            NL_ERROR( NL_INP_ERR );

        wh = w * (1.0 + d / den);

        if( wh LT NL_WMIN OR wh GT NL_WMAX )
            NL_ERROR( NL_WEI_ERR );

        N_Weight( Pk, wh, &Pw[k] );

        if( threed EQ NL_NO )
            N_CPtSetZ( NL_NOZ, &Pw[k] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}

/**********************************************************************/
/* N_CRVSHAPEREMOVEKNOTS: Remove all removable knots from a curve being shaped     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  shaping routine  removes all  removable knots  from a  NURBS 
     curve. The routine is for shape operators where only a given range 
     of knots are to be removed. The removal is done in place, i.e. the
     original curve is destroyed. A typical calling example is:

       NL_CURVE   cur;
       NL_REAL    tol;
       NL_INDEX   fk, lk;
       NL_STACKS  SG;
       ...
       (define cur; get tol, fk and lk);
       ...
       N_CrvShapeRemoveKnots(&cur,tol,fk,lk,&SG);


   ACCESS:
   
     cur   , in/out ,  NURBS curve
     tol   , input  ,  Tolerance to check removability
     fk,lk , input  ,  Indexes of first and last knots
     SG    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeRemoveKnots( NL_CURVE *cur, NL_REAL tol, NL_INDEX fk, NL_INDEX lk, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeRemoveKnots");

    NL_FLAG rmf, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, fout, l, ns;

    NL_DEGREE p;

    NL_REAL *U, *br, *er, *te, *minl, *maxl, *minr, *maxr, *max, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, lto, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Rw;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL cto = 1.0e-05;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and check indexes */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur, &knt );

    if( fk LE p OR lk GT n )
        NL_ERROR( NL_IND_ERR );

    ns = n;

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( cur ) )
    {
        N_CrvGetMinMaxWeightsAndPts( cur, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    lto = cto * fabs( U[m] - U[0] );

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

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;

        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( cur, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        b = br[fk];
        s = sr[fk];
        r = fk;

        for ( i = fk + 1; i <= lk; i++ )
        {
            if( br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot -> finished */

        if( b EQ NL_BIGD )
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

        for ( i = r - k; i <= l; i++ )
        {
            if( U[i]NEQ U[i + 1] )
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
                if( U[i]NEQ U[i + 1] )
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
                    br[r] = NL_BIGD;
                    continue;
                }
            }

            /* Save new control points */

            if( (p + s) % 2 )
            {
                N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[ii - 1] );
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
                U[i - 1] = U[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Pw[i], &Pw[i - 1] );

            n--;
            m--;
            lk--;
            N_CrvSetSizeIndices( cur, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p OR fk GT lk )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( U[i]NEQ U[i + 1] )
                {
                    error = N_CrvRemoveKnotMaxErr( cur, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NL_BIGD;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
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
/* N_CrvShapeCreateBasis: Make rational basis function for curve warping           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine creates a rational basis function used to warp
     curves. It is a degree p rational basis function that is defined by 
     symmetrically placed knots and by unit weights except w[p] which is
     used as a shape control tool. A typical calling example is:

       NL_CFUN    cfn;
       NL_REAL    wp;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (get wp and p);
       ...
       N_CrvShapeCreateBasis(wp,p,&cfn,NL_NEW,&S);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_CrvShapeCreateBasis");
...
         change wp
         N_CrvShapeCreateBasis(wp,p,&cfn,NL_OLD,&S);
       }

     If  the NL_NEW  option is  chosen, memory  to store  cfn's  members is 
     allocated. During  interactive design, i.e. with the  NL_OLD flag set, 
     w[p] is updated and no other change is made.


   ACCESS:
   
     wp  , input  ,  Weight  for w[p] (the  middle  weight; the rest are 
                     set 1.0)
     p   , input  ,  Degree of rational basis function
     cfn , in/out ,  Rational basis function represented as  NURBS curve 
                     function
     flg , input  ,  Flag:
                       NL_NEW: allocate  memory for  cfn and  compute knots 
                            and weights
                       NL_OLD: update middle weight w[p]
     SG  , input  ,  cfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeCreateBasis( NL_REAL wp, NL_DEGREE p, NL_CFUN *cfn, NL_FLAG flg, NL_STACKS *SG )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeCreateBasis");

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL *w, *U, uinc;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute function */

    switch( flg )
    {
        case NL_NEW:

            w = N_AllocReal1dArray( 2 * p, SG );

            if( w EQ NULL )
                NL_QUIT;

            U = N_AllocReal1dArray( 3 * p + 1, SG );

            if( U EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= 2 *p; i++ )
                w[i] = 1.0;

            w[p] = wp;
            uinc = 1.0 / ((NL_REAL)p + 1.0);

            for ( i = 0; i <= p; i++ )
            {
                U[2 * p + i + 1] = 1.0;
                U[i] = 0.0;
            }

            for ( i = 1; i <= p; i++ )
                U[p + i] = i * uinc;

            error = N_CFuncFromKnots( cfn, w, 2 * p, p, U, 3 * p + 1, SG );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_OLD:

            N_CrvFuncCntrlValKnots( cfn, &w, &U );

            w[p] = wp;
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
/* N_CrvShapeModifyCPts: Reposition curve control points                          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine repositions at most (degree+1) number of curve 
     control points to shape the curve locally in a given direction with 
     a given distance. More precisely, the control points  Pw[I[0]],...,
     Pw[I[k]] are repositioned to Phw[I[0]],...,Phw[I[k]] so that |C(u)-
     Ch(u)|=d and dir{C(u),Ch(u)}=V, where d is a given distance, V is a 
       NL_PRIVATE  NL_STRING  rname = "N_CrvShapeModifyCPts");
given direction vector, and I[0],...,I[k] are the indexes of points
     to be moved. The shaping is done in place, i.e. the  original curve  
     is destroyed. A typical calling example is:

       NL_CURVE      cur;
       NL_INDEX      *I, k;
       NL_PARAMETER  u;
       NL_VECTOR     V;
       NL_REAL       *gam, d, alf;
       ...
       (define cur; get arrays I and gam; choose u, d and V);
       ...
       N_CrvShapeModifyCPts(&cur,I,gam,k,u,V,d,&alf,NL_PREPARE);
       while( NOT DONE )
       {
         ...
         get dalf;
         N_CrvShapeModifyCPts(&cur,I,gam,k,u,V,d,&dalf,NL_INTERACT);
       }

     To prepare  for interactive  shape modification,  several utilities 
     are  provided:  N_BasisFindIndexNode,  N_BasisComputeIndexNodeArray,  N_BasisFindNodeSpan and  N_BasisFindKnotToTurnParamIntoNode. These 
     utilities  allow the  designer to  compute  nodes, find node spans, 
     refine the  curve with  the appropriate  knot, and  select the most
     suitable control points. To insert a  knot, see the  many utilities
     N_TOO*** in the tools directory.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     I   , input  ,  Index array; control points with  indexes I[0],...,
                     I[k] are moved
     gam , input  ,  Weight factors (Pw[I[r]] is weighted by gam[r])
     k   , input  ,  Highest indexes in I and gam
     u   , input  ,  Parameter where curve point is to be moved
     V   , input  ,  Direction vector (V=dir{C(u),Ch(u)})
     d   , input  ,  Distance (d=|C(u)-Ch(u)|)
     alf , in/out ,  Magnitude of repositioning vector
     flg , input  ,  Flag:
                       NL_PREPARE : get alf and reposition control points
                       NL_INTERACT: reposition control points with dalf


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeModifyCPts( NL_CURVE *cur, NL_INDEX *I, NL_REAL *gam, NL_INDEX k, NL_PARAMETER u, NL_VECTOR V, NL_REAL d, NL_REAL *alf, NL_FLAG flg )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeModifyCPts");

    NL_FLAG error = NL_NO, threed = NL_NO;

    NL_INDEX r, n, m;

    NL_DEGREE p;

    NL_REAL *U, ktol, mag, R, w, sum;

    NL_POINT P;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notations and check input */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( I[0]LT 0 OR I[k]GT n )
        NL_ERROR( NL_IND_ERR );

    if( N_CrvIs3d( cur ) )
        threed = NL_YES;

    switch( flg )
    {
        case NL_PREPARE:
            break;

        case NL_INTERACT:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    ktol = fabs( U[m] - U[0] ) * NL_PTOL;

    /* Compute alpha */

    if( flg EQ NL_PREPARE )
    {
        N_VectorMagnitude( V, &mag );

        sum = 0.0;

        for ( r = 0; r <= k; r++ )
        {
            error = N_CrvBasisIEval( cur, I[r], u, NL_LEFT, &R );

            if( error EQ NL_YES )
                NL_OUT;

            if( R LT ktol )
                gam[r] = 0.0;
            sum += gam[r] * R;
        }

        if( N_FloatOpIsBad( d, mag * sum, NL_DIVISION ) )
            NL_ERROR( NL_INP_ERR );

        *alf = d / (mag * sum);
    }

    /* Reposition control points */

    if( flg EQ NL_PREPARE OR flg EQ NL_INTERACT )
    {
        for ( r = 0; r <= k; r++ )
        {
            N_CPtToPtAndW( Pw[I[r]], &P, &w );
            N_VectorBlendPt( *alf * gam[r], V, &P );
            N_Weight( P, w, &Pw[I[r]] );

            if( threed EQ NL_NO )
                N_CPtSetZ( NL_NOZ, &Pw[I[r]] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}

/**********************************************************************/
/* N_CrvShapeWarp: Warp NURBS curve                                   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine warps a NURBS  curve either with  a predefined
     warp function or with a rational basis function. To facilitate with
     various warps, the warp region [us,ue] does not have to fall within
     [U[0],U[m]], the knot  interval. If us<U[0] or ue>U[m], the warp is
     performed on an imaginary  (extended) curve, however, only the real
     portion of the curve is  actually computed. When interactive change
     is required,  only those  quantities are  recomputed that  directly 
     affect the change in the warp. A typical calling example is:

       NL_CURVE      curP, curQ, curV;
       NL_CFUN       cfnD;
       NL_DEGREE     p;
       NL_INDEX      npo, ks, ke, si, ei;
       NL_PARAMETER  us, ue;
       NL_VECTOR     V;
       NL_REAL       *t, d, tol, wp;
       NL_STACKS     SP, SQ;
       ...
       (define curP; get us, ue, npo, ks, ke; get either  curV or V; get
        rational basis function and d, or distance function; get tol);
       ...
       N_CrvShapeCreateBasis(wp,p,&cfnD,NL_NEW,&SQ);
       ...
       N_CrvShapeWarp(&curP,us,ue,npo,ks,ke,NULL,V,&cfnD,d,tol,NL_PREPARE,NL_YES,
                &t,&si,&ei,&curQ,&SP,&SQ);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_CrvShapeWarp");
...
         get new wp;
         N_CrvShapeCreateBasis(wp,p,&cfnD,NL_OLD,&SQ);
         ...
         N_CrvShapeWarp(&curP,us,ue,npo,ks,ke,NULL,V,&cfnD,d,tol,NL_INTERACT,NL_NO,
                  &t,&si,&ei,&curQ,&SP,&SQ);
         ...
         get new d;
         N_CrvShapeWarp(&curP,us,ue,npo,ks,ke,NULL,V,&cfnD,d,tol,NL_INTERACT,NL_NO,
                  &t,&si,&ei,&curQ,&SP,&SQ);
       }
       N_CrvShapeWarp(&curP,us,ue,npo,ks,ke,NULL,V,&cfnD,d,tol,NL_CLEANUP,NL_NO,
                &t,&si,&ei,&curQ,&SP,&SQ);

     THE WARPED NL_CURVE IS  STORED IN curQ. ALTHOUGH  THE INPUT NL_CURVE DOES
     NOT CHANGE EITHER  GEOMETIRICALLY OR PARAMETRICALLY, ITS DEFINITION
     IS  DESTROYED  IN  THAT  ITS  KNOT  NL_VECTOR  IS  REFINED. IF  IT  IS 
     UNACCEPTABLE, EITHER SAVE THE ORIGINAL NL_CURVE OR  REMOVE UNNECESSARY 
     KNOTS. THE NL_CURVE CAN BE REFINED  PRIOR TO SHAPING. IN THIS CASE THE
     REFINEMENT IS SKIPPED IN THE  "NL_PREPARE" STAGE, HOWEVER, THE ROUTINE
     CHECKS IF AT LEAST p+1 CONTROL NL_POINTS ARE LOCAL TO [us,ue].


   ACCESS:
   
     curP  , in/out ,  NURBS curve
     us,ue , input  ,  Warp  curve  over  [us,ue]. Both  can  exceed the 
                       parameter bounds  U[0] and  U[m] to achieve warps 
                       at the ends of the curve or over the entire arc.
     npo   , input  ,  Number of  knots to be  inserted. Depends  on the
                       warp  distance; for  large  warps,  more  control 
                       points are  needed to  obtain a smooth  change in 
                       shape. The default is 3*degree for small warps.
     ks,ke , input  ,  Continuity controls; the warped  curve is at most
                       C^{p-ks} continuous at us and C^{p-ke} at ue (1<=
                       ks,ke<=p!). To obtain a cusp, set ks=ke=p.
     curV  , input  ,  Direction curve:
                          = NULL: use input vector V
                         != NULL: use curV
     V     , input  ,  Warp direction
     cfnD  , input  ,  Warp distance function or rational basis function
                       (must be parametrized between [0,1]!)
     d     , input  ,  Warp distance:
                         > 0.0: cfnD is rational basis function
                         < 0.0: cfnD is distance function
     tol   , inpout ,  Knot removal tolerance. For  shaping accuracy, 1%
                       of the curve's size is a good default
     flg   , input  ,  Flag:
                         NL_PREPARE : get some entities and do first warp
                         NL_INTERACT: change warp by changing d, V, etc.
                         NL_CLEANUP : remove unneccesary knots
     ref   , input  ,  Flag:
                         NL_YES: refine curve
                         NL_NO : curve  is  already  refined (must  have at 
                              least p+1 control points local to [us,ue])
     t     , in/out ,  Local parametrization
     si,ei , in/out ,  Indexes of local control points
     curQ  , output ,  Warped  curve; if curP = curQ,  only the  NL_PREPARE 
                       and NL_CLEANUP options are allowed
     SP    , input  ,  curP's stack
     SQ    , input  ,  curQ's and t's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvShapeWarp( NL_CURVE *curP, NL_PARAMETER us, NL_PARAMETER ue, NL_INDEX npo, NL_INDEX ks, NL_INDEX ke, NL_CURVE *curV, NL_VECTOR V, NL_CFUN *cfnD, NL_REAL d, NL_REAL tol, NL_FLAG flg, NL_FLAG ref, NL_REAL ** t, NL_INDEX *si, NL_INDEX *ei, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvShapeWarp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k, n, m, s = 0, e = 0, nkt, nkx, fk, lk;

    NL_DEGREE p, pb = 0;

    NL_REAL *U, *tl = NULL, *dist, ul, ur, sum, ts, te, rmax = 0.0, w, f;

    NL_POINT Q;

    NL_CPOINT *Pw, *Qw = NULL, Sw, Ew;

    NL_KNOTVECTOR *knt = NULL, *knx;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &U );

    if( us GE ue )
        NL_ERROR( NL_INP_ERR );

    if( us GE U[m]OR ue LE U[0] )
        NL_ERROR( NL_INP_ERR );

    switch( flg )
    {
        case NL_PREPARE:

            N_CrvGetKnotVector( curP, &knt );
            break;

        case NL_INTERACT:
            if( curP EQ curQ )
                NL_ERROR( NL_INP_ERR );

            s = *si;
            e = *ei;
            tl = *t;
            N_CrvGetCPts( curQ, &n, &Qw );
            break;

        case NL_CLEANUP:

            s = *si;
            e = *ei;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Prepare for warp */

    if( flg EQ NL_PREPARE )
    {
        /* Set knot interval */

        if( us LT U[0] )
            ul = U[0];
        else
            ul = us;

        if( ue GT U[m] )
            ur = U[m];
        else
            ur = ue;

        /* Refine curve */

        if( ref EQ NL_YES )
        {
            f = (ur - ul) / (ue - us);
            nkt = (NL_INDEX)(f * NL_MAX( npo, 3 * p )) + 1;
            nkx = nkt + 2 * p + 1;

            knx = N_AllocKnotVectorAndArray( nkx, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            error = N_BasisSplitNLongestSpans( knt, p, ul, ur, nkt, ks, ke, knx );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvRefine( curP, knx, curP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( curP NEQ curQ )
        {
            N_CrvInitArrays( curQ );
            error = N_CrvCopy( curP, curQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPts( curQ, &n, &Qw );
        }
        else
        {
            N_CrvGetCPts( curP, &n, &Qw );
        }

        N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &U );
        N_CrvGetKnotVector( curP, &knt );

        /* Get local parametrization */

        error = N_BasisFindSpan( knt, p, ul, NL_LEFT, &s );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knt, p, ur, NL_LEFT, &e );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
            s = 1;

        if( us LT U[0] )
            s = 0;

        if( ue EQ U[m] )
            e = n + p;

        if( ue GT U[m] )
            e = n + p + 1;

        if( ref EQ NL_NO AND e - p - s - 1 LT p )
            NL_ERROR( NL_INP_ERR );

        k = e - p - s + 1;

        tl = N_AllocReal1dArray( k, SQ );

        if( tl EQ NULL )
            NL_QUIT;

        dist = N_AllocReal1dArray( k, &SL );

        if( dist EQ NULL )
            NL_QUIT;

        tl[0] = 0.0;
        tl[k] = 1.0;

        if( s EQ 0 )
            N_Combine2CPts( 2.0, Pw[0], -1.0, Pw[1], &Sw );
        else
            N_CopyCPt( Pw[s - 1], &Sw );

        if( e EQ n + p + 1 )
            N_Combine2CPts( 2.0, Pw[n], -1.0, Pw[n - 1], &Ew );
        else
            N_CopyCPt( Pw[e - p], &Ew );

        N_DistCptCpt( Sw, Pw[s], &dist[0] );
        sum = dist[0];

        for ( i = s; i < e - p - 1; i++ )
        {
            N_DistCptCpt( Pw[i], Pw[i + 1], &dist[i - s + 1] );
            sum += dist[i - s + 1];
        }

        N_DistCptCpt( Pw[e - p - 1], Ew, &dist[e - p - s + 1] );
        sum += dist[e - p - s + 1];

        for ( i = 1; i < k; i++ )
            tl[i] = tl[i - 1] + dist[i - 1] / sum;

        /* Rescale parameters */

        if( us LT U[0]OR ue GT U[m] )
        {
            if( us LT U[0] )
                ts = (ul - us) / (ue - us);
            else
                ts = 0.0;

            if( ue GT U[m] )
                te = (ur - us) / (ue - us);
            else
                te = 1.0;

            tl[0] = ts;
            tl[k] = te;

            for ( i = 1; i < k; i++ )
                tl[i] = ts + tl[i] * (te - ts);
        }

        /* Output some parameters for interactive step */

        *si = s;
        *ei = e;
        *t = tl;
    }

    /* Interactively warp */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Recover original control points */

        if( curP NEQ curQ )
        {
            for ( i = s; i < e - p; i++ )
                N_CopyCPt( Pw[i], &Qw[i] );
        }

        /* Reposition control points */

        if( d GE 0.0 )
        {
            N_CFuncGetDegree( cfnD, &pb );

            error = N_CrvFuncEvalRatBasis( cfnD, pb, 0.5, NL_LEFT, &rmax );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( i = s; i < e - p; i++ )
        {
            N_CPtToPtAndW( Qw[i], &Q, &w );

            if( d LT 0.0 )
            {
                error = N_CFuncEval( cfnD, tl[i - s + 1], NL_LEFT, &f );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                error = N_CrvFuncEvalRatBasis( cfnD, pb, tl[i - s + 1], NL_LEFT, &f );

                if( error EQ NL_YES )
                    NL_OUT;

                f = (d * f) / rmax;
            }

            if( curV NEQ NULL )
            {
                error = N_CrvEval( curV, tl[i - s + 1], NL_LEFT, &V );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            N_VectorBlendPt( f, V, &Q );
            N_Weight( Q, w, &Qw[i] );
        }
    }

    /* Remove unnecessary knots */

    if( flg EQ NL_CLEANUP )
    {
        if( us GT U[0] )
            fk = s + (p + 1) / 2;
        else
            fk = p + 1;

        if( ue LT U[m] )
            lk = e - (p + 2) / 2;
        else
            lk = n;

        error = N_CrvShapeRemoveKnots( curQ, tol, fk, lk, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check dimensionality */

    if( NOT N_CrvIs3d( curP ) )
        N_Crv3dTo2d( curQ );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_IsectCrvShapeLineCrv: Line-curve intersection for bending                      */
/**********************************************************************/

NL_PRIVATE NL_INTEGER nop = 20;
NL_PRIVATE NL_INTEGER itl = 10;
NL_PRIVATE NL_INTEGER cvl = 10;
NL_PRIVATE NL_REAL toc = 1.0e-05;

/*******************************************************************//**


   DESCRIPTION:

     This  shaping  routine  computes the  intersection of a  line and a 
     curve under the following assumptions:

       (1) the curve is in 2-D;
       (2) the curve is strictly convex/concave;
       (3) it has a reasonable parametrization;
       (4) the line intersects the curve only once; and
       (5) there are no special cases such as touch.

     The intersector is used for bending  curves with simple bend curves
     such as circles and parabolas. A typical calling example is:

       NL_FLAG       its;
       NL_CURVE      cur;
       NL_LINESEG    lsg;
       NL_PARAMETER  us, ue, ug, uc;
       NL_REAL       tol;
       NL_POINT      Q;
       ...
       (define cur and lsg; get us, ue, ug and tol);
       ...
       N_IsectCrvShapeLineCrv(&cur,&lsg,us,ue,ug,NL_YES,tol,&Q,&uc,&its);
       ...
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING   rname = "N_IsectCrvShapeLineCrv");
...
         ug = uc;
         get next lsg;
         N_IsectCrvShapeLineCrv(&cur,&lsg,us,ue,ug,NL_NO,tol,&Q,&uc,&its);
         ...
       } 

     The curve  papameter  corresponding to the intersection is returned
     so that it can be used as a guess parameter for the next call.


   ACCESS:
   
     cur   , input  ,  NURBS curve
     us,ue , input  ,  Get intersection of segment defined over [us,ue]
     ug    , input  ,  Guess parameter
     flg   , input  ,  Flag:
                         NL_YES: compute guess parameter
                         NL_NO : guess parameter passed in
     tol   , input  ,  Tolerance; the  distance  between a  point on the 
                       line and its projection to cur should not deviate 
                       more than tol
     Q     , output ,  Intersection point
     uc    , output ,  Curve parameter corresponding to Q
     its   , output ,  Flag:
                         NL_TRUE : intersection found
                         NL_FALSE: no intersection found
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsectCrvShapeLineCrv( NL_CURVE *cur, NL_LINESEG *lsg, NL_PARAMETER us, NL_PARAMETER ue, NL_PARAMETER ug, NL_FLAG flg, NL_REAL tol, NL_POINT *Q, NL_PARAMETER *uc, NL_FLAG *its )
{

    NL_PRIVATE NL_STRING rname = _T("N_IsectCrvShapeLineCrv");

    NL_FLAG sch, itf, conv = NL_NO, error = NL_NO;

    NL_INDEX k, m, it;

    NL_REAL *U, ul, ur, uold, unew = 0.0, t, d;

    NL_POINT D[2], R;

    NL_LINESEG lin;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check dimension and initialize */

    if( N_CrvIs3d( cur ) )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetKnots( cur, &m, &U );

    *its = NL_TRUE;
    ul = us;
    ur = ue;

    /* Check end points */

    error = N_CrvEval( cur, us, NL_LEFT, Q );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtInfLine( *lsg, *Q, &d );

    if( error EQ NL_YES )
        NL_OUT;

    if( d LT tol )
    {
        *uc = us;
        NL_OUT;
    }

    error = N_CrvEval( cur, ue, NL_LEFT, Q );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtInfLine( *lsg, *Q, &d );

    if( error EQ NL_YES )
        NL_OUT;

    if( d LT tol )
    {
        *uc = ue;
        NL_OUT;
    }

    /* Compute guess value if needed */

    uold = ug;

    if( flg EQ NL_YES )
    {
        error = ST_refine( cur, lsg, &ul, &ur, &uold, &sch );

        if( error EQ NL_YES )
            NL_OUT;

        if( sch EQ NL_NO )
        {
            *its = NL_FALSE;
            NL_OUT;
        }
    }

    /* Use Newton itertions till convergence reached */

    it = 0;

    while( it LT itl )
    {
        k = 0;

        while( k LT cvl )
        {
            /* Get point and first derivate */

            error = N_CrvDerivs( cur, uold, NL_LEFT, 1, D );

            if( error EQ NL_YES )
                NL_OUT;

            /* Intersect tangent with given line */

            N_CreateLineStartDirVector( &lin, D[0], D[1], NL_UNBOUNDED );

            error = N_IsectLineLine( *lsg, lin, &R, &t, &t, &itf );

            if( error EQ NL_YES )
                NL_OUT;

            if( itf EQ NL_FALSE )
                break;

            /* Project intersection point to curve */

            error = N_CrvClosestPt( cur, R, uold, tol, toc, &unew, Q );

            if( error EQ NL_YES )
                break;

            /* Check convergence */

            N_DistPtPt( *Q, R, &d );

            if( d LT tol )
            {
                conv = NL_YES;
                break;
            }

            /* Check parameters */

            if( fabs( unew - uold )LT NL_PTOL )
            {
                *its = NL_FALSE;
                NL_OUT;
            }

            if( unew EQ U[0]OR unew EQ U[m] )
                break;

            uold = unew;
            k++;
        }

        if( conv EQ NL_YES )
            break;

        /* Get better guess parameter */

        error = ST_refine( cur, lsg, &ul, &ur, &uold, &sch );

        if( error EQ NL_YES )
            NL_OUT;

        if( sch EQ NL_NO )
        {
            *its = NL_FALSE;
            NL_OUT;
        }

        it++;
    }

    /* Check convergence */

    if( it GE itl )
        NL_ERROR( NL_CON_ERR );

    if( unew LT us OR unew GT ue )
    {
        *its = NL_FALSE;
        NL_OUT;
    }

    *uc = unew;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

/**********************************************************************/
/* ST_refine: Refine parameter span to find guess parameter            */
/**********************************************************************/

NL_FLAG ST_refine( NL_CURVE *cur, NL_LINESEG *lsg, NL_PARAMETER *ul, NL_PARAMETER *ur, NL_PARAMETER *ug, NL_FLAG *sch )
{

    NL_FLAG sidel, sider = NL_NO, error = NL_NO;

    NL_INDEX i;

    NL_REAL ts, te, tl, tr = 0.0, tinc;

    NL_POINT C;

    /* Initialize */

    ts = *ul;
    te = *ur;
    tl = ts;
    *sch = NL_NO;

    error = N_CrvEval( cur, ts, NL_LEFT, &C );

    if( error EQ NL_YES )
        NL_OUT;

    sidel = N_PtGetSideOfLine( lsg, C );
    tinc = (te - ts) / nop;

    /* Zip through the curve and bracket intersection */

    for ( i = 1; i <= nop; i++ )
    {
        tr = ts + i * tinc;

        if( i EQ nop )
            tr = te;

        error = N_CrvEval( cur, tr, NL_LEFT, &C );

        if( error EQ NL_YES )
            NL_OUT;

        sider = N_PtGetSideOfLine( lsg, C );

        if( sidel NEQ sider )
            break;

        tl = tr;
        sidel = sider;
    }

    if( sidel NEQ sider )
    {
        *ul = tl;
        *ur = tr;
        *ug = 0.5 *( tl + tr );
        *sch = NL_YES;
    }

    /* Exit */

    EXIT:

    return (error);
}
#endif // NLIB_UNUSED
