// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfShape.c: Shape surface routines                                 */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#if NLIB_UNUSED

/**********************************************************************/
/* N_SrfShapeAxialBend: Surface axial bending                         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine bends a NURBS surface toward a cylinder from a 
     bend axis along a  v-isocurve. Given an isocurve S(uc,v), a control
     point Pw[i][j] is  mapped from a center  computed along the surface
     normal  obtained at  (uc,v[i][j]), where  v[i][j] is a  v-parameter
     corresponding  to  Pw[i][j]  (that  is,  the  axis is  obtained  by 
     offsetting the isocurve S(uc,v) a given distance). Only  a strip of 
     the surface defined over [us,ue] is  bent. When  interactive change  
     is required,  only those  quantities  are recomputed  that directly  
     affect the change in the bend. A typical calling example is:

       NL_SURFACE    surP, surQ;
       NL_INDEX      npu, npv, so, eo;
       NL_PARAMETER  us, uc, ue;
       NL_REAL       **vo, d, rad, lam, tol;
       NL_STACKS     SP, SQ;
       ...
       (define surP; get us, uc,..., npv; get d, rad, lam and tol);
       ...
       N_SrfShapeAxialBend(&surP,us,uc,ue,npu,npv,d,rad,lam,tol,NL_PREPARE,
                NL_YES,NL_FLIP,&vo,&so,&eo,&surQ,&SP,&SQ);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeAxialBend");
...
         get new lam;
         N_SrfShapeAxialBend(&surP,us,uc,ue,npu,npv,d,rad,lam,tol,NL_INTERACT,
                  NL_NO,NL_FLIP,&vo,&so,&eo,&surQ,&SP,&SQ);
       }
       N_SrfShapeAxialBend(&surP,us,uc,ue,npu,npv,d,rad,lam,tol,NL_CLEANUP,
                NL_NO,NL_FLOP,&vo,&so,&eo,&surQ,&SP,&SQ);

     THE BENT NL_SURFACE IS STORED IN surQ. ALTHOUGH THE INPUT NL_SURFACE DOES 
     NOT CHANGE EITHER  GEOMETIRICALLY OR PARAMETRICALLY, ITS DEFINITION 
     IS DESTROYED  IN THAT  ITS KNOT  NL_VECTOR IS  REFINED AND  ITS NL_DEGREE 
     IS  RAISED  IF  IT  WAS  NL_LINEAR  IN  THE  U-DIRECTION. IF  THIS  IS 
     UNACCEPTABLE,  EITHER  SAVE  THE  ORIGINAL  NL_SURFACE  OR  APPLY KNOT 
     REMOVAL AND NL_DEGREE REDUCTION AFTER BENDING IS FINISHED. THE NL_SURFACE  
     CAN BE REFINED PRIOR TO SHAPING IN WHICH CASE THE ROUTINE SKIPS THE
     REFINEMENT.


   ACCESS:
   
     surP     , in/out ,  NURBS surface
     us,uc,ue , input  ,  Bend  surface  over  [us,ue]  along  a  v-line 
                          computed at uc (us<=uc<=ue)
     npu,npv  , input  ,  Number of knots to be inserted. Depends on the
                          bend; for large bends, more control points are  
                          needed to obtain a smooth change in shape. The 
                          default is  nu*(p+1)/nv*(q+1), where nu and nv
                          are the  number of  knot spans in  [us,ue] and
                          [V[0],V[s]],    respectively.   ALTHOUGH   THE 
                          NL_SURFACE  IS  BENT IN  THE  U-DIRECTION, IT  IS
                          REFINED IN NL_BOTH DIRECTIONS TO OBTAIN  A SMOOTH 
                          BEND. 
     d,rad    , input  ,  Offset distance and cylinder radius
     lam      , input  ,  Cross ratio for bending. Some special values: 
                            = 1.0: maps poins onto the cylinder
                            = 0.0: maps points to the axis
                            = INF: maps points to surP
     tol      , inpout ,  Knot removal tolerance. For  shaping accuracy, 
                          1% of the surface's size is a good default
     flg      , input  ,  Flag:
                            NL_PREPARE : prepare and do first bend
                            NL_INTERACT: change bend by changing lam
                            NL_CLEANUP : remove unneccesary knots
     ref      , input  ,  Flag:
                            NL_YES: refine surface
                            NL_NO : surface is already refined 
     flp      , input  ,  Flag:
                            NL_FLIP: flip  surface  normal - surface normal 
                                  can flip if  parametrization  changes; 
                                  must flip  back to get  desired effect
                            NL_FLOP: surface normal OK
     vo       , in/out ,  V-parameters corresponding to control points
     so,eo    , in/out ,  U-indexes of local control points
     surQ     , output ,  Bent surface; if surP = surQ, only the NL_PREPARE  
                          and NL_CLEANUP options are available
     SP       , input  ,  surP's stack
     SQ       , input  ,  surQ's and vo's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeAxialBend( NL_SURFACE *surP, NL_PARAMETER us, NL_PARAMETER uc, NL_PARAMETER ue, NL_INDEX npu, NL_INDEX npv, NL_REAL d, NL_REAL rad, NL_REAL lam, NL_REAL tol, NL_FLAG flg, NL_FLAG ref, NL_FLAG flp, NL_REAL *** vo, NL_INDEX *so, NL_INDEX *eo, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeAxialBend");

    NL_FLAG its, error = NL_NO;

    NL_INDEX i, j, n, m, r, s, su = 0, eu = 0, nku, nkv, nkx, nky, fku, lku, nsp;

    NL_DEGREE p, q;

    NL_REAL ** v = NULL, *U, *V, v0, sum, sb, tb, w;

    NL_POINT ** A, ** SD, O, P, Q, R, S, K, L;

    NL_VECTOR SU, SV, N, D, E, F;

    NL_CPOINT ** Pw, ** Qw = NULL;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL, *knx, *kny;

    NL_LINESEG tnP, tnC;

    NL_RMATRIX rma;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    if( uc LT us OR uc GT ue )
        NL_ERROR( NL_INP_ERR );

    if( us GE U[r]OR ue LE U[0] )
        NL_ERROR( NL_INP_ERR );

    if( us LT U[0] )
        us = U[0];

    if( ue GT U[r] )
        ue = U[r];

    switch( flg )
    {
        case NL_PREPARE:

            N_SrfGetKnotVectors( surP, &knu, &knv );
            break;

        case NL_INTERACT:
            if( surP EQ surQ )
                NL_ERROR( NL_INP_ERR );

            su = *so;
            eu = *eo;
            v = *vo;
            N_SrfGetCPts( surQ, &n, &m, &Qw );
            break;

        case NL_CLEANUP:

            su = *so;
            eu = *eo;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Prepare for bend */

    if( flg EQ NL_PREPARE )
    {
        /* Degree elevate if required */

        if( p EQ 1 )
        {
            error = N_SrfElevateDegree( surP, 1, NL_UDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
            N_SrfGetKnotVectors( surP, &knu, &knv );
        }

        /* Refine surface */

        if( ref EQ NL_YES )
        {
            error = N_BasisFindSpan( knu, p, us, NL_LEFT, &su );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpan( knu, p, ue, NL_LEFT, &eu );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knv, q, &nsp );

            nku = (eu - su + 1) * (p + 1);
            nku = NL_MAX( npu, nku );
            nkx = nku + 2;

            nkv = nsp * (q + 1);
            nkv = NL_MAX( npv, nkv );
            nky = nkv + 2;

            knx = N_AllocKnotVectorAndArray( nkx, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            kny = N_AllocKnotVectorAndArray( nky, &SL );

            if( kny EQ NULL )
                NL_QUIT;

            error = N_BasisSplitNLongestSpans( knu, p, us, ue, nku, 1, 1, knx );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisSplitNLongestSpans( knv, q, V[0], V[s], nkv, 1, 1, kny );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, knx, NL_UDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, kny, NL_VDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
            N_SrfGetKnotVectors( surP, &knu, &knv );
        }

        if( surP NEQ surQ )
        {
            N_SrfInitArrays( surQ );
            error = N_SrfCopy( surP, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
        }
        else
        {
            N_SrfGetCPts( surP, &n, &m, &Qw );
        }

        /* Get indexes */

        error = N_BasisFindSpan( knu, p, us, NL_LEFT, &su );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knu, p, ue, NL_LEFT, &eu );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
            su = 0;

        if( ue EQ U[r] )
            eu = n + p + 1;

        /* Compute v-parameters */

        A = N_AllocPt2dArray( eu - p - su - 1, m, &SL );

        if( A EQ NULL )
            NL_QUIT;

        v = N_AllocReal2dArray( eu - p - su - 1, m, SQ );

        if( v EQ NULL )
            NL_QUIT;

        for ( i = su; i < eu - p; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_CPtToPtEuclid( Pw[i][j], &A[i - su][j] );
        }

        for ( i = 0; i < eu - p - su; i++ )
        {
            error = N_FitCalcCrvParamValues( (NL_VOID *)A[i], m, NL_EPOINT, NL_CHORDLENGTH, v[i] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* Output indexes for interactive step */

        *so = su;
        *eo = eu;
        *vo = v;
    }

    /* Interactively bend */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Recover original control points */

        if( surP NEQ surQ )
        {
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Pw[i][j], &Qw[i][j] );
            }
        }

        /* For each row, reposition control points for bend region */

        error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        SD = N_AllocPt2dArray( 1, 1, &SL );

        if( SD EQ NULL )
            NL_QUIT;

        for ( j = 0; j <= m; j++ )
        {
            sum = 0.0;

            for ( i = su; i < eu - p; i++ )
            {
                N_CPtToPtAndW( Qw[i][j], &Q, &w );

                error = N_SrfEvalPtPtDerivNormalFast( surP, uc, v[i - su][j], NL_LEFT, NL_LEFT, &O, &SU, &SV, &N, SD );

                if( error EQ NL_YES )
                    NL_OUT;

                if( flp EQ NL_FLIP )
                    N_VectorReverseInPlace( &N );

                N_VectorBlendPt( d, N, &O );
                N_VectorDiff( Q, O, &D );

                error = N_VectorNormalizeRef( &D );

                if( error EQ NL_YES )
                    NL_OUT;

                N_Combine2Pts( 1.0, O, rad, D, &R );
                N_DistPtPt( O, R, &sb );
                N_DistPtPt( O, Q, &tb );

                if( tb LT NL_MTOL )
                    NL_ERROR( NL_INP_ERR );

                sb = sb / tb;
                tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);

                N_Combine2Pts( 1.0 - tb, O, tb, Q, &Q );
                N_Weight( Q, w, &Qw[i][j] );

                sum += v[i - su][j];
            }

            v0 = sum / ((NL_REAL)eu - (NL_REAL)p - (NL_REAL)su);

            error = N_SrfEvalPtPtDerivNormalFast( surP, uc, v0, NL_LEFT, NL_LEFT, &O, &SU, &SV, &N, SD );

            if( error EQ NL_YES )
                NL_OUT;

            if( flp EQ NL_FLIP )
                N_VectorReverseInPlace( &N );

            N_VectorBlendPt( d, N, &O );

            /* Reattach control points on the left */

            if( su GT 0 )
            {
                /* Compute end tangents */

                error = N_SrfEvalPtPtDerivNormalFast( surP, us, v0, NL_LEFT, NL_LEFT, &S, &SU, &P, &P, SD );

                if( error EQ NL_YES )
                    NL_OUT;

                N_VectorDiff( S, O, &D );

                error = N_VectorNormalizeRef( &D );

                if( error EQ NL_YES )
                    NL_OUT;

                N_Combine2Pts( 1.0, O, rad, D, &R );
                N_DistPtPt( O, R, &sb );
                N_DistPtPt( O, S, &tb );

                if( tb LT NL_MTOL )
                    NL_ERROR( NL_INP_ERR );

                sb = sb / tb;
                tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);

                N_Combine2Pts( 1.0 - tb, O, tb, S, &Q );

                N_VectorCross( D, SU, &SV );
                N_VectorCross( D, SV, &F );

                N_CreateLineStartDirVector( &tnP, S, SU, NL_UNBOUNDED );
                N_CreateLineStartDirVector( &tnC, R, F, NL_UNBOUNDED );

                error = N_IsectLineLine( tnP, tnC, &P, &sb, &tb, &its );

                if( error EQ NL_YES )
                    NL_OUT;

                if( its EQ NL_TRUE )
                {
                    N_VectorDiff( Q, P, &E );

                    error = N_VectorNormalizeRef( &E );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorCross( D, E, &F );

                    if( NOT N_VectorsAreParallel( SV, F ) )
                        N_VectorReverseInPlace( &E );
                }
                else
                {
                    N_VectorCopy( SU, &E );
                }

                /* Get transformation matrix and map points */

                N_VectorCross( SV, SU, &K );
                N_VectorCross( SV, E, &L );

                error = N_CreateTransformMatrixFromAxes( S, SV, SU, K, Q, SV, E, L, &rma, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i < su; i++ )
                    N_TransformCPt( Qw[i][j], &rma, &Qw[i][j] );
            }

            if( eu LT n + p + 1 )
            {
                /* Compute end tangents */

                error = N_SrfEvalPtPtDerivNormalFast( surP, ue, v0, NL_LEFT, NL_LEFT, &S, &SU, &P, &P, SD );

                if( error EQ NL_YES )
                    NL_OUT;

                N_VectorDiff( S, O, &D );

                error = N_VectorNormalizeRef( &D );

                if( error EQ NL_YES )
                    NL_OUT;

                N_Combine2Pts( 1.0, O, rad, D, &R );
                N_DistPtPt( O, R, &sb );
                N_DistPtPt( O, S, &tb );

                if( tb LT NL_MTOL )
                    NL_ERROR( NL_INP_ERR );

                sb = sb / tb;
                tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);

                N_Combine2Pts( 1.0 - tb, O, tb, S, &Q );

                N_VectorCross( D, SU, &SV );
                N_VectorCross( D, SV, &F );

                N_CreateLineStartDirVector( &tnP, S, SU, NL_UNBOUNDED );
                N_CreateLineStartDirVector( &tnC, R, F, NL_UNBOUNDED );

                error = N_IsectLineLine( tnP, tnC, &P, &sb, &tb, &its );

                if( error EQ NL_YES )
                    NL_OUT;

                if( its EQ NL_TRUE )
                {
                    N_VectorDiff( Q, P, &E );

                    error = N_VectorNormalizeRef( &E );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorCross( D, E, &F );

                    if( NOT N_VectorsAreParallel( SV, F ) )
                        N_VectorReverseInPlace( &E );
                }
                else
                {
                    N_VectorCopy( SU, &E );
                }

                /* Get transformation matrix and map points */

                N_VectorCross( SV, SU, &K );
                N_VectorCross( SV, E, &L );

                error = N_CreateTransformMatrixFromAxes( S, SV, SU, K, Q, SV, E, L, &rma, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = eu - p; i <= n; i++ )
                    N_TransformCPt( Qw[i][j], &rma, &Qw[i][j] );
            }
        }
    }

    /* Remove unnecessary knots */

    if( flg EQ NL_CLEANUP )
    {
        if( su GE p )
            fku = su + (p + 1) / 2;
        else
            fku = p + 1;

        if( eu LE n )
            lku = eu - (p + 2) / 2;
        else
            lku = n;

        error = N_SrfShapeRemoveKnots( surQ, tol, fku, lku, q + 1, m, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:
    ;
    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_SrfShapeApproxPts: Shape surface to approximate given points                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies the shape of a given base surface to
     approximate a  set of points. It performs  constrained  shaping to
     produce an approximating surface. A typical calling example is:

       NL_FLAG     ftl;
       NL_SURFACE  surB, surS;
       NL_POINT    *P;
       NL_REAL     tol;
       NL_INDEX    np, dsu, deu, dsv, dev;
       NL_STACKS   SG;
       ...
       (get base surface surB, points P, and tol);
       ...
       N_SrfInitArrays(&surS);
       N_SrfShapeApproxPts(&surB,P,np,dsu,deu,dsv,dev,tol,&surS,&ftl,&SG);

     MEMORY FOR THE NL_SURFACE STRUCTURE OF surS  MUST BE ALLOCATED IN THE
     CALLING ROUTINE (AS SHOWN  IN THE  EXAMPLE ABOVE).


   ACCESS:
   
     surB    , input  ,  Base surface
     P       , input  ,  Random points output surface must approximate
     np      , input  ,  Highest index in P
     dsu,dsv , input  ,  Start derivative constraint; 0,..,dsu/dsv deri-
                         vatives not to change at the start
     deu,dev , input  ,  End derivative  constraint;  0,..,deu/dev deri-
                         vatives not to change at the end
     tol     , input  ,  The filtered average error of the approximation
                         must be  less than  tol. The shaping  iteration 
                         stops when:
                         1. the average error is less than tol;
                         2. there is no improvement in  two  consecutive
                            iterates; or
                         3. the error gets worse
     surS    , output ,  Shaped surface approximating P[i], i=0,...,np
     ftl     , output ,  Flag:
                           NL_YES: approximant is within tol
                           NL_NO : error  condition  is not  satisfied, but 
                                the best surface is returned
     SG      , input  ,  surS' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeApproxPts
  (NL_SURFACE *surB,   /* in : Base Surface - defines point uv-mapping and */
                       /*      output surS's 1st iteration degrees, knotVectors, controlPoint counts */
   NL_POINT   *P,      /* in : cloud-of-points, sized:[np+1] */
   NL_INDEX    np,     /* in : highest index in P */
   NL_INDEX    dsu,    /* in : number of derivatives to constrain at v=vmin boundary 0=pos, 1=cross-tangent, etc. */
   NL_INDEX    deu,    /* in : number of derivatives to constrain at v=vmax boundary 0=pos, 1=cross-tangent, etc. */
   NL_INDEX    dsv,    /* in : number of derivatives to constrain at u=umin boundary 0=pos, 1=cross-tangent, etc. */
   NL_INDEX    dev,    /* in : number of derivatives to constrain at u=umax boundary 0=pos, 1=cross-tangent, etc. */
   NL_REAL     tol,    /* in : iteration stops when Avg Error < tol, or convergence ceases */
   NL_SURFACE *surS,   /* out: approximating surface - may have more knots and control points than surB as needed to meet tol */
   NL_FLAG    *ftl,    /* out: NL_YES = approximant is within tol */
                       /*      NL_NO  = tolerance was not met but best surface is returned */
   NL_STACKS  *SG )    /* in : surS's memory stack */
{

    NL_FLAG fin, error = NL_NO;

    NL_INDEX ** KU, ** KV, *I, *J, *K, *nd, *iu, *jv, i, j, k, m, n, r, s, dn, dm, nI, mJ, ni, mj, lo, hi, nf, nt, loop;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *ut, *vt, *ui, *vi, *d, *eps, dis, sum, emax, emp, emc, eav; 

    NL_POINT *T, R;

    NL_VECTOR ** DD, VV;

    NL_KNOTVECTOR knu, knv;

    NL_RMATRIX rma;

    NL_SURFACE surA, surI;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL dtl = 0.001;
    NL_PRIVATE NL_REAL efp = 0.05;
    NL_PRIVATE NL_REAL cie = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Prepare for shaping and get local notation */

    N_SrfInitArrays( &surI );
    error = N_SrfCopy( surB, &surI, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnots( surB, &r, &s, &U, &V );
    N_SrfGetArraySizes( surB, &n, &m, &r, &s );
    N_SrfGetDegrees( surB, &p, &q );

    nI = n + np;
    mJ = m + np;
    dn = NL_MAX( 1, n / 2 );
    dm = NL_MAX( 1, m / 2 );
    nf = NL_MAX( 1, (NL_INDEX)(efp * np) );
    *ftl = NL_NO;
    emc = NL_BIGD;
    emp = NL_BIGD;
    fin = NL_NO;

    /* Allocate memory */

    I = N_AllocInt1dArray( nI, &SL );

    if( I EQ NULL )
        NL_QUIT;

    J = N_AllocInt1dArray( mJ, &SL );

    if( J EQ NULL )
        NL_QUIT;

    KU = N_AllocInt2dArray( np, 0, &SL );

    if( KU EQ NULL )
        NL_QUIT;

    KV = N_AllocInt2dArray( np, 0, &SL );

    if( KV EQ NULL )
        NL_QUIT;

    nd = N_AllocInt1dArray( np, &SL );

    if( nd EQ NULL )
        NL_QUIT;

    iu = N_AllocInt1dArray( np, &SL );

    if( iu EQ NULL )
        NL_QUIT;

    jv = N_AllocInt1dArray( np, &SL );

    if( jv EQ NULL )
        NL_QUIT;

    DD = N_AllocPt2dArray( np, 0, &SL );

    if( DD EQ NULL )
        NL_QUIT;

    d = N_AllocReal1dArray( np, &SL );

    if( d EQ NULL )
        NL_QUIT;

    eps = N_AllocReal1dArray( np, &SL );

    if( eps EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= np; i++ )
        KU[i][0] = KV[i][0] = nd[i] = 0;

    /* Get the parameters and the difference vectors */

    error = N_SrfProjectPts( surB, P, np, NL_NO, NL_YES, dtl, &T, &ut, &vt, &nt, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= nt; i++ )
    {
        N_SrfEvalPt( surB, ut[i], vt[i], NL_LEFT, NL_LEFT, &R );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( T[i], R, &DD[i][0] );

        iu[i] = jv[i] = i;
    }

    /*********************/
    /* Iteratively shape */
    /*********************/

    /* First check if input surface is already within tolerance */

    loop = 1;

    /* Compute filtered average error */

    for ( i = 0; i <= nt; i++ )
    {
        error = N_SrfEvalPt( &surI, ut[i], vt[i], NL_LEFT, NL_LEFT, &R );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( T[i], R, &VV );
        N_VectorMagnitude( VV, &d[i] );
    }

    emax = 0.0;

    for ( i = 0; i <= nt; i++ )
    {
        lo = NL_MAX( 0, i - nf );
        hi = NL_MIN( np, i + nf );

        sum = 0.0;

        for ( j = lo; j <= hi; j++ )
            sum += d[j];
        eps[i] = sum / ((NL_REAL)hi - (NL_REAL)lo + 1.0);

        if( eps[i]GT emax )
            emax = eps[i];
    }

    if( emax LT tol )
    {
        fin = NL_YES;
        *ftl = NL_YES;
        loop = 0;

        N_SrfInitArrays( surS );
        error = N_SrfCopy( &surI, surS, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* While tolerance requirement is not satisfied do */

    while( loop )
    {
        /* Shape base surface */

        for ( k = 0; k <= dsu; k++ )
            I[k] = 0;

        for ( k = 0; k <= deu; k++ )
            I[n - k] = 0;

        for ( k = dsu + 1; k <= n - deu - 1; k++ )
            I[k] = 1;

        for ( k = 0; k <= dsv; k++ )
            J[k] = 0;

        for ( k = 0; k <= dev; k++ )
            J[m - k] = 0;

        for ( k = dsv + 1; k <= m - dev - 1; k++ )
            J[k] = 1;

        N_SrfInitArrays( &surA );
        error = N_SrfCopy( &surI, &surA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfShapeDerivConstraints( &surA, ut, vt, nt, nt, iu, jv, nt, I, J, DD, KU, KV, nd, NL_PREPARE, &rma, &K, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Compute filtered average error */

        for ( i = 0; i <= nt; i++ )
        {
            error = N_SrfEvalPt( &surA, ut[i], vt[i], NL_LEFT, NL_LEFT, &R );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDiff( T[i], R, &VV );
            N_VectorMagnitude( VV, &d[i] );
        }

        emax = eav = 0.0;

        for ( i = 0; i <= nt; i++ )
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
        eav /= (NL_REAL)np + 1.0;

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
            N_SrfInitArrays( surS );
            error = N_SrfCopy( &surA, surS, SG );

            if( error EQ NL_YES )
                NL_OUT;

            break;
        }

        /* Add extra knots */

        j = NL_MIN( n + dn, (n + nt) / 2 );
        ni = j - n + 1;

        i = NL_MIN( m + dm, (m + nt) / 2 );
        mj = i - m + 1;

        error = N_KnotsAdd( U, r, p, ni, &ui, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_KnotsAdd( V, s, q, mj, &vi, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorFromRealArray( &knu, ui, ni - 1 );
        N_KnotVectorFromRealArray( &knv, vi, mj - 1 );

        /* Insert knots */

        error = N_SrfInsertKnots( &surI, &knu, NL_UDIR, &surI, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfInsertKnots( &surI, &knv, NL_VDIR, &surI, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetKnots( &surI, &r, &s, &U, &V );
        N_SrfGetArraySizes( &surI, &n, &m, &r, &s );

        if( n GT nI )
        {
            j = NL_MAX( n + nI, nI + nI );

            error = N_Realloc1dIntArray( &I, nI, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            nI = j;
        }

        if( m GT mJ )
        {
            j = NL_MAX( m + mJ, mJ + mJ );

            error = N_Realloc1dIntArray( &J, mJ, j, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            mJ = j;
        }

        emp = emc;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_SRFSHAPEAXIALDEFORM: Axial deformations of surfaces                           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:


     This  shaping routine  modifies the  shape of a surface by applying 
     the following axial deformations:

       NL_PINCH: {x|y|z} coordinate is scaled according to s({x|y|z})
       NL_TAPER: P is scaled according to s({x|y|z})
       NL_TWIST: P is rotated according to s({x|y|z})
       NL_SHEAR: {x|y|z} coordinate is translated according to s({x|y|z})

     where {x|y|z} means x, y or z  coordinate, s({x|y|z}) is a function  
     of  either  x, y or z, and P is a non-weighted control point. Also, 
     if the  point P is scaled according to, say, s(y), then  only its x 
     and z  coordinates  change. The  scaling  function  is  given  as a 
     B-spline function. A typical calling example is:
  
       NL_SURFACE  sur;
       NL_REAL     a;
       NL_CFUN     cfn;
       ...
       (get sur, cfn and a);
       ...
       N_SrfShapeAxialDeform(&sur,&cfn,a,NL_TAPER,NL_XDIR,NL_YCRD);

     The transformation is performed in-place, i.e. the original surface
     is destroyed.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     cfn , input  ,  Shape function
     a   , input  ,  Scalar factor (amplitude)
     tra , input  ,  Flag:
                       NL_PINCH: surface is pinched
                       NL_TAPER: surface is tapered
                       NL_TWIST: surface is twisted
                       NL_SHEAR: surface is sheared
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

NL_FLAG N_SrfShapeAxialDeform( NL_SURFACE *sur, NL_CFUN *cfn, NL_REAL a, NL_FLAG tra, NL_FLAG dir, NL_FLAG cor )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL w;

    NL_POINT P;

    NL_CPOINT ** Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notations */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Shape surface */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtAndW( Pw[i][j], &P, &w );

            error = N_TransformPtWithShapeFuncAndScale( &P, cfn, a, tra, dir, cor );

            if( error EQ NL_YES )
                NL_OUT;

            N_Weight( P, w, &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_SrfShapeDerivConstraints: Constraint-based surface shaping with int or appr.       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies the shape of a surface by  satisfying 
     a set of derivative constraints, or  approximating  them  with  the 
     minimum  amount of  change in the  position of  control points. The 
     derivative constraints are given at selected parameter values. When 
     interactive shaping is required, only those quantities are computed 
     that  directly  affect the change in the surface's shape. A typical 
     calling example is:

       NL_SURFACE    sur;
       NL_INDEX      **KU, **KV, *I, *J, *K, *nd, *iu, *jv, nu, nv, np; 
       NL_PARAMETER  *u, *v;
       NL_VECTOR     **DD;
       NL_RMATRIX    rma;
       NL_STACKS     SM;
       ...
       (define sur; get u, v, I, J, DD, KU, KV, iu, jv and nd);
       ...
       N_SrfShapeDerivConstraints(&sur,u,v,nu,nv,iu,jv,np,I,J,DD,KU,KV,nd,NL_PREPARE,
                &rma,&K,&SM);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeDerivConstraints");
...
         get new DD;
         N_SrfShapeDerivConstraints(&sur,u,v,nu,nv,iu,jv,np,I,J,DD,KU,KV,nd,NL_INTERACT,
                  &rma,&K,&SM);
       }

     THE SHAPING IS DONE IN-PLACE, IE THE ORIGINAL NL_SURFACE IS DESTROYED.
     THE NL_SURFACE IS UPDATED EITHER PRECISELY OR APPROXIMATELY.  THAT IS, 
     IF THE SYSTEM IS FULLY OR UNDER-CONSTRAINED, THE CONDITIONS ARE MET
     PRECISELY. IF THE  SYSTEM IS  NL_OVER-CONSTRAINED, THE  CONDITIONS ARE 
     MET APPROXIMATELY IN THE LEAST-SQUARES SENSE.


   ACCESS:
   
     sur   , in/out ,  NURBS surface
     u,v   , input  ,  Parameters where constraints are assumed
     nu,nv , input  ,  Highest indexes in u and v
     iu,jv , input  ,  Arrays to define parameter pairs. Example: if the  
                       following pairs are selected
                         (u[0],v[0]),(u[1],v[1]),(u[1],v[2])
                       then
                         iu[3] = {0,1,1}
                         jv[3] = {0,1,2}
     np    , index  ,  Highest index in iu and jv
     I,J   , input  ,  Arrays: <I[k],J[l]> = 
                         <1,1>             : Pw[k][l] IS to change
                         <0,0>|<0,1>|<1,0> : Pw[k][l] is NOT to change
     DD    , input  ,  Derivative differences (constraints): DD[i][j] is
                       the j-th  constraint  assumed at  the i-th  (u,v) 
                       pair. DD does not have to be a 2-D array. It  can 
                       be an  array  of  pointers  pointing to arrays of 
                       different length. Example:
                         DD[i][0] = point (0-th derivative) change
                         DD[i][1] = u-derivative change
                         DD[i][2] = mixed derivative change
     KU,KV , input  ,  Types  of  constraints:  KU[i][k]  and   KV[i][l] 
                       specify that at the i-th (u,v) pair the KU[i][k]-
                       th derivative is  constrained in the u-direction,
                       and the KV[i][l]-th derivative is constrained in
                       the v-direction. Example (see DD above):
                         KU[i][0] = 0  KV[i][0] = 0
                         KU[i][1] = 1  KV[i][1] = 0 
                         KU[i][2] = 1  KV[i][2] = 1 
     nd    , input  ,  Highest indexes in the arrays pointed to by DD[i] 
                       or KU[i] or KV[i] is nd[i]. In the example above,
                       nd[i] = 2.
     flg   , input  ,  Flag:
                         NL_PREPARE : get some entities and shape
                         NL_INTERACT: change shape by changing DD
     rma   , in/out ,  Matrix defining control point changes
     K     , in/out ,  1-D indexes of control points that change
     SG    , input  ,  rma's and K's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeDerivConstraints( NL_SURFACE *sur, NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX nu, NL_INDEX nv, NL_INDEX *iu, NL_INDEX *jv, NL_INDEX np, NL_INDEX *I, NL_INDEX *J, NL_VECTOR ** DD, NL_INDEX ** KU, NL_INDEX ** KV, NL_INDEX *nd, NL_FLAG flg, NL_RMATRIX *rma, NL_INDEX ** K, NL_STACKS *SG )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeDerivConstraints");

    NL_FLAG error = NL_NO;

    NL_INDEX ** M, *KL, *O, *usp, *vsp, *udh, *vdh, spu, spv, udr, vdr, i, j, k, l, n, m, r, s, nb, mb, row, col, ii, jj;

    NL_DEGREE p, q;

    NL_BASISDERIVATIVES BD;

    NL_REAL ** B, *U, *V, w;

    NL_POINT *DP, P;

    NL_CPOINT ** Pw;

    NL_VECTOR *DV;

    NL_KNOTVECTOR *knu, *knv;

    NL_RMATRIX rmb, rmt, rbt, rmi;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    if( u[0]LT U[0]OR u[nu]GT U[r] )
        NL_ERROR( NL_PAR_ERR );

    if( v[0]LT V[0]OR v[nv]GT V[s] )
        NL_ERROR( NL_PAR_ERR );

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
        N_SrfGetKnotVectors( sur, &knu, &knv );

        /* Get span arrays and adjust I and J arrays */

        O = N_AllocInt1dArray( NL_MAX( n, m ), &SL );

        if( O EQ NULL )
            NL_QUIT;

        usp = N_AllocInt1dArray( nu, &SL );

        if( usp EQ NULL )
            NL_QUIT;

        vsp = N_AllocInt1dArray( nv, &SL );

        if( vsp EQ NULL )
            NL_QUIT;

        k = 0;

        for ( i = 0; i <= n; i++ )
            O[i] = 0;

        for ( i = 0; i <= nu; i++ )
        {
            error = N_BasisFindSpan( knu, p, u[i], NL_LEFT, &usp[i] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = usp[i] - p; j <= usp[i]; j++ )
                O[j] = 1;
        }

        for ( i = 0; i <= n; i++ )
        {
            I[i] = I[i] * O[i];

            if( I[i]EQ 1 )
                k++;
        }

        l = 0;

        for ( j = 0; j <= m; j++ )
            O[j] = 0;

        for ( j = 0; j <= nv; j++ )
        {
            error = N_BasisFindSpan( knv, q, v[j], NL_LEFT, &vsp[j] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = vsp[j] - q; i <= vsp[j]; i++ )
                O[i] = 1;
        }

        for ( j = 0; j <= m; j++ )
        {
            J[j] = J[j] * O[j];

            if( J[j]EQ 1 )
                l++;
        }

        /* Mark control points that change and get 1-D indexing */

        M = N_AllocInt2dArray( n, m, &SL );

        if( M EQ NULL )
            NL_QUIT;

        KL = N_AllocInt1dArray( k * l, SG );

        if( KL EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                M[i][j] = -1;
        }

        mb = -1;

        for ( k = 0; k <= np; k++ )
        {
            for ( i = 0; i <= p; i++ )
            {
                ii = usp[iu[k]] - p + i;

                for ( j = 0; j <= q; j++ )
                {
                    jj = vsp[jv[k]] - q + j;

                    if( I[ii]*J[jj]GT 0 )
                    {
                        if( M[ii][jj]LT 0 )
                        {
                            mb++;
                            M[ii][jj] = mb;
                            KL[mb] = ii * (m + 1) + jj;
                        }
                    }
                }
            }
        }

        /* Get highest derivatives, and number of constraints */

        udh = N_AllocInt1dArray( np, &SL );

        if( udh EQ NULL )
            NL_QUIT;

        vdh = N_AllocInt1dArray( np, &SL );

        if( vdh EQ NULL )
            NL_QUIT;

        nb = -1;
        k = 0;
        l = 0;

        for ( i = 0; i <= np; i++ )
        {
            udh[i] = 0;
            vdh[i] = 0;

            for ( j = 0; j <= nd[i]; j++ )
            {
                if( KU[i][j]GT udh[i] )
                    udh[i] = KU[i][j];

                if( KV[i][j]GT vdh[i] )
                    vdh[i] = KV[i][j];
            }

            if( udh[i]GT k )
                k = udh[i];

            if( vdh[i]GT l )
                l = vdh[i];

            nb += nd[i] + 1;
        }

        /* Get matrix containing required derivatives */

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

        for ( k = 0; k <= np; k++ )
        {
            error = N_SrfBasisDerivs( sur, u[iu[k]], v[jv[k]], NL_LEFT, NL_LEFT, NL_FALSE, udh[k], vdh[k], BD, &spu, &spv );

            if( error EQ NL_YES )
                NL_OUT;

            for ( l = 0; l <= nd[k]; l++ )
            {
                row++;
                udr = KU[k][l];
                vdr = KV[k][l];

                for ( i = 0; i <= p; i++ )
                {
                    ii = spu - p + i;

                    for ( j = 0; j <= q; j++ )
                    {
                        jj = spv - q + j;
                        col = M[ii][jj];

                        if( col GE 0 )
                            B[row][col] = BD[udr][vdr][i][j];
                    }
                }
            }
        }

        /* Compute matrix */

        if( nb EQ mb )
        {
            /* Fully-determined system -> Precise solution */

            N_InitRealMatrix( rma );
            error = N_RealMatrixInverseSVD( &rmb, rma, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else if( nb LT mb )
        {
            /* Under-determined system -> Minimum length precise solution */

            N_InitRealMatrix( &rmt );
            error = N_RealMatrixTranspose( &rmb, &rmt, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rbt );
            error = N_RealMatrixTransposeMultiply( &rmb, &rbt, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( &rmi );
            error = N_RealMatrixInverseSVD( &rbt, &rmi, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( rma );
            error = N_RealMatrixMultiply( &rmt, &rmi, rma, SG );

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
            error = N_RealMatrixInverseSVD( &rbt, &rmi, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_InitRealMatrix( rma );
            error = N_RealMatrixMultiply( &rmi, &rmt, rma, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* Output index array for interactive step */

        *K = KL;
    }

    /* Interactively shape */

    if( flg EQ NL_PREPARE OR flg EQ NL_INTERACT )
    {
        KL = *K;

        /* Get right hand side of constraints */

        N_GetMaxIndexRealMatrix( rma, &mb, &nb );

        DV = N_AllocPt1dArray( nb, &SL );

        if( DV EQ NULL )
            NL_QUIT;

        l = 0;

        for ( k = 0; k <= np; k++ )
        {
            for ( i = 0; i <= nd[k]; i++ )
            {
                N_CopyPt( DD[k][i], &DV[l] );
                l++;
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

        for ( l = 0; l <= mb; l++ )
        {
            i = KL[l] / (m + 1);
            j = KL[l] % (m + 1);

            N_CPtToPtAndW( Pw[i][j], &P, &w );
            N_Sum2Pts( P, DP[l], &P );
            N_Weight( P, w, &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_SrfShapeCentralBend: Surface central bending                     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine  bends a NURBS surface  toward a sphere from a
     bend center positioned  along the surface normal, which is computed 
     at given  parameters  uc and  vc. Only a  region of  the surface is 
     bent; it is defined by a closed and convex polygon in the parameter
     space. Each  control point is  associated with a  pair of parameter 
     values that are  checked against the  polygonal region. If they are 
     in, the  control point is  mapped. If not, it is  reattached to the
     bent  region. When  interactive  change  is  required,  only  those 
     quantities are  recomputed that  directly affect  the change in the 
     bend. A typical calling example is:

       NL_SURFACE    surP, surQ;
       EPOLGON    ppl;
       NL_INDEX      npu, npv;
       NL_PARAMETER  uc, vc;
       NL_REAL       *u, *v, d, rad, tol, lam;
       NL_BOOLEAN    **IN;
       NL_STACKS     SP, SQ;
       ...
       (define surP; get ppl, npu, npv; get uc, vc, d, rad, lam, tol);
       ...
       N_SrfShapeCentralBend(&surP,&ppl,uc,vc,npu,npv,d,rad,lam,tol,NL_PREPARE,NL_YES,NL_FLIP,
                &u,&v,&IN,&surQ,&SP,&SQ);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeCentralBend");
...
         get new lam;
         ...
         N_SrfShapeCentralBend(&surP,&ppl,uc,vc,npu,npv,d,rad,lam,tol,NL_INTERACT,NL_NO,
                  NL_FLOP,&u,&v,&IN,&surQ,&SP,&SQ);
         ...
       }
       N_SrfShapeCentralBend(&surP,&ppl,uc,vc,npu,npv,d,rad,lam,tol,NL_CLEANUP,NL_NO,NL_FLOP,
                &u,&v,&IN,&surQ,&SP,&SQ);

     THE BENT  NL_SURFACE IS  STORED IN  surQ.  ALTHOUGH  THE INPUT NL_SURFACE 
     DOES  NOT  CHANGE  EITHER  GEOMETRICALLY  OR  PARAMETRICALLY,   ITS 
     DEFINITION IS  DESTROYED IN  THAT ITS KNOT  VECTORS ARE REFINED AND 
     ITS  DEGREES  ARE  RAISED  IN  THE  NL_LINEAR  DIRECTION(S). IF IT  IS   
     UNACCEPTABLE,  SAVE  THE  ORIGINAL  NL_SURFACE OR  REMOVE  UNNECESSARY 
     KNOTS  AND  REDUCE THE  NL_DEGREE. IF  surP = surQ, NL_NO  INTERACTION IS 
     ALLOWED, I.E. ONLY THE "NL_PREPARE" AND "NL_CLEANUP" FLAGS ARE NL_USED. THIS 
     CAPABILITY IS  USEFUL TO  PERFORM  A SERIES OF  WARPS ON THE  INPUT 
     NL_SURFACE surP. THE  NL_SURFACE CAN BE REFINED  BEFORE  SHAPING. IN THIS 
     CASE THE REFINEMENT IS SKIPPED IN THE "NL_PREPARE" STAGE.


   ACCESS:
   
     surP    , in/out ,  NURBS surface
     ppl     , input  ,  Closed polygon  residing in the parameter space 
                         of surP. It must at least partially overlap the 
                         knot  rectangle. IF THE ENTIRE NL_SURFACE IS TO BE
                         BENT,  DEFINE  ppl   LARGER   THAN   THE   KNOT 
                         NL_RECTANGLE. THIS ENSURES THAT ALL CONTROL NL_POINTS
                         ARE MAPPED TO THE SPHERE.
     uc,vc   , input  ,  Parameters of center point = S(uc,vc)+d*N (N is
                         computed at (uc,vc) as well)
     npu,npv , input  ,  Number of u/v knots to be inserted. Depends  on 
                         the bend; for  large bends  more control points 
                         are needed to obtain a smooth  change in shape. 
                         The default is  nsu*(p+1)/nsv*(q+1), where  nsu
                         and nsv are the number of spans in the two knot
                         vectors.
     d,rad   , input  ,  Offset distance and sphere radius
     lam     , input  ,  Cross ratio. Some special values:
                           = 1.0: maps points onto the sphere
                           = 0.0: maps points to the center of sphere
                           = INF: maps points to surP
     tol     , input  ,  Knot removal  tolerance. For  shaping accuracy, 
                         1% of the surface's size is a good default
     flg     , input  ,  Flag:
                           NL_PREPARE : get some entities and do first bend
                           NL_INTERACT: change bend by changing lam
                           NL_CLEANUP : remove unneccesary knots
     ref     , input  ,  Flag:
                           NL_YES: refine surface
                           NL_NO : surface is already refined
     flp     , input  ,  Flag:
                           NL_FLIP: flip surface normal
                           NL_FLOP: surface normal is OK
     u,v     , in/out ,  Local parametrizations
     IN      , in/out ,  Boolean array  indicating which  local  control 
                         point is inside ppl
     surQ    , output ,  Bent  surface; if  surP=surQ, only the  NL_PREPARE  
                         and NL_CLEANUP options are allowed
     SP      , input  ,  surP's stack
     SQ      , input  ,  Stack holding pointers surQ, u, v and IN
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeCentralBend( NL_SURFACE *surP, NL_EPOLYGON *ppl, NL_PARAMETER uc, NL_PARAMETER vc, NL_INDEX npu, NL_INDEX npv, NL_REAL d, NL_REAL rad, NL_REAL lam, NL_REAL tol, NL_FLAG flg, NL_FLAG ref, NL_FLAG flp, NL_REAL ** u, NL_REAL ** v, NL_BOOLEAN *** IN, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeCentralBend");

    NL_FLAG its, error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, nku, nkv, nkx, nky, nsu, nsv;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *ul = NULL, *vl = NULL, dx, dy, x, y, sb, tb, w;

    NL_POINT ** A, ** SD, C, O, P, Q, R, S, Sh;

    NL_VECTOR SU, SV, N, SUh, SVh, Nh, D, M, T;

    NL_CPOINT ** Pw, ** Qw = NULL;

    NL_LINESEG tnP, tnS;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL, *knx, *kny;

    NL_BOOLEAN ** PIP = NULL;

    NL_RMATRIX rma;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    if( uc LT U[0]OR uc GT U[r] )
        NL_ERROR( NL_INP_ERR );

    if( vc LT V[0]OR vc GT V[s] )
        NL_ERROR( NL_INP_ERR );

    switch( flg )
    {
        case NL_PREPARE:

            N_SrfGetKnotVectors( surP, &knu, &knv );
            break;

        case NL_INTERACT:
            if( surP EQ surQ )
                NL_ERROR( NL_INP_ERR );

            ul = *u;
            vl = *v;
            PIP = *IN;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
            break;

        case NL_CLEANUP:

            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    SD = N_AllocPt2dArray( 1, 1, &SL );

    /* Prepare for bend */

    if( flg EQ NL_PREPARE )
    {
        /* Degree elevate */

        if( p EQ 1 )
        {
            error = N_SrfElevateDegree( surP, 1, NL_UDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
            N_SrfGetKnotVectors( surP, &knu, &knv );
        }

        if( q EQ 1 )
        {
            error = N_SrfElevateDegree( surP, 1, NL_VDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
            N_SrfGetKnotVectors( surP, &knu, &knv );
        }

        /* Refine surface */

        if( ref EQ NL_YES )
        {
            N_BasisGetSpanCount( knu, p, &nsu );
            N_BasisGetSpanCount( knv, q, &nsv );

            nku = nsu * (p + 1);
            nku = NL_MAX( npu, nku );
            nkx = nku + 2;

            nkv = nsv * (q + 1);
            nkv = NL_MAX( npv, nkv );
            nky = nkv + 2;

            knx = N_AllocKnotVectorAndArray( nkx, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            kny = N_AllocKnotVectorAndArray( nky, &SL );

            if( kny EQ NULL )
                NL_QUIT;

            error = N_BasisSplitNLongestSpans( knu, p, U[0], U[r], nku, 1, 1, knx );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisSplitNLongestSpans( knv, q, V[0], V[s], nkv, 1, 1, kny );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, knx, NL_UDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, kny, NL_VDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
            N_SrfGetKnotVectors( surP, &knu, &knv );
        }

        if( surP NEQ surQ )
        {
            N_SrfInitArrays( surQ );
            error = N_SrfCopy( surP, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
        }
        else
        {
            N_SrfGetCPts( surP, &n, &m, &Qw );
        }

        /* Get local parametrization */

        ul = N_AllocReal1dArray( n, SQ );

        if( ul EQ NULL )
            NL_QUIT;

        vl = N_AllocReal1dArray( m, SQ );

        if( vl EQ NULL )
            NL_QUIT;

        A = N_AllocPt2dArray( n, m, &SL );

        if( A EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_CPtToPtEuclid( Pw[i][j], &A[i][j] );
        }

        error = N_FitCalcSrfParamValues( (NL_VOID ** )A, n, m, NL_EPOINT, NL_CHORDLENGTH, ul, vl );

        if( error EQ NL_YES )
            NL_OUT;

        /* Compute Boolean array */

        PIP = N_AllocInt2dArray( n, m, SQ );

        if( PIP EQ NULL )
            NL_QUIT;

        dx = U[r] - U[0];
        dy = V[s] - V[0];

        for ( i = 0; i <= n; i++ )
        {
            x = U[0] + ul[i] * dx;

            for ( j = 0; j <= m; j++ )
            {
                y = V[0] + vl[j] * dy;

                N_PtFromXYZ( x, y, 0.0, &A[i][j] );
            }
        }

        error = N_PtsAreInPolygon( ppl, A, n, m, PIP );

        if( error EQ NL_YES )
            NL_OUT;

        /* Output some parameters for interactive step */

        *u = ul;
        *v = vl;
        *IN = PIP;
    }

    /* Interactively bend */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Recover original control points */

        if( surP NEQ surQ )
        {
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                    N_CopyCPt( Pw[i][j], &Qw[i][j] );
            }
        }

        /* Map control points */

        error = N_SrfEvalPtPtDerivNormalFast( surP, uc, vc, NL_LEFT, NL_LEFT, &O, &SU, &SV, &N, SD );

        if( error EQ NL_YES )
            NL_OUT;

        if( flp EQ NL_FLIP )
            N_VectorReverseInPlace( &N );

        N_VectorBlendPt( d, N, &O );

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
            {
                if( PIP[i][j] )
                {
                    N_CPtToPtAndW( Qw[i][j], &Q, &w );
                    N_VectorDiff( Q, O, &D );

                    error = N_VectorNormalizeRef( &D );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_Combine2Pts( 1.0, O, rad, D, &R );
                    N_DistPtPt( O, R, &sb );
                    N_DistPtPt( O, Q, &tb );

                    if( tb LT NL_MTOL )
                        NL_ERROR( NL_INP_ERR );

                    sb = sb / tb;
                    tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);

                    N_Combine2Pts( 1.0 - tb, O, tb, Q, &Q );
                    N_Weight( Q, w, &Qw[i][j] );
                }
            }
        }

        /* Reattach unmapped control points */

        error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_PtFromXYZ( uc, vc, 0.0, &C );

        k = 0;

        for ( j = 0; j <= m; j++ )
        {
            for ( i = 0; i <= n; i++ )
            {
                if( NOT PIP[i][j] )
                {
                    /* Get boundary point and map it */

                    N_PtFromXYZ( ul[i], vl[j], 0.0, &P );

                    error = N_IsectLinePolygon( ppl, C, P, k, &Q, &l, &its );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( its EQ NL_FALSE )
                        continue;
                    else
                        k = l;

                    N_PtToXYZ( Q, &x, &y, &w );

                    if( x LT U[0] )
                        x = U[0];

                    if( x GT U[r] )
                        x = U[r];

                    if( y LT V[0] )
                        y = V[0];

                    if( y GT V[s] )
                        y = V[s];

                    error = N_SrfEvalPtPtDerivNormalFast( surP, x, y, NL_LEFT, NL_LEFT, &S, &SU, &SV, &N, SD );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorDiff( S, O, &D );

                    error = N_VectorNormalizeRef( &D );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_Combine2Pts( 1.0, O, rad, D, &R );
                    N_DistPtPt( O, R, &sb );
                    N_DistPtPt( O, S, &tb );

                    if( tb LT NL_MTOL )
                        NL_ERROR( NL_INP_ERR );

                    sb = sb / tb;
                    tb = (lam * sb) / (1.0 + (lam - 1.0) * sb);

                    N_Combine2Pts( 1.0 - tb, O, tb, S, &Sh );

                    /* Get tangent plane at bent point */

                    N_VectorCross( SU, D, &M );
                    N_VectorCross( D, M, &SUh );

                    N_CreateLineStartDirVector( &tnP, S, SU, NL_UNBOUNDED );
                    N_CreateLineStartDirVector( &tnS, R, SUh, NL_UNBOUNDED );

                    error = N_IsectLineLine( tnP, tnS, &P, &sb, &tb, &its );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( its EQ NL_TRUE )
                    {
                        N_VectorDiff( Sh, P, &SUh );

                        error = N_VectorNormalizeRef( &SUh );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_VectorCross( SUh, D, &T );

                        if( NOT N_VectorsAreParallel( M, T ) )
                            N_VectorReverseInPlace( &SUh );
                    }
                    else
                    {
                        N_VectorCopy( SU, &SUh );
                    }

                    N_VectorCross( D, SV, &M );
                    N_VectorCross( M, D, &SVh );

                    N_CreateLineStartDirVector( &tnP, S, SV, NL_UNBOUNDED );
                    N_CreateLineStartDirVector( &tnS, R, SVh, NL_UNBOUNDED );

                    error = N_IsectLineLine( tnP, tnS, &P, &sb, &tb, &its );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( its EQ NL_TRUE )
                    {
                        N_VectorDiff( Sh, P, &SVh );

                        error = N_VectorNormalizeRef( &SVh );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_VectorCross( D, SVh, &T );

                        if( NOT N_VectorsAreParallel( M, T ) )
                            N_VectorReverseInPlace( &SVh );
                    }
                    else
                    {
                        N_VectorCopy( SV, &SVh );
                    }

                    N_VectorCross( SUh, SVh, &Nh );

                    /* Get rigid body motion and map control point */

                    N_VectorCross( N, SU, &T );
                    N_VectorCross( SV, N, &M );
                    N_VectorCombine( 0.5, M, 0.5, SU, &SU );
                    N_VectorCombine( 0.5, T, 0.5, SV, &SV );

                    N_VectorCross( Nh, SUh, &T );
                    N_VectorCross( SVh, Nh, &M );
                    N_VectorCombine( 0.5, M, 0.5, SUh, &SUh );
                    N_VectorCombine( 0.5, T, 0.5, SVh, &SVh );

                    error = N_CreateTransformMatrixFromAxes( S, SU, SV, N, Sh, SUh, SVh, Nh, &rma, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_TransformCPt( Qw[i][j], &rma, &Qw[i][j] );
                }
            }
        }
    }

    /* Remove unnecessary knots */

    if( flg EQ NL_CLEANUP )
    {
        error = N_SrfShapeRemoveKnots( surQ, tol, p + 1, n, q + 1, m, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_SrfShapeConstraints: Constraint-based surface modification       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies the shape of a surface by  satisfying 
     a set of derivative constraints, or  approximating  them  with  the 
     minimum  amount of  change in the  position of  control points. The 
     derivative constraints are given at selected parameter values. When 
     interactive shaping is required, only those quantities are computed 
     that  directly  affect the change in the surface's shape. A typical 
     calling example is:

       NL_FLAG       upd;
       NL_SURFACE    sur;
       NL_INDEX      **KU, **KV, *I, *J, *K, *nd, *iu, *jv, nu, nv, np; 
       NL_PARAMETER  *u, *v;
       NL_VECTOR     **DD;
       NL_RMATRIX    rma;
       NL_STACKS     SM;
       ...
       (define sur; get u, v, I, J, DD, KU, KV, iu, jv and nd);
       ...
       N_SrfShapeConstraints(&sur,u,v,nu,nv,iu,jv,np,I,J,DD,KU,KV,nd,NL_PREPARE,
                &rma,&K,&upd,&SM);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeConstraints");
...
         get new DD;
         N_SrfShapeConstraints(&sur,u,v,nu,nv,iu,jv,np,I,J,DD,KU,KV,nd,NL_INTERACT,
                  &rma,&K,&upd,&SM);
       }

     THE SHAPING IS DONE IN-PLACE, IE THE ORIGINAL NL_SURFACE IS DESTROYED.
     THE NL_SURFACE IS  UPDATED IF IT IS UNDER- OR FULLY-CONSTRAINED. IF IT 
     IS NL_OVER-CONSTRAINED, A NL_FLAG IS RETURNED. KNOT REFINEMENT  CAN  HELP 
     AVOIDING MORE THAN pxq CONSTRAINTS PER SPAN  NL_RECTANGLE, WHERE p AND 
     q ARE THE DEGREES OF THE NL_SURFACE.


   ACCESS:
   
     sur   , in/out ,  NURBS surface
     u,v   , input  ,  Parameters where constraints are assumed
     nu,nv , input  ,  Highest indexes in u and v
     iu,jv , input  ,  Arrays to define parameter pairs. Example: if the  
                       following pairs are selected
                         (u[0],v[0]),(u[1],v[1]),(u[1],v[2])
                       then
                         iu[3] = {0,1,1}
                         jv[3] = {0,1,2}
     np    , index  ,  Highest index in iu and jv
     I,J   , input  ,  Arrays: <I[k],J[l]> = 
                         <1,1>             : Pw[k][l] IS to change
                         <0,0>|<0,1>|<1,0> : Pw[k][l] is NOT to change
     DD    , input  ,  Derivative differences (constraints): DD[i][j] is
                       the j-th  constraint  assumed at  the i-th  (u,v) 
                       pair. DD does not have to be a 2-D array. It  can 
                       be an  array  of  pointers  pointing to arrays of 
                       different length. Example:
                         DD[i][0] = point (0-th derivative) change
                         DD[i][1] = u-derivative change
                         DD[i][2] = mixed derivative change
     KU,KV , input  ,  Types  of  constraints:  KU[i][k]  and   KV[i][l] 
                       specify that at the i-th (u,v) pair the KU[i][k]-
                       th derivative is  constrained in the u-direction,
                       and the KV[i][l]-th derivative is constrained in
                       the v-direction. Example (see DD above):
                         KU[i][0] = 0  KV[i][0] = 0
                         KU[i][1] = 1  KV[i][1] = 0 
                         KU[i][2] = 1  KV[i][2] = 1 
     nd    , input  ,  Highest indexes in the arrays pointed to by DD[i] 
                       or KU[i] or KV[i] is nd[i]. In the example above,
                       nd[i] = 2.
     flg   , input  ,  Flag:
                         NL_PREPARE : get some entities and shape
                         NL_INTERACT: change shape by changing DD
     rma   , in/out ,  Matrix defining control point changes
     K     , in/out ,  1-D indexes of control points that change
     upd   , output ,  Flag:
                          NL_YES: surface is updated, the surface is  under 
                               or fully constrained
                          NL_NO : surface  is  NOT  updated, the surface is 
                               NL_OVER constrained
     SG    , input  ,  rma's and K's stack
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeConstraints( NL_SURFACE *sur, NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX nu, NL_INDEX nv, NL_INDEX *iu, NL_INDEX *jv, NL_INDEX np, NL_INDEX *I, NL_INDEX *J, NL_VECTOR ** DD, NL_INDEX ** KU, NL_INDEX ** KV, NL_INDEX *nd, NL_FLAG flg, NL_RMATRIX *rma, NL_INDEX ** K, NL_FLAG *upd, NL_STACKS *SG )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeConstraints");

    NL_FLAG error = NL_NO;

    NL_INDEX ** M, *KL, *O, *usp, *vsp, *udh, *vdh, spu, spv, udr, vdr, i, j, k, l, n, m, r, s, nb, mb, row, col, ii, jj;

    NL_DEGREE p, q;

    NL_REAL **** BD, ** B, *U, *V, w;

    NL_POINT *DP, P;

    NL_CPOINT ** Pw;

    NL_VECTOR *DV;

    NL_KNOTVECTOR *knu, *knv;

    NL_RMATRIX rmb, rmt, rbt, rmi;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    if( u[0]LT U[0]OR u[nu]GT U[r] )
        NL_ERROR( NL_PAR_ERR );

    if( v[0]LT V[0]OR v[nv]GT V[s] )
        NL_ERROR( NL_PAR_ERR );

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
        N_SrfGetKnotVectors( sur, &knu, &knv );

        /* Get span arrays and adjust I and J arrays */

        O = N_AllocInt1dArray( NL_MAX( n, m ), &SL );

        if( O EQ NULL )
            NL_QUIT;

        usp = N_AllocInt1dArray( nu, &SL );

        if( usp EQ NULL )
            NL_QUIT;

        vsp = N_AllocInt1dArray( nv, &SL );

        if( vsp EQ NULL )
            NL_QUIT;

        k = 0;

        for ( i = 0; i <= n; i++ )
            O[i] = 0;

        for ( i = 0; i <= nu; i++ )
        {
            error = N_BasisFindSpan( knu, p, u[i], NL_LEFT, &usp[i] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = usp[i] - p; j <= usp[i]; j++ )
                O[j] = 1;
        }

        for ( i = 0; i <= n; i++ )
        {
            I[i] = I[i] * O[i];

            if( I[i]EQ 1 )
                k++;
        }

        l = 0;

        for ( j = 0; j <= m; j++ )
            O[j] = 0;

        for ( j = 0; j <= nv; j++ )
        {
            error = N_BasisFindSpan( knv, q, v[j], NL_LEFT, &vsp[j] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = vsp[j] - q; i <= vsp[j]; i++ )
                O[i] = 1;
        }

        for ( j = 0; j <= m; j++ )
        {
            J[j] = J[j] * O[j];

            if( J[j]EQ 1 )
                l++;
        }

        /* Mark control points that change and get 1-D indexing */

        M = N_AllocInt2dArray( n, m, &SL );

        if( M EQ NULL )
            NL_QUIT;

        KL = N_AllocInt1dArray( k * l, SG );

        if( KL EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                M[i][j] = -1;
        }

        mb = -1;

        for ( k = 0; k <= np; k++ )
        {
            for ( i = 0; i <= p; i++ )
            {
                ii = usp[iu[k]] - p + i;

                for ( j = 0; j <= q; j++ )
                {
                    jj = vsp[jv[k]] - q + j;

                    if( I[ii]*J[jj]GT 0 )
                    {
                        if( M[ii][jj]LT 0 )
                        {
                            mb++;
                            M[ii][jj] = mb;
                            KL[mb] = ii * (m + 1) + jj;
                        }
                    }
                }
            }
        }

        /* Get highest derivatives, and number of constraints */

        udh = N_AllocInt1dArray( np, &SL );

        if( udh EQ NULL )
            NL_QUIT;

        vdh = N_AllocInt1dArray( np, &SL );

        if( vdh EQ NULL )
            NL_QUIT;

        nb = -1;
        k = 0;
        l = 0;

        for ( i = 0; i <= np; i++ )
        {
            udh[i] = 0;
            vdh[i] = 0;

            for ( j = 0; j <= nd[i]; j++ )
            {
                if( KU[i][j]GT udh[i] )
                    udh[i] = KU[i][j];

                if( KV[i][j]GT vdh[i] )
                    vdh[i] = KV[i][j];
            }

            if( udh[i]GT k )
                k = udh[i];

            if( vdh[i]GT l )
                l = vdh[i];

            nb += nd[i] + 1;
        }

        /* If over-constrained -> out */

        if( nb GT mb )
        {
            *upd = NL_NO;
            NL_OUT;
        }

        /* Get matrix containing required derivatives */

        BD = N_AllocReal4dArray( k, l, p, q, &SL );

        if( BD EQ NULL )
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

        for ( k = 0; k <= np; k++ )
        {
            error = N_SrfBasisDerivs( sur, u[iu[k]], v[jv[k]], NL_LEFT, NL_LEFT, NL_FALSE, udh[k], vdh[k], BD, &spu, &spv );

            if( error EQ NL_YES )
                NL_OUT;

            for ( l = 0; l <= nd[k]; l++ )
            {
                row++;
                udr = KU[k][l];
                vdr = KV[k][l];

                for ( i = 0; i <= p; i++ )
                {
                    ii = spu - p + i;

                    for ( j = 0; j <= q; j++ )
                    {
                        jj = spv - q + j;
                        col = M[ii][jj];

                        if( col GE 0 )
                            B[row][col] = BD[udr][vdr][i][j];
                    }
                }
            }
        }

        /* Compute matrix */

        if( nb EQ mb )
        {
            /* Fully-determined system */

            N_InitRealMatrix( rma );
            error = N_RealMatrixInversePivot( &rmb, rma, SG );

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
            error = N_RealMatrixMultiply( &rmt, &rmi, rma, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* Output index array for interactive step */

        *K = KL;
    }

    /* Interactively shape */

    if( flg EQ NL_PREPARE OR flg EQ NL_INTERACT )
    {
        KL = *K;

        /* Get right hand side of constraints */

        N_GetMaxIndexRealMatrix( rma, &mb, &nb );

        DV = N_AllocPt1dArray( nb, &SL );

        if( DV EQ NULL )
            NL_QUIT;

        l = 0;

        for ( k = 0; k <= np; k++ )
        {
            for ( i = 0; i <= nd[k]; i++ )
            {
                N_CopyPt( DD[k][i], &DV[l] );
                l++;
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

        for ( l = 0; l <= mb; l++ )
        {
            i = KL[l] / (m + 1);
            j = KL[l] % (m + 1);

            N_CPtToPtAndW( Pw[i][j], &P, &w );
            N_Sum2Pts( P, DP[l], &P );
            N_Weight( P, w, &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_SrfShapeFlatten: Flatten a NURBS surface                         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine  flattens a  surface against  a plane.  Either  
     all control points local to a polygonal region are mapped, or  only  
     those  that are  on the same  side of the  plane. A typical calling 
     example is:

       NL_SURFACE  surP, surQ;
       EPOLGON  ppl;
       NL_INDEX    npu, npv, ksu, keu, ksv, kev;
       NL_PLANE    pln;
       NL_VECTOR   W;
       NL_REAL     tol;
       NL_STACKS   SP, SQ;
       ...
       (define surP; get ppl, npu,...,kev; get plane and tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfShapeFlatten(&surP,&ppl,npu,npv,ksu,keu,ksv,kev,&pln,W,tol,NL_YES,NL_LEFT,
                NL_YES,&surQ,&SP,&SQ);

     THE  NL_SURFACE  CAN  BE  REFINED  BEFORE  SHAPING. IN  THIS  CASE THE 
     REFINEMENT IS  SKIPPED, HOWEVER, THE  ROUTINE  CHECKS  IF  AT LEAST 
     (p+1)x(q+1) CONTROL NL_POINTS ARE LOCAL TO THE BOUNDING BOX OF ppl. IF
     REFINEMENT IS REQUIRED, IT IS RESTRICTED TO THE BOUNDARIES OF ppl.


   ACCESS:
   
     surP    , in/out ,  NURBS surface
     ppl     , input  ,  Closed polygon  residing in the papameter space 
                         of surP. It must at least partially overlap the 
                         knot rectangle.
     npu,npv , input  ,  Number of u/v knots to be inserted. The default 
                         is 3*p/3*q.
     ksu,keu , input  ,  Continuity   controls   in   u-direction;   the 
                         flattened   surface   is  at   most   C^{p-ksu} 
                           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeFlatten");
continuous along a u-line bounding ppl from the 
                         left, and C^{p-keu} continuous  along  an  iso-
                         line bounding  the  region from the right (1 <= 
                         ksu,keu <= p!). To obtain a u-crease, set ksu = 
                         keu = p.
     ksv,kev , input  ,  The same as above applied to the v-direction
     pln     , input  ,  Plane
     W       , input  ,  Flatten direction
     tol     , input  ,  Knot removal  tolerance. For  shaping accuracy, 
                         1% of the surface's size is a good default
     stf     , input  ,  Side test flag:
                           NL_YES: map points on given side of plane
                           NL_NO : map points local to ppl
     lor     , input  ,  Side indicator:
                           NL_LEFT : map points on the left of plane
                           NL_RIGHT: map points on the right of plane
     ref     , input  ,  Flag:
                           NL_YES: refine surface
                           NL_NO : surface is already refined
     surQ    , output ,  Flattened surface
     SP      , input  ,  surP's stack
     SQ      , input  ,  surQ's stack    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeFlatten( NL_SURFACE *surP, NL_EPOLYGON *ppl, NL_INDEX npu, NL_INDEX npv, NL_INDEX ksu, NL_INDEX keu, NL_INDEX ksv, NL_INDEX kev, NL_PLANE *pln, NL_VECTOR W, NL_REAL tol, NL_FLAG stf, NL_FLAG lor, NL_FLAG ref, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeFlatten");

    NL_FLAG side, its, error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, su, eu, sv, ev, nku, nkv, nkx, nky, fku, lku, fkv, lkv, il, jl, ih, jh;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *tu, *tv, w, t, us, ue, vs, ve, du, dv, x, y;

    NL_POINT ** A, Q;

    NL_CPOINT ** Qw;

    NL_KNOTVECTOR *knu, *knv, *knx, *kny;

    NL_MINMAXBOX box;

    NL_BOOLEAN ** IN = NULL;

    NL_LINESEG lin;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Qw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    if( stf NEQ NL_YES AND stf NEQ NL_NO )
        NL_ERROR( NL_CAL_ERR );

    if( ref NEQ NL_YES AND ref NEQ NL_NO )
        NL_ERROR( NL_CAL_ERR );

    if( lor NEQ NL_LEFT AND lor NEQ NL_RIGHT )
        NL_ERROR( NL_CAL_ERR );

    /* Refine surface */

    N_PolygonGetBBox( ppl, &box );
    N_GetBBoxData( &box, &us, &ue, &vs, &ve, &t, &t );

    if( ue LE U[0]OR us GE U[r] )
        NL_ERROR( NL_INP_ERR );

    if( ve LE V[0]OR vs GE V[s] )
        NL_ERROR( NL_INP_ERR );

    if( us LT U[0] )
        us = U[0];

    if( ue GT U[r] )
        ue = U[r];

    if( vs LT V[0] )
        vs = V[0];

    if( ve GT V[s] )
        ve = V[s];

    if( ref EQ NL_YES )
    {
        nku = NL_MAX( npu, 3 * p ) + 1;
        nkx = nku + 2 * p + 1;

        nkv = NL_MAX( npv, 3 * q ) + 1;
        nky = nkv + 2 * q + 1;

        knx = N_AllocKnotVectorAndArray( nkx, &SL );

        if( knx EQ NULL )
            NL_QUIT;

        kny = N_AllocKnotVectorAndArray( nky, &SL );

        if( kny EQ NULL )
            NL_QUIT;

        error = N_BasisSplitNLongestSpans( knu, p, us, ue, nku, ksu, keu, knx );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisSplitNLongestSpans( knv, q, vs, ve, nkv, ksv, kev, kny );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfInsertKnots( surP, knx, NL_UDIR, surQ, SP, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfInsertKnots( surQ, kny, NL_VDIR, surQ, SQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_SrfCopy( surP, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SrfGetCPts( surQ, &n, &m, &Qw );
    N_SrfGetKnotVectors( surQ, &knu, &knv );

    /* Get local parametrization */

    if( stf EQ NL_NO )
    {
        error = N_BasisFindSpan( knu, p, us, NL_LEFT, &su );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knu, p, ue, NL_LEFT, &eu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, vs, NL_LEFT, &sv );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, ve, NL_LEFT, &ev );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
        {
            su = 0;
            il = 0;
        }
        else
            il = su - 1;

        if( ue EQ U[r] )
        {
            eu = n + p + 1;
            ih = n;
        }
        else
            ih = eu - p;

        if( vs EQ V[0] )
        {
            sv = 0;
            jl = 0;
        }
        else
            jl = sv - 1;

        if( ve EQ V[s] )
        {
            ev = m + q + 1;
            jh = m;
        }
        else
            jh = ev - q;

        if( ref EQ NL_NO AND eu - p - su - 1 LT p )
            NL_ERROR( NL_INP_ERR );

        if( ref EQ NL_NO AND ev - q - sv - 1 LT q )
            NL_ERROR( NL_INP_ERR );

        k = ih - il;
        l = jh - jl;

        tu = N_AllocReal1dArray( k, &SL );

        if( tu EQ NULL )
            NL_QUIT;

        tv = N_AllocReal1dArray( l, &SL );

        if( tv EQ NULL )
            NL_QUIT;

        A = N_AllocPt2dArray( k, l, &SL );

        if( A EQ NULL )
            NL_QUIT;

        for ( i = il; i <= ih; i++ )
        {
            for ( j = jl; j <= jh; j++ )
            {
                N_CPtToPtEuclid( Qw[i][j], &A[i - il][j - jl] );
            }
        }

        error = N_FitCalcSrfParamValues( (NL_VOID ** )A, k, l, NL_EPOINT, NL_CHORDLENGTH, tu, tv );

        if( error EQ NL_YES )
            NL_OUT;

        IN = N_AllocInt2dArray( k, l, &SL );

        if( IN EQ NULL )
            NL_QUIT;

        du = ue - us;
        dv = ve - vs;

        for ( i = 0; i <= k; i++ )
        {
            x = us + tu[i] * du;

            for ( j = 0; j <= l; j++ )
            {
                y = vs + tv[j] * dv;

                N_PtFromXYZ( x, y, 0.0, &A[i][j] );
            }
        }

        if( us EQ U[0] )
            for ( j = 0; j <= l; j++ )
                N_VectorBlendPt( NL_PTOL, NL_UNITX, &A[0][j] );

        if( ue EQ U[r] )
            for ( j = 0; j <= l; j++ )
                N_VectorBlendPt( -NL_PTOL, NL_UNITX, &A[k][j] );

        if( vs EQ V[0] )
            for ( i = 0; i <= k; i++ )
                N_VectorBlendPt( NL_PTOL, NL_UNITY, &A[i][0] );

        if( ve EQ V[s] )
            for ( i = 0; i <= k; i++ )
                N_VectorBlendPt( -NL_PTOL, NL_UNITY, &A[i][l] );

        error = N_PtsAreInPolygon( ppl, A, k, l, IN );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        su = 0;
        eu = n + p + 1;
        sv = 0;
        ev = m + q + 1;
    }

    /* Flatten surface */

    for ( i = su; i < eu - p; i++ )
    {
        for ( j = sv; j < ev - q; j++ )
        {
            N_CPtToPtAndW( Qw[i][j], &Q, &w );

            if( stf EQ NL_YES )
            {
                side = N_PtGetSideOfPlane( pln, Q );

                if( side NEQ lor )
                    continue;
            }
            else
            {
                if( IN[i - su][j - sv]EQ NL_FALSE )
                    continue;
            }

            N_CreateLineStartDirVector( &lin, Q, W, NL_UNBOUNDED );

            error = N_IsectLinePlane( lin, *pln, &Q, &t, &its );

            if( error EQ NL_YES )
                NL_OUT;

            if( its EQ NL_TRUE )
                N_Weight( Q, w, &Qw[i][j] );
        }
    }

    /* Remove unnecessary knots */

    if( su GE p )
        fku = su + (p + 1) / 2;
    else
        fku = p + 1;

    if( eu LE n )
        lku = eu - (p + 2) / 2;
    else
        lku = n;

    if( sv GE q )
        fkv = sv + (q + 1) / 2;
    else
        fkv = q + 1;

    if( ev LE m )
        lkv = ev - (q + 2) / 2;
    else
        lkv = m;

    error = N_SrfShapeRemoveKnots( surQ, tol, fku, lku, fkv, lkv, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_SRFSHAPEINTP: Shape surface to interpolate given points                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies the  shape of a given base surface to
     pass  through a  set of  points. It  performs  constrained  shaping 
     based on the parametrization of points obtained by  projection onto 
     the  surface. The  shaping is done in one step, i.e., it is assumed
     that the points are not too far from the surface. A typical calling 
     example is:

       NL_SURFACE surB, surS;
       NL_POINT   *P;
       NL_INDEX   np, dsu, deu, dsv, dev;
       NL_STACKS  SG;
       ...
       (get base surface surB and points P);
       ...
       N_SrfInitArrays(&surS);
       N_SrfShapeInterp(&surB,P,np,dsu,deu,dsv,dev,NL_YES,&surS,&SG);

     MEMORY FOR THE NL_SURFACE STRUCTURE OF  surS  MUST BE ALLOCATED IN THE
     CALLING ROUTINE (SHOWN IN THE EXAMPLE  ABOVE). THIS ROUTINE  PURGES
     THE DATA SET; NL_POINTS  THAT ARE CLOSE TO  THE BOUNDARIES ARE REMOVED 
     AS THEY CAUSE NON-SENSICAL  SURFACES WHEN INTERPOLATED. THE CURRENT
     SETTING  ALLOWS 90% OF THE  INTERIOR OF  EACH NL_PARAMETER  SPAN TO BE 
     NL_USED.


   ACCESS:
   
     surB , input  ,  Base surface
     P    , input  ,  Random points output surface must interpolate
     np   , input  ,  Highest index in P
     dsu  , input  ,  Start  derivative  constraint  in u-dir.; 0,..,dsu 
                      derivatives not to change at the u-start
     deu  , input  ,  End  derivative  constraint  in  u-dir.;  0,..,deu 
                      derivatives not to change at the u-end
     dsv  , input  ,  Start  derivative  constraint in  v-dir.; 0,..,dsv 
                      derivatives not to change at the v-start
     dev  , input  ,  End  derivative  constraint  in  v-dir.;  0,..,dev
                      derivatives not to change at the v-end
     lfl  , input  ,  Flag:
                        NL_YES: localize surface before shaping
                        NL_NO : do not localize, let routine refine surface
                             globally
                      LOCALIZATION PROVIDES (1) SOLVABLE SYSTEM OF EQUA-
                      TIIONS, AND (2) LOCAL  EFFECTS (WHICH MAY BE UNDE-
                      SIRABLE). GLOBALIZATION   PROVIDES   MORE   GLOBAL 
                      CHANGES, HOWEVER, THE  SYSTEM OF EQUATIONS MAY NOT
                      BE SOLVABLE! RECOMMENDED DEFAULT: lfl = NL_YES!!
     surS , output ,  Shaped surface interpolating P[i], i=0,...,np
     SG   , input  ,  surS' stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeInterp( NL_SURFACE *surB, NL_POINT *P, NL_INDEX np, NL_INDEX dsu, NL_INDEX deu, NL_INDEX dsv, NL_INDEX dev, NL_FLAG lfl, NL_SURFACE *surS, NL_STACKS *SG )
{

    NL_FLAG upd, error = NL_NO;

    NL_INDEX ** KU, ** KV, *I, *J, *K, *nd, *iu, *jv, i, j, k, m, n, r, s, nI, mJ, nt, kp, rc, sc, ri, si;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *ut, *vt, *up, *vp, *uc, *vc, *ui = NULL, *vi = NULL, cdu, cdv, d, dm, uu, vv, ltu, ltv, UL, UR, VB, VT;

    NL_POINT *T, *Q, R;

    NL_VECTOR ** DD;

    NL_KNOTVECTOR knu, knv;

    NL_RMATRIX rma;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL dtl = 0.001;
    NL_PRIVATE NL_REAL cdt = 0.05;
    NL_PRIVATE NL_REAL ktl = 0.05;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Prepare for shaping and get local notation */

    N_SrfInitArrays( surS );
    error = N_SrfCopy( surB, surS, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( np LT 0 )
        NL_OUT;

    N_SrfGetKnots( surS, &r, &s, &U, &V );
    N_SrfGetArraySizes( surS, &n, &m, &r, &s );
    N_SrfGetDegrees( surS, &p, &q );

    nI = n + np + 2;
    mJ = m + np + 2;

    cdu = cdt * (U[r] - U[0]);
    cdv = cdt * (V[s] - V[0]);
    ltu = ktl * (U[r] - U[0]);
    ltv = ktl * (V[s] - V[0]);

    /* Allocate memory */

    I = N_AllocInt1dArray( nI, &SL );

    if( I EQ NULL )
        NL_QUIT;

    J = N_AllocInt1dArray( mJ, &SL );

    if( J EQ NULL )
        NL_QUIT;

    KU = N_AllocInt2dArray( np, 0, &SL );

    if( KU EQ NULL )
        NL_QUIT;

    KV = N_AllocInt2dArray( np, 0, &SL );

    if( KV EQ NULL )
        NL_QUIT;

    nd = N_AllocInt1dArray( np, &SL );

    if( nd EQ NULL )
        NL_QUIT;

    iu = N_AllocInt1dArray( np, &SL );

    if( iu EQ NULL )
        NL_QUIT;

    jv = N_AllocInt1dArray( np, &SL );

    if( jv EQ NULL )
        NL_QUIT;

    DD = N_AllocPt2dArray( np, 0, &SL );

    if( DD EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( np, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    ut = N_AllocReal1dArray( np, &SL );

    if( ut EQ NULL )
        NL_QUIT;

    vt = N_AllocReal1dArray( np, &SL );

    if( vt EQ NULL )
        NL_QUIT;

    up = N_AllocReal1dArray( np, &SL );

    if( up EQ NULL )
        NL_QUIT;

    vp = N_AllocReal1dArray( np, &SL );

    if( vp EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= np; i++ )
        KU[i][0] = KV[i][0] = nd[i] = 0;

    /* Get the parameters and purge data set */

    error = N_SrfProjectPts( surS, P, np, NL_NO, NL_NO, dtl, &T, &ut, &vt, &nt, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    kp = -1;
    UL = U[0] + ltu;
    UR = U[r] - ltu;
    VB = V[0] + ltv;
    VT = V[s] - ltv;

    for ( i = 0; i <= nt; i++ )
    {
        if( ut[i]GT UL AND ut[i]LT UR AND vt[i]GT VB AND vt[i]LT VT )
        {
            kp++;
            up[kp] = ut[i];
            vp[kp] = vt[i];
            N_VectorCopy( T[i], &Q[kp] );
        }
    }

    /* Eliminate clusters */

    error = N_FindClustersRealArray( up, kp, cdu, NL_PARAMETERS, &uc, &rc, NULL, NULL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_FindClustersRealArray( vp, kp, cdv, NL_PARAMETERS, &vc, &sc, NULL, NULL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* While shaping is not successful do */

    while( 1 )
    {
        /* See if additional degrees of freedom are needed */

        ri = si = -1;

        if( lfl EQ NL_YES )
        {
            error = N_KnotsRefine( U, r, p, uc, rc, dsu, deu, &ui, &ri, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_KnotsRefine( V, s, q, vc, sc, dsv, dev, &vi, &si, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( ri GE 0 )
        {
            N_KnotVectorFromRealArray( &knu, ui, ri );

            error = N_SrfInsertKnots( surS, &knu, NL_UDIR, surS, SG, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( si GE 0 )
        {
            N_KnotVectorFromRealArray( &knv, vi, si );

            error = N_SrfInsertKnots( surS, &knv, NL_VDIR, surS, SG, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        N_SrfGetKnots( surS, &r, &s, &U, &V );
        N_SrfGetArraySizes( surS, &n, &m, &r, &s );

        if( n GT nI )
        {
            k = NL_MAX( n + nI, nI + nI );

            error = N_Realloc1dIntArray( &I, nI, k, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            nI = k;
        }

        if( m GT mJ )
        {
            k = NL_MAX( m + mJ, mJ + mJ );

            error = N_Realloc1dIntArray( &J, mJ, k, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            mJ = k;
        }

        /* Set up for shaping */

        for ( i = 0; i <= kp; i++ )
        {
            N_SrfEvalPt( surS, up[i], vp[i], NL_LEFT, NL_LEFT, &R );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDiff( Q[i], R, &DD[i][0] );

            iu[i] = jv[i] = i;
        }

        for ( k = 0; k <= dsu; k++ )
            I[k] = 0;

        for ( k = 0; k <= deu; k++ )
            I[n - k] = 0;

        for ( k = dsu + 1; k <= n - deu - 1; k++ )
            I[k] = 1;

        for ( k = 0; k <= dsv; k++ )
            J[k] = 0;

        for ( k = 0; k <= dev; k++ )
            J[m - k] = 0;

        for ( k = dsv + 1; k <= m - dev - 1; k++ )
            J[k] = 1;

        /* Shape curve now */

        error = N_SrfShapeConstraints( surS, up, vp, kp, kp, iu, jv, kp, I, J, DD, KU, KV, nd, NL_PREPARE, &rma, &K, &upd, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( upd EQ NL_NO )
        {
            N_SrfGetKnots( surS, &r, &s, &U, &V );

            dm = 0.0;

            for ( i = p; i < r - p; i++ )
            {
                if( U[i]NEQ U[i + 1] )
                {
                    d = U[i + 1] - U[i];

                    if( d GT dm )
                    {
                        dm = d;
                        k = i;
                    }
                }
            }
            uu = 0.5 *( U[k] + U[k + 1] );

            dm = 0.0;

            for ( j = q; j < s - q; j++ )
            {
                if( V[j]NEQ V[j + 1] )
                {
                    d = V[j + 1] - V[j];

                    if( d GT dm )
                    {
                        dm = d;
                        k = j;
                    }
                }
            }
            vv = 0.5 *( V[k] + V[k + 1] );

            error = N_SrfInsertKnot( surS, uu, 1, NL_UDIR, surS, SG, SG );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnot( surS, vv, 1, NL_VDIR, surS, SG, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetKnots( surS, &r, &s, &U, &V );
            N_SrfGetArraySizes( surS, &n, &m, &r, &s );
        }
        else
        {
            break;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_SrfShapeModifyWeight: Modify one surface weight                                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine modifies  one weight to push/pull the surface
     toward a given control point with a specified distance. The weight
     is allowed  to vary in  the range of  [NL_WMIN,NL_WMAX] as  specified in
     "globals.h". The shaping is done in place, ie the original surface 
     is destroyed. To facilitate with interactive shape design, several 
     quantities,  needed  to  compute  the  new  weight, are  output. A 
     typical calling example is:

       NL_SURFACE    sur;
       NL_INDEX      k, l;
       NL_PARAMETER  u, v;
       NL_REAL       d, wkl, pklp, rkluv;
       ...
       (define sur; get u, v and d);
       ...
       N_SrfShapeModifyWeight(&sur,k,l,u,v,d,&wkl,&pklp,&rkluv,NL_PREPARE);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeModifyWeight");
...
         d = d+dd;
         N_SrfShapeModifyWeight(&sur,k,l,u,v,d,&wkl,&pklp,&rkluv,NL_INTERACT);
       }

     To prepare  for interactive  shape modification, several utilities 
     are  provided:  N_BasisFindIndexNode,  N_BasisComputeIndexNodeArray,  N_BasisFindNodeSpan and N_BasisFindKnotToTurnParamIntoNode. These 
     utilities  allow the  designer to  compute nodes, find node spans, 
     refine the surface with the appropriate knot, and  select the most
     suitable control point. To  insert a knot, see the  many utilities
     N_TOO*** in the tools directory. 


   ACCESS:
   
     sur   , in/out ,  NURBS surface
     k,l   , input  ,  Indexes of weight to be changed
     u,v   , input  ,  Parameters where  surface point is to be pulled or 
                       pushed
     d     , input  ,  Distance of pull/push
     wkl   , in/out ,  Original weight of Pw[k][l]
     pklp  , in/out ,  Distance between S(u,v) and Pw[k][l]
     rkluv , in/out ,  Value of rational basis function
     flg   , input  ,  Flag:
                         NL_PREPARE : get pklp, rkluv and update wkl
                         NL_INTERACT: update wkl


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeModifyWeight( NL_SURFACE *sur, NL_INDEX k, NL_INDEX l, NL_PARAMETER u, NL_PARAMETER v, NL_REAL d, NL_REAL *wkl, NL_REAL *pklp, NL_REAL *rkluv, NL_FLAG flg )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeModifyWeight");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    NL_REAL w, wh, den;

    NL_POINT Pkl, P;

    NL_CPOINT ** Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notations and check input */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    if( k LT 0 OR k GT n )
        NL_ERROR( NL_IND_ERR );

    if( l LT 0 OR l GT m )
        NL_ERROR( NL_IND_ERR );

    if( NOT N_IsSrfRat( sur ) )
        NL_ERROR( NL_INP_ERR );

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

    N_CPtToPtAndW( Pw[k][l], &Pkl, &w );

    if( flg EQ NL_PREPARE )
    {
        error = N_SrfBasisIEval( sur, k, l, u, v, NL_LEFT, NL_LEFT, rkluv );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfEvalPt( sur, u, v, NL_LEFT, NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( Pkl, P, pklp );

        *wkl = w;
    }

    /* Compute new weight and reweight control point */

    if( flg EQ NL_PREPARE OR flg EQ NL_INTERACT )
    {
        w = *wkl;

        den = (*rkluv) * (*pklp - d);

        if( N_FloatOpIsBad( d, den, NL_DIVISION ) )
            NL_ERROR( NL_INP_ERR );

        wh = w * (1.0 + d / den);

        if( wh LT NL_WMIN OR wh GT NL_WMAX )
            NL_ERROR( NL_WEI_ERR );

        N_Weight( Pkl, wh, &Pw[k][l] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}

/**********************************************************************/
/* N_SrfShapePolylineWarp: Surface polyline warp                                    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping  routine warps a NURBS  surface along a polygonal path
     residing in  the parameter  space. The  warp is done by  sweeping a 
     warp function  along the polygonal trajectory. The warp function is 
     either  a  given  function  or  a  rational  basis  function.  When  
     interactive   change  is   required,  only  those   quantities  are 
     recomputed  that directly  affect the change in the warp. A typical 
     calling example is:

       NL_SURFACE    surP, surQ;
       NL_CURVE      curW;
       EPOLGON    ppl;
       NL_CFUN       cfnD;
       NL_DEGREE     p;
       NL_INDEX      npu, npv, siu, eiu, siv, eiv;
       NL_VECTOR     W;
       NL_REAL       **D, d, tol, wp, off;
       NL_STACKS     SP, SQ;
       ...
       (define surP; get ppl, npu and npv; get either curW or W; get
        rational basis function and d, or distance function; get tol
        and off);
       ...
       N_SrfShapeCreateRatBasis(wp,p,&cfnD,NL_NEW,&SQ);
       ...
       N_SrfShapePolylineWarp(&surP,&ppl,npu,npv,NULL,W,&cfnD,d,tol,off,NL_PREPARE,NL_YES,
                &D,&siu,&eiu,&siv,&eiv,&surQ,&SP,&SQ);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapePolylineWarp");
...
         get new wp;
         N_SrfShapeCreateRatBasis(wp,p,&cfnD,NL_OLD,&SQ);
         ...
         N_SrfShapePolylineWarp(&surP,&ppl,npu,npv,NULL,W,&cfnD,d,tol,off,NL_INTERACT,NL_NO,
                  &D,&siu,&eiu,&siv,&eiv,&surQ,&SP,&SQ);
         ...
         get new d;
         N_SrfShapePolylineWarp(&surP,&ppl,npu,npv,NULL,W,&cfnD,d,tol,off,NL_INTERACT,NL_NO,
                  &D,&siu,&eiu,&siv,&eiv,&surQ,&SP,&SQ);
         ...
       }
       N_SrfShapePolylineWarp(&surP,&ppl,npu,npv,NULL,W,&cfnD,d,tol,off,NL_CLEANUP,NL_NO,
                &D,&siu,&eiu,&siv,&eiv,&surQ,&SP,&SQ);

     THE WARPED  NL_SURFACE IS STORED IN  surQ. ALTHOUGH  THE INPUT NL_SURFACE 
     DOES  NOT  CHANGE  EITHER  GEOMETRICALLY  OR  PARAMETRICALLY,   ITS 
     DEFINITION IS DESTROYED IN THAT ITS KNOT VECTORS ARE REFINED. IF IT  
     IS   UNACCEPTABLE,  EITHER   SAVE  THE  ORIGINAL NL_SURFACE OR  REMOVE 
     UNNECESSARY KNOTS. IF surP = surQ, NL_NO  INTERACTION IS ALLOWED, I.E.
     ONLY THE "NL_PREPARE" AND "NL_CLEANUP" FLAGS ARE NL_USED. THIS CAPABILITY IS
     USEFUL TO PERFORM A SERIES OF WARPS ON THE  INPUT NL_SURFACE surP. THE 
     NL_SURFACE CAN BE REFINED BEFORE  SHAPING. IN THIS CASE THE REFINEMENT 
     IS SKIPPED IN THE  "NL_PREPARE"  STAGE, HOWEVER, THE ROUTINE CHECKS IF 
     AT LEAST  (p+1)x(q+1)  CONTROL NL_POINTS ARE LOCAL TO THE BOUNDING BOX
     OF ppl.


   ACCESS:
   
     surP    , in/out ,  NURBS surface
     ppl     , input  ,  Polygonal path  residing in the papameter space 
                         of surP. 
     npu,npv , input  ,  Number of u/v knots to be inserted. Depends  on 
                         the warp distance; for large warps more control 
                         points are needed to obtain a smooth  change in 
                         shape. The default is 3*p/3*q  for small warps.
     curW    , input  ,  Direction curve:
                            = NULL: use input vector W
                           != NULL: use curW
     W       , input  ,  Warp direction
     cfnD    , input  ,  Warp function or rational basis function  (must 
                         be parametrized between [0,1]!)
     d       , input  ,  Warp distance:
                           > 0.0: cfnD is rational basis function
                           < 0.0: cfnD is distance function
     tol     , inpout ,  Knot removal  tolerance. For  shaping accuracy, 
                         1% of the surface's size is a good default
     off     , input  ,  Offset distance to define gravity field of ppl.
                         The larger the offset, the  thicker the polygon 
                         is and hence the more control points are moved.
     flg     , input  ,  Flag:
                           NL_PREPARE : get some entities and do first warp
                           NL_INTERACT: change warp by changing d, W, etc
                           NL_CLEANUP : remove unneccesary knots
     ref     , input  ,  Flag:
                           NL_YES: refine surface
                           NL_NO : surface is already refined
     D       , in/out ,  Distance array; D[i][j] is
                           >= 0.0: Pw[i][j] is  in the  gravity field of
                                   ppl and hence to be repositioned
                           <  0.0: Pw[i][j] is not to be moved
     siu,eiu , in/out ,  U-indexes of local control points
     siv,eiv , in/out ,  V-indexes of local control points
     surQ    , output ,  Warped surface; if surP=surQ, only the  NL_PREPARE
                         and NL_CLEANUP options are allowed
     SP      , input  ,  surP's stack
     SQ      , input  ,  Stack holding pointers surQ and D
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapePolylineWarp( NL_SURFACE *surP, NL_EPOLYGON *ppl, NL_INDEX npu, NL_INDEX npv, NL_CURVE *curW, NL_VECTOR W, NL_CFUN *cfnD, NL_REAL d, NL_REAL tol, NL_REAL off, NL_FLAG flg, NL_FLAG ref, NL_REAL *** D, NL_INDEX *siu, NL_INDEX *eiu, NL_INDEX *siv, NL_INDEX *eiv, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapePolylineWarp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, su = 0, eu = 0, sv = 0, ev = 0, nku, nkv, nkx, nky, fku, lku, fkv, lkv;

    NL_DEGREE p, q, pb = 0;

    NL_REAL ** Dl = NULL, *U, *V, *tlu, *tlv, rmax = 0.0, w, f, us, ue, vs, ve, t, du, dv, x, y, o_off;

    NL_POINT ** A, Q;

    NL_CPOINT ** Pw, ** Qw = NULL;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL, *knx, *kny;

    NL_MINMAXBOX box;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    if( off LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );
    o_off = 1.0 / off;

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    switch( flg )
    {
        case NL_PREPARE:

            N_SrfGetKnotVectors( surP, &knu, &knv );
            break;

        case NL_INTERACT:
            if( surP EQ surQ )
                NL_ERROR( NL_INP_ERR );

            su = *siu;
            eu = *eiu;
            Dl = *D;
            sv = *siv;
            ev = *eiv;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
            break;

        case NL_CLEANUP:

            su = *siu;
            eu = *eiu;
            sv = *siv;
            ev = *eiv;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Prepare for warp */

    if( flg EQ NL_PREPARE )
    {
        /* Set knot interval */

        N_PolygonGetBBox( ppl, &box );
        N_GetBBoxData( &box, &us, &ue, &vs, &ve, &f, &f );

        if( ue LE U[0]OR us GE U[r] )
            NL_ERROR( NL_INP_ERR );

        if( ve LE V[0]OR vs GE V[s] )
            NL_ERROR( NL_INP_ERR );

        us -= off;
        ue += off;
        vs -= off;
        ve += off;

        if( us LT U[0] )
            us = U[0];

        if( ue GT U[r] )
            ue = U[r];

        if( vs LT V[0] )
            vs = V[0];

        if( ve GT V[s] )
            ve = V[s];

        /* Refine surface */

        if( ref EQ NL_YES )
        {
            nku = NL_MAX( npu, 3 * p ) + 1;
            nkx = nku + 2 * p + 1;

            nkv = NL_MAX( npv, 3 * q ) + 1;
            nky = nkv + 2 * q + 1;

            knx = N_AllocKnotVectorAndArray( nkx, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            kny = N_AllocKnotVectorAndArray( nky, &SL );

            if( kny EQ NULL )
                NL_QUIT;

            error = N_BasisSplitNLongestSpans( knu, p, us, ue, nku, 1, 1, knx );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisSplitNLongestSpans( knv, q, vs, ve, nkv, 1, 1, kny );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, knx, NL_UDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, kny, NL_VDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( surP NEQ surQ )
        {
            N_SrfInitArrays( surQ );
            error = N_SrfCopy( surP, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
        }
        else
        {
            N_SrfGetCPts( surP, &n, &m, &Qw );
        }

        N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
        N_SrfGetKnotVectors( surP, &knu, &knv );

        /* Get local parametrization */

        error = N_BasisFindSpan( knu, p, us, NL_LEFT, &su );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knu, p, ue, NL_LEFT, &eu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, vs, NL_LEFT, &sv );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, ve, NL_LEFT, &ev );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
            su = 1;

        if( ue EQ U[r] )
            eu = n + p;

        if( vs EQ V[0] )
            sv = 1;

        if( ve EQ V[s] )
            ev = m + q;

        if( ref EQ NL_NO AND eu - p - su - 1 LT p )
            NL_ERROR( NL_INP_ERR );

        if( ref EQ NL_NO AND ev - q - sv - 1 LT q )
            NL_ERROR( NL_INP_ERR );

        k = eu - p - su + 1;
        l = ev - q - sv + 1;

        tlu = N_AllocReal1dArray( k, &SL );

        if( tlu EQ NULL )
            NL_QUIT;

        tlv = N_AllocReal1dArray( l, &SL );

        if( tlv EQ NULL )
            NL_QUIT;

        A = N_AllocPt2dArray( k, l, &SL );

        if( A EQ NULL )
            NL_QUIT;

        for ( i = su - 1; i <= eu - p; i++ )
        {
            for ( j = sv - 1; j <= ev - q; j++ )
            {
                N_CPtToPtEuclid( Pw[i][j], &A[i - su + 1][j - sv + 1] );
            }
        }

        error = N_FitCalcSrfParamValues( (NL_VOID ** )A, k, l, NL_EPOINT, NL_CHORDLENGTH, tlu, tlv );

        if( error EQ NL_YES )
            NL_OUT;

        /* Compute distance array */

        Dl = N_AllocReal2dArray( k - 2, l - 2, SQ );

        if( Dl EQ NULL )
            NL_QUIT;

        du = ue - us;
        dv = ve - vs;

        for ( i = 1; i < k; i++ )
        {
            x = us + tlu[i] * du;

            for ( j = 1; j < l; j++ )
            {
                y = vs + tlv[j] * dv;

                N_PtFromXYZ( x, y, 0.0, &A[i - 1][j - 1] );
            }
        }

        error = N_PtsAreInPolygonGravityField( ppl, A, k - 2, l - 2, off, Dl );

        if( error EQ NL_YES )
            NL_OUT;

        /* Output some parameters for interactive step */

        *siu = su;
        *eiu = eu;
        *D = Dl;
        *siv = sv;
        *eiv = ev;
    }

    /* Interactively warp */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Recover original control points */

        if( surP NEQ surQ )
        {
            for ( i = su; i < eu - p; i++ )
            {
                for ( j = sv; j < ev - q; j++ )
                {
                    N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }
        }

        /* Reposition control points */

        if( d GE 0.0 )
        {
            N_CFuncGetDegree( cfnD, &pb );

            error = N_CrvFuncEvalRatBasis( cfnD, pb, 0.5, NL_LEFT, &rmax );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( i = su; i < eu - p; i++ )
        {
            for ( j = sv; j < ev - q; j++ )
            {
                if( Dl[i - su][j - sv]GE 0.0 )
                {
                    N_CPtToPtAndW( Qw[i][j], &Q, &w );

                    t = 0.5 *( 1.0 - (Dl[i - su][j - sv] * o_off) );

                    if( d LT 0.0 )
                    {
                        error = N_CFuncEval( cfnD, t, NL_LEFT, &f );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_CrvFuncEvalRatBasis( cfnD, pb, t, NL_LEFT, &f );

                        if( error EQ NL_YES )
                            NL_OUT;

                        f = (d * f) / rmax;
                    }

                    if( curW NEQ NULL )
                    {
                        error = N_CrvEval( curW, t, NL_LEFT, &W );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    N_VectorBlendPt( f, W, &Q );
                    N_Weight( Q, w, &Qw[i][j] );
                }
            }
        }
    }

    /* Remove unnecessary knots */

    if( flg EQ NL_CLEANUP )
    {
        if( su GE p )
            fku = su + (p + 1) / 2;
        else
            fku = p + 1;

        if( eu LE n )
            lku = eu - (p + 2) / 2;
        else
            lku = n;

        if( sv GE q )
            fkv = sv + (q + 1) / 2;
        else
            fkv = q + 1;

        if( ev LE m )
            lkv = ev - (q + 2) / 2;
        else
            lkv = m;

        error = N_SrfShapeRemoveKnots( surQ, tol, fku, lku, fkv, lkv, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_SRFSHAPEREMOVEKNOTS: Remove all removable knots from a surface being shaped   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  removes  all removable  knots  from a NURBS
     surface. The  routine is  for shape operators  where only a given
     range of knots are to be removed. The  removal is  done in place, 
     i.e. the original surface is destroyed. A typical calling example 
     is:

       NL_SURFACE  sur;
       NL_REAL     tol;
       NL_INDEX    fku, lku, fkv, lkv;
       NL_STACKS   SG;
       ...
       (define sur; get tol, fku, lku, fkv and lkv);
       ...
       N_SrfShapeRemoveKnots(&sur,tol,fku,lku,fkv,lkv,&SG);


   ACCESS:
   
     sur     , in/out ,  NURBS surface
     tol     , input  ,  Tolerance to check removability
     fku,lku , input  ,  Indexes of first and last u-knots
     fkv,lkv , input  ,  Indexes of first and last v-knots
     SG      , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeRemoveKnots( NL_SURFACE *sur, NL_REAL tol, NL_INDEX fku, NL_INDEX lku, NL_INDEX fkv, NL_INDEX lkv, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeRemoveKnots");

    NL_FLAG krm, rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sru, *srv, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru, su, rv, sv, ns, ms;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, *U, *V, *alf, *oma, *bet, *omb, *minl, *maxl, *minr, *maxr, *max, *bru, *brv, lam = 0.0, oml = 0.0, wmin, wmax, pmax, tmp, al, be, ob, bu, bv, stu, stv, wi, wj;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Rw;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL sto = 1.0e-05;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and check indexes */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    if( fku LE p OR lku GT n )
        NL_ERROR( NL_IND_ERR );

    if( fkv LE q OR lkv GT m )
        NL_ERROR( NL_IND_ERR );

    ns = n;
    ms = m;

    /* Adjust removal tolerance in case of rational surfaces */

    if( N_IsSrfRat( sur ) )
    {
        N_SrfMinMaxWeightPosVectors( sur, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    stu = sto * fabs( U[r] - U[0] );
    stv = sto * fabs( V[s] - V[0] );

    /* Allocate local memory */

    i = NL_MAX( p, q );
    j = NL_MAX( n, m );

    alf = N_AllocReal1dArray( 2 * i, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * i, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * i, &SL );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * i, &SL );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt2dArray( j, 2 * i, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( r, s, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( r, s, &SL );

    if( te EQ NULL )
        NL_QUIT;

    minl = N_AllocReal1dArray( i, &SL );

    if( minl EQ NULL )
        NL_QUIT;

    maxl = N_AllocReal1dArray( i, &SL );

    if( maxl EQ NULL )
        NL_QUIT;

    minr = N_AllocReal1dArray( i, &SL );

    if( minr EQ NULL )
        NL_QUIT;

    maxr = N_AllocReal1dArray( i, &SL );

    if( maxr EQ NULL )
        NL_QUIT;

    max = N_AllocReal1dArray( i + 1, &SL );

    if( max EQ NULL )
        NL_QUIT;

    bru = N_AllocReal1dArray( r, &SL );

    if( bru EQ NULL )
        NL_QUIT;

    sru = N_AllocInt1dArray( r, &SL );

    if( sru EQ NULL )
        NL_QUIT;

    brv = N_AllocReal1dArray( s, &SL );

    if( brv EQ NULL )
        NL_QUIT;

    srv = N_AllocInt1dArray( s, &SL );

    if( srv EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( i = 0; i <= r; i++ )
    {
        bru[i] = NL_BIGD;
        sru[i] = 0;

        for ( j = 0; j <= s; j++ )
            er[i][j] = 0.0;
    }

    for ( j = 0; j <= s; j++ )
    {
        brv[j] = NL_BIGD;
        srv[j] = 0;
    }

    /* Compute the maximums of knot removal errors for each distinct knot */

    ru = p + 1;

    while( ru LE n )
    {
        i = ru;

        while( ru LE n AND U[ru]EQ U[ru + 1] )
            ru++;

        sru[ru] = ru - i + 1;

        error = N_SrfRemoveOneKnot( sur, ru, sru[ru], 0, m, NL_UDIR, &bru[ru] );

        if( error EQ NL_YES )
            NL_OUT;

        ru++;
    }

    rv = q + 1;

    while( rv LE m )
    {
        i = rv;

        while( rv LE m AND V[rv]EQ V[rv + 1] )
            rv++;

        srv[rv] = rv - i + 1;

        error = N_SrfRemoveOneKnot( sur, rv, srv[rv], 0, n, NL_VDIR, &brv[rv] );

        if( error EQ NL_YES )
            NL_OUT;

        rv++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        bu = bru[fku];
        su = sru[fku];
        ru = fku;

        for ( i = fku + 1; i <= lku; i++ )
        {
            if( bru[i]LT bu )
            {
                bu = bru[i];
                su = sru[i];
                ru = i;
            }
        }

        bv = brv[fkv];
        sv = srv[fkv];
        rv = fkv;

        for ( j = fkv + 1; j <= lkv; j++ )
        {
            if( brv[j]LT bv )
            {
                bv = brv[j];
                sv = srv[j];
                rv = j;
            }
        }

        /* If no more removable knot -> finished */

        if( bu EQ NL_BIGD AND bv EQ NL_BIGD )
            break;

        if( bu LT bv )
            krm = NL_UDIR;
        else
            krm = NL_VDIR;

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR: /* Remove in the u-direction */

                /* Compute maximums of basis function */

                rmf = NL_TRUE;

                if( (p + su) % 2 )
                {
                    k = (p + su + 1) / 2;
                    l = ru - k + p + 1;
                    al = (U[ru] - U[ru - k]) / (U[ru - k + p + 1] - U[ru - k]);
                    be = (U[ru] - U[ru - k + 1]) / (U[ru - k + p + 2] - U[ru - k + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    error = N_BasisFindAllSpanMaxima( knu, ru - k, p, stu, minl, maxl, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( i = 0; i <= p; i++ )
                        {
                            minl[i] = 0.0;
                            maxl[i] = 1.0;
                        }
                    }

                    error = N_BasisFindAllSpanMaxima( knu, ru - k + 1, p, stu, minr, maxr, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( i = 0; i <= p; i++ )
                        {
                            minr[i] = 0.0;
                            maxr[i] = 1.0;
                        }
                    }

                    max[0] = lam * al * maxl[0];

                    for ( i = 1; i <= p; i++ )
                    {
                        minl[i] *= lam * al;
                        minr[i - 1] *= oml * ob;
                        maxl[i] *= lam * al;
                        maxr[i - 1] *= oml * ob;

                        max[i] = NL_MAX( fabs( maxl[i] - minr[i - 1] ), fabs( maxr[i - 1] - minl[i] ) );
                    }
                    max[p + 1] = oml * ob * maxr[p];
                }
                else
                {
                    k = (p + su) / 2;
                    l = ru - k + p;

                    error = N_BasisFindAllSpanMaxima( knu, ru - k, p, stu, minl, max, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( i = 0; i <= p; i++ )
                            max[i] = 1.0;
                    }
                }

                /* Check the error */

                for ( i = ru - k; i <= l; i++ )
                {
                    if( U[i]NEQ U[i + 1] )
                    {
                        tmp = max[i - ru + k] * bu;

                        for ( j = q; j <= s - q - 1; j++ )
                        {
                            if( V[j]NEQ V[j + 1] )
                            {
                                te[i][j] = er[i][j] + tmp;

                                if( te[i][j]GT tol )
                                {
                                    rmf = NL_FALSE;
                                    break;
                                }
                            }
                        }
                    }

                    if( rmf EQ NL_FALSE )
                        break;
                }

                /* If error test passed -> update error vector and remove knot */

                if( rmf EQ NL_TRUE )
                {
                    for ( i = ru - k; i <= l; i++ )
                    {
                        for ( j = q; j <= s - q - 1; j++ )
                        {
                            if( U[i]NEQ U[i + 1]AND V[j]NEQ V[j + 1] )
                            {
                                er[i][j] = te[i][j];
                            }
                        }
                    }

                    fout = (2 * ru - su - p) / 2;
                    first = ru - p;
                    last = ru - su;
                    off = first - 1;

                    /* Save some parameters */

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        alf[i - first] = (U[i + p + 1] - U[i]) / (U[ru] - U[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[ru]);
                        omb[j - first] = 1.0 - bet[j - first];
                        i++;
                        j--;
                    }

                    /* Remove the knot for each row */

                    wfl = NL_TRUE;

                    for ( col = 0; col <= m; col++ )
                    {
                        i = first;
                        j = last;
                        ii = 1;
                        jj = last - off;

                        N_CopyCPt( Pw[off][col], &Rw[col][0] );
                        N_CopyCPt( Pw[last + 1][col], &Rw[col][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Pw[i][col], oma[i - first], Rw[col][ii - 1], &Rw[col][ii] );
                            N_Combine2CPts( bet[j - first], Pw[j][col], omb[j - first], Rw[col][jj + 1], &Rw[col][jj] );
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
                                N_CPtGetW( Rw[col][i - off], &wi );
                                N_CPtGetW( Rw[col][j - off], &wj );

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

                        if( (p + su) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[col][jj + 1], oml, Rw[col][ii - 1], &Rw[col][ii - 1] );
                        }
                    } /* End for each row */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        bru[ru] = NL_BIGD;
                        continue;
                    }
                    else
                    {
                        /* Save control points */

                        for ( col = 0; col <= m; col++ )
                        {
                            i = first;
                            j = last;

                            while( (j - i)GT 0 )
                            {
                                N_CopyCPt( Rw[col][i - off], &Pw[i][col] );
                                N_CopyCPt( Rw[col][j - off], &Pw[j][col] );
                                i++;
                                j--;
                            }
                        }
                    }

                    /* Successful removal -> shift down some entinties */

                    if( su EQ 1 )
                    {
                        for ( j = q; j <= s - q - 1; j++ )
                        {
                            if( V[j]NEQ V[j + 1] )
                            {
                                er[ru - 1][j] = NL_MAX( er[ru - 1][j], er[ru][j] );
                            }
                        }
                    }

                    if( su GT 1 )
                        sru[ru - 1] = sru[ru] - 1;

                    for ( i = ru + 1; i <= r; i++ )
                    {
                        bru[i - 1] = bru[i];
                        sru[i - 1] = sru[i];
                        U[i - 1] = U[i];

                        for ( j = q; j <= m; j++ )
                            er[i - 1][j] = er[i][j];
                    }

                    for ( col = 0; col <= m; col++ )
                    {
                        for ( i = fout + 1; i <= n; i++ )
                        {
                            N_CopyCPt( Pw[i][col], &Pw[i - 1][col] );
                        }
                    }

                    n--;
                    r--;
                    lku--;
                    N_SrfSetSizeIndices( sur, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( (n EQ p OR fku GT lku)AND( m EQ q OR fkv GT lkv ) )
                        break;

                    /* Update error bounds */

                    k = NL_MAX( ru - p, p + 1 );
                    l = NL_MIN( n, ru + p - su );

                    for ( i = k; i <= l; i++ )
                    {
                        if( U[i]NEQ U[i + 1] )
                        {
                            error = N_SrfRemoveOneKnot( sur, i, sru[i], 0, m, NL_UDIR, &bru[i] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    for ( j = q + 1; j <= s - q - 1; j++ )
                    {
                        if( V[j]NEQ V[j + 1] )
                        {
                            error = N_SrfRemoveOneKnot( sur, j, srv[j], first, last, NL_VDIR, &brv[j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    bru[ru] = NL_BIGD;
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */

                /* Compute maximums of basis functions over each span */

                rmf = NL_TRUE;

                if( (q + sv) % 2 )
                {
                    k = (q + sv + 1) / 2;
                    l = rv - k + q + 1;
                    al = (V[rv] - V[rv - k]) / (V[rv - k + q + 1] - V[rv - k]);
                    be = (V[rv] - V[rv - k + 1]) / (V[rv - k + q + 2] - V[rv - k + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    error = N_BasisFindAllSpanMaxima( knv, rv - k, q, stv, minl, maxl, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( j = 0; j <= q; j++ )
                        {
                            minl[j] = 0.0;
                            maxl[j] = 1.0;
                        }
                    }

                    error = N_BasisFindAllSpanMaxima( knv, rv - k + 1, q, stv, minr, maxr, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( j = 0; j <= q; j++ )
                        {
                            minr[j] = 0.0;
                            maxr[j] = 1.0;
                        }
                    }

                    max[0] = lam * al * maxl[0];

                    for ( j = 1; j <= q; j++ )
                    {
                        minl[j] *= lam * al;
                        minr[j - 1] *= oml * ob;
                        maxl[j] *= lam * al;
                        maxr[j - 1] *= oml * ob;

                        max[j] = NL_MAX( fabs( maxl[j] - minr[j - 1] ), fabs( maxr[j - 1] - minl[j] ) );
                    }
                    max[q + 1] = oml * ob * maxr[q];
                }
                else
                {
                    k = (q + sv) / 2;
                    l = rv - k + q;

                    error = N_BasisFindAllSpanMaxima( knv, rv - k, q, stv, minl, max, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( j = 0; j <= q; j++ )
                            max[j] = 1.0;
                    }
                }

                /* Check the error */

                for ( j = rv - k; j <= l; j++ )
                {
                    if( V[j]NEQ V[j + 1] )
                    {
                        tmp = max[j - rv + k] * bv;

                        for ( i = p; i <= r - p - 1; i++ )
                        {
                            if( U[i]NEQ U[i + 1] )
                            {
                                te[i][j] = er[i][j] + tmp;

                                if( te[i][j]GT tol )
                                {
                                    rmf = NL_FALSE;
                                    break;
                                }
                            }
                        }
                    }

                    if( rmf EQ NL_FALSE )
                        break;
                }

                /* If error test passed -> update error vector and remove knot */

                if( rmf EQ NL_TRUE )
                {
                    for ( j = rv - k; j <= l; j++ )
                    {
                        for ( i = p; i <= r - p - 1; i++ )
                        {
                            if( V[j]NEQ V[j + 1]AND U[i]NEQ U[i + 1] )
                            {
                                er[i][j] = te[i][j];
                            }
                        }
                    }

                    fout = (2 * rv - sv - q) / 2;
                    first = rv - q;
                    last = rv - sv;
                    off = first - 1;

                    /* Save some parameters */

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        alf[i - first] = (V[i + q + 1] - V[i]) / (V[rv] - V[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (V[j + q + 1] - V[j]) / (V[j + q + 1] - V[rv]);
                        omb[j - first] = 1.0 - bet[j - first];
                        i++;
                        j--;
                    }

                    /* Remove knot for each column */

                    wfl = NL_TRUE;

                    for ( row = 0; row <= n; row++ )
                    {
                        i = first;
                        j = last;
                        ii = 1;
                        jj = last - off;

                        N_CopyCPt( Pw[row][off], &Rw[row][0] );
                        N_CopyCPt( Pw[row][last + 1], &Rw[row][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Pw[row][i], oma[i - first], Rw[row][ii - 1], &Rw[row][ii] );
                            N_Combine2CPts( bet[j - first], Pw[row][j], omb[j - first], Rw[row][jj + 1], &Rw[row][jj] );
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
                                N_CPtGetW( Rw[row][i - off], &wi );
                                N_CPtGetW( Rw[row][j - off], &wj );

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

                        if( (q + sv) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[row][jj + 1], oml, Rw[row][ii - 1], &Rw[row][ii - 1] );
                        }
                    } /* End for each column */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        brv[rv] = NL_BIGD;
                        continue;
                    }
                    else
                    {
                        /* Save control points */

                        for ( row = 0; row <= n; row++ )
                        {
                            i = first;
                            j = last;

                            while( (j - i)GT 0 )
                            {
                                N_CopyCPt( Rw[row][i - off], &Pw[row][i] );
                                N_CopyCPt( Rw[row][j - off], &Pw[row][j] );
                                i++;
                                j--;
                            }
                        }
                    }

                    /* Successful removal -> shift down some entinties */

                    if( sv EQ 1 )
                    {
                        for ( i = p; i <= r - p - 1; i++ )
                        {
                            if( U[i]NEQ U[i + 1] )
                            {
                                er[i][rv - 1] = NL_MAX( er[i][rv - 1], er[i][rv] );
                            }
                        }
                    }

                    if( sv GT 1 )
                        srv[rv - 1] = srv[rv] - 1;

                    for ( j = rv + 1; j <= s; j++ )
                    {
                        brv[j - 1] = brv[j];
                        srv[j - 1] = srv[j];
                        V[j - 1] = V[j];

                        for ( i = p; i <= n; i++ )
                            er[i][j - 1] = er[i][j];
                    }

                    for ( row = 0; row <= n; row++ )
                    {
                        for ( j = fout + 1; j <= m; j++ )
                        {
                            N_CopyCPt( Pw[row][j], &Pw[row][j - 1] );
                        }
                    }

                    m--;
                    s--;
                    lkv--;
                    N_SrfSetSizeIndices( sur, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( (n EQ p OR fku GT lku)AND( m EQ q OR fkv GT lkv ) )
                        break;

                    /* Update error bounds */

                    k = NL_MAX( rv - q, q + 1 );
                    l = NL_MIN( m, rv + q - sv );

                    for ( j = k; j <= l; j++ )
                    {
                        if( V[j]NEQ V[j + 1] )
                        {
                            error = N_SrfRemoveOneKnot( sur, j, srv[j], 0, n, NL_VDIR, &brv[j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    for ( i = p + 1; i <= r - p - 1; i++ )
                    {
                        if( U[i]NEQ U[i + 1] )
                        {
                            error = N_SrfRemoveOneKnot( sur, i, sru[i], first, last, NL_UDIR, &bru[i] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    brv[rv] = NL_BIGD;
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* Compact surface */

    if( n LT ns OR m LT ms )
    {
        error = N_SrfCompress( sur, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_SrfShapeCreateRatBasis: Make rational basis function for surface warping         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine creates a rational basis function used to warp
     a surface. It is a  degree  (p,q) rational  basis function  that is 
     defined by symmetrically  placed knots and by unit  weights  except 
     w[p][q] which is used  as a shape  control tool. A  typical calling 
     example is:

       NL_SFUN    sfn;
       NL_REAL    wpq;
       NL_DEGREE  p, q;
       NL_STACKS  S;
       ...
       (get wpq, p and q);
       ...
       N_SrfShapeCreateRatBasis(wpq,p,q,&sfn,NL_NEW,&S);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeCreateRatBasis");
...
         change wpq;
         N_SrfShapeCreateRatBasis(wpq,p,q,&sfn,NL_OLD,&S);
       }

     If  the NL_NEW  option is  chosen, memory  to store  sfn's  members is 
     allocated. During  interactive design, i.e. with the  NL_OLD flag set, 
     w[p][q] is updated and no other change is made.


   ACCESS:
   
     wpq , input  ,  Weight for w[p][q] (the middle weight; the rest are 
                     set 1.0)
     p,q , input  ,  Degrees of rational basis function
     sfn , in/out ,  Rational  basis function  represented  as a surface
                     function
     flg , input  ,  Flag:
                       NL_NEW: allocate memory  for sfn, compute  knots and
                            weights
                       NL_OLD: update middle weight w[p][q]
     SG  , input  ,  sfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeCreateRatBasis( NL_REAL wpq, NL_DEGREE p, NL_DEGREE q, NL_SFUN *sfn, NL_FLAG flg, NL_STACKS *SG )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeCreateRatBasis");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL ** w, *U, *V, uinc, vinc;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute function */

    switch( flg )
    {
        case NL_NEW:

            w = N_AllocReal2dArray( 2 * p, 2 * q, SG );

            if( w EQ NULL )
                NL_QUIT;

            U = N_AllocReal1dArray( 3 * p + 1, SG );

            if( U EQ NULL )
                NL_QUIT;

            V = N_AllocReal1dArray( 3 * q + 1, SG );

            if( V EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= 2 *p; i++ )
            {
                for ( j = 0; j <= 2 *q; j++ )
                    w[i][j] = 1.0;
            }

            w[p][q] = wpq;
            uinc = 1.0 / ((NL_REAL)p + 1.0);
            vinc = 1.0 / ((NL_REAL)q + 1.0);

            for ( i = 0; i <= p; i++ )
            {
                U[2 * p + i + 1] = 1.0;
                U[i] = 0.0;
            }

            for ( i = 1; i <= p; i++ )
                U[p + i] = i * uinc;

            for ( j = 0; j <= q; j++ )
            {
                V[2 * q + j + 1] = 1.0;
                V[j] = 0.0;
            }

            for ( j = 1; j <= q; j++ )
                V[q + j] = j * vinc;

            error = N_SFuncFromKnots( sfn, w, 2 * p, 2 * q, p, q, U, V, 3 * p + 1, 3 * q + 1, SG );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_OLD:

            N_SFuncGetKnots( sfn, &w, &U, &V );

            w[p][q] = wpq;
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
/* N_SrfShapeModifyCPts: Reposition surface control points                        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This shaping routine  repositions surface  control points to shape 
     the surface locally  in a given  direction with  a given distance. 
     Any number of points in any arbitrary arrangement can be marked to 
     be repositioned. To facilitate with random control  point input, a
     1-D array of indexes is  input to the routine. The  transformation
     between the 1-D and 2-D array indexing is as follows:

       (i,j) -> k = i*(m+1)+j

          |-> i = k/(m+1)
       k -|
          |-> j = k%(m+1)

     where m is the  highest index in the  v-direction.  Control points 
     local to the chosen parameters are moved so that  |S(u,v)-Sh(u,v)|
     = d and dir{S(u,v),Sh(u,v)}=T, where d is a given distance, T is a 
       NL_PRIVATE  NL_STRING  rname = "N_SrfShapeModifyCPts");
given  direction vector, and  S(u,v) and  Sh(u,v) are  the surface 
     points before and after modification, respectively. The shaping is 
     done  in place, i.e. the  original surface is destroyed. A typical 
     calling example is:

       NL_SURFACE    sur;
       NL_INDEX      *I, k;
       NL_PARAMETER  u, v;
       NL_VECTOR     T;
       NL_REAL       *gam, d, alf;
       ...
       (define sur; get arrays I and gam; choose u, v, d and T);
       ...
       N_SrfShapeModifyCPts(&sur,I,gam,k,u,v,T,d,&alf,NL_PREPARE);
       while( NOT DONE )
       {
         ...
         get dalf;
         N_SrfShapeModifyCPts(&sur,I,gam,k,u,v,T,d,&dalf,NL_INTERACT);
       }

     To prepare  for interactive  shape modification, several utilities 
     are  provided:  
       N_BasisFindIndexNode,  
       N_BasisFindIndexNodeArray,  
       N_BasisFindNodeSpan and 
       N_BasisFindKnotToTurnParamIntoNode. 
     These utilities allow the designer to compute nodes, find node spans, 
     refine the surface with the appropriate knots, and select the most
     suitable control points. To insert a knot, see the  many utilities
     N_TOO*** in the tools directory.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     I   , input  ,  Index array; control points with indexes I[0],...,
                     I[k] are considered
     gam , input  ,  Weight factors (I[l]-th control  point is weighted 
                     by gam[l])
     k   , input  ,  Highest indexes in I and gam
     u,v , input  ,  Parameters where surface point is to be moved
     T   , input  ,  Direction vector (T=dir{S(u,v),Sh(u,v)})
     d   , input  ,  Distance (d=|S(u,v)-Sh(u,v)|)
     alf , in/out ,  Magnitude of repositioning vector
     flg , input  ,  Flag:
                       NL_PREPARE : get alf and reposition control points
                       NL_INTERACT: reposition control points with dalf


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeModifyCPts( NL_SURFACE *sur, NL_INDEX *I, NL_REAL *gam, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_VECTOR T, NL_REAL d, NL_REAL *alf, NL_FLAG flg )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeModifyCPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, l, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, ktol, mag, R, w, sum;

    NL_POINT P;

    NL_CPOINT ** Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    switch( flg )
    {
        case NL_PREPARE:
            break;

        case NL_INTERACT:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    ktol = NL_MIN( U[r] - U[0], V[s] - V[0] ) * NL_PTOL;

    /* Compute alpha */

    if( flg EQ NL_PREPARE )
    {
        N_VectorMagnitude( T, &mag );

        sum = 0.0;

        for ( l = 0; l <= k; l++ )
        {
            i = I[l] / (m + 1);
            j = I[l] % (m + 1);

            error = N_SrfBasisIEval( sur, i, j, u, v, NL_LEFT, NL_LEFT, &R );

            if( error EQ NL_YES )
                NL_OUT;

            if( R LT ktol )
                gam[l] = 0.0;
            sum += gam[l] * R;
        }

        if( N_FloatOpIsBad( d, mag * sum, NL_DIVISION ) )
            NL_ERROR( NL_INP_ERR );

        *alf = d / (mag * sum);
    }

    /* Reposition control points */

    if( flg EQ NL_PREPARE OR flg EQ NL_INTERACT )
    {
        for ( l = 0; l <= k; l++ )
        {
            i = I[l] / (m + 1);
            j = I[l] % (m + 1);

            N_CPtToPtAndW( Pw[i][j], &P, &w );
            N_VectorBlendPt( *alf * gam[l], T, &P );
            N_Weight( P, w, &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}

/**********************************************************************/
/* N_SrfShapeRegionWarp: Surface region warp                          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  shaping  routine  warps  a  NURBS  surface  over a  specified 
     polygonal region residing in the parameter  space. The warp is done
     either with a predefined warp function or with a bivariate rational 
     basis  function. To facilitate with  various warps, the warp region 
     does  not have to fall within the knot rectangle. If it is (partly)
     outside the knot  rectangle, the warp is  performed on an imaginary  
     (extended)  surface,  however, only  the real  portion is computed. 
     When  interactive change is required, only those quantities are re-
     computed  that  directly  affect the  change in the warp. A typical 
     calling example is:

       NL_SURFACE    surP, surQ, surW;
       EPOLGON    ppl;
       NL_SFUN       sfnD;
       NL_DEGREE     p, q;
       NL_INDEX      npu, npv, ksu, keu, ksv, kev, siu, eiu, siv, eiv;
       NL_VECTOR     W;
       NL_REAL       *tu, *tv, d, tol, wpq;
       NL_BOOLEAN    **IN;
       NL_STACKS     SP, SQ;
       ...
       (define surP; get ppl, npu,...,kev; get either  surW or W; get
        rational basis function and d, or distance function; get tol);
       ...
       N_SrfShapeCreateRatBasis(wpq,p,q,&sfnD,NL_NEW,&SQ);
       ...
       N_SrfShapeRegionWarp(&surP,&ppl,npu,npv,ksu,keu,ksv,kev,NULL,W,&sfnD,d,tol,
                NL_PREPARE,NL_YES,&tu,&tv,&siu,&eiu,&siv,&eiv,&IN,&surQ,
                &SP,&SQ);
       while( NOT DONE )
       {
           NL_PRIVATE  NL_STRING  rname = "N_SrfShapeRegionWarp");
...
         get new wpq;
         N_SrfShapeCreateRatBasis(wpq,p,q,&sfnD,NL_OLD,&SQ);
         ...
         N_SrfShapeRegionWarp(&surP,&ppl,npu,npv,ksu,keu,ksv,kev,NULL,W,&sfnD,d,
                  tol,NL_INTERACT,NL_NO,&tu,&tv,&siu,&eiu,&siv,&eiv,&IN,&surQ,
                  &SP,&SQ);
         ...
         get new d;
         N_SrfShapeRegionWarp(&surP,&ppl,npu,npv,ksu,keu,ksv,kev,NULL,W,&sfnD,d,
                  tol,NL_INTERACT,NL_NO,&tu,&tv,&siu,&eiu,&siv,&eiv,&IN,&surQ,
                  &SP,&SQ);
         ...
       }
       N_SrfShapeRegionWarp(&surP,&ppl,npu,npv,ksu,keu,ksv,kev,NULL,W,&sfnD,d,tol,
                NL_CLEANUP,NL_NO,&tu,&tv,&siu,&eiu,&siv,&eiv,&IN,&surQ,
                &SP,&SQ);

     THE WARPED  NL_SURFACE IS STORED IN  surQ. ALTHOUGH  THE INPUT NL_SURFACE 
     DOES  NOT  CHANGE  EITHER  GEOMETIRICALLY  OR  PARAMETRICALLY,  ITS 
     DEFINITION IS DESTROYED IN THAT ITS KNOT VECTORS ARE REFINED. IF IT  
     IS   UNACCEPTABLE,  EITHER   SAVE  THE  ORIGINAL NL_SURFACE OR  REMOVE 
     UNNECESSARY KNOTS. IF surP = surQ, NL_NO  INTERACTION IS ALLOWED, I.E.
     ONLY THE "NL_PREPARE" AND "NL_CLEANUP" FLAGS ARE NL_USED. THIS CAPABILITY IS
     USEFUL TO PERFORM A SERIES OF WARPS ON THE  INPUT NL_SURFACE surP. THE 
     NL_SURFACE CAN BE REFINED BEFORE  SHAPING. IN THIS CASE THE REFINEMENT 
     IS SKIPPED IN THE  "NL_PREPARE"  STAGE, HOWEVER, THE ROUTINE CHECKS IF 
     AT LEAST  (p+1)x(q+1)  CONTROL NL_POINTS ARE LOCAL TO THE BOUNDING BOX
     OF ppl.


   ACCESS:
   
     surP    , in/out ,  NURBS surface
     ppl     , input  ,  Closed polygon  residing in the papameter space 
                         of surP. It must at least partially overlap the 
                         knot rectangle.
     npu,npv , input  ,  Number of u/v knots to be inserted. Depends  on 
                         the warp distance; for large warps more control 
                         points are needed to obtain a smooth  change in 
                         shape. The default is 3*p/3*q  for small warps.
     ksu,keu , input  ,  Continuity controls in u-direction; the  warped 
                         surface is at most C^{p-ksu} continuous along a
                         u-line bounding the warp  region from the left,
                         and  C^{p-keu}  continuous  along  an  iso-line 
                         bounding the  region from the right (1<=ksu,keu
                         <=p!). To obtain a u-crease, set ksu=keu=p.
     ksv,kev , input  ,  The same as above applied to the v-direction
     surW    , input  ,  Direction surface:
                            = NULL: use input vector W
                           != NULL: use surW
     W       , input  ,  Warp direction
     sfnD    , input  ,  Warp  distance function  or bivariate  rational 
                         basis function  (must be  parametrized  between 
                         [0,1]!)
     d       , input  ,  Warp distance:
                           > 0.0: sfnD is rational basis function
                           < 0.0: sfnD is distance function
     tol     , input  ,  Knot removal  tolerance. For  shaping accuracy, 
                         1% of the surface's size is a good default
     flg     , input  ,  Flag:
                           NL_PREPARE : get some entities and do first warp
                           NL_INTERACT: change warp by changing d, W, etc
                           NL_CLEANUP : remove unneccesary knots
     ref     , input  ,  Flag:
                           NL_YES: refine surface
                           NL_NO : surface is already refined
     tu,tv   , in/out ,  Local parametrizations
     siu,eiu , in/out ,  U-indexes of local control points
     siv,eiv , in/out ,  V-indexes of local control points
     IN      , in/out ,  Boolean array  indicating which  local  control 
                         point is inside ppl
     surQ    , output ,  Warped surface; if surP=surQ, only the  NL_PREPARE
                         and NL_CLEANUP options are allowed
     SP      , input  ,  surP's stack
     SQ      , input  ,  Stack holding pointers surQ, tu, tv and IN
    


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfShapeRegionWarp( NL_SURFACE *surP, NL_EPOLYGON *ppl, NL_INDEX npu, NL_INDEX npv, NL_INDEX ksu, NL_INDEX keu, NL_INDEX ksv, NL_INDEX kev, NL_SURFACE *surW, NL_VECTOR W, NL_SFUN *sfnD, NL_REAL d, NL_REAL tol, NL_FLAG flg, NL_FLAG ref, NL_REAL ** tu, NL_REAL ** tv, NL_INDEX *siu, NL_INDEX *eiu, NL_INDEX *siv, NL_INDEX *eiv, NL_BOOLEAN *** IN, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfShapeRegionWarp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, su = 0, eu = 0, sv = 0, ev = 0, nku, nkv, nkx, nky, fku, lku, fkv, lkv;

    NL_DEGREE p, q, pb = 0, qb = 0;

    NL_REAL *U, *V, *tlu = NULL, *tlv = NULL, ul, ur, vl, vr, tsu, teu, tsv, tev, rmax = 0.0, w, f, us, ue, vs, ve, du, dv, x, y;

    NL_POINT ** A, Q, R;

    NL_CPOINT ** Pw, ** Qw = NULL, Tw;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL, *knx, *kny;

    NL_MINMAXBOX box;

    NL_BOOLEAN ** PIP = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notations and check input */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    switch( flg )
    {
        case NL_PREPARE:

            N_SrfGetKnotVectors( surP, &knu, &knv );
            break;

        case NL_INTERACT:
            if( surP EQ surQ )
                NL_ERROR( NL_INP_ERR );

            su = *siu;
            eu = *eiu;
            tlu = *tu;
            PIP = *IN;
            sv = *siv;
            ev = *eiv;
            tlv = *tv;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
            break;

        case NL_CLEANUP:

            su = *siu;
            eu = *eiu;
            sv = *siv;
            ev = *eiv;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Prepare for warp */

    if( flg EQ NL_PREPARE )
    {
        /* Set knot rectangle */

        N_PolygonGetBBox( ppl, &box );
        N_GetBBoxData( &box, &us, &ue, &vs, &ve, &f, &f );

        if( ue LE U[0]OR us GE U[r] )
            NL_ERROR( NL_INP_ERR );

        if( ve LE V[0]OR vs GE V[s] )
            NL_ERROR( NL_INP_ERR );

        if( us LT U[0] )
            ul = U[0];
        else
            ul = us;

        if( ue GT U[r] )
            ur = U[r];
        else
            ur = ue;

        if( vs LT V[0] )
            vl = V[0];
        else
            vl = vs;

        if( ve GT V[s] )
            vr = V[s];
        else
            vr = ve;

        /* Refine surface */

        if( ref EQ NL_YES )
        {
            f = (ur - ul) / (ue - us);
            nku = (NL_INDEX)(f * NL_MAX( npu, 3 * p )) + 1;
            nkx = nku + 2 * p + 1;

            f = (vr - vl) / (ve - vs);
            nkv = (NL_INDEX)(f * NL_MAX( npv, 3 * q )) + 1;
            nky = nkv + 2 * q + 1;

            knx = N_AllocKnotVectorAndArray( nkx, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            kny = N_AllocKnotVectorAndArray( nky, &SL );

            if( kny EQ NULL )
                NL_QUIT;

            error = N_BasisSplitNLongestSpans( knu, p, ul, ur, nku, ksu, keu, knx );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisSplitNLongestSpans( knv, q, vl, vr, nkv, ksv, kev, kny );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, knx, NL_UDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_SrfInsertKnots( surP, kny, NL_VDIR, surP, SP, SP );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( surP NEQ surQ )
        {
            N_SrfInitArrays( surQ );
            error = N_SrfCopy( surP, surQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPts( surQ, &n, &m, &Qw );
        }
        else
        {
            N_SrfGetCPts( surP, &n, &m, &Qw );
        }

        N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
        N_SrfGetKnotVectors( surP, &knu, &knv );

        /* Get local parametrization */

        error = N_BasisFindSpan( knu, p, ul, NL_LEFT, &su );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knu, p, ur, NL_LEFT, &eu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, vl, NL_LEFT, &sv );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, vr, NL_LEFT, &ev );

        if( error EQ NL_YES )
            NL_OUT;

        if( us EQ U[0] )
            su = 1;

        if( us LT U[0] )
            su = 0;

        if( ue EQ U[r] )
            eu = n + p;

        if( ue GT U[r] )
            eu = n + p + 1;

        if( vs EQ V[0] )
            sv = 1;

        if( vs LT V[0] )
            sv = 0;

        if( ve EQ V[s] )
            ev = m + q;

        if( ve GT V[s] )
            ev = m + q + 1;

        if( ref EQ NL_NO AND eu - p - su - 1 LT p )
            NL_ERROR( NL_INP_ERR );

        if( ref EQ NL_NO AND ev - q - sv - 1 LT q )
            NL_ERROR( NL_INP_ERR );

        k = eu - p - su + 1;
        l = ev - q - sv + 1;

        tlu = N_AllocReal1dArray( k, SQ );

        if( tlu EQ NULL )
            NL_QUIT;

        tlv = N_AllocReal1dArray( l, SQ );

        if( tlv EQ NULL )
            NL_QUIT;

        A = N_AllocPt2dArray( k, l, &SL );

        if( A EQ NULL )
            NL_QUIT;

        for ( i = su; i < eu - p; i++ )
        {
            if( sv EQ 0 )
            {
                N_Combine2CPts( 2.0, Pw[i][0], -1.0, Pw[i][1], &Tw );
                N_CPtToPtEuclid( Tw, &A[i - su + 1][0] );
            }
            else
            {
                N_CPtToPtEuclid( Pw[i][sv - 1], &A[i - su + 1][0] );
            }

            if( ev EQ m + q + 1 )
            {
                N_Combine2CPts( 2.0, Pw[i][m], -1.0, Pw[i][m - 1], &Tw );
                N_CPtToPtEuclid( Tw, &A[i - su + 1][l] );
            }
            else
            {
                N_CPtToPtEuclid( Pw[i][ev - q], &A[i - su + 1][l] );
            }
        }

        for ( j = sv; j < ev - q; j++ )
        {
            if( su EQ 0 )
            {
                N_Combine2CPts( 2.0, Pw[0][j], -1.0, Pw[1][j], &Tw );
                N_CPtToPtEuclid( Tw, &A[0][j - sv + 1] );
            }
            else
            {
                N_CPtToPtEuclid( Pw[su - 1][j], &A[0][j - sv + 1] );
            }

            if( eu EQ n + p + 1 )
            {
                N_Combine2CPts( 2.0, Pw[n][j], -1.0, Pw[n - 1][j], &Tw );
                N_CPtToPtEuclid( Tw, &A[k][j - sv + 1] );
            }
            else
            {
                N_CPtToPtEuclid( Pw[eu - p][j], &A[k][j - sv + 1] );
            }
        }

        for ( i = su; i < eu - p; i++ )
        {
            for ( j = sv; j < ev - q; j++ )
            {
                N_CPtToPtEuclid( Pw[i][j], &A[i - su + 1][j - sv + 1] );
            }
        }

        N_Combine2Pts( 2.0, A[1][0], -1.0, A[2][0], &Q );
        N_Combine2Pts( 2.0, A[0][1], -1.0, A[0][2], &R );
        N_Combine2Pts( 0.5, Q, 0.5, R, &A[0][0] );

        N_Combine2Pts( 2.0, A[1][l], -1.0, A[2][l], &Q );
        N_Combine2Pts( 2.0, A[0][l - 1], -1.0, A[0][l - 2], &R );
        N_Combine2Pts( 0.5, Q, 0.5, R, &A[0][l] );

        N_Combine2Pts( 2.0, A[k - 1][0], -1.0, A[k - 2][0], &Q );
        N_Combine2Pts( 2.0, A[k][1], -1.0, A[k][2], &R );
        N_Combine2Pts( 0.5, Q, 0.5, R, &A[k][0] );

        N_Combine2Pts( 2.0, A[k - 1][l], -1.0, A[k - 2][l], &Q );
        N_Combine2Pts( 2.0, A[k][l - 1], -1.0, A[k][l - 2], &R );
        N_Combine2Pts( 0.5, Q, 0.5, R, &A[k][l] );

        error = N_FitCalcSrfParamValues( (NL_VOID ** )A, k, l, NL_EPOINT, NL_CHORDLENGTH, tlu, tlv );

        if( error EQ NL_YES )
            NL_OUT;

        /* Rescale parameters */

        if( us LT U[0]OR ue GT U[r] )
        {
            if( us LT U[0] )
                tsu = (ul - us) / (ue - us);
            else
                tsu = 0.0;

            if( ue GT U[r] )
                teu = (ur - us) / (ue - us);
            else
                teu = 1.0;

            tlu[0] = tsu;
            tlu[k] = teu;

            for ( i = 1; i < k; i++ )
                tlu[i] = tsu + tlu[i] * (teu - tsu);
        }

        if( vs LT V[0]OR ve GT V[s] )
        {
            if( vs LT V[0] )
                tsv = (vl - vs) / (ve - vs);
            else
                tsv = 0.0;

            if( ve GT V[s] )
                tev = (vr - vs) / (ve - vs);
            else
                tev = 1.0;

            tlv[0] = tsv;
            tlv[l] = tev;

            for ( j = 1; j < l; j++ )
                tlv[j] = tsv + tlv[j] * (tev - tsv);
        }

        /* Compute Boolean array */

        PIP = N_AllocInt2dArray( k - 2, l - 2, SQ );

        if( PIP EQ NULL )
            NL_QUIT;

        du = ue - us;
        dv = ve - vs;

        for ( i = 1; i < k; i++ )
        {
            x = us + tlu[i] * du;

            for ( j = 1; j < l; j++ )
            {
                y = vs + tlv[j] * dv;

                N_PtFromXYZ( x, y, 0.0, &A[i - 1][j - 1] );
            }
        }

        error = N_PtsAreInPolygon( ppl, A, k - 2, l - 2, PIP );

        if( error EQ NL_YES )
            NL_OUT;

        /* Output some parameters for interactive step */

        *siu = su;
        *eiu = eu;
        *tu = tlu;
        *siv = sv;
        *eiv = ev;
        *tv = tlv;
        *IN = PIP;
    }

    /* Interactively warp */

    if( flg EQ NL_INTERACT OR flg EQ NL_PREPARE )
    {
        /* Recover original control points */

        if( surP NEQ surQ )
        {
            for ( i = su; i < eu - p; i++ )
            {
                for ( j = sv; j < ev - q; j++ )
                {
                    N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }
        }

        /* Reposition control points */

        if( d GE 0.0 )
        {
            N_SFuncGetDegrees( sfnD, &pb, &qb );

            error = N_SrfFuncEvalRatBasis( sfnD, pb, qb, 0.5, 0.5, NL_LEFT, NL_LEFT, &rmax );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( i = su; i < eu - p; i++ )
        {
            for ( j = sv; j < ev - q; j++ )
            {
                if( PIP[i - su][j - sv] )
                {
                    N_CPtToPtAndW( Qw[i][j], &Q, &w );

                    if( d LT 0.0 )
                    {
                        error = N_SrfFuncEvalPt( sfnD, tlu[i - su + 1], tlv[j - sv + 1], NL_LEFT, NL_LEFT, &f );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }
                    else
                    {
                        error = N_SrfFuncEvalRatBasis( sfnD, pb, qb, tlu[i - su + 1], tlv[j - sv + 1], NL_LEFT, NL_LEFT, &f );

                        if( error EQ NL_YES )
                            NL_OUT;

                        f = (d * f) / rmax;
                    }

                    if( surW NEQ NULL )
                    {
                        error = N_SrfEvalPt( surW, tlu[i - su + 1], tlv[j - sv + 1], NL_LEFT, NL_LEFT, &W );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    N_VectorBlendPt( f, W, &Q );
                    N_Weight( Q, w, &Qw[i][j] );
                }
            }
        }
    }

    /* Remove unnecessary knots */

    if( flg EQ NL_CLEANUP )
    {
        if( su GE p )
            fku = su + (p + 1) / 2;
        else
            fku = p + 1;

        if( eu LE n )
            lku = eu - (p + 2) / 2;
        else
            lku = n;

        if( sv GE q )
            fkv = sv + (q + 1) / 2;
        else
            fkv = q + 1;

        if( ev LE m )
            lkv = ev - (q + 2) / 2;
        else
            lkv = m;

        error = N_SrfShapeRemoveKnots( surQ, tol, fku, lku, fkv, lkv, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
#endif // NLIB_UNUSED
