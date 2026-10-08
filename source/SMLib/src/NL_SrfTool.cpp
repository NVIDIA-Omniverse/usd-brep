// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************/
/* SrfTool.c : Tool Function Definitions that act on NL_SURFACE objects */
/************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

NL_PRIVATE NL_REAL sto = 1.0e-05;
NL_PRIVATE NL_REAL NOREM = 1.0e+25;
// NL_PRIVATE NL_REAL eps = 1.0e-03;

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  inserts one new  knot into a  NURBS surface 
     either in u- or in v-direction. The  new knot must be an interior 
     knot and the  sum of the  multiplicities of  the old and  the new 
     knots must be less than or equal to the respective degree. If the  
     output  surface  is  initialized  to  NULL, memory  to  store new 
     control points and knots is allocated. A typical  calling example
     is as follows:

       NL_SURFACE    surP, surQ;
       NL_PARAMETER  t;
       NL_INDEX      mt;
       NL_STACKS     SP, SQ;
       ...
       (define surP, get t and mt);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfInsertKnot(&surP,t,mt,NL_UDIR,&surQ,&SP,&SQ);
       N_SrfInsertKnot(&surP,t,mt,NL_VDIR,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects. If surP is the same as surQ, the
     insertion is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     t    , input  ,  New knot to be inserted
     mt   , input  ,  Number of times t is to be inserted
     dir  , input  ,  Flag:
                        NL_UDIR: Insert in u-direction
                        NL_VDIR: Insert in v-direction
     surQ , output ,  Surface after knot inserion
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfInsertKnot
 (NL_SURFACE  *surP,  /* in : tgt surface */
  NL_PARAMETER t,     /* in : new knot to be inserted */
  NL_INDEX     mt,    /* in : number of times to insert new knot t */
  NL_FLAG      dir,   /* in : NL_UDIR: Insert in u-direction */
                      /*      NL_VDIR: Insert in v-direction */
  NL_SURFACE  *surQ,  /* out: Surface after knot inserion */
  NL_STACKS   *SP,    /* in : surP's stack */
  NL_STACKS   *SQ )   /* in : surQ's stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfInsertKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, spu = 0, mlu = 0 , spv = 0, mlv = 0, a, ni, mi, ri, si;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, ** alf, ** oma;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, *Rw;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlu + mt)GT p OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            ni = n + mt;
            mi = m;
            ri = r + mt;
            si = s;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlv + mt)GT q OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            ni = n;
            mi = m + mt;
            ri = r;
            si = s + mt;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        surA = *surP;

        error = N_AllocSrfArrays( surP, ni, mi, p, q, ri, si, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, ni, mi, p, q, ri, si, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Allocate local memory */

    a = NL_MAX( p, q );
    alf = N_AllocReal2dArray( a, a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal2dArray( a, a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( a, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Insert the knot */

    if( dir EQ NL_UDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= mt; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf[i][j] = (t - UP[a + j]) / (UP[spu + j + 1] - UP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each row do */

        for ( l = 0; l <= m; l++ )
        {
            a = spu - p;

            /* Load auxiliary control points */

            for ( i = 0; i <= p - mlu; i++ )
                N_CopyCPt( Pw[a + i][l], &Rw[i] );

            /* Save unaltered control points */

            for ( i = 0; i <= a; i++ )
                N_CopyCPt( Pw[i][l], &Qw[i][l] );

            for ( i = spu - mlu; i <= n; i++ )
                N_CopyCPt( Pw[i][l], &Qw[i + mt][l] );

            /* Now insert the knot */

            for ( i = 1; i <= mt; i++ )
            {
                a = spu - p + i;

                for ( j = 0; j <= p - i - mlu; j++ )
                {
                    N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                }
                N_CopyCPt( Rw[0], &Qw[a][l] );
                N_CopyCPt( Rw[p - i - mlu], &Qw[spu + mt - i - mlu][l] );
            }

            /* Load the remaining control points */

            for ( i = a + 1; i < spu - mlu; i++ )
                N_CopyCPt( Rw[i - a], &Qw[i][l] );
        } /* End for each row */

        /* Load knot vectors */

        for ( i = 0; i <= spu; i++ )
            UQ[i] = UP[i];

        for ( i = 1; i <= mt; i++ )
            UQ[i + spu] = t;

        for ( i = spu + 1; i <= r; i++ )
            UQ[i + mt] = UP[i];

        for ( i = 0; i <= s; i++ )
            VQ[i] = VP[i];
    } /* End of NL_UDIR */

    if( dir EQ NL_VDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= mt; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf[i][j] = (t - VP[a + j]) / (VP[spv + j + 1] - VP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column do */

        for ( k = 0; k <= n; k++ )
        {
            a = spv - q;

            /* Load auxiliary control points */

            for ( i = 0; i <= q - mlv; i++ )
                N_CopyCPt( Pw[k][a + i], &Rw[i] );

            /* Save unaltered control points */

            for ( i = 0; i <= a; i++ )
                N_CopyCPt( Pw[k][i], &Qw[k][i] );

            for ( i = spv - mlv; i <= m; i++ )
                N_CopyCPt( Pw[k][i], &Qw[k][i + mt] );

            /* Now insert the knot */

            for ( i = 1; i <= mt; i++ )
            {
                a = spv - q + i;

                for ( j = 0; j <= q - i - mlv; j++ )
                {
                    N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                }
                N_CopyCPt( Rw[0], &Qw[k][a] );
                N_CopyCPt( Rw[q - i - mlv], &Qw[k][spv + mt - i - mlv] );
            }

            /* Load the remaining control points */

            for ( i = a + 1; i < spv - mlv; i++ )
                N_CopyCPt( Rw[i - a], &Qw[k][i] );
        } /* End for each column */

        /* Load knot vectors */

        for ( i = 0; i <= spv; i++ )
            VQ[i] = VP[i];

        for ( i = 1; i <= mt; i++ )
            VQ[i + spv] = t;

        for ( i = spv + 1; i <= s; i++ )
            VQ[i + mt] = VP[i];

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];
    } /* End of NL_VDIR */

    /* If insertion is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfInsertKnot */


/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  splits a NURBS  surface into two  surfaces 
     either in u- or in v-direction. The split must be at an interior 
     parameter value. 
     If the output surface is initialized to NULL AND the Stacks SG
     are Non-NULL, memory to store curve control points and knots is 
     allocated.
     A typical calling example is:

       NL_SURFACE    sur, surL, surR;
       NL_PARAMETER  t;
       NL_STACKS     SG;
       ...
       (define sur, get t);
       ...
       N_SrfInitArrays(&surL);
       N_SrfInitArrays(&surR);
       N_SrfSplit(&sur,t,NL_UDIR,&surL,&surR,&SG);

     If memory is  available, surL  and surR are not  initialized and 
     the  routine  assumes  that  memory  allocation  has  been done. 
     However, it  checks  for  the  proper  amount by  looking at the 
     highest  indexes  in  surL's an  surR's  knot vector and polygon 
     objects. 


   ACCESS:
   
     sur  , input  ,  NURBS surface to be split
     t    , input  ,  Parameter where surface is to be split
     dir  , input  ,  Flag:
                        NL_UDIR: Split in u-direction
                        NL_VDIR: Split in v-direction
     surL , output ,  Left half of surface defined  over [U[0],t]  or
                      [V[0],t]
     surR , output ,  Right half of surface defined over [t,U[r]]  or
                      [t,V[s]]
     SG   , input  ,  (opt) surL's and surR's memory stack pointer or NULL



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_SrfSplit
 ( NL_SURFACE   * sur,          // in : TgtSurface
   NL_PARAMETER   dSplitParam,  // in : Tgt SplitParam
   NL_FLAG        dir,          // in : SplitDir: NL_UDIR=Split KnotVectorU, NL_VDir=Split KnotVectorV
   NL_SURFACE   * surL,         // out: Split surface result, Ivl=[MinParam, TgtParam]
   NL_SURFACE   * surR,         // out: Split surface result, Ivl=[TgtParam, MaxParam]
   NL_STACKS    * SG )          // in : stack for new object memory
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfSplit");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, spu = 0, mlu = 0, spv = 0, mlv = 0, a, b, c, nl, ml, rl, sl, nr, mr, rr, sr;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UL, *VL, *UR, *VR;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Lw, ** Rw, Sw[NL_MAXDEG + 1];

    NL_REAL *alf[NL_MAXDEG + 1];
    NL_REAL stack_alf[NL_MAXDEG + 1][NL_MAXDEG + 1];
    NL_REAL *oma[NL_MAXDEG + 1];
    NL_REAL stack_oma[NL_MAXDEG + 1][NL_MAXDEG + 1];

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    knu = sur->knu;
    knv = sur->knv;

    a = NL_MAX( p, q );

    for ( i = 0; i <= a; i++ )
    {
        alf[i] = &stack_alf[i][0];
        oma[i] = &stack_oma[i][0];
    }

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, dSplitParam, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knu, p, dSplitParam, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;

            if( mlu > p )
            {
                mlu = p; /* Bad knot vector */
                NL_OUT;
            }

            nl = spu - mlu;
            ml = m;
            rl = nl + p + 1;
            sl = s;
            nr = n + p - spu;
            mr = m;
            rr = nr + p + 1;
            sr = s;

            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, dSplitParam, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knv, q, dSplitParam, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            if( mlv > q )
            {
                mlv = q; /* Bad knot vector */
                NL_OUT;
            }

            nl = n;
            ml = spv - mlv;
            rl = r;
            sl = ml + q + 1;
            nr = n;
            mr = m + q - spv;
            rr = r;
            sr = mr + q + 1;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( SG != NULL )
    {
        error = N_SrfSizeArrays( surL, nl, ml, p, q, rl, sl, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_SrfSizeArrays( surR, nr, mr, p, q, rr, sr, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    Lw = surL->net->Pw;
    UL = surL->knu->U;
    VL = surL->knv->U;

    Rw = surR->net->Pw;
    UR = surR->knu->U;
    VR = surR->knv->U;

    /* Split the surface */

    if( dir EQ NL_UDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= p - mlu; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf[i][j] = (dSplitParam - U[a + j]) / (U[spu + j + 1] - U[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each row do */

        a = spu - p;

        for ( l = 0; l <= m; l++ )
        {
            /* Load auxiliary control points */

            for ( i = 0; i <= p - mlu; i++ )
                N_CopyCPt( Pw[a + i][l], &Sw[i] );

            /* Save unaltered control points */

            for ( i = 0; i <= a; i++ )
                N_CopyCPt( Pw[i][l], &Lw[i][l] );

            for ( i = spu - mlu; i <= n; i++ )
                N_CopyCPt( Pw[i][l], &Rw[i - a][l] );

            /* Now split the surface */

            for ( i = 1; i <= p - mlu; i++ )
            {
                b = a + i;
                c = p - mlu - i;

                for ( j = 0; j <= p - i - mlu; j++ )
                {
                    N_Combine2CPts( alf[i][j], Sw[j + 1], oma[i][j], Sw[j], &Sw[j] );
                }
                N_CopyCPt( Sw[0], &Lw[b][l] );
                N_CopyCPt( Sw[c], &Rw[c][l] );
            }
        } /* End for each row */

        /* Load knot vectors */

        a = spu - mlu;

        for ( i = 0; i <= a; i++ )
            UL[i] = U[i];

        for ( i = 0; i <= p; i++ )
            UL[a + i + 1] = dSplitParam;

        a = spu + 1;

        for ( i = 0; i <= p; i++ )
            UR[i] = dSplitParam;

        for ( i = a; i <= r; i++ )
            UR[i - a + p + 1] = U[i];

        for ( i = 0; i <= s; i++ )
            VL[i] = VR[i] = V[i];
    } /* End of NL_UDIR */

    if( dir EQ NL_VDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= q - mlv; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf[i][j] = (dSplitParam - V[a + j]) / (V[spv + j + 1] - V[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column do */

        a = spv - q;

        for ( k = 0; k <= n; k++ )
        {
            /* Load auxiliary control points */

            for ( i = 0; i <= q - mlv; i++ )
                N_CopyCPt( Pw[k][a + i], &Sw[i] );

            /* Save unaltered control points */

            for ( i = 0; i <= a; i++ )
                N_CopyCPt( Pw[k][i], &Lw[k][i] );

            for ( i = spv - mlv; i <= m; i++ )
                N_CopyCPt( Pw[k][i], &Rw[k][i - a] );

            /* Now split the surface */

            for ( i = 1; i <= q - mlv; i++ )
            {
                b = a + i;
                c = q - mlv - i;

                for ( j = 0; j <= q - i - mlv; j++ )
                {
                    N_Combine2CPts( alf[i][j], Sw[j + 1], oma[i][j], Sw[j], &Sw[j] );
                }
                N_CopyCPt( Sw[0], &Lw[k][b] );
                N_CopyCPt( Sw[c], &Rw[k][c] );
            }
        } /* End for each column */

        /* Load knot vectors */

        a = spv - mlv;

        for ( i = 0; i <= a; i++ )
            VL[i] = V[i];

        for ( i = 0; i <= q; i++ )
            VL[a + i + 1] = dSplitParam;

        a = spv + 1;

        for ( i = 0; i <= q; i++ )
            VR[i] = dSplitParam;

        for ( i = a; i <= s; i++ )
            VR[i - a + q + 1] = V[i];

        for ( i = 0; i <= r; i++ )
            UL[i] = UR[i] = U[i];
    } /* End of NL_VDIR */

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfSplit */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine performs inverse knot  insertion, i.e. given a
     point on one of the control net legs, it inserts a knot such that 
     the  given  point becomes  a new  control  point. If  the  output 
     surface is  initialized  to NULL, memory  to  store  new  control 
     points and knots is allocated. A typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_POINT    P;
       NL_STACKS   SP, SQ;
       ...
       (define surP, get P);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfInsertKnotPt(&surP,P,&surQ,&SP,&SQ);
       N_SrfInsertKnotPt(&surP,P,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects. If surP is the same as surQ, the
     insertion is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     P    , input  ,  Point to become a control point
     surQ , output ,  Surface after knot inserion
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfInsertKnotPt( NL_SURFACE *surP, NL_POINT P, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfInsertKnotPt");

    NL_FLAG flg, dir, error = NL_NO;

    NL_INDEX i, j, i1, j1, n, m;

    NL_DEGREE p, q;

    NL_REAL *U, *V, dl, dr, alf, t, w, w1;

    NL_POINT ** R, Q;

    NL_CPOINT ** Pw;

    NL_ENET ntl;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &i, &j, &U, &V );

    /* Get Euclidean polygon */

    error = N_SrfGetENet( surP, 0, n, 0, m, &ntl, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_ENetGetPts( &ntl, &i, &j, &R );

    /* Get leg index */

    error = N_NetGetClosestLegIndex( &ntl, P, &Q, &i, &j, &dir, &alf, &flg );

    if( error EQ NL_YES )
        NL_OUT;

    if( flg EQ NL_FALSE )
        NL_ERROR( NL_INP_ERR );

    /* Get knot to be inserted */

    if( dir EQ NL_UDIR )
    {
        i1 = i + 1;
        j1 = j;
    }
    else
    {
        i1 = i;
        j1 = j + 1;
    }

    if( N_IsSrfRat( surP ) )
    {
        N_DistPtPt( R[i][j], Q, &dl );
        N_DistPtPt( Q, R[i1][j1], &dr );
        N_CPtGetW( Pw[i][j], &w );
        N_CPtGetW( Pw[i1][j1], &w1 );
        alf = w * dl / (w * dl + w1 * dr);
    }

    if( dir EQ NL_UDIR )
    {
        t = U[i + 1] + alf * (U[i + p + 1] - U[i + 1]);
    }
    else
    {
        t = V[j + 1] + alf * (V[j + q + 1] - V[j + 1]);
    }

    /* Insert the knot */

    error = N_SrfInsertKnot( surP, t, 1, dir, surQ, SP, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfInsertKnotPt */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools routine decomposes a  NURBS  surface  into its Bezier 
     constituents  without  using  knot   refinement.  Each  piece  is 
     represented as a NURBS surface even though the patches are Bezier
     surfaces. MEMORY TO STORE THE OUTPUT SURFACES IS ALLOCATED INSIDE 
     THE ROUTINE. A typical calling example is:

       NL_SURFACE surP, ***surQ;
       NL_INDEX   i, j, kk, ll;
       NL_CPOINT  **Pw;
       NL_STACKS  SQ;
       ...
       (define surP);
       ...
       N_SrfDecomposeToBez(&surP,&surQ,&kk,&ll,&SQ);
       ...
       Pw = surQ[i][j]->net->Pw;
       ...

     surQ[i][j],  0<=i<=k,  0<=j<=l, is  a  pointer  to  the  (i,j)-th 
     surface.


   ACCESS:
   
     surP  , input  ,  NURBS surface to be decomposed
     surQ  , output ,  2-D array of Bezier surfaces
     kk,ll , output ,  Highest indexes in surQ
     SQ    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfDecomposeToBez( NL_SURFACE *surP, NL_SURFACE **** surQ, NL_INDEX *kk, NL_INDEX *ll, NL_STACKS *SQ )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, ru, su, rv, sv, nsu, nsv, mlu, mlv, isu, ieu, isv, iev, iq, jq, save;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, *uals, *vals, *omus, *omvs, num = 0.0;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, ** NQw = NULL, ** Bw, ** NBw, ** tmp;

    NL_SURFACE *** surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* Allocate memory */

    N_BasisGetSpanCount( knu, p, &nsu );
    N_BasisGetSpanCount( knv, q, &nsv );

    surA = N_Alloc2dArraySrfPtrsParameters( p, q, p, q, 2 * p + 1, 2 * q + 1, nsu - 1, nsv - 1, SQ );

    if( surA EQ NULL )
        NL_QUIT;

    Bw = N_AllocCPt2dArray( p, m, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    NBw = N_AllocCPt2dArray( p, m, &SL );

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

    /* Initialize for u-directional decomposition */

    isu = p;
    ieu = p + 1;
    iq = -1;

    for ( i = 0; i <= p; i++ )
    {
        for ( l = 0; l <= m; l++ )
            N_CopyCPt( Pw[i][l], &Bw[i][l] );
    }

    /* Decompose in u-direction into Bezier strips */

    while( ieu LT r )
    {
        iq = iq + 1;

        /* Get knot multiplicity */

        i = ieu;

        while( ieu LT r AND UP[ieu]EQ UP[ieu + 1] )
            ieu++;
        mlu = ieu - i + 1;
        ru = p - mlu;

        /* Insert the knot */

        if( mlu LT p )
        {
            num = UP[ieu] - UP[isu];

            for ( i = p; i > mlu; i-- )
            {
                uals[i - mlu - 1] = num / (UP[isu + i] - UP[isu]);
                omus[i - mlu - 1] = 1.0 - uals[i - mlu - 1];
            }

            for ( i = 1; i <= ru; i++ )
            {
                su = mlu + i;
                save = ru - i;

                for ( j = p; j >= su; j-- )
                {
                    for ( l = 0; l <= m; l++ )
                    {
                        N_Combine2CPts( uals[j - su], Bw[j][l], omus[j - su], Bw[j - 1][l], &Bw[j][l] );
                    }
                }

                if( ieu LT r )
                {
                    for ( l = 0; l <= m; l++ )
                    {
                        N_CopyCPt( Bw[p][l], &NBw[save][l] );
                    }
                }
            }
        }

        /* Bezier strip completed. Initialize for */
        /* v-directional decomposition            */

        isv = q;
        iev = q + 1;
        jq = -1;

        N_SrfGetCPtsAndKnots( surA[iq][0], &Qw, &UQ, &VQ );

        for ( j = 0; j <= q; j++ )
        {
            for ( k = 0; k <= p; k++ )
                N_CopyCPt( Bw[k][j], &Qw[k][j] );
        }

        /* Decompose in v-direction into Bezier patches */

        while( iev LT s )
        {
            jq = jq + 1;

            N_SrfGetCPtsAndKnots( surA[iq][jq], &Qw, &UQ, &VQ );

            if( jq LT nsv - 1 )
                N_SrfGetCPts( surA[iq][jq + 1], &i, &j, &NQw );

            /* Get knot multiplicity */

            i = iev;

            while( iev LT s AND VP[iev]EQ VP[iev + 1] )
                iev++;
            mlv = iev - i + 1;
            rv = q - mlv;

            /* Insert the knot */

            if( mlv LT q )
            {
                num = VP[iev] - VP[isv];

                for ( i = q; i > mlv; i-- )
                {
                    vals[i - mlv - 1] = num / (VP[isv + i] - VP[isv]);
                    omvs[i - mlv - 1] = 1.0 - vals[i - mlv - 1];
                }

                for ( i = 1; i <= rv; i++ )
                {
                    sv = mlv + i;
                    save = rv - i;

                    for ( j = q; j >= sv; j-- )
                    {
                        for ( k = 0; k <= p; k++ )
                        {
                            N_Combine2CPts( vals[j - sv], Qw[k][j], omvs[j - sv], Qw[k][j - 1], &Qw[k][j] );
                        }
                    }

                    if( iev LT s )
                    {
                        for ( k = 0; k <= p; k++ )
                        {
                            N_CopyCPt( Qw[k][q], &NQw[k][save] );
                        }
                    }
                }
            }

            /* Get knot vectors */

            for ( i = 0; i <= p; i++ )
            {
                UQ[i] = UP[isu];
                UQ[i + p + 1] = UP[ieu];
            }

            for ( j = 0; j <= q; j++ )
            {
                VQ[j] = VP[isv];
                VQ[j + q + 1] = VP[iev];
            }

            /* Patch completed - prepare for next Bezier patch */

            if( iev LT s )
            {
                for ( i = rv; i <= q; i++ )
                {
                    for ( k = 0; k <= p; k++ )
                        N_CopyCPt( Bw[k][iev - q + i], &NQw[k][i] );
                }
            }

            isv = iev;
            iev = iev + 1;
        }

        /* Bezier strip decomposed - prepare for next strip */

        if( ieu LT r )
        {
            for ( i = ru; i <= p; i++ )
            {
                for ( l = 0; l <= m; l++ )
                    N_CopyCPt( Pw[ieu - p + i][l], &NBw[i][l] );
            }
        }

        isu = ieu;
        ieu = ieu + 1;
        tmp = Bw;
        Bw = NBw;
        NBw = tmp;
    }

    *kk = nsu - 1;
    *ll = nsv - 1;
    *surQ = surA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfDecomposeToBez */

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine extracts an  iso-curve from a NURBS surface at
     a given  parameter  value either in  u- or in  v-direction. If the  
     output curve is initialized to NULL, memory to store curve control 
     points and knots is allocated. A typical calling example is: 

       NL_SURFACE    sur;
       NL_PARAMETER  t;
       NL_CURVE      cur;
       NL_STACKS     SC;
       ...
       (define sur, get t);
       ...
       N_CrvInitArrays(&cur);
       N_SrfExtractIsoCrv(&sur,t,NL_UDIR,&cur,&SC);

     If memory is  available, cur  is  not  initialized and the routine
     assumes that memory allocation has  been done. However, provided 
     that a non-null stack SC is provided, it  checks  
     for the proper  amount by looking  at the highest indexes in cur's 
     knot vector and polygon objects.  

   ACCESS:
   
     sur , input  ,  NURBS surface
     t   , input  ,  Parameter at which curve is to be extracted
     dir , input  ,  Flag:
                       NL_UDIR: Extract a u-curve (at a fixed v-value)
                       NL_VDIR: Extract a v-curve (at a fixed u-value)
     cur , output ,  Extracted curve
     SC  , input  ,  (optional) cur's stack or NULL (do not check cur)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfExtractIsoCrv( NL_SURFACE *sur, NL_PARAMETER t, NL_FLAG dir, NL_CURVE *cur, NL_STACKS *SC )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfExtractIsoCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, spu = 0, mlu = 0, spv = 0, mlv = 0, a, ne, me;

    NL_DEGREE p, q, pe;

    NL_REAL *U, *V, *UC;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, *Cw, Rw[NL_MAXDEG + 1];

    /* Use (max) fixed size arrays */

    NL_REAL *alf[NL_MAXDEG + 1];
    NL_REAL stack_alf[NL_MAXDEG + 1][NL_MAXDEG + 1];
    NL_REAL *oma[NL_MAXDEG + 1];
    NL_REAL stack_oma[NL_MAXDEG + 1][NL_MAXDEG + 1];

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    knu = sur->knu;
    knv = sur->knv;

    a = NL_MAX( p, q );

    for ( i = 0; i <= a; i++ )
    {
        alf[i] = &stack_alf[i][0];
        oma[i] = &stack_oma[i][0];
    }

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            ne = n;
            pe = p;
            me = r;
            break;

        case NL_VDIR:

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;

            ne = m;
            pe = q;
            me = s;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed..only for NLib...others (SMLib) will have a good cur */

    if( SC != NULL )
    {
        error = N_CrvSizeArrays( cur, ne, pe, me, rname, SC );

        if( error EQ NL_YES )
            NL_OUT;
    }

    Cw = cur->pol->Pw;
    UC = cur->knt->U;

    /* See if boundary curve is required */

    if( dir EQ NL_UDIR AND( t EQ V[q]OR t EQ V[s - q] ) )
    {
        if( t EQ V[q] )
        {
            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Pw[i][0], &Cw[i] );
        }
        else
        {
            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Pw[i][m], &Cw[i] );
        }

        for ( i = 0; i <= r; i++ )
            UC[i] = U[i];

        NL_OUT;
    }

    if( dir EQ NL_VDIR AND( t EQ U[p]OR t EQ U[r - p] ) )
    {
        if( t EQ U[p] )
        {
            for ( j = 0; j <= m; j++ )
                N_CopyCPt( Pw[0][j], &Cw[j] );
        }
        else
        {
            for ( j = 0; j <= m; j++ )
                N_CopyCPt( Pw[n][j], &Cw[j] );
        }

        for ( j = 0; j <= s; j++ )
            UC[j] = V[j];

        NL_OUT;
    }

    /* Extract the u-curve */

    if( dir EQ NL_UDIR )
    {
        if( mlv > q )
        {
            error = NL_YES;
            NL_OUT;
        }

        /* Save the v-alpha's */

        for ( i = 1; i <= q - mlv; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf[i][j] = (t - V[a + j]) / (V[spv + j + 1] - V[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column compute curve control point */

        a = spv - q;

        for ( k = 0; k <= n; k++ )
        {
            /* Load auxiliary control points */

            for ( i = 0; i <= q - mlv; i++ )
                N_CopyCPt( Pw[k][a + i], &Rw[i] );

            /* Now insert the knot */

            for ( i = 1; i <= q - mlv; i++ )
            {
                for ( j = 0; j <= q - i - mlv; j++ )
                {
                    N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                }
            }

            /* Load curve control point */

            N_CopyCPt( Rw[0], &Cw[k] );
        } /* End for each column */

        /* Load curve knot vector */

        for ( i = 0; i <= r; i++ )
            UC[i] = U[i];
    } /* End of NL_UDIR */

    /* Extract the v-curve */

    if( dir EQ NL_VDIR )
    {
        if( mlu > p )
        {
            error = NL_YES;
            NL_OUT;
        }

        /* Save the u-alpha's */

        for ( i = 1; i <= p - mlu; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf[i][j] = (t - U[a + j]) / (U[spu + j + 1] - U[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column compute curve control point */

        a = spu - p;

        for ( l = 0; l <= m; l++ )
        {
            /* Load auxiliary control points */

            for ( i = 0; i <= p - mlu; i++ )
                N_CopyCPt( Pw[a + i][l], &Rw[i] );

            /* Now insert the knot */

            for ( i = 1; i <= p - mlu; i++ )
            {
                for ( j = 0; j <= p - i - mlu; j++ )
                {
                    N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                }
            }

            /* Load curve control point */

            N_CopyCPt( Rw[0], &Cw[l] );
        } /* End for each column */

        /* Load curve knot vector */

        for ( j = 0; j <= s; j++ )
            UC[j] = V[j];
    } /* End of NL_VDIR */

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfExtractIsoCrv */


/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  refines a  NURBS  surface  with a given knot 
     vector in  either  u- or  v-direction. It is  assumed that the new 
     knot vector fits into the old  ones, i.e. T[0] < X[0] <=...<= X[r] 
     < T[k] holds where T[0],...,T[k] are the old knots in either the U
     or  the V knot  vector, and  X[0],...,X[r] are the new knots to be
     inserted. If the  output surface is initialized to NULL, memory to 
     store new control  points and  knots is  allocated. If  the output  
     surface is the same as the input surface, knot refinement  is done  
     in place and the original  surface is destroyed. A typical calling 
     example is:

       NL_SURFACE     surP, surQ;
       NL_KNOTVECTOR  knx;
       NL_STACKS      SP, SQ;
       ...
       (define surP and knx);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfInsertKnots(&surP,&knx,NL_UDIR,&surQ,&SP,&SQ);
       N_SrfInsertKnots(&surP,&knx,NL_VDIR,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects.


   ACCESS:
   
     surP , input  ,  NURBS surface
     knx  , input  ,  New knot vector to be inserted
     dir  , input  ,  Flag:
                        NL_UDIR: Refine in u-direction
                        NL_VDIR: Refine in v-direction
     surQ , output ,  Surface after knot refinement
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfInsertKnots( NL_SURFACE *surP, NL_KNOTVECTOR *knx, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfInsertKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, a, b, k, l, n, m, r, s, xr, t, row, col, ni, mi, ri, si;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, *X, alf, oma;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );
    N_KnotVectorGetKnots( knx, &xr, &X );

    /* Check parameters and compute highest indexes */

    if( xr LT 0 )
        NL_ERROR( NL_INP_ERR );

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, X[0], rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_KnotVectorIsEndParam( knu, X[xr], rname );

            if( error EQ NL_YES )
                NL_OUT;

            ni = n + xr + 1;
            mi = m;
            ri = r + xr + 1;
            si = s;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, X[0], rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_KnotVectorIsEndParam( knv, X[xr], rname );

            if( error EQ NL_YES )
                NL_OUT;

            ni = n;
            mi = m + xr + 1;
            ri = r;
            si = s + xr + 1;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        surA = *surP;

        error = N_AllocSrfArrays( surP, ni, mi, p, q, ri, si, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, ni, mi, p, q, ri, si, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Refine U knot vector */

    if( dir EQ NL_UDIR )
    {
        /* Find knot span indexes */

        error = N_BasisFindSpan( knu, p, X[0], NL_LEFT, &a );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knu, p, X[xr], NL_LEFT, &b );

        if( error EQ NL_YES )
            NL_OUT;
        b++;

        /* Initialize output knot vectors */

        for ( i = 0; i <= a; i++ )
            UQ[i] = UP[i];

        for ( i = b + p; i <= r; i++ )
            UQ[i + xr + 1] = UP[i];

        for ( j = 0; j <= s; j++ )
            VQ[j] = VP[j];

        /* Save unaltered control points */

        for ( col = 0; col <= m; col++ )
        {
            for ( j = 0; j <= a - p; j++ )
                N_CopyCPt( Pw[j][col], &Qw[j][col] );

            for ( j = b - 1; j <= n; j++ )
                N_CopyCPt( Pw[j][col], &Qw[j + xr + 1][col] );
        }

        /* Now refine the knot vector */

        i = b + p - 1;
        k = b + p + xr;

        for ( j = xr; j >= 0; j-- )
        {
            while( X[j]LE UP[i]AND i GT a )
            {
                for ( col = 0; col <= m; col++ )
                {
                    N_CopyCPt( Pw[i - p - 1][col], &Qw[k - p - 1][col] );
                }
                UQ[k] = UP[i];
                k--;
                i--;
            }

            for ( col = 0; col <= m; col++ )
            {
                N_CopyCPt( Qw[k - p][col], &Qw[k - p - 1][col] );
            }

            for ( l = 1; l <= p; l++ )
            {
                t = k - p + l;
                alf = UQ[k + l] - X[j];

                if( fabs( alf )LE 0.0 )
                {
                    for ( col = 0; col <= m; col++ )
                    {
                        N_CopyCPt( Qw[t][col], &Qw[t - 1][col] );
                    }
                }
                else
                {
                    alf = alf / (UQ[k + l] - UP[i - p + l]);
                    oma = 1.0 - alf;

                    for ( col = 0; col <= m; col++ )
                    {
                        N_Combine2CPts( alf, Qw[t - 1][col], oma, Qw[t][col], &Qw[t - 1][col] );
                    }
                }
            }
            UQ[k] = X[j];
            k--;
        }
    } /* End of NL_UDIR */

    /* Refine V knot vector */

    if( dir EQ NL_VDIR )
    {
        /* Find knot span indexes */

        error = N_BasisFindSpan( knv, q, X[0], NL_LEFT, &a );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, X[xr], NL_LEFT, &b );

        if( error EQ NL_YES )
            NL_OUT;
        b++;

        /* Initialize output knot vector */

        for ( j = 0; j <= a; j++ )
            VQ[j] = VP[j];

        for ( j = b + q; j <= s; j++ )
            VQ[j + xr + 1] = VP[j];

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        /* Save unaltered control points */

        for ( row = 0; row <= n; row++ )
        {
            for ( j = 0; j <= a - q; j++ )
                N_CopyCPt( Pw[row][j], &Qw[row][j] );

            for ( j = b - 1; j <= m; j++ )
                N_CopyCPt( Pw[row][j], &Qw[row][j + xr + 1] );
        }

        /* Now refine the knot vector */

        i = b + q - 1;
        k = b + q + xr;

        for ( j = xr; j >= 0; j-- )
        {
            while( X[j]LE VP[i]AND i GT a )
            {
                for ( row = 0; row <= n; row++ )
                {
                    N_CopyCPt( Pw[row][i - q - 1], &Qw[row][k - q - 1] );
                }
                VQ[k] = VP[i];
                k--;
                i--;
            }

            for ( row = 0; row <= n; row++ )
            {
                N_CopyCPt( Qw[row][k - q], &Qw[row][k - q - 1] );
            }

            for ( l = 1; l <= q; l++ )
            {
                t = k - q + l;
                alf = VQ[k + l] - X[j];

                if( fabs( alf )LE 0.0 )
                {
                    for ( row = 0; row <= n; row++ )
                    {
                        N_CopyCPt( Qw[row][t], &Qw[row][t - 1] );
                    }
                }
                else
                {
                    alf = alf / (VQ[k + l] - VP[i - q + l]);
                    oma = 1.0 - alf;

                    for ( row = 0; row <= n; row++ )
                    {
                        N_Combine2CPts( alf, Qw[row][t - 1], oma, Qw[row][t], &Qw[row][t - 1] );
                    }
                }
            }
            VQ[k] = X[j];
            k--;
        }
    } /* End of NL_VDIR */

    /* If refinement is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfInsertKnots */



/*******************************************************************//**


   DESCRIPTION:

     This tools routine extracts a surface patch  from a NURBS surface 
     given by the  parameters of its  corner points. The extraction is 
     done  via  knot  insertion, i.e.  the  knots  must  satisfy  knot 
     insertion requirements. If the  output surface is  initialized to 
     NULL, memory to store new control  points and knots is allocated. 
     If the output surface is  the same as  the input surface, surface
     extraction  is  done  in   place  and  the  original  surface  is 
     destroyed. A typical calling example is:

       NL_SURFACE    surP, surQ;
       NL_PARAMETER  ul, vl, ur, vr;
       NL_STACKS     SP, SQ;
       ...
       (define surP, get ul, ur, vl, vr);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfExtractPatch(&surP,ul,ur,vl,vr,&surQ,&SP,&SQ); 
       N_SrfExtractPatch(&surP,ul,ur,vl,vr,&surP,&SP,&SP); 

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects. 

     Additional Note: We first determine if the requested patch is within
     tolerance to existing knots. If so, we snap to the requested patch
     to the existing kot value in order to avoid knots within tolerance.
     This tolerance is hard coded at 1.0e-08.

   ACCESS:
   
     surP  , input  ,  NURBS surface
     ul,ur , input  ,  Parameters in u-direction defining the patch
     vl,vr , input  ,  Parameters in v-direction defining the patch
     surQ  , output ,  Surface after extraction
     SP    , input  ,  surP's stack
     SQ    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfExtractPatch( NL_SURFACE *surP, NL_PARAMETER ul, NL_PARAMETER ur, NL_PARAMETER vl, NL_PARAMETER vr, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfExtractPatch");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, ll, lk, lr, n, m, r, s, sul, mul, sur, mur, svl, mvl, svr, mvr, is, ie, js, je, row, col;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, alf, oma, left;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, ** Aw;

    NL_SURFACE surA;

    NL_REAL dTol = 1.0e-08;  /* typically hard coded in NLIB */

    NL_BOOLEAN bSnapped = FALSE;

    NL_RECTANGLE sInRect;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* Remember requested box */

    sInRect.ul = ul;
    sInRect.ur = ur;
    sInRect.vb = vl;
    sInRect.vt = vr;
    
    /* Snap left u to existing u knots if within tolerance */
    for( i = 0; i < r; i++ ) 
    {
        if( ul > UP[i] - dTol && ul < UP[i] + dTol ) {
            ul = UP[i];
            bSnapped = TRUE;
            break;
        }
    }

    /* Snap right u to existing u knots if within tolerance */
    for( i = 0; i < r; i++ ) 
    {
        if( ur > UP[i] - dTol && ur < UP[i] + dTol ) {
            ur = UP[i];
            bSnapped = TRUE;
            break;
        }
    }

    /* Snap left v to existing v knots if within tolerance */
    for( i = 0; i < s; i++ ) 
    {
        if( vl > VP[i] - dTol && vl < VP[i] + dTol ) {
            vl = VP[i];
            bSnapped = TRUE;
            break;
        }
    }

    /* Snap right v to existing v knots if within tolerance */
    for( i = 0; i < s; i++ ) 
    {
        if( vr > VP[i] - dTol && vr < VP[i] + dTol ) {
            vr = VP[i];
            bSnapped = TRUE;
            break;
        }
    }


    /* Check for degenerate parameters */

    if( ur LE ul )
        NL_ERROR( NL_INP_ERR );

    if( vr LE vl )
        NL_ERROR( NL_INP_ERR );


    /* Find knot spans and set new indexes */

    error = N_BasisFindSpanAndMult( knu, p, ul, NL_LEFT, &sul, &mul );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpanAndMult( knu, p, ur, NL_LEFT, &sur, &mur );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpanAndMult( knv, q, vl, NL_LEFT, &svl, &mvl );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpanAndMult( knv, q, vr, NL_LEFT, &svr, &mvr );

    if( error EQ NL_YES )
        NL_OUT;

    is = sul - p;

    if( ur EQ UP[r - p] )
    {
        sur = r;
        mur = p + 1;
    }
    ie = sur - mur;

    js = svl - q;

    if( vr EQ VP[s - q] )
    {
        svr = s;
        mvr = q + 1;
    }
    je = svr - mvr;

    n = ie - is;
    r = sur - sul - mur + 2 * p + 1;

    if( n < 0 )
        NL_ERROR( NL_INP_ERR );

    /* Get auxiliary control points */

    Aw = N_AllocCPt2dArray( n, m, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    /***********************************************/
    /* Get surface strip, i.e. for each column do: */
    /*   (1) get initial conrol points             */
    /*   (2) insert left knot                      */
    /*   (3) insert right knot                     */
    /***********************************************/

    for ( col = 0; col <= m; col++ )
    {
        /* Get initial control points */

        for ( i = is; i <= ie; i++ )
            N_CopyCPt( Pw[i][col], &Aw[i - is][col] );

        /* Insert the left knot */

        ll = sul - p;

        for ( i = 1; i <= p - mul; i++ )
        {
            for ( j = 0; j <= p - i - mul; j++ )
            {
                left = UP[ll + i + j];
                alf = (ul - left) / (UP[sul + j + 1] - left);
                oma = 1.0 - alf;
                N_Combine2CPts( alf, Aw[j + 1][col], oma, Aw[j][col], &Aw[j][col] );
            }
        }

        /* Insert the right knot */

        lr = sur - p;
        lk = n - p + mur;

        for ( i = 1; i <= p - mur; i++ )
        {
            for ( j = p - i - mur; j >= 0; j-- )
            {
                k = lk + i + j;
                left = UP[lr + i + j];

                if( left LT ul )
                    left = ul;

                alf = (ur - left) / (UP[sur + j + 1] - left);
                oma = 1.0 - alf;
                N_Combine2CPts( alf, Aw[k][col], oma, Aw[k - 1][col], &Aw[k][col] );
            }
        }
    }

    /* See if memory is needed */

    m = je - js;
    s = svr - svl - mvr + 2 * q + 1;

    if( m < 0 )
        NL_ERROR( NL_INP_ERR );

    if( surP EQ surQ )
    {
        surA = *surP;

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

    /************************************************/
    /* Extract surface patch, i.e. for each row do: */
    /*   (1) get initial control points             */
    /*   (2) insert left knot                       */
    /*   (3) insert right knot                      */
    /************************************************/

    for ( row = 0; row <= n; row++ )
    {
        /* Get initial control points */

        for ( j = js; j <= je; j++ )
            N_CopyCPt( Aw[row][j], &Qw[row][j - js] );

        /* Insert the left knot */

        ll = svl - q;

        for ( i = 1; i <= q - mvl; i++ )
        {
            for ( j = 0; j <= q - i - mvl; j++ )
            {
                left = VP[ll + i + j];
                alf = (vl - left) / (VP[svl + j + 1] - left);
                oma = 1.0 - alf;
                N_Combine2CPts( alf, Qw[row][j + 1], oma, Qw[row][j], &Qw[row][j] );
            }
        }

        /* Insert the right knot */

        lr = svr - q;
        lk = m - q + mvr;

        for ( i = 1; i <= q - mvr; i++ )
        {
            for ( j = q - i - mvr; j >= 0; j-- )
            {
                k = lk + i + j;
                left = VP[lr + i + j];

                if( left LT vl )
                    left = vl;

                alf = (vr - left) / (VP[svr + j + 1] - left);
                oma = 1.0 - alf;
                N_Combine2CPts( alf, Qw[row][k], oma, Qw[row][k - 1], &Qw[row][k] );
            }
        }
    }

    /* Load knot vectors */

    k = -1;

    for ( i = 0; i <= p; i++ )
        UQ[++k] = ul;

    for ( i = sul + 1; i <= sur - mur; i++ )
        UQ[++k] = UP[i];

    for ( i = 0; i <= p; i++ )
        UQ[++k] = ur;

    k = -1;

    for ( j = 0; j <= q; j++ )
        VQ[++k] = vl;

    for ( j = svl + 1; j <= svr - mvr; j++ )
        VQ[++k] = VP[j];

    for ( j = 0; j <= q; j++ )
        VQ[++k] = vr;

    /* If insertion is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* When final interval was snapped away from what was requested */ 
    /* scale the output back to the requested interval */

    if( bSnapped EQ TRUE )
    {
        N_SrfReparamToInterval( surQ, sInRect, NL_UVDIR ) ;     
    }


    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfExtractPatch */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine conditionally removes one knot multiple  times 
     from a  NURBS surface  either  in u- or in v-direction.  The knot  
     must be an interior knot. Knots are removed only if the resulting
     change in surface shape is less than the input tolerance.  If the 
     output  surface  is  initialized to  NULL, memory  to  store  new 
     control points and knots is allocated. 

     A typical calling example is:

       NL_SURFACE    surP, surQ;
       NL_PARAMETER  t;
       NL_REAL       tol;
       NL_INDEX      nt, rt;
       NL_STACKS     SQ;
       ...
       (define surP, get t and tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveKnotConditional(&surP,t,nt,tol,NL_UDIR,&rt,&surQ,&SQ);
       N_SrfRemoveKnotConditional(&surP,t,nt,tol,NL_VDIR,&rt,&surP,&SQ);

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects. If surP is the same as surQ, the
     removal is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     t,nt , input  ,  Knot "t" to be  removed "nt" times. If  nt > mlt,
                      the multiplicity of the knot, nt=mlt  is assumed.
     tol  , input  ,  Tolerance to check removability
     dir  , input  ,  Flag:
                        NL_UDIR: Remove in u-direction
                        NL_VDIR: Remove in v-direction
     rt   , output ,  Number of knots removed
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveKnotConditional
 (NL_SURFACE   *surP,  /* in : target surface */
  NL_PARAMETER  t,     /* in : knot to be removed */
  NL_INDEX      nt,    /* in : number of times to be removed */
  NL_REAL       tol,   /* in : tol to check removability */
  NL_FLAG       dir,   /* in : NL_UDIR: Remove in u-direction */
                       /*      NL_VDIR: Remove in v-direction */
  NL_INDEX     *rt,    /* out: number of knots actually removed */
  NL_SURFACE   *surQ,  /* out: Surface after knot removal */
  NL_STACKS    *SQ )   /* in : surQ's stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveKnotConditional");

    NL_FLAG rmf, error = NL_NO;

    NL_INDEX i, j, k, l, row, col, ii, jj, first, last, off, n, m, r, s, a, spn, mlt, fout;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb, lam = 0.0;
    NL_REAL oml = 0, del, omd, wmin, wmax, tmp, dw, maxl, maxdw;
    NL_REAL maxr, max, lto, wi, wj, pmax;

    NL_KNOTVECTOR *kup, *kvp, *kuq, *kvq;

    NL_CPOINT ** Pw, ** Qw, *Rw, A;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &kup, &kvp );

    /* Adjust removal tolerance in case of rational surfaces */

    /* GWC:CHANGE do tolerance checks in 3Space */
    if( N_IsSrfRat( surP ) )
    {
        N_SrfMinMaxWeightPosVectors( surP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
    }

    /* Check parameters */

    /* find spn and mlt for t param */
    switch( dir )
    {
        case NL_UDIR:

            /* exit - t is an endKnot value */
            error = N_KnotVectorIsEndParam( kup, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            /* find span and multiplicity for t param value */
            error = N_BasisFindSpanAndMult( kup, p, t, NL_LEFT, &spn, &mlt );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        case NL_VDIR:

            /* exit - t is an endKnot value */
            error = N_KnotVectorIsEndParam( kvp, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            /* find span and multiplicity for t param value */
            error = N_BasisFindSpanAndMult( kvp, q, t, NL_LEFT, &spn, &mlt );

            if( error EQ NL_YES )
                NL_OUT;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* limit removal count to multiplicity of knot */
    if( nt GT mlt )
        nt = mlt;

    /* See if memory is needed */

    /* get pointers to output surface ControlPoint and knot arrays */
    if( surP EQ surQ )
    {
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
        N_SrfGetKnotVectors( surP, &kuq, &kvq );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
        N_SrfGetKnotVectors( surQ, &kuq, &kvq );
    }

    /* Allocate local memory */

    a = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * a, &SL );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * a, &SL );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( 2 * a, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Initialize */

    if( surP NEQ surQ )
    {
        for ( row = 0; row <= n; row++ )
        {
            for ( col = 0; col <= m; col++ )
                N_CopyCPt( Pw[row][col], &Qw[row][col] );
        }

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= s; j++ )
            VQ[j] = VP[j];
    }

    *rt = 0;

    /* Remove in u-direction */

    if( dir EQ NL_UDIR )
    {
        fout = (2 * spn - mlt - p) / 2;
        rmf = NL_TRUE;
        first = spn - p;
        last = spn - mlt;

        /* Try to remove the knot "nt" times */

        lto = sto * fabs( UP[r] - UP[0] );

        for ( k = 0; k < nt; k++ )
        {
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT k )
            {
                alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (t - UQ[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (UQ[j + p - k + 1] - UQ[j - k]) / (UQ[j + p - k + 1] - t);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            del = (t - UQ[i]) / (UQ[i + p + 1] - UQ[i]);
            omd = 1.0 - del;

            if( (j - i)LT k )
            {
                if( i - first - 1 GE 0 AND j - first + 1 GE 0 )
                {
                    lam = alf[i - first - 1] / (alf[i - first - 1] + bet[j - first + 1]);
                    oml = 1.0 - lam;

                    error = N_BasisFindGlobalMax( kuq, i - 1, p, lto, &maxl, &tmp );

                    if( error EQ NL_YES )
                        maxl = 1.0;

                    error = N_BasisFindGlobalMax( kuq, i, p, lto, &maxr, &tmp );

                    if( error EQ NL_YES )
                        maxr = 1.0;

                    max = NL_MAX( lam * alf[i - first - 1] * maxl, oml * omd * maxr );
                }
                else
                {
                    lam = oml = 0.5;
                    max = 1.0;
                }
            }
            else
            {
                error = N_BasisFindGlobalMax( kuq, i, p, lto, &max, &tmp );

                if( error EQ NL_YES )
                    max = 1.0;
            }

            /* Try to remove one knot along each row */

            for ( col = 0; col <= m; col++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[off][col], &Rw[0] );
                N_CopyCPt( Qw[last + 1][col], &Rw[last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT k )
                {
                    N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[ii - 1], &Rw[ii] );
                    N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Check if knot is removable */

                if( (j - i)LT k )
                {
                    /* gwc:change N_DistCptCptHomo to N_DistCptCpt calls*/
                    N_DistCptCptHomo( Rw[ii - 1], Rw[jj + 1], &dw );

                    maxdw = max * dw;

                    if( maxdw GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }

                    N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                }
                else
                {
                    N_Combine2CPts( del, Rw[jj + 1], omd, Rw[ii - 1], &A );
                    N_DistCptCptHomo( Qw[i][col], A, &dw );

                    maxdw = max * dw;

                    if( maxdw GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }
                }

                /* Check for disallowed weights */

                if( N_IsSrfRat( surP ) )
                {
                    i = first;
                    j = last;
                    wmin = NL_BIGD;
                    wmax = NL_SMAD;

                    while( (j - i)GT k )
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
                        rmf = NL_FALSE;
                        break;
                    }
                }

                /* Save new control points */

                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    N_CopyCPt( Rw[i - off], &Qw[i][col] );
                    N_CopyCPt( Rw[j - off], &Qw[j][col] );
                    i++;
                    j--;
                }
            } /* End for each row */

            /* See if knot was removable for each row */

            if( rmf EQ NL_FALSE )
                break;

            /* Successful removal -> shift down knots */

            for ( l = spn - k; l <= r - k - 1; l++ )
                UQ[l] = UQ[l + 1];

            first--;
            last++;
        } /* End of for "k<nt" loop */

        /* If no knot was removed --> out */

        if( k EQ 0 )
            NL_OUT;

        /* Shift down control points */

        j = fout;
        i = j;

        for ( l = 1; l < k; l++ )
        {
            if( l % 2 )
                i++;
            else
                j--;
        }

        for ( col = 0; col <= m; col++ )
        {
            a = j;

            for ( l = i + 1; l <= n; l++ )
            {
                N_CopyCPt( Qw[l][col], &Qw[a][col] );
                a++;
            }
        }

        /* Complete output */

        *rt = k;

        N_SrfSetSizeIndices( surQ, n - k, m, p, q, r - k, s );

        NL_OUT;
    } /* End of NL_UDIR */

    /* Remove in v-direction */

    if( dir EQ NL_VDIR )
    {
        fout = (2 * spn - mlt - q) / 2;
        rmf = NL_TRUE;
        first = spn - q;
        last = spn - mlt;

        /* Try to remove the knot "nt" times */

        lto = sto * fabs( VP[s] - VP[0] );

        for ( k = 0; k < nt; k++ )
        {
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT k )
            {
                alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (t - VQ[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (VQ[j + q - k + 1] - VQ[j - k]) / (VQ[j + q - k + 1] - t);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            del = (t - VQ[i]) / (VQ[i + q + 1] - VQ[i]);
            omd = 1.0 - del;

            if( (j - i)LT k )
            {
                if( i - first - 1 GE 0 AND j - first + 1 GE 0 )
                {
                    lam = alf[i - first - 1] / (alf[i - first - 1] + bet[j - first + 1]);
                    oml = 1.0 - lam;

                    error = N_BasisFindGlobalMax( kvq, i - 1, q, lto, &maxl, &tmp );

                    if( error EQ NL_YES )
                        maxl = 1.0;

                    error = N_BasisFindGlobalMax( kvq, i, q, lto, &maxr, &tmp );

                    if( error EQ NL_YES )
                        maxr = 1.0;

                    max = NL_MAX( lam * alf[i - first - 1] * maxl, oml * omd * maxr );
                }
                else
                {
                    lam = oml = 0.5;
                    max = 1.0;
                }
            }
            else
            {
                error = N_BasisFindGlobalMax( kvq, i, q, lto, &max, &tmp );

                if( error EQ NL_YES )
                    max = 1.0;
            }

            /* Try to remove one knot along each column */

            for ( row = 0; row <= n; row++ )
            {

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[row][off], &Rw[0] );
                N_CopyCPt( Qw[row][last + 1], &Rw[last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT k )
                {
                    N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[ii - 1], &Rw[ii] );
                    N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Check if knot is removable */

                if( (j - i)LT k )
                {
                    N_DistCptCptHomo( Rw[ii - 1], Rw[jj + 1], &dw );

                    maxdw = max * dw;

                    if( maxdw GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }

                    N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                }
                else
                {
                    N_Combine2CPts( del, Rw[jj + 1], omd, Rw[ii - 1], &A );
                    N_DistCptCptHomo( Qw[row][i], A, &dw );

                    maxdw = max * dw;

                    if( maxdw GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }
                }

                /* Check for disallowed weights */

                if( N_IsSrfRat( surP ) )
                {
                    i = first;
                    j = last;
                    wmin = NL_BIGD;
                    wmax = NL_SMAD;

                    while( (j - i)GT k )
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
                        rmf = NL_FALSE;
                        break;
                    }
                }

                /* Save new control points */

                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    N_CopyCPt( Rw[i - off], &Qw[row][i] );
                    N_CopyCPt( Rw[j - off], &Qw[row][j] );
                    i++;
                    j--;
                }
            } /* End for each column */

            /* See if knot was removable for each column */

            if( rmf EQ NL_FALSE )
                break;

            /* Successful removal -> shift down knots */

            for ( l = spn - k; l <= s - k - 1; l++ )
                VQ[l] = VQ[l + 1];

            first--;
            last++;
        } /* End of for "k<nt" loop */

        /* If no knot was removed --> out */

        if( k EQ 0 )
            NL_OUT;

        /* Shift down control points */

        j = fout;
        i = j;

        for ( l = 1; l < k; l++ )
        {
            if( l % 2 )
                i++;
            else
                j--;
        }

        for ( row = 0; row <= n; row++ )
        {
            a = j;

            for ( l = i + 1; l <= m; l++ )
            {
                N_CopyCPt( Qw[row][l], &Qw[row][a] );
                a++;
            }
        }

        /* Complete output */

        *rt = k;

        N_SrfSetSizeIndices( surQ, n, m - k, p, q, r, s - k );

        NL_OUT;
    } /* End of NL_VDIR */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveKnotConditional */


/*******************************************************************//**


   DESCRIPTION:

     This tools routine  updates surface  removal error  bound for one
     removal step in either u- or v-direction. That is, given the knot
     index "r",  multiplicity "s", the  knot T[r], T is either U or V, 
     is removed one time  in either direction and the maximum error is 
     updated. It is  assumed  that (1) T[r] is  an  interior knot, (2) 
     T[r] != T[r+1], and (3) the multiplicity of the  knot is "s>0". A 
     typical calling example is:


       NL_SURFACE  sur;
       NL_INDEX    r, s, f, l;
       NL_REAL     mr;
       ...
       (define sur, get r, s, f and l);
       ...
       N_SrfRemoveOneKnot(&sur,r,s,f,l,NL_UDIR,&mr);

     THE ROUTINE DOES NOT CHECK FOR THE PROPER NL_INDEX AND  MULTIPLICITY
     OF THE  KNOT. IT ASSUMES  THAT THEY ARE  CORRECT. IF  mr IS TO BE
     INITIALIZED LOCALLY, mr=NL_BIGD IS CHECKED. 


   ACCESS:
   
     sur , input  ,  NURBS surface
     r,s , input  ,  Index  and  multiplicity  of  knot to be removed. 
                     T[r] != T[r+1] must hold (T is either U or V)
     f,l , input  ,  First and last  row/column  to be used  to update
                     the error
     dir , input  ,  Flag:
                       NL_UDIR: Remove in u-direction
                       NL_VDIR: Remove in v-direction
     mr  , output ,  Maximum  error (if  mr = NL_BIGD, it is  initialized
                     locally)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveOneKnot( NL_SURFACE *sur, NL_INDEX r, NL_INDEX s, NL_INDEX f, NL_INDEX l, NL_FLAG dir, NL_REAL *mr )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveOneKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, row, col, ii, jj, first, last, off, n, m;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *alf, *oma, *bet, *omb, del, omd, dw;

    NL_CPOINT ** Pw, *Rw, A;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &i, &j, &U, &V );

    /* Allocate local memory */

    i = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * i, &S );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * i, &S );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * i, &S );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * i, &S );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( 2 * i, &S );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Remove knot in the requested direction */

    if( *mr EQ NL_BIGD )
        *mr = -1.0;

    switch( dir )
    {
        case NL_UDIR: /* Remove in u-direction */

            first = r - p;
            last = r - s;
            off = first - 1;

			/* to avoid possible crash we make sure future index [last + 1 - off] must be positive */
			if( off > last + 1 )
				NL_QUIT;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                alf[i - first] = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            del = (U[r] - U[i]) / (U[i + p + 1] - U[i]);
            omd = 1.0 - del;

            /* Update maximum error for the requested rows */

            for ( col = f; col <= l; col++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Pw[off][col], &Rw[0] );
				N_CopyCPt( Pw[last + 1][col], &Rw[last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT 0 )
                {
                    N_Combine2CPts( alf[i - first], Pw[i][col], oma[i - first], Rw[ii - 1], &Rw[ii] );
                    N_Combine2CPts( bet[j - first], Pw[j][col], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Compute the error */

                if( (j - i)LT 0 )
                {
                    N_DistCptCptHomo( Rw[ii - 1], Rw[jj + 1], &dw );
                }
                else
                {
                    N_Combine2CPts( del, Rw[jj + 1], omd, Rw[ii - 1], &A );
                    N_DistCptCptHomo( Pw[i][col], A, &dw );
                }

                if( dw GT *mr )
                    *mr = dw;
            }
            break;

        case NL_VDIR: /* Remove in v-direction */

            first = r - q;
            last = r - s;
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                alf[i - first] = (V[i + q + 1] - V[i]) / (V[r] - V[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (V[j + q + 1] - V[j]) / (V[j + q + 1] - V[r]);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            del = (V[r] - V[i]) / (V[i + q + 1] - V[i]);
            omd = 1.0 - del;

            /* Update maximum error for the requested rows */

            for ( row = f; row <= l; row++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Pw[row][off], &Rw[0] );
                N_CopyCPt( Pw[row][last + 1], &Rw[last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT 0 )
                {
                    N_Combine2CPts( alf[i - first], Pw[row][i], oma[i - first], Rw[ii - 1], &Rw[ii] );
                    N_Combine2CPts( bet[j - first], Pw[row][j], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Compute the error */

                if( (j - i)LT 0 )
                {
                    N_DistCptCptHomo( Rw[ii - 1], Rw[jj + 1], &dw );
                }
                else
                {
                    N_Combine2CPts( del, Rw[jj + 1], omd, Rw[ii - 1], &A );
                    N_DistCptCptHomo( Pw[row][i], A, &dw );
                }

                if( dw GT *mr )
                    *mr = dw;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfRemoveOneKnot */


/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  removes  all removable  knots  from a NURBS
     surface. If the output surface is initialized to NULL, memory  to  
     store  new  control  points  and  knots  is  allocated. A typical 
     calling example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     tol;
       NL_STACKS   SQ;
       ...
       (define surP, get tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveAllKnots(&surP,tol,NL_UDIR ,&surQ,&SQ);
       N_SrfRemoveAllKnots(&surP,tol,NL_UVDIR,&surP,&SQ);

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects. If surP is the same as surQ, the
     removal is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     tol  , input  ,  Tolerance to check removability
     dir  , input  ,  Flag:
                        NL_UDIR : remove all removable u-knots
                        NL_VDIR : remove all removable v-knots
                        NL_UVDIR: remove all removable u- and v-knots
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveAllKnots( NL_SURFACE *surP, NL_REAL tol, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveAllKnots");

    NL_FLAG krm, rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sru = NULL, *srv = NULL, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru = 0, su = 0, rv = 0, sv = 0, ns, ms;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb, *minl, *maxl, *minr, *maxr, *max, *bru = NULL, *brv = NULL, lam = 0.0, oml = 0, wmin, wmax, pmax, tmp, al, be, ob, bu = 0.0, bv = 0, stu, stv, wi, wj;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, ** Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    ns = n;
    ms = m;

    /* Adjust removal tolerance in case of rational surfaces */

    if( N_IsSrfRat( surP ) )
    {
        N_SrfMinMaxWeightPosVectors( surP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    stu = sto * fabs( UP[r] - UP[0] );
    stv = sto * fabs( VP[s] - VP[0] );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsKnotVectorAndKnots( surP, &Qw, &knu, &knv, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsKnotVectorAndKnots( surQ, &Qw, &knu, &knv, &UQ, &VQ );
    }

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

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        bru = N_AllocReal1dArray( r, &SL );

        if( bru EQ NULL )
            NL_QUIT;

        sru = N_AllocInt1dArray( r, &SL );

        if( sru EQ NULL )
            NL_QUIT;
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        brv = N_AllocReal1dArray( s, &SL );

        if( brv EQ NULL )
            NL_QUIT;

        srv = N_AllocInt1dArray( s, &SL );

        if( srv EQ NULL )
            NL_QUIT;
    }

    /* Initialize */

    if( surP NEQ surQ )
    {
        error = N_SrfCopy( surP, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= r; i++ )
    {
        for ( j = 0; j <= s; j++ )
            er[i][j] = 0.0;
    }

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        for ( i = 0; i <= r; i++ )
        {
            bru[i] = NL_BIGD;
            sru[i] = 0;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        for ( j = 0; j <= s; j++ )
        {
            brv[j] = NL_BIGD;
            srv[j] = 0;
        }
    }

    /* Compute the maximum of knot removal errors for each distinct knot */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        ru = p + 1;

        while( ru LE n )
        {
            i = ru;

            while( ru LE n AND UQ[ru]EQ UQ[ru + 1] )
                ru++;
            sru[ru] = ru - i + 1;

            error = N_SrfRemoveOneKnot( surP, ru, sru[ru], 0, m, NL_UDIR, &bru[ru] );

            if( error EQ NL_YES )
                NL_OUT;

            ru++;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        rv = q + 1;

        while( rv LE m )
        {
            i = rv;

            while( rv LE m AND VQ[rv]EQ VQ[rv + 1] )
                rv++;
            srv[rv] = rv - i + 1;

            error = N_SrfRemoveOneKnot( surP, rv, srv[rv], 0, n, NL_VDIR, &brv[rv] );

            if( error EQ NL_YES )
                NL_OUT;

            rv++;
        }
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
        {
            bu = bru[p + 1];
            su = sru[p + 1];
            ru = p + 1;

            for ( i = p + 2; i <= r - p - 1; i++ )
            {
                if( bru[i]LT bu )
                {
                    bu = bru[i];
                    su = sru[i];
                    ru = i;
                }
            }
        }

        if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
        {
            bv = brv[q + 1];
            sv = srv[q + 1];
            rv = q + 1;

            for ( j = q + 2; j <= s - q - 1; j++ )
            {
                if( brv[j]LT bv )
                {
                    bv = brv[j];
                    sv = srv[j];
                    rv = j;
                }
            }
        }

        /* If no more removable knot -> finished */

        if( dir EQ NL_UDIR )
        {
            if( bu EQ NL_BIGD OR bu EQ NOREM )
                break;
        }
        else if( dir EQ NL_VDIR )
        {
            if( bv EQ NL_BIGD OR bv EQ NOREM )
                break;
        }
        else if( dir EQ NL_UVDIR )
        {
            if( (bu EQ NL_BIGD OR bu EQ NOREM)AND( bv EQ NL_BIGD OR bv EQ NOREM ) )
                break;
        }

        if( dir EQ NL_UVDIR )
        {
            if( bu LT bv )
                krm = NL_UDIR;
            else
                krm = NL_VDIR;
        }
        else
        {
            krm = dir;
        }

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR: /* Remove in the u-direction */

                rmf = NL_TRUE;

                if( (p + su) % 2 )
                {
                    /* Compute maximums of basis functions over each span */

                    k = (p + su + 1) / 2;
                    l = ru - k + p + 1;
                    al = (UQ[ru] - UQ[ru - k]) / (UQ[ru - k + p + 1] - UQ[ru - k]);
                    be = (UQ[ru] - UQ[ru - k + 1]) / (UQ[ru - k + p + 2] - UQ[ru - k + 1]);
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
                    /* Compute maximum of basis function */

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
                    if( UQ[i]NEQ UQ[i + 1] )
                    {
                        tmp = max[i - ru + k] * bu;

                        for ( j = q; j <= s - q - 1; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
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
                            if( UQ[i]NEQ UQ[i + 1]AND VQ[j]NEQ VQ[j + 1] )
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
                        alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (UQ[ru] - UQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[ru]);
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

                        N_CopyCPt( Qw[off][col], &Rw[col][0] );
                        N_CopyCPt( Qw[last + 1][col], &Rw[col][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[col][ii - 1], &Rw[col][ii] );
                            N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[col][jj + 1], &Rw[col][jj] );
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
                            N_Combine2CPts( lam, Rw[col][jj + 1], oml, Rw[col][ii - 1], &Rw[col][jj + 1] );
                        }
                    } /* End for each row */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        bru[ru] = NOREM;
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
                                N_CopyCPt( Rw[col][i - off], &Qw[i][col] );
                                N_CopyCPt( Rw[col][j - off], &Qw[j][col] );
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
                            if( VQ[j]NEQ VQ[j + 1] )
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
                        UQ[i - 1] = UQ[i];

                        for ( j = q; j <= m; j++ )
                            er[i - 1][j] = er[i][j];
                    }

                    for ( col = 0; col <= m; col++ )
                    {
                        for ( i = fout + 1; i <= n; i++ )
                        {
                            N_CopyCPt( Qw[i][col], &Qw[i - 1][col] );
                        }
                    }

                    n--;
                    r--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_UDIR )
                    {
                        if( n EQ p )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( ru - p, p + 1 );
                    l = NL_MIN( n, ru + p - su );

                    for ( i = k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1]AND bru[i]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, i, sru[i], 0, m, NL_UDIR, &bru[i] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( j = q + 1; j <= s - q - 1; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1]AND brv[j]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, j, srv[j], first, last, NL_VDIR, &brv[j] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    bru[ru] = NOREM;
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */

                rmf = NL_TRUE;

                if( (q + sv) % 2 )
                {
                    /* Compute maximums of basis functions over each span */

                    k = (q + sv + 1) / 2;
                    l = rv - k + q + 1;
                    al = (VQ[rv] - VQ[rv - k]) / (VQ[rv - k + q + 1] - VQ[rv - k]);
                    be = (VQ[rv] - VQ[rv - k + 1]) / (VQ[rv - k + q + 2] - VQ[rv - k + 1]);
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
                    /* Compute maximum of basis function */

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
                    if( VQ[j]NEQ VQ[j + 1] )
                    {
                        tmp = max[j - rv + k] * bv;

                        for ( i = p; i <= r - p - 1; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
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
                            if( VQ[j]NEQ VQ[j + 1]AND UQ[i]NEQ UQ[i + 1] )
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
                        alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (VQ[rv] - VQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (VQ[j + q + 1] - VQ[j]) / (VQ[j + q + 1] - VQ[rv]);
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

                        N_CopyCPt( Qw[row][off], &Rw[row][0] );
                        N_CopyCPt( Qw[row][last + 1], &Rw[row][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[row][ii - 1], &Rw[row][ii] );
                            N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[row][jj + 1], &Rw[row][jj] );
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
                            N_Combine2CPts( lam, Rw[row][jj + 1], oml, Rw[row][ii - 1], &Rw[row][jj + 1] );
                        }
                    } /* End for each column */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        brv[rv] = NOREM;
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
                                N_CopyCPt( Rw[row][i - off], &Qw[row][i] );
                                N_CopyCPt( Rw[row][j - off], &Qw[row][j] );
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
                            if( UQ[i]NEQ UQ[i + 1] )
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
                        VQ[j - 1] = VQ[j];

                        for ( i = p; i <= n; i++ )
                            er[i][j - 1] = er[i][j];
                    }

                    for ( row = 0; row <= n; row++ )
                    {
                        for ( j = fout + 1; j <= m; j++ )
                        {
                            N_CopyCPt( Qw[row][j], &Qw[row][j - 1] );
                        }
                    }

                    m--;
                    s--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_VDIR )
                    {
                        if( m EQ q )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( rv - q, q + 1 );
                    l = NL_MIN( m, rv + q - sv );

                    for ( j = k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1]AND brv[j]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, j, srv[j], 0, n, NL_VDIR, &brv[j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( i = p + 1; i <= r - p - 1; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1]AND bru[i]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, i, sru[i], first, last, NL_UDIR, &bru[i] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    brv[rv] = NOREM;
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* Compact surface */

    if( n LT ns OR m LT ms )
    {
        error = N_SrfCompress( surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveAllKnots */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools routine  removes  all  removable  knots  from a NURBS 
     surface. It reparametrizes the surface with respect to arc length 
     to be  able to remove as  many knots as possible. This routine is 
     suitable for  NURBS surfaces stiched together from Bezier  pieces 
     with bad parametrization. If the output surface is initialized to  
     NULL, memory to store new control points and knots is  allocated. 
     If the output  surface is  the  same as  the input  surface, knot 
     removal  is in place  and the  original  surface is  destroyed. A 
     typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     tol;
       NL_STACKS   SQ;
       ...
       (define surP, get tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveAllKnotsArcLen(&surP,tol,NL_UDIR ,&surQ,&SQ);
       N_SrfRemoveAllKnotsArcLen(&surP,tol,NL_UVDIR,&surP,&SQ);

     If memory is  available, surQ is not initialized and the routine
     assumes that memory allocation has been done. However, it checks  
     for  the  proper  amount by  looking  at the  highest indexes in 
     surQ's knot vector  and  control net objects. THE  ROUTINE  USES  
     0.1% NL_RELATIVE TOLERANCE TO APPROXIMATE ARC LENGTH.


   ACCESS:
   
     surP , input  ,  NURBS surface
     tol  , input  ,  Tolerance 
     dir  , input  ,  Flag:
                        NL_UDIR : clean in u-direction
                        NL_VDIR : clean in v-direction
                        NL_UVDIR: clean in both directions
     surQ , output ,  Surface after cleaning
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveAllKnotsArcLen( NL_SURFACE *surP, NL_REAL tol, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveAllKnotsArcLen");

    NL_FLAG error = NL_NO;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check flag */

    switch( dir )
    {
        case NL_UDIR:
            break;

        case NL_VDIR:
            break;

        case NL_UVDIR:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Reparametrize and clean */

    if( surP EQ surQ )
    {
        error = N_SrfReparamArcLength( surP, eps, surP, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfRemoveAllKnots( surP, tol, dir, surP, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_SrfReparamArcLength( surP, eps, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfRemoveAllKnots( surQ, tol, dir, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveAllKnotsArcLen */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine elevates the degree of a NURBS surface  from any
     degree to any higher degree. If the  output surface is  initialized 
     to NULL, memory to store new control points and knots is allocated. 
     If  the output  surface is  the same as  the input  surface, degree 
     elevation is done in place and the  original surface is  destroyed. 
     A typical calling example is:

       NL_SURFACE surP, surQ;
       NL_INDEX   t;
       NL_STACKS  SP, SQ;
       ...
       (define surP, get t);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfElevateDegree(&surP,t,NL_UDIR,&surQ,&SP,&SQ);
       N_SrfElevateDegree(&surP,t,NL_VDIR,&surP,&SP,&SP);

     If memory is  available, surQ  is not  initialized and  the routine
     assumes  that memory  allocation  has been done. However, it checks  
     for the proper amount by looking  at the  highest indexes in surQ's  
     knot vector and control net objects.

   ACCESS:
   
     surP , input  ,  NURBS surface
     t    , input  ,  Increment (new degree is old_degree+t)
     dir  , input  ,  Flag:
                        NL_UDIR: Elevate in u-direction
                        NL_VDIR: Elevate in v-direction
     surQ , output ,  Surface after degree elevation
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfElevateDegree( NL_SURFACE *surP, NL_INDEX t, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfElevateDegree");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, row, col, np, mp, rp, sp, nq, mq, rq, sq, r, s, a, b, mlt, kind, cind, pind = 0, lbz, rbz, bi, bj, di, dj, first, last, oldr, save;

    NL_DEGREE pp, qp, pq, qq;

    NL_REAL *UP, *VP, *UQ, *VQ, ** ralf, ** roma, ** rbet, ** romb, *alfs, *omas, num, den;

    NL_RMATRIX dm;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, ** Bw, ** Nw, ** Dw;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &np, &mp, &Pw, &pp, &qp, &rp, &sp, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:
            if( t LT 0 OR pp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knu, pp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knu, pp, &s );

            nq = np + t * s;
            rq = rp + t * (s + 1);
            pq = (NL_DEGREE)(pp + t);
            mq = mp;
            sq = sp;
            qq = qp;

            bi = pp;
            bj = mp;
            di = pq;
            dj = mp;

            break;

        case NL_VDIR:
            if( t LT 0 OR qp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knv, qp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knv, qp, &s );

            nq = np;
            rq = rp;
            pq = pp;
            mq = mp + t * s;
            sq = sp + t * (s + 1);
            qq = (NL_DEGREE)(qp + t);

            bi = np;
            bj = qp;
            di = np;
            dj = qq;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        surA = *surP;

        error = N_AllocSrfArrays( surP, nq, mq, pq, qq, rq, sq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, nq, mq, pq, qq, rq, sq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* See if elevation is required */

    if( t EQ 0 )
    {
        for ( row = 0; row <= np; row++ )
        {
            for ( col = 0; col <= mp; col++ )
                N_CopyCPt( Pw[row][col], &Qw[row][col] );
        }

        for ( i = 0; i <= rp; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= sp; j++ )
            VQ[j] = VP[j];

        NL_OUT;
    }

    /* Allocate local memory */

    a = NL_MAX( pp, qp );

    Bw = N_AllocCPt2dArray( bi, bj, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt2dArray( bi, bj, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    Dw = N_AllocCPt2dArray( di, dj, &SL );

    if( Dw EQ NULL )
        NL_QUIT;

    ralf = N_AllocReal2dArray( a, 2 * a, &SL );

    if( ralf EQ NULL )
        NL_QUIT;

    roma = N_AllocReal2dArray( a, 2 * a, &SL );

    if( roma EQ NULL )
        NL_QUIT;

    rbet = N_AllocReal2dArray( a, 2 * a, &SL );

    if( rbet EQ NULL )
        NL_QUIT;

    romb = N_AllocReal2dArray( a, 2 * a, &SL );

    if( romb EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( a, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( a, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Degree elevate in u-direction */

    if( dir EQ NL_UDIR )
    {
        a = pp;
        b = pp + 1;
        r = -1;
        cind = 1;
        kind = pq + 1;

        for ( i = 0; i <= pq; i++ )
            UQ[i] = UP[a];

        for ( j = 0; j <= sp; j++ )
            VQ[j] = VP[j];

        /* Initialize Bezier strip */

        for ( col = 0; col <= mp; col++ )
        {
            N_CopyCPt( Pw[0][col], &Qw[0][col] );

            for ( i = 0; i <= pp; i++ )
            {
                N_CopyCPt( Pw[i][col], &Bw[i][col] );
            }
        }

        /* Get degree elevation matrix */

        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( pp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the i-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the i-th and the (i-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT rp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT rp AND UP[b]EQ UP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = pp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = pq - (r + 1) / 2;
            else
                rbz = pq;

            /* Save some entities */

            if( r GT 0 )
            {
                num = UP[b] - UP[a];

                for ( k = pp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = UP[b] - UP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (UP[b] - UQ[i]) / (UP[a] - UQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (UP[b] - UQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each row, perform curve degree elevation */

            for ( col = 0; col <= mp; col++ )
            {
                pind = cind;

                /* Insert knot */

                if( r GT 0 )
                {
                    for ( j = 1; j <= r; j++ )
                    {
                        save = r - j;
                        s = mlt + j;

                        for ( k = pp; k >= s; k-- )
                        {
                            N_Combine2CPts( alfs[k - s], Bw[k][col], omas[k - s], Bw[k - 1][col], &Bw[k][col] );
                        }
                        N_CopyCPt( Bw[pp][col], &Nw[save][col] );
                    }
                } /* End of insert knot */

                /* Now degree elevate Bezier */

                error = N_BezSrfElevateDegree( Bw, pp, t, &dm, NL_UDIR, lbz, pq, col, Dw );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Remove the knot UP[a] */

                if( oldr GT 1 )
                {
                    first = kind - 2;
                    last = kind;

                    for ( k = 1; k < oldr; k++ )
                    {
                        i = first;
                        j = last;
                        l = j - kind + 1;

                        while( (j - i)GT k )
                        {
                            if( i LT cind )
                            {
                                N_Combine2CPts( ralf[k][i - first], Qw[i][col], roma[k][i - first], Qw[i - 1][col], &Qw[i][col] );
                            }

                            if( j GE lbz )
                            {
                                N_Combine2CPts( rbet[k][j - last], Dw[l][col], romb[k][j - last], Dw[l + 1][col], &Dw[l][col] );
                            }
                            i++;
                            j--;
                            l--;
                        }
                        first--;
                        last++;
                    }
                } /* End of removing knot */

                /* Load control points */

                for ( i = lbz; i <= rbz; i++ )
                {
                    N_CopyCPt( Dw[i][col], &Qw[pind][col] );
                    pind++;
                }

                /* Initialize for next pass through */

                if( b LT rp )
                {
                    for ( i = 0; i < r; i++ )
                        N_CopyCPt( Nw[i][col], &Bw[i][col] );

                    for ( i = r; i <= pp; i++ )
                        N_CopyCPt( Pw[b - pp + i][col], &Bw[i][col] );
                }
            } /* End for each row */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ pp )
            {
                for ( i = 0; i < pq - oldr; i++ )
                {
                    UQ[kind] = UP[a];
                    kind++;
                }
            }

            if( b LT rp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( i = 0; i <= pq; i++ )
                    UQ[kind + i] = UP[b];
            }
        } /* End of while */
    }     /* End of NL_UDIR */

    /* Degree elevate in v-direction */

    if( dir EQ NL_VDIR )
    {
        a = qp;
        b = qp + 1;
        r = -1;
        cind = 1;
        kind = qq + 1;

        for ( i = 0; i <= rp; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= qq; j++ )
            VQ[j] = VP[a];

        /* Initialize Bezier strip */

        for ( row = 0; row <= np; row++ )
        {
            N_CopyCPt( Pw[row][0], &Qw[row][0] );

            for ( j = 0; j <= qp; j++ )
            {
                N_CopyCPt( Pw[row][j], &Bw[row][j] );
            }
        }

        /* Get degree elevation matrix */

        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( qp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the j-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the j-th and the (j-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT sp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT sp AND VP[b]EQ VP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = qp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = qq - (r + 1) / 2;
            else
                rbz = qq;

            /* Save some entities */

            if( r GT 0 )
            {
                num = VP[b] - VP[a];

                for ( k = qp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (VP[a + k] - VP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = VP[b] - VP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (VP[b] - VQ[i]) / (VP[a] - VQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (VP[b] - VQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each column, perform curve elevation */

            for ( row = 0; row <= np; row++ )
            {
                pind = cind;

                /* Insert knot */

                if( r GT 0 )
                {
                    for ( j = 1; j <= r; j++ )
                    {
                        save = r - j;
                        s = mlt + j;

                        for ( k = qp; k >= s; k-- )
                        {
                            N_Combine2CPts( alfs[k - s], Bw[row][k], omas[k - s], Bw[row][k - 1], &Bw[row][k] );
                        }
                        N_CopyCPt( Bw[row][qp], &Nw[row][save] );
                    }
                } /* End of insert knot */

                /* Now degree elevate Bezier */

                error = N_BezSrfElevateDegree( Bw, qp, t, &dm, NL_VDIR, lbz, qq, row, Dw );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Remove the knot VP[a] */

                if( oldr GT 1 )
                {
                    first = kind - 2;
                    last = kind;

                    for ( k = 1; k < oldr; k++ )
                    {
                        i = first;
                        j = last;
                        l = j - kind + 1;

                        while( (j - i)GT k )
                        {
                            if( i LT cind )
                            {
                                N_Combine2CPts( ralf[k][i - first], Qw[row][i], roma[k][i - first], Qw[row][i - 1], &Qw[row][i] );
                            }

                            if( j GE lbz )
                            {
                                N_Combine2CPts( rbet[k][j - last], Dw[row][l], romb[k][j - last], Dw[row][l + 1], &Dw[row][l] );
                            }
                            i++;
                            j--;
                            l--;
                        }
                        first--;
                        last++;
                    }
                } /* End of removing knot */

                /* Load control points */

                for ( j = lbz; j <= rbz; j++ )
                {
                    N_CopyCPt( Dw[row][j], &Qw[row][pind] );
                    pind++;
                }

                /* Initialize for next pass through */

                if( b LT sp )
                {
                    for ( j = 0; j < r; j++ )
                        N_CopyCPt( Nw[row][j], &Bw[row][j] );

                    for ( j = r; j <= qp; j++ )
                        N_CopyCPt( Pw[row][b - qp + j], &Bw[row][j] );
                }
            } /* End for each column */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ qp )
            {
                for ( j = 0; j < qq - oldr; j++ )
                {
                    VQ[kind] = VP[a];
                    kind++;
                }
            }

            if( b LT sp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( j = 0; j <= qq; j++ )
                    VQ[kind + j] = VP[b];
            }
        } /* End of while */
    }     /* End of NL_VDIR */

    /* If insertion is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfElevateDegree */



/*******************************************************************//**


   DESCRIPTION:

     This tools routine reduces the degree of a NURBS surface by one.
     If the output  surface is  initialized to  NULL, memory to store 
     new control points and knots is allocated. If the output surface 
     is the  same as the input  surface, degree  reduction is done in 
     place and the original  surface is  destroyed. A typical calling 
     example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     tol, mtol;
       NL_FLAG     rfl;
       NL_STACKS   SP, SQ;
       ...
       (define surP, get tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfReduceDegree(&surP,tol,NL_UDIR,&rfl,&surQ,&mtol,&SP,&SQ);
       N_SrfReduceDegree(&surP,tol,NL_VDIR,&rfl,&surP,&mtol,&SP,&SP);

     If memory is available, surQ is not initialized and  the routine
     assumes that memory allocation has been done. However, it checks  
     for the  proper  amount  by looking  at the  highest  indexes in 
     surQ's knot vector and control net objects.

   ACCESS:
   
     surP , input  ,  NURBS surface
     tol  , input  ,  Tolerance of reduction
     dir  , input  ,  Flag: 
                        NL_UDIR: Reduce in u-direction
                        NL_VDIR: Reduce in v-direction
     rfl  , output ,  Flag:
                        NL_YES: Reduction is successful
                        NL_NO : Reduction is not successful
     surQ , output ,  Surface after degree reduction
     mtol , output ,  Maximum tolerance over the surface
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReduceDegree( NL_SURFACE *surP, NL_REAL tol, NL_FLAG dir, NL_FLAG *rfl, NL_SURFACE *surQ, NL_REAL *mtol, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReduceDegree");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, k, l, row, col, np, mp, rp, sp, nq, mq, rq, sq, r, s, a, b, c, spn, mlt, oldmlt, kind, cind, lbz, bi, bj, di, dj, first, last, oldr, save, kk, ll, ii, jj, pind = 0;

    NL_DEGREE pp, qp, pq, qq;

    NL_REAL *UP, *VP, *UQ, *VQ, *X, ** ralf, ** roma, ** rbet, ** romb, ** ealf, ** eoma, ** alam, ** aoml, ** e, ** max, *alfs, dw, *omas, *dalf, *doma, *dbet, *domb, *minl, *maxl, de, *minr, *maxr, num, wmin, pmax, alf, bet, omb, tmp, den, wi, wl, lto;

    NL_KNOTVECTOR *knu, *knv, knx;

    NL_CPOINT ** Pw, ** Qw, ** Bw, ** Nw, ** Dw, A;

    NL_SURFACE surA;
    N_SrfInitArrays( &surA );

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &np, &mp, &Pw, &pp, &qp, &rp, &sp, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* Adjust tolerance in case of rational surfaces */

    if( N_IsSrfRat( surP ) )
    {
        N_SrfMinMaxWeightPosVectors( surP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }
    else
    {
        rat = NL_NO;
    }

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:
            if( pp LE 1 )
            {
                *rfl = NL_NO;
                NL_OUT;
            }

            N_BasisGetSpanCount( knu, pp, &s );

            nq = np - s;
            rq = rp - s - 1;
            pq = pp - 1;
            mq = mp;
            sq = sp;
            qq = qp;

            bi = pp;
            bj = mp;
            di = pq;
            dj = mp;

            lto = sto * fabs( UP[rp] - UP[0] );
            break;

        case NL_VDIR:
            if( qp LE 1 )
            {
                *rfl = NL_NO;
                NL_OUT;
            }

            N_BasisGetSpanCount( knv, qp, &s );

            nq = np;
            rq = rp;
            pq = pp;
            mq = mp - s;
            sq = sp - s - 1;
            qq = qp - 1;

            bi = np;
            bj = qp;
            di = np;
            dj = qq;

            lto = sto * fabs( VP[sp] - VP[0] );
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        surA = *surP;

        error = N_AllocSrfArrays( surP, nq, mq, pq, qq, rq, sq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, nq, mq, pq, qq, rq, sq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Allocate local memory */

    a = NL_MAX( pp, qp );

    Bw = N_AllocCPt2dArray( bi, bj, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt2dArray( bi, bj, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    Dw = N_AllocCPt2dArray( di, dj, &SL );

    if( Dw EQ NULL )
        NL_QUIT;

    ralf = N_AllocReal2dArray( a, 2 * a, &SL );

    if( ralf EQ NULL )
        NL_QUIT;

    roma = N_AllocReal2dArray( a, 2 * a, &SL );

    if( roma EQ NULL )
        NL_QUIT;

    rbet = N_AllocReal2dArray( a, 2 * a, &SL );

    if( rbet EQ NULL )
        NL_QUIT;

    romb = N_AllocReal2dArray( a, 2 * a, &SL );

    if( romb EQ NULL )
        NL_QUIT;

    ealf = N_AllocReal2dArray( a, 2 * a, &SL );

    if( ealf EQ NULL )
        NL_QUIT;

    eoma = N_AllocReal2dArray( a, 2 * a, &SL );

    if( eoma EQ NULL )
        NL_QUIT;

    alam = N_AllocReal2dArray( a, 2 * a, &SL );

    if( alam EQ NULL )
        NL_QUIT;

    aoml = N_AllocReal2dArray( a, 2 * a, &SL );

    if( aoml EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( a, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( a, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    dalf = N_AllocReal1dArray( a, &SL );

    if( dalf EQ NULL )
        NL_QUIT;

    doma = N_AllocReal1dArray( a, &SL );

    if( doma EQ NULL )
        NL_QUIT;

    dbet = N_AllocReal1dArray( a, &SL );

    if( dbet EQ NULL )
        NL_QUIT;

    domb = N_AllocReal1dArray( a, &SL );

    if( domb EQ NULL )
        NL_QUIT;

    X = N_AllocReal1dArray( 3 * a + 2, &SL );

    if( X EQ NULL )
        NL_QUIT;

    e = N_AllocReal2dArray( rp, sp, &SL );

    if( e EQ NULL )
        NL_QUIT;

    minl = N_AllocReal1dArray( a, &SL );

    if( minl EQ NULL )
        NL_QUIT;

    maxl = N_AllocReal1dArray( a, &SL );

    if( maxl EQ NULL )
        NL_QUIT;

    minr = N_AllocReal1dArray( a, &SL );

    if( minr EQ NULL )
        NL_QUIT;

    maxr = N_AllocReal1dArray( a, &SL );

    if( maxr EQ NULL )
        NL_QUIT;

    max = N_AllocReal2dArray( a, a + 1, &SL );

    if( max EQ NULL )
        NL_QUIT;

    /* Initialize error vector */

    for ( i = 0; i <= rp; i++ )
    {
        for ( j = 0; j <= sp; j++ )
            e[i][j] = 0.0;
    }

    *mtol = 0.0;

    /* Degree reduce in u-direction */

    if( dir EQ NL_UDIR )
    {
        a = pp;
        b = pp + 1;
        r = -1;
        mlt = pp + 1;
        spn = pp + 1;
        cind = 1;
        kind = pq + 1;
        *rfl = NL_YES;

        for ( i = 0; i <= pq; i++ )
            UQ[i] = UP[a];

        for ( j = 0; j <= sp; j++ )
            VQ[j] = VP[j];

        N_BezDegreeReduceCoefs( pp, dalf, doma, dbet, domb );

        /* Initialize Bezier strip */

        for ( col = 0; col <= mp; col++ )
        {
            N_CopyCPt( Pw[0][col], &Qw[0][col] );

            for ( i = 0; i <= pp; i++ )
            {
                N_CopyCPt( Pw[i][col], &Bw[i][col] );
            }
        }

        /*************************************************************/
        /* Loop through the U knot vector and do the following:      */
        /*   (1) Extract the i-th Bezier strip.                      */
        /*   (2) Degree reduce the strip.                            */
        /*   (3) Remove the knot between the i-th and the (i-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b < rp )
        {
            /* Get multiplicity of the knot */

            i = spn;

            while( spn LT rp AND UP[spn]EQ UP[spn + 1] )
                spn++;
            oldmlt = mlt;
            mlt = spn - i + 1;

            b = b + mlt - 1;
            oldr = r;
            r = pp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            /* Save knot insertion alfas */

            if( r GT 0 )
            {
                num = UP[b] - UP[a];

                for ( k = pp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            /* Save knot removal alfas, betas and maximums of basis function */

            if( oldr GT 0 )
            {
                first = kind;
                last = kind;
                den = UP[b] - UP[a];

                for ( k = 0; k < oldr; k++ )
                {
                    i = first;
                    j = last;

                    while( (j - i)GT k )
                    {
                        ralf[k][i - first] = (UP[b] - UQ[i - 1]) / (UP[a] - UQ[i - 1]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                        rbet[k][last - j] = (UP[b] - UQ[j - k - 1]) / den;
                        romb[k][last - j] = 1.0 - rbet[k][last - j];
                        i++;
                        j--;
                    }

                    kk = a + oldr - k;
                    ll = kk - (2 * pp - k + 1) / 2;
                    jj = k % 2;
                    c = -1;

                    for ( ii = 0; ii <= pp; ii++ )
                        X[++c] = UP[ll];

                    for ( ii = ll + 1; ii <= a - oldmlt; ii++ )
                        X[++c] = UP[ii];

                    for ( ii = 1; ii <= pp - k; ii++ )
                        X[++c] = UP[a];

                    for ( ii = kk - ll - jj; ii <= 2 *pp; ii++ )
                        X[++c] = UP[b];

                    N_KnotVectorFromRealArray( &knx, X, c );

                    if( (j - i)LT k )
                    {
                        alf = (UP[a] - UQ[i - 2]) / (UP[b] - UQ[i - 2]);
                        bet = (UP[a] - UQ[i - 1]) / (UP[b] - UQ[i - 1]);
                        omb = 1.0 - bet;
                        alam[k][i - first] = alf / (alf + bet);
                        aoml[k][i - first] = 1.0 - alam[k][i - first];

                        error = N_BasisFindAllSpanMaxima( &knx, pp, pp, lto, minl, maxl, &tmp );

                        if( error EQ NL_YES )
                        {
                            for ( ii = 0; ii <= pp; ii++ )
                            {
                                minl[ii] = 0.0;
                                maxl[ii] = 1.0;
                            }
                        }

                        error = N_BasisFindAllSpanMaxima( &knx, pp + 1, pp, lto, minr, maxr, &tmp );

                        if( error EQ NL_YES )
                        {
                            for ( ii = 0; ii <= pp; ii++ )
                            {
                                minr[ii] = 0.0;
                                maxr[ii] = 1.0;
                            }
                        }

                        max[k][0] = fabs( alam[k][i - first] * alf * maxl[0] );

                        for ( ii = 1; ii <= pp; ii++ )
                        {
                            minl[ii] *= alam[k][i - first] * alf;
                            minr[ii - 1] *= aoml[k][i - first] * omb;
                            maxl[ii] *= alam[k][i - first] * alf;
                            maxr[ii - 1] *= aoml[k][i - first] * omb;

                            max[k][ii] = NL_MAX( fabs( maxl[ii] - minr[ii - 1] ), fabs( maxr[ii - 1] - minl[ii] ) );
                        }
                        max[k][pp + 1] = fabs( aoml[k][i - first] * omb * maxr[pp] );
                    }
                    else
                    {
                        ealf[k][i - first] = (UP[a] - UQ[i - 1]) / (UP[b] - UQ[i - 1]);
                        eoma[k][i - first] = 1.0 - ealf[k][i - first];

                        error = N_BasisFindAllSpanMaxima( &knx, pp, pp, lto, minl, max[k], &tmp );

                        if( error EQ NL_YES )
                        {
                            for ( ii = 0; ii <= pp; ii++ )
                                max[k][ii] = 1.0;
                        }
                    }
                    first--;
                    last++;
                }
            }

            /* For each row, perform curve degree reduction */

            for ( col = 0; col <= mp; col++ )
            {
                pind = cind;

                /* Insert knot */

                if( r GT 0 )
                {
                    for ( j = 1; j <= r; j++ )
                    {
                        save = r - j;
                        s = mlt + j;

                        for ( k = pp; k >= s; k-- )
                        {
                            N_Combine2CPts( alfs[k - s], Bw[k][col], omas[k - s], Bw[k - 1][col], &Bw[k][col] );
                        }
                        N_CopyCPt( Bw[pp][col], &Nw[save][col] );
                    }
                } /* End of insert knot */

                /* Now degree reduce Bezier */

                error = N_BezSrfReduceDegree( Bw, pp, qp, NL_UDIR, col, dalf, doma, dbet, domb, Dw, &de );

                if( error EQ NL_YES )
                    NL_OUT;

                e[a][col] = e[a][col] + de;

                if( e[a][col]GT *mtol )
                    *mtol = e[a][col];

                if( e[a][col]GT tol )
                {
                    *rfl = NL_NO;
                    break;
                }

                /* Remove the knot UP[a] */

                if( oldr GT 0 )
                {
                    first = kind;
                    last = kind;

                    for ( k = 0; k < oldr; k++ )
                    {
                        i = first;
                        j = last;
                        l = j - kind;

                        while( (j - i)GT k )
                        {
                            N_Combine2CPts( ralf[k][i - first], Qw[i - 1][col], roma[k][i - first], Qw[i - 2][col], &Qw[i - 1][col] );
                            N_Combine2CPts( rbet[k][last - j], Dw[l][col], romb[k][last - j], Dw[l + 1][col], &Dw[l][col] );

                            if( rat EQ NL_YES )
                            {
                                N_CPtGetW( Qw[i - 1][col], &wi );
                                N_CPtGetW( Dw[l][col], &wl );

                                if( wi LT NL_WMIN )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }

                                if( wl LT NL_WMIN )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }

                                if( wi GT NL_WMAX )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }

                                if( wl GT NL_WMAX )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }
                            }
                            i++;
                            j--;
                            l--;
                        }

                        if( *rfl EQ NL_NO )
                            break;

                        /* Compute the error */

                        if( (j - i)LT k )
                        {
                            N_DistCptCptHomo( Qw[i - 2][col], Dw[l + 1][col], &dw );
                        }
                        else
                        {
                            N_Combine2CPts( ealf[k][i - first], Dw[l + 1][col], eoma[k][i - first], Qw[i - 2][col], &A );
                            N_DistCptCptHomo( Qw[i - 1][col], A, &dw );
                        }

                        kk = a + oldr - k;
                        ll = kk - (2 * pp - k + 1) / 2;

                        for ( ii = ll; ii <= a; ii++ )
                        {
                            if( UP[ii]NEQ UP[ii + 1] )
                            {
                                e[ii][col] = e[ii][col] + dw * max[k][ii - ll];

                                if( e[ii][col]GT *mtol )
                                    *mtol = e[ii][col];

                                if( e[ii][col]GT tol )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }
                            }
                        }

                        if( *rfl EQ NL_NO )
                            break;

                        /* Average control points */

                        if( (j - i)LT k )
                        {
                            N_Combine2CPts( alam[k][i - first], Dw[l + 1][col], aoml[k][i - first], Qw[i - 2][col], &Qw[i - 2][col] );
                        }
                        first--;
                        last++;
                    }

                    if( *rfl EQ NL_NO )
                        break;
                    pind = i - 1;
                } /* End of removing knot */

                /* Load control points */

                for ( i = lbz; i <= pq; i++ )
                {
                    N_CopyCPt( Dw[i][col], &Qw[pind][col] );
                    pind++;
                }

                /* Initialize for next pass through */

                if( b LT rp )
                {
                    for ( i = 0; i < r; i++ )
                        N_CopyCPt( Nw[i][col], &Bw[i][col] );

                    for ( i = r; i <= pp; i++ )
                        N_CopyCPt( Pw[b - pp + i][col], &Bw[i][col] );
                }
            } /* End for each row */

            if( *rfl EQ NL_NO )
                break;

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ pp )
            {
                for ( i = 0; i < pq - oldr; i++ )
                {
                    UQ[kind] = UP[a];
                    kind++;
                }
            }

            if( b LT rp )
            {
                a = b;
                b++;
                spn++;
            }
            else
            {
                for ( i = 0; i <= pq; i++ )
                    UQ[kind + i] = UP[b];
            }
        } /* End of while */
    }     /* End of NL_UDIR */

    /* Degree reduce in v-direction */

    if( dir EQ NL_VDIR )
    {
        a = qp;
        b = qp + 1;
        r = -1;
        mlt = qp + 1;
        spn = qp + 1;
        cind = 1;
        kind = qq + 1;
        *rfl = NL_YES;

        for ( j = 0; j <= qq; j++ )
            VQ[j] = VP[a];

        for ( i = 0; i <= rp; i++ )
            UQ[i] = UP[i];

        N_BezDegreeReduceCoefs( qp, dalf, doma, dbet, domb );

        /* Initialize Bezier strip */

        for ( row = 0; row <= np; row++ )
        {
            N_CopyCPt( Pw[row][0], &Qw[row][0] );

            for ( i = 0; i <= qp; i++ )
            {
                N_CopyCPt( Pw[row][i], &Bw[row][i] );
            }
        }

        /*************************************************************/
        /* Loop through the V knot vector and do the following:      */
        /*   (1) Extract the j-th Bezier strip.                      */
        /*   (2) Degree reduce the strip.                            */
        /*   (3) Remove the knot between the j-th and the (j-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b < sp )
        {
            /* Get multiplicity of the knot */

            i = spn;

            while( spn LT sp AND VP[spn]EQ VP[spn + 1] )
                spn++;
            oldmlt = mlt;
            mlt = spn - i + 1;

            b = b + mlt - 1;
            oldr = r;
            r = qp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            /* Save knot insertion alfas */

            if( r GT 0 )
            {
                num = VP[b] - VP[a];

                for ( k = qp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (VP[a + k] - VP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            /* Save knot removal alfas, betas  and basis function maximums */

            if( oldr GT 0 )
            {
                first = kind;
                last = kind;
                den = VP[b] - VP[a];

                for ( k = 0; k < oldr; k++ )
                {
                    i = first;
                    j = last;

                    while( (j - i)GT k )
                    {
                        ralf[k][i - first] = (VP[b] - VQ[i - 1]) / (VP[a] - VQ[i - 1]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                        rbet[k][last - j] = (VP[b] - VQ[j - k - 1]) / den;
                        romb[k][last - j] = 1.0 - rbet[k][last - j];
                        i++;
                        j--;
                    }

                    kk = a + oldr - k;
                    ll = kk - (2 * qp - k + 1) / 2;
                    jj = k % 2;
                    c = -1;

                    for ( ii = 0; ii <= qp; ii++ )
                        X[++c] = VP[ll];

                    for ( ii = ll + 1; ii <= a - oldmlt; ii++ )
                        X[++c] = VP[ii];

                    for ( ii = 1; ii <= qp - k; ii++ )
                        X[++c] = VP[a];

                    for ( ii = kk - ll - jj; ii <= 2 *qp; ii++ )
                        X[++c] = VP[b];

                    N_KnotVectorFromRealArray( &knx, X, c );

                    if( (j - i)LT k )
                    {
                        alf = (VP[a] - VQ[i - 2]) / (VP[b] - VQ[i - 2]);
                        bet = (VP[a] - VQ[i - 1]) / (VP[b] - VQ[i - 1]);
                        omb = 1.0 - bet;
                        alam[k][i - first] = alf / (alf + bet);
                        aoml[k][i - first] = 1.0 - alam[k][i - first];

                        error = N_BasisFindAllSpanMaxima( &knx, qp, qp, lto, minl, maxl, &tmp );

                        if( error EQ NL_YES )
                        {
                            for ( ii = 0; ii <= qp; ii++ )
                            {
                                minl[ii] = 0.0;
                                maxl[ii] = 1.0;
                            }
                        }

                        error = N_BasisFindAllSpanMaxima( &knx, qp + 1, qp, lto, minr, maxr, &tmp );

                        if( error EQ NL_YES )
                        {
                            for ( ii = 0; ii <= qp; ii++ )
                            {
                                minr[ii] = 0.0;
                                maxr[ii] = 1.0;
                            }
                        }

                        max[k][0] = fabs( alam[k][i - first] * alf * maxl[0] );

                        for ( ii = 1; ii <= qp; ii++ )
                        {
                            minl[ii] *= alam[k][i - first] * alf;
                            minr[ii - 1] *= aoml[k][i - first] * omb;
                            maxl[ii] *= alam[k][i - first] * alf;
                            maxr[ii - 1] *= aoml[k][i - first] * omb;

                            max[k][ii] = NL_MAX( fabs( maxl[ii] - minr[ii - 1] ), fabs( maxr[ii - 1] - minl[ii] ) );
                        }
                        max[k][qp + 1] = fabs( aoml[k][i - first] * omb * maxr[qp] );
                    }
                    else
                    {
                        ealf[k][i - first] = (VP[a] - VQ[i - 1]) / (VP[b] - VQ[i - 1]);
                        eoma[k][i - first] = 1.0 - ealf[k][i - first];

                        error = N_BasisFindAllSpanMaxima( &knx, qp, qp, lto, minl, max[k], &tmp );

                        if( error EQ NL_YES )
                        {
                            for ( ii = 0; ii <= qp; ii++ )
                                max[k][ii] = 1.0;
                        }
                    }
                    first--;
                    last++;
                }
            }

            /* For each column, perform curve degree reduction */

            for ( row = 0; row <= np; row++ )
            {
                pind = cind;

                /* Insert knot */

                if( r GT 0 )
                {
                    for ( j = 1; j <= r; j++ )
                    {
                        save = r - j;
                        s = mlt + j;

                        for ( k = qp; k >= s; k-- )
                        {
                            N_Combine2CPts( alfs[k - s], Bw[row][k], omas[k - s], Bw[row][k - 1], &Bw[row][k] );
                        }
                        N_CopyCPt( Bw[row][qp], &Nw[row][save] );
                    }
                } /* End of insert knot */

                /* Now degree reduce Bezier */

                error = N_BezSrfReduceDegree( Bw, pp, qp, NL_VDIR, row, dalf, doma, dbet, domb, Dw, &de );

                if( error EQ NL_YES )
                    NL_OUT;

                e[row][a] = e[row][a] + de;

                if( e[row][a]GT *mtol )
                    *mtol = e[row][a];

                if( e[row][a]GT tol )
                {
                    *rfl = NL_NO;
                    break;
                }

                /* Remove the knot VP[a] */

                if( oldr GT 0 )
                {
                    first = kind;
                    last = kind;

                    for ( k = 0; k < oldr; k++ )
                    {
                        i = first;
                        j = last;
                        l = j - kind;

                        while( (j - i)GT k )
                        {
                            N_Combine2CPts( ralf[k][i - first], Qw[row][i - 1], roma[k][i - first], Qw[row][i - 2], &Qw[row][i - 1] );
                            N_Combine2CPts( rbet[k][last - j], Dw[row][l], romb[k][last - j], Dw[row][l + 1], &Dw[row][l] );

                            if( rat EQ NL_YES )
                            {
                                N_CPtGetW( Qw[row][i - 1], &wi );
                                N_CPtGetW( Dw[row][l], &wl );

                                if( wi LT NL_WMIN )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }

                                if( wl LT NL_WMIN )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }

                                if( wi GT NL_WMAX )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }

                                if( wl GT NL_WMAX )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }
                            }
                            i++;
                            j--;
                            l--;
                        }

                        if( *rfl EQ NL_NO )
                            break;

                        /* Compute the error */

                        if( (j - i)LT k )
                        {
                            N_DistCptCptHomo( Qw[row][i - 2], Dw[row][l + 1], &dw );
                        }
                        else
                        {
                            N_Combine2CPts( ealf[k][i - first], Dw[row][l + 1], eoma[k][i - first], Qw[row][i - 2], &A );
                            N_DistCptCptHomo( Qw[row][i - 1], A, &dw );
                        }

                        kk = a + oldr - k;
                        ll = kk - (2 * qp - k + 1) / 2;

                        for ( ii = ll; ii <= a; ii++ )
                        {
                            if( VP[ii]NEQ VP[ii + 1] )
                            {
                                e[row][ii] = e[row][ii] + dw * max[k][ii - ll];

                                if( e[row][ii]GT *mtol )
                                    *mtol = e[row][ii];

                                if( e[row][ii]GT tol )
                                {
                                    *rfl = NL_NO;
                                    break;
                                }
                            }
                        }

                        if( *rfl EQ NL_NO )
                            break;

                        /* Average control points */

                        if( (j - i)LT k )
                        {
                            N_Combine2CPts( alam[k][i - first], Dw[row][l + 1], aoml[k][i - first], Qw[row][i - 2], &Qw[row][i - 2] );
                        }
                        first--;
                        last++;
                    }

                    if( *rfl EQ NL_NO )
                        break;
                    pind = i - 1;
                } /* End of removing knot */

                /* Load control points */

                for ( i = lbz; i <= qq; i++ )
                {
                    N_CopyCPt( Dw[row][i], &Qw[row][pind] );
                    pind++;
                }

                /* Initialize for next pass through */

                if( b LT sp )
                {
                    for ( i = 0; i < r; i++ )
                        N_CopyCPt( Nw[row][i], &Bw[row][i] );

                    for ( i = r; i <= qp; i++ )
                        N_CopyCPt( Pw[row][b - qp + i], &Bw[row][i] );
                }
            } /* End for each column */

            if( *rfl EQ NL_NO )
                break;

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ qp )
            {
                for ( i = 0; i < qq - oldr; i++ )
                {
                    VQ[kind] = VP[a];
                    kind++;
                }
            }

            if( b LT sp )
            {
                a = b;
                b++;
                spn++;
            }
            else
            {
                for ( i = 0; i <= qq; i++ )
                    VQ[kind + i] = VP[b];
            }
        } /* End of while */
    }     /* End of NL_VDIR */

    /* If insertion is in place, kill old surface */

    if( surP EQ surQ )
    {
        if( *rfl EQ NL_YES )
        {
            N_FreeSrf( &surA, SP );
        }
        else
        {
            N_FreeSrf( surP, SP );
            *surP = surA;
        }
    }
    else
    {
        if( *rfl EQ NL_NO )
        {
            N_FreeSrf( surQ, SQ );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReduceDegree */



/*******************************************************************//**


   DESCRIPTION:

     This tools routine reduces the degree of a NURBS surface as much 
     as possible. That is, it keeps reducing  the degree by one until 
     the reduction error exceeds the tolerance. If the output surface 
     is the  same as the  input surface, degree  reduction is done in 
     place and the  original surface is  destroyed. A typical calling 
     example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     tol;
       NL_STACKS   SP, SQ;
       ...
       (define surP, get tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfReduceDegreeToTol(&surP,tol,&surQ,&SP,&SQ);
       N_SrfReduceDegreeToTol(&surP,tol,&surP,&SP,&SP);

     If memory is available, surQ is not initialized and the  routine
     assumes that memory allocation has been done. However, it checks  
     for  the proper  amount  by  looking at  the  highest indexes in 
     surQ's knot vector and control net objects.

   ACCESS:
   
     surP , input  ,  NURBS surface
     tol  , input  ,  Tolerance of degree reduction
     surQ , output ,  Surface after degree reduction
     SP   , input  ,  surP's memory stack
     SQ   , input  ,  surQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReduceDegreeToTol( NL_SURFACE *surP, NL_REAL tol, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_FLAG rfu, rfv, dir, error = NL_NO;

    NL_REAL ctol, mtol;

    NL_SURFACE *sur;

    NL_STACKS *S, SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Reduce the degree to as low as possible */

    ctol = 0.0;
    rfu = NL_TRUE;
    rfv = NL_TRUE;
    dir = NL_UDIR;

    sur = surP;
    S = SP;

    if( surP NEQ surQ )
    {
        error = N_SrfCopy( surP, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        sur = surQ;
        S = SQ;
    }

    while( (rfu EQ NL_TRUE OR rfv EQ NL_TRUE)AND ctol LE tol )
    {
        if( rfu EQ NL_TRUE AND dir EQ NL_UDIR )
        {
            error = N_SrfReduceDegree( sur, tol - ctol, NL_UDIR, &rfu, sur, &mtol, S, S );

            if( error EQ NL_YES )
                NL_OUT;

            if( rfu EQ NL_TRUE )
                ctol += mtol;
        }

        if( rfv EQ NL_TRUE AND dir EQ NL_VDIR )
        {
            error = N_SrfReduceDegree( sur, tol - ctol, NL_VDIR, &rfv, sur, &mtol, S, S );

            if( error EQ NL_YES )
                NL_OUT;

            if( rfv EQ NL_TRUE )
                ctol += mtol;
        }

        if( rfu EQ NL_TRUE AND rfv EQ NL_TRUE )
        {
            if( dir EQ NL_UDIR )
                dir = NL_VDIR;
            else
                dir = NL_UDIR;
        }

        if( rfu EQ NL_FALSE )
            dir = NL_VDIR;

        if( rfv EQ NL_FALSE )
            dir = NL_UDIR;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReduceDegreeToTol */

/*******************************************************************//**


   DESCRIPTION:
     Obsolete:  Use  N_MakeSrfsCompatible

     This tools  routine makes  a set of surfaces  compatible, i.e. it 
     raises the degrees and inserts knots until all surfaces  have the 
     same degree and are defined over the same knot vectors. A typical 
     calling example is:

       NL_SURFACE **sur;
       NL_INDEX   k;
       NL_STACKS  S;
       ...
       (define array of sur);
       ...
       N_MakeSrfsCompatibleUV(sur,k,&S); 

     THE  ALGORITHM  WORKS  IN  PLACE, I.E. THE  ORIGINAL SURFACES ARE 
     DESTROYED! ALL SURFACES MUST BELONG TO THE SAME STACK!

   ACCESS:
   
     sur , in/out ,  An array of NURBS surfaces
     k   , input  ,  Highest index in array
     S   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MakeSrfsCompatibleUV( NL_SURFACE ** sur, NL_INDEX k, NL_STACKS *S )
{
    return (N_MakeSrfsCompatible( sur, k, NL_UVDIR, S ));
} /* end N_MakeSrfsCompatibleUV */



/*******************************************************************//**


   DESCRIPTION:

     This tools  routine makes  a set of surfaces  compatible, i.e. it 
     raises the degrees and inserts knots until all surfaces  have the 
     same degree and are defined over the same knot vectors. A typical 
     calling example is:

       NL_SURFACE **sur;
       NL_INDEX   k;
       NL_FLAG    UorV;
       NL_STACKS  S;
       ...
       (define array of sur);
       ...
       N_MakeSrfsCompatible(sur, k, UorV, &S); 

     THE  ALGORITHM  WORKS  IN  PLACE, I.E. THE  ORIGINAL SURFACES ARE 
     DESTROYED! ALL SURFACES MUST BELONG TO THE SAME STACK!

   ACCESS:
   
     sur , in/out ,  An array of NURBS surfaces
     k   , input  ,  Highest index in array
     UorV, input  ,  compatible in NL_UDIR, NL_VDIR, NL_UVDIR (both)
     S   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MakeSrfsCompatible( NL_SURFACE ** sur, NL_INDEX k, NL_FLAG UorV, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, r = 0, s = 0, tu, tv, iu, iv;

    NL_DEGREE p, q, ph, qh;

    NL_REAL *U, *V, d1, d2, d3, d4, us, ue, vs, ve;

    NL_KNOTVECTOR ** knu, ** knv, ** kux = NULL, ** kvx = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make surface definitions compatible */

    N_MakeSrfsRatCompatible( sur, k );

    /* Scale surface knot vectors if necessary */

    N_SrfGetParameterBounds( sur[0], &us, &ue, &vs, &ve );
    iu = iv = 0;

    for ( i = 1; i <= k; i++ )
    {
        N_SrfGetParameterBounds( sur[i], &d1, &d2, &d3, &d4 );

        if( us NEQ d1 OR ue NEQ d2 )
            iu = 1;

        if( vs NEQ d3 OR ve NEQ d4 )
            iv = 1;
    }

    if( iu == 1 AND( UorV EQ NL_VDIR ) )
        iu = 0;

    if( iv == 1 AND( UorV EQ NL_UDIR ) )
        iv = 0;

    if( iu EQ 1 AND iv EQ 0 )
    {
        for ( i = 0; i <= k; i++ )
            N_SrfReparam( sur[i], 0.0, 1.0, vs, ve );
    }
    else if( iu EQ 0 AND iv EQ 1 )
    {
        for ( i = 0; i <= k; i++ )
            N_SrfReparam( sur[i], us, ue, 0.0, 1.0 );
    }
    else if( iu EQ 1 AND iv EQ 1 )
    {
        for ( i = 0; i <= k; i++ )
            N_SrfReparamToInterval( sur[i], NL_UNITSQUARE, NL_UVDIR );
    }

    /* Get highest degrees */

    N_SrfGetDegrees( sur[0], &ph, &qh );

    for ( i = 1; i <= k; i++ )
    {
        N_SrfGetDegrees( sur[i], &p, &q );

        if( p GT ph )
            ph = p;

        if( q GT qh )
            qh = q;
    }

    /* Elevate the degrees */

    for ( i = 0; i <= k; i++ )
    {
        N_SrfGetDegrees( sur[i], &p, &q );

        tu = ph - p;
        tv = qh - q;

        if( tu GT 0 )
        {
            error = N_SrfElevateDegree( sur[i], tu, NL_UDIR, sur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( tv GT 0 )
        {
            error = N_SrfElevateDegree( sur[i], tv, NL_VDIR, sur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Merge knot vectors */

    knu = N_Alloc1dArrayKnotVectPtrs( k, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    knv = N_Alloc1dArrayKnotVectPtrs( k, &SL );

    if( knv EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        N_SrfGetKnotVectors( sur[i], &knu[i], &knv[i] );
    }

    if( UorV NEQ NL_VDIR )
    {
        error = N_GetCompatibleKnotArray( knu, k, &kux, &SL );

        if( error EQ NL_YES )
            NL_QUIT;
    }

    if( UorV NEQ NL_UDIR )
    {
        error = N_GetCompatibleKnotArray( knv, k, &kvx, &SL );

        if( error EQ NL_YES )
            NL_QUIT;
    }

    /* Refine surfaces */

    for ( i = 0; i <= k; i++ )
    {
        if( UorV NEQ NL_VDIR )
            N_KnotVectorGetKnots( kux[i], &r, &U );

        if( UorV NEQ NL_UDIR )
            N_KnotVectorGetKnots( kvx[i], &s, &V );

        if( UorV NEQ NL_VDIR AND r GE 0 )
        {
            error = N_SrfInsertKnots( sur[i], kux[i], NL_UDIR, sur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( UorV NEQ NL_UDIR AND s GE 0 )
        {
            error = N_SrfInsertKnots( sur[i], kvx[i], NL_VDIR, sur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_MakeSrfsCompatible */

#if NLIB_UNUSED

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This  tools routine  removes  all  removable  knots  from a NURBS 
     surface with boundary constraints. It reparametrizes  the surface 
     with respect to arc length to be able to remove as  many knots as 
     possible. This  routine is  suitable for  NURBS  surfaces stiched 
     together  from  Bezier pieces  with  bad  parametrization. If the 
     output  surface is  initialized to  NULL,  memory  to  store  new 
     control points and knots is  allocated. If the output  surface is  
     the same as the input surface, knot removal  is in place  and the  
     original surface is  destroyed. A typical calling example is:
 
       NL_SURFACE  surP, surQ;
       NL_REAL     emx, eub, evb, eut, evt;
       NL_STACKS   SQ;
       ...
       (define surP, choose error tolerances);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfReparmAndRemoveKnotsKeepBoundaries(&surP,NL_BOTH,NL_NO ,emx,eub,evb,eut,evt,NL_UDIR ,&surQ,&SQ);
       N_SrfReparmAndRemoveKnotsKeepBoundaries(&surP,NL_BOTH,NL_END,emx,eub,evb,eut,evt,NL_UVDIR,&surP,&SQ);
 
     If memory is  available, surQ is not initialized and the routine
     assumes that memory allocation has been done. However, it checks  
     for  the  proper  amount by  looking  at the  highest indexes in 
     surQ's knot vector  and  control net objects. THE  ROUTINE  USES  
     0.1% NL_RELATIVE TOLERANCE TO APPROXIMATE ARC LENGTH.
 
 
   ACCESS:
   
     surP , input  ,  NURBS surface
     ufl  , input  ,  Flag:
                        NL_NO   : do not constrain u-tangents
                        NL_START: constrain u-tangents at u=umin
                        NL_END  : constrain u-tangents at u=umax
                        NL_BOTH : constrain u-tangents at both ends
     vfl  , input  ,  Flag:
                        NL_NO   : do not constrain v-tangents
                        NL_START: constrain v-tangents at v=vmin
                        NL_END  : constrain v-tangents at v=vmax
                        NL_BOTH : constrain v-tangents at both ends
     emx  , input  ,  Overall error tolerance, i.e. the new surface will
                      not deviate from the origonal surface by more than
                      emx
     eub  , input  ,  Error  tolerance  along  the   v=vmin  and  v=vmax 
                      boundaries, i.e. for u-knot removal
     evb  , input  ,  Error  tolerance  along  the   u=umin  and  u=umax 
                      boundaries, i.e. for v-knot removal
     eut  , input  ,  Error  tolerance for  u-tangents on the u=umin and
                      u=umax  boundaries. IT  IS  ANGULAR TOLERANCE MEA-
                      SURED IN DEGREES.
     evt  , input  ,  Error  tolerance  for v-tangents on the v=vmin and
                      v=vmax  boundaries.  IT IS  ANGULAR TOLERANCE MEA-
                      SURED IN DEGREES.
     dir  , input  ,  Flag:
                        NL_UDIR : remove all removable u-knots
                        NL_VDIR : remove all removable v-knots
                        NL_UVDIR: remove all removable u- and v-knots
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SrfReparmAndRemoveKnotsKeepBoundaries( NL_SURFACE *surP, NL_FLAG ufl, NL_FLAG vfl, NL_REAL emx, NL_REAL eub, NL_REAL evb, NL_REAL eut, NL_REAL evt, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{

    NL_FLAG error = NL_NO;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Reparametrize and clean */

    if( surP EQ surQ )
    {
        error = N_SrfReparamArcLength( surP, eps, surP, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfRemoveKnotsKeepBoundaries( surP, ufl, vfl, NL_TANGENT, emx, eub, evb, eut, evt, dir, surP, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_SrfReparamArcLength( surP, eps, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfRemoveKnotsKeepBoundaries( surQ, ufl, vfl, NL_TANGENT, emx, eub, evb, eut, evt, dir, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReparmAndRemoveKnotsKeepBoundaries */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine joins two surfaces at a common boundary.  The
     surfaces must already be compatible in the direction of the com-
     boundary (same knots). If the output surface is  initialized  to  
     NULL, memory to store new control points and knots is allocated. 
     A typical calling example is:

       NL_SURFACE    sur1, sur2, surJ;
       NL_REAL       tol;
       NL_STACKS     SG;
       ...
       (define sur1 and sur2, and choose tol);
       ...
       N_SrfInitArrays(&surJ);
       N_SrfJoin(&sur1,&sur2,NL_UDIR,tol,&surJ,&SG);

     If memory is available, surJ is not initialized and this routine
     assumes that memory allocation has been done. However, it checks  
     for the proper amount by  looking  at  the  highest  indexes  in 
     surJ's knot vector and polygon objects. 


   ACCESS:
   
     sur1 , input  ,  First NURBS surface 
     sur2 , input  ,  Second NURBS surface
     dir  , input  ,  Flag:
                        NL_UDIR: The common boundary  is  sur1's  u=umax
                              curve  and  sur2's  u=umin  curve  (the
                              surfaces must already be  compatible in
                              the v-direction). 
                        NL_VDIR: The common boundary  is  sur1's  v=vmax
                              curve  and  sur2's  v=vmin  curve  (the
                              surfaces must already be  compatible in
                              the u-direction). 
     tol  , input  ,  Knot removal tolerance.  The knot corresponding
                      to the merged boundary  has  multiplicity equal 
                      to the degree. Knot removal will  be  attempted
                      using this tolerance
     surJ , output ,  The joined  (merged)  surface.  surJ's  pointer 
                      address may be the same as  sur1  or  sur2  (in
                      place join)
     SG   , input  ,  surJ's memory stack pointer. If sur1 = surJ (or
                      sur2 = surJ),  then  SG  must  also  be  sur1's 
                      (sur2's) stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfJoin( NL_SURFACE *sur1, NL_SURFACE *sur2, NL_FLAG dir, NL_REAL tol, NL_SURFACE *surJ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfJoin");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, n1, n2, nj, m1, m2, mj, r1, r2, rj, s1, s2, sj;

    NL_DEGREE p1, q1, p2, q2, pj, qj;

    NL_SURFACE surA, surB;

    NL_REAL *U1, *U2, *UJ, *V1, *V2, *VJ, dd, w, w1, w2;

    NL_BOOLEAN rat1, rat2;

    NL_CPOINT ** Pw1, ** Pw2, ** PwJ;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur1, &n1, &m1, &Pw1, &p1, &q1, &r1, &s1, &U1, &V1 );
    N_SrfGetCPtsDegreesAndKnots( sur2, &n2, &m2, &Pw2, &p2, &q2, &r2, &s2, &U2, &V2 );

    /* Get rationality of both surfaces */

    rat1 = N_IsSrfRat( sur1 );
    rat2 = N_IsSrfRat( sur2 );

    /* Check for errors, compute indexes, and raise degree if required */

    if( dir EQ NL_UDIR )
    {
        if( m1 NEQ m2 OR q1 NEQ q2 )
            NL_ERROR( NL_INP_ERR );

        if( p1 EQ p2 )
            pj = p1;
        else
        {
            N_SrfInitArrays( &surB );

            if( p1 LT p2 )
            {
                error = N_SrfElevateDegree( sur1, (NL_INDEX)(p2 - p1), NL_UDIR, &surB, SG, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                N_SrfGetCPtsDegreesAndKnots( &surB, &n1, &m1, &Pw1, &pj, &q1, &r1, &s1, &U1, &V1 );
            }
            else
            {
                error = N_SrfElevateDegree( sur2, (NL_INDEX)(p1 - p2), NL_UDIR, &surB, SG, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                N_SrfGetCPtsDegreesAndKnots( &surB, &n2, &m2, &Pw2, &pj, &q2, &r2, &s2, &U2, &V2 );
            }
        }

        nj = n1 + n2;
        rj = nj + pj + 1;
        mj = m1;
        sj = s1;
        qj = q1;
    }
    else
    {
        if( n1 NEQ n2 OR p1 NEQ p2 )
            NL_ERROR( NL_INP_ERR );

        if( q1 EQ q2 )
            qj = q1;
        else
        {
            N_SrfInitArrays( &surB );

            if( q1 LT q2 )
            {
                error = N_SrfElevateDegree( sur1, (NL_INDEX)(q2 - q1), NL_VDIR, &surB, SG, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                N_SrfGetCPtsDegreesAndKnots( &surB, &n1, &m1, &Pw1, &p1, &qj, &r1, &s1, &U1, &V1 );
            }
            else
            {
                error = N_SrfElevateDegree( sur2, (NL_INDEX)(q1 - q2), NL_VDIR, &surB, SG, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                N_SrfGetCPtsDegreesAndKnots( &surB, &n2, &m2, &Pw2, &p2, &qj, &r2, &s2, &U2, &V2 );
            }
        }

        mj = m1 + m2;
        sj = mj + qj + 1;
        nj = n1;
        rj = r1;
        pj = p1;
    }

    /* See if memory is needed for surJ */

    if( sur1 EQ surJ )
    {
        surA = *sur1;

        error = N_AllocSrfArrays( sur1, nj, mj, pj, qj, rj, sj, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( sur1, &PwJ, &UJ, &VJ );
    }
    else if( sur2 EQ surJ )
    {
        surA = *sur2;

        error = N_AllocSrfArrays( sur2, nj, mj, pj, qj, rj, sj, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( sur2, &PwJ, &UJ, &VJ );
    }
    else
    {
        error = N_SrfSizeArrays( surJ, nj, mj, pj, qj, rj, sj, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surJ, &PwJ, &UJ, &VJ );
    }

    /* Load knots and control points */

    if( dir EQ NL_UDIR )
    {
        for ( ii = 0; ii <= s1; ii++ )
            VJ[ii] = V1[ii];

        for ( ii = 0; ii < r1; ii++ )
            UJ[ii] = U1[ii];
        jj = ii;
        dd = U1[r1] - U2[0];

        for ( ii = pj + 1; ii <= r2; ii++ )
            UJ[jj++] = U2[ii] + dd;

        for ( ii = 0; ii <= mj; ii++ )
        {
            for ( jj = 0; jj < n1; jj++ )
                N_CopyCPt( Pw1[jj][ii], &PwJ[jj][ii] );

            N_Combine2CPts( 0.5, Pw1[n1][ii], 0.5, Pw2[0][ii], &PwJ[n1][ii] );

            if( rat1 NEQ rat2 )
            {
                N_CPtGetW( Pw1[n1][ii], &w1 );

                if( w1 EQ NL_NOW )
                    w1 = 1.0;
                N_CPtGetW( Pw2[0][ii], &w2 );

                if( w2 EQ NL_NOW )
                    w2 = 1.0;
                N_CPtSetW( 0.5 *(w1 + w2), &PwJ[n1][ii] );
            }

            kk = n1 + 1;

            for ( jj = 1; jj <= n2; jj++ )
            {
                N_CopyCPt( Pw2[jj][ii], &PwJ[kk][ii] );
                kk += 1;
            }
        }

        /* attempt knot removal */

        error = N_SrfRemoveKnotConditional( surJ, U1[r1], pj, tol, NL_UDIR, &ii, surJ, SG );
        error = NL_NO;
    }
    else
    {
        for ( ii = 0; ii <= r1; ii++ )
            UJ[ii] = U1[ii];

        for ( ii = 0; ii < s1; ii++ )
            VJ[ii] = V1[ii];
        jj = ii;
        dd = V1[s1] - V2[0];

        for ( ii = qj + 1; ii <= s2; ii++ )
            VJ[jj++] = V2[ii] + dd;

        for ( ii = 0; ii <= nj; ii++ )
        {
            for ( jj = 0; jj < m1; jj++ )
                N_CopyCPt( Pw1[ii][jj], &PwJ[ii][jj] );

            N_Combine2CPts( 0.5, Pw1[ii][m1], 0.5, Pw2[ii][0], &PwJ[ii][m1] );

            if( rat1 NEQ rat2 )
            {
                N_CPtGetW( Pw1[ii][m1], &w1 );

                if( w1 EQ NL_NOW )
                    w1 = 1.0;
                N_CPtGetW( Pw2[ii][0], &w2 );

                if( w2 EQ NL_NOW )
                    w2 = 1.0;
                N_CPtSetW( 0.5 *(w1 + w2), &PwJ[ii][m1] );
            }

            kk = m1 + 1;

            for ( jj = 1; jj <= m2; jj++ )
            {
                N_CopyCPt( Pw2[ii][jj], &PwJ[ii][kk] );
                kk += 1;
            }
        }

        /* attempt knot removal */

        error = N_SrfRemoveKnotConditional( surJ, V1[s1], qj, tol, NL_VDIR, &ii, surJ, SG );
        error = NL_NO;
    }

    /* Make sure weights/rationality is consistly set */

    if( (rat1 AND( NOT rat2 ))OR( rat2 AND( NOT rat1 ) ) )
    {
        N_SrfGetCPtsDegreesAndKnots( surJ, &nj, &mj, &PwJ, &pj, &qj, &rj, &sj, &UJ, &VJ );

        for ( ii = 0; ii <= nj; ii++ )
            for ( jj = 0; jj <= mj; jj++ )
            {
                N_CPtGetW( PwJ[ii][jj], &w );

                if( w EQ NL_NOW )
                    N_CPtSetW( 1.0, &PwJ[ii][jj] );
            }
    }

    /* If merge is in place, kill old surface */

    if( sur1 EQ surJ OR sur2 EQ surJ )
        N_FreeSrf( &surA, SG );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfJoin */



/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This tools routine eliminates degenerate patch strips of a  surface.
     The strip [u1,u2] is defined to  be  degenerate if its image surface
     strip between the isocurves S(u1,v) and S(u2,v) has width less  than
     some given tolerance. A similar statement defines a degenerate strip
     between [v1,v2]. If a strip is removed, parameterization of the sur-
     face changes,  however,  the surface will not  change  geometrically
     more than the given tolerance.  If the entire surface is  degenerate
     in one direction,  no patch strips are removed in that direction.  A
     typical calling example is:
 
       NL_SURFACE  surP, surQ;
       NL_REAL     tol;
       NL_INDEX    nu, nv;
       NL_STACKS   SP, SQ;
       ...
       (define surP and choose tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveDegenPatch(&surP,tol,NL_UVDIR,&nu,&nv,&surQ,&SP,&SQ);

     If memory is  available, surQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks
     for the proper amount  by looking at the highest indexes in surQ's  
     knot vector and polygon objects.
 
 
   ACCESS:
   
     surP  , input  ,  NURBS surface
     tol   , input  ,  Tolerance.  A strip is degenerate  if its width is
                       less than tol
     dflg  , input  ,  Flag:
                        NL_UDIR : remove only degenerate u-strips
                        NL_VDIR : remove only degenerate v-strips
                        NL_UVDIR: remove degenerate u- and v-strips
     nu    , output ,  Number of u-strips removed.  If nu = 0,  then surQ
                       is not created (surP is not modified)
     nv    , output ,  Number of v-strips removed.  If nv = 0,  then surQ
                       is not created (surP is not modified)
     surQ  , output ,  Surface after removal of degenerate  strips  (only 
                       if nu > 0 or nv > 0).
     SP    , input  ,  surP's stack
     SQ    , input  ,  surQ's stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_SrfRemoveDegenPatch( NL_SURFACE *surP, NL_REAL tol, NL_FLAG dflg, NL_INDEX *nu, NL_INDEX *nv, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveDegenPatch");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, np, mp, rp, sp, nq, mq, rq, sq, nuu, nvv, mult1, mult2, k1, k2, span1, span2, span3;

    NL_DEGREE p, q;

    NL_REAL dlen, du, u1, u2, d1, d2, d3, *UP, *VP, *UQ, *VQ, ** ustps, ** vstps, dv, v1, v2;

    NL_CPOINT ** Pw, ** Qw;

    NL_SURFACE surA, surB, *surptr;
    N_SrfInitArrays( &surA );

    NL_KNOTVECTOR *knu, *knv;

    NL_CNET *net;

    NL_STACKS SL, *SS;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &np, &mp, &Pw, &p, &q, &rp, &sp, &UP, &VP );

    *nu = 0;
    *nv = 0;

    /* Only check surfaces with at least two strips */

    if( np LE p AND dflg EQ NL_UDIR )
        NL_OUT;

    if( mp LE q AND dflg EQ NL_VDIR )
        NL_OUT;

    if( np LE p AND mp LE q AND dflg EQ NL_UVDIR )
        NL_OUT;

    /* Find degeneracies */

    nuu = nvv = -1;

    error = N_SrfFindDegenPatch( surP, tol, dflg, &ustps, &nuu, &vstps, &nvv, &SL );

    if( error EQ NL_YES OR( nuu LT 0 AND nvv LT 0 ) )
        NL_OUT;

    if( (dflg EQ NL_UDIR OR dflg EQ NL_UVDIR)AND nuu GE 0 )
        if( nuu EQ 0 AND ustps[0][0]EQ UP[0]AND ustps[0][1]EQ UP[rp] )
            nuu = -1;

    if( (dflg EQ NL_VDIR OR dflg EQ NL_UVDIR)AND nvv GE 0 )
        if( nvv EQ 0 AND vstps[0][0]EQ VP[0]AND vstps[0][1]EQ VP[sp] )
            nvv = -1;

    if( nuu LT 0 AND nvv LT 0 )
        NL_OUT;

    /* There are degenerate strips to be removed */

    /* Make copy of surP and work on the copy */

    if( surP EQ surQ )
    {
        surA = *surP;
        N_SrfInitArrays( surP );
        surptr = surP;
        error = N_SrfCopy( &surA, surptr, SP );

        if( error EQ NL_YES )
        {
            *surP = surA;
            NL_OUT;
        }
        SS = SP;
    }
    else if( N_SrfAreArraysNULL( surQ ) )
    {
        surptr = surQ;
        error = N_SrfCopy( surP, surptr, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        SS = SQ;
    }
    else
    {
        surptr = &surB;
        N_SrfInitArrays( surptr );
        error = N_SrfCopy( surP, surptr, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        SS = &SL;
    }

    /* Remove u-strips */

    if( nuu GE 0 )
    {

        dlen = 0.00005 *(UP[rp] - UP[0]);

        /* Make strips to be removed into Beziers and reparameterize */
        /* so that their parameter lengths are small.                */

        ii = 0;

        while( ii LE nuu )
        {
            /* find number of contiguous degenerate strips */

            for ( jj = ii; jj < nuu; jj++ )
                if( ustps[jj][1]NEQ ustps[jj + 1][0] )
                    break;

            /* special case out contiguous strips equal entire surface */

            if( ustps[ii][0]EQ UP[0]AND ustps[jj][1]EQ UP[rp] )
            { /* just remove internal knots */
                error = N_SrfRemoveAllKnots( surptr, tol, NL_UDIR, surptr, SS );

                if( error EQ NL_YES )
                {
                    if( surptr EQ surP )
                        *surP = surA;

                    NL_OUT;
                }
                else
                {
                    *nu = nuu + 1;
                    goto DO_V;
                }
            }

            /* make sure the strip ends are p-multiplicity knots */

            N_SrfGetKnotVectors( surptr, &knu, &knv );
            error = N_BasisFindSpanAndMult( knu, p, ustps[ii][0], NL_LEFT, &span1, &mult1 );

            if( mult1 LT p )
            {
                error = N_SrfInsertKnot( surptr, ustps[ii][0], p - mult1, NL_UDIR, surptr, SS, SS );

                if( error EQ NL_YES )
                {
                    if( surptr EQ surP )
                        *surP = surA;

                    NL_OUT;
                }
                N_SrfGetKnotVectors( surptr, &knu, &knv );
            }

            error = N_BasisFindSpanAndMult( knu, p, ustps[jj][1], NL_LEFT, &span2, &mult2 );

            if( mult2 LT p )
            {
                error = N_SrfInsertKnot( surptr, ustps[jj][1], p - mult2, NL_UDIR, surptr, SS, SS );

                if( error EQ NL_YES )
                {
                    if( surptr EQ surP )
                        *surP = surA;

                    NL_OUT;
                }
                N_SrfGetKnotVectors( surptr, &knu, &knv );
            }

            N_SrfGetKnots( surptr, &rq, &sq, &UQ, &VQ );

            du = ustps[jj][1] - ustps[ii][0];

            if( du GT dlen )               /* reparameterize */
            {
                if( ustps[ii][0]EQ UQ[0] ) /* strip at start */
                {
                    error = N_BasisFindSpan( knu, p, ustps[jj][1], NL_RIGHT, &span2 );
                    span2 += 1;

                    u2 = UQ[0] + dlen;
                    d2 = dlen / du;

                    for ( kk = ii; kk < jj; kk++ )
                    {
                        ustps[kk][1] = d2 * (ustps[kk][1] - UQ[0]) + UQ[0];
                        ustps[kk + 1][0] = d2 * (ustps[kk + 1][0] - UQ[0]) + UQ[0];
                    }

                    for ( kk = p + 1; kk < span2; kk++ )
                        UQ[kk] = d2 * (UQ[kk] - UQ[0]) + UQ[0];

                    d3 = (UQ[rq] - u2) / (UQ[rq] - ustps[jj][1]);

                    for ( kk = jj + 1; kk <= nuu; kk++ )
                    {
                        ustps[kk][0] = d3 * (ustps[kk][0] - ustps[jj][1]) + u2;

                        if( ustps[kk][1]NEQ UQ[rq] )
                            ustps[kk][1] = d3 * (ustps[kk][1] - ustps[jj][1]) + u2;
                    }

                    for ( kk = span2 + p; kk < rq - p; kk++ )
                        UQ[kk] = d3 * (UQ[kk] - UQ[span2]) + u2;

                    ustps[jj][1] = u2;

                    for ( kk = 0; kk < p; kk++ )
                        UQ[span2 + kk] = u2;
                }
                else if( ustps[jj][1]EQ UQ[rq] ) /* strip at end */
                {
                    error = N_BasisFindSpan( knu, p, ustps[ii][0], NL_LEFT, &span1 );

                    u2 = UQ[rq] - dlen;
                    d2 = dlen / du;

                    for ( kk = ii; kk < jj; kk++ )
                    {
                        ustps[kk][1] = d2 * (ustps[kk][1] - UQ[span1]) + u2;
                        ustps[kk + 1][0] = d2 * (ustps[kk + 1][0] - UQ[span1]) + u2;
                    }

                    for ( kk = span1 + 1; kk < rq - p; kk++ )
                        UQ[kk] = d2 * (UQ[kk] - UQ[span1]) + u2;

                    d3 = (u2 - UQ[0]) / (UQ[span1] - UQ[0]);

                    for ( kk = 0; kk < ii; kk++ )
                    {
                        ustps[kk][1] = d3 * (ustps[kk][1] - UQ[0]) + UQ[0];

                        if( ustps[kk][0]NEQ UQ[0] )
                            ustps[kk][0] = d3 * (ustps[kk][0] - UQ[0]) + UQ[0];
                    }

                    for ( kk = p + 1; kk <= span1 - p; kk++ )
                        UQ[kk] = d3 * (UQ[kk] - UQ[0]) + UQ[0];

                    ustps[ii][0] = u2;

                    for ( kk = 0; kk < p; kk++ )
                        UQ[span1 - kk] = u2;
                }
                else
                { /* internal strip */
                    error = N_BasisFindSpan( knu, p, ustps[ii][0], NL_LEFT, &span1 );
                    error = N_BasisFindSpan( knu, p, ustps[jj][1], NL_RIGHT, &span2 );
                    span2 += 1;

                    d1 = 0.5 *(ustps[ii][0] + ustps[jj][1]);
                    u1 = d1 - 0.499 *dlen;
                    u2 = d1 + 0.499 *dlen;
                    d2 = (u2 - u1) / du;

                    for ( kk = ii; kk < jj; kk++ )
                    {
                        ustps[kk][1] = d2 * (ustps[kk][1] - UQ[span1]) + u1;
                        ustps[kk + 1][0] = d2 * (ustps[kk + 1][0] - UQ[span1]) + u1;
                    }

                    for ( kk = span1 + 1; kk < span2; kk++ )
                        UQ[kk] = d2 * (UQ[kk] - UQ[span1]) + u1;

                    d3 = (u1 - UQ[0]) / (UQ[span1] - UQ[0]);

                    for ( kk = 0; kk < ii; kk++ )
                    {
                        ustps[kk][1] = d3 * (ustps[kk][1] - UQ[0]) + UQ[0];

                        if( ustps[kk][0]NEQ UQ[0] )
                            ustps[kk][0] = d3 * (ustps[kk][0] - UQ[0]) + UQ[0];
                    }

                    for ( kk = p + 1; kk <= span1 - p; kk++ )
                        UQ[kk] = d3 * (UQ[kk] - UQ[0]) + UQ[0];

                    ustps[ii][0] = u1;

                    for ( kk = 0; kk < p; kk++ )
                        UQ[span1 - kk] = u1;

                    d3 = (UQ[rq] - u2) / (UQ[rq] - ustps[jj][1]);

                    for ( kk = jj + 1; kk <= nuu; kk++ )
                    {
                        ustps[kk][0] = d3 * (ustps[kk][0] - ustps[jj][1]) + u2;

                        if( ustps[kk][1]NEQ UQ[rq] )
                            ustps[kk][1] = d3 * (ustps[kk][1] - ustps[jj][1]) + u2;
                    }

                    for ( kk = span2 + p; kk < rq - p; kk++ )
                        UQ[kk] = d3 * (UQ[kk] - UQ[span2]) + u2;

                    ustps[jj][1] = u2;

                    for ( kk = 0; kk < p; kk++ )
                        UQ[span2 + kk] = u2;
                }
            }

            ii = jj + 1;
        }

        /* Now remove the degenerate strips. There are  */
        /* two cases: interior strips vs. end strips.   */

        N_SrfGetNetAndKnotVectors( surptr, &net, &p, &q, &knu, &knv );

        ii = 0;

        while( ii LE nuu )
        {
            /* find number of contiguous degenerate strips */

            for ( jj = ii; jj < nuu; jj++ )
                if( ustps[jj][1]NEQ ustps[jj + 1][0] )
                    break;

            /* get local notation */

            N_SrfGetCPtsDegreesAndKnots( surptr, &nq, &mq, &Qw, &p, &q, &rq, &sq, &UQ, &VQ );

            /* now remove the strip */

            if( ustps[ii][0]EQ UP[0] )
            { /* start strip */

                error = N_BasisFindSpan( knu, p, ustps[jj][1], NL_RIGHT, &span2 );

                k1 = p + 1;

                for ( kk = span2 + p + 1; kk <= rq; kk++ )
                    UQ[k1++] = UQ[kk];
                N_KnotVectorFromRealArray( knu, UQ, k1 - 1 );

                for ( k2 = 0; k2 <= mq; k2++ )
                {
                    k1 = 1;

                    for ( kk = span2 + 1; kk <= nq; kk++ )
                        N_CopyCPt( Qw[kk][k2], &Qw[k1++][k2] );
                }
                N_CNetFromCPts( net, Qw, k1 - 1, mq );
            }
            else if( ustps[jj][1]EQ UP[rp] )
            { /* end strip */

                error = N_BasisFindSpan( knu, p, ustps[ii][0], NL_LEFT, &span1 );
                span1 += 1;

                k1 = span1 - p;

                for ( kk = 0; kk <= p; kk++ )
                    UQ[k1++] = UQ[rq];
                N_KnotVectorFromRealArray( knu, UQ, span1 );

                for ( k2 = 0; k2 <= mq; k2++ )
                    N_CopyCPt( Qw[nq][k2], &Qw[nq - rq + span1][k2] );
                N_CNetFromCPts( net, Qw, nq - rq + span1, mq );
            }
            else
            { /* interior strip */

                u2 = 0.5 *(ustps[ii][0] + ustps[jj][1]);
                error = N_BasisFindSpanAndMult( knu, p, u2, NL_LEFT, &span2, &mult2 );

                if( mult2 LT p )
                {
                    error = N_SrfInsertKnot( surptr, u2, p - mult2, NL_UDIR, surptr, SS, SS );

                    if( error EQ NL_YES )
                    {
                        if( surptr EQ surP )
                            *surP = surA;

                        NL_OUT;
                    }

                    N_SrfGetNetAndKnotVectors( surptr, &net, &p, &q, &knu, &knv );
                    N_SrfGetCPtsDegreesAndKnots( surptr, &nq, &mq, &Qw, &p, &q, &rq, &sq, &UQ, &VQ );
                }

                error = N_BasisFindSpan( knu, p, ustps[ii][0], NL_LEFT, &span1 );
                error = N_BasisFindSpan( knu, p, u2, NL_LEFT, &span2 );
                error = N_BasisFindSpan( knu, p, ustps[jj][1], NL_LEFT, &span3 );

                k1 = span1 - p + 1;

                for ( kk = 0; kk < p; kk++ )
                    UQ[k1++] = u2;

                for ( kk = span3 + 1; kk <= rq; kk++ )
                    UQ[k1++] = UQ[kk];
                N_KnotVectorFromRealArray( knu, UQ, k1 - 1 );

                for ( k2 = 0; k2 <= mq; k2++ )
                {
                    k1 = span1 - p;
                    N_CopyCPt( Qw[span2 - p][k2], &Qw[k1++][k2] );

                    for ( kk = span3 - p + 1; kk <= nq; kk++ )
                        N_CopyCPt( Qw[kk][k2], &Qw[k1++][k2] );
                }
                N_CNetFromCPts( net, Qw, k1 - 1, mq );
            }

            ii = jj + 1;
        }
    }

    *nu = nuu + 1;

    DO_V:

    /* Remove v-strips */

    if( nvv GE 0 )
    {
        dlen = 0.00005 *(VP[sp] - VP[0]);

        /* Make strips to be removed into Beziers and reparameterize */
        /* so that their parameter lengths are small.                */

        ii = 0;

        while( ii LE nvv )
        {
            /* find number of contiguous degenerate strips */

            for ( jj = ii; jj < nvv; jj++ )
                if( vstps[jj][1]NEQ vstps[jj + 1][0] )
                    break;

            /* special case out contiguous strips equal entire surface */

            if( vstps[ii][0]EQ VP[0]AND vstps[jj][1]EQ VP[sp] )
            { /* just remove internal knots */
                error = N_SrfRemoveAllKnots( surptr, tol, NL_VDIR, surptr, SS );

                if( error EQ NL_YES )
                {
                    if( surptr EQ surP )
                        *surP = surA;

                    NL_OUT;
                }
                else
                {
                    *nv = nvv + 1;
                    goto WRAP_UP;
                }
            }

            /* make sure the strip ends are q-multiplicity knots */

            N_SrfGetKnotVectors( surptr, &knu, &knv );
            error = N_BasisFindSpanAndMult( knv, q, vstps[ii][0], NL_LEFT, &span1, &mult1 );

            if( mult1 LT q )
            {
                error = N_SrfInsertKnot( surptr, vstps[ii][0], q - mult1, NL_VDIR, surptr, SS, SS );

                if( error EQ NL_YES )
                {
                    if( surptr EQ surP )
                        *surP = surA;

                    NL_OUT;
                }
                N_SrfGetKnotVectors( surptr, &knu, &knv );
            }

            error = N_BasisFindSpanAndMult( knv, q, vstps[jj][1], NL_LEFT, &span2, &mult2 );

            if( mult2 LT q )
            {
                error = N_SrfInsertKnot( surptr, vstps[jj][1], q - mult2, NL_VDIR, surptr, SS, SS );

                if( error EQ NL_YES )
                {
                    if( surptr EQ surP )
                        *surP = surA;

                    NL_OUT;
                }
                N_SrfGetKnotVectors( surptr, &knu, &knv );
            }

            N_SrfGetKnots( surptr, &rq, &sq, &UQ, &VQ );

            dv = vstps[jj][1] - vstps[ii][0];

            if( dv GT dlen )               /* reparameterize */
            {
                if( vstps[ii][0]EQ VQ[0] ) /* strip at start */
                {
                    error = N_BasisFindSpan( knv, q, vstps[jj][1], NL_RIGHT, &span2 );
                    span2 += 1;

                    v2 = VQ[0] + dlen;
                    d2 = dlen / dv;

                    for ( kk = ii; kk < jj; kk++ )
                    {
                        vstps[kk][1] = d2 * (vstps[kk][1] - VQ[0]) + VQ[0];
                        vstps[kk + 1][0] = d2 * (vstps[kk + 1][0] - VQ[0]) + VQ[0];
                    }

                    for ( kk = p + 1; kk < span2; kk++ )
                        VQ[kk] = d2 * (VQ[kk] - VQ[0]) + VQ[0];

                    d3 = (VQ[sq] - v2) / (VQ[sq] - vstps[jj][1]);

                    for ( kk = jj + 1; kk <= nvv; kk++ )
                    {
                        vstps[kk][0] = d3 * (vstps[kk][0] - vstps[jj][1]) + v2;

                        if( vstps[kk][1]NEQ VQ[sq] )
                            vstps[kk][1] = d3 * (vstps[kk][1] - vstps[jj][1]) + v2;
                    }

                    for ( kk = span2 + q; kk < sq - q; kk++ )
                        VQ[kk] = d3 * (VQ[kk] - VQ[span2]) + v2;

                    vstps[jj][1] = v2;

                    for ( kk = 0; kk < q; kk++ )
                        VQ[span2 + kk] = v2;
                }
                else if( vstps[jj][1]EQ VQ[sq] ) /* strip at end */
                {
                    error = N_BasisFindSpan( knv, q, vstps[ii][0], NL_LEFT, &span1 );

                    v2 = VQ[sq] - dlen;
                    d2 = dlen / dv;

                    for ( kk = ii; kk < jj; kk++ )
                    {
                        vstps[kk][1] = d2 * (vstps[kk][1] - VQ[span1]) + v2;
                        vstps[kk + 1][0] = d2 * (vstps[kk + 1][0] - VQ[span1]) + v2;
                    }

                    for ( kk = span1 + 1; kk < sq - q; kk++ )
                        VQ[kk] = d2 * (VQ[kk] - VQ[span1]) + v2;

                    d3 = (v2 - VQ[0]) / (VQ[span1] - VQ[0]);

                    for ( kk = 0; kk < ii; kk++ )
                    {
                        vstps[kk][1] = d3 * (vstps[kk][1] - VQ[0]) + VQ[0];

                        if( vstps[kk][0]NEQ VQ[0] )
                            vstps[kk][0] = d3 * (vstps[kk][0] - VQ[0]) + VQ[0];
                    }

                    for ( kk = q + 1; kk <= span1 - q; kk++ )
                        VQ[kk] = d3 * (VQ[kk] - VQ[0]) + VQ[0];

                    vstps[ii][0] = v2;

                    for ( kk = 0; kk < q; kk++ )
                        VQ[span1 - kk] = v2;
                }
                else
                { /* internal strip */
                    error = N_BasisFindSpan( knv, q, vstps[ii][0], NL_LEFT, &span1 );
                    error = N_BasisFindSpan( knv, q, vstps[jj][1], NL_RIGHT, &span2 );
                    span2 += 1;

                    d1 = 0.5 *(vstps[ii][0] + vstps[jj][1]);
                    v1 = d1 - 0.499 *dlen;
                    v2 = d1 + 0.499 *dlen;
                    d2 = (v2 - v1) / dv;

                    for ( kk = ii; kk < jj; kk++ )
                    {
                        vstps[kk][1] = d2 * (vstps[kk][1] - VQ[span1]) + v1;
                        vstps[kk + 1][0] = d2 * (vstps[kk + 1][0] - VQ[span1]) + v1;
                    }

                    for ( kk = span1 + 1; kk < span2; kk++ )
                        VQ[kk] = d2 * (VQ[kk] - VQ[span1]) + v1;

                    d3 = (v1 - VQ[0]) / (VQ[span1] - VQ[0]);

                    for ( kk = 0; kk < ii; kk++ )
                    {
                        vstps[kk][1] = d3 * (vstps[kk][1] - VQ[0]) + VQ[0];

                        if( vstps[kk][0]NEQ VQ[0] )
                            vstps[kk][0] = d3 * (vstps[kk][0] - VQ[0]) + VQ[0];
                    }

                    for ( kk = q + 1; kk <= span1 - q; kk++ )
                        VQ[kk] = d3 * (VQ[kk] - VQ[0]) + VQ[0];

                    vstps[ii][0] = v1;

                    for ( kk = 0; kk < q; kk++ )
                        VQ[span1 - kk] = v1;

                    d3 = (VQ[sq] - v2) / (VQ[sq] - vstps[jj][1]);

                    for ( kk = jj + 1; kk <= nvv; kk++ )
                    {
                        vstps[kk][0] = d3 * (vstps[kk][0] - vstps[jj][1]) + v2;

                        if( vstps[kk][1]NEQ VQ[sq] )
                            vstps[kk][1] = d3 * (vstps[kk][1] - vstps[jj][1]) + v2;
                    }

                    for ( kk = span2 + q; kk < sq - q; kk++ )
                        VQ[kk] = d3 * (VQ[kk] - VQ[span2]) + v2;

                    vstps[jj][1] = v2;

                    for ( kk = 0; kk < q; kk++ )
                        VQ[span2 + kk] = v2;
                }
            }

            ii = jj + 1;
        }

        /* Now remove the degenerate strips. There are  */
        /* two cases: interior strips vs. end strips.   */

        N_SrfGetNetAndKnotVectors( surptr, &net, &p, &q, &knu, &knv );

        ii = 0;

        while( ii LE nvv )
        {
            /* find number of contiguous degenerate strips */

            for ( jj = ii; jj < nvv; jj++ )
                if( vstps[jj][1]NEQ vstps[jj + 1][0] )
                    break;

            /* get local notation */

            N_SrfGetCPtsDegreesAndKnots( surptr, &nq, &mq, &Qw, &p, &q, &rq, &sq, &UQ, &VQ );

            /* now remove the strip */

            if( vstps[ii][0]EQ VP[0] )
            { /* start strip */

                error = N_BasisFindSpan( knv, q, vstps[jj][1], NL_RIGHT, &span2 );

                k1 = q + 1;

                for ( kk = span2 + q + 1; kk <= sq; kk++ )
                    VQ[k1++] = VQ[kk];
                N_KnotVectorFromRealArray( knv, VQ, k1 - 1 );

                for ( k2 = 0; k2 <= nq; k2++ )
                {
                    k1 = 1;

                    for ( kk = span2 + 1; kk <= mq; kk++ )
                        N_CopyCPt( Qw[k2][kk], &Qw[k2][k1++] );
                }
                N_CNetFromCPts( net, Qw, nq, k1 - 1 );
            }
            else if( vstps[jj][1]EQ VP[sp] )
            { /* end strip */

                error = N_BasisFindSpan( knv, q, vstps[ii][0], NL_LEFT, &span1 );
                span1 += 1;

                k1 = span1 - q;

                for ( kk = 0; kk <= q; kk++ )
                    VQ[k1++] = VQ[sq];
                N_KnotVectorFromRealArray( knv, VQ, span1 );

                for ( k2 = 0; k2 <= nq; k2++ )
                    N_CopyCPt( Qw[k2][mq], &Qw[k2][mq - sq + span1] );
                N_CNetFromCPts( net, Qw, nq, mq - sq + span1 );
            }
            else
            { /* interior strip */

                v2 = 0.5 *(vstps[ii][0] + vstps[jj][1]);
                error = N_BasisFindSpanAndMult( knv, q, v2, NL_LEFT, &span2, &mult2 );

                if( mult2 LT q )
                {
                    error = N_SrfInsertKnot( surptr, v2, q - mult2, NL_VDIR, surptr, SS, SS );

                    if( error EQ NL_YES )
                    {
                        if( surptr EQ surP )
                            *surP = surA;

                        NL_OUT;
                    }

                    N_SrfGetNetAndKnotVectors( surptr, &net, &p, &q, &knu, &knv );
                    N_SrfGetCPtsDegreesAndKnots( surptr, &nq, &mq, &Qw, &p, &q, &rq, &sq, &UQ, &VQ );
                }

                error = N_BasisFindSpan( knv, q, vstps[ii][0], NL_LEFT, &span1 );
                error = N_BasisFindSpan( knv, q, v2, NL_LEFT, &span2 );
                error = N_BasisFindSpan( knv, q, vstps[jj][1], NL_LEFT, &span3 );

                k1 = span1 - q + 1;

                for ( kk = 0; kk < q; kk++ )
                    VQ[k1++] = v2;

                for ( kk = span3 + 1; kk <= sq; kk++ )
                    VQ[k1++] = VQ[kk];
                N_KnotVectorFromRealArray( knv, VQ, k1 - 1 );

                for ( k2 = 0; k2 <= nq; k2++ )
                {
                    k1 = span1 - q;
                    N_CopyCPt( Qw[k2][span2 - q], &Qw[k2][k1++] );

                    for ( kk = span3 - q + 1; kk <= mq; kk++ )
                        N_CopyCPt( Qw[k2][kk], &Qw[k2][k1++] );
                }
                N_CNetFromCPts( net, Qw, nq, k1 - 1 );
            }

            ii = jj + 1;
        }
    }

    *nv = nvv + 1;

    WRAP_UP:

    /* Compact surface */

    N_SrfCompress( surptr, SS );

    if( error EQ NL_YES )
    {
        if( surptr EQ surP )
            *surP = surA;
        NL_OUT;
    }

    /* If removal is in place, kill old surface */

    if( surP EQ surQ )
        N_FreeSrf( &surA, SP );

    /* Copy result to surQ if memory was already there */

    if( surptr EQ & surB )
    {
        N_SrfGetArraySizes( surptr, &nq, &mq, &rq, &sq );
        error = N_SrfSizeArrays( surQ, nq, mq, p, q, rq, sq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_SrfCopy( surptr, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveDegenPatch */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine  extracts the four boundary curves from a  NURBS 
     surface. If the  output  curves  are initialized to NULL, memory to 
     store  curve  control  points  and  knots is  allocated. A  typical 
     calling example is: 

       NL_SURFACE  sur;
       NL_CURVE    curL, curR, curB, curT;
       NL_STACKS   SC;
       ...
       (define sur);
       ...
       N_CrvInitArrays(&curL);
       N_CrvInitArrays(&curR);
       N_CrvInitArrays(&curB); N_CrvInitArrays(&curT);
       N_SrfExtractBoundaryCrvs(&sur,&curL,&curR,&curB,&curT,&SC);

     If memory is available, the four curves are not initialized and the 
     routine assumes that memory allocations have been done. However, it  
     checks  for the proper  amount by looking at the highest indexes in 
     the knot vector and polygon objects. 


   ACCESS:
   
     sur  , input  ,  NURBS surface
     curL , output ,  Left boundary at u=umin
     curR , output ,  Right boundary at u=umax
     curB , output ,  Bottom boundary at v=vmin
     curT , output ,  Top boundary at v=vmax
     SC   , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfExtractBoundaryCrvs( NL_SURFACE *sur, NL_CURVE *curL, NL_CURVE *curR, NL_CURVE *curB, NL_CURVE *curT, NL_STACKS *SC )
{

    NL_FLAG error = NL_NO;

    NL_INDEX r, s;

    NL_REAL *U, *V;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and extract boundaries */

    N_SrfGetKnots( sur, &r, &s, &U, &V );

    error = N_SrfExtractIsoCrv( sur, U[0], NL_VDIR, curL, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfExtractIsoCrv( sur, U[r], NL_VDIR, curR, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfExtractIsoCrv( sur, V[0], NL_UDIR, curB, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfExtractIsoCrv( sur, V[s], NL_UDIR, curT, SC );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfExtractBoundaryCrvs */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes  one knot  multiple times from a  NURBS 
     surface  either  in  u- or  in  v-direction. The  knot  must be an 
     interior knot. IT IS ASSUMED THAT THE KNOT IS PRECISELY REMOVABLE,
     I.E. NL_NO NL_ERROR  CHECKING IS  PERFORMED. If  the  output  surface is  
     initialized to NULL, memory to store new control points  and knots  
     is allocated. A typical calling example is:

       NL_SURFACE    surP, surQ;
       NL_PARAMETER  t;
       NL_INDEX      nt;
       NL_STACKS     SQ;
       ...
       (define surP, get t and nt);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveKnotMultiple(&surP,t,nt,NL_UDIR,&surQ,&SQ);
       N_SrfRemoveKnotMultiple(&surP,t,nt,NL_VDIR,&surP,&SQ);

     If memory is  available, surQ is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by  looking at the highest indexes in surQ's 
     knot vector and polygon  objects. If surP is the same as surQ, the
     removal is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     t,nt , input  ,  Knot "t"  to be  removed "nt"  times (nt  must be 
                      less  than or  equal to the  multiplicity  of the 
                      knot)
     dir  , input  ,  Flag:
                        NL_UDIR: Remove in u-direction
                        NL_VDIR: Remove in v-direction
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveKnotMultiple( NL_SURFACE *surP, NL_PARAMETER t, NL_INDEX nt, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveKnotMultiple");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, row, col, ii, jj, first, last, off, n, m, r, s, a, spn, mlt, fout;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetKnotVectors( surP, &knu, &knv );

    /* Check parameters */

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spn, &mlt );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spn, &mlt );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    if( nt GT mlt )
        NL_ERROR( NL_KNT_ERR );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Allocate local memory */

    a = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * a, &SL );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * a, &SL );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( 2 * a, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Initialize */

    if( surP NEQ surQ )
    {
        for ( row = 0; row <= n; row++ )
        {
            for ( col = 0; col <= m; col++ )
            {
                N_CopyCPt( Pw[row][col], &Qw[row][col] );
            }
        }

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= s; j++ )
            VQ[j] = VP[j];
    }

    /* Remove in u-direction */

    if( dir EQ NL_UDIR )
    {
        fout = (2 * spn - mlt - p) / 2;
        first = spn - p;
        last = spn - mlt;

        for ( k = 0; k < nt; k++ )
        {
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT k )
            {
                alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (t - UQ[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (UQ[j + p - k + 1] - UQ[j - k]) / (UQ[j + p - k + 1] - t);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            /* Remove one knot along each row */

            for ( col = 0; col <= m; col++ )
            {

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[off][col], &Rw[0] );
                N_CopyCPt( Qw[last + 1][col], &Rw[last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT k )
                {
                    N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[ii - 1], &Rw[ii] );
                    N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Save new control points */

                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    N_CopyCPt( Rw[i - off], &Qw[i][col] );
                    N_CopyCPt( Rw[j - off], &Qw[j][col] );
                    i++;
                    j--;
                }
            }

            /* Shift down knots */

            for ( l = spn - k; l <= r - k - 1; l++ )
                UQ[l] = UQ[l + 1];

            first--;
            last++;
        }

        /* Shift down control points */

        j = fout;
        i = j;

        for ( l = 1; l < k; l++ )
        {
            if( l % 2 )
                i++;
            else
                j--;
        }

        for ( col = 0; col <= m; col++ )
        {
            a = j;

            for ( l = i + 1; l <= n; l++ )
            {
                N_CopyCPt( Qw[l][col], &Qw[a][col] );
                a++;
            }
        }

        N_SrfSetSizeIndices( surQ, n - nt, m, p, q, r - nt, s );
    }

    /* Remove in v-direction */

    if( dir EQ NL_VDIR )
    {
        fout = (2 * spn - mlt - q) / 2;
        first = spn - q;
        last = spn - mlt;

        /* Remove the knot "nt" times */

        for ( k = 0; k < nt; k++ )
        {
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT k )
            {
                alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (t - VQ[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (VQ[j + q - k + 1] - VQ[j - k]) / (VQ[j + q - k + 1] - t);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            /* Remove one knot along each column */

            for ( row = 0; row <= n; row++ )
            {

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[row][off], &Rw[0] );
                N_CopyCPt( Qw[row][last + 1], &Rw[last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT k )
                {
                    N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[ii - 1], &Rw[ii] );
                    N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Save new control points */

                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    N_CopyCPt( Rw[i - off], &Qw[row][i] );
                    N_CopyCPt( Rw[j - off], &Qw[row][j] );
                    i++;
                    j--;
                }
            }

            /* Shift down knots */

            for ( l = spn - k; l <= s - k - 1; l++ )
                VQ[l] = VQ[l + 1];

            first--;
            last++;
        }

        /* Shift down control points */

        j = fout;
        i = j;

        for ( l = 1; l < k; l++ )
        {
            if( l % 2 )
                i++;
            else
                j--;
        }

        for ( row = 0; row <= n; row++ )
        {
            a = j;

            for ( l = i + 1; l <= m; l++ )
            {
                N_CopyCPt( Qw[row][l], &Qw[row][a] );
                a++;
            }
        }

        N_SrfSetSizeIndices( surQ, n, m - nt, p, q, r, s - nt );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveKnotMultiple */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  removes  all removable  knots  from a NURBS
     surface. If the output surface is initialized to NULL, memory  to  
     store  new  control  points  and  knots  is  allocated. A typical 
     calling example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     tol;
       NL_STACKS   SQ;
       ...
       (define surP, get tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveKnots(&surP,tol,NL_UDIR ,&surQ,&SQ);
       N_SrfRemoveKnots(&surP,tol,NL_UVDIR,&surP,&SQ);

     If memory is  available, surQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in surQ's 
     knot vector and polygon objects. If surP is the same as surQ, the
     removal is done in place. THIS  VERSION USES AN  APPROXIMATION OF
     THE PRECISE NL_ERROR TOLERANCE. THAT IS, IT IS FASTER THAN N_SrfRemoveAllKnots,
     HOWEVER, IT REMOVES LESS KNOTS.


   ACCESS:
   
     surP , input  ,  NURBS surface
     tol  , input  ,  Tolerance to check removability
     dir  , input  ,  Flag:
                        NL_UDIR : remove all removable u-knots
                        NL_VDIR : remove all removable v-knots
                        NL_UVDIR: remove all removable u- and v-knots
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveKnots( NL_SURFACE *surP, NL_REAL tol, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveKnots");

    NL_FLAG krm, rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sru = NULL, *srv = NULL, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru = 0, su = 0, rv = 0, sv = 0, ns, ms;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb, *bru = NULL, *brv = NULL, lam = 0.0, oml = 0.0, wmin, wmax, pmax, tmp, al, be, bu = 0.0, bv = 0.0, wi, wj;

    NL_CPOINT ** Pw, ** Qw, ** Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    ns = n;
    ms = m;

    /* Adjust removal tolerance in case of rational surfaces */

    if( N_IsSrfRat( surP ) )
    {
        N_SrfMinMaxWeightPosVectors( surP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

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

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        bru = N_AllocReal1dArray( r, &SL );

        if( bru EQ NULL )
            NL_QUIT;

        sru = N_AllocInt1dArray( r, &SL );

        if( sru EQ NULL )
            NL_QUIT;
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        brv = N_AllocReal1dArray( s, &SL );

        if( brv EQ NULL )
            NL_QUIT;

        srv = N_AllocInt1dArray( s, &SL );

        if( srv EQ NULL )
            NL_QUIT;
    }

    /* Initialize */

    if( surP NEQ surQ )
    {
        error = N_SrfCopy( surP, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= r; i++ )
    {
        for ( j = 0; j <= s; j++ )
            er[i][j] = 0.0;
    }

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        for ( i = 0; i <= r; i++ )
        {
            bru[i] = NL_BIGD;
            sru[i] = 0;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        for ( j = 0; j <= s; j++ )
        {
            brv[j] = NL_BIGD;
            srv[j] = 0;
        }
    }

    /* Compute the maximum of knot removal errors for each distinct knot */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        ru = p + 1;

        while( ru LE n )
        {
            i = ru;

            while( ru LE n AND UQ[ru]EQ UQ[ru + 1] )
                ru++;
            sru[ru] = ru - i + 1;

            error = N_SrfRemoveOneKnot( surP, ru, sru[ru], 0, m, NL_UDIR, &bru[ru] );

            if( error EQ NL_YES )
                NL_OUT;

            ru++;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        rv = q + 1;

        while( rv LE m )
        {
            i = rv;

            while( rv LE m AND VQ[rv]EQ VQ[rv + 1] )
                rv++;
            srv[rv] = rv - i + 1;

            error = N_SrfRemoveOneKnot( surP, rv, srv[rv], 0, n, NL_VDIR, &brv[rv] );

            if( error EQ NL_YES )
                NL_OUT;

            rv++;
        }
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
        {
            bu = bru[p + 1];
            su = sru[p + 1];
            ru = p + 1;

            for ( i = p + 2; i <= r - p - 1; i++ )
            {
                if( bru[i]LT bu )
                {
                    bu = bru[i];
                    su = sru[i];
                    ru = i;
                }
            }
        }

        if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
        {
            bv = brv[q + 1];
            sv = srv[q + 1];
            rv = q + 1;

            for ( j = q + 2; j <= s - q - 1; j++ )
            {
                if( brv[j]LT bv )
                {
                    bv = brv[j];
                    sv = srv[j];
                    rv = j;
                }
            }
        }

        /* If no more removable knot -> finished */

        if( dir EQ NL_UDIR )
        {
            if( bu EQ NL_BIGD OR bu EQ NOREM )
                break;
        }
        else if( dir EQ NL_VDIR )
        {
            if( bv EQ NL_BIGD OR bv EQ NOREM )
                break;
        }
        else if( dir EQ NL_UVDIR )
        {
            if( (bu EQ NL_BIGD OR bu EQ NOREM)AND( bv EQ NL_BIGD OR bv EQ NOREM ) )
                break;
        }

        if( dir EQ NL_UVDIR )
        {
            if( bu LT bv )
                krm = NL_UDIR;
            else
                krm = NL_VDIR;
        }
        else
        {
            krm = dir;
        }

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR: /* Remove in the u-direction */
                if( (p + su) % 2 )
                {
                    k = (p + su + 1) / 2;
                    l = ru - k + p + 1;
                    al = (UQ[ru] - UQ[ru - k]) / (UQ[ru - k + p + 1] - UQ[ru - k]);
                    be = (UQ[ru] - UQ[ru - k + 1]) / (UQ[ru - k + p + 2] - UQ[ru - k + 1]);
                    lam = al / (al + be);
                    oml = 1.0 - lam;
                }
                else
                {
                    k = (p + su) / 2;
                    l = ru - k + p;
                }

                /* Check the error */

                rmf = NL_TRUE;

                for ( i = ru - k; i <= l; i++ )
                {
                    if( UQ[i]NEQ UQ[i + 1] )
                    {
                        for ( j = q; j <= s - q - 1; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                            {
                                te[i][j] = er[i][j] + bu;

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
                            if( UQ[i]NEQ UQ[i + 1]AND VQ[j]NEQ VQ[j + 1] )
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
                        alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (UQ[ru] - UQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[ru]);
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

                        N_CopyCPt( Qw[off][col], &Rw[col][0] );
                        N_CopyCPt( Qw[last + 1][col], &Rw[col][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[col][ii - 1], &Rw[col][ii] );
                            N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[col][jj + 1], &Rw[col][jj] );
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
                            N_Combine2CPts( lam, Rw[col][jj + 1], oml, Rw[col][ii - 1], &Rw[col][jj + 1] );
                        }
                    } /* End for each row */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        bru[ru] = NOREM;
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
                                N_CopyCPt( Rw[col][i - off], &Qw[i][col] );
                                N_CopyCPt( Rw[col][j - off], &Qw[j][col] );
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
                            if( VQ[j]NEQ VQ[j + 1] )
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
                        UQ[i - 1] = UQ[i];

                        for ( j = q; j <= m; j++ )
                            er[i - 1][j] = er[i][j];
                    }

                    for ( col = 0; col <= m; col++ )
                    {
                        for ( i = fout + 1; i <= n; i++ )
                        {
                            N_CopyCPt( Qw[i][col], &Qw[i - 1][col] );
                        }
                    }

                    n--;
                    r--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_UDIR )
                    {
                        if( n EQ p )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( ru - p, p + 1 );
                    l = NL_MIN( n, ru + p - su );

                    for ( i = k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1]AND bru[i]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, i, sru[i], 0, m, NL_UDIR, &bru[i] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( j = q + 1; j <= s - q - 1; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1]AND brv[j]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, j, srv[j], first, last, NL_VDIR, &brv[j] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    bru[ru] = NOREM;
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */
                if( (q + sv) % 2 )
                {
                    k = (q + sv + 1) / 2;
                    l = rv - k + q + 1;
                    al = (VQ[rv] - VQ[rv - k]) / (VQ[rv - k + q + 1] - VQ[rv - k]);
                    be = (VQ[rv] - VQ[rv - k + 1]) / (VQ[rv - k + q + 2] - VQ[rv - k + 1]);
                    lam = al / (al + be);
                    oml = 1.0 - lam;
                }
                else
                {
                    k = (q + sv) / 2;
                    l = rv - k + q;
                }

                /* Check the error */

                rmf = NL_TRUE;

                for ( j = rv - k; j <= l; j++ )
                {
                    if( VQ[j]NEQ VQ[j + 1] )
                    {
                        for ( i = p; i <= r - p - 1; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                            {
                                te[i][j] = er[i][j] + bv;

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
                            if( VQ[j]NEQ VQ[j + 1]AND UQ[i]NEQ UQ[i + 1] )
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
                        alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (VQ[rv] - VQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (VQ[j + q + 1] - VQ[j]) / (VQ[j + q + 1] - VQ[rv]);
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

                        N_CopyCPt( Qw[row][off], &Rw[row][0] );
                        N_CopyCPt( Qw[row][last + 1], &Rw[row][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[row][ii - 1], &Rw[row][ii] );
                            N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[row][jj + 1], &Rw[row][jj] );
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

                        /* Save new control points */

                        if( (q + sv) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[row][jj + 1], oml, Rw[row][ii - 1], &Rw[row][jj + 1] );
                        }
                    } /* End for each column */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        brv[rv] = NOREM;
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
                                N_CopyCPt( Rw[row][i - off], &Qw[row][i] );
                                N_CopyCPt( Rw[row][j - off], &Qw[row][j] );
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
                            if( UQ[i]NEQ UQ[i + 1] )
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
                        VQ[j - 1] = VQ[j];

                        for ( i = p; i <= n; i++ )
                            er[i][j - 1] = er[i][j];
                    }

                    for ( row = 0; row <= n; row++ )
                    {
                        for ( j = fout + 1; j <= m; j++ )
                        {
                            N_CopyCPt( Qw[row][j], &Qw[row][j - 1] );
                        }
                    }

                    m--;
                    s--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_VDIR )
                    {
                        if( m EQ q )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( rv - q, q + 1 );
                    l = NL_MIN( m, rv + q - sv );

                    for ( j = k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1]AND brv[j]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, j, srv[j], 0, n, NL_VDIR, &brv[j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( i = p + 1; i <= r - p - 1; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1]AND bru[i]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, i, sru[i], first, last, NL_UDIR, &bru[i] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    brv[rv] = NOREM;
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* Compact surface */

    if( n LT ns OR m LT ms )
    {
        error = N_SrfCompress( surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveKnots */

#endif // NLIB_UNUSED

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This tools routine removes one knot  one time from a NURBS surface  
     either in u- or in v-direction to  compute derivative changes. The
     knot is removed without error check and only the first two rows or
     columns of control points are computed. A typical calling example:
 
       NL_SURFACE  surP, surQ;
       NL_INDEX    rr, ss;
       ...
       (define surP);
       ...
       N_SrfRemoveKnot(&surP,rr,ss,NL_UDIR,NL_START,NL_END,&surQ);
 
     IT IS ASSUMED  THAT MEMORY FOR  surQ IS  ALLOCATED IN  THE CALLING
     ROUTINE.
 
 
   ACCESS:
   
     surP  , input  ,  NURBS surface
     rr,ss , input  ,  Index and multiplicity of knot to be removed
     dir   , input  ,  Flag:
                         NL_UDIR: Remove in u-direction
                         NL_VDIR: Remove in v-direction
     ufl   , input  ,  Flag:
                         NL_START: remove for u=umin cross-derivative
                         NL_END  : remove for u=umax cross-derivative
                         NL_BOTH : remove for both cross-derivative
     vfl   , input  ,  Flag:
                         NL_START: remove for v=vmin cross-derivative
                         NL_END  : remove for v=vmax cross-derivative
                         NL_BOTH : remove for both cross-derivative
     surQ  , output ,  Surface after knot removal
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SrfRemoveKnot( NL_SURFACE *surP, NL_INDEX rr, NL_INDEX ss, NL_FLAG dir, NL_FLAG ufl, NL_FLAG vfl, NL_SURFACE *surQ )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, row, col, ii, jj, first, last, off, n, m, r, s, fout;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb, lam = 0.0, oml = 0.0;

    NL_CPOINT ** Pw, ** Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );

    /* Allocate local memory */

    i = NL_MAX( p, q );

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

    Rw = N_AllocCPt1dArray( 2 * i, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Initialize */

    switch( dir )
    {
        case NL_UDIR:
            if( vfl EQ NL_START OR vfl EQ NL_BOTH )
            {
                for ( j = 0; j <= 1; j++ )
                {
                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }

            if( vfl EQ NL_END OR vfl EQ NL_BOTH )
            {
                for ( j = m - 1; j <= m; j++ )
                {
                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }

            if( rr EQ p + 1 AND( ufl EQ NL_START OR ufl EQ NL_BOTH ) )
            {
                if( p EQ 2 )
                    ii = 3;
                else
                    ii = 1;

                for ( j = 0; j <= m; j++ )
                {
                    for ( i = 0; i <= ii; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }

            if( rr EQ n AND( ufl EQ NL_END OR ufl EQ NL_BOTH ) )
            {
                if( p EQ 2 )
                    ii = n - 3;
                else
                    ii = n - 1;

                for ( j = 0; j <= m; j++ )
                {
                    for ( i = ii; i <= n; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }
            break;

        case NL_VDIR:
            if( ufl EQ NL_START OR ufl EQ NL_BOTH )
            {
                for ( j = 0; j <= m; j++ )
                {
                    for ( i = 0; i <= 1; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }

            if( ufl EQ NL_END OR ufl EQ NL_BOTH )
            {
                for ( j = 0; j <= m; j++ )
                {
                    for ( i = n - 1; i <= n; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }

            if( rr EQ q + 1 AND( vfl EQ NL_START OR vfl EQ NL_BOTH ) )
            {
                if( q EQ 2 )
                    jj = 3;
                else
                    jj = 1;

                for ( j = 0; j <= jj; j++ )
                {
                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }

            if( rr EQ m AND( vfl EQ NL_END OR vfl EQ NL_BOTH ) )
            {
                if( q EQ 2 )
                    jj = m - 3;
                else
                    jj = m - 1;

                for ( j = jj; j <= m; j++ )
                {
                    for ( i = 0; i <= n; i++ )
                        N_CopyCPt( Pw[i][j], &Qw[i][j] );
                }
            }
            break;
    }

    for ( i = 0; i <= r; i++ )
        UQ[i] = UP[i];

    for ( j = 0; j <= s; j++ )
        VQ[j] = VP[j];

    /* Remove in u-direction */

    if( dir EQ NL_UDIR )
    {

        first = rr - p;
        last = rr - ss;
        off = first - 1;
        fout = (2 * rr - ss - p) / 2;

        i = first;
        j = last;

        while( (j - i)GT 0 )
        {
            alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (UQ[rr] - UQ[i]);
            oma[i - first] = 1.0 - alf[i - first];
            bet[j - first] = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[rr]);
            omb[j - first] = 1.0 - bet[j - first];
            i++;
            j--;
        }

        if( (j - i)LT 0 )
        {
            lam = alf[i - first - 1] / (alf[i - first - 1] + bet[j - first + 1]);
            oml = 1.0 - lam;
        }

        if( vfl EQ NL_START OR vfl EQ NL_BOTH )
        {
            for ( col = 0; col <= 1; col++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[off][col], &Rw[0] );
                N_CopyCPt( Qw[last + 1][col], &Rw[last + 1 - off] );

                while( (j - i)GT 0 )
                {

                    N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[ii - 1], &Rw[ii] );

                    N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                if( (j - i)LT 0 )
                    N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );

                i = first;
                j = last;

                while( (j - i)GT 0 )
                {
                    N_CopyCPt( Rw[i - off], &Qw[i][col] );
                    N_CopyCPt( Rw[j - off], &Qw[j][col] );
                    i++;
                    j--;
                }

                for ( i = fout + 1; i <= n; i++ )
                    N_CopyCPt( Qw[i][col], &Qw[i - 1][col] );
            }
        }

        if( vfl EQ NL_END OR vfl EQ NL_BOTH )
        {
            for ( col = m - 1; col <= m; col++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[off][col], &Rw[0] );
                N_CopyCPt( Qw[last + 1][col], &Rw[last + 1 - off] );

                while( (j - i)GT 0 )
                {

                    N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[ii - 1], &Rw[ii] );

                    N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                if( (j - i)LT 0 )
                    N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );

                i = first;
                j = last;

                while( (j - i)GT 0 )
                {
                    N_CopyCPt( Rw[i - off], &Qw[i][col] );
                    N_CopyCPt( Rw[j - off], &Qw[j][col] );
                    i++;
                    j--;
                }

                for ( i = fout + 1; i <= n; i++ )
                    N_CopyCPt( Qw[i][col], &Qw[i - 1][col] );
            }
        }

        if( vfl EQ NL_NO )
        {
            first = 0;
            last = m;
        }
        else if( vfl EQ NL_START )
        {
            first = 2;
            last = m;
        }
        else if( vfl EQ NL_END )
        {
            first = 0;
            last = m - 2;
        }
        else if( vfl EQ NL_BOTH )
        {
            first = 2;
            last = m - 2;
        }

        if( rr EQ p + 1 AND( ufl EQ NL_START OR ufl EQ NL_BOTH ) )
        {
            if( p EQ 2 )
            {
                for ( col = first; col <= last; col++ )
                {
                    N_CopyCPt( Qw[0][col], &Rw[0] );
                    N_CopyCPt( Qw[3][col], &Rw[3] );
                    N_Combine2CPts( alf[0], Qw[1][col], oma[0], Rw[0], &Rw[1] );
                    N_Combine2CPts( bet[1], Qw[2][col], omb[1], Rw[3], &Rw[2] );
                    N_Combine2CPts( lam, Rw[2], oml, Rw[1], &Qw[1][col] );
                }
            }
            else
            {
                for ( col = first; col <= last; col++ )
                {
                    N_Combine2CPts( alf[0], Qw[1][col], oma[0], Qw[0][col], &Qw[1][col] );
                }
            }
        }

        if( rr EQ n AND( ufl EQ NL_END OR ufl EQ NL_BOTH ) )
        {
            if( p EQ 2 )
            {
                for ( col = first; col <= last; col++ )
                {
                    N_CopyCPt( Qw[n - 3][col], &Rw[0] );
                    N_CopyCPt( Qw[n][col], &Rw[3] );
                    N_Combine2CPts( alf[0], Qw[n - 2][col], oma[0], Rw[0], &Rw[1] );
                    N_Combine2CPts( bet[1], Qw[n - 1][col], omb[1], Rw[3], &Rw[2] );
                    N_Combine2CPts( lam, Rw[2], oml, Rw[1], &Qw[n - 2][col] );
                    N_CopyCPt( Qw[n][col], &Qw[n - 1][col] );
                }
            }
            else
            {
                for ( col = first; col <= last; col++ )
                {

                    N_Combine2CPts( bet[p - 1], Qw[n - 1][col], omb[p - 1], Qw[n][col], &Qw[n - 2][col] );
                    N_CopyCPt( Qw[n][col], &Qw[n - 1][col] );
                }
            }
        }

        for ( i = rr + 1; i <= r; i++ )
            UQ[i - 1] = UQ[i];

        N_SrfSetSizeIndices( surQ, n - 1, m, p, q, r - 1, s );
    }

    /* Remove in v-direction */

    if( dir EQ NL_VDIR )
    {

        first = rr - q;
        last = rr - ss;
        off = first - 1;
        fout = (2 * rr - ss - q) / 2;

        i = first;
        j = last;

        while( (j - i)GT 0 )
        {
            alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (VQ[rr] - VQ[i]);
            oma[i - first] = 1.0 - alf[i - first];
            bet[j - first] = (VQ[j + q + 1] - VQ[j]) / (VQ[j + q + 1] - VQ[rr]);
            omb[j - first] = 1.0 - bet[j - first];
            i++;
            j--;
        }

        if( (j - i)LT 0 )
        {
            lam = alf[i - first - 1] / (alf[i - first - 1] + bet[j - first + 1]);
            oml = 1.0 - lam;
        }

        if( ufl EQ NL_START OR ufl EQ NL_BOTH )
        {
            for ( row = 0; row <= 1; row++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[row][off], &Rw[0] );
                N_CopyCPt( Qw[row][last + 1], &Rw[last + 1 - off] );

                while( (j - i)GT 0 )
                {

                    N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[ii - 1], &Rw[ii] );

                    N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                if( (j - i)LT 0 )
                    N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );

                i = first;
                j = last;

                while( (j - i)GT 0 )
                {
                    N_CopyCPt( Rw[i - off], &Qw[row][i] );
                    N_CopyCPt( Rw[j - off], &Qw[row][j] );
                    i++;
                    j--;
                }

                for ( j = fout + 1; j <= m; j++ )
                    N_CopyCPt( Qw[row][j], &Qw[row][j - 1] );
            }
        }

        if( ufl EQ NL_END OR ufl EQ NL_BOTH )
        {
            for ( row = n - 1; row <= n; row++ )
            {
                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Qw[row][off], &Rw[0] );
                N_CopyCPt( Qw[row][last + 1], &Rw[last + 1 - off] );

                while( (j - i)GT 0 )
                {

                    N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[ii - 1], &Rw[ii] );

                    N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[jj + 1], &Rw[jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                if( (j - i)LT 0 )
                    N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );

                i = first;
                j = last;

                while( (j - i)GT 0 )
                {
                    N_CopyCPt( Rw[i - off], &Qw[row][i] );
                    N_CopyCPt( Rw[j - off], &Qw[row][j] );
                    i++;
                    j--;
                }

                for ( j = fout + 1; j <= m; j++ )
                    N_CopyCPt( Qw[row][j], &Qw[row][j - 1] );
            }
        }

        if( ufl EQ NL_NO )
        {
            first = 0;
            last = n;
        }
        else if( ufl EQ NL_START )
        {
            first = 2;
            last = n;
        }
        else if( ufl EQ NL_END )
        {
            first = 0;
            last = n - 2;
        }
        else if( ufl EQ NL_BOTH )
        {
            first = 2;
            last = n - 2;
        }

        if( rr EQ q + 1 AND( vfl EQ NL_START OR vfl EQ NL_BOTH ) )
        {
            if( q EQ 2 )
            {
                for ( row = first; row <= last; row++ )
                {
                    N_CopyCPt( Qw[row][0], &Rw[0] );
                    N_CopyCPt( Qw[row][3], &Rw[3] );
                    N_Combine2CPts( alf[0], Qw[row][1], oma[0], Rw[0], &Rw[1] );
                    N_Combine2CPts( bet[1], Qw[row][2], omb[1], Rw[3], &Rw[2] );
                    N_Combine2CPts( lam, Rw[2], oml, Rw[1], &Qw[row][1] );
                }
            }
            else
            {
                for ( row = first; row <= last; row++ )
                {
                    N_Combine2CPts( alf[0], Qw[row][1], oma[0], Qw[row][0], &Qw[row][1] );
                }
            }
        }

        if( rr EQ m AND( vfl EQ NL_END OR vfl EQ NL_BOTH ) )
        {
            if( q EQ 2 )
            {
                for ( row = first; row <= last; row++ )
                {
                    N_CopyCPt( Qw[row][m - 3], &Rw[0] );
                    N_CopyCPt( Qw[row][m], &Rw[3] );
                    N_Combine2CPts( alf[0], Qw[row][m - 2], oma[0], Rw[0], &Rw[1] );
                    N_Combine2CPts( bet[1], Qw[row][m - 1], omb[1], Rw[3], &Rw[2] );
                    N_Combine2CPts( lam, Rw[2], oml, Rw[1], &Qw[row][m - 2] );
                    N_CopyCPt( Qw[row][m], &Qw[row][m - 1] );
                }
            }
            else
            {
                for ( row = first; row <= last; row++ )
                {

                    N_Combine2CPts( bet[q - 1], Qw[row][m - 1], omb[q - 1], Qw[row][m], &Qw[row][m - 2] );
                    N_CopyCPt( Qw[row][m], &Qw[row][m - 1] );
                }
            }
        }

        for ( j = rr + 1; j <= s; j++ )
            VQ[j - 1] = VQ[j];

        N_SrfSetSizeIndices( surQ, n, m - 1, p, q, r, s - 1 );
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveKnot */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS surface
     while  maintaining boundary  constraints. If  the output surface is 
     initialized to NULL, memory to store  new control points and  knots 

     is allocated. A typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     emx, eub, evb, eut, evt;
       NL_STACKS   SQ;
       ...
       (define surP, choose error tolerances);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveKnotsKeepBoundaries(&surP,NL_BOTH,NL_NO ,NL_TANGENT    ,emx,eub,evb,eut,evt,NL_UDIR ,
                &surQ,&SQ);
       N_SrfRemoveKnotsKeepBoundaries(&surP,NL_BOTH,NL_END,DERIVATIVES,emx,eub,evb,eut,evt,NL_UVDIR,
                &surP,&SQ);

     If memory is  available, surQ  is not  initialized  and the routine
     assumes that memory  allocation  has been done. However, it  checks 

     for the  proper amount  by looking at the highest indexes in surQ's 
     knot vector  and polygon objects. If  surP is the same as surQ, the
     removal is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     ufl  , input  ,  Flag:
                        NL_NO   : do not constrain u-derivatives
                        NL_START: constrain u-derivatives at u=umin
                        NL_END  : constrain u-derivatives at u=umax
                        NL_BOTH : constrain u-derivatives at both ends
     vfl  , input  ,  Flag:
                        NL_NO   : do not constrain v-derivatives
                        NL_START: constrain v-derivatives at v=vmin
                        NL_END  : constrain v-derivatives at v=vmax
                        NL_BOTH : constrain v-derivatives at both ends
     tnf  , input  ,  Flag:
                        NL_TANGENT    : deviation in tangent directions are
                                     checked 
                        DERIVATIVES: absolute  deviation in  derivatives 
                                     are checked
     emx  , input  ,  Overall error tolerance, i.e. the new surface will
                      not deviate from the origonal surface by more than
                      emx
     eub  , input  ,  Error  tolerance  along  the   v=vmin  and  v=vmax 
                      boundaries, i.e. for u-knot removal
     evb  , input  ,  Error  tolerance  along  the   u=umin  and  u=umax 
                      boundaries, i.e. for v-knot removal
     eut  , input  ,  Error  tolerance for  u-derivatives  on the u=umin 
                      and u=umax  boundaries. FOR TANGENTS IT IS ANGULAR
                      TOLERANCE MEASURED IN  DEGREES. FOR DERIVATIVES IT
                      IS NL_ABSOLUTE TOLERANCE.
     evt  , input  ,  Error  tolerance  for  v-derivatives on the v=vmin 
                      and v=vmax  boundaries. FOR TANGENTS IT IS ANGULAR
                      TOLERANCE MEASURED IN  DEGREES. FOR DERIVATIVES IT
                      IS NL_ABSOLUTE TOLERANCE.
     dir  , input  ,  Flag:
                        NL_UDIR : remove all removable u-knots
                        NL_VDIR : remove all removable v-knots
                        NL_UVDIR: remove all removable u- and v-knots
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveKnotsKeepBoundaries( NL_SURFACE *surP, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG tnf, NL_REAL emx, NL_REAL eub, NL_REAL evb, NL_REAL eut, NL_REAL evt, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveKnotsKeepBoundaries");

    NL_FLAG krm, rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sru = NULL, *srv = NULL, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru = 0, su = 0, rv = 0, sv = 0, ns, ms;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, *mru = NULL, *mrv = NULL, *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb, *minl, *maxl, *minr, *maxr, *max, *tul, *tur, *tvb, *tvt, *tea, *teb, *ele, *eri, *ebo, *eto, stu, svt = 0.0, tol, lam = 0.0, oml = 0.0, wmin, wmax, pmax, tmp, al, be, ob, bu = 0.0, bv = 0.0, stv, wi, wj, eus, eue, evs, eve, mus, mue, mvs, mve, sut = 0.0;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, ** Rw;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    ns = n;
    ms = m;

    /* Adjust removal tolerances */

    if( N_IsSrfRat( surP ) )
    {
        N_SrfMinMaxWeightPosVectors( surP, &wmin, &tmp, &tmp, &pmax );
        emx = (emx * wmin) / (1.0 + pmax);
        eub = (eub * wmin) / (1.0 + pmax);
        evb = (evb * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    if( tnf EQ NL_TANGENT )
    {
        al = (eut * NL_PI) / 180.0;
        be = (evt * NL_PI) / 180.0;
        sut = sin( al );
        svt = sin( be );
    }

    stu = sto * fabs( UP[r] - UP[0] );
    stv = sto * fabs( VP[s] - VP[0] );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsKnotVectorAndKnots( surP, &Qw, &knu, &knv, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsKnotVectorAndKnots( surQ, &Qw, &knu, &knv, &UQ, &VQ );
    }

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

    tul = N_AllocReal1dArray( s, &SL );

    if( tul EQ NULL )
        NL_QUIT;

    tur = N_AllocReal1dArray( s, &SL );

    if( tur EQ NULL )
        NL_QUIT;

    tvb = N_AllocReal1dArray( r, &SL );

    if( tvb EQ NULL )
        NL_QUIT;

    tvt = N_AllocReal1dArray( r, &SL );

    if( tvt EQ NULL )
        NL_QUIT;

    ele = N_AllocReal1dArray( s, &SL );

    if( ele EQ NULL )
        NL_QUIT;

    eri = N_AllocReal1dArray( s, &SL );

    if( eri EQ NULL )
        NL_QUIT;

    ebo = N_AllocReal1dArray( r, &SL );

    if( ebo EQ NULL )
        NL_QUIT;

    eto = N_AllocReal1dArray( r, &SL );

    if( eto EQ NULL )
        NL_QUIT;

    tea = N_AllocReal1dArray( NL_MAX( r, s ), &SL );

    if( tea EQ NULL )
        NL_QUIT;

    teb = N_AllocReal1dArray( NL_MAX( r, s ), &SL );

    if( teb EQ NULL )
        NL_QUIT;

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        mru = N_AllocReal1dArray( r, &SL );

        if( mru EQ NULL )
            NL_QUIT;

        sru = N_AllocInt1dArray( r, &SL );

        if( sru EQ NULL )
            NL_QUIT;
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        mrv = N_AllocReal1dArray( s, &SL );

        if( mrv EQ NULL )
            NL_QUIT;

        srv = N_AllocInt1dArray( s, &SL );

        if( srv EQ NULL )
            NL_QUIT;
    }

    error = N_AllocSrfArrays( &surA, n, m, p, q, r, s, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize */

    if( surP NEQ surQ )
    {
        error = N_SrfCopy( surP, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= r; i++ )
    {
        tvb[i] = 0.0;
        tvt[i] = 0.0;
        ebo[i] = 0.0;
        eto[i] = 0.0;

        for ( j = 0; j <= s; j++ )
            er[i][j] = 0.0;
    }

    for ( j = 0; j <= s; j++ )
    {
        tul[j] = 0.0;
        tur[j] = 0.0;
        ele[j] = 0.0;
        eri[j] = 0.0;
    }

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        for ( i = 0; i <= r; i++ )
        {
            mru[i] = NL_BIGD;
            sru[i] = 0;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        for ( j = 0; j <= s; j++ )
        {
            mrv[j] = NL_BIGD;
            srv[j] = 0;
        }
    }

    /* Compute the maximum of knot removal errors for each distinct knot */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        ru = p + 1;

        while( ru LE n )
        {
            i = ru;

            while( ru LE n AND UQ[ru]EQ UQ[ru + 1] )
                ru++;
            sru[ru] = ru - i + 1;

            error = N_SrfRemoveOneKnot( surP, ru, sru[ru], 0, m, NL_UDIR, &mru[ru] );

            if( error EQ NL_YES )
                NL_OUT;

            ru++;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        rv = q + 1;

        while( rv LE m )
        {
            i = rv;

            while( rv LE m AND VQ[rv]EQ VQ[rv + 1] )
                rv++;
            srv[rv] = rv - i + 1;

            error = N_SrfRemoveOneKnot( surP, rv, srv[rv], 0, n, NL_VDIR, &mrv[rv] );

            if( error EQ NL_YES )
                NL_OUT;

            rv++;
        }
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
        {
            bu = mru[p + 1];
            su = sru[p + 1];
            ru = p + 1;

            for ( i = p + 2; i <= n; i++ )
            {
                if( mru[i]LT bu )
                {
                    bu = mru[i];
                    su = sru[i];
                    ru = i;
                }
            }
        }

        if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
        {
            bv = mrv[q + 1];
            sv = srv[q + 1];
            rv = q + 1;

            for ( j = q + 2; j <= m; j++ )
            {
                if( mrv[j]LT bv )
                {
                    bv = mrv[j];
                    sv = srv[j];
                    rv = j;
                }
            }
        }

        /* If no more removable knot -> finished */

        if( dir EQ NL_UDIR )
        {
            if( bu EQ NL_BIGD OR bu EQ NOREM )
                break;
        }
        else if( dir EQ NL_VDIR )
        {
            if( bv EQ NL_BIGD OR bv EQ NOREM )
                break;
        }
        else if( dir EQ NL_UVDIR )
        {
            if( (bu EQ NL_BIGD OR bu EQ NOREM)AND( bv EQ NL_BIGD OR bv EQ NOREM ) )
                break;
        }

        if( dir EQ NL_UVDIR )
        {
            if( bu LT bv )
                krm = NL_UDIR;
            else
                krm = NL_VDIR;
        }
        else
        {
            krm = dir;
        }

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR: /* Remove in the u-direction */

                /* Get maximums of basis functions */

                if( (p + su) % 2 )
                {
                    k = (p + su + 1) / 2;
                    l = ru - k + p + 1;
                    al = (UQ[ru] - UQ[ru - k]) / (UQ[ru - k + p + 1] - UQ[ru - k]);
                    be = (UQ[ru] - UQ[ru - k + 1]) / (UQ[ru - k + p + 2] - UQ[ru - k + 1]);
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

                /***************************************/
                /* Check positional and tangent errors */
                /***************************************/

                rmf = NL_TRUE;

                /* Check surface error */

                for ( i = ru - k; i <= l; i++ )
                {
                    if( UQ[i]NEQ UQ[i + 1] )
                    {
                        tea[i] = ebo[i] + max[i - ru + k] * bu;

                        if( tea[i]GT eub )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        teb[i] = eto[i] + max[i - ru + k] * bu;

                        if( teb[i]GT eub )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        tmp = max[i - ru + k] * bu;

                        for ( j = q; j <= m; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                            {
                                te[i][j] = er[i][j] + tmp;

                                if( te[i][j]GT emx )
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

                /* If error test passed, update error vector */

                if( rmf EQ NL_TRUE )
                {
                    for ( i = ru - k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1] )
                        {
                            ebo[i] = tea[i];
                            eto[i] = teb[i];

                            for ( j = q; j <= m; j++ )
                            {
                                if( VQ[j]NEQ VQ[j + 1] )
                                    er[i][j] = te[i][j];
                            }
                        }
                    }
                }
                else
                {
                    mru[ru] = NOREM;
                    continue;
                }

                /* Get derivative errors for one removal step */

                error = N_SrfCrossBoundaryDerivErr( surQ, &surA, ru, su, NL_UDIR, ufl, vfl, &eus, &eue, &evs, &eve, tnf, &mus, &mue, &mvs, &mve );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Check v-tangent error along v=vmin */

                if( vfl EQ NL_START OR vfl EQ NL_BOTH )
                {
                    tol = evt;

                    if( tnf EQ NL_TANGENT )
                        tol = mvs * svt;

                    for ( i = ru - k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1] )
                        {
                            tea[i] = tvb[i] + evs;

                            if( tea[i]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( i = ru - k; i <= l; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                                tvb[i] = tea[i];
                        }
                    }
                    else
                    {
                        mru[ru] = NOREM;
                        continue;
                    }
                }

                /* Check v-tangent error along v=vmax */

                if( vfl EQ NL_END OR vfl EQ NL_BOTH )
                {
                    tol = evt;

                    if( tnf EQ NL_TANGENT )
                        tol = mve * svt;

                    for ( i = ru - k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1] )
                        {
                            teb[i] = tvt[i] + eve;

                            if( teb[i]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( i = ru - k; i <= l; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                                tvt[i] = teb[i];
                        }
                    }
                    else
                    {
                        mru[ru] = NOREM;
                        continue;
                    }
                }

                /* Check u-derivative constraint on u=umin */

                if( ru EQ p + 1 AND( ufl EQ NL_START OR ufl EQ NL_BOTH ) )
                {
                    tol = eut;

                    if( tol EQ NL_TANGENT )
                        tol = mus * sut;

                    for ( j = q; j <= m; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1] )
                        {
                            tea[j] = tul[j] + eus;

                            if( tea[j]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( j = q; j <= m; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                                tul[j] = tea[j];
                        }
                    }
                    else
                    {
                        mru[ru] = NOREM;
                        continue;
                    }
                }

                /* Check u-derivative constraint on u=umax */

                if( ru EQ n AND( ufl EQ NL_END OR ufl EQ NL_BOTH ) )
                {
                    tol = eut;

                    if( tnf EQ NL_TANGENT )
                        tol = mue * sut;

                    for ( j = q; j <= m; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1] )
                        {
                            teb[j] = tur[j] + eue;

                            if( teb[j]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( j = q; j <= m; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                                tur[j] = teb[j];
                        }
                    }
                    else
                    {
                        mru[ru] = NOREM;
                        continue;
                    }
                }

                /* Remove knot */

                if( rmf EQ NL_TRUE )
                {
                    fout = (2 * ru - su - p) / 2;
                    first = ru - p;
                    last = ru - su;
                    off = first - 1;

                    /* Save some parameters */

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (UQ[ru] - UQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[ru]);
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

                        N_CopyCPt( Qw[off][col], &Rw[col][0] );
                        N_CopyCPt( Qw[last + 1][col], &Rw[col][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {

                            N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[col][ii - 1], &Rw[col][ii] );

                            N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[col][jj + 1], &Rw[col][jj] );
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

                            N_Combine2CPts( lam, Rw[col][jj + 1], oml, Rw[col][ii - 1], &Rw[col][jj + 1] );
                        }
                    } /* End for each row */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        mru[ru] = NOREM;
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
                                N_CopyCPt( Rw[col][i - off], &Qw[i][col] );
                                N_CopyCPt( Rw[col][j - off], &Qw[j][col] );
                                i++;
                                j--;
                            }
                        }
                    }

                    /* Successful removal -> shift down some entinties */

                    if( su EQ 1 )
                    {
                        for ( j = q; j <= m; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                            {
                                er[ru - 1][j] = NL_MAX( er[ru - 1][j], er[ru][j] );
                            }
                        }

                        ebo[ru - 1] = NL_MAX( ebo[ru - 1], ebo[ru] );
                        eto[ru - 1] = NL_MAX( eto[ru - 1], eto[ru] );
                        tvb[ru - 1] = NL_MAX( tvb[ru - 1], tvb[ru] );
                        tvt[ru - 1] = NL_MAX( tvt[ru - 1], tvt[ru] );
                    }

                    if( su GT 1 )
                        sru[ru - 1] = sru[ru] - 1;

                    for ( i = ru + 1; i <= r; i++ )
                    {
                        mru[i - 1] = mru[i];
                        sru[i - 1] = sru[i];
                        ebo[i - 1] = ebo[i];
                        eto[i - 1] = eto[i];
                        UQ[i - 1] = UQ[i];

                        if( vfl EQ NL_START OR vfl EQ NL_BOTH )
                            tvb[i - 1] = tvb[i];

                        if( vfl EQ NL_END OR vfl EQ NL_BOTH )
                            tvt[i - 1] = tvt[i];

                        for ( j = q; j <= m; j++ )
                            er[i - 1][j] = er[i][j];
                    }

                    for ( col = 0; col <= m; col++ )
                    {
                        for ( i = fout + 1; i <= n; i++ )
                        {
                            N_CopyCPt( Qw[i][col], &Qw[i - 1][col] );
                        }
                    }

                    n--;
                    r--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_UDIR )
                    {
                        if( n EQ p )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( ru - p, p + 1 );
                    l = NL_MIN( n, ru + p - su );

                    for ( i = k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1]AND mru[i]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, i, sru[i], 0, m, NL_UDIR, &mru[i] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( j = q + 1; j <= m; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1]AND mrv[j]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, j, srv[j], first, last, NL_VDIR, &mrv[j] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */

                /* Get maximum of basis function */

                if( (q + sv) % 2 )
                {
                    k = (q + sv + 1) / 2;
                    l = rv - k + q + 1;
                    al = (VQ[rv] - VQ[rv - k]) / (VQ[rv - k + q + 1] - VQ[rv - k]);
                    be = (VQ[rv] - VQ[rv - k + 1]) / (VQ[rv - k + q + 2] - VQ[rv - k + 1]);
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

                /***************************************/
                /* Check positional and tangent errors */
                /***************************************/

                rmf = NL_TRUE;

                /* Check surface error */

                for ( j = rv - k; j <= l; j++ )
                {
                    if( VQ[j]NEQ VQ[j + 1] )
                    {
                        tea[j] = ele[j] + max[j - rv + k] * bv;

                        if( tea[j]GT evb )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        teb[j] = eri[j] + max[j - rv + k] * bv;

                        if( teb[j]GT evb )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        tmp = max[j - rv + k] * bv;

                        for ( i = p; i <= n; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                            {
                                te[i][j] = er[i][j] + tmp;

                                if( te[i][j]GT emx )
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

                /* If error test passed, update error vector */

                if( rmf EQ NL_TRUE )
                {
                    for ( j = rv - k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1] )
                        {
                            ele[j] = tea[j];
                            eri[j] = teb[j];

                            for ( i = p; i <= n; i++ )
                            {
                                if( UQ[i]NEQ UQ[i + 1] )
                                    er[i][j] = te[i][j];
                            }
                        }
                    }
                }
                else
                {
                    mrv[rv] = NOREM;
                    continue;
                }

                /* Get derivative errors for one removal step */

                error = N_SrfCrossBoundaryDerivErr( surQ, &surA, rv, sv, NL_VDIR, ufl, vfl, &eus, &eue, &evs, &eve, tnf, &mus, &mue, &mvs, &mve );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Check u-tangent error along u=umin */

                if( ufl EQ NL_START OR ufl EQ NL_BOTH )
                {
                    tol = eut;

                    if( tnf EQ NL_TANGENT )
                        tol = mus * sut;

                    for ( j = rv - k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1] )
                        {
                            tea[j] = tul[j] + eus;

                            if( tea[j]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( j = rv - k; j <= l; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                                tul[j] = tea[j];
                        }
                    }
                    else
                    {
                        mrv[rv] = NOREM;
                        continue;
                    }
                }

                /* Check u-tangent error along u=umax */

                if( ufl EQ NL_END OR ufl EQ NL_BOTH )
                {
                    tol = eut;

                    if( tnf EQ NL_TANGENT )
                        tol = mue * sut;

                    for ( j = rv - k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1] )
                        {
                            teb[j] = tur[j] + eue;

                            if( teb[j]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( j = rv - k; j <= l; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
                                tur[j] = teb[j];
                        }
                    }
                    else
                    {
                        mrv[rv] = NOREM;
                        continue;
                    }
                }

                /* Check v-derivative constraint on v=vmin */

                if( rv EQ q + 1 AND( vfl EQ NL_START OR vfl EQ NL_BOTH ) )
                {
                    tol = evt;

                    if( tnf EQ NL_TANGENT )
                        tol = mvs * svt;

                    for ( i = p; i <= n; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1] )
                        {
                            tea[i] = tvb[i] + evs;

                            if( tea[i]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( i = p; i <= n; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                                tvb[i] = tea[i];
                        }
                    }
                    else
                    {
                        mrv[rv] = NOREM;
                        continue;
                    }
                }

                /* Check v-derivative constraint on v=vmax */

                if( rv EQ m AND( vfl EQ NL_END OR vfl EQ NL_BOTH ) )
                {
                    tol = evt;

                    if( tnf EQ NL_TANGENT )
                        tol = mve * svt;

                    for ( i = p; i <= n; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1] )
                        {
                            teb[i] = tvt[i] + eve;

                            if( teb[i]GT tol )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }
                    }

                    if( rmf EQ NL_TRUE )
                    {
                        for ( i = p; i <= n; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                                tvt[i] = teb[i];
                        }
                    }
                    else
                    {
                        mrv[rv] = NOREM;
                        continue;
                    }
                }

                /* Remove knot */

                if( rmf EQ NL_TRUE )
                {
                    fout = (2 * rv - sv - q) / 2;
                    first = rv - q;
                    last = rv - sv;
                    off = first - 1;

                    /* Save some parameters */

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (VQ[rv] - VQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (VQ[j + q + 1] - VQ[j]) / (VQ[j + q + 1] - VQ[rv]);
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

                        N_CopyCPt( Qw[row][off], &Rw[row][0] );
                        N_CopyCPt( Qw[row][last + 1], &Rw[row][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {

                            N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[row][ii - 1], &Rw[row][ii] );

                            N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[row][jj + 1], &Rw[row][jj] );
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

                            N_Combine2CPts( lam, Rw[row][jj + 1], oml, Rw[row][ii - 1], &Rw[row][jj + 1] );
                        }
                    } /* End for each column */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        mrv[rv] = NOREM;
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
                                N_CopyCPt( Rw[row][i - off], &Qw[row][i] );
                                N_CopyCPt( Rw[row][j - off], &Qw[row][j] );
                                i++;
                                j--;
                            }
                        }
                    }

                    /* Successful removal -> shift down some entinties */

                    if( sv EQ 1 )
                    {
                        for ( i = p; i <= n; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
                            {
                                er[i][rv - 1] = NL_MAX( er[i][rv - 1], er[i][rv] );
                            }
                        }

                        ele[rv - 1] = NL_MAX( ele[rv - 1], ele[rv] );
                        eri[rv - 1] = NL_MAX( eri[rv - 1], eri[rv] );
                        tul[rv - 1] = NL_MAX( tul[rv - 1], tul[rv] );
                        tur[rv - 1] = NL_MAX( tur[rv - 1], tur[rv] );
                    }

                    if( sv GT 1 )
                        srv[rv - 1] = srv[rv] - 1;

                    for ( j = rv + 1; j <= s; j++ )
                    {
                        mrv[j - 1] = mrv[j];
                        srv[j - 1] = srv[j];
                        ele[j - 1] = ele[j];
                        eri[j - 1] = eri[j];
                        VQ[j - 1] = VQ[j];

                        if( ufl EQ NL_START OR ufl EQ NL_BOTH )
                            tul[j - 1] = tul[j];

                        if( ufl EQ NL_END OR ufl EQ NL_BOTH )
                            tur[j - 1] = tur[j];

                        for ( i = p; i <= n; i++ )
                            er[i][j - 1] = er[i][j];
                    }

                    for ( row = 0; row <= n; row++ )
                    {
                        for ( j = fout + 1; j <= m; j++ )
                        {
                            N_CopyCPt( Qw[row][j], &Qw[row][j - 1] );
                        }
                    }

                    m--;
                    s--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_VDIR )
                    {
                        if( m EQ q )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( rv - q, q + 1 );
                    l = NL_MIN( m, rv + q - sv );

                    for ( j = k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1]AND mrv[j]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, j, srv[j], 0, n, NL_VDIR, &mrv[j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( i = p + 1; i <= n; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1]AND mru[i]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, i, sru[i], first, last, NL_UDIR, &mru[i] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
        } /* End of switch */
    }     /* End of while */

    /* Compact surface */

    if( n LT ns OR m LT ms )
    {
        error = N_SrfCompress( surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveKnotsKeepBoundaries */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This tools  routine computes  the  cross-boundary  derivative error
     after one knot is removed either in u- or in v-direction. A typical 
     calling example:
 
       NL_SURFACE  surP, surQ;
       NL_INDEX    rr, ss;
       NL_REAL     eus, eue, evs, eve, mus, mue, mvs, mve;
       ...
       (define surP);
       ...
       N_SrfCrossBoundaryDerivErr(&surP,&surQ,rr,ss,NL_UDIR,NL_START,NL_END,&eus,&eue,&evs,&eve,
                NL_TANGENT,&mus,&mue,&mvs,&mve);
 
     IT IS ASSUMED  THAT  MEMORY FOR  surQ IS  ALLOCATED IN  THE CALLING
     ROUTINE.
 
 
   ACCESS:
   
     surP  , input  ,  NURBS surface
     surQ  , input  ,  Working surface
     rr,ss , input  ,  Index and multiplicity of knot to be removed
     dir   , input  ,  Flag:
                         NL_UDIR: Remove in u-direction
                         NL_VDIR: Remove in v-direction
     ufl   , input  ,  Flag:
                         NL_START: remove for u=umin cross-derivative
                         NL_END  : remove for u=umax cross-derivative
                         NL_BOTH : remove for both cross-derivative
     vfl   , input  ,  Flag:
                         NL_START: remove for v=vmin cross-derivative
                         NL_END  : remove for v=vmax cross-derivative
                         NL_BOTH : remove for both cross-derivative
     eus   , output ,  Error along u=umin
     eue   , output ,  Error along u=umax
     evs   , output ,  Error along v=vmin
     eve   , output ,  Error along v=vmax
     tnf   , input  ,  Flag:
                         NL_TANGENT   : tangent
                         NL_DERIVATIVE: derivative
     mus   , output ,  Minimum of cross-boundary across u=umin
     mue   , output ,  Minimum of cross-boundary across u=umax
     mvs   , output ,  Minimum of cross-boundary across v=vmin
     mve   , output ,  Minimum of cross-boundary across v=vmax
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SrfCrossBoundaryDerivErr( NL_SURFACE *surP, NL_SURFACE *surQ, NL_INDEX rr, NL_INDEX ss, NL_FLAG dir, NL_FLAG ufl, NL_FLAG vfl, NL_REAL *eus, NL_REAL *eue, NL_REAL *evs, NL_REAL *eve, NL_FLAG tnf, NL_REAL *mus, NL_REAL *mue, NL_REAL *mvs, NL_REAL *mve )
{

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s;

    NL_DEGREE p, q;

    NL_POINT P;

    NL_CURVE cus, cush, cue, cueh, cvs, cvsh, cve, cveh;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize and get surQ */

    *eus = 0.0;
    *mus = 0.0;
    *eue = 0.0;
    *mue = 0.0;
    *evs = 0.0;
    *mvs = 0.0;
    *eve = 0.0;
    *mve = 0.0;

    error = N_SrfRemoveKnot( surP, rr, ss, dir, ufl, vfl, surQ );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetArraySizes( surP, &n, &m, &r, &s );
    N_SrfGetDegrees( surP, &p, &q );

    /* Remove u-knot */

    if( dir EQ NL_UDIR )
    {
        if( vfl EQ NL_START OR vfl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cvs );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_BOTTOM, &cvs, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cvsh );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_BOTTOM, &cvsh, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cvs, &cvsh, evs );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cvs, &P, mvs );
        }

        if( vfl EQ NL_END OR vfl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cve );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_TOP, &cve, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cveh );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_TOP, &cveh, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cve, &cveh, eve );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cve, &P, mve );
        }

        if( rr EQ p + 1 AND( ufl EQ NL_START OR ufl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cus );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_LEFT, &cus, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cush );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_LEFT, &cush, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cus, &cush, eus );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cus, &P, mus );
        }

        if( rr EQ n AND( ufl EQ NL_END OR ufl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cue );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_RIGHT, &cue, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cueh );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_RIGHT, &cueh, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cue, &cueh, eue );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cue, &P, mue );
        }
    }

    /* Remove v-knot */

    if( dir EQ NL_VDIR )
    {
        if( ufl EQ NL_START OR ufl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cus );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_LEFT, &cus, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cush );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_LEFT, &cush, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cus, &cush, eus );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cus, &P, mus );
        }

        if( ufl EQ NL_END OR ufl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cue );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_RIGHT, &cue, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cueh );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_RIGHT, &cueh, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cue, &cueh, eue );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cue, &P, mue );
        }

        if( rr EQ q + 1 AND( vfl EQ NL_START OR vfl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cvs );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_BOTTOM, &cvs, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cvsh );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_BOTTOM, &cvsh, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cvs, &cvsh, evs );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cvs, &P, mvs );
        }

        if( rr EQ m AND( vfl EQ NL_END OR vfl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cve );
            error = N_CrossBoundDerivCrvNurbsSrf( surP, NL_TOP, &cve, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &cveh );
            error = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_TOP, &cveh, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvDiffCrvGetMaxChange( &cve, &cveh, eve );

            if( error EQ NL_YES )
                NL_OUT;

            if( tnf EQ NL_TANGENT )
                N_CrvGetMinPosVector( &cve, &P, mve );
        }
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfCrossBoundaryDerivErr */

#endif  // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  removes  all  removable  knots  from a  NURBS
     surface with constraints on some of them. That is, knots are passed 
     in that are not to be removed. If the output surface is initialized  
     to NULL, memory to store new control points and knots is allocated. 
     A typical calling example is:

       NL_SURFACE    surP, surQ;
       NL_PARAMETER  *UK, *VK;
       NL_INDEX      mu, mv;
       NL_REAL       tol;
       NL_STACKS     SQ;
       ...
       (define surP, get UK, VK and tol);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfRemoveKnotsConstraints(&surP,UK,mu,NULL,mv,tol,NL_UDIR ,&surQ,&SQ);
       N_SrfRemoveKnotsConstraints(&surP,UK,mu,VK  ,mv,tol,NL_UVDIR,&surP,&SQ);

     If  memory is  available, surQ  is not  initialized and the routine
     assumes that memory  allocation has been  done. However, it  checks  
     for the proper  amount by looking at the  highest indexes in surQ's 
     knot vector and  polygon objects. If surP is  the same as surQ, the
     removal is done in place.


   ACCESS:
   
     surP , input  ,  NURBS surface
     UK   , input  ,  Array of u-knots not to be removed
     mu   , input  ,  Highest index in UK
     VK   , input  ,  Array of v-knots not to be removed
     mv   , input  ,  Highest index in VK
     tol  , input  ,  Tolerance to check removability
     dir  , input  ,  Flag:
                        NL_UDIR : remove all removable u-knots
                        NL_VDIR : remove all removable v-knots
                        NL_UVDIR: remove all removable u- and v-knots
     surQ , output ,  Surface after knot removal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRemoveKnotsConstraints( NL_SURFACE *surP, NL_PARAMETER *UK, NL_INDEX mu, NL_PARAMETER *VK, NL_INDEX mv, NL_REAL tol, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRemoveKnotsConstraints");

    NL_FLAG krm, krb, rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sru = NULL, *srv = NULL, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru = 0, su = 0, rv = 0, sv = 0, ns, ms, ku = 0, kv = 0;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb, *minl, *maxl, *minr, *maxr, *max, *bru = NULL, *brv = NULL, lam = 0.0, oml = 0.0, wmin, wmax, pmax, tmp, al, be, ob, bu = 0, bv = 0, stu, stv, wi, wj;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw, ** Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    ns = n;
    ms = m;

    /* Adjust removal tolerance in case of rational surfaces */

    if( N_IsSrfRat( surP ) )
    {
        N_SrfMinMaxWeightPosVectors( surP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    stu = sto * fabs( UP[r] - UP[0] );
    stv = sto * fabs( VP[s] - VP[0] );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsKnotVectorAndKnots( surP, &Qw, &knu, &knv, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsKnotVectorAndKnots( surQ, &Qw, &knu, &knv, &UQ, &VQ );
    }

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

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        bru = N_AllocReal1dArray( r, &SL );

        if( bru EQ NULL )
            NL_QUIT;

        sru = N_AllocInt1dArray( r, &SL );

        if( sru EQ NULL )
            NL_QUIT;
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        brv = N_AllocReal1dArray( s, &SL );

        if( brv EQ NULL )
            NL_QUIT;

        srv = N_AllocInt1dArray( s, &SL );

        if( srv EQ NULL )
            NL_QUIT;
    }

    /* Initialize */

    if( surP NEQ surQ )
    {
        error = N_SrfCopy( surP, surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= r; i++ )
    {
        for ( j = 0; j <= s; j++ )
            er[i][j] = 0.0;
    }

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        for ( i = 0; i <= r; i++ )
        {
            bru[i] = NL_BIGD;
            sru[i] = 0;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        for ( j = 0; j <= s; j++ )
        {
            brv[j] = NL_BIGD;
            srv[j] = 0;
        }
    }

    /* Compute the maximum of knot removal errors for each distinct knot */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        if( UK NEQ NULL )
        {
            ku = 0;

            while( UK[ku]EQ UQ[0] )
                ku++;
        }

        ru = p + 1;

        while( ru LE n )
        {
            i = ru;

            while( ru LE n AND UQ[ru]EQ UQ[ru + 1] )
                ru++;
            sru[ru] = ru - i + 1;

            krb = NL_YES;

            if( UK NEQ NULL )
            {
                while( ku LT mu AND UK[ku]EQ UK[ku + 1] )
                    ku++;

                if( ku LE mu AND UK[ku]EQ UQ[ru] )
                {
                    krb = NL_NO;
                    ku++;
                }
            }

            if( krb EQ NL_YES )
            {
                error = N_SrfRemoveOneKnot( surP, ru, sru[ru], 0, m, NL_UDIR, &bru[ru] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                bru[ru] = NOREM;
            }

            ru++;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        if( VK NEQ NULL )
        {
            kv = 0;

            while( VK[kv]EQ VQ[0] )
                kv++;
        }

        rv = q + 1;

        while( rv LE m )
        {
            i = rv;

            while( rv LE m AND VQ[rv]EQ VQ[rv + 1] )
                rv++;
            srv[rv] = rv - i + 1;

            krb = NL_YES;

            if( VK NEQ NULL )
            {
                while( kv LT mv AND VK[kv]EQ VK[kv + 1] )
                    kv++;

                if( kv LE mv AND VK[kv]EQ VQ[rv] )
                {
                    krb = NL_NO;
                    kv++;
                }
            }

            if( krb EQ NL_YES )
            {
                error = N_SrfRemoveOneKnot( surP, rv, srv[rv], 0, n, NL_VDIR, &brv[rv] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                brv[rv] = NOREM;
            }

            rv++;
        }
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
        {
            bu = bru[p + 1];
            su = sru[p + 1];
            ru = p + 1;

            for ( i = p + 2; i <= r - p - 1; i++ )
            {
                if( bru[i]LT bu )
                {
                    bu = bru[i];
                    su = sru[i];
                    ru = i;
                }
            }
        }

        if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
        {
            bv = brv[q + 1];
            sv = srv[q + 1];
            rv = q + 1;

            for ( j = q + 2; j <= s - q - 1; j++ )
            {
                if( brv[j]LT bv )
                {
                    bv = brv[j];
                    sv = srv[j];
                    rv = j;
                }
            }
        }

        /* If no more removable knot -> finished */

        if( dir EQ NL_UDIR )
        {
            if( bu EQ NL_BIGD OR bu EQ NOREM )
                break;
        }
        else if( dir EQ NL_VDIR )
        {
            if( bv EQ NL_BIGD OR bv EQ NOREM )
                break;
        }
        else if( dir EQ NL_UVDIR )
        {
            if( (bu EQ NL_BIGD OR bu EQ NOREM)AND( bv EQ NL_BIGD OR bv EQ NOREM ) )
                break;
        }

        if( dir EQ NL_UVDIR )
        {
            if( bu LT bv )
                krm = NL_UDIR;
            else
                krm = NL_VDIR;
        }
        else
        {
            krm = dir;
        }

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR: /* Remove in the u-direction */

                rmf = NL_TRUE;

                if( (p + su) % 2 )
                {
                    /* Compute maximums of basis functions over each span */

                    k = (p + su + 1) / 2;
                    l = ru - k + p + 1;
                    al = (UQ[ru] - UQ[ru - k]) / (UQ[ru - k + p + 1] - UQ[ru - k]);
                    be = (UQ[ru] - UQ[ru - k + 1]) / (UQ[ru - k + p + 2] - UQ[ru - k + 1]);
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
                    /* Compute maximum of basis function */

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
                    if( UQ[i]NEQ UQ[i + 1] )
                    {
                        tmp = max[i - ru + k] * bu;

                        for ( j = q; j <= s - q - 1; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1] )
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
                            if( UQ[i]NEQ UQ[i + 1]AND VQ[j]NEQ VQ[j + 1] )
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
                        alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (UQ[ru] - UQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[ru]);
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

                        N_CopyCPt( Qw[off][col], &Rw[col][0] );
                        N_CopyCPt( Qw[last + 1][col], &Rw[col][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Qw[i][col], oma[i - first], Rw[col][ii - 1], &Rw[col][ii] );
                            N_Combine2CPts( bet[j - first], Qw[j][col], omb[j - first], Rw[col][jj + 1], &Rw[col][jj] );
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
                            N_Combine2CPts( lam, Rw[col][jj + 1], oml, Rw[col][ii - 1], &Rw[col][jj + 1] );
                        }
                    } /* End for each row */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        bru[ru] = NOREM;
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
                                N_CopyCPt( Rw[col][i - off], &Qw[i][col] );
                                N_CopyCPt( Rw[col][j - off], &Qw[j][col] );
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
                            if( VQ[j]NEQ VQ[j + 1] )
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
                        UQ[i - 1] = UQ[i];

                        for ( j = q; j <= m; j++ )
                            er[i - 1][j] = er[i][j];
                    }

                    for ( col = 0; col <= m; col++ )
                    {
                        for ( i = fout + 1; i <= n; i++ )
                        {
                            N_CopyCPt( Qw[i][col], &Qw[i - 1][col] );
                        }
                    }

                    n--;
                    r--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_UDIR )
                    {
                        if( n EQ p )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( ru - p, p + 1 );
                    l = NL_MIN( n, ru + p - su );

                    for ( i = k; i <= l; i++ )
                    {
                        if( UQ[i]NEQ UQ[i + 1]AND bru[i]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, i, sru[i], 0, m, NL_UDIR, &bru[i] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( j = q + 1; j <= s - q - 1; j++ )
                        {
                            if( VQ[j]NEQ VQ[j + 1]AND brv[j]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, j, srv[j], first, last, NL_VDIR, &brv[j] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    bru[ru] = NOREM;
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */

                rmf = NL_TRUE;

                if( (q + sv) % 2 )
                {
                    /* Compute maximums of basis functions over each span */

                    k = (q + sv + 1) / 2;
                    l = rv - k + q + 1;
                    al = (VQ[rv] - VQ[rv - k]) / (VQ[rv - k + q + 1] - VQ[rv - k]);
                    be = (VQ[rv] - VQ[rv - k + 1]) / (VQ[rv - k + q + 2] - VQ[rv - k + 1]);
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
                    /* Compute maximum of basis function */

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
                    if( VQ[j]NEQ VQ[j + 1] )
                    {
                        tmp = max[j - rv + k] * bv;

                        for ( i = p; i <= r - p - 1; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1] )
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
                            if( VQ[j]NEQ VQ[j + 1]AND UQ[i]NEQ UQ[i + 1] )
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
                        alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (VQ[rv] - VQ[i]);
                        oma[i - first] = 1.0 - alf[i - first];
                        bet[j - first] = (VQ[j + q + 1] - VQ[j]) / (VQ[j + q + 1] - VQ[rv]);
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

                        N_CopyCPt( Qw[row][off], &Rw[row][0] );
                        N_CopyCPt( Qw[row][last + 1], &Rw[row][last + 1 - off] );

                        /* Get new control points for one removal step */

                        while( (j - i)GT 0 )
                        {
                            N_Combine2CPts( alf[i - first], Qw[row][i], oma[i - first], Rw[row][ii - 1], &Rw[row][ii] );
                            N_Combine2CPts( bet[j - first], Qw[row][j], omb[j - first], Rw[row][jj + 1], &Rw[row][jj] );
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
                            N_Combine2CPts( lam, Rw[row][jj + 1], oml, Rw[row][ii - 1], &Rw[row][jj + 1] );
                        }
                    } /* End for each column */

                    /* See if weights are in the allowable range  */

                    if( wfl EQ NL_FALSE )
                    {
                        brv[rv] = NOREM;
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
                                N_CopyCPt( Rw[row][i - off], &Qw[row][i] );
                                N_CopyCPt( Rw[row][j - off], &Qw[row][j] );
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
                            if( UQ[i]NEQ UQ[i + 1] )
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
                        VQ[j - 1] = VQ[j];

                        for ( i = p; i <= n; i++ )
                            er[i][j - 1] = er[i][j];
                    }

                    for ( row = 0; row <= n; row++ )
                    {
                        for ( j = fout + 1; j <= m; j++ )
                        {
                            N_CopyCPt( Qw[row][j], &Qw[row][j - 1] );
                        }
                    }

                    m--;
                    s--;
                    N_SrfSetSizeIndices( surQ, n, m, p, q, r, s );

                    /* If no more internal knots -> finished */

                    if( dir EQ NL_VDIR )
                    {
                        if( m EQ q )
                            break;
                    }
                    else if( dir EQ NL_UVDIR )
                    {
                        if( n EQ p AND m EQ q )
                            break;
                    }

                    /* Update error bounds */

                    k = NL_MAX( rv - q, q + 1 );
                    l = NL_MIN( m, rv + q - sv );

                    for ( j = k; j <= l; j++ )
                    {
                        if( VQ[j]NEQ VQ[j + 1]AND brv[j]NEQ NOREM )
                        {
                            error = N_SrfRemoveOneKnot( surQ, j, srv[j], 0, n, NL_VDIR, &brv[j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( i = p + 1; i <= r - p - 1; i++ )
                        {
                            if( UQ[i]NEQ UQ[i + 1]AND bru[i]NEQ NOREM )
                            {
                                error = N_SrfRemoveOneKnot( surQ, i, sru[i], first, last, NL_UDIR, &bru[i] );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }
                }
                else
                {
                    /* Knot is not removable */

                    brv[rv] = NOREM;
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* Compact surface */

    if( n LT ns OR m LT ms )
    {
        error = N_SrfCompress( surQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRemoveKnotsConstraints */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine determines the topology of a regular network of
     surfaces. That is, given a topologically NxM set of surfaces in  a
     linear array, this function determines the adjacency relationships
     of the set.  Optionally,  it also forces all u,v-parameterizations
     to be consistently oriented. A typical calling example is:

       NL_SURFACE  **surfs1, ***surfs2;
       NL_REAL     ***ptrans;
       NL_INDEX    **i2d1d, *row, *col, nsurfs, nr, nc, guide;
       NL_STACKS   SG;
       ...
       (load surfs1 with the surfaces, allocate row and col, and)
       (allocate ptrans and set guide if u,v-orientations are to)
       (be made consistent);
       ...

       N_SrfNetworkTopology(surfs1,nsurfs,&surfs2,&nr,&nc,&i2d1d,row,col,
              guide,ptrans,&SG);

     Note that surfs2 and i2d1d are allocated in this routine; row, col
     and ptrans must be allocated in the calling routine:
       row    = N_AllocInt1dArray(nsurfs,&SG);
       col    = N_AllocInt1dArray(nsurfs,&SG);
       ptrans = N_AllocReal3dArray(nsurfs,1,2,&SG);

     This routine uses the global tolerance, NL_MTOL,  to  compare surface
     corner points for equality.


   ACCESS:
   
     surfs1 , input  ,  linear array of surfaces
     nsurfs , input  ,  there are nsurfs+1 surfaces 
     surfs2 , output ,  2D array of surfaces;  the 2D indices imply the
                        the topological adjacency relationships between
                        the surfaces
     nr,nc  , output ,  high index of rows and columns of surfaces
     i2d1d  , output ,  map from 2D index to 1D index of each surface
     row,col, output ,  map from 1D index to 2D index of each surface
     guide  , input  ,  flag/index:
                         n  : index between 0 and nsurfs.  Surface with
                              this  index  is  assumed  to have correct
                              u,v - orientation.  All  others  will  be 
                              oriented to be consistent with this surf
                         -1 : Don't modify surface u,v-orientations
     ptrans , output ,  != NULL : Memory for u,v-maps (optional). Let u
                                  and v be the old parameters,  s and t
                                  the new ones  (for the i-th surface).
                                  Then:
                                    s = u*ptrans[i][0][0] + 
                                        v*ptrans[i][0][1] + 
                                          ptrans[i][0][2];
                                    t = u*ptrans[i][1][0] + 
                                        v*ptrans[i][1][1] + 
                                          ptrans[i][1][2];
                        == NULL : Parameter maps not returned
     SG     , output ,  stack for surfs2 and i2d1d


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfNetworkTopology( NL_SURFACE ** surfs1, NL_INDEX nsurfs, NL_SURFACE **** surfs2, NL_INDEX *nr, NL_INDEX *nc, NL_INDEX *** i2d1d, NL_INDEX *row, NL_INDEX *col, NL_INDEX guide, NL_REAL *** ptrans, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfNetworkTopology");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, ** surcors, ** neigh, ncp, k1, k2, k3, k4, k5, k6, nrg = 0, ncg = 0, nrr, ncc, ** ind21, uclosed, vclosed;

    NL_SURFACE *** surs2;

    NL_POINT *corners, P;

    NL_REAL mtol2, dd, us = 0.0, ue = 0.0, vs = 0.0, ve = 0.0;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error conditions */

    if( guide GT nsurfs )
        NL_ERROR( NL_INP_ERR );

    if( nsurfs LT 0 )
        NL_ERROR( NL_INP_ERR );

    if( row EQ NULL OR col EQ NULL )
        NL_ERROR( NL_INP_ERR );

    /* Compute all distinct corner points */

    mtol2 = NL_MTOL * NL_MTOL;

    corners = N_AllocPt1dArray( 4 * (nsurfs + 1), &SL );

    if( corners EQ NULL )
        NL_QUIT;

    surcors = N_AllocInt2dArray( nsurfs, 3, &SL );

    if( surcors EQ NULL )
        NL_QUIT;

    ncp = -1;

    for ( ii = 0; ii <= nsurfs; ii++ )
    {
        N_SrfGetParameterBounds( surfs1[ii], &us, &ue, &vs, &ve );

        error = N_SrfEvalPt( surfs1[ii], us, vs, NL_LEFT, NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        if( ii EQ 0 AND ncp LT 0 ) /* first corner point */
        {
            ncp = 0;
            corners[0] = P;
            surcors[ii][0] = 0;
        }
        else
        { /* general case */
            for ( jj = 0; jj <= ncp; jj++ )
            {
                N_DistSqPtPt( P, corners[jj], &dd );

                if( dd LE mtol2 )
                    break;
            }

            if( jj LE ncp )
            {
                surcors[ii][0] = jj;
            }
            else
            {
                ncp += 1;
                corners[ncp] = P;
                surcors[ii][0] = ncp;
            }
        }

        error = N_SrfEvalPt( surfs1[ii], ue, vs, NL_LEFT, NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        for ( jj = 0; jj <= ncp; jj++ )
        {
            N_DistSqPtPt( P, corners[jj], &dd );

            if( dd LE mtol2 )
                break;
        }

        if( jj LE ncp )
        {
            surcors[ii][1] = jj;
        }
        else
        {
            ncp += 1;
            corners[ncp] = P;
            surcors[ii][1] = ncp;
        }

        error = N_SrfEvalPt( surfs1[ii], ue, ve, NL_LEFT, NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        for ( jj = 0; jj <= ncp; jj++ )
        {
            N_DistSqPtPt( P, corners[jj], &dd );

            if( dd LE mtol2 )
                break;
        }

        if( jj LE ncp )
        {
            surcors[ii][2] = jj;
        }
        else
        {
            ncp += 1;
            corners[ncp] = P;
            surcors[ii][2] = ncp;
        }

        error = N_SrfEvalPt( surfs1[ii], us, ve, NL_LEFT, NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        for ( jj = 0; jj <= ncp; jj++ )
        {
            N_DistSqPtPt( P, corners[jj], &dd );

            if( dd LE mtol2 )
                break;
        }

        if( jj LE ncp )
        {
            surcors[ii][3] = jj;
        }
        else
        {
            ncp += 1;
            corners[ncp] = P;
            surcors[ii][3] = ncp;
        }
    }

    /* Now build up the neighbors array */

    neigh = N_AllocInt2dArray( nsurfs, 3, &SL );

    if( neigh EQ NULL )
        NL_QUIT;

    for ( ii = 0; ii <= nsurfs; ii++ )
        for ( jj = 0; jj <= 3; jj++ )
            neigh[ii][jj] = -2;

    for ( ii = 0; ii <= nsurfs; ii++ )
    {
        for ( jj = 0; jj <= 3; jj++ )
        {
            if( neigh[ii][jj]GT - 2 )
                continue;         /* already processed */

            k1 = surcors[ii][jj]; /* indices into corners */
            k2 = surcors[ii][(jj + 1) % 4];

            if( k1 EQ k2 )
            {
                neigh[ii][jj] = -1; /* pole, no neighbor */
                continue;
            }

            k3 = surcors[ii][(jj + 3) % 4]; /* opposite edge */
            k4 = surcors[ii][(jj + 2) % 4];

            if( k1 EQ k3 AND k2 EQ k4 )
            {
                neigh[ii][jj] = -1; /* closed, no neighbor */
                continue;
            }

            for ( kk = 0; kk <= nsurfs; kk++ )
            {
                if( kk EQ ii )
                    continue;

                k3 = 0;

                for ( k4 = 0; k4 <= 3; k4++ )
                {
                    if( surcors[kk][k4]EQ k1 OR surcors[kk][k4]EQ k2 )
                    {
                        k3 += 1;

                        if( k3 EQ 2 )
                            break;
                    }
                }

                if( k3 EQ 2 )
                {
                    neigh[ii][jj] = kk; /* this is the neighbor surface */

                    if( k4 EQ 1 )
                        neigh[kk][0] = ii;

                    else if( k4 EQ 2 )
                        neigh[kk][1] = ii;

                    else if( surcors[kk][0]EQ k1 OR surcors[kk][0]EQ k2 )
                        neigh[kk][3] = ii;

                    else
                        neigh[kk][2] = ii;
                    break;
                }
            }
        }
    }

    /* Now determine 2-dimensionality (nr,nc) */

    nrr = ncc = 0;

    uclosed = vclosed = 0;

    for ( ii = 0; ii <= 3; ii++ )
    {
        if( ii EQ 2 AND vclosed EQ 1 )
        {
            nrg = 0;
            continue;
        }

        if( ii EQ 3 AND uclosed EQ 1 )
        {
            ncg = 0;
            continue;
        }

        if( guide GE 0 )
        {
            k1 = k3 = guide;
        }
        else
        {
            k1 = k3 = 0;
        }

        k4 = neigh[k3][ii];

        k5 = 0;

        while( k4 GE 0 AND k4 NEQ k1 )
        {
            k5 += 1;

            if( ii % 2 EQ 0 )
                nrr += 1;
            else
                ncc += 1;

            for ( kk = 0; kk <= 3; kk++ )
                if( neigh[k4][kk]EQ k3 )
                    break;

            if( kk GT 3 )
                NL_ERROR( NL_IND_ERR );

            kk = (kk + 2) % 4;

            k3 = k4;
            k4 = neigh[k4][kk];
        }

        if( k4 EQ k1 AND ii EQ 0 )
            vclosed = 1;

        if( k4 EQ k1 AND ii EQ 1 )
            uclosed = 1;

        if( ii EQ 0 )
            nrg = k5; /* 2D indices of the guide surface */

        if( ii EQ 3 )
            ncg = k5;
    }

    if( (nrr + 1) * (ncc + 1) - 1 NEQ nsurfs )
        NL_ERROR( NL_IND_ERR );

    *nr = nrr;
    *nc = ncc;

    /* Allocate surfs2 and i2d1d and load them, and row and col */

    surs2 = N_Alloc2dArraySrfPtrs( nrr, ncc, SG );

    if( surs2 EQ NULL )
        NL_QUIT;

    ind21 = N_AllocInt2dArray( nrr, ncc, SG );

    if( ind21 EQ NULL )
        NL_QUIT;

    *surfs2 = surs2;
    *i2d1d = ind21;

    for ( ii = 0; ii <= nrr; ii++ )
        for ( jj = 0; jj <= ncc; jj++ )
            surs2[ii][jj] = NULL;

    for ( ii = 0; ii <= nsurfs; ii++ )
    {
        row[ii] = col[ii] = -10;
    }

    if( guide GE 0 )
        k3 = guide;
    else
        k3 = 0;

    surs2[nrg][ncg] = N_AllocSrf( SG );

    if( surs2[nrg][ncg]EQ NULL )
        NL_QUIT;
    N_SrfInitArrays( surs2[nrg][ncg] );

    error = N_SrfCopy( surfs1[k3], surs2[nrg][ncg], SG );

    if( error EQ NL_YES )
        NL_OUT;

    ind21[nrg][ncg] = k3;
    row[k3] = nrg;
    col[k3] = ncg;

    k4 = neigh[k3][1]; /* go right from guide surface */

    k5 = 0;

    while( k4 GE 0 AND ncg + k5 LT ncc )
    {

        k5 += 1;

        for ( kk = 0; kk <= 3; kk++ )
            if( neigh[k4][kk]EQ k3 )
                break;

        if( kk GT 3 )
            NL_ERROR( NL_IND_ERR );

        kk = (kk + 2) % 4;

        k3 = k4;
        k4 = neigh[k4][kk];

        surs2[nrg][ncg + k5] = N_AllocSrf( SG );

        if( surs2[nrg][ncg + k5]EQ NULL )
            NL_QUIT;
        N_SrfInitArrays( surs2[nrg][ncg + k5] );

        error = N_SrfCopy( surfs1[k3], surs2[nrg][ncg + k5], SG );

        if( error EQ NL_YES )
            NL_OUT;

        ind21[nrg][ncg + k5] = k3;
        row[k3] = nrg;
        col[k3] = ncg + k5;
    }

    if( guide GE 0 )
        k3 = guide;
    else
        k3 = 0;

    k4 = neigh[k3][3]; /* go left from guide surface */

    k5 = 0;

    while( k4 GE 0 AND ncg - k5 GT 0 )
    {

        k5 += 1;

        for ( kk = 0; kk <= 3; kk++ )
            if( neigh[k4][kk]EQ k3 )
                break;

        if( kk GT 3 )
            NL_ERROR( NL_IND_ERR );

        kk = (kk + 2) % 4;

        k3 = k4;
        k4 = neigh[k4][kk];

        surs2[nrg][ncg - k5] = N_AllocSrf( SG );

        if( surs2[nrg][ncg - k5]EQ NULL )
            NL_QUIT;
        N_SrfInitArrays( surs2[nrg][ncg - k5] );

        error = N_SrfCopy( surfs1[k3], surs2[nrg][ncg - k5], SG );

        if( error EQ NL_YES )
            NL_OUT;

        ind21[nrg][ncg - k5] = k3;
        row[k3] = nrg;
        col[k3] = ncg - k5;
    }

    for ( ii = 0; ii <= ncc; ii++ ) /* do each column now, above */
    {                               /* and below row nrg         */
        k3 = ind21[nrg][ii];

        k4 = neigh[k3][0];          /* go down from this surface */

        k5 = 0;

        while( k4 GE 0 AND nrg - k5 GT 0 )
        {

            k5 += 1;

            for ( kk = 0; kk <= 3; kk++ )
                if( neigh[k4][kk]EQ k3 )
                    break;

            if( kk GT 3 )
                NL_ERROR( NL_IND_ERR );

            kk = (kk + 2) % 4;

            k3 = k4;
            k4 = neigh[k4][kk];

            surs2[nrg - k5][ii] = N_AllocSrf( SG );

            if( surs2[nrg - k5][ii]EQ NULL )
                NL_QUIT;
            N_SrfInitArrays( surs2[nrg - k5][ii] );

            error = N_SrfCopy( surfs1[k3], surs2[nrg - k5][ii], SG );

            if( error EQ NL_YES )
                NL_OUT;

            ind21[nrg - k5][ii] = k3;
            row[k3] = nrg - k5;
            col[k3] = ii;
        }

        k3 = ind21[nrg][ii];

        k4 = neigh[k3][2]; /* go up from this surface */

        k5 = 0;

        while( k4 GE 0 AND nrg + k5 LT nrr )
        {

            k5 += 1;

            for ( kk = 0; kk <= 3; kk++ )
                if( neigh[k4][kk]EQ k3 )
                    break;

            if( kk GT 3 )
                NL_ERROR( NL_IND_ERR );

            kk = (kk + 2) % 4;

            k3 = k4;
            k4 = neigh[k4][kk];

            surs2[nrg + k5][ii] = N_AllocSrf( SG );

            if( surs2[nrg + k5][ii]EQ NULL )
                NL_QUIT;
            N_SrfInitArrays( surs2[nrg + k5][ii] );

            error = N_SrfCopy( surfs1[k3], surs2[nrg + k5][ii], SG );

            if( error EQ NL_YES )
                NL_OUT;

            ind21[nrg + k5][ii] = k3;
            row[k3] = nrg + k5;
            col[k3] = ii;
        }
    }

    /* Error check */

    for ( ii = 0; ii <= nrr; ii++ )
        for ( jj = 0; jj <= ncc; jj++ )
        {
            if( surs2[ii][jj]EQ NULL )
                NL_ERROR( NL_IND_ERR );
        }

    for ( ii = 0; ii <= nsurfs; ii++ )
    {
        if( row[ii]EQ - 10 )
            NL_ERROR( NL_IND_ERR );
    }

    /* check if we are finished */

    if( guide LT 0 )
        NL_OUT;

    /* Orient the u,v parameterizations consistently and build */
    /* the parameter maps                                      */

    k1 = guide;

    if( ptrans NEQ NULL )
    {

        ptrans[k1][0][0] = 1.0;
        ptrans[k1][0][1] = 0.0;
        ptrans[k1][0][2] = 0.0;

        ptrans[k1][1][0] = 0.0;
        ptrans[k1][1][1] = 1.0;
        ptrans[k1][1][2] = 0.0;
    }

    for ( jj = 0; jj < 2; jj++ )
    {
        if( jj EQ 0 )
        {
            k3 = ncc - ncg; /* go right from guide surface */
            k4 = 1;
        }
        else
        {
            k3 = ncg; /* go left from guide surface */
            k4 = -1;
        }

        k1 = guide;

        for ( ii = 1; ii <= k3; ii++ )
        {
            k2 = ind21[nrg][ncg + k4 * ii];

            for ( kk = 0; kk <= 3; kk++ )
                if( neigh[k2][kk]EQ k1 )
                    break;

            if( kk GT 3 )
                NL_ERROR( NL_IND_ERR ); /* kk is neighboring edge */

            if( ptrans NEQ NULL )
            {
                N_SrfGetParameterBounds( surs2[nrg][ncg + k4 * ii], &us, &ue, &vs, &ve );

                ptrans[k2][0][0] = 1.0;
                ptrans[k2][0][1] = 0.0;
                ptrans[k2][0][2] = 0.0;

                ptrans[k2][1][0] = 0.0;
                ptrans[k2][1][1] = 1.0;
                ptrans[k2][1][2] = 0.0;
            }

            if( kk EQ 0 OR kk EQ 2 )
            { /* switch u,v-parameterization */

                error = N_SwapUV( surs2[nrg][ncg + k4 * ii], SG );

                if( error EQ NL_YES )
                    NL_OUT;

                k6 = surcors[k2][1];
                surcors[k2][1] = surcors[k2][3];
                surcors[k2][3] = k6;

                if( ptrans NEQ NULL )
                {
                    ptrans[k2][1][0] = 1.0;
                    ptrans[k2][1][1] = 0.0;
                    ptrans[k2][1][2] = 0.0;

                    ptrans[k2][0][0] = 0.0;
                    ptrans[k2][0][1] = 1.0;
                    ptrans[k2][0][2] = 0.0;

                    dd = us;
                    us = vs;
                    vs = dd;
                    dd = ue;
                    ue = ve;
                    ve = dd;
                }
            }

            if( (jj EQ 0 AND( kk EQ 1 OR kk EQ 2 ))OR( jj EQ 1 AND( kk EQ 0 OR kk EQ 3 ) ) )
            { /* reverse u-parameterization */

                error = N_SrfReverse( surs2[nrg][ncg + k4 * ii], NL_UDIR, surs2[nrg][ncg + k4 * ii], SG );

                if( error EQ NL_YES )
                    NL_OUT;

                k6 = surcors[k2][1];
                surcors[k2][1] = surcors[k2][0];
                surcors[k2][0] = k6;
                k6 = surcors[k2][2];
                surcors[k2][2] = surcors[k2][3];
                surcors[k2][3] = k6;

                if( ptrans NEQ NULL )
                {
                    ptrans[k2][0][0] = -ptrans[k2][0][0];
                    ptrans[k2][0][1] = -ptrans[k2][0][1];
                    ptrans[k2][0][2] = -ptrans[k2][0][2] + us + ue;
                }
            }

            if( (jj EQ 0 AND( surcors[k2][0]EQ surcors[k1][2] ))OR( jj EQ 1 AND( surcors[k2][2]EQ surcors[k1][0] ) ) )
            { /* reverse v-parameterization */

                error = N_SrfReverse( surs2[nrg][ncg + k4 * ii], NL_VDIR, surs2[nrg][ncg + k4 * ii], SG );

                if( error EQ NL_YES )
                    NL_OUT;

                k6 = surcors[k2][1];
                surcors[k2][1] = surcors[k2][2];
                surcors[k2][2] = k6;
                k6 = surcors[k2][0];
                surcors[k2][0] = surcors[k2][3];
                surcors[k2][3] = k6;

                if( ptrans NEQ NULL )
                {
                    ptrans[k2][1][0] = -ptrans[k2][1][0];
                    ptrans[k2][1][1] = -ptrans[k2][1][1];
                    ptrans[k2][1][2] = -ptrans[k2][1][2] + vs + ve;
                }
            }

            k1 = k2;
        }
    }

    for ( k5 = 0; k5 <= ncc; k5++ ) /* in each column, go up and   */
    {                               /* down from row nrg           */
        for ( jj = 0; jj < 2; jj++ )
        {
            if( jj EQ 0 )
            {
                k3 = nrr - nrg; /* go up from row nrg */
                k4 = 1;
            }
            else
            {
                k3 = nrg; /* go down from row nrg */
                k4 = -1;
            }

            k1 = ind21[nrg][k5];

            for ( ii = 1; ii <= k3; ii++ )
            {
                k2 = ind21[nrg + k4 * ii][k5];

                for ( kk = 0; kk <= 3; kk++ )
                    if( neigh[k2][kk]EQ k1 )
                        break;

                if( kk GT 3 )
                    NL_ERROR( NL_IND_ERR ); /* kk is neighboring edge */

                if( ptrans NEQ NULL )
                {
                    N_SrfGetParameterBounds( surs2[nrg + k4 * ii][k5], &us, &ue, &vs, &ve );

                    ptrans[k2][0][0] = 1.0;
                    ptrans[k2][0][1] = 0.0;
                    ptrans[k2][0][2] = 0.0;

                    ptrans[k2][1][0] = 0.0;
                    ptrans[k2][1][1] = 1.0;
                    ptrans[k2][1][2] = 0.0;
                }

                if( kk EQ 1 OR kk EQ 3 )
                { /* switch u,v-parameterization */

                    error = N_SwapUV( surs2[nrg + k4 * ii][k5], SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k6 = surcors[k2][1];
                    surcors[k2][1] = surcors[k2][3];
                    surcors[k2][3] = k6;

                    if( ptrans NEQ NULL )
                    {
                        ptrans[k2][1][0] = 1.0;
                        ptrans[k2][1][1] = 0.0;
                        ptrans[k2][1][2] = 0.0;

                        ptrans[k2][0][0] = 0.0;
                        ptrans[k2][0][1] = 1.0;
                        ptrans[k2][0][2] = 0.0;

                        dd = us;
                        us = vs;
                        vs = dd;
                        dd = ue;
                        ue = ve;
                        ve = dd;
                    }
                }

                if( (jj EQ 0 AND( kk EQ 1 OR kk EQ 2 ))OR( jj EQ 1 AND( kk EQ 0 OR kk EQ 3 ) ) )
                { /* reverse v-parameterization */

                    error = N_SrfReverse( surs2[nrg + k4 * ii][k5], NL_VDIR, surs2[nrg + k4 * ii][k5], SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k6 = surcors[k2][1];
                    surcors[k2][1] = surcors[k2][2];
                    surcors[k2][2] = k6;
                    k6 = surcors[k2][0];
                    surcors[k2][0] = surcors[k2][3];
                    surcors[k2][3] = k6;

                    if( ptrans NEQ NULL )
                    {
                        ptrans[k2][1][0] = -ptrans[k2][1][0];
                        ptrans[k2][1][1] = -ptrans[k2][1][1];
                        ptrans[k2][1][2] = -ptrans[k2][1][2] + vs + ve;
                    }
                }

                if( (jj EQ 0 AND( surcors[k2][0]EQ surcors[k1][2] ))OR( jj EQ 1 AND( surcors[k2][2]EQ surcors[k1][0] ) ) )
                { /* reverse u-parameterization */

                    error = N_SrfReverse( surs2[nrg + k4 * ii][k5], NL_UDIR, surs2[nrg + k4 * ii][k5], SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k6 = surcors[k2][1];
                    surcors[k2][1] = surcors[k2][0];
                    surcors[k2][0] = k6;
                    k6 = surcors[k2][2];
                    surcors[k2][2] = surcors[k2][3];
                    surcors[k2][3] = k6;

                    if( ptrans NEQ NULL )
                    {
                        ptrans[k2][0][0] = -ptrans[k2][0][0];
                        ptrans[k2][0][1] = -ptrans[k2][0][1];
                        ptrans[k2][0][2] = -ptrans[k2][0][2] + us + ue;
                    }
                }

                k1 = k2;
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfNetworkTopology */
#endif // NLIB_UNUSED
