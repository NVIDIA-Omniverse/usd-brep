// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfFit.c: Surface fitting routines                                 */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_DataStruct.h>
#include <NL_Globals.h>

#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */


/* Prototypes only referenced within SrfFit.c */
static NL_REAL ST_CalcAreaRotPts( NL_REAL );

static NL_FLAG ST_KnotInsertInPlace(NL_CPOINT*, NL_INDEX, NL_DEGREE, NL_KNOTVECTOR*, NL_REAL);
static NL_FLAG ST_CrvFitPts(NL_VOID*, NL_FLAG, NL_RMATRIX*, NL_CPOINT*);
static NL_FLAG ST_CrvInterpDerivsKnots(NL_VOID*, NL_INDEX, NL_FLAG, NL_DEGREE, NL_VOID*, NL_VOID*, NL_KNOTVECTOR*, NL_RMATRIX*, NL_CPOINT*);
static NL_FLAG ST_CrvInterpTangentsKnots(NL_VOID*, NL_INDEX, NL_FLAG, NL_DEGREE, NL_VOID*, NL_FLAG, NL_KNOTVECTOR*, NL_RMATRIX*, NL_CPOINT*);

static NL_INDEX ST_BestSubdInStrip(NL_INDEX, NL_INDEX**, NL_INDEX, NL_INDEX*, NL_INDEX*, NL_REAL*);
static NL_INDEX ST_NumPtsInStrip(NL_REAL*, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL, NL_INDEX*, NL_INDEX*);
static NL_INDEX ST_BinarySearch(NL_REAL*, NL_INDEX, NL_INDEX, NL_REAL, NL_FLAG);
static NL_INDEX ST_NumPtsInBox(NL_REAL*, NL_INDEX*, NL_INDEX, NL_INDEX, NL_REAL*, NL_REAL, NL_REAL, NL_REAL, NL_REAL);

static NL_VOID ST_AddKnotsStrip(NL_REAL*, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL, NL_INDEX, NL_INDEX, NL_REAL*,
    NL_INDEX*, NL_REAL*, NL_INDEX*, NL_INDEX*, NL_INDEX*, NL_INDEX*);

#if NLIB_UNUSED
static NL_REAL ST_CalcAreaSphereRotPtsGlobal( NL_REAL );
static NL_REAL ST_CalcAreaSphereRotPtsLocal( NL_REAL );
#endif // NLIB_UNUSED

/*****************************************************************************/
/* N_FitCalcSrfParamValues: Parametrization for global surface interpolation */
/*****************************************************************************/


/*******************************************************************//**


   DESCRIPTION:

     This  fitting routine computes parameter values for global surface 
     interpolation assigning each data point a surface UV parameter point.
     
     This fitting routine assumes the input points are from a regularly 
     sampled grid and assigns each row and col of sample points to an
     iso-parameter curve in one of three ways:
       NL_UNIFORM     = assign u,v values from index values.
       NL_CHORDLENGTH = assign u,v values from 3d spacing between points.
       NL_CENTRIPETAL = assign u,v values from the square root of the 3d 
                          spacing between points.  
     
     In order to facilitate both NL_POINT and NL_CPOINT objects,
     the routine employs a NL_VOID pointer as input parameter and assumes
     that the calling routine typecasts the given pointer to the appro-
     priate type. The calling mechanism works as follows:

       NL_CPOINT     **Pw;
       NL_POINT      **P;
       NL_INDEX      n, m;
       NL_PARAMETER  *u, *v;
       ...
       (define data point array and allocate memory for u and v);
       ...
       N_FitCalcSrfParamValues((NL_VOID **)P ,n,m,NL_EPOINT,NL_CHORDLENGTH,u,v);
       N_FitCalcSrfParamValues((NL_VOID **)Pw,n,m,NL_HPOINT,NL_CENTRIPETAL,u,v);

     It is assumed that memory to store the  parameters is allocated in
     the calling routine. A recommended default for the parametrization 
     type is NL_CHORDLENGTH.


   ACCESS:
   
     A   , input  ,  NL_VOID pointer representing either NL_POINT or NL_CPOINT
     n,m , input  ,  Highest indexes in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     u,v , output ,  Parameters corresponding to data points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcSrfParamValues
  (NL_VOID    ** A,     /* in : an array[n+1][m+1] of either NL_POINT or NL_CPOINT objects */
   NL_INDEX      n,     /* in : A highest row index value */
   NL_INDEX      m,     /* in : A highest col index value */
   NL_FLAG       ptp,   /* in : type of objects in A, NL_EPOINT = A contains NL_POINT objects  */
                        /*                            NL_HPOINT = A contains NL_CPOINT objects */
   NL_FLAG       par,   /* in : select param assignment method */ 
                        /*        NL_UNIFORM    : u[i] = i*d */
                        /*                        v[i] = j*d */
                        /*        NL_CHORDLENGTH: u[i] = u[i-1] + Avg_j(Dist(P[i][j],P[i-1][j]) */
                        /*                        v[j] = v[j-1] + Avg_i(Dist(P[i][j],P[i][j-1]) */
                        /*        NL_CENTRIPETAL: u[i] = u[i-1] + Avg_j(sqrt(Dist(P[i][j],P[i-1][j])) */
                        /*                        v[j] = v[j-1] + Avg_i(sqrt(Dist(P[i][j],P[i][j-1])) */
   NL_PARAMETER *u,     /* out: u[i] parameters for all P[i][*] points, sized:[n+1] */
   NL_PARAMETER *v )    /* out: v[j] parameters for all P[*][j] points, sized:[m+1] */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcSrfParamValues");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, num;

    NL_REAL *dist, sum, d;

    NL_POINT ** P = NULL;

    NL_CPOINT ** Pw = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check flags */

    switch( par )
    {
        case NL_UNIFORM:
            break;

        case NL_CHORDLENGTH:
            break;

        case NL_CENTRIPETAL:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    switch( ptp )
    {
        case NL_EPOINT:
            P = (NL_POINT ** )A;
            break;

        case NL_HPOINT:
            Pw = (NL_CPOINT ** )A;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Compute uniform parametrization */

    u[0] = 0.0;
    u[n] = 1.0;
    v[0] = 0.0;
    v[m] = 1.0;

    if( par EQ NL_UNIFORM )
    {
        d = 1.0 / n;

        for ( i = 1; i < n; i++ )
            u[i] = i * d;

        d = 1.0 / m;

        for ( j = 1; j < m; j++ )
            v[j] = j * d;

        NL_OUT;
    }

    /*******************************************************/
    /* Compute chord length or centripetal parametrization */
    /*******************************************************/

    dist = N_AllocReal1dArray( NL_MAX( n, m ), &SL );

    if( dist EQ NULL )
        NL_QUIT;

    /* Get u-directional parameters */

    for ( i = 1; i < n; i++ )
        u[i] = 0.0;

    num = m + 1;

    for ( j = 0; j <= m; j++ )
    {
        sum = 0.0;

        for ( i = 1; i <= n; i++ )
        {
            if( ptp EQ NL_EPOINT )
                N_DistPtPt( P[i - 1][j], P[i][j], &dist[i] );
            else
                N_DistCptCptHomo( Pw[i - 1][j], Pw[i][j], &dist[i] );

            if( par EQ NL_CENTRIPETAL )
                dist[i] = sqrt( dist[i] );
            sum += dist[i];
        }

        if( sum GT NL_MTOL )
        {
            d = 0.0;

            for ( i = 1; i < n; i++ )
            {
                d += dist[i];
                u[i] += d / sum;
            }
        }
        else
            num--;
    }

    if( num EQ 0 )
        NL_ERROR( NL_INP_ERR );

    for ( i = 1; i < n; i++ )
        u[i] /= num;

    /* Get v-directional parameters */

    for ( j = 1; j < m; j++ )
        v[j] = 0.0;

    num = n + 1;

    for ( i = 0; i <= n; i++ )
    {
        sum = 0.0;

        for ( j = 1; j <= m; j++ )
        {
            if( ptp EQ NL_EPOINT )
                N_DistPtPt( P[i][j - 1], P[i][j], &dist[j] );
            else
                N_DistCptCptHomo( Pw[i][j - 1], Pw[i][j], &dist[j] );

            if( par EQ NL_CENTRIPETAL )
                dist[j] = sqrt( dist[j] );
            sum += dist[j];
        }

        if( sum GT NL_MTOL )
        {
            d = 0.0;

            for ( j = 1; j < m; j++ )
            {
                d += dist[j];
                v[j] += d / sum;
            }
        }
        else
            num--;
    }

    if( num EQ 0 )
        NL_ERROR( NL_INP_ERR );

    for ( j = 1; j < m; j++ )
        v[j] /= num;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitSrfToPts: Global surface interpolation with arbitrary degrees      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface interpolating a given
     set of points. The degrees are arbitrary and the type of parametri-
     zation can be  chosen. If the output surface is  initialized to the
     NULL surface, memory is allocated  locally. Otherwise it is checked
     if enough memory is passed in. A typical calling example is:

       NL_POINT    **P;
       NL_INDEX    n, m;
       NL_DEGREE   p, q;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define array P and choose degrees);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfToPts(P,n,m,p,q,NL_CHORDLENGTH,&sur,&SG);

     A recommended default for the parametrization type is  NL_CHORDLENGTH.


   ACCESS:
   
     P   , input  ,  Points to be interpolated
     n,m , input  ,  Highest indexes in P
     p,q , input  ,  Degrees of interpolating surface
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     sur , output ,  Interpolating surface
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfToPts
  (NL_POINT  ** P,     /* in : grid of sample points, sized[n+1][m+1] */
   NL_INDEX     n,     /* in : max 1st index of P grid */
   NL_INDEX     m,     /* in : max 2nd index of P grid */
   NL_DEGREE    p,     /* in : output surface degree U */
   NL_DEGREE    q,     /* in : output surface degree V */
   NL_FLAG      par,   /* in : map flag: NL_UNIFORM    : Uniform parametrization      */
                       /*                NL_CHORDLENGTH: Chord length parametrization */
                       /*                NL_CENTRIPETAL: Centripetal parametrization  */
   NL_SURFACE  *sur,   /* out: approximating surface */
   NL_STACKS   *SG )   /* in : sur's memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfToPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, ub, vb, usb, vsb;

    NL_REAL ** A, *N;

    NL_PARAMETER *u, *v;

    NL_CPOINT ** Pw, *Qw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for surface memory */

    if( p GT n OR q GT m )
        NL_ERROR( NL_INP_ERR );

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsKnotVectorAndKnots( sur, &Pw, &knu, &knv, &N, &N );

    /* Get parameters and compute knot vectors */

    u = N_AllocReal1dArray( n, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( m, &SL );

    if( v EQ NULL )
        NL_QUIT;

    error = N_FitCalcSrfParamValues( (NL_VOID ** )P, n, m, NL_EPOINT, par, u, v );

    if( error EQ NL_YES )
        NL_OUT;

    N_FitCrvCalcKnotVector( u, n, p, knu );
    N_FitCrvCalcKnotVector( v, m, q, knv );

    /* Get auxiliary arrays */

    Qw = N_AllocCPt1dArray( NL_MAX( n, m ), &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( NL_MAX( p, q ), &SL );

    if( N EQ NULL )
        NL_QUIT;

    /* Interpolate in u-direction */

    if( p EQ 1 )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_PtToCPt( P[i][j], &Pw[i][j] );
        }
    }
    else
    {
        ub = 2 * p - 1;
        usb = p - 1;

        error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, ub, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cm, &A );

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j < ub; j++ )
                A[i][j] = 0.0;
        }

        A[0][usb] = 1.0;
        A[n][usb] = 1.0;

        for ( i = 1; i < n; i++ )
        {
            error = N_BasisEval( knu, p, u[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            for ( k = 0; k <= p; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= m; j++ )
        {
            error = N_FitCrvCPtsFromSrfData( (NL_VOID ** )P, n, m, NL_EPOINT, j, NL_UDIR, &cm, Qw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Qw[i], &Pw[i][j] );
        }
    }

    /* Interpolate in v-direction */

    if( q GT 1 )
    {
        vb = 2 * q - 1;
        vsb = q - 1;

        error = N_SetRealMatrix( &cm, m, m, NL_MT_BANDED, vb, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cm, &A );

        for ( i = 0; i <= m; i++ )
        {
            for ( j = 0; j < vb; j++ )
                A[i][j] = 0.0;
        }

        A[0][vsb] = 1.0;
        A[m][vsb] = 1.0;

        for ( i = 1; i < m; i++ )
        {
            error = N_BasisEval( knv, q, v[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            for ( k = 0; k <= q; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= n; i++ )
        {
            error = N_FitCrvCPtsFromSrfData( (NL_VOID ** )Pw, n, m, NL_HPOINT, i, NL_VDIR, &cm, Qw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= m; j++ )
                N_CopyCPt( Qw[j], &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitSrfToPtsKNOTS: Surface interpolation with given knot vectors            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface interpolating a given
     set of points. This is a general routine requiring the input of two
     knot vectors, parameters at interpolation points, and  the input of
     NL_POINT or NL_CPOINT data. If the output  surface is  initialized to the
     NULL surface, memory is allocated  locally. Otherwise it is checked
     if enough memory is passed in. A typical calling example is:

       NL_POINT       **P;
       NL_CPOINT      **Pw;
       NL_INDEX       n, m;
       NL_DEGREE      p, q;
       NL_PARAMETER   *u, *v;
       NL_KNOTVECTOR  *knu, *knv;
       NL_SURFACE     sur;
       NL_STACKS      SG;
       ...
       (get arrays P/Pw, u, v; define knu, knv; choose p, q);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfToPtsKnots((NL_VOID **)P ,n,m,NL_EPOINT,u,v,knu,knv,p,q,&sur,&SG);
       N_FitSrfToPtsKnots((NL_VOID **)Pw,n,m,NL_HPOINT,u,v,knu,knv,p,q,&sur,&SG);

     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT VECTORS ARE COMPUTED
     IN THE CALLING ROUTINE.


   ACCESS:
   
     A   , input  ,  Pointer to NL_POINT or NL_CPOINT data
     n,m , input  ,  Highest indexes in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     u,v , input  ,  Parameters corresponding to data points
     knu , input  ,  Knot vector in u-direction
     knv , input  ,  Knot vector in v-direction
     p,q , input  ,  Degrees of interpolating surface
     sur , output ,  Interpolating surface
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfToPtsKnots
  (NL_VOID     ** A,     /* in : Sample Point Grid, sized:[n+1][m+1] */
   NL_INDEX       n,     /* in : highest 1st A index */
   NL_INDEX       m,     /* in : highest 2nd A index */
   NL_FLAG        ptp,   /* in : A type flag: NL_EPOINT = A contains NL_POINT objects  */
                         /*                   NL_HPOINT = A contains NL_CPOINT objects */
   NL_PARAMETER  *u,     /* in : U param values for each A[i][*] point, sized:[n+1] */
   NL_PARAMETER  *v,     /* in : V param values for each A[*][j] point, sized:[n+1] */
   NL_KNOTVECTOR *knu,   /* in : output surface knot vector U */
   NL_KNOTVECTOR *knv,   /* in : output surface knot vector V */
   NL_DEGREE      p,     /* in : output surface degree U */
   NL_DEGREE      q,     /* in : output surface degree V */
   NL_SURFACE    *sur,   /* out: approximating surface */
   NL_STACKS     *SG )   /* in : sur memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfToPtsKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, ub, vb, usb, vsb, r, s, rs, ss;

    NL_REAL ** B, *N, *U, *V, *US, *VS;

    NL_POINT ** Q;

    NL_CPOINT ** Pw, ** Qw, *Rw;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for surface memory */

    N_KnotVectorGetKnots( knu, &r, &U );
    N_KnotVectorGetKnots( knv, &s, &V );
    rs = n + p + 1;
    ss = m + q + 1;

    if( r NEQ rs OR s NEQ ss )
        NL_ERROR( NL_INP_ERR );

    if( p GT n OR q GT m )
        NL_ERROR( NL_INP_ERR );

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &US, &VS );

    for ( i = 0; i <= r; i++ )
        US[i] = U[i];

    for ( j = 0; j <= s; j++ )
        VS[j] = V[j];

    /* Get auxiliary arrays */

    Rw = N_AllocCPt1dArray( NL_MAX( n, m ), &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( NL_MAX( p, q ), &SL );

    if( N EQ NULL )
        NL_QUIT;

    /* Interpolate in u-direction */

    if( p EQ 1 )
    {
        switch( ptp )
        {
            case NL_EPOINT:

                Q = (NL_POINT ** )A;

                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                        N_PtToCPt( Q[i][j], &Pw[i][j] );
                }
                break;

            case NL_HPOINT:

                Qw = (NL_CPOINT ** )A;

                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                        N_CopyCPt( Qw[i][j], &Pw[i][j] );
                }
                break;

            default:

                NL_ERROR( NL_CAL_ERR );
        }
    }
    else
    {
        ub = 2 * p - 1;
        usb = p - 1;

        error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, ub, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cm, &B );

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j < ub; j++ )
                B[i][j] = 0.0;
        }

        B[0][usb] = 1.0;
        B[n][usb] = 1.0;

        for ( i = 1; i < n; i++ )
        {
            error = N_BasisEval( knu, p, u[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            if( l LT 0 OR l + p GE ub )
                NL_ERROR( NL_INP_ERR );

            for ( k = 0; k <= p; k++ )
                B[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= m; j++ )
        {
            error = N_FitCrvCPtsFromSrfData( A, n, m, ptp, j, NL_UDIR, &cm, Rw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Rw[i], &Pw[i][j] );
        }
    }

    /* Interpolate in v-direction */

    if( q GT 1 )
    {
        vb = 2 * q - 1;
        vsb = q - 1;

        error = N_SetRealMatrix( &cm, m, m, NL_MT_BANDED, vb, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cm, &B );

        for ( i = 0; i <= m; i++ )
        {
            for ( j = 0; j < vb; j++ )
                B[i][j] = 0.0;
        }

        B[0][vsb] = 1.0;
        B[m][vsb] = 1.0;

        for ( i = 1; i < m; i++ )
        {
            error = N_BasisEval( knv, q, v[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            if( l LT 0 OR l + q GE vb )
                NL_ERROR( NL_INP_ERR );

            for ( k = 0; k <= q; k++ )
                B[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= n; i++ )
        {
            error = N_FitCrvCPtsFromSrfData( (NL_VOID ** )Pw, n, m, NL_HPOINT, i, NL_VDIR, &cm, Rw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= m; j++ )
                N_CopyCPt( Rw[j], &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFLSTSQAPPROX: Global surface approximation with arbitrary degree       */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This  fitting  routine  computes  a  least-squares  NURBS   
     (n+1)x(m+1) controlPoint surface approximation to a  given set of  
     (r+1)x(s+1) points sampled from a regular grid. The approximation 
     attempts to place each row and col of grid points on isoparameter 
     curves.  The approximation routine works well when the grid sample
     points are selected regularly.
     
     The degrees are arbitrary 
     and the type of parametrization  and the number of  control  points  
     can be chosen. If the output surface is  initialized  to  the  NULL  
     surface, memory is allocated locally. Otherwise  it is  checked  if 
     enough memory is passed in. A typical calling example is:
 
       NL_POINT    **P;
       NL_INDEX    r, s, m, n;
       NL_DEGREE   p, q;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define array P, choose degrees, n and m);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqApprox(P,r,s,n,m,p,q,NL_CHORDLENGTH,&sur,&SG);
 
     A recommended default for the parametrization  type is NL_CHORDLENGTH.
 
 
   ACCESS:
   
     P   , input  ,  Points to be approximated
     r,s , input  ,  Highest indexes in P
     n,m , input  ,  Highest indexes of control point array of  approxi-
                     mating surface (must satisfy n <= r, m <= s)
     p,q , input  ,  Degrees  of  approximating  surface  (must  satisfy 
                     p <= n, q <= m)
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     sur , output ,  Approximating surface
     SG  , input  ,  sur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfLstSqApprox
  (NL_POINT ** P,      /* in : Grid of Points to be Approximated, sized:[r+1][s+1] */
   NL_INDEX    r,      /* in : highest index1 in P                              */
   NL_INDEX    s,      /* in : highest index2 in P                              */ 
   NL_INDEX    n,      /* in : Output sur ControlPoint count U,  note:( n <= r) */ 
   NL_INDEX    m,      /* in : Output sur ControlPoint count V,  note:( m <= s) */ 
   NL_DEGREE   p,      /* in : Output sur Deg U,  note:( p <= n)                */ 
   NL_DEGREE   q,      /* in : Output sur Deg V,  note:( q <= m)                */ 
   NL_FLAG     par,    /* in : NL_UNIFORM    : Uniform parametrization          */
                       /*      NL_CHORDLENGTH: Chord length parametrization     */
                       /*      NL_CENTRIPETAL: Centripetal parametrization      */
   NL_SURFACE *sur,    /* out: The fitted surface. Tries to place each P row    */
                       /*       and col of points on an isoParameter line.      */
                       /*        (n+1)x(m+1) control points, and                */   
                       /*        (n+p+1)x(m+q+1) knots                          */   
   NL_STACKS  *SG )    /* in : sur memory stack                                 */ 
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfLstSqApprox");

    NL_FLAG error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, l, ub, vb, usb, vsb, rj, sj, ej, lk, hk, lj, hj;

    NL_REAL ** A, ** NTN, *N, *ts, *te;

    NL_PARAMETER *u, *v;

    NL_POINT *Rk, *R;

    NL_CPOINT ** Pw, ** Qw, *Rkw, *Rw, *Aw;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error */

    if( n GT r OR n LT 1 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    if( m GT s OR m LT 1 OR q GT m )
        NL_ERROR( NL_INP_ERR );

    if( n EQ 1 )
    {
        if( m GT 1 AND q GT m )
            NL_ERROR( NL_INP_ERR );
    }
    else if( m EQ 1 )
    {
        if( n GT 1 AND p GT n )
            NL_ERROR( NL_INP_ERR );
    }

    else if( (n GT 1 AND m GT 1)AND( p GT n OR q GT m ) )
        NL_ERROR( NL_INP_ERR );

    /* Check for surface memory */

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &N, &N );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Get parameters and knot vectors */

    u = N_AllocReal1dArray( r, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( s, &SL );

    if( v EQ NULL )
        NL_QUIT;

    /* assign u,v param values to each sample Point assuming sample points come from a grid */
    error = N_FitCalcSrfParamValues( (NL_VOID ** )P, r, s, NL_EPOINT, par, u, v );

    if( error EQ NL_YES )
        NL_OUT;

    /*  */
    error = N_FitCalcKnotVectorCrvApprox( u, r, n, p, knu );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_FitCalcKnotVectorCrvApprox( v, s, m, q, knv );

    if( error EQ NL_YES )
        NL_OUT;

    /* Only corner points wanted */

    if( n EQ 1 AND m EQ 1 )
    {
        N_PtToCPt( P[0][0], &Pw[0][0] );
        N_PtToCPt( P[r][0], &Pw[1][0] );
        N_PtToCPt( P[0][s], &Pw[0][1] );
        N_PtToCPt( P[r][s], &Pw[1][1] );

        NL_OUT;
    }

    /* Reset data points if ruling is required */

    if( n EQ 1 )
    {
        for ( j = 0; j <= s; j++ )
            N_CopyPt( P[r][j], &P[1][j] );
    }

    if( m EQ 1 )
    {
        for ( i = 0; i <= r; i++ )
            N_CopyPt( P[i][s], &P[i][1] );
    }

    /* Allocate memory */

    ub = 2 * p + 1;
    vb = 2 * q + 1;
    usb = p;
    vsb = q;
    i = NL_MAX( r - 2, s - 2 );
    k = NL_MAX( p, q );
    j = NL_MAX( n - 2, m - 2 );
    l = NL_MAX( ub - 1, vb - 1 );

    A = N_AllocReal2dArray( i, k, &SL );

    if( A EQ NULL )
        NL_QUIT;

    NTN = N_AllocReal2dArray( j, l, &SL );

    if( NTN EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( k, &SL );

    if( N EQ NULL )
        NL_QUIT;

    ts = N_AllocReal1dArray( i, &SL );

    if( ts EQ NULL )
        NL_QUIT;

    te = N_AllocReal1dArray( i, &SL );

    if( te EQ NULL )
        NL_QUIT;

    index = N_AllocInt1dArray( i, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( j, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( j, &SL );

    if( end EQ NULL )
        NL_QUIT;

    Rk = N_AllocPt1dArray( i, &SL );

    if( Rk EQ NULL )
        NL_QUIT;

    R = N_AllocPt1dArray( j, &SL );

    if( R EQ NULL )
        NL_QUIT;

    Rkw = N_AllocCPt1dArray( i, &SL );

    if( Rkw EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( j, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    Aw = N_AllocCPt1dArray( j, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    Qw = N_AllocCPt2dArray( n, s, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    /* Compute coefficient matrix for u-directional approximation */

    if( n GT 1 )
    {
        rj = p;
        sj = p - 1;
        ej = -2;

        for ( i = 0; i <= n - 2; i++ )
        {
            for ( j = 0; j < ub; j++ )
                NTN[i][j] = 0.0;
        }

        for ( i = 0; i <= r - 2; i++ )
            A[i][p] = 0.0;

        for ( i = 0; i <= NL_MIN( p - 1, n - 2 ); i++ )
            start[i] = 0;

        end[0] = -2;

        for ( i = 1; i <= r - 1; i++ )
        {
            error = N_BasisEval( knu, p, u[i], NL_LEFT, N, &j );

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
            end[i] = r - 2;

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
                    NTN[i][j - i + usb] += A[k][i - index[k]] * A[k][j - index[k]];
                }
            }
        }

        N_CreateRealMatrix( &cm, n - 2, n - 2, NTN, NL_MT_BANDED, ub );

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        /* Approximate rows in the u-direction */

        for ( i = 1; i <= r - 1; i++ )
        {
            error = N_BasisIEval( knu, 0, p, u[i], NL_LEFT, &ts[i - 1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisIEval( knu, n, p, u[i], NL_LEFT, &te[i - 1] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( m EQ 1 )
            l = 1;
        else
            l = s;

        for ( j = 0; j <= l; j++ )
        {
            if( m EQ 1 )
            {
                N_PtToCPt( P[0][j], &Pw[0][j] );
                N_PtToCPt( P[r][j], &Pw[n][j] );
            }
            else
            {
                N_PtToCPt( P[0][j], &Qw[0][j] );
                N_PtToCPt( P[r][j], &Qw[n][j] );
            }

            for ( i = 1; i <= r - 1; i++ )
            {
                N_TranslateSum2Pts( P[i][j], -ts[i - 1], P[0][j], -te[i - 1], P[r][j], &Rk[i - 1] );
            }

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

            error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, Aw, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n - 2; i++ )
            {
                if( m EQ 1 )
                    N_CopyCPt( Aw[i], &Pw[i + 1][j] );
                else
                    N_CopyCPt( Aw[i], &Qw[i + 1][j] );
            }
        }
    }

    /* Compute coefficient matrix for v-directional approximation */

    if( m GT 1 )
    {
        rj = q;
        sj = q - 1;
        ej = -2;

        for ( i = 0; i <= m - 2; i++ )
        {
            for ( j = 0; j < vb; j++ )
                NTN[i][j] = 0.0;
        }

        for ( i = 0; i <= s - 2; i++ )
            A[i][q] = 0.0;

        for ( i = 0; i <= NL_MIN( q - 1, m - 2 ); i++ )
            start[i] = 0;

        end[0] = -2;

        for ( i = 1; i <= s - 1; i++ )
        {
            error = N_BasisEval( knv, q, v[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            if( j EQ q )
                l = 1;
            else
                l = 0;

            if( j EQ q OR j EQ m )
                hk = q - 1;
            else
                hk = q;

            for ( k = 0; k <= hk; k++ )
                A[i - 1][k] = N[l + k];

            index[i - 1] = NL_MAX( 0, j - q - 1 );

            if( j GT rj )
            {
                for ( k = 1; k <= j - rj; k++ )
                {
                    sj++;
                    ej++;

                    if( sj LE m - 2 )
                        start[sj] = i - 1;

                    if( ej GE 0 )
                        end[ej] = i - 2;
                }
                rj = j;
            }
        }

        if( sj LT m - 2 OR end[0]EQ - 1 )
            NL_ERROR( NL_INP_ERR );

        for ( i = NL_MAX( 0, ej + 1 ); i <= m - 2; i++ )
            end[i] = s - 2;

        for ( i = 0; i <= m - 2; i++ )
        {
            lj = NL_MAX( 0, i - q );
            hj = NL_MIN( m - 2, i + q );

            for ( j = lj; j <= hj; j++ )
            {
                lk = NL_MAX( start[i], start[j] );
                hk = NL_MIN( end[i], end[j] );

                for ( k = lk; k <= hk; k++ )
                {
                    NTN[i][j - i + vsb] += A[k][i - index[k]] * A[k][j - index[k]];
                }
            }
        }

        N_CreateRealMatrix( &cm, m - 2, m - 2, NTN, NL_MT_BANDED, vb );

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        /* Approximate rows in the v-direction */

        for ( j = 1; j <= s - 1; j++ )
        {
            error = N_BasisIEval( knv, 0, q, v[j], NL_LEFT, &ts[j - 1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisIEval( knv, m, q, v[j], NL_LEFT, &te[j - 1] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( n EQ 1 )
        {
            for ( i = 0; i <= n; i++ )
            {
                N_PtToCPt( P[i][0], &Pw[i][0] );
                N_PtToCPt( P[i][s], &Pw[i][m] );

                for ( j = 1; j <= s - 1; j++ )
                {
                    N_TranslateSum2Pts( P[i][j], -ts[j - 1], P[i][0], -te[j - 1], P[i][s], &Rk[j - 1] );
                }

                for ( j = 1; j <= m - 1; j++ )
                {
                    N_CopyPt( NL_ZERO, &R[j - 1] );

                    lk = start[j - 1];
                    hk = end[j - 1];

                    for ( k = lk; k <= hk; k++ )
                    {
                        N_VectorBlendPt( A[k][j - index[k] - 1], Rk[k], &R[j - 1] );
                    }
                }

                error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, Aw, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = 0; j <= m - 2; j++ )
                {
                    N_CopyCPt( Aw[j], &Pw[i][j + 1] );
                }
            }
        }
        else
        {
            for ( i = 0; i <= n; i++ )
            {
                N_CopyCPt( Qw[i][0], &Pw[i][0] );
                N_CopyCPt( Qw[i][s], &Pw[i][m] );

                for ( j = 1; j <= s - 1; j++ )
                {
                    N_TranslateSum2CPts( Qw[i][j], -ts[j - 1], Qw[i][0], -te[j - 1], Qw[i][s], &Rkw[j - 1] );
                }

                for ( j = 1; j <= m - 1; j++ )
                {
                    N_CopyCPt( NL_CZERO, &Rw[j - 1] );

                    lk = start[j - 1];
                    hk = end[j - 1];

                    for ( k = lk; k <= hk; k++ )
                    {
                        N_VectorBlendCPt( A[k][j - index[k] - 1], Rkw[k], &Rw[j - 1] );
                    }
                }

                error = N_RealMatrixForBack( &cm, (NL_VOID *)Rw, NL_HPOINT, Aw, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = 0; j <= m - 2; j++ )
                {
                    N_CopyCPt( Aw[j], &Pw[i][j + 1] );
                }
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}



/**********************************************************************/
/* N_FITSRFCALCKNOTVECTORS: Compute knot vectors for surface approx to random points */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes the knot vectors for surface  approx-
     imation to random (non-NxM) sets of points. A typical calling exam-
     ple is:

       NL_REAL        *u, *v, us, ue, vs, ve;
       NL_INDEX       nn;
       NL_DEGREE      p, q;
       NL_KNOTVECTOR  knu, knv, knus, knvs;
       ...
       (load arrays u and v, define knu, knv, knus, knvs, and set
        us, ue, vs, ve);
       ...
       N_FitSrfCalcKnotVectors(u,v,nn,p,q,pflg,us,ue,vs,ve,&knus,&knvs,&knu,&knv);

     IT IS ASSUMED THAT MEMORY FOR knu and knv  (knots  as  well as the
     structures themselves) HAVE BEEN ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     u    , input  ,  u parameters
     v    , input  ,  v parameters
     nn   , input  ,  high index of parameters in u and v
     p    , input  ,  Degree of approximating surface in u-direction
     q    , input  ,  Degree of approximating surface in v-direction
     pflg , input  ,  Flag:
                        NL_YES : Where knus and/or knvs are not given, use
                              the values in us,ue,vs,ve to set the  max
                              and min knots
                        NL_NO  : Do not use us,ue,vs,ve. Max and min knots
                              will be set by the max/min values in  the
                              arrays u and v (if knus/knvs not given)
     us,ue, input  ,  Max and min values for the knots  (values in knus
     vs,ve            and knvs override these in any case)
     knus , input  ,  Base knot vector in u-direction.  This  input  is 
                      optional.  If given, these knots will be a subset
                      of the knots in knu (knus = NULL means not given)
     knvs , input  ,  Base knot vector in v-direction.  This  input  is 
                      optional.  If given, these knots will be a subset
                      of the knots in knv (knvs = NULL means not given)
     knu  , output ,  Surface knot vector in the u-direction.  knu must
                      be fully defined in the calling routine  (e.g. by
                      calling N_KnotVectorFromRealArray); this routine  simply  computes
                      and loads the knot values. If knu = knus, then no
                      u-knots are computed in this routine
     knv  , output ,  Surface knot vector in the v-direction.  knv must
                      be fully defined in the calling routine  (e.g. by
                      calling N_KnotVectorFromRealArray); this routine  simply  computes
                      and loads the knot values. If knv = knvs, then no
                      v-knots are computed in this routine


   RETURN CODES:

     None

   ***********************************************************************/

NL_FLAG N_FitSrfCalcKnotVectors 
  (NL_REAL       *u,     /* in : u parameter values, sized:[nn+1] */ 
   NL_REAL       *v,     /* in : v parameter values, sized:[nn+1] */ 
   NL_INDEX       nn,    /* in : highest u and v index */ 
   NL_DEGREE      p,     /* in : approximating surface degree U */ 
   NL_DEGREE      q,     /* in : approximating surface degree V */ 
   NL_FLAG        pflg,  /* in : NL_YES = use us,ue,vs,ve values to set max/min u and v knot values */ 
                         /*      NL_NO  = find max/min u and v knot values in u and v arrays        */
   NL_REAL        us,    /* in : min U param value, overridden by any input knu value   */ 
   NL_REAL        ue,    /* in : max U param value, overridden by any input knu value   */ 
   NL_REAL        vs,    /* in : min V param value, overridden by any input knv value   */ 
   NL_REAL        ve,    /* in : max V param value, overridden by any input knv value   */ 
   NL_KNOTVECTOR *knus,  /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */ 
   NL_KNOTVECTOR *knvs,  /* in : opt starter V knots, if given these knots will be in the output, NULL to ignore */ 
   NL_KNOTVECTOR *knu,   /* i/o: sized knot vector whose knot values are to be determined */
   NL_KNOTVECTOR *knv )  /* i/o: sized knot vector whose knot values are to be determined */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfCalcKnotVectors");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, r, rs = 0, s, ss = 0, *uidx = NULL, *vidx, ku1 = 0, ku2 = 0, kv1 = 0, kv2 = 0, ** nums, iii, j1, j2, j3, j4, k1, k2, k3, k4, mx1, mx2, i1, i2, nu, nv, mins[4], maxs[4], i3, i4, k33;

    NL_REAL *usort = NULL, *vsort = NULL, *U, *US = NULL, *V, *VS = NULL, ** kts, spar, epar, rats[4] ;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if( knu EQ knus AND knv EQ knvs )
        NL_ERROR( NL_INP_ERR );

    if( knu EQ NULL OR knv EQ NULL )
        NL_ERROR( NL_INP_ERR );

    N_KnotVectorGetKnots( knu, &r, &U );

    if( r LT 2 * p + 1 )
        NL_ERROR( NL_INP_ERR );

    if( U EQ NULL )
        NL_ERROR( NL_INP_ERR );
    N_KnotVectorGetKnots( knv, &s, &V );

    if( s LT 2 * q + 1 )
        NL_ERROR( NL_INP_ERR );

    if( V EQ NULL )
        NL_ERROR( NL_INP_ERR );

    /* when given knus, set min and max knot values (us, ue) */
    if( knus NEQ NULL )
    {
        N_KnotVectorGetKnots( knus, &rs, &US );

        if( rs GT r )
            NL_ERROR( NL_INP_ERR );

        if( rs LT 2 * p + 1 )
            NL_ERROR( NL_INP_ERR );
        us = US[0];
        ue = US[rs];

        for ( ii = 1; ii <= p; ii++ )
        {
            if (    ( us NEQ US[ii] ) 
                 OR ( ue NEQ US[rs - ii] ) )
            { NL_ERROR( NL_INP_ERR ); }
        }
    }

    /* when given knus, set min and max knot values (vs, ve) */
    if( knvs NEQ NULL )
    {
        N_KnotVectorGetKnots( knvs, &ss, &VS );

        if( ss GT s )
            NL_ERROR( NL_INP_ERR );

        if( ss LT 2 * q + 1 )
            NL_ERROR( NL_INP_ERR );
        vs = VS[0];
        ve = VS[ss];

        for ( ii = 1; ii <= q; ii++ )
        {
            if ( vs NEQ VS[ii]OR ve NEQ VS[ss - ii] )
            { NL_ERROR( NL_INP_ERR ); }
        }
    }

    /* initialize for different methods */

    for ( ii = 0; ii < 4; ii++ )
    {
        mins[ii] = maxs[ii] = -1; /* a -1 value indicates method not yet used or failed in upcoming ST_AddKnotsStrip() call */
        rats[ii] = 1 ; 
    }

    /* r = 2*p+1 and s = 2*q+1 are special cases (no internal knots) */
    /* find us,ue for no internal knots */ 
    if( r EQ 2 *p + 1 )
    {
        if( knu NEQ knus )
        {
            if( knus EQ NULL AND pflg EQ NL_NO )
            {
                us = ue = u[0];

                for ( ii = 1; ii <= nn; ii++ )
                {
                    if( u[ii]LT us )
                        us = u[ii];

                    else if( u[ii]GT ue )
                        ue = u[ii];
                }
            }

            for ( ii = 0; ii <= p; ii++ )
            {
                U[ii] = us;
                U[p + ii + 1] = ue;
            }
        }
        else
        {
            us = U[0];
            ue = U[r];
        }
    } /* end r = 2*p+1 and s = 2*q+1 are special cases (no internal knots) */
    else /* expect internal knots branch */
    {
        usort = N_AllocReal1dArray( nn, &SL ); /* must sort parameters  for upcoming ST_AddKnotsStrip() call */

        if( usort EQ NULL )
            NL_QUIT;
        uidx = N_AllocInt1dArray( nn, &SL );

        if( uidx EQ NULL )
            NL_QUIT;

        for ( ii = 0; ii <= nn; ii++ )
            usort[ii] = u[ii];
        N_SortRealArray( usort, nn, uidx );

        ku1 = 0;
        ku2 = nn;

        if( knus EQ NULL AND pflg EQ NL_NO )
        {
            us = usort[0];
            ue = usort[nn];
        }
        else
        {
            while( usort[ku1]LT us AND ku1 LT nn )
                ku1 += 1;

            while( usort[ku2]GT ue AND ku2 GT 0 )
                ku2 -= 1;

            if( ku1 GE ku2 )
                NL_ERROR( NL_INP_ERR );
        }
    } /* end internal u knots branch */

    /* when there are no internal v knots */
    if( s EQ 2 *q + 1 )
    {
        if( knv NEQ knvs )
        {
            if( knvs EQ NULL AND pflg EQ NL_NO )
            {
                vs = ve = v[0];

                for ( ii = 1; ii <= nn; ii++ )
                {
                    if( v[ii]LT vs )
                        vs = v[ii];

                    else if( v[ii]GT ve )
                        ve = v[ii];
                }
            }

            for ( ii = 0; ii <= q; ii++ )
            {
                V[ii] = vs;
                V[q + ii + 1] = ve;
            }
        }
        else
        {
            vs = V[0];
            ve = V[s];
        }
    }
    else /* expect internal v knots */
    {
        vsort = N_AllocReal1dArray( nn, &SL ); /* must sort parameters for upcoming ST_AddKnotsStrip() call */

        if( vsort EQ NULL )
            NL_QUIT;
        vidx = N_AllocInt1dArray( nn, &SL );

        if( vidx EQ NULL )
            NL_QUIT;

        for ( ii = 0; ii <= nn; ii++ )
            vsort[ii] = v[ii];
        N_SortRealArray( vsort, nn, vidx );

        kv1 = 0;
        kv2 = nn;

        if( knvs EQ NULL AND pflg EQ NL_NO )
        {
            vs = vsort[0];
            ve = vsort[nn];
        }
        else
        {
            while( vsort[kv1]LT vs AND kv1 LT nn )
                kv1 += 1;

            while( vsort[kv2]GT ve AND kv2 GT 0 )
                kv2 -= 1;

            if( kv1 GE kv2 )
                NL_ERROR( NL_INP_ERR );
        }
    }

    /* low work - no internal knots already done */
    if( r EQ 2 * p + 1 AND s EQ 2 * q + 1 )
        NL_OUT; /* finished */

    /* Special case:  r = 2*p+1  ,  s > 2*q+1 */
    /* no internal u knots and internal v knots */
    if( r EQ 2 *p + 1 AND s GT 2 *q + 1 )
    {
        if( knvs EQ knv )
            NL_OUT; /* finished */

        kts = N_AllocReal2dArray( 5, s, &SL );

        if( kts EQ NULL )
            NL_QUIT;
        nums = N_AllocInt2dArray( 5, s, &SL );

        if( nums EQ NULL )
            NL_QUIT;

        if( knvs EQ NULL )
        {                        /* Get v knots using 4 methods. */
            iii = s - 2 * q - 1; /* Accept best result.          */

            /* generate 4 different canditate knot vectors. Accept best result.     */
            /* in : meth = 1: knots = pars' IndexInterval boundary parameter values */
            /*      meth = 2: knots = average(every pars' IndexInterval values      */
            /*                  to either side of every IndexInterval boundary)     */ 
            /*      meth = 3: knots = avg(pars' values in paramIntervals on either  */
            /*                               side of the paramInterval Boundaries)  */                                                  
            /*      meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
            /*                                   IndexInterval BoundaryMax,         */
            /*                                   all contained points               */ 
            /* default meth : knots = even spacing                                  */                                      

            /* mins[i] = min number of points in any one knot interval */
            /* maxs[i] = max number of points in any one knot interval */
            ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, iii, 1, kts[0], nums[0], &rats[0], &mins[0], &maxs[0], &ii, &ii );
            ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, iii, 2, kts[1], nums[1], &rats[1], &mins[1], &maxs[1], &ii, &ii );
            ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, iii, 3, kts[2], nums[2], &rats[2], &mins[2], &maxs[2], &ii, &ii );
            ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, iii, 4, kts[3], nums[3], &rats[3], &mins[3], &maxs[3], &ii, &ii );

            jj = ST_BestSubdInStrip( 3, nums, iii, mins, maxs, rats );

            for ( ii = 1; ii <= iii; ii++ )
                V[q + ii] = kts[jj][ii - 1];

            for ( ii = 0; ii <= q; ii++ )
            {
                V[ii] = vs;
                V[s - ii] = ve;
            }

            NL_OUT; /* finished */
        }
        else  /* given knvs - use them */
        {
            if( ss EQ s )
            {
                for ( ii = 0; ii <= s; ii++ )
                    V[ii] = VS[ii];
                NL_OUT;
            }

            iii = s - ss; /* we need to add this many knots */

            ii = q;       /* Load each distinct knot into kts[4]. */
            kk = -1;      /* kk is the high index.                */

            while( ii LE ss - q )
            {
                kts[4][++kk] = VS[ii];
                ii += 1;

                while( ii LT ss - q AND VS[ii]EQ VS[ii - 1] )
                    ii += 1;
            }
            /* Get number of parm points */
            for ( jj = kv1, ii = 0; ii < kk; ii++ ) /* in each existing strip.   */
            {
                nums[4][ii] = ST_NumPtsInStrip( vsort, jj, kv2, kts[4][ii], kts[4][ii + 1], &j1, &j2 );
                jj = j1;
            }

            N_SortIndexArrayMap( nums[4], kk - 1, nums[5] ); /* sort them */

            j1 = j3 = nums[4][kk - 1];

            for ( ii = kk - 1; ii >= 0; ii-- )               /* compute how many knots */
            {                                                /* to add in each strip   */
                while( nums[4][ii] < j1 )
                {
                    for ( jj = kk - 1; jj > ii; jj-- )
                    {
                        nums[4][jj] += 1;
                        iii -= 1;

                        if( iii LE 0 )
                            goto done_v;
                    }
                    j1 = j3 / (nums[4][kk - 1] + 1);
                }

                nums[4][ii] = 1;
                iii -= 1;

                if( iii LE 0 )
                    goto done_v;

                if( ii EQ kk - 1 )
                    j1 /= 2;
            }

            for ( ii = 0; ii <= s; ii++ )
                for ( jj = kk - 1; jj >= 0; jj-- )
                {
                    nums[4][jj] += 1;
                    iii -= 1;

                    if( iii LE 0 )
                        goto done_v;
                }

            done_v: /* now divide the strips and add the knots */
            for ( ii = 0; ii <= ss; ii++ )
                V[ii] = VS[ii];

            for ( ii = kk - 1; ii >= 0; ii-- )
            {
                spar = kts[4][nums[5][ii]];
                epar = kts[4][nums[5][ii] + 1];
                j3 = nums[4][ii];

                /* generate 4 different canditate knot vectors. Accept best result.     */
                /* in : meth = 1: knots = pars' IndexInterval boundary parameter values */
                /*      meth = 2: knots = average(every pars' IndexInterval values      */
                /*                  to either side of every IndexInterval boundary)     */ 
                /*      meth = 3: knots = avg(pars' values in paramIntervals on either  */
                /*                               side of the paramInterval Boundaries)  */                                                  
                /*      meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
                /*                                   IndexInterval BoundaryMax,         */
                /*                                   all contained points               */                                      
                /* default meth : knots = even spacing                                  */                                      

                /* mins[i] = min number of points in any one knot interval */
                /* maxs[i] = max number of points in any one knot interval */
                ST_AddKnotsStrip( vsort, kv1, kv2, spar, epar, j3, 1, kts[0], nums[0], &rats[0], &mins[0], &maxs[0], &k1, &k2 );
                ST_AddKnotsStrip( vsort, k1,  k2,  spar, epar, j3, 2, kts[1], nums[1], &rats[1], &mins[1], &maxs[1], &jj, &jj );
                ST_AddKnotsStrip( vsort, k1,  k2,  spar, epar, j3, 3, kts[2], nums[2], &rats[2], &mins[2], &maxs[2], &jj, &jj );
                ST_AddKnotsStrip( vsort, k1,  k2,  spar, epar, j3, 4, kts[3], nums[3], &rats[3], &mins[3], &maxs[3], &jj, &jj );

                jj = ss;

                while( V[jj] > spar )
                {
                    V[jj + j3] = V[jj];
                    jj -= 1;
                }

                k33 = ST_BestSubdInStrip( 3, nums, j3, mins, maxs, rats );

                for ( j1 = 1; j1 <= j3; j1++ )
                    V[jj + j1] = kts[k33][j1 - 1];

                ss += j3;

                if( ss GE s )
                    NL_OUT; /* finished */
            }
        }
    } /* this ends case:  r = 2*p+1  ,  s > 2*q+1 */

    /* Special case:  r > 2*p+1  ,  s = 2*q+1 */

    if( r GT 2 *p + 1 AND s EQ 2 *q + 1 )
    {
        if( knus EQ knu )
            NL_OUT; /* finished */

        kts = N_AllocReal2dArray( 5, r, &SL );

        if( kts EQ NULL )
            NL_QUIT;
        nums = N_AllocInt2dArray( 5, r, &SL );

        if( nums EQ NULL )
            NL_QUIT;

        if( knus EQ NULL )
        {                        /* Get knots using 4 methods. */
            iii = r - 2 * p - 1; /* Accept best result.        */


            /* generate 4 different canditate knot vectors. Accept best result.     */
            /* in : meth = 1: knots = pars' IndexInterval boundary parameter values */
            /*      meth = 2: knots = average(every pars' IndexInterval values      */
            /*                  to either side of every IndexInterval boundary)     */ 
            /*      meth = 3: knots = avg(pars' values in paramIntervals on either  */
            /*                               side of the paramInterval Boundaries)  */                                                  
            /*      meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
            /*                                   IndexInterval BoundaryMax,         */
            /*                                   all contained points               */                                      
            /* default meth : knots = even spacing                                  */                                      

            /* mins[i] = min number of points in any one knot interval */
            /* maxs[i] = max number of points in any one knot interval */
            ST_AddKnotsStrip( usort, ku1, ku2, us, ue, iii, 1, kts[0], nums[0], &rats[0], &mins[0], &maxs[0], &ii, &ii );
            ST_AddKnotsStrip( usort, ku1, ku2, us, ue, iii, 2, kts[1], nums[1], &rats[1], &mins[1], &maxs[1], &ii, &ii );
            ST_AddKnotsStrip( usort, ku1, ku2, us, ue, iii, 3, kts[2], nums[2], &rats[2], &mins[2], &maxs[2], &ii, &ii );
            ST_AddKnotsStrip( usort, ku1, ku2, us, ue, iii, 4, kts[3], nums[3], &rats[3], &mins[3], &maxs[3], &ii, &ii );

            jj = ST_BestSubdInStrip( 3, nums, iii, mins, maxs, rats );

            for ( ii = 1; ii <= iii; ii++ )
                U[p + ii] = kts[jj][ii - 1];

            for ( ii = 0; ii <= p; ii++ )
            {
                U[ii] = us;
                U[r - ii] = ue;
            }

            NL_OUT; /* finished */
        }
        else /* given knvs - use them */
        {
            if( rs EQ r )
            {
                for ( ii = 0; ii <= r; ii++ )
                    U[ii] = US[ii];
                NL_OUT;
            }

            iii = r - rs; /* we need to add this many knots */

            ii = p;       /* Load each distinct knot into kts[4]. */
            kk = -1;      /* kk is the high index.                */

            while( ii LE rs - p )
            {
                kts[4][++kk] = US[ii];
                ii += 1;

                while( ii LT rs - p AND US[ii]EQ US[ii - 1] )
                    ii += 1;
            }
            /* Get number of parm points */
            for ( jj = ku1, ii = 0; ii < kk; ii++ ) /* in each existing strip.   */
            {
                nums[4][ii] = ST_NumPtsInStrip( usort, jj, ku2, kts[4][ii], kts[4][ii + 1], &j1, &j2 );
                jj = j1;
            }

            N_SortIndexArrayMap( nums[4], kk - 1, nums[5] ); /* sort them */

            j1 = j3 = nums[4][kk - 1];

            for ( ii = kk - 1; ii >= 0; ii-- )               /* compute how many knots */
            {                                                /* to add in each strip   */
                while( nums[4][ii] < j1 )
                {
                    for ( jj = kk - 1; jj > ii; jj-- )
                    {
                        nums[4][jj] += 1;
                        iii -= 1;

                        if( iii LE 0 )
                            goto done_u;
                    }
                    j1 = j3 / (nums[4][kk - 1] + 1);
                }

                nums[4][ii] = 1;
                iii -= 1;

                if( iii LE 0 )
                    goto done_u;

                if( ii EQ kk - 1 )
                    j1 /= 2;
            }

            for ( ii = 0; ii <= r; ii++ )
                for ( jj = kk - 1; jj >= 0; jj-- )
                {
                    nums[4][jj] += 1;
                    iii -= 1;

                    if( iii LE 0 )
                        goto done_u;
                }

            done_u: /* now divide the strips and add the knots */
            for ( ii = 0; ii <= rs; ii++ )
                U[ii] = US[ii];

            for ( ii = kk - 1; ii >= 0; ii-- )
            {
                spar = kts[4][nums[5][ii]];
                epar = kts[4][nums[5][ii] + 1];
                j3 = nums[4][ii];


                /* generate 4 different canditate knot vectors. Accept best result.     */
                /* in : meth = 1: knots = pars' IndexInterval boundary parameter values */
                /*      meth = 2: knots = average(every pars' IndexInterval values      */
                /*                  to either side of every IndexInterval boundary)     */ 
                /*      meth = 3: knots = avg(pars' values in paramIntervals on either  */
                /*                               side of the paramInterval Boundaries)  */                                                  
                /*      meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
                /*                                   IndexInterval BoundaryMax,         */
                /*                                   all contained points               */                                      
                /* default meth : knots = even spacing                                  */                                      

                /* mins[i] = min number of points in any one knot interval */
                /* maxs[i] = max number of points in any one knot interval */
                ST_AddKnotsStrip( usort, ku1, ku2, spar, epar, j3, 1, kts[0], nums[0], &rats[0], &mins[0], &maxs[0], &k1, &k2 );
                ST_AddKnotsStrip( usort, k1,  k2,  spar, epar, j3, 2, kts[1], nums[1], &rats[1], &mins[1], &maxs[1], &jj, &jj );
                ST_AddKnotsStrip( usort, k1,  k2,  spar, epar, j3, 3, kts[2], nums[2], &rats[2], &mins[2], &maxs[2], &jj, &jj );
                ST_AddKnotsStrip( usort, k1,  k2,  spar, epar, j3, 4, kts[3], nums[3], &rats[3], &mins[3], &maxs[3], &jj, &jj );

                jj = rs;

                while( U[jj] > spar )
                {
                    U[jj + j3] = U[jj];
                    jj -= 1;
                }

                k33 = ST_BestSubdInStrip( 3, nums, j3, mins, maxs, rats );

                for ( j1 = 1; j1 <= j3; j1++ )
                    U[jj + j1] = kts[k33][j1 - 1];

                rs += j3;

                if( rs GE r )
                    NL_OUT; /* finished */
            }
        }
    } /* this ends case:  r > 2*p+1  ,  s = 2*q+1 */

    /* Now general case:  r > 2*p+1  and  s > 2*q+1 */

    kk = NL_MAX( r, s );
    kts = N_AllocReal2dArray( 13, kk, &SL );

    if( kts EQ NULL )
        NL_QUIT;

    /* initialize array */
    for (ii = 0; ii <= 13; ii++)
      {
        for(jj = 0; jj <= kk; jj++)
          { kts[ii][jj] = 0.0; }
      }

    nums = N_AllocInt2dArray( 2, kk, &SL );

    if( nums EQ NULL )
        NL_QUIT;

    /* Get candidate u-knots first */

    if( knus EQ NULL )
    {
        mx1 = 3;      /* kts[0] ,..., kts[3] are candidate u-knot vectors */

        iii = r - 2 * p - 1;   /* iii = number of internal knots = Total KnotCnt - end repeated knots */
        nu = iii + 1;          /* nu  = high index of internal knots + 2. Maybe internal knots + one EndKnot at each end? */


        /* generate 4 different canditate knot vectors. Accept best result.     */
        /* in : meth = 1: knots = pars' IndexInterval boundary parameter values */
        /*      meth = 2: knots = average(every pars' IndexInterval values      */
        /*                  to either side of every IndexInterval boundary)     */ 
        /*      meth = 3: knots = avg(pars' values in paramIntervals on either  */
        /*                               side of the paramInterval Boundaries)  */                                                  
        /*      meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
        /*                                   IndexInterval BoundaryMax,         */
        /*                                   all contained points               */                                      
        /* default meth : knots = even spacing                                  */                                      

        /* in : usort = sorted list of parameter values                                 */
        /* in : ku1   = min index in pars to examine                                    */      
        /* in : ku2   = max index in pars to examine                                    */      
        /* in : us    = min index value of param interval (the strip) being examined    */
        /* in : ue    = max index value of param interval (the strip) being examined    */
        /* in : iii   = max index of UniqueKnot vals (KnotCnt - duplicate EndKnots - 1) */     
        /* in : in : meth = 1: knots = even number of par vals for each knot span    */
        /*           meth = 2: knots = average(every pars' IndexInterval values      */
        /*                       to either side of every IndexInterval boundary)     */ 
        /*           meth = 3: knots = avg(pars' values in paramIntervals on either  */
        /*                                    side of the paramInterval Boundaries)  */       
        /*           meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
        /*                                        IndexInterval BoundaryMax,         */
        /*                                        all contained points               */ 
        /*           DefMeth : knots = evenly spaced                                 */
        /*      out: -1 = requested knot seq had zero length spans - rtns even spaced knots */
        /* in : p = degree of BSpline Curve receiving thes knots                    */     
        /* out: kts[0] = knot values, sized:[nd+1]                                       */
        /* out: nums[0] = number of pars values in each returned knot interval            */
        /* out: mins[0] = min number of pars values between any pair of output knots      */
        /* out: maxs[0] = max number of pars values between any pair of output knots      */
        /* out: &ii = pars index of the first value found bounding pm1                */
        /* out: &ii = pars index of the first value found bounding pm2                */
        /* kts[13] = scratch:                                                             */
        ST_AddKnotsStrip( usort, ku1, ku2, us, ue, nu, 1, kts[0], nums[0], &rats[0], &mins[0], &maxs[0], &ii, &ii );
        ST_AddKnotsStrip( usort, ku1, ku2, us, ue, nu, 2, kts[1], nums[0], &rats[1], &mins[1], &maxs[1], &ii, &ii );
        ST_AddKnotsStrip( usort, ku1, ku2, us, ue, nu, 3, kts[2], nums[0], &rats[2], &mins[2], &maxs[2], &ii, &ii );
        ST_AddKnotsStrip( usort, ku1, ku2, us, ue, nu, 4, kts[3], nums[0], &rats[3], &mins[3], &maxs[3], &ii, &ii );

        kts[0][0]  = kts[1][0]  = kts[2][0]  = kts[3][0]  = us;
        kts[0][nu] = kts[1][nu] = kts[2][nu] = kts[3][nu] = ue;

    } /* end no given knus branch */
    else /* given a knus branch */
    {
        if( knus EQ knu OR r EQ rs )
        {
            mx1 = 0;
            iii = 0;
            mins[0] = 1;
        }
        else
        {
            mx1 = 3;
            iii = 8;
        }

        ii = p;  /* Load each distinct knot into kts[iii]. */
        nu = -1; /* nu is the high index.                  */

        while( ii LE rs - p )
        {
            kts[iii][++nu] = US[ii];
            ii += 1;

            while( ii LT rs - p AND US[ii]EQ US[ii - 1] )
                ii += 1;
        }

        iii = r - rs;                               /* we need to add this many knots if mx1 = 3 */

        if( mx1 EQ 3 )
        {                                           /* Get number of parm points */
            for ( jj = ku1, ii = 0; ii < nu; ii++ ) /* in each existing strip.   */
            {
                nums[0][ii] = ST_NumPtsInStrip( usort, jj, ku2, kts[8][ii], kts[8][ii + 1], &j1, &j2 );
                jj = j1;
            }

            N_SortIndexArrayMap( nums[0], nu - 1, nums[1] ); /* sort them */

            j1 = j3 = nums[0][nu - 1];

            for ( ii = nu - 1; ii >= 0; ii-- )               /* compute how many knots */
            {                                                /* to add in each strip   */
                while( nums[0][ii] < j1 )
                {
                    for ( jj = nu - 1; jj > ii; jj-- )
                    {
                        nums[0][jj] += 1;
                        iii -= 1;

                        if( iii LE 0 )
                            goto done_uu;
                    }
                    j1 = j3 / (nums[0][nu - 1] + 1);
                }

                nums[0][ii] = 1;
                iii -= 1;

                if( iii LE 0 )
                    goto done_uu;

                if( ii EQ nu - 1 )
                    j1 /= 2;
            }

            for ( ii = 0; ii <= r; ii++ )
                for ( jj = nu - 1; jj >= 0; jj-- )
                {
                    nums[0][jj] += 1;
                    iii -= 1;

                    if( iii LE 0 )
                        goto done_uu;
                }

            done_uu: /* now divide the strips and add the knots */
            for ( ii = 0; ii <= nu; ii++ )
            {
                kts[0][ii] = kts[1][ii] = kts[8][ii];
                kts[2][ii] = kts[3][ii] = kts[8][ii];
            }

            iii = r - rs;

            for ( ii = nu - 1; ii >= 0; ii-- )
            {
                spar = kts[8][nums[1][ii]];
                epar = kts[8][nums[1][ii] + 1];
                j3 = nums[0][ii];

                /* generate 4 different canditate knot vectors. Accept best result.     */
                /* in : meth = 1: knots = pars' IndexInterval boundary parameter values */
                /*      meth = 2: knots = average(every pars' IndexInterval values      */
                /*                  to either side of every IndexInterval boundary)     */ 
                /*      meth = 3: knots = avg(pars' values in paramIntervals on either  */
                /*                               side of the paramInterval Boundaries)  */                                                  
                /*      meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
                /*                                   IndexInterval BoundaryMax,         */
                /*                                   all contained points               */                                      
                /* default meth : knots = even spacing                                  */                                      

                /* in : usort = sorted list of parameter values                                 */
                /* in : ku1, k1   = min index in pars to examine                                    */      
                /* in : ku2, k2   = max index in pars to examine                                    */      
                /* in : spar    = min index value of param interval (the strip) being examined    */
                /* in : epar    = max index value of param interval (the strip) being examined    */
                /* in : j3   = max index of UniqueKnot vals (KnotCnt - duplicate EndKnots - 1) */     
                /* in : in : meth = 1: knots = even number of par vals for each knot span    */
                /*           meth = 2: knots = average(every pars' IndexInterval values      */
                /*                       to either side of every IndexInterval boundary)     */ 
                /*           meth = 3: knots = avg(pars' values in paramIntervals on either  */
                /*                                    side of the paramInterval Boundaries)  */       
                /*           meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
                /*                                        IndexInterval BoundaryMax,         */
                /*                                        all contained points               */ 
                /*           DefMeth : knots = evenly spaced                                 */
                /*      out: -1 = requested knot seq had zero length spans - rtns even spaced knots */
                /* in : p = degree of BSpline Curve receiving thes knots                    */     
                /* out: &kts[9] = knot values, sized:[nd+1]                                       */
                /* out: nums[2] = number of pars values in each returned knot interval            */
                /* out: mins[0] = min number of pars values between any pair of output knots      */
                /* out: maxs[0] = max number of pars values between any pair of output knots      */
                /* out: k1, &jj = pars index of the first value found bounding pm1                */
                /* out: k2, &jj = pars index of the first value found bounding pm2                */
                /* kts[13] = scratch:                                                             */
                ST_AddKnotsStrip( usort, ku1, ku2, spar, epar, j3, 1, kts[9],  nums[2], &rats[0], &mins[0], &maxs[0], &k1, &k2 );
                ST_AddKnotsStrip( usort, k1,  k2,  spar, epar, j3, 2, kts[10], nums[2], &rats[1], &mins[1], &maxs[1], &jj, &jj );
                ST_AddKnotsStrip( usort, k1,  k2,  spar, epar, j3, 3, kts[11], nums[2], &rats[2], &mins[2], &maxs[2], &jj, &jj );
                ST_AddKnotsStrip( usort, k1,  k2,  spar, epar, j3, 4, kts[12], nums[2], &rats[3], &mins[3], &maxs[3], &jj, &jj );

                jj = nu;

                while( kts[0][jj] > spar )
                {
                    kts[0][jj + j3] = kts[0][jj];
                    kts[1][jj + j3] = kts[1][jj];
                    kts[2][jj + j3] = kts[2][jj];
                    kts[3][jj + j3] = kts[3][jj];
                    jj -= 1;
                }

                for ( j1 = 1; j1 <= j3; j1++ )
                {
                    kts[0][jj + j1] = kts[9][j1 - 1];
                    kts[1][jj + j1] = kts[10][j1 - 1];
                    kts[2][jj + j1] = kts[11][j1 - 1];
                    kts[3][jj + j1] = kts[12][j1 - 1];
                }

                nu += j3;
                iii -= j3;

                if( iii LE 0 )
                    break; /* finished with u-knots */
            }
        }
    } /* end given a knus branch */

    /* Now get candidate v-knots  */

    if( knvs EQ NULL )
    {
        mx2 = 3;      /* kts[4] ,..., kts[7] are candidate v-knot vectors */

        iii = s - 2 * q - 1;
        nv = iii + 1; /* high index of v-knots in kts[] */

        /* generate 4 different canditate knot vectors */

        /* in : usort = sorted list of parameter values                                 */
        /* in : kv1   = min index in pars to examine                                    */      
        /* in : kv2   = max index in pars to examine                                    */      
        /* in : vs    = min index value of param interval (the strip) being examined    */
        /* in : ve    = max index value of param interval (the strip) being examined    */
        /* in : iii   = max index of UniqueKnot vals (KnotCnt - duplicate EndKnots - 1) */     
        /* in : in : meth = 1: knots = even number of par vals for each knot span    */
        /*           meth = 2: knots = average(every pars' IndexInterval values      */
        /*                       to either side of every IndexInterval boundary)     */ 
        /*           meth = 3: knots = avg(pars' values in paramIntervals on either  */
        /*                                    side of the paramInterval Boundaries)  */       
        /*           meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
        /*                                        IndexInterval BoundaryMax,         */
        /*                                        all contained points               */ 
        /*           DefMeth : knots = evenly spaced                                 */
        /*      out: -1 = requested knot seq had zero length spans - rtns even spaced knots */
        /* in : q = degree of BSpline Curve receiving thes knots                    */     
        /* out: &kts[4][1] = knot values, sized:[nd+1]                                       */
        /* out: nums[0] = number of pars values in each returned knot interval            */
        /* out: mins[0] = min number of pars values between any pair of output knots      */
        /* out: maxs[0] = max number of pars values between any pair of output knots      */
        /* out: &ii = pars index of the first value found bounding pm1                */
        /* out: &ii = pars index of the first value found bounding pm2                */
        /* kts[13] = scratch:                                                             */
        ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, nv, 1, kts[4], nums[0], &rats[0], &mins[0], &maxs[0], &ii, &ii );
        ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, nv, 2, kts[5], nums[0], &rats[1], &mins[1], &maxs[1], &ii, &ii );
        ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, nv, 3, kts[6], nums[0], &rats[2], &mins[2], &maxs[2], &ii, &ii );
        ST_AddKnotsStrip( vsort, kv1, kv2, vs, ve, nv, 4, kts[7], nums[0], &rats[3], &mins[3], &maxs[3], &ii, &ii );

        kts[4][0]  = kts[5][0]  = kts[6][0]  = kts[7][0]  = vs;
        kts[4][nv] = kts[5][nv] = kts[6][nv] = kts[7][nv] = ve;
    } /* given no knvs branch */
    else /* given a knvs */
    {
        if( knvs EQ knv OR s EQ ss )
        {
            mx2 = 0;
            iii = 4;
            maxs[0] = 1;
        }
        else
        {
            mx2 = 3;
            iii = 8;
        }

        ii = q;  /* Load each distinct knot into kts[iii]. */
        nv = -1; /* nv is the high index.                  */

        while( ii LE ss - q )
        {
            kts[iii][++nv] = VS[ii];
            ii += 1;

            while( ii LT ss - q AND VS[ii]EQ VS[ii - 1] )
                ii += 1;
        }

        iii = s - ss;                               /* we need to add this many knots if mx2 = 3 */

        if( mx2 EQ 3 )
        {                                           /* Get number of parm points */
            for ( jj = kv1, ii = 0; ii < nv; ii++ ) /* in each existing strip.   */
            {
                nums[0][ii] = ST_NumPtsInStrip( vsort, jj, kv2, kts[8][ii], kts[8][ii + 1], &j1, &j2 );
                jj = j1;
            }

            N_SortIndexArrayMap( nums[0], nv - 1, nums[1] ); /* sort them */

            j1 = j3 = nums[0][nv - 1];

            for ( ii = nv - 1; ii >= 0; ii-- )               /* compute how many knots */
            {                                                /* to add in each strip   */
                while( nums[0][ii] < j1 )
                {
                    for ( jj = nv - 1; jj > ii; jj-- )
                    {
                        nums[0][jj] += 1;
                        iii -= 1;

                        if( iii LE 0 )
                            goto done_vv;
                    }
                    j1 = j3 / (nums[0][nv - 1] + 1);
                }

                nums[0][ii] = 1;
                iii -= 1;

                if( iii LE 0 )
                    goto done_vv;

                if( ii EQ nv - 1 )
                    j1 /= 2;
            }

            for ( ii = 0; ii <= s; ii++ )
                for ( jj = nv - 1; jj >= 0; jj-- )
                {
                    nums[0][jj] += 1;
                    iii -= 1;

                    if( iii LE 0 )
                        goto done_vv;
                }

            done_vv: /* now divide the strips and add the knots */
            for ( ii = 0; ii <= nv; ii++ )
            {
                kts[4][ii] = kts[5][ii] = kts[8][ii];
                kts[6][ii] = kts[7][ii] = kts[8][ii];
            }

            iii = s - ss;

            for ( ii = nv - 1; ii >= 0; ii-- )
            {
                spar = kts[8][nums[1][ii]];
                epar = kts[8][nums[1][ii] + 1];
                j3 = nums[0][ii];
                
                /* in : usort = sorted list of parameter values                                 */
                /* in : kv1, k1   = min index in pars to examine                                    */      
                /* in : kv2, k2   = max index in pars to examine                                    */      
                /* in : spar    = min index value of param interval (the strip) being examined    */
                /* in : epar    = max index value of param interval (the strip) being examined    */
                /* in : j3   = max index of UniqueKnot vals (KnotCnt - duplicate EndKnots - 1) */     
                /* in : in : meth = 1: knots = even number of par vals for each knot span    */
                /*           meth = 2: knots = average(every pars' IndexInterval values      */
                /*                       to either side of every IndexInterval boundary)     */ 
                /*           meth = 3: knots = avg(pars' values in paramIntervals on either  */
                /*                                    side of the paramInterval Boundaries)  */       
                /*           meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
                /*                                        IndexInterval BoundaryMax,         */
                /*                                        all contained points               */ 
                /*           DefMeth : knots = evenly spaced                                 */
                /*      out: -1 = requested knot seq had zero length spans - rtns even spaced knots */
                /* in : q = degree of BSpline Curve receiving thes knots                    */     
                /* out: &kts[9] = knot values, sized:[nd+1]                                       */
                /* out: nums[2] = number of pars values in each returned knot interval            */
                /* out: mins[0] = min number of pars values between any pair of output knots      */
                /* out: maxs[0] = max number of pars values between any pair of output knots      */
                /* out: k1, &jj = pars index of the first value found bounding pm1                */
                /* out: k2, &jj = pars index of the first value found bounding pm2                */
            
                ST_AddKnotsStrip( vsort, kv1, kv2, spar, epar, j3, 1, kts[9],  nums[2], &rats[0], &mins[0], &maxs[0], &k1, &k2 );
                ST_AddKnotsStrip( vsort, k1,  k2,  spar, epar, j3, 2, kts[10], nums[2], &rats[1], &mins[1], &maxs[1], &jj, &jj );
                ST_AddKnotsStrip( vsort, k1,  k2,  spar, epar, j3, 3, kts[11], nums[2], &rats[2], &mins[2], &maxs[2], &jj, &jj );
                ST_AddKnotsStrip( vsort, k1,  k2,  spar, epar, j3, 4, kts[12], nums[2], &rats[3], &mins[3], &maxs[3], &jj, &jj );

                jj = nv;

                while( kts[4][jj] > spar )
                {
                    kts[4][jj + j3] = kts[4][jj];
                    kts[5][jj + j3] = kts[5][jj];
                    kts[6][jj + j3] = kts[6][jj];
                    kts[7][jj + j3] = kts[7][jj];
                    jj -= 1;
                }

                for ( j1 = 1; j1 <= j3; j1++ )
                {
                    kts[4][jj + j1] = kts[9][j1 - 1];
                    kts[5][jj + j1] = kts[10][j1 - 1];
                    kts[6][jj + j1] = kts[11][j1 - 1];
                    kts[7][jj + j1] = kts[12][j1 - 1];
                }

                nv += j3;
                iii -= j3;

                if( iii LE 0 )
                    break; /* finished with v-knots */
            }
        }
    } /* end given a knvs branch */

    /* Now decide which of the (mx1+1)*(mx2+1) combinations is best */

    k1 = -1; /* k1 is best u-knot vector */
    k2 = -1; /* k2 is best v-knot vector */

    j1 = -1; /* j1 is best min number of parm points  */
    j2 = nn; /* j2 is best max number of parm points  */
    i3 = -1; /* i3 is the number of boxes with j1 pts */

    /* for every u candidate knot vector */
    for ( ii = 0; ii <= mx1; ii++ )
    {
        /* skip bad u candidate knot vectors */
        if( mins[ii] LT 0 )
            continue;

        /* for every v candidate knot vector */
        for ( jj = 0; jj <= mx2; jj++ )
        {
            /* skip bad v candidate knot vectors */
            if( maxs[jj]LT 0 )
                continue;

            /* compute min/max number of param points in any one element for this grid */

            j3 = nn; /* min param points in any one surface element for this grid      */
            j4 = -1; /* max param points in any one surface element for this grid      */
            i4 = -1; /* number of boxes with j3 pts */

            k33 = ku1;

            /* for every u span */
            for ( i1 = 0; i1 < nu; i1++ )
            {
                /* find the index of the contained param values in the current u span */
                k3 = ST_BinarySearch( usort, k33, ku2, kts[ii][i1], NL_LEFT );
                k4 = ST_BinarySearch( usort, k33, ku2, kts[ii][i1 + 1], NL_RIGHT );

                /* limit the bounding index values */
                if( k3 GT ku2 )
                    k3 = ku2;

                if( k4 LT k33 )
                    k4 = k33;
                k33 = k3;

                /* for every v span */
                for ( i2 = 0; i2 < nv; i2++ )
                {   
                    /* numer of param points in USpan x VSpan element */
                    kk = ST_NumPtsInBox( usort, uidx, k3, k4, v, kts[ii][i1], kts[ii][i1 + 1], kts[jj + 4][i2], kts[jj + 4][i2 + 1] );
                    
                    /* save the min elem param point count */
                    if( kk LE j3 )
                    {
                        if( kk LT j3 )
                        {
                            j3 = kk;
                            i4 = 1;
                        }
                        else
                            i4 += 1;
                    }

                    /* save the max elem param point count */
                    if( kk GT j4 )
                        j4 = kk;
                } /* end iter every vspan */
            } /* end iter every uspan */

            /* remember the best candidate u x v knot pairing */
            if(  ( j3 GT j1 )                                /*    min element point count less than current best     */
               OR( j3 EQ j1 AND i4 LT i3 )                   /* OR tied min element point count AND fewer min count elements */
               OR( j3 EQ j1 AND i4 EQ i3 AND j4 LT j2 ) )    /* OR     tied min element point count                   */
                                                             /*    AND tied min count elements                        */
                                                             /*    AND max element point count less than current best */
            {
                k1 = ii; /* save current best u span index */
                k2 = jj; /* save current best v span index */
                j1 = j3; /*   this combination's element param point count min */
                j2 = j4; /*   this combination's element param point count max */
                i3 = i4; /*   the number of elements in this combination that contain the min number of points */ 
            }
        } /* end iter every v candidate knot vector */
    } /* end iter every u candidate knot vector */

    /* Load best knot vectors */

    if( knus NEQ knu )
    {
        for ( ii = 0; ii <= p; ii++ )
        {
            U[ii] = us;
            U[r - ii] = ue;
        }

        if( knus EQ NULL )
        {
            for ( jj = 1; jj < nu; jj++ )
                U[jj + p] = kts[k1][jj];
        }
        else
        {
            ii = p + 1;
            jj = 1;
            kk = p + 1;

            while( 1 )
            {
                if( US[ii]NEQ kts[k1][jj] )
                    U[kk++] = kts[k1][jj];
                else
                {
                    do
                    {
                        U[kk++] = US[ii++];
                    } while ( US[ii - 1]EQ US[ii] );
                }

                jj += 1;

                if( jj GE nu )
                    break;
            }
        }
    }

    if( knvs NEQ knv )
    {
        k2 += 4;

        for ( ii = 0; ii <= q; ii++ )
        {
            V[ii] = vs;
            V[s - ii] = ve;
        }

        if( knvs EQ NULL )
        {
            for ( jj = 1; jj < nv; jj++ )
                V[jj + q] = kts[k2][jj];
        }
        else
        {
            ii = q + 1;
            jj = 1;
            kk = q + 1;

            while( 1 )
            {
                if( VS[ii]NEQ kts[k2][jj] )
                    V[kk++] = kts[k2][jj];
                else
                {
                    do
                    {
                        V[kk++] = VS[ii++];
                    } while ( VS[ii - 1]EQ VS[ii] );
                }

                jj += 1;

                if( jj GE nv )
                    break;
            }
        }
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_FitSrfCalcKnotVectors */

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITCALCKNOTSRANDOM: Knot vectors for random data approximation               */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     Given random parameter  pairs in the  surface's  parameter  domain, 
     this routine  computes  knot vectors for approximating  random data
     points  with the  parameter  pairs as assigned  parameter values. A
     typical calling example is as follows:

       NL_REAL        *u, *v, u0, u1, v0, v1, *UC, *VC, per;
       NL_INDEX       k, n, m, rc, sc;
       NL_DEGREE      p, q;
       NL_KNOTVECTOR  *knu, *knv;
       ...
       (get u, v, ...)
       ...
       N_FitCalcKnotsRandom(u,v,k,p,q,n,m,u0,u1,v0,v1,NL_YES,UC,rc,VC,sc,per,knu,knv);

     MEMORY FOR THE knu AND knv OBJECTS MUST BE ALLOCATED IN THE CALLING
     ROUTINE!


   ACCESS:
   
     u,v   , input  ,  Parameter pairs
     k     , input  ,  Highest index in (u[0..k],v[0..k])
     p,q   , input  ,  Degrees of the approximation
     n,m   , input  ,  Highest indexes of control points for the approx-
                       imation
     u0,u1 , input  ,  U-parameter bounds
     v0,v1 , input  ,  V-parameter bounds
     flg   , input  ,  Flag:
                         NL_YES: use parameter bounds to set min/max  knots
                         NL_NO : compute min/max knots based on (u,v) pairs
     UC,VC , input  ,  Candidate knots:
                         !NULL: select  knots  from  this knot  array if 
                                possible
                          NULL: compute knots internally
     rc,sc , input  ,  Highest indexes in UC and VC
     per   , input  ,  Percentage of knot interval usage, i.e., an ideal
                       knot is  computed that  centers an inteval around 
                       it.  Candidate  knots  are  selected  from  these 
                       intervals.
                       1.0: the entire interval is  used to  find  knots 
                        |   in UC, VC.  This  gives  maximum use of  the 
                        |   knots, however, the approximant  may not  be 
                        |   as pleasing as desired.
                       \|/
                       0.0: This  gives zero  flexibility  and  produces 
                            knot vectors based on parameter averaging.
                       For most data sets per = 0.75 is quite adequate.
     knu   , output ,  U-knot vector
     knv   , output ,  V-knot vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcKnotsRandom( NL_REAL *u, NL_REAL *v, NL_INDEX k, NL_DEGREE p, NL_DEGREE q, NL_INDEX n, NL_INDEX m, NL_REAL u0, NL_REAL u1, NL_REAL v0, NL_REAL v1, NL_FLAG flg, NL_REAL *UC, NL_REAL *VC, NL_INDEX rc, NL_INDEX sc, NL_REAL per, NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcKnotsRandom");

    NL_FLAG error = NL_NO;

    NL_INDEX *in, i, j, l, le, ri, r, s, iu, iv, im = 0, nn, mm, rn;

    NL_REAL *us, *vs, *ur, *vr, *ul, *vl, *U, *V, *UI, *VI, *UP, *VP, sum, ua, va, f, a, b, d, ff, inv, sg, ui, vi;

    NL_KNOTVECTOR *kni, *knp;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL rf = 0.01;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input  */

    if( (k + 1)LT( n + 1 ) * (m + 1) )
        NL_ERROR( NL_INP_ERR );

    /* Get min/max knots */

    us = N_AllocReal1dArray( k, &SL );

    if( us EQ NULL )
        NL_QUIT;

    vs = N_AllocReal1dArray( k, &SL );

    if( vs EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        us[i] = u[i];
        vs[i] = v[i];
    }

    if( flg EQ NL_NO )
    {
        u0 = u1 = us[0];
        v0 = v1 = vs[0];

        for ( i = 1; i <= k; i++ )
        {
            if( us[i]LT u0 )
                u0 = us[i];

            if( us[i]GT u1 )
                u1 = us[i];

            if( vs[i]LT v0 )
                v0 = vs[i];

            if( vs[i]GT v1 )
                v1 = vs[i];
        }
    }

    /* Jitter parameters to avoid symmetric, coincident values */

    f = ((NL_REAL)n) / ((NL_REAL)m);
    mm = (NL_INDEX)sqrt( k / f );
    nn = (NL_INDEX)(f * mm);
    inv = 1.0 / ((NL_REAL)RAND_MAX);
    ui = (u1 - u0) / nn;
    vi = (v1 - v0) / mm;

    for ( i = 0; i <= k; i++ )
    {
        rn = N_GenerateRandomNumber();
        ff = N_RandomFlipsFlops();
        f = rf * ((NL_REAL)rn) * inv;

        if( ff EQ NL_FLIP )
            sg = 1.0;
        else
            sg = -1.0;
        us[i] += (sg * f * ui);

        if( us[i]LT u0 )
            us[i] = u0;

        if( us[i]GT u1 )
            us[i] = u1;

        rn = N_GenerateRandomNumber();
        ff = N_RandomFlipsFlops();
        f = rf * ((NL_REAL)rn) * inv;

        if( ff EQ NL_FLIP )
            sg = 1.0;
        else
            sg = -1.0;
        vs[i] += (sg * f * vi);

        if( vs[i]LT v0 )
            vs[i] = v0;

        if( vs[i]GT v1 )
            vs[i] = v1;
    }

    /* Sort parameters */

    N_ShellSortReal( us, k );
    N_ShellSortReal( vs, k );

    /* Get memory for various parameter positions */

    ur = N_AllocReal1dArray( nn, &SL );
    ul = N_AllocReal1dArray( nn, &SL );

    if( ur EQ NULL )
        NL_QUIT;

    if( ul EQ NULL )
        NL_QUIT;

    vr = N_AllocReal1dArray( mm, &SL );
    vl = N_AllocReal1dArray( mm, &SL );

    if( vr EQ NULL )
        NL_QUIT;

    if( vl EQ NULL )
        NL_QUIT;

    in = N_AllocInt1dArray( NL_MAX( nn, mm ), &SL );

    if( in EQ NULL )
        NL_QUIT;

    /***********************************/
    /* Compute the u-knot vector first */
    /***********************************/

    /* See if there are coincident parameters at the ends */

    l = 0;

    while( us[l]EQ us[l + 1] )
        l++;

    if( u0 EQ us[0] )
        in[0] = l;
    else
        in[0] = -1;

    l = k;

    while( us[l]EQ us[l - 1] )
        l--;

    if( u1 EQ us[k] )
        in[1] = l - 1;
    else
        in[1] = k;

    /* Get representative parameters */

    ul[0] = u0;
    ur[0] = u0;
    le = in[0] + 1;
    ul[1] = u1;
    ur[nn] = u1;
    ri = in[1];

    i = 2;

    while( i LE nn - 1 )
    {
        /* Get split lines by averaging */

        sum = 0.0;

        for ( j = le; j <= ri; j++ )
            sum += us[j];
        ua = sum / ((NL_REAL)ri - (NL_REAL)le + 1.0);

        /* Get index the average falls in */

        error = N_FindIntervalRealArray( &us[le], ri - le, ua, NL_LEFT, &iu );

        if( error EQ NL_YES )
            NL_OUT;

        if( iu EQ - 1 )
            NL_ERROR( NL_INP_ERR );

        /* Insert ua into the sequence */

        for ( j = 0; j <= i - 1; j++ )
            if( ua LT ul[j] )
                break;

        for ( r = i - 1; r >= j; r-- )
        {
            ul[r + 1] = ul[r];
            in[r + 1] = in[r];
        }

        ul[j] = ua;
        in[j] = le + iu;

        /* Get new left and right indexes */

        if( i LT nn - 1 )
        {
            im = -1;

            for ( j = 0; j <= i - 1; j++ )
            {
                if( (in[j + 1] - in[j])GT im )
                {
                    im = in[j + 1] - in[j];
                    le = in[j] + 1;
                    ri = in[j + 1];
                }
            }
        }

        if( im EQ - 1 )
            NL_ERROR( NL_INP_ERR );

        i++;
    }

    for ( i = 1; i <= nn - 1; i++ )
    {
        sum = 0.0;

        for ( j = in[i - 1] + 1; j <= in[i]; j++ )
            sum += us[j];
        ur[i] = sum / ((NL_REAL)in[i] - (NL_REAL)in[i - 1]);
    }

    /* Now go and get the knot vector */

    if( UC NEQ NULL )
    {
        kni = N_AllocKnotVectorAndArray( n + p + 1, &SL );

        if( kni EQ NULL )
            NL_QUIT;

        knp = N_AllocKnotVectorAndArray( n + p, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knu, &r, &U );
        N_KnotVectorGetKnots( knp, &i, &UP );
        N_KnotVectorGetKnots( kni, &i, &UI );

        for ( i = 0; i <= p; i++ )
        {
            U[i] = ur[0];
            U[n + i + 1] = ur[nn];
        }

        /* Get ideal knot vectors */

        error = N_FitCalcKnotVectorCrvApprox( ur, nn, n, p, kni );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( ur, nn, n, (NL_DEGREE)(p - 1), knp );

        if( error EQ NL_YES )
            NL_OUT;

        /* Select knots from UC */

        j = 0;

        for ( i = p + 1; i <= n; i++ )
        {
            a = (1.0 - per) * UI[i] + per * UP[i - 1];
            b = (1.0 - per) * UI[i] + per * UP[i];

            while( UC[j]LE a )
                j++;

            f = b - a;
            l = -1;

            while( j LE rc AND( UC[j]GT a AND UC[j]LT b ) )
            {
                d = fabs( UI[i] - UC[j] );

                if( d LT f )
                {
                    f = d;
                    l = j;
                }
                j++;
            }

            if( l EQ - 1 )
                U[i] = UI[i];
            else
                U[i] = UC[l];
        }
    }
    else
    {
        error = N_FitCalcKnotVectorCrvApprox( ur, nn, n, p, knu );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /*********************************/
    /* Now compute the v-knot vector */
    /*********************************/

    /* See if there are coincident parameters at the ends */

    l = 0;

    while( vs[l]EQ vs[l + 1] )
        l++;

    if( v0 EQ vs[0] )
        in[0] = l;
    else
        in[0] = -1;

    l = k;

    while( vs[l]EQ vs[l - 1] )
        l--;

    if( v1 EQ vs[k] )
        in[1] = l - 1;
    else
        in[1] = k;

    /* Get representative parameters */

    vl[0] = v0;
    vr[0] = v0;
    le = in[0] + 1;
    vl[1] = v1;
    vr[mm] = v1;
    ri = in[1];

    i = 2;

    while( i LE mm - 1 )
    {
        /* Get split lines by averaging */

        sum = 0.0;

        for ( j = le; j <= ri; j++ )
            sum += vs[j];
        va = sum / ((NL_REAL)ri - (NL_REAL)le + 1.0);

        /* Get index the average falls in */

        error = N_FindIntervalRealArray( &vs[le], ri - le, va, NL_LEFT, &iv );

        if( error EQ NL_YES )
            NL_OUT;

        if( iv EQ - 1 )
            NL_ERROR( NL_INP_ERR );

        /* Insert va into the sequence */

        for ( j = 0; j <= i - 1; j++ )
            if( va LT vl[j] )
                break;

        for ( s = i - 1; s >= j; s-- )
        {
            vl[s + 1] = vl[s];
            in[s + 1] = in[s];
        }

        vl[j] = va;
        in[j] = le + iv;

        /* Get new left and right indexes */

        if( i LT mm - 1 )
        {
            im = -1;

            for ( j = 0; j <= i - 1; j++ )
            {
                if( (in[j + 1] - in[j])GT im )
                {
                    im = in[j + 1] - in[j];
                    le = in[j] + 1;
                    ri = in[j + 1];
                }
            }
        }

        if( im EQ - 1 )
            NL_ERROR( NL_INP_ERR );

        i++;
    }

    for ( j = 1; j <= mm - 1; j++ )
    {
        sum = 0.0;

        for ( i = in[j - 1] + 1; i <= in[j]; i++ )
            sum += vs[i];
        vr[j] = sum / ((NL_REAL)in[j] - (NL_REAL)in[j - 1]);
    }

    /* Now go and get the knot vector */

    if( VC NEQ NULL )
    {
        kni = N_AllocKnotVectorAndArray( m + q + 1, &SL );

        if( kni EQ NULL )
            NL_QUIT;

        knp = N_AllocKnotVectorAndArray( m + q, &SL );

        if( knp EQ NULL )
            NL_QUIT;

        N_KnotVectorGetKnots( knv, &s, &V );
        N_KnotVectorGetKnots( knp, &i, &VP );
        N_KnotVectorGetKnots( kni, &i, &VI );

        for ( j = 0; j <= q; j++ )
        {
            V[j] = vr[0];
            V[m + j + 1] = vr[mm];
        }

        /* Get ideal knot vectors */

        error = N_FitCalcKnotVectorCrvApprox( vr, mm, m, q, kni );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( vr, mm, m, (NL_DEGREE)(q - 1), knp );

        if( error EQ NL_YES )
            NL_OUT;

        /* Select knots from VC */

        i = 0;

        for ( j = q + 1; j <= m; j++ )
        {
            a = (1.0 - per) * VI[j] + per * VP[j - 1];
            b = (1.0 - per) * VI[j] + per * VP[j];

            while( VC[i]LE a )
                i++;

            f = b - a;
            l = -1;

            while( i LE sc AND( VC[i]GT a AND VC[i]LT b ) )
            {
                d = fabs( VI[j] - VC[i] );

                if( d LT f )
                {
                    f = d;
                    l = i;
                }
                i++;
            }

            if( l EQ - 1 )
                V[j] = VI[j];
            else
                V[j] = VC[l];
        }
    }
    else
    {
        error = N_FitCalcKnotVectorCrvApprox( vr, mm, m, q, knv );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SetKnotIndex( knu, n + p + 1 );
    N_SetKnotIndex( knv, m + q + 1 );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitPlaneToPts: Best fitting planar NURBS patch to random points         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a best fitting  planar patch to a set
     of random 3-D points. A best fit plane is found first which then is  
     segmented  into a  rectangular  region to  contain the entire point 
     set. A typical calling example is:

       NL_POINT    *P;
       NL_INDEX    n;
       NL_REAL     tpc, era, erm;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get P and tpc);
       ...
       N_SrfInitArrays(&sur);
       N_FitPlaneToPts(P,n,tpc,&sur,&era,&erm,&SG);


   ACCESS:
   
     P     , input  ,  Points in 3-D
     n     , input  ,  Highest index in P
     tpc   , input  ,  Tolerance to find best  bounding rectangle; it is
                       expressed as a percentage, e.g., tpc = 0.1  gives 
                       a  bounding rectangle  that is about 1 degree off 
                       the  absolute  minimum. In general, the  bounding 
                       box tolerance is set as  "tpc" times an estimated  
                       minimum  box  area. A good  default is  somewhere 
                       between 0.1 and 0.01.
     sur   , output ,  Best fitting NURBS planar patch
     era   , output ,  Average absolute error
     erm   , output ,  Maximum absolute error
     SG    , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitPlaneToPts 
  (NL_POINT   *P,    /* in : cloud-of-points, sized:n+1]  */
   NL_INDEX    n,    /* in : highest index in p           */
   NL_REAL     tpc,  /* in : percentage tolerance to find best bounding rectangle, good values between 0.01 and 0.1 */
   NL_SURFACE *sur,  /* out: Best fitting NURBS planar patch */
   NL_REAL    *era,  /* out: Average absolute error          */
   NL_REAL    *erm,  /* out: Maximum absolute error          */
   NL_STACKS  *SG )  /* in : sur's stack                     */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitPlaneToPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, im = 0;

    NL_REAL mag, ml;

    NL_POINT *QP, *QT, C, R[4], P00, P10, P01, P11;

    NL_VECTOR AX[4], V, X, Y, Z;

    NL_RMATRIX rma, rmi;

    NL_PLANE pln;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    if( n LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Get best fitting infinite plane and project points onto it */

    QP = N_AllocPt1dArray( n, &SL );

    if( QP EQ NULL )
        NL_QUIT;

    error = N_PlaneFit3dPts( P, n, &C, &Z, era, erm );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreatePlanePtNormal( &pln, C, Z );

    for ( i = 0; i <= n; i++ )
    {
        error = N_ProjectPtPlane( pln, P[i], &QP[i] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get local coordinate system */

    N_VectorCreate( 0.0, 0.0, 0.0, &AX[0] );
    N_VectorCreate( 1.0, 0.0, 0.0, &AX[1] );
    N_VectorCreate( 0.0, 1.0, 0.0, &AX[2] );
    N_VectorCreate( 0.0, 0.0, 1.0, &AX[3] );

    ml = -1.0;

    for ( i = 1; i <= 3; i++ )
    {
        N_VectorCross( Z, AX[i], &V );
        N_VectorMagnitude( V, &mag );

        if( mag GT ml )
        {
            ml = mag;
            im = i;
        }
    }

    N_VectorCross( Z, AX[im], &X );
    N_VectorCross( Z, X, &Y );

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    /* Transform points onto the <x-y> plane */

    QT = N_AllocPt1dArray( n, &SL );

    if( QT EQ NULL )
        NL_QUIT;

    N_InitRealMatrix( &rma );
    error = N_CreateTransformMatrixFromAxes( C, X, Y, Z, AX[0], AX[1], AX[2], AX[3], &rma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= n; i++ )
        N_TransformPt( QP[i], &rma, &QT[i] );

    /* Find best box in 2-D */

    error = N_BoundRect2dPts( QT, n, NL_YES, tpc, &R[0], &R[1], &R[2], &R[3], &mag );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get 3-D rectangle and NURBS plane */

    N_InitRealMatrix( &rmi );
    error = N_RealMatrixInversePivot( &rma, &rmi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_TransformPt( R[0], &rmi, &P00 );
    N_TransformPt( R[1], &rmi, &P10 );
    N_TransformPt( R[2], &rmi, &P11 );
    N_TransformPt( R[3], &rmi, &P01 );

    error = N_CreateSrfCornerPts( P00, P10, P01, P11, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_FitPlaneToPts */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FitPtsNormals: Surface interpolate points and normals array 
                       with C11  bicubic surfaces                     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a C11 continuous nonrational bicubic
     NURBS surface that very nearly interpolates a given array of points 
     and normals. If the output surface is initialized to the NULL 
     surface, memory is  allocated locally. Otherwise it is checked if 
     enough memory is passed in. A typical calling example is:
     

       NL_POINT    **P;
       NL_VECTOR   **N;
       NL_INDEX    k, l;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get array P and N);
       ...
       N_SrfInitArrays(&sur);
       N_FitPtsNormals(P,N,k,l,NL_AKIMA,&sur,&SG);

     A recommended  default for the tangent is NL_AKIMA. See N_FitSrfInterpBicubic.
     use: NL_POINT **P = N_AllocPt2dArray(k, l, &SG);     
          NL_VECTOR **N = N_AllocPt2dArray(k, k, &SG); 


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     N    , input  ,  Normals to be interpolated
     k,l  , input  ,  Highest indexes in P
     tan  , input  ,  Flag:
                        NL_BESSEL: Tangents computed by Bessel's method
                        NL_AKIMA : Tangents computed by Akima's method
     sur  , output ,  Interpolating surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitPtsNormals
  (NL_POINT   ** P,      /* in : Grid of Sample Point Positions, sized:[k+1][l+1] */
   NL_VECTOR  ** N,      /* in : Grid of Sample Point Normals, sized:[k+1][l+1]  */
   NL_INDEX      k,      /* in : highest 1st index in P and N */
   NL_INDEX      l,      /* in : highest 2nd index in P and N  */
   NL_FLAG       tan,    /* in : tangent flag: NL_BESSEL= Tangents computed by Bessel's method */
                         /*                    NL_AKIMA = Tangents computed by Akima's method  */
   NL_SURFACE   *sur,    /* out: approximated surface */
   NL_STACKS    *SG )    /* in : sur's memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitPtsNormals");

    NL_FLAG uclosed = NL_NO, vclosed = NL_NO, error = NL_NO;

    NL_INDEX i, j, ii, jj, n, m, ib, jb;

    NL_REAL *U, *V, dui, dui1, dvj, dvj1, alf, oma, bet, omb, d, a, b, gam, omg, utl, vtl, od3, od9;

    NL_PARAMETER *u, *v, *r, *s;

    NL_POINT ** B;

    NL_CPOINT ** Pw;

    NL_VECTOR ** SU, ** SV, ** TU, ** TV, ** DUV, AV, BV, D1, D2;

    NL_ENET ntl;

    NL_PLANE Pln;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Set some constants */

    ib = 3 * k;
    jb = 3 * l;
    n = 2 * k + 1;
    m = 2 * l + 1;
    od3 = 1.0 / 3.0;
    od9 = 1.0 / 9.0;

    /* Check for surface memory */

    error = N_SrfSizeArrays( sur, n, m, 3, 3, n + 4, m + 4, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Allocate memory */

    u = N_AllocReal1dArray( k + 2, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( l + 2, &SL );

    if( v EQ NULL )
        NL_QUIT;

    r = N_AllocReal1dArray( l, &SL );

    if( r EQ NULL )
        NL_QUIT;

    s = N_AllocReal1dArray( k, &SL );

    if( s EQ NULL )
        NL_QUIT;

    B = N_AllocPt2dArray( ib, jb, &SL );

    if( B EQ NULL )
        NL_QUIT;

    SU = N_AllocPt2dArray( k + 3, l, &SL );

    if( SU EQ NULL )
        NL_QUIT;

    SV = N_AllocPt2dArray( k, l + 3, &SL );

    if( SV EQ NULL )
        NL_QUIT;

    TU = N_AllocPt2dArray( k + 2, l + 2, &SL );

    if( TU EQ NULL )
        NL_QUIT;

    TV = N_AllocPt2dArray( k + 2, l + 2, &SL );

    if( TV EQ NULL )
        NL_QUIT;

    DUV = N_AllocPt2dArray( k, l, &SL );

    if( DUV EQ NULL )
        NL_QUIT;

    /* Compute the chords and the parameters */

    N_ENetFromPts( &ntl, k, l, P );

    if( N_ENetIsClosed( &ntl, NL_UDIR ) )
        uclosed = NL_YES;

    if( N_ENetIsClosed( &ntl, NL_VDIR ) )
        vclosed = NL_YES;

    utl = 0.0;
    vtl = 0.0;

    for ( i = 0; i <= k + 2; i++ )
        u[i] = 0.0;

    for ( j = 0; j <= l + 2; j++ )
        v[j] = 0.0;

    for ( j = 0; j <= l; j++ )
    {
        r[j] = 0.0;

        for ( i = 1; i <= k; i++ )
        {
            N_VectorDiff( P[i][j], P[i - 1][j], &SU[i + 1][j] );
            N_VectorMagnitude( SU[i + 1][j], &d );

            u[i + 1] += d;
            r[j] += d;
        }

        if( uclosed EQ NL_YES )
        {
            N_VectorCopy( SU[k + 1][j], &SU[1][j] );
            N_VectorCopy( SU[k][j], &SU[0][j] );
            N_VectorCopy( SU[2][j], &SU[k + 2][j] );
            N_VectorCopy( SU[3][j], &SU[k + 3][j] );
        }
        else
        {
            N_VectorCombine( 2.0, SU[2][j], -1.0, SU[3][j], &SU[1][j] );
            N_VectorCombine( 2.0, SU[1][j], -1.0, SU[2][j], &SU[0][j] );
            N_VectorCombine( 2.0, SU[k + 1][j], -1.0, SU[k][j], &SU[k + 2][j] );
            N_VectorCombine( 2.0, SU[k + 2][j], -1.0, SU[k + 1][j], &SU[k + 3][j] );
        }

        utl += r[j];
    }

    for ( i = 0; i <= k; i++ )
    {
        s[i] = 0.0;

        for ( j = 1; j <= l; j++ )
        {
            N_VectorDiff( P[i][j], P[i][j - 1], &SV[i][j + 1] );
            N_VectorMagnitude( SV[i][j + 1], &d );

            v[j + 1] += d;
            s[i] += d;
        }

        if( vclosed EQ NL_YES )
        {
            N_VectorCopy( SV[i][l + 1], &SV[i][1] );
            N_VectorCopy( SV[i][l], &SV[i][0] );
            N_VectorCopy( SV[i][2], &SV[i][l + 2] );
            N_VectorCopy( SV[i][3], &SV[i][l + 3] );
        }
        else
        {
            N_VectorCombine( 2.0, SV[i][2], -1.0, SV[i][3], &SV[i][1] );
            N_VectorCombine( 2.0, SV[i][1], -1.0, SV[i][2], &SV[i][0] );
            N_VectorCombine( 2.0, SV[i][l + 1], -1.0, SV[i][l], &SV[i][l + 2] );
            N_VectorCombine( 2.0, SV[i][l + 2], -1.0, SV[i][l + 1], &SV[i][l + 3] );
        }

        vtl += s[i];
    }

    for ( i = 2; i <= k; i++ )
        u[i] = u[i - 1] + u[i] / utl;

    u[0] = -u[2];
    u[k + 1] = 1.0;
    u[k + 2] = 2.0 - u[k];

    for ( j = 2; j <= l; j++ )
        v[j] = v[j - 1] + v[j] / vtl;

    v[0] = -v[2];
    v[l + 1] = 1.0;
    v[l + 2] = 2.0 - v[l];

    /* Get knot vectors */

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = 0.0;
        U[n + i + 1] = 1.0;
    }

    for ( i = 1; i < k; i++ )
    {
        ii = 2 * (i + 1);
        U[ii] = u[i + 1];
        U[ii + 1] = u[i + 1];
    }

    for ( j = 0; j <= 3; j++ )
    {
        V[j] = 0.0;
        V[m + j + 1] = 1.0;
    }

    for ( j = 1; j < l; j++ )
    {
        jj = 2 * (j + 1);
        V[jj] = v[j + 1];
        V[jj + 1] = v[j + 1];
    }

    /* Compute the unit tangents */

    switch( tan )
    {
        case NL_BESSEL:
            for ( i = 0; i <= k; i++ )
            {
                dui = u[i + 1] - u[i];
                dui1 = u[i + 2] - u[i + 1];

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

                for ( j = 0; j <= l; j++ )
                {
                    dvj = v[j + 1] - v[j];
                    dvj1 = v[j + 2] - v[j + 1];

                    if( N_FloatOpIsBad( dvj, dvj + dvj1, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    bet = dvj / (dvj + dvj1);
                    omb = 1.0 - bet;

                    if( N_FloatOpIsBad( omb, dvj, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );

                    if( N_FloatOpIsBad( bet, dvj1, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    omb = omb / dvj;
                    bet = bet / dvj1;

                    N_VectorCombine( oma, SU[i + 1][j], alf, SU[i + 2][j], &TU[i + 1][j + 1] );
                    N_VectorCombine( omb, SV[i][j + 1], bet, SV[i][j + 2], &TV[i + 1][j + 1] );

                    error = N_VectorNormalizeRef( &TU[i + 1][j + 1] );

                    if( error EQ NL_YES )
                    {
                        if( i EQ 0 OR i EQ k )
                        {
                            if( i EQ 0 )
                                N_VectorCopy( SU[i + 2][j], &TU[i + 1][j + 1] );

                            if( i EQ k )
                                N_VectorCopy( SU[i + 1][j], &TU[i + 1][j + 1] );

                            error = N_VectorNormalizeRef( &TU[i + 1][j + 1] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                        else
                        {
                            NL_OUT;
                        }
                    }

                    error = N_VectorNormalizeRef( &TV[i + 1][j + 1] );

                    if( error EQ NL_YES )
                    {
                        if( j EQ 0 OR j EQ l )
                        {
                            if( j EQ 0 )
                                N_VectorCopy( SV[i][j + 2], &TV[i + 1][j + 1] );

                            if( j EQ l )
                                N_VectorCopy( SV[i][j + 1], &TV[i + 1][j + 1] );

                            error = N_VectorNormalizeRef( &TV[i + 1][j + 1] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                        else
                        {
                            NL_OUT;
                        }
                    }
                }
            }
            break;

        case NL_AKIMA:
            for ( i = 0; i <= k; i++ )
            {
                for ( j = 0; j <= l; j++ )
                {
                    N_VectorCross( SU[i][j], SU[i + 1][j], &AV );
                    N_VectorMagnitude( AV, &a );
                    N_VectorCross( SU[i + 2][j], SU[i + 3][j], &BV );
                    N_VectorMagnitude( BV, &b );

                    if( (a + b)GT NL_LTOL )
                        alf = a / (a + b);
                    else
                        alf = 0.5;
                    oma = 1.0 - alf;

                    N_VectorCross( SV[i][j], SV[i][j + 1], &AV );
                    N_VectorMagnitude( AV, &a );
                    N_VectorCross( SV[i][j + 2], SV[i][j + 3], &BV );
                    N_VectorMagnitude( BV, &b );

                    if( (a + b)GT NL_LTOL )
                        bet = a / (a + b);
                    else
                        bet = 0.5;
                    omb = 1.0 - bet;

                    N_VectorCombine( oma, SU[i + 1][j], alf, SU[i + 2][j], &TU[i + 1][j + 1] );
                    N_VectorCombine( omb, SV[i][j + 1], bet, SV[i][j + 2], &TV[i + 1][j + 1] );

                    error = N_VectorNormalizeRef( &TU[i + 1][j + 1] );

                    if( error EQ NL_YES )
                    {
                        if( i EQ 0 OR i EQ k )
                        {
                            if( i EQ 0 )
                                N_VectorCopy( SU[i + 2][j], &TU[i + 1][j + 1] );

                            if( i EQ k )
                                N_VectorCopy( SU[i + 1][j], &TU[i + 1][j + 1] );

                            error = N_VectorNormalizeRef( &TU[i + 1][j + 1] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                        else
                        {
                            NL_OUT;
                        }
                    }

                    error = N_VectorNormalizeRef( &TV[i + 1][j + 1] );

                    if( error EQ NL_YES )
                    {
                        if( j EQ 0 OR j EQ l )
                        {
                            if( j EQ 0 )
                                N_VectorCopy( SV[i][j + 2], &TV[i + 1][j + 1] );

                            if( j EQ l )
                                N_VectorCopy( SV[i][j + 1], &TV[i + 1][j + 1] );

                            error = N_VectorNormalizeRef( &TV[i + 1][j + 1] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                        else
                        {
                            NL_OUT;
                        }
                    }
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Get Bezier points along rows and columns */

    for ( j = 0; j <= l; j++ )
    {
        jj = 3 * j;

        for ( i = 0; i < k; i++ )
        {
            ii = 3 * i;
            a = od3 * r[j] * (u[i + 2] - u[i + 1]);

            N_CopyPt( P[i][j], &B[ii][jj] );
            N_VectorPtAlongVector( P[i][j], a, TU[i + 1][j + 1], &B[ii + 1][jj] );
            N_VectorPtAlongVector( P[i + 1][j], -a, TU[i + 2][j + 1], &B[ii + 2][jj] );
        }

        N_CopyPt( P[k][j], &B[ib][jj] );
    }

    for ( i = 0; i <= k; i++ )
    {
        ii = 3 * i;

        for ( j = 0; j < l; j++ )
        {
            jj = 3 * j;
            b = od3 * s[i] * (v[j + 2] - v[j + 1]);

            N_VectorPtAlongVector( P[i][j], b, TV[i + 1][j + 1], &B[ii][jj + 1] );
            N_VectorPtAlongVector( P[i][j + 1], -b, TV[i + 1][j + 2], &B[ii][jj + 2] );
        }
    }

    /* Get mixed partial derivatives */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            N_VectorScale( TU[i + 1][j + 1], r[j], &TU[i + 1][j + 1] );
            N_VectorScale( TV[i + 1][j + 1], s[i], &TV[i + 1][j + 1] );
        }
    }

    for ( i = 0; i <= k; i++ )
    {
        N_VectorCombine( 2.0, TU[i + 1][1], -1.0, TU[i + 1][2], &TU[i + 1][0] );
        N_VectorCombine( 2.0, TU[i + 1][l + 1], -1.0, TU[i + 1][l], &TU[i + 1][l + 2] );
    }

    for ( j = 0; j <= l; j++ )
    {
        N_VectorCombine( 2.0, TV[1][j + 1], -1.0, TV[2][j + 1], &TV[0][j + 1] );
        N_VectorCombine( 2.0, TV[k + 1][j + 1], -1.0, TV[k][j + 1], &TV[k + 2][j + 1] );
    }

    for ( i = 0; i <= k; i++ )
    {
        dui = u[i + 1] - u[i];
        dui1 = u[i + 2] - u[i + 1];

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

        for ( j = 0; j <= l; j++ )
        {
            dvj = v[j + 1] - v[j];
            dvj1 = v[j + 2] - v[j + 1];

            if( N_FloatOpIsBad( dvj, dvj + dvj1, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            bet = dvj / (dvj + dvj1);
            omb = 1.0 - bet;

            if( N_FloatOpIsBad( omb, dvj, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            if( N_FloatOpIsBad( bet, dvj1, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            omb = omb / dvj;
            bet = bet / dvj1;

            N_VectorDiff( TV[i + 1][j + 1], TV[i][j + 1], &D1 );
            N_VectorDiff( TV[i + 2][j + 1], TV[i + 1][j + 1], &D2 );
            N_VectorCombine( oma, D1, alf, D2, &AV );

            N_VectorDiff( TU[i + 1][j + 1], TU[i + 1][j], &D1 );
            N_VectorDiff( TU[i + 1][j + 2], TU[i + 1][j + 1], &D2 );
            N_VectorCombine( omb, D1, bet, D2, &BV );

            gam = alf / (alf + bet);
            omg = 1.0 - gam;

            N_VectorCombine( omg, AV, gam, BV, &DUV[i][j] );
        }
    }

    /* Compute inner Bezier points */

    for ( i = 0; i < k; i++ )
    {
        ii = 3 * i;
        dui1 = u[i + 2] - u[i + 1];

        for ( j = 0; j < l; j++ )
        {
            jj = 3 * j;
            dvj1 = v[j + 2] - v[j + 1];
            gam = od9 * dui1 * dvj1;

            N_Combine4Pts( gam, DUV[i][j], 1.0, B[ii + 1][jj], 1.0, B[ii][jj + 1], -1.0, B[ii][jj], &B[ii + 1][jj + 1] );

            N_Combine4Pts( -gam, DUV[i + 1][j], 1.0, B[ii + 2][jj], 1.0, B[ii + 3][jj + 1], -1.0, B[ii + 3][jj], &B[ii + 2][jj + 1] );

            N_Combine4Pts( -gam, DUV[i][j + 1], 1.0, B[ii][jj + 2], 1.0, B[ii + 1][jj + 3], -1.0, B[ii][jj + 3], &B[ii + 1][jj + 2] );

            N_Combine4Pts( gam, DUV[i + 1][j + 1], 1.0, B[ii + 3][jj + 2], 1.0, B[ii + 2][jj + 3], -1.0, B[ii + 3][jj + 3], &B[ii + 2][jj + 2] );
        }
    }

    /* Reset the Bij by projecting the neighboring Bij onto the plane at each point
    First reset at each corner */

    if( N )
    {
        /* if no normals,then bypass projection */
        /* At P00, the neighbors are 
                          B[0][1], B[1][1] and B[1][0]  */
        N_CreatePlanePtNormal( &Pln, P[0][0], N[0][0] ); /* the plane at P00 */
        N_ProjectPtPlane( Pln, B[0][1], &B[0][1] );
        N_ProjectPtPlane( Pln, B[1][1], &B[1][1] );
        N_ProjectPtPlane( Pln, B[1][0], &B[1][0] );

        /* At Pk0, the neighbors are 
                          B[ib-1][0], B[ib-1][1] and B[ib][1]  */
        N_CreatePlanePtNormal( &Pln, P[k][0], N[k][0] ); /* the plane at Pk0 */
        N_ProjectPtPlane( Pln, B[ib - 1][0], &B[ib - 1][0] );
        N_ProjectPtPlane( Pln, B[ib - 1][1], &B[ib - 1][1] );
        N_ProjectPtPlane( Pln, B[ib][1], &B[ib][1] );

        /* At P0l, the neighbors are 
                          B[0][jb-1], B[1][jb-1] and B[jb][jb]  */
        N_CreatePlanePtNormal( &Pln, P[0][l], N[0][l] ); /* the plane at P0l */
        N_ProjectPtPlane( Pln, B[0][jb - 1], &B[0][jb - 1] );
        N_ProjectPtPlane( Pln, B[1][jb - 1], &B[1][jb - 1] );
        N_ProjectPtPlane( Pln, B[1][jb], &B[1][jb] );

        /* At Pkl, the neighbors are 
                          B[ib-1][jb], B[ib-1][jb-1] and B[ib][jb-1]  */
        N_CreatePlanePtNormal( &Pln, P[k][l], N[k][l] ); /* the plane at Pkl */
        N_ProjectPtPlane( Pln, B[ib - 1][jb], &B[ib - 1][jb] );
        N_ProjectPtPlane( Pln, B[ib - 1][jb - 1], &B[ib - 1][jb - 1] );
        N_ProjectPtPlane( Pln, B[ib][jb - 1], &B[ib][jb - 1] );

        /* Reset along each side
        note: the Bij in the rows and columns of 
        P and N are not used in the final srf 
              hence these are not reset.     */

        /* Along j = 0 and j = jb, the neighbors are
                    B[i-1][.], B[i-1][.], B[i+1][.], B[i+1][.]  */
        ii = 0;

        for ( i = 1; i < ib; i++ )
        {
            if( i % 3 EQ 0 )
            {
                /* reset at j = 0 and j = l  */
                ii++;
                N_CreatePlanePtNormal( &Pln, P[ii][0], N[ii][0] ); /* the plane at Pii0 */
                N_ProjectPtPlane( Pln, B[i - 1][0], &B[i - 1][0] );
                N_ProjectPtPlane( Pln, B[i - 1][1], &B[i - 1][1] );
                N_ProjectPtPlane( Pln, B[i + 1][1], &B[i + 1][1] );
                N_ProjectPtPlane( Pln, B[i + 1][0], &B[i + 1][0] );

                N_CreatePlanePtNormal( &Pln, P[ii][l], N[ii][l] ); /* the plane at Piil */
                N_ProjectPtPlane( Pln, B[i - 1][jb], &B[i - 1][jb] );
                N_ProjectPtPlane( Pln, B[i - 1][jb - 1], &B[i - 1][jb - 1] );
                N_ProjectPtPlane( Pln, B[i + 1][jb - 1], &B[i + 1][jb - 1] );
                N_ProjectPtPlane( Pln, B[i + 1][jb], &B[i + 1][jb] );
            }
        }

        /* Along i = 0 and i = ib, the neighbors are
                          B[.][j-1], B[.][j-1], B[.][j+1], B[.][j+1]  */
        jj = 0;

        for ( j = 1; j < jb; j++ )
        {
            if( j % 3 EQ 0 )
            {
                /* reset at i = 0 and i = k */
                jj++;
                N_CreatePlanePtNormal( &Pln, P[0][jj], N[0][jj] ); /* the plane at P0jj */
                N_ProjectPtPlane( Pln, B[0][j - 1], &B[0][j - 1] );
                N_ProjectPtPlane( Pln, B[1][j - 1], &B[1][j - 1] );
                N_ProjectPtPlane( Pln, B[1][j + 1], &B[1][j + 1] );
                N_ProjectPtPlane( Pln, B[0][j + 1], &B[0][j + 1] );

                N_CreatePlanePtNormal( &Pln, P[k][jj], N[k][jj] ); /* the plane at Pkjj */
                N_ProjectPtPlane( Pln, B[ib][j - 1], &B[ib][j - 1] );
                N_ProjectPtPlane( Pln, B[ib - 1][j - 1], &B[ib - 1][j - 1] );
                N_ProjectPtPlane( Pln, B[ib - 1][j + 1], &B[ib - 1][j + 1] );
                N_ProjectPtPlane( Pln, B[ib][j + 1], &B[ib][j + 1] );
            }
        }

        /* For the interior points the neighbors of Piijj are
                          B[i-1][j-1], B[i-1][j+1], B[i+1][j+1], B[i+1][j-1]  */

        jj = 0;

        for ( j = 1; j < jb; j++ )
        {
            if( j % 3 EQ 0 )
            {
                jj++;
                ii = 0;

                for ( i = 1; i < ib; i++ )
                {
                    if( i % 3 EQ 0 )
                    {
                        ii++;
                        N_CreatePlanePtNormal( &Pln, P[ii][jj], N[ii][jj] ); /* the plane at Piijj */
                        N_ProjectPtPlane( Pln, B[i - 1][j - 1], &B[i - 1][j - 1] );
                        N_ProjectPtPlane( Pln, B[i - 1][j + 1], &B[i - 1][j + 1] );
                        N_ProjectPtPlane( Pln, B[i + 1][j + 1], &B[i + 1][j + 1] );
                        N_ProjectPtPlane( Pln, B[i + 1][j - 1], &B[i + 1][j - 1] );
                    }
                }
            }
        }
    } /* end  if (N)  */
    /* Compute control points */

    N_PtToCPt( P[0][0], &Pw[0][0] );
    N_PtToCPt( P[k][0], &Pw[n][0] );
    N_PtToCPt( P[0][l], &Pw[0][m] );
    N_PtToCPt( P[k][l], &Pw[n][m] );

    ii = 0;

    for ( i = 1; i < ib; i++ )
    {
        if( (i % 3)EQ 0 )
            continue;

        ii++;
        N_PtToCPt( B[i][0], &Pw[ii][0] );
        N_PtToCPt( B[i][jb], &Pw[ii][m] );
    }

    jj = 0;

    for ( j = 1; j < jb; j++ )
    {
        if( (j % 3)EQ 0 )
            continue;

        jj++;
        N_PtToCPt( B[0][j], &Pw[0][jj] );
        N_PtToCPt( B[ib][j], &Pw[n][jj] );
    }

    ii = 0;

    for ( i = 1; i < ib; i++ )
    {
        if( (i % 3)EQ 0 )
            continue;

        ii++;
        jj = 0;

        for ( j = 1; j < jb; j++ )
        {
            if( (j % 3)EQ 0 )
                continue;

            jj++;
            N_PtToCPt( B[i][j], &Pw[ii][jj] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFCALCPARAMS: Compute parameters for surface fit of random data        */
/**********************************************************************/

NL_PRIVATE_TLS NL_REAL ** pts2;
NL_PRIVATE_TLS NL_REAL ** p2;
NL_PRIVATE_TLS NL_INDEX glo_np;
/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  computes the u,v parameters for fitting  an
     approximation  surface  to a set of random (non-NxM) points.  This
     routine does not provide for input of boundary curves. It projects
     the points to a plane,  and hence,  yields a good parameterization
     only in the case that the surface  represents  a  unique  function 
     over the plane. The plane,  together with coordinate axes lying in
     the plane,  may be input,  or this routine will compute them.  The 
     axes are the images of the orthogonal (u,v)-vectors  defining  the  
     parameterization of the points, i.e. the axes indicate the flow of
     the parameter lines in the vicinity of the given 3D point. A typi-
     cal calling example is:
 
       NL_POINT      *P;
       NL_PARAMETER  us, ue, vs, ve;
       NL_PARAMETER  *uu, *vv;
       NL_INDEX      np;
       NL_POINT      Og;
       NL_VECTOR     X, Y, Z;
       NL_FLAG       flg;
       ...
       (define array P, allocate arrays uu and vv, and optionally,
        choose (Og,X,Y,Z));
       ...
       N_FitSrfCalcParams(P,np,flg,Og,X,Y,Z,us,ue,vs,ve,uu,vv);
 
 
   ACCESS:
   
     P   , input  ,  Points requiring parameterization
     np  , input  ,  The high index of the arrays: P,uu,vv
     flg , input  ,  Flag:
                      NL_YES : The plane and axes (Og,X,Y,Z) are input
                      NL_NO  : They are not input,  they are  computed  in
                            this routine
     Og  , input  ,  Point on the plane (origin of local coord system)
     X,Y , input  ,  Axes defining local  coord  system  on  the  plane 
                     (images of the orthogonal u,v-vectors). These vec-
                     tors indicate the two principal directions of  the 
                     point cloud. Note that they do not have to be  or-
                     thogonal,  however,  they  must be orthogonal to Z
                     and not parallel to one another
     Z   , input  ,  Normal to the plane
     us  , input  ,  All parameters in uu and vv are scaled  to  values
     ue,vs,ve        in the box, [us,ue]x[vs,ve]
     uu  , output ,  The u-parameters of the points in P.  This  memory
                     must be passed in
     vv  , output ,  The v-parameters of the points in P.  This  memory
                     must be passed in

  
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfCalcParams
  (NL_POINT     *P,     /* Points to parameterize, sized:[np+1] */
   NL_INDEX      np,    /* highest P index  */
   NL_FLAG       flg,   /* NL_YES = plane and axes are input, NL_N0 = to be computed */
   NL_POINT      Og,    /*   Point on the plane (origin of coord system) */
   NL_VECTOR     X,     /*   First vector of projection plane */
   NL_VECTOR     Y,     /*   Second vector of projection plane */
   NL_VECTOR     Z,     /*   Normal to projection plane */
   NL_PARAMETER  us,    /* output U param Start value */
   NL_PARAMETER  ue,    /* output U param End value   */
   NL_PARAMETER  vs,    /* output V param Start value  */
   NL_PARAMETER  ve,    /* output V param End value    */
   NL_PARAMETER *uu,    /* Output U param values per point, sized:[np+1] */
   NL_PARAMETER *vv )   /* Output V param values per point, sized:[np+1] */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfCalcParams");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj = 0, k1, k2;

    NL_REAL ** RM, d1, minu, maxu, minv, maxv, rhs1, rhs2, f1, f2 ;
    NL_REAL tt[8], ff[8], du, dv, a, b, c, d, ttol, tmin, x, y, z;

    NL_POINT O1, *pts = NULL, Q;

    NL_VECTOR X1, Y1, Z1, V;

    NL_RMATRIX rma;

    NL_PLANE pln;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Branch on whether or not local system input */

    if( flg EQ NL_YES )
    {
        /* Check for bad input */

        N_VectorCross( X, Y, &Z1 );
        error = N_VectorsAngle( Z, Z1, &d1 );

        if( error EQ NL_YES OR d1 GT 0.5 )
            NL_ERROR( NL_INP_ERR );

        error = N_VectorNormalize( X, &X1, &d1 );
        error = N_VectorNormalize( Y, &Y1, &d1 );

        /* Make plane defined by (Og,Z) */

        N_CreatePlanePtNormal( &pln, Og, Z );

        O1 = Og;
    }
    else
    {
        /* Get least squares plane and center of gravity point */

        error = N_FitPlanePts( P, np, &a, &b, &c, &d );   /* [a b c] = plane unit normal */

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCreate( a, b, c, &Z1 );   /* Z1 = plane unit normal */

        N_CopyPt( P[0], &Q );

        for ( ii = 1; ii <= np; ii++ )
            N_Sum2Pts( Q, P[ii], &Q );
        d = 1.0 / ((NL_REAL)np + 1.0);
        N_ScalePt( d, Q, &O1 );           /* O1 = center of gravity point */

        N_CreatePlanePtNormal( &pln, O1, Z1 ); /* projection plane */

        /* Now project all points to the plane */

        pts = N_AllocPt1dArray( np, &SL );

        if( pts EQ NULL )
            NL_QUIT;

        for ( ii = 0; ii <= np; ii++ )
        {
            error = N_ProjectPtPlane( pln, P[ii], &pts[ii] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        /* Get an orthonormal system on the plane */

        N_VectorCross( Z1, NL_UNITX, &X1 );  /* try 1, Let X1 = Cross(Z1,X) */
        N_VectorMagnitude( X1, &d1 );
        N_VectorCross( Z1, NL_UNITY, &V );   /* try 2, Let X1 = Cross(z1,Y) */
        N_VectorMagnitude( V, &d );

        if( d GT d1 )  /* keep the try most orthogonal to Z1 */
        {
            d1 = d;
            X1 = V;
        }
        N_VectorCross( Z1, NL_UNITZ, &V );   /* try 3, Let X1 = Cross(z1,Z) */
        N_VectorMagnitude( V, &d );

        if( d GT d1 )
            X1 = V;   /* keep the try most orthogonal to Z1 */

        error = N_VectorNormalize( X1, &X1, &d1 );
        N_VectorCross( Z1, X1, &Y1 );
        error = N_VectorNormalize( Y1, &Y1, &d1 );

        /* Transform the points into 2D */

        N_InitRealMatrix( &rma );
        error = N_CreateTransformMatrixFromAxes( O1, X1, Y1, Z1, NL_ZERO, NL_UNITX, NL_UNITY, NL_UNITZ, &rma, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        pts2 = N_AllocReal2dArray( np, 1, &SL );

        if( pts2 EQ NULL )
            NL_QUIT;
        /* transform all points into new coordinate system - let coord1 = u, coor2 = v */
        for ( ii = 0; ii <= np; ii++ )
        {
            N_TransformPt( P[ii], &rma, &Q );
            N_PtToXYZ( Q, &pts2[ii][0], &pts2[ii][1], &d );
        }

        /* find a rotation in the projection plane that will give */
        /* the projected points the smallest area 2d bounding box */

        /* Consider the function f(t). The value of f is       */
        /* the area of the minmax box of the points in pts2    */
        /* in the coord system obtained by rotating t degrees. */
        /* ST_CalcAreaRotPts() is the implementation of function f(t).  */

        p2 = N_AllocReal2dArray( np, 1, &SL ); /* p2 and glo_np are required */

        if( p2 EQ NULL )
            NL_QUIT;                           /* to evaluate f(t)           */

        glo_np = np;
        ttol = 1.e-5;

        /* Now find the orientation that yields the minimum area box. */
        /* First bracket the minimum, then find it.                   */

        tt[1] = 0.0;
        tt[2] = 15.0;
        tt[3] = 35.0; /* get minimum of these 6 */
        tt[4] = 53.0;
        tt[5] = 75.0;
        tt[6] = 98.0;

        f1 = NL_BIGD;

        for ( ii = 1; ii <= 6; ii++ )
        {
            ff[ii] = ST_CalcAreaRotPts( tt[ii] );

            if( ff[ii]LT f1 )
            {
                f1 = ff[ii];
                jj = ii; /* jj is index of minimum */
            }
        }

        /* now get two neighbors that are bigger (bracket) */

        k1 = k2 = -1;

        for ( ii = jj - 1; ii >= 1; ii-- )
            if( ff[ii]GT ff[jj] )
            {
                k1 = ii;
                break;
            }

        if( k1 EQ - 1 )
        {
            tt[0] = -10.0;
            ff[0] = ST_CalcAreaRotPts( tt[0] );

            if( ff[0]GT ff[jj] )
                k1 = 0;
        }

        if( k1 GT - 1 )
        {
            for ( ii = jj + 1; ii <= 6; ii++ )
                if( ff[ii]GT ff[jj] )
                {
                    k2 = ii;
                    break;
                }

            if( k2 EQ - 1 )
            {
                tt[7] = 108.0;
                ff[7] = ST_CalcAreaRotPts( tt[7] );

                if( ff[7]GT ff[jj] )
                    k2 = 7;
            }
        }

        if( k1 EQ - 1 OR k2 EQ - 1 )
            tmin = tt[jj];
        else
            N_FuncFindMinima( tt[k1], tt[jj], tt[k2], ff[jj], ST_CalcAreaRotPts, ttol, 0.0, &tmin, &d );

        /* Now rotate X1,Y1 through tmin degrees */

        if( tmin NEQ 0.0 )
        {
            N_GetRealMatrixPtr( &rma, &RM );
            tmin = tmin * (NL_PI / 180.0);
            a = sin( tmin );
            b = cos( tmin );
            N_PtToXYZ( Z1, &x, &y, &z );

            RM[0][0] = x * x + b * (1.0 - x * x);
            RM[0][1] = x * y * (1.0 - b) - z * a;
            RM[0][2] = z * x * (1.0 - b) + y * a;
            RM[1][0] = x * y * (1.0 - b) + z * a;
            RM[1][1] = y * y + b * (1.0 - y * y);
            RM[1][2] = y * z * (1.0 - b) - x * a;
            RM[2][0] = z * x * (1.0 - b) - y * a;
            RM[2][1] = y * z * (1.0 - b) + x * a;
            RM[2][2] = z * z + b * (1.0 - z * z);

            N_PtToXYZ( X1, &x, &y, &z );
            a = RM[0][0] * x + RM[0][1] * y + RM[0][2] * z;
            b = RM[1][0] * x + RM[1][1] * y + RM[1][2] * z;
            c = RM[2][0] * x + RM[2][1] * y + RM[2][2] * z;
            N_VectorCreate( a, b, c, &X1 );
            error = N_VectorNormalize( X1, &X1, &d1 );

            N_PtToXYZ( Y1, &x, &y, &z );
            a = RM[0][0] * x + RM[0][1] * y + RM[0][2] * z;
            b = RM[1][0] * x + RM[1][1] * y + RM[1][2] * z;
            c = RM[2][0] * x + RM[2][1] * y + RM[2][2] * z;
            N_VectorCreate( a, b, c, &Y1 );
            error = N_VectorNormalize( Y1, &Y1, &d1 );
        }
    }

    /* Project all points to the plane and get their local */
    /* coordinates (which are the (u,v)-values).           */

    minu = minv = 0.001 *NL_BIGD; /* need these for re-scaling */
    maxu = maxv = -minu;

    RM = N_AllocReal2dArray( 1, 1, &SL );

    /* handle cases where X1 and Y1 are either orthogonal or not.
    ** let V  = projection of point onto UV plane in axis aligned coordinates
    ** get UV = decomposition of those coordinates into X1, Y1 coordinates as
    **   V = u * X1 + v * Y1
    **   solve for u and v in the matrix equation
    **     [ X1.Dot(V) ] = [ 1.0    X1.Dot(Y1) ] [ u ]   where X1.Dot(X1) = 1.0 and
    **     [ Y1.Dot(V) ] = [ Y1.Dot(X1)   1.0  ] [ v ]         Y1.Dot(y1) = 1.0.
    */
    if( RM EQ NULL )
        NL_QUIT;
    RM[0][0] = RM[1][1] = 1.0;
    N_VectorDot( X1, Y1, &RM[0][1] );
    RM[1][0] = RM[0][1];

    N_CreateRealMatrix( &rma, 1, 1, RM, NL_MT_FULL, 1 );
    error = N_RealMatrixLuDecompose( &rma );

    if( error EQ NL_YES )
        NL_OUT;

    /* for every point - project to plane and save min/max u/v values */
    for ( ii = 0; ii <= np; ii++ )
    {
        if( flg EQ NL_YES )
        {
            error = N_ProjectPtPlane( pln, P[ii], &Q );

            if( error EQ NL_YES )
                NL_OUT;
            N_VectorDir( O1, Q, &V );
        }
        else
            N_VectorDir( O1, pts[ii], &V );

        /* decompose plane vector, V, into X, Y coordinates */
        N_VectorDot( V, X1, &rhs1 );
        N_VectorDot( V, Y1, &rhs2 );

        /* compute u and v coordinates in the X1, Y1 vector system */
        f1 = rhs1;              /* forward substitution */
        f2 = rhs2 - f1 * RM[1][0];

        vv[ii] = f2 / RM[1][1]; /* backward substitution */
        uu[ii] = (f1 - vv[ii] * RM[0][1]) / RM[0][0];

        /* save min/max u/v values */
        if( uu[ii]LT minu )
            minu = uu[ii];

        if( uu[ii]GT maxu )
            maxu = uu[ii];

        if( vv[ii]LT minv )
            minv = vv[ii];

        if( vv[ii]GT maxv )
            maxv = vv[ii];
    } /* end iter every point projecting into X1,Y1 vector coordinates */

    /* Now re-scale to [us,ue]x[vs,ve] */

    du = (ue - us) / (maxu - minu);
    dv = (ve - vs) / (maxv - minv);

    for ( ii = 0; ii <= np; ii++ )
    {
        uu[ii] = us + du * (uu[ii] - minu);

        if( uu[ii]GT ue )
            uu[ii] = ue;
        vv[ii] = vs + dv * (vv[ii] - minv);

        if( vv[ii]GT ve )
            vv[ii] = ve;
    } /* end iter every point scaling to given parameter range */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_FitSrfCalcParams */

#if NLIB_UNUSED

/**********************************************************************/
/* N_FitCalcSrfParamsBoundary: Compute parameters for surface fit of random data        */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  computes the u,v parameters for fitting an
     approximation  surface to a set of random (non-NxM) points.  This
     routine  requires all four boundary curves of the surface;  these
     curves must be nonrational and already compatible in the b-spline
     sense (same knots). Points which are possibly on the  boundaries,
     or exterior to the region implied  by  the  four  boundaries  are 
     flagged (and should not be used  for  subsequent  fitting).  This
     routine creates a bilinearly blended Coons surface from the  four
     boundaries, and uses this surface to derive the parameters (using
     point projection to the Coons surface). A typical calling example 
     is:
 
       NL_POINT      *P;
       NL_CURVE      **bdys;
       NL_PARAMETER  *uu, *vv;
       NL_INDEX      np, nb, *bidx, *bcas;
       NL_STACKS     SG;
       ...
       (define array P and the boundary curves, and allocate arrays
        uu and vv);
       ...
       N_FitCalcSrfParamsBoundary(P,np,bdys,uu,vv,&nb,&bidx,&bcas,&SG);
 
 
   ACCESS:
   
     P    , input  ,  Points requiring parameterization
     np   , input  ,  The high index of the arrays: P,uu,vv
     bdys , input  ,  bdys[i], 0<=i<=3, is a boundary curve of the surf-
                      ace ,  ordered:  i=0=NL_BOTTOM , i=1=NL_RIGHT , i=2=NL_TOP,
                      i=3=NL_LEFT. The curves must be nonrational  and  al-
                      ready be compatible in the b-spline sense. 
     uu   , output ,  The u-parameters of the points in P
     vv   , output ,  The v-parameters of the points in P
                      Note: these values will fall in the ranges implied
                      by the domains of the boundary curves. Both uu and
                      vv must be allocated in the calling routine
     nb   , output ,  If nb >= 0, nb is the high index  for  the  arrays
                      bidx and bflg
     bidx , output ,  If nb >= 0, then for i=0,...,nb, P[bidx[i]]  is  a
                      "bad" point in the sense that either: 
                      (1) the point appears not to be  interior  to  the
                          surface region implied by the boundaries, or
                      (2) u,v parameters could  not  be  determined  for 
                          this point (no convergence).
                      In either of these cases, the point should not  be
                      used in the surface fitting process.  This  memory 
                      is allocated in this routine and should be deallo-
                      cated in the calling routine
     bcas , output ,  If nb >= 0, then for i=0,...,nb, bcas[i] indicates
                      which "bad" case (1 or 2, as defined above).  This  
                      memory is allocated in this routine and should  be 
                      deallocated in the calling routine
     SG   , input  ,  Memory stack for bidx and bcas
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitCalcSrfParamsBoundary 
  (NL_POINT     * P,    /* in : cloud-of-points, sized:[np+1] */ 
   NL_INDEX       np,   /* in : highest uu, vv, and P index value */ 
   NL_CURVE    ** bdys, /* in : compatible boundary curves, 0=Bottom, 1=Right, 2=Top, 3=Left */ 
   NL_PARAMETER * uu,   /* out: U params, sized:[np+1] */
   NL_PARAMETER * vv,   /* out: V params, sized:[np+1] */
   NL_INDEX     * nb,   /* out: highest bidx and bcas index value or 0 */
   NL_INDEX    ** bidx, /* out: p[bidx[i]] is a bad point, either looks outside boundaries or no u,v could be found for point */
   NL_INDEX    ** bcas, /* out: case flag: bcas[i]: 1=pt looks outside boundary, 2=no uv value could be found for point */
   NL_STACKS    * SG )  /* in : bidx and bcas memory stack */ 
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcSrfParamsBoundary");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, nu, nv, nbl, *bidxl = NULL, *bcasl = NULL, bad = 0, size = 0;

    NL_INTEGER sav_itlim = 0;

    NL_REAL *U, *V, us, ue, vs, ve, d1, d2, d3, *ug, *vg, ueps, veps, top, toc, u1, u2 = 0.0, v1, v2 = 0.0;

    NL_CURVE ** curU, ** curV;

    NL_SURFACE sur;

    NL_POINT ** gpts, ** SD, Q1, Q2;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for errors */

    for ( ii = 0; ii <= 3; ii++ )
        if( N_IsCrvRat( bdys[ii] ) )
            NL_ERROR( NL_INP_ERR );

    N_CrvGetKnots( bdys[0], &jj, &U );
    N_CrvGetKnots( bdys[2], &kk, &V );

    if( jj NEQ kk )
        NL_ERROR( NL_INP_ERR );

    for ( ii = 0; ii <= jj; ii++ )
        if( U[ii]NEQ V[ii] )
            NL_ERROR( NL_INP_ERR );

    N_CrvGetKnots( bdys[1], &jj, &U );
    N_CrvGetKnots( bdys[3], &kk, &V );

    if( jj NEQ kk )
        NL_ERROR( NL_INP_ERR );

    for ( ii = 0; ii <= jj; ii++ )
        if( U[ii]NEQ V[ii] )
            NL_ERROR( NL_INP_ERR );

    /* Construct the bilinearly blended Coons surface */

    curU = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

    if( curU EQ NULL )
        NL_QUIT;
    error = N_CrvCopy( bdys[0], curU[0], &SL );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvCopy( bdys[2], curU[1], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    curV = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

    if( curV EQ NULL )
        NL_QUIT;
    error = N_CrvCopy( bdys[3], curV[0], &SL );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvCopy( bdys[1], curV[1], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &sur );
    error = N_CreateCoonsSrf( curU, curV, &sur, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_OUT;

    /* Reparameterize the surface if necessary */

    N_CrvGetParamBounds( bdys[0], &us, &ue );
    N_CrvGetParamBounds( bdys[1], &vs, &ve );

    N_SrfGetKnots( &sur, &jj, &kk, &U, &V );

    if( us NEQ U[0]OR ue NEQ U[jj]OR vs NEQ V[0]OR ve NEQ V[kk] )
        N_SrfReparam( &sur, us, ue, vs, ve );

    /* A grid of points will be evaluated on the surface and used */
    /* to search for start points for the Newton iterations.      */

    error = N_CrvArcLength( bdys[0], us, ue, 0.01, NL_RELATIVE, &d2 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvArcLength( bdys[2], us, ue, 0.01, NL_RELATIVE, &d3 );

    if( error EQ NL_YES )
        NL_OUT;
    d1 = 0.5 *( d2 + d3 );

    error = N_CrvArcLength( bdys[1], vs, ve, 0.01, NL_RELATIVE, &d2 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvArcLength( bdys[3], vs, ve, 0.01, NL_RELATIVE, &d3 );

    if( error EQ NL_YES )
        NL_OUT;
    d2 = 0.5 *( d2 + d3 );

    nu = 40; /* (nu+1)*(nv+1) grid points */
    nv = 40;

    if( d1 / d2 GT 20.0 )
        nu = 60;

    else if( d2 / d1 GT 20.0 )
        nv = 60;

    /* Get the grid points */

    ug = N_AllocReal1dArray( nu + nv + 2, &SL );

    if( ug EQ NULL )
        NL_QUIT;
    vg = &ug[nu + 1];
    gpts = N_AllocPt2dArray( nu, nv, &SL );

    if( gpts EQ NULL )
        NL_QUIT;

    d1 = (ue - us) / nu;

    for ( ii = 0; ii < nu; ii++ )
        ug[ii] = us + ii * d1;
    ug[nu] = ue;

    d1 = (ve - vs) / nv;

    for ( ii = 0; ii < nv; ii++ )
        vg[ii] = vs + ii * d1;
    vg[nv] = ve;

    error = N_SrfEvalPtGrid( &sur, ug, vg, nu, nv, NL_LEFT, NL_LEFT, gpts );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now zip through all the points in P, project each one to */
    /* the Coons surface (twice for reliability), and use the   */
    /* result for the (u,v) of the point (unless bad).          */

    nbl = -1;
    *bidx = NULL;
    *bcas = NULL;
    ueps = 0.000001 *( ue - us ); /* stay this far away from bndy */
    veps = 0.000001 *( ve - vs );

    sav_itlim = NL_ITLIM;         /* don't use so many iterations */
    NL_ITLIM = 20;
    top = 2.0 *NL_MTOL;
    toc = 0.00001;

    for ( ii = 0; ii <= np; ii++ )
    {
        d1 = d2 = NL_BIGD;
        u1 = v1 = 0.0;

        for ( jj = 0; jj <= nu; jj++ )
            for ( kk = 0; kk <= nv; kk++ )
            {
                N_DistSqPtPt( P[ii], gpts[jj][kk], &d3 );

                if( d3 LT d2 )
                {
                    if( d3 GE d1 )
                    {
                        u2 = ug[jj];
                        v2 = vg[kk];
                        d2 = d3;
                    }
                    else
                    {
                        u2 = u1;
                        v2 = v1;
                        d2 = d1;
                        u1 = ug[jj];
                        v1 = vg[kk];
                        d1 = d3;
                    }
                }
            }

        error = N_GetClosestPtOnSrf( &sur, P[ii], u1, v1, top, toc, &u1, &v1, &Q1, SD );

        if( error EQ NL_YES )
            d1 = -1.0;
        else
            N_DistSqPtPt( P[ii], Q1, &d1 );

        error = N_GetClosestPtOnSrf( &sur, P[ii], u2, v2, top, toc, &u2, &v2, &Q2, SD );

        if( error EQ NL_YES )
            d2 = -1.0;
        else
            N_DistSqPtPt( P[ii], Q2, &d2 );

        if( d1 GE 0.0 AND( d2 LT 0.0 OR d1 LE d2 ) )
        {
            uu[ii] = u1;
            vv[ii] = v1;

            if( u1 - us GT ueps AND ue - u1 GT ueps AND v1 - vs GT veps AND ve - v1 GT veps )
                continue;
            bad = 1;
        }

        if( d2 GE 0.0 AND( d1 LT 0.0 OR d2 LT d1 ) )
        {
            uu[ii] = u2;
            vv[ii] = v2;

            if( u2 - us GT ueps AND ue - u2 GT ueps AND v2 - vs GT veps AND ve - v2 GT veps )
                continue;
            bad = 1;
        }

        if( d1 LT 0.0 AND d2 LT 0.0 )
            bad = 2;

        /* bad case */

        if( nbl EQ - 1 )
        {
            bidxl = N_AllocInt1dArray( 20, SG );

            if( bidxl EQ NULL )
                NL_QUIT;
            bcasl = N_AllocInt1dArray( 20, SG );

            if( bcasl EQ NULL )
                NL_QUIT;
            size = 20;
        }
        else if( nbl GE size )
        {
            error = N_Realloc1dIntArray( &bidxl, size, size + 20, SG );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_Realloc1dIntArray( &bcasl, size, size + 20, SG );

            if( error EQ NL_YES )
                NL_OUT;
            size += 20;
        }

        nbl += 1;
        bidxl[nbl] = ii;
        bcasl[nbl] = bad;

        if( bad EQ 2 )
        {
            uu[ii] = us;
            vv[ii] = vs; /* must put something in these */
        }
    }

    /* compact the "bad" arrays and set the pointers */

    if( nbl GE 0 )
    {
        error = N_Realloc1dIntArray( &bidxl, size, nbl, SG );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_Realloc1dIntArray( &bcasl, size, nbl, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    *nb = nbl;
    *bidx = bidxl;
    *bcas = bcasl;

    /* End NURBS and Exit */

    EXIT:

    NL_ITLIM = sav_itlim; /* reset NL_ITLIM */

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FitCalcSrfParamsBoundarySRF: Parameters for surface fitting of random data            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine  computes the u,v parameters for fitting an
     approximation  surface to a set of random (non-NxM) points.  This
     routine  requires all four boundary curves of the surface;  these
     curves must be  nonrational and must be defined on the same para-
     meter  range. 
     
     The routine uses the boundary curves and a percentage of the P 
     sample points to build a Bilinear Coons, Bicubic Coons, or 
     Biquadratic base surface. The P points are then projected to the          
     base surface to get u,v guesses for each point which can be used  
     by fitting routines for connecting the points P to an 
     approximation surface.                                               

       NL_CURVE    **curU, **curV;
       NL_SURFACE  *surB;
       NL_POINT    *P, *T;
       NL_REAL     *u, *v, tol, per;
       NL_INDEX    n, m, nu, nv;
       NL_STACKS   SG;
       ...
       (get boundaries, P, tol, per, nu, nv and surB);
       ...
       N_FitCalcSrfParamsBoundarySrf(P,n,curU,curV,NL_YES,BILINEAR,tol,per,nu,-1,NULL,
                &T,&u,&v,&m,&SG);

     MEMORY FOR T, u  AND  v  IS ALLOCATED INSIDE ROUTINE! NOTE ON THE 
     BASE SURFACES:
      (1) The bilinearly  blended  Coons patch  gives good results for 
          simple patches. It is stable and reliable, however, tends to
          miss details inherent in the data.
      (2) The biquadratic surface gives good results in general,  how-
          ever,  it is the  slowest of the three  base  surfaces.  Its
          quality heavily depends on the complexity of the data and of
          the selection of the subset of points.
      (3) The bicubically blended base surface is fast  to compute and 
          is able to capture  internal  details, however, it  tends to
          wrinkle the surface especially if the  data represents a bit
          more complex patch.
     RECOMMENDATION: FOR VERY SIMPLE  PATCHES USE THE  BILINEAR COONS. 
     IF INTERNAL  DETAILS ARE TO  BE CAPTURED,  USE THE  BICUBIC COONS
     OPTION.  IF IT WIGGLES TOO MUCH,  USE A NL_BIQUADRATIC NL_SURFACE  WITH
     LOTS OF NL_POINTS, I.E. HIGH per RATE.


   ACCESS:
   
     P    , input  ,  Points to be approximated
     n    , input  ,  Highest index in array P
     curU , input  ,  U-boundaries defined on the same parameter span:
                        curU[0] is at v=vmin
                        curU[1] is at v=vmax
     curV , input  ,  V-boundaries defined on the same parameter span:
                        curV[0] is at u=umin
                        curV[1] is at u=umax    
     thf  , input  ,  Flag:
                        NL_YES: thin  data set;  tol is for decomposition
                             for thinning and for projection
                        NL_NO : do not thin data set; tol is for decompo-
                             sition for projection
     bsf  , input  ,  Flag:
                        NL_BILINEARCOONS:  base  surface is a  bilinearly 
                                        blended Coons patch
                        NL_BIQUADRATIC:    base surface is  a biquadratic
                                        surface patch
                        NL_BICUBICCOONS:   base  surface is a bicubically 
                                        blended Coons patch
                      THIS NL_FLAG IS NL_USED ONLY IF surB = NULL
     tol  , input  ,  Surface  flatness tolerance;  MUST BE A NL_RELATIVE 
                      TOLERANCE!! 1%, I.E. 0.01 IS SUGGESTED.
     per  , input  ,  Not used - modify thf value.  Percentage of points  selected to  form the base 
                      surface. Although it may  change from surface to
                      surface, a good  initial guess is  10 points per
                      unit surface area
     nu,nv, input  ,  Highest indexes of approximating base surfaces:
                        >=0 index passed in
                        < 0 compute indexes internally
     surB , input  ,  Base surface:
                        != NULL: passed in
                         = NULL: computed internaly
     T    , output ,  Thinned points or subset of points P successfully 
                      projected onto the base surface
     u,v  , output ,  Parameters  corresponding to T[0..m]
     m    , output ,  Highest index in T[0..m], u[0..m], v[0..m]
     SG   , input  ,  T's, u's and v's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitCalcSrfParamsBoundarySrf
  (NL_POINT    * P,        /* in : target points, sized:[n+1] */
   NL_INDEX      n,        /* in : highest index in P */
   NL_CURVE   ** curU,     /* in : compatible U boundaries, curU[0] = vMin, curU[1] = vMax */
   NL_CURVE   ** curV,     /* in : compatible V boundaries, curV[0] = uMin, curV[1] = uMax */
   NL_FLAG       thf,      /* in : oneof NL_YES= thin data set; tol is for deomposition, thinning, and projection */
                           /*            NL_NO = don't thin data; tol is for decomposition and projection */
   NL_FLAG       bsf,      /* in : oneof NL_BILINEARCOONS=  base surface is a bilinearly blended Coons patch  */
                           /*            NL_BIQUADRATIC  =  base surface is a biquadratic surface patch       */
                           /*            NL_BICUBICCOONS =  base surface is a bicubically blended Coons patch */
   NL_REAL       tol,      /* in : Surface flatness tolerance;  MUST BE A RELATIVE TOLERANCE!! */  
                           /*            1%, I.E. 0.01 IS SUGGESTED.                              */                   
   NL_REAL       per,      /* in : not used - percentage of points selected to form base surface - suggest 10% = 0.10 */                    
   NL_INDEX      nu,       /* in : highest ControlPoint U index in base surface - neg. number = compute internally */                         
   NL_INDEX      nv,       /* in : highest ControlPoint V index in base surface - neg. number = compute internally */                                                           
   NL_SURFACE  * surB,     /* in : Opt Base Surface, NULL to ignore */            
   NL_POINT   ** T,        /* out: Thinned points of P successfully projected onto base surface, sized:[m+1] */                                                   
   NL_REAL    ** u,        /* out: U parameters for points in T, sized:[m+1] */               
   NL_REAL    ** v,        /* out: V parameters for points in T, sized:[m+1] */                                              
   NL_INDEX    * m,        /* out: size parameter, number of vals in T, u, and v arrays */
   NL_STACKS   * SG )      /* in : output memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCalcSrfParamsBoundarySRF");

    NL_FLAG *vst, error = NL_NO;

    NL_INDEX k, nb, mb = 0, nt, mt, n0, n1, mc, r, s, ind, rn;

    NL_DEGREE ps, qs;

    NL_REAL *U, *V, *ub = NULL, *vb = NULL, inv, fac, l0, l1, lu, lv, mag;

    NL_REAL trl = 0.005;

    NL_POINT *B = NULL, *R;

    NL_CURVE ** crlU, ** crlV, ** derU, ** derV;

    NL_SURFACE *sur;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if base surface is passed in */

    if( surB NEQ NULL )
    {
        sur = surB;

        goto COMPUTE_PARAMETERS;
    }

    /* Get local curves */

    crlU = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

    if( crlU EQ NULL )
        NL_QUIT;

    crlV = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

    if( crlV EQ NULL )
        NL_QUIT;

    error = N_CrvCopy( curU[0], crlU[0], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvCopy( curU[1], crlU[1], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvCopy( curV[0], crlV[0], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvCopy( curV[1], crlV[1], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get base surface */

    sur = N_AllocSrf( &SL );

    if( sur EQ NULL )
        NL_QUIT;

    N_SrfInitArrays( sur );
    error = N_CreateCoonsSrf( crlU, crlV, sur, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetDegrees( sur, &ps, &qs );

    if( bsf NEQ NL_BILINEARCOONS )
    {
        /* Estimate highest indexes */

        if( nu LT 0 )
        {
            N_CrvGetArraySizes( crlU[0], &n0, &mc );
            N_CrvGetArraySizes( crlU[1], &n1, &mc );

            nu = NL_MAX( n0, n1 );
        }

        if( nv LT 0 )
        {
            N_CrvGetArraySizes( crlV[0], &n0, &mc );
            N_CrvGetArraySizes( crlV[1], &n1, &mc );

            nv = NL_MAX( n0, n1 );
        }

        /* Select points randomly */

        nb = (NL_INDEX)( n * per );
        inv = 1.0 / ((NL_REAL)RAND_MAX);

        vst = N_AllocFlag1dArray( n, &SL );

        if( vst EQ NULL )
            NL_QUIT;

        R = N_AllocPt1dArray( nb, &SL );

        if( R EQ NULL )
            NL_QUIT;

        for ( k = 0; k <= n; k++ )
            vst[k] = NL_NO;

        k = -1;

        while( k LT nb )
        {
            rn = N_GenerateRandomNumber();
            fac = inv * ((NL_REAL)rn);
            ind = (NL_INDEX)( n * fac );

            if( vst[ind]EQ NL_NO )
            {
                N_VectorCopy( P[ind], &R[++k] );
                vst[ind] = NL_YES;
            }
        }

        /* Get parameters of random points */

        error = N_SrfProjectPts( sur, R, nb, NL_NO, NL_YES, trl, &B, &ub, &vb, &mb, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* If case of tensor product, add points along the boundary */

        if( bsf EQ NL_BIQUADRATIC )
        {
            nt = ps * nu;
            mt = qs * nv;
            mc = mb + 2 * nt + 2 * mt + 4;

            error = N_Realloc1dPtArray( &B, mb, mc, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &ub, mb, mc, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &vb, mb, mc, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetKnots( crlU[0], &r, &U );
            N_CrvGetKnots( crlV[0], &s, &V );

            error = N_CrvEvalEvenSpacedPts( crlU[0], U[0], U[r], nt, 0.1, &B[mb + 1], &ub[mb + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = mb + 1; k <= mb + nt + 1; k++ )
                vb[k] = V[0];

            mb += nt + 1;

            error = N_CrvEvalEvenSpacedPts( crlU[1], U[0], U[r], nt, 0.1, &B[mb + 1], &ub[mb + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = mb + 1; k <= mb + nt + 1; k++ )
                vb[k] = V[s];

            mb += nt + 1;

            error = N_CrvEvalEvenSpacedPts( crlV[0], V[0], V[s], mt, 0.1, &B[mb + 1], &vb[mb + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = mb + 1; k <= mb + mt + 1; k++ )
                ub[k] = U[0];

            mb += mt + 1;

            error = N_CrvEvalEvenSpacedPts( crlV[1], V[0], V[s], mt, 0.1, &B[mb + 1], &vb[mb + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = mb + 1; k <= mb + mt + 1; k++ )
                ub[k] = U[r];

            mb += mt + 1;
        }
    }

    if( bsf EQ NL_BICUBICCOONS )
    {
        if( n LE( 2 * (nu + nv) ) )
            NL_ERROR( NL_INP_ERR );

        derU = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

        if( derU EQ NULL )
            NL_QUIT;

        derV = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

        if( derV EQ NULL )
            NL_QUIT;

        error = N_Create4CrossBoundaryDerivCrvs( crlU, crlV, B, mb, ub, vb, ps, qs, nu, nv, NULL, NULL, derU, derV, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetKnots( crlU[0], &r, &U );
        N_CrvGetKnots( crlV[0], &s, &V );

        error = N_CrvArcLength( crlV[0], V[0], V[s], 0.05, NL_RELATIVE, &l0 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvArcLength( crlV[1], V[0], V[s], 0.05, NL_RELATIVE, &l1 );

        if( error EQ NL_YES )
            NL_OUT;

        lv = 0.5 *( l0 + l1 );

        error = N_CrvArcLength( crlU[0], U[0], U[r], 0.05, NL_RELATIVE, &l0 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvArcLength( crlU[1], U[0], U[r], 0.05, NL_RELATIVE, &l1 );

        if( error EQ NL_YES )
            NL_OUT;

        lu = 0.5 *( l0 + l1 );
        l0 = lu / (lu + lv);
        l1 = lv / (lu + lv);

        error = N_CrvGetAveragePosMag( derU[0], &mag );

        if( error EQ NL_YES )
            NL_OUT;

        fac = (l1 * lv) / mag;
        N_ConstantMultiplyCrv( fac, derU[0] );

        error = N_CrvGetAveragePosMag( derU[1], &mag );

        if( error EQ NL_YES )
            NL_OUT;

        fac = (l1 * lv) / mag;
        N_ConstantMultiplyCrv( fac, derU[1] );

        error = N_CrvGetAveragePosMag( derV[0], &mag );

        if( error EQ NL_YES )
            NL_OUT;

        fac = (l0 * lu) / mag;
        N_ConstantMultiplyCrv( fac, derV[0] );

        error = N_CrvGetAveragePosMag( derV[1], &mag );

        if( error EQ NL_YES )
            NL_OUT;

        fac = (l0 * lu) / mag;
        N_ConstantMultiplyCrv( fac, derV[1] );

        N_SrfInitArrays( sur );
        error = N_CreateCoonsBoundaryCrvs( crlV[0], derV[0], crlV[1], derV[1], crlU[0], derU[0], crlU[1], derU[1], NL_NO, sur, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( bsf EQ NL_BIQUADRATIC )
    {
        N_SrfInitArrays( sur );
        error = N_FitSrfLstSqBoundary( B, NULL, ub, vb, mb, NULL, NULL, nu, nv, 2, 2, NULL, NULL, NL_FULL, NL_LUPIV, NL_YES, NULL, sur, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get parameters */

    COMPUTE_PARAMETERS:

    error = N_SrfProjectPts( sur, P, n, thf, NL_YES, tol, T, u, v, m, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FitSrfToPtsAndBoundary: Surface approximation with boundary curves specified     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes a NURBS surface approximating a given
     set of nxm points. In addition to the points, the 4 boundary curves
     are specified,  and they form the precise boundaries of the surface
     (no approximation on the boundaries).  Each pair of opposite bound-
     aries must be compatible in the b-spline sense on input  (same deg-
     ree and knots),  and their degree must be greater than 1.  Further-
     more, the boundary curves must be nonrational.  Optionally, the u,v
     parameters may also be input.  One of two fitting  methods  may  be
     chosen:

       1) Number of control points to be used  is  input.  The  boundary
          curves' knots will be  used  where  feasible,  but  additional 
          knots may be added as required. That is, the final surface may
          contain knots not present in the boundary curves,  even in the
          case that the number of surface control points is smaller than 
          the number of boundary control points.
       2) The knots and number of control points to use for the  fitting
          operation will be taken from the boundary curves, hence, the
          final surface will not contain any knots not already contained
          in the original boundary curves.

       Note that in both methods,  the  boundaries  of the final surface
       will be geometrically and parametrically precisely equal  to  the
       input boundary curves.  Additionally, with Method 2, the b-spline
       structure of the boundaries will be maintained. With both methods
       the degree is inherited from the boundaries.

     The output surface must  be initialized to the NULL surface. A typ-
     ical calling example is as follows:
 
       NL_POINT    **P;
       NL_INDEX    mu, mv, meth, n, m;
       NL_CURVE    **curU, **curV;
       NL_REAL     *up, *vp;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get points P,  boundary curves,  and the  parameters (optional),
        and choose the method);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfToPtsAndBoundary(P,mu,mv,up,vp,curU,curV,1,n,m,&sur,&SG);
 

 
   ACCESS:
   
     P     , input  ,  Point array  to  be  approximated.  Must  include
                       boundary points,  e.g.  P[i][0] is the i-th point
                       on the curU[0] boundary: P[i][0] = curU[0](up[i])
     mu,mv , input  ,  Highest  indexes  in  P,  P[i][j],  0<=i<=mu  and 
                       0<=j<=mv.  Note that mu > degree(curU[0]) > 1 and
                       mv > degree(curV[0]) > 1 must hold
     up,vp , input  ,  Optional surface parameters corresponding  to the 
                       P[i][j]:
                       != NULL : parameters are input
                       == NULL : parameters are computed in this routine
                       Note: both up and vp must be either NULL  or  not
                             NULL.  For best results, if they are input,
                             they  should  not  force a parameterization
                             that is greatly different than that implied
                             by the given boundary curves
     curU  , input  ,  The u-boundary curves (curU[0] is boundary at
                       v=vmin and curU[1] at v=vmax). Must be compatible
                       in the b-spline sense, and must be nonrational.
     curV  , input  ,  The v-boundary curves (curV[0] is boundary at
                       u=umin and curV[1] at u=umax). Must be compatible
                       in the b-spline sense, and must be nonrational.
     meth  , input  ,  Method to be used:
                         = 1 : use Method 1 (new knots allowed)
                         = 2 : use Method 2 (no new knots)
     n,m   , input  ,  Number of control points for the surface fit (not
                       used if meth == 2).   p <= n < mu and q <= m < mv 
                       must hold, where p and q are the degrees  of  the
                       curU and curV curves, respectively
     sur   , output ,  Approximating surface
     SG    , input  ,  memory stack for sur 
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfToPtsAndBoundary 
  (NL_POINT   ** P,    /* in : grid-of-points, sized[mu+1][mv+1] */      
   NL_INDEX      mu,   /* in : highest 1st index in P            */      
   NL_INDEX      mv,   /* in : highest 2nd index in P            */      
   NL_REAL     * up,   /* in : opt u parameters of P points, sized:[mu+1], NULL to compute internally */      
   NL_REAL     * vp,   /* in : opt v parameters of P points, sized:[mv+1], NULL to compute internally */      
   NL_CURVE   ** curU, /* in : u-boundary curves, curU[0] = v at vmin, curvU[1] = v at vmax */      
   NL_CURVE   ** curV, /* in : v-boundary curves, curV[0] = u at umin, curvV[1] = u at umax */      
   NL_INDEX      meth, /* in : 1=new knots allowed, 2=no new knots                          */      
   NL_INDEX      n,    /* in : when meth = 1, output U ControlPoint count, p <= n < mu      */      
   NL_INDEX      m,    /* in : when meth = 1, output V ControlPoint count, q <= m < mv      */      
   NL_SURFACE  * sur,  /* out: approximating surface */      
   NL_STACKS   * SG )  /* in : sur's memory stack    */      
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfToPtsAndBoundary");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, nn, mm, k1, k2, k3, mult1, mult2, ns, ms, jc, kp1, kp2, kn1, kn2, kn3, kn4, kpmin, count;

    NL_DEGREE p, q, pq1, pq2;

    NL_REAL *U, *V, *UU, *VV, *X, us, ue, vs, ve, *upars, *vpars, duv, uv, d1, top, toc, *UP, *UI, a, b, fac, per = 0.8, *VI, *VP;

    NL_CPOINT *Uw0, *Uw1, *Vw0, *Vw1, ** Pw;

    NL_POINT Q;

    NL_CURVE curves[4], ** temp_curs;

    NL_KNOTVECTOR *knu, *knv, knx, *kni, *knp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error */

    if( (up EQ NULL AND vp NEQ NULL)OR( vp EQ NULL AND up NEQ NULL ) )
        NL_ERROR( NL_INP_ERR );

    if( meth EQ 1 AND( n GE mu OR m GE mv ) )
        NL_ERROR( NL_INP_ERR );

    N_CrvGetCPtsDegreeAndKnots( curU[0], &ii, &Uw0, &pq1, &jj, &UU );
    us = UU[0];
    ue = UU[jj];
    N_CrvGetCPtsDegreeAndKnots( curU[1], &jj, &Uw1, &pq2, &kk, &UU );

    if( ii NEQ jj OR pq1 NEQ pq2 )
        NL_ERROR( NL_INP_ERR );

    if( us NEQ UU[0]OR ue NEQ UU[kk] )
        NL_ERROR( NL_INP_ERR );

    if( mu LE pq1 OR pq1 EQ 1 )
        NL_ERROR( NL_INP_ERR );

    nn = ii;
    p = pq1;

    N_CrvGetCPtsDegreeAndKnots( curV[0], &ii, &Vw0, &pq1, &jj, &VV );
    vs = VV[0];
    ve = VV[jj];
    N_CrvGetCPtsDegreeAndKnots( curV[1], &jj, &Vw1, &pq2, &kk, &VV );

    if( ii NEQ jj OR pq1 NEQ pq2 )
        NL_ERROR( NL_INP_ERR );

    if( vs NEQ VV[0]OR ve NEQ VV[kk] )
        NL_ERROR( NL_INP_ERR );

    if( mv LE pq1 OR pq1 EQ 1 )
        NL_ERROR( NL_INP_ERR );

    mm = ii;
    q = pq1;

    if( meth EQ 1 AND( n LT p OR m LT q ) )
        NL_ERROR( NL_INP_ERR );

    /* Compute the parameters if necessary */

    if( up NEQ NULL )
    {
        upars = up;
        vpars = vp;
    }
    else
    {
        top = 10.0 *NL_MTOL;
        toc = 1.e-5;

        upars = N_AllocReal1dArray( mu + mv + 2, &SL );

        if( upars EQ NULL )
            NL_QUIT;
        vpars = &upars[mu + 1];

        /* use the parameterization implied by the boundaries */

        upars[0] = us;
        upars[mu] = ue;

        for ( ii = 1; ii < mu; ii++ )
        {
            k1 = 0;
            upars[ii] = 0.0;
            duv = (ue - upars[ii - 1]) / ((NL_REAL)mu - (NL_REAL)ii + 1.0);

            for ( jj = 0; jj <= 1; jj++ )
            {
                uv = upars[ii - 1] + duv;

                for ( kk = 1; kk <= 3; kk++ )
                {
                    error = N_CrvClosestPt( curU[jj], P[ii][jj * mv], uv, top, toc, &d1, &Q );

                    if( error EQ NL_NO )
                    {
                        upars[ii] += d1;
                        k1 += 1;
                        break;
                    }
                    else
                    {
                        if( kk EQ 1 )
                            uv = upars[ii - 1] + 0.67 *duv;

                        else if( kk EQ 2 )
                            uv = upars[ii - 1] + 1.33 *duv;
                    }
                }
            }

            if( k1 EQ 0 )
            {
                NL_ERROR( NL_NUM_ERR );
            }
            else
                upars[ii] = upars[ii] / k1;
        }

        vpars[0] = vs;
        vpars[mv] = ve;

        for ( ii = 1; ii < mv; ii++ )
        {
            k1 = 0;
            vpars[ii] = 0.0;
            duv = (ve - vpars[ii - 1]) / ((NL_REAL)mv - (NL_REAL)ii + 1.0);

            for ( jj = 0; jj <= 1; jj++ )
            {
                uv = vpars[ii - 1] + duv;

                for ( kk = 1; kk <= 3; kk++ )
                {
                    error = N_CrvClosestPt( curV[jj], P[jj * mu][ii], uv, top, toc, &d1, &Q );

                    if( error EQ NL_NO )
                    {
                        vpars[ii] += d1;
                        k1 += 1;
                        break;
                    }
                    else
                    {
                        if( kk EQ 1 )
                            uv = vpars[ii - 1] + 0.67 *duv;

                        else if( kk EQ 2 )
                            uv = vpars[ii - 1] + 1.33 *duv;
                    }
                }
            }

            if( k1 EQ 0 )
            {
                NL_ERROR( NL_NUM_ERR );
            }
            else
                vpars[ii] = vpars[ii] / k1;
        }
    }

    /* Now determine the knots and number of control points to */
    /* use for surface fitting. This depends on meth.          */

    if( meth EQ 1 )
    {
        ns = n;
        ms = m;
    }
    else
    {
        ns = NL_MIN( nn, mu - 1 );
        ms = NL_MIN( mm, mv - 1 );
    }

    ii = NL_MAX( ns + p + 1, ms + q + 1 );

    kni = N_AllocKnotVectorAndArray( ii, &SL );

    if( kni EQ NULL )
        NL_QUIT;

    knp = N_AllocKnotVectorAndArray( ii, &SL );

    if( knp EQ NULL )
        NL_QUIT;

    /* u-knots first */

    knu = N_AllocKnotVectorAndArray( ns + p + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;
    N_KnotVectorGetKnots( knu, &ii, &U );

    /* Loop until knot vector done */

    while( 1 )
    {
        N_SetKnotIndex( knu, ns + p + 1 );

        N_SetKnotIndex( kni, ns + p + 1 );
        N_SetKnotIndex( knp, ns + p );

        N_KnotVectorGetKnots( knp, &ii, &UP );
        N_KnotVectorGetKnots( kni, &ii, &UI );

        for ( ii = 0; ii <= p; ii++ )
        {
            U[ii] = us;
            U[ns + ii + 1] = ue;
        }

        if( p EQ ns )
            break; /* u-knots completed */

        /* Get ideal knot vectors for degrees p and p-1 */

        error = N_FitCalcKnotVectorCrvApprox( upars, mu, ns, p, kni );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( upars, mu, ns, (NL_DEGREE)( p - 1 ), knp );

        if( error EQ NL_YES )
            NL_OUT;

        /* Select knots from UU */

        jj = p + 1;

        for ( ii = p + 1; ii <= ns; ii++ )
        {
            a = (1.0 - per) * UI[ii] + per * UP[ii - 1];
            b = (1.0 - per) * UI[ii] + per * UP[ii];

            while( UU[jj]LE a )
                jj += 1;

            fac = b - a;
            jc = -1;

            while( UU[jj]GT a AND UU[jj]LT b )
            {
                d1 = fabs( UI[ii] - UU[jj] );

                if( d1 LT fac )
                {
                    fac = d1;
                    jc = jj;
                }

                jj += 1;
            }

            /* now depends on method */

            if( meth EQ 1 )
            {
                if( jc EQ - 1 )
                    U[ii] = UI[ii];
                else
                    U[ii] = UU[jc];
            }
            else
            {
                if( jc EQ - 1 )
                {
                    ns -= 1;
                    ii -= 1;

                    if( ns LT p )
                        NL_ERROR( NL_NUM_ERR );
                    break;
                }
                else
                    U[ii] = UU[jc];
            }
        }

        if( ii GT ns )
            break; /* u-knots completed */
    }

    if( meth EQ 2 AND ns LT nn AND mu GT nn )
    {
        kpmin = 2 * (p + 1); /* try to use more knots in areas where */
        /* there are plenty of parameter values */
        kn1 = p;
        kn2 = p + 1;   /* [kn1,kn2] index range in U */

        kn3 = kn4 = p; /* [kn3,kn4] index range in UU */

        kp1 = kp2 = 0; /* [kp1,kp2] index range in upars */

        count = 0;

        while( 1 )
        {
            for ( ii = kn3; ii <= nn; ii++ )
                if( UU[ii]GT U[kn1] )
                    break;

            if( ii GT nn )
                goto NEXT_U;
            else
                kn3 = ii;

            while( UU[kn4]LT U[kn2] )
                kn4 += 1;

            kn4 -= 1;

            if( kn3 GT kn4 )
                goto NEXT_U;

            for ( ii = kp1; ii < mu; ii++ )
                if( upars[ii]GT U[kn1] )
                    break;

            if( ii GE mu )
                goto NEXT_U;
            else
                kp1 = ii;

            while( upars[kp2]LT U[kn2] )
                kp2 += 1;

            kp2 -= 1;

            if( kp2 - kp1 LE 2 * kpmin )
                goto NEXT_U;

            k1 = (kp1 + kp2) / 2;

            if( (kp2 - kp1) % 2 EQ 0 )
                a = upars[k1];
            else
                a = 0.5 *( upars[k1] + upars[k1 + 1] );

            k1 = kn3;
            d1 = fabs( a - UU[k1] ); /* find UU knot closest */
            /* to parm value a      */
            for ( ii = kn3 + 1; ii <= kn4; ii++ )
                if( fabs( a - UU[ii] )LT d1 )
                {
                    k1 = ii;
                    d1 = fabs( a - UU[ii] );
                }

            k2 = k3 = 0;

            for ( ii = kp1; ii <= kp2; ii++ )
                if( upars[ii]LT UU[k1] )
                    k2 += 1;
                else
                    k3 += 1;

            if( k2 GE kpmin AND k3 GE kpmin )
            {                    /* plenty of parm values to */
                k2 = ns + p + 1; /* either side              */

                while( U[k2]GT UU[k1] )
                {
                    U[k2 + 1] = U[k2];
                    k2 -= 1;
                }

                U[k2 + 1] = UU[k1];
                ns += 1;
                N_SetKnotIndex( knu, ns + p + 1 );

                count += 1;
                kn2 += 1;
            }

            NEXT_U:
            if( kn2 GT ns )
            {
                if( count EQ 0 )
                    break;
                else
                    count = 0;

                kn1 = p;
                kn2 = p + 1;
                kn3 = kn4 = p;
                kp1 = kp2 = 0;
            }
            else
            {
                kn1 = kn2;

                while( U[kn1]EQ U[kn1 + 1] )
                    kn1 += 1;
                kn2 = kn1 + 1;

                k1 = NL_MIN( kp1, kp2 );
                kp1 = kp2 = k1;
                k1 = NL_MIN( kn3, kn4 );
                kn3 = kn4 = k1;
            }
        }
    }

    /* now v-knots */

    knv = N_AllocKnotVectorAndArray( ms + q + 1, &SL );

    if( knv EQ NULL )
        NL_QUIT;
    N_KnotVectorGetKnots( knv, &ii, &V );

    /* Loop until knot vector done */

    while( 1 )
    {
        N_SetKnotIndex( knv, ms + q + 1 );

        N_SetKnotIndex( kni, ms + q + 1 );
        N_SetKnotIndex( knp, ms + q );

        N_KnotVectorGetKnots( knp, &ii, &VP );
        N_KnotVectorGetKnots( kni, &ii, &VI );

        for ( ii = 0; ii <= q; ii++ )
        {
            V[ii] = vs;
            V[ms + ii + 1] = ve;
        }

        if( q EQ ms )
            break; /* v-knots completed */

        /* Get ideal knot vectors for degrees q and q-1 */

        error = N_FitCalcKnotVectorCrvApprox( vpars, mv, ms, q, kni );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitCalcKnotVectorCrvApprox( vpars, mv, ms, (NL_DEGREE)( q - 1 ), knp );

        if( error EQ NL_YES )
            NL_OUT;

        /* Select knots from VV */

        jj = q + 1;

        for ( ii = q + 1; ii <= ms; ii++ )
        {
            a = (1.0 - per) * VI[ii] + per * VP[ii - 1];
            b = (1.0 - per) * VI[ii] + per * VP[ii];

            while( VV[jj]LE a )
                jj += 1;

            fac = b - a;
            jc = -1;

            while( VV[jj]GT a AND VV[jj]LT b )
            {
                d1 = fabs( VI[ii] - VV[jj] );

                if( d1 LT fac )
                {
                    fac = d1;
                    jc = jj;
                }

                jj += 1;
            }

            /* now depends on method */

            if( meth EQ 1 )
            {
                if( jc EQ - 1 )
                    V[ii] = VI[ii];
                else
                    V[ii] = VV[jc];
            }
            else
            {
                if( jc EQ - 1 )
                {
                    ms -= 1;
                    ii -= 1;

                    if( ms LT q )
                        NL_ERROR( NL_NUM_ERR );
                    break;
                }
                else
                    V[ii] = VV[jc];
            }
        }

        if( ii GT ms )
            break; /* v-knots completed */
    }

    if( meth EQ 2 AND ms LT mm AND mv GT mm )
    {
        kpmin = 2 * (q + 1); /* try to use more knots in areas where */
        /* there are plenty of parameter values */
        kn1 = q;
        kn2 = q + 1;   /* [kn1,kn2] index range in V */

        kn3 = kn4 = q; /* [kn3,kn4] index range in VV */

        kp1 = kp2 = 0; /* [kp1,kp2] index range in vpars */

        count = 0;

        while( 1 )
        {
            for ( ii = kn3; ii <= mm; ii++ )
                if( VV[ii]GT V[kn1] )
                    break;

            if( ii GT mm )
                goto NEXT_V;
            else
                kn3 = ii;

            while( VV[kn4]LT V[kn2] )
                kn4 += 1;

            kn4 -= 1;

            if( kn3 GT kn4 )
                goto NEXT_V;

            for ( ii = kp1; ii < mv; ii++ )
                if( vpars[ii]GT V[kn1] )
                    break;

            if( ii GE mv )
                goto NEXT_V;
            else
                kp1 = ii;

            while( vpars[kp2]LT V[kn2] )
                kp2 += 1;

            kp2 -= 1;

            if( kp2 - kp1 LE 2 * kpmin )
                goto NEXT_V;

            k1 = (kp1 + kp2) / 2;

            if( (kp2 - kp1) % 2 EQ 0 )
                a = vpars[k1];
            else
                a = 0.5 *( vpars[k1] + vpars[k1 + 1] );

            k1 = kn3;
            d1 = fabs( a - VV[k1] ); /* find VV knot closest */
            /* to parm value a      */
            for ( ii = kn3 + 1; ii <= kn4; ii++ )
                if( fabs( a - VV[ii] )LT d1 )
                {
                    k1 = ii;
                    d1 = fabs( a - VV[ii] );
                }

            k2 = k3 = 0;

            for ( ii = kp1; ii <= kp2; ii++ )
                if( vpars[ii]LT VV[k1] )
                    k2 += 1;
                else
                    k3 += 1;

            if( k2 GE kpmin AND k3 GE kpmin )
            {                    /* plenty of parm values to */
                k2 = ms + q + 1; /* either side              */

                while( V[k2]GT VV[k1] )
                {
                    V[k2 + 1] = V[k2];
                    k2 -= 1;
                }

                V[k2 + 1] = VV[k1];
                ms += 1;
                N_SetKnotIndex( knv, ms + q + 1 );

                count += 1;
                kn2 += 1;
            }

            NEXT_V:
            if( kn2 GT ms )
            {
                if( count EQ 0 )
                    break;
                else
                    count = 0;

                kn1 = q;
                kn2 = q + 1;
                kn3 = kn4 = q;
                kp1 = kp2 = 0;
            }
            else
            {
                kn1 = kn2;

                while( V[kn1]EQ V[kn1 + 1] )
                    kn1 += 1;
                kn2 = kn1 + 1;

                k1 = NL_MIN( kp1, kp2 );
                kp1 = kp2 = k1;
                k1 = NL_MIN( kn3, kn4 );
                kn3 = kn4 = k1;
            }
        }
    }

    /* Now do surface fit */

    error = N_FitSrfLstSqKnots( (NL_VOID ** )P, mu, mv, NL_EPOINT, upars, vpars, knu, knv, ns, ms, p, q, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Refine surface if necessary */

    ii = NL_MAX( nn, mm );
    jj = NL_MAX( ns, ms );
    kk = NL_MAX( ii, jj ) + NL_MAX( p, q );
    X = N_AllocReal1dArray( kk, &SL );

    if( X EQ NULL )
        NL_QUIT;

    k1 = -1;
    jj = p + 1;
    ii = p + 1;

    while( ii LE nn )
    {
        mult1 = 1;

        while( UU[ii]EQ UU[ii + mult1] )
            mult1 += 1;

        while( U[jj]LT UU[ii] )
            jj += 1;

        if( U[jj]EQ UU[ii] )
        {
            mult2 = 1;

            while( U[jj]EQ U[jj + mult2] )
                mult2 += 1;
        }
        else
        {
            mult2 = 0;
        }

        for ( kk = 1; kk <= mult1 - mult2; kk++ )
            X[++k1] = UU[ii];

        jj += mult2;
        ii += mult1;
    }

    if( k1 GE 0 )
    {
        N_KnotVectorFromRealArray( &knx, X, k1 );

        error = N_SrfInsertKnots( sur, &knx, NL_UDIR, sur, SG, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    k1 = -1;
    jj = q + 1;
    ii = q + 1;

    while( ii LE mm )
    {
        mult1 = 1;

        while( VV[ii]EQ VV[ii + mult1] )
            mult1 += 1;

        while( V[jj]LT VV[ii] )
            jj += 1;

        if( V[jj]EQ VV[ii] )
        {
            mult2 = 1;

            while( V[jj]EQ V[jj + mult2] )
                mult2 += 1;
        }
        else
        {
            mult2 = 0;
        }

        for ( kk = 1; kk <= mult1 - mult2; kk++ )
            X[++k1] = VV[ii];

        jj += mult2;
        ii += mult1;
    }

    if( k1 GE 0 )
    {
        N_KnotVectorFromRealArray( &knx, X, k1 );

        error = N_SrfInsertKnots( sur, &knx, NL_VDIR, sur, SG, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Refine boundary curves if necessary */

    temp_curs = N_AllocArrayCrvPtrs( 1, &SL );

    if( temp_curs EQ NULL )
        NL_QUIT;

    N_SrfGetKnots( sur, &ii, &jj, &U, &V );
    ns = ii - p - 1;
    ms = jj - q - 1;

    k1 = -1;
    jj = p + 1;
    ii = p + 1;

    while( ii LE ns )
    {
        mult1 = 1;

        while( U[ii]EQ U[ii + mult1] )
            mult1 += 1;

        while( UU[jj]LT U[ii] )
            jj += 1;

        if( UU[jj]EQ U[ii] )
        {
            mult2 = 1;

            while( UU[jj]EQ UU[jj + mult2] )
                mult2 += 1;
        }
        else
        {
            mult2 = 0;
        }

        for ( kk = 1; kk <= mult1 - mult2; kk++ )
            X[++k1] = U[ii];

        jj += mult2;
        ii += mult1;
    }

    if( k1 GE 0 )
    {
        N_KnotVectorFromRealArray( &knx, X, k1 );

        for ( ii = 0; ii <= 1; ii++ )
        {
            temp_curs[ii] = &curves[ii];
            N_CrvInitArrays( temp_curs[ii] );

            error = N_CrvRefine( curU[ii], &knx, temp_curs[ii], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        N_CrvGetCPts( temp_curs[0], &nn, &Uw0 );
        N_CrvGetCPts( temp_curs[1], &nn, &Uw1 );
    }

    k1 = -1;
    jj = q + 1;
    ii = q + 1;

    while( ii LE ms )
    {
        mult1 = 1;

        while( V[ii]EQ V[ii + mult1] )
            mult1 += 1;

        while( VV[jj]LT V[ii] )
            jj += 1;

        if( VV[jj]EQ V[ii] )
        {
            mult2 = 1;

            while( VV[jj]EQ VV[jj + mult2] )
                mult2 += 1;
        }
        else
        {
            mult2 = 0;
        }

        for ( kk = 1; kk <= mult1 - mult2; kk++ )
            X[++k1] = V[ii];

        jj += mult2;
        ii += mult1;
    }

    if( k1 GE 0 )
    {
        N_KnotVectorFromRealArray( &knx, X, k1 );

        for ( ii = 0; ii <= 1; ii++ )
        {
            temp_curs[ii] = &curves[ii + 2];
            N_CrvInitArrays( temp_curs[ii] );

            error = N_CrvRefine( curV[ii], &knx, temp_curs[ii], &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        N_CrvGetCPts( temp_curs[0], &mm, &Vw0 );
        N_CrvGetCPts( temp_curs[1], &mm, &Vw1 );
    }

    /* Replace outer control points with those of 4 boundary curves */

    N_SrfGetCPts( sur, &nn, &mm, &Pw );

    for ( ii = 0; ii <= nn; ii++ )
    {
        N_CPtToPtEuclid( Uw0[ii], &Q );
        N_PtToCPt( Q, &Pw[ii][0] );
        N_CPtToPtEuclid( Uw1[ii], &Q );
        N_PtToCPt( Q, &Pw[ii][mm] );
    }

    for ( ii = 0; ii <= mm; ii++ )
    {
        N_CPtToPtEuclid( Vw0[ii], &Q );
        N_PtToCPt( Q, &Pw[0][ii] );
        N_CPtToPtEuclid( Vw1[ii], &Q );
        N_PtToCPt( Q, &Pw[nn][ii] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/***********************************************************************/
/* N_FITSRFAPPROXTOL: Surface approximation with error bound specified */
/***********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes a NURBS surface approximating a given
     set of points, sampled on a regular grid, to a user given tolerance. 
     
     The  method starts with an
     interpolating surface of given degrees, and works its way up to the 
     required degrees by repeating the following steps:
       (1) remove as many knots as possible;
       (2) least-squares fit with a higher degree surface; and
       (3) adjust parameter values and error vector.
     If  one of  the above  steps fails, the  process is  restarted with 
     another  interpolating surface  of  one  degree  higher. A  typical 
     calling example is as follows:
 
       NL_POINT    **P;
       NL_INDEX    mu, mv;
       NL_DEGREE   ps, pr, qs,qr;
       NL_REAL     E;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get array P, and choose ps, pr, qs, qr and E);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfApproxTol(P,mu,mv,ps,pr,qs,qr,E,NL_SINGLE,NL_YES,&sur,&SG);
 
     The  algorithm can  use single or multiple  knots for data approxi-
     mation. The  single-knot  approximation  smooths  out  the   data,
     whereas  the  multiple-knot  approximant captures  local  phenomena 
     such as  flat spots  and cusps. It also  allows the  choice of knot 
     vectors for least-squares  approximation: (1) use  the knot  vector
     obtained from  knot removal, or (2)  let the routine  compute a new 
     knot vector that depends purely on the data points.
 
   ACCESS:
   
     P     , input  ,  Point array to be approximated
     mu,mv , input  ,  Highest indexes in P
     ps,qs , input  ,  Start iterations with  degrees (ps,qs) (if  cusps 
                       are to be computed, (ps,qs)=(1,1) is recommended. 
                       Otherwise  (ps,qs)=(2,2) is a good default)
     pr,qr , input  ,  Required degrees of approximating surface
     E     , input  ,  Error tolerance
     ktp   , input  ,  Flag:
                         NL_SINGLE  : use single knots for approximation
                         NL_MULTIPLE: use multiple knots for approximation
     atp   , input  ,  NL_FLAG:
                         NL_YES: use knot vector obtained from knot removal
                              for least-squares approximation
                         NL_NO : compute new knot vector for  least-squares
                              approximation
     sur   , output ,  Approximating surface
     SG    , input  ,  sur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfApproxTol
  (NL_POINT  ** P,     /* in : regular Point grid to be approximated, sized:[mu+1][mv+1] */
   NL_INDEX     mu,    /* in : highest 1st P index */
   NL_INDEX     mv,    /* in : highest 2nd P index */
   NL_DEGREE    ps,    /* in : iteration start output surface Degree U, typically 2, use 1 with cusps */
   NL_DEGREE    pr,    /* in : final output surface Degree U */
   NL_DEGREE    qs,    /* in : iteration start output surface Degree V, typically 2, use 1 with cusps */
   NL_DEGREE    qr,    /* in : final output surface Degree V */                                         
   NL_REAL      E,     /* in : error tolerance */
   NL_FLAG      ktp,   /* in : knot mult flag: NL_SINGLE  = use single knots for approximation   */
                       /*                      NL_MULTIPLE= use multiple knots for approximation */
   NL_FLAG      atp,   /* in : knot vector flag: NL_YES = do knot removal of last iteration surface */
                       /*                        NL_NO  = compute new knot vector by clumping sample point parameter values */
   NL_SURFACE  *sur,   /* out: Approximated surface */
   NL_STACKS   *SG )   /* in : sur memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfApproxTol");

    NL_FLAG apr, reset = NL_NO, error = NL_NO;

    NL_INDEX k, l, ns, rs, ms, ss, nh = 0, rh, mh = 0, sh, nsu, nsv, mlu, mlv;

    NL_DEGREE iu, ju, iv, jv, p, q;

    NL_REAL ** er, *U, *V, diag, ltop, ltoc;

    NL_PARAMETER *u, *us, *v, *vs;

    NL_POINT Q, ** SD;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL top = 1.0e-04;
    NL_PRIVATE NL_REAL toc = 1.0e-06;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Adjust tolerance */

    N_Pts2dGetMaxExtent( P, mu, mv, &diag );

    ltop = diag * top;
    ltoc = toc;

    /* Check for surface memory */

    if( pr GT mu OR qr GT mv )
        NL_ERROR( NL_INP_ERR );

    if( pr LT ps OR qr LT qs )
        NL_ERROR( NL_INP_ERR );

    if( N_SrfAreArraysNULL( sur ) )
        reset = NL_YES;

    error = N_SrfSizeArrays( sur, mu, mv, pr, qr, mu + pr + 1, mv + qr + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnotVectors( sur, &knu, &knv );
    N_SrfGetKnots( sur, &k, &l, &U, &V );

    /* Allocate memory and compute initial parameters */

    er = N_AllocReal2dArray( mu, mv, &SL );

    if( er EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    us = N_AllocReal1dArray( mu, &SL );

    if( us EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mv, &SL );

    if( v EQ NULL )
        NL_QUIT;

    vs = N_AllocReal1dArray( mv, &SL );

    if( vs EQ NULL )
        NL_QUIT;

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    /* assign each input point a uv parameter value assuming a regular grid of input points */
    error = N_FitCalcSrfParamValues( (NL_VOID ** )P, mu, mv, NL_EPOINT, NL_CHORDLENGTH, us, vs );

    if( error EQ NL_YES )
        NL_OUT;

    /***********************************************************/
    /*  Fit surface as follows:                                */
    /*    iu=ps; iv=qs;        iteration surface degrees       */
    /*    while( iu<=pr and iv<=qr ) do                        */
    /*       interpolate with degree (iu,iv)                   */
    /*       ju=iu; jv=iv;                                     */
    /*       while( ju<=pr and jv<=qr ) do                     */
    /*          remove knots                                   */
    /*          compute degree elevated surface's knot vector  */
    /*          least-squares approximate                      */
    /*          if approximation fails break                   */
    /*          adjust parameters and error vector             */
    /*          incement ju, jv                                */
    /*       increment iu, iv                                  */
    /***********************************************************/

    iu = ps;
    iv = qs;

    while( iu LE pr AND iv LE qr )
    {
        /* Interpolate with degree (iu,iv) */

        for ( k = 0; k <= mu; k++ )
        {
            u[k] = us[k];

            for ( l = 0; l <= mv; l++ )
                er[k][l] = 0.0;
        }

        for ( l = 0; l <= mv; l++ )
            v[l] = vs[l];

        /* build multiple end knot and single interior knot U and V knot vectors      */
        /* where interior knots are computed as averages of sample point param values */
        /* as U[i] = Avg(u[i],u[i+1]...u[i+p-1])                                      */
        N_FitCrvCalcKnotVector( u, mu, iu, knu );
        N_FitCrvCalcKnotVector( v, mv, iv, knv );
        N_SrfSetSizeIndices( sur, mu, mv, iu, iv, mu + iu + 1, mv + iv + 1 );
        
        /* interpolate points with one unique knot for each sample point - big surface for big sample sets */
        error = N_FitSrfToPtsKnots( (NL_VOID ** )P, mu, mv, NL_EPOINT, u, v, knu, knv, iu, iv, sur, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        ju = iu;
        jv = iv;

        while( ju LE pr AND jv LE qr )
        {
            /* Remove as many knots as possible */

            /* Remove as many knots and control points from sur as possible without     */
            /* increasing the surface shape change at any u,v param surface point by    */
            /* more than tolerance E.                                                   */
            /* After this call the number of knots and control points in sur is unknown */
            error = N_FitSrfApproxRemoveKnots( sur, u, v, er, mu, mv, E, NL_UVDIR );

            if( error EQ NL_YES )
                NL_OUT;

            if( ju EQ pr AND jv EQ qr )
                break;

            N_SrfGetArraySizes( sur, &ns, &ms, &rs, &ss );

            if( ju LT pr )
                p = ju + 1;
            else
                p = ju;

            if( jv LT qr )
                q = jv + 1;
            else
                q = jv;

            /* Get new knot vectors */

            if( atp EQ NL_YES )
            {
                if( ktp EQ NL_SINGLE )
                {
                    if( ju LT pr )
                    {
                        nh = ns + 1;
                        rh = rs + 2;

                        if( nh GT mu )
                            break;

                        for ( k = 0; k <= ju + 1; k++ )
                            U[rh--] = U[rs];

                        for ( k = ns; k >= ju + 1; k-- )
                            U[rh--] = U[k];

                        for ( k = 0; k <= ju + 1; k++ )
                            U[rh--] = U[0];
                    }

                    if( jv LT qr )
                    {
                        mh = ms + 1;
                        sh = ss + 2;

                        if( mh GT mv )
                            break;

                        for ( k = 0; k <= jv + 1; k++ )
                            V[sh--] = V[ss];

                        for ( k = ms; k >= jv + 1; k-- )
                            V[sh--] = V[k];

                        for ( k = 0; k <= jv + 1; k++ )
                            V[sh--] = V[0];
                    }
                } /* end ktp EQ NL_SINGLE branch */
                else /* ktp EQ NL_MULTIPLE branch */ 
                {
                    if( ju LT pr )
                    {
                        N_BasisGetSpanCount( knu, ju, &nsu );

                        nh = ns + nsu;
                        rh = rs + nsu + 1;

                        if( nh GT mu )
                            break;

                        while( rs GT 0 )
                        {
                            k = rs;

                            while( rs GT 0 AND U[rs]EQ U[rs - 1] )
                                rs--;
                            mlu = k - rs + 1;

                            for ( l = 1; l <= mlu + 1; l++ )
                                U[rh--] = U[rs];
                            rs--;
                        }
                    }

                    if( jv LT qr )
                    {
                        N_BasisGetSpanCount( knv, jv, &nsv );

                        mh = ms + nsv;
                        sh = ss + nsv + 1;

                        if( mh GT mv )
                            break;

                        while( ss GT 0 )
                        {
                            k = ss;

                            while( ss GT 0 AND V[ss]EQ V[ss - 1] )
                                ss--;
                            mlv = k - ss + 1;

                            for ( l = 1; l <= mlv + 1; l++ )
                                V[sh--] = V[ss];
                            ss--;
                        }
                    }
                } /* ktp EQ NL_MULTIPLE branch */
            }  /* end atp EQ NL_YES - get new knot vectors through knot removal */
            else /* atp EQ NL_NO */
                 /* compute specified number of knots knot vector by clumping sample point parameter values */
            {
                nh = NL_MIN( ns + 1, mu );
                mh = NL_MIN( ms + 1, mv );

                /* compute nh+p+1 sized knot vector from mu+1 u values */
                /*   makes fewer knots than parameter values by clumping parameter values */
                error = N_FitCalcKnotVectorCrvApprox( u, mu, nh, p, knu );

                if( error EQ NL_YES )
                    NL_OUT;

                /* compute mh+q+1 sized knot vector from mv+1 u values */
                /*   makes fewer knots than parameter values by clumping parameter values */
                error = N_FitCalcKnotVectorCrvApprox( v, mv, mh, q, knv );

                if( error EQ NL_YES )
                    NL_OUT;
            } /* end computing new knot vectors directly from sample point parameter values */

            /* Approximate by least-squares */

            N_SrfSetSizeIndices( sur, nh, mh, p, q, nh + p + 1, mh + q + 1 );

            /* build a (nh+1)x(mh+1) control point surface using the knot vectors stored */
            /* in knu and knv that minimizes the least-square distance                   */
            /*  between each P sample point and its associated surface uv point          */
            error = N_FitSrfLstSqKnots( (NL_VOID ** )P, mu, mv, NL_EPOINT, u, v, knu, knv, nh, mh, p, q, sur, &SL );

            if( error EQ NL_YES )
                break;

            /* Adjust parameters and error matrix */

            apr = NL_TRUE;

            for ( k = 1; k < mu; k++ )
            {
                for ( l = 1; l < mv; l++ )
                {
                    error = N_GetClosestPtOnSrf( sur, P[k][l], u[k], v[l], ltop, ltoc, &u[k], &v[l], &Q, SD );

                    if( error EQ NL_YES )
                    {
                        apr = NL_FALSE;
                        break;
                    }

                    N_DistPtPt( P[k][l], Q, &er[k][l] );

                    if( er[k][l]GT E )
                    {
                        apr = NL_FALSE;
                        break;
                    }
                }

                if( apr EQ NL_FALSE )
                    break;
            }

            if( apr EQ NL_FALSE )
                break;

            if( ju LT pr )
                ju++;

            if( jv LT qr )
                jv++;
        }

        if( ju EQ pr AND jv EQ qr )
            break;

        if( iu LT pr )
            iu++;

        if( iv LT qr )
            iv++;
    } /* end  while( iu LE pr AND iv LE qr ) */

    /* Compact surface */

    if( reset EQ NL_YES )
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
/* N_FitSrfToVariablePts: Surface approximation to variable number of points       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface approximating a given
     set of  rectangularly arranged points. In each column the number of 
     points may be  different. Optionally any  number of boundary curves
     may be specified. The boundary curves must be non-rational   appro-
     ximating the points along the perimeter of the point set. The point
     set is arranged as follows:

       P[0][0],...,P[0][m[0]]
       P[1][0],...,P[1][m[1]]
       ...
       P[n][0],...,P[n][m[n]]

     See N_AllocVariable2dPtArray  for allocating  memory for such  an array. A  typical 
     calling example is:

       NL_POINT    **P;
       NL_INDEX    n, *m;
       NL_DEGREE   p, q;
       NL_REAL     eps, peu, pev, pek;
       NL_CURVE    **bndU, **bndV
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (define array P, choose p, q, peu, ..., pek, get bndU and bndV);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfToVariablePts(P,n,m,p,q,eps,peu,pev,pek,bndU,bndV,&sur,&SC,&SS);
       N_FitSrfToVariablePts(P,n,m,p,q,eps,peu,pev,pek,NULL,NULL,&sur,&SC,&SS);

     The tolerance eps is split into three parts:

       1. pev*eps is used to  approximate columns of  data points in the
          v-direction;
       2. peu*eps is used to  approximate rows of control  points in the
          u-direction; and
       3. pek*eps  is used to perform a global knot removal of the final
          surface.

     peu+pev+pek=1.0  must always hold. This  method allows  the user to 
     control the  approximation  based on the  distribution of the data. 
     For example, if the columns of data  points are dense, however, the
     columns are placed, i.e., sampled,  far apart, the distribution may 
     be  {peu,pev,pek}={0.0,0.8,0.2}. That is,  80% of the error  toler-
     ance is used to approximate columns of data points,  0% is  allowed 
     for u-directional approximation,  i.e. an interpolant is used,  and 
     20% of the error is given to the knot  removal routine  to clean up 
     the surface. If the points are well distributed,  a good default is 
     {0.4,0.4,0.2}.


   ACCESS:
   
     P    , input  ,  Points to be approximated
     n    , input  ,  Highest index in u-direction
     m    , input  ,  Highest indexes in v-direction
     p,q  , input  ,  Degrees of approximating surface
     eps  , input  ,  Tolerance;  the  approximating  surface  does  not 
                      deviate from the data points more than eps
     peu  , input  ,  Percentage of tolerance allowed for  u-directional
                      approximation
     pev  , input  ,  Percentage of tolerance allowed for  v-directional
                      approximation
     pek  , input  ,  Percentage of tolerance allowed for  knot removal.
                      peu+pev+pek=1.0 MUST HOLD!
     bndU , input  ,  Boundary curves in the u-direction:
                        bndU[0]: v=vmin boundary
                        bndU[1]: v=vmax boundary
                      If bndU=NULL, no boundary  passed in. Both bndU[0]
                      and bndU[1] can independently  be NULL, indicating
                      no constraints along v=vmin and/or v=vmax.
     bndV , input  ,  Boundary curves in the v-direction:
                        bndV[0]: u=umin boundary
                        bndV[1]: u=umax boundary
                      If bndV=NULL, no boundary  passed in. Both bndV[0]
                      and bndV[1] can independently  be NULL, indicating
                      no constraints along u=umin and/or u=umax.
     sur  , output ,  Approximating surface
     SC   , input  ,  bndU's and bndV's memory stacks
     SS   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfToVariablePts
  (NL_POINT   ** P,    /* in : rows-of-points, sized:[n+1][m[i]+1] */ 
   NL_INDEX      n,    /* in : highest 1st index in P */ 
   NL_INDEX    * m,    /* in : highest 2nd index in each P row, sized:[n+1] */ 
   NL_DEGREE     p,    /* in : output surface degree U */ 
   NL_DEGREE     q,    /* in : output surface degree U */ 
   NL_REAL       eps,  /* in : max distance between output surface and any P point */ 
   NL_REAL       peu,  /* in : Percentage of tolerance allowed for u-directional tolerance */ 
   NL_REAL       pev,  /* in : Percentage of tolerance allowed for v-directional tolerance  */ 
   NL_REAL       pek,  /* in : Percentage of tolerance allowed for  knot removal. */
                       /*      note: 1.0 = peu + pev + pek */ 
   NL_CURVE   ** bndU, /* in : opt u boundaries, bndU[0]: v=vmin, bndU[1]: v=vmax, NULL to ignore  */ 
   NL_CURVE   ** bndV, /* in : opt v boundaries, bndV[0]: u=umin, bndV[1]: u=umax, NULL to ignore */ 
   NL_SURFACE   * sur, /* out: approximating surface */ 
   NL_STACKS    * SC,  /* in : bndU's and bndV's memory stacks */ 
   NL_STACKS    * SS ) /* in : sur's memory stack              */ 
{               

    NL_PRIVATE NL_STRING rname = _T("N_FitSrfToVariablePts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, is, ie, js, je, ns, ms, rs, ss, mu = 0, mv = 0;

    NL_DEGREE ps, qs, pc, qc;

    NL_REAL *US, *VS, *UT, *VT, *UR = NULL, *VR = NULL, epu, epv, epk;

    NL_POINT *Q;

    NL_CPOINT ** Sw, *Cw;

    NL_KNOTVECTOR *knu, *knv;

    NL_CURVE ** curV, ** curU;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check incoming data */

    if( (peu + pev + pek)GT 1.0 )
        NL_ERROR( NL_INP_ERR );

    epu = peu * eps;
    epv = pev * eps;
    epk = pek * eps;
    ps = p;
    qs = q;

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
        {
            if( N_IsCrvRat( bndU[0] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndU[0], &pc );
            ps = NL_MAX( ps, pc );
        }

        if( bndU[1]NEQ NULL )
        {
            if( N_IsCrvRat( bndU[1] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndU[1], &pc );
            ps = NL_MAX( ps, pc );
        }
    }

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
        {
            N_CrvGetDegree( bndU[0], &pc );

            if( pc LT ps )
            {
                error = N_CrvElevateDegree( bndU[0], ps - pc, bndU[0], SC, SC );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }

        if( bndU[1]NEQ NULL )
        {
            N_CrvGetDegree( bndU[1], &pc );

            if( pc LT ps )
            {
                error = N_CrvElevateDegree( bndU[1], ps - pc, bndU[1], SC, SC );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL )
        {
            if( N_IsCrvRat( bndV[0] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndV[0], &qc );
            qs = NL_MAX( qs, qc );
        }

        if( bndV[1]NEQ NULL )
        {
            if( N_IsCrvRat( bndV[1] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndV[1], &qc );
            qs = NL_MAX( qs, qc );
        }
    }

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL )
        {
            N_CrvGetDegree( bndV[0], &qc );

            if( qc LT qs )
            {
                error = N_CrvElevateDegree( bndV[0], qs - qc, bndV[0], SC, SC );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }

        if( bndV[1]NEQ NULL )
        {
            N_CrvGetDegree( bndV[1], &qc );

            if( qc LT qs )
            {
                error = N_CrvElevateDegree( bndV[1], qs - qc, bndV[1], SC, SC );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    /* Check compatibility */

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL AND bndU[1]NEQ NULL )
        {
            if( NOT N_CrvsAreCombatible( bndU, 1 ) )
            {
                error = N_CrvsMakeCompatible( bndU, 1, SC );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL AND bndV[1]NEQ NULL )
        {
            if( NOT N_CrvsAreCombatible( bndV, 1 ) )
            {
                error = N_CrvsMakeCompatible( bndV, 1, SC );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
            N_CrvGetKnots( bndU[0], &mu, &UR );

        else if( bndU[1]NEQ NULL )
            N_CrvGetKnots( bndU[1], &mu, &UR );
    }
    else
    {
        UR = NULL;
    }

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL )
            N_CrvGetKnots( bndV[0], &mv, &VR );

        else if( bndV[1]NEQ NULL )
            N_CrvGetKnots( bndV[1], &mv, &VR );
    }
    else
    {
        VR = NULL;
    }

    /* Approximate columns of data points */

    curV = N_AllocArrayCrvPtrs( n, &SL );

    if( curV EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        curV[i] = N_AllocCrv( &SL );

        if( curV[i]EQ NULL )
            NL_QUIT;
    }

    is = 0;
    ie = n;

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL )
        {
            N_CrvInitArrays( curV[0] );
            error = N_CrvCopy( bndV[0], curV[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            is = 1;
        }

        if( bndV[1]NEQ NULL )
        {
            N_CrvInitArrays( curV[n] );
            error = N_CrvCopy( bndV[1], curV[n], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            ie = n - 1;
        }
    }

    knv = NULL;

    for ( i = is; i <= ie; i++ )
    {
        N_CrvInitArrays( curV[i] );
        error = N_FitCrvApproxKnotsTol( P[i], m[i], qs, NULL, NULL, NL_TANGENT, epv, &knv, NL_YES, curV[i], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curV, n, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curV[0], &ss, &VT );
    N_CrvGetArraySizes( curV[0], &ms, &ss );

    /* Now approximate in the u-direction */

    curU = N_AllocArrayCrvPtrs( ms, &SL );

    if( curV EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= ms; j++ )
    {
        curU[j] = N_AllocCrv( &SL );

        if( curU[j]EQ NULL )
            NL_QUIT;
    }

    js = 0;
    je = ms;

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
        {
            N_CrvInitArrays( curU[0] );
            error = N_CrvCopy( bndU[0], curU[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            js = 1;
        }

        if( bndU[1]NEQ NULL )
        {
            N_CrvInitArrays( curU[ms] );
            error = N_CrvCopy( bndU[1], curU[ms], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            je = ms - 1;
        }
    }

    Q = N_AllocPt1dArray( n, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    knu = NULL;

    for ( j = js; j <= je; j++ )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CrvGetCPts( curV[i], &ms, &Cw );
            N_CPtToPtEuclid( Cw[j], &Q[i] );
        }

        N_CrvInitArrays( curU[j] );
        error = N_FitCrvApproxKnotsTol( Q, n, ps, NULL, NULL, NL_TANGENT, epu, &knu, NL_YES, curU[j], &SL, &SL );

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

    error = N_SrfSizeArrays( sur, ns, ms, ps, qs, rs, ss, rname, SS );

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

    /* Remove all removable knots */

    if( epk GT 0.0 )
    {
        error = N_SrfRemoveKnotsConstraints( sur, UR, mu, VR, mv, epk, NL_UVDIR, sur, SS );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFAPPROXSHAPE: Approximate random points based on surface shaping       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  interpolates boundary  curves, any  number of  
     cross-derivatives, and  approximates internal  points  by shaping a 
     base  surface, while  maintaining  the boundary  curves and  cross-
     derivatives. This  routine  works  best for large  number of points 
     representing a simple surface as in reverse engineering  after  the 
     point set has been  segmented into simple  parts. The points can be 
     very irregularly  spaced as long as they  represent a simple shape. 
     The method  first computes a base  surface which  then is forced to 
     approximate the  given  points via  constrained  shaping. The  base 
     surface  is a  bi-linearly  or  bi-cubically  blended  Coons patch, 
     interpolating the boundary curves  and any number of cross-boundary 
     derivatives. A typical calling example is:

       NL_FLAG     ftl;
       NL_SURFACE  sur;
       NL_CURVE    **curU, **curV, **derU, **derV;
       NL_POINT    *P;
       NL_INDEX    k;
       NL_REAL     tol
       NL_STACKS   SC, SG;
       ...
       (get points P, tol, boundaries and cross-derivatives);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfApproxShape(P,k,curU,curV,derU,derV,tol,&sur,&ftl,&SC,&SG);
       N_FitSrfApproxShape(P,k,curU,curV,NULL,NULL,tol,&sur,&ftl,&SC,&SG);

     THE  DERIVATIVES  MUST BE NL_TWIST  COMPATIBLE AT  EACH CORNER. IF THE 
     BOUNDARIES AND THE  DERIVATIVES ARE NOT  COMPATIBLE IN THE B-SPLINE
     SENSE, THEY WILL BE MADE SO, I.E. THE IMPUT DATA WILL BE DESTROYED!
     THE BOUNDARY CURVES AND CROSS-DERIVATIVES MUST BE NON-RATIONAL!


   ACCESS:
   
     P    , input  ,  Random points output surface must approximate
     k    , input  ,  Highest index in P
     curU , in/out ,  U-boundaries:
                       curU[0]: v=vmin boundary
                       curU[1]: v=vmax boundary
                      MUST BE NON-RATIONAL! Curves need not be compatible,
                      will be made compatible by this function.
     curV , in/out ,  V-boundaries:
                       curV[0]: u=umin boundary
                       curV[1]: u=umax boundary
                      MUST BE NON-RATIONAL!  Curves need not be compatible,
                      will be made compatible by this function.
     derU , in/out ,  Cross-boundary derivatives:
                        = NULL: None past in
                       != NULL: One or both passed in
                        derU[0]: cross-derivative across u=umin boundary
                        derU[1]: cross-derivative across u=umax boundary
                      derU[0] or derU[1] can be individually NULL, indi-
                      cating the  absence of  cross-derivative along the 
                      edge it belongs. MUST BE NON-RATIONAL!
     derV , in/out ,  Cross-boundary derivatives:
                        = NULL: None past in
                       != NULL: One or both passed in
                        derV[0]: cross-derivative across v=vmin boundary
                        derV[1]: cross-derivative across v=vmax boundary
                      derV[0] or derV[1] can be individually NULL, indi-
                      cating the  absence of  cross-derivative along the 
                      edge it belongs. MUST BE NON-RATIONAL!
     tol  , input  ,  The  filtered  average  error of the approximation 
                      must be less than tol. The shaping iteration stops 
                      when:
                         1. the average error is less than tol;
                         2. there is no improvement in  two  consecutive
                            iterates; or
                         3. the error gets worse
     sur  , output ,  Surface   approximating  P[i],  i=0,...,k,   while 
                      interpolating curU, curV, derU and derV
     ftl  , output ,  Flag:
                        NL_YES: approximant is within tol
                        NL_NO : error  condition  is not satisfied, but the 
                             best surface is returned
     SC   , input  ,  Stack of  curU, curV, derU  and derV. These curves
                      must all be on the same stack SC 
     SG   , input  ,  sur's stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
                                                                                  
NL_FLAG N_FitSrfApproxShape                                                       
  (NL_POINT   * P,      /* in : cloud-of-points, sized:[k+1] */
   NL_INDEX     k,      /* in : highest index in P */
   NL_CURVE  ** curU,   /* i/o: U bndry position cstrn, curU[0]: v=vmin boundary, curU[1]: v=vmax boundary */
   NL_CURVE  ** curV,   /* i/o: V bndry position cstrn, curV[0]: u=umin boundary, curV[1]: u=umax boundary */
   NL_CURVE  ** derU,   /* i/o: opt bndry U-cross-tangent cstrn, derU[0]: u=umin boundary, curU[1]: u=umax boundary, NULL to ignore */
   NL_CURVE  ** derV,   /* i/o: opt bndry V-cross-tangent cstrn, derV[0]: v=vmin boundary, curV[1]: v=vmax boundary, NULL to ignore */
   NL_REAL      tol,    /* in : max avg error allowed, iteration stops when 1. avg error < tol,                         */
                        /*                                                  2. no improvement in consecutive iterations */ 
                        /*                                                  3. error gets worse                         */                              
   NL_SURFACE * sur,    /* out: approximating surface */                               
   NL_FLAG    * ftl,    /* out: NL_YES = approximate is within tol */
                        /*      NL_NO  = error condition not satisfied, but best surface found is returned */
   NL_STACKS  * SC,     /* in : curU, curV, derU, derV memory stack. */
   NL_STACKS  * SG )    /* in : sur's memory stack */

  {
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfApproxShape");

    NL_FLAG error = NL_NO, L = NL_NO, R = NL_NO, B = NL_NO, T = NL_NO;

    NL_INDEX rc0, rc1, sc0, sc1, rd0 = 0, rd1 = 0, sd0 = 0, sd1 = 0, rs, ss, code, dsu, deu, dsv, dev, i, ns, ms;

    NL_DEGREE ps, qs;

    NL_REAL *UC0, *UC1, *VC0, *VC1, *UD0 = NULL, *UD1 = NULL, *VD0 = NULL, *VD1 = NULL, *US, *VS, uu, vv;

    NL_POINT ** SD, ** SE, Ts[2], Te[2];

    NL_VECTOR D00_u, D00_v, D10_u, D10_v, D01_u, D01_v, D11_u, D11_v;

    NL_SURFACE surB, surS;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input data */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( derU NEQ NULL )
    {
        if( derU[0]NEQ NULL )
            if( N_IsCrvRat( derU[0] ) )
                NL_ERROR( NL_INP_ERR );

        if( derU[1]NEQ NULL )
            if( N_IsCrvRat( derU[1] ) )
                NL_ERROR( NL_INP_ERR );
    }

    if( derV NEQ NULL )
    {
        if( derV[0]NEQ NULL )
            if( N_IsCrvRat( derV[0] ) )
                NL_ERROR( NL_INP_ERR );

        if( derV[1]NEQ NULL )
            if( N_IsCrvRat( derV[1] ) )
                NL_ERROR( NL_INP_ERR );
    }

    /* Get locals and allocate memory */

    dsu = deu = dsv = dev = 0;

    N_CrvGetKnots( curU[0], &rc0, &UC0 );
    N_CrvGetKnots( curU[1], &rc1, &UC1 );
    N_CrvGetKnots( curV[0], &sc0, &VC0 );
    N_CrvGetKnots( curV[1], &sc1, &VC1 );

    if( derU NEQ NULL )
    {
        if( derU[0]NEQ NULL )
        {
            L = NL_YES;
            dsu = 1;
            N_CrvGetKnots( derU[0], &rd0, &UD0 );
        }

        if( derU[1]NEQ NULL )
        {
            R = NL_YES;
            deu = 1;
            N_CrvGetKnots( derU[1], &rd1, &UD1 );
        }
    }

    if( derV NEQ NULL )
    {
        if( derV[0]NEQ NULL )
        {
            B = NL_YES;
            dsv = 1;
            N_CrvGetKnots( derV[0], &sd0, &VD0 );
        }

        if( derV[1]NEQ NULL )
        {
            T = NL_YES;
            dev = 1;
            N_CrvGetKnots( derV[1], &sd1, &VD1 );
        }
    }

    code = (L EQ NL_YES) << 3 | (R EQ NL_YES) << 2 | (B EQ NL_YES) << 1 | (T EQ NL_YES);

    /* Get bilinear Coons patch first - side effect: boundary curves made compatible */

    N_SrfInitArrays( &surB );
    error = N_CreateCoonsSrf( curU, curV, &surB, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* No or all cross-boundaries are given */

    if( code EQ 0 )
        goto SHAPE;

    if( code EQ 15 )
        goto BCC;

    /* Allocate memory */

    SD = N_AllocPt2dArray( 1, 1, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    SE = N_AllocPt2dArray( 1, 1, &SL );

    if( SE EQ NULL )
        NL_QUIT;

    if( derU EQ NULL )
    {
        derU = N_AllocArrayCrvPtrs( 1, SC );

        if( derU EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= 1; i++ )
        {
            derU[i] = N_AllocCrv( SC );

            if( derU[i]EQ NULL )
                NL_QUIT;
        }
    }

    if( derV EQ NULL )
    {
        derV = N_AllocArrayCrvPtrs( 1, SC );

        if( derV EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= 1; i++ )
        {
            derV[i] = N_AllocCrv( SC );

            if( derV[i]EQ NULL )
                NL_QUIT;
        }
    }

    if( derU NEQ NULL )
    {
        if( derU[0]EQ NULL )
        {
            derU[0] = N_AllocCrv( SC );

            if( derU[0]EQ NULL )
                NL_QUIT;
        }

        if( derU[1]EQ NULL )
        {
            derU[1] = N_AllocCrv( SC );

            if( derU[1]EQ NULL )
                NL_QUIT;
        }
    }

    if( derV NEQ NULL )
    {
        if( derV[0]EQ NULL )
        {
            derV[0] = N_AllocCrv( SC );

            if( derV[0]EQ NULL )
                NL_QUIT;
        }

        if( derV[1]EQ NULL )
        {
            derV[1] = N_AllocCrv( SC );

            if( derV[1]EQ NULL )
                NL_QUIT;
        }
    }

    /* Compute boundary derivatives */

    error = N_CrvDerivs( curU[0], UC0[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curU[0], UC0[rc0], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D00_u );
    N_VectorCopy( Te[1], &D10_u );

    error = N_CrvDerivs( curU[1], UC1[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curU[1], UC1[rc1], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D01_u );
    N_VectorCopy( Te[1], &D11_u );

    error = N_CrvDerivs( curV[0], VC0[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV[0], VC0[sc0], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D00_v );
    N_VectorCopy( Te[1], &D01_v );

    error = N_CrvDerivs( curV[1], VC1[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV[1], VC1[sc1], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D10_v );
    N_VectorCopy( Te[1], &D11_v );

    /* Compute base surface without cross derivatives */

    N_SrfInitArrays( &surS );
    error = N_SrfShapeApproxPts( &surB, P, k, 0, 0, 0, 0, tol, &surS, ftl, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnots( &surS, &rs, &ss, &US, &VS );

    /* One cross-boundary is missing */

    if( code EQ 14 )
    {
        /* Top cross-boundary is missing */

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 7 )
    {
        /* Left cross-boundary is missing */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 13 )
    {
        /* Bottom cross-boundary is missing */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 11 )
    {
        /* Right cross-boundary is missing */

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }

    /* Two cross-derivatives are missing */

    if( code EQ 12 )
    {
        /* Bottom and top cross-derivatives are missing */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 3 )
    {
        /* Left and right cross-derivatives are missing */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 10 )
    {
        /* Upper right twist is missing */

        error = N_SrfDerivs( &surS, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 6 )
    {
        /* Upper left twist is missing */

        error = N_SrfDerivs( &surS, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 5 )
    {
        /* Lower left twist is missing */

        error = N_SrfDerivs( &surS, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 9 )
    {
        /* Lower right twist is missing */

        error = N_SrfDerivs( &surS, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }

    /* Three cross-derivatives are missing */

    if( code EQ 8 )
    {
        /* Lower and upper right twists are missing */

        error = N_SrfDerivs( &surS, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        N_CrvInitArrays( derU[1] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_RIGHT, derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Te[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 2 )
    {
        /* Upper left and right twists are missing */

        error = N_SrfDerivs( &surS, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Te[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        N_CrvInitArrays( derV[1] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_TOP, derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 4 )
    {
        /* Lower left and upper right twists are missing */

        error = N_SrfDerivs( &surS, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        N_CrvInitArrays( derU[0] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_LEFT, derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Ts[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 1 )
    {
        /* Lower left and right twists are missing */

        error = N_SrfDerivs( &surS, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        N_CrvInitArrays( derV[0] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_BOTTOM, derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Ts[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }

    /* Make bi-cubic Coons patch */

    BCC:

    N_SrfInitArrays( &surB );
    error = N_CreateCoonsBoundaryCrvs( curV[0], derU[0], curV[1], derU[1], curU[0], derV[0], curU[1], derV[1], NL_YES, &surB, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnots( &surB, &rs, &ss, &US, &VS );
    N_SrfGetDegrees( &surB, &ps, &qs );
    N_SrfGetArraySizes( &surB, &ns, &ms, &rs, &ss );

    if( ns EQ ps )
    {
        uu = 0.5 *( US[0] + US[rs] );

        error = N_SrfInsertKnot( &surB, uu, 1, NL_UDIR, &surB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( ms EQ qs )
    {
        vv = 0.5 *( VS[0] + VS[ss] );

        error = N_SrfInsertKnot( &surB, vv, 1, NL_VDIR, &surB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Shape base surface to pass through the given points */

    SHAPE:

    error = N_SrfShapeApproxPts( &surB, P, k, dsu, deu, dsv, dev, tol, sur, ftl, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFAPPROXTANGENTSTOL: Surface approximation with error bounds and tangents     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine  computes a surface  approximating a  given  set 
     of points  to given  tolerances.  Tangents can be  specified  on  the
     boundaries. Separate tolerances can be specified for interior points,
     boundary points and boundary tangents. A typical calling example:
 
       NL_POINT    **P;
       NL_DEGREE   p, q;
       NL_VECTOR   ***Tu, ***Tv;
       NL_INDEX    k, l;
       NL_REAL     emx, eub, evb, eut, evt;
       NL_SURFACE  sur;
       ...
       (get array P, tangents and tolerances);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfApproxTangentsTol(P,k,l,p,q,Tu,Tv,emx,eub,evb,eut,evt);
 
     All  tangent  tolerances are  considered  directions  only  and are 
     measured in degrees.
 
 
   ACCESS:
   
     P     , input  ,  Points to  be  approximated,  P[i][j], i=0,...,k; 
                       j=0,...l.
     k,l   , input  ,  Highest indexes in P
     p,q   , input  ,  Degrees of approximating surface
     Tu    , input  ,  Tangent vectors in the u-direction along the umin  
                       and umax boundaries.  Tu[i][j], i=0,1; j=0,...,l, 
                       is a  pointer  to the  tangents at the j-th point 
                       along  the  umin  (i=0) or  along  the umax (i=1) 
                       boundary:
                         Tu[i][j]  = NULL: no  tangent  supplied  at the 
                                           point (i,j)
                         Tu[i][j] != NULL: tangent supplied
                         Tu        = NULL: no tangent supplied on either 
                                           boundary
     Tv    , input  ,  Tangent vectors in the v-direction along the vmin 
                       and  vmax boundaries. Tv[i][j], i=0,...,k; j=0,1, 
                       is a pointer to the tangents  at  the  i-th point 
                       along  the  vmin (j=0)  or  along  the vmax (j=1) 
                       boundary:
                         Tv[i][j]  = NULL: no  tangent  supplied  at the 
                                           point (i,j)
                         Tv[i][j] != NULL: tangent supplied
                         Tv        = NULL: no tangent supplied on either 
                                           boundary
     emx  , input  ,  Overall error tolerance, i.e. the surface will not 
                      deviate from the origonal points by more than emx
     eub  , input  ,  Error  tolerance  along  the   v=vmin  and  v=vmax 
                      boundaries
     evb  , input  ,  Error  tolerance  along  the   u=umin  and  u=umax 
                      boundaries
     eut  , input  ,  Error  tolerance for  u-tangents on the u=umin and 
                      u=umax boundaries (angular tolerance in degrees)
     evt  , input  ,  Error tolerance  for  v-tangents on the v=vmin and 
                      v=vmax boundaries (angular tolerance in degrees)
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfApproxTangentsTol 
  (NL_POINT    ** P,     /* in : grid of sample points, sized:[k+1][l+1] */
   NL_INDEX       k,     /* in : highest 1st P index */
   NL_INDEX       l,     /* in : highest 2nd P index */
   NL_DEGREE      p,     /* in : output surface degree U */
   NL_DEGREE      q,     /* in : output surface degree V */
   NL_VECTOR  *** Tu,    /* in : opt tangent vectors along U Boundaries, sized:[2][k+1], NULL to ignore */
   NL_VECTOR  *** Tv,    /* in : opt tangent vectors along V Boundaries, sized:[2][l+1], NULL to ignore */
   NL_REAL        emx,   /* in : overall tolerance - max distance between surface and any given P point */
   NL_REAL        eub,   /* in : U boundary tolerance - max distance between surface u bounday point and boundary P points */
   NL_REAL        evb,   /* in : V boundary tolerance - max distance between surface v bounday point and boundary P points */
   NL_REAL        eut,   /* in : U boundary tangent angle tolerance, degrees */
   NL_REAL        evt,   /* in : V boundary tangent angle tolerance, degrees */
   NL_SURFACE    *sur,   /* out: approximating surface */
   NL_STACKS     *SG )   /* in : sur's memory stack */
{                         
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfApproxTangentsTol");

    NL_FLAG ufl, vfl, le, ri, error = NL_NO;

    NL_INDEX i, j, n = 0, m = 0, nr, mr;

    NL_REAL *u, *v;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get flags and check surface memory */

    le = ri = ufl = NL_NO;

    if( Tu NEQ NULL )
    {
        for ( j = 0; j <= l; j++ )
        {
            if( Tu[0][j]NEQ NULL )
                le = NL_YES;

            if( Tu[1][j]NEQ NULL )
                ri = NL_YES;

            if( le EQ NL_YES AND ri EQ NL_YES )
                break;
        }

        if( le EQ NL_YES AND ri EQ NL_NO )
            ufl = NL_START;

        else if( le EQ NL_NO AND ri EQ NL_YES )
            ufl = NL_END;

        else if( le EQ NL_YES AND ri EQ NL_YES )
            ufl = NL_BOTH;
    }

    le = ri = vfl = NL_NO;

    if( Tv NEQ NULL )
    {
        for ( i = 0; i <= k; i++ )
        {
            if( Tv[i][0]NEQ NULL )
                le = NL_YES;

            if( Tv[i][1]NEQ NULL )
                ri = NL_YES;

            if( le EQ NL_YES AND ri EQ NL_YES )
                break;
        }

        if( le EQ NL_YES AND ri EQ NL_NO )
            vfl = NL_START;

        else if( le EQ NL_NO AND ri EQ NL_YES )
            vfl = NL_END;

        else if( le EQ NL_YES AND ri EQ NL_YES )
            vfl = NL_BOTH;
    }

    if( ufl EQ NL_NO )
        n = k;

    else if( ufl EQ NL_START )
        n = k + 1;

    else if( ufl EQ NL_END )
        n = k + 1;

    else if( ufl EQ NL_BOTH )
        n = k + 2;

    if( vfl EQ NL_NO )
        m = l;

    else if( vfl EQ NL_START )
        m = l + 1;

    else if( vfl EQ NL_END )
        m = l + 1;

    else if( vfl EQ NL_BOTH )
        m = l + 2;

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get parameters and knot vectors */

    u = N_AllocReal1dArray( k, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( l, &SL );

    if( v EQ NULL )
        NL_QUIT;

    knu = N_AllocKnotVectorAndArray( n + p + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    knv = N_AllocKnotVectorAndArray( m + q + 1, &SL );

    if( knv EQ NULL )
        NL_QUIT;

    error = N_FitCalcSrfParamValues( (NL_VOID ** )P, k, l, NL_EPOINT, NL_CHORDLENGTH, u, v );

    if( error EQ NL_YES )
        NL_OUT;

    if( ufl EQ NL_NO )
        N_FitCrvCalcKnotVector( u, k, p, knu );

    else if( ufl EQ NL_START )
        N_FitCalcKnotVectorDeriv( u, k, p, NL_START, knu );

    else if( ufl EQ NL_END )
        N_FitCalcKnotVectorDeriv( u, k, p, NL_END, knu );

    else if( ufl EQ NL_BOTH )
        N_FitCalcKnotVectorEndDerivs( u, k, p, knu );

    if( vfl EQ NL_NO )
        N_FitCrvCalcKnotVector( v, l, q, knv );

    else if( vfl EQ NL_START )
        N_FitCalcKnotVectorDeriv( v, l, q, NL_START, knv );

    else if( vfl EQ NL_END )
        N_FitCalcKnotVectorDeriv( v, l, q, NL_END, knv );

    else if( vfl EQ NL_BOTH )
        N_FitCalcKnotVectorEndDerivs( v, l, q, knv );

    /* Fit data points */
    error = N_FitSrfInterpTangents( P, k, l, u, v, knu, knv, p, q, Tu, Tv, NL_TANGENT, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Remove knots */
    error = N_FitSrfApproxRemoveKnotsTangents( sur, u, v, Tu, Tv, k, l, ufl, vfl, emx, eub, evb, eut, evt );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact surface */
    N_SrfGetArraySizes( sur, &nr, &mr, &i, &j );

    if( nr LT n OR mr LT m )
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
/* N_FITSRFLSTSQKNOTS: Surface approximation with given knot vectors            */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This  fitting  routine  computes  a  least-squares  NURBS   surface 
     approximation to a  given set of grid-sampled points that have already
     been mapped to uvPoints on the surface.  The function creates
     a NURB surface of specified degree and control point count and then
     computes the control point locations to minimize the least-squares
     fit of the nurbs surface to the input grid points.  The function
     works well when the input grid of points has been regularly sampled.
     
     This is a general routine 
     requiring the input of two knot vectors, parameters at data  points
     and  the input of  NL_POINT or  NL_CPOINT data. If  the output surface is  
     initialized  to  the  NULL  surface, memory  is allocated  locally. 
     Otherwise it is  checked if  enough memory  is passed in. A typical 
     calling example is:
 
       NL_POINT       **P;
       NL_CPOINT      **Pw;
       NL_INDEX       r, s, m, n;
       NL_DEGREE      p, q;
       NL_PARAMETER   *u, *v;
       NL_KNOTVECTOR  *knu, *knv;
       NL_SURFACE     sur;
       NL_STACKS      SG;
       ...
       (get arrays P/Pw, u, v; define knu, knv; choose n, m, p, q);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqKnots((NL_VOID **)P ,r,s,NL_EPOINT,u,v,knu,knv,n,m,p,q,&sur,&SG);
       N_FitSrfLstSqKnots((NL_VOID **)Pw,r,s,NL_HPOINT,u,v,knu,knv,n,m,p,q,&sur,&SG);
 
     IT IS ASSUMED THAT THE NL_PARAMETERS AND THE KNOT VECTORS ARE COMPUTED
     IN THE CALLING ROUTINE.
 
 
   ACCESS:
   
     A   , input  ,  Pointer to NL_POINT or NL_CPOINT data
     r,s , input  ,  Highest indexes in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     u,v , input  ,  Parameters corresponding to data points
     knu , input  ,  Knot vector in u-direction
     knv , input  ,  Knot vector in v-direction
     n,m , input  ,  Highest indexes of control point array of  approxi-
                     mating surface (must satisfy n <= r, m <= s)
     p,q , input  ,  Degrees  of  approximating  surface  (must  satisfy 
                     p <= n, q <= m)
     sur , output ,  Approximating surface
     SG  , input  ,  sur's memory stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfLstSqKnots
  (NL_VOID     ** A,      /* in : Grid of Sample Points, sized:[r+1][s+1] */
   NL_INDEX       r,      /* in : highest 1st A index */
   NL_INDEX       s,      /* in : highest 2nd A index */
   NL_FLAG        ptp,    /* in : A type flag: NL_EPOINT = A contains NL_POINT  objects */
                          /*                   NL_HPOINT = A contains NL_CPOINT objects */
   NL_PARAMETER  *u,      /* in : u param values for all A[i][*] points, sized:[r+1] */
   NL_PARAMETER  *v,      /* in : v param values for all A[*][j] points, sized:[s+1] */
   NL_KNOTVECTOR *knu,    /* in : output knot vector u, with n+p+1+1 knots */
   NL_KNOTVECTOR *knv,    /* in : output knot vector v, with m+q+1+1 knots */
   NL_INDEX       n,      /* in : output highest control point index U */
   NL_INDEX       m,      /* in : output highest control point index V */
   NL_DEGREE      p,      /* in : output degree U */
   NL_DEGREE      q,      /* in : output degree V */
   NL_SURFACE    *sur,    /* out: approximated surface, [n+1][m+1] control points */
   NL_STACKS     *SG )    /* in : sur memory stack */
{                         
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfLstSqKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX *index, *start, *end, i, j, k, l, ub, vb, usb, vsb, rj, sj, ej, lk, hk, lj, hj, rk, sk, rs, ss;

    NL_REAL ** B, ** NTN, *N, *UK, *VK, *US, *VS, *ts, *te;

    NL_POINT ** S = NULL, *Rk, *R;

    NL_CPOINT ** Pw, ** Qw, ** Sw = NULL, *Rkw, *Rw, *Aw;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error */

    N_KnotVectorGetKnots( knu, &rk, &UK );
    N_KnotVectorGetKnots( knv, &sk, &VK );
    rs = n + p + 1;
    ss = m + q + 1;

    if( rk NEQ rs OR sk NEQ ss )
        NL_ERROR( NL_INP_ERR );

    if( n GT r OR n LT 1 OR p GT n )
        NL_ERROR( NL_INP_ERR );

    if( m GT s OR m LT 1 OR q GT m )
        NL_ERROR( NL_INP_ERR );

    if( n EQ 1 )
    {
        if( m GT 1 AND q GT m )
            NL_ERROR( NL_INP_ERR );
    }
    else if( m EQ 1 )
    {
        if( n GT 1 AND p GT n )
            NL_ERROR( NL_INP_ERR );
    }

    else if( (n GT 1 AND m GT 1)AND( p GT n OR q GT m ) )
        NL_ERROR( NL_INP_ERR );

    /* Check for surface memory */

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &US, &VS );

    for ( i = 0; i <= rk; i++ )
        US[i] = UK[i];

    for ( j = 0; j <= sk; j++ )
        VS[j] = VK[j];

    /* Handle special cases */

    switch( ptp )
    {
        case NL_EPOINT:

            S = (NL_POINT ** )A;

            if( n EQ 1 AND m EQ 1 )
            {
                N_PtToCPt( S[0][0], &Pw[0][0] );
                N_PtToCPt( S[r][0], &Pw[1][0] );
                N_PtToCPt( S[0][s], &Pw[0][1] );
                N_PtToCPt( S[r][s], &Pw[1][1] );

                NL_OUT;
            }

            if( n EQ 1 )
            {
                for ( j = 0; j <= s; j++ )
                    N_CopyPt( S[r][j], &S[1][j] );
            }

            if( m EQ 1 )
            {
                for ( i = 0; i <= r; i++ )
                    N_CopyPt( S[i][s], &S[i][1] );
            }
            break;

        case NL_HPOINT:

            Sw = (NL_CPOINT ** )A;

            if( n EQ 1 AND m EQ 1 )
            {
                N_CopyCPt( Sw[0][0], &Pw[0][0] );
                N_CopyCPt( Sw[r][0], &Pw[1][0] );
                N_CopyCPt( Sw[0][s], &Pw[0][1] );
                N_CopyCPt( Sw[r][s], &Pw[1][1] );

                NL_OUT;
            }

            if( n EQ 1 )
            {
                for ( j = 0; j <= s; j++ )
                    N_CopyCPt( Sw[r][j], &Sw[1][j] );
            }

            if( m EQ 1 )
            {
                for ( i = 0; i <= r; i++ )
                    N_CopyCPt( Sw[i][s], &Sw[i][1] );
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Allocate memory */

    ub = 2 * p + 1;
    vb = 2 * q + 1;
    usb = p;
    vsb = q;
    i = NL_MAX( r - 2, s - 2 );
    k = NL_MAX( p, q );
    j = NL_MAX( n - 2, m - 2 );
    l = NL_MAX( ub - 1, vb - 1 );

    B = N_AllocReal2dArray( i, k, &SL );

    if( B EQ NULL )
        NL_QUIT;

    NTN = N_AllocReal2dArray( j, l, &SL );

    if( NTN EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( k, &SL );

    if( N EQ NULL )
        NL_QUIT;

    ts = N_AllocReal1dArray( i, &SL );

    if( ts EQ NULL )
        NL_QUIT;

    te = N_AllocReal1dArray( i, &SL );

    if( te EQ NULL )
        NL_QUIT;

    index = N_AllocInt1dArray( i, &SL );

    if( index EQ NULL )
        NL_QUIT;

    start = N_AllocInt1dArray( j, &SL );

    if( start EQ NULL )
        NL_QUIT;

    end = N_AllocInt1dArray( j, &SL );

    if( end EQ NULL )
        NL_QUIT;

    Rk = N_AllocPt1dArray( i, &SL );

    if( Rk EQ NULL )
        NL_QUIT;

    R = N_AllocPt1dArray( j, &SL );

    if( R EQ NULL )
        NL_QUIT;

    Rkw = N_AllocCPt1dArray( i, &SL );

    if( Rkw EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( j, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    Aw = N_AllocCPt1dArray( j, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    Qw = N_AllocCPt2dArray( n, s, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    /* Compute coefficient matrix for u-directional approximation */

    if( n GT 1 )
    {
        rj = p;
        sj = p - 1;
        ej = -2;

        for ( i = 0; i <= n - 2; i++ )
        {
            for ( j = 0; j < ub; j++ )
                NTN[i][j] = 0.0;
        }

        for ( i = 0; i <= r - 2; i++ )
            B[i][p] = 0.0;

        for ( i = 0; i <= NL_MIN( p - 1, n - 2 ); i++ )
            start[i] = 0;

        end[0] = -2;

        for ( i = 1; i <= r - 1; i++ )
        {
            error = N_BasisEval( knu, p, u[i], NL_LEFT, N, &j );

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
            end[i] = r - 2;

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
                    NTN[i][j - i + usb] += B[k][i - index[k]] * B[k][j - index[k]];
                }
            }
        }

        N_CreateRealMatrix( &cm, n - 2, n - 2, NTN, NL_MT_BANDED, ub );

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        /* Approximate rows in the u-direction */

        for ( i = 1; i <= r - 1; i++ )
        {
            error = N_BasisIEval( knu, 0, p, u[i], NL_LEFT, &ts[i - 1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisIEval( knu, n, p, u[i], NL_LEFT, &te[i - 1] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( m EQ 1 )
            l = 1;
        else
            l = s;

        switch( ptp )
        {
            case NL_EPOINT:
                for ( j = 0; j <= l; j++ )
                {
                    if( m EQ 1 )
                    {
                        N_PtToCPt( S[0][j], &Pw[0][j] );
                        N_PtToCPt( S[r][j], &Pw[n][j] );
                    }
                    else
                    {
                        N_PtToCPt( S[0][j], &Qw[0][j] );
                        N_PtToCPt( S[r][j], &Qw[n][j] );
                    }

                    for ( i = 1; i <= r - 1; i++ )
                    {
                        N_TranslateSum2Pts( S[i][j], -ts[i - 1], S[0][j], -te[i - 1], S[r][j], &Rk[i - 1] );
                    }

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

                    error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, Aw, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    for ( i = 0; i <= n - 2; i++ )
                    {
                        if( m EQ 1 )
                            N_CopyCPt( Aw[i], &Pw[i + 1][j] );
                        else

                            N_CopyCPt( Aw[i], &Qw[i + 1][j] );
                    }
                }
                break;

            case NL_HPOINT:
                for ( j = 0; j <= l; j++ )
                {
                    if( m EQ 1 )
                    {
                        N_CopyCPt( Sw[0][j], &Pw[0][j] );
                        N_CopyCPt( Sw[r][j], &Pw[n][j] );
                    }
                    else
                    {
                        N_CopyCPt( Sw[0][j], &Qw[0][j] );
                        N_CopyCPt( Sw[r][j], &Qw[n][j] );
                    }

                    for ( i = 1; i <= r - 1; i++ )
                    {
                        N_TranslateSum2CPts( Sw[i][j], -ts[i - 1], Sw[0][j], -te[i - 1], Sw[r][j], &Rkw[i - 1] );
                    }

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

                    error = N_RealMatrixForBack( &cm, (NL_VOID *)Rw, NL_HPOINT, Aw, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    for ( i = 0; i <= n - 2; i++ )
                    {
                        if( m EQ 1 )
                            N_CopyCPt( Aw[i], &Pw[i + 1][j] );
                        else

                            N_CopyCPt( Aw[i], &Qw[i + 1][j] );
                    }
                }
        }
    }

    /* Compute coefficient matrix for v-directional approximation */

    if( m GT 1 )
    {
        rj = q;
        sj = q - 1;
        ej = -2;

        for ( i = 0; i <= m - 2; i++ )
        {
            for ( j = 0; j < vb; j++ )
                NTN[i][j] = 0.0;
        }

        for ( i = 0; i <= s - 2; i++ )
            B[i][q] = 0.0;

        for ( i = 0; i <= NL_MIN( q - 1, m - 2 ); i++ )
            start[i] = 0;

        end[0] = -2;

        for ( i = 1; i <= s - 1; i++ )
        {
            error = N_BasisEval( knv, q, v[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            if( j EQ q )
                l = 1;
            else
                l = 0;

            if( j EQ q OR j EQ m )
                hk = q - 1;
            else
                hk = q;

            for ( k = 0; k <= hk; k++ )
                B[i - 1][k] = N[l + k];

            index[i - 1] = NL_MAX( 0, j - q - 1 );

            if( j GT rj )
            {
                for ( k = 1; k <= j - rj; k++ )
                {
                    sj++;
                    ej++;

                    if( sj LE m - 2 )
                        start[sj] = i - 1;

                    if( ej GE 0 )
                        end[ej] = i - 2;
                }
                rj = j;
            }
        }

        if( sj LT m - 2 OR end[0]EQ - 1 )
            NL_ERROR( NL_INP_ERR );

        for ( i = NL_MAX( 0, ej + 1 ); i <= m - 2; i++ )
            end[i] = s - 2;

        for ( i = 0; i <= m - 2; i++ )
        {
            lj = NL_MAX( 0, i - q );
            hj = NL_MIN( m - 2, i + q );

            for ( j = lj; j <= hj; j++ )
            {
                lk = NL_MAX( start[i], start[j] );
                hk = NL_MIN( end[i], end[j] );

                for ( k = lk; k <= hk; k++ )
                {
                    NTN[i][j - i + vsb] += B[k][i - index[k]] * B[k][j - index[k]];
                }
            }
        }

        N_CreateRealMatrix( &cm, m - 2, m - 2, NTN, NL_MT_BANDED, vb );

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        /* Approximate rows in the v-direction */

        for ( j = 1; j <= s - 1; j++ )
        {
            error = N_BasisIEval( knv, 0, q, v[j], NL_LEFT, &ts[j - 1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisIEval( knv, m, q, v[j], NL_LEFT, &te[j - 1] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        if( n EQ 1 )
        {
            switch( ptp )
            {
                case NL_EPOINT:
                    for ( i = 0; i <= n; i++ )
                    {
                        N_PtToCPt( S[i][0], &Pw[i][0] );
                        N_PtToCPt( S[i][s], &Pw[i][m] );

                        for ( j = 1; j <= s - 1; j++ )
                        {
                            N_TranslateSum2Pts( S[i][j], -ts[j - 1], S[i][0], -te[j - 1], S[i][s], &Rk[j - 1] );
                        }

                        for ( j = 1; j <= m - 1; j++ )
                        {
                            N_CopyPt( NL_ZERO, &R[j - 1] );

                            lk = start[j - 1];
                            hk = end[j - 1];

                            for ( k = lk; k <= hk; k++ )
                            {
                                N_VectorBlendPt( B[k][j - index[k] - 1], Rk[k], &R[j - 1] );
                            }
                        }

                        error = N_RealMatrixForBack( &cm, (NL_VOID *)R, NL_EPOINT, Aw, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        for ( j = 0; j <= m - 2; j++ )
                            N_CopyCPt( Aw[j], &Pw[i][j + 1] );
                    }
                    break;

                case NL_HPOINT:
                    for ( i = 0; i <= n; i++ )
                    {
                        N_CopyCPt( Sw[i][0], &Pw[i][0] );
                        N_CopyCPt( Sw[i][s], &Pw[i][m] );

                        for ( j = 1; j <= s - 1; j++ )
                        {
                            N_TranslateSum2CPts( Sw[i][j], -ts[j - 1], Sw[i][0], -te[j - 1], Sw[i][s], &Rkw[j - 1] );
                        }

                        for ( j = 1; j <= m - 1; j++ )
                        {
                            N_CopyCPt( NL_CZERO, &Rw[j - 1] );

                            lk = start[j - 1];
                            hk = end[j - 1];

                            for ( k = lk; k <= hk; k++ )
                            {
                                N_VectorBlendCPt( B[k][j - index[k] - 1], Rkw[k], &Rw[j - 1] );
                            }
                        }

                        error = N_RealMatrixForBack( &cm, (NL_VOID *)Rw, NL_HPOINT, Aw, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        for ( j = 0; j <= m - 2; j++ )
                            N_CopyCPt( Aw[j], &Pw[i][j + 1] );
                    }
            }
        }
        else
        {
            for ( i = 0; i <= n; i++ )
            {
                N_CopyCPt( Qw[i][0], &Pw[i][0] );
                N_CopyCPt( Qw[i][s], &Pw[i][m] );

                for ( j = 1; j <= s - 1; j++ )
                {
                    N_TranslateSum2CPts( Qw[i][j], -ts[j - 1], Qw[i][0], -te[j - 1], Qw[i][s], &Rkw[j - 1] );
                }

                for ( j = 1; j <= m - 1; j++ )
                {
                    N_CopyCPt( NL_CZERO, &Rw[j - 1] );

                    lk = start[j - 1];
                    hk = end[j - 1];

                    for ( k = lk; k <= hk; k++ )
                    {
                        N_VectorBlendCPt( B[k][j - index[k] - 1], Rkw[k], &Rw[j - 1] );
                    }
                }

                error = N_RealMatrixForBack( &cm, (NL_VOID *)Rw, NL_HPOINT, Aw, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = 0; j <= m - 2; j++ )
                    N_CopyCPt( Aw[j], &Pw[i][j + 1] );
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFFUNCINTP: Surface function interpolation with arbitrary degrees    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  fitting  routine  computes a  surface  function of  arbitrary 
     degrees  interpolating a  given  set of  function  values at chosen 
     parameter values. If the output function is initialized to the NULL 
     function, memory is  allocated. Otherwise it is  checked if  enough  
     memory is  passed in. A  typical calling example is:

       NL_REAL       **F;
       NL_INDEX      n, m;
       NL_DEGREE     p, q;
       NL_PARAMETER  *u, *v;
       NL_SFUN       sfn;
       NL_STACKS     SG;
       ...
       (define array F, compute parameters u and v, and choose degrees);
       ...
       N_SFuncInitArrays(&sfn);
       N_FitSrfFuncInterp(F,n,m,p,q,u,v,&sfn,&SG);


   ACCESS:
   
     F   , input  ,  Function values to be interpolated
     n,m , input  ,  Highest indexes in F
     p,q , input  ,  Degrees of interpolating function
     u,v , input  ,  Parameters where F[i][j] are assumed
     sfn , output ,  Interpolating function
     SG  , input  ,  sfn's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfFuncInterp 
  (NL_REAL    ** F,     /* in : grid-of-points, sized:[n+1][m+1]  */ 
   NL_INDEX      n,     /* in : highest 1st index in F  */ 
   NL_INDEX      m,     /* in : highest 2nd index in F  */ 
   NL_DEGREE     p,     /* in : output function degree U  */ 
   NL_DEGREE     q,     /* in : output function degree V  */ 
   NL_PARAMETER *u,     /* in : Point U parameters, sized:[n+1][m+1]  */ 
   NL_PARAMETER *v,     /* in : Point V parameters, sized:[n+1][m+1]  */ 
   NL_SFUN      *sfn,   /* out: Interpolating function  */
   NL_STACKS    *SG )   /* in : sfn's memory stack      */ 
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfFuncInterp");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, ub, vb, usb, vsb;

    NL_REAL ** A, ** fuv, *N, *g, *h;

    NL_RMATRIX cm;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for function memory */

    if( p GT n OR q GT m )
        NL_ERROR( NL_INP_ERR );

    error = N_SFuncSizeArrays( sfn, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnotVectors( sfn, &knu, &knv );
    N_SrfFuncCntrlVal( sfn, &i, &j, &fuv );

    /* Compute knot vectors */

    N_FitCrvCalcKnotVector( u, n, p, knu );
    N_FitCrvCalcKnotVector( v, m, q, knv );

    /* Get auxiliary arrays */

    g = N_AllocReal1dArray( NL_MAX( n, m ), &SL );

    if( g EQ NULL )
        NL_QUIT;

    h = N_AllocReal1dArray( NL_MAX( n, m ), &SL );

    if( h EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( NL_MAX( p, q ), &SL );

    if( N EQ NULL )
        NL_QUIT;

    /* Interpolate in u-direction */

    if( p EQ 1 )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                fuv[i][j] = F[i][j];
        }
    }
    else
    {
        ub = 2 * p - 1;
        usb = p - 1;

        error = N_SetRealMatrix( &cm, n, n, NL_MT_BANDED, ub, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cm, &A );

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j < ub; j++ )
                A[i][j] = 0.0;
        }

        A[0][usb] = 1.0;
        A[n][usb] = 1.0;

        for ( i = 1; i < n; i++ )
        {
            error = N_BasisEval( knu, p, u[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            for ( k = 0; k <= p; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= m; j++ )
        {
            for ( i = 0; i <= n; i++ )
                g[i] = F[i][j];

            error = N_RealMatrixRightForBack( &cm, g, h );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= n; i++ )
                fuv[i][j] = h[i];
        }
    }

    /* Interpolate in v-direction */

    if( q GT 1 )
    {
        vb = 2 * q - 1;
        vsb = q - 1;

        error = N_SetRealMatrix( &cm, m, m, NL_MT_BANDED, vb, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cm, &A );

        for ( i = 0; i <= m; i++ )
        {
            for ( j = 0; j < vb; j++ )
                A[i][j] = 0.0;
        }

        A[0][vsb] = 1.0;
        A[m][vsb] = 1.0;

        for ( i = 1; i < m; i++ )
        {
            error = N_BasisEval( knv, q, v[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            for ( k = 0; k <= q; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cm );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                g[j] = fuv[i][j];

            error = N_RealMatrixRightForBack( &cm, g, h );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= m; j++ )
                fuv[i][j] = h[j];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFINTPBNDS: Surface interpolation to (nxm) points and boundaries     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface interpolating a given
     set of (nxm) grid of points with boundary constraints. That is, the
     points as well as the  boundaries must be  precisely  interpolated. 
     The boundary  curves  must be  non-rational and  compatible  in the 
     B-spline sense on input. Optionally the parameters may be passed in 
     (if not passed in they are computed internally). A typical  calling 
     example is:

       NL_POINT    **P;
       NL_INDEX    k, l;
       NL_REAL     *u, *v;
       NL_CURVE    **bndU, **bndV
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define array P, get bndU and bndV, and optionally u and v);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfInterpBoundary(P,k,l,bndU,bndV,u,v,&sur,&SG);
       N_FitSrfInterpBoundary(P,k,l,bndU,bndV,NULL,NULL,&sur,&SG);


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k,l  , input  ,  Highest indeces in u- and v-direction
     bndU , input  ,  Boundary curves in the u-direction:
                        bndU[0]: v=vmin boundary
                        bndU[1]: v=vmax boundary
                      THESE CURVES MUST BE NON-RATIONAL AND COMPATIBLE!
     bndV , input  ,  Boundary curves in the v-direction:
                        bndV[0]: u=umin boundary
                        bndV[1]: u=umax boundary
                      THESE CURVES MUST BE NON-RATIONAL AND COMPATIBLE!
     u,v  , input  ,  Parameters where points are assumed:
                        u = !NULL: u-parameters are passed in
                        u =  NULL: compute u-parameters internally
                        v = !NULL: v-parameters are passed in
                        v =  NULL: compute v-parameters internally
     sur  , output ,  Interpolating surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfInterpBoundary
  (NL_POINT  ** P,     /* in : grid-of-points, sized:[k+1][l+1] */
   NL_INDEX     k,     /* in : highest 1st index in k */
   NL_INDEX     l,     /* in : highest 2nd index in l */
   NL_CURVE  ** bndU,  /* in : U boundary curves, bndU[0]: v=vmin boundary, bndU[1]: v=vmax boundary */
   NL_CURVE  ** bndV,  /* in : V boundary curves, bndV[0]: u=umin boundary, bndV[1]: u=umax boundary */
   NL_REAL    * u,     /* in : opt U Param values for P, NULL= compute internally */
   NL_REAL    * v,     /* in : opt V Param values for P, NULL= compute internally */
   NL_SURFACE * sur,   /* out: Interpolating surface */
   NL_STACKS  * SG )   /* in : sur's memory stack    */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfInterpBoundary");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, r, s, ni, mi, ri, si, nc;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UI, *VI, *US, *VS, *ui, *vi, *ub = NULL, *ut = NULL, *vl = NULL, *vr = NULL, fac, alf, oma;

    NL_POINT *Q, *T;

    NL_CPOINT ** Sw, *Cw;

    NL_KNOTVECTOR *knu, *knv, *kbu, *kbv;

    NL_CURVE ** curV, ** curU;

    NL_STACKS SL;

    NL_PRIVATE NL_REAL per = 0.8;
    NL_PRIVATE NL_REAL pdt = 0.001;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check incoming data */

    if( N_IsCrvRat( bndU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( bndU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( bndV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( bndV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvsAreCombatible( bndU, 1 ) )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvsAreCombatible( bndV, 1 ) )
        NL_ERROR( NL_INP_ERR );

    /* Adjust parameters if needed */

    N_CrvGetKnotVector( bndU[0], &kbu );
    N_CrvGetKnotVector( bndV[0], &kbv );

    N_KnotVectorGetKnots( kbu, &r, &U );
    N_KnotVectorGetKnots( kbv, &s, &V );

    N_CrvGetDegree( bndU[0], &p );
    N_CrvGetDegree( bndV[0], &q );

    Q = N_AllocPt1dArray( NL_MAX( k, l ), &SL );

    if( Q EQ NULL )
        NL_QUIT;

    if( u NEQ NULL )
    {
        if( u[0]NEQ U[0]OR u[k]NEQ U[r] )
        {
            ui = N_AllocReal1dArray( k, &SL );

            if( ui EQ NULL )
                NL_QUIT;

            fac = (U[r] - U[0]) / (u[k] - u[0]);

            for ( i = 1; i < k; i++ )
                ui[i] = U[0] + fac * (u[i] - u[0]);

            ui[0] = U[0];
            ui[k] = U[r];
        }
        else
        {
            ui = u;
        }
    }
    else
    {
        ui = N_AllocReal1dArray( k, &SL );

        if( ui EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= k; i++ )
            N_VectorCopy( P[i][0], &Q[i] );

        error = N_CrvProjectPts( bndU[0], Q, k, pdt, &T, &ub, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= k; i++ )
            N_VectorCopy( P[i][l], &Q[i] );

        error = N_CrvProjectPts( bndU[1], Q, k, pdt, &T, &ut, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( v NEQ NULL )
    {
        if( v[0]NEQ V[0]OR v[l]NEQ V[s] )
        {
            vi = N_AllocReal1dArray( l, &SL );

            if( vi EQ NULL )
                NL_QUIT;

            fac = (V[s] - V[0]) / (v[l] - v[0]);

            for ( j = 1; j < l; j++ )
                vi[j] = V[0] + fac * (v[j] - v[0]);

            vi[0] = V[0];
            vi[l] = V[s];
        }
        else
        {
            vi = v;
        }
    }
    else
    {
        vi = N_AllocReal1dArray( l, &SL );

        if( vi EQ NULL )
            NL_QUIT;

        for ( j = 0; j <= l; j++ )
            N_VectorCopy( P[0][j], &Q[j] );

        error = N_CrvProjectPts( bndV[0], Q, l, pdt, &T, &vl, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= l; j++ )
            N_VectorCopy( P[k][j], &Q[j] );

        error = N_CrvProjectPts( bndV[1], Q, l, pdt, &T, &vr, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Interpolate in the v-direction first */

    knv = NULL;
    error = N_KnotsCopy( kbv, &knv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    curV = N_AllocArrayCrvPtrs( k, &SL );

    if( curV EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        curV[i] = N_AllocCrv( &SL );

        if( curV[i]EQ NULL )
            NL_QUIT;
    }

    N_CrvInitArrays( curV[0] );
    error = N_CrvCopy( bndV[0], curV[0], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( curV[k] );
    error = N_CrvCopy( bndV[1], curV[k], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    fac = 1.0 / k;

    for ( i = 1; i < k; i++ )
    {
        alf = i * fac;
        oma = 1.0 - alf;

        if( v EQ NULL )
        {
            for ( j = 0; j <= l; j++ )
                vi[j] = oma * vl[j] + alf * vr[j];
        }

        N_CrvInitArrays( curV[i] );
        error = N_FitCrvKnotsAndTangents( P[i], l, vi, NULL, NULL, NL_TANGENT, &knv, per, q, curV[i], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curV, k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curV[0], &si, &VI );
    N_CrvGetArraySizes( curV[0], &mi, &si );

    /* Now interpolate in the u-direction */

    knu = NULL;
    error = N_KnotsCopy( kbu, &knu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    curU = N_AllocArrayCrvPtrs( mi, &SL );

    if( curU EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= mi; j++ )
    {
        curU[j] = N_AllocCrv( &SL );

        if( curU[j]EQ NULL )
            NL_QUIT;
    }

    N_CrvInitArrays( curU[0] );
    error = N_CrvCopy( bndU[0], curU[0], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( curU[mi] );
    error = N_CrvCopy( bndU[1], curU[mi], &SL );

    if( error EQ NL_YES )
        NL_OUT;

    fac = 1.0 / mi;

    for ( j = 1; j < mi; j++ )
    {
        alf = j * fac;
        oma = 1.0 - alf;

        for ( i = 0; i <= k; i++ )
        {
            N_CrvGetCPts( curV[i], &nc, &Cw );
            N_CPtToPtEuclid( Cw[j], &Q[i] );
        }

        if( u EQ NULL )
        {
            for ( i = 0; i <= k; i++ )
                ui[i] = oma * ub[i] + alf * ut[i];
        }

        N_CrvInitArrays( curU[j] );
        error = N_FitCrvKnotsAndTangents( Q, k, ui, NULL, NULL, NL_TANGENT, &knu, per, p, curU[j], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Make curves compatible */

    error = N_CrvsMakeCompatible( curU, mi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( curU[0], &ri, &UI );
    N_CrvGetArraySizes( curU[0], &ni, &ri );

    /* Get output surface */

    error = N_SrfSizeArrays( sur, ni, mi, p, q, ri, si, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );

    for ( j = 0; j <= mi; j++ )
    {
        N_CrvGetCPts( curU[j], &ni, &Cw );

        for ( i = 0; i <= ni; i++ )
            N_CopyCPt( Cw[i], &Sw[i][j] );
    }

    for ( i = 0; i <= ri; i++ )
        US[i] = UI[i];

    for ( j = 0; j <= si; j++ )
        VS[j] = VI[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFINTPSHAPE: Interpolate points based on surface shaping              */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine  interpolates a set of points, boundary curves
     and, if given, any  number of  cross-derivatives  by shaping a base  
     surface, interpolating  the boundary curves and  cross-derivatives, 
     so that it passes through the given points. This routine works best 
     if the points represent a simple surface as  in reverse engineering  
     after  the point  set has  been  segmented  into simple  parts. The 
     points can be very  irregularly spaced as long as they  represent a
     simple shape. The method  first computes a base surface  which then 
     is forced to pass through the given points via constrained shaping.
     The  base surface  is a bi-linearly  or bi-cubically  blended Coons 
     patch, interpolating  the boundary curves  and any number of cross-
     boundary derivatives. A typical calling example is:

       NL_SURFACE  sur;
       NL_CURVE    **curU, **curV, **derU, **derV;
       NL_POINT    *P;
       NL_INDEX    k;
       NL_STACKS   SC, SG;
       ...
       (get points P, boundaries and cross-derivatives);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfInterpShape(P,k,curU,curV,derU,derV,NL_YES,&sur,&SC,&SG);
       N_FitSrfInterpShape(P,k,curU,curV,NULL,NULL,NL_YES,&sur,&SC,&SG);

     THE  DERIVATIVES  MUST BE NL_TWIST  COMPATIBLE AT  EACH CORNER. IF THE 
     BOUNDARIES AND THE  DERIVATIVES ARE NOT  COMPATIBLE IN THE B-SPLINE
     SENSE, THEY WILL BE MADE SO, I.E. THE IMPUT DATA WILL BE DESTROYED!
     THE BOUNDARY CURVES AND CROSS-DERIVATIVES MUST BE NON-RATIONAL!


   ACCESS:
   
     P    , input  ,  Random points output surface must interpolate
     k    , input  ,  Highest index in P
     curU , in/out ,  U-boundaries:
                       curU[0]: v=vmin boundary
                       curU[1]: v=vmax boundary
                      MUST BE NON-RATIONAL!
     curV , in/out ,  V-boundaries:
                       curV[0]: u=umin boundary
                       curV[1]: u=umax boundary
                      MUST BE NON-RATIONAL!
     derU , in/out ,  Cross-boundary derivatives:
                        = NULL: None past in
                       != NULL: One or both passed in
                        derU[0]: cross-derivative across u=umin boundary
                        derU[1]: cross-derivative across u=umax boundary
                      derU[0] or derU[1] can be individually NULL, indi-
                      cating the  absence of  cross-derivative along the 
                      edge it belongs. MUST BE NON-RATIONAL!
     derV , in/out ,  Cross-boundary derivatives:
                        = NULL: None past in
                       != NULL: One or both passed in
                        derV[0]: cross-derivative across v=vmin boundary
                        derV[1]: cross-derivative across v=vmax boundary
                      derV[0] or derV[1] can be individually NULL, indi-
                      cating the  absence of  cross-derivative along the 
                      edge it belongs. MUST BE NON-RATIONAL!
     lfl  , input  ,  Flag:
                        NL_YES: localize surface before shaping
                        NL_NO : do not localize, let routine refine surface
                             globally
                      LOCALIZATION PROVIDES (1) SOLVABLE SYSTEM OF EQUA-
                      TIIONS, AND (2) LOCAL  EFFECTS (WHICH MAY BE UNDE-
                      SIRABLE). GLOBALIZATION   PROVIDES   MORE   GLOBAL 
                      CHANGES, HOWEVER, THE  SYSTEM OF EQUATIONS MAY NOT
                      BE SOLVABLE! RECOMMENDED DEFAULT: lfl = NL_YES!!
     sur  , output ,  Surface interpolating P[i], i=0,...,k, curU,  curV,
                      derU and derV
     SC   , input  ,  Stack of  curU, curV, derU  and derV. These curves
                      must all be on the same stack SC 
     SG   , input  ,  sur's stack
    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfInterpShape 
  (NL_POINT   * P,     /* in : cloud-of-points, sized:[k+1] */                                                                       
   NL_INDEX     k,     /* in : highest index in P */                                                                                 
   NL_CURVE  ** curU,  /* i/o: U bndry position cstrn, curU[0]: v=vmin boundary, curU[1]: v=vmax boundary */                         
   NL_CURVE  ** curV,  /* i/o: V bndry position cstrn, curV[0]: u=umin boundary, curV[1]: u=umax boundary */                         
   NL_CURVE  ** derU,  /* i/o: opt bndry U-cross-tangent cstrn, derU[0]: u=umin boundary, curU[1]: u=umax boundary, NULL to ignore */
   NL_CURVE  ** derV,  /* i/o: opt bndry V-cross-tangent cstrn, derV[0]: v=vmin boundary, curV[1]: v=vmax boundary, NULL to ignore */
   NL_FLAG      lfl,   /* in : NL_YES = localize surface before shaping - always solvable - recommended default value!!! */
                       /*      NL_NO  = refine surface globally - may lead to an unsolvable set of equations */            
   NL_SURFACE * sur,   /* out: approximating surface */            
   NL_STACKS  * SC,    /* in : curU, curV, derU, derV memory stack. */            
   NL_STACKS  * SG )   /* in : sur's memory stack */                              
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfInterpShape");

    NL_FLAG error = NL_NO, L = NL_NO, R = NL_NO, B = NL_NO, T = NL_NO;

    NL_INDEX rc0, rc1, sc0, sc1, rd0 = 0, rd1 = 0, sd0 = 0, sd1 = 0, rs, ss, code, dsu, deu, dsv, dev, i;

    NL_REAL *UC0, *UC1, *VC0, *VC1, *UD0 = NULL, *UD1 = NULL, *VD0 = NULL, *VD1 = NULL, *US, *VS;

    NL_POINT ** SD, ** SE, Ts[2], Te[2];

    NL_VECTOR D00_u, D00_v, D10_u, D10_v, D01_u, D01_v, D11_u, D11_v;

    NL_SURFACE surB, surS;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input data */

    if( N_IsCrvRat( curU[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curU[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[0] ) )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( curV[1] ) )
        NL_ERROR( NL_INP_ERR );

    if( derU NEQ NULL )
    {
        if( derU[0]NEQ NULL )
            if( N_IsCrvRat( derU[0] ) )
                NL_ERROR( NL_INP_ERR );

        if( derU[1]NEQ NULL )
            if( N_IsCrvRat( derU[1] ) )
                NL_ERROR( NL_INP_ERR );
    }

    if( derV NEQ NULL )
    {
        if( derV[0]NEQ NULL )
            if( N_IsCrvRat( derV[0] ) )
                NL_ERROR( NL_INP_ERR );

        if( derV[1]NEQ NULL )
            if( N_IsCrvRat( derV[1] ) )
                NL_ERROR( NL_INP_ERR );
    }

    /* Get locals and allocate memory */

    dsu = deu = dsv = dev = 0;

    N_CrvGetKnots( curU[0], &rc0, &UC0 );
    N_CrvGetKnots( curU[1], &rc1, &UC1 );
    N_CrvGetKnots( curV[0], &sc0, &VC0 );
    N_CrvGetKnots( curV[1], &sc1, &VC1 );

    if( derU NEQ NULL )
    {
        if( derU[0]NEQ NULL )
        {
            L = NL_YES;
            dsu = 1;
            N_CrvGetKnots( derU[0], &rd0, &UD0 );
        }

        if( derU[1]NEQ NULL )
        {
            R = NL_YES;
            deu = 1;
            N_CrvGetKnots( derU[1], &rd1, &UD1 );
        }
    }

    if( derV NEQ NULL )
    {
        if( derV[0]NEQ NULL )
        {
            B = NL_YES;
            dsv = 1;
            N_CrvGetKnots( derV[0], &sd0, &VD0 );
        }

        if( derV[1]NEQ NULL )
        {
            T = NL_YES;
            dev = 1;
            N_CrvGetKnots( derV[1], &sd1, &VD1 );
        }
    }

    code = (L EQ NL_YES) << 3 | (R EQ NL_YES) << 2 | (B EQ NL_YES) << 1 | (T EQ NL_YES);

    /* Get bilinear Coons patch first */

    N_SrfInitArrays( &surB );
    error = N_CreateCoonsSrf( curU, curV, &surB, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* No or all cross-boundaries are given */

    if( code EQ 0 )
        goto SHAPE;

    if( code EQ 15 )
        goto BCC;

    /* Allocate memory */

    SD = N_AllocPt2dArray( 1, 1, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    SE = N_AllocPt2dArray( 1, 1, &SL );

    if( SE EQ NULL )
        NL_QUIT;

    if( derU EQ NULL )
    {
        derU = N_AllocArrayCrvPtrs( 1, SC );

        if( derU EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= 1; i++ )
        {
            derU[i] = N_AllocCrv( SC );

            if( derU[i]EQ NULL )
                NL_QUIT;
        }
    }

    if( derV EQ NULL )
    {
        derV = N_AllocArrayCrvPtrs( 1, SC );

        if( derV EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= 1; i++ )
        {
            derV[i] = N_AllocCrv( SC );

            if( derV[i]EQ NULL )
                NL_QUIT;
        }
    }

    if( derU NEQ NULL )
    {
        if( derU[0]EQ NULL )
        {
            derU[0] = N_AllocCrv( SC );

            if( derU[0]EQ NULL )
                NL_QUIT;
        }

        if( derU[1]EQ NULL )
        {
            derU[1] = N_AllocCrv( SC );

            if( derU[1]EQ NULL )
                NL_QUIT;
        }
    }

    if( derV NEQ NULL )
    {
        if( derV[0]EQ NULL )
        {
            derV[0] = N_AllocCrv( SC );

            if( derV[0]EQ NULL )
                NL_QUIT;
        }

        if( derV[1]EQ NULL )
        {
            derV[1] = N_AllocCrv( SC );

            if( derV[1]EQ NULL )
                NL_QUIT;
        }
    }

    /* Compute boundary derivatives */

    error = N_CrvDerivs( curU[0], UC0[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curU[0], UC0[rc0], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D00_u );
    N_VectorCopy( Te[1], &D10_u );

    error = N_CrvDerivs( curU[1], UC1[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curU[1], UC1[rc1], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D01_u );
    N_VectorCopy( Te[1], &D11_u );

    error = N_CrvDerivs( curV[0], VC0[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV[0], VC0[sc0], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D00_v );
    N_VectorCopy( Te[1], &D01_v );

    error = N_CrvDerivs( curV[1], VC1[0], NL_LEFT, 1, Ts );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV[1], VC1[sc1], NL_LEFT, 1, Te );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCopy( Ts[1], &D10_v );
    N_VectorCopy( Te[1], &D11_v );

    /* Compute base surface without cross derivatives */

    N_SrfInitArrays( &surS );
    error = N_SrfShapeInterp( &surB, P, k, 0, 0, 0, 0, lfl, &surS, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnots( &surS, &rs, &ss, &US, &VS );

    /* One cross-boundary is missing */

    if( code EQ 14 )
    {
        /* Top cross-boundary is missing */

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 7 )
    {
        /* Left cross-boundary is missing */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 13 )
    {
        /* Bottom cross-boundary is missing */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 11 )
    {
        /* Right cross-boundary is missing */

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }

    /* Two cross-derivatives are missing */

    if( code EQ 12 )
    {
        /* Bottom and top cross-derivatives are missing */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 3 )
    {
        /* Left and right cross-derivatives are missing */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 10 )
    {
        /* Upper right twist is missing */

        error = N_SrfDerivs( &surS, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 6 )
    {
        /* Upper left twist is missing */

        error = N_SrfDerivs( &surS, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 5 )
    {
        /* Lower left twist is missing */

        error = N_SrfDerivs( &surS, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 9 )
    {
        /* Lower right twist is missing */

        error = N_SrfDerivs( &surS, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }

    /* Three cross-derivatives are missing */

    if( code EQ 8 )
    {
        /* Lower and upper right twists are missing */

        error = N_SrfDerivs( &surS, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        N_CrvInitArrays( derU[1] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_RIGHT, derU[1], SC );

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[0], UD0[rd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Te[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 2 )
    {
        /* Upper left and right twists are missing */

        error = N_SrfDerivs( &surS, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[rs], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Te[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[0], VD0[sd0], NL_LEFT, 1, Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Te[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        N_CrvInitArrays( derV[1] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_TOP, derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 4 )
    {
        /* Lower left and upper right twists are missing */

        error = N_SrfDerivs( &surS, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[0], VS[ss], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derV[0] );
        error = N_CreateDerivField( &surS, NL_BOTTOM, &D00_v, &D10_v, &Ts[1], &Te[1], derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        N_CrvInitArrays( derU[0] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_LEFT, derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get top cross-derivative */

        error = N_CrvDerivs( derU[1], UD1[rd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Ts[1] );

        N_CrvInitArrays( derV[1] );
        error = N_CreateDerivField( &surS, NL_TOP, &D01_v, &D11_v, &Ts[1], &Te[1], derV[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }
    else if( code EQ 1 )
    {
        /* Lower left and right twists are missing */

        error = N_SrfDerivs( &surS, US[0], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SD );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_SrfDerivs( &surS, US[rs], VS[0], NL_LEFT, NL_LEFT, NL_FALSE, 1, 1, SE );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get left cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[0], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SD[1][1], &Ts[1] );

        N_CrvInitArrays( derU[0] );
        error = N_CreateDerivField( &surS, NL_LEFT, &D00_u, &D01_u, &Ts[1], &Te[1], derU[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get bottom cross-derivative */

        N_CrvInitArrays( derV[0] );
        error = N_CrossBoundDerivCrvNonRatSrf( &surS, NL_BOTTOM, derV[0], SC );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get right cross-derivative */

        error = N_CrvDerivs( derV[1], VD1[sd1], NL_LEFT, 1, Te );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( SE[1][1], &Ts[1] );

        N_CrvInitArrays( derU[1] );
        error = N_CreateDerivField( &surS, NL_RIGHT, &D10_u, &D11_u, &Ts[1], &Te[1], derU[1], SC );

        if( error EQ NL_YES )
            NL_OUT;

        goto BCC;
    }

    /* Make bi-cubic Coons patch */

    BCC:

    N_SrfInitArrays( &surB );
    error = N_CreateCoonsBoundaryCrvs( curV[0], derU[0], curV[1], derU[1], curU[0], derV[0], curU[1], derV[1], NL_YES, &surB, SC, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Shape base surface to pass through the given points */

    SHAPE:

    error = N_SrfShapeInterp( &surB, P, k, dsu, deu, dsv, dev, lfl, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFINTPTAN: Surface interpolation with tangent constraints           */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface interpolating a given
     set of points. Tangent or derivative constraints may also be speci-
     fied  along the boundaries. Optionally, knot  vectors and  interpo-
     lation  parameters  may be input. If the  output surface is initia-
     lized to the NULL surface,  memory is allocated  locally. Otherwise 
     it is  checked if  enough  memory is  passed in. A typical  calling 
     example is:

       NL_POINT       **P;
       NL_VECTOR      ***Tu, ***Tv;
       NL_INDEX       n, m;
       NL_DEGREE      p, q;
       NL_PARAMETER   *u, *v;
       NL_KNOTVECTOR  *knu, *knv;
       NL_SURFACE     sur;
       NL_STACKS      SG;
       ...
       (define arrays P, Tu and Tv, and optionally load u, v, knu
        and knv);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfInterpTangents(P,n,m,u,NULL,knu,NULL,p,q,Tu,Tv,NL_TANGENT,&sur,&SG);

     If no tangents are defined in  the u- or v-direction (or both), the
     routine will  check if the data indicates a closed surface and will
     ensure NL_C1 closure.


   ACCESS:
   
     P   , input  ,  Points to be interpolated; P[i][j], i=0,...,n, j=0,
                     ...m.
     n,m , input  ,  Highest indexes in P
     u,v , input  ,  Parameters corresponding to data points; u[i], i=0,
                     ...,n, v[j], j=0,...,m:
                       != NULL: parameters are  passed in (either u or v
                                or both) 
                        = NULL: no parameters are passed in (no  u or no
                                v or neither u nor v), must be computed
     knu , input  ,  Knot vector in u-direction:
                       != NULL: knu passed in
                        = NULL: no knu passed in, must be computed. NOTE
                                THAT knu=NULL MUST HOLD IF u=NULL.
     knv , input  ,  Knot vector in v-direction:
                       != NULL: knv passed in
                        = NULL: no knv passed in, must be computed. NOTE
                                THAT knv=NULL MUST HOLD IF v=NULL.
     p,q , input  ,  Degrees of interpolating surface
     Tu  , input  ,  Tangent  vectors/derivatives  in  the   u-direction 
                     along  the  umin  and  umax  boundaries.  Tu[i][j], 
                     i=0,1; j=0,...,m, is a  pointer to the  tangents or
                     derivatives at the j-th point  along the umin (i=0)
                     or along the umax (i=1) boundary:
                       Tu[i][j]  = NULL: no tangent/derivative  supplied
                                         at the point (i,j)
                       Tu[i][j] != NULL: tangent/derivative supplied
                       Tu        = NULL: no tangent/derivative  supplied
                                         on either boundary
     Tv  , input  ,  Tangent  vectors/derivatives  in  the   v-direction 
                     along  the  vmin  and  vmax  boundaries.  Tv[k][l], 
                     k=0,...,n; l=0,1, is a  pointer to the  tangents or
                     derivatives at the k-th point  along the vmin (l=0)
                     or along the vmax (l=1) boundary:
                       Tv[k][l]  = NULL: no tangent/derivative  supplied
                                         at the point (k,l)
                       Tv[k][l] != NULL: tangent/derivative supplied
                       Tv        = NULL: no tangent/derivative  supplied
                                         on either boundary
     tnf , input  ,  Flag:
                       NL_TANGENT   : Tangent directions are given; magnitudes may change
                       NL_DERIVATIVE: Vectors are derivatives,  i.e. magnitudes
                                    will not change. THIS OPTION IS
                                   AVAILABLE ONLY IF PARAMETERS AND KNOT
                                   VECTORS ARE SUPPLIED.
     sur , output ,  Interpolating surface
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfInterpTangents
  (NL_POINT    ** P,     /* in : sample points, sized:[n+1][m+1] */
   NL_INDEX       n,     /* in : max 1st P index */
   NL_INDEX       m,     /* in : max 2nd P index */
   NL_PARAMETER  *u,     /* in : u param values, sized[n+1], NULL=find param values */
   NL_PARAMETER  *v,     /* in : v param values, sized[m+1], NULL=find param values */
   NL_KNOTVECTOR *knu,   /* in : opt U KnotVector, NULL = compute knu, must be NULL when u is NULL */
   NL_KNOTVECTOR *knv,   /* in : opt V KnotVector, NULL = compute knv, must be NULL when v is NULL  */
   NL_DEGREE      p,     /* in : output sur degree U */
   NL_DEGREE      q,     /* in : output sur degree V */
   NL_VECTOR  *** Tu,    /* in : opt U Tangent values along surface boundaries, NULL to ignore */
   NL_VECTOR  *** Tv,    /* in : opt V Tangent values along surface boundaries, NULL to ignore */
   NL_FLAG        tnf,   /* in : tangent flag: NL_TANGENT   : Tangent directions are given; magnitudes may change */
                         /*                    NL_DERIVATIVE: Vectors are derivatives,  magnitudes fixed.         */
                         /*                         only availablbe when parameters and knot vectors are supplied */
   NL_SURFACE    *sur,   /* out: approximated surface */
   NL_STACKS     *SG )   /* in : sur's memory stack */
{                         
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfInterpTangents");

    NL_FLAG tu, tv, le, ri, error = NL_NO;

    NL_BOOLEAN u_closed = NL_FALSE, v_closed = NL_FALSE;

    NL_INDEX i, j, k, l, ns = 0, ms = 0, rs, ss, rk, sk, bw, sbw;

    NL_REAL ** A, *N, *Us, *Vs, *Uk, *Vk, len;

    NL_PARAMETER *ul, *vl;

    NL_VECTOR ** Tul, ** Tvl;

    NL_POINT *Q;

    NL_CPOINT ** Pw, ** Pu, ** Pv, ** Pt, *Qw, *Rw;

    NL_RMATRIX cun, cul, cur, cub, cvn, cvl, cvr, cvb;

    NL_KNOTVECTOR *kus, *kvs, *kun, *kul, *kur, *kub, *kvn, *kvl, *kvr, *kvb;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    if( u EQ NULL AND knu NEQ NULL )
        NL_ERROR( NL_INP_ERR );

    if( v EQ NULL AND knv NEQ NULL )
        NL_ERROR( NL_INP_ERR );

    if( tnf EQ NL_DERIVATIVE AND( u EQ NULL OR v EQ NULL ) )
        NL_ERROR( NL_INP_ERR );

    /* Get local memory */

    Tul = N_AllocPt2dArray( 1, m, &SL );

    if( Tul EQ NULL )
        NL_QUIT;

    Tvl = N_AllocPt2dArray( n, 1, &SL );

    if( Tvl EQ NULL )
        NL_QUIT;

    ul = N_AllocReal1dArray( n, &SL );

    if( ul EQ NULL )
        NL_QUIT;

    vl = N_AllocReal1dArray( m, &SL );

    if( vl EQ NULL )
        NL_QUIT;

    Q = N_AllocPt1dArray( NL_MAX( n, m ), &SL );

    if( Q EQ NULL )
        NL_QUIT;

    Qw = N_AllocCPt1dArray( NL_MAX( n + 2, m + 2 ), &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( NL_MAX( n + 2, m + 2 ), &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    kun = N_AllocKnotVectorAndArray( n + p + 1, &SL );

    if( kun EQ NULL )
        NL_QUIT;

    kul = N_AllocKnotVectorAndArray( n + p + 2, &SL );

    if( kul EQ NULL )
        NL_QUIT;

    kur = N_AllocKnotVectorAndArray( n + p + 2, &SL );

    if( kur EQ NULL )
        NL_QUIT;

    kub = N_AllocKnotVectorAndArray( n + p + 3, &SL );

    if( kub EQ NULL )
        NL_QUIT;

    kvn = N_AllocKnotVectorAndArray( m + q + 1, &SL );

    if( kvn EQ NULL )
        NL_QUIT;

    kvl = N_AllocKnotVectorAndArray( m + q + 2, &SL );

    if( kvl EQ NULL )
        NL_QUIT;

    kvr = N_AllocKnotVectorAndArray( m + q + 2, &SL );

    if( kvr EQ NULL )
        NL_QUIT;

    kvb = N_AllocKnotVectorAndArray( m + q + 3, &SL );

    if( kvb EQ NULL )
        NL_QUIT;

    N = N_AllocReal1dArray( NL_MAX( p, q ), &SL );

    if( N EQ NULL )
        NL_QUIT;

    /* Get parameters */

    if( u EQ NULL )
    {    
        error = N_FitSrfInterpParams( (NL_VOID ** )P, n, m, NL_EPOINT, NL_CHORDLENGTH, NL_UDIR, ul );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        for ( i = 0; i <= n; i++ )
            ul[i] = u[i];
    }

    if( v EQ NULL )
    {
        error = N_FitSrfInterpParams( (NL_VOID ** )P, n, m, NL_EPOINT, NL_CHORDLENGTH, NL_VDIR, vl );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        for ( j = 0; j <= m; j++ )
            vl[j] = v[j];
    }

    /* Set flags */

    le = ri = tu = NL_NO;

    if( Tu NEQ NULL )
    {
        for ( j = 0; j <= m; j++ )
        {
            if( Tu[0][j]NEQ NULL )
                le = NL_YES;

            if( Tu[1][j]NEQ NULL )
                ri = NL_YES;

            if( le EQ NL_YES AND ri EQ NL_YES )
                break;
        }

        if( le EQ NL_YES AND ri EQ NL_NO )
            tu = NL_LEFT;

        else if( le EQ NL_NO AND ri EQ NL_YES )
            tu = NL_RIGHT;

        else if( le EQ NL_YES AND ri EQ NL_YES )
            tu = NL_BOTH;
    }
    else
    {
        u_closed = N_2dPtSetIsClosed( (NL_VOID ** )P, n, m, NL_EPOINT, NL_UDIR, NL_MTOL );

        if( u_closed )
        {
            tu = NL_BOTH;

            for ( j = 0; j <= m; j++ ) /* close with NL_C1 continuity */
            {
                N_VectorDiff( P[1][j], P[n - 1][j], &Tul[0][j] );
                N_VectorCopy( Tul[0][j], &Tul[1][j] );
            }
        }
    }

    le = ri = tv = NL_NO;

    if( Tv NEQ NULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( Tv[i][0]NEQ NULL )
                le = NL_YES;

            if( Tv[i][1]NEQ NULL )
                ri = NL_YES;

            if( le EQ NL_YES AND ri EQ NL_YES )
                break;
        }

        if( le EQ NL_YES AND ri EQ NL_NO )
            tv = NL_LEFT;

        else if( le EQ NL_NO AND ri EQ NL_YES )
            tv = NL_RIGHT;

        else if( le EQ NL_YES AND ri EQ NL_YES )
            tv = NL_BOTH;
    }
    else
    {
        v_closed = N_2dPtSetIsClosed( (NL_VOID ** )P, n, m, NL_EPOINT, NL_VDIR, NL_MTOL );

        if( v_closed )
        {
            tv = NL_BOTH;

            for ( i = 0; i <= n; i++ ) /* close with NL_C1 continuity */
            {
                N_VectorDiff( P[i][1], P[i][m - 1], &Tvl[i][0] );
                N_VectorCopy( Tvl[i][0], &Tvl[i][1] );
            }
        }
    }

    /* Check for surface memory */

    if( tu EQ NL_NO )
        ns = n;

    else if( tu EQ NL_LEFT )
        ns = n + 1;

    else if( tu EQ NL_RIGHT )
        ns = n + 1;

    else if( tu EQ NL_BOTH )
        ns = n + 2;

    if( tv EQ NL_NO )
        ms = m;

    else if( tv EQ NL_LEFT )
        ms = m + 1;

    else if( tv EQ NL_RIGHT )
        ms = m + 1;

    else if( tv EQ NL_BOTH )
        ms = m + 2;

    if( p GT ns OR q GT ms )
        NL_ERROR( NL_INP_ERR );

    rs = ns + p + 1;
    ss = ms + q + 1;

    error = N_SrfSizeArrays( sur, ns, ms, p, q, rs, ss, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsKnotVectorAndKnots( sur, &Pw, &kus, &kvs, &Us, &Vs );

    Pu = N_AllocCPt2dArray( ns, ms, &SL );

    if( Pu EQ NULL )
        NL_QUIT;

    Pv = N_AllocCPt2dArray( ns, ms, &SL );

    if( Pv EQ NULL )
        NL_QUIT;

    Pt = N_AllocCPt2dArray( ns, ms, &SL );

    if( Pt EQ NULL )
        NL_QUIT;

    /* Get output knot vectors */

    if( knu NEQ NULL )
    {
        N_KnotVectorGetKnots( knu, &rk, &Uk );

        if( rk NEQ rs )
            NL_ERROR( NL_INP_ERR );

        for ( i = 0; i <= rk; i++ )
            Us[i] = Uk[i];
    }
    else
    {
        if( tu EQ NL_NO )
            N_FitCrvCalcKnotVector( ul, n, p, kus );

        else if( tu EQ NL_LEFT )
            N_FitCalcKnotVectorDeriv( ul, n, p, NL_START, kus );

        else if( tu EQ NL_RIGHT )
            N_FitCalcKnotVectorDeriv( ul, n, p, NL_END, kus );

        else if( tu EQ NL_BOTH )
            N_FitCalcKnotVectorEndDerivs( ul, n, p, kus );
    }

    if( knv NEQ NULL )
    {
        N_KnotVectorGetKnots( knv, &sk, &Vk );

        if( sk NEQ ss )
            NL_ERROR( NL_INP_ERR );

        for ( j = 0; j <= sk; j++ )
            Vs[j] = Vk[j];
    }
    else
    {
        if( tv EQ NL_NO )
            N_FitCrvCalcKnotVector( vl, m, q, kvs );

        else if( tv EQ NL_LEFT )
            N_FitCalcKnotVectorDeriv( vl, m, q, NL_START, kvs );

        else if( tv EQ NL_RIGHT )
            N_FitCalcKnotVectorDeriv( vl, m, q, NL_END, kvs );

        else if( tv EQ NL_BOTH )
            N_FitCalcKnotVectorEndDerivs( vl, m, q, kvs );
    }

    /* Check special case */

    if( tu EQ NL_NO AND tv EQ NL_NO )
    {
        error = N_FitSrfToPtsKnots( (NL_VOID ** )P, n, m, NL_EPOINT, ul, vl, kus, kvs, p, q, sur, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Copy vectors */

    if( Tu NEQ NULL )
    {
        for ( j = 0; j <= m; j++ )
        {
            if( Tu[0][j]NEQ NULL )
                N_VectorCopy( *Tu[0][j], &Tul[0][j] );

            if( Tu[1][j]NEQ NULL )
                N_VectorCopy( *Tu[1][j], &Tul[1][j] );
        }
    }

    if( Tv NEQ NULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( Tv[i][0]NEQ NULL )
                N_VectorCopy( *Tv[i][0], &Tvl[i][0] );

            if( Tv[i][1]NEQ NULL )
                N_VectorCopy( *Tv[i][1], &Tvl[i][1] );
        }
    }

    /* Set magnitudes */

    if( tnf EQ NL_TANGENT )
    {
        if( Tu NEQ NULL OR u_closed )
        {
            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= n; i++ )
                    N_CopyPt( P[i][j], &Q[i] );

                N_DistPolygon( Q, n, &len );
                len /= (Us[rs] - Us[0]);

                if( u_closed )
                {
                    error = N_VectorNormalizeRef( &Tul[0][j] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorScale( Tul[0][j], len, &Tul[0][j] );

                    error = N_VectorNormalizeRef( &Tul[1][j] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorScale( Tul[1][j], len, &Tul[1][j] );
                }
                else
                {
                    if( Tu[0][j]NEQ NULL )
                    {
                        error = N_VectorNormalizeRef( &Tul[0][j] );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_VectorScale( Tul[0][j], len, &Tul[0][j] );
                    }

                    if( Tu[1][j]NEQ NULL )
                    {
                        error = N_VectorNormalizeRef( &Tul[1][j] );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_VectorScale( Tul[1][j], len, &Tul[1][j] );
                    }
                }
            }
        }

        if( Tv NEQ NULL OR v_closed )
        {
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                    N_CopyPt( P[i][j], &Q[j] );

                N_DistPolygon( Q, m, &len );
                len /= (Vs[ss] - Vs[0]);

                if( v_closed )
                {
                    error = N_VectorNormalizeRef( &Tvl[i][0] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorScale( Tvl[i][0], len, &Tvl[i][0] );

                    error = N_VectorNormalizeRef( &Tvl[i][1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_VectorScale( Tvl[i][1], len, &Tvl[i][1] );
                }
                else
                {
                    if( Tv[i][0]NEQ NULL )
                    {
                        error = N_VectorNormalizeRef( &Tvl[i][0] );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_VectorScale( Tvl[i][0], len, &Tvl[i][0] );
                    }

                    if( Tv[i][1]NEQ NULL )
                    {
                        error = N_VectorNormalizeRef( &Tvl[i][1] );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_VectorScale( Tvl[i][1], len, &Tvl[i][1] );
                    }
                }
            }
        }
    }

    /* Get knot vectors for local fitting */

    /* U-knot vectors */

    N_KnotVectorGetKnots( kun, &rk, &Uk );

    if( tu EQ NL_NO )
    {
        for ( i = 0; i <= rs; i++ )
            Uk[i] = Us[i];
    }
    else if( tu EQ NL_LEFT )
    {
        j = -1;

        for ( i = 0; i <= p; i++ )
            Uk[++j] = Us[i];

        for ( i = p + 2; i <= rs; i++ )
            Uk[++j] = Us[i];
    }
    else if( tu EQ NL_RIGHT )
    {
        j = -1;

        for ( i = 0; i <= ns - 1; i++ )
            Uk[++j] = Us[i];

        for ( i = ns + 1; i <= rs; i++ )
            Uk[++j] = Us[i];
    }
    else if( tu EQ NL_BOTH )
    {
        j = -1;

        for ( i = 0; i <= p; i++ )
            Uk[++j] = Us[i];

        for ( i = p + 2; i <= ns - 1; i++ )
            Uk[++j] = Us[i];

        for ( i = ns + 1; i <= rs; i++ )
            Uk[++j] = Us[i];
    }

    N_KnotVectorGetKnots( kul, &rk, &Uk );

    if( tu EQ NL_LEFT )
    {
        for ( i = 0; i <= rs; i++ )
            Uk[i] = Us[i];
    }
    else if( tu EQ NL_BOTH )
    {
        j = -1;

        for ( i = 0; i <= ns - 1; i++ )
            Uk[++j] = Us[i];

        for ( i = ns + 1; i <= rs; i++ )
            Uk[++j] = Us[i];
    }

    N_KnotVectorGetKnots( kur, &rk, &Uk );

    if( tu EQ NL_RIGHT )
    {
        for ( i = 0; i <= rs; i++ )
            Uk[i] = Us[i];
    }
    else if( tu EQ NL_BOTH )
    {
        j = -1;

        for ( i = 0; i <= p; i++ )
            Uk[++j] = Us[i];

        for ( i = p + 2; i <= rs; i++ )
            Uk[++j] = Us[i];
    }

    N_KnotVectorGetKnots( kub, &rk, &Uk );

    if( tu EQ NL_BOTH )
    {
        for ( i = 0; i <= rs; i++ )
            Uk[i] = Us[i];
    }

    /* V-knot vectors */

    N_KnotVectorGetKnots( kvn, &rk, &Vk );

    if( tv EQ NL_NO )
    {
        for ( j = 0; j <= ss; j++ )
            Vk[j] = Vs[j];
    }
    else if( tv EQ NL_LEFT )
    {
        i = -1;

        for ( j = 0; j <= q; j++ )
            Vk[++i] = Vs[j];

        for ( j = q + 2; j <= ss; j++ )
            Vk[++i] = Vs[j];
    }
    else if( tv EQ NL_RIGHT )
    {
        i = -1;

        for ( j = 0; j <= ms - 1; j++ )
            Vk[++i] = Vs[j];

        for ( j = ms + 1; j <= ss; j++ )
            Vk[++i] = Vs[j];
    }
    else if( tv EQ NL_BOTH )
    {
        i = -1;

        for ( j = 0; j <= q; j++ )
            Vk[++i] = Vs[j];

        for ( j = q + 2; j <= ms - 1; j++ )
            Vk[++i] = Vs[j];

        for ( j = ms + 1; j <= ss; j++ )
            Vk[++i] = Vs[j];
    }

    N_KnotVectorGetKnots( kvl, &rk, &Vk );

    if( tv EQ NL_LEFT )
    {
        for ( j = 0; j <= ss; j++ )
            Vk[j] = Vs[j];
    }
    else if( tv EQ NL_BOTH )
    {
        i = -1;

        for ( j = 0; j <= ms - 1; j++ )
            Vk[++i] = Vs[j];

        for ( j = ms + 1; j <= ss; j++ )
            Vk[++i] = Vs[j];
    }

    N_KnotVectorGetKnots( kvr, &rk, &Vk );

    if( tv EQ NL_RIGHT )
    {
        for ( j = 0; j <= ss; j++ )
            Vk[j] = Vs[j];
    }
    else if( tv EQ NL_BOTH )
    {
        i = -1;

        for ( j = 0; j <= q; j++ )
            Vk[++i] = Vs[j];

        for ( j = q + 2; j <= ss; j++ )
            Vk[++i] = Vs[j];
    }

    N_KnotVectorGetKnots( kvb, &rk, &Vk );

    if( tv EQ NL_BOTH )
    {
        for ( j = 0; j <= ss; j++ )
            Vk[j] = Vs[j];
    }

    /* Get interpolation matrices */

    /* U-direction */

    bw = 2 * p - 1;
    sbw = p - 1;

    /* cun */

    error = N_SetRealMatrix( &cun, n, n, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cun, &A );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[n][sbw] = 1.0;

    for ( i = 1; i < n; i++ )
    {
        error = N_BasisEval( kun, p, ul[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - i - 1;

        for ( k = 0; k <= p; k++ )
            A[i][l + k] = N[k];
    }

    error = N_RealMatrixLuDecompose( &cun );

    if( error EQ NL_YES )
        NL_OUT;

    /* cul */

    if( tu EQ NL_LEFT OR tu EQ NL_BOTH )
    {
        error = N_SetRealMatrix( &cul, n + 1, n + 1, NL_MT_BANDED, bw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cul, &A );

        for ( i = 0; i <= n + 1; i++ )
        {
            for ( j = 0; j < bw; j++ )
                A[i][j] = 0.0;
        }

        A[0][sbw] = 1.0;
        A[1][sbw - 1] = -1.0;
        A[1][sbw] = 1.0;
        A[n + 1][sbw] = 1.0;

        for ( i = 2; i <= n; i++ )
        {
            error = N_BasisEval( kul, p, ul[i - 1], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            if( p EQ 2 )
                l = 0;
            else
                l = j - i - 1;

            if( l LT 0 OR l + p GE bw )
                NL_ERROR( NL_INP_ERR );

            for ( k = 0; k <= p; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cul );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* cur */

    if( tu EQ NL_RIGHT OR tu EQ NL_BOTH )
    {
        error = N_SetRealMatrix( &cur, n + 1, n + 1, NL_MT_BANDED, bw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cur, &A );

        for ( i = 0; i <= n + 1; i++ )
        {
            for ( j = 0; j < bw; j++ )
                A[i][j] = 0.0;
        }

        A[0][sbw] = 1.0;
        A[n][sbw] = -1.0;
        A[n][sbw + 1] = 1.0;
        A[n + 1][sbw] = 1.0;

        for ( i = 1; i <= n - 1; i++ )
        {
            error = N_BasisEval( kur, p, ul[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            if( p EQ 2 )
                l = 0;
            else
                l = j - i - 1;

            if( l LT 0 OR l + p GE bw )
                NL_ERROR( NL_INP_ERR );

            for ( k = 0; k <= p; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cur );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* cub */

    if( tu EQ NL_BOTH )
    {
        error = N_SetRealMatrix( &cub, n + 2, n + 2, NL_MT_BANDED, bw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cub, &A );

        for ( i = 0; i <= n + 2; i++ )
        {
            for ( j = 0; j < bw; j++ )
                A[i][j] = 0.0;
        }

        A[0][sbw] = 1.0;
        A[1][sbw - 1] = -1.0;
        A[1][sbw] = 1.0;
        A[n + 1][sbw] = -1.0;
        A[n + 1][sbw + 1] = 1.0;
        A[n + 2][sbw] = 1.0;

        for ( i = 2; i <= n; i++ )
        {
            error = N_BasisEval( kub, p, ul[i - 1], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            for ( k = 0; k <= p; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cub );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* V-direction */

    bw = 2 * q - 1;
    sbw = q - 1;

    /* cvn */

    error = N_SetRealMatrix( &cvn, m, m, NL_MT_BANDED, bw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cvn, &A );

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j < bw; j++ )
            A[i][j] = 0.0;
    }

    A[0][sbw] = 1.0;
    A[m][sbw] = 1.0;

    for ( i = 1; i < m; i++ )
    {
        error = N_BasisEval( kvn, q, vl[i], NL_LEFT, N, &j );

        if( error EQ NL_YES )
            NL_OUT;

        l = j - i - 1;

        for ( k = 0; k <= q; k++ )
            A[i][l + k] = N[k];
    }

    error = N_RealMatrixLuDecompose( &cvn );

    if( error EQ NL_YES )
        NL_OUT;

    /* cvl */

    if( tv EQ NL_LEFT OR tv EQ NL_BOTH )
    {
        error = N_SetRealMatrix( &cvl, m + 1, m + 1, NL_MT_BANDED, bw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cvl, &A );

        for ( i = 0; i <= m + 1; i++ )
        {
            for ( j = 0; j < bw; j++ )
                A[i][j] = 0.0;
        }

        A[0][sbw] = 1.0;
        A[1][sbw - 1] = -1.0;
        A[1][sbw] = 1.0;
        A[m + 1][sbw] = 1.0;

        for ( i = 2; i <= m; i++ )
        {
            error = N_BasisEval( kvl, q, vl[i - 1], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            if( q EQ 2 )
                l = 0;
            else
                l = j - i - 1;

            if( l LT 0 OR l + q GE bw )
                NL_ERROR( NL_INP_ERR );

            for ( k = 0; k <= q; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cvl );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* cvr */

    if( tv EQ NL_RIGHT OR tv EQ NL_BOTH )
    {
        error = N_SetRealMatrix( &cvr, m + 1, m + 1, NL_MT_BANDED, bw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cvr, &A );

        for ( i = 0; i <= m + 1; i++ )
        {
            for ( j = 0; j < bw; j++ )
                A[i][j] = 0.0;
        }

        A[0][sbw] = 1.0;
        A[m][sbw] = -1.0;
        A[m][sbw + 1] = 1.0;
        A[m + 1][sbw] = 1.0;

        for ( i = 1; i <= m - 1; i++ )
        {
            error = N_BasisEval( kvr, q, vl[i], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            if( q EQ 2 )
                l = 0;
            else
                l = j - i - 1;

            if( l LT 0 OR l + q GE bw )
                NL_ERROR( NL_INP_ERR );

            for ( k = 0; k <= q; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cvr );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* cvb */

    if( tv EQ NL_BOTH )
    {
        error = N_SetRealMatrix( &cvb, m + 2, m + 2, NL_MT_BANDED, bw, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( &cvb, &A );

        for ( i = 0; i <= m + 2; i++ )
        {
            for ( j = 0; j < bw; j++ )
                A[i][j] = 0.0;
        }

        A[0][sbw] = 1.0;
        A[1][sbw - 1] = -1.0;
        A[1][sbw] = 1.0;
        A[m + 1][sbw] = -1.0;
        A[m + 1][sbw + 1] = 1.0;
        A[m + 2][sbw] = 1.0;

        for ( i = 2; i <= m; i++ )
        {
            error = N_BasisEval( kvb, q, vl[i - 1], NL_LEFT, N, &j );

            if( error EQ NL_YES )
                NL_OUT;

            l = j - i - 1;

            for ( k = 0; k <= q; k++ )
                A[i][l + k] = N[k];
        }

        error = N_RealMatrixLuDecompose( &cvb );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get u-directional fit */

    if( Tu NEQ NULL OR u_closed )
    {
        for ( j = 0; j <= m; j++ )
        {
            for ( i = 0; i <= n; i++ )
                N_CopyPt( P[i][j], &Q[i] );

            if( u_closed )
            {
                k = n + 2;

                error = ST_CrvInterpDerivsKnots( (NL_VOID *)Q, n, NL_EPOINT, p, (NL_VOID *) &Tul[0][j], (NL_VOID *) &Tul[1][j], kub, &cub, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Qw[i], &Pu[i][j] );

                continue;
            }

            if( Tu[0][j]EQ NULL AND Tu[1][j]EQ NULL )
            {
                k = n;

                error = ST_CrvFitPts( (NL_VOID *)Q, NL_EPOINT, &cun, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                if( tu EQ NL_LEFT )
                {
                    error = ST_KnotInsertInPlace( Qw, k, p, kun, Us[p + 1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k++;
                }
                else if( tu EQ NL_RIGHT )
                {
                    error = ST_KnotInsertInPlace( Qw, k, p, kun, Us[ns] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k++;
                }
                else if( tu EQ NL_BOTH )
                {
                    error = ST_KnotInsertInPlace( Qw, k, p, kun, Us[p + 1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    error = ST_KnotInsertInPlace( Qw, k + 1, p, kul, Us[ns] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k += 2;
                }

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Qw[i], &Pu[i][j] );
            }
            else if( Tu[0][j]NEQ NULL AND Tu[1][j]EQ NULL )
            {
                k = n + 1;

                error = ST_CrvInterpTangentsKnots( (NL_VOID *)Q, n, NL_EPOINT, p, (NL_VOID *) &Tul[0][j], NL_START, kul, &cul, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                if( tu EQ NL_BOTH )
                {
                    error = ST_KnotInsertInPlace( Qw, k, p, kul, Us[ns] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k++;
                }

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Qw[i], &Pu[i][j] );
            }
            else if( Tu[0][j]EQ NULL AND Tu[1][j]NEQ NULL )
            {
                k = n + 1;

                error = ST_CrvInterpTangentsKnots( (NL_VOID *)Q, n, NL_EPOINT, p, (NL_VOID *) &Tul[1][j], NL_END, kur, &cur, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                if( tu EQ NL_BOTH )
                {
                    error = ST_KnotInsertInPlace( Qw, k, p, kur, Us[p + 1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    k++;
                }

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Qw[i], &Pu[i][j] );
            }
            else if( Tu[0][j]NEQ NULL AND Tu[1][j]NEQ NULL )
            {
                k = n + 2;

                error = ST_CrvInterpDerivsKnots( (NL_VOID *)Q, n, NL_EPOINT, p, (NL_VOID *) &Tul[0][j], (NL_VOID *) &Tul[1][j], kub, &cub, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Qw[i], &Pu[i][j] );
            }
        }

        for ( i = 0; i <= ns; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_CopyCPt( Pu[i][j], &Qw[j] );

            l = m;

            error = ST_CrvFitPts( (NL_VOID *)Qw, NL_HPOINT, &cvn, Rw );

            if( error EQ NL_YES )
                NL_OUT;

            if( tv EQ NL_LEFT )
            {
                error = ST_KnotInsertInPlace( Rw, l, q, kvn, Vs[q + 1] );

                if( error EQ NL_YES )
                    NL_OUT;

                l++;
            }
            else if( tv EQ NL_RIGHT )
            {
                error = ST_KnotInsertInPlace( Rw, l, q, kvn, Vs[ms] );

                if( error EQ NL_YES )
                    NL_OUT;

                l++;
            }
            else if( tv EQ NL_BOTH )
            {
                error = ST_KnotInsertInPlace( Rw, l, q, kvn, Vs[q + 1] );

                if( error EQ NL_YES )
                    NL_OUT;

                error = ST_KnotInsertInPlace( Rw, l + 1, q, kvl, Vs[ms] );

                if( error EQ NL_YES )
                    NL_OUT;

                l += 2;
            }

            for ( j = 0; j <= l; j++ )
                N_CopyCPt( Rw[j], &Pu[i][j] );
        }
    }

    /* Get v-directional fit */

    if( Tv NEQ NULL OR v_closed )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
                N_CopyPt( P[i][j], &Q[j] );

            if( v_closed )
            {
                l = m + 2;

                error = ST_CrvInterpDerivsKnots( (NL_VOID *)Q, m, NL_EPOINT, q, (NL_VOID *) &Tvl[i][0], (NL_VOID *) &Tvl[i][1], kvb, &cvb, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Qw[j], &Pv[i][j] );

                continue;
            }

            if( Tv[i][0]EQ NULL AND Tv[i][1]EQ NULL )
            {
                l = m;

                error = ST_CrvFitPts( (NL_VOID *)Q, NL_EPOINT, &cvn, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                if( tv EQ NL_LEFT )
                {
                    error = ST_KnotInsertInPlace( Qw, l, q, kvn, Vs[q + 1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l++;
                }
                else if( tv EQ NL_RIGHT )
                {
                    error = ST_KnotInsertInPlace( Qw, l, q, kvn, Vs[ms] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l++;
                }
                else if( tv EQ NL_BOTH )
                {
                    error = ST_KnotInsertInPlace( Qw, l, q, kvn, Vs[q + 1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    error = ST_KnotInsertInPlace( Qw, l + 1, q, kvl, Vs[ms] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l += 2;
                }

                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Qw[j], &Pv[i][j] );
            }
            else if( Tv[i][0]NEQ NULL AND Tv[i][1]EQ NULL )
            {
                l = m + 1;

                error = ST_CrvInterpTangentsKnots( (NL_VOID *)Q, m, NL_EPOINT, q, (NL_VOID *) &Tvl[i][0], NL_START, kvl, &cvl, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                if( tv EQ NL_BOTH )
                {
                    error = ST_KnotInsertInPlace( Qw, l, q, kvl, Vs[ms] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l++;
                }

                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Qw[j], &Pv[i][j] );
            }
            else if( Tv[i][0]EQ NULL AND Tv[i][1]NEQ NULL )
            {
                l = m + 1;

                error = ST_CrvInterpTangentsKnots( (NL_VOID *)Q, m, NL_EPOINT, q, (NL_VOID *) &Tvl[i][1], NL_END, kvr, &cvr, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                if( tv EQ NL_BOTH )
                {
                    error = ST_KnotInsertInPlace( Qw, l, q, kvr, Vs[q + 1] );

                    if( error EQ NL_YES )
                        NL_OUT;

                    l++;
                }

                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Qw[j], &Pv[i][j] );
            }
            else if( Tv[i][0]NEQ NULL AND Tv[i][1]NEQ NULL )
            {
                l = m + 2;

                error = ST_CrvInterpDerivsKnots( (NL_VOID *)Q, m, NL_EPOINT, q, (NL_VOID *) &Tvl[i][0], (NL_VOID *) &Tvl[i][1], kvb, &cvb, Qw );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = 0; j <= l; j++ )
                    N_CopyCPt( Qw[j], &Pv[i][j] );
            }
        }

        for ( j = 0; j <= ms; j++ )
        {
            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Pv[i][j], &Qw[i] );

            k = n;

            error = ST_CrvFitPts( (NL_VOID *)Qw, NL_HPOINT, &cun, Rw );

            if( error EQ NL_YES )
                NL_OUT;

            if( tu EQ NL_LEFT )
            {
                error = ST_KnotInsertInPlace( Rw, k, p, kun, Us[p + 1] );

                if( error EQ NL_YES )
                    NL_OUT;

                k++;
            }
            else if( tu EQ NL_RIGHT )
            {
                error = ST_KnotInsertInPlace( Rw, k, p, kun, Us[ns] );

                if( error EQ NL_YES )
                    NL_OUT;

                k++;
            }
            else if( tu EQ NL_BOTH )
            {
                error = ST_KnotInsertInPlace( Rw, k, p, kun, Us[p + 1] );

                if( error EQ NL_YES )
                    NL_OUT;

                error = ST_KnotInsertInPlace( Rw, k + 1, p, kul, Us[ns] );

                if( error EQ NL_YES )
                    NL_OUT;

                k += 2;
            }

            for ( i = 0; i <= k; i++ )
                N_CopyCPt( Rw[i], &Pv[i][j] );
        }
    }

    /* See if correction surface is required */

    if( tu EQ NL_NO OR tv EQ NL_NO )
    {
        if( tu EQ NL_NO )
        {
            for ( i = 0; i <= ns; i++ )
            {
                for ( j = 0; j <= ms; j++ )
                    N_CopyCPt( Pv[i][j], &Pw[i][j] );
            }
        }

        if( tv EQ NL_NO )
        {
            for ( i = 0; i <= ns; i++ )
            {
                for ( j = 0; j <= ms; j++ )
                    N_CopyCPt( Pu[i][j], &Pw[i][j] );
            }
        }

        NL_OUT;
    }

    /* Get data interpolation */

    for ( j = 0; j <= m; j++ )
    {
        for ( i = 0; i <= n; i++ )
            N_CopyPt( P[i][j], &Q[i] );

        k = n;

        error = ST_CrvFitPts( (NL_VOID *)Q, NL_EPOINT, &cun, Qw );

        if( error EQ NL_YES )
            NL_OUT;

        if( tu EQ NL_LEFT )
        {
            error = ST_KnotInsertInPlace( Qw, k, p, kun, Us[p + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            k++;
        }
        else if( tu EQ NL_RIGHT )
        {
            error = ST_KnotInsertInPlace( Qw, k, p, kun, Us[ns] );

            if( error EQ NL_YES )
                NL_OUT;

            k++;
        }
        else if( tu EQ NL_BOTH )
        {
            error = ST_KnotInsertInPlace( Qw, k, p, kun, Us[p + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = ST_KnotInsertInPlace( Qw, k + 1, p, kul, Us[ns] );

            if( error EQ NL_YES )
                NL_OUT;

            k += 2;
        }

        for ( i = 0; i <= k; i++ )
            N_CopyCPt( Qw[i], &Pt[i][j] );
    }

    for ( i = 0; i <= ns; i++ )
    {
        for ( j = 0; j <= m; j++ )
            N_CopyCPt( Pt[i][j], &Qw[j] );

        l = m;

        error = ST_CrvFitPts( (NL_VOID *)Qw, NL_HPOINT, &cvn, Rw );

        if( error EQ NL_YES )
            NL_OUT;

        if( tv EQ NL_LEFT )
        {
            error = ST_KnotInsertInPlace( Rw, l, q, kvn, Vs[q + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            l++;
        }
        else if( tv EQ NL_RIGHT )
        {
            error = ST_KnotInsertInPlace( Rw, l, q, kvn, Vs[ms] );

            if( error EQ NL_YES )
                NL_OUT;

            l++;
        }
        else if( tv EQ NL_BOTH )
        {
            error = ST_KnotInsertInPlace( Rw, l, q, kvn, Vs[q + 1] );

            if( error EQ NL_YES )
                NL_OUT;

            error = ST_KnotInsertInPlace( Rw, l + 1, q, kvl, Vs[ms] );

            if( error EQ NL_YES )
                NL_OUT;

            l += 2;
        }

        for ( j = 0; j <= l; j++ )
            N_CopyCPt( Rw[j], &Pt[i][j] );
    }

    /* Get surface points */

    for ( i = 0; i <= ns; i++ )
    {
        for ( j = 0; j <= ms; j++ )
        {
            N_TranslateSum2CPts( Pu[i][j], 1.0, Pv[i][j], -1.0, Pt[i][j], &Pw[i][j] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_FITSRFINTPBICUBIC: Surface interpolation with C11 bicubic surfaces          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a C11 continuous nonrational bicubic
     NURBS surface interpolating a given set  of points. If  the output 
     surface is initialized to the NULL surface,  memory  is  allocated 
     locally. Otherwise it is  checked if enough memory is passed in. A 
     typical calling example is:

       NL_POINT    **P;
       NL_INDEX    k, l;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get array P);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfInterpBicubic(P,k,l,NL_AKIMA,&sur,&SG);

     A recommended  default for the tangent is NL_AKIMA.


   ACCESS:
   
     P    , input  ,  Points to be interpolated
     k,l  , input  ,  Highest indexes in P
     tan  , input  ,  Flag:
                        NL_BESSEL: Tangents computed by Bessel's method
                        NL_AKIMA : Tangents computed by Akima's method
     sur  , output ,  Interpolating surface
     SG   , input  ,  sur's memory stack

     (use: NL_POINT **P = N_AllocPt2dArray(k, l, &SG);     
          

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfInterpBicubic
  (NL_POINT ** P,    /* in : grid of sample points, sized:[k+1][l+1] */
   NL_INDEX    k,    /* in : highest 1st P index */
   NL_INDEX    l,    /* in : highest 2nd P index */
   NL_FLAG     tan,  /* in : tangent flag: NL_BESSEL: Tangents computed by Bessel's method */
                     /*                    NL_AKIMA : Tangents computed by Akima's method  */
   NL_SURFACE *sur,  /* out: Interpolating surface */
   NL_STACKS  *SG )  /* in : sur's memory stack */
{
    return (N_FitPtsNormals( P, NULL, k, l, tan, sur, SG ));
}

NL_PRIVATE_TLS NL_FLAG error;
#if NLIB_UNUSED

NL_PRIVATE_TLS NL_POINT *Q, *R;
NL_PRIVATE_TLS NL_REAL *u, *v, *vsort;
NL_PRIVATE_TLS NL_INDEX gn;
/**********************************************************************/
/* N_FitSphereToPtsGlobal: Best fitting sphere or sphere patch to a set of points   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a  least-squares sphere or  spherical
     patch to a  set of 3-D points. An initial  least-squares  sphere is
     computed, and  then it is  improved  by  Gauss-Newton  iteration to
     minimize the  true distances from  the sphere. If the least-squares
     plane is  better  suited, i.e. the average  error for the  plane is 
     less than that of the sphere, a planar patch is returned. A typical 
     calling example is:

       NL_FLAG     sut;
       NL_POINT    *P;
       NL_INDEX    n;
       NL_REAL     tol, tpc, era, erm;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get P, tpc and tol);
       ...
       N_SrfInitArrays(&sur);
       N_FitSphereToPtsGlobal(P,n,tol,NL_MAXIMUM,NL_YES,tpc,NL_QUADRATIC,NL_QUINTIC,&sur,&sut,
                &era,&erm,&SG);

     THIS VERSION OF  SPHERE FITTING  SEGMENTS THE NL_FULL SPHERE GLOBALLY, 
     I.E. IT CENTERS THE NL_POINT SET  AROUND THE GLOBAL ORIGIN SITUATED ON
     THE EQUATOR. THEN IT ROTATES THE NL_POINT SET AROUND THE GLOBAL X AXIS
     TO OBTAIN THE BEST ENCLOSING SPHERICAL NL_RECTANGLE.


   ACCESS:
   
     P     , input  ,  Points in 3-D
     n     , input  ,  Highest index in P
     tol   , input  ,  Tolerance to  check  coplanarity; the  maximum or 
                       the average  error from  the least-squares  plane 
                       must be less than "tol"
     etp   , input  ,  Flag:
                         NL_MAXIMUM: tol is maximum deviation
                         NL_AVERAGE: tol is average deviation
     pfl   , input  ,  Flag:
                         NL_YES: find the best fitting patch to the points
                         NL_NO : output the full sphere
     tpc   , input  ,  Tolerance to find best  bounding rectangle; it is
                       expressed as a percentage, e.g., tpc = 0.1  gives 
                       a  bounding rectangle  that is about 1 degree off 
                       the  absolute  minimum. In general, the  bounding 
                       box tolerance is set as  "tpc" times an estimated  
                       minimum  box  area. A good  default is  somewhere 
                       between 0.1 and 0.01.
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: profile circle is quadratic
                         NL_QUINTIC  : profile circle is quintic
     rtp   , input  ,  Flag:
                         NL_QUADRATIC: revolution iso-curve is quadratic
                         NL_QUINTIC  : revolution iso-curve is quintic
     sur   , output ,  Best fitting NURBS sphere
     sut   , output ,  Flag:
                         NL_NPLANE : a planar patch is returned
                         NL_NSPHERE: a sphere/patch is returned
     era   , output ,  Average absolute error
     erm   , output ,  Maximum absolute error
     SG    , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSphereToPtsGlobal
  (NL_POINT   *P,    /* in : cloud-of-points, sized:[n+1] */
   NL_INDEX    n,    /* in : highest index in P           */
   NL_REAL     tol,  /* in : max or avgerage coplanarity tolerance   */
   NL_FLAG     etp,  /* in : tol type flag: NL_MAXIMUM or NL_AVERAGE */
   NL_FLAG     pfl,  /* in : NL_YES = return the best-fit plane or sphere */
                     /*      NL_NO  = force return of sphere              */
   NL_REAL     tpc,  /* in : percentage tolerance to find best bounding rectangle, good values between 0.01 and 0.1 */
   NL_FLAG     ctp,  /* in : NL_QUADRATIC: profile circle is quadratic */
                     /*      NL_QUINTIC  : profile circle is quintic   */
   NL_FLAG     rtp,  /* in : NL_QUADRATIC: revolution iso-curve is quadratic */
                     /*      NL_QUINTIC  : revolution iso-curve is quintic   */
   NL_SURFACE *sur,  /* out: best fitting nurbs sphere or plane-patch */
   NL_FLAG    *sut,  /* out: NL_NPLANE : a planar patch is returned   */
                     /*      NL_NSPHERE: a sphere/patch is returned   */
   NL_REAL    *era,  /* out: Average absolute error  */
   NL_REAL    *erm,  /* out: Maximum absolute error  */
   NL_STACKS  *SG )  /* in : sur's stack             */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSphereToPtsGlobal");

    NL_FLAG pfi, sfi, cpl, sfl;

    NL_INDEX i, iv = 0, kl, kr, imin = 0;

    NL_REAL rot[37], RA[37], Amin, ua, va, pea, pem, sea = 0.0, sem = 0.0, cx = 0.0, cy = 0.0, cz = 0.0, r = 0.0, dot, a, b, c, dv, us, ue, vs, ve, alf, rtol, alr, bp, cp, cosal, sinal;

    NL_POINT B, C;

    NL_VECTOR N, V, QXY, RXY;

    NL_CURVE cur;

    NL_RMATRIX rma, rxyz, rinv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    if( n LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Fit a plane first and check if points are coplanar */

    error = NL_NO;
    cpl = NL_NO;
    gn = n;

    error = N_PlaneFit3dPts( P, n, &C, &N, &pea, &pem );

    if( error EQ NL_YES )
        NL_OUT;

    switch( etp )
    {
        case NL_MAXIMUM:
            if( pem LT tol )
                cpl = NL_YES;
            break;

        case NL_AVERAGE:
            if( pea LT tol )
                cpl = NL_YES;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* If the points are not coplanar, attempt to fit a sphere */

    pfi = sfi = NL_NO;

    if( cpl EQ NL_NO )
    {
        error = N_SphereFit3dPts( P, n, tol, etp, &cx, &cy, &cz, &r, &sea, &sem, &sfl );

        if( error EQ NL_YES )
            NL_OUT;

        if( sfl EQ NL_TRUE )
        {
            if( pea LT sea )
                pfi = NL_YES;
            else
                sfi = NL_YES;
        }
        else
        {
            pfi = NL_YES;
        }
    }
    else
    {
        pfi = NL_YES;
    }

    /* Plane fit was chosen either because the points are coplanar, or */
    /* because the NL_AVERAGE error for the plane is smaller than the one */
    /* for the sphere                                                  */

    if( pfi EQ NL_YES )
    {
        /* Get planar patch */

        error = N_FitPlaneToPts( P, n, tpc, sur, era, erm, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *sut = NL_NPLANE;

        NL_OUT;
    }

    /* Output entire sphere */

    N_VectorCreate( cx, cy, cz, &C );

    if( sfi EQ NL_YES AND pfl EQ NL_NO )
    {
        error = N_CreateSphere( C, r, 0.0, 180.0, 360.0, ctp, rtp, sur, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *era = sea;
        *erm = sem;
        *sut = NL_NSPHERE;

        NL_OUT;
    }

    /* Allocate memories */

    Q = N_AllocPt1dArray( n, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    R = N_AllocPt1dArray( n, &SL );

    if( R EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( n, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( n, &SL );

    if( v EQ NULL )
        NL_QUIT;

    vsort = N_AllocReal1dArray( n + 1, &SL );

    if( vsort EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SetRealMatrix( &rxyz, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SetRealMatrix( &rinv, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute spherical patch: project points onto the sphere */
    /* and center sphere at the origin                         */

    for ( i = 0; i <= n; i++ )
    {
        N_VectorDiff( P[i], C, &V );

        error = N_VectorNormalizeRef( &V );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorPtAlongVector( C, r, V, &B );
        N_VectorDiff( B, C, &R[i] );
    }

    /* Get spherical center of gravity */

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( R[i], &a, &b, &c );
        N_VectorCreate( a, b, 0.0, &QXY );

        error = N_VectorsAngle( R[i], NL_UNITZ, &u[i] );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( NL_ZERO, QXY, &c );

        if( c LT NL_MTOL )
        {
            v[i] = 0.0;
        }
        else
        {
            error = N_VectorsAngle( QXY, NL_UNITX, &v[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDot( QXY, NL_UNITY, &dot );

            if( dot LT 0.0 )
                v[i] = 360.0 - v[i];
        }

        vsort[i] = v[i];
    }

    N_ShellSortReal( vsort, n );

    vsort[n + 1] = 360.0 + vsort[0];

    a = b = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        dv = fabs( vsort[i + 1] - vsort[i] );

        if( dv GT b )
        {
            b = dv;
            iv = i;
        }
    }

    vs = vsort[iv + 1];
    ve = vsort[iv];

    if( vs GT ve )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( v[i]LE 360.0 AND v[i]GE vs )
                v[i] -= 360.0;
        }
    }

    ua = u[0];
    va = v[0];

    for ( i = 1; i <= n; i++ )
    {
        ua += u[i];
        va += v[i];
    }

    a = (NL_REAL)n + 1.0;
    b = 1.0 / a;

    ua *= b;
    va *= b;

    /* Find normal corresponding to the average spherical point */

    if( va LT 0.0 )
        va = 360.0 + va;

    ua *= NL_RAD;
    va *= NL_RAD;

    a = r * sin( ua ) * cos( va );
    b = r * sin( ua ) * sin( va );
    c = r * cos( ua );

    N_VectorCreate( a, b, c, &N );

    /* Rotate N to become the X-axis, and transform points */

    error = N_CreateTransformMatrixFromVector( N, NL_XDIR, &rxyz, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= n; i++ )
        N_TransformPt( R[i], &rxyz, &Q[i] );

    /* Now find minimum rotation angle */

    rot[1] = 0.0;
    rot[2] = 10.0;
    rot[3] = 20.0;
    rot[4] = 30.0;
    rot[5] = 40.0;
    rot[6] = 50.0;
    rot[7] = 60.0;
    rot[8] = 70.0;
    rot[9] = 80.0;
    rot[10] = 90.0;
    rot[11] = 100.0;
    rot[12] = 110.0;
    rot[13] = 120.0;
    rot[14] = 130.0;
    rot[15] = 140.0;
    rot[16] = 150.0;
    rot[17] = 160.0;
    rot[18] = 170.0;
    rot[19] = 180.0;
    rot[20] = 190.0;
    rot[21] = 200.0;
    rot[22] = 210.0;
    rot[23] = 220.0;
    rot[24] = 230.0;
    rot[25] = 240.0;
    rot[26] = 250.0;
    rot[27] = 260.0;
    rot[28] = 270.0;
    rot[29] = 280.0;
    rot[30] = 290.0;
    rot[31] = 300.0;
    rot[32] = 310.0;
    rot[33] = 320.0;
    rot[34] = 330.0;
    rot[35] = 340.0;
    rot[36] = 350.0;

    Amin = NL_BIGD;

    for ( i = 1; i <= 36; i++ )
    {
        RA[i] = ST_CalcAreaSphereRotPtsGlobal( rot[i] );

        if( RA[i]LT 0.0 )
            NL_OUT;

        if( RA[i]LT Amin )
        {
            Amin = RA[i];
            imin = i;
        }
    }

    kl = kr = -1;

    for ( i = imin - 1; i >= 1; i-- )
    {
        if( RA[i]GT Amin )
        {
            kl = i;
            break;
        }
    }

    if( kl GT - 1 )
    {
        for ( i = imin + 1; i <= 36; i++ )
        {
            if( RA[i]GT Amin )
            {
                kr = i;
                break;
            }
        }
    }

    rtol = tpc * Amin;

    if( kl EQ - 1 OR kr EQ - 1 )
    {
        alf = rot[imin];
    }
    else
    {
        N_FuncFindMinima( rot[kl], rot[imin], rot[kr], Amin, ST_CalcAreaSphereRotPtsGlobal, rtol, 0.0, &alf, &dot );
    }

    /* Rotate point set about the X axis with alf */

    alr = NL_RAD * alf;
    cosal = cos( alr );
    sinal = sin( alr );

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( Q[i], &a, &b, &c );

        bp = b * cosal - c * sinal;
        cp = b * sinal + c * cosal;

        N_VectorCreate( a, bp, cp, &R[i] );
    }

    /* Get best sperical box */

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( R[i], &a, &b, &c );
        N_VectorCreate( a, b, 0.0, &RXY );

        error = N_VectorsAngle( R[i], NL_UNITZ, &u[i] );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( NL_ZERO, RXY, &c );

        if( c LT NL_MTOL )
        {
            v[i] = 0.0;
        }
        else
        {
            error = N_VectorsAngle( RXY, NL_UNITX, &v[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDot( RXY, NL_UNITY, &dot );

            if( dot LT 0.0 )
                v[i] = 360.0 - v[i];
        }

        vsort[i] = v[i];
    }

    N_ShellSortReal( vsort, n );

    vsort[n + 1] = 360.0 + vsort[0];

    a = b = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        dv = fabs( vsort[i + 1] - vsort[i] );

        if( dv GT b )
        {
            b = dv;
            iv = i;
        }
    }

    vs = vsort[iv + 1];
    ve = vsort[iv];

    if( vs GT ve )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( v[i]LE 360.0 AND v[i]GE vs )
                v[i] -= 360.0;
        }
    }

    us = 200.0;
    ue = -200.0;
    vs = 400.0;
    ve = -400.0;

    for ( i = 0; i <= n; i++ )
    {
        if( u[i]LT us )
            us = u[i];

        if( u[i]GT ue )
            ue = u[i];

        if( v[i]LT vs )
            vs = v[i];

        if( v[i]GT ve )
            ve = v[i];
    }

    /* Create local NURBS patch */

    c = vs * NL_RAD;
    a = r * cos( c );
    b = r * sin( c );

    N_VectorCreate( a, b, 0.0, &V );

    N_CrvInitArrays( &cur );
    error = N_CreateCircArc( NL_ZERO, NL_UNITZ, V, r, us, ue, ctp, &cur, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    a = ve - vs;

    error = N_CreateRevolvedSrf( &cur, NL_ZERO, NL_UNITZ, a, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Position patch to fit original points */

    error = N_RealMatrixInversePivot( &rxyz, &rinv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    a = -alf;

    error = N_SrfRotateAtPt( sur, NL_ZERO, NL_UNITX, a );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfTransform( sur, &rinv );
    N_SrfTranslate( sur, C );

    *era = sea;
    *erm = sem;
    *sut = NL_NSPHERE;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

NL_PRIVATE_TLS NL_VECTOR N;
NL_PRIVATE_TLS NL_RMATRIX rma;
NL_PRIVATE_TLS NL_STACKS SL;
/**********************************************************************/
/* N_FitSphereToPtsLocal: Best fitting sphere or sphere patch to a set of points   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a  least-squares sphere or  spherical
     patch to a  set of 3-D points. An initial  least-squares  sphere is
     computed, and  then it is  improved  by  Gauss-Newton  iteration to
     minimize the  true distances from  the sphere. If the least-squares
     plane is  better  suited, i.e. the average  error for the  plane is 
     less than that of the sphere, a planar patch is returned. A typical 
     calling example is:

       NL_FLAG     sut;
       NL_POINT    *P;
       NL_INDEX    n;
       NL_REAL     tol, tpc, era, erm;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get P, tpc and tol);
       ...
       N_SrfInitArrays(&sur);
       N_FitSphereToPtsLocal(P,n,tol,NL_MAXIMUM,NL_YES,tpc,NL_QUADRATIC,NL_QUINTIC,&sur,&sut,
                &era,&erm,&SG);

     THIS VERSION OF SPHERE FITTING SEGMENTS THE NL_FULL SPHERE LOCALLY, IE
     IT ROTATES THE NL_POINTS  AROUND THE NORMAL  PASSING THROUGH THE LOCAL
     CENTER OF GRAVITY.


   ACCESS:
   
     P     , input  ,  Points in 3-D
     n     , input  ,  Highest index in P
     tol   , input  ,  Tolerance to  check  coplanarity; the  maximum or 
                       the average  error from  the least-squares  plane 
                       must be less than "tol"
     etp   , input  ,  Flag:
                         NL_MAXIMUM: tol is maximum deviation
                         NL_AVERAGE: tol is average deviation
     pfl   , input  ,  Flag:
                         NL_YES: find the best  fitting patch to the points
                         NL_NO : output the full sphere
     tpc   , input  ,  Tolerance to find best  bounding rectangle; it is
                       expressed as a percentage, e.g., tpc = 0.1  gives 
                       a  bounding rectangle  that is about 1 degree off 
                       the  absolute  minimum. In general, the  bounding 
                       box tolerance is set as  "tpc" times an estimated  
                       minimum  box  area. A good  default is  somewhere 
                       between 0.1 and 0.01.
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: profile circle is quadratic
                         NL_QUINTIC  : profile circle is quintic
     rtp   , input  ,  Flag:
                         NL_QUADRATIC: revolution iso-curve is quadratic
                         NL_QUINTIC  : revolution iso-curve is quintic
     sur   , output ,  Best fitting NURBS sphere
     sut   , output ,  Flag:
                         NL_NPLANE : a planar patch is returned
                         NL_NSPHERE: a sphere/patch is returned
     era   , output ,  Average absolute error
     erm   , output ,  Maximum absolute error
     SG    , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSphereToPtsLocal 
  (NL_POINT   * P,    /* in : cloud-of-points, sized:[n+1] */                                                            
   NL_INDEX     n,    /* in : highest index in P           */                                                            
   NL_REAL      tol,  /* in : max or avgerage coplanarity tolerance   */                                                 
   NL_FLAG      etp,  /* in : tol type flag: NL_MAXIMUM or NL_AVERAGE */                                                 
   NL_FLAG      pfl,  /* in : NL_YES = return the best-fit plane or sphere */                                            
                      /*      NL_NO  = force return of sphere              */                                            
   NL_REAL      tpc,  /* in : percentage tolerance to find best bounding rectangle, good values between 0.01 and 0.1 */  
   NL_FLAG      ctp,  /* in : NL_QUADRATIC: profile circle is quadratic */                                               
                      /*      NL_QUINTIC  : profile circle is quintic   */                                               
   NL_FLAG      rtp,  /* in : NL_QUADRATIC: revolution iso-curve is quadratic */                                         
                      /*      NL_QUINTIC  : revolution iso-curve is quintic   */                                         
   NL_SURFACE * sur,  /* out: best fitting nurbs sphere or plane-patch */                                                
   NL_FLAG    * sut,  /* out: NL_NPLANE : a planar patch is returned   */                                                
                      /*      NL_NSPHERE: a sphere/patch is returned   */                                              
   NL_REAL    * era,  /* out: Average absolute error  */                                                               
   NL_REAL    * erm,  /* out: Maximum absolute error  */                                                               
   NL_STACKS  * SG )  /* in : sur's stack             */                                                               
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSphereToPtsLocal");

    NL_FLAG pfi, sfi, cpl, sfl;

    NL_INDEX i, iv = 0, kl, kr, imin = 0;

    NL_REAL rot[37], RA[37], Amin, ua, va, pea, pem, sea = 0.0, sem = 0.0, cx = 0.0, cy = 0.0, cz = 0.0, r = 0.0, dot, a, b, c, dv, us, ue, vs, ve, alf, rtol;

    NL_POINT B, C;

    NL_VECTOR V, QXY, RXY;

    NL_CURVE cur;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    if( n LT 2 )
        NL_ERROR( NL_INP_ERR );

    /* Fit a plane first and check if points are coplanar */

    error = NL_NO;
    cpl = NL_NO;
    gn = n;

    error = N_PlaneFit3dPts( P, n, &C, &N, &pea, &pem );

    if( error EQ NL_YES )
        NL_OUT;

    switch( etp )
    {
        case NL_MAXIMUM:
            if( pem LT tol )
                cpl = NL_YES;
            break;

        case NL_AVERAGE:
            if( pea LT tol )
                cpl = NL_YES;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* If the points are not coplanar, attempt to fit a sphere */

    pfi = sfi = NL_NO;

    if( cpl EQ NL_NO )
    {
        error = N_SphereFit3dPts( P, n, tol, etp, &cx, &cy, &cz, &r, &sea, &sem, &sfl );

        if( error EQ NL_YES )
            NL_OUT;

        if( sfl EQ NL_TRUE )
        {
            if( pea LT sea )
                pfi = NL_YES;
            else
                sfi = NL_YES;
        }
        else
        {
            pfi = NL_YES;
        }
    }
    else
    {
        pfi = NL_YES;
    }

    /* Plane fit was chosen either because the points are coplanar, or */
    /* because the NL_AVERAGE error for the plane is smaller than the one */
    /* for the sphere                                                  */

    if( pfi EQ NL_YES )
    {
        /* Get planar patch */

        error = N_FitPlaneToPts( P, n, tpc, sur, era, erm, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *sut = NL_NPLANE;

        NL_OUT;
    }

    /* Output entire sphere */

    N_VectorCreate( cx, cy, cz, &C );

    if( sfi EQ NL_YES AND pfl EQ NL_NO )
    {
        error = N_CreateSphere( C, r, 0.0, 180.0, 360.0, ctp, rtp, sur, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *era = sea;
        *erm = sem;
        *sut = NL_NSPHERE;

        NL_OUT;
    }

    /* Allocate memories */

    Q = N_AllocPt1dArray( n, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    R = N_AllocPt1dArray( n, &SL );

    if( R EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( n, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( n, &SL );

    if( v EQ NULL )
        NL_QUIT;

    vsort = N_AllocReal1dArray( n + 1, &SL );

    if( vsort EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute spherical patch: project points onto the sphere */
    /* and center sphere at the origin                         */

    for ( i = 0; i <= n; i++ )
    {
        N_VectorDiff( P[i], C, &V );

        error = N_VectorNormalizeRef( &V );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorPtAlongVector( C, r, V, &B );
        N_VectorDiff( B, C, &Q[i] );
    }

    /* Get spherical center of gravity */

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( Q[i], &a, &b, &c );
        N_VectorCreate( a, b, 0.0, &QXY );

        error = N_VectorsAngle( Q[i], NL_UNITZ, &u[i] );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( NL_ZERO, QXY, &c );

        if( c LT NL_MTOL )
        {
            v[i] = 0.0;
        }
        else
        {
            error = N_VectorsAngle( QXY, NL_UNITX, &v[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDot( QXY, NL_UNITY, &dot );

            if( dot LT 0.0 )
                v[i] = 360.0 - v[i];
        }

        vsort[i] = v[i];
    }

    N_ShellSortReal( vsort, n );

    vsort[n + 1] = 360.0 + vsort[0];

    a = b = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        dv = fabs( vsort[i + 1] - vsort[i] );

        if( dv GT b )
        {
            b = dv;
            iv = i;
        }
    }

    vs = vsort[iv + 1];
    ve = vsort[iv];

    if( vs GT ve )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( v[i]LE 360.0 AND v[i]GE vs )
                v[i] -= 360.0;
        }
    }

    ua = u[0];
    va = v[0];

    for ( i = 1; i <= n; i++ )
    {
        ua += u[i];
        va += v[i];
    }

    a = (NL_REAL)n + 1.0;
    b = 1.0 / a;

    ua *= b;
    va *= b;

    /* Find normal corresponding to the average spherical point */

    if( va LT 0.0 )
        va = 360.0 + va;

    ua *= NL_RAD;
    va *= NL_RAD;

    a = r * sin( ua ) * cos( va );
    b = r * sin( ua ) * sin( va );
    c = r * cos( ua );

    N_VectorCreate( a, b, c, &N );

    /* Now find minimum rotation angle */

    rot[1] = 0.0;
    rot[2] = 10.0;
    rot[3] = 20.0;
    rot[4] = 30.0;
    rot[5] = 40.0;
    rot[6] = 50.0;
    rot[7] = 60.0;
    rot[8] = 70.0;
    rot[9] = 80.0;
    rot[10] = 90.0;
    rot[11] = 100.0;
    rot[12] = 110.0;
    rot[13] = 120.0;
    rot[14] = 130.0;
    rot[15] = 140.0;
    rot[16] = 150.0;
    rot[17] = 160.0;
    rot[18] = 170.0;
    rot[19] = 180.0;
    rot[20] = 190.0;
    rot[21] = 200.0;
    rot[22] = 210.0;
    rot[23] = 220.0;
    rot[24] = 230.0;
    rot[25] = 240.0;
    rot[26] = 250.0;
    rot[27] = 260.0;
    rot[28] = 270.0;
    rot[29] = 280.0;
    rot[30] = 290.0;
    rot[31] = 300.0;
    rot[32] = 310.0;
    rot[33] = 320.0;
    rot[34] = 330.0;
    rot[35] = 340.0;
    rot[36] = 350.0;

    Amin = NL_BIGD;

    for ( i = 1; i <= 36; i++ )
    {
        RA[i] = ST_CalcAreaSphereRotPtsLocal( rot[i] );

        if( RA[i]LT 0.0 )
            NL_OUT;

        if( RA[i]LT Amin )
        {
            Amin = RA[i];
            imin = i;
        }
    }

    kl = kr = -1;

    for ( i = imin - 1; i >= 1; i-- )
    {
        if( RA[i]GT Amin )
        {
            kl = i;
            break;
        }
    }

    if( kl GT - 1 )
    {
        for ( i = imin + 1; i <= 36; i++ )
        {
            if( RA[i]GT Amin )
            {
                kr = i;
                break;
            }
        }
    }

    rtol = tpc * Amin;

    if( kl EQ - 1 OR kr EQ - 1 )
    {
        alf = rot[imin];
    }
    else
    {
        N_FuncFindMinima( rot[kl], rot[imin], rot[kr], Amin, ST_CalcAreaSphereRotPtsLocal, rtol, 0.0, &alf, &dot );
    }

    /* Rotate point set about the N axis with alf */

    error = N_CreateRotationMatrixAboutAxis( NL_ZERO, N, alf, &rma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= n; i++ )
        N_TransformPt( Q[i], &rma, &R[i] );

    /* Get best sperical box */

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( R[i], &a, &b, &c );
        N_VectorCreate( a, b, 0.0, &RXY );

        error = N_VectorsAngle( R[i], NL_UNITZ, &u[i] );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( NL_ZERO, RXY, &c );

        if( c LT NL_MTOL )
        {
            v[i] = 0.0;
        }
        else
        {
            error = N_VectorsAngle( RXY, NL_UNITX, &v[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDot( RXY, NL_UNITY, &dot );

            if( dot LT 0.0 )
                v[i] = 360.0 - v[i];
        }

        vsort[i] = v[i];
    }

    N_ShellSortReal( vsort, n );

    vsort[n + 1] = 360.0 + vsort[0];

    a = b = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        dv = fabs( vsort[i + 1] - vsort[i] );

        if( dv GT b )
        {
            b = dv;
            iv = i;
        }
    }

    vs = vsort[iv + 1];
    ve = vsort[iv];

    if( vs GT ve )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( v[i]LE 360.0 AND v[i]GE vs )
                v[i] -= 360.0;
        }
    }

    us = 200.0;
    ue = -200.0;
    vs = 400.0;
    ve = -400.0;

    for ( i = 0; i <= n; i++ )
    {
        if( u[i]LT us )
            us = u[i];

        if( u[i]GT ue )
            ue = u[i];

        if( v[i]LT vs )
            vs = v[i];

        if( v[i]GT ve )
            ve = v[i];
    }

    /* Create local NURBS patch */

    c = vs * NL_RAD;
    a = r * cos( c );
    b = r * sin( c );

    N_VectorCreate( a, b, 0.0, &V );

    N_CrvInitArrays( &cur );
    error = N_CreateCircArc( NL_ZERO, NL_UNITZ, V, r, us, ue, ctp, &cur, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    a = ve - vs;

    error = N_CreateRevolvedSrf( &cur, NL_ZERO, NL_UNITZ, a, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Position patch to fit original points */

    a = -alf;

    error = N_SrfRotateAtPt( sur, NL_ZERO, N, a );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfTranslate( sur, C );

    *era = sea;
    *erm = sem;
    *sut = NL_NSPHERE;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFINTPPARAMS: Parametrization for surface interpolation in u/v-dir     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This  fitting routine computes papameter values for global surface 
     interpolation.  The  data  points are  assumed at  those parameter 
     values. In order to facilitate with both NL_POINT and NL_CPOINT objects,
     the routine employs  a NL_VOID pointer as input parameter and assumes
     that the calling routine typecasts the given pointer to the appro-
     priate  type. This  version  computes  either  u- or v-directional 
     parameters only. The calling mechanism works as follows:
 
       NL_CPOINT     **Pw;
       NL_POINT      **P;
       NL_INDEX      n, m;
       NL_PARAMETER  *t;
       ...
       (define data point array and allocate memory for t);
       ...
       N_FitSrfInterpParams((NL_VOID **)P ,n,m,NL_EPOINT,NL_CHORDLENGTH,NL_UDIR,t);
       N_FitSrfInterpParams((NL_VOID **)Pw,n,m,NL_HPOINT,NL_CENTRIPETAL,NL_VDIR,t);
 
     It is assumed that memory to store the parameter t is allocated in
     the calling routine. A recommended default for the parametrization 
     type is NL_CHORDLENGTH.
 
 
   ACCESS:
   
     A   , input  ,  NL_VOID pointer representing either NL_POINT or NL_CPOINT
     n,m , input  ,  Highest indexes in A
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in
     par , input  ,  Flag:
                       NL_UNIFORM    : Uniform parametrization
                       NL_CHORDLENGTH: Chord length parametrization
                       NL_CENTRIPETAL: Centripetal parametrization
     dir , input  ,  Flag:
                       NL_UDIR: Compute parameters in u-direction
                       NL_VDIR: Compute parameters in v-direction
     t   , output ,  Parameters corresponding to data points
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfInterpParams
  (NL_VOID    ** A,    /* in : grid of sample points, sized:[n+1][m+1] */
   NL_INDEX      n,    /* in : max 1st A index */
   NL_INDEX      m,    /* in : max 2nd A index */
   NL_FLAG       ptp,  /* in : object flag: NL_EPOINT = A contains NL_POINT objects  */
                       /*                   NL_HPOINT = a contains NL_CPOINT objects */
   NL_FLAG       par,  /* in : rule flag: NL_UNIFORM    : Uniform parametrization      */
                       /*                 NL_CHORDLENGTH: Chord length parametrization */
                       /*                 NL_CENTRIPETAL: Centripetal parametrization  */
   NL_FLAG       dir,  /* in : dir flag: NL_UDIR: Compute parameters in u-direction */
                       /*                NL_VDIR: Compute parameters in v-direction */
   NL_PARAMETER *t )   /* out: parameters associated with A points, sized:[n+1] */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfInterpParams");

    NL_INDEX i, j, num;

    NL_REAL *dist, sum, d;

    NL_POINT ** P = NULL;

    NL_CPOINT ** Pw = NULL;

    NL_STACKS SLocal;

    /* Start NURBS */

    N_InitNurbs( &SLocal );

    /* Check flags */

    switch( par )
    {
        case NL_UNIFORM:
            break;

        case NL_CHORDLENGTH:
            break;

        case NL_CENTRIPETAL:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    switch( ptp )
    {
        case NL_EPOINT:
            P = (NL_POINT ** )A;
            break;

        case NL_HPOINT:
            Pw = (NL_CPOINT ** )A;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    switch( dir )
    {
        case NL_UDIR:
            t[0] = 0.0;
            t[n] = 1.0;
            break;

        case NL_VDIR:
            t[0] = 0.0;
            t[m] = 1.0;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Compute uniform parametrization */

    if( par EQ NL_UNIFORM )
    {
        if( dir EQ NL_UDIR )
        {
            d = 1.0 / n;

            for ( i = 1; i < n; i++ )
                t[i] = i * d;
        }
        else
        {
            d = 1.0 / m;

            for ( j = 1; j < m; j++ )
                t[j] = j * d;
        }

        NL_OUT;
    }

    /* Compute chord length or centripetal parametrization */

    dist = N_AllocReal1dArray( NL_MAX( n, m ), &SLocal );

    if( dist EQ NULL )
        NL_QUIT;

    if( dir EQ NL_UDIR )
    {
        for ( i = 1; i < n; i++ )
            t[i] = 0.0;

        num = m + 1;

        for ( j = 0; j <= m; j++ )
        {
            sum = 0.0;

            for ( i = 1; i <= n; i++ )
            {
                if( ptp EQ NL_EPOINT )
                    N_DistPtPt( P[i - 1][j], P[i][j], &dist[i] );
                else
                    N_DistCptCptHomo( Pw[i - 1][j], Pw[i][j], &dist[i] );

                if( par EQ NL_CENTRIPETAL )
                    dist[i] = sqrt( dist[i] );
                sum += dist[i];
            }

            if( sum GT NL_MTOL )
            {
                d = 0.0;

                for ( i = 1; i < n; i++ )
                {
                    d += dist[i];
                    t[i] += d / sum;
                }
            }
            else
                num--;
        }

        if( num EQ 0 )
            NL_ERROR( NL_INP_ERR );

        for ( i = 1; i < n; i++ )
            t[i] /= num;
    }
    else
    {
        for ( j = 1; j < m; j++ )
            t[j] = 0.0;

        num = n + 1;

        for ( i = 0; i <= n; i++ )
        {
            sum = 0.0;

            for ( j = 1; j <= m; j++ )
            {
                if( ptp EQ NL_EPOINT )
                    N_DistPtPt( P[i][j - 1], P[i][j], &dist[j] );
                else
                    N_DistCptCptHomo( Pw[i][j - 1], Pw[i][j], &dist[j] );

                if( par EQ NL_CENTRIPETAL )
                    dist[j] = sqrt( dist[j] );
                sum += dist[j];
            }

            if( sum GT NL_MTOL )
            {
                d = 0.0;

                for ( j = 1; j < m; j++ )
                {
                    d += dist[j];
                    t[j] += d / sum;
                }
            }
            else
                num--;
        }

        if( num EQ 0 )
            NL_ERROR( NL_INP_ERR );

        for ( j = 1; j < m; j++ )
            t[j] /= num;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SLocal );

    return (NL_NO);
}

/**********************************************************************/
/* N_FITSRFAPPROXREMOVEKNOTS: Remove all removable knots from an approximating surface */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine removes  all removable  knots  from a NURBS
     surface  approximating, at supplied parameter values, a given set
     of points. An  error matrix is  updated as knots are removed from
     the surface in place, i.e. the  original surface  is destroyed. A
     typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    mu, mv;
       NL_REAL     **er, *u, *v, E;
       ...
       (define sur, get u, v, er);
       ...
       N_FitSrfApproxRemoveKnots(&sur,u,v,er,mu,mv,E,NL_UVDIR);

     This routine  is used in N_FITSRFAPPROXTOL to remove knots from approxima-
     ting  surfaces. Knots  can  be  removed  in  u-,  v-  or  in both 
     directions.


   ACCESS:
   
     sur   , in/out ,  NURBS surface
     u,v   , input  ,  Parameter at which error is checked
     er    , input  ,  An (mu x mv) error matrix to be updated 
     mu,mv , input  ,  Highest indexes in u and v
     E     , input  ,  Error tolerance
     dir   , input  ,  NL_FLAG:
                         NL_UDIR : remove knots in u-direction
                         NL_VDIR : remove knots in v-direction
                         NL_UVDIR: remove knots in both directions


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfApproxRemoveKnots
  (NL_SURFACE *sur,       /* i/o: target surface */
   NL_REAL    *uParams,   /* in : u param values at which error is checked, sized:[mu+1] */
   NL_REAL    *vParams,   /* in : v param values at which error is checker, sized:[mv+1] */
   NL_REAL  ** er,        /* out: error matrix for each check point, sized:[mu+1][mv+1] */
   NL_INDEX    mu,        /* in : highest index in uParams */
   NL_INDEX    mv,        /* in : highest index in vParams */
   NL_REAL     E,         /* in : error tolerance */
   NL_FLAG     dir )      /* in : target flag: NL_UDIR  = remove knots in u-direction     */
                          /*                   NL_VDIR  = remove knots in v-direction     */
                          /*                   NL_UVDIR = remove knots in both directions */
{

    NL_FLAG krm, rmf, errorFlag = NL_NO;

    NL_INDEX *uleft = NULL, *uright = NULL, *vleft = NULL, *vright = NULL, *sru = NULL, *srv = NULL, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru = 0, su = 0, rv = 0, sv = 0, a;

    NL_DEGREE p, q;

    NL_REAL ** te, ** bru = NULL, ** brv = NULL, *U, *V, *alf, *oma, *bet, *omb, *mru = NULL, *mrv = NULL, *fuv, lam = 0.0, oml = 0.0, al, be, ob, bu = 0.0, bv = 0.0, bf, N1, tmp, fl;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, *Rw;

    NL_CFUN cfu, cfv;

    NL_STACKS SLocal;

    NL_REAL NOREM = 1.0e+25;

    /* Start NURBS */

    N_InitNurbs( &SLocal );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Allocate local memory */

    i = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * i, &SLocal );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * i, &SLocal );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * i, &SLocal );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * i, &SLocal );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( 2 * i, &SLocal );

    if( Rw EQ NULL )
        NL_QUIT;

    fuv = N_AllocReal1dArray( NL_MAX( n, m ), &SLocal );

    if( fuv EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( mu, mv, &SLocal );

    if( te EQ NULL )
        NL_QUIT;

    /* Set error matrix to 0s */
    for ( ii = 0; ii < mu; ++ii )
        for ( jj = 0; jj < mv; ++jj )
            er[ii][jj] = 0;

    /* Set error matrix to 0s */
    for ( ii = 0 ; ii < mu ; ++ii )
        for ( jj = 0; jj < mv; ++jj )
            te[ii][jj] = 0;

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        mru = N_AllocReal1dArray( r, &SLocal );

        if( mru EQ NULL )
            NL_QUIT;

        sru = N_AllocInt1dArray( r, &SLocal );

        if( sru EQ NULL )
            NL_QUIT;

        uleft = N_AllocInt1dArray( n, &SLocal );

        if( uleft EQ NULL )
            NL_QUIT;

        uright = N_AllocInt1dArray( n, &SLocal );

        if( uright EQ NULL )
            NL_QUIT;

        bru = N_AllocReal2dArray( r, m, &SLocal );

        if( bru EQ NULL )
            NL_QUIT;
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        mrv = N_AllocReal1dArray( s, &SLocal );

        if( mrv EQ NULL )
            NL_QUIT;

        srv = N_AllocInt1dArray( s, &SLocal );

        if( srv EQ NULL )
            NL_QUIT;

        vleft = N_AllocInt1dArray( m, &SLocal );

        if( vleft EQ NULL )
            NL_QUIT;

        vright = N_AllocInt1dArray( m, &SLocal );

        if( vright EQ NULL )
            NL_QUIT;

        brv = N_AllocReal2dArray( n, s, &SLocal );

        if( brv EQ NULL )
            NL_QUIT;
    }

    /* Initialize */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        for ( i = 0; i <= r; i++ )
        {
            mru[i] = NL_BIGD;
            sru[i] = 0;
        }

        for ( i = 0; i <= r; i++ )
        {
            for ( j = 0; j <= m; j++ )
                bru[i][j] = 0.0;
        }

        N_CFuncFromKnots( &cfv, fuv, m, q, V, s, &SLocal );
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        for ( j = 0; j <= s; j++ )
        {
            mrv[j] = NL_BIGD;
            srv[j] = 0;
        }

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= s; j++ )
                brv[i][j] = 0.0;
        }

        N_CFuncFromKnots( &cfu, fuv, n, p, U, r, &SLocal );
    }

    /* Compute the maximum of knot removal errors for each */
    /* distinct knot and get ranges of parameter indexes   */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        ru = p + 1;

        while( ru LE n )
        {
            i = ru;

            while( ru LE n AND U[ru]EQ U[ru + 1] )
                ru++;
            sru[ru] = ru - i + 1;

            errorFlag = N_FitSrfRemovalBoundary( sur, ru, sru[ru], 0, m, NL_UDIR, bru, &mru[ru] );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            ru++;
        }

        k = 1;

        for ( i = 0; i <= n; i++ )
        {
            uleft[i] = k;

            while( k LT mu AND uParams[k]GT U[i]AND uParams[k]LE U[i + 1] )
                k++;

            for ( l = k; l < mu; l++ )
            {
                if( uParams[l]GE U[i + p + 1] )
                    break;
            }
            uright[i] = l - 1;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        rv = q + 1;

        while( rv LE m )
        {
            i = rv;

            while( rv LE m AND V[rv]EQ V[rv + 1] )
                rv++;
            srv[rv] = rv - i + 1;

            errorFlag = N_FitSrfRemovalBoundary( sur, rv, srv[rv], 0, n, NL_VDIR, brv, &mrv[rv] );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            rv++;
        }

        k = 1;

        for ( j = 0; j <= m; j++ )
        {
            vleft[j] = k;

            while( k LT mv AND vParams[k]GT V[j]AND vParams[k]LE V[j + 1] )
                k++;

            for ( l = k; l < mv; l++ )
            {
                if( vParams[l]GE V[j + q + 1] )
                    break;
            }
            vright[j] = l - 1;
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
                for ( j = 0; j <= m; j++ )
                    fuv[j] = bru[ru][j];

                rmf = NL_TRUE;

                if( (p + su) % 2 )
                {
                    k = (p + su + 1) / 2;
                    i = ru - k;
                    al = (U[ru] - U[i]) / (U[i + p + 1] - U[i]);
                    be = (U[ru] - U[i + 1]) / (U[i + p + 2] - U[i + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    for ( k = uleft[i]; k <= uright[i + 1]; k++ )
                    {
                      errorFlag = N_BasisIEval( knu, i, p, uParams[k], NL_LEFT, &bf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        errorFlag = N_BasisIEval( knu, i + 1, p, uParams[k], NL_LEFT, &N1 );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = fabs( lam * al * bf - oml * ob * N1 );

                        for ( l = 0; l <= mv; l++ )
                        {
                          errorFlag = N_CFuncEval( &cfv, vParams[l], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + tmp * fl;

                            if( te[k][l]GT E )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }
                else
                {
                    k = (p + su) / 2;
                    i = ru - k;

                    for ( k = uleft[i]; k <= uright[i]; k++ )
                    {
                      errorFlag = N_BasisIEval( knu, i, p, uParams[k], NL_LEFT, &bf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        for ( l = 0; l <= mv; l++ )
                        {
                          errorFlag = N_CFuncEval( &cfv, vParams[l], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + bf * fl;

                            if( te[k][l]GT E )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }

                /* If error test passed, update error vector and remove knot */

                if( rmf EQ NL_TRUE )
                {
                    if( (p + su) % 2 )
                        a = uright[i + 1];
                    else
                        a = uright[i];

                    for ( k = uleft[i]; k <= a; k++ )
                    {
                        for ( l = 0; l <= mv; l++ )
                            er[k][l] = te[k][l];
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

                    for ( col = 0; col <= m; col++ )
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

                        /* Save new control points */

                        if( (p + su) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                        }

                        i = first;
                        j = last;

                        while( (j - i)GT 0 )
                        {
                            N_CopyCPt( Rw[i - off], &Pw[i][col] );
                            N_CopyCPt( Rw[j - off], &Pw[j][col] );
                            i++;
                            j--;
                        }
                    } /* End for each row */

                    /* Successful removal -> shift down some entinties */

                    if( su GT 1 )
                        sru[ru - 1] = sru[ru] - 1;

                    for ( i = ru + 1; i <= r; i++ )
                    {
                        mru[i - 1] = mru[i];
                        sru[i - 1] = sru[i];
                        U[i - 1] = U[i];

                        for ( j = 0; j <= m; j++ )
                            bru[i - 1][j] = bru[i][j];
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
                    N_SrfSetSizeIndices( sur, n, m, p, q, r, s );

                    if( dir EQ NL_UVDIR )
                        N_CFuncSetSizeIndices( &cfu, n, p, r );

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
                        if( U[i]NEQ U[i + 1]AND mru[i]NEQ NOREM )
                        {
                          errorFlag = N_FitSrfRemovalBoundary( sur, i, sru[i], 0, m, NL_UDIR, bru, &mru[i] );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( j = q + 1; j <= m; j++ )
                        {
                            if( V[j]NEQ V[j + 1]AND mrv[j]NEQ NOREM )
                            {
                              errorFlag = N_FitSrfRemovalBoundary( sur, j, srv[j], first, last, NL_VDIR, brv, &mrv[j] );

                                if(errorFlag EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }

                    /* Update index ranges in u-direction */

                    for ( i = ru - p - 1; i <= ru - su; i++ )
                    {
                        for ( k = uright[i] + 1; k <= mu; k++ )
                        {
                            if( uParams[k]GE U[i + p + 1] )
                                break;
                        }
                        uright[i] = k - 1;
                    }

                    for ( i = ru - su + 1; i <= n; i++ )
                    {
                        uleft[i] = uleft[i + 1];
                        uright[i] = uright[i + 1];
                    }
                }
                else
                {
                    /* Knot is not removable */

                    mru[ru] = NOREM;
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */
                for ( i = 0; i <= n; i++ )
                    fuv[i] = brv[i][rv];

                rmf = NL_TRUE;

                if( (q + sv) % 2 )
                {
                    k = (q + sv + 1) / 2;
                    j = rv - k;
                    al = (V[rv] - V[j]) / (V[j + q + 1] - V[j]);
                    be = (V[rv] - V[j + 1]) / (V[j + q + 2] - V[j + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    for ( l = vleft[j]; l <= vright[j + 1]; l++ )
                    {
                      errorFlag = N_BasisIEval( knv, j, q, vParams[l], NL_LEFT, &bf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        errorFlag = N_BasisIEval( knv, j + 1, q, vParams[l], NL_LEFT, &N1 );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = fabs( lam * al * bf - oml * ob * N1 );

                        for ( k = 0; k <= mu; k++ )
                        {
                          errorFlag = N_CFuncEval( &cfu, uParams[k], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + tmp * fl;

                            if( te[k][l]GT E )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }
                else
                {
                    k = (q + sv) / 2;
                    j = rv - k;

                    for ( l = vleft[j]; l <= vright[j]; l++ )
                    {
                      errorFlag = N_BasisIEval( knv, j, q, vParams[l], NL_LEFT, &bf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        for ( k = 0; k <= mu; k++ )
                        {
                          errorFlag = N_CFuncEval( &cfu, uParams[k], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + bf * fl;

                            if( te[k][l]GT E )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }

                /* If error test passed, update error vector and remove knot */

                if( rmf EQ NL_TRUE )
                {
                    if( (q + sv) % 2 )
                        a = vright[j + 1];
                    else
                        a = vright[j];

                    for ( l = vleft[j]; l <= a; l++ )
                    {
                        for ( k = 0; k <= mu; k++ )
                            er[k][l] = te[k][l];
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

                    /* Remove the knot for each column */

                    for ( row = 0; row <= n; row++ )
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

                        /* Save new control points */

                        if( (q + sv) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                        }

                        i = first;
                        j = last;

                        while( (j - i)GT 0 )
                        {
                            N_CopyCPt( Rw[i - off], &Pw[row][i] );
                            N_CopyCPt( Rw[j - off], &Pw[row][j] );
                            i++;
                            j--;
                        }
                    } /* End for each row */

                    /* Successful removal -> shift down some entinties */

                    if( sv GT 1 )
                        srv[rv - 1] = srv[rv] - 1;

                    for ( j = rv + 1; j <= s; j++ )
                    {
                        mrv[j - 1] = mrv[j];
                        srv[j - 1] = srv[j];
                        V[j - 1] = V[j];

                        for ( i = 0; i <= n; i++ )
                            brv[i][j - 1] = brv[i][j];
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
                    N_SrfSetSizeIndices( sur, n, m, p, q, r, s );

                    if( dir EQ NL_UVDIR )
                        N_CFuncSetSizeIndices( &cfv, m, q, s );

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
                        if( V[j]NEQ V[j + 1]AND mrv[j]NEQ NOREM )
                        {
                          errorFlag = N_FitSrfRemovalBoundary( sur, j, srv[j], 0, n, NL_VDIR, brv, &mrv[j] );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    if( dir EQ NL_UVDIR )
                    {
                        for ( i = p + 1; i <= n; i++ )
                        {
                            if( U[i]NEQ U[i + 1]AND mru[i]NEQ NOREM )
                            {
                              errorFlag = N_FitSrfRemovalBoundary( sur, i, sru[i], first, last, NL_UDIR, bru, &mru[i] );

                                if(errorFlag EQ NL_YES )
                                    NL_OUT;
                            }
                        }
                    }

                    /* Update index ranges in u-direction */

                    for ( j = rv - q - 1; j <= rv - sv; j++ )
                    {
                        for ( k = vright[j] + 1; k <= mv; k++ )
                        {
                            if( vParams[k]GE V[j + q + 1] )
                                break;
                        }
                        vright[j] = k - 1;
                    }

                    for ( j = rv - sv + 1; j <= m; j++ )
                    {
                        vleft[j] = vleft[j + 1];
                        vright[j] = vright[j + 1];
                    }
                }
                else
                {
                    /* Knot is not removable */

                    mrv[rv] = NOREM;
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SLocal );

    return (errorFlag);
}

/**********************************************************************/
/* N_FitSrfRemovalBoundary: Update surface removal bound for surface fitting         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine updates surface removal error  bound for one
     removal step in either u- or v-direction. That is, given the knot
     index "r",  multiplicity "s", the  knot T[r], T is either U or V, 
     is removed one time  in  either direction, and the bounds and the 
     maximum  error  are  updated. It is  assumed that (1) T[r] is  an 
     interior knot, (2) T[r] != T[r+1] and (3) the multiplicity of the  
     knot is "s>0". A typical calling example is:


       NL_SURFACE  sur;
       NL_INDEX    r, s, f, l;
       NL_REAL     **br, mr;
       ...
       (define sur, get r, s, f and l);
       ...
       N_FitSrfRemovalBoundary(&sur,r,s,f,l,NL_UDIR,br,&mr);

     THE ROUTINE DOES NOT CHECK FOR THE PROPER NL_INDEX AND  MULTIPLICITY
     OF THE KNOT. IT  ASSUMES THAT  THEY ARE  CORRECT. IF  mr IS TO BE 
     INITIALIZED LOCALLY, mr = NL_BIGD IS CHECKED.


   ACCESS:
   
     sur , input  ,  NURBS surface
     r,s , input  ,  Index  and  multiplicity  of  knot to be  removed
                     T[r] != T[r+1] must hold (T is either U or V)
     f,l , input  ,  First and last  row/column  to be used  to update
                     the error
     dir , input  ,  Flag:
                       NL_UDIR: Remove in u-direction
                       NL_VDIR: Remove in v-direction
     br  , in/out ,  Knot removal error at each (u,v) knot pair 
     mr  , in/out ,  Maximum  error (if mr = NL_BIGD, it  is  initialized
                     locally)
                      


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfRemovalBoundary( NL_SURFACE *sur, NL_INDEX r, NL_INDEX s, NL_INDEX f, NL_INDEX l, NL_FLAG dir, NL_REAL ** br, NL_REAL *mr )
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfRemovalBoundary");

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

                br[r][col] += dw;

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

                br[row][r] += dw;

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

    return (NL_NO);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFINTPREMOVEKNOTS: Remove all removable knots from an interpolating surface */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting  routine removes  all removable  knots  from a NURBS
     surface  interpolating, at supplied parameter values, a given set
     of  points. The  knots are removed in  place, i.e.  the  original 
     surface is destroyed. A typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    mu, mv;
       NL_REAL     *u, *v, tlu, tlv, tls;
       ...
       (define sur, get u, v, tlu, tlv and tls);
       ...
       N_FitSrfInterpRemoveKnots(&sur,u,v,mu,mv,tlu,tlv,tls);


   ACCESS:
   
     sur   , in/out ,  NURBS surface
     u,v   , input  ,  Parameter at which error is checked
     mu,mv , input  ,  Highest indexes in u and v
     tlu   , input  ,  Error tolerance along u-boundary
     tlv   , input  ,  Error tolerance along v-boundary
     tls   , input  ,  Over all error tolerance


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfInterpRemoveKnots 
  (NL_SURFACE *sur,        /* in : target surface */ 
   NL_REAL    *uParams,    /* in : u params at which error is checked, sized:[mu+1][mv+1] */ 
   NL_REAL    *vParams,    /* in : v params at which error is checked, sized:[mu+1][mv+1] */
   NL_INDEX    mu,         /* in : highest 1st u and v index */ 
   NL_INDEX    mv,         /* in : highest 2nd u and v index */ 
   NL_REAL     tlu,        /* in : max distance surface points may move along u-boundary */ 
   NL_REAL     tlv,        /* in : max distance surface points may move along v-boundary */ 
   NL_REAL     tls )       /* in : max distance any specified surface uv point may move */                   
{

    NL_FLAG krm, rmf, errorFlag = NL_NO;

    NL_INDEX *uleft, *uright, *vleft, *vright, *sru, *srv, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru, su, rv, sv, a;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, ** bru, ** brv, *U, *V, *alf, *oma, *bet, *omb, *mru, *mrv, *fuv, lam = 0.0, oml = 0.0, al, be, ob, bu, bv, Nbf, N1, tmp, fl;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, *Rw;

    NL_CFUN cfu, cfv;

    NL_STACKS SLocal;

    NL_REAL NOREM = 1.0e+25;

    /* Start NURBS */

    N_InitNurbs( &SLocal );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Allocate local memory */

    i = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * i, &SLocal );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * i, &SLocal );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * i, &SLocal );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * i, &SLocal );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( 2 * i, &SLocal );

    if( Rw EQ NULL )
        NL_QUIT;

    fuv = N_AllocReal1dArray( NL_MAX( n, m ), &SLocal );

    if( fuv EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( mu, mv, &SLocal );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( mu, mv, &SLocal );

    if( te EQ NULL )
        NL_QUIT;

    mru = N_AllocReal1dArray( r, &SLocal );

    if( mru EQ NULL )
        NL_QUIT;

    sru = N_AllocInt1dArray( r, &SLocal );

    if( sru EQ NULL )
        NL_QUIT;

    uleft = N_AllocInt1dArray( n, &SLocal );

    if( uleft EQ NULL )
        NL_QUIT;

    uright = N_AllocInt1dArray( n, &SLocal );

    if( uright EQ NULL )
        NL_QUIT;

    bru = N_AllocReal2dArray( r, m, &SLocal );

    if( bru EQ NULL )
        NL_QUIT;

    mrv = N_AllocReal1dArray( s, &SLocal );

    if( mrv EQ NULL )
        NL_QUIT;

    srv = N_AllocInt1dArray( s, &SLocal );

    if( srv EQ NULL )
        NL_QUIT;

    vleft = N_AllocInt1dArray( m, &SLocal );

    if( vleft EQ NULL )
        NL_QUIT;

    vright = N_AllocInt1dArray( m, &SLocal );

    if( vright EQ NULL )
        NL_QUIT;

    brv = N_AllocReal2dArray( n, s, &SLocal );

    if( brv EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= mv; j++ )
            er[i][j] = 0.0;
    }

    for ( i = 0; i <= r; i++ )
    {
        mru[i] = NL_BIGD;
        sru[i] = 0;
    }

    for ( i = 0; i <= r; i++ )
    {
        for ( j = 0; j <= m; j++ )
            bru[i][j] = 0.0;
    }

    N_CFuncFromKnots( &cfv, fuv, m, q, V, s, &SLocal );

    for ( j = 0; j <= s; j++ )
    {
        mrv[j] = NL_BIGD;
        srv[j] = 0;
    }

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= s; j++ )
            brv[i][j] = 0.0;
    }

    N_CFuncFromKnots( &cfu, fuv, n, p, U, r, &SLocal );

    /* Compute the maximum of knot removal errors for each */
    /* distinct knot and get ranges of parameter indexes   */

    ru = p + 1;

    while( ru LE n )
    {
        i = ru;

        while( ru LE n AND U[ru]EQ U[ru + 1] )
            ru++;
        sru[ru] = ru - i + 1;

        errorFlag = N_FitSrfRemovalBoundary( sur, ru, sru[ru], 0, m, NL_UDIR, bru, &mru[ru] );

        if(errorFlag EQ NL_YES )
            NL_OUT;

        ru++;
    }

    k = 1;

    for ( i = 0; i <= n; i++ )
    {
        uleft[i] = k;

        while( k LT mu AND uParams[k]GT U[i]AND uParams[k]LE U[i + 1] )
            k++;

        for ( l = k; l < mu; l++ )
        {
            if( uParams[l]GE U[i + p + 1] )
                break;
        }
        uright[i] = l - 1;
    }

    rv = q + 1;

    while( rv LE m )
    {
        i = rv;

        while( rv LE m AND V[rv]EQ V[rv + 1] )
            rv++;
        srv[rv] = rv - i + 1;

        errorFlag = N_FitSrfRemovalBoundary( sur, rv, srv[rv], 0, n, NL_VDIR, brv, &mrv[rv] );

        if(errorFlag EQ NL_YES )
            NL_OUT;

        rv++;
    }

    k = 1;

    for ( j = 0; j <= m; j++ )
    {
        vleft[j] = k;

        while( k LT mv AND vParams[k]GT V[j]AND vParams[k]LE V[j + 1] )
            k++;

        for ( l = k; l < mv; l++ )
        {
            if( vParams[l]GE V[j + q + 1] )
                break;
        }
        vright[j] = l - 1;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

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

        /* If no more removable knot -> finished */

        if( (bu EQ NL_BIGD OR bu EQ NOREM)AND( bv EQ NL_BIGD OR bv EQ NOREM ) )
            break;

        if( bu LT bv )
            krm = NL_UDIR;
        else
            krm = NL_VDIR;

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR: /* Remove in the u-direction */
                for ( j = 0; j <= m; j++ )
                    fuv[j] = bru[ru][j];

                rmf = NL_TRUE;

                if( (p + su) % 2 )
                {
                    k = (p + su + 1) / 2;
                    i = ru - k;
                    al = (U[ru] - U[i]) / (U[i + p + 1] - U[i]);
                    be = (U[ru] - U[i + 1]) / (U[i + p + 2] - U[i + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    for ( k = uleft[i]; k <= uright[i + 1]; k++ )
                    {
                      errorFlag = N_BasisIEval( knu, i, p, uParams[k], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        errorFlag = N_BasisIEval( knu, i + 1, p, uParams[k], NL_LEFT, &N1 );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = fabs( lam * al * Nbf - oml * ob * N1 );

                        for ( l = 0; l <= mv; l++ )
                        {
                          errorFlag = N_CFuncEval( &cfv, vParams[l], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + tmp * fl;

                            if( te[k][l]GT tls )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( te[k][0]GT tlu )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( te[k][mv]GT tlu )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }
                else
                {
                    k = (p + su) / 2;
                    i = ru - k;

                    for ( k = uleft[i]; k <= uright[i]; k++ )
                    {
                      errorFlag = N_BasisIEval( knu, i, p, uParams[k], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        for ( l = 0; l <= mv; l++ )
                        {
                          errorFlag = N_CFuncEval( &cfv, vParams[l], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + Nbf * fl;

                            if( te[k][l]GT tls )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( te[k][0]GT tlu )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( te[k][mv]GT tlu )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }

                /* If error test passed, update error vector and remove knot */

                if( rmf EQ NL_TRUE )
                {
                    if( (p + su) % 2 )
                        a = uright[i + 1];
                    else
                        a = uright[i];

                    for ( k = uleft[i]; k <= a; k++ )
                    {
                        for ( l = 0; l <= mv; l++ )
                            er[k][l] = te[k][l];
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

                    for ( col = 0; col <= m; col++ )
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

                        /* Save new control points */

                        if( (p + su) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                        }

                        i = first;
                        j = last;

                        while( (j - i)GT 0 )
                        {
                            N_CopyCPt( Rw[i - off], &Pw[i][col] );
                            N_CopyCPt( Rw[j - off], &Pw[j][col] );
                            i++;
                            j--;
                        }
                    } /* End for each row */

                    /* Successful removal -> shift down some entinties */

                    if( su GT 1 )
                        sru[ru - 1] = sru[ru] - 1;

                    for ( i = ru + 1; i <= r; i++ )
                    {
                        mru[i - 1] = mru[i];
                        sru[i - 1] = sru[i];
                        U[i - 1] = U[i];

                        for ( j = 0; j <= m; j++ )
                            bru[i - 1][j] = bru[i][j];
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
                    N_SrfSetSizeIndices( sur, n, m, p, q, r, s );
                    N_CFuncSetSizeIndices( &cfu, n, p, r );

                    /* If no more internal knots -> finished */

                    if( n EQ p AND m EQ q )
                        break;

                    /* Update error bounds */

                    k = NL_MAX( ru - p, p + 1 );
                    l = NL_MIN( n, ru + p - su );

                    for ( i = k; i <= l; i++ )
                    {
                        if( U[i]NEQ U[i + 1]AND mru[i]NEQ NOREM )
                        {
                          errorFlag = N_FitSrfRemovalBoundary( sur, i, sru[i], 0, m, NL_UDIR, bru, &mru[i] );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    for ( j = q + 1; j <= m; j++ )
                    {
                        if( V[j]NEQ V[j + 1]AND mrv[j]NEQ NOREM )
                        {
                          errorFlag = N_FitSrfRemovalBoundary( sur, j, srv[j], first, last, NL_VDIR, brv, &mrv[j] );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    /* Update index ranges in u-direction */

                    for ( i = ru - p - 1; i <= ru - su; i++ )
                    {
                        for ( k = uright[i] + 1; k <= mu; k++ )
                        {
                            if( uParams[k]GE U[i + p + 1] )
                                break;
                        }
                        uright[i] = k - 1;
                    }

                    for ( i = ru - su + 1; i <= n; i++ )
                    {
                        uleft[i] = uleft[i + 1];
                        uright[i] = uright[i + 1];
                    }
                }
                else
                {
                    /* Knot is not removable */

                    mru[ru] = NOREM;
                }
                break;

            case NL_VDIR: /* Remove in the v-direction */
                for ( i = 0; i <= n; i++ )
                    fuv[i] = brv[i][rv];

                rmf = NL_TRUE;

                if( (q + sv) % 2 )
                {
                    k = (q + sv + 1) / 2;
                    j = rv - k;
                    al = (V[rv] - V[j]) / (V[j + q + 1] - V[j]);
                    be = (V[rv] - V[j + 1]) / (V[j + q + 2] - V[j + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    for ( l = vleft[j]; l <= vright[j + 1]; l++ )
                    {
                      errorFlag = N_BasisIEval( knv, j, q, vParams[l], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        errorFlag = N_BasisIEval( knv, j + 1, q, vParams[l], NL_LEFT, &N1 );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = fabs( lam * al * Nbf - oml * ob * N1 );

                        for ( k = 0; k <= mu; k++ )
                        {
                          errorFlag = N_CFuncEval( &cfu, uParams[k], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + tmp * fl;

                            if( te[k][l]GT tls )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( te[0][l]GT tlv )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( te[mu][l]GT tlv )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }
                else
                {
                    k = (q + sv) / 2;
                    j = rv - k;

                    for ( l = vleft[j]; l <= vright[j]; l++ )
                    {
                      errorFlag = N_BasisIEval( knv, j, q, vParams[l], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        for ( k = 0; k <= mu; k++ )
                        {
                          errorFlag = N_CFuncEval( &cfu, uParams[k], NL_LEFT, &fl );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;

                            te[k][l] = er[k][l] + Nbf * fl;

                            if( te[k][l]GT tls )
                            {
                                rmf = NL_FALSE;
                                break;
                            }
                        }

                        if( te[0][l]GT tlv )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( te[mu][l]GT tlv )
                        {
                            rmf = NL_FALSE;
                            break;
                        }

                        if( rmf EQ NL_FALSE )
                            break;
                    }
                }

                /* If error test passed, update error vector and remove knot */

                if( rmf EQ NL_TRUE )
                {
                    if( (q + sv) % 2 )
                        a = vright[j + 1];
                    else
                        a = vright[j];

                    for ( l = vleft[j]; l <= a; l++ )
                    {
                        for ( k = 0; k <= mu; k++ )
                            er[k][l] = te[k][l];
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

                    /* Remove the knot for each column */

                    for ( row = 0; row <= n; row++ )
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

                        /* Save new control points */

                        if( (q + sv) % 2 )
                        {
                            N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                        }

                        i = first;
                        j = last;

                        while( (j - i)GT 0 )
                        {
                            N_CopyCPt( Rw[i - off], &Pw[row][i] );
                            N_CopyCPt( Rw[j - off], &Pw[row][j] );
                            i++;
                            j--;
                        }
                    } /* End for each row */

                    /* Successful removal -> shift down some entinties */

                    if( sv GT 1 )
                        srv[rv - 1] = srv[rv] - 1;

                    for ( j = rv + 1; j <= s; j++ )
                    {
                        mrv[j - 1] = mrv[j];
                        srv[j - 1] = srv[j];
                        V[j - 1] = V[j];

                        for ( i = 0; i <= n; i++ )
                            brv[i][j - 1] = brv[i][j];
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
                    N_SrfSetSizeIndices( sur, n, m, p, q, r, s );
                    N_CFuncSetSizeIndices( &cfv, m, q, s );

                    /* If no more internal knots -> finished */

                    if( n EQ p AND m EQ q )
                        break;

                    /* Update error bounds */

                    k = NL_MAX( rv - q, q + 1 );
                    l = NL_MIN( m, rv + q - sv );

                    for ( j = k; j <= l; j++ )
                    {
                        if( V[j]NEQ V[j + 1]AND mrv[j]NEQ NOREM )
                        {
                          errorFlag = N_FitSrfRemovalBoundary( sur, j, srv[j], 0, n, NL_VDIR, brv, &mrv[j] );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    for ( i = p + 1; i <= n; i++ )
                    {
                        if( U[i]NEQ U[i + 1]AND mru[i]NEQ NOREM )
                        {
                          errorFlag = N_FitSrfRemovalBoundary( sur, i, sru[i], first, last, NL_UDIR, bru, &mru[i] );

                            if(errorFlag EQ NL_YES )
                                NL_OUT;
                        }
                    }

                    /* Update index ranges in u-direction */

                    for ( j = rv - q - 1; j <= rv - sv; j++ )
                    {
                        for ( k = vright[j] + 1; k <= mv; k++ )
                        {
                            if( vParams[k]GE V[j + q + 1] )
                                break;
                        }
                        vright[j] = k - 1;
                    }

                    for ( j = rv - sv + 1; j <= m; j++ )
                    {
                        vleft[j] = vleft[j + 1];
                        vright[j] = vright[j + 1];
                    }
                }
                else
                {
                    /* Knot is not removable */

                    mrv[rv] = NOREM;
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SLocal );

    return (errorFlag);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFAPPROXREMOVEKNOTSTANGENTS: Remove all removable knots from an approximating surface */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  removes  all  removable  knots  from a NURBS
     surface  approximating,  at  supplied parameter values, a given set
     of  points  with   possible  tangent  constraints.  The  knots  are 
     removed  in  place,  i.e. the  original  surface  is  destroyed.  A 
     typical calling example is:
 
       NL_SURFACE  sur;
       NL_VECTOR   ***Tu, ***Tv;
       NL_INDEX    mu, mv;
       NL_REAL     *u, *v, emx, eub, evb, eut, evt;
       ...
       (define sur, get u, v and the tolerances);
       ...
       N_FitSrfApproxRemoveKnotsTangents(&sur,u,v,Tu,Tv,mu,mv,NL_START,NL_END,emx,eub,evb,eut,evt);
 
     This routine is used in N_FITSRFAPPROXTANGENTSTOL to remove knots from approximating 
 
     surfaces.
 
 
   ACCESS:
   
     sur   , in/out ,  NURBS surface
     u,v   , input  ,  Parameter at which error is checked
     Tu,Tv , input  ,  Tangent constraints
     mu,mv , input  ,  Highest indexes in u, v, Tu and Tv
     ufl   , input  ,  Flag:
                         NL_NO   : no u-tangents specified
                         NL_START: remove for u=umin cross-tangents
                         NL_END  : remove for u=umax cross-tangents
                         NL_BOTH : remove for both cross-tangents
     vfl   , input  ,  Flag:
                         NL_NO   : no v-tangents specified
                         NL_START: remove for v=vmin cross-tangents
                         NL_END  : remove for v=vmax cross-tangents
                         NL_BOTH : remove for both cross-tangents
     emx  , input  ,  Overall error tolerance, i.e. the new surface will
                      not deviate from the origonal surface by more than
                      emx
     eub  , input  ,  Error  tolerance  along  the   v=vmin  and  v=vmax 
                      boundaries, i.e. for u-knot removal
     evb  , input  ,  Error  tolerance  along  the   u=umin  and  u=umax 
                      boundaries, i.e. for v-knot removal
     eut  , input  ,  Error  tolerance for  u-tangents on the u=umin and 
                      u=umax boundaries
     evt  , input  ,  Error tolerance  for  v-tangents on the v=vmin and 
                      v=vmax  boundaries
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfApproxRemoveKnotsTangents 
  (NL_SURFACE  * sur,      /* i/o: target surface */ 
   NL_REAL     * uParams,  /* in : u params at which errors occur, sized:[mu+1][mv+1] */ 
   NL_REAL     * vParams,  /* in : v params at which errors occur, sized:[mu+1][mv+1] */
   NL_VECTOR *** Tu,       /* in : tangent constraints U, sized:[mu+1][mv+1] */ 
   NL_VECTOR *** Tv,       /* in : tangent constraints V, sized:[mu+1][mv+1] */ 
   NL_INDEX      mu,       /* in : highest 1st u, v, Tu, and Tv index */ 
   NL_INDEX      mv,       /* in : highest 2nd u, v, Tu, and Tv index */ 
   NL_FLAG       ufl,      /* in : oneof NL_NO, NL_START, NL_END, NL_BOTH */ 
   NL_FLAG       vfl,      /* in : oneof NL_NO, NL_START, NL_END, NL_BOTH */ 
   NL_REAL       emx,      /* in : max distance surface will move from any specified uv point initial position */ 
   NL_REAL       eub,      /* in : max distance surface will move from any v=vmin or v=vmax uv point */ 
   NL_REAL       evb,      /* in : max distance surface will move from any u=umin or u=umax uv point */ 
   NL_REAL       eut,      /* in : u tangent tolerance angle, degrees */ 
   NL_REAL       evt )     /* in : v tangent tolerance angle, degrees */ 
{

    NL_FLAG krm, rmf, errorFlag = NL_NO;

    NL_INDEX *uleft, *uright, *vleft, *vright, *sru, *srv, i, j, k, l, row, col, ii, jj, first, last, off, fout, n, m, r, s, ru, su, rv, sv, a;

    NL_DEGREE p, q;

    NL_REAL ** er, ** te, *U, *V, *alf, *oma, *bet, *omb, *mru, *mrv, lam = 0.0, oml = 0.0, al, be, ob, bu, bv, Nbf, N1, tmp, euc, evc;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, *Rw;

    NL_SURFACE surA;

    NL_STACKS SLocal;

    NL_REAL NOREM = 1.0e+25;

    /* Start NURBS */

    N_InitNurbs( &SLocal );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Allocate local memory */

    i = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * i, &SLocal );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * i, &SLocal );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * i, &SLocal );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * i, &SLocal );

    if( omb EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( 2 * i, &SLocal );

    if( Rw EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( mu, mv, &SLocal );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( mu, mv, &SLocal );

    if( te EQ NULL )
        NL_QUIT;

    mru = N_AllocReal1dArray( r, &SLocal );

    if( mru EQ NULL )
        NL_QUIT;

    sru = N_AllocInt1dArray( r, &SLocal );

    if( sru EQ NULL )
        NL_QUIT;

    uleft = N_AllocInt1dArray( n, &SLocal );

    if( uleft EQ NULL )
        NL_QUIT;

    uright = N_AllocInt1dArray( n, &SLocal );

    if( uright EQ NULL )
        NL_QUIT;

    mrv = N_AllocReal1dArray( s, &SLocal );

    if( mrv EQ NULL )
        NL_QUIT;

    srv = N_AllocInt1dArray( s, &SLocal );

    if( srv EQ NULL )
        NL_QUIT;

    vleft = N_AllocInt1dArray( m, &SLocal );

    if( vleft EQ NULL )
        NL_QUIT;

    vright = N_AllocInt1dArray( m, &SLocal );

    if( vright EQ NULL )
        NL_QUIT;

    errorFlag = N_AllocSrfArrays( &surA, n, m, p, q, r, s, &SLocal );

    if(errorFlag EQ NL_YES )
        NL_OUT;

    /* Initialize */

    for ( i = 0; i <= r; i++ )
    {
        mru[i] = NL_BIGD;
        sru[i] = 0;
    }

    for ( j = 0; j <= s; j++ )
    {
        mrv[j] = NL_BIGD;
        srv[j] = 0;
    }

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= mv; j++ )
            er[i][j] = 0.0;
    }

    al = (eut * NL_PI) / 180.0;
    be = (evt * NL_PI) / 180.0;
    euc = cos( al );
    evc = cos( be );

    /* Compute the maximum of knot removal errors for each */
    /* distinct knot and get ranges of parameter indexes   */

    ru = p + 1;

    while( ru LE n )
    {
        i = ru;

        while( ru LE n AND U[ru]EQ U[ru + 1] )
            ru++;
        sru[ru] = ru - i + 1;

        errorFlag = N_SrfRemoveOneKnot( sur, ru, sru[ru], 0, m, NL_UDIR, &mru[ru] );

        if(errorFlag EQ NL_YES )
            NL_OUT;

        ru++;
    }

    k = 1;

    for ( i = 0; i <= n; i++ )
    {
        uleft[i] = k;

        while( k LT mu AND uParams[k]GT U[i]AND uParams[k]LE U[i + 1] )
            k++;

        for ( l = k; l < mu; l++ )
        {
            if( uParams[l]GE U[i + p + 1] )
                break;
        }
        uright[i] = l - 1;
    }

    rv = q + 1;

    while( rv LE m )
    {
        i = rv;

        while( rv LE m AND V[rv]EQ V[rv + 1] )
            rv++;
        srv[rv] = rv - i + 1;

        errorFlag = N_SrfRemoveOneKnot( sur, rv, srv[rv], 0, n, NL_VDIR, &mrv[rv] );

        if(errorFlag EQ NL_YES )
            NL_OUT;

        rv++;
    }

    k = 1;

    for ( j = 0; j <= m; j++ )
    {
        vleft[j] = k;

        while( k LT mv AND vParams[k]GT V[j]AND vParams[k]LE V[j + 1] )
            k++;

        for ( l = k; l < mv; l++ )
        {
            if( vParams[l]GE V[j + q + 1] )
                break;
        }
        vright[j] = l - 1;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

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

        /* If no more removable knot -> finished */

        if( (bu EQ NL_BIGD OR bu EQ NOREM)AND( bv EQ NL_BIGD OR bv EQ NOREM ) )
            break;

        if( bu LT bv )
            krm = NL_UDIR;
        else
            krm = NL_VDIR;

        /* Switch to the appropriate direction */

        switch( krm )
        {
            case NL_UDIR:

                /***************************************/
                /* Check positional and tangent errors */
                /***************************************/

                rmf = NL_YES;

                /* Check positional errors */

                if( (p + su) % 2 )
                {
                    k = (p + su + 1) / 2;
                    i = ru - k;
                    al = (U[ru] - U[i]) / (U[i + p + 1] - U[i]);
                    be = (U[ru] - U[i + 1]) / (U[i + p + 2] - U[i + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    for ( k = uleft[i]; k <= uright[i + 1]; k++ )
                    {
                      errorFlag = N_BasisIEval( knu, i, p, uParams[k], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        errorFlag = N_BasisIEval( knu, i + 1, p, uParams[k], NL_LEFT, &N1 );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = fabs( lam * al * Nbf - oml * ob * N1 ) * bu;

                        for ( l = 0; l <= mv; l++ )
                        {
                            te[k][l] = er[k][l] + tmp;

                            if( te[k][l]GT emx )
                            {
                                rmf = NL_NO;
                                break;
                            }
                        }

                        if( te[k][0]GT eub )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( te[k][mv]GT eub )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( rmf EQ NL_NO )
                            break;
                    }
                }
                else
                {
                    k = (p + su) / 2;
                    i = ru - k;

                    for ( k = uleft[i]; k <= uright[i]; k++ )
                    {
                      errorFlag = N_BasisIEval( knu, i, p, uParams[k], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = Nbf * bu;

                        for ( l = 0; l <= mv; l++ )
                        {
                            te[k][l] = er[k][l] + tmp;

                            if( te[k][l]GT emx )
                            {
                                rmf = NL_NO;
                                break;
                            }
                        }

                        if( te[k][0]GT eub )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( te[k][mv]GT eub )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( rmf EQ NL_NO )
                            break;
                    }
                }

                if( rmf EQ NL_NO )
                {
                    mru[ru] = NOREM;
                    continue;
                }

                /* Check tangent error */

                errorFlag = N_FitSrfTangentError( sur, &surA, ru, su, NL_UDIR, ufl, vfl, Tu, Tv, uParams, vParams, mu, mv, euc, evc, &rmf );

                if(errorFlag EQ NL_YES )
                    NL_OUT;

                if( rmf EQ NL_NO )
                {
                    mru[ru] = NOREM;
                    continue;
                }

                /* Update error matrix and remove knot */

                if( (p + su) % 2 )
                    a = uright[i + 1];
                else
                    a = uright[i];

                for ( k = uleft[i]; k <= a; k++ )
                {
                    for ( l = 0; l <= mv; l++ )
                        er[k][l] = te[k][l];
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

                for ( col = 0; col <= m; col++ )
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

                    /* Save new control points */

                    if( (p + su) % 2 )
                    {
                        N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                    }

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        N_CopyCPt( Rw[i - off], &Pw[i][col] );
                        N_CopyCPt( Rw[j - off], &Pw[j][col] );
                        i++;
                        j--;
                    }
                } /* End for each row */

                /* Successful removal -> shift down some entinties */

                if( su GT 1 )
                    sru[ru - 1] = sru[ru] - 1;

                for ( i = ru + 1; i <= r; i++ )
                {
                    mru[i - 1] = mru[i];
                    sru[i - 1] = sru[i];
                    U[i - 1] = U[i];
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
                N_SrfSetSizeIndices( sur, n, m, p, q, r, s );

                /* If no more internal knots -> finished */

                if( n EQ p AND m EQ q )
                    break;

                /* Update error bounds */

                k = NL_MAX( ru - p, p + 1 );
                l = NL_MIN( n, ru + p - su );

                for ( i = k; i <= l; i++ )
                {
                    if( U[i]NEQ U[i + 1]AND mru[i]NEQ NOREM )
                    {
                      errorFlag = N_SrfRemoveOneKnot( sur, i, sru[i], 0, m, NL_UDIR, &mru[i] );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;
                    }
                }

                for ( j = q + 1; j <= m; j++ )
                {
                    if( V[j]NEQ V[j + 1]AND mrv[j]NEQ NOREM )
                    {
                      errorFlag = N_SrfRemoveOneKnot( sur, j, srv[j], first, last, NL_VDIR, &mrv[j] );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;
                    }
                }

                /* Update index ranges in u-direction */

                for ( i = ru - p - 1; i <= ru - su; i++ )
                {
                    for ( k = uright[i] + 1; k <= mu; k++ )
                    {
                        if( uParams[k]GE U[i + p + 1] )
                            break;
                    }
                    uright[i] = k - 1;
                }

                for ( i = ru - su + 1; i <= n; i++ )
                {
                    uleft[i] = uleft[i + 1];
                    uright[i] = uright[i + 1];
                }
                break;

            case NL_VDIR:

                /***************************************/
                /* Check positional and tangent errors */
                /***************************************/

                rmf = NL_YES;

                /* Check positional errors */

                if( (q + sv) % 2 )
                {
                    k = (q + sv + 1) / 2;
                    j = rv - k;
                    al = (V[rv] - V[j]) / (V[j + q + 1] - V[j]);
                    be = (V[rv] - V[j + 1]) / (V[j + q + 2] - V[j + 1]);
                    ob = 1.0 - be;
                    lam = al / (al + be);
                    oml = 1.0 - lam;

                    for ( l = vleft[j]; l <= vright[j + 1]; l++ )
                    {
                      errorFlag = N_BasisIEval( knv, j, q, vParams[l], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        errorFlag = N_BasisIEval( knv, j + 1, q, vParams[l], NL_LEFT, &N1 );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = fabs( lam * al * Nbf - oml * ob * N1 ) * bv;

                        for ( k = 0; k <= mu; k++ )
                        {
                            te[k][l] = er[k][l] + tmp;

                            if( te[k][l]GT emx )
                            {
                                rmf = NL_NO;
                                break;
                            }
                        }

                        if( te[0][l]GT evb )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( te[mu][l]GT evb )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( rmf EQ NL_NO )
                            break;
                    }
                }
                else
                {
                    k = (q + sv) / 2;
                    j = rv - k;

                    for ( l = vleft[j]; l <= vright[j]; l++ )
                    {
                      errorFlag = N_BasisIEval( knv, j, q, vParams[l], NL_LEFT, &Nbf );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;

                        tmp = Nbf * bv;

                        for ( k = 0; k <= mu; k++ )
                        {
                            te[k][l] = er[k][l] + tmp;

                            if( te[k][l]GT emx )
                            {
                                rmf = NL_NO;
                                break;
                            }
                        }

                        if( te[0][l]GT evb )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( te[mu][l]GT evb )
                        {
                            rmf = NL_NO;
                            break;
                        }

                        if( rmf EQ NL_NO )
                            break;
                    }
                }

                if( rmf EQ NL_NO )
                {
                    mrv[rv] = NOREM;
                    continue;
                }

                /* Check tangent error */

                errorFlag = N_FitSrfTangentError( sur, &surA, rv, sv, NL_VDIR, ufl, vfl, Tu, Tv, uParams, vParams, mu, mv, euc, evc, &rmf );

                if(errorFlag EQ NL_YES )
                    NL_OUT;

                if( rmf EQ NL_NO )
                {
                    mrv[rv] = NOREM;
                    continue;
                }

                /* Update error matrix and remove knot */

                if( (q + sv) % 2 )
                    a = vright[j + 1];
                else
                    a = vright[j];

                for ( l = vleft[j]; l <= a; l++ )
                {
                    for ( k = 0; k <= mu; k++ )
                        er[k][l] = te[k][l];
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

                /* Remove the knot for each column */

                for ( row = 0; row <= n; row++ )
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

                    /* Save new control points */

                    if( (q + sv) % 2 )
                    {
                        N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
                    }

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        N_CopyCPt( Rw[i - off], &Pw[row][i] );
                        N_CopyCPt( Rw[j - off], &Pw[row][j] );
                        i++;
                        j--;
                    }
                } /* End for each row */

                /* Successful removal -> shift down some entinties */

                if( sv GT 1 )
                    srv[rv - 1] = srv[rv] - 1;

                for ( j = rv + 1; j <= s; j++ )
                {
                    mrv[j - 1] = mrv[j];
                    srv[j - 1] = srv[j];
                    V[j - 1] = V[j];
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
                N_SrfSetSizeIndices( sur, n, m, p, q, r, s );

                /* If no more internal knots -> finished */

                if( n EQ p AND m EQ q )
                    break;

                /* Update error bounds */

                k = NL_MAX( rv - q, q + 1 );
                l = NL_MIN( m, rv + q - sv );

                for ( j = k; j <= l; j++ )
                {
                    if( V[j]NEQ V[j + 1]AND mrv[j]NEQ NOREM )
                    {
                      errorFlag = N_SrfRemoveOneKnot( sur, j, srv[j], 0, n, NL_VDIR, &mrv[j] );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;
                    }
                }

                for ( i = p + 1; i <= n; i++ )
                {
                    if( U[i]NEQ U[i + 1]AND mru[i]NEQ NOREM )
                    {
                      errorFlag = N_SrfRemoveOneKnot( sur, i, sru[i], first, last, NL_UDIR, &mru[i] );

                        if(errorFlag EQ NL_YES )
                            NL_OUT;
                    }
                }

                /* Update index ranges in u-direction */

                for ( j = rv - q - 1; j <= rv - sv; j++ )
                {
                    for ( k = vright[j] + 1; k <= mv; k++ )
                    {
                        if( vParams[k]GE V[j + q + 1] )
                            break;
                    }
                    vright[j] = k - 1;
                }

                for ( j = rv - sv + 1; j <= m; j++ )
                {
                    vleft[j] = vleft[j + 1];
                    vright[j] = vright[j + 1];
                }
                break;
        } /* End of switch */
    }     /* End of while */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SLocal );

    return (errorFlag);
}

/**********************************************************************/
/* N_FITSRFTANGENTERROR: Tangent error for one knot removal for fitting           */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting routine computes  the  cross-boundary derivative error
     after one knot is removed either in u- or in v-direction. A typical 
     calling example:
 
       NL_FLAG     rmf;
       NL_SURFACE  surP, surQ;
       NL_INDEX    rr, ss, mu, mv;
       NL_REAL     *u, *v, eut, evt;
       NL_VECTOR   ***Tu, ***Tv;
       ...
       (define surP, and get rest of input data);
       ...
       N_FitSrfTangentError(&surP,&surQ,rr,ss,NL_UDIR,NL_START,NL_END,Tu,Tv,u,v,mu,mv,
                eut,evt,&rmf);
 
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
                         NL_NO   : no tangents specified
                         NL_START: remove for u=umin cross-tangent
                         NL_END  : remove for u=umax cross-tangent
                         NL_BOTH : remove for both cross-tangent
     vfl   , input  ,  Flag:
                         NL_NO   : no tangents specified
                         NL_START: remove for v=vmin cross-tangent
                         NL_END  : remove for v=vmax cross-tangent
                         NL_BOTH : remove for both cross-tangent
     Tu,Tv , input  ,  Tangents at discrete points (see N_FITSRFINTPTAN)
     u,v   , input  ,  Parameters data points are assumed at
     mu,mv , input  ,  Highest indexes in u, v, Tu and Tv
     euc   , input  ,  U-tangent error tolerance
     evc   , input  ,  V-tangent error tolerance
     rmf   , output ,  Flag:
                         NL_YES: knot is removable
                         NL_NO : knot is NOT removable
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfTangentError 
  (NL_SURFACE   *surP,    /* in : target surface */ 
   NL_SURFACE   *surQ,    /* in : working surface */ 
   NL_INDEX      rr,      /* in : index of knot to be removed */ 
   NL_INDEX      ss,      /* in : multiplicity of knot to be removed */ 
   NL_FLAG       dir,     /* in : NL_UDIR = remove knot from U KnotVector, NL_VDIR = remove from V KnotVector */ 
   NL_FLAG       ufl,     /* in : constrain umin/umax boundary cross-tangents, NL_NO, NL_START, NL_END, or NL_BOTH */ 
   NL_FLAG       vfl,     /* in : constrain vmin/vmax boundary cross-tangents, NL_NO, NL_START, NL_END, or NL_BOTH  */ 
   NL_VECTOR *** Tu,      /* in : u Tangents at specified points, sized:[mu+1][mv+1], Tu[i][j] = NULL to ignore */ 
   NL_VECTOR *** Tv,      /* in : v Tangents at specified points, sized:[mu+1][mv+1], Tv[i][j] = NULL to ignore */ 
   NL_REAL      *uParams, /* in : u parameters at specified points, sized:[mu+1][mv+1] */ 
   NL_REAL      *vParams, /* in : v parameters at specified points, sized:[mu+1][mv+1] */
   NL_INDEX      mu,      /* in : highest 1st Tu, Tv, uParams, and vParams index */ 
   NL_INDEX      mv,      /* in : highest 2nd Tu, Tv, uParams, and vParams index */ 
   NL_REAL       euc,     /* in : u tangent angle tolerance, degrees */ 
   NL_REAL       evc,     /* in : v tangent angle tolerance, degrees */ 
   NL_FLAG      *rmf )    /* out: NL_YES = knot is removable     */ 
                          /*      NL_NO  = knot is not removable */
{

    NL_FLAG errorFlag = NL_NO;

    NL_INDEX n, m, r, s, i, j;

    NL_DEGREE p, q;

    NL_REAL vcs;

    NL_POINT V;

    NL_CURVE cush, cueh, cvsh, cveh;

    NL_STACKS SLocal;

    /* Start NURBS */

    N_InitNurbs( &SLocal );

    /* Initialize and get surQ */

    *rmf = NL_YES;

    errorFlag = N_SrfRemoveKnot( surP, rr, ss, dir, ufl, vfl, surQ );

    if(errorFlag EQ NL_YES )
        NL_OUT;

    N_SrfGetArraySizes( surP, &n, &m, &r, &s );
    N_SrfGetDegrees( surP, &p, &q );

    /* Remove u-knot */

    if( dir EQ NL_UDIR )
    {
        if( vfl EQ NL_START OR vfl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cvsh );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_BOTTOM, &cvsh, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mu; i++ )
            {
                if( Tv[i][0]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cvsh, uParams[i], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tv[i][0], V, &vcs );

                    if( vcs LT evc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }

        if( vfl EQ NL_END OR vfl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cveh );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_TOP, &cveh, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mu; i++ )
            {
                if( Tv[i][1]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cveh, uParams[i], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tv[i][1], V, &vcs );

                    if( vcs LT evc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }

        if( rr EQ p + 1 AND( ufl EQ NL_START OR ufl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cush );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_LEFT, &cush, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= mv; j++ )
            {
                if( Tu[0][j]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cush, vParams[j], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tu[0][j], V, &vcs );

                    if( vcs LT euc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }

        if( rr EQ n AND( ufl EQ NL_END OR ufl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cueh );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_RIGHT, &cueh, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= mv; j++ )
            {
                if( Tu[1][j]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cueh, vParams[j], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tu[1][j], V, &vcs );

                    if( vcs LT euc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }
    }

    /* Remove v-knot */

    if( dir EQ NL_VDIR )
    {
        if( ufl EQ NL_START OR ufl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cush );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_LEFT, &cush, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= mv; j++ )
            {
                if( Tu[0][j]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cush, vParams[j], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tu[0][j], V, &vcs );

                    if( vcs LT euc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }

        if( ufl EQ NL_END OR ufl EQ NL_BOTH )
        {
            N_CrvInitArrays( &cueh );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_RIGHT, &cueh, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= mv; j++ )
            {
                if( Tu[1][j]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cueh, vParams[j], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tu[1][j], V, &vcs );

                    if( vcs LT euc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }

        if( rr EQ q + 1 AND( vfl EQ NL_START OR vfl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cvsh );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_BOTTOM, &cvsh, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mu; i++ )
            {
                if( Tv[i][0]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cvsh, uParams[i], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tv[i][0], V, &vcs );

                    if( vcs LT evc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }

        if( rr EQ m AND( vfl EQ NL_END OR vfl EQ NL_BOTH ) )
        {
            N_CrvInitArrays( &cveh );
            errorFlag = N_CrossBoundDerivCrvNurbsSrf( surQ, NL_TOP, &cveh, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= mu; i++ )
            {
                if( Tv[i][1]NEQ NULL )
                {
                  errorFlag = N_CrvEval( &cveh, uParams[i], NL_LEFT, &V );

                    if(errorFlag EQ NL_YES )
                        NL_OUT;

                    N_VectorsCosAngle( *Tv[i][1], V, &vcs );

                    if( vcs LT evc )
                    {
                        *rmf = NL_NO;
                        NL_OUT;
                    }
                }
            }
        }
    }

    EXIT:

    N_EndNurbs( &SLocal );

    return (errorFlag);
}

#if NLIB_UNUSED

/**********************************************************************************/
/* N_FitSrfInterpVariablePts: Surface interpolation to variable number of points  */
/**********************************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface interpolating a given
     set of  rectangularly  arranged points. In  each row the  number of 
     points may be  different. Optionally any  number of boundary curves
     may be specified. The boundary curves must be non-rational interpo-
     lating the  points along the  perimeter of the point set. The point
     set is arranged as follows:

       P[0][0],...,P[0][m[0]]
       P[1][0],...,P[1][m[1]]
       ...
       P[n][0],...,P[n][m[n]]

     See N_AllocVariable2dPtArray  for allocating  memory for such  an array. A  typical 
     calling example is:

       NL_POINT    **P;
       NL_INDEX    n, *m;
       NL_DEGREE   p, q;
       NL_REAL     per;
       NL_CURVE    **bndU, **bndV
       NL_SURFACE  sur;
       NL_STACKS   SC, SS;
       ...
       (define array P, choose p, q and per, get bndU and bndV);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfInterpVariablePts(P,n,m,p,q,per,bndU,bndV,&sur,&SC,&SS);
       N_FitSrfInterpVariablePts(P,n,m,p,q,per,NULL,NULL,&sur,&SC,&SS);


   ACCESS:
   
     P    , input  ,  Points to be interpolated 
     n    , input  ,  Highest index in u-direction
     m    , input  ,  Highest indexes in v-direction
     p,q  , input  ,  Degrees of interpolating surface
     per  , input  ,  Percentage of  flexibility in  choosing the knots
                      for interpolation; 0.0 <= per <= 1.0:
                        1.0: maximum flexibility
                               PRO: good data reduction
                               CON: lesser quality of interpolant
                        0.0: no flexibility
                               PRO: better quality of interpolant
                               CON: larger number of control points
                      RECOMMENDATION:
                        Dense  data set --> per = {0.9 - 1.0}
                        pSarse data set --> per = {0.6 - 0.9}
                      THE BEST WAY TO ADJUST per IS  INTERACTIVELY, I.E.
                      NL_START  WITH 1.0, IF  INTERPOLANT WIGGLES TOO MUCH, 
                      DECREASE per TILL RESULT IS ACCEPTABLE.
     bndU , input  ,  Boundary curves in the u-direction:
                        bndU[0]: v=vmin boundary
                        bndU[1]: v=vmax boundary
                      If bndU=NULL, no boundary  passed in. Both bndU[0]
                      and bndU[1] can independently  be NULL, indicating
                      no constraints along v=vmin and/or v=vmax.
     bndV , input  ,  Boundary curves in the v-direction:
                        bndV[0]: u=umin boundary
                        bndV[1]: u=umax boundary
                      If bndV=NULL, no boundary  passed in. Both bndV[0]
                      and bndV[1] can independently  be NULL, indicating
                      no constraints along u=umin and/or u=umax.
     sur  , output ,  Interpolating surface
     SC   , input  ,  bndU's and bndV's memory stacks
     SS   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitSrfInterpVariablePts
  (NL_POINT ** P,     /* in : rows-of-points, sized:[n+1][m[i]+1] */
   NL_INDEX    n,     /* in : highest 1st index in P                 */
   NL_INDEX   *m,     /* in : highest 2nd indices in p, sized:[n+1]  */
   NL_DEGREE   p,     /* in : output surface degree U */
   NL_DEGREE   q,     /* in : output surface degree V */ 
   NL_REAL     per,   /* in : flexibility in knot selection, 0 <= per <= 1 */
                      /*      0.0 = min flexibility, better interpolation, higher control point count */
                      /*      1.0 = max flexibility, loose interpolaation, lower control point count */
   NL_CURVE ** bndU,  /* in : u boundary curves, bndU[0]: v=vmin boundary, bndU[1]: v=vmax boundary  */
   NL_CURVE ** bndV,  /* in : v boundary curves, bndV[0]: u=umin boundary, bndV[1]: u=umax boundary  */                   
   NL_SURFACE *sur,   /* out: approximating surface */
   NL_STACKS  *SC,    /* in : bndU's and bndV's memory stacks */
   NL_STACKS  *SS )   /* in : sur's memory stack              */
{

    NL_PRIVATE NL_STRING rname = _T("N_FitSrfInterpVariablePts");

    NL_FLAG errorFlag = NL_NO;

    NL_INDEX i, j, k, is, ie, js, je, mh, ns, ms, rs, ss, mr = 0, mxu, mxr;

    NL_DEGREE ps, qs, pc, qc;

    NL_REAL *US, *VS, *UT, *VT, *UR = NULL, *XU = NULL, *XR = NULL, *ua, *uf, fac;

    NL_POINT ** Rnv, *Qnv;

    NL_CPOINT ** Sw, *Cw, *Qw, *Rw;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL, *knr = NULL, *kna = NULL, knx;

    NL_CURVE ** curV, curU;

    NL_CPOLYGON *pol;

    NL_RMATRIX cm;

    NL_STACKS SLocal;

    /* Start NURBS */

    N_InitNurbs( &SLocal );

    /* Check incoming data */

    ps = p;
    qs = q;

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
        {
            if( N_IsCrvRat( bndU[0] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndU[0], &pc );
            ps = NL_MAX( ps, pc );
        }

        if( bndU[1]NEQ NULL )
        {
            if( N_IsCrvRat( bndU[1] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndU[1], &pc );
            ps = NL_MAX( ps, pc );
        }
    }

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
        {
            N_CrvGetDegree( bndU[0], &pc );

            if( pc LT ps )
            {
              errorFlag = N_CrvElevateDegree( bndU[0], ps - pc, bndU[0], SC, SC );

                if(errorFlag EQ NL_YES )
                    NL_OUT;
            }
        }

        if( bndU[1]NEQ NULL )
        {
            N_CrvGetDegree( bndU[1], &pc );

            if( pc LT ps )
            {
              errorFlag = N_CrvElevateDegree( bndU[1], ps - pc, bndU[1], SC, SC );

                if(errorFlag EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL )
        {
            if( N_IsCrvRat( bndV[0] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndV[0], &qc );
            qs = NL_MAX( qs, qc );
        }

        if( bndV[1]NEQ NULL )
        {
            if( N_IsCrvRat( bndV[1] ) )
                NL_ERROR( NL_INP_ERR );

            N_CrvGetDegree( bndV[1], &qc );
            qs = NL_MAX( qs, qc );
        }
    }

    /* Check compatibility */

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL AND bndU[1]NEQ NULL )
        {
            if( NOT N_CrvsAreCombatible( bndU, 1 ) )
            {
              errorFlag = N_CrvsMakeCompatible( bndU, 1, SC );

                if(errorFlag EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
            N_CrvGetKnotVector( bndU[0], &knr );

        else if( bndU[1]NEQ NULL )
            N_CrvGetKnotVector( bndU[1], &knr );

        N_KnotVectorGetKnots( knr, &mr, &UR );
    }

    /* Interpolate columns of data points */

    curV = N_AllocArrayCrvPtrs( n, &SLocal );

    if( curV EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        curV[i] = N_AllocCrv( &SLocal );

        if( curV[i]EQ NULL )
            NL_QUIT;
    }

    is = 0;
    ie = n;

    if( bndV NEQ NULL )
    {
        if( bndV[0]NEQ NULL )
        {
            N_CrvInitArrays( curV[0] );
            errorFlag = N_CrvCopy( bndV[0], curV[0], &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            is = 1;
        }

        if( bndV[1]NEQ NULL )
        {
            N_CrvInitArrays( curV[n] );
            errorFlag = N_CrvCopy( bndV[1], curV[n], &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            ie = n - 1;
        }
    }

    mh = 0;

    for ( i = 0; i <= n; i++ )
    {
        if( m[i]GT mh )
            mh = m[i];
    }

    ua = N_AllocReal1dArray( NL_MAX( mh, n ), &SLocal );

    if( ua EQ NULL )
        NL_QUIT;

    uf = N_AllocReal1dArray( mh, &SLocal );

    if( uf EQ NULL )
        NL_QUIT;

    for ( k = 0; k <= mh; k++ )
        ua[k] = 0.0;

    j = 0;

    for ( i = 0; i <= n; i++ )
    {
        if( m[i]EQ mh )
        {
          errorFlag = N_FitCalcCrvParamValues( (NL_VOID *)P[i], mh, NL_EPOINT, NL_CHORDLENGTH, uf );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( k = 0; k <= mh; k++ )
                ua[k] += uf[k];

            j++;
        }
    }

    if( j GT 1 )
    {
        fac = 1.0 / j;

        for ( k = 0; k <= mh; k++ )
            ua[k] *= fac;
    }

    knv = N_AllocKnotVectorAndArray( mh + qs + 1, &SLocal );

    if( knv EQ NULL )
        NL_QUIT;

    N_FitCrvCalcKnotVector( ua, mh, qs, knv );

    for ( i = is; i <= ie; i++ )
    {
        N_CrvInitArrays( curV[i] );
        errorFlag = N_FitCrvKnotsAndTangents( P[i], m[i], NULL, NULL, NULL, NL_TANGENT, &knv, per, qs, curV[i], &SLocal, &SLocal );

        if(errorFlag EQ NL_YES )
            NL_OUT;
    }

    /* Make interpolatory curves compatible */

    if( NOT N_CrvsAreCombatible( curV, n ) )
    {
      errorFlag = N_CrvsMakeCompatible( curV, n, &SLocal );

        if(errorFlag EQ NL_YES )
            NL_OUT;
    }

    /* Now interpolate in the u-direction */

    N_CrvGetKnotVector( curV[0], &knv );
    N_CrvGetArraySizes( curV[0], &ms, &ss );

    ns = n;
    rs = ns + ps + 1;

    /* Get knot vector */

    Rnv = N_AllocPt2dArray( ns, ms, &SLocal );

    if( Rnv EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= ns; i++ )
    {
        N_CrvGetCPts( curV[i], &ms, &Cw );

        for ( j = 0; j <= ms; j++ )
            N_CPtToPtEuclid( Cw[j], &Rnv[i][j] );
    }

    errorFlag = N_FitSrfInterpParams( (NL_VOID ** )Rnv, ns, ms, NL_EPOINT, NL_CHORDLENGTH, NL_UDIR, ua );

    if(errorFlag EQ NL_YES )
        NL_OUT;

    if( bndU NEQ NULL )
    {
        if( ua[0]NEQ UR[0]OR ua[ns]NEQ UR[mr] )
        {
            fac = (UR[mr] - UR[0]) / (ua[ns] - ua[0]);

            for ( i = 1; i < ns; i++ )
            {
                ua[i] = UR[0] + fac * (ua[i] - ua[0]);
            }
            ua[0] = UR[0];
            ua[ns] = UR[mr];
        }
    }

    knu = N_AllocKnotVectorAndArray( rs, &SLocal );

    if( knu EQ NULL )
        NL_QUIT;

    N_FitCrvCalcKnotVector( ua, ns, ps, knu );

    pol = N_AllocCPolygonAndArray( ns, &SLocal );

    if( pol EQ NULL )
        NL_QUIT;

    N_CrvFromCPolygonAndKnotVector( &curU, pol, ps, knu );

    /* See if refinement is needed */

    mxr = mxu = -1;

    if( bndU NEQ NULL )
    {
        XR = N_AllocReal1dArray( rs, &SLocal );

        if( XR EQ NULL )
            NL_QUIT;

        XU = N_AllocReal1dArray( mr, &SLocal );

        if( XU EQ NULL )
            NL_QUIT;

        errorFlag = N_MergeKnotVectors( knu, knr, ps, XU, &mxu, XR, &mxr );

        if(errorFlag EQ NL_YES )
            NL_OUT;
    }

    /* Update surface indexes and check/allocate memory */

    if( mxu GE 0 )
    {
        ns += mxu + 1;
        rs += mxu + 1;
    }

    errorFlag = N_SrfSizeArrays( sur, ns, ms, ps, qs, rs, ss, rname, SS );

    if(errorFlag EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );

    Qw = N_AllocCPt1dArray( ns, &SLocal );

    if( Qw EQ NULL )
        NL_QUIT;

    Qnv = N_AllocPt1dArray( n, &SLocal );

    if( Qnv EQ NULL )
        NL_QUIT;

    /* Refine boundaries and get surface boundaries */

    if( bndU NEQ NULL )
    {
        if( mxr GE 0 )
        {
            if( bndU[0]NEQ NULL )
            {
              errorFlag = N_CrvRefineToKnotVector( bndU[0], XR, mxr, Qw );

                if(errorFlag EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= ns; i++ )
                    N_CopyCPt( Qw[i], &Sw[i][0] );
            }

            if( bndU[1]NEQ NULL )
            {
              errorFlag = N_CrvRefineToKnotVector( bndU[1], XR, mxr, Qw );

                if(errorFlag EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= ns; i++ )
                    N_CopyCPt( Qw[i], &Sw[i][ms] );
            }
        }
        else
        {
            if( bndU[0]NEQ NULL )
            {
                N_CrvGetCPts( bndU[0], &k, &Rw );

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Rw[i], &Sw[i][0] );
            }

            if( bndU[1]NEQ NULL )
            {
                N_CrvGetCPts( bndU[1], &k, &Rw );

                for ( i = 0; i <= k; i++ )
                    N_CopyCPt( Rw[i], &Sw[i][ms] );
            }
        }
    }

    /* Now interpolate cross-sectional curves */

    errorFlag = N_FitCalcMatrix( knu, ps, ua, n, NL_NO, &cm, &SLocal );

    if(errorFlag EQ NL_YES )
        NL_OUT;

    js = 0;
    je = ms;

    if( bndU NEQ NULL )
    {
        if( bndU[0]NEQ NULL )
            js = 1;

        if( bndU[1]NEQ NULL )
            je = ms - 1;
    }

    N_CrvGetCPts( &curU, &k, &Rw );

    for ( j = js; j <= je; j++ )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CrvGetCPts( curV[i], &ms, &Cw );
            N_CPtToPtEuclid( Cw[j], &Qnv[i] );
        }

        errorFlag = N_FitCrvMatrix( (NL_VOID *)Qnv, n, NL_EPOINT, ps, &cm, Rw );

        if(errorFlag EQ NL_YES )
            NL_OUT;

        if( mxu GE 0 )
        {
          errorFlag = N_CrvRefineToKnotVector( &curU, XU, mxu, Qw );

            if(errorFlag EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= ns; i++ )
                N_CopyCPt( Qw[i], &Sw[i][j] );
        }
        else
        {
            for ( i = 0; i <= ns; i++ )
                N_CopyCPt( Rw[i], &Sw[i][j] );
        }
    }

    /* Get surface knot vectors */

    if( mxu GE 0 )
    {
        N_KnotVectorFromRealArray( &knx, XU, mxu );

        kna = N_AllocKnotVectorAndArray( rs, &SLocal );

        if( kna EQ NULL )
            NL_QUIT;

        errorFlag = N_BasisInsertKnots( knu, ps, &knx, kna );

        if(errorFlag EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kna, &rs, &UT );
    }
    else
    {
        N_KnotVectorGetKnots( knu, &rs, &UT );
    }

    N_KnotVectorGetKnots( knv, &ss, &VT );

    for ( i = 0; i <= rs; i++ )
        US[i] = UT[i];

    for ( j = 0; j <= ss; j++ )
        VS[j] = VT[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SLocal );

    return (errorFlag);
}

#endif  // NLIB_UNUSED

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

/*  binary search to find parm value in sorted array */
NL_INDEX ST_BinarySearch /* rtn: pars array index of interval boundary (side = NL_LEFT ? min : max) containing tgt param */
 ( NL_REAL *pars,        /* in : parameter array to search */
   NL_INDEX n1,          /* in : min parameter array index for search */
   NL_INDEX n2,          /* in : max parameter array index for search */
   NL_REAL  pm,          /* in : tgt parameter value  */
   NL_FLAG  side )       /* in : NL_LEFT  = return index immediately less than tgt param value  */
                         /* in : NL_RIGHT = return index immediately greater than tgt param value  */
{

    NL_INDEX low, mid, high;

    low = n1;
    high = n2;
    mid = (n2 + n1) / 2;

    switch( side )
    {
        case NL_LEFT: /* pm <= pars[mid] ,..., pars[n2] */
            if( pm LE pars[n1] )
                return (n1);

            if( pm GT pars[n2] )
                return (n2 + 1);

            while( pm LE pars[mid - 1]OR pm GT pars[mid] )
            {
                if( pm LE pars[mid - 1] )
                    high = mid;

                else if( low NEQ mid )
                    low = mid;

                else
                    low = high;

                mid = (low + high) / 2;
            }

            break;

        case NL_RIGHT: /* pm >= pars[n1] ,..., pars[mid] */
            if( pm GE pars[n2] )
                return (n2);

            if( pm LT pars[n1] )
                return (n1 - 1);

            while( pm LT pars[mid]OR pm GE pars[mid + 1] )
            {
                if( pm LT pars[mid] )
                    high = mid;

                else if( low NEQ mid )
                    low = mid;

                else
                    low = high;

                mid = (low + high) / 2;
            }

            break;
    }

    return (mid);
}

/***********************************************************************/
/*  return TRUE when any of the spans are zero or negative length      */
/***********************************************************************/
NL_BOOLEAN ST_HasNullSpans
  ( NL_REAL  *kts,     /* out: knot values, sized:[nd+1]                  */
    NL_INDEX  nd,      /* in : max index of unique knot values to compute */
    NL_REAL   tol)     /* in : Maximum size for a zero length span        */
{
  NL_INDEX i0, i1 ;
  NL_REAL  MinSpan = 1.0e26 ; 

  /* for every span */
  for(i0=0, i1=1; i1<=nd ; i0++, i1++)
    {
      /* for debug - save min span size */
      if(MinSpan GT (kts[i1] - kts[i0]))
        { MinSpan = (kts[i1] - kts[i0]) ; }

      if(kts[i1] LE kts[i0] + tol)  /* gwc: this should have a tolerance of some kind - done */
        { return(NL_TRUE) ; }
    }
  
  /* arrive here when all spans are nonNULL */
  return(NL_FALSE) ; 

} /* end ST_HasNullSpans          */



/***********************************************************************/
/*  Create a knot sequence that reflects the sample                    */
/*  value density in an input sorted param array                       */ 
/* Methods: An IndexInterval is defined by dividing the input array    */
/*    into evenly spaced number of sequences - they are polytiles.     */
/*  meth = 1: knots = pars' IndexInterval boundary parameter values    */
/*  meth = 2: knots = average(every pars' IndexInterval values         */
/*              to either side of every IndexInterval boundary)        */
/*  meth = 3: knots = avg(pars' values in paramIntervals on either     */
/*                           side of the paramInterval Boundaries)     */
/*  meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,            */
/*                               IndexInterval BoundaryMax,            */
/*                               all contained points                  */
/*  default : knots = evenly spaced                                    */
/*                                                                     */
/* when a method generates a zero length span it's output is replaced  */
/*  with the default output                                            */
/***********************************************************************/
NL_VOID ST_AddKnotsStrip
  ( NL_REAL *pars,     /* in : sorted list of parameter values                                 */
    NL_INDEX n1,       /* in : min index in pars to examine                                    */      
    NL_INDEX n2,       /* in : max index in pars to examine                                    */      
    NL_REAL  pm1,      /* in : min index value of param interval (the strip) being examined    */
                       /* note: on output, kts[0] != pm1, instead kts[0] > pm1                 */      
    NL_REAL  pm2,      /* in : max index value of param interval (the strip) being examined    */
                       /* note: on output, kts[nd] = pm2                                       */     
    NL_INDEX nd,       /* in : max index of UniqueKnot vals                                    */     
    NL_INDEX meth,     /* in : in : meth = 1: knots = even number of par vals for each knot span    */
                       /*           meth = 2: knots = average(every pars' IndexInterval values      */
                       /*                       to either side of every IndexInterval boundary)     */ 
                       /*           meth = 3: knots = avg(pars' values in paramIntervals on either  */
                       /*                                    side of the paramInterval Boundaries)  */                                                  
                       /*           meth = 4: kts = Smoothed(avg(IndexInterval BoundaryMin,         */
                       /*                                        IndexInterval BoundaryMax,         */
                       /*                                        all contained points               */ 
                       /*           DefMeth : knots = evenly spaced                                 */
                       /*      out: -1 = requested knot seq had zero length spans - rtns even spaced knots */                                  
    NL_REAL  *kts,     /* out: knot values, sized:[nd+1], kts[0] = pm1, kts[nd] = pm2          */
    NL_INDEX *nks,     /* out: number of pars values in each returned knot interval            */
    NL_REAL  *ratio,   /* in : MaxIntervalLength/MinIntervalLength                             */
    NL_INDEX *min,     /* out: min number of pars values between any pair of output knots      */
    NL_INDEX *max,     /* out: max number of pars values between any pair of output knots      */
    NL_INDEX *i1,      /* out: pars index of the first value found bounding pm1                */
    NL_INDEX *i2 )     /* out: pars index of the first value found bounding pm2                */

{

    NL_INDEX ii1, ii2, ii, jj, kk, ll, k1, k2, cnt;

    NL_REAL d, alf, dp, d1, d2, MinIvl, MaxIvl, tol;

    NL_REAL SmoothCnt = 2 ; /* number of times smoothing is applied to candidate knot vector when smoothing is requested */

    /* rtn: pars array index of interval boundary (side = NL_LEFT ? min : max) containing tgt param */
    ii1 = ST_BinarySearch( pars, n1, n2, pm1, NL_LEFT );
    ii2 = ST_BinarySearch( pars, n1, n2, pm2, NL_RIGHT );

    if( ii1 GT n2 )
        ii1 = n2;

    if( ii2 LT n1 )
        ii2 = n1;

    if( ii2 LE ii1 ) /* gwc changed: to be LE instead of LT */
        meth = -1;   /* no parms in strip - use default method: equal spacing */

    *i1 = ii1;  /* set output: min pars index being used */
    *i2 = ii2;  /* set output: max pars index being used */

    MinIvl =  1.0E26 ; 
    MaxIvl = -1.0E26 ; 
    tol    = (pm2 - pm1) * 1.0E-3 ; 

    /* for 2 times so that failed methods (generated zero length spans) can be replaced with the default output (even spaced knots) */
    for(cnt=0;cnt<2;cnt++)
    {
       /* skip the 2nd pass unless met has been set to -2 */
       if( cnt EQ 1 )
         {
           if( meth NEQ -2 ) break ;     /* quit if requested method generated knot vector with no zero length spans */
           else              meth = -1 ; /* get default output */
         }

        /* switch on meth to compute knot vectors in various manners */
        switch( meth )
        {
            case 1: /* Knot values set to place even numbers of points in each knot interval */
            case 3: /* smotthed(Knot values set to place even numbers of points in each knot interval) */

                d = ((NL_REAL)ii2 - (NL_REAL)ii1 + 1.0) / ((NL_REAL)nd + 1.0);  /* number of index values per UniqueKnot */
                kk = nd / 2;                         /* median UniqueKnot index */

                /* set lowest output knot value = input strip min value */
                kts[0] = pm1;

                for ( jj = 1; jj < nd; jj++ )   /* for every internal UniqueKnot values */
                {
                    if( jj LE kk )              /* for the first half of UniqueKnots - round up */
                      { ll = (NL_INTEGER)( jj * d );     /* the index min par index inc from min index value for the next set of evenly grouped par values */ 
                        alf = jj * d - ll;               /* the remainder par index inc from min index value part for the next set of evenly frouped par values */
                      }
                    else                        /* for the second half of UniqueKnots - round down */
                      { ll = (NL_INTEGER)( jj * d ) - 1; /* the index min par index inc from min index value for the next set of evenly grouped par values */ 
                        alf = jj * d - 1 - ll;           /* the remainder par index inc from min index value part for the next set of evenly frouped par values */
                      }

                    /* set all UniqueKnot values = linear interp of two param values bracketing even stride of param values */ 
                    kts[jj] = (1.0 - alf) * pars[ll + ii1] + alf * pars[ll + ii1 + 1];

                }
             
                /* set highest output knot value = input strip max value */
                kts[nd] = pm2;

                /* for cases 3 and 4: Smooth Knot distribution: kts_i[jj] = kts_i-1[jj-1] + kts_i1-[jj++] for SmoothCnt iters */
                if(meth GT 2)
                  {
                    for(ii=0;ii<SmoothCnt;ii++)
                      {
                        d1 = kts[0] ;                   /* d1 = pre smoothed kts_0[jj-1] value */
                        for ( jj = 1; jj < nd; jj++ )   /* for every internal UniqueKnot values */
                          {
                            d2 = kts[jj] ;        
                            kts[jj] = (d1 + kts[jj+1]) / 2.0 ;
                            d1 = d2 ; 

                          } /* end iter every internal knot for smoothing */
                      }
                  }

                /* reject solutions with NULL length spans */
                if(ST_HasNullSpans(kts, nd, tol))
                  { meth = -2 ; 
                    break ;
                  }

                break;

            case 2: /* InternalKnot[i] = average(Block[i] Par values), Block[i] = ith set of ConstCount of points */
            case 4: /* smoothed(InternalKnot[i] = average(Block[i] Par values) */

                d = ((NL_REAL)ii2 - (NL_REAL)ii1 + 1.0) / ((NL_REAL)nd - 1.0)  ; /* number of index values per internal knot */
                kk = nd / 2;                        /* median UniqueKnot index */
                k1 = ii1;                           /* current target pars index value */

                /* set lowest output knot value = input strip min value */
                kts[0] = pm1;

                for ( jj = 1; jj < nd; jj++ )       /* for every internal UniqueKnot */
                {
                    if( jj LE kk )              /* for the first half of UniqueKnots - round up */
                      { ll = (NL_INTEGER)( jj * d );     /* the index min par index inc from min index value for the next set of evenly grouped par values */ 
                      }
                    else                        /* for the second half of UniqueKnots - round down */
                      { ll = (NL_INTEGER)( jj * d ) - 1; /* the index min par index inc from min index value for the next set of evenly grouped par values */ 
                      }

                    alf = pars[k1];                 /* current target par value */
                    if(jj LT nd-1 )
                      k2 = (ll + ii1) - k1;           /* number of pars in next interval */
                    else
                      k2 = ii2 - k1;

                    for ( ii = 1; ii <= k2; ii++ )
                        alf += pars[k1 + ii];
                    kts[jj] = alf / ((NL_REAL)k2 + 1.0);       /* kts[jj] = Avg Par value of points in this interval */
                                                    
                    k1 = ll + ii1 + 1;
                }

                /* set highest output knot value = input strip max value */
                kts[nd] = pm2;

                /* for cases 3 and 4: Smooth Knot distribution: kts_i[jj] = kts_i-1[jj-1] + kts_i1-[jj++] for SmoothCnt iters */
                if(meth GT 2)
                  {
                    for(ii=0;ii<SmoothCnt;ii++)
                      {
                        d1 = kts[0] ;                   /* d1 = pre smoothed kts_0[jj-1] value */
                        for ( jj = 1; jj < nd; jj++ )   /* for every internal UniqueKnot values */
                          {
                            d2 = kts[jj] ;        
                            kts[jj] = (d1 + kts[jj+1]) / 2.0 ;
                            d1 = d2 ; 

                          } /* end iter every internal knot for smoothing */
                      }
                  }

                /* reject solutions with NULL length spans */
                if(ST_HasNullSpans(kts, nd, tol))
                  { meth = -2 ; 
                    break ;
                  }

                break;

            case -1: /* default method, requested method had zero length spans, or no parms in strip; just divide equally */

                dp = (pm2 - pm1) / (nd);
                d  = pm1 ; 

                /* set knots - 1st knot = pm1 */
                for ( ii = 0; ii < nd; ii++, d+=dp )
                {                             
                    kts[ii] = d ;  
                }

                /* set highest output knot value = to input strip max value */
                kts[nd] = pm2;

                break ; 

        } /* end switch on meth */
    } /* end iter twice to replace solutions with NULL spans to default output */

    /* arrive here when requested knot sequence method had no zero length spans */
    kk = ii2 - ii1 + 2;  /* min number of points in one span */
    ll = 0;              /* max number of points in one span */
    k1 = ii1;            /* current lower bound index to search - k2 current found upper bound index for param range */
    nks[nd] = 0 ;        /* last nks entry is the upper bound of the last span - set its count to zero */

    /* for every pair of output knot values - get number of pars values in that interval */
    for ( ii = 0; ii < nd; ii++ )
    {
        nks[ii] = ST_NumPtsInStrip  /* rtn: number of array value in specified param range (inclusive) */
                       ( pars,      /* in : sorted array of param values to examine                    */
                         k1,        /* in : min pars index to examine                                  */
                         ii2,       /* in : max pars index to examine                                  */
                         kts[ii],   /* in : min param value                                            */
                         kts[ii+1], /* in : max param value                                            */
                         &jj,       /* out: pars index of 1st value in param range (inclusive)         */
                         &k2 );     /* out: pars index of last value in param range (inclusive)        */
        k1 = k2;

        /* save min number of pars values in any single output knot interval */
        if( nks[ii] LT kk )
            kk = nks[ii];

        /* save max number of pars values in any single output knot interval */
        if( nks[ii] GT ll )
            ll = nks[ii];

        /* save max and min Ivl lengths */
        if(MinIvl GT (kts[ii+1] - kts[ii]))
          { MinIvl = (kts[ii+1] - kts[ii]) ; }
        if(MaxIvl LT (kts[ii+1] - kts[ii]))
          { MaxIvl = (kts[ii+1] - kts[ii]) ; }
    }

    /* set output */
    /* min and max number of points between any pair of output knots */
    *min   = kk;
    *max   = ll;
    *ratio = MaxIvl / MinIvl ; 

} /* end ST_AddKnotsStrip */

/**********************************************/
/*  compute number of parm points in a strip  */

NL_INDEX ST_NumPtsInStrip    /* rtn: number of array value in specified param range (inclusive) */
 ( NL_REAL *pars,            /* in : sorted array of param values to examine                    */
   NL_INDEX n1,              /* in : min pars index to examine                                  */
   NL_INDEX n2,              /* in : max pars index to examine                                  */
   NL_REAL pm1,              /* in : min param value                                            */
   NL_REAL pm2,              /* in : max param value                                            */
   NL_INDEX *i1,             /* out: pars index of 1st value in param range (inclusive)         */
   NL_INDEX *i2 )            /* out: pars index of last value in param range (inclusive)        */
{

    NL_INDEX ii1, ii2, nn;

    ii1 = ST_BinarySearch( pars, n1, n2, pm1, NL_LEFT );
    ii2 = ST_BinarySearch( pars, n1, n2, pm2, NL_RIGHT );

    nn = ii2 - ii1 + 1;

    if( nn LT 0 )
        nn = 0;

    if( ii1 GT n2 )
        ii1 = n2;

    if( ii2 LT n1 )
        ii2 = n1;

    *i1 = ii1;
    *i2 = ii2;

    return (nn);
}

/**********************************************/
/*  compute number of parm points in a box  */

NL_INDEX ST_NumPtsInBox( NL_REAL *p_sort, NL_INDEX *p_idx, NL_INDEX n1, NL_INDEX n2, NL_REAL *p_unsort, NL_REAL ps1, NL_REAL ps2, NL_REAL pu1, NL_REAL pu2 )
{

    NL_INDEX i1, i2, jj, kk, num;

    i1 = ST_BinarySearch( p_sort, n1, n2, ps1, NL_LEFT );
    i2 = ST_BinarySearch( p_sort, n1, n2, ps2, NL_RIGHT );

    num = 0;

    if( i2 LT i1 )
        return (num);

    if( i1 GT n2 )
        i1 = n2;

    if( i2 LT n1 )
        i2 = n1;

    for ( jj = i1; jj <= i2; jj++ )
    {
        kk = p_idx[jj];

        if( p_unsort[kk]GE pu1 AND p_unsort[kk]LE pu2 )
            num += 1;
    }

    return (num);
}

/****************************************************************************************/
/*        decide which subdivision of a strip is best                                   */
/* idea 1: functional =  G1 * IvlLengthRatio        (bigger ratios -> bigger costs)     */
/*                     + G2 * PointCntInOneIvlRatio (bigger ratios -> bigger costs)     */
/*                     + G3 * StandardDeviation     (bigger deviations -> bigger costs) */

NL_INDEX ST_BestSubdInStrip /* rtn: index of best knot sequence */
 (NL_INDEX    n,            /* in : Max KnotSequence index = Num of KnotSequences - 1 */
  NL_INDEX ** nums,         /* in : Array of num of points in each knot interval      */
  NL_INDEX    nu,           /* in : Max Knot index = Num of Knots in one sequence - 1 */
  NL_INDEX  * mins,         /* in : Array of min Point count in any one knot interval */
  NL_INDEX  * maxs,         /* in : Array of max Point count in any one knot interval */
  NL_REAL   * ratios)       /* in : Array of MaxIntervalLength/MinIntervalLength      */
{

    NL_INDEX ii, jj ;
    NL_REAL  avg, var, diffSq, ivl, thisCost, MaxMinRat, LowCost = 1.0e26 ;
    NL_INDEX LowCostIdx = 0 ;

    /* weight our different concerns - a bigger weight means penalize the knot sequence more.           */
    /* Put the biggest weight on the thing you want to minimize the most, e.g.                          */
    /* if uniform knot intervals are most important set LengthRatioGain = 100 and the others to 1.0     */
    /* if uniform samples per ivl are more important set VarGain = 100 and the others to 1.0            */
    /* if you require at least one sample per interval - set CntRatioGain = 100 and the others to 1.0   */
    /* various ratios between these gains will change what kinds of knot sequences are considered best  */
    /* Currently - the system favors equal intervals at the cost of constant sample points per interval */ 
    NL_REAL LengthRatioGain = 100.0 ;   /* weight to resist variations in interval lengths */
    NL_REAL CntRatioGain    = 1.0 ;     /* weight to resist large maxs[ii]/mins[ii] ratios */
    NL_REAL VarGain         = 10.0 ;    /* weight to resist large variations in number of samples per interval */

    /* for every KnotSequence */
    for ( ii = 0; ii <= n; ii++ )
    {
        if( mins[ii] LT 0 )
            continue; /* not used */

      /* get knot variance while scaling Nums values from 0 to 1 */
      avg = 0.0 ; 
      var = 0.0 ;
      ivl = ((NL_REAL)maxs[ii] - (NL_REAL)mins[ii]) ;
      if(ivl > 0.0)
        {
          for(jj=0;jj<nu;jj++)
            { avg += ((NL_REAL)nums[ii][jj] - (NL_REAL)mins[ii]) ; }
          avg = avg / ivl / nu ; 
          for(jj=0;jj<nu;jj++)
            { diffSq = avg - (((NL_REAL)nums[ii][jj] - (NL_REAL)mins[ii]) / ivl) ;
              var += diffSq * diffSq ;
            }
          var = var / nu ;
        }

       /* Max/Min points in one ivl ratio - limit divide by zeros */
       if(mins[ii] > 0) { MaxMinRat = maxs[ii] / mins[ii] ; }
       else             { MaxMinRat = 100 ; } /* arbitrary large number * to heavily penalize zero sample ivls */

       /* scale the var value by the larger of the ratio and MaxMinRat */
       /* to let the relative size differences in the gains have about the same weight */
       var = (ratios[ii] > MaxMinRat) ? ratios[ii] : MaxMinRat ;

       /* compute the cost */
       thisCost =   LengthRatioGain * ratios[ii]     /* ivl length ratio         >= 1.0 */
                  + CntRatioGain    * MaxMinRat      /* min pointCnt in one ivl  >= 1.0 */
                  + VarGain         * var ;          /* var pointCnt in ivls: [0 - 1]   */

       /* remember the lowest cost knot sequence */
       if(thisCost < LowCost)
         {
           LowCost    = thisCost ;
           LowCostIdx = ii ;
         }
     } /* end iter ii, all sequences to check */

   /* all done */
   return (LowCostIdx) ;

/* this was the old selection mechanism - which basically picked the biggest mins[ii] value */
/* its been replaced by a system which can trade off between sample distribution and knot interval size variations */
/* obsolete */
/*    NL_INDEX ii, jj, kdx, kmn, kmx, j1, j2;        */
/*                                                   */
/*    kmn = -1;                                      */
/*                                                   */
      /* for every KnotSequence */                   
/*    for ( ii = 0; ii <= n; ii++ )                  */
/*    {                                              */
/*        if( mins[ii] LT 0 )                        */
/*            continue;                              */
/*                                                   */
/*        if( mins[ii] LT kmn )                      */
/*            continue;                              */
/*                                                   */
          /* save the greatest min value seen */      
/*        if( mins[ii] GT kmn )                      */
/*        {                                          */
/*            kmn = mins[ii];                        */
/*            kmx = maxs[ii];                        */
/*            kdx = ii;                              */
/*            continue;                              */
/*        }                                          */
/*                                                   */
/*        j1 = j2 = 0;                               */
/*                                                   */
          /* count the number of knot intervals that have the min number of points */
/*        for ( jj = 0; jj <= nu; jj++ )             */
/*        {                                          */
/*            if( nums[ii][jj] EQ kmn )              */
/*                j1 += 1;                           */
/*                                                   */
/*            if( nums[kdx][jj] EQ kmn )             */
/*                j2 += 1;                           */
/*        }                                          */
/*                                                   */
          /* save KnotSequence with lowest number of KnotIntervals with the smallest number of knot intervals containing min point counts */ 
/*        if( j2 LT j1 )                             */
/*            continue;                              */
/*                                                   */
          /* if there is a tie - save the knotSequence with the smallest max number of points in one KnotInterval */
/*        if( j2 EQ j1 AND maxs[ii] GE kmx )         */
/*            continue;                              */
/*                                                   */
/*        kmn = mins[ii];                            */
/*        kmx = maxs[ii];                            */
/*        kdx = ii;                                  */
/*    }                                              */
/*                                                   */
/*    return (kdx);                                  */

} /* end ST_BestSubdInStrip */

/**********************************************************************/
/* ST_KnotInsertInPlace: Curve knot insertion in place                            */
/**********************************************************************/

NL_FLAG ST_KnotInsertInPlace( NL_CPOINT *Pw, NL_INDEX n, NL_DEGREE p, NL_KNOTVECTOR *knt, NL_REAL uParam )
{

    NL_FLAG errorFlag = NL_NO;

    NL_INDEX i, k, spn, mlt;

    NL_REAL *U, alf, oma;

    NL_CPOINT *Rw;

    NL_STACKS SLocal;

    N_InitNurbs( &SLocal );

    N_KnotVectorGetKnots( knt, &i, &U );

    errorFlag = N_BasisFindSpanAndMult( knt, p, uParam, NL_LEFT, &spn, &mlt );

    if( errorFlag EQ NL_YES )
        NL_OUT;

    k = spn - p;

    Rw = N_AllocCPt1dArray( p, &SLocal );

    if( Rw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p - mlt; i++ )
        N_CopyCPt( Pw[k + i], &Rw[i] );

    for ( i = n; i >= spn - mlt; i-- )
        N_CopyCPt( Pw[i], &Pw[i + 1] );

    k = spn - p + 1;

    for ( i = 0; i <= p - mlt - 1; i++ )
    {
        alf = (uParam - U[k + i]) / (U[spn + i + 1] - U[k + i]);
        oma = 1.0 - alf;
        N_Combine2CPts( alf, Rw[i + 1], oma, Rw[i], &Rw[i] );
    }
    N_CopyCPt( Rw[0], &Pw[k] );
    N_CopyCPt( Rw[p - mlt - 1], &Pw[spn - mlt] );

    for ( i = k + 1; i < spn - mlt; i++ )
        N_CopyCPt( Rw[i - k], &Pw[i] );

    N_EndNurbs( &SLocal );

    EXIT:

    return (errorFlag);
}

/**********************************************************************/
/* ST_CrvFitPts: Fit points with non-rational curve                       */
/**********************************************************************/

NL_FLAG ST_CrvFitPts( NL_VOID *A, NL_FLAG ptp, NL_RMATRIX *cm, NL_CPOINT *Pw )
{

    NL_FLAG errorFlag = NL_NO;

    NL_STACKS SLocal;

    N_InitNurbs( &SLocal );

    errorFlag = N_RealMatrixForBack( cm, A, ptp, Pw, &SLocal );

    if( errorFlag EQ NL_YES )
        NL_OUT;

    N_EndNurbs( &SLocal );

    EXIT:

    return (errorFlag);
}

/*************************************************************************************/
/* ST_CrvInterpDerivsKnots: Curve interpolation with end derivatives and knot vector */
/*************************************************************************************/

NL_FLAG ST_CrvInterpDerivsKnots( NL_VOID *A, NL_INDEX k, NL_FLAG ptp, NL_DEGREE p, NL_VOID *As, NL_VOID *Ae, NL_KNOTVECTOR *knt, NL_RMATRIX *cm, NL_CPOINT *Pw )
{

    NL_PRIVATE NL_STRING rname = _T("ST_CrvInterpDerivsKnots");

    NL_FLAG errorFlag = NL_NO;

    NL_INDEX i, n;

    NL_REAL *U, fact;

    NL_POINT *QPt, *RPt;

    NL_CPOINT *Qw, *Rw;

    NL_VECTOR Vs, Ve;

    NL_CVECTOR Ws, We;

    NL_STACKS SLocal;

    N_InitNurbs( &SLocal );

    N_KnotVectorGetKnots( knt, &n, &U );

    n = k + 2;

    switch( ptp )
    {
        case NL_EPOINT:

            RPt = (NL_POINT *)A;
            Vs = *(NL_VECTOR *)As;
            Ve = *(NL_VECTOR *)Ae;

            QPt = N_AllocPt1dArray( n, &SLocal );

            if( QPt EQ NULL )
                NL_QUIT;

            fact = (U[p + 1] - U[1]) / p;
            N_CopyPt( RPt[0], &QPt[0] );
            N_ScalePt( fact, Vs, &QPt[1] );

            for ( i = 2; i <= k; i++ )
                N_CopyPt( RPt[i - 1], &QPt[i] );

            fact = (U[n + p] - U[n]) / p;
            N_ScalePt( fact, Ve, &QPt[n - 1] );
            N_CopyPt( RPt[k], &QPt[n] );

            errorFlag = N_RealMatrixForBack( cm, (NL_VOID *)QPt, NL_EPOINT, Pw, &SLocal );

            if( errorFlag EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            Ws = *(NL_CVECTOR *)As;
            We = *(NL_CVECTOR *)Ae;

            Qw = N_AllocCPt1dArray( n, &SLocal );

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

            errorFlag = N_RealMatrixForBack( cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SLocal );

            if( errorFlag EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    N_EndNurbs( &SLocal );

    EXIT:

    return (errorFlag);
}

/***************************************************************************************/
/* ST_CrvInterpTangentsKnots: Curve interpolation with end derivatives and knot vector */
/***************************************************************************************/

NL_FLAG ST_CrvInterpTangentsKnots( NL_VOID *A, NL_INDEX k, NL_FLAG ptp, NL_DEGREE p, NL_VOID *T, NL_FLAG whr, NL_KNOTVECTOR *knt, NL_RMATRIX *cm, NL_CPOINT *Pw )
{

    NL_PRIVATE NL_STRING rname = _T("ST_CrvInterpTangentsKnots");

    NL_FLAG errorFlag = NL_NO;

    NL_INDEX i, n;

    NL_REAL *U, fact;

    NL_POINT *QPt, *RPt;

    NL_CPOINT *Qw, *Rw;

    NL_VECTOR V;

    NL_CVECTOR W;

    NL_STACKS SLocal;

    N_InitNurbs( &SLocal );

    N_KnotVectorGetKnots( knt, &n, &U );

    n = k + 1;

    switch( ptp )
    {
        case NL_EPOINT:

            RPt = (NL_POINT *)A;
            V = *(NL_VECTOR *)T;

            QPt = N_AllocPt1dArray( n, &SLocal );

            if(QPt EQ NULL )
                NL_QUIT;

            switch( whr )
            {
                case NL_START:

                    fact = (U[p + 1] - U[1]) / p;
                    N_CopyPt( RPt[0], &QPt[0] );
                    N_ScalePt( fact, V, &QPt[1] );

                    for ( i = 2; i <= n; i++ )
                        N_CopyPt( RPt[i - 1], &QPt[i] );
                    break;

                case NL_END:

                    fact = (U[n + p] - U[n]) / p;
                    N_ScalePt( fact, V, &QPt[n - 1] );
                    N_CopyPt( RPt[k], &QPt[n] );

                    for ( i = 0; i <= n - 2; i++ )
                        N_CopyPt( RPt[i], &QPt[i] );
                    break;

                default:
                    NL_ERROR( NL_CAL_ERR );
            }

            errorFlag = N_RealMatrixForBack( cm, (NL_VOID *)QPt, NL_EPOINT, Pw, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;
            break;

        case NL_HPOINT:

            Rw = (NL_CPOINT *)A;
            W = *(NL_CVECTOR *)T;

            Qw = N_AllocCPt1dArray( n, &SLocal );

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

            errorFlag = N_RealMatrixForBack( cm, (NL_VOID *)Qw, NL_HPOINT, Pw, &SLocal );

            if(errorFlag EQ NL_YES )
                NL_OUT;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    N_InitNurbs( &SLocal );

    EXIT:

    return (errorFlag);
}

/***************************************************************/

/******   Compute the area of minmax box of points   ******/
/******   which have been rotated by t degrees       ******/

NL_REAL ST_CalcAreaRotPts( NL_REAL t )
{

    NL_REAL cs, ss, x, y, minx, maxx, miny, maxy, tr;

    NL_INDEX ii;

    minx = miny = 0.001 *NL_BIGD;
    maxx = maxy = -minx;

    if( t EQ 0.0 ) /* this is a special case */
    {
        for ( ii = 0; ii <= glo_np; ii++ )
        {
            if( pts2[ii][0]LT minx )
                minx = pts2[ii][0];

            if( pts2[ii][0]GT maxx )
                maxx = pts2[ii][0];

            if( pts2[ii][1]LT miny )
                miny = pts2[ii][1];

            if( pts2[ii][1]GT maxy )
                maxy = pts2[ii][1];
        }
    }
    else
    {
        tr = t * (NL_PI / 180.0);
        cs = cos( tr );
        ss = sin( tr );

        for ( ii = 0; ii <= glo_np; ii++ )
        {
            p2[ii][0] = x = cs * pts2[ii][0] + ss * pts2[ii][1];
            p2[ii][1] = y = cs * pts2[ii][1] - ss * pts2[ii][0];

            if( x LT minx )
                minx = x;

            if( x GT maxx )
                maxx = x;

            if( y LT miny )
                miny = y;

            if( y GT maxy )
                maxy = y;
        }
    }

    return ((maxx - minx) * (maxy - miny));
}

#if NLIB_UNUSED
/**********************************************************************/
/* ST_CalcAreaSphereRotPtsGlobal: Area of sphere patch of rotated point set                */
/**********************************************************************/

NL_REAL ST_CalcAreaSphereRotPtsGlobal( NL_REAL ang )
{

    NL_INDEX i, iv = 0;

    NL_REAL a, b, c, bp, cp, alr, cosal, sinal, dot, dv, us, ue, vs, ve, AA;

    NL_POINT RXY;

    alr = NL_RAD * ang;
    cosal = cos( alr );
    sinal = sin( alr );

    for ( i = 0; i <= gn; i++ )
    {
        N_PtToXYZ( Q[i], &a, &b, &c );

        bp = b * cosal - c * sinal;
        cp = b * sinal + c * cosal;

        N_VectorCreate( a, bp, cp, &R[i] );
    }

    for ( i = 0; i <= gn; i++ )
    {
        N_PtToXYZ( R[i], &a, &b, &c );
        N_VectorCreate( a, b, 0.0, &RXY );

        error = N_VectorsAngle( R[i], NL_UNITZ, &u[i] );

        if( error EQ NL_YES )
            return (-1.0);

        N_DistPtPt( NL_ZERO, RXY, &c );

        if( c LT NL_MTOL )
        {
            v[i] = 0.0;
        }
        else
        {
            error = N_VectorsAngle( RXY, NL_UNITX, &v[i] );

            if( error EQ NL_YES )
                return (-1.0);

            N_VectorDot( RXY, NL_UNITY, &dot );

            if( dot LT 0.0 )
                v[i] = 360.0 - v[i];
        }

        vsort[i] = v[i];
    }

    N_ShellSortReal( vsort, gn );

    vsort[gn + 1] = 360.0 + vsort[0];

    a = b = -1.0;

    for ( i = 0; i <= gn; i++ )
    {
        dv = fabs( vsort[i + 1] - vsort[i] );

        if( dv GT b )
        {
            b = dv;
            iv = i;
        }
    }

    vs = vsort[iv + 1];
    ve = vsort[iv];

    if( vs GT ve )
    {
        for ( i = 0; i <= gn; i++ )
        {
            if( v[i]LE 360.0 AND v[i]GE vs )
                v[i] -= 360.0;
        }
    }

    us = 200.0;
    ue = -200.0;
    vs = 400.0;
    ve = -400.0;

    for ( i = 0; i <= gn; i++ )
    {
        if( u[i]LT us )
            us = u[i];

        if( u[i]GT ue )
            ue = u[i];

        if( v[i]LT vs )
            vs = v[i];

        if( v[i]GT ve )
            ve = v[i];
    }

    error = N_AreaSpherePatch( us, ue, vs, ve, &AA );

    if( error EQ NL_YES )
        return (-1.0);

    return AA;
}

/**********************************************************************/
/* ST_CalcAreaSphereRotPtsLocal: Area of sphere patch of rotated point set                */
/**********************************************************************/

NL_REAL ST_CalcAreaSphereRotPtsLocal( NL_REAL ang )
{

    NL_INDEX i, iv = 0;

    NL_REAL a, b, c, dot, dv, us, ue, vs, ve, AA;

    NL_POINT RXY;

    error = N_CreateRotationMatrixAboutAxis( NL_ZERO, N, ang, &rma, &SL );

    if( error EQ NL_YES )
        return (-1.0);

    for ( i = 0; i <= gn; i++ )
        N_TransformPt( Q[i], &rma, &R[i] );

    for ( i = 0; i <= gn; i++ )
    {
        N_PtToXYZ( R[i], &a, &b, &c );
        N_VectorCreate( a, b, 0.0, &RXY );

        error = N_VectorsAngle( R[i], NL_UNITZ, &u[i] );

        if( error EQ NL_YES )
            return (-1.0);

        N_DistPtPt( NL_ZERO, RXY, &c );

        if( c LT NL_MTOL )
        {
            v[i] = 0.0;
        }
        else
        {
            error = N_VectorsAngle( RXY, NL_UNITX, &v[i] );

            if( error EQ NL_YES )
                return (-1.0);

            N_VectorDot( RXY, NL_UNITY, &dot );

            if( dot LT 0.0 )
                v[i] = 360.0 - v[i];
        }

        vsort[i] = v[i];
    }

    N_ShellSortReal( vsort, gn );

    vsort[gn + 1] = 360.0 + vsort[0];

    a = b = -1.0;

    for ( i = 0; i <= gn; i++ )
    {
        dv = fabs( vsort[i + 1] - vsort[i] );

        if( dv GT b )
        {
            b = dv;
            iv = i;
        }
    }

    vs = vsort[iv + 1];
    ve = vsort[iv];

    if( vs GT ve )
    {
        for ( i = 0; i <= gn; i++ )
        {
            if( v[i]LE 360.0 AND v[i]GE vs )
                v[i] -= 360.0;
        }
    }

    us = 200.0;
    ue = -200.0;
    vs = 400.0;
    ve = -400.0;

    for ( i = 0; i <= gn; i++ )
    {
        if( u[i]LT us )
            us = u[i];

        if( u[i]GT ue )
            ue = u[i];

        if( v[i]LT vs )
            vs = v[i];

        if( v[i]GT ve )
            ve = v[i];
    }

    error = N_AreaSpherePatch( us, ue, vs, ve, &AA );

    if( error EQ NL_YES )
        return (-1.0);

    return AA;
}
#endif // NLIB_UNUSED
