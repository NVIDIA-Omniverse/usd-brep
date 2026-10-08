// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* NL_Bezier.c : Utility Function Definitions                                  */
/*****************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/* file local declarations */

/* Prototype for recursive routine */

NL_FLAG ST_BezCalcArcLength( NL_CPOINT *Pw, NL_DEGREE p, NL_REAL *eps, NL_REAL *len, int depth );

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  computes  the degree  elevation matrix for 
     elevating the degree by any given number. If the output matrix is 
     initialized to NULL, memory to store the elements is allocated. A 
     typical calling example is as follows:

       NL_DEGREE   p;
       NL_INDEX    t;
       NL_RMATRIX  dm;
       NL_STACKS   S;
       ...
       N_InitRealMatrix(&dm);
       N_BezGetDegreeElevationMatrix(p,t,&dm,&S);

     If memory is  available, dm is not  initialized and  the  routine
     assumes that memory allocation has  been done. However, it checks  
     for the proper  amount by looking at  the highest indexes in dm's 
     structure.


   ACCESS:
   
     p  , input  ,  Original degree
     t  , input  ,  Increment (new degree is p+t)
     dm , output ,  Degree elevation matrix
     SG , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezGetDegreeElevationMatrix( NL_DEGREE p, NL_INDEX t, NL_RMATRIX *dm, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezGetDegreeElevationMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l;

    /*        NL_INTEGER  **bin;    */

    NL_DEGREE q, r;

    NL_REAL ** RM, inv;

    /* NL_STACKS   SL; */

    /* Start NURBS */

    /* N_InitNurbs(&SL); */

    /* See if memory is needed */

    q = (NL_DEGREE)( p + t );

    error = N_CheckMemRealMatrix( dm, q, p, NL_MT_FULL, p, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( dm, &RM );

    /* Get binomial coefficients */

    /*      bin = N_AllocInt2dArray(q,q,&SL);    */
    /*      if( bin EQ NULL )  NL_QUIT;    */
    /*                                  */
    /*      N_PascalTriRow(bin,q);            */

    /* Compute matrix */

    r = (NL_DEGREE)( q / 2 );

    RM[0][0] = 1.0;
    RM[q][p] = 1.0;

    for ( i = 1; i <= r; i++ )
    {
        inv = 1.0 / NL_PascalTri[q][i];

        if( (i - t)GT 0 )
            k = i - t;
        else
            k = 0;

        if( i GT p )
            l = p;
        else
            l = i;

        for ( j = k; j <= l; j++ )
            RM[i][j] = inv * NL_PascalTri[p][j] * NL_PascalTri[t][i - j];
    }

    for ( i = r + 1; i < q; i++ )
    {
        if( (i - t)GT 0 )
            k = i - t;
        else
            k = 0;

        if( i GT p )
            l = p;
        else
            l = i;

        for ( j = k; j <= l; j++ )
            RM[i][j] = RM[q - i][p - j];
    }

    /* End NURBS and Exit */

    EXIT:

    /* N_EndNurbs(&SL); */

    return (error);
} /* end N_BezGetDegreeElevationMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine  computes the matrix  necessary to compute the
     product of two Bezier entities (functions/curves/surfaces). If the 
     output matrix is initialized to NULL, memory is allocated to store 
     the elements. A typical calling example is as follows:

       NL_DEGREE   p, q;
       NL_RMATRIX  pm;
       NL_STACKS   S;
       ...
       N_InitRealMatrix(&pm);
       N_BezFuncMultiplyBezCrvMatrix(p,q,&pm,&S);

     If memory is  available, pm is not  initialized and  the  routine
     assumes that memory allocation has  been done. However, it checks  
     for the proper  amount by looking at  the highest indexes in pm's 
     structure.


   ACCESS:
   
     p  , input  ,  Degree of first Bezier entity
     q  , input  ,  Degree of second Bezier entity
     pm , output ,  Product matrix
     SG , input  ,  pm's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezFuncMultiplyBezCrvMatrix( NL_DEGREE p, NL_DEGREE q, NL_RMATRIX *pm, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezFuncMultiplyBezCrvMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh, n, m, r;

    /*        NL_INTEGER  **bin;    */

    NL_REAL ** A, inv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if memory is needed */

    n = p + q;
    m = p;

    error = N_CheckMemRealMatrix( pm, n, m, NL_MT_FULL, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( pm, &A );

    /* Get binomial coefficients */

    /*        bin = N_AllocInt2dArray(n,n,&SL);    */
    /*        if( bin EQ NULL )  NL_QUIT;    */
    /*                                    */
    /*        N_PascalTriRow(bin,n);            */

    /* Compute matrix */

    r = n / 2;

    A[0][0] = 1.0;
    A[n][m] = 1.0;

    for ( i = 1; i <= r; i++ )
    {
        inv = 1.0 / NL_PascalTri[n][i];

        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
            A[i][j] = inv * NL_PascalTri[p][j] * NL_PascalTri[q][i - j];
    }

    for ( i = r + 1; i < n; i++ )
    {
        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
            A[i][j] = A[n - i][m - j];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezFuncMultiplyBezCrvMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine elevates  the degree  of a curve  from any 
     degree to any  higher degree. A typical  calling  example  is as
     follows:

       NL_CPOINT   *Pw, *Qw;
       NL_DEGREE   p;
       NL_INDEX    f, l, t;
       NL_RMATRIX  dm;
       ...
       (define Pw and allocate memory for Qw);
       ...
       N_InitRealMatrix(&dm); 
       N_BezElevateDegree(Pw,p,t,&dm,f,l,Qw);

     The degree elevation matrix can be precomputed and passed in. No
     memory is allocated  inside the routine in  that case. Also, the 
     number of  control  points  computed can  be  restricted  by the 
     indexes "f"  (first)  and  "l" (last). THE  ROUTINE ASSUMES THAT 
     SUFFICIENT  MEMORY IS AVAILABLE TO  STORE THE NL_NEW CONTROL NL_POINTS 
     "Qw".


   ACCESS:
   
     Pw  , input  ,  Original control points
     p   , input  ,  Original degree
     t   , input  ,  Increment (new degree is p+t)
     dm  , input  ,  Degree elevation matrix
     f,l , input  ,  First and last indexes
     Qw  , output ,  New control points after degree elevation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezElevateDegree( NL_CPOINT *Pw, NL_DEGREE p, NL_INDEX t, NL_RMATRIX *dm, NL_INDEX f, NL_INDEX l, NL_CPOINT *Qw )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, a, b;

    NL_REAL ** RM;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* See if degree elevation matrix is needed */

    if( N_RealMatrixIsNULL( dm ) )
    {
        error = N_BezGetDegreeElevationMatrix( p, t, dm, &S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( dm, &RM );

    /* Compute new control points */

    for ( i = f; i <= l; i++ )
    {
        if( (i - t)GT 0 )
            a = i - t;
        else
            a = 0;

        if( i GT p )
            b = p;
        else
            b = i;

        N_CopyCPt( NL_CZERO, &Qw[i] );

        for ( j = a; j <= b; j++ )
        {
            N_VectorBlendCPt( RM[i][j], Pw[j], &Qw[i] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezElevateDegree */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier routine elevates  the degree of a function from any 
     degree to  any higher degree (below the maximum allowed degree). 
     A typical calling example is as follows:

       NL_REAL    *f, *g;
       NL_DEGREE   p;
       NL_INDEX    s, e, t;
       NL_RMATRIX  dm;
       ...
       (define f; get s and e; allocate memory for g);
       ...
       N_InitRealMatrix(&dm); 
       N_BezFuncDegreeElevate(f,p,t,&dm,s,e,g);

     The degree elevation matrix can be precomputed and passed in. No
     memory is allocated  inside the routine in  that case. Also, the 
     number of  control  values  computed can  be  restricted  by the 
     indexes "s"  (start)  and  "e" (end). THE  ROUTINE  ASSUMES THAT 
     SUFFICIENT  MEMORY IS AVAILABLE TO  STORE THE NL_NEW CONTROL VALUES 
     "g".


   ACCESS:
   
     f   , input  ,  Original control values
     p   , input  ,  Original degree
     t   , input  ,  Increment (new degree is p+t)
     dm  , input  ,  Degree elevation matrix
     s,e , input  ,  Start and end indexes
     g   , output ,  New control values after degree elevation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezFuncDegreeElevate( NL_REAL *f, NL_DEGREE p, NL_INDEX t, NL_RMATRIX *dm, NL_INDEX s, NL_INDEX e, NL_REAL *g )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezFuncDegreeElevate");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh;

    NL_REAL ** A;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Check indexes and degree */

    if( s LT 0 OR e GT p + t )
        NL_ERROR( NL_INP_ERR );

    if( t LT 0 OR s GT e )
        NL_ERROR( NL_INP_ERR );

    if( p + t GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* See if degree elevation matrix is needed */

    if( N_RealMatrixIsNULL( dm ) )
    {
        error = N_BezGetDegreeElevationMatrix( p, t, dm, &S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( dm, &A );

    /* Compute new control values */

    for ( i = s; i <= e; i++ )
    {
        g[i] = 0.0;

        jl = NL_MAX( 0, i - t );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
            g[i] += A[i][j] * f[j];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezFuncDegreeElevate */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the control values of the product of 
     two Bezier functions. Given functions  f  of degree  p, and  g of 
     degree  q, the product  fg is  another Bezier  function of degree 
     p+q. A typical calling example is as follows:

       NL_REAL     *f, *g, *fg;
       NL_DEGREE   p, q;
       NL_INDEX    s, e;
       NL_RMATRIX  pm;
       ...
       (get f, g, p, q, s, e; allocate memory for fg);
       ...
       N_InitRealMatrix(&pm); 
       N_BezFuncMultiplyBezFunc(f,p,g,q,&pm,s,e,fg);

     The product computation matrix can be  precomputed and passed in. 
     If it is initialized to the NULL matrix, it  is  computed  inside 
     the routine. Also, the number of control values can be restricted 
     by  the indexes  "s" (start)  and "e" (end). THE ROUTINE  ASSUMES 
     THAT SUFFICIENT MEMORY IS AVAILABLE TO STORE VALUES OF fg. 


   ACCESS:
   
     f   , input  ,  Control values of first function
     p   , input  ,  Degree of first function
     g   , input  ,  Control values of second function
     q   , input  ,  Degree of second function
     pm  , input  ,  Product computation matrix
     s,e , input  ,  Start and end indexes (must satisfy 0<=s<=e<=p+q)
     fg  , output ,  Control  values of the  product  function (MEMORY
                     MUST BE ALLOCATED IN THE  CALLING ROUTINE TO HOLD
                     UP TO fg[p+q])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezFuncMultiplyBezFunc( NL_REAL *f, NL_DEGREE p, NL_REAL *g, NL_DEGREE q, NL_RMATRIX *pm, NL_INDEX s, NL_INDEX e, NL_REAL *fg )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezFuncMultiplyBezFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh;

    NL_REAL ** A;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( s LT 0 OR e GT p + q OR s GT e )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrix is needed */

    if( N_RealMatrixIsNULL( pm ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, q, pm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pm, &A );

    /* Compute control values */

    for ( i = s; i <= e; i++ )
    {
        fg[i] = 0.0;

        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
        {
            fg[i] += A[i][j] * f[j] * g[i - j];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezFuncMultiplyBezFunc */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the control  values of the product of 
     two bivariate Bezier functions. Given functions f of degree (p,q), 
     and g of degree (r,s), the product f*g is another Bezier  function 
     of degree (p+r,q+s). A typical calling example is as follows:

       NL_REAL     **f, **g, **fg;
       NL_DEGREE   p, q, r, s;
       NL_INDEX    su, eu, sv, ev;
       NL_RMATRIX  pmu, pmv;
       ...
       (get f, g,..., ev; allocate memory for fg);
       ...
       N_InitRealMatrix(&pmu); 
       N_InitRealMatrix(&pmv); 
       N_BezSrfFuncMultiply(f,p,q,g,r,s,&pmu,&pmv,su,eu,sv,ev,fg);

     The product computation matrix can be  precomputed  and passed in. 
     If it is initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the number  of control values can be restricted 
     by  the indexes  su,....,ev. THE  ROUTINE  ASSUMES THAT SUFFICIENT 
     MEMORY IS AVAILABLE TO STORE VALUES OF fg. 


   ACCESS:
   
     f     , input  ,  Control values of first function
     p,q   , input  ,  Degrees of first function
     g     , input  ,  Control values of second function
     r,s   , input  ,  Degrees of second function
     pmu   , input  ,  Product computation matrix in u-direction
     pmv   , input  ,  Product computation matrix in v-direction
     su,eu , input  ,  Start  and  end  indexes  in  u-direction  (must 
                       satisfy 0<=su<=eu<=p+r)
     sv,ev , input  ,  Start  and  end  indexes  in  v-direction  (must 
                       satisfy 0<=sv<=ev<=q+s)
     fg    , output ,  Control values of the  product  function (MEMORY
                       MUST BE ALLOCATED IN THE CALLING ROUTINE TO HOLD
                       UP TO fg[p+r][q+s])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfFuncMultiply( NL_REAL ** f, NL_DEGREE p, NL_DEGREE q, NL_REAL ** g, NL_DEGREE r, NL_DEGREE s, NL_RMATRIX *pmu, NL_RMATRIX *pmv, NL_INDEX su, NL_INDEX eu, NL_INDEX sv, NL_INDEX ev, NL_REAL ** fg )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfFuncMultiply");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, kl, kh, ll, lh;

    NL_REAL ** U, ** V;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( su LT 0 OR eu GT p + r OR su GT eu )
        NL_ERROR( NL_INP_ERR );

    if( sv LT 0 OR ev GT q + s OR sv GT ev )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrices are needed */

    if( N_RealMatrixIsNULL( pmu ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, r, pmu, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( N_RealMatrixIsNULL( pmv ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( q, s, pmv, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pmu, &U );
    N_GetRealMatrixPtr( pmv, &V );

    /* Compute control values */

    for ( i = su; i <= eu; i++ )
    {
        for ( j = sv; j <= ev; j++ )
        {
            fg[i][j] = 0.0;

            kl = NL_MAX( 0, i - r );
            kh = NL_MIN( p, i );
            ll = NL_MAX( 0, j - s );
            lh = NL_MIN( q, j );

            for ( k = kl; k <= kh; k++ )
            {
                for ( l = ll; l <= lh; l++ )
                {
                    fg[i][j] += U[i][k] * V[j][l] * f[k][l] * g[i - k][j - l];
                }
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfFuncMultiply */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine  computes the control points of the product of 
     a Bezier function and a Bezier curve. Given a function f of degree 
     p, and  a curve C of degree q, the product  f*C is  another Bezier  
     curve of degree p+q. A typical calling example is as follows:

       NL_REAL     *f;
       NL_CPOINT   *Pw, *Qw;
       NL_DEGREE   p, q;
       NL_INDEX    s, e;
       NL_RMATRIX  pm;
       ...
       (get f, Pw, p, q, s, e; allocate memory for Qw);
       ...
       N_InitRealMatrix(&pm); 
       N_BezFuncMultiplyBezCrv(f,p,Pw,q,&pm,s,e,Qw);

     The product computation matrix can be  precomputed and  passed in. 
     If it is initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the number  of control points can be restricted 
     by  the indexes  "s" (start)  and  "e" (end). THE ROUTINE  ASSUMES 
     THAT SUFFICIENT MEMORY IS AVAILABLE TO STORE Qw.


   ACCESS:
   
     f   , input  ,  Control values of function
     p   , input  ,  Degree of function
     Pw  , input  ,  Control points of curve
     q   , input  ,  Degree of curve
     pm  , input  ,  Product computation matrix
     s,e , input  ,  Start and end indexes (must satisfy  0<=s<=e<=p+q)
     Qw  , output ,  Control  points of product  curve (MEMORY  MUST BE 
                     ALLOCATED  IN THE  CALLING ROUTINE  TO HOLD  UP TO 
                     Qw[p+q])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezFuncMultiplyBezCrv( NL_REAL *f, NL_DEGREE p, NL_CPOINT *Pw, NL_DEGREE q, NL_RMATRIX *pm, NL_INDEX s, NL_INDEX e, NL_CPOINT *Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezFuncMultiplyBezCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh;

    NL_REAL ** A;

    NL_CPOINT Tw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( s LT 0 OR e GT p + q OR s GT e )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrix is needed */

    if( N_RealMatrixIsNULL( pm ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, q, pm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pm, &A );

    /* Compute control points */

    for ( i = s; i <= e; i++ )
    {
        N_CopyCPt( NL_CZERO, &Qw[i] );

        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
        {
            N_ScaleCPtXYZ( f[j], Pw[i - j], &Tw );
            N_VectorBlendCPt( A[i][j], Tw, &Qw[i] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezFuncMultiplyBezCrv */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes  the control points of  the product of 
     a bivariate Bezier function and a Bezier surface. Given a  function 
     f of degree (p,q), and a surface S of degree (r,s), the product f*S 
     is another Bezier  surface of degree  (p+r,q+s). A  typical calling 
     example is as follows:

       NL_REAL     **f;
       NL_CPOINT   **Pw, **Qw;
       NL_DEGREE   p, q, r, s;
       NL_INDEX    su, eu, sv, ev;
       NL_RMATRIX  pmu, pmv;
       ...
       (get f, Pw,..., ev; allocate memory for Qw);
       ...
       N_InitRealMatrix(&pmu); 
       N_InitRealMatrix(&pmv); 
       N_BezSrfMultiplySrfFunc(f,p,q,Pw,r,s,&pmu,&pmv,su,eu,sv,ev,Qw);

     The product computation  matrix can be  precomputed  and passed in. 
     If it is  initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the number  of control points  can be restricted 
     by  the indexes  su,....,ev. THE  ROUTINE  ASSUMES  THAT SUFFICIENT 
     MEMORY IS AVAILABLE TO STORE THE CONTROL NL_POINTS Qw[0..p+r][0..q+s]. 


   ACCESS:
   
     f     , input  ,  Control values of function
     p,q   , input  ,  Degrees of function
     Pw    , input  ,  Control points of surface
     r,s   , input  ,  Degrees of surface
     pmu   , input  ,  Product computation matrix in u-direction
     pmv   , input  ,  Product computation matrix in v-direction
     su,eu , input  ,  Start  and   end  indexes  in  u-direction  (must 
                       satisfy 0<=su<=eu<=p+r)
     sv,ev , input  ,  Start  and  end   indexes  in  v-direction  (must 
                       satisfy 0<=sv<=ev<=q+s)
     Qw    , output ,  Control  points of  the  product  surface (MEMORY
                       MUST BE ALLOCATED IN  THE CALLING ROUTINE TO HOLD
                       UP TO Qw[p+r][q+s])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfMultiplySrfFunc( NL_REAL ** f, NL_DEGREE p, NL_DEGREE q, NL_CPOINT ** Pw, NL_DEGREE r, NL_DEGREE s, NL_RMATRIX *pmu, NL_RMATRIX *pmv, NL_INDEX su, NL_INDEX eu, NL_INDEX sv, NL_INDEX ev, NL_CPOINT ** Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfMultiplySrfFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, kl, kh, ll, lh;

    NL_REAL ** U, ** V;

    NL_CPOINT Tw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( su LT 0 OR eu GT p + r OR su GT eu )
        NL_ERROR( NL_INP_ERR );

    if( sv LT 0 OR ev GT q + s OR sv GT ev )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrices are needed */

    if( N_RealMatrixIsNULL( pmu ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, r, pmu, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( N_RealMatrixIsNULL( pmv ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( q, s, pmv, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pmu, &U );
    N_GetRealMatrixPtr( pmv, &V );

    /* Compute control points */

    for ( i = su; i <= eu; i++ )
    {
        for ( j = sv; j <= ev; j++ )
        {
            N_CopyCPt( NL_CZERO, &Qw[i][j] );

            kl = NL_MAX( 0, i - r );
            kh = NL_MIN( p, i );
            ll = NL_MAX( 0, j - s );
            lh = NL_MIN( q, j );

            for ( k = kl; k <= kh; k++ )
            {
                for ( l = ll; l <= lh; l++ )
                {
                    N_ScaleCPtXYZ( f[k][l], Pw[i - k][j - l], &Tw );
                    N_VectorBlendCPt( U[i][k] * V[j][l], Tw, &Qw[i][j] );
                }
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfMultiplySrfFunc */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the dot product of two  curves. Both 
     rational  and non-rational curves are handled. The  product  is a
     rational or non-rational Bezier function computed as follows:

                              N_1(u)   N_2(u)   num(u)
            C_1(u) * C_2(u) = ------ * ------ = ------
                              D_1(u)   D_2(u)   den(u)

     where num(u) and den(u) are  Bezier functions. A typical  calling  
     example is:

       NL_CPOINT   *Pw, *Qw;
       NL_DEGREE   p, q;
       NL_INDEX    s, e;
       NL_REAL     *num, *den;
       NL_RMATRIX  pm;
       ...
       (get Pw, Qw, p, q, s, e, and get memory for num and den);
       ...
       N_InitRealMatrix(&pm); 
       N_BezCrvDotProduct(Pw,p,Qw,q,&pm,s,e,num,den);

     The product computation matrix can be  precomputed and passed in. 
     If it is initialized to the NULL matrix, it  is  computed  inside 
     the routine. Also, the number of control points can be restricted 
     by the  indexes  "s" (start) and  "e" (end). THE  ROUTINE ASSUMES 
     THAT SUFFICIENT MEMORY IS  AVAILABLE TO  STORE VALUES  OF num AND 
     den.


   ACCESS:
   
     Pw  , input  ,  Control points of first curve
     p   , input  ,  Degree of first curve
     Qw  , input  ,  Control points of second curve
     q   , input  ,  Degree of second curve
     pm  , input  ,  Product computation matrix
     s,e , input  ,  Start and end indexes (must satisfy 0<=s<=e<=p+q)
     num , output ,  Control values of the product of the  numerators.
                     MUST HAVE  ENOUGH  MEMORY TO  HOLD  VALUES  UP TO 
                     num[p+q]. 
     den , output ,  Control  values of the  prod of the denominators.
                     MUST  HAVE ENOUGH  MEMORY  TO  HOLD  VALUES UP TO 
                     den[p+q]. IF  THE  NL_CURVE  IS  NON-RATIONAL,  THIS 
                     ARRAY IS NOT NL_USED AND HENCE IT REMAINS NL_UNDEFINED!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCrvDotProduct( NL_CPOINT *Pw, NL_DEGREE p, NL_CPOINT *Qw, NL_DEGREE q, NL_RMATRIX *pm, NL_INDEX s, NL_INDEX e, NL_REAL *num, NL_REAL *den )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezCrvDotProduct");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX i, j, jl, jh;

    NL_REAL ** A, wp, wq, dot, dow;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes and set rational flag */

    N_CPtGetW( Pw[0], &wp );
    N_CPtGetW( Qw[0], &wq );

    if( s LT 0 OR e GT p + q OR s GT e )
        NL_ERROR( NL_INP_ERR );

    if( wp NEQ NL_NOW OR wq NEQ NL_NOW )
        rat = NL_YES;

    /* See if product matrix is needed */

    if( N_RealMatrixIsNULL( pm ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, q, pm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pm, &A );

    /* Compute control values */

    for ( i = s; i <= e; i++ )
    {
        num[i] = 0.0;

        if( rat EQ NL_YES )
            den[i] = 0.0;

        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
        {
            N_Dot2CPts( Pw[j], Qw[i - j], &dot, &dow );

            num[i] += A[i][j] * dot;

            if( rat EQ NL_YES )
                den[i] += A[i][j] * dow;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezCrvDotProduct */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  computes the  control  points of  the cross 
     product of two Bezier curves. Given a curve C_1 of degree p, and a 
     curve C_2 of degree q, the  cross  product  C_1 x C_2  is  another 
     Bezier  curve of  degree  p+q. A  typical  calling  example  is as 
     follows:

       NL_CPOINT   *Pw, *Qw, *Rw;
       NL_DEGREE   p, q;
       NL_INDEX    s, e;
       NL_RMATRIX  pm;
       ...
       (get Pw, Qw, p, q, s, e; allocate memory for Rw);
       ...
       N_InitRealMatrix(&pm); 
       N_BezCrossProduct(Pw,p,Qw,q,&pm,s,e,Rw);

     The product computation matrix can be  precomputed and  passed in. 
     If it is initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the number  of control points can be restricted 
     by  the indexes  "s" (start)  and  "e" (end). THE ROUTINE  ASSUMES 
     THAT SUFFICIENT MEMORY IS AVAILABLE TO STORE Rw.


   ACCESS:
   
     Pw  , input  ,  Control points of first curve
     p   , input  ,  Degree of first curve
     Qw  , input  ,  Control points of second curve
     q   , input  ,  Degree of second curve
     pm  , input  ,  Product computation matrix
     s,e , input  ,  Start and end indexes (must satisfy  0<=s<=e<=p+q)
     Rw  , output ,  Control  points of product  curve (MEMORY  MUST BE 
                     ALLOCATED  IN THE  CALLING ROUTINE  TO HOLD  UP TO 
                     Rw[p+q])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCrossProduct( NL_CPOINT *Pw, NL_DEGREE p, NL_CPOINT *Qw, NL_DEGREE q, NL_RMATRIX *pm, NL_INDEX s, NL_INDEX e, NL_CPOINT *Rw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezCrossProduct");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh;

    NL_REAL ** A;

    NL_CPOINT Tw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( s LT 0 OR e GT p + q OR s GT e )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrix is needed */

    if( N_RealMatrixIsNULL( pm ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, q, pm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pm, &A );

    /* Compute control points */

    for ( i = s; i <= e; i++ )
    {
        N_CopyCPt( NL_CZERO, &Rw[i] );

        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
        {
            N_Cross2CPts( Pw[j], Qw[i - j], &Tw );
            N_VectorBlendCPt( A[i][j], Tw, &Rw[i] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezCrossProduct */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the dot product of two surfaces. Both 
     rational  and non-rational surfaces are handled. The  product is a
     rational or non-rational Bezier function computed as follows:

                             N_1(u,v)   N_2(u,v)   num(u,v)
       S_1(u,v) * S_2(u,v) = -------- * -------- = --------
                             D_1(u,v)   D_2(u,v)   den(u,v)

     where  num(u,v)  and  den(u,v)  are  Bezier  functions. A  typical  
     calling  example is:

       NL_CPOINT   **Pw, **Qw;
       NL_REAL     **num, **den;
       NL_DEGREE   p, q, r, s;
       NL_INDEX    su, eu, sv, ev;
       NL_RMATRIX  pmu, pmv;
       ...
       (get Pw, Qw,..., ev; allocate memory for num and den);
       ...
       N_InitRealMatrix(&pmu); 
       N_InitRealMatrix(&pmv); 
       N_BezSrfDotProduct(Pw,p,q,Qw,r,s,&pmu,&pmv,su,eu,sv,ev,num,den);

     The product computation matrix can be  precomputed  and passed in. 
     If it is initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the number of control points  can be restricted 
     by  the indexes  su,....,ev. THE  ROUTINE  ASSUMES THAT SUFFICIENT 
     MEMORY IS AVAILABLE TO STORE VALUES OF num AND den. 


   ACCESS:
   
     Pw    , input  ,  Control points of first surface
     p,q   , input  ,  Degrees of first surface
     Qw    , input  ,  Control points of second surface
     r,s   , input  ,  Degrees of second surface
     pmu   , input  ,  Product computation matrix in u-direction
     pmv   , input  ,  Product computation matrix in v-direction
     su,eu , input  ,  Start  and  end  indexes  in  u-direction  (must 
                       satisfy 0<=su<=eu<=p+r)
     sv,ev , input  ,  Start  and  end  indexes  in  v-direction  (must 
                       satisfy 0<=sv<=ev<=q+s)
     num   , output ,  Control values of the product of the numerators.
                       MUST HAVE  ENOUGH  MEMORY TO  HOLD VALUES  UP TO 
                       num[p+r][q+s]. 
     den   , output ,  Control values of the  prod of the denominators.
                       MUST HAVE ENOUGH  MEMORY  TO  HOLD  VALUES UP TO 
                       den[p+r][q+s]. IF  THE NL_SURFACE  IS NON-RATIONAL,  
                       THIS  ARRAY IS  NOT  NL_USED AND  HENCE IT  REMAINS 
                       NL_UNDEFINED!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfDotProduct( NL_CPOINT ** Pw, NL_DEGREE p, NL_DEGREE q, NL_CPOINT ** Qw, NL_DEGREE r, NL_DEGREE s, NL_RMATRIX *pmu, NL_RMATRIX *pmv, NL_INDEX su, NL_INDEX eu, NL_INDEX sv, NL_INDEX ev, NL_REAL ** num, NL_REAL ** den )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfDotProduct");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX i, j, k, l, kl, kh, ll, lh;

    NL_REAL ** U, ** V, wp, wq, dot, dow;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes and rationality */

    N_CPtGetW( Pw[0][0], &wp );
    N_CPtGetW( Qw[0][0], &wq );

    if( su LT 0 OR eu GT p + r OR su GT eu )
        NL_ERROR( NL_INP_ERR );

    if( sv LT 0 OR ev GT q + s OR sv GT ev )
        NL_ERROR( NL_INP_ERR );

    if( wp NEQ NL_NOW OR wq NEQ NL_NOW )
        rat = NL_YES;

    /* See if product matrices are needed */

    if( N_RealMatrixIsNULL( pmu ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, r, pmu, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( N_RealMatrixIsNULL( pmv ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( q, s, pmv, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pmu, &U );
    N_GetRealMatrixPtr( pmv, &V );

    /* Compute control points */

    for ( i = su; i <= eu; i++ )
    {
        for ( j = sv; j <= ev; j++ )
        {
            num[i][j] = 0.0;

            if( rat EQ NL_YES )
                den[i][j] = 0.0;

            kl = NL_MAX( 0, i - r );
            kh = NL_MIN( p, i );
            ll = NL_MAX( 0, j - s );
            lh = NL_MIN( q, j );

            for ( k = kl; k <= kh; k++ )
            {
                for ( l = ll; l <= lh; l++ )
                {
                    N_Dot2CPts( Pw[k][l], Qw[i - k][j - l], &dot, &dow );

                    num[i][j] += U[i][k] * V[j][l] * dot;

                    if( rat EQ NL_YES )
                        den[i][j] += U[i][k] * V[j][l] * dow;
                }
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfDotProduct */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  computes  the  control  points of  the cross 
     product of two surfaces. Given two Bezier surfaces of degrees (p,q) 
     and (r,s), their cross product  is another Bezier surface of degree
     (p+r,q+s). A typical calling example is:

       NL_CPOINT   **Pw, **Qw, **Rw;
       NL_DEGREE   p, q, r, s;
       NL_INDEX    su, eu, sv, ev;
       NL_RMATRIX  pmu, pmv;
       ...
       (get Pw, Qw,..., ev; allocate memory for Rw);
       ...
       N_InitRealMatrix(&pmu); 
       N_InitRealMatrix(&pmv); 
       N_BezSrfCrossProduct(Pw,p,q,Qw,r,s,&pmu,&pmv,su,eu,sv,ev,Rw);

     The product computation matrix can be  precomputed  and  passed in. 
     If it is  initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the  number of control points  can be restricted 
     by  the  indexes  su,....,ev. THE  ROUTINE  ASSUMES THAT SUFFICIENT 
     MEMORY IS AVAILABLE TO STORE VALUES OF Rw.


   ACCESS:
   
     Pw    , input  ,  Control points of first surface
     p,q   , input  ,  Degrees of first surface
     Qw    , input  ,  Control points of second surface
     r,s   , input  ,  Degrees of second surface
     pmu   , input  ,  Product computation matrix in u-direction
     pmv   , input  ,  Product computation matrix in v-direction
     su,eu , input  ,  Start  and  end   indexes  in  u-direction  (must 
                       satisfy 0<=su<=eu<=p+r)
     sv,ev , input  ,  Start  and  end   indexes  in  v-direction  (must 
                       satisfy 0<=sv<=ev<=q+s)
     Rw    , output ,  Control  points  of  cross  product surface. MUST 
                       HAVE  ENOUGH   MEMORY   TO   HOLD  VALUES  UP  TO  
                       Rw[p+r][q+s]. 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfCrossProduct( NL_CPOINT ** Pw, NL_DEGREE p, NL_DEGREE q, NL_CPOINT ** Qw, NL_DEGREE r, NL_DEGREE s, NL_RMATRIX *pmu, NL_RMATRIX *pmv, NL_INDEX su, NL_INDEX eu, NL_INDEX sv, NL_INDEX ev, NL_CPOINT ** Rw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfCrossProduct");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, kl, kh, ll, lh;

    NL_REAL ** U, ** V;

    NL_CPOINT Tw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( su LT 0 OR eu GT p + r OR su GT eu )
        NL_ERROR( NL_INP_ERR );

    if( sv LT 0 OR ev GT q + s OR sv GT ev )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrices are needed */

    if( N_RealMatrixIsNULL( pmu ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, r, pmu, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( N_RealMatrixIsNULL( pmv ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( q, s, pmv, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pmu, &U );
    N_GetRealMatrixPtr( pmv, &V );

    /* Compute control points */

    for ( i = su; i <= eu; i++ )
    {
        for ( j = sv; j <= ev; j++ )
        {
            N_CopyCPt( NL_CZERO, &Rw[i][j] );

            kl = NL_MAX( 0, i - r );
            kh = NL_MIN( p, i );
            ll = NL_MAX( 0, j - s );
            lh = NL_MIN( q, j );

            for ( k = kl; k <= kh; k++ )
            {
                for ( l = ll; l <= lh; l++ )
                {
                    N_Cross2CPts( Pw[k][l], Qw[i - k][j - l], &Tw );
                    N_VectorBlendCPt( U[i][k] * V[j][l], Tw, &Rw[i][j] );
                }
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfCrossProduct */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  elevates the  degree of a  surface from any 
     degree to  any higher  degree  either  in u- or  v-direction.  The 
     routine computes only one row/column at a time. A typical  calling  
     example is as follows:

       NL_CPOINT   **Pw, **Qw;
       NL_DEGREE   r;
       NL_INDEX    f, l, roc, t;
       NL_RMATRIX  dm;
       ...
       (define Pw and allocate memory for Qw);
       ...
       N_InitRealMatrix(&dm); 
       N_BezSrfElevateDegree(Pw,r,t,&dm,NL_UDIR,f,l,roc,Qw);

     The degree elevation matrix can be precomputed  and passed in. No
     memory is  allocated inside the  routine in that  case. Also, the 
     number of  control points computed can  be restricted by the  "f" 
     and "l" indexes. THE  ROUTINE ASSUMES  THAT SUFFICIENT  MEMORY IS  
     AVAILABLE TO  STORE THE NL_NEW CONTROL NL_POINTS "Qw".


   ACCESS:
   
     Pw  , input  ,  Original control points
     r   , input  ,  Original degree
     t   , input  ,  Increment (new degree is r+t)
     dm  , input  ,  Degree elevation matrix
     f,l , input  ,  First and last  indexes used  in the  direction of
                     degree elevation
     roc , input  ,  Row or column index
     Qw  , output ,  New control points after degree elevation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfElevateDegree( NL_CPOINT ** Pw, NL_DEGREE r, NL_INDEX t, NL_RMATRIX *dm, NL_FLAG dir, NL_INDEX f, NL_INDEX l, NL_INDEX roc, NL_CPOINT ** Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfElevateDegree");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, k, a, b;
    NL_REAL ** RM;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* See if degree elevation matrix is needed */
    if( N_RealMatrixIsNULL( dm ) )
    {
        error = N_BezGetDegreeElevationMatrix( r, t, dm, &S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( dm, &RM );

    /* Compute new control points */
    switch( dir )
    {
        case NL_UDIR: /* Elevate in the u-direction */
            for ( i = f; i <= l; i++ )
            {
                if( (i - t)GT 0 )
                    a = i - t;
                else
                    a = 0;

                if( i GT r )
                    b = r;
                else
                    b = i;

                N_CopyCPt( NL_CZERO, &Qw[i][roc] );

                for ( k = a; k <= b; k++ )
                {
                    N_VectorBlendCPt( RM[i][k], Pw[k][roc], &Qw[i][roc] );
                }
            }
            break;

        case NL_VDIR: /* Elevate in the v-direction */
            for ( j = f; j <= l; j++ )
            {
                if( (j - t)GT 0 )
                    a = j - t;
                else
                    a = 0;

                if( j GT r )
                    b = r;
                else
                    b = j;

                N_CopyCPt( NL_CZERO, &Qw[roc][j] );

                for ( k = a; k <= b; k++ )
                {
                    N_VectorBlendCPt( RM[j][k], Pw[roc][k], &Qw[roc][j] );
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
} /* end N_BezSrfElevateDegree */


/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  elevates the  degree of a  volume from any 
     degree to  any higher  degree  either  in u-, v-, or w- direction.  
     The routine computes only one row/column/level at a time. 
     A typical  calling example is as follows:

       NL_CPOINT   ***Pw, ***Qw;
       NL_DEGREE   r;
       NL_INDEX    f, l, roc, col, t;
       NL_RMATRIX  dm;
       ...
       (define Pw and allocate memory for Qw);
       ...
       N_InitRealMatrix(&dm); 
       N_BezSrfElevateDegree(Pw,r,t,&dm,NL_UDIR,f,l,roc,col,Qw);

     The degree elevation matrix can be precomputed  and passed in. No
     memory is  allocated inside the  routine in that  case. Also, the 
     number of  control points computed can  be restricted by the  "f" 
     and "l" indexes. THE  ROUTINE ASSUMES  THAT SUFFICIENT  MEMORY IS  
     AVAILABLE TO  STORE THE NL_NEW CONTROL NL_POINTS "Qw".


   ACCESS:
   
     Pw  , input  ,  Original control points
     r   , input  ,  Original degree
     t   , input  ,  Increment (new degree is r+t)
     dm  , input  ,  Degree elevation matrix
     dir , input  ,  Direction to elevate, 
                     oneof: NL_UDIR, NL_VDIR, or NL_WDIR
     f,l , input  ,  First and last  indexes used  in the  direction of
                     degree elevation
     roc , input  ,  Row or column index 
                     When dir == NL_UDIR, roc = Column index
                          dir == NL_VDir, roc = Row    index
                          dir == NL_WDir, roc = Row    index
     col , input  ,  Col or Level index                
                     When dir == NL_UDIR, col = Level  index
                          dir == NL_VDir, col = Level  index
                          dir == NL_WDir, col = Column index
     Qw  , output ,  New control points after degree elevation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezVolumeDegreeElevate( NL_CPOINT *** Pw, NL_DEGREE r, NL_INDEX t, NL_RMATRIX *dm, NL_FLAG dir, NL_INDEX f, NL_INDEX l, NL_INDEX roc, NL_INDEX col, NL_CPOINT *** Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezVolumeDegreeElevate");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, kk, a, b;

    NL_REAL ** RM;

    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* See if degree elevation matrix is needed */
    if( N_RealMatrixIsNULL( dm ) )
    {
        error = N_BezGetDegreeElevationMatrix( r, t, dm, &S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( dm, &RM );

    /* Compute new control points */

    switch( dir )
    {
        case NL_UDIR: /* Elevate in the u-direction */
            for ( i = f; i <= l; i++ )
            {
                if( (i - t)GT 0 )
                    a = i - t;
                else
                    a = 0;

                if( i GT r )
                    b = r;
                else
                    b = i;

                N_CopyCPt( NL_CZERO, &Qw[i][roc][col] );

                for ( kk = a; kk <= b; kk++ )
                {
                    N_VectorBlendCPt( RM[i][kk], Pw[kk][roc][col], &Qw[i][roc][col] );
                }
            }
            break;

        case NL_VDIR: /* Elevate in the v-direction */
            for ( j = f; j <= l; j++ )
            {
                if( (j - t)GT 0 )
                    a = j - t;
                else
                    a = 0;

                if( j GT r )
                    b = r;
                else
                    b = j;

                N_CopyCPt( NL_CZERO, &Qw[roc][j][col] );

                for ( kk = a; kk <= b; kk++ )
                {
                    N_VectorBlendCPt( RM[j][kk], Pw[roc][kk][col], &Qw[roc][j][col] );
                }
            }
            break;

        case NL_WDIR: /* Elevate in the w-direction */
            for ( k = f; k <= l; k++ )
            {
                if( (k - t)GT 0 )
                    a = k - t;
                else
                    a = 0;

                if( k GT r )
                    b = r;
                else
                    b = k;

                N_CopyCPt( NL_CZERO, &Qw[roc][col][k] );

                for ( kk = a; kk <= b; kk++ )
                {
                    N_VectorBlendCPt( RM[k][kk], Pw[roc][col][kk], &Qw[roc][col][k] );
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
} /* end N_BezVolumeDegreeElevate */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine elevates the degree of a surface function from 
     any degree to any higher degree either in u- or  v-direction.  The 
     routine computes only one row/column at a time. A typical  calling  
     example is as follows:

       NL_REAL     **fp, **fq;
       NL_DEGREE   r;
       NL_INDEX    f, l, roc, t;
       NL_RMATRIX  dm;
       ...
       (define fp and allocate memory for fq);
       ...
       N_InitRealMatrix(&dm); 
       N_BezSrfFuncElevateDegree(fp,r,t,&dm,NL_UDIR,f,l,roc,fq);

     The degree elevation matrix  can be precomputed  and passed in. No
     memory is  allocated  inside the  routine in that  case. Also, the 
     number of  control points  computed can  be restricted by the  "f" 
     and "l" indexes. THE  ROUTINE  ASSUMES  THAT SUFFICIENT  MEMORY IS  
     AVAILABLE TO STORE THE NL_NEW CONTROL VALUES "fq".


   ACCESS:
   
     fp  , input  ,  Original control values
     r   , input  ,  Original degree
     t   , input  ,  Increment (new degree is r+t)
     dm  , input  ,  Degree elevation matrix
     f,l , input  ,  First and last  indexes used  in the  direction of
                     degree elevation
     roc , input  ,  Row or column index
     fq  , output ,  New  control values  after degree  elevation (must 
                     have memory to hold fq[0..r+t])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfFuncElevateDegree( NL_REAL ** fp, NL_DEGREE r, NL_INDEX t, NL_RMATRIX *dm, NL_FLAG dir, NL_INDEX f, NL_INDEX l, NL_INDEX roc, NL_REAL ** fq )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfFuncElevateDegree");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, a, b;

    NL_REAL ** RM;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* See if degree elevation matrix is needed */

    if( N_RealMatrixIsNULL( dm ) )
    {
        error = N_BezGetDegreeElevationMatrix( r, t, dm, &S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( dm, &RM );

    /* Compute new control values */

    switch( dir )
    {
        case NL_UDIR:
            for ( i = f; i <= l; i++ )
            {
                if( (i - t)GT 0 )
                    a = i - t;
                else
                    a = 0;

                if( i GT r )
                    b = r;
                else
                    b = i;

                fq[i][roc] = 0.0;

                for ( k = a; k <= b; k++ )
                    fq[i][roc] += RM[i][k] * fp[k][roc];
            }
            break;

        case NL_VDIR:
            for ( j = f; j <= l; j++ )
            {
                if( (j - t)GT 0 )
                    a = j - t;
                else
                    a = 0;

                if( j GT r )
                    b = r;
                else
                    b = j;

                fq[roc][j] = 0.0;

                for ( k = a; k <= b; k++ )
                    fq[roc][j] += RM[j][k] * fp[roc][k];
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezSrfFuncElevateDegree */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine evaluates a Bernstein  polynomial at a given
     parameter value. A typical calling example is as follows:

       NL_DEGREE     p;
       NL_INDEX      i;
       NL_PARAMETER  u;
       NL_REAL       B;
       ...
       N_BezEvalOneBasis(p,i,u,&B);

     The evaluation is based on the recursive definition of Bernstein
     polynomials.


   ACCESS:
   
     p  , input  ,  Degree of the polynomial
     i  , input  ,  Index; B[i](u) is to be computed
     u  , input  ,  Parameter
     B  , output ,  Polynomial value


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezEvalOneBasis( NL_DEGREE p, NL_INDEX i, NL_PARAMETER u, NL_REAL *B )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezEvalOneBasis");

    NL_FLAG error = NL_NO;

    NL_INDEX j, k;

    NL_REAL *tmp, omu;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Check error */

    if( i LT 0 OR i GT p )
        NL_ERROR( NL_IND_ERR );

    if( u LT 0.0 OR u GT 1.0 )
        NL_ERROR( NL_PAR_ERR );

    /* Special cases */

    if( u EQ 0.0 )
    {
        if( i EQ 0 )
            *B = 1.0;
        else
            *B = 0.0;

        NL_OUT;
    }

    if( u EQ 1.0 )
    {
        if( i EQ p )
            *B = 1.0;
        else
            *B = 0.0;

        NL_OUT;
    }

    /* Get local memory */

    tmp = N_AllocReal1dArray( p, &S );

    if( tmp EQ NULL )
        NL_QUIT;

    /* Compute zero degree polynomials */

    for ( j = 0; j <= p; j++ )
        tmp[j] = 0.0;

    tmp[p - i] = 1.0;

    /* Compute the i-th polynomial */

    omu = 1.0 - u;

    for ( k = 1; k <= p; k++ )
    {
        for ( j = p; j >= k; j-- )
        {
            tmp[j] = omu * tmp[j] + u * tmp[j - 1];
        }
    }

    *B = tmp[p];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezEvalOneBasis */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine evaluates all Bernstein polynomials at a given
     parameter value. A typical calling example is:

       NL_DEGREE     p;
       NL_PARAMETER  u;
       NL_REAL       *B;
       ...
       (get p, u, and allocate memory for B);
       ...
       N_BezEvalBasis(p,u,B);


   ACCESS:
   
     p  , input  ,  Degree of the polynomials
     u  , input  ,  Parameter
     B  , output ,  Polynomial values. MEMORY FOR  B MUST  BE ALLOCATED
                    IN THE CALLING ROUTINE TO HOLD VALUES UP TO B[p].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezEvalBasis( NL_DEGREE p, NL_PARAMETER u, NL_REAL *B )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezEvalBasis");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL temp, omu, saved;

    /* Check error */

    if( u LT 0.0 OR u GT 1.0 )
        NL_ERROR( NL_PAR_ERR );

    /* Special cases */

    if( u EQ 0.0 )
    {
        B[0] = 1.0;

        for ( i = 1; i <= p; i++ )
            B[i] = 0.0;

        NL_OUT;
    }

    if( u EQ 1.0 )
    {
        for ( i = 0; i < p; i++ )
            B[i] = 0.0;
        B[p] = 1.0;

        NL_OUT;
    }

    /* Compute all Bernstein polynomials */

    omu = 1.0 - u;
    B[0] = 1.0;

    for ( i = 1; i <= p; i++ )
    {
        saved = 0.0;

        for ( j = 0; j < i; j++ )
        {
            temp = B[j];
            B[j] = saved + omu * temp;
            saved = u * temp;
        }
        B[i] = saved;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_BezEvalBasis */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  computes a  point  on  a Bezier  curve by 
     evaluating all Bernstein polynomials and multiplying them by the 
     control points. A typical calling example is:

       NL_CPOINT     *Pw;
       NL_DEGREE     p;
       NL_PARAMETER  u;
       NL_POINT      C;
       ...
       (get Pw, p and u);
       ...
       N_BezCrvEvalPt(Pw,p,u,&C);


   ACCESS:
   
     Pw  , input  ,  Bezier control points
     p   , input  ,  Degree (highest index in Pw)
     u   , input  ,  Parameter value 
     C   , output ,  Point on the curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCrvEvalPt( NL_CPOINT *Pw, NL_DEGREE p, NL_PARAMETER u, NL_POINT *C )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL *B;

    NL_CPOINT Cw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute Bernstein polynomials */

    B = N_AllocReal1dArray( p, &SL );

    if( B EQ NULL )
        NL_QUIT;

    error = N_BezEvalBasis( p, u, B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the point on the curve */

    N_CopyCPt( NL_CZERO, &Cw );

    for ( i = 0; i <= p; i++ )
    {
        N_VectorBlendCPt( B[i], Pw[i], &Cw );
    }

    N_CPtToPtEuclid( Cw, C );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezCrvEvalPt */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes a point on a Bezier curve function 
     by evaluating all Bernstein polynomials and multiplying them by 
     the control coefficients. A typical calling example is:

       NL_REAL       *c, f;
       NL_DEGREE     p;
       NL_PARAMETER  u;
       ...
       (get u and coefficients c);
       ...
       N_BezFuncEvalPt(c,p,u,&f);


   ACCESS:
   
     c  , input  ,  Bezier coefficients
     p  , input  ,  Degree (highest index in c)
     u  , input  ,  Parameter value 
     f  , output ,  Value of Bezier function computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezFuncEvalPt( NL_REAL *c, NL_DEGREE p, NL_PARAMETER u, NL_REAL *f )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL *B;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute Bernstein polynomials */

    B = N_AllocReal1dArray( p, &SL );

    if( B EQ NULL )
        NL_QUIT;

    error = N_BezEvalBasis( p, u, B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute function value */

    *f = 0.0;

    for ( i = 0; i <= p; i++ )
    {
        *f += B[i] * c[i];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezFuncEvalPt */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine  computes  a point  on a  Bezier  surface by 
     evaluating all Bernstein  polynomials and multiplying them by the 
     control points. A typical calling example is:

       NL_CPOINT     **Pw;
       NL_POINT      S;
       NL_DEGREE     p, q;
       NL_PARAMETER  u, v;
       ...
       (get Pw, u and v);
       ...
       N_BezSrfEvalPt(Pw,p,q,u,v,&S);


   ACCESS:
   
     Pw  , input  ,  Bezier control points
     p,q , input  ,  Degrees (highest indexes in Pw)
     u,v , input  ,  Parameter values 
     S   , output ,  Surface point computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfEvalPt( NL_CPOINT ** Pw, NL_DEGREE p, NL_DEGREE q, NL_PARAMETER u, NL_PARAMETER v, NL_POINT *S )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL *BU, *BV;

    NL_CPOINT Sw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute Bernstein polynomials */

    BU = N_AllocReal1dArray( p, &SL );

    if( BU EQ NULL )
        NL_QUIT;

    BV = N_AllocReal1dArray( q, &SL );

    if( BV EQ NULL )
        NL_QUIT;

    error = N_BezEvalBasis( p, u, BU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BezEvalBasis( q, v, BV );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute surface point */

    N_CopyCPt( NL_CZERO, &Sw );

    for ( i = 0; i <= p; i++ )
    {
        for ( j = 0; j <= q; j++ )
            N_VectorBlendCPt( BU[i] * BV[j], Pw[i][j], &Sw );
    }
    N_CPtToPtEuclid( Sw, S );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfEvalPt */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes a point on a Bezier surface function 
     by evaluating  all Bernstein  polynomials and multiplying them by 
     the control coefficients. A typical calling example is:

       NL_REAL       **s, f;
       NL_DEGREE     p, q;
       NL_PARAMETER  u, v;
       ...
       (get u, v and coefficients s);
       ...
       N_BezSrfFuncEvalPt(s,p,q,u,v,&f);


   ACCESS:
   
     s   , input  ,  Bezier coefficients
     p,q , input  ,  Degrees (highest indexes in s)
     u,v , input  ,  Parameter values 
     f   , output ,  Value of Bezier function computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfFuncEvalPt( NL_REAL ** s, NL_DEGREE p, NL_DEGREE q, NL_PARAMETER u, NL_PARAMETER v, NL_REAL *f )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL *BU, *BV;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute Bernstein polynomials */

    BU = N_AllocReal1dArray( p, &SL );

    if( BU EQ NULL )
        NL_QUIT;

    BV = N_AllocReal1dArray( q, &SL );

    if( BV EQ NULL )
        NL_QUIT;

    error = N_BezEvalBasis( p, u, BU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BezEvalBasis( q, v, BV );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute function value */

    *f = 0.0;

    for ( i = 0; i <= p; i++ )
    {
        for ( j = 0; j <= q; j++ )
            *f += BU[i]*BV[j]*s[i][j];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfFuncEvalPt */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine  computes degree reduction  coefficients for
     reducing the  degree  by one. A typical  calling  example  is as
     follows:

       NL_DEGREE  p;
       NL_REAL    *alf, *oma, *bet, *omb;
       ...
       (allocate memory for alf, oma, bet, omb);
       ...
       N_BezDegreeReduceCoefs(p,alf,oma,bet,omb);

     THE ROUTINE ASSUMES THAT SUFFICIENT MEMORY IS AVAILABLE TO STORE 
     THE COEFFICIENTS.


   ACCESS:
   
     p       , input  ,  Original degree
     alf,oma , output ,  Coefficients  to reduce from  the left. Must 
                         have room for at least (p-2) elemets.
     bet,omb , output ,  Coefficients to reduce from the  right. Must 
                         have room for at least (p-2) elemets.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_VOID N_BezDegreeReduceCoefs( NL_DEGREE p, NL_REAL *alf, NL_REAL *oma, NL_REAL *bet, NL_REAL *omb )
{
    NL_INDEX i, r;

    /* Compute coefficients */

    r = (p - 1) / 2;

    for ( i = 1; i <= r; i++ )
    {
        alf[i] = (NL_REAL)p / ((NL_REAL)p - (NL_REAL)i);
        oma[i] = 1.0 - alf[i];
    }

    for ( i = p - 2; i >= r; i-- )
    {
        bet[i] = (NL_REAL)p / ((NL_REAL)i + 1);
        omb[i] = 1.0 - bet[i];
    }
} /* end N_BezDegreeReduceCoefs */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine reduces the  degree of a  curve by one  and
     computes the maximum error of the reduction. A  typical  calling 
     example is:

       NL_CPOINT  *Pw, *Qw;
       NL_DEGREE  p;
       NL_REAL    *alf, *oma, *bet, *omb, e;
       ...
       (define Pw, allocate memory for Qw and compute alf, oma, bet, 
        and omb);
       ...
       N_BezDegreeReduceCoefs(p,alf,oma,bet,omb);
       N_BezReduceDegree(Pw,p,alf,oma,bet,omb,Qw,&e);

     THE ROUTINE ASSUMES THAT SUFFICIENT MEMORY IS AVAILABLE TO STORE 
     THE NL_NEW CONTROL NL_POINTS "Qw" AND  THAT THE REDUCTION COEFFICIENTS 
     HAVE BEEN PRECOMPUTED.


   ACCESS:
   
     Pw  , input  ,  Original control points
     p   , input  ,  Original degree
     alf , input  ,  Reduction coefficients
     oma , input  ,  Reduction coefficients
     bet , input  ,  Reduction coefficients
     omb , input  ,  Reduction coefficients
     Qw  , output ,  New control points after degree reduction
     e   , output ,  Maximum error


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezReduceDegree( NL_CPOINT *Pw, NL_DEGREE p, NL_REAL *alf, NL_REAL *oma, NL_REAL *bet, NL_REAL *omb, NL_CPOINT *Qw, NL_REAL *e )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, r;

    NL_REAL a, b, u, B, B1, dw;

    NL_CPOINT PL, PR;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get end points */

    N_CopyCPt( Pw[0], &Qw[0] );
    N_CopyCPt( Pw[p], &Qw[p - 1] );

    /* Reduce the degree */

    r = (p - 1) / 2;

    if( p % 2 ) /* Odd degree */
    {
        /* Compute from the left */

        for ( i = 1; i <= r - 1; i++ )
        {
            N_Combine2CPts( alf[i], Pw[i], oma[i], Qw[i - 1], &Qw[i] );
        }

        /* Compute from the right */

        for ( i = p - 2; i >= r + 1; i-- )
        {
            N_Combine2CPts( bet[i], Pw[i + 1], omb[i], Qw[i + 1], &Qw[i] );
        }

        /* Compute middle control point */

        N_Combine2CPts( alf[r], Pw[r], oma[r], Qw[r - 1], &PL );
        N_Combine2CPts( bet[r], Pw[r + 1], omb[r], Qw[r + 1], &PR );
        N_Combine2CPts( 0.5, PL, 0.5, PR, &Qw[r] );

        /* Compute the error */

        u = 0.5 *( 1.0 - sqrt( 1.0 / p ) );

        error = N_BezEvalOneBasis( p, r, u, &B );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BezEvalOneBasis( p, r + 1, u, &B1 );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistCptCptHomo( PL, PR, &dw );

        a = 0.5 *( (NL_REAL)p - (NL_REAL)r ) / (NL_REAL)p;
        b = fabs( B - B1 );
        *e = a * b * dw;
    }
    else /* Even degree */
    {
        /* Compute from the left */

        for ( i = 1; i <= r; i++ )
        {
            N_Combine2CPts( alf[i], Pw[i], oma[i], Qw[i - 1], &Qw[i] );
        }

        /* Compute from the right */

        for ( i = p - 2; i >= r + 1; i-- )
        {
            N_Combine2CPts( bet[i], Pw[i + 1], omb[i], Qw[i + 1], &Qw[i] );
        }

        /* Compute the error */

        u = (r + 1.0) / p;

        error = N_BezEvalOneBasis( p, r + 1, u, &B1 );

        if( error EQ NL_YES )
            NL_OUT;

        N_Combine2CPts( 0.5, Qw[r], 0.5, Qw[r + 1], &PL );
        N_DistCptCptHomo( Pw[r + 1], PL, &dw );

        *e = B1 * dw;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezReduceDegree */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine reduces  the degree of a  surface by one  for
     one row/column of control  points. It also  computes the  maximum 
     error of the reduction. A typical calling example is as follows:

       NL_CPOINT  **Pw, **Qw;
       NL_DEGREE  p, q;
       NL_FLAG    dir;
       NL_INDEX   k;
       NL_REAL    *alf, *oma, *bet, *omb, *e;
       ...
       (define Pw, allocate memory for Qw and for the reduction 
        coefficients alf, oma, bet and omb);
       ...
       N_BezDegreeReduceCoefs(p,alf,oma,bet,omb);
       N_BezSrfReduceDegree(Pw,p,q,NL_UDIR,k,alf,oma,bet,omb,Qw,&e);

     THE ROUTINE ASSUMES THAT  SUFFICIENT MEMORY IS AVAILABLE TO STORE 
     THE NL_NEW CONTROL NL_POINTS "Qw" AND  THAT THE  REDUCTION COEFFICIENTS 
     HAVE BEEN PRECOMPUTED.


   ACCESS:
   
     Pw  , input  ,  Original control points
     p,q , input  ,  Original degrees
     dir , input  ,  Flag:
                       NL_UDIR: Reduce in u-direction
                       NL_VDIR: Reduce in v-direction
     k   , input  ,  Row/column index
     alf , input  ,  Reduction coefficients
     oma , input  ,  Reduction coefficients
     bet , input  ,  Reduction coefficients
     omb , input  ,  Reduction coefficients
     Qw  , output ,  New control points after degree reduction
     e   , output ,  Maximum error


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfReduceDegree( NL_CPOINT ** Pw, NL_DEGREE p, NL_DEGREE q, NL_FLAG dir, NL_INDEX k, NL_REAL *alf, NL_REAL *oma, NL_REAL *bet, NL_REAL *omb, NL_CPOINT ** Qw, NL_REAL *e )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfReduceDegree");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, r;

    NL_REAL a, b, u, B, B1, dw;

    NL_CPOINT PL, PR;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Check flag */

    if( dir NEQ NL_UDIR AND dir NEQ NL_VDIR )
        NL_ERROR( NL_CAL_ERR );

    /* Reduce in u-direction */

    if( dir EQ NL_UDIR )
    {
        /* Get end points */

        N_CopyCPt( Pw[0][k], &Qw[0][k] );
        N_CopyCPt( Pw[p][k], &Qw[p - 1][k] );

        /* Reduce the degree */

        r = (p - 1) / 2;

        if( p % 2 ) /* Odd degree */
        {
            /* Compute from the left */

            for ( i = 1; i <= r - 1; i++ )
            {
                N_Combine2CPts( alf[i], Pw[i][k], oma[i], Qw[i - 1][k], &Qw[i][k] );
            }

            /* Compute from the right */

            for ( i = p - 2; i >= r + 1; i-- )
            {
                N_Combine2CPts( bet[i], Pw[i + 1][k], omb[i], Qw[i + 1][k], &Qw[i][k] );
            }

            /* Compute middle control point */

            N_Combine2CPts( alf[r], Pw[r][k], oma[r], Qw[r - 1][k], &PL );
            N_Combine2CPts( bet[r], Pw[r + 1][k], omb[r], Qw[r + 1][k], &PR );
            N_Combine2CPts( 0.5, PL, 0.5, PR, &Qw[r][k] );

            /* Compute the error */

            u = 0.5 *( 1.0 - sqrt( 1.0 / p ) );

            error = N_BezEvalOneBasis( p, r, u, &B );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BezEvalOneBasis( p, r + 1, u, &B1 );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistCptCptHomo( PL, PR, &dw );

            a = 0.5 *( (NL_REAL)p - (NL_REAL)r ) / (NL_REAL)p;
            b = fabs( B - B1 );
            *e = a * b * dw;
        }
        else /* Even degree */
        {
            /* Compute from the left */

            for ( i = 1; i <= r; i++ )
            {
                N_Combine2CPts( alf[i], Pw[i][k], oma[i], Qw[i - 1][k], &Qw[i][k] );
            }

            /* Compute from the right */

            for ( i = p - 2; i >= r + 1; i-- )
            {
                N_Combine2CPts( bet[i], Pw[i + 1][k], omb[i], Qw[i + 1][k], &Qw[i][k] );
            }

            /* Compute the error */

            u = (r + 1.0) / p;

            error = N_BezEvalOneBasis( p, r + 1, u, &B1 );

            if( error EQ NL_YES )
                NL_OUT;

            N_Combine2CPts( 0.5, Qw[r][k], 0.5, Qw[r + 1][k], &PL );
            N_DistCptCptHomo( Pw[r + 1][k], PL, &dw );

            *e = B1 * dw;
        }
    } /* End of NL_UDIR */

    /* Reduce in v-direction */

    if( dir EQ NL_VDIR )
    {
        /* Get end points */

        N_CopyCPt( Pw[k][0], &Qw[k][0] );
        N_CopyCPt( Pw[k][q], &Qw[k][q - 1] );

        /* Reduce the degree */

        r = (q - 1) / 2;

        if( q % 2 ) /* Odd degree */
        {
            /* Compute from the left */

            for ( j = 1; j <= r - 1; j++ )
            {
                N_Combine2CPts( alf[j], Pw[k][j], oma[j], Qw[k][j - 1], &Qw[k][j] );
            }

            /* Compute from the right */

            for ( j = q - 2; j >= r + 1; j-- )
            {
                N_Combine2CPts( bet[j], Pw[k][j + 1], omb[j], Qw[k][j + 1], &Qw[k][j] );
            }

            /* Compute middle control point */

            N_Combine2CPts( alf[r], Pw[k][r], oma[r], Qw[k][r - 1], &PL );
            N_Combine2CPts( bet[r], Pw[k][r + 1], omb[r], Qw[k][r + 1], &PR );
            N_Combine2CPts( 0.5, PL, 0.5, PR, &Qw[k][r] );

            /* Compute the error */

            u = 0.5 *( 1.0 - sqrt( 1.0 / q ) );

            error = N_BezEvalOneBasis( q, r, u, &B );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BezEvalOneBasis( q, r + 1, u, &B1 );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistCptCptHomo( PL, PR, &dw );

            a = 0.5 *( (NL_REAL)q - (NL_REAL)r ) / (NL_REAL)q;
            b = fabs( B - B1 );
            *e = a * b * dw;
        }
        else /* Even degree */
        {
            /* Compute from the left */

            for ( j = 1; j <= r; j++ )
            {
                N_Combine2CPts( alf[j], Pw[k][j], oma[j], Qw[k][j - 1], &Qw[k][j] );
            }

            /* Compute from the right */

            for ( j = q - 2; j >= r + 1; j-- )
            {
                N_Combine2CPts( bet[j], Pw[k][j + 1], omb[j], Qw[k][j + 1], &Qw[k][j] );
            }

            /* Compute the error */

            u = (r + 1.0) / q;

            error = N_BezEvalOneBasis( q, r + 1, u, &B1 );

            if( error EQ NL_YES )
                NL_OUT;

            N_Combine2CPts( 0.5, Qw[k][r], 0.5, Qw[k][r + 1], &PL );
            N_DistCptCptHomo( Pw[k][r + 1], PL, &dw );

            *e = B1 * dw;
        }
    } /* End of NL_VDIR */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezSrfReduceDegree */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes a conic arc given by the end points, 
     the end tangents and one point along the arc. A  typical  calling 
     example is as follows:

       NL_POINT   P0, P1, P2, P;
       NL_VECTOR  T0, T2;
       NL_REAL    w1;
       ...
       (define P0, T0 and P2, T2, and get P)
       ... 
       N_BezConicArcFromPtsAndTangents(P0,T0,P2,T2,P,&P1,&w1);

     The middle control point along with the  corresponding weight are
     returned.


   ACCESS:
   
     P0,T0  , input  ,  Start point and tangent
     P2,T2  , input  ,  End point and tangent
     P      , input  ,  Point on the arc
     P1     , output ,  Middle control point
     w1     , output ,  Middle weight


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezConicArcFromPtsAndTangents( NL_POINT P0, NL_VECTOR T0, NL_POINT P2, NL_VECTOR T2, NL_POINT P, NL_POINT *P1, NL_REAL *w1 )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezConicArcFromPtsAndTangents");

    NL_FLAG ints, error = NL_NO;

    NL_REAL t0, t2, a, b, c, alf, bet, gam, u, num, den;

    NL_POINT M;

    NL_VECTOR V0, V1, V2;

    NL_LINESEG l0, l2;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get middle control point */

    N_CreateLineStartDirVector( &l0, P0, T0, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &l2, P2, T2, NL_UNBOUNDED );

    error = N_IsectLineLine( l0, l2, P1, &t0, &t2, &ints );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute middle weight */

    if( ints EQ NL_TRUE )
    {
        N_CreateLinePtPt( &l0, P0, P2, NL_BOUNDED );
        N_CreateLinePtPt( &l2, *P1, P, NL_UNBOUNDED );

        error = N_IsectLineLine( l0, l2, &M, &t0, &t2, &ints );

        if( error EQ NL_YES )
            NL_OUT;

        if( ints EQ NL_FALSE )
            NL_ERROR( NL_CAL_ERR );

        if( N_FloatOpIsBad( t0, 1.0 - t0, NL_DIVISION ) )
            NL_ERROR( NL_CAL_ERR );

        a = sqrt( t0 / (1.0 - t0) );
        u = a / (1.0 + a);

        N_VectorDiff( P, P0, &V0 );
        N_VectorDiff( *P1, P, &V1 );
        N_VectorDiff( P, P2, &V2 );

        N_VectorDot( V0, V1, &alf );
        N_VectorDot( V1, V2, &bet );
        N_VectorDot( V1, V1, &gam );

        a = (1.0 - u) * (1.0 - u);
        b = u * u;
        c = 2.0 *u * (1.0 - u);

        num = a * alf + b * bet;
        den = c * gam;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        *w1 = num / den;
    }

    /* Compute direction vector */

    if( ints EQ NL_FALSE )
    {
        N_CreateLineStartDirVector( &l0, P, T0, NL_UNBOUNDED );
        N_CreateLinePtPt( &l2, P0, P2, NL_BOUNDED );

        error = N_IsectLineLine( l0, l2, &M, &t0, &t2, &ints );

        if( error EQ NL_YES )
            NL_OUT;

        if( ints EQ NL_FALSE )
            NL_ERROR( NL_CAL_ERR );

        if( N_FloatOpIsBad( t2, 1.0 - t2, NL_DIVISION ) )
            NL_ERROR( NL_CAL_ERR );

        a = sqrt( t2 / (1.0 - t2) );
        u = a / (1.0 + a);
        b = 2.0 *u * (1.0 - u);

        num = -t0 * (1.0 - b);

        if( N_FloatOpIsBad( num, b, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        b = num / b;
        *w1 = 0.0;

        N_VectorScale( T0, b, P1 );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezConicArcFromPtsAndTangents */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes a circular arc given the end points
     and end tangents of the arc. The circular arc is  represented by
     1, 2 or 4 NURBS circle segments  depending on the sweep angle. A
     typical calling example is:

       NL_POINT   P1, P2;
       NL_VECTOR  T1, T2;
       NL_CPOINT  Pw[9];
       NL_INDEX   n;
       ...
       (get P1, T1, and P2, T2);
       ...
       N_BezCircArcFromPtsAndTangents(P1,T1,P2,T2,Pw,&n);

     IT IS ASSUMED THAT THE NL_END  TANGENTS ARE UNIT  TANGENTS AND THEY
     MAKE EQUAL ANGLES WITH THE CHORD JOINING THE NL_END NL_POINTS.


   ACCESS:
   
     P1,T1 , input  ,  Start point and UNIT tangent
     P2,T2 , input  ,  End point and UNIT tangent
     Pw    , output ,  Control points of NURBS circular arc
     n     , output ,  Highest index in Pw


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCircArcFromPtsAndTangents( NL_POINT P1, NL_VECTOR T1, NL_POINT P2, NL_VECTOR T2, NL_CPOINT *Pw, NL_INDEX *n )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezCircArcFromPtsAndTangents");

    NL_FLAG its, error = NL_NO;

    NL_INDEX k;

    NL_REAL t1, t2, d, dot1, dot2, dot, r, w, cw, al, ar, bet, omb;

    NL_POINT R, A, B, C;

    NL_VECTOR V;

    NL_LINESEG l1, l2;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize and compute some parameters */

    cw = 0.5 *sqrt( 2.0 );
    k = 0;

    N_Weight( P1, 1.0, &Pw[0] );

    N_VectorDiff( P2, P1, &V );

    error = N_VectorNormalize( V, &V, &d );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get special cases */

    N_CreateLineStartDirVector( &l1, P1, T1, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &l2, P2, T2, NL_UNBOUNDED );

    error = N_IsectLineLine( l1, l2, &R, &t1, &t2, &its );

    if( error EQ NL_YES )
        NL_OUT;

    if( its EQ NL_FALSE )
    {
        N_VectorDot( T1, V, &dot1 );
        N_VectorDot( T2, V, &dot2 );

        /* Collinear tangents */

        if( fabs( dot1 - 1.0 )LT NL_PTOL AND fabs( dot2 - 1.0 )LT NL_PTOL )
        {
            N_Combine2Pts( 0.5, P1, 0.5, P2, &C );
            N_Weight( C, 1.0, &Pw[++k] );
            N_Weight( P2, 1.0, &Pw[++k] );

            NL_OUT;
        }

        /* Tangents are perpendicular to the chord */

        if( fabs( dot1 )LT NL_PTOL AND fabs( dot2 )LT NL_PTOL )
        {
            N_Combine2Pts( 0.5, P1, 0.5, P2, &C );
            N_VectorDot( T1, T2, &dot );

            if( dot LT 0.0 ) /* Tangents are anti-parallel */
            {
                r = 0.5 *d;

                N_VectorPtAlongVector( P1, r, T1, &A );
                N_Weight( A, cw, &Pw[++k] );
                N_VectorPtAlongVector( C, r, T1, &A );
                N_Weight( A, 1.0, &Pw[++k] );
                N_VectorPtAlongVector( P2, r, T1, &A );
                N_Weight( A, cw, &Pw[++k] );
                N_Weight( P2, 1.0, &Pw[++k] );

                NL_OUT;
            }
            else /* Tangents are parallel */
            {
                r = 0.25 *d;

                N_Combine2Pts( 0.5, P1, 0.5, C, &B );
                N_VectorPtAlongVector( P1, r, T1, &A );
                N_Weight( A, cw, &Pw[++k] );
                N_VectorPtAlongVector( B, r, T1, &A );
                N_Weight( A, 1.0, &Pw[++k] );
                N_VectorPtAlongVector( C, r, T1, &A );
                N_Weight( A, cw, &Pw[++k] );
                N_Weight( C, 1.0, &Pw[++k] );

                N_Combine2Pts( 0.5, C, 0.5, P2, &B );
                N_VectorPtAlongVector( C, -r, T2, &A );
                N_Weight( A, cw, &Pw[++k] );
                N_VectorPtAlongVector( B, -r, T2, &A );
                N_Weight( A, 1.0, &Pw[++k] );
                N_VectorPtAlongVector( P2, -r, T2, &A );
                N_Weight( A, cw, &Pw[++k] );
                N_Weight( P2, 1.0, &Pw[++k] );

                NL_OUT;
            }
        }

        /* Anti-collinear or parallel tangents */

        NL_ERROR( NL_CAL_ERR );
    }

    if( t1 * t2 GT 0.0 )
        NL_ERROR( NL_INP_ERR );

    if( fabs( t1 )LT NL_PTOL OR fabs( t2 )LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    /* Compute the circle */

    al = 1.0 / fabs( t1 );
    ar = 1.0 / fabs( t2 );
    w = 0.25 *d * (al + ar);

    if( t1 GT 0.0 ) /* One arc */
    {
        N_Weight( R, w, &Pw[++k] );
        N_Weight( P2, 1.0, &Pw[++k] );

        NL_OUT;
    }
    else /* Two arcs */
    {
        w = -w;
        bet = w / (1.0 + w);
        omb = 1.0 - bet;

        N_Combine2Pts( omb, P1, bet, R, &A );
        N_Combine2Pts( bet, R, omb, P2, &B );
        N_Combine2Pts( 0.5, A, 0.5, B, &C );

        w = 0.5 *( 1.0 + w );

        if( w LT 0.0 )
            NL_ERROR( NL_INP_ERR );

        w = sqrt( w );

        N_Weight( A, w, &Pw[++k] );
        N_Weight( C, 1.0, &Pw[++k] );
        N_Weight( B, w, &Pw[++k] );
        N_Weight( P2, 1.0, &Pw[++k] );

        NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    *n = k;

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezCircArcFromPtsAndTangents */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine computes the middle weight of a conic arc so 
     that the resultant conic is a good approximation to the  circular 
     arc defined by the same control triangle. If the control triangle 
     happens to be isosceles, the precise circle weight is returned. A
     typical calling example is:

       NL_POINT  P0, P1, P2;
       NL_REAL   w;
       ...
       (get P0, P1 and P2);
       ...
       N_BezSetConicWeight(P0,P1,P2,&w);


   ACCESS:
   
     P0,P1,P2 , input  ,  Vertices of control triangle
     w        , output ,  Weight at P1


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BezSetConicWeight( NL_POINT P0, NL_POINT P1, NL_POINT P2, NL_REAL *w )
{
    NL_REAL d, fl, fr, sl, sr, s;

    N_DistPtPt( P0, P2, &d );
    N_DistPtPt( P0, P1, &fl );
    N_DistPtPt( P2, P1, &fr );

    if( d * fl * fr == 0.0 )
        *w = 1.0;
    else
    {
        d *= 0.5;
        sl = d / (d + fl);
        sr = d / (d + fr);
        s = 0.5 *( sl + sr );

        *w = s / (1.0 - s);
    }
} /* end N_BezSetConicWeight */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine splits a conic arc at its shoulder point. The
     arc is  given by  its Bezier control  points  and  weights. IT IS 
     ASSUMED THAT  THE  CONTROL  NL_POINTS ARE  NOT WEIGHTED  AND THE NL_END 
     WEIGHTS ARE SET TO 1. A typical calling example is as follows:

       NL_POINT  P[3], Q[3], R[3];
       NL_REAL   wp, wq, wr;
       ...
       (define P[0], P[1], P[2] and wp);
       ... 
       N_BezConicSplit(P,wp,Q,&wq,R,&wr);

     The split curves are  returned in  the normal form, i.e. the  end 
     weights are set to 1.


   ACCESS:
   
     P  , input  ,  Control points of Bezier conic
     wp , input  ,  Middle weight
     Q  , output ,  Control points of left half
     wq , output ,  Middle weight of segment Q
     R  , output ,  Control points of right half
     wr , output ,  Middle weight of segment R


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_VOID N_BezConicSplit( NL_POINT *P, NL_REAL wp, NL_POINT *Q, NL_REAL *wq, NL_POINT *R, NL_REAL *wr )
{
    NL_REAL alf, oma;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get end points */

    N_CopyPt( P[0], &Q[0] );
    N_CopyPt( P[2], &R[2] );

    /* Split segment defined by finite control points */

    if( wp NEQ 0.0 )
    {
        alf = 1.0 / (1.0 + wp);
        oma = 1.0 - alf;

        N_Combine2Pts( alf, P[0], oma, P[1], &Q[1] );
        N_Combine2Pts( alf, P[2], oma, P[1], &R[1] );
        N_Combine2Pts( 0.5, Q[1], 0.5, R[1], &Q[2] );
        N_CopyPt( Q[2], &R[0] );

        *wq = sqrt( 0.5 *( 1.0 + wp ) );
        *wr = *wq;
    }

    /* Split semi-elliptical segment */

    if( wp EQ 0.0 )
    {
        N_Combine2Pts( 1.0, P[0], 1.0, P[1], &Q[1] );
        N_Combine2Pts( 1.0, P[2], 1.0, P[1], &R[1] );
        N_Combine2Pts( 0.5, Q[1], 0.5, R[1], &Q[2] );
        N_CopyPt( Q[2], &R[0] );

        *wq = 0.5 *sqrt( 2.0 );
        *wr = *wq;
    }

    /* End NURBS */

    N_EndNurbs( &S );
} /* end N_BezConicSplit */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine computes  the NON-NL_ZERO elements of the power
     basis conversion matrix. If  the output matrix is  initialized to 
     the NULL  matrix, memory  to store the  elements is  allocated. A 
     typical calling example is as follows:

       NL_DEGREE   p;
       NL_RMATRIX  pom;
       NL_STACKS   SG;
       ...
       N_InitRealMatrix(&pom);
       N_BezGetPowerConversionMatrix(p,&pom,&SG);

     If memory is available, pom is not  initialized and  the  routine
     assumes that memory allocation has  been done. However, it checks  
     for the proper amount by looking at  the highest indexes in pom's 
     structure. THE MATRIX IS LOWER NL_LEFT TRIANGULAR.


   ACCESS:
   
     p   , input  ,  Bezier curve degree
     pom , output ,  Power basis conversion matrix
     SG  , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezGetPowerConversionMatrix( NL_DEGREE p, NL_RMATRIX *pom, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezGetPowerConversionMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, sign;

    /*        NL_INTEGER  **bin;    */

    NL_REAL ** RM;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check degree */

    if( p LT 0 )
        NL_ERROR( NL_CAL_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( pom, p, p, NL_MT_LOWERLEFT, p, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( pom, &RM );

    /* Get binomial coefficients */

    /*        bin = N_AllocInt2dArray(p,p,&SL);    */
    /*        if( bin EQ NULL )  NL_QUIT;    */
    /*                                    */
    /*        N_PascalTriRow(bin,p);            */

    /******************/
    /* Compute matrix */
    /******************/

    /* Ser corner elements */

    RM[0][0] = 1.0;
    RM[p][p] = 1.0;

    if( p % 2 )
        RM[p][0] = -1.0;
    else
        RM[p][0] = 1.0;

    /* Set elements in first column, last row and along the diagonal */

    sign = -1;

    for ( i = 1; i < p; i++ )
    {
        RM[i][i] = NL_PascalTri[p][i];
        RM[i][0] = sign * RM[i][i];
        RM[p][p - i] = RM[i][0];
        sign = -sign;
    }

    /* Compute remaning elements */

    k = (p + 1) / 2;
    l = p - 1;

    for ( j = 1; j < k; j++ )
    {
        sign = -1;

        for ( i = j + 1; i <= l; i++ )
        {
            RM[i][j] = (NL_REAL)sign * (NL_REAL)NL_PascalTri[p][j] * (NL_REAL)NL_PascalTri[p - j][i - j];
            RM[l][p - i] = RM[i][j];
            sign = -sign;
        }
        l--;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezGetPowerConversionMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine computes  the  NON-NL_ZERO  elements of the re-
     parametrization  matrix.  The  matrix  reparametrizes  the  power 
     basis  curve, defined  over [a,b], to be defined over [ap,bp]. If  
     the output matrix is  initialized  to the NULL matrix, memory  to 
     store the elements is allocated. A typical calling  example is as 
     follows:

       NL_DEGREE     p;
       NL_PARAMETER  a, b, ap, bp;
       NL_RMATRIX    rem;
       NL_STACKS     SG;
       ...
       N_InitRealMatrix(&rem);
       N_BezGetReparamMatrix(p,a,b,ap,bp,&rem,&SG);

     If memory is available, rem is not  initialized and  the  routine
     assumes that memory allocation has  been done. However, it checks  
     for the proper amount by looking at  the highest indexes in rem's 
     structure. THE MATRIX IS UPPER NL_RIGHT TRIANGULAR!


   ACCESS:
   
     p     , input  ,  Curve degree
     a,b   , inpit  ,  Parameters  over  which  original   segment  is 
                       defined
     ap,bp , input  ,  Parameters  over which  reparametrized curve is
                       defined
     rem   , output ,  Reparametrization matrix
     SG    , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezGetReparamMatrix( NL_DEGREE p, NL_PARAMETER a, NL_PARAMETER b, NL_PARAMETER ap, NL_PARAMETER bp, NL_RMATRIX *rem, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezGetReparamMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    /*        NL_INTEGER    **bin;    */

    NL_REAL ** RM, fact;

    NL_PARAMETER c, d;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check degree and parameters */

    if( p LT 0 )
        NL_ERROR( NL_CAL_ERR );

    if( b - a LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rem, p, p, NL_MT_UPPERRIGHT, p, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rem, &RM );

    /* Get binomial coefficients */

    /*        bin = N_AllocInt2dArray(p,p,&SL);    */
    /*        if( bin EQ NULL )  NL_QUIT;    */
    /*                                    */
    /*        N_PascalTriRow(bin,p);            */

    /******************/
    /* Compute matrix */
    /******************/

    c = (bp - ap) / (b - a);
    d = (ap * b - bp * a) / (b - a);

    /* Set elements in first row and along the diagonal */

    RM[0][0] = 1.0;

    for ( i = 1; i <= p; i++ )
    {
        RM[0][i] = d * RM[0][i - 1];
        RM[i][i] = c * RM[i - 1][i - 1];
    }

    /* Compute remaning elements */

    for ( i = 1; i < p; i++ )
    {
        fact = RM[i][i];

        for ( j = i + 1; j <= p; j++ )
        {
            fact = fact * d;
            RM[i][j] = NL_PascalTri[j][i] * fact;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezGetReparamMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the NON-NL_ZERO elements of the inverse 
     of the power  basis  conversion  matrix. If  the output matrix is  
     initialized to the NULL matrix, memory  to store the  elements is  
     allocated. A typical calling example is as follows:

       NL_DEGREE   p;
       NL_RMATRIX  ipm;
       NL_STACKS   SG;
       ...
       N_InitRealMatrix(&ipm);
       N_BezInversePowerMatrix(p,&ipm,&SG);

     If memory is available, ipm is not  initialized and  the  routine
     assumes that memory allocation has  been done. However, it checks  
     for the proper amount by looking at  the highest indexes in ipm's 
     structure. THE MATRIX IS LOWER NL_LEFT TRIANGULAR.


   ACCESS:
   
     p   , input  ,  Curve degree
     ipm , output ,  Inverse of power basis conversion matrix
     SG  , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezInversePowerMatrix( NL_DEGREE p, NL_RMATRIX *ipm, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezInversePowerMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, s;

    NL_REAL ** PM, ** IM, sum;

    NL_RMATRIX pom;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check degree */

    if( p LT 0 )
        NL_ERROR( NL_CAL_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( ipm, p, p, NL_MT_LOWERLEFT, p, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( ipm, &IM );

    /* Get power basis conversion matrix */

    N_InitRealMatrix( &pom );
    error = N_BezGetPowerConversionMatrix( p, &pom, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &pom, &PM );

    /******************/
    /* Compute matrix */
    /******************/

    /* Set elements in first column, last row and along the diagonal */

    for ( i = 0; i <= p; i++ )
    {
        IM[i][0] = 1.0;
        IM[p][i] = 1.0;
        IM[i][i] = 1.0 / PM[i][i];
    }

    /* Compute remaning elements */

    k = (p + 1) / 2;
    l = p - 1;

    for ( j = 1; j < k; j++ )
    {
        for ( i = j + 1; i <= l; i++ )
        {
            sum = 0.0;

            for ( s = j; s < i; s++ )
                sum -= PM[i][s] * IM[s][j];

            IM[i][j] = sum / PM[i][i];
            IM[l][p - i] = IM[i][j];
        }
        l--;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezInversePowerMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the NON-NL_ZERO elements of the inverse
     of  the  reparametrization  matrix (the  reparametrization matrix 
     reparametrizes  the  power basis  curve, defined over [a,b] to be  
     defined over [ap,bp]). If the  output matrix  is  initialized  to 
     the NULL  matrix,  memory to  store the  elements is allocated. A 
     typical calling example is as follows:

       NL_DEGREE     p;
       NL_PARAMETER  a, b, ap, bp;
       NL_RMATRIX    irm;
       NL_STACKS     SG;
       ...
       N_InitRealMatrix(&irm);
       N_BezInverseReparamMatrix(p,a,b,ap,bp,&irm,&SG);

     If memory is available, irm is not  initialized and  the  routine
     assumes that memory allocation has  been done. However, it checks  
     for the proper amount by looking at  the highest indexes in irm's 
     structure. THE MATRIX IS UPPER NL_RIGHT TRIANGULAR!


   ACCESS:
   
     p     , input  ,  Curve degree
     a,b   , inpit  ,  Parameters   over  which  original  segment  is 
                       defined
     ap,bp , input  ,  Parameters over which reparametrized  curve  is 
                       defined
     irm   , output ,  Inverse of reparametrization matrix
     SG    , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezInverseReparamMatrix( NL_DEGREE p, NL_PARAMETER a, NL_PARAMETER b, NL_PARAMETER ap, NL_PARAMETER bp, NL_RMATRIX *irm, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezInverseReparamMatrix");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    /*        NL_INTEGER    **bin;    */

    NL_REAL ** RM, fact;

    NL_PARAMETER c, d;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check degree and parameters */

    if( p LT 0 )
        NL_ERROR( NL_CAL_ERR );

    if( bp - ap LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( irm, p, p, NL_MT_UPPERRIGHT, p, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( irm, &RM );

    /* Get binomial coefficients */

    /*        bin = N_AllocInt2dArray(p,p,&SL);    */
    /*        if( bin EQ NULL )  NL_QUIT;    */
    /*                                    */
    /*        N_PascalTriRow(bin,p);            */

    /******************/
    /* Compute matrix */
    /******************/

    c = (b - a) / (bp - ap);
    d = (bp * a - ap * b) / (bp - ap);
    ;

    /* Set elements in first row and along the diagonal */

    RM[0][0] = 1.0;

    for ( i = 1; i <= p; i++ )
    {
        RM[0][i] = d * RM[0][i - 1];
        RM[i][i] = c * RM[i - 1][i - 1];
    }

    /* Compute remaning elements */

    for ( i = 1; i < p; i++ )
    {
        fact = RM[i][i];

        for ( j = i + 1; j <= p; j++ )
        {
            fact = fact * d;
            RM[i][j] = NL_PascalTri[j][i] * fact;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezInverseReparamMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier routine  converts a Bezier segment  into power basis 
     form. To  facilitate with NURBS  conversion, the power basis con-
     version matrix can be passed in. If it is initialized to the NULL
     matrix, it is  computed locally. Otherwise  it is assumed that it 
     has been precomputed and passed in. A typical calling example is: 

       NL_CPOINT     *Pw, *bw;
       NL_DEGREE     p;
       NL_RMATRIX    pom;
       NL_PARAMETER  a, b;
       NL_STACKS     SG;
       ...
       (get Pw, p, a, b, and allocate memory for bw);
       ...
       N_InitRealMatrix(&pom);
       N_BezToPowerBasis(Pw,p,&pom,a,b,bw);

     IT IS  ASSUMED THAT  MEMORY FOR bw IS  ALLOCATED IN  THE CALLING
     ROUTINE.  IT IS  ALSO  ASSUMED THAT MEMORY TO  STORE THE  MATRIX 
     STRUCTURE pom IS ALSO ALLOCATED IN THE CALLING ROUTINE (I.E. THE
     MATRIX  OBJECT IS  DEFINED  IN THE  CALLING  ROUTINE). THE POWER 
     BASIS NL_CURVE IS DEFINED NL_OVER [a,b].


   ACCESS:
   
     Pw  , input  ,  Bezier curve control points
     p   , input  ,  Bezier curve degree
     pom , input  ,  Power basis conversion matrix
     a,b , input  ,  Parameter range over which curve is defined
     bw  , output ,  Power basis coefficients
     SG  , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezToPowerBasis( NL_CPOINT *Pw, NL_DEGREE p, NL_RMATRIX *pom, NL_PARAMETER a, NL_PARAMETER b, NL_CPOINT *bw )
{
    NL_FLAG error = NL_NO;

    NL_RMATRIX rem, com;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get reparametrization matrix */

    N_InitRealMatrix( &rem );
    error = N_BezGetReparamMatrix( p, a, b, 0.0, 1.0, &rem, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get power basis conversion matrix */

    if( N_RealMatrixIsNULL( pom ) )
    {
        error = N_BezGetPowerConversionMatrix( p, pom, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get conversion matrix */

    N_InitRealMatrix( &com );
    error = N_RealMatrixMultiply( &rem, pom, &com, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get power basis coefficients */

    error = N_RealMatrixMultiplyCPtArray( &com, Pw, bw );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezToPowerBasis */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine  converts  a power  basis curve  into Bezier 
     form. To  facilitate  with NURBS  conversion, the  inverse of the
     power basis conversion  matrix can be passed in. If it is initia-
     lized to the NULL matrix, it is computed locally. Otherwise it is 
     assumed that  it has been  precomputed and  passed in. A  typical 
     calling example is as follows:

       NL_CPOINT     *bw, *Pw;
       NL_DEGREE     p;
       NL_RMATRIX    ipm;
       NL_PARAMETER  a, b;
       NL_STACKS     SG;
       ...
       (get bw, p, a, b, and allocate memory for Pw);
       ...
       N_InitRealMatrix(&ipm);
       N_PowerBasisToBez(bw,p,&ipm,a,b,Pw);

     IT IS  ASSUMED THAT  MEMORY FOR Pw IS  ALLOCATED IN  THE CALLING
     ROUTINE.  IT IS  ALSO  ASSUMED THAT MEMORY TO  STORE THE  MATRIX 
     STRUCTURE ipm IS ALSO ALLOCATED IN THE CALLING ROUTINE (I.E. THE
     MATRIX  OBJECT IS  DEFINED IN THE  CALLING  ROUTINE). THE BEZIER
     NL_CURVE IS DEFINED NL_OVER [a,b].


   ACCESS:
   
     bw  , input  ,  Power basis coefficients
     p   , input  ,  Power basis curve degree
     ipm , input  ,  Inverse of power basis conversion matrix
     a,b , input  ,  Parameter range over which curve is defined
     Pw  , output ,  Bezier curve control points
     SG  , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PowerBasisToBez( NL_CPOINT *bw, NL_DEGREE p, NL_RMATRIX *ipm, NL_PARAMETER a, NL_PARAMETER b, NL_CPOINT *Pw )
{
    NL_FLAG error = NL_NO;

    NL_RMATRIX irm, com;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get inverse of power basis conversion matrix */

    if( N_RealMatrixIsNULL( ipm ) )
    {
        error = N_BezInversePowerMatrix( p, ipm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get inverse of reparametrization matrix */

    N_InitRealMatrix( &irm );
    error = N_BezInverseReparamMatrix( p, a, b, 0.0, 1.0, &irm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get conversion matrix */

    N_InitRealMatrix( &com );
    error = N_RealMatrixMultiply( ipm, &irm, &com, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier control points */

    error = N_RealMatrixMultiplyCPtArray( &com, bw, Pw );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_PowerBasisToBez */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine converts  a Bezier  patch into  power basis 
     form. To  facilitate with NURBS  conversion, the power basis con-
     version matrices can be passed in. If they are initialized to the 
     NULL matrices, they are computed locally. Otherwise it is assumed 
     that they have been precomputed and passed in. A  typical calling 
     example is as follows:

       NL_CPOINT     **Pw, **bw;
       NL_DEGREE     p, q;
       NL_RMATRIX    pum, pvm;
       NL_PARAMETER  a, b, c, d;
       NL_STACKS     SG;
       ...
       (get Pw, p, q, a, b, c, d and allocate memory for bw);
       ...
       N_InitRealMatrix(&pum);
       N_InitRealMatrix(&pvm);
       N_BezSrfToPower(Pw,p,q,&pum,&pvm,a,b,c,d,bw);

     IT IS  ASSUMED THAT  MEMORY FOR bw IS  ALLOCATED IN  THE CALLING
     ROUTINE.  IT IS  ALSO  ASSUMED THAT MEMORY TO  STORE THE  MATRIX 
     STRUCTURES pum AND pvm IS  ALSO ALLOCATED IN THE CALLING ROUTINE 
     (I.E. THE MATRIX OBJECTS ARE  DEFINED  IN THE CALLING  ROUTINE). 
     THE POWER BASIS NL_SURFACE IS DEFINED NL_OVER [a,b] x [c,d].


   ACCESS:
   
     Pw      , input  ,  Bezier surface control points
     p,q     , input  ,  Bezier surface degrees
     pum,pvm , input  ,  Power basis conversion  matrices (in  u- and v-directions)
     a,b,c,d , input  ,  Bezier surface is defined over [a,b] x [c,d]
     bw      , output ,  Power basis coefficients
     SG      , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfToPower
  ( NL_CPOINT ** Pw,   /* in : Bezier surface control points                       */
    NL_DEGREE p,       /* in : Bezier surface degrees                              */
    NL_DEGREE q,       /* in : Bezier surface degrees                              */
    NL_RMATRIX *pum,   /* in : Power basis conversion  matrices (in  u-directions) */
    NL_RMATRIX *pvm,   /* in : Power basis conversion  matrices (in  v-directions) */
    NL_PARAMETER a,    /* in : a of Bezier surface definition over [a,b] x [c,d]   */
    NL_PARAMETER b,    /* in : b of Bezier surface definition over [a,b] x [c,d]   */
    NL_PARAMETER c,    /* in : c of Bezier surface definition over [a,b] x [c,d]   */
    NL_PARAMETER d,    /* in : d of Bezier surface definition over [a,b] x [c,d]   */
    NL_CPOINT ** bw)   /* out: Power basis coefficients                            */
{
    NL_FLAG error = NL_NO;

    NL_RMATRIX rum, rvm, cum, cvm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get reparametrization matrices */

    N_InitRealMatrix( &rum );
    error = N_BezGetReparamMatrix( p, a, b, 0.0, 1.0, &rum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &rvm );
    error = N_BezGetReparamMatrix( q, c, d, 0.0, 1.0, &rvm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get power basis conversion matrices */

    if( N_RealMatrixIsNULL( pum ) )
    {
        error = N_BezGetPowerConversionMatrix( p, pum, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( N_RealMatrixIsNULL( pvm ) )
    {
        error = N_BezGetPowerConversionMatrix( q, pvm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get conversion matrices */

    N_InitRealMatrix( &cum );
    error = N_RealMatrixMultiply( &rum, pum, &cum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &cvm );
    error = N_RealMatrixMultiply( &rvm, pvm, &cvm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get power basis coefficients */

    error = N_RealMatrixMultiplyRealMatrixTranspose( &cum, Pw, &cvm, bw );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfToPower */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier  routine  converts a  power basis  patch into Bezier
     form. To  facilitate  with  NURBS  conversion, the inverse of the 
     power  basis  conversion  matrices  can be passed in. If they are 
     initialized to  the NULL  matrices,  they are  computed  locally. 
     Otherwise  it is  assumed that  they have  been  precomputed  and 
     passed in. A typical calling example is as follows:

       NL_CPOINT     **bw, **Pw;
       NL_DEGREE     p, q;
       NL_RMATRIX    ipu, ipv;
       NL_PARAMETER  a, b, c, d;
       NL_STACKS     SG;
       ...
       (get bw, p, q, a, b, c, d and allocate memory for Pw);
       ...
       N_InitRealMatrix(&ipu);
       N_InitRealMatrix(&ipv);
       N_BezSrfPowerToBez(bw,p,q,&ipu,&ipv,a,b,c,d,Pw);

     IT IS  ASSUMED THAT  MEMORY FOR Pw IS  ALLOCATED IN  THE CALLING
     ROUTINE.  IT IS  ALSO  ASSUMED THAT MEMORY TO  STORE THE  MATRIX 
     STRUCTURES ipu AND ipv IS  ALSO ALLOCATED IN THE CALLING ROUTINE 
     (I.E. THE MATRIX OBJECTS ARE  DEFINED  IN THE CALLING  ROUTINE). 
     THE BEZIER NL_SURFACE IS DEFINED NL_OVER [a,b] x [c,d].


   ACCESS:
   
     bw      , input  ,  Power basis coefficients
     p,q     , input  ,  Power basis surface degrees
     ipu,ipv , input  ,  Inverse of power  basis conversion  matrices 
                         (in  u- and and v-directions)
     a,b,c,d , input  ,  Power   basis   surface   is   defined  over 
                         [a,b] x [c,d]
     Pw      , output ,  Bezier surface control points
     SG      , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfPowerToBez( NL_CPOINT ** bw, NL_DEGREE p, NL_DEGREE q, NL_RMATRIX *ipu, NL_RMATRIX *ipv, NL_PARAMETER a, NL_PARAMETER b, NL_PARAMETER c, NL_PARAMETER d, NL_CPOINT ** Pw )
{
    NL_FLAG error = NL_NO;

    NL_RMATRIX iru, irv, cum, cvm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get inverse of power basis conversion matrices */

    if( N_RealMatrixIsNULL( ipu ) )
    {
        error = N_BezInversePowerMatrix( p, ipu, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( N_RealMatrixIsNULL( ipv ) )
    {
        error = N_BezInversePowerMatrix( q, ipv, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get inverse of reparametrization matrices */

    N_InitRealMatrix( &iru );
    error = N_BezInverseReparamMatrix( p, a, b, 0.0, 1.0, &iru, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &irv );
    error = N_BezInverseReparamMatrix( q, c, d, 0.0, 1.0, &irv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get conversion matrices */

    N_InitRealMatrix( &cum );
    error = N_RealMatrixMultiply( ipu, &iru, &cum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &cvm );
    error = N_RealMatrixMultiply( ipv, &irv, &cvm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier control points */

    error = N_RealMatrixMultiplyRealMatrixTranspose( &cum, bw, &cvm, Pw );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfPowerToBez */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine  reparametrizes a  Bezier curve of any degree 
     with a Bezier function of any degree. The Bezier function must be
     monotonic in  order  for the  reparametrization to  make sense. A
     typical calling example is as follows:

       NL_CPOINT   *Pw, *Qw;
       NL_REAL     *f;
       NL_DEGREE   p, q;

       ...

       N_BezReparam(Pw,p,f,q,NULL,Qw);

     The binomial coefficients, stored in the  Pascal triangle, can be
     passed in. If NULL is passed  in, the triangle is computed inside
     the routine. IT IS ASSUMED THAT SUFFICIENT MEMORY IS AVAILABLE TO  
     STORE THE NL_NEW CONTROL NL_POINTS "Qw".


   ACCESS:
   
     Pw  , input  ,  Original control points
     p   , input  ,  Original degree
     f   , input  ,  Bezier function coefficients
     q   , input  ,  Degree of Bezier function
     bin , input  ,  Binomial coefficient array pointer:
                     No longer computed locally - set to NULL.
     Qw  , output ,  New control points after  reparametrization. MUST 
                     HOLD AT LEAST p*q+1 CONTROL NL_POINTS!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezReparam( NL_CPOINT *Pw, NL_DEGREE p, NL_REAL *f, NL_DEGREE q, NL_INTEGER ** bin, NL_CPOINT *Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezReparam");

    NL_FLAG error = NL_NO, uarray = NL_NO;

    NL_INDEX i, j, r, s, jmin, jmax, qs, qs1, pq;

    NL_REAL alf, oma, fac;

    NL_CPOINT ** Uw, ** Vw, Aw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Check degree */

    pq = p * q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* See if Pascal triangle is needed */

    if( bin EQ NULL )
    {
        bin = N_AllocInt2dArray( pq, pq, &S );

        if( bin EQ NULL )
            NL_QUIT;

        N_PascalTriRow( bin, pq );
    }

    /* Compute new control points */

    Uw = N_AllocCPt2dArray( p, pq, &S );

    if( Uw EQ NULL )
        NL_QUIT;

    Vw = N_AllocCPt2dArray( p, pq, &S );

    if( Vw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p; i++ )
    {
        N_CopyCPt( Pw[i], &Uw[i][0] );
    }

    for ( s = 1; s <= p; s++ )
    {
        qs = q * s;
        qs1 = q * (s - 1);

        for ( r = 0; r <= qs; r++ )
        {
            for ( i = 0; i <= p - s; i++ )
            {
                if( uarray EQ NL_YES )
                    N_CopyCPt( NL_CZERO, &Uw[i][r] );
                else
                    N_CopyCPt( NL_CZERO, &Vw[i][r] );

                jmin = NL_MAX( 0, r - q );
                jmax = NL_MIN( r, qs1 );

                for ( j = jmin; j <= jmax; j++ )
                {
                    alf = f[r - j];
                    oma = 1.0 - alf;
                    fac = (NL_REAL)bin[qs1][j] * (NL_REAL)bin[q][r - j];

                    if( uarray EQ NL_YES )
                    {
                        N_Combine2CPts( oma, Vw[i][j], alf, Vw[i + 1][j], &Aw );
                        N_VectorBlendCPt( fac, Aw, &Uw[i][r] );
                    }
                    else
                    {
                        N_Combine2CPts( oma, Uw[i][j], alf, Uw[i + 1][j], &Aw );
                        N_VectorBlendCPt( fac, Aw, &Vw[i][r] );
                    }
                }
                fac = 1.0 / bin[qs][r];

                if( uarray EQ NL_YES )
                    N_ScaleCPt( fac, Uw[i][r], &Uw[i][r] );
                else
                    N_ScaleCPt( fac, Vw[i][r], &Vw[i][r] );
            }
        }

        if( uarray EQ NL_YES )
            uarray = NL_NO;
        else
            uarray = NL_YES;
    }

    if( uarray EQ NL_NO )
    {
        for ( r = 0; r <= pq; r++ )
        {
            N_CopyCPt( Uw[0][r], &Qw[r] );
        }
    }
    else
    {
        for ( r = 0; r <= pq; r++ )
        {
            N_CopyCPt( Vw[0][r], &Qw[r] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezReparam */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine splits a Bezier curve at a given parameter. A
     typical calling example is as follows:

       NL_CPOINT     *Pw, *Qw, *Rw;
       NL_DEGREE     p;
       NL_PARAMETER  u;
       ...
       (get Pw, u; allocate memory for Qw and Rw);
       ...
       N_BezSplit(Pw,p,u,Qw,Rw);

     IT IS ASSUMED THAT MEMORY TO STORE Qw[0],...,Qw[p] and Rw[0],...,
     Rw[p] IS ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     Pw  , input  ,  Original control points
     p   , input  ,  Degree
     u   , input  ,  Parameter where curve is to be split (must  be in
                     [0,1])
     Qw  , output ,  Control points of left segment
     Rw  , output ,  Control points of right segment


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSplit( NL_CPOINT *Pw, NL_DEGREE p, NL_PARAMETER u, NL_CPOINT *Qw, NL_CPOINT *Rw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSplit");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k;

    NL_CPOINT *Aw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Check parameter */

    if( u LT 0.0 OR u GT 1.0 )
        NL_ERROR( NL_PAR_ERR );

    /* Special cases */

    if( u EQ 0.0 )
    {
        for ( i = 0; i <= p; i++ )
        {
            N_CopyCPt( Pw[0], &Qw[i] );
            N_CopyCPt( Pw[i], &Rw[i] );
        }

        NL_OUT;
    }

    if( u EQ 1.0 )
    {
        for ( i = 0; i <= p; i++ )
        {
            N_CopyCPt( Pw[i], &Qw[i] );
            N_CopyCPt( Pw[p], &Rw[i] );
        }

        NL_OUT;
    }

    /* Split curve */

    Aw = N_AllocCPt1dArray( p, &S );

    if( Aw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p; i++ )
    {
        N_CopyCPt( Pw[i], &Aw[i] );
    }

    N_CopyCPt( Pw[0], &Qw[0] );
    N_CopyCPt( Pw[p], &Rw[p] );

    for ( k = 1; k <= p; k++ )
    {
        for ( i = 0; i <= p - k; i++ )
        {
            N_Combine2CPts( 1.0 - u, Aw[i], u, Aw[i + 1], &Aw[i] );
        }

        N_CopyCPt( Aw[0], &Qw[k] );
        N_CopyCPt( Aw[p - k], &Rw[p - k] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BezSplit */

/*******************************************************************//**
   DESCRIPTION:

     Recursive head to Bezier routine to compute the arc
     length of a Bezier curve. A typical calling example is as follows:

       NL_CPOINT  *Pw;
       NL_DEGREE  p;
       NL_REAL    *eps, *len;
       ...
       (get Pw);
       ...
       N_BezCalcArcLength(Pw,p,eps,len);

   ACCESS:
   
     Pw  , input  ,  Control points
     p   , input  ,  Degree
     eps , in/out ,  Tolerance
     len , in/out ,  Total length 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

    Note:  The recursion is limited to 10 levels
   ***********************************************************************/
NL_FLAG N_BezCalcArcLength
 (NL_CPOINT *Pw,    // in : Control points
  NL_DEGREE  p,     // in : Degree
  NL_REAL   *eps,   // i/o: Tolerance in: tol to beat, out: tolerance in Len 
  NL_REAL   *len )  // i/o: accumulating length 
{
    return (ST_BezCalcArcLength( Pw, p, eps, len, 0 ));
} /* end N_BezCalcArcLength */

/*******************************************************************//**
   DESCRIPTION:

     Recursive Body of Bezier routine to compute the arc
     length of a Bezier curve. A typical calling example is as follows:

       NL_CPOINT  *Pw;
       NL_DEGREE  p;
       NL_REAL    *eps, *len;
       ...
       (get Pw);
       ...
       N_BezCalcArcLength(Pw,p,eps,len);

   ACCESS:
   
     Pw  , input  ,  Control points
     p   , input  ,  Degree
     eps , in/out ,  Tolerance
     len , in/out ,  Total length 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

    Note:  The recursion is limited to 10 levels
   ***********************************************************************/
NL_FLAG ST_BezCalcArcLength
 (NL_CPOINT *Pw,    // in : Control points
  NL_DEGREE  p,     // in : Degree
  NL_REAL   *eps,   // i/o: Tolerance in: tol to beat, out: tolerance in Len increment
  NL_REAL   *len,   // i/o: accumulating length
  int        depth) // in : recursion depth
{
  // locals
  NL_FLAG    error = NL_NO;
  NL_REAL    del, epl, epr, UB, LB;
  NL_CPOINT *Qw, *Rw;
  NL_STACKS  S;
  
  /* Limit recursion to 20 levels */
  if( depth > NL_RECURSELIM )
    { return (1); }

  // init nurbs
  N_InitNurbs( &S );
  
  /* Get upper and lower bounds */
  N_DistCPolygon( Pw,    p,     &UB );
  N_DistCptCpt  ( Pw[0], Pw[p], &LB );
  
  /* upper/lower bound range */
  del = fabs( UB - LB );

  // if upper/lower bound range is less than tol
  if( del LE *eps )
    {
      // set len = UpperBound + LowerBound (later when divided by 2 - this sets arcLength = Avb(UB, LB)
      *len += UB + LB;

      // change tol Bound length
      *eps = del;
    }
  else // upper/lower bound range is greater than tol
    {
      // alloc two new Bsezzier arcs
      Qw = N_AllocCPt1dArray( p, &S );
      Rw = N_AllocCPt1dArray( p, &S );
  
      if(Qw EQ NULL || Rw EQ NULL )
        { NL_OUT; }

      // split this arc into two equal sized smaller arcs
      error = N_BezSplit( Pw, p, 0.5, Qw, Rw );
  
      if( error EQ NL_YES )
          NL_OUT;

      // divide tol by two for the smaller pieces - this lets Sum of recursive tols <= initial input tol
      epl = *eps / 2.0;

      // recurse on 1st half of BezierArc (adds 1st half arc length to total in len)
      error = ST_BezCalcArcLength( Qw, p, &epl, len, depth + 1 );
  
      if( error EQ NL_YES )
          NL_OUT;

      // pick a tol so sum of piece tols <= input tol
      epr = *eps - epl;

      // recurse on 2nd half of Bezier Arc (adds 2nd half arc length to total in len)
      error = ST_BezCalcArcLength( Rw, p, &epr, len, depth + 1 );
  
      if( error EQ NL_YES )
          NL_OUT;

      // set output tol to tol actually found
      *eps = epl + epr;
  }

  // exit and terminate Nurbs stack
  EXIT:
  N_EndNurbs( &S );

  // all done
  return (error);

} /* end ST_BezCalcArcLength */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the arc length of a Bezier curve. The
     length is  computed by  recursively  subdividing  the arc till the 
     upper bound (control polygon) and  the lower bound  (chord) do not
     deviate more  than a certain  tolerance. The method is very robust
     and  accurate! However, it is  slower than  numerical  integration
     techniques (that sometimes do not work). The  average run time for
     tolerances 10^{-1} to 10^{-5} is 3 to 250 iterations for curves of
     degrees 1-4. A typical calling example is as follows:

       NL_CPOINT  *Pw;
       NL_DEGREE  p;
       NL_REAL    tol, len;
       ...
       (get Pw);
       ...
       N_BezCalcLength(Pw,p,tol,&len);


   ACCESS:
   
     Pw  , input  ,  Control points
     p   , input  ,  Degree
     tol , input  ,  Tolerance
     len , output ,  Arc length 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_BezCalcLength
 (NL_CPOINT *Pw,   // in : Control points
  NL_DEGREE  p,    // in : Degree
  NL_REAL    tol,  // in : Tolerance
  NL_REAL   *len ) // out: Arc length 
{
  // locals
  NL_FLAG   error = NL_NO;
  NL_REAL   eps ;
  NL_REAL   total = 0.0 ;
  NL_STACKS S;
  
  /* Start NURBS */
  N_InitNurbs( &S );

  // double the tolerance
  eps = 2.0 * tol ;
  
  /* pass the call along to compute arc length */
  error = N_BezCalcArcLength(Pw,
                             p,
                             &eps,
                             &total );

  // check state
  if( error EQ NL_YES )
      NL_OUT;

  // divide total by 2.0
  *len = 0.5 *total;
  
  /* End NURBS and Exit */
  EXIT:
  N_EndNurbs( &S );

  // all done
  return (error);

} /* end N_BezCalcLength */

/*******************************************************************//**


   DESCRIPTION:

     Given end  points and end  tangents  <Ps,Ts> and <Pe,Te>, and one 
     additional point and tangent <P,T>. This routine computes a cubic
     Bezier curve that assumes  <Ps,Ts> and  <Pe,Te>, and approximates 
     <P,T>. A typical calling example is:

       NL_POINT   Ps, P, Pe, P1, P2;
       NL_VECTOR  Ts, T, Te;
       ...
       (get Ps,...,Te);
       ...
       N_BezCubicFromPtsAndTangents(Ps,Ts,P,T,Pe,Te,&P1,&P2);

     The approximation is done by using least-squares.


   ACCESS:
   
     Ps,Ts , input  ,  Start point and start tangent
     P,T   , input  ,  Point and tangent
     Pe,Te , input  ,  End point and end tangent
     P1,P2 , output ,  Middle control points of cubic


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCubicFromPtsAndTangents( NL_POINT Ps, NL_VECTOR Ts, NL_POINT P, NL_VECTOR T, NL_POINT Pe, NL_VECTOR Te, NL_POINT *P1, NL_POINT *P2 )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezCubicFromPtsAndTangents");

    NL_FLAG error = NL_NO;

    NL_REAL a[7][3], b[7], A[3][3], B[3], alf, bet, gam, s, s2, s3, t, t2, t3, den, numa, numb;

    NL_VECTOR V, D;

    /* Estimate parameter point is assumed at */

    N_DistPtPt( Ps, P, &alf );
    N_DistPtPt( P, Pe, &bet );

    t = alf / (alf + bet);
    t2 = t * t;
    t3 = t * t2;
    s = 1.0 - t;
    s2 = s * s;
    s3 = s * s2;

    /* Set up overdetermined system */

    alf = 3.0 *s2 * t;
    N_VectorScale( Ts, alf, &V );
    N_PtToXYZ( V, &a[1][1], &a[2][1], &a[3][1] );

    alf = 3.0 *s * t2;
    N_VectorScale( Te, alf, &V );
    N_PtToXYZ( V, &a[1][2], &a[2][2], &a[3][2] );

    alf = -(s3 + 3.0 *s2 * t);
    bet = -(t3 + 3.0 *s * t2);

    N_TranslateSum2Pts( P, alf, Ps, bet, Pe, &V );
    N_PtToXYZ( V, &b[1], &b[2], &b[3] );

    gam = 2.0 *s * t;
    alf = s2 - gam;
    N_VectorCross( T, Ts, &V );
    N_VectorScale( V, alf, &V );
    N_PtToXYZ( V, &a[4][1], &a[5][1], &a[6][1] );

    alf = gam - t2;
    N_VectorCross( T, Te, &V );
    N_VectorScale( V, alf, &V );
    N_PtToXYZ( V, &a[4][2], &a[5][2], &a[6][2] );

    N_VectorDiff( Ps, Pe, &D );
    N_VectorCross( T, D, &V );
    N_VectorScale( V, gam, &V );
    N_PtToXYZ( V, &b[4], &b[5], &b[6] );

    /* Solve by least squares */

    A[1][1] = a[1][1] * a[1][1] + a[2][1] * a[2][1] + a[3][1] * a[3][1] + a[4][1] * a[4][1] + a[5][1] * a[5][1] + a[6][1] * a[6][1];
    A[1][2] = a[1][1] * a[1][2] + a[2][1] * a[2][2] + a[3][1] * a[3][2] + a[4][1] * a[4][2] + a[5][1] * a[5][2] + a[6][1] * a[6][2];
    A[2][1] = A[1][2];
    A[2][2] = a[1][2] * a[1][2] + a[2][2] * a[2][2] + a[3][2] * a[3][2] + a[4][2] * a[4][2] + a[5][2] * a[5][2] + a[6][2] * a[6][2];

    B[1] = a[1][1] * b[1] + a[2][1] * b[2] + a[3][1] * b[3] + a[4][1] * b[4] + a[5][1] * b[5] + a[6][1] * b[6];
    B[2] = a[1][2] * b[1] + a[2][2] * b[2] + a[3][2] * b[3] + a[4][2] * b[4] + a[5][2] * b[5] + a[6][2] * b[6];

    den = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    numa = B[1] * A[2][2] - B[2] * A[1][2];
    numb = B[2] * A[1][1] - B[1] * A[2][1];

    if( N_FloatOpIsBad( numa, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( numb, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    alf = numa / den;
    bet = numb / den;

    if( alf LT 0.0 OR bet GT 0.0 )
        NL_ERROR( NL_INP_ERR );

    /* Compute inner control points */

    N_VectorPtAlongVector( Ps, alf, Ts, P1 );
    N_VectorPtAlongVector( Pe, bet, Te, P2 );

    /* Exit */

    EXIT:

    return (error);
} /* end N_BezCubicFromPtsAndTangents */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes a curve interpolating a given set of 
     points. A typical calling example is:

       NL_POINT   *Q;
       NL_INDEX   n;
       NL_CPOINT  *Pw;
       ...
       (define array Q);
       ...
       N_BezInterpNPts(Q,n,Pw);

     MEMORY TO STORE  Pw  MUST BE ALLOCATED IN THE  CALLING ROUTINE TO 
     HOLD Pw[0],...,Pw[n].


   ACCESS:
   
     Q   , input  ,  Points to be interpolated
     n   , input  ,  Highest index in Q
     Pw  , output ,  Control points of interpolating curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezInterpNPts( NL_POINT *Q, NL_INDEX n, NL_CPOINT *Pw )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_DEGREE pn;

    NL_REAL ** A, *B, *dist, sum;

    NL_PARAMETER *u;

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get parameters */

    u = N_AllocReal1dArray( n, &SL );

    if( u EQ NULL )
        NL_QUIT;

    dist = N_AllocReal1dArray( n, &SL );

    if( dist EQ NULL )
        NL_QUIT;

    pn = (NL_DEGREE)n;

    u[0] = 0.0;
    u[n] = 1.0;
    sum = 0.0;

    for ( i = 1; i <= n; i++ )
    {
        N_DistPtPt( Q[i - 1], Q[i], &dist[i] );
        sum += dist[i];
    }

    for ( i = 1; i < n; i++ )
        if( N_FloatOpIsBad( dist[i], sum, NL_DIVISION ) )
            break;

    if( i EQ n )
        for ( i = 1; i < n; i++ )
            u[i] = u[i - 1] + dist[i] / sum;
    else
        for ( i = 1; i < n; i++ )
            u[i] = (NL_REAL)i / (NL_REAL)n;

    /* Compute coefficient matrix */

    B = N_AllocReal1dArray( n, &SL );

    if( B EQ NULL )
        NL_QUIT;

    error = N_SetRealMatrix( &cm, n, n, NL_MT_FULL, n, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( j = 0; j <= n; j++ )
        A[0][j] = A[n][j] = 0.0;

    A[0][0] = 1.0;
    A[n][n] = 1.0;

    for ( i = 1; i < n; i++ )
    {
        error = N_BezEvalBasis( pn, u[i], B );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= n; j++ )
            A[i][j] = B[j];
    }

    /* Get control points */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezInterpNPts */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes a cubic curve interpolating a  given
     set of points. A typical calling example is:

       NL_POINT   *Q;
       NL_CPOINT  *Pw;
       ...
       (define array Q);
       ...
       N_BezInterp4Pts(Q,Pw);

     MEMORY TO STORE  Pw  MUST BE ALLOCATED IN THE  CALLING ROUTINE TO 
     HOLD Pw[0],...,Pw[3].


   ACCESS:
   
     Q   , input  ,  Points to be interpolated (Q[0],...,Q[3])
     Pw  , output ,  Control points of interpolating curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezInterp4Pts( NL_POINT *Q, NL_CPOINT *Pw )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL ** A, B[4], dist[4], sum;

    NL_PARAMETER u[4];

    NL_RMATRIX cm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get parameters */

    u[0] = 0.0;
    u[3] = 1.0;
    sum = 0.0;

    for ( i = 1; i <= 3; i++ )
    {
        N_DistPtPt( Q[i - 1], Q[i], &dist[i] );
        sum += dist[i];
    }

    if( N_FloatOpIsBad( dist[1], sum, NL_DIVISION )OR N_FloatOpIsBad( dist[2], sum, NL_DIVISION ) )
    {
        u[1] = 1.0 / 3.0;
        u[2] = 2.0 / 3.0;
    }
    else
    {
        u[1] = u[0] + dist[1] / sum;
        u[2] = u[1] + dist[2] / sum;
    }

    /* Compute coefficient matrix */

    error = N_SetRealMatrix( &cm, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &cm, &A );

    for ( j = 0; j <= 3; j++ )
        A[0][j] = A[3][j] = 0.0;

    A[0][0] = 1.0;
    A[3][3] = 1.0;

    for ( i = 1; i < 3; i++ )
    {
        error = N_BezEvalBasis( 3, u[i], B );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= 3; j++ )
            A[i][j] = B[j];
    }

    /* Get control points */

    error = N_RealMatrixLuDecompose( &cm );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixForBack( &cm, (NL_VOID *)Q, NL_EPOINT, Pw, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezInterp4Pts */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine  computes the control points of the product of 
     a Bezier function and a Bezier curve. Given a function f of degree 
     p, and  a curve C of degree q, the product  f*C is  another Bezier  
     curve of degree  p+q. This version  computes the product in 4-D. A 
     typical calling example is as follows:

       NL_REAL     *f;
       NL_CPOINT   *Pw, *Qw;
       NL_DEGREE   p, q;
       NL_INDEX    s, e;
       NL_RMATRIX  pm;
       ...
       (get f, Pw, p, q, s, e; allocate memory for Qw);
       ...
       N_InitRealMatrix(&pm); 
       N_BezFuncMultiplyBezCrv4D(f,p,Pw,q,&pm,s,e,Qw);

     The product computation matrix can be  precomputed and  passed in. 
     If it is initialized to the  NULL matrix, it  is  computed  inside 
     the routine. Also, the number  of control points can be restricted 
     by  the indexes  "s" (start)  and  "e" (end). THE ROUTINE  ASSUMES 
     THAT SUFFICIENT MEMORY IS AVAILABLE TO STORE Qw.


   ACCESS:
   
     f   , input  ,  Control values of function
     p   , input  ,  Degree of function
     Pw  , input  ,  Control points of curve
     q   , input  ,  Degree of curve
     pm  , input  ,  Product computation matrix
     s,e , input  ,  Start and end indexes (must satisfy  0<=s<=e<=p+q)
     Qw  , output ,  Control  points of product  curve (MEMORY  MUST BE 
                     ALLOCATED  IN THE  CALLING ROUTINE  TO HOLD  UP TO 
                     Qw[p+q])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezFuncMultiplyBezCrv4D( NL_REAL *f, NL_DEGREE p, NL_CPOINT *Pw, NL_DEGREE q, NL_RMATRIX *pm, NL_INDEX s, NL_INDEX e, NL_CPOINT *Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezFuncMultiplyBezCrv4D");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh;

    NL_REAL ** A;

    NL_CPOINT Tw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check indexes */

    if( s LT 0 OR e GT p + q OR s GT e )
        NL_ERROR( NL_INP_ERR );

    /* See if product matrix is needed */

    if( N_RealMatrixIsNULL( pm ) )
    {
        error = N_BezFuncMultiplyBezCrvMatrix( p, q, pm, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_GetRealMatrixPtr( pm, &A );

    /* Compute control points */

    for ( i = s; i <= e; i++ )
    {
        N_CopyCPt( NL_CZERO, &Qw[i] );

        jl = NL_MAX( 0, i - q );
        jh = NL_MIN( p, i );

        for ( j = jl; j <= jh; j++ )
        {
            N_ScaleCPt( f[j], Pw[i - j], &Tw );
            N_VectorBlendCPt( A[i][j], Tw, &Qw[i] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezFuncMultiplyBezCrv4D */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier  routine extends a Bezier  curve so that the extention
     and the original  curve share the same  derivatives in homogeneous  
     space. If the parametrization is maintained, the directions of the
     derivatives are preserved. Otherwise, the directions are reversed. 
     A typical calling example is as follows:

       NL_CPOINT   *Pw, *Qw;
       NL_DEGREE   p;

       ...

       N_BezCrvExtend(Pw,p,NL_START,NL_NO,Qw);

     The binomial coefficients, stored in the  Pascal  triangle, can be
     passed in. If NULL is  passed  in, the triangle is computed inside
     the routine. IT IS ASSUMED THAT  SUFFICIENT MEMORY IS AVAILABLE TO  
     STORE THE NL_NEW CONTROL NL_POINTS Qw.


   ACCESS:
   
     Pw  , input  ,  Control points of Bezier curve
     p   , input  ,  Degree of Bezier curve
     whr , input  ,  Flag:
                       NL_START: extend at the start
                       NL_END  : extend at the end
     rev , input  ,  Flag:
                       NL_YES: reverse parametrization
                       NL_NO : do not reverse parametrization
     bin , input  ,  Binomial coefficient array pointer,
                     Not used: No Longer Computed locally - set to NULL
     Qw  , output ,  New control points. MUST HOLD Qw[0],...,Qw[p].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCrvExtend( NL_CPOINT *Pw, NL_DEGREE p, NL_FLAG whr, NL_FLAG rev, NL_CPOINT *Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezCrvExtend");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL fac, dls, dgs, cls, cgs, sgs;

    NL_CPOINT Aw, Dw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if Pascal triangle is needed */

    /* Extend the curve */

    if( whr EQ NL_START )
    {
        if( rev EQ NL_YES )
            N_CopyCPt( Pw[0], &Qw[0] );
        else
            N_CopyCPt( Pw[0], &Qw[p] );

        dgs = -1.0;
        cgs = 1.0;
        sgs = -1.0;

        for ( i = 1; i <= p; i++ )
        {
            dls = dgs;
            cls = cgs;

            N_CopyCPt( NL_CZERO, &Dw );

            for ( j = 0; j <= i; j++ )
            {
                fac = dls * NL_PascalTri[i][j];
                N_VectorBlendCPt( fac, Pw[j], &Dw );
                dls = -dls;
            }

            N_CopyCPt( NL_CZERO, &Aw );

            for ( j = 0; j <= i - 1; j++ )
            {
                fac = cls * NL_PascalTri[i][j];

                if( rev EQ NL_YES )
                    N_VectorBlendCPt( fac, Qw[j], &Aw );
                else
                    N_VectorBlendCPt( fac, Qw[p - j], &Aw );

                cls = -cls;
            }

            if( rev EQ NL_YES )
            {
                N_ScaleCPt( -1.0, Dw, &Dw );
                N_Sum2CPts( Dw, Aw, &Qw[i] );
            }
            else
            {
                N_ScaleCPt( sgs, Dw, &Dw );
                N_Sum2CPts( Dw, Aw, &Qw[p - i] );
            }

            dgs = -dgs;
            cgs = -cgs;
            sgs = -sgs;
        }
    }
    else if( whr EQ NL_END )
    {
        if( rev EQ NL_NO )
            N_CopyCPt( Pw[p], &Qw[0] );
        else
            N_CopyCPt( Pw[p], &Qw[p] );

        cgs = 1.0;
        sgs = 1.0;

        for ( i = 1; i <= p; i++ )
        {
            dls = 1.0;
            cls = cgs;

            N_CopyCPt( NL_CZERO, &Dw );

            for ( j = 0; j <= i; j++ )
            {
                fac = dls * NL_PascalTri[i][j];
                N_VectorBlendCPt( fac, Pw[p - j], &Dw );
                dls = -dls;
            }

            N_CopyCPt( NL_CZERO, &Aw );

            for ( j = 0; j <= i - 1; j++ )
            {
                fac = cls * NL_PascalTri[i][j];

                if( rev EQ NL_NO )
                    N_VectorBlendCPt( fac, Qw[j], &Aw );
                else
                    N_VectorBlendCPt( fac, Qw[p - j], &Aw );

                cls = -cls;
            }

            if( rev EQ NL_YES )
            {
                N_ScaleCPt( sgs, Dw, &Dw );
                N_Sum2CPts( Dw, Aw, &Qw[p - i] );
            }
            else
            {
                N_Sum2CPts( Dw, Aw, &Qw[i] );
            }

            cgs = -cgs;
            sgs = -sgs;
        }
    }
    else
    {
        NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezCrvExtend */

/*******************************************************************//**


   DESCRIPTION:

     This  Bezier routine  extends a  Bezier  surface  strip so that the 
     extention and the original surface share the same drivatives in 4-D 
     space. If the  parametrizations are  maintained, the  directions of 
     the  derivatives are  preserved.  Otherwise, they  are  reversed. A 
     typical calling example is as follows:

       NL_SURFACE  surP, surQ;
       NL_INTEGER  **bin;
       NL_STACKS   SG;
       ...
       (get surP and bin);
       ...
       N_SrfInitArrays(&surQ);
       N_BezSrfExtend4D(&surP,NL_VDIR,NL_END  ,NL_YES,&surQ,&SG);

     The binomial coefficients, stored in the  Pascal  triangle, can be
     passed in. If NULL is  passed  in, the triangle is computed inside
     the routine. If surQ is initialized, memory  is allocated. If not,
     it is checked if enough memory is passed in.


   ACCESS:
   
     surP , input  ,  Bezier strip
     dir  , input  ,  Flag:
                        NL_UDIR: surface is Bezier in u-direction
                        NL_VDIR: surface is Bezier in v-direction
     whr  , input  ,  Flag:
                        NL_START: extend at u=umin/v=vmin
                        NL_END  : extend at u=umax/v=vmax
     rev  , input  ,  Flag:
                        NL_YES: reverse parametrization
                        NL_NO : do not reverse parametrization
     bin  , input  ,  Binomial coefficient array pointer:
                      No Longer computed locally - set to NULL.
     surQ , output ,  New Bezier strip
     SG   , input  ,  Stack of surP and surQ


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezSrfExtend4D( NL_SURFACE *surP, NL_FLAG dir, NL_FLAG whr, NL_FLAG rev, NL_SURFACE *surQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BezSrfExtend4D");

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q, pq;

    NL_INDEX i, j, n, m, r, s;

    NL_REAL *UP, *VP, *UQ, *VQ;

    NL_CPOINT ** Pw, ** Qw, *Aw, *Bw;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check input */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );

    if( dir EQ NL_UDIR AND p NEQ n )
        NL_ERROR( NL_INP_ERR );

    if( dir EQ NL_VDIR AND q NEQ m )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        surA = *surP;

        error = N_AllocSrfArrays( surP, n, m, p, q, r, s, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* See if Pascal triangle is needed and allocate memory */

    pq = (NL_DEGREE)( NL_MAX( p, q ) );

    Aw = N_AllocCPt1dArray( pq, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    Bw = N_AllocCPt1dArray( pq, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    /* Extend the strip in u-direction */

    if( dir EQ NL_UDIR )
    {
        for ( j = 0; j <= m; j++ )
        {
            for ( i = 0; i <= p; i++ )
                N_CopyCPt( Pw[i][j], &Aw[i] );

            error = N_BezCrvExtend( Aw, p, whr, rev, Bw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = 0; i <= p; i++ )
                N_CopyCPt( Bw[i], &Qw[i][j] );
        }
    }

    /* Extend the strip in v-direction */

    if( dir EQ NL_VDIR )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= q; j++ )
                N_CopyCPt( Pw[i][j], &Aw[j] );

            error = N_BezCrvExtend( Aw, q, whr, rev, Bw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( j = 0; j <= q; j++ )
                N_CopyCPt( Bw[j], &Qw[i][j] );
        }
    }

    /* Get knot vector and release memory if needed */

    for ( i = 0; i <= r; i++ )
        UQ[i] = UP[i];

    for ( j = 0; j <= s; j++ )
        VQ[j] = VP[j];

    if( surP EQ surQ )
        N_FreeSrf( &surA, SG );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BezSrfExtend4D */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the center, the radius, the start and
     the end  angles, and the start and end points of a Bezier circular 
     arc in 2D. If the arc is a straight line, the following predefined 
     quantities are returned:

       C   = (NL_INFINITE,NL_INFINITE,NL_INFINITE)
       r   = NL_INFINITE
       alf = NL_UNDEFINED
       bet = NL_UNDEFINED

     A typical calling example is:

       NL_CPOINT  P0w, P1w, P2w;
       NL_POINT   C, Ps, Pe;
       NL_REAL    r, alf, bet;
       ...
       (get P0w, P1w, P2w);
       ...
       N_BezCenterRadiusFrom3CPts(P0w,P1w,P2w,&C,&r,&alf,&bet,&Ps,&Pe);

     IT IS ASSUMED THAT P0w, P1w, and P2w DETERMINE A BEZIER CIRCLE!


   ACCESS:
   
     P0w,P1w,P2w , input  ,  Control points of Bezier conic
     C,r         , output ,  Center and radius of circle
     alf,bet     , output ,  Start and end angles 
     Ps,Pe       , output ,  Start and end points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BezCenterRadiusFrom3CPts                                                                             
 ( NL_CPOINT P0w,    /* in : CPt0 of 3 control points from a bezier conic                                 */
   NL_CPOINT P1w,    /* in : CPt1 of 3 control points from a bezier conic                                 */
   NL_CPOINT P2w,    /* in : CPt2 of 3 control points from a bezier conic                                 */
   NL_POINT *C,      /* out: circle center,       set to (NL_INFINITE, NL_INFINITE, NL_INFINITE) for line */
   NL_REAL  *r,      /* out: circle radius,       set to NL_INFINITE for line                             */
   NL_REAL  *alf,    /* out: start angle degrees, set to NL_UNDEFINED for line                            */
   NL_REAL  *bet,    /* out: end angle degrees,   set to NL_UNDEFINED for line                            */
   NL_POINT *Ps,     /* out: start point = CPt0                                                           */
   NL_POINT *Pe )    /* out: end point   = CPt2                                                           */
{
    NL_PRIVATE NL_STRING rname = _T("N_BezCenterRadiusFrom3CPts");

    NL_FLAG error = NL_NO;

    NL_REAL a, b, c, d;

    NL_POINT P0, P1, P2, M;

    NL_VECTOR N, V, R;

    /* Get Euclidean points and output end points */

    N_CPtToPtEuclid( P0w, &P0 );
    N_CPtToPtEuclid( P1w, &P1 );
    N_CPtToPtEuclid( P2w, &P2 );
    N_CopyPt( P0, Ps );
    N_CopyPt( P2, Pe );

    /* Check if arc is a line */

    error = N_DistPtLineSeg( P1, P0, P2, &d );

    if( error EQ NL_YES )
        NL_OUT;

    if( d LT NL_MTOL ) /* control points are colinear - mark output as having no circular approximation */
    {
        N_PtFromXYZ( NL_INFINITE, NL_INFINITE, NL_INFINITE, C );

        *r   = NL_INFINITE;
        *alf = NL_UNDEFINED;
        *bet = NL_UNDEFINED;

        NL_OUT;
    }

    /* Get radius, center and angles */

    N_Combine2Pts( 0.5, P0, 0.5, P2, &M );
    N_DistPtPt( P0, M, &a );
    N_DistPtPt( M, P1, &b );
    N_DistPtPt( P1, P0, &c );

    if( N_FloatOpIsBad( a * c, b, NL_DIVISION ) )
        NL_ERROR( NL_INP_ERR );

    *r = (a * c) / b;

    N_VectorDiff( P0, P1, &V );
    N_VectorPerpendicular( V, &N );
    N_VectorDiff( P2, P0, &V );
    N_VectorDot( N, V, &d );

    if( d LT 0.0 )
        N_VectorReverseInPlace( &N );

    error = N_VectorNormalizeRef( &N );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorPtAlongVector( P0, *r, N, C );
    N_VectorCreate( 1.0, 0.0, 0.0, &R );

    N_VectorDiff( P0, *C, &V );

    error = N_VectorDirectedAngle( R, V, alf );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDiff( P2, *C, &V );

    error = N_VectorDirectedAngle( R, V, bet );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_BezCenterRadiusFrom3CPts */

/*******************************************************************//**


   DESCRIPTION:

     This Bezier routine computes the  weighted control points of a 
     Bezier arc given by the iso-sceles triangle. A typical calling 
     example is:

       NL_POINT   P0, P1, P2;
       NL_CPOINT  Pw[3];
       ...
       (get P0, P1 and P2);
       ...
       N_BezCircArcFrom3Pts(P0,P1,P2,Pw);

     MEMORY FOR Pw MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     P0,P1,P2 , input  ,  Vertices of control triangle
     Pw       , output ,  Weighted control points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BezCircArcFrom3Pts( NL_POINT P0, NL_POINT P1, NL_POINT P2, NL_CPOINT *Pw )
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
} /* end N_BezCircArcFrom3Pts */
