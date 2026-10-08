// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* BasisAdv.c : Advanced Function Definitions that act on NL_KNOTVECTOR objects */
/*****************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>



/*******************************************************************//**


   DESCRIPTION:

     This routine finds the  smallest and the  largest non-zero spans in 
     a given knot vector. It is assumed that the knot vector is clamped, 
     i.e. the end knots are repeated with multiplicity equals  degree+1.
     A typical calling example is:
 
       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_REAL        ds, dl
       ...
       (define knt and get p);
       ...
       N_BasisGetLongestAndShortestSpans(&knt,p,&ds,&dl);


   ACCESS:
   
     knt   , input  ,  Knot vector
     p     , input  ,  Degree 
     ds,dl , output ,  Smallest and largest span distances


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BasisGetLongestAndShortestSpans( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_REAL *ds, NL_REAL *dl)
{
    NL_INDEX i, m;
    NL_REAL *U, d;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Find smallest and largest spans */
    *ds = U[p + 1] - U[p];
    *dl = U[p + 1] - U[p];

    for ( i = p + 1; i < m - p; i++ )
    {
        if( U[i]NEQ U[i + 1] )
        {
            d = U[i + 1] - U[i];

            if( d LT *ds )
                *ds = d;

            if( d GT *dl )
                *dl = d;
        }
    }

    N_EndNurbs( &S );
} /* end N_BasisGetLongestAndShortestSpans */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes new knots to merge two knot vectors that have
     the same end  multiplicities. To  be able to  merge them, the  knot
     vectors are  rescaled to a  common  interval, the longest of the two
     input spans. A  typical calling example is:

       NL_KNOTVECTOR  knr, kns, kxr, kxs;
       NL_DEGREE      p;
       ...
       (define knr, kns; get p; allocate memory for kxr and kxs);
       ...
       N_GetCompatibleKnotArrayMult(&knr,&kns,p,&kxr,&kxs);

     knots within NL_PTOL (1.0e-12) of one another are considered aligned.

     IT  IS  ASSUMED  THAT  MEMORY FOR kxr AND kxs  IS  ALLOCATED IN THE 
     CALLING  ROUTINE. 
     

   ACCESS:
   
     knr , in/out ,  Knot vector (WILL BE RESCALED)
     kns , in/out ,  Knot vector (WILL BE RESCALED)
     p   , input  ,  Degree of knr and kns
     kxr , output ,  Knot vector of new knots to be inserted into knr
     kxs , output ,  Knot vector of new knots to be inserted into kns


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetCompatibleKnotArrayMult( NL_KNOTVECTOR *knr, NL_KNOTVECTOR *kns, NL_DEGREE p, NL_KNOTVECTOR *kxr, NL_KNOTVECTOR *kxs )
{
    NL_INDEX i, j, k, mr, ms, mxr, mxs, ir, js, mlr, mls;
    NL_REAL *R, *S, *XR, *XS;
    NL_INTERVAL I;

    /* Get local notation, scale and initialize */
    N_KnotVectorGetKnots( knr, &mr, &R );
    N_KnotVectorGetKnots( kns, &ms, &S );
    N_KnotVectorGetKnots( kxr, &mxr, &XR );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( fabs( R[0] - S[0] )GT NL_PTOL OR fabs( R[mr] - S[ms] )GT NL_PTOL )
    {
        if( (R[mr] - R[0])GT( S[ms] - S[0] ) )
        {
            N_CreateInterval( &I, R[0], R[mr] );
            N_BasisReparam( kns, p, I );
        }
        else
        {
            N_CreateInterval( &I, S[0], S[ms] );
            N_BasisReparam( knr, p, I );
        }
    }

    mxr = -1;
    mxs = -1;
    i = p + 1;
    j = p + 1;

    /* Find knots to be inserted */
    while( i LT mr - p OR j LT ms - p )
    {
        if( fabs( R[i] - S[j] )LT NL_PTOL )
        {
            ir = i;

            while( i LT mr - p AND R[i]EQ R[i + 1] )
                i++;
            mlr = i - ir + 1;

            js = j;

            while( j LT ms - p AND S[j]EQ S[j + 1] )
                j++;
            mls = j - js + 1;

            for ( k = mlr + 1; k <= mls; k++ )
                XR[++mxr] = S[j];

            for ( k = mls + 1; k <= mlr; k++ )
                XS[++mxs] = R[i];

            i++;
            j++;
        }

        if( R[i]LT S[j] )
        {
            ir = i;

            while( i LT mr - p AND R[i]EQ R[i + 1] )
                i++;
            mlr = i - ir + 1;

            for ( k = 1; k <= mlr; k++ )
                XS[++mxs] = R[i];

            i++;
        }

        if( R[i]GT S[j] )
        {
            js = j;

            while( j LT ms - p AND S[j]EQ S[j + 1] )
                j++;
            mls = j - js + 1;

            for ( k = 1; k <= mls; k++ )
                XR[++mxr] = S[j];

            j++;
        }
    }

    /* Define output knot vectors */

    N_SetKnotIndex( kxr, mxr );
    N_SetKnotIndex( kxs, mxs );
} /* end N_GetCompatibleKnotArrayMult */


/*******************************************************************//**


   DESCRIPTION:

     This routine refines a knot vector with a  given knot vector. It is
     assumed that  new knots fit  into the old  ones, i.e. the following
     relationship must hold:

       U[p] < X[0] < X[1] <...< X[mx] < U[m-p]  

     where U[0],...,U[m] are the old knots, X[0],...,X[mx] are the knots
     to be inserted, and p is the degree. A typical calling example is:

       NL_KNOTVECTOR  knr, knx, kns;
       NL_DEGREE      p;
       ...
       (define knr; get knx; allocate memory for kns);
       ...
       N_BasisInsertKnots(&knr,p,&knx,&kns);

     MEMORY TO  STORE THE OUTPUT KNOT  NL_VECTOR  MUST BE  ALLOCATED IN THE 
     CALLING ROUTINE.
     

   ACCESS:
   
     knr , input  ,  Knot vector to be refined
     p   , input  ,  Degree of knr
     knx , input  ,  Knot vector to be inserted into knr
     kns , output ,  Refined knot vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisInsertKnots( NL_KNOTVECTOR *knr, NL_DEGREE p, NL_KNOTVECTOR *knx, NL_KNOTVECTOR *kns )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisInsertKnots");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, mr, ms, mx;
    NL_REAL *R, *X, *S;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation and initialize */
    N_KnotVectorGetKnots( knr, &mr, &R );
    N_KnotVectorGetKnots( knx, &mx, &X );
    N_KnotVectorGetKnots( kns, &ms, &S );

    if (X[0]LE R[p]OR X[mx]GE R[mr - p])
    {
        N_ErrSet(NL_KNT_ERR, rname);  error = 1;  goto EXIT;
    }

    /* Refine knot vector */
    for ( i = 0; i <= p; i++ )
        S[i] = R[i];

    i = p + 1;
    j = 0;
    ms = p;

    while( i LT mr - p OR j LE mx )
    {
        if( i GE mr - p )
        {
            S[++ms] = X[j++];
        }
        else if( j GT mx )
        {
            S[++ms] = R[i++];
        }
        else
        {
            if( R[i]GE X[j] )
                S[++ms] = X[j++];

            if( R[i]LT X[j] )
                S[++ms] = R[i++];
        }
    }

    for ( i = mr - p; i <= mr; i++ )
        S[++ms] = R[i];

    N_SetKnotIndex( kns, ms );

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BasisInsertKnots */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine adds knots to a knot vector. It recursively finds the
     longest  span and inserts  the mid point into the knot vector. The
     insertion  is  done  IN-PLACE, i.e. the  original  knot  vector is
     destroyed. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INTEGER     k;
       NL_DEGREE      p;
       ...
       (define knt and get k);
       ...
       N_BasisSplitLongestSpan(&knt,p,k);

     IT IS  ASSUMED THAT knt HAS  ENOUGH MEMORY TO HOLD THE NL_OLD AND THE 
     NEW KNOTS.


   ACCESS:
   
     knt , in/out ,  Knot vector
     p   , input  ,  Degree 
     k   , input  ,  Number of new knots to be added


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BasisSplitLongestSpan( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_INTEGER k )
{
    NL_INDEX i, j, l=0, m;
    NL_REAL *U, smax, uadd;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Add knots */
    N_KnotVectorGetKnots( knt, &m, &U );

    for ( i = 1; i <= k; i++ )
    {
        smax = 0.0;

        for ( j = p; j < m - p; j++ )
        {
            if( U[j + 1] - U[j]GT smax )
            {
                smax = U[j + 1] - U[j];
                l = j;
            }
        }

        uadd = 0.5 *(U[l] + U[l + 1]);

        for ( j = m; j >= l + 1; j-- )
            U[j + 1] = U[j];

        U[l + 1] = uadd;
        m++;
    }

    N_SetKnotIndex( knt, m );

    N_EndNurbs( &S );
} /* end N_BasisSplitLongestSpan */

/*******************************************************************//**


   DESCRIPTION:

     This routine refines a  knot interval by insering a given number of
     knots into the  interval. Let  [us,ue] be  the given  interval, the 
     routine recursively finds the longest span and inserts the midpoint 
     into [us,ue]. The  continuities at  us and  ue can be controlled. A 
     typical calling example is:

       NL_KNOTVECTOR  knt, knx;
       NL_DEGREE      p;
       NL_INDEX       n, ks, ke;
       NL_PARAMETER   us, ue;
       ...
       (define knt; get memory for knx; get us, ue, n, ks, and ke);
       ...
       N_BasisSplitNLongestSpans(&knt,p,us,ue,n,ks,ke,&knx);

     IT IS ASSUMED THE MEMORY TO STORE  knx  IS ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     knt   , input  ,  Knot vector
     p     , input  ,  Degree 
     us,ue , input  ,  Start and end parameters
     n     , input  ,  Number of knots to be added
     ks,ke , input  ,  Continuity  control; the  curve  must  be at most
                       C^{p-ks} continuous at us and C^{p-ke} at ue. 1<=
                         NL_PRIVATE  NL_STRING  rname = "N_BasisSplitNLongestSpans");
ks,ke<=p!
     knx   , output ,  New knot vector to be merged with knt


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisSplitNLongestSpans( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER us, NL_PARAMETER ue, NL_INDEX n, NL_INDEX ks, NL_INDEX ke, NL_KNOTVECTOR *knx )
{

    NL_PRIVATE NL_STRING rname = _T("N_BasisSplitNLongestSpans");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, l=0, is, ie, s, mx, mv, m;
    NL_REAL *U, *V, *X, vmax, vadd;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Check input */
    error = N_KnotVectorIsParamOutOfBounds( knt, us, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knt, ue, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( ks LT 1 OR ks GT p )
        NL_ERROR( NL_INP_ERR );

    if( ke LT 1 OR ke GT p )
        NL_ERROR( NL_INP_ERR );

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );
    N_KnotVectorGetKnots( knx, &mx, &X );

    mx = -1;

    /* Handle left knot */
    error = N_BasisFindSpanAndMult( knt, p, us, NL_LEFT, &is, &s );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 1; i <= ks - s; i++ )
        X[++mx] = us;

    /* Handle right knot */
    error = N_BasisFindSpanAndMult( knt, p, ue, NL_LEFT, &ie, &s );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 1; i <= ke - s; i++ )
        X[++mx] = ue;

    /* Add knots */
    V = N_AllocReal1dArray( ie - is + n + 2, &S );

    if( V EQ NULL )
        NL_QUIT;

    mv = 0;
    V[0] = us;

    for ( i = is + 1; i <= ie - s; i++ )
        V[++mv] = U[i];
    V[++mv] = ue;

    for ( i = 1; i <= n; i++ )
    {
        vmax = 0.0;

        for ( j = 0; j < mv; j++ )
        {
            if( V[j + 1] - V[j]GT vmax )
            {
                vmax = V[j + 1] - V[j];
                l = j;
            }
        }

        vadd = 0.5 *(V[l] + V[l + 1]);
        X[++mx] = vadd;

        for ( j = mv; j >= l + 1; j-- )
            V[j + 1] = V[j];
        V[l + 1] = vadd;
        mv++;
    }

    /* Sort array and define knot vector */
    N_ShellSortReal( X, mx );
    N_SetKnotIndex( knx, mx );

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisSplitNLongestSpans */


#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine increases the multiplicity of each internal knot of a
     knot vector. It computes a new knot vector to be inserted into the
     original one to obtain the required multiplicities. As an example,
     consider the following degree 4 knot vector:
 
       U = { 0, 0, 0, 0, 0, 0.2, 0.4, 0.4, 0.7, 1, 1, 1, 1, 1 }
  NL_PRIVATE  NL_STRING  rname = "N_BasisIncreaseKnotMult");

     Inreasing the  multiplicity of  each internal  knot by 2 gives the
     following new knots vector:

       X = { 0.2, 0.2, 0.4, 0.4, 0.7, 0.7 }

     Inserting  X into  U yields the required multiplicities. A typical 
     calling example is:

       NL_KNOTVECTOR  knt, knx;
       NL_INDEX       t;
       NL_DEGREE      p;
       ...
       (define knt; get p and t; allocate memory for knx);
       ...
       N_BasisIncreaseKnotMult(&knt,p,t,&knx);

     THIS  ROUTINE IS  SUITABLE TO LOWER THE CONTINUITY OF A NL_CURVE OR A
     NL_SURFACE. MEMORY TO STORE THE OUTPUT KNOT NL_VECTOR  MUST BE ALLOCATED 
     IN THE CALLING ROUTINE.
     

   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree of knt
     t   , input  ,  Increase multiplicity of each knot by t
     knx , output ,  Knot vector to be inserted into knt


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisIncreaseKnotMult( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_INDEX t, NL_KNOTVECTOR *knx )
{

    NL_PRIVATE NL_STRING rname = _T("N_BasisIncreaseKnotMult");
    NL_FLAG error = NL_NO;
    NL_INDEX i, k, mu, mx;
    NL_REAL *U, *X;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation and initialize */
    N_KnotVectorGetKnots( knt, &mu, &U );
    N_KnotVectorGetKnots( knx, &mx, &X );

    mx = -1;
    i = p + 1;

    /* Find knots to be inserted */
    while( i LT mu - p )
    {
        k = i;

        while( i LT mu - p AND U[i]EQ U[i + 1] )
            i++;

        if( i - k + t + 1 GT p )
            NL_ERROR( NL_KNT_ERR );

        for ( k = 1; k <= t; k++ )
            X[++mx] = U[i];

        i++;
    }

    N_SetKnotIndex( knx, mx );

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BasisIncreaseKnotMult */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the node  corresponding to a given index. A 
     typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_INDEX       k;
       NL_PARAMETER   t;
       ... 
       (define knt, get p and k);
       ...
       N_BasisFindIndexNode(&knt,p,k,&t);

     The index k must satisfy 0<=k<=n, where n is the highest index of
     control  points. IT IS ASSUMED THAT THE NL_END KNOTS ARE CLAMPED, IE 
     THEY ARE REPEATED WITH MULTIPLICITY (p+1), WHERE p IS THE NL_DEGREE.


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     k   , input  ,  Index 
     t   , output ,  Node corresponding to k


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisFindIndexNode( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_INDEX k, NL_PARAMETER *t )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisFindIndexNode");
    NL_FLAG error = NL_NO;
    NL_INDEX i, n, m;
    NL_REAL *U, sum;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation and check index */
    N_KnotVectorGetKnots( knt, &m, &U );
    n = m - p - 1;

    if( k LT 0 OR k GT n )
        NL_ERROR( NL_IND_ERR );

    /* Compute node */
    if( k EQ 0 )
    {
        *t = U[0];
        NL_OUT;
    }

    if( k EQ n )
    {
        *t = U[m];
        NL_OUT;
    }

    sum = 0.0;

    for ( i = 1; i <= p; i++ )
        sum += U[k + i];
    *t = sum / p;

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisFindIndexNode */

/*******************************************************************//**


   DESCRIPTION:

     This routine finds the node span a given parameter is in. That is,
     given  u, it finds the nodes  t[k] and t[k+1] such that  t[k]<=u<=
     t[k+1]. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   u, tk, tk1;
       NL_INDEX       k;
       ... 
       (define knt, get p and u);
       ...
       N_BasisFindNodeSpan(&knt,p,u,&tk,&tk1,&k);


   ACCESS:
   
     knt    , input  ,  Knot vector
     p      , input  ,  Degree 
     u      , input  ,  Parameter
     tk,tk1 , output ,  Node span
     k      , output ,  Index of left node


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisFindNodeSpan( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER u, NL_PARAMETER *tk, NL_PARAMETER *tk1, NL_INDEX *k )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisFindNodeSpan");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, n, m, spn;
    NL_REAL *U, sum, fact, tl, tr=0;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Check parameter */
    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );
    n = m - p - 1;

    /* Special cases */
    if( u EQ U[0] )
    {
        error = N_BasisFindIndexNode( knt, p, 0, tk );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindIndexNode( knt, p, 1, tk1 );

        if( error EQ NL_YES )
            NL_OUT;

        *k = 0;

        NL_OUT;
    }

    if( u EQ U[m] )
    {
        error = N_BasisFindIndexNode( knt, p, n - 1, tk );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindIndexNode( knt, p, n, tk1 );

        if( error EQ NL_YES )
            NL_OUT;

        *k = n - 1;

        NL_OUT;
    }

    /* Find node span */
    error = N_BasisFindSpan( knt, p, u, NL_LEFT, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindIndexNode( knt, p, spn - p, &tl );

    if( error EQ NL_YES )
        NL_OUT;

    fact = 1.0 / p;

    for ( i = spn - p + 1; i <= spn; i++ )
    {
        sum = 0.0;

        for ( j = 1; j <= p; j++ )
            sum += U[i + j];
        tr = fact * sum;

        if( u GE tl AND u LE tr )
            break;

        tl = tr;
    }

    *tk = tl;
    *tk1 = tr;
    *k = i - 1;

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisFindNodeSpan */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a new parameter uh so that if uh is inserted
     into the  knot vector, a  given parameter  u becomes a new node. A 
     typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   u, uh;
       ... 
       (define knt, get p and u);
       ...
       N_BasisFindKnotToTurnParamIntoNode(&knt,p,u,&uh);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Parameter to become a node
     uh  , output ,  Parameter to be inserted to get u as a new node


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisFindKnotToTurnParamIntoNode( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER u, NL_PARAMETER *uh )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisFindKnotToTurnParamIntoNode");
    NL_FLAG error = NL_NO;
    NL_INDEX i, k, m;
    NL_REAL *U, sum, tmp;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Check parameter */
    error = N_KnotVectorIsEndParam( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Find node span */
    error = N_BasisFindNodeSpan( knt, p, u, &tmp, &tmp, &k );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute new parameter */
    sum = 0.0;

    for ( i = 2; i <= p; i++ )
        sum += U[k + i];

    *uh = p * u - sum;

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisFindKnotToTurnParamIntoNode */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes all  non-vanishing  basis  functions at a 
     given array of parameter values. It is  assumed that the required 
     memory to  store the basis  functions is allocated in the calling 
     routine. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   *u;
       NL_REAL        **N;
       NL_INDEX       *spn, n;
       ...
       (define knt, get p, u, and allocate memory for N and spn);
       ...
       N_BasisEvalArray(&knt,p,u,n,NL_LEFT,N,spn);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Array of parameters given in INCREASING ORDER!
     n   , input  ,  Highest index in u
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     N   , output ,  All basis  functions  computed at  u[0],...,u[n].
                     N[i][0..p] are the  non-vanishing basis functions
                     computed at  u[i]. MEMORY FOR N MUST BE ALLOCATED 
                     IN THE CALLING ROUTINE TO HOLD UP TO N[n][p].
     spn , output ,  spn[i] is  the left  index of  span  u[i]  is in. 
                     MEMORY FOR  spn MUST BE ALLOCATED  IN THE CALLING 
                     ROUTINE TO HOLD UP TO spn[n].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisEvalArray( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER *u, NL_INDEX n, NL_FLAG flg, NL_REAL ** N, NL_INDEX *spn )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisEvalArray");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, k, l, m, js, je;
    NL_REAL *U, *left, *right, saved, temp;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Check parameters and initialize indexes */
    if( n LT 0 )
        NL_ERROR( NL_IND_ERR );

    if( u[0]LT U[0]OR u[n]GT U[m] )
        NL_ERROR( NL_INP_ERR );

    js = 0;
    je = n;

    /* Special cases for end values */
    if( u[0]EQ U[p] )
    {
        N[0][0] = 1.0;
        spn[0] = p;
        js = 1;

        for ( k = 1; k <= p; k++ )
            N[0][k] = 0.0;
    }

    if( u[n]EQ U[m - p] )
    {
        N[n][p] = 1.0;
        spn[n] = m - p - 1;
        je = n - 1;

        for ( k = 0; k < p; k++ )
            N[n][k] = 0.0;
    }

    if( je LT js )
        NL_OUT;

    /* Find the knot span u[js] is in */
    error = N_BasisFindSpan( knt, p, u[js], flg, &i );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the non-vanishing B-splines */
    left = N_AllocReal1dArray( p, &S );

    if( left EQ NULL )
        NL_QUIT;

    right = N_AllocReal1dArray( p, &S );

    if( right EQ NULL )
        NL_QUIT;

    for ( j = js; j <= je; j++ )
    {
        spn[j] = i;
        N[j][0] = 1.0;

        for ( k = 1; k <= p; k++ )
        {
            left[k] = u[j] - U[i + 1 - k];
            right[k] = U[i + k] - u[j];
            saved = 0;

            for ( l = 0; l < k; l++ )
            {
                temp = N[j][l] / (right[l + 1] + left[k - l]);
                N[j][l] = saved + right[l + 1] * temp;
                saved = left[k - l] * temp;
            }
            N[j][k] = saved;
        }

        switch( flg )
        {
            case NL_LEFT:
                while( j LT je AND u[j + 1]GE U[i + 1] )
                    i++;
                break;

            case NL_RIGHT:
                while( j LT je AND u[j + 1]GT U[i + 1] )
                    i++;
                break;
        }
    }

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisEvalArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes all non-vanishing  basis functions and their
     derivatives for an array of given  parameter values. It  is assumed  
     that the required  memory to  store the basis  functions and  their 
     derivatives is  allocated  in  the  calling  routine.  The  highest 
     indexes  are  ND[d][p][n],  where  d  is  the  highest   derivative 
     required, p is the degree of the curve, and n is the  highest index  
     of parameters. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   *u;
       NL_INDEX       n, der, *spn;
       NL_REAL        ***ND;
       ...
       (define knt, get p, u, der and allocate memory for ND and spn);
       ...
       N_BasisDerivsArray(&knt,p,u,n,NL_LEFT,der,ND,spn);

     MEMORY FOR ND AND spn MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Parameters
     n   , input  ,  Highest index in u
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[l],u[l+1])
                              (NL_RIGHT DERIVATIVES REQUIRED) 
                       NL_RIGHT: u is in (u[l],u[l+1]]
                              (NL_LEFT DERIVATIVES REQUIRED) 
     der , input  ,  Highest derivative required 
     ND  , output ,  All basis functions and their derivatives  computed 
                     at u[0],...,u[n]. ND[k][i][j] is the kth derivative
                     of the  basis  function  N[l-p+i]  computed at u[j] 
                     where u[j] is in {u[l],u[l+1]}.
     spn , output ,  spn[j] is the left index of the span u[j] is in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisDerivsArray( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER *u, NL_INDEX n, NL_FLAG flg, NL_INDEX der, NL_REAL *** ND, NL_INDEX *spn )
{

    NL_FLAG error = NL_NO;
    NL_INDEX i, j, k, l, r, s1, s2, rk, pk, i1, i2, mder;
    NL_REAL *U, *left, *right, ** ndu, ** a, saved, temp, d;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &k, &U );

    /* Compute span array */
    error = N_BasisFindSpan( knt, p, u[n], flg, &spn[n] );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpan( knt, p, u[0], flg, &k );

    if( error EQ NL_YES )
        NL_OUT;

    spn[0] = k;

    for ( i = 1; i <= n - 1; i++ )
    {
        switch( flg )
        {
            case NL_LEFT:
                while( u[i]GE U[k + 1] )
                    k++;

            case NL_RIGHT:
                while( u[i]GT U[k + 1] )
                    k++;
        }
        spn[i] = k;
    }

    /* Get maximum derivative index and set zero derivatives */
    mder = NL_MIN( p, der );

    for ( k = p + 1; k <= der; k++ )
    {
        for ( i = 0; i <= p; i++ )
        {
            for ( j = 0; j <= n; j++ )
                ND[k][i][j] = 0.0;
        }
    }

    /* Allocate memory */
    left = N_AllocReal1dArray( p, &S );

    if( left EQ NULL )
        NL_QUIT;

    right = N_AllocReal1dArray( p, &S );

    if( right EQ NULL )
        NL_QUIT;

    ndu = N_AllocReal2dArray( p, p, &S );

    if( ndu EQ NULL )
        NL_QUIT;

    a = N_AllocReal2dArray( 1, p, &S );

    if( a EQ NULL )
        NL_QUIT;

    /* Compute the basis functions */
    for ( j = 0; j <= n; j++ )
    {
        l = spn[j];
        ndu[0][0] = 1.0;

        for ( i = 1; i <= p; i++ )
        {
            left[i] = u[j] - U[l + 1 - i];
            right[i] = U[l + i] - u[j];
            saved = 0.0;

            for ( r = 0; r < i; r++ )
            {
                ndu[i][r] = right[r + 1] + left[i - r];
                temp = ndu[r][i - 1] / ndu[i][r];
                ndu[r][i] = saved + right[r + 1] * temp;
                saved = left[i - r] * temp;
            }
            ndu[i][i] = saved;
        }

        /* Load basis functions */
        for ( i = 0; i <= p; i++ )
            ND[0][i][j] = ndu[i][p];

        /* Compute derivatives */
        for ( r = 0; r <= p; r++ )
        {
            s1 = 0;
            s2 = 1;
            a[0][0] = 1.0;

            for ( k = 1; k <= mder; k++ )
            {
                d = 0.0;
                rk = r - k;
                pk = p - k;

                if( r GE k )
                {
                    a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                    d = a[s2][0] * ndu[rk][pk];
                }

                if( rk GE - 1 )
                    i1 = 1;
                else
                    i1 = -rk;

                if( (r - 1)LE pk )
                    i2 = k - 1;
                else
                    i2 = p - r;

                for ( i = i1; i <= i2; i++ )
                {
                    a[s2][i] = (a[s1][i] - a[s1][i - 1]) / ndu[pk + 1][rk + i];
                    d += a[s2][i] * ndu[rk + i][pk];
                }

                if( r LE pk )
                {
                    a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                    d += a[s2][k] * ndu[r][pk];
                }

                ND[k][r][j] = d;
                N_SwapIntegers( &s1, &s2 );
            }
        }

        /* Multiply through by the correct factors */

        r = p;

        for ( k = 1; k <= mder; k++ )
        {
            for ( i = 0; i <= p; i++ )
                ND[k][i][j] *= r;
            r *= (p - k);
        }
    }

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisDerivsArray */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivative of one basis function with
     respect to a knot. The  multiplicity of  the knot  must be less
     than the degree. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       i, k;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_REAL        Nk;
       ...
       (define knt, get i, k, p and u);
       ...
       N_BasisIKnotDeriv(&knt,i,k,p,u,NL_LEFT,NL_RIGHT,&Nk);


   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline
     k   , input  ,  Index of knot, i.e. the derivative with respect 
                     to u_k is computed
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flk , input  ,  Flag:
                       NL_LEFT : left derivative. NL_INDEX  k MUST SATISFY
                               u_(k) != u_(k-1)
                       NL_RIGHT: right derivative. NL_INDEX k MUST SATISFY
                               u_(k) != u_(k+1)
     flp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     Nk  , output ,  Basis function derivative computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisIKnotDeriv( NL_KNOTVECTOR *knt, NL_INDEX i, NL_INDEX k, NL_DEGREE p, NL_PARAMETER u, NL_FLAG flk, NL_FLAG flp, NL_REAL *Nk )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisIKnotDeriv");
    NL_FLAG error = NL_NO;
    NL_INDEX a, b, m, mlt;
    NL_REAL *U, *Uh, Na, Nb;
    NL_KNOTVECTOR *knh;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Check input data */
    N_KnotVectorGetKnots( knt, &m, &U );

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT m - p - 1 )
        NL_ERROR( NL_IND_ERR );

    if( k LE p OR k GT m - p - 1 )
        NL_ERROR( NL_IND_ERR );

    switch( flk )
    {
        case NL_LEFT:
            if( U[k]EQ U[k - 1] )
                NL_ERROR( NL_INP_ERR );

            a = k;

            while( U[a]EQ U[a + 1] )
                a++;
            mlt = a - k + 1;

            if( mlt GE p )
                NL_ERROR( NL_KML_ERR );
            break;

        case NL_RIGHT:
            if( U[k]EQ U[k + 1] )
                NL_ERROR( NL_INP_ERR );

            a = k;

            while( U[a]EQ U[a - 1] )
                a--;
            mlt = k - a + 1;

            if( mlt GE p )
                NL_ERROR( NL_KML_ERR );
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Get knot vector knh */
    knh = N_AllocKnotVectorAndArray( m + 1, &SL );

    if( knh EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( knh, &a, &Uh );

    switch( flk )
    {
        case NL_LEFT:

            b = -1;

            for ( a = 0; a <= k + mlt - 1; a++ )
                Uh[++b] = U[a];
            Uh[++b] = U[k];

            for ( a = k + mlt; a <= m; a++ )
                Uh[++b] = U[a];
            break;

        case NL_RIGHT:

            b = -1;

            for ( a = 0; a <= k; a++ )
                Uh[++b] = U[a];
            Uh[++b] = U[k];

            for ( a = k + 1; a <= m; a++ )
                Uh[++b] = U[a];
            break;
    }

    /* Evaluate derivative */
    if( i LT k - p - 1 OR i GT k )
    {
        *Nk = 0.0;
        NL_OUT;
    }

    if( i EQ k - p - 1 )
    {
        error = N_BasisIEval( knh, k - p, p, u, flp, &Na );

        if( error EQ NL_YES )
            NL_OUT;

        *Nk = Na / (U[k] - U[k - p]);
    }
    else if( i LE k - 1 AND i GE k - p )
    {
        error = N_BasisIEval( knh, i + 1, p, u, flp, &Na );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knh, i, p, u, flp, &Nb );

        if( error EQ NL_YES )
            NL_OUT;

        *Nk = (Na / (U[i + p + 1] - U[i + 1])) - (Nb / (U[i + p] - U[i]));
    }
    else if( i EQ k )
    {
        error = N_BasisIEval( knh, k, p, u, flp, &Nb );

        if( error EQ NL_YES )
            NL_OUT;

        *Nk = -Nb / (U[k + p] - U[k]);
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BasisIKnotDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivatives of all non-vanishing basis
     functions with respect to a  knot. The multiplicity of  the knot  
     must be less than the degree. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       k;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_REAL        *Nk;
       ...
       (define knt, get k, p and u);
       ...
       N_BasisKnotDerivs(&knt,k,p,u,NL_LEFT,NL_RIGHT,Nk);

     MEMORY  TO  STORE  Nk[0],...,Nk[p+1]  MUST  BE  ALLOCATED IN THE 
     CALLING ROUTINE.


   ACCESS:
   
     knt , input  ,  Knot vector
     k   , input  ,  Index of knot, i.e. the derivative  with respect 
                     to u_k is computed
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative. NL_INDEX  k MUST SATISFY
                              u_(k) != u_(k-1)
                       NL_RIGHT: right  derivative. NL_INDEX k MUST SATISFY
                              u_(k) != u_(k+1)
     flp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     Nk  , output ,  Basis  function  derivatives  computed at u. The 
                     values are stored in Nk[0],...,Nk[p+1].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisKnotDerivs( NL_KNOTVECTOR *knt, NL_INDEX k, NL_DEGREE p, NL_PARAMETER u, NL_FLAG flk, NL_FLAG flp, NL_REAL *Nk )
{

    NL_FLAG error = NL_NO;
    NL_INDEX i;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get basis functions */
    for ( i = k - p - 1; i <= k; i++ )
    {
        error = N_BasisIKnotDeriv( knt, i, k, p, u, flk, flp, &Nk[i - k + p + 1] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BasisKnotDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivatives of all non-vanishing basis
     functions of a surface with respect to a  knot. The multiplicity 
     of the knot in either direction must be less than the respective 
     degree. A typical calling example is:

       NL_KNOTVECTOR  knu, knv;
       NL_DEGREE      p, q;
       NL_INDEX       k, spn;
       NL_PARAMETER   u, v;
       NL_REAL        **Bk;
       ...
       (define knu, knv; get p, q, k, u and v);
       ...
       N_BiBasisKnotDeriv(&knu,&knv,p,q,k,u,v,NL_UDIR,NL_LEFT,NL_RIGHT,NL_LEFT,Bk,&spn);

     MEMORY TO STORE Bk[0][0],...,Bk[p+1][q] (OR Bk[p][q+1])  MUST BE  
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     knu , input  ,  U-knot vector
     knv , input  ,  V-knot vector
     p,q , input  ,  Degrees
     k   , input  ,  Index of knot, i.e. the derivative  with respect 
                     to t_k is computed (t is either u or v)
     u,v , input  ,  Parameter values
     dir , input  ,  Flag:
                       NL_UDIR: u-derivative required. Bk MUST  STORE UP
                             TO [p+1][q].
                       NL_VDIR: v-derivative required. Bk MUST  STORE UP
                             TO [p][q+1].
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative. NL_INDEX  k MUST SATISFY
                              t_(k) != t_(k-1)
                       NL_RIGHT: right  derivative. NL_INDEX k MUST SATISFY
                              t_(k) != t_(k+1)
                              (t is either u or v)
     ulp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]]
     vlp , input  ,  Flag:
                       NL_LEFT : v is in [v[j],v[j+1]) 
                       NL_RIGHT: v is in (v[j],v[j+1]]
     Bk  , output ,  Basis  function  derivatives  computed at (u,v). 
                     The  values are stored in Bk[0][0],...,Bk[a][b], 
                     where a = {p+1|p} and b = {q+1|q} (see above).
     spn , output ,  Span index


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BiBasisKnotDeriv( NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv, NL_DEGREE p, NL_DEGREE q, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG dir, NL_FLAG flk, NL_FLAG ulp, NL_FLAG vlp, NL_REAL ** Bk, NL_INDEX *spn )
{

    NL_FLAG error = NL_NO;
    NL_INDEX i, j;
    NL_REAL *Nu, *Nv;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get basis functions */
    if( dir EQ NL_UDIR )
    {
        Nu = N_AllocReal1dArray( p + 1, &SL );

        if( Nu EQ NULL )
            NL_QUIT;

        Nv = N_AllocReal1dArray( q, &SL );

        if( Nv EQ NULL )
            NL_QUIT;

        error = N_BasisKnotDerivs( knu, k, p, u, flk, ulp, Nu );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisEval( knv, q, v, vlp, Nv, spn );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= p + 1; i++ )
        {
            for ( j = 0; j <= q; j++ )
            {
                Bk[i][j] = Nu[i] * Nv[j];
            }
        }
    }

    if( dir EQ NL_VDIR )
    {
        Nu = N_AllocReal1dArray( p, &SL );

        if( Nu EQ NULL )
            NL_QUIT;

        Nv = N_AllocReal1dArray( q + 1, &SL );

        if( Nv EQ NULL )
            NL_QUIT;

        error = N_BasisEval( knu, p, u, ulp, Nu, spn );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisKnotDerivs( knv, k, q, v, flk, vlp, Nv );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= p; i++ )
        {
            for ( j = 0; j <= q + 1; j++ )
            {
                Bk[i][j] = Nu[i] * Nv[j];
            }
        }
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BiBasisKnotDeriv */

#endif // NLIB_UNUSED
