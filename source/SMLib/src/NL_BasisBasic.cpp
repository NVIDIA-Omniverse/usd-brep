// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************************/
/* BasisBasic.c : Basic Function Definitions that act on NL_KNOTVECTOR objects */
/*******************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>


/* -------------------------- */
/* Local Function Definitions */
/* -------------------------- */

NL_BOOLEAN ST_notdone( NL_INDEX *, NL_KNOTVECTOR **, NL_INDEX );

/**********************************************************************/
/* N_KnotVectorIsParamOutOfBounds: Check if parameter is out of range */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if a given parameter is out of range, 
     i.e. if u < U[0] or u > U[m]. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_PARAMETER   u;
       NL_STRING      rname;
       ...
       (define knt, get u and rname);
       ...
       N_KnotVectorIsParamOutOfBounds(&knt,u,rname);


   ACCESS:
   
     knt   , input  ,  Knot vector
     u     , input  ,  Parameter value
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_KnotVectorIsParamOutOfBounds( NL_KNOTVECTOR *knt, NL_PARAMETER u, NL_STRING rname )
{
    NL_FLAG error = NL_NO;
    NL_INDEX m;
    NL_REAL *U;

    /* Convert to local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Check parameter */
    if( u LT U[0]OR u GT U[m] )
    {
        N_ErrSet( NL_PAR_ERR, rname );
        error = NL_YES;
    }

    return (error);
} /* N_KnotVectorIsParamOutOfBounds */

/*****************************************************************************/
/* N_KnotVectorIsEndParam: Check if parameter <= first knot or >= last knot  */
/*****************************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if a given parameter is equal to or less
     than  the first  knot, or  equal to or greater than the last knot, 
     i.e. if u <= U[0] or u >= U[m]. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_PARAMETER   u;
       NL_STRING      rname;
       ...
       (define knt, get u and rname);
       ...
       N_KnotVectorIsEndParam(&knt,u,rname);


   ACCESS:
   
     knt   , input  ,  Knot vector
     u     , input  ,  Parameter value
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     NL_YES : param u is on or outside of the [1stKnot LastKnot] span boundary
              to machine precision.
     NL_NO  : param u is inside the [1stKnot LastKnot] span to machine precision.

   ***********************************************************************/
NL_FLAG N_KnotVectorIsEndParam /* rtn: NL_YES = u is on or outside knot span, NL_NO = inside  */
 (NL_KNOTVECTOR * knt,         /* in : Knotvector specifying span [1stKnot LastKnot] to check */
  NL_PARAMETER    u,           /* in : u parameter to classify (to machine precision)         */
  const TCHAR   * rname )      /* in : calling function's name label used for error reporting */
{
    NL_FLAG error = NL_NO;
    NL_INDEX m;
    NL_REAL *U;

    /* Convert to local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Check parameter */
    if( u LE U[0] OR u GE U[m] )
    {
        N_ErrSet( NL_PAR_ERR, rname );
        error = NL_YES;
    }

    return (error);
} /* end N_KnotVectorIsEndParam */


/**********************************************************************/
/* N_KnotVectorIsValid: Check knot vector definition                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks the proper definition of  knot vectors.
     More specifically, it checks for the following:
       (1) end multiplicities must be p+1;
       (2) knots must be non-decreasing; and
       (3) the multiplicity of each internal  knot must not  exceed p;
     where p is the degree of the curve. A typical calling example is: 

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_STRING      rname;
       ...
       (define knt, get p and rname);
       ...
       N_KnotVectorIsValid(&knt,p,rname);
   

   ACCESS:
   
     knt   , input  ,  Knot vector
     p     , input  ,  Degree
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No eror
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_KnotVectorIsValid( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_STRING rname )
{
    NL_INDEX i, is, ie, m;
    NL_REAL *U;

    /* Convert to local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Check end knots */
    i = 0;

    while( i LT m AND U[i]EQ U[i + 1] )
        i++;

    if( i NEQ p )
    {
        N_ErrSet( NL_KNT_ERR, rname );
        return (1);
    }

    i = m;

    while( i GT 0 AND U[i]EQ U[i - 1] )
        i--;

    if( (m - i)NEQ p )
    {
        N_ErrSet( NL_KNT_ERR, rname );
        return (1);
    }

    /* Check internal knots */

    is = p;
    ie = p + 1;

    while( ie LE m - p )
    {
        if( U[is]GT U[ie] )
        {
            N_ErrSet( NL_KNT_ERR, rname );
            return (1);
        }

        i = ie;

        while( ie LT m - p AND U[ie]EQ U[ie + 1] )
            ie++;

        if( (ie - i + 1)GT p )
        {
            N_ErrSet( NL_KNT_ERR, rname );
            return (1);
        }

        is = ie;
        ie++;
    }

    return (0);
} /* end N_KnotVectorIsValid */

/*******************************************************************//**

   DESCRIPTION:

     This  routine allocates  memory to store members of a knot
     vector structure. Proper  error check is performed in case 
     memory allocation fails. A typical calling example is:

       NL_KNOTVECTOR  *kns;
       NL_STACKS      S;
       ...
       kns = N_AllocKnotVector(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     kns  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_KNOTVECTOR *N_AllocKnotVector( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocKnotVector");
    NL_KNOTVECTOR *kns;
    NL_KNTNODE *knd;

    /* Allocate memory for the structure */
    kns = (NL_KNOTVECTOR *)N_Malloc( sizeof( NL_KNOTVECTOR ) );

    if( kns EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */
    knd = (NL_KNTNODE *)N_Malloc( sizeof( NL_KNTNODE ) );

    if( knd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( kns );
        kns = NULL;
        return NULL;
    }

    knd->ptr = kns;
    knd->next = S->knt;
    S->knt = knd;

    return kns;
} /* end N_AllocKnotVector */


/*******************************************************************//**


   DESCRIPTION:

     This  routine  allocates  memory to store  a knot vector object.
     Proper error check is performed in case memory allocation fails.
     A typical calling example is:

       NL_KNOTVECTOR  *knt;
       NL_INDEX       m;
       ...
       (get m);
       ...
       knt = N_AllocKnotVectorAndArray(m,&S);


   ACCESS:
   
     m  , input  ,  Highest index in knot vector array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     knt  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_KNOTVECTOR *N_AllocKnotVectorAndArray( NL_INDEX m, NL_STACKS *S )
{
    NL_REAL *U;
    NL_KNOTVECTOR *knt;

    /* Allocate memory  */
    knt = N_AllocKnotVector( S );

    if( knt EQ NULL )
        return NULL;

    U = N_AllocReal1dArray( m, S );

    if( U EQ NULL )
        return NULL;

    /* Build object */
    N_KnotVectorFromRealArray( knt, U, m );

    return knt;
} /* end N_AllocKnotVectorAndArray */


/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 1-D array of knot vectors
     defined  by the same  index m. Proper  error check is performed in 
     case memory allocation fails. A typical calling example is:

       NL_KNOTVECTOR  **kn2;
       NL_INDEX       m, k;
       ...
       (get m and k);
       ...
       kn2 = N_Alloc1dArrayKnotVectors(m,k,&S);


   ACCESS:
   
     m   , input  ,  Highest index in knot vector arrays
     k   , input  ,  Highest  index  of knot  vector array  kn2[0],...,
                     kn2[k]; kn2[i], 0<=i<=k, is a pointer to  the i-th 
                     knot vector object.
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     kn2  : Pointer to array of knot vectors if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_KNOTVECTOR ** N_Alloc1dArrayKnotVectors( NL_INDEX m, NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc1dArrayKnotVectors");
    NL_INDEX i;
    NL_KNOTVECTOR ** kn2;
    NL_KN2NODE *k2d;

    /* Allocate memory for knot vector pointer array */
    kn2 = (NL_KNOTVECTOR ** )N_Malloc( (k + 1) * sizeof( NL_KNOTVECTOR * ) );

    if( kn2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Allocate memory for each knot vector in the array */
    for ( i = 0; i <= k; i++ )
    {
        kn2[i] = N_AllocKnotVectorAndArray( m, S );

        if( kn2[i]EQ NULL )
        {
            N_Free( kn2 );
            kn2 = NULL;
            return NULL;
        }
    }

    /* Put pointer on memory stack */
    k2d = (NL_KN2NODE *)N_Malloc( sizeof( NL_KN2NODE ) );

    if( k2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( kn2 );
        kn2 = NULL;
        return NULL;
    }

    k2d->ptr = kn2;
    k2d->next = S->kn2;
    S->kn2 = k2d;

    return kn2;
} /* end N_Alloc1dArrayKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates memory  to store a 1-D array of knot vector
     pointers. Proper error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_KNOTVECTOR  **kpa;
       NL_INDEX       k;
       ...
       (get k);
       ...
       kpa = N_Alloc1dArrayKnotVectPtrs(k,&S);


   ACCESS:
   
     k   , input  ,  Highest index of knot vector pointer array
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     kpa  : Pointer to array of knot vector pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_KNOTVECTOR ** N_Alloc1dArrayKnotVectPtrs( NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc1dArrayKnotVectPtrs");
    NL_KNOTVECTOR ** kpa;
    NL_KN2NODE *k2d;

    /* Allocate memory */
    kpa = (NL_KNOTVECTOR ** )N_Malloc( (k + 1) * sizeof( NL_KNOTVECTOR * ) );

    if( kpa EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */
    k2d = (NL_KN2NODE *)N_Malloc( sizeof( NL_KN2NODE ) );

    if( k2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( kpa );
        kpa = NULL;
        return NULL;
    }

    k2d->ptr = kpa;
    k2d->next = S->kn2;
    S->kn2 = k2d;

    return kpa;
} /* end N_Alloc1dArrayKnotVectPtrs */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine breaks a knot vector object down to its 
     components. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       m;
       NL_REAL        *U;
       ...
       N_KnotVectorGetKnots(&knt,&m,&U);


   ACCESS:
   
     knt , input  ,  Knot vector object
     m   , output ,  Highest index in U
     U   , output ,  Knot array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_KnotVectorGetKnots( NL_KNOTVECTOR *knt, NL_INDEX *m, NL_REAL ** U )
{
    *m = knt->m;
    *U = knt->U;
} /* end N_KnotVectorGetKnots */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes  proper pointer assignments to define
     a knot vector  object from the knots. Memory for the knot vector  
     structure and for the knots is allocated in the calling routine; 
     only the pointers are passed down. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_REAL        *U;
       NL_INDEX       m;
       ...
       (allocate memory for U);
       ...
       N_KnotVectorFromRealArray(&knt,U,m);


   ACCESS:
   
     knt , in/out ,  Knot vector
     U   , input  ,  Knots
     m   , input  ,  Highest index in U


   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_KnotVectorFromRealArray
 (NL_KNOTVECTOR *knt, // in : input KNOTVECTOR to build
  NL_REAL       *U,   // out: set knt->U = U, knot array
  NL_INDEX      m )   // out: set knt->m = m, highest index in U
{
    knt->m = m;
    knt->U = U;
} /* end N_KnotVectorFromRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets the index to complete knot vector 
     definition. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       m;
       ...
       N_SetKnotIndex(&knt,m);


   ACCESS:
   
     knt , in/out ,  Knot vector
     m   , input  ,  Highest index in knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SetKnotIndex( NL_KNOTVECTOR *knt, NL_INDEX m )
{
    knt->m = m;
} /* end N_SetKnotIndex */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores members of a 
     knot vector structure. Given  a knot vector  pointer, the  routine
     searches for the  pointer on the  memory  stack.  It it  is found,
     memory is deallocated. If not, the routine does nothing. A typical
     calling example is:

       NL_KNOTVECTOR  *knt;
       NL_STACKS      S;
       ...
       N_FreeKnotVector(knt,&S);


   ACCESS:
   
     knt , input  ,  Knot vector pointer 
     S   , input  ,  knt's


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeKnotVector( NL_KNOTVECTOR *knt, NL_STACKS *S )
{
    NL_KNTNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->knt NEQ NULL )
    {
        prev = S->knt;
        curr = S->knt;

        while( curr NEQ NULL AND curr->ptr NEQ knt )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->knt = NULL;
            }
            else /* More than one node */
            {
                S->knt = S->knt->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
        }
    }
} /* end N_FreeKnotVector */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prints knot vector data out to the standard 
     output device. The print is arranged as follows:

           m       --> highest index in knots

           <stop>   --> hit any key to continue

           u0       -->
           u1       -->
           .        -->
           .        --> knots
           .        -->
           um       -->

           <stop>   --> hit any key to continue

     A typical calling example is:

       NL_KNOTVECTOR  knt;
       ...
       N_KnotsPrint(&knt);


   ACCESS:
   
     knt , input ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_KnotsPrint( NL_KNOTVECTOR *knt )
{
    NL_INDEX i, m;
    NL_REAL *U;

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Print highest index in knot vector */
    N_FPRINTF( stdout, _T("%ld\n"), m );

    NL_PAUSE;

        /* Print knot values */
        for (i = 0; i <= m; i++)
        N_FPRINTF( stdout, _T("%18.16f\n"), U[i] );

    NL_PAUSE;

} /* end N_KnotsPrint */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine copies a knot vector into another knot vector.
     A typical calling example is:

       NL_KNOTVECTOR  *kna, *knb;
       NL_STACKS      S;
       ...
       (define kna, and allocate memory for knb or set knb = NULL);
       ...
       N_KnotsCopy(kna,&knb,&S);

     If knb = NULL, memory will be allocated  for knb.  Otherwise, it is  
     assumed that memory is already available.


   ACCESS:
   
     kna , input  ,  Knot vector to be copied
     knb , output ,  New knot vector
     S   , input  ,  knb's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_KnotsCopy( NL_KNOTVECTOR *kna, NL_KNOTVECTOR ** knb, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_KnotsCopy");
    NL_FLAG error;
    NL_INDEX i, ma, mb;
    NL_REAL *UA, *UB;

    /* Get local notation */
    N_KnotVectorGetKnots( kna, &ma, &UA );

    /* See if memory is needed */
    error = N_KnotsCheck( knb, ma, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_KnotVectorGetKnots( *knb, &mb, &UB );

    /* Copy knots */
    for ( i = 0; i <= ma; i++ )
        UB[i] = UA[i];

    return (0);
} /* end N_KnotsCopy */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine checks if memory is  needed to store a knot
     vector knt. If knt=NULL, memory is allocated. If not, the routine 
     checkes if enough memory is available. A typical calling example:

       NL_KNOTVECTOR  *knt;
       NL_INDEX       m;
       NL_STRING      rname;
       NL_STACKS      S;
       ...
       (get m and rname);
       ...
       N_KnotsCheck(&knt,m,rname,&S);


   ACCESS:
   
     knt   , in/out ,  Knot vector to be created
     m     , input  ,  Highest index of knots
     rname , input  ,  Routine name
     S     , input  ,  knt's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_KnotsCheck( NL_KNOTVECTOR ** knt, NL_INDEX m, NL_STRING rname, NL_STACKS *S )
{
    NL_INDEX mp;
    NL_REAL *UP;
    NL_KNOTVECTOR *knl;

    /* See if memory is needed */
    if( *knt EQ NULL )
    {
        knl = N_AllocKnotVectorAndArray( m, S );

        if( knl EQ NULL )
        {
            N_ErrSet( NL_MEM_ERR, rname );
            return (1);
        }

        *knt = knl;
    }
    else
    {
        N_KnotVectorGetKnots( *knt, &mp, &UP );

        if( mp LT m )
        {
            N_ErrSet( NL_STO_ERR, rname );
            return (1);
        }

        N_SetKnotIndex( *knt, m );
    }

    return (0);
} /* end N_KnotsCheck */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     Given a knot vector and a set of  parameters, this routine refines
     the knot vector so  that each Greville  span contains no more than
     one  parameter. The  Greville span is  obtained by  bracketing the
     Greville abscissa of degree p by Greville abscissae of degree p-1.
     The best use of this routine is in  shaping to localize the effect 
     of control points. A typical calling example is:
 
       NL_REAL    *U, *t, *s; 
       NL_INDEX   m, k, l, ds, de;
       NL_DEGREE  p;
       ...
       (get arrays U and t)
       ...
       N_KnotsRefine(U,m,p,t,k,ds,de,&s,&l);

     MEMORY FOR s IS ALLOCATED IN THE CALLING ROUTINE!

 
   ACCESS:
   
     U  , input  ,  Knot vector as in Nlib curves and surfaces
     m  , input  ,  Highest index in U
     p  , input  ,  Degree
     t  , input  ,  Random parameters, i.e. not ordered
     k  , input  ,  Highest index in t
     ds , input  ,  Start  derivative  constraint; 0,..,ds  derivatives
                    not to change at the start
     de , input  ,  End derivative  constraint; 0,..,de derivatives not 
                    to change at the end
     s  , output ,  Knots to be inserted so that each Greville span has
                    no more than one parameter
     l  , output ,  Highest index in s
     SG , input  ,  s' stack
 
 
   RETURN CODES:
 
     0 : No eror
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_KnotsRefine( NL_REAL *U, NL_INDEX m, NL_DEGREE p, NL_REAL *t, NL_INDEX k, NL_INDEX ds, NL_INDEX de, NL_REAL ** s, NL_INDEX *l, NL_STACKS *SG )
{
    NL_PRIVATE NL_REAL ktl = 1.0e-12;
    NL_FLAG found, error = NL_NO;
    NL_INDEX i, j, fs, n, nh, mh, lh, na, is, ik, lo, hi, mi, ii, jj, kp;
    NL_REAL *V, *z, *a, *so, *tp, sum, sv, ltl, tt, uu = 0.0, tl, tr, op1;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get memories */
    n = m - p - 1;
    is = -1;
    nh = n + k + k + ds + de + 2;
    mh = m + k + k + ds + de + 2;
    lh = k + k + ds + de + 2;
    ltl = ktl * (U[m] - U[0]);

    V = N_AllocReal1dArray( mh, &SL );

    if( V EQ NULL )
        NL_QUIT;

    z = N_AllocReal1dArray( nh, &SL );

    if( z EQ NULL )
        NL_QUIT;

    so = N_AllocReal1dArray( lh, SG );

    if( so EQ NULL )
        NL_QUIT;

    a = N_AllocReal1dArray( k, &SL );

    if( a EQ NULL )
        NL_QUIT;

    tp = N_AllocReal1dArray( k, &SL );

    if( tp EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= m; i++ )
        V[i] = U[i];

    /* Purge input parameters */
    kp = -1;
    tl = U[0] + ltl;
    tr = U[m] - ltl;

    for ( i = 0; i <= k; i++ )
    {
        if( t[i]GT tl AND t[i]LT tr )
            tp[++kp] = t[i];
    }

    /* Get Greville brackets */
    op1 = 1.0 / (p - 1.0);

    for ( i = 1; i <= n; i++ )
    {
        sum = 0.0;

        for ( j = i + 1; j <= i + p - 1; j++ )
            sum += V[j];

        z[i] = op1 * sum;
    }

    /* Check end constraints */
    tl = tr = tp[0];

    for ( i = 1; i <= kp; i++ )
    {
        if( tp[i]LT tl )
            tl = tp[i];

        if( tp[i]GT tr )
            tr = tp[i];
    }

    if( ds GT 0 )
    {
        while( tl LE z[ds + 1] )
        {
            uu = 0.5 *( V[p] + V[p + 1] );

            if( m + 1 GT mh )
            {
                error = N_Realloc1dRealArray( &V, mh, mh + mh, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                mh = mh + mh;
            }

            if( n + 1 GT nh )
            {
                error = N_Realloc1dRealArray( &z, nh, nh + nh, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                nh = nh + nh;
            }

            if( is + 1 GT lh )
            {
                error = N_Realloc1dRealArray( &so, lh, lh + lh, SG );

                if( error EQ NL_YES )
                    NL_OUT;

                lh = lh + lh;
            }

            for ( j = m; j >= p + 1; j-- )
                V[j + 1] = V[j];

            V[p + 1] = uu;
            so[is + 1] = uu;

            for ( j = n; j >= p; j-- )
                z[j + 1] = z[j];

            for ( i = 2; i <= p; i++ )
            {
                sum = 0.0;

                for ( j = i + 1; j <= i + p - 1; j++ )
                    sum += V[j];

                z[i] = op1 * sum;
            }

            n++;
            m++;
            is++;
        }
    }

    if( de GT 0 )
    {
        while( tr GE z[n - de] )
        {
            uu = 0.5 *( V[n] + V[n + 1] );

            if( m + 1 GT mh )
            {
                error = N_Realloc1dRealArray( &V, mh, mh + mh, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                mh = mh + mh;
            }

            if( n + 1 GT nh )
            {
                error = N_Realloc1dRealArray( &z, nh, nh + nh, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                nh = nh + nh;
            }

            if( is + 1 GT lh )
            {
                error = N_Realloc1dRealArray( &so, lh, lh + lh, SG );

                if( error EQ NL_YES )
                    NL_OUT;

                lh = lh + lh;
            }

            for ( j = m; j >= n + 1; j-- )
                V[j + 1] = V[j];

            V[n + 1] = uu;
            so[is + 1] = uu;

            z[n + 1] = z[n];

            for ( i = n - p + 2; i <= n; i++ )
            {
                sum = 0.0;

                for ( j = i + 1; j <= i + p - 1; j++ )
                    sum += V[j];

                z[i] = op1 * sum;
            }

            n++;
            m++;
            is++;
        }
    }

    /* For each bracket find the number of parameters present */
    fs = 1;
    found = NL_YES;

    while( found EQ NL_YES )
    {
        for ( i = fs; i <= n - 1; i++ )
        {
            found = NL_NO;

            /* Check span (z[i],z[i+1]) */

            if( fabs( z[i + 1] - z[i] )GT ltl )
            {
                /* If span is not degenerate, find parameters */

                na = -1;
                sum = 0.0;

                for ( j = 0; j <= kp; j++ )
                {
                    if( tp[j]GT( z[i] + ltl )AND tp[j]LT( z[i + 1] - ltl ) )
                    {
                        na++;
                        a[na] = tp[j];
                        sum += tp[j];
                    }
                }

                if( na GT 0 )
                {
                    /* If more than one parameter found, get the average */

                    N_ShellSortReal( a, na );

                    tt = sum / ((NL_REAL)na + 1);

                    /* Get the knot to be inserted */

                    sv = 0.0;

                    for ( j = i + 2; j <= i + p - 1; j++ )
                        sv += V[j];
                    uu = (p - 1.0) * tt - sv;

                    if( fabs( uu - tt )LT ltl )
                        uu = tt;

                    /* Find knot span parameter is in */

                    lo = p;
                    hi = m - p;
                    mi = (lo + hi) / 2;

                    if( uu EQ V[m - p] )
                    {
                        ik = n;
                    }
                    else
                    {
                        while( uu LT V[mi]OR uu GE V[mi + 1] )
                        {
                            if( uu LT V[mi] )
                                hi = mi;
                            else
                                lo = mi;
                            mi = (lo + hi) / 2;
                        }

                        ik = mi;
                    }

                    if( fabs( uu - V[ik + 1] )LT ltl )
                        ik++;

                    /* Insert a knot: (1) see if uu falls on a knot */

                    if( fabs( uu - V[ik] )LT ltl )
                    {
                        sum = 0.0;

                        for ( j = 1; j <= na; j++ )
                            sum += a[j];
                        tt = sum / na;
                        uu = (p - 1.0) * tt - sv;
                    }

                    /* Check memory */

                    if( m + 1 GT mh )
                    {
                        error = N_Realloc1dRealArray( &V, mh, mh + mh, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        mh = mh + mh;
                    }

                    if( n + 1 GT nh )
                    {
                        error = N_Realloc1dRealArray( &z, nh, nh + nh, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        nh = nh + nh;
                    }

                    if( is + 1 GT lh )
                    {
                        error = N_Realloc1dRealArray( &so, lh, lh + lh, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        lh = lh + lh;
                    }

                    /* Insert a knot: (2) insert new knot or its modification */

                    for ( j = m; j >= ik + 1; j-- )
                        V[j + 1] = V[j];

                    V[ik + 1] = uu;
                    so[is + 1] = uu;

                    /* Recompute Greville brackets */

                    for ( jj = n; jj >= ik; jj-- )
                        z[jj + 1] = z[jj];

                    for ( ii = ik - p + 2; ii <= ik; ii++ )
                    {
                        sum = 0.0;

                        for ( jj = ii + 1; jj <= ii + p - 1; jj++ )
                            sum += V[jj];

                        z[ii] = op1 * sum;
                    }

                    n++;
                    m++;
                    is++;
                    fs = i;
                    found = NL_YES;
                }
            }

            if( found EQ NL_YES )
                break;
        }

        if( found EQ NL_NO )
            break;
    }

    if( is GT 0 )
        N_ShellSortReal( so, is );

    *s = so;
    *l = is;

    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end N_KnotsRefine */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     Given a knot vector, this utility routine adds required number of
     knots to the knot  vector. It recursively finds the largest spans
     and adds the  midpoint to the  new knot vector until the required
     number of knots are reached. A typical calling example is:
 
       NL_REAL    *U, *s; 
       NL_INDEX   m, k;
       NL_DEGREE  p;
       NL_STACKS  SG;
       ...
       (get arrays U)
       ...
       N_KnotsAdd(U,m,p,k,&s,&SG);

     MEMORY FOR s IS ALLOCATED INSIDE THE ROUTINE!

 
   ACCESS:
   
     U  , input  ,  Knot vector as in Nlib curves and surfaces
     m  , input  ,  Highest index in U
     p  , input  ,  Degree
     k  , input  ,  NUMBER of knots to be added
     s  , output ,  Sorted array of new knots to be added
     SG , input  ,  s's memory stack
 
 
   RETURN CODES:
 
     0 : No eror
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_KnotsAdd( NL_REAL *U, NL_INDEX m, NL_DEGREE p, NL_INDEX k, NL_REAL ** s, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;
    NL_INDEX i, j = 0, kk, n;
    NL_REAL *V, *so, d, dm, vv;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get memory */
    V = N_AllocReal1dArray( m + k, &SL );

    if( V EQ NULL )
        NL_QUIT;

    so = N_AllocReal1dArray( k - 1, SG );

    if( so EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= m; i++ )
        V[i] = U[i];

    kk = k - 1;
    n = m - p - 1;

    /* Recursively add knots */
    while( kk GE 0 )
    {
        dm = 0.0;

        for ( i = p; i <= n; i++ )
        {
            if( V[i]NEQ V[i + 1] )
            {
                d = fabs( V[i + 1] - V[i] );

                if( d GT dm )
                {
                    dm = d;
                    j = i;
                }
            }
        }

        vv = 0.5 *( V[j] + V[j + 1] );

        for ( i = m; i >= j + 1; i-- )
            V[i + 1] = V[i];
        V[j + 1] = vv;
        so[kk] = vv;

        n++;
        m++;
        kk--;
    }

    N_ShellSortReal( so, k - 1 );

    *s = so;

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_KnotsAdd */

/*******************************************************************//**


   DESCRIPTION:

     This routine finds the knot span a given parameter is in. It is 
     assumed that the knot vector is clamped, i.e. the end knots are 
     repeated with multiplicity equals degree + 1. 
     When u > knt->U[m-p] return last  span index = m-p-1
     When u < knt->U[p]   return first span index = p
     else returns span index = spn 
       where knt->U[spn] <  u <= knt->U[spn+1]  for flg = NL_LEFT
       where knt->U[spn] <= u <  knt->U[spn+1]  for flg = NL_RIGHT

     A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_INDEX       spn;
       ... 
       (define knt, get p and u);
       ...
       N_BasisFindSpan(&knt,p,u,NL_LEFT,&spn);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     spn , output ,  Left index of span u is in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_BasisFindSpan
 (NL_KNOTVECTOR * knt,    /* in : knot vector                                      */
 NL_DEGREE        p,      /* in : degree                                           */
 NL_PARAMETER     u,      /* in : param value to classify                          */
 NL_FLAG          flg,    /* in : NL_LEFT = u[spn] <  u <= u[spn+1]                */
                          /*      NL_RIGHT= u[spn] <= u <  u[spn+1]                */
 NL_INDEX       * spn )   /* out: knot vector index of span left side containing u */
{
  NL_PRIVATE NL_STRING rname = _T("N_BasisFindSpan");
  NL_FLAG              error = NL_NO;
  NL_INDEX             low, high, mid, m, lastmid;
  NL_REAL            * U;
  NL_INDEX             iTemp;
  NL_REAL              dKnotTol = 1.0e-12; /* cbiTol: Just meant to avoid a zero-divide. */

  /* Get local notation */
  m = knt->m;  /* max U index        */
  U = knt->U;  /* vector of U values */

  /* Check parameter */

  /*   Allow this to be outside span domain
   * error = N_KnotVectorIsParamOutOfBounds(knt,u,rname);
   * if (error EQ NL_YES)
   * { NL_OUT; }
   */

  /* Initialize running indexes and switch to NL_LEFT or NL_RIGHT */
  low     = p;
  high    = m - p;
  mid     = (low + high) / 2;
  lastmid = mid;
    
  /* out of bounds cases */
  /*GWC: used to be if( u EQ U[m - p])  in the NL_LEFT case below - changed to shorten out of bounds cases for one extra equality check */
  /* When u is greater than or equal ivl[max] */
  if( u GE U[m - p] )   
    {
      iTemp = m - p - 1;
      /*
       * Here we put in a hack to allow things to work in the presence of
       * bad data.  We require U[*spn] < U[*spn+1], otherwise the evaluator
       * routines that call this will run into a divide-by-zero and fail,
       * and such failures will abort entire high-level operations.
       * The bad data in question is too many equal knots at the ends of
       * knot vectors.  For example, a cubic knot vector is supposed to have
       * four (degree+1) equal knots at each end, but we might see seven --
       * essentially a zero-length knot span.  That case is flagged in
       * AssertValid(), and will soon have a healer method, but at any rate,
       * let's not allow it to abort entire operations.  We can avoid the
       * problem and get a good evaluation by adjusting the index.  [B405]
       */

      /* Require U[ *spn ] < U[ *spn+1 ] */
      /*  degenerate spans: stop when iTemp = p */
      while ( U[ iTemp+1 ] LT U[ iTemp ] + dKnotTol  &&  iTemp > p )
        { iTemp--; }
      *spn = iTemp;
      NL_OUT;
    }

  /*GWC: used to be if( u EQ U[m - p]) in the NL_RIGHT case below - changed to shorten half the out of bounds cases for no extra cost */
  /* When u is less than or equal ivl[min] */
  if( u LE U[p] )
    {
      iTemp = p;
      /* Require U[ *spn ] < U[ *spn+1 ]; see note above. */
      /*  degenerate spans: stop when iTemp = m-p-1 */
      while ( U[ iTemp+1 ] LT U[ iTemp ] + dKnotTol  &&  iTemp < (m-p-1) )
        { iTemp++; }
      *spn = iTemp;
      NL_OUT;
    }

  /* switch between LEFT and RIGHT cases */
  switch( flg )
    {
      case NL_LEFT: /* u must be in [u[j],u[j+1]) i.e.  u[spn] < u <= u[spn+1] */
          while( u LT U[mid] OR u GE U[mid + 1] )
            {
              if( u LT U[mid] )
                  high = mid;
              else
                  low = mid;

              mid = (low + high) / 2;

              if( mid == lastmid )
                  break;
              lastmid = mid;
            }
          break;

      case NL_RIGHT: /* u must be in (u[j],u[j+1]] i.e.  u[spn] <= u < u[spn+1] */

          while( u LE U[mid] OR u GT U[mid + 1] )
            {
              if( u GT U[mid] )
                  low = mid;
              else
                  high = mid;

              mid = (low + high) / 2;

              if( mid == lastmid )
                  break;
              lastmid = mid;
            }
          break;

      default:

          NL_ERROR( NL_CAL_ERR );
    } /* end switch NL_LEFT or NL_RIGHT */

  *spn = mid;
  /* error = !(*spn >= p && *spn <= m-p-1) ;*/   /* GWC: why is this error condition removed? */
  EXIT:                                          /* as such: out of bounds u values are assigned silently to 1st and last spans */

  return (error);
} /* end N_BasisFindSpan */


/*******************************************************************//**


   DESCRIPTION:

    This routine  finds the knot span (reported as the largest index 
    of the knot bounding the low end of the span) containing a given  
    parameter. It computes how many times the given parameter  value  
    appears  in the  knot  vector  (multiplicity  is needed for many 
    routines such as knot  refinement).  If the param value is not a 
    knot  value the  output  multiplicity  value is  set to 0. It is 
    assumed that the knot  vector is clamped, i.e. the end knots are 
    repeated with multiplicities equal degree + 1.

     A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_INDEX       spn, mlt;
       ...
       (define knt, get u and p);
       ...
       N_BasisFindSpanAndMult(&knt,p,u,NL_LEFT,&spn,&mlt);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     spn , output ,  Left index of span that u is in
     mlt , output ,  u appears with multiplicity mlt in knt


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisFindSpanAndMult
 (NL_KNOTVECTOR *knt,  /* in : knot vector                              */
  NL_DEGREE      p,    /* in : curve degree                             */
  NL_PARAMETER   u,    /* in : parameter value                          */
  NL_FLAG        flg,  /* in : NL_LEFT = u is in interval [u[j],u[j+1]) */
                       /*      NL_RIGHT= u is in interval (u[j],u[j+1]] */
  NL_INDEX      *spn,  /* out: index of span containing u               */
  NL_INDEX      *mlt ) /* out: u appears with multiplicity mlt in KnotVector knt - 0 for not in Knt */
{
  NL_PRIVATE NL_STRING rname = _T("N_BasisFindSpanAndMult");
  NL_FLAG error = NL_NO;
  NL_INDEX k, s, m;
  NL_REAL *U;

  /* Get local notation */
  N_KnotVectorGetKnots( knt, &m, &U );

  /* Get knot span */
  error = N_BasisFindSpan( knt, p, u, flg, &k );

  if( error EQ NL_YES )
      NL_OUT;

  /* Get multiplicity of the parameter */
  *spn = k;
  s = 0;

  switch( flg )
  {
      case NL_LEFT:
          if( u EQ U[m - p] )
          {
              *mlt = p + 1;
              NL_OUT;
          }

          while( k GE 0 AND u EQ U[k--] )
              s++;
          break;

      case NL_RIGHT:
          if( u EQ U[p] )
          {
              *mlt = p + 1;
              NL_OUT;
          }

          k++;

          while( k LE m AND u EQ U[k++] )
              s++;
          break;

      default:

          NL_ERROR( NL_CAL_ERR );
  }

  *mlt = s;

  /* End NURBS and Exit */
  EXIT:

  return (error);
} /* end N_BasisFindSpanAndMult */


/*******************************************************************//**


   DESCRIPTION:

     This  routine  finds  the  number  of non-zero spans in a given 
     knot vector. It is assumed that the knot vector is clamped, i.e. 
     the end knots are  repeated with multiplicity equals degree + 1.
     A typical calling example is:
 
       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_INDEX       nsp;
       ...
       (define knt and get p);
       ...
       N_BasisGetSpanCount(&knt,p,&nsp);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     nsp , output ,  Number of non-zero spans


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BasisGetSpanCount( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_INDEX *nsp )
{
    NL_INDEX i, m;
    NL_REAL *U;

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Find number of spans */
    *nsp = 0;

    for ( i = p; i < m - p; i++ )
    {
        if( U[i] NEQ U[i + 1] )
            ( *nsp)++;
    }
} /* end N_BasisGetSpanCount */


/*******************************************************************//**


   DESCRIPTION:

     This routine  computes the global  maximum of a basis function. It 
     uses Newton iteration with the  start value obtained by bracketing 
     the root. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       i;
       NL_DEGREE      p;
       NL_REAL        tol, max, u;
       ...
       (define knt, get i, p and tol);
       ...
       N_BasisFindGlobalMax(&knt,i,p,tol,&max,&u);
 

   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline
     p   , input  ,  Degree 
     tol , input  ,  Tolerance for convergence
     max , output ,  Global maximum
     u   , output ,  Parameter where maximum is attained


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisFindGlobalMax( NL_KNOTVECTOR *knt, NL_INDEX i, NL_DEGREE p, NL_REAL tol, NL_REAL *max, NL_REAL *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisFindGlobalMax");
    NL_PRIVATE NL_INTEGER itl = 20;
    NL_PRIVATE NL_INTEGER nok = 10;

    NL_FLAG conv = NL_NO, error = NL_NO;
    NL_INDEX m, k, nos, s, it;
    NL_REAL *U, ND[3];
    NL_PARAMETER du, ul, ur, u0 = 0.0;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation and check index */
    N_KnotVectorGetKnots( knt, &m, &U );

    if( i LT 0 OR i GT m - p - 1 )
        NL_ERROR( NL_IND_ERR );

    /* Consider linear as special case */
    if( p EQ 1 )
    {
        *max = 1.0;
        *u = U[i + 1];
        NL_OUT;
    }

    /* Check is p-fold inner multiple knot exists */
    s = 1;

    for ( k = i + 1; k < i + p; k++ )
    {
        if( U[k] EQ U[k + 1] )
            s++;
    }

    if( s EQ p )
    {
        *max = 1.0;
        *u = U[i + 1];
        NL_OUT;
    }

    /* Get guess parameter */
    nos = p * nok;
    du = (U[i + p + 1] - U[i]) / nos;

    ur = U[i];
    ul = U[i];

    while( ur LT U[i + p + 1] )
    {
        ul = ur;
        ur = ur + du;

        if( ur GT U[i + p + 1] )
            ur = U[i + p + 1];

        error = N_BasisIDerivs( knt, i, p, ur, NL_LEFT, 1, ND );

        if( error EQ NL_YES )
            NL_OUT;

        if( ND[1]LT 0.0 )
            break;
    }

    /* Perform Newton interations till convergence reached */
    it = 0;

    while( it LT itl )
    {
        u0 = 0.5 *( ul + ur );

        /* Do Newton with guess parameter */

        k = 0;

        while( k LT itl )
        {
            error = N_BasisIDerivs( knt, i, p, u0, NL_LEFT, 2, ND );

            if( error EQ NL_YES )
                NL_OUT;

            if( fabs( ND[1] )LT tol AND ND[0]GT tol )
            {
                conv = NL_YES;
                break;
            }

            if( N_FloatOpIsBad( ND[1], ND[2], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            u0 = u0 - (ND[1] / ND[2]);

            if( u0 LE U[i]OR u0 GE U[i + p + 1] )
                break;

            k++;
        }

        if( conv EQ NL_YES )
            break;

        /* No convergence -> refine [ul,ur] and get a better guess */
        du = (ur - ul) / nok;
        ur = ul;

        for ( k = 1; k <= nok; k++ )
        {
            ul = ur;
            ur = ur + du;

            error = N_BasisIDerivs( knt, i, p, ur, NL_LEFT, 1, ND );

            if( error EQ NL_YES )
                NL_OUT;

            if( ND[1]LT 0.0 )
                break;
        }

        it++;
    }

    /* If no convergence, quit */
    if( it GE itl )
        NL_ERROR( NL_CON_ERR );

    /* Convergence reached -> get maximum */
    *max = ND[0];
    *u = u0;

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisFindGlobalMax */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the local minimums and maximums of a basis 
     function in each knot span. It computes the global maximum first
     followed  by computing  the minima  and maxima over each span. A 
     typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       i;
       NL_DEGREE      p;
       NL_REAL        tol, *min, *max, u;
       ...
       (define knt; get memory for min and max; get i, p and tol);
       ...
       N_BasisFindAllSpanMaxima(&knt,i,p,tol,min,max,&u);

     MEMORY FOR min AND max MUST BE ALLOCATED IN THE CALLING ROUTINE!  


   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline
     p   , input  ,  Degree 
     tol , input  ,  Tolerance for convergence
     min , output ,  Local minimuns; min[k] is the minimum value over
                     [U[i+k],U[i+k+1]]
     max , output ,  Local maximums; max[k] is the maximum value over
                     [U[i+k],U[i+k+1]]
     u   , output ,  Parameter where global maximum is attained


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisFindAllSpanMaxima( NL_KNOTVECTOR *knt, NL_INDEX i, NL_DEGREE p, NL_REAL tol, NL_REAL *min, NL_REAL *max, NL_PARAMETER *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisFindAllSpanMaxima");
    NL_FLAG error = NL_NO;
    NL_INDEX m, s, k, spn;
    NL_REAL *U, Nm, Nl, Nr;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation and check index */
    N_KnotVectorGetKnots( knt, &m, &U );

    if( i LT 0 OR i GT m - p - 1 )
        NL_ERROR( NL_IND_ERR );

    /* Consider linear as special case */
    if( p EQ 1 )
    {
        min[0] = 0.0;
        min[1] = 0.0;
        max[0] = 1.0;
        max[1] = 1.0;

        *u = U[i + 1];
        NL_OUT;
    }

    /* Check is p-fold inner multiple knot exists */
    s = 1;

    for ( k = i + 1; k < i + p; k++ )
    {
        if( U[k]EQ U[k + 1] )
            s++;
    }

    if( s EQ p )
    {
        for ( k = 0; k <= p; k++ )
            max[k] = 1.0;

        for ( k = 1; k < p; k++ )
            min[k] = 1.0;

        if( U[i]EQ U[i + 1] )
            min[0] = 1.0;
        else
            min[0] = 0.0;

        if( U[i + p]EQ U[i + p + 1] )
            min[p] = 1.0;
        else
            min[p] = 0.0;

        *u = U[i + 1];
        NL_OUT;
    }

    /* Get global maximum */
    error = N_BasisFindGlobalMax( knt, i, p, tol, &Nm, u );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get span index of global maximum */
    spn = -1;

    for ( k = i; k <= i + p; k++ )
    {
        if( *u GE U[k]AND *u LT U[k + 1] )
        {
            spn = k;
            break;
        }
    }

    if( spn EQ - 1 )
        NL_ERROR( NL_CON_ERR );

    /* Compute local extrema */
    for ( k = 0; k < spn - i; k++ )
    {
        error = N_BasisIEval( knt, i, p, U[i + k], NL_LEFT, &min[k] );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knt, i, p, U[i + k + 1], NL_LEFT, &max[k] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    max[spn - i] = Nm;

    error = N_BasisIEval( knt, i, p, U[spn], NL_LEFT, &Nl );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIEval( knt, i, p, U[spn + 1], NL_LEFT, &Nr );

    if( error EQ NL_YES )
        NL_OUT;

    if( Nl LT Nr )
        min[spn - i] = Nl;
    else
        min[spn - i] = Nr;

    for ( k = spn - i + 1; k <= p; k++ )
    {
        error = N_BasisIEval( knt, i, p, U[i + k + 1], NL_LEFT, &min[k] );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knt, i, p, U[i + k], NL_LEFT, &max[k] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisFindAllSpanMaxima */

/*******************************************************************//**


   DESCRIPTION:

     This routine returns all distinct knots of a  given  knot  vector,
     together with their multiplicities.  A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_REAL        *knots;
       NL_INDEX       n, *mults;
       NL_STACKS      SG;
       ... 
       (define knt);
       ...
       N_BasisGetKnotsAndMults(&knt,&knots,&mults,&n,&SG);

     Memory to store the knots and multiplicities are allocated in this
     routine.  Example:  if knt has the knots: 0,0,0,1,2,2,3,3,3 , then
     n = 3 , knots = 0,1,2,3 , and mults = 3,1,2,3


   ACCESS:
   
     knt   , input  ,  Knot vector
     knots , output ,  All distinct knot values
     mults , output ,  The multiplicities of each distinct knot
     n     , output ,  High index of arrays knots and mults
     SG    , input  ,  Memory stack for knots and mults


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisGetKnotsAndMults( NL_KNOTVECTOR *knt, NL_REAL ** knots, NL_INDEX ** mults, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisGetKnotsAndMults");
    NL_FLAG error = NL_NO;
    NL_INDEX ii, jj, nn, m, *muls;
    NL_REAL *kts, *U;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Count the distinct knots */
    nn = 0;

    for ( ii = 1; ii <= m; ii++ )
        if( U[ii]NEQ U[ii - 1] )
            nn += 1;

    /* Allocate memory */

    kts = N_AllocReal1dArray( nn, SG );
    muls = N_AllocInt1dArray( nn, SG );

    if( kts EQ NULL OR muls EQ NULL )
        NL_QUIT;

    *n = nn;
    *knots = kts;
    *mults = muls;

    /* Extract distinct knots and multiplicities */
    jj = 0;
    kts[0] = U[0];
    muls[0] = 1;

    for ( ii = 1; ii <= m; ii++ )
        if( U[ii]EQ U[ii - 1] )
            muls[jj] += 1;
        else
        {
            jj += 1;
            kts[jj] = U[ii];
            muls[jj] = 1;
        }

    if( jj NEQ nn )
        NL_ERROR( NL_IND_ERR );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisGetKnotsAndMults */


/*******************************************************************//**


   DESCRIPTION:

     This routine  scales a knot vector to a given  interval. That  is, 
     U[0] <= U[1] <= ... <= U[m] is  mapped  to  a <= ... <= U[p+1]' <= 
     ... <= U[m-p-1]' <= ... <= b, where I=[a,b] is the given interval. 
     A typical calling example is: 

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_INTERVAL    I;
       ...
       (define knt; get p and I);
       ...
       N_BasisReparam(&knt,p,I);

     The knot vector is rescaled IN-PLACE, i.e. the  original knots are
     destroyed.


   ACCESS:
   
     knt , in/out ,  Knot vector
     p   , input  ,  Degree
     I   , input  ,  Parameter interval


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BasisReparam( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_INTERVAL I )
{
    NL_INDEX i, m;
    NL_REAL *U, fac, ul, ur, u0;

    /* Get local notation */
    N_KnotVectorGetKnots( knt, &m, &U );
    N_IntervalGetData( &I, &ul, &ur );

    /* Compute new knots */
    if( U[0]NEQ ul OR U[m]NEQ ur )
    {
        u0 = U[0];
        fac = (ur - ul) / (U[m] - U[0]);

        for ( i = 0; i <= p; i++ )
            U[i] = ul;

        for ( i = p + 1; i <= m - p - 1; i++ )
            U[i] = ul + fac * (U[i] - u0);

        for ( i = m - p; i <= m; i++ )
            U[i] = ur;
    }
} /* end N_BasisReparam */

/*******************************************************************//**


   DESCRIPTION:

     This routine  merges a  set of knot  vectors, i.e. it computes a
     set of new vectors to be used to  refine the  input knot vectors
     to make them  compatible using a knotTol of 1.0e-08. A set  of  knot  vectors is said to be
     compatible if all has the same knots. A typical calling  example
     is as follows:

       NL_KNOTVECTOR  **knt, **knx;
       NL_INDEX       k;
       NL_STACKS      SK;
       ....
       (define set of knot  vectors: knt[i]  is a pointer to the i-th
        knot vector);
       ...
       N_GetCompatibleKnotArray(knt, k, &knx, &SK);

     knx[i]->U[j] represents the j-th knot of the i-th knot vector to
     be used  to  refine  knt[i]. MEMORY  TO  STORE THE  OUTPUT  KNOT 
     VECTORS IS ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     knt    , input  ,  Array of knot vectors
     k      , input  ,  Highest index in knt
     knx    , output ,  Arrays of new knots to be inserted into knt knotvectors
     SK     , input  ,  knx's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_GetCompatibleKnotArray
  ( NL_KNOTVECTOR ** knt, 
    NL_INDEX k, 
    NL_KNOTVECTOR *** knx, 
    NL_STACKS *SK )
{
    /* pass the call along */
    return( N_GetCompatibleKnotVectorToTol(knt, k, 1.0e-08, knx, SK) ) ;

} /* end N_GetCompatibleKnotArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine merges a set of  knot vectors, same as 
     N_GetCompatibleKnotVectorToTol() with the addition of making
     the knot vector parameter ranges all equal.  N_GetCompatibleKnotVectorToTol()
     assumes the knot vectors share the same parameter range.

     It computes a set
     of new vectors to be used to  refine the input knot vectors to make 
     them have knots at the same parameter values. For example, given

       U_1 = { 0.,0.,0.,.2,.5,.5,1.,1.,1. }
       U_2 = { 0.,0.,0.,0.,.2,.2,.4,.4,.4,.7,.7,1.,1.,1.,1.}
       U_3 = { 0.,0.,0.,.3,.3,.3,.8,.8,1.,1.,1.}

     the routine computes

       X_1 = { .3,.4,.7,.8 }
       X_2 = { .3,.5,.8 }
       X_3 = { .2,.4,.5,.7 }

     to be inserted into U_1, U_2 and U_3 to have knots at the parameter
     values  .2, .3, .4, .5, .7 and .8.  Multiplicities of knots are NOT
     carried over to other knot vectors. A typical calling example:

       NL_KNOTVECTOR  **knt, **knx;
       NL_INDEX       k;
       NL_REAL        tol;
       NL_STACKS      SK;
       ....
       (define set of knot  vectors: knt[i]  is a pointer to the i-th
        knot vector, and get tol);
       ...
       N_GetCompatibleKnotVector(knt,k,tol,&knx,&SK);

     knx[i]->U[j] represents the j-th knot of the i-th knot vector to be
     used to refine  knt[i]. MEMORY TO  STORE THE OUTPUT KNOT VECTORS IS 
     ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     knt , in/out ,  Array  of  knot  vectors. MAY  BE  RESCALED  IF NOT 
                     DEFINED NL_OVER THE SAME SPAN!
     k   , input  ,  Highest index in knt
     tol , input  ,  Tolerance used to check if two knots are the same
     knx , output ,  Array of new knot vectors
     SK  , input  ,  knx's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_GetCompatibleKnotVector
  ( NL_KNOTVECTOR ** knt, 
    NL_INDEX k, 
    NL_REAL tol, 
    NL_KNOTVECTOR *** knx, 
    NL_STACKS *SK )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_GetCompatibleKnotVector"); */

    NL_FLAG error = NL_NO;

    NL_INDEX m, r, s, *deg ;

    NL_REAL *U, ul, ur;

    NL_INTERVAL I;

    NL_DEGREE p;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if defined over the same span */

    deg = N_AllocInt1dArray( k, &SL );

    if( deg EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( knt[0], &m, &U );

    ul = U[0];
    ur = U[m];

    for ( r = 0; r <= k; r++ )
    {
        N_KnotVectorGetKnots( knt[r], &m, &U );

        s = 0;

        while( s LT m AND U[s]EQ U[s + 1] )
            s++;
        deg[r] = s;

        if( U[0]LT ul )
            ul = U[0];

        if( U[m]GT ur )
            ur = U[m];
    }

    N_CreateInterval( &I, ul, ur );

    for ( r = 0; r <= k; r++ )
    {
        N_KnotVectorGetKnots( knt[r], &m, &U );

        if( U[0] NEQ ul OR U[m] NEQ ur)
        {
            p = (NL_DEGREE)deg[r];
            N_BasisReparam( knt[r], p, I );
        }
    }

    /* now pass the call along */
    error = N_GetCompatibleKnotVectorToTol(knt, k, tol, knx, SK) ;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_GetCompatibleKnotVector */

/*******************************************************************//**

   DESCRIPTION:

     This routine  merges a  set of knot  vectors, i.e. it computes a
     set of new vectors to be used to  refine the  input knot vectors
     to make them  compatible. A set  of  knot  vectors is said to be
     compatible  if  all  has the  same  knots. This  version  uses a 
     tolerance to  avoid  multiple  knots that are very  close to one 
     another. That is, assume the following knots:

       U1={ ... ,0.40200000001, 0.40200000001, 0.40200000001, ... }
         NL_PRIVATE  NL_STRING  rname = "N_GetCompatibleKnotVectorToTol");
       U2={ ... ,0.40200000000, ............., ............., ... }
       U3={ ... ,0.40200000002, 0.40200000002, ............., ... }

     that is, the knot 0.402 appears  with multiplicities  3, 1 and 2 
     in U1, U2 and  U3, respectively, and  differ in  the 11th digit. 
     Using a tolerance, say,  10^{-7}, after merging, the knot in U1, 
     U2 and U3 looks like this:

       U1={ ... ,0.40200000001, 0.40200000001, 0.40200000001, ... }
       U2={ ... ,0.40200000001, ............., ............., ... }
       U3={ ... ,0.40200000001, 0.40200000001, ............., ... }

     that is, the original  knots are slightly moved  around to avoid
     tiny  differences usually  due to  round off  errors. The  knots 
     to be inserted are:

       X1={ ..., ............., ............., ... }
       X2={ ..., 0.40200000001, 0.40200000001, ... }
       X3={ ..., 0.40200000001, ............., ... }

     A typical calling example is:

       NL_KNOTVECTOR  **knt, **knx;
       NL_INDEX       k;
       NL_REAL        tol;
       NL_STACKS      SK;
       ....
       (define set of knot vectors and get the tolerance);
       ...
       N_GetCompatibleKnotVectorToTol(knt,k,tol,&knx,&SK);

     knx[i]->U[j] represents the j-th knot of the i-th knot vector to
     be used  to  refine  knt[i]. MEMORY  TO  STORE THE  OUTPUT  KNOT 
     VECTORS IS ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     knt , in/out ,  Array of knot vectors
     k   , input  ,  Highest index in knt
     tol , input  ,  Tolerance
     knx , output ,  Array of new knots to be inserted into the input knot vectors to make them compatible
     SK  , input  ,  knx's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_GetCompatibleKnotVectorToTol
  ( NL_KNOTVECTOR ** knt, 
    NL_INDEX k, 
    NL_REAL tol, 
    NL_KNOTVECTOR *** knx, 
    NL_STACKS *SK )
{

    NL_PRIVATE NL_STRING rname = _T("N_GetCompatibleKnotVectorToTol");

    NL_FLAG error = NL_NO;

    NL_INDEX *knotIndices, *mlt, r, s, m, mh, numAve, knotIndex;

    NL_REAL *U, us, uAve, uTol, gapSize ;

    NL_KNOTVECTOR ** kna;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get memory */
    /* get s = total number of knots in all knotvectors */
    s = 0;
    
    for ( r = 0; r <= k; r++ )
    {
        N_KnotVectorGetKnots( knt[r], &m, &U );
        s += m;
    }

    knotIndices = N_AllocInt1dArray( k, &SL );

    if( knotIndices EQ NULL )
        NL_QUIT;

    mlt = N_AllocInt1dArray( k, &SL );

    if( mlt EQ NULL )
        NL_QUIT;

    /* allocate new knot vectors each with memory for total number of knots */
    kna = N_Alloc1dArrayKnotVectors( s, k, SK );

    if( kna EQ NULL )
        NL_QUIT;

    /* Initialize */
    /* set each new knot vector KnotCount to 0 - knots get added in later */
    for ( r = 0; r <= k; r++ )
    {
        knotIndices[r] = 0;
        N_SetKnotIndex( kna[r], -1 );
    }

    /* General algorithm: this works its way up all knot vectors,
     * each iteration of the outside loop finds the next lowest
     * knot in any of the lists, and saves it, with the multiplicities
     * it has in of each knot vector.
     */

    /* Loop while not finished - that is while any knotIndices[i] knotIndexValue is */
    /*  less than the associated max KnotIndexValue stored in knt[i] knotvectors    */
    while( ST_notdone( knotIndices, knt, k ) )
    {
        /* Get smallest next knot value in any list */

        N_KnotVectorGetKnots( knt[0], &m, &U );

        if( knotIndices[0]GT m )
            NL_ERROR( NL_INP_ERR );

        /* next knot value and gap size in knotvector knt[0] */
        us = U[knotIndices[0]];
        knotIndex = knotIndices[0] + 1;

        /* Use LE not EQ: allow for slightly out-of-order knot values. [B168] */
        while( knotIndex LE m AND U[knotIndex] LE U[knotIndices[0]] )
            knotIndex++;
        gapSize = (knotIndex LE m) ? U[knotIndex] - U[knotIndices[0]] : -1.0 ;

        for ( r = 1; r <= k; r++ )
        {
            N_KnotVectorGetKnots( knt[r], &m, &U );

            if( knotIndices[r]GT m )
                NL_ERROR( NL_INP_ERR );

            /* save smallest next knot value of all knotvector knt[i] */
            if( U[knotIndices[r]]LT us )
            {
               us = U[knotIndices[r]];

               /* get associated gap size */
               knotIndex = knotIndices[r] + 1;
               /* Use LE not EQ: allow for slightly out-of-order knot values. [B168] */
               while( knotIndex LE m AND U[knotIndex] LE U[knotIndices[r]] )
                   knotIndex++;
               gapSize = (knotIndex LE m) ? U[knotIndex] - U[knotIndices[r]] : -1.0 ;
            }
        }

        /* watch out for gaps smaller than tol by reducing tol when needed to less than the next gap size */
        uTol = (gapSize LT tol AND gapSize NEQ -1.0) ? 0.99999 * gapSize : tol ;

        /* Compute multiplicities */

        for ( r = 0; r <= k; r++ )
        {
            N_KnotVectorGetKnots( knt[r], &m, &U );

            if( fabs( U[knotIndices[r]] - us) LT uTol)
            {
                s = knotIndices[r];

                /* Use LE not EQ: allow for slightly out-of-order knot values. [B168] */
                while( knotIndices[r]LT m AND U[knotIndices[r] + 1] LE U[knotIndices[r]] )
                    knotIndices[r]++;

                mlt[r] = knotIndices[r] - s + 1;
            }
            else
            {
                mlt[r] = 0;
                knotIndices[r]--;
            }
        }

        /* Adjust knots */

        /* Take the average value of the knots within uTol of the current smallest next knot value */
        /* 'us' currently holds the smallest next knot value */
        numAve = 0;
        uAve = 0.0;

        for ( r = 0; r <= k; r++ )
        {
            N_KnotVectorGetKnots( knt[r], &m, &U );
            
            if( mlt[r] GT 0 AND fabs( U[knotIndices[r]] - us ) LT uTol ) 
            {
                uAve += U[knotIndices[r]];
                numAve++;
            }
        }

        us = uAve / numAve; /* set 'us' to the knot value average */

        /* move all next KnotValues within uTol of the average to the average */
        for ( r = 0; r <= k; r++ )
        {
            N_KnotVectorGetKnots( knt[r], &m, &U );

            if( fabs( U[knotIndices[r]] - us )LT uTol )  /* check gets same knots as in average because */
            {                                            /* KnotAverage is bounded by KnotMin and KnotMin+uTol */
                for ( s = 1; s <= mlt[r]; s++ )
                    U[knotIndices[r] - mlt[r] + s] = us;
            }
        }

        /* Get the highest multiplicity */

        mh = mlt[0];

        for ( r = 1; r <= k; r++ )
        {
            if( mlt[r]GT mh )
                mh = mlt[r];
        }

        /* Load needed new knots into output knot vector */

        for ( r = 0; r <= k; r++ )
        {
            N_KnotVectorGetKnots( kna[r], &m, &U );

            for ( s = mlt[r] + 1; s <= mh; s++ )
                U[++m] = us;

            N_SetKnotIndex( kna[r], m );
            knotIndices[r]++;
        }
    } /* End of while */

    *knx = kna;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_GetCompatibleKnotVectorToTol */

/*******************************************************************//**

   DESCRIPTION:

     Figure out if any of the respective knot index values in knotIndices 
     are less than the maximum knot index value stored in the Knt knotvectors.
     If so, return NL_TRUE,
     else   return NL_FALSE.

     This is used by N_GetCompatibleKnotVectorToTol to figure out
       when all the knots in all the knot vectors being made compatible
       have been processed.

   ***********************************************************************/
NL_BOOLEAN ST_notdone
  ( NL_INDEX *knotIndices,   /* in : a knot index value to compare with each knot vector in knt */
    NL_KNOTVECTOR ** knt,    /* in : array of knot vectors */
    NL_INDEX k )             /* in : max index value in the arrays knotIndices and knt */
{

    NL_FLAG flg = NL_FALSE;

    NL_INDEX j, m;

    NL_REAL *U;

    for ( j = 0; j <= k; j++ )
    {
        N_KnotVectorGetKnots( knt[j], &m, &U );

        if( knotIndices[j]LT m )
        {
            flg = NL_TRUE;
            break;
        }
    }

    return flg;

} /* end ST_notdone */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the nodes of a given knot vector. A  typical 
     calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   *t;
       ... 
       (define knt, get p and allocate memory for t);
       ...
       N_BasisFindIndexNodeArray(&knt,p,t);

     Memory to store the nodes must be allocated in the calling routine
     to hold up to m-p elements, where  m is the highest  index in knt.
     IT IS ASSUMED THAT THE KNOT NL_VECTOR IS CLAMPED, I.E.  THE NL_END KNOTS
     ARE REPEATED WITH MULTIPLICITY (p+1).


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     t   , output ,  All nodes


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BasisFindIndexNodeArray( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER *t )
{

    NL_INDEX i, j, n, m;

    NL_REAL *U, sum, fact;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_KnotVectorGetKnots( knt, &m, &U );
    n = m - p - 1;

    /* Compute the nodes */

    t[0] = U[0];
    t[n] = U[m];
    fact = 1.0 / p;

    for ( i = 1; i <= n - 1; i++ )
    {
        sum = 0.0;

        for ( j = 1; j <= p; j++ )
            sum += U[i + j];

        t[i] = fact * sum;
    }

    /* End NURBS and Exit */

    N_EndNurbs( &S );
} /* end N_BasisFindIndexNodeArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes all non-vanishing basis  functions given a
     parameter value. It is assumed that the required memory to store
     the  basis  functions is  allocated in  the calling routine. The 
     highest index is  N[p], where  p is  the degree  of the curve. A
     typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_REAL        *N;
       NL_INDEX       spn;
       ...
       (define knt, get p, u, and allocate memory for N);
       ...
       N_BasisEval(&knt,p,u,NL_LEFT,N,&spn);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     N   , output ,  All  basis  functions  stored  in N[0],...,N[p].
                     MEMORY MUST BE ALLOCATED IN THE CALLING ROUTINE!
     spn , output ,  Left index of span u is in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisEval( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER u, NL_FLAG flg, NL_REAL *N, NL_INDEX *spn )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisEval");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k, l, m;

    NL_REAL *U, left[NL_MAXDEG + 2], right[NL_MAXDEG + 2], saved, delta, temp;

    /* Divide by zero?  Note that the denominator is the sum of
     * the two things being multiplied by it, in the following
     * two statements, so the quotients in parentheses can never
     * be greater than 1.0.  Therefore, the only way for any floating-
     * point problem is if the delta is exactly zero, which can
     * happen only if there are more than degree+1 consecutive
     * equal knots (which would probably be caught before here).
     * So we essentially just have to guard against the denominator
     * being exactly zero ... but use something very tiny, to be safe.
     */
    NL_REAL dZeroTol = 1e-24;

    /* Get local notation */

    m = knt->m;
    U = knt->U;

    /* Check parameter */

    /*  allow parameter to evaluate outside domain
    if ( u LT U[0]  OR  u GT U[m] )
    {
      N_ErrSet(NL_PAR_ERR,rname);
      error = NL_YES;
      NL_OUT;
    } */

    /* Special cases for end values */

    if( u EQ U[p] )
    {
        N[0] = 1.0;
        *spn = p;

        for ( k = 1; k <= p; k++ )
            N[k] = 0.0;
        NL_OUT;
    }

    if( u EQ U[m - p] )
    {
        N[p] = 1.0;
        *spn = m - p - 1;

        for ( k = 0; k < p; k++ )
            N[k] = 0.0;
        NL_OUT;
    }

    /* Find the knot span u is in */

    N_BasisFindSpan( knt, p, u, flg, &i );
    *spn = i;

    /* Compute the non-vanishing B-splines */

    N[0] = 1.0;

    for ( k = 1; k <= p; k++ )
    {
        left[k] = u - U[i + 1 - k];
        right[k] = U[i + k] - u;
        saved = 0.0;

        for ( l = 0; l < k; l++ )
        {
            delta = right[l + 1] + left[k - l];

            if( delta < dZeroTol )
            {
                NL_ERROR( NL_NUM_ERR );
            }
            temp = N[l];

            /* Note: keep the divisions inside the parentheses:
             * dividing N[l] by the knot difference can be huge.
             */
            N[l] = saved + temp * (right[l + 1] / delta);
            saved = temp * (left[k - l] / delta);
        }
        N[k] = saved;
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_BasisEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes all non-vanishing  basis functions and their
     derivatives for one given parameter value. It  is assumed  that the 
     required memory to  store the basis functions and their derivatives
     is  allocated  in  the  calling  routine. The  highest indexes  are 
     ND[d][p], where d is the  highest derivative  required and p is the 
     degree of the curve. A typical calling example is:

       NL_KNOTVECTOR  knt;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_INDEX       der, spn;
       NL_REAL        **ND;
       ...
       (define knt, get p, u, der and allocate memory for ND);
       ...
       N_BasisDerivs(&knt,p,u,NL_LEFT,der,ND,&spn);


   ACCESS:
   
     knt , input  ,  Knot vector
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES REQUIRED) 
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES REQUIRED) 
     der , input  ,  Highest derivative required 
     ND  , output ,  All basis functions and their derivatives. ND[k][i] 
                     is  the  k-th  derivative  of  the  basis  function 
                     N[j-p+i], where  u is  in {u[j],u[j+1]}. MEMORY FOR 
                       NL_PRIVATE  NL_STRING  rname = "N_BasisDerivs");
ND MUST BE ALLOCATED IN THE CALLING ROUTINE!
     spn , output ,  Left index of span u is in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisDerivs( NL_KNOTVECTOR *knt, NL_DEGREE p, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_REAL ** ND, NL_INDEX *spn )
{

    NL_PRIVATE NL_STRING rname = _T("N_BasisDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, r, s1, s2, rk, pk, j1, j2, mder;

    NL_REAL *U, *left, *right, ** ndu, ** a, saved, delta, d;

    NL_STACKS S;

    /* Divide by zero?  Note that the denominator is the sum of
     * the two things being multiplied by it, in the following
     * two statements, so the quotients in parentheses can never
     * be greater than 1.0.  Therefore, the only way for any floating-
     * point problem is if the delta is exactly zero, which can
     * happen only if there are more than degree+1 consecutive
     * equal knots (which would probably be caught before here).
     * So we essentially just have to guard against the denominator
     * being exactly zero ... but use something very tiny, to be safe.
     */
    NL_REAL dZeroTol = 1e-24;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_KnotVectorGetKnots( knt, &k, &U );

    /* Find the knot span u is in */

    error = N_BasisFindSpan( knt, p, u, flg, &i );

    if( error EQ NL_YES )
        NL_OUT;
    *spn = i;

    /* Get maximum derivative index and set zero derivatives */

    mder = NL_MIN( p, der );

    for ( k = p + 1; k <= der; k++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            ND[k][j] = 0.0;
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

    ndu[0][0] = 1.0;

    for ( j = 1; j <= p; j++ )
    {
        left[j] = u - U[i + 1 - j];
        right[j] = U[i + j] - u;
        saved = 0.0;

        for ( r = 0; r < j; r++ )
        {
            /* if( N_FloatOpIsBad(ndu[r][j-1],ndu[j][r],NL_DIVISION) )  NL_ERROR(NL_NUM_ERR); */

            delta = right[r + 1] + left[j - r];
            ndu[j][r] = delta;

            if( delta < dZeroTol )
            {
                NL_ERROR( NL_NUM_ERR );
            }
            /*
             * Note: keep the divisions inside the parentheses: dividing
             * ndu[][] by the knot difference can be huge.
             */
            ndu[r][j] = saved + ndu[r][j - 1] * (right[r + 1] / delta);
            saved = ndu[r][j - 1] * (left[j - r] / delta);
        }
        ndu[j][j] = saved;
    }

    /* Load the basis functions */

    for ( j = 0; j <= p; j++ )
        ND[0][j] = ndu[j][p];

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
                j1 = 1;
            else
                j1 = -rk;

            if( (r - 1)LE pk )
                j2 = k - 1;
            else
                j2 = p - r;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if( r LE pk )
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ND[k][r] = d;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors */

    r = p;

    for ( k = 1; k <= mder; k++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            ND[k][j] *= r;
        }
        r *= (p - k);
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine computes  one basis function at a given parameter
     value. It is assumed that the knot vector is clamped, i.e., end 
     knots are  repeated  with  multiplicity = degree + 1. A typical
     calling example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       i;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_REAL        N;
       ...
       (define knt, get i, p and u);
       ...
       N_BasisIEval(&knt,i,p,u,NL_LEFT,&N);


   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline (0<=i<=n)
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     N   , output ,  Basis function computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisIEval( NL_KNOTVECTOR *knt, NL_INDEX i, NL_DEGREE p, NL_PARAMETER u, NL_FLAG flg, NL_REAL *N )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisIEval");

    NL_FLAG error = NL_NO;

    NL_INDEX j, k, n, m;

    NL_REAL *U, *NA, UL, UR, saved, temp;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_KnotVectorGetKnots( knt, &m, &U );

    n = m - p - 1;

    /* Check parameter and index */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT n )
        NL_ERROR( NL_IND_ERR );

    /* Special cases for end values */

    if( u EQ U[p] )
    {
        if( i EQ 0 )
            *N = 1.0;
        else
            *N = 0.0;
        NL_OUT;
    }

    if( u EQ U[m - p] )
    {
        if( i EQ n )
            *N = 1.0;
        else
            *N = 0.0;

        NL_OUT;
    }

    /* Get memory */

    NA = N_AllocReal1dArray( p, &S );

    if( NA EQ NULL )
        NL_QUIT;

    /* Compute degree zero B-splines */
    switch( flg )
    {
        case NL_LEFT:
            if( u LT U[i]OR u GE U[i + p + 1] )
            {
                *N = 0.0;
                NL_OUT;
            }

            for ( j = 0; j <= p; j++ )
            {
                if( u GE U[i + j]AND u LT U[i + j + 1] )
                    NA[j] = 1.0;
                else
                    NA[j] = 0.0;
            }
            break;

        case NL_RIGHT:
            if( u LE U[i]OR u GT U[i + p + 1] )
            {
                *N = 0.0;
                NL_OUT;
            }

            for ( j = 0; j <= p; j++ )
            {
                if( u GT U[i + j]AND u LE U[i + j + 1] )
                    NA[j] = 1.0;
                else
                    NA[j] = 0.0;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Compute the i-th B-spline using a triangular array */

    for ( k = 1; k <= p; k++ )
    {
        if( NA[0]EQ 0.0 )
            saved = 0.0;
        else
            saved = ((u - U[i]) * NA[0]) / (U[i + k] - U[i]);

        for ( j = 0; j < p - k + 1; j++ )
        {
            UR = U[i + j + k + 1];
            UL = U[i + j + 1];

            if( NA[j + 1]EQ 0.0 )
            {
                NA[j] = saved;
                saved = 0.0;
            }
            else
            {
                temp = NA[j + 1] / (UR - UL);
                NA[j] = saved + (UR - u) * temp;
                saved = (u - UL) * temp;
            }
        }
    }

    *N = NA[0];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes one  basis  function and its  derivatives
     at a given parameter value. It is  assumed that the knot vector 
     is  clamped, i.e., end  knots are  repeated with multiplicity = 
     degree + 1. Memory  for the B-spline and derivative values must
     be  allocated in the calling routine. Maximum index is ND[der],
     where der is the highest derivative required. A typical calling
     example is:

       NL_KNOTVECTOR  knt;
       NL_INDEX       i;
       NL_DEGREE      p;
       NL_PARAMETER   u;
       NL_INDEX       der;
       NL_REAL        *ND;
       ...
       (define knt, get i, p, u, der, and allocate memory for ND);
       ...
       N_BasisIDerivs(&knt,i,p,u,NL_LEFT,der,ND);


   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline (0<=i<=n)
     p   , input  ,  Degree 
     u   , input  ,  Parameter value
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                              (NL_RIGHT NL_DERIVATIVE REQUIRED) 
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT NL_DERIVATIVE REQUIRED)
     der , input  ,  Maximum derivative required 
     ND  , output ,  Basis function and  derivatives computed at u.
                     MEMORY   MUST  BE  ALLOCATED  IN  THE  CALLING 
                     ROUTINE.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BasisIDerivs( NL_KNOTVECTOR *knt, NL_INDEX i, NL_DEGREE p, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_REAL *ND )
{
    NL_PRIVATE NL_STRING rname = _T("N_BasisIDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX j, k, l, n, m, mder;

    NL_REAL ** nt, *nd, *U, UL, UR, saved, temp;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_KnotVectorGetKnots( knt, &m, &U );

    n = m - p - 1;

    /* Check parameter and index */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT n )
        NL_ERROR( NL_IND_ERR );

    /* Allocate memory */

    nt = N_AllocReal2dArray( p, p, &S );

    if( nt EQ NULL )
        NL_QUIT;

    nd = N_AllocReal1dArray( p, &S );

    if( nd EQ NULL )
        NL_QUIT;

    /* Compute the degree zero B-splines */

    switch( flg )
    {
        case NL_LEFT:
            if( u EQ U[m - p]AND i GE n - p )
            {
                for ( j = 0; j <= p; j++ )
                {
                    if( u GT U[i + j]AND u LE U[i + j + 1] )
                    {
                        nt[j][0] = 1.0;
                    }
                    else
                    {
                        nt[j][0] = 0.0;
                    }
                }
            }
            else
            {
                if( u LT U[i]OR u GE U[i + p + 1] )
                {
                    for ( j = 0; j <= der; j++ )
                        ND[j] = 0.0;
                    NL_OUT;
                }

                for ( j = 0; j <= p; j++ )
                {
                    if( u GE U[i + j]AND u LT U[i + j + 1] )
                    {
                        nt[j][0] = 1.0;
                    }
                    else
                    {
                        nt[j][0] = 0.0;
                    }
                }
            }
            break;

        case NL_RIGHT:
            if( u EQ U[p]AND i LE p )
            {
                for ( j = 0; j <= p; j++ )
                {
                    if( u GE U[i + j]AND u LT U[i + j + 1] )
                    {
                        nt[j][0] = 1.0;
                    }
                    else
                    {
                        nt[j][0] = 0.0;
                    }
                }
            }
            else
            {
                if( u LE U[i]OR u GT U[i + p + 1] )
                {
                    for ( j = 0; j <= der; j++ )
                        ND[j] = 0.0;
                    NL_OUT;
                }

                for ( j = 0; j <= p; j++ )
                {
                    if( u GT U[i + j]AND u LE U[i + j + 1] )
                    {
                        nt[j][0] = 1.0;
                    }
                    else
                    {
                        nt[j][0] = 0.0;
                    }
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Compute the full triangular array */

    for ( k = 1; k <= p; k++ )
    {
        if( nt[0][k - 1]EQ 0.0 )
            saved = 0.0;
        else
            saved = ((u - U[i]) * nt[0][k - 1]) / (U[i + k] - U[i]);

        for ( j = 0; j < p - k + 1; j++ )
        {
            UR = U[i + j + k + 1];
            UL = U[i + j + 1];

            if( nt[j + 1][k - 1]EQ 0.0 )
            {
                nt[j][k] = saved;
                saved = 0.0;
            }
            else
            {
                temp = nt[j + 1][k - 1] / (UR - UL);
                nt[j][k] = saved + (UR - u) * temp;
                saved = (u - UL) * temp;
            }
        }
    }

    /* Compute derivatives */

    ND[0] = nt[0][p];

    mder = NL_MIN( p, der );

    for ( k = p + 1; k <= der; k++ )
        ND[k] = 0.0;

    for ( k = 1; k <= mder; k++ )
    {
        /* Load the appropriate column into the derivative array */

        for ( j = 0; j <= k; j++ )
            nd[j] = nt[j][p - k];

        /* Compute the triangular table of width = k */

        for ( l = 1; l <= k; l++ )
        {
            if( nd[0]EQ 0.0 )
                saved = 0.0;
            else
                saved = nd[0] / (U[i + p - k + l] - U[i]);

            for ( j = 0; j < k - l + 1; j++ )
            {
                UR = U[p - k + l + i + j + 1];
                UL = U[i + j + 1];

                if( nd[j + 1]EQ 0.0 )
                {
                    nd[j] = ((NL_REAL)p - (NL_REAL)k + l) * saved;
                    saved = 0.0;
                }
                else
                {
                    temp = nd[j + 1] / (UR - UL);
                    nd[j] = ((NL_REAL)p - (NL_REAL)k + l) * (saved - temp);
                    saved = temp;
                }
            }
        }
        ND[k] = nd[0];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_BasisIDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes new knots to make two knot
     vectors  compatible in the following sense. Given  two knot vectors 
     R and S, they are first scaled  to the unit span. Then  the routine 
     finds all r-knots not present in S, and all  s-knots not present in 
     R. These knots are then used to refine  R and S. After  refinement, 
     they will  have the same  internal  knots with  possibly  different  
     multiplicities. An example follows.

        R = { 0, 0, 0, 0, 0.3, 0.5 0.5, 0.7, 1, 1, 1, 1 }
        S = { 0, 0, 0, 0.2, 0.2, 0.5, 0.6, 1, 1, 1 }
       
     are refined to obtain

        R' = { 0, 0, 0, 0, 0.2, 0.3, 0.5 0.5, 0.6, 0.7, 1, 1, 1, 1 }
        S' = { 0, 0, 0, 0.2, 0.2, 0.3, 0.5, 0.6, 0.7, 1, 1, 1 }

     using the output vectors
   
        X = { 0.2, 0.6 }
        Y = { 0.3, 0.7 }

     to be  inserted  into  R  and  S, respectively. A  typical  calling 
     example is:

       NL_KNOTVECTOR  knr, kns, kxr, kxs;
       NL_DEGREE      p, q;
       ...
       (define knr, kns; get p and q; allocate memory for kxr and kxs);
       ...
       N_MakeKnotsCompatible(&knr,&kns,p,q,&kxr,&kxs);

     IT  IS  ASSUMED  THAT  MEMORY FOR kxr AND kxs  IS  ALLOCATED IN THE 
     CALLING  ROUTINE. THE  INPUT KNOT  VECTORS ARE  SCALED TO  THE UNIT 
     SPAN, I.E. THE ORIGINAL  KNOTS ARE DESTROYED. HOWEVER, THE NL_CURVE OR
     NL_SURFACE, DEFINED BY  THESE KNOT VECTORS,  WILL  NOT  CHANGE  EITHER 
     PARAMETRICALLY OR GEOMETRICALLY.
     

   ACCESS:
   
     knr , in/out ,  Knot vector (WILL BE RESCALED)
     kns , in/out ,  Knot vector (WILL BE RESCALED)
     p   , input  ,  Degree of knr
     q   , input  ,  Degree of kns
     kxr , output ,  Knot vector of new knots to be inserted into knr
     kxs , output ,  Knot vector of new knots to be inserted into kns


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_MakeKnotsCompatible( NL_KNOTVECTOR *knr, NL_KNOTVECTOR *kns, NL_DEGREE p, NL_DEGREE q, NL_KNOTVECTOR *kxr, NL_KNOTVECTOR *kxs )
{

    NL_INDEX i, j, mr, ms, mxr, mxs, nr, ns;

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
            N_BasisReparam( kns, q, I );
        }
        else
        {
            N_CreateInterval( &I, S[0], S[ms] );
            N_BasisReparam( knr, p, I );
        }
    }

    nr = mr - p - 1;
    ns = ms - q - 1;
    mxr = -1;
    mxs = -1;
    i = p + 1;
    j = q + 1;

    /* Find knots to be inserted */

    while( i LE nr OR j LE ns )
    {
        if( fabs( R[i] - S[j] )LT NL_PTOL )
        {
            while( i LE nr AND R[i]EQ R[i + 1] )
                i++;

            while( j LE ns AND S[j]EQ S[j + 1] )
                j++;

            i++;
            j++;
        }

        if( R[i]LT S[j] )
        {
            while( i LE nr AND R[i]EQ R[i + 1] )
                i++;

            XS[++mxs] = R[i];
            i++;
        }

        if( R[i]GT S[j] )
        {
            while( j LE ns AND S[j]EQ S[j + 1] )
                j++;

            XR[++mxr] = S[j];
            j++;
        }
    }

    /* Define output knot vectors */

    N_SetKnotIndex( kxr, mxr );
    N_SetKnotIndex( kxs, mxs );
} /* end N_MakeKnotsCompatible */

