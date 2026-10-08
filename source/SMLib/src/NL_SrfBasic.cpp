// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***************************************************************************/
/* SrfBasic.c : Basic Function Definitions that act on NL_SURFACE objects  */
/***************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>


/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if (n+p+1) equals r, and (m+q+1) equals
     s, i.e. it  checks if  the number of  control points, the  degree
     and the number of knots are properly related in both  directions.
     It also checks if all  control points  have the same rationality.
     A typical callig example is:

       NL_SURFACE  sur;
       NL_STRING   rname;
       ...
       (define sur and get rname);
       ...
       N_SrfCountsAreValid(&sur,rname);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfCountsAreValid( NL_SURFACE *sur, NL_STRING rname )
{
    NL_FLAG oldrat, newrat, error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, w;

    NL_CPOINT ** Pw;

    /* Convert to local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* Check definition and degrees */

    if( (n + p + 1)NEQ r OR( m + q + 1 )NEQ s )
        NL_ERROR( NL_SUR_ERR );

    if( p GT NL_DMAX OR q GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Check for consistency */

    N_CPtGetW( Pw[0][0], &w );

    if( w EQ NL_NOW )
        oldrat = 0;
    else
        oldrat = 1;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtGetW( Pw[i][j], &w );

            if( w EQ NL_NOW )
                newrat = 0;
            else
                newrat = 1;

            if( newrat NEQ oldrat )
                NL_ERROR( NL_SUR_ERR );
        }
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfCountsAreValid */

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks whether all the surface weights are 
     within the  range <NL_WMIN,NL_WMAX>. These weight limits are set in
     "globals.h". A typical calling example is:

       NL_SURFACE  sur;
       NL_STRING   rname;
       ...
       (define sur and get rname);
       ...
       N_SrfWeightsAreValid(&sur,rname);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* NL_FLAG  N_SrfWeightsAreValid */
NL_FLAG N_SrfWeightsAreValid( NL_SURFACE *sur, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL w;

    NL_CPOINT ** Pw;

    /* Convert to local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check surface weights */

    if( N_IsSrfRat( sur ) )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
            {
                N_CPtGetW( Pw[i][j], &w );

                if( w LT NL_WMIN OR w GT NL_WMAX )
                {
                    N_ErrSet( NL_WEI_ERR, rname );
                    error = NL_YES;
                    break;
                }
            }

            if( error EQ NL_YES )
                break;
        }
    }

    return (error);
} /* end N_SrfWeightsAreValid */


/*******************************************************************//**


   DESCRIPTION:

     This error routine performs a complete surface check, i.e. it 
     checks the input data for:
       (1) surface definition constants,
       (2) surface weights, and
       (3) the knot vectors.
     Other error routines are used to check  each type of error. A
     typical calling example is:
 
       NL_SURFACE  sur;
       NL_STRING   rname;
       ...
       (define sur and get rname);
       ...
       N_SrfIsValid(&sur,rname);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* NL_FLAG  N_SrfIsValid */
NL_FLAG N_SrfIsValid( NL_SURFACE *sur, const TCHAR * rname )
{
    NL_FLAG error;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    /* Convert to local notation */

    N_SrfGetKnotVectors( sur, &knu, &knv );
    N_SrfGetDegrees( sur, &p, &q );

    /* Check surface definition */

    error = N_SrfCountsAreValid( sur, (NL_STRING) rname );

    if( error EQ NL_YES )
        return (1);

    /* Check surface weights */

    error = N_SrfWeightsAreValid( sur, (NL_STRING) rname );

    if( error EQ NL_YES )
        return (1);

    /* Check knot vectors */

    error = N_KnotVectorIsValid( knu, p, (NL_STRING) rname );

    if( error EQ NL_YES )
        return (1);

    error = N_KnotVectorIsValid( knv, q, (NL_STRING) rname );

    if( error EQ NL_YES )
        return (1);

    /* Exit */

    return (0);
} /* end N_SrfIsValid */


/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if a surface  structure has  sufficient
     memory to store given control points and knots. A typical calling
     example is:

       NL_SURFACE  sur;
       NL_INDEX    np, mp, rk, sk;
       NL_STRING   rname;
       ...
       (define sur, get np, mp, rk, sk and rname);
       ...
       N_SrfIsSized(&sur,np,mp,rk,sk,rname);


   ACCESS:
   
     sur    , input ,  NURBS surface
     np,mp  , input ,  Expected highest indexes in control point array
     rk,sk  , input ,  Expected highest indexes in knot vector arrays
     rname  , input ,  Routine name error is checked in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfIsSized( NL_SURFACE *sur, NL_INDEX np, NL_INDEX mp, NL_INDEX rk, NL_INDEX sk, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s;

    /* Get local notation */

    N_SrfGetArraySizes( sur, &n, &m, &r, &s );

    /* Check storage */

    if( n LT np OR m LT mp OR r LT rk OR s LT sk )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_SrfIsSized */

/*******************************************************************//**


  DESCRIPTION:

     This error routine checks if any two consecutive control points
     within the interior of a surface are equal. Only the first indexes of
     equal (within Tolerance) control points returned ( 0 = all are different).
     Use Tol = NL_MTOL for strict equality.

       NL_SURFACE   Sur;
       ...
       (define Sur);
       ...
       N_SrfHasEqualCPts(&Sur, Tol, &Nu, &Nv);


   ACCESS:   
     Sur   , input  ,  NURBS surface
     Tol   , input  .  Distance tolerance between adjacent control points

   RETURN CODES:

     0 : No control points are equal (within Tol)
     1 : Control Points Pw[Nu][Nv-1] and Pw[Nu][Nv] are equal
     2 ; Control points Pw[Nu-1][Nv] and Pw[Nu][Nv] are equal
   ***********************************************************************/
/* NL_FLAG  N_SrfHasEqualCPts */
NL_FLAG N_SrfHasEqualCPts( NL_SURFACE *sur, NL_REAL Tol, NL_INDEX *Nu, NL_INDEX *Nv )
{
    NL_CPOINT Cp0, Cp1, Rw;
    NL_REAL d;
    NL_INDEX i, j, m, n;
    n = sur->net->n;
    m = sur->net->m;

    for ( i = 0; i <= n; i++ )
    {
        Cp1 = sur->net->Pw[i][0];

        for ( j = 1; j <= m; j++ )
        {
            Cp0 = Cp1;
            Cp1 = sur->net->Pw[i][j];
            N_Diff2CPts( Cp0, Cp1, &Rw );
            N_CPtMagnitude( Rw, &d );

            if( d < Tol )
            {
                *Nu = i;
                *Nv = j;
                return (1);
            }
        }
    }

    for ( j = 0; j <= m; j++ )
    {
        Cp1 = sur->net->Pw[0][j];

        for ( i = 1; i <= n; i++ )
        {
            Cp0 = Cp1;
            Cp1 = sur->net->Pw[i][j];
            N_Diff2CPts( Cp0, Cp1, &Rw );
            N_CPtMagnitude( Rw, &d );

            if( d < Tol )
            {
                *Nu = i;
                *Nv = j;
                return (2);
            }
        }
    }

    return (0);
} /* end N_SrfHasEqualCPts */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a surface
     structure. Proper  error check is  performed in case memory 
     allocation fails. A typical calling example is:

       NL_SURFACE  *sus;
       NL_STACKS   S;
       ...
       sus = N_AllocSrf(&S);


   ACCESS:
   
     S , input  ,  Memory stack pointer


   RETURN CODES:

     sus  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/
/* NL_SURFACE  *N_AllocSrf */
NL_SURFACE *N_AllocSrf( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocSrf");

    NL_SURFACE *sus;

    NL_SURNODE *sud;

    /* Allocate memory for the structure */

    sus = (NL_SURFACE *)N_Malloc( sizeof( NL_SURFACE ) );

    if( sus EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    sud = (NL_SURNODE *)N_Malloc( sizeof( NL_SURNODE ) );

    if( sud EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( sus );
        sus = NULL;
        return NULL;
    }

    sud->ptr = sus;
    sud->next = S->sur;
    S->sur = sud;

    /* Exit */

    return sus;
} /* end N_AllocSrf */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to  store a surface defined by 
     the usual parameters <n,m,p,q,r,s>. Proper error  checks are 
     performed in case memory allocations fail. A typical calling
     example is:

       NL_SURFACE  *sur;
       NL_INDEX    n, m, r, s;
       NL_DEGREE   p, q;
       NL_STACKS   S;
       ...
       (get n, m, p, q, r and s);
       ...
       sur = N_AllocSrfAndArrays(n,m,p,q,r,s,&S);


   ACCESS:
   
     n,m , input  ,  Highest indexes in control point array
     p,q , input  ,  Degrees in u- and v-directions
     r,s , input  ,  Highest indexes in knot vector arrays
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     sur  : Pointer to surface if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_SURFACE  *N_AllocSrfAndArrays */
NL_SURFACE *N_AllocSrfAndArrays( NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s, NL_STACKS *S )
{
    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    NL_SURFACE *sur;

    /* Allocate memory */

    net = N_AllocCNetAndArrays( n, m, S );

    if( net EQ NULL )
        return NULL;

    knu = N_AllocKnotVectorAndArray( r, S );

    if( knu EQ NULL )
        return NULL;

    knv = N_AllocKnotVectorAndArray( s, S );

    if( knv EQ NULL )
        return NULL;

    sur = N_AllocSrf( S );

    if( sur EQ NULL )
        return NULL;

    /* Build surface structure */

    N_SrfFromCNetAndKnotVectors( sur, net, p, q, knu, knv );

    /* Exit */

    return sur;
} /* end N_AllocSrfAndArrays */


/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory  to store an array of surface 
     pointers. Proper  error check  is performed  in case  memory 
     allocations fail.  A typical calling example is:

       NL_SURFACE  **sua;
       NL_INDEX    k;
       ...
       (get k);
       ...
       sua = N_AllocArraySrfPtrs(k,&S);


   ACCESS:
   
     k  , input  ,  Highest index in surface pointer array
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     sua  : Pointer to array of surface pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SURFACE ** N_AllocArraySrfPtrs( NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocArraySrfPtrs");

    NL_SURFACE ** sua;

    NL_SU2NODE *s2d;

    /* Allocate memory */

    sua = (NL_SURFACE ** )N_Malloc( (k + 1) * sizeof( NL_SURFACE * ) );

    if( sua EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointers on memory stacks */

    s2d = (NL_SU2NODE *)N_Malloc( sizeof( NL_SU2NODE ) );

    if( s2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( sua );
        sua = NULL;
        return NULL;
    }

    s2d->ptr = sua;
    s2d->next = S->su2;
    S->su2 = s2d;

    /* Exit */

    return sua;
} /* end N_AllocArraySrfPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory  to store a  2-D array of surface
     pointers. Proper  error  check  is  performed  in  case   memory  
     allocation fails. A typical calling example is:

       NL_SURFACE  ***ss3;
       NL_INDEX    k, l;
       NL_STACKS   S;
       ...
       (get k and l);
       ...
       ss3 = N_Alloc2dArraySrfPtrs(k,l,&S);


   ACCESS:
   
     k,l  , input  ,  Highest  indexes  in  surface  pointer   array;
                      ss3[i][j]  is a pointer to the (i,j)th surface. 
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     ss3  : Pointer to 2-D array of surface pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SURFACE *** N_Alloc2dArraySrfPtrs( NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc2dArraySrfPtrs");

    NL_INDEX i, j;

    NL_SURFACE *** ss3, ** ss2;

    NL_SU2NODE *s2d;

    NL_SU3NODE *s3d;

    /* Allocate memory for surface pointer arrays */

    ss3 = (NL_SURFACE *** )N_Malloc( (k + 1) * sizeof( NL_SURFACE ** ) );

    if( ss3 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    ss2 = (NL_SURFACE ** )N_Malloc( (k + 1) * (l + 1) * sizeof( NL_SURFACE * ) );

    if( ss2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( ss3 );
        ss3 = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    j = 0;

    for ( i = 0; i <= k; i++ )
    {
        ss3[i] = &ss2[j];
        j = j + l + 1;
    }

    /* Put pointers on memory stacks */

    s2d = (NL_SU2NODE *)N_Malloc( sizeof( NL_SU2NODE ) );

    if( s2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( ss3 );
        ss3 = NULL;
        N_Free( ss2 );
        ss2 = NULL;
        return NULL;
    }

    s3d = (NL_SU3NODE *)N_Malloc( sizeof( NL_SU3NODE ) );

    if( s3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( ss3 );
        ss3 = NULL;
        N_Free( ss2 );
        ss2 = NULL;
        N_Free( s2d );
        s2d = NULL;
        return NULL;
    }

    s2d->ptr = ss2;
    s2d->next = S->su2;
    S->su2 = s2d;

    s3d->ptr = ss3;
    s3d->next = S->su3;
    S->su3 = s3d;

    /* Exit */

    return ss3;
} /* end N_Alloc2dArraySrfPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory  to store a  2-D array of surfaces
     each defined by the same  set of parameters <n,m,p,q,r,s>. Proper 
     error  check is  performed in  case  memory  allocations  fail. A 
     typical calling example is:

       NL_SURFACE  ***su3;
       NL_INDEX    n, m, r, s, k, l;
       NL_DEGREE   p, q; 
       NL_STACKS   S;
       ...
       (get n, m, p, q, r, s, k and l);
       ...
       su3 = N_Alloc2dArraySrfPtrsParameters(n,m,p,q,r,s,k,l,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in control point arrays
     p,q  , input  ,  Degrees in u- and v-directions
     r,s  , input  ,  Highest indexes in knot vector arrays
     k,l  , input  ,  Highest indexes in surface array  su3[0][0],...,
                      su3[k][l]; su3[i][j] is a pointer to the (i,j)th
                      surface. 
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     su3  : Pointer to 2-D array of surfaces if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SURFACE *** N_Alloc2dArraySrfPtrsParameters( NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc2dArraySrfPtrsParameters");

    NL_INDEX i, j;

    NL_SURFACE *** su3, ** su2;

    NL_SU2NODE *s2d;

    NL_SU3NODE *s3d;

    /* Allocate memory for surface pointer arrays */

    su3 = (NL_SURFACE *** )N_Malloc( (k + 1) * sizeof( NL_SURFACE ** ) );

    if( su3 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    su2 = (NL_SURFACE ** )N_Malloc( (k + 1) * (l + 1) * sizeof( NL_SURFACE * ) );

    if( su2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( su3 );
        su3 = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    j = 0;

    for ( i = 0; i <= k; i++ )
    {
        su3[i] = &su2[j];
        j = j + l + 1;
    }

    /* Allocate memory for the individual surfaces */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            su3[i][j] = N_AllocSrfAndArrays( n, m, p, q, r, s, S );

            if( su3[i][j]EQ NULL )
            {
                N_Free( su3 );
                su3 = NULL;
                N_Free( su2 );
                su2 = NULL;
                return NULL;
            }
        }
    }

    /* Put pointers on memory stacks */

    s2d = (NL_SU2NODE *)N_Malloc( sizeof( NL_SU2NODE ) );

    if( s2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( su3 );
        su3 = NULL;
        N_Free( su2 );
        su2 = NULL;
        return NULL;
    }

    s3d = (NL_SU3NODE *)N_Malloc( sizeof( NL_SU3NODE ) );

    if( s3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( su3 );
        su3 = NULL;
        N_Free( su2 );
        su2 = NULL;
        N_Free( s2d );
        s2d = NULL;
        return NULL;
    }

    s2d->ptr = su2;
    s2d->next = S->su2;
    S->su2 = s2d;

    s3d->ptr = su3;
    s3d->next = S->su3;
    S->su3 = s3d;

    /* Exit */

    return su3;
} /* end N_Alloc2dArraySrfPtrsParameters */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store an array  of  surface point-
     ers.  It also allocates the  surface  structures  pointed to by the
     pointers. Proper error check is performed in case memory allocation
     fails. A typical calling example is:

       NL_SURFACE  **surs;
       NL_INDEX    k;
       NL_STACKS   S;
       ...
       (get k);
       ...
       surs = N_AllocArraySrfPtrsInit(k,NL_YES,&S);


   ACCESS:
   
     k   , input  ,  Highest index of surface pointer array
     flg , input  ,  Flag:
                      NL_YES: initialize the surface objects to null
                      NL_NO : do not initialize the surface objects
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     surs : Pointer to array of surface pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SURFACE ** N_AllocArraySrfPtrsInit( NL_INDEX k, NL_FLAG flg, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocArraySrfPtrsInit");

    NL_SURFACE ** surs;

    NL_INDEX ii;

    NL_SU2NODE *u2d;

    /* Allocate memory */

    surs = (NL_SURFACE ** )N_Malloc( (k + 1) * sizeof( NL_SURFACE * ) );

    if( surs EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    u2d = (NL_SU2NODE *)N_Malloc( sizeof( NL_SU2NODE ) );

    if( u2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( surs );
        surs = NULL;
        return NULL;
    }

    u2d->ptr = surs;
    u2d->next = S->su2;
    S->su2 = u2d;

    /* Allocate the surface structures */

    for ( ii = 0; ii <= k; ii++ )
    {
        surs[ii] = N_AllocSrf( S );

        if( surs[ii]EQ NULL )
            return NULL;

        if( flg EQ NL_YES )
            N_SrfInitArrays( surs[ii] );
    }

    /* Exit */

    return surs;
} /* end N_AllocArraySrfPtrsInit */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine deallocates memory that stores surface data,
     i.e.   control  net   structure,  control   points,  knot  vector 
     structures  and knots. IT DOES NOT  DEALLOCATE MEMORY THAT STORES 
     THE NL_SURFACE STRUCTURE ITSELF. A typical calling example is:

       NL_SURFACE  *sur;
       NL_STACKS   S;
       ...
       N_FreeSrf(sur,&S);


   ACCESS:
   
     sur , input  ,  Surface pointer
     S   , input  ,  sur's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeSrf */
NL_VOID N_FreeSrf( NL_SURFACE *sur, NL_STACKS *S )
{
    NL_DEGREE p, q;

    NL_REAL *U, *V;

    NL_CPOINT ** Pw;

    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    if( sur == NULL )
        return;

    /* Get locals */
    N_SrfGetNetAndKnotVectors( sur, &net, &p, &q, &knu, &knv );

    if( net == NULL || knu == NULL || knv == NULL )
        return;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Kill surface constituents */
    N_FreeCNet( net, S );
    N_FreeCPt2dArray( Pw, S );
    N_FreeKnotVector( knu, S );
    N_FreeReal1dArray( U, S );
    N_FreeKnotVector( knv, S );
    N_FreeReal1dArray( V, S );
} /* end N_FreeSrf */

/*******************************************************************//**
   DESCRIPTION:

     This utility  routine deallocates  memory that stores members of a 
     surface structure. Given  a surface  pointer, the  routine
     searches for the  pointer on the  memory  stack.  It it  is found,
     memory is deallocated. If not, nothing is done.
     A typical calling example is:

       NL_SURFACE    *sur;
       NL_STACKS      S;
       ...
       N_FreeCrvStruct(sus, &S);


   ACCESS:
   
     sur , input  ,  NL_SURFACE pointer 
     S   , input  ,  NL_STACKS


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeSrfStruct( NL_SURFACE *sur, NL_STACKS *S )
{
    NL_SURNODE *prev, *surr;

    /* Traverse memory stack to find pointer */

    if( S->sur NEQ NULL )
    {
        prev = S->sur;
        surr = S->sur;

        while( surr NEQ NULL AND surr->ptr NEQ sur )
        {
            prev = surr;
            surr = surr->next;
        }

        if( prev EQ surr )           /* First node         */
        {
            if( surr->next EQ NULL ) /* One node only      */
            {
                S->sur = NULL;
            }
            else /* More than one node */
            {
                S->sur = S->sur->next;
            }
        }
        else                /* Not the first node */
        if( surr NEQ NULL ) /* Node found         */
        {
            prev->next = surr->next;
        }

        if( surr NEQ NULL ) /* Release memory     */
        {
            N_Free( surr->ptr );
            N_Free( surr );
        }
    }
} /* End N_FreeSrfStruct */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper pointer assignments to define
     a surface  object  from  control net  and knot  vector objects.
     Memory for the various structures  is allocated  in the calling
     routine; only the  pointers are passed down. A  typical calling
     example is:
 
       NL_SURFACE     sur;
       NL_CNET        net;
       NL_DEGREE      p, q;
       NL_KNOTVECTOR  knu, knv;
       ...
       (define net, knu and knv);
       ...
       N_SrfFromCNetAndKnotVectors(&sur,&net,p,q,&knu,&knv);


   ACCESS:
   
     sur     , in/out ,  NURBS surface
     net     , input  ,  Control net
     p,q     , input  ,  Degrees
     knu,knv , input  ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfFromCNetAndKnotVectors */
NL_VOID N_SrfFromCNetAndKnotVectors( NL_SURFACE *sur, NL_CNET *net, NL_DEGREE p, NL_DEGREE q, NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv )
{
    sur->net = net;
    sur->p = p;
    sur->q = q;
    sur->knu = knu;
    sur->knv = knv;
} /* end N_SrfFromCNetAndKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This routine  generates a  surface object given control points 
     and knots. It  allocates  memory to store the control net  and 
     knot vector objects, and makes  proper pointer  assignments to
     create the surface object. All other memory allocation is done
     in  the calling  routine; only  pointers  are  passed  down. A 
     typical calling example is:

       NL_SURFACE  sur;
       NL_CPOINT   **Pw;
       NL_INDEX    n, m, r, s;
       NL_DEGREE   p, q;
       NL_REAL     *U, *V;
       NL_STACKS   S;
       ...
       (allocate memory for Pw, U and V);
       ...
       N_SrfFromCPtsAndKnots(&sur,Pw,n,m,p,q,U,V,r,s,&S);


   ACCESS:
   
     sur  , in/out ,  NURBS surface
     Pw   , input  ,  Control points
     n,m  , input  ,  Highest indexes in Pw
     p,q  , input  ,  Degrees
     U,V  , input  ,  Knots
     r,s  , input  ,  Highest indexes in U and V
     S    , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfFromCPtsAndKnots */
NL_FLAG N_SrfFromCPtsAndKnots( NL_SURFACE *sur, NL_CPOINT ** Pw, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_REAL *U, NL_REAL *V, NL_INDEX r, NL_INDEX s, NL_STACKS *S )
{
    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    /* Allocate memory for control net and knot vector structures */

    net = N_AllocCNet( S );

    if( net EQ NULL )
        return (1);

    knu = N_AllocKnotVector( S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVector( S );

    if( knv EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_CNetFromCPts( net, Pw, n, m );
    N_KnotVectorFromRealArray( knu, U, r );
    N_KnotVectorFromRealArray( knv, V, s );
    N_SrfFromCNetAndKnotVectors( sur, net, p, q, knu, knv );

    /* Exit */

    return (0);
} /* end N_SrfFromCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a  surface  object given the coordinates
     of the  control points and the knots. It  allocates  memory  to 
     store the control net and knot vector objects, and makes proper 
     pointer  assignments  to  create the  surface object. All other 
     memory allocation is done in the calling routine; only pointers 
     are passed down. A typical calling example is:

       NL_SURFACE  sur;
       NL_REAL     **wx, **wy, **wz, **w, *U, *V;
       NL_INDEX    n, m, r, s;
       NL_DEGREE   p, q;
       NL_STACKS   S;
       ...
       (allocate memory for wx, wy, wz, w, U and V);
       ...
       N_SrfFromCPtCoordsAndKnots(&sur,wx,wy,wz,w,n,m,p,q,U,V,r,s,&S);


   ACCESS:
   
     sur         , in/out ,  NURBS surface
     wx,wy,wz,w  , input  ,  Coordinates of control points
     n,m         , input  ,  Highest indexes in <wx,wy,wz,w>
     p,q         , input  ,  Degrees
     U,V         , input  ,  Knots
     r,s         , input  ,  Highest indexes in U and V
     S           , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfFromCPtCoordsAndKnots */
NL_FLAG N_SrfFromCPtCoordsAndKnots( NL_SURFACE *sur, NL_REAL ** wx, NL_REAL ** wy, NL_REAL ** wz, NL_REAL ** w, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_REAL *U, NL_REAL *V, NL_INDEX r, NL_INDEX s, NL_STACKS *S )
{
    NL_CPOINT ** Pw;

    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    /* Allocate memory */

    Pw = N_XYZTo2dCPtArray( wx, wy, wz, w, n, m, S );

    if( Pw EQ NULL )
        return (1);

    net = N_AllocCNet( S );

    if( net EQ NULL )
        return (1);

    knu = N_AllocKnotVector( S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVector( S );

    if( knv EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_CNetFromCPts( net, Pw, n, m );
    N_KnotVectorFromRealArray( knu, U, r );
    N_KnotVectorFromRealArray( knv, V, s );
    N_SrfFromCNetAndKnotVectors( sur, net, p, q, knu, knv );

    /* Exit */

    return (0);
} /* end N_SrfFromCPtCoordsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This  routine generates a surface object defined in  power basis 
     form. That is, given the  vector coefficients of the power basis
     surface  along  with  the rectangle  over  which the  surface is 
     defined, this routine creates a NL_SURFACE  object that  represents 
     the power basis surface. A typical calling example is:

       NL_SURFACE    spl;
       NL_CPOINT     **aw;
       NL_INDEX      n, m;
       NL_PARAMETER  a, b, c, d;
       NL_STACKS     S;
       ...
       (get aw array, n, m, a, b, c and d);
       ...
       N_CreateSplineSrf(&spl,aw,n,m,a,b,c,d,&S);

     MEMORY FOR THE NL_SURFACE STRUCTURE spl IS ALLOCATED IN THE CALLING 
     ROUTINE.  ROUTINES  HANDLING  SURFACES IN  POWER  BASIS FORM ARE 
     FOUND WITH THE PREFIX spl, i.e. N_spl***.c


   ACCESS:
   
     spl     , in/out ,  NURBS surface
     aw      , input  ,  Vector coefficients
     n,m     , input  ,  Highest indexes in aw
     a,b,c,d , input  ,  Parameter rectangle bounds
     S       , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSplineSrf( NL_SURFACE *spl, NL_CPOINT ** aw, NL_INDEX n, NL_INDEX m, NL_PARAMETER a, NL_PARAMETER b, NL_PARAMETER c, NL_PARAMETER d, NL_STACKS *S )
{
    NL_REAL *U, *V;

    NL_DEGREE p, q;

    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    /* Allocate memory */

    net = N_AllocCNet( S );

    if( net EQ NULL )
        return (1);

    knu = N_AllocKnotVector( S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVector( S );

    if( knv EQ NULL )
        return (1);

    U = N_AllocReal1dArray( 1, S );

    if( U EQ NULL )
        return (1);

    V = N_AllocReal1dArray( 1, S );

    if( V EQ NULL )
        return (1);

    U[0] = a;
    U[1] = b;
    V[0] = c;
    V[1] = d;

    p = (NL_DEGREE)n;
    q = (NL_DEGREE)m;

    /* Make pointer assignments */

    N_CNetFromCPts( net, aw, n, m );
    N_KnotVectorFromRealArray( knu, U, 1 );
    N_KnotVectorFromRealArray( knv, V, 1 );
    N_SrfFromCNetAndKnotVectors( spl, net, p, q, knu, knv );

    /* Exit */

    return (0);
} /* end N_CreateSplineSrf */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine detaches the control net and the knot  vector
     objects from a given surface object. A typical calling example is:

       NL_SURFACE     sur;
       NL_CNET        *net;
       NL_DEGREE      p, q;
       NL_KNOTVECTOR  *knu, *knv;
       ...
       N_SrfGetNetAndKnotVectors(&sur,&net,&p,&q,&knu,&knv);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     net     , output ,  Control net
     p,q     , output ,  Degrees
     knu,knv , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetNetAndKnotVectors */
NL_VOID N_SrfGetNetAndKnotVectors( NL_SURFACE *sur, NL_CNET ** net, NL_DEGREE *p, NL_DEGREE *q, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv )
{
    *net = sur->net;
    *p = sur->p;
    *q = sur->q;
    *knu = sur->knu;
    *knv = sur->knv;
} /* end N_SrfGetNetAndKnotVectors */


/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  breaks  a surface  object  down  to its 
     components, i.e. indexes, control point array and knot vectors.
     A typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    n, m, r, s;
       NL_CPOINT   **Pw;
       NL_DEGREE   p, q;
       NL_REAL     *U, *V;
       ...
       N_SrfGetCPtsDegreesAndKnots(&sur,&n,&m,&Pw,&p,&q,&r,&s,&U,&V);


   ACCESS:
   
     sur , input  ,  NURBS surface
     n,m , output ,  Highest indexes in Pw
     Pw  , output ,  Control points
     p,q , output ,  Degrees
     r,s , output ,  Highest indexes in U and V
     U,V , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/
/* NL_VOID N_SrfGetCPtsDegreesAndKnots */
NL_VOID N_SrfGetCPtsDegreesAndKnots
 (const NL_SURFACE *sur, /* in : tgt surface */
  NL_INDEX         *n,   /* out: max 1st index in Pw - number of U control points = n + 1 */
  NL_INDEX         *m,   /* out: max 2nd index in Pw - number of V control points = m + 1 */
  NL_CPOINT      ***Pw,  /* out: Pw sized:[n+1][m+1]  */
  NL_DEGREE        *p,   /* out: u degree */
  NL_DEGREE        *q,   /* out: v degree */
  NL_INDEX         *r,   /* out: max knot index in U array - number of u knots = r + 1 */
  NL_INDEX         *s,   /* out: max knot index in V array - number of v knots = s + 1 */
  NL_REAL         **U,   /* out: U knot vector sized:[r+1] */
  NL_REAL         **V )  /* out: V knot vector sized:[s+1] */
{
    *n = sur->net->n;
    *m = sur->net->m;
    *Pw = sur->net->Pw;
    *p = sur->p;
    *q = sur->q;
    *r = sur->knu->m;
    *s = sur->knv->m;
    *U = sur->knu->U;
    *V = sur->knv->U;
} /* end                */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control net info from surface object.
     A typical calling example is:

       NL_SURFACE  sur; 
       NL_INDEX    n, m;
       NL_CPOINT   **Pw;
       ...
       N_SrfGetCPts(&sur,&n,&m,&Pw);


   ACCESS:
   
     sur , input  ,  NURBS surface
     n,m , output ,  Highest indexes in Pw
     Pw  , output ,  Control points


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetCPts */
NL_VOID N_SrfGetCPts( NL_SURFACE *sur, NL_INDEX *n, NL_INDEX *m, NL_CPOINT *** Pw )
{
    *n = sur->net->n;
    *m = sur->net->m;
    *Pw = sur->net->Pw;
} /* end N_SrfGetCPts */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the highest indexes in surface 
     definition. A typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    n, m, r, s;
       ...
       N_SrfGetArraySizes(&sur,&n,&m,&r,&s);


   ACCESS:
   
     sur , input  ,  NURBS surface
     n,m , output ,  Highest indexes in Pw
     r,s , output ,  Highest indexes in U and V


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetArraySizes */
NL_VOID N_SrfGetArraySizes( NL_SURFACE *sur, NL_INDEX *n, NL_INDEX *m, NL_INDEX *r, NL_INDEX *s )
{
    *n = sur->net->n;
    *m = sur->net->m;
    *r = sur->knu->m;
    *s = sur->knv->m;
} /* end N_SrfGetArraySizes */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control points and knots from a surface 
     object. A typical calling example is:

       NL_SURFACE  sur;
       NL_CPOINT   **Pw;
       NL_REAL     *U, *V;
       ...
       N_SrfGetCPtsAndKnots(&sur,&Pw,&U,&V);


   ACCESS:
   
     sur , input  ,  NURBS surface
     Pw  , output ,  Control point array
     U   , output ,  U-knot vector array
     V   , output ,  V-knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetCPtsAndKnots */
NL_VOID N_SrfGetCPtsAndKnots
 (NL_SURFACE  * sur,  /* in : target surface       */
  NL_CPOINT *** Pw,   /* out: Control Point Arrays */
  NL_REAL    ** U,    /* out: U Knots */
  NL_REAL    ** V )   /* out: V knots */
{
    *Pw = sur->net->Pw;
    *U = sur->knu->U;
    *V = sur->knv->U;
} /* end N_SrfGetCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control points, knot vectors and knots 
     from a surface object. A typical calling example is:

       NL_SURFACE     sur;
       NL_CPOINT      **Pw;
       NL_KNOTVECTOR  *knu, *knv;
       NL_REAL        *U, *V;
       ...
       N_SrfGetCPtsKnotVectorAndKnots(&sur,&Pw,&knu,&knv,&U,&V);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     Pw      , output ,  Control point array
     knu,knv , output ,  Knot vector objects
     U,V     , output ,  Knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/
/* NL_VOID  N_SrfGetCPtsKnotVectorAndKnots */
NL_VOID N_SrfGetCPtsKnotVectorAndKnots( NL_SURFACE *sur, NL_CPOINT *** Pw, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_REAL ** U, NL_REAL ** V )
{
    *Pw = sur->net->Pw;
    *knu = sur->knu;
    *knv = sur->knv;
    *U = sur->knu->U;
    *V = sur->knv->U;
} /* end N_SrfGetCPtsKnotVectorAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets control point and knot vector pointers 
     of a surface object. A typical calling example is:

       NL_SURFACE  sur;
       NL_CPOINT   **Pw;
       NL_REAL     *U, *V;
       ...
       N_SrfSetCPtsAndKnots(&sur,Pw,U,V);


   ACCESS:
   
     sur , in/out ,  NURBS surface
     Pw  , input  ,  Control point array
     U   , input  ,  U-knot vector array
     V   , input  ,  V-knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfSetCPtsAndKnots */
NL_VOID N_SrfSetCPtsAndKnots( NL_SURFACE *sur, NL_CPOINT ** Pw, NL_REAL *U, NL_REAL *V )
{
    sur->net->Pw = Pw;
    sur->knu->U = U;
    sur->knv->U = V;
} /* end N_SrfSetCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vectors info from surface object.
     A typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    r, s;
       NL_REAL     *U, *V;
       ...
       N_SrfGetKnots(&sur,&r,&s,&U,&V);


   ACCESS:
   
     sur , input  ,  NURBS surface
     r,s , output ,  Highest indexes in U and V
     U,V , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetKnots */
NL_VOID N_SrfGetKnots( NL_SURFACE *sur, NL_INDEX *r, NL_INDEX *s, NL_REAL ** U, NL_REAL ** V )
{
    *r = sur->knu->m;
    *s = sur->knv->m;
    *U = sur->knu->U;
    *V = sur->knv->U;
} /* end N_SrfGetKnots */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the parameter bounds from a surface 
     object. A typical calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  ul, ur, vb, vt;
       ...
       N_SrfGetParameterBounds(&sur,&ul,&ur,&vb,&vt);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     ul,ur , output ,  Parameter bounds in U-knot vector
     vb,vt , output ,  Parameter bounds in V-knot vector


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetParameterBounds */
NL_VOID N_SrfGetParameterBounds( NL_SURFACE *sur, NL_PARAMETER *ul, NL_PARAMETER *ur, NL_PARAMETER *vb, NL_PARAMETER *vt )
{
    *ul = sur->knu->U[0];
    *ur = sur->knu->U[sur->knu->m];
    *vb = sur->knv->U[0];
    *vt = sur->knv->U[sur->knv->m];
} /* end N_SrfGetParameterBounds */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vector objects from surface object. 
     A typical calling example is:

       NL_SURFACE     sur;
       NL_KNOTVECTOR  *knu, *knv;
       ...
       N_SrfGetKnotVectors(&sur,&knu,&knv);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     knu,knv , output ,  Knot vector objects


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetKnotVectors */
NL_VOID N_SrfGetKnotVectors( NL_SURFACE *sur, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv )
{
    *knu = sur->knu;
    *knv = sur->knv;
} /* end N_SrfGetKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the degrees of a surface. A typical 
     calling example is:

       NL_SURFACE  sur;
       NL_DEGREE   p, q;
       ...
       N_SrfGetDegrees(&sur,&p,&q);


   ACCESS:
   
     sur , input  ,  NURBS surface
     p,q , output ,  Degrees


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfGetDegrees */
NL_VOID N_SrfGetDegrees( NL_SURFACE *sur, NL_DEGREE *p, NL_DEGREE *q )
{
    *p = sur->p;
    *q = sur->q;
} /* end N_SrfGetDegrees */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine compacts  control  point and  knot  vector 
     arrays. That  is, given  a surface  with  control point  and knot 
     vector arrays larger than required. This routine redefines  these 
     arrays to the appropriate  sizes which  makes surface  definition 
     more memory efficient. A typical calling example is:

       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define surface);
       ...
       N_SrfCompress(&sur,&SG);

     SG MUST BE sur's STACK, I.E.  ALL MEMORY ALLOCATED FOR  sur, MUST 
     BE ON SG.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     SG  , input  ,  sur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfCompress( NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_REAL *U, *V, *S, *T;

    NL_CPOINT ** Pw, ** Qw;

    /* Get surface data */

    N_SrfGetArraySizes( sur, &n, &m, &r, &s );
    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Allocate memory for new control point and knot vector arrays */

    Qw = N_AllocCPt2dArray( n, m, SG );

    if( Qw EQ NULL )
        NL_QUIT;

    S = N_AllocReal1dArray( r, SG );

    if( S EQ NULL )
        NL_QUIT;

    T = N_AllocReal1dArray( s, SG );

    if( T EQ NULL )
        NL_QUIT;

    /* Copy control points and knots */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CopyCPt( Pw[i][j], &Qw[i][j] );
        }
    }

    for ( i = 0; i <= r; i++ )
        S[i] = U[i];

    for ( j = 0; j <= s; j++ )
        T[j] = V[j];

    /* Redefine surface and kill old memory */

    N_SrfSetCPtsAndKnots( sur, Qw, S, T );

    N_FreeCPt2dArray( Pw, SG );
    N_FreeReal1dArray( U, SG );
    N_FreeReal1dArray( V, SG );

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfCompress */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine clamps given parameters with respect to the
     knot rectangle. A typical calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       ...
       N_ClampSrfAtParams(&sur,&u,&v);


   ACCESS:
   
     sur , input  ,  NURBS surface
     u,v , in/out ,  Parameters to be clamped


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ClampSrfAtParams( NL_SURFACE *sur, NL_PARAMETER *u, NL_PARAMETER *v )
{
    NL_INDEX r, s;

    NL_REAL *U, *V;

    N_SrfGetKnots( sur, &r, &s, &U, &V );

    if( *u LT U[0] )
        *u = U[0];

    if( *u GT U[r] )
        *u = U[r];

    if( *v LT V[0] )
        *v = V[0];

    if( *v GT V[s] )
        *v = V[s];
} /* end N_ClampSrfAtParams */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets parameters to complete surface 
     definition. A typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    n, m, r, s;
       NL_DEGREE   p, q;
       ...
       N_SrfSetSizeIndices(&sur,n,m,p,q,r,s);


   ACCESS:
   
     sur , in/out ,  NURBS surface
     n,m , input  ,  Highest indexes in control net array
     p,q , input  ,  Degrees
     r,s , input  ,  Highest indexes in knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfSetSizeIndices */
NL_VOID N_SrfSetSizeIndices( NL_SURFACE *sur, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s )
{
    sur->net->n = n;
    sur->net->m = m;
    sur->p = p;
    sur->q = q;
    sur->knu->m = r;
    sur->knv->m = s;
} /* end N_SrfSetSizeIndices */


/*******************************************************************//**


   DESCRIPTION:

     Given a  surface object,  this  routine swaps the role of u and v-
     directional  parameters, i.e.  S_old(u,v) = S_new(v,u). A  typical
     calling example is:

       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       N_SwapUV(&sur,&S);

     It is assumed that the  stack S contains  all memory  necessary to 
     define sur. IT IS IMPORTANT BECAUSE NL_OLD CONTROL NL_POINTS ARE KILLED!


   ACCESS:
   
     sur , in/out ,  NURBS surface 
     S   , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SwapUV( NL_SURFACE *sur, NL_STACKS *S )
{
    NL_INDEX i, j, n, m;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, ** Qw;

    NL_CNET *net;

    /* Get local notation */

    N_SrfGetNetAndKnotVectors( sur, &net, &p, &q, &knu, &knv );
    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Compute new control points */

    Qw = N_AllocCPt2dArray( m, n, S );

    if( Qw EQ NULL )
        return (1);

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CopyCPt( Pw[i][j], &Qw[j][i] );
        }
    }

    /* Define new surface */

    N_CNetFromCPts( net, Qw, m, n );
    N_SrfFromCNetAndKnotVectors( sur, net, q, p, knv, knu );

    /* Kill old memory */

    N_FreeCPt2dArray( Pw, S );

    /* Exit */

    return (0);
} /* end N_SwapUV */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine initializes a surface structure by  setting
     pointers to NULL and the degrees to -1. It is  used to  check if
     memory allocation is needed, i.e. if the surface is  initialized 
     to the  NULL surface, memory  is allocated to  hold the  control 
     net  and  knot vectors objects.  Otherwise  it is  assumed  that 
     memory has already been allocated. A typical calling example is:

       NL_SURFACE  sur;
       ...
       N_SrfInitArrays(&sur);
     

   ACCESS:
   
     sur , in/out ,  NURBS surface


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SrfInitArrays */
NL_VOID N_SrfInitArrays( NL_SURFACE *sur )
{
    sur->net = NULL;
    sur->p = -1;
    sur->q = -1;
    sur->knu = NULL;
    sur->knv = NULL;
} /* end N_SrfInitArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the surface is initialized to 
     the NULL surface. If yes, NL_TRUE is returned. Otherwise, NL_FALSE 
     is  returned. This  routine  is  used  to  check  if  memory 
     allocation is needed. A typical calling example is:

       NL_SURFACE  sur;
       ...
       if( N_SrfAreArraysNULL(&sur) )  --> allocate memory;
     

   ACCESS:
   
     sur , input ,  NURBS surface


   RETURN CODES:

     NL_TRUE:  Surface is initialized to NULL (need memory)
     NL_FALSE: Surface is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

/* NL_BOOLEAN  N_SrfAreArraysNULL */
NL_BOOLEAN N_SrfAreArraysNULL( NL_SURFACE *sur )
{
    NL_DEGREE p, q;

    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    /* Get local notation */

    N_SrfGetNetAndKnotVectors( sur, &net, &p, &q, &knu, &knv );

    /* Check initialization */

    if( net EQ NULL OR p EQ - 1 OR q EQ - 1 OR knu EQ NULL OR knv EQ NULL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_SrfAreArraysNULL */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the surface is rational or not. A
     typical calling example is:

       NL_SURFACE  sur;
       ...
       if( N_IsSrfRat(&sur) )  --> handle rational case;
     

   ACCESS:
   
     sur , input ,  NURBS surface


   RETURN CODES:

     NL_TRUE:  Surface is rational
     NL_FALSE: Surface is NOT rational

   ***********************************************************************/

/* NL_BOOLEAN  N_IsSrfRat */
NL_BOOLEAN N_IsSrfRat( NL_SURFACE *sur )
{
    NL_INDEX n, m;

    NL_REAL w;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check rationality */

    N_CPtGetW( Pw[0][0], &w );

    if( w NEQ NL_NOW )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_IsSrfRat */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine checks if a NURBS surface is degenerate to a
     single point. A typical calling example is:
 
       NL_SURFACE  sur;
       ...
       if( N_SrfIsDegen(&sur) )  --> handle point;
     
 
   ACCESS:
   
     sur  , input ,  NURBS suface
 
 
   RETURN CODES:
 
     NL_TRUE : Surface is a point
     NL_FALSE: Surface is NOT a point
 
   ***********************************************************************/

/* NL_BOOLEAN  N_SrfIsDegen */
NL_BOOLEAN N_SrfIsDegen( NL_SURFACE *sur )
{
    NL_FLAG dst = NL_YES;

    NL_INDEX i, j, n, m;

    NL_REAL d, fac;

    NL_POINT Q, M;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check if surface is a point */

    fac = 1.0 / (((NL_REAL)n + 1.0) * ((NL_REAL)m + 1.0));
    N_CopyPt( NL_ZERO, &M );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &Q );
            N_Sum2Pts( M, Q, &M );
        }
    }
    N_ScalePt( fac, M, &M );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &Q );
            N_DistPtPt( Q, M, &d );

            if( d GT NL_MTOL )
            {
                dst = NL_NO;
                break;
            }
        }

        if( dst EQ NL_NO )
            break;
    }

    if( dst EQ NL_YES )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_SrfIsDegen */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a NURBS surface is closed or not.
     A typical calling example is:

       NL_SURFACE  sur;
       ...
       if( N_SrfIsClosed(&sur,NL_UDIR) )  --> handle closed case;
     

   ACCESS:
   
     sur , input ,  NURBS surface
     dir , input ,  Flag:
                      NL_UDIR: check if closed in u-direction
                      NL_VDIR: check if closed in v-direction


   RETURN CODES:

     NL_TRUE : Surface is closed
     NL_FALSE: Surface is NOT closed

   ***********************************************************************/

NL_BOOLEAN N_SrfIsClosed( NL_SURFACE *sur, NL_FLAG dir )
{
    NL_FLAG flg = NL_TRUE;

    NL_INDEX i, j, n, m;

    NL_REAL d;

    NL_POINT Ps, Pe;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check closeness */

    switch( dir )
    {
        case NL_UDIR:
            for ( j = 0; j <= m; j++ )
            {
                N_CPtToPtEuclid( Pw[0][j], &Ps );
                N_CPtToPtEuclid( Pw[n][j], &Pe );
                N_DistPtPt( Ps, Pe, &d );

                if( d GT NL_MTOL )
                {
                    flg = NL_FALSE;
                    break;
                }
            }
            break;

        case NL_VDIR:
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i][0], &Ps );
                N_CPtToPtEuclid( Pw[i][m], &Pe );
                N_DistPtPt( Ps, Pe, &d );

                if( d GT NL_MTOL )
                {
                    flg = NL_FALSE;
                    break;
                }
            }
            break;

        default:

            flg = NL_FALSE;
    }

    return (flg);
} /* end N_SrfIsClosed */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine checks if a 1-D point set is closed or not.
     A typical calling example is:

       NL_INDEX   m;
       NL_REAL    tol;
       NL_POINT   *P;
       NL_CPOINT  *Pw;
       ...
       (get P/Pw, choose tol);
       ...
       if( N_1dPtSetIsClosed((NL_VOID *)P ,m,NL_EPOINT,tol) ) --> handle closed data;
       if( N_1dPtSetIsClosed((NL_VOID *)Pw,m,NL_HPOINT,tol) ) --> handle closed data;
     
     This routine handles both NL_POINT and NL_CPOINT input.
  

   ACCESS:
   
     A   , input ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     m   , input ,  Highest index in A
     ptp , input ,  Flag:
                      NL_EPOINT: Euclidean point pointer passed in
                      NL_HPOINT: Homogeneous point pointer passed in
     tol , input ,  Tolerance for point coincidence checking


   RETURN CODES:

     NL_TRUE : Data set is closed
     NL_FALSE: Data set is NOT closed

   ***********************************************************************/

/* NL_BOOLEAN  N_1dPtSetIsClosed */
NL_BOOLEAN N_1dPtSetIsClosed( NL_VOID *A, NL_INDEX m, NL_FLAG ptp, NL_REAL tol )
{
    NL_FLAG cls = NL_TRUE;

    NL_REAL d;

    NL_POINT *P;

    NL_CPOINT *Pw;

    /* Check closeness */

    switch( ptp )
    {
        case NL_EPOINT:

            P = (NL_POINT *)A;

            N_DistPtPt( P[0], P[m], &d );

            if( d GT tol )
                cls = NL_FALSE;
            break;

        case NL_HPOINT:

            Pw = (NL_CPOINT *)A;

            N_DistCptCptHomo( Pw[0], Pw[m], &d );

            if( d GT tol )
                cls = NL_FALSE;
            break;

        default:

            cls = NL_FALSE;
    }

    return cls;
} /* end N_1dPtSetIsClosed */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine checks if a 2-D point set is closed or not.
     A typical calling example is:

       NL_INDEX   n, m;
       NL_REAL    tol;
       NL_POINT   **P;
       NL_CPOINT  **Pw;
       ...
       (get P/Pw, choose tol);
       ...
       if( N_2dPtSetIsClosed((NL_VOID **)P ,n,m,NL_EPOINT, NL_UDIR,tol) ) --> u-closed;
       if( N_2dPtSetIsClosed((NL_VOID **)Pw,n,m,NL_HPOINT,NL_UVDIR,tol) ) --> uv-closed;
     
     This routine handles both NL_POINT and NL_CPOINT input.
  

   ACCESS:
   
     A   , input ,  NL_VOID pointer representing NL_POINT or NL_CPOINT data
     n,m , input ,  Highest indexes in A
     ptp , input ,  Flag:
                      NL_EPOINT: Euclidean point pointer passed in
                      NL_HPOINT: Homogeneous point pointer passed in
     dir , input ,  Flag:
                      NL_UDIR : check in u-direction e.g. cylinder data
                      NL_VDIR : check in v-direction
                      NL_UVDIR: check in both diretions, e.g. torus data
     tol , input ,  Tolerance for point coincidence checking


   RETURN CODES:

     NL_TRUE : Data set is closed
     NL_FALSE: Data set is NOT closed

   ***********************************************************************/

/* NL_BOOLEAN  N_2dPtSetIsClosed */
NL_BOOLEAN N_2dPtSetIsClosed( NL_VOID ** A, NL_INDEX n, NL_INDEX m, NL_FLAG ptp, NL_FLAG dir, NL_REAL tol )
{
    NL_FLAG cls = NL_TRUE;

    NL_INDEX i, j;

    NL_REAL d;

    NL_POINT ** P;

    NL_CPOINT ** Pw;

    /* Check direction flag */

    switch( dir )
    {
        case NL_UDIR:
            break;

        case NL_VDIR:
            break;

        case NL_UVDIR:
            break;

        default:
            return NL_FALSE;
    }

    /* Check closeness */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        switch( ptp )
        {
            case NL_EPOINT:

                P = (NL_POINT ** )A;

                for ( j = 0; j <= m; j++ )
                {
                    N_DistPtPt( P[0][j], P[n][j], &d );

                    if( d GT tol )
                    {
                        cls = NL_FALSE;
                        break;
                    }
                }
                break;

            case NL_HPOINT:

                Pw = (NL_CPOINT ** )A;

                for ( j = 0; j <= m; j++ )
                {
                    N_DistCptCptHomo( Pw[0][j], Pw[n][j], &d );

                    if( d GT tol )
                    {
                        cls = NL_FALSE;
                        break;
                    }
                }
                break;

            default:

                cls = NL_FALSE;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        switch( ptp )
        {
            case NL_EPOINT:

                P = (NL_POINT ** )A;

                for ( i = 0; i <= n; i++ )
                {
                    N_DistPtPt( P[i][0], P[i][m], &d );

                    if( d GT tol )
                    {
                        cls = NL_FALSE;
                        break;
                    }
                }
                break;

            case NL_HPOINT:

                Pw = (NL_CPOINT ** )A;

                for ( i = 0; i <= n; i++ )
                {
                    N_DistCptCptHomo( Pw[i][0], Pw[i][m], &d );

                    if( d GT tol )
                    {
                        cls = NL_FALSE;
                        break;
                    }
                }
                break;

            default:

                cls = NL_FALSE;
        }
    }

    /* Return result */

    return cls;
} /* end N_2dPtSetIsClosed */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if two surfaces are coincident or not.
     A typical calling example is:

       NL_REAL     tol;
       NL_SURFACE  surP, surQ;
       NL_STACKS   SG;
       ...
       (get surP and surQ, choose tol);
       ...
       if( N_SrfsAreCoincident(&surP,&surQ,tol,&SG) ) --> handle coincident case;
     

   ACCESS:
   
     surP , input ,  NURBS surface
     surQ , input ,  NURBS surface
     tol  , input ,  Tolerance for coincidence checking


   RETURN CODES:

     NL_TRUE : Surfaces are coincident
     NL_FALSE: Surfaces are NOT coincident

   ***********************************************************************/

/* NL_BOOLEAN  N_SrfsAreCoincident */
NL_BOOLEAN N_SrfsAreCoincident( NL_SURFACE *surP, NL_SURFACE *surQ, NL_REAL tol, NL_STACKS *SG )
{
    NL_FLAG error;

    NL_INDEX i, j, n, m;

    NL_REAL dw, dwmax;

    NL_CPOINT ** Pw, ** Qw;

    NL_SURFACE ** sur;

    /* Get array of surface pointers */

    sur = N_AllocArraySrfPtrs( 1, SG );

    if( sur EQ NULL )
        return NL_FALSE;

    sur[0] = surP;
    sur[1] = surQ;

    /* Make input surfaces compatible */

    error = N_MakeSrfsCompatibleUV( sur, 1, SG );

    if( error EQ NL_YES )
        return NL_FALSE;

    /* Check coincidence */

    N_SrfGetCPts( sur[0], &n, &m, &Pw );
    N_SrfGetCPts( sur[1], &n, &m, &Qw );

    dwmax = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_DistCptCptHomo( Pw[i][j], Qw[i][j], &dw );

            if( dw GT dwmax )
                dwmax = dw;
        }
    }

    /* Return result */

    if( dwmax LT tol )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_SrfsAreCoincident */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine checks if two surfaces are equal or not.
     Two surfaces are equal when they have the same degrees, number of knots,
     weights, and control points and all those values are within
     tolerance of one another.

     A typical calling example is:

       NL_REAL    tol3d, tol1d;
       NL_SURFACE surP, surQ;
       ...
       (get surP and surQ, choose tolerances);
       ...
       if( N_SrfsAreEqual(&surP,&surQ,tol3d,tol1d) ) --> handle equal case;
     

   ACCESS:
   
     surP , input ,  NURBS surface
     surQ , input ,  NURBS surface
     tol3d, input ,  Tolerance for coincident control point checking
     tol1d, input ,  Tolerance for coincident knot checking


   RETURN CODES:

     NL_TRUE : Surfaces are equal
     NL_FALSE: Surfaces are NOT equal

   ***********************************************************************/

NL_BOOLEAN N_SrfsAreEqual( NL_SURFACE *surP, NL_SURFACE *surQ, NL_REAL tol3d, NL_REAL tol1d )
{
    NL_INTEGER ii, jj ;

    NL_INDEX   Pn, Pm, Qn, Qm ;    /* Highest indices in control point arrays */ 
                          
    NL_CPOINT **Pw, **Qw ;         /* array of control points */
                          
    NL_DEGREE  Pp, Pq, Qp, Qq;     /* surface degrees */
                          
    NL_INDEX   Pr, Ps, Qr, Qs ;    /* Highest indices in KnotArrays */
                          
    NL_REAL   *PU, *PV, *QU, *QV ; /* arrays of knots (multiple knots are represented multiple times ) */

    NL_BOOLEAN bRtn ;

    NL_REAL d;

    NL_CPOINT Rw;

    /* surface data */
    N_SrfGetCPtsDegreesAndKnots(surP, &Pn, &Pm, &Pw, &Pp, &Pq, &Pr, &Ps, &PU, &PV ) ;
    N_SrfGetCPtsDegreesAndKnots(surQ, &Qn, &Qm, &Qw, &Qp, &Qq, &Qr, &Qs, &QU, &QV ) ;

    /* check sizes */
    bRtn = (   Pp == Qp && Pq == Qq    /* same degrees */
            && Pn == Qn && Pm == Qm    /* same control point counts */
            && Pr == Qr && Ps == Qs) ; /* same knot counts */

    /* check control points */
    if(bRtn)
      {
        for(ii=0;ii<=Pn && bRtn ;ii++)
          {
            for(jj=0;jj<=Pm && bRtn;jj++)
              {
                /* distance between control points */
                N_Diff2CPts( Pw[ii][jj], Qw[ii][jj], &Rw );
                N_CPtMagnitude( Rw, &d );

                /* check for equivalent locations */
                bRtn &= (d <= tol3d) ;

              } /* end iter jj, every control point */
          } /* end iter ii, every control point */
      } /* end control points check */

    /* check U knots */
    if(bRtn)
      {
        /* multiple knots are represented multiple times */
        for(ii=0;ii<=Pr && bRtn ;ii++)
          {
            /* check for equivalent locations */
            bRtn &= (fabs(PU[ii] - QU[ii]) <= tol1d) ;

          } /* end iter every control point */
      } /* end knots check */

    /* check V knots */
    if(bRtn)
      {
        /* multiple knots are represented multiple times */
        for(ii=0;ii<=Ps && bRtn ;ii++)
          {
            /* check for equivalent locations */
            bRtn &= (fabs(PV[ii] - QV[ii]) <= tol1d) ;

          } /* end iter every control point */
      } /* end knots check */

    /* all done */
    return bRtn;

} /* end N_SrfsAreEqual */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine checks if a NURBS surface has a pole at one of 
     the four boundaries. A typical calling example is:
 
       NL_SURFACE  sur;
       ...
       if( N_SrfIsSingular(&sur,NL_BOTTOM) )  --> handle pole;
     
 
   ACCESS:
   
     sur  , input ,  NURBS suface
     side , input ,  Flag:
                       NL_LEFT  : check u=umin boundary
                       NL_RIGHT : check u=umax boundary
                       NL_BOTTOM: check v=vmin boundary
                       NL_TOP   : check v=vmax boundary
 
 
   RETURN CODES:
 
     NL_TRUE : Surface has a pole
     NL_FALSE: Surface does NOT have a pole
 
   ***********************************************************************/

/* NL_BOOLEAN  N_SrfIsSingular */
NL_BOOLEAN N_SrfIsSingular( NL_SURFACE *sur, NL_FLAG side )
{
    NL_FLAG dst = NL_YES;

    NL_INDEX i, j, n, m;

    NL_REAL d, fac;

    NL_POINT Q, M;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check pole */

    switch( side )
    {
        case NL_LEFT:

            N_CPtToPtEuclid( Pw[0][0], &M );
            N_CPtToPtEuclid( Pw[0][(m / 2) + 1], &Q );
            N_DistPtPt( M, Q, &d );

            if( d GT NL_MTOL )
            {
                dst = NL_NO;
                break;
            }

            fac = 1.0 / ((NL_REAL)m + 1.0);

            for ( j = 1; j <= m; j++ )
            {
                N_CPtToPtEuclid( Pw[0][j], &Q );
                N_Sum2Pts( M, Q, &M );
            }

            N_ScalePt( fac, M, &M );

            for ( j = 0; j <= m; j++ )
            {
                N_CPtToPtEuclid( Pw[0][j], &Q );
                N_DistPtPt( Q, M, &d );

                if( d GT NL_MTOL )
                {
                    dst = NL_NO;
                    break;
                }
            }
            break;

        case NL_RIGHT:

            N_CPtToPtEuclid( Pw[n][0], &M );
            N_CPtToPtEuclid( Pw[n][(m / 2) + 1], &Q );
            N_DistPtPt( M, Q, &d );

            if( d GT NL_MTOL )
            {
                dst = NL_NO;
                break;
            }

            fac = 1.0 / ((NL_REAL)m + 1.0);

            for ( j = 1; j <= m; j++ )
            {
                N_CPtToPtEuclid( Pw[n][j], &Q );
                N_Sum2Pts( M, Q, &M );
            }

            N_ScalePt( fac, M, &M );

            for ( j = 0; j <= m; j++ )
            {
                N_CPtToPtEuclid( Pw[n][j], &Q );
                N_DistPtPt( Q, M, &d );

                if( d GT NL_MTOL )
                {
                    dst = NL_NO;
                    break;
                }
            }
            break;

        case NL_BOTTOM:

            N_CPtToPtEuclid( Pw[0][0], &M );
            N_CPtToPtEuclid( Pw[(n / 2) + 1][0], &Q );
            N_DistPtPt( M, Q, &d );

            if( d GT NL_MTOL )
            {
                dst = NL_NO;
                break;
            }

            fac = 1.0 / ((NL_REAL)n + 1.0);

            for ( i = 1; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i][0], &Q );
                N_Sum2Pts( M, Q, &M );
            }

            N_ScalePt( fac, M, &M );

            for ( i = 0; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i][0], &Q );
                N_DistPtPt( Q, M, &d );

                if( d GT NL_MTOL )
                {
                    dst = NL_NO;
                    break;
                }
            }
            break;

        case NL_TOP:

            N_CPtToPtEuclid( Pw[0][m], &M );
            N_CPtToPtEuclid( Pw[(n / 2) + 1][m], &Q );
            N_DistPtPt( M, Q, &d );

            if( d GT NL_MTOL )
            {
                dst = NL_NO;
                break;
            }

            fac = 1.0 / ((NL_REAL)n + 1.0);

            for ( i = 1; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i][m], &Q );
                N_Sum2Pts( M, Q, &M );
            }

            N_ScalePt( fac, M, &M );

            for ( i = 0; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i][m], &Q );
                N_DistPtPt( Q, M, &d );

                if( d GT NL_MTOL )
                {
                    dst = NL_NO;
                    break;
                }
            }
            break;

        default:

            dst = NL_NO;
    }

    if( dst EQ NL_YES )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_SrfIsSingular */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine checks if a NURBS surface is flat or not. Flat
     means that the  surface patch can be approximated, to within a user
     specified tolerance, by a  planar quadrilateral. If  the surface is 
     not flat, it  determines  which  direction to  subdivide. A typical 
     calling example is:
 
       NL_FLAG     flt, dir;
       NL_REAL     eps;
       NL_SURFACE  sur;
       ...
       (get sur and tolerance eps)
       ...
       N_SrfIsFlat(&sur,eps,&flt,&dir);
     
 
   ACCESS:
   
     sur , input  ,  NURBS surface
     eps , input  ,  Flatness tolerance
     flt , output ,  Flatness indicator:
                       NL_YES: surface is flat
                       NL_NO : surface is NOT flat
     dir , output ,  Direction of subdivision if flt = NL_NO:
                       NL_UDIR: subdivide at a v value
                       NL_VDIR: subdivide at a u value
 
 
   RETURN CODES:
 
     0 : No eror
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

/* NL_FLAG  N_SrfIsFlat */
NL_FLAG N_SrfIsFlat( NL_SURFACE *sur, NL_REAL eps, NL_FLAG *flt, NL_FLAG *dir )
{
    NL_FLAG degen, flat, straight = NL_NO, error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL ** dist, a, b, c, d, du, dv, min, max, maxu = 0.0, maxv = 0.0, dot;

    NL_PARAMETER us, ue, vs, ve;

    NL_POINT P00, P10, P01, P11, P, Q, ** SD;

    NL_VECTOR N00, N10, N01, N11, N = { 0,0,0 }, A, B, C, D;

    NL_CPOINT ** Pw;

    NL_CURVE cur;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check if surface is closed */

    if( N_SrfIsClosed( sur, NL_UDIR ) )
    {
        degen = NL_YES;

        for ( j = 0; j <= m; j++ )
        {
            for ( i = 1; i <= n; i++ )
            {
                N_DistCptCpt( Pw[i - 1][j], Pw[i][j], &d );

                if( d GT NL_MTOL )
                {
                    degen = NL_NO;
                    break;
                }
            }

            if( degen EQ NL_NO )
                break;
        }

        if( degen EQ NL_YES )
        {
            N_SrfGetParameterBounds( sur, &us, &ue, &vs, &ve );

            N_CrvInitArrays( &cur );
            error = N_SrfExtractIsoCrv( sur, us, NL_VDIR, &cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( N_CrvIsLine( &cur, eps ) )
                *flt = NL_YES;
            else
            {
                *flt = NL_NO;
                *dir = NL_UDIR;
            }

            NL_OUT;
        }

        *flt = NL_NO;
        *dir = NL_VDIR;
        NL_OUT;
    }

    if( N_SrfIsClosed( sur, NL_VDIR ) )
    {
        degen = NL_YES;

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 1; j <= m; j++ )
            {
                N_DistCptCpt( Pw[i][j - 1], Pw[i][j], &d );

                if( d GT NL_MTOL )
                {
                    degen = NL_NO;
                    break;
                }
            }

            if( degen EQ NL_NO )
                break;
        }

        if( degen EQ NL_YES )
        {
            N_SrfGetParameterBounds( sur, &us, &ue, &vs, &ve );

            N_CrvInitArrays( &cur );
            error = N_SrfExtractIsoCrv( sur, vs, NL_UDIR, &cur, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            if( N_CrvIsLine( &cur, eps ) )
                *flt = NL_YES;
            else
            {
                *flt = NL_NO;
                *dir = NL_VDIR;
            }

            NL_OUT;
        }

        *flt = NL_NO;
        *dir = NL_UDIR;
        NL_OUT;
    }

    /* Initialize */

    *flt = NL_YES;

    dist = N_AllocReal2dArray( n, m, &SL );

    if( dist EQ NULL )
        NL_QUIT;

    SD = N_AllocPt2dArray( 1, 1, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    /* Get the four corner points */

    N_CPtToPtEuclid( Pw[0][0], &P00 );
    N_CPtToPtEuclid( Pw[n][0], &P10 );
    N_CPtToPtEuclid( Pw[0][m], &P01 );
    N_CPtToPtEuclid( Pw[n][m], &P11 );

    /* Check for degenerate boundaries */

    if( (N_SrfIsSingular( sur, NL_LEFT )AND N_SrfIsSingular( sur, NL_RIGHT ))OR( N_SrfIsSingular( sur, NL_BOTTOM )AND N_SrfIsSingular( sur, NL_TOP ) ) )
    {
        N_DistPtPt( P00, P10, &a );
        N_DistPtPt( P01, P11, &b );
        N_DistPtPt( P00, P01, &c );
        N_DistPtPt( P10, P11, &d );
        *flt = NL_NO;

        if( a LT NL_MTOL AND b LT NL_MTOL AND c LT NL_MTOL AND d LT NL_MTOL )
        {
            if( N_SrfIsDegen( sur ) )
            {
                *flt = NL_YES;
                NL_OUT;
            }

            *dir = NL_UDIR;
        }
        else if( a LT NL_MTOL AND b LT NL_MTOL )
        {
            *dir = NL_UDIR;
        }
        else
        {
            if( c LT NL_MTOL AND d LT NL_MTOL )
                *dir = NL_VDIR;
        }

        NL_OUT;
    }

    /* Get plane of the four corner points */

    N_Combine4Pts( 0.25, P00, 0.25, P10, 0.25, P01, 0.25, P11, &P );

    N_VectorDiff( P10, P00, &A );
    N_VectorDiff( P01, P00, &B );
    N_VectorCross( A, B, &N00 );

    N_VectorDiff( P11, P10, &C );
    N_VectorCross( A, C, &N10 );

    N_VectorDiff( P11, P01, &D );
    N_VectorCross( D, B, &N01 );

    N_VectorCross( D, C, &N11 );

    for ( i = 0; i < 2; i++ )
    {
        N_VectorMagnitude( N00, &a );
        N_VectorMagnitude( N10, &b );
        N_VectorMagnitude( N01, &c );
        N_VectorMagnitude( N11, &d );

        max = a;
        A = N00;

        if( b GT max )
        {
            max = b;
            A = N10;
        }

        if( c GT max )
        {
            max = c;
            A = N01;
        }

        if( d GT max )
        {
            max = d;
            A = N11;
        }

        N_VectorDot( A, N00, &dot );

        if( dot LT 0.0 )
            N_VectorScale( N00, -1.0, &N00 );

        N_VectorDot( A, N10, &dot );

        if( dot LT 0.0 )
            N_VectorScale( N10, -1.0, &N10 );

        N_VectorDot( A, N01, &dot );

        if( dot LT 0.0 )
            N_VectorScale( N01, -1.0, &N01 );

        N_VectorDot( A, N11, &dot );

        if( dot LT 0.0 )
            N_VectorScale( N11, -1.0, &N11 );

        N_Combine4Pts( 0.25, N00, 0.25, N10, 0.25, N01, 0.25, N11, &N );

        error = N_VectorNormalizeRef( &N );

        if( error EQ NL_NO )
            break;

        else if( i EQ 1 )
            NL_OUT;

        else
        {
            N_SrfGetParameterBounds( sur, &us, &ue, &vs, &ve );
            du = 0.01 *( ue - us );
            dv = 0.01 *( ve - vs );
            us += du;
            ue -= du;
            vs += dv;
            ve -= dv;
            error = N_SrfEvalPtPtDerivNormalFast( sur, us, vs, NL_LEFT, NL_LEFT, &Q, &B, &C, &N00, SD );
            error = N_SrfEvalPtPtDerivNormalFast( sur, us, ve, NL_LEFT, NL_LEFT, &Q, &B, &C, &N01, SD );
            error = N_SrfEvalPtPtDerivNormalFast( sur, ue, vs, NL_LEFT, NL_LEFT, &Q, &B, &C, &N10, SD );
            error = N_SrfEvalPtPtDerivNormalFast( sur, ue, ve, NL_LEFT, NL_LEFT, &Q, &B, &C, &N11, SD );
        }
    }

    N_VectorDot( P, N, &du );
    N_PtToXYZ( N, &a, &b, &c );
    d = -du;

    /* Check flatness of control points */

    flat = NL_YES;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &P );
            N_DistSignedPtPlane( a, b, c, d, P, &dist[i][j] );

            dist[i][j] = fabs( dist[i][j] );

            if( dist[i][j]GT eps )
                flat = NL_NO;
        }
    }

    /* Check straightness of boundary curves */

    if( flat EQ NL_YES )
    {
        maxu = maxv = 0.0;
        straight = NL_YES;

        for ( i = 1; i < n; i++ )
        {
            N_CPtToPtEuclid( Pw[i][0], &P );

            error = N_DistPtLineSeg( P, P00, P10, &du );

            if( error EQ NL_YES )
                NL_OUT;

            if( du GT maxu )
                maxu = du;

            N_CPtToPtEuclid( Pw[i][m], &P );

            error = N_DistPtLineSeg( P, P01, P11, &du );

            if( error EQ NL_YES )
                NL_OUT;

            if( du GT maxu )
                maxu = du;
        }

        for ( j = 1; j < m; j++ )
        {
            N_CPtToPtEuclid( Pw[0][j], &P );

            error = N_DistPtLineSeg( P, P00, P01, &dv );

            if( error EQ NL_YES )
                NL_OUT;

            if( dv GT maxv )
                maxv = dv;

            N_CPtToPtEuclid( Pw[n][j], &P );

            error = N_DistPtLineSeg( P, P10, P11, &dv );

            if( error EQ NL_YES )
                NL_OUT;

            if( dv GT maxv )
                maxv = dv;
        }

        if( maxu GT eps OR maxv GT eps )
            straight = NL_NO;
    }

    /* Determine which direction to subdivide */

    if( flat EQ NL_NO OR straight EQ NL_NO )
    {
        if( flat EQ NL_NO )
        {
            du = 0.0;

            for ( j = 0; j <= m; j++ )
            {
                min = max = dist[0][j];

                for ( i = 0; i <= n; i++ )
                {
                    if( dist[i][j]LT min )
                        min = dist[i][j];

                    if( dist[i][j]GT max )
                        max = dist[i][j];
                }
                du += max - min;
            }
            du /= ((NL_REAL)m + 1.0);

            dv = 0.0;

            for ( i = 0; i <= n; i++ )
            {
                min = max = dist[i][0];

                for ( j = 0; j <= m; j++ )
                {
                    if( dist[i][j]LT min )
                        min = dist[i][j];

                    if( dist[i][j]GT max )
                        max = dist[i][j];
                }
                dv += max - min;
            }
            dv /= ((NL_REAL)n + 1.0);

            if( du GT dv )
                *dir = NL_VDIR;
            else
                *dir = NL_UDIR;
        }
        else if( straight EQ NL_NO )
        {
            if( maxu GT maxv )
                *dir = NL_VDIR;
            else
                *dir = NL_UDIR;
        }

        *flt = NL_NO;
    }

    /* NL_END NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfIsFlat */

/*******************************************************************//**


   DESCRIPTION:

     Given a surface object, this routine allocates memory to store
     control points and knots. A typical calling example is:

       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       N_AllocSrfArrays(&sur,n,m,p,q,r,s,&S);

     where  <n,p,r> and  <m,q,s>  are the usual surface parameters. 
     Since the declaration "NL_SURFACE sur"  defines the data type and 
     allocates  memory, memory  is needed to  store the control net 
     and knot vector objects only.


   ACCESS:
   
     sur , in/out ,  NURBS surface data type
     n,m , input  ,  Highest indexes in control point array
     p,q , input  ,  Degrees of the surface
     r,s , input  ,  Highest indexes in knot vector arrays
     S   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllocSrfArrays
 (NL_SURFACE *sur, /* in : target surface */
  NL_INDEX    n,   /* in : highest U control point index */
  NL_INDEX    m,   /* in : highest V control point index */
  NL_DEGREE   p,   /* in : u degree */
  NL_DEGREE   q,   /* in : v degree */
  NL_INDEX    r,   /* in : highest U knot index */
  NL_INDEX    s,   /* in : highest V knot index */
  NL_STACKS  *S )  /* in : sur's stack */
{
    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    /* Allocate memory */

    net = N_AllocCNetAndArrays( n, m, S );

    if( net EQ NULL )
        return (1);

    knu = N_AllocKnotVectorAndArray( r, S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVectorAndArray( s, S );

    if( knv EQ NULL )
        return (1);

    /* Build surface structure */

    N_SrfFromCNetAndKnotVectors( sur, net, p, q, knu, knv );

    /* Exit */

    return (0);
} /* end N_AllocSrfArrays */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if memory is needed to store a surface.
     If the surface is initiaized to the NULL surface  (via N_SrfInitArrays()),
     memory is  allocated. If not, the  routine checks if enough memory 
     is available. A typical calling example is:

       NL_SURFACE  sur;
       NL_INDEX    n, m, r, s;
       NL_DEGREE   p, q;
       NL_STRING   rname;
       NL_STACKS   S;
       ...
       (get n, m, p, q, r, s and rname);
       ...
       N_SrfInitArrays(&sur);
       N_SrfSizeArrays(&sur,n,m,p,q,r,s,rname,&S);

     IT IS  ASSUMED THAT MEMORY TO STORE THE NL_SURFACE STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     sur         , in/out ,  NURBS surface to be created
     n,m,p,q,r,s , input  ,  Usual surface parameters
     rname       , input  ,  Routine name
     S           , input  ,  sur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfSizeArrays */
NL_FLAG N_SrfSizeArrays
 (NL_SURFACE *sur,   /* in : target sur */
  NL_INDEX    n,     /* in : highest U control point index */
  NL_INDEX    m,     /* in : highest V control point index */
  NL_DEGREE   p,     /* in : u degree */
  NL_DEGREE   q,     /* in : v degree */
  NL_INDEX    r,     /* in : highest U knot index */
  NL_INDEX    s,     /* in : highest V knot index */
  NL_STRING   rname, /* in : Routine making request's name */
  NL_STACKS   *S )   /* in : sur's stack */
{
    NL_FLAG error;

    /* See if memory is needed */

    /* when sur is initialized to NULL */
    if( N_SrfAreArraysNULL( sur ) )
    {
        /* allocate memory in sur for input size parameters */
        error = N_AllocSrfArrays( sur, n, m, p, q, r, s, S );

        if( error EQ 1 )
            return (1);
    }
    else
    {
        /* check current sur size values are greater than or equal to given size values */
        error = N_SrfIsSized( sur, n, m, r, s, rname );

        if( error EQ 1 )
        {
            /* allocate memory in sur for input size parameters */
            error = N_AllocSrfArrays( sur, n, m, p, q, r, s, S );

            if( error EQ 1 )
                return (1);
        }

        /* store input size values in sur */
        N_SrfSetSizeIndices( sur, n, m, p, q, r, s );
    }

    /* Exit */

    return (0);
} /* end N_SrfSizeArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine copies a given surface into a new surface.
     A typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_STACKS   S;
       ...
       (define surP);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfCopy(&surP,&surQ,&S);

     If  the  surface is  initialized to  the empty  surface (NULL), 
     memory will be allocated  for surQ.  Otherwise,  it is  assumed 
     that memory is already available.


   ACCESS:
   
     surP , input  ,  NURBS surface to be copied
     surQ , output ,  Copied surface
     S    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfCopy */
NL_FLAG N_SrfCopy( NL_SURFACE *surP, NL_SURFACE *surQ, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfCopy");

    NL_FLAG error;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_CPOINT ** Pw, ** Qw;

    NL_REAL *UP, *VP, *UQ, *VQ;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );

    /* See if memory is needed */

    error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );

    /* Copy the surface */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            N_CopyCPt( Pw[i][j], &Qw[i][j] );
    }

    for ( i = 0; i <= r; i++ )
        UQ[i] = UP[i];

    for ( j = 0; j <= s; j++ )
        VQ[j] = VP[j];

    /* Exit */

    return (0);
} /* end N_SrfCopy */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  extracts the  coordinate functions wx(u,v), 
     wy(u,v), wz(u,v) and w(u,v) from a NURBS surface. If the surface is 
     non-rational, no w(u,v) is returned. A typical calling example is:

       NL_SURFACE  sur;
       NL_SFUN     wx, wy, wz, w;
       NL_STACKS   S;
       ...
       (define sur);
       ...
       N_SFuncInitArrays(&wx);
       N_SFuncInitArrays(&wy);
       N_SFuncInitArrays(&wz);
       N_SFuncInitArrays(&w );
       N_SrfGetCoordFuncs(&sur,&wx,&wy,&wz,&w,&S);

     If wx, wy, wz and w  are initialized to NULL,  memory is allocated. 
     Otherwise, it is assumed that memory is already available.


   ACCESS:
   
     sur , input  ,  NURBS surface
     wx  , output ,  X-coordinate function
     wy  , output ,  Y-coordinate function
     wz  , output ,  Z-coordinate function 
     w   , output ,  W-coordinate function (if rational)
     S   , input  ,  Stack of wx, wy, wz and w


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfGetCoordFuncs( NL_SURFACE *sur, NL_SFUN *wx, NL_SFUN *wy, NL_SFUN *wz, NL_SFUN *w, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfGetCoordFuncs");

    NL_FLAG error, rat = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL ** fx, ** fy, ** fz, ** fw = NULL, *U, *V, *UX, *VX, *UY, *VY, *UZ, *VZ, *UW = NULL, *VW = NULL, a;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* See if memory is needed */

    error = N_SFuncSizeArrays( wx, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        return (1);
    N_SFuncGetKnots( wx, &fx, &UX, &VX );

    error = N_SFuncSizeArrays( wy, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        return (1);
    N_SFuncGetKnots( wy, &fy, &UY, &VY );

    error = N_SFuncSizeArrays( wz, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        return (1);
    N_SFuncGetKnots( wz, &fz, &UZ, &VZ );

    if( N_IsSrfRat( sur ) )
    {
        error = N_SFuncSizeArrays( w, n, m, p, q, r, s, rname, S );

        if( error EQ NL_YES )
            return (1);

        N_SFuncGetKnots( w, &fw, &UW, &VW );

        rat = NL_YES;
    }

    /* Define output entities */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToWxWyWz( Pw[i][j], &fx[i][j], &fy[i][j], &fz[i][j], &a );

            if( rat EQ NL_YES )
                fw[i][j] = a;
        }
    }

    for ( i = 0; i <= r; i++ )
    {
        UX[i] = U[i];
        UY[i] = U[i];
        UZ[i] = U[i];

        if( rat EQ NL_YES )
            UW[i] = U[i];
    }

    for ( j = 0; j <= s; j++ )
    {
        VX[j] = V[j];
        VY[j] = V[j];
        VZ[j] = V[j];

        if( rat EQ NL_YES )
            VW[j] = V[j];
    }

    /* Exit */

    return (0);
} /* end N_SrfGetCoordFuncs */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  extracts the numerator and the denominator
     from  a  NURBS  surface. If  the  surface  is  non-rational,  no 
     denominator is returned. A typical calling example is:

       NL_SURFACE  sur, num;
       NL_SFUN     den;
       NL_STACKS   S;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&num);
       N_SFuncInitArrays(&den);
       N_SrfGetNumAndDen(&sur,&num,&den,&S);

     If  num and  den are  initialized to NULL, memory is  allocated. 
     Otherwise, it is assumed that memory is already available.


   ACCESS:
   
     sur , input  ,  NURBS surface
     num , output ,  Numerator of sur
     den , output ,  Denominator of sur (if rational)
     S   , input  ,  num's and den's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfGetNumAndDen( NL_SURFACE *sur, NL_SURFACE *num, NL_SFUN *den, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfGetNumAndDen");

    NL_FLAG error, rat = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL ** fuv = NULL, *US, *VS, *UN, *VN, *UD = NULL, *VD = NULL, xw, yw, zw, w;

    NL_CPOINT ** Pw, ** Nw;

    /* Get local notation and set rational flag */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &US, &VS );

    if( N_IsSrfRat( sur ) )
        rat = NL_YES;

    /* See if memory is needed */

    error = N_SrfSizeArrays( num, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_SrfGetCPtsAndKnots( num, &Nw, &UN, &VN );

    if( rat EQ NL_YES )
    {
        error = N_SFuncSizeArrays( den, n, m, p, q, r, s, rname, S );

        if( error EQ NL_YES )
            return (1);

        N_SFuncGetKnots( den, &fuv, &UD, &VD );
    }

    /* Define output entities */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToWxWyWz( Pw[i][j], &xw, &yw, &zw, &w );
            N_CPtFromWxWyWz( xw, yw, zw, NL_NOW, &Nw[i][j] );

            if( rat EQ NL_YES )
                fuv[i][j] = w;
        }
    }

    for ( i = 0; i <= r; i++ )
    {
        UN[i] = US[i];

        if( rat EQ NL_YES )
            UD[i] = US[i];
    }

    for ( j = 0; j <= s; j++ )
    {
        VN[j] = VS[j];

        if( rat EQ NL_YES )
            VD[j] = VS[j];
    }

    /* Exit */

    return (0);
} /* end N_SrfGetNumAndDen */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes a  NURBS surface given its numerator and 
     denominator. If the surface is non-rational, the denominator is not
     used; it is assumed to be NULL. A typical calling example is:

       NL_SURFACE  sur, num;
       NL_SFUN     den;
       NL_STACKS   S;
       ...
       (define num and den);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSrfFromNumAndDen(&num,&den,&sur,&S); (    RATIONAL)
       N_CreateSrfFromNumAndDen(&num,NULL,&sur,&S); (NON-RATIONAL)

     If sur is initialized  to NULL, memory is  allocated. Otherwise, it 
     is assumed that  memory is already available. THE NUMERATOR NL_SURFACE
     AND THE DENOMINATOR NL_FUNCTION MUST BE DEFINED CONSISTENTLY, I.E. THE
     CONTROL NL_POINT/VALUE INDEXES, THE KNOTS AND THE DEGREES  MUST BE THE 
     SAME.


   ACCESS:
   
     num , input  ,  Numerator of sur
     den , input  ,  Denominator of sur:
                       != NULL: rational case
                        = NULL: non-rational case
     sur , output ,  NURBS surface
     S   , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSrfFromNumAndDen( NL_SURFACE *num, NL_SFUN *den, NL_SURFACE *sur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSrfFromNumAndDen");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, nn, mn, rn, sn, nd, md, rd, sd;

    NL_DEGREE pn, qn, pd, qd;

    NL_REAL ** fuv = NULL, *U, *V, *US, *VS, xw, yw, zw, w;

    NL_CPOINT ** Pw, ** Nw;

    /* Get local notation and check consistency */

    N_SrfGetCPtsDegreesAndKnots( num, &nn, &mn, &Nw, &pn, &qn, &rn, &sn, &U, &V );

    if( den NEQ NULL )
    {
        N_SFuncGetComponents( den, &nd, &md, &fuv, &pd, &qd, &rd, &sd, &U, &V );

        if( nn NEQ nd OR mn NEQ md OR pn NEQ pd )
            NL_ERROR( NL_INP_ERR );

        if( qn NEQ qd OR rn NEQ rd OR sn NEQ sd )
            NL_ERROR( NL_INP_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, nn, mn, pn, qn, rn, sn, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &US, &VS );

    /* Define surface */

    for ( i = 0; i <= nn; i++ )
    {
        for ( j = 0; j <= mn; j++ )
        {
            N_CPtToWxWyWz( Nw[i][j], &xw, &yw, &zw, &w );

            if( den NEQ NULL )
                w = fuv[i][j];
            else
                w = NL_NOW;

            N_CPtFromWxWyWz( xw, yw, zw, w, &Pw[i][j] );
        }
    }

    for ( i = 0; i <= rn; i++ )
        US[i] = U[i];

    for ( j = 0; j <= sn; j++ )
        VS[j] = V[j];

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateSrfFromNumAndDen */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine saves surface definition data in a file. 
     The data file is arranged as follows:

           n m               --> highest indexes in cp array
           p q               --> degrees    
           rat               --> rationality (0-no,1-yes)
           x00 y00 z00 (w00) -->
           x01 y01 z01 (w01) -->
           .                 --> 
           .                 --> 
           .                 -->
           x0m y0m z0m (w0m) -->
           x10 y10 z10 (w10) --> xyz(w) components of cp's
           x11 y11 z11 (w11) -->
           .                 -->
           .                 -->
           .                 --> 
           xnm ynm znm (wnm) -->
           u0                -->
           u1                -->
           .                 -->
           .                 --> u-knots
           .                 -->
           ur                -->
           v0                -->
           v1                -->
           .                 -->
           .                 --> v-knots
           .                 -->
           vs                -->

     The  data file is  named as specified in the  argument list. A
     typical calling example is:

       NL_SURFACE  sur;
       TCHAR* fname;
       ...
       (get file name fname);
       ...
       N_WriteSrf(&sur,fname);


   ACCESS:
   
     sur   , input  ,  NURBS surface to be saved
     fname , output ,  Name of the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_WriteSrf( NL_SURFACE *sur, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_WriteSrf");

    NL_FLAG error = NL_NO;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Call file pointer counterpart */

    error = N_SrfWriteToFile( sur, fptr );

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_WriteSrf */

/*******************************************************************//**


   DESCRIPTION:

     This routine outputs an array of (index) k surfaces into a file
     The  data file is  named as specified in the  argument list. A
     typical calling example is:

       NL_SURFACE  **srfs;
       TCHAR* fname;
       NL_INDEX    k
       ...
       (get file name fname);
       ( get surfaces list and index k)
       ...
       N_WriteSrfArray(srfs, k, fname);


   ACCESS:
   
     srfs   , input  ,  array of NURBS NL_SURFACE pointers to be saved
     k      , input  ,  index of surfaces
     fname  , output ,  Name of the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_WriteSrfArray( NL_SURFACE ** srfs, NL_INDEX k, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_WriteSrfArray");

    NL_FLAG error = NL_NO;

    FILE *fptr;

    NL_INDEX i;

    /* Open file */

    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    N_FPRINTF( fptr, _T("%ld\n"), k );

    for ( i = 0; i <= k; i++ )
    {
        error = N_SrfWriteToFile( srfs[i], fptr );
    }

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_WriteSrfArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prints surface  definition data out to the 
     standard output device. The print is arranged as follows:

           n m               --> highest indexes in cp array
           p q               --> degrees    
           rat               --> rationality (0-no,1-yes)

           <stop>            --> hit any key to continue
  
           x00 y00 z00 (w00) -->
           x01 y01 z01 (w01) -->
           .                 --> 
           .                 --> 
           .                 -->
           x0m y0m z0m (w0m) -->
           x10 y10 z10 (w10) --> xyz(w) components of cp's
           x11 y11 z11 (w11) -->
           .                 -->
           .                 -->
           .                 --> 
           xnm ynm znm (wnm) -->

           <stop>            --> hit any key to continue

           u0                -->
           u1                -->
           .                 -->
           .                 --> u-knots
           .                 -->
           ur                -->

           <stop>            --> hit any key to continue

           v0                -->
           v1                -->
           .                 -->
           .                 --> v-knots
           .                 -->
           vs                -->

     This routine allows the programmer to quickly check a surface's
     control points and knots, as well as the appropriate indexes. A
     typical calling example is:

       NL_SURFACE  sur;
       ...
       N_PrintSrfData(&sur);


   ACCESS:
   
     sur , input  ,  NURBS surface to be printed


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PrintSrfData( NL_SURFACE *sur )
{
    NL_PRIVATE NL_STRING rname = _T("N_PrintSrfData");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_CPOINT ** Pw;

    NL_REAL *U, *V, wx, wy, wz, w;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* Get rational flag */

    if( N_IsSrfRat( sur ) )
        rat = NL_YES;
    else
        rat = NL_NO;

    /* Print surface data */

    N_FPRINTF( stdout, _T("%ld %ld\n"), n, m );
    N_FPRINTF( stdout, _T("%hd %hd\n"), p, q );
    N_FPRINTF( stdout, _T("%hd\n"), rat );
    NL_PAUSE;

    switch( rat )
    {
        case NL_NO: /* Non-rational */
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_CPtToWxWyWz( Pw[i][j], &wx, &wy, &wz, &w );
                    N_FPRINTF( stdout, _T("%18.16f %18.16f %18.16f\n"), wx, wy, wz );
                }
                NL_PAUSE;
            }
            break;

        case NL_YES: /* Rational */
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_CPtToWxWyWz( Pw[i][j], &wx, &wy, &wz, &w );
                    N_FPRINTF( stdout, _T("%18.16f %18.16f %18.16f %18.16f\n"), wx, wy, wz, w );
                }
                NL_PAUSE;
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    NL_PAUSE;

    for ( i = 0; i <= r; i++ )
        N_FPRINTF( stdout, _T("%18.16f\n"), U[i] );

    NL_PAUSE;

    for ( j = 0; j <= s; j++ )
        N_FPRINTF( stdout, _T("%18.16f\n"), V[j] );

    /* Exit */

    EXIT:

    return (error);
} /* end N_PrintSrfData */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine creates a surface from data saved in a 
     file. The data file is assumed to be arranged as follows:

           n m               --> highest indexes in cp array
           p q               --> degrees    
           rat               --> rationality (0-no,1-yes)
           x00 y00 z00 (w00) -->
           x01 y01 z01 (w01) -->
           .                 --> 
           .                 --> 
           .                 -->
           x0m y0m z0m (w0m) -->
           x10 y10 z10 (w10) --> xyz(w) components of cp's
           x11 y11 z11 (w11) -->
           .                 -->
           .                 -->
           .                 --> 
           xnm ynm znm (wnm) -->
           u0                -->
           u1                -->
           .                 -->
           .                 --> u-knots
           .                 -->
           ur                -->
           v0                -->
           v1                -->
           .                 -->
           .                 --> v-knots
           .                 -->
           vs                -->

     If memory is available, the data is copied into the  approriate
     members  of   the   surface  structure.  Otherwise,  memory  is 
     allocated first. A typical calling example is:

       NL_SURFACE  sur;
       TCHAR*  fname;
       NL_STACKS   S;
       ...
       (get fname);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSrfFromDataFile(&sur,fname,NL_YES,&S);

     IF THE NL_FLAG chk IS SET TO NL_YES, THE FOLLOWING CHECKS ARE DONE:
       (1) CONSISTENCY, I.E. r = n+p+1, s = m+q+1;
       (2) DEGREES ARE LESS THEN THE NL_MAXIMUM ALLOWED DEGREES;
       (3) WEIGHTS ARE IN THE ALLOWED RANGE; AND
       (4) INTERNAL KNOT MULTIPLICITIES ARE <= THE NL_DEGREE.
     IF chk = NL_YES, THE FOLLOWING SIMPLIFICATION IS PERFORMED:
       (1) IF ALL THE  WEIGHTS ARE  EQUAL, THE  NL_SURFACE IS CONVERTED 
           INTO A NON-RATIONAL NL_SURFACE.
     IF chk = NL_NO, THE NL_SURFACE IS TAKEN AS IS!!!!
     Rational surfaces are stored, where the control points are read and
     written in homogeneous  format   wX, wY, wZ, W. 
     They can be converted to Euclidean by dividing thru by the weight.


   ACCESS:
   
     sur   , in/out ,  NURBS surface to be created
     fname , input  ,  Name of the file
     chk   , input  ,  Flag:
                         NL_YES: check surface
                         NL_NO : do not check surface; use it as it is
     S     , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSrfFromDataFile( NL_SURFACE *sur, TCHAR* fname, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSrfFromDataFile");

    NL_FLAG error = NL_NO;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("r") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Call file pointer counterpart */

    error = N_SrfReadFromFile( sur, fptr, chk, S );

    /* Exit */
    N_FileClose( fptr );

    EXIT:

    return (error);
} /* end N_CreateSrfFromDataFile */

/**********************************************************************/
/* N_ReadSrfArray: Input an array of surfaces from a file             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine inputs an array of (index) k surfaces from a file
     The  data file is  named as specified in the  argument list. A
     typical calling example is:

       NL_SURFACE  **srfs;
       TCHAR* fname;
       NL_INDEX    i;
       NL_STACKS   S;
       ...
       (get fname);
       ...
;
       N_ReadSrfArray(&srfs, &i, fname, NL_YES, &S);

   ACCESS:
   
     srfs   , output  ,  array of NURBS NL_SURFACE pointers to be read
     i      , output  ,  index of number of surfaces in srfs array
     fname  , input   ,  Name of the file to read


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ReadSrfArray( NL_SURFACE *** srfs, NL_INDEX *k, TCHAR* fname, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_ReadSrfArray");

    NL_FLAG error = NL_NO;

    FILE *fptr;

    NL_SURFACE ** surs = NULL;

    NL_INDEX i = 0, j;

    /* Open file */

    fptr = N_FileOpen( fname, _T("r") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Get parameters from the top */

	if (EOF == N_FSCANF(fptr, _T("%ld"), &i)) { NL_ERROR(NL_SCN_ERR); }

    surs = N_AllocArraySrfPtrsInit( i, NL_YES, S );

    /* Call file pointer counterpart */

    for ( j = 0; j <= i; j++ )
    {
        error = N_SrfReadFromFile( surs[j], fptr, chk, S );
    }

    /* Exit */
    N_FileClose( fptr );

    EXIT:
    *k = i;
    *srfs = surs;

    return (error);
}

/*******************************************************************//**


   DESCRIPTION:

     Given a surface object, this routine extracts the denominator and
     creates a surface function from it. A typical calling example is:

       NL_SURFACE  sur;
       NL_SFUN     sfn;
       NL_STACKS   S;
       ...
       (define surface);
       ...
       N_SFuncInitArrays(&sfn);
       N_SrfGetDenominatorFunc(&sur,&sfn,&S);

     Since  the declarations  "NL_SURFACE sur" and  "NL_SFUN sfn" define the 
     data types and allocate  memory, only the pointers are passed in. 
     If sfn is  initialized  to  NULL,  memory is  allocated  locally.
     Otherwise, it is assumed  that memory has  been allocated  in the
     calling routine.


   ACCESS:
   
     sur , input  ,  NURBS surface
     sfn , in/out ,  Surface function
     S   , input  ,  sfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfGetDenominatorFunc */
NL_FLAG N_SrfGetDenominatorFunc( NL_SURFACE *sur, NL_SFUN *sfn, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfGetDenominatorFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, ** fuv, *UF, *VF;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* See if memory is needed */

    error = N_SFuncSizeArrays( sfn, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( sfn, &fuv, &UF, &VF );

    /* Copy data */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            N_CPtGetW( Pw[i][j], &fuv[i][j] );
    }

    for ( i = 0; i <= r; i++ )
        UF[i] = U[i];

    for ( j = 0; j <= s; j++ )
        VF[j] = V[j];

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfGetDenominatorFunc */

/*******************************************************************//**


   DESCRIPTION:

     Given a  surface  object, this  routine  maps  the  control net to
     Euclidean space. Memory for Euclidean points is allocated locally,
     however, the Euclidean  net  data  type  is  declared  (allocated)
     in the calling routine. A typical calling example is:

       NL_INDEX    ku, lu, kv, lv;
       NL_SURFACE  sur;
       NL_ENET     ntl;
       NL_STACKS   S;
       ...
       (define surface);
       ...
       N_SrfGetENet(&sur,ku,lu,kv,lv,&ntl,&S);

     Since  the  declarations  "NL_SURFACE sur" and  "NL_ENET ntl" define the 
     data types and allocate memory, only the pointers are passed in. 


   ACCESS:
   
     sur         , input  ,  NURBS surface
     ku,lu,kv,lv , input  ,  Start and  end  indexes in  uv-directions.
                             Only the  control  points  Pw[ku][kv],...,
                             Pw[lu][lv]  are   mapped.  The   Euclidean 
                             points   are   stored    in   P[0][0],...,
                             P[lu-ku][lv-kv].
     ntl         , output ,  Point net (MEMORY  TO  STORE  VERTICES  IS
                             ALLOCATED LOCALLY)
     S           , input  ,  ntl's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_SrfGetENet( NL_SURFACE *sur, NL_INDEX ku, NL_INDEX lu, NL_INDEX kv, NL_INDEX lv, NL_ENET *ntl, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfGetENet");

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw, *Pw2;

    NL_POINT ** P, *P2;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Check indexes */

    if( ku GT lu OR ku LT 0 OR lu GT n )
    {
        N_ErrSet( NL_IND_ERR, rname );
        return (1);
    }

    if( kv GT lv OR kv LT 0 OR lv GT m )
    {
        N_ErrSet( NL_IND_ERR, rname );
        return (1);
    }

    /* Map control points */

    P = N_AllocPt2dArray( lu - ku, lv - kv, S );

    if( P EQ NULL )
        return (1);

    /*   for ( i=ku; i <= lu; i++ ) 
    **   
    {
    **     for ( j=kv; j <= lv; j++ )
    **     
    {
    **       N_CPtToPtEuclid(Pw[i][j],&P[i-ku][j-kv]);
    **     
    }
    **   
    }
    */
    for ( i = ku; i <= lu; i++ )
    {
        Pw2 = &Pw[i][kv];
        P2 = P[i - ku];

        for ( j = kv; j <= lv; j++ )
        {
            N_CPtToPtEuclid( *Pw2, P2 );
            Pw2++;
            P2++;
        }
    }

    /* Build point net structure */

    N_ENetFromPts( ntl, lu - ku, lv - kv, P );

    /* Exit */

    return (0);
} /* end N_SrfGetENet */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  makes a  NURBS surface from its  coordinate
     functions wx(u,v), wy(u,v), wz(u,v) and w(u,v). If  the surface  is
     non-rational, w(u,v) = NULL. A typical calling example is:

       NL_SURFACE  sur;
       NL_SFUN     wx, wy, wz, w;
       NL_STACKS   S;
       ...
       (define wx, wy, wz and w);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSrfFromCoordFuncs(&wx,&wy,&wz,&w  ,&sur,&S); (    RATIONAL)
       N_CreateSrfFromCoordFuncs(&wx,&wy,&wz,NULL,&sur,&S); (NON-RATIONAL)

     If sur is  initialized to NULL, memory is  allocated. Otherwise, it 
     is  assumed  that  memory  is  already  available.  THE  COORDINATE 
     FUNCTIONS MUST BE FULLY COMPATIBLE, I.E. THE CONTROL VALUE INDEXES, 
     THE KNOTS AND THE DEGREES MUST BE THE SAME.


   ACCESS:
   
     wx  , input  ,  X-coordinate function
     wy  , input  ,  Y-coordinate function
     wz  , input  ,  Z-coordinate function
     w   , input  ,  W-coordinate function:
                       != NULL: rational
                        = NULL: non-rational
     sur , output ,  NURBS surface
     S   , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSrfFromCoordFuncs( NL_SFUN *wx, NL_SFUN *wy, NL_SFUN *wz, NL_SFUN *w, NL_SURFACE *sur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSrfFromCoordFuncs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, nx, mx, rx, sx, ny, my, ry, sy, nz, mz, rz, sz, nw, mw, rw, sw;

    NL_DEGREE px, qx, py, qy, pz, qz, pw, qw;

    NL_REAL ** fx, ** fy, ** fz, ** fw = NULL, *U, *V, *UX, *VX, *UY, *VY, *UZ, *VZ, *UW, *VW, a;

    NL_CPOINT ** Pw;

    /* Get local notation and check consistency */

    N_SFuncGetComponents( wx, &nx, &mx, &fx, &px, &qx, &rx, &sx, &UX, &VX );
    N_SFuncGetComponents( wy, &ny, &my, &fy, &py, &qy, &ry, &sy, &UY, &VY );
    N_SFuncGetComponents( wz, &nz, &mz, &fz, &pz, &qz, &rz, &sz, &UZ, &VZ );

    if( nx NEQ ny OR mx NEQ my OR px NEQ py )
        NL_ERROR( NL_INP_ERR );

    if( qx NEQ qy OR rx NEQ ry OR sx NEQ sy )
        NL_ERROR( NL_INP_ERR );

    if( nx NEQ nz OR mx NEQ mz OR px NEQ pz )
        NL_ERROR( NL_INP_ERR );

    if( qx NEQ qz OR rx NEQ rz OR sx NEQ sz )
        NL_ERROR( NL_INP_ERR );

    if( w NEQ NULL )
    {
        N_SFuncGetComponents( w, &nw, &mw, &fw, &pw, &qw, &rw, &sw, &UW, &VW );

        if( nx NEQ nw OR mx NEQ mw OR px NEQ pw )
            NL_ERROR( NL_INP_ERR );

        if( qx NEQ qw OR rx NEQ rw OR sx NEQ sw )
            NL_ERROR( NL_INP_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, nx, mx, px, qx, rx, sx, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Define surface */

    for ( i = 0; i <= nx; i++ )
    {
        for ( j = 0; j <= mx; j++ )
        {
            if( w NEQ NULL )
                a = fw[i][j];
            else
                a = NL_NOW;

            N_CPtFromWxWyWz( fx[i][j], fy[i][j], fz[i][j], a, &Pw[i][j] );
        }
    }

    for ( i = 0; i <= rx; i++ )
        U[i] = UX[i];

    for ( j = 0; j <= sx; j++ )
        V[j] = VX[j];

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateSrfFromCoordFuncs */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine converts a rational surface to a non-rational 
     surface by  simply setting  the fourth coordinates to  the special 
     value NL_NOW. This  conversion makes  sense only if the  weights are 
     known to be one. A typical calling example is:

       NL_SURFACE  sur;
       ...
       N_SrfRatToNonRat(&sur);

     After the conversion the surface is  considered  as a non-rational 
     surface, i.e. the weights are simply ignored.



   ACCESS:
   
     sur , in/out ,  NURBS surface


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfRatToNonRat( NL_SURFACE *sur )
{

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Convert to non-rational */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtSetW( NL_NOW, &Pw[i][j] );
        }
    }
} /* end N_SrfRatToNonRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine converts a non-rational surface to a rational 
     surface by simply setting the fourth coordinates to 1.0. A typical 
     calling example is:

       NL_SURFACE  sur;
       ...
       N_SrfNonRatToRat(&sur);

     After  the  conversion  the  surface is  considered  as a rational 
     surface, i.e. the weights are used in all computations.



   ACCESS:
   
     sur , in/out ,  NURBS surface


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfNonRatToRat( NL_SURFACE *sur )
{

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Convert to rational */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtSetW( 1.0, &Pw[i][j] );
        }
    }
} /* end N_SrfNonRatToRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes a set of surface definitions compatible,
     i.e. it  makes sure that all surfaces are rational or non-rational. 
     A typical calling example is:

       NL_SURFACE  **sur;
       NL_INDEX    k;
       ...
       (define array of sur);
       ...
       N_MakeSrfsRatCompatible(sur,k);

     After the  conversion all  surfaces are  either rational or  remain 
     non-rational.



   ACCESS:
   
     sur , in/out ,  Array of NURBS surfaces
     k   , input  ,  Highest index in array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_MakeSrfsRatCompatible( NL_SURFACE ** sur, NL_INDEX k )
{

    NL_FLAG allnra;

    NL_INDEX i;

    /* Check rationality */

    allnra = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( N_IsSrfRat( sur[i] ) )
        {
            allnra = NL_FALSE;
            break;
        }
    }

    /* If not all non-rational, make all surfaces rational */

    if( allnra EQ NL_FALSE )
    {
        for ( i = 0; i <= k; i++ )
        {
            if( NOT N_IsSrfRat( sur[i] ) )
            {
                N_SrfNonRatToRat( sur[i] );
            }
        }
    }
} /* end N_MakeSrfsRatCompatible */

/*******************************************************************//**


   DESCRIPTION:

     Given a surface object, this routine scales the knot vectors to a 
     given rectangle. A typical calling example is:

       NL_SURFACE    sur;
       NL_RECTANGLE  R;
       ...
       (get rectangle R);
       ...
       N_SrfReparamToInterval(&sur,R,NL_UDIR);

     The  knot vectors  are rescaled IN-PLACE, i.e. the original knots 
     are destroyed.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     R   , input  ,  Parameter rectangle
     dir , input  ,  Flag:
                       NL_UDIR : Rescale u-knot vector
                       NL_VDIR : Rescale v-knot vector
                       NL_UVDIR: Rescale both knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfReparamToInterval( NL_SURFACE *sur, NL_RECTANGLE R, NL_FLAG dir )
{

    NL_INDEX i, j, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, fac, a, b, c, d, u0, v0;

    /* Get local notation */

    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnots( sur, &r, &s, &U, &V );
    N_RectangleGetData( &R, &a, &b, &c, &d );

    /* Compute new knots */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        if( a NEQ U[0]OR b NEQ U[r] )
        {
            u0 = U[0];
            fac = (b - a) / (U[r] - U[0]);

            for ( i = 0; i <= p; i++ )
                U[i] = a;

            for ( i = p + 1; i <= r - p - 1; i++ )
                U[i] = fac * (U[i] - u0) + a;

            for ( i = r - p; i <= r; i++ )
                U[i] = b;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        if( c NEQ V[0]OR d NEQ V[s] )
        {
            v0 = V[0];
            fac = (d - c) / (V[s] - V[0]);

            for ( j = 0; j <= q; j++ )
                V[j] = c;

            for ( j = q + 1; j <= s - q - 1; j++ )
                V[j] = fac * (V[j] - v0) + c;

            for ( j = s - q; j <= s; j++ )
                V[j] = d;
        }
    }
} /* end N_SrfReparamToInterval */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates an NL_IGES file and writes  surfaces  to
     it as 128 Entities.  The NL_IGES file is named as  specified  in  the  
     argument list. The Start and Global sections of the NL_IGES file con-
     tain a minimum amount of information;  this  can  be  subsequently
     edited as desired. A typical calling example is:

       NL_SURFACE  **surs;
       TCHAR* fname;
       ...
       (get file name, fname, and create surfaces in surs);
       ...
       N_WriteSrfToIgesFile(surs,ns,fname);


   ACCESS:
   
     surs  , input  ,  Pointers to the NURBS surfaces to be written  to
                       the NL_IGES file. surs[i] is a pointer  to the i-th
                       surface
     ns    , input  ,  High index of surface  pointers  (there are ns+1
                       pointers in surs)
     fname , output ,  Name of the NL_IGES file (may  not  be more than 30
                       characters long)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_WriteSrfToIgesFile( NL_SURFACE ** surs, NL_INDEX ns, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_WriteSrfToIgesFile");
    NL_PRIVATE NL_STRING blank = _T(" ");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s, ii, jj, kk, npl, dline, pline;

    NL_INTEGER rat, uclo, vclo;

    NL_DEGREE p, q;

    NL_CPOINT ** Pw;

    NL_REAL *U, *V, x, y, z, w, maxc, d1, d2, d3, d4, d5, d6;

    NL_MINMAXBOX box;

    FILE *fptr = NULL;

    /* Check for input error and open file */

    if( ns LT 0 )
        NL_ERROR( NL_INP_ERR );
    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Write Start Section */

    N_FPRINTF( fptr, _T("Nurbs surfaces written out using the%36sS%7d\n"), blank, 1 );

    /* Write Global Section */

    maxc = 0.0;

    for ( ii = 0; ii <= ns; ii++ )
    {
        N_SrfGetBBox( surs[ii], &box );
        N_GetBBoxData( &box, &d1, &d2, &d3, &d4, &d5, &d6 );

        if( fabs( d1 )GT maxc )
            maxc = fabs( d1 );

        if( fabs( d2 )GT maxc )
            maxc = fabs( d2 );

        if( fabs( d3 )GT maxc )
            maxc = fabs( d3 );

        if( fabs( d4 )GT maxc )
            maxc = fabs( d4 );

        if( fabs( d5 )GT maxc )
            maxc = fabs( d5 );

        if( fabs( d6 )GT maxc )
            maxc = fabs( d6 );
    }

    maxc = 1.001 *maxc;

    N_FPRINTF( fptr, _T(",,8HNlib 7.0,30H%30s,1H ,1H ,%17sG%7d\n"), fname, blank, 1 );
    N_FPRINTF( fptr, _T("%3d,%3d,%2d,%3d,%3d,1H , %1.1f,1,2HIN,%37sG%7d\n"), 32, 38, 7, 308, 15, 1.0, blank, 2 );
    N_FPRINTF( fptr, _T("1, %5.3f,13H             , %18.13lf,%26sG%7d\n"), 0.001, NL_MTOL, blank, 3 );
    N_FPRINTF( fptr, _T("%22.10lf,1H ,1H ,%3d,0,%35sG%7d\n"), maxc, 9, blank, 4 );
    N_FPRINTF( fptr, _T("13H             ;%55sG%7d\n"), blank, 5 );

    /* Write the Directory Entries */
    pline = 1;
    dline = 1;

    for ( ii = 0; ii <= ns; ii++ )
    {
        /* Compute number of lines in Parameter Data Section */

        N_SrfGetArraySizes( surs[ii], &n, &m, &r, &s );

        npl = 3;                        /* integer data & trim bounds */
        npl += (r / 3 + 1);             /* u knots */
        npl += (s / 3 + 1);             /* v knots */
        npl += ((m + 1) * (n / 4 + 1)); /* weights */
        npl += ((m + 1) * (n + 1));     /* control points */

        /* Write the Directory Entry for this surface */

        N_FPRINTF( fptr, _T("%8d%8ld%8d%8d%8d%8d%8d%8d00000000D%7ld\n"), 128, pline, 0, 1, 4, 0, 0, 0, dline );
        dline += 1;
        N_FPRINTF( fptr, _T("%8d%8d%8ld%8ld%8d%8d%8dNURBSURF%8ldD%7ld\n"), 128, 1, (ii + 2) % 8, npl, 0, 0, 0, ii + 1, dline );
        dline += 1;

        pline += npl;
    }

    /* Write the Parameter Data Section */

    pline = 1;

    for ( ii = 0; ii <= ns; ii++ )
    {
        /* Get local notation */

        N_SrfGetCPtsDegreesAndKnots( surs[ii], &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

        /* Compute the Properties */

        if( N_SrfIsClosed( surs[ii], NL_UDIR ) )
            uclo = 1;
        else
            uclo = 0;

        if( N_SrfIsClosed( surs[ii], NL_VDIR ) )
            vclo = 1;
        else
            vclo = 0;

        rat = 1;

        if( N_IsSrfRat( surs[ii] ) )
        {
            for ( jj = 0; jj <= n; jj++ )
            {
                for ( kk = 0; kk <= m; kk++ )
                {
                    N_CPtGetW( Pw[jj][kk], &w );

                    if( w NEQ 1.0 )
                    {
                        rat = 0;
                        break;
                    }
                }

                if( rat EQ 0 )
                    break;
            }
        }

        /* Write the integer data */

        N_FPRINTF( fptr, _T("%3d,%5ld,%5ld,%3d,%3d,%2ld,%2ld,%2ld,%2d,%2d,%26s%7ldP%7ld\n"), 128, n, m, p, q, uclo, vclo, rat, 0, 0, blank, 2 * ii + 1, pline );
        pline += 1;

        /* Write the u-knots */

        for ( jj = 0; jj <= r; jj += 3 )
        {
            if( jj EQ r )
                N_FPRINTF( fptr, _T("%19.12lf,%44s %7ldP%7ld\n"), U[jj], blank, 2 * ii + 1, pline );

            else if( jj EQ r - 1 )
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[jj], U[jj + 1], blank, 2 *ii + 1, pline );

            else
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), U[jj], U[jj + 1], U[jj + 2], 2 *ii + 1, pline );

            pline += 1;
        }

        /* Write the v-knots */

        for ( jj = 0; jj <= s; jj += 3 )
        {
            if( jj EQ s )
                N_FPRINTF( fptr, _T("%19.12lf,%44s %7ldP%7ld\n"), V[jj], blank, 2 * ii + 1, pline );

            else if( jj EQ s - 1 )
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), V[jj], V[jj + 1], blank, 2 *ii + 1, pline );

            else
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), V[jj], V[jj + 1], V[jj + 2], 2 *ii + 1, pline );

            pline += 1;
        }

        /* Write the weights */

        if( rat EQ 1 )
        { /* non-rational */
            for ( jj = 0; jj <= m; jj++ )
                for ( kk = 0; kk <= n; kk += 4 )
                {
                    if( kk EQ n )
                        N_FPRINTF( fptr, _T("%13lf,%50s %7ldP%7ld\n"), 1.0, blank, 2 * ii + 1, pline );

                    else if( kk EQ n - 1 )
                        N_FPRINTF( fptr, _T("%13lf , %13lf,%34s %7ldP%7ld\n"), 1.0, 1.0, blank, 2 *ii + 1, pline );

                    else if( kk EQ n - 2 )
                        N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf,%18s %7ldP%7ld\n"), 1.0, 1.0, 1.0, blank, 2 *ii + 1, pline );

                    else
                        N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf , %13lf,   %7ldP%7ld\n"), 1.0, 1.0, 1.0, 1.0, 2 *ii + 1, pline );

                    pline += 1;
                }
        }
        else
        { /* rational */
            for ( jj = 0; jj <= m; jj++ )
                for ( kk = 0; kk <= n; kk += 4 )
                {
                    if( kk EQ n )
                    {
                        N_CPtGetW( Pw[kk][jj], &d1 );
                        N_FPRINTF( fptr, _T("%13.7lf,%50s %7ldP%7ld\n"), d1, blank, 2 * ii + 1, pline );
                    }
                    else if( kk EQ n - 1 )
                    {
                        N_CPtGetW( Pw[kk][jj], &d1 );
                        N_CPtGetW( Pw[kk + 1][jj], &d2 );
                        N_FPRINTF( fptr, _T("%13.7lf , %13.7lf,%34s %7ldP%7ld\n"), d1, d2, blank, 2 * ii + 1, pline );
                    }
                    else if( kk EQ n - 2 )
                    {
                        N_CPtGetW( Pw[kk][jj], &d1 );
                        N_CPtGetW( Pw[kk + 1][jj], &d2 );
                        N_CPtGetW( Pw[kk + 2][jj], &d3 );
                        N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf,%18s %7ldP%7ld\n"), d1, d2, d3, blank, 2 * ii + 1, pline );
                    }
                    else
                    {
                        N_CPtGetW( Pw[kk][jj], &d1 );
                        N_CPtGetW( Pw[kk + 1][jj], &d2 );
                        N_CPtGetW( Pw[kk + 2][jj], &d3 );
                        N_CPtGetW( Pw[kk + 3][jj], &d4 );
                        N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf , %13.7lf,   %7ldP%7ld\n"), d1, d2, d3, d4, 2 * ii + 1, pline );
                    }

                    pline += 1;
                }
        }

        /* Write the Euclidean control points */

        for ( jj = 0; jj <= m; jj++ )
            for ( kk = 0; kk <= n; kk++ )
            {
                N_CPtToXYZ( Pw[kk][jj], &x, &y, &z );
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), x, y, z, 2 * ii + 1, pline );

                pline += 1;
            }

        /* Write the u,v trim bounds */

        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[0], U[r], blank, 2 * ii + 1, pline );
        pline += 1;
        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf;%22s %7ldP%7ld\n"), V[0], V[s], blank, 2 * ii + 1, pline );
        pline += 1;
    }

    /* Write the Terminate Section */

    N_FPRINTF( fptr, _T("S%7dG%7dD%7ldP%7ld%40sT%7d\n"), 1, 5, dline - 1, pline - 1, blank, 1 );

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_WriteSrfToIgesFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if surface weights are equal or not. A 
     typical calling example is:

       NL_SURFACE  sur;
       ...
       if( N_SrfAreWeightsEqual(&sur) )  --> surface weights are equal;
     

   ACCESS:
   
     sur , input ,  NURBS surface


   RETURN CODES:

     NL_TRUE : Surface weights are equal
     NL_FALSE: Surface weights are NOT equal

   ***********************************************************************/

/* NL_BOOLEAN  N_SrfAreWeightsEqual */
NL_BOOLEAN N_SrfAreWeightsEqual( NL_SURFACE *sur )
{

    NL_INDEX i, j, n, m;

    NL_REAL w, wmin, wmax;

    NL_CPOINT ** Pw;

    /* Get net */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Get min and max weights */

    N_CPtGetW( Pw[0][0], &w );
    wmin = wmax = w;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtGetW( Pw[i][j], &w );

            if( w LT wmin )
                wmin = w;

            if( w GT wmax )
                wmax = w;
        }
    }

    /* Check equality */

    if( fabs( wmax - wmin )LT NL_WTOL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_SrfAreWeightsEqual */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine  maps a rational  surface to Euclidean space. 
     That is,  for each  control point Pw = (xw,yw,zw,w), it  computes a  
     new control point Qw = (x,y,z,NL_NOW). A typical calling example is:

       NL_SURFACE  sur;
       ...
       N_SrfMakeNonRat(&sur);


   ACCESS:
   
     sur , in/out ,  NURBS surface


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfMakeNonRat( NL_SURFACE *sur )
{

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get net */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Map to Euclidean space */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtNoW( Pw[i][j], &Pw[i][j] );
        }
    }
} /* end N_SrfMakeNonRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prunes a rational surface, i.e. it checks if 
     the weights are  equal, and if so, the  surface is converted into 
     non-rational form.

       NL_SURFACE  sur;
       ...
       N_SrfPruneRat(&sur);


   ACCESS:
   
     sur , in/out ,  NURBS surface


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfPruneRat( NL_SURFACE *sur )
{
    if( N_IsSrfRat( sur ) )
    {
        if( N_SrfAreWeightsEqual( sur ) )
            N_SrfMakeNonRat( sur );
    }
} /* end N_SrfPruneRat */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine checks if a NURBS surface is flat or not. Flat
     means that the  surface patch can be approximated, to within a user
     specified tolerance, by a  planar quadrilateral. This routine is an
     inexpensive  version of  N_SrfIsFlat and  determines the  direction of 
     subdivision in an approximative manner.  It also does not check for
     collinearity of the boundaries, as does N_SrfIsFlat. A typical calling
     example:
 
       NL_FLAG     flt, dir;
       NL_REAL     eps;
       NL_SURFACE  sur;
       ...
       (get sur and tolerance eps)
       ...
       N_SrfIsFlatCheap(&sur,eps,&flt,&dir);
     
 
   ACCESS:
   
     sur , input  ,  NURBS surface
     eps , input  ,  Flatness tolerance
     flt , output ,  Flatness indicator:
                       NL_YES: surface is flat
                       NL_NO : surface is NOT flat
     dir , output ,  Direction of subdivision 
                       NL_UDIR: subdivide at a v value
                       NL_VDIR: subdivide at a u value
 
 
   RETURN CODES:
 
     0 : No eror
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

/* NL_FLAG  N_SrfIsFlatCheap */
NL_FLAG N_SrfIsFlatCheap( NL_SURFACE *sur, NL_REAL eps, NL_FLAG *flt, NL_FLAG *dir )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL dis, a, b, c, d, lu, lv, ln;

    NL_POINT P00, P10, P01, P11, P, A, B;

    NL_VECTOR N, C, D;

    NL_CPOINT ** Pw;

    /* Check if surface is closed */

    if( N_SrfIsClosed( sur, NL_UDIR ) )
    {
        *flt = NL_NO;
        *dir = NL_VDIR;
        NL_OUT;
    }

    if( N_SrfIsClosed( sur, NL_VDIR ) )
    {
        *flt = NL_NO;
        *dir = NL_UDIR;
        NL_OUT;
    }

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Get approximating plane */

    N_CPtToPtEuclid( Pw[0][0], &P00 );
    N_CPtToPtEuclid( Pw[n][0], &P10 );
    N_CPtToPtEuclid( Pw[0][m], &P01 );
    N_CPtToPtEuclid( Pw[n][m], &P11 );

    N_Combine4Pts( 0.25, P00, 0.25, P10, 0.25, P01, 0.25, P11, &P );

    N_VectorCombine( 0.5, P00, 0.5, P10, &A );
    N_VectorCombine( 0.5, P10, 0.5, P11, &B );
    N_VectorDiff( A, P, &C );
    N_VectorDiff( B, P, &D );
    N_VectorMagnitude( C, &lv );
    N_VectorMagnitude( D, &lu );
    N_VectorCross( C, D, &N );
    N_VectorMagnitude( N, &ln );

    if( ln LT NL_MTOL )
    {
        *flt = NL_NO;

        if( lu GT lv )
            *dir = NL_VDIR;
        else
            *dir = NL_UDIR;

        NL_OUT;
    }

    error = N_PlanePtNormalToImplicit( P, N, &a, &b, &c, &d );

    if( error EQ NL_YES )
        NL_OUT;

    /* Check flatness */

    *flt = NL_YES;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &P );
            N_DistSignedPtPlane( a, b, c, d, P, &dis );

            if( fabs( dis )GT eps )
            {
                *flt = NL_NO;
                break;
            }
        }

        if( *flt EQ NL_NO )
            break;
    }

    if( lu GT lv )
        *dir = NL_VDIR;
    else
        *dir = NL_UDIR;

    /* NL_END NURBS and Exit */

    EXIT:

    return (error);
} /* end N_SrfIsFlatCheap */

/**********************************************************************/
/* N_SrfReadFromFile: Read surface from file (file pointer passed in)          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates a surface from data saved in a file.
     The file pointer is passed in.  The file is assumed to be  opened
     and correctly positioned to read the surface. The data is assumed
     to be arranged as follows:

           n m               --> highest indexes in cp array
           p q               --> degrees    
           rat               --> rationality (0-no,1-yes)
           x00 y00 z00 (w00) -->
           x01 y01 z01 (w01) -->
           .                 --> 
           .                 --> 
           .                 -->
           x0m y0m z0m (w0m) -->
           x10 y10 z10 (w10) --> xyz(w) components of cp's
           x11 y11 z11 (w11) -->
           .                 -->
           .                 -->
           .                 --> 
           xnm ynm znm (wnm) -->
           u0                -->
           u1                -->
           .                 -->
           .                 --> u-knots
           .                 -->
           ur                -->
           v0                -->
           v1                -->
           .                 -->
           .                 --> v-knots
           .                 -->
           vs                -->

     If memory is available, the data is copied into the  approriate
     members  of   the   surface  structure.  Otherwise,  memory  is 
     allocated first. A typical calling example is:

       NL_SURFACE  sur;
       FILE     *fptr;
       NL_STACKS   S;
       ...
       (open file, assign pointer, and position for surface read);
       ...
       N_SrfInitArrays(&sur);
       N_SrfReadFromFile(&sur,fptr,NL_YES,&S);

     IF THE NL_FLAG chk IS SET TO NL_YES, THE FOLLOWING CHECKS ARE DONE:
       (1) CONSISTENCY, I.E. r = n+p+1, s = m+q+1;
       (2) DEGREES ARE LESS THEN THE NL_MAXIMUM ALLOWED DEGREES;
       (3) WEIGHTS ARE IN THE ALLOWED RANGE; AND
       (4) INTERNAL KNOT MULTIPLICITIES ARE <= THE NL_DEGREE.
     IF chk = NL_YES, THE FOLLOWING SIMPLIFICATION IS PERFORMED:
       (1) IF ALL THE  WEIGHTS ARE  EQUAL, THE  NL_SURFACE IS CONVERTED 
           INTO A NON-RATIONAL NL_SURFACE.
     IF chk = NL_NO, THE NL_SURFACE IS TAKEN AS IS!!!!
     Rational surfaces are stored, where the control points are read and
     written in homogeneous  format   wX, wY, wZ, W. 
     They can be converted to Euclidean by dividing thru by the weight.


   ACCESS:
   
     sur   , in/out ,  NURBS surface to be created
     fptr  , input  ,  Pointer to data file
     chk   , input  ,  Flag:
                         NL_YES: check surface
                         NL_NO : do not check surface; use it as it is
     S     , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReadFromFile( NL_SURFACE *sur, FILE *fptr, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReadFromFile");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_CPOINT ** Pw;

    NL_REAL *U, *V, x, y, z, wx, wy, wz, w;

    /* Get parameters from the top */

	if (EOF == N_FSCANF(fptr, _T("%ld%ld"), &n, &m)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd%hd"), &p, &q)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd"), &rat)) { NL_ERROR(NL_SCN_ERR); }

    r = n + p + 1;
    s = m + q + 1;

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Read in data */

    switch( rat )
    {
        case NL_NO: /* Non-rational */
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
					if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &x, &y, &z)) { NL_ERROR(NL_SCN_ERR); }
                    N_CPtFromWxWyWz( x, y, z, NL_NOW, &Pw[i][j] );
                }
            }
            break;

        case NL_YES: /* Rational */
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
					if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf%lf"), &wx, &wy, &wz, &w)) { NL_ERROR(NL_SCN_ERR); }
                    N_CPtFromWxWyWz( wx, wy, wz, w, &Pw[i][j] );
                }
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= r; i++ )
	if (EOF == N_FSCANF(fptr, _T("%lf"), &U[i])) { NL_ERROR(NL_SCN_ERR); }

    for ( j = 0; j <= s; j++ )
	if (EOF == N_FSCANF(fptr, _T("%lf"), &V[j])) { NL_ERROR(NL_SCN_ERR); }

    /* Check surface and prune */

    if( chk EQ NL_YES )
    {
        error = N_SrfIsValid( sur, rname );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfPruneRat( sur );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfReadFromFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine writes a surface to a file. The file pointer 
     is passed in.  The file is assumed to  be  opened  and  correctly 
     positioned to write the surface.  The  data  is assumed to be ar-
     ranged as follows:

           n m               --> highest indexes in cp array
           p q               --> degrees    
           rat               --> rationality (0-no,1-yes)
           x00 y00 z00 (w00) -->
           x01 y01 z01 (w01) -->
           .                 --> 
           .                 --> 
           .                 -->
           x0m y0m z0m (w0m) -->
           x10 y10 z10 (w10) --> xyz(w) components of cp's
           x11 y11 z11 (w11) -->
           .                 -->
           .                 -->
           .                 --> 
           xnm ynm znm (wnm) -->
           u0                -->
           u1                -->
           .                 -->
           .                 --> u-knots
           .                 -->
           ur                -->
           v0                -->
           v1                -->
           .                 -->
           .                 --> v-knots
           .                 -->
           vs                -->

     A typical calling example is:

       NL_SURFACE  sur;
       FILE     *fptr;
       ...
       (open file, assign pointer, and position for surface write);
       ...
       N_SrfWriteToFile(&sur,fptr);


   ACCESS:
   
     sur   , input  ,  NURBS surface to be saved
     fptr  , output ,  Pointer to the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfWriteToFile( NL_SURFACE *sur, FILE *fptr )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfWriteToFile");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_CPOINT ** Pw;

    NL_REAL *U, *V, wx, wy, wz, w;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* Get rational flag */

    if( N_IsSrfRat( sur ) )
        rat = NL_YES;
    else
        rat = NL_NO;

    /* Create the output file */

    N_FPRINTF( fptr, _T("%ld %ld\n"), n, m );
    N_FPRINTF( fptr, _T("%hd %hd\n"), p, q );
    N_FPRINTF( fptr, _T("%hd\n"), rat );

    switch( rat )
    {
        case NL_NO: /* Non-rational */
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_CPtToWxWyWz( Pw[i][j], &wx, &wy, &wz, &w );
                    N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), wx, wy, wz );
                }
            }
            break;

        case NL_YES: /* Rational */
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_CPtToWxWyWz( Pw[i][j], &wx, &wy, &wz, &w );
                    N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f %18.16f\n"), wx, wy, wz, w );
                }
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= r; i++ )
        N_FPRINTF( fptr, _T("%18.16f\n"), U[i] );

    for ( j = 0; j <= s; j++ )
        N_FPRINTF( fptr, _T("%18.16f\n"), V[j] );

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfWriteToFile */

/*******************************************************************//**


   DESCRIPTION:

     Given a period surface,  extend a full domain of the surface to give 
     a 100% wrapped surface. This allows trimming curves to extend beyond
     the periodic domain. The resulting surface can be trimmed by the domain
     of the trimming curve

       NL_SURFACE  sur;
       NL_FLAG     UorV       parametric direction of extension(NL_UDIR, NL_VDIR)
       NL_FLAG     MinOrMax   extend domain up (NL_END=Max) or down (NL_START=Min)
       NL_STACKS   S;

     It is assumed that the  stack S contains  all memory  necessary to 
     define sur. The Control net and the knot vector are replaced.

   ACCESS:
   
     sur ,     in/out ,       NURBS surface 
     UorV ,    input  ,       parametric direction of extension NL_UDIR or NL_VDIR
     MinOrMax, input ,        extend domain up (NL_END) or down (NL_START)
     S   ,     input  ,       sur's stack

   EXAMPLE OF USE:  
     NL_POINT    C;
     NL_VECTOR   X, Y;
     NL_REAL     rb, rt, as, ae, h;
     NL_SURFACE  sur;
     NL_PARAMETER ul, ur, vl, vr;

     C.x = 0.0; C.y = 0.0; C.z = 0.0;
     X.x = 1.0; X.y = 0.0; X.z = 0.0;
     Y.x = 0.0; Y.y = 1.0; Y.z = 0.0;
     rb = rt = 1.0;
     as = 0.0; ae = 360.0;
     h = 4.0;

     N_SrfInitArrays(&sur);
     N_CreateCylCone(C,X,Y,rb,rt,as,ae,h,NL_QUADRATIC,NL_UDIR,&sur,&S);

     N_ExtendPeriodicSurface(&sur, NL_UDIR, NL_END, &S);

     ul = 0.6; ur = 1.6;
     vl = 0.0; vr = 1.0;
     N_SrfExtractPatch(&sur, ul, ur, vl, vr, &sur, &S, &S );

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ExtendPeriodicSurface( NL_SURFACE *sur, NL_FLAG UorV, NL_FLAG MinOrMax, NL_STACKS *S )
{
    NL_INDEX i, j, n, m, mm, jj;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    NL_REAL kMin, kMax, *Knt;

    NL_CPOINT ** Pw, ** Qw;

    NL_CNET *net;

    /* Get local notation */

    N_SrfGetNetAndKnotVectors( sur, &net, &p, &q, &knu, &knv );
    N_SrfGetCPts( sur, &n, &m, &Pw );

    if( UorV == NL_UDIR )
    {
        /* Fully wrap control points in U*/
        Qw = N_AllocCPt2dArray( 2 * n, m, S );

        if( Qw EQ NULL )
            return (1);

        for ( j = 0; j <= m; j++ )
        {
            for ( i = 0; i <= n; i++ )
            {
                N_CopyCPt( Pw[i][j], &Qw[i][j] );
            }

            for ( i = 1; i <= n; i++ )
            {
                N_CopyCPt( Pw[i][j], &Qw[i + n][j] );
            }
        }

        N_CNetFromCPts( net, Qw, 2 * n, m );

        /* Extend knots */
        mm = knu->m;
        Knt = N_AllocReal1dArray( 2 * mm - p - 1, S );
        kMin = knu->U[0];
        kMax = knu->U[mm];

        jj = 0;

        if( MinOrMax == NL_END )
        {
            /* wrap upwards */
            for ( i = 0; i < mm; i++ )
            {
                Knt[jj++] = knu->U[i];
            }

            for ( i = p + 1; i <= mm; i++ )
            {
                Knt[jj++] = knu->U[i] + kMax - kMin;
            }
        }
        else /* NL_START wrap downwards */
        {
            for ( i = 0; i < mm; i++ )
            {
                Knt[jj++] = knu->U[i] - kMax + kMin;
            }

            for ( i = p + 1; i <= mm; i++ )
            {
                Knt[jj++] = knu->U[i];
            }
        }

        /* Kill old memory */

        N_FreeCPt2dArray( Pw, S );
        N_FreeReal1dArray( knu->U, S );

        knu->U = Knt;
        knu->m = 2 * mm - p - 1;
    } /* end U wrap */

    if( UorV == NL_VDIR )
    {
        /* wrap in V */
        Qw = N_AllocCPt2dArray( n, 2 * m, S );

        if( Qw EQ NULL )
            return (1);

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
            {
                N_CopyCPt( Pw[i][j], &Qw[i][j] );
            }

            for ( j = 1; j <= m; j++ )
            {
                N_CopyCPt( Pw[i][j], &Qw[i][j + m] );
            }
        }

        N_CNetFromCPts( net, Qw, n, 2 * m );

        /* Extend knots */
        mm = knv->m;
        Knt = N_AllocReal1dArray( 2 * mm - q - 1, S );
        kMin = knv->U[0];
        kMax = knv->U[mm];

        jj = 0;

        if( MinOrMax == NL_END )
        {
            /* wrap upwards */
            for ( i = 0; i < mm; i++ )
            {
                Knt[jj++] = knv->U[i];
            }

            for ( i = q + 1; i <= mm; i++ )
            {
                Knt[jj++] = knv->U[i] + kMax - kMin;
            }
        }
        else /* NL_START wrap downwards */
        {
            for ( i = 0; i < mm; i++ )
            {
                Knt[jj++] = knv->U[i] - kMax + kMin;
            }

            for ( i = q + 1; i <= mm; i++ )
            {
                Knt[jj++] = knv->U[i];
            }
        }

        /* Kill old memory */

        N_FreeCPt2dArray( Pw, S );
        N_FreeReal1dArray( knv->U, S );

        knv->U = Knt;
        knv->m = 2 * mm - q - 1;
    } /* end V wrap */

    N_SrfFromCNetAndKnotVectors( sur, net, p, q, knu, knv );

    /* Exit */
    return (0);
} /* end N_ExtendPeriodicSurface */

/**********************************************************************/
/* N_surfJet:  Implementation of NL_SURFJET struct                    */
/**********************************************************************/

/*
 *  This struct pretends to be a class that represents an evaluated
 *  point on a surface.  See description on nurbsdef.h.
 */

/* a local helper function */
static NL_BOOLEAN surfJetEval( NL_SURFJET *sJet, int numDerivs )
{
    NL_PRIVATE NL_STRING rname = _T("N_surfJet");
    NL_FLAG ret;
    NL_FLAG leftRight = NL_LEFT;
    NL_FLAG triang = NL_TRUE;

    if( numDerivs > sJet->MAX_DERIVS )
        numDerivs = (int)sJet->MAX_DERIVS;

    /*  See whether we need to do anything  */
    if( sJet->numEval >= numDerivs )
        return NL_TRUE;

    if( N_surfJetUnset( sJet ) )
    {
        N_ErrSet( NL_INP_ERR, rname );
        return NL_FALSE;
    }

    ret = N_SrfDerivs( sJet->mySurf, sJet->u, sJet->v, leftRight, leftRight, triang, numDerivs, numDerivs, sJet->derivs );

    if( ret == NL_NO )
        sJet->numEval = numDerivs;

    return (ret == NL_NO);
}

/* Utilities */

NL_VOID N_surfJetInit( NL_SURFJET *sJet, NL_SURFACE *srf )
{
    sJet->MAX_DERIVS = SURFJET__MAX_DERIVS;

    sJet->mySurf = srf;

    sJet->derivs[0] = sJet->der0;
    sJet->derivs[1] = sJet->der1;
    sJet->derivs[2] = sJet->der2;

    N_surfJetReset( sJet );
}

NL_BOOLEAN N_surfJetUnset( NL_SURFJET *sJet )
{
    if( sJet->numEval > -2 )
        return NL_FALSE;

    return NL_TRUE;
}

NL_VOID N_surfJetReset( NL_SURFJET *sJet )
{
    sJet->numEval = -2;
}

NL_VOID N_surfJetSetParam( NL_SURFJET *sJet, double u, double v )
{
    sJet->u = u;
    sJet->v = v;
    sJet->numEval = -1;
}

/* Evaluation:  basic data */

NL_POINT *N_surfJetPos( NL_SURFJET *sJet )
{
    surfJetEval( sJet, 0 );
    return &( sJet->derivs[0][0] );
}

NL_VECTOR *N_surfJetDer_u( NL_SURFJET *sJet )
{
    surfJetEval( sJet, 1 );
    return &( sJet->derivs[1][0] );
}

NL_VECTOR *N_surfJetDer_v( NL_SURFJET *sJet )
{
    surfJetEval( sJet, 1 );
    return &( sJet->derivs[0][1] );
}

NL_VECTOR *N_surfJetDer_uu( NL_SURFJET *sJet )
{
    surfJetEval( sJet, 2 );
    return &( sJet->derivs[2][0] );
}

NL_VECTOR *N_surfJetDer_uv( NL_SURFJET *sJet )
{
    surfJetEval( sJet, 2 );
    return &( sJet->derivs[1][1] );
}

NL_VECTOR *N_surfJetDer_vv( NL_SURFJET *sJet )
{
    surfJetEval( sJet, 2 );
    return &( sJet->derivs[0][2] );
}

/*  versions that make copies: */
NL_VOID N_surfJetPosCopy( NL_SURFJET *sJet, NL_POINT *pt )
{
    N_VectorCopy( *( N_surfJetPos( sJet ) ), pt );
}

NL_VOID N_surfJetDer_uCopy( NL_SURFJET *sJet, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_surfJetDer_u( sJet ) ), deriv );
}

NL_VOID N_surfJetDer_vCopy( NL_SURFJET *sJet, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_surfJetDer_v( sJet ) ), deriv );
}

NL_VOID N_surfJetDer_uuCopy( NL_SURFJET *sJet, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_surfJetDer_uu( sJet ) ), deriv );
}

NL_VOID N_surfJetDer_uvCopy( NL_SURFJET *sJet, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_surfJetDer_uv( sJet ) ), deriv );
}

NL_VOID N_surfJetDer_vvCopy( NL_SURFJET *sJet, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_surfJetDer_vv( sJet ) ), deriv );
}

/*  Conveniences: derived data */

NL_BOOLEAN N_surfJetNormal( NL_SURFJET *sJet, NL_VECTOR *norm )
{
    NL_FLAG err;
    N_VectorCrossRef( N_surfJetDer_u( sJet ), N_surfJetDer_v( sJet ), norm );
    err = N_VectorNormalizeRef( norm );

    if( err == NL_YES )
    {
        N_VectorCopy( NL_ZERO, norm );
        return NL_FALSE;
    }
    return NL_TRUE;
}

NL_BOOLEAN N_surfJetPrinCurvature( NL_SURFJET *sJet, NL_REAL *k1, NL_VECTOR *T1, NL_REAL *k2, NL_VECTOR *T2 )
{
    /* NL_BOOLEAN ret = NL_TRUE; */
    NL_FLAG leftRight = NL_LEFT;
    NL_FLAG getPrinCrv = NL_YES;
    NL_FLAG err, retVal;

    NL_POINT dummyPt;
    NL_VECTOR dummyVec1, dummyVec2, dummyVec3; /* These can't be the same */
    NL_VECTOR dummyVec4, dummyVec5;
    NL_REAL dummyReal;

    /* just to check overflow: */
    /* double EPS = 1e-12; */

    /* initialize our derivs */
    surfJetEval( sJet, 2 );

    err = N_SrfEvalPtCurvature( sJet->mySurf, sJet->u, sJet->v, leftRight, leftRight, getPrinCrv, &dummyReal, &dummyReal, &dummyPt, &dummyVec1, &dummyVec2, &dummyVec3, k1, k2, &dummyVec4, &dummyVec5, T1, T2, &retVal );

    if( err == NL_YES )
    {
        *k1 = *k2 = 0;
        N_VectorCopy( NL_ZERO, T1 );
        N_VectorCopy( NL_ZERO, T2 );
        return NL_FALSE;
    }

    return NL_TRUE;
}

NL_BOOLEAN N_surfJetRelax( NL_SURFJET *sJet, NL_POINT *pt )
{

    NL_FLAG err = NL_YES;

    NL_PARAMETER new_U = 0.0, new_V = 0.0;
    NL_POINT tmpPt;

    if( !N_surfJetUnset( sJet ) )
    {
        /*  Param is set: it's the guess point.  Use the local solver:  */

        err = N_SrfGetClosestPt( sJet->mySurf, *pt, sJet->u, sJet->v, NL_MTOL, NL_MTOL, &new_U, &new_V, &tmpPt );
    }

    /*  If we didn't have a guess, or if it failed with a guess,
    *  try it without a guess: use the global solver, N_CrvClosestPtMultiple.
    */

    if( err == NL_YES )
    {
        /*  No guess param  */

        /* TODO: N_SrfProjectPts sets up all of the temp surface data, and then
        * uses it on what is presumably a list of points, and then
        * deletes it.  It would help a lot to set it up as in the curve
        * routine N_CrvClosestPtMultiple: the data (NL_GCPTEMP) is controlled by the
        * caller, and can be used for multiple calls.
        */

        NL_INDEX numOut;
        NL_STACKS localStacks;
        NL_POINT *projPt;
        NL_REAL *uVals;
        NL_REAL *vVals;
        N_InitNurbs( &localStacks );

        err = N_SrfProjectPts( sJet->mySurf, pt, 0, NL_NO, NL_YES, 0.001, &projPt, &uVals, &vVals, &numOut, &localStacks );

        if( numOut != 0 )
            err = NL_YES;

        if( err == NL_NO )
        {
            new_U = uVals[0];
            new_V = vVals[0];
        }
        N_EndNurbs( &localStacks );
    }

    if( err == NL_NO )
        N_surfJetSetParam( sJet, new_U, new_V );

    return (err == NL_NO);
}
