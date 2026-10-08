// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************************/
/* FrameBasic.c : Basic Function Declarations for NL_CPOLYGON, NL_CNET, NL_CMESH objects */
/*****************************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>



/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a polygon
     structure.  Proper  error check is performed in case memory 
     allocation fails. A typical calling example is:
  
       NL_CPOLYGON  *pos;
       NL_STACKS    S;
       ...
       pos = N_AllocCPolygon(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     pos  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CPOLYGON *N_AllocCPolygon( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPolygon");

    NL_CPOLYGON *pos;

    NL_POLNODE *pod;

    /* Allocate memory for the structure */

    pos = (NL_CPOLYGON *)N_Malloc( sizeof( NL_CPOLYGON ) );

    if( pos EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    pod = (NL_POLNODE *)N_Malloc( sizeof( NL_POLNODE ) );

    if( pod EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( pos );
        pos = NULL;
        return NULL;
    }

    pod->ptr = pos;
    pod->next = S->pol;
    S->pol = pod;

    /* Exit */

    return pos;
} /* end N_AllocCPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a polygon object. Proper  
     error  check is  performed in  case memory  allocation fails. A 
     typical calling example is:

       NL_CPOLYGON  *pol;
       NL_INDEX     n;
       NL_STACKS    S;
       ...
       (get n);
       ...
       pol = N_AllocCPolygonAndArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in control polygon array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     pol  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CPOLYGON *N_AllocCPolygonAndArray( NL_INDEX n, NL_STACKS *S )
{
    NL_CPOINT *Pw;
    NL_CPOLYGON *pol;

    /* Allocate memory */
    pol = N_AllocCPolygon( S );

    if( pol EQ NULL )
        return NULL;

    Pw = N_AllocCPt1dArray( n, S );

    if( Pw EQ NULL )
        return NULL;

    /* Build the object */
    N_CPolygonFromCPts( pol, Pw, n );

    return pol;
} /* end N_AllocCPolygonAndArray */


/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores members of a 
     control polygon structure. Given  a polygon  pointer, the  routine
     searches for the  pointer on the  memory  stack.  It it is  found,
     memory is deallocated. If not, the routine does nothing. A typical
     calling example is:

       NL_CPOLYGON  *pol;
       NL_STACKS    S;
       ...
       N_FreeCPolygon(pol,&S);


   ACCESS:
   
     pol , input  ,  Control polygon pointer 
     S   , input  ,  pol's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCPolygon( NL_CPOLYGON *pol, NL_STACKS *S )
{
    NL_POLNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->pol NEQ NULL )
    {
        prev = S->pol;
        curr = S->pol;

        while( curr NEQ NULL AND curr->ptr NEQ pol )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->pol = NULL;
            }
            else /* More than one node */
            {
                S->pol = S->pol->next;
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
} /* end N_FreeCPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine breaks a polygon object down to its 
     components. A typical calling example is:

       NL_CPOLYGON  pol;
       NL_INDEX     n;
       NL_CPOINT    *Pw;
       ...
       N_CPolygonGetCPts(&pol,&n,&Pw);


   ACCESS:
   
     pol , input  ,  Control polygon
     n   , output ,  Highest index in Pw
     Pw  , output ,  Control points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPolygonGetCPts( NL_CPOLYGON *pol, NL_INDEX *n, NL_CPOINT ** Pw )
{
    *n = pol->n;
    *Pw = pol->Pw;
} /* end N_CPolygonGetCPts */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper pointer assignments to define
     a control polygon object from a  set of  control points. Memory
     for the polygon structure is  allocated in the calling routine;
     only the pointer is passed down. A typical calling  example is:

       NL_CPOLYGON  pol;
       NL_CPOINT    *Pw;
       NL_INDEX     n;
       ...
       (allocate memory for Pw);
       ...
       N_CPolygonFromCPts(&pol,Pw,n);


   ACCESS:
   
     pol , in/out ,  Polygon
     Pw  , input  ,  Control points
     n   , input  ,  Highest index in Pw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPolygonFromCPts( NL_CPOLYGON *pol, NL_CPOINT *Pw, NL_INDEX n )
{
    pol->n = n;
    pol->Pw = Pw;
} /* end N_CPolygonFromCPts */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a polygon object given the wx, wy, wz and 
     w components of the control points. It allocates memory to store 
     the control points  and  makes  proper  pointer assignments. The  
     curve  is  restricted to  2-D or to  non-rational by setting the 
     approriate  coordinate  values  to NL_NOZ or NL_NOW. Memory  for the 
     polygon object and for the wx, wy, wz and w  arrays is allocated 
     in the calling  routine. A typical calling example is:

       NL_CPOLYGON  pol;
       NL_REAL      *wx, *wy, *wz, *w;
       NL_INDEX     n;
       NL_STACKS    S;
       ...
       (allocate memory for wx, wy, wz and w);
       ...
       N_CPolygonFromCPtCoords(&pol,wx,wy,wz,w,n,&S);


   ACCESS:
   
     pol        , in/out ,  Polygon
     wx,wy,wz,w , input  ,  wx, wy, wz and w components
     n          , input  ,  Highest index in <wx,wy,wz,w>
     S          , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CPolygonFromCPtCoords( NL_CPOLYGON *pol, NL_REAL *wx, NL_REAL *wy, NL_REAL *wz, NL_REAL *w, NL_INDEX n, NL_STACKS *S )
{
    NL_INDEX i;

    NL_CPOINT *Pw;

    /* Allocate memory for point array */

    Pw = N_AllocCPt1dArray( n, S );

    if( Pw EQ NULL )
        return (1);

    /* Fill in point array */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtFromWxWyWz( wx[i], wy[i], wz[i], w[i], &Pw[i] );
    }

    /* Make pointer assignments */

    N_CPolygonFromCPts( pol, Pw, n );

    /* Exit */

    return (0);
} /* end N_CPolygonFromCPtCoords */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a point
     polygon structure.  Proper error check is performed in case
     memory allocation fails. A typical calling example is:
  
       NL_EPOLYGON  *pps;
       NL_STACKS    S;
       ...
       pps = N_AllocPtPolygonStruct(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     pps  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_EPOLYGON *N_AllocPtPolygonStruct( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPtPolygonStruct");
    NL_EPOLYGON *pps;

    NL_PPLNODE *ppd;

    /* Allocate memory for the structure */

    pps = (NL_EPOLYGON *)N_Malloc( sizeof( NL_EPOLYGON ) );

    if( pps EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    ppd = (NL_PPLNODE *)N_Malloc( sizeof( NL_PPLNODE ) );

    if( ppd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( pps );
        pps = NULL;
        return NULL;
    }

    ppd->ptr = pps;
    ppd->next = S->ppl;
    S->ppl = ppd;

    /* Exit */

    return pps;
} /* end N_AllocPtPolygonStruct */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a point polygon object.
     Proper error check is performed in case memory allocation fails.
     A typical calling example is:

       NL_EPOLYGON  *ppl;
       NL_INDEX     n;
       NL_STACKS    S;
       ...
       (get n);
       ...
       ppl = N_AllocPtPolygon(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in point polygon array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     ppl  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_EPOLYGON *N_AllocPtPolygon( NL_INDEX n, NL_STACKS *S )
{
    NL_POINT *PtPtr;

    NL_EPOLYGON *ppl;

    /* Allocate memory */

    ppl = N_AllocPtPolygonStruct( S );

    if( ppl EQ NULL )
        return NULL;

    PtPtr = N_AllocPt1dArray( n, S );

    if( PtPtr EQ NULL )
        return NULL;

    /* Build the object */

    N_EPolygonFromPts( ppl, n, PtPtr );

    /* Exit */

    return ppl;
} /* end N_AllocPtPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 1-D array of point polygon
     pointers, as well as the point polygons themselves, defined by the
     same index m.  Proper error check is performed in case memory
     allocation fails.  A typical calling example is:

       NL_EPOLYGON  **pp2;
       NL_INDEX     m, k;
       ...
       (get m and k);
       ...
       pp2 = N_Alloc1dPtPolygon(m,k,&S);


   ACCESS:
   
     m   , input  ,  Highest index in point polygon arrays
     k   , input  ,  Highest  index  of point polygon array  pp2[0],...,
                     pp2[k]; pp2[i], 0<=i<=k, is a pointer to  the i-th 
                     point polygon object.
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     pp2  : Pointer to array of point polygons if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_EPOLYGON ** N_Alloc1dPtPolygon( NL_INDEX m, NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc1dPtPolygon");
    NL_INDEX i;

    NL_EPOLYGON ** pp2;

    NL_PP2NODE *nodePtr;

    /* Allocate memory for point polygon pointer array */

    pp2 = (NL_EPOLYGON ** )N_Malloc( (k + 1) * sizeof( NL_EPOLYGON * ) );

    if( pp2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Allocate memory for each point polygon in the array */

    for ( i = 0; i <= k; i++ )
    {
        pp2[i] = N_AllocPtPolygon( m, S );

        if( pp2[i]EQ NULL )
        {
            N_Free( pp2 );
            pp2 = NULL;
            return NULL;
        }
    }

    /* Put pointer on memory stack */

    nodePtr = (NL_PP2NODE *)N_Malloc( sizeof( NL_PP2NODE ) );

    if( nodePtr EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( pp2 );
        pp2 = NULL;
        return NULL;
    }

    nodePtr->ptr = pp2;
    nodePtr->next = S->pp2;
    S->pp2 = nodePtr;

    /* Exit */

    return pp2;
} /* end N_Alloc1dPtPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates memory  to store a 1-D array of point polygon
     pointers. Proper error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_EPOLYGON  **ppa;
       NL_INDEX     k;
       ...
       (get k);
       ...
       ppa = N_Alloc1dPtPolygonPtrs(k,&S);


   ACCESS:
   
     k   , input  ,  Highest index of point polygon pointer array
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     ppa  : Pointer to array of point polygon pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_EPOLYGON ** N_Alloc1dPtPolygonPtrs( NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc1dPtPolygonPtrs");
    NL_EPOLYGON ** ppa;

    NL_PP2NODE *nodePtr;

    /* Allocate memory */

    ppa = (NL_EPOLYGON ** )N_Malloc( (k + 1) * sizeof( NL_EPOLYGON * ) );

    if( ppa EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    nodePtr = (NL_PP2NODE *)N_Malloc( sizeof( NL_PP2NODE ) );

    if( nodePtr EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( ppa );
        ppa = NULL;
        return NULL;
    }

    nodePtr->ptr = ppa;
    nodePtr->next = S->pp2;
    S->pp2 = nodePtr;

    /* Exit */

    return ppa;
} /* end N_Alloc1dPtPolygonPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This geometry  routine intersects a line  segment with a convex and
     closed polygon. Its main use is to compute intersections of a large
     number of  line  segments with  the same  polygon. To  speed up the 
     computation, the  principle of  coherence is applied, i.e.  a guess 
     polygon leg  index is  passed in, and  the  index of  the  one that 
     intersects the line is returned. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_POINT     S, E, Q;
       NL_INDEX     i, j;
       NL_FLAG      its;
       ...
       (get ppl, S, E and i);
       ...
       N_IsectLinePolygon(&ppl,S,E,i,&Q,&j,&its);


   ACCESS:
   
     ppl  , input  ,  Closed and convex 2-D polygon
     S,E  , input  ,  Line segment <S,E>
     i    , input  ,  Guess  index;  <S,E>  is  first  intersected  with 
                      <P[i],P[i+1]>
     Q    , output ,  Intersection point
     j    , output ,  <S,E> intersects <P[j],P[j+1]>
     its  , output ,  Flag = 
                        NL_TRUE : Line segment and polgon intersect
                        NL_FALSE: Line segment and polygon do not intersect


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsectLinePolygon( NL_EPOLYGON *ppl, NL_POINT S, NL_POINT E, NL_INDEX i, NL_POINT *Q, NL_INDEX *j, NL_FLAG *its )
{
    NL_PRIVATE NL_STRING rname = _T("N_IsectLinePolygon");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, k, n;

    NL_POINT *P;

    /* Get locals, check index and initialize */

    N_EPolygonGetPts( ppl, &n, &P );

    if( i LT 0 OR i GT n )
        NL_ERROR( NL_INP_ERR );

    *its = NL_FALSE;
    ii = i;

    /* First: try to intersect guess leg */

    if( i EQ n )
        ii = 0;

    error = N_IsectLineSegs( P[ii], P[ii + 1], S, E, Q, its );

    if( error EQ NL_YES )
        NL_OUT;

    if( *its EQ NL_TRUE )
    {
        *j = ii;
        NL_OUT;
    }

    /* Second: try to intersect next leg */

    if( i EQ n - 1 )
        ii = -1;

    error = N_IsectLineSegs( P[ii + 1], P[ii + 2], S, E, Q, its );

    if( error EQ NL_YES )
        NL_OUT;

    if( *its EQ NL_TRUE )
    {
        *j = ii + 1;
        NL_OUT;
    }

    /* Third: try to intersect previous leg */

    if( i EQ 0 )
        ii = n;
    else
        ii = i;

    error = N_IsectLineSegs( P[ii - 1], P[ii], S, E, Q, its );

    if( error EQ NL_YES )
        NL_OUT;

    if( *its EQ NL_TRUE )
    {
        *j = ii - 1;
        NL_OUT;
    }

    /* Fourth: still no intersection -> go around the polygon */

    for ( k = 0; k < n; k++ )
    {
        error = N_IsectLineSegs( P[k], P[k + 1], S, E, Q, its );

        if( error EQ NL_YES )
            NL_OUT;

        if( *its EQ NL_TRUE )
        {
            *j = k;
            break;
        }
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_IsectLinePolygon */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the bounding box of a polygon. A
     typical calling example is:

       NL_EPOLYGON   ppl;
       NL_MINMAXBOX  box;
       ...
       (get ppl);
       ...
       N_PolygonGetBBox(&ppl,&box);


   ACCESS:
   
     ppl , input  ,  Polygon
     box , output ,  Bounding box


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PolygonGetBBox( NL_EPOLYGON *ppl, NL_MINMAXBOX *box )
{
    NL_INDEX i, n;
    NL_REAL x, y, z, xl, xr, yb, yt, zn, zf;
    NL_POINT *P;

    /* Get local notation */
    N_EPolygonGetPts( ppl, &n, &P );

    /* Compute min-max box */
    N_PtToXYZ( P[0], &x, &y, &z );

    xl = xr = x;
    yb = yt = y;
    zn = zf = z;

    for ( i = 1; i <= n; i++ )
    {
        N_PtToXYZ( P[i], &x, &y, &z );

        if( x LT xl )
            xl = x;

        if( x GT xr )
            xr = x;

        if( y LT yb )
            yb = y;

        if( y GT yt )
            yt = y;

        if( z LT zn )
            zn = z;

        if( z GT zf )
            zf = z;
    }

    N_BBoxDefine( box, xl, xr, yb, yt, zn, zf );
} /* end N_PolygonGetBBox */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine finds the index of the polygon leg that is
     the closest  to a given point. It also returns the projection of
     this point to the closest leg. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_POINT     P, Q;
       NL_INDEX     j;
       NL_REAL      t;
       NL_FLAG      flg;
       ...
       (get polygon and P);
       ...
       N_PolygonGetClosestLegIndex(&ppl,P,&Q,&j,&t,&flg);


   ACCESS:
   
     ppl , input  ,  Polygon
     P   , input  ,  Given point
     Q   , output ,  Projection of P to the closest leg
     j   , output ,  Index  of  closest  leg, i.e.  P  is  closest to 
                     <P[j],P[j+1]>
     t   , output ,  Parameter corresponding to Q
     flg , output ,  Flag = 
                       NL_TRUE : Projection is successful
                       NL_FALSE: Projection  is  unsuccessful,  i.e.  P
                              projects outside of each leg.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PolygonGetClosestLegIndex( NL_EPOLYGON *ppl, NL_POINT P, NL_POINT *Q, NL_INDEX *j, NL_REAL *t, NL_FLAG *flg )
{
    NL_FLAG prj, error;
    NL_INDEX i, k, n;
    NL_POINT *R, S;
    NL_LINESEG lsg;
    NL_REAL u, d, dmin;

    /* Get local notation */
    N_EPolygonGetPts( ppl, &n, &R );

    /* Compute leg index */
    dmin = NL_BIGD;
    *flg = NL_FALSE;

    for ( i = 0; i < n; i++ )
    {
        N_CreateLinePtPt( &lsg, R[i], R[i + 1], NL_BOUNDED );

        error = N_ProjectPtLine( lsg, P, &S, &u, &prj );

        if( error EQ NL_YES )
            return (1);

        if( prj EQ NL_FALSE )
            continue;

        N_DistPtPt( P, S, &d );

        if( d LT dmin )
        {
            dmin = d;
            *j = i;
            *t = u;
        }
    }

    if( dmin NEQ NL_BIGD )
    {
        u = *t;
        d = 1.0 - u;
        k = *j;
        N_Combine2Pts( d, R[k], u, R[k + 1], Q );
        *flg = NL_TRUE;
    }

    /* Exit */

    return (0);
} /* end N_PolygonGetClosestLegIndex */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a polygon object given the vertices of
     the polygon. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_INDEX     n;
       NL_POINT     *P;
       ...
       (get n and array P);
       ...
       N_EPolygonFromPts(&ppl,n,P);


   ACCESS:
   
     ppl , output ,  Polygon object
     n   , input  ,  Highest index in P
     P   , input  ,  Vertices of polygon



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_EPolygonFromPts( NL_EPOLYGON *ppl, NL_INDEX n, NL_POINT *P )
{
    ppl->n = n;
    ppl->P = P;
} /* end N_EPolygonFromPts */



/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a polygon object down to its 
     components. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_INDEX     n;
       NL_POINT     *P;
       ...
       (get ppl);
       ...
       N_EPolygonGetPts(&ppl,&n,&P);


   ACCESS:
   
     ppl  , input  ,  Polygon
     n    , output ,  Highest index in P
     P    , output ,  Vertex array of polygon


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_EPolygonGetPts( NL_EPOLYGON *ppl, NL_INDEX *n, NL_POINT ** P )
{
    *n = ppl->n;
    *P = ppl->P;
} /* end N_EPolygonGetPts */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if  points on a grid are inside a 2-D 
     closed polygon. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_POINT     **Q;
       NL_INDEX     r, s;
       NL_BOOLEAN   **IN;
       ...
       (get ppl and Q; allocate memory for IN);
       ...
       G_ispqpl(&ppl,Q,r,s,IN);

     MEMORY FOR IN MUST BE ALLOCATED IN THE  CALLING ROUTINE TO HOLD UP
     TO IN[r][s]!


   ACCESS:
   
     ppl , input  ,  Closed 2-D polygon
     Q   , input  ,  Point grid in 2-D; Q[i][j] must have 
                       the same x coordinate for i=0,...r, and 
                       the same y coordinate for j=0,...s
     r,s , input  ,  Highest indexes in Q
     IN  , output ,  Boolean array: IN[i][j] =
                       NL_TRUE  if Q[i][j] is in
                       NL_FALSE if Q[i][j] is out 
                    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PtsAreInPolygon( NL_EPOLYGON *ppl, NL_POINT ** Q, NL_INDEX r, NL_INDEX s, NL_BOOLEAN ** IN )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, nip;

    NL_REAL *x, y, xl, xr, yb, yt, dum;

    NL_POINT *P;

    NL_MINMAXBOX box;

    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation */
    N_EPolygonGetPts( ppl, &n, &P );

    /* Check box overlap */
    N_PolygonGetBBox( ppl, &box );
    N_GetBBoxData( &box, &xl, &xr, &yb, &yt, &dum, &dum );

    if( Q[0][0].y GT yt OR Q[r][s].y LT yb OR Q[0][0].x GT xr OR Q[r][s].x LT xl )
    {
        for ( i = 0; i <= r; i++ )
        {
            for ( j = 0; j <= s; j++ )
                IN[i][j] = NL_FALSE;
        }

        NL_OUT;
    }

    /* For each row shoot one ray and count number of intersections */

    x = N_AllocReal1dArray( n, &SL );

    if( x EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= s; j++ )
    {
        /* See if ray intersects bounding box */

        y = Q[0][j].y;

        if( y GE yb AND y LE yt )
        {
            /* Get all intersections */

            l = -1;

            for ( k = 1; k <= n; k++ )
            {
                if( ((P[k].y GT y)AND( P[k - 1].y LT( y + NL_MTOL ) ))OR( (P[k - 1].y GT y)AND( P[k].y LT( y + NL_MTOL ) ) ) )
                {
                    x[++l] = P[k].x + ((y - P[k].y) * (P[k].x - P[k - 1].x)) / (P[k].y - P[k - 1].y);
                }
            }

            /* Check each point on the same row */

            for ( i = 0; i <= r; i++ )
            {
                nip = 0;

                for ( k = 0; k <= l; k++ )
                    if( x[k]GT Q[i][j].x )
                        nip++;

                if( nip % 2 )
                    IN[i][j] = NL_TRUE;
                else
                    IN[i][j] = NL_FALSE;
            }
        }
        else
        {
            for ( i = 0; i <= r; i++ )
                IN[i][j] = NL_FALSE;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_PtsAreInPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if points on a grid are inside a 2-D 
     polygon's  gravity  field.  The  gravity  field  is  generated by 
     offsetting the polygon with a  +-  distance, and  rounding at the 
     corners. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_POINT     **Q;
       NL_INDEX     r, s;
       NL_REAL      **D, off;
       ...
       (get ppl, Q and off; allocate memory for D);
       ...
       N_PtsAreInPolygonGravityField(&ppl,Q,r,s,off,D);

     D[i][j] records the closest  distance of points to the polygon if
     they lie in the  gravity field. D[i][j] < 0.0 means  that Q[i][j] 
     does not lie  in the  gravity  field. MEMORY TO STORE  D  MUST BE 
     ALLOCATED IN THE CALLING ROUTINE TO HOLD UP TO D[r][s]!
       


   ACCESS:
   
     ppl , input  ,  Closed 2-D polygon
     Q   , input  ,  Point grid in 2-D; Q[i][j] must have 
                       the same x coordinate for i=0,...r, and 
                       the same y coordinate for j=0,...s
     r,s , input  ,  Highest indexes in Q
     off , input  ,  Offset distance  defining a gravity  field around 
                     each polygon leg
     D   , output ,  Distance array: D[i][j] is
                       >= 0.0 if Q[i][j] is in
                       <  0.0 if Q[i][j] is out 
                    

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PtsAreInPolygonGravityField( NL_EPOLYGON *ppl, NL_POINT ** Q, NL_INDEX r, NL_INDEX s, NL_REAL off, NL_REAL ** D )
{
    NL_FLAG error;

    NL_INDEX i, j, k, n, li, ri, bi, ti;

    NL_REAL d, ddx, ddy, XL, YB, lf, rf, bf, tf, xl, xr, yb, yt, dum;

    NL_POINT *P;

    NL_MINMAXBOX box;

    /* Get local notation */

    N_EPolygonGetPts( ppl, &n, &P );

    /* Initialize */

    for ( i = 0; i <= r; i++ )
    {
        for ( j = 0; j <= s; j++ )
            D[i][j] = -1.0;
    }

    ddx = fabs( Q[r][0].x - Q[0][0].x );
    ddy = fabs( Q[0][s].y - Q[0][0].y );

    if( N_FloatOpIsBad( 1.0, ddx, NL_DIVISION ) )
        return (1);

    if( N_FloatOpIsBad( 1.0, ddy, NL_DIVISION ) )
        return (1);

    ddx = 1.0 / ddx;
    ddy = 1.0 / ddy;
    XL = Q[0][0].x;
    YB = Q[0][0].y;

    /* For each polygon leg do: */

    for ( k = 1; k <= n; k++ )
    {
        /* Get extended bounding box */

        N_LineCalcBBox( P[k - 1], P[k], off, &box );
        N_GetBBoxData( &box, &xl, &xr, &yb, &yt, &dum, &dum );

        /* Get indexes of points inside the box */

        lf = fabs( (xl - XL) * ddx );
        rf = fabs( (xr - XL) * ddx );
        bf = fabs( (yb - YB) * ddy );
        tf = fabs( (yt - YB) * ddy );

        li = (NL_INDEX)(NL_MAX( 0, r * lf ));
        ri = (NL_INDEX)(NL_MIN( r, r * rf + 1 ));
        bi = (NL_INDEX)(NL_MAX( 0, s * bf ));
        ti = (NL_INDEX)(NL_MIN( s, s * tf + 1 ));

        while( li LT r AND Q[li][0].x LT xl )
            li++;

        while( li GT 0 AND Q[li - 1][0].x GE xl )
            li--;

        while( ri GT 0 AND Q[ri][0].x GT xr )
            ri--;

        while( ri LT r AND Q[ri + 1][0].x LE xr )
            ri++;

        while( bi LT s AND Q[0][bi].y LT yb )
            bi++;

        while( bi GT 0 AND Q[0][bi - 1].y GE yb )
            bi--;

        while( ti GT 0 AND Q[0][ti].y GT yt )
            ti--;

        while( ti LT s AND Q[0][ti + 1].y LE yt )
            ti++;

        /* For each point inside the box check if it is in gravity field */

        for ( i = li; i <= ri; i++ )
        {
            for ( j = bi; j <= ti; j++ )
            {
                error = N_DistPtLineSeg( Q[i][j], P[k - 1], P[k], &d );

                if( error EQ NL_YES )
                    return (1);

                if( d LE off )
                {
                    if( D[i][j]GE 0.0 )
                    {
                        if( d LT D[i][j] )
                            D[i][j] = d;
                    }
                    else
                    {
                        D[i][j] = d;
                    }
                }
            }
        }
    }

    /* Exit */

    return (0);
} /* end N_PtsAreInPolygonGravityField */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if a polygon is closed or not. A 
     typical calling example is:

       NL_EPOLYGON  ppl;
       ...
       (define ppl);
       ...
       if( N_EPolygonIsClosed(&ppl) )  --> handle closed polygon;
     

   ACCESS:
   
     ppl , input ,  Polygon


   RETURN CODES:

     NL_TRUE : Polygon is closed
     NL_FALSE: Polygon is NOT closed

   ***********************************************************************/

NL_BOOLEAN N_EPolygonIsClosed( NL_EPOLYGON *ppl )
{
    NL_INDEX n;

    NL_REAL d;

    NL_POINT *P;

    /* Get local notation */

    N_EPolygonGetPts( ppl, &n, &P );

    /* Check closeness */

    N_DistPtPt( P[0], P[n], &d );

    if( d LT NL_MTOL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_EPolygonIsClosed */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if a point is inside a 2-D closed 
     polygon. A typical calling example is:

       NL_EPOLYGON  ppl;
       NL_POINT     Q;
       ...
       (get ppl);
       ...
       if( N_PtIsContainedByEPolygon(&ppl,Q) )  --> point is in; 

    Note: for a closed polygon P[0] = P[n]; 
    Note: orientation of polygon points (cw,or ccw) determines 'inside'

   ACCESS:
   
     ppl , input ,  Polygon in 2-D
     Q   , input ,  Point in 2-D


   RETURN CODES:

     NL_TRUE : Point is on or in
     NL_FALSE: Point is on or out

   ***********************************************************************/

NL_BOOLEAN N_PtIsContainedByEPolygon( NL_EPOLYGON *ppl, NL_POINT Q )
{
    NL_INDEX i, k, m, n, jj;

    NL_REAL x, num, den;

    NL_POINT *P;

    /* Get local notation */

    N_EPolygonGetPts( ppl, &n, &P );

    /* Shoot ray and count number of intersections */

    k = 0; /*  intersection count  */

    for ( i = 1; i <= n; i++ )
    {
        if( P[i].y GT Q.y AND P[i - 1].y GT Q.y )
            continue;

        if( P[i].y LT Q.y AND P[i - 1].y LT Q.y )
            continue;

        if( P[i].x LT Q.x AND P[i - 1].x LT Q.x )
            continue;

        num = (Q.y - P[i].y) * (P[i].x - P[i - 1].x);
        den = P[i].y - P[i - 1].y;

        if( NOT N_FloatOpIsBad( num, den, NL_DIVISION ) )
        {
            x = P[i].x + (num / den);

            if( x GT Q.x AND P[i].y NEQ Q.y )
            {
                k++;

                if( P[i - 1].y EQ Q.y )
                { /* check if it should be counted twice */
                    if( i EQ 1 )
                        jj = n - 1;
                    else
                        jj = i - 2;

                    if( ((Q.y - P[jj].y) * (Q.y - P[i].y))GT 0.0 )
                        k++;
                }
            }
        }
        else
        { /* polygon leg parallel to ray */
            k += 1;
            m = i + 1;

            if( i EQ 1 )
                jj = n - 1;
            else
            {
                jj = i - 2;

                if( i == n )
                    m = 1;
            }

            if( ((Q.y - P[jj].y)*( Q.y - P[m].y ))GT 0.0 )
            {
                if( i < n )
                    k++;
            }

            i += 1;
        }
    }

    if( k % 2 )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_PtIsContainedByEPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the signed area of a 2D closed poly-
     gon.  This also  indicates  orientation of the polygon.  A  typical 
     calling example is:

       NL_EPOLYGON  ppl;
       ...
       (define ppl);
       ...
       if ( N_PolygonGetArea(&ppl) GT 0.0 )  --> polygon is counterclockwise;
     

   ACCESS:
   
     ppl , input ,  Closed 2D polygon


   RETURN CODES:

     signed area of polygon

     > 0 : Polygon is oriented counterclockwise
     < 0 : Polygon is oriented clockwise

   ***********************************************************************/

NL_REAL N_PolygonGetArea( NL_EPOLYGON *ppl )
{
    NL_INDEX ii, nn;

    NL_REAL area, xold, yold, yorig, x, y;

    NL_POINT *P;

    /* Get local notation */

    N_EPolygonGetPts( ppl, &nn, &P );

    /* Compute area */

    area = 0.0;

    xold = P[nn].x;
    yorig = P[nn].y;
    yold = 0.0;

    for ( ii = 0; ii <= nn; ii++ )
    {
        x = P[ii].x;
        y = P[ii].y - yorig;
        area = area + (xold - x) * (yold + y);
        xold = x;
        yold = y;
    }

    area = 0.5 *area;

    return (area);
} /* end N_PolygonGetArea */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine sets the index in polygon definition. A 
     typical calling example is:

       NL_EPOLYGON  ppl;
       NL_INDEX     n;
       ...
       N_PolygonSetIndex(&ppl,n);


   ACCESS:
   
     ppl , in/out ,  Polygon object
     n   , input  ,  Highest index in polygon array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PolygonSetIndex( NL_EPOLYGON *ppl, NL_INDEX n )
{
    ppl->n = n;
} /* end N_PolygonSetIndex */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the signed offset polygon of a given
     open or closed 2D polygon.  Only polygon offset is  computed,  self 
     intersection  check or  other post  processing  steps are not done.
     A typical calling example is:

       NL_EPOLYGON  ppl, qpl;
       NL_REAL      d;
       ...
       (get polygon and d);
       ...
       N_PolygonOffset(&ppl,d,&qpl);

     MEMORY FOR QPL MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     ppl , input  ,  Polygon
     d   , input  ,  Signed offset distance:
                     d>0: offset lies to the left of the polygon
                     d<0: offset lies to to the right of the polygon
     qpl , output ,  Offset polygon


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PolygonOffset( NL_EPOLYGON *ppl, NL_REAL d, NL_EPOLYGON *qpl )
{
    NL_PRIVATE NL_STRING rname = _T("N_PolygonOffset");

    NL_FLAG flg, cld, error;

    NL_INDEX i, n;

    NL_REAL t1, t2;

    NL_POINT *P, *Q, R;

    NL_VECTOR V, W;

    NL_LINESEG lc, lp, ls;

    /* Get local notation and check for closeness */

    N_EPolygonGetPts( ppl, &n, &P );
    N_EPolygonGetPts( qpl, &i, &Q );

    if( i LT n )
        NL_ERROR( NL_INP_ERR );

    if( N_EPolygonIsClosed( ppl ) )
        cld = NL_YES;
    else
        cld = NL_NO;

    /* Offset start point/leg */

    N_VectorDiff( P[1], P[0], &V );
    N_VectorPerpendicular( V, &W );

    error = N_VectorNormalizeRef( &W );

    if( error EQ NL_YES )
        NL_OUT;

    /* initialize line segments with any two points */
    N_CreateLinePtPt( &ls, P[0], V, NL_UNBOUNDED );


    if( cld EQ NL_NO )
    {
        N_VectorPtAlongVector( P[0], d, W, &Q[0] );
        N_CreateLineStartDirVector( &lp, Q[0], V, NL_UNBOUNDED );
    }
    else
    {
        N_VectorPtAlongVector( P[0], d, W, &R );
        N_CreateLineStartDirVector( &lp, R, V, NL_UNBOUNDED );
        ls = lp;
    }

    /* Offset polygon */

    for ( i = 1; i < n; i++ )
    {
        N_VectorDiff( P[i + 1], P[i], &V );
        N_VectorPerpendicular( V, &W );

        error = N_VectorNormalizeRef( &W );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorPtAlongVector( P[i], d, W, &R );
        N_CreateLineStartDirVector( &lc, R, V, NL_UNBOUNDED );

        error = N_IsectLineLine( lp, lc, &Q[i], &t1, &t2, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        lp = lc;
    }

    /* Offset end point/leg */

    if( cld EQ NL_NO )
    {
        N_VectorPtAlongVector( P[n], d, W, &Q[n] );
    }
    else
    {
        error = N_IsectLineLine( lp, ls, &Q[0], &t1, &t2, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( Q[0], &Q[n] );
    }

    N_PolygonSetIndex( qpl, n );

    /* Exit */

    EXIT:

    return (error);
} /* end N_PolygonOffset */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store  members of a control
     net structure. Proper error check is performed in case memory 
     allocation fails. A typical calling example is:

       NL_CNET    *nes;
       NL_STACKS  S;
       ...
       nes = N_AllocCNet(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     nes  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CNET *N_AllocCNet( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCNet");

    NL_CNET *nes;

    NL_NETNODE *ned;

    /* Allocate memory for the structure */

    nes = (NL_CNET *)N_Malloc( sizeof( NL_CNET ) );

    if( nes EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    ned = (NL_NETNODE *)N_Malloc( sizeof( NL_NETNODE ) );

    if( ned EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( nes );
        nes = NULL;
        return NULL;
    }

    ned->ptr = nes;
    ned->next = S->net;
    S->net = ned;

    /* Exit */

    return nes;
} /* end N_AllocCNet */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to store  a control  net object. 
     Proper error check is performed in case memory allocation fails.
  
       NL_CNET    *net;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n and m);
       ...
       net = N_AllocCNetAndArrays(n,m,&S);


   ACCESS:
   
     n,m , input  ,  Highest indexes in control net array 
     S   , input  ,  Memory stack pointer


   RETURN CODES:

     net  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CNET *N_AllocCNetAndArrays( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_CPOINT ** Pw;

    NL_CNET *net;

    /* Allocate memory */

    net = N_AllocCNet( S );

    if( net EQ NULL )
        return NULL;

    Pw = N_AllocCPt2dArray( n, m, S );

    if( Pw EQ NULL )
        return NULL;

    /* Build the object */

    N_CNetFromCPts( net, Pw, n, m );

    /* Exit */

    return net;
} /* end N_AllocCNetAndArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores members of a 
     control net  structure. Given a  control net pointer, the  routine
     searches for the  pointer  on the  memory stack.  It it is  found,
     memory is deallocated. If not, the routine does nothing. A typical
     calling example is:

       NL_CNET    *net;
       NL_STACKS  S;
       ...
       N_FreeCNet(net,&S);


   ACCESS:
   
     net , input  ,  Control net pointer 
     S   , input  ,  net's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCNet( NL_CNET *net, NL_STACKS *S )
{
    NL_NETNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->net NEQ NULL )
    {
        prev = S->net;
        curr = S->net;

        while( curr NEQ NULL AND curr->ptr NEQ net )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->net = NULL;
            }
            else /* More than one node */
            {
                S->net = S->net->next;
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
} /* end N_FreeCNet */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper pointer assignments to define
     a control net object  from a set  of control points. Memory for
     the net structure is allocated in the calling routine; only the
     pointer is passed down. A typical calling example is:

       NL_CNET    net;
       NL_CPOINT  **Pw;
       NL_INDEX   n, m;
       ...
       (allocate memory for Pw);
       ...
       N_CNetFromCPts(&net,Pw,n,m);


   ACCESS:
   
     net , in/out ,  Control net
     Pw  , input  ,  Control points
     n,m , input  ,  Highest indexes in Pw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CNetFromCPts( NL_CNET *net, NL_CPOINT ** Pw, NL_INDEX n, NL_INDEX m )
{
    net->n = n;
    net->m = m;
    net->Pw = Pw;
} /* end N_CNetFromCPts */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a control net object given the wx, wy, wz 
     and w components of  the control points. It allocates  memory to  
     store the control points, and makes proper pointer  assignments. 
     The  surface  is  restricted  to  non-rational  by  setting  the 
     w coordinate values to NL_NOW. All other memory allocation is done 
     in the calling routine. A typical calling example is:

       NL_CNET    net;
       NL_REAL    **wx, **wy, **wz, **w;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (allocate memory for wx, wy, wz and w);
       ...
       N_CNetFromCPtCoords(&net,wx,wy,wz,w,n,m,&S); 


   ACCESS:
   
     net        , in/out ,  Control net
     wx,wy,wz,w , input  ,  wx, wy, wz and w components
     n,m        , input  ,  Highest indexes in <wx,wy,wz,w>
     S          , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CNetFromCPtCoords( NL_CNET *net, NL_REAL ** wx, NL_REAL ** wy, NL_REAL ** wz, NL_REAL ** w, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_INDEX i, j;

    NL_CPOINT ** Pw;

    /* Allocate memory for the point array */

    Pw = N_AllocCPt2dArray( n, m, S );

    if( Pw EQ NULL )
        return (1);

    /* Fill in point array */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtFromWxWyWz( wx[i][j], wy[i][j], wz[i][j], w[i][j], &Pw[i][j] );
        }
    }

    /* Make pointer assignments */

    N_CNetFromCPts( net, Pw, n, m );

    /* Exit */

    return (0);
} /* end                  */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine breaks a control net object down to its 
     components. A typical calling example is:

       NL_CNET    net;
       NL_INDEX   n, m;
       NL_CPOINT  **Pw;
       ...
       N_CNetGetCPts(&net,&n,&m,&Pw);


   ACCESS:
   
     net , input  ,  Control net
     n,m , output ,  Highest indexes in Pw
     Pw  , output ,  Control points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CNetGetCPts( NL_CNET *net, NL_INDEX *n, NL_INDEX *m, NL_CPOINT *** Pw )
{
    *n = net->n;
    *m = net->m;
    *Pw = net->Pw;
} /* end N_CNetGetCPts */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a point net object given the vertices 
     of the net. A typical calling example is:

       NL_ENET   ntl;
       NL_INDEX  n, m;
       NL_POINT  **P;
       ...
       (get n, m and array P);
       ...
       N_ENetFromPts(&ntl,n,m,P);


   ACCESS:
   
     ntl , output ,  Point net object
     n,m , input  ,  Highest indexes in P
     P   , input  ,  Vertices of point net



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ENetFromPts( NL_ENET *ntl, NL_INDEX n, NL_INDEX m, NL_POINT ** P )
{
    ntl->n = n;
    ntl->m = m;
    ntl->P = P;
} /* end N_ENetFromPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the bounding box of a point net. A
     typical calling example is:

       NL_ENET       ntl;
       NL_MINMAXBOX  box;
       ...
       (get ntl);
       ...
       N_ENetGetBBox(&ntl,&box);


   ACCESS:
   
     ntl , input  ,  Point net
     box , output ,  Bounding box


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ENetGetBBox( NL_ENET *ntl, NL_MINMAXBOX *box )
{
    NL_INDEX i, j, n, m;

    NL_POINT ** P;

    NL_REAL x, y, z, xl, xr, yb, yt, zn, zf;

    /* Get local notation */

    N_ENetGetPts( ntl, &n, &m, &P );

    /* Compute min-max box */

    N_PtToXYZ( P[0][0], &x, &y, &z );

    xl = xr = x;
    yb = yt = y;
    zn = zf = z;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_PtToXYZ( P[i][j], &x, &y, &z );

            if( x LT xl )
                xl = x;

            if( x GT xr )
                xr = x;

            if( y LT yb )
                yb = y;

            if( y GT yt )
                yt = y;

            if( z LT zn )
                zn = z;

            if( z GT zf )
                zf = z;
        }
    }

    N_BBoxDefine( box, xl, xr, yb, yt, zn, zf );
} /* end N_ENetGetBBox */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a net object down to its components. 
     A typical calling example is:

       NL_ENET   ntl;
       NL_INDEX  n, m;
       NL_POINT  **P;
       ...
       (get ntl);
       ...
       N_ENetGetPts(&ntl,&n,&m,&P);


   ACCESS:
   
     ntl  , input  ,  Net
     n,m  , output ,  Highest indexes in P
     P    , output ,  Vertex array of net


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ENetGetPts( NL_ENET *ntl, NL_INDEX *n, NL_INDEX *m, NL_POINT *** P )
{
    *n = ntl->n;
    *m = ntl->m;
    *P = ntl->P;
} /* end N_ENetGetPts */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if a point net is closed or not. A 
     typical calling example is:

       NL_ENET  ntl;
       ...
       (define ntl);
       ...
       if( N_ENetIsClosed(&ntl) )  --> handle closed point net;
     

   ACCESS:
   
     ntl , input ,  Point net
     dir , input ,  Flag:
                      NL_UDIR: Check in u-direction
                      NL_VDIR: Check in v-direction


   RETURN CODES:

     NL_TRUE : Point net is closed
     NL_FALSE: Point net is NOT closed

   ***********************************************************************/

NL_BOOLEAN N_ENetIsClosed( NL_ENET *ntl, NL_FLAG dir )
{
    NL_FLAG closed = NL_TRUE;

    NL_INDEX i, j, n, m;

    NL_REAL d;

    NL_POINT ** P;

    /* Get local notation */

    N_ENetGetPts( ntl, &n, &m, &P );

    /* Check closeness */

    switch( dir )
    {
        case NL_UDIR:
            for ( j = 0; j <= m; j++ )
            {
                N_DistPtPt( P[0][j], P[n][j], &d );

                if( d GT NL_MTOL )
                {
                    closed = NL_FALSE;
                    break;
                }
            }
            break;

        case NL_VDIR:
            for ( i = 0; i <= n; i++ )
            {
                N_DistPtPt( P[i][0], P[i][m], &d );

                if( d GT NL_MTOL )
                {
                    closed = NL_FALSE;
                    break;
                }
            }
            break;

        default:

            closed = NL_FALSE;
    }

    return (closed);
} /* end N_ENetIsClosed */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry  routine finds the index of the control net leg that 
     is the closest to a given point. It also returns the projection of
     this point to the closest leg. A typical calling example is:

       NL_ENET   ntl;
       NL_POINT  P, Q;
       NL_INDEX  i, j;
       NL_REAL   t;
       NL_FLAG   flg;
       ...
       (get ntl and P);
       ...
       N_NetGetClosestLegIndex(&ntl,P,&Q,&i,&j,NL_UDIR,&t,&flg);


   ACCESS:
   
     ntl , input  ,  Control net
     P   , input  ,  Given point
     Q   , output ,  Projection of P to the closest leg
     i,j , output ,  Indexes of closest leg
     dir , output ,  Flag = 
                       NL_UDIR: P is closest to <P[i][j],P[i+1][j]>
                       NL_VDIR: P is closest to <P[i][j],P[i][j+1]>
     t   , output ,  Parameter corresponding to Q
     flg , output ,  Flag = 
                       NL_TRUE : Projection is successful
                       NL_FALSE: Projection   is   unsuccessful,  i.e.  P
                              projects outside of each leg.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_NetGetClosestLegIndex( NL_ENET *ntl, NL_POINT P, NL_POINT *Q, NL_INDEX *i, NL_INDEX *j, NL_FLAG *dir, NL_REAL *t, NL_FLAG *flg )
{
    NL_FLAG prj, error;

    NL_INDEX k, l, n, m, iu, ju, iv, jv;

    NL_POINT ** R, S;

    NL_LINESEG lsg;

    NL_REAL u, v, du, dum, dv, dvm, tu, tv;

    /* Get local notation */

    N_ENetGetPts( ntl, &n, &m, &R );

    *flg = NL_FALSE;

    /* Scan the legs in u-direction */

    dum = NL_BIGD;
    iu = ju = 0;
    tu = 0.0;

    for ( l = 0; l <= m; l++ )
    {
        for ( k = 0; k < n; k++ )
        {
            N_CreateLinePtPt( &lsg, R[k][l], R[k + 1][l], NL_BOUNDED );

            error = N_ProjectPtLine( lsg, P, &S, &u, &prj );

            if( error EQ NL_YES )
                return (1);

            if( prj EQ NL_FALSE )
                continue;

            N_DistPtPt( P, S, &du );

            if( du LT dum )
            {
                dum = du;
                iu = k;
                ju = l;
                tu = u;
            }
        }
    }

    /* Scan the legs in v-direction */

    dvm = NL_BIGD;
    iv = jv = 0;
    tv = 0.0;

    for ( k = 0; k <= n; k++ )
    {
        for ( l = 0; l < m; l++ )
        {
            N_CreateLinePtPt( &lsg, R[k][l], R[k][l + 1], NL_BOUNDED );

            error = N_ProjectPtLine( lsg, P, &S, &v, &prj );

            if( error EQ NL_YES )
                return (1);

            if( prj EQ NL_FALSE )
                continue;

            N_DistPtPt( P, S, &dv );

            if( dv LT dvm )
            {
                dvm = dv;
                iv = k;
                jv = l;
                tv = v;
            }
        }
    }

    /* Pick the closest leg */

    if( dum NEQ NL_BIGD AND dvm NEQ NL_BIGD )
    {
        if( dum LT dvm )
        {
            N_Combine2Pts( 1.0 - tu, R[iu][ju], tu, R[iu + 1][ju], Q );
            *t = tu;
            *i = iu;
            *j = ju;
            *dir = NL_UDIR;
        }
        else
        {
            N_Combine2Pts( 1.0 - tv, R[iv][jv], tv, R[iv][jv + 1], Q );
            *t = tv;
            *i = iv;
            *j = jv;
            *dir = NL_VDIR;
        }
        *flg = NL_TRUE;
    }
    else if( dum NEQ NL_BIGD )
    {
        N_Combine2Pts( 1.0 - tu, R[iu][ju], tu, R[iu + 1][ju], Q );
        *t = tu;
        *i = iu;
        *j = ju;
        *dir = NL_UDIR;
        *flg = NL_TRUE;
    }
    else if( dvm NEQ NL_BIGD )
    {
        N_Combine2Pts( 1.0 - tv, R[iv][jv], tv, R[iv][jv + 1], Q );
        *t = tv;
        *i = iv;
        *j = jv;
        *dir = NL_VDIR;
        *flg = NL_TRUE;
    }

    /* Exit */

    return (0);
} /* end N_NetGetClosestLegIndex */
#endif // NLIB_UNUSED
