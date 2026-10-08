// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*************************************************************************/
/* CrvBasic.c : Basic Function Definitions that act on NL_CURVE objects  */
/*************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the curve is three diemsional or
     not. A typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_CrvIs3d(&cur) )  --> handle 3-D case;


   ACCESS:

     cur , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE:  Curve is three dimensional
     NL_FALSE: Curve is NOT three dimensional

   ***********************************************************************/

NL_BOOLEAN N_CrvIs3d(NL_CURVE* cur)
{
    NL_REAL t, wz;

    NL_CPOINT* Pw;

    /* Get local notation */

    Pw = cur->pol->Pw;

    /* Check dimensionality */

    N_CPtToWxWyWz(Pw[0], &t, &t, &wz, &t);

    if (wz NEQ NL_NOZ)
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CrvIs3d */

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if (n+p+1) equals m, that is, the number 
     of control  points, plus the degree, plus one equals the number of 
     knots. It  also  performs a  consistency check  by checking if all 
     control points have the same dimension and  rationality. A typical
     calling example is:

       NL_CURVE   cur;
       NL_STRING  rname;
       ...
       (define cur and get rname);
       ...
       N_CrvCountsAreValid(&cur,rname);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvCountsAreValid( NL_CURVE *cur, NL_STRING rname )
{
    NL_FLAG olddim, oldrat, newdim, newrat, error = NL_NO;
    NL_INDEX i, n, m;
    NL_REAL *U, x, y, z, w;
    NL_DEGREE p;
    NL_CPOINT *Pw;

    /* Convert to local notation */
    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Check definition and degree */
    if( (n + p + 1)NEQ m )
        NL_ERROR( NL_CUR_ERR );

    if( p GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Check for consistency */
    N_CPtToWxWyWz( Pw[0], &x, &y, &z, &w );

    if( z EQ NL_NOZ )
        olddim = 2;
    else
        olddim = 3;

    if( w EQ NL_NOW )
        oldrat = 0;
    else
        oldrat = 1;

    for ( i = 1; i <= n; i++ )
    {
        N_CPtToWxWyWz( Pw[i], &x, &y, &z, &w );

        if( z EQ NL_NOZ )
            newdim = 2;
        else
            newdim = 3;

        if( newdim NEQ olddim )
            NL_ERROR( NL_CUR_ERR );

        if( w EQ NL_NOW )
            newrat = 0;
        else
            newrat = 1;

        if( newrat NEQ oldrat )
            NL_ERROR( NL_CUR_ERR );
    }

    /* Exit */
    EXIT:

    return (error);
} /* end N_CrvCountsAreValid */

/*******************************************************************//**


   DESCRIPTION:

     This error  routine checks whether all the curve weights are 
     within the range <NL_WMIN,NL_WMAX>. Non-rational curves are
     considered within bounds.  These weight limits are set in
     "globals.h". A typical calling example is:

       NL_CURVE   cur;
       NL_STRING  rname;
       ...
       (define cur and get rname);
       ...
       N_CrvWeightsAreValid(&cur,rname);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     rname , input  ,  Name of routine in which error is checked 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvWeightsAreValid( NL_CURVE *cur, NL_STRING rname )
{
    NL_FLAG error = NL_NO;
    NL_INDEX i, n;
    NL_REAL w;
    NL_CPOINT *Pw;

    /* Convert to local notation */
    N_CrvGetCPts( cur, &n, &Pw );

    /* Check the weights */
    if( N_IsCrvRat( cur ) )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CPtGetW( Pw[i], &w );

            /* when weight is out of bounds */
            if( w LT NL_WMIN OR w GT NL_WMAX )
            {
                /* set error and quit */
                N_ErrSet( NL_WEI_ERR, rname );
                error = NL_YES;
                break;
            }
        }
    }

    return (error);
} /* end N_CrvWeightsAreValid */

/*******************************************************************//**


   DESCRIPTION:

     This error routine performs a complete curve check, i.e. it checks
     the input data for:
       (1) curve definition constants,
       (2) curve weights, and
       (3) the knot vector.
     Other  error  routines  are used  to check  each type of  error. A 
     typical calling example is:

       NL_CURVE   cur;
       NL_STRING  rname;
       ...
       (define cur and get rname);
       ...
       N_CrvIsValid(&cur,rname);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsValid( NL_CURVE *cur, const TCHAR * rname )
{
    NL_FLAG error = NL_NO;
    NL_DEGREE p;
    NL_KNOTVECTOR *knt;
    NL_INDEX Nu;

    /* Convert to local notation */

    N_CrvGetKnotVector( cur, &knt );
    N_CrvGetDegree( cur, &p );

    /* Check curve definition */

    error = N_CrvCountsAreValid( cur, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    /* Check curve weights */
    error = N_CrvWeightsAreValid( cur, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    /* Check knot vector */
    error = N_KnotVectorIsValid( knt, p, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    /* Check for Equal Control Points */

    error = N_CrvHasEqualCPts( cur, NL_MTOL, &Nu );

    if( error EQ NL_YES )
        return (1);

    /* Exit */

    return (0);
} /* end N_CrvIsValid */



/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if the curve end points reverse 
     direction at curve start or end. 
     It is not necessarily an error, but it
     is a useful warning. We only test if there are 4 or more 
     control points and a normalised 'dot' of -0.95 as the threshold.
     Optionally, the routine will correct the reversal.
     A typical calling example is:

       NL_CURVE   cur;
       NL_FLAG  Fixit = true;
       ...
       (define cur);
       ...
       N_CrvIsNotReversed(&cur, Fixit);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     Fixit , input  ,  TRUE = Move CPts to fix a Cpts sequence reversal
                       FALSE= don't

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsNotReversed
 ( NL_CURVE *cur,         /* in : NURBS curve */
   NL_FLAG Fixit)         /* in : TRUE = Move CPts to fix a Cpts sequence reversal, FALSE=don't */
{
    NL_INDEX n, m;
    NL_REAL *U, dot, mag;
    NL_DEGREE p;
    NL_CPOINT *Pw;
    NL_POINT P0, P1, P2, P1P0, P2P1;
    NL_FLAG error = 0;

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( n < 4 )
        return (0);

    N_CPtToPtEuclid( Pw[0], &P0 );
    N_CPtToPtEuclid( Pw[1], &P1 );
    N_CPtToPtEuclid( Pw[2], &P2 );
    N_VectorDiff( P1, P0, &P1P0 );
    N_VectorDiff( P2, P1, &P2P1 );
    N_VectorNormalize( P1P0, &P1P0, &mag );
    N_VectorNormalize( P2P1, &P2P1, &mag );

    N_VectorDot( P1P0, P2P1, &dot );

    if( dot < -0.95 && mag > NL_MTOL )
    {
      if( Fixit )
        {
            Pw[1].x = (P0.x + P2.x) / 2.0;
            Pw[1].y = (P0.y + P2.y) / 2.0;

            if( Pw[1].z != NL_NOZ )
                Pw[1].z = (P0.z + P2.z) / 2.0;

            if( Pw[1].w != NL_NOW )
                Pw[1].w = 1.0;
        }
        error = 1; /*front end */
    }

    N_CPtToPtEuclid( Pw[n], &P0 );
    N_CPtToPtEuclid( Pw[n - 1], &P1 );
    N_CPtToPtEuclid( Pw[n - 2], &P2 );
    N_VectorDiff( P1, P0, &P1P0 );
    N_VectorDiff( P2, P1, &P2P1 );
    N_VectorNormalize( P1P0, &P1P0, &mag );
    N_VectorNormalize( P2P1, &P2P1, &mag );

    N_VectorDot( P1P0, P2P1, &dot );

    if( dot < -0.95 && mag > NL_MTOL )
    {
        if( Fixit )
        {
            Pw[n - 1].x = (P0.x + P2.x) / 2.0;
            Pw[n - 1].y = (P0.y + P2.y) / 2.0;

            if( Pw[n - 1].z != NL_NOZ )
                Pw[n - 1].z = (P0.z + P2.z) / 2.0;

            if( Pw[n - 1].w != NL_NOW )
                Pw[n - 1].w = 1.0;
        }
        error += 2; /*back end */
    }

    return (error);
}


/*******************************************************************//**


   DESCRIPTION:

     This error  routine checks  if a  curve structure  has sufficient
     memory to store given control points and knots. A typical calling
     example is:

       NL_CURVE   cur;
       NL_INDEX   np, mk;
       NL_STRING  rname;
       ...
       (define cur and get np, mk and rname);
       ...
       N_CrvIsSized(&cur,np,mk,rname);


   ACCESS:
   
     cur   , input ,  NURBS curve
     np    , input ,  Expected highest index in control point array
     mk    , input ,  Expected highest index in knot vector array
     rname , input ,  Routine name error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsSized( NL_CURVE *cur, NL_INDEX np, NL_INDEX mk, const NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    /* Get local notation */
    N_CrvGetArraySizes( cur, &n, &m );

    /* Check storage */
    if( n LT np OR m LT mk )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_CrvIsSized */

/*******************************************************************//**


  DESCRIPTION:

     This error routine checks if any two consecutive control points
     within a curve are equal. Only the first index of equal (within 
     Tolerance) control points is returned. 
     Use Tol = NL_MTOL for strict equality.

       NL_CURVE      Cur;
       ...
       (define Cur);
       ...
       N_CrvHasEqualCPts(&Cur, Tol, &Nu);


   ACCESS:   
     Cur   , input  ,  NURBS curve
     Tol   , inout  .  Distance tolerance between adjacent control points

   RETURN CODES:

     0 : No control points are equal (within Tol)
     1 : Control Points Pw[Nu] and Pw[Nu-1] are equal (within Tol)
     
   ***********************************************************************/
NL_FLAG N_CrvHasEqualCPts( NL_CURVE *cur, NL_REAL Tol, NL_INDEX *Nu )
{
    NL_CPOINT Cp0, Cp1, Rw;
    NL_REAL d;
    NL_INDEX i, n;
    n = cur->pol->n;
    Cp1 = cur->pol->Pw[0];

    for ( i = 1; i <= n; i++ )
    {
        Cp0 = Cp1;
        Cp1 = cur->pol->Pw[i];
        N_Diff2CPts( Cp0, Cp1, &Rw );
        N_CPtMagnitude( Rw, &d );

        if( d < Tol )
        {
            *Nu = i;
            return (1);
        }
    }

    return (0);
} /* end N_CrvHasEqualCPts */

/*******************************************************************//**

  DESCRIPTION:

     This error routine finds and fixes coincident Cpts spaces out Cpts
     within Tol of one another. Coincident control points are found
     in sequence between neighbors Cpt[ii] and Cpt[ii-1].

     when ii is in the first half of the Control polygon,
              Coincident Cpt[ii] is spread out towards Cpt[ii+1].
     when ii is in the 2nd half of the control polygon,
              Coincident Cpt[ii-1] is spread out towards Cpt[ii-2]

     When the space available for spreading control points is large the move
     is limited to 10*Tol from the 1st Coincdent Cpt.  When the space is small
     the 2nd Cpt is placed in the center of the availble space.

     Note: when the gap between a Coincident Cpt's neighbors is less than 2*Tol
     the output of this method will still have near coincident control points.
     Assuming most coincident control points will be towards the ends of the curve,
     coincident points are fixed from the middle out to the ends so that a sequence
     of several coincident Cpts will have a chance to get spread out in a single
     pass of this method.

     If the curve is just too short to spread the control points this function will
     not be able to fix the problem.  For some coincident cases with multiple
     Cpts in the very center of the Cpt sequence, a sequence of calls to this
     method will continue to spread control point positions and may resolve
     any coincident control point problems.  This function executes
     twice when the first pass fails to fix all the coincidences.

     No action is taken when the Curve has only 2 Cpts.

       NL_CURVE      Cur;
       ...
(define Cur);
       ...
       N_CrvReplaceEqualCPts(&Cur, Tol);

   ACCESS:   
     Cur   , input  ,  NURBS curve
     Tol   , inout  .  Distance tolerance between adjacent control points
     Cur   , output ,  NURBS curve with control points adjusted.
   RETURN CODES:
     0 : No control points were changed
     1 : some control points were changed
     pOptStillBroken : when given and when rtn is 1
         0 : No coincident Cpts remain
         1 : Coincident Cpts remain - repeating this call
             may fix the problem - but that's not guaranteed.
   ***********************************************************************/
NL_FLAG N_CrvReplaceEqualCPts
 (NL_CURVE * cur,             // in : Target curve
  NL_REAL    Tol)             // in : 3d min dist between unique points
{
  // init output
  NL_FLAG  bRtn = FALSE ;     // FALSE = no change, TRUE = changed

  // locals
  NL_INDEX  ii, jj ;                     
  NL_INDEX  n = cur->pol->n ; // max CPt index
  NL_FLAG   bDone = TRUE ;   // TRUE = Cpts still has coincident Cpts
                              // FALSE=Cpts are no longer coincident

  // no work - need at least 3 Cpts
  if(n < 2)
    { return(bRtn) ; }
  
  NL_INDEX  nHalf = n/2 ;
  NL_CPOINT Rw ;             // Rw = Vector between neighbor Cpts
  NL_REAL   dDistTgt ;       // dDistTgt = Dist between TgtCpt and its previous neighbor Cpt. Desired to end up greater than Tol
  NL_REAL   dDistGap ;       // dDistGap = the space available between TgtCpt neighbors for spreading out a duplicate TgtCpt position
  NL_REAL   r = 0.01 ;       // percent of dist along vector.  must be between [0 1]

  // TgtCpt = Pw[jj] - its usually moved when coincident
  //                   except when jj=n and then Pw[jj-1] is moved instead.
  // +------+---------+---------+----------+
  // | iter | Cp0     | Cp1     | Cp2      |
  // +------+---------+---------+----------+
  // |  1   | Pw[0]   | Pw[1]   | Pw[2]    |  coincident when Pw[jj] and Pw[jj-1] are close
  // |  jj  | Pw[jj-1]| Pw[jj]  | Pw[jj+1] |  when jj < n/2 move Pw[jj]   towards Pw[jj+1]
  // |  n   | Pw[jj-2]| Pw[jj-1]| Pw[jj]   |  when jj > n/2 move Pw[jj-1] towards Pw[jj-2]
  // +------+---------+---------+----------+
      
  // for two passes - just in case sequences of multiple coincident points need to be spread out in multiple passes
  for(ii=0;ii<2;ii++)
    {
      // iter every CPt in the 2nd half of the Cpt set - fix duplicates in order from the middle to end
      for(jj=nHalf+1;jj<=n;jj++)
        {
          // Space between Pw[jj] and Pw[jj-1] (duplicate when dist < Tol)
          N_Diff2CPts( cur->pol->Pw[jj-1], cur->pol->Pw[jj], &Rw) ;
          N_CPtMagnitude( Rw, &dDistTgt) ;
      
          // when Pw[jj-1] and Pw[jj] are duplicate Cpts - move Pw[jj-1] towards Pw[jj-2]
          if(dDistTgt < Tol) 
            {
              // space available to move Pw[jj-1] between Pw[jj-2] and Pw[jj]
              N_Diff2CPts( cur->pol->Pw[jj-2], cur->pol->Pw[jj], &Rw) ;
              N_CPtMagnitude( Rw, &dDistGap) ;
      
              // plan to move Pw[jj-1] between Pw[jj] and Pw[jj-2] limited to 10.0 * Tol
              r = (0.5*dDistGap > 10.0*Tol) ? 10.0*Tol/dDistGap : 0.5 ;
              
              // set Pw[jj-1] = (r)*Pw[jj-2] + (1-r)*Pw[jj] - keep moving point close to Pw[jj]
              N_Combine2CPts((r),   cur->pol->Pw[jj-2],
                             (1-r), cur->pol->Pw[jj], &cur->pol->Pw[jj-1]) ;
      
              // remember the change
              bRtn = 1;
              if(dDistGap < 2 * Tol) { bDone = FALSE ; }
      
            } // end Pw[jj-1] and Pw[jj] are duplicate Cpts
        } // end iter jj, every iter every CPt in the 2nd half of the Cpt set - fix duplicates in order from the middle to end
      
      // iter every CPt in the 1st half of the Cpt set - fix duplicates in order from the middle to end
      for(jj=nHalf;jj>=1;jj--)
        {
          // Space between Pw[jj] and Pw[jj-1] (duplicate when dist < Tol)
          N_Diff2CPts( cur->pol->Pw[jj-1], cur->pol->Pw[jj], &Rw) ;
          N_CPtMagnitude( Rw, &dDistTgt) ;
      
          // when Pw[jj-1] and Pw[jj] are duplicate Cpts - move Pw[jj] towards Pw[jj+1]
          if(dDistTgt < Tol) 
            {
              // space available to move Pw[jj] between Pw[jj-1] and Pw[jj+1]
              N_Diff2CPts( cur->pol->Pw[jj-1], cur->pol->Pw[jj+1], &Rw) ;
              N_CPtMagnitude( Rw, &dDistGap) ;
      
              // plan to move Pw[jj] between Pw[jj-1] and Pw[jj+1] limited to 10.0 * Tol
              r = (0.5*dDistGap > 10.0*Tol) ? 10.0*Tol/dDistGap : 0.5 ;
      
              // set Pw[jj] = (1-r)*Pw[jj-1] + (r)*Pw[jj+1] - Keep moving point close to Pw[jj-1]
              N_Combine2CPts( (1-r), cur->pol->Pw[jj-1],
                              (r),   cur->pol->Pw[jj+1], &cur->pol->Pw[jj]) ;
      
              // remember the change
              bRtn = 1;
              if(dDistGap < 2 * Tol) { bDone = FALSE ; }
      
           } // end Pw[jj-1] and Pw[jj] are coincident check
        } // end iter jj, every CPt in the 1st half of the Cpt set - fix duplicates in order from the middle to end

      // all done?
      if(bDone == TRUE)
        { break ; }

    } // end iter ii, multiple passes in case a sequence of duplicate Cpts need multiple passes to spread out

  // all done
  return (bRtn) ;

} /* end N_CrvReplaceEqualCPts */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a curve
     structure. Proper error check is performed in case memory 
     allocation fails. A typical calling example is:

       NL_CURVE   *cus;
       NL_STACKS  S;
       ...
       cus = N_AllocCrv(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     cus  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE *N_AllocCrv( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCrv");

    NL_CURVE *cus;

    NL_CURNODE *cud;

    /* Allocate memory for the structure */

    cus = (NL_CURVE *)N_Malloc( sizeof( NL_CURVE ) );

    if( cus EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    cud = (NL_CURNODE *)N_Malloc( sizeof( NL_CURNODE ) );

    if( cud EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cus );
        cus = NULL;
        return NULL;
    }

    cud->ptr = cus;
    cud->next = S->cur;
    S->cur = cud;

    /* Exit */

    return cus;
} /* end N_AllocCrv */



/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a curve  defined by the 
     usual parameters <n,p,m>. Proper error checks are performed in 
     case memory allocations fail. A typical calling example is:

       NL_CURVE   *cur;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (get n, p and m);
       ...
       cur = N_AllocCrvAndArrays(n,p,m,&S);


   ACCESS:
   
     n  , input  ,  Highest index in control point array
     p  , input  ,  Degree of the curve
     m  , input  ,  Highest index in knot vector array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     cur  : Pointer to curve if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE *N_AllocCrvAndArrays( NL_INDEX n, NL_DEGREE p, NL_INDEX m, NL_STACKS *S )
{
    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    NL_CURVE *cur;

    /* Allocate memory */

    pol = N_AllocCPolygonAndArray( n, S );

    if( pol EQ NULL )
        return NULL;

    knt = N_AllocKnotVectorAndArray( m, S );

    if( knt EQ NULL )
        return NULL;

    cur = N_AllocCrv( S );

    if( cur EQ NULL )
        return NULL;

    /* Build curve structure */
    N_CrvFromCPolygonAndKnotVector( cur, pol, p, knt );

    /* Exit */

    return cur;
} /* end N_AllocCrvAndArrays */


/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 1-D array of curves each
     defined by the same set of parameters <n,p,m>. Proper error check 
     is performed in case  memory allocation fails. A  typical calling
     example is:

       NL_CURVE   **cu2;
       NL_INDEX   n, m, k;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (get n, p, m and k);
       ...
       cu2 = N_Alloc1dArrayCrvs(n,p,m,k,&S);


   ACCESS:
   
     n  , input  ,  Highest index in control point arrays
     p  , input  ,  Degree of each curve
     m  , input  ,  Highest index in knot vector arrays
     k  , input  ,  Highest index of curve  array cu2[0],...,cu2[k];
                    cu2[i], 0<=i<=k, is a pointer to the i-th curve.
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     cu2  : Pointer to array of curves if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE ** N_Alloc1dArrayCrvs( NL_INDEX n, NL_DEGREE p, NL_INDEX m, NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc1dArrayCrvs");

    NL_INDEX i;

    NL_CURVE ** cu2;

    NL_CU2NODE *u2d;

    /* Allocate memory for curve pointer arrays */

    cu2 = (NL_CURVE ** )N_Malloc( (k + 1) * sizeof( NL_CURVE * ) );

    if( cu2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Allocate memory for each curve in the array */

    for ( i = 0; i <= k; i++ )
    {
        cu2[i] = N_AllocCrvAndArrays( n, p, m, S );

        if( cu2[i]EQ NULL )
        {
            N_Free( cu2 );
            cu2 = NULL;
            return NULL;
        }
    }

    /* Put pointer on memory stack */

    u2d = (NL_CU2NODE *)N_Malloc( sizeof( NL_CU2NODE ) );

    if( u2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cu2 );
        cu2 = NULL;
        return NULL;
    }

    u2d->ptr = cu2;
    u2d->next = S->cu2;
    S->cu2 = u2d;

    /* Exit */

    return cu2;
} /* end N_Alloc1dArrayCrvs */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store an array of curve pointers.
     Proper error check is performed in case memory allocation fails. A
     typical calling example is:

       NL_CURVE   **cua;
       NL_INDEX   k;
       NL_STACKS  S;
       ...
       (get k);
       ...
       cua = N_AllocArrayCrvPtrs(k,&S);


   ACCESS:
   
     k , input  ,  Highest index of curve pointer array
     S , input  ,  Memory stacks pointer


   RETURN CODES:

     cua  : Pointer to array of curve pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE ** N_AllocArrayCrvPtrs( NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocArrayCrvPtrs");

    NL_CURVE ** cua;

    NL_CU2NODE *u2d;

    /* Allocate memory */

    cua = (NL_CURVE ** )N_Malloc( (k + 1) * sizeof( NL_CURVE * ) );

    if( cua EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    u2d = (NL_CU2NODE *)N_Malloc( sizeof( NL_CU2NODE ) );

    if( u2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cua );
        cua = NULL;
        return NULL;
    }

    u2d->ptr = cua;
    u2d->next = S->cu2;
    S->cu2 = u2d;

    /* Exit */

    return cua;
} /* end N_AllocArrayCrvPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store an array of curve pointers.
     It also allocates the curve structures pointed to by the pointers.
     Proper error check is performed in case memory allocation fails. A
     typical calling example is:

       NL_CURVE   **curs;
       NL_INDEX   k;
       NL_STACKS  S;
       ...
       (get k);
       ...
       curs = N_AllocArrayCrvPtrsAndData(k,NL_YES,&S);


   ACCESS:
   
     k   , input  ,  Highest index of curve pointer array
     flg , input  ,  Flag:
                      NL_YES: initialize the curve objects to null objects
                      NL_NO : do not initialize the curve objects
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     curs : Pointer to array of curve pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE ** N_AllocArrayCrvPtrsAndData( NL_INDEX k, NL_FLAG flg, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocArrayCrvPtrsAndData");

    NL_CURVE ** curs;

    NL_INDEX ii;

    NL_CU2NODE *u2d;

    /* Allocate memory */

    curs = (NL_CURVE ** )N_Malloc( (k + 1) * sizeof( NL_CURVE * ) );

    if( curs EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    u2d = (NL_CU2NODE *)N_Malloc( sizeof( NL_CU2NODE ) );

    if( u2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( curs );
        curs = NULL;
        return NULL;
    }

    u2d->ptr = curs;
    u2d->next = S->cu2;
    S->cu2 = u2d;

    /* Allocate the curve structures */

    for ( ii = 0; ii <= k; ii++ )
    {
        curs[ii] = N_AllocCrv( S );

        if( curs[ii]EQ NULL )
            return NULL;

        if( flg EQ NL_YES )
            N_CrvInitArrays( curs[ii] );
    }

    /* Exit */

    return curs;
} /* end N_AllocArrayCrvPtrsAndData */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to  store an  array of NL_REAL curve 
     pointers. Proper error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_CURVE   ***cub;
       NL_INDEX   k;
       NL_STACKS  S;
       ...
       (get k);
       ...
       cub = N_AllocArrayRealCrvPtrs(k,&S);


   ACCESS:
   
     k , input  ,  Highest index of curve pointer array
     S , input  ,  Memory stacks pointer


   RETURN CODES:

     cub  : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE *** N_AllocArrayRealCrvPtrs( NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocArrayRealCrvPtrs");

    NL_CURVE *** cub;

    NL_CU3NODE *u3d;

    /* Allocate memory */

    cub = (NL_CURVE *** )N_Malloc( (k + 1) * sizeof( NL_CURVE ** ) );

    if( cub EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    u3d = (NL_CU3NODE *)N_Malloc( sizeof( NL_CU3NODE ) );

    if( u3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cub );
        cub = NULL;
        return NULL;
    }

    u3d->ptr = cub;
    u3d->next = S->cu3;
    S->cu3 = u3d;

    /* Exit */

    return cub;
} /* end N_AllocArrayRealCrvPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to  store an  array of triple curve 
     pointers. Proper error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_CURVE   ****cuc;
       NL_INDEX   k;
       NL_STACKS  S;
       ...
       (get k);
       ...
       cuc = N_AllocArrayTripleCrvPtrs(k,&S);


   ACCESS:
   
     k , input  ,  Highest index of curve pointer array
     S , input  ,  Memory stacks pointer


   RETURN CODES:

     cuc  : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE **** N_AllocArrayTripleCrvPtrs( NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocArrayTripleCrvPtrs");

    NL_CURVE **** cuc;

    NL_CU4NODE *u4d;

    /* Allocate memory */

    cuc = (NL_CURVE **** )N_Malloc( (k + 1) * sizeof( NL_CURVE *** ) );

    if( cuc EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    u4d = (NL_CU4NODE *)N_Malloc( sizeof( NL_CU4NODE ) );

    if( u4d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cuc );
        cuc = NULL;
        return NULL;
    }

    u4d->ptr = cuc;
    u4d->next = S->cu4;
    S->cu4 = u4d;

    /* Exit */

    return cuc;
} /* end N_AllocArrayTripleCrvPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory  to store a  2-D array of curve
     pointers. Proper  error  check  is performed  in  case  memory  
     allocation fails. A typical calling example is:

       NL_CURVE   ***cs3;
       NL_INDEX   k, l;
       NL_STACKS  S;
       ...
       (get k and l);
       ...
       cs3 = N_Alloc2dArrayCrvPtrs(k,l,&S);


   ACCESS:
   
     k,l  , input  ,  Highest  indexes  in   curve  pointer  array: 
                      cs3[i][j] points to the (i,j)th curve 
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     cs3  : Pointer to 2-D array of curve pointers if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CURVE *** N_Alloc2dArrayCrvPtrs( NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc2dArrayCrvPtrs");

    NL_INDEX i, j;

    NL_CURVE *** cs3, ** cs2;

    NL_CU2NODE *c2d;

    NL_CU3NODE *c3d;

    /* Allocate memory for curve pointer arrays */

    cs3 = (NL_CURVE *** )N_Malloc( (k + 1) * sizeof( NL_CURVE ** ) );

    if( cs3 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    cs2 = (NL_CURVE ** )N_Malloc( (k + 1) * (l + 1) * sizeof( NL_CURVE * ) );

    if( cs2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cs3 );
        cs3 = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    j = 0;

    for ( i = 0; i <= k; i++ )
    {
        cs3[i] = &cs2[j];
        j = j + l + 1;
    }

    /* Put pointers on memory stacks */

    c2d = (NL_CU2NODE *)N_Malloc( sizeof( NL_CU2NODE ) );

    if( c2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cs3 );
        cs3 = NULL;
        N_Free( cs2 );
        cs2 = NULL;
        return NULL;
    }

    c3d = (NL_CU3NODE *)N_Malloc( sizeof( NL_CU3NODE ) );

    if( c3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cs3 );
        cs3 = NULL;
        N_Free( cs2 );
        cs2 = NULL;
        N_Free( c2d );
        c2d = NULL;
        return NULL;
    }

    c2d->ptr = cs2;
    c2d->next = S->cu2;
    S->cu2 = c2d;

    c3d->ptr = cs3;
    c3d->next = S->cu3;
    S->cu3 = c3d;

    /* Exit */

    return cs3;
} /* end N_Alloc2dArrayCrvPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper pointer  assignments to define
     a curve object from  polygon and knot vector  objects. Memory is
     allocated in the calling routine; only pointers are passed down.
     A typical calling example is:

       NL_CURVE       cur;
       NL_CPOLYGON    pol;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (define pol and knt);
       ...
       N_CrvFromCPolygonAndKnotVector(&cur,&pol,p,&knt);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     pol , input  ,  Polygon
     p   , input  ,  Degree
     knt , input  ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFromCPolygonAndKnotVector( NL_CURVE *cur, NL_CPOLYGON *pol, NL_DEGREE p, NL_KNOTVECTOR *knt )
{
    cur->pol = pol;
    cur->p = p;
    cur->knt = knt;
} /* end          */

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a curve object given  control points and
     knots. It allocates  memory to  store the  control  polygon and
     knot vector objects, and  makes proper  pointer  assignments to
     create  the curve  object. Memory  for  the  curve structure is
     allocated in the calling routine. A typical calling example is:

       NL_CURVE   cur;
       NL_CPOINT  *Pw;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_REAL    *U;
       NL_STACKS  S;
       ...
       (allocate memory for Pw and U);
       ...
       N_CrvFromCPtsAndKnots(&cur,Pw,n,p,U,m,&S);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     Pw  , input  ,  Control points
     n   , input  ,  Highest index in Pw
     p   , input  ,  Degree
     U   , input  ,  Knots
     m   , input  ,  Highest index in U
     S   , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/


NL_FLAG N_CrvFromCPtsAndKnots( NL_CURVE *cur, NL_CPOINT *Pw, NL_INDEX n, NL_DEGREE p, NL_REAL *U, NL_INDEX m, NL_STACKS *S )
{
    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    /* Allocate memory for polygon and knot vector structures */

    pol = N_AllocCPolygon( S );

    if( pol EQ NULL )
        return (1);

    knt = N_AllocKnotVector( S );

    if( knt EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_CPolygonFromCPts( pol, Pw, n );
    N_KnotVectorFromRealArray( knt, U, m );
    N_CrvFromCPolygonAndKnotVector( cur, pol, p, knt );

    /* Exit */

    return (0);
} /* end          */

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a curve object  given the <wx,wy,wz,w> 
     coordinates of the control points and the knots. It allocates 
     memory to store the control  polygon and knot vector objects, 
     and  makes  proper  pointer  assignments to  create the curve 
     object. Memory  for the  curve  structure is allocated in the 
     calling routine. A typical calling example is:

       NL_CURVE   cur;
       NL_REAL    *wx, *wy, *wz, *w, *U;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (allocate memory for wx, wy, wz, w and U);
       ...
       N_CrvFromCPtCoordsAndKnots(&cur,wx,wy,wz,w,n,p,U,m,&S);


   ACCESS:
   
     cur         , in/out ,  NURBS curve
     wx,wy,wz,w  , input  ,  Coordinates of control points
     n           , input  ,  Highest index in <wx,wy,wz,w>
     p           , input  ,  Degree
     U           , input  ,  Knots
     m           , input  ,  Highest index in U
     S           , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvFromCPtCoordsAndKnots */
NL_FLAG N_CrvFromCPtCoordsAndKnots( NL_CURVE *cur, NL_REAL *wx, NL_REAL *wy, NL_REAL *wz, NL_REAL *w, NL_INDEX n, NL_DEGREE p, NL_REAL *U, NL_INDEX m, NL_STACKS *S )
{
    NL_CPOINT *Pw;

    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    /* Allocate memory */

    Pw = N_XYZToCPtArray( wx, wy, wz, w, n, S );

    if( Pw EQ NULL )
        return (1);

    pol = N_AllocCPolygon( S );

    if( pol EQ NULL )
        return (1);

    knt = N_AllocKnotVector( S );

    if( knt EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_CPolygonFromCPts( pol, Pw, n );
    N_KnotVectorFromRealArray( knt, U, m );
    N_CrvFromCPolygonAndKnotVector( cur, pol, p, knt );

    /* Exit */

    return (0);
} /* end         */

/*******************************************************************//**
   DESCRIPTION:

     Given a curve object, this  routine allocates memory to store
     control points and knots. A typical calling example is:

       NL_CURVE   cur;
       NL_STACKS  S;
       ...
       N_AllocCrvArrays(&cur,n,p,m,&S);

     where  <n,p,m>  are  the  usual  curve parameters. Since  the
     declaration  "NL_CURVE cur"  defines the data type and allocates
     memory, memory is needed to store the polygon and knot vector
     objects only.


   ACCESS:
   
     cur , in/out ,  NURBS curve data type
     n   , input  ,  Highest index in control point array
     p   , input  ,  Degree of the curve
     m   , input  ,  Highest index in knot vector array
     S   , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_AllocCrvArrays
 (NL_CURVE  *cur,  // i/o: in : NURBS curve ptr before sizes set and internal arrays have been allocated
                   //      out: NURBs curve ptr after  sizes set and internal arrays allocated
  NL_INDEX   n,    // in : Highest index in control point array
  NL_DEGREE  p,    // in : Degree of the curve
  NL_INDEX   m,    // in : Highest index in knot vector array (m = n + p + 1)
  NL_STACKS *S )   // in : cur's stack
{
    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    /* Allocate memory */

    pol = N_AllocCPolygonAndArray( n, S );

    if( pol EQ NULL )
        return (1);

    knt = N_AllocKnotVectorAndArray( m, S );

    if( knt EQ NULL )
        return (1);

    /* Build curve structure */

    N_CrvFromCPolygonAndKnotVector( cur, pol, p, knt );

    /* Exit */

    return (0);
} /* end N_AllocCrvArrays */

/*******************************************************************//**
   DESCRIPTION:

     This utility routine checks if memory is needed to store a curve.
     If the curve is  initialized to the NULL curve  (via N_CrvInitArrays()),
     memory is allocated. If not, the routine checkes if enough memory 
     is available. A typical calling example is:

       NL_CURVE   cur;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STRING  rname;
       NL_STACKS  S;
       ...
       (get n, p, m and rname);
       ...
       N_CrvInitArrays(&cur);
       N_CrvSizeArrays(&cur,n,p,m,rname,&S);

     IT IS  ASSUMED THAT MEMORY TO STORE THE NL_CURVE STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     cur   , in/out ,  NURBS curve to be created
     n,p,m , input  ,  n=highest Cpt index, p=degree, m=highest knot index
     rname , input  ,  Routine name
     S     , input  ,  cur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CrvSizeArrays
 ( NL_CURVE  * cur,    /* in : NURBS curve to be created    */
   NL_INDEX    n,      /* in : highest control point index  */
   NL_DEGREE   p,      /* in : degree                       */
   NL_INDEX    m,      /* in : highest knot index           */
   NL_STRING   rname,  /* in : name of calling routine      */
   NL_STACKS * S )     /* in : new object stack             */
{
    NL_FLAG error;

    /* See if memory is needed */

    if( N_CrvAreArraysNULL( cur ) )
    {
        error = N_AllocCrvArrays( cur, n, p, m, S );

        if( error EQ 1 )
            return (1);
    }
    else
    {
        error = N_CrvIsSized( cur, n, m, rname );

        if( error EQ 1 )
            return (1);

        N_CrvSetSizeIndices( cur, n, p, m );
    }

    /* Exit */

    return (0);
} /* end N_CrvSizeArrays */

/*******************************************************************//**


   DESCRIPTION:

     This  routine generates a  curve object  defined in  power basis 
     form. That is, given the  vector coefficients of the power basis
     curve along with the  interval over which the  curve is defined,
     this  routine  creates a  NL_CURVE object that represents the power 
     basis curve. A typical calling example is as follows:

       NL_CURVE      cpl;
       NL_CPOINT     *aw;
       NL_INDEX      n;
       NL_PARAMETER  a, b;
       NL_STACKS     S;
       ...
       (get aw array, n, a, and b);
       ...
       N_CreateCrvFromPowerBasis(&cpl,aw,n,a,b,&S);

     MEMORY FOR THE NL_CURVE  STRUCTURE cpl IS ALLOCATED  IN THE CALLING 
     ROUTINE. ROUTINES HANDLING  CURVES IN POWER BASIS FORM ARE FOUND
     WITH THE PREFIX cpl, i.e. N_cpl***.c


   ACCESS:
   
     cpl , in/out ,  NURBS curve
     aw  , input  ,  Vector coefficients
     n   , input  ,  Highest index in aw
     a,b , input  ,  Parameter bounds
     S   , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCrvFromPowerBasis( NL_CURVE *cpl, NL_CPOINT *aw, NL_INDEX n, NL_PARAMETER a, NL_PARAMETER b, NL_STACKS *S )
{
    NL_REAL *U;

    NL_DEGREE p;

    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    /* Allocate memory */

    pol = N_AllocCPolygon( S );

    if( pol EQ NULL )
        return (1);

    knt = N_AllocKnotVector( S );

    if( knt EQ NULL )
        return (1);

    U = N_AllocReal1dArray( 1, S );

    if( U EQ NULL )
        return (1);

    U[0] = a;
    U[1] = b;
    p = (NL_DEGREE)n;

    /* Make pointer assignments */

    N_CPolygonFromCPts( pol, aw, n );
    N_KnotVectorFromRealArray( knt, U, 1 );
    N_CrvFromCPolygonAndKnotVector( cpl, pol, p, knt );

    /* Exit */

    return (0);
} /* end N_CreateCrvFromPowerBasis */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine detaches the  polygon and the  knot vector
     objects from a given curve object. A typical calling example is:
   
       NL_CURVE       cur;
       NL_CPOLYGON    *pol;
       NL_DEGREE      p;
       NL_KNOTVECTOR  *knt;
       ...
       N_CrvDetachPolygonKnot(&cur,&pol,&p,&knt);


   ACCESS:
   
     cur , input  ,  NURBS curve
     pol , output ,  Control polygon
     p   , output ,  Degree
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvDetachPolygonKnot( NL_CURVE *cur, NL_CPOLYGON ** pol, NL_DEGREE *p, NL_KNOTVECTOR ** knt )
{
    *pol = cur->pol;
    *p = cur->p;
    *knt = cur->knt;
} /* end N_CrvDetachPolygonKnot */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine breaks a curve object down to its components,
     i.e. indexes,  control  point  array  and  knot vector. A  typical 
     calling example is:

       NL_CURVE   cur;
       NL_INDEX   n, m;
       NL_CPOINT  *Pw;
       NL_DEGREE  p;
       NL_REAL    *U;
       ...
       N_CrvGetCPtsDegreeAndKnots(&cur,&n,&Pw,&p,&m,&U);


   ACCESS:
   
     cur , input  ,  NURBS curve
     n   , output ,  Highest index in Pw
     Pw  , output ,  Control points
     p   , output ,  Degree
     m   , output ,  Highest index in U
     U   , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_CrvGetCPtsDegreeAndKnots */
NL_VOID N_CrvGetCPtsDegreeAndKnots
 ( NL_CURVE    *cur,    /* in : target curve                                                    */
   NL_INDEX    *n,      /* in : Highest index in control point array                            */
   NL_CPOINT ** Pw,     /* out: array of control points                                         */
   NL_DEGREE   *p,      /* out: curve degree                                                    */
   NL_INDEX    *m,      /* out: Highest index in KnotArray                                      */
   NL_REAL   ** U )     /* out: array of knots (multiple knots are represented multiple times ) */
{
    /* when curve is not initialized to the NULL curve */
    if( N_CrvAreArraysNULL( cur ) == NL_FALSE )
    {
        *n  = cur->pol->n;
        *Pw = cur->pol->Pw;
        *p  = cur->p;
        *m  = cur->knt->m;
        *U  = cur->knt->U;
    }
    else
    {
        *n  = -1;
        *Pw = NULL;
        *p  = -1;
        *m  = -1;
        *U  = NULL;
    }
} /* end             */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the highest indexes of a curve 
     definition. A typical calling example is:

       NL_CURVE  cur;
       NL_INDEX  n, m;
       ...
       N_CrvGetArraySizes(&cur,&n,&m);


   ACCESS:
   
     cur , input  ,  NURBS curve
     n   , output ,  Highest index in Pw
     m   , output ,  Highest index in U


   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_CrvGetArraySizes( NL_CURVE *cur, NL_INDEX *n, NL_INDEX *m )
{
    *n = cur->pol->n;
    *m = cur->knt->m;
} /* end N_CrvGetArraySizes */

/**********************************************************************/
/* N_CrvGetConicData                                                */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine computes the high index of control points and knots
     in a circle, based on start,end angles (edited for tolerance near 360 etc)
     and  conic type(NL_QUADRATIC, NL_QUARTIC, NL_QUINTIC)
     It is called by all routines requiring a consistent set of values.
       ctp      IN       conic type    NL_QUADRATIC, NL_QUARTIC, NL_QUINTIC
       as       IN       start angles for arc in degrees
       ae       IN/NL_OUT   end angles for arc(input, then adjusted for tolerance) in degrees
       n      , NL_OUT  ,   Highest index in control point array
       p      , NL_OUT  ,   Degree of the curve
       m      , NL_OUT  ,   Highest index in knot vector array

   ***********************************************************************/

NL_VOID N_CrvGetConicData( NL_FLAG ctp, NL_REAL as, NL_REAL *ae, NL_INDEX *n, NL_DEGREE *p, NL_INDEX *m )
{
    NL_REAL theta;
    NL_REAL AngleTolDeg = 1.0e-6; /* Dont make this too small */

    /* Get number of arcs */
    while( *ae LT( as + AngleTolDeg ) )
        *ae += 360.0;

    while( (*ae - as)GT 360.0 - AngleTolDeg )
        *ae -= 360.0;

    if( (*ae - as)LT AngleTolDeg )
        *ae = as + 360.0;

    theta = *ae - as;

    switch( ctp )
    {
        case NL_QUADRATIC:
            *p = 2;

            if( theta LT 91.0 )
                *n = 2;

            else if( theta LT 181.0 )
                *n = 4;

            else if( theta LT 271.0 )
                *n = 6;

            else
                *n = 8;

            break;

        case NL_QUARTIC:
            *p = 4;

            if( theta LT 121.0 )
                *n = 4;

            else if( theta LT 241.0 )
                *n = 8;

            else
                *n = 12;

            break;

        case NL_QUINTIC:
            *p = 5;
            *n = 5;
            break;
    }
    *m = *n + *p + 1;
    return;
}

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control polygon information from a 
     curve object. A typical calling example is:

       NL_CURVE   cur;
       NL_INDEX   n;
       NL_CPOINT  *Pw;
       ...
       N_CrvGetCPts(&cur,&n,&Pw);


   ACCESS:
   
     cur , input  ,  NURBS curve
     n   , output ,  Highest index in Pw
     Pw  , output ,  Control points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetCPts
 ( NL_CURVE   * cur,   /* target curve                        */ 
   NL_INDEX   * n,     /* higest index in control point array */
   NL_CPOINT ** Pw )   /* control point array                 */ 
{
    *n = cur->pol->n;
    *Pw = cur->pol->Pw;
} /* end N_CrvGetCPts */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control points and knots from a curve 
     object. A typical calling example is:

       NL_CURVE   cur;
       NL_CPOINT  *Pw;
       NL_REAL    *U;
       ...
       N_CrvGetCPtsAndKnots(&cur,&Pw,&U);


   ACCESS:
   
     cur , input  ,  NURBS curve
     Pw  , output ,  Control point array
     U   , output ,  Knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetCPtsAndKnots
 ( NL_CURVE   * cur,  /* target curve        */
   NL_CPOINT ** Pw,   /* Control Point array */ 
   NL_REAL   ** U )   /* Knot Vector array   */ 
{
    *Pw = cur->pol->Pw;
    *U  = cur->knt->U;
} /* end N_CrvGetCPtsAndKnots */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control points, knot vector and knots 
     from a curve object. A typical calling example is:

       NL_CURVE       cur;
       NL_CPOINT      *Pw;
       NL_KNOTVECTOR  *knt;
       NL_REAL        *U;
       ...
       N_CrvGetCPtsKnotVectorAndKnots(&cur,&Pw,&knt,&U);


   ACCESS:
   
     cur , input  ,  NURBS curve
     Pw  , output ,  Control point array
     knt , output ,  Knot vector object
     U   , output ,  Knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetCPtsKnotVectorAndKnots( NL_CURVE *cur, NL_CPOINT ** Pw, NL_KNOTVECTOR ** knt, NL_REAL ** U )
{
    *Pw = cur->pol->Pw;
    *knt = cur->knt;
    *U = cur->knt->U;
} /* end N_CrvGetCPtsKnotVectorAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets control point and knot vector pointers
     of a curve object. A typical calling example is:

       NL_CURVE   cur;
       NL_CPOINT  *Pw;
       NL_REAL    *U;
       ...
       N_CrvSetCPtsAndKnots(&cur,Pw,U);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     Pw  , input  ,  Control point array
     U   , input  ,  Knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvSetCPtsAndKnots( NL_CURVE *cur, NL_CPOINT *Pw, NL_REAL *U )
{
    cur->pol->Pw = Pw;
    cur->knt->U = U;
} /* end N_CrvSetCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vector info from curve object. A
     typical calling example is:

       NL_CURVE  cur;
       NL_INDEX  m;
       NL_REAL   *U;
       ...
       N_CrvGetKnots(&cur,&m,&U);


   ACCESS:
   
     cur , input  ,  NURBS curve
     m   , output ,  Highest index in U
     U   , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetKnots( NL_CURVE *cur, NL_INDEX *m, NL_REAL ** U )
{
    *m = cur->knt->m;
    *U = cur->knt->U;
} /* end N_CrvGetKnots */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the parameter bounds from a curve object. 
     A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  ul, ur;
       ...
       N_CrvGetParamBounds(&cur,&ul,&ur);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     ul,ur , output ,  Parameter bounds


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetParamBounds
 (NL_CURVE const *cur,  /* in : tgt curve */
  NL_PARAMETER *ul,     /* out: cur's NaturalInterval min value */
  NL_PARAMETER *ur )    /* out: cur's NaturalInterval max value */
{
    *ul = cur->knt->U[0];
    *ur = cur->knt->U[cur->knt->m];
} /* end N_CrvGetParamBounds */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vector object from curve object. A
     typical calling example is:

       NL_CURVE       cur;
       NL_KNOTVECTOR  *knt;
       ...
       N_CrvGetKnotVector(&cur,&knt);


   ACCESS:
   
     cur , input  ,  NURBS curve
     knt , output ,  Knot vector object


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetKnotVector( NL_CURVE *cur, NL_KNOTVECTOR ** knt )
{
    *knt = cur->knt;
} /* end N_CrvGetKnotVector */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the degree of a curve. A typical calling 
     example is:

       NL_CURVE   cur;
       NL_DEGREE  p;
       ...
       N_CrvGetDegree(&cur,&p);

   ACCESS:
   
     cur , input  ,  NURBS curve
     p   , output ,  Degree

   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetDegree( NL_CURVE *cur, NL_DEGREE *p )
{
    *p = cur->p;
} /* end N_CrvGetDegree */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine compacts  control point  and  knot  vector 
     arrays. That is, given a curve with control point and knot vector
     arrays larger than required. This routine redefines  these arrays
     to the appropriate sizes which makes curve definition more memory
     efficient. A typical calling example is:

       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define curve);
       ...
       N_CrvCompress(&cur,&SG);

     SG MUST BE cur's STACK, I.E. ALL  MEMORY ALLOCATED FOR  cur, MUST 
     BE ON SG.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     SG  , input  ,  cur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvCompress( NL_CURVE *cur, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;
    NL_INDEX i, n, m;
    NL_REAL *U, *V;
    NL_CPOINT *Pw, *Qw;

    /* Get curve data */
    N_CrvGetArraySizes( cur, &n, &m );
    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Allocate memory for new control point and knot vector arrays */
    Qw = N_AllocCPt1dArray( n, SG );

    if( Qw EQ NULL )
        NL_QUIT;

    V = N_AllocReal1dArray( m, SG );

    if( V EQ NULL )
        NL_QUIT;

    /* Copy control points and knots */
    for ( i = 0; i <= n; i++ )
    {
        N_CopyCPt( Pw[i], &Qw[i] );
    }

    for ( i = 0; i <= m; i++ )
        V[i] = U[i];

    /* Redefine curve and kill old memory */
    N_CrvSetCPtsAndKnots( cur, Qw, V );

    N_FreeCPt1dArray( Pw, SG );
    N_FreeReal1dArray( U, SG );

    /* Exit */
    EXIT:

    return (error);
} /* end N_CrvCompress */


/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine expands   control point  and  knot  vector 
     arrays. That is, given a curve with control point and knot vector
     arrays and a set of target sizes, this routine replace the existing
     curve arrays with larger (or smaller ones) after copying existing
     values. A typical calling example is:

       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (define curve);
       ...
       N_CrvExpand(&cur, new_n, new_m, &SG);

     SG MUST BE cur's STACK, I.E. ALL  MEMORY ALLOCATED FOR  cur, MUST 
     BE ON SG.


   ACCESS:
   
     cur   , in/out ,  NURBS curve
     new_n , input  ,  Desired Highest index in Pw
     new_m , input  ,  Desired Highest index in U
     SG    , input  ,  cur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvExpand( NL_CURVE *cur, NL_INDEX new_n, NL_INDEX new_m, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;
    NL_INDEX i, n, m, min_n, min_m;
    NL_REAL *U, *V;
    NL_DEGREE p ;
    NL_CPOINT *Pw, *Qw;

    /* Get current curve data */
    N_CrvGetDegree( cur, &p ) ;
    N_CrvGetArraySizes( cur, &n, &m );
    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Save min sizes for upcoming copy */
    min_n = NL_MIN(n, new_n) ;
    min_m = NL_MIN(m, new_m) ;

    /* Allocate memory for new control point and knot vector arrays */
    Qw = N_AllocCPt1dArray( new_n, SG );

    if( Qw EQ NULL )
        NL_QUIT;

    V = N_AllocReal1dArray( new_m, SG );

    if( V EQ NULL )
        NL_QUIT;

    /* Copy control points and knots */
    for ( i = 0; i <= min_n; i++ )
    {
        N_CopyCPt( Pw[i], &Qw[i] );
    }

    for ( i = 0; i <= min_m; i++ )
        V[i] = U[i];

    /* Redefine curve and kill old memory */
    N_CrvSetSizeIndices( cur, new_n, p, new_m) ;
    N_CrvSetCPtsAndKnots( cur, Qw, V );

    N_FreeCPt1dArray( Pw, SG );
    N_FreeReal1dArray( U, SG );

    /* Exit */
    EXIT:

    return (error);
} /* end N_CrvExpand */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine clamps a given parameter with respect to the
     knot span. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       ...
       N_CrvClampKnot(&cur,&u);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , in/out ,  Parameter to be clamped


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvClampKnot( NL_CURVE *cur, NL_PARAMETER *u )
{
    NL_INDEX m;

    NL_REAL *U;

    N_CrvGetKnots( cur, &m, &U );

    if( *u LT U[0] )
        *u = U[0];

    if( *u GT U[m] )
        *u = U[m];
} /* end N_CrvClampKnot */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets parameters to complete curve definition.
     A typical calling example is:

       NL_CURVE   cur;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       ...
       N_CrvSetSizeIndices(&cur,n,p,m);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     n   , input  ,  Highest index in polygon array
     p   , input  ,  Degree
     m   , input  ,  Highest index in knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvSetSizeIndices( NL_CURVE *cur, NL_INDEX n, NL_DEGREE p, NL_INDEX m )
{
    cur->pol->n = n;
    cur->p = p;
    cur->knt->m = m;
} /* end N_CrvSetSizeIndices */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine initializes a curve structure by setting
     pointers to NULL and the degree to -1. It is used to check if
     memory allocation is needed, i.e. if the curve is initialized 
     to the NULL  curve, memory is  allocated to  hold the polygon 
     and knot vector  objects. Otherwise it is assumed that memory
     has already been allocated. A typical calling example is:

       NL_CURVE  cur;
       ...
       N_CrvInitArrays(&cur);
     

   ACCESS:
   
     cur , in/out ,  NURBS curve


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvInitArrays( NL_CURVE *cur )
{
    cur->pol = NULL;
    cur->p = -1;
    cur->knt = NULL;
} /* end N_CrvInitArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the curve is initialized to the
     NULL  curve.  If  yes, NL_TRUE  is  returned. Otherwise, NL_FALSE is
     returned.  This routine is  used to check if memory allocation
     is needed. A typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_CrvAreArraysNULL(&cur) )  --> allocate memory;
     

   ACCESS:
   
     cur , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE:  Curve is initialized to NULL (need memory)
     NL_FALSE: Curve is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_CrvAreArraysNULL( NL_CURVE *cur )
{
    NL_DEGREE p;

    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    /* Get local notation - get cur pol, p, and knt values */
    N_CrvDetachPolygonKnot( cur, &pol, &p, &knt );

    /* Check initialization */
    if( pol EQ NULL OR p EQ - 1 OR knt EQ NULL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CrvAreArraysNULL */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the curve is rational or not. A
     typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_IsCrvRat(&cur) )  --> handle rational case;
     

   ACCESS:
   
     cur , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE:  Curve is rational
     NL_FALSE: Curve is NOT rational

   ***********************************************************************/

NL_BOOLEAN N_IsCrvRat( NL_CURVE *cur )
{
    NL_INDEX n;

    NL_REAL w;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check rationality */

    N_CPtGetW( Pw[0], &w );

    if( w NEQ NL_NOW )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_IsCrvRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a NURBS curve is closed or not.
     A typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_CrvIsClosed(&cur) )  --> handle closed curve;
     

   ACCESS:
   
     cur , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE : Curve is closed
     NL_FALSE: Curve is NOT closed

   ***********************************************************************/

NL_BOOLEAN N_CrvIsClosed( NL_CURVE *cur )
{
    NL_INDEX n;

    NL_REAL d;

    NL_POINT P0, Pn;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check closeness */

    N_CPtToPtEuclid( Pw[0], &P0 );
    N_CPtToPtEuclid( Pw[n], &Pn );
    N_DistPtPt( P0, Pn, &d );

    if( d LT NL_MTOL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CrvIsClosed */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine checks if a NURBS curve is degenerate to a
     single point. A typical calling example is:
 
       NL_CURVE  cur;
       ...
       if( N_CrvIsDegen(&cur) )  --> handle point;
     
 
   ACCESS:
   
     cur  , input ,  NURBS curve
 
 
   RETURN CODES:
 
     NL_TRUE : Curve is a point
     NL_FALSE: Curve is NOT a point
 
   ***********************************************************************/

NL_BOOLEAN N_CrvIsDegen( NL_CURVE *cur )
{
    NL_FLAG dst = NL_YES;

    NL_INDEX i, n;

    NL_REAL d, fac;

    NL_POINT Q, M;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check if curve is a point */

    fac = 1.0 / ((NL_REAL)n + (NL_REAL)1);
    N_CopyPt( NL_ZERO, &M );

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &Q );
        N_Sum2Pts( M, Q, &M );
    }
    N_ScalePt( fac, M, &M );

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &Q );
        N_DistPtPt( Q, M, &d );

        if( d GT NL_MTOL )
        {
            dst = NL_NO;
            break;
        }
    }

    if( dst EQ NL_YES )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_CrvIsDegen */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a NURBS curve is a line or not.
     A typical calling example is:

       NL_REAL   tol;
       NL_CURVE  cur;
       ...
       (get tol)
       ...
       if( N_CrvIsLine(&cur,tol) )  --> handle line;
     

   ACCESS:
   
     cur , input ,  NURBS curve
     tol , input ,  Tolerance to measure straightness (by measuring
                    distance in the space in which the curve lies)


   RETURN CODES:

     NL_TRUE : Curve is a line
     NL_FALSE: Curve is NOT a line

   ***********************************************************************/

NL_BOOLEAN N_CrvIsLine( NL_CURVE *cur, NL_REAL tol )
{
    NL_FLAG dst = NL_YES, error = NL_NO;

    NL_INDEX i, n;

    NL_REAL d;

    NL_POINT Q, A, B;

    NL_CPOINT *Pw;

    NL_VECTOR line;

    NL_POINT Aprime, Bprime;

    NL_REAL length;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check straightness */

    N_CPtToPtEuclid( Pw[0], &A );
    N_CPtToPtEuclid( Pw[n], &B );

    N_DistPtPt( A, B, &d );

    if( d LE tol )
    {
        for ( i = 1; i < n; i++ )
        {
            N_CPtToPtEuclid( Pw[i], &Q );
            N_DistPtPt( A, Q, &d );

            if( d GT tol )
                return NL_FALSE;
        }
    }

    /* extend A and B out a little which in effect grows the line between start and end point
       this ensures that even small hooks at the end will project onto the line
       so we can get a valid distance from the line for every point */
    N_VectorDir( A, B, &line );
    N_VectorMagnitude( line, &length );
    N_VectorNormalizeRef( &line );
    N_VectorPtAlongVector( A, length + tol, line, &Bprime );
    N_VectorReverseInPlace( &line );
    N_VectorPtAlongVector( B, length + tol, line, &Aprime );

    for ( i = 1; i < n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &Q );

        error = N_DistPerpPtLineSeg( Q, Aprime, Bprime, &d );

        if( error EQ NL_YES )
        {
            dst = NL_NO;
            break;
        }

        if( d LT 0.0 OR d GT tol )
        {
            dst = NL_NO;
            break;
        }
    }

    if( dst EQ NL_YES )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_CrvIsLine */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine checks if two curves are coincident or not.
     A typical calling example is:

       NL_REAL    tol;
       NL_CURVE   curP, curQ;
       NL_STACKS  SG;
       ...
       (get curP and curQ, choose tol);
       ...
       if( N_CrvsAreCoincident(&curP,&curQ,tol,&SG) ) --> handle coincident case;
     

   ACCESS:
   
     curP , input ,  NURBS curve
     curQ , input ,  NURBS curve
     tol  , input ,  Tolerance for coincidence checking


   RETURN CODES:

     NL_TRUE : Curves are coincident
     NL_FALSE: Curves are NOT coincident

   ***********************************************************************/

NL_BOOLEAN N_CrvsAreCoincident( NL_CURVE *curP, NL_CURVE *curQ, NL_REAL tol, NL_STACKS *SG )
{
    NL_FLAG error;

    NL_INDEX i, n;

    NL_REAL dw, dwmax;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE ** cur;

    /* Get array of curve pointers */

    cur = N_AllocArrayCrvPtrs( 1, SG );

    if( cur EQ NULL )
        return NL_FALSE;

    cur[0] = curP;
    cur[1] = curQ;

    /* Make input curves compatible */

    error = N_CrvsMakeCompatible( cur, 1, SG );

    if( error EQ NL_YES )
        return NL_FALSE;

    /* Check coincidence */

    N_CrvGetCPts( cur[0], &n, &Pw );
    N_CrvGetCPts( cur[1], &n, &Qw );

    dwmax = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        N_DistCptCptHomo( Pw[i], Qw[i], &dw );

        if( dw GT dwmax )
            dwmax = dw;
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
} /* end N_CrvsAreCoincident */


/*******************************************************************//**


   DESCRIPTION:

     This utility  routine checks if two curves are equal or not.
     Two curves are equal when they have the same degree, number of knots,
     weights, and control points and all those values are within
     tolerance of one another.

     A typical calling example is:

       NL_REAL    tol3d, tol1d;
       NL_CURVE   curP, curQ;
       ...
       (get curP and curQ, choose tol);
       ...
       if( N_CrvsAreEqual(&curP,&curQ,tol) ) --> handle equal case;
     

   ACCESS:
   
     curP , input ,  NURBS curve
     curQ , input ,  NURBS curve
     tol3d, input ,  Tolerance for coincident control point checking
     tol1d, input ,  Tolerance for coincident knot checking


   RETURN CODES:

     NL_TRUE : Curves are equal
     NL_FALSE: Curves are NOT equal

   ***********************************************************************/

NL_BOOLEAN N_CrvsAreEqual( NL_CURVE *curP, NL_CURVE *curQ, NL_REAL tol3d, NL_REAL tol1d )
{
    NL_INTEGER ii ;

    NL_INDEX   Pn,  Qn ;  /* Highest index in control point array */ 
                          
    NL_CPOINT *Pw, *Qw ;  /* array of control points */
                          
    NL_DEGREE  Pp,  Qp ;  /* curve degree */
                          
    NL_INDEX   Pm,  Qm ;  /* Highest index in KnotArray */
                          
    NL_REAL   *PU, *QU ;  /* array of knots (multiple knots are represented multiple times ) */

    NL_BOOLEAN bRtn ;

    NL_REAL d;

    NL_CPOINT Rw;

    
    /* get curve data */
    N_CrvGetCPtsDegreeAndKnots( curP, &Pn, &Pw, &Pp, &Pm, &PU) ;
    N_CrvGetCPtsDegreeAndKnots( curQ, &Qn, &Qw, &Qp, &Qm, &QU) ;

    /* check counts */
    bRtn = (   Pp == Qp       /* same degree */
            && Pn == Qn       /* same control point count */
            && Pm == Qm ) ;   /* same knot count */

    /* check control points */
    if(bRtn)
      {
        for(ii=0;ii<=Pn && bRtn ;ii++)
          {
            /* distance between control points */
            N_Diff2CPts( Pw[ii], Qw[ii], &Rw );
            N_CPtMagnitude( Rw, &d );

            /* check for equivalent locations */
            bRtn &= (d <= tol3d) ;

          } /* end iter every control point */
      } /* end control points check */

    /* check knots */
    if(bRtn)
      {
        /* multiple knots are represented multiple times */
        for(ii=0;ii<=Pm && bRtn ;ii++)
          {
            /* check for equivalent locations */
            bRtn &= (fabs(PU[ii] - QU[ii]) <= tol1d) ;

          } /* end iter every control point */
      } /* end knots check */

    /* all done */
    return bRtn;

} /* end N_CrvsAreEqual */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a set of curves  are compatible, ie
     it checks if all  curves are rational or non-rational, 2-D or 3-D, 
     and if they are defined over the same knot  vector. Two  knots are 
     considered the same if  their difference is below NL_PTOL, defined in
     "globals.h". A typical calling example is:

       NL_CURVE  **cur;
       NL_INDEX  k;
       ...
       (define array of cur);
       ...
       if( N_CrvsAreCombatible(cur,k) ) --> handle compatible case;


   ACCESS:
   
     cur , in/out ,  Array of NURBS curves
     k   , input  ,  Highest index in array


   RETURN CODES:

     NL_TRUE  :  Curves are compatible
     NL_FALSE :  Curves are NOT compatible

   ***********************************************************************/

NL_BOOLEAN N_CrvsAreCombatible( NL_CURVE ** cur, NL_INDEX k )
{
    NL_FLAG allra, allnra, all2d, all3d, allnm, allknt;

    NL_INDEX i, j, n, m, n0, m0;

    NL_REAL *U, *U0;

    /* All must be rational or non rational */

    allra = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( NOT N_IsCrvRat( cur[i] ) )
        {
            allra = NL_FALSE;
            break;
        }
    }

    allnra = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( N_IsCrvRat( cur[i] ) )
        {
            allnra = NL_FALSE;
            break;
        }
    }

    if( allra EQ NL_FALSE AND allnra EQ NL_FALSE )
        return NL_FALSE;

    /* All must be 2-D or 3-D */

    all2d = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( N_CrvIs3d( cur[i] ) )
        {
            all2d = NL_FALSE;
            break;
        }
    }

    all3d = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( NOT N_CrvIs3d( cur[i] ) )
        {
            all3d = NL_FALSE;
            break;
        }
    }

    if( all2d EQ NL_FALSE AND all3d EQ NL_FALSE )
        return NL_FALSE;

    /* All must have the same indexes */

    N_CrvGetArraySizes( cur[0], &n0, &m0 );

    allnm = NL_TRUE;

    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetArraySizes( cur[i], &n, &m );

        if( n NEQ n0 OR m NEQ m0 )
        {
            allnm = NL_FALSE;
            break;
        }
    }

    if( allnm EQ NL_FALSE )
        return NL_FALSE;

    /* All must have the same knots */

    N_CrvGetKnots( cur[0], &m0, &U0 );

    allknt = NL_TRUE;

    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetKnots( cur[i], &m, &U );

        for ( j = 0; j <= m; j++ )
        {
            if( fabs( U[j] - U0[j] )GT NL_PTOL )
            {
                allknt = NL_FALSE;
                break;
            }
        }

        if( allknt EQ NL_FALSE )
            break;
    }

    if( allknt EQ NL_FALSE )
        return NL_FALSE;

    /* Exit */

    return NL_TRUE;
} /* end N_CrvsAreCombatible */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine copies a given curve into another curve.
     A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_STACKS  S;
       ...
       (define curP);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvCopy(&curP,&curQ,&S);

     If the curve is initialized to the empty curve (NULL), memory
     will be allocated  for curQ.  Otherwise,  it is  assumed that
     memory is already available.


   ACCESS:
   
     curP , input  ,  NURBS curve to be copied
     curQ , output ,  Copied curve 
     S    , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvCopy( NL_CURVE *curP, NL_CURVE *curQ, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvCopy");

    NL_FLAG error;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_CPOINT *Pw, *Qw;

    NL_REAL *UP, *UQ;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* See if memory is needed */

    error = N_CrvSizeArrays( curQ, n, p, m, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );

    /* Copy the curve */

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Pw[i], &Qw[i] );

    for ( i = 0; i <= m; i++ )
        UQ[i] = UP[i];

    /* Exit */

    return (0);
} /* end N_CrvCopy */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  extracts the numerator and the denominator
     from a NURBS curve. If the curve is non-rational, no denominator
     is returned. A typical calling example is:

       NL_CURVE   cur, num;
       NL_CFUN    den;
       NL_STACKS  S;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&num);
       N_CFuncInitArrays(&den);
       N_CrvGetNumAndDenom(&cur,&num,&den,&S);

     If  num and  den are  initialized to NULL, memory is  allocated. 
     Otherwise, it is assumed that memory is already available.


   ACCESS:
   
     cur , input  ,  NURBS curve
     num , output ,  Numerator of cur
     den , output ,  Denominator of cur (if rational)
     S   , input  ,  num's and den's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetNumAndDenom( NL_CURVE *cur, NL_CURVE *num, NL_CFUN *den, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetNumAndDenom");

    NL_FLAG error, rat = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *UC, *UN, *UD = NULL, *fu = NULL, xw, yw, zw, w;

    NL_CPOINT *Pw, *Nw;

    /* Get local notation and set rational flag */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &UC );

    if( N_IsCrvRat( cur ) )
        rat = NL_YES;

    /* See if memory is needed */

    error = N_CrvSizeArrays( num, n, p, m, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_CrvGetCPtsAndKnots( num, &Nw, &UN );

    if( rat EQ NL_YES )
    {
        error = N_CFuncSizeArrays( den, n, p, m, rname, S );

        if( error EQ NL_YES )
            return (1);

        N_CrvFuncCntrlValKnots( den, &fu, &UD );
    }

    /* Define output entities */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToWxWyWz( Pw[i], &xw, &yw, &zw, &w );
        N_CPtFromWxWyWz( xw, yw, zw, NL_NOW, &Nw[i] );

        if( rat EQ NL_YES )
            fu[i] = w;
    }

    for ( i = 0; i <= m; i++ )
    {
        UN[i] = UC[i];

        if( rat EQ NL_YES )
            UD[i] = UC[i];
    }

    /* Exit */

    return (0);
} /* end N_CrvGetNumAndDenom */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  extracts the  coordinate  functions wx(u), 
     wy(u), wz(u) and w(u) from a NURBS curve. If the  curve is 2-D, no 
     wz(u) is  returned. Similarly,  if it is  non-rational, no w(u) is 
     returned. A typical calling example is:

       NL_CURVE   cur;
       NL_CFUN    wx, wy, wz, w;
       NL_STACKS  S;
       ...
       (define cur);
       ...
       N_CFuncInitArrays(&wx);
       N_CFuncInitArrays(&wy);
       N_CFuncInitArrays(&wz);
       N_CFuncInitArrays(&w );
       N_CrvGetCoordFuncs(&cur,&wx,&wy,&wz,&w,&S);

     If wx, wy, wz and w are initialized to NULL, memory is  allocated. 
     Otherwise, it is assumed that memory is already available.


   ACCESS:
   
     cur , input  ,  NURBS curve
     wx  , output ,  X-coordinate function
     wy  , output ,  Y-coordinate function
     wz  , output ,  Z-coordinate function (if curve is 3-D)
     w   , output ,  W-coordinate function (if curve is rational)
     S   , input  ,  Stack of wx, wy, wz and w


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetCoordFuncs( NL_CURVE *cur, NL_CFUN *wx, NL_CFUN *wy, NL_CFUN *wz, NL_CFUN *w, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetCoordFuncs");

    NL_FLAG error, thd = NL_NO, rat = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *fx, *fy, *fz = NULL, *fw = NULL, *U, *UX, *UY, *UZ = NULL, *UW = NULL, a, b;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* See if memory is needed */

    error = N_CFuncSizeArrays( wx, n, p, m, rname, S );

    if( error EQ NL_YES )
        return (1);
    N_CrvFuncCntrlValKnots( wx, &fx, &UX );

    error = N_CFuncSizeArrays( wy, n, p, m, rname, S );

    if( error EQ NL_YES )
        return (1);
    N_CrvFuncCntrlValKnots( wy, &fy, &UY );

    if( N_CrvIs3d( cur ) )
    {
        error = N_CFuncSizeArrays( wz, n, p, m, rname, S );

        if( error EQ NL_YES )
            return (1);

        N_CrvFuncCntrlValKnots( wz, &fz, &UZ );

        thd = NL_YES;
    }

    if( N_IsCrvRat( cur ) )
    {
        error = N_CFuncSizeArrays( w, n, p, m, rname, S );

        if( error EQ NL_YES )
            return (1);

        N_CrvFuncCntrlValKnots( w, &fw, &UW );

        rat = NL_YES;
    }

    /* Define output entities */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToWxWyWz( Pw[i], &fx[i], &fy[i], &a, &b );

        if( thd EQ NL_YES )
            fz[i] = a;

        if( rat EQ NL_YES )
            fw[i] = b;
    }

    for ( i = 0; i <= m; i++ )
    {
        UX[i] = U[i];
        UY[i] = U[i];

        if( thd EQ NL_YES )
            UZ[i] = U[i];

        if( rat EQ NL_YES )
            UW[i] = U[i];
    }

    /* Exit */

    return (0);
} /* end N_CrvGetCoordFuncs */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes a  NURBS curve given its numerator and 
     denominator. If the curve is non-rational, the denominator is not
     used; it is assumed to be NULL. A typical calling example is:

       NL_CURVE   cur, num;
       NL_CFUN    den;
       NL_STACKS  S;
       ...
       (define num and den);
       ...
       N_CrvInitArrays(&cur);
       N_CreateCrvFromNumAndDenom(&num,&den,&cur,&S); (    RATIONAL)
       N_CreateCrvFromNumAndDenom(&num,NULL,&cur,&S); (NON-RATIONAL)

     If cur is initialized to NULL, memory is allocated. Otherwise, it 
     is assumed that  memory is already available. THE NUMERATOR NL_CURVE
     AND THE DENOMINATOR  NL_FUNCTION MUST BE  DEFINED CONSISTENTLY, I.E.
     THE CONTROL NL_POINT/VALUE  INDEXES, THE KNOTS AND THE DEGREES  MUST 
     BE THE SAME.


   ACCESS:
   
     num , input  ,  Numerator of cur
     den , input  ,  Denominator of cur:
                       != NULL: rational case
                        = NULL: non-rational case
     cur , output ,  NURBS curve
     S   , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCrvFromNumAndDenom( NL_CURVE *num, NL_CFUN *den, NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCrvFromNumAndDenom");

    NL_FLAG error = NL_NO;

    NL_INDEX i, nn, mn, nd, md;

    NL_DEGREE pn, pd;

    NL_REAL *UC, *UN, *UD, *fu = NULL, xw, yw, zw, w;

    NL_CPOINT *Pw, *Nw;

    /* Get local notation and check consistency */

    N_CrvGetCPtsDegreeAndKnots( num, &nn, &Nw, &pn, &mn, &UN );

    if( den NEQ NULL )
    {
        N_CFuncGetData( den, &nd, &fu, &pd, &md, &UD );

        if( nn NEQ nd OR mn NEQ md OR pn NEQ pd )
            NL_ERROR( NL_INP_ERR );
    }

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, nn, pn, mn, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &UC );

    /* Define curve */

    for ( i = 0; i <= nn; i++ )
    {
        N_CPtToWxWyWz( Nw[i], &xw, &yw, &zw, &w );

        if( den NEQ NULL )
            w = fu[i];
        else
            w = NL_NOW;

        N_CPtFromWxWyWz( xw, yw, zw, w, &Pw[i] );
    }

    for ( i = 0; i <= mn; i++ )
        UC[i] = UN[i];

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateCrvFromNumAndDenom */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  makes a  NURBS  curve  from its  coordinate
     functions  wx(u),  wy(u),  wz(u) and  w(u). If  the  curve  is 2-D, 
     wz(u)=NULL is assumed. Similarly, if it is non-rational, w(u)=NULL. 
     A typical calling example is:

       NL_CURVE   cur;
       NL_CFUN    wx, wy, wz, w;
       NL_STACKS  S;
       ...
       (define wx, wy, wz and w);
       ...
       N_CrvInitArrays(&cur);
       N_CreateCrvFromCoordFuncs(&wx,&wy,&wz ,&w  ,&cur,&S); (    RATIONAL 3-D)
       N_CreateCrvFromCoordFuncs(&wx,&wy,NULL,&w  ,&cur,&S); (    RATIONAL 2-D)
       N_CreateCrvFromCoordFuncs(&wx,&wy,&wz ,NULL,&cur,&S); (NON-RATIONAL 3-D)
       N_CreateCrvFromCoordFuncs(&wx,&wy,NULL,NULL,&cur,&S); (NON-RATIONAL 2-D)

     If cur is  initialized to NULL, memory is  allocated. Otherwise, it 
     is  assumed  that  memory  is  already  available.  THE  COORDINATE 
     FUNCTIONS MUST BE FULLY COMPATIBLE, I.E. THE CONTROL VALUE INDEXES, 
     THE KNOTS AND THE DEGREES MUST BE THE SAME.


   ACCESS:
   
     wx  , input  ,  X-coordinate function
     wy  , input  ,  Y-coordinate function
     wz  , input  ,  Z-coordinate function:
                       != NULL: 3-D
                        = NULL: 2-D
     w   , input  ,  W-coordinate function:
                       != NULL: rational
                        = NULL: non-rational
     cur , output ,  NURBS curve
     S   , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCrvFromCoordFuncs( NL_CFUN *wx, NL_CFUN *wy, NL_CFUN *wz, NL_CFUN *w, NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCrvFromCoordFuncs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, nx, mx, ny, my, nz, mz, nw, mw;

    NL_DEGREE px, py, pz, pw;

    NL_REAL *U, *UX, *UY, *UZ, *UW, *fx, *fy, *fz = NULL, *fw = NULL, a, b, c, d;

    NL_CPOINT *Pw;

    /* Get local notation and check consistency */

    N_CFuncGetData( wx, &nx, &fx, &px, &mx, &UX );
    N_CFuncGetData( wy, &ny, &fy, &py, &my, &UY );

    if( nx NEQ ny OR px NEQ py OR mx NEQ my )
        NL_ERROR( NL_INP_ERR );

    if( wz NEQ NULL )
    {
        N_CFuncGetData( wz, &nz, &fz, &pz, &mz, &UZ );

        if( nx NEQ nz OR px NEQ pz OR mx NEQ mz )
            NL_ERROR( NL_INP_ERR );
    }

    if( w NEQ NULL )
    {
        N_CFuncGetData( w, &nw, &fw, &pw, &mw, &UW );

        if( nx NEQ nw OR px NEQ pw OR mx NEQ mw )
            NL_ERROR( NL_INP_ERR );
    }

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, nx, px, mx, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Define curve */

    for ( i = 0; i <= nx; i++ )
    {
        a = fx[i];
        b = fy[i];

        if( wz NEQ NULL )
            c = fz[i];
        else
            c = NL_NOZ;

        if( w NEQ NULL )
            d = fw[i];
        else
            d = NL_NOW;

        N_CPtFromWxWyWz( a, b, c, d, &Pw[i] );
    }

    for ( i = 0; i <= mx; i++ )
        U[i] = UX[i];

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateCrvFromCoordFuncs */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine saves curve definition data in  a file. 
     The data file is arranged as follows:

           n             --> highest index in control point array
           p             --> degree
           rat           --> rationality of the curve (0-no,1-yes)
           dim           --> dimension of the curve (2 or 3)
           x0 y0 (z0 w0) -->
           x1 y1 (z1 w1) -->
           .             --> 
           .             --> xy(zw) components of control points 
           .             -->
           xn yn (zn wn) -->
           u0            -->
           u1            -->
           .             -->
           .             --> knots
           .             -->
           um            -->

     The data file is named as specified in  the argument list. A
     typical calling example is:

       NL_CURVE   cur;
       TCHAR* fname;
       ...
       (get file name fname);
       ...
       N_CrvWriteToFile(&cur,fname);


   ACCESS:
   
     cur   , input  ,  NURBS curve to be saved
     fname , output ,  Name of the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvWriteToFile( NL_CURVE *cur, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvWriteToFile");

    NL_FLAG error = NL_NO;

    FILE *fptr;

    /* Open file */
    fptr = N_FileOpen( fname, _T("w"));

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Call file pointer counterpart */

    error = N_CrvWriteToFilePtr( cur, fptr );

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_CrvWriteToFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine saves the definition of an array of curves 
     in a file. The data file is arranged as follows:

           r                 --> highest index of curve array  
           n_0               --> highest index in Pw(0)
           p_0               --> degree of curve(0)
           rat_0             --> rationality of the curve(0)
           dim_0             --> dimension of the curve(0)
           <x0 y0 (z0 w0)>_0 
           ...               --> coordinates of control points
           <xn yn (zn wn)>_0  
           <u0>_0         
           ...               --> knots
           <um>_0
           .
           .
           .
           n_r               --> highest index in Pw(r)
           p_r               --> degree of curve(r)
           rat_r             --> rationality of the curve(r)
           dim_r             --> dimension of the curve(r)
           <x0 y0 (z0 w0)>_r 
           ...               --> coordinates of control points
           <xn yn (zn wn)>_r  
           <u0>_r         
           ...               --> knots
           <um>_r

     A typical calling example is:

       NL_INDEX   r;
       NL_CURVE   **cua;
       TCHAR*  fname;
       ...
       (get curves cua);
       ...
       N_CrvArrayWriteToFile(cua,r,fname);


   ACCESS:
   
     cua   , input  ,  Array of NURBS curves (array of pointers)
     r     , input  ,  Highest index in cua
     fname , output ,  Name of the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvArrayWriteToFile( NL_CURVE ** cua, NL_INDEX r, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvArrayWriteToFile");

    NL_FLAG dim, rat, type, error = NL_NO;

    NL_INDEX i, k, n, m;

    NL_DEGREE p;

    NL_CPOINT *Pw;

    NL_REAL *U, wx, wy, wz, w;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Save all curves */

    N_FPRINTF( fptr, _T("%ld\n"), r );

    for ( k = 0; k <= r; k++ )
    {
        N_CrvGetCPtsDegreeAndKnots( cua[k], &n, &Pw, &p, &m, &U );

        if( N_IsCrvRat( cua[k] ) )
            rat = 1;
        else
            rat = 0;

        if( N_CrvIs3d( cua[k] ) )
            dim = 3;
        else
            dim = 2;

        if( rat EQ 0 )
        {
            if( dim EQ 2 )
                type = 1;
            else
                type = 2;
        }

        else if( dim EQ 2 )
            type = 3;

        else
            type = 4;

        N_FPRINTF( fptr, _T("%ld\n"), n );
        N_FPRINTF( fptr, _T("%hd\n"), p );
        N_FPRINTF( fptr, _T("%hd\n"), rat );
        N_FPRINTF( fptr, _T("%hd\n"), dim );

        switch( type )
        {
            case 1: /* 2-D non-rational */
                for ( i = 0; i <= n; i++ )
                {
                    N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                    N_FPRINTF( fptr, _T("%18.16f %18.16f\n"), wx, wy );
                }
                break;

            case 2: /* 3-D non-rational */
                for ( i = 0; i <= n; i++ )
                {
                    N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                    N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), wx, wy, wz );
                }
                break;

            case 3: /* 2-D rational */
                for ( i = 0; i <= n; i++ )
                {
                    N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                    N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), wx, wy, w );
                }
                break;

            case 4: /* 3-D rational */
                for ( i = 0; i <= n; i++ )
                {
                    N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                    N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f %18.16f\n"), wx, wy, wz, w );
                }
                break;

            default: /* Wrong type */

                NL_ERROR( NL_CAL_ERR );
        }

        for ( i = 0; i <= m; i++ )
            N_FPRINTF( fptr, _T("%18.16f\n"), U[i] );
    }

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_CrvArrayWriteToFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prints curve  definition data out to the 
     standard output device. The print is arranged as follows:

           n             --> highest index in control point array
           p             --> degree
           rat           --> rationality of the curve (0-no,1-yes)
           dim           --> dimension of the curve (2 or 3)

           <stop>        --> hit any key to continue

           x0 y0 (z0 w0) -->
           x1 y1 (z1 w1) -->
           .             --> 
           .             --> xy(zw) components of control points 
           .             -->
           xn yn (zn wn) -->

           <stop>        --> hit any key to continue

           u0            -->
           u1            -->
           .             -->
           .             --> knots
           .             -->
           um            -->

     This routine allows the programmer to quickly check a curve's
     control points and knots, as well as the appropriate indexes.
     A typical calling example is:

       NL_CURVE  cur;
       ...
       N_CrvPrint(&cur);


   ACCESS:
   
     cur   , input  ,  NURBS curve to be printed


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvPrint( NL_CURVE *cur )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPrint");

    NL_FLAG dim, rat, type, error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_CPOINT *Pw;

    NL_REAL *U, wx, wy, wz, w;

    if( N_CrvAreArraysNULL( cur ) )
    {
        NL_OUT;
    }

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Get different types of output */

    if( N_IsCrvRat( cur ) )
        rat = 1;
    else
        rat = 0;

    if( N_CrvIs3d( cur ) )
        dim = 3;
    else
        dim = 2;

    if( rat EQ 0 )
    {
        if( dim EQ 2 )
            type = 1;
        else
            type = 2;
    }
    else
    {
        if( dim EQ 2 )
            type = 3;
        else
            type = 4;
    }

    /* Print curve data */
    N_FPRINTF( stdout, _T("%ld\n"), n );
    N_FPRINTF( stdout, _T("%hd\n"), p );
    N_FPRINTF( stdout, _T("%hd\n"), rat );
    N_FPRINTF( stdout, _T("%hd\n"), dim );
    NL_PAUSE;

    switch( type )
    {
        case 1: /* 2-D non-rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( stdout, _T("%18.16f  %18.16f\n"), wx, wy );
            }
            break;

        case 2: /* 3-D non-rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( stdout, _T("%18.16f  %18.16f  %18.16f\n"), wx, wy, wz );
            }
            break;

        case 3: /* 2-D rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( stdout, _T("%18.16f  %18.16f  %18.16f\n"), wx, wy, w );
            }
            break;

        case 4: /* 3-D rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( stdout, _T("%18.16f  %18.16f  %18.16f  %18.16f\n"), wx, wy, wz, w );
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    NL_PAUSE;

    for ( i = 0; i <= m; i++ )
    {
        N_FPRINTF( stdout, _T("%18.16f\n"), U[i] );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvPrint */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates a curve from data saved in a file.
     The data file is assumed to be arranged as follows:

           n             --> highest index in control point array
           p             --> degree    
           rat           --> rationality of the curve (0-no,1-yes)
           dim           --> dimension of the curve (2 or 3)
           x0 y0 (z0 w0) -->
           x1 y1 (z1 w1) -->
           .             --> 
           .             --> xy(zw) components of control points 
           .             -->
           xn yn (zn wn) -->
           u0            -->
           u1            -->
           .             -->
           .             --> knots
           .             -->
           um            -->

     If memory is available, the data is  copied into the approriate
     members of the curve structure. Otherwise, memory  is allocated
     first. A typical calling example is:

       NL_CURVE   cur;
       TCHAR* fname;
       NL_STACKS  S;
       ...
       (get file name fname);
       ...
       N_CrvInitArrays(&cur);
       N_CrvReadFromFile(&cur,fname,NL_YES,&S);

     IF THE NL_FLAG chk IS SET TO NL_YES, THE FOLLOWING CHECKS ARE DONE:
       (1) CONSISTENCY, I.E. m = n+p+1;
       (2) NL_DEGREE IS LESS THEN THE NL_MAXIMUM ALLOWED NL_DEGREE;
       (3) WEIGHTS ARE IN THE ALLOWED RANGE; AND
       (4) INTERNAL KNOT MULTIPLICITIES ARE <= THE NL_DEGREE.
     IF chk = NL_YES, THE FOLLOWING SIMPLIFICATIONS ARE PERFORMED:
       (1) IF THE NL_CURVE  LIES IN THE Z=0 NL_PLANE, IT IS CONVERTED INTO 
           A 2-D NL_CURVE; AND
       (2) IF ALL THE WEIGHTS ARE EQUAL, THE NL_CURVE IS CONVERTED INTO
           A NON-RATIONAL NL_CURVE.
     IF chk = NL_NO, THE NL_CURVE IS TAKEN AS IS!!!!  
     Rational curves are stored, where the control points are read and
     written in homogeneous  format   wX, wY, wZ, W. 
     They can be converted to Euclidean by dividing thru by the weight.


   ACCESS:
   
     cur   , in/out ,  NURBS curve to be created
     fname , input  ,  Name of the data file
     chk   , input  ,  Flag:
                         NL_YES: check curve
                         NL_NO : do not check curve; use as it is
     S     , input  ,  cur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReadFromFile( NL_CURVE *cur, const TCHAR* fname, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvReadFromFile");

    NL_FLAG error = NL_NO;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("r") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Call file pointer counterpart */

    error = N_CrvReadFromFilePtr( cur, fptr, chk, S );

    /* Exit */
    N_FileClose( fptr );

    EXIT:

    return (error);
} /* end N_CrvReadFromFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates an array of curves from data saved in 
     a file. The data file is assumed to be arranged as follows:

           r                 --> highest index of curve array  
           n_0               --> highest index in Pw(0)
           p_0               --> degree of curve(0)
           rat_0             --> rationality of the curve(0)
           dim_0             --> dimension of the curve(0)
           <x0 y0 (z0 w0)>_0 
           ...               --> coordinates of control points
           <xn yn (zn wn)>_0  
           <u0>_0         
           ...               --> knots
           <um>_0
           .
           .
           .
           n_r               --> highest index in Pw(r)
           p_r               --> degree of curve(r)
           rat_r             --> rationality of the curve(r)
           dim_r             --> dimension of the curve(r)
           <x0 y0 (z0 w0)>_r 
           ...               --> coordinates of control points
           <xn yn (zn wn)>_r  
           <u0>_r         
           ...               --> knots
           <um>_r

     ALL MEMORIES  ARE  ALLOCATED  INSIDE THE  ROUTINE. ONLY A  POINTER 
     NEEDS TO BE PASSED IN.

       NL_CURVE   **cua;
       NL_INDEX   r;
       TCHAR* fname;
       NL_STACKS  S;
       ...
       (get file name fname);
       ...
       N_CrvArrayReadFromFile(&cua,&r,fname,NL_YES,&S);


   ACCESS:
   
     cua   , output ,  NURBS curves to be created
     r     , output ,  Highest index in cua
     fname , input  ,  Name of the data file
     chk   , input  ,  Flag:
                         NL_YES: check curves for validity (see below)
                         NL_NO : do not check curves
     S     , input  ,  cua's stack

     IF THE NL_FLAG chk IS SET TO NL_YES, THE FOLLOWING CHECKS ARE DONE FOR
     EACH NL_CURVE:
       (1) CONSISTENCY, I.E. m = n+p+1;
       (2) NL_DEGREE IS LESS THEN THE NL_MAXIMUM ALLOWED NL_DEGREE;
       (3) WEIGHTS ARE IN THE ALLOWED RANGE; AND
       (4) INTERNAL KNOT MULTIPLICITIES ARE <= THE NL_DEGREE.
     IF chk = NL_YES, THE FOLLOWING SIMPLIFICATIONS ARE PERFORMED:
       (1) IF THE NL_CURVE  LIES IN THE Z=0 NL_PLANE, IT IS CONVERTED INTO 
           A 2-D NL_CURVE; AND
       (2) IF ALL THE WEIGHTS ARE EQUAL, THE NL_CURVE IS CONVERTED INTO
           A NON-RATIONAL NL_CURVE.
     IF chk = NL_NO, THE CURVES ARE TAKEN AS IS!!!!  
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvArrayReadFromFile( NL_CURVE *** cua, NL_INDEX *r, const TCHAR* fname, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE const NL_STRING rname = _T("N_CrvArrayReadFromFile");

    NL_FLAG dim, rat, type, error = NL_NO;

    NL_INDEX i, k, n, m, t;

    NL_DEGREE p;

    NL_CPOINT *Pw;

    NL_REAL *U, wx, wy, wz, w;

    NL_CURVE ** cpt;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("r") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Allocate memory for array of pointers */

	if (EOF == N_FSCANF(fptr, _T("%ld"), &t)) { NL_ERROR(NL_SCN_ERR); }

    cpt = N_AllocArrayCrvPtrs( t, S );

    if( cpt EQ NULL )
        NL_ERROR( NL_MEM_ERR );

    /* Create curves */

    for ( k = 0; k <= t; k++ )
    {
		if (EOF == N_FSCANF(fptr, _T("%ld"), &n)) { NL_ERROR(NL_SCN_ERR); }
		if (EOF == N_FSCANF(fptr, _T("%hd"), &p)) { NL_ERROR(NL_SCN_ERR); }
		if (EOF == N_FSCANF(fptr, _T("%hd"), &rat)) { NL_ERROR(NL_SCN_ERR); }
		if (EOF == N_FSCANF(fptr, _T("%hd"), &dim)) { NL_ERROR(NL_SCN_ERR); }

        m = n + p + 1;

        cpt[k] = N_AllocCrvAndArrays( n, p, m, S );

        if( cpt[k]EQ NULL )
            NL_ERROR( NL_MEM_ERR );

        N_CrvGetCPtsAndKnots( cpt[k], &Pw, &U );

        if( rat EQ NL_NO )
        {
            if( dim EQ 2 )
                type = 1;
            else
                type = 2;
        }

        else if( dim EQ 2 )
            type = 3;

        else
            type = 4;

        switch( type )
        {
            case 1: /* 2-D non-rational */
                for ( i = 0; i <= n; i++ )
                {
					if (EOF == N_FSCANF(fptr, _T("%lf%lf"), &wx, &wy)) { NL_ERROR(NL_SCN_ERR); }
                    N_CPtFromWxWyWz( wx, wy, NL_NOZ, NL_NOW, &Pw[i] );
                }
                break;

            case 2: /* 3-D non-rational */
                for ( i = 0; i <= n; i++ )
                {
					if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &wx, &wy, &wz)) { NL_ERROR(NL_SCN_ERR); }
                    N_CPtFromWxWyWz( wx, wy, wz, NL_NOW, &Pw[i] );
                }
                break;

            case 3: /* 2-D rational */
                for ( i = 0; i <= n; i++ )
                {
					if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &wx, &wy, &w)) { NL_ERROR(NL_SCN_ERR); }
                    N_CPtFromWxWyWz( wx, wy, NL_NOZ, w, &Pw[i] );
                }
                break;

            case 4: /* 3-D rational */
                for ( i = 0; i <= n; i++ )
                {
					if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf%lf"), &wx, &wy, &wz, &w)) { NL_ERROR(NL_SCN_ERR); }
                    N_CPtFromWxWyWz( wx, wy, wz, w, &Pw[i] );
                }
                break;

            default: /* Wrong type */

                NL_ERROR( NL_CAL_ERR );
        }

        for ( i = 0; i <= m; i++ )
		if (EOF == N_FSCANF(fptr, _T("%lf"), &U[i])) { NL_ERROR(NL_SCN_ERR); }

        if( chk EQ NL_YES )
        {
            error = N_CrvIsValid( cpt[k], rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvPruneRat( cpt[k], NL_YES );
        }
    }

    *cua = cpt;
    *r = t;

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_CrvArrayReadFromFile */

/*******************************************************************//**


   DESCRIPTION:

     Given a curve  object, this routine extracts the  denominator  and
     creates a curve function from it. A typical calling example is:

       NL_CURVE   cur;
       NL_CFUN    cfn;
       NL_STACKS  S;
       ...
       (define curve);
       ...
       N_CFuncInitArrays(&cfn);
       N_CrvGetDenomCrvFunc(&cur,&cfn,&S);

     Since the declarations "NL_CURVE cur" and  "NL_CFUN cfn" define the data 
     types and allocate memory, only the pointers are passed in. If cfn
     is initialized to NULL, memory is allocated locally. Otherwise, it
     is assumed  that memory  allocation  has been  done in the calling 
     routine.


   ACCESS:
   
     cur , input  ,  NURBS curve 
     cfn , output ,  Curve function
     S   , input  ,  cfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetDenomCrvFunc( NL_CURVE *cur, NL_CFUN *cfn, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetDenomCrvFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *U, *UF, *fu;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Check if memory is needed */

    error = N_CFuncSizeArrays( cfn, n, p, m, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( cfn, &fu, &UF );

    /* Copy data */

    for ( i = 0; i <= n; i++ )
        N_CPtGetW( Pw[i], &fu[i] );

    for ( i = 0; i <= m; i++ )
        UF[i] = U[i];

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvGetDenomCrvFunc */

/*******************************************************************//**


   DESCRIPTION:

     Given a curve  object,  this routine  maps the  control polygon to
     Euclidean space. Memory for Euclidean points is allocated locally,
     however, the Euclidean  polygon data  type is declared (allocated)
     in the calling routine. A typical calling example is:

       NL_INDEX     k, l;
       NL_CURVE     cur;
       NL_EPOLYGON  ppl;
       NL_STACKS    S;
       ...
       (define curve);
       ...
       N_CrvGetEPolygon(&cur,k,l,&ppl,&S);

     Since the declarations  "NL_CURVE cur" and "NL_EPOLYGON ppl"  define the 
     data types and allocate memory, only the pointers are passed in. 


   ACCESS:
   
     cur , input  ,  NURBS curve 
     k,l , input  ,  Start  and  end  indexes,  i.e.  only the  control 
                     points Pw[k],...,Pw[l]  are mapped. The  Euclidean
                     points are stored in P[0],...,P[l-k].
     ppl , output ,  Polygon (MEMORY  TO STORE  VERTICES  IS  ALLOCATED
                     LOCALLY)
     S   , input  ,  ppl's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvGetEPolygon */
NL_FLAG N_CrvGetEPolygon( NL_CURVE *cur, NL_INDEX k, NL_INDEX l, NL_EPOLYGON *ppl, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetEPolygon");

    NL_INDEX i, n;

    NL_CPOINT *Pw;

    NL_POINT *P;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check indexes */

    if( k GT l OR k LT 0 OR l GT n )
    {
        N_ErrSet( NL_IND_ERR, rname );
        return (1);
    }

    /* Map control points */

    P = N_AllocPt1dArray( l - k, S );

    if( P EQ NULL )
        return (1);

    for ( i = k; i <= l; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P[i - k] );
    }

    /* Build polgon structure */

    N_EPolygonFromPts( ppl, l - k, P );

    /* Exit */

    return (0);
} /* end N_CrvGetEPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine deallocates memory that stores curve data,
     i.e. polygon  structure, control points, knot  vector structure
     and knots. IT DOES  NOT DEALLOCATE MEMORY THAT STORES THE NL_CURVE
     STRUCTURE ITSELF. A typical calling example is:

       NL_CURVE   *cur;
       NL_STACKS  S;
       ...
       N_FreeCrv(cur,&S);


   ACCESS:
   
     cur , input  ,  Curve pointer
     S   , input  ,  cur's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeCrv */
NL_VOID N_FreeCrv( NL_CURVE *cur, NL_STACKS *S )
{
    NL_DEGREE p;

    NL_REAL *U;

    NL_CPOINT *Pw;

    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    if( cur == NULL )
        return;

    /* Get locals */

    N_CrvDetachPolygonKnot( cur, &pol, &p, &knt );

    if( pol == NULL || knt == NULL )
        return;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Kill curve constituents */

    N_FreeCPolygon( pol, S );
    N_FreeCPt1dArray( Pw, S );
    N_FreeKnotVector( knt, S );
    N_FreeReal1dArray( U, S );
} /* end N_FreeCrv */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores members of a 
     curve structure. Given  a curve  pointer, the  routine
     searches for the  pointer on the  memory  stack.  It it  is found,
     memory is deallocated. If not, nothing is done. 
     A typical calling example is:

       curve    *cur;
       NL_STACKS      S;
       ...
       N_FreeCrvStruct(cur, &S);


   ACCESS:
   
     cur , input  ,  curve pointer 
     S   , input  ,  NL_STACKS


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCrvStruct( NL_CURVE *cur, NL_STACKS *S )
{
    NL_CURNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->cur NEQ NULL )
    {
        prev = S->cur;
        curr = S->cur;

        while( curr NEQ NULL AND curr->ptr NEQ cur )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->cur = NULL;
            }
            else /* More than one node */
            {
                S->cur = S->cur->next;
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
            N_Free( curr );
        }
    }
} /* End N_FreeCrvStruct */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine converts a 3-D curve to a 2-D curve by simply
     setting the third coordinates to the special value NL_NOZ. A typical 
     calling example is:

       NL_CURVE  cur;
       ...
       N_Crv3dTo2d(&cur);

     After the conversion the curve is interpreted as a curve  lying in 
     the [x,y] plane.


   ACCESS:
   
     cur , in/out ,  NURBS curve


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Crv3dTo2d( NL_CURVE *cur )
{
    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to 2-D */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtSetZ( NL_NOZ, &Pw[i] );
    }
} /* end N_Crv3dTo2d */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine converts a 2-D curve to a 3-D curve by simply
     setting  the third  coordinates to  0.0. A typical calling example 
     is:

       NL_CURVE  cur;
       ...
       N_Crv2dTo3d(&cur);

     After the conversion the curve is interpreted as a curve  lying in 
     the [x,y] plane of the [x,y,z] coordinate system.


   ACCESS:
   
     cur , in/out ,  NURBS curve


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Crv2dTo3d( NL_CURVE *cur )
{
    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to 3-D */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtSetZ( 0.0, &Pw[i] );
    }
} /* end N_Crv2dTo3d */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine  converts a rational  curve to a non-rational 
     curve by simply setting the fourth coordinates to the special value 
     NL_NOW. This conversion makes sense only if  the weights are known to
     be one. A typical calling example is:

       NL_CURVE  cur;
       ...
       N_CrvRatToNonRat(&cur);

     After  the  conversion the  curve is  considered  as a non-rational 
     curve, i.e. the weights are simply ignored.



   ACCESS:
   
     cur , in/out ,  NURBS curve


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvRatToNonRat( NL_CURVE *cur )
{
    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to non-rational */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtSetW( NL_NOW, &Pw[i] );
    }
} /* end N_CrvRatToNonRat */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine  converts a non-rational curve to a rational 
     curve  by simply  setting the fourth coordinates to 1.0. A typical 
     calling example is:

       NL_CURVE  cur;
       ...
       N_CrvRatToNonRat(&cur);

     After the conversion the curve is considered as a  rational curve, 
     i.e. the the fourth coordinates are used in all computations.



   ACCESS:
   
     cur , in/out ,  NURBS curve


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvNonRatToRat( NL_CURVE *cur )
{
    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to rational */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtSetW( 1.0, &Pw[i] );
    }
} /* end N_CrvNonRatToRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes a set of curve  definitions compatible,
     i.e. it  makes sure  that all curves  are rational or non-rational 
     and have the same dimension. A typical calling example is:

       NL_CURVE  **cur;
       NL_INDEX  k;
       ...
       (define array of cur);
       ...
       N_CrvsMakeRatCompatible(cur,k);

     After the conversion all curves are either rational or remain non-
     rational, or 3-D or remain 2-D.


   ACCESS:
   
     cur , in/out ,  Array of NURBS curves
     k   , input  ,  Highest index in array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvsMakeRatCompatible( NL_CURVE ** cur, NL_INDEX k )
{
    NL_FLAG allnra, all2d;

    NL_INDEX i;

    /* Check rationality */

    if( !cur )
        return;

    allnra = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( N_IsCrvRat( cur[i] ) )
        {
            allnra = NL_FALSE;
            break;
        }
    }

    /* Check dimensionality */

    all2d = NL_TRUE;

    for ( i = 0; i <= k; i++ )
    {
        if( N_CrvIs3d( cur[i] ) )
        {
            all2d = NL_FALSE;
            break;
        }
    }

    /* If not all non-rational, make all curves rational */

    if( allnra EQ NL_FALSE )
    {
        for ( i = 0; i <= k; i++ )
        {
            if( NOT N_IsCrvRat( cur[i] ) )
            {
                N_CrvNonRatToRat( cur[i] );
            }
        }
    }

    /* If not all 2-D, make all curves 3-D */

    if( all2d EQ NL_FALSE )
    {
        for ( i = 0; i <= k; i++ )
        {
            if( NOT N_CrvIs3d( cur[i] ) )
            {
                N_Crv2dTo3d( cur[i] );
            }
        }
    }
} /* end N_CrvsMakeRatCompatible */

/*******************************************************************//**


   DESCRIPTION:

     Given a  curve object, this  routine scales  the knot vector to a 
     given  interval. That  is, U[0] <= U[1] <= ... <= U[m] is  mapped
     to  a <= ... <=  U[p+1]' <= ... <=  U[m-p-1]'  <= ... <= b, where
     I=[a,b] is the given  interval. A typical calling example is: 

       NL_CURVE     cur;
       NL_INTERVAL  I;
       ...
       (get new interval I);
       ...
       N_CrvReparamToInterval(&cur,I);

     The knot vector is rescaled IN-PLACE, i.e. the original knots are
     destroyed.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     I   , input  ,  Parameter interval


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvReparamToInterval( NL_CURVE *cur, NL_INTERVAL I )
{
    NL_INDEX i, m;

    NL_DEGREE p;

    NL_REAL *U, fac, a, b, u0;

    /* Get local notation */

    N_CrvGetDegree( cur, &p );
    N_CrvGetKnots( cur, &m, &U );
    N_IntervalGetData( &I, &a, &b );

    /* Compute new knots */

    if( a NEQ U[0]OR b NEQ U[m] )
    {
        u0 = U[0];
        fac = (b - a) / (U[m] - U[0]);

        for ( i = 0; i <= p; i++ )
            U[i] = a;

        for ( i = p + 1; i <= m - p - 1; i++ )
            U[i] = fac * (U[i] - u0) + a;

        for ( i = m - p; i <= m; i++ )
            U[i] = b;
    }
} /* end N_CrvReparamToInterval */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if curve weights are equal or not. A 
     typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_CrvAreWeightsEqual(&cur) )  --> curve weights are equal;
     

   ACCESS:
   
     cur , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE : Curve weights are equal
     NL_FALSE: Curve weights are NOT equal

   ***********************************************************************/

/* NL_BOOLEAN  N_CrvAreWeightsEqual */
NL_BOOLEAN N_CrvAreWeightsEqual( NL_CURVE *cur )
{
    NL_INDEX i, n;

    NL_REAL w, wmin, wmax;

    NL_CPOINT *Pw;

    /* Get polygon */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Get min and max weights */

    N_CPtGetW( Pw[0], &w );
    wmin = wmax = w;

    for ( i = 1; i <= n; i++ )
    {
        N_CPtGetW( Pw[i], &w );

        if( w LT wmin )
            wmin = w;

        if( w GT wmax )
            wmax = w;
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
} /* end N_CrvAreWeightsEqual */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine maps a rational curve to Euclidean space. That
     is,  for each  control point Pw = (xw,yw,zw,w), it  computes a  new 
     control point Qw = (x,y,z,NL_NOW). A typical calling example is:

       NL_CURVE  cur;
       ...
       N_CrvMakeNonRat(&cur);


   ACCESS:
   
     cur , in/out ,  NURBS curve


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvMakeNonRat( NL_CURVE *cur )
{
    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get polygon */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Map to Euclidean space */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtNoW( Pw[i], &Pw[i] );
    }
} /* end N_CrvMakeNonRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prunes a rational curve, i.e. it checks if the
     weights  are equal, and  if so, the  curve  is  converted into non-
     rational form. It also  checks if the curve lies in  the z=0 plane. 
     If yes, the curve is converted into a 2-D curve.

       NL_CURVE  cur;
       ...
       N_CrvPruneRat(&cur,NL_YES);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     flg , input  ,  Flag:
                       NL_YES: check if curve is in z=0 plane
                       NL_NO : consider cur  as a 3-D curve  even  if all z
                            coordinates are zero


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvPruneRat( NL_CURVE *cur, NL_FLAG flg )
{
    if( N_IsCrvRat( cur ) )
    {
        if( N_CrvAreWeightsEqual( cur ) )
            N_CrvMakeNonRat( cur );
    }

    if( flg EQ NL_YES )
    {
        if( N_CrvIsInZ0Plane( cur ) )
            N_Crv3dTo2d( cur );
    }
} /* end N_CrvPruneRat */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a NURBS curve is degenerated to a
     single point in 4-D. A typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_Crv4dIsDegen(&cur) )  --> handle 4-D point;
     

   ACCESS:
   
     cur  , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE : Curve is a point
     NL_FALSE: Curve is NOT a point

   ***********************************************************************/

/* NL_BOOLEAN  N_Crv4dIsDegen */
NL_BOOLEAN N_Crv4dIsDegen( NL_CURVE *cur )
{
    NL_FLAG dst = NL_YES;

    NL_INDEX i, n;

    NL_REAL d, fac;

    NL_CPOINT *Pw, Mw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check if curve is a point */

    fac = 1.0 / ((NL_REAL)n + (NL_REAL)1);
    N_CopyCPt( NL_CZERO, &Mw );

    for ( i = 0; i <= n; i++ )
        N_Sum2CPts( Mw, Pw[i], &Mw );

    N_ScaleCPt( fac, Mw, &Mw );

    for ( i = 0; i <= n; i++ )
    {
        N_DistCptCptHomo( Pw[i], Mw, &d );

        if( d GT NL_MTOL )
        {
            dst = NL_NO;
            break;
        }
    }

    if( dst EQ NL_YES )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_Crv4dIsDegen */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the curve lies in the z=0 plane. A 
     typical calling example is:

       NL_CURVE  cur;
       ...
       if( N_CrvIsInZ0Plane(&cur) )  --> curve lies in z=0;
     

   ACCESS:
   
     cur , input ,  NURBS curve


   RETURN CODES:

     NL_TRUE : Curve lies in z=0 plane
     NL_FALSE: Curve DOES NOT lie in z=0 plane

   ***********************************************************************/

/* NL_BOOLEAN  N_CrvIsInZ0Plane */
NL_BOOLEAN N_CrvIsInZ0Plane( NL_CURVE *cur )
{
    NL_FLAG twod = NL_YES;

    NL_INDEX i, n;

    NL_REAL z;

    NL_CPOINT *Pw;

    /* Get polygon */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Check if all z = 0 */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtGetZ( Pw[i], &z );

        if( z NEQ 0.0 AND z NEQ NL_NOZ )
        {
            twod = NL_NO;
            break;
        }
    }

    if( twod EQ NL_YES )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CrvIsInZ0Plane */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a  NURBS curve is planar or not. If
     the curve is a  point, a line or a  3-D curve,  NL_FALSE is returned.
     If the curve is a 2-D non-degenerate curve, NL_TRUE is returned along
     with a point and the  unit normal of the  plane. A typical calling 
     example is:

       NL_FLAG    flt;
       NL_REAL    tol;
       NL_CURVE   cur;
       NL_POINT   PP;
       NL_VECTOR  NN;
       NL_STACKS  SG;
       ...
       (get tol)
       ...
       N_CrvIsPlanar(&cur,tol,&PP,&NN,&flt,&SG)
     

   ACCESS:
   
     cur , input  ,  NURBS curve
     tol , input  ,  Tolerance to measure flatness
     PP  , output ,  Point on the plane
     NN  , output ,  Unit normal to the plane
     flt , output ,  Flag:
                       NL_TRUE : curve is planar
                       NL_FALSE: curve is NOT planar
     SG  , input  ,  Global memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsPlanar( NL_CURVE *cur, NL_REAL tol, NL_POINT *PP, NL_VECTOR *NN, NL_FLAG *flt, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX ii, n, count1, count2;

    NL_REAL a, b, c, d, fac, ang, mag;

    NL_POINT *P, CG;

    NL_VECTOR V1, V2, NN1, NN2, VN;

    NL_CPOINT *Pw;

    /* Check special cases */

    P = NULL;

    *flt = NL_FALSE;

    N_CrvGetCPts( cur, &n, &Pw );

    if( n LT 2 )
        NL_OUT;

    if( N_CrvIsDegen( cur ) )
        NL_OUT;

    if( N_CrvIsLine( cur, tol ) )
        NL_OUT;

    /* Project all control points to Euclidean space */

    P = N_AllocPt1dArray( n, SG );

    if( P EQ NULL )
        NL_QUIT;

    for ( ii = 0; ii <= n; ii++ )
        N_CPtToPtEuclid( Pw[ii], &P[ii] );

    /*  Compute candidate point on plane and normal vector  */

    count1 = count2 = 0;
    N_CopyPt( NL_ZERO, &CG );
    N_CopyPt( NL_ZERO, &NN1 );
    N_CopyPt( NL_ZERO, &NN2 );

    for ( ii = 0; ii <= n; ii++ )
    {
        N_Sum2Pts( CG, P[ii], &CG );

        if( ii GT 0 AND ii LT n )
        {
            N_VectorDir( P[ii - 1], P[ii], &V1 );
            N_VectorDir( P[ii], P[ii + 1], &V2 );

            error = N_VectorsAngle( V1, V2, &ang );

            N_VectorCross( V1, V2, &VN );
            error = N_VectorNormalize( VN, &VN, &mag );

            if( error EQ NL_NO )
            {
                if( count1 GT 0 )
                {
                    N_VectorDot( VN, NN1, &d );

                    if( d LT 0.0 )
                        N_VectorScale( VN, -1.0, &VN );
                }
                else if( count2 GT 0 )
                {
                    N_VectorDot( VN, NN2, &d );

                    if( d LT 0.0 )
                        N_VectorScale( VN, -1.0, &VN );
                }
            }

            if( error EQ NL_YES OR ang LT 1.0 OR ang GT 179. )
            {
                if( error EQ NL_NO )
                {
                    N_VectorSum( NN2, VN, &NN2 );
                    count2 += 1;
                }
            }
            else
            {
                N_VectorSum( NN1, VN, &NN1 );
                count1 += 1;
            }
        }
    }

    if( count1 + count2 EQ 0 )
    {
        error = NL_NO;
        NL_OUT;
    }

    N_ScalePt( 1.0 / ((NL_REAL)n + (NL_REAL)1), CG, PP );

    if( count1 GT 0 )
        N_VectorScale( NN1, 1.0 / count1, NN );
    else
        N_VectorScale( NN2, 1.0 / count2, NN );

    error = N_VectorNormalize( *NN, NN, &mag );

    if( error EQ NL_YES )
        NL_OUT;

    /* Check flatness */

    error = N_PlanePtNormalToImplicit( *PP, *NN, &a, &b, &c, &d );

    if( error EQ NL_YES )
        NL_OUT;

    for ( ii = 0; ii <= n; ii++ )
    {
        N_DistSignedPtPlane( a, b, c, d, P[ii], &fac );

        if( fabs( fac )GT tol )
            NL_OUT;
    }

    *flt = NL_TRUE;

    /* Exit */

    EXIT:
    if( P NEQ NULL )
        N_FreePt1dArray( P, SG );

    return (error);
} /* end N_CrvIsPlanar */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates a curve from data saved in a file.
     The file pointer is passed in. The file is assumed to be opened
     and correctly positioned to read the curve. The data is assumed
     to be arranged as follows:

           n             --> highest index in control point array
           p             --> degree    
           rat           --> rationality of the curve (0-no,1-yes)
           dim           --> dimension of the curve (2 or 3)
           x0 y0 (z0 w0) -->
           x1 y1 (z1 w1) -->
           .             --> 
           .             --> xy(zw) components of control points 
           .             -->
           xn yn (zn wn) -->
           u0            -->
           u1            -->
           .             -->
           .             --> knots
           .             -->
           um            -->

     If memory is available, the data is  copied into the approriate
     members of the curve structure. Otherwise, memory  is allocated
     first. A typical calling example is:

       NL_CURVE   cur;
       FILE    *fptr;
       NL_STACKS  S;
       ...
       (open file, assign pointer, and position for curve read);
       ...
       N_CrvInitArrays(&cur);
       N_CrvReadFromFilePtr(&cur,fptr,NL_YES,&S);

     IF THE NL_FLAG chk IS SET TO NL_YES, THE FOLLOWING CHECKS ARE DONE:
       (1) CONSISTENCY, I.E. m = n+p+1;
       (2) NL_DEGREE IS LESS THEN THE NL_MAXIMUM ALLOWED NL_DEGREE;
       (3) WEIGHTS ARE IN THE ALLOWED RANGE; AND
       (4) INTERNAL KNOT MULTIPLICITIES ARE <= THE NL_DEGREE.
     IF chk = NL_YES, THE FOLLOWING SIMPLIFICATIONS ARE PERFORMED:
       (1) IF THE NL_CURVE  LIES IN THE Z=0 NL_PLANE, IT IS CONVERTED INTO 
           A 2-D NL_CURVE; AND
       (2) IF ALL THE WEIGHTS ARE EQUAL, THE NL_CURVE IS CONVERTED INTO
           A NON-RATIONAL NL_CURVE.
     IF chk = NL_NO, THE NL_CURVE IS TAKEN AS IS!!!! 
     Rational curves are stored, where the control points are read and
     written in homogeneous  format   wX, wY, wZ, W. 
     They can be converted to Euclidean by dividing thru by the weight.

   ACCESS:
   
     cur   , in/out ,  NURBS curve to be created
     fptr  , input  ,  Pointer to data file
     chk   , input  ,  Flag:
                         NL_YES: check curve
                         NL_NO : do not check curve; use as it is
     S     , input  ,  cur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReadFromFilePtr( NL_CURVE *cur, FILE *fptr, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvReadFromFilePtr");

    NL_FLAG dim, rat, type, error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_CPOINT *Pw;

    NL_REAL *U, x, y, z, wx, wy, wz, w;

    /* Get parameters from the top */

	if (EOF == N_FSCANF(fptr, _T("%ld"), &n)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd"), &p)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd"), &rat)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd"), &dim)) { NL_ERROR(NL_SCN_ERR); }

    m = n + p + 1;

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, n, p, m, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get different types of input */

    if( rat EQ NL_NO )
    {
        if( dim EQ 2 )
            type = 1;
        else
            type = 2;
    }
    else
    {
        if( dim EQ 2 )
            type = 3;
        else
            type = 4;
    }

    /* Read in data */

    switch( type )
    {
        case 1: /* 2-D non-rational */
            for ( i = 0; i <= n; i++ )
            {
				if (EOF == N_FSCANF(fptr, _T("%lf%lf"), &x, &y)) { NL_ERROR(NL_SCN_ERR); }
                N_CPtFromWxWyWz( x, y, NL_NOZ, NL_NOW, &Pw[i] );
            }
            break;

        case 2: /* 3-D non-rational */
            for ( i = 0; i <= n; i++ )
            {
				if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &x, &y, &z)) { NL_ERROR(NL_SCN_ERR); }
                N_CPtFromWxWyWz( x, y, z, NL_NOW, &Pw[i] );
            }
            break;

        case 3: /* 2-D rational */
            for ( i = 0; i <= n; i++ )
            {
				if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &wx, &wy, &w)) { NL_ERROR(NL_SCN_ERR); }
                N_CPtFromWxWyWz( wx, wy, NL_NOZ, w, &Pw[i] );
            }
            break;

        case 4: /* 3-D rational */
            for ( i = 0; i <= n; i++ )
            {
				if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf%lf"), &wx, &wy, &wz, &w)) { NL_ERROR(NL_SCN_ERR); }
                N_CPtFromWxWyWz( wx, wy, wz, w, &Pw[i] );
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= m; i++ )
    {
		if (EOF == N_FSCANF(fptr, _T("%lf"), &U[i])) { NL_ERROR(NL_SCN_ERR); }
    }

    /* Check curve and prune */

    if( chk EQ NL_YES )
    {
        error = N_CrvIsValid( cur, rname );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvPruneRat( cur, NL_YES );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvReadFromFilePtr */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine writes a curve to a file.  The  file pointer 
     is passed in.  The file is assumed to  be  opened  and  correctly 
     positioned to write the curve. The data is assumed to be arranged 
     as follows:

           n             --> highest index in control point array
           p             --> degree
           rat           --> rationality of the curve (0-no,1-yes)
           dim           --> dimension of the curve (2 or 3)
           x0 y0 (z0 w0) -->
           x1 y1 (z1 w1) -->
           .             --> 
           .             --> xy(zw) components of control points 
           .             -->
           xn yn (zn wn) -->
           u0            -->
           u1            -->
           .             -->
           .             --> knots
           .             -->
           um            -->

     A typical calling example is:

       NL_CURVE   cur;
       FILE    *fptr;
       ...
       (open file, assign pointer, and position for curve write);
       ...
       N_CrvWriteToFilePtr(&cur,fptr);


   ACCESS:
   
     cur   , input  ,  NURBS curve to be saved
     fptr  , output ,  Pointer to the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvWriteToFilePtr( NL_CURVE *cur, FILE *fptr )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvWriteToFilePtr");

    NL_FLAG dim, rat, type, error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_CPOINT *Pw;

    NL_REAL *U, wx, wy, wz, w;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Get different types of output */

    if( N_IsCrvRat( cur ) )
        rat = 1;
    else
        rat = 0;

    if( N_CrvIs3d( cur ) )
        dim = 3;
    else
        dim = 2;

    if( rat EQ 0 )
    {
        if( dim EQ 2 )
            type = 1;
        else
            type = 2;
    }
    else
    {
        if( dim EQ 2 )
            type = 3;
        else
            type = 4;
    }

    /* Create the output file */
    N_FPRINTF( fptr, _T("%ld\n"), n );
    N_FPRINTF( fptr, _T("%hd\n"), p );
    N_FPRINTF( fptr, _T("%hd\n"), rat );
    N_FPRINTF( fptr, _T("%hd\n"), dim );

    switch( type )
    {
        case 1: /* 2-D non-rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( fptr, _T("%18.16f %18.16f\n"), wx, wy );
            }
            break;

        case 2: /* 3-D non-rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), wx, wy, wz );
            }
            break;

        case 3: /* 2-D rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), wx, wy, w );
            }
            break;

        case 4: /* 3-D rational */
            for ( i = 0; i <= n; i++ )
            {
                N_CPtToWxWyWz( Pw[i], &wx, &wy, &wz, &w );
                N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f %18.16f\n"), wx, wy, wz, w );
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= m; i++ )
    {
        N_FPRINTF( fptr, _T("%18.16f\n"), U[i] );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvWriteToFilePtr */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates an  NL_IGES file for curve  visualization
     with  or  without the control  polygon. The  NL_IGES file  is named as 
     specified in the argument list. A typical calling example is:

       NL_CURVE   **curs;
       NL_INDEX   *ccl, lcl, nc;
       NL_REAL    tol;
       NL_STRING  fname;
       ...
       (get file name, curs, ccl, lcl, tol);
       ...
       N_CrvWriteIgesCPolygon(curs,nc,tol,NL_YES,ccl,NL_NO,lcl,fname);


   ACCESS:
   
     curs  , input  ,  Pointers to the NURBS curves to be written to the 
                       NL_IGES file. curs[i] is a pointer to the i-th curve
     nc    , input  ,  High  index of  curve  pointers  (there  are nc+1
                       pointers in curs)
     tol   , input  ,  Minimum model resolution tolerance  (Parameter 19
                       of the Global Section)
     cfl   , input  ,  Flag:
                         NL_YES: output curve
                         NL_NO : do not output curve
     ccl   , input  ,  Curve  colors (nc+1  color indexes). See NL_IGES for
                       color values (or set any values 1 to 9 if not im-
                       portant).
     nfl   , input  ,  Flag:
                         NL_YES: output control polygon
                         NL_NO : do not output control polygon
     lcl   , input  ,  Color index for control polygons
     fname , output ,  Name of the NL_IGES file (may  not  be more than 30
                       characters long)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvWriteIgesCPolygon( NL_CURVE ** curs, NL_INDEX nc, NL_REAL tol, NL_FLAG cfl, NL_INDEX *ccl, NL_FLAG pfl, NL_INDEX lcl, TCHAR* fname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *col=NULL, i, k, nl, np, n, m;

    NL_POINT ** P;

    NL_CPOINT *Pw;

    NL_CURVE ** curl;

    NL_STACKS SL;

    /* Open Nlib */

    N_InitNurbs( &SL );

    /* See if curve is to be output */

    if( cfl EQ NL_YES )
    {
        curl = curs;
        nl = nc;
    }
    else
    {
        curl = NULL;
        nl = -1;
    }

    /* See if polygon is to be output */

    if( pfl EQ NL_NO )
    {
        P = NULL;
        np = -1;
    }
    else
    {
        /* Get number of lines */

        np = 0;

        for ( k = 0; k <= nc; k++ )
        {
            N_CrvGetArraySizes( curs[k], &n, &m );

            np += (n + 1);
        }

        P = N_AllocPt2dArray( np, 1, &SL );

        if( P EQ NULL )
            NL_QUIT;

        col = N_AllocInt1dArray( np, &SL );

        if( col EQ NULL )
            NL_QUIT;

        /* Load line segments */

        np = -1;

        for ( k = 0; k <= nc; k++ )
        {
            N_CrvGetCPts( curs[k], &n, &Pw );

            for ( i = 1; i <= n; i++ )
            {
                np++;
                N_CPtToPtEuclid( Pw[i - 1], &P[np][0] );
                N_CPtToPtEuclid( Pw[i], &P[np][1] );
                col[np] = lcl;
            }
        }
    }

    /* Create NL_IGES file */

    error = N_WriteLineCrvSrfIges( P, np, col, curl, nl, ccl, NULL, -1, tol, fname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Close Nlib */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvWriteIgesCPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates an NL_IGES file  and writes curves to it
     as 126 Entities.  The NL_IGES file is named as specified in the  arg-
     ument list. The Start and Global sections of the NL_IGES file contain
     a minimum amount of information;  this  can be subsequently edited
     as desired. A typical calling example is:

       NL_CURVE    **curs;
       TCHAR* fname;
       ...
       (get file name, fname, and tol, and create curves in curs);
       ...
       N_CrvWriteIges(curs,nc,tol,fname);


   ACCESS:
   
     curs  , input  ,  Pointers to the NURBS curves to be written to the
                       NL_IGES file. curs[i] is a pointer to the i-th curve
     nc    , input  ,  High  index  of  curve  pointers  (there are nc+1
                       pointers in curs)
     tol   , input  ,  Minimum model resolution tolerance  (Parameter 19
                       of the Global Section)
     fname , output ,  Name of the NL_IGES file (may  not  be  more than 30
                       characters long)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvWriteIges( NL_CURVE ** curs, NL_INDEX nc, NL_REAL tol, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvWriteIges");
    NL_PRIVATE NL_STRING blank = _T(" ");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, ii, jj, npl, dline, pline;

    NL_INTEGER rat, closed, planar;

    NL_DEGREE p;

    NL_CPOINT *Pw;

    NL_REAL *U, x, y, z, w, maxc, d1, d2, d3, d4, d5, d6;

    NL_MINMAXBOX box;

    NL_POINT norm;

    FILE *fptr = NULL;

    /* Check for input error and open file */

    if( nc LT 0 )
        NL_ERROR( NL_INP_ERR );

    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Write Start Section */

    N_FPRINTF( fptr, _T("Nurbs curves written out using the%38sS%7d\n"), blank, 1 );

    /* Write Global Section */

    maxc = 0.0;

    for ( ii = 0; ii <= nc; ii++ )
    {
        N_CrvGetBBox( curs[ii], &box );
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
    N_FPRINTF( fptr, _T("1, %5.3f,13H             , %18.13lf,%26sG%7d\n"), 0.001, tol, blank, 3 );
    N_FPRINTF( fptr, _T("%22.10lf,1H ,1H ,%3d,0,%35sG%7d\n"), maxc, 9, blank, 4 );
    N_FPRINTF( fptr, _T("13H             ;%55sG%7d\n"), blank, 5 );

    /* Write the Directory Entries */
    pline = 1;
    dline = 1;

    for ( ii = 0; ii <= nc; ii++ )
    {
        /* Compute number of lines in Parameter Data Section */

        N_CrvGetArraySizes( curs[ii], &n, &m );

        npl = 3;            /* integer data, trim bounds & unit normal */
        npl += (m / 3 + 1); /* knots */
        npl += (n / 4 + 1); /* weights */
        npl += (n + 1);     /* control points */

        /* Write the Directory Entry for this curve */

        N_FPRINTF( fptr, _T("%8d%8ld%8d%8d%8d%8d%8d%8d00000000D%7ld\n"), 126, pline, 0, 1, 4, 0, 0, 0, dline );
        dline += 1;
        N_FPRINTF( fptr, _T("%8d%8d%8ld%8ld%8d%8d%8dNURBSURF%8ldD%7ld\n"), 126, 1, (ii + 2) % 8, npl, 0, 0, 0, ii + 1, dline );
        dline += 1;

        pline += npl;
    }

    /* Write the Parameter Data Section */
    pline = 1;

    for ( ii = 0; ii <= nc; ii++ )
    {
        /* Get local notation */

        N_CrvGetCPtsDegreeAndKnots( curs[ii], &n, &Pw, &p, &m, &U );

        /* Compute the Properties */

        if( N_CrvIsClosed( curs[ii] ) )
            closed = 1;
        else
            closed = 0;

        rat = 1;

        if( N_IsCrvRat( curs[ii] ) )
        {
            for ( jj = 0; jj <= n; jj++ )
            {
                N_CPtGetW( Pw[jj], &w );

                if( w NEQ 1.0 )
                {
                    rat = 0;
                    break;
                }
            }
        }

        N_CopyPt( NL_ZERO, &norm );

        if( N_CrvIs3d( curs[ii] ) )
            planar = 0;     /* just flag as planar */
        else                /* if xy-planar        */
        {                   /* in xy-plane */
            if( N_CrvIsLine( curs[ii], tol ) )
                planar = 0; /* plane not unique */
            else
            {
                planar = 1;
                N_PtFromXYZ( 0.0, 0.0, 1.0, &norm );
            }
        }

        /* Write the integer data */

        N_FPRINTF( fptr, _T("%3d,%5ld,%3d,%2ld,%2ld,%2ld,%2d,%39s%7ldP%7ld\n"), 126, n, p, planar, closed, rat, 0, blank, 2 * ii + 1, pline );
        pline += 1;

        /* Write the knots */

        for ( jj = 0; jj <= m; jj += 3 )
        {
            if( jj EQ m )
                N_FPRINTF( fptr, _T("%19.12lf,%44s %7ldP%7ld\n"), U[jj], blank, 2 * ii + 1, pline );

            else if( jj EQ m - 1 )
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[jj], U[jj + 1], blank, 2 *ii + 1, pline );

            else
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), U[jj], U[jj + 1], U[jj + 2], 2 *ii + 1, pline );

            pline += 1;
        }

        /* Write the weights */

        if( rat EQ 1 )
        { /* non-rational */
            for ( jj = 0; jj <= n; jj += 4 )
            {
                if( jj EQ n )
                    N_FPRINTF( fptr, _T("%13lf,%50s %7ldP%7ld\n"), 1.0, blank, 2 * ii + 1, pline );

                else if( jj EQ n - 1 )
                    N_FPRINTF( fptr, _T("%13lf , %13lf,%34s %7ldP%7ld\n"), 1.0, 1.0, blank, 2 *ii + 1, pline );

                else if( jj EQ n - 2 )
                    N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf,%18s %7ldP%7ld\n"), 1.0, 1.0, 1.0, blank, 2 *ii + 1, pline );

                else
                    N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf , %13lf,   %7ldP%7ld\n"), 1.0, 1.0, 1.0, 1.0, 2 *ii + 1, pline );

                pline += 1;
            }
        }
        else
        { /* rational */
            for ( jj = 0; jj <= n; jj += 4 )
            {
                if( jj EQ n )
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_FPRINTF( fptr, _T("%13.7lf,%50s %7ldP%7ld\n"), d1, blank, 2 * ii + 1, pline );
                }
                else if( jj EQ n - 1 )
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_CPtGetW( Pw[jj + 1], &d2 );
                    N_FPRINTF( fptr, _T("%13.7lf , %13.7lf,%34s %7ldP%7ld\n"), d1, d2, blank, 2 * ii + 1, pline );
                }
                else if( jj EQ n - 2 )
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_CPtGetW( Pw[jj + 1], &d2 );
                    N_CPtGetW( Pw[jj + 2], &d3 );
                    N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf,%18s %7ldP%7ld\n"), d1, d2, d3, blank, 2 * ii + 1, pline );
                }
                else
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_CPtGetW( Pw[jj + 1], &d2 );
                    N_CPtGetW( Pw[jj + 2], &d3 );
                    N_CPtGetW( Pw[jj + 3], &d4 );
                    N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf , %13.7lf,   %7ldP%7ld\n"), d1, d2, d3, d4, 2 * ii + 1, pline );
                }

                pline += 1;
            }
        }

        /* Write the Euclidean control points */

        for ( jj = 0; jj <= n; jj++ )
        {
            N_CPtToXYZ( Pw[jj], &x, &y, &z );
            N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), x, y, z, 2 * ii + 1, pline );

            pline += 1;
        }

        /* Write the trim bounds */

        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[0], U[m], blank, 2 * ii + 1, pline );
        pline += 1;

        /* Write the unit normal */

        N_PtToXYZ( norm, &x, &y, &z );
        N_FPRINTF( fptr, _T("%15lf , %15lf , %15lf;%12s %7ldP%7ld\n"), x, y, z, blank, 2 * ii + 1, pline );
        pline += 1;
    }

    /* Write the Terminate Section */

    N_FPRINTF( fptr, _T("S%7dG%7dD%7ldP%7ld%40sT%7d\n"), 1, 5, dline - 1, pline - 1, blank, 1 );

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_CrvWriteIges */

/**********************************************************************/
/* N_crvJet:  Implementation of NL_CURVEJET struct                     */
/**********************************************************************/

/*
 *  This struct pretends to be a class that represents an evaluated
 *  point on a curve.  See description on nurbsdef.h.
 */

/* a local helper function */
static NL_BOOLEAN curveJetEval( NL_CURVEJET *poc, int numDerivs )
{
    NL_PRIVATE NL_STRING rname = _T("N_crvJet");
    NL_FLAG ret;
    NL_FLAG leftRight;

    if( numDerivs > poc->MAX_DERIVS )
        numDerivs = (int)poc->MAX_DERIVS;

    /*  See whether we need to do anything  */
    if( poc->numEval >= numDerivs )
        return NL_TRUE;

    leftRight = NL_LEFT;

    if( N_crvJetUnset( poc ) )
    {
        N_ErrSet( NL_INP_ERR, rname );
        return NL_FALSE;
    }

    ret = N_CrvDerivs( poc->myCurve, poc->param, leftRight, numDerivs, poc->derivs );

    if( ret == NL_NO )
        poc->numEval = numDerivs;

    return (ret == NL_NO);
}

NL_VOID N_crvJetInit( NL_CURVEJET *poc, NL_CURVE *crv )
{
    poc->MAX_DERIVS = CURVEJET__MAX_DERIVS;

    poc->myCurve = crv;

    N_crvJetReset( poc );
}

NL_BOOLEAN N_crvJetUnset( NL_CURVEJET *poc )
{
    if( poc->numEval > -2 )
        return NL_FALSE;

    return NL_TRUE;
}

NL_VOID N_crvJetReset( NL_CURVEJET *poc )
{
    poc->numEval = -2;
}

NL_VOID N_crvJetSetParam( NL_CURVEJET *poc, double prm )
{
    poc->param = prm;
    poc->numEval = -1;
}

NL_POINT *N_crvJetPos( NL_CURVEJET *poc )
{
    curveJetEval( poc, 0 );
    return &( poc->derivs[0] );
}

NL_VECTOR *N_crvJetDer1( NL_CURVEJET *poc )
{
    curveJetEval( poc, 1 );
    return &( poc->derivs[1] );
}

NL_VECTOR *N_crvJetDer2( NL_CURVEJET *poc )
{
    curveJetEval( poc, 2 );
    return &( poc->derivs[2] );
}

/* versions that make copies: */
NL_VOID N_crvJetPosCopy( NL_CURVEJET *poc, NL_POINT *pt )
{
    N_VectorCopy( *( N_crvJetPos( poc ) ), pt );
}

NL_VOID N_crvJetDer1Copy( NL_CURVEJET *poc, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_crvJetDer1( poc ) ), deriv );
}

NL_VOID N_crvJetDer2Copy( NL_CURVEJET *poc, NL_VECTOR *deriv )
{
    N_VectorCopy( *( N_crvJetDer2( poc ) ), deriv );
}

NL_BOOLEAN N_crvJetCurvature( NL_CURVEJET *poc, NL_VECTOR *kappaVec )
{
    NL_BOOLEAN ret = NL_TRUE;

    double numer, denom;
    NL_VECTOR cross, unitTan;

    /* just to check overflow: */
    double EPS = 1e-12;

    /* initialize our derivs */
    curveJetEval( poc, 2 );

    N_VectorCrossRef( N_crvJetDer1( poc ), N_crvJetDer2( poc ), &cross );

    /*  We're going to need the magnitude of the cross product to check
    *  for overflow, but while we're at it, we can use it to quit early
    *  if it's zero.
    */
    N_VectorMagnitudeRef( &cross, &numer );

    if( numer < NL_ZCTL )
    {
        N_VectorCopy( NL_ZERO, kappaVec );
        return NL_TRUE;
    }

    N_VectorMagnitudeRef( N_crvJetDer1( poc ), &denom );

    denom = denom * denom * denom;

    if( denom > numer * EPS )
        N_VectorScaleRef( &cross, 1 / denom, &cross );
    else
        ret = NL_FALSE;

    /*  This now points in the direction of the binormal,
    *  we want it in the direction of N.
    *  Cross it with the unitized tangent.
    *  The magnitude won't change, since they're perpendicular.
*/
    N_VectorCopy( *( N_crvJetDer1( poc ) ), &unitTan );
    N_VectorNormalizeRef( &unitTan );

    /*  Note: can't pass the same vector for in and out to N_VectorCrossRef.  */
    N_VectorCrossRef( &cross, &unitTan, kappaVec );

    return ret;
}

NL_BOOLEAN N_crvJetRelax( NL_CURVEJET *poc, NL_POINT *pt, NL_GCPTEMP *crvData, NL_STACKS *stks )
{
    NL_FLAG err = NL_YES;

    NL_PARAMETER prm = 0.0;
    NL_POINT tmpPt;

    if( !N_crvJetUnset( poc ) )
    {
        /*  Param is set: it's the guess point.  Use the local solver:  */

        err = N_CrvClosestPt( poc->myCurve, *pt, poc->param, NL_MTOL, NL_MTOL, &prm, &tmpPt );
    }

    /*  If we didn't have a guess, or if it failed with a guess,
    *  try it without a guess: use the global solver, N_CrvClosestPtMultiple.
    */

    if( err == NL_YES )
    {
        /*  No guess param  */
        NL_FLAG Qret;

        err = N_CrvClosestPtMultiple( poc->myCurve, *pt, -1, NL_MTOL, NL_MTOL, crvData, &prm, &tmpPt, &Qret, stks );

        if( Qret == NL_NO )
            err = NL_YES;
    }

    if( err == NL_NO )
        N_crvJetSetParam( poc, prm );

    return (err == NL_NO);
}
