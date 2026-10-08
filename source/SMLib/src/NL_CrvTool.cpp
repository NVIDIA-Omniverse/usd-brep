// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************/
/* CrvTool.c : Tool Function Definitions that act on NL_CURVE objects */
/*******************************************************************/

#include "StdAfx.h"


#include <nurbs.h>
#include <NL_Globals.h>


NL_PRIVATE NL_REAL cto = 1.0e-05;
NL_PRIVATE NL_REAL NOREM = 1.0e+25;

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine  inserts a  new knot  into a  NURBS curve.  The 
     new knot must be an interior knot and the sum of the multiplicities
     of  the old  and the  new knots  must be  less than or equal to the 
     degree. If the output curve is initialized to NULL, memory to store 
     new  control points and  knots is allocated. If the output curve is
     the same as the input  curve, knot  insertion is  done in place and 
     the original curve is destroyed. A typical calling example is: 

       NL_CURVE      curP, curQ;
       NL_PARAMETER  u;
       NL_INDEX      r;
       NL_STACKS     SP, SQ;
       ...
       (define curP, get u and r);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvInsertKnot(&curP,u,r,&curQ,&SP,&SQ);
       N_CrvInsertKnot(&curP,u,r,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and  polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     u    , input  ,  New knot
     r    , input  ,  Number of times u is to be inserted
     curQ , output ,  Curve after knot inserion
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvInsertKnot
 (NL_CURVE    *curP,   // in : input Crv
  NL_PARAMETER u,      // in : knot param value to insert
  NL_INDEX     r,      // in : number of times to insert knot at param = u
  NL_CURVE    *curQ,   // out: output Crv, when InputCrv == OutputCrv, knot insertion is done in place
  NL_STACKS   *SP,     // in : curP's stack
  NL_STACKS   *SQ )    // in : curQ's stack
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvInsertKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, spn, mlt;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, alf, oma;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    N_CrvGetKnotVector( curP, &knt );

    /* Check parameter and get knot span */

    error = N_KnotVectorIsEndParam( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpanAndMult( knt, p, u, NL_LEFT, &spn, &mlt );

    if( error EQ NL_YES )
        NL_OUT;

    if( (mlt + r)GT p OR r LT 0 )
        NL_ERROR( NL_PAR_ERR );

    k = spn - p;

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        curA = *curP;

        error = N_AllocCrvArrays( curP, n + r, p, m + r, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n + r, p, m + r, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Get auxiliary control points */

    Rw = N_AllocCPt1dArray( p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p - mlt; i++ )
        N_CopyCPt( Pw[k + i], &Rw[i] );

    /* Save unaltered control points */

    for ( i = 0; i <= k; i++ )
        N_CopyCPt( Pw[i], &Qw[i] );

    for ( i = spn - mlt; i <= n; i++ )
        N_CopyCPt( Pw[i], &Qw[i + r] );

    /* Now insert the knot */

    for ( i = 1; i <= r; i++ )
    {
        k = spn - p + i;

        for ( j = 0; j <= p - i - mlt; j++ )
        {
            alf = (u - UP[k + j]) / (UP[spn + j + 1] - UP[k + j]);
            oma = 1.0 - alf;
            N_Combine2CPts( alf, Rw[j + 1], oma, Rw[j], &Rw[j] );
        }
        N_CopyCPt( Rw[0], &Qw[k] );
        N_CopyCPt( Rw[p - i - mlt], &Qw[spn + r - i - mlt] );
    }

    /* Load the remaining control points */

    for ( i = k + 1; i < spn - mlt; i++ )
        N_CopyCPt( Rw[i - k], &Qw[i] );

    /* Load the knot vector */

    for ( i = 0; i <= spn; i++ )
        UQ[i] = UP[i];

    for ( i = 1; i <= r; i++ )
        UQ[i + spn] = u;

    for ( i = spn + 1; i <= m; i++ )
        UQ[i + r] = UP[i];

    /* If insertion is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvInsertKnot */


/*******************************************************************//**


   DESCRIPTION:

     This tools routine  splits a NURBS  curve into two NURBS curves. 
     The split must be at an interior  parameter value. If the output 
     curves  are  initialized  to NULL, memory  to  store new control 
     points and knots is allocated. A typical calling example is: 

       NL_CURVE      cur, curL, curR;
       NL_PARAMETER  u;
       NL_STACKS     SG;
       ...
       (define cur, get u);
       ...
       N_CrvInitArrays(&curL);
       N_CrvInitArrays(&curR);
       N_CrvSplit(&cur,u,&curL,&curR,&SG);

     If memory is  available, curL and curR  are not initialized  and 
     the  routine assumes  that  memory  allocation  has   been done. 
     However,  it checks  for the  proper  amount  by looking  at the 
     highest indexes  in curL's  and curR's  knot vector  and polygon 
     objects.

     Note that splitting a curve near a knot will create a very small
     span, and may violate some multiplicity rules. A test has been added
     to remove unintended small differences from knot values.


   ACCESS:
   
     cur  , input  ,  NURBS curve to be split
     u    , input  ,  Parameter where curve is to be split
     curL , output ,  Left half of split curve, defined over [U[0],u]
     curR , output ,  Right   half  of   split  curve,  defined  over
                      [u,U[m]].
     SG   , input  ,  curL's and curR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvSplit( NL_CURVE *cur, NL_PARAMETER u, NL_CURVE *curL, NL_CURVE *curR, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvSplit");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, spn, mlt;

    NL_DEGREE p;

    NL_REAL *U, *UL, *UR, alf, oma, PTol;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Lw, *Rw, *Sw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Check parameter u */
    /* if u is within PTol of knot, set u to knot value */
    /* NL_PTOL is far too small to use here */
    PTol = (U[m - p] - U[p]) * 1.0E-6;

    for ( i = p; i <= m - p; i++ )
    {
        if( fabs( U[i] - u ) < PTol )
        {
            u = U[i];
            break;
        }
    }

    /* Check parameter for domain */
    N_CrvGetKnotVector( cur, &knt );
    error = N_KnotVectorIsEndParam( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get span, and knot multiplicity */

    error = N_BasisFindSpanAndMult( knt, p, u, NL_LEFT, &spn, &mlt );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed */

    error = N_CrvSizeArrays( curL, spn - mlt, p, spn - mlt + p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curL, &Lw, &UL );

    error = N_CrvSizeArrays( curR, n + p - spn, p, n - spn + 2 * p + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curR, &Rw, &UR );

    /* Get auxiliary control points */

    Sw = N_AllocCPt1dArray( p, &SL );

    if( Sw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p - mlt; i++ )
        N_CopyCPt( Pw[spn - p + i], &Sw[i] );

    /* Save unaltered control points */

    for ( i = 0; i <= spn - p; i++ )
        N_CopyCPt( Pw[i], &Lw[i] );

    for ( i = spn - mlt; i <= n; i++ )
        N_CopyCPt( Pw[i], &Rw[i + p - spn] );

    /* Now split the curve */

    for ( i = 1; i <= p - mlt; i++ )
    {
        k = spn - p + i;

        for ( j = 0; j <= p - i - mlt; j++ )
        {
            alf = (u - U[k + j]) / (U[spn + j + 1] - U[k + j]);
            oma = 1.0 - alf;
            N_Combine2CPts( alf, Sw[j + 1], oma, Sw[j], &Sw[j] );
        }
        N_CopyCPt( Sw[0], &Lw[k] );
        N_CopyCPt( Sw[p - i - mlt], &Rw[p - i - mlt] );
    }

    /* Load knot vectors */

    k = spn - mlt;

    for ( i = 0; i <= k; i++ )
        UL[i] = U[i];

    for ( i = 0; i <= p; i++ )
        UL[k + i + 1] = u;

    k = spn + 1;

    for ( i = 0; i <= p; i++ )
        UR[i] = u;

    for ( i = k; i <= m; i++ )
        UR[i - k + p + 1] = U[i];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvSplit */


/*******************************************************************//**


   DESCRIPTION:

     This tools routine decomposes a NURBS curve into segments that do 
     not contain G1 tangent discontinuities. When curP is initially G^{1} 
     continuous the curve is copied into the output and k is set to 0.
     MEMORY  TO STORE THE OUTPUT CURVES ARE ALLOCATED INSIDE THE ROUTINE. 
     This routine is very similar to N_CrvDecomposeContinuity
     A typical calling example is:

       NL_CURVE   curP, **curQ;
       NL_INDEX   i, k;
       NL_CPOINT  *Pw;
       NL_STACKS  SQ;
       ...
       (define curP);
       ...
       N_CrvSplitAtInteriorLines(&curP,&curQ,&k,&SQ);
       ...
       Pw = curQ[i]->pol->Pw;
       ...

     curQ[i], 0<=i<=k, is a pointer to the i-th curve.


   ACCESS:
   
     curP , input  ,  NURBS curve to be decomposed
     curQ , output ,  Array of curves  with no  knots of full multipli-
                      city
     k    , output ,  Highest index in curQ
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvSplitAtInteriorLines                                                   
 ( NL_CURVE   * curP,   /*  in : target curve                            */    
   NL_CURVE *** curQ,   /*  out: array of curves                         */    
   NL_INDEX   * k,      /*  out: highest index in output array of curves */    
   NL_STACKS  * SQ )    /*  in : new object context stack                */ 
{                                                                                                                                                       
    NL_FLAG error = NL_NO;                                                      
    NL_INDEX i, n, m, nSplits = 0, startIndex = 0;
    NL_DEGREE p;
    NL_REAL *UP;
    NL_REAL *SplitParams;
    NL_CPOINT *Pw;
    NL_CURVE ** curA;
    NL_POINT dl[2];
    NL_POINT dir;
    NL_REAL dirDiff = 0.0;
    NL_INDEX sameDirCnt = 0;

    /* Get local notation */
    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

	N_CrvDerivs( curP, UP[0], NL_LEFT, 1, dl );
	dir.x = dl[1].x;
	dir.y = dl[1].y;
	dir.z = dl[1].z;

    /* Start after multiple knots */
    i = p + 1;

    /* How do I know how many to allocate?? */
    SplitParams = N_AllocReal1dArray( 10, SQ );
    if( SplitParams EQ NULL )
        NL_QUIT;


    while( i LT m )
    {
        startIndex = i;

        /* Advance over multiple knots */
        while( i LT m AND UP[i] EQ UP[i + 1] )
            i++;

        /* Calc new derivative at this knot */
        N_CrvDerivs(curP, UP[i], NL_LEFT, 1, dl);
        
        /* If it is not the first, then compare derivatives */
        if( i GT p + 1 )
        {
            N_VectorsAngle( dir, dl[1], &dirDiff );

            /* if same direction AND first one at same dir, then split at last parameter */
            if( dirDiff LE 1.0 )
            {
                if( sameDirCnt == 0 && nSplits > 0 ) {
                    SplitParams[nSplits++] = UP[startIndex-1];
                }
                sameDirCnt++;
            }
            /* if not same dir, then split at this parameter */
            else 
            {
                if( sameDirCnt GT 0 ) {
                    SplitParams[nSplits++] = UP[startIndex-1];
                    sameDirCnt = 0;
                }
            }
        }
        

        dir = dl[1];

        i++;
    }

    /* Define output curves */
    curA = N_AllocArrayCrvPtrs( nSplits + 1, SQ );
    if( curA EQ NULL )
        NL_QUIT;

    /* Split curve into sections */
    for ( i = 0; i < nSplits; i++ )
    {
        NL_CURVE* curR = N_AllocCrv( SQ );
        N_CrvInitArrays( curR );

        curA[i] = N_AllocCrv( SQ );
        N_CrvInitArrays( curA[i] );

        error = N_CrvSplit( curP, SplitParams[i], curA[i], curR, SQ );

        curP = curR;
    }

    /* If last split then use right curve */
    curA[nSplits] = curP;

    *k = nSplits + 1;
    *curQ = curA;


    EXIT:

    return (error);

} /* end N_CrvSplitAtInteriorLines */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine decomposes a NURBS curve into line and arc
     segments that do not contain G1 tangent discontinuities. 
     When curP is initially G^{1} continuous the curve is copied into 
     the output and k is set to 0.
     MEMORY  TO STORE THE OUTPUT CURVES ARE ALLOCATED INSIDE THE ROUTINE. 

     A typical calling example is:

       NL_CURVE   curP, **curQ;
       NL_REAL    tol = 0.01;
       NL_INDEX   i, k;
       NL_CPOINT  *Pw;
       NL_STACKS  SQ;
       ...
       (define curP);
       ...
       N_CrvSplitIntoLinesAndArcs(&curP,tol,&curQ,&k,&SQ);
       ...
       Pw = curQ[i]->pol->Pw;
       ...

     curQ[i], 0<=i<=k, is a pointer to the i-th curve.


   ACCESS:
   
     curP , input  ,  NURBS curve to be decomposed
     tol  , input  ,  Line Tolerance 
     curQ , output ,  Array of curves  with no  knots of full multiplicity
     k    , output ,  Highest index in curQ
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvSplitIntoLinesAndArcs                                                   
 ( NL_CURVE   * curP,    /*  in : target curve                            */ 
   NL_REAL      lineTol, /*  in : Line Tolerance                          */ 
   NL_CURVE *** curQ,    /*  out: array of curves                         */    
   NL_INDEX   * k,       /*  out: highest index in output array of curves */    
   NL_STACKS  * SQ )     /*  in : new object context stack                */ 
{                                                                                                                                                       
    NL_CURVE ** curA;
    NL_INDEX nCurves;
    NL_CURVE** newCurves;
    NL_FLAG error = NL_NO;
    NL_INDEX ii, jj;
    NL_INDEX nSplitCurves;
	NL_CURVE** splitCurves;

	
    /* Initialize ouput */
    *k = -1;

    /* If degenerate then do nothing */
	if (N_CrvIsDegen(curP) == NL_TRUE)
		return( NL_YES ); 

	/* Split original curve at discontinuities */
	error = N_CrvDecomposeAtG1Continuity(curP, &newCurves, &nCurves, SQ);
	if (error EQ NL_YES)
		return( NL_NO );

    /* Define output curves */
    curA = N_AllocArrayCrvPtrs( nCurves + 1, SQ );
    if( curA EQ NULL )
        NL_QUIT;

    for( ii = 0; ii <= nCurves; ii++ )
	{
		/* Remove repeated control points */
        N_CrvReplaceEqualCPts(newCurves[ii], NL_MTOL); 

        
        N_CrvSplitAtInteriorLines( newCurves[ii], &splitCurves, &nSplitCurves, SQ );

		/* Check if this segment is a line */
        for( jj = 0; jj < nSplitCurves; jj++ )
        {

            if (N_CrvIsLine(splitCurves[jj], lineTol) == NL_TRUE) {
                curA[++*k] = N_AllocCrv( SQ );
                N_CrvInitArrays( curA[*k] );
                N_CrvCopy( splitCurves[jj], curA[*k], SQ );
		    }
            else {
                curA[++*k] = N_AllocCrv( SQ );
                N_CrvInitArrays( curA[*k] );
	            error = N_ApproxContinuousCrvWithArcs(splitCurves[jj], lineTol * 10, curA[*k], SQ);
            }
        }
	}

    *curQ = curA;

    EXIT:

    return (error);

} /* end N_CrvSplitIntoLinesAndArcs */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine performs  inverse knot insertion, i.e. given a
     point on  one of the polygon legs, it inserts a knot such that the
     given  point becomes  a new control  point. If the output curve is 
     initialized to NULL, memory to store new  control points and knots 
     is allocated. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_POINT   P;
       NL_STACKS  SP, SQ;
       ...
       (define curP, get P);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvInverseKnotInsert(&curP,P,&curQ,&SP,&SQ);
       N_CrvInverseKnotInsert(&curP,P,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's 
     knot vector and polygon  objects. If curP is the same as curQ, the
     insertion is done in place.


   ACCESS:
   
     curP , input  ,  NURBS curve
     P    , input  ,  Point to become a new control point
     curQ , output ,  Curve after knot inserion
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvInverseKnotInsert( NL_CURVE *curP, NL_POINT P, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvInverseKnotInsert");

    NL_FLAG flg, error = NL_NO;

    NL_INDEX j, n;

    NL_DEGREE p;

    NL_REAL *U, dl, dr, u, t, w, w1;

    NL_POINT *R, Q;

    NL_CPOINT *Pw;

    NL_EPOLYGON ppl;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &j, &U );

    /* Get Euclidean polygon */

    error = N_CrvGetEPolygon( curP, 0, n, &ppl, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_EPolygonGetPts( &ppl, &j, &R );

    /* Get leg index */

    error = N_PolygonGetClosestLegIndex( &ppl, P, &Q, &j, &t, &flg );

    if( error EQ NL_YES )
        NL_OUT;

    if( flg EQ NL_FALSE )
        NL_ERROR( NL_INP_ERR );

    /* Get knot to be inserted */

    if( N_IsCrvRat( curP ) )
    {
        N_DistPtPt( R[j], Q, &dl );
        N_DistPtPt( Q, R[j + 1], &dr );
        N_CPtGetW( Pw[j], &w );
        N_CPtGetW( Pw[j + 1], &w1 );
        t = (w * dl) / (w * dl + w1 * dr);
    }

    u = U[j + 1] + t * (U[j + p + 1] - U[j + 1]);

    /* Insert the knot */

    error = N_CrvInsertKnot( curP, u, 1, curQ, SP, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvInverseKnotInsert */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  decomposes a  NURBS  curve  into its Bezier 
     constituents  without  using  knot   refinement.  Each  piece  is 
     represented as a  NURBS curve even though the segments are Bezier
     curves. MEMORY TO STORE THE OUTPUT CURVES IS ALLOCATED INSIDE THE
     ROUTINE. A typical calling example is:

       NL_CURVE   curP, **curQ;
       NL_INDEX   i, k;
       NL_CPOINT  *Pw;
       NL_STACKS  SQ;
       ...
       (define curP);
       ...
       N_CrvDecomposeBez(&curP,&curQ,&k,&SQ);
       ...
       Pw = curQ[i]->pol->Pw;
       ...

     curQ[i], 0<=i<=k, is a pointer to the i-th curve.


   ACCESS:
   
     curP , input  ,  NURBS curve to be decomposed
     curQ , output ,  Array of Bezier curves
     k    , output ,  Highest index in curQ
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvDecomposeBez /* eff: turn each span interval into a Bezier curve  */
 ( NL_CURVE   * curP,     /* in : target curve                                 */
   NL_CURVE *** curQ,     /* out: A Bezier curve for every input span interval */
   NL_INDEX   * k,        /* out: Highest index in Bezier curve array          */ 
   NL_STACKS  * SQ )      /* in : stack for new objects                        */
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s, nsp, mlt, is, ie, iq, save;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *alfs, *omas, num;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *NQw = NULL;

    NL_CURVE ** curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP,  /* in : target curve                                                    */
                               &n,     /* in : Highest index in control point array                            */
                               &Pw,    /* out: array of control points                                         */
                               &p,     /* out: curve degree                                                    */
                               &m,     /* out: Highest index in KnotArray                                      */
                               &UP );  /* out: array of knots (multiple knots are represented multiple times ) */
    N_CrvGetKnotVector( curP, &knt );

    /* Get number of segments and allocate memory */

    N_BasisGetSpanCount( knt, p, &nsp );                         /* nsp = number of spans in curP */

    curA = N_Alloc1dArrayCrvs( p, p, 2 * p + 1, nsp - 1, SQ );   /* curA = an array of NL_CURVES, highest index = nsp-1 */

    if( curA EQ NULL )
        NL_QUIT;

    /* locals */ 
    alfs = N_AllocReal1dArray( p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    is = p;        /* is = knot index of interval start */ 
    ie = p + 1;    /* ie = knot index of interval end */ 
    iq = -1;

    /* skip over first knot over-multiplicity  ( bad data correction ) */
    while( ie LT m AND UP[ie - 1]EQ UP[ie] )
    {
        is++;
        ie++;
    }

    N_CrvGetCPts( curA[0],  /* 1st output curve */ 
                 &i,        /* i = highest index in Qw */ 
                 &Qw );     /* Qw = 1st output curve control point array */ 

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Pw[i + is - p], &Qw[i] );

    /* Loop through the knot vector and extract each segment */

    while( ie LT m )
    {
        /* Initialize */

        iq = iq + 1;     /* iq = output curve index */

        if( iq >= nsp )
            NL_QUIT;

        N_CrvGetCPtsAndKnots( curA[iq],  /* current curve       */
                             &Qw,        /* Control Point array */
                             &UQ );      /* Knot Vector array   */

        if( iq LT nsp - 1 )
            N_CrvGetCPts( curA[iq + 1],  /* next curve                          */
                         &i,             /* higest index in control point array */
                         &NQw );         /* control point array                 */

        /* Get knot multiplicity */

        i = ie;

        while( ie LT m AND UP[ie]EQ UP[ie + 1] )
            ie++;
        mlt = ie - i + 1;                /* mlt = span end knot multiplicity */ 
        r = p - mlt;                     /* r   = knumber of knot insertions needed to make span end full multiplicity */

        /* Insert the knot on the rhs only */

        if( mlt LT p ) /* if r > 0 */
        {
            num = UP[ie] - UP[is];       /* num = current span param length */ 

            for ( i = p; i > mlt; i-- )
            {
                alfs[i - mlt - 1] = num / (UP[is + i] - UP[is]);
                omas[i - mlt - 1] = 1.0 - alfs[i - mlt - 1];
            }

            /* for every need top span knot insertions */ 
            for ( i = 1; i <= r; i++ )
            {
                s = mlt + i;
                save = r - i;

                for ( j = p; j >= s; j-- )
                {
                    N_Combine2CPts( alfs[j - s], /* alpha of Cw = alpha * Aw + beta * Bw */
                                      Qw[j],     /* Aw    of Cw = alpha * Aw + beta * Bw */
                                    omas[j - s], /* beta  of Cw = alpha * Aw + beta * Bw */
                                      Qw[j - 1], /* Bw    of Cw = alpha * Aw + beta * Bw */
                                     &Qw[j] );   /* Cw    of Cw = alpha * Aw + beta * Bw */
                }

                /* copy duplicated control points from this curve to next curve */ 
                if( ie LT m )
                {
                    N_CopyCPt( Qw[p], &NQw[save] );
                }
            }
        } /* end need to insert knots at interval end check */ 

        /* Build output Bezier span knot vector */
        /* ex for p = 3:[is is is is ie ie ie ie] */
        for ( i = 0; i <= p; i++ )
        {
            UQ[i] = UP[is];          
            UQ[i + p + 1] = UP[ie];
        }

        /* Segment completed - prepare for next piece, */ 
        /* copy remaining next ControlPoints (not already set from knot insertion) from original curve */

        if( ie LT m )
        {
            if( r < 0 )
                r = 0;

            for ( i = r; i <= p; i++ )
                N_CopyCPt( Pw[ie - p + i], &NQw[i] );
        }
        is = ie;
        ie = ie + 1;
    } /* end while orig curve span intervals left to process */ 

    *k = nsp - 1;
    *curQ = curA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvDecomposeBez */

#ifdef USE_NLIB_TESS

/*******************************************************************//**


   DESCRIPTION:

     This tools routine decomposes a NURBS  curve into segments that do 
     not contain knots with degree-fold mutiplicity, i.e. each piece is 
     at least  C^{1} continuous. When curP is initially C^{1} continuous the
     curve is copied into the output and k is set to 0.
     MEMORY  TO STORE THE OUTPUT  CURVES IS 
     ALLOCATED INSIDE THE ROUTINE. A typical calling example is:

       NL_CURVE   curP, **curQ;
       NL_INDEX   i, k;
       NL_CPOINT  *Pw;
       NL_STACKS  SQ;
       ...
       (define curP);
       ...
       N_CrvDecomposeContinuity(&curP,&curQ,&k,&SQ);
       ...
       Pw = curQ[i]->pol->Pw;
       ...

     curQ[i], 0<=i<=k, is a pointer to the i-th curve.


   ACCESS:
   
     curP , input  ,  NURBS curve to be decomposed
     curQ , output ,  Array of curves  with no  knots of full multipli-
                      city
     k    , output ,  Highest index in curQ
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvDecomposeContinuity                                                   
 ( NL_CURVE   * curP,   /*  in : target curve                                   */    
   NL_CURVE *** curQ,   /*  out: array of curves (no knots of full multiplicity */    
   NL_INDEX   * k,      /*  out: highest index in output array of curves        */    
   NL_STACKS  * SQ )    /*  in : new object context stack                       */ 
{                                                                               
                                                                                
    NL_FLAG error = NL_NO;                                                      

    NL_INDEX *ks, *ke, *ns, *ne, i, j, l, n, m, r, ie;

    NL_DEGREE p;

    NL_REAL *UP, *UQ;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE ** curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* Get number of segments */

    r = n / p + 1;

    ks = N_AllocInt1dArray( 4 * (r + 1), &SL );

    if( ks EQ NULL )
        NL_QUIT;

    ke = &ks[r + 1];
    ns = &ke[r + 1];
    ne = &ns[r + 1];

    r = -1;
    ie = p + 1;
    ns[0] = 0;
    ks[0] = ie;

    while( ie LT m )
    {
        i = ie;

        while( ie LT m AND UP[ie]EQ UP[ie + 1] )
            ie++;

        if( ie - i + 1 GE p )
        {
            r++;
            ne[r] = i - 1;
            ke[r] = i;

            if( ie LT m )
            {
                ns[r + 1] = ne[r];
                ks[r + 1] = ie + 1;
            }
        }

        ie++;
    }

    /* Define output curves */

    curA = N_AllocArrayCrvPtrs( r, SQ );

    if( curA EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= r; i++ )
    {
        n = ne[i] - ns[i];
        m = ke[i] - ks[i] + 2 * p + 1;

        curA[i] = N_AllocCrvAndArrays( n, p, m, SQ );

        if( curA[i]EQ NULL )
            NL_QUIT;

        N_CrvGetCPtsAndKnots( curA[i], &Qw, &UQ );

        for ( j = 0; j <= n; j++ )
            N_CopyCPt( Pw[ns[i] + j], &Qw[j] );

        l = 0;

        for ( j = 0; j <= p; j++ )
            UQ[j] = UP[ks[i] - 1];

        for ( j = p + 1; j <= n; j++ )
        {
            UQ[j] = UP[ks[i] + l];
            l++;
        }

        for ( j = n + 1; j <= m; j++ )
            UQ[j] = UP[ke[i]];
    }

    *k = r;
    *curQ = curA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvDecomposeContinuity */

#endif // USE_NLIB_TESS

/*******************************************************************//**


   DESCRIPTION:

     This tools routine decomposes a NURBS curve into segments that do 
     not contain G1 tangent discontinuities. When curP is initially G^{1} 
     continuous the curve is copied into the output and k is set to 0.
     MEMORY  TO STORE THE OUTPUT CURVES ARE ALLOCATED INSIDE THE ROUTINE. 
     This routine is very similar to N_CrvDecomposeContinuity
     A typical calling example is:

       NL_CURVE   curP, **curQ;
       NL_INDEX   i, k;
       NL_CPOINT  *Pw;
       NL_STACKS  SQ;
       ...
       (define curP);
       ...
       N_CrvDecomposeAtG1Continuity(&curP,&curQ,&k,&SQ);
       ...
       Pw = curQ[i]->pol->Pw;
       ...

     curQ[i], 0<=i<=k, is a pointer to the i-th curve.


   ACCESS:
   
     curP , input  ,  NURBS curve to be decomposed
     curQ , output ,  Array of curves  with no  knots of full multipli-
                      city
     k    , output ,  Highest index in curQ
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvDecomposeAtG1Continuity                                                   
 ( NL_CURVE   * curP,   /*  in : target curve                                   */    
   NL_CURVE *** curQ,   /*  out: array of curves (no knots of full multiplicity */    
   NL_INDEX   * k,      /*  out: highest index in output array of curves        */    
   NL_STACKS  * SQ )    /*  in : new object context stack                       */ 
{                                                                               
                                                                                
    NL_FLAG error = NL_NO;                                                      
    NL_INDEX *ks, *ke, *ns, *ne, i, j, l, n, m, r, ie;
    NL_DEGREE p;
    NL_REAL *UP, *UQ;
    NL_CPOINT *Pw, *Qw;
    NL_CURVE ** curA;
    NL_POINT dl[2];
    NL_POINT dr[2];
    NL_REAL angle = 0.0;
    NL_REAL AngleTolDeg = 1.0;
    NL_BOOLEAN IsClosed = NL_FALSE;
    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* Get number of segments */

    r = n / p + 1;

    ks = N_AllocInt1dArray( 4 * (r + 1), &SL );

    if( ks EQ NULL )
        NL_QUIT;

    ke = &ks[r + 1];
    ns = &ke[r + 1];
    ne = &ns[r + 1];

    r = -1;
    ie = p + 1;
    ns[0] = 0;
    ks[0] = ie;

    IsClosed = N_CrvIsClosed( curP );

    while( ie LT m )
    {
        i = ie;

        while( ie LT m AND UP[ie]EQ UP[ie + 1] )
            ie++;

        N_CrvDerivs(curP, UP[i], NL_LEFT, 1, dl);
        if( ie == m AND IsClosed )
        {
            N_CrvDerivs(curP, UP[0], NL_RIGHT, 1, dr);
        }
        else
        {
            N_CrvDerivs(curP, UP[ie], NL_RIGHT, 1, dr);
        }
        N_VectorsAngle( dl[1], dr[1], &angle );

        if( (ie - i + 1 GE p AND angle GT AngleTolDeg ) OR (ie GE m ) )
        {
            r++;
            ne[r] = i - 1;
            ke[r] = i;

            if( ie LT m )
            {
                ns[r + 1] = ne[r];
                ks[r + 1] = ie + 1;
            }
        }

        ie++;
    }

    /* Define output curves */

    curA = N_AllocArrayCrvPtrs( r, SQ );

    if( curA EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= r; i++ )
    {
        n = ne[i] - ns[i];
        m = ke[i] - ks[i] + 2 * p + 1;

        curA[i] = N_AllocCrvAndArrays( n, p, m, SQ );

        if( curA[i]EQ NULL )
            NL_QUIT;

        N_CrvGetCPtsAndKnots( curA[i], &Qw, &UQ );

        for ( j = 0; j <= n; j++ )
            N_CopyCPt( Pw[ns[i] + j], &Qw[j] );

        l = 0;

        for ( j = 0; j <= p; j++ )
            UQ[j] = UP[ks[i] - 1];

        for ( j = p + 1; j <= n; j++ )
        {
            UQ[j] = UP[ks[i] + l];
            l++;
        }

        for ( j = n + 1; j <= m; j++ )
            UQ[j] = UP[ke[i]];
    }

    *k = r;
    *curQ = curA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvDecomposeAtG1Continuity */

/*******************************************************************//**
   DESCRIPTION:

     This tools routine refines a NURBS curve with a given knot vector.
     It is assumed that the  new knot vector  "fits" into the  old one, 
     i.e. U[0] < X[0] <=...<= X[r] < U[m] holds where U[0],...,U[m] are
     the old knots and  X[0],...,X[r]  are the new  ones. If the output 
     curve is initialized to NULL, memory to store  new  control points 
     and knots is  allocated. If  the output  curve is  the same as the 
     input curve, knot refinement  is done  in place  and the  original 
     curve is destroyed. A typical calling example is:

       NL_CURVE       curP, curQ;
       NL_KNOTVECTOR  knx;
       NL_STACKS      SP, SQ;
       ...
       (define curP and knx);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRefine(&curP,&knx,&curQ,&SP,&SQ);
       N_CrvRefine(&curP,&knx,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's  
     knot vector and polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     knx  , input  ,  New knot vector
     curQ , output ,  Curve after knot refinement
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CrvRefine
 (NL_CURVE      *curP,  // in : NURBS curve
  NL_KNOTVECTOR *knx,   // in : ordered array of new knot values to insert into curP->Knt array
  NL_CURVE      *curQ,  // out: Curve after adding knots in knx to curP (ok if curP == curA)
  NL_STACKS     *SP,    // in : curP's stack
  NL_STACKS     *SQ )   // in : curQ's stack
{
  // locals
  NL_PRIVATE NL_STRING rname = _T("N_CrvRefine");
  NL_FLAG        error = NL_NO;
  NL_INDEX       i, j, k, l, a, b, n, m, r, t;
  NL_DEGREE      p;
  NL_REAL       *UP, *UQ, *X, alf, oma;
  NL_KNOTVECTOR *knt;
  NL_CPOINT     *Pw, *Qw;
  NL_CURVE       curA;
  NL_STACKS      SL;
  
  /* Start NURBS */
  N_InitNurbs( &SL );
  
  /* Get local notation */
  N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
  N_CrvGetKnotVector( curP, &knt );
  N_KnotVectorGetKnots( knx, &r, &X );
  
  /* Check input parameters  */
  if( r LT 0 )
      NL_ERROR( NL_INP_ERR );
  
  error = N_KnotVectorIsEndParam /* rtn: NL_YES = u is on or outside knot span, NL_NO = inside  */
            (knt,                /* in : Knotvector specifying span [1stKnot LastKnot] to check */
             X[0],               /* in : u parameter to classify (to machine precision)         */
             rname );            /* in : calling function's name label used for error reporting */
  
  if( error EQ NL_YES )
      NL_OUT;
  
  error = N_KnotVectorIsEndParam /* rtn: NL_YES = u is on or outside knot span, NL_NO = inside  */
            (knt,                /* in : Knotvector specifying span [1stKnot LastKnot] to check */
             X[r],               /* in : u parameter to classify (to machine precision)         */
             rname );            /* in : calling function's name label used for error reporting */
  
  if( error EQ NL_YES )
      NL_OUT;
  
  /* See if memory is needed */
  if( curP EQ curQ )
    {
      curA = *curP;  // copy input to make room for output
  
      error = N_AllocCrvArrays( curP, n + r + 1, p, m + r + 1, SP );
  
      if( error EQ NL_YES )
          NL_OUT;
  
      N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
  else // curP NE curQ - work directly in curQ memory
    {
      error = N_CrvSizeArrays( curQ, n + r + 1, p, m + r + 1, rname, SQ );
  
      if( error EQ NL_YES )
          NL_OUT;
  
      N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }
  
  /* Find knot spans */
  error = N_BasisFindSpan( knt, p, X[0], NL_LEFT, &a );
  
  if( error EQ NL_YES )
      NL_OUT;
  
  error = N_BasisFindSpan( knt, p, X[r], NL_LEFT, &b );
  
  if( error EQ NL_YES )
      NL_OUT;
  b++;
  
  /* Initialize output knot vector */
  for ( j = 0; j <= a; j++ )
      UQ[j] = UP[j];
  
  for ( j = b + p; j <= m; j++ )
      UQ[j + r + 1] = UP[j];
  
  /* Save unaltered control points */
  for ( j = 0; j <= a - p; j++ )
      N_CopyCPt( Pw[j], &Qw[j] );
  
  for ( j = b - 1; j <= n; j++ )
      N_CopyCPt( Pw[j], &Qw[j + r + 1] );
  
  /* Now refine the knot vector */
  i = b + p - 1;
  k = b + p + r;

  // for every knt vector knot
  for ( j = r; j >= 0; j-- )
    {
      while( X[j]LE UP[i]AND i GT a )
        {
          N_CopyCPt( Pw[i - p - 1], &Qw[k - p - 1] );
          UQ[k] = UP[i];
          k--;
          i--;
        }
  
      N_CopyCPt( Qw[k - p], &Qw[k - p - 1] );
  
      for ( l = 1; l <= p; l++ )
        {
          t = k - p + l;
          alf = UQ[k + l] - X[j];
  
          if( fabs( alf )LE 0.0 )
            {
              N_CopyCPt( Qw[t], &Qw[t - 1] );
            }
          else
            {
              alf = alf / (UQ[k + l] - UP[i - p + l]);
              oma = 1.0 - alf;
              N_Combine2CPts( alf, Qw[t - 1], oma, Qw[t], &Qw[t - 1] );
            }
        }
      UQ[k] = X[j];
      k--;
  }
  
  /* If refinement is in place, kill old curve */
  if( curP EQ curQ )
      N_FreeCrv( &curA, SP );
  
  /* End NURBS and Exit */
  EXIT:
  N_EndNurbs( &SL );

  // all done
  return (error);

} /* end N_CrvRefine */


/*******************************************************************//**


   DESCRIPTION:

     This tools routine extracts a  curve segment  from a NURBS curve. 
     The  segment is given by the parameters ul, ur of its end points. 
     The extraction is done via knot  insertion, i.e.  the  knots must 
     satisfy knot insertion requirements. The parameters  must satisfy
     U[0]<=ul and  ur<=U[m]. If  the  output  curve is  initialized to 
     NULL, memory to  store new control points and knots is allocated. 
     If the  output  curve  is  the  same as  the  input  curve, curve 
     extraction is done in place and the original curve  is destroyed. 
     A typical calling example is:

       NL_CURVE      curP, curQ;
       NL_PARAMETER  ul, ur;
       NL_STACKS     SP, SQ;
       ...
       (define curP, get ul and ur);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvExtractCrvSeg(&curP,ul,ur,&curQ,&SP,&SQ);
       N_CrvExtractCrvSeg(&curP,ul,ur,&curP,&SP,&SP);

     If memory is  available, curQ is  not initialized and the routine
     assumes that memory allocation has been done.  However, it checks  
     for the proper amount by looking at the highest indexes in curQ's 
     knot vector and polygon objects. 


   ACCESS:
   
     curP  , input  ,  NURBS curve
     ul,ur , input  ,  Left and right parameters defining the segment
     curQ  , output ,  Extracted curve
     SP    , input  ,  curP's stack
     SQ    , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvExtractCrvSeg( NL_CURVE *curP, NL_PARAMETER ul, NL_PARAMETER ur, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvExtractCrvSeg");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, ll, lk, lr, n, m, spl, mll, spr, mlr, is, ie;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, alf, oma, left;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE curA;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    N_CrvGetKnotVector( curP, &knt );

    /* Check parameters */

    if( ur LE ul )
        NL_ERROR( NL_INP_ERR );

    /* Find knot spans and set new indexes */

    error = N_BasisFindSpanAndMult( knt, p, ul, NL_LEFT, &spl, &mll );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpanAndMult( knt, p, ur, NL_LEFT, &spr, &mlr );

    if( error EQ NL_YES )
        NL_OUT;

    is = spl - p;

    if( ur EQ UP[m - p] )
    {
        spr = m;
        mlr = p + 1;
    }
    ie = spr - mlr;

    n = ie - is;
    m = spr - spl - mlr + 2 * p + 1;

    if( n < 0 )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        curA = *curP;

        error = N_AllocCrvArrays( curP, n, p, m, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Get initial control points */

    for ( i = is; i <= ie; i++ )
        N_CopyCPt( Pw[i], &Qw[i - is] );

    /* Insert the left knot */

    ll = spl - p;

    for ( i = 1; i <= p - mll; i++ )
    {
        for ( j = 0; j <= p - i - mll; j++ )
        {
            left = UP[ll + i + j];
            alf = (ul - left) / (UP[spl + j + 1] - left);
            oma = 1.0 - alf;
            N_Combine2CPts( alf, Qw[j + 1], oma, Qw[j], &Qw[j] );
        }
    }

    /* Insert the right knot */

    lr = spr - p;
    lk = n - p + mlr;

    for ( i = 1; i <= p - mlr; i++ )
    {
        for ( j = p - i - mlr; j >= 0; j-- )
        {
            k = lk + i + j;
            left = UP[lr + i + j];

            if( left LT ul )
                left = ul;

            alf = (ur - left) / (UP[spr + j + 1] - left);
            oma = 1.0 - alf;
            N_Combine2CPts( alf, Qw[k], oma, Qw[k - 1], &Qw[k] );
        }
    }

    /* Load the knot vector */

    j = -1;

    for ( i = 0; i <= p; i++ )
        UQ[++j] = ul;

    for ( i = spl + 1; i <= spr - mlr; i++ )
        UQ[++j] = UP[i];

    for ( i = 0; i <= p; i++ )
        UQ[++j] = ur;

    /* If insertion is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvExtractCrvSeg */

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine removes  one knot multiple  times from a NURBS 
     curve. The knot must be  an interior knot. If the  output curve is 
     initialized to NULL, memory to store new  control points and knots 
     is allocated. If the output curve is the same as the input  curve, 
     knot removal is done in place and the original curve is destroyed. 
     A typical calling example is:

       NL_CURVE      curP, curQ;
       NL_PARAMETER  u;
       NL_REAL       tol;
       NL_INDEX      nu, ru;
       NL_STACKS     SQ;
       ...
       (define curP, get u, nu and tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveKnot(&curP,u,nu,tol,&ru,&curQ,&SQ); 
       N_CrvRemoveKnot(&curP,u,nu,tol,&ru,&curP,&SQ); 

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects. 

   ACCESS:
   
     curP , input  ,  NURBS curve
     u,nu , input  ,  Knot "u" to be  removed "nu" times. If  nu > mlt,
                      the multiplicity of the knot, nu=mlt  is assumed.
     tol  , input  ,  Tolerance to check removability
     ru   , output ,  Number of knots removed
     curQ , output ,  Curve after knot removal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveKnot( NL_CURVE *curP, NL_PARAMETER u, NL_INDEX nu, NL_REAL tol, NL_INDEX *ru, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, ii, jj, first, last, off, n, m, spn, mlt, fout;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, alf, oma, bet, omb, lam, oml, del, omd, wmin, wmax, pmax, tmp, dw, maxl, maxr, max, lto, wi, wj;

    NL_KNOTVECTOR *knp, *knq;

    NL_CPOINT *Pw, *Qw, *Rw, A;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    N_CrvGetKnotVector( curP, &knp );

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
    }

    lto = cto * fabs( UP[m] - UP[0] );

    /* Check parameter and get knot span */
    error = N_KnotVectorIsEndParam( knp, u, rname );   /* NL_YES = on or outside KnotSpan, NL_NO = inside KnotSpan */

    /* don't remove boundary knots */
    if( error EQ NL_YES )
      { NL_OUT; }

    error = N_BasisFindSpanAndMult( knp, p, u, NL_LEFT, /* in: knp=KnotVector, p=degree, u=tgtParam, NL_Left=choose left spans over right */
                                   &spn, &mlt );        /* out: spn = Indx of last knot in span containing U, mlt=multiplicity of Knot[KnotIndex] */
    if( error EQ NL_YES )
      { NL_OUT; }

    if( nu GT mlt ) /* limit the number of knots removed at u to the number that are there */
      { nu = mlt; }

    /* See if memory is needed */

    if( curP EQ curQ ) /* Remove knots in place - no memory need */
      {
        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
        N_CrvGetKnotVector( curP, &knq );
      }
    else /* Remove knots from copy of curP in CurQ - allocate curQ memory */
      {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
        N_CrvGetKnotVector( curQ, &knq );
      }

    /* Get local memory */

    Rw = N_AllocCPt1dArray( 2 * p, &SL );  /* room for computing control point locations after knot removal */

    if( Rw EQ NULL )
      { NL_QUIT; }

    /* Initialize */

    if( curP NEQ curQ ) /* when asked, copy curP(CtrlPts and Knots) into curQ */
      {
        for ( i = 0; i <= n; i++ )
          { N_CopyCPt( Pw[i], &Qw[i] ); }

        for ( i = 0; i <= m; i++ )
          { UQ[i] = UP[i]; }
      }

    *ru = 0;

    /* Remove the knot "nu" times */

    fout = (2 * spn - mlt - p) / 2;
    first = spn - p;
    last = spn - mlt;

    /* for every knot to be removed */
    for(k=0;k<nu;k++)
      {
        off = first - 1;
        i   = first;
        j   = last;
        ii  = 1;
        jj  = last - off;

        N_CopyCPt( Qw[off], &Rw[0] );
        N_CopyCPt( Qw[last + 1], &Rw[last + 1 - off] );

        /* Get new control points for one removal step */

        alf = bet = 0.5;

        while( (j - i)GT k )
          {
            alf = (UQ[i + p + 1] - UQ[i]) / (u - UQ[i]);
            oma = 1.0 - alf;
            bet = (UQ[j + p - k + 1] - UQ[j - k]) / (UQ[j + p - k + 1] - u);
            omb = 1.0 - bet;
            N_Combine2CPts( alf, Qw[i], oma, Rw[ii - 1], &Rw[ii] );
            N_Combine2CPts( bet, Qw[j], omb, Rw[jj + 1], &Rw[jj] );
            i++;
            j--;
            ii++;
            jj--;
          }

        /* Check if knot is removable - won't change the shape of the curve by more than tol*/

        del = (u - UQ[i]) / (UQ[i + p + 1] - UQ[i]);
        omd = 1.0 - del;

        if( (j - i)LT k )
          {
            lam = alf / (alf + bet);
            oml = 1.0 - lam;
            N_DistCptCptHomo( Rw[ii - 1], Rw[jj + 1], &dw );

            error = N_BasisFindGlobalMax( knq, i - 1, p, lto, &maxl, &tmp );

            if( error EQ NL_YES )
              { maxl = 1.0; }

            error = N_BasisFindGlobalMax( knq, i, p, lto, &maxr, &tmp );

            if( error EQ NL_YES )
              { maxr = 1.0; }

            max = NL_MAX( lam * alf * maxl, oml * omd * maxr );

            if( max * dw GT tol )
              { break; }

            N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
          }
        else
          {
            N_Combine2CPts( del, Rw[jj + 1], omd, Rw[ii - 1], &A );
            N_DistCptCptHomo( Qw[i], A, &dw );

            error = N_BasisFindGlobalMax( knq, i, p, lto, &max, &tmp );  /* find ith CtrlPt's  max basis value */ 

            if( error EQ NL_YES ) /* when N_BasisFindGlobalMax() returns an error assume max == 1 */
              { max = 1.0; }

            if( max * dw GT tol ) /* don't move control points (to remove knots) if change moves the curve by more than tol */
              { break; }
          }

        /* Check for disallowed weights - weights out of the range [NL_WMIN NL_WMAX] */

        if( N_IsCrvRat( curP ) )
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
                break;
          }

        /* Save new control points */

        i = first;
        j = last;

        while( (j - i)GT k )
          {
            N_CopyCPt( Rw[i - off], &Qw[i] );
            N_CopyCPt( Rw[j - off], &Qw[j] );
            i++;
            j--;
          }

        /* Fill the hole in the note array by Shifting down higher value knots */
        for ( l = spn - k; l <= m - k - 1; l++ )
           { UQ[l] = UQ[l + 1]; }

        first--;
        last++;
      } /* end iter every knot to be removed */

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

    for ( l = i + 1; l <= n; l++ )
      {
        N_CopyCPt( Qw[l], &Qw[j] );
        j++;
      }

    *ru = k;

    /* Complete output curve's structure */

    N_CrvSetSizeIndices( curQ, n - k, p, m - k );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_CrvRemoveKnot */

#if NLIB_UNUSED

/*******************************************************************//**

   DESCRIPTION:

     This tools routine removes the coincident control points from a NURBS 
     curve where N_CrvReplaceEqualCPts neglects to do so. Currently it only 
     works on the start of the curve. N_CrvReplaceEqualCPts should be used 
     for the curve interior. Control point and necessary knot removal is 
     done in place. A typical calling example is:

       NL_CURVE      curP;
       ...
       (define curP);
       ...
       N_CrvRemoveDuplicateCPts(&curP); 
 

   ACCESS:
   
     curP , input/output  ,  NURBS curve

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveDuplicateCPts( NL_CURVE *curP, NL_REAL tol )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveDuplicateCPts"); */

    NL_FLAG error = NL_NO;

    NL_INDEX k, l, nCPts, nKnots, spn, mlt, nDupCPts = 0;

    NL_DEGREE deg;

    NL_REAL *UP, *UQ, dist;

    NL_KNOTVECTOR *knp;

    NL_CPOINT *Pw, *Qw;

    /* NL_REAL tol, length; */

    
    /* Get local notation */
    N_CrvGetCPtsDegreeAndKnots( curP, &nCPts, &Pw, &deg, &nKnots, &UP );
    N_CrvGetKnotVector( curP, &knp );

    /* find multiplicity at start */
    error = N_BasisFindSpanAndMult( knp, deg, UP[0], NL_LEFT, &spn, &mlt );
    if( error EQ NL_YES )
        NL_OUT;

    if( mlt LE 1 )
        NL_OUT;


    /* N_CrvArcLength( curP, UP[0], UP[nKnots], 1.0e-05, NL_RELATIVE, &length ); */
    /* tol  = length / 100; */

    /* make sure control points are the same */
    for( k = 0; k < mlt - 1; k++ )
    {
        N_DistCptCpt( Pw[k], Pw[k+1], &dist );
        if( dist <= tol )
            nDupCPts++;
    }

    if( nDupCPts == 0 || nCPts <= 3 )
        NL_OUT;
    
    N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );

    /* Remove the knot "nu" times */
    for ( k = 0; k < mlt - 1; k++ )
    {
        /* Shift down knots */
        for ( l = 1; l <= nKnots - k - 1; l++ )
            UQ[l] = UQ[l + 1];
    }

    /* If no knot was removed --> out */
    if( k EQ 0 )
        NL_OUT;

    /* rename start parameter to next parameter */
    UQ[0] = UQ[1];

    /* Shift down control points */
    for ( k = 0; k < mlt - 1; k++ )
    {
        for ( l = 1; l <= nCPts - k - 1; l++ )
            N_CopyCPt( Qw[l+1], &Qw[l] );
    }

    /* Complete output curve's structure */
    N_CrvSetSizeIndices( curP, nCPts - k, deg, nKnots - k );

    /* Exit */
    EXIT:

    return (error);
} /* end N_CrvRemoveDuplicateCPts */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine computes  the knot removal  error bound for one
     removal step. That is, given the index  "r" and multiplicity  "s",
     the knot  U[r]  is  removed  one time  and  the  maximum  error is 
     returned. It is  assumed  that (1) U[r] is  an  interior knot, (2) 
     U[r] != U[r+1], and (3) the multiplicity of the knot is "s > 0". A 
     typical calling example is:

       NL_CURVE   cur;
       NL_INDEX   r, s;
       NL_REAL    br;
       ...
       (define cur, get r and s);
       ...
       N_CrvRemoveKnotMaxErr(&cur,r,s,&br);

     THE ROUTINE DOES  NOT CHECK FOR THE PROPER NL_INDEX AND  MULTIPLICITY
     OF THE KNOT. IT ASSUMES THAT THEY ARE CORRECT.


   ACCESS:
   
     cur , input  ,  NURBS curve
     r   , input  ,  Index of  knot U[r] to be removed - U[r] != U[r+1] 
                     must hold.
     s   , input  ,  Multiplicity of U[r]
     br  , output ,  Maximum error after knot removal
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveKnotMaxErr( NL_CURVE *cur, NL_INDEX r, NL_INDEX s, NL_REAL *br )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, ii, jj;

    NL_DEGREE p;

    NL_REAL *U, alf, oma, bet, omb;

    NL_CPOINT *Pw, *Rw, A;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &i, &Pw, &p, &j, &U );

    /* Get local memory */

    Rw = N_AllocCPt1dArray( 2 * p, &S );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Compute removal error */

    i = r - p;
    j = r - s;
    ii = 1;
    jj = p - s + 1;

    N_CopyCPt( Pw[r - p - 1], &Rw[0] );
    N_CopyCPt( Pw[r - s + 1], &Rw[p - s + 2] );

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

    /* Compute error bound */

    if( (j - i)LT 0 )
    {
        N_DistCptCptHomo( Rw[ii - 1], Rw[jj + 1], br );
    }
    else
    {
        alf = (U[r] - U[i]) / (U[i + p + 1] - U[i]);
        oma = 1.0 - alf;

        N_Combine2CPts( alf, Rw[ii + 1], oma, Rw[ii - 1], &A );
        N_DistCptCptHomo( Pw[i], A, br );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvRemoveKnotMaxErr */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS curve. 
     If the  output curve is initialized to NULL, memory  to  store new  
     control points and knots is allocated. If the  output curve is the 
     same as the input  curve, knot  removal is  done in place  and the 
     original curve is destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SQ;
       ...
       (define curP, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveKnots(&curP,tol,&curQ,&SQ);
       N_CrvRemoveKnots(&curP,tol,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects. 

   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance to check removability
     curQ , output ,  Curve after knot removal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveKnots( NL_CURVE *curP, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveKnots");

    NL_FLAG rmf, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, fout, l, ns;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *br, *er, *te, *minl, *maxl, *minr, *maxr, *max, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, lto, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    ns = n;

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    lto = cto * fabs( UP[m] - UP[0] );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsKnotVectorAndKnots( curP, &Qw, &knt, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsKnotVectorAndKnots( curQ, &Qw, &knt, &UQ );
    }

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

    if( curP NEQ curQ )
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

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

        while( r LE n AND UQ[r]EQ UQ[r + 1] )
            r++;

        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( curP, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        b = br[p + 1];
        s = sr[p + 1];
        r = p + 1;

        for ( i = p + 2; i <= m - p - 1; i++ )
        {
            if( br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot -> finished */

        if( b EQ NL_BIGD OR b EQ NOREM )
            break;

        /* Check error of removal */

        rmf = NL_TRUE;

        if( (p + s) % 2 )
        {
            /* Compute maximums of basis functions over each span */

            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (UQ[r] - UQ[r - k]) / (UQ[r - k + p + 1] - UQ[r - k]);
            bet = (UQ[r] - UQ[r - k + 1]) / (UQ[r - k + p + 2] - UQ[r - k + 1]);
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
            if( UQ[i]NEQ UQ[i + 1] )
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
                if( UQ[i]NEQ UQ[i + 1] )
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

            N_CopyCPt( Qw[off], &Rw[0] );
            N_CopyCPt( Qw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (UQ[i + p + 1] - UQ[i]) / (UQ[r] - UQ[i]);
                oma = 1.0 - alf;
                bet = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Qw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Qw[j], omb, Rw[jj + 1], &Rw[jj] );
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
                    br[r] = NOREM;
                    continue;
                }
            }

            /* Save new control points */

            if( (p + s) % 2 )
            {
                N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
            }

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                N_CopyCPt( Rw[i - off], &Qw[i] );
                N_CopyCPt( Rw[j - off], &Qw[j] );
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
                UQ[i - 1] = UQ[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Qw[i], &Qw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( curQ, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( curQ, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        error = N_CrvCompress( curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveKnots */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS  curve
     with  point and  tangent constraints  applied at given parameters.
     If the  output curve is initialized to NULL, memory  to  store new  
     control points and knots is allocated. If the  output curve is the 
     same as the input  curve, knot  removal is  done in place  and the 
     original curve is destroyed. A typical calling example is:

       NL_FLAG       *cst;
       NL_CURVE      curP, curQ;
       NL_PARAMETER  *u;
       NL_INDEX      nu;
       NL_REAL       tol;
       NL_STACKS     SQ;
       ...
       (define curP, u, and cst; get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveKnotsParams(&curP,u,nu,cst,tol,&curQ,&SQ);
       N_CrvRemoveKnotsParams(&curP,u,nu,cst,tol,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects. 

   ACCESS:
   
     curP , input  ,  NURBS curve
     u    , input  ,  Parameter values at which constraints are applied
     nu   , input  ,  Highest index in u
     cst  , input  ,  cst[i] means:
                        NL_EPOINT    : constrain C(u[i])
                        NL_DERIVATIVE: constrain C'(u[i])
                        NL_BOTH      : constrain both
     tol  , input  ,  Tolerance to check removability
     curQ , output ,  Curve after knot removal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveKnotsParams( NL_CURVE *curP, NL_PARAMETER *u, NL_INDEX nu, NL_FLAG *cst, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveKnotsParams");

    NL_FLAG rmf, rem, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, fout, l, ns, lp, rp, lt, rt, left = 0, right = 0;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *br, *er, *te, *minl, *maxl, *minr, *maxr, *max, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, lto, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input and get local notation */

    for ( i = 0; i <= nu; i++ )
    {
        switch( cst[i] )
        {
            case NL_EPOINT:
                break;

            case NL_DERIVATIVE:
                break;

            case NL_BOTH:
                break;

            default:
                NL_ERROR( NL_INP_ERR );
        }
    }

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    ns = n;

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    lto = cto * fabs( UP[m] - UP[0] );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsKnotVectorAndKnots( curP, &Qw, &knt, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsKnotVectorAndKnots( curQ, &Qw, &knt, &UQ );
    }

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

    if( curP NEQ curQ )
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

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

        while( r LE n AND UQ[r]EQ UQ[r + 1] )
            r++;

        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( curP, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        b = br[p + 1];
        s = sr[p + 1];
        r = p + 1;

        for ( i = p + 2; i <= m - p - 1; i++ )
        {
            if( br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot -> finished */

        if( b EQ NL_BIGD OR b EQ NOREM )
            break;

        /* Check constraints */

        lp = NL_MAX( p, r - p - 1 );
        rp = NL_MIN( n + 1, r + p - s + 2 );
        lt = NL_MAX( p, r - p );
        rt = NL_MIN( n + 1, r + p - s + 1 );

        rem = NL_YES;

        for ( i = 0; i <= nu; i++ )
        {
            if( cst[i]EQ NL_EPOINT OR cst[i]EQ NL_BOTH )
            {
                left = lp;
                right = rp;
            }

            if( cst[i]EQ NL_DERIVATIVE )
            {
                left = lt;
                right = rt;
            }

            if( u[i]GT UQ[left]AND u[i]LT UQ[right] )
            {
                rem = NL_NO;
                break;
            }
        }

        if( rem EQ NL_NO )
        {
            br[r] = NOREM;
            continue;
        }

        /* Check error of removal */

        rmf = NL_TRUE;

        if( (p + s) % 2 )
        {
            /* Compute maximums of basis functions over each span */

            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (UQ[r] - UQ[r - k]) / (UQ[r - k + p + 1] - UQ[r - k]);
            bet = (UQ[r] - UQ[r - k + 1]) / (UQ[r - k + p + 2] - UQ[r - k + 1]);
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
            if( UQ[i]NEQ UQ[i + 1] )
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
                if( UQ[i]NEQ UQ[i + 1] )
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

            N_CopyCPt( Qw[off], &Rw[0] );
            N_CopyCPt( Qw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (UQ[i + p + 1] - UQ[i]) / (UQ[r] - UQ[i]);
                oma = 1.0 - alf;
                bet = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Qw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Qw[j], omb, Rw[jj + 1], &Rw[jj] );
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
                    br[r] = NOREM;
                    continue;
                }
            }

            /* Save new control points */

            if( (p + s) % 2 )
            {
                N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
            }

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                N_CopyCPt( Rw[i - off], &Qw[i] );
                N_CopyCPt( Rw[j - off], &Qw[j] );
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
                UQ[i - 1] = UQ[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Qw[i], &Qw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( curQ, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( curQ, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        error = N_CrvCompress( curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveKnotsParams */

#endif //NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine elevates the  degree of a NURBS curve from any
     degree to any higher degree. If the output curve is initialized to 
     NULL, memory to store new  control points and  knots is allocated. 
     If the  output  curve  is the  same  as  the  input  curve, degree 
     elevation is done in place and the  original  curve is  destroyed. 
     A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_INDEX   t;
       NL_STACKS  SP, SQ;
       ...
       (define curP, get t);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvElevateDegree(&curP,t,&curQ,&SP,&SQ);
       N_CrvElevateDegree(&curP,t,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the routine
     assumes  that memory allocation has been done. However, it checks  
     for the proper amount by looking at the highest indexes in curQ's  
     knot vector  and  polygon  objects.

   ACCESS:
   
     curP , input  ,  NURBS curve
     t    , input  ,  Increment (new degree is old_degree+t)
     curQ , output ,  Curve after degree elevation
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvElevateDegree( NL_CURVE *curP, NL_INDEX t, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvElevateDegree");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, np, mp, nq, mq, mlt, r, s, a, b, kind, cind, first, last, oldr, save, lbz, rbz;

    NL_DEGREE p, q;

    NL_REAL *UP, *UQ, *alfs, *omas, alf, oma, bet, omb, gam, omg, num, den;

    NL_RMATRIX dm;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Bw, *Nw, *Dw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &np, &Pw, &p, &mp, &UP );
    N_CrvGetKnotVector( curP, &knt );

    /* Check error */

    if( t LT 0 OR p + t GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    error = N_KnotVectorIsValid( knt, p, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed */

    N_BasisGetSpanCount( knt, p, &s );

    nq = np + t * s;
    q = (NL_DEGREE)(p + t);
    mq = mp + t * (s + 1);

    if( curP EQ curQ )
    {
        curA = *curP;

        error = N_AllocCrvArrays( curP, nq, q, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, nq, q, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* See if elevation is required */

    if( t EQ 0 )
    {
        for ( i = 0; i <= np; i++ )
        {
            N_CopyCPt( Pw[i], &Qw[i] );
        }

        for ( i = 0; i <= mp; i++ )
            UQ[i] = UP[i];

        NL_OUT;
    }

    /* Get local memory */

    Bw = N_AllocCPt1dArray( p, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt1dArray( p, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    Dw = N_AllocCPt1dArray( q, &SL );

    if( Dw EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    a = p;
    b = p + 1;
    r = -1;
    cind = 1;
    kind = q + 1;

    for ( i = 0; i <= q; i++ )
        UQ[i] = UP[a];

    N_CopyCPt( Pw[0], &Qw[0] );

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Pw[i], &Bw[i] );

    /* Get degree elevation matrix */

    N_InitRealMatrix( &dm );
    error = N_BezGetDegreeElevationMatrix( p, t, &dm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*************************************************************/
    /* Loop through the knot vector and do the following:        */
    /*   (1) Extract the i-th Bezier segment.                    */
    /*   (2) Degree elevate the segment.                         */
    /*   (3) Remove the knot between the i-th and the (i-1)-th   */
    /*       segment.                                            */
    /*************************************************************/

    while( b LT mp )
    {
        /* Get multiplicity of the knot */

        i = b;

        while( b LT mp AND UP[b]EQ UP[b + 1] )
            b++;

        mlt = b - i + 1;

        /* Insert knot to get Bezier segment */

        oldr = r;
        r = p - mlt;

        if( oldr GT 0 )
            lbz = (oldr + 2) / 2;
        else
            lbz = 1;

        if( r GT 0 )
            rbz = q - (r + 1) / 2;
        else
            rbz = q;

        if( r GT 0 )
        {
            /* Save the alphas */

            num = UP[b] - UP[a];

            for ( k = p; k > mlt; k-- )
            {
                alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
            }

            /* Now insert the knot */

            for ( j = 1; j <= r; j++ )
            {
                save = r - j;
                s = mlt + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfs[k - s], Bw[k], omas[k - s], Bw[k - 1], &Bw[k] );
                }
                N_CopyCPt( Bw[p], &Nw[save] );
            }
        } /* End of insert knot */

        /* Now degree elevate Bezier segment */

        error = N_BezElevateDegree( Bw, p, t, &dm, lbz, q, Dw );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot UP[a] */

        if( oldr GT 1 )
        {
            first = kind - 2;
            last = kind;
            den = UP[b] - UP[a];
            bet = (UP[b] - UQ[kind - 1]) / den;
            omb = 1.0 - bet;

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        alf = (UP[b] - UQ[i]) / (UP[a] - UQ[i]);
                        oma = 1.0 - alf;
                        N_Combine2CPts( alf, Qw[i], oma, Qw[i - 1], &Qw[i] );
                    }

                    if( j GE lbz )
                    {
                        if( (j - k)LE kind - q + oldr )
                        {
                            gam = (UP[b] - UQ[j - k]) / den;
                            omg = 1.0 - gam;
                            N_Combine2CPts( gam, Dw[l], omg, Dw[l + 1], &Dw[l] );
                        }
                        else
                        {
                            N_Combine2CPts( bet, Dw[l], omb, Dw[l + 1], &Dw[l] );
                        }
                    }
                    i++;
                    j--;
                    l--;
                }
                first--;
                last++;
            }
        } /* End of removing knot */

        /* Load knot vector and control points */

        if( a NEQ p )
        {
            for ( i = 0; i < q - oldr; i++ )
            {
                UQ[kind] = UP[a];
                kind++;
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            N_CopyCPt( Dw[i], &Qw[cind] );
            cind++;
        }

        /* Initialize for next pass through */

        if( b LT mp )
        {
            for ( i = 0; i < r; i++ )
                N_CopyCPt( Nw[i], &Bw[i] );

            for ( i = r; i <= p; i++ )
                N_CopyCPt( Pw[b - p + i], &Bw[i] );

            a = b;
            b++;
        }
        else
        {
            /* Get the end knot */

            for ( i = 0; i <= q; i++ )
                UQ[kind + i] = UP[b];
        }
    } /* End of while loop */

    /* If degree elevation is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvElevateDegree */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine reduces the degree of a NURBS curve by one. If
     the output curve  is initialized  to NULL,  memory to  store  new  
     control points and knots is allocated. If the output curve is the  
     same as  the input  curve, degree  reduction is done in place and 
     the original curve is destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol, mtol;
       NL_FLAG    rfl;
       NL_STACKS  SP, SQ;
       ...
       (define curP, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvReduceDegreeOnce(&curP,tol,&rfl,&curQ,&mtol,&SP,&SQ); 
       N_CrvReduceDegreeOnce(&curP,tol,&rfl,&curP,&mtol,&SP,&SP); 

     If memory is  available, curQ is not  initialized and the routine
     assumes  that memory allocation has been done. However, it checks  
     for the proper amount by looking at the highest indexes in curQ's  
     knot vector  and  polygon  objects.

   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance of degree reduction
     rfl  , output ,  Flag:
                        NL_YES : Reduction is successful
                        NL_NO  : Reduction is not successful
     curQ , output ,  Curve after degree reduction
     mtol , output ,  Maximum error over the curve
     SP   , input  ,  curP's memory stack
     SQ   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReduceDegreeOnce( NL_CURVE *curP, NL_REAL tol, NL_FLAG *rfl, NL_CURVE *curQ, NL_REAL *mtol, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvReduceDegreeOnce");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, k, l, np, mp, nq, mq, spn, mlt, r, s, a, b, c, kind, cind, first, last, oldr, save, lbz, ii, jj, kk, ll, oldmlt;

    NL_DEGREE p, q;

    NL_REAL *UP, *UQ, *X, *alfs, *omas, *dalf, *doma, *dbet, *domb, *e, *minl, *maxl, *minr, *maxr, *max, alf, oma, bet, omb, lam = 0.0, oml = 0.0, de, dw, tmp, num, wmin, pmax, den, lto, wi, wl;

    NL_KNOTVECTOR *knt, knx;

    NL_CPOINT *Pw, *Qw, *Bw, *Nw, *Dw, A;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &np, &Pw, &p, &mp, &UP );
    N_CrvGetKnotVector( curP, &knt );

    /* Initialize curA */
    N_CrvInitArrays( &curA );

    if( p LE 1 )
    {
        *rfl = NL_NO;
        NL_OUT;
    }

    /* Adjust tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }
    else
    {
        rat = NL_NO;
    }

    lto = cto * fabs( UP[mp] - UP[0] );

    /* See if memory is needed */

    N_BasisGetSpanCount( knt, p, &s );

    nq = np - s;
    q = p - 1;
    mq = mp - s - 1;

    if( curP EQ curQ )
    {
        curA = *curP;

        error = N_AllocCrvArrays( curP, nq, q, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, nq, q, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Get local memory */

    Bw = N_AllocCPt1dArray( p, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt1dArray( p, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    Dw = N_AllocCPt1dArray( q, &SL );

    if( Dw EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    dalf = N_AllocReal1dArray( p, &SL );

    if( dalf EQ NULL )
        NL_QUIT;

    doma = N_AllocReal1dArray( p, &SL );

    if( doma EQ NULL )
        NL_QUIT;

    dbet = N_AllocReal1dArray( p, &SL );

    if( dbet EQ NULL )
        NL_QUIT;

    domb = N_AllocReal1dArray( p, &SL );

    if( domb EQ NULL )
        NL_QUIT;

    X = N_AllocReal1dArray( 3 * p + 2, &SL );

    if( X EQ NULL )
        NL_QUIT;

    e = N_AllocReal1dArray( mp, &SL );

    if( e EQ NULL )
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

    /* Get degree reduction coefficients */

    N_BezDegreeReduceCoefs( p, dalf, doma, dbet, domb );

    /* Initialize */

    a = p;
    b = p + 1;
    r = -1;
    spn = p + 1;
    cind = 1;
    kind = q + 1;
    mlt = p + 1;
    *rfl = NL_YES;
    *mtol = 0.0;

    for ( i = 0; i <= q; i++ )
        UQ[i] = UP[a];

    for ( i = 0; i < mp; i++ )
        e[i] = 0.0;

    N_CopyCPt( Pw[0], &Qw[0] );

    for ( i = 0; i <= p; i++ )
    {
        N_CopyCPt( Pw[i], &Bw[i] );
    }

    /*************************************************************/
    /* Loop through the knot vector and do the following:        */
    /*   (1) Extract the i-th Bezier segment.                    */
    /*   (2) Degree reduce the segment.                          */
    /*   (3) Remove the knot between the i-th and the (i-1)-th   */
    /*       segment.                                            */
    /*************************************************************/

    while( b < mp )
    {
        /* Get multiplicity of the knot */

        i = spn;

        while( spn LT mp AND UP[spn]EQ UP[spn + 1] )
            spn++;

        oldmlt = mlt;
        mlt = spn - i + 1;

        /* Insert knot to get Bezier segment */

        b = b + mlt - 1;
        oldr = r;
        r = p - mlt;

        if( oldr GT 0 )
            lbz = (oldr + 2) / 2;
        else
            lbz = 1;

        if( r GT 0 )
        {
            /* Save the alphas */

            num = UP[b] - UP[a];

            for ( k = p; k > mlt; k-- )
            {
                alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
            }

            /* Now insert the knot */

            for ( j = 1; j <= r; j++ )
            {
                save = r - j;
                s = mlt + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfs[k - s], Bw[k], omas[k - s], Bw[k - 1], &Bw[k] );
                }
                N_CopyCPt( Bw[p], &Nw[save] );
            }
        } /* End of insert knot */

        /* Now degree reduce Bezier segment */

        error = N_BezReduceDegree( Bw, p, dalf, doma, dbet, domb, Dw, &de );

        if( error EQ NL_YES )
            NL_OUT;

        e[a] = e[a] + de;

        if( e[a]GT *mtol )
            *mtol = e[a];

        if( e[a]GT tol )
        {
            *rfl = NL_NO;
            break;
        }

        /* Remove the knot UP[a] */

        if( oldr GT 0 )
        {
            first = kind;
            last = kind;
            den = UP[b] - UP[a];

            for ( k = 0; k < oldr; k++ )
            {
                i = first;
                j = last;
                l = j - kind;

                while( (j - i)GT k )
                {
                    alf = (UP[b] - UQ[i - 1]) / (UP[a] - UQ[i - 1]);
                    oma = 1.0 - alf;
                    bet = (UP[b] - UQ[j - k - 1]) / den;
                    omb = 1.0 - bet;
                    N_Combine2CPts( alf, Qw[i - 1], oma, Qw[i - 2], &Qw[i - 1] );
                    N_Combine2CPts( bet, Dw[l], omb, Dw[l + 1], &Dw[l] );

                    if( rat EQ NL_YES )
                    {
                        N_CPtGetW( Qw[i - 1], &wi );
                        N_CPtGetW( Dw[l], &wl );

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

                /* Load the knot vector to compute precise error */

                kk = a + oldr - k;
                ll = kk - (2 * p - k + 1) / 2;
                jj = k % 2;
                c = -1;

                for ( ii = 0; ii <= p; ii++ )
                    X[++c] = UP[ll];

                for ( ii = ll + 1; ii <= a - oldmlt; ii++ )
                    X[++c] = UP[ii];

                for ( ii = 1; ii <= p - k; ii++ )
                    X[++c] = UP[a];

                for ( ii = kk - ll - jj; ii <= 2 *p; ii++ )
                    X[++c] = UP[b];

                N_KnotVectorFromRealArray( &knx, X, c );

                /* Compute the error */

                if( (j - i)LT k )
                {
                    alf = (UP[a] - UQ[i - 2]) / (UP[b] - UQ[i - 2]);
                    bet = (UP[a] - UQ[i - 1]) / (UP[b] - UQ[i - 1]);
                    omb = 1.0 - bet;
                    lam = alf / (alf + bet);
                    oml = 1.0 - lam;

                    N_DistCptCptHomo( Qw[i - 2], Dw[l + 1], &dw );

                    error = N_BasisFindAllSpanMaxima( &knx, p, p, lto, minl, maxl, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( ii = 0; ii <= p; ii++ )
                        {
                            minl[ii] = 0.0;
                            maxl[ii] = 1.0;
                        }
                    }

                    error = N_BasisFindAllSpanMaxima( &knx, p + 1, p, lto, minr, maxr, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( ii = 0; ii <= p; ii++ )
                        {
                            minr[ii] = 0.0;
                            maxr[ii] = 1.0;
                        }
                    }

                    max[0] = fabs( lam * alf * maxl[0] );

                    for ( ii = 1; ii <= p; ii++ )
                    {
                        minl[ii] *= lam * alf;
                        minr[ii - 1] *= oml * omb;
                        maxl[ii] *= lam * alf;
                        maxr[ii - 1] *= oml * omb;

                        max[ii] = NL_MAX( fabs( maxl[ii] - minr[ii - 1] ), fabs( maxr[ii - 1] - minl[ii] ) );
                    }
                    max[p + 1] = fabs( oml * omb * maxr[p] );
                }
                else
                {
                    alf = (UP[a] - UQ[i - 1]) / (UP[b] - UQ[i - 1]);
                    oma = 1.0 - alf;
                    N_Combine2CPts( alf, Dw[l + 1], oma, Qw[i - 2], &A );
                    N_DistCptCptHomo( Qw[i - 1], A, &dw );

                    error = N_BasisFindAllSpanMaxima( &knx, p, p, lto, minl, max, &tmp );

                    if( error EQ NL_YES )
                    {
                        for ( ii = 0; ii <= p; ii++ )
                            max[ii] = 1.0;
                    }
                }

                for ( ii = ll; ii <= a; ii++ )
                {
                    if( UP[ii]NEQ UP[ii + 1] )
                    {
                        e[ii] = e[ii] + dw * max[ii - ll];

                        if( e[ii]GT *mtol )
                            *mtol = e[ii];

                        if( e[ii]GT tol )
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
                    N_Combine2CPts( lam, Dw[l + 1], oml, Qw[i - 2], &Qw[i - 2] );
                }
                first--;
                last++;
            }

            if( *rfl EQ NL_NO )
                break;
            cind = i - 1;
        } /* End of removing knot */

        /* Load knot vector and control points */

        if( a NEQ p )
        {
            for ( i = 0; i < q - oldr; i++ )
            {
                UQ[kind] = UP[a];
                kind++;
            }
        }

        for ( i = lbz; i <= q; i++ )
        {
            N_CopyCPt( Dw[i], &Qw[cind] );
            cind++;
        }

        /* Initialize for next pass through */

        if( b LT mp )
        {
            for ( i = 0; i < r; i++ )
                N_CopyCPt( Nw[i], &Bw[i] );

            for ( i = r; i <= p; i++ )
                N_CopyCPt( Pw[b - p + i], &Bw[i] );

            a = b;
            b++;
            spn++;
        }
        else
        {
            for ( i = 0; i <= q; i++ )
                UQ[kind + i] = UP[b];
        }
    } /* End of while loop */

    /* If insertion is in place, kill old curve */

    if( curP EQ curQ )
    {
        if( *rfl EQ NL_YES )
        {
            N_FreeCrv( &curA, SP );
        }
        else
        {
            N_FreeCrv( curP, SP );
            *curP = curA;
        }
    }
    else
    {
        if( *rfl EQ NL_NO )
        {
            N_FreeCrv( curQ, SQ );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvReduceDegreeOnce */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine reduces the degree of a NURBS curve as much as
     possible. That is, it keeps reducing  the degree by one until the
     reduction error exceeds the tolerance. If the output curve is the  
     same as  the input  curve, degree  reduction is done in place and 
     the original curve is destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SP, SQ;
       ...
       (define curP, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvReduceDegree(&curP,tol,&curQ,&SP,&SQ);
       N_CrvReduceDegree(&curP,tol,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the routine
     assumes  that memory allocation has been done. However, it checks  
     for the proper amount by looking at the highest indexes in curQ's  
     knot vector  and  polygon  objects.

   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance of degree reduction
     curQ , output ,  Curve after degree reduction
     SP   , input  ,  curP's memory stack
     SQ   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReduceDegree( NL_CURVE *curP, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{

    NL_FLAG rfl, error = NL_NO;

    NL_REAL mtol, ctol;

    NL_CURVE *cur;

    NL_STACKS *S, SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Reduce the degree to as low as possible */

    ctol = 0.0;
    rfl = NL_TRUE;

    cur = curP;
    S = SP;

    if( curP NEQ curQ )
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        cur = curQ;
        S = SQ;
    }

    while( rfl EQ NL_TRUE AND ctol LT tol )
    {
        error = N_CrvReduceDegreeOnce( cur, tol - ctol, &rfl, cur, &mtol, S, S );

        if( error EQ NL_YES )
            NL_OUT;
        ctol += mtol;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvReduceDegree */


/**********************************************************************/
/* N_CrvsMakeCompatible: Make curves compatible                                   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine makes  a set of  curves  compatible, i.e. it 
     raises  the degrees and  inserts knots until all curves have the 
     same degree and are defined over the same knot vector. A typical 
     calling example is:

       NL_CURVE   **cur;
       NL_INDEX   k;       
       NL_REAL    KnotTol;
       NL_STACKS  S;
       ...
       (define array of cur);
       ...
       N_CrvsMakeCompatible(cur,k, KnotTol,&S); 

     THE  ALGORITHM   WORKS  IN  PLACE, I.E. THE  ORIGINAL CURVES ARE 
     DESTROYED! ALL CURVES MUST BELONG TO THE SAME STACK!

   ACCESS:
   
     cur , in/out ,  An array of NURBS curves
     k   , input  ,  Highest index in array
     KnotTol, input, Tolerance for same knot
     S   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatibleKnotTol
  ( NL_CURVE ** cur, 
    NL_INDEX k, 
    NL_REAL KnotTol, 
    NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, m, t;

    NL_DEGREE p, ph;

    NL_REAL *U, us, ue, u1, u2;

    NL_KNOTVECTOR ** knt, ** knx;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curve definitions compatible */

    if( !cur )
        NL_OUT;

    N_CrvsMakeRatCompatible( cur, k );

    /* If necessary, scale knot vectors to the unit interval */

    N_CrvGetParamBounds( cur[0], &us, &ue );

    /* remember if any curve's parambounds are not equal - by making sure that (i LE k) */
    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetParamBounds( cur[i], &u1, &u2 );
        /* if the curves are far apart - then scale the knot vectors below. */
        if( fabs( u1 - us ) > 1.0e-12 OR fabs( u2 - ue ) > 1.0e-12 )
        {
            break;
        }
    }

    /* when any curve's parambounds are not equal - scale all curves to the unit interval */
    if( i LE k ) 
    {
        for ( i = 0; i <= k; i++ )
            N_CrvReparamToInterval( cur[i], NL_UNITSPAN );
        /* Set the KnotTol for make compatible higher then 1.0e-8
           now that all curves are scaled to domain 0.0 - 1.0 */
        if( KnotTol == 0.0 )
            KnotTol = 0.001;
    }

    if( KnotTol == 0.0 ) /* if still not set */
    /* set KnotTol at 0.001 of curve domain */
    {
        KnotTol = (ue - us) / 1000.0;
    }

    /* Get highest degree */

    N_CrvGetDegree( cur[0], &ph );

    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetDegree( cur[i], &p );

        if( p GT ph )
            ph = p;
    }

    /* Elevate the degrees */

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetDegree( cur[i], &p );

        t = ph - p;

        if( t GT 0 )
        {
            error = N_CrvElevateDegree( cur[i], t, cur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Merge knot vectors */

    knt = N_Alloc1dArrayKnotVectPtrs( k, &SL );

    if( knt EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetKnotVector( cur[i], &knt[i] );
    }

    /* Note: do we want to do this?  N_GetCompatibleKnotArray() works with a hard-coded
     * tolerance of 1e-8, so this "if" just puts a lower limit on tol.
     * It effectively says "if (KnotTol LE 1.0e-8) KnotTol = 1.0e-8;"
     * Just calling N_GetCompatibleKnotVectorToTol() would remove the lower-limit check.
     * It makes sense to leave it though, because the 1e-8 knot tol
     * is found in several places.
     */
    if( KnotTol LE 1.0e-8 )
        error = N_GetCompatibleKnotArray( knt, k, &knx, &SL );
    else
        error = N_GetCompatibleKnotVectorToTol( knt, k, KnotTol, &knx, &SL );

    if( error EQ NL_YES )
        NL_QUIT;

    /* Refine curves */

    for ( i = 0; i <= k; i++ )
    {
        N_KnotVectorGetKnots( knx[i], &m, &U );

        if( m GE 0 )
        {
            error = N_CrvRefine( cur[i], knx[i], cur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvsMakeCompatibleKnotTol */

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine makes  a set of  curves  compatible, i.e. it 
     raises  the degrees and  inserts knots until all curves have the 
     same degree and are defined over the same knot vector. A typical 
     calling example is:

       NL_CURVE   **cur;
       NL_INDEX   k;
       NL_STACKS  S;
       ...
       (define array of cur);
       ...
       N_CrvsMakeCompatible(cur,k,&S); 

     THE  ALGORITHM   WORKS  IN  PLACE, I.E. THE  ORIGINAL CURVES ARE 
     DESTROYED! ALL CURVES MUST BELONG TO THE SAME STACK!

     Users should use N_CrvsMakeCompatibleKnotTol where the knot tolerance is supplied.
     Default compatible knot tolerance is (ur-ul)/1000 (from first curve).

   ACCESS:
   
     cur , in/out ,  An array of NURBS curves
     k   , input  ,  Highest index in array
     S   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatible( NL_CURVE ** cur, NL_INDEX k, NL_STACKS *S )
{
    NL_REAL KnotTol = 0.0;

    return (N_CrvsMakeCompatibleKnotTol( cur, k, KnotTol, S ));
} /* end N_CrvsMakeCompatible */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine eliminates all degenerate segments of a curve.
     The segment [u1,u2] is defined to  be  degenerate  if  its  image
     curve segment between C(u1) and C(u2) has length less  than  some
     given tolerance. If a segment is removed, parameterization of the 
     curve changes,  however,  the curve will not change geometrically
     more than the given tolerance. If the entire curve is degenerate,
     nothing is done (nr = 0). A typical calling example is:

       NL_CURVE     curP, curQ;
       NL_REAL      tol;
       NL_INDEX     nr;
       NL_STACKS    SP, SQ;
       ...
       (define curP and choose tol);
       ...
       N_CrvRemoveDegenSegs(&curP,tol,&nr,&curP,&SP,&SP);
       N_CrvInitArrays(&curQ);
       N_CrvRemoveDegenSegs(&curP,tol,&nr,&curQ,&SP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks
     for the proper amount  by looking at the highest indexes in curQ's  
     knot vector and polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance. A segment is removed if its arc length
                      is less than tol. tol should be very small;  i.e.
                      the function is designed to remove truely  degen-
                      erate segments.
     nr   , output ,  Number of segments removed. If nr = 0,  then curQ
                      is not created (curP is not modified).
     curQ , output ,  Curve after removal of degenerate segments  (only 
                      if nr > 0).
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveDegenSegs( NL_CURVE *curP, NL_REAL tol, NL_INDEX *nr, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveDegenSegs");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, np, mp, nq, mq, ns, mult1, mult2, k1, span1, span2, span3;

    NL_DEGREE p;

    NL_REAL dlen, du, u1, u2, d1, d2, d3, *UP, *UQ, ** segs;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE curA, curB, *curptr;

    NL_KNOTVECTOR *knt;

    NL_CPOLYGON *pol;

    NL_STACKS SL, *SS;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &np, &Pw, &p, &mp, &UP );

    *nr = 0;

    /* Only check curves with at least two segments */

    if( np LE p )
        NL_OUT;

    /* Find degeneracies */

    error = N_CrvGetDegenSegs( curP, tol, &segs, &ns, &SL );

    if( error EQ NL_YES OR ns LT 0 )
        NL_OUT;

    if( ns EQ 0 AND segs[0][0]EQ UP[0]AND segs[0][1]EQ UP[mp] )
        NL_OUT;

    /* There are degenerate segments to be removed */

    dlen = 0.00005 *( UP[mp] - UP[0] );

    /* Make copy of curP and work on the copy */
    /* Initialize curA */
    N_CrvInitArrays( &curA );

    if( curP EQ curQ )
    {
        curA = *curP;
        N_CrvInitArrays( curP );
        curptr = curP;
        error = N_CrvCopy( &curA, curptr, SP );

        if( error EQ NL_YES )
        {
            *curP = curA;
            NL_OUT;
        }
        SS = SP;
    }
    else if( N_CrvAreArraysNULL( curQ ) )
    {
        curptr = curQ;
        error = N_CrvCopy( curP, curptr, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        SS = SQ;
    }
    else
    {
        curptr = &curB;
        N_CrvInitArrays( curptr );
        error = N_CrvCopy( curP, curptr, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        SS = &SL;
    }

    /* Make segments to be removed into Beziers and reparameterize */
    /* so that their parameter lengths are small.                  */

    ii = 0;

    while( ii LE ns )
    {
        /* find number of contiguous degenerate segments */

        for ( jj = ii; jj < ns; jj++ )
            if( segs[jj][1]NEQ segs[jj + 1][0] )
                break;

        /* special case out contiguous segments equal entire curve */

        if( segs[ii][0]EQ UP[0]AND segs[jj][1]EQ UP[mp] )
        { /* just remove internal knots */
            error = N_CrvRemoveKnots( curptr, tol, curptr, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;

                NL_OUT;
            }
            else
            {
                *nr = ns + 1;
                goto WRAP_UP;
            }
        }

        /* make sure the segment ends are p-multiplicity knots */

        N_CrvGetKnotVector( curptr, &knt );
        error = N_BasisFindSpanAndMult( knt, p, segs[ii][0], NL_LEFT, &span1, &mult1 );

        if( mult1 LT p )
        {
            error = N_CrvInsertKnot( curptr, segs[ii][0], p - mult1, curptr, SS, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;

                NL_OUT;
            }
            N_CrvGetKnotVector( curptr, &knt );
        }

        error = N_BasisFindSpanAndMult( knt, p, segs[jj][1], NL_LEFT, &span2, &mult2 );

        if( mult2 LT p )
        {
            error = N_CrvInsertKnot( curptr, segs[jj][1], p - mult2, curptr, SS, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;

                NL_OUT;
            }
            N_CrvGetKnotVector( curptr, &knt );
        }

        N_CrvGetKnots( curptr, &mq, &UQ );

        du = segs[jj][1] - segs[ii][0];

        if( du GT dlen )              /* reparameterize */
        {
            if( segs[ii][0]EQ UQ[0] ) /* segment at start */
            {
                error = N_BasisFindSpan( knt, p, segs[jj][1], NL_RIGHT, &span2 );
                span2 += 1;

                u2 = UQ[0] + dlen;
                d2 = dlen / du;

                for ( kk = ii; kk < jj; kk++ )
                {
                    segs[kk][1] = d2 * (segs[kk][1] - UQ[0]) + UQ[0];
                    segs[kk + 1][0] = d2 * (segs[kk + 1][0] - UQ[0]) + UQ[0];
                }

                for ( kk = p + 1; kk < span2; kk++ )
                    UQ[kk] = d2 * (UQ[kk] - UQ[0]) + UQ[0];

                d3 = (UQ[mq] - u2) / (UQ[mq] - segs[jj][1]);

                for ( kk = jj + 1; kk <= ns; kk++ )
                {
                    segs[kk][0] = d3 * (segs[kk][0] - segs[jj][1]) + u2;

                    if( segs[kk][1]NEQ UQ[mq] )
                        segs[kk][1] = d3 * (segs[kk][1] - segs[jj][1]) + u2;
                }

                for ( kk = span2 + p; kk < mq - p; kk++ )
                    UQ[kk] = d3 * (UQ[kk] - UQ[span2]) + u2;

                segs[jj][1] = u2;

                for ( kk = 0; kk < p; kk++ )
                    UQ[span2 + kk] = u2;
            }
            else if( segs[jj][1]EQ UQ[mq] ) /* segment at end */
            {
                error = N_BasisFindSpan( knt, p, segs[ii][0], NL_LEFT, &span1 );

                u2 = UQ[mq] - dlen;
                d2 = dlen / du;

                for ( kk = ii; kk < jj; kk++ )
                {
                    segs[kk][1] = d2 * (segs[kk][1] - UQ[span1]) + u2;
                    segs[kk + 1][0] = d2 * (segs[kk + 1][0] - UQ[span1]) + u2;
                }

                for ( kk = span1 + 1; kk < mq - p; kk++ )
                    UQ[kk] = d2 * (UQ[kk] - UQ[span1]) + u2;

                d3 = (u2 - UQ[0]) / (UQ[span1] - UQ[0]);

                for ( kk = 0; kk < ii; kk++ )
                {
                    segs[kk][1] = d3 * (segs[kk][1] - UQ[0]) + UQ[0];

                    if( segs[kk][0]NEQ UQ[0] )
                        segs[kk][0] = d3 * (segs[kk][0] - UQ[0]) + UQ[0];
                }

                for ( kk = p + 1; kk <= span1 - p; kk++ )
                    UQ[kk] = d3 * (UQ[kk] - UQ[0]) + UQ[0];

                segs[ii][0] = u2;

                for ( kk = 0; kk < p; kk++ )
                    UQ[span1 - kk] = u2;
            }
            else
            { /* internal segment */
                error = N_BasisFindSpan( knt, p, segs[ii][0], NL_LEFT, &span1 );
                error = N_BasisFindSpan( knt, p, segs[jj][1], NL_RIGHT, &span2 );
                span2 += 1;

                d1 = 0.5 *( segs[ii][0] + segs[jj][1] );
                u1 = d1 - 0.499 *dlen;
                u2 = d1 + 0.499 *dlen;
                d2 = (u2 - u1) / du;

                for ( kk = ii; kk < jj; kk++ )
                {
                    segs[kk][1] = d2 * (segs[kk][1] - UQ[span1]) + u1;
                    segs[kk + 1][0] = d2 * (segs[kk + 1][0] - UQ[span1]) + u1;
                }

                for ( kk = span1 + 1; kk < span2; kk++ )
                    UQ[kk] = d2 * (UQ[kk] - UQ[span1]) + u1;

                d3 = (u1 - UQ[0]) / (UQ[span1] - UQ[0]);

                for ( kk = 0; kk < ii; kk++ )
                {
                    segs[kk][1] = d3 * (segs[kk][1] - UQ[0]) + UQ[0];

                    if( segs[kk][0]NEQ UQ[0] )
                        segs[kk][0] = d3 * (segs[kk][0] - UQ[0]) + UQ[0];
                }

                for ( kk = p + 1; kk <= span1 - p; kk++ )
                    UQ[kk] = d3 * (UQ[kk] - UQ[0]) + UQ[0];

                segs[ii][0] = u1;

                for ( kk = 0; kk < p; kk++ )
                    UQ[span1 - kk] = u1;

                d3 = (UQ[mq] - u2) / (UQ[mq] - segs[jj][1]);

                for ( kk = jj + 1; kk <= ns; kk++ )
                {
                    segs[kk][0] = d3 * (segs[kk][0] - segs[jj][1]) + u2;

                    if( segs[kk][1]NEQ UQ[mq] )
                        segs[kk][1] = d3 * (segs[kk][1] - segs[jj][1]) + u2;
                }

                for ( kk = span2 + p; kk < mq - p; kk++ )
                    UQ[kk] = d3 * (UQ[kk] - UQ[span2]) + u2;

                segs[jj][1] = u2;

                for ( kk = 0; kk < p; kk++ )
                    UQ[span2 + kk] = u2;
            }
        }

        ii = jj + 1;
    }

    /* Now remove the degenerate segments. There are  */
    /* two cases: interior segments vs. end segments. */

    N_CrvDetachPolygonKnot( curptr, &pol, &p, &knt );

    ii = 0;

    while( ii LE ns )
    {
        /* find number of contiguous degenerate segments */

        for ( jj = ii; jj < ns; jj++ )
            if( segs[jj][1]NEQ segs[jj + 1][0] )
                break;

        /* get local notation */

        N_CrvGetCPtsDegreeAndKnots( curptr, &nq, &Qw, &p, &mq, &UQ );

        /* now remove the segment */

        if( segs[ii][0]EQ UP[0] )
        { /* start segment */

            error = N_BasisFindSpan( knt, p, segs[jj][1], NL_RIGHT, &span2 );

            k1 = p + 1;

            for ( kk = span2 + p + 1; kk <= mq; kk++ )
                UQ[k1++] = UQ[kk];
            error = N_Realloc1dRealArray( &UQ, mq, k1 - 1, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;
                NL_OUT;
            }
            N_KnotVectorFromRealArray( knt, UQ, k1 - 1 );

            k1 = 1;

            for ( kk = span2 + 1; kk <= nq; kk++ )
                N_CopyCPt( Qw[kk], &Qw[k1++] );
            error = N_Realloc1dCPtArray( &Qw, nq, k1 - 1, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;
                NL_OUT;
            }
            N_CPolygonFromCPts( pol, Qw, k1 - 1 );
        }
        else if( segs[jj][1]EQ UP[mp] )
        { /* end segment */

            error = N_BasisFindSpan( knt, p, segs[ii][0], NL_LEFT, &span1 );
            span1 += 1;

            k1 = span1 - p;

            for ( kk = 0; kk <= p; kk++ )
                UQ[k1++] = UQ[mq];
            error = N_Realloc1dRealArray( &UQ, mq, span1, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;
                NL_OUT;
            }
            N_KnotVectorFromRealArray( knt, UQ, span1 );

            N_CopyCPt( Qw[nq], &Qw[nq - mq + span1] );
            error = N_Realloc1dCPtArray( &Qw, nq, nq - mq + span1, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;
                NL_OUT;
            }
            N_CPolygonFromCPts( pol, Qw, nq - mq + span1 );
        }
        else
        { /* interior segment */

            u2 = 0.5 *( segs[ii][0] + segs[jj][1] );
            error = N_BasisFindSpanAndMult( knt, p, u2, NL_LEFT, &span2, &mult2 );

            if( mult2 LT p )
            {
                error = N_CrvInsertKnot( curptr, u2, p - mult2, curptr, SS, SS );

                if( error EQ NL_YES )
                {
                    if( curptr EQ curP )
                        *curP = curA;

                    NL_OUT;
                }
                N_CrvDetachPolygonKnot( curptr, &pol, &p, &knt );
                N_CrvGetCPtsDegreeAndKnots( curptr, &nq, &Qw, &p, &mq, &UQ );
            }

            error = N_BasisFindSpan( knt, p, segs[ii][0], NL_LEFT, &span1 );
            error = N_BasisFindSpan( knt, p, u2, NL_LEFT, &span2 );
            error = N_BasisFindSpan( knt, p, segs[jj][1], NL_LEFT, &span3 );

            k1 = span1 - p + 1;

            for ( kk = 0; kk < p; kk++ )
                UQ[k1++] = u2;

            for ( kk = span3 + 1; kk <= mq; kk++ )
                UQ[k1++] = UQ[kk];
            error = N_Realloc1dRealArray( &UQ, mq, k1 - 1, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;
                NL_OUT;
            }
            N_KnotVectorFromRealArray( knt, UQ, k1 - 1 );

            k1 = span1 - p;
            N_CopyCPt( Qw[span2 - p], &Qw[k1++] );

            for ( kk = span3 - p + 1; kk <= nq; kk++ )
                N_CopyCPt( Qw[kk], &Qw[k1++] );
            error = N_Realloc1dCPtArray( &Qw, nq, k1 - 1, SS );

            if( error EQ NL_YES )
            {
                if( curptr EQ curP )
                    *curP = curA;
                NL_OUT;
            }
            N_CPolygonFromCPts( pol, Qw, k1 - 1 );
        }

        ii = jj + 1;
    }

    WRAP_UP:

    /* If removal is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* Copy result to curQ if memory was already there */

    if( curptr EQ & curB )
    {
        N_CrvGetArraySizes( curptr, &ii, &jj );
        error = N_CrvSizeArrays( curQ, ii, p, jj, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvCopy( curptr, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    *nr = ns + 1;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveDegenSegs */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine extracts a curve segment that crosses the  clo-
     sure of a closed curve.  It  is  assumed  that  the curve is  geo-
     metrically closed,  and  that  it is at least G0 continuous at the
     point of closure.  The resulting curve segment is parameterized on 
     [0,1],  and  the  new  curve's  parameter which corresponds to the
     point of closure is also returned. If the output curve is initial-
     ized to NULL, memory to store new control points and knots is all-
     ocated. If the output  curve  is  the  same as  the  input  curve,
     curve extraction is done in place and the original  curve  is  de-
     stroyed. A typical calling example is:

       NL_CURVE      curP, curQ;
       NL_PARAMETER  us, ue, uc;
       NL_REAL       tol;
       NL_STACKS     SP, SQ;
       ...
       (define curP, get us and ue, and choose tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvExtractSegClosed(&curP,us,ue,tol,&curQ,&uc,&SP,&SQ);
       N_CrvExtractSegClosed(&curP,us,ue,tol,&curP,&uc,&SP,&SP);

     If memory is  available, curQ is  not initialized and the routine
     assumes that memory allocation has been done.  However, it checks 
     for the proper amount by looking at the highest indexes in curQ's 
     knot vector and polygon objects. 


   ACCESS:
   
     curP  , input  ,  Closed NURBS curve
     us,ue , input  ,  Start and end parameters defining the segment to
                       be extracted.  Notice that either us<ue or us>ue
                       may hold,  and  that  the resulting segment runs
                       from C(us) to C(ue), across the closure point
     tol   , input  ,  Knot removal tolerance.  The extraction causes a
                       knot of multiplicity equal to the degree  to  be
                       placed at uc. Removal of this knot is attempted,
                       subject to this  tolerance  (knot removal is not
                       attempted if tol < 0.0)
     curQ  , output ,  Extracted curve
     uc    , output ,  C(uc) is the original point of closure on curQ
     SP    , input  ,  curP's stack
     SQ    , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvExtractSegClosed( NL_CURVE *curP, NL_PARAMETER us, NL_PARAMETER ue, NL_REAL tol, NL_CURVE *curQ, NL_PARAMETER *uc, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvExtractSegClosed");

    NL_FLAG error = NL_NO;

    NL_REAL dd, *U;

    NL_INDEX m, ii;

    NL_DEGREE p;

    NL_PARAMETER ul, ur, u1, u2;

    NL_CURVE curA, ** curs;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get curP's parameter bounds and check for errors */

    N_CrvGetParamBounds( curP, &ul, &ur );

    if( fabs( us - ue )LE NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    if( us EQ ul OR us EQ ur )
        NL_ERROR( NL_INP_ERR );

    if( ue EQ ul OR ue EQ ur )
        NL_ERROR( NL_INP_ERR );

    /* Allocate memory for the two curve pieces */

    curs = N_AllocArrayCrvPtrsAndData( 1, NL_YES, &SL );

    if( curs EQ NULL )
        NL_QUIT;

    /* Extract the two curve pieces, and reverse them if necessary */

    if( us LT ue )
    {
        error = N_CrvExtractCrvSeg( curP, ul, us, curs[0], SP, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvReverse( curs[0], curs[0], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvExtractCrvSeg( curP, ue, ur, curs[1], SP, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvReverse( curs[1], curs[1], &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_CrvExtractCrvSeg( curP, us, ur, curs[0], SP, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvExtractCrvSeg( curP, ul, ue, curs[1], SP, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Handle case input/output pointers the same */
    /* Initialize curA */
    N_CrvInitArrays( &curA );

    if( curP EQ curQ )
    {
        curA = *curP;
        N_CrvInitArrays( curP );
    }

    /* Set parameters of two curve pieces correctly */

    N_CrvGetParamBounds( curs[0], &ul, &ur );
    N_CrvGetParamBounds( curs[1], &u1, &u2 );

    dd = (ur - ul) / ((ur - ul) + (u2 - u1));
    *uc = dd;

    N_CrvReparam( curs[0], 0.0, dd );
    N_CrvReparam( curs[1], dd, 1.0 );

    /* Now composite the two curves */

    error = N_Iges102CompositeCrv( 1, curs, NL_IGES, 0.0, curQ, &SL, SQ );

    if( error EQ NL_YES )
    {
        if( curP EQ curQ )
            *curP = curA;
        NL_OUT;
    }

    /* Make sure that last knot is precisely 1.0 */

    N_CrvGetDegree( curQ, &p );
    N_CrvGetKnots( curQ, &m, &U );

    for ( ii = 0; ii <= p; ii++ )
        U[m - ii] = 1.0;

    /* Try to remove knot uc (=dd) */

    error = N_CrvRemoveKnot( curQ, dd, p, tol, &ii, curQ, SQ );

    if( error EQ NL_YES )
    {
        if( curP EQ curQ )
            *curP = curA;
        NL_OUT;
    }

    /* If extraction is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvExtractSegClosed */

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine makes  a set of  curves  compatible, i.e. it 
     raises  the degrees and  inserts knots until all curves have the 
     same degree and are defined over the same knot vector. It uses a
     tolerance to  adjust  knots if  they are  very close together. A 
     typical calling example is:

       NL_CURVE   **cur;
       NL_INDEX   k;
       NL_REAL    tol;
       NL_STACKS  S;
       ...
       (define array of cur and get tol);
       ...
       N_CrvsMakeCompatibleAdjKnots(cur,k,tol,&S); 

     THE  ALGORITHM   WORKS  IN  PLACE, I.E. THE  ORIGINAL CURVES ARE 
     DESTROYED! ALL CURVES MUST BELONG TO THE SAME STACK!

   ACCESS:
   
     cur , in/out ,  An array of NURBS curves
     k   , input  ,  Highest index in array
     tol , input  ,  Tolerance to check if two knots are the same
     S   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatibleAdjKnots( NL_CURVE ** cur, NL_INDEX k, NL_REAL tol, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvsMakeCompatibleAdjKnots");

    NL_FLAG error = NL_NO, scale = NL_NO;

    NL_INDEX i, m, t, kk;

    NL_DEGREE p, ph;

    NL_REAL *U, ul, ur, dd;

    NL_KNOTVECTOR ** knt, ** knx;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curve definitions compatible */

    N_CrvsMakeRatCompatible( cur, k );

    /* Scale knot vectors to the unit interval if necessary */

    N_CrvGetKnots( cur[0], &m, &U );
    ul = U[0];
    ur = U[m];

    dd = ur;

    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetKnots( cur[i], &m, &U );

        if( fabs( U[0] - ul )GT tol OR fabs( U[m] - ur )GT tol )
        {
            scale = NL_YES;
            break;
        }

        if( U[m]LT dd )
            dd = U[m];
    }

    if( scale EQ NL_YES )
    {
        for ( i = 0; i <= k; i++ )
        {
            N_CrvReparamToInterval( cur[i], NL_UNITSPAN );
        }

        dd = 1.0;
    }

    /* Get highest degree */

    N_CrvGetDegree( cur[0], &ph );

    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetDegree( cur[i], &p );

        if( p GT ph )
            ph = p;
    }

    /* Elevate the degrees */

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetDegree( cur[i], &p );

        t = ph - p;

        if( t GT 0 )
        {
            error = N_CrvElevateDegree( cur[i], t, cur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Ensure that the last span in all knot vectors */
    /* has length greater than tol                   */

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetKnots( cur[i], &m, &U );

        if( dd - U[m - ph - 1]LE tol )
        {
            /* must remove this knot */

            if( m EQ 2 * ph + 1 )
                NL_ERROR( NL_INP_ERR );

            error = N_CrvRemoveKnot( cur[i], U[m - ph - 1], ph, NL_BIGD, &kk, cur[i], S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Merge knot vectors */

    knt = N_Alloc1dArrayKnotVectPtrs( k, &SL );

    if( knt EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetKnotVector( cur[i], &knt[i] );
    }

    error = N_GetCompatibleKnotVectorToTol( knt, k, tol, &knx, &SL );

    if( error EQ NL_YES )
        NL_QUIT;

    /* Refine curves */

    for ( i = 0; i <= k; i++ )
    {
        N_KnotVectorGetKnots( knx[i], &m, &U );

        if( m GE 0 )
        {
            error = N_CrvRefine( cur[i], knx[i], cur[i], S, S );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvsMakeCompatibleAdjKnots */

/*******************************************************************//**


   DESCRIPTION:

     This  utility tool routine removes knots that have multiplicity
     greater than degree. This also deletes a control point for each knot 
     that is removed. When successive control points are equal and the curve 
     for that span is a line, the control points are reset so that the curve
     does not have a zero derivative at either end. The curve is then compacted 
     so as to use the minimal amount of memory.

       NL_CURVE   crv;
       NL_STACKS  SG;
       int     nRemoved;
       ...
(define curve);
       ...
       N_tooCrvCleanSpans(&crv, &nRemoved, &SG);

     SG MUST BE cur's STACK, i.e. ALL  MEMORY ALLOCATED FOR  crv, MUST 
     BE ON SG.


   ACCESS:
   
     crv , in/out ,  NURBS curve
     nRemoved, out , number of knots removed, = number of control points removed
     SG  , input  ,  cur's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_tooCrvCleanSpans( NL_CURVE *pCrv, int *nRemoved, NL_STACKS *pS )
{
    NL_FLAG error = NL_NO;
    NL_INDEX i, nPw, mU, nPwNew, mUNew;
    NL_INDEX iMin, iMax, mult, multUL, multUR, multCPtL, multCPtR;
    NL_DEGREE pDeg;
    NL_REAL *U, *UNew, u, distL, distR, alpha;
    NL_CPOINT *Pw, *PwNew, PwL, PwR, deltaPw;
    int nToRemove = 0;

    /* Get curve data */

    N_CrvGetCPtsDegreeAndKnots( pCrv, &nPw, &Pw, &pDeg, &mU, &U );

    /* Determine how many spans that need to be removed */
    /* If all the weights are equal set to NL_NOW */

    if( N_CrvAreWeightsEqual( pCrv ) )
        N_CrvMakeNonRat( pCrv );

    iMin = pDeg + 1;

    if( fabs( U[iMin] - U[iMin - 1] ) < NL_PTOL )
    {
        error = NL_YES;
        NL_OUT;
    }
    iMax = nPw + 1;

    if( fabs( U[iMax] - U[iMax - 1] ) < NL_PTOL )
    {
        error = NL_YES;
        NL_OUT;
    }
    i = iMin;

    while( i < iMax )
    {
        mult = 1;
        i++;
        u = U[i];

        while( i < iMax AND u == U[i] )
        {
            mult++;

            if( mult > pDeg )
                Pw[i].x = NL_NOZ; /* tag these for removeal */

            i++;
        }

        if( mult > pDeg )
            nToRemove += (mult - pDeg);
    }

    if( nToRemove > 0 )
    {
        NL_INDEX k;

        /* Allocate smaller control point and knot vector arrays */
        nPwNew = nPw - nToRemove;
        mUNew = mU - nToRemove;
        PwNew = N_AllocCPt1dArray( nPwNew, pS );

        if( PwNew EQ NULL )
            NL_QUIT;

        UNew = N_AllocReal1dArray( mUNew, pS );

        if( UNew EQ NULL )
            NL_QUIT;

        /* Copy knots and control points, removing the excess */

        k = 0;

        for ( i = 0; i <= nPw; i++ )
        {
            if( Pw[i].x != NL_NOZ )
            {
                UNew[k] = U[i];
                N_CopyCPt( Pw[i], &PwNew[k] );
                k++;
            }
        }

        for ( i = nPw + 1; i <= mU; i++ )
        {
            UNew[k] = U[i];
            k++;
        }

        *nRemoved = nToRemove;

        /* now look for what are really line segs and */
        /* separate the duplicate control points      */

        iMax = mUNew - pDeg;

        for ( i = pDeg; i < iMax; i++ )
        {                               /* check each non - zero span */
            if( UNew[i] < UNew[i + 1] ) /* span has length */
            {                           /* change only Bezier spans */
                multUL = multUR = 1;

                for ( k = 1; k < pDeg; k++ )
                {
                    if( UNew[i - k] == UNew[i] )
                        multUL += 1;

                    if( UNew[i + 1 + k] == UNew[i + 1] )
                        multUR += 1;
                }

                if( multUL == pDeg && multUR == pDeg )
                { /* this is a Bezier span */
                    multCPtL = multCPtR = 1;
                    PwL = PwNew[i - pDeg];
                    PwR = PwNew[i];

                    for ( k = 1; k < pDeg; k++ )
                    {
                        N_DistCptCptHomo( PwL, PwNew[i - pDeg + k], &distL );

                        if( distL < NL_PTOL )
                            multCPtL += 1;
                        N_DistCptCptHomo( PwR, PwNew[i - k], &distR );

                        if( distR < NL_PTOL )
                            multCPtR += 1;
                    }

                    if( multCPtL + multCPtR > pDeg )
                    { /* reset the intermediate cpoints */
                        N_Diff2CPts( PwR, PwL, &deltaPw );
                        alpha = 1.0 / (NL_REAL)pDeg;

                        for ( k = 1; k < pDeg; k++ )
                        {
                            N_Combine2CPts( 1.0, PwL, k * alpha, deltaPw, &PwNew[i - pDeg + k] );
                        }
                    }
                    else if( multCPtL + multCPtR == 3 ) /* for cubics only */
                    {
                        if( multCPtL == 2 )
                        {
                            N_Combine2CPts( 0.5, PwL, 0.5, PwNew[i - pDeg + 2], &PwNew[i - pDeg + 1] );
                        }

                        if( multCPtR == 2 )
                        {
                            N_Combine2CPts( 0.5, PwR, 0.5, PwNew[i - 2], &PwNew[i - 1] );
                        }
                    }
                }
            }
        }

        /* redefine curve and kill old memory */

        N_CrvSetCPtsAndKnots( pCrv, PwNew, UNew );
        N_CrvSetSizeIndices( pCrv, nPwNew, pDeg, mUNew );

        N_FreeCPt1dArray( Pw, pS );
        N_FreeReal1dArray( U, pS );
    }

    EXIT:
    return (error);
} /* end N_tooCrvCleanSpans */
