// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmNurbsCrv.cpp
* PURPOSE: Local copies of GW nurbs functions.
**********************************************************************/

#include "StdAfx.h"
#include <SmNurbsCrv.h>
#include <nurbs.h>
#include <SmBSplineCurve.h>

#ifndef __THREAD__
#define __THREAD__
#include <thread>
#endif

static SM_THREAD_LOCAL TCHAR* rname = NULL;

constexpr gw_REAL     WMIN   = 1.0e-03;  /* Minimum weight                  */
constexpr gw_REAL     WMAX   = 100.0;    /* Maximum weight                  */

#define InitCP(a,b) (b)->x = (a).x; (b)->y = (a).y; \
                    (b)->z = (a).z; (b)->w = (a).w; 

/*******************************************************************//**
PURPOSE: SmStackHandler constructor

NOTES:
***********************************************************************/
SmStackHandler::SmStackHandler
  (gw_STACKS * pStacks) 
{ 
  m_pStacks = pStacks; 
  N_InitNurbs(m_pStacks); 

} // end SmStackHandler::SmStackHandler default constructor

/*******************************************************************//**
PURPOSE: SmStackHandler destructor

NOTES:
***********************************************************************/
SmStackHandler::~SmStackHandler()                   
{ 
  N_EndNurbs(m_pStacks); 
  m_pStacks = NULL ;

} // end SmStackHandler::~SmStackHandler destructor


/*******************************************************************//**
PURPOSE: Extract the vectors from a reference frame as GW point
    and vectors.

NOTES: 
***********************************************************************/
void sm_ExtractFrame
 (const SmAxis2Placement & crReferenceFrame,
  NL_POINT               & rOrig, 
  NL_POINT               & rXAxis,
  NL_POINT               & rYAxis)
{
    const SmPoint3d  &crOrig  = crReferenceFrame.GetOriginRef();
    const SmVector3d &crXAxis = crReferenceFrame.GetXAxisRef();
    const SmVector3d &crYAxis = crReferenceFrame.GetYAxisRef();
    COPY_XYZ(crOrig, rOrig);
    COPY_XYZ(crXAxis,rXAxis);
    COPY_XYZ(crYAxis,rYAxis);

} // end sm_ExtractFrame

/*******************************************************************//**
PURPOSE: Given a gw_KNOTVECTOR extract the STEP compatible form where
    knots are unique and multiplicities are a separate list.

NOTES: When pOptIvl != NULL only knots contained in pOptIvl are
       added to output rKnots.  rKnots only contains pOptIvl end points
       if those end points happen to also be unique knot values.
***********************************************************************/
SmStatus sm_GetKnots
  (const gw_KNOTVECTOR * pKnt,                 // in : complete knot vector
   SmTArray<double>    & rKnots,               // out: Unique knot vector
   SmTArray<ULONG>     * pKnotMultiplicities,  // out: multiplicity value for each knot
   const SmExtent1d    * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
{
  // init output
  rKnots.ReSet();
  if (pKnotMultiplicities) pKnotMultiplicities->ReSet();

  // local
  gw_REAL* U = pKnt->U;

  // Copy unique knots and compute multiplicities
  if(pOptIvl == NULL || pOptIvl->ContainsValue(U[0]))
    { rKnots.Add(U[0]);
      if (pKnotMultiplicities) pKnotMultiplicities->Add(1);
    }

  // for every input knot but the first
  for (int j=1; j<=pKnt->m; j++) 
    {
      // when not checking intervals or when value is in the interval
      if(pOptIvl == NULL || pOptIvl->ContainsValue(U[j]))
        { 
          // if knot is a duplicate
          if(   rKnots.GetSize() > 0
             && SM_IS_ZERO(rKnots[rKnots.GetSize()-1] - U[j])) 
            {
              // when asked - count up multiplicities 
              if (pKnotMultiplicities) 
                {
                  (*pKnotMultiplicities)[rKnots.GetSize()-1] = 
                      (*pKnotMultiplicities)[rKnots.GetSize()-1] + 1;
                }
            }
          else // knot is unqiue
            {
              // add knot to output arrays
              rKnots.Add(U[j]);
              if (pKnotMultiplicities) pKnotMultiplicities->Add(1);
            }
         } // end interval check
    } // end iter every input knot

  // when asked only return

  // all done
  return SM_SUCCESS;

} // end sm_GetKnots

/*******************************************************************//**
PURPOSE: Given a gw_KNOTVECTOR extract the knots into a SmTArray.

NOTES: 
***********************************************************************/
SmStatus sm_GetKnotsAll
  (const gw_KNOTVECTOR * pKnt,                 // in : complete knot vector
   SmTArray<double>    & rKnots)               // out: complete knot vector
{
  // init output
  rKnots.ReSet();

  // local
  gw_REAL* U = pKnt->U;

  // Add each knot to the output array.
  for (int j=0; j<=pKnt->m; j++) 
    {
      rKnots.Add(U[j]);
    } // end iter every input knot

  // all done
  return SM_SUCCESS;

} // end sm_GetKnotsAll

/*******************************************************************//**
PURPOSE: Copy the data for a nurb curve.

NOTES: pTo is allocated prior to this call and is same size as
   cpFrom.
***********************************************************************/
SmStatus sm_CopyNurbCurve
  (const gw_CURVE *cpFrom,
         gw_CURVE *pTo)
{
  // set gw_CURVE size parameters and internal pointers
  sm_InitNurbCurveMemory(pTo,cpFrom->pol->n,cpFrom->p,cpFrom->knt->m);

  // copy the knot values - okay to use smos_MemCpy on base types (doubles)
  SER(smos_MemCpy(pTo->knt->U, cpFrom->knt->U, sizeof(gw_REAL)   * (pTo->knt->m + 1), sizeof(gw_REAL)   * (pTo->knt->m + 1)));

  // copy the control point values - okay to use smos_MemCpy on static class objects (gw_CPOINT)
  SER(smos_MemCpy(pTo->pol->Pw,cpFrom->pol->Pw,sizeof(gw_CPOINT) * (pTo->pol->n + 1), sizeof(gw_CPOINT) * (pTo->pol->n + 1)));

  // all done
  return SM_SUCCESS;

} // end sm_CopyNurbCurve

/*******************************************************************//**
PURPOSE: convenience function for sm_ComputeNurbCurveSize(args)

NOTES: 
***********************************************************************/
ULONG sm_ComputeNurbCurveSize
 (const gw_CURVE *cpCurve)
{
  // pass the call along
  return( cpCurve ? sm_ComputeNurbCurveSize(cpCurve->pol->n,
                                            cpCurve->knt->m)
                  : 0) ;

} // end sm_ComputeNurbCurveSize
   
/*******************************************************************//**
PURPOSE: Given highest knots index and highest cpoint index,
     compute the size in bytes of the resulting nurb curve.

NOTES: 
***********************************************************************/
ULONG sm_ComputeNurbCurveSize
 (gw_INDEX  lCpointHighestIndex, 
  gw_INDEX  lKnotsHighestIndex)
{
  ULONG lTotalSize =   ALIGN_SIZE(sizeof(gw_CURVE))  
                     + ALIGN_SIZE(sizeof(gw_CPOLYGON))  
                     + ALIGN_SIZE(sizeof(gw_KNOTVECTOR))  
                     + ALIGN_SIZE(sizeof(gw_REAL)   * (lKnotsHighestIndex+1))  
                     + ALIGN_SIZE(sizeof(gw_CPOINT) * (lCpointHighestIndex+1));
  return lTotalSize;

} // end sm_ComputeNurbCurveSize

/*******************************************************************//**
PURPOSE: Given a pointer to a proper sized block of memory, the degree, 
     the highest knots index, and the highest gw_CPOINT index -- initialize the 
     memory to be a Nurb curve.

NOTES: Note that the size of the memory block must be --
    ULONG lTotalSize =   sizeof(gw_CURVE) 
                       + sizeof(gw_CPOLYGON) 
                       + sizeof(gw_KNOTVECTOR)   
                       + sizeof(gw_REAL)   * (lKnotsHighestIndex+1)   
                       + sizeof(gw_CPOINT) * (lCpointHighestIndex+1);

***********************************************************************/
void sm_InitNurbCurveMemory
 (gw_CURVE  * pCurveMemory,
  gw_INDEX    lCpointHighestIndex, 
  gw_DEGREE   lDegree, 
  gw_INDEX    lKnotsHighestIndex)
{
  pCurveMemory->pol     =   (gw_CPOLYGON*)(((char*)pCurveMemory)   + ALIGN_SIZE(sizeof(gw_CURVE)));
  pCurveMemory->p       =   lDegree;
  pCurveMemory->knt     =   (gw_KNOTVECTOR*)(((char*)pCurveMemory) + ALIGN_SIZE(sizeof(gw_CURVE)) 
                                                                   + ALIGN_SIZE(sizeof(gw_CPOLYGON)));
  pCurveMemory->pol->n  =   lCpointHighestIndex;
  pCurveMemory->pol->Pw =   (gw_CPOINT*)(((char*)pCurveMemory)     + ALIGN_SIZE(sizeof(gw_CURVE)) 
                                                                   + ALIGN_SIZE(sizeof(gw_CPOLYGON))  
                                                                   + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)) 
                                                                   + ALIGN_SIZE(sizeof(gw_REAL) * (lKnotsHighestIndex+1)));
  pCurveMemory->knt->m =   lKnotsHighestIndex;
  pCurveMemory->knt->U =   (double*)(((char*)pCurveMemory)         + ALIGN_SIZE(sizeof(gw_CURVE)) 
                                                                   + ALIGN_SIZE(sizeof(gw_CPOLYGON)) 
                                                                   + ALIGN_SIZE(sizeof(gw_KNOTVECTOR)));
} // end sm_InitNurbCurveMemory

/*******************************************************************//**
PURPOSE: Given degree, highest knot index and hightest control points index,
    allocate the space required for a set of nurb curves as a single big block.
    Return a pointer to the block and set the output array of curves to
    point to each individual curve.

NOTES: Numerical values for the counts and the degree are 
    also initialized.  The lCpointHighestIndex and lKnotsHighestIndex 
    are the highest index values which are one less than the count.
***********************************************************************/
char *sm_AllocateBlockOfNurbCurves
 (gw_INDEX          lNumberOfCurves,
  gw_INDEX          lCpointHighestIndex, 
  gw_DEGREE         lDegree, 
  gw_INDEX          lKnotsHighestIndex, 
  SmTArray<void*> & rCurves)
{
  ULONG lTotalSize = sm_ComputeNurbCurveSize(lCpointHighestIndex,lKnotsHighestIndex);

  // okay to use smos_Calloc on static class (gw_CURVE, gw_CPOLYGON, gw_KNOTVECTOR,.. ) objects.
  char* pMemBlock = (char*) smos_Calloc((size_t)lNumberOfCurves*(size_t)lTotalSize, 1);
  NERN(pMemBlock);
  
  // Now load data from input curve
  rCurves.SetSize(lNumberOfCurves);
  for (long i=0; i<lNumberOfCurves; i++) 
    {
      gw_CURVE *pNewCur = SM_REINTERPRET_CAST(gw_CURVE*,&pMemBlock[i*lTotalSize]);
      rCurves[i] = SM_REINTERPRET_CAST(SmObject*,pNewCur);
      sm_InitNurbCurveMemory(pNewCur,lCpointHighestIndex,lDegree,lKnotsHighestIndex);
    }

  return pMemBlock;

} // end sm_AllocateBlockOfNurbCurves

/*******************************************************************//**
PURPOSE: Given degree, number of knots and number of control points 
    allocate and init the space required for a nurb curve as a single big mem block.

NOTES: Numerical values for the counts and the degree are 
    also initialized.  The lCpointHighestIndex and lKnotsHighestIndex 
    are the highest index values which are one less than the count.

MEMORY LAYOUT:  gw_CURVE
                    |     +-----------+ +-----+ 
                    V     |           | |     | 
                 +--------+-----------V-+-----V----------------+
                 | pol p knt   n Pw   m U   U_Block   Pw_Block |
                 +--+----------^--+----------------------^-----+
                    |          |  |                      | 
                    +----------+  +----------------------+
       gw_Curve memory block is allocated and freed in a single block
       internal pointers are set to mimic the NLib memory allocation structure
***********************************************************************/
gw_CURVE * sm_AllocateNurbCurve
 (gw_INDEX         lCpointHighestIndex, // in : Highest CptIndex of (Number ControlPoints = lCpointHighestIndex + 1)
  gw_DEGREE        lDegree,             // in : Curve Degree
  gw_INDEX         lKnotsHighestIndex)  // in : Highest KntIndex of (Number of Knots = lKnotsHighestIndex + 1) where (m = n + p + 1)
{
  ULONG lTotalSize = sm_ComputeNurbCurveSize(lCpointHighestIndex,lKnotsHighestIndex);

  // okay to use smos_Calloc on static class (gw_CURVE, gw_CPOLYGON, gw_KNOTVECTOR,.. ) objects.
  gw_CURVE * pNewCur = SM_REINTERPRET_CAST(gw_CURVE*, smos_Calloc(lTotalSize, 1));
  NERN(pNewCur);
  
  // set gw_Curve obj internal values and pointers
  sm_InitNurbCurveMemory(pNewCur,lCpointHighestIndex,lDegree,lKnotsHighestIndex);

  // all done
  return pNewCur;

} // end sm_AllocateNurbCurve

/*******************************************************************//**
PURPOSE: Allocate a nurb and copy the data from the source curve.
   The new curve is a single block of memory.  The source could be either
   a single block or one of the segmented things we get from the NLib.

NOTES: 
***********************************************************************/
gw_CURVE * sm_AllocateAndCopyNurbCurve
 (const gw_CURVE  *cpSrcCur)
{
  gw_INDEX    lCpointHighestIndex; 
  gw_CPOINT * paCpoint; 
  gw_DEGREE   lDegree;
  gw_INDEX    lKnotsHighestIndex; 
  gw_REAL   * paKnots;

  // locals
  N_CrvGetCPtsDegreeAndKnots(SM_CONST_CAST(gw_CURVE*,cpSrcCur),
                             &lCpointHighestIndex,
                             &paCpoint,
                             &lDegree,
                             &lKnotsHighestIndex,
                             &paKnots);

  // allocate mem on the stack
  gw_CURVE *pNewCur = sm_AllocateNurbCurve(lCpointHighestIndex,
                                           lDegree,
                                           lKnotsHighestIndex);

  // copy the Nurb values - okay to use smos_MemCpy on base (double) and static class (gw_CPOINT) objects
  SE(smos_MemCpy(pNewCur->knt->U, cpSrcCur->knt->U, sizeof(gw_REAL)   * (pNewCur->knt->m + 1), sizeof(gw_REAL)   * (pNewCur->knt->m + 1)));
  SE(smos_MemCpy(pNewCur->pol->Pw,cpSrcCur->pol->Pw,sizeof(gw_CPOINT) * (pNewCur->pol->n + 1), sizeof(gw_CPOINT) * (pNewCur->pol->n + 1)));

  // all done
  return pNewCur;

} // end sm_AllocateAndCopyNurbCurve    

/*******************************************************************//**
PURPOSE: Free a NurbCurve allocated by sm_AllocateNurbCurve or 
         sm_AllocateAndCopyNurbCurve

NOTES: All that's needed is smos_Free(pNurb).
       This function is here as a reminder not to call delete pNurb ;
***********************************************************************/
void sm_FreeNurbCurve
 (gw_CURVE * pNurb) // in : NurbCurve to be deallocated
{                                  
  if(pNurb) { smos_Free(pNurb) ; }

} // end sm_FreeNurbCurve

/*******************************************************************//**
PURPOSE: Edit knot values so close knots (knotSpacing<ScaledZero) are the same and 
near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by a tad more than sTol2d.

NOTES: Formerly this simply coalesced knots that were within
    tolerance, which caused problems when tol was on the order of the
    commonly-used hard-coded knot tolerance of 1e-8.  [T1000C; bd, 21 Dec 05]
***********************************************************************/
void sm_FixupKnotVector 
 (gw_KNOTVECTOR *pKnotVec,   // in : Target Knot Vector whose knots might be spread apart
  SmTol2d        sTol2d )    // in : Param Space min distance between distinct points
{
  ULONG   nKts = pKnotVec->m + 1;
  double *pKts = pKnotVec->U;

  double u0 = pKnotVec->U[0];
  double u1 = pKnotVec->U[nKts-1];

  SmScaledZero sScaledZero = SmTol::GetScaledZero(u0, u1) ;
                       // used to be: SM_EFF_ZERO * ( 1.0 + smos_Max(smos_Fabs(u0), smos_Fabs(u1))) ;

  // Spread knots out by a little more than the given tolerance.
  // This helps keep things neat if for example this routine is called
  // again with the same tol: the check for dDelta < sTol2d below
  // won't be ambiguous.
  double dIncrement = sTol2d * 1.01;
  double dDelta;

  // It will make the algorithm much easier if we first collect
  // unique knots.  Compress Knots closer than sScaledZero apart to have the same knot value

  SmTArray<double> vKts;
  SmTArray<ULONG>  vMults;

  ULONG i = 0;
  ULONG mult = 1;
  for ( i=0; i+1 < nKts; i++ )   // note: can't say nKts-1
    {
      dDelta = pKts[i+1] - pKts[i];  // note, no fabs(): non-decreasing.
      if ( dDelta < sScaledZero )
        {
          // same (multiple) knot
          mult++;
          pKts[i+1] = pKts[i];  // make it definite - Change Knot Vector here
        }
      else
        {
          vMults.Add( mult );
          vKts.Add( pKts[i] );
          mult = 1;
        }
    }
  vMults.Add( mult );
  vKts.Add( pKts[nKts-1] );

  // Now run the algorithm on the list of distinct knots.
  ULONG numDistinct = vKts.GetSize();

  for(i=0;i+1 < numDistinct;i++)  // note: can't say numDistinct-1
    {
      dDelta = vKts[i+1] - vKts[i];  // note, no fabs(): non-decreasing.

      if ( dDelta < sTol2d )
        {
          // Not meant to be the same, but too close.
          // Spread the knots away from their midpoint, for the least effect
          // on the curve, but don't adjust the start or end knots.

          if ( i < 1 )
              vKts[i+1] = vKts[i] + dIncrement;
          else if ( i >= numDistinct-2 )
              vKts[i] = vKts[i+1] - dIncrement;
          else
            {
              double mid = ( vKts[i] + vKts[i+1] ) / 2;
              vKts[i  ] = mid - dIncrement/2;
              vKts[i+1] = mid + dIncrement/2;
            }
        }
    }

  // Dump back into input knot vector
  ULONG j, l = 0;
  for ( i=0; i < numDistinct; i++ )
    {
      for ( j = 0; j < vMults[i]; j++ )
        {
          pKts[ l++ ] = vKts[ i ];
        }
    }

  SM_ASSERT( l == nKts );

} // end sm_FixupKnotVector

/*******************************************************************//**
PURPOSE: Create the Bezier curves corresponding to segments of a nurb.

NOTES: rCurves contains allready allocated pointers to nurbs
***********************************************************************/
gw_FLAG  sm_CreateBezSegments 
 (gw_CURVE            * curP, 
  SmTArray<gw_CURVE*> & rCurves)
{
  gw_FLAG        error = NL_NO;
  gw_INDEX       i, j, n, m, r, s, nsp, mlt, is, ie, iq, save;
  gw_DEGREE      p;
  gw_REAL        *UP, *UQ, alfs[GW_MAX_DEGREE], omas[GW_MAX_DEGREE], num;
  gw_KNOTVECTOR  *knt  = NULL;
  gw_CPOINT      *Pw = NULL, *Qw  = NULL, *NQw  = NULL;
  gw_CURVE       **curA;

  /* Get local notation */
  knt = curP->knt;
  N_CrvGetCPtsDegreeAndKnots(curP,&n,&Pw,&p,&m,&UP);

  /* Get number of non-zero segments */
  N_BasisGetSpanCount(knt,p,&nsp);

//  rCurves.SetSize(nsp);
//  for (i=0; i<nsp; i++) {
//      gw_CURVE *pCrv = sm_AllocateNurbCurve(p,p,2*p+1);
//      rCurves[i] = (SmObject*)pCrv;
//  }
  curA = &rCurves[0];  
//  curA = N_Alloc1dArrayCrvs(p,p,2*p+1,nsp-1,SQ);
//  if( curA EQ NULL )  NL_QUIT;

//  alfs = N_AllocReal1dArray(p,&SL);
//  if( alfs EQ NULL )  NL_QUIT;

//  omas = N_AllocReal1dArray(p,&SL);
//  if( omas EQ NULL )  NL_QUIT;

  /* Initialize */
  is = p;  ie = p+1;  iq = -1;

  /* skip over first knot over-multiplicity  ( bad data) */
  while( ie LT m  AND  UP[ie-1] EQ UP[ie] )  { is++;  ie++; }

  N_CrvGetCPts(curA[0],&i,&Qw);

  /* if we had bad starting knots move control point start on the next */
  for( i=0; i<=p; i++ )  N_CopyCPt(Pw[i + is - p],&Qw[i]);
  

   /* Loop through the knot vector and extract each segment */

  while( ie LT m )
  {
    /* Initialize */

    iq = iq+1;

    /* safety check...CurA array [0]..[nsp-1]  */
    if ( iq >= nsp) 
        return(error);
    Qw = curA[iq]->pol->Pw;
    UQ = curA[iq]->knt->U; 

    // check to make sure we are right size
    if (curA[iq]->pol->n != p) return 1;
    if (curA[iq]->p != p) return 1;
    if (curA[iq]->knt->m != 2*p+1) return 1;

    if( iq LT nsp-1 )  
    {
      NQw = curA[iq+1]->pol->Pw;
    }

    /* Get knot multiplicity */

    i=ie;
    while( ie LT m  AND  UP[ie] EQ UP[ie+1] )  ie++;
    mlt = ie-i+1;
    r   = p-mlt;

    /* If multiplicity < degree then insert the knot */

    if( mlt LT p )
    {
      num = UP[ie]-UP[is];
      for( i=p; i>mlt; i-- )
      {
        alfs[i-mlt-1] = num/(UP[is+i]-UP[is]);
        omas[i-mlt-1] = 1.0-alfs[i-mlt-1];
      }

      for( i=1; i<=r; i++ )
      {
        s    = mlt+i;
        save = r-i;
        for( j=p; j>=s; j-- )
        {
          N_Combine2CPts(alfs[j-s],Qw[j],omas[j-s],Qw[j-1],&Qw[j]);
        }
        if( ie LT m )
        {
          InitCP(Qw[p],&NQw[save]);
        }
      }
    }

    /* Get knot vector */

    for( i=0; i<=p; i++ )
    {
      UQ[i] = UP[is];  UQ[i+p+1] = UP[ie];
    }

    /* Segment completed - prepare for next piece */

    if( ie LT m )
    {       
      if (r < 0) 
          r = 0;
      for( i=r; i<=p; i++ )
      {
        InitCP(Pw[ie-p+i],&NQw[i]);
      }
    }
    is = ie;  ie = ie+1;
  }

  /* End NURBS and Exit */


  return(error);

} // end sm_CreateBezSegments

/*******************************************************************//**
PURPOSE: Split a nurb curve into two pieces.

NOTES: 
***********************************************************************/
gw_FLAG  sm_CrvSplit
 (gw_CURVE        * cur, 
  gw_PARAMETER      u, 
  gw_CURVE       *& curL, 
  gw_CURVE       *& curR, 
  gw_CURVE        * pExistingL, 
  gw_CURVE        * pExistingR)
{
  gw_FLAG        error = NL_NO;
  gw_INDEX       i, j, k, n, m, spn, mlt;
  gw_DEGREE      p;
  gw_REAL        *U, *UL, *UR, alf, oma;
  gw_KNOTVECTOR  *knt;
  gw_CPOINT      *Pw, *Lw, *Rw;
  gw_CPOINT      Sw[GW_MAX_DEGREE];

  /* Get local notation */
  knt = cur->knt;
  N_CrvGetCPtsDegreeAndKnots(cur,&n,&Pw,&p,&m,&U);

  /* Check parameter */
  error = N_KnotVectorIsEndParam(knt,u,_T("sm_CrvSplit"));
  if( error EQ NL_YES )  NL_OUT;

  /* Get span, and knot multiplicity */
  error = N_BasisFindSpanAndMult(knt,p,u,NL_LEFT,&spn,&mlt); 
  if( error EQ NL_YES )  NL_OUT;

  /* Allocate memory needed for each curve */
  if (pExistingL) {
      if (pExistingL->pol->n != spn-mlt) return 1;
      if (pExistingL->knt->m != spn-mlt+p+1) return 1;
      curL = pExistingL;
  }
  else {
      curL = sm_AllocateNurbCurve(spn-mlt,p,spn-mlt+p+1);
  }
//  error = N_CrvSizeArrays(curL,spn-mlt,p,spn-mlt+p+1,rname,SG);
//  if( error EQ NL_YES )  NL_OUT;

  Lw = curL->pol->Pw;
  UL = curL->knt->U;

  if (pExistingR) {
      if (pExistingR->pol->n != n+p-spn) return 1;
      if (pExistingR->knt->m != n-spn+2*p+1) return 1;
      curR = pExistingR;
  }
  else {
      curR = sm_AllocateNurbCurve(n+p-spn,p,n-spn+2*p+1);
  }
//  error = N_CrvSizeArrays(curR,n+p-spn,p,n-spn+2*p+1,rname,SG);
//  if( error EQ NL_YES )  NL_OUT;

  Rw = curR->pol->Pw;
  UR = curR->knt->U;

//  /* Get auxiliary control points */
//  Sw = N_AllocCPt1dArray(p,&SL);
//  if( Sw EQ NULL )  NL_QUIT;

  for( i=0; i<=p-mlt; i++ )
  {
    InitCP(Pw[spn-p+i],&Sw[i]);
  }


  /* Save unaltered control points */
  for( i=0; i<=spn-p; i++ )
  {
    InitCP(Pw[i],&Lw[i]);
  }

  for( i=spn-mlt; i<=n; i++ )
  {
    InitCP(Pw[i],&Rw[i+p-spn]);
  }

  /* Now split the curve */
  for( i=1; i<=p-mlt; i++ )
  {
    k = spn-p+i;
    for( j=0; j<=p-i-mlt; j++ )
    {
      alf = (u-U[k+j])/(U[spn+j+1]-U[k+j]);
      oma = 1.0-alf;
      N_Combine2CPts(alf,Sw[j+1],oma,Sw[j],&Sw[j]);
    }
    InitCP(Sw[0      ],&Lw[k      ]);
    InitCP(Sw[p-i-mlt],&Rw[p-i-mlt]);
  }


  /* Load knot vectors */
  k = spn-mlt;
  for( i=0; i<=k; i++ )  UL[i      ] = U[i];
  for( i=0; i<=p; i++ )  UL[k+i+1  ] = u;

  k = spn+1;
  for( i=0; i<=p; i++ )  UR[i      ] = u;
  for( i=k; i<=m; i++ )  UR[i-k+p+1] = U[i];


  /* End NURBS and Exit */
  EXIT:

  return(error);

} // end sm_CrvSplit

/*******************************************************************//**
PURPOSE: Trim a nurb curve to a smaller parametric range.

NOTES: 
***********************************************************************/
gw_FLAG  sm_CrvTrim
 (gw_CURVE        * curP,          // in : gw_Curve to trim
  gw_PARAMETER      ul,            // in : new UMin value
  gw_PARAMETER      ur,            // in : new UMax value
  gw_CURVE       *& curQ,          // out: copied and trimmed output curve, either newly allocated or pExistingCurQ
  gw_CURVE        * pExistingCurQ) // in : NotNULL = edit this curve, don't allocate new Curve
                                   //      NULL    = edit a newly allocated curve
                                   //      default:[NULL]
{
  gw_FLAG        error = NL_NO;
  gw_INDEX       i, j, k, ll, lk, lr, n, m, spl, mll, spr, mlr, is, ie;
  gw_DEGREE      p;
  gw_REAL        *UP, *UQ, alf, oma, left;
  gw_KNOTVECTOR  *knt;
  gw_CPOINT      *Pw, *Qw;

  /* Get local notation */
  knt = curP->knt;
  N_CrvGetCPtsDegreeAndKnots(curP,&n,&Pw,&p,&m,&UP);  // n=highest Pw index, Pw=CPts, p=degree, m=hihest UP index, UP=knots

  /* Check parameters */
  if( ur LE ul )  
    { error = NL_INP_ERR; NL_OUT; }

  /* Find knot spans and set new indexes - checks are made exactly (no tolerances) */
  error = N_BasisFindSpanAndMult(knt,p,ul,NL_LEFT,  // knt=KnotVector, p=degree, ul=knot, LEFT=ivl include left bdry,
                                 &spl,&mll);        // spl=KnotIndex beginning span containing ul, mll=knot multiplicity when ul is an existing knot value
  if( error EQ NL_YES ) 
    { NL_OUT; }

  error = N_BasisFindSpanAndMult(knt,p,ur,NL_LEFT,  // knt=KnotVector, p=degree, ur=knot, LEFT=ivl include left bdry,
                                 &spr,&mlr);        // spr=KnotIndex beginning span containing ur, mlr=knot multiplicity when ul is an existing knot value
  if( error EQ NL_YES ) 
    { NL_OUT; }

  is = spl-p;             // 1st knotIndex with a nonZero basis function in trim interval
  if( ur EQ UP[m-p] )     // when ur equal 1st knotIndex ending the last span 
    {                     
      spr = m;            // switch spr so that spr looks like the 1st KnotIndex of an imaginary span that comes after the last span
      mlr = p+1;          // the multipliciy for the bounding KnotIndex is always degree+1
    }                     
  ie = spr-mlr;           // last knot index with a nonZero basis function in trim interval

  //
  n = ie-is;              // highestIndex of control points that span the trim interval
  m = spr-spl-mlr+2*p+1;  // highestIndex of knots that span the trim interval

  // Degenerate trim interval (e.g. from floating point noise) produces
  // no control points.  Return an error so the caller can skip or
  // recover rather than allocating an undersized curve.
  if (n < 0)
    { error = NL_YES; NL_OUT; }

  // get the output curve
  if (pExistingCurQ) 
    {
      curQ = pExistingCurQ;

      // Check for correct size
      if(   curQ->p != p 
         || curQ->pol->n != n 
         || curQ->knt->m != m) 
       { return 1; }
    }
  else 
    { curQ = sm_AllocateNurbCurve(n,p,m); }

  //  error = N_CrvSizeArrays(curQ,n,p,m,rname,SQ);
  //  if( error EQ NL_YES )  NL_OUT;
  
  // output curve locals
  Qw = curQ->pol->Pw;
  UQ = curQ->knt->U;

  /* Copy control point values that cover the span interval */
  for( i=is; i<=ie; i++ ) 
    {
      InitCP(Pw[i],&Qw[i-is]);
    }

  /* Insert the left knot */
  ll = spl-p;
  for( i=1; i<=p-mll; i++ )
    {
      for( j=0; j<=p-i-mll; j++ )
        {
          left = UP[ll+i+j];
          alf  = (ul-left)/(UP[spl+j+1]-left);
          oma  = 1.0-alf;
          N_Combine2CPts(alf,Qw[j+1],oma,Qw[j],&Qw[j]);
        }
    }

  /* Insert the right knot */
  lr = spr-p;  lk = n-p+mlr;
  for( i=1; i<=p-mlr; i++ )
    {
      for( j=p-i-mlr; j>=0; j-- )
        {
          k    = lk+i+j;
          left = UP[lr+i+j];
          if( left LT ul )  left = ul;

          alf = (ur-left)/(UP[spr+j+1]-left);
          oma = 1.0-alf;
          N_Combine2CPts(alf,Qw[k],oma,Qw[k-1],&Qw[k]);
        }
    }

  /* Load the knot vector */
  j = -1;
  for( i=0;     i<=p;       i++ )  UQ[++j] = ul;
  for( i=spl+1; i<=spr-mlr; i++ )  UQ[++j] = UP[i];
  for( i=0;     i<=p;       i++ )  UQ[++j] = ur;

  /* End NURBS and Exit */
  EXIT:
  return(error);

} // end sm_CrvTrim

/*******************************************************************//**
PURPOSE: This geometry processing routine reparametrizes a NURBS curve such
     that the end weights become specific  values.
NOTES: 
     It retains the original domain
***********************************************************************/
gw_FLAG  sm_CrvReparamWeights 
 (gw_CURVE * cur, 
  gw_REAL    wt0, 
  gw_REAL    wtn)
{

  gw_FLAG      error = NL_NO;

  gw_INDEX     i, j, n, m;

  gw_DEGREE    p;

  gw_REAL      *U, *w, alf, bet, gam, del, exp, w0, wn, num, den, fact, 
               a, b, aa, bb, u0, fac;

  gw_CPOINT    *Pw;


  gw_REAL      tol = 1.0e-12;


  gw_STACKS   S;

  /* Check rationality */
  if( NOT N_IsCrvRat(cur) ) return(0);


  /* Get local notation and rescale span */
  N_CrvGetCPtsDegreeAndKnots(cur,&n,&Pw,&p,&m,&U);

  /* capture the original domain */
  aa = U[0]; bb = U[m];

  /* if existing end weights are the same, then exit */
  if (Pw[0].w == wt0 && Pw[n].w == wtn) return(0);

  /* Start NURBS environment */
  N_InitNurbs(&S);

  /* temporarily reset domain from 0::1 */
  a=0.0; b=1.0;
  if( a NEQ U[0]  OR  b NEQ U[m] )
  {
    u0  = U[0];
    fac = (b-a)/(U[m]-U[0]); 

    for( i=0;   i<=p;     i++ )  U[i] = a;
    for( i=p+1; i<=m-p-1; i++ )  U[i] = fac*(U[i]-u0)+a;
    for( i=m-p; i<=m;     i++ )  U[i] = b;
  }  

  /* Reparametrize curve */

 
  N_CPtGetW(Pw[0],&w0);
  N_CPtGetW(Pw[n],&wn);

  exp = 1.0/p;
  w0  = pow(w0/wt0,exp);
  wn  = pow(wn/wtn,exp);

  alf = wn;
  bet = 0.0;
  gam = wn-w0;
  del = w0;

  if( (alf*del-bet*gam) LT tol ) goto EXIT;

  w = N_AllocReal1dArray(n, &S);
  if( w EQ NULL )  goto EXIT;

  for( i=0; i<=n; i++ )
  {
    N_CPtGetW(Pw[i],&w[i]);

    if( N_FloatOpIsBad(1.0,w[i],NL_DIVISION) )goto EXIT;

    fact = 1.0/w[i];
    N_ScaleCPt(fact,Pw[i],&Pw[i]);
  }

  N_ScaleCPt(wt0,Pw[0],&Pw[0]);
  for( i=1; i<n; i++ )
  {
    fact = 1.0;
    for( j=1; j<=p; j++ )  fact *= (gam*U[i+j]+del);
    if( fact LT 0.0 )  fact = -fact;

    if( N_FloatOpIsBad(w[i],fact,NL_DIVISION) ) goto EXIT;

    w[i] = w[i]/fact;
    N_ScaleCPt(w[i],Pw[i],&Pw[i]);
  }
  N_ScaleCPt(wtn,Pw[n],&Pw[n]);

  Pw[0].w = wt0;
  Pw[n].w = wtn;

  for( i=0; i<=m; i++ )
  {
    num = alf*U[i]+bet;
    den = gam*U[i]+del;

    if( N_FloatOpIsBad(num,den, NL_DIVISION) )goto EXIT;

    U[i] = num/den;
  }

  /* reset domain to input domain */
  a=aa; b=bb;
  if( a NEQ U[0]  OR  b NEQ U[m] )
  {
    u0  = U[0];
    fac = (b-a)/(U[m]-U[0]); 

    for( i=0;   i<=p;     i++ )  U[i] = a;
    for( i=p+1; i<=m-p-1; i++ )  U[i] = fac*(U[i]-u0)+a;
    for( i=m-p; i<=m;     i++ )  U[i] = b;
  }  

  /* End NURBS and Exit */


  EXIT:
  N_EndNurbs(&S);
  return(error);

} // end sm_CrvReparamWeights

/*******************************************************************//**
PURPOSE: Map control polygon to Euclidian space.

NOTES: 
***********************************************************************/
gw_FLAG sm_CrvGetEPolygon 
 (gw_CURVE    * cur, 
  gw_INDEX      k, 
  gw_INDEX      l, 
  gw_EPOLYGON * ppl, 
  NL_POINT    * P)
{
  gw_INDEX   i, n;
  gw_CPOINT  *Pw;
  /* Get local notation */
  N_CrvGetCPts(cur,&n,&Pw);
  /* Check indexes */
  if( k GT l  OR  k LT 0  OR  l GT n ) {
    N_ErrSet(NL_IND_ERR,_T("Evaluate"));
    return(1);
  }
  /* Map control points */
  for( i=k; i<=l; i++ ) {
     N_CPtToPtEuclid(Pw[i],&P[i-k]);
  }
  /* Build polgon structure */
  ppl->n = l-k;
  ppl->P = P;
  /* Exit */
  return(0);

} // end sm_CrvGetEPolygon

/*******************************************************************//**
PURPOSE: Compute all non-vanishing basis functions at a given parameter

NOTES: 
***********************************************************************/
gw_FLAG  sm_BasisDerivs
 (gw_KNOTVECTOR *knt,                  // in : 
  gw_DEGREE      p,                    // in : 
  gw_PARAMETER   u,                    // in : 
  gw_FLAG        flg,                  // in : 
  gw_INDEX       der,                  // in : 
  gw_REAL       *ND[GW_MAX_DEGREE],    // out: 
  gw_INDEX      *spn )                 // out: 
{
  // locals
  gw_FLAG    error = NL_NO;
  gw_INDEX   i, j, k, r, s1, s2, rk, pk, j1, j2, mder;
  gw_REAL    *U, saved, d;
  gw_REAL    left[GW_MAX_DEGREE];
  gw_REAL    right[GW_MAX_DEGREE];
  gw_REAL    ndu[GW_MAX_DEGREE][GW_MAX_DEGREE];
  gw_REAL    a[GW_MAX_DEGREE][GW_MAX_DEGREE];

  /* Same-knot tolerance: see discussion by its first use, below. */
  /* (Originally hard-coded at 1e-8, throughout the code.)        */
  gw_REAL dKnotTol = SM_EFF_ZERO_SQ;

  /* Get local notation */
  k = knt->m;
  U = knt->U;

  /*  N_KnotVectorGetKnots(knt,&k,&U); */

  /* Find the knot span u is in */
  error = N_BasisFindSpan(knt,p,u,flg,&i);
  if( error EQ NL_YES )  NL_OUT; 

  /* In case of early exit: */
  error = NL_YES;

  *spn = i;
  /* Get maximum derivative index and set zero derivatives */
  mder = p < der ? p:der; 
  for( k=p+1; k<=der; k++ ) 
    {
      gw_REAL *pNDkj = ND[k];
      for( j=0; j<=p; j++ ) 
        {
          pNDkj[0] = 0.0;
          pNDkj++;
        }
    }

  /* Compute the basis functions */
  ndu[0][0] = 1.0;
  for( j=1; j<=p; j++ ) 
    {
      left[j]  = u-U[i+1-j]; 
      right[j] = U[i+j]-u; 
      saved    = 0.0;
      gw_REAL *pnduj = ndu[j];
      for( r=0; r<j; r++ ) 
        {
          ndu[j][r] = right[r+1] + left[j-r];

          /* Divide by zero?  Note that the denominator is the sum of
           * the two things being multiplied by it, in the following
           * two statements, so the quotients in parentheses can never
           * be greater than 1.0.  Therefore, the only way for any floating-
           * point problem is if the delta is exactly zero, which can
           * happen only if there are more than degree+1 consecutive
           * equal knots (which would probably be caught before here).
           * So we essentially just have to guard against the denominator
           * being exactly zero ... but use something very tiny, to be safe.
           *
           * Note: keep the divisions inside the parentheses: dividing
           * ndu[][] by the knot difference can be huge.
           */
          if (fabs( pnduj[r]) <= dKnotTol /* SM_EFF_ZERO_SQ */ )
            { goto EXIT; }

          ndu[r][j] = saved + ndu[r][j-1] * ( right[r+1] / pnduj[r] );
          saved     =         ndu[r][j-1] * (  left[j-r] / pnduj[r] );
        }
      ndu[j][j] = saved;
    } 

  /* Load the basis functions */
  {
    gw_REAL *pND0 = ND[0];
    for( j=0; j<=p; j++ ) 
      {
        pND0[j] = ndu[j][p];
      }
  }

  /* Compute derivatives */
  for( r=0; r<=p; r++ ) 
    {
      s1 = 0;  
      s2 = 1;  
      a[0][0] = 1.0;
      for( k=1; k<=mder; k++ ) 
        {
          gw_REAL *pas1 = a[s1];
          gw_REAL *pas2 = a[s2];
          d  = 0.0;  
          rk = r-k;  
          pk = p-k;
          gw_REAL *ndupk1 = ndu[pk+1];
          if( r GE k ) 
            {
              if (fabs(ndupk1[rk])<= dKnotTol /* 1e-8 */ ) goto EXIT;
              a[s2][0] = pas1[0]/ndupk1[rk];
              d        = pas2[0]*ndu[rk][pk];
            }
          if( rk    GE -1 )  j1 = 1;    else  j1 = -rk;
          if( (r-1) LE pk )  j2 = k-1;  else  j2 = p-r;
          for( j=j1; j<=j2; j++ ) 
            {
              if (fabs(ndupk1[rk+j])<= dKnotTol /* 1e-8 */ ) goto EXIT;
              pas2[j]  = (pas1[j]-pas1[j-1])/ndupk1[rk+j];
              d        += pas2[j]*ndu[rk+j][pk];
            }
          if( r LE pk ) 
            {
              if (fabs(ndupk1[r])<= dKnotTol /* 1e-8 */ ) goto EXIT;
              pas2[k]  = -pas1[k-1]/ndupk1[r];
              d        +=  pas2[k]*ndu[r][pk];
            }
          ND[k][r] = d;
          long iTmp = s1;
          s1 = s2;
          s2 = iTmp;
          //N_SwapIntegers(&s1,&s2);
        } 
    }

  /* Multiply through by the correct factors */
  r = p;
  for( k=1; k<=mder; k++ ) 
    {
      gw_REAL *NDk = ND[k];
      for( j=0; j<=p; j++ ) 
        {
          NDk[j] *= r;
        }
      r *= (p-k);
    }

  /* Made it through. */
  error = NL_NO;

  /* End NURBS and Exit */
  EXIT:
  return(error);

} // end sm_BasisDerivs

/*******************************************************************//**
PURPOSE: Computes all non-vanishing rational basis functions.

NOTES: 
***********************************************************************/
gw_FLAG  sm_CrvRationalBasisDerivs
  (gw_CURVE      *cur,    // in : 
   gw_PARAMETER   u,      // in : 
   gw_FLAG        flg,    // in : 
   gw_INDEX       der,    // in : 
   gw_REAL      **RD,     // out: 
   gw_INDEX      *spn )   // out: 
{
  // locals
  gw_FLAG        error = NL_NO;
  gw_INDEX       i, j, k, nsp;
  gw_REAL        ND_STACK[GW_MAX_DERIV][GW_MAX_DEGREE];
  gw_REAL        *ND[GW_MAX_DERIV];
  gw_REAL        d[GW_MAX_DEGREE];
  long     tri_STACK[GW_MAX_DERIV][GW_MAX_DERIV];
  long     *tri[GW_MAX_DERIV];
  gw_REAL        v, w;
  gw_DEGREE      p;
  gw_KNOTVECTOR  *knt;
  gw_CPOINT      *Pw;

  // init double array pointers
  for (i=0; i<=der; i++) {  ND[i]  = ND_STACK[i]; }
  for (i=0; i<=der; i++) {  tri[i] = tri_STACK[i]; }

  /* Get local notation */
  Pw  = cur->pol->Pw;
  p   = cur->p;
  knt = cur->knt;

  /* Get derivatives of all basis functions */

  error = sm_BasisDerivs(knt,p,u,flg,der,ND,&nsp); 
  if( error EQ NL_YES )  NL_OUT;

  /* Get derivatives of the denominator */

  for( k=0; k<=der; k++ )
    {
      d[k] = 0.0;
      for( j=nsp-p; j<=nsp; j++ )
        {
          w = Pw[j].w;
          d[k] += w*ND[k][j-nsp+p];
        }
    }


  /* Compute derivatives of rational basis function */

  N_PascalTriRow(tri,der);

  for( i=0; i<=p; i++ )
    {
      w = Pw[nsp-p+i].w;
      for( k=0; k<=der; k++ )
        {
          v = w*ND[k][i];
          for( j=1; j<=k; j++ )
            {
              v -= tri[k][j]*d[j]*RD[k-j][i];
            }
          RD[k][i] = v/d[0];
        }
    }

  *spn = nsp;

  /* End NURBS and Exit */
  EXIT:
  return(error);

} // end sm_CrvRationalBasisDerivs

/*******************************************************************//**
PURPOSE: Compute the point and derivatives of a curve given a parameter.

NOTES: Checks the type of gw_CURVE and 
  gets Basis Values from NLib with oneof
    o. sm_CrvRationalBasisDerivs()
    o. sm_BasisDerivs()
  then computes
    W    = Sum_i(Basis[0][i] * P[i])
    Ws   = Sum_i(Basis[1][i] * P[i])
    Wss  = Sum_i(Basis[2][i] * P[i]) 
    . . .
***********************************************************************/
gw_FLAG  sm_CrvDerivs
 (gw_CURVE *cur,             // in : target curve
  gw_PARAMETER u,            // in : target parameter
  gw_FLAG flg,               // in : oneof LEFT  = at interval boundaries compute values from lower param interval
                             //            RIGHT = at interval boundaries compute values from upper param interval
  gw_INDEX der,              // in : number of derivatives to compute
  NL_POINT *CD )             // out: array of [pos, 1st, 2nd, ... ] output values
{
  // locals
  gw_FLAG        error = NL_NO;
  gw_INDEX       i, k, spn;
  gw_DEGREE      p;
  gw_REAL        BD_STACK[GW_MAX_DERIV][GW_MAX_DEGREE];
  gw_REAL        *BD[GW_MAX_DERIV];
  gw_KNOTVECTOR  *knt;
  NL_EPOLYGON    ppl;
  NL_POINT       *P;
  NL_POINT   P_IN[GW_MAX_DEGREE+1];

  if (der > GW_MAX_DERIV) return 1; // error

  for (i=0; i<=der; i++) { BD[i] = BD_STACK[i]; }

  /* Get local notation */
  p   = cur->p;     // degree
  knt = cur->knt;   // knot vector struct

  /* Check parameter */
//  error = N_KnotVectorIsParamOutOfBounds(knt,u,"Evaluate");
//  if( error EQ NL_YES )  NL_OUT;

  /* Compute basis function derivatives */
  if( N_IsCrvRat(cur) )
    {
      error = sm_CrvRationalBasisDerivs(cur,u,flg,der,BD,&spn);  
      if( error EQ NL_YES )  NL_OUT;
    }
  else
    {
      error = sm_BasisDerivs(knt,p,u,flg,der,BD,&spn); 
      if( error EQ NL_YES )  NL_OUT;
    }

  error = sm_CrvGetEPolygon(cur,spn-p,spn,&ppl,P_IN);
  if( error EQ NL_YES )  NL_OUT;

  P = ppl.P;

  // Compute values as W = Sum_i(BasisVal[i] * ControlPoint[i]) 
  // for every requested pos and deriv value
  for( k=0; k<=der; k++ )
    {
      NL_POINT *pCD = &CD[k];
      pCD->x = pCD->y = pCD->z = 0.0;
      for( i=0; i<=p; i++ )
        {
          //      N_VectorBlendPt(BD[k][i],P[i],&CD[k]);
            pCD->x += BD[k][i] * P[i].x;
            pCD->y += BD[k][i] * P[i].y;
            pCD->z += BD[k][i] * P[i].z;
        } // end iter every nonZero Basis function
    } // end iter every requested evalution (position plus derivs)

  /* End NURBS and Exit */
  EXIT:
  return(error);

} // end sm_CrvDerivs

/*******************************************************************//**
PURPOSE: Compute all non-vanishing basis functions given a parameter.

NOTES: 
***********************************************************************/
gw_FLAG  sm_BasisEval
 (gw_KNOTVECTOR *knt,    // in : Curve's knot vector 
  gw_DEGREE      p,      // in : curve degree 
  gw_PARAMETER   u,      // in : target param
  gw_FLAG        flg,    // in : oneof: LEFT  = for evals exactly on param boundaries - use lower param span
                         //             RIGHT = for evals exactly on param boundaries - use upper param span
  gw_REAL       *N,      // out: array of NonZero basis values for the following span 
  gw_INDEX      *spn )   // out: span associated with non-zero basis values
                         //      Basis value N[i] correlates to control point P[spn - p + i]
{
  /* Same-knot tolerance: see discussion in sm_BasisDerivs(). */
  /* (Originally hard-coded at 1e-8, throughout the code.)      */
  gw_REAL  dKnotTol = SM_EFF_ZERO_SQ;

  gw_FLAG  error = NL_YES; /* init for early termination */
  gw_INDEX i, k, l, m;
  gw_REAL  *U, saved, temp;
  gw_REAL  left[GW_MAX_DEGREE], right[GW_MAX_DEGREE];

  /* Get local notation */
  m = knt->m;
  U = knt->U;
//  N_KnotVectorGetKnots(knt,&m,&U);

  /* Special cases for end values */
  if( u EQ U[p] )
  {
    N[0] = 1.0; *spn = p;
    for( k=1; k<=p; k++ ) N[k] = 0.0;
    NL_OUT;
  }

  if( u EQ U[m-p] )
  {
    N[p] = 1.0; *spn = m-p-1;
    for( k=0; k<p; k++ ) N[k] = 0.0;
    NL_OUT;
  }

  /* Find the knot span u is in */
  N_BasisFindSpan(knt,p,u,flg,&i); 
  *spn = i;

  /* Compute the non-vanishing B-splines */
  N[0] = 1.0;
  for(k=1; k<=p; k++)
  {
    left[k]  = u-U[i+1-k]; 
    right[k] = U[i+k]-u;
    saved    = 0;
    for(l=0; l<k; l++)
    {
      double dDelta = right[l+1] + left[k-l];
      if ( dDelta < dKnotTol )
        { goto EXIT; }
      /*
       * Note: keep the divisions inside the parentheses, as they are:
       * dividing N[l] by the knot difference can be huge.
       * (See discussion in sm_BasisDerivs().)
       */
      temp  = N[l];
      N[l]  = saved + temp * ( right[l+1] / dDelta );
      saved =         temp * (  left[k-l] / dDelta );
    }
    N[k] = saved;
  } 

  /* Made it through. */
  error = NL_NO;

  /* End NURBS and Exit */
  EXIT:
  return( error );

} // end sm_BasisEval

/*******************************************************************//**
PURPOSE: Evaluate a point on a nurb curve.

NOTES: Call sm_Basis(..,N,..) and then let C = Sum_i(P[i]*N[i])
***********************************************************************/
gw_FLAG sm_CrvEval
 (gw_CURVE const * cur, // in : target curve
  gw_PARAMETER  u,      // in : target param
  gw_FLAG       flg,    // in : oneof LEFT  = at interval boundaries compute values from lower param interval
                        //            RIGHT = at interval boundaries compute values from upper param interval
  NL_POINT    * C )     // out: W of W = cur(u)
{
  // locals
  gw_FLAG        error = NL_NO;
  gw_INDEX       i, spn ;
  gw_REAL        N[GW_MAX_DEGREE];

  /* Get local notation */
  gw_CPOINT      *Pw    = cur->pol->Pw;
  gw_DEGREE       p     = cur->p;
  gw_KNOTVECTOR  *knt   = cur->knt;
  gw_INDEX        spn_p = 0 ;
  gw_CPOINT       Cw;
  
  // get array of all nonZero basis values for span containing u
  error = sm_BasisEval(knt,   // in : Curve's knot vector                                                        
                       p,     // in : curve degree                                                               
                       u,     // in : target param                                                               
                       flg,   // in : oneof: LEFT  = for evals exactly on param boundaries - use lower param span
                              //             RIGHT = for evals exactly on param boundaries - use upper param span
                       N,     // out: array of NonZero basis values for the following span                       
                       &spn); // out: span associated with non-zero basis values 
                              //      Basis value N[i] correlates to control point P[spn - p + i]                         
  if( error EQ NL_YES )  
    { NL_OUT; }

  /* Compute the point on the curve */
  Cw.x = 0.0; Cw.y = 0.0; Cw.z = 0.0; Cw.w = 0.0;

  // get control point index for first nonZero basis function in span, spn.
  spn_p = spn - p;

  // for every nonZero basis in span, spn
  for( i=0; i<=p; i++ )
    {
      // build Cw = Sum_i(N[i]*Pw[i]
      MAC_A_updcpt(N[i], Pw[spn_p+i], &Cw);
    }

  TO_EUCLID(Cw,*C); 

  /* End NURBS and Exit */
EXIT:
  return(error);

} // end sm_CrvEval


NL_PRIVATE  gw_REAL    cto = 1.0e-05;


/* ---------------------------------------------------------------------


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS curve. 
     If the  output curve is initialized to NULL, memory  to  store new  
     control points and knots is allocated. If the  output curve is the 
     same as the input  curve, knot  removal is  done in place  and the 
     original curve is destroyed. A typical calling example is:

       gw_CURVE   curP, curQ;
       gw_REAL    tol;
       gw_STACKS  SQ;
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
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     sm_BasisFindAllSpanMaxima - Compute minimums and maximums of basis function
     N_CrvGetMinMaxWeightsAndPts - Compute min-max curve weights and position vectors
     N_CrvRemoveKnotMaxErr - Compute curve removal error bound
     U_****** - A number of utility routines
     S_****** - A few memory allocation routines
     A_****** - A few arithmetic routines
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */
gw_FLAG sm_CrvRemoveKnots 
 (gw_CURVE  * curP, 
  gw_REAL      tol, 
  gw_CURVE  * curQ, 
  gw_STACKS * SQ )
{

  gw_FLAG        rmf, rat = NL_NO, error = NL_NO;

  gw_INDEX       *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, 
              fout, l, ns;

  gw_DEGREE      p;

  gw_REAL        *UP, *UQ, *br, *er, *te, *minl, *maxl, *minr, *maxr,
              *max, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb,
              lam = 0.0, oml = 0.0, lto, wi, wj;

  gw_KNOTVECTOR  *knt;

  gw_CPOINT      *Pw, *Qw, *Rw;

  gw_STACKS      SL;



  /* Start NURBS environment */


  N_InitNurbs(&SL);


  /* Get local notation */


  N_CrvGetCPtsDegreeAndKnots(curP,&n,&Pw,&p,&m,&UP);
  ns = n;
  

  /* Adjust removal tolerance in case of rational curves */


  if( N_IsCrvRat(curP) )
  {
    N_CrvGetMinMaxWeightsAndPts(curP,&wmin,&tmp,&tmp,&pmax);
    tol = (tol*wmin)/(1.0+pmax);
    rat = NL_YES;
  }

  lto = cto*fabs(UP[m]-UP[0]);


  /* See if memory is needed */


  if( curP EQ curQ )
  {
    N_CrvGetCPtsKnotVectorAndKnots(curP,&Qw,&knt,&UQ);
  }
  else
  {
    error = N_CrvSizeArrays(curQ,n,p,m,rname,SQ);
    if( error EQ NL_YES )  NL_OUT;

    N_CrvGetCPtsKnotVectorAndKnots(curQ,&Qw,&knt,&UQ);
  }

  /* Get local memory */


  Rw = N_AllocCPt1dArray(2*p,&SL);
  if( Rw EQ NULL )  NL_QUIT;

  br = N_AllocReal1dArray(m,&SL);
  if( br EQ NULL )  NL_QUIT;

  sr = N_AllocInt1dArray(m,&SL);
  if( sr EQ NULL )  NL_QUIT;

  er = N_AllocReal1dArray(m,&SL);
  if( er EQ NULL )  NL_QUIT;

  te = N_AllocReal1dArray(m,&SL);
  if( te EQ NULL )  NL_QUIT;

  minl = N_AllocReal1dArray(p,&SL);
  if( minl EQ NULL )  NL_QUIT;

  maxl = N_AllocReal1dArray(p,&SL);
  if( maxl EQ NULL )  NL_QUIT;

  minr = N_AllocReal1dArray(p,&SL);
  if( minr EQ NULL )  NL_QUIT;

  maxr = N_AllocReal1dArray(p,&SL);
  if( maxr EQ NULL )  NL_QUIT;

  max = N_AllocReal1dArray(p+1,&SL);
  if( max EQ NULL )  NL_QUIT;


  /* Initialize */


  if( curP NEQ curQ )
  {
    error = N_CrvCopy(curP,curQ,SQ);
    if( error EQ NL_YES )  NL_OUT;
  }

  for( i=0; i<=m; i++ )
  {
    br[i] = NL_BIGD;
    sr[i] = 0;
    er[i] = 0.0;
  }


  /* Compute knot removal errors for each distinct knot */

  r = p+1;
  while( r LE n )
  {
    i = r;
    while( r LE n  AND  UQ[r] EQ UQ[r+1] )  r++;
    sr[r] = r-i+1;

    if( sr[r] > p ) 
        NL_OUT;

    error = sm_CrvRemoveKnotMaxErr(curP,r,sr[r],&br[r]);
    if( error EQ NL_YES )  NL_OUT;

    r++;
  } 


  /* Try to remove each knot */


  while( TRUE )
  {
    /* Find knot with smallest error */

    b = br[p+1]; 
    s = sr[p+1];
    r = p+1;
    for( i=p+2; i<=m-p-1; i++ )
    {
      if( br[i] LT b )  
      {
        b = br[i];  s = sr[i];  r = i;
      }
    }


    /* If no more removable knot -> finished */

    if( b EQ NL_BIGD )  break;

    /* Check error of removal */

    rmf = TRUE;
    if( (p+s)%2 )
    {
      /* Compute maximums of basis functions over each span */

      k   = (p+s+1)/2;
      alf = (UQ[r]-UQ[r-k  ])/(UQ[r-k+p+1]-UQ[r-k  ]);
      bet = (UQ[r]-UQ[r-k+1])/(UQ[r-k+p+2]-UQ[r-k+1]);
      omb = 1.0-bet;
      lam = alf/(alf+bet);
      oml = 1.0-lam; 

      error = sm_BasisFindAllSpanMaxima(knt,r-k  ,p,lto,minl,maxl,&tmp);
      if( error EQ NL_YES )  
      {
        for( i=0; i<=p; i++ ) {  minl[i] = 0.0;  maxl[i] = 1.0;  }
      }

      error = sm_BasisFindAllSpanMaxima(knt,r-k+1,p,lto,minr,maxr,&tmp);
      if( error EQ NL_YES )
      {  
        for( i=0; i<=p; i++ ) {  minr[i] = 0.0;  maxr[i] = 1.0;  }
      }

      max[0] = lam*alf*maxl[0];
      for( i=1; i<=p; i++ )
      {
        minl[i] *= lam*alf;   minr[i-1] *= oml*omb;
        maxl[i] *= lam*alf;   maxr[i-1] *= oml*omb;
       
        max[i] = (fabs(maxl[i]-minr[i-1])) > (fabs(maxr[i-1]-minl[i])) ?
                  fabs(maxl[i]-minr[i-1])  :  fabs(maxr[i-1]-minl[i]);
      }
      max[p+1] = oml*omb*maxr[p];

      /* Check the error */

      for( i=r-k; i<=r-k+p+1; i++ )
      {
        if( UQ[i] NEQ UQ[i+1] )  
        { 
          te[i] = er[i]+max[i-r+k]*b; 
          if( te[i] GT tol )  {  rmf = FALSE;  break;  }
        }
      }
    }
    else
    {
      /* Compute maximum of basis function */

      k = (p+s)/2;

      error = sm_BasisFindAllSpanMaxima(knt,r-k,p,lto,minl,max,&tmp);
      if( error EQ NL_YES )  
      {
        for( i=0; i<=p; i++ )  max[i] = 1.0;
      }

      /* Check the error */

      for( i=r-k; i<=r-k+p; i++ )
      {
        if( UQ[i] NEQ UQ[i+1] )  
        { 
          te[i] = er[i]+max[i-r+k]*b; 
          if( te[i] GT tol )  {  rmf = FALSE;  break;  }
        }
      }
    }

    /* If error test passed -> update error vector */

    if( rmf EQ TRUE )
    {
      if( (p+s)%2 )  l = r-k+p+1;  else  l = r-k+p;

      for( i=r-k; i<=l; i++ )  
      {
        if( UQ[i] NEQ UQ[i+1] )  er[i] = te[i];
      }
    }

    /* Remove the knot one time */    

    if( rmf EQ TRUE )
    {
      fout  = (2*r-s-p)/2;    
      first = r-p;    
      last  = r-s;
      off   = first-1;        
      i     = first;  
      j     = last;
      ii    = 1;
      jj    = last-off;

      N_CopyCPt(Qw[off   ],&Rw[0         ]);
      N_CopyCPt(Qw[last+1],&Rw[last+1-off]);

      /* Get new control points for one removal step */

      while( (j-i) GT 0 )
      {
        alf = (UQ[i+p+1]-UQ[i])/(UQ[r]-UQ[i]);
        oma = 1.0-alf;
        bet = (UQ[j+p+1]-UQ[j])/(UQ[j+p+1]-UQ[r]);
        omb = 1.0-bet;
        N_Combine2CPts(alf,Qw[i],oma,Rw[ii-1],&Rw[ii]);
        N_Combine2CPts(bet,Qw[j],omb,Rw[jj+1],&Rw[jj]);
        i ++;  j --;  
        ii++;  jj--;
      }

      /* Check for disallowed weights */

      if( rat EQ NL_YES )
      {
        i    = first;
        j    = last;
        wmin =  NL_BIGD;
        wmax =  NL_SMAD;
        while( (j-i) GT 0 )
        {
          wi = Rw[i-off].w;
          wj = Rw[j-off].w;
          if( wi LT wmin )  wmin = wi;
          if( wj LT wmin )  wmin = wj;
          if( wi GT wmax )  wmax = wi;
          if( wj GT wmax )  wmax = wj;
          i++;  j--;
        }
        if( wmin LT WMIN  OR  wmax GT WMAX )  {  br[r] = NL_BIGD;  continue;  }
      }

      /* Save new control points */

      if( (p+s)%2 )
      {
        N_Combine2CPts(lam,Rw[jj+1],oml,Rw[ii-1],&Rw[ii-1]);
      }
  
      i = first;
      j = last;
      while( (j-i) GT 0 )
      {
        N_CopyCPt(Rw[i-off],&Qw[i]);
        N_CopyCPt(Rw[j-off],&Qw[j]);
        i++;  j--;
      }

      /* Shift down some parameters */

      if( s EQ 1 )  er[r-1] = er[r-1] > er[r] ? er[r-1] : er[r];

      if( s GT 1 )  sr[r-1] = sr[r]-1;

      for( i=r+1; i<=m; i++ )
      {
        br[i-1] = br[i];
        sr[i-1] = sr[i];
        er[i-1] = er[i];
      }

      /* Shift down knots and control points */

      for( i=r+1; i<=m; i++ )  UQ[i-1] = UQ[i];
      m--;  curQ->knt->m--;

      for( i=fout+1; i<=n; i++ )
      {
        N_CopyCPt(Qw[i],&Qw[i-1]);
      }
      n--;  curQ->pol->n--;

      /* If no more internal knots -> finished */

      if( n EQ p )  break;

      /* Update error bounds */

      k = ((r-p)>(p+1)) ? (r-p) : (p+1);
      l = (n < (r+p-s)) ? n : (r+p-s);
      for( i=k; i<=l; i++ )
      {
        if( UQ[i] NEQ UQ[i+1] ) 
        { 
          error = sm_CrvRemoveKnotMaxErr(curQ,i,sr[i],&br[i]);
          if( error EQ NL_YES )  NL_OUT;
        }
      }
    }
    else
    {
      /* Knot is not removable */

      br[r] = NL_BIGD;
    }

  } /* End of while loop */


  /* Compact output curve */


  if( n LT ns )
  {
    error = N_CrvCompress(curQ,SQ);
    if( error EQ NL_YES )  NL_OUT;
  }


  /* End NURBS and Exit */


  EXIT:

  N_EndNurbs(&SL);

  return(error);

} // end sm_CrvRemoveKnots 


/* ---------------------------------------------------------------------


   DESCRIPTION:

     This routine computes the local minimums and maximums of a basis 
     function in each knot span. It computes the global maximum first
     followed  by computing  the minima  and maxima over each span. A 
     typical calling example is:

       gw_KNOTVECTOR  knt;
       gw_INDEX       i;
       gw_DEGREE      p;
       gw_REAL        tol, *min, *max, u;
       ...
       (define knt; get memory for min and max; get i, p and tol);
       ...
       sm_BasisFindAllSpanMaxima(&knt,i,p,tol,min,max,&u);

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
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     sm_BasisFindGlobalMax - Global maximum of basis function
     N_BasisIEval - Evaluate basis function
     N_KnotVectorGetKnots - Knot vector info
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */
gw_FLAG sm_BasisFindAllSpanMaxima 
 (gw_KNOTVECTOR * knt, 
  gw_INDEX        i, 
  gw_DEGREE       p, 
  gw_REAL         tol, 
  gw_REAL       * min, 
  gw_REAL       * max, 
  gw_PARAMETER  * u )
{

  gw_FLAG    error = NL_NO;

  gw_INDEX   m, s, k, spn;

  gw_REAL    *U, Nm, Nl, Nr;


  /* Get local notation and check index */


  N_KnotVectorGetKnots(knt,&m,&U);

  if( i LT 0  OR  i GT m-p-1 )  
  { 
    smos_ErrorMessage( NL_IND_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
    NL_QUIT;
  } 

  /* Consider linear as special case */


  if( p EQ 1 )
  {
    min[0] = 0.0;  min[1] = 0.0;
    max[0] = 1.0;  max[1] = 1.0;

    *u     = U[i+1];
    NL_OUT;
  }


  /* Check is p-fold inner multiple knot exists */


  s = 1;
  for( k=i+1; k<i+p; k++ )
  {
    if( U[k] EQ U[k+1] )  s++;
  }

  if( s EQ p )
  {
    for( k=0; k<=p; k++ )  max[k] = 1.0;

    for( k=1; k< p; k++ )  min[k] = 1.0;

    if( U[i  ] EQ U[i+1  ] )  min[0] = 1.0;  else  min[0] = 0.0;
    if( U[i+p] EQ U[i+p+1] )  min[p] = 1.0;  else  min[p] = 0.0;
    
    *u   = U[i+1];
    NL_OUT;
  }


  /* Get global maximum */


  error = sm_BasisFindGlobalMax(knt,i,p,tol,&Nm,u);
  if( error EQ NL_YES )  NL_OUT;


  /* Get span index of global maximum */


  spn = -1;
  for( k=i; k<=i+p; k++ )
  {
    if( *u GE U[k]  AND  *u LT U[k+1] )
    {
      spn = k;  break;
    }
  }

  if( spn EQ -1 )  
  {
    smos_ErrorMessage( NL_CON_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
    NL_QUIT;
  } 


  /* Compute local extrema */


  for( k=0; k<spn-i; k++ )
  {
    error = sm_BasisIEval(knt,i,p,U[i+k],NL_LEFT,&min[k]);
    if( error EQ NL_YES )  NL_OUT;

    error = sm_BasisIEval(knt,i,p,U[i+k+1],NL_LEFT,&max[k]);
    if( error EQ NL_YES )  NL_OUT;
  }

  max[spn-i] = Nm;

  error = sm_BasisIEval(knt,i,p,U[spn],NL_LEFT,&Nl);
  if( error EQ NL_YES )  NL_OUT;

  error = sm_BasisIEval(knt,i,p,U[spn+1],NL_LEFT,&Nr);
  if( error EQ NL_YES )  NL_OUT;

  if( Nl LT Nr )  min[spn-i] = Nl;  else  min[spn-i] = Nr;

  for( k=spn-i+1; k<=p; k++ )
  {
    error = sm_BasisIEval(knt,i,p,U[i+k+1],NL_LEFT,&min[k]);
    if( error EQ NL_YES )  NL_OUT;

    error = sm_BasisIEval(knt,i,p,U[i+k],NL_LEFT,&max[k]);
    if( error EQ NL_YES )  NL_OUT;
  }


  /* End NURBS and Exit */


  EXIT:

  return(error);

} // end sm_BasisFindAllSpanMaxima 

/* ---------------------------------------------------------------------


   DESCRIPTION:

     This  routine computes  one basis function at a given parameter
     value. It is assumed that the knot vector is clamped, i.e., end 
     knots are  repeated  with  multiplicity = degree + 1. A typical
     calling example is:

       gw_KNOTVECTOR  knt;
       gw_INDEX       i;
       gw_DEGREE      p;
       gw_PARAMETER   u;
       gw_REAL        N;
       ...
       (define knt, get i, p and u);
       ...
       sm_BasisIEval(&knt,i,p,u,LEFT,&N);


   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline (0<=i<=n)
     p   , input  ,  Degree 
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       LEFT : u is in [u[j],u[j+1]) 
                       RIGHT: u is in (u[j],u[j+1]] 
     N   , output ,  Basis function computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     N_KnotVectorIsParamOutOfBounds - Check parameter range
     N_AllocReal1dArray - Allocate memory for real array
     N_KnotVectorGetKnots - Knot vector info
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */
gw_FLAG sm_BasisIEval 
 (gw_KNOTVECTOR * knt, 
  gw_INDEX        i, 
  gw_DEGREE       p, 
  gw_PARAMETER    u, 
  gw_FLAG         flg,
  gw_REAL       * N )
{

  gw_FLAG    error = NL_NO;

  gw_INDEX   j, k, n, m;

  gw_REAL    *U, NA[GW_MAX_DEGREE], UL, UR, saved, temp;

  /* Get local notation */


  N_KnotVectorGetKnots(knt,&m,&U);

  n = m-p-1;


  /* Check parameter and index */


//  error = N_KnotVectorIsParamOutOfBounds(knt,u,rname);
//  if( error EQ NL_YES )  NL_OUT;

  if( i LT 0  OR i GT n )  
  {
    smos_ErrorMessage( NL_IND_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
    NL_QUIT;
  }


  /* Special cases for end values */


  if( u EQ U[p] )
  {
    if( i EQ 0 )  *N = 1.0;  else  *N = 0.0;
    NL_OUT;
  }

  if( u EQ U[m-p] )
  {
    if( i EQ n )  *N = 1.0;  else  *N = 0.0;
    NL_OUT;
  }


  /* Get memory */


//  NA = N_AllocReal1dArray(p,&S);
//  if( NA EQ NULL )  NL_QUIT;
  NA[0] = 0.0;

  /* Compute degree zero B-splines */


  switch( flg )
  {
    case NL_LEFT:

      if( u LT U[i]  OR  u GE U[i+p+1] )
      {
        *N = 0.0;
        NL_OUT;
      }

      for( j=0; j<=p; j++ )
      {
        if( u GE U[i+j] AND u LT U[i+j+1] ) NA[j] = 1.0; else NA[j] = 0.0;
      }
      break;


    case NL_RIGHT:

      if( u LE U[i]  OR  u GT U[i+p+1] )
      {
        *N = 0.0;
        NL_OUT;
      }

      for( j=0; j<=p; j++ )
      {
        if( u GT U[i+j] AND u LE U[i+j+1] ) NA[j] = 1.0; else NA[j] = 0.0;
      }
      break;


    default:
      {
        smos_ErrorMessage( NL_CAL_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
        NL_QUIT;
      }
  }


  /* Compute the i-th B-spline using a triangular array */


  for( k=1; k<=p; k++ )
  {
    if( NA[0] EQ 0.0 ) saved = 0.0;  else
                       saved = ((u-U[i])*NA[0])/(U[i+k]-U[i]);

    for( j=0; j<p-k+1; j++ )      
    {
      UR = U[i+j+k+1]; 
      UL = U[i+j+1];
      if( NA[j+1] EQ 0.0 ) 
      {
        NA[j] = saved; 
        saved = 0.0;
      } 
      else
      {
        temp  = NA[j+1]/(UR-UL);
        NA[j]  = saved + (UR-u)*temp;
        saved = (u-UL)*temp;
      }
    }
  }

  *N = NA[0];


  /* End NURBS and Exit */


  EXIT:

  return(error);

} // end sm_BasisIEval 

/* ---------------------------------------------------------------------


   DESCRIPTION:

     This tools routine computes  the knot removal  error bound for one
     removal step. That is, given the index  'r' and multiplicity  's',
     the knot  U[r]  is  removed  one time  and  the  maximum  error is 
     returned. It is  assumed  that (1) U[r] is  an  interior knot, (2) 
     U[r] != U[r+1], and (3) the multiplicity of the knot is 's > 0'. A 
     typical calling example is:

       gw_CURVE   cur;
       gw_INDEX   r, s;
       gw_REAL    br;
       ...
       (define cur, get r and s);
       ...
       N_CrvRemoveKnotMaxErr(&cur,r,s,&br);

     THE ROUTINE DOES  NOT CHECK FOR THE PROPER gw_INDEX AND  MULTIPLICITY
     OF THE KNOT. IT ASSUMES THAT THEY ARE CORRECT.

   ACCESS:
   
     cur , input  ,  NURBS curve
     r   , input  ,  Index of  knot U[r] to be removed - U[r] != U[r+1] 
                     must hold.
     s   , input  ,  Multiplicity of U[r]
     br  , output ,  Maximum error after knot removal
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     N_AllocCPt1dArray - Allocate memory for control point array
     N_DistCptCptHomo - Compute homogeneous distance
     N_CrvGetCPtsDegreeAndKnots - Get curve components
     N_CopyCPt - Initialize control point
     N_Combine2CPts - Combination of control points
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */
gw_FLAG sm_CrvRemoveKnotMaxErr 
 (gw_CURVE * cur, 
  gw_INDEX   r, 
  gw_INDEX   s, 
  gw_REAL  * br ) 
{

  gw_FLAG        error = NL_NO;

  gw_INDEX       i, j, ii, jj;

  gw_DEGREE      p;

  gw_REAL        *U, alf, oma, bet, omb;

  gw_CPOINT      *Pw, Rw[GW_MAX_DEGREE*2], A;

  /* Get local notation */
  N_CrvGetCPtsDegreeAndKnots(cur,&i,&Pw,&p,&j,&U);

  /* Compute removal error */
  i  = r-p;
  j  = r-s;
  ii = 1;
  jj = p-s+1;

  N_CopyCPt(Pw[r-p-1],&Rw[0    ]);
  N_CopyCPt(Pw[r-s+1],&Rw[p-s+2]);

  /* Get new control points for one removal step */

  while( (j-i) GT 0 )
  {
    alf = (U[i+p+1]-U[i])/(U[r]-U[i]);
    oma = 1.0-alf;
    bet = (U[j+p+1]-U[j])/(U[j+p+1]-U[r]);
    omb = 1.0-bet;
    N_Combine2CPts(alf,Pw[i],oma,Rw[ii-1],&Rw[ii]);
    N_Combine2CPts(bet,Pw[j],omb,Rw[jj+1],&Rw[jj]);
    i ++;  j --;  
    ii++;  jj--;
  }

  /* Compute error bound */

  if( (j-i) LT 0 )
  {
    N_DistCptCptHomo(Rw[ii-1],Rw[jj+1],br);
  }
  else
  {
    alf = (U[r]-U[i])/(U[i+p+1]-U[i]);
    oma = 1.0-alf;

    N_Combine2CPts(alf,Rw[jj+1],oma,Rw[ii-1],&A);
    N_DistCptCptHomo(Pw[i],A,br);
  }


  /* End NURBS and Exit */

  return(error);

} // end sm_CrvRemoveKnotMaxErr 

/* ---------------------------------------------------------------------


   DESCRIPTION:

     This routine computes one  basis  function and its  derivatives
     at a given parameter value. It is  assumed that the knot vector 
     is  clamped, i.e., end  knots are  repeated with multiplicity = 
     degree + 1. Memory  for the B-spline and derivative values must
     be  allocated in the calling routine. Maximum index is ND[der],
     where der is the highest derivative required. A typical calling
     example is:

       gw_KNOTVECTOR  knt;
       gw_INDEX       i;
       gw_DEGREE      p;
       gw_PARAMETER   u;
       gw_INDEX       der;
       gw_REAL        *ND;
       ...
       (define knt, get i, p, u, der, and allocate memory for ND);
       ...
       N_BasisIDerivs(&knt,i,p,u,LEFT,der,ND);


   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline (0<=i<=n)
     p   , input  ,  Degree 
     u   , input  ,  Parameter value
     flg , input  ,  Flag:
                       LEFT : u is in [u[j],u[j+1]) 
                              (RIGHT DERIVATIVE REQUIRED) 
                       RIGHT: u is in (u[j],u[j+1]]
                              (LEFT DERIVATIVE REQUIRED)
     der , input  ,  Maximum derivative required 
     ND  , output ,  Basis function and  derivatives computed at u.
                     MEMORY   MUST  BE  ALLOCATED  IN  THE  CALLING 
                     ROUTINE.


   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     N_KnotVectorIsParamOutOfBounds - Check parameter range
     N_KnotVectorGetKnots - Knot vector info
     N_AllocReal1dArray - Allocate memory for real array
     N_AllocReal2dArray - Allocate memory for real matrix
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */
gw_FLAG sm_BasisIDerivs
 (gw_KNOTVECTOR * knt, 
  gw_INDEX        i, 
  gw_DEGREE       p, 
  gw_PARAMETER    u, 
  gw_FLAG         flg, 
  gw_INDEX        der, 
  gw_REAL       * ND )
{

  gw_FLAG    error = NL_NO;

  gw_INDEX   j, k, l, n, m, mder;

  gw_REAL     nt_stack[(GW_MAX_DEGREE+1)*(GW_MAX_DEGREE+1)];
  gw_REAL    *nt[GW_MAX_DEGREE+1], nd[GW_MAX_DEGREE+1], *U, UL, UR, saved, temp;


  /* Get local notation */


  N_KnotVectorGetKnots(knt,&m,&U);

  n = m-p-1;


  /* Check parameter and index */


//  error = N_KnotVectorIsParamOutOfBounds(knt,u,rname);
//  if( error EQ NL_YES )  NL_OUT;

  if( i LT 0  OR  i GT n )  
  {
    smos_ErrorMessage( NL_IND_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
    NL_QUIT;
  }


  /* Allocate memory */


//  nt = N_AllocReal2dArray(p,p,&S);
//  if( nt EQ NULL )  NL_QUIT;
  for (j=0; j<=p; j++) {
      nt[j] = &nt_stack[j*(p+1)];
  }

//  nd = N_AllocReal1dArray(p,&S);
//  if( nd EQ NULL )  NL_QUIT;


  /* Compute the degree zero B-splines */


  switch( flg )
  {
    case NL_LEFT:

      if( u EQ U[m-p]  AND  i GE n-p )
      { 
        for( j=0; j<=p; j++ )
        {
          if( u GT U[i+j]  AND  u LE U[i+j+1] ) 
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
        if( u LT U[i]  OR  u GE U[i+p+1] ) 
        { 
          for( j=0; j<=der; j++ ) ND[j] = 0.0;    
          NL_OUT;
        }

        for( j=0; j<=p; j++ )
        {
          if( u GE U[i+j]  AND  u LT U[i+j+1] ) 
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

      if( u EQ U[p]  AND  i LE p )
      { 
        for( j=0; j<=p; j++ )
        {
          if( u GE U[i+j]  AND  u LT U[i+j+1] ) 
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
        if( u LE U[i]  OR  u GT U[i+p+1] ) 
        { 
          for( j=0; j<=der; j++ ) ND[j] = 0.0;    
          NL_OUT;
        }

        for( j=0; j<=p; j++ )
        {
          if( u GT U[i+j]  AND  u LE U[i+j+1] ) 
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
      {
        smos_ErrorMessage( NL_CAL_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
        NL_QUIT;
      }
  }


  /* Compute the full triangular array */


  for( k=1; k<=p; k++ )
  {
    if( nt[0][k-1] EQ 0.0 ) saved = 0.0; else
                            saved = ((u-U[i])*nt[0][k-1])/(U[i+k]-U[i]);
    for( j=0; j<p-k+1; j++ )      
    {
      UR = U[i+j+k+1]; 
      UL = U[i+j+1];
      if( nt[j+1][k-1] EQ 0.0 ) 
      {
        nt[j][k] = saved; 
        saved    = 0.0;
      } 
      else
      {
        temp     = nt[j+1][k-1]/(UR-UL);
        nt[j][k] = saved + (UR-u)*temp;
        saved    = (u-UL)*temp;
      }  
    }
  }


  /* Compute derivatives */


  ND[0] = nt[0][p];

  mder = p < der ? p:der;

  for( k=p+1; k<=der; k++ ) ND[k] = 0.0;

  for( k=1; k<=mder; k++ )
  {
    /* Load the appropriate column into the derivative array */

    for( j=0; j<=k; j++ ) nd[j] = nt[j][p-k];

    /* Compute the triangular table of width = k */

    for( l=1; l<=k; l++ )
    {
      if( nd[0] EQ 0.0 ) saved = 0.0; else
                         saved = nd[0]/(U[i+p-k+l]-U[i]);
      for( j=0; j<k-l+1; j++ )
      {
        UR = U[p-k+l+i+j+1]; 
        UL = U[i+j+1];
        if( nd[j+1] EQ 0.0 ) 
        {
          nd[j] = (p-k+l)*saved; 
          saved = 0.0;
        } 
        else
        {
          temp  = nd[j+1]/(UR-UL);
          nd[j] = (p-k+l)*(saved-temp);
          saved = temp;
        } 
      }
    }
    ND[k] = nd[0];
  }


  /* End NURBS and Exit */


  EXIT:

  return(error);

} // end gw_FLAG sm_BasisIDerivs


NL_PRIVATE  long  itl = 20;
NL_PRIVATE  long  nok = 10;


/* ---------------------------------------------------------------------


   DESCRIPTION:

     This routine  computes the global  maximum of a basis function. It 
     uses Newton iteration with the  start value obtained by bracketing 
     the root. A typical calling example is:

       gw_KNOTVECTOR  knt;
       gw_INDEX       i;
       gw_DEGREE      p;
       gw_REAL        tol, max, u;
       ...
       (define knt, get i, p and tol);
       ...
       sm_BasisFindGlobalMax(&knt,i,p,tol,&max,&u);
 

   ACCESS:
   
     knt , input  ,  Knot vector
     i   , input  ,  Index of B-spline
     p   , input  ,  Degree 
     tol , input  ,  Tolerance for convergence
     max , output ,  Global maximum
     u   , output ,  Parameter where maximum is attained


   RETURN CODES:

     0 : No error
     1 : Error saved in NERR


   EXTERNAL REFERENCES:

     N_InitNurbs - Start NURBS
     N_KnotVectorGetKnots - Knot vector info
     N_FloatOpIsBad - Floating point check
     N_BasisIDerivs - Compute basis function and derivatives
     N_EndNurbs - End nurbs

   ------------------------------------------------------------------ */

gw_FLAG sm_BasisFindGlobalMax 
 (gw_KNOTVECTOR * knt, 
  gw_INDEX        i, 
  gw_DEGREE       p, 
  gw_REAL         tol, 
  gw_REAL       * max, 
  gw_REAL       * u )

{

  gw_FLAG       conv = NL_NO, error = NL_NO;

  gw_INDEX      m, k, nos, s, it;

  gw_REAL       *U, ND[3];

  gw_PARAMETER  du, ul, ur, u0 = 0.0;

//  gw_STACKS     S;



  /* Start NURBS */


//  N_InitNurbs(&S);


  /* Get local notation and check index */


  N_KnotVectorGetKnots(knt,&m,&U);

  if( i LT 0  OR  i GT m-p-1 )  
  {
    smos_ErrorMessage( NL_IND_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
    NL_QUIT;
  }


  /* Consider linear as special case */


  if( p EQ 1 )
  {
    *max = 1.0;
    *u   = U[i+1];
    NL_OUT;
  }


  /* Check is p-fold inner multiple knot exists */


  s = 1;
  for( k=i+1; k<i+p; k++ )
  {
    if( U[k] EQ U[k+1] )  s++;
  }

  if( s EQ p )
  {
    *max = 1.0;
    *u   = U[i+1];
    NL_OUT;
  }


  /* Get guess parameter */


  nos = p*nok;
  du  = (U[i+p+1]-U[i])/nos;

  ur = U[i];
  ul = U[i]; 
  while( ur LT U[i+p+1] )
  {
    ul = ur;
    ur = ur+du;
    if( ur GT U[i+p+1] )  ur = U[i+p+1];

    error = sm_BasisIDerivs(knt,i,p,ur,NL_LEFT,1,ND);
    if( error EQ NL_YES )  NL_OUT;

    if( ND[1] LT 0.0 )  break;
  }


  /* Perform Newton interations till convergence reached */


  it = 0;
  while( it LT itl )
  {
    u0 = 0.5*(ul+ur);

    /* Do Newton with guess parameter */

    k = 0;
    while( k LT itl )
    {
      error = sm_BasisIDerivs(knt,i,p,u0,NL_LEFT,2,ND);

      if( error EQ NL_YES )  NL_OUT;

      if( fabs(ND[1]) LT tol  AND  ND[0] GT tol )  {  conv = NL_YES;  break;  }

//      if( N_FloatOpIsBad(ND[1],ND[2],NL_DIVISION) )  gw_ERROR(NUM_ERR);
      if (ND[2] == 0.0) 
      {
        smos_ErrorMessage( NL_NUM_ERR, (TCHAR *)FILE_NAME, LINE_NUMBER, NULL, NULL, FUNC_NAME );
        NL_QUIT;
      }

      u0 = u0 - (ND[1]/ND[2]);

      if( u0 LE U[i]  OR  u0 GE U[i+p+1] )  break;

      k++;
    }

    if( conv EQ NL_YES )  break;

    /* No convergence -> refine [ul,ur] and get a better guess */

    du = (ur-ul)/nok;
    ur = ul;
    for( k=1; k<=nok; k++ )
    {
      ul = ur;
      ur = ur+du;

      error = sm_BasisIDerivs(knt,i,p,ur,NL_LEFT,1,ND);
      if( error EQ NL_YES )  NL_OUT;

      if( ND[1] LT 0.0 )  break;
    }

    it++;
  }


  /* If no convergence, quit */


  if( it GE itl )  { 
      error = NL_YES;
      NL_OUT;
  }


  /* Convergence reached -> get maximum */


  *max = ND[0]; 
  *u   = u0;


  /* End NURBS and Exit */


  EXIT:

//  N_EndNurbs(&S);

  return(error);

} // end gw_FLAG 


/*******************************************************************//**
PURPOSE: return true when curve bounding box's largest side is less than tol.

NOTES: tolerance set to Max(dScaledZero,d3DTol)
***********************************************************************/   
SmBoolean sm_IsNCrvDegenerate
  (const gw_CURVE *pCur,                // in : target representation
   double          d3DTol               // in : min distance between distinct points, default:[SM_EFF_ZERO]
  )                                
{
  // For speed - build bounding box directly from NLib representation
  NL_MINMAXBOX sNLBox ;
  N_CrvGetBBox((gw_CURVE *)pCur, &sNLBox) ;
  SmExtent3d sBox(sNLBox.xl, sNLBox.yb, sNLBox.zn,
                  sNLBox.xr, sNLBox.yt, sNLBox.zf) ;
  SM_ASSERT(!sBox.HasNegativeVolume()) ;
 
  // select a tolerance - scale it to the 1st control point location
  // Was 1000 times - changed to one to fix issue when object is miles away from origin - st
  double dScaledZero = SM_EFF_ZERO * (1.0 + sBox.GetMaxDimension());
  double dTol        = smos_Max(d3DTol, dScaledZero);

 // test the bbox for point sized
  SmBoolean bDegenerate = sBox.IsPointSized(dTol) ;

  // all done
  return(bDegenerate) ;

} // end sm_IsNCrvDegenerate

/*******************************************************************//**
PURPOSE: Pretty Print gw_CURVE struct

NOTES: 
***********************************************************************/
void Dump_NCrv
 (gw_CURVE * pCur,     // in : Curve to pretty print
  SmBoolean  bAbbrev)  // in : TRUE = skip some outputs
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // locals
  gw_INDEX   lMaxCPtIndex; 
  gw_CPOINT *paCpoint; 
  gw_DEGREE  lDegree;
  gw_INDEX   lMaxKnotIndex; 
  gw_REAL   *paKnots;
  gw_FLAG    rat = N_IsCrvRat(pCur) ? 1 : 0 ;
  gw_FLAG    dim = N_CrvIs3d(pCur)  ? 3 : 2 ;

  // gw_CURVE ptr
  smos_sprintf(sBuff,_T("\n  m_pNurb(gw_CURVE):[0x%p]"),pCur);
  smos_sprintf(sBuffForFile,_T("\n  m_pNurb(gw_CURVE):[%s]"),pCur ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // no work - pCur is NULL
  if(pCur == NULL) return ;

  // ControlPtCount, Degree, RationalFlag, Dim, and NaturalInterval
  N_CrvGetCPtsDegreeAndKnots(pCur,&lMaxCPtIndex,&paCpoint,&lDegree,&lMaxKnotIndex,&paKnots);
  smos_sprintf(sBuff,_T(" CtrlPt_Cnt:[%ld], Knot_Cnt:[%d], Deg:[%d], RationalFlag:[%d], Dim:[%d], Ivl:[%16.16lf,%16.16lf]"),
             lMaxCPtIndex+1,
             lMaxKnotIndex+1,
             lDegree, 
             rat, 
             dim,
             paKnots[0],
             paKnots[lMaxKnotIndex]);
  smos_WriteBuffer(sBuff);

  // When abbreviation - skip some outputs
  ULONG increm = (bAbbrev && lMaxCPtIndex > 10) ? lMaxCPtIndex : 1 ;

  // Control Point label
  ULONG     iImageDim   = paCpoint[0].z == NL_NOZ ? 2 : 3 ;
  SmBoolean bIsRational = paCpoint[0].w == NL_NOW ? FALSE : TRUE ;
  if(iImageDim == 2)
    { if(!bIsRational) { smos_sprintf(sBuff,_T("\n  ControlPoints [CartX, CartY]  #Total:[%lu]\n"), lMaxCPtIndex+1) ; }
      else             { smos_sprintf(sBuff,_T("\n  ControlPoints [W*X, W*Y, W]  #Total:[%lu]\n"), lMaxCPtIndex+1) ; }
    }
  else
    { if(!bIsRational) { smos_sprintf(sBuff,_T("\n  ControlPoints [CartX, CartY, CartZ]  #Total:[%lu]\n"), lMaxCPtIndex+1) ; }
      else             { smos_sprintf(sBuff,_T("\n  ControlPoints [W*X, W*Y, W*Z, W]  #Total:[%lu]\n"), lMaxCPtIndex+1) ; }
    }                    
  smos_WriteBuffer(sBuff) ;

  // control points
  for (long j = 0; j<=lMaxCPtIndex; j+= increm) 
    {
      // output x, y, optional z, optional w
      smos_sprintf(sBuff,_T("   Dof[%lu]: %s[%16.16lf %16.16lf"), j, (j<10)?_T(" ") : _T(""), paCpoint[j].x,paCpoint[j].y);
      smos_WriteBuffer(sBuff);
      if (paCpoint[j].z != NL_NOZ) { smos_sprintf(sBuff,_T(" %16.16lf"),paCpoint[j].z);
                                     smos_WriteBuffer(sBuff);
                                   }
      if (paCpoint[j].w != NL_NOW) { smos_sprintf(sBuff,_T(" %16.16lf"),paCpoint[j].w);
                                     smos_WriteBuffer(sBuff);
                                   }
      smos_WriteBuffer(_T("]\n"));
    } // end iter every control point

  // output u knots
  SmTArray<double> sKnots;
  SmTArray<ULONG>  sMult;
  sm_GetKnots(pCur->knt, sKnots, &sMult);

  smos_sprintf(sBuff, _T("   Knot [value, multiplicity]  # Unique= %ld,  # Total= %ld"),
               sKnots.GetSize(),
               lMaxKnotIndex+1);
  smos_WriteBuffer(sBuff);

  increm = bAbbrev ? sKnots.GetSize()-1 : 1;
  for (ULONG i=0; i<sKnots.GetSize(); i+= increm) 
    {
      smos_sprintf(sBuff,_T("\n     %6.16lf, %ld"), sKnots[i],sMult[i]);
      smos_WriteBuffer(sBuff);
    }
  smos_WriteBuffer(_T("\n"));

  // locals
  SmBoolean bDegenerate = sm_IsNCrvDegenerate(pCur) ;

  // for Degenerate curves
  if(bDegenerate)
    {
      smos_sprintf(sBuff,_T("  NOTICE: DegenerateCurve - All %ld Control Points are coincident.\n"), lMaxCPtIndex+1);
      smos_WriteBuffer(sBuff);
      if(lMaxCPtIndex > 1)
        {
          smos_sprintf(sBuff,_T("%s") , _T("          Not best practice: consider replacing with a point or a two point degenerate curve.\n") );
          smos_WriteBuffer(sBuff);
        }
    } // end bDegenerate check

  // for nonDegenerate curves
  if(!bDegenerate)
    {
      // report any endPair ControlPoint coincidence
  
      // check for start ControlPoint coincidence
      SmVector3d sEuclid ;
      double dStartCPtDist ; 
      N_CPtToPtEuclid(pCur->pol->Pw[0], (NL_POINT *)&sEuclid) ;                
      double dStartScaledZero = SM_EFF_ZERO * (1.0 + sEuclid.GetMaxDimension()) ; 
      N_DistCptCpt( pCur->pol->Pw[0], pCur->pol->Pw[1], &dStartCPtDist) ;
      if(SM_IS_ZERO_TO_TOL(dStartCPtDist, dStartScaledZero))
        {
          smos_WriteBuffer(_T("  WARNING: BSplineCurve's 1st two control points are coincident - not an error, but not good practice.\n")) ;
        }

      // check for end ControlPoint coincidence
      NL_INDEX iN = pCur->pol->n ;
      double dEndCPtDist ;
      N_CPtToPtEuclid(pCur->pol->Pw[iN], (NL_POINT *)&sEuclid) ;                
      double dEndScaledZero = SM_EFF_ZERO * (1.0 + sEuclid.GetMaxDimension()) ; 
      N_DistCptCpt( pCur->pol->Pw[iN - 1], pCur->pol->Pw[iN], &dEndCPtDist) ;
      if(SM_IS_ZERO_TO_TOL(dEndCPtDist, dEndScaledZero))
        {
          smos_WriteBuffer(_T("  WARNING: BSplineCurve's last two control points are coincident - not an error, but not good practice.\n")) ;
        }

    } // end not degenerate check

} // end Dump_NCrv
