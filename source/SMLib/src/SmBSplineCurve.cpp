// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBSplineCurve.cpp
* PURPOSE: Implementation of SmBSplineCurve methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineCurve.h>

#include <nurbs.h>
#include <SmPolarBox.h>
#include <SmGeomUtility.h>
#include <SmLine.h>
#include <SmCircle.h>
#include <SmHermiteCurve.h>
#include <SmParabola.h>
#include <SmHyperbola.h>
#include <SmOffsetCurve.h>
#include <SmBSplineSurface.h>
#include <SmPolynomial.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#include <SmPseudoBox.h>

#ifdef SM_DEBUG_CODE
#include <SmFace.h>
#include <SmBrep.h>
#include <SmEdgeuse.h>
#include <SmCrvOnSurf.h>
// Remove Composites
// #include <SmCEdge.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Private constructor to create a spline without any nurb.

NOTES: Note that you can not do much with these splines except use
    the Set methods to load them with data.
***********************************************************************/
SmBSplineCurve::SmBSplineCurve
  (ULONG lDimension)             // in : curve dimension
 : SmCurve(lDimension), 
   m_pNurb(NULL), 
   m_bNurbIsBorrowed(FALSE),
   m_bOutOfBoundsEnabled(FALSE),
   m_eKnotType(SM_KT_UNSPECIFIED),
   m_eBSplineCurveForm(SM_CF_UNSPECIFIED) 
{
} // end SmBSplineCurve::SmBSplineCurve constructor with no NURB

/*******************************************************************//**
PURPOSE: Private constructor for fast performance when already 
   have allocated nurb.  

NOTES: Need to be very careful how you use this constructor.
   You should never attempt to edit it.
***********************************************************************/
SmBSplineCurve::SmBSplineCurve
  (gw_CURVE *pAlreadyAllocatedNurb, // in : Pointer to an existing gw_CURVE
   ULONG     lDim,                  // in : Dimension of the curve
   SmBoolean bNurbIsBorrowed,       // in : TRUE  = borrowing the nurb,
                                    //      FALSE = object owns the nurb.
   const SmContext *pContext)       // in : set this->m_cpContext when given
 : SmCurve(lDim), 
   m_pNurb(pAlreadyAllocatedNurb), 
   m_bNurbIsBorrowed(bNurbIsBorrowed), 
   m_bOutOfBoundsEnabled(FALSE),
   m_eKnotType(SM_KT_UNSPECIFIED),
   m_eBSplineCurveForm(SM_CF_UNSPECIFIED)
{ 
  SM_ASSERT(pAlreadyAllocatedNurb != NULL);
  SM_ASSERT(lDim > 0 && lDim < 4);
  if(pContext) 
    { m_cpContext = pContext ; }

  // when m_pNurb is locally owned - tune it
  if (!bNurbIsBorrowed) 
    {
      gw_CURVE *pCur = GetGwNurbPointer();
      if (pCur) 
        {
          // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
          //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
          gw_KNOTVECTOR *pKnt = pCur->knt;
          sm_FixupKnotVector(pKnt, SM_EFF_ZERO) ;
        }
    } // end owned m_pNurb check

} // end SmBSplineCurve::SmBSplineCurve fast constructor for already allocated NURBS

/*******************************************************************//**
PURPOSE:  Construct a B-Spline Curve given NLib curve data
    and the dimension.  

NOTES: 
***********************************************************************/  
SmBSplineCurve::SmBSplineCurve
  (ULONG             lDim,     // in : dimension
   ULONG             n,        // in : high index of Pw
   ULONG             m,        // in : high index of U, m = n + p + 1
   short             p,        // in : degree
   gw_CPOINT       * Pw,       // in : Control polygon
   double          * U,        // in : knots
   const SmContext * pContext) // in : set this->m_cpContext when given
 : SmCurve(lDim),
   m_pNurb(NULL), 
   m_bNurbIsBorrowed(FALSE),
   m_bOutOfBoundsEnabled(FALSE),
   m_eKnotType(SM_KT_UNSPECIFIED),
   m_eBSplineCurveForm(SM_CF_UNSPECIFIED) 
{ 
  if(pContext) 
    { m_cpContext = pContext ; }

  // check input - no asserts in constructors
  if(m != n + p + 1)
    { WARN(_T("SmBSpline Constructor: m != n+p+1; make KnotCount, ControlPointCount, and Degree compatible")) ; }

  gw_CURVE *pNewCur = sm_AllocateNurbCurve(n, p, m);

  // okay to use smos_MemCpy on base (double) and static class (gw_CPOINT) objects.
  SE(smos_MemCpy(pNewCur->knt->U, U, sizeof(gw_REAL) * ((size_t)m + (size_t)1), sizeof(gw_REAL) * ((size_t)m + (size_t)1)));
  SE(smos_MemCpy(
      pNewCur->pol->Pw, Pw, sizeof(gw_CPOINT) * ((size_t)n + (size_t)1), sizeof(gw_CPOINT) * ((size_t)n + (size_t)1)));

  m_pNurb = pNewCur;

  if (pNewCur) 
    {
      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnt = pNewCur->knt;
      sm_FixupKnotVector(pKnt, SM_EFF_ZERO) ;
    }

} // end SmBSplineCurve::SmBSplineCurve constructor from NLib curve data

/*******************************************************************//**
PURPOSE:  Construct a B-Spline Curve given a pointer to a NLib
   NURB curve and the corresponding dimension.  

NOTES: The NLib curve will
   not be consumed but will be copied.
   
   The resulting curve will be projected to the XY plane or 
   the X line if the dimension of the input curve is not the same as the 
   dimension asked for.

***********************************************************************/
SmBSplineCurve::SmBSplineCurve
  (ULONG lDim,                // in : dimension; 1,2,3 are currently valid 
   const gw_CURVE *cpGwNurb)  // in : NLib curve
 : SmCurve(lDim),
   m_pNurb(NULL), 
   m_bNurbIsBorrowed(FALSE),
   m_bOutOfBoundsEnabled(FALSE),
   m_eKnotType(SM_KT_UNSPECIFIED),
   m_eBSplineCurveForm(SM_CF_UNSPECIFIED) 
{ 
  SM_ASSERT(cpGwNurb != NULL);
  SM_ASSERT(lDim > 0 && lDim < 4);

  // This routine allocates a single piece of memory to
  // contain the nurb curve.  The following order is used
  // to map the memory to the gw_CURVE structure:
  //    gw_CURVE
  //    gw_CPOLYGON
  //    gw_KNOTVECTOR
  //    <array of double for knots>
  //    <array of double*4 for CPOINTS>
  const gw_CURVE *pSrcCur = SM_REINTERPRET_CAST(const gw_CURVE*,cpGwNurb);
  SM_ASSERT(m_pNurb == NULL) ;
  m_pNurb = sm_AllocateAndCopyNurbCurve(pSrcCur);

  if (m_pNurb) 
    {
      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnt = m_pNurb->knt;
      sm_FixupKnotVector(pKnt, SM_EFF_ZERO) ;
    }

} // end SmBSplineCurve::SmBSplineCurve Std constructor from NLib NURB curve

/*******************************************************************//**
PURPOSE: Copy constructor for a B-Spline curve.

NOTES: 
    This constructor should only be used in the case of a new allocated
    curve or one which is being declared on the stack.  It should not
    be used to assign to a preexisting curve.
***********************************************************************/
SmBSplineCurve::SmBSplineCurve
  (const SmBSplineCurve & crSourceCurve) // in : Curve to copy
: SmCurve(crSourceCurve), 
  m_pNurb(NULL),
  m_bNurbIsBorrowed(FALSE),
  m_bOutOfBoundsEnabled(crSourceCurve.m_bOutOfBoundsEnabled),
  m_eKnotType(crSourceCurve.m_eKnotType),
  m_eBSplineCurveForm(crSourceCurve.m_eBSplineCurveForm) 
{

  if (m_pNurb && !m_bNurbIsBorrowed) 
    {
      smos_Free(m_pNurb);
      m_pNurb = NULL;
    }

  const gw_CURVE *pSrcCur = crSourceCurve.GetGwNurbPointer();

  // GWC - changed logic - analytic curves can lazy evaluate their m_pNurb pointers
  if(pSrcCur != NULL)
    {
      // GWC - no longer need this check
      // SM_ASSERT(pSrcCur != NULL) ;
      // if (!pSrcCur) 
      //   { SE(SM_ERR); }
  
      SM_ASSERT(m_pNurb == NULL) ;
      m_pNurb = sm_AllocateAndCopyNurbCurve(pSrcCur);

      if (m_pNurb) 
        {
          // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
          //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
          gw_KNOTVECTOR *pKnt = m_pNurb->knt;
          sm_FixupKnotVector(pKnt, SM_EFF_ZERO) ;
        }
    } // end SourceCurve.m_pNurb is not NULL check

} // end SmBSplineCurve::SmBSplineCurve copy constructor

/*******************************************************************//**
PURPOSE: Default destructor for B-Spline curves.

NOTES:  
***********************************************************************/
SmBSplineCurve::~SmBSplineCurve()
{
  if(!m_bNurbIsBorrowed)
    {
      if (m_pNurb) { smos_Free(m_pNurb) ; m_pNurb = NULL ; }
    }

} // end SmBSplineCurve::~SmBSplineCurve destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmBSplineCurve

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmBSplineCurve::operator==
  (const SmCurve& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmCurve::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmBSplineCurve &rOther = (SmBSplineCurve &)crOther ;

      // check equivalence of these objects
      bRtn =    ( m_pNurb == rOther.m_pNurb)
             || ( m_pNurb == NULL && rOther.m_pNurb == NULL)
             || (   m_pNurb != NULL && rOther.m_pNurb != NULL
                 && N_CrvsAreEqual(m_pNurb, rOther.GetGwNurbPointer(), SM_EFF_ZERO, SM_EFF_ZERO )) ;
    }

  // all done
  return bRtn ;

} // end SmBSplineCurve::operator==

/*******************************************************************//**
PURPOSE: support sm_SplitQuadraticBezier

NOTES:
***********************************************************************/
#define SM_CPOINT_AVE(a1,a2,result) \
    (result).x = ((a1).x+(a2).x)/2.0; \
    (result).y = ((a1).y+(a2).y)/2.0; \
    if ((a1).z != NL_NOZ) { (result).z = ((a1).z+(a2).z)/2.0; } \
    else { (result).z = NL_NOZ; } \
    if ((a1).w != NL_NOW) { (result).w = ((a1).w+(a2).w)/2.0; } \
    else { (result).w = NL_NOW; }

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static void sm_SplitQuadraticBezier
  (gw_CPOINT & rCP1,       // in : start control-polygon vertex, curve start point 
   gw_CPOINT & rCP2,       // in : mid   control-polygon vertex
   gw_CPOINT & rCP3,       // in : end   control-polygon vertex, curve end point
   gw_CPOINT * paCPoints)  // out: control-polygon after splitting, sized:[5]
{
    paCPoints[0] = rCP1;
    SM_CPOINT_AVE(rCP1,rCP2,paCPoints[1]);
    paCPoints[4] = rCP3;
    SM_CPOINT_AVE(rCP2,rCP3,paCPoints[3]);
    SM_CPOINT_AVE(paCPoints[1],paCPoints[3],paCPoints[2]);

} // end sm_SplitQuadraticBezier

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static void sm_SplitCubicBezier
  (gw_CPOINT & rCP1,        // in : start   control-polygon vertex, curve start point
   gw_CPOINT & rCP2,        // in : 1st mid control-polygon vertex, sets curve start tangent
   gw_CPOINT & rCP3,        // in : 2nd mid control-polygon vertex, sets curve end tangent
   gw_CPOINT & rCP4,        // in : last    control-polygon vertex, curve end point
   gw_CPOINT * paCPoints)   // out: control-polygon after splitting, sized:[7]
{
  paCPoints[0] = rCP1;
  SM_CPOINT_AVE(rCP1,rCP2,paCPoints[1]);
  paCPoints[6] = rCP4;
  SM_CPOINT_AVE(rCP3,rCP4,paCPoints[5]);
  gw_CPOINT sMid;
  SM_CPOINT_AVE(rCP2,rCP3,sMid);
  SM_CPOINT_AVE(paCPoints[1],sMid,paCPoints[2]);
  SM_CPOINT_AVE(sMid,paCPoints[5],paCPoints[4]);
  SM_CPOINT_AVE(paCPoints[2],paCPoints[4],paCPoints[3]);

} // end sm_SplitCubicBezier
#undef SM_CPOINT_AVE

/*******************************************************************//**
PURPOSE: Given an interval on the curve, compute the axis aligned, and/or
    non-axis aligned, and/or polar bounding boxes.

NOTES: At least one of the outputs must be non-NULL.

METHOD --- find the subSet of spans that contribute to the target
  interval and work directly from the control Polygon to build
  bounding boxes.

  special cases:
    lines,
    quadratic beziers (common in caches of quadratic curves),
    cubic beziers     (common in caches of cubic curves).

***********************************************************************/
SmStatus SmBSplineCurve::CalculateBoundingBox
  (const SmExtent1d & crInterval,  // in : target interval
   SmExtent3d       * pNormalBox,  // out: Axis aligned box                       
   SmPseudoBox      * pPseudoBox,  // out: Non-axis aligned box                    
   SmPolarBox       * pPolarBox,   // out: Surface normal vector field bounding box
   SmBoolean)                      // in : bExpandBox = not-used
                                   //      TRUE = Expand BBox before return
                                   //      FALSE= don't
                                   //      default:[TRUE]
  const
{ 
  // no work - nothing to compute
  if(   pNormalBox == NULL 
     && pPseudoBox == NULL
     && pPolarBox  == NULL)
    { return SM_SUCCESS ; }

  // locals
  SmExtent1d         sNaturalInterval = GetNaturalInterval();
  const gw_CURVE    *pCur             = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();

  // error - pCur is NULL
  if(pCur == NULL)
    {
      ULONG ii ;
      SmPoint3d sPV[2] ;
      // Should catch this error and make sure every derived class has its ownCalculateBoundingBox() method
      // for now - just approx bounding box and signal an error
      if (pNormalBox) { pNormalBox->Init() ; }
      if (pPseudoBox) { SmVector3d sX(1,0,0), sY(0,1,0), sZ(0,0,1) ;
                        pPseudoBox->Init() ;   // init intervals - leave basis vectors alone
                        pPseudoBox->SetBasis(sX, sY, sZ) ;
                      }
      if (pPolarBox)  { pPolarBox->ReSet() ; }
      for(ii=0;ii<5;ii++)
        {
          this->Evaluate(crInterval.Evaluate((double)ii/5.0), 1, TRUE, sPV) ;  // NonZeroTangents
          if (pNormalBox) { pNormalBox->AddPoint3d(sPV[0]) ; }
          if (pPseudoBox) { pPseudoBox->AddPoint3d(sPV[0]) ; }
          if (pPolarBox)  { pPolarBox->AddVector3d(sPV[1]) ; }
        } // end iter some sample points

      SE_MSG(SM_ERR, _T("Derived type needs a CalculateBoundingBox method - computing approx bounding box instead")) ;
      return(SM_SUCCESS) ;
    } // end thisCurve has a BSplineCurve representation check

  const gw_CPOLYGON *pCPolygon        = pCur->pol; // Use natural domain
  ULONG lStartPoint=0, lNumPoints=0;

  // Build target interval by trimming input interval to natural interval
  SmExtent1d sInterval(sNaturalInterval.ClampValue(crInterval.GetMin()),
                       sNaturalInterval.ClampValue(crInterval.GetMax()));
  
  // get the startPoint and the numberPoints of the control polygon
  // that contribute to the target interval's shape
  // If the interval is equal to the natural interval then we don't need
  // to chop it up and use a sub segment.
  if (   sNaturalInterval.AreEqual(sInterval, SM_EFF_ZERO)
      || pCur->pol->n == 0)
    {
      lStartPoint = 0;
      lNumPoints  = pCur->pol->n + 1;
    }
  else 
    { // use control polygon which contributes to the included spans
      gw_INDEX lStartSpan, lEndSpan;

      // find Spans for Interval Min and Max
      if(NL_YES == N_BasisFindSpan(pCur->knt,pCur->p,sInterval.GetMin(),NL_LEFT, &lStartSpan))
        { SER(SM_ERR); }

      if(NL_YES == N_BasisFindSpan(pCur->knt,pCur->p,sInterval.GetMax(),NL_RIGHT,&lEndSpan))
        { SER(SM_ERR); }

      // GW_SER(N_BasisFindSpan(pCur->knt,pCur->p,sInterval.GetMin(),NL_LEFT, &lStartSpan));
      // GW_SER(N_BasisFindSpan(pCur->knt,pCur->p,sInterval.GetMax(),NL_RIGHT,&lEndSpan));

      lStartPoint = lStartSpan - pCur->p;
      lNumPoints  = lEndSpan - lStartPoint + 1;
    }

  if ( lNumPoints < 1 )
    { SER( SM_ERR ); }

  // init PseudoBox: Compute the basis vectors of the PseudoBox using the control polygon
  //                 running from the start span to the end span
  if (pPseudoBox) 
    {
      // init intervals - leave basis vectors alone
      pPseudoBox->Init() ; 

      // locals to find control point triple with a near-max cross product (max base * height size)
      //  looking at a subset of the control points to limit the cost of this computation
      ULONG ii, jj, kk, iMax=1, jMax=2, kMax=3 ;
      SmPoint3d sP0, sP1, sP2; 
      double dThisCross, dMaxCross = 0.0 ;
      ULONG lMaxSmp = 7 ;                                            // max number of control points to check
      ULONG lSmpCnt = lNumPoints > lMaxSmp ? lMaxSmp : lNumPoints ;  // actual number of control points to check
      ULONG lInc    = lNumPoints / lSmpCnt ;
      SM_ASSERT(lInc > 0) ;
      SM_ASSERT(lInc * lSmpCnt <= lNumPoints) ;

      // look for a good non-colinear set of control points
      switch(lNumPoints)
        { 
          case 0:   iMax = 0 ; jMax = 0 ; kMax = 0 ; break ;
          case 1:   iMax = 0 ; jMax = 0 ; kMax = 0 ; break ;
          case 2:   iMax = 0 ; jMax = 0 ; kMax = 1 ; break ;
          case 3:   iMax = 0 ; jMax = 1 ; kMax = 2 ; break ;
          default:
            for(ii=0;ii<lSmpCnt-2;ii++)
              {
                gw_CPOINT *pCP0 = &pCPolygon->Pw[ii*lInc];
                TO_EUCLID(*pCP0,  sP0);
             
                for(jj=ii+1;jj<lSmpCnt-1;jj++)
                  {
                    gw_CPOINT *pCP1 = &pCPolygon->Pw[jj*lInc];
                    TO_EUCLID(*pCP1,  sP1);

                    for(kk=jj+1;kk<lSmpCnt;kk++)
                      {
                        gw_CPOINT *pCP2 = &pCPolygon->Pw[kk*lInc];
                        TO_EUCLID(*pCP2,  sP2);

                        smgu_TriangleAreaSquared(sP0, sP1, sP2, dThisCross) ;
                        if(dThisCross > dMaxCross)
                          {
                            dMaxCross = dThisCross ;
                            iMax = ii ;
                            jMax = jj ;
                            kMax = kk ;
                          }
                      } // end iter kk
                  } // end iter jj
              } // end iter ii
            break ;
        } // end switch on lNumPoints

      gw_CPOINT *pCP ;
      pCP = &pCPolygon->Pw[iMax*lInc]; TO_EUCLID(*pCP, sP0);
      pCP = &pCPolygon->Pw[jMax*lInc]; TO_EUCLID(*pCP, sP1);
      pCP = &pCPolygon->Pw[kMax*lInc]; TO_EUCLID(*pCP, sP2);

      SmVector3d sV0      = sP1 - sP0 ;
      SmVector3d sV1      = sP2 - sP0 ;
      double     sV0LenSq = sV0.LengthSquared() ;
      double     sV1LenSq = sV1.LengthSquared() ;

      // set sV0 larger than sV1
      if(sV1LenSq > sV0LenSq)
        {
          SmVector3d sTmp = sV0 ;
          sV0             = sV1 ;
          sV1             = sTmp ;
          
          double dTmp = sV0LenSq ;
          sV0LenSq    = sV1LenSq ;
          sV1LenSq    = dTmp ;
        }

      // Note that we will just use the standard basis vectors for degenerate curves
      if(sV0LenSq < SM_EFF_ZERO_SQ)
        {
          // use default basis vectors
          *pPseudoBox = SmPseudoBox(); 
        }
      else
        { 
          SmVector3d sBasis1, sBasis2, sBasis3;
          sV0.MakeUnitOrthoVectors(&sV1,sBasis1,sBasis2,sBasis3);
          pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
        }
    } // end if pPseudoBox check
  
  // obsolete - select pseudo-box basis without optimization
  //      // init PseudoBox: Compute the basis vectors of the PseudoBox using the control polygon
  //      //                 running from the start span to the end span
  //      if (pPseudoBox) 
  //        {
  //          // initialize the PseudoBox Size
  //          pPseudoBox->Init() ; 
  //      
  //          // Use first, mid, and last points to try to compute a basis vector set
  //          // If we are unable to then we need to just use the default vectors
  //          ULONG l1 = 0 , l2 = lNumPoints/2, l3 = lNumPoints - 1 ;
  //      
  //          SmPoint3d sP0, sPn, sPmid; 
  //          gw_CPOINT *pCP0   = &pCPolygon->Pw[l1   ];
  //          gw_CPOINT *pCPn   = &pCPolygon->Pw[l1+l3];
  //          gw_CPOINT *pCPmid = &pCPolygon->Pw[l1+l2];
  //          TO_EUCLID(*pCP0,  sP0);
  //          TO_EUCLID(*pCPn,  sPn);
  //          TO_EUCLID(*pCPmid,sPmid);
  //      
  //          SmVector3d sV1 = sPn - sP0;
  //      
  //          // for closed curves - try different sample points to generate sV1
  //          if (sP0.CloserThan(SM_EFF_ZERO, sPn))
  //            { switch(lNumPoints)
  //                { 
  //                  case 0:
  //                  case 1:   l1 = 0 ; l2 = 0 ;        l3 = 0 ;          break ;
  //                  case 2:   l1 = 0 ; l2 = 0 ;        l3 = 1 ;          break ;
  //                  case 3:   
  //                  case 4:   l1 = 0 ; l2 = 1 ;        l3 = 2 ;          break ;
  //                  default:  l1 = 1 ; l2 = lNumPoints/2 ; l3 = lNumPoints - 2 ; break ;
  //                }
  //      
  //              pCP0   = &pCPolygon->Pw[l1   ];
  //              pCPn   = &pCPolygon->Pw[l1+l3];
  //              pCPmid = &pCPolygon->Pw[l1+l2];
  //              TO_EUCLID(*pCP0,  sP0);
  //              TO_EUCLID(*pCPn,  sPn);
  //              TO_EUCLID(*pCPmid,sPmid);
  //      
  //              sV1 = sPn - sP0 ;
  //            }
  //      
  //          // Note that we will just use the standard basis vectors if the curve is
  //          // closed.  If the mid point lies on same line as the start and end point
  //          // then we will get two arbitrary vectors for Basis2 and Basis3 otherwise V2 will
  //          // determine the direction for Basis2 and orthogonal direction for Basis3
  //      
  //          if (sP0.CloserThan(SM_EFF_ZERO, sPn))
  //      
  //            {
  //              // use default basis vectors
  //              *pPseudoBox = SmPseudoBox(); 
  //            }
  //          else
  //            { 
  //              // use basis vectors based on selected sample points 
  //              SmVector3d sV2 ;
  //              if(!sP0.CloserThan(SM_EFF_ZERO, sPmid)) sV2 = sPmid - sP0; 
  //              else                                    sV2 = sPn - sPmid ;
  //               
  //              SmVector3d sBasis1, sBasis2, sBasis3;
  //              sV1.MakeUnitOrthoVectors(&sV2,sBasis1,sBasis2,sBasis3);
  //              pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
  //            }
  //        } // end if pPseudoBox check
  // end Obsolete
    
  // init polarBox
  if(pPolarBox) { pPolarBox->ReSet() ; }

  // set pCPoint equal to first ControlPolygon Vertex being used
  gw_CPOINT *pCPoint = &pCPolygon->Pw[lStartPoint];
  gw_CPOINT aCPts[7];

  // fine-tune bounding boxes of quadratic bezier curves -
  //   instead of using the bezier's control-polygon,
  //   split once and use the tighter control-polygon for the 2 spans

  // if pCur is a quadratic bezier curve
  if (    ((int)pCur->p == (int) pCur->pol->n) 
       && ((int)pCur->p == (int)(lNumPoints-1))) 
    {
      SmPoint3d sPnt;
      TO_EUCLID(*pCPoint,sPnt);

      // we have a Bezier span, if line then recompute from start,end
      double dTol = SM_EFF_ZERO * (1.0 + sPnt.GetMaxDimension());
      SmPoint3d sLinePnt, sLinePnt2 ;
      SmVector3d sLineVec;

      // note: can be a degree 2 or 3 'line'
      if (this->IsLine(4, dTol, sLinePnt, sLineVec))
        {
          // compute line's bboxes and return
          this->EvaluatePoint(crInterval.GetMin(), sLinePnt);
          if (pNormalBox) { *pNormalBox = SmExtent3d(sLinePnt); }
          if (pPseudoBox) { pPseudoBox->AddPoint3d(sLinePnt); }

          this->EvaluatePoint(crInterval.GetMax(), sLinePnt2); 
          if (pNormalBox) { pNormalBox->AddPoint3d(sLinePnt2); }
          if (pPseudoBox) { pPseudoBox->AddPoint3d(sLinePnt2); }
          if (pPolarBox)  { pPolarBox->AddVector3d(sLinePnt2 - sLinePnt) ; }

          return (SM_SUCCESS);

        } // end isLine check

      // Optimization for quadratic bezier curves 
      if (pCur->p == 2) 
        {
          // make 5 vertex control-polygon after splitting current 3 point control-polygon
          sm_SplitQuadraticBezier(pCPolygon->Pw[0],
                                    pCPolygon->Pw[1],
                                    pCPolygon->Pw[2],
                                    aCPts);
          // Remove middle point, its just the average of points 1 and 3
          //                      and adds no information to any bounding box
          aCPts[2] = aCPts[3]; 
          aCPts[3] = aCPts[4];
          lNumPoints = 4;

          // set pCPoint to point into this array
          pCPoint = &aCPts[0];
        }   

      // Optimization for cubic bezier curves - subdivide once
      else if (pCur->p == 3) 
        {
          // make 7 vertex control-polygon after splitting current 4 point control-polygon
          sm_SplitCubicBezier(pCPolygon->Pw[0],pCPolygon->Pw[1],
                              pCPolygon->Pw[2],pCPolygon->Pw[3],aCPts);

          // Remove middle point, its just the average of points 2 and 4
          //                      and adds no information to any bounding box
          aCPts[3] = aCPts[4]; 
          aCPts[4] = aCPts[5]; 
          aCPts[5] = aCPts[6];
          lNumPoints = 6;
          
          // set pCPoint to point into this array
          pCPoint = &aCPts[0];
        }   
  } // end pCur is a qudratic bezier curve check

  // convert starting control-polygon vertex to euclidean coordinates
  SmPoint3d sLastPoint, sPoint;
  TO_EUCLID(pCPoint[0],sPoint);

  // init pBBox
  if(pNormalBox) { *pNormalBox = SmExtent3d(sPoint); }

  // arrive here after all the BBoxes have been initialized
  //   and control-polygons for quadratic and cubic beziers have
  //       been refined by a 1 step sub-division.

  // Build the BBoxes by adding in remaining control-polygon vertices
  for (ULONG i=0; i<lNumPoints; i++)
    {
      TO_EUCLID(pCPoint[i],sPoint);
      if(pNormalBox) { pNormalBox->AddPoint3d(sPoint);
                     }
      if(pPseudoBox) { pPseudoBox->AddPoint3d(sPoint);
                     }
      if(pPolarBox)  { if(i>0) { pPolarBox->AddVector3d(sPoint - sLastPoint) ; }
                       sLastPoint = sPoint ;
                     }

    } // end iter every control point

  return SM_SUCCESS;

} // end SmBSplineCurve::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Copy a SmBSplineCurve in a generic way.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::Copy
  (const SmContext & crContext,
   SmCurve        *& rpNewCurve) 
  const
{
  rpNewCurve = new (crContext) SmBSplineCurve(*this);
  NER(rpNewCurve);
  return SM_SUCCESS;

} // end SmBSplineCurve::Copy

/*******************************************************************//**
PURPOSE: Copy a SmBSplineCurve in a generic way.

NOTES:
***********************************************************************/
SmStatus SmBSplineCurve::Copy
(const SmContext& crContext,
    SmBSplineCurve*& rpNewCurve)
    const
{
    rpNewCurve = new (crContext) SmBSplineCurve(*this);
    NER(rpNewCurve);
    return SM_SUCCESS;

} // end SmBSplineCurve::Copy


/*******************************************************************//**
PURPOSE: Copy this curve as a SmBSplineCurve if possible - else
            set output to NULL

NOTES: The default behavior is to just copy the curve.
***********************************************************************/
SmStatus SmBSplineCurve::CopyAnalyticAsNurb
  (const SmContext & crContext,     // in : context for new object construction
   SmCurve        *& rpNewCurve)    // out: newly copied SmBSplineCurve of appropriate
                                    //      analytic derived type when analytic
 const
{
  if(IsKindOf(SmBSplineCurve_TYPE))
    { 
      return(SmBSplineCurve::Copy(crContext, rpNewCurve)) ; 
    }
  else 
    { rpNewCurve = NULL ; 
      return SM_SUCCESS ;
    }

} // end SmBSplineCurve::CopyAnalyticAsNurb

/*******************************************************************//**
PURPOSE: When an SmBSplineCurve can be represented by Analytic Curve
     to SM_EFF_ZERO tolerances, create and return a derived analytic 
     object (SmLine, SmCircle) equivalent. 

NOTES: Will only generate an equivalent when the curve is 
   geometrically and parametrically equivalent to the corresponding analytic.

   rpNewAnalyticCurve is set as follows 
     1. to new SmLine Object when input Curve has
          degree == 1 and ControlPointCount == 2.
     2. to new SmCircle object when input Curve has
          degree == 2, IsAnalytic(), passes the IsArc() geometry test, and
          endpoints are within tolerance of the analytic curve.
     3. to NULL for all other cases.
***********************************************************************/
SmStatus SmBSplineCurve::CreateAnalyticCurve
 (const SmContext  & crContext,           // in : context for construction 
  const SmExtent1d & crInterval,          // in : interval for review
  SmCurve         *& rpNewAnalyticCurve)  // out: Set to new SmLine or SmCircle when 
 const                                    //      this curve can be represented by such otherwise
                                          //      set to NULL.
{
  // init outputs
  rpNewAnalyticCurve = NULL;

  // don't build analytics when they are turned off
#ifndef USE_ANALYTICS
  return(SM_SUCCESS) ;
#endif

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      this->Dump();

      SmEdge *pEdge = (SmEdge *)GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  SmBSplineCurve *pAnalytic = NULL;
  SmObjDelete     sClean;
  SmPoint3d       sLinePnt;
  SmVector3d      sLineVec;

  // use midPoint to set dScaledZero
  SmPoint3d sMidPnt;
  SER(EvaluatePoint(crInterval.Evaluate(0.456789),sMidPnt));
  double dScaledZero = SM_EFF_ZERO * (1.0 + sMidPnt.GetMaxDimension());

  // check for a line bspline( degree = 1; controlPointCount ==2)
  if (GetDegree() == 1 && GetNumberControlPoints() == 2) 
    {
      // the IsLine check will always pass because of the previous definition check
      if (IsLine(10,dScaledZero,sLinePnt,sLineVec)) 
        {
           // lineVec = EndPoint - StartPoint and if lineVec=0  IsLine=FALSE 
           SmPoint3d  sLineOrigin = sLinePnt - crInterval.GetMin() * 
                                    (sLineVec / crInterval.GetLength());
           SmVector3d sLineVector = sLineVec  / sLineVec.Length();
           double     dScale      = sLineVec.Length() / crInterval.GetLength();
           ULONG      lDimension  = GetDim();
           pAnalytic = new (crContext) SmLine(sLineOrigin, sLineVector, crInterval,
                                              dScale, lDimension);
           sClean.SetObj(pAnalytic);
        }
   // else return ( SM_SUCCESS); 
    } // end line check
  else 
    { // check for circular arcs
      SmAxis2Placement sRefFrame;
      double dRadius, dStartAngle, dEndAngle;
      if (   GetDegree() == 2 
          && IsRational() 
          && IsArc(20,dScaledZero,sRefFrame,dRadius,dStartAngle,dEndAngle)) 
        { 
          // whole curve is circular arc
          SmExtent1d sNurbIvl = GetNaturalInterval() ;

          // when given interval is less than natural interval
          //      compute analytic interval for given interval
          if (!sNurbIvl.IsContainedBy(crInterval)) 
            {
              // evaluate endPoints and endVecs
              SmPoint3d sStartPoint, sEndPoint ;
              EvaluatePoint(crInterval.GetMin(), sStartPoint) ;
              EvaluatePoint(crInterval.GetMax(), sEndPoint) ;
              SmVector3d sStartVec = sStartPoint - sRefFrame.GetOriginRef() ;
              SmVector3d sEndVec   = sEndPoint - sRefFrame.GetOriginRef() ;
              SM_ASSERT(SM_ARE_SAME(sStartVec.Length(), dRadius)) ;
              SM_ASSERT(SM_ARE_SAME(sEndVec.Length(),   dRadius)) ;

              // get angleRad to start and end Vecs
              SmVector3d sZAxis = sRefFrame.GetZAxis() ;
              double dStartAngRad, dEndAngRad ;
              sZAxis.CCWAngleBetween(sRefFrame.GetXAxisRef(), sStartVec, dStartAngRad) ;
              sZAxis.CCWAngleBetween(sRefFrame.GetXAxisRef(), sEndVec,   dEndAngRad) ;
              
              // get angleDeg in interval [0 360] for start and end vecs
              double dStartAngDeg = dStartAngRad < 0.0 ? dStartAngRad * 180.0 / SM_PI  
                                                       : dStartAngRad * 180.0 / SM_PI + 360.0 ;
              double dEndAngDeg   = dEndAngRad   < 0.0 ? dEndAngRad   * 180.0 / SM_PI   
                                                       : dEndAngRad   * 180.0 / SM_PI + 360.0 ;
              if(dEndAngDeg < dStartAngDeg) { dEndAngDeg += 360.0 ; }

              // set the anal interval bounds
              dStartAngle = dStartAngDeg ;
              dEndAngle   = dEndAngDeg ;
            } // end get anal interval for given trim interval branch

          // SMLib rule: circles are represented as
          //  degree2 rational BSplines with all internal knots doubled.

          SmBSplineCurve *pCrv = (SmBSplineCurve *)this ;
          SmObjDelete sCrvClean;

          // when all internal knots are not multiple = 2
          SmTArray<double> sKnots ;
          SmTArray<ULONG> sMultiplicities ;
          GetKnots(sKnots, &sMultiplicities);
          for (ULONG i=1; i+1<sMultiplicities.GetSize(); i++)  // note: can't say sMultiplicities.GetSize()-1
            {
              // copy the BSplineCurve and make its knots double
              if (sMultiplicities[i] != 2) 
                {
                  // make a temprorary BSplineCopy so it can be modified
                  if(pCrv == this) 
                    { pCrv = new (crContext) SmBSplineCurve(*this) ;
                      sCrvClean.SetObj(pCrv) ;
                    }
                    
                  // double the knot
                  SM_ASSERT(sMultiplicities[i] == 1) ;
                  pCrv->InsertOneKnot(sKnots[i], 1) ; 
                } // end need to double a knot check
            } // end iter all internal knot check

          // create SmCircle from this SmBSplineCurve
          SmExtent1d sAngleIvl(dStartAngle,dEndAngle);
          ULONG lDimension = GetDim();
          pAnalytic = new(crContext) SmCircle(sRefFrame.GetOriginRef(), 
                                              sRefFrame.GetXAxisRef(),
                                              sRefFrame.GetYAxisRef(), 
                                              sAngleIvl, dRadius, lDimension, &crContext, 
                                              pCrv);
          SM_DUMP_AND_ASSERT2_VALID(pAnalytic) ;
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              Dump() ;
              pAnalytic->Dump() ;
            }
#endif // SM_DEBUG_CODE

          sClean.SetObj(pAnalytic);
        }
    } // end circular arc check

  // when an analytic derived object was created
  if (pAnalytic) 
    {
      // if original and analytic given interval begin endpoints are too far apart 
      SmPoint3d sOrPnt, sAnPnt;
      SER(EvaluatePoint(crInterval.GetMin(),sOrPnt));
      SER(pAnalytic->EvaluatePoint(crInterval.GetMin(),sAnPnt));
      if (sOrPnt.DistanceBetween(sAnPnt) > dScaledZero) 
        {
          // don't return the analytic object
          return SM_SUCCESS;
        }
      
      // if original and analytic end given interval endpoints are too far apart 
      SER(EvaluatePoint(crInterval.GetMax(),sOrPnt));
      SER(pAnalytic->EvaluatePoint(crInterval.GetMax(),sAnPnt));
      if (sOrPnt.DistanceBetween(sAnPnt) > dScaledZero) 
        {
          // don't return the analytic object
          return SM_SUCCESS;
        }

      // get here when orig endpoints are close to analytic endpoints
      // return the analytic object
      sClean.Clear();
      rpNewAnalyticCurve = pAnalytic;
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateAnalyticCurve

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the 
    continuities at each of the unique knots
    
NOTES:       It includes the first and
    last knots which should always be clamped.  The minimum continuity of
    the curve will be be the minimum continuity of the internal knots.
    The end knots will always be discontinuous.

    Note that the tolerances have been loosened to accomodate
    for translation to single precision.  Basically we try to get 7 decimal
    places of accuracy. 
    We allow optional choice of dContinuityAngleTol ( default is 
    SM_CONTINUITY_ANGLE set (=1.0)  in SmConfig.h
***********************************************************************/
SmStatus SmBSplineCurve::CalculateContinuities
  (SmContinuityType           & reMinContinuityInCurve,
   SmTArray<SmContinuityType> & rContinuitiesAtKnots,
   double                       dContinuityAngleTol) 
  const
{
  // ensure a pNurb
  if(m_pNurb == NULL) { ((SmBSplineCurve *)this)->MakeNurb(); } 
  SM_ASSERT(m_pNurb != NULL) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      Dump();
    }
#endif // SM_DEBUG_CODE

  // Get knot vector
  double           sdData[256];
  ULONG            slData[256];
  SmTArray<double> sKnots(256,sdData);
  SmTArray<ULONG>  sKnotMultiplicities(256,slData);
  GetKnots(sKnots,&sKnotMultiplicities);

  // init output - 1 continuity for each knot, end continuity values SM_CT_DISCONTINUOUS
  rContinuitiesAtKnots.SetSize(sKnots.GetSize());
  rContinuitiesAtKnots[0]                  = SM_CT_DISCONTINUOUS;
  rContinuitiesAtKnots[sKnots.GetSize()-1] = SM_CT_DISCONTINUOUS;
  reMinContinuityInCurve = SM_CT_CINFINITY;

  // Note that the best way to figure these things out is to actually
  // measure the values.  If we rely only on the multiplicities then
  // we could be fooled by coincident control points stacking up on
  // each other.  Also we would like to classify joints between Bezier
  // segments as being C1 or C2 as opposed to G1 or G2.

  // for every internal knot
  for (ULONG i=1; i+1<sKnots.GetSize(); i++)   // note: can't say sKnots.GetSize()-1
    {
      double dKnotVal = sKnots[i];
      SmVector3d sPVV_Left[4], sPVV_Right[4];

      // calc and store this knot's continuity
      EvaluateContinuity(dKnotVal, TRUE,            // in : from left eval
                        *this,                      // in : curve
                         dKnotVal, FALSE,           // in : from right eval
                         rContinuitiesAtKnots[i],   // out: Continuity Type
                         sPVV_Left, sPVV_Right,     // out: optional evals [pos, 1stDeriv, 2ndDeriv, 3rdDeriv]
                         dContinuityAngleTol) ;     // in : continuity angle tolerance

      // save the minimum internal knot continuity
      if (rContinuitiesAtKnots[i] < (long)reMinContinuityInCurve) 
        {
          reMinContinuityInCurve = (SmContinuityType)rContinuitiesAtKnots[i]; 
        }

    } // end iter every internal knot

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CalculateContinuities

/*******************************************************************//**
PURPOSE: Convert the curve to a 2D curve by setting the dimension
    to 2 and setting the Z values to NL_NOZ.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::ConvertTo2D
  ()
{
  m_lDim            = 2;
  gw_CURVE    *pCur = GetOrCreateGwNurbPointer();
  gw_CPOLYGON *pPol = pCur->pol;
  gw_CPOINT   *pPw  = pPol->Pw;
  gw_INDEX    i;
  for (i=0; i<=pPol->n; i++) 
    {
      pPw[i].z = NL_NOZ;
    }
  return SM_SUCCESS;

} // end SmBSplineCurve::ConvertTo2D

/*******************************************************************//**
PURPOSE: Convert the curve to a 3D curve by setting the dimension
    to 3 and setting the Z values to zero from NL_NOZ.

NOTES: 
    This will replace all the z values with NL_NOZ (or other) values
    It ignores any existing z values
***********************************************************************/
SmStatus SmBSplineCurve::ConvertTo3D
  ()
{
    m_lDim = 3;
    gw_CURVE *pCur = GetOrCreateGwNurbPointer();
    gw_CPOLYGON *pPol = pCur->pol;
    gw_CPOINT *pPw = pPol->Pw;
    gw_INDEX  i;
    for (i=0; i<=pPol->n; i++) {
        pPw[i].z = 0.0;
    }
    return SM_SUCCESS;

} // end SmBSplineCurve::ConvertTo3D

/*******************************************************************//**
PURPOSE: Create an circular arc of less than 180 degrees from a center 
             point and two points on the circle.

NOTES: returns an error and does not build an arc when
  1. Dist(Center,Point1) != Dist(Center,Point2)         - Point1 and Point2 don't lie on the same circle.
  2. AngularSweepRad(Point1, Point2) < SM_EFF_ZERO_SQRT - the requested arc is degenerate.
  3. AreColinear(Center, Point1, Point2)                - the points don't define an orientation plane.
***********************************************************************/
SmStatus SmBSplineCurve::CreateArcFromPoints
  (const SmContext    & crContext,           // in : new object construction
   ULONG                lDimensionOfResult,  // NotUsed: in : 2 or 3
   const SmPoint3d    & crCenter,            // in : Arc Center point
   const SmPoint3d    & crPoint1,            // in : Arc Start Point
   const SmPoint3d    & crPoint2,            // in : Arc End Point
   SmNurbCircleParam    eParameterization,   // in : oneof SM_CO_QUADRATIC = build degree 2 curve
                                             //            SM_CO_QUINTIC   = build degree 5 curve
   SmBSplineCurve    *& rpNewBSplineCurve)   // out: new BSpline Arc 
{
  SM_REF1(lDimensionOfResult) ;
  // radii 0 and 1
  SmVector3d sV0      = crPoint1 - crCenter ;
  SmVector3d sV1      = crPoint2 - crCenter ;
  double     dRadius  = sV0.Length() ;
  double     dRadius2 = sV1.Length() ;

  // Z vec and arc angular sweep
  double     dAngleRad;
  SmVector3d sZVec = sV0 * sV1;
  SER(sZVec.Unitize())
  SER(sZVec.CCWAngleBetween(sV0,sV1,dAngleRad));

  // tolerance
  double dTol = SM_EFF_ZERO * (1.0 + crPoint1.GetMaxDimension() + crCenter.GetMaxDimension());

  // both radii must have same length
  if (smos_Fabs(dRadius - dRadius2) > dTol) 
    { SER(SM_ERR) ; } // Both points not on circle

  // angular sweep must be nonDegenerate
  if (smos_Fabs(dAngleRad) < SM_EFF_ZERO_SQRT) 
    { SER(SM_ERR) ; } // Degenerate case here

  // center, start, and end points can't be colinear (no orienation plane is defined)
  if (smos_Fabs(dAngleRad) > SM_PI - SM_EFF_ZERO_SQRT) 
    { SER(SM_ERR) ; } // Nearly 180 degree case here
  
  // make sure Z vec is oriented for a counterClockwise arc 
  if (dAngleRad < 0.0) 
    { sZVec = - sZVec ; 
      SER(sZVec.CCWAngleBetween(sV0,sV1,dAngleRad)) ;
    }

  // arc angular sweep should be positive
  if (dAngleRad < SM_EFF_ZERO) 
    { SER(SM_ERR) ; } // Can't happen ??
  
  // define arc origin, x axis, and y axis placement
  SmAxis2Placement sPlacement;
  SmVector3d sYVec = sZVec * sV0;
  SmVector3d sXVec = sYVec * sZVec;
  SER(sXVec.Unitize());
  SER(sYVec.Unitize());
  SER(sPlacement.SetCanonical(crCenter,sXVec,sYVec));
  
  // pass the call along
  SER(SmBSplineCurve::CreateCircleSegment(crContext,
                                          3,
                                          sPlacement,
                                          dRadius,
                                          0.0,
                                          dAngleRad*180.0/SM_PI,
                                          eParameterization,
                                          rpNewBSplineCurve));

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateArcFromPoints

/*******************************************************************//**
PURPOSE: Create a new B-Spline curve equivalent to joining
     a sequential set of B-Spline curve segments.  

NOTES:
     Input curves are modified to have the same degree and dimension.

     output curve parameterization:
       1. 1st curve parameterization is preserved
       2. Subsequent segment parameterizations are shifted so that each segments
            beginning knot value is the same as the previous segments last knot value.
          Example: if 4 curve segments parameterized from 0 to 1 are joined the output curve
              will be parameterized from 0 to 4.
     
     When given, cpOptSenses can reverse a curve segment
       when cpOptSenses[i] = TRUE  - don't reverse segment parameterization
            cpOptSenses[i] = FALSE - do  

     Ownership of the input curve segments is NOT placed into the output BSplineCurve.  
     Any memory management of those curves has to be done by the calling function.
***********************************************************************/
SmStatus SmBSplineCurve::CreateByJoining
  (const SmContext & crContext,                    // in : context for new object construction
   const SmTArray<SmBSplineCurve*> & crSegments,   // in : Individual B-Spline curves which must be end to end
                                                   //      connected.
   const SmTArray<SmBoolean> * cpOptSenses,        // in : for each segment, TRUE= no reverse,FALSE=do
                                                   //      NULL=no segments are reversed
   SmBSplineCurve *& rpNewBSplineCurve)            // out: 
{
  // check input
  if (cpOptSenses) 
    {
      SM_ASSERT(cpOptSenses->GetSize() == crSegments.GetSize());
    }

  // locals
  ULONG ii, jj ;
  SmTArray<SmPoint3d> sCtrlPts;
  SmTArray<double>    sWeights;
  SmTArray<ULONG>     sKnotMult;
  SmTArray<double>    sKnots;
  ULONG               lDimension  = 0;;
  ULONG               lDegree     = 0;
  SmPoint3d           sLastPoint;
  double              dLastWeight = 0.0;
  SmBoolean           bIsRational = FALSE;
  ULONG               lMaxDegree  = 0;

  ULONG lMaxDim = 0;
  ULONG lMinDim = 5;

  // for every segment - find the max degree, dim, and rationality
  for(ii=0; ii<crSegments.GetSize(); ii++) 
    {
      SmBSplineCurve *pSeg = crSegments[ii];

      if (pSeg->IsRational()) bIsRational = TRUE;
      if (pSeg->GetDegree() > lMaxDegree) 
          lMaxDegree = pSeg->GetDegree();
      if (pSeg->GetDim() > lMaxDim) lMaxDim = pSeg->GetDim(); 
      if (pSeg->GetDim() < lMinDim) lMinDim = pSeg->GetDim();
    }

  // When curves come from mixed dimensions: Convert 3D curves to 2D
  if (lMaxDim != lMinDim) 
    {
      for(ii=0; ii<crSegments.GetSize(); ii++) 
        {
          SmBSplineCurve *pSeg = crSegments[ii];
          if (pSeg->GetDim() == 3) pSeg->ConvertTo2D();
        }
    }

  // For every curve, add knots and weights to joined curve arrays 
  for(ii=0; ii<crSegments.GetSize(); ii++) 
    {
      SmBSplineCurve *pSeg = crSegments[ii];

      // 1st make all curves the same degree
      if (pSeg->GetDegree() < lMaxDegree) 
        {
          SER(pSeg->DegreeElevate(lMaxDegree));
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          pSeg->Dump();
          sm_GraphicsLoop();
          pSeg->DrawWithKnots();
          sm_GraphicsLoop();
        }
#endif  // SM_DEBUG_CODE

      ULONG               lSegDim, lSegDeg;
      SmTArray<SmPoint3d> sSegPts;
      SmBSplineCurveForm  eSegCurveForm;
      SmTArray<ULONG>     sSegKnotMult;
      SmTArray<double>    sSegKnots;
      SmKnotType          eKnotType;
      SmTArray<double>    sSegWeights;

      // Get curve canonical reps - watch for reversed curves as needed
      if (cpOptSenses && (*cpOptSenses)[ii] == FALSE) 
        {
          SmBSplineCurve * pBSC = new (crContext) SmBSplineCurve(*pSeg);
          SmObjDelete sClean(pBSC);
          SmExtent1d sIvl = pBSC->GetNaturalInterval();
          SER(pBSC->ReverseParameterization(sIvl,sIvl));
          SER(pBSC->GetCanonical(lSegDim,lSegDeg,
                                 sSegPts,eSegCurveForm,sSegKnotMult,sSegKnots,
                                 eKnotType,sSegWeights));
        }
      else 
        {
          SER(pSeg->GetCanonical(lSegDim,lSegDeg,
                                 sSegPts,eSegCurveForm,sSegKnotMult,sSegKnots,
                                 eKnotType,sSegWeights));
        }

      // when needed - build an array of rational weights for this curve
      if (bIsRational && !pSeg->IsRational()) 
        {
          // Set the rationality of the segment to TRUE
          sSegWeights.ReSet();
          sSegWeights.InsertAt(0,1.0,sSegPts.GetSize());
        }

      // for the first curve
      if (ii==0) 
        {
          // add an extra knot, CtrlPnt, and sSegKNotMult at the beginning
          // then get last CtrlPt and Weight
          lDimension = lSegDim;
          lDegree    = lSegDeg;
          sLastPoint = sSegPts[0];
          if (bIsRational) 
            {
              dLastWeight = sSegWeights[0];
              sWeights.Add(sSegWeights[0]);
//                sLastPoint = sLastPoint / dLastWeight;
            }
          sCtrlPts.Add(sLastPoint);
          sKnots.Add(sSegKnots[0]);
          sKnotMult.Add(sSegKnotMult[0]);
        }
      else // get last CtrlPt and Weight
        {
          sLastPoint = sCtrlPts.GetLast();
          if (bIsRational) 
            {
              dLastWeight = sWeights.GetLast();
            }
        }

      // Make sure curve dimensions are the same - should already be fixed
      if (lDimension != lSegDim) SER(SM_ERR_INVALID_INPUT);

      // Make sure degrees of curves are same - should already be fixed
      if (lDegree != lSegDeg) 
        {
          // GWC: this looks like a dead code branch
          if (lSegDeg == 1) 
            {
              // Increase degree of curve to 2 or 3
              if (lDegree == 2) 
                {
                  lSegDeg = 2;
                  for(jj=0; jj<sSegKnotMult.GetSize(); jj++) 
                    {
                      sSegKnotMult[jj] = 2;
                    }
                  sSegKnotMult[0] = 3;
                  sSegKnotMult[sSegKnotMult.GetSize()-1] = 3;
                  SmPoint3d sMid = (sSegPts[0]+sSegPts[1]) / 2.0;
                  sSegPts.InsertAt(1,sMid,1);
                }
              else if (lDegree == 3) 
                {
                  lSegDeg = 3;
                  for(jj=0; jj<sSegKnotMult.GetSize(); jj++) 
                    {
                      sSegKnotMult[jj] = 3;
                    }
                  sSegKnotMult[0] = 4;
                  sSegKnotMult[sSegKnotMult.GetSize()-1] = 4;
                  SmPoint3d sMid1 = sSegPts[0] + (sSegPts[1]-sSegPts[0]) / 3.0;
                  SmPoint3d sMid2 = sSegPts[0] + 2.0 * (sSegPts[1]-sSegPts[0]) / 3.0;
                  sSegPts.InsertAt(1,sMid1,1);
                  sSegPts.InsertAt(2,sMid2,1);
                }
            }
          else 
            {
              SER(SM_ERR_INVALID_INPUT);
            }
        } // end Segment needs to increase its degree check  - dead code block;

      // Make sure end-points are the same
      double dScaledZero = SM_EFF_ZERO_SQRT * 1000.0 * (1.0 + sLastPoint.GetMaxDimension()) ;
      if (sLastPoint.DistanceBetween(sSegPts[0]) > dScaledZero)
        {
          SER(SM_ERR_INVALID_INPUT);
        }

      // make sure end-weights are the same
      if (bIsRational && !SM_IS_ZERO(dLastWeight-sSegWeights[0])) 
        {
          if(   smos_Fabs(dLastWeight)    < SM_EFF_ZERO
             || smos_Fabs(sSegWeights[0]) < SM_EFF_ZERO) 
            { SER(SM_ERR); }

          // Scale the weights
          double dScale = dLastWeight/sSegWeights[0];
          for(jj=0; jj<sSegPts.GetSize(); jj++) 
            {
              sSegWeights[jj] = sSegWeights[jj]*dScale;
            }
        }
          
      // Now append CtrlPt, Knot, and Weight arrays.

      // Knots
      double dKnotOffset = sKnots.GetLast() - sSegKnots[0];
      for(jj=1; jj<sSegKnots.GetSize(); jj++) 
        {
          sKnots.Add( sSegKnots[jj] + dKnotOffset );
          if (jj == sSegKnots.GetSize() - 1) 
            {
              // remove extra multiplicity from last knot
              sKnotMult.Add( sSegKnotMult[jj]-1 );
            }
          else 
            { // handle interior knots
              sKnotMult.Add( sSegKnotMult[jj] );
            }
        }

      // Weights and CtrlPts
      for(jj=1; jj<sSegPts.GetSize(); jj++) 
        {
          if (bIsRational) 
            {
              sWeights.Add(sSegWeights[jj]);
//                sSegPts[jj] = sSegPts[jj]/sSegWeights[jj];
            }
          sCtrlPts.Add(sSegPts[jj]);
        }

    } // end iter each curve segment

  // Clamp that last knot
  sKnotMult[sKnotMult.GetSize()-1] = sKnotMult.GetLast() + 1;

  SmTArray<double> *pWeights = NULL;
  if (bIsRational)  pWeights = &sWeights;

  // Create output from canonical data
  SER(SmBSplineCurve::CreateCanonical(crContext, lDimension, lDegree,
                                      sCtrlPts, SM_CF_UNSPECIFIED,
                                      sKnotMult,
                                      sKnots,   SM_KT_UNSPECIFIED,
                                      pWeights, NULL,
                                      rpNewBSplineCurve));
  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateByJoining
    
/*******************************************************************//**
PURPOSE: Create a new BSplineCurve by doing a translation/scaling along the
    two vectors. 
    
NOTES: We scale it so that the start and end points move along
    the vectors and then rotating it about the axis defined by the ending
    start and end point.  This really doesn't work well for cases where the curve
    is closed because we can't construct a vector from the first to the
    second.  
***********************************************************************/
SmStatus SmBSplineCurve::CreateByScaleTransRot
  (const SmContext & crContext,
   const SmVector3d & crStartPointVec,
   const SmVector3d & crEndPointVec,
   double dRotationDeg,
   SmBSplineCurve *& rpnewBSplineCurve) 
  const
{
    SmPoint3d sStart, sEnd;
    SmExtent1d sIvl = GetNaturalInterval();
    SER(EvaluatePoint(sIvl.GetMin(),sStart));
    SER(EvaluatePoint(sIvl.GetMax(),sEnd));

    if (sStart.CloserThan(SM_EFF_ZERO, sEnd)){
        SER(SM_ERR);
    }
    SmBSplineCurve *pRes = new(crContext) SmBSplineCurve(*this);
    SmObjDelete sCleanRes(pRes);

    SmPoint3d sStart2 = sStart + crStartPointVec;
    SmPoint3d sEnd2 = sEnd + crEndPointVec;
    SmVector3d sRotVec = sEnd2 - sStart2;
    SER(sRotVec.Unitize());

    SmAxis2Placement sRot;
    sRot.RotateAboutAxisAtPoint(dRotationDeg*SM_PI/180.0,sStart2,sRotVec);

    SmControlPointFormType eForm = SM_CP_EUCLIDIAN_RATIONAL;
    if (!pRes->IsRational()) eForm = SM_CP_NON_RATIONAL;

    // Now Let's go through and project each control point down onto
    // the line and use that to compute a transformation.
    SmVector3d sLineVec = sEnd - sStart;
    for (ULONG i=0; i<pRes->GetNumberControlPoints(); i++) {
        SmPoint3d sPnt;
        double dWeight = 0.0;
        SER(pRes->GetControlPoint(eForm,i,sPnt,dWeight));
        double dParam;
        // Parameter contains relative stength of start v.s. end.
        smgu_LineClosestPoint(sStart,sLineVec,sPnt,dParam);
        // Note that it is entirely possible for dParam to be outside the
        // 0-1 range.  It will effectively continue the effect of the 
        // vector transformation.  The resulting vector could be greater
        // in magnitude and angular direction than either of the originals.
        SmVector3d sTransVec = crStartPointVec + dParam * (crEndPointVec - crStartPointVec);
        sPnt = sPnt + sTransVec; // Move point
        sRot.TransformPoint(sPnt,sPnt); // Rotate point
        
        // Now put point back into curve
        SER(pRes->SetControlPoint(eForm,i,sPnt,dWeight));
    }

    sCleanRes.Clear();
    rpnewBSplineCurve = pRes;
    return SM_SUCCESS;

} // end SmBSplineCurve::CreateByScaleTransRot

/*******************************************************************//**
PURPOSE: This is the STEP Based canonical creation.  

NOTES: See STEP Part 42
    for details about this form.  Note that the control points are Euclidian
    even when weights exist.  If you have homogeneous 
    coordinates you need to perform homogeneous division prior to calling
    this method (x = x/w, y=y/w, z=z/w w=w).
***********************************************************************/
SmStatus SmBSplineCurve::CreateCanonical
  (const SmContext           & crContext,             // in : context for new object construction
   ULONG                       lDimension,            // in : valid dimension - 2 or 3  
   ULONG                       lDegree,               // in : valid degree - 1 through 32  
   const SmTArray<SmPoint3d> & crControlPointsList,   // in : Euclidian form of the control points.
   SmBSplineCurveForm          eBSplineCurveForm,     // in : oneof  SM_CF_POLYLINE_FORM,  SM_CF_PARABOLIC_ARC, 
                                                      //             SM_CF_CIRCULAR_ARC,   SM_CF_HYPERBOLIC_ARC,
                                                      //             SM_CF_ELLIPTIC_ARC,   SM_CF_UNSPECIFIED, 
                                                      //             SM_CF_HELICAL_ARC   
   const SmTArray<ULONG>     & crKnotMultiplicities,  // in : Multiplicities of the end knots must be lDegree+1 and
                                                      //      and internal multiplicities must be lDegree or less.
   const SmTArray<double>    & crKnots,               // in : unique knots in ascending order
   SmKnotType                  eKnotType,             // in : oneof  SM_KT_UNIFORM_KNOTS,  SM_KT_QUASI_UNIFORM_KNOTS,      
                                                      //             SM_KT_UNSPECIFIED,    SM_KT_PIECEWISE_BEZIER_KNOTS    
   const SmTArray<double>    * cpOptWeights,          // in : optional associated weigths for each control point
   const SmExtent1d          * cpOptTrimInterval,     // in : If specified contains a trimming interval to resize the curve
   SmBSplineCurve           *& rpNewBSplineCurve)     // out: New BSplineCurve
{ 
  // Do some simple input checking for valid numbers
  RANGE_ER(1,lDimension,3);
  RANGE_ER(1,lDegree,32);
  if (cpOptWeights) 
    {  // If have weights make sure have right number of them
      if (cpOptWeights->GetSize() != crControlPointsList.GetSize()) SER(SM_ERR_INVALID_INPUT);
    }

  // Make compatable knots array
  SmTArray<double> sNewKnots;
  for (ULONG i=0; i<crKnots.GetSize(); i++) 
    {
      double dKnot = crKnots[i];
      long   lMult = crKnotMultiplicities[i];
      for (long k=0; k<lMult; k++) 
        {
          sNewKnots.Add(dKnot);
        }
    }

  // Make compatable weight array
  SmTArray<double> sTmpWeights;
  gw_REAL *W;
  if (cpOptWeights) 
    {
      W = (gw_REAL*)cpOptWeights->GetDataArray();
    }
  else 
    {
      for (ULONG j=0; j<crControlPointsList.GetSize(); j++) 
        {
          sTmpWeights.Add(1.0);
        }
      W = sTmpWeights.GetDataArray();
    }

  // NLIB locals
  NL_STACKS      SC;
  gw_CURVE       Cur;
  SmStackHandler sStackKp(&SC);
  NL_POINT      *Pw = SM_REINTERPRET_CAST( NL_POINT*,crControlPointsList.GetDataArray());
  gw_REAL       *U  = (gw_REAL*)sNewKnots.GetDataArray();
  gw_INDEX       n  = crControlPointsList.GetSize() - 1;
  gw_DEGREE      p  = (gw_DEGREE)lDegree;
  gw_PARAMETER ivl[2];

  // set curve ivl
  if (cpOptTrimInterval) 
    {
      ivl[0] = cpOptTrimInterval->GetMin();
      ivl[1] = cpOptTrimInterval->GetMax();
    }
  else 
    {
      ivl[0] = sNewKnots[p];
      ivl[1] = sNewKnots[sNewKnots.GetSize()-p-1];
    }

  // NLIB curve construction
  N_CrvInitArrays(&Cur);
  if(NL_YES == N_Iges126Crv(n,p,U,W,Pw,ivl,&Cur,&SC))
    { SER(SM_ERR); }

  // when asked - bump 2d curves up to 3d curves
  if(lDimension == 3 && !N_CrvIs3d( &Cur )) 
    { 
      N_Crv2dTo3d( &Cur ) ;
    }

  // make SMLib SmBSplineCurve - side effect: packs the memory
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(lDimension,&Cur);
  rpNewBSplineCurve->m_eKnotType         = eKnotType;
  rpNewBSplineCurve->m_eBSplineCurveForm = eBSplineCurveForm;

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateCanonical

/*******************************************************************//**
PURPOSE: Create a piecewise Cardinal Spline curve from a given set of
   control points and tension parameters, and interpolation type.

NOTES: It is composed of cubic Bezier splines joined with C1 continuity.
   It is a local interpolation method: the curve interpolates all of the
   given points, and each segment is defined using only the neighboring
   points, parameter values, and tension values.

   The i-th Bezier segment goes through two bounding points, Pi and Pi+1.
   Derivatives at a point Pi are a combination of the incoming and
   outgoing point differences, (Pi - Pi-1) and (Pi+1 - Pi).

   The parameter value at Pi is ui.

   For each segment, the formula is:

        u1 - u          u - u0
   A1 = ------- P0  +  ------- P1
        u1 - u0        u1 - u0

        u2 - u          u - u1
   A2 = ------- P1  +  ------- P2
        u2 - u1        u2 - u1

        u3 - u          u - u2
   A3 = ------- P2  +  ------- P3
        u3 - u2        u3 - u2

        u2 - u          u - u0
   B1 = ------- A1  +  ------- A2
        u2 - u0        u2 - u0

        u3 - u          u - u1
   B2 = ------- A2  +  ------- A3
        u3 - u1        u3 - u1

   and finally the curve C(u) in that segment is

        u2 - u          u - u1
   C  = ------- B1  +  ------- B2
        u2 - u1        u2 - u1

   Here we have set i == 1 for clarity; this segment runs from P1 to P2 (u1 to u2).

   It is easy to demonstrate that C(u1) = P1 and C(u2) = P2.  To define the curve,
   we need that plus the first derivatives at P1 and P2.  They come out to be:

            ( u2 - u1                    u1 - u0               )
   C'(u1) = ( ------- * ( P1 - P0 )  +   ------- * ( P2 - P1 ) )  /  ( u2 - u0 )
            ( u1 - u0                    u2 - u1               )
    and
            ( u3 - u2                    u2 - u1               )
   C'(u2) = ( ------- * ( P2 - P1 )  +   ------- * ( P3 - P2 ) )  /  ( u3 - u1 )
            ( u2 - u1                    u3 - u2               )

   The derivative vectors are then scaled according to the given Tension values:
   C'(ui) is multiplied by ( 1.0 - Tension[i] ).

   The knot vector is calculated in this method, according to the input parameter ePz,
   which is one of SM_CP_UNIFORM, SM_CP_CHORDLENGTH, or SM_CP_CENTRIPETAL.
   If the points are not fairly uniform (not equally spaced, or with sharp turns),
   using SM_CP_CENTRIPETAL generally gives the 'best' curve.

   For a Uniform knot vector, this reduces to:
        C'(ui)   = 0.5 * ( 1-Tension[i  ] ) * (Pi+1 - Pi-1)
        C'(ui+1) = 0.5 * ( 1-Tension[i+1] ) * (Pi+2 - Pi  )

   i.e., the tangent at each point is parallel to the vector from the previous
   to the next point.

   When Ti = 0.0 (for all i), this results in the Catmull-Rom spline.
   When Ti = 1.0 (for all i), the spline turns into a polyline.

   If bIsPeriodic is passed in as True, then this method will add the first point
   at the end: the caller should not repeat the first point at the end.
   The curve will join itself smoothly at the start/end point.
   (If bIsPeriodic is False, and the first and last points are the same,
   then the curve will be closed, but not smoothly.)

   In addition, tensions can be either all '1' or not '1'

   Sample Usage:
        SmTArray<SmPoint3d> sPnts;
        sPnts.Add(SmPoint3d(0,0,0));
        sPnts.Add(SmPoint3d(1,0,0));
        sPnts.Add(SmPoint3d(1,1,0));
        sPnts.Add(SmPoint3d(0,1,0));
        SmTArray<double> sTensionParams;
        sTensionParams.Add(0.5);
        sTensionParams.Add(0.5);
        sTensionParams.Add(0.5);
        sTensionParams.Add(0.5);
        SmBSplineCurve * pNewBSP = NULL;
        SmBoolean bIsPeriodic = FALSE;
        SER(SmBSplineCurve::CreateCardinalCurve(crContext,sPnts,
            sTensionParams,bIsPeriodic,pNewBSP));

***********************************************************************/
SmStatus SmBSplineCurve::CreateCardinalSpline
  (const SmContext     & crContext,      // in:
   SmTArray<SmPoint3d> & rPoints,        // in: Points to be interpolated.
   SmTArray<double>    & rTensionParams, // in: rTensionParams[i] corresponds to rPoints[i].
   SmBoolean             bIsPeriodic,    // in: see notes above.
   SmBSplineCurve     *& rpNewBSP,       // out: resulting spline
   SmCurveParameterizationType ePz )     // in,  opt: one of SM_CP_UNIFORM, SM_CP_CHORDLENGTH,
                                         //           or SM_CP_CENTRIPETAL.  Default: SM_CP_UNIFORM.
{
    ULONG lTotalPnts = rPoints.GetSize();
    SM_ASSERT(lTotalPnts >= 3);
    SM_ASSERT(rTensionParams.GetSize() == lTotalPnts);

    if (bIsPeriodic)
    {
        rPoints.Add(rPoints[0]);
        rTensionParams.Add(rTensionParams[0]);
        lTotalPnts++;
    }

    ULONG ii;

    // Derive parametrization.
    SmTArray<double> sKnots(lTotalPnts);
    double dDist, dParam = 0.0, dDelta = 1.0;
    sKnots.Add( dParam );
    for( ii=1; ii<lTotalPnts; ii++ )
    {
        dDist = rPoints[ii].DistanceBetween( rPoints[ii-1] );

        switch ( ePz )
        {
          case SM_CP_UNIFORM:     { dDelta = 1.0;              break; }
          case SM_CP_CHORDLENGTH: { dDelta = dDist;            break; }
          case SM_CP_CENTRIPETAL: { dDelta = smos_Sqrt(dDist); break; }

        } // end switch

        dParam += dDelta;
        sKnots.Add( dParam );
    }

    ULONG lDimension = 3;
    ULONG lDegree = 3;
    SmTArray<ULONG> sKnotMult(lTotalPnts,NULL,lTotalPnts);

    // Check for polyline: all Tensions == 1.0.
    if ( smos_Fabs(rTensionParams[0] - 1.0) <= SM_EFF_ZERO )
    {
        for (ii=1; ii<lTotalPnts; ii++)
        {
            // Will allow tensions be either all '1' or not '1'
            if (smos_Fabs(rTensionParams[ii] - 1.0) > SM_EFF_ZERO)
              { SER(SM_ERR); }
        }

        // Create degree one polyline instead.

        // Knot vector for the B-Spline: Piecewise linear Bezier.
        sKnotMult[0] = 2;
        for ( ii=1; ii+1<lTotalPnts; ii++ ) // note: can't say lTotalPnts-1
          { sKnotMult[ii] = 1; }
        sKnotMult[lTotalPnts-1] = 2;

        lDegree = 1;
        SER(SmBSplineCurve::CreateCanonical(crContext, lDimension, lDegree,
            rPoints, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
            SM_KT_UNSPECIFIED, NULL, NULL, rpNewBSP));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
        if (bDebugMe0) {
            if ( FALSE )
              { smgfx_Erase(); }
            smgfx_SetLook(1,2, 0,0,0);
            rpNewBSP->DrawWDeriv( rpNewBSP->GetNaturalInterval() );
            rpNewBSP->Dump();
            sm_GraphicsLoop();
        }
#endif
        return SM_SUCCESS;
    }


    // General case (not a polyline).

    SmTArray<SmPoint3d> s3DCtrlPts;
    SmPoint3d sPt0, sPt1, sPt2, sPt3;
    for (ii=1; ii<lTotalPnts; ii++)
      {
        sPt1 = rPoints[ii-1];
        sPt2 = rPoints[ii];
        if ( ii==1 )
        {
            if (bIsPeriodic)
              { sPt0 = rPoints[lTotalPnts-2]; }
            else
              { sPt0 = sPt1; }
        }
        else
          { sPt0 = rPoints[ii-2]; }

        if ( ii==lTotalPnts-1 )
        {
            if (bIsPeriodic)
              { sPt3 = rPoints[1]; }
            else
              { sPt3 = sPt2; }
        }
        else
          { sPt3 = rPoints[ii+1]; }

        // Now calculate the derivatives at the start and end of this segment.

        // Point differences
        SmVector3d sDiff1( sPt1 - sPt0 );
        SmVector3d sDiff2( sPt2 - sPt1 );
        SmVector3d sDiff3( sPt3 - sPt2 );

        // Local knot values
        // Do a periodic wrap at the ends.
        double t0 = ( ii>1 ) ? sKnots[ii-2] : sKnots[0] - ( sKnots[lTotalPnts-1] - sKnots[lTotalPnts-2] );
        double t1 = sKnots[ii-1];
        double t2 = sKnots[ii  ];
        double t3 = ( ii<lTotalPnts-1 ) ? sKnots[ii+1] : sKnots[lTotalPnts-1] + ( sKnots[1] - sKnots[0] );

        // Knot differences
        double dDt10 = t1 - t0;
        double dDt21 = t2 - t1;
        double dDt32 = t3 - t2;
        double dDt20 = t2 - t0;
        double dDt31 = t3 - t1;

        SmVector3d sTan1( ( sDiff1 * dDt21 / dDt10  +  sDiff2 * dDt10 / dDt21 ) / dDt20 );
        SmVector3d sTan2( ( sDiff2 * dDt32 / dDt21  +  sDiff3 * dDt21 / dDt32 ) / dDt31 );

        // Scale by the parameter delta for this interval.
        sTan1 *= dDt21;
        sTan2 *= dDt21;

        // Apply the tension.
        sTan1 *= ( 1.0 - rTensionParams[ii-1] );
        sTan2 *= ( 1.0 - rTensionParams[ ii ] );


        // SmHermiteCurve is a convenient way to get the B-Spline (Bezier)
        // control points from start and end points and derivatives.

        SmHermiteCurve sHerm( sPt1, sTan1, sPt2, sTan2, 3 );
        sHerm.SetContext(NULL);
        // Convert from Hermite into Bezier.  (Reuse sPt0 - sPt3.)
        sHerm.GetBezierPoints( sPt0, sPt1, sPt2, sPt3 );

        if (s3DCtrlPts.GetSize() == 0) // Add 1st point only for 1st segment.
          { s3DCtrlPts.Add( sPt0 ); }

        s3DCtrlPts.Add( sPt1 );
        s3DCtrlPts.Add( sPt2 );
        s3DCtrlPts.Add( sPt3 );
    }

    // Knot vector for the B-Spline: Piecewise cubic Bezier.
    sKnotMult[0] = 4;
    for (ii=1; ii+1<lTotalPnts; ii++) // note: can't say lTotalPnts-1
      { sKnotMult[ii] = 3; }
    sKnotMult[lTotalPnts-1] = 4;

    SER(SmBSplineCurve::CreateCanonical(crContext, lDimension, lDegree,
        s3DCtrlPts, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
        SM_KT_UNSPECIFIED, NULL, NULL, rpNewBSP ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
SmBoolean sbDrawTangents = TRUE;
    if (bDebugMe) {
        if ( FALSE )
          { smgfx_Erase(); }
        rpNewBSP->Dump();
        smgfx_SetLook(1,2, 0,0,0); rpNewBSP->DrawWDeriv(rpNewBSP->GetNaturalInterval()); sm_GraphicsLoop();
        sm_GraphicsLoop();
        //rpNewBSP->WriteToFile(_T("curve.smc"),TRUE);
        rpNewBSP->DrawPolygon(); sm_GraphicsLoop();
        sm_GraphicsLoop();
        SmVector3d sPV1[2], sPV2[2];
        smgfx_SetLook(2,4, 1,0,0);
        for (ii=0; ii<lTotalPnts; ii++) {
            SER(rpNewBSP->Evaluate(sKnots[ii],1,TRUE,sPV1));
            if ( sbDrawTangents )
              { sPV1[1].Draw(sPV1); }
            else
              { sPV1[0].Draw(); }
            sm_GraphicsLoop();
        }

        // Compare derivs at each interior knot.
        for ( ii=1; ii<lTotalPnts-1; ii++ ) {
            SER(rpNewBSP->Evaluate( sKnots[ii], 1, FALSE, sPV1 ));
            SER(rpNewBSP->Evaluate( sKnots[ii], 1, TRUE,  sPV2 ));
            SmVector3d sDiff = sPV2[1] - sPV1[1];
        }
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineCurve::CreateCardinalSpline

/*******************************************************************//**
PURPOSE: Create a nurb which exactly represents a segment of a circle
    or the entire circle.

NOTES: Note that the circle starts at the start angle and goes
    counter clockwise until it hits the end angle.  An angle of 0 
    corresponds to the X axis of the Axis2Placement.  The start angle 
    can be greater than the end angle. 
    
    Parameterization along the curve will approximate but not
    exactly equal angle parameterization.     
***********************************************************************/
SmStatus SmBSplineCurve::CreateCircleSegment
  (const SmContext & crContext,                // in : new object context
   ULONG lDimensionOfResult,                   // in : 2 or 3 
   const SmAxis2Placement & crReferenceFrame,  // in : Circle lies on the X, Y plane of the reference
                                               //      frame with zero degrees at the X axis.
   double dRadius,                             // in : any number > 0.0 
   double dStartAngDeg,                        // in : angle from XAxis in degrees: [-360,360] 
   double dEndAngDeg,                          // in : angle from XAxis in degrees: [-360,360]
   SmNurbCircleParam eParameterization,        // in : SM_CO_QUADRATIC - produces a circle using a 
                                               //                        quadratic (degree 2) parameterization.
                                               //      SM_CO_QUINTIC   - produces a circle using a quintic
                                               //                        (degree 5) parametrization.
   SmBSplineCurve *& rpNewBSplineCurve)        // out: the new circle
{
  // check input
  LE_ZERO_ER(dRadius);
  RANGE_ER(2,lDimensionOfResult,3);
  RANGE_ER(-360.0,dStartAngDeg,360.0);
  RANGE_ER(-360.0,dEndAngDeg,360.0);

  // locals
  NL_POINT C;
  NL_POINT X, Y;
  sm_ExtractFrame(crReferenceFrame,C,X,Y);
  NL_STACKS SC;
  SmStackHandler sStackKp(&SC);
  
  gw_FLAG ctp=0;
  if      (eParameterization == SM_CO_QUADRATIC){ ctp = NL_QUADRATIC; } // degree 2
  else if (eParameterization == SM_CO_QUINTIC)  { ctp = NL_QUINTIC;   } // degree 5
  else SER(SM_ERR_INVALID_INPUT);
  
  // create circle start and stop specified by angles but parameterizaton may vary from TRUE degrees 
  gw_CURVE Cur;
  N_CrvInitArrays(&Cur);
  if(NL_YES == N_CreateCircArc(C,X,Y,dRadius,dStartAngDeg,dEndAngDeg,ctp,&Cur,&SC))
    { SER(SM_ERR); }

  // GW_SER(N_CreateCircArc(C,X,Y,dRadius,dStartAngDeg,dEndAngDeg,ctp,&Cur,&SC));
  
  if (lDimensionOfResult == 2) N_Crv3dTo2d(&Cur);

  // copy curve into
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(lDimensionOfResult,&Cur);
  rpNewBSplineCurve->m_eBSplineCurveForm = SM_CF_CIRCULAR_ARC;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
// draw 
if(bDebugMe)
  {
    smgfx_Erase() ;
    smgfx_SetLook(1,2, 0,0,0) ; crReferenceFrame.Draw() ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,5, 0,0,0) ; rpNewBSplineCurve->DrawParams() ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;

  }
#endif // SM_DEBUG_CODE
  
  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateCircleSegment

/*******************************************************************//**
PURPOSE: Create a conic segment given coefficients of its quadric
    equation.
    
NOTES:   
 Note that the output curves will have default analytic/STEP
    domains. Users might want to call ::AdjustSTEPInterval to resize it.

Example:   
                                  2          2
         The quadric equation:  aX + bXY + cY + dX + eY + f = 0    
         The output curve can be one of the following conics: 
         SmCircle, SmEllipse, SmParabola or SmHyperbola

***********************************************************************/
SmStatus SmBSplineCurve::CreateConicSegment
  (const SmContext & crContext,
   double dA, double dB, double dC,
   double dD, double dE, double dF,
   SmBSplineCurve *& rpNewBSplineCurve)
{
    SM_ASSERT(smos_Fabs(dA) + smos_Fabs(dB) + smos_Fabs(dC) > SM_EFF_ZERO);
    double dA1 = dA, dC1 = dC, dD1 = dD, dE1 = dE, dF1 = dF;
    double dRotateAngle = 0.0;
    double dCos = 1.0;
    double dSin = 0.0;
    if (dB != 0.0) {
        // We'll rotate the coordinate axes to eliminate the 'xy' term
        // tan(2*dRotateAngle) = dB / (dA - dC) or
        // cot(2*dRotateAngle) = (dA - dC) / dB
        if (smos_Fabs(dA-dC) < SM_EFF_ZERO) {
            dRotateAngle = SM_PI/4.0;
        }
        else {
            dRotateAngle = atan2(dB, dA-dC)/2.0;
        }
        dCos = smos_Cosine(dRotateAngle);
        dSin = smos_Sine(dRotateAngle);
        dA1 = dA*dCos*dCos + dB*dCos*dSin + dC*dSin*dSin;
        dC1 = dA*dSin*dSin - dB*dSin*dCos + dC*dCos*dCos;
        dD1 = dD*dCos + dE*dSin;
        dE1 = -dD*dSin + dE*dCos;
    }
    // Now the transformed equation looks like this:
    //    2     2
    // A1x + C1y + D1x + E1y + F1 = 0
    double dQ1 = 4.0*dA1*dC1*dF1 - dC1*dD1*dD1 - dA1*dE1*dE1;
    if (smos_Fabs(dQ1) < SM_EFF_ZERO) {
        SER(SM_ERR); // Invaid
    }

    if (smos_Fabs(dA1*dC1) < SM_EFF_ZERO) { // Parabola: dA1*dC1 = 0.0
        double dOrigX, dOrigY;
        double dFocalDist;
        if (smos_Fabs(dC1) < SM_EFF_ZERO) { // dC1 = 0.0
            //    2
            // A1x + D1x + E1y + F1 = 0
            dOrigX = -dD1/dA1/2.0;
            dOrigY = dD1*dD1/dA1/dE1/4.0 - dF1/dE1;
            dFocalDist = -dE1/dA1/4.0;
            if (dFocalDist > 0.0) {
                dRotateAngle += SM_PI/2.0; // 'Upward'
            }
            else {
                dRotateAngle -= SM_PI/2.0; // 'Downward'
            }
            dCos = smos_Cosine(dRotateAngle);
            dSin = smos_Sine(dRotateAngle);
        }
        else { // dA1 = 0.0
            //    2
            // C1y + D1x + E1y + F1 = 0
            dOrigX = -dE1/dC1/2.0;
            dOrigY = dE1*dE1/dC1/dD1/4.0 - dF1/dD1;
            dFocalDist = -dD1/dC1/4.0;
            if (dFocalDist < 0.0) {
                dRotateAngle += SM_PI;
                dCos = smos_Cosine(dRotateAngle);
                dSin = smos_Sine(dRotateAngle);
            }
        }
        SmVector3d sOrigin(dOrigX, dOrigY, 0.0);
        SmVector3d sXAxis(dCos,dSin,0.0);  // X axis of the 3D coordinate system
        SmVector3d sYAxis(-dSin,dCos,0.0); // Y axis of the 3D coordinate system
        SmAxis2Placement sPosition;
        sPosition.SetCanonical(sOrigin,sXAxis,sYAxis);
        SmParabola * pNewParabola = NULL;
        SER(SmParabola::CreateCanonical(crContext,sPosition,
            smos_Fabs(dFocalDist),pNewParabola));
        rpNewBSplineCurve = pNewParabola;
    }
    else if (dA1*dC1 > 0.0) { // Ellipse: dA1*dC1 > 0.0
        //            2                 2
        //    (X - Ox)          (Y - Oy)
        //---------------- + --------------- =  1
        //   (dMajRad)2        (dMinRad)2
        if (dQ1*(dA1+dC1) > -SM_EFF_ZERO) {
            SER(SM_ERR); // Invaid
        }
        double dConstant = -dF1 + (dD1*dD1/dA1/4.0) + (dE1*dE1/dC1/4.0);
        double dMajRad = smos_Sqrt(dConstant/dA1);
        double dMinRad = smos_Sqrt(dConstant/dC1);

        SmVector3d sOrigin(-dD1/dA1/2.0, -dE1/dC1/2.0, 0.0);
        SmVector3d sXAxis(dCos,dSin,0.0);
        SmVector3d sYAxis(-dSin,dCos,0.0);
        SmAxis2Placement sPosition;
        sPosition.SetCanonical(sOrigin,sXAxis,sYAxis);
        if (smos_Fabs(dMajRad-dMinRad) < SM_EFF_ZERO) {
            // Create SmCircle
            SmCircle * pNewCircle = NULL;
            SER(SmCircle::CreateCanonical(crContext,sPosition,dMajRad,pNewCircle));
            rpNewBSplineCurve = pNewCircle;
        }
        else {
            SmEllipse * pNewEllipse = NULL;
            SER(SmEllipse::CreateCanonical(crContext,sPosition,
                dMajRad,dMinRad,pNewEllipse));
            rpNewBSplineCurve = pNewEllipse;
        }
    }
    else { // Hyperbola: dA1*dC1 < 0.0
        double dSemiAxis, dSemiImageAxis;
        double dConstant = -dF1 + (dD1*dD1/dA1/4.0) + (dE1*dE1/dC1/4.0);
        if (dConstant/dC1 < 0.0) {
            //            2                     2
            //    (X - Ox)              (Y - Oy)
            //---------------- - --------------------- =  1
            //   (dSemiAxis)2       (dSemiImageAxis)2
            dSemiAxis = smos_Sqrt(smos_Fabs(dConstant/dA1));
            dSemiImageAxis = smos_Sqrt(smos_Fabs(dConstant/dC1));
        }
        else { // dConstant/dA1 < 0.0
            //            2                     2
            //    (Y - Oy)              (X - Ox)
            //---------------- - --------------------- =  1
            //   (dSemiAxis)2       (dSemiImageAxis)2
            dSemiAxis = smos_Sqrt(smos_Fabs(dConstant/dC1));
            dSemiImageAxis = smos_Sqrt(smos_Fabs(dConstant/dA1));
            dRotateAngle += SM_PI/2.0;
            dCos = smos_Cosine(dRotateAngle);
            dSin = smos_Sine(dRotateAngle);
        }
        SmVector3d sOrigin(-dD1/dA1/2.0, -dE1/dC1/2.0, 0.0);
        SmVector3d sXAxis(dCos,dSin,0.0);
        SmVector3d sYAxis(-dSin,dCos,0.0);
        SmAxis2Placement sPosition;
        sPosition.SetCanonical(sOrigin,sXAxis,sYAxis);
        SmHyperbola * pNewHyperbola = NULL;
        SER(SmHyperbola::CreateCanonical(crContext,sPosition,
            dSemiAxis, dSemiImageAxis,pNewHyperbola));
        rpNewBSplineCurve = pNewHyperbola;
    }

    return SM_SUCCESS;

} // end SmBSplineCurve::CreateConicSegment

/*******************************************************************//**
PURPOSE: Create a degenerate B-Spline curve representing a point.

NOTES: 2d and 3d curves are the same.  make sure that the
 input crPoint.z = NL_NOZ for 2d curves.
***********************************************************************/
SmStatus SmBSplineCurve::CreateDegenerateCurve
 (const SmContext  & crContext,          // in : context for new object construction
  ULONG              lDimensionOfResult, // in : desired curve dimension (2 or 3)
  const SmPoint3d  & crPoint,            // in : Point marking the degenerate curve
  SmBSplineCurve  *& rpNewBSplineCurve)  // out: the new degenerate curve
{
  // check input
  RANGE_ER(2,lDimensionOfResult,3);

  // locals
  SmBSplineCurve    * pBSC = NULL ;
  SM_OBJ_ARRAY(sCntrlPoly, SmPoint3d, 2) ; sCntrlPoly.SetSize(2) ;
  SM_OBJ_ARRAY(sKnots,     double,    2) ; sKnots.SetSize(2) ;
  SM_OBJ_ARRAY(sKnotMult,  ULONG,     2) ; sKnotMult.SetSize(2) ;

  // control points
  sCntrlPoly[0] = crPoint ;
  sCntrlPoly[1] = crPoint ;

  // unique knots
  sKnots[0] = 0.0; 
  sKnots[1] = 1.0;

  // knot multiplicities
  sKnotMult[0] = 2;
  sKnotMult[1] = 2;

  // create the BSplineCurve
  SER(SmBSplineCurve::CreateCanonical(crContext,          // in : context for new object construction
                                      lDimensionOfResult, // in : valid dimension - 2 or 3  
                                      1,                  // in : valid degree - 1 through 32  
                                      sCntrlPoly,         // in : Euclidian form of the control points.
                                      SM_CF_UNSPECIFIED,  // in : oneof  SM_CF_POLYLINE_FORM,  SM_CF_PARABOLIC_ARC, 
                                                          //             SM_CF_CIRCULAR_ARC,   SM_CF_HYPERBOLIC_ARC,
                                                          //             SM_CF_ELLIPTIC_ARC,   SM_CF_UNSPECIFIED,
                                                          //             SM_CF_HELICAL_ARC   
                                      sKnotMult,          // in : Multiplicities of the end knots must be lDegree+1 and
                                                          //      and internal multiplicities must be lDegree or less.
                                      sKnots,             // in : unique knots in ascending order
                                      SM_KT_UNSPECIFIED,  // in : oneof  SM_KT_UNIFORM_KNOTS,  SM_KT_QUASI_UNIFORM_KNOTS,      
                                                          //             SM_KT_UNSPECIFIED,    SM_KT_PIECEWISE_BEZIER_KNOTS    
                                      NULL,               // in : optional associated weigths for each control point
                                      NULL,               // in : If specified contains a trimming interval to resize the curve
                                      pBSC));             // out: New BSplineCurve
  NER(pBSC);

  // set output
  rpNewBSplineCurve = pBSC;

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateDegenerateCurve

/*******************************************************************//**
PURPOSE: Create a B-Spline curve exactly representing an ellipse 
    segment or an entire ellipse.

NOTES: Note that the ellipse starts at the start angle and goes
    counter clockwise until it hits the end angle.  An angle of 0 
    corresponds to the X axis of the Axis2Placement.  The start angle 
    can be greater than the end angle.  

    Start and End angles within about 1.0e-4 degrees of 0, 180, 360
    are snapped to those values.
    If both radii are equal it will return a SM_CF_CIRCULAR_ARC 
***********************************************************************/
SmStatus SmBSplineCurve::CreateEllipseSegment
  (const SmContext        & crContext,          // in : context for new object construction
   ULONG                    lDimensionOfResult, // in : 2 = Make UVTrimCurve in 2d, 3 = Make 3d XYZ curve
   const SmAxis2Placement & crReferenceFrame,   // in : ellipse center:[RefFrame origin], running CCW:[StartAng, EndAng in RefFrame XY plane]
   double                   dRadiusAtXAxis,     // in : ellipse Radius at RefFrame X, must be > 0,  when dRadiusAtXAxis == dRadiusAtYAxis          
   double                   dRadiusAtYAxis,     // in : ellipse Radius at RefFrame y, must be > 0,  rtns circular arc
   double                   dStartAngDeg,       // in : Start Angle Degrees, where: |dEndAngDeg - dStartAngDeg| <= 360.0 
   double                   dEndAngDeg,         // in : End Angle Degrees,   where: |dEndAngDeg - dStartAngDeg| <= 360.0  
   SmNurbCircleParam        eParameterization,  // in : SM_CO_QUADRATIC: degree 2 ellipse is created with double internal knots
                                                //      SM_CO_QUARTIC  : degree 4 ellipse is created possibly with quadruple internal knots
                                                //                         (but better parameterization than SM_CO_QUADRATIC)
                                                //      SM_CO_QUINTIC  : degree 5 ellipse is created with no internal knots
   SmBSplineCurve       *& rpNewBSplineCurve)   // out: New BSplineEllipseSegment
{
  RANGE_ER(2,lDimensionOfResult,3);
  RANGE_ER(0.0,dStartAngDeg,360.0);
  RANGE_ER(0.0,dEndAngDeg,360.0);
  LE_ZERO_ER(dRadiusAtXAxis);
  LE_ZERO_ER(dRadiusAtYAxis);
    
  NL_POINT C;
  NL_POINT X, Y;
  sm_ExtractFrame(crReferenceFrame,C,X,Y);
  NL_STACKS SC;
  gw_CURVE Cur;
  SmStackHandler sStackKp(&SC);
  N_CrvInitArrays(&Cur);
    
  gw_FLAG ctp=0;
  if (eParameterization == SM_CO_QUADRATIC) ctp = NL_QUADRATIC;  // degree 2
  else if (eParameterization == SM_CO_QUINTIC) ctp = NL_QUINTIC;  // degree 5
  else SER(SM_ERR_INVALID_INPUT);
    
  // Build the circular arc
  // gwc note: start and end angles within about 1.0e-4 degrees of
  //           0 and 180 are snapped to 0 and 180.
  //           so the constructed curve's domain may vary slightly
  //           from [dStartAngDeg dEndAngDeg]
  if(NL_YES == N_CreateEllipticalArc(C,X,Y,dRadiusAtXAxis,dRadiusAtYAxis,
                                     dStartAngDeg,dEndAngDeg,ctp,&Cur,&SC))
    { SER(SM_ERR); }

  //  GW_SER(N_CreateEllipticalArc(C,X,Y,dRadiusAtXAxis,dRadiusAtYAxis,
  //                               dStartAngDeg,dEndAngDeg,ctp,&Cur,&SC));
    
  if (lDimensionOfResult == 2) N_Crv3dTo2d(&Cur);
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(lDimensionOfResult,&Cur);

  rpNewBSplineCurve->m_eBSplineCurveForm = 
      (fabs(dRadiusAtXAxis - dRadiusAtYAxis) < SM_EFF_ZERO) ? SM_CF_CIRCULAR_ARC: SM_CF_ELLIPTIC_ARC;

  return SM_SUCCESS;

} // end SmBSplineCurve::CreateEllipseSegment

/*******************************************************************//**
PURPOSE: Create a B-Spline curve representing a line segment.

NOTES: The two points can not be the same.
***********************************************************************/
SmStatus SmBSplineCurve::CreateLineSegment
  (const SmContext  & crContext,           // in : context for new object construction
   ULONG              lDimensionOfResult,  // in : specify desired dimension: 2 or 3 
   const SmPoint3d  & crStartPoint,        // in : Line Start Point
   const SmPoint3d  & crEndPoint,          // in : Line End Point
   SmBSplineCurve  *& rpNewBSplineCurve)   // out: new BSpline Line [NULL on input]
{
  // check input
  RANGE_ER(2,lDimensionOfResult,3);
  SAME_PNT_ER(crStartPoint,crEndPoint);
 // SM_ASSERT_MSG(rpNewBSplineCurve == NULL, _T("SmBSplineCurve::CreateLineSegment NonNULL input pointer")) ;
  
  // NLIB compatable locals
  NL_POINT P;
  COPY_XYZ(crStartPoint,P);
  SmVector3d sLineVec = crEndPoint - crStartPoint;
  NL_POINT V;
  COPY_XYZ(sLineVec,V);
  
  NL_STACKS SC;
  SmStackHandler sStackKp(&SC);
  
  // allocate the NLIB Nurb
  gw_CURVE        *pNewCur   = sm_AllocateNurbCurve(1,1,3);

  if(NL_YES == N_CrvLineFromPtAndVector(P,V,pNewCur,&SC))
    { SER(SM_ERR); }
  // GW_SER(N_CrvLineFromPtAndVector(P,V,pNewCur,&SC));

  if (lDimensionOfResult == 2) N_Crv3dTo2d(pNewCur);

  // construct SMLib BSplineCurve
  rpNewBSplineCurve = new (crContext) SmBSplineCurve(pNewCur,lDimensionOfResult,FALSE);
  rpNewBSplineCurve->m_eBSplineCurveForm = SM_CF_POLYLINE_FORM;

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateLineSegment

/*******************************************************************//**
PURPOSE: Create a new curve which is a mirror of the curve about the
    X, Y plane of the SmAxis2Placement.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::CreateMirrorCurve
  (const SmContext & crContext,
   const SmAxis2Placement & crMirrorPlane,
   SmCurve *& rpMirrorCurve) 
  //const
{
  NL_STACKS SC;
  NL_RMATRIX rma;
  SmStackHandler sStackKp(&SC);
  N_InitRealMatrix(&rma);
  N_SetRealMatrix(&rma, 3, 3, NL_MT_FULL, 3, &SC);
  gw_REAL** RM = rma.RM;

  crMirrorPlane.GetMirrorMatrix(RM);

  gw_CURVE* pNurb = GetGwNurbPointer();
  if (!pNurb)
  {
      MakeNurb();
  }
  
  SmBSplineCurve* pBSC = new(crContext) SmBSplineCurve(*this);
  NER(pBSC);

  gw_CURVE* pCur = pBSC->GetOrCreateGwNurbPointer();
  N_CrvTransform(pCur, &rma);

  rpMirrorCurve = pBSC;
  
  return SM_SUCCESS;


} // end SmBSplineCurve::CreateMirrorCurve

/*******************************************************************//**
PURPOSE: Create a curve by projecting an existing curve into
    a plane using either parallel or perspective projection.

NOTES: If parallel projection then the direction of the 
    projection can not be perpendicular to the plane normal.  If 
    perspective projection the center of projection can not lie on
    the plane.
***********************************************************************/
SmStatus SmBSplineCurve::CreatePlaneProjection
  (const SmContext  & crContext,                // in : context for new object construction
   SmProjectionType   eProjectionType,          // in : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE
   const SmPoint3d  & rProjectionPlanePoint,    // in : Point on target plane 
   const SmVector3d & rProjectionPlaneNormal,   // in : Normal of target plane
   const SmVector3d & rProjDirOrCenterOfProj,   // in : When eProjectionType == SM_PT_PARALLEL, vector is projection direction.
                                                //      When eProjectionType == SM_PT_PERSPECTIVE, vector is 'eye' point of 
                                                //                                                 the projection.
   SmCurve *& rpProjectedCurve)                 // out: 
  const
{
  AERN_MSG(eProjectionType == SM_PT_PARALLEL || eProjectionType == SM_PT_PERSPECTIVE, 
           SM_ERR_INVALID_INPUT, 
           _T("SmBSplineCurve::CreatePlaneProjection, unsupported eProjectionType input option")) ;

    ZERO_VEC_ER(rProjectionPlaneNormal);
    
    // check for bad projection vector
  if (SM_IS_ZERO(rProjectionPlaneNormal.LengthSquared())) 
    {
        SER(SM_ERR_INVALID_INPUT);
    }

    // Check for bad projection direction
    gw_FLAG prj=0;
  if (eProjectionType == SM_PT_PARALLEL) 
    {
        prj = NL_PARALLEL;
      if (SM_IS_ZERO(rProjectionPlaneNormal.Dot(rProjDirOrCenterOfProj))) 
        {
            SER(SM_ERR_INVALID_INPUT);
        }
    }
  else if (eProjectionType == SM_PT_PERSPECTIVE) 
    {
        prj = NL_PERSPECTIVE;
        // Make sure center of projection is not on the plane
        SmPoint3d sPntOnPlane = 
            rProjDirOrCenterOfProj.ProjectPointToPlane(rProjectionPlanePoint,rProjectionPlaneNormal);
      if (SM_IS_ZERO(sPntOnPlane.DistanceBetween(rProjDirOrCenterOfProj))) 
        {
            // projection point can not be on the plane
            SER(SM_ERR_INVALID_INPUT);
        }
    }
    else SER(SM_ERR_INVALID_INPUT);
    
    NL_STACKS SC;
    gw_CURVE Cur;
    SmStackHandler sStackKp(&SC);
    N_CrvInitArrays(&Cur);
    
    NL_POINT P;
    COPY_XYZ(rProjectionPlanePoint,P);
    NL_POINT N, A;
    COPY_XYZ(rProjectionPlaneNormal,N);
    COPY_XYZ(rProjDirOrCenterOfProj,A);
    
    if(NL_YES == N_CrvProjectOntoPlane(((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer(),P,N,A,prj, &Cur, &SC))
      { SER(SM_ERR); }
    // GW_SER(N_CrvProjectOntoPlane(((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer(),P,N,A,prj, &Cur, &SC));
    
    rpProjectedCurve = new (crContext) SmBSplineCurve(3,&Cur);

  // all done
    return SM_SUCCESS;

} // end SmBSplineCurve::CreatePlaneProjection

/*******************************************************************//**
PURPOSE: Create a curve with zero length or degenerate 'point' curve.

NOTES: parameterized from 0 to 1 - see SmCurve::EditParameterization()
       when a specific parameter range is desired
***********************************************************************/
SmStatus SmBSplineCurve::CreatePointCurve
  (const SmContext & crContext,         // in : context for new object construction
   SmPoint3d       & rPnt,              // in : target point 
   SmBSplineCurve *& rpCurve)           // out: new Curve
{
  // pass the call along
  SER(SmBSplineCurve::CreateDegenerateCurve(crContext, 3, rPnt, rpCurve))  ; 

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreatePointCurve

/*******************************************************************//**
PURPOSE: Create a simple offset of the given curve.  

NOTES: The OffsetCurve is the approximation of the implicit offset.
    The resulting curve may have self-intersections.  
    See the SmCompositeCurve method for creating trimmed offsets method 
    which will trim away excess parts.
    Note that you may create a composite curve with only a single segment to
    do trimming on the offset of a single curve.
    This offset routine only works with curves which are G1 or
    better.  The curve DOES NOT have to lie in a plane whose normal is
    the offset plane normal.  However, the results of offsetting non-planar 
    curves is somewhat less predictable.
***********************************************************************/
SmStatus SmBSplineCurve::CreateSimpleOffset
 (const SmContext  & crContext,             // in : context for new object construction
  double             dApproxTol3d,          // in : Distance between curve created and theoretical
                                            //      exact offset, (1.0e-2 to 1.0e-6) are typical
                                            //      values used for curves of size 1.0.
  const SmVector3d & crOffsetPlaneNormal,   // in : Normal of the reference plane used for offseting.
  double             dOffsetDistance,       // in : BaseCurve/OffsetCurve dist in OffsetPlane.
                                            //      If this is a positive value the offset will lie to
                                            //      the right hand side of the curve as viewed from the
                                            //      positive side of the offset plane.  A negative offset
                                            //      value produces a curve on the left hand side of the
                                            //      base curve.
  SmBSplineCurve  *& rpNewBSplineCurve,     // out: If there is no valid offset, this value will be NULL.
                                            //      Otherwise, it will be offset curve.
  double           & rdMaxGap3d,            // out: Max error of offset approximation.
  SmBoolean      bOptMatchParameterization) // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                            //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                            //      default:[FALSE], FALSE produces lower control point count curves for slightly more cost.
 const
{
  // return SM_ERR_INVALID_INPUT when length of crOffsetPlaneNormal is zero 
  ZERO_VEC_ER(crOffsetPlaneNormal);

  // set outputs
  rdMaxGap3d = 0.0;

  // return SM_ERR_INVALID_INPUT when curve's internal continuities are less than C0
  SmTArray<SmContinuityType> sContinuities;
  SmContinuityType eMinContinuity;
  SER(CalculateContinuities(eMinContinuity,sContinuities));
  if (eMinContinuity <= SM_CT_C0) 
    {
      SER(SM_ERR_INVALID_INPUT);  // not enough continuity in this curve
    }

  // do arc-offset directly
  SmAxis2Placement sReferenceFrame;
  double dRadius,dStartAng,dEndAng;    
  if (this->IsArc(11,dApproxTol3d/100.0,sReferenceFrame,dRadius,dStartAng,dEndAng)) 
    {
      SmVector3d sZ = sReferenceFrame.GetZAxis();
      double dAngle;
      SER(sZ.AngleBetween(crOffsetPlaneNormal,dAngle));
      if (dAngle < SM_EFF_ZERO_SQRT || dAngle > SM_PI - SM_EFF_ZERO_SQRT) 
        {
          if (sZ.Dot(crOffsetPlaneNormal) > 0.0)
              dRadius += dOffsetDistance;
          else
              dRadius -= dOffsetDistance;
          if (dRadius < 0.0) 
            {
              dRadius *= -1.0;
              SmPoint3d sOrigin;
              SmVector3d sXAxis;
              SmVector3d sYAxis;
              sReferenceFrame.GetCanonical(sOrigin,sXAxis,sYAxis);
              sXAxis = -sXAxis;
              sYAxis = -sYAxis;
              sReferenceFrame.SetCanonical(sOrigin,sXAxis,sYAxis);
            }
          if (dRadius < dApproxTol3d) 
            { 
              // Offset is a degenerate curve.
              SER(SmBSplineCurve::CreateDegenerateCurve(crContext,3,
                  sReferenceFrame.GetOriginRef(),rpNewBSplineCurve));
              NER(rpNewBSplineCurve);
              return SM_SUCCESS; 
            }
          SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sReferenceFrame,dRadius,
              dStartAng,dEndAng,SM_CO_QUADRATIC,rpNewBSplineCurve));
          NER(rpNewBSplineCurve);
          SmExtent1d sOrigIvl = GetNaturalInterval();
          SER(rpNewBSplineCurve->EditParameterization(sOrigIvl));
          return SM_SUCCESS;
        } // end angle is zero or SM_PI check
    } // end IsArc check

  // arrive here for all curves 
  // but arcs spanning angles less than tol or greater than 180 degrees

  // Create an implicit offset and approximate it
  SmExtent1d sIvl = GetNaturalInterval();
  SmOffsetCurve sOff(3,*this,sIvl,crOffsetPlaneNormal,dOffsetDistance);
  sOff.SetContext(NULL);
  SmBSplineCurve *pOffsetApprox;
  double dMaxGap3d;
  SmTArray<double> sBreaks;
  sBreaks.Add(sIvl.GetMin());
  sBreaks.Add(sIvl.GetMax());

  // build the approximation
  SER( sOff.ApproximateCurve( crContext,  
                              sBreaks,
                              dApproxTol3d, 
                              dMaxGap3d, 
                              pOffsetApprox, 
                              bOptMatchParameterization, 
                              FALSE ));

  // set output
  rpNewBSplineCurve = pOffsetApprox;
  rdMaxGap3d        = dMaxGap3d;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,0);
      DrawWithKnots();
      smgfx_SetColor(1,0,0);
      rpNewBSplineCurve->DrawWithKnots();
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::CreateSimpleOffset

/*******************************************************************//**
PURPOSE: Create a section curve through other curves with possible tangency.

NOTES:
***********************************************************************/

SmStatus SmBSplineCurve::CreateSectionCurve
( 
  const SmContext &                 crContext,              ///< [in] :   Context where new curve construction takes place <br>
  double                            dNormalizedParameter,   ///< [in] :   Parameter on each input curve through which to build new curve <br>
  ULONG                             lParameterization,      ///< [in] :   0 - Uniform, 1 - Chord Length, 2 - Centripetal <br>   
  double                            dTangentStrength,       ///< [in] :   Determine magnitude of tangent (only when cpStart(End)Tangent specified )  <br>
  ULONG                             lContinuity,            ///< [in] :   0 - C0 continuity,  1 - G1 Continuity, 2 - G2 Continuity <br> 
  const SmTArray<SmBSplineCurve*> & crSectionCurves,        ///< [in] :   Input curves <br>
  const SmBSplineSurface          * cpStartTangent,         ///< [in] :   Optional input surface to determine start tangent of result <br>  
  const SmBSplineSurface          * cpEndTangent,           ///< [in] :   Optional input surface to determine end tangent of result   <br>
  SmBSplineCurve                 *& rpNewSection            ///< [out]:   Result curve <br>
) const
{
  SmTArray<SmPoint3d> sPointsStart, sPointsEnd;
  for(ULONG i = 0; i < crSectionCurves.GetSize(); i++)
  {
    SmBSplineCurve *pCrv = crSectionCurves[i];
    SmExtent1d sIvl = pCrv->GetNaturalInterval();
    SmPoint3d sPnt;
    pCrv->EvaluatePoint( sIvl.Evaluate( dNormalizedParameter ), sPnt );
    sPointsStart.Add( sPnt );
  }

  ULONG lDegree = 3;
  if(sPointsStart.GetSize() == 3) { lDegree = 2; }
  if(sPointsStart.GetSize() == 2) { lDegree = 1; }
  if(sPointsStart.GetSize() < 2) { return SM_ERR; }

  SmTArray<SmVector3d> sVecs;
  SmTArray<SmVector3d> sSeconds;

  SmCurveParameterizationType eParameterization = SM_CP_UNIFORM;
  if(lParameterization == 1) eParameterization = SM_CP_CHORDLENGTH;
  if(lParameterization == 2) eParameterization = SM_CP_CENTRIPETAL;

  SmVector3d sPnt, sDU, sDV, sDUV, sDUU, sDVV;

  if(cpStartTangent)
  {
    SmExtent2d sDom = cpStartTangent->GetNaturalUVDomain();
    SER( cpStartTangent->Evaluate2ndDerivatives( sDom.Evaluate( 0.0, dNormalizedParameter ),
         TRUE, TRUE, sPnt, sDU, sDV, sDUV, sDUU, sDVV ) );
    sDU = sDU * sPointsStart.GetSize() * dTangentStrength;
    if(lContinuity > 0)
    {
      sVecs.Add( sDU );
      if(lDegree < 3) lDegree++;
    }
    sDUU = sDUU * sPointsStart.GetSize() * dTangentStrength;
    if(lContinuity > 1)
    {
      sSeconds.Add( sDUU );
      if(lDegree < 3) lDegree++;
    }
  }

  if(cpEndTangent)
  {
    SmExtent2d sDom = cpEndTangent->GetNaturalUVDomain();
    SER( cpEndTangent->Evaluate2ndDerivatives( sDom.Evaluate( 1.0, dNormalizedParameter ),
         TRUE, TRUE, sPnt, sDU, sDV, sDUV, sDUU, sDVV ) );
    sDU = sDU * sPointsStart.GetSize() * dTangentStrength;
    if(lContinuity > 0)
    {
      sVecs.Add( sDU );
      if(lDegree < 3) lDegree++;
    }
    sDUU = sDUU * sPointsStart.GetSize() * dTangentStrength;
    if(lContinuity > 1)
    {
      sSeconds.Add( sDUU );
      if(lDegree < 3) lDegree++;
    }
  }

  SER( SmBSplineCurve::CreateInterpolatingCurve( crContext, eParameterization, 3,
       lDegree, sPointsStart, sVecs, &sSeconds, FALSE, rpNewSection ) );

  return SM_SUCCESS;

} // end SmBSplineCurve::CreateSectionCurve

/*******************************************************************//**
PURPOSE: Change the NURB parameterization of the curve to the given 
         parameterization.  

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::EditParameterization
  (const SmExtent1d & crNewParameterization, // in : new parameter range for curve
   SmBoolean          bNotify)               // in : TRUE  = make notify calls (previous behavior)
                                             //      FALSE = Skip notify call
                                             //      default:[TRUE]
{
  // get current interval
  SmExtent1d sOldIvl = GetNaturalInterval();

  // low work: current interval == new interval
  if (   SM_ARE_SAME(sOldIvl.GetMin(),crNewParameterization.GetMin())
      && SM_ARE_SAME(sOldIvl.GetMax(),crNewParameterization.GetMax()) ) 
    { return SM_SUCCESS; }

  // switch to NLib parameters
  gw_CURVE *pCur = GetOrCreateGwNurbPointer();

  // MakeNurb() can fail for degenerate inputs (e.g. a zero-radius circle),
  // leaving a NULL basis that N_BasisReparam() below would deref (the assert in
  // GetOrCreateGwNurbPointer() is a no-op in release). Fail gracefully.
  if (pCur == NULL || pCur->knt == NULL || pCur->knt->U == NULL)
    {
      SE_MSG(SM_ERR, _T("SmBSplineCurve::EditParameterization: degenerate curve has no NURB basis; skipping reparameterization")) ;
      return SM_ERR ;
    }

  gw_INTERVAL ivl;
  ivl.ul = crNewParameterization.GetMin();
  ivl.ur = crNewParameterization.GetMax();
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // make the NLib call
  N_BasisReparam(pCur->knt,pCur->p,ivl);

  // Notify the BSpline
  if(bNotify == TRUE)
    {
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::EditParameterization

/*******************************************************************//**
PURPOSE: Edit one of the endpoints of the NURB curve by replacing it
   with the given point which should be near the previous end point.

NOTES: This method has been modified recently to handle the
   case where the weight is not given and the curve is rational.  It 
   assumes that the sNewEndPoint is a Euclidian point if the weight is
   not given.

   gwc:Note - this function should be used sparingly because it can
              easily ruin the curvature at the end of a curve.
***********************************************************************/
SmStatus SmBSplineCurve::EditEndPoint
  (const SmPoint3d sNewEndPoint,   // in : If weight is given: X,Y,Z of a homogeneous point.
                                   //      Otherwise           X,Y,Z of a Euclidian point.
   SmBoolean bEditStartPoint,      // in : TRUE = modify startPoint, else endPoint
   const double * pdOptWeight,     // in : Used to set rational CPoints, NULL for NonRational Curves
   SmEditEndType eNewEndPointType, // NotUsed: in : Specify how sNewEndPoint was selected (for debug purposes)
                                   //      default:[SM_EE_UNKNOWN]
   SmBoolean bHealUseFlag)         // NotUsed: in : TRUE = called to heal loop gap problems,
                                   //               when in debug mode, output large move warnings
                                   //      FALSE = called to construct geometry
                                   //               when in debug mode, don't output move warnings
                                   //      default:[TRUE]
{
  SM_REF2(eNewEndPointType, bHealUseFlag) ;
  if(m_pNurb == NULL) { MakeNurb(); } SM_ASSERT(m_pNurb != NULL) ;
  gw_CURVE    *pCur      = SM_REINTERPRET_CAST(gw_CURVE*,m_pNurb);
  gw_CPOLYGON *pCPOLYGON = pCur->pol; // Use natural domain

#ifdef SM_DEBUG_CODE
#ifdef GWC
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // gwc: let's keep track of when this method is getting called in debug mode.
  if(bHealUseFlag && GetNumberControlPoints() > 2)
    {
      // point and tangent on the curve (projected through surface when curve is a UVTrimCurve)
      ULONG lDim = GetDim() ;
      SmPoint3d sPV[2], sPV3D[3], sNewEndPoint3D ;
      double dNormalizedParam = bEditStartPoint ? 0.0 : 1.0 ;
      double dParam           = GetNaturalInterval().Evaluate(dNormalizedParam) ;
      Evaluate(dParam, 1, TRUE, sPV) ;
      if(sPV[1].Length() > 1000.0 * SM_EFF_ZERO) { sPV[1].Unitize() ; }

      // curve topology object users
      SmEdge    *pEdge    = (GetOwner() && GetOwner()->IsKindOf(SmEdge_TYPE)   ) ? (SmEdge *)GetEdge() : NULL ;
      SmFace    *pFace    = (GetOwner() && GetOwner()->IsKindOf(SmFace_TYPE)   ) ? (SmFace *)GetFace() : NULL ;
      SmEdgeuse *pEdgeuse = (GetOwner() && GetOwner()->IsKindOf(SmEdgeuse_TYPE)) ? (SmEdgeuse *)GetOwner() : NULL ;
      if(pEdgeuse){ pEdge = pEdgeuse->GetEdge() ; 
                    pFace = pEdgeuse->GetFace() ;
                  }
      SmSurface *pSurface = pFace ? pFace->GetSurface() : NULL ;
      SmBrep    *pBrep    =   pEdge ? pEdge->GetBrep() 
                            : pFace ? pFace->GetBrep()
                            : NULL ;

// inside SM_DEBUG_CODE block

      // curve position, tangent, and target point (projected from UVTrimCurve through Surface if appropriate)
      if(pEdgeuse) { double dCurvature ;
                     SmPoint2d sUV(sPV[0].x, sPV[0].y) ;
                     pSurface->EvaluatePoint(sUV, sPV3D[0]) ;
                     pSurface->EvaluateNormalSection(sUV, TRUE, TRUE, sPV[1], sPV3D[1], dCurvature, sPV3D[2]) ; 
                     if(sNewEndPoint.z == 0 || sNewEndPoint.z == NL_NOZ)
                       {
                         SmPoint2d sNewUV(sNewEndPoint.x, sNewEndPoint.y) ;
                         pSurface->EvaluatePoint(sNewUV,sNewEndPoint3D) ;
                       }
                   } 
      else         { sPV3D[0]       = sPV[0] ;         // curve position to move
                     sPV3D[1]       = sPV[1] ;         // curve tangent at curve position
                     sNewEndPoint3D = sNewEndPoint ;   // move target
                   }

      // NewEndPoint to OldEndPoint Gap and its orthogonal decomposition
      SmPoint3d sGap       = sNewEndPoint3D - sPV3D[0] ;
      double    dGapSize   = sGap.Length() ;
      double    dTanDist   = sGap.Dot(sPV3D[1]) ;
      double    dOrthoDist = (sGap - dTanDist * sPV3D[1]).Length() ;
      dTanDist = smos_Fabs(dTanDist) ;

// inside SM_DEBUG_CODE block

      // curve and brep tolerances
      double dScaledZero = SM_EFF_ZERO_SQRT * (1.0 + sNewEndPoint3D.GetMaxDimension()) ;
      double dBrepTol    = pBrep ? (double)pBrep->GetTolerance()/100.0 : 0.0 ;
      double dCheckTol   = smos_Max(dScaledZero, dBrepTol) ;
      if(   dTanDist   > dCheckTol
         || dOrthoDist > dCheckTol
         || dGapSize   > dCheckTol)
        {
          if(pEdgeuse)
            { SM_ASSERT(lDim == 2) ;
              smos_sprintf(sBuff, _T("UVTrimCurve EndControlPoint move: to[%s] BSpline[dim=%d,deg=%d,cpt#=%d]\n               [GapDist, TanDist, OrthoDist, Tol] = [%16.16lf, %16.16lf, %16.16lf, %16.16lf]"), 
                         sEditEndTypeDesc[eNewEndPointType],
                         GetDim(),                                                                 
                         GetDegree(), 
                         GetNumberControlPoints(),
                         dGapSize, dTanDist, dOrthoDist, dCheckTol);
            }
// inside SM_DEBUG_CODE block

          else if(lDim == 3)
            { smos_sprintf(sBuff, _T("3DCurve EndControlPoint move: to[%s] BSpline[dim=%d,deg=%d,cpt#=%d]\n               [GapDist, TanDist, OrthoDist, Tol] = [%16.16lf, %16.16lf, %16.16lf, %16.16lf]"), 
                         sEditEndTypeDesc[eNewEndPointType],
                         GetDim(),
                         GetDegree(), 
                         GetNumberControlPoints(),
                         dGapSize, dTanDist, dOrthoDist, dCheckTol);
            }
          else
            {
              SM_ASSERT(lDim == 2) ;
              smos_sprintf(sBuff, _T("2DCurve EndControlPoint move: to[%s] BSpline[dim=%d,deg=%d,cpt#=%d]\n               [GapDist, TanDist, OrthoDist, Tol] = [%16.16lf, %16.16lf, %16.16lf, %16.16lf]"), 
                         sEditEndTypeDesc[eNewEndPointType],
                         GetDim(),
                         GetDegree(), 
                         GetNumberControlPoints(),
                         dGapSize, dTanDist, dOrthoDist, dCheckTol);
            }
          SM_DBG_WARN(sBuff) ;

// inside SM_DEBUG_CODE block
SmBoolean bDebugMe = FALSE ;
          if(bDebugMe)
            {
              if(pBrep)    pBrep->Dump() ;
              if(pFace)    pFace->Dump() ;
              if(pEdge)    pEdge->Dump() ;
              if(pEdgeuse) pEdgeuse->Dump() ;
              this->Dump() ; 

              // draw brep and edge
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,1,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;

              // draw tgt curve properties and UVtrimCurve projected through Surface (if appropriate), 
              smgfx_SetLook(3,4, 0,0,1) ; if(!pEdgeuse) this->DrawControlPoints() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 0,0,1) ; if(!pEdgeuse) this->DrawParams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; if(pEdgeuse && pFace) { SmCrvOnSurf sCrvOnSurf(*this, *pFace->GetSurface()) ;
                                                                  sCrvOnSurf.Draw() ; sm_GraphicsLoop() ;
                                                                  sCrvOnSurf.DrawParams() ; sm_GraphicsLoop() ;
                                                                  smgfx_SetLook(3,4, 1,.5,.5) ; if(pSurface) pSurface->DrawUV(10,10) ; sm_GraphicsLoop() ;
                                                                  smgfx_SetLook(3,4, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                                                                }
              // draw tgt point and gap
              smgfx_SetLook(5,6, 1,0,0) ; sNewEndPoint3D.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,1) ; sNewEndPoint3D.DrawPointToPoint(sPV3D[0]) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
        }
    }
#endif // GWC
#endif // SM_DEBUG_CODE

  // inform the public - usually discards old caches
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // when editing the Start Point
  if (bEditStartPoint) 
    {
      // set coordinates
      pCPOLYGON->Pw[0].x = sNewEndPoint.x;
      pCPOLYGON->Pw[0].y = sNewEndPoint.y;
      if (pCPOLYGON->Pw[0].z != NL_NOZ) { pCPOLYGON->Pw[0].z = sNewEndPoint.z;
                                      }

      // handle weights
      if (pdOptWeight)                
        { if (pCPOLYGON->Pw[0].w == NL_NOW) SER(SM_ERR);
          pCPOLYGON->Pw[0].w = *pdOptWeight;
        }
      // assume point needs to be projected into homogeneous space
      else if (pCPOLYGON->Pw[0].w != NL_NOW) 
        {
          pCPOLYGON->Pw[0].x *= pCPOLYGON->Pw[0].w;
          pCPOLYGON->Pw[0].y *= pCPOLYGON->Pw[0].w;
          if (pCPOLYGON->Pw[0].z != NL_NOZ) { pCPOLYGON->Pw[0].z *= pCPOLYGON->Pw[0].w;
                                          }
        }

      // Give derived Analytic curves a chance to update their 'step' data
      RefreshAnalytics() ;

      // watch out for errors
      if(IsKindOf(SmEllipse_TYPE))
        {
          ERR_MSG(_T("Moving endPoint on a Circle or Ellipse stops it from being a circle or Ellipse")) ;
        }
    } // end StartPoint editing
  else // edit EndPoint
    {
      gw_INDEX n = pCPOLYGON->n;  

      // set coordinates
      pCPOLYGON->Pw[n].x = sNewEndPoint.x;
      pCPOLYGON->Pw[n].y = sNewEndPoint.y;
      if (pCPOLYGON->Pw[n].z != NL_NOZ) { pCPOLYGON->Pw[n].z = sNewEndPoint.z;
                                      }
      // handle weights
      if (pdOptWeight) 
        {
          if (pCPOLYGON->Pw[n].w == NL_NOW) SER(SM_ERR);
          pCPOLYGON->Pw[n].w = *pdOptWeight;
        }
      // assume point needs to be projected into homogeneous space
      else if (pCPOLYGON->Pw[n].w != NL_NOW) 
        {
          pCPOLYGON->Pw[n].x *= pCPOLYGON->Pw[n].w;
          pCPOLYGON->Pw[n].y *= pCPOLYGON->Pw[n].w;
          if (pCPOLYGON->Pw[n].z != NL_NOZ) { pCPOLYGON->Pw[n].z *= pCPOLYGON->Pw[n].w;
                                          }
        }

      // Give derived Analytic curves a chance to update their 'step' data
      RefreshAnalytics() ;

      // watch out for errors
      if(IsKindOf(SmEllipse_TYPE))
        {
          ERR_MSG(_T("Moving endPoint on a Circle or Ellipse stops it from being a circle or Ellipse")) ;
        }
    } // end EndPoint editing

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Someday maybe we should make sure that we didn't put any kinks in
  // the end of the curve.
  // gwc:someday should be today!

  return SM_SUCCESS;

} // end SmBSplineCurve::EditEndPoint
#ifdef SM_DEBUG_CODE
static ULONG s_lCurveEvalCount = 0;
#endif

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
ULONG smcurv_GetCurveEvalCount() 
{ 
    #ifdef SM_DEBUG_CODE
    return s_lCurveEvalCount;
    #else
    return 0;
    #endif

} // end smcurv_GetCurveEvalCount

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optional derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmBSplineCurve::Evaluate
 (double     dParameter,              // in : tgt param
  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
  SmBoolean  bFromLeft,               // in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d aPointAndDerivatives[],  // out: array or (pos, tang, 2nd deriv, ...), sized:[lNumDerivatives+1]
  SmBoolean  bNonZeroTangents)        // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
    const                             //      FALSE= return exact tangent values
                                      //      note: Surprisingly TRUE is the common choice because most tangent uses
                                      //            are for their direction (Binorm, SurfNorm comps), but when the 
                                      //            tangent is being used for its magnitude (like an arc-length comp)
                                      //            then set this to FALSE.
                                      //      default:[TRUE]
{
// every 10,000 calls check to see if the escape key is pressed - if so, quit
// static ULONG lCounter = 0;
//  lCounter ++;
//  if((lCounter % 10000 == 0) && smos_ExcapeCallback() ) 
//    { return SM_ERR; } // JLMCC removed per FS

  // pass simple calls along to EvaluatePoint
  if (lNumDerivatives == 0) 
    {
      SER(EvaluatePoint(dParameter,aPointAndDerivatives[0]));
      return SM_SUCCESS; 
    }

  // accumulate total number of Evaluates - see smcurv_GetCurveEvalCount()
  #ifdef SM_DEBUG_CODE
  s_lCurveEvalCount ++;
  #endif
  // natural extent
  gw_CURVE *pCurve = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  SmExtent1d sNaturalInterval(pCurve->knt->U[0], pCurve->knt->U[pCurve->knt->m]) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump_NCrv(pCurve, false) ;

      SmEdge *pEdge = (SmEdge *)this->GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(3,4, 1,0,0) ; this->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 5, 6, 1, 0, 1 ); if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when dParameter is out-of-bounds and not allowed to evaluate out-of-bounds 
  if (   !sNaturalInterval.ContainsValue(dParameter)
      && !IsOutOfBoundsEnabled()) // GWC:MODIFIED OUTOFBOUNDS
    {
      // clamp output of bounds parameters to natural interval
      dParameter = sNaturalInterval.ClampValue(dParameter);        
    }

  // pass the call along For the NLib BasisEval calls and the summing of curve values 
  gw_FLAG flg = bFromLeft ? NL_LEFT : NL_RIGHT;
  if(NL_YES == sm_CrvDerivs(pCurve,
                         dParameter,
                         flg,
                         (gw_INDEX)lNumDerivatives,
                         SM_REINTERPRET_CAST( NL_POINT*, aPointAndDerivatives)) )
    { SER(SM_ERR); }

  // When asked, try to find a direction for zero-length first derivatives based on the
  //  2nd derivative value
  // (This is the same trick as is used for surface evaluations.)
  if(bNonZeroTangents)
    {
      double dScaledZero = SM_EFF_ZERO * (1.0 + aPointAndDerivatives[0].GetMaxDimension());
      double dNewLen     = 1.1 * dScaledZero ;

      // see if the 1st order derivative is smaller than dNewLen (this gives a continuous modified function)
      if(   lNumDerivatives >= 1
         && aPointAndDerivatives[1].LengthSquared() < dNewLen * dNewLen)
        {
          // make sure 2nd order derivatives are available
          if(lNumDerivatives < 2)
            { 
              // recurse back to this function so the next section sets the 1st derivative
              SmVector3d sVals[3] ;
              Evaluate(dParameter, 2, bFromLeft, sVals) ;

              // save the 1st derivative
              aPointAndDerivatives[1] = sVals[1] ;
            }
          else // fix the zero 1st derivative function here
            {
              double dLen  = aPointAndDerivatives[2].Length();
              if(dLen > dNewLen)
                {
                  aPointAndDerivatives[1] = (dNewLen/dLen) * aPointAndDerivatives[2] ;

                  // If we're at the 'top' of the domain, i.e., the 'good' parameter is
                  // entering a singularity instead of leaving it, then the degenerate
                  // derivative is shrinking, so its change (the 2nd derivative) is in the opposite
                  // direction of the derivative.
                  if(dParameter > GetNaturalInterval().GetMid() )
                    {
                      aPointAndDerivatives[1] *= -1;
                    }
                } // end 2nd derivative is non-zero check
            } // end fix the zero 1st derivative function here branch
        } // end if the 1st derivative is zero check
    } // if fixing zero 1st derivative direction with 2nd derivative values check


//      
//        // we (GWC) have decided to handle one case of zero 1st derivative values
//        //  - when the 1st deriviative is zero at the natural end of the curve
//        //    due to the two last (or first) control points being coincident
//        //  - then compute a 1st derivtive sampled a small way into the last (or first) span.
//        //  - To prevent a discontinous 1st derivative function over the length of the
//        //    curve, all evaluations between
//        //    the end of the curve and the sample point are set to use the same
//        //    1st derivative value, i.e. that one sampled at the sample point.
//        //  - This means, that we have to check for the last two end control point
//        //    coincident case for more than just the 1st derivative = 0 check.
//        // when evaluation is between the sample point and the end of a curve with
//        // a pair of ending coincident control points - exchange 1st derivative
//        // actual value with the 1st derivative value taken at the sample point.
//        //    For consistency - change the 2nd derivative value to zero.  We are
//        //     treating this bit of curve as a straight segment no matter what
//        //     its actual shape. This will put a C2 dicontinuity into what should
//        //     normally be seen as a continuous function.  It's unclear what this
//        //     hack will do to the various operators implemented in SMLib, but the
//        //     hope is that this will allow these cases to rumbel through the system
//        //     without causing any damage.
//        //
//        // rule: sample point is now 2% of the way into the first (or last span)
//      
//        // begin repeated control point protection here
//        if(FALSE) // this method not working
//          {
//            // is this evaluation near a natural curve end
//            NL_INDEX  iM = pCurve->knt->m ;
//            NL_DEGREE iP = pCurve->p ;
//            double dNormalizedSmpDist = .002 ; // a number between 0 adn 1 - closer to 0 the better.
//            double dNearStartParam =   (1-dNormalizedSmpDist) * pCurve->knt->U[iP]           
//                                     +   dNormalizedSmpDist   * pCurve->knt->U[iP + 1] ;
//      
//            double dNearEndParam   =     dNormalizedSmpDist   * pCurve->knt->U[iM - iP - 1 ] 
//                                     + (1-dNormalizedSmpDist) * pCurve->knt->U[iM - iP] ;
//                    
//            SmBoolean bNearStart = SM_IS_CONTAINED(dParameter, pCurve->knt->U[iP], dNearStartParam) ;
//            SmBoolean bNearEnd   = SM_IS_CONTAINED(dParameter, dNearEndParam,      pCurve->knt->U[iM - iP]) ;
//      
//            // if eval parameter is near the start of the natural interval
//            if(bNearStart)
//              {
//                double dScaledZero = SM_EFF_ZERO * (1.0 + aPointAndDerivatives[0].GetMaxDimension()) ;
//      
//                // get dist between 1st pair of starting control points
//                double dCPtDist ;
//                N_DistCptCpt( pCurve->pol->Pw[0], pCurve->pol->Pw[1], &dCPtDist) ;
//      
//                // if this curve has a coincident pair of starting control points
//                if(dCPtDist < dScaledZero)
//                  {
//                    // evaluate curve at dNearStartParam to get tangent vector.
//                    // if the 1st span length is zero (an unexpected error condition)
//                    //   then we'll just re-evaluate the same point - can't do better
//                    NL_POINT sPV[2] ;
//                    sm_CrvDerivs(pCurve, dNearStartParam, flg, 1, sPV) ;
//      
//                    // For some cases (like a degenerate curve) this new deriviative should be zero
//                    // So no check on the output is needed here - just use whatever was found.
//                    aPointAndDerivatives[1] = *((SmVector3d *)(&sPV[1])) ;
//                    if(lNumDerivatives >= 2) 
//                      { aPointAndDerivatives[2].Set(0.0, 0.0, 0.0) ; }
//          
//      #ifdef SM_DEBUG_CODE
//                   if(bDebugMe)
//                     {
//                       Dump_NCrv(pCurve, false) ;
//                     }
//      #endif // SM_DEBUG_CODE
//      
//                  } // end first two CPoints are coincident check - so fudge a 1st deriv value
//              } // end param is at the natural interval start branch
//      
//            // else if eval parameter is near the end of the natural interval 
//            else if(bNearEnd)
//              {
//                double dScaledZero = SM_EFF_ZERO * (1.0 + aPointAndDerivatives[0].GetMaxDimension()) ;
//      
//                // get dist between last pair of ending control points
//                double dCPtDist ;
//                NL_INDEX iN = pCurve->pol->n ;
//                N_DistCptCpt( pCurve->pol->Pw[iN - 1], pCurve->pol->Pw[iN], &dCPtDist) ;
//      
//                // if this curve has a coincident pair of starting control points
//                if(dCPtDist < dScaledZero)
//                  {
//                    // evaluate curve at dNearEndParam to get tangent vector.
//                    // if the 1st span length is zero (an unexpected error condition)
//                    //   then we'll just re-evaluate the same point - can't do better
//                    NL_POINT sPV[2] ;
//                    sm_CrvDerivs(pCurve, dNearEndParam, flg, 1, sPV) ;
//      
//                    // For some cases (like a degenerate curve) this new deriviative should be zero
//                    // So no check on the output is needed here - just use whatever was found.
//                    aPointAndDerivatives[1] = *((SmVector3d *)(&sPV[1])) ;
//                    if(lNumDerivatives >= 2) 
//                      { aPointAndDerivatives[2].Set(0.0, 0.0, 0.0) ; }
//      
//      #ifdef SM_DEBUG_CODE
//                   if(bDebugMe)
//                     {
//                       Dump_NCrv(pCurve, false) ;
//                     }
//      #endif // SM_DEBUG_CODE
//      
//                  } // end last two CPoints are coincident check - so fudge a 1st deriv value
//              } // end param is at the natural interval end branch
//          } // end repeated control point kludge hack here
//                
//        // Make sure that the first derivative is non-zero when possible
//        // by stepping off a little.
//        if(   FALSE 
//           && lNumDerivatives > 0) 
//           {
//            double dStepOff = 0.001;
//            while(   aPointAndDerivatives[1].LengthSquared() < SM_EFF_ZERO_SQ*100.0 
//                  && dStepOff < 0.5) 
//              {
//                if (dParameter < sNaturalInterval.Evaluate(dStepOff)) 
//                  {
//                    dParameter = sNaturalInterval.Evaluate(dStepOff);
//                  }
//                else if (dParameter > sNaturalInterval.Evaluate(1.0-dStepOff)) 
//                  {
//                    dParameter = sNaturalInterval.Evaluate(1.0-dStepOff);
//                  }
//                else if (bFromLeft) 
//                  {
//                    dParameter = dParameter - dStepOff * sNaturalInterval.GetLength();
//                  }
//                else  
//                  { /* if (bFromRight)  */
//                    dParameter = dParameter + dStepOff * sNaturalInterval.GetLength();
//                  }
//      
//                dParameter = sNaturalInterval.ClampValue(dParameter);
//      
//                SmVector3d sPV[2];
//                GW_SER(sm_CrvDerivs(pCurve,
//                                    dParameter,
//                                    flg,
//                                    (gw_INDEX)lNumDerivatives,
//                                    SM_REINTERPRET_CAST(NL_POINT*, sPV)));
//                aPointAndDerivatives[1] = sPV[1];
//                dStepOff *= 4.0;
//              }
//          }

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::Evaluate

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/

SmStatus SmBSplineCurve::EvaluatePoint
  (double      dParameter, 
   SmPoint3d & rPoint) 
 const
{ 
  // check input
#ifdef SM_DEBUG_CODE
//static const SmBSplineCurve *pLast = NULL ;
//        SM_ASSERT_MSG((pLast == this || IsBounded()), 
//                       _T("Err: Using BSpline evaluator on infinite curve") );
//  if(!IsBounded()) { pLast = this ; }
#endif                  

// every 10,000 calls check to see if the escape key is pressed - if so, quit
//static ULONG lCounter = 0;
//  lCounter ++;
 // if((lCounter % 10000 == 0) && smos_ExcapeCallback() ) 
  //  { return SM_ERR; }

  // accumulate total number of Evaluates - see smcurv_GetCurveEvalCount()
#ifdef SM_DEBUG_CODE
  s_lCurveEvalCount ++;
#endif

  // natural extent
  gw_CURVE *pCurve = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  SmExtent1d sNaturalInterval(pCurve->knt->U[0], pCurve->knt->U[pCurve->knt->m]) ;

  // when dParameter is out-of-bounds and not allowed to evaluate out-of-bounds 
  if (   !sNaturalInterval.ContainsValue(dParameter)
      && !IsOutOfBoundsEnabled()) // GWC:MODIFIED OUTOFBOUNDS
    {
      // clamp output of bounds parameters to natural interval
      dParameter = sNaturalInterval.ClampValue(dParameter);        
    }

  // low work - begin point evaluation
  if (dParameter == sNaturalInterval.GetMin()) 
    {
      TO_EUCLID(pCurve->pol->Pw[0], rPoint);
      return SM_SUCCESS;
    }
  // low work - end point evaluation
  else if (dParameter == sNaturalInterval.GetMax()) 
    {
      TO_EUCLID(pCurve->pol->Pw[pCurve->pol->n], rPoint);
      return SM_SUCCESS;
    }
  
  // pass the call along  
  gw_FLAG flg = NL_LEFT;  // LEFT/RIGHT doesn't matter on C0 curves for point evals
  NL_POINT C;
  gw_PARAMETER u = dParameter;
  if(NL_YES == sm_CrvEval(pCurve, u, flg, &C))
    { SER(SM_ERR); }
  // GW_SER(sm_CrvEval(pCurve, u, flg, &C) );

  // set output
  COPY_XYZ(C, rPoint);

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::EvaluatePoint

/*******************************************************************//**
PURPOSE: Compute the curvature of a curve at a given parameter

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
    Note: Radius Of Curvature = 1.0/Curvature
***********************************************************************/

SmStatus SmBSplineCurve::EvaluateCurvature
  (double dParameter, 
   double &rCurvature) 
 const
{ 
    SmExtent1d sNaturalInterval;
    gw_CURVE *pCurve = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    {
        gw_INDEX m = pCurve->knt->m;
        gw_REAL *U = pCurve->knt->U;
        sNaturalInterval.SetMinMax(U[0],U[m]);
        if (   !sNaturalInterval.ContainsValue(dParameter)
            && !IsOutOfBoundsEnabled())  // GWC:MODIFIED OUTOFBOUNDS
          {
            dParameter = sNaturalInterval.ClampValue(dParameter);        
          }
    }
       
    // Compute curvature = cross(Ct,Ctt).Length() / Ct.Length() ** 3
    gw_PARAMETER u = dParameter;            
    gw_REAL    num, den; 
    NL_POINT   D[3];      // D[0] = C (postion), D[1] = Ct, D[2] = Ctt
    NL_POINT  B;
    sm_CrvDerivs(pCurve, u, TRUE, 2, D);  
    N_VectorCross(D[1], D[2], &B);
    N_VectorMagnitude(B, &num);
    N_VectorMagnitude(D[1], &den);
    den = den*den*den;

    if( N_FloatOpIsBad(num,den,NL_DIVISION) ) rCurvature=0.0 ;  
    else rCurvature = num/den;

    return SM_SUCCESS;

} // end SmBSplineCurve::EvaluatePoint

/*******************************************************************//**
PURPOSE: This method scales the tangents to produce an optimal blend
    in terms of the radius of curvature being largest over the curve and
    varying the least.  

NOTES: A cool trick performed by making the control polygon
    points equal distance from one another.
***********************************************************************/
SmStatus SmBSplineCurve::FairBlendCurveDerivatives
  (SmTArray<SmPoint3d>  & rPoints,               // in : [1st Curve EndPoint, 2nd Curve EndPoint]
   SmTArray<SmVector3d> & rTangents,             // i/o: [1st Curve End 1stDeriv, 2nd Curve 1stDeriv]
   SmTArray<SmVector3d> * pOptHigherOrderDerivs, // i/o: Optional [1st Curve End (i+2)th Deriv, 2nd Curve (i+2)th Deriv] pairs, NULL to ignore
   SmBoolean              bPreventInflection)    // in : 
{
  // A static function

  // check input
  if (rPoints.GetSize()   != 2) SER(SM_ERR);
  if (rTangents.GetSize() != 2) SER(SM_ERR);

  // locals
  double dScale1 = 1.0;
  double dScale2 = 1.0;

  // Let's compute an optimal scale to get fairest curve with equally spaced control points.
  // Find tangent length that would make hermite equally spaced.
  // Solve for two lines parameterized by T where ||L1(t)-L2(t)|| = t;
  // t^2 = (P1+tV1 - P2-tV2)
  // t^2 = (a + tb)^2 where a = P1-P2 and b = V1-V2
  // 0 = t^2 (b^2-1) + t (2ab) + a^2 
  SmVector3d a   = rPoints[1]-rPoints[0];
  SmVector3d sV1 = -rTangents[1];
  SER(sV1.Unitize());
  SmVector3d sV0 = rTangents[0];
  SER(sV0.Unitize());
  SmVector3d b   = sV1-sV0;
    
  // Set up quadratic equation coefficients.
  double coeff[3];
  coeff[2] = b.Dot(b) - 1;   // A of quadratic equation
  coeff[1] = 2.0 * a.Dot(b); // B
  coeff[0] = a.Dot(a);       // C 
    
  // Solve for t using quadratic equation.
  ULONG nSol;
  double dSol[2];
  SmPolynomial::SolveQuadraticEqn(coeff,SM_EFF_ZERO,nSol,dSol);
  double dT = 0.0;
  if (nSol == 1) { dT = 3.0 * dSol[0]; }
  if (nSol == 2) { if      (dSol[0] < 0.0) { dT = 3.0 * dSol[1]; }
                   else if (dSol[1] < 0.0) { dT = 3.0 * dSol[0]; }
                   else                    { dT = 3.0 * smos_Min(dSol[0],dSol[1]); }
                }
    
  if (dT > 0.0) { dScale1 = dT/rTangents[0].Length();
                  dScale2 = dT/rTangents[1].Length();
                }

  if(bPreventInflection)
    {
      SmVector3d sZ        = rTangents[0] * rTangents[1];
      SmVector3d sNormal1  = sZ * rTangents[1];
      SmVector3d sVec      = rPoints[0] - rPoints[1];
      SER(sNormal1.Unitize());
      SER(sVec.Unitize());
      double     dDot1     = sVec.Dot(sNormal1);
      SmPoint3d  sCtrlPnt1 = rPoints[0] + rTangents[0]*dScale1 / 3.0;
      SmVector3d sLineVec  = sCtrlPnt1 - rPoints[0];
      SmVector3d sVec1     = sCtrlPnt1 - rPoints[1];
      double     dDot2     = sVec1.Dot(sNormal1);
      double     dT1       = 1.0;
      double     dT2       = 1.0;

      if (dDot1 * dDot2 < 0.0) 
        {
          SER(smgu_LinePlaneIntersect(rPoints[0],
                                      sLineVec,
                                      rPoints[1],
                                      sNormal1,
                                      dT1));
          if (dT1 < 0.03)             { dT1 = 1.0; }
          if (dT1 < 0.0 || dT1 > 1.0) { SE(SM_ERR); }
        }
        
      SmVector3d sNormal2  = sZ * rTangents[0];
      SER(sNormal2.Unitize());
      sVec = -sVec;
      double     dDot3     = sVec.Dot(sNormal2);
      SmPoint3d  sCtrlPnt2 = rPoints[1] - rTangents[1]*dScale2 / 3.0;
      SmVector3d sVec2     = sCtrlPnt2 - rPoints[0];
      double     dDot4     = sVec2.Dot(sNormal2);
      SmVector3d sLineVec2 = sCtrlPnt2 - rPoints[1];

      if (dDot3 * dDot4 < 0.0) 
        {
          SER(smgu_LinePlaneIntersect(rPoints[1],
                                      sLineVec2,
                                      rPoints[0],
                                      sNormal2,
                                      dT2));
          if (dT2 < 0.03) dT2 = 1.0;
          if (dT2 < 0.0 || dT2 > 1.0) { SE(SM_ERR); }
        }

      dT = smos_Min(dT1,dT2);
      dScale1   = dScale1*dT;
      dScale2   = dScale2*dT;
    }

  // set output - new Tangent values
  rTangents[0] = rTangents[0]*dScale1;
  rTangents[1] = rTangents[1]*dScale2;

  // If we don't have any higher order just quit with results for simple Hermite
  if(!pOptHigherOrderDerivs || pOptHigherOrderDerivs->GetSize() == 0) 
    { return SM_SUCCESS; }

  // set higher order outputs
  SmTArray<SmVector3d> & rHigherOrderDerivs = *pOptHigherOrderDerivs;

#ifdef SM_DEBUG_CODE
  ULONG lDegree = 3;
#endif

  if (rHigherOrderDerivs.GetSize() == 2) 
    {
#ifdef SM_DEBUG_CODE
      lDegree = 5;
#endif
      rHigherOrderDerivs[0] = rHigherOrderDerivs[0]*dScale1*dScale1;
      rHigherOrderDerivs[1] = rHigherOrderDerivs[1]*dScale2*dScale2;
    }
  if (rHigherOrderDerivs.GetSize() == 4) 
    {
#ifdef SM_DEBUG_CODE
      lDegree = 7;
#endif
      rHigherOrderDerivs[0] = rHigherOrderDerivs[0]*dScale1*dScale1;
      rHigherOrderDerivs[1] = rHigherOrderDerivs[1]*dScale2*dScale2;
      rHigherOrderDerivs[2] = rHigherOrderDerivs[2]*dScale1*dScale1*dScale1;
      rHigherOrderDerivs[3] = rHigherOrderDerivs[3]*dScale2*dScale2*dScale2;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmContext       sContext ;
      SmBSplineCurve *pProfile2;
      SER(SmBSplineCurve::CreateInterpolatingCurve(sContext,
                                                   SM_CP_UNIFORM,
                                                   3,lDegree,
                                                   rPoints,
                                                   rTangents,
                                                   &rHigherOrderDerivs,
                                                   FALSE,
                                                   pProfile2));
      NER(pProfile2);
      double dLength = pProfile2->ApproximateLength(pProfile2->GetNaturalInterval(),10);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; pProfile2->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; pProfile2->DrawPolygon(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; pProfile2->DrawCurvature(0.5*dLength,60); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::FairBlendCurveDerivatives

/*******************************************************************//**
PURPOSE: Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.


NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::FixupKnotVector
  (double dTolerance)
{
  gw_CURVE *pCur = GetOrCreateGwNurbPointer();
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

  // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
  //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
   sm_FixupKnotVector(pCur->knt,dTolerance);
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
  return SM_SUCCESS;

} // end SmBSplineCurve::FixupKnotVector

/*******************************************************************//**
PURPOSE: Return true when the knot multiplicity is incorrect

NOTES: At ends, the knot multiplicity greater than deg + 1 is not allowed.
       At interior, the knot multiplicity greater than degree is not allowed
       Requires future "healing" function to repair

***********************************************************************/
SmBoolean SmBSplineCurve::HasKnotMultiplicityGreaterThanDegree() const
{
    // locals
    gw_CURVE *pCur = GetGwNurbPointer();

    // no work - no BSplineCurve representation
    if(pCur == NULL)
        return FALSE ;

    // check for degeneracy
    SmBoolean bDegenerate = IsDegenerate() ;

    // no work - degenerate curve
    if(bDegenerate)
        return FALSE ;

    // control point array max index
    NL_INDEX iN = pCur->pol->n ;

    // no work - less than three control points
    if(iN < 2)
        return FALSE ;

    ULONG degree = (ULONG)pCur->p;

    SmTArray<double> sKnots;
    SmTArray<ULONG> sKnotMult;
    SER(sm_GetKnots( pCur->knt, sKnots, &sKnotMult, NULL) );

    for( ULONG ii = 0; ii < sKnotMult.GetSize(); ii++ ) {
        if( ii == 0 || ii == sKnotMult.GetSize() - 1 ) {
            if( sKnotMult[ii] > degree + 1 )
                return( TRUE );
        }
        else {
            if( sKnotMult[ii] > degree  )
                return( TRUE );
        }
    }
 

    // all done
    return ( FALSE ) ; 

} // end SmBSplineCurve::HasKnotMultiplicityGreaterThanDegree

/*******************************************************************//**
PURPOSE: Return true when the curve is nonDegenerate and has
  a repeated pair of control points at either end of the curve. 

NOTES: 
***********************************************************************/
SmBoolean SmBSplineCurve::HasRepeatedEndControlPoints
  (double dTol)     // in : min distance between distinct points, default:[SM_ZONE_TOL_3D=1.0e-5]
                    //      when less than or equal to 0.0 reset to SM_ZONE_TOL_3D
 const
{
  // locals
  gw_CURVE *pCur = GetGwNurbPointer();

  // no work - no BSplineCurve representation
  if(pCur == NULL)
    { return FALSE ; }

  // check for degeneracy
  SmBoolean bDegenerate = IsDegenerate() ;

  // no work - degenerate curve
  if(bDegenerate)
    { return FALSE ; }

  // control point array max index
  NL_INDEX iN = pCur->pol->n ;

  // no work - less than three control points
  if(iN < 2)
    { return FALSE ; }

  // check for start and end ControlPoint coincidence
  double dStartCPtDist, dEndCPtDist ;

  // pick tolerance
  double dTolerance = dTol > 0.0 ? dTol : SM_ZONE_TOL_3D ;

  //      double dStartScaledZero = SM_EFF_ZERO * (1.0 + ((SmVector3d *)(&pCur->pol->Pw[0]))->GetMaxDimension()) ; 
  //      double dEndScaledZero   = SM_EFF_ZERO * (1.0 + ((SmVector3d *)(&pCur->pol->Pw[iN]))->GetMaxDimension()) ;
  
  // spacing between CPts at either end of the curve 
  N_DistCptCpt( pCur->pol->Pw[0],      pCur->pol->Pw[1],  &dStartCPtDist) ;
  N_DistCptCpt( pCur->pol->Pw[iN - 1], pCur->pol->Pw[iN], &dEndCPtDist) ;

  // CPt pairs at either end of the curve are coincident checks
  SmBoolean bStartRepeat = SM_IS_ZERO_TO_TOL(dStartCPtDist, dTolerance) ;
  SmBoolean bEndRepeat   = SM_IS_ZERO_TO_TOL(dEndCPtDist,   dTolerance) ;

  // all done
  return ( bStartRepeat || bEndRepeat ) ; 

} // end SmBSplineCurve::HasRepeatedEndControlPoints

/*******************************************************************//**
PURPOSE: Return true when enough internal neighboring control
         points are coincident to create an internal pole 
    (i.e. has a subrange of neighboring internal DomainPoints that 
          map to a single ImageSpacePoint, 
     i.e. a singularity where all domain points within some internal subdomain 
          of the curve maps to a single 3d location.)

NOTES: Internal poles are not allowed in SMLib.  
       Some of the many problems caused by internal poles include
       1. 1st derivatives values run to zero causing (among other problems)
          NewtonRaphson based sample and walk algorithms to fail.
       2. curve normals and binormals within the domain region of the pole are not defined.
       3. for curves - increases the likelihood that the 3d curve will have a kink,
***********************************************************************/
SmBoolean SmBSplineCurve::HasInternalPole
  (double dTol)     // in : min distance between distinct points, default:[SM_ZONE_TOL_3D=1.0e-5]
                    //      when less than or equal to 0.0 reset to SM_ZONE_TOL_3D
 const
{
  // init return
  SmBoolean bRtn = FALSE ;

  // locals
  ULONG ii ;
  NL_INDEX jj ;
  gw_CURVE *pCur = GetGwNurbPointer();

  // no work - no BSplineCurve representation
  if(pCur == NULL)
    { return FALSE ; }

  // check for degeneracy
  SmBoolean bDegenerate = IsDegenerate(dTol) ;

  // no work - degenerate curve
  if(bDegenerate)
    { return FALSE ; }

  // control point array max index
  NL_DEGREE iDegree = pCur->p ;
  SmPoint3d sPt, sPtU ;
  SmBoolean bTest = TRUE ;
  SmTArray<ULONG>   sMults ;
  SmTArray<double>  sUniqueKnots ;
  GetKnots(sUniqueKnots, &sMults) ;
  ULONG lCPIndex, lKntCnt = 0 ; 
          
  // pick tolerance
  double dTolerance = dTol > 0.0 ? dTol : SM_ZONE_TOL_3D ;

  // test - any sequence of degree + 1 coincident test points
  //        will create a degenerate interval in which
  //        all interval DomainPoints lose their one-to-one mapping
  //        to image space by mapping to a single image point.

  // for every knot span - seek sets of coincident control points
  for(ii=0;ii+1<sUniqueKnots.GetSize() && bRtn == FALSE;ii++) // note: can't say sUniqueKnots.GetSize()-1
    {
      lKntCnt += sMults[ii] ;

      // get the controlPoint index for the 1st control point whose basis function
      // is nonZero over interval U = [sUniqueKnots[ii], sUniqueKnots[ii+1]]
      lCPIndex = (lKntCnt - 1) - iDegree ;

      TO_EUCLID(pCur->pol->Pw[lCPIndex], sPt) ;

      bTest = TRUE ;

      // for the next Degree number of neighbor control points
      for(jj=1;jj<=iDegree && bTest == TRUE;jj++)
        {
          // ensure points in euclidean space
          TO_EUCLID(pCur->pol->Pw[lCPIndex+jj], sPtU) ;
          bTest &= sPt.CloserThan(dTolerance, sPtU) ;
         }
      
      // remember when ivl is degenerate
      bRtn |= bTest ;

    } // end iter ii, every control point in surface

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return bRtn ;

} // end SmBSplineCurve::HasInternalPole

/*******************************************************************//**
PURPOSE: Separate duplicate end control points on nonDegenerate curves,
   returns TRUE when a curve has been modified.  

NOTES: Used to heal one kind of bad BSplineCurve data.
  If a nonDegenerate curve has a duplicate pair of control points
  at either of its ends, it moves the 2nd control point 
  between the 1st and 3rd control points.
  The amount of the move is set by the parameter, SM_FR_NORMALIZED_PARAM
    which is used by both the curve and surface version of this function
    so that their behaviors will always match.
  This modifies the parameterization of the curve some, but preserves
  the shape of the control polygon.

***********************************************************************/
SmBoolean SmBSplineCurve::FixRepeatedEndControlPoints()
{
  // locals
  gw_CURVE *pCur = GetGwNurbPointer();

  // no work - no BSplineCurve representation
  if(pCur == NULL)
    { return FALSE ; }

  // check for degeneracy
  SmBoolean bDegenerate = IsDegenerate() ;

  // no work - degenerate curve
  if(bDegenerate)
    { return FALSE ; }

  // control point array max index
  NL_INDEX iN = pCur->pol->n ;

  // no work - less than three control points
  if(iN < 2)
    { return FALSE ; }

  // check for start and end ControlPoint coincidence
  double dStartCPtDist, dEndCPtDist ;

  double dStartScaledZero = SM_EFF_ZERO * (1.0 + ((SmVector3d *)(&pCur->pol->Pw[0]))->GetMaxDimension()) ; 
  double dEndScaledZero   = SM_EFF_ZERO * (1.0 + ((SmVector3d *)(&pCur->pol->Pw[iN]))->GetMaxDimension()) ;
   
  N_DistCptCpt( pCur->pol->Pw[0],      pCur->pol->Pw[1],  &dStartCPtDist) ;
  N_DistCptCpt( pCur->pol->Pw[iN - 1], pCur->pol->Pw[iN], &dEndCPtDist) ;

  SmBoolean bStartRepeat = SM_IS_ZERO_TO_TOL(dStartCPtDist, dStartScaledZero) ;
  SmBoolean bEndRepeat   = SM_IS_ZERO_TO_TOL(dEndCPtDist,   dEndScaledZero) ;

  // fix repeated control points
  if(bStartRepeat || bEndRepeat)
    {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      // draw before
      if(bDebugMe)
        {
          this->Dump() ;

          SmEdge *pEdge = (SmEdge *)this->GetEdge() ;
          SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

          smgfx_Erase() ;
          smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
          smgfx_SetLook(3,4, 0,1,0) ; this->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook( 5, 6, 0, 1, 1 ); if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // let the system know the curve is about to be edited
      Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

      // start control point repeat
      if(bStartRepeat)
        {
          // move control point 1 midway between control points 0 and 2
          N_Combine2CPts( SM_FR_NORMALIZED_PARAM,      pCur->pol->Pw[0], 
                          (1-SM_FR_NORMALIZED_PARAM),  pCur->pol->Pw[2], 
                          &(pCur->pol->Pw[1]) );
        }

      // end control point repeat
      if(bEndRepeat)
        {
          // move control point 1 midway between control points 0 and 2
          N_Combine2CPts((1-SM_FR_NORMALIZED_PARAM), pCur->pol->Pw[iN - 2], 
                          SM_FR_NORMALIZED_PARAM,    pCur->pol->Pw[iN], 
                          &(pCur->pol->Pw[iN - 1]) );
        }

      // inform the public - all done with changes
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
#ifdef SM_DEBUG_CODE
      // draw after
      if(bDebugMe)
        {
          this->Dump() ;

          smgfx_SetLook(3,4, 1,0,0) ; this->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end bRepeatStart || bRepeatEnd check

  // all done
  return ( bStartRepeat || bEndRepeat ) ; 

} // end SmBSplineCurve::FixRepeatedEndControlPoints

/*******************************************************************//**
PURPOSE: Get the control polygon in cartesian coordinates
           and corresponding weights.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::GetControlPolygon
  (SmTArray<SmPoint3d> & rControlPointsList,  // out: control points in cartesian coordinates
   SmTArray<double>    & rWeights)            // out: associated weights for Rational Curves
  const                                       //      rWeights.GetSize() == 0 for non-rational curves
{
  // locals
  gw_CURVE    *pCur        = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  gw_CPOLYGON *pCPol       = pCur->pol;
  gw_CPOINT   *Pw          = pCPol->Pw;
  SmBoolean    bIsRational = FALSE;

  // init output
  rControlPointsList.SetSize(pCPol->n + 1);
  rWeights.SetSize(pCPol->n + 1);

  // For every Control Point
  for (long i=0; i<=pCPol->n; i++) 
    {
      SmPoint3d sPnt;
      sPnt.x =  Pw[i].x;
      sPnt.y =  Pw[i].y;
      sPnt.z = (Pw[i].z == NL_NOZ) ? 0.0 : Pw[i].z ;

      // convert from homogeneous to cartesian coordinates
      if (Pw[i].w != NL_NOW)
      {
          bIsRational = TRUE;
          rWeights[i] = Pw[i].w;
          double wReciprocal = 1.0 / Pw[i].w;
          sPnt.x = sPnt.x * wReciprocal; // / Pw[i].w;
          sPnt.y = sPnt.y * wReciprocal; // / Pw[i].w;
          sPnt.z = sPnt.z * wReciprocal; // / Pw[i].w;
      }

      // set output
      rControlPointsList[i] = sPnt;

    } // end iter every ControlPoint
     
  if (!bIsRational) rWeights.ReSet();

  return SM_SUCCESS;

} // end SmBSplineCurve::GetControlPolygon

/*******************************************************************//**
PURPOSE: Get pointer to controlPoint array. 

NOTES: Return pointer to double array of control point
                values stored as [x, y, z, w]

WARNING ---  When a controlPoint value is modified
  internal caches become out of date.  Remember to call 
  Notify(SM_NO_PRE_EDIT,  this, SM_NO_GET_OWNER(this), NULL) and 
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL).
***********************************************************************/
SmStatus SmBSplineCurve::GetControlPointsPointer
  (ULONG   &lControlPointCount,    // out: number of control points
   double *&pControlPoints)        // out: array of controlPoints stored as doubles
                                   //      [ x0, y0, z0, w0, x1.. ]
                                   //      For 2d control Point   z == NL_NOZ or 0.0
                                   //      for nonRational points w == NL_NOW
                                   //      for Rational points x,y,z are stored in homogeneous space
                                   //        i.e. CartesianX = x/w
                                   //             CartesianY = y/w
                                   //             CartesianZ = (z != NL_NOZ) ? z/w : NL_NOZ ;
 const
{
  // locals
  gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();

  // set output
  lControlPointCount = pCur->pol->n + 1;
  pControlPoints     = (double *)pCur->pol->Pw ;

  // all done
  return(SM_SUCCESS) ;

} // end SmBSplineCurve::GetControlPointsPointer 

/*******************************************************************//**
PURPOSE: Get the STEP Part 42 based canonical data for the 
    B-Spline curve.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the curve is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.
***********************************************************************/
SmStatus SmBSplineCurve::GetCanonical
  (ULONG               & rlDimension,           // out: Image Space Dimension Size      
   ULONG               & rlDegree,              // out: Curve polynomial degree
   SmTArray<SmPoint3d> & rControlPointsList,    // out: Euclidian form of the control points.
   SmBSplineCurveForm  & reBSplineCurveForm,    // oneof: SM_CF_POLYLINE_FORM, 
                                                //        SM_CF_CIRCULAR_ARC,  
                                                //        SM_CF_ELLIPTIC_ARC,  
                                                //        SM_CF_PARABOLIC_ARC, 
                                                //        SM_CF_HYPERBOLIC_ARC,
                                                //        SM_CF_HELICAL_ARC,
                                                //        SM_CF_UNSPECIFIED    
   SmTArray<ULONG>     & rKnotMultiplicities,   // out: multiplicity value for each knot
   SmTArray<double>    & rUniqueKnots,          // out: Unique knot vector              
   SmKnotType          & reKnotType,            // out: oneof:
                                                // oneof: SM_KT_UNIFORM_KNOTS,             
                                                //        SM_KT_UNSPECIFIED,          
                                                //        SM_KT_QUASI_UNIFORM_KNOTS,  
                                                //        SM_KT_PIECEWISE_BEZIER_KNOTS
   SmTArray<double>    & rWeights)              // out: associated weight for every ControlPoint,
                                                //      rWeigths.GetSize() == 0 for non-rational curves
  const
{
  rlDimension = GetDim();
  reBSplineCurveForm = m_eBSplineCurveForm;
  reKnotType = m_eKnotType;

  gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();

  rlDegree = pCur->p;

  SER(GetKnots(rUniqueKnots,&rKnotMultiplicities))

  SER(GetControlPolygon(rControlPointsList,rWeights));

  return SM_SUCCESS;

} // end SmBSplineCurve::GetCanonical

/*******************************************************************//**
PURPOSE: Get the degree of a BSpline.

NOTES: 
***********************************************************************/
ULONG SmBSplineCurve::GetDegree
  () 
 const 
{ 
    gw_CURVE *pCurve = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    return pCurve->p; 

} // end SmBSplineCurve::GetDegree

/*******************************************************************//**
PURPOSE: Get the natural endPoints and endWeights of a BSpline curve.  

NOTES: This function short cuts evaluation by taking first
    and last point of the control polygon because we only deal with 
    clamped nurbs.  Note this will be much faster then evaluation.
***********************************************************************/
void SmBSplineCurve::GetEnds
  (SmPoint3d & rStartPoint,   // out: Curve StartPoint
   SmPoint3d & rEndPoint,     // out: Curve EndPoint
   double    * pdStartW,      // out: Curve StartWeight
   double    * pdEndW)        // out: Cruve EndWeight
  const
{
  // get curve
  gw_CURVE *pCurve = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();

  // set output
  TO_EUCLID(pCurve->pol->Pw[0], rStartPoint);
  TO_EUCLID(pCurve->pol->Pw[pCurve->pol->n], rEndPoint);

  if (pdStartW) 
    { *pdStartW = (pCurve->pol->Pw[0].w == NL_NOW) ? 1.0 : pCurve->pol->Pw[0].w ; }
  if (pdEndW) 
    { *pdEndW = (pCurve->pol->Pw[pCurve->pol->n].w == NL_NOW) ? 1.0 : pCurve->pol->Pw[pCurve->pol->n].w ; }

} // end SmBSplineCurve::GetEnds

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities 
     of a BSpline curve.

NOTES: This is the STEP compatible form of the knots not the
    NLIB knot vector associated with NURBS.
***********************************************************************/
SmStatus SmBSplineCurve::GetKnots
  (SmTArray<double> & rUniqueKnots,         // out: Unique knot vector                     
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: multiplicity value for each knot, NULL to ignore, default:[NULL]
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
  const
{
  gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  SER(sm_GetKnots(pCur->knt,rUniqueKnots,pKnotMultiplicities,pOptIvl));
  return SM_SUCCESS;

} // end SmBSplineCurve::GetKnots

/*******************************************************************//**
PURPOSE: Get list of all knots of a BSpline curve.

NOTES: This is the normal NLIB knot vector associated with NURBS.
***********************************************************************/
SmStatus SmBSplineCurve::GetKnotsAll
  ( SmTArray<double> & rKnots )
  const
{
  gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  SER( sm_GetKnotsAll( pCur->knt, rKnots ));
  return SM_SUCCESS;

} // end SmBSplineCurve::GetKnots

/*******************************************************************//**
PURPOSE: Get the number of natural knots.  

NOTES:  The number of natural knots
    is the number of knots if the knots are represented as a single array
    with duplicated knot values.
***********************************************************************/
ULONG SmBSplineCurve::GetNumberNaturalKnots
  () 
 const
{
  gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  return pCur->knt->m + 1;

} // end SmBSplineCurve::GetNumberNaturalKnots

/*******************************************************************//**
PURPOSE: Get the number of control points in the control polygon for
    this curve.

NOTES:  NOT the index, but the count
***********************************************************************/
ULONG SmBSplineCurve::GetNumberControlPoints
  () 
 const
{
    gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    return pCur->pol->n + 1;

} // end SmBSplineCurve::GetNumberControlPoints

/*******************************************************************//**
PURPOSE: Return the Greville abscissa for a given control point.

NOTES: 
   The Greville abscissae are the parameter values that generally
   correspond to the control points.
***********************************************************************/
double SmBSplineCurve::GetGrevilleAbscissa( ULONG lCtrlPointIndex ) const
{
  // locals
  if ( m_pNurb == NULL )
    {
      SmBSplineCurve *pNonConstThis = SM_CONST_CAST( SmBSplineCurve *, this );
      pNonConstThis->MakeNurb();
    }
  SM_ASSERT(m_pNurb != NULL);
  gw_CURVE *pCur = GetGwNurbPointer();

  ULONG lNumPts = pCur->pol->n + 1;
  ULONG lNumKts = pCur->knt->m + 1;
  double * pdKnots = pCur->knt->U;
  ULONG lDeg    = pCur->p;
  
  SM_ASSERT( lNumKts == lNumPts + lDeg + 1 );

  if ( lCtrlPointIndex < 1 )
    { return pdKnots[0]; }         // at or below the beginning

  if ( lCtrlPointIndex >= lNumPts-1 )
    { return pdKnots[lNumKts-1]; } // at or beyond the end

  ULONG i;
  double dRetVal = 0;

  for ( i = 1; i <= lDeg; i++ )
    {
      dRetVal += pdKnots[ lCtrlPointIndex + i ];
    }
  dRetVal /= lDeg;

  return dRetVal;

} // end GetGrevilleAbscissa



/*******************************************************************//**
PURPOSE: Return all of the Greville abscissae for a curve.

NOTES: 
   The Greville abscissae are the parameter values that generally
   correspond to the control points.
***********************************************************************/
SmStatus SmBSplineCurve::GetGrevilleAbscissae( SmTArray< double > &vGrevilles ) const
{
  // locals
  if ( m_pNurb == NULL )
    {
      SmBSplineCurve *pNonConstThis = SM_CONST_CAST( SmBSplineCurve *, this );
      pNonConstThis->MakeNurb();
    }
  NER( m_pNurb );

  gw_CURVE *pCur = GetGwNurbPointer();

  ULONG lNumPts = pCur->pol->n + 1;
  double * pdKnots = pCur->knt->U;
  ULONG lDeg    = pCur->p;
  
#ifdef SM_DEBUG_CODE
  ULONG lNumKts = pCur->knt->m + 1;
  SM_ASSERT( lNumKts == lNumPts + lDeg + 1 );
#endif // SM_DEBUG_CODE

  ULONG i, lCtrlPtIdx;
  double dThisVal;

  vGrevilles.ReSet();

  for ( lCtrlPtIdx = 0; lCtrlPtIdx < lNumPts; lCtrlPtIdx++ )
    {
      dThisVal = 0;
      for ( i = 1; i <= lDeg; i++ )
      {
          dThisVal += pdKnots[ lCtrlPtIdx + i ];
      }

      dThisVal /= lDeg;

      vGrevilles.Add( dThisVal );
    }

  return SM_SUCCESS;

} // end GetGrevilleAbscissae
 
/*******************************************************************//**
PURPOSE: Determine if the bspline curve has null segments.

NOTES: 
***********************************************************************/
SmBoolean SmBSplineCurve::HasNullSegments
  () 
 const
{
    double dData[256];
    SmTArray<double> sKnots(256,dData);
    GetKnots(sKnots);
    SmVector3d sPV[2];
    ULONG  i;
    for (i=0; i<sKnots.GetSize(); i++) {
        SER(Evaluate(sKnots[i],1,TRUE,sPV));
        if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) {
            return TRUE;
        }
        SER(Evaluate(sKnots[i],1,FALSE,sPV));
        if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) {
            return TRUE;
        }
    }
    return FALSE;

} // end SmBSplineCurve::HasNullSegments

/*******************************************************************//**
PURPOSE: Return TRUE when curve has the same analytic domain
            and knot vector as the given curve.

NOTES:
***********************************************************************/
SmBoolean SmBSplineCurve::HasSameParameterization
  (const SmBSplineCurve *cpOtherCurve)    // in : target curve
  const 
{
  const SmBSplineCurve *cpThisCurve = this ;

  // analytic parameterization locals
  SmExtent1d sAnalDomain1 = cpThisCurve->GetSTEPInterval() ;
  SmExtent1d sAnalDomain2 = cpOtherCurve->GetSTEPInterval() ;

  // Nurb parameterization locals
  SmTArray<double> sKnots1, sKnots2 ;           
  SmTArray<ULONG>  sMults1, sMults2 ;
  cpThisCurve ->GetKnots(sKnots1, &sMults1) ;  
  cpOtherCurve->GetKnots(sKnots2, &sMults2) ;

  // The analytic domains must be compatible
  SmBoolean bRtn = (   sAnalDomain1.IsContainedBy(sAnalDomain2, SM_EFF_ZERO)
                    && sAnalDomain2.IsContainedBy(sAnalDomain1, SM_EFF_ZERO)) ;

  // The knot vectors must be the same
  ULONG ii ;
  ULONG lCount1 = sKnots1.GetSize() ;
  ULONG lCount2 = sKnots2.GetSize() ;
  if(lCount1 != lCount2) { bRtn = FALSE ; }
  
  // for every knot and multiplicity
  for(ii=0;ii<lCount1 && bRtn;ii++) 
    {
      // check compatibility
      bRtn &= SM_ARE_SAME(sKnots1[ii], sKnots2[ii]) ;
      bRtn &= sMults1[ii] == sMults2[ii] ;

    } // end iter every knot checking for compatibility

  // all done
  return(bRtn) ;

} // end SmBSplineCurve::HasSameParameterization

/*******************************************************************//**
PURPOSE: Get the null segments of this curve.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::GetNullSegments
  (SmTArray<double> & rSegments) // out: array of param pairs whose ends map to zero length 1st derivatives
                                 //      ex:[p0 p1 p2 p3] intervals [p0 p1] and [p2 p3] are null length in 3 space
 const
{
  rSegments.ReSet();

  double dData[256];
  SmTArray<double> sKnots(256,dData);
  GetKnots(sKnots);
  SmVector3d sPV[2];
  for (ULONG i=1; i<sKnots.GetSize(); i++) 
    {
      SER(Evaluate(sKnots[i-1],1,TRUE,sPV));
      if (sPV[1].LengthSquared() > SM_EFF_ZERO_SQ) 
        { continue; }

      SER(Evaluate(sKnots[i],1,FALSE,sPV));

      // save NULL length intervals
      if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
        {
          rSegments.Add(sKnots[i-1]);
          rSegments.Add(sKnots[i]);
        }
    }
  return SM_SUCCESS;

} // end SmBSplineCurve::GetNullSegments

/*******************************************************************//**
PURPOSE: Insert a given number of knots.

NOTES: 
   - Never creates any multiple knots.
   - Inserts between existing knots, as evenly as possible.
   - If the optional interval is given:
     - Inserts all knots within that interval;
     - Inserts the interval's end points, if they are not
       already at knot values.
     - Assumes that that interval is contained in the natural domain,
       but does not check.
***********************************************************************/
SmStatus SmBSplineCurve::InsertKnots(
      int             iNumToInsert,
      SmExtent1d    * pOptInterval
  )
{
  SmTArray< double > sAllKnots;
  this->GetKnots( sAllKnots );

  SmExtent1d sSubInterval;
  if ( pOptInterval != NULL )
  {
      sSubInterval = *pOptInterval;
  }
  else
  {
      sSubInterval = this->GetNaturalInterval();
  }

  SmTArray< double > sKnotsToInsert;

  SmStatus eStat = smgu_InsertKnotsIntoKnotVector( iNumToInsert,
         sAllKnots, sSubInterval, sKnotsToInsert );

  if ( eStat != SM_SUCCESS ) {
      return eStat;
  }

  // Got the knots, now do the insertion.
  return this->InsertKnots( sKnotsToInsert );

} // end SmBSplineCurve::InsertKnots( count, where )


///*******************************************************************//**
//PURPOSE: Check if 'THIS' curve is an ARC by testing a number of
//    sampled points. 
//
//NOTES: return 'TRUE' for arcs
//                with ReferenceFrame (CenterPoint, XAxis, YAxis) 
//                with StartAngle = 0.0  (measured CCW from XAxis about ZAxis)
//                and  EndAngle   = arc angle in degrees of the curve.
//Example:
//  User will provide (1) How many test points are needed, and
//                    (2) a tolerance for point-testing.
//***********************************************************************/
//SmBoolean SmBSplineCurve::IsArc
//  (ULONG lNumberOfSamplePoints,        // in : number of points to sample and test
//   double dTol3d,                      // in : max deviation from exact Arc allowed each samplePoint
//   SmAxis2Placement & rReferenceFrame, // out: orientation for found arc,
//                                       //      XAxis = centerPoint to StartPoint of curve's interval
//                                       //      YAxis = perp to XAxis in direction of Pt on Curve at .15 of interval
//   double & rdRadius,                  // out: Found Arc radius
//   double & rdStartAngDeg,             // out: Start angle in degrees
//                                       //      relative to a counter clockwise angle about Z from 
//                                       //      the X axis of reference frame.
//   double & rdEndAngDeg)               // out: End angle in degrees
//                                       //      relative to a counter clockwise angle about Z from 
//                                       //      the X axis of the reference frame.
// const
//{
//  // no work - degenerate curve
//  //  curveStart == CurveEnd and start, mid, end Point 1st derivatives are all zero length
//  if (IsDegenerate(dTol3d)) { return FALSE; }
//
//  // get 3 samplePoints [.00, .45, .90] from the curve
//  // Don't use [0, .5, 1] in case of closed curve.
//  // (Note, used to be 0, .15, .30, but bigger span gives much better basis.)
//
//  SmExtent1d sInterval = this->GetNaturalInterval();
//  SmPoint3d sP1, sP2, sP3;
//  EvaluatePoint( sInterval.Evaluate(0.00), sP1 );
//  EvaluatePoint( sInterval.Evaluate(0.45), sP2 );
//  EvaluatePoint( sInterval.Evaluate(0.90), sP3 );
//
//#ifdef SM_DEBUG_CODE
//  // draw
//SmBoolean bDebugMe = FALSE ; 
//  if(bDebugMe)
//    {
//      Dump() ;
//      SmEdge *pEdge = (SmEdge *)GetEdge() ;
//      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;
//
//      smgfx_Erase() ;
//      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//      smgfx_SetLook(5,6, 1,0,0) ; sP1.Draw() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(5,6, 1,0,0) ; sP2.Draw() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(5,6, 1,0,0) ; sP3.Draw() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(3,4, 0,1,0) ; Draw() ;       sm_GraphicsLoop() ;
//      smgfx_SetLook(3,4, 0,1,0) ; DrawParams() ; sm_GraphicsLoop() ;
//      sm_GraphicsLoop() ;
//    }
//#endif // SM_DEBUG_CODE
//
//  // get vectors between sample points
//  SmVector3d sV1 = sP2 - sP1;
//  SmVector3d sV2 = sP3 - sP1;
//  double dApproxLengthOfArc = sP1.DistanceBetween(sP2) + sP2.DistanceBetween(sP3);
//
//  SmVector3d sX, sY, sZ;
//
//  // no work - sample points have no separation (probably a degenerate curve)
//  if (sV1.LengthSquared() < SM_EFF_ZERO_SQ) { return FALSE ; }
//
//  // create coordinate system: xAxis = sV1, XYPlane = sV1/sV2 plane
//  if (sV1.MakeUnitOrthoVectors(&sV2,sX,sY,sZ) != SM_SUCCESS) 
//    {
//      // fail when sV1/sV2 don't specify a unique coordinate system
//      return FALSE;
//    }
//
//  // place coordinate system in SmAxis2Placement object
//  SmAxis2Placement sPlane;
//  sPlane.SetCanonical(sP1,sX,sY);
//
//  // drop 3 points to the plane (they should already be on the plane to tolerances)
//  SmPoint2d sUVP1 = smgu_PlaneDropPoint(sPlane,sP1);
//  SmPoint2d sUVP2 = smgu_PlaneDropPoint(sPlane,sP2);
//  SmPoint2d sUVP3 = smgu_PlaneDropPoint(sPlane,sP3);
//
//
//#ifdef SM_DEBUG_CODE
//  // check tolerance of drop points
//  SmPoint3d sCheck1 = smgu_PlaneEvaluatePoint(sPlane, sUVP1) ;
//  SmPoint3d sCheck2 = smgu_PlaneEvaluatePoint(sPlane, sUVP2) ;
//  SmPoint3d sCheck3 = smgu_PlaneEvaluatePoint(sPlane, sUVP3) ;
//  double dLength1 = (sCheck1 - sP1).Length() ;
//  double dLength2 = (sCheck2 - sP2).Length() ;
//  double dLength3 = (sCheck3 - sP3).Length() ;
//  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_3Max(sCheck1.GetMaxDimension(),
//                                                      sCheck2.GetMaxDimension(),
//                                                      sCheck3.GetMaxDimension())) ;
//  SM_ASSERT(   dLength1 < dScaledZero
//            && dLength2 < dScaledZero
//            && dLength3 < dScaledZero) ;
//#endif // SM_DEBUG_CODE
//
//  // get circle center from 3Points
//  SmPoint2d sUVCircleCenter;
//  if (smgu_CircleCenterFrom3Points(sUVP1,sUVP2,sUVP3,sUVCircleCenter) != SM_SUCCESS) 
//    { return FALSE; }
//
//  SmPoint3d sCenter = smgu_PlaneEvaluatePoint(sPlane,sUVCircleCenter);
//
//  // get circle radius
//  double dRadius = sCenter.DistanceBetween(sP1);
//
//  // failure - radius is too large
//  if (dRadius > dApproxLengthOfArc / SM_EFF_ZERO_SQRT) 
//    { // Size of radius too big to make an accurate arc.
//      return FALSE;
//    }
//
//  // Now test each point to see if it is correct distance from 
//  // center of circle and on plane.
//  double dAveRadius = dRadius;
//  for (ULONG i=1; i<lNumberOfSamplePoints; i++) 
//    {
//      // get next samplePoint param value
//      double dParam = (i==lNumberOfSamplePoints-1)
//                      ? sInterval.GetMax() 
//                      : sInterval.Evaluate((double)i/(lNumberOfSamplePoints-1.0));
//
//      // get next samplePoint
//      SmPoint3d sPnt;
//      if (this->EvaluatePoint(dParam,sPnt) != SM_SUCCESS) 
//        { return FALSE; }
//
//#ifdef SM_DEBUG_CODE
//      if (bDebugMe) 
//        {
//          Dump() ;
//          SmEdge *pEdge = (SmEdge *)GetEdge() ;
//          SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;
//
//          smgfx_Erase() ;
//          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//          smgfx_SetLook(5,6, 1,1,0) ; sP1.Draw() ; sm_GraphicsLoop() ;
//          smgfx_SetLook(5,6, 1,1,0) ; sP2.Draw() ; sm_GraphicsLoop() ;
//          smgfx_SetLook(5,6, 1,1,0) ; sP3.Draw() ; sm_GraphicsLoop() ;
//          smgfx_SetLook(3,4, 0,1,0) ; Draw() ;       sm_GraphicsLoop() ;
//          smgfx_SetLook(3,4, 0,1,0) ; DrawParams() ; sm_GraphicsLoop() ;
//          smgfx_SetLook(7,10, 1,0,0) ; sPnt.Draw() ; sm_GraphicsLoop() ;
//          sm_GraphicsLoop() ;
//        }
//#endif
//      // get samplePoint/CircleCenter distance
//      double dTestRadius = sPnt.DistanceBetween(sCenter);
//
//      // failure - dist is not close to current radius value
//      if (smos_Fabs(dTestRadius-dRadius) > dTol3d) 
//        { return FALSE; }
//
//      dAveRadius += dTestRadius;
//
//      // get dist to plane
//      double dDistToPlane;
//      if (smgu_PlanePointDistance(sCenter,sZ,sPnt,dDistToPlane) != SM_SUCCESS) 
//        { return FALSE; }
//
//      // failure - samplePoint is too far from circle plane
//      if (dDistToPlane > dTol3d) 
//        { // Not in plane
//          return FALSE; 
//        } 
//    } // end iter every samplePoint
//
//  // get average radius size
//  rdRadius = dAveRadius / lNumberOfSamplePoints;
//
//  // If we made it to here then we have a circle.
//  SmVector3d sCenterToP1 = sP1-sCenter;
//  SmVector3d sCenterToP2 = sP2-sCenter;
//
//  // create final coordinate system - xAxis = CenterPoint to SP1
//  //                                  yAxis = perp to xAxis in direction of sCenterToP2
//  if (sCenterToP1.MakeUnitOrthoVectors(&sCenterToP2,sX,sY,sZ) != SM_SUCCESS) 
//    { return FALSE; }
//
//  rReferenceFrame.SetCanonical(sCenter, sX, sY);
//
//  // arc start is 0.0
//  rdStartAngDeg = 0.0;
//
//  // get arc stop in degrees
//  SmPoint3d sEnd;
//  EvaluatePoint(sInterval.GetMax(),sEnd);
//  SmVector3d sVToEnd = sEnd - sCenter;
//  sZ.CCWAngleBetween(sX, sVToEnd, rdEndAngDeg);
//  rdEndAngDeg = rdEndAngDeg*180.0/SM_PI;
//
//  // snap near closed circles to closed circles and
//  // make interval positive
//  if (smos_Fabs(rdEndAngDeg) < SM_EFF_ZERO_SQRT)
//      rdEndAngDeg = 360.0;
//  else if (rdEndAngDeg < 0.0)
//      rdEndAngDeg += 360.0;
//
//#ifdef SM_DEBUG_CODE
//  // draw 
//  if(bDebugMe)
//    {
//      Dump() ;
//      SmEdge *pEdge = (SmEdge *)GetEdge() ;
//      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;
//
//      smgfx_Erase() ;
//      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//      smgfx_SetLook(5,6, 1,1,0) ; sP1.Draw() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(5,6, 1,1,0) ; sP2.Draw() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(5,6, 1,1,0) ; sP3.Draw() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(3,4, 0,1,0) ; Draw() ;       sm_GraphicsLoop() ;
//      smgfx_SetLook(3,4, 0,1,0) ; DrawParams() ; sm_GraphicsLoop() ;
//      smgfx_SetLook(1,2, 0,1,1) ; rReferenceFrame.Draw() ; sm_GraphicsLoop() ;
//      sm_GraphicsLoop() ;
//    }
//#endif // SM_DEBUG_CODE
//
//  // all done
//  return TRUE;    
//
//} // end SmBSplineCurve::IsArc

/*******************************************************************//**
PURPOSE: Check Curve for linearity and constant speed.

NOTES:
  - Returns True for any straight curve with a constant first derivative.
  - Returns False for a zero-length curve.
  - The vector returned is the whole line.

 Caller will provide (1) How many test points are needed, and
                     (2) a tolerance for testing tangent vector
METHOD ---
  Checks control points for special cases, else calls the parent's virtual method.

  Always identifies any BSplineCurve with just 2 control points as a line
***********************************************************************/
SmBoolean SmBSplineCurve::IsLine
  (ULONG        lNumSamples, // in : number of sample points to generate and test
   double       dTol,                  // in : max deviation allowed for any samplePoint from ideal line
   SmPoint3d  & rLinePoint,            // out: line's base point
   SmVector3d & rLineVector)           // out: line's direction vector
  const
{
    /*  
  if ( lNumSamples < 3 )
    { lNumSamples = 3; }

  // Too much to worry about with rational curves:
  if ( IsRational() )
    { return SmCurve::IsLine( lNumSamples, dTol, rLinePoint, rLineVector ); }

  SmTArray< SmPoint3d > sCPts;
  SmTArray< double    > sWts;
  this->GetControlPolygon( sCPts, sWts );
  ULONG lNumPts = sCPts.GetSize();

  // Return False for a zero-length line.
  // Also, if it's closed, it's not a line.
  // This will catch both of those:
  SmVector3d sBaseVec( sCPts[lNumPts-1] - sCPts[0] );
  if ( sBaseVec.LengthSquared() < dTol*dTol )
    {
      return FALSE; }

  // special case - BsplineCurve with two Control Point is always a line
  if ( lNumPts == 2 )
    {
      rLinePoint  = sCPts[0];
      rLineVector = sBaseVec;
      return TRUE;
    }

  // If this is a Bezier curve (no interior knots),
  // then it's constant speed iff the control points are evenly spaced.
  //  But: in practice, this ends up being overly stringent.  Because of
  //  the variation-diminishing property of B-splines, control points can
  //  be off by more than tol while the curve evaluations are actually
  //  within tol.  (On a simple cubic Bezier, I got control-point
  //  differences larger than evaluation differences by a factor of 4.5.)
  //  So the above 'if-and-only-if' is actually just an 'if'.
  //  Rather than trying to adjust tolerances to compensate, just defer to
  //  the base class method if we get a violation, instead of returning False.
  if ( GetDegree() == lNumPts-1 )
    {
      // All adjacent point diffs should match this:
      SmVector3d sStandardVec = sBaseVec / (double)( lNumPts-1 );

      ULONG ii;
      SmVector3d sTestVec, sErrorVec;
      for ( ii = 1; ii < lNumPts; ii++ )
        {
          sTestVec = sCPts[ii] - sCPts[ii-1];
          sErrorVec = sTestVec - sStandardVec;
          if ( sErrorVec.LengthSquared() > dTol*dTol )
            {
              //return FALSE;
              return SmCurve::IsLine( lNumSamples, dTol, rLinePoint, rLineVector );
            }
        }

      // Point diffs all match.
      rLinePoint  = sCPts[0];
      rLineVector = sBaseVec;
      return TRUE;
    }

  // No more optimizations.
  
  return SmCurve::IsLine( lNumSamples, dTol, rLinePoint, rLineVector );
    */
  
 //JLMCC switched from above to use pointers below
    
  if ( lNumSamples < 3 )
  { lNumSamples = 3; }

// Too much to worry about with rational curves:
if ( IsRational() )
  { return SmCurve::IsLine( lNumSamples, dTol, rLinePoint, rLineVector ); }

ULONG lNumPts = 0;
double* pCPts = nullptr;
this->GetControlPointsPointer( lNumPts, pCPts );


// Return False for a zero-length line.
// Also, if it's closed, it's not a line.
// This will catch both of those:
SmVector3d sEndVec(&pCPts[(lNumPts - 1) * 4], true);
SmVector3d sStartVec(&pCPts[0], true);
SmVector3d sBaseVec(sEndVec - sStartVec);

if ( sBaseVec.LengthSquared() < dTol * dTol )
  {
    return FALSE; }

// special case - BsplineCurve with two Control Point is always a line
if ( lNumPts == 2 )
  {
    rLinePoint  = sStartVec;
    rLineVector = sBaseVec;
    return TRUE;
  }

// If this is a Bezier curve (no interior knots),
// then it's constant speed iff the control points are evenly spaced.
//  But: in practice, this ends up being overly stringent.  Because of
//  the variation-diminishing property of B-splines, control points can
//  be off by more than tol while the curve evaluations are actually
//  within tol.  (On a simple cubic Bezier, I got control-point
//  differences larger than evaluation differences by a factor of 4.5.)
//  So the above 'if-and-only-if' is actually just an 'if'.
//  Rather than trying to adjust tolerances to compensate, just defer to
//  the base class method if we get a violation, instead of returning False.
if ( GetDegree() == lNumPts-1 )
  {
    // All adjacent point diffs should match this:
    SmVector3d sStandardVec = sBaseVec / (double)( lNumPts-1 );

    SmVector3d sTestVec, sErrorVec;
    SmVector3d sPreviousVec = sStartVec;
    for (ULONG  ii = 4; ii < lNumPts * 4; ii += 4 )
      {
        SmVector3d sCurrentVec(&pCPts[ii], true);
        sTestVec = sCurrentVec - sPreviousVec;
        sPreviousVec = sCurrentVec;
        sErrorVec = sTestVec - sStandardVec;
        if ( sErrorVec.LengthSquared() > dTol*dTol )
          {
            //return FALSE;
            return SmCurve::IsLine( lNumSamples, dTol, rLinePoint, rLineVector );
          }
      }

    // Point diffs all match.
    rLinePoint = sStartVec;
    rLineVector = sBaseVec;
    return TRUE;
  }

// No more optimizations.

return SmCurve::IsLine( lNumSamples, dTol, rLinePoint, rLineVector );


} // end SmBSplineCurve::IsLine

/*******************************************************************//**
PURPOSE: Check if 'THIS' curve is piecewise c0, that is 
  every internal knot has a multiplicity equal to degree,
  end knots have a multiplicity of degree+1, and
  the curve is non-rational.

  These kinds of curves are common because they are
  created by the Surface/Surface intersection and
  the DropCurve() algorithms.

NOTES: 
  The test is done completely on the knot and element counts.
  Returns TRUE when unique_knot_count == (total_knot_count - 2)/degree
***********************************************************************/
SmBoolean SmBSplineCurve::IsPiecewiseC0
  ( ) const
{
  // locals
  SmTArray<double> sKnots ;
  SmTArray<ULONG>  sMultiplicities ;

  // Query this Bspline
  ULONG lDegree = GetDegree() ;
  if(SM_SUCCESS != GetKnots(sKnots, &sMultiplicities)) 
    { 
      return FALSE ; 
    }
  //ULONG lTotalKnotCount  = sKnots.GetSize() ;
  ULONG lUniqueKnotCount = sMultiplicities.GetSize() ;

  // when every knot is of multiplicity degree (+2 on the ends)
  // and the curve is non-rational
  SmBoolean IsPiecewiseC0 =   (   IsRational()                        == FALSE
                               && sMultiplicities[0]                  == lDegree + 1
                               && sMultiplicities[lUniqueKnotCount-1] == lDegree + 1)
                            ? TRUE
                            : FALSE ;

  // Check all internal knot multiplicities equal lDegree
  if(IsPiecewiseC0 == TRUE)
    {
      // check the multiplicity of every internal knot
      for(ULONG ii=1;ii+1<sKnots.GetSize();ii++)   // note: can't say sKnots.GetSize()-1
        {
          if(lDegree != sMultiplicities[ii]) { IsPiecewiseC0 = FALSE ;
                                               break ;
                                             }
        } // end iter every internal knot
    } // end PiecewiseC0 check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {  
    // show this curve's representation
    Dump() ;
    sKnots.Dump() ;
    sMultiplicities.Dump() ;
  }
#endif

  // all done
  return(IsPiecewiseC0) ;

} // end SmBSplineCurve::IsPiecewiseC0

/*******************************************************************//**
PURPOSE: return true when curve bounding box's largest side is less than tol.

NOTES: tolerance set to Max(dScaledZero,d3DTol)
***********************************************************************/   
SmBoolean SmBSplineCurve::IsDegenerate
  (double      d3DTol,             // in : min distance between distinct points,
                                   //      default:[SM_EFF_ZERO]
   const SmExtent1d *pInterval)    // in : target interval to check, NULL = use NaturalInterval
                                   //      default:[NULL]
 const
{
  // low work - for nonNaturalInterval intervals pass the call 
  //  along to the general SmCurve::IsDegenerate
  if(   (   pInterval != NULL
         && !pInterval->AreEqual(GetNaturalInterval()))
     || IsKindOf(SmCrvOnSurf_TYPE))
    {
      return( SmCurve::IsDegenerate(d3DTol, pInterval) ) ;
    }

  // ensure existence of m_pNurb
  if(m_pNurb == NULL) 
    { ((SmBSplineCurve *)this)->MakeNurb(); } 
  SM_ASSERT(m_pNurb != NULL) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE

  // pass the call along
  return(sm_IsNCrvDegenerate(m_pNurb, d3DTol)) ;

} // end SmBSplineCurve::IsDegenerate


/*******************************************************************//**
PURPOSE: Determines if the curve is rational or not.

NOTES: 
***********************************************************************/   
SmBoolean SmBSplineCurve::IsRational
  () 
 const
{
    gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    if (pCur->pol->Pw[0].w == NL_NOW) {
        return FALSE;
    }
    return TRUE;

} // end SmBSplineCurve::IsRational

/*******************************************************************//**
PURPOSE: Check if the given segment of 'THIS' curve is a LINE by 
            testing a number of sampled points. 
            
NOTES: It will return 'TRUE' if it is a line.
Example:
  User will provide (1) How many test points are needed, and
                    (2) a tolerance for testing tangent vector
***********************************************************************/
SmBoolean SmBSplineCurve::IsSegmentLine
  (const SmExtent1d & crIntervalArg,          // in : param interval to query
   ULONG              lSamplePointCountArg,   // in : number of points to check
   double             dToleranceArg,          // in : min dist between distinct points
   SmPoint3d        & rLinePointArg,          // out: curve start point
   SmVector3d       & rLineVectorArg)         // out: curve unit chord when curve length > tol, else could be degenerate 
  const
{
  // curve end points
  SmPoint3d sStartPt, sEndPt;
  SER(EvaluatePoint(crIntervalArg.GetMin(),sStartPt));
  SER(EvaluatePoint(crIntervalArg.GetMax(),sEndPt  ));

  // Used only if the chord length is substantial   
  SmVector3d sChord(sEndPt - sStartPt);
  SmVector3d sUnitChord; // set if chord sane

  // If the chord is short wrt the tolerance, use only 3 points
  // (hopefully the midpoint will fall apart of the start/end points)
  // when the curve is closed, and the start/end points coincide
  double dChordLength = (sStartPt - sEndPt).Length();
  SmBoolean bChordShort = FALSE;

  // for endpoints close together (short lines and closed curves)
  if (dChordLength < dToleranceArg) 
    {
        lSamplePointCountArg = 3;
        bChordShort = TRUE;
    }
  else // end points are more than tol apart
    {
      // unitize the chord
        sUnitChord = sChord;
        SM_ASSERT(sUnitChord.Unitize() == SM_SUCCESS); // diminishing edge: taken care above
    }

  // When this curve is of type SmLine
  SmLine* pL = SM_CAST_NONNULL_PTR(SmLine, this);
  if (pL) 
    {
      // set outputs
        rLinePointArg = sStartPt;
        rLineVectorArg = sChord; 
      if (bChordShort) 
        {
            SE(SM_ERR);
        }
        return TRUE;
    } // end curve is type SmLine check

  // Use no less than 3 points
  if (lSamplePointCountArg < 3)
      lSamplePointCountArg = 3;

  // Evaluating the curve and its derivative in all internal points
  for (ULONG i=1; i+1<lSamplePointCountArg; i++)  // note: can't say lSamplePointCountArg-1
    {
      double dParam = crIntervalArg.Evaluate((double)i/(lSamplePointCountArg-1.0));
      SmVector3d sPtDir[2];
      SER(Evaluate(dParam,1,TRUE,sPtDir));

      // If the segment is short, check if the midpoint coincided with 
      // the start/end points, and decide upon the result
      if (bChordShort ) 
        {
          double dDi = (sPtDir[0] - sStartPt).Length();
          if (dDi > dToleranceArg)
              return FALSE;
          rLinePointArg = sStartPt;
          rLineVectorArg = sChord; // DANGER: line, degenerating to a single
                                   // point, no way of warning
          SE(SM_ERR); 
          return FALSE;
        }

      // the chord is long now: check the distance between the curve point
      // and the assumed line
      double dDist;
      SER(smgu_LinePointDistance(sStartPt,sChord,sPtDir[0],dDist));

      if (dDist > dToleranceArg)
          return FALSE;

      double dDerivLen = sPtDir[1].Length();

      // we have no other clue wrt the world's 
      // resolution than the eps given to us        
      if (dDerivLen > dToleranceArg )  
        { 
          double dSinAngle = (sPtDir[1] * sUnitChord).Length() / dDerivLen;
          double dDiscr    = smos_Fabs(dSinAngle) * dChordLength / lSamplePointCountArg;
          // the derivative should not cause a large departure from the line 
          // along sampled interval
          if (dDiscr > dToleranceArg) 
            {
                return FALSE;
            }
        } // end Curve Deriv greater than tol check
    } // end iter every internal sample point

  // If we are here, it should be a line
  rLinePointArg = sStartPt;
  rLineVectorArg = sChord; 

  // all done
  return TRUE;

} // end SmBSplineCurve::IsSegmentLine

/*******************************************************************//**
PURPOSE: Get a range of knots from the knot vector and put them into
    the double array. 
    
NOTES: Note it is up to the user to make sure that there
    are at least (lEndIndex - lStartIndex + 1) doubles in the array.

  Be careful for this method, if you do not input sufficient
    array size it may over write some memory.
***********************************************************************/
SmStatus SmBSplineCurve::GetKnotsExpert
  (ULONG lStartIndex, 
   ULONG lEndIndex, 
   double *adKnots) 
  const
{
    gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    gw_KNOTVECTOR *knt = pCur->knt;
    SM_ASSERT(lEndIndex <= (ULONG)knt->m);
    SM_ASSERT(lStartIndex <= lEndIndex);
    for (ULONG i=lStartIndex; i<=lEndIndex; i++) {
        adKnots[i-lStartIndex] = knt->U[i];
    }
    return SM_SUCCESS;

} // end SmBSplineCurve::GetKnotsExpert

/*******************************************************************//**
PURPOSE: Get a control point in various forms.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::GetControlPoint
  (SmControlPointFormType eCtrlPointForm,  // SM_CP_NON_RATIONAL - do perspective projection and
                                           // set W=1.0 if it is rational.
                                           // SM_CP_HOMOGENEOUS_RATIONAL - don't do division and return W
                                           // SM_CP_EUCLIDIAN_RATIONAL - do division and return W
   ULONG lIndex,
   SmPoint3d & rControlPoint,
   double & rdWeight) 
  const
{
  double dTmp[4];
  dTmp[2] = dTmp[3] = 1.0; // Default for non-rational, 2d or 3d.
  SER(GetControlPointsExpert(eCtrlPointForm,lIndex,lIndex,4,dTmp));
  rControlPoint.Set(dTmp[0],dTmp[1],dTmp[2]);
  rdWeight = dTmp[3];
  if ( this->GetDim() == 2 )
    {
      rControlPoint.z = 0.0;
      rdWeight = dTmp[2];
    }
  return SM_SUCCESS;

} // end SmBSplineCurve::GetControlPoint

/*******************************************************************//**
PURPOSE: Get the control points of a given range from the control polygon
         of the curve.

NOTES: 
    Control Point data is packed as:
     when Curve->Dim() == 2                                                                 
         && SM_CP_NON_RATIONAL         pt = [X Y]           stride must be a multiple of 2                         
         && SM_CP_HOMOGENEOUS_RATIONAL pt = [X Y W]         stride must be a multiple of 3                         
         && SM_CP_EUCLIDIAN_RATIONAL   pt = [X/W Y/W W]     stride must be a multiple of 3                         
     when Curve->Dim() == 3                                                                 
         && SM_CP_NON_RATIONAL         pt = [X Y Z]         stride must be a multiple of 3                         
         && SM_CP_HOMOGENEOUS_RATIONAL pt = [X Y Z W]       stride must be a multiple of 4                         
         && SM_CP_EUCLIDIAN_RATIONAL   pt = [X/W Y/W Z/W W] stride must be a multiple of 4  
                                
     PtSize = ((SM_CP_NON_RATIONAL) ? this->Dim() : this->Dim() + 1)

    Be careful for this method, if you do not input sufficient
    array size it may over write some memory.
      ArraySize >= [lCtrlPointStride * (lEndIndex - lStartIndex + 1)]

   lCtrlPointStride: The number of doubles between control points in the output array.
     It can be any number equal to or bigger than [GetDim() + (eCtrlPointForm == SM_CP_NON_RATIONAL ? 0 : 1)],
     otherwise the next controlpoint data will overwrite the last.

     When lCtrlPointStride = a multiple of PtSize, the output array can represent a tightly packed
       array of control points as used in SmBSpline representations.  When
       lCtrolPointStride > PtSize, The output array will contain control point data with extra double storage between
          each point block of data.

   The Stride may be a multiple of these sizes, if you wish to space
   the points out in the array.
***********************************************************************/
SmStatus SmBSplineCurve::GetControlPointsExpert
  (SmControlPointFormType eCtrlPointForm,    // oneof: SM_CP_NON_RATIONAL           - output euclidean coords only
                                             //        SM_CP_HOMOGENEOUS_RATIONAL   - output homogeneous coords and weights
                                             //        SM_CP_EUCLIDIAN_RATIONAL     - output euclidean coords and weights
   ULONG                  lStartIndex,       // in: First control point index to be copied
   ULONG                  lEndIndex,         // in: Last  control point index to be copied
   ULONG                  lCtrlPointStride,  // in: spacing between control point data in output adControlPoints
                                             //     lCtrlPointStride >= ((SM_CP_NON_RATIONAL) ? this->Dim() : this->Dim() + 1)
   double               * adControlPoints)   // out: sized >= [lCtrlPointStride * (lEndIndex - lStartIndex + 1)]
                                             //      SM_CP_NON_RATIONAL         && 2d ? ordered:[X Y          X Y          ...] 
                                             //      SM_CP_HOMOGENEOUS_RATIONAL && 2d ? ordered:[X Y W        X Y W        ...] 
                                             //      SM_CP_EUCLIDIAN_RATIONAL   && 2d ? ordered:[X/W Y/W W    X/W Y/W W    ...] 
                                             //      SM_CP_NON_RATIONAL         && 3d ? ordered:[X Y Z        X Y Z        ...] 
                                             //      SM_CP_HOMOGENEOUS_RATIONAL && 3d ? ordered:[X Y Z W      X Y Z W      ...] 
                                             //      SM_CP_EUCLIDIAN_RATIONAL   && 3d ? ordered:[X/W Y/W Z/W  X/W Y/W Z/W  ...] 
  const
{
  gw_CURVE    * pCur        = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
  gw_CPOLYGON * pol         = pCur->pol;
  SmBoolean     bIsRational = IsRational();
  ULONG         lDim        = GetDim();
  ULONG         lMinStride  = lDim + (eCtrlPointForm == SM_CP_NON_RATIONAL ? 0 : 1) ;
  SmBoolean     bOKStride   = lCtrlPointStride >= lMinStride ; // stride must be larger than point BlockSize

  // check inputs - consistent indexing and strides
  if(   lStartIndex > lEndIndex        // okay when lStartIndex <= lEndIndex
     || lEndIndex   > (ULONG)pol->n    // okay when lEndIndex   <= gw_CPOLYGON->n and lStartIndex >= 0 (always TRUE)
     || bOKStride  != TRUE)            // okay when lCtrlPointStride >= Number of doubles in one Control Point.
    {
      SER_MSG(SM_ERR_INVALID_INPUT,_T("SmBSplineCurve::GetControlPointsExpert inconsistent input args")); // Can not get a rational from a non-rational curve
    }

  ULONG lAddress = 0;

  // for every requested control point
  for (ULONG i=lStartIndex; i<=lEndIndex; i++) 
    {
      gw_CPOINT *pCpt = &pol->Pw[i];
      double dX = pCpt->x;
      double dY = pCpt->y;
      double dZ = pCpt->z != NL_NOZ ? pCpt->z : 0.0 ; 
      double dW = pCpt->w != NL_NOW ? pCpt->w : 1.0 ;

      // SM_CP_NON_RATIONAL
      if (eCtrlPointForm == SM_CP_NON_RATIONAL) 
        {
          if (bIsRational) 
            {
              adControlPoints[lAddress]   = dX / dW;
              adControlPoints[lAddress+1] = dY / dW;
              if (lDim > 2) 
                {
                  adControlPoints[lAddress+2] = dZ / dW;
                }
            }
          else // nonRational curves
            {
              adControlPoints[lAddress]   = dX;
              adControlPoints[lAddress+1] = dY;
              if (lDim > 2) 
                {
                  adControlPoints[lAddress+2] = dZ;
                }
            }
        } // end SM_CP_NON_RATIONAL branch 

      // SM_CP_HOMOGENEOUS_RATIONAL
      else if (eCtrlPointForm == SM_CP_HOMOGENEOUS_RATIONAL) 
        { 
          adControlPoints[lAddress]   = dX;
          adControlPoints[lAddress+1] = dY;
          if (lDim > 2) 
            {
              adControlPoints[lAddress+2] = dZ;
              adControlPoints[lAddress+3] = dW;
            }
          else 
            { // 2D rational
              adControlPoints[lAddress+2] = dW;
            }
        } // end SM_CP_HOMOGENEOUS_RATIONAL branch

      // SM_CP_EUCLIDIAN_RATIONAL 
      else if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL) 
        {
          adControlPoints[lAddress]   = dX / dW;
          adControlPoints[lAddress+1] = dY / dW;
          if (lDim > 2) 
            {
              adControlPoints[lAddress+2] = dZ / dW;
              adControlPoints[lAddress+3] = dW;
            }
          else 
            { // 2D rational
              adControlPoints[lAddress+2] = dW;
            }
        } // end SM_CP_EUCLIDIAN_RATIONAL

      lAddress += lCtrlPointStride;
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::GetControlPointsExpert

/*******************************************************************//**
PURPOSE: Get the natural interval of the curve.  

NOTES: This is the interval
   which defines the minimum and maximum parameters at which the curve 
   can be evaluated.  In some cases this interval can contain 
   essentially infinite values.
***********************************************************************/
SmExtent1d SmBSplineCurve::GetNaturalInterval
  () 
 const
{
  // parse the NLib curve
  gw_CURVE *pCurve = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();

  // watch out for something odd
  if ( pCurve == NULL) // if there is no NURB curve
    { return SmExtent1d(0.0, 0.0); }

  // build and return interval equal to the natural interval
  return ( SmExtent1d(pCurve->knt->U[0],
                      pCurve->knt->U[pCurve->knt->m]) 
         ) ;

} // end SmBSplineCurve::GetNaturalInterval

//      /*******************************************************************//**
//      PURPOSE: Get the STEP-based parameter interval of the curve. 
//      
//      NOTES: As a NURBS, it will return the natural interval.
//          All analytic curves should have their own implementations.
//      ***********************************************************************/
//      SmExtent1d SmBSplineCurve::GetSTEPInterval
//        () 
//       const
//      {
//          return GetNaturalInterval();
//      
//      } // end SmBSplineCurve::GetSTEPInterval

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this curve.

NOTES: 
   At the SmBSplineCurve level, this is just the Nurbs parameterization.

   For curves of degree 3 or higher, or even 2, this expansion
   can result in wildly pathological curve behavior even for the
   expansion factors used here.  Use this method with caution.
***********************************************************************/
SmExtent1d SmBSplineCurve::GetMaxAnalyticDomain() 
 const
{
  SmExtent1d sDom = GetNaturalInterval();

  // Do not extend closed curves.
  if ( IsClosed( sDom ) )
    { return sDom; }

  // Expansion depends on degree.
  ULONG lDeg = this->GetDegree();
  if      ( lDeg == 1 ) { sDom.ExpandRelative( 100.0 ); }
  else if ( lDeg == 2 ) { sDom.ExpandRelative(   1.0 ); }
  else                  { sDom.ExpandRelative(   0.5 ); }

  return sDom;

} // end SmBSplineSurface::GetMaxAnalyticDomain

/*******************************************************************//**
PURPOSE: Get the number of unique knots in this curve.

NOTES: 
***********************************************************************/
ULONG SmBSplineCurve::GetNumberOfUniqueKnots
  () 
 const
{
    gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    gw_KNOTVECTOR *pKnt = pCur->knt;
    gw_REAL* U = pKnt->U;
    // Copy unique knots and compute multiplicities
    ULONG lCount = 0;
    for (ULONG j = GetDegree(); j < pKnt->m - GetDegree(); j++) {
        if (! SM_IS_ZERO(U[j] - U[j+1])) {
            lCount ++;
        }
    }

    lCount ++; // There is one more than the number of spans which
    // we just counted.
    return lCount;

} // end SmBSplineCurve::GetNumberOfUniqueKnots

/*******************************************************************//**
PURPOSE: Get the Euclidian control polygon of a B-Spline curve.  

NOTES: If the curve is rational then the perspective projection 
    will be performed to produce the resulting polygon.
***********************************************************************/
SmStatus SmBSplineCurve::GetPolygon
  (const SmExtent1d    & crInterval,        // in : interval to examine
   SmTArray<SmPoint3d> & rEuclidianPolygon) // out: Control points in euclidean space within crInterval
  const
{
    // If the interval is equal to the natural interval then we don't need
    // to chop it up and use a sub segment.
    SmExtent1d sNaturalInterval = GetNaturalInterval();
    
    gw_CURVE *pCur = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer();
    gw_CPOLYGON *pCPOLYGON = pCur->pol; // Use natural domain

    // First make sure interval sent in is valid
    // Handle special case of degenerate curve differently - skip the interval 
    // test.
    if(   pCur->pol->n > 0 
       && !crInterval.IsContainedBy(sNaturalInterval, SM_EFF_ZERO)) 
      { SER(SM_ERR_INVALID_INPUT); }
    
    ULONG lStartPoint, lNumPoints;
    if(   (   sNaturalInterval.GetMin() == crInterval.GetMin() 
           && sNaturalInterval.GetMax() == crInterval.GetMax()) 
       || pCur->pol->n == 0) 
     {
        lStartPoint = 0;
        lNumPoints = pCur->pol->n + 1;
      }
    else 
      { // use control polygon which contributes to the included spans
        gw_INDEX lStartSpan, lEndSpan;
        if(NL_YES == N_BasisFindSpan(pCur->knt,pCur->p,crInterval.GetMin(),NL_LEFT,&lStartSpan))
          { SER(SM_ERR); }
        // GW_SER(N_BasisFindSpan(pCur->knt,pCur->p,crInterval.GetMin(),NL_LEFT,&lStartSpan));
        if(NL_YES == N_BasisFindSpan(pCur->knt,pCur->p,crInterval.GetMax(),NL_RIGHT,&lEndSpan))
          { SER(SM_ERR); }
        // GW_SER(N_BasisFindSpan(pCur->knt,pCur->p,crInterval.GetMax(),NL_RIGHT,&lEndSpan));
 
        lStartPoint = lStartSpan - pCur->p;
        lNumPoints = lEndSpan - lStartPoint + 1;
      }
    
    SmPoint3d sPoint;
    gw_CPOINT *pCPOINT = &pCPOLYGON->Pw[lStartPoint];
    rEuclidianPolygon.ReSet();
    for (ULONG i=0; i<lNumPoints; i++) 
      {
        TO_EUCLID(pCPOINT[i],sPoint);
        rEuclidianPolygon.Add(sPoint);
      }
    
    return SM_SUCCESS;

} // end SmBSplineCurve::GetPolygon

//       
//      /*******************************************************************//**
//      PURPOSE: Checks if a curve is a planar curve given a 3d tolerance.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmBoolean SmBSplineCurve::IsPlanar
//        (double d3DTolerance) 
//       const
//      {
//          if (this->IsAnalytic() || this->IsDegenerate(d3DTolerance)) {
//              return TRUE;
//          }
//      
//          SmExtent1d sExt = this->GetNaturalInterval();
//          SmPoint3d sPData[256];
//          SmTArray<SmPoint3d> sPolygonPts(256,sPData);
//          if (this->GetPolygon(sExt,sPolygonPts) != SM_SUCCESS) {
//              SE(SM_ERR);
//              return FALSE;
//          }
//          ULONG lPolygonSize = sPolygonPts.GetSize();
//          if (lPolygonSize <= 3) return TRUE;
//      
//          for (ULONG i=1; i<lPolygonSize; i++) {
//              SmVector3d sVec0 = sPolygonPts[i] - sPolygonPts[0];
//              if (sVec0.LengthSquared() < SM_EFF_ZERO_SQ) {
//                  continue;
//              }
//              SE(sVec0.Unitize());
//              for (ULONG j=i+1; j<lPolygonSize; j++) {
//                  SmVector3d sVec1 = sPolygonPts[j] - sPolygonPts[0];
//                  if (sVec1.LengthSquared() < SM_EFF_ZERO_SQ ||
//                      sVec1.IsParallelTo(sVec0,0.1)) {
//                      continue;
//                  }
//                  SE(sVec1.Unitize());
//                  SmVector3d sNormal = sVec0*sVec1;
//                  SE(sNormal.Unitize());
//                  for (ULONG k=j+1; k<lPolygonSize; k++) {
//                      double dDistToPlane;
//                      SE(smgu_PlanePointDistance(sPolygonPts[0],sNormal,sPolygonPts[k],dDistToPlane));
//                      if (dDistToPlane > d3DTolerance) {
//                          return FALSE;
//                      }
//                  }
//                  break;
//              }
//              break;
//          }
//      
//          return TRUE;
//      
//      } // end SmBSplineCurve::IsPlanar

/*******************************************************************//**
PURPOSE: Checks if a curve is on the plane by the given tolerance.

NOTES:
  Finds the max distance between the curve's controlPolygon and the
  given plane. 
***********************************************************************/
SmStatus SmBSplineCurve::IsOnPlane
  (const SmPoint3d  & crPlanePointArg,     // in : point on plane
   const SmVector3d & crPlaneNormalArg,    // in : plane normal direction
   double             d3DToleranceArg,     // out: max allowed deviation between curve and plane
   SmBoolean        & rbIsOnPlaneArg,      // out: TRUE = all curve points are within tol of plane
   double           & rdMaxDistToPlaneArg, // out: max distance between the curve's control polygon and the given plane
   const SmExtent1d * pOptInterval)        // in : optional curve interval to check, NULL for Natural Interval, default:[NULL]
  const
{
  // Method does a quick 3-point test, and drops the curve if the 3 points
  // are found ON the plane.

  const SmBSplineCurve* pBSC = this;
  SmPoint3d sStartPt, sEndPt, sMidPt;
  SmExtent1d sIV;
  if( pOptInterval)
        sIV = *pOptInterval;
  else
        sIV = pBSC->GetNaturalInterval();

  // get Curve start, mid, end points
  SER(pBSC->EvaluatePoint(sIV.GetMin(),sStartPt));
  SER(pBSC->EvaluatePoint(sIV.GetMax(),sEndPt));
  SER(pBSC->EvaluatePoint(sIV.Evaluate(0.5),sMidPt));

  // project points to plane
  SmPoint3d sProjStartPt, sProjEndPt, sProjMidPt;
  SER(smgu_PointProjectToPlane(sStartPt,
                               crPlanePointArg,
                               crPlaneNormalArg,
                               sProjStartPt));
  SER(smgu_PointProjectToPlane(sEndPt,
                               crPlanePointArg,
                               crPlaneNormalArg,
                               sProjEndPt));
  SER(smgu_PointProjectToPlane(sMidPt,
                               crPlanePointArg,
                               crPlaneNormalArg,
                               sProjMidPt));

  // get start, mid, end point distances to plane
  double dStDiscr  =  (sStartPt - sProjStartPt).Length();
  double dEndDiscr = (sEndPt - sProjEndPt).Length();
  double dMidDiscr = (sMidPt - sProjMidPt).Length();

  // low work - points not on plane
  if (   dStDiscr  > d3DToleranceArg 
      || dEndDiscr > d3DToleranceArg 
      || dMidDiscr > d3DToleranceArg) 
    {
      rbIsOnPlaneArg = FALSE;
      return SM_SUCCESS;
    }

  // get max of 3 sample point distances - good enough for lines
  double dMaxDist = smos_3Max(dStDiscr,dMidDiscr,dEndDiscr);

  // for nonLine curves - get max distance to controlPoint
  if (!IsKindOf(SmLine_TYPE)) 
    {
      SmTArray<SmPoint3d> sPolygon;
      SER(GetPolygon(sIV,sPolygon));
      for (ULONG i=0; i<sPolygon.GetSize(); i++) 
        {
          SmPoint3d sPnt = sPolygon[i];
          double dDist;
          SER(smgu_PlanePointDistance(crPlanePointArg,crPlaneNormalArg,sPnt,dDist));
          if (dDist > dMaxDist) dMaxDist = dDist;
        }
    }

  // set output
  rdMaxDistToPlaneArg = dMaxDist;
  rbIsOnPlaneArg      = (rdMaxDistToPlaneArg < d3DToleranceArg ) ? TRUE : FALSE;

  return SM_SUCCESS;

} // end SmBSplineCurve::IsOnPlane

/*******************************************************************//**
PURPOSE: Join the other curve onto this curve.  

NOTES: The result is that this curve is modified.
***********************************************************************/
SmStatus SmBSplineCurve::JoinWith
  (ULONG lJoinEndThis,           // in : 0 = Join at thisCurve start 
                                 //      1 = Join at thisCurve end
   SmBSplineCurve *pOtherCurve,  // in : 
   ULONG lJoinEndOther,          // out: 0 = Join at OtherCurve start
                                 //      1 = Join at OtherCurve end
   double* pGapTolerance )       // in:  Optional tolerance reprsenting max gap between endpoints
                                 //      If not specified then use default tolerance based on length
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe) 
    {
      this->Dump();
      pOtherCurve->Dump();
      smgfx_SetLook( 2,4, 1,0,0 ); DrawWDeriv(GetNaturalInterval(),0); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,0,1 ); pOtherCurve->DrawWDeriv(pOtherCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  const SmContext *pContext = GetContext();

  SmExtent1d sIvl1 = GetNaturalInterval();
  SmExtent1d sIvl2 = pOtherCurve->GetNaturalInterval();

  // Get a tolerance to use.
  double dTol = 1.0;
  if( pGapTolerance != NULL )
    {
      dTol = *pGapTolerance;
    }
  else
    {
      double dLength = ApproximateLength(sIvl1,10) + pOtherCurve->ApproximateLength(sIvl2, 10);
      dTol = dLength * SM_EFF_ZERO_SQRT * 1000.0;
    }

  // Make sure that the points of join ends are close enough
  SmPoint3d sPnt1, sPnt2;
  SER(EvaluatePoint(sIvl1.Evaluate(lJoinEndThis),sPnt1));
  SER(pOtherCurve->EvaluatePoint(sIvl2.Evaluate(lJoinEndOther),sPnt2));
  if (sPnt1.DistanceBetween(sPnt2) > dTol) { return SM_ERR; }

  // First let's handle the simple case of two lines
  // with the same vector.
  SmVector3d sLineVec1, sLineVec2;
  SmPoint3d sLinePoint1, sLinePoint2;
  if (IsLine(5,dTol,sLinePoint1,sLineVec1) &&
      pOtherCurve->IsLine(5,dTol,sLinePoint2,sLineVec2) ) 
    {
      // Create a line from the non-joining ends and test the other points.
      SmPoint3d sPnt1B, sPnt2B;
      SER(EvaluatePoint(sIvl1.Evaluate(1-lJoinEndThis),sPnt1B));
      SER(pOtherCurve->EvaluatePoint(sIvl2.Evaluate(1-lJoinEndOther),sPnt2B));
      double dDist, dParam;
      SER(smgu_SegmentPointDistance(sPnt2B,sPnt1B,sPnt1,dDist,dParam));
      SmBoolean bCanJoinLines = FALSE;
      if (dDist < dTol) 
        {
          // Only can join continuous lines.
          SER(smgu_SegmentPointDistance(sPnt2B,sPnt1B,sPnt2,dDist,dParam));
          if (dDist < dTol) 
            {
              // Only can join continuous lines.
              bCanJoinLines = TRUE;
            }
        }
      if (bCanJoinLines) 
        {
          SmLine *pLine = SM_CAST_NONNULL_PTR(SmLine,this);
          if (pLine) 
            {
              SmPoint3d sLinePnt;
              SmVector3d sLineVec;
              SER(pLine->GetCanonical(sLinePnt,sLineVec));
              SmExtent1d sNewInterval = pLine->GetSTEPInterval();
              if (lJoinEndThis == 1) 
                {
                  SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sPnt2B,dParam));
                  sNewInterval.SetMinMax(sNewInterval.GetMin(),dParam);
                }
              else 
                {
                  SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sPnt2B,dParam));
                  sNewInterval.SetMinMax(dParam,sNewInterval.GetMax());
                }
              SER(pLine->AdjustSTEPInterval(sNewInterval));
            }
          else 
            {
              SmBSplineCurve *pNewLine = NULL ;
              if (lJoinEndThis == 1) 
                {
                  SER(SmBSplineCurve::CreateLineSegment(*pContext,GetDim(),sPnt1B,sPnt2B,pNewLine));
                }
              else 
                {
                  SER(SmBSplineCurve::CreateLineSegment(*pContext,GetDim(),sPnt2B,sPnt1B,pNewLine));
                }
              SmObjDelete sClean(pNewLine);
              SER(this->SetFromGwNurb(0, pNewLine->GetOrCreateGwNurbPointer()));
            }
          return SM_SUCCESS; // Done
        }
    }
  
  // Try using nurb-join
  SmTArray<SmBSplineCurve*> sCurves;
  SmTArray<SmBoolean> sOrients;
  if (lJoinEndThis == 1) 
    {
      sCurves.Add(this);
      sCurves.Add(pOtherCurve);
      sOrients.Add(TRUE);
      if (lJoinEndOther == 0) 
        {
          sOrients.Add(TRUE);
        }
      else 
        {
          sOrients.Add(FALSE);
        }
    }
  else 
    {
      sCurves.Add(pOtherCurve);
      sCurves.Add(this);
      if (lJoinEndOther == 1) 
        {
          sOrients.Add(TRUE);
        }
      else 
        {
          sOrients.Add(FALSE);
        }
      sOrients.Add(TRUE);
    }
  SmBSplineCurve *pComposite = NULL ;
  if (SmBSplineCurve::CreateByJoining(*pContext,sCurves,&sOrients,pComposite) != SM_SUCCESS) 
    {
      return SM_ERR;
    }
  SmObjDelete sClean(pComposite);
  SER(this->SetFromGwNurb(0, pComposite->GetOrCreateGwNurbPointer()));

  return SM_SUCCESS;

} // end SmBSplineCurve::JoinWith

/*******************************************************************//**
PURPOSE: Make the NURB curve representation - abstract method

NOTES: This function is used by Derived Types like SmLine 
    and SmCircle to build BrepCurves from member data stored in 
    their instances.
***********************************************************************/
SmStatus SmBSplineCurve::MakeNurb
  ()
{
  if (m_pNurb == NULL) { SER(SM_ERR); }
  return SM_SUCCESS;

} // end SmBSplineCurve::MakeNurb

/*******************************************************************//**
PURPOSE: Make the weights of this curve match those of the given curve.

NOTES: The curves must be compatible (MakeCurvesCompatible());
    in particular, they must have the same number of control points,
    and that's the only thing that this routine checks.
    Returns SM_ERR if not.
***********************************************************************/
SmStatus SmBSplineCurve::MatchWeights
  ( const SmBSplineCurve *pOther )
{
  NER( pOther );

  // Work directly on the arrays of coutrol points and weights.
  ULONG jj, lNumCPts, lNumCPtsOther;
  double *pOurPts = NULL, *pOtherPts = NULL;

  this  ->GetControlPointsPointer( lNumCPts,      pOurPts );
  pOther->GetControlPointsPointer( lNumCPtsOther, pOtherPts );
  if ( lNumCPtsOther != lNumCPts )
    { return SM_ERR; }

  // Don't Notify() unless we actually get modified.
  SmBoolean bModified = FALSE;

  // The control point arrays are sequential quadruples of doubles: x,y,z,w.
  // Index directly into those arrays.
  ULONG idx = 0;
  for ( jj = 0; jj < lNumCPts; jj++ )
    {
      if ( ! SM_ARE_SAME( pOurPts[idx+3], pOtherPts[idx+3] ) )
        {
          if ( !bModified )
            {
              bModified = TRUE;
              this->Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
            }

          double dRatio = pOtherPts[idx+3] / pOurPts[idx+3];
          pOurPts[idx+0] *= dRatio;
          pOurPts[idx+1] *= dRatio;
          pOurPts[idx+2] *= dRatio;
          pOurPts[idx+3]  = pOtherPts[idx+3];
        }
      idx += 4;
    }

  if ( bModified )
    { this->Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL); }

  return SM_SUCCESS;

} // end SmBSplineCurve::MatchWeights

/*******************************************************************//**
PURPOSE: Given a type of validity check to perform, determine if the
    curve passes.

NOTES: This method violates predicate protocol by having side effects.
       It calls N_CrvIsNotReversed asking to move Cpts to fix a
       Cpts sequence reversal.
***********************************************************************/
SmBoolean SmBSplineCurve::PassesValidityCheck
  (SmValidityCheckType eChecks,
   SmValidityCheckType & reCheckFailed) //this is the check which failed
  const
{
  reCheckFailed = SM_VC_NONE;
  
  if (eChecks == SM_VC_ALL || eChecks == SM_VC_DEFINITION) 
    {
      gw_FLAG err = N_CrvIsValid(((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer(),
                                 _T("SmBSplineCurve::PassesValidityCheck"));
      if (err) 
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          if(bDebugMe)
            {
              this->Dump() ;
            }
#endif // SM_DEBUG_CODE
          reCheckFailed = SM_VC_DEFINITION;
          return FALSE;
        }
    }
  if (eChecks == SM_VC_ALL || eChecks == SM_VC_REVERSE_DIRECTION) 
    {

      // GWC_NOTE: GWC_N_CrvIsNotReversed_HAS_SIDE_EFFECTS GWC_LINE ;
      //           Making the last N_CrvIsNotReversed arg == true here violates the idea that we
      //           are just checking validity since it may also modify the curve if a bad case is found.
      //           However, on balance this is a preferred solution
      gw_FLAG err = N_CrvIsNotReversed( ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer(), 
                                        true) ; /* in : TRUE = Move CPts to fix a Cpts sequence reversal, FALSE=don't */

      if (err) 
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          if(bDebugMe)
            {
              this->Dump() ;
            }
#endif // SM_DEBUG_CODE
          reCheckFailed = SM_VC_REVERSE_DIRECTION;
          return FALSE;
        }

    }
  return SmCurve::PassesValidityCheck(eChecks,reCheckFailed);

} // end SmBSplineCurve::PassesValidityCheck

/*******************************************************************//**
PURPOSE: Given a curve raise the degree to the specified degree.
    
NOTES: rtn SM_SUCCESS when NewDegree >= CurrentDegree
           IW_ERR     when NewDegree <  CurrentDegree
***********************************************************************/
SmStatus SmBSplineCurve::DegreeElevate
  (ULONG lNewDegree)
{
    // no work - no degree increase needed
    if (lNewDegree < GetDegree()) SER(SM_ERR);
    if (lNewDegree == GetDegree()) return SM_SUCCESS;

    // locals
    NL_STACKS SC;
    SmStackHandler sStackKp(&SC);

    gw_CURVE sCur;
    N_CrvInitArrays(&sCur);
    ULONG lIncrement = lNewDegree - GetDegree();
    if(NL_YES == N_CrvElevateDegree(GetOrCreateGwNurbPointer(),lIncrement,&sCur,&SC,&SC))
      { SER(SM_ERR); }
    // GW_SER(N_CrvElevateDegree(GetOrCreateGwNurbPointer(),lIncrement,&sCur,&SC,&SC));

    // If made it here we removed at least one knot
    // This will create a shrunken version of the curve
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),&sCur);
    SmObjDelete sClean(pTmpBSC);

    // Swap Nurbs with this
    gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
    pTmpBSC->m_pNurb = m_pNurb;
    m_pNurb          = pTmp;

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmBSplineCurve::DegreeElevate

/*******************************************************************//**
PURPOSE: Drop a point onto this curve using LocalPointSolve(with GuessParam) or
         GlobalPointSolve(without GuessParam) and snap to endPoints when asked.

NOTES:
***********************************************************************/
//SmStatus SmBSplineCurve::DropPoint
//  (const SmExtent1d & crInterval,         // in : target curve allowed domain
//   const SmPoint3d  & crPointToDrop,      // in : Point to drop to curve
//   const SmVector3d * cpVecToCrvInside,   // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
//                                          //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
//                                          //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
//   double             dDistTol,           // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
//                                          //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
//                                          //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
//                                          //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
//   const double     * pOptGuessParam,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
//   SmBoolean        & rbSuccess,          // out: TRUE = found a drop point
//   double           & rdDroppedParameter, // out: found drop curve param
//   double           & rdDistanceToCurve,  // out: found drop distance
//   SmSolverOperationType eOperationType)  // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
// const                                    //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
//                                          //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
//                                          //      default:[SM_SO_MINIMIZE] to preserve original behavior
//{
//  // init output
//  rbSuccess = FALSE;
//  rdDroppedParameter = 0.0;
//  rdDistanceToCurve  = 0.0;
//
//  // check inputs
//  if(   eOperationType != SM_SO_MINIMIZE
//     && eOperationType != SM_SO_INTERSECT
//     && eOperationType != SM_SO_NORMALIZE)
//    {
//      WARN(_T("Bad DropPoint eOperationType - selecting eOperationType = SM_SO_MINIMIZE")) ;
//      eOperationType = SM_SO_MINIMIZE ;
//    }
//
//#ifdef SM_DEBUG_CODE
//SmBoolean bDebugMe = FALSE;
//  // draw PointToDrop(red), TargetCurve(blue)
//  if (bDebugMe)
//    {
//      Dump();
//
//      smgfx_SetLook(3,5, 1,0,0); crPointToDrop.Draw(); sm_GraphicsLoop();
//      smgfx_SetLook(3,5, 0,0,1); DrawWDeriv(crInterval,0); sm_GraphicsLoop();
//      sm_GraphicsLoop();
//    }
//#endif // SM_DEBUG_CODE
//
//  SmTArray<SmPoint3d> sControlPoints;
//  SmTArray<double> sWeights;
//
//  this->GetControlPolygon(sControlPoints, sWeights);
//
//  double dMin = SM_BIG_DOUBLE;
//  ULONG lMinIndex = 0;
//
//  for (ULONG ii = 0; ii < sControlPoints.GetSize(); ii++)
//  {
//      double dDistSq = sControlPoints[ii].DistanceBetweenSquared(crPointToDrop);
//      if (dDistSq < dMin)
//      {
//          dMin = dDistSq;
//          lMinIndex = ii;
//      }
//  }
//
//  const double dMinParam = crInterval.SnapValue(this->GetGrevilleAbscissa(lMinIndex));
//
//  return this->SmCurve::DropPoint(crInterval, crPointToDrop, cpVecToCrvInside, dDistTol, &dMinParam, rbSuccess,
//                                  rdDroppedParameter, rdDistanceToCurve, eOperationType);
//
//} // end SmBSplineCurve::DropPoint


/*******************************************************************//**
PURPOSE: Remove extra knots in the B-Spline curve.  

NOTES: A knot will be removed if it does not move the curve 
   more than the tolerance.
***********************************************************************/
SmStatus SmBSplineCurve::RemoveExtraKnots
  (double d3DRemovabilityTolerance)
{
    NL_STACKS SC;
    SmStackHandler sStackKp(&SC);
    gw_CURVE *crv = GetOrCreateGwNurbPointer();
    if (crv == NULL) return SM_SUCCESS;
    long lNumOldKnots = crv->knt->m;
    sm_CrvRemoveKnots(crv,d3DRemovabilityTolerance,crv,&SC);

    // If no knots needed removing then just return here
    if (crv->knt->m == lNumOldKnots) return SM_SUCCESS;

    // If made it here we removed at least one knot
    // This will create a shrunken version of the curve
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    SmBSplineCurve *pTmpBSC = new (*GetContext()) SmBSplineCurve(GetDim(),crv);
    SmObjDelete sClean(pTmpBSC);

    // Swap Nurbs with this
    gw_CURVE *pTmp   = pTmpBSC->m_pNurb;
    pTmpBSC->m_pNurb = m_pNurb;
    m_pNurb          = pTmp;

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    return SM_SUCCESS;

} // end SmBSplineCurve::RemoveExtraKnots

/*******************************************************************//**
PURPOSE: Reverse curve parameterizationwhile preserving its NaturalInterval range.

NOTES: 1. example: NewPoint(sNatIvl.Min) == OldPoint(sNatIvl.Max) with negated tangents.
       2. For convenience, Given an old interval of interest in crOldInterval, 
                           Output the new domain for crOldInterval in rNewInterval.
          The crOldInterval value does not affect the reparameterization of the curve.
***********************************************************************/
SmStatus SmBSplineCurve::ReverseParameterization
  (const SmExtent1d & crOldInterval,  // in : interval of interest, may be a subset of natural interval
   SmExtent1d       & rNewInterval)   // out: new domain for crOldInterval on modified curve               
{
    NL_STACKS SC;
    SmStackHandler sStackKp(&SC);

    SmExtent1d sIvl = GetNaturalInterval();
    double dTNewMax = 1.0 - (crOldInterval.GetMin() - sIvl.GetMin()) / (sIvl.GetMax() - sIvl.GetMin());
    double dTNewMin = 1.0 - (crOldInterval.GetMax() - sIvl.GetMin()) / (sIvl.GetMax() - sIvl.GetMin());
    
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    gw_CURVE *pCur = GetOrCreateGwNurbPointer();
    if(NL_YES == N_CrvReverse(pCur,pCur,&SC))
      { SER(SM_ERR); }
    // GW_SER(N_CrvReverse(pCur,pCur,&SC));
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    SmExtent1d sNewIvl = GetNaturalInterval();
    rNewInterval.SetMinMax(sNewIvl.Evaluate(dTNewMin),sNewIvl.Evaluate(dTNewMax));
    return SM_SUCCESS;

} // end SmBSplineCurve::ReverseParameterization

/*******************************************************************//**
PURPOSE: Edit an existing BSpline by setting it with the values
    in the NLib Nurb.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::SetFromGwNurb
  (ULONG,                         // in : not used
   const gw_CURVE* cpGwNurbCurve) // in : set m_pNurb = copy of this object
                                  //      
{
  // tell the hierarchy we are about to edit the parameterization
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // free existing m_pNurb 
  if (m_pNurb && !m_bNurbIsBorrowed) { smos_Free(m_pNurb); m_pNurb = NULL ; }

  if (cpGwNurbCurve == NULL) return (SM_ERR);

  // let m_pNurb = copy(cpGwNurbCurve)
  m_pNurb = sm_AllocateAndCopyNurbCurve((gw_CURVE *)cpGwNurbCurve);
  m_bNurbIsBorrowed = FALSE;

  // tell hierarchy parameterization has been edtied
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::SetFromGwNurb

/*******************************************************************//**
PURPOSE: Edit an existing BSpline by setting it with new canonical
   values.  

NOTES: See SmBSplineCurve::CreateCanonical for additional 
   documentation on the parameters.
***********************************************************************/
SmStatus SmBSplineCurve::SetCanonical
  (ULONG lDimension,
   ULONG lDegree, 
   const SmTArray<SmPoint3d> & crControlPointsList,
   SmBSplineCurveForm eBSplineCurveForm,
   const SmTArray<ULONG> & crKnotMultiplicities,
   const SmTArray<double> & crKnots,
   SmKnotType eKnotType,
   const SmTArray<double> * cpWeights,
   const SmExtent1d * cpOptTrimInterval)
{
    // Create a temporary one
    SmBSplineCurve *pTmp = NULL ;
    const SmContext *pContext = GetContext();
    NER(pContext);

    SER(SmBSplineCurve::CreateCanonical(*pContext,
        lDimension, lDegree, crControlPointsList,
        eBSplineCurveForm, crKnotMultiplicities, crKnots,
        eKnotType, cpWeights, cpOptTrimInterval, pTmp));
            
    SmObjDelete sCleanup(pTmp);
            
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    
    // Swap Nurbs with this
    gw_CURVE *pTmpNurb = pTmp->m_pNurb;
    pTmp->m_pNurb      = m_pNurb;
    m_pNurb            = pTmpNurb;
    
    // Swap dimension
    ULONG lTmp = pTmp->GetDim();
    pTmp->m_lDim = lTmp;
    m_lDim = lTmp;

    if (m_bNurbIsBorrowed) {
        pTmp->m_bNurbIsBorrowed = TRUE;
        m_bNurbIsBorrowed = FALSE;
    }
    
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmBSplineCurve::SetCanonical

/*******************************************************************//**
PURPOSE: Edit the curve by setting one control point.
 
NOTES:
***********************************************************************/
SmStatus SmBSplineCurve::SetControlPoint
  (SmControlPointFormType eCtrlPointForm,    //  Either SM_CP_EUCLIDIAN_RATIONAL with a weight
                                             // or SM_CP_NON_RATIONAL which ignores the weight
   ULONG lIndex,
   const SmPoint3d & crControlPoint,
   double dWeight
   )
{
    gw_CURVE *pCur = GetOrCreateGwNurbPointer();
    gw_CPOLYGON *pCPol = pCur->pol;
    if (lIndex > (ULONG)pCPol->n) {
        SER(SM_ERR);
    }
 
    SmPoint3d sControlPoint = crControlPoint;

    // They're stored in homogeneous rational form:
    if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL)
    {
        sControlPoint.x *= dWeight;
        sControlPoint.y *= dWeight;
        sControlPoint.z *= dWeight;

        // make sure all the other control points are also rational 
        for (ULONG i = 0 ; i <= (ULONG)pCPol->n ; i++) 
        { 
            gw_CPOINT *pCpti = &pCPol->Pw[i];
            if (pCpti->w == NL_NOW) pCpti->w = 1.0;
        }
    }
    if (eCtrlPointForm == SM_CP_NON_RATIONAL)
        dWeight = 1.0;
 
    gw_CPOINT *pCpt = &pCPol->Pw[lIndex];
 
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    pCpt->x = sControlPoint.x;
    pCpt->y = sControlPoint.y;
    pCpt->z = sControlPoint.z;
    if ( IsRational() )
        pCpt->w = dWeight;
 
    // Warn
    if(IsKindOf(SmEllipse_TYPE))
      {
        ERR_MSG(_T("SmBSplineCurve::SetControlPoint modified the shape of an ellipse/circle object")) ;
      }

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
 
    return SM_SUCCESS;

} // end SmBSplineCurve::SetControlPoint

/*******************************************************************//**
PURPOSE: Set the pointer to the underlying NURBS.

NOTES: 
***********************************************************************/
void SmBSplineCurve::SetGwNurbPointer
  (void *pGwNurbCurve)                // in : new m_pNurb pointer value
{ 
  SM_ASSERT(pGwNurbCurve == NULL || m_pNurb == NULL) ;
  m_pNurb = (gw_CURVE *) pGwNurbCurve; 

} // end SmBSplineCurve::SetGwNurbPointer

/*******************************************************************//**
PURPOSE: Set the BSplineCurve using data arrays.  

NOTES: This method
    should be flexible enough to handle most forms of NURBS and 
    Bezier curves which exist in various software products.  It is
    somewhat like the form used by OPENGL.  Please see the OPENGL
    description of gluNurbsCurve to better understand this method.
    
      This is an expert method which should only be used by
    those who are willing to take the risk.  
***********************************************************************/
SmStatus SmBSplineCurve::SetExpert
  (ULONG lDimension,
   ULONG lDegree, 
   SmBSplineCurveForm eBSplineCurveForm,
   SmEndKnotFormType eEndKnotForm,    // Defines how many knots on end of curve
   ULONG lNumKnots,                   
   const double *cadKnots,
   SmControlPointFormType eCtrlPointForm,  
   ULONG lCtrlPointStride,      // Doubles to skip between points in data array
   const double *cadCtrlPoints) // Double array of control points
{
    if (eEndKnotForm == SM_EK_UNCLAMPPED) {
        lNumKnots = lNumKnots + 2;
    }
    ULONG lCtrlPointCount = lNumKnots - lDegree - 1;

    gw_CURVE *pCur = sm_AllocateNurbCurve(lCtrlPointCount-1, (gw_DEGREE)lDegree, lNumKnots - 1);
    gw_KNOTVECTOR *pKnt = pCur->knt;
    gw_CPOLYGON *pPol = pCur->pol;
    gw_CPOINT *Pw = pPol->Pw;

    ULONG lIdx = 0;
    if (eEndKnotForm == SM_EK_UNCLAMPPED) {
        pKnt->U[lIdx++] = cadKnots[0];
        lNumKnots = lNumKnots - 2;
    }
    for (ULONG i=0; i<lNumKnots; i++) {
        pKnt->U[lIdx++] = cadKnots[i];
    }

    if (eEndKnotForm == SM_EK_UNCLAMPPED) {
        pKnt->U[pKnt->m] = cadKnots[lNumKnots-1];
    }

    for (ULONG j=0; j<lCtrlPointCount; j++) {
        double dX = cadCtrlPoints[j*lCtrlPointStride];
        double dY = cadCtrlPoints[j*lCtrlPointStride + 1];
        double dZ = 0.0;
        double dW = 1.0;
        ULONG lNum = 2;
        if (lDimension > 2) {
            dZ = cadCtrlPoints[j*lCtrlPointStride + 2];
            lNum ++;
        }
        else {
            dZ = NL_NOZ;
        }
        if (eCtrlPointForm != SM_CP_NON_RATIONAL) {
            dW = cadCtrlPoints[j*lCtrlPointStride + lNum];
        }
        else { // not rational
            dW = NL_NOW;
        }
        if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL) {
            // Put back into Euclidian form - suitable for
            // Create Canonical.
            dX = dX * dW;
            dY = dY * dW;
            if (lDimension > 2) {
                dZ = dZ * dW;
            }
        }
        Pw[j].x = dX;
        Pw[j].y = dY;
        Pw[j].z = dZ;
        Pw[j].w = dW;
    }
    
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    m_eBSplineCurveForm = eBSplineCurveForm;
    m_bNurbIsBorrowed = FALSE;
    m_eKnotType = SM_KT_UNSPECIFIED;

    // Swap Nurbs with this
    if (m_pNurb && !m_bNurbIsBorrowed) { smos_Free(m_pNurb); m_pNurb = NULL ; } 
    m_pNurb = pCur;

    // Swap dimension
    m_lDim = lDimension;

    // Warn
    if(IsKindOf(SmEllipse_TYPE))
      {
        ERR_MSG(_T("SmBSplineCurve::SetExpert modified the shape of an ellipse/circle object")) ;
      }

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmBSplineCurve::SetExpert

/*******************************************************************//**
PURPOSE: Simplify a bspline curve by a zero distance offset

NOTES: Replace a 'multiple-interior-knots lots-of-spans' curve
       with a simplified non-rational curve.
       Approximation uses internally computed tolerances as
        old: dTol = 1.0e-4 * Curve's BoundingBoxSize.
        new: GetApproxTol3d()
       Useful data reduction tool, when precision is not important.
***********************************************************************/
SmStatus SmBSplineCurve::Simplify 
  (SmBSplineCurve *& rpNewCurve,   // out: Approximate curve built with 
                                   //      old: Tol = 1.0e-4 * Curve's BoundingBoxSize
                                   //      new: GetApproxTol3d()
   double          * pOptMaxGap3d) // out: achieved max gap for approximation, NULL to ignore
                                   //      default:[NULL]
{ 
  // init output
  if(pOptMaxGap3d) { *pOptMaxGap3d = 0.0 ; }

  // locals
  const SmContext *pContext        = this->GetContext();
  SmExtent1d       sIvl            = this->GetNaturalInterval();

  // pick a tolerance for upcoming approximation
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(this) ;
#else // SM_USE_OLDTOL 

  SM_OLDTOL_LINE // get bounding box and size
  SM_OLDTOL_LINE SmExtent3d sCrvBox;
  SM_OLDTOL_LINE this->CalculateBoundingBox(sIvl, &sCrvBox);
  SM_OLDTOL_LINE double size = sCrvBox.GetSize().Length();
  SM_OLDTOL_LINE if (size >= 1.0e15 || size <= 0.0) size = 1.0;
  SM_OLDTOL_LINE 
  SM_OLDTOL_LINE SmApproxTol3d sApproxTol3d = 1.0e-4 * size ;
#endif // SM_USE_OLDTOL

  // ApproximateCurve locals
  double dMaxGap3d;
  SmTArray<double> sBreaks; 
  sBreaks.Add(sIvl.GetMin()); 
  sBreaks.Add(sIvl.GetMax());

  // Approximate the curve
  this->ApproximateCurve(*pContext, 
                          sBreaks, 
                          sApproxTol3d, 
                          dMaxGap3d, 
                          rpNewCurve,
                          TRUE,    // in : bOptCreateAnalytics      
                          FALSE,   // in : bOptMatchParameterization
                          FALSE) ; // in : bJustCopyBSplines        
  
  // set output
  if(pOptMaxGap3d) { *pOptMaxGap3d = dMaxGap3d ; }
  
  // all done    
  return ( SM_SUCCESS);

} // end SmBSplineCurve::Simplify

/*******************************************************************//**
PURPOSE: Determine if a parameter lies within the interval or near enough
    to the ends (within tolerance) so that it should be snapped to the end 
    of the interval.  

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::SnapInsideInterval
  (const SmExtent1d & rInterval,
   double dDistanceTolerance,
   double dCurveParameter,
   SmBoolean & bSuccessfulSnap,
   double & rdSnappedParameter,        
   SmBoolean & bOnBoundary) 
  const        
{ 
    if(m_pNurb == NULL) { ((SmBSplineCurve *)this)->MakeNurb(); } SM_ASSERT(m_pNurb != NULL) ;
    bOnBoundary = FALSE;
    bSuccessfulSnap = FALSE;
    
    if (rInterval.ContainsValue(dCurveParameter)) {
        bSuccessfulSnap = TRUE;
        rdSnappedParameter = dCurveParameter;
        if (SM_ARE_SAME(dCurveParameter,rInterval.GetMin())  ||
            SM_ARE_SAME(dCurveParameter,rInterval.GetMax()) ) {
            bOnBoundary = TRUE;
        }
        return SM_SUCCESS;
    }
    
    if (dCurveParameter < rInterval.GetMin()) {
        SmPoint3d sPnt;
        SER(EvaluatePoint(dCurveParameter,sPnt));
        SmPoint3d sMinPnt;
        SER(EvaluatePoint(rInterval.GetMin(),sMinPnt));
        if (sPnt.DistanceBetween(sMinPnt) < dDistanceTolerance) {
            bSuccessfulSnap = TRUE;
            bOnBoundary = TRUE;
            rdSnappedParameter = rInterval.GetMin();
        }
        return SM_SUCCESS;
    }
    else {
        // Curve parameter must be > rInterval.GetMax()
        SmPoint3d sPnt;
        SER(EvaluatePoint(dCurveParameter,sPnt));
        SmPoint3d sMaxPnt;
        SER(EvaluatePoint(rInterval.GetMax(),sMaxPnt));
        if (sPnt.DistanceBetween(sMaxPnt) < dDistanceTolerance) {
            bSuccessfulSnap = TRUE;
            bOnBoundary = TRUE;
            rdSnappedParameter = rInterval.GetMax();
        }
    }
    return SM_SUCCESS;

} // end SmBSplineCurve::SnapInsideInterval

/*******************************************************************//**
PURPOSE: Subdivide the NURB Curve at given continuity points or
    lower continuity.  

NOTES: If no subdivisions done a copy of the curve
    will be placed into rResultingCurves.
***********************************************************************/
SmStatus SmBSplineCurve::SubdivideAtDiscontinuity
  (const SmContext           & crContext,           // in : context for new object construction
   SmContinuityType            eContinuityToSplit,  // in : curve is split at internal continuities <= to this value 
   SmTArray<SmBSplineCurve*> & rResultingCurves,    // out: newly copied curves - may be 1 curve copy when no internal discontinuities
   double                      dContinuityAngleTol) // in : continuity angle tolerance
{
  // init output
  rResultingCurves.ReSet();

  // locals
  ULONG ii ;
  SmContinuityType           eMinCont;
  SmTArray<double>           sKnots;
  SmTArray<SmContinuityType> sContinuities;
  SER(GetKnots(sKnots,NULL));  // out: sKnots = Unique knot vector
  SER(CalculateContinuities(eMinCont, sContinuities, dContinuityAngleTol));

  // check state
  if (sKnots.GetSize() != sContinuities.GetSize()) { SER(SM_ERR); }

  // init iter params
  double dStart = sKnots[0];

  // for every unique interval (every unique knot but the first)
  for(ii=1;ii<sKnots.GetSize();ii++) 
    {
      double dCurrent = sKnots[ii];

      // for internal discontinuites or the last segment
      if (ii==sKnots.GetSize()-1 || sContinuities[ii] <= eContinuityToSplit) 
        {
          // copy the curve
          SmBSplineCurve *pCopy = new (crContext) SmBSplineCurve(*this);

          // trim it to the interval between discontinuities (may be the whole curve)
          SmExtent1d sTrim(dStart,dCurrent);
          pCopy->Trim(sTrim) ; // may snap sIvl by tol to existing knots

          // add trimmed copy to output
          rResultingCurves.Add(pCopy);

          // update iter param
          dStart = dCurrent;
        }
    } // end iter every internal unique knot

  // all done
  return SM_SUCCESS;

} // end SmBSplineCurve::SubdivideAtDiscontinuity

/*******************************************************************//**
PURPOSE: Subdivide the NURB Curve to a minimum set of lines and arcs

NOTES: If no subdivisions done a copy of the curve
       will be placed into rResultingCurves.
***********************************************************************/
SmStatus SmBSplineCurve::SubdivideToLinesAndArcs(
    const SmContext&            crContext,
    double                      dTol,
    SmTArray<SmBSplineCurve*> & rResultingCurves )
                                           
{
    rResultingCurves.ReSet();

    // If degenerate then do nothing
    if( this->IsDegenerate( dTol ) ) {
        return( SM_SUCCESS );
    }

    // If already linear then do nothing
    if( this->IsLinear( dTol, NULL, NULL, NULL ) ) {
        return( SM_SUCCESS );
    }

    // if already an arc, then do nothing
    SmAxis2Placement sFrame;
    double dRadius, dStartAngle, dEndAngle;
    if( this->IsArc( 16, dTol, sFrame, dRadius, dStartAngle, dEndAngle ) ) {
        return( SM_SUCCESS );
    }

    NL_FLAG error = NL_NO;
    NL_INDEX nCurves = 0;
    NL_CURVE** newCurves;

    /* Start NURBS */ 
    NL_STACKS  SG;
    N_InitNurbs(&SG);

    gw_CURVE* curP = ((SmBSplineCurve *)this)->GetGwNurbPointer();

    error = N_CrvSplitIntoLinesAndArcs(curP, dTol, &newCurves, &nCurves, &SG);
    if (error EQ NL_YES)
      { return( SM_ERR ); }

    for( NL_INDEX ii = 0; ii <= nCurves; ii++ ) {
        SmBSplineCurve* bs = new( crContext ) SmBSplineCurve( newCurves[ii], 3, FALSE );
        rResultingCurves.Add( bs );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
           smgfx_SetLook( 4, 3, 1,0,0 ); bs->Draw(); sm_GraphicsLoop();
        }
#endif
    }

    return SM_SUCCESS;

} // end SmBSplineCurve::SubdivideToLinesAndArcs

/*******************************************************************//**
PURPOSE: Tessellate a curve using either chord height and/or angular
    tessellation tolerance.

NOTES: If one of the tolerances are zero then that tolerance is
    not used in the calculation.  Both of the tolerances must not be zero.
    At least one of the outputs must be a non-NULL pointer.
***********************************************************************/
SmStatus SmBSplineCurve::Tessellate
  (const SmExtent1d & crInterval,                   // in : interval to tessellate
   double             dChordHeightTolerance,        // in : max height/length ratio between tess pts, 0=ignore
   double             dCrvAngleToleranceDeg,        // in : max angle between tess tangents, 0=ignore
   ULONG              lMinimumNumberOfSegments,     // in : max distance between tess pts, 0=ignore
   SmTArray<double>     * pParameters,              // out: array of split parameter values, NULL=ignore          
   SmTArray<SmPoint3d>  * pPoints,                  // out: associated 3D points, NULL=ignore                     
   SmTArray<SmVector3d> * pOptTangents)             // out: Optional array of tangent points for each sample point
                                                    //      NULL to ignore.                                       
   const
{
    // The SmCurve TessellateByBisection function works much
    // faster so it is now the default.
 //   if (TRUE || !m_pNurb) {   
    
    // init MaxLength control parameter      
    double dMaxLength =   (lMinimumNumberOfSegments > 1)
                        ? ApproximateLength(crInterval,5) / ((double)lMinimumNumberOfSegments)
                        : 0.0 ;
 
    // pass the call along
    return SmCurve::TessellateByBisection(crInterval,
                                          dChordHeightTolerance,
                                          dCrvAngleToleranceDeg,
                                          dMaxLength,              // GWC:CHANGE from 0.0
                                                                   // to give Bsplines lMinimumNumberOfSegments control
                                          pParameters, pPoints, pOptTangents);
 //   }

// replaced Cache Based tessellation scheme
//   AssertValid();
//    if (SM_IS_ZERO(dChordHeightTolerance) &&
//        SM_IS_ZERO(dCrvAngleToleranceDeg)) SER(SM_ERR_INVALID_INPUT);
//    if (pParameters == NULL && pPoints == NULL)
//    SER(SM_ERR_INVALID_INPUT);
//    
//    if (pParameters) pParameters->ReSet();
//    if (pPoints) pPoints->ReSet();
//
//    SmCurveCache sCurveCache(*this,crInterval,FALSE,
//        dChordHeightTolerance, dCrvAngleToleranceDeg,
//        0.0,lMinimumNumberOfSegments,TRUE);
//    sCurveCache.SetContext(NULL);
//    
//    SER(sCurveCache.Tessellate());
//
//    SER(sCurveCache.GetTessellation(pParameters,pPoints));
//    return SM_SUCCESS;
//

} // end SmBSplineCurve::Tessellate

/*******************************************************************//**
PURPOSE: Test a curve to see if it is a possible fillet cross section
  and return TRUE in rbIsFilletCrossSection when curve shape is a known
  fillet cross-section shape.

NOTES:

METHOD --- The test is based solely on the shape of this curve.
  Fillet curves have known shapes - compare this curve
  to those and if any match then return that this curve 
  might be a fillet curve.
  
  Known Fillet Curve shapes are returned in rlFilletCrossSection:
  10     = degenerate curve - no length.
  2      = a Line
  0 or 1 = a circular arc
  G1     = curve is within tolerance of a curve interpolating endPoint positions and tangents
  G2     = curve is within tolerance of a curve interpolating endPoint positions, tangents, and 2nd derivatives.
  G3     = curve is within tolerance of a curve interpolating endPoint positions, tangents, 2nd and 3rd derivatives.
   
***********************************************************************/
SmStatus SmBSplineCurve::TestForFilletCrossSection
  (const SmContext & crContext,                    // in : context for new object construction
   double            d3DTolerance,                 // in : max allowed deviation between curve and classification type 
   SmBoolean       & rbIsFilletCrossSection,       // out: TRUE = might be a fillet cross section
                                                   //      FALSE= Not a fillet cross section
   ULONG           & rlFilletCrossSection,         // out: 0 - circular,   1 - Approx Circular,
                                                   //      2 - Linear      3 - G1 Blend, 
                                                   //      4 - G2 Blend,   5 - G3 Blend
                                                   //     10 - Degenerate
   double          & rdDistance,                   // out: chord length between beg/end curve points
   double          & rdRadius,                     // out: radius of circular and approx circular arcs
   double          & rdBlendScale)                 // out:
    
 const
{
  // init output
  rdBlendScale           = 1.0;
  rbIsFilletCrossSection = FALSE;
  rdRadius               = 0.0;

  // decide - degenerate curves might be fillet-cross sections
  if (IsDegenerate()) 
    {
      rbIsFilletCrossSection = TRUE;
      rlFilletCrossSection   = 10;
      return SM_SUCCESS;
    } // end is degenerate check

  // eval min/max curve points (positions, tangents, and 2nd derivatives)
  SmExtent1d sIvl = GetNaturalInterval();
  SmPoint3d sPVVV[4], sPVVV2[4];
  SER(Evaluate(sIvl.GetMin(),3,TRUE,sPVVV));
  SER(Evaluate(sIvl.GetMax(),3,TRUE,sPVVV2));

  // get chord distance between beg/end curve points
  rdDistance = sPVVV[0].DistanceBetween(sPVVV2[0]);

  // decide - lines might be fillet-cross sections 
  SmPoint3d sPnt, sVec;
  if (IsLine(10,d3DTolerance,sPnt,sVec)) 
    {
      rbIsFilletCrossSection = TRUE;
      rlFilletCrossSection   = 2;
      return SM_SUCCESS;

    } // end isLine check

  // decide - arcs might be fillet-cross sections
  SmAxis2Placement sPlace;
  double dRadius, dStartAngDeg, dEndAngDeg;
  if (IsArc(10,d3DTolerance,sPlace,dRadius,dStartAngDeg,dEndAngDeg)) 
    {
      rbIsFilletCrossSection = TRUE;
      rlFilletCrossSection   = 1; 
      if (IsRational()) {
                          rlFilletCrossSection = 0;
                        }
      rdRadius = dRadius;
      return SM_SUCCESS;
   }

  // locals - arrays of positions, tangents, and 2nd derivatives
  SmTArray<SmPoint3d>  sPnts;
  SmTArray<SmVector3d> sTangents;
  SmTArray<SmVector3d> sHigher;

  // load min point evals
  sPnts.Add(sPVVV[0]);
  sTangents.Add(sPVVV[1]);
  sHigher.Add(sPVVV[2]);

  // load max point evals
  sPnts.Add(sPVVV2[0]);
  sTangents.Add(sPVVV2[1]);
  sHigher.Add(sPVVV2[2]);

  // see if beg/end positions and tangents define a circle
  double dCrvAngleTolDeg = 1.0;
  SmBoolean bIsCircle;
  SER(smgu_CircleFromPointsTangents(sPnts[0],sTangents[0],sPnts[1],sTangents[1],
      dCrvAngleTolDeg,bIsCircle,sPlace,dRadius,dStartAngDeg,dEndAngDeg));
  if (bIsCircle) rdRadius = dRadius;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      Dump();
      smgfx_Erase();
      Draw();
      DrawPolygon();
      sm_GraphicsLoop();
  }
#endif

  // See if it is a G1 Blend Curve
  {
    // create a temporary curve to interpolate the end points and tangents
    SmBSplineCurve *pBlend = NULL ;
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,3,sPnts,sTangents,NULL,FALSE,pBlend));
    SmObjDelete sCleanBlend1(pBlend);
    SmExtent1d sBIvl = pBlend->GetNaturalInterval();
    SmPoint3d sMidPnt, sMyMidPnt;

    // evaluate the mid point on this curve and the interpolated curve
    SER(pBlend->EvaluatePoint(sBIvl.Evaluate(0.5),sMidPnt));
    SER(EvaluatePoint(sIvl.Evaluate(0.5),sMyMidPnt));

    // if midPoints are within tolerance of one another
    if (sMyMidPnt.DistanceBetween(sMidPnt) < d3DTolerance) 
      {
        // get max distance between the two curves
        double dMaxDist;
        SER(CurveMaxDistanceBetween(sIvl,*pBlend,sBIvl.GetMin(),sBIvl.GetMax(),10,NULL,dMaxDist));

        // decide - curve is within tolerance of interpolation - might be a fillet cross section
        if (dMaxDist < d3DTolerance) 
          {
            rbIsFilletCrossSection = TRUE;
            rlFilletCrossSection   = 3;
            return SM_SUCCESS;
          } // end max curve/curve distance check
      }  // end midPoint distance check
  } // end check for g1 blends
  
  // See if it is a G2 Blend Curve
  {
    SmBSplineCurve *pBlend = NULL ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
        if (bDebugMe3) {
            sPnts.Dump();
            sTangents.Dump();
            sHigher.Dump();
        }
#endif
    // create a temporary curve to interpolate end point positions, tangents, and 2nd derivatives
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,5,sPnts,sTangents,&sHigher,FALSE,pBlend));
    SmObjDelete sCleanBlend1(pBlend);
    SmExtent1d sBIvl = pBlend->GetNaturalInterval();

    // get midPoints on this and interpolating curves
    SmPoint3d sMidPnt, sMyMidPnt;
    SER(pBlend->EvaluatePoint(sBIvl.Evaluate(0.5),sMidPnt));
    SER(EvaluatePoint(sIvl.Evaluate(0.5),sMyMidPnt));
    if (sMyMidPnt.DistanceBetween(sMidPnt) < d3DTolerance) 
      {
        // get max curve/curve distance
        double dMaxDist;
        SER(CurveMaxDistanceBetween(sIvl,*pBlend,sBIvl.GetMin(),sBIvl.GetMax(),10,NULL,dMaxDist));

        // when max distance is within tolerance - curve might be a g2 fillet curve
        if (dMaxDist < d3DTolerance)
          {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
            if (bDebugMe4) {
                smgfx_Erase();
                smgfx_SetColor(0,0,0);
                Draw();
                DrawPolygon();
                Dump();
                sm_GraphicsLoop();
                smgfx_SetColor(1,0,0);
                pBlend->Draw();
                pBlend->DrawPolygon();
                pBlend->Dump();
                sm_GraphicsLoop();
            }
#endif

            rbIsFilletCrossSection = TRUE;
            rlFilletCrossSection   = 4;
            return SM_SUCCESS;

          } // end curve/curve distance check
      } // end midPoint distance check
  } // end g2 blend check
  
  // check for G3 blend
  {
    // rebuild sHigher array to hold end point 2nd and 3rd order derivatives 
    sHigher.ReSet();
    sHigher.Add(sPVVV[2]);
    sHigher.Add(sPVVV2[2]);
    sHigher.Add(sPVVV[3]);
    sHigher.Add(sPVVV2[3]);

    // create a temporary curve to interpolate end point positions, tangents, 2nd, and 3rd order derivatives
    SmBSplineCurve *pBlend = NULL ;
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,7,sPnts,sTangents,&sHigher,FALSE,pBlend));
    SmObjDelete sCleanBlend1(pBlend);

    // get this and interpolating curve midPoint positions
    SmExtent1d sBIvl = pBlend->GetNaturalInterval();
    SmPoint3d sMidPnt, sMyMidPnt;
    SER(pBlend->EvaluatePoint(sBIvl.Evaluate(0.5),sMidPnt));
    SER(EvaluatePoint(sIvl.Evaluate(0.5),sMyMidPnt));

    // when midPoints are within tolerance
    if (sMyMidPnt.DistanceBetween(sMidPnt) < d3DTolerance) 
      {
        // get max curve/curve distance
        double dMaxDist;
        SER(CurveMaxDistanceBetween(sIvl,*pBlend,sBIvl.GetMin(),sBIvl.GetMax(),10,NULL,dMaxDist));

        // when max distance is within tolerance - curve might be a G3 fillet curve
        if (dMaxDist < d3DTolerance) 
          {
            rbIsFilletCrossSection = TRUE;
            rlFilletCrossSection   = 5;
            return SM_SUCCESS;
          } // end max distance check
      } // end midPoint distance check
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe5 = FALSE;
    if (bDebugMe5) {
        smgfx_Erase();
        smgfx_SetColor(0,0,0);
        Draw();
        DrawPolygon();
        Dump();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        pBlend->Draw();
        pBlend->DrawPolygon();
        pBlend->Dump();
        sm_GraphicsLoop();
    }
#endif
  } // end G3 blend check
  
  // Not a blend cross section that we know about here - perhaps someone invented a new one
  return SM_SUCCESS;

} // end SmBSplineCurve::TestForFilletCrossSection

/*******************************************************************//**
PURPOSE: Scale, rotate and translate a B-Spline curve.

NOTES: If scale is passed in apply it first then the rotation and
   move defined by the placement.
***********************************************************************/
SmStatus SmBSplineCurve::Transform
  (const SmAxis2Placement & crRotateNMove, // in : affine rotate and move transformation      
   const SmVector3d       * cpOptScale)    // in : optional scaling about current origin point before RotateNMove
                                           //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                           //      other geom types only support isoptropic scaling
{
    SmVector3d sIdentityScale(1, 1, 1);
    
    // no work - identity transform
    if (crRotateNMove.IsIdentity() && (cpOptScale == NULL || *cpOptScale == sIdentityScale))
    {
      return SM_SUCCESS;
    }

    NL_STACKS SC;
    NL_RMATRIX rma;
    SmStackHandler sStackKp(&SC);
    N_InitRealMatrix(&rma);
    N_SetRealMatrix(&rma,3,3,NL_MT_FULL,3,&SC);
    gw_REAL  **RM = rma.RM;

    crRotateNMove.GetMatrix(RM);

    SmVector3d sScale(1,1,1);
    if (cpOptScale) sScale = *cpOptScale;

    RM[0][0] *= sScale.x;
    RM[0][1] *= sScale.y;
    RM[0][2] *= sScale.z;

    RM[1][0] *= sScale.x;
    RM[1][1] *= sScale.y;
    RM[1][2] *= sScale.z;

    RM[2][0] *= sScale.x;
    RM[2][1] *= sScale.y;
    RM[2][2] *= sScale.z;

    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    gw_CURVE *pCur = GetOrCreateGwNurbPointer();
    N_CrvTransform(pCur,&rma);

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmBSplineCurve::Transform

/*******************************************************************//**
PURPOSE: Trim the curve-pair to remove a small gap between them

NOTES: If the two curves do not intersect AND the gap between
     the closest endpoints is less than dMin, then the curves are
     extended and intersected.  When the extended curves intersect
     one or both of the curve end points is moved to eliminate the gap.

     If CrvToFix = 0, change both curve end points to extended curve intersection point
     If CrvToFix = 1, change only pCurve1 endpoint to 'this' curve initial endPoint
     If CrvToFix = 2, change only 'this' curve  endpoint to pCurve1 initial endPoint

RETURNS:  0 = gap is too large to fix or extended curves fail to intersect
          1 = success the curve(s) either start out with no gap or have been modified to remove a gap
***********************************************************************/
int SmBSplineCurve::TrimGap
  (SmBSplineCurve * pCurve1,   // i/o: curve whose endPoint might be moved
   int              CrvToFix,  // in : 0 = change both curve end points to XSect of extended curves
                               //      1 = change only pCurve1 endpoint to 'this' curve initial endPoint
                               //      2 = change only 'this' curve  endpoint to pCurve1 initial endPoint
   double           dMin)      // in : max 3d gap size to be closed
{
  // locals
  SmBSplineCurve * pCurve0 = this;
  SmContext        Context; // use local context for extended curves
  SmSolutionArray  sSolutions;
  SmPoint3d        sP0, sP1;

  // only close gap if less than dMin
  double dMinSq = dMin*dMin;

  // curve 0 start and end points 
  SmPoint3d  S0, E0;
  SmExtent1d sExtent0 = pCurve0->GetNaturalInterval();
  double     t0size   = sExtent0.GetMax() - sExtent0.GetMin();
  pCurve0->EvaluatePoint(sExtent0.GetMin(), S0);
  pCurve0->EvaluatePoint(sExtent0.GetMax(), E0);

  // curve 1 start and end points
  SmPoint3d S1, E1;
  SmExtent1d sExtent1 = pCurve1->GetNaturalInterval();
  double     t1size   = sExtent1.GetMax() - sExtent1.GetMin();
  pCurve1->EvaluatePoint(sExtent1.GetMin(), S1);
  pCurve1->EvaluatePoint(sExtent1.GetMax(), E1);
    
  // Extend curves based on closest endpoints
  // by domain * 0.3
  double dS0S1 =(S0 - S1).LengthSquared();
  double dS0E1 =(S0 - E1).LengthSquared();
  double dE0S1 =(E0 - S1).LengthSquared();
  double dE0E1 =(E0 - E1).LengthSquared(); 
  double dmin  = smos_4Min( dS0S1, dS0E1, dE0S1, dE0E1) ;

  // Test if Gap is greater than dMin
  if (dmin > dMinSq)      return(0);  // gap is too large to fix
  if (dmin < SM_EFF_ZERO) return(1);  // curves have no gap

  // figure out which end points to move
  int ExtCase =   ( dmin >= dS0S1) ? 1
                : ( dmin >= dS0E1) ? 2
                : ( dmin >= dE0S1) ? 3
                : ( dmin >= dE0E1) ? 4
                : 0 ;

  SM_ASSERT(ExtCase != 0) ;
  
  // extend the input curves past the ends with the smallest gaps  
  SmBSplineCurve *pCurve0E = NULL,  *pCurve1E = NULL;
  SmObjDelete     sClean0(pCurve0E), sClean1(pCurve1E) ;

  if (ExtCase == 1 || ExtCase == 2)
    {
      double t = sExtent0.GetMin() - 0.03*t0size;
      pCurve0->CreateExtendedCurve(Context, t, SM_CT_G1, pCurve0E, 1);
    }
  if (ExtCase == 3 || ExtCase == 4)
    {
      double t = sExtent0.GetMax() + 0.03*t0size;
      pCurve0->CreateExtendedCurve(Context, t, SM_CT_G1, pCurve0E, 1);
    }
  if (ExtCase == 1 || ExtCase == 3)
    {
      double t = sExtent1.GetMin() - 0.03*t1size;
      pCurve1->CreateExtendedCurve(Context, t, SM_CT_G1, pCurve1E, 1);
    }
  if (ExtCase == 2 || ExtCase == 4)
    {
      double t = sExtent1.GetMax() + 0.03*t1size;
      pCurve1->CreateExtendedCurve(Context, t, SM_CT_G1, pCurve1E, 1);
    }

  // intersect the extended curves
  pCurve0E->GlobalCurveIntersect(pCurve0E->GetNaturalInterval(),
                                *pCurve1E,  
                                 pCurve1E->GetNaturalInterval(),
                                 dmin*5.0, 
                                 sSolutions);

  // failure - the extended curves don't intersect
  if ( sSolutions.GetSize() < 1 ) 
    { return (0); }

  // for every intersection
  for (ULONG ii = 0; ii < sSolutions.GetSize(); ii++)
    {
      if (sSolutions[ii].m_eSolutionType == SM_ST_SINGLE_VALUE)
        {
          // locals
          double     t0       = sSolutions[ii].m_vStart[0];   // 1st curve intersection param
          double     t1       = sSolutions[ii].m_vStart[1];   // 2nd curve intersection param
          sExtent0    = pCurve0E->GetNaturalInterval();           
          sExtent1    = pCurve1E->GetNaturalInterval();

          //Trim the domain of the two curves to the intersection
          double tmin0 = (ExtCase == 1 || ExtCase == 2) ? t0 : sExtent0.GetMin() ;
          double tmax0 = (ExtCase == 3 || ExtCase == 4) ? t0 : sExtent0.GetMax() ;
          double tmin1 = (ExtCase == 1 || ExtCase == 3) ? t1 : sExtent1.GetMin() ;
          double tmax1 = (ExtCase == 2 || ExtCase == 4) ? t1 : sExtent1.GetMax() ;

          SmExtent1d sExt0(tmin0, tmax0);    
          SmExtent1d sExt1(tmin1, tmax1);    
          pCurve0E->Trim(sExt0) ;  // may snap sIvl by tol to existing knots
          pCurve1E->Trim(sExt1) ;  // may snap sIvl by tol to existing knots

          // only do this once
          break ;

        } // end intersection is a point intersection check
    } // end iter every curve intersection

  // arrive here after the extended curves have been trimmed to a common intersection point

  // Now lets average the two trimmed endpoints
  sExtent0 = pCurve0E->GetNaturalInterval();  // after trimming         
  sExtent1 = pCurve1E->GetNaturalInterval();

  if (ExtCase == 1 || ExtCase == 2) { pCurve0E->EvaluatePoint(sExtent0.GetMin(), sP0) ; }
  if (ExtCase == 3 || ExtCase == 4) { pCurve0E->EvaluatePoint(sExtent0.GetMax(), sP0) ; }
  if (ExtCase == 1 || ExtCase == 3) { pCurve1E->EvaluatePoint(sExtent1.GetMin(), sP1) ; }
  if (ExtCase == 2 || ExtCase == 4) { pCurve1E->EvaluatePoint(sExtent1.GetMax(), sP1) ; }

  // Compute midpoint from 2 closest points from extended curves
  // When CrvToFix == 0, move both curve endPoints to the extended curve intersection point
  //      CrvToFix == 1, move both curve endPoints to Curve0 initial end point
  //      CrvToFix == 2, move both curve endPoints to Curve1 initial end point
  SmPoint3d SmMid =   (CrvToFix ==  1) ? ((ExtCase == 1 || ExtCase == 2) ? S0 : E0)
                    : (CrvToFix ==  2) ? ((ExtCase == 1 || ExtCase == 3) ? S1 : E1)
                    : 0.5 * (sP0 + sP1) ;

  // replace the endpoint of the original curves
  if( ExtCase == 1 || ExtCase == 2) { pCurve0->SetControlPoint(SM_CP_NON_RATIONAL, 0, SmMid, 1.0) ; }
  if( ExtCase == 3 || ExtCase == 4) { pCurve0->SetControlPoint(SM_CP_NON_RATIONAL, pCurve0->GetNumberControlPoints() - 1, SmMid, 1.0) ; }
  if( ExtCase == 1 || ExtCase == 3) { pCurve1->SetControlPoint(SM_CP_NON_RATIONAL, 0, SmMid, 1.0) ; }
  if( ExtCase == 2 || ExtCase == 4) { pCurve1->SetControlPoint(SM_CP_NON_RATIONAL, pCurve1->GetNumberControlPoints() - 1, SmMid, 1.0) ; }

  return 1; // success...we have modified the curves

} // end SmBSplineCurve::TrimGap

/*******************************************************************//**
PURPOSE: Adjust the curve to the given STEP-interval.

NOTES: Since B-Spline's parametrization is STEP-based.
    Simply call the original Trim or CreateExtendedCurve() methods
    as needed.
***********************************************************************/
SmStatus SmBSplineCurve::AdjustSTEPInterval
  (const SmExtent1d & crNewSTEPInterval)
{
  SmExtent1d sIvl = GetNaturalInterval();

  // See if it needs extension.
  double dTol = SM_EFF_ZERO * ( 1 + sIvl.GetMax() );

  if (      m_pNurb != NULL
       && ! crNewSTEPInterval.IsContainedBy( sIvl, dTol ))
    {
      // Start of curve:
      double dParam = crNewSTEPInterval.GetMin();
      SmBSplineCurve *pNewBSplCrv = NULL;
      SmStatus eStat;
      const SmContext *pContext = GetContext();
      if ( ! sIvl.ContainsValue( dParam, dTol ) )
        {
          eStat = CreateExtendedCurve( *pContext, dParam, SM_CT_CINFINITY, pNewBSplCrv, TRUE );

          if ( eStat == SM_SUCCESS && pNewBSplCrv != NULL && pNewBSplCrv->m_pNurb != NULL )
            {
              Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
              // *this = *pNewBSplCrv;
              if ( m_pNurb != NULL ) { smos_Free(m_pNurb); }
              m_pNurb = pNewBSplCrv->m_pNurb;
              pNewBSplCrv->m_pNurb = NULL;
              delete pNewBSplCrv;
              pNewBSplCrv = NULL;
              Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
            }
        }

      // End of curve:
      dParam = crNewSTEPInterval.GetMax();
      if ( ! sIvl.ContainsValue( dParam, dTol ) )
        {
          eStat = CreateExtendedCurve( *pContext, dParam, SM_CT_CINFINITY, pNewBSplCrv, TRUE );

          if ( eStat == SM_SUCCESS && pNewBSplCrv != NULL && pNewBSplCrv->m_pNurb != NULL )
            {
              Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
              // *this = *pNewBSplCrv;
              if ( m_pNurb != NULL ) { smos_Free(m_pNurb); }
              m_pNurb = pNewBSplCrv->m_pNurb;
              pNewBSplCrv->m_pNurb = NULL;
              delete pNewBSplCrv;
              pNewBSplCrv = NULL;
              Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
            }
        }
    } // end if need to extend curve


  // Now trim if necessary.
  sIvl = crNewSTEPInterval;
  SER( Trim( sIvl ));

  return SM_SUCCESS;

} // end SmBSplineCurve::AdjustSTEPInterval

/*******************************************************************//**
PURPOSE: Write SmBSplineCurve to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  gw_REAL    wx, wy, wz, w;

  SmFileType eType = rDB.GetFileType();
  gw_CURVE  *cur  = ((SmBSplineCurve *)this)->GetOrCreateGwNurbPointer() ;
  SER(cur != NULL ? SM_SUCCESS : SM_ERR) ;

  // m_pNurb locals 
  gw_INDEX   i, n, m;
  gw_DEGREE  p;
  gw_CPOINT  *Pw;
  gw_REAL    *U ; 
  N_CrvGetCPtsDegreeAndKnots(cur,&n,&Pw,&p,&m,&U);
  ULONG lKnotType         = (ULONG) m_eKnotType ;
  ULONG lBSplineCurveForm = (ULONG) m_eBSplineCurveForm ;

  // set type flag for different types of output 
  gw_FLAG rat = N_IsCrvRat(cur) ? 1 : 0 ;
  gw_FLAG dim = N_CrvIs3d(cur)  ? 3 : 2 ;
  gw_FLAG type = (rat EQ 0) ? ((dim EQ 2) ? 1 : 2)
                            : ((dim EQ 2) ? 3 : 4) ;

  // Create the output file 

  // output header, n, p, rat, and dim
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << n   << " Number of Control Points \n"; 
      rFileOut << p   << " Degree \n";                   
      rFileOut << rat << " RationalFlag \n";             
      rFileOut << dim << " Dimension \n";                

      if(lDBVersionNumber > 32)
        {
          rFileOut << lKnotType         << " Knot Type Enum: 1=2d NonRational, 2=3d NonRational, 3=2d Rational, 4=3d Rational\n";
          rFileOut << lBSplineCurveForm << " BSplineCurveForm Enum\n";
        }
    }
  else // Binary
    {
      SER(rDB.WriteLong(n));
      SER(rDB.WriteShort(p));
      SER(rDB.WriteShort(rat));
      SER(rDB.WriteShort(dim));

      if(lDBVersionNumber > 32)
        {
          SER(rDB.WriteLong(lKnotType));
          SER(rDB.WriteLong(lBSplineCurveForm));
        }
    }

  // switch on curve type
  //  type = 1: 2d non-rational
  //  type = 2: 3d non-rational
  //  type = 3: 2d rational
  //  type = 4: 3d rational
  switch( type )
    {
      // 2-D non-rational
      case 1 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     N_CPtToXYZ(Pw[i],&wx,&wy,&wz);
                     if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                                              rFileOut << wx << " " << wy << "\n";    
                                            }
                     else /* Binary */      { SER(rDB.WriteDouble(wx));
                                              SER(rDB.WriteDouble(wy));
                                            }
                   } // end iter every control point
               } break;

       // 3-D non-rational 
      case 2 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     N_CPtToXYZ(Pw[i],&wx,&wy,&wz);
                     if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                                              rFileOut << wx << " " << wy << " " << wz << "\n";    
                                            }
                     else /* Binary */      { SER(rDB.WriteDouble(wx));
                                              SER(rDB.WriteDouble(wy));
                                              SER(rDB.WriteDouble(wz));
                                            }
                   } // end iter every control point
               } break;

      // 2-D rational 
      case 3 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     N_CPtToWxWyWz(Pw[i],&wx,&wy,&wz,&w);
                     if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                                              rFileOut << wx << " " << wy << " " << w << "\n";    
                                            }
                     else /* Binary */      { SER(rDB.WriteDouble(wx));
                                              SER(rDB.WriteDouble(wy));
                                              SER(rDB.WriteDouble(w));
                                            }
                   } // end iter every control point
               } break;

      // 3-D rational 
      case 4 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     N_CPtToWxWyWz(Pw[i],&wx,&wy,&wz,&w);
                     if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                                              rFileOut << wx << " " << wy << " " << wz << " " << w << "\n";    
                                            }
                     else  /* Binary */     { SER(rDB.WriteDouble(wx));
                                              SER(rDB.WriteDouble(wy));
                                              SER(rDB.WriteDouble(wz));
                                              SER(rDB.WriteDouble(w));
                                            } 
                   } // end iter every control point
               } break;

      // Wrong type 
      default : SER(SM_ERR) ; 

    } // end switch on curve type outputing control points

  // for every knot 
  for( i=0; i<=m; i++ )
    {
      if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                               rFileOut << U[i] << "\n";    
                             }
      else /* Binary */      { SER(rDB.WriteDouble(U[i]));
                             }
    } // end iter every knot   

  // all done
  return SM_SUCCESS ;

} // end SmBSplineCurve::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmBSplineCurve from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmBSplineCurve::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmBSplineCurve_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmBSplineCurve *pBSplineCurve =   (rpNewCurve == NULL)
                                  ? new (crContext) SmBSplineCurve(lDim)
                                  : (SmBSplineCurve *)rpNewCurve ;
  // file type
  SmFileType eType = rDB.GetFileType();

  // locals
  gw_REAL    wx, wy, wz, w;

  // Get parameters from the top 
  gw_INDEX   i, n ;
  gw_DEGREE  p;
  gw_FLAG    lReadDim, rat;
  ULONG lKnotType         = SM_KT_UNSPECIFIED ;
  ULONG lBSplineCurveForm = SM_CF_UNSPECIFIED ;
  if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                           rFileIn >> n   ;      rDB.GoToNextLine() ;
                           rFileIn >> p   ;      rDB.GoToNextLine() ;
                           rFileIn >> rat ;      rDB.GoToNextLine() ;
                           rFileIn >> lReadDim ; rDB.GoToNextLine() ;
                           if(lDBVersionNumber > 32)                                           
                             {
                               rFileIn >> lKnotType ; rDB.GoToNextLine() ;
                               rFileIn >> lBSplineCurveForm ; rDB.GoToNextLine() ;
                             }
                         }
  else /* Binary */      { SER(rDB.ReadLong(n));
                           SER(rDB.ReadShort(p));
                           SER(rDB.ReadShort(rat));
                           SER(rDB.ReadShort(lReadDim));
                           if(lDBVersionNumber > 32)
                              {
                                SER(rDB.ReadLong(lKnotType));
                                SER(rDB.ReadLong(lBSplineCurveForm));
                              }
                         }
  // GWC: [prog_test SmCurve::ReadFromFile("C:\\TestFiles\\pt_TestFiles\\Drop\\PS_LiftCrv_Crv1.dat")]
  //  works when lDim == 3 and lReadDim == 2 so removing this check - its not needed.
  //      if(lDim != (ULONG)lReadDim)
  //        {
  //          // 2D curves used to written out as 3d curves with an ignored z value.
  //          // ignore those cases here
  //          if(lDim != 2 || lReadDim != 3)
  //            {
  //              SM_ASSERT_MSG(lDim == (ULONG)lReadDim,
  //                            _T("Warning: Curve Read lReadDim value not equal to called lDim value. Not a problem for older files")) ;
  //            }
  //        }

  // knot count
  gw_INDEX m = n+p+1;

  // alloc m_pNurb memory 
  gw_CURVE *cur = sm_AllocateNurbCurve(n,p,m);
  if(cur == NULL)
    { SER(SM_ERR) ; }

  // curve locals
  gw_CPOINT  *Pw;
  gw_REAL    *U ; 
  N_CrvGetCPtsAndKnots(cur,&Pw,&U);

  // Get different types of input 
  gw_FLAG type = (rat EQ NL_NO) ? ((lReadDim EQ 2) ? 1 : 2)
                                : ((lReadDim EQ 2) ? 3 : 4) ;

  // Read in data 
  switch( type )
    {
      // 2-D non-rational
      case 1 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                              rFileIn >> wx >> wy;
                                              rDB.GoToNextLine() ;
                                            }
                     else /* Binary */      { SER(rDB.ReadDouble(wx));
                                              SER(rDB.ReadDouble(wy));
                                            }
                     N_CPtFromWxWyWz(wx,wy,NL_NOZ,NL_NOW,&Pw[i]);
                   }
               } break ;

      // 3-D non-rational
      case 2 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                              rFileIn >> wx >> wy >> wz;
                                              rDB.GoToNextLine() ;
                                            }
                     else /* Binary */      { SER(rDB.ReadDouble(wx));
                                              SER(rDB.ReadDouble(wy));
                                              SER(rDB.ReadDouble(wz));
                                            }
                     N_CPtFromWxWyWz(wx,wy,wz,NL_NOW,&Pw[i]);
                   }
               } break ;

      // 2-D rational
      case 3 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                              rFileIn >> wx >> wy >> w;
                                              rDB.GoToNextLine() ;
                                            }
                     else /* Binary */      { SER(rDB.ReadDouble(wx));
                                              SER(rDB.ReadDouble(wy));
                                              SER(rDB.ReadDouble(w));
                                            }
                     N_CPtFromWxWyWz(wx,wy,NL_NOZ,w,&Pw[i]);
                   }
               } break ;

      // 3-D rational
      case 4 : { // for every control point
                 for( i=0; i<=n; i++ )
                   {   
                     if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                              rFileIn >> wx >> wy >> wz >> w;
                                              rDB.GoToNextLine() ;
                                            }
                     else /* Binary */      { SER(rDB.ReadDouble(wx));
                                              SER(rDB.ReadDouble(wy));
                                              SER(rDB.ReadDouble(wz));
                                              SER(rDB.ReadDouble(w));
                                            }
                     N_CPtFromWxWyWz(wx,wy,wz,w,&Pw[i]);
                   }
               } break ;

     // Wrong type
      default : SER(SM_ERR) ; 

    } // end switch on curve rat/lReadDim types

  // for every knot
  for( i=0; i<=m; i++ )
    {
      if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                               rFileIn >> U[i];
                               rDB.GoToNextLine() ;
                             }
      else /* Binary */      { SER(rDB.ReadDouble(U[i]));
                             }
    }    

  // Set output
  pBSplineCurve->m_pNurb               = cur ;
  pBSplineCurve->m_bNurbIsBorrowed     = FALSE ;
  pBSplineCurve->m_bOutOfBoundsEnabled = FALSE ;
  pBSplineCurve->m_eKnotType           = (SmKnotType        ) lKnotType ;
  pBSplineCurve->m_eBSplineCurveForm   = (SmBSplineCurveForm) lBSplineCurveForm ;
  pBSplineCurve->m_lDim                = (ULONG)lReadDim ;

  rpNewCurve = pBSplineCurve ;

  // fix 2d curve representations
  if (lDim == 2 && lReadDim == 3) 
    {
      SER(rpNewCurve->ConvertTo2D());
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmBSplineCurve::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmBSplineCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmBSplineCurve_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:   Dump preceded by a one line message

NOTES:
***********************************************************************/
void SmBSplineCurve::Dump
  (const TCHAR * message) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%s "), message);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmBSplineCurve::Dump

/*******************************************************************//**
PURPOSE:  Dump preceded by an integer (ULONG)

NOTES:
***********************************************************************/
void SmBSplineCurve::Dump
  (ULONG i) 
 const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     smos_sprintf(sBuff,_T("\n%ld "),i);
     smos_WriteBuffer(sBuff);
     this->Dump();

} // end SmBSplineCurve::Dump

/*******************************************************************//**
PURPOSE: Dump bspline curve data out for debugging.
            Dump is in similar format to Write to file

NOTES: 
***********************************************************************/

void SmBSplineCurve::Dump
  (void) 
const
{
     this->Dump(FALSE);
} // end SmBSplineCurve::Dump

/*******************************************************************//**
PURPOSE: Dump with abbreviated option

NOTES: if bAbbrev = TRUE just dump 1st and last control points
         and knots
***********************************************************************/
void SmBSplineCurve::Dump
  (SmBoolean bAbbrev)  // in : TRUE = dump 1st and last control points and knots
 const                 //      FALSE= dump all control points and knots
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmBSplineCurve::Dump()")) ;

  // ouput cache data
  SmCurve::Dump(FALSE) ;

  smos_sprintf(sBuff,_T("\nSmBSplineCurve:[0x%p]"),this);
  smos_sprintf(sBuffForFile,_T("\nSmBSplineCurve:[%s]"),_T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  if ( !m_pNurb )
    { smos_sprintf(sBuff,_T("%s"), _T(", m_pNurb = NULL no more to dump."));
      smos_WriteBuffer(sBuff);
      return; 
    }

  // pretty print the gw_CURVE
  gw_CURVE *pCur = GetGwNurbPointer();
  Dump_NCrv(pCur, bAbbrev) ;
  if(pCur == NULL) return ;

  // if rational: print start and end point evaluations
  if(N_IsCrvRat(pCur))
    {
      SmExtent1d sDomain0 = GetNaturalInterval();
      SmPoint3d StartPoint, EndPoint;
      EvaluatePoint(sDomain0.GetMin(), StartPoint);
      EvaluatePoint(sDomain0.GetMax(), EndPoint);
      smos_sprintf(sBuff,_T("\nStart: %6.16lf %6.16lf %6.16lf\n"), StartPoint.x, StartPoint.y, StartPoint.z);
      smos_WriteBuffer(sBuff);
      smos_sprintf(sBuff,_T("End  : %6.16lf %6.16lf %6.16lf\n"), EndPoint.x, EndPoint.y, EndPoint.z);
      smos_WriteBuffer(sBuff);
    } // end if rational check

  smos_WriteBuffer(_T("End SmBSplineCurve::Dump()\n")) ;

} // end SmBSplineCurve::Dump

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmBSplineCurve.

NOTES:  Does include the Curve's attribute memory
***********************************************************************/
ULONG SmBSplineCurve::GetMemoryUsed  // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,     // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)             // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + m_pNurb memory
  rlMemoryAllocated = sizeof(*this) + sm_ComputeNurbCurveSize(m_pNurb) ;
  
  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }
  
  // all done
  return(lUsed) ;

} // end SmBSplineCurve::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels

NOTES:
   9 and 10 are unused, as of 4/18/2013.
***********************************************************************/
SmAssertReportLabel sAssertBSplineCurve_list[] =
{
 /*  0 */ {SM_AT_POINTER,   _T("Bad Null Ptr"),        _T("BSplineCurve m_pNurb ptr is NULL") },
 /*  1 */ {SM_AT_KNOTS,     _T("Bad Knot Count"),      _T("BSplineCurve Knot Vector Start and End knots do not have degree+1 multiplicity") },
 /*  2 */ {SM_AT_KNOTS,     _T("Bad Knot Count"),      _T("BSplineCurve Knot Vector Internal knot has multiplicity greater than degree") },
 /*  3 */ {SM_AT_GEOMETRIC, _T("Warn Coincident CPts"), _T("BSplineCurve's 1st two control points are coincident - not an error by itself, but commonly part of a problem.") },
 /*  4 */ {SM_AT_GEOMETRIC, _T("Warn Coincident CPts"), _T("BSplineCurve's last two control points are coincident - not an error by itself, but copmmonly part of a problem.") },

 /*  5 */ {SM_AT_GEOMETRIC, _T("Bad SpeedPt"),   _T("2D BSplineCurve Warning: zero-length derivative at interior multiple knot") },
 /*  6 */ {SM_AT_GEOMETRIC, _T("Bad SpeedPt"),   _T("3D BSplineCurve Warning: zero-length derivative at interior multiple knot") },
 /*  7 */ {SM_AT_GEOMETRIC, _T("Bad CurveKink"), _T("2D BSplineCurve Warning: Kink at interior knot") },
 /*  8 */ {SM_AT_GEOMETRIC, _T("Bad CurveKink"), _T("3D BSplineCurve Warning: Kink at interior knot") },

 /*  9 */ {SM_AT_GEOMETRIC, _T("Bad CurveCusp"), _T("2D BSplineCurve ERROR: Curve kinks back on itself at interior knot") },
 /* 10 */ {SM_AT_GEOMETRIC, _T("Bad CurveCusp"), _T("3D BSplineCurve ERROR: Curve kinks back on itself at interior knot") },
 /* 11 */ {SM_AT_GEOMETRIC, _T("Bad SpeedPt"),   _T("2D BSplineCurve Warning: Interior 1st Deriv too slow, less than AvgSpeed/10000 (often an almost kink or almost singular point - never good practice)") },
 /* 12 */ {SM_AT_GEOMETRIC, _T("Bad SpeedPt"),   _T("3D BSplineCurve Warning: Interior 1st Deriv too slow, less than AvgSpeed/10000 (often an almost kink or almost singular point - never good practice)") },

 /* 13 */ {SM_AT_GEOMETRIC, _T("Bad CurveCusp"),     _T("2D BSplineCurve Warning: Extremely small radius of curvature (often an almost kink point - sometimes what's desired)") },
 /* 14 */ {SM_AT_GEOMETRIC, _T("Bad CurveCusp"),     _T("3D BSplineCurve Warning: Extremely small radius of curvature (often an almost kink point - sometimes what's desired)") },
 /* 15 */ {SM_AT_GEOMETRIC, _T("Bad Repeated CPts"), _T("BSplineCurve has Internal Coincident ControlPoints (causes internal point singularity)") },
 /* 16 */ {SM_AT_KNOTS,     _T("Bad BSpline Rep"),   _T("Number of knots inconsistent with number of ControlPoints and Curve degree") },
} ;                                                     
      
/*******************************************************************//**
PURPOSE: Check the validity of an SmBSplineCurve.
      
NOTES: Checks made:
 - Make sure m_pNurb is not NULL by calling MakeNurb() when needed.
   (Derived types, like SmLine and SmCircle, delay making the m_pNurb
   until needed.)
 - knots with multiplicity > degree;
 - kinks at multiple knots
 - zero first derivative at multiple knots
 - vanishing first derivatives anywhere
 - tiny radius of curvature anywhere

 Note: can't add an optional argument to this because it's virtual:
       would have to add it to every class's method.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmBSplineCurve::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
#ifdef SM_DEBUG_CODE // see also draw block at method exit
SmBoolean bDebugMe = FALSE ;
#endif // no SM_DEBUG_CODE

  // check base class validity
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmCurve::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // build m_pNurb when not available - this is a side effect!
  if (m_pNurb == NULL) 
    {
      SmBSplineCurve *pBSC = SM_CONST_CAST(SmBSplineCurve*,this);
      pBSC->MakeNurb();
    }

  /*  0 */ // check for nonNULL m_pNurb pointer
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pNurb != NULL ), _T("") ) ;

  if( m_pNurb == NULL )
    { 
      return(bRtn) ;
    }

  // Check knot vector
  ULONG idx ;
  SmTArray<double> sKnots;
  SmTArray<ULONG>  sMults;
  GetKnots( sKnots, &sMults );
  ULONG lNumKnots = sKnots.GetSize();
  SM_ASSERT( lNumKnots == sMults.GetSize() );

  /*  1 */ // check end knots for expected multiplicity count == Degree + 1
  ULONG lDeg = GetDegree();
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (sMults[0] == lDeg+1 && sMults[lNumKnots-1] == lDeg+1), _T("") ) ;
  if( bRtn == FALSE )
    { 
      return( bRtn ); 
    }
  
  /*  2 */ // check internal knots for multiplicty count limit <= Degree  
  for ( idx = 1; idx < lNumKnots - 1; idx++ ) 
    { bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (sMults[idx] <= lDeg), _T("") ) ; }

  if( bRtn == FALSE )
    { 
      return( bRtn ); 
    }

  /* 16 */ // check number of knots consistent with number of ControlPoints and degree
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(16, SM_LEVEL_0, (   m_pNurb == NULL
                                                    || (   (  m_pNurb->knt->m + 1) // number of knots
                                                        == (  m_pNurb->pol->n + 1  // = number of control pts
                                                            + m_pNurb->p + 1))),   // + degree + 1    
                                                   _T("") ) ;
  if( bRtn == FALSE )                              
    { 
      return( bRtn ); 
    }

  SmStatus eStat1, eStat2;

  // check for coincident end control points when the curve is not degenerate
  SmBoolean bDegenerate = SmTol::IsDegenerate(this) ; 
  if(!bDegenerate)
    {
      // for now - allow endPair ControlPoint coincidence, but output a warning
  
      // check for start ControlPoint coincidence
      double dStartCPtDist ;
      SmVector3d sEuclid ;
      N_CPtToPtEuclid(m_pNurb->pol->Pw[0], (NL_POINT *)&sEuclid) ;
      double dStartScaledZero = SM_EFF_ZERO * (1.0 + sEuclid.GetMaxDimension()) ; 
      N_DistCptCpt( m_pNurb->pol->Pw[0], m_pNurb->pol->Pw[1], &dStartCPtDist) ;

      if(SM_IS_ZERO_TO_TOL(dStartCPtDist, dStartScaledZero))
        {
          /*  3 */
          bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, !SM_IS_ZERO_TO_TOL(dStartCPtDist, dStartScaledZero), dStartScaledZero, dStartCPtDist, _T("")) ;
          WARN(_T("BSplineCurve's 1st two control points are coincident - not an error by itself, but commonly part of a problem.\n")) ;
        }

#ifdef SM_DEBUG_CODE
      if(bDebugMe) // see also draw block at method exit
        {
          this->Dump() ;
        }
#endif // SM_DEBUG_CODE

      // check for end ControlPoint coincidence
      NL_INDEX iN = m_pNurb->pol->n ;
      double dEndCPtDist ;
      N_CPtToPtEuclid(m_pNurb->pol->Pw[iN], (NL_POINT *)&sEuclid) ;
      double dEndScaledZero = SM_EFF_ZERO * (1.0 + sEuclid.GetMaxDimension()) ; 
      N_DistCptCpt( m_pNurb->pol->Pw[iN - 1], m_pNurb->pol->Pw[iN], &dEndCPtDist) ;
      if(SM_IS_ZERO_TO_TOL(dEndCPtDist, dEndScaledZero))
        {

#ifdef SM_DEBUG_CODE
          // cull repeated notifications
static    SmTArray<const SmCurve *> sBadCurveList ;
static constexpr ULONG sListLength = 6;
static       ULONG lFoundItem ;
          if(!sBadCurveList.FindElement(this, lFoundItem))
            {
              bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, !SM_IS_ZERO_TO_TOL(dStartCPtDist, dStartScaledZero), dStartScaledZero, dStartCPtDist, _T("")) ;
              WARN(_T("BSplineCurve's last two control points are coincident - not an error by itself, but commonly part of a problem.\n")) ;
              if(sBadCurveList.GetSize() == sListLength) { sBadCurveList.RemoveLast() ; }
              sBadCurveList.InsertAt(0, this) ; 
            }
#else // no SM_DEBUG_CODE
              bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, !SM_IS_ZERO_TO_TOL(dStartCPtDist, dStartScaledZero), dStartScaledZero, dStartCPtDist, _T("")) ;
              WARN(_T("BSplineCurve's last two control points are coincident - not an error, but not good practice.\n")) ;
#endif // no SM_DEBUG_CODE

        }

    } // end not degenerate check

  // Check for kinks.  Check at internal knot multiplicities.
  // We'll flag an error for angles > SM_CONTINUITY_ANGLE.
  // We'll accept degenerate curves

  // skip kink and zero internal 1st derive checks for degenerate curves
  // and for analytic curves.
  SmBoolean bAnalytic = IsAnalytic();

  // for debug - data at failing tests
  double dMinRadius         =  SM_BIG_DOUBLE ;
  double dMinRadiusParam    = -SM_BIG_DOUBLE ;
  double dMin1stDerivParam  = -SM_BIG_DOUBLE ;
#ifdef SM_DEBUG_CODE
  double dMinSinAngRadParam = -SM_BIG_DOUBLE ;
#endif
  double dMaxSinAngRadParam = -SM_BIG_DOUBLE ;      
  if(!bDegenerate && !bAnalytic)
    {

      double dCosineTol = 0.9998476951563913; // cosine of SM_CONTINUITY_ANGLE (1.0 degree)
      SmVector3d pV1[2], pV2[2];
      SmVector3d sCross;

      // for non degenerate curves
      for ( idx = 1; idx < lNumKnots - 1 && bRtn ; idx++ )
        {
          // skip knots with low mults that guarantee C1 or better continuity
          if ( sMults[idx] < lDeg )
            { continue; }

          // Lower interval evaluation
          double dParam = sKnots[idx];               
          Evaluate( dParam, 1, FALSE, pV1 ); // FALSE = evaluate P in lower interval where P is on the right of the interva     
          SmVector3d pV1Dir = pV1[1] ;          
          eStat1 = pV1[1].Unitize(FALSE);
          if ( eStat1 != SM_SUCCESS )                          
            {                                                    
              if(GetDim() == 2)
                {
                  /*  5 */ 
                  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, eStat1 == SM_SUCCESS, SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmVector2d(dParam,0.0)) ;
                  SM_ASSERT_SET_VALUE(pV1Dir.Length()) ; 
                  // SM_ASSERT_MSG( FALSE, _T("2D Curve Warning: zero-length derivative at interior multiple knot") ); 
                }             
              if(GetDim() == 3)
                { 
                  /*  6 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, eStat1 == SM_SUCCESS, SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmVector2d(dParam,0.0)) ;
                  SM_ASSERT_SET_VALUE(pV1Dir.Length()) ; 
                  // SM_ASSERT_MSG( FALSE, _T("3D Curve Warning: zero-length derivative at interior multiple knot") ); 
                }
#ifdef SM_DEBUG_CODE
              if(bDebugMe) // see also draw block at method exit
                {
                  this->Dump() ;
                }
#endif // SM_DEBUG_CODE

              // skip further checks - only need to report the 1st problem
              continue;
            } // end Lower Eval 1st Deriv is zero check

          // upper interval eval
          Evaluate( dParam, 1, TRUE,  pV2 );  // TRUE  = evaluate P in upper interval where P is on the left of the interval 
          SmVector3d pV2Dir = pV2[1] ;
          eStat2 = pV2[1].Unitize(FALSE);
          if ( eStat2 != SM_SUCCESS )
            {    
              if(GetDim() == 2)
                {
                  /*  5 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, eStat2 == SM_SUCCESS, SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmVector2d(dParam,0.0)) ;
                  SM_ASSERT_SET_VALUE(pV2Dir.Length()) ; 
                  // SM_ASSERT_MSG( FALSE, _T("2D Curve Warning: zero-length derivative at interior multiple knot" )); 
                }
              if(GetDim() == 3)
                {
                  /*  6 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, eStat2 == SM_SUCCESS, SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmVector2d(dParam,0.0)) ;
                  SM_ASSERT_SET_VALUE(pV2Dir.Length()) ; 
                  // SM_ASSERT_MSG( FALSE, _T("3D Curve Warning: zero-length derivative at interior multiple knot" ));  
                }
              bRtn = FALSE;
              
              // skip further checks - only need to report the 1st problem
              continue;
            } // end upper Eval 1st deriv degenerate check

          // arrive here when lower and upper interval 1st derivatives are NonDegenerate and unitized
          // Check the angle between 1st derivatives
          double dDot = pV1[1].Dot( pV2[1] );

          // when angle between lower and upper ivl 1st Derivs is too big - curve is not G1
          if ( dDot < dCosineTol )
            {
              dMaxSinAngRadParam = dParam ;

              // double dAngTolRad = SM_DEG2RAD( SM_CONTINUITY_ANGLE );
              if(GetDim() == 2)
                { 
                  /*  7 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(7, SM_LEVEL_0, dDot > dCosineTol, dCosineTol, dDot, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmPoint2d(dMaxSinAngRadParam, 0.0)) ;
                  SM_ASSERT_SET_VALUE(dDot) ; 
                  // SM_ASSERT_MSG( FALSE, _T("2D Curve Warning: Kink at interior knot" )); 
                }
              if(GetDim() == 3)
                { 
                  /*  8 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(8, SM_LEVEL_0, dDot > dCosineTol, dCosineTol, dDot, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmPoint2d(dMaxSinAngRadParam, 0.0)) ;
                  SM_ASSERT_SET_VALUE(dDot) ;
                  // SM_ASSERT_MSG( FALSE, _T("3D Curve Warning: Kink at interior knot" )); 
                }

              bRtn = FALSE;

              // skip further checks - only need to report the 1st problem
              continue;
            } // end Curve is not G1 check

        } // end for iter every interior knot

      // Check for nonregular curve: vanishing first derivative.
      // Allow zero 1st derivatives at the NaturalParameter ends of the curve.

      // Get a representive size
      SmExtent1d sDomain = GetNaturalInterval();
      SmExtent3d sBBox;
      CalculateBoundingBox( sDomain, &sBBox );
      double dSizeXYZ = sBBox.GetSize().Length();
      double dSizeT   =  sDomain.GetLength() ;

      // dTol: for finite   curves dTol and dMinSep are proportional to curve length
      //       for infinite curves dTol and dMinSep are set to absolute values
      double dTol    = IsBounded() ? (dSizeXYZ/dSizeT) / 10000        // dTol proportional to average curve speed
                                   : 1e-03 ;                          // else dTol is absolute
      double dMinSep = IsBounded() ? dSizeXYZ / smos_Max(20., dSizeT) // 3dTol for GlobalPropertyAnalysis() to find minimum 1st derivs
                                   : 5.0 ;                            //     (sloppy here is OK and cheaper than tight)

      // gwc:removed old tolerances in favor of tolerances proportional to average curve speed
      //      double dTol    = IsBounded() ? dSizeXYZ / 10000                 // dTol proportional to length
      //                                   : 1e-03 ;                          // else dTol is absolute
      //      double dMinSep = IsBounded() ? dSizeXYZ / smos_Max(20., dSizeT) // min separation between solutions
      //                                   : 5.0 ;                            //  

      SmSolutionArray rSolutionArray;
      this->GlobalPropertyAnalysis( sDomain, SM_CP_MINIMIZE_FIRST_DERIVATIVE,
                                    NULL, NULL, dMinSep, rSolutionArray );


      // Note: the solution value is not necessarily the length of 
      // the first derivative, it's the value of whatever function
      // the particular solver was zeroing or minimizing (such as C'.C').
      // So we have to evaluate it at the solution parameter.
      ULONG ii ;
      for(ii=0;ii<rSolutionArray.GetSize();ii++)
        {
          // solution parameter value
          double dParam = rSolutionArray[ii].m_vStart[0];

          // skip solutions on the natural boundary ends - they are not a problem
          if(sDomain.IsValueOnBoundary(dParam, SM_EFF_ZERO * ( 1.0 + sDomain.GetLength())))
            { continue ; }

          // 1stDeriv magnitude
          SmVector3d sPtDer[2];
          this->Evaluate( dParam, 1, FALSE, sPtDer );
          double dDerivLength = sPtDer[1].Length();

          // check 1stDeriv > tolerance
          if ( dDerivLength < dTol )
            {
              // save values for debugging
              dMin1stDerivParam = dParam ;

              if(GetDim() == 2)
                { 
                  /* 11 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(11, SM_LEVEL_0, dDerivLength >= dTol, dTol, dDerivLength, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmPoint2d(dMin1stDerivParam, 0.0)) ;
                  SM_ASSERT_SET_VALUE(dDerivLength) ;
                  // SM_ASSERT_MSG( FALSE, _T("2D Curve Warning: Vanishing first derivative" ));
                }
              if(GetDim() == 3)
                { 
                  /* 12 */
                  bRtn &= SM_ASSERT_VALUE_REPORT(12, SM_LEVEL_0, dDerivLength >= dTol, dTol, dDerivLength, _T("")) ;
                  SM_ASSERT_SET_OWNER_PARAM(SmPoint2d(dMin1stDerivParam, 0.0)) ;
                  SM_ASSERT_SET_VALUE(dDerivLength) ;
                  // SM_ASSERT_MSG( FALSE, _T("3D Curve Warning: Vanishing first derivative" ));
                }

#ifdef SM_DEBUG_CODE
              if(bDebugMe) // see also draw block at method exit
                {
                  this->Dump() ;
                }
#endif // SM_DEBUG_CODE

              // only need to report one problem
              break ;
            } // end DerivLength too short check
           
          // GWC: it's not clear if solutions are ordered by minimum value or param value - check all solns
          //      // only need to check the 1st non-boundary solution 
          //      // since solutions are ordered by with the smallest minimum value first 
          //      break ;

       } // end iter all min Deriv solutions

//    // if this is a UVTrimCurve with a given pCrvOnSurf pointer
      SmBSplineCurve *pCopyCurve = NULL ;
      SmObjDelete sCopyClean(pCopyCurve) ; // Will be set only if a copy is made.
//    if(   this->GetDim() == 2
//       && pOptCrvOnSurf != NULL)
//      {
//        // copy and scale the UVTrimCurve so that it lives on a surface
//        // mapping which is similar to arc-length parameterizations
//
//        // Surface UVDomain
//        SmExtent2d sSurfUVDomain = pOptCrvOnSurf->GetNaturalUVDomain() ;
//
//        // MidCurve Evaluation
//        SmPoint3d sMidCurvePointXYZ ;
//        this->EvaluatePoint(GetNaturalInterval().Evaluate(.5), sMidCurvePointXYZ) ;
//
//        // Surface deriviatives at midCurve evaluation watch out for singularities
//        SmPoint2d sMidCurvePointUV(sMidCurvePointXYZ.x, sMidCurvePointXYZ.y) ;
//        SmSurfParamType eSingularDirection ;
//        if(pOptCrvOnSurf->IsSingularity(sMidCurvePointUV, eSingularDirection))
//          {
//            sMidCurvePointUV = sSurfUVDomain.Evaluate(.5, .5) ;
//          }
//        SmPoint3d sSurfPoint, sSurfDu, sSurfDv ;
//        pOptCrvOnSurf->Evaluate1stDerivatives(sMidCurvePointUV, TRUE, TRUE,
//                                              sSurfPoint, sSurfDu, sSurfDv) ; 
//
//        // pick scale factors so that surface is near arclength parameterization
//        // watch out for surface singularities
//        double dScaleU = sSurfDu.Length() ;
//        double dScaleV = sSurfDv.Length() ;
//
//        // Copy the UVTrimCurve and scale its control points
//        pCopyCurve = new (this->GetContext()) SmBSplineCurve(*this) ;
//        sCopyClean.SetObj( pCopyCurve );
//        ULONG lControlPointCount = pCopyCurve->m_pNurb->pol->n + 1 ;
//        CPOINT *pPw = pCopyCurve->m_pNurb->pol->Pw ;
//        for(ULONG ii=0;ii<lControlPointCount;ii++)
//          {
//            pPw[ii].x *= dScaleU ;
//            pPw[ii].y *= dScaleV ;
//          } 
//            
//      } // end scaling of UVTrimCurves when given pOptCrvOnSurf

      if ( pCopyCurve == NULL )
        { pCopyCurve = (SmBSplineCurve*)this; }

      // Another type of kink check: vanishing radius of curvature.
      pCopyCurve->GlobalPropertyAnalysis( sDomain, SM_CP_MINIMIZE_RADIUS_OF_CURVATURE,
                                          NULL, NULL, dMinSep, rSolutionArray );

      if(dMinRadius < dTol)
        {
          // Note, the same comment as for MINIMIZE_FIRST_DERIVATIVE could
          // apply here, but in this case the function value used is indeed
          // the actual radius of curvature.

          // save values for debugging convenience
          dMinRadius      =   rSolutionArray.GetSize() > 0
                            ? rSolutionArray[0].m_vStart.m_dSolutionValue 
                            : SM_BIG_DOUBLE ;
          dMinRadiusParam =   rSolutionArray.GetSize() > 0
                            ? rSolutionArray[0].m_vStart.m_adParameters[0]
                            : 0.0 ;

          if(GetDim() == 2)
            { 
              /* 13 */
              bRtn &= SM_ASSERT_VALUE_REPORT(13, SM_LEVEL_0, dMinRadius >= dTol, dTol, dMinRadius, _T("")) ;
              SM_ASSERT_SET_OWNER_PARAM(SmPoint2d(dMinRadiusParam, 0.0)) ;
              // SM_ASSERT_MSG( FALSE, _T("2D Curve Warning: Extremely small radius of curvature") );
            }
          if(GetDim() == 3)
            { 
              /* 14 */
              bRtn &= SM_ASSERT_VALUE_REPORT(14, SM_LEVEL_0, dMinRadius >= dTol, dTol, dMinRadius, _T("")) ;
              SM_ASSERT_SET_OWNER_PARAM(SmPoint2d(dMinRadiusParam, 0.0)) ;

              // SM_ASSERT_MSG( FALSE, _T("3D Curve Warning: Extremely small radius of curvature") );
            }
          bRtn = FALSE;
        }
    } // end not degnerate curve check

  /* 15 */ // Are Curve Internal ControlPoints Coincident
  double dPoleTol = SM_ZONE_TOL_3D/10.0 ;  // gwc: need a real tolerance here - this one is just made up. 
  SmBoolean bHasInternalPole = HasInternalPole(dPoleTol) ; // TRUE = internal 'pole' due coincident Cpts
  bRtn &= SM_ASSERT_VALUE_REPORT(15, SM_LEVEL_0, bHasInternalPole == FALSE, dPoleTol, SM_UNDEF_DOUBLE, _T("")) ;

#ifdef SM_DEBUG_CODE
  // place for a break point
  if(bHasInternalPole)
    { if(bDebugMe)
        { Dump(FALSE) ; }
    }

  // draw 
  if(bDebugMe)
    {
      Dump(TRUE) ;

      SmObject  *pObj      = (SmEdge *)GetEdge() ;
      SmEdge    *pEdge     = SM_CAST_PTR(SmEdge, pObj) ; 
// Remove Composites
//      SmCEdge   *pCEdge    = SM_CAST_PTR(SmCEdge, pObj) ; 
      SmFace    *pFace     = SM_CAST_PTR(SmFace, pObj) ; 
      SmEdgeuse *pEdgeuse  = SM_CAST_PTR(SmEdgeuse, pObj) ;
      SmSurface *pSurface  = pEdgeuse && pEdgeuse->GetFace() ? pEdgeuse->GetFace()->GetSurface() : NULL ; 
      SmBrep    *pBrep =   pEdge    ? pEdge->GetBrep() 
// Remove Composites
//                         : pCEdge   ? pCEdge->GetBrep()
                         : pFace    ? pFace->GetBrep()
                         : pEdgeuse ? pEdgeuse->GetBrep() 
                         : NULL ;
      double dFailedParam = smos_4Min(smos_Fabs(dMinRadiusParam), 
                                      smos_Fabs(dMin1stDerivParam), 
                                      smos_Fabs(dMinSinAngRadParam), 
                                      smos_Fabs(dMaxSinAngRadParam)) ;
      if(pEdge)    { pEdge->Dump() ; }
// Remove Composites
//      if(pCEdge)   { pCEdge->Dump() ; }
      if(pFace)    { pFace->Dump() ; }
      if(pEdgeuse) { pEdgeuse->Dump() ; }
      if(pSurface) { pSurface->Dump() ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      if(GetDim() == 3) { smgfx_SetLook(3,4, 1,0,0) ; Draw() ; sm_GraphicsLoop() ; 
                          smgfx_SetLook(3,4, 1,0,0) ; DrawParams() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(7,8, 1,0,0) ; if(dFailedParam != SM_BIG_DOUBLE) this->DrawAt(dFailedParam) ; sm_GraphicsLoop() ;
                        }
      else              { if(pEdgeuse && pSurface)
                            { SmCrvOnSurf sCrvOnSurf((SmCurve &)*this, *pSurface) ; 
                              smgfx_SetLook(3,4, 1,0,0) ; sCrvOnSurf.Draw() ; sm_GraphicsLoop() ; 
                              smgfx_SetLook(3,4, 1,0,0) ; sCrvOnSurf.DrawParams() ; sm_GraphicsLoop() ; 
                              smgfx_SetLook(7,8, 1,0,0) ; if(dFailedParam != SM_BIG_DOUBLE) sCrvOnSurf.DrawAt(dFailedParam) ; sm_GraphicsLoop() ;
                            }
                        }
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 1,0,0) ; if(dFailedParam != SM_BIG_DOUBLE) this->DrawAt(dFailedParam) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done 
  // SM_ASSERT( bRtn );

  return( bRtn );

} // end SmBSplineCurve::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmBSplineCurve::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmCurve::AssertHeal(rAReport, pAList) ) ;
//     }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmBSplineCurve::AssertHeal fix not yet supported") ;  
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmBSplineCurve::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Check a UVTrimCurve to see if it jumps a periodic surface
            boundary.  Returns TRUE when curve is ok.

NOTES: This function is only for 2d curves being used as
   trim curves on an associated surface.

METHOD --- review the uv values of the control points for
           near periodic sized jumps in their positions.
           
           Not all periodic sized jumps are bad.  Sometimes
           a UVTrimCurve is a simple line spanning the 
           entire surface in a single jump.
           So, only consider jumps bad when the jump between
           a pair of control points is nearly the periodic size
           in a curve that has more than 2 control points.
           
***********************************************************************/
SmBoolean SmBSplineCurve::AssertNoPeriodicJump
  (SmBoolean   bSurfaceClosedU,   // in : TRUE = Surface closed in U (U isoLines are closed)
   SmBoolean   bSurfaceClosedV,   // in : TRUE = Surface closed in V (V isoLines are closed)
   SmExtent2d &rSurfaceUVDomain)  // in : Surface Domain
{
  // return value
  SmBoolean bRtn = TRUE ;

  // no work - not 2d, not periodic
  if(   GetDim() != 2
     || (   bSurfaceClosedU == FALSE
         && bSurfaceClosedV == FALSE)) { return TRUE ; }

  // make sure curve has a Nurb representation
  if(m_pNurb == NULL)
    { MakeNurb() ; }

  // get curve control points in 2d euclidean space
  SmTArray<SmPoint3d> sPolygon ; 
  SmExtent1d sIvl = GetNaturalInterval() ;
  GetPolygon(sIvl, sPolygon) ;

  // no work - single pair control point curves are assumed to not jump boundaries
  if(sPolygon.GetSize() == 2) { return TRUE ; }
  
  // get slightly shortened surface periods
  double dPeriodU = .9 * rSurfaceUVDomain.XLength() ;
  double dPeriodV = .9 * rSurfaceUVDomain.YLength() ;

  // locals
  ULONG ii, i1;
  double dJumpU, dJumpV ;

  // for every pair of control Points
  for(ii=0,i1=1;i1<sPolygon.GetSize();ii++,i1++)
    {
      dJumpU = smos_Fabs(sPolygon[i1].x - sPolygon[ii].x) ;
      dJumpV = smos_Fabs(sPolygon[i1].y - sPolygon[ii].y) ;

      // look for a potential bad U Jump
      if(bSurfaceClosedU && dJumpU >= dPeriodU) bRtn = FALSE ;
      if(bSurfaceClosedV && dJumpV >= dPeriodV) bRtn = FALSE ;

    } // end iter every controlPoint


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      SmEdgeuse *pEdgeuse = GetOwner() ? SM_CAST_PTR(SmEdgeuse, GetOwner()) : NULL ;
      SmFaceuse *pFaceuse = pEdgeuse  ? pEdgeuse->GetFaceuse() : NULL ;
      SmFace    *pFace    = pFaceuse  ? pFaceuse->GetFace() : NULL ;
      SmSurface *pSurface = pFace     ? pFace->GetSurface() : NULL ;

      Dump() ; 
      rSurfaceUVDomain.Dump() ;
      if(pSurface) pSurface->Dump() ;
    }

  // inform the public
  SM_ASSERT_MSG(bRtn, _T("SmBSplineCurve::AssertNoPeriodicJump - Periodic Jump Seen in UVTrimCurve\n")) ;

#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmBSplineCurve::AssertNoPeriodicJump

