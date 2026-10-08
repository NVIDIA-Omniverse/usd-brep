// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmHermiteCurve.cpp
* PURPOSE: Source file for SmHermiteCurve methods.
**********************************************************************/

 
#include "StdAfx.h"

#include <SmHermiteCurve.h>
#include <SmVector2d.h>
#include <SmPseudoBox.h>
#include <SmPolarBox.h>
#include <SmExtent3d.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>

#ifdef SM_DEBUG_CODE
#include <SmBSplineCurve.h>
#include <SmEdge.h>
// Remove Composites
// #include <SmCEdge.h>
#include <SmFace.h>
#include <SmSurface.h>
#include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Construct an offset Hermite approximation to the input 
    Hermite.  This algorithm interpolates the start/end points/tangents
    and the mid point of the theoretical offset.  The mathematics used
    was developed by Dr. Tom Sederberg.  Note that this does not work
    well for curves which are not parallel to plane of projection.

NOTES: 
***********************************************************************/
SmHermiteCurve::SmHermiteCurve
  (ULONG lDimension,
   const SmHermiteCurve & crHermiteToOffset,
   const SmVector3d & crOffsetPlaneNormal,
   double dOffsetDistance)
 : SmCurve(lDimension)
{
    SmVector3d sN = crOffsetPlaneNormal;
    SE(sN.Unitize());
    // Convert to Bezier form
    SmPoint3d sP0 = crHermiteToOffset.m_vP1;
    SmPoint3d sP1 = crHermiteToOffset.m_vP1 + crHermiteToOffset.m_vD1 / 3.0;
    SmPoint3d sP2 = crHermiteToOffset.m_vP2 - crHermiteToOffset.m_vD2 / 3.0;
    SmPoint3d sP3 = crHermiteToOffset.m_vP2;

    // Compute a
    SmVector3d a0 = sP1 - sP0;
    SmVector3d a1 = sP2 - sP1;
    SmVector3d a2 = sP3 - sP2;
    SmVector3d a3 = sP3 - sP0;

    if (a0.LengthSquared() < SM_EFF_ZERO_SQ) SE(SM_ERR);
    if (a2.LengthSquared() < SM_EFF_ZERO_SQ) SE(SM_ERR);

    // Compute a0 Transpose and a2 Transpose
    SmVector3d a0T = a0 * crOffsetPlaneNormal;
    SmVector3d a2T = a2 * crOffsetPlaneNormal;
    if (a0T.LengthSquared() < SM_EFF_ZERO_SQ) SE(SM_ERR);
    if (a2T.LengthSquared() < SM_EFF_ZERO_SQ) SE(SM_ERR);
    a0T.Unitize();
    a2T.Unitize();

    // Test for first case where all points are on same line (relative to offset plane
    // projection.
    double d = dOffsetDistance;
    SmPoint3d sQ0, sQ1, sQ2, sQ3;
    sQ0 = sP0 + d * a0T;
    sQ3 = sP3 + d * a2T;
    if (smos_Fabs(a1.Dot(a0T)) < SM_EFF_ZERO && smos_Fabs(a2.Dot(a0T)) < SM_EFF_ZERO) {
        // Have straight line.
        sQ1 = sP1 + d * a0T;
        sQ2 = sP2 + d * a2T;
    }
    else if (smos_Fabs(a2.Dot(a0T)) < SM_EFF_ZERO) {
        // Have case where end edges of control polygon are parallel
        sQ1 = sP1 + d * a0T + (8.0 * d / 3.0) * a0 / (a0.Length() + a2.Length());
        sQ2 = sP2 + d * a2T - (8.0 * d / 3.0) * a2 / (a0.Length() + a2.Length());
    }
    else {
        // Have standard Bezier offset case
        // Compute V
        SmVector3d a1a3 = a1 + a3;
        if (a1a3.LengthSquared() < SM_EFF_ZERO_SQ) SE(SM_ERR);
        SmVector3d V = 2.0 * (a1+a3) / a1a3.Length() - a0 / a0.Length() - a2 / a2.Length();
        sQ1 = sP1 + d * a0T + (4.0 * d / 3.0) * ((V.Dot(a2)) / (a0.Dot(a2T*a2.Length()))) * a0;
        sQ2 = sP2 + d * a2T + (4.0 * d / 3.0) * ((V.Dot(a0)) / (a2.Dot(a0T*a0.Length()))) * a2;
    }
    
    m_vP1 = sQ0;
    m_vD1 = 3.0 * (sQ1 - sQ0);
    m_vP2 = sQ3;
    m_vD2 = 3.0 * (sQ3 - sQ2);
    m_vC = -3.0*m_vP1 - 2.0*m_vD1 + 3.0*m_vP2 - m_vD2;
    m_vD = 2.0*m_vP1 + m_vD1 - 2.0*m_vP2 + m_vD2;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) 
      {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        sP0.Draw(); 
        sP1.Draw();
        sP2.Draw();
        sP3.Draw();
        a0.Draw(&sP0);
        a1.Draw(&sP1);
        a2.Draw(&sP2);
        smgfx_SetColor(1,0,0);
        sQ0.Draw(); 
        sQ1.Draw();
        sQ2.Draw();
        sQ3.Draw();
        a0 = sQ1 - sQ0;
        a0.Draw(&sQ0);
        a1 = sQ2 - sQ1;
        a1.Draw(&sQ1);
        a2 = sQ3 - sQ2;
        a2.Draw(&sQ2);
        DrawWDeriv(GetNaturalInterval(),0);
        crHermiteToOffset.DrawWDeriv(crHermiteToOffset.GetNaturalInterval(),0);
        sm_GraphicsLoop();
      }
#endif


} // end SmHermiteCurve::SmHermiteCurve constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmHermiteCurve

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmHermiteCurve::operator==
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
      SmHermiteCurve &rOther = (SmHermiteCurve &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_vP1  == rOther.m_vP1
              && m_vD1  == rOther.m_vD1
              && m_vP2  == rOther.m_vP2
              && m_vD2  == rOther.m_vD2
              && m_vC   == rOther.m_vC 
              && m_vD   == rOther.m_vD) ;
    }

  // all done
  return bRtn ;

} // end SmHermiteCurve::operator==

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the 
    continuities at each of the unique knots
    
NOTES: HermiteCurves are continuous throughout.  Return array 
***********************************************************************/
SmStatus SmHermiteCurve::CalculateContinuities
  (SmContinuityType           & reMinContinuityInCurve,  // out:
   SmTArray<SmContinuityType> & rContinuitiesAtKnots,    // out:
   double                       dContinuityAngleTol)     // NotUsed: in :
  const
{
  SM_REF1(dContinuityAngleTol) ;
  // init output
  rContinuitiesAtKnots.ReSet() ;

  // set output
  reMinContinuityInCurve = SM_CT_CINFINITY ;
  rContinuitiesAtKnots.Add( SM_CT_DISCONTINUOUS ) ;
  rContinuitiesAtKnots.Add( SM_CT_DISCONTINUOUS ) ;
   
  // all done
  return SM_SUCCESS;

} // end SmHermiteCurve::CalculateContinuities

/*******************************************************************//**
PURPOSE: Given an interval on the curve, compute the axis alligned or
    non-axis alligned bounding box.

NOTES: At least one of the outputs must be non-NULL.   

   Never expands the BBox prior to being returned.
   BBox is just the bouding box of the Hermit curves equivalent control points.
***********************************************************************/
SmStatus SmHermiteCurve::CalculateBoundingBox
  (const SmExtent1d & crInterval,  // NotUsed: in :
   SmExtent3d       * pNormalBox,  // out: Axis alligned box                       
   SmPseudoBox      * pPseudoBox,  // out: Non-axis aligned box                    
   SmPolarBox       * pPolarBox,   // out: Surface normal vector field bounding box
   SmBoolean          bExpandBox)  // NotUsed: in : bExpandBox = not-used
                                   //      TRUE = expand BBox slightly prior to return
                                   //      FALSE= don't; default:[TRUE]
 const
{ 
  SM_REF2(crInterval, bExpandBox) ;
    SM_ASSERT(   pNormalBox != NULL 
              || pPseudoBox != NULL
              || pPolarBox  != NULL);

    // Compute Bezier polygon for Hermite and use points to compute bounding
    // boxes.
    SmPoint3d sPnts[4];
    sPnts[0] = m_vP1;
    sPnts[1] = m_vP1 + m_vD1 / 3.0;
    sPnts[2] = m_vP2 - m_vD2 / 3.0;
    sPnts[3] = m_vP2;

    // Cumpute the basis vectors of the PseudoBox using the control polygon
    if (pPseudoBox) 
      {
        SmVector3d sV1 = sPnts[3] - sPnts[0];

        // Note pseudoBox principle direction is set from beg to end points.
        // 2nd axis is selected by the begin point tangent.
        // When the curve is closed the pseudo box gets default directions
        if (SM_IS_ZERO(sV1.LengthSquared()))
          {
            // use default basis vectors
            *pPseudoBox = SmPseudoBox(); // Initialize it 
          }
        else
          {  
            SmVector3d sBasis1, sBasis2, sBasis3;
            sV1.MakeUnitOrthoVectors(&m_vD1,sBasis1,sBasis2,sBasis3);
            pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
          }
      } // end init pseudoBox

    // load the bounding boxes
    if (pNormalBox) { *pNormalBox = SmExtent3d(sPnts[0]);
                       pNormalBox->AddPoint3d(sPnts[1]);
                       pNormalBox->AddPoint3d(sPnts[2]);
                       pNormalBox->AddPoint3d(sPnts[3]);
                    }
    if (pPseudoBox) { pPseudoBox->AddPoint3d(sPnts[0]);
                      pPseudoBox->AddPoint3d(sPnts[1]);
                      pPseudoBox->AddPoint3d(sPnts[2]);
                      pPseudoBox->AddPoint3d(sPnts[3]);
                    }
    if (pPolarBox)  { pPolarBox->ReSet() ;
                      pPolarBox->AddVector3d(sPnts[1] - sPnts[0]) ;
                      pPolarBox->AddVector3d(sPnts[2] - sPnts[1]) ;
                      pPolarBox->AddVector3d(sPnts[3] - sPnts[2]) ;
                    }

    return SM_SUCCESS;

} // end SmHermiteCurve::CalculateBoundingBox


/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmHermiteCurve::Evaluate
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
    // Initialize derivatives requested greater than third
    for (ULONG i=4; i<lNumDerivatives; i++) {
        aPointAndDerivatives[i].x = 0.0;
        aPointAndDerivatives[i].y = 0.0;
        aPointAndDerivatives[i].z = 0.0;
    }

    // Compute a, b, c, d of power basis - note we speeded this
    // up by computing these and storing them 
    const SmVector3d & A = m_vP1;
    const SmVector3d & B = m_vD1;
    const SmVector3d & C = m_vC;
    const SmVector3d & D = m_vD;

    double u = dParameter;

    // Compute position
    aPointAndDerivatives[0] = A + u * (B + u * (C + u * D));

    if (lNumDerivatives >= 1) {
        aPointAndDerivatives[1] = B + u * (2.0*C + 3.0*u*D);
    }
    if (lNumDerivatives >= 2) {
        aPointAndDerivatives[2] = 2.0*C + 6.0*u*D;
    }
    if (lNumDerivatives >= 3) {
        aPointAndDerivatives[3] = 6.0*D;
    }

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

    return SM_SUCCESS;

} // end SmHermiteCurve::Evaluate


/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmHermiteCurve::EvaluatePoint
  (double dParameter, 
   SmPoint3d & rPoint) 
 const
{ 
    // Compute a, b, c, d of power basis - note we speed this
    // up by computing these and storing them 
    const SmVector3d & A = m_vP1;
    const SmVector3d & B = m_vD1;
    const SmVector3d & C = m_vC;
    const SmVector3d & D = m_vD;

    double u = dParameter;

    // Compute position
    rPoint = A + u * (B + u * (C + u * D));

    return SM_SUCCESS;

} // end SmHermiteCurve::EvaluatePoint

/*******************************************************************//**
PURPOSE: Compute BSpline ControlPolygon to define a BSplineCurve
  that is exactly equal to the Hermite Curve over the domain 0 to 1.

NOTES: 
***********************************************************************/
SmStatus SmHermiteCurve::GetBSplineControlPolygon
  (SmTArray<SmPoint3d> &rPoly)     // out: set to 4 BSplineControl Points
 const 
{
  // init output
  rPoly.SetSize(4) ;

  // set output
  rPoly[0] = m_vP1 ;
  rPoly[1].Set(m_vP1.x + m_vD1.x / 3.0,
               m_vP1.y + m_vD1.y / 3.0,
               m_vP1.z + m_vD1.z / 3.0) ;
  rPoly[2].Set(m_vP2.x - m_vD2.x / 3.0,
               m_vP2.y - m_vD2.y / 3.0,
               m_vP2.z - m_vD2.z / 3.0) ;
  rPoly[3] = m_vP2 ;

  // all done
  return(SM_SUCCESS) ;

} // end SmHermiteCurve::GetBSplineControlPolygon

/*******************************************************************//**
PURPOSE: Compute the chord height of a Hermite curve.

NOTES: 
***********************************************************************/
double SmHermiteCurve::ComputeChordHeight
  () 
 const
{
    SmPoint3d sPnt1, sPnt2;
    sPnt1 = m_vP1 + m_vD1 / 3.0;
    sPnt2 = m_vP2 - m_vD2 / 3.0;
    double dDist, dParam;
    if (m_vP1.DistanceBetweenSquared(m_vP2) < SM_EFF_ZERO_SQ) {
        dDist = m_vP1.DistanceBetween(sPnt1) + 
            sPnt1.DistanceBetween(sPnt2) + 
            sPnt2.DistanceBetween(m_vP2);
    }
    else {
        SE(smgu_SegmentPointDistance(m_vP1,m_vP2,sPnt1,dDist,dParam));
        double dDist2;
        SE(smgu_SegmentPointDistance(m_vP1,m_vP2,sPnt2,dDist2,dParam));
        dDist = smos_Max(dDist,dDist2);
    }
    return dDist;

} // end SmHermiteCurve::ComputeChordHeight

/*******************************************************************//**
PURPOSE: Compute angle spanned of a Hermite curve in degrees.

NOTES: 
***********************************************************************/
double SmHermiteCurve::ComputeAngleSpannedInDegrees
  () 
 const
{
    SmPoint3d sPnt1, sPnt2;
    sPnt1 = m_vP1 + m_vD1 / 3.0;
    sPnt2 = m_vP2 - m_vD2 / 3.0;

    SmVector3d sVec1 = sPnt1 - m_vP1;
    SmVector3d sVec2 = sPnt2 - sPnt1;
    SmVector3d sVec3 = m_vP2 - sPnt2;
    double dAngle1, dAngle2;
    double dTotalAngle = 0.0;
    if (sVec2.LengthSquared() < SM_EFF_ZERO_SQ) {
        if (sVec1.AngleBetween(sVec2,dAngle1) != SM_SUCCESS) {
            return 360.0;
        }
        return dAngle1 * 180.0 / SM_PI;
    }

    if (sVec1.AngleBetween(sVec2,dAngle1) != SM_SUCCESS) {
        return 360.0;
    }
    dTotalAngle += dAngle1;
    if (sVec2.AngleBetween(sVec3,dAngle2) != SM_SUCCESS) {
        return 360.0;
    }
    dTotalAngle += dAngle2;
    return dTotalAngle * 180.0 / SM_PI;

} // end SmHermiteCurve::ComputeAngleSpannedInDegrees

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities 
     of a BSpline curve.

NOTES: This is an equivalent knot vector made of degree 3 entries for
 the curve domain
***********************************************************************/
SmStatus SmHermiteCurve::GetKnots
  (SmTArray<double> & rKnots,               // out: Unique knot vector                     
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: multiplicity value for each knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
  const
{
  // init output
  rKnots.ReSet() ;
  if(pKnotMultiplicities) { pKnotMultiplicities->ReSet() ; }

  // locals
  SmExtent1d sIvl = GetNaturalInterval() ;

  // interval start
  if(pOptIvl == NULL || pOptIvl->ContainsValue(sIvl.GetMin()))
    {
      rKnots.Add(sIvl.GetMin()) ;
      if(pKnotMultiplicities)
        {
          pKnotMultiplicities->Add(3) ;
        }
    }

  // interval end
  if(pOptIvl == NULL || pOptIvl->ContainsValue(sIvl.GetMax()))
    {
      rKnots.Add(sIvl.GetMax()) ;
      if(pKnotMultiplicities)
        {
          pKnotMultiplicities->Add(3) ;
        }
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmHermiteCurve::GetKnots

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels

NOTES:
***********************************************************************/
SmAssertReportLabel sAssertHermiteCurve_list[] =
{
 /*  0 */ {SM_AT_VECTOR,    _T("Uninit Vector"), _T("Defining Pt/1stDeriv Vector is uninitialized") },
 /*  1 */ {SM_AT_VECTOR,    _T("Uninit Vector"), _T("Power Basis Vector is uninitialized") },
} ;   
      
/*******************************************************************//**
PURPOSE: Check the validity of an SmHermiteCurve.
      
NOTES: Checks made:

 Note: can't add an optional argument to this because it's virtual:
       would have to add it to every class's method.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmHermiteCurve::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // return value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmCurve::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // check for initialized end values
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, 
             (   ( (GetDim() == 3) && (m_vP1.IsUndef() == FALSE) 
                                   && (m_vD1.IsUndef() == FALSE) 
                                   && (m_vP2.IsUndef() == FALSE) 
                                   && (m_vD2.IsUndef() == FALSE))
              || ( (GetDim() == 2) && (m_vP1.x != SM_UNDEF_DOUBLE) && (m_vP1.y != SM_UNDEF_DOUBLE)
                                   && (m_vD1.x != SM_UNDEF_DOUBLE) && (m_vD1.y != SM_UNDEF_DOUBLE)
                                   && (m_vP2.x != SM_UNDEF_DOUBLE) && (m_vP2.y != SM_UNDEF_DOUBLE)
                                   && (m_vD2.x != SM_UNDEF_DOUBLE) && (m_vD2.y != SM_UNDEF_DOUBLE))), 
             _T("")) ;
                                                  
  // check for initialized Power Basis
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, 
             (   ( (GetDim() == 3) && (m_vC.IsUndef() == FALSE) 
                                   && (m_vD.IsUndef() == FALSE))
              || ( (GetDim() == 2) && (m_vC.x != SM_UNDEF_DOUBLE) && (m_vC.y != SM_UNDEF_DOUBLE)
                                   && (m_vD.x != SM_UNDEF_DOUBLE) && (m_vD.y != SM_UNDEF_DOUBLE))), 
             _T("")) ;
                                                  
    
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump() ;

      SmBSplineCurve *pCubic  = NULL ;
      SmObjDelete     sClean(pCubic) ;
      SmBSplineCurve::CreateFromHermiteCurve(*GetContext(), *this, pCubic, 0, 1) ;
      SM_ASSERT_VALID(pCubic) ;

      SmObject  *pObj      = GetOwner() ;
      SmEdge    *pEdge     = SM_CAST_PTR(SmEdge, pObj) ; 
// Remove Composites
//      SmCEdge   *pCEdge    = SM_CAST_PTR(SmCEdge, pObj) ; 
      SmFace    *pFace     = SM_CAST_PTR(SmFace, pObj) ; 
      SmEdgeuse *pEdgeuse  = SM_CAST_PTR(SmEdgeuse, pObj) ;
      SmSurface *pSurface  = pEdgeuse && pEdgeuse->GetFace() ? pEdgeuse->GetFace()->GetSurface() : NULL ; 
      SmBrep    *pBrep     =   pEdge    ? pEdge->GetBrep() 
// Remove Composites
//                             : pCEdge   ? pCEdge->GetBrep()
                             : pFace    ? pFace->GetBrep()
                             : pEdgeuse ? pEdgeuse->GetBrep() 
                             : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;

      smgfx_SetLook(3,4, 1,0,0) ; pCubic->Draw() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(3,4, 1,0,0) ; pCubic->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; pCubic->DrawSpeed() ; sm_GraphicsLoop() ;

      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done 
  // SM_ASSERT( bRtn );
  return( bRtn );

} // end SmHermiteCurve::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmHermiteCurve::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmHermiteCurve::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmHermiteCurve::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmHermiteCurve to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmHermiteCurve::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // NotUsed: in : database version to get proper sequence of writes                                                                       
 const
{
  SM_REF1(lDBVersionNumber) ;
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  if (eType == SM_ASCII) 
    {
      rFileOut << m_vP1.x << " " << m_vP1.y << " " << m_vP1.z  << " SmHermiteCurve Initial point        \n" ; 
      rFileOut << m_vD1.x << " " << m_vD1.y << " " << m_vD1.z  << " SmHermiteCurve Initial direction    \n" ; 
      rFileOut << m_vP2.x << " " << m_vP2.y << " " << m_vP2.z  << " SmHermiteCurve Final point          \n" ; 
      rFileOut << m_vD2.x << " " << m_vD2.y << " " << m_vD2.z  << " SmHermiteCurve Final direction      \n" ; 
      rFileOut << m_vC.x  << " " << m_vC.y  << " " << m_vC.z   << " SmHermiteCurve C of power basis eqn \n" ; 
      rFileOut << m_vD.x  << " " << m_vD.y  << " " << m_vD.z   << " SmHermiteCurve D of power basis eqn \n" ;
    }
  else 
    {
      SER(rDB.WriteDouble( m_vP1.x )) ; SER(rDB.WriteDouble( m_vP1.y )) ; SER(rDB.WriteDouble( m_vP1.z )) ; 
      SER(rDB.WriteDouble( m_vD1.x )) ; SER(rDB.WriteDouble( m_vD1.y )) ; SER(rDB.WriteDouble( m_vD1.z )) ; 
      SER(rDB.WriteDouble( m_vP2.x )) ; SER(rDB.WriteDouble( m_vP2.y )) ; SER(rDB.WriteDouble( m_vP2.z )) ; 
      SER(rDB.WriteDouble( m_vD2.x )) ; SER(rDB.WriteDouble( m_vD2.y )) ; SER(rDB.WriteDouble( m_vD2.z )) ; 
      SER(rDB.WriteDouble( m_vC.x )) ;  SER(rDB.WriteDouble( m_vC.y )) ;  SER(rDB.WriteDouble( m_vC.z )) ; 
      SER(rDB.WriteDouble( m_vD.x )) ;  SER(rDB.WriteDouble( m_vD.y )) ;  SER(rDB.WriteDouble( m_vD.z )) ; 
    }

  // all done
  return SM_SUCCESS;

} // end SmHermiteCurve::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmHermiteCurve from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmHermiteCurve::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // NotUsed: in : database version to get proper sequence of writes
{
  SM_REF3(lType, lDim, lDBVersionNumber) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmHermiteCurve_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmHermiteCurve *pHCrv =   (rpNewCurve == NULL)
                          ? new (crContext) SmHermiteCurve()
                          : (SmHermiteCurve *)rpNewCurve ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  if (eType == SM_ASCII) 
    {
      rFileIn >> pHCrv->m_vP1.x >> pHCrv->m_vP1.y >> pHCrv->m_vP1.z ;  rDB.GoToNextLine() ;              
      rFileIn >> pHCrv->m_vD1.x >> pHCrv->m_vD1.y >> pHCrv->m_vD1.z ;  rDB.GoToNextLine() ;     
      rFileIn >> pHCrv->m_vP2.x >> pHCrv->m_vP2.y >> pHCrv->m_vP2.z ;  rDB.GoToNextLine() ;     
      rFileIn >> pHCrv->m_vD2.x >> pHCrv->m_vD2.y >> pHCrv->m_vD2.z ;  rDB.GoToNextLine() ;
      rFileIn >> pHCrv->m_vC.x  >> pHCrv->m_vC.y  >> pHCrv->m_vC.z ;   rDB.GoToNextLine() ;
      rFileIn >> pHCrv->m_vD.x  >> pHCrv->m_vD.y  >> pHCrv->m_vD.z ;   rDB.GoToNextLine() ;     
    }
  else 
    {
      SER(rDB.ReadDouble( pHCrv->m_vP1.x )) ; SER(rDB.ReadDouble( pHCrv->m_vP1.y )) ; SER(rDB.ReadDouble( pHCrv->m_vP1.z )) ; 
      SER(rDB.ReadDouble( pHCrv->m_vD1.x )) ; SER(rDB.ReadDouble( pHCrv->m_vD1.y )) ; SER(rDB.ReadDouble( pHCrv->m_vD1.z )) ; 
      SER(rDB.ReadDouble( pHCrv->m_vP2.x )) ; SER(rDB.ReadDouble( pHCrv->m_vP2.y )) ; SER(rDB.ReadDouble( pHCrv->m_vP2.z )) ; 
      SER(rDB.ReadDouble( pHCrv->m_vD2.x )) ; SER(rDB.ReadDouble( pHCrv->m_vD2.y )) ; SER(rDB.ReadDouble( pHCrv->m_vD2.z )) ; 
      SER(rDB.ReadDouble( pHCrv->m_vC.x )) ;  SER(rDB.ReadDouble( pHCrv->m_vC.y )) ;  SER(rDB.ReadDouble( pHCrv->m_vC.z )) ; 
      SER(rDB.ReadDouble( pHCrv->m_vD.x )) ;  SER(rDB.ReadDouble( pHCrv->m_vD.y )) ;  SER(rDB.ReadDouble( pHCrv->m_vD.z )) ; 
    }

  // all done
  rpNewCurve = pHCrv ; 
  return SM_SUCCESS;

} // end SmHermiteCurve::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmHermiteCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmHermiteCurve_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmHermiteCurve::Dump
  () 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmHermiteCurve::Dump()")) ;

  // ouput cache data
  SmCurve::Dump(FALSE) ;

  smos_sprintf(sBuff,_T("\nSmHermiteCurve = 0x%p "),this);
  smos_sprintf(sBuffForFile,_T("\nSmHermiteCurve = %s "),_T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // EndPoints and EndTangents
  smos_sprintf(sBuff,_T("\n  Beg Pt: [%16.16lf %16.16lf %16.16lf]"), m_vP1.x, m_vP1.y, m_vP1.z); smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("  1stDeriv: [%16.16lf %16.16lf %16.16lf]"), m_vD1.x, m_vD1.y, m_vD1.z); smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\n  Beg Pt: [%16.16lf %16.16lf %16.16lf]"), m_vP2.x, m_vP2.y, m_vP2.z); smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("  1stDeriv: [%16.16lf %16.16lf %16.16lf]"), m_vD2.x, m_vD2.y, m_vD2.z); smos_WriteBuffer(sBuff);

  // Power Basis
  smos_sprintf(sBuff,_T("\n  PowerBasis C: [%16.16lf %16.16lf %16.16lf]"), m_vC.x, m_vC.y, m_vC.z); smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\n  PowerBasix D: [%16.16lf %16.16lf %16.16lf]"), m_vD.x, m_vD.y, m_vD.z); smos_WriteBuffer(sBuff);

  smos_WriteBuffer(_T("\n End SmHermiteCurve::Dump()\n")) ;

} // end SmHermiteCurve::Dump
