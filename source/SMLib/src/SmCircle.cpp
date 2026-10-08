// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCircle.cpp
* PURPOSE: Implementation of SmCircle methods.
**********************************************************************/

  
#include "StdAfx.h"

#include <SmCircle.h>
#include <SmGeomUtility.h>

/*******************************************************************//**
PURPOSE: Constructor for an Circle object.

NOTES:
  When declaring automatic objects:
    SmCircle(sPnt, sVec, sVec, sIvl, dRad) ; FAILS because m_pContext = NULL
    SmCircle(sPnt, sVec, sVec, sIvl, dRad, 2/3, TRUE/FALSE, cpContext) ; SUCCEEDS

  When declaring heap objects:
    SmCircle *pCircle = new (cpContex) SmCircle(sPnt, sVec, sVec, sIvl, dRad) ; SUCCEEDS
 
***********************************************************************/
SmCircle::SmCircle
( 
  const SmPoint3d  & crCenter,          // in : Center of Circle
  const SmVector3d & crXAxis,           // in : X axis of Circle - corresponds to an angle of 0 degrees
  const SmVector3d & crYAxis,           // in : Y axis of Circle - corresponds to an angle of 90 degrees
  const SmExtent1d & crAnalDomain,      // in : Angular domain.  Must be between -360 and 360 inclusive
  double             dRadius,           // in : radius
  ULONG              lDimension,        // in : space dimension containing circle (2 or 3)
  const SmContext  * cpContext,         // in : must be given for automatic variables, optional for
                                        //      stack variables built with overloaded new.
                                        //      default:[NULL]
  const SmBSplineCurve * pOptNurb,      // in : Optional Nurb copied to make m_pNurb object
                                        //      If Given, when m_bInsideOut == FALSE
                                        //                its StartPoint == CircleStart
                                        //                its EndPoint   == CricleEnd
                                        //                it must lie in the plane [crCenter, cross(X,Y)]
                                        //                its radius     == radius
                                        //      When m_bInsideOut == TRUE
                                        //                The endPoints must be swapped.
                                        //      default:[NULL]
  SmBoolean bInsideOut                  // in : TRUE = Nurb Tangent = -LineVector
                                        //      FALSE= Nurb Tangent =  LineVector
)
: SmEllipse(crCenter,crXAxis,crYAxis,crAnalDomain,dRadius,dRadius,lDimension,cpContext,pOptNurb,bInsideOut)
{
  SM_ASSERT(dRadius > SM_EFF_ZERO);
  m_eBSplineCurveForm = SM_CF_CIRCULAR_ARC;

} // end SmCircle::SmCircle constructor

/*******************************************************************//**
PURPOSE: Constructor for an Circle object.

NOTES:
  When declaring automatic objects:
    SmCircle(sPnt, sPnt, sPnt) ;                             FAILS because m_pContext = NULL
    SmCircle(sPnt, sPnt, sPnt, 2/3, TRUE/FALSE, cpContext) ; SUCCEEDS

  When declaring heap objects:
    SmCircle *pCircle = new (cpContex) SmCircle(sPnt, sPnt, sPnt) ; SUCCEEDS
***********************************************************************/
SmCircle::SmCircle
  (const     SmPoint3d & cStartPoint,    // in : Circular Arc Start Point 
   const     SmPoint3d & cMidPoint,      // in : Circular Arc Mid Point
   const     SmPoint3d & cEndPoint,      // in : Circular Arc End Point
   ULONG                 lDimension,     // in : dimensions of points vectors [2 or 3]
   SmBoolean             bClosedCircle,  // in : TRUE = Return rAnalDomain for closed Circle
                                         //      FALSE= Return rAnalDomain for Circular Arc from StartPoint to EndPoint
   const SmContext     * cpContext)      // in : must be given for automatic variables, optional for
                                         //      stack variables built with overloaded new.
 : SmEllipse(lDimension, cpContext)
{
  SmPoint3d  sCenter ;      
  SmVector3d sXAxis ;       
  SmVector3d sYAxis ;       
  SmExtent1d sAnalDomain ;  
  double     dRadius ;      
  
  // compute circle parameters
  SmStatus sStatus = smgu_CircleFrom3Points(cStartPoint, cMidPoint, cEndPoint,
                                            sCenter, sXAxis, sYAxis,   
                                            sAnalDomain, dRadius, 
                                            lDimension, bClosedCircle) ;
  SM_ASSERT(sStatus == SM_SUCCESS) ;

  // set Parent Object values
  m_vPosition.SetCanonical(sCenter, sXAxis, sYAxis) ; 
  m_vAnalDomain    = sAnalDomain ;
  m_dRadiusAtXAxis = dRadius ;
  m_dRadiusAtYAxis = dRadius ;

  // make the associated NURB and set the Form value
  SE(MakeNurb());    
  m_eBSplineCurveForm = SM_CF_CIRCULAR_ARC;

} // end SmCircle::SmCircle constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmCircle

NOTES: 
***********************************************************************/
SmCircle::SmCircle
  (const SmCircle & crSource)   // in : target surface to copy
: SmEllipse(crSource)
//      : SmEllipse(crSource.m_vPosition.GetOrigin(),
//                  crSource.m_vPosition.GetXAxis(), 
//                  crSource.m_vPosition.GetYAxis(), 
//                  crSource.m_vAnalDomain,          
//                  crSource.m_dRadiusAtXAxis,       
//                  crSource.m_dRadiusAtYAxis,       
//                  crSource.GetDim(),
//                  m_cpContext,
//                  NULL,              
//                  crSource.m_bInsideOut)
{
  SetFromGwNurb(GetDim(),crSource.m_pNurb);

} // end SmCircle::SmCircle copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCircle

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCircle::operator==
  (const SmCurve& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmEllipse::operator ==(crOther) ;

  // all done
  return bRtn ;

} // end SmCircle::operator==

/*******************************************************************//**
PURPOSE: Copy a Circle

NOTES: 
***********************************************************************/
SmStatus SmCircle::Copy     
  (const SmContext & crContext,
   SmCurve        *& rpNewCurve) 
  const
{
  rpNewCurve = new (crContext) SmCircle(*this) ;
  NER(rpNewCurve);

  return SM_SUCCESS;

} // end SmCircle::Copy


/*******************************************************************//**
PURPOSE: Method to create closed Circle object.

NOTES: 
***********************************************************************/
SmStatus SmCircle::CreateCanonical
  (const SmContext        & crContext,    // in : new object context
   const SmAxis2Placement & crOrigin,     // in : circle orientation (origin, xAxis, yAxis)
   double                   dRadius,      // in : dist from origin to circle circumference
   SmCircle              *& rpNewCircle,  // out: new object
   const SmExtent1d       * pInterval,    // opt: Specify interval in degrees [-360 <= min <= max <= 360.0]
                                          //      NULL = [0.0 360.0]
                                          //      default:[NULL]
   SmBoolean              * pInsideOut)   // opt: TRUE: for compatibility with sphere and torus,
                                          //            Nurb curve runs in opposite direction from Analytic curve
                                          //      FALSE: Normal case - nurb and analytic curves are the same shape.
                                          //      default:[FALSE]
{
  // Reject zero/negative radius: no valid NURB (m_pNurb stays NULL, later
  // crashing NURB accessors), and the ctor assert is a no-op in release.
  if (!(dRadius > SM_EFF_ZERO))
    {
      rpNewCircle = NULL;
      SE_MSG(SM_ERR, _T("SmCircle::CreateCanonical: non-positive radius; cannot create circle")) ;
      return SM_ERR;
    }

  // new circle, interval  = given interval or [0,360]
  //             InsideOut = given value or FALSE
  //             dimension = 3
  SmExtent1d crNullInterval (0.0, 360.0);
  rpNewCircle = new (crContext) SmCircle
                        (crOrigin.GetOriginRef(),
                         crOrigin.GetXAxisRef(), 
                         crOrigin.GetYAxisRef(),
                         pInterval ? *pInterval : crNullInterval,
                         dRadius, 3, &crContext, NULL,
                         pInsideOut ? *pInsideOut : FALSE);
  // all done
  return SM_SUCCESS;

} // end SmCircle::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data.

NOTES: 
***********************************************************************/
SmStatus SmCircle::GetCanonical
  (SmAxis2Placement & rOrigin,       // out: 
   double           & rdRadius,      // out: 
   SmBoolean        * pOptInsideOut) // out: Optional
  const
{
  rOrigin    = m_vPosition;
  rdRadius   = m_dRadiusAtXAxis;
  if(pOptInsideOut) { *pOptInsideOut = m_bInsideOut ; }

  return SM_SUCCESS;

} // end SmCircle::GetCanonical

/*******************************************************************//**
PURPOSE: Build a SmCircle for given SmBSplineCurve when its 
            geometry and representation are that of an SmCircle.

NOTES: 
***********************************************************************/
SmBoolean SmCircle::IsNurbCurveCircle
  (const SmContext        & crContext,
   const SmBSplineCurve   * pTestCurve,
   SmCircle              *& rpCircle,
   double                   dToleranceScale)
{

  // low work - already a circle
  if(pTestCurve->IsKindOf(SmCircle_TYPE))
    {
      // copy the circle
      pTestCurve->Copy(crContext, (SmCurve *&)rpCircle) ;
      return(TRUE) ; 
    }

  // no work - not a degree 2 rational curve
  ULONG lDeg = pTestCurve->GetDegree();
  if(   lDeg != 2
     || !pTestCurve->IsRational())
    { return FALSE; }

  // scale zero value
  SmPoint3d sMidPoint ;
  pTestCurve->EvaluatePoint(pTestCurve->GetNaturalInterval().Evaluate(.4567), sMidPoint) ;
  double dScaledZero =   dToleranceScale  
                       * ANALYTIC_TOL_SCALE * SM_EFF_ZERO  
                       * (1.0 + sMidPoint.GetMaxDimension());

  // skip degenerate curves
  if (pTestCurve->IsDegenerate(dScaledZero)) 
    { return FALSE; }

  // Get circle reference frame, radius and angle extent
  SmAxis2Placement sRefFrame;
  double dRadius, dStartAngDeg, dEndAngDeg;
  if (!pTestCurve->IsArc(5, dScaledZero,sRefFrame, dRadius, dStartAngDeg, dEndAngDeg)) 
    {
      // when isoParameterCurve is not a circular Arc
      return FALSE;
    }
  SmExtent1d sAnalDomain(dStartAngDeg,dEndAngDeg);

  // make new circle
  rpCircle = new (crContext) SmCircle(sRefFrame.GetOriginRef(),
                                      sRefFrame.GetXAxisRef(),
                                      sRefFrame.GetYAxisRef(),
                                      sAnalDomain,
                                      dRadius, 3,
                                      &crContext,
                                      pTestCurve) ;
  // all done
  return(TRUE) ;

} // end SmCircle::IsNurbCurveCircle 

/*******************************************************************//**
PURPOSE: Check if 'THIS' curve is an ARC.

NOTES: It is, of course.
***********************************************************************/
  SmBoolean SmCircle::IsArc (ULONG,                                // in, unused:  lNumberOfSamplePoints
                             double,                               // in, unused:  dTol
                             SmAxis2Placement & rReferenceFrame,   // out
                             double           & rdRadius,          // out
                             double           & rdStartAngDeg,     // out
                             double           & rdEndAngDeg) const // out
{
  rReferenceFrame = m_vPosition;
  rdRadius        = m_dRadiusAtXAxis;
  rdStartAngDeg   = m_vAnalDomain.GetMin();
  rdEndAngDeg     = m_vAnalDomain.GetMax();

  return TRUE;

} // end SmCircle::IsArc

/*******************************************************************//**
PURPOSE: Create a simple offset of this SmCircle.

NOTES: Just copy and change the radius, with checks:
   - Offset is to the right, looking down from above the normal.  So:
     - If the given normal is parallel to our z-axis, a positive outside increases the radius
     - If anti-parallel, the opposite
   - Else (not parallel or anti-parallel), this defaults to the SmBSplineCurve version.
   - If the resulting radius is negative, SM_ERR is returned.
   - If the resulting radius is zero, a degenerate curve is returned.
***********************************************************************/
SmStatus SmCircle::CreateSimpleOffset
 (const SmContext  & crContext,
  double             dApproxTol3d,
  const SmVector3d & crOffsetPlaneNormal,
  double             dOffsetDistance,
  SmBSplineCurve  *& rpNewBSplineCurve,
  double           & rdMaxGap3d,
  SmBoolean          bOptMatchParameterization )
 const
{
  // Init outputs
  rpNewBSplineCurve = NULL;
  rdMaxGap3d = 0;

  double dMyRadius = m_dRadiusAtXAxis;
  double dOffsetRad;

  SmVector3d sMyZAxis = m_vPosition.GetZAxis();
  if     (crOffsetPlaneNormal.IsParallelTo( sMyZAxis)) { dOffsetRad = dMyRadius + dOffsetDistance ; }
  else if(crOffsetPlaneNormal.IsParallelTo(-sMyZAxis)) { dOffsetRad = dMyRadius - dOffsetDistance ; }
  else
    {
      // Out-of-plane offset would not be a circle.
      // (Actually, it would be an ellipse, and it could be returned as
      // an SmEllipse if we were to find a reason to do that calculation.)
      return SmBSplineCurve::CreateSimpleOffset(crContext,
                                                dApproxTol3d,
                                                crOffsetPlaneNormal,
                                                dOffsetDistance,
                                                rpNewBSplineCurve,
                                                rdMaxGap3d,
                                                bOptMatchParameterization) ;
    }

  if     (dOffsetRad < -dApproxTol3d) { return SM_ERR; }
  else if(dOffsetRad <  dApproxTol3d) { // Offset is a degenerate curve.
                                        SER(SmBSplineCurve::CreateDegenerateCurve(crContext,
                                                                                  3,
                                                                                  m_vPosition.GetOriginRef(),
                                                                                  rpNewBSplineCurve)) ;
                                        NER(rpNewBSplineCurve);
                                        return SM_SUCCESS;
                                      }

  // Ok, all is good.  Create an SmCircle for output.
  SmCurve *pOffsetCurve = NULL ;
  SER(this->Copy(crContext, pOffsetCurve));  NER(pOffsetCurve) ;

  SmCircle *pOffsetCircle = SM_CAST_PTR(SmCircle, pOffsetCurve) ;
  if(pOffsetCircle != NULL) { pOffsetCircle->SetRadius(dOffsetRad) ;
                              pOffsetCircle->MakeNurb() ;
                            }
  else                      { delete pOffsetCurve ; pOffsetCurve = NULL ;
                              return SmBSplineCurve::CreateSimpleOffset(crContext,
                                                                        dApproxTol3d,
                                                                        crOffsetPlaneNormal,
                                                                        dOffsetDistance,
                                                                        rpNewBSplineCurve,
                                                                        rdMaxGap3d,
                                                                        bOptMatchParameterization) ;
                            }

  rpNewBSplineCurve = pOffsetCircle ;

  //cbi maybe?
  if(rpNewBSplineCurve == NULL) { rpNewBSplineCurve = SM_CAST_PTR(SmBSplineCurve, pOffsetCurve) ; }

  // all done
  return( rpNewBSplineCurve != NULL ? SM_SUCCESS
                                    : SM_ERR ) ;

} /// end SmCircle::CreateSimpleOffset

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertCircle_list[] =
{
  {SM_AT_RADIUS,       _T("Radius"), _T("m_dRadiusAtXAxis and m_dRadiusAtYAxis are equal") }
} ;

/*******************************************************************//**
PURPOSE:  Check that Circle Analytic and Nurb representations are
             equivalent.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmCircle::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init return value
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmEllipse::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // Check that analytic ellipse is a circle
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, SM_ARE_SAME(m_dRadiusAtXAxis, m_dRadiusAtYAxis), SM_EFF_ZERO, m_dRadiusAtXAxis - m_dRadiusAtYAxis, _T("")) ;

  // all done
  return(bRtn) ;

} // end SmCircle::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmCircle::AssertHeal
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
//       return ( SmEllipse::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmCircle::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmCircle::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmCircle to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmCircle::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // class SmCircle has no data of its own to write

  // Write the base object
  SmEllipse::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS ;

} // end SmCircle::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCircle from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmCircle::ReadFromDB
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
         || rpNewCurve->IsKindOf(SmCircle_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCircle *pCircle =   (rpNewCurve == NULL)
                      ? new (crContext) SmCircle()
                      : (SmCircle *)rpNewCurve ;

  // SmCircle class has no data of its own to read

  // set the output
  rpNewCurve = pCircle ;

  // read the base type
  SmEllipse::ReadFromDB(SmEllipse_TYPE, rDB, lDim, crContext, rpNewCurve, lDBVersionNumber) ; 

  // all done
  return SM_SUCCESS;

} // end SmCircle::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCircle::IsKindOf( SM_TYPE t ) const
{
  return ((SmCircle_TYPE == t) ? TRUE : SmEllipse::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Circle data out for debugging.

NOTES: 
***********************************************************************/
void SmCircle::Dump () const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmCircle::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  // output dump
  smos_sprintf(sBuff,       _T("\nSmCircle = 0x%p"), this) ;
  smos_sprintf(sBuffForFile,_T("\nSmCircle = %s"), _T("notNULL") );
  smos_WriteBuffer(sBuff, sBuffForFile);

  // get the parent's dump
  SmEllipse::Dump();

  smos_WriteBuffer(_T(" End SmCircle::Dump()\n")) ;

} // end SmCircle::Dump
