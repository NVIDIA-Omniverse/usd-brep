// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLine.cpp
* PURPOSE: Source file for SmLine methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmLine.h>
#include <SmVector2d.h>
#include <SmPseudoBox.h>
#include <SmPolarBox.h>
#include <SmExtent3d.h>
#include <SmEllipse.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <nurbs.h>
#include <SmNurbsCrv.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#include <SmSurfOfExtrusion.h>   // get parameter constraints from owner surface in MakeNurb
#include <SmSurfOfRevolution.h>  // get parameter constraints from owner surface in MakeNurb


#ifdef SM_DEBUG_CODE
#include <SmEdge.h>
#include <SmBrep.h>
#endif

/*******************************************************************//**
PURPOSE: Create a line segment between two points.

NOTES:  Nurb Domain = Analytic Domain = [0.0 1.0]
***********************************************************************/
SmLine::SmLine
 (const SmPoint3d & crStartPoint,       // in : Start of line = Start + s*(End-Start), for s:[0 1]
  const SmPoint3d & crEndPoint,         // in : End   of line = Start + s*(End-Start), for s:[0 1]
                                        //      will be unitized before storing
  ULONG             lDimension,         // in : line image space size, default:[3]
  const SmContext * cpContext)          // in : context for this object construction
 : SmBSplineCurve(lDimension)
{
  // init line parameters
  SmVector3d sVec   = crEndPoint - crStartPoint;
  double     dScale = sVec.Length();
  SmExtent1d sIvl(0.0, 1.0);

  // watch out for degenerate lines
  if(dScale < SM_EFF_ZERO) { sVec.Set(1,0,0) ; 
                             dScale = 1.0 ;
                             sIvl.SetMinMax(0.0, 0.0) ;
                           }
  else                     { sVec /= dScale;  // Unitize.
                           }

  // when passed a context - use it
  if(cpContext) { SM_ASSERT(   GetContext() == NULL 
                            || GetContext() == cpContext) ; 
                  SetContext(cpContext) ; 
                }
  SM_ASSERT(GetContext() != NULL) ;

  // build line
  m_vLinePoint  = crStartPoint ;
  m_vLineVector = sVec ;
  m_vAnalDomain = sIvl ;
  m_dScale      = dScale ;
  m_bInsideOut  = FALSE ;

} // end SmLine::SmLine 2 pt line constructor

/*******************************************************************//**
PURPOSE: Create an infinite line from a given Point and a Vector.

NOTES: The given vector determines the speed of parameterization.
    Therefore, it is not assummed to be a unit vector.
    
  When declaring automatic objects:
    SmLine(sPoint, sVector) ;  FAILS beacause m_pContext = NULL
    SmLine(sPoint, sVector, 2/3, TRUE/FALSE, cpContext) ;  SUCCEEDS

  When declaring heap objects:
    SmLine *pLine = new (cpContex) SmLine(sPoint, sVector) ;  SUCCEEDS

***********************************************************************/
SmLine::SmLine
  (const SmPoint3d      & crLinePoint,         // in : P of line = P + s*V 
   const SmVector3d     & crNonUnitVector,     // in : V of line = P + s*V
   ULONG                  lDimension,          // in : 
   SmBoolean              bMakeNurb,           // NotUsed: in : FALSE=just doing evals, skip build for performance
   const SmContext      * cpContext,           // in : required for automatic variable with bMakeNurb == TRUE
                                               //      not needed when built with overladed new because value is already set.
                                               //      default:[NULL]
   SmBoolean              bInsideOut)          // in : Only needed for Cone compatibility
                                               //      TRUE = Nurb Tangent = -LineVector
                                               //      FALSE= Nurb Tangent =  LineVector
                                               //      default:[FALSE]
: SmBSplineCurve(lDimension),
  m_vAnalDomain(-SM_INFINITE_PARAMETER,SM_INFINITE_PARAMETER),
  m_bInsideOut(bInsideOut)
{
  SM_REF1(bMakeNurb) ; 
  double dLength = crNonUnitVector.Length();
  SM_ASSERT(dLength > SM_EFF_ZERO);

  m_vLinePoint  = crLinePoint;
  m_vLineVector = crNonUnitVector/dLength ;
  m_dScale      = dLength;

  // when passed a context - use it
  if(cpContext) { SM_ASSERT(   GetContext() == NULL 
                            || GetContext() == cpContext) ; 
                  SetContext(cpContext) ; 
                }
  SM_ASSERT(GetContext() != NULL) ;

  // make the associated NURB curve
  // gwc: always make the m_pNurb object
  //      if (bMakeNurb) 
  //        {
  //          // requires this->GetContext != NULL
  //          SM_ASSERT(GetContext() != NULL) ;
  //          SE(MakeNurb());
  //        }

  // When object has a context - make the m_pNurb
  if(GetContext() != NULL)  
    { SE(MakeNurb()) ; }

  m_eBSplineCurveForm = SM_CF_POLYLINE_FORM;

} // end SmLine::SmLine constructor

/*******************************************************************//**
PURPOSE: Create a bounded line from a given point, a unitized
    vector, a parameter domain and the 'speed' of parameterization.

NOTES:
  Calls Unitize on input Vector before storing it, so though it should
  be a unitized vector it can be input with a non unit length.
  If the input vector is zero-length, then an arbitrary direction is
  selected for the curve and the AnalDomain is modified to make this a degenerate
  curve.

  When declaring automatic objects:
    SmLine(sPoint, sUnitVec, sAnalDomain, dScale) ;  FAILS because m_pContext = NULL
    SmLine(sPoint, sUnitVec, sAnalDomain, dScale, 2_or_3, TRUE_or_FALSE, cpContext) ;  SUCCEEDS

  When declaring heap objects:
    SmLine *pLine = new (cpContex) SmLine(sPoint, sUnitVec, sAnalDomain, dScale) ;  SUCCEEDS

***********************************************************************/
SmLine::SmLine          
  (const SmPoint3d      & crLinePoint,       // in : P     of line = P + s*scale*unitV 
   const SmVector3d     & crUnitVector,      // in : V     of line = P + s*scale*unitV
                                             //      will be unitized before storing
   const SmExtent1d     & crAnalDomain,      // in : limits on s, crAnalDomain.Min <= s <= crAnalDomain.Max
   double                 dScale,            // in : scale of line = P + s*scale*unitV        
   ULONG                  lDimension,        // in : sizeof LinePoint and LineVector, default:[3]
   const SmContext      * cpContext,         // in : required when making an automatic variable
                                             //      optionally when using overloaded new.
   const SmBSplineCurve * pOptNurb,          // in : Optional Nurb copied to make m_pNurb object
                                             //      If Given, its StartPoint == bInsideOut ? StepEndPoint   : StepStartPoint
                                             //                its EndPoint   == bInsideOut ? StepStartPoint : StepEndPoint
                                             //      default:[NULL]
   SmBoolean              bInsideOut)        // in : Only needed for Cone compatibility
                                             //      TRUE = Nurb Tangent = -LineVector
                                             //      FALSE= Nurb Tangent =  LineVector
                                             //      default:[FALSE]
 : SmBSplineCurve(lDimension), 
   m_vAnalDomain(crAnalDomain),
   m_dScale(dScale),
   m_bInsideOut(bInsideOut)  
{
  // force m_vLineVector to be a unit vector
  double dLength = crUnitVector.Length() ;
   
  // set line position, direction, and scale
  m_vLinePoint  = crLinePoint;
  if(dLength > SM_EFF_ZERO)
    {
      m_vLineVector = crUnitVector/dLength ;
    }
  else
    {
      SM_DBG_WARN(_T("Constructing bounded SmLine with zero-length"));

      // give line an arbitrary direction and shrink AnalDomain down to a degenerate point
      m_vLineVector.Set(1,0,0) ;
      m_vAnalDomain.SetMinMax(m_vAnalDomain.GetMin(), m_vAnalDomain.GetMin()) ;
    }

  // when passed a context - use it
  if(cpContext) { SM_ASSERT(   GetContext() == NULL 
                            || GetContext() == cpContext) ; 
                  SetContext(cpContext) ; 
                }
  SM_ASSERT(GetContext() != NULL) ;

  // make the associated NURB curve
  if(pOptNurb) { gw_CURVE *pGwNurbCurve = ((SmBSplineCurve *)pOptNurb)->GetOrCreateGwNurbPointer() ;

                 // copy and save the gw_CURVE 
                 SetFromGwNurb(0, pGwNurbCurve) ;
               }
  else         { SE(MakeNurb()); 
               }

  // set form factor
  m_eBSplineCurveForm = SM_CF_POLYLINE_FORM;

} // end SmLine::SmLine constructor

/*******************************************************************//**
PURPOSE: Create a line segment between two points.

NOTES:  Nurb Domain = Analytic Domain = [0.0 1.0]
***********************************************************************/
SmStatus SmLine::CreateLineSegment
  (const SmContext  & crContext,           // in : context for new object construction
   ULONG              lDimensionOfResult,  // in : 2 or 3
   const SmPoint3d  & crStartPoint,        // in : LineStartPoint
   const SmPoint3d  & crEndPoint,          // in : LineEndPoint
   SmLine          *& rpNewLine,           // out: NewLine
   SmExtent1d       * pOptInterval)        // in : parameter range for the line
{
  // init line parameters
  SmVector3d sVec   = crEndPoint - crStartPoint;
  double     dScale = sVec.Length();
  SER(sVec.Unitize());
  SmExtent1d sIvl(0.0,1.0);

  // build line
  SmLine *pLine = new (crContext) SmLine(crStartPoint,sVec,sIvl,dScale,lDimensionOfResult);
  NER(pLine);

  // when asked
  if(pOptInterval)
    {
      // adjust the parameter range
      pLine->EditParameterization( *pOptInterval ) ; 
    }

  // all done - set output
  rpNewLine = pLine;
  return SM_SUCCESS;

} // end SmLine::CreateLineSegment

/*******************************************************************//**
PURPOSE: Copy constructor for SmLine

NOTES: 
***********************************************************************/
SmLine::SmLine
  (const SmLine & crSource)                 // in : target surface to copy
 : SmBSplineCurve(crSource),
   m_vLinePoint(crSource.m_vLinePoint),
   m_vLineVector(crSource.m_vLineVector),
   m_vAnalDomain(crSource.m_vAnalDomain),
   m_dScale(crSource.m_dScale),
   m_bInsideOut(crSource.m_bInsideOut)
{
} // end SmLine::SmLine copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmLine

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmLine::operator==
  (const SmCurve& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmBSplineCurve::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmLine &rOther = (SmLine &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_vLinePoint  == rOther.m_vLinePoint
              && m_vLineVector == rOther.m_vLineVector
              && m_vAnalDomain == rOther.m_vAnalDomain
              && SM_IS_ZERO(m_dScale - rOther.m_dScale)
              && m_bInsideOut  == rOther.m_bInsideOut) ;                       
    }

  // all done
  return bRtn ;

} // end SmLine::operator==

/*******************************************************************//**
PURPOSE: Set curve's STEP-interval to given interval

NOTES: Changing the STEP-interval changes the shape of the line
  by changing which segment of the analytically defined infinite
  line is represented by this SmLine object.
***********************************************************************/
SmStatus SmLine::AdjustSTEPInterval
  (const SmExtent1d & crNewSTEPInterval)
{
  // no work - two intervals are the same
  if(   crNewSTEPInterval.IsContainedBy(m_vAnalDomain, SM_EFF_ZERO)
     && m_vAnalDomain.IsContainedBy(crNewSTEPInterval, SM_EFF_ZERO))
     { return SM_SUCCESS; }

  // set the domain
  m_vAnalDomain = crNewSTEPInterval;

  // rebuild the Nurb Curve
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  if(m_pNurb) 
    { SER(MakeNurb()); }
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  return SM_SUCCESS;

} // end SmLine::AdjustSTEPInterval

/*******************************************************************//**
PURPOSE: Return Length from start to end of given interval
NOTES: Not approximate....exact
***********************************************************************/
double SmLine::ApproximateLength
 (const SmExtent1d & crInterval,       // in : curve interval to query
  ULONG              lSampleCnt,       // NotUsed: in : number of smp pts including end points, 5 is a reasonable number
  double           * pOptUVTurnAngDeg) // out: 2d Curves only. Optional UV Space turning angle. 0.0 for 3dCurves
 const
{
  SM_REF1(lSampleCnt) ; 
  // init output
  if(pOptUVTurnAngDeg) { *pOptUVTurnAngDeg = 0.0 ; }

  // get interval endPoints
  SmPoint3d sStartPoint, sEndPoint;
  EvaluatePoint(crInterval.GetMin(),sStartPoint);
  EvaluatePoint(crInterval.GetMax(),sEndPoint);

  // return distance between points
  double dRet = sStartPoint.DistanceBetween(sEndPoint);
  return dRet;

} // end SmLine::ApproximateLength

/*******************************************************************//**
PURPOSE: Convert the line to a 2D line.

NOTES: Do all the assignments because this function gets used
       by constructors to set control point Z values appropriately
***********************************************************************/
SmStatus SmLine::ConvertTo2D
  ()
{
  // project STEP params to XY plane
  m_lDim          = 2;
  m_vLinePoint.z  = 0.0;
  m_vLineVector.z = 0.0;

  // convert the underlying Nurb
  SmBSplineCurve::ConvertTo2D() ;

  // all done
  return SM_SUCCESS;

} // end SmLine::ConvertTo2D

/*******************************************************************//**
PURPOSE: Convert the line to a 3D line.

NOTES: 
***********************************************************************/
SmStatus SmLine::ConvertTo3D
  ()
{
  if (GetDim() == 3) { return SM_SUCCESS; }
  m_lDim = 3;

  // convert the underlying Nurb
  SmBSplineCurve::ConvertTo3D() ;

  // all done
  return SM_SUCCESS;

} // end SmLine::ConvertTo3D

/*******************************************************************//**
PURPOSE: Convert from STEP to NURBS parameterization.

NOTES: 
***********************************************************************/
SmStatus SmLine::ConvertTFromSTEPToNURBS
  (double   dSTEPParam,    // in : Analytic Domain parameter
   double & rdNURBSParam)  // out: Nurb Domain parameter            
  const
{
  SmExtent1d sNurbIvl = GetNaturalInterval() ;

  // convert linear coordinate
  if(IsBounded()) { rdNURBSParam =   m_bInsideOut
                                   ? (  sNurbIvl.GetMax() 
                                      + (  (dSTEPParam - m_vAnalDomain.GetMin()) 
                                         /  m_vAnalDomain.GetLength() 
                                         * -sNurbIvl.GetLength()))
                                   : (  sNurbIvl.GetMin() 
                                      + (  (dSTEPParam - m_vAnalDomain.GetMin()) 
                                         / m_vAnalDomain.GetLength() 
                                         * sNurbIvl.GetLength())) ;
                  }
  else            { rdNURBSParam =   m_bInsideOut
                                   ? -dSTEPParam
                                   :  dSTEPParam ;
                  }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // check conversions reciprocity
  double sCheckParam ;
  ConvertTFromNURBSToSTEP(rdNURBSParam, sCheckParam) ; 
  double dTDist       = dSTEPParam - sCheckParam ;
  double dScaledTZero = SM_EFF_ZERO * 1000.0 * (1.0 + dTDist) ;

  SM_ASSERT(dTDist < dScaledTZero) ;

  // check quality of the conversion
  SmPoint3d sSTEPPoint, sNurbPoint ;
  Evaluate    (rdNURBSParam, 0, TRUE, &sNurbPoint) ;
  EvaluateSTEP(dSTEPParam,   0, TRUE, &sSTEPPoint) ;
  double dDist = (sNurbPoint - sSTEPPoint).Length() ;
  double dScaledZero = SM_EFF_ZERO * (1.0 + sNurbPoint.GetMaxDimension()) ;

  SM_ASSERT(dDist < dScaledZero) ;
  if(bDebugMe)
    {
      Dump() ;
    }
#endif

  // all done
  return SM_SUCCESS;    

} // end SmLine::ConvertTFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert from NURBS to STEP parameterization.

NOTES: 
***********************************************************************/
SmStatus SmLine::ConvertTFromNURBSToSTEP
  (double   dNURBSParam,  // in : Nurb Domain parameter
   double & rdSTEPParam)  // out: Analytic Domain parameter
  const
{
  // swap NurbPoint when needed
  SmExtent1d sNurbIvl = GetNaturalInterval();
  
  // convert linear coordinate
  SmBoolean bIsBounded = IsBounded() ;
  if(bIsBounded)  { double dNurbsLength = sNurbIvl.GetLength() ;
                    if(SM_IS_ZERO(dNurbsLength))
                      { rdSTEPParam = dNURBSParam ; }
                    else
                      {
                        rdSTEPParam =   m_vAnalDomain.GetMin()
                                      + (m_bInsideOut ? ((dNURBSParam - sNurbIvl.GetMax()) / -sNurbIvl.GetLength() * m_vAnalDomain.GetLength())
                                                      : ((dNURBSParam - sNurbIvl.GetMin()) /  sNurbIvl.GetLength() * m_vAnalDomain.GetLength())) ;
                      }
                  } 
  else            { rdSTEPParam =   m_bInsideOut
                                  ? -dNURBSParam
                                  :  dNURBSParam ;
                  }

  // all done                                
  return SM_SUCCESS;
   
} // end SmLine::ConvertTFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Virtual Copy method

NOTES: 
***********************************************************************/
SmStatus SmLine::Copy
  (const SmContext & crContext,    // in : 
   SmCurve        *& rpNewCurve)   // out: 
  const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  SmLine *pCopy = new (crContext) SmLine(*this); NER(pCopy) ;
  rpNewCurve    = pCopy ;
  SM_DUMP_AND_ASSERT2_VALID(rpNewCurve) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
       Dump() ;
       rpNewCurve->Dump() ;
    }
#endif
  return SM_SUCCESS;

} // end SmLine::Copy

/*******************************************************************//**
PURPOSE: Method to create a canonical line object.

NOTES: 
  Analytic Domain = Nurb Domain = [-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER]
  dScale          = crLineVector.Length() ;              
***********************************************************************/
SmStatus SmLine::CreateCanonical
  (const SmContext  & crContext,      // in : context for new object construction
   const SmPoint3d  & crLinePoint,    // in : Origin of Line
   const SmVector3d & crUnitVector,   // in : Unitized Vector
   SmLine *& rpNewLine)               // out: new object
{
  // make the unbounded line and return
  rpNewLine = new (crContext) SmLine(crLinePoint, crUnitVector);
  return SM_SUCCESS;

} // end SmLine::CreateCanonical

/*******************************************************************//**
PURPOSE: Create a simple offset of this SmLine.

NOTES: 
   - Offset is to the right, looking down from above the normal,
     so the offset direction is the line vector crossed with the normal.
   - If the line has no component in the direction of the normal,
     then the result would be a degenerate curve, however no offset
     direction is defined, therefore SM_INVALID_INPUT is returned.

   An SmLine is returned in rpNewBSplineCurve.
***********************************************************************/
  SmStatus SmLine::CreateSimpleOffset(const SmContext  & crContext,                   // in:
                                      double             dApproxTol3d,                // in: Not used here.
                                      const SmVector3d & crOffsetPlaneNormal,         // in:
                                      double             dOffsetDistance,             // in:
                                      SmBSplineCurve  *& rpNewBSplineCurve,           // out:
                                      double           & rdMaxGap3d,                  // out: Always 0.0 here.
                                      SmBoolean          bOptMatchParameterization    // in, opt: Not used here.
                                ) const
{
  // Init outputs
  rpNewBSplineCurve = NULL;
  rdMaxGap3d = 0;

  // The offset direction is the line vector crossed with the normal.
  SmVector3d sOffDir = m_vLineVector * crOffsetPlaneNormal;

  SmStatus eStat = sOffDir.Unitize( FALSE );  // False: do not print warning if zero vector.
  if ( eStat != SM_SUCCESS )
    { return SM_ERR_INVALID_INPUT; }  // Let the caller print warning if warranted.

  
  // Ok, all is good.  Create an SmLine for output.
  SmCurve *pOffsetCurve = NULL;
  SER( this->Copy( crContext, pOffsetCurve ));  NER( pOffsetCurve );

  SmLine *pOffsetLine = SM_CAST_PTR( SmLine, pOffsetCurve );
  if ( pOffsetLine != NULL )
  {
      pOffsetLine->m_vLinePoint += dOffsetDistance * sOffDir;
      pOffsetLine->MakeNurb();
  }
  else
  {
      delete pOffsetCurve;  pOffsetCurve = NULL;
      return SmBSplineCurve::CreateSimpleOffset( crContext, dApproxTol3d, crOffsetPlaneNormal,
                  dOffsetDistance, rpNewBSplineCurve, rdMaxGap3d, bOptMatchParameterization );
  }

  rpNewBSplineCurve = pOffsetLine;

  //cbi maybe?
  if ( rpNewBSplineCurve == NULL )
    { rpNewBSplineCurve = SM_CAST_PTR( SmBSplineCurve, pOffsetCurve ); }

  if ( rpNewBSplineCurve == NULL )
    { return SM_ERR; }

  return SM_SUCCESS;

} /// end SmLine::CreateSimpleOffset

/*******************************************************************//**
PURPOSE: Given an interval on the curve, compute the axis alligned or
    non-axis alligned bounding box.

NOTES: At least one of the outputs must be non-NULL.
    The returned bounding box is not increased.
***********************************************************************/
SmStatus SmLine::CalculateBoundingBox
  (const SmExtent1d & crNurbInterval,  // in : Desired Nurb Domain Interval
   SmExtent3d       * pNormalBox,      // out: Axis alligned box                       
   SmPseudoBox      * pPseudoBox,      // out: Non-axis aligned box                    
   SmPolarBox       * pPolarBox,       // out: Surface normal vector field bounding box
   SmBoolean)                          // in : bExpandBox = not-used
                                       //      FALSE = don't expand returned bounding box
  const
{ 
  SM_ASSERT(   pNormalBox != NULL 
            || pPseudoBox != NULL
            || pPolarBox  != NULL);

  // get interval endPoints
  SmPoint3d sStartPoint, sEndPoint;
  SER(EvaluatePoint(crNurbInterval.GetMin(),sStartPoint));
  SER(EvaluatePoint(crNurbInterval.GetMax(),sEndPoint));
  
  // Cumpute the basis vectors of the PseudoBox using the control polygon
  if (pPseudoBox) 
    {
      SmVector3d sV1 = sEndPoint - sStartPoint;

      // note: align pseudo box with lines tangent vector
      //       If the vector is zero length - 
      //       default basis vectors will be used.
      if (SM_IS_ZERO(sV1.LengthSquared())) 
        {
          // use default basis vectors
          *pPseudoBox = SmPseudoBox(); 
        }
      else
        {  
          SmVector3d sBasis1, sBasis2, sBasis3;
          sV1.MakeUnitOrthoVectors(NULL,sBasis1,sBasis2,sBasis3);
          pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
        }
    } // end init pseudoBox


  if(pNormalBox) { *pNormalBox = SmExtent3d(sStartPoint);
                   pNormalBox->AddPoint3d(sEndPoint);
                 }
  if(pPseudoBox) { pPseudoBox->AddPoint3d(sStartPoint);
                   pPseudoBox->AddPoint3d(sEndPoint);
                 }
  if(pPolarBox)  { pPolarBox->ReSet() ;
                   pPolarBox->AddVector3d(sEndPoint - sStartPoint) ;
                 }

  return SM_SUCCESS;

} // end SmLine::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Drop a point to an ellipse segment or its endPoints very fast.

NOTES: Only supports SM_SO_MINIMIZE and SM_SO_INTERSECT
   SM_SO_MINIMIZE:  find closest point 
                      to curve or if cpdOptTargetDistance is given
                      point within pdOptTargetDistance + dDistanceTolerance.
   SM_SO_INTERSECT: find closest point within dDistanceTolerance.
   
   For points within line segment - drop vector is perp to line
   For points beyond line segment - drop vector is to nearest segment endPoint 

RETURNS --- 
  SM_SUCCESS when operation is SM_SO_MINIMIZE or SM_SO_INTERSECT a
             and curve is a circle or circular arc, else
  SM_ERR     where GeneralPointSolve() will have to be called to get a solution.
***********************************************************************/
SmStatus SmLine::DropPointFast                  // rtn: SM_SUCCESS=case supported, SM_ERR=call GeneralPointSolve() to get solution
 (const SmExtent1d      & crNurbInterval,       // in : Nurb Domain of curve to search for solutions
  SmSolverOperationType   eSolverOperation,     // in : oneof: SM_SO_MINIMIZE =find closest point (more than one for closed curves)
                                                //             to curve or if cpdOptTargetDistance is given
                                                //             point within pdOptTargetDistance + dDistanceTolerance. 
                                                //             SM_SO_INTERSECT=find closest point within dDistanceTolerance.
  const SmPoint3d       & crTestPoint,          // in : target point
  const SmVector3d      * ,                     // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams
  double                  dDistanceTolerance,   // in : Skip Solutions whose drop distance is too far away
                                                //      operation == MINIMIZE save solution if cpdOptTargetDistance == NULL
                                                //                            or DropDist < cpdOptTargetDistance + dDistanceTolerance
                                                //      operation == INTERSECT save solution if DropDist < dDistanceTolerance
  const double          * cpdOptTargetDistance, // in : only used for operation Minimize.  When given
                                                //      skip solutions whose dropDist > cpdOptTargetDistance + dDistanceTolerance.
                                                //      else keep all solutions. 
  SmSolutionRequestedType ,                     // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions 
  SmSolutionArray       & rSolutions)           // out: array of problem solutions reported as Curve parameter values 
 const        
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmEdge *pEdge = (SmEdge *)GetEdge() ; 
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawParams(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0) ; crTestPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  // init output
  rSolutions.ReSet();

  // only good for MINIMIZE and INTERSECT operations
  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_INTERSECT) 
    { return SM_ERR; }
    
  // init output
  SmSolution sSol;
  sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
  sSol.m_lNumObjects   = 1;
  sSol.m_apObjects[0]  = SM_CONST_CAST(SmLine*,this);
  sSol.m_apNodes[0]    = NULL;
  sSol.m_lNumVariables = 1;

  // get distance from TestPoint to infinite line
  // get distance from TestPoint to line segment [StPnt,EndPnt].
  // Gives TestPoint/EndPoint distance for TestPoints beyond the ends of the segment. 
  double dDist, dAnalParam, dRawNurbParam, dNurbParam ;
  SER(smgu_LinePointDistance(m_vLinePoint,
                             m_vLineVector,
                             crTestPoint,
                             dDist,
                             &dAnalParam));

  // convert AnalParam to NurbParam
  dAnalParam /= m_dScale ;
  ConvertTFromSTEPToNURBS(dAnalParam, dRawNurbParam) ;

  // clip endPoints
  dNurbParam = crNurbInterval.ClampValue(dRawNurbParam) ;

  // when clamped
  if(dNurbParam != dRawNurbParam)
    {
      SmPoint3d sPoint ;
      EvaluatePoint(dNurbParam, sPoint) ;
      dDist = sPoint.DistanceBetween(crTestPoint) ;
    } // end need to recompute Dist check

  // skip solutions when MINIMIZING   and sol greater than OptTargetDist + Tol
  //                when INTERSECTING and sol greater than Tol  
  if(   (   eSolverOperation == SM_SO_MINIMIZE  
         && cpdOptTargetDistance 
         && dDist > *cpdOptTargetDistance + dDistanceTolerance) 
     || (   eSolverOperation == SM_SO_INTERSECT  
          && dDist > dDistanceTolerance)) 
    {
      return SM_SUCCESS;
    }

  // keep solution
  sSol.m_vStart.m_dSolutionValue = dDist;
  sSol.m_vStart[0]               = dNurbParam ;
  rSolutions.Add(sSol);

  // all done
  return SM_SUCCESS;

} // end SmLine::DropPointFast

/*******************************************************************//**
PURPOSE: Compute the curvature of a curve at a given parameter

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
    Note: Radius Of Curvature = 1.0/Curvature
***********************************************************************/

SmStatus SmLine::EvaluateCurvature
  (double   dParameter,     // NotUsed: in : not used
   double & rCurvature)     // out: always zero
 const
{
  SM_REF1(dParameter) ; 
  rCurvature = 0.0 ;
  return(SM_SUCCESS) ;

} // end SmLine::EvaluateCurvature 

/*******************************************************************//**
PURPOSE: Get the interval of this line.

NOTES: 
***********************************************************************/
SmExtent1d SmLine::GetNaturalInterval() const
{ 
  // SM_ASSERT(m_pNurb != NULL) ;
  if(m_pNurb) 
    { return SmBSplineCurve::GetNaturalInterval() ; }
  else
    { return m_vAnalDomain ; }

} // end SmLine::GetNaturalInterval

/*******************************************************************//**
PURPOSE: Method to get canonical data.

NOTES:  Line(s) = LinePoint + s*LineVector so that
                 Line(0) = LinePoint
                 Line(1) = LinePoint + LineVector
***********************************************************************/
SmStatus SmLine::GetCanonical
  (SmPoint3d  & rLinePoint,       // out: Pt    of Line = Pt + u * Vec
   SmVector3d & rNonUnitVector,   // out: Vec   of Line = Pt + u * Vec, Vec is not usually unit length
   SmExtent1d * pOptAnalDomain,   // out: Valid range of parameter values u,    NULL to ignore
   SmBoolean  * pOptInsideOut)    // out: TRUE = Nurb and Analytic definitions run in opposite directions NULL to ignore
  const
{
  rLinePoint     = m_vLinePoint;
  rNonUnitVector = m_dScale*m_vLineVector;
  if(pOptAnalDomain) 
    { pOptAnalDomain->SetMinMax(m_vAnalDomain.GetMin(), m_vAnalDomain.GetMax()) ; }
  if(pOptInsideOut) { *pOptInsideOut = m_bInsideOut ; }

  // all done
  return SM_SUCCESS;

} // end SmLine::GetCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data.

NOTES:  Line(s) = LinePoint + s*dScale*LineUnitVector so that
                 Line(0) = LinePoint
                 Line(1) = LinePoint + dScale * LineUnitVector
***********************************************************************/
SmStatus SmLine::GetCanonical
  (SmPoint3d  & rLinePoint,       // out: Pt      of Line = Pt + u * Scale * UnitVec
   SmVector3d & rUnitVector,      // out: UnitVec of Line = Pt + u * Scale * UnitVec
   double     & rScale,           // out: Scale   of Line = Pt + u * Scale * UnitVec
   SmExtent1d & rAnalDomain,      // out: Valid range of parameter values u
   SmBoolean  * pOptInsideOut)    //  out: TRUE = Nurb and Analytic definitions run in opposite directions
                                  //      FALSE= Nurb and Analytic definitions run in same directions
                                  //      NULL to ignore, default:[NULL]
  const
{
  rLinePoint  = m_vLinePoint ;
  rUnitVector = m_vLineVector ;
  rScale      = m_dScale ;
  rAnalDomain.SetMinMax(m_vAnalDomain.GetMin(), m_vAnalDomain.GetMax()) ;
  if(pOptInsideOut) { *pOptInsideOut = m_bInsideOut ; }
  
  // all done
  return SM_SUCCESS;

} // end SmLine::GetCanonical

/*******************************************************************//**
PURPOSE: Method to set canonical data.

NOTES:  Line(s) = LinePoint + s* dScale * LineUnitVector so that
                 Line(0) = LinePoint
                 Line(1) = LinePoint + dScale * LineUnitVector
***********************************************************************/
SmStatus SmLine::SetCanonical
  (SmPoint3d  & rLinePoint,      // in : Pt    of Line = Pt + u * Scale * Vec                 
   SmVector3d & rUnitVector,     // in : Vec   of Line = Pt + u * Scale * Vec, Must be unit length                
   double       dScale,          // in : Scale of Line = Pt + u * Scale * Vec, Must be greater than 0.0
   SmExtent1d & rAnalDomain)     // in : Valid range of parameter values u,   
{
  // check input
  SM_ASSERT(SM_ARE_SAME(rUnitVector.Length(), 1.0)) ;
  SM_ASSERT(dScale > SM_EFF_ZERO) ;

  // set data
  m_vLinePoint  = rLinePoint ;
  m_vLineVector = rUnitVector ;
  m_dScale      = dScale ;
  m_vAnalDomain.SetMinMax(rAnalDomain.GetMin(), rAnalDomain.GetMax()) ;

  // Clean up the m_pNurb
  if(GetContext() != NULL)  
    { SE(MakeNurb()) ; }
  else if (m_pNurb) 
    { smos_Free(m_pNurb); m_pNurb = NULL; }

  // all done
  return SM_SUCCESS;

} // end SmLine::SetCanonical

/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the line.  This method can be used to find the 
     closest point on the curve to the given point; the farthest point 
     from the given point; all points on the curve where the vector 
     from point to curve is perpendicular to the tangent vector on the curve
     (normal points); all points of intersection where point is within
     tolerance of the curve; or the points on a curve at a given distance
     from the point.  Valid solver operations for this method include:
     SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT.
     The returned solutions contain angular parameters as
     defined in STEP Part-42.

NOTES: 
***********************************************************************/
SmStatus SmLine::GlobalPointSolveSTEP
  (const SmExtent1d        & crSTEPInterval,        // in : domain for solutions even when outside curve's domain
   SmSolverOperationType     eSolverOperation,      // in : oneof SM_SO_MINIMIZE 
                                                    //            SM_SO_MAXIMIZE 
                                                    //            SM_SO_NORMALIZE
                                                    //            SM_SO_INTERSECT
   const SmPoint3d         & crTestPoint,           // in : Target Point                                                     
   double                    dDistanceTolerance,    // in : Max allowed deviation for SM_SO_INTERSECT                                                     
   const double            * ,                      // in : cpdOptTargetDistance = Not Used                                                    
   const SmVector3d        * ,                      // in : cpOptVectors         = Not Used                                                    
   SmSolutionRequestedType   ,                      // in : eSolutionRequested   = Not Used                                                     
   SmSolutionArray         & rSolutions)            // out: Solution Param in StepDomain
                                                    //      Usually 1 solution
                                                    //      Possibly 2 for SM_SO_MAXIMIZE and
                                                    //        point is equaldistant from both endPoints                                                     
{
  // check input
  SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE 
            || eSolverOperation == SM_SO_MAXIMIZE 
            || eSolverOperation == SM_SO_NORMALIZE
            || eSolverOperation == SM_SO_INTERSECT);

  // init output
  rSolutions.ReSet();

  // get limited interval
  SmExtent1d sAnalIvl ;
  m_vAnalDomain.Intersect(crSTEPInterval, sAnalIvl) ;
  sAnalIvl.Union(crSTEPInterval, sAnalIvl) ;

  // Locals: Solution param, Solution distance, and temporary soloution object
  double dParam;
  double dDistanceToCurve;
  SmSolution sSol;

  // For maximize: Sol = furthest line endPoint, 2 solutions when equidistant to both endPoints
  if (eSolverOperation == SM_SO_MAXIMIZE) 
    {
      // get distances from TestPoint to Line Start and End Points
      SmPoint3d sP1, sP2;
      SER(EvaluateSTEP(sAnalIvl.GetMin(),0,TRUE,&sP1));
      SER(EvaluateSTEP(sAnalIvl.GetMax(),0,TRUE,&sP2));
      double dDist1 = crTestPoint.DistanceBetween(sP1);
      double dDist2 = crTestPoint.DistanceBetween(sP2);
      double dParam2, dDistance2 ;
      
      // set params for 1st and possible 2nd solutions
      if(dDist1 > dDist2) { dParam            = crSTEPInterval.GetMax();
                            dDistanceToCurve  = dDist2;
                            dParam2           = crSTEPInterval.GetMin();
                            dDistance2        = dDist1 ;
                          }
      else                { dParam            = crSTEPInterval.GetMin();
                            dDistanceToCurve  = dDist1;
                            dParam2           = crSTEPInterval.GetMax();
                            dDistance2        = dDist2 ;
                          }

      // when endPoint distances are equal - output 2 solutions
      if(SM_ARE_SAME_TO_TOL(dDist1, dDist2, dDistanceTolerance))
        {
          // output a 2nd solution
          sSol.m_eSolutionType           = SM_ST_SINGLE_VALUE;
          sSol.m_lNumObjects             = 1;
          sSol.m_apObjects[0]            = SM_CONST_CAST(SmLine*,this);
          sSol.m_apNodes[0]              = NULL;
          sSol.m_lNumVariables           = 1;
          sSol.m_vStart[0]               = dParam2 ;
          sSol.m_vStart.m_dSolutionValue = dDistance2 ;
          rSolutions.Add(sSol);
        }

    } // end maximize point/line distance
  else // SolverOperation == MINIMIZE, NORMALIZE, INTERSECT
    {
      // get nearest LinePoint to TestPoint
      SmVector3d sLineVec = m_dScale * m_vLineVector;
      SER(smgu_LineClosestPoint(m_vLinePoint,sLineVec,crTestPoint,dParam));

      // when solution is beyond line bounds - clamp or quit
      if (!sAnalIvl.ContainsValue(dParam)) 
        {
          if (eSolverOperation == SM_SO_NORMALIZE) 
            {
              return SM_SUCCESS; // No answer found
            }
          dParam = crSTEPInterval.ClampValue(dParam);
        }

      // Get the LinePoint/TestPoint distance
      SmPoint3d sP1;
      SER(EvaluateSTEP(dParam,0,TRUE,&sP1));
      double dDist1 = crTestPoint.DistanceBetween(sP1);

      // when intersecting - no solutions when dist > tol
      if (   eSolverOperation == SM_SO_INTERSECT
          && dDist1 > dDistanceTolerance) 
        {
          return SM_SUCCESS; // No answer found
        }

      // set distance value
      dDistanceToCurve = dDist1;
    }

  // output a solution
  sSol.m_eSolutionType           = SM_ST_SINGLE_VALUE;
  sSol.m_lNumObjects             = 1;
  sSol.m_apObjects[0]            = SM_CONST_CAST(SmLine*,this);
  sSol.m_apNodes[0]              = NULL;
  sSol.m_lNumVariables           = 1;
  sSol.m_vStart[0]               = dParam;
  sSol.m_vStart.m_dSolutionValue = dDistanceToCurve;
  rSolutions.Add(sSol);

  // all done
  return SM_SUCCESS;

} // end SmLine::GlobalPointSolveSTEP

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmLine::Evaluate
 (double     dNurbParam,              // in : tgt param
  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
  SmBoolean  bFromLeft,               // NotUsed: in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d aPointAndDerivatives[],  // out: array or (pos, tang, 2nd deriv, ...), sized:[lNumDerivatives+1]
  SmBoolean  )                        // in : bNonZeroTangents : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
    const                             //                         FALSE= return exact tangent values
                                      //      note: Lines are either degenerate or have nonZero tangents - so not used here.
                                      //      default:[TRUE]
{ 
  SM_REF1(bFromLeft) ; 
  // convert to STEP parameter
  double dSTEPParam  ;
  ConvertTFromNURBSToSTEP(dNurbParam, dSTEPParam) ;

  // Set 2nd order and higher derivatives to zero
  for (ULONG i=2; i<lNumDerivatives+1; i++) 
    {
      aPointAndDerivatives[i].x = 0.0;
      aPointAndDerivatives[i].y = 0.0;
      aPointAndDerivatives[i].z = 0.0;
    }

  // position
  aPointAndDerivatives[0] = m_vLinePoint + dSTEPParam * m_vLineVector * m_dScale;

  // 1st Derivative
  if (lNumDerivatives >= 1) 
    {
      SmExtent1d sNurbIvl = GetNaturalInterval() ;
      double     dJacobi  = m_vAnalDomain.GetLength() / sNurbIvl.GetLength() ;
      aPointAndDerivatives[1] =
          m_bInsideOut ? -m_vLineVector * m_dScale * dJacobi
                       :  m_vLineVector * m_dScale * dJacobi ;
    }

  return SM_SUCCESS;

} // end SmLine::Evaluate

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmLine::EvaluateSTEP
  (double dSTEPParam,                 // in : target STEP parameter to evaluate
   ULONG lNumDerivatives,             // in : 0 - produces Euclidian point only - see SmCurve::EvaluatePoint
                                      //      1 - produces first derivative and point
                                      //      2 - produces second derivative, first derivative and point.
                                      //      N - produces N-th derivative and lower derivatives
   SmBoolean bFromLeft,               // NotUsed: in : bFromLeft = if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d aPointAndDerivatives[], // out: array of [position, 1st deriv, 2nd deriv, ...] values.
                                      //      expected size:[lNumDerivatives+1]
   SmZoneTol3d dZoneTol3d)            // NotUsed: in : Unused
  const
{ 
  SM_REF2(bFromLeft, dZoneTol3d) ; 
  // Set 2nd order and higher derivatives to zero
  ULONG ii;
  for (ii=2; ii<lNumDerivatives+1; ii++) 
    {
      aPointAndDerivatives[ii].x = 0.0;
      aPointAndDerivatives[ii].y = 0.0;
      aPointAndDerivatives[ii].z = 0.0;
    }

  // position
  aPointAndDerivatives[0] = m_vLinePoint + dSTEPParam * m_vLineVector * m_dScale;

  // 1st Derivative
  if (lNumDerivatives >= 1) 
    {
      aPointAndDerivatives[1] = m_vLineVector * m_dScale;
    }

  return SM_SUCCESS;

} // end SmLine::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmLine::EvaluatePoint
  (double dNurbParam,       // in : Target Nurb Parameter
  SmPoint3d & rPoint)       // out: euclidean point
 const
{ 
  // convert to STEP parameter
  double dSTEPParam  ;
  ConvertTFromNURBSToSTEP(dNurbParam, dSTEPParam) ;

  rPoint = m_vLinePoint + dSTEPParam * m_dScale * m_vLineVector ;
  return SM_SUCCESS;

} // end SmLine::EvaluatePoint

//      /*******************************************************************//**
//      PURPOSE: This is a fast evaluator for a line that is inline.  It 
//          assumes that the parameter is clampped or that you are evaluating
//          an infinite line.
//      
//      NOTES: 
//      ***********************************************************************/
//      inline SmStatus SmLine::EvaluatePointFast
//        (double dParameter, 
//        SmPoint3d & rPoint) 
//       const
//      {
//        rPoint = m_vLinePoint + dParameter * m_vLineVector * m_dScale;
//        return SM_SUCCESS;
//      
//      } // end SmLine::EvaluatePointFast

/*******************************************************************//**
PURPOSE: Fast IsLine check for SmLine objects

NOTES: 
***********************************************************************/
SmBoolean SmLine::IsLine
  (ULONG ,                         // in : lNumberOfSamplePoints = not used in this function
   double ,                        // in : dTol = not used in this function
   SmPoint3d  & rLinePoint,        // out: Start Point of Line
   SmVector3d & rLineVector)       // out: LineVec = (EndPoint - StartPoint)
  const
{
  // set output
  EvaluateSTEPPoint(m_vAnalDomain.GetMin(),rLinePoint);
  rLineVector =  m_vAnalDomain.IsBounded() 
               ? m_vAnalDomain.GetLength() * m_dScale * m_vLineVector
               : m_dScale * m_vLineVector ;

  // all done
  return TRUE;

} // end SmLine::IsLine

/*******************************************************************//**
PURPOSE: Determine if a Line is bounded
NOTES: unbounded when param limits equal +/-SM_INFINITE_PARAMETER
***********************************************************************/
SmBoolean SmLine::IsBounded() const
{ 
  // bounded when neither end is infinite
  return( m_vAnalDomain.IsBounded() ) ;

} // end SmLine::IsBounded

/*******************************************************************//**
PURPOSE: Determine if a line is degenerate to a point.  

NOTES: 
***********************************************************************/
SmBoolean SmLine::IsDegenerate
  (double            d3DTolerance,    // in : min distance between unique points, 0 = use dScaledZero
                                      //      default:[SM_EFF_ZERO]
   const SmExtent1d *pInterval)       // in : interval to examine, NULL = use Natural Interval       
                                      //      default:[NULL]
  const
{
  double dTol = smos_Max( d3DTolerance, SM_EFF_ZERO );
  double dDist;
  
  // get line length in 3D
  if ( pInterval != NULL )
    {
        SM_ASSERT( pInterval->IsContainedBy( GetNaturalInterval(), SM_EFF_ZERO_PARAM ));
#ifdef SM_DEBUG_CODE 
        if (!pInterval->IsContainedBy(GetNaturalInterval(), SM_EFF_ZERO_PARAM))
        {
            TCHAR sBuff[SM_TBLOCK_SIZE];
            smos_sprintf(sBuff, _T("SmLine::IsDegenerate:  pInterval %16.16lf to %16.16lf\n"), pInterval->GetMin(),pInterval->GetMax());
            smos_WriteBuffer(sBuff);
            smos_sprintf(sBuff, _T("SmLine::IsDegenerate:  NInterval %16.16lf to %16.16lf\n"), GetNaturalInterval().GetMin(), GetNaturalInterval().GetMax());
            smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE

      double dStepMin, dStepMax;
      ConvertTFromNURBSToSTEP( pInterval->GetMin(), dStepMin );
      ConvertTFromNURBSToSTEP( pInterval->GetMax(), dStepMax );

      dDist = m_dScale * smos_Fabs( dStepMax - dStepMin );
    }
  else
    {
      dDist = m_dScale * m_vAnalDomain.GetLength();
    }

  // return degenerate when line is shorter than tolerance
  return ( dDist < dTol );

} // end SmLine::IsDegenerate

/*******************************************************************//**
PURPOSE: Determine if a curve is on a boundary of the given domain.

NOTES: 
   This is intended for 2d (uv) curves, but no check is made.
   Any z component is ignored.

   Returns one of: SM_SP_UMIN, SM_SP_UMAX, SM_SP_VMIN, SM_SP_VMAX, SM_SP_NEITHER.

   The curve need not cover the entire boundary, just lie entirely on it.
***********************************************************************/
SmSurfParamType SmLine::IsDomainBoundary
   ( const SmExtent2d &rDomain, // in : the given 2d domain
           double      dTol )   // in : optional, default=SM_EFF_ZERO
 const
{
  SmSurfParamType eRet = SM_SP_NEITHER;

  if ( smos_Fabs( m_vLineVector.x ) < dTol )
    {
      if ( smos_Fabs( m_vLinePoint.x - rDomain.GetUMin() ) < dTol )
        { eRet = SM_SP_UMIN; }
      if ( smos_Fabs( m_vLinePoint.x - rDomain.GetUMax() ) < dTol )
        { eRet = SM_SP_UMAX; }
    }
  if ( smos_Fabs( m_vLineVector.y ) < dTol )
    {
      if ( smos_Fabs( m_vLinePoint.y - rDomain.GetVMin() ) < dTol )
        { eRet = SM_SP_VMIN; }
      if ( smos_Fabs( m_vLinePoint.y - rDomain.GetVMax() ) < dTol )
        { eRet = SM_SP_VMAX; }
    }

  return eRet;

} // end SmLine::IsDomainBoundary

/*******************************************************************//**
PURPOSE: Intersect a Line with an Ellipse

NOTES: 
***********************************************************************/
SmStatus SmLine::IntersectWithEllipse
  (const SmExtent1d & crInterval,               // in : line interval in Nurb Domain
   const SmEllipse  & crOtherCurve,             // in : other curve to intersect
   const SmExtent1d & crOtherInterval,          // in : other curve interval
   double             dDistanceTolerance,       // in : Find points where curves are within this 3D distance
   SmBoolean        & rbNeedsMoreIntersections, // out: TRUE = pass call to general curve/curve intersector 
                                                //      FALSE= intersections found here
   SmSolutionArray  & rSolutions)               // out: solutions
  const                                 
{
  // init output
  rSolutions.ReSet();

  // pass the call to SmEllipse::IntersectWithLine
  SER(crOtherCurve.IntersectWithLine(crOtherInterval, 
                                     *this, crInterval,
                                     dDistanceTolerance,
                                     rbNeedsMoreIntersections,
                                     rSolutions));

  // set output - for every solution
  for (ULONG i=0; i<rSolutions.GetSize(); i++) 
    {
      SmSolution & rSol = rSolutions[i];

      // swap the object and parameter orders
      SmObject *pObj      = rSol.m_apObjects[0] ;
      rSol.m_apObjects[0] = rSol.m_apObjects[1] ;
      rSol.m_apObjects[1] = pObj ;

      double dVal         = rSol.m_vStart[0];
      rSol.m_vStart[0]    = rSol.m_vStart[1];
      rSol.m_vStart[1]    = dVal;

      if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES) 
        {
          SM_DBG_WARN(_T("Ellipse/Line intersection found a range_of_values solution")) ;
          double dVal2   = rSol.m_vEnd[0];
          rSol.m_vEnd[0] = rSol.m_vEnd[1];
          rSol.m_vEnd[1] = dVal2;
        }
    } // end iter every solution

  // all done
  return SM_SUCCESS;

} // end SmLine::IntersectWithEllipse

/*******************************************************************//**
PURPOSE: Intersect Two Lines analytically

NOTES:  solutions: sSol.m_vStart[0] = this param 
                            sSol.m_vStart[1] = other param
***********************************************************************/
SmStatus SmLine::IntersectWithLine
  (const SmExtent1d & crInterval,                 // in : line interval in Nurb Domain                                        
   const SmLine     & crOtherCurve,               // in : other line to intersect                             
   const SmExtent1d & crOtherInterval,            // in : other line interval in Nurb Domain                                 
   double             dDistanceTolerance,         // in : Find points where curves are within this 3D distance 
   SmBoolean        & rbNeedsMoreIntersections,   // out: TRUE = pass call to general curve/curve intersector  
                                                  //      FALSE= intersections found here                      
   SmSolutionArray  & rSolutions)                 // out: solutions: sSol.m_vStart[0] = this param
                                                  //                 sSol.m_vStart[1] = other param                                             
  const
{
  // init output
  rSolutions.ReSet();

  // locals
  const SmCurve *pThis  = this;
  const SmCurve *pOther = &crOtherCurve;
  
  // Self-intersection test.
  if (pThis == pOther) { rbNeedsMoreIntersections = FALSE;
                         return SM_SUCCESS;
                       }
  
  // Get Line endPoints and vecs
  SmPoint3d sStartPoint1, sEndPoint1 ;
  SmPoint3d sStartPoint2, sEndPoint2 ;
  SER(EvaluatePoint(crInterval.GetMin(),sStartPoint1));
  SER(EvaluatePoint(crInterval.GetMax(),sEndPoint1));
  SER(crOtherCurve.EvaluatePoint(crOtherInterval.GetMin(),sStartPoint2));
  SER(crOtherCurve.EvaluatePoint(crOtherInterval.GetMax(),sEndPoint2));
  SmVector3d sVec1 = sEndPoint1-sStartPoint1 ;

  // check for disjoint solution
  SmExtent3d sBBox1(sStartPoint1);
  sBBox1.AddPoint3d(sEndPoint1);
  sBBox1.ExpandAbsolute(dDistanceTolerance);
  SmExtent3d sBBox2(sStartPoint2);
  sBBox2.AddPoint3d(sEndPoint2);
  if (sBBox1.AreDisjoint(sBBox2)) 
    {
      rbNeedsMoreIntersections = FALSE; 
      return SM_SUCCESS;
    }

  // intersect the 2 segments
  SmVector3d sVec2 = sEndPoint2-sStartPoint2;
  ULONG lNumInt = 0;
  SmPoint3d sIntPnts[2];
  if (   sVec1.LengthSquared() > SM_EFF_ZERO_SQ 
      && sVec2.LengthSquared() > SM_EFF_ZERO_SQ) 
    {
      // lNumInt: 0=No xSect, 1=Point xSect, 2=Coincident segment
      SER(smgu_SegmentSegmentIntersect(sStartPoint1, sEndPoint1,
                                       sStartPoint2, sEndPoint2,
                                       dDistanceTolerance,
                                       lNumInt, sIntPnts));
    }

  SmSolution sSol;
  sSol.m_lNumObjects = 2 ;
  sSol.m_apObjects[0] = (SmObject *)this ;
  sSol.m_apObjects[1] = (SmObject *)&crOtherCurve ;

  // when 1 or more solutions were found
  if (lNumInt > 0) 
    {
      // find params for 1st solution
      double dParam1, dParam2;
      SER(smgu_LineClosestPoint(sStartPoint1, sVec1, sIntPnts[0], dParam1));
      SmPoint3d sIntPnt1 = sStartPoint1 + dParam1 * sVec1;
      SER(smgu_LineClosestPoint(sStartPoint2,sVec2,sIntPnts[0],dParam2));
      SmPoint3d sIntPnt2 = sStartPoint2 + dParam2 * sVec2;

      // set sSol to be a single value solution
      sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
      sSol.m_lNumVariables = 2;
      sSol.m_vStart[0] = crInterval.Evaluate(dParam1);
      sSol.m_vStart[1] = crOtherInterval.Evaluate(dParam2);
      sSol.m_vStart.m_dSolutionValue = sIntPnt1.DistanceBetween(sIntPnt2);
    }

  // when segments are coincident 
  if (lNumInt > 1) 
    {
      // find params for solution
      double dParam1, dParam2;
      SER(smgu_LineClosestPoint(sStartPoint1,sVec1,sIntPnts[1],dParam1));
      SmPoint3d sIntPnt1 = sStartPoint1 + dParam1 * sVec1;
      SER(smgu_LineClosestPoint(sStartPoint2,sVec2,sIntPnts[1],dParam2));
      SmPoint3d sIntPnt2 = sStartPoint2 + dParam2 * sVec2;

      // change sSol to be a range solution
      sSol.m_eSolutionType = SM_ST_RANGE_OF_VALUES;
      sSol.m_vEnd[0] = crInterval.Evaluate(dParam1);
      sSol.m_vEnd[1] = crOtherInterval.Evaluate(dParam2);
      double dDist = sIntPnt1.DistanceBetween(sIntPnt2);
      sSol.m_vEnd.m_dSolutionValue = dDist;

      // Sort by parameter values of first curve
      if (sSol.m_vStart[0] > sSol.m_vEnd[0]) 
        { 
          // Swap
          sSol.m_vEnd[0] = sSol.m_vStart[0];
          sSol.m_vEnd[1] = sSol.m_vStart[1];
          sSol.m_vEnd.m_dSolutionValue = sSol.m_vStart.m_dSolutionValue;
          sSol.m_vStart[0] = crInterval.Evaluate(dParam1);
          sSol.m_vStart[1] = crOtherInterval.Evaluate(dParam2);
          sSol.m_vStart.m_dSolutionValue = dDist;
        }
    }

  // set output
  if (lNumInt > 0) 
    {
      rSolutions.Add(sSol); 
    }
  rbNeedsMoreIntersections = FALSE;

  // all done
  return SM_SUCCESS;

} // end SmLine::IntersectWithLine

/*******************************************************************//**
PURPOSE: Join the other curve onto this curve.  

NOTES: 
  The result is that this curve is modified.
  This will work only if the resulting curve can be an SmLine,
    otherwise return SM_ERR.  This requires that the other curve is linear,
    and is collinear with this line (and has coincident end points of course).
  The other curve need not have the same parameterization as this curve:
    the speed of the result will match that of this line.

  [debugging note: B54; B182]
***********************************************************************/
SmStatus SmLine::JoinWith
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
      smgfx_Erase();
      smgfx_SetLook( 2,4, 1,0,0 ); DrawWDeriv(GetNaturalInterval(),0); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,0,1 ); pOtherCurve->DrawWDeriv(pOtherCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // We'll need these:
  SmExtent1d sIvl1 = this       ->GetNaturalInterval();
  SmExtent1d sIvl2 = pOtherCurve->GetNaturalInterval();

  double dThisLen;
  this->Length( sIvl1, 0.0, dThisLen );  // 0.0: tol: not used.

  // Get a tolerance to use.
  double dTol = ( pGapTolerance != NULL )
                      ? *pGapTolerance
                      : dThisLen * SM_EFF_ZERO_SQRT * 1000.0;

  // Make sure that the points of join ends are close enough.
  // Grab all end points here.
  SmPoint3d  sThisCommonPt,  sThisEndPt;
  SmPoint3d sOtherCommonPt, sOtherEndPt;
  this   ->    GetEnds(  sThisCommonPt,  sThisEndPt );
  pOtherCurve->GetEnds( sOtherCommonPt, sOtherEndPt );

  // Sort out which are the common point vs. the new end points.
  if ( lJoinEndThis == 1 )
    {
      SmPoint3d sTmp = sThisCommonPt;
      sThisCommonPt = sThisEndPt;
      sThisEndPt = sTmp;
    }
  if ( lJoinEndOther == 1 )
    {
      SmPoint3d sTmp = sOtherCommonPt;
      sOtherCommonPt = sOtherEndPt;
      sOtherEndPt = sTmp;
    }

  // Check join-point distance.
  if ( sThisCommonPt.DistanceBetween( sOtherCommonPt ) > dTol )
    { return SM_ERR; }

  // Other curve must be linear.
  SmPoint3d sTmpPt, sTmpVec;
  if ( ! pOtherCurve->IsLine( 5, dTol, sTmpPt, sTmpVec ) )
    { return SM_ERR; }

  // And it must be collinear with us.
  // We'll check that by checking the distance between the common point
  // from the line connecting what will be the end points;, i.e.,
  // what will be this line.  (Makes more sense than an angle check.)
  SmVector3d sWholeLineVec( sOtherEndPt - sThisEndPt );
  double dDist;
  SER( smgu_LinePointDistance( sThisEndPt, sWholeLineVec, sThisCommonPt, dDist ));
  if ( dDist > dTol )
    { return SM_ERR; }

  // Passed all tests, good to go.  Modify ourself.

  // For joining at the end of this line, we just update the end parameter.
  // For joining at our start, we could modify either our start parameter,
  // or our start point and our end parameter.
  // Well leave the point as it is and change the start parameter.

  // Length of other curve's param range will be ours, scaled by the 3d lengths.
  // Work with the analytical domain, then update the Nurbs to that.
  double dOtherLen    = sOtherCommonPt.DistanceBetween( sOtherEndPt );
  SmExtent1d sStepIvl = this->GetSTEPInterval();
  double dDeltaParam  = sStepIvl.GetLength() * dOtherLen / dThisLen;

  // Join ends refer to NURBS parameters. For an inside-out line, its NURBS
  // start is the analytic end, so extend the opposite end of the STEP interval.
  SmBoolean bExtendSTEPMin = (lJoinEndThis == 0) != m_bInsideOut;
  double dNewParam = bExtendSTEPMin ? sStepIvl.GetMin() - dDeltaParam
                                    : sStepIvl.GetMax() + dDeltaParam;
  m_vAnalDomain.AddValue( dNewParam );

  // Don't update the start point, we did the parameters instead.
  // m_vLinePoint = ( lJoinEndThis == 0 ) ? sOtherEndPt : sThisEndPt;

  // And update the Nurbs.
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  this->MakeNurb();
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmLine::JoinWith

/*******************************************************************//**
PURPOSE: Given an interval on a curve compute the length.  

NOTES:  
***********************************************************************/
SmStatus SmLine::Length
  (const SmExtent1d & crInterval,        // in : Nurb Domain of interest
   double             dDesiredAccuracy,  // NotUsed: in : not used
   double           & rdLength)          // out: length of line for given interval
  const
{
  SM_REF1(dDesiredAccuracy) ; 
  // check for degenerate intervals 
  if (crInterval.GetLength() < SM_EFF_ZERO) 
    {
      rdLength = 0.0;
      return SM_SUCCESS;
    }

  
  if (m_pNurb)
  {
       // get interval endPoints
       SmPoint3d sStartPoint, sEndPoint;
       EvaluatePoint(crInterval.GetMin(),sStartPoint);
       EvaluatePoint(crInterval.GetMax(),sEndPoint);
       rdLength = sStartPoint.DistanceBetween(sEndPoint);
  }
  else     
  { // else return curve length = IntervalLength * Scale
     rdLength = crInterval.GetLength() * m_dScale ;
  }
  return SM_SUCCESS ;

} // end SmLine::Length

/*******************************************************************//**
PURPOSE: Make the Nurb representation corresponding to a bounded line

NOTES: the m_bInsideOut flag specifies if the
                the Nurb parameterization runs in the same or opposite
                direction of the AnalDomain.
                 
  When m_bInsideOut = FALSE,  NurbDomain = StepDomain and
                              NurbStartPoint = StepStartPoint
                              NurbEndPoint   = StepEndPoint

       m_bInsideOut = TRUE,   NurbDomain = StepDomain but
                              NurbStartPoint = StepEndPoint
                              NurbEndPoint   = StepStartPoint
***********************************************************************/
SmStatus SmLine::MakeNurb()
{
  // free existing m_pNurb pointer if any
  if (m_pNurb) { smos_Free(m_pNurb); m_pNurb = NULL; }

  // get min/max points
  SmPoint3d sStartPoint, sEndPoint;
  if(m_bInsideOut) { SER(EvaluateSTEPPoint(m_vAnalDomain.GetMax(), sStartPoint)) ;
                     SER(EvaluateSTEPPoint(m_vAnalDomain.GetMin(), sEndPoint)) ;
                   }
  else             { SER(EvaluateSTEPPoint(m_vAnalDomain.GetMin(), sStartPoint)) ;
                     SER(EvaluateSTEPPoint(m_vAnalDomain.GetMax(), sEndPoint)) ;
                   }

  // allocate new nurb curve
  const SmContext *pContext = GetContext();
  if (pContext == NULL) 
    return (SM_ERR);
  gw_CURVE *pNewCur = sm_AllocateNurbCurve(1,1,3);

  // set naturalInterval = analDomain Interval
  pNewCur->knt->U[0] = pNewCur->knt->U[1] = m_vAnalDomain.GetMin();
  pNewCur->knt->U[2] = pNewCur->knt->U[3] = m_vAnalDomain.GetMax();

  // set ControlPoints = [MinPoint, MaxPoint] - not rational
  COPY_XYZ(sStartPoint,pNewCur->pol->Pw[0]); pNewCur->pol->Pw[0].w = NL_NOW;
  COPY_XYZ(sEndPoint,  pNewCur->pol->Pw[1]); pNewCur->pol->Pw[1].w = NL_NOW;

  // save the Nurb pointer
  SM_ASSERT(m_pNurb == NULL) ;
  m_pNurb = pNewCur;

  // Set ControlPoint Z values for 2d curves
  if( m_lDim == 2)
    {
      this->ConvertTo2D() ;
    }

  // when a SmSurfOfExtrusion GenCurve, look for Parent Surface NURB Interval constraint
  if(m_pOwner && m_pOwner->IsKindOf(SmSurfOfExtrusion_TYPE))
    {
      SmExtent1d sNURBIvl = ((SmSurfOfExtrusion*)m_pOwner)->GetGenDirParamExtent() ;
      EditParameterization(sNURBIvl, FALSE) ; // FALSE = No Need for Notify
    }

  // when a SmSurfOfRevolution GenCurve, look for Parent Surface NURB Interval constraint
  if(m_pOwner && m_pOwner->IsKindOf(SmSurfOfRevolution_TYPE))
    {
      SmExtent1d sNURBIvl = ((SmSurfOfRevolution*)m_pOwner)->GetGenDirParamExtent() ;
      EditParameterization(sNURBIvl, FALSE) ; // FALSE = No Need for Notify
    }

  // all done
  return SM_SUCCESS;

} // end SmLine::MakeNurb

/*******************************************************************//**
PURPOSE: Refresh the STEP Parameters from the NURB parameters

NOTES: Should be called whenever one changes the shape
  of the line by editing the Nurb representations, either moving control
  points or modifying break point values.

  Preserves the current m_bInsideOut value

  Sets m_vAnalDomain = Nurb->GetNaturalInterval()
  Sets m_dScale      = 1.0 ;
  Sets m_vLinePoint  = if  (m_bInsideOut   == FALSE) Nurb->Evaluate(0.0) 
                       else(m_bInsideOut   == TRUE)  
  Sets m_vLineVector = unit vector such that 
    if(m_bInsideOut   == FALSE) Step(VMin) = Nurb(VMin) and
                                Step(vMax) = Nurb(VMax)
    else(m_bInsideOut == TRUE)  Step(VMax) = Nurb(VMin) and
                                Step(vMin) = Nurb(VMax)    

************************************************************************/
SmStatus SmLine::RefreshAnalytics()
{
  // check state - expected to be a finite curve
  SM_ASSERT(IsBounded()) ;

  // refresh AnalDomain
  m_vAnalDomain = GetNaturalInterval() ;

  // get Nurb endPoints
  SmPoint3d sStartPoint, sEndPoint ;
  GetEnds(sStartPoint, sEndPoint) ;

  // switch on m_bInsideOut
  if(m_bInsideOut == FALSE)
    {
      m_vLineVector = (sEndPoint - sStartPoint) ;
      m_dScale      = m_vLineVector.Length() / m_vAnalDomain.GetLength() ;
      if(m_dScale < SM_EFF_ZERO) { m_dScale = 1.0 ; }
      m_vLineVector.Unitize() ;
      m_vLinePoint = sStartPoint - m_vLineVector * m_dScale * m_vAnalDomain.GetMin() ;
#ifdef SM_DEBUG_CODE
      if(!sEndPoint.CloserThan(SM_EFF_ZERO * fabs(  1.0
                                                    + m_vLinePoint.GetMaxDimension() 
                                                    + m_vLineVector.GetMaxDimension() * (1.0 + m_vAnalDomain.GetMax()) ),
                                     m_vLinePoint + m_dScale * m_vAnalDomain.GetMax() * m_vLineVector))
        {
          SM_ASSERT(sEndPoint.CloserThan(SM_EFF_ZERO * fabs(  1.0
                                                        + m_vLinePoint.GetMaxDimension() 
                                                        + m_vLineVector.GetMaxDimension() * (1.0 + m_vAnalDomain.GetMax()) ),
                                         m_vLinePoint + m_dScale * m_vAnalDomain.GetMax() * m_vLineVector)) ;
        }
#endif // SM_DEBUG_CODE
    
    }
  else
    {
      m_vLineVector = (sStartPoint - sEndPoint) ;
      m_dScale      = m_vLineVector.Length() / m_vAnalDomain.GetLength() ;
      if(m_dScale < SM_EFF_ZERO) { m_dScale = 1.0 ; }
      m_vLineVector.Unitize() ;
      m_vLinePoint = sEndPoint - m_vLineVector * m_dScale * m_vAnalDomain.GetMin() ;
#ifdef SM_DEBUG_CODE
      if(!sStartPoint.CloserThan(SM_EFF_ZERO * (  1.0 
                                                          + m_vLinePoint.GetMaxDimension() 
                                                          + m_vLineVector.GetMaxDimension() * (1.0 + m_vAnalDomain.GetMax()) ),
                                           m_vLinePoint + m_dScale * m_vAnalDomain.GetMax() * m_vLineVector))
        {
          SM_ASSERT(sStartPoint.CloserThan(SM_EFF_ZERO * (  1.0 
                                                          + m_vLinePoint.GetMaxDimension() 
                                                          + m_vLineVector.GetMaxDimension() * (1.0 + m_vAnalDomain.GetMax()) ),
                                           m_vLinePoint + m_dScale * m_vAnalDomain.GetMax() * m_vLineVector)) ;
        }
#endif // SM_DEBUG_CODE

    }
    
 SM_DUMP_AND_ASSERT2_VALID(this) ; 

 // all done
 return(SM_SUCCESS) ;

} // end SmLine::RefreshAnalytics

/*******************************************************************//**
PURPOSE: Reverse the parameterization of a curve and update an 
    interval on the curve.

NOTES: 
***********************************************************************/
SmStatus SmLine::ReverseParameterization
  (const SmExtent1d & crOldInterval,   // in : current curve interval in Nurb Domain       
   SmExtent1d       & rNewInterval)    // out: curve interval after reversal
{
  // set output interval
  rNewInterval = crOldInterval;

  // reverse analytic curve

  // when bounded
  if(IsBounded())
    {
      //                                                  Max + Min
      //                                                      |
      // switch from   +Start-------Min---------Max----------->
      // to            <------------Max---------Min------Start+
      // switch startPoint to current endPoint position  
      // This effectively mirrors the Start point to its corresponding
      // position on the other side of the AnalDomain.  The AnalDomain
      // remains the same.
      SmPoint3d sNewStartPoint;
      SER(EvaluateSTEPPoint(  m_vAnalDomain.GetMax()
                            + m_vAnalDomain.GetMin(),
                            sNewStartPoint));
      m_vLinePoint  = sNewStartPoint;
    }

  // negate the tangent vector
  m_vLineVector = - m_vLineVector;

  // reverse Nurb curve
  if (m_pNurb) { SmBSplineCurve::ReverseParameterization(crOldInterval,rNewInterval) ; }

  // all done
  return SM_SUCCESS;

} // end SmLine::ReverseParameterization

// gwc: no need for SmLine version of EditParameterization
//       This implementation was a mistake.  It was
//       changing both the STEP and the NURB intervals together.
//       The EditParameterization() call should only edit the
//       m_pNurb parameterization and if that is all that it does
//       then the SmBSplineCurve::EditParameterization() method
//       is just fine for SmLines.
// OBSOLETE MISTAKE
//   /*******************************************************************//**
//   PURPOSE: Change NURB parameterization of curve to given extent range 
//             - no Analytic-STEP range change  
//   
//   NOTES: 
//   ***********************************************************************/
//   SmStatus SmLine::EditParameterization
//     (const SmExtent1d & crNewParameterization, // in : new parameter range for curve
//      SmBoolean          bNotify)               // in : TRUE  = make notify calls (previous behavior)
//                                                //      FALSE = Skip notify call
//                                                //      default:[TRUE]
//   {
//     // get current interval
//     SmExtent1d sOldIvl = GetNaturalInterval();
//   
//     // low work: current interval == new interval
//     if (   SM_ARE_SAME(sOldIvl.GetMin(),crNewParameterization.GetMin())
//         && SM_ARE_SAME(sOldIvl.GetMax(),crNewParameterization.GetMax()) ) 
//       { return SM_SUCCESS; }
//   
//     // to adjust, change m_vLinePoint, m_dScale, m_vAnalDomain
//     // let LinePoint1 = LinePoint0 + v * LineVector ;
//     // PtMin = LinePoint0 + Scale0 * UMin0 * LineVector = LinePoint1 + Scale1 * UMin1
//     // PtMax = LinePoint0 + Scale0 * UMax0 * LineVector = LinePoint1 + Scale1 * UMax1
//     // which yields matrix eqn for v and Scale 1
//     //  [1 uMin1] [   v    ] = [ Scale0 * uMin0 ]
//     //  [1 uMax1] [ Scale1 ]   [ Scale0 * uMax0 ]
//     // which solves as
//     //  [   v    ] = [ uMax1 -uMin1] [ Scale0 * uMin0 ]
//     //  [ Scale1 ]   [  -1      1  ] [ Scale0 * uMax0 ] / (uMax1 - uMin1)
//   
//     double dDet = crNewParameterization.GetLength() ;
//     if(SM_IS_ZERO(dDet))
//       {
//         SER_MSG(SM_ERR,_T("SmLine::EditParameterization input error - given extent cannot be degenerate")) ;
//       }
//     double dV      = (+ crNewParameterization.GetMax() * m_vAnalDomain.GetMin()
//                       - crNewParameterization.GetMin() * m_vAnalDomain.GetMax()) * m_dScale / dDet ;
//     double dScale1 = (- 1 * m_vAnalDomain.GetMin()
//                       + 1 * m_vAnalDomain.GetMax()) * m_dScale / dDet ;
//   
//     // change the line representation
//     m_dScale      = dScale1 ;
//     m_vLinePoint  = m_vLinePoint + dV * m_vLineVector ;
//     m_vAnalDomain = crNewParameterization ;
//   
//     // rebuild the Nurb
//     MakeNurb() ;
//   
//     // Notify the BSpline
//     if(bNotify == TRUE)
//       {
//         Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
//       }
//   
//   #ifdef SM_DEBUG_CODE
//   SmBoolean bDebugMe = FALSE ;
//     if(bDebugMe)
//       {
//         SmBoolean bOK = SM_ASSERT_VALID_NO_STREAM(this) ;
//         if(!bOK)
//           {
//             bOK = SM_ASSERT_VALID_NO_STREAM(this) ; // place for a break
//           }
//       }
//   #endif // SM_DEBUG_CODE
//   
//     // all done
//     return SM_SUCCESS;
//   
//   } // end SmLine::EditParameterization
// end gwc removed mistaken implementation

/*******************************************************************//**
PURPOSE: Given a point on the line, find its analytic parameter

NOTES: This method finds the analytic parameter of the
  nearest Line point to TargetPoint. The returned param is not
  guaranteed to be within the analytic domain
***********************************************************************/
SmStatus SmLine::STEPInversion
  (const SmPoint3d      & crPointOnCurve,       // in : Target Point
   double               & rdAnalyticParameter,  // out: STEPParameter closest to Target Point
   SmCurveLocationType  * pOptLoc,              // out: Point's classification to curve
                                                //      NULL to ignore, default:[NULL]
   SmZoneTol3d            dZoneTol3d)           // NotUsed: in : Unused
  const
{
  SM_REF1(dZoneTol3d) ; 
  // find u to minimize |Line(u) - P|
  //  where Line(u) = LinePoint + u * dSacle * LineVector
  SmVector3d sLineVec = m_dScale * m_vLineVector;
  SER(smgu_LineClosestPoint(m_vLinePoint,
                            sLineVec,
                            crPointOnCurve,
                            rdAnalyticParameter));

  // when asked - classify the point
  if(pOptLoc)
    {
      // check that point maps to inside of line 
      *pOptLoc =   m_vAnalDomain.ContainsValue(rdAnalyticParameter) 
                 ? SM_CL_INTERIOR
                 : SM_CL_EXTERIOR ;

      // check distance to point
      if(*pOptLoc == SM_CL_INTERIOR)
        {
          SmPoint3d sPoint ;
          EvaluateSTEPPoint(rdAnalyticParameter, sPoint) ;
          double dDist = sPoint.DistanceBetween(crPointOnCurve) ;
          double dScaledZero = SM_EFF_ZERO * (1.0 + sPoint.GetMaxDimension()) ;

          // when dist is too large - point is not on the line
          if(dDist > dScaledZero)
            { *pOptLoc = SM_CL_EXTERIOR ; }
        }
    } // end need to classify point check

  // all done
  return SM_SUCCESS;

} // end SmLine::STEPInversion

/*******************************************************************//**
PURPOSE: Tessellate a curve using either chord height and/or angular
    tessellation tolerance.

NOTES: If one of the tolerances are zero then that tolerance is
    not used in the calculation.  Both of the tolerances must not be zero.
    At least one of the outputs must be a non-NULL pointer.
***********************************************************************/
SmStatus SmLine::Tessellate
  (const SmExtent1d     & crInterval,      // in : line interval in Nurb domain
   double,                                 // in : dChordHeightTolerance - NOT USED
   double,                                 // in : dAngleTolDeg - NOT USED
   ULONG     lMinimumNumberOfSegments,     // in : Min number of segments in this tessellation 
   SmTArray<double>     * pOptParameters,  // out: Opt Nurb Parameters for each tessellation point NULL to ignore
   SmTArray<SmPoint3d>  * pOptPoints,      // out: Every tessellation point NULL to ignore, default:[NULL]
   SmTArray<SmVector3d> * pOptTangents)    // out: Optional array of tangent points for each sample point NULL to ignore
   const
{
  // when asked - build evenly spaced param point array 
  if (pOptParameters) 
    { 
      // start with Nurb Min Param
      pOptParameters->ReSet();
      pOptParameters->Add(crInterval.GetMin());

      // for every interval
      for (ULONG i=1; i<lMinimumNumberOfSegments; i++) 
        {
          double dT = crInterval.Evaluate(  (double)i
                                          / (double)lMinimumNumberOfSegments) ;
          pOptParameters->Add(dT);
        }
      pOptParameters->Add(crInterval.GetMax());

    } // end Optional Parameters check 

  // just build array of point samples
  if(    pOptPoints
     && !pOptTangents)
    {
      // locals
      SmTArray<SmPoint3d> sLocalPoints ;
      SmTArray<SmPoint3d> *pLocalPoints   =    pOptPoints 
                                            ?  pOptPoints 
                                            : &sLocalPoints ;
      // init arrays
      pLocalPoints->ReSet();

      // add start point
      SmPoint3d sPnt;
      SER(Evaluate(crInterval.GetMin(),0,TRUE,&sPnt));
      pLocalPoints->Add(sPnt) ;

      // for every segment - add midPoint
      for (ULONG i=1; i<lMinimumNumberOfSegments; i++) 
        {
          double dT = crInterval.Evaluate((i*1.0)/lMinimumNumberOfSegments);
          SER(Evaluate(dT,0,TRUE,&sPnt));
          pLocalPoints->Add(sPnt) ;
        }

      // add endPoint
      SER(Evaluate(crInterval.GetMax(),0,TRUE,&sPnt));
      pLocalPoints->Add(sPnt) ;

    } // end pOptPoints or pOptTangents check

  // build array of point and tangent samples
  else if(   pOptPoints
          || pOptTangents)
    {
      // locals
      SmTArray<SmPoint3d> sLocalPoints, sLocalTangents ;
      SmTArray<SmPoint3d> *pLocalPoints   =    pOptPoints 
                                            ?  pOptPoints 
                                            : &sLocalPoints ;
      SmTArray<SmPoint3d> *pLocalTangents =    pOptTangents 
                                            ?  pOptTangents 
                                            : &sLocalPoints ;

      // init arrays
      pLocalPoints->ReSet();
      pLocalTangents->ReSet();

      // add start point
      SmPoint3d sPnt[2];
      SER(Evaluate(crInterval.GetMin(),1,TRUE,sPnt));
      pLocalPoints->Add(sPnt[0]) ;
      pLocalTangents->Add(sPnt[1]) ;

      // for every segment - add midPoint
      for (ULONG i=1; i<lMinimumNumberOfSegments; i++) 
        {
          double dT = crInterval.Evaluate((i*1.0)/lMinimumNumberOfSegments);
          SER(Evaluate(dT,1,TRUE,sPnt));
          pLocalPoints->Add(sPnt[0]) ;
          pLocalTangents->Add(sPnt[1]) ;
        }

      // add endPoint
      SER(Evaluate(crInterval.GetMax(),1,TRUE,sPnt));
      pLocalPoints->Add(sPnt[0]) ;
      pLocalTangents->Add(sPnt[1]) ;

    } // end pOptPoints or pOptTangents check

  // all done
  return SM_SUCCESS;

} // end SmLine::Tessellate

/*******************************************************************//**
PURPOSE: Scale and transform an SmLine curve.
NOTES: Non-Uniform scaling is implemented for lines 
                but not for other analytical curves.
***********************************************************************/
SmStatus SmLine::Transform
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

  // Get Anal StartPoint and Line(1.0) point.
  // Eval at 1.0 because that will be used to calculate m_dScale,
  // which is the speed, or distance per unit parameter.
  SmPoint3d sStartPoint = m_vLinePoint;
  SmPoint3d sEndPoint;
  EvaluateSTEPPoint( 1.0, sEndPoint );

  // scale the points
  if (cpOptScale) 
    {
      sStartPoint = sStartPoint.Multiply(*cpOptScale);
      sEndPoint   = sEndPoint.Multiply(*cpOptScale);
    }    

  // rotate and move the points
  crRotateNMove.TransformPoint(sStartPoint, sStartPoint);
  crRotateNMove.TransformPoint(sEndPoint, sEndPoint);

  // reset the analytic parameters
  m_vLinePoint  = sStartPoint;
  m_vLineVector = sEndPoint - sStartPoint;
  m_dScale      = m_vLineVector.Length();
  m_vLineVector.Unitize();    

  // transform the nurb curve
  if (m_pNurb) 
    {
      Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
      SER(SmBSplineCurve::Transform(crRotateNMove,cpOptScale));
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  // all done
  return SM_SUCCESS;

} // end SmLine::Transform

/*******************************************************************//**
PURPOSE: Deform line smoothly to force its endPoints to interpolate given
   EndPt and EndTan tgts while preserving the general shape of this curve.
  

NOTES:
    1. Method behavior depends on the eTgtStartTanType and eTgtEndTanType requests as:
        Both EndTans are SM_IV_UNCONSTRAINED - Edits this Crv in-place to EndTgts, *ppOptNewCurve = NULL
        Either EndTan is SM_IV_SAME          - Builds new BSplineCurve to EndTgts, *ppOptNewCurve = NewBSplineCurve 
        Either EndTan is SM_IV_SPECIFIED     - Builds new BSplineCurve to EndTgts, *ppOptNewCurve = NewBSplineCurve 
***********************************************************************/
SmStatus SmLine::DeformToEndPointTargets
 (SmPoint3d     * pTgtStartPt,         // in :  NotNULL = Curve StartPt new Tgt position 
                                       //       NULL    = leave as is
  SmPoint3d     * pTgtEndPt,           // in :  NotNULL = Curve EndPt new Tgt position
                                       //       NULL    = leave as is
  SmCurve      ** ppOptNewCurve,       // out: Ptr to new curve when editing shape changes this curve's type
                                       //      Not used when this curve can be edited in place without changing its type.
                                       //      default:[NULL]. Returns Err if not given when its needed
  SmVector3d    * pOptTgtStartTan,     // in : optional Start 1stDir tangent dir (NonUnit but only uses dir, not speed)
  SmInValueType   eTgtStartTanType,    // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pOptTgtStartTan dir
                                       //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                       //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                       //      default:[SM_IV_SAME]
  SmVector3d    * pOptTgtEndTan,       // in : optional End 1stDir tangent dir (NonUnit but only uses dir, not speed)
  SmInValueType   eTgtEndTanType,      // in : oneof SM_IV_SPECIFIED    : set End1stDir = pOptTgtEndTan dir
                                       //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                       //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                       //      default:[SM_IV_SAME]
 SmSurface      * pOptSurfCrvOnSurf,   // in : Optional surface for upon which the target points are define. CrvOnSurf editing. If the CrvOnSurf surf agrees with pOptSurfCrvOnSurf
                                       //      then we edit the underlying UV curve instead of approximating. Default is NULL.
SmOrientType    * pOptCrvOrientation)  // in : orientation for when we have a crv on surf and will adjust UV points.
{
  SM_REF2(pOptSurfCrvOnSurf, pOptCrvOrientation);

  // no work - no requested  changes
  if(   (pTgtStartPt      == NULL)
     && (pTgtEndPt        == NULL)
     && (eTgtStartTanType == SM_IV_SAME || eTgtStartTanType == SM_IV_UNCONSTRAINED)
     && (eTgtEndTanType   == SM_IV_SAME || eTgtEndTanType   == SM_IV_UNCONSTRAINED))
    { return SM_SUCCESS ; }

  // check input - when either EndTanType is SM_IV_SPECIFIED or SM_IV_SAME, ppOptNewCurve must be NonNULL
  SER_MSG(  (   (ppOptNewCurve != NULL)
             || ((eTgtStartTanType == SM_IV_UNCONSTRAINED) && (eTgtEndTanType == SM_IV_UNCONSTRAINED))) 
          ? SM_SUCCESS : SM_ERR_INVALID_INPUT,
          _T("SmLine::DeformToEndPointTargets - when TgtTanType is SM_IV_SAME or SM_IV_SPECIFIED - ppOptNewCurve must be NonNULL to hold method's output")) ;

  // init output
  if(ppOptNewCurve) { *ppOptNewCurve = NULL ; }

  // asked to just move the end points - edit EndPts in-place (changes tan dirs)
  if(    eTgtStartTanType == SM_IV_UNCONSTRAINED
      && eTgtEndTanType   == SM_IV_UNCONSTRAINED)
    {
      // init line parameters
      SmExtent1d sIvl(0, 1.0) ;
      SmVector3d sStartPt = pTgtStartPt ? *pTgtStartPt : (m_vLinePoint + m_dScale * m_vAnalDomain.GetMin() * m_vLineVector) ;
      SmVector3d sEndPt   = pTgtEndPt   ? *pTgtEndPt   : (m_vLinePoint + m_dScale * m_vAnalDomain.GetMax() * m_vLineVector) ;
      SmVector3d sVec     = sEndPt - sStartPt ;
      double     dScale   = sVec.Length() ;

      // watch out for degenerate lines
      if(dScale < SM_EFF_ZERO) { sVec.Set(1,0,0) ; 
                                 dScale = 1.0 ;
                                 sIvl.SetMinMax(0.0, 0.0) ;
                               }
      else                     { sVec /= dScale ;   // Unitize.
                               }

      // deform the line
      m_vLinePoint  = sStartPt ;
      m_vLineVector = sVec ;
      m_vAnalDomain = sIvl ;
      m_dScale      = dScale ;
      // m_bInsideOut = m_bInsideOut ; // leave as is

      // refresh m_pNurb
      MakeNurb() ;

      // all done
      return(SM_SUCCESS) ;

    } // end edit-in-place branch
  else // create and return a new BSpline Curve smoothly deformed to hit EndPoint tgts
    {
      // when needed - make m_pNurb
      if(m_pNurb == NULL)
        { MakeNurb() ; }

      // make new line BSplineCurve
      *ppOptNewCurve = new (GetContext()) SmBSplineCurve(*this) ;

      // pass the call along to the BSplineCurve
      return (*(SmBSplineCurve **)ppOptNewCurve)->SmBSplineCurve::DeformToEndPointTargets(pTgtStartPt,
                                                                                          pTgtEndPt,
                                                                                          NULL,
                                                                                          pOptTgtStartTan, eTgtStartTanType, 
                                                                                          pOptTgtEndTan,   eTgtEndTanType) ;
    } // end create and deform new BSplineCurve  to EndPt tgts branch 

} // end SmLine::DeformToEndPointTargets

/*******************************************************************//**
PURPOSE: Trim this line - does no work if asked to extend. 

NOTES:  
  Trim modifies the analytic interval and m_pNurb
  but it does not modify the StartPoint, LineVec, and dScale values.
***********************************************************************/
SmStatus SmLine::Trim
 (SmExtent1d & crTrimInterval,  // in : Desired trim interval in Nurb Domain - not snapped
  SmBoolean    bNotify,         // in : internal use only - use default, default:[TRUE]
                                //      TRUE  = call Notify after trimming (previous behavior)
                                //      FALSE = skip Notify after trimming
                                //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
  SmBoolean    bSkipDebugCheck) // NotUsed: in : internal use only - use default, default:[FALSE]
                                //      FALSE= in debug mode silently run this->AssertValid()
                                //      TRUE = don't run AssertValid() before returning
{
  SM_REF1(bSkipDebugCheck) ; 
  // no work - current natural interval is a subSet of targetInterval
  SmExtent1d sNatIvl = GetNaturalInterval();
  if (sNatIvl.IsContainedBy(crTrimInterval, SM_EFF_ZERO))
    { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#ifdef SM_USE_CONSTRUCTOR_ASSERT_VALID
  if(!SM_ASSERT_VALID_CONSTRUCTION(this))
#endif
    if(bDebugMe)
      { // place to break 
        Dump() ;
      }
#endif // SM_DEBUG_CODE
  
  // When asked - just return the Nurb interval 
  //   - for lines this is the requested Nurb interval
  if(bNotify == UNSURE)
    { return SM_SUCCESS ; }

  // trim the analytic interval
  double dStepMin, dStepMax ;
  ConvertTFromNURBSToSTEP(crTrimInterval.GetMin(), dStepMin) ;
  ConvertTFromNURBSToSTEP(crTrimInterval.GetMax(), dStepMax) ;
  SM_ASSERT(dStepMax > dStepMin || m_bInsideOut) ;
  if(m_bInsideOut) { m_vAnalDomain.SetMinMax(dStepMax, dStepMin) ; }
  else             { m_vAnalDomain.SetMinMax(dStepMin, dStepMax) ; }

  Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER( this ), NULL );
  // to avoid tolerances - just rebuild the nurbCurve
  MakeNurb() ;
  
  // Set Nurb Parameterization - which calls Notify
  SmBSplineCurve::EditParameterization(crTrimInterval, bNotify) ;

  Notify( SM_NO_POST_EDIT, this, SM_NO_GET_OWNER( this ), NULL );

  //      SER(SmBSplineCurve::Trim(crTrimInterval));

#ifdef SM_DEBUG_CODE 
#ifdef SM_USE_CONSTRUCTOR_ASSERT_VALID
  if(!SM_ASSERT_VALID_CONSTRUCTION(this))
#endif // SM_USE_CONSTRUCTOR_ASSERT_VALID
    { // place to break 
      if(bDebugMe)
        { Dump() ; }
    }
#endif // no SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmLine::Trim

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmLine.

NOTES: Does not include the curve's attribute memory
***********************************************************************/
ULONG SmLine::GetMemoryUsed        // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,   // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)           // in : uses without increment eMarkType value
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

} // end SmLine::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertLine_list[] =
{
  {SM_AT_POINTS, _T("End Points"), _T("sSTEPStartPoint closer than sNurbEndPoint") },
  {SM_AT_POINTS, _T("End Points"), _T("sSTEPEndPoint closer than sNurbStartPoint") },
  {SM_AT_POINTS, _T("End Points"), _T("sSTEPStartPoint closer than sNurbStartPoint") },
  {SM_AT_POINTS, _T("End Points"), _T("sSTEPEndPoint closer than sNurbEndPoint") }
} ;


/*******************************************************************//**
PURPOSE: Make sure STEP and Nurb parameterizations are equivalent.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmLine::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ; 
  // run parent AssertValid
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmBSplineCurve::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  SmBoolean bBounded = IsBounded() ;

  // when line is bounded and there is a Nurb representation
  if(bBounded && m_pNurb)
    {
      // locals
      SmExtent1d sNurbDomain = GetNaturalInterval() ;

      // evaluate STEP endPoints
      SmPoint3d  sSTEPStartPoint, sSTEPEndPoint ;
      EvaluateSTEPPoint(m_vAnalDomain.GetMin(), sSTEPStartPoint) ;
      EvaluateSTEPPoint(m_vAnalDomain.GetMax(), sSTEPEndPoint) ;

      // evaluate Nurb endPoints
      SmPoint3d  sNurbStartPoint, sNurbEndPoint ;
      SmBSplineCurve::EvaluatePoint(sNurbDomain.GetMin(), sNurbStartPoint) ;
      SmBSplineCurve::EvaluatePoint(sNurbDomain.GetMax(), sNurbEndPoint) ;

      // verify that STEP and Nurb representations are equivalent
      double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(sSTEPStartPoint.GetMaxDimension(),
                                                         sSTEPEndPoint.GetMaxDimension())) ;

      if(m_bInsideOut) { 
          bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (sSTEPStartPoint.CloserThan(dScaledZero, sNurbEndPoint)), dScaledZero, sSTEPStartPoint.DistanceBetween(sNurbEndPoint), _T("") ) ;
          bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (sSTEPEndPoint.CloserThan(dScaledZero, sNurbStartPoint)), dScaledZero, sSTEPEndPoint.DistanceBetween(sNurbStartPoint), _T("") ) ;
      }
      else { 
          bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (sSTEPStartPoint.CloserThan(dScaledZero, sNurbStartPoint)), dScaledZero, sSTEPStartPoint.DistanceBetween(sNurbStartPoint), _T("") ) ;
          bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (sSTEPEndPoint.CloserThan(dScaledZero, sNurbEndPoint)), dScaledZero, sSTEPEndPoint.DistanceBetween(sNurbEndPoint), _T("") ) ;

      }
    } // end bounded line check

  // all done
#ifdef SM_DEBUG_CODE
  if(!bRtn)
    { 
      // SM_ASSERT(bRtn);
    }
#endif // SM_DEBUG_CODE

  return(bRtn) ;

} // end SmLine::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmLine::AssertHeal
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
//       return ( SmBSplineCurve::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmLine::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmLine::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmLine to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmLine::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type
  SmFileType      eType    = rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
      
  if (eType == SM_ASCII) 
    {
      rFileOut << m_vLinePoint.x  << " " << m_vLinePoint.y  << " " << m_vLinePoint.z  << " SmLine Line Point \n" ;
      rFileOut << m_vLineVector.x << " " << m_vLineVector.y << " " << m_vLineVector.z << " SmLine Line Vector \n" ;
      rFileOut << m_vAnalDomain.GetMin() << " " << m_vAnalDomain.GetMax() << " SmLine Line Domain \n" ;
      rFileOut << m_dScale        << " SmLine Line Scale \n" ;
      rFileOut << m_bInsideOut    << " SmLine InsideOut Flag \n" ;
    }
  else 
    {
      SER(rDB.WriteDouble(m_vLinePoint.x));
      SER(rDB.WriteDouble(m_vLinePoint.y));
      SER(rDB.WriteDouble(m_vLinePoint.z));

      SER(rDB.WriteDouble(m_vLineVector.x));
      SER(rDB.WriteDouble(m_vLineVector.y));
      SER(rDB.WriteDouble(m_vLineVector.z));

      SER(rDB.WriteDouble(m_vAnalDomain.GetMin()));
      SER(rDB.WriteDouble(m_vAnalDomain.GetMax()));

      SER(rDB.WriteDouble (m_dScale));
      SER(rDB.WriteBoolean(m_bInsideOut));
    }

  // Write the base object
  SmBSplineCurve::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS ;

} // end SmLine::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmLine from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmLine::ReadFromDB
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
         || rpNewCurve->IsKindOf(SmLine_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmLine *pLine =   (rpNewCurve == NULL)
                  ? new (crContext) SmLine()
                  : (SmLine *)rpNewCurve ;

  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmPoint3d  sLinePoint ;
  SmVector3d sLineVector ;
  double     dAnalMin = 0.0, dAnalMax = 0.0;
  double     dScale = 0.0;
  SmBoolean  bInsideOut ;
             
  if (eType == SM_ASCII) 
    {
      rFileIn >> sLinePoint.x  >> sLinePoint.y  >> sLinePoint.z ;   rDB.GoToNextLine() ;
      rFileIn >> sLineVector.x >> sLineVector.y >> sLineVector.z ;  rDB.GoToNextLine() ; 
      rFileIn >> dAnalMin >> dAnalMax ;                             rDB.GoToNextLine() ;
      rFileIn >> dScale ;                                           rDB.GoToNextLine() ;
      rFileIn >> bInsideOut ;                                       rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(sLinePoint.x));
      SER(rDB.ReadDouble(sLinePoint.y));
      SER(rDB.ReadDouble(sLinePoint.z));

      SER(rDB.ReadDouble(sLineVector.x));
      SER(rDB.ReadDouble(sLineVector.y));
      SER(rDB.ReadDouble(sLineVector.z));

      SER(rDB.ReadDouble(dAnalMin));
      SER(rDB.ReadDouble(dAnalMax));

      SER(rDB.ReadDouble (dScale));
      SER(rDB.ReadBoolean(bInsideOut));
    }

  // load the obj
  pLine->m_vLinePoint     = sLinePoint ;
  pLine->m_vLineVector    = sLineVector ;
  pLine->m_vAnalDomain.SetMinMax(dAnalMin, dAnalMax) ; 
  pLine->m_dScale         = dScale ;
  pLine->m_bInsideOut     = bInsideOut ;

  // set the output
  rpNewCurve = pLine ;

  // read the base type
  SmBSplineCurve::ReadFromDB(SmBSplineCurve_TYPE, rDB, lDim, crContext, rpNewCurve, lDBVersionNumber) ; 

  // all done
  return SM_SUCCESS;

} // end SmLine::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmLine::IsKindOf( SM_TYPE t ) const
{
  return ((SmLine_TYPE == t) ? TRUE : SmBSplineCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump preceded by a one line message

NOTES:
***********************************************************************/
void SmLine::Dump
  (const TCHAR * message) 
const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T( "\n%s " ), message );
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmLine::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmLine::Dump
  (ULONG i) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%ld "), i);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmLine::Dump

/*******************************************************************//**
PURPOSE: Dump Line data out for debugging.

NOTES: 
***********************************************************************/
void SmLine::Dump
  (void) 
 const
{
  // eval Nurb and Step endPoints
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmLine::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  SmPoint3d sNurbEndPoint, sNurbStartPoint ;
  SmPoint3d sStepEndPoint, sStepStartPoint ;
  SmExtent1d sNurbIvl = GetNaturalInterval();
  EvaluatePoint(sNurbIvl.GetMin(),sNurbStartPoint) ;
  EvaluatePoint(sNurbIvl.GetMax(),sNurbEndPoint) ;
  EvaluateSTEPPoint(m_vAnalDomain.GetMin(),sStepStartPoint) ;
  EvaluateSTEPPoint(m_vAnalDomain.GetMax(),sStepEndPoint) ;

  // output representations
  smos_sprintf(sBuff,       _T("\nSmLine = 0x%p"), this) ;
  smos_sprintf(sBuffForFile,_T("\nSmLine = %sn"), _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);
  smos_WriteBuffer(_T("\n   AnalDomain  = "));            m_vAnalDomain.Dump();
  smos_WriteBuffer(_T("\n   NurbDomain  = "));            sNurbIvl.Dump();
  smos_WriteBuffer(_T("\n   Line Point  = ")) ;           m_vLinePoint.Dump();
  smos_WriteBuffer(_T("\n   Line Vector = ")) ;           m_vLineVector.Dump();
  smos_sprintf(sBuff,_T("\n   Line Scale  = %16.16lf"), m_dScale); smos_WriteBuffer(sBuff);
  if(m_bInsideOut) { smos_WriteBuffer(_T("\n   bInsideOut = TRUE ")) ;  
                     smos_WriteBuffer(_T("\n     Line STEP Start Point = "));     sStepStartPoint.Dump();
                     smos_WriteBuffer(_T("\n     Line STEP End   Point = "));     sStepEndPoint.Dump();
                   }   
  else             { smos_WriteBuffer(_T("\n   bInsideOut = FALSE ")) ; }
  smos_WriteBuffer(_T("\n     Line NURB Start Point = "));     sNurbStartPoint.Dump();
  smos_WriteBuffer(_T("\n     Line NURB End   Point = "));     sNurbEndPoint.Dump();
  smos_WriteBuffer(_T("\n"));
  if(m_pNurb) { SmBSplineCurve::Dump(); }
  else        { smos_WriteBuffer(_T(" Line Has no underlying NURB Curve ")) ; }

  smos_WriteBuffer(_T(" End SmLine::Dump()\n")) ;

} // end SmLine::Dump

