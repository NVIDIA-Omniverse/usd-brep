// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmParabola.cpp
* PURPOSE: Implementation of SmParabola methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmParabola.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmNurbsCrv.h>
#include <SmAttribute.h>
#include <SmDatabaseIO.h>
#include <SmAssertArray.h>  // AssertValid interface
#include <SmSurfOfExtrusion.h>   // get parameter constraints from owner surface in MakeNurb
#include <SmSurfOfRevolution.h>  // get parameter constraints from owner surface in MakeNurb

/*******************************************************************//**
PURPOSE: Constructor for a Parabola object. A parabola is parametrised
    as: C(u) = O + F * (u * u * X + 2u * Y) where O is the Origin, F is
    the Focal dist, X & Y are axes of Ref. frame. The parameter u is
    range from -Infinity to Infinity.

NOTES: 
***********************************************************************/
SmParabola::SmParabola
  (
  const SmPoint3d   & crCenter,      // Center of Parabola
  const SmVector3d  & crXAxis,       // X axis of Parabola - corresponds to an angle of 0 degrees
  const SmVector3d  & crYAxis,       // Y axis of Parabola - corresponds to an angel of 90 degrees
  const SmExtent1d  & crAnalDomain,  // 
  double             dFocalDist,
  ULONG              lDimension,     // in : default:[3]     
  const SmContext  * cpContext       // in : required for automatic variable with bMakeNurb == TRUE
                                     //      not needed when built with overladed new because value is already set.
                                     //      default:[NULL]
  )
 : SmBSplineCurve(lDimension),
   m_vAnalDomain(crAnalDomain),
   m_dFocalDist(dFocalDist)
{
    SM_ASSERT(dFocalDist > SM_EFF_ZERO);

    // set m_vPosition
    m_vPosition.SetCanonical(crCenter,crXAxis,crYAxis);

    // Trim infinite domains as needed
    double dMin = m_vAnalDomain.GetMin() < -SM_INFINITE_PARAMETER_SQRT ? -SM_INFINITE_PARAMETER_SQRT : m_vAnalDomain.GetMin() ;
    double dMax = m_vAnalDomain.GetMax() >  SM_INFINITE_PARAMETER_SQRT ?  SM_INFINITE_PARAMETER_SQRT : m_vAnalDomain.GetMax() ;
    m_vAnalDomain.SetMinMax(dMin, dMax) ;

    // when passed a context - use it
    if(cpContext) { SM_ASSERT(   GetContext() == NULL 
                              || GetContext() == cpContext) ; 
                    SetContext(cpContext) ; 
                  }
    SM_ASSERT(GetContext() != NULL) ;

    SE(MakeNurb());    
    m_eBSplineCurveForm = SM_CF_PARABOLIC_ARC;

} // end SmParabola::SmParabola construtor

/*******************************************************************//**
PURPOSE: Copy constructor for SmParabola

NOTES: 
***********************************************************************/
SmParabola::SmParabola
  (const SmParabola & crSource)                 // in : target parabola to copy
 : SmBSplineCurve(crSource),
   m_vPosition(crSource.m_vPosition),
   m_vAnalDomain(crSource.m_vAnalDomain),
   m_dFocalDist(crSource.m_dFocalDist)
{
} // end SmParabola::SmParabola copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmParabola

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmParabola::operator==
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
      SmParabola &rOther = (SmParabola &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_vPosition   == rOther.m_vPosition
              && m_vAnalDomain == rOther.m_vAnalDomain
              && SM_IS_ZERO(m_dFocalDist - rOther.m_dFocalDist)) ;                       
    }

  // all done
  return bRtn ;

} // end SmParabola::operator==

/*******************************************************************//**
PURPOSE: Adjust the curve by the given STEP-interval.

NOTES: 
***********************************************************************/
SmStatus SmParabola::AdjustSTEPInterval
  (const SmExtent1d & crNewSTEPInterval)
{
  // low work - New == Old interval
  if(   crNewSTEPInterval.IsContainedBy(m_vAnalDomain) 
     && m_vAnalDomain.IsContainedBy(crNewSTEPInterval))
    { return SM_SUCCESS ; }
  
  // make the assignment
  m_vAnalDomain = crNewSTEPInterval;

  // Trim infinite domains as needed
  double dMin = m_vAnalDomain.GetMin() < -SM_INFINITE_PARAMETER_SQRT ? -SM_INFINITE_PARAMETER_SQRT : m_vAnalDomain.GetMin() ;
  double dMax = m_vAnalDomain.GetMax() >  SM_INFINITE_PARAMETER_SQRT ?  SM_INFINITE_PARAMETER_SQRT : m_vAnalDomain.GetMax() ;
  m_vAnalDomain.SetMinMax(dMin, dMax) ;

  // rebuild the nurb
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  smos_Free(m_pNurb);
  m_pNurb = NULL;
  SER(MakeNurb());
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmParabola::AdjustSTEPInterval

/*******************************************************************//**
PURPOSE: Given a point on the parabola, find its corresponding analytic
    parameter (-Infinity to Infinity)

NOTES: This method requires a point exactly on the curve. Therefore,
    users should be cautious when calling this method.
***********************************************************************/
SmStatus SmParabola::STEPInversion
  (const SmPoint3d      & crPointOnCurve,      // in : Target Point                         
   double               & rdAnalyticParameter, // out: STEPParameter closest to Target Point, range:m_vAnalDomain or positive      
   SmCurveLocationType  * pOptLoc,             // out: Point's classification to curve, NULL to ignore, default:[NULL]
   SmZoneTol3d            dZoneTol3d)          // NotUsed: in : Unused
  const
{
  SM_REF1(dZoneTol3d) ;
  const SmPoint3d  &rCenter = m_vPosition.GetOriginRef();
  const SmVector3d &rYAxis  = m_vPosition.GetYAxisRef();

  // First, find its local coordinates with respect to axes of ellipse
  SmVector3d sVec = crPointOnCurve - rCenter;
  double     dY   = sVec.Dot(rYAxis);
  rdAnalyticParameter = dY/2.0/this->m_dFocalDist;

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

} // end SmParabola::STEPInversion

/*******************************************************************//**
PURPOSE: Virtual Copy method

NOTES: 
***********************************************************************/
SmStatus SmParabola::Copy
  (const SmContext & crContext,    // in : 
   SmCurve        *& rpNewCurve)   // out: 
  const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  SmParabola *pCopy = new (crContext) SmParabola(*this); NER(pCopy) ;
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

} // end SmParabola::Copy

/*******************************************************************//**
PURPOSE: Method to create a canonical Parabola object.

NOTES: 
***********************************************************************/
SmStatus SmParabola::CreateCanonical
  (const SmContext & crContext,
   const SmAxis2Placement & crOrigin,
   double dFocalDist,
   SmParabola *& rpNewParabola)
{
    rpNewParabola = new (crContext) SmParabola(crOrigin.GetOriginRef(),
                                               crOrigin.GetXAxisRef(), 
                                               crOrigin.GetYAxisRef(),
                                               SmExtent1d(-SM_INFINITE_PARAMETER_SQRT,
                                                           SM_INFINITE_PARAMETER_SQRT),
                                               dFocalDist,
                                               3);

    return SM_SUCCESS;

} // end SmParabola::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data.

NOTES: 
***********************************************************************/
SmStatus SmParabola::GetCanonical
  (SmAxis2Placement & rOrigin,
   double & rdFocalDist) 
  const
{
    rOrigin = m_vPosition;
    rdFocalDist = m_dFocalDist;

    return SM_SUCCESS;

} // end SmParabola::GetCanonical

/*******************************************************************//**
PURPOSE: Given a STEP-based parametric value of the curve determine
    the corresponding Euclidian point and optionally derivatives.

NOTES: The step parameter is an angle in degrees.
Example:
    lNumDerivatives = 0 - produces Euclidian point only
                      1 - produces first derivative and point
***********************************************************************/
SmStatus SmParabola::EvaluateSTEP
  (double dSTEPParameter,             // in :
   ULONG lNumDerivatives,             // in :
   SmBoolean bFromLeft,               // NotUsed: in : if P is on interval boundary     //ignored
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d aPointAndDerivatives[], // out: 
   SmZoneTol3d dZoneTol3d)            // NotUsed: in :
  const
{
  SM_REF2(bFromLeft, dZoneTol3d) ;
  if (lNumDerivatives > 3) SER(SM_ERR);
  if (!m_vAnalDomain.ContainsValue(dSTEPParameter)) 
    { dSTEPParameter = m_vAnalDomain.ClampValue(dSTEPParameter) ; }

  // evaluate position
  aPointAndDerivatives[0] =   m_vPosition.GetOriginRef()  
                            + m_dFocalDist * dSTEPParameter*dSTEPParameter * m_vPosition.GetXAxisRef()  
                            + m_dFocalDist *       2.0*dSTEPParameter      * m_vPosition.GetYAxisRef();

  // 1st derivative
  if (lNumDerivatives > 0) 
    {
      aPointAndDerivatives[1] =   2.0 * m_dFocalDist * dSTEPParameter * m_vPosition.GetXAxisRef() 
                                + 2.0 * m_dFocalDist *                  m_vPosition.GetYAxisRef();
    }

  // 2nd derivative
  if (lNumDerivatives > 1) 
    {
      aPointAndDerivatives[2] =   2.0 * m_dFocalDist * m_vPosition.GetXAxisRef() ;
    }

  // 3rd derivative
  if (lNumDerivatives > 2) 
    {
      aPointAndDerivatives[3].x = 0.0 ;
      aPointAndDerivatives[3].y = 0.0 ;
      aPointAndDerivatives[3].z = 0.0 ;
    }

  return SM_SUCCESS;

} // end SmParabola::EvaluateSTEP


/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the parabola.  This method can be used to find the 
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
SmStatus SmParabola::GlobalPointSolveSTEP
  (const SmExtent1d      & crInterval,             // in : step domain
   SmSolverOperationType   eSolverOperation,       // in :
   const SmPoint3d       & crTestPoint,            // in :
   double                  dDistanceTolerance,     // in :
   const double          * cpdOptTargetDistance,   // in :
   const SmVector3d      * cpOptVectors,           // NotUsed: in :
   SmSolutionRequestedType eSolutionRequested,     // in :
   SmSolutionArray       & rSolutions)             // out: step domain parameters
{
  SM_REF1(cpOptVectors) ;
    SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE
              || eSolverOperation == SM_SO_MAXIMIZE
              || eSolverOperation == SM_SO_NORMALIZE
              || eSolverOperation == SM_SO_INTERSECT);

    SER(AdjustSTEPInterval(crInterval));

    const SmPoint3d &rCenter = m_vPosition.GetOriginRef();
    SmVector3d       sZAxis  = m_vPosition.GetZAxis();

    // Project the point onto the plane which Ellipse is on
    SmPoint3d sProjPoint;
    SER(smgu_PointProjectToPlane(crTestPoint,rCenter,sZAxis,sProjPoint));

    SmExtent1d sIvl = GetNaturalInterval();
    SER(GlobalPointSolve(sIvl,eSolverOperation,crTestPoint,
                         dDistanceTolerance,cpdOptTargetDistance,
                         NULL,eSolutionRequested,rSolutions));

    for (ULONG i=0; i<rSolutions.GetSize(); i++) 
      {
        SmSolution & rSol = rSolutions[i];
        SmPoint3d sPointOnCurve;
        SER(EvaluatePoint(rSol.m_vStart[0],sPointOnCurve));
        double dAngleParam;
        SER(STEPInversion(sPointOnCurve,dAngleParam));
        rSol.m_vStart[0] = dAngleParam;
      }
    
    return SM_SUCCESS;

} // end SmParabola::GlobalPointSolveSTEP

//      /*******************************************************************//**
//      PURPOSE:  
//      
//      NOTES: 
//      ***********************************************************************/
//      virtual SmStatus SmParabola::ConvertTFromSTEPToNURBS
//        (double   dSTEPParam,
//         double & rdNURBSParam) 
//        const
//      {
//      
//      
//      } // end SmParabola::ConvertTFromSTEPToNURBS
//      
//      /*******************************************************************//**
//      PURPOSE:  
//      
//      NOTES: 
//      ***********************************************************************/
//      virtual SmStatus SmParabola::ConvertTFromNURBSToSTEP
//        (double   dNURBSParam,
//         double & rdSTEPParam) 
//        const
//      {
//      
//      } // end SmParabola::ConvertTFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Determine if a parabola is bounded
NOTES: 
***********************************************************************/
SmBoolean SmParabola::IsBounded
  () 
 const
{
  return( m_vAnalDomain.IsBounded() ) ;

} // end SmParabola::IsBounded

/*******************************************************************//**
PURPOSE: Make the Nurb representation for a parabola
NOTES: 
***********************************************************************/
SmStatus SmParabola::MakeNurb
  ()
{
  // remove existing parabola Nurb Representation
  if (m_pNurb) { smos_Free(m_pNurb); m_pNurb = NULL; }

  // get start and end point and tangent values
  double dT1 = m_vAnalDomain.GetMin();
  double dT2 = m_vAnalDomain.GetMax();
  SmVector3d sStartPV[2];
  SmVector3d sEndPV[2];
  SER(EvaluateSTEP(dT1,1,TRUE,sStartPV));
  SER(EvaluateSTEP(dT2,1,TRUE,sEndPV));

  // place control point at endPointTangent Line intersections
  double dX1, dX2 ;
  SER(smgu_LineLineClosestPoint(sStartPV[0],
                                sStartPV[1],
                                sEndPV[0],  
                                sEndPV[1],
                                dX1,dX2));
  SmPoint3d sMidCtrlPnt = sStartPV[0] + dX1 * sStartPV[1];

  // Create a canonical 3 ControlPoint Bspline
  SmPoint3d sData[3];
  SmTArray<SmPoint3d> sCntrlPoly(3,sData);
  sCntrlPoly.Add(sStartPV[0]);
  sCntrlPoly.Add(sMidCtrlPnt);
  sCntrlPoly.Add(sEndPV[0]);
  double adKData[2];
  SmTArray<double> sKnots(2,adKData,2);
  sKnots[0] = dT1; sKnots[1] = dT2;
  ULONG alKMData[2];
  SmTArray<ULONG> sKnotMult(2,alKMData,2);
  sKnotMult[0] = 3;
  sKnotMult[1] = 3;

  // build temporary SmBSplineCurve with appropriate Nurb objecct
  SmBSplineCurve *pTmpBSC = NULL;
  const SmContext * cpContext = GetContext();
  SER(SmBSplineCurve::CreateCanonical(*cpContext,3,2,sCntrlPoly,SM_CF_UNSPECIFIED,
      sKnotMult,sKnots,SM_KT_UNSPECIFIED,NULL,NULL,pTmpBSC));
  SmObjDelete sClean(pTmpBSC);

  // Place a copy of pTmpBSC->m_pNurb into this Curve's m_pNurb
  gw_CURVE *pTmp = pTmpBSC->GetOrCreateGwNurbPointer();
  this->SetFromGwNurb(0, pTmp);

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

  // all done - on exit - delete temporary object
  return SM_SUCCESS;

} // end SmParabola::MakeNurb

/*******************************************************************//**
PURPOSE: Reverse the parameterization of a curve and update an 
    interval on the curve.

NOTES: 
***********************************************************************/
SmStatus SmParabola::ReverseParameterization
  (const SmExtent1d & crOldInterval,   // in : current curve interval       
   SmExtent1d & rNewInterval)          // out: curve interval after reversal
{
    if (m_pNurb) {
        smos_Free(m_pNurb);
        m_pNurb = NULL;
    }

    Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER( this ), NULL );

    rNewInterval.SetMinMax(-crOldInterval.GetMax(),-crOldInterval.GetMin());
    m_vAnalDomain.SetMinMax(-m_vAnalDomain.GetMax(),-m_vAnalDomain.GetMin());

    SmVector3d sNegYAxis = - m_vPosition.GetYAxisRef();
    m_vPosition.SetCanonical(m_vPosition.GetOriginRef(),m_vPosition.GetXAxisRef(),sNegYAxis);
    
    return SM_SUCCESS;

} // end SmParabola::ReverseParameterization

/*******************************************************************//**
PURPOSE: Scale and transform a Parabola curve.

NOTES: Scaling is not allowed on analytical curves.
***********************************************************************/
SmStatus SmParabola::Transform
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

    if (cpOptScale && !(cpOptScale->x == 1.0 &&
        cpOptScale->y == 1.0 && cpOptScale->z == 1.0)) 
      {
        if (!SM_ARE_SAME(cpOptScale->x,cpOptScale->y)  ||
            !SM_ARE_SAME(cpOptScale->x,cpOptScale->z) ) 
          {
            ERR_MSG(_T("Unable to scale analytical curves non-uniformly\n"));
            SER(SM_ERR);
          }
      }

    double dScale = 1.0;
    if (cpOptScale) 
      {
        dScale = cpOptScale->x; // Uniform scaling
      }

    SmAxis2Placement sTmpA2P;
    SmAxis2Placement sPlace;
    SER(sPlace.SetCanonical(m_vPosition.GetOriginRef() * dScale, 
                           m_vPosition.GetXAxisRef(), m_vPosition.GetYAxisRef()));
    sPlace.TransformAxis2Placement(crRotateNMove,sTmpA2P);
    m_vPosition = sTmpA2P;

    this->m_dFocalDist *= dScale;

    if (m_pNurb) 
      {
        SER(SmBSplineCurve::Transform(crRotateNMove,cpOptScale));
      }

    return SM_SUCCESS;

} // end SmParabola::Transform

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmParabola.

NOTES: Does not include the curve's attribute memory
***********************************************************************/
ULONG SmParabola::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
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
  rlMemoryAllocated =   sizeof(*this) 
                      + sm_ComputeNurbCurveSize(m_pNurb) ;
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

} // end SmParabola::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Write SmParabola to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmParabola::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type
  SmFileType      eType    = rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  const SmPoint3d  &rOrig = m_vPosition.GetOriginRef() ;
  const SmVector3d &rX    = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rY    = m_vPosition.GetYAxisRef() ;
      
  if (eType == SM_ASCII) 
    {
      rFileOut << rOrig.x << " " << rOrig.y << " " << rOrig.z             << " SmParabola Axis Origin \n" ;
      rFileOut << rX.x    << " " << rX.y    << " " << rX.z                << " SmParabola X Axis\n" ;
      rFileOut << rY.x    << " " << rY.y    << " " << rY.z                << " SmParabola Y Axis\n" ;
      rFileOut << m_vAnalDomain.GetMin() << " " << m_vAnalDomain.GetMax() << " SmParabola anular arc domain in degrees \n" ;
      rFileOut << m_dFocalDist                                            << " SmParabola Focal Distance          \n" ;
    }
  else 
    {
      SER(rDB.WriteDouble(rOrig.x));
      SER(rDB.WriteDouble(rOrig.y));
      SER(rDB.WriteDouble(rOrig.z));

      SER(rDB.WriteDouble(rX.x));
      SER(rDB.WriteDouble(rX.y));
      SER(rDB.WriteDouble(rX.z));

      SER(rDB.WriteDouble(rY.x));
      SER(rDB.WriteDouble(rY.y));
      SER(rDB.WriteDouble(rY.z));

      SER(rDB.WriteDouble(m_vAnalDomain.GetMin()));
      SER(rDB.WriteDouble(m_vAnalDomain.GetMax()));

      SER(rDB.WriteDouble(m_dFocalDist));
    }

  // Write the base object
  SmBSplineCurve::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS ;

} // end SmParabola::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmParabola from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmParabola::ReadFromDB
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
         || rpNewCurve->IsKindOf(SmParabola_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmParabola *pParabola =   (rpNewCurve == NULL)
                            ? new (crContext) SmParabola()
                            : (SmParabola *)rpNewCurve ;

  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmPoint3d sOrig ;
  SmVector3d sX, sY ;
  double dAnalMin = 0.0, dAnalMax = 0.0;
  double dFocalDist = 0.0;
             
  if (eType == SM_ASCII) 
    {
      rFileIn >> sOrig.x >> sOrig.y >> sOrig.z ; rDB.GoToNextLine() ;
      rFileIn >> sX.x    >> sX.y    >> sX.z ;    rDB.GoToNextLine() ;
      rFileIn >> sY.x    >> sY.y    >> sY.z ;    rDB.GoToNextLine() ;
      rFileIn >> dAnalMin >> dAnalMax ;          rDB.GoToNextLine() ;
      rFileIn >> dFocalDist ;                    rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(sOrig.x));
      SER(rDB.ReadDouble(sOrig.y));
      SER(rDB.ReadDouble(sOrig.z));

      SER(rDB.ReadDouble(sX.x));
      SER(rDB.ReadDouble(sX.y));
      SER(rDB.ReadDouble(sX.z));

      SER(rDB.ReadDouble(sY.x));
      SER(rDB.ReadDouble(sY.y));
      SER(rDB.ReadDouble(sY.z));

      SER(rDB.ReadDouble(dAnalMin));
      SER(rDB.ReadDouble(dAnalMax));

      SER(rDB.ReadDouble(dFocalDist));
    }

  // load the obj
  pParabola->m_vPosition.SetCanonical(sOrig, sX, sY) ;   
  pParabola->m_vAnalDomain.SetMinMax(dAnalMin, dAnalMax) ; 
  pParabola->m_dFocalDist = dFocalDist ;

  // set the output
  rpNewCurve = pParabola ;

  // read the base type
  SmBSplineCurve::ReadFromDB(SmBSplineCurve_TYPE, rDB, lDim, crContext, rpNewCurve, lDBVersionNumber) ; 

  // all done
  return SM_SUCCESS;

} // end SmParabola::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmParabola::IsKindOf( SM_TYPE t ) const
{
  return ((SmParabola_TYPE == t) ? TRUE : SmBSplineCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Parabola data out for debugging.

NOTES: 
***********************************************************************/
void SmParabola::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmParabola::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_sprintf(sBuff,_T("SmParabola - Focal Distance = %16.16lf"),m_dFocalDist);
  smos_WriteBuffer(sBuff);
  m_vAnalDomain.Dump();
  m_vPosition.Dump();

  SmBSplineCurve::Dump();

  smos_WriteBuffer(_T(" End SmParabola::Dump()\n")) ;

} // end SmParabola::Dump
