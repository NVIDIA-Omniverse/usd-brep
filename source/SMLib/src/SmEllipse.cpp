// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmEllipse.cpp
* PURPOSE: Implementation of SmEllipse methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmEllipse.h>
#include <SmLine.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <nurbs.h>
#include <SmNurbsCrv.h>
#include <SmCircle.h>
#include <SmPolynomial.h>
#include <SmAttribute.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#include <SmSurfOfExtrusion.h>   // get parameter constraints from owner surface in MakeNurb
#include <SmSurfOfRevolution.h>  // get parameter constraints from owner surface in MakeNurb

#ifdef SM_DEBUG_CODE
#include <SmBrep.h>
#include <SmEdge.h>
#endif // SM_DEBUG_CODE

// GWC: Not yet ready to use  SM_TOLERANT_XSECT code
#ifdef SM_TOLERANT_XSECT
#undef SM_TOLERANT_XSECT
#endif

/*******************************************************************//**
PURPOSE: Copy constructor for SmPolarConverstion

NOTES: 
***********************************************************************/
SmPolarConversion::SmPolarConversion
  (const SmContext         & crContext, // in : context for new object construction
   const SmPolarConversion & crSource)  // in : object to copy
 : m_bPolarConversionPossible(crSource.m_bPolarConversionPossible)
{
  // init internal arrays
  m_vNaturalKnots.ReSet();
  m_vAngles.ReSet();
  m_vExactConversion.ReSet();

  // copy internal arrays
  m_vNaturalKnots.Append(crSource.m_vNaturalKnots);
  m_vAngles.Append(crSource.m_vAngles);
  m_vExactConversion.Append(crSource.m_vExactConversion);

  // copy the source curve pointer
  if(crSource.m_cpCurve) { SmCurve *pCrvCopy = NULL ;
                           crSource.m_cpCurve->Copy(crContext, pCrvCopy) ;
                           m_cpCurve     = (SmEllipse *)pCrvCopy ;
                           m_bIsBorrowed = FALSE ;
                         }
  else                   { m_cpCurve     = NULL ;
                           m_bIsBorrowed = TRUE ; 
                         }

} // end SmPolarConversion::SmPolarConversion constructor

/*******************************************************************//**
PURPOSE: SmPolarConversion destructor

NOTES:
***********************************************************************/
SmPolarConversion::~SmPolarConversion() 
{ 
  // free nested BSpline Curve pointer
  if(   m_bIsBorrowed == FALSE
     && m_cpCurve) { delete m_cpCurve ; m_cpCurve = NULL ; }

} // end SmPolarConversion::~SmPolarConversion destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmPolarConversion

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmPolarConversion::operator==
  (const SmPolarConversion& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // locals
  SmBoolean bRtn = TRUE ;

  if(bRtn)
    {
      SmBoolean bHasCurve = m_cpCurve != NULL && m_bIsBorrowed == FALSE ;

      // check equivalence of these objects
      bRtn = (   m_bPolarConversionPossible   == crOther.m_bPolarConversionPossible
              && m_vNaturalKnots.GetSize()    == crOther.m_vNaturalKnots.GetSize()
              && m_vAngles.GetSize()          == crOther.m_vAngles.GetSize()  
              && m_vExactConversion.GetSize() == crOther.m_vExactConversion.GetSize()  
              && (   bHasCurve              == FALSE
                  || ( m_cpCurve            == crOther.m_cpCurve)
                  || ( m_cpCurve == NULL && crOther.m_cpCurve == NULL)
                  || (    m_cpCurve != NULL && crOther.m_cpCurve != NULL
                      && *m_cpCurve == *crOther.m_cpCurve))) ;
      
      // check real array values
      if(bRtn)
        {
          ULONG ii ;
          for(ii=0;ii<m_vNaturalKnots.GetSize()&&bRtn;ii++)
            {
              bRtn &= SM_IS_ZERO(m_vNaturalKnots[ii] - crOther.m_vNaturalKnots[ii]) ;
            }

          for(ii=0;ii<m_vAngles.GetSize()&&bRtn;ii++)
            {
              bRtn &= SM_IS_ZERO(m_vAngles[ii] - crOther.m_vAngles[ii]) ;
            }
        }                       
    }

  // all done
  return bRtn ;

} // end SmPolarConversion::operator==

/*******************************************************************//**
PURPOSE: Copy the polar conversion from an existing polar converter.

NOTES: 
***********************************************************************/
void SmPolarConversion::CopyFrom
  (const SmPolarConversion & crSource)
{
  // copy internal values
  m_bPolarConversionPossible = crSource.m_bPolarConversionPossible;

  // init internal arrays
  m_vNaturalKnots.ReSet();
  m_vAngles.ReSet();
  m_vExactConversion.ReSet();

  // append internal arrays
  m_vNaturalKnots.Append(crSource.m_vNaturalKnots);
  m_vAngles.Append(crSource.m_vAngles);
  m_vExactConversion.Append(crSource.m_vExactConversion);

  // copy the source curve pointer
  if(crSource.m_cpCurve) { SmCurve *pCrvCopy = NULL ;
                           crSource.m_cpCurve->Copy(*crSource.m_cpCurve->GetContext(), pCrvCopy) ;
                           m_cpCurve      = (SmEllipse *)pCrvCopy ;
                           m_bIsBorrowed = FALSE ;
                         }
  else                   { m_cpCurve      = NULL ;
                           m_bIsBorrowed = TRUE ; 
                         }

} // end SmPolarConversion::CopyFrom

/*******************************************************************//**
PURPOSE: Set up the polar conversion tables using this degree 2 
    circular NURBS.  This method assumes that the zero angle corresponds
    to the start of the curve.

NOTES: The input NURBS must be the typical one used to make
    a circular NURBS with double interior knots and equal length control
    polygon edges between the knots.
***********************************************************************/
SmStatus SmPolarConversion::SetUpPolarConversion
  (SmEllipse              * cpQuadCircle,        // in : target circular curve (rational quadratic)
   SmBoolean                bIsBorrowed,         // in : TRUE = Pointer to QuadCircleCurve is stored but not owned for deletion
                                                 //      FALSE= Pointer to QuadCircleCurve is owned and deleted with this object
   const SmExtent1d       & crIntervalDeg,       // NotUsed: in : analytic interval for circle [-360, 360] MaxLength=360
   const SmAxis2Placement & crPlacement)         // NotUsed: in : no longer used.
{
  SM_REF2(crIntervalDeg, crPlacement) ;
  // check state
  SM_ASSERT(cpQuadCircle->IsKindOf(SmEllipse_TYPE)) ;

  // init state
  m_bPolarConversionPossible = FALSE;

  // clean up any existing curve memory
  //   ( For debugging purposes only - this function may be called
  //     with cpQuadCircle == m_cpCurve)
  if(   m_cpCurve
     && m_cpCurve != cpQuadCircle
     && m_bIsBorrowed == FALSE) 
    { 
      delete m_cpCurve ; m_cpCurve = NULL ; 
    }

  // store the curve
  m_cpCurve     = cpQuadCircle ;
  m_bIsBorrowed = bIsBorrowed ;

  // no work - not a degree 2 curve
  if (cpQuadCircle->GetDegree() != 2) 
    { return SM_ERR; }
  
  // get curve knots
  SmTArray<ULONG> sMultiplicities;
  cpQuadCircle->GetKnots( m_vNaturalKnots, &sMultiplicities );

  ULONG j, lCnt = m_vNaturalKnots.GetSize();

  // no work - start/end knots are not full multiplicities
  if (   sMultiplicities[0]        != 3
      || sMultiplicities.GetLast() != 3) 
    { SER(SM_ERR); }

  // no work - internal knots not double - no conversion possible
  for (j=1; j+1<lCnt; j++)  // note: can't say lCnt-1
    {
      if (sMultiplicities[j] != 2) 
        { return SM_SUCCESS; }
    }

  // locals
  double dRadius; 
  double dAngleDeg ;
  double dSpanStartAngle, dSpanEndAngle=0.0;
  SmPoint3d  sPnt1, sPnt2;
  SmAxis2Placement sPlacement;
  SmBoolean bInsideOut = cpQuadCircle->GetInsideOut() ;

  ULONG ii;
  lCnt = m_vNaturalKnots.GetSize();

  // We have to make sure that the interval in cpQuadCircle
  // contains all of the angles that the Nurbs curve requires.
  // (This will be an issue if the Nurbs curve has increased
  // in size, such as after JoinWith().)

  // If it's already a full circle, then there's no problem here.

  SmExtent1d sAnalIvl = cpQuadCircle->GetSTEPInterval();

  if ( sAnalIvl.GetLength() < 360.0 -SM_EFF_ZERO )
    {
      ULONG lIdx = ( bInsideOut ) ? lCnt-1 : 0;
      double dLowestAngleParam = m_vNaturalKnots[ lIdx ];
      lIdx = ( bInsideOut ) ? 0: lCnt-1;
      double dHighestAngleParam = m_vNaturalKnots[ lIdx ];

      double dLowestDeg, dHighestDeg;

      // Temporarily set the curve's domain to a full circle,
      // so that STEPInversion() won't bump into the limits.
      // It doesn't matter which period we set it to,
      // because we will sort it out afterwards.
      SmExtent1d fullCircle( 0, 360 );
      cpQuadCircle->SetSTEPInterval( fullCircle );

      // Get Polar angle about ZAxis from XAxis in degrees.
      SER( cpQuadCircle->EvaluatePoint( dLowestAngleParam, sPnt1 ));
      cpQuadCircle->STEPInversion( sPnt1, dLowestDeg );

      SER( cpQuadCircle->EvaluatePoint( dHighestAngleParam, sPnt2 ));
      cpQuadCircle->STEPInversion( sPnt2, dHighestDeg );

      // STEPInversion will snap dHighestDeg to Min param according to tolerance
      // Only account for noise here. [B630]
      //if ( dHighestDeg <= dLowestDeg + SM_EFF_ZERO_DEG )
      if ( dHighestDeg <= dLowestDeg + SM_EFF_ZERO)
        { dHighestDeg += 360; }  // closed curve.

      // Move to period of greatest overlap with input sAnalIvl.
      // Do that by getting their midpoints within 180 of each other.
      double dInputMid = sAnalIvl.GetMid();
      double dThisMid  = ( dLowestDeg + dHighestDeg ) / 2.0;

      while ( dThisMid > dInputMid + 180.0 + SM_EFF_ZERO )
      {
          dLowestDeg  -= 360.0;
          dHighestDeg -= 360.0;
          dThisMid    -= 360.0;
      }
      while ( dThisMid < dInputMid - 180.0 - SM_EFF_ZERO )
      {
          dLowestDeg  += 360.0;
          dHighestDeg += 360.0;
          dThisMid    += 360.0;
      }

      // Now put this back into the curve, so that STEPInversion()
      // won't have problems.  

      // watch out for tolerance creep: 
      // Computing the exact angle from an inverse mapping of an evaluation may be off by a few bits.
      //   ArcTan(xAxis, Eval(endParam)-Center) 
      //     may vary by a small bit when the input was originally OK 
      double dLowChangeDeg = dLowestDeg - sAnalIvl.GetMin() ;
      if(   dLowChangeDeg <  SM_EFF_ZERO * 1000.0
         && dLowChangeDeg > -SM_EFF_ZERO * 1000.0)  // use 1000 since that's just 360 rounded up a bit
      dLowestDeg = sAnalIvl.GetMin() ;

      double dHighChangeDeg = dHighestDeg - sAnalIvl.GetMax() ;
      if(   dHighChangeDeg <  SM_EFF_ZERO * 1000.00
         && dHighChangeDeg > -SM_EFF_ZERO * 1000.0)
      dHighestDeg = sAnalIvl.GetMax() ;

      sAnalIvl.SetMinMax( dLowestDeg, dHighestDeg );
      cpQuadCircle->SetSTEPInterval( sAnalIvl );

  } // end if original interval not 360.


  // Tolerance is around 6 decimal places of accuracy.
  // evaluate start Point and near-mid Point to pick tolerance
  SmExtent1d sNurbIvl    = cpQuadCircle->GetNaturalInterval();
  SER(cpQuadCircle->EvaluatePoint(sNurbIvl.GetMin(),sPnt1));
  SER(cpQuadCircle->EvaluatePoint(sNurbIvl.Evaluate(0.4567),sPnt2));
  double     dScaledZero = SM_EFF_ZERO_SQRT * (1.0 + sPnt1.GetMaxDimension() + sPnt2.GetMaxDimension() );

  // get ellipse position, radius, and STEPInterval bounds
  if(cpQuadCircle->IsKindOf(SmEllipse_TYPE))
    {
      sPlacement = cpQuadCircle->GetPosition() ;
      dRadius    = cpQuadCircle->GetXRadius() ;

      // // no work - sPlacement and crPlacement should be the same when  input curve is a circle
      // if(   sPlacement.GetXAxisRef().DistanceBetween(crPlacement.GetXAxisRef()) > SM_EFF_ZERO_SQRT
      //    || sPlacement.GetYAxisRef().DistanceBetween(crPlacement.GetYAxisRef()) > SM_EFF_ZERO_SQRT) 
      //   { return SM_SUCCESS; }
    }
  else // check for arc, get radius and start/end angles measured CCW from CenterPoint to StartPoint vector
    {
      double dStartAngle, dEndAngle ;
      if (!cpQuadCircle->IsArc(20, dScaledZero, sPlacement, dRadius, dStartAngle, dEndAngle))
        { // no work - curve is not an arc
          return SM_SUCCESS; 
        }

      // no work - curve should lie in plane perp to crPlacement.ZAxis
      // SmVector3d sPlaceZAxis = sPlacement.GetZAxis() ;
      // SmVector3d sGivenZAxis = crPlacement.GetZAxis() ;
      // double     dZAxisDist  = sPlaceZAxis.DistanceBetween(sGivenZAxis) ;
      // if (dZAxisDist > SM_EFF_ZERO_SQRT) 
      //   { return SM_SUCCESS; }
    }

  //      The underlying nurb can be larger than the given analytic interval
  //        SM_ASSERT(dEndAngle-dStartAngle <= crIntervalDeg.GetLength() + 360.0 * SM_EFF_ZERO) ;

  // get curve control points and weights
  SmTArray<double>    sWeights;
  SmTArray<SmPoint3d> sControlPolygon;
  SER(cpQuadCircle->GetControlPolygon( sControlPolygon, sWeights ));

  // Clear cached array values
  m_vAngles.SetDataSize( m_vNaturalKnots.GetSize() );
  m_vAngles.ReSet();
  m_vExactConversion.ReSet();

  double dAngleTol = SM_EFF_ZERO_DEG ;

  // for every span boundary
  for (ii=0; ii<lCnt; ii++) 
    {
      // get Polar angle about ZAxis from XAxis in degrees watching out for periodicity
      SER(cpQuadCircle->EvaluatePoint(m_vNaturalKnots[ii], sPnt2));
      cpQuadCircle->STEPInversion(sPnt2, dAngleDeg) ;

      // warning - angles on closed seams are always returned as the min boundary value - 
      //           but sometimes they need to be the max boundary

      // when knots match up with Interval endPoints - use the sAnalIvl end value when appropriate
      SmBoolean bEndPt = FALSE ;
      if(ii==0 && (   smos_Fabs(dAngleDeg - sAnalIvl.GetMin()) < dAngleTol
                   || smos_Fabs(dAngleDeg - sAnalIvl.GetMax()) < dAngleTol))
        { 
          // bInsideOut == 1 means that the Nurb and Step parameterizations are switched,
          //                 so make sure proper endValue is used for endpoints 
          dAngleDeg = bInsideOut ? sAnalIvl.GetMax()
                                 : sAnalIvl.GetMin() ;
          bEndPt = TRUE ; 
        }
      if(ii==lCnt-1 && (   smos_Fabs(dAngleDeg - sAnalIvl.GetMax()) < dAngleTol
                        || smos_Fabs(dAngleDeg - sAnalIvl.GetMin()) < dAngleTol))
        {
          dAngleDeg = bInsideOut ? sAnalIvl.GetMin()
                                 : sAnalIvl.GetMax() ;
          bEndPt = TRUE ;                                  
        }

      if(bEndPt == FALSE)
        {
          // snap values to -360, 0, and 360
          if     (smos_Fabs(dAngleDeg + 360.0) < dAngleTol) dAngleDeg = -360.0 ;
          else if(smos_Fabs(dAngleDeg        ) < dAngleTol) dAngleDeg =    0.0 ;
          else if(smos_Fabs(dAngleDeg - 360.0) < dAngleTol) dAngleDeg =  360.0 ;
        }

      // save the angle value in the angle array
      dSpanStartAngle = dSpanEndAngle ;
      dSpanEndAngle   = dAngleDeg ;
      m_vAngles.Add(dAngleDeg);

      // skip interval classification for 1st boundary
      if(ii==0) {continue;}

      // set the exactConversion status for each interval (not Boundary)
      // to TRUE when weights are constant - FALSE otherwise
      double dWeighta = sWeights[ii*2-2] ;
      double dWeightb = sWeights[ii*2-1] ;
      double dWeightc = sWeights[ii*2] ;
      double dCosHalfAng = smos_Cosine( (dSpanEndAngle-dSpanStartAngle)/2.0 * SM_PI/180.0 ) ;
      if(   SM_ARE_SAME(dWeighta,dWeightc)
         && SM_ARE_SAME(dWeightb/dWeightc, dCosHalfAng) )
            { m_vExactConversion.Add(TRUE) ; }
      else  
            { m_vExactConversion.Add(FALSE); }
    } // end iter every curve span

  // quit - something went wrong and there is not an angle and an ExactConversion entry for every natural knot 
  if(   m_vAngles.GetSize()            != m_vNaturalKnots.GetSize()
     || m_vExactConversion.GetSize()+1 != m_vAngles.GetSize()) 
    { SER(SM_ERR); }

  // Reset the curve's analytic interval to what we've calculated.
  // They will probably be the same, but might have been cleaned up near 0 and 360.
  sAnalIvl.Init();
  sAnalIvl.AddValue( m_vAngles[0] );
  sAnalIvl.AddValue( m_vAngles[lCnt-1] );
  cpQuadCircle->SetSTEPInterval( sAnalIvl );



  // all done - conversion tables are set up
  //  - mark the Possible bit as TRUE and return
  m_bPolarConversionPossible = TRUE;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // Check monotonic.
      for ( ii = 1; ii < lCnt; ii++ )
        {
          if ( ! bInsideOut && m_vAngles[ii] <= m_vAngles[ii-1] )
            SM_ASSERT( FALSE );
          if (   bInsideOut && m_vAngles[ii] >= m_vAngles[ii-1] )
            SM_ASSERT( FALSE );
        }

      SM_DUMP_AND_ASSERT_VALID(this) ;
      Dump() ;
      m_cpCurve->Dump() ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmPolarConversion::SetUpPolarConversion

/*******************************************************************//**
PURPOSE: Convert from a Polar parameter to a NURBS parameter.

NOTES: 
***********************************************************************/
SmStatus SmPolarConversion::ConvertToNURBSParameter
  (double          dPolarParameterDeg,  // in : angle in degrees to convert
   double        & rdNurbsParameter,    // out: curve parameter value
   SmBoolean     & rbExactConversion,   // out: TRUE  = conversion was exact
                                        //      FALSE = conversion was approximate to about SM_ZONE_TOL_3D/10.0
   SmZoneTol3d     dZoneTol3d)          // in : Tolerance for snapping near max param to min. Default=0.0
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
      m_cpCurve->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // init output
  rbExactConversion = FALSE ;

  // locals
  SmExtent1d sSTEPIvl    = m_cpCurve->GetSTEPInterval() ;
  SmBoolean  bInsideOut  = m_cpCurve->GetInsideOut() ;
  double     dAngleTol   = SM_EFF_ZERO_DEG ;  
  double     dNURBSGuess ; 
  
  // when bInsideOut the NurbDomain is opposite to the StepDomain 
  if(bInsideOut) { dNURBSGuess =  m_vNaturalKnots.GetLast()
                                + (dPolarParameterDeg - sSTEPIvl.GetMin())
                                / (sSTEPIvl.GetMax() - sSTEPIvl.GetMin()) 
                                * (m_vNaturalKnots[0] - m_vNaturalKnots.GetLast()) ;
                 }
  else           { dNURBSGuess =  m_vNaturalKnots[0]
                                + (dPolarParameterDeg - sSTEPIvl.GetMin())
                                / (sSTEPIvl.GetMax() - sSTEPIvl.GetMin()) 
                                * (m_vNaturalKnots.GetLast() - m_vNaturalKnots[0]) ;
                 }

  // when exact conversions tables are initialized - get a translation or a better guess
  if(m_bPolarConversionPossible)
    {
      SmBoolean  bInside ;
      SmExtent1d sAngleRange( m_vAngles[0] );
      sAngleRange.AddValue( m_vAngles.GetLast() );
      bInside = sAngleRange.ContainsValue( dPolarParameterDeg, dAngleTol );
      if (!bInside)
      {
          // SER_MSG(SM_ERR, _T("PolarConverter Angle Domain does not contain given angle"))
          // Note, don't clamp 'periodic'. [B90]
          dPolarParameterDeg = sAngleRange.ClampValue( dPolarParameterDeg );
      }

      // check first angle boundary
      if(SM_ARE_SAME_TO_TOL(dPolarParameterDeg, m_vAngles[0], dAngleTol)) 
        {
          rbExactConversion = TRUE ;
          rdNurbsParameter  = m_vNaturalKnots[0] ;
          dNURBSGuess       = rdNurbsParameter ;
        }
      else
        { // for every arc interval
          for (ULONG ii=1; ii<m_vAngles.GetSize(); ii++) 
            {
              // check interval other bound
              if(SM_ARE_SAME_TO_TOL(dPolarParameterDeg, m_vAngles[ii], dAngleTol)) 
                {
                  rbExactConversion = TRUE ;
                  rdNurbsParameter  = m_vNaturalKnots[ii] ;
                  dNURBSGuess       = rdNurbsParameter ;
                  break ;
                }

              // check interval interior
              if(   (!bInsideOut && dPolarParameterDeg <= m_vAngles[ii])
                 || ( bInsideOut && dPolarParameterDeg >= m_vAngles[ii])) 
                {
                  double dStartAngDeg = m_vAngles[ii-1]     ;
                  double dEndAngDeg   = m_vAngles[ii]       ;
                  double dAngleDeg    = dPolarParameterDeg ;

                  // Try cheap exact conversion when possible
                  SER(AngleParamToNurbParam(m_vNaturalKnots[ii-1],
                                            m_vNaturalKnots[ii],
                                            dStartAngDeg, 
                                            dEndAngDeg,
                                            dAngleDeg, 
                                            rdNurbsParameter));
                  rbExactConversion = m_vExactConversion[ii-1];
                  dNURBSGuess       = rdNurbsParameter ;
                  break ;
                } // end found containing arc interval check
            } // end iter every stored arc interval boundary boundary
        } // end param not a lower boundary branch
    } // end m_bPolarConversionPossible check

  // gwc: for tolerance purposes - always refine the current guess with a NewtonRaphson solution
  // when polar conversion is not possible nor exact
  //      if(   !m_bPolarConversionPossible
  //         || !rbExactConversion) 
    {  
      // Use numerical techniques to convert to NURBS coordinates.
             
      // get Target 3DPoint and see if current guess is good enough
      SmPoint3d s3DPnt[1], sNurbPnt;
      SER(m_cpCurve->EvaluateSTEP(dPolarParameterDeg, 0, FALSE, s3DPnt, dZoneTol3d));
      SER(m_cpCurve->EvaluatePoint(dNURBSGuess, sNurbPnt));
      double dDist = (sNurbPnt - s3DPnt[0]).Length() ;

      // when current guess is good enough
      if(dDist < SM_EFF_ZERO * (1.0 + sNurbPnt.GetMaxDimension()))
        {
          // set output
          rdNurbsParameter  = dNURBSGuess ;
          rbExactConversion = TRUE ;
        }
      else // Find closest point on Curve to GuessPoint 
        {
          SmBoolean bFoundAnswer;
          SmSolution sSolution;
          SER(m_cpCurve->LocalPropertyAnalysis(m_cpCurve->GetNaturalInterval(), 
                                               SM_CP_POINT_INVERSION,
                                               dNURBSGuess, NULL, &s3DPnt[0],
                                               bFoundAnswer, sSolution));
          //      SER(m_cpCurve->LocalPointSolve(m_cpCurve->GetNaturalInterval(),
          //                                     SM_SO_MINIMIZE, 
          //                                     s3DPnt, NULL, NULL, NULL, 
          //                                     dNURBSGuess,
          //                                     bFoundAnswer, sSolution));
          SER_MSG((bFoundAnswer ? SM_SUCCESS : SM_ERR),  _T("ConvertToNURBSParemeter: LocalPropertyAnalysis failed when it should have worked")) ;

          // set output
          rdNurbsParameter  = sSolution.m_vStart[0];
          rbExactConversion = TRUE ;
        }

    } // end need for numerical iteration check

#ifdef SM_DEBUG_CODE
SmBoolean bFirstCall = FALSE ; // This static will not work in multi-threading
                               // Setting it false will skip the test totally.
  if(bFirstCall)
    {
      // verify the conversion is reciprical
      double dCheckParamDeg ;
      SmBoolean bCheckExact ;
//      bFirstCall = FALSE ;
      ConvertToPolarParameter(rdNurbsParameter, dCheckParamDeg, bCheckExact) ;
//      bFirstCall = TRUE ;
      double dScaledZero =   SM_EFF_ZERO 
                           * (1.0 +  180.0/SM_PI 
                                   * smos_Max(smos_Fabs(m_vNaturalKnots[0]),
                                              smos_Fabs(m_vNaturalKnots.GetLast()))) ;
      SM_ASSERT(   bCheckExact       != TRUE
                || rbExactConversion != TRUE
                || smos_Fabs(dPolarParameterDeg - dCheckParamDeg) < dScaledZero
                || smos_Fabs(dPolarParameterDeg - dCheckParamDeg) - 360.0 < dScaledZero) ;

      // verify that the conversion is valid
      double dDist = 0.0 ;
      if(m_cpCurve->IsKindOf(SmEllipse_TYPE))
        {
          // only check for ellipse type curve - 
          // BSplineCurve::EvaluateSTEP just returns Evaluate(param)
          SmPoint3d sSTEPPoint, sNurbPoint ;
          m_cpCurve->EvaluateSTEP(dPolarParameterDeg, 0, TRUE, &sSTEPPoint) ;
          m_cpCurve->Evaluate    (rdNurbsParameter,   0, TRUE, &sNurbPoint) ;

          dDist  = (sSTEPPoint - sNurbPoint).Length() ;
          dScaledZero = SM_EFF_ZERO * 10000.0 * (1.0 + sNurbPoint.GetMaxDimension()) ;
          SM_ASSERT(dDist <= dScaledZero) ;
        }

      if(bDebugMe || dDist > dScaledZero)
        {
          m_cpCurve->Dump() ;
        }
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPolarConversion::ConvertToNURBSParameter

/*******************************************************************//**
PURPOSE: Convert from a NURBS parameter to a Polar parameter.

NOTES: When m_cpCurve is not analytic will just set
                rdPolarParameterDeg = dNurbsparameter
***********************************************************************/
SmStatus SmPolarConversion::ConvertToPolarParameter
  (double      dNurbsParameter,       // in : target nurbs parameter
   double    & rdPolarParameterDeg,   // out: equivalent polar parameter in degrees
   SmBoolean & rbExactConversion)     // out: TRUE  = conversion was exact
                                      //      FALSE = conversion was approximate to about SM_ZONE_TOL_3D/10.0 
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(m_cpCurve) ;
    }
#endif // SM_DEBUG_CODE

  NER( m_cpCurve ); // Can't do anything if not set.

  // for tolerance reasons - don't use the polar conversion function

  // avoid roundoff errors at boundaries
  SmPoint3d sPoint ;
  SmExtent1d sAnalIvl = m_cpCurve->GetSTEPInterval() ;
  SmExtent1d sNurbIvl = m_cpCurve->GetNaturalInterval() ;
  SmExtentPointType ePointType = sNurbIvl.ClassifyPoint(dNurbsParameter, SM_EFF_ZERO) ;

  // switch on ePointType - map boundary values and invert internal values
  switch(ePointType)
    {
      case SM_EP_START   : rdPolarParameterDeg = (m_cpCurve->GetInsideOut() == 0) ? sAnalIvl.GetMin() : sAnalIvl.GetMax() ;
                           break ;
      case SM_EP_END     : rdPolarParameterDeg = (m_cpCurve->GetInsideOut() == 0) ? sAnalIvl.GetMax() : sAnalIvl.GetMin() ;
                           break ;
      case SM_EP_BOTH    : if(ePointType == SM_EP_BOTH   ) WARN(_T("ConvertToPolarParameter: input dNurbsParameter classifies to both ends of NurbIvl"));
      case SM_EP_OUTSIDE : if(ePointType == SM_EP_OUTSIDE) WARN(_T("ConvertToPolarParameter: input dNurbsParameter classifies outside of NurbIvl"));
      case SM_EP_UNKNOWN : if(ePointType == SM_EP_UNKNOWN) WARN(_T("ConvertToPolarParameter: input dNurbsParameter classifies to UNKNONW - a bug"));
      case SM_EP_INSIDE  : // use a conversion based arcTan2
                           m_cpCurve->EvaluatePoint(dNurbsParameter, sPoint) ;
                           m_cpCurve->STEPInversion(sPoint, rdPolarParameterDeg) ;
                           break ;
    } // end switch on dNurbsParameter NurbIvl classification type

  rbExactConversion = TRUE ;

  // make sure to return the proper end value for closed curve seam conversions
  if(m_cpCurve->IsClosed(m_cpCurve->GetNaturalInterval()))
    {
      SmBoolean bInsideOut = m_cpCurve->GetInsideOut() ;
      double dScaledZero   =  (m_vNaturalKnots.GetLast() - m_vNaturalKnots[0])
                             / 360.0
                             * SM_EFF_ZERO_DEG ;

      // when values are at opposing ends - move the end angle
      if     (   ( bInsideOut && (   SM_ARE_SAME_TO_TOL(dNurbsParameter,     m_vNaturalKnots.GetLast(),dScaledZero)
                                  && SM_ARE_SAME_TO_TOL(rdPolarParameterDeg, m_vAngles[0],             SM_EFF_ZERO_DEG)))
              || (!bInsideOut && (   SM_ARE_SAME_TO_TOL(dNurbsParameter,     m_vNaturalKnots[0], dScaledZero)
                                  && SM_ARE_SAME_TO_TOL(rdPolarParameterDeg, m_vAngles.GetLast(),SM_EFF_ZERO_DEG))))
        { 
          // increment, rather than PolarParam = minAngle, to preserve tolerances 
          rdPolarParameterDeg -= 360.0 ; 
        }
      else if(   ( bInsideOut && (   SM_ARE_SAME_TO_TOL(dNurbsParameter,     m_vNaturalKnots[0], dScaledZero)
                                  && SM_ARE_SAME_TO_TOL(rdPolarParameterDeg, m_vAngles.GetLast(),SM_EFF_ZERO_DEG)))
              || (!bInsideOut && (   SM_ARE_SAME_TO_TOL(dNurbsParameter,     m_vNaturalKnots.GetLast(),dScaledZero)
                                  && SM_ARE_SAME_TO_TOL(rdPolarParameterDeg, m_vAngles[0],             SM_EFF_ZERO_DEG))))
        { 
          // increment, rather than PolarParam = maxAngle, to preserve tolerances 
          rdPolarParameterDeg += 360.0 ; 
        }

    } // end closed curve check

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(m_cpCurve) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

  //      // init output - get rdPolarParameterDeg guess 
  //      rbExactConversion      = FALSE ;
  //      SmExtent1d sSTEPIvl    = m_cpCurve->GetSTEPInterval() ;
  //      SmExtent1d sNurbIvl    = m_cpCurve->GetNaturalInterval() ;
  //      double     dScaledZero = SM_EFF_ZERO * (1.0 + sNurbIvl.GetLength()) ;
  //      rdPolarParameterDeg    =  sSTEPIvl.GetMin()
  //                              + (dNurbsParameter - sNurbIvl.GetMin())/(sNurbIvl.GetMax() - sNurbIvl.GetMin()) 
  //                              * (sSTEPIvl.GetMax() - sSTEPIvl.GetMin()) ;
  //      
  //      // when exact conversions tables are initialized - get a translation or a better guess
  //      if(m_bPolarConversionPossible)
  //        {
  //          if (dNurbsParameter < m_vNaturalKnots[0]        - dScaledZero) SER(SM_ERR);
  //          if (dNurbsParameter > m_vNaturalKnots.GetLast() + dScaledZero) SER(SM_ERR);
  //      
  //          // check first knot boundary
  //          if(SM_ARE_SAME_TO_TOL(dNurbsParameter, m_vNaturalKnots[0], dScaledZero)) 
  //            {
  //              rbExactConversion   = TRUE ;
  //              rdPolarParameterDeg = m_vAngles[0] ;
  //            }
  //          else
  //            { // for every arc interval
  //              for (ULONG ii=1; ii<m_vNaturalKnots.GetSize(); ii++) 
  //                {
  //                  // check interval upper bound
  //                  if(SM_ARE_SAME_TO_TOL(dNurbsParameter, m_vNaturalKnots[ii], dScaledZero)) 
  //                    {
  //                      rbExactConversion   = TRUE ;
  //                      rdPolarParameterDeg = m_vAngles[ii] ;
  //                      break ;
  //                    }
  //      
  //                  // check interval interior
  //                  if (dNurbsParameter <= m_vNaturalKnots[ii]) 
  //                    {
  //                      double dAngleDeg ;
  //                      double dStartSpanAngleDeg = m_vAngles[ii-1] ;
  //                      double dEndSpanAngleDeg   = m_vAngles[ii]   ;
  //      
  //                      // Try cheap exact conversion when possible
  //                      SER(NurbParamToAngleParam(m_vNaturalKnots[ii-1],
  //                                                m_vNaturalKnots[ii],
  //                                                dStartSpanAngleDeg,
  //                                                dEndSpanAngleDeg,
  //                                                dNurbsParameter,
  //                                                dAngleDeg));
  //      
  //                      // set outputs (degrees and conversion exactness flag)
  //                      rdPolarParameterDeg = dAngleDeg ;
  //                      rbExactConversion   = m_vExactConversion[ii-1];
  //                      break ;
  //                    } // end in span check
  //                } // end iter every interval
  //            } // end param not a lower boundary branch
  //        } // end m_bPolarConversionPossible check
  //      
  //      // when polar conversion is not possible nor exact
  //      if(   (   !m_bPolarConversionPossible
  //             || !rbExactConversion)
  //         && (!SM_ARE_SAME_TO_TOL(rdPolarParameterDeg, dNurbsParameter, dScaledZero)))
  //        {  
  //          // Use numerical techniques to convert to STEP Polar coordinates.
  //          // const SmContext * pContext  = m_cpCurve->GetContext();   //unused
  //      
  //          // get Target 3DPoint and see if current guess is good enough
  //          SmPoint3d s3DPnt, sSTEPPoint ;
  //          SER(m_cpCurve->EvaluatePoint(dNurbsParameter, s3DPnt));
  //          SER(m_cpCurve->EvaluateSTEPPoint(rdPolarParameterDeg, sSTEPPoint));
  //          double dDist = (sSTEPPoint - s3DPnt).Length() ;
  //      
  //          // when current guess is good enough
  //          if(dDist < SM_EFF_ZERO * (1.0 + sSTEPPoint.GetMaxDimension()))
  //            {
  //              // set output
  //              rbExactConversion = TRUE ;
  //            }
  //          else // Find closest point on Curve to GuessPoint
  //            {
  //              SmSolutionArray sSolutions ;
  //              SER(((SmBSplineCurve *)m_cpCurve)->GlobalPointSolveSTEP
  //                                    (m_cpCurve->GetSTEPInterval(),
  //                                     SM_SO_MINIMIZE, s3DPnt, 
  //                                     0.0, NULL, NULL, 
  //                                     SM_SR_SINGLE, sSolutions));
  //              if (sSolutions.GetSize() == 0) 
  //                { return SM_ERR; }
  //      
  //              // set output
  //              rdPolarParameterDeg  = sSolutions[0].m_vStart[0];
  //              rbExactConversion    = TRUE ;
  //           }
  //      
  //        } // end need for numerical iteration check
  //      
  //      // all done
  //      return(SM_SUCCESS) ;

} // end SmPolarConversion::ConvertToPolarParameter

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPolarIsCurrent_list[] = /* SM_LIST_1 */
{
/*  0 */  {SM_AT_KNOTS,  _T("Knots"),  _T("Number of sNaturalKnots != number of m_vNaturalKnots") },
/*  1 */  {SM_AT_KNOTS,  _T("Angles"), _T("sNaturalKnot values != m_vNaturalKnot values") },
/*  2 */  {SM_AT_ANGLE,  _T("Angles"), _T("Computed Angles must match Anlges in storage") },
/*  3 */  {SM_AT_ANGLE,  _T("Angles"), _T("Stored Angles must increase or decrease monotonically") },
} ;

/*******************************************************************//**
PURPOSE: Return TRUE when all cached data is consistent
            with the nested curve.

NOTES:
***********************************************************************/
SmBoolean SmPolarConversion::IsCurrent
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests);

  // init return
  SmBoolean bRtn = TRUE ;

  // These were already checked in the calling function
  if(   m_cpCurve == NULL || m_bPolarConversionPossible == FALSE)
    return FALSE;
    
  // locals
  ULONG ii ;
  SmExtent1d sNurbIvl = m_cpCurve->GetNaturalInterval() ;
  SmExtent1d sStepIvl = m_cpCurve->GetSTEPInterval() ;
  SmAxis2Placement sPlacement ;
  double           dRad1 = 0.0, dRad2 = 0.0;
  SmTArray<ULONG>  sMultiplicities;
  SmTArray<double> sNaturalKnots; 
  SmTArray<double> sAngles;

  // get curve knots
  m_cpCurve->GetKnots(sNaturalKnots,&sMultiplicities);
  ULONG lCnt = sNaturalKnots.GetSize() ;

  // for each knot - check stored knot value
  bRtn &= SM_ASSERT_OBJ_BOOLEAN_REPORT(this, SM_LIST_1, 0, SM_LEVEL_0, (sNaturalKnots.GetSize() == m_vNaturalKnots.GetSize()), this, _T("") ) ;

  for(ii=0;ii<lCnt && bRtn ;ii++) 
    {  
      bRtn &= SM_ASSERT_OBJ_BOOLEAN_REPORT(this, SM_LIST_1, 1, SM_LEVEL_0, SM_ARE_SAME(sNaturalKnots[ii], m_vNaturalKnots[ii]), this, _T("") )  ;
    } // end iter every knot checking angle values

  // build angles for circles and ellipses
  double dAngleTol = SM_EFF_ZERO_DEG ;
  if( bRtn && m_cpCurve->IsKindOf(SmEllipse_TYPE))
    {
      ((SmEllipse *)m_cpCurve)->GetCanonical(sPlacement, dRad1, dRad2) ;
      //const SmVector3d &rOrigin = sPlacement.GetOriginRef() ;
      //const SmVector3d &rXAxis  = sPlacement.GetXAxisRef() ;
      SmVector3d        sZAxis  = sPlacement.GetZAxis() ;

      // monotonic local
      double dThisDir, dAngleDir = 0.0 ;

      // for each knot get a PolarParam angleDeg
      for(ii=0;ii<lCnt && bRtn;ii++)
        {
          // Get CCW angle to vec = (thisKnotPoint - Origin)
          SmPoint3d sPoint ;
          m_cpCurve->EvaluatePoint(sNaturalKnots[ii], sPoint) ;

          // convert point to polar param (deg)
          double dAngleDeg ;
          m_cpCurve->STEPInversion(sPoint, dAngleDeg) ;

          //      SmVector3d sVec = sPoint - rOrigin ;
          //      double dAngRad ;
          //      sZAxis.CCWAngleBetween(rXAxis, sVec, dAngRad) ;
          //      double dRawAngDeg = dAngRad * 180.0 / SM_PI ; 
          //      double dAngDeg = sStepIvl.ClampPeriodicValue(dRawAngDeg, 360.0) ;

          // computed angles must match angles in storage
          bRtn &= SM_ASSERT_OBJ_VALUE_REPORT(this, SM_LIST_1, 2, SM_LEVEL_0, 
                                         (   SM_ARE_SAME_TO_TOL(dAngleDeg, m_vAngles[ii], dAngleTol) 
                                          || SM_ARE_SAME_TO_TOL(dAngleDeg,       m_vAngles[ii]+360.0, dAngleTol) 
                                          || SM_ARE_SAME_TO_TOL(dAngleDeg+360.0, m_vAngles[ii],       dAngleTol)), 
                                         this,
                                         dAngleTol, smos_3Min(dAngleDeg       - m_vAngles[ii],      
                                                              dAngleDeg       - m_vAngles[ii]+360.0,
                                                              dAngleDeg+360.0 - m_vAngles[ii]), 
                                         _T("") );

          // angles must increase or decrease monotonically
          if (ii == 1) 
            { 
              dAngleDir = (m_vAngles[ii] - m_vAngles[ii-1] > 0.0) ? 1.0 : -1.0 ; 
            }
          else if(ii  > 1) 
            { 
              dThisDir  = (m_vAngles[ii] - m_vAngles[ii-1] > 0.0) ? 1.0 : -1.0 ;
              bRtn &= SM_ASSERT_OBJ_BOOLEAN_REPORT(this, SM_LIST_1, 3, SM_LEVEL_0, (dThisDir == dAngleDir), this, _T("") );
            }

        } // end iter every knot checking angle values
    } // end Curve is an ellipse type check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
      if(FALSE)
        {
          SmEllipse *pTempCrv = SM_CONST_CAST( SmEllipse*, m_cpCurve );
          ((SmPolarConversion *)this)->SetUpPolarConversion(pTempCrv, 
                                                            m_bIsBorrowed, 
                                                            m_cpCurve->GetSTEPInterval(), 
                                                            m_cpCurve->GetPosition()) ;
        }  
          
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmPolarConversion::IsCurrent

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPolarConversion_list[] =
{
  {SM_AT_FLAG,  _T("Flags"),       _T("m_bPolarConversionPossible must be false when m_cpCurve is NULL") },
  {SM_AT_CACHE, _T("Nested Call"), _T("SmPolarConversion::IsCurrent() failed") },
} ;

/*******************************************************************//**
PURPOSE: Make sure cached data is current with curve description

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmPolarConversion::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests);

  SmBoolean bRtn = TRUE ;
  
  if(m_cpCurve == NULL)
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_bPolarConversionPossible == FALSE), _T("") ) ;
  else
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, IsCurrent(pAList), _T("") ) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump() ;
      m_cpCurve->Dump() ;

      SmEdge *pEdge = (SmEdge *)m_cpCurve->GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(m_cpCurve) m_cpCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  return(bRtn) ;

} // end SmPolarConversion::AssertValid

/*******************************************************************//**
PURPOSE: Default Constructor for an ellipse object.

NOTES: 
***********************************************************************/
SmEllipse::SmEllipse
  (ULONG             lDimension,    // in : 
   const SmContext * cpContext )    // in : 
 : SmBSplineCurve(lDimension),
   m_dRadiusAtXAxis(0.0),  
   m_dRadiusAtYAxis(0.0)
{ 
  // when passed a context - use it
  if(cpContext) { SM_ASSERT(   GetContext() == NULL 
                            || GetContext() == cpContext) ; 
                  SetContext(cpContext) ; 
                }
  SM_ASSERT(GetContext() != NULL) ;

} // end SmEllipse::SmEllipse default constructor

/*******************************************************************//**
PURPOSE: Constructor for an ellipse object.

NOTES: 
***********************************************************************/
SmEllipse::SmEllipse
  (const SmPoint3d      & crCenter,        // in : Center of Ellipse
   const SmVector3d     & crXAxis,         // in : X axis of ellipse - corresponds to an angle of 0 degrees
   const SmVector3d     & crYAxis,         // in : Y axis of ellipse - corresponds to an angel of 90 degrees
   const SmExtent1d     & crAnalDomain,    // in : Angular domain.  (Max - Min) <= 360.0
   double                 dRadiusAtXAxis,  // in : X Axis Radius
   double                 dRadiusAtYAxis,  // in : Y Axis Radius
   ULONG                  lDimension,      // in : oneof 2 or 3
   const SmContext      * cpContext,       // in : required when making an automatic variable
                                           //      optionally when using overloaded new.
   const SmBSplineCurve * pOptNurb,        // in : Optional Nurb copied to make m_pNurb object
                                           //      If Given, its StartPoint == EllipseStart
                                           //                its EndPoint   == EllipseEnd
                                           //                it must lie in the plane [crCenter, cross(X,Y)]
                                           //                its radius     == radius
                                           //      default:[NULL]
   SmBoolean              bInsideOut)      // in : TRUE = Nurb Tangent = -LineVector
                                           //      FALSE= Nurb Tangent =  LineVector
 : SmBSplineCurve(lDimension),
   m_vPosition(crCenter,crXAxis,crYAxis),             
   m_vAnalDomain(crAnalDomain),
   m_dRadiusAtXAxis(dRadiusAtXAxis),
   m_dRadiusAtYAxis(dRadiusAtYAxis),
   m_bInsideOut(bInsideOut)
{
  // check inputs
  SM_ASSERT((crAnalDomain.GetMax() - crAnalDomain.GetMin()) <=  360.0 + SM_EFF_ZERO) ;
  SM_ASSERT( dRadiusAtXAxis > SM_EFF_ZERO);
  SM_ASSERT( dRadiusAtYAxis > SM_EFF_ZERO);
  SM_ASSERT( crAnalDomain.GetLength() > SM_EFF_ZERO) ;

#ifdef SM_DEBUG_CODE
  if(   dRadiusAtXAxis           <= SM_EFF_ZERO
     || dRadiusAtYAxis           <= SM_EFF_ZERO
     || crAnalDomain.GetLength() <= SM_EFF_ZERO)
    {
      WARN(_T("Bad inputs to Ellipse Constructor")) ;
    }
#endif // SM_DEBUG_CODE

  // when passed a context - use it
  if(cpContext) { SM_ASSERT(   GetContext() == NULL 
                            || GetContext() == cpContext) ; 
                  SetContext(cpContext) ; 
                }
  SM_ASSERT(GetContext() != NULL) ;

  // make the associated NURB curve
  SmBoolean bCallMakeNurb = TRUE;
  if(pOptNurb)
    {
      gw_CURVE *pGwNurbCurve = pOptNurb->GetGwNurbPointer() ;

      // copy and save the gw_CURVE 
      SmStatus eStat = SetFromGwNurb(0, pGwNurbCurve);

      // If problems there, call MakeNurb().
      if ( eStat == SM_SUCCESS && m_vPolarConverter.IsPolarConversionPossible() )
        { bCallMakeNurb = FALSE; }
    }

  if ( bCallMakeNurb )
    { 
      SE(MakeNurb()); 
    }
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // set form factor   
  m_eBSplineCurveForm = SM_CF_ELLIPTIC_ARC;

} // end SmEllipse::SmEllipse constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmEllipse

NOTES: 
***********************************************************************/
SmEllipse::SmEllipse
  (const SmEllipse & crSource)                 // in : target surface to copy
 : SmBSplineCurve(crSource),
   m_vPosition(crSource.m_vPosition),
   m_vAnalDomain(crSource.m_vAnalDomain),
   m_dRadiusAtXAxis(crSource.m_dRadiusAtXAxis),
   m_dRadiusAtYAxis(crSource.m_dRadiusAtYAxis),
   m_bInsideOut(crSource.m_bInsideOut)
{
  // stop infinite recursion
  // don't copy underlying PolarConverter - reSet it
  m_vPolarConverter.SetUpPolarConversion(this, TRUE, m_vAnalDomain, m_vPosition) ;

} // end SmEllipse::SmEllipse copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmEllipse

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmEllipse::operator==
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
      SmEllipse &rOther = (SmEllipse &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_vPosition       == rOther.m_vPosition
              && m_vAnalDomain     == rOther.m_vAnalDomain
              && SM_IS_ZERO(m_dRadiusAtXAxis - rOther.m_dRadiusAtXAxis)  
              && SM_IS_ZERO(m_dRadiusAtYAxis - rOther.m_dRadiusAtYAxis)  
              && m_vPolarConverter == rOther.m_vPolarConverter
              && m_bInsideOut      == rOther.m_bInsideOut) ;                       
    }

  // all done
  return bRtn ;

} // end SmEllipse::operator==

/*******************************************************************//**
PURPOSE: Given a rational parameter of an arc span (defined by a start
    & an end angle) and parameter domain, this routine will calculate
    the corresponding angular parameter.

NOTES: Works only for one quadratic segment of a circular arc. 
   The start/end values must be for a single knot span in the arc and
   not the whole curve.

METHOD:
    Let SpanAngle denote the size of the span and 
        theta = SpanAngle/2.
    If angle satisfied: 0 <= angle <= theta < PI,
    Then, solve t (Nurbs param) from the following quadratic equation

    (CAGD - Hoschek/Lasser, p.157, A K PETERS)
                               2                        2
                          (1-t)  + 2t(1-t)COS(theta) + t COS(2theta)
            COS(angle) = --------------------------------------------
                               2                        2
                          (1-t)  + 2t(1-t)COS(theta) + t

    dStartNurbParam - Start parameter of the span
    dEndNurbParam   - End parameter of the span
    dStartAngle     - start angle of the span
    dEndAngle       - end angle of the span
    dNurbParam      - input quadratic parameter
    rdAngleParam    - output angular parameter

    This algorithm only works for arcs.

***********************************************************************/
SmStatus SmPolarConversion::NurbParamToAngleParam
  (double dStartSpanNurbParam,  // in : Span Start parameter - not curve StartParam
   double dEndSpanNurbParam,    // in : Span End parameter   - not curve EndParam
   double dStartSpanAngleDeg,   // in : Span Start angle [degrees] - not curve StartAngle  
   double dEndSpanAngleDeg,     // in : Span End angle [degrees]   - not curve EndAngle     
   double dNurbParam,           // in : input quadratic parameter  
   double & rdAngleParamDeg)    // out: output angular parameter in degrees   
  const
{ 
  // get normalized T value                                                                 
  double dT           = (dNurbParam - dStartSpanNurbParam)/(dEndSpanNurbParam - dStartSpanNurbParam);
  double dTSq         = dT*dT;
  double dTheta       = (dEndSpanAngleDeg - dStartSpanAngleDeg)/2.0 * SM_PI/ 180.0 ;

  // evaluate the expression
  double dNumerator   = (1.0-dT)*(1.0-dT) + 2*dT*(1.0-dT)*cos(dTheta) + dTSq*cos(2*dTheta);
  double dDenominator = (1.0-dT)*(1.0-dT) + 2*dT*(1.0-dT)*cos(dTheta) + dTSq;
  double dAngleRad    = acos(dNumerator/dDenominator);

  // watch out for backward sweeping intervals (the angle needs to be negated)
  if(dStartSpanAngleDeg > dEndSpanAngleDeg) { dAngleRad = -dAngleRad ; }

  // convert angle back into given range
  rdAngleParamDeg     =   dStartSpanAngleDeg
                        + dAngleRad * 180.0/SM_PI ; 
  // all done
  return SM_SUCCESS;

} // end SmPolarConversion::NurbParamToAngleParam

/*******************************************************************//**
PURPOSE: Given an angular parameter of an arc span (defined by a start
    & an end angle), this routine will calculate its normalized ([0,1])
    quadratic-nurb parameter.

NOTES: Works only for one quadratic segment of a circular arc
   specified by 2 knots and 3 control Points.  The weights on the
   control point must be {1.0 cos(arcAngle/2.0) 1.0}.  The multiplicities on
   the knots must be 2 (for interior knots) or 3 (for end knots).
   The start/end values must be for a single knot span in the arc and
   not the whole curve.

METHOD:
    Let SpanAngle denotes the size of each span and theta = SpanAngle/2.
    If angle satisfied: 0 <= angle <= theta < PI,
    Then, solve t (Nurbs param) from the following quadratic equation

    (CAGD - Hoschek/Lasser, p.157, A K PETERS)
                               2                        2
                          (1-t)  + 2t(1-t)COS(theta) + t COS(2theta)
            COS(angle) = _____________________________________________
                               2                        2
                          (1-t)  + 2t(1-t)COS(theta) + t

  0 =   COS(angle) * (1-t)*(1-t)             =   COS(angle) * (1 - 2*t + t*t)           
      + COS(angle) * 2*t * (1-t)*COS(theta)    + COS(angle) * (2*COS(theta)*t - 2*COS(theta)*t*t) 
      + COS(angle) * t*t                       + COS(angle) * t*t                   
      - (1-t)*(1-t)                            + -1 + 2*t - t*t                        
      - 2*t * (1-t) * COS(theta)               + -2*COS(theta)*t + 2*COS(theta)*t*t           
      - t*t*COS(2theta)                        + -COS(2theta)*t*t                    

  0 = t*t * ( 2*COS(angle)*( 1.0 - COS(theta)) - 1.0 + 2*COS(theta) - COS(2*theta))
      + t * ( 2*COS(angle)*(-1.0 + COS(theta)) + 2.0 - 2*COS(theta))
      +     (COS(angle) - 1.0) 

    dStartSpanAngle - start angle of the span
    dEndSpanAngle   - end angle of the span
    dAngleParam - input angular parameter
    rdNurbParam - output normalized quadratic parameter between [0,1]

***********************************************************************/
SmStatus SmPolarConversion::AngleParamToNurbParam
  (double dStartSpanNurbParam,  // in : Span Start parameter - not curve StartParam
   double dEndSpanNurbParam,    // in : Span End parameter   - not curve EndParam
   double dStartSpanAngleDeg,   // in : Span Start angle [degrees] - not curve StartAngle  
   double dEndSpanAngleDeg,     // in : Span End angle [degrees]   - not curve EndAngle     
   double dAngleParamDeg,       // in : angular parameter [degrees]
   double & rdNurbParam)        // out: Nurb param in given Nurb interval
 const
{
  // locals
  double dTheta    = (dEndSpanAngleDeg - dStartSpanAngleDeg)/2.0 * SM_PI/180.0 ;
  double dAngleTol =  SM_EFF_ZERO_DEG ;
  
  // watch out for periodicity
  SmExtent1d sIvl(smos_Min(dStartSpanAngleDeg, dEndSpanAngleDeg),
                  smos_Max(dStartSpanAngleDeg, dEndSpanAngleDeg)) ;
  SmBoolean bInside = sIvl.ContainsValue( dAngleParamDeg ); // Note, don't check 'periodic'. [B90]
  dAngleParamDeg = sIvl.ClampValue( dAngleParamDeg );

  SER_MSG(bInside ? SM_SUCCESS : SM_ERR, _T("AngleParam not contained in given PolarConversion Interval")) ;

  // quick convert end parameters
  if     (smos_Fabs(dAngleParamDeg-dStartSpanAngleDeg) < dAngleTol) 
    { rdNurbParam = dStartSpanNurbParam;
      return SM_SUCCESS;
    }
  else if(smos_Fabs(dAngleParamDeg-dEndSpanAngleDeg)   < dAngleTol) 
    { rdNurbParam = dEndSpanNurbParam;
      return SM_SUCCESS;
    }

  // shift angle to measure from StartAngle
  double dAngleRad = (dAngleParamDeg - dStartSpanAngleDeg) * SM_PI/180.0 ;

  // 0 = t*t * ( 2*COS(angle)*( 1.0 - COS(theta)) - 1.0 + 2*COS(theta) - COS(2*theta))
  //     + t * ( 2*COS(angle)*(-1.0 + COS(theta)) + 2.0 - 2*COS(theta))
  //     +     (COS(angle) - 1.0) 

  // get coefficients of quadratic equation
  double dCosAngle = cos(dAngleRad) ;
  double dCosTheta = cos(dTheta) ;
  double dA = 2.0*dCosAngle * ( 1.0 - dCosTheta) - 1.0 + 2.0*dCosTheta - cos(2.0*dTheta) ;
  double dB = 2.0*dCosAngle * (-1.0 + dCosTheta) + 2.0 - 2.0*dCosTheta ;
  double dC = dCosAngle - 1.0 ;

  // quadratic equation determinate
  double dD = dB*dB - 4.0*dA*dC ;

  // check determinate for error condition
  if (dD < 0.0) { SE(SM_ERR) ;
                  return SM_ERR; 
                }

  // Solve quadratic Equation for Normalized T - get solution in region [0.0 to 1.0]
  dD = sqrt(dD);
  double dT = (-dB + dD)/(2.0*dA);

  if ( dT < 0.0 || dT > 1.0 )
    {
      dT = (-dB - dD) / ( 2.0*dA );

      if (dT < 0.0 || dT > 1.0)
        {
          SE(SM_ERR) ;
          return SM_ERR;
        }
    }

  // scale normalized T to Nurb interval
  double dNurbParam = (dEndSpanNurbParam - dStartSpanNurbParam) * dT + dStartSpanNurbParam ;

  // set output
  rdNurbParam = dNurbParam;

#ifdef SM_DEBUG_CODE
  // check conversion for reciprocity
  double dCheckParamDeg ;
  NurbParamToAngleParam(dStartSpanNurbParam,
                        dEndSpanNurbParam,
                        dStartSpanAngleDeg,
                        dEndSpanAngleDeg,
                        rdNurbParam,
                        dCheckParamDeg) ;
  double dDist = smos_Fabs(dCheckParamDeg - dAngleParamDeg) ;
  
  // only check to very loose tolerance now that all angle conversions
  // are refined with a NewtonRaphson solve                        
  SM_ASSERT(dDist < SM_EFF_ZERO_DEG) ;
 
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPolarConversion::AngleParamToNurbParam

/*******************************************************************//**
PURPOSE: Write SmPolarConversion to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmPolarConversion::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
      
  // locals
  ULONG     ii ; 
  ULONG     lNaturalKnotCnt     = m_vNaturalKnots.GetSize() ;
  ULONG     lAnglesCnt          = m_vAngles.GetSize() ;
  ULONG     lExactConversionCnt = m_vExactConversion.GetSize() ;
  SmBoolean bHasCurve           = m_cpCurve != NULL && m_bIsBorrowed == FALSE ;
                                                                                                    
  if (eType == SM_ASCII)                                                
    {                                                                   
      rFileOut << m_bPolarConversionPossible << " SmPolarConversion PolarConversionPossible - TRUE=tables are initialized \n";        
      rFileOut << lNaturalKnotCnt            << " SmPolarConversion Number of Natural Knots \n";      
      rFileOut << lAnglesCnt                 << " SmPolarConversion Number of Angles \n";      
      rFileOut << lExactConversionCnt        << " SmPolarConversion Number of ExactConversion Flags \n";      
      rFileOut << bHasCurve                  << " SmPolarConversion TRUE = Has Curve for stored conversions, FALSE = back pointer \n" ;
    }                                                                    
  else                                                                   
    {                                                                    
      SER(rDB.WriteBoolean(m_bPolarConversionPossible));
      SER(rDB.WriteLong   (lNaturalKnotCnt));
      SER(rDB.WriteLong   (lAnglesCnt));
      SER(rDB.WriteLong   (lExactConversionCnt));
      SER(rDB.WriteBoolean(bHasCurve));
    }

  // Write Arrays
  if (eType == SM_ASCII) 
    {
      // output the Natural Knots
      for(ii=0; ii<lNaturalKnotCnt; ii++)     { rFileOut << m_vNaturalKnots[ii] << " " ;
                                                if(ii>0 && ii%5 == 0) { rFileOut << "\n"; }
                                              }
      if(   lNaturalKnotCnt > 0 
         && lNaturalKnotCnt %5 != 0)          { rFileOut << "\n"; }

      // output the Angles
      for(ii=0; ii<lAnglesCnt; ii++)          { rFileOut << m_vAngles[ii] << " " ; 
                                                if(ii>0 && ii%5 == 0) { rFileOut << "\n"; }
                                              }
      if(   lAnglesCnt > 0
         && lAnglesCnt %5 != 0)               { rFileOut << "\n"; }

      // output the Exact Conversion Flags
      for(ii=0; ii<lExactConversionCnt; ii++) { rFileOut << m_vExactConversion[ii] << " " ; 
                                                if(ii>0 && ii%5 == 0) { rFileOut << "\n"; }
                                              }
      if(   lExactConversionCnt > 0
         && lExactConversionCnt %5 != 0)      { rFileOut << "\n"; }
    }
  else //Binary
    {
      if (lNaturalKnotCnt     > 0) { SER(rDB.WriteDoubles(m_vNaturalKnots.GetDataArray(),lNaturalKnotCnt)); }
      if (lAnglesCnt          > 0) { SER(rDB.WriteDoubles(m_vAngles.GetDataArray(),lAnglesCnt)); }
      if (lExactConversionCnt > 0) { SER(rDB.WriteBooleans(m_vExactConversion.GetDataArray(),lExactConversionCnt)); }
    }

  // write out the m_cpCurve it's the curve already being written out that called this write method
  if(bHasCurve) { ULONG lCurveDim = m_cpCurve->GetDim() ;
                  if (eType == SM_ASCII)  { rFileOut << " SmPolarConversion->CurveBeingConverted \n"; }
                  SER(rDB.WriteType(m_cpCurve->GetType(), &lCurveDim)) ; 
                  SER(m_cpCurve->WriteToDB(rDB, lDBVersionNumber)) ;
                }
  else          { if (eType == SM_ASCII)  { rFileOut << "Has No Copy of SmPolarConversion->CurveBeingConverted \n"; }
                }

  // all done
  return SM_SUCCESS;

} // end SmPolarConversion::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmPolarConversion from a given stream  

NOTES:  
  
***********************************************************************/
SmStatus SmPolarConversion::ReadFromDB
 (SmDatabaseIO       & rDB,                 // in : target output stream
  const SmContext    & crContext,           // in : context for new object construction
  const SmEllipse    * cpCurve,             // in : The curve when m_cpCurve is used as a back pointer, else can be NULL when
                                            //       the curve is owned - This value will be created and stored from read values.
  SmPolarConversion *&rpNewPolarConversion, // out: NULL on input = new object allocated in this routine built from stream data
                                            //      NotNull on input = assumed empty object already allocated filled here from stream data
  ULONG               lDBVersionNumber)     // in : database version to get proper sequence of writes
{
  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  ULONG      ii ;
  SmBoolean  bPolarConversionPossible ;
  ULONG      lNaturalKnotCnt ;
  ULONG      lAnglesCnt ;
  ULONG      lExactConversionCnt ;
  SmBoolean  bHasCurve ;

  if (eType == SM_ASCII) 
    {
      rFileIn >> bPolarConversionPossible ; rDB.GoToNextLine() ;
      rFileIn >> lNaturalKnotCnt ;          rDB.GoToNextLine() ;        
      rFileIn >> lAnglesCnt ;               rDB.GoToNextLine() ;             
      rFileIn >> lExactConversionCnt ;      rDB.GoToNextLine() ;   
      rFileIn >> bHasCurve ;                rDB.GoToNextLine() ; 
    }
  else 
    {
      SER(rDB.ReadBoolean(bPolarConversionPossible)) ;
      SER(rDB.ReadLong   (lNaturalKnotCnt)) ;         
      SER(rDB.ReadLong   (lAnglesCnt)) ;              
      SER(rDB.ReadLong   (lExactConversionCnt)) ;   
      SER(rDB.ReadBoolean(bHasCurve)) ;  
    }

  // make an empty SmPolarConversion object
  if(rpNewPolarConversion == NULL)
    { rpNewPolarConversion = new SmPolarConversion() ; }

  // set initial values
  rpNewPolarConversion->m_bPolarConversionPossible = bPolarConversionPossible ;
  rpNewPolarConversion->m_vNaturalKnots.SetSize(lNaturalKnotCnt) ;
  rpNewPolarConversion->m_vAngles.SetSize(lAnglesCnt) ;
  rpNewPolarConversion->m_vExactConversion.SetSize(lExactConversionCnt) ;

  // read arrays
  double    *pNaturalKnots     = rpNewPolarConversion->m_vNaturalKnots.GetDataArray() ;
  double    *pAngles           = rpNewPolarConversion->m_vAngles.GetDataArray() ;
  SmBoolean *pExactConversions = rpNewPolarConversion->m_vExactConversion.GetDataArray() ;

  if (eType == SM_ASCII) 
    {
      for (ii=0; ii<lNaturalKnotCnt; ii++)     { rFileIn >> pNaturalKnots[ii]; 
                                                 if(ii>0 && ii%5 == 0) { rDB.GoToNextLine() ; }
                                               }  
      if(   lNaturalKnotCnt > 0 
         && lNaturalKnotCnt %5 != 0)           { rDB.GoToNextLine() ; }

      for (ii=0; ii<lAnglesCnt; ii++)          { rFileIn >> pAngles[ii];  
                                                 if(ii>0 && ii%5 == 0) { rDB.GoToNextLine() ; }
                                               } 
      if(   lAnglesCnt > 0 
         && lAnglesCnt %5 != 0)                { rDB.GoToNextLine() ; }

      for (ii=0; ii<lExactConversionCnt; ii++) { rFileIn >> pExactConversions[ii];  
                                                 if(ii>0 && ii%5 == 0) { rDB.GoToNextLine() ; }
                                               } 
      if(   lExactConversionCnt > 0 
         && lExactConversionCnt %5 != 0)       { rDB.GoToNextLine() ; }
                                               
    }
  else 
    {//Binary
      SER(rDB.ReadDoubles(pNaturalKnots    ,lNaturalKnotCnt));
      SER(rDB.ReadDoubles(pAngles          ,lAnglesCnt));
      SER(rDB.ReadBooleans(pExactConversions,lExactConversionCnt));
    }

  // containing curve
  if(bHasCurve)
    {
      // read m_cpCurve object from file

      // locals
      SM_TYPE      lContainedType ;
      ULONG        lContainedDim ;
      SmCurve     *pNewCurve = NULL ;

                  
      // read curve type
      if (eType == SM_ASCII)  { rDB.GoToNextLine() ; }
      rDB.ReadType(lContainedType, &lContainedDim) ;

      // pass the call along to appropriate derived type
      SmCurve::ReadFromDB(lContainedType, rDB, lContainedDim, crContext, pNewCurve, lDBVersionNumber) ;

      // save the read curve
      SM_ASSERT(pNewCurve->IsKindOf(SmEllipse_TYPE)) ; 
      rpNewPolarConversion->m_cpCurve = (SmEllipse*) pNewCurve ;
      rpNewPolarConversion->m_bIsBorrowed = FALSE ;
    }
  else // m_cpCurve is a back pointer - its the caller's responsibility to pass that value along to this function
    {
      // containing curve pointer
      rpNewPolarConversion->m_cpCurve = cpCurve ;
      if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
      rpNewPolarConversion->m_bIsBorrowed = TRUE ;
    }

  // all done
  return SM_SUCCESS;

} // end SmPolarConversion::ReadFromDB

/*******************************************************************//**
PURPOSE: Pretty print SmPolarConversion data

NOTES:
***********************************************************************/
void SmPolarConversion::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmPolarConversion::Dump()")) ;

  // header
  smos_sprintf(sBuff, _T("\nSmPolarConversion - m_bPolarConversionPossible = %s"),
             m_bPolarConversionPossible ? _T("TRUE") : _T("FALSE")) ;
  smos_WriteBuffer(sBuff);

  // knots and Angles
  if(m_bPolarConversionPossible)
    {
      smos_WriteBuffer(_T("\n   Bndry   Knot             Angle                  IntervalExact"));
      for(ULONG ii=0;ii<m_vNaturalKnots.GetSize();ii++)
        {
          double    dKnot  = m_vNaturalKnots[ii] ;
          double    dAngle = m_vAngles[ii] ;
          SmBoolean bExact = ii == 0 ? TRUE : m_vExactConversion[ii-1] ;

          smos_sprintf(sBuff, _T("\n   [%2lu] %16.16lf, %16.16lf,    %s"),
                     ii,
                     dKnot, 
                     dAngle, 
                       ii==0 
                     ? _T("   ") 
                     : bExact ? _T("TRUE") : _T("FALSE")) ;
          smos_WriteBuffer(sBuff);
        }
     }
   else
     {
       smos_WriteBuffer(_T("\n   NaturalKnots"));
       m_vNaturalKnots.Dump() ;
     }

  // curve and ownership
  smos_sprintf(sBuff,       _T("\n  Conversion Curve = 0x%p  type:[%s] - %s"), 
             m_cpCurve,
               m_cpCurve->IsKindOf(SmCircle_TYPE)       ? _T("SmCircle")
             : m_cpCurve->IsKindOf(SmEllipse_TYPE)      ? _T("SmEllipse")
             : m_cpCurve->IsKindOf(SmBSplineCurve_TYPE) ? _T("SmBSSplineCurve") : _T("unknown"),
             m_bIsBorrowed ? _T("Borrowed (not deleted with this object)") 
                           : _T("Not Borrowed (deleted with this object)")) ;
  smos_sprintf(sBuffForFile,_T("\n  Conversion Curve = %sn  type:[%s] - %s"), 
             m_cpCurve ? _T("notNULL") : _T("NULL"),
               m_cpCurve->IsKindOf(SmCircle_TYPE)       ? _T("SmCircle")
             : m_cpCurve->IsKindOf(SmEllipse_TYPE)      ? _T("SmEllipse")
             : m_cpCurve->IsKindOf(SmBSplineCurve_TYPE) ? _T("SmBSSplineCurve") : _T("unknown"),
             m_bIsBorrowed ? _T("Borrowed (not deleted with this object)") 
                           : _T("Not Borrowed (deleted with this object)")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);


  smos_WriteBuffer(_T("\nEnd SmPolarConversion::Dump()")) ;

} // end SmPolarConversion::Dump

/*******************************************************************//**
PURPOSE: Convert the curve to a 2D curve by setting the dimension
    to 2 and setting the Z values to 0.

NOTES: This actually projects the ellipse to the x-y plane.
***********************************************************************/
SmStatus SmEllipse::ConvertTo2D
  ()
{
  // modify the NURB
  SmBSplineCurve::ConvertTo2D() ;

  // Get current position
  SmVector3d sOrigin = m_vPosition.GetOriginRef() ;
  SmVector3d sXAxis  = m_vPosition.GetXAxisRef() ;
  SmVector3d sYAxis  = m_vPosition.GetYAxisRef() ;

  // convert position to 2d
  sOrigin.z = 0.0 ;
  sXAxis.z  = 0.0 ;
  sYAxis.z  = 0.0 ;
  SmVector3d sNewX ;
  SmVector3d sNewY ;

  // Most times X and Y are not still perp to one another
  double dXDotX = sXAxis.Dot(sXAxis) ;
  double dXDotY = sXAxis.Dot(sYAxis) ;
  double dYDotY = sYAxis.Dot(sYAxis) ;
  double dScaledZero = SM_EFF_ZERO ;

  // sometimes X and Y are either zero or perp to one another
  // This will happen if and only if at least one of the original
  // axes was parallel to the x-y plane before projection.
  if(smos_Fabs(dXDotY) < dScaledZero)
    {
      // use the projected vectors as the new ellipse vectors
      sNewX = m_dRadiusAtXAxis * sXAxis ;
      sNewY = m_dRadiusAtYAxis * sYAxis ;
    }
  else // axis are no longer perp to one another
    {
      // compute new angles for min/max radius values
      //      Curve = Origin + R1*Cos(Theta)*X + R2*Sin(Theta)*Y
      //      Dist  = (R1*Cos(Theta)*X + R2*Sin(Theta)*Y)**2
      //      min/max distances occur when 1st derivative goes to zero
      //      0 = dDist/dTheta =   ( R1*Cos(Theta)*X + R2*Sin(Theta)*Y)
      //                         * (-R1*Sin(Theta)*X + R2*Cos(Theta)*Y) ;
      //      0 = + R1*R2*X*Y*Cos(Theta)*Cos(Theta)
      //          - R1*R1*X*X*Cos(Theta)*Sin(Theta)
      //          + R2*R2*Y*Y*Sin(Theta)*Cos(Theta)
      //          - R2*R1*Y*X*Sin(Theta)*Sin(Theta)
      //      0 = + R1*R2*X*Y*Cos(Theta)/Sin(Theta)
      //          - R1*R1*X*X
      //          + R2*R2*Y*Y
      //          - R2*R1*Y*X*Sin(Theta)/Cos(Theta)
      //      Let t = Sin(Theta)/Cos(Theta) = Tan(Theta) 
      //      0 = + R1*R2*X*Y/t - R2*R1*Y*X*t + R2*R2*Y*Y-R1*R1*X*X
      //      0 = - (R2*R1*Y*X)*t*t + (R2*R2*Y*Y-R1*R1*X*X)*t + (R1*R2*X*Y)
      double adCoefs[3], adSols[2] ;
      ULONG  lNumSols ;
      adCoefs[0] = -m_dRadiusAtYAxis * m_dRadiusAtXAxis * dXDotY ;
      adCoefs[1] =  m_dRadiusAtYAxis * m_dRadiusAtYAxis * dYDotY
                   -m_dRadiusAtXAxis * m_dRadiusAtXAxis * dXDotX ;
      adCoefs[2] =  m_dRadiusAtYAxis * m_dRadiusAtXAxis * dXDotY ;
      SmPolynomial::SolveQuadraticEqn(adCoefs, dScaledZero, lNumSols, adSols) ;
      SM_ASSERT(lNumSols == 2) ;
      double dThetaXRad = smos_ArcTangent( adSols[0] )  ;
      double dThetaYRad = smos_ArcTangent( adSols[1] )  ;
      if(dThetaYRad < dThetaXRad) { dThetaYRad += SM_PI ; }

      // get max vectors for each angle
      sNewX = m_dRadiusAtXAxis*smos_Cosine(dThetaXRad)*sXAxis + m_dRadiusAtYAxis*smos_Sine(dThetaXRad)*sYAxis ;
      sNewY = m_dRadiusAtXAxis*smos_Cosine(dThetaXRad)*sXAxis + m_dRadiusAtYAxis*smos_Sine(dThetaXRad)*sYAxis ;
    
    } // end projected axis are no longer perp branch
  
  // State: sNewX and sNewY are the actual axis vectors, length and all.

  // compute new radius values
  m_dRadiusAtXAxis = sNewX.Length() ;  
  m_dRadiusAtYAxis = sNewY.Length() ;

  // check for errors
  if(   IsKindOf(SmCircle_TYPE)
     && !SM_ARE_SAME(m_dRadiusAtXAxis, m_dRadiusAtYAxis))
    {
      ERR_MSG(_T("Circle no longer a circle after ConvertTo2D")) ;
    }

  if(   m_dRadiusAtXAxis < dScaledZero    
     || m_dRadiusAtYAxis < dScaledZero)
    {
      ERR_MSG(_T("Ellipse projected to degenerate line by ConvertTo2D")) ;
    }

  // compute new axis values
  // The two radius values are just the lengths of the vectors, taken just above,
  // so this is Unitize(), avoiding a redundant Length() operation.
  if(m_dRadiusAtXAxis > dScaledZero) { sNewX = sNewX/m_dRadiusAtXAxis ; }
  if(m_dRadiusAtYAxis > dScaledZero) { sNewY = sNewY/m_dRadiusAtYAxis ; }

  // Don't pass zero vectors to the SmAxis2Placement.
  if ( m_dRadiusAtXAxis <= dScaledZero )
    { sNewX.Set(  sNewY.y, -sNewY.x, 0.0 ); } // = Y x Z, sNewY is unit.
  if ( m_dRadiusAtYAxis <= dScaledZero )
    { sNewY.Set( -sNewX.y,  sNewX.x, 0.0 ); } // = Z x X, sNewX is unit.

  // modify the Position
  m_vPosition.SetCanonical(sOrigin, sNewX, sNewY) ;

  // recompute the polar conversion
  m_vPolarConverter.SetUpPolarConversion(this, TRUE, m_vAnalDomain, m_vPosition) ;
    
  // all done
  return SM_SUCCESS;

} // end SmEllipse::ConvertTo2D

/*******************************************************************//**
PURPOSE: Convert the line to a 3D line.

NOTES: 
***********************************************************************/
SmStatus SmEllipse::ConvertTo3D
  ()
{
  if (GetDim() == 3) { return SM_SUCCESS; }
  m_lDim = 3;

  // convert the underlying Nurb
  SmBSplineCurve::ConvertTo3D() ;

  // all done
  return SM_SUCCESS;

} // end SmEllipse::ConvertTo3D

/*******************************************************************//**
PURPOSE: Convert parameter from a STEP based parameterization of 
     a curve to a NURBS based parameterization.  

NOTES: By default the NURBS and STEP parameterization of curves
     are the same.  Things like circles, ellipses, and conics have 
     different parameterization.
***********************************************************************/
SmStatus SmEllipse::ConvertTFromSTEPToNURBS
  (double   dSTEPParam,    // in : angle in degrees        
   double & rdNURBSParam)  // out: parameter in Nurb space 
  const
{
  // convert from STEP_s to Parm_p
//        double dParamP = dSTEPParam ;
//        double dParamP = m_bInsideOut ? m_vAnalDomain.GetMin() + m_vAnalDomain.GetMax() - dSTEPParam
//                                      : dSTEPParam ;

  // convert from Param_p to Nurb_n
  SmBoolean bExactConversion ; 
  SmStatus sRtn = m_vPolarConverter.ConvertToNURBSParameter
                                      (dSTEPParam, 
                                       rdNURBSParam,
                                       bExactConversion) ; 
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      // check conversions reciprocity
      // check may fail when ConvertToNURBSParameter was clamped
      double sCheckParam ;
      ConvertTFromNURBSToSTEP(rdNURBSParam, sCheckParam) ; 
      double dTDist       = dSTEPParam - sCheckParam ;
      double dScaledTZero = SM_EFF_ZERO * 1000.0 * (1.0 + smos_Fabs(dSTEPParam)) ;

      if(smos_Fabs(dTDist) >= dScaledTZero)
        { 
          SM_DUMP_AND_ASSERT2_VALID(&m_vPolarConverter) ;
          SM_ASSERT(smos_Fabs(dTDist) < dScaledTZero);
          if (smos_Fabs(dTDist) >= dScaledTZero)
          {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff, _T("ConvertTFromSTEPToNURBS:  dTDist %16.16lf must be less than dScaledTZero  %16.16lf\n"),smos_Fabs(dTDist), dScaledTZero);
              smos_WriteBuffer(sBuff);
          }
        }

      // check quality of the conversion
      SmPoint3d sSTEPPoint, sNurbPoint ;
      Evaluate    (rdNURBSParam, 0, TRUE, &sNurbPoint) ;
      EvaluateSTEP(dSTEPParam,   0, TRUE, &sSTEPPoint) ;
      double dDist = (sNurbPoint - sSTEPPoint).Length() ;
      double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + sNurbPoint.GetMaxDimension()) ;

      SM_ASSERT(dDist < dScaledZero) ;
      if(bDebugMe || dDist > dScaledZero)
        { Dump() ; }

#endif // SM_DEBUG_CODE

  // all done
  return(sRtn) ;

} // end SmCurve::ConvertTFromSTEPToNURBS

/*******************************************************************//**
PURPOSE: Convert parameter from a NURBS based parameterization 
     of a curve to the STEP based parameterization.  

NOTES: By default the NURBS and STEP parameterization of curves
     are the same.  Things like circles, ellipses, and conics have 
     different parameterization.
***********************************************************************/
SmStatus SmEllipse::ConvertTFromNURBSToSTEP
  (double   dNURBSParam,        // in : parameter in Nurb space        
   double & rdSTEPParam)        // out: angle in degrees
  const
{
  // convert from Nurb_N to Polar_P
  SmBoolean bExactConversion ;
  double dPolarP ; 
  SmStatus sRtn = m_vPolarConverter.ConvertToPolarParameter
                                      (dNURBSParam, 
                                       dPolarP, 
                                       bExactConversion) ;

  // convert from Polar_P to STEP_S
  rdSTEPParam = dPolarP ;
//        rdSTEPParam = m_bInsideOut ? m_vAnalDomain.GetMin() + m_vAnalDomain.GetMax() - dPolarP
//                                   : dPolarP ;

  // all done
  return sRtn;

} // end SmEllipse::ConvertTFromNURBSToSTEP

/*******************************************************************//**
PURPOSE: Adjust the curve by the given STEP-interval and rebuild 
            underlying Nurb.

NOTES: Parameterization stays the same, only the extent changes.
***********************************************************************/
SmStatus SmEllipse::AdjustSTEPInterval
  (const SmExtent1d & crNewSTEPInterval)  // in : new arc domain in degrees,
                                          //   range:[-360 <= Min <= Max <= 360], maxLength=360 
{
  // no work - new interval is same as old 
  if(   crNewSTEPInterval.IsContainedBy(m_vAnalDomain, SM_EFF_ZERO)
     && m_vAnalDomain.IsContainedBy(crNewSTEPInterval, SM_EFF_ZERO))
    { return SM_SUCCESS; }

  // Locals to get the Nurbs parameters corresponding to the new STEP interval.
  SmBoolean bExactConversion ;
  double dNurbsT0 = 0.0, dNurbsT1 = 0.0;
  double dStepT0 = crNewSTEPInterval.GetMin();
  double dStepT1 = crNewSTEPInterval.GetMax();

  // After these calls, the conversion may not be reciprocal. Need to reparameterize
  SER( m_vPolarConverter.ConvertToNURBSParameter(dStepT0, dNurbsT0, bExactConversion) );
  SER( m_vPolarConverter.ConvertToNURBSParameter(dStepT1, dNurbsT1, bExactConversion) );
  SmExtent1d sNewNurbsIvl( dNurbsT0, dNurbsT1 );

  // save the input
  m_vAnalDomain = crNewSTEPInterval;

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  SER(MakeNurb());

  // MakeNurb() creates m_pNurb parameterized 0 -> 1   [B284]
  SmBSplineCurve::EditParameterization( sNewNurbsIvl );

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // compare the NURBS parameters to the STEP interval
  double dNurb = dNurbsT0, dStep = dStepT0;
  for (ULONG di = 0; di < 2; di++ )
    {
      if (di == 1)
        { dNurb = dNurbsT1; dStep = dStepT1; }
      
      // check conversions reciprocity
      double sCheckParam ;
      ConvertTFromNURBSToSTEP(dNurb, sCheckParam) ; 
      double dTDist       = dStep- sCheckParam ;
      double dScaledTZero = SM_EFF_ZERO * 1000.0 * (1.0 + smos_Fabs(dStep)) ;

      if(smos_Fabs(dTDist) >= dScaledTZero)
        { 
          SM_DUMP_AND_ASSERT2_VALID(&m_vPolarConverter) ;
          SM_ASSERT(smos_Fabs(dTDist) < dScaledTZero); 
        }

      // check quality of the conversion
      SmPoint3d sSTEPPoint, sNurbPoint ;
      Evaluate    (dNurb, 0, TRUE, &sNurbPoint) ;
      EvaluateSTEP(dStep,   0, TRUE, &sSTEPPoint) ;
      double dDist = (sNurbPoint - sSTEPPoint).Length() ;
      double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + sNurbPoint.GetMaxDimension()) ;

      SM_ASSERT(dDist < dScaledZero) ;
      if(bDebugMe || dDist > dScaledZero)
        { Dump() ; }
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmEllipse::AdjustSTEPInterval

/*******************************************************************//**
PURPOSE: Given a point on the ellipse plane, find its corresponding 
    ellipse angle parameter (0.0 to 360.0 degrees).  

NOTES:
    The returned angle used in EvaluateSTEP(dAngle,sPt) will compute
    a point on the ellipse which is on the line EllipseCenter/GivenPoint.

    Except when the GivenPoint is exactly on the ellipse X or Y axes,
    this line will not be perpendicular to the ellipse, so this
    call is NOT equivalent to dropping a Point onto the ellipse.

    (1) If the curve is closed and we have a point on the seam,
        this method will return ONLY the start parameter (i.e. 0.0)
***********************************************************************/
SmStatus SmEllipse::STEPInversion
  (const SmPoint3d      & crPointOnCurve,     // in : targetPoint - must be on the ellipse to within SM_EFF_ZERO
   double               & rdAngularParameter, // out: angular parameter in degrees,
                                              //      range:m_vAnalDomain or positive
   SmCurveLocationType  * pOptLoc,            // out: Point's classification to curve
                                              //      NULL to ignore, default:[NULL]
   SmZoneTol3d            dZoneTol3d)         // in : Tolerance for returning max param as min on closed curve 
 const
{
  const SmPoint3d  &rCenter = m_vPosition.GetOriginRef();
  const SmVector3d &rXAxis  = m_vPosition.GetXAxisRef();
  const SmVector3d &rYAxis  = m_vPosition.GetYAxisRef();

  // First, find its local coordinates with respect to axes of ellipse
  SmVector3d sVec = crPointOnCurve - rCenter;
  double     dX   = sVec.Dot(rXAxis);
  double     dY   = sVec.Dot(rYAxis);

  // from tan = m_dRadiusAtXAxis/m_dRadiusAtYAxis * dY / dX
  // get dAngleParam in radians [-Pi, +Pi]
  double dAngleParam = 0;

  if (m_dRadiusAtYAxis != 0) {
      dAngleParam = smos_ArcTangent2(m_dRadiusAtXAxis / m_dRadiusAtYAxis * dY, dX);
  }

  // convert to degrees, in [ 0, 360 )
  // strictly positive result [B630]
  if ( dAngleParam < 0 )
    { dAngleParam += SM_2PI; }
  rdAngularParameter =  SM_RAD2DEG(dAngleParam);
                       
  // watch out for periodicity - map output to proper period
  SmBoolean bInArc = m_vAnalDomain.ContainsPeriodicValue(rdAngularParameter, 360.0, &rdAngularParameter) ;                        

  // Get 3d tolerance [B630]
  const SmEdge *pEdge = GetEdge();
  SmZoneTol3d   dZTol3d = 0.0; 
  SmApproxTol3d dApproxTol3d = 0.0;
  if ( dZoneTol3d != 0.0 )
  { dZTol3d = dZoneTol3d; }
  else if ( pEdge )
  { dZTol3d = SmTol::GetZoneTol3d( (const SmObject*)pEdge ); }

  if (dZTol3d == 0.0 ) // Because sometimes an edge has 0.0 tol
  {
      dZTol3d = SmTol::GetZoneTol3d( GetContext() );
  }
  dApproxTol3d = dZTol3d / 100.;

  // Calculate end point on Ellipse
  // Stripped from SmEllipse::EvaluateSTEP to avoid infinite loop in debug checks
  double dAngleRad = m_vAnalDomain.GetMax() * SM_PI / 180.0;
  double dCosU = smos_Cosine( dAngleRad );
  double dSinU = smos_Sine( dAngleRad );
  SmPoint3d aEnd =   m_vPosition.GetOriginRef() 
                   + m_dRadiusAtXAxis*dCosU*m_vPosition.GetXAxisRef() 
                   + m_dRadiusAtYAxis*dSinU*m_vPosition.GetYAxisRef();

  // Matches test in SmEllipse::EvaluateSTEP
  // double dScaledAngZero = SM_EFF_ZERO * 100.0 * (1.0 + 360.0) ;

  // return min seam values for closed ellipses
  if(   SM_IS_ZERO_TO_TOL( aEnd.DistanceBetween(crPointOnCurve), dApproxTol3d) &&
        SM_ARE_SAME_TO_TOL(m_vAnalDomain.GetMax(), rdAngularParameter, SM_EFF_ZERO_DEG)
     && SM_ARE_SAME_TO_TOL(m_vAnalDomain.GetLength(), 360.0, SM_EFF_ZERO_DEG)) 
    { 
      rdAngularParameter = m_vAnalDomain.GetMin() ; 
    }

  // when asked - classify the point
  if(pOptLoc)
    {
      // 1st check point is in angular sweep
      if(!bInArc) { *pOptLoc = SM_CL_EXTERIOR ; }
      else // then check point/curve distance 
        {
          // check distance between Point and curve
          SmPoint3d sEllipsePoint ;
          EvaluateSTEPPoint(rdAngularParameter, sEllipsePoint) ;
          double dDist = sEllipsePoint.DistanceBetween(crPointOnCurve) ;
          double dScaledZero = SM_EFF_ZERO * (1.0 + sEllipsePoint.GetMaxDimension()) ;
          if(dDist > dScaledZero) { *pOptLoc = SM_CL_EXTERIOR ; }
          else                    { *pOptLoc = SM_CL_INTERIOR ; }
        }

      // check for seam values
      if(    SM_ARE_SAME(m_vAnalDomain.GetMin(), rdAngularParameter)  // on seam
         &&  SM_ARE_SAME(m_vAnalDomain.GetLength(), 360.0))           // curve is closed
        { 
          *pOptLoc = SM_CL_SEAM ; 
        }
    } // end need to ClassifyPoint check

  // all done
  return SM_SUCCESS;

} // end SmEllipse::STEPInversion

/*******************************************************************//**
PURPOSE: Virtual Copy a SmEllipse

NOTES: 
***********************************************************************/
SmStatus SmEllipse::Copy
  (const SmContext & crContext,    // in : 
   SmCurve        *& rpNewCurve)   // out: 
  const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  SmEllipse *pCopy = new (crContext) SmEllipse(*this); NER(pCopy) ;
  pCopy->SetFromGwNurb(GetDim(),m_pNurb);
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

} // end SmEllipse::Copy

/*******************************************************************//**
PURPOSE: Method to create a canonical ellipse object.

NOTES: 
***********************************************************************/
SmStatus SmEllipse::CreateCanonical
  (const SmContext        & crContext,    // in : new object context                       
   const SmAxis2Placement & crOrigin,     // in : circle orientation (origin, xAxis, yAxis)
   double                   dRadius1,     // in : dist from origin to ellipse circumference in XAxis direction
   double                   dRadius2,     // in : dist from origin to ellipse circumference in YAxis direction
   SmEllipse             *& rpNewEllipse, // out: new object
   const SmExtent1d       * pInterval,    // opt: Specify interval in degrees [-360 <= min <= max <= 360.0]
                                          //      default:[NULL], NULL = [0.0 360.0]
   SmBoolean              * pInsideOut)   // opt: TRUE: for compatibility with sphere and torus,
                                          //            Nurb curve runs in opposite direction from Analytic curve
                                          //      FALSE: Normal case - nurb and analytic curves are the same shape.
                                          //      default:[FALSE]
{
  const SmExtent1d extent1d (0.0, 360.0);   // must do it this way for Borland

  // make the ellipse
  rpNewEllipse = new (crContext) SmEllipse(crOrigin.GetOriginRef(),
                                           crOrigin.GetXAxisRef(), 
                                           crOrigin.GetYAxisRef(),
                                             pInterval 
                                           ? *pInterval
                                           : extent1d,   //see above
                                           dRadius1,
                                           dRadius2,
                                           3, &crContext, NULL,
                                           pInsideOut ? *pInsideOut : FALSE);
  // all done
  if ( rpNewEllipse->m_pNurb == NULL)
  {
      delete rpNewEllipse; rpNewEllipse = NULL;
      return (SM_ERR);
  }
  return SM_SUCCESS;

} // end SmEllipse::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data.

NOTES: 
***********************************************************************/
SmStatus SmEllipse::GetCanonical
  (SmAxis2Placement & rOrigin,          // out: ellipse center point
                                        //      XAxis = unitVector to ellipse  0 degree endPoint
                                        //      YAxis = unitVector to ellipse 90 degree Point   
   double           & rdRadius1,        // out: radius of  0 degree ellipse Point
   double           & rdRadius2,        // out: radius of 90 degree ellipse Point
   SmExtent1d       * pOptAnalDomain,   // out: deg interval for ellipse
                                        //      NULL to ignore, default:[NULL]
   SmBoolean        * pOptInsideOut)    // out: TRUE = Nurb and Analytic definitions run in opposite directions
                                        //      FALSE= Nurb and Analytic definitions run in same directions
                                        //      NULL to ignore, default:[NULL]       
  const
{
  // get ellipse STEP parameters
  rOrigin   = m_vPosition;
  rdRadius1 = m_dRadiusAtXAxis;
  rdRadius2 = m_dRadiusAtYAxis;
  if(pOptAnalDomain) 
    { pOptAnalDomain->SetMinMax(m_vAnalDomain.GetMin(), m_vAnalDomain.GetMax()) ; }
  if(pOptInsideOut) { *pOptInsideOut = m_bInsideOut ; }

  // all done
  return SM_SUCCESS;

} // end SmEllipse::GetCanonical

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this ellipse.

NOTES: 
***********************************************************************/
SmExtent1d SmEllipse::GetMaxAnalyticDomain() 
 const
{
  // Hmm, the parameterization spans 360 degrees, but the start can slide
  // anywhere from -360 to 0, so our return is not well defined.
  // If our domain already spans 360 degrees, then just return it.
  // If our current start is not negative, return [0,360].
  // Otherwise (spans < 360, start is negative),
  //  if it's all negative (end is negative too), return [-360,0]
  //  otherwise (spans < 360 and contains 0), return [start,start+360].

  SmExtent1d sIvl( m_vAnalDomain );

  if ( sIvl.GetLength() > 360-SM_EFF_ZERO )
    { return sIvl; }

  if ( sIvl.GetMin() > 0-SM_EFF_ZERO ) { sIvl.SetMinMax(  0, 360 ); return sIvl; }

  if ( sIvl.GetMax() < 0+SM_EFF_ZERO ) { sIvl.SetMinMax( -360, 0 ); return sIvl; }

  double dMin = sIvl.GetMin();
  sIvl.SetMinMax( dMin, dMin+360 );

  return sIvl;

} // end SmEllipse::GetMaxAnalyticDomain

/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the ellipse.  This method can be used to find the 
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
SmStatus SmEllipse::GlobalPointSolveSTEP
  (const SmExtent1d      & crInterval,             // NotUsed: in : STEP domain for solutions even when outside curve's domain
   SmSolverOperationType   eSolverOperation,       // in : 
   const SmPoint3d       & crTestPoint,            // in : 
   double                  dDistanceTolerance,     // in : 
   const double          * cpdOptTargetDistance,   // in : 
   const SmVector3d      * cpOptVectors,           // NotUsed: in : 
   SmSolutionRequestedType eSolutionRequested,     // in : 
   SmSolutionArray       & rSolutions)             // out: Step Domain parameters
{
  SM_REF2(crInterval, cpOptVectors) ;
  // check input
  SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE
            || eSolverOperation == SM_SO_MAXIMIZE
            || eSolverOperation == SM_SO_NORMALIZE
            || eSolverOperation == SM_SO_INTERSECT);

  // this breaks the representation - setting StepDomain = NurbDomain
  //      // modify the ellipse interval - is this wise?
  //      SER(AdjustSTEPInterval(crInterval));

  // ellipse locals
  //const SmPoint3d &rCenter = m_vPosition.GetOriginRef();
  SmVector3d       sZAxis  = m_vPosition.GetZAxis();

  // Drop the point using NURBS-based solver
  SmExtent1d sNurbIvl = GetNaturalInterval();
  SER(GlobalPointSolve(sNurbIvl,eSolverOperation,crTestPoint,
                       dDistanceTolerance,cpdOptTargetDistance,
                       NULL,eSolutionRequested,rSolutions));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
if(bDebugMe)
  {
    rSolutions.Dump() ;
  }
#endif // SM_DEBUG_CODE

  // Convert NURBS solutions to STEP solutions
  for (ULONG i=0; i<rSolutions.GetSize(); i++) 
    {
      SmSolution & rSol = rSolutions[i];
      SmPoint3d sPointOnCurve;
      SER(EvaluatePoint(rSol.m_vStart[0],sPointOnCurve));

      // First take care of start and end equivalents
      if (smos_Fabs(sNurbIvl.GetMin()-rSol.m_vStart[0]) < SM_EFF_ZERO) 
        {
          rSol.m_vStart[0] =   m_bInsideOut  
                             ? m_vAnalDomain.GetMax()
                             : m_vAnalDomain.GetMin() ;
          continue;
        }
      if (smos_Fabs(sNurbIvl.GetMax()-rSol.m_vStart[0]) < SM_EFF_ZERO) 
        {
          rSol.m_vStart[0] =   m_bInsideOut  
                             ? m_vAnalDomain.GetMin()
                             : m_vAnalDomain.GetMax() ;
          continue;
        }

      double dAngleParam;
      SER(STEPInversion(sPointOnCurve,dAngleParam));
      rSol.m_vStart[0] = dAngleParam;

      
      //      // convert from NurbParam to StepParam
      //      SmSolution & rSol = rSolutions[i];
      //      double dStepParam, dNurbParam = rSol.m_vStart[0] ;
      //      ConvertTFromNURBSToSTEP(dNurbParam, dStepParam) ;
      //      
      //      // save the StepParam
      //      rSol.m_vStart[0] = dStepParam;

    }
  
  // all done
  return SM_SUCCESS;

} // end SmEllipse::GlobalPointSolveSTEP

/*******************************************************************//**
PURPOSE: Rotate the given point about the Z axis until it is
  on the Z, X plane.  

NOTES:  yuk: SmSurfOfRevolution::TransformPointToStartPlane() and
             SmEllipse         ::TransformPointToStartPlane() 
   should be the same function.  If you modify one, make sure to modify the other.
***********************************************************************/
SmStatus SmEllipse::TransformPointToStartPlane
  (const SmPoint3d & crPointToTransform, // in : target point
   double            dDistTol3d,         // in : Dist3d when points are close enough to seams to return 2 answers
   SmPoint3d       & rTransformedPoint,  // out: output point on the X/Z plane       
   ULONG           & rlNumAngles,        // out: 0 - point is on axis
                                         //      1 - point is not on seam 
                                         //      2 - point is on seam of curve of revolution
   double            adAnglesDeg[2],     // out: Angles in degrees used to transform the point
                                         //      on the plane to the original point.  
                                         //      range:[m_vAnalDomain] or positive(0 to 360.0)
   SmBoolean       & bInside,            // out: TRUE = point is inside trim domain
                                         //      FALSE= point is outside trim domain
   SmBoolean         bSnapToSeams)       // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams
                                         //      TRUE=previous behavior, default:[FALSE] 
 const
{
  double dDistToZAxis ;
  // pass the call along
  SmStatus sRtn = smgu_TransformPointToStartPlane
     (crPointToTransform,  // in : target point
      m_vPosition,         // in : rotate about Origin and ZAxis, measure angles from XAxis
      m_vAnalDomain,       // in : Interval of supported points (commonly [0 360], think SmEllipse and SmSurfOfRevolution AnalDomains)
      dDistTol3d,          // in : Dist3d when points are close enough to seams to return 2 answers
      rTransformedPoint,   // out: output point on the X/Z plane 
      dDistToZAxis,        // out: distance to Z axis       
      rlNumAngles,         // out: 1 - point is not on seam 
                           //      2 - point is on seam of periodic interval or on the z axis
      adAnglesDeg,         // out: Angles in degrees used to transform the point
                           //      on the plane to the original point.  
                           //      range:[m_vAnalDomain] or positive(0 to 360.0)
      bInside,             // out: TRUE = point is inside trim domain
                           //      FALSE= point is outside trim domain
      bSnapToSeams) ;      // in : TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams
                           //      TRUE=previous behavior, FALSE=NewBehavior
   
  // low work - point is on the axis to within tolerance - e.g. an ambiguous center point
  double dScaledZero = SM_EFF_ZERO * (1.0 + crPointToTransform.GetMaxDimension());
  if (dDistToZAxis < dScaledZero) 
    { // Point is on the axis of revolution - no unique solutions 
      rlNumAngles = 0;
    } // end point on Z_Axis check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump() ;  
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmEdge *pEdge = (SmEdge *)GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;
      
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; crPointToTransform.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 0,1,0) ; rTransformedPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,1,1,0) ; (5.0*m_vPosition.GetZAxis()).Draw(&m_vPosition.GetOriginRef()) ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,0,1,1) ; (5.0*m_vPosition.GetXAxis()).Draw(&m_vPosition.GetOriginRef()) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // all done
  return sRtn ;

} // end SmEllipse::TransformPointToStartPlane

/*******************************************************************//**
PURPOSE: Drop a point to an ellipse segment or its endPoints very fast.

NOTES: Only supports SM_SO_MINIMIZE and SM_SO_INTERSECT
   SM_SO_MINIMIZE:  find closest point (more than one for closed curves)
                      to curve or if cpdOptTargetDistance is given
                      point within pdOptTargetDistance + dDistanceTolerance.
   SM_SO_INTERSECT: find closest point within dDistanceTolerance.
   
   For points within curve segment - drop vector is perp to curve
   For points beyond curve segment - drop vector is to nearest curve endPoint 

RETURNS --- 
  SM_SUCCESS when operation is SM_SO_MINIMIZE or SM_SO_INTERSECT a
             and curve is a circle or circular arc, else
  SM_ERR     where GeneralPointSolve() will have to be called to get a solution.

***********************************************************************/
SmStatus SmEllipse::DropPointFast                  // rtn: SM_SUCCESS=case supported, SM_ERR=call GeneralPointSolve() to get solution
 (const SmExtent1d      & crInterval,              // in : Nurb Domain of curve to search for solutions
  SmSolverOperationType   eSolverOperation,        // in : oneof: SM_SO_MINIMIZE =find closest point (more than one for closed curves)
                                                   //             to curve or if cpdOptTargetDistance is given
                                                   //             point within pdOptTargetDistance + dDistanceTolerance. 
                                                   //             SM_SO_INTERSECT=find closest point within dDistanceTolerance.
  const SmPoint3d       & crTestPoint,             // in : target point
  const SmVector3d      * cpOptInPointingVector,   // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams
  double                  dDistanceTolerance,      // in : Skip Solutions whose drop distance is too far away
                                                   //      operation == MINIMIZE save solution if cpdOptTargetDistance == NULL
                                                   //                            or DropDist < cpdOptTargetDistance + dDistanceTolerance
                                                   //      operation == INTERSECT save solution if DropDist < dDistanceTolerance
  const double          * cpdOptTargetDistance,    // in : only used for operation Minimize.  When given
                                                   //      skip solutions whose dropDist > cpdOptTargetDistance + dDistanceTolerance.
                                                   //      else keep all solutions. 
  SmSolutionRequestedType eSolutionRequested,      // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions 
  SmSolutionArray       & rSolutions)              // out: array of problem solutions reported as Curve parameter values 
 const        
{
#if 0
  return SM_ERR; // Don't do dropping on ellipse - sometimes produces errors.
#else
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmEdge *pEdge = (SmEdge *)GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      // draw point, ellipse origin, ellipse
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(3,4, 1,0,0); crTestPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); m_vPosition.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,0,1); Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // init outputs
  rSolutions.ReSet();

  // no work - no  
  if (!m_vPolarConverter.IsPolarConversionPossible()) 
    { return SM_ERR; } /* end Draw */

  // no work - not a circle or circular arc
  if (!IsCircle()) 
    { return SM_ERR; }

  // no work - operation not minimize or intersect
  if (   eSolverOperation != SM_SO_MINIMIZE 
      && eSolverOperation != SM_SO_INTERSECT) 
    { return SM_ERR; }

  // locals
  ULONG     lNumAngles;
  double    adAnglesDeg[2];
  SmPoint3d sPointInPlane;
  SmBoolean bInside ;

  // get angle of TestPoint in ellipse coordinate system as a side effect of
  // rotating TestPoint about ellipse's Z-Axis to the X/Z Plane
  SER(TransformPointToStartPlane(crTestPoint,        // in
                                 dDistanceTolerance, // in : Dist3d when points are close enough to seams to return 2 answers
                                 sPointInPlane,      // out: rotated point
                                 lNumAngles,         // out: 0 - point is on axis
                                                     //      1 - point is not on seam
                                                     //      2 - point is on seam of surface of revolution
                                 adAnglesDeg,        // out: angles ranging from 0.0 to 360.0
                                 bInside));          // out: TRUE = point is inside trim domain
                                                     //      FALSE= point is outside trim domain
  
  // lNumAngles == 2 TestPoint is on seam of closed ellipse  
  // and we are given an cpOptInPointingVector                             
  if (lNumAngles == 2)
    {
      // when Optional Interior Pointing Vector is given - use it to select desired solution
      if(cpOptInPointingVector) 
        {
          double dDot = cpOptInPointingVector->Dot(m_vPosition.GetYAxisRef());
          if (dDot > SM_EFF_ZERO_SQRT) 
            { // Keep start parameter of periodic surface
              lNumAngles = 1;
            }
          else if (dDot < -SM_EFF_ZERO_SQRT) 
            { // Keep end parameter of periodic surface
              lNumAngles = 1;
              adAnglesDeg[0] = adAnglesDeg[1];
            }
        } // end given OptInteriorPointingVector branch
    } // end 2 angles were found check

  // Check for case where point is on Z_AXIS of the ellipse
  if (lNumAngles == 0) 
    {
      adAnglesDeg[0] = GetStartAngleDeg();
      adAnglesDeg[1] = GetEndAngleDeg();
      lNumAngles = 2;
    }

  // initialize solution object
  SmSolution sSol;
  sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
  sSol.m_lNumObjects   = 1;
  sSol.m_apObjects[0]  = SM_CONST_CAST(SmEllipse*, this);
  sSol.m_apNodes[0]    = NULL;
  sSol.m_lNumVariables = 1;

  // Parameter-space tolerance for inside-domain checks:
  // param tol = 3d tol / ds/dt.  ds = 2*PI*Rad, dt = 360; cancel out the 2.
  //   double dParamTol = dDistanceTolerance / ( m_dRadiusAtXAxis * SM_PI / 180.0 );
  // But in practice that causes problems, because ConvertToNURBSParameter()
  // checks vs. SM_EFF_ZERO_DEG.

  double dParamTol = SM_EFF_ZERO_DEG;

  // In case of two solutions, if one is better, use it: compare solution values.  [220629]
  double dDists[2];

  // For every angle, convert the angle to curve parameter and set up the Solutions.
  for (ULONG ii=0; ii<lNumAngles; ii++) 
    {
      // locals
      double dTParameter;
      SmBoolean bExactConversion;
      SmPoint3d sPnt;

      // clamp angle to full curve analytic extent - so convert angle will work
      double dClampedAngle = adAnglesDeg[ii];
      // Do not do periodic clamp if two angles were found: if they're not exactly
      // on the seam, it will clamp them both to the same value.  [B90]
      if ( lNumAngles == 1 )
        { dClampedAngle = m_vAnalDomain.ClampPeriodicValue( adAnglesDeg[ii], 360.0, dParamTol ); }
      else
        { dClampedAngle = m_vAnalDomain.ClampValue( adAnglesDeg[ii] ); }

      // convert angle to curve parameter value
      SER(m_vPolarConverter.ConvertToNURBSParameter(dClampedAngle,dTParameter,bExactConversion, dDistanceTolerance));

      // clamp parameter to given Nurb interval - which may be a subset of the full curve interval
      SmExtent1d sNurbIvl = GetNaturalInterval() ;
      SM_ASSERT(sNurbIvl.ContainsValue(dTParameter)) ;
      dTParameter = crInterval.ClampValue(dTParameter);

      // when the conversion is not exact
      if (!bExactConversion) 
        {
          SER(EvaluateSTEP(dClampedAngle,0,TRUE,&sPnt));
          SmSolution sSol3;
          SmBoolean bFoundAnswer;

          // refine parameter value with NewtonRaphson iteration
          SER(this->LocalPropertyAnalysis(crInterval,SM_CP_POINT_INVERSION,
              dTParameter,NULL,&sPnt,bFoundAnswer,sSol3));
          if (bFoundAnswer) {
              dTParameter = sSol3.m_vStart[0];
          }
        }

      // reclamp point and get 3DPoint
      SM_ASSERT(crInterval.ContainsValue(dTParameter)) ;
      dTParameter = crInterval.ClampValue(dTParameter);
      SER(EvaluatePoint(dTParameter,sPnt));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2) 
        {
          smgfx_SetColor(0,0,0);
          sPnt.Draw();
          sm_GraphicsLoop();
          smgfx_SetColor(1,0,0);
          crTestPoint.Draw();
          sm_GraphicsLoop();
          m_vPosition.Draw();
          sm_GraphicsLoop();
          smgfx_SetColor(0,0,1);
          Draw();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // get testPoint/ProjectedPoint gap
      dDists[ii] = sPnt.DistanceBetween(crTestPoint);

      // skip points too far away
      if (   (   eSolverOperation == SM_SO_MINIMIZE
              && cpdOptTargetDistance 
              && dDists[ii] > *cpdOptTargetDistance + dDistanceTolerance) 
          || (   eSolverOperation == SM_SO_INTERSECT
              && dDists[ii] > dDistanceTolerance))
        {
          continue;
        }

      // save the solution
      sSol.m_vStart.m_dSolutionValue = dDists[ii];
      sSol.m_vStart[0]               = dTParameter;
      rSolutions.Add(sSol);

    } // end iter every angle

  // Chose the value to use when only one solution is requested
  if (eSolutionRequested == SM_SR_SINGLE) 
    {
      if (rSolutions.GetSize() > 1) 
        {
          // Note: if the two values are equal, use the first.
          // This preserves the old behavior, and otherwise causes several Fillet failures.
          if ( dDists[1] < dDists[0] )
            { rSolutions.RemoveAt(0); }
          else
            { rSolutions.SetSize(1); }
        }
    }

  return SM_SUCCESS;
#endif 

} // end SmEllipse::DropPointFast

/*******************************************************************//**
PURPOSE: Given a STEP-based parametric value of the curve determine
    the corresponding Euclidian point. 

NOTES: The step parameter is an angle in degrees
    which is clamped to the analytic domain.
***********************************************************************/
SmStatus SmEllipse::EvaluateSTEPPoint
(
    double      dSTEPParameter,       // in : ellipse angle in degrees (near to but not the CCW angle from X)     
    SmPoint3d & rPoint,               // out: Euclidian point
    SmZoneTol3d dZoneTol3d            // in : Tolerance for error checking snap when Param ~ Max
)
const
{
    SmVector3d aPoint[1];
    SER( EvaluateSTEP( dSTEPParameter, 0, FALSE, aPoint, dZoneTol3d ) );
    rPoint = aPoint[0];

    return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Given a STEP-based parametric value of the curve determine
    the corresponding Euclidian point and optionally derivatives.

NOTES: The step parameter is an angle in degrees
    which is clamped to the analytic domain.
    lNumDerivatives = 0 - produces Euclidian point only
                      1 - produces first derivative and point
***********************************************************************/
SmStatus SmEllipse::EvaluateSTEP
  (double dSTEPParameter,             // in : ellipse angle in degrees (near to but not the CCW angle from X)     
   ULONG lNumDerivatives,             // in : 0 = Euclidian Point only
                                      //      1 = Euclidian Point and 1st derivative
   SmBoolean bFromLeftbFromLeft,      // NotUsed: in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d aPointAndDerivatives[], // out: values 
   SmZoneTol3d dZoneTol3d)            // in : Tolerance for error checking snap when Param is near STEP domain Max
  const
{
  SM_REF1(bFromLeftbFromLeft) ;
  // check inputs
  if (lNumDerivatives > 3) SER(SM_ERR);
  if (!m_vAnalDomain.ContainsPeriodicValue(dSTEPParameter,360.0,&dSTEPParameter)) 
    {
      dSTEPParameter = m_vAnalDomain.ClampPeriodicValue(dSTEPParameter,360.0);
    }

  // get input in radians
  double dAngleRad = dSTEPParameter * SM_PI / 180.0;
  double dCosU     = smos_Cosine(dAngleRad);
  double dSinU     = smos_Sine(dAngleRad);

  // eval position
  aPointAndDerivatives[0] =   m_vPosition.GetOriginRef() 
                            + m_dRadiusAtXAxis*dCosU*m_vPosition.GetXAxisRef() 
                            + m_dRadiusAtYAxis*dSinU*m_vPosition.GetYAxisRef();

  if (lNumDerivatives > 0) 
    {
      SmVector3d sUnScaled    = - m_dRadiusAtXAxis*dSinU*m_vPosition.GetXAxisRef()   
                                + m_dRadiusAtYAxis*dCosU*m_vPosition.GetYAxisRef() ;
      aPointAndDerivatives[1] = sUnScaled * SM_PI / 180.0;
    }

  if (lNumDerivatives > 1) 
    {
      SmVector3d sUnScaled    = - m_dRadiusAtXAxis*dCosU*m_vPosition.GetXAxisRef()   
                                - m_dRadiusAtYAxis*dSinU*m_vPosition.GetYAxisRef() ;
      aPointAndDerivatives[2] = sUnScaled * SM_PI / 180.0 * SM_PI / 180.0;
    }

  if (lNumDerivatives > 2) 
    {
      SmVector3d sUnScaled    =   m_dRadiusAtXAxis*dSinU*m_vPosition.GetXAxisRef()   
                                - m_dRadiusAtYAxis*dCosU*m_vPosition.GetYAxisRef() ;
      aPointAndDerivatives[3] = sUnScaled * SM_PI / 180.0 * SM_PI / 180.0 * SM_PI / 180.0 ;
    }

#ifdef SM_DEBUG_CODE
  double dAngle = 0.0;
  STEPInversion(aPointAndDerivatives[0], dAngle, NULL, dZoneTol3d);
  SM_ASSERT(SM_ARE_SAME_TO_TOL(dAngle, dSTEPParameter, SM_EFF_ZERO_DEG) || SM_ARE_SAME_TO_TOL(dAngle + 360.0, dSTEPParameter, SM_EFF_ZERO_DEG));
  if (!SM_ARE_SAME_TO_TOL(dAngle, dSTEPParameter, SM_EFF_ZERO_DEG) && !SM_ARE_SAME_TO_TOL(dAngle + 360.0, dSTEPParameter, SM_EFF_ZERO_DEG))
  {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("SM_ARE_SAME_TO_TOL Comparing %16.16lf and %16.16lf within Tol = %16.16lf\n"), dAngle, dSTEPParameter, SM_EFF_ZERO_DEG);
      smos_WriteBuffer(sBuff);
      smos_sprintf(sBuff, _T("SM_ARE_SAME_TO_TOL Comparing %16.16lf and %16.16lf within Tol = %16.16lf\n"), dAngle + 360.0, dSTEPParameter, SM_EFF_ZERO_DEG);
      smos_WriteBuffer(sBuff);
  }
#else
    SM_REF1(dZoneTol3d);
#endif

  return SM_SUCCESS;

} // end SmEllipse::EvaluateSTEP

/*******************************************************************//**
PURPOSE: Intersect a two Ellipses using geometric techniques.

NOTES: In some cases it will defer to the numerical intersector.
   Right now this method only works with two circles.
***********************************************************************/
SmStatus SmEllipse::IntersectWithEllipse
  (const SmExtent1d & crInterval,          // in : line interval                                       
   const SmEllipse & crOtherCurve,         // in : other curve to intersect                            
   const SmExtent1d & crOtherInterval,     // in : other curve interval                                
   double dDistanceTolerance,              // in : Find points where curves are within this 3D distance 
   SmBoolean & rbNeedsMoreIntersections,   // out: TRUE = pass call to general curve/curve intersector 
                                           //      FALSE= intersections found here                     
   SmSolutionArray & rSolutions)           // out: solutions                                           
 const
{
  // init output
  rSolutions.ReSet();
  rbNeedsMoreIntersections = TRUE;

  // Right now this method only works with two circles.
  // In the future we should be able to do ellipses because
  // they are fairly common and easy enough to implement.
  if (   !IsCircle()
      || !crOtherCurve.IsCircle()) 
    { return SM_SUCCESS; }

  // no work - the same curve does not self intersect
  if (this == &crOtherCurve) 
    {
      rbNeedsMoreIntersections = FALSE;
      return SM_SUCCESS;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
  if (bDebugMe0) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); this->       Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); crOtherCurve.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // locals
  SmVector3d       sThisNormal = this->m_vPosition.GetZAxis();
  const SmPoint3d &rThisCenter = this->m_vPosition.GetOriginRef();

  SmVector3d       sOtherNormal = crOtherCurve.m_vPosition.GetZAxis();
  const SmPoint3d &rOtherCenter = crOtherCurve.m_vPosition.GetOriginRef();

  ULONG lNumInt;
  SmPoint3d aIntersectionPoints[4];
  SmBoolean bMayNeedRefinement = FALSE;


  // Check for coplanar circles.
  // Use the sine of the angle, not the cosine, because checking for
  // the cosine being close to 1.0 is full of noise error.  (The cosine
  // function just grazes 1.0.)
  SmVector3d sCross = sThisNormal * sOtherNormal;
  double dSineAng = sCross.Length();

  // For the 'exactly' parallel test: don't test against SM_EFF_ZERO,
  // because that will be sensitive to noise that is in the data
  // more often than not.
  // Previously, when we used the cosine and checked for it being close
  // to 1.0, that noise gets washed out by dot product calculation,
  // and the cosine ends up at 1.0 exactly.  But the sine test will catch
  // it all, and we'll end up rejecting lots of cases that we shouldn't.
  // Checking (1 - cosine) against SM_EFF_ZERO is similar to checking
  // the sine against SM_EFF_ZERO_SQRT: sine = sqrt( 1 - cosine^2 ).
  // (Note this is not the case in IntersectWithLine(): there the
  // check is at 90 degrees, so the cosine is appropriate.)

  SmBoolean bExactlyParallel = ( dSineAng <= SM_EFF_ZERO_SQRT );  // about 0.0000573 degrees
  SmBoolean bNearlyParallel  = ( dSineAng <  SM_SINE_10DEG );

  if ( ! bExactlyParallel )
    {
      if ( bNearlyParallel )
        {
          return SM_SUCCESS; // Don't use analytical intersection because
                             // the angle is too small between the planes.
        }

      // If two ellipses in intersecting planes intersect, then all (both)
      // intersections must be in the both planes, which is to say, on the
      // line of intersection of the two planes.  Get that line and see if
      // it intersects this ellipse.  If not, there's no intersection.
      // If so, use the intersection points as guess points to check
      // and refine if necessary.

      SmPoint3d sLinePnt;
      SmVector3d sLineVec;
      SER( smgu_IntersectTwoPlanes( rThisCenter, sThisNormal, rOtherCenter, sOtherNormal,
          sLinePnt, sLineVec ));

      // intersect circle with line of intersections
      double aLineParams[4];
      SER( smgu_LineCoPlanarCircleIntersect( sLinePnt, sLineVec, rThisCenter, sThisNormal,
                                             GetXRadius(), dDistanceTolerance * 10.0,
                                             lNumInt, aLineParams ));
      // no intersections
      if ( lNumInt == 0 )
        {
          rbNeedsMoreIntersections = FALSE;
          return SM_SUCCESS;
        }

      // Use each line intersection as a guess point for iteration.
      ULONG ii;
      for ( ii=0; ii<lNumInt; ii++ )
        {
          // get the 3d intersection point
          aIntersectionPoints[ii] = sLinePnt + aLineParams[ii]*sLineVec;
        }
      bMayNeedRefinement = TRUE;

    } // end circles on different planes branch

  else
    {
      // Circles are on the same or parallel planes

      // Check whether it's the same plane.  Project OtherCenter to ThisPlane.
      SmPoint3d sProjPnt;
      SER(smgu_PointProjectToPlane(rOtherCenter,rThisCenter,sThisNormal,sProjPnt));
      double dDistFromPlane = sProjPnt.DistanceBetween(rOtherCenter);

      // no work - no intersections
      if (dDistFromPlane > dDistanceTolerance) 
        {
          rbNeedsMoreIntersections = FALSE;
          return SM_SUCCESS;
        }


      // intersect two coplanar circles
      SER( smgu_CircleCoPlanarCircleIntersect( sThisNormal, rThisCenter, GetXRadius(),
                                               rOtherCenter, crOtherCurve.GetXRadius(), dDistanceTolerance,
                                               lNumInt, aIntersectionPoints ));
      
      // no intersections - all done
      if (lNumInt == 0) 
        {
          rbNeedsMoreIntersections = FALSE;
          return SM_SUCCESS;
        }

      // check for coincident circles
      if (lNumInt > 2) 
        {
          SmBoolean bNeedsMoreWork;
#ifdef SM_TOLERANT_XSECT
          SER(GlobalCoincidenceChecker(crInterval,crOtherCurve,crOtherInterval,dDistanceTolerance,
                                       NULL, NULL, bNeedsMoreWork,rSolutions, FALSE));
#else  // NO SM_TOLERANT_XSECT
          SER(GlobalCoincidenceChecker(crInterval,crOtherCurve,crOtherInterval,dDistanceTolerance,
                                       bNeedsMoreWork,rSolutions, FALSE));
#endif // NO SM_TOLERANT_XSECT
          rbNeedsMoreIntersections = FALSE;
          //if (bNeedsMoreWork) {
          //    return SM_SUCCESS;
          //}
          // This is that coincident possibly periodic case that is a pain 
          // to take care of.  We should look into some of the revolution
          // intersection stuff for how to solve it some day.  
          return SM_SUCCESS;
        }
    }

  // State at this point:
  //  Both ellipses are circles.
  //  If the circles are coplanar, then there are either one or two intersections,
  //   and they are precise (to tolerance).
  //  If they are not coplanar, then the planes are not close to coplanar (to 10 degrees).
  //   and we have one or two guess points for the intersection.
  // Either way (precise or not), the solution points are 3d positions.

  // Next: for each one of the 3d solutions, drop the point to each curve.
  // Check the distance between the dropped solutions on the curve.

  // Drop point onto each curve to get the intersection point.
  SmSolution sSData1[4];
  SmSolutionArray sSolutions1(4,sSData1);
  SmSolution sSData2[4];
  SmSolutionArray sSolutions2(4,sSData2);

  SmSolution sSol;
  sSol.m_lNumObjects   = 2;
  sSol.m_apObjects[0]  = SM_CONST_CAST(SmEllipse*,this);
  sSol.m_apObjects[1]  = SM_CONST_CAST(SmEllipse*,&crOtherCurve);
  sSol.m_lNumVariables = 2;
  sSol.m_eSolutionType = SM_ST_SINGLE_VALUE; 

  ULONG ii, jj, kk;
  for (ii=0; ii<lNumInt; ii++)
    {
      SER( this->GlobalPointSolve( crInterval, SM_SO_INTERSECT, aIntersectionPoints[ii],
          dDistanceTolerance*10.0, NULL, NULL, SM_SR_ALL, sSolutions1 ));

      if (sSolutions1.GetSize() == 0)
        { continue; }

      SER( crOtherCurve.GlobalPointSolve( crOtherInterval, SM_SO_INTERSECT, aIntersectionPoints[ii],
          dDistanceTolerance*10.0, NULL, NULL, SM_SR_ALL, sSolutions2 ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) {
          smgfx_Erase();
          smgfx_SetLook(2,4, 1,0,0); aIntersectionPoints[ii].Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); this->Draw();        sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); crOtherCurve.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();

      }
#endif

      // Now check distances between the 3d points dropped to the two curves.
      // If refinement is needed (the non-coplanar case), then if they are
      // close enough, call the local curve/curve intersector to see about
      // a true intersection.
      // After that possible refinement, if close enough, fill in the
      // SmSolution sSol and add it to the output.

      // Check every combination, if we get more than one drop solution.
      // (Can't really have more than one solution when dropping to a circle,
      // but it won't hurt to leave this extra logic in; could come into play
      // if we do general ellipses.)

      SmPoint3d sP1, sP2;
      double dDist;

      for (jj=0; jj<sSolutions1.GetSize(); jj++)
        {
          for (kk=0; kk<sSolutions2.GetSize(); kk++)
            {
              sSol.m_vStart[0] = sSolutions1[jj].m_vStart[0];
              sSol.m_vStart[1] = sSolutions2[kk].m_vStart[0];

              SER( this->       EvaluatePoint( sSol.m_vStart[0], sP1 ));
              SER( crOtherCurve.EvaluatePoint( sSol.m_vStart[1], sP2 ));

              dDist = sP1.DistanceBetween(sP2);

              double dScaledZero = SM_EFF_ZERO * (1.0 + sP1.GetMaxDimension() );

              // Do refinement in any case.  In the tangent case, there can be quite a bit
              // of noise, enough to result in incorrect classifications.  [Reg_091216_s...]
              // And if the current solution is good, then refinement doesn't cost much.
              bMayNeedRefinement = TRUE;

              if ( bMayNeedRefinement )
                {
                  if (    dDist < dDistanceTolerance * 100.0 // close enough to be a contender
                       && dDist > dDistanceTolerance / 100.0 // far enough to need refinement
                     )
                    {
                      SmSolution sSol2;
                      SmBoolean bFoundAnswer;
                      SER( this->LocalCurveSolve( crInterval,
                                       crOtherCurve, crOtherInterval,
                                       SM_SO_MINIMIZE, dDistanceTolerance,
                                       &dDist, NULL,
                                       sSol.m_vStart[0],
                                       sSol.m_vStart[1],
                                       bFoundAnswer,
                                       sSol2 ));

                      // If the result of the refinement is better than the original point-drops,
                      // use it regardless of the output flag from LoaclCurveSolve():
                      // often the closest approach is in the output, even if it's not
                      // quite within tol.
                      // But do check that the returned parameters are within range.
                      // If they are, then it can't hurt anything to check how good they are.

                      //if (  bFoundAnswer
                      //     && sSol2.m_vStart.m_dSolutionValue <= dDistanceTolerance + dScaledZero )

                      double dT1 = sSol2.m_vStart[0];
                      double dT2 = sSol2.m_vStart[1];
                      if (   crInterval.     ContainsValue( dT1, SM_EFF_ZERO )
                          && crOtherInterval.ContainsValue( dT2, SM_EFF_ZERO ) )
                        {
                          SER( this->       EvaluatePoint( dT1, sP1 ));
                          SER( crOtherCurve.EvaluatePoint( dT2, sP2 ));

                          double dDist2 = sP1.DistanceBetween( sP2 );

                          if ( dDist2 < dDist )
                            {
                              dDist = dDist2;
                              sSol.m_vStart[0] = sSol2.m_vStart[0];
                              sSol.m_vStart[1] = sSol2.m_vStart[1];
                            }
                        }
                    }
                } // end if may need refinement


              // All refined if necessary; see about using this result.
              if ( dDist <= dDistanceTolerance + dScaledZero )
                {
                  sSol.m_vStart.m_dSolutionValue = dDist;
                  rSolutions.Add(sSol);
                }

            } // end inner for loop (kk) on second curve solutions
        } // end outer for loop (jj) on first curve solutions
    } // end loop (ii )on all intersections (one or two)

  rbNeedsMoreIntersections = FALSE;
      
  return SM_SUCCESS;

} // end SmEllipse::IntersectWithEllipse

/*******************************************************************//**
PURPOSE: Intersect this Ellipse with a line.

NOTES:  sSol.m_vStart[0] = ellipse param
        sSol.m_vStart[1] = line param

   Right now this method works only with circles.
***********************************************************************/
SmStatus SmEllipse::IntersectWithLine
  (const SmExtent1d & crInterval,         // in : ellipse interval in Nurb Domain
   const SmLine     & crLine,             // in : line to intersect                             
   const SmExtent1d & crOtherInterval,    // in : other line interval in Nurb Domain
   double dDistanceTolerance,             // in : Find points where curves are within this 3D distance
   SmBoolean & rbNeedsMoreIntersections,  // out: TRUE = pass call to general curve/curve intersector
                                          //      FALSE= intersections found here
   SmSolutionArray & rSolutions)          // out: solutions:  sSol.m_vStart[0] = ellipse param
                                          //                  sSol.m_vStart[1] = line param   
 const
{
  // init output
  rSolutions.ReSet();
  rbNeedsMoreIntersections = TRUE;

  // Right now this method works only with circles.
  if ( !IsCircle() )
    { return SM_SUCCESS; }

  SmVector3d       sThisNormal = this->m_vPosition.GetZAxis();
  const SmPoint3d &rThisCenter = this->m_vPosition.GetOriginRef();

  SmPoint3d  sStart   = crLine.GetLinePoint();
  SmVector3d sLineVec = crLine.GetLineVector() * crLine.GetLineScale();
  SmPoint3d  sEnd     = sStart + sLineVec;

  ULONG lNumInt;
  SmPoint3d aIntersectionPoints[4];
  SmBoolean bMayNeedRefinement = FALSE;

  // Check to see if circles and line are somewhat on the same plane.
  // If within 10 degrees of being on the same plane use 'coplanar' intersection.
  // (Note, we're checking dDot (which is cosine) against a sine value.
  // That works out because we're checking against 90 degrees, so checking
  // the cosine at 90 degrees is the same as checking the sine at 0.)

  double dTol = SM_EFF_ZERO * (1.0 + sStart.GetMaxDimension());
  double dDot = 0.0;
  if (sLineVec.LengthSquared() < dTol*dTol) 
    {
      dDot = SM_BIG_DOUBLE;
    }
  else 
    {
      SmVector3d sTmpLV = sLineVec;
      SER(sTmpLV.Unitize());
      dDot = sThisNormal.Dot(sTmpLV);
    }

  if (smos_Fabs(dDot) > SM_SINE_10DEG)
    {
      // Line is not within 10 degrees of this ellipse's plane.
      // Only one intersection possible.
      // If there is an intersection, it would be where the line intersects
      // the plane of the ellipse.  Use that point as a guess.
      double dLineParam;
      SER( smgu_LinePlaneIntersect( sStart, sLineVec, rThisCenter, sThisNormal, dLineParam ));
      lNumInt = 1;
      aIntersectionPoints[0] = sStart + dLineParam * sLineVec;
      bMayNeedRefinement = TRUE;
    }
  else 
    {
      if (smos_Fabs(dDot) > SM_EFF_ZERO_SQRT) 
        {
          bMayNeedRefinement = TRUE; // not exactly in plane.
        }

      // Project the line to the plane of the ellipse.
      SmPoint3d  sLinePlanePnt;
      SmVector3d sLinePlaneVec;
      SER( smgu_PointProjectToPlane(  sStart,   rThisCenter, sThisNormal, sLinePlanePnt ));
      SER( smgu_VectorProjectToPlane( sLineVec, sThisNormal, sLinePlaneVec ));
      
      if (sLinePlaneVec.LengthSquared() < SM_EFF_ZERO_SQ) 
        {
          rbNeedsMoreIntersections = TRUE; // Line is perp to plane.
          return SM_SUCCESS;
        }

      // Intersect the projected line with the ellipse.
      double aLineParams[4];
      SER(smgu_LineCoPlanarCircleIntersect( sLinePlanePnt, sLinePlaneVec,
                                            rThisCenter,   sThisNormal,
                                            GetXRadius(),  dDistanceTolerance * 10.0,
                                            lNumInt, aLineParams));
      if (lNumInt == 0) 
        {
          rbNeedsMoreIntersections = FALSE;
          return SM_SUCCESS;
        }
      for (ULONG m=0; m<lNumInt; m++) 
        {
          aIntersectionPoints[m] = sLinePlanePnt + aLineParams[m]*sLinePlaneVec;
        }
    }

  // State at this point:
  //  The ellipse is a circle.
  //  If the line is parallel to the ellipse's plane, then there are either
  //   one or two intersections, and they are precise (to tolerance).
  //  If they are not coplanar, then we have one or two guess points for the intersection.
  // Either way (precise or not), the solution points are 3d positions.

  // Next: for each one of the 3d solutions, drop the point to each curve.
  // Check the distance between the dropped solutions on the curve.


  // Drop point onto each curve to get the intersection point.
  SmSolution sSData1[4];
  SmSolution sSData2[4];
  SmSolutionArray sSolutions1(4,sSData1);
  SmSolutionArray sSolutions2(4,sSData2);

  SmSolution sSol;
  sSol.m_lNumObjects   = 2;
  sSol.m_apObjects[0]  = SM_CONST_CAST(SmEllipse*,this);
  sSol.m_apObjects[1]  = SM_CONST_CAST(SmLine*,&crLine);
  sSol.m_lNumVariables = 2;
  sSol.m_eSolutionType = SM_ST_SINGLE_VALUE; 

  ULONG ii, jj, kk;
  for (ii=0; ii<lNumInt; ii++) 
    {
      if ( crLine.GlobalPointSolve( crOtherInterval, SM_SO_INTERSECT,
                                    aIntersectionPoints[ii],
                                    dDistanceTolerance*10.0,
                                    NULL, NULL, SM_SR_ALL,
                                    sSolutions2 )
            != SM_SUCCESS )
        {
          return SM_SUCCESS;
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook(2,4, 1,0,0); aIntersectionPoints[ii].Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); this-> Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); crLine.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();

        }
#endif

      if (sSolutions2.GetSize() == 0)
        { continue; }

      if ( this->GlobalPointSolve( crInterval, SM_SO_INTERSECT,
                                   aIntersectionPoints[ii],
                                   dDistanceTolerance*10.0,
                                   NULL, NULL, SM_SR_ALL,
                                   sSolutions1 )
            != SM_SUCCESS) 
        {
          return SM_SUCCESS;
        }

      // Now check distances between the 3d points dropped to the two curves.
      // If refinement is needed (the non-coplanar case), then if they are
      // close enough, call the local curve/curve intersector to see about
      // a true intersection.
      // After that possible refinement, if close enough, fill in the
      // SmSolution sSol and add it to the output.

      // Check every combination, if we get more than one drop solution.
      // (Can't have more than one solution when dropping to a line,
      // nor to a circle, but it won't hurt to leave this extra logic in.)

      SmPoint3d sP1, sP2;
      double dDist;

      for (jj=0; jj<sSolutions1.GetSize(); jj++) 
        {
          for (kk=0; kk<sSolutions2.GetSize(); kk++) 
            {
              sSol.m_vStart[0] = sSolutions1[jj].m_vStart[0]; // ellipse
              sSol.m_vStart[1] = sSolutions2[kk].m_vStart[0]; // line

              SER( this-> EvaluatePoint( sSol.m_vStart[0], sP1 ));
              SER( crLine.EvaluatePoint( sSol.m_vStart[1], sP2 ));

              dDist = sP1.DistanceBetween(sP2);

              double dScaledZero = SM_EFF_ZERO * (1.0 + sP1.GetMaxDimension() );

              if ( bMayNeedRefinement )
                {
                  if (    dDist < dDistanceTolerance * 10.0  // close enough to be a contender
                       && dDist > dDistanceTolerance / 100.0 // far enough to need refinement
                     )
                    {
                      SmSolution sSol2;
                      SmBoolean bFoundAnswer;
                      SER( LocalCurveSolve( crInterval,
                                  crLine, crOtherInterval,
                                  SM_SO_MINIMIZE, dDistanceTolerance,
                                  &dDist, NULL,
                                  sSol.m_vStart[0],
                                  sSol.m_vStart[1],
                                  bFoundAnswer,
                                  sSol2 ));

                      // note: sSol2.m_vStart.m_dSolutionValue is positive definite
                      if (    bFoundAnswer
                           && sSol2.m_vStart.m_dSolutionValue <= dDistanceTolerance + dScaledZero )
                        {
                          dDist = sSol2.m_vStart.m_dSolutionValue;
                          sSol.m_vStart[0] = sSol2.m_vStart[0];
                          sSol.m_vStart[1] = sSol2.m_vStart[1];
                        }
                    }
                } // end if may need refinement

              if ( dDist <= dDistanceTolerance + dScaledZero )
                {
                  sSol.m_vStart.m_dSolutionValue = dDist;
                  rSolutions.Add(sSol);
                }

            } // end inner for loop (kk) on second curve solutions
        } // end outer for loop (jj) on first curve solutions

    } // end loop (ii )on all intersections (one or two)

  rbNeedsMoreIntersections = FALSE;
      
  return SM_SUCCESS;

} // end SmEllipse::IntersectWithLine

/*******************************************************************//**
PURPOSE: Is this ellipse a circle or a circular arc.   

NOTES: 
***********************************************************************/
SmBoolean SmEllipse::IsCircle
  () 
 const
{
  double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Max(smos_Fabs(GetXRadius()),smos_Fabs(GetYRadius())));

  // TRUE when radii are equal
  if (smos_Fabs(GetXRadius()-GetYRadius()) < dScaledZero) 
    { return TRUE; }
  return FALSE;

} // end SmEllipse::IsCircle

/*******************************************************************//**
PURPOSE: Determine if a circle is degenerate to a point.  

NOTES: 
***********************************************************************/
SmBoolean SmEllipse::IsDegenerate
  (double             d3DTolerance, // in : min distance between unique points, 0 = use dScaledZero
   const SmExtent1d * pInterval)    // NotUsed: in : interval to examine, NULL = use Natural Interval       
 const
{
  SM_REF1(pInterval) ;
  double dTol = smos_Max(d3DTolerance,SM_EFF_ZERO) ;

  // characterize the arc radius and length
  double dMaxRadius = smos_Max(m_dRadiusAtXAxis, m_dRadiusAtYAxis) ;
  double dArcLength = dMaxRadius * m_vAnalDomain.GetLength() * SM_PI / 180.0 ;
   
  // check for small radius or short interval
  SmBoolean bIsDegenerate =    (dMaxRadius < dTol / 2.0)
                            || (dArcLength < dTol) ;

  // all done
  return(bIsDegenerate) ;                              

} // end SmEllipse::IsDegenerate

/*******************************************************************//**
PURPOSE: Join the other curve onto this curve.  

NOTES: 
  The result is that this curve is modified.
  This will work only if the resulting curve can be an SmEllipse,
    otherwise return SM_ERR.  This requires that the other curve is part of
    the same ellipse as this curve, and has the coincident end points as
    indicated by the flags.
  The input end-flags refer to the NURBs representation, not the STEP:
    the m_bInsideOut flag is not considered.
***********************************************************************/
SmStatus SmEllipse::JoinWith
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

  //const SmContext *pContext = GetContext();

  // Check for the same ellipse, geometrically.

  // These checks work from the SmEllipse analytic representation.
  // Therefore, the other curve must actually be an SmEllipse.
  SmEllipse *pOtherEll = SM_CAST_PTR( SmEllipse, pOtherCurve );
  if ( pOtherEll == NULL )
    { return SM_ERR; }

  // Get geometric info.
  SmAxis2Placement sPlacementThis, sPlacementThat;
  double dRad1This = 0.0, dRad1That = 0.0;
  double dRad2This = 0.0, dRad2That = 0.0;

  this->GetCanonical( sPlacementThis, dRad1This, dRad2This );
  pOtherEll->GetCanonical( sPlacementThat, dRad1That, dRad2That );

  // Get a tolerance to use.
  double dTol = ( pGapTolerance != NULL )
                      ? *pGapTolerance
                      : smos_Max( GetXRadius(), GetYRadius() ) * SM_EFF_ZERO_SQRT * 1000.0;

  // Same curve? Geometry checks:
  if ( smos_Fabs( dRad1This - dRad1That ) > dTol )
    { return SM_ERR; }
  if ( smos_Fabs( dRad2This - dRad2That ) > dTol )
    { return SM_ERR; }

  // Note, SmAxis2Placement has an == operator, but it uses SM_EFF_ZERO,
  // and there's no way to pass a tolerance to a binary operator.
  //if ( ! ( sPlacementThis == sPlacementThat ) ) { return SM_ERR; }
  if ( ! sPlacementThis.GetOriginRef().CloserThan( dTol, sPlacementThat.GetOriginRef() ) )
    { return SM_ERR; }

  // These axis tests are too stringent: theycould be flipped,
  // or, if we are circles, all we need is coplanar.  [Reg_060221, Reg_060324]
  // if ( ! sPlacementThis.GetXAxisRef().CloserThan( dTol, sPlacementThat.GetXAxisRef() ) )
  //   { return SM_ERR; }
  // if ( ! sPlacementThis.GetYAxisRef().CloserThan( dTol, sPlacementThat.GetYAxisRef() ) )
  //   { return SM_ERR; }

  // We do need the same normals though, plus or minus.
  SmVector3d sCross = sPlacementThis.GetZAxis() * sPlacementThat.GetZAxis();
  if ( sCross.Length() > dTol )
    { return SM_ERR; }

  // If circular, then we're good to go, otherwise x and y axes must be compatible.
  if ( smos_Fabs( dRad2This - dRad1This ) > dTol )
    {
      sCross = sPlacementThis.GetXAxisRef() * sPlacementThat.GetXAxisRef();
      if ( sCross.Length() > dTol )
        { return SM_ERR; }
      sCross = sPlacementThis.GetYAxisRef() * sPlacementThat.GetYAxisRef();
      if ( sCross.Length() > dTol )
        { return SM_ERR; }
    }


  // Make sure that the points of join ends are close enough.
  // Grab all end points here.
  SmPoint3d  sThisCommonPt,  sThisEndPt;
  SmPoint3d sOtherCommonPt, sOtherEndPt;
  this  ->   GetEnds(  sThisCommonPt,  sThisEndPt );
  pOtherEll->GetEnds( sOtherCommonPt, sOtherEndPt );

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


  // Passed all tests, good to go.  Modify ourself.

  // Modify our analytic interval, then call MakeNurb() to update the NURBs.
  // Drop the other-end point of the other curve to the (whole) ellipse
  // and add that parameter to our start or end as indicated.
  // If the other end point drops to the interior of this curve,
  // then the two curves cover more than the whole ellipse,
  // and just the whole ellipse is returned.

  // Find the ellipse parameter of the other-end point of the other curve.
  // Allow it to find the proper value if it's outside our domain
  // (which it probably is).
  // Extend our analytic domain to span 360, off the end that we're joining with.
  // Here we're working with our analytic definition,
  // so we have to consider our inside-out flag.
  SmExtent1d sOrigDomain( m_vAnalDomain );

  // (Just to be safe, with TRUE/FALSE definitions:)
  ULONG lInsideOutFlag = ( m_bInsideOut ) ? 1 : 0;
  // (Sorry, just a bit tricky here:)
  if ( lJoinEndThis == lInsideOutFlag )
    { m_vAnalDomain.AddValue( m_vAnalDomain.GetMax() - 360.0 ); }
  else
    { m_vAnalDomain.AddValue( m_vAnalDomain.GetMin() + 360.0 ); }

  double dNewEllipseAngle;
  this->STEPInversion( sOtherEndPt, dNewEllipseAngle );

  // If it's within the original domain, then we want to end up with
  // the whole 360 degrees -- which we already set m_vAnalDomain to be.
  // So change it only if the dropped param is outside the original domain.
  if ( ! sOrigDomain.ContainsValue( dNewEllipseAngle, SM_EFF_ZERO ) )
    {
      m_vAnalDomain = sOrigDomain;
      m_vAnalDomain.AddValue( dNewEllipseAngle );
    }

  // And update the Nurbs.
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  this->MakeNurb();
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmLine::JoinWith

/*******************************************************************//**
PURPOSE: Make the Nurb representation for an ellipse
METHOD --- 
  1. Free current m_pNurb data if needed
  2. Call NLIb N_CreateEllipticalArc() to make the circle as a gw_CURVE object
  3. Save the gw_CURVE in m_pNurb
  4. Update m_vPolarConverter object
***********************************************************************/
SmStatus SmEllipse::MakeNurb
  ()
{
  // free existing m_pNurb object
  if (m_pNurb) { smos_Free(m_pNurb); m_pNurb = NULL; }

  // circle locals
  NL_POINT C;
  NL_POINT X, Y;

  // get circle origin and axis from Circle Position
  COPY_XYZ(m_vPosition.GetOriginRef(),C);
  COPY_XYZ(m_vPosition.GetXAxisRef(), X);
  double dStartAngDeg = GetStartAngleDeg() ;
  double dEndAngDeg   = GetEndAngleDeg() ;

  // Get X and Y vectors depending on bInsideOut flag
  if(m_bInsideOut) { // reverse and swap Start/End angles
                     double dT = dStartAngDeg ;
                     dStartAngDeg = 360.0 - dEndAngDeg ;
                     dEndAngDeg   = 360.0 - dT ;

                     if(dStartAngDeg > 360.0) { dStartAngDeg -= 360.0 ;
                                                dEndAngDeg   -= 360.0 ;
                                              }
                     
                     // negate the Y vector
                     SmVector3d sYAxis = -m_vPosition.GetYAxisRef() ;
                     COPY_XYZ(sYAxis, Y);
                   }
  else             { COPY_XYZ(m_vPosition.GetYAxisRef(), Y);
                   }
  NL_STACKS SC;
  SmStackHandler sStackKp(&SC);

  // allocate the gw_CURVE
  gw_CURVE Cur;
  N_CrvInitArrays(&Cur);
  SM_ASSERT( smos_Fabs(dEndAngDeg-dStartAngDeg) > SM_EFF_ZERO_SQRT) ;
  SM_ASSERT( smos_Fabs(dEndAngDeg-dStartAngDeg) < 360.0 + SM_EFF_ZERO_SQRT) ;
  
  // Convert 3d tolerance to Angle tol [B630]
  
  SmTol1d dAngleTol;
  const SmEdge *pEdge = GetEdge();
  if ( pEdge )
  { 
      SmZoneTol3d dZTol3d = SmTol::GetZoneTol3d( (const SmObject*)pEdge ); 
      SmApproxTol3d dApproxTol3d = dZTol3d / 100.;
      
      double dMidP = ( dStartAngDeg + dEndAngDeg ) / 2.0;
      SmVector3d aPointAndDer[2];
      EvaluateSTEP( dMidP, 1, FALSE, aPointAndDer );

      dAngleTol = dApproxTol3d / aPointAndDer[1].Length();
  }
  else
  {
      dAngleTol = 0.01; // Preserves behavior
  }


  // Build the circular arc
  // gwc note: start and end angles within about 1.0e-4 degrees of
  //           0 and 180 are snapped to 0 and 180.
  //           So the constructed curve's domain may vary slightly
  //           from [dStartAngDeg dEndAngDeg]
  if(NL_YES == N_CreateEllipticalArcTol(C, X, Y,
                                        m_dRadiusAtXAxis, m_dRadiusAtYAxis,
                                        dStartAngDeg, dEndAngDeg, dAngleTol,
                                        NL_QUADRATIC, &Cur, &SC))
    { SER(SM_ERR); }
  // GW_SER(N_CreateEllipticalArc(C, X, Y,
  //                              m_dRadiusAtXAxis, m_dRadiusAtYAxis,
  //                              dStartAngDeg, dEndAngDeg,
  //                              QUADRATIC, &Cur, &SC));
  
  // when arc's domain may have been modified
  if(   SM_IS_ZERO_TO_TOL(        dStartAngDeg, 3.0e-4)
     || SM_IS_ZERO_TO_TOL(180.0 - dStartAngDeg, 3.0e-4)
     || SM_IS_ZERO_TO_TOL(360.0 - dStartAngDeg, 3.0e-4)
     || SM_IS_ZERO_TO_TOL(        dEndAngDeg,   3.0e-4)
     || SM_IS_ZERO_TO_TOL(180.0 - dEndAngDeg,   3.0e-4)
     || SM_IS_ZERO_TO_TOL(360.0 - dEndAngDeg,   3.0e-4))
    {
      // compute analytic domain from curve

      // get curve endPoints
      SmPoint3d sStartPt, sEndPt ;
      TO_EUCLID(Cur.pol->Pw[0],         sStartPt);
      TO_EUCLID(Cur.pol->Pw[Cur.pol->n],sEndPt);

      // convert endPoints to ellipse angles
      double dAnalStart, dAnalEnd ;
      STEPInversion(sStartPt, dAnalStart) ;
      STEPInversion(sEndPt,   dAnalEnd) ;

      // Reset these to our analytic values.  (They were set for creating the Nurbs.)
      dStartAngDeg = ( m_bInsideOut ) ? GetEndAngleDeg()   : GetStartAngleDeg();
      dEndAngDeg   = ( m_bInsideOut ) ? GetStartAngleDeg() : GetEndAngleDeg();

      // watch out for periodicity
      if(!SM_IS_ZERO_TO_TOL(dAnalStart - dStartAngDeg, 3.0e-3))
        {
          if(SM_IS_ZERO_TO_TOL(dAnalStart + 360.0 - dStartAngDeg, 3.0e-3)) dAnalStart += 360.0 ;
          if(SM_IS_ZERO_TO_TOL(dAnalStart - 360.0 - dStartAngDeg, 3.0e-3)) dAnalStart -= 360.0 ;
          SM_ASSERT(SM_IS_ZERO_TO_TOL(dAnalStart - dStartAngDeg, 3.0e-3)) ;
        } 
      if(!SM_IS_ZERO_TO_TOL(dAnalEnd - dEndAngDeg, 3.0e-3))
        {
          if(SM_IS_ZERO_TO_TOL(dAnalEnd + 360.0 - dEndAngDeg, 3.0e-3)) dAnalEnd += 360.0 ;
          if(SM_IS_ZERO_TO_TOL(dAnalEnd - 360.0 - dEndAngDeg, 3.0e-3)) dAnalEnd -= 360.0 ;
          SM_ASSERT(SM_IS_ZERO_TO_TOL(dAnalEnd - dEndAngDeg, 3.0e-3)) ;
        } 

      // Set anal domain when its changed
      if(   !SM_IS_ZERO_TO_TOL(dAnalStart - dStartAngDeg, SM_EFF_ZERO)
         || !SM_IS_ZERO_TO_TOL(dAnalEnd   - dEndAngDeg,   SM_EFF_ZERO))
        {   
          // Either the start or end angle could be larger: m_bInsideOut.
          //m_vAnalDomain.SetMinMax(dAnalStart, dAnalEnd) ;
          m_vAnalDomain.Init();
          m_vAnalDomain.AddValue( dAnalStart );
          m_vAnalDomain.AddValue( dAnalEnd   );
        }
    } // end need to tweak analdomain check
  
  // set the m_pNurb pointer and use Notify to rebuild polarConverter 
  SetFromGwNurb(0, &Cur);

  // Set ControlPoint Z values for 2d curves
  if( m_lDim == 2)
    {
      this->ConvertTo2D() ;
    }

  // When this is a SmSurfOfExtrusion or SmSurfOfRevolution GenCurve,
  // look for Parent Surface NURB Interval constraint.
  // However, do not do this if the owner does not have a Nurbs surface
  // already made: leads to infinite recursion.  [B518]
  if(m_pOwner && m_pOwner->IsKindOf(SmSurfOfExtrusion_TYPE))
    {
      SmSurfOfExtrusion* pOwner = SM_CAST_PTR( SmSurfOfExtrusion, m_pOwner );
      if ( pOwner && pOwner->GetGwNurbPointer() != NULL )
        {
          SmExtent1d sNURBIvl = pOwner->GetGenDirParamExtent() ;
          this->EditParameterization(sNURBIvl, FALSE) ; // FALSE = No Need for Notify
        }
    }

  if(m_pOwner && m_pOwner->IsKindOf(SmSurfOfRevolution_TYPE))
    {
      SmSurfOfRevolution* pOwner = SM_CAST_PTR( SmSurfOfRevolution, m_pOwner );
      if ( pOwner && pOwner->GetGwNurbPointer() != NULL )
        {
          SmExtent1d sNURBIvl = pOwner->GetGenDirParamExtent() ;
          this->EditParameterization(sNURBIvl, FALSE) ; // FALSE = No Need for Notify
        }
    }

#ifdef SM_DEBUG_CODE
  // check that Nurb and Step end points are equivalent
  SmPoint3d sStartNurbPoint, sStartStepPoint ;
  SmPoint3d sEndNurbPoint, sEndStepPoint ;
  SmExtent1d sNurbIvl = GetNaturalInterval() ;

  // start point check
  SmBSplineCurve::EvaluatePoint(sNurbIvl.GetMin(), sStartNurbPoint) ;
  EvaluateSTEPPoint(m_vAnalDomain.GetMin(),        sStartStepPoint) ;
  SmBSplineCurve::EvaluatePoint(sNurbIvl.GetMax(), sEndNurbPoint) ;
  EvaluateSTEPPoint(m_vAnalDomain.GetMax(),        sEndStepPoint) ;

  // m_bInsideOut swaps endpoint correlations
  double dMinDist = sStartNurbPoint.DistanceBetween(m_bInsideOut ? sEndStepPoint
                                                                 : sStartStepPoint) ;
  double dMaxDist = sEndNurbPoint.DistanceBetween  (m_bInsideOut ? sStartStepPoint
                                                                 : sEndStepPoint) ;
  double dMinScaledZero = SM_EFF_ZERO * 5.0 * (1.0 + sStartNurbPoint.GetMaxDimension()) ;
  double dMaxScaledZero = SM_EFF_ZERO * 5.0 * (1.0 + sEndNurbPoint.GetMaxDimension()) ;

  // end point distance check
  SM_ASSERT(SM_IS_ZERO_TO_TOL(dMinDist, dMinScaledZero)) ;
  SM_ASSERT(SM_IS_ZERO_TO_TOL(dMaxDist, dMaxScaledZero)) ;

#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmEllipse::MakeNurb

/*******************************************************************//**
PURPOSE: Receive notification of things happening to the object and
take appropriate actions.  

NOTES: For example cleaning up the cache of an
object which is being deleted or edited.  Also clean up any attributes
and relations specific to this class which are not handled automatically
by construtors.

***********************************************************************/
void SmEllipse::Notify                 // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
 (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3                   
  SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
  SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL             
  SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                     
                                       // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
                                       // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                       // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                       // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
                                       // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
                                       // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
                                       // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                      
                                       // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
                                       // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                       // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                       // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL   
                                       // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     // 
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump() ;
      SM_DUMP_AND_ASSERT_VALID(this) ;  // don't expect to pass this. 
                               // just testing for out-of-date cases which
                               // should be updated below.

      SmEdge *pEdge = (SmEdge *)GetEdge() ;
      SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  switch (eNotifyOperation) 
    {
      case SM_NO_POST_EDIT:
          // recompute the polar converter
          m_vPolarConverter.SetUpPolarConversion(this, TRUE, m_vAnalDomain, m_vPosition) ;
          break ;

      case SM_NO_ADD_TO_BREP           : break ;
      case SM_NO_SPLIT_IN_BREP         : break ;
      case SM_NO_MERGE_IN_BREP         : break ;
      case SM_NO_TRIM_NO_SPLIT_IN_BREP : break ;
      case SM_NO_COINCIDENT            : break ;
      case SM_NO_RM_FROM_BREP          : break ;
      case SM_NO_CHANGE_GEOMETRY       : break ;
      case SM_NO_CHANGE_OWNER          : break ;
      case SM_NO_CONSTRUCTION          : break ;
      case SM_NO_COPY                  : break ;
      case SM_NO_PRE_EDIT              : break ;
      case SM_NO_SPLIT                 : break ;
      case SM_NO_MERGE                 : break ;
      case SM_NO_REG_PROPAGATION       : break ; 
      case SM_NO_DESTRUCTION           : break ;
      case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmEllipse::Notify - SM_NO_UNKNOWN event signalled")) ; } 
                                         break ;
    }
  
  // Propagate notification up hierarchy
  SmCurve::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmEllipse::Notify

// no longer needed - handled by SmEllipse::Notify     
//      /*******************************************************************//**
//      PURPOSE: set m_pNurb = CopyOf(cpGwNurbCurve) and initialize
//                  the polar converter
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmEllipse::SetFromGwNurb
//        (ULONG,                        // in : not used
//         const void* cpGwNurbCurve)    // in : a (gw_CURVE *) pointer to a Nurb Curve
//                                       //      to be copied into the m_pNurb pointer.
//      {
//        SER(SmBSplineCurve::SetFromGwNurb(0, (gw_CURVE *)cpGwNurbCurve));
//        SER(m_vPolarConverter.SetUpPolarConversion(this, TRUE, m_vAnalDomain, m_vPosition));
//        return SM_SUCCESS;
//      
//      } // end SmEllipse::SetFromGwNurb

/*******************************************************************//**
PURPOSE: Reverse the parameterization of a curve and update an 
    interval on the curve.

NOTES: 
***********************************************************************/
SmStatus SmEllipse::ReverseParameterization
  (const SmExtent1d & crOldInterval,  // in : current curve interval       
   SmExtent1d & rNewInterval)         // out: curve interval after reversal
{
    if (!m_pNurb) 
      {
        SER(MakeNurb());
      }
    m_vPosition.SetCanonical( m_vPosition.GetOriginRef(),
                              m_vPosition.GetXAxisRef(),
                             -m_vPosition.GetYAxisRef());
    m_vAnalDomain.SetMinMax(360.0-m_vAnalDomain.GetMax(),360.0-m_vAnalDomain.GetMin());
    SER(SmBSplineCurve::ReverseParameterization(crOldInterval,rNewInterval));

    return SM_SUCCESS;

} // end SmEllipse::ReverseParameterization

/*******************************************************************//**
PURPOSE: Trim this ellipse.

NOTES:  The input trim interval is given for the underlying BSpline domain,
   not the STEP angular domain.
***********************************************************************/
SmStatus SmEllipse::Trim
 (SmExtent1d & crTrimInterval,  // in : desired new Nurb domain
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
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  if(bDebugMe || lDebugCount == lCount)
    { Dump() ; }
#endif // SM_DEBUG_CODE

  // Old method, restored: Trim m_pNurb with given interval
  // - Problem: created very short spans on occasion
  //   That has been resolved.

  // First ask SmBSplineCurve::Trim to compute the actual trim interval.
  // - No Curve edit here, just occasionally modify crTrimInterval slightly.
  SER( SmBSplineCurve::Trim( crTrimInterval, UNSURE ));  // UNSURE = only snap crTrimInterval to within tol knots and boundaries

  // all done when asked to stop here
  if(bNotify == UNSURE)
    { return SM_SUCCESS ; }

  // Convert Nurb Parameters into STEP parameters
  double dMinSTEPParam, dMaxSTEPParam ;
  ConvertTFromNURBSToSTEP(crTrimInterval.GetMin(), dMinSTEPParam) ;
  ConvertTFromNURBSToSTEP(crTrimInterval.GetMax(), dMaxSTEPParam) ;
  SM_ASSERT(   (!m_bInsideOut && dMinSTEPParam < dMaxSTEPParam)
            || ( m_bInsideOut && dMinSTEPParam > dMaxSTEPParam)) ;

  // pass the call to Trim the BSpline - currently this modifies
  // TrimInterval by snapping to existing knots within tolerance
  // to avoid creating tolerance-size knot spacing. So this call has
  // to come before the Convert to Step and Set m_vAnalDomain calls and
  // SmBSplineCurve::Trim has to skip its notify step.
  SER( SmBSplineCurve::Trim( crTrimInterval, FALSE, TRUE )); // FALSE = Trim Curve - skip Notify call after trimming
                                                             // TRUE  = Trim Curve - skip AssertValid call before return
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { Dump() ; }
#endif // SM_DEBUG_CODE

  // store the angular interval paying attention to m_bInsideOut
  if(!m_bInsideOut) { m_vAnalDomain.SetMinMax(dMinSTEPParam, dMaxSTEPParam); }
  else              { m_vAnalDomain.SetMinMax(dMaxSTEPParam, dMinSTEPParam); }

  // rebuild the polar converter
  m_vPolarConverter.SetUpPolarConversion(this, TRUE, m_vAnalDomain, m_vPosition) ;

#ifdef SM_DEBUG_CODE
#ifdef SM_USE_CONSTRUCTOR_ASSERT_VALID
      if(!SM_ASSERT_VALID_CONSTRUCTION(this))
        { // place to break
          if(bDebugMe)
            { Dump() ; }
        }
#endif // SM_USE_CONSTRUCTOR_ASSERT_VALID
#endif // no SM_DEBUG_CODE

  // all done
  if(bNotify == TRUE)
    {
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;


//  // New method, revoked: Change the interval and rebuild the m_pNurb structure to
//  //             eliminate the chance of having Trim create very short BSplineSpans
//  // Problem: changes the parameterization of the curve (speed on the interior).
//  // [ prog_test, with m_bMakeComposites set to False in ManifoldBoolean() ]
//
//  // When asked - just return the new Nurb interval
//  //   - for Ellipses this is now the requested Nurb interval
//  if(bNotify == UNSURE)
//    { return SM_SUCCESS ; }
//
//  // Convert Nurb Parameters into STEP parameters
//  double dMinSTEPParam, dMaxSTEPParam ;
//  ConvertTFromNURBSToSTEP(crTrimInterval.GetMin(), dMinSTEPParam) ;
//  ConvertTFromNURBSToSTEP(crTrimInterval.GetMax(), dMaxSTEPParam) ;
//  SM_ASSERT(   (!m_bInsideOut && dMinSTEPParam < dMaxSTEPParam)
//            || ( m_bInsideOut && dMinSTEPParam > dMaxSTEPParam)) ;
//
//#ifdef SM_DEBUG_CODE
//  // check that the STEP/NURB conversion is good to ScaledZero tolerances
//  double dMinNurbTest, dMaxNurbTest ;
//  ConvertTFromSTEPToNURBS(dMinSTEPParam, dMinNurbTest) ;
//  ConvertTFromSTEPToNURBS(dMaxSTEPParam, dMaxNurbTest) ;
//  SM_ASSERT_MSG(SM_IS_ZERO_TO_TOL(crTrimInterval.GetMin() - dMinNurbTest, SmTol::GetScaledZero(dMinNurbTest)),
//                _T("SmEllipse::Trim conversion from NURB to STEP params out of tolerance")) ;
//  SM_ASSERT_MSG(SM_IS_ZERO_TO_TOL(crTrimInterval.GetMax() - dMaxNurbTest, SmTol::GetScaledZero(dMaxNurbTest)),
//                _T("SmEllipse::Trim conversion from NURB to STEP params out of tolerance")) ;
//#endif // SM_DEBUG_CODE
//
//  // locals
//  SmExtent1d sNewSTEPIvl;
//  if ( m_bInsideOut )
//    { SER(sNewSTEPIvl.SetMinMax(dMaxSTEPParam, dMinSTEPParam)); }
//  else
//    { SER(sNewSTEPIvl.SetMinMax(dMinSTEPParam, dMaxSTEPParam)); }
//
//  // Trim the curve with a call to AdjustSTEPInterval
//  SmStatus sRtn = AdjustSTEPInterval(sNewSTEPIvl) ;
//
//#ifdef SM_DEBUG_CODE
//#ifdef SM_USE_CONSTRUCTOR_ASSERT_VALID
//  if(!SM_ASSERT_VALID_CONSTRUCTION(this))
//#endif // SM_USE_CONSTRUCTOR_ASSERT_VALID
//    { // place to break
//      if(bDebugMe)
//        { Dump() ; }
//    }
//#endif // no SM_DEBUG_CODE
//
//  // all done
//  return(sRtn) ;


} // end SmEllipse::Trim

/*******************************************************************//**
PURPOSE: Scale and transform an Ellipse/Circle curve.

NOTES: Non-Uniform Scaling is not allowed on analytical curves.
***********************************************************************/
SmStatus SmEllipse::Transform
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

  // check input and get scaling value - only allow uniform scaling
  double dScale = 1.0 ;
  if(cpOptScale)
    { 
      if(   !SM_ARE_SAME(cpOptScale->x,cpOptScale->y) 
         || !SM_ARE_SAME(cpOptScale->x,cpOptScale->z)
         || smos_Fabs( cpOptScale->x ) < SM_EFF_ZERO) 
        { 
          ERR_MSG(_T("Unable to scale analytical surfaces non-uniformly\n"));
          SER(SM_ERR); 
        }
      else
        {
          dScale = cpOptScale->x ;
        }
    }

  // scale current ellipse placement
  SmAxis2Placement sEllPlace;
  SER(sEllPlace.SetCanonical(m_vPosition.GetOriginRef() * dScale, 
                             m_vPosition.GetXAxisRef(), 
                             m_vPosition.GetYAxisRef()));
  
  // apply rotation to scaled placement
  SmAxis2Placement sTmpA2P;
  sEllPlace.TransformAxis2Placement(crRotateNMove,sTmpA2P);

  // update ellipse placement
  m_vPosition       = sTmpA2P;

  // update ellipse size
  m_dRadiusAtXAxis *= dScale;
  m_dRadiusAtYAxis *= dScale;

  // apply transformation to underlying Nurb
  // which calls Notify which causes PolarConverter to be updated
  if (m_pNurb) 
    {
      SER(SmBSplineCurve::Transform(crRotateNMove,cpOptScale));
    }

  // all done
  return SM_SUCCESS;

} // end SmEllipse::Transform

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmEllipse.

NOTES: Does not include the curve's attribute memory
***********************************************************************/
ULONG SmEllipse::GetMemoryUsed     // rtn: smaller size of actually used memory in bytes
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

} // end SmEllipse::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertEllipse_list[] =
{
  {SM_AT_FLAG,   _T("bad m_bInsideOut flag value"), _T("m_bInsideOut can only be TRUE for circles") },
  {SM_AT_POINTS, _T("bad STEP to Nurb Approx"),     _T("Dist(EvalSTEP(STEPT), EvalNurb(Convert(STEPT)) greater than tol") },
  {SM_AT_POINTS, _T("Bad Nurb to STEP Approx"),     _T("Dist(NurbPt, EvalSTEP(STEPInversion(NurbPt)) greater than tol") }
} ;

/*******************************************************************//**
PURPOSE:  Check that Ellipse Analytic and Nurb representations are
             equivalent.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmEllipse::AssertValid
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
           ? SmBSplineCurve::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // make sure nested SmPolarConversion is up to date
  bRtn &= m_vPolarConverter.AssertValid(pAList) ;

  // check that the use of bInsideOut is limited to circles
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, m_bInsideOut == FALSE || IsKindOf(SmCircle_TYPE), _T("")) ;

  // Check that the BsplineCurve shape is the same as the analytic shape

  // locals
  SmExtent1d sNurbIvl = GetNaturalInterval() ;
  ULONG ii, lSmpCount = 5 ;
  for(ii=0;ii<lSmpCount;ii++)
    {
      // get equivalent Nurb and STEP params
      double dNurbT, dSTEPT = m_vAnalDomain.Evaluate((double)ii/(double)(lSmpCount-1)) ;
      ConvertTFromSTEPToNURBS(dSTEPT, dNurbT) ;

      // Evaluate the points
      SmPoint3d sNurbPoint, sSTEPPoint, sCheckPoint ;
      EvaluatePoint    (dNurbT, sNurbPoint) ;
      EvaluateSTEPPoint(dSTEPT, sSTEPPoint) ;
      double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + sNurbPoint.GetMaxDimension()) ;

      // Points are equal - others vary by a small amount
      double dDist = (sNurbPoint - sSTEPPoint).Length() ;
      bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, dDist < dScaledZero, dScaledZero, dDist, _T("")) ;

      // every Nurb Point must lie on the Ellipse
      double dSTEPParam ;
      STEPInversion(sNurbPoint, dSTEPParam) ;
      EvaluateSTEPPoint(dSTEPParam, sCheckPoint) ;
      double dCheckDist = (sCheckPoint-sNurbPoint).Length() ;
      bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, dCheckDist < dScaledZero, dScaledZero, dCheckDist, _T("")) ;

    } // end iter every sample point

   // all done
  return(bRtn) ;

} // end SmEllipse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmEllipse::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmEllipse::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmEllipse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmEllipse to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmEllipse::WriteToDB
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
      rFileOut << rOrig.x << " " << rOrig.y << " " << rOrig.z << " SmEllipse Axis Origin \n" ;
      rFileOut << rX.x    << " " << rX.y    << " " << rX.z    << " SmEllipse X Axis\n" ;
      rFileOut << rY.x    << " " << rY.y    << " " << rY.z    << " SmEllipse Y Axis\n" ;
      rFileOut << m_vAnalDomain.GetMin() << " " << m_vAnalDomain.GetMax() << " SmEllipse angular arc domain in degrees \n" ;
      rFileOut << m_dRadiusAtXAxis   << " SmEllipse X_Axis radius \n" ;
      rFileOut << m_dRadiusAtYAxis   << " SmEllipse Y_Axis radius \n" ;
      rFileOut << m_bInsideOut       << " SmEllipse InsideOut Flag \n" ;
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

      SER(rDB.WriteDouble(m_dRadiusAtXAxis));
      SER(rDB.WriteDouble(m_dRadiusAtYAxis));

      SER(rDB.WriteBoolean(m_bInsideOut));
    }

  // write the polar converter
  if (eType == SM_ASCII) { rFileOut << " SmEllipse->PolarConverter \n"; }
  SER(m_vPolarConverter.WriteToDB(rDB, lDBVersionNumber)) ; 

  // Write the base object
  SmBSplineCurve::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS ;

} // end SmEllipse::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmEllipse from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmEllipse::ReadFromDB
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
         || rpNewCurve->IsKindOf(SmEllipse_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmEllipse *pEllipse =   (rpNewCurve == NULL)
                                  ? new (crContext) SmEllipse()
                                  : (SmEllipse *)rpNewCurve ;

  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmPoint3d sOrig ;
  SmVector3d sX, sY ;
  double dAnalMin = 0.0, dAnalMax = 0.0;
  double dRadiusAtXAxis = 0.0;
  double dRadiusAtYAxis = 0.0;
  SmBoolean bInsideOut = false;
             
  if (eType == SM_ASCII) 
    {
      rFileIn >> sOrig.x >> sOrig.y >> sOrig.z ; rDB.GoToNextLine() ;
      rFileIn >> sX.x    >> sX.y    >> sX.z ; rDB.GoToNextLine() ;  
      rFileIn >> sY.x    >> sY.y    >> sY.z ; rDB.GoToNextLine() ;  
      rFileIn >> dAnalMin >> dAnalMax ; rDB.GoToNextLine() ;
      rFileIn >> dRadiusAtXAxis ; rDB.GoToNextLine() ;
      rFileIn >> dRadiusAtYAxis ; rDB.GoToNextLine() ;
      rFileIn >> bInsideOut ;  rDB.GoToNextLine() ;  
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

      SER(rDB.ReadDouble(dRadiusAtXAxis));
      SER(rDB.ReadDouble(dRadiusAtYAxis));

      SER(rDB.ReadBoolean(bInsideOut));
    }

  // PolarConverter - read directly into empty analytic
  SmPolarConversion *pPolarConverter = &pEllipse->m_vPolarConverter ;
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(SmPolarConversion::ReadFromDB(rDB, crContext, pEllipse, pPolarConverter, lDBVersionNumber)) ;

  // load the obj
  pEllipse->m_vPosition.SetCanonical(sOrig, sX, sY) ;   
  pEllipse->m_vAnalDomain.SetMinMax(dAnalMin, dAnalMax) ; 
  pEllipse->m_dRadiusAtXAxis = dRadiusAtXAxis ;
  pEllipse->m_dRadiusAtYAxis = dRadiusAtYAxis ;
  pEllipse->m_bInsideOut     = bInsideOut ;    

  // set the output
  rpNewCurve = pEllipse ;

  // read the base type
  SmBSplineCurve::ReadFromDB(SmBSplineCurve_TYPE, rDB, lDim, crContext, rpNewCurve, lDBVersionNumber) ; 

  // all done
  return SM_SUCCESS;

} // end SmEllipse::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmEllipse::IsKindOf( SM_TYPE t ) const
{
  return ((SmEllipse_TYPE == t) ? TRUE : SmBSplineCurve::IsKindOf( (t) ));
}


/*******************************************************************//**
PURPOSE: Dump Ellipse data out for debugging.

NOTES: 
***********************************************************************/
void SmEllipse::Dump()
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmEllipse::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_sprintf(sBuff,       _T("\nSmEllipse = 0x%p"), this) ;
  smos_sprintf(sBuffForFile,_T("\nSmEllipse = %sn"), _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n  AnalDomain  = ")); m_vAnalDomain.Dump();
  smos_WriteBuffer(_T("\n  NurbDomain  = ")); GetNaturalInterval().Dump();
  smos_sprintf(sBuff,_T("\n  Radius1 = %16.16lf, Radius2 = %16.16lf"),
                      m_dRadiusAtXAxis,m_dRadiusAtYAxis);
  smos_WriteBuffer(sBuff);

  // InsideOut
  if(!m_bInsideOut) 
    { 
      smos_WriteBuffer(_T(",   bInsideOut = FALSE ")) ; 
    }
  else // InsideOut == TRUE, report flipped endPoints
    { 
      // eval Nurb and Step endPoints
      SmPoint3d sNurbEndPoint, sNurbStartPoint ;
      SmPoint3d sStepEndPoint, sStepStartPoint ;
      SmExtent1d sNurbIvl = GetNaturalInterval();
      EvaluatePoint(sNurbIvl.GetMin(),sNurbStartPoint) ;
      EvaluatePoint(sNurbIvl.GetMax(),sNurbEndPoint) ;
      EvaluateSTEPPoint(m_vAnalDomain.GetMin(),sStepStartPoint) ;
      EvaluateSTEPPoint(m_vAnalDomain.GetMax(),sStepEndPoint) ;

      smos_WriteBuffer(_T(",   bInsideOut = TRUE ")) ;  
      smos_WriteBuffer(_T("\n      Line STEP Start Point = "));     sStepStartPoint.Dump();
      smos_WriteBuffer(_T("\n      Line STEP End   Point = "));     sStepEndPoint.Dump();
      smos_WriteBuffer(_T("\n      Line Nurb Start Point = "));     sNurbStartPoint.Dump();
      smos_WriteBuffer(_T("\n      Line Nurb End   Point = "));     sNurbEndPoint.Dump();
    }   

  smos_WriteBuffer(_T("\n  Transform       = ")); m_vPosition.Dump();
  smos_WriteBuffer(_T("\n  Polar Converter = ")); m_vPolarConverter.Dump();
  smos_WriteBuffer(_T("\n"));
  SmBSplineCurve::Dump();

  smos_WriteBuffer(_T(" End SmEllipse::Dump()\n")) ;

} // end Dump
