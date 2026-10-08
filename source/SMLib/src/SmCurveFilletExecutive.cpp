// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCurveFilletExecutive.cpp
* PURPOSE: Implementation of the SmCurveFilletExecutive class.
**********************************************************************/

  
#include "StdAfx.h"

#include <SmCurve.h>
#include <SmBSplineCurve.h>
#include <SmOffsetCurve.h>
#include <SmCurveFilletExecutive.h>


  // Constructor
SmCurveFilletExecutive::SmCurveFilletExecutive(
    SmCurve * pCurve1,
    SmCurve * pCurve2,
    double    dRadius,
    SmBoolean bDoTrim,
    SmBoolean bDoJoin )
  :
  m_pCurve1( pCurve1 ),
  m_pCurve2( pCurve2 ),
  m_dRadius( dRadius ),
  m_bDoTrim( bDoTrim ),
  m_bDoJoin( bDoJoin ),
  m_dParam1( SM_UNDEF_DOUBLE ),
  m_dParam2( SM_UNDEF_DOUBLE ),
  m_pResult( NULL )
{}


/*******************************************************************//**
PURPOSE: Perform the fillet operation, with no return argument.

NOTES: Use GetResult() to get the fillet curve after this call.
***********************************************************************/
SmStatus SmCurveFilletExecutive::DoFillet() { return DoFillet( m_pResult ); }


/*******************************************************************//**
PURPOSE: Perform the fillet operation, and return the result in the argument.

NOTES: 
   If the given curves do not intersect, a fillet can still be created
   if the offsets intersect.  This will attempt that.
***********************************************************************/
SmStatus SmCurveFilletExecutive::DoFillet( SmBSplineCurve *& rpNewFilletCurve )
{
  rpNewFilletCurve = NULL;
  NER( m_pCurve1 );
  NER( m_pCurve2 );
  if ( m_dRadius <= 0 ) { SER( SM_ERR_INVALID_INPUT ); }

  // Before we get started: If Joining is requested, then both of the
  // given curves must be SmBSplineCurve; other types cannont be joined.
  if ( m_bDoJoin )
  {
      if ( ! m_pCurve1->IsKindOf( SmBSplineCurve_TYPE ) )
        { SER_MSG( SM_ERR_INVALID_INPUT, _T("Curve Fillet Error: Requested to join non-SmBSplineCurves.") ); }
      if ( ! m_pCurve2->IsKindOf( SmBSplineCurve_TYPE ) )
        { SER_MSG( SM_ERR_INVALID_INPUT, _T("Curve Fillet Error: Requested to join non-SmBSplineCurves.") ); }
  }

  // Declare our offset curves first, in case the base curves don't intersect.
  // (Offset curves will use the contexts of the base curves.)
  SmOffsetCurve *pOffCrv1 = NULL;
  SmOffsetCurve *pOffCrv2 = NULL;
  const SmContext *pCtxt = m_pCurve1->GetContext();
  SmObjDelete sClean1( pOffCrv1 );
  SmObjDelete sClean2( pOffCrv2 );
  SmVector3d sPlanePt, sPlaneNorm;
  SmBoolean bHaveOffsets = FALSE;

  SmExtent1d sDom1 = m_pCurve1->GetNaturalInterval();
  SmExtent1d sDom2 = m_pCurve2->GetNaturalInterval();

  SmSolutionArray sSolutions;
  double dTol = SM_EFF_ZERO_SQRT;  //cbiTol: check this.
  SmStatus eStat = m_pCurve1->GlobalCurveIntersect( sDom1, *m_pCurve2, sDom2, dTol, sSolutions, TRUE );

  // If the given curves don't intersect, it may still be possible to fillet them:
  // see if they have offsets that intersect.
  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
  {
      // If we find an offset-curve intersection,
      // then we will not have to do the offset intersection below.
      bHaveOffsets = TRUE;

      // We need a plane normal to create offset curves,
      // and we don't have the curve intersection.
      SmTArray< SmCurve* > sCrvArray;
      sCrvArray.Add( m_pCurve1 );
      sCrvArray.Add( m_pCurve2 );
      if ( ! m_pCurve1->AreCoPlanar ( sCrvArray, dTol, &sPlanePt, &sPlaneNorm ) )
        {SER( SM_ERR_INVALID_INPUT ); }

      bHaveOffsets = TRUE;
      pOffCrv1 = new (*pCtxt) SmOffsetCurve( m_pCurve1->GetDim(), *m_pCurve1, sDom1, sPlaneNorm, m_dRadius );
      pOffCrv2 = new (*pCtxt) SmOffsetCurve( m_pCurve2->GetDim(), *m_pCurve2, sDom2, sPlaneNorm, m_dRadius );
      sClean1.SetObj( pOffCrv1 );
      sClean2.SetObj( pOffCrv2 );

      eStat = pOffCrv1->GlobalCurveIntersect( sDom1, *pOffCrv2, sDom2, dTol, sSolutions, TRUE );
  }
  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
  {
      pOffCrv1->SetOffsetDistance( -m_dRadius );
      eStat = pOffCrv1->GlobalCurveIntersect( sDom1, *pOffCrv2, sDom2, dTol, sSolutions, TRUE );
  }
  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
  {
      pOffCrv2->SetOffsetDistance( -m_dRadius );
      eStat = pOffCrv1->GlobalCurveIntersect( sDom1, *pOffCrv2, sDom2, dTol, sSolutions, TRUE );
  }
  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
  {
      pOffCrv1->SetOffsetDistance( m_dRadius );
      eStat = pOffCrv1->GlobalCurveIntersect( sDom1, *pOffCrv2, sDom2, dTol, sSolutions, TRUE );
  }


  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
    { SER_MSG( SM_ERR, _T("Curve Fillet Error: No intersection of offset curves.") ); }

  if ( sSolutions.GetSize() > 1 || sSolutions[0].m_eSolutionType != SM_ST_SINGLE_VALUE )
    { WARN( _T("Warning: Curve Fillet found multiple intersections; using the first.")  ); }

  double dT1 = sSolutions[0].m_vStart[0];
  double dT2 = sSolutions[0].m_vStart[1];

  // Figure out which sides.
  double dCheckParam1 = ( m_dParam1 != SM_UNDEF_DOUBLE ) ? m_dParam1 : sDom1.GetMid();
  // double dCheckParam2 = ( m_dParam2 != SM_UNDEF_DOUBLE ) ? m_dParam2 : sDom2.GetMid();
  SmBoolean bHiSide1 = ( dT1 < dCheckParam1 );
  SmBoolean bHiSide2 = ( dT2 < dCheckParam1 );

  // To get Left/Right we need the normal at the intersection.
  SmVector3d sEvals[2];
  m_pCurve1->Evaluate( dT1, 1, !bHiSide1,  sEvals );
  SmPoint3d  sPt1  = sEvals[0];
  SmVector3d sTan1 = sEvals[1];
  m_pCurve2->Evaluate( dT2, 1, !bHiSide2,  sEvals );
  SmPoint3d  sPt2  = sEvals[0];
  SmVector3d sTan2 = sEvals[1];

  sPlaneNorm = sTan1 * sTan2;
  eStat = sPlaneNorm.Unitize();

  SER_MSG( eStat, _T("Curve Fillet Error: Curves are tangent at intersection.") );


  // Intersect the offsets; the base curve intersection is a good guess.
  double dOffT1, dOffT2, dDev;

  if ( bHaveOffsets )
  {
      dOffT1 = dT1;
      dOffT2 = dT2;
  }
  else
  {
      double dOffDist1 = ( bHiSide2 ) ? -m_dRadius :  m_dRadius;
      double dOffDist2 = ( bHiSide1 ) ?  m_dRadius : -m_dRadius;

      pOffCrv1 = new (*pCtxt) SmOffsetCurve( m_pCurve1->GetDim(), *m_pCurve1, sDom1, sPlaneNorm, dOffDist1 );
      pOffCrv2 = new (*pCtxt) SmOffsetCurve( m_pCurve2->GetDim(), *m_pCurve2, sDom2, sPlaneNorm, dOffDist2 );
      sClean1.SetObj( pOffCrv1 );
      sClean2.SetObj( pOffCrv2 );
      SmBoolean bFound;

      SER( pOffCrv1->LocalCurveIntersect( sDom1, *pOffCrv2, sDom2, dTol, dT1, dT2,
                                          bFound, dOffT1, dOffT2, dDev) );

      if ( ! bFound )
        { SER_MSG( SM_ERR, _T("Curve Fillet Error: No intersection of offset curves.") ); }
  }

  // Get start, end, and center points.
  SmPoint3d sCrv1Pt, sCrv2Pt, sCenter;
  m_pCurve1->EvaluatePoint( dOffT1, sCrv1Pt );
  m_pCurve2->EvaluatePoint( dOffT2, sCrv2Pt );
  pOffCrv1->EvaluatePoint( dOffT1, sCenter );

  // Just to be safe:
  SmPoint3d sTempPt;
  pOffCrv2->EvaluatePoint( dOffT2, sTempPt );
  SM_ASSERT( sCenter.DistanceBetween( sTempPt ) <= dTol );
  sCenter = ( sCenter + sTempPt ) / 2;  // Midpoint.

  // Create the circle.  Use SmBSplineCurve::CreateArcFromPoints().
  // Use the context and dimension of pCurve1.
  ULONG lDim = m_pCurve1->GetDim();
  SmBSplineCurve::CreateArcFromPoints( *pCtxt, lDim,
     sCenter, sCrv1Pt, sCrv2Pt, SM_CO_QUADRATIC, m_pResult );

  // Check result
  NER( m_pResult );


  // Trim the originals if requested.
  // Note: if DoJoin is specified, then we have to trim.
  // Just do that silently here.
  if ( m_bDoTrim || m_bDoJoin )
  {
      if ( ! m_bDoTrim )
        { WARN( _T("Warning: Curve Fillet Join requested without Trim: Trimming input curves.") ); }

      if ( bHiSide1 )
        { sDom1.SetMinMax( dOffT1, sDom1.GetMax() ); }
      else
        { sDom1.SetMinMax( sDom1.GetMin(), dOffT1 ); }
      m_pCurve1->Trim( sDom1 ); // may snap sIvl by tol to existing knots

      if ( bHiSide2 )
        { sDom2.SetMinMax( dOffT2, sDom2.GetMax() ); }
      else
        { sDom2.SetMinMax( sDom2.GetMin(), dOffT2 ); }
      m_pCurve2->Trim( sDom2 );  // may snap sIvl by tol to existing knots
  }

  // Join the three curves into one if requested.
  if ( m_bDoJoin )
  {
      SmBSplineCurve *pBSpline = SM_CAST_PTR( SmBSplineCurve, m_pCurve1 );
      NER( pBSpline );  // Already check this above, should be ok here.

      // End flags: 0 start, 1 end.
      ULONG bFilletEnd = 0; // Fillet starts at pCurve1.

      ULONG bOtherEnd = ( bHiSide1 ) ? 0 : 1;
      m_pResult->JoinWith( bFilletEnd, pBSpline, bOtherEnd );


      pBSpline = SM_CAST_PTR( SmBSplineCurve, m_pCurve2 );
      NER( pBSpline );  // Already check this above, should be ok here.

      bFilletEnd = 1;
      bOtherEnd = ( bHiSide2 ) ? 0 : 1;
      m_pResult->JoinWith( bFilletEnd, pBSpline, bOtherEnd );
  }

  rpNewFilletCurve = m_pResult;

  return SM_SUCCESS;
}


