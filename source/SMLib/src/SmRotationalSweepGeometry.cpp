// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmRotationalSweepGeometry.cpp
* PURPOSE   --- Source for class members.
**********************************************************************/

#include "StdAfx.h"

#include <SmRotationalSweepGeometry.h>

#include <SmLine.h>
#include <SmBSplineSurface.h>
#include <SmGeomUtility.h>
#include <SmTopologySweep.h>

#ifdef SM_DEBUG_CODE
  #include <SmCircle.h>
#endif

/*******************************************************************//**
PURPOSE: returns TRUE = Curve is a line

NOTES: checks the distance between curve samples and a line
       running through the curve end points.

       Could have just fit the curve points to a line but
       that might get confusing (a nonlinear curve cut short enough might
       be found to be linear to various lines over various intervals)
       - perhaps it's best to keep this test simple.
***********************************************************************/
static SmStatus sm_CheckEdgeCurveLine
  (SmCurve     * pCurveArg,    // in : curve to query
   SmExtent1d  & rIVArg,       // in : curve interval to query
   double        dEpsArg,      // in : min dist between distinct points
   SmBoolean   & brIsLineArg)  // out: TRUE = curve is linear over interval, FALSE = isn't
{
  SmBSplineCurve* pBSP = SM_CAST_PTR(SmBSplineCurve, pCurveArg);
  if(pBSP==NULL) { SER(SM_ERR); }

  ULONG nSamplePoints = 5;
  // something like this needs to be done here:
  // find a realistic number of points from the relation between
  // appr_length and tol, depending on assumed complexity of the curve
  // (obtained from the order or similar). But since nobody seems to care
  // throughout the code visible to me, I leave it this way.

  SmPoint3d  sLinePt;
  SmVector3d sLineVect;
  brIsLineArg = pBSP->IsSegmentLine(rIVArg,         // in : interval to query
                                    nSamplePoints,  // in : number of sample points
                                    dEpsArg,        // in : min dist between distinct points
                                    sLinePt,        // out: curve start point
                                    sLineVect) ;    // out: curve unit-chord
  // all done
  return SM_SUCCESS;

} // end sm_CheckEdgeCurveLine

/*******************************************************************//**
PURPOSE: checks to see if edge->endVertices are on the rotation axis

NOTES: when the curve is linear additionally checks to see
       if the curve is parallel or perpendicular to the rotation axis
***********************************************************************/
static SmStatus sm_CheckEdgeVerticesOnAxis
 (const SmPoint3d   & rBasePtArg,        // in : point on axis
  const SmVector3d    rAxisArg,          // in : vector direction of axis
  const SmEdge      * pEdgeArg,          // in : edge to query
  SmBoolean           bEdgeIsLineArg,    // in : TRUE = Edge is a line, FALSE = Edge is a general curve
  double              dEpsArg,           // in : Min distance between unique points
  SmBoolean         & bStartOnArg,       // out: TRUE = EdgeStart is on curve
  SmBoolean         & bEndOnArg,         // out: TRUE = EdgeEnd is on curve
  SmBoolean         & bParallelArg,      // out: TRUE = LinearEdge is parallel to rAxis
  SmBoolean         & bPerpArg)          // out: TRUE = LinearEdge is perpendicular to rAxis
{
  // init output
  bStartOnArg  = FALSE ;
  bEndOnArg    = FALSE ;
  bParallelArg = FALSE ;
  bPerpArg     = FALSE ;

  // locals
  SmVertex  * pStartVtx = pEdgeArg->GetStartVertex();
  SmVertex  * pEndVtx   = pEdgeArg->GetOtherVertex(pStartVtx);
  SmPoint3d   sStartPt( pStartVtx->GetPoint() );
  SmPoint3d   sEndPt  (   pEndVtx->GetPoint() );

  // distance from edge start point to rotation axis
  double dStartDist, dEndDist;
  SER(smgu_LinePointDistance(rBasePtArg, rAxisArg,
                             sStartPt, dStartDist));

  // distance from edge end point to rotation axis
  SER(smgu_LinePointDistance(rBasePtArg, rAxisArg,
                             sEndPt, dEndDist));

  // set output: remember if EdgeEndPoints are on rotation axis
  if (dStartDist < dEpsArg)
    { bStartOnArg = TRUE ; }
  if(dEndDist < dEpsArg )
    { bEndOnArg = TRUE ; }

  // when curve is linear - set perpendicular and parallel checks
  if (bEdgeIsLineArg)
    {
      // local
      SmVector3d sLineVec(sStartPt, sEndPt) ;

      // NonZero length lines can be parallel or perpendicular
      if(sLineVec.LengthSquared() > dEpsArg * dEpsArg)
        {

          // lines are parallel when vectors are parallel
          bParallelArg = rAxisArg.IsParallelTo(sLineVec, SM_EFF_ZERO_DEG) ;

          // lines are perp when vectors are perp
          bPerpArg     = rAxisArg.IsPerpendicularTo(sLineVec, SM_EFF_ZERO_DEG) ;

        } // end is LineLength nonZero check

      // old test - not valid: if(smos_Fabs(dStartDist - dEndDist) < dEpsArg)
      //                         bParallelArg = TRUE;
      //
      // old test - not valid: double dLineMin, dLineMax; // parameter locations on the line
      //                       SER(smgu_LineClosestPoint(rBasePtArg, rAxisArg, sStartPt, dLineMin));
      //                       SER(smgu_LineClosestPoint(rBasePtArg, rAxisArg, sEndPt, dLineMax));
      //                       if (smos_Fabs( dLineMin - dLineMax) < dEpsArg)
      //                           bPerpArg = TRUE;
    } // end curve is linear check

  // all done
  return SM_SUCCESS;

} // end sm_CheckEdgeVerticesOnAxis

/*******************************************************************//**
PURPOSE: Check to see if Edge is coincident with the rotation axis

NOTES:
***********************************************************************/
static SmStatus sm_CheckEdgeOnAxis
  (const SmPoint3d   & rBasePtArg,       // in : point on axis
   const SmVector3d    rAxisArg,         // in : vector direction of axis
   const SmEdge      * pEdgeArg,         // in : edge to query
   SmBoolean           bEdgeIsLineArg,   // in : TRUE = Edge is a line, FALSE = Edge is a general curve
   double              dEpsArg,          // in : Min distance between unique points
   SmBoolean         & bOnEdgeArg)       // out: TRUE = Edge is coincident with the rotation axis, FALSE = isn't
{
  // init output
  bOnEdgeArg = FALSE ;

  // locals
  SmCurve   * pOriginalEdgeCurve = pEdgeArg->GetCurve();
  SmBoolean bStartOn, bEndOn, bDummyParallel, bDummyPerp;

  // see if edge->EndVertices are on rotation axis
  SER(sm_CheckEdgeVerticesOnAxis(rBasePtArg,       // in : point on axis
                                 rAxisArg,         // in : vector direction of axis
                                 pEdgeArg,         // in : edge to query
                                 bEdgeIsLineArg,   // in : TRUE = Edge is a line, FALSE = Edge is a general curve
                                 dEpsArg,          // in : Min distance between unique points
                                 bStartOn,         // out: TRUE = EdgeStart is on curve
                                 bEndOn,           // out: TRUE = EdgeEnd is on curve
                                 bDummyParallel,   // out: TRUE = LinearEdge is parallel to rAxis
                                 bDummyPerp));     // out: TRUE = LinearEdge is perpendicular to rAxis

  // low work - either endVertex is not on rotation axis
  if (!bStartOn || !bEndOn )
    {
      bOnEdgeArg = FALSE;
      return SM_SUCCESS;
    }

  // low work - edge is linear (and both end-points are on the rotation axis)
  if (bEdgeIsLineArg)
    {
      bOnEdgeArg = TRUE;
      return SM_SUCCESS;
    }

  // arrive here when non-linear edge has both endpoints on the rotation axis
  // try intersecting the curve with the rotation axis to see if it's coincident
  //  within tolerance although not linear.

  // line->Vertex end points
  SmVertex  * pStartVtx = pEdgeArg->GetVertex();
  SmVertex  * pEndVtx   = pEdgeArg->GetOtherVertex(pStartVtx);
  SmPoint3d   sStartPt( pStartVtx->GetPoint() );
  SmPoint3d   sEndPt  (   pEndVtx->GetPoint() );

  // rotation axis parameters closest to Start/End Vtx locations
  double dLineMin, dLineMax;
  SER(smgu_LineClosestPoint(rBasePtArg, rAxisArg, sStartPt, dLineMin));
  SER(smgu_LineClosestPoint(rBasePtArg, rAxisArg, sEndPt,   dLineMax));

  // order the rotation axis parameters
  if (dLineMax < dLineMin )
    {
      SM_SWAP(double, dLineMax, dLineMin);
    }

  // make a temporary SmLine object for the rotation axis
  SmLine      * pAxisLine = new (*pOriginalEdgeCurve->GetContext()) SmLine(rBasePtArg, rAxisArg);
  SmObjDelete   sCleanLine(pAxisLine);

  // intersect the curve with the rotation line
  SmSolutionArray sSolutions;
  SmExtent1d      sEdgeIV = pEdgeArg->GetInterval();
  SER( pOriginalEdgeCurve->GlobalCurveIntersect(sEdgeIV,
                                                *pAxisLine,
                                                SmExtent1d(dLineMin, dLineMax),
                                                dEpsArg,
                                                sSolutions));
  ULONG nSolutions = sSolutions.GetSize();

  // when there are no intersections               - return FALSE
  // when there is more than one solution          - return FALSE
  // When there is just one single valued solution - return FALSE
  // when there is just one range of values solutions -
  if (   nSolutions == 0
      || nSolutions  > 1
      || sSolutions[1].m_eSolutionType == SM_ST_SINGLE_VALUE)
    { bOnEdgeArg = FALSE;
    }
  else  // Must be coincident segment
    {
      SM_ASSERT(sSolutions[0].m_eSolutionType == SM_ST_RANGE_OF_VALUES);

      // Get start and end parameters of range on axis
      double dAxisOnStart = sSolutions[0].m_vStart[1];
      double dAxisOnEnd   = sSolutions[0].m_vEnd[1];

      // for the edge to be ON, this segment of the axis
      // must not be less than the one we had above:
      if (   dAxisOnStart > dLineMin + dEpsArg
          || dAxisOnEnd   < dLineMax - dEpsArg)
        {
          // We can not process the partially-on case
          // so let's just ignore it anyway:
              SER(SM_ERR); // 'RotSweep: edge crosses axis, xsection IGNORED'
        }
      else
        {
          bOnEdgeArg = TRUE;
        }
    } // end found a coincident segment branch

  // all done
  return SM_SUCCESS;

} // end sm_CheckEdgeOnAxis

/*******************************************************************//**
PURPOSE: Checks if the accumulated rotation angle is 360 degrees.

NOTES: Reports an error if it exceeds 360.
***********************************************************************/
SmStatus SmRotationalSweepGeometry::IsSweepClosed
  (ULONG nRepeatsArg,
   SmBoolean& bRet)
  const
{
  double dRepeats = (double)nRepeatsArg;
  bRet = FALSE;
  if (360.0 - SM_EFF_ZERO < dRepeats * m_dAngle &&
                            dRepeats * m_dAngle < 360.0 + SM_EFF_ZERO )
      bRet = TRUE;
  if (dRepeats * m_dAngle > 360.0 + SM_EFF_ZERO)
      return SM_ERR;

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::IsSweepClosed

/*******************************************************************//**
PURPOSE: Verify that sweeping a curve will produce a valid swept surface.
NOTE: not all tests are released.

  Sets the output to oneof
   rSweepState = SM_SC_OKAY               = okay to sweep this surface
                 SM_SC_ROT_ON_CURVE       = revolving curve about point on curve produces a BowTie or ConeApex surface
                 SM_SC_SWEEP_ALONG_LENGTH = sweeping curve along it's length produces degenerate surface points
                 SM_SC_SELF_INTERSECT     = sweeping this curve produces a self-intersecting surface
                 SM_SC_UNKNOWN            = Sweep method failed so swept surface validity is unknown

NOTES:

  INVALID SURFACES: A swept surface is invalid if it contains any multi-valued or degenerate points
                    other than seams and poles.
    multi-valued = any two surface parameter points mapping to the same 3d point location.
    degenerate   = any surface point where the surface TangentU and the TangentV vectors are the same
                   (TangentU == +/-TangentV) or where either of the surface 1st derivative magnitudes
                   goes to zero (DerivativeU == 0 and/or DerivativeV == 0). At degenerate points, the
                   surface does not have a well defined surface normal vector.

    Revolving a curve about its endPoint(s) will create a valid Swept surface with a pole(s).
    That surface will have the following properties along its v0 = VMax or v0 = VMin iso-parameter curve:

        Position(u0,v0) == Position(u1,v0)   for u0 != u1  - all iso-Curve points have the same 3d point location
        TangentU(u0,v0) == 0                 for all u0    - all iso-Curve tangents in that direction are zero
        TangentV(u0,v0) != 0                 for all u0    - the iso-Curve cross-tangents are non-zero
        TangentV(u0,v0) != TangentV(u1,v0)   for u0 != u1  - the iso-Curve cross-tangents are unique

    Revovling a curve about one of its interior points will create an invalid Swept surface with a single point
    of degeneracy on one of its isoBoundary curves.
      if rotation axis is perpendicular to the curve tangent at the rotation point
         - a invalid "Bow Tie" surface is created.
      if rotation axis is not perpendicular to the curve tangent at the rotation point
         - an invalid cone like surface running through its apex point is created.

  SWEEP CHECKS:  This method checks for the following set of sweep problems

  SM_SC_OKAY,                sweeping this curve will produce a valid swept surface

  SM_SC_ROT_ON_CURVE,        rotating a curve about a point on the curve creates a cone apex or a
                             bow-tie surface with a degenerate surface point on one of its boundaries.
                                Split the curve at the point and try the same sweep on the pieces.

  SM_SC_SWEEP_ALONG_LENGTH,  The curve has point(s) tangent to the sweep direction, sweeping this curve
                             will produce a line of degenerate surface points for each such tangency point
                             where TangentU == +/-TangentV.
                                This sweep can't work, either change the sweep (ex: pick a new sweep direction)
                                or the sweep geometry (ex: trim the curve to remove the tangent point) to try again.

  SM_SC_SELF_INTERSECT,      Sweeping the curve will create a self-intersecting swept surface.
                                It's possible that a shorter sweep amount will work, or
                                Split the curve into segments at every point the curve is tangent to the sweep surfaces
                                and sweep the segements to produce a set of valid swept surfaces that intersect one another.

  SM_SC_UNKNOWN              The check method failed to run properly so the sweep validity is unknown
***********************************************************************/
SmStatus SmRotationalSweepGeometry::CheckCurveSweep
 ( const SmCurve     & crCurve,       // in : Curve to check for valid sweep
   const SmExtent1d  & crIvl,         // in : Interval to sweep
   double              dDistTol3d,    // in : min dist between unique points
   const SmPoint3d   & crSweepPoint,  // in : RotSweep:[RotAxis point],  TransSweep:[NotUsed]
   const SmVector3d  & crSweepVec,    // in : RotSweep:[RotAxis vector], TransSweep:[Trans vector]
   double              dSweepAmount,  // in : RotSweep:[SweepAngleDeg],  TransSweep:[distance]
   SmSweepCheckType  & rSweepState)   // out: SM_SC_OKAY               = okay to sweep this surface
  const                               //      SM_SC_ROT_ON_CURVE       = revolving curve about point on curve produces a BowTie or ConeApex surface
                                      //      SM_SC_SWEEP_ALONG_LENGTH = sweeping curve along it's length produces degenerate surface points
                                      //      SM_SC_SELF_INTERSECT     = sweeping this curve produces a self-intersecting surface
                                      //      SM_SC_UNKNOWN            = Sweep method failed so swept surface validity is unknown
{
  // init output
  rSweepState = SM_SC_UNKNOWN ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      const SmContext * cpContext = crCurve.GetContext() ;
      const SmEdge    * cpEdge    = crCurve.GetEdge() ;
      SmBrep          * pBrep     = cpEdge ? cpEdge->GetBrep() : NULL ;
      double            dSize     = crCurve.ApproximateLength(crIvl, 5) ;
      SmVector3d        sX, sY, sZ ;
      crSweepVec.MakeUnitOrthoVectors(NULL, sZ, sX, sY) ;
      SmExtent1d sSweepIvl(0.0, dSweepAmount) ;
      SmCircle sSweepCircle(crSweepPoint, sX, sY, sSweepIvl, dSize/1.865, 3, cpContext) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; crSweepPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&crSweepPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(crSweepPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sSweepCircle.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#else
  SM_REF1(dSweepAmount);
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii ;
  SmSolutionArray sLineXSectSols ;
  SmSolutionArray sDistToLineSols ;
  SmSolutionArray sDistToPlaneSols ;
  SmSolutionArray sSelfXSectSols ;

  // SM_SC_ROT_ON_CURVE: Check for illegal rotations about curve mid points

  // intersect the curve with the rotation line
  SmBoolean bNeedsMoreIntersections = FALSE ;
  SmLine sSweepAxis(crSweepPoint, crSweepVec, 3, TRUE, crCurve.GetContext()) ;
  SER(crCurve.IntersectWithLine(crIvl, sSweepAxis, sSweepAxis.GetNaturalInterval(), dDistTol3d, bNeedsMoreIntersections, sLineXSectSols)) ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      const SmContext * cpContext = crCurve.GetContext() ;
      const SmEdge    * cpEdge    = crCurve.GetEdge() ;
      SmBrep          * pBrep     = cpEdge ? cpEdge->GetBrep() : NULL ;
      double            dSize     = crCurve.ApproximateLength(crIvl, 5) ;
      SmVector3d        sX, sY, sZ ;
      crSweepVec.MakeUnitOrthoVectors(NULL, sZ, sX, sY) ;
      SmExtent1d sSweepIvl(0.0, dSweepAmount) ;
      SmCircle sSweepCircle(crSweepPoint, sX, sY, sSweepIvl, dSize/1.865, 3, cpContext) ;

      sLineXSectSols.Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; crSweepPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&crSweepPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(crSweepPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sSweepCircle.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; sLineXSectSols.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every solution - check for midCurve rotations
  for(ii=0;ii<sLineXSectSols.GetSize();ii++)
    {
      SmSolution &rSolution = sLineXSectSols[ii] ;
      double      dParam    = rSolution.m_vStart[0] ;

      // when the solution is not an end point
      if(crIvl.IsValueOnBoundary(dParam, dDistTol3d) == FALSE)
        {
          // attempting to rotate a curve about a mid point
          rSweepState = SM_SC_ROT_ON_CURVE ;

          // all done
          return(SM_SUCCESS) ;
        }
    } // end iter every soluton looking for midCurve rotations

#ifdef SM_SWEEP_CHECKS
  // GWC: add these checks after verifying new SM_CP_MAXIMA_TO_LINE and SM_CP_MAXIMA_TO_PLANE options
  //      and GlobalCurveSelfIntersect() methods
  // locals
  ULONG i2 ;
  SmVector3d sOptVecs[2], sPV[2] ;
  SmTArray<SmVector3d> sCurvePVs ;

  sOptVecs[0] = crSweepPoint ;
  sOptVecs[1] = crSweepVec ;

  // get all places on the curve where it doubles back on itself as measured as distance from rotation axis
  //   watch out: solution also finds line/curve intersection points
  //              arrive here after quiting for any internal line/curve intersections
  //              So, there may be two line/curve intersection points at the curve endpoints.
  crCurve.GlobalPropertyAnalysis(crIvl,                  // in : ThisCurve target interval
                                 SM_CP_MAXIMA_TO_LINE,   // in : The property of the curve to extract or find on the curve
                                 NULL,                   // in : Value used to specify a particular property.
                                 sOptVecs,               // in : Vectors or points used to define a particular property.
                                                         //      If there is a point, that should be the first value in the array
                                                         //      If there is a normal (projection), it will come before all other vectors
                                 dDistTol3d,             // in : 3D Tolerance used to determine when two answers are equivalent
                                 sDistToLineSols,        // out: SolArray of Curve Points at dist to line maxima
                                 NULL,                   // out: opt check comb: curve sample points
                                                         //      NULL to ignore, default:[NULL]
                                 NULL) ;                 // out: opt check comb: tines - lengths proportional to property being zeroed.
                                                         //      NULL to ignore, default:[NULL]

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      const SmContext * cpContext = crCurve.GetContext() ;
      const SmEdge    * cpEdge    = crCurve.GetEdge() ;
      SmBrep          * pBrep     = cpEdge ? cpEdge->GetBrep() : NULL ;
      double            dSize     = crCurve.ApproximateLength(crIvl, 5) ;
      SmVector3d        sX, sY, sZ ;
      crSweepVec.MakeUnitOrthoVectors(NULL, sZ, sX, sY) ;
      SmExtent1d sSweepIvl(0.0, dSweepAmount) ;
      SmCircle sSweepCircle(crSweepPoint, sX, sY, sSweepIvl, dSize/1.865, 3, cpContext) ;

      sDistToLineSols.Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; crSweepPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&crSweepPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(crSweepPoint) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sSweepCircle.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; sDistToLineSols.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when there are solutions - check if any of the nonIntersection maxima are tangent to the sweep direction = BadSweep
  //                            find all Sols where the curve is tangent to any Zrot = constant planes
  if(sDistToLineSols.GetSize() > 0)
    {
      sCurvePVs.SetSize(sDistToLineSols.GetSize()*2) ;
      SmVector3d *pPVArray = sCurvePVs.GetDataArray() ;

      // for every solution - see if any of the maxima are tangent to the sweep direction
      for(ii=0,i2=0;ii<sDistToLineSols.GetSize();ii++,i2+=2)
        {
          SmSolution &rSolution = sDistToLineSols[ii] ;
          double      dParam    = rSolution.m_vStart[0] ;
          double      dDist ;

          // evaluate curve at maxima
          crCurve.Evaluate(dParam, 1, TRUE, pPVArray+i2, TRUE) ;

          // find Line/Curve distance
          smgu_LinePointDistance(crSweepPoint, crSweepVec, sCurvePVs[i2], dDist) ;

          // when the solution is not an intersection point
          if(dDist > dDistTol3d)
            {
              // when sweep direction is parallel to the tangent direction
              // note: for translational sweep - IsPerpendicularTo() call becomes IsParallelTo().
              if(sCurvePVs[i2+1].IsPerpendicularTo(crSweepVec, SM_EFF_ZERO_DEG))
                {
                  rSweepState = SM_SC_SWEEP_ALONG_LENGTH ;

                  // all done
                  return(SM_SUCCESS) ;

                } // end tangent and sweep directions are parallel check
            } // end soluiton not an endPoint check
        } // end iter every maxima to rot axis solution

      // arrive here when curve has distance to rot axis maximas that are never parallal to the sweep direction
      // next - get any Z=const plane maxima - only curves with those can self-intersect
      //        if we have to, intersect projected curve segments and check any of those for a self-intersect condition

      // Get any places where the curve becomes tangent to the z=constant planes
      crCurve.GlobalPropertyAnalysis(crIvl,                  // in : ThisCurve target interval
                                     SM_CP_MAXIMA_TO_PLANE,  // in : The property of the curve to extract or find on the curve
                                     NULL,                   // in : Value used to specify a particular property.
                                     sOptVecs,               // in : Vectors or points used to define a particular property.
                                                             //      If there is a point, that should be the first value in the array
                                                             //      If there is a normal (projection), it will come before all other vectors
                                     dDistTol3d,             // in : 3D Tolerance used to determine when two answers are equivalent
                                     sDistToPlaneSols,       // out: SolArray of Points of tangency to Zrot=const planes
                                     NULL,                   // out: opt check comb: curve sample points
                                                             //      NULL to ignore, default:[NULL]
                                     NULL) ;                 // out: opt check comb: tines - lengths proportional to property being zeroed.
                                                             //      NULL to ignore, default:[NULL]
#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          const SmContext * cpContext = crCurve.GetContext() ;
          const SmEdge    * cpEdge    = crCurve.GetEdge() ;
          SmBrep          * pBrep     = cpEdge ? cpEdge->GetBrep() : NULL ;
          double            dSize     = crCurve.ApproximateLength(crIvl, 5) ;
          SmVector3d        sX, sY, sZ ;
          crSweepVec.MakeUnitOrthoVectors(NULL, sZ, sX, sY) ;
          SmExtent1d sSweepIvl(0.0, dSweepAmount) ;
          SmCircle sSweepCircle(crSweepPoint, sX, sY, sSweepIvl, dSize/1.865, 3, cpContext) ;

          sDistToPlaneSols.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1,0,0) ; crSweepPoint.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&crSweepPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(crSweepPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,0) ; sSweepCircle.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; sDistToPlaneSols.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // when there are solutions - this curve could possibly self intersect when rotated
      if(sDistToPlaneSols.GetSize() > 0)
        {
          // arrive here when the curve has both tangencies to the Rrot = constant cylinders and the
          // Zrot = constant planes.  It's possible that these curves can sweep into self intersecting swept surfaces

          // crSweepPoint,
          // crSweepVec,
          SmVector3d sProjVec, sY, sAuxData ;
          crSweepVec.MakeUnitOrthoVectors(NULL, sProjVec, sY, sAuxData) ;

          // project the curve back to a common rotation plane
          SmProjectedCurve sProjCurve(&crCurve, &crSweepPoint, &sProjVec, &sAuxData, SM_PT_ROTATION, FALSE, crCurve.GetContext()) ;

          // I see two ways to check for self intersections
          //    1.) Since we have the points where the curve becomes tangent to the Rrot=const cylinders
          //        and the Zrot = const planes we could do a sequence of intersections between the curve
          //        segments which can self-intersect. We skip looking for intersections in segments
          //        that can not intersect.  (A segment can only intersect another segment that is seperated
          //        by at least one tangency to a Rrot = const cylinder AND at least one tangency to a Zrot=const plane.)
          //    2.) We can pass sProjCurve to the GlobalCurveSelfIntersect() algorithm which breaks the curve
          //        into its subdivision segments and checks every subdivision segment against every other.
          //  In a curve case which has many points of tangency, approach 1 ends up making many intersection
          //  calls of various combinations of overlapping curve segments.  Approach 2 always looks at the
          //  whold curve.  I can't tell which approach would be more economical.  So, currently I'm
          //  going to try the approach with the least coding, approach 2.  Just seems a shame not to use
          //  all that tangent point information we gathered above to save more time here. (We already
          //  used that information to skip this section all together unless the curve was tangent to
          //  both the constant rotation cylinders and planes.)

          // Look for self intersections of the projected curves
          sProjCurve.GlobalCurveSelfIntersect(crIvl, dDistTol3d, sSelfXSectSols) ;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              const SmContext * cpContext = crCurve.GetContext() ;
              const SmEdge    * cpEdge    = crCurve.GetEdge() ;
              SmBrep          * pBrep     = cpEdge ? cpEdge->GetBrep() : NULL ;
              double            dSize     = crCurve.ApproximateLength(crIvl, 5) ;
              SmVector3d        sX, sY, sZ ;
              crSweepVec.MakeUnitOrthoVectors(NULL, sZ, sX, sY) ;
              SmExtent1d sSweepIvl(0.0, dSweepAmount) ;
              SmCircle sSweepCircle(crSweepPoint, sX, sY, sSweepIvl, dSize/1.865, 3, cpContext) ;

              sSelfXSectSols.Dump() ;

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,7, 1,0,0) ; crSweepPoint.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&crSweepPoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(crSweepPoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; sSweepCircle.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; sSelfXSectSols.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // work in positive angles
          if(dSweepAmount < 0) { dSweepAmount += 360 ; }

          // for every projected self-intersecting solution - see if intersection happens during sweep duration
          for(ii=0;ii<sSelfXSectSols.GetSize();ii++)
            {
              SmSolution &rSol = sSelfXSectSols[ii] ;

              // For point solutions
              if(rSol.m_eSolutionType == SM_ST_SINGLE_VALUE)
                {
                  // xsect params
                  double dParam0 = rSol.m_vStart[0] ;
                  double dParam1 = rSol.m_vStart[1] ;

                  // skip closed curve end-point solutions
                  if(   crIvl.IsValueOnBoundary(dParam0)
                     && crIvl.IsValueOnBoundary(dParam1))
                    {  continue ; }

                  // sxect 3d points
                  SmPoint3d sPoint0, sPoint1 ;
                  crCurve.EvaluatePoint(dParam0, sPoint0) ;
                  crCurve.EvaluatePoint(dParam1, sPoint1) ;

                  // vectors
                  SmVector3d sVec0 = sPoint0 - crSweepPoint ;
                  SmVector3d sVec1 = sPoint1 - crSweepPoint ;

                  // sweep distance between points
                  double dStartToEndAngRad ;   // in range [-Pi, +Pi]
                  sProjVec.CCWAngleBetween(sVec0, sVec1, dStartToEndAngRad) ;
                  double dStartToEndAngDeg = SM_RAD2DEG(dStartToEndAngRad) ;

                  // work in positive angles
                  if(dStartToEndAngDeg < 0) { dStartToEndAngDeg += 360 ; }

                  // When sweep is large enough to cause self-intersection return error
                  if(   dSweepAmount > dStartToEndAngRad         // start runs into end
                     || dSweepAmount + dStartToEndAngRad > 360)  // end runs into start
                    {
                       // attempting to rotate a curve through itself
                       rSweepState = SM_SC_SELF_INTERSECT ;

                       // all done
                       return(SM_SUCCESS) ;
                    }
                } // end SM_ST_SINGLE_VALUE branch
              else // SM_ST_RANGE_OF_VALUES solution branch
                {
                  // we need the minimum 3d rotation angle between the two segments
                  // whose intervals are coincident in projected space.

                  // we don't know the parameters of points which are coincident,
                  // just that each point in one interval maps to some point in the other interval
                  // through some-unknowned sweep amount.

                  // The only complete check I can think of is the expensive - create the sweep
                  // surface from one segment and do a curve/surface intersection with the other
                  // if that happens then we have an self-intersecting surface

                  // currently analytic class SmSurfOfRevolution is not as general as these sweep surfaces
                  // so work in the world of BSplines
                  SmBSplineCurve *pBSplineCurve = SM_CAST_NONNULL_PTR(SmBSplineCurve, &crCurve) ;

                  // when class SmSufOfRevolution is fully extended (with this very CheckCurveSweep() code)
                  // work directly with the SmCurve in the next section and remove the BSpline dependency
                  if(pBSplineCurve == NULL)
                    {
                      WARN(_T("SmRotationalSweepGeometry::CheckCurveSweep found a NonBSpline projected coincident segment - case not supported until SmSurOfRevolution is extended")) ;
                      rSweepState = SM_SC_UNKNOWN ;

                      // all done
                      return(SM_SUCCESS) ;
                    }

                  // locals for temps
                  SmContext       sContext ;
                  SmSolutionArray sSurfCurveXSects ;
                  SmExtent1d      sSegIvl0(rSol.m_vStart[0], rSol.m_vEnd[0]) ;
                  SmExtent1d      sSegIvl1(rSol.m_vStart[1], rSol.m_vEnd[1]) ;

                  // copy the curve into its segments
                  SmBSplineCurve sSegment0(*pBSplineCurve) ; sSegment0.SetContext(pBSplineCurve->GetContext()) ;
                  SmBSplineCurve sSegment1(*pBSplineCurve) ; sSegment1.SetContext(pBSplineCurve->GetContext()) ;

                  // trim the segments to their intervals
                  sSegment0.Trim(sSegIvl0, FALSE) ;  // may snap sIvl by tol to existing knots
                  sSegment1.Trim(sSegIvl1, FALSE) ;  // may snap sIvl by tol to existing knots

                  // create the surface of revolutions from each segment
                  SmBSplineSurface *pSweptSegment0 = NULL ;
                  SmBSplineSurface *pSweptSegment1 = NULL ;
                  SmBSplineSurface::CreateSurfOfRevolution(sContext, &sSegment0, crSweepPoint, sProjVec, dSweepAmount, pSweptSegment0) ;
                  SmBSplineSurface::CreateSurfOfRevolution(sContext, &sSegment1, crSweepPoint, sProjVec, dSweepAmount, pSweptSegment1) ;
                  SmObjDelete sClean0(pSweptSegment0) ;
                  SmObjDelete sClean1(pSweptSegment1) ;

                  // intersect sweptSegment0 with Segment1 - any intersections are failures
                  pSweptSegment0->GlobalCurveIntersect(pSweptSegment0->GetNaturalUVDomain(),
                                                       sSegment1, sSegIvl1, dDistTol3d,
                                                       sSurfCurveXSects) ;

                  // any intersections are failures
                  if(sSurfCurveXSects.GetSize() > 0)
                    {
                       rSweepState = SM_SC_SELF_INTERSECT ;

                      // all done
                      return(SM_SUCCESS) ;
                    }

                  // intersect sweptSegment1 with Segment0 - any intersections are failures
                  pSweptSegment1->GlobalCurveIntersect(pSweptSegment1->GetNaturalUVDomain(),
                                                       sSegment0, sSegIvl0, dDistTol3d,
                                                       sSurfCurveXSects) ;

                  // any intersections are failures
                  if(sSurfCurveXSects.GetSize() > 0)
                    {
                       rSweepState = SM_SC_SELF_INTERSECT ;

                      // all done
                      return(SM_SUCCESS) ;
                    }

                } // end SM_ST_RANGE_OF_VALUES solution branch
            } // end iter every selfXSect solution


        } // end curve also has z=const plane maximas (curve is tangent to one of these planes in one or more places)
    } // end curve has r=const cylinder maximas (curve is tangent to one of these cylinders in one or more places)

#endif // SM_SWEEP_CHECKS

  // arrive here after passing all tests
  rSweepState = SM_SC_OKAY ;

  // all done
  return SM_SUCCESS ;

} // end SmRotationalSweepGeometry::CheckCurveSweep

/*******************************************************************//**
PURPOSE: Constructor for a rotational sweep.

NOTES:
***********************************************************************/
SmRotationalSweepGeometry::SmRotationalSweepGeometry
 (const SmPoint3d  & crCenterArg,
  const SmVector3d & crAxisArg,
  double             dAngleArg,     //
                                    // A positive number between 0->360.
                                    // Difference from 0 and 2pi must be
                                    // significant.
                                    // For negative angles, invert
                                    // the axis
  double             dEpsArg)       // for checking input
: SmSweepGeometryCreation(dEpsArg),
  m_sBasePt(crCenterArg),
  m_sAxis(crAxisArg),
  m_dAngle(dAngleArg),              // degrees
  m_dEps(dEpsArg)
{
  // sorting out over-degenerate input:
  SmStatus eStat = m_sAxis.Unitize();
  SM_ASSERT( eStat == SM_SUCCESS );

  SM_ASSERT( 0.0 <= dAngleArg && dAngleArg <= 360 );

  double dSmallAngle = 0.0001;

  if (dAngleArg != 0.0)
    {
      SM_ASSERT( dAngleArg > dSmallAngle )
    }
  if (dAngleArg != 360.)
    {
      double dDiscr = smos_Fabs( 360. - dAngleArg);
      SM_ASSERT(dDiscr > dSmallAngle);
    }

} // end SmRotationalSweepGeometry constructor

/*******************************************************************//**
PURPOSE: Copy a RotationalSweepGeometry object.

NOTES:
***********************************************************************/
SmSweepGeometryCreation * SmRotationalSweepGeometry::Copy()
{
  SmRotationalSweepGeometry *pNew = new SmRotationalSweepGeometry(m_sBasePt,
                                                                  m_sAxis,
                                                                  m_dAngle,
                                                                  m_dEps );
  pNew->SetMakeAnalytics( m_bMakeAnalytics );

  return pNew;

} // end SmRotationalSweepGeometry::Copy

/*******************************************************************//**
PURPOSE: Return a coordinate system that is the start of the sweep.

NOTES: Returns SM_ERR if the given point is right on the rotation axis.
***********************************************************************/
SmStatus SmRotationalSweepGeometry::GetStartCoordSystem
  (const SmPoint3d  & rPt,       // in :
   SmAxis2Placement & rCoordSys) // out:
 const
{
  // Get the vector from the given point to the axis, perpendicular.
  SmVector3d sDiffVec( rPt - m_sBasePt );
  double dDot = sDiffVec.Dot( m_sAxis );
  SmPoint3d sPtOnAxis( m_sBasePt + dDot * m_sAxis );
  sDiffVec = rPt - sPtOnAxis;

  double dScaledZero = SM_EFF_ZERO * ( 1.0 + rPt.GetMaxDimension() );
  double dLen = sDiffVec.Length();
  if ( dLen < dScaledZero )
    { return SM_ERR; }

  sDiffVec /= dLen;

  rCoordSys.SetCanonical( rPt, m_sAxis, sDiffVec );

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::GetStartCoordSystem

/*******************************************************************//**
PURPOSE: Return a coordinate system that is the end of the sweep.

NOTES: Returns SM_ERR if the given point is right on the rotation axis.
***********************************************************************/
SmStatus SmRotationalSweepGeometry::GetEndCoordSystem
  (const SmPoint3d  & rPt,       // in :
   SmAxis2Placement & rCoordSys) // out:
 const
{
  // Get the start C.S. and rotate by m_dAngle about m_sBasePt/m_sAxis.
  SER( GetStartCoordSystem( rPt, rCoordSys ));

  // No need to rotate by 360.
  if ( m_dAngle < 360 - SM_EFF_ZERO )
    { rCoordSys.RotateAboutAxisAtPoint( SM_DEG2RAD( m_dAngle ), m_sBasePt, m_sAxis ); }

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::GetEndCoordSystem

/*******************************************************************//**
PURPOSE: Return the transform from start to end of the sweep.

NOTES:
***********************************************************************/
SmStatus SmRotationalSweepGeometry::GetSweepTransform
  (SmAxis2Placement & rCoordSys) // out:
 const
{
  rCoordSys.Init();

  // No need to rotate by 360.
  if ( m_dAngle < 360 - SM_EFF_ZERO )
    { rCoordSys.RotateAboutAxisAtPoint( SM_DEG2RAD( m_dAngle ), m_sBasePt, m_sAxis ); }

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::GetSweepTransform

/*******************************************************************//**
PURPOSE: Create a curve which represents the sweeping of a
     vertex into higher dimension.

NOTES:
***********************************************************************/
SmStatus SmRotationalSweepGeometry::VertexSweepHigher
  (const SmContext & crContext,
   const SmVertex  * pOriginalVertexArg,
   SmCurve        *& rpNewCurveArg,
   double          & rdNewEdgeToleranceArg)
  const
{
  SmPoint3d sOriginalPt( pOriginalVertexArg->GetPoint() );

  double dOrigVertexTolerance = pOriginalVertexArg->GetTolerance();

  double dRotCenterPar = m_sAxis.Dot(sOriginalPt - m_sBasePt);
  SmPoint3d sRotCenter( m_sBasePt + dRotCenterPar * m_sAxis);

  double   dEps = smos_Max(m_dEps, dOrigVertexTolerance);

  SmVector3d sXVect(sOriginalPt - sRotCenter);
  if (sXVect.Length() < dEps )
    {
      // point lies on axis
      rpNewCurveArg = 0;
      return SM_SUCCESS;
    }
  SER(sXVect.Unitize());

  SmVector3d sYVect( sXVect * m_sAxis );
  double dDiscr = sYVect.Length(); // sin angle between axis and center-to-po
                                   // also distance between point and axis
  if (dDiscr < dEps)
    {
      // point lies on axis
      rpNewCurveArg = 0;
      return SM_SUCCESS;
    }
  SER(sYVect.Unitize());

  SmVector3d sZVect( sXVect * sYVect );

  if (sZVect.Dot(m_sAxis) < 0.0 )
      sYVect = -sYVect;

  double dRad = (sRotCenter - sOriginalPt).Length();

  SmAxis2Placement sA2P;
  sA2P.SetCanonical(sRotCenter,sXVect,sYVect);

  SmNurbCircleParam eParameterization = SM_CO_QUADRATIC;  // degree 2
  // The circle generator can cope all angles.
  // If there is need, SetParameterization() member service can be
  // introduced for the user to be able to change it.

  SmBSplineCurve *pCircle = NULL;
  SER(SmBSplineCurve::CreateCircleSegment(crContext,
                                          3,         // lDimensionOfResult
                                          sA2P,      // ReferenceFrame
                                          dRad,
                                          0.0,       // dStartAngle
                                          m_dAngle,  //dEndAngle grad
                                          eParameterization, // degree of nurbc
                                          pCircle));

  NER(pCircle); // If circle not created error out

  rpNewCurveArg = pCircle;
  rdNewEdgeToleranceArg = dOrigVertexTolerance;

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::VertexSweepHigher

/*******************************************************************//**
PURPOSE: Create a point which represents the sweep of a vertex to the
    same dimension.

NOTES:
***********************************************************************/
SmStatus SmRotationalSweepGeometry::VertexSweepSame
 (const SmVertex * pOriginalVertexArg,
  SmPoint3d      & rPointArg,
  double         & rdNewPointToleranceArg)
 const
{
  SmBoolean bSweepIsClosed = FALSE;
  SER( IsSweepClosed(1, bSweepIsClosed));

  if (bSweepIsClosed)
    {
      rPointArg = pOriginalVertexArg->GetPoint();
      rdNewPointToleranceArg = pOriginalVertexArg->GetTolerance(); // GWC:12/11/04
      return SM_SUCCESS;
    }

  SmPoint3d sOriginalPt( pOriginalVertexArg->GetPoint() );
  SmPoint3d sCopiedPt(sOriginalPt);

  double dAngleRad = m_dAngle * SM_PI / 180.;

  SmAxis2Placement sRefFrame;
  sRefFrame.RotateAboutAxisAtPoint(dAngleRad, m_sBasePt, m_sAxis);
  sRefFrame.TransformPoint(sCopiedPt, sCopiedPt);

  rPointArg = sCopiedPt;
  rdNewPointToleranceArg = pOriginalVertexArg->GetTolerance(); // GWC:12/11/04

  double dDi = (rPointArg - pOriginalVertexArg->GetPoint()).Length();
  if (dDi < pOriginalVertexArg->GetTolerance() )
    {
      SER(SM_ERR); // This is a problem for now
    }

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::VertexSweepSame

/*******************************************************************//**
PURPOSE: Create a new BSplineSurface by revolving the given curve about
      the given revolution axis described by a point and an axis vector.

NOTES: currently, pCurveArg must be of Type:[SmBSplineSurface]
***********************************************************************/
static SmStatus sm_CreateRotSrf
  (const SmContext                 & crContext,               // in : context for new geometry creation
   SmCurve                         * pCurveArg,               // in : generator curve - currently must be TYPE:[SmBSplineSurface]
   const SmPoint3d                 & rBasePtArg,              // in : rotation point
   const SmVector3d                & rAxisArg,                // in : rotation axis
   double                            dAngleDeg,               // in : rotation angle
   double                            dEpsArg,                 // NotUsed: in : 
   const SmRotationalSweepGeometry * pGeomCreatorArg,         // NotUsed: in : 
   SmVertex                        * pStartVtxArg,            // NotUsed: in : 
   SmVertex                        * pEndVtxArg,              // NotUsed: in : 
   SmBSplineSurface               *& pNewSurfArg,             // out: the new surface
   SmSurfParamType                 & eStartTrimCurveParamArg) // out: one of SM_SP_U or SM_SP_V (always SM_SP_U)
{
  SM_REF4(dEpsArg, pGeomCreatorArg, pStartVtxArg, pEndVtxArg) ;
  // init output
  pNewSurfArg = NULL;

  // only support BSpline Curves for now.  GWC? should switch to SmSurfOfRevolution
  SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,pCurveArg);
  NER(pBSC); // for the time being this works only for BSplineCurves

  // pass the call along
  SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,      // in : new object context
                                               pBSC,           // in : Generating curve
                                               rBasePtArg,     // in : Origin of the axis of revoluation
                                               rAxisArg,       // in : Direction vector of axis of revolution
                                               dAngleDeg,      // in : Angle of revolution (> 0.0 && <= 360.0) in degrees
                                               pNewSurfArg,    // out: The new BSpline Surface
                                               FALSE));        // in : The U direction (constant V iso curves)
                                                               //      will be circular degree 2 if TRUE
                                                               //      If FALSE the V direction will be circular degree 2

  // set output
  eStartTrimCurveParamArg = SM_SP_U;

  // all done
  return SM_SUCCESS;

} // end sm_CreateRotSrf

/*******************************************************************//**
PURPOSE: Create geometry for sweeping an SmEdge into higher dim.
      This function creates and outputs a new Sweep Surface and a set
      of new Sweep Boundary Curves that can later be used to create a
      swept face with the method SmBrep::MakeFaceFromCurves().

WARNING: Given inappropriate input, this method returns SM_ERR without
         creating any output, and without modifying any of the input.

NOTES:
***********************************************************************/
SmStatus SmRotationalSweepGeometry::EdgeSweepHigher
 (const SmContext             & crContext,              // in : context for new object construction
  const SmEdge                * pEdgeToSweep,           // in : Edge to be swept to a higher dimension
  SmSurface                  *& rpNewSurface,           // out: Surface created by the sweep of the edge
  double                      & rdNewFaceTol,           // out: Tolerance of the face - derived from edge tolerance.
  SmCurve                    *& rpNewStartCurve,        // out: Curve Copy in orig position trimmed to edge ivl.
  double                      & rdNewStartCurveTol,     // out: Tolerance of the new start curve. value:[pEdgeArg->Tol]
  SmCurve                    *& rpNewFarEndCurve,       // out: Curve copy moved to end sweep position.
  double                      & rdNewEdgeTol,           // out: Tolerance of this new edge. value:[pEdgeArg->Tol]
  SmCurve                    *& rpStartVertCurve,       // out: start vertex sweep curve.
  double                      & rdNewStartVertEdgeTol,  // out: start vertex sweep curve tolerance.
  SmCurve                    *& rpEndVertCurve,         // out: end vertex sweep curve.
  double                      & rdNewEndVertEdgeTol,    // out: end vertex sweep curve tolerance.
  SmTArray<SmCurve*>          & r3DTrimmingCurves,      // out: ordered new 3D TrimCurves (ptrs to previously output curves)
  SmTArray<SmBSplineCurve*>   & rUVTrimmingCurves,      // out: associated new UV TrimCurves when easy, not built when expensive.
  SmTArray<SmOrientType>      & rOrients,               // out: associated TrimCurve Orients for a valid outer loop. SM_OT_SAME, SM_OT_OPPOSITE
  SmVector3d                  & rsNewSurfaceNormal)     // out: The surface normal of the surface at the start vertex.
 const
{
  // Init outputs
  rpNewStartCurve  = NULL;
  rpNewFarEndCurve = NULL;
  rpStartVertCurve = NULL;
  rpEndVertCurve   = NULL;

  r3DTrimmingCurves.ReSet();
  rUVTrimmingCurves.ReSet();
  rOrients.ReSet();

  rpNewSurface = NULL;

  rdNewFaceTol          = -1.0;
  rdNewStartCurveTol    = -1.0;
  rdNewEdgeTol          = -1.0;
  rdNewStartVertEdgeTol = -1.0;
  rdNewEndVertEdgeTol   = -1.0;

  NER(pEdgeToSweep);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
  if (bDebugMe1)
    {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 2,5, 0,0,1 ); m_sAxis.Draw( &m_sBasePt ); sm_GraphicsLoop();
      smgfx_SetLook( 2,5, 1,0,0 ); pEdgeToSweep->Draw();   sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  SmCurve    * pOriginalEdgeCurve = pEdgeToSweep->GetCurve();
  SmExtent1d   sOriginalEdgeIvl   = pEdgeToSweep->GetInterval();
  SmBrep     * pBrep              = pEdgeToSweep->GetBrep();
  double       dBrepTol           = pBrep->GetTolerance();
  double       dEps               = smos_Max(m_dEps, dBrepTol);

  // check for input line
  SmBoolean bIsLine;
  SER(sm_CheckEdgeCurveLine(pOriginalEdgeCurve,
                            sOriginalEdgeIvl,
                            dEps,
                            bIsLine));

  // check edge on axis
  SmBoolean bOnEdge;
  SER( sm_CheckEdgeOnAxis(m_sBasePt, m_sAxis,
                          pEdgeToSweep,
                          bIsLine,
                          dEps,
                          bOnEdge));

  // no work - edge is on axis and it's not transformed by the rotation
  if (bOnEdge)
    { return SM_SUCCESS; }

  // GWC: to be added when we have time to debug all the code written for this case
  // check to see if the curve will sweep into a valid surface
  //   added for Bug 317 - but always useful
  SmSweepCheckType eSweepCheckType ;
  SER( CheckCurveSweep(*pOriginalEdgeCurve, sOriginalEdgeIvl, dEps, m_sBasePt, m_sAxis, m_dAngle, eSweepCheckType) ) ;

  if(eSweepCheckType != SM_SC_OKAY)
    {
      // Inform the user of the problem
      switch(eSweepCheckType)
        { case SM_SC_ROT_ON_CURVE :
            smos_WriteBuffer(_T("Tried to Revolve a Curve about a point on the curve. Split curve at point and try again")) ;
            break ;
          case SM_SC_SWEEP_ALONG_LENGTH :
            smos_WriteBuffer(_T("Tried to Sweep Curve tangent to the sweep direction. Sweep can't work - would build a degenerate surface")) ;
            break ;
          case SM_SC_SELF_INTERSECT :
            smos_WriteBuffer(_T("Tried to Sweep Curve through itself. Sweep can't work - would build a self-intersecting surface")) ;
            break ;
          case SM_SC_UNKNOWN :
            smos_WriteBuffer(_T("CurveSweepCheck failed because a nonBSpline curve might sweept through itself.  NonBSpline curves are not yet fully supported")) ;
            break ;
          default:
            break;
        }
      return(SM_ERR) ;
    }

  // StartVertex: According to the CURVE, the edge may be inverted
  // This selection ensures that the rpNewFarEndCurve is the same as it
  // would be obtained from SweepEdgeSame.
  // But it matters only when finding the rpStartVertCurve.
  SmVertex   * pStartVtx = pEdgeToSweep->GetStartVertex();
  SmVertex   * pEndVtx   = pEdgeToSweep->GetOtherVertex(pStartVtx);
  SmPoint3d    sStartPt( pStartVtx->GetPoint() );
  SmPoint3d    sEndPt  (   pEndVtx->GetPoint() );

  double dStVtxTol = pStartVtx->GetTolerance();
  double dEndVtxTol = pEndVtx->GetTolerance();
  double dOrigEdgeTol = pEdgeToSweep->GetTolerance();

  // Have to copy the original edge curve because we have to trim it.
  // It will be used only as the generator curve for the
  // RotationalSweepSurface creation, thus can be deleted after
  // the operation (we use the U/VMin curve as the copied edge curve
  // instead of this curve).
  SmCurve* pCopiedOrigEdgeCurve;
  SER(pOriginalEdgeCurve->Copy(crContext,pCopiedOrigEdgeCurve));

  SER(pCopiedOrigEdgeCurve->Trim(sOriginalEdgeIvl));  // may snap sIvl by tol to existing knots

  SmObjDelete sCleanup1(pCopiedOrigEdgeCurve);

  SmBoolean bStartOn, bEndOn, bParallel, bPerp;
  SER(sm_CheckEdgeVerticesOnAxis(m_sBasePt,
                                 m_sAxis,
                                 pEdgeToSweep,
                                 bIsLine,
                                 dEps,
                                 bStartOn,
                                 bEndOn,
                                 bParallel,
                                 bPerp));

  SmBSplineSurface * pNewSurf    = NULL;
  SmBSplineSurface * pSurfToTrim = NULL;

  SmSurfParamType eStartTrimCurveParam = SM_SP_U;
      // This setting seems to be needed for CreateLinearSweep:
      // I assume it is what's needed for SurfOfRevolution too.
  SmBoolean bIsPlane = FALSE;
  if (bIsLine && bPerp)
    {
      // Creates the surface by SmBSplineSurface::CreateSurfOfRevolution,
      // (Should be using SmSurfOfRevolution if SmSurfOfRevolution can be more general than coplanar generator and rotation axis curves)
      SER(sm_CreateRotSrf(crContext,                // in : context for new geometry creation
                          pCopiedOrigEdgeCurve,     // in : generator curve - currently must be type:[SmBSplineCurve]
                          m_sBasePt,                // in : rotation point
                          m_sAxis,                  // in : rotation axis
                          m_dAngle,                 // in : rotation angle
                          dEps,                     // in : not used
                          this,                     // in : not used
                          pStartVtx,                // in : not used
                          pEndVtx,                  // in : not used
                          pSurfToTrim,              // out: the new surface
                          eStartTrimCurveParam));   // out: one of SM_SP_U or SM_SP_V (always SM_SP_U)

      double dStartVRad, dEndVRad;
      SER(smgu_LinePointDistance(m_sBasePt,m_sAxis,sStartPt,dStartVRad));
      SER(smgu_LinePointDistance(m_sBasePt, m_sAxis, sEndPt, dEndVRad));

      double dLineStartVPar, dLineEndVPar;
      SER(smgu_LineClosestPoint(m_sBasePt, m_sAxis,
          sStartPt, dLineStartVPar));
      SER(smgu_LineClosestPoint(m_sBasePt,m_sAxis,sEndPt, dLineEndVPar));

      // These 2 coincide within eps:
      SmPoint3d sStartVBasePt(m_sBasePt + m_sAxis * dLineStartVPar);
      SmPoint3d   sEndVBasePt( m_sBasePt + m_sAxis * dLineEndVPar);

      // Remaining on the safe side:
      double dDiscr = (sStartVBasePt - sEndVBasePt).Length();
      if (dDiscr > dEps) { SER(SM_ERR); }

      SmPoint3d sRotCenter( 0.5*(sStartVBasePt + sEndVBasePt));

      // Could use any of start/end vertices, but the farther one
      // gives more accurate results (the other may not give at all):
      SmVector3d sXVect;
      if (dStartVRad < dEndVRad) { sXVect = sEndPt - sEndVBasePt; }
      else { sXVect = sStartPt - sStartVBasePt; }

      SmVector3d sZDir( m_sAxis);
      SER(sZDir.Unitize()); // perhaps it is unit already

      SmVector3d sYVect( sZDir * sXVect );

      sXVect = 1.1 * sXVect;
      sYVect = 1.1 * sYVect;

      // The 4 corners of the plane (swollen a bit to avoid numerics)
      SmPoint3d sPlaneUV00( sRotCenter - sXVect - sYVect);
      SmPoint3d sPlaneUV10( sRotCenter + sXVect - sYVect);

      SmPoint3d sPlaneUV01( sRotCenter - sXVect + sYVect);
      SmPoint3d sPlaneUV11( sRotCenter + sXVect + sYVect);

      SER(SmBSplineSurface::CreateBilinearSurface(crContext,
          sPlaneUV00, sPlaneUV01, sPlaneUV10, sPlaneUV11, pNewSurf));

      NER(pNewSurf); // the creator must have coped
      NER(pSurfToTrim); // the creator must have coped
      bIsPlane = TRUE;
//        eStartTrimCurveParam = SM_SP_V; // base curve: Vmin (u0->1, v0)
    }
  else
    {
      // Creates the surface by SmBSplineSurface::CreateSurfOfRevolution,
      // (Should be using SmSurfOfRevolution except SmSurfOfRevolution needs to be more general than coplanar generator curves)
      SER(sm_CreateRotSrf(crContext,                // in : context for new geometry creation
                          pCopiedOrigEdgeCurve,     // in : generator curve
                          m_sBasePt,                // in : rotation point
                          m_sAxis,                  // in : rotation axis
                          m_dAngle,                 // in : rotation angle
                          dEps,                     // in : not used
                          this,                     // in : not used
                          pStartVtx,                // in : not used
                          pEndVtx,                  // in : not used
                          pNewSurf,                 // out: the new surface
                          eStartTrimCurveParam));   // out: one of SM_SP_U or SM_SP_V (always SM_SP_U)
    }

  if (pNewSurf==NULL)
    {
      // m_dAngle must be small, but how small?
      return SM_SUCCESS;
    }

  // If the function fails, delete the new surface
  SmObjDelete sCleanNewSurf(pNewSurf);

  SmExtent2d sSurfDomain( pNewSurf->GetNaturalUVDomain() );
  SmPoint2d sStartUV = sSurfDomain.GetMin();
  SER(pNewSurf->EvaluateNormal(sStartUV,
      TRUE,TRUE,
      rsNewSurfaceNormal));


  SmBoolean bFlipOrients = FALSE;
  if (pSurfToTrim)
    {
      SmVector3d sPlaneNormal;
      SmExtent2d sDomain( pSurfToTrim->GetNaturalUVDomain() );
      SmPoint2d pStartParam = sDomain.GetMin();
      SER(pSurfToTrim->EvaluateNormal( pStartParam,
          TRUE,TRUE,
          sPlaneNormal));
      double dDot = rsNewSurfaceNormal.Dot(sPlaneNormal);
      if (smos_Fabs(dDot) < 0.8) { SER(SM_ERR); }
      if (dDot < 0.0)
          bFlipOrients = TRUE;
    }


  // Create corresponding analytics if requested
  if (m_bMakeAnalytics)
    {
      SmSurface *pBSS;
      sCleanNewSurf.Clear();
      SmObjDelete sClean3(pNewSurf);

      // Copy pNewSurf, when possible as an analytic surface
      SER(pNewSurf->CopyAndAddAnalytics(crContext,pBSS));
      pNewSurf = SM_CAST_PTR(SmBSplineSurface,pBSS);
      sCleanNewSurf.SetObj(pNewSurf);
    }

  // eStartTrimCurveParam fixes which of the umin/vmin line
  // of the surface represents the copy of the edge.
  // The bSwapStartEnd flag setting will fail in most of the cases
  // if we made a mistake.
  SmTArray<SmCurve*> a3DTrimmingCurves;
  SmObjsDelete<SmCurve*> sClean3D(&a3DTrimmingCurves);
  SmTArray<SmBSplineCurve*> aUVTrimmingCurves;
  SmObjsDelete<SmBSplineCurve*> sCleanUV(&aUVTrimmingCurves);

  SmTArray<SmOrientType> aOrients;
  SmBSplineSurface* pS = (pSurfToTrim == NULL) ? pNewSurf : pSurfToTrim;
  SER(pS->CreateNaturalUVTrimCurves(
      crContext,
      pS->GetNaturalUVDomain(),
      eStartTrimCurveParam,
      FALSE,
      a3DTrimmingCurves,
      aUVTrimmingCurves,
      aOrients));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2)
    {
      a3DTrimmingCurves[0]->Dump();
      a3DTrimmingCurves[1]->Dump();
      a3DTrimmingCurves[2]->Dump();
      a3DTrimmingCurves[3]->Dump();
      smgfx_Erase();
      a3DTrimmingCurves[0]->Draw();
      sm_GraphicsLoop();
      a3DTrimmingCurves[1]->Draw();
      sm_GraphicsLoop();
      a3DTrimmingCurves[2]->Draw();
      sm_GraphicsLoop();
      a3DTrimmingCurves[3]->Draw();
      sm_GraphicsLoop();
    }
#endif


  SmCurve* pEC = a3DTrimmingCurves[0];
  SmPoint3d sCurveStartPt, sCurveEndPt;
  SER(pEC->EvaluatePoint(pEC->GetNaturalInterval().GetMin(), sCurveStartPt));
  SER(pEC->EvaluatePoint(pEC->GetNaturalInterval().GetMax(), sCurveEndPt));

  // The edge curve may be oriented arbitrarily wrt edge
  SmBoolean bSwapStartEnd = FALSE;
  double dStSt = (sCurveStartPt - sStartPt).Length();
  double dStEnd = (sCurveStartPt - sEndPt).Length();
  double dEndSt = (sCurveEndPt - sStartPt).Length();
  double dEndEnd = (sCurveEndPt - sEndPt).Length();
  if (dStEnd < dStSt)
    {
      bSwapStartEnd = TRUE;
      if (dStEnd > dEps) { SER(SM_ERR); }
      if (dEndSt > dEps) { SER(SM_ERR); }
    }
  else
    {
      if (dEndEnd > dEps) { SER(SM_ERR); }
      if (dStSt   > dEps) { SER(SM_ERR); }
    }

  // With the purpose to orient the loop correctly,
  // here we invert the orientations, swap the side curves,
  // and flip the flag too.
  if (bFlipOrients )
    {
      bSwapStartEnd = (bSwapStartEnd==TRUE) ? FALSE : TRUE;
      aOrients.SetAt(0, SM_REVERSE_ORIENTATION(aOrients[0]));
      aOrients.SetAt(1, SM_REVERSE_ORIENTATION(aOrients[1]));
      aOrients.SetAt(2, SM_REVERSE_ORIENTATION(aOrients[2]));
      aOrients.SetAt(3, SM_REVERSE_ORIENTATION(aOrients[3]));

      SmCurve* pC = a3DTrimmingCurves[1];
      a3DTrimmingCurves.SetAt(1, a3DTrimmingCurves[3]);
      a3DTrimmingCurves.SetAt(3, pC);

      SmOrientType eOR = aOrients[1];
      aOrients.SetAt(1, aOrients[3]);
      aOrients.SetAt(3, eOR);

      if (pSurfToTrim == NULL)
        {
          SmBSplineCurve* pBSC = aUVTrimmingCurves[1];
          aUVTrimmingCurves.SetAt(1, aUVTrimmingCurves[3]);
          aUVTrimmingCurves.SetAt(3, pBSC);
        }
    }


  // Now, create the results:
  SmBoolean bIsDegenerate = FALSE;

  //------0----- processing the base curve
  r3DTrimmingCurves.Add(a3DTrimmingCurves[0]);
  rOrients.Add(aOrients[0]);
  if (pSurfToTrim == NULL) { rUVTrimmingCurves.Add(aUVTrimmingCurves[0]); }
  rpNewStartCurve = r3DTrimmingCurves[0];
  rdNewStartCurveTol = dOrigEdgeTol;

  //----1----- the first side curve (return NULL if degenerate):
  // may correspond to either startV or endV,
  // but that does not affect the r3D and rUV
  SmCurve* pC = a3DTrimmingCurves[1];
  SmBSplineCurve* pUV = aUVTrimmingCurves[1];
  if (pC->IsDegenerate(rdNewStartCurveTol))
    {
      bIsDegenerate = TRUE;
     SM_ASSERT(pC  != NULL) ; delete pC ;  pC  = NULL ;
     SM_ASSERT(pUV != NULL) ; delete pUV ; pUV = NULL ;
     pC = NULL;
     pUV = NULL;
     a3DTrimmingCurves[1] = NULL;
     aUVTrimmingCurves[1] = NULL;
    }
  else
    {
      r3DTrimmingCurves.Add(pC);
      rOrients.Add(aOrients[1]);
      if (pSurfToTrim == NULL) { rUVTrimmingCurves.Add(pUV); }
    }

  if (bSwapStartEnd)
    {
      rdNewEndVertEdgeTol = dEndVtxTol;
      rpEndVertCurve = pC;
    }
  else
    {
      rdNewStartVertEdgeTol = dStVtxTol;
      rpStartVertCurve = pC;
    }

  //----2----- the swept base curve
  r3DTrimmingCurves.Add(a3DTrimmingCurves[2]);
  rOrients.Add(aOrients[2]);
  if (pSurfToTrim == NULL) { rUVTrimmingCurves.Add(aUVTrimmingCurves[2]); }
  rdNewEdgeTol = dOrigEdgeTol;
  if (m_dAngle != 360.0)
    {
      rpNewFarEndCurve = a3DTrimmingCurves[2];
    }
  else
    {
      rpNewFarEndCurve = NULL;
    }

  //----3----- the second side curve (return NULL if degenerate):
  pC = a3DTrimmingCurves[3];
  pUV = aUVTrimmingCurves[3];
  if (pC->IsDegenerate(rdNewStartCurveTol))
    {
      bIsDegenerate = TRUE;
     SM_ASSERT(pC  != NULL) ; delete pC ;  pC  = NULL ;
     SM_ASSERT(pUV != NULL) ; delete pUV ; pUV = NULL ;
     pC = NULL;
     pUV = NULL;
     a3DTrimmingCurves[3] = NULL;
     aUVTrimmingCurves[3] = NULL;
    }
  else
    {
      r3DTrimmingCurves.Add(pC);
      rOrients.Add(aOrients[3]);
      if (pSurfToTrim == NULL) { rUVTrimmingCurves.Add(pUV); }
    }

  if (!pEdgeToSweep->IsClosed())
    {
      if (!bSwapStartEnd)
        {
          rdNewEndVertEdgeTol = dEndVtxTol;
          rpEndVertCurve = pC;
        }
      else
        {
          rdNewStartVertEdgeTol = dStVtxTol;
          rpStartVertCurve = pC;
        }
    }

  // If we have a degenerate point and a plane we can trim it with just a circle.
  if (bIsDegenerate && bIsPlane && m_dAngle > 360.0 - SM_EFF_ZERO_SQRT)
    {
      rpNewStartCurve = NULL;
      SM_ASSERT(r3DTrimmingCurves[0] != NULL) ; delete r3DTrimmingCurves[0] ; r3DTrimmingCurves[0] = NULL ;
      r3DTrimmingCurves.RemoveAt(0);
      if (pSurfToTrim == NULL)
        {
          SM_ASSERT(rUVTrimmingCurves[0] != NULL) ; delete rUVTrimmingCurves[0] ; rUVTrimmingCurves[0] = NULL ;
          rUVTrimmingCurves.RemoveAt(0);
        }
      rOrients.RemoveAt(0);
      if ((rpStartVertCurve == NULL && !bSwapStartEnd) ||
          (rpEndVertCurve == NULL && bSwapStartEnd))
        {
          SM_ASSERT(r3DTrimmingCurves[0] != NULL) ; delete r3DTrimmingCurves[0] ; r3DTrimmingCurves[0] = NULL ;
          r3DTrimmingCurves.RemoveAt(0);
          if (pSurfToTrim == NULL)
            {
              SM_ASSERT(rUVTrimmingCurves[0] != NULL) ; delete rUVTrimmingCurves[0] ; rUVTrimmingCurves[0] = NULL ;
              rUVTrimmingCurves.RemoveAt(0);
            }
          rOrients.RemoveAt(0);
          rpNewFarEndCurve = NULL;
        }
      else
        {
          SM_ASSERT(r3DTrimmingCurves.GetLast() != NULL) ; delete r3DTrimmingCurves.GetLast() ;
          r3DTrimmingCurves.RemoveLast();
          if (pSurfToTrim == NULL)
            {
              SM_ASSERT(rUVTrimmingCurves.GetLast() != NULL) ; delete rUVTrimmingCurves.GetLast() ;
              rUVTrimmingCurves.RemoveLast();
            }
          rOrients.RemoveLast();
          rpNewFarEndCurve = NULL;
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3)
    {
      r3DTrimmingCurves[0]->Dump();
      r3DTrimmingCurves[1]->Dump();
      r3DTrimmingCurves[2]->Dump();
      r3DTrimmingCurves[3]->Dump();
      smgfx_Erase();
      r3DTrimmingCurves[0]->Draw();
      sm_GraphicsLoop();
      r3DTrimmingCurves[1]->Draw();
      sm_GraphicsLoop();
      r3DTrimmingCurves[2]->Draw();
      sm_GraphicsLoop();
      r3DTrimmingCurves[3]->Draw();
      sm_GraphicsLoop();
    }
#endif

  // All done, clean up and return.
  sCleanNewSurf.Clear();
  sClean3D.Clear();
  if (pSurfToTrim == NULL) { sCleanUV.Clear(); }
  else                     { delete pSurfToTrim; pSurfToTrim = NULL ; }

  rpNewSurface = pNewSurf;
  rdNewFaceTol = dOrigEdgeTol;

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::EdgeSweepHigher

/*******************************************************************//**
PURPOSE: Create geometry for sweeping an SmEdge into same dim

NOTES: This assumes that the edge does not degenerate to a single
                point
***********************************************************************/
SmStatus SmRotationalSweepGeometry::EdgeSweepSame
 (const SmContext & crContext,              // in : context for new object construction
  const SmEdge    * pOriginalEdgeArg,       // in : target edge to sweep
  SmCurve        *& rpNewCurveArg,          // out: new curve at swept position
  double          & rdNewEdgeToleranceArg)  // out: new edge tolerance
 const
{
  // Edge is swept on its own
  rpNewCurveArg         = NULL;
  rdNewEdgeToleranceArg = 0.0;

  SmCurve   * pOriginalEdgeCurve = pOriginalEdgeArg->GetCurve();
  SmExtent1d  sOriginalEdgeIV    = pOriginalEdgeArg->GetInterval();
  double      dOriginalEdgeTol   = pOriginalEdgeArg->GetTolerance();
  if (pOriginalEdgeCurve == NULL)
    {
      SER(SM_ERR);
      return SM_ERR;
    }

  SmBrep * pBrep    = pOriginalEdgeArg->GetBrep();
  double   dBrepTol = pBrep->GetTolerance();
  double    dEps    = smos_Max(m_dEps, dBrepTol);

  // GWC: perhaps we could call CheckCurveSweep() here.
  //      I'm not sure yet because that checks to see if the curve builds a valid
  //      swept surface. This method does not build a surface just builds a copy of the
  //      curve swept to its final position.
  //      It may be the case that the overall algorithm might need the check
  //      here to make a valid output, but for the time being the check
  //      is only being added to EdgeSweepHigher where the curve is used to build the
  //      sweep surface.

  SmBoolean bIsLine;
  SER(sm_CheckEdgeCurveLine(pOriginalEdgeCurve, sOriginalEdgeIV,
                               dEps,
                               bIsLine));

  SmBoolean bEdgeOnAxis;
  SER(sm_CheckEdgeOnAxis(m_sBasePt, m_sAxis,
      pOriginalEdgeArg,
      bIsLine,
      smos_Max(m_dEps, dOriginalEdgeTol),
      bEdgeOnAxis));


  SmCurve *pNewEdgeCurve;
  SER(pOriginalEdgeCurve->Copy(crContext,pNewEdgeCurve));
  SmExtent1d sTrimIvl = pOriginalEdgeArg->GetInterval();
  SER(pNewEdgeCurve->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots

  if (bEdgeOnAxis )
    {
      rpNewCurveArg = pNewEdgeCurve;
      rdNewEdgeToleranceArg = smos_Max(m_dEps, dOriginalEdgeTol);
      return SM_SUCCESS;
    }

  if (m_dAngle == 360.)
    {
      rpNewCurveArg = pNewEdgeCurve;
      rdNewEdgeToleranceArg = smos_Max(m_dEps, dOriginalEdgeTol);
      return SM_SUCCESS;
    }


  SmObjDelete sClean(pNewEdgeCurve);

  double dAngleRad = m_dAngle * SM_PI / 180.;

  SmAxis2Placement sRefFrame;
  sRefFrame.RotateAboutAxisAtPoint(dAngleRad, m_sBasePt, m_sAxis);
  SER(pNewEdgeCurve->Transform(sRefFrame));

  sClean.Clear();
  rpNewCurveArg = pNewEdgeCurve;
  rdNewEdgeToleranceArg = dOriginalEdgeTol;

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::EdgeSweepSame

/*******************************************************************//**
PURPOSE:  Create geometry for sweeping an SmFace in same dim

NOTES:
***********************************************************************/
SmStatus SmRotationalSweepGeometry::FaceSweepSame
 (const SmContext & crContext,                 // in : context for new object construction
  const SmFace    * pOriginalFaceArg,          // in : target face to sweep
  SmSurface      *& rpNewSurfaceArg,           // out: new surface in swept position
  double          & rdNewFaceToleranceArg)     // out: new face tolerance
 const
{
  rpNewSurfaceArg = NULL;

  if (smos_Fabs(m_dAngle-360.0) < SM_EFF_ZERO)
    {
      return SM_SUCCESS; // Nothing required for this case
    }

  double dOrigFaceTol = pOriginalFaceArg->GetTolerance();

  SmSurface* pOriginalFaceSurface = pOriginalFaceArg->GetSurface();

  SmSurface *pNewFaceSurf = NULL;

  // Copy pOriginalFaceSurface, when possible as an analytic surface
  SER(pOriginalFaceSurface->CopyAndAddAnalytics(crContext,pNewFaceSurf));
  NER(pNewFaceSurf);
  SmObjDelete sClean(pNewFaceSurf);

  double dAngleRad = m_dAngle * SM_PI / 180.;

  SmAxis2Placement sRefFrame;
  sRefFrame.RotateAboutAxisAtPoint(dAngleRad, m_sBasePt, m_sAxis);
  SER(pNewFaceSurf->Transform(sRefFrame));

  sClean.Clear();
  rpNewSurfaceArg = pNewFaceSurf;
  rdNewFaceToleranceArg = dOrigFaceTol;

  return SM_SUCCESS;

} // end SmRotationalSweepGeometry::FaceSweepSame

