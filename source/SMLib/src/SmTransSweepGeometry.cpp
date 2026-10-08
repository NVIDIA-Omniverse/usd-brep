// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTranslationalSweepGeometry.cpp
* PURPOSE   --- Source for class members.
**********************************************************************/

#include "StdAfx.h"

#include <SmTransSweepGeometry.h>
#include <SmSurface.h>
#include <SmLine.h>
#include <SmBSplineSurface.h>
#include <SmAxis2Placement.h>
#include <SmTopologySweep.h>
#include <SmProjectedCurve.h>

  // for IsEdgeType1

/*******************************************************************//**
PURPOSE: Constructor for a translation sweep that takes a vector.

NOTES:
***********************************************************************/
SmTranslationalSweepGeometry::SmTranslationalSweepGeometry
  (const SmVector3d & crSweepVectArg,
   double             dEps)
: SmSweepGeometryCreation(dEps),
  m_sSweepVector(crSweepVectArg)
{
}

/*******************************************************************//**
PURPOSE: Copy a SmTransSweepGeometry object.

NOTES:
***********************************************************************/
SmSweepGeometryCreation * SmTranslationalSweepGeometry::Copy()
{
  SmTranslationalSweepGeometry *pNew = new SmTranslationalSweepGeometry( m_sSweepVector, m_dEps );
  pNew->SetMakeAnalytics( m_bMakeAnalytics );

  return pNew;

} // end Copy


/*******************************************************************//**
PURPOSE: Return a coordinate system that is the start of the sweep.

NOTES:
  The z-axis will be the m_sSweepVector.  The x- and y-axes are arbitrary.
  We'll use the global x-axis if possible, else global y, else global z.
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::GetStartCoordSystem
  (const SmPoint3d  & rPt,       // in :
   SmAxis2Placement & rCoordSys) // out:
  const
{
  // If the x-component of the sweep vector is small enough,
  // use the global x-axis to get an x-axis that's perpendicular
  // to the sweep direction.
  SmVector3d sXAxis, sYAxis;
  if (    smos_Fabs( m_sSweepVector.x ) <= smos_Fabs( m_sSweepVector.y )
       && smos_Fabs( m_sSweepVector.x ) <= smos_Fabs( m_sSweepVector.z )
     )
    { sXAxis.Set( 1,0,0 ); }
  else if ( smos_Fabs( m_sSweepVector.y ) <= smos_Fabs( m_sSweepVector.z ) )
    { sXAxis.Set( 0,1,0 ); }
  else
    { sXAxis.Set( 0,0,1 ); }

  sYAxis = m_sSweepVector * sXAxis;
  SER( sYAxis.Unitize() );

  sXAxis = sYAxis * m_sSweepVector;
  SER( sXAxis.Unitize() );

  rCoordSys.SetCanonical( rPt, sXAxis, sYAxis );

  return SM_SUCCESS;

} // end GetStartCoordSystem


/*******************************************************************//**
PURPOSE: Return a coordinate system that is the end of the sweep.

NOTES:
  The z-axis will be the m_sSweepVector.  The x- and y-axes are arbitrary.
  We'll use the global x-axis if possible, else global y, else global z.
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::GetEndCoordSystem
  (const SmPoint3d  & rPt,       // in :
   SmAxis2Placement & rCoordSys) // out:
  const
{
  // Get the start C.S. and translate by m_sSweepVector.
  SER( GetStartCoordSystem( rPt, rCoordSys ));

  rCoordSys.Translate( m_sSweepVector );

  return SM_SUCCESS;

} // end GetStartCoordSystem

/*******************************************************************//**
PURPOSE: Return the transform from start to end of the sweep.

NOTES:
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::GetSweepTransform
  (SmAxis2Placement & rCoordSys)  // out:
  const
{
  rCoordSys.Init();

  rCoordSys.Translate( m_sSweepVector );

  return SM_SUCCESS;

} // end GetSweepTransform

/*******************************************************************//**
PURPOSE: Create a curve which represents the sweeping of a
     vertex into higher dimension.

NOTES:
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::VertexSweepHigher
  (const SmContext & crContext,              // in : context for new object construction
   const SmVertex  * pOriginalVertexArg,     // in : target vertex to sweep
   SmCurve        *& rpNewCurveArg,          // out: newly swept curve
   double          & rdNewEdgeToleranceArg)  // out:
 const
{
  // get curve start and end points
  SmPoint3d sOriginalPt( pOriginalVertexArg->GetPoint() );
  SmPoint3d sSweptPt( sOriginalPt + m_sSweepVector );

  // get vertex tolerance
  double dOrigVertexTolerance = pOriginalVertexArg->GetTolerance();

  // check state - no sweeps shorter than 2* vertex tolerance
  double dSweepLength = m_sSweepVector.Length();
  if ( dSweepLength < 2.0*dOrigVertexTolerance )
    {
      SER(SM_ERR); // SweepVertex: diminishing sweep
    }

  // construct the line
  SmLine *pLine;
  SER(SmLine::CreateLineSegment(crContext,
                                3,            // dimension
                                sOriginalPt,
                                sSweptPt,
                                pLine));
  NER(pLine); // If line not created error out

  // set output
  rpNewCurveArg         = pLine;
  rdNewEdgeToleranceArg = dOrigVertexTolerance;

  // all done
  return SM_SUCCESS;

} // end SmTranslationalSweepGeometry::VertexSweepHigher

/*******************************************************************//**
PURPOSE: Create a point which represents the sweep of a vertex to the
    same dimension.

NOTES:
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::VertexSweepSame
  (const SmVertex *pOriginalVertexArg,  // in : target vertex to sweep
   SmPoint3d & rPointArg,               // out: swept point
   double & rdNewPointToleranceArg)     // out: new point tolerance
  const
{
  // get start and stop sweep points
  SmPoint3d sOriginalPt( pOriginalVertexArg->GetPoint() );
  SmPoint3d sSweptPt( sOriginalPt + m_sSweepVector );

  // get vertex tolerance
  double dOrigVertexTolerance = pOriginalVertexArg->GetTolerance();

  // check state - no sweeps less than 2*vertexTolerance
  double dSweepLength = m_sSweepVector.Length();
  if ( dSweepLength < 2.0*dOrigVertexTolerance) {
      SER(SM_ERR); // SweepVertex: diminishing sweep
  }

  // set output
  rPointArg              = sSweptPt;
  rdNewPointToleranceArg = dOrigVertexTolerance;

  // all done
  return SM_SUCCESS;

} // end SmTranslationalSweepGeometry::VertexSweepSame

/*******************************************************************//**
PURPOSE: Create geometry for sweeping an SmEdge into higher dim.
    This function will create the bounding curves of the sweep so that
    we will not have to do individual sweeps of vertices and edges in
    most cases.

NOTES:
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::EdgeSweepHigher
  (const SmContext             & crContext,                // in : context for new object construction
   const SmEdge                * pOriginalEdgeArg,         // in : Edge to be swept to a higher dimension
   SmSurface                  *& rpNewSurfaceArg,          // out: Surface created by the sweep of the edge
   double                      & rdNewFaceToleranceArg,    // out: Tolerance of the face - derived from edge tolerance.
   SmCurve                    *& rpStartCurve,             // out: Curve Copy in orig position trimmed to edge ivl.
   double                      & rdStartCurveTolerance,    // out: Tolerance of the new start curve. value:[pEdgeArg->Tol]
   SmCurve                    *& rpNewEdgeCurveArg,        // out: Curve copy moved to end sweep position.
   double                      & rdNewEdgeToleranceArg,    // out: Tolerance of this new edge. value:[pEdgeArg->Tol]
   SmCurve                    *& rpStartVertexCurveArg,    // out: start vertex sweep curve.
   double                      & rdNewSVEdgeToleranceArg,  // out: start vertex sweep curve tolerance.
   SmCurve                    *& rpEndVertexCurveArg,      // out: end vertex sweep curve.
   double                      & rdNewEVEdgeToleranceArg,  // out: end vertex sweep curve tolerance.
   SmTArray<SmCurve*>          & r3DTrimmingCurves,        // out: ordered new 3D TrimCurves (ptrs to previously output curves)
   SmTArray<SmBSplineCurve*>   & rUVTrimmingCurves,        // out: associated new UV TrimCurves when easy, not built when expensive.
   SmTArray<SmOrientType>      & rOrients,                 // out: associated TrimCurve Orients for a valid outer loop. SM_OT_SAME, SM_OT_OPPOSITE
   SmVector3d                  & rsSurfNormalStVtxArg)     // out: The surface normal of the surface at the start vertex.
  const
{
  // init output arrays
  r3DTrimmingCurves.ReSet();
  rUVTrimmingCurves.ReSet();
  rOrients.ReSet();

  // check required input Edge pointer
  NER(pOriginalEdgeArg);

  // local edge->curve pointer
  SmCurve        * pOriginalEdgeCurve = pOriginalEdgeArg->GetCurve();
  SmBSplineCurve * pBSC               = SM_CAST_PTR(SmBSplineCurve,pOriginalEdgeCurve);
  SmExtent1d       sTrimIvl           = pOriginalEdgeArg->GetInterval();
  double           dSweepLength       = m_sSweepVector.Length() ;
  SmVector3d       sSweepVec          = m_sSweepVector/dSweepLength ;
  SmPoint3d        sSweepPoint(0,0,0) ;
  NER(pBSC);

  // get Brep tolerance
  double dBrepTol = pOriginalEdgeArg->GetBrep()->GetTolerance();

  // GWC: to be added when we have time to debug all the code written for this case
  // check to see if the curve will sweep into a valid surface
  //   added for Bug 317 - but always useful
  SmSweepCheckType eSweepCheckType ;
  SER( CheckCurveSweep(*pOriginalEdgeCurve, sTrimIvl, m_dEps, sSweepPoint, sSweepVec, dSweepLength, eSweepCheckType) ) ;

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

  // Note that we have to trim the original curve because the edge may
  // only be on a portion of the curve.
  SER(pBSC->Copy(crContext,rpStartCurve));
  SmObjDelete sCleanup1(rpStartCurve);
  SER(rpStartCurve->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots

  // get start/end vertex and tolerance
  SmVertex* pStartVertex = pOriginalEdgeArg->GetStartVertex();
  SmVertex* pEndVertex   = pOriginalEdgeArg->GetOtherVertex(pStartVertex);

  double    dStVtxTol    = pStartVertex->GetTolerance();
  double    dEndVtxTol   = pEndVertex->GetTolerance();

  // handle closed edges
  if (pEndVertex == pStartVertex)
    {
      pEndVertex = NULL;
      dEndVtxTol = 0.0;
    }

  // get edge tolerance
  double dOrigEdgeTol = pOriginalEdgeArg->GetTolerance();

  // check state - no sweeps shorter than 2 * edge or vertex tolerances
  if (!(   dSweepLength > 2.0*dStVtxTol
        && dSweepLength > 2.0*dEndVtxTol
        && dSweepLength > 2.0*dOrigEdgeTol) )
    {
      SER(SM_ERR); //   'SweepEdge: diminishing sweep');
    }

  // newFaceTolerance = EdgeTolerance
  rdNewFaceToleranceArg = dOrigEdgeTol;

  // create the sweep surface from the edge->curve
  //  Note: at some point in the future for sweeps which
  //        produce planes we will want to create them directly even
  //        though the the sweep may not be perpendicular to the edge.
  SmBSplineSurface *pNewFaceSurf = NULL;
  SmBSplineCurve *pBSC2 = SM_CAST_PTR(SmBSplineCurve,rpStartCurve);
  NER(pBSC2);  // right now this only works for B-Spline Curves
  SER( SmBSplineSurface::CreateLinearSweep(crContext,
                                           *pBSC2,           // curve to sweep
                                           m_sSweepVector,   // Defines both magnitude and dir of the sweep
                                           pNewFaceSurf));
  NER(pNewFaceSurf);

  // Create corresponding analytic surface if requested
  if (m_bMakeAnalytics)
    {
      SmSurface *pBSS;
      SmObjDelete sClean3(pNewFaceSurf);

      // Copy pNewFaceSurf for new face, when possible as an analytic surface
      SER(pNewFaceSurf->CopyAndAddAnalytics(crContext,pBSS));
      pNewFaceSurf = SM_CAST_PTR(SmBSplineSurface,pBSS);
    }
  SmObjDelete sCleanNewSurf(pNewFaceSurf);

  // Note that CreateLinearSweep is a deterministic function and
  // that we could simply extract the ISO curves and points directly
  // from the surface.  It should be changed ASAP to give an example
  // of how to do this.  As the sweeps
  // become more complex, people will not want to implement separate
  // sweeps for vertices and edges if they are just doing simple
  // solid type sweeps.
  SmExtent2d sSurfDomain( pNewFaceSurf->GetNaturalUVDomain() );

  // evaluate start edge endPoints
  SmPoint3d sSurf00, sSurf01 ;
  SER( pNewFaceSurf->EvaluatePoint(sSurfDomain.GetMin(), sSurf00));
  SER( pNewFaceSurf->EvaluatePoint(sSurfDomain.Evaluate(0.0,1.0), sSurf01));

  SmPoint3d sStartPt( pStartVertex->GetPoint() );

  double dSt00 = (sSurf00 - sStartPt).Length();
  double dSt01 = (sSurf01 - sStartPt).Length();

  double dMin = dSt00;
  SmPoint2d pStartUV = sSurfDomain.GetMin();

  SmBoolean bSwapEnds = FALSE;

  if (    dSt01 < dMin
      && !pOriginalEdgeArg->IsClosed())
    {
      bSwapEnds = TRUE;
      pStartUV = sSurfDomain.Evaluate(0.0,1.0);
      dMin = dSt01;
    }

  if (dMin > dBrepTol) SER(SM_ERR); //EdgeSweepHigh: New surface creation

  SER(pNewFaceSurf->EvaluateNormal(pStartUV,
      TRUE,TRUE,
      rsSurfNormalStVtxArg));

  // Create curve along minimum U value
  r3DTrimmingCurves.Add(rpStartCurve);
  rOrients.Add(SM_OT_OPPOSITE);
  SmLine *pUVCurve = NULL;
  SER(SmLine::CreateLineSegment(crContext,2,SmVector3d(sSurfDomain.Evaluate(0,0)),
      SmVector3d(sSurfDomain.Evaluate(0,1)),pUVCurve));
  rUVTrimmingCurves.Add(pUVCurve);
  rdStartCurveTolerance = dOrigEdgeTol;

  // Create curve along minimum V value
  SmBSplineCurve *pSideCurve = NULL ;
  SER(pNewFaceSurf->CreateIsoParametricCurve(crContext,
                                             SM_SP_V,
                                             sSurfDomain.GetMin().y,
                                             dOrigEdgeTol,
                                             pSideCurve));
  SmObjDelete sCleanup3(pSideCurve);
  if (!bSwapEnds)
    {
      rdNewSVEdgeToleranceArg = dStVtxTol;
      rpStartVertexCurveArg = pSideCurve;
    }
  else
    {
      rdNewEVEdgeToleranceArg = dEndVtxTol;
      rpEndVertexCurveArg = pSideCurve;
    }
  r3DTrimmingCurves.Add(pSideCurve);
  rOrients.Add(SM_OT_SAME);
  SER(SmLine::CreateLineSegment(crContext,2,SmVector3d(sSurfDomain.Evaluate(0,0)),
      SmVector3d(sSurfDomain.Evaluate(1,0)),pUVCurve));
  rUVTrimmingCurves.Add(pUVCurve);

  // Create curve along maximum U value
  SER(pNewFaceSurf->CreateIsoParametricCurve(crContext,
                                             SM_SP_U,
                                             sSurfDomain.GetMax().x,
                                             dOrigEdgeTol,
                                             pSideCurve));
  SmObjDelete sCleanNewEdgeCurve(pSideCurve);
  rdNewEdgeToleranceArg = dOrigEdgeTol;
  rpNewEdgeCurveArg     = pSideCurve;
  r3DTrimmingCurves.Add(rpNewEdgeCurveArg);
  rOrients.Add(SM_OT_SAME);
  SER(SmLine::CreateLineSegment(crContext,2,SmVector3d(sSurfDomain.Evaluate(1,0)),
      SmVector3d(sSurfDomain.Evaluate(1,1)),pUVCurve));
  rUVTrimmingCurves.Add(pUVCurve);

  // Create curve along maximum V value
  SER(pNewFaceSurf->CreateIsoParametricCurve(crContext,
                                             SM_SP_V,
                                             sSurfDomain.GetMax().y,
                                             dOrigEdgeTol,
                                             pSideCurve));
  SmObjDelete sCleanup2(pSideCurve);
  r3DTrimmingCurves.Add(pSideCurve);
  if (bSwapEnds)
    {
      rdNewSVEdgeToleranceArg = dStVtxTol;
      rpStartVertexCurveArg   = pSideCurve;
    }
  else
    {
      rdNewEVEdgeToleranceArg = dEndVtxTol;
      rpEndVertexCurveArg     = pSideCurve;
    }
  rOrients.Add(SM_OT_OPPOSITE);
  SER(SmLine::CreateLineSegment(crContext,2,SmVector3d(sSurfDomain.Evaluate(0,1)),
      SmVector3d(sSurfDomain.Evaluate(1,1)),pUVCurve));
  rUVTrimmingCurves.Add(pUVCurve);

  if (pOriginalEdgeArg->IsClosed())
    {
      rpEndVertexCurveArg = NULL;
    }

  // retain output values
  sCleanup1.Clear();
  sCleanNewEdgeCurve.Clear();
  sCleanup2.Clear();
  sCleanup3.Clear();
  sCleanNewSurf.Clear();
  rpNewSurfaceArg = pNewFaceSurf;

  // all done
  return SM_SUCCESS;

} // end SmTranslationalSweepGeometry::EdgeSweepHigher

/*******************************************************************//**
PURPOSE: Create geometry for sweeping an SmEdge into same dim

NOTES:
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::EdgeSweepSame
  (const SmContext & crContext,              // in : context for new object construction
   const SmEdge    * pOriginalEdgeArg,       // in : target edge to sweep
   SmCurve        *& rpNewCurveArg,          // out: new curve at swept position
   double          & rdNewEdgeToleranceArg)  // out: new edge tolernance
 const
{
  // Edge is swept on its own,

  // GWC: perhaps we could call CheckCurveSweep() here.
  //      I'm not sure yet because that checks to see if the curve builds a valid
  //      swept surface. This method does not build a surface just builds a copy of the
  //      curve swept to its final position.
  //      It may be the case that the overall algorithm might need the check
  //      here to make a valid output, but for the time being the check
  //      is only being added to EdgeSweepHigher where the curve is used to build the
  //      sweep surface.

  // get edge->Curve and tolerance
  SmCurve * pOriginalEdgeCurve = pOriginalEdgeArg->GetCurve();
  double    dOriginalEdgeTol   = pOriginalEdgeArg->GetTolerance();

  // check state - no sweep shorter than 2 * tolerance
  double dSweepLength =  m_sSweepVector.Length();
  if ( dSweepLength < 2.0*dOriginalEdgeTol )
    {
      SER(SM_ERR); // SweepEdge: diminishing sweep
    }

  // create the new curve
  SmCurve *pNewEdgeCurve;
  SER(pOriginalEdgeCurve->Copy(crContext,pNewEdgeCurve));
  SmObjDelete sClean(pNewEdgeCurve);

  // trim the new curve to edge->interval
  SmExtent1d sIvl = pOriginalEdgeArg->GetInterval();
  pNewEdgeCurve->Trim(sIvl);     // may snap sIvl by tol to existing knots

  // define the translation transformation
  SmAxis2Placement sRefFrame;
  sRefFrame.Translate(m_sSweepVector);

  // translate the curve
  SER(pNewEdgeCurve->Transform(sRefFrame));

  // retain the curve - set output
  sClean.Clear();
  rpNewCurveArg         = pNewEdgeCurve;
  rdNewEdgeToleranceArg = dOriginalEdgeTol;

  // all done
  return SM_SUCCESS;

} // end SmTranslationalSweepGeometry::EdgeSweepSame

/*******************************************************************//**
PURPOSE:  Create geometry for sweeping an SmFace in same dim

NOTES:
***********************************************************************/
SmStatus SmTranslationalSweepGeometry::FaceSweepSame
 (const SmContext & crContext,             // in : context for new object construction
  const SmFace    * pOriginalFaceArg,      // in : target face to sweep
  SmSurface      *& rpNewSurfaceArg,       // out: new surface in swept position
  double          & rdNewFaceToleranceArg) // out: new face tolerance
 const
{
  // local surface and tolerance values
  SmSurface * pOriginalFaceSurface = pOriginalFaceArg->GetSurface();
  double      dOrigFaceTol         = pOriginalFaceArg->GetTolerance();

  // check state - no sweep shorter than 2 * tolerance
  double dSweepLength =  m_sSweepVector.Length();
  if ( dSweepLength < 2.0*dOrigFaceTol ) {
      SER(SM_ERR); // SweepEdge: diminishing sweep
  }

  // make a new face copy
  SmSurface *pNewFaceSurf = NULL;

  // Copy pOriginalFaceSurface for new face, when possible as an analytic surface
  SER(pOriginalFaceSurface->CopyAndAddAnalytics(crContext,pNewFaceSurf));
  NER(pNewFaceSurf);
  SmObjDelete sClean(pNewFaceSurf);

  // construct the translate transformation
  SmAxis2Placement sRefFrame;
  sRefFrame.Translate(m_sSweepVector);

  // translate the new face
  SER(pNewFaceSurf->Transform(sRefFrame));

  // retain the new face - set output
  sClean.Clear();
  rpNewSurfaceArg       = pNewFaceSurf;
  rdNewFaceToleranceArg = dOrigFaceTol;

  // all done
  return SM_SUCCESS;

} // end SmTranslationalSweepGeometry::FaceSweepSame

/*******************************************************************//**
PURPOSE: Verify that sweeping a curve will produce a valid swept surface.

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
SmStatus SmTranslationalSweepGeometry::CheckCurveSweep
 ( const SmCurve     & crCurve,       // in : Curve to check for valid sweep
   const SmExtent1d  & crIvl,         // in : Interval to sweep
   double              dDistTol3d,    // in : min dist between unique points
   const SmPoint3d   & crSweepPoint,  // NotUsed: in : RotSweep:[RotAxis point],  TransSweep:[NotUsed]
   const SmVector3d  & crSweepVec,    // in : RotSweep:[RotAxis vector], TransSweep:[Trans vector]
   double              dSweepAmount,  // in : RotSweep:[SweepAngleDeg],  TransSweep:[distance]
   SmSweepCheckType  & rSweepState)   // out: SM_SC_OKAY               = okay to sweep this surface
  const                               //      SM_SC_ROT_ON_CURVE       = revolving curve about point on curve produces a BowTie or ConeApex surface
                                      //      SM_SC_SWEEP_ALONG_LENGTH = sweeping curve along it's length produces degenerate surface points
                                      //      SM_SC_SELF_INTERSECT     = sweeping this curve produces a self-intersecting surface
                                      //      SM_SC_UNKNOWN            = Sweep method failed so swept surface validity is unknown
{
 SM_REF1(crSweepPoint) ;
  // init output
  rSweepState = SM_SC_UNKNOWN ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      const SmContext * cpContext = crCurve.GetContext() ;
      const SmEdge    * cpEdge    = crCurve.GetEdge() ;
      SmBrep          * pBrep     = cpEdge ? cpEdge->GetBrep() : NULL ;
      SmExtent1d        sSweepIvl(0.0, dSweepAmount) ;
      SmVector3d        sX        = crSweepVec ;
      SmPoint3d         sPt ;
      crCurve.EvaluatePoint(crIvl.GetMin(), sPt) ;
      sX.Unitize() ;
      SmLine sSweepLine(sPt, sX, sSweepIvl, 1.0, 3, cpContext) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; sPt.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&sPt) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(sPt) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; sSweepLine.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#else
  SM_REF1(dSweepAmount);
#endif // SM_DEBUG_CODE

  // locals
  SmSolutionArray sPointParallelSols ;

  // get all places on the curve parallel to the sweep vector
  crCurve.GlobalPropertyAnalysis(crIvl,                    // in : ThisCurve target interval
                                 SM_CP_PARALLEL_TO_VECTOR, // in : The property of the curve to extract or find on the curve
                                 NULL,                     // in : Value used to specify a particular property.
                                &crSweepVec,               // in : Vectors or points used to define a particular property.
                                                           //      If there is a point, that should be the first value in the array
                                                           //      If there is a normal (projection), it will come before all other vectors
                                 dDistTol3d,               // in : 3D Tolerance used to determine when two answers are equivalent
                                 sPointParallelSols,       // out: SolArray of Curve Points at dist to line maxima
                                 NULL,                     // out: opt check comb: curve sample points
                                                           //      NULL to ignore, default:[NULL]
                                 NULL) ;                   // out: opt check comb: tines - lengths proportional to property being zeroed.
                                                           //      NULL to ignore, default:[NULL]
  if(sPointParallelSols.GetSize() > 0)
    {
      rSweepState = SM_SC_SWEEP_ALONG_LENGTH ;

      return(SM_SUCCESS) ;
    }

#ifdef SM_SWEEP_CHECKS
  // gwc: the following should replace all the checks in this function when
  //      new SM_CP_MAXIMA_TO_LINE and SM_CP_MAXIMA_TO_PLANE options
  //      and GlobalCurveSelfIntersect() methods are verified


  // locals
  ULONG ii, i2 ;
  SmSolutionArray sPointXSectSols ;
  SmSolutionArray sDistToLineSols ;
  SmSolutionArray sDistToPlaneSols ;
  SmSolutionArray sSelfXSectSols ;

  // locals
  SmVector3d sOptVecs[2], sPV[2] ;
  SmTArray<SmVector3d> sCurvePVs ;

  // load sweep point and sweepVec into sOptVecs array
  crCurve.EvaluatePoint(crIvl.Evaluate(0), sOptVecs[0]) ;
  sOptVecs[1] = crSweepVec ;

  // get all places on the curve where it doubles back on itself as measured as distance from translation axis
  //   note: solution also finds line/curve intersection points
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
      SmExtent1d        sSweepIvl(0.0, dSweepAmount) ;
      SmVector3d        sX        = crSweepVec ;
      SmPoint3d         sPt ;
      crCurve.EvaluatePoint(crIvl.GetMin(), sPt) ;
      sX.Unitize() ;
      SmLine sSweepLine(sPt, sX, sSweepIvl, 1.0, 3, cpContext) ;

      sDistToLineSols.Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; sPt.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&sPt) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(sPt) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; sSweepLine.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; sDistToLineSols.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when there are solutions - check if any of the maxima are tangent to the sweep direction = BadSweep
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

          // evaluate curve at maxima
          crCurve.Evaluate(dParam, 1, TRUE, pPVArray+i2, TRUE) ;

          // when sweep direction is parallel to the tangent direction
          // note: for rotational sweep - IsParallelTo() call becomes (IsPerpendicularTo).
          if(sCurvePVs[i2+1].IsParallelTo(crSweepVec, SM_EFF_ZERO_DEG))
            {
              rSweepState = SM_SC_SWEEP_ALONG_LENGTH ;

              // all done
              return(SM_SUCCESS) ;

            } // end tangent and sweep directions are parallel check
        } // end iter every maxima to rot axis solution

      // arrive here when curve has distance to rot axis maximas that are never parallal to the sweep direction
      // It's possible that these curves can sweep into self intersecting swept surfaces

      // project the curve back to a common translation plane
      SmProjectedCurve sProjCurve(&crCurve, &sOptVecs[0], &sOptVecs[1], NULL, SM_PT_PARALLEL, FALSE, crCurve.GetContext()) ;

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
          SmExtent1d        sSweepIvl(0.0, dSweepAmount) ;
          SmVector3d        sX        = crSweepVec ;
          SmPoint3d         sPt ;
          crCurve.EvaluatePoint(crIvl.GetMin(), sPt) ;
          sX.Unitize() ;
          SmLine sSweepLine(sPt, sX, sSweepIvl, 1.0, 3, cpContext) ;

          sSelfXSectSols.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,1) ; if(cpEdge) cpEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; crCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; crCurve.DrawParams(&crIvl) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,1,1) ; sProjCurve.Draw(&crIvl, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1,0,0) ; sPt.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,.5,0); crSweepVec.Draw(&sPt) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,.5); crSweepVec.DrawPlane(sPt) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sSweepLine.Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1,0,0) ; sSelfXSectSols.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

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

              // vector
              SmVector3d sVec10 = sPoint1 - sPoint0 ;

              // sweep distance between points
              double dPointDistSq = sVec10.LengthSquared() ;

              // When sweep is large enough to cause self-intersection return error
              if(dSweepAmount*dSweepAmount > dPointDistSq)
                {
                   // attempting to translate a curve through itself
                   rSweepState = SM_SC_SELF_INTERSECT ;

                   // all done
                   return(SM_SUCCESS) ;
                }
            } // end SM_ST_SINGLE_VALUE branch
          else // SM_ST_RANGE_OF_VALUES solution branch
            {
              // we need the minimum 3d translation distance between two segments
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

              // when class SmSufOfExtrusion is fully extended (with this very CheckCurveSweep() code)
              // work directly with the SmCurve in the next section and remove the BSpline dependency
              if(pBSplineCurve == NULL)
                {
                  WARN(_T("SmTranslationalSweepGeometry::CheckCurveSweep found a NonBSpline projected coincident segment - case not supported until SmSurOfExtrusion is extended")) ;
                  rSweepState = SM_SC_UNKNOWN ;

                  // all done
                  return(SM_SUCCESS) ;
                }

              // locals for temps
              SmContext       sContext ;
              SmSolutionArray sSurfCurveXSects ;
              SmExtent1d      sSegIvl0(rSol.m_vStart[0], rSol.m_vEnd[0]) ;
              SmExtent1d      sSegIvl1(rSol.m_vStart[1], rSol.m_vEnd[1]) ;
              SmVector3d      sProjVec = dSweepAmount * sProjVec ;

              // copy the curve into its segments
              SmBSplineCurve sSegment0(*pBSplineCurve) ; sSegment0.SetContext(pBSplineCurve->GetContext()) ;
              SmBSplineCurve sSegment1(*pBSplineCurve) ; sSegment1.SetContext(pBSplineCurve->GetContext()) ;

              // trim the segments to their intervals
              sSegment0.Trim(sSegIvl0, FALSE) ;   // may snap sIvl by tol to existing knots
              sSegment1.Trim(sSegIvl1, FALSE) ;   // may snap sIvl by tol to existing knots

              // create the translation surfaces from each segment
              SmBSplineSurface *pSweptSegment0 = NULL ;
              SmBSplineSurface *pSweptSegment1 = NULL ;
              SmBSplineSurface::CreateLinearSweep(sContext, sSegment0, sProjVec, pSweptSegment0) ;
              SmBSplineSurface::CreateLinearSweep(sContext, sSegment1, sProjVec, pSweptSegment0) ;
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

    } // end curve has r=const cylinder maximas (curve is tangent to one of these cylinders in one or more places)

#endif // SM_SWEEP_CHECKS

  // arrive here after passing all tests
  rSweepState = SM_SC_OKAY ;

  // all done
  return SM_SUCCESS ;

} // end SmTranslationalSweepGeometry::CheckCurveSweep


