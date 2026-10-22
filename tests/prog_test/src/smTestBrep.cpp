// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- smTestBrep.cpp
* PURPOSE --- Shared analytic fixtures for topology and healing regressions.
**********************************************************************/

#include "StdAfx.h"
#include <smTestBrep.h>
#include <SmThreadLocalStorage.h>
#include <SmAssertArray.h>
#include <SmCrvOnSurf.h>
#include <SmFaceProps.h>
#include <SmPrimitiveCreation.h>
#include <SmTrimmingTools.h>
#include <SmTrackTopologyChanges.h>
#include <SmLine.h>
#include <NL_Geometry.h>

SmBrep * sm_make_xy_sheet(const SmContext & crContext, double dSize)
{
  SmAxis2Placement sRefFrame ;
  sRefFrame.SetCanonical(SmPoint3d(0.0, 0.0, 0.0),
                         SmVector3d(1.0, 0.0, 0.0),
                         SmVector3d(0.0, 1.0, 0.0)) ;
  return SmPrimitiveCreation::CreateRectangle(crContext, 1.0e-5, dSize, dSize, sRefFrame) ;
}

SmBoolean sm_HasAssertReport(const SmAssertArray & crAsserts, ULONG lTestIndex, const void * pOwner)
{
  for(ULONG ii=0; ii<crAsserts.GetSize(); ii++)
    {
      if(   crAsserts[ii] != NULL
         && crAsserts[ii]->GetTestIndex() == lTestIndex
         && crAsserts[ii]->GetOwner() == pOwner)
        { return TRUE ; }
    }
  return FALSE ;
}

SmStatus sm_MakeLoopOrientationCase
 (const SmContext &crContext, // in : allocation context
  ULONG            lCase,     // in : original knot-sampling case number
  SmBrep         *&rpBrep,    // out: caller-owned brep
  SmFace         *&rpFace,    // out: face with the sampled boundary
  SmLoop         *&rpLoop)    // out: loop whose orientation is measured
{
  rpBrep = NULL;
  rpFace = NULL;
  rpLoop = NULL;
  SmBrepConstructor sBuilder;
  SmBrep *pBrep = sBuilder.StartBrep(crContext);
  NER(pBrep);
  SmObjDelete sCleanup(pBrep);
  pBrep->SetTolerance(1e-7,FALSE);
  SmShell *pOuter=NULL, *pInner=NULL;
  SER(sBuilder.StartShell(pOuter,pInner));
  SmFace *pFace = sBuilder.StartFace(SM_OT_SAME);
  SmTorus *pTorus = NULL;
  SER(SmTorus::CreateCanonical
    (crContext,          // in : allocation context
     SmAxis2Placement(), // in : torus placement
     2.0,                // in : major radius
     0.5,                // in : minor radius
     pTorus));           // out: torus surface
  SER(sBuilder.SetFaceSurface(pTorus,pTorus->GetNaturalUVDomain()));
  SmLoop *pLoop = sBuilder.StartLoop(SM_OT_SAME);
  SmPoint2d sPoints[9] = {SmPoint2d(.4,.4),SmPoint2d(.4,.6),SmPoint2d(.6,.6),
                         SmPoint2d(.6,.4),SmPoint2d(.9,.4),SmPoint2d(.9,.9),
                         SmPoint2d(.1,.9),SmPoint2d(.1,.4),SmPoint2d(.4,.4)};
  double dTimes[9] = {0,.3,.6,.9,.911,.922,.933,.944,1};
  const SmBoolean bReverse = lCase == 2 || lCase == 4 || lCase == 7 || lCase == 8 || lCase >= 10;
  if(lCase == 0)
    for(ULONG lPoint=0; lPoint<9; ++lPoint) { dTimes[lPoint] = double(lPoint)/8.0; }
  if(lCase == 5)
    for(ULONG lPoint=0; lPoint<9; ++lPoint) { sPoints[lPoint] = SmPoint2d(.5,.5)+(sPoints[lPoint]-SmPoint2d(.5,.5))*1e-5; }
  gw_CPOINT sCP[9];
  double dKnots[12] = {0};
  for(ULONG lPoint=0; lPoint<9; ++lPoint)
    {
      ULONG lNext = bReverse ? 8-lPoint : lPoint;
      sCP[lPoint] = {sPoints[lNext].x,sPoints[lNext].y,NL_NOZ,NL_NOW};
      dKnots[lPoint+1] = bReverse ? 1.0-dTimes[lNext] : dTimes[lNext];
    }
  dKnots[10] = 1.0;
  if(lCase >= 6)
    {
      // A rational quadratic UV circle exercises curved-span refinement.
      const SmPoint2d sCircle[9] = {SmPoint2d(.6,.5),SmPoint2d(.6,.6),SmPoint2d(.5,.6),
                                   SmPoint2d(.4,.6),SmPoint2d(.4,.5),SmPoint2d(.4,.4),
                                   SmPoint2d(.5,.4),SmPoint2d(.6,.4),SmPoint2d(.6,.5)};
      const double dCircleKnots[12] = {0,0,0,.25,.25,.5,.5,.75,.75,1,1,1};
      for(ULONG lPoint=0; lPoint<9; ++lPoint)
        {
          ULONG lNext = bReverse ? 8-lPoint : lPoint;
          double dWeight = (lPoint%2) ? smos_Sqrt(.5) : 1.0;
          sCP[lPoint] = {dWeight*sCircle[lNext].x,dWeight*sCircle[lNext].y,NL_NOZ,dWeight};
        }
      for(ULONG lPoint=0; lPoint<12; ++lPoint) { dKnots[lPoint] = dCircleKnots[lPoint]; }
    }
  SmBSplineCurve sUV
    (2,                    // in : curve dimension
     8,                    // in : highest control point index
     lCase >= 6 ? 11 : 10, // in : highest knot index
     lCase >= 6 ? 2 : 1,   // in : degree
     sCP,                  // in : control points
     dKnots,               // in : knots
     &crContext);          // in : allocation context
  SmCrvOnSurf *pCurve = new(crContext) SmCrvOnSurf
    (sUV,         // in : defining UV curve
     *pTorus,     // in : base surface
     NULL,        // in : natural surface domain
     2,           // in : copy and own both inputs
     &crContext); // in : allocation context
  SmPoint3d sPoint;
  SER(pCurve->EvaluatePoint(0.0,sPoint));
  SmVertex *pVertex = sBuilder.StartVertexOfLoop(sPoint);
  NER(sBuilder.StartEdge
    (SM_OT_SAME, // in : edgeuse orientation
     NULL,       // in : create a new edge
     pVertex,    // in : start vertex
     pVertex));  // in : end vertex
  SER(sBuilder.SetEdgeCurve(pCurve,FALSE));
  SER(sBuilder.SetLimits(SmVertexNode(sPoint),SmVertexNode(sPoint)));
  SER(sBuilder.EndEdge());
  SER(sBuilder.EndLoop());
  if(lCase == 8 || lCase == 9)
    {
      // Two disjoint CW holes require seams. An outer CCW loop with a
      // contained CW hole does not; checking only one loop is insufficient.
      double dCenter = lCase == 8 ? .2 : .5;
      SmPoint2d sHole[4] = {SmPoint2d(dCenter-.02,dCenter-.02),SmPoint2d(dCenter-.02,dCenter+.02),
                            SmPoint2d(dCenter+.02,dCenter+.02),SmPoint2d(dCenter+.02,dCenter-.02)};
      NER(sBuilder.StartLoop(SM_OT_SAME));
      SmVertex *pVertices[4];
      for(ULONG lPoint=0; lPoint<4; ++lPoint)
        {
          SmPoint3d sHolePoint;
          SER(pTorus->EvaluatePoint(sHole[lPoint],sHolePoint));
          pVertices[lPoint] = sBuilder.StartVertexOfLoop(sHolePoint);
        }
      for(ULONG lPoint=0; lPoint<4; ++lPoint)
        {
          ULONG lNext = (lPoint+1)%4;
          NER(sBuilder.StartEdge
            (SM_OT_SAME,         // in : edgeuse orientation
             NULL,               // in : create a new edge
             pVertices[lPoint],  // in : start vertex
             pVertices[lNext])); // in : end vertex
          SmLine sLine
            (sHole[lPoint], // in : start point
             sHole[lNext],  // in : end point
             2,             // in : dimension
             &crContext);   // in : allocation context
          SER(sBuilder.SetEdgeCurve(new(crContext) SmCrvOnSurf
            (sLine,               // in : defining UV curve
             *pTorus,             // in : base surface
             NULL,                // in : natural surface domain
             2,                   // in : copy and own both inputs
             &crContext),FALSE)); // in : allocation context
          SER(sBuilder.SetLimits(SmVertexNode(pVertices[lPoint]->GetPoint()),SmVertexNode(pVertices[lNext]->GetPoint())));
          SER(sBuilder.EndEdge());
        }
      SER(sBuilder.EndLoop());
    }
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(FALSE));
  SER(sBuilder.EndBrep());

  SmTArray<SmEdgeuse*> sUses;
  pLoop->GetLoopuse()->GetEdgeuses(sUses);
  if(lCase == 3 || lCase == 4)
    {
      // A plausible closed cached trim with the opposite orientation.
      // TRUE must still measure the defining 3D curve, leaving this cache intact.
      gw_CPOINT sReverseCP[9];
      double dReverseKnots[11];
      for(ULONG lPoint=0; lPoint<9; ++lPoint) { sReverseCP[lPoint] = sCP[8-lPoint]; }
      for(ULONG lPoint=0; lPoint<11; ++lPoint) { dReverseKnots[lPoint] = 1.0-dKnots[10-lPoint]; }
      sUses[0]->SetUVTrimCurve(new(crContext) SmBSplineCurve
        (2,                     // in : curve dimension
         8,                     // in : highest control point index
         10,                    // in : highest knot index
         1,                     // in : degree
         sReverseCP,            // in : control points
         dReverseKnots,         // in : knots
         &crContext),0.0,TRUE); // in : allocation context
    }
  rpBrep = pBrep;
  rpFace = pFace;
  rpLoop = pLoop;
  sCleanup.Clear();
  return SM_SUCCESS;
}

SmStatus sm_MakeTorusVertexLoopCase
 (SmContext                &rContext, // i/o: allocation context and Boolean state
  ULONG                     lCase,    // in : original vertex-loop case number
  SmTorusVertexLoopTestCase &rCase)   // out: caller-owned brep and expected seam topology
{
  SmBrepConstructor sBuilder;
  SmBrep           *pBrep = sBuilder.StartBrep(rContext);
  NER(pBrep);
  SmObjDelete       sCleanup(pBrep);
  SmShell          *pOuter = NULL, *pInner = NULL;
  pBrep->SetTolerance(1e-7,FALSE);
  SER(sBuilder.StartShell(pOuter,pInner));
  SmFace  *pFace  = sBuilder.StartFace(SM_OT_SAME);
  SmTorus *pTorus = NULL;
  SER(SmTorus::CreateCanonical
    (rContext,           // in : allocation context
     SmAxis2Placement(), // in : torus placement
     2.0,                // in : major radius
     0.5,                // in : minor radius
     pTorus));           // out: torus surface
  SER(sBuilder.SetFaceSurface(pTorus,pTorus->GetNaturalUVDomain()));
  NER(sBuilder.StartLoop(SM_OT_SAME));
  SmPoint2d sUV[4] = {SmPoint2d(.4,.4),SmPoint2d(.4,.6),SmPoint2d(.6,.6),SmPoint2d(.6,.4)};
  SmPoint3d sPoints[4];
  SmVertex *pHoleVertices[4];
  for(ULONG lPoint=0; lPoint<4; ++lPoint)
    {
      SER(pTorus->EvaluatePoint(sUV[lPoint],sPoints[lPoint]));
      pHoleVertices[lPoint] = sBuilder.StartVertexOfLoop(sPoints[lPoint]);
    }
  for(ULONG lPoint=0; lPoint<4; ++lPoint)
    {
      ULONG        lNext = (lPoint+1)%4;
      SmLine       sUVLine(sUV[lPoint],sUV[lNext],2,&rContext);
      SmCrvOnSurf *pCurve = new(rContext) SmCrvOnSurf
        (sUVLine,    // in : defining UV line
         *pTorus,    // in : base surface
         NULL,       // in : natural surface domain
         2,          // in : copy and own both inputs
         &rContext); // in : allocation context
      NER(sBuilder.StartEdge
        (SM_OT_SAME,             // in : edgeuse orientation
         NULL,                   // in : create a new edge
         pHoleVertices[lPoint],  // in : start vertex
         pHoleVertices[lNext])); // in : end vertex
      SER(sBuilder.SetEdgeCurve(pCurve,FALSE));
      SER(sBuilder.SetLimits(SmVertexNode(sPoints[lPoint]),SmVertexNode(sPoints[lNext])));
      SER(sBuilder.EndEdge());
    }
  SER(sBuilder.EndLoop());
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(FALSE));
  SER(sBuilder.EndBrep());
  pBrep->m_bEditingEnabled = TRUE;
  SmTemporaryChangeValue<SmBoolean> sDoingBool(rContext.GetDoingBooleanRef(),TRUE);
  ULONG     lFixture   = lCase%10;
  ULONG     lKind      = lFixture/2;
  SmBoolean bFirstU    = lFixture%2 == 0;
  ULONG     lNumPoints = lKind == 0 ? 1 : lKind == 3 ? 2 : lKind == 4 ? 5 : 3;
  SmVertex *pVertices[5] = {NULL,NULL,NULL,NULL,NULL};
  for(ULONG lPoint=0; lPoint<lNumPoints; ++lPoint)
    {
      SmPoint2d sPointUV(.2,.3);
      if(lPoint > 0)
        {
          SmBoolean bOnU = lKind == 4 ? (lPoint <= 2) == bFirstU : lKind == 1 ? bFirstU : !bFirstU;
          double    dParameter = .25+.5*((lPoint-1)%2);
          sPointUV = lKind == 3 ? SmPoint2d(0,0)
            : bOnU ? SmPoint2d(0,dParameter) : SmPoint2d(dParameter,0);
        }
      SmPoint3d sPoint;
      SmLoop   *pVertexLoop = NULL;
      SER(pTorus->EvaluatePoint(sPointUV,sPoint));
      SER(pBrep->MakeVertexLoop
        (pFace,               // in : target face
         sPoint,              // in : point on the surface
         pVertexLoop,         // out: new vertex loop
         pVertices[lPoint])); // out: new vertex
    }
  SmCurveClassification &rU        = rCase.m_sU;
  SmCurveClassification &rV        = rCase.m_sV;
  SmSurfParamType        &reCrossed = rCase.m_eCrossed;
  SmSurfParamType        &reMissing = rCase.m_eMissing;
  SmSurfParamType        &reNear    = rCase.m_eNear;
  pFace->HasSeamProblem
    (rU,        // out: U seam classification
     rV,        // out: V seam classification
     reCrossed, // out: illegally crossed seams
     reMissing, // out: missing seams
     reNear);   // out: boundaries near seams
  SER_MSG(reMissing == SM_SP_BOTH ? SM_SUCCESS : SM_ERR,_T("Vertex loops hid a missing torus seam"));
  SER_MSG(reCrossed == SM_SP_NEITHER ? SM_SUCCESS : SM_ERR,_T("Vertex loops were counted as seam crossings"));
  rCase.m_lExpectedEdges = 4+rU.GetSize()+rV.GetSize();
  rCase.m_lExpectedVertices = pBrep->GetNumVertices()+(lKind == 3 ? 0 : 1);
  rCase.m_pBrep = pBrep;
  rCase.m_pFace = pFace;
  rCase.m_lNumPoints = lNumPoints;
  for(ULONG lPoint=0; lPoint<lNumPoints; ++lPoint)
    { rCase.m_pVertices[lPoint] = pVertices[lPoint]; }
  sCleanup.Clear();
  return SM_SUCCESS;
}

static thread_local SmBoolean s_bVertexuseAsserted = FALSE;

static void sm_RecordVertexuseAssert
 (SmStatus     sStatus, // in : reported error status
  const TCHAR *,        // in : source file (unused)
  ULONG,                // in : source line (unused)
  const TCHAR *)        // in : diagnostic text (unused)
{
  if(sStatus == SM_ERR_ASSERT_FAILURE) { s_bVertexuseAsserted = TRUE; }
}

// A sector across the missing second seam retains two periodic UV locations.
static SmStatus sm_CheckMissingSeamSector
 (SmVertexuse *pVertexuse, // in : corner use after inserting the first seam
  SmBoolean    bMissingU)  // in : TRUE = missing U seam, FALSE = missing V seam
{
  SmEdgeuse *pEdgeuse = pVertexuse->GetEdgeuse();
  SmPoint3d  sStart,sEnd;
  SER(pEdgeuse->NormalizedEvaluate(0.,TRUE,sStart,NULL,TRUE));
  SER(pEdgeuse->GetCWEdgeuse()->NormalizedEvaluate(1.,TRUE,sEnd,NULL,TRUE));

  SmUVSectorIO               sSector;
  SmErrorCallbackFunctionPtr pfPrevious = SmThreadLocalStorage::GetErrCallbackFunction();
  s_bVertexuseAsserted = FALSE;
  smos_SetErrorCallback(sm_RecordVertexuseAssert);
  SmStatus sStatus = pVertexuse->ComputeUVSector(sSector,TRUE);
  smos_SetErrorCallback(pfPrevious);
  SER(sStatus);
  SER_MSG(!s_bVertexuseAsserted ? SM_SUCCESS : SM_ERR,_T("Missing-seam sector required a common UV corner"));
  if(   sSector.m_sBegEdgeUVPoint.DistanceBetween(SmPoint2d(sStart.x,sStart.y)) > SM_EFF_ZERO_PARAM
     || sSector.m_sEndEdgeUVPoint.DistanceBetween(SmPoint2d(sEnd.x,sEnd.y)) > SM_EFF_ZERO_PARAM
     || sSector.m_bSectorMissingSeamU != bMissingU
     || sSector.m_bSectorMissingSeamV != !bMissingU)
    { sSector.Dump(); SER_MSG(SM_ERR,_T("Missing-seam sector lost its endpoint locations or seam flag")); }
  return SM_SUCCESS;
}

SmStatus sm_InsertTorusVertexLoopSeams
 (SmContext                &rContext, // i/o: allocation context and Boolean state
  ULONG                     lCase,    // in : direct-insertion case number (0 through 19)
  SmTorusVertexLoopTestCase &rCase)   // i/o: fixture and expected seam topology
{
  SmBrep   *pBrep = rCase.m_pBrep;
  SmFace   *pFace = rCase.m_pFace;
  ULONG     lMode = lCase/10, lFixture = lCase%10;
  ULONG     lKind = lFixture/2;
  SmBoolean bFirstU = lFixture%2 == 0;
  SmTemporaryChangeValue<SmBoolean> sDoingBool(rContext.GetDoingBooleanRef(),TRUE);
  SmCurveClassification &rU        = rCase.m_sU;
  SmCurveClassification &rV        = rCase.m_sV;
  SmSurfParamType        &reCrossed = rCase.m_eCrossed;
  SmSurfParamType        &reMissing = rCase.m_eMissing;
  SmSurfParamType        &reNear    = rCase.m_eNear;
  ULONG lExpectedTotalEdges = rCase.m_lExpectedEdges;
  ULONG lExpectedVertices = rCase.m_lExpectedVertices;
  if(lMode == 0)
    {
      SmCurveClassification *pClasses[2] = {bFirstU ? &rU : &rV,bFirstU ? &rV : &rU};
      for(ULONG lSeam=0; lSeam<2; ++lSeam)
        {
          SmCurveClassification &rClass = *pClasses[lSeam];
          SmTArray<SmFace*>       sNewFaces;
          SmTArray<SmEdge*>       sNewEdges;
          SmTArray<SmVertex*>     sNewVertices;
          ULONG                  lExpectedEdges = lKind == 4 || (lKind == 1 && lSeam == 0) || (lKind == 2 && lSeam == 1) ? 3 : 1;
          SER_MSG(rClass.GetSize() == lExpectedEdges ? SM_SUCCESS : SM_ERR,_T("Seam was not divided at its vertices"));
          // Discard trims from the intermediate open domain, as SplitAtSeam
          // does before inserting the next seam through existing vertices.
          SER(pFace->RemoveUVTrimCurves());
          SmTrackTopologyChanges sChanges(pBrep,TRUE);
          SER(rClass.MergeIntervals
            (&sNewFaces,    // out: new faces
             &sNewEdges,    // out: new edges
             &sNewVertices, // out: new vertices
             pFace));       // in : original face
          sChanges.StopTracking();
          if(pBrep->GetNumFaces() != 1 || sNewFaces.GetSize() || sNewEdges.GetSize() != lExpectedEdges)
            { SER_MSG(SM_ERR,_T("Seam insertion split the torus face or lost a segment")); }
          if(!sChanges.IsInRmList(pFace) || !sChanges.IsInAddList(pFace)) { return SM_ERR; }
          if(lSeam == 0)
            {
              SmVertex *pCorner = rClass[0].m_vStart.GetVertexObject();
              NER(pCorner);
              SmTArray<SmVertexuse*> sCornerUses;
              pCorner->GetVertexusesOfFace(pFace,sCornerUses);
              if(sCornerUses.GetSize() != 4) { return SM_ERR; }
              for(ULONG lUse=0; lUse<sCornerUses.GetSize(); ++lUse)
                { SER(sm_CheckMissingSeamSector(sCornerUses[lUse],!bFirstU)); }
              SmCurveClassification &rSecond = *pClasses[1];
              rSecond[0].m_vStart.SetClassObject(SM_PC_VERTEX,pCorner);
              rSecond[rSecond.GetSize()-1].m_vEnd.SetClassObject(SM_PC_VERTEX,pCorner);
            }
        }
    }
  else if(lMode == 1)
    {
      SmTArray<SmFace*>   sNewFaces;
      SmTArray<SmEdge*>   sNewEdges;
      SmTArray<SmVertex*> sNewVertices;
      SER(pFace->SplitAtSeam
        (rU,              // i/o: U seam classification
         rV,              // i/o: V seam classification
         reCrossed,       // in : illegally crossed seams
         reMissing,       // in : missing seams
         reNear,          // in : boundaries near seams
         &sNewFaces,      // out: new faces
         &sNewEdges,      // out: new seam edges
         &sNewVertices)); // out: new seam vertices
      if(sNewFaces.GetSize() || sNewEdges.GetSize() != lExpectedTotalEdges-4
         || sNewVertices.GetSize() != (lKind == 3 ? 0UL : 1UL))
        { SER_MSG(SM_ERR,_T("SplitAtSeam lost the shared corner or its output topology")); }
    }
  if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != lExpectedTotalEdges
     || pBrep->GetNumVertices() != lExpectedVertices)
    { SER_MSG(SM_ERR,_T("Torus seam repair split the face or duplicated a seam vertex")); }
  return SM_SUCCESS;
}
