// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- smFaceTest.cpp
* PURPOSE --- Test face classification and derived UV data.
**********************************************************************/

#include "StdAfx.h"
#include <smFaceTest.h>
#include <smTestBrep.h>
#include <SmAssertArray.h>
#include <SmCrvOnSurf.h>
#include <SmFaceProps.h>
#include <SmPrimitiveCreation.h>
#include <SmTrimmingTools.h>
#include <SmTrackTopologyChanges.h>
#include <SmLine.h>

static SmStatus my_test_seam_uv_redistribution_at_poles()
{
  // A sphere with the positive octant removed has a three-arc boundary through
  // the north pole. The missing meridian seam joins that loop to the south pole.
  // Reverse the sphere axis to exercise both swapping and retaining the UV sides.
  for(int iAxis=0; iAxis<2; iAxis++)
    {
      SmContext sContext ;
      SmBrepConstructor sBuilder ;
      SmBrep * pBrep = sBuilder.StartBrep(sContext) ;
      NER(pBrep) ;
      SmObjDelete sCleanup(pBrep) ;
      pBrep->SetTolerance(1e-7, FALSE) ;
      NER(sBuilder.StartRegion(TRUE, NULL)) ;
      SmShell * pOuter = NULL, * pInner = NULL ;
      SER(sBuilder.StartShell(pOuter, pInner)) ;
      SmFace * pFace = NULL ;
      SmVertex * pVertices[4] = {NULL, NULL, NULL, NULL} ;
      SmEdge * pEdges[6] = {NULL, NULL, NULL, NULL, NULL, NULL} ;
      const SmPoint3d sPoints[4] = {SmPoint3d(0,0,0.5), SmPoint3d(0.5,0,0),
                                   SmPoint3d(0,0.5,0), SmPoint3d(0,0,0)} ;
      const int iStart[6] = {0,0,1,0,3,3}, iEnd[6] = {1,2,2,3,2,1} ;
      const SmVector3d sZ[3] = {SmVector3d(0,1,0), SmVector3d(-1,0,0), SmVector3d(0,0,1)} ;
      const SmVector3d sX[3] = {SmVector3d(0,0,1), SmVector3d(0,0,1), SmVector3d(1,0,0)} ;
      // Signed, one-based edge numbers in the primary faceuse's loop order.
      const int iUses[4][3] = {{-1,2,-3}, {2,-5,-4}, {1,-6,-4}, {3,-5,6}} ;
      const SmVector3d sPlaneZ[3] = {SmVector3d(-1,0,0), SmVector3d(0,1,0), SmVector3d(0,0,1)} ;
      for(int iFace=0; iFace<4; iFace++)
        {
          SmFace * pNewFace = sBuilder.StartFace(iFace == 1 ? SM_OT_OPPOSITE : SM_OT_SAME) ;
          NER(pNewFace) ;
          SmAxis2Placement sFrame ;
          if(iFace == 0)
            {
              pFace = pNewFace ;
              sFrame.SetCanonical(SmPoint3d(0,0,0), SmVector3d(-1,0,0), SmVector3d(0,iAxis == 0 ? -1 : 1,0)) ;
              SmSphere * pSphere = NULL ;
              SER(SmSphere::CreateCanonical(sContext, sFrame, 0.5, pSphere)) ;
              SER(sBuilder.SetFaceSurface(pSphere, pSphere->GetNaturalUVDomain())) ;
            }
          else
            {
              sFrame.SetCanonical(SmPoint3d(0,0,0), sX[iFace-1], sPlaneZ[iFace-1] * sX[iFace-1]) ;
              SmPlane * pPlane = NULL ;
              SER(SmPlane::CreateCanonical(sContext, sFrame, pPlane)) ;
              SER(pPlane->AdjustSTEPUVDomain(SmExtent2d(-1,-1,1,1))) ;
              SER(sBuilder.SetFaceSurface(pPlane, pPlane->GetNaturalUVDomain())) ;
            }
          NER(sBuilder.StartLoop(SM_OT_SAME)) ;
          for(int iUse=0; iUse<3; iUse++)
            {
              int iSignedEdge = iUses[iFace][iUse] ;
              int iEdge = (iSignedEdge > 0 ? iSignedEdge : -iSignedEdge) - 1 ;
              SmVertex *& pStart = pVertices[iStart[iEdge]] ;
              SmVertex *& pEnd = pVertices[iEnd[iEdge]] ;
              if(!pStart) { pStart = sBuilder.StartVertexOfLoop(sPoints[iStart[iEdge]]) ; }
              if(!pEnd) { pEnd = sBuilder.StartVertexOfLoop(sPoints[iEnd[iEdge]]) ; }
              NER(pStart) ;
              NER(pEnd) ;
              SmBoolean bNewEdge = pEdges[iEdge] == NULL ;
              pEdges[iEdge] = sBuilder.StartEdge(iSignedEdge > 0 ? SM_OT_SAME : SM_OT_OPPOSITE,
                                                pEdges[iEdge], pStart, pEnd) ;
              NER(pEdges[iEdge]) ;
              if(bNewEdge)
                {
                  if(iEdge < 3)
                    {
                      sFrame.SetCanonical(SmPoint3d(0,0,0), sX[iEdge], sZ[iEdge] * sX[iEdge]) ;
                      SmCircle * pCircle = NULL ;
                      SmExtent1d sArc(0,90) ;
                      SER(SmCircle::CreateCanonical(sContext, sFrame, 0.5, pCircle, &sArc)) ;
                      SER(sBuilder.SetEdgeCurve(pCircle, FALSE)) ;
                    }
                  else
                    {
                      SmBSplineCurve * pLine = NULL ;
                      SER(SmBSplineCurve::CreateLineSegment(sContext, 3, pStart->GetPoint(), pEnd->GetPoint(), pLine)) ;
                      SER(sBuilder.SetEdgeCurve(pLine, FALSE)) ;
                    }
                  SER(sBuilder.SetLimits(SmVertexNode(pStart->GetPoint()), SmVertexNode(pEnd->GetPoint()))) ;
                }
              SER(sBuilder.EndEdge()) ;
            }
          SER(sBuilder.EndLoop()) ;
          SER(sBuilder.EndFace()) ;
        }
      SER(sBuilder.EndShell(FALSE)) ;
      SER(sBuilder.EndRegion()) ;
      SER(sBuilder.EndBrep()) ;

      pBrep->m_bEditingEnabled = TRUE ;
      SmCurveClassification sU, sV ;
      SmSurfParamType eCrossed, eMissing, eNear ;
      if(   !pFace->HasSeamProblem(sU, sV, eCrossed, eMissing, eNear)
         || eCrossed != SM_SP_NEITHER || eMissing != SM_SP_U || eNear != SM_SP_NEITHER)
        { return SM_ERR ; }
      SER(pFace->SplitAtSeam(sU, sV, eCrossed, eMissing, eNear)) ;
      pBrep->m_bEditingEnabled = FALSE ;
      double dMeanCurve, dMaxCurve, dMeanVertex, dMaxVertex, dMeanUV, dMaxUV ;
      SER(pFace->CreateUVTrimCurves(FALSE, NULL, NULL, dMeanCurve, dMaxCurve,
                                   dMeanVertex, dMaxVertex, dMeanUV, dMaxUV)) ;

      SmTArray<SmFace*> sFaces ;
      SmTArray<SmEdge*> sEdges ;
      SmTArray<SmVertex*> sVertices ;
      SmTArray<SmLoop*> sLoops ;
      SmTArray<SmEdgeuse*> sUses ;
      pBrep->GetFaces(sFaces) ;
      pBrep->GetEdges(sEdges) ;
      pBrep->GetVertices(sVertices) ;
      pFace->GetLoops(sLoops) ;
      if(sFaces.GetSize() != 4 || sEdges.GetSize() != 7 || sVertices.GetSize() != 5 || sLoops.GetSize() != 1)
        { return SM_ERR ; }
      sLoops[0]->GetEdgeuses(sUses) ;
      SmOrientType eOrientation ;
      SER(pFace->ComputeLoopOrientation(sLoops[0], eOrientation)) ;
      if(sUses.GetSize() != 5 || eOrientation != SM_OT_SAME)
        { return SM_ERR ; }

      for(ULONG iFace=0; iFace<sFaces.GetSize(); iFace++)
        {
          SER(sFaces[iFace]->CreateUVTrimCurves(FALSE, NULL, NULL, dMeanCurve, dMaxCurve,
                                               dMeanVertex, dMaxVertex, dMeanUV, dMaxUV)) ;
        }
      if(!pBrep->IsManifoldSolid()) { return SM_ERR ; }
      SmAssertArray sAsserts ;
      SER(pBrep->ValidatePointers(&sAsserts)) ;
      if(   sAsserts.GetSize() != 0
         || !pBrep->AssertValid(&sAsserts, SM_LEVEL_0, SM_WALK)
         || !pBrep->AssertValid(&sAsserts, SM_LEVEL_2, SM_WALK)
         || sAsserts.GetSize() != 0)
        { return SM_ERR ; }
    }
  return SM_SUCCESS ;
}

/***********************************************************************
PURPOSE --- GetLoopCrossings on a planar sheet against degenerate and
  area inner boundaries.

  Classify line is y=10 across a 20x20 XY sheet [0,20]x[0,20].
    1. Empty sheet: outer crossed twice.
    2. Inner vertex loop at (10,10): point-boundary crack, count 0.
    3. Inner crack (strut edge) from (10,6) to (10,14): strut crossings
       are not loop crossings, count 0.
    4. Inner rectangular hole [6,14]x[6,14]: area loop crossed twice.
***********************************************************************/
static SmStatus sm_classify_loop_crossings
  (SmFace              * pFace,
   SmCurve             * pCurve,
   const SmContext     * cpContext,
   SmTArray<SmLoop*>   & rLoops,
   SmTArray<ULONG>     & rCounts)
{
  rLoops.ReSet() ;
  rCounts.ReSet() ;
  if(pFace == NULL || pCurve == NULL)
    { return SM_ERR ; }

  SmCurveClassification sCurveClass(pCurve,
                                    pCurve->GetNaturalInterval(),
                                    NULL,
                                    SmTol::GetZoneTol3d(pCurve),
                                    0, NULL, FALSE, TRUE,
                                    pFace->GetSurface()) ;
  sCurveClass.SetSurface1(pFace->GetSurface()) ;
  SER(pFace->CurveOnClassify(TRUE, sCurveClass, FALSE, TRUE, NULL, TRUE)) ;

  SmFaceProps sFaceProps(pFace, 0, cpContext) ;
  SER(sFaceProps.SetProps_Stage2(pFace, 0, SM_HO_ALL)) ;

  pFace->GetLoops(rLoops) ;
  SER(sCurveClass.GetLoopCrossings(rLoops, &sFaceProps, &rCounts, NULL, NULL)) ;
  return SM_SUCCESS ;
}

static SmStatus sm_expect_outer_inner_crossings
  (const SmTArray<SmLoop*> & crLoops,
   const SmTArray<ULONG>   & crCounts,
   ULONG                     lExpectedLoopCnt,
   ULONG                     lExpectedOuter,
   ULONG                     lExpectedInner)
{
  if(crLoops.GetSize() != lExpectedLoopCnt || crCounts.GetSize() != lExpectedLoopCnt)
    { return SM_ERR ; }

  ULONG lOuterSeen = 0 ;
  ULONG lInnerSeen = 0 ;
  for(ULONG ii=0; ii<crLoops.GetSize(); ii++)
    {
      if(crLoops[ii] == NULL)
        { return SM_ERR ; }
      if(crLoops[ii]->IsOuterLoop())
        {
          if(crCounts[ii] != lExpectedOuter)
            { return SM_ERR ; }
          lOuterSeen++ ;
        }
      else
        {
          if(crCounts[ii] != lExpectedInner)
            { return SM_ERR ; }
          lInnerSeen++ ;
        }
    }
  if(lOuterSeen != 1 || lInnerSeen != (lExpectedLoopCnt - 1))
    { return SM_ERR ; }
  return SM_SUCCESS ;
}

static SmStatus my_test_loop_orientation_periodic_winding()
{
  MYPRINTF(_T("\n********** Entered: my_test_loop_orientation_periodic_winding"));
  SmBoolean bAllPassed = TRUE;
  for(ULONG lCase=0; lCase<40; ++lCase)
    {
      // Test winding and contractible loops in U and V, traversed both ways
      // from every vertex. Include starts inside the domain and on either seam side.
      ULONG     lRotation = lCase/4;
      SmBoolean bReverse = (lCase%2) != 0, bSwapUV = (lCase%4) >= 2;
      SmBoolean bWinding = lRotation < 4;
      ULONG     lCount = bWinding ? 4 : 6;
      ULONG     lFirst = bWinding ? lRotation : lRotation-4;
      SmPoint2d sStarts[6] = {SmPoint2d(.1,.4),SmPoint2d(.4,.6),
                            SmPoint2d(.8,.6),SmPoint2d(0,.4)};
      SmPoint2d sEnds[6] = {SmPoint2d(.4,.6),SmPoint2d(.8,.6),
                          SmPoint2d(1,.4),SmPoint2d(.1,.4)};
      if(!bWinding)
        {
          sStarts[0].Set(.9,.4); sEnds[0].Set(1,.4);
          sStarts[1].Set(0,.4);  sEnds[1].Set(.1,.4);
          sStarts[2].Set(.1,.4); sEnds[2].Set(.1,.6);
          sStarts[3].Set(.1,.6); sEnds[3].Set(0,.6);
          sStarts[4].Set(1,.6);  sEnds[4].Set(.9,.6);
          sStarts[5].Set(.9,.6); sEnds[5].Set(.9,.4);
        }
      SmPoint2d sOrderedStarts[6],sOrderedEnds[6];
      for(ULONG lEdge=0; lEdge<lCount; ++lEdge)
        {
          ULONG     lIndex = (lFirst+(bReverse ? lCount-1-lEdge : lEdge))%lCount;
          SmPoint2d sStart = bReverse ? sEnds[lIndex] : sStarts[lIndex];
          SmPoint2d sEnd = bReverse ? sStarts[lIndex] : sEnds[lIndex];
          sOrderedStarts[lEdge] = bSwapUV ? SmPoint2d(sStart.y,sStart.x) : sStart;
          sOrderedEnds[lEdge] = bSwapUV ? SmPoint2d(sEnd.y,sEnd.x) : sEnd;
        }

      SmContext         sContext;
      SmBrepConstructor sBuilder;
      SmBrep           *pBrep = sBuilder.StartBrep(sContext);
      NER(pBrep);
      SmObjDelete sCleanup(pBrep);
      pBrep->SetTolerance(1e-7,FALSE);
      SmShell *pOuter=NULL,*pInner=NULL;
      SER(sBuilder.StartShell(pOuter,pInner));
      SmFace  *pFace = sBuilder.StartFace(SM_OT_SAME);
      SmTorus *pTorus = NULL;
      SER(SmTorus::CreateCanonical
        (sContext,           // in : allocation context
         SmAxis2Placement(), // in : torus placement
         2.0,                // in : major radius
         0.5,                // in : minor radius
         pTorus));           // out: torus surface
      SER(sBuilder.SetFaceSurface(pTorus,pTorus->GetNaturalUVDomain()));
      SmLoop   *pLoop = sBuilder.StartLoop(SM_OT_SAME);
      SmVertex *pVertices[6];
      for(ULONG lEdge=0; lEdge<lCount; ++lEdge)
        {
          SmPoint3d sPoint;
          SER(pTorus->EvaluatePoint(sOrderedStarts[lEdge],sPoint));
          pVertices[lEdge] = sBuilder.StartVertexOfLoop(sPoint);
        }
      for(ULONG lEdge=0; lEdge<lCount; ++lEdge)
        {
          ULONG lNext = (lEdge+1)%lCount;
          NER(sBuilder.StartEdge
            (SM_OT_SAME,         // in : edgeuse orientation
             NULL,               // in : create a new edge
             pVertices[lEdge],   // in : start vertex
             pVertices[lNext])); // in : end vertex
          SmLine sLine(sOrderedStarts[lEdge],sOrderedEnds[lEdge],2,&sContext);
          SmCrvOnSurf *pCurve = new(sContext) SmCrvOnSurf
            (sLine,      // in : defining UV curve
             *pTorus,    // in : base surface
             NULL,       // in : natural surface domain
             2,          // in : copy and own both inputs
             &sContext); // in : allocation context
          SER(sBuilder.SetEdgeCurve(pCurve,FALSE));
          SER(sBuilder.SetLimits(SmVertexNode(pVertices[lEdge]->GetPoint()),SmVertexNode(pVertices[lNext]->GetPoint())));
          SER(sBuilder.EndEdge());
        }
      SER(sBuilder.EndLoop());
      SER(sBuilder.EndFace());
      SER(sBuilder.EndShell(FALSE));
      SER(sBuilder.EndBrep());

      SmOrientType eOrientation = SM_OT_UNKNOWN;
      SmOrientType eExpected = bWinding ? SM_OT_UNKNOWN
        : bReverse != bSwapUV ? SM_OT_OPPOSITE : SM_OT_SAME;
      SER(pFace->ComputeLoopOrientation
        (pLoop,        // in : target loop
         eOrientation, // out: measured orientation
         NULL,         // out: optional UV bounds
         TRUE));       // in : use defining 3D curves
      if(eOrientation != eExpected)
        {
          TCHAR sMessage[SM_TBLOCK_SIZE];
          smos_sprintf(sMessage,_T("\nPeriodic orientation case %lu: got %d, expected %d"),lCase,eOrientation,eExpected);
          MYPRINTF(sMessage);
          bAllPassed = FALSE;
        }
    }
  if(!bAllPassed) { return SM_ERR; }
  MYPRINTF(_T("\nLoop orientation periodic winding: forty cases passed"));
  return SM_SUCCESS;
}

static SmStatus my_test_get_loop_crossings()
{
  MYPRINTF(_T("\n********** Entered: my_test_get_loop_crossings")) ;

  SmContext sContext ;
  SmBSplineCurve * pLine = NULL ;
  SER(SmBSplineCurve::CreateLineSegment(sContext, 3,
                                        SmPoint3d(0.0, 10.0, 0.0),
                                        SmPoint3d(20.0, 10.0, 0.0),
                                        pLine)) ;
  if(pLine == NULL)
    { return SM_ERR ; }
  SmObjDelete sCleanupLine(pLine) ;

  SmTArray<SmLoop*> sLoops ;
  SmTArray<ULONG> sCounts ;

  // 1. Empty sheet: outer loop crossed twice.
    {
      SmBrep * pBrep = sm_make_xy_sheet(sContext, 20.0) ;
      if(pBrep == NULL)
        { return SM_ERR ; }
      SmObjDelete sCleanupBrep(pBrep) ;

      SmTArray<SmFace*> sFaces ;
      pBrep->GetFaces(sFaces) ;
      if(sFaces.GetSize() != 1)
        { return SM_ERR ; }

      SER(sm_classify_loop_crossings(sFaces[0], pLine, &sContext, sLoops, sCounts)) ;
      SER(sm_expect_outer_inner_crossings(sLoops, sCounts, 1, 2, 0)) ;
      MYPRINTF(_T("\nGetLoopCrossings empty sheet: outer=2")) ;
    }

  // 2. Inner vertex loop: zero-length crack, not a crossing.
    {
      SmBrep * pBrep = sm_make_xy_sheet(sContext, 20.0) ;
      if(pBrep == NULL)
        { return SM_ERR ; }
      SmObjDelete sCleanupBrep(pBrep) ;

      SmTArray<SmFace*> sFaces ;
      pBrep->GetFaces(sFaces) ;
      if(sFaces.GetSize() != 1)
        { return SM_ERR ; }
      SmFace * pFace = sFaces[0] ;

      SmLoop * pVertexLoop = NULL ;
      SmVertex * pVertex = NULL ;
      SER(pBrep->MakeVertexLoop(pFace, SmPoint3d(10.0, 10.0, 0.0), pVertexLoop, pVertex)) ;
      if(   pVertexLoop == NULL
         || pVertexLoop->GetLoopuse() == NULL
         || pVertexLoop->GetLoopuse()->IsVertexLoopuse() == FALSE
         || pVertexLoop->IsOuterLoop())
        { return SM_ERR ; }

      SER(sm_classify_loop_crossings(pFace, pLine, &sContext, sLoops, sCounts)) ;
      SER(sm_expect_outer_inner_crossings(sLoops, sCounts, 2, 2, 0)) ;

      SmTArray<SmLoop*> sVertexOnly ;
      sVertexOnly.Add(pVertexLoop) ;
      SmCurveClassification sCurveClass(pLine,
                                        pLine->GetNaturalInterval(),
                                        NULL,
                                        SmTol::GetZoneTol3d(pLine),
                                        0, NULL, FALSE, TRUE,
                                        pFace->GetSurface()) ;
      sCurveClass.SetSurface1(pFace->GetSurface()) ;
      SER(pFace->CurveOnClassify(TRUE, sCurveClass, FALSE, TRUE, NULL, TRUE)) ;
      SmFaceProps sFaceProps(pFace, 0, &sContext) ;
      SER(sFaceProps.SetProps_Stage2(pFace, 0, SM_HO_ALL)) ;
      SmTArray<ULONG> sVertexCounts ;
      SER(sCurveClass.GetLoopCrossings(sVertexOnly, &sFaceProps, &sVertexCounts, NULL, NULL)) ;
      if(sVertexCounts.GetSize() != 1 || sVertexCounts[0] != 0)
        { return SM_ERR ; }

      MYPRINTF(_T("\nGetLoopCrossings vertex-loop: outer=2, vertex-loop=0")) ;
    }

  // 3. Inner crack: two vertex loops joined by MakeEdgeInFace. The classify
  //    line crosses the strut edge; strut crossings are not loop crossings.
    {
      SmBrep * pBrep = sm_make_xy_sheet(sContext, 20.0) ;
      if(pBrep == NULL)
        { return SM_ERR ; }
      SmObjDelete sCleanupBrep(pBrep) ;
      pBrep->m_bEditingEnabled = TRUE ;

      SmTArray<SmFace*> sFaces ;
      pBrep->GetFaces(sFaces) ;
      if(sFaces.GetSize() != 1)
        { return SM_ERR ; }
      SmFace * pFace = sFaces[0] ;

      SmLoop * pLoopA = NULL ;
      SmLoop * pLoopB = NULL ;
      SmVertex * pVA = NULL ;
      SmVertex * pVB = NULL ;
      SER(pBrep->MakeVertexLoop(pFace, SmPoint3d(10.0, 6.0, 0.0), pLoopA, pVA)) ;
      SER(pBrep->MakeVertexLoop(pFace, SmPoint3d(10.0, 14.0, 0.0), pLoopB, pVB)) ;

      SmBSplineCurve * pCrackCrv = NULL ;
      SER(SmBSplineCurve::CreateLineSegment(sContext, 3,
                                            pVA->GetPoint(),
                                            pVB->GetPoint(),
                                            pCrackCrv)) ;
      SmEdge * pCrackEdge = NULL ;
      SmLoop * pNewLoop = NULL ;
      SmFace * pNewFace = NULL ;
      SER(pBrep->MakeEdgeInFace(pFace, pVA, pVB,
                                pCrackCrv, NULL, pCrackCrv->GetNaturalInterval(),
                                SM_OT_SAME, SM_ZONE_TOL_3D,
                                pCrackEdge, pNewLoop, pNewFace)) ;
      if(pCrackEdge == NULL || pNewFace != NULL)
        { return SM_ERR ; }

      SER(sm_classify_loop_crossings(pFace, pLine, &sContext, sLoops, sCounts)) ;
      SER(sm_expect_outer_inner_crossings(sLoops, sCounts, 2, 2, 0)) ;

      SmBoolean bSawCrack = FALSE ;
      for(ULONG ii=0; ii<sLoops.GetSize(); ii++)
        {
          if(sLoops[ii]->IsOuterLoop())
            { continue ; }
          SmLoopuse * pLoopuse = sLoops[ii]->GetLoopuse() ;
          if(pLoopuse == NULL || pLoopuse->IsVertexLoopuse())
            { return SM_ERR ; }
          bSawCrack = TRUE ;
        }
      if(bSawCrack == FALSE)
        { return SM_ERR ; }

      MYPRINTF(_T("\nGetLoopCrossings crack: outer=2, crack=0")) ;
    }

  // 4. Inner rectangular hole: area loop crossed twice.
    {
      SmBrep * pBrep = sm_make_xy_sheet(sContext, 20.0) ;
      if(pBrep == NULL)
        { return SM_ERR ; }
      SmObjDelete sCleanupBrep(pBrep) ;

      SmAxis2Placement sHoleFrame ;
      sHoleFrame.SetCanonical(SmPoint3d(6.0, 6.0, 0.0),
                              SmVector3d(1.0, 0.0, 0.0),
                              SmVector3d(0.0, 1.0, 0.0)) ;
      SmBrep * pHole = SmPrimitiveCreation::CreateRectangle(sContext, 1.0e-5, 8.0, 8.0, sHoleFrame) ;
      if(pHole == NULL)
        { return SM_ERR ; }

      SER(SmPrimitiveCreation::Boolean2D(pBrep, pHole, SM_2D_DIFFERENCE, pBrep)) ;

      SmTArray<SmFace*> sFaces ;
      pBrep->GetFaces(sFaces) ;
      if(sFaces.GetSize() != 1)
        { return SM_ERR ; }

      SER(sm_classify_loop_crossings(sFaces[0], pLine, &sContext, sLoops, sCounts)) ;
      SER(sm_expect_outer_inner_crossings(sLoops, sCounts, 2, 2, 2)) ;
      MYPRINTF(_T("\nGetLoopCrossings inner hole: outer=2, inner=2")) ;
    }

  return SM_SUCCESS ;
}

/***********************************************************************
PURPOSE --- SmFace::AssertValid tests 37 and 38: Face Loops must be nested
  and disjoint, and the two tests must cover each other's blind spots.

  Test 37 (SM_LEVEL_0) classifies one sampled UV point per InnerLoop.
  Its fixture is a 20x20 XY sheet cut back to x<=12 and holding a 6x8
  hole at [2,8]x[6,14].  Cutting the sheet back leaves the SmPlane with
  its full UVDomain, so there is UV room to the right of the OuterLoop.
  Sliding the hole UVTrimCurves into that room leaves an InnerLoop that
  no longer nests, the shape that reaches SmTess as a
  CheckPolygonNesting() failure.

  Test 38 (SM_LEVEL_2) intersects the Loop UVTrimCurves pairwise.  Its
  fixture is a 20x20 XY sheet with two holes, one of them slid in UV
  until it meets the other.  Two Loops of one Face must be disjoint, so
  both a corner overlap and a bare corner touch are reported - the touch
  is where the two holes would share a Vertex.  Every Loop's sampled UV
  point still classifies correctly in both, so test 37 is fooled and only
  the curve/curve sweep sees the contact.

  Test 39 (SM_LEVEL_0) compares the Loop Vertices pairwise.  Tests 37 and
  38 both work in UVSpace and only ever see a shared Vertex as geometry,
  so neither states the topological rule that two Loops of one Face may
  not share a Vertex.  The fixture cuts two holes whose corners sit 0.01
  apart and calls SmBrep::GlueVertices() on that corner pair, which leaves
  one SmVertex reached from both InnerLoops - improperly connected Loops.

  Tests 38 and 39 are checked in both directions to show neither subsumes
  the other.  Case 5 moves UVTrimCurves only, so the Loops touch in
  UVSpace while keeping their own Vertices: 38 reports and 39 is silent.
  Case 6 moves Vertexuses only, so the Loops share a Vertex while their
  UVTrimCurves stay 0.01 apart: 39 reports and 38 is silent.
***********************************************************************/
static SmStatus sm_cut_rectangle_from_sheet(const SmContext & crContext, SmBrep *& rpBrep,
                                            double dCornerX, double dCornerY,
                                            double dSizeX, double dSizeY)
{
  SmAxis2Placement sCutFrame ;
  sCutFrame.SetCanonical(SmPoint3d(dCornerX, dCornerY, 0.0),
                         SmVector3d(1.0, 0.0, 0.0),
                         SmVector3d(0.0, 1.0, 0.0)) ;
  SmBrep * pCutter = SmPrimitiveCreation::CreateRectangle(crContext, 1.0e-5, dSizeX, dSizeY, sCutFrame) ;
  if(pCutter == NULL)
    { return SM_ERR ; }

  return SmPrimitiveCreation::Boolean2D(rpBrep, pCutter, SM_2D_DIFFERENCE, rpBrep) ;
}

static SmFace * sm_make_cutback_sheet_with_hole(const SmContext & crContext, SmBrep *& rpBrep)
{
  rpBrep = sm_make_xy_sheet(crContext, 20.0) ;
  if(rpBrep == NULL)
    { return NULL ; }

  // cut the sheet back to x <= 12, shrinking the OuterLoop but not the SmPlane
  if(sm_cut_rectangle_from_sheet(crContext, rpBrep, 12.0, -2.0, 20.0, 24.0) != SM_SUCCESS)
    { return NULL ; }

  if(sm_cut_rectangle_from_sheet(crContext, rpBrep, 2.0, 6.0, 6.0, 8.0) != SM_SUCCESS)
    { return NULL ; }

  SmTArray<SmFace*> sFaces ;
  rpBrep->GetFaces(sFaces) ;
  if(sFaces.GetSize() != 1)
    { return NULL ; }
  return sFaces[0] ;
}

// eff: build a 20x20 XY sheet holding a 6x8 hole at [2,8]x[6,14] and a 6x6 hole
//      at [13,19]x[13,19], far enough apart that the second hole's UVTrimCurves
//      can be slid over the first hole
static SmFace * sm_make_sheet_with_two_holes(const SmContext & crContext, SmBrep *& rpBrep)
{
  rpBrep = sm_make_xy_sheet(crContext, 20.0) ;
  if(rpBrep == NULL)
    { return NULL ; }

  if(sm_cut_rectangle_from_sheet(crContext, rpBrep, 2.0, 6.0, 6.0, 8.0) != SM_SUCCESS)
    { return NULL ; }

  if(sm_cut_rectangle_from_sheet(crContext, rpBrep, 13.0, 13.0, 6.0, 6.0) != SM_SUCCESS)
    { return NULL ; }

  SmTArray<SmFace*> sFaces ;
  rpBrep->GetFaces(sFaces) ;
  if(sFaces.GetSize() != 1)
    { return NULL ; }
  return sFaces[0] ;
}

// rtn: the Face InnerLoop whose Vertices all sit in x [dMinX,dMaxX], NULL when there is none
static SmLoop * sm_find_inner_loop_in_x_range(const SmFace * pFace, double dMinX, double dMaxX)
{
  SmTArray<SmLoop*> sLoops ;
  pFace->GetLoops(sLoops) ;

  for(ULONG ii=0; ii<sLoops.GetSize(); ii++)
    {
      if(sLoops[ii] == NULL || sLoops[ii]->IsOuterLoop())
        { continue ; }

      SmTArray<SmVertex*> sVertices ;
      sLoops[ii]->GetVertices(sVertices) ;
      if(sVertices.GetSize() == 0)
        { continue ; }

      SmBoolean bInRange = TRUE ;
      for(ULONG jj=0; jj<sVertices.GetSize(); jj++)
        {
          double dX = sVertices[jj]->GetPoint().x ;
          if(dX < dMinX || dX > dMaxX)
            { bInRange = FALSE ; break ; }
        }

      if(bInRange)
        { return sLoops[ii] ; }
    }
  return NULL ;
}

// eff: translate every UVTrimCurve of pLoop, leaving the 3d Edge curves alone
static SmStatus sm_slide_loop_uv(SmLoop * pLoop, double dDeltaU, double dDeltaV)
{
  SmAxis2Placement sSlide ;
  sSlide.SetCanonical(SmPoint3d(dDeltaU, dDeltaV, 0.0),
                      SmVector3d(1.0, 0.0, 0.0),
                      SmVector3d(0.0, 1.0, 0.0)) ;

  SmTArray<SmEdgeuse*> sEdgeuses ;
  pLoop->GetEdgeuses(sEdgeuses) ;
  if(sEdgeuses.GetSize() == 0)
    { return SM_ERR ; }

  for(ULONG ii=0; ii<sEdgeuses.GetSize(); ii++)
    {
      SmBSplineCurve * pUVTrimCurve = NULL ;
      SER(sEdgeuses[ii]->GetOrCreateUVTrimCurve(pUVTrimCurve)) ;
      if(pUVTrimCurve == NULL)
        { return SM_ERR ; }
      SER(pUVTrimCurve->Transform(sSlide)) ;
    }
  return SM_SUCCESS ;
}

// rtn: TRUE when SmFace::AssertValid test 37 reports pFace
static SmBoolean sm_face_reports_bad_loop_nesting(const SmFace * pFace)
{
  SmTArray<ULONG> sTestRequests ;
  sTestRequests.Add(37) ;

  SmAssertArray sAsserts ;
  pFace->AssertValid(&sAsserts, SM_LEVEL_GIVEN, SM_NO_WALK, &sTestRequests) ;
  return sm_HasAssertReport(sAsserts, 37, pFace) ;
}

// rtn: TRUE when SmFace::AssertValid test 38 reports pFace
static SmBoolean sm_face_reports_intersecting_loops(const SmFace * pFace)
{
  SmTArray<ULONG> sTestRequests ;
  sTestRequests.Add(38) ;

  SmAssertArray sAsserts ;
  pFace->AssertValid(&sAsserts, SM_LEVEL_GIVEN, SM_NO_WALK, &sTestRequests) ;
  return sm_HasAssertReport(sAsserts, 38, pFace) ;
}

// rtn: TRUE when SmFace::AssertValid test 39 reports pFace
static SmBoolean sm_face_reports_connected_loops(const SmFace * pFace)
{
  SmTArray<ULONG> sTestRequests ;
  sTestRequests.Add(39) ;

  SmAssertArray sAsserts ;
  pFace->AssertValid(&sAsserts, SM_LEVEL_GIVEN, SM_NO_WALK, &sTestRequests) ;
  return sm_HasAssertReport(sAsserts, 39, pFace) ;
}

// eff: build a 20x20 XY sheet holding a 6x6 hole at [2,8]x[6,12] and a 6x6 hole
//      at [8.01,14.01]x[12.01,18.01].  The two holes stay disjoint, but one
//      corner of each sits 0.01 away in x and y from a corner of the other, so
//      the two corner Vertices are close enough to glue into one.
static SmFace * sm_make_sheet_with_two_near_holes(const SmContext & crContext, SmBrep *& rpBrep)
{
  rpBrep = sm_make_xy_sheet(crContext, 20.0) ;
  if(rpBrep == NULL)
    { return NULL ; }

  if(sm_cut_rectangle_from_sheet(crContext, rpBrep, 2.0, 6.0, 6.0, 6.0) != SM_SUCCESS)
    { return NULL ; }

  if(sm_cut_rectangle_from_sheet(crContext, rpBrep, 8.01, 12.01, 6.0, 6.0) != SM_SUCCESS)
    { return NULL ; }

  SmTArray<SmFace*> sFaces ;
  rpBrep->GetFaces(sFaces) ;
  if(sFaces.GetSize() != 1)
    { return NULL ; }
  return sFaces[0] ;
}

// rtn: the Loop Vertex closest to crPoint, NULL when the Loop has no Vertices
static SmVertex * sm_find_loop_vertex_near(const SmLoop * pLoop, const SmPoint3d & crPoint)
{
  SmTArray<SmVertex*> sVertices ;
  pLoop->GetVertices(sVertices) ;

  SmVertex * pClosest = NULL ;
  double     dClosest = SM_BIG_DOUBLE ;

  for(ULONG ii=0; ii<sVertices.GetSize(); ii++)
    {
      if(sVertices[ii] == NULL)
        { continue ; }

      double dDistSq = sVertices[ii]->GetPoint().DistanceBetweenSquared(crPoint) ;
      if(dDistSq < dClosest)
        {
          dClosest = dDistSq ;
          pClosest = sVertices[ii] ;
        }
    }
  return pClosest ;
}

static SmStatus my_test_face_loop_nesting()
{
  MYPRINTF(_T("\n********** Entered: my_test_face_loop_nesting")) ;

  SmContext sContext ;

  // 1. Hole inside the cut back sheet: properly nested, nothing to report.
    {
      SmBrep * pBrep = NULL ;
      SmFace * pFace = sm_make_cutback_sheet_with_hole(sContext, pBrep) ;
      SmObjDelete sCleanupBrep(pBrep) ;
      if(pFace == NULL)
        { return SM_ERR ; }

      if(sm_face_reports_bad_loop_nesting(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 37: nested hole passes")) ;
    }

  // 2. Same hole slid into the UV room beyond the cut back OuterLoop.
    {
      SmBrep * pBrep = NULL ;
      SmFace * pFace = sm_make_cutback_sheet_with_hole(sContext, pBrep) ;
      SmObjDelete sCleanupBrep(pBrep) ;
      if(pFace == NULL)
        { return SM_ERR ; }

      SmTArray<SmLoop*> sLoops ;
      pFace->GetLoops(sLoops) ;
      if(sLoops.GetSize() != 2)
        { return SM_ERR ; }

      // the SmPlane spans 20mm over a [0,1] UVDomain, so 0.5 slides the hole 10mm right
      SmLoop * pInnerLoop = sLoops[0]->IsOuterLoop() ? sLoops[1] : sLoops[0] ;
      SER(sm_slide_loop_uv(pInnerLoop, 0.5, 0.0)) ;

      if(!sm_face_reports_bad_loop_nesting(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 37: unnested hole reported")) ;
    }

  // 3. Two holes in an uncut sheet: the Loops are disjoint, nothing to report.
    {
      SmBrep * pBrep = NULL ;
      SmFace * pFace = sm_make_sheet_with_two_holes(sContext, pBrep) ;
      SmObjDelete sCleanupBrep(pBrep) ;
      if(pFace == NULL)
        { return SM_ERR ; }

      if(sm_face_reports_intersecting_loops(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 38: disjoint holes pass")) ;
    }

  // 4. Far hole slid until its UV corner overlaps the near hole's UV corner.
    {
      SmBrep * pBrep = NULL ;
      SmFace * pFace = sm_make_sheet_with_two_holes(sContext, pBrep) ;
      SmObjDelete sCleanupBrep(pBrep) ;
      if(pFace == NULL)
        { return SM_ERR ; }

      SmLoop * pFarLoop = sm_find_inner_loop_in_x_range(pFace, 13.0, 19.0) ;
      if(pFarLoop == NULL)
        { return SM_ERR ; }

      // the SmPlane spans 20mm over a [0,1] UVDomain, so the near hole covers
      // UV [0.1,0.4]x[0.3,0.7] and the far hole covers UV [0.65,0.95]x[0.65,0.95].
      // -0.30 in U leaves the two hole corners overlapping on [0.35,0.4]x[0.65,0.7],
      // while every Loop's sampled edgeuse midpoint keeps classifying correctly.
      SER(sm_slide_loop_uv(pFarLoop, -0.30, 0.0)) ;

      if(sm_face_reports_bad_loop_nesting(pFace))
        { return SM_ERR ; }
      if(!sm_face_reports_intersecting_loops(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 38: crossing holes reported where 37 is fooled")) ;
    }

  // 5. Near hole slid until one UV corner lands exactly on a far hole UV corner.
  //    Two Loops of one Face may not share a Vertex, so the bare touch counts.
    {
      SmBrep * pBrep = NULL ;
      SmFace * pFace = sm_make_sheet_with_two_holes(sContext, pBrep) ;
      SmObjDelete sCleanupBrep(pBrep) ;
      if(pFace == NULL)
        { return SM_ERR ; }

      SmLoop * pNearLoop = sm_find_inner_loop_in_x_range(pFace, 2.0, 8.0) ;
      if(pNearLoop == NULL)
        { return SM_ERR ; }

      // +0.25 in U and -0.05 in V carry the near hole's UV [0.4,0.7] corner onto
      // the far hole's UV [0.65,0.65] corner, leaving the near hole spanning
      // UV [0.35,0.65]x[0.25,0.65].  The two holes then meet at that one UV
      // point and nowhere else, so no sampled edgeuse midpoint lands in the
      // other hole and test 37 still passes.
      SER(sm_slide_loop_uv(pNearLoop, 0.25, -0.05)) ;

      if(sm_face_reports_bad_loop_nesting(pFace))
        { return SM_ERR ; }
      if(!sm_face_reports_intersecting_loops(pFace))
        { return SM_ERR ; }

      // sliding UVTrimCurves left the two holes with their own Vertices, so the
      // Loops meet in UVSpace without being connected - test 39 has nothing to say
      if(sm_face_reports_connected_loops(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 38: holes touching at one UV point reported, 39 silent")) ;
    }

  // 6. Two InnerLoops glued into one shared corner Vertex: improperly connected
  //    Loops.  Unlike case 5 this is a topological defect - one SmVertex really
  //    is reached from both Loops - so test 39 sees it without any UV geometry.
    {
      SmBrep * pBrep = NULL ;
      SmFace * pFace = sm_make_sheet_with_two_near_holes(sContext, pBrep) ;
      SmObjDelete sCleanupBrep(pBrep) ;
      if(pFace == NULL)
        { return SM_ERR ; }

      SmLoop * pNearLoop = sm_find_inner_loop_in_x_range(pFace, 1.0, 8.005) ;
      SmLoop * pFarLoop  = sm_find_inner_loop_in_x_range(pFace, 8.005, 14.5) ;
      if(pNearLoop == NULL || pFarLoop == NULL || pNearLoop == pFarLoop)
        { return SM_ERR ; }

      // two disjoint holes share no Vertex yet
      if(sm_face_reports_connected_loops(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 39: disjoint holes pass")) ;

      // glue the near hole's [8,12] corner to the far hole's [8.01,12.01] corner,
      // leaving one Vertex that both InnerLoops reach
      SmVertex * pSurvivor = sm_find_loop_vertex_near(pNearLoop, SmPoint3d(8.0,  12.0,  0.0)) ;
      SmVertex * pToDelete = sm_find_loop_vertex_near(pFarLoop,  SmPoint3d(8.01, 12.01, 0.0)) ;
      if(pSurvivor == NULL || pToDelete == NULL || pSurvivor == pToDelete)
        { return SM_ERR ; }

      SER(pBrep->GlueVertices(pSurvivor, pToDelete)) ;

      if(!sm_face_reports_connected_loops(pFace))
        { return SM_ERR ; }

      // the glue moved Vertexuses, not curves, so the two holes' UVTrimCurves
      // still stand 0.01 apart - test 38 cannot see this defect at all
      if(sm_face_reports_intersecting_loops(pFace))
        { return SM_ERR ; }
      MYPRINTF(_T("\nOKAY: AssertValid Face 39: Loops sharing a Vertex reported, 38 silent")) ;
    }

  return SM_SUCCESS ;
}

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
/***********************************************************************
PURPOSE --- Exercise loop closure without healing or regenerating the input trims.
***********************************************************************/
static SmStatus my_check_loop_uv_gap
 (const SmContext & crContext,      // in : context for test objects
  SmSurface       * pSurface,      // in : consumed test surface
  const SmPoint2d & crStartUV,      // in : start of the first edge
  const SmPoint2d & crEndUV,        // in : end of the last edge
  double            dEdgeZone,     // in : zone tolerance of each edge
  SmBoolean         bExpectedValid)// in : expected UV closure result
{
    SmBrepData sData;
    sData.m_sZoneTol3d = 1e-5;
    sData.m_lNumRegions = 1;
    sData.m_vSurfaces.Add(pSurface);
    SmRegionData sRegion;
    sRegion.m_bIsVoidFlag = TRUE;
    sRegion.m_lStartShell = 0;
    sRegion.m_lNumShells = 1;
    sData.m_vRegions.Add(sRegion);
    SmShellData sShell;
    sShell.m_lShellType = 0;
    sShell.m_lFaceuseStart = 0;
    sShell.m_lNumFaceuses = 2;
    sData.m_vShells.Add(sShell);
    for (int nSide = 0; nSide < 2; ++nSide)
    {
        SmFaceuseData sUse;
        sUse.m_lFace = 0;
        sUse.m_bOrientation = nSide == 0;
        sData.m_vFaceuses.Add(sUse);
    }
    const SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
    SmFaceData sFace;
    sFace.m_lSurface = 0;
    sFace.m_sZoneTol3d = 1e-5;
    sFace.m_vUVDomain = sDomain;
    sFace.m_lStartLoop = 0;
    sFace.m_lNumLoops = 1;
    sData.m_vFaces.Add(sFace);
    SmLoopData sLoop;
    sLoop.m_lLoopType = 0;
    sLoop.m_lStartEU = 0;
    sLoop.m_lNumEU = 3;
    sData.m_vLoops.Add(sLoop);
    const SmPoint2d asUV[] = { crStartUV, sDomain.Evaluate(.8, .2), sDomain.Evaluate(.7, .8), crEndUV };
    for (ULONG lEdge = 0; lEdge < 3; ++lEdge)
    {
        SmVertexData sVertex;
        SER(pSurface->EvaluatePoint(asUV[lEdge], sVertex.m_vPoint));
        sVertex.m_sZoneTol3d = dEdgeZone;
        sData.m_vVertices.Add(sVertex);
        SmBSplineCurve * pUV = NULL;
        SER(SmBSplineCurve::CreateLineSegment(
            crContext,                                  // in : context for test objects
            2,                                          // in : UV dimension
            SmPoint3d(asUV[lEdge].x, asUV[lEdge].y, 0),   // in : curve start
            SmPoint3d(asUV[lEdge+1].x, asUV[lEdge+1].y, 0),// in : curve end
            pUV                                         // out: trim curve
        ));
        sData.m_vUVCurves.Add(pUV);
        sData.m_v3DCurves.Add(new (crContext) SmCrvOnSurf(
            *pUV,        // in : UV curve defining the 3D edge
            *pSurface,   // in : supporting surface
            &sDomain,    // in : surface domain
            1,           // in : own a copy of the UV curve, borrow the surface
            &crContext   // in : context for test objects
        ));
        SmEdgeData sEdge;
        sEdge.m_lCurve = lEdge;
        sEdge.m_lPrimEU = lEdge;
        sEdge.m_lStartVertex = lEdge;
        sEdge.m_lEndVertex = (lEdge + 1) % 3;
        sEdge.m_vInterval = pUV->GetNaturalInterval();
        sEdge.m_sZoneTol3d = dEdgeZone;
        sData.m_vEdges.Add(sEdge);
        SmEUData sUse;
        sUse.m_lEUType = 0;
        sUse.m_lEdge = lEdge;
        sUse.m_lLoop = 0;
        sUse.m_bOrientation = TRUE;
        sUse.m_lUVCurve = lEdge;
        sUse.m_lNextEU = -static_cast<long>(lEdge) - 1;
        sUse.m_lMateNextEU = lEdge;
        sData.m_vEdgeuses.Add(sUse);
    }
    SmBrep * pBrep = new (crContext) SmBrep();
    SmObjDelete sDeleteBrep(pBrep);
    SmTArray<SmAttribute*> sAttributes;
    SER(pBrep->MakeTopologyFromData(
        &sData,       // in : exact geometry and topology records
        sAttributes,  // i/o: attributes
        FALSE,        // in : do not generate trims
        FALSE,        // in : do not replace geometry with analytics
        TRUE,         // in : retain the supplied tolerances
        FALSE,        // in : defer trim checking to the test
        FALSE,        // in : do not request healing
        FALSE         // in : disable healing
    ));
    SmLoop * pLoop = sData.m_vLoops[0].m_pLoop;
    SmAssertArray sReports;
    ULONG lClosure;
    const SmStatus eStatus = SmTrimmingTools::CheckLoop(
        pLoop->GetFace()->GetSurface(), // in : target loop surface
        pLoop,                         // in : target loop
        1e-5,                          // in : singularity tolerance
        lClosure,                      // out: loop closure classification
        &sReports,                     // i/o: validation reports
        SM_LEVEL_2                     // in : all validation levels
    );
    SmBoolean bUVReport = FALSE;
    for (ULONG lReport = 0; lReport < sReports.GetSize(); ++lReport)
    {
        bUVReport |= sReports[lReport]->GetListIndex() == 3 &&
                     (sReports[lReport]->GetTestIndex() == 1 || sReports[lReport]->GetTestIndex() == 2);
    }
    const SmBoolean bValid = eStatus == SM_SUCCESS && lClosure < 2 && !bUVReport;
    if (bValid != bExpectedValid)
    {
        SM_ASSERT_MSG(FALSE, _T("Loop UV-gap validation disagrees with the bounded neighborhood or seam guard"));
        return SM_ERR;
    }
    // Check the validator entrypoint independently so the direct call cannot
    // supply a missing report. AssertValid currently records CheckLoop failures
    // without folding them into its Boolean return; require TRUE for valid cases
    // and check the UV reports for both valid and invalid cases.
    SmAssertArray sAssertReports;
    const SmBoolean bAssertValid = pLoop->AssertValid(&sAssertReports, SM_LEVEL_2, SM_NO_WALK);
    SmBoolean bAssertUVReport = FALSE;
    for (ULONG lReport = 0; lReport < sAssertReports.GetSize(); ++lReport)
    {
        bAssertUVReport |= sAssertReports[lReport]->GetListIndex() == 3 &&
                           (sAssertReports[lReport]->GetTestIndex() == 1 || sAssertReports[lReport]->GetTestIndex() == 2);
    }
    if ((bExpectedValid && !bAssertValid) || bAssertUVReport == bExpectedValid)
    {
        SM_ASSERT_MSG(FALSE, _T("Loop AssertValid disagrees with the bounded neighborhood or seam guard"));
        return SM_ERR;
    }
    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- A small near-pole gap may span a large nonperiodic UV interval.
            Keep physical-gap, interior-excursion, and periodic-seam failures.
***********************************************************************/
static SmStatus my_test_loop_uv_gap_neighborhood()
{
    SmContext sContext;
    // Reduced rational bicubic patch from SHAPE face #46578479.
    const double aadPoints[16][4] = {
        { -75.52350209986, -29.40907447757, -0.02903593761593, 1.336660247407 },
        { -75.52350209986, -29.40907447757001, -0.02903593761593, 0.7735687303299 },
        { -75.52350209986, -29.40907447757, -0.02903593761593, 0.7170092890336 },
        { -75.52350209986, -29.40907447757, -0.02903593761593, 1.166981923518 },
        { -75.517601183, -29.29067419864, -0.2296546081682, 1.018970918309 },
        { -75.51760101133, -29.49423342932, -0.3421600656059, 0.5897115897985 },
        { -75.51760084171001, -29.69537420033, -0.2071049728044, 0.5465948546757 },
        { -75.51760089530001, -29.63182756749, 0.03564256517874, 0.8896207129399 },
        { -75.33092321605, -29.23456048059, -0.3314884753882, 1.092070110926 },
        { -75.33092295951, -29.53874999253, -0.4996114343651, 0.6320164684919 },
        { -75.33092270602, -29.83932547006, -0.2977913418667, 0.5858066141551 },
        { -75.33092278610999, -29.74436431713, 0.06495936640571, 0.9534405479153 },
        { -75.12960303577, -29.26668903197, -0.2733521841475, 1.555957825257 },
        { -75.12960282774, -29.51336589164, -0.4096883888209, 0.90048336641 },
        { -75.12960262215999, -29.75711201695, -0.2460261168409, 0.8346445674721 },
        { -75.12960268712, -29.68010502599, 0.04813986209213, 1.358441428445 },
    };
    const double adUKnots[] = { -.01253127294337, -.01253127294337, -.01253127294337, -.01253127294337,
                               1.248394544457, 1.248394544457, 1.248394544457, 1.248394544457 };
    const double adVKnots[] = { -.416060457619, -.416060457619, -.416060457619, -.416060457619,
                               1.232034003285, 1.232034003285, 1.232034003285, 1.232034003285 };
    for (int nCase = 0; nCase < 5; ++nCase)
    {
        SmBSplineSurface * pSurface = new (sContext) SmBSplineSurface();
        SER(pSurface->SetExpert(
            3,                          // in : U degree
            3,                          // in : V degree
            SM_SF_UNSPECIFIED,           // in : surface form
            SM_EK_CLAMPPED,              // in : full clamped knot vectors
            8,                          // in : U knot count
            adUKnots,                   // in : U knots
            8,                          // in : V knot count
            adVKnots,                   // in : V knots
            SM_CP_EUCLIDIAN_RATIONAL,    // in : Euclidean points with weights
            16,                         // in : U stride in doubles
            4,                          // in : V stride in doubles
            &aadPoints[0][0]            // in : control-point net
        ));
        SmPoint2d sStart(.01379541752769611, -.1745106431752232);
        SmPoint2d sEnd(.00451503652006906, -.416060457619);
        // Zero is reproduced exactly by the endpoint evaluation of both trim
        // lines, so these cases reach the zero-width isoparametric-curve path.
        if (nCase >= 3)
            sStart.x = sEnd.x = 0.0;
        if (nCase == 1 || nCase == 4)
        {
            SER(pSurface->SwapUV());
            sStart.Set(sStart.y, sStart.x);
            sEnd.Set(sEnd.y, sEnd.x);
        }
        SER(my_check_loop_uv_gap(
            sContext,                                    // in : context for test objects
            pSurface,                                    // in : consumed test surface
            sStart,                                      // in : start of the first edge
            sEnd,                                        // in : end of the last edge
            nCase == 2 ? .0005 : .002952363602285994,      // in : zone tolerance of each edge
            nCase != 2                                   // in : expected UV closure result
        ));
    }
    // Empty weights are valid for both patches and isoparametric curves. A
    // raised middle control row rejects each path despite close endpoints.
    const double adBulgeU[] = {0,0,0,1,1,1};
    const double adBulgeV[] = {0,0,1,1};
    for (int nCase = 0; nCase < 4; ++nCase)
    {
        const SmBoolean bBulge = nCase == 0 || nCase == 3;
        const double dHeight = bBulge ? 1. : 0.;
        const double aadNonRationalPoints[6][3] = {
            {0,0,0}, {0,.01,0}, {.005,0,dHeight}, {.005,.01,dHeight}, {.01,0,0}, {.01,.01,0}
        };
        SmBSplineSurface * pSurface = new (sContext) SmBSplineSurface();
        SER(pSurface->SetExpert(
            2,                      // in : U degree
            1,                      // in : V degree
            SM_SF_UNSPECIFIED,       // in : surface form
            SM_EK_CLAMPPED,          // in : full clamped knot vectors
            6,                      // in : U knot count
            adBulgeU,               // in : U knots
            4,                      // in : V knot count
            adBulgeV,               // in : V knots
            SM_CP_NON_RATIONAL,     // in : nonrational control points
            6,                      // in : U stride in doubles
            3,                      // in : V stride in doubles
            &aadNonRationalPoints[0][0] // in : control-point net
        ));
        SER(my_check_loop_uv_gap(
            sContext,                           // in : context for test objects
            pSurface,                           // in : consumed test surface
            SmPoint2d(0,nCase < 2 ? .2 : .5),    // in : start of the first edge
            SmPoint2d(1,nCase < 2 ? .8 : .5),    // in : end of the last edge
            .01,                                // in : zone tolerance of each edge
            !bBulge                             // in : expected UV closure result
        ));
    }
    for (int nPole = 0; nPole < 3; ++nPole)
    {
        SmSphere * pSphere = new (sContext) SmSphere(
            SmPoint3d(0,0,0),             // in : sphere center
            SmVector3d(1,0,0),            // in : sphere X axis
            SmVector3d(0,1,0),            // in : sphere Y axis
            SmExtent2d(0,-90,360,90),     // in : angular domain in degrees
            1.,                          // in : radius
            FALSE                        // in : swap U and V
        );
        const SmExtent2d sDomain = pSphere->GetNaturalUVDomain();
        // Use a binary fraction for the constant-V case so trim evaluation
        // preserves an exactly zero-width gap, even without the seam guard.
        const double dV = sDomain.GetMin().y +
                          (nPole == 1 ? 0. : (nPole == 2 ? 1./8192. : 1e-4)*sDomain.YLength());
        SER(my_check_loop_uv_gap(
            sContext,                              // in : context for test objects
            pSphere,                               // in : consumed test surface
            SmPoint2d(sDomain.GetMin().x,dV),       // in : start of the first edge
            SmPoint2d(sDomain.GetMax().x,dV + (nPole == 0 ? 1e-5*sDomain.YLength() : 0.)), // in : end of the last edge
            .001,                                  // in : zone tolerance of each edge
            nPole == 1                             // in : expected UV closure result
        ));
    }
    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- Coplanar face/line intersections must retain the source range
  while the solution array grows to hold disjoint material intervals.
***********************************************************************/
static SmStatus my_test_face_line_intersect_growth()
{
  // Four holes cross the new capacity of 4; sixteen also cross the old 16.
  // Include cases without holes and just below the first failing case.
  const ULONG aHoleCounts[] = {0, 3, 4, 8, 16};
  for(ULONG lNumHoles : aHoleCounts)
    {
      SmContext sContext;
      const double dWidth = 2.0*lNumHoles + 2.0;
      SmBrep * pBrep = SmPrimitiveCreation::CreateRectangle(sContext, 1e-5, dWidth, 4.0, SmAxis2Placement());
      if(pBrep == NULL)
        { return SM_ERR; }
      SmObjDelete sCleanupBrep(pBrep);

      // Outer rectangle [0,width] x [0,4], with holes [2i+1,2i+2] x [1,3].
      for(ULONG ii=0; ii<lNumHoles; ++ii)
        {
          SER(sm_cut_rectangle_from_sheet(sContext, pBrep, 2.0*ii+1.0, 1.0, 1.0, 2.0));
        }

      SmTArray<SmFace*> sFaces;
      pBrep->GetFaces(sFaces);
      if(sFaces.GetSize() != 1 || !pBrep->AssertValid(NULL, SM_LEVEL_2, SM_WALK))
        { return SM_ERR; }

      SmFace * pFace = sFaces[0];
      const SmExtent1d sLineDomain(0.0, dWidth);
      const double dTol = 1e-8;
      for(int iRay=0; iRay<2; ++iRay)
        {
          // Start with a fresh array on each call so earlier calls cannot
          // hide the reallocation. Ray mode keeps only the first interval.
          SmSolutionArray sSolutions;
          SER(pFace->GlobalLineIntersect(pFace->GetUVDomain(), SmPoint3d(0,2,0),
                                          SmVector3d(1,0,0), &sLineDomain, iRay != 0,
                                          SmTol::GetZoneTol3d(pBrep), sSolutions));
          const ULONG lExpected = iRay ? 1 : lNumHoles+1;
          if(sSolutions.GetSize() != lExpected)
            { return SM_ERR; }

          for(ULONG ii=0; ii<lExpected; ++ii)
            {
              const SmSolution & rSol = sSolutions[ii];
              const double dStart = 2.0*ii;
              const double dEnd = ii == lNumHoles ? dWidth : 2.0*ii+1.0;
              if(   rSol.m_eSolutionType != SM_ST_RANGE_OF_VALUES
                 || rSol.m_lNumVariables != 3
                 || !(smos_Fabs(rSol.m_vStart[0]-dStart) <= dTol)
                 || !(smos_Fabs(rSol.m_vEnd[0]-dEnd) <= dTol))
                { return SM_ERR; }

              // Check the surface parameters as well as the line intervals.
              SmPoint3d sStart, sEnd;
              SER(pFace->GetSurface()->EvaluatePoint(SmPoint2d(rSol.m_vStart[1],rSol.m_vStart[2]), sStart));
              SER(pFace->GetSurface()->EvaluatePoint(SmPoint2d(rSol.m_vEnd[1],rSol.m_vEnd[2]), sEnd));
              if(   !(sStart.DistanceBetween(SmPoint3d(dStart,2,0)) <= dTol)
                 || !(sEnd.DistanceBetween(SmPoint3d(dEnd,2,0)) <= dTol))
                { return SM_ERR; }
            }
        }
    }
  MYPRINTF(_T("\nOKAY: face/line intersections survive solution-array growth"));
  return SM_SUCCESS;
}

// A circular loop on a cylinder without its seam exercises the legacy
// single-periodic endpoint choice used during healing. Test both UV directions.
static SmStatus my_test_single_periodic_vertexuse
 (SmBoolean         bSwapUV,    // in : TRUE = cylinder closed in V, FALSE = closed in U
  const SmExtent1d &crInterval) // in : curve and trim parameter interval
{
  MYPRINTF(_T("\n********** Entered: my_test_single_periodic_vertexuse"));
  SmContext         sContext;
  SmBrepConstructor sBuilder;
  SmBrep           *pBrep = sBuilder.StartBrep(sContext);
  SmObjDelete       sCleanup(pBrep);
  SmShell          *pOuter = NULL,*pInner = NULL;
  SER(sBuilder.StartShell(pOuter,pInner));
  SmFace     *pFace = sBuilder.StartFace(SM_OT_SAME);
  SmCylinder *pCylinder = new(sContext) SmCylinder
    (SmPoint3d(0,0,0),  // in : cylinder origin
     SmVector3d(1,0,0), // in : cylinder X axis
     SmVector3d(0,1,0), // in : cylinder Y axis
     1.,               // in : radius
     0.,               // in : start angle in degrees
     360.,             // in : end angle in degrees
     1.,               // in : height
     bSwapUV,          // in : TRUE = rotation in V, FALSE = rotation in U
     FALSE,            // in : retain generator direction
     FALSE,            // in : use an analytic generator
     &sContext);       // in : context for the surface
  SmExtent2d sDomain = pCylinder->GetNaturalUVDomain();
  SER(sBuilder.SetFaceSurface(pCylinder,sDomain));
  NER(sBuilder.StartLoop(SM_OT_SAME));
  SmVertex *pVertex = sBuilder.StartVertexOfLoop(SmPoint3d(1,0,.5));
  NER(sBuilder.StartEdge(SM_OT_SAME,NULL,pVertex,pVertex));

  SmAxis2Placement sAxes;
  SmCircle        *pCircle = NULL;
  sAxes.Translate(SmVector3d(0,0,.5));
  SER(SmCircle::CreateCanonical(sContext,sAxes,1.,pCircle));
  SER(pCircle->EditParameterization(crInterval));
  SER(sBuilder.SetEdgeCurve(pCircle,FALSE));
  SER(sBuilder.SetLimits(SmVertexNode(pVertex->GetPoint()),SmVertexNode(pVertex->GetPoint())));
  SER(sBuilder.EndEdge());
  SER(sBuilder.EndLoop());
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(TRUE));
  SER(sBuilder.EndBrep());

  SmTArray<SmLoop*>    sLoops;
  SmTArray<SmEdgeuse*> sEdgeuses;
  pFace->GetLoops(sLoops);
  if(sLoops.GetSize() != 1) { return SM_ERR; }
  sLoops[0]->GetEdgeuses(sEdgeuses);
  if(sEdgeuses.GetSize() != 1) { return SM_ERR; }

  // Supply the derived trim explicitly: this intermediate face intentionally
  // lacks its seam and is not ready for full face validation or trim generation.
  SmPoint3d sStart(bSwapUV ? (sDomain.GetUMin() + sDomain.GetUMax()) / 2. : sDomain.GetUMin(),
                  bSwapUV ? sDomain.GetVMin() : (sDomain.GetVMin() + sDomain.GetVMax()) / 2.,0.);
  SmPoint3d sEnd = sStart;
  if(bSwapUV) { sEnd.y = sDomain.GetVMax(); }
  else        { sEnd.x = sDomain.GetUMax(); }
  SmBSplineCurve *pTrim = NULL;
  SER(SmBSplineCurve::CreateLineSegment(sContext,2,sStart,sEnd,pTrim));
  SER(pTrim->EditParameterization(crInterval));
  sEdgeuses[0]->SetUVTrimCurve(pTrim,0.);
  for(ULONG lSide = 0; lSide < 2; ++lSide)
    {
      SmEdgeuse *pEdgeuse = lSide ? sEdgeuses[0]->GetMate() : sEdgeuses[0];
      SmPoint3d  sExpected = pEdgeuse->GetOrientation() == SM_OT_SAME ? sStart : sEnd;
      SmPoint2d  sUV;
      SER(pEdgeuse->GetVertexuse()->ComputeUVPoint(sUV,FALSE));
      if(sUV.DistanceBetween(SmPoint2d(sExpected.x,sExpected.y)) > SM_EFF_ZERO_PARAM) { return SM_ERR; }
    }
  return SM_SUCCESS;
}

static SmStatus my_test_doubly_periodic_vertexuse_corners
 (const SmExtent1d &crMajorInterval, // in : parameter interval of the major-circle seam
  const SmExtent1d &crMinorInterval) // in : parameter interval of the minor-circle seam
{
  MYPRINTF(_T("\n********** Entered: my_test_doubly_periodic_vertexuse_corners"));
  SmContext         sContext;
  SmBrepConstructor sBuilder;
  SmBrep           *pBrep = sBuilder.StartBrep(sContext);
  SmObjDelete       sCleanup(pBrep);
  SmShell          *pOuter = NULL,*pInner = NULL;
  SmTorus          *pTorus = NULL;
  pBrep->SetTolerance(1e-7,FALSE);
  SER(sBuilder.StartShell(pOuter,pInner));
  SmFace *pFace = sBuilder.StartFace(SM_OT_SAME);
  SER(SmTorus::CreateCanonical
    (sContext,           // in : context for the surface
     SmAxis2Placement(), // in : canonical torus placement
     3.0,                // in : major radius
     1.0,                // in : minor radius
     pTorus));           // out: torus surface
  SER(sBuilder.SetFaceSurface(pTorus,pTorus->GetNaturalUVDomain()));
  NER(sBuilder.StartLoop(SM_OT_SAME));

  SmVertex        *pVertex = sBuilder.StartVertexOfLoop(SmPoint3d(4,0,0));
  SmEdge          *pEdges[2] = {NULL,NULL};
  SmAxis2Placement sMinorAxes;
  sMinorAxes.SetCanonical(SmPoint3d(3,0,0),SmVector3d(1,0,0),SmVector3d(0,0,1));
  for(ULONG lUse = 0; lUse < 4; ++lUse)
    {
      ULONG lEdge = lUse % 2;
      pEdges[lEdge] = sBuilder.StartEdge
        (lUse < 2 ? SM_OT_SAME : SM_OT_OPPOSITE, // in : edgeuse orientation around the natural boundary
         pEdges[lEdge],                        // in : existing seam edge, or NULL to create it
         pVertex,                              // in : start vertex
         pVertex);                             // in : end vertex
      NER(pEdges[lEdge]);
      if(lUse < 2)
        {
          SmCircle *pCircle = NULL;
          SER(SmCircle::CreateCanonical
            (sContext,                                   // in : context for the curve
             lEdge ? sMinorAxes : SmAxis2Placement(),     // in : minor or major circle placement
             lEdge ? 1.0 : 4.0,                           // in : seam circle radius
             pCircle));                                  // out: seam circle
          SER(pCircle->EditParameterization(lEdge ? crMinorInterval : crMajorInterval));
          SER(sBuilder.SetEdgeCurve(pCircle,FALSE));
          SER(sBuilder.SetLimits(SmVertexNode(pVertex->GetPoint()),SmVertexNode(pVertex->GetPoint())));
        }
      SER(sBuilder.EndEdge());
    }
  SER(sBuilder.EndLoop());
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(TRUE));
  SER(sBuilder.EndBrep());

  double dMeanCurve,dMaxCurve,dMeanVertex,dMaxVertex,dMeanUV,dMaxUV;
  SER(pFace->CreateUVTrimCurves
    (FALSE,       // in : preserve edge and vertex tolerances
     NULL,        // in : generate UV trims
     NULL,        // in : no supplied trim orientations
     dMeanCurve,  // out: mean curve-to-surface gap
     dMaxCurve,   // out: maximum curve-to-surface gap
     dMeanVertex, // out: mean vertex-to-trim gap
     dMaxVertex,  // out: maximum vertex-to-trim gap
     dMeanUV,     // out: mean UV endpoint gap
     dMaxUV));    // out: maximum UV endpoint gap
  SmAssertArray sReports;
  if(   pBrep->ValidatePointers(&sReports) != SM_SUCCESS
     || !pBrep->AssertValid(&sReports,SM_LEVEL_2,SM_WALK)
     || sReports.GetSize()) { return SM_ERR; }

  // Record the valid loop's UV endpoints before moving any derived trims.
  // 3D proximity cannot distinguish the four equivalent periodic corners.
  SmTArray<SmLoop*>    sLoops;
  SmTArray<SmEdgeuse*> sEdgeuses;
  SmPoint2d           sExpectedCorners[4][2];
  SmVector3d          sSideShifts[4];
  SmExtent2d          sDomain = pTorus->GetNaturalUVDomain();
  pFace->GetLoops(sLoops);
  if(sLoops.GetSize() != 1) { return SM_ERR; }
  sLoops[0]->GetEdgeuses(sEdgeuses);
  if(sEdgeuses.GetSize() != 4) { return SM_ERR; }
  for(ULONG lUse = 0; lUse < 4; ++lUse)
    {
      SmPoint3d sStart,sEnd;
      SER(sEdgeuses[lUse]->NormalizedEvaluate(0.,TRUE,sStart,NULL,FALSE));
      SER(sEdgeuses[lUse]->NormalizedEvaluate(1.,TRUE,sEnd,NULL,FALSE));
      sExpectedCorners[lUse][0].Set(sStart.x,sStart.y);
      sExpectedCorners[lUse][1].Set(sEnd.x,sEnd.y); // The mate starts at this edgeuse's end.
      if(smos_Fabs(sEnd.x - sStart.x) < SM_EFF_ZERO_PARAM)
        {
          sSideShifts[lUse].Set
            (SM_ARE_SAME_TO_TOL(sStart.x,sDomain.GetUMin(),SM_EFF_ZERO_PARAM) ? sDomain.XLength() : -sDomain.XLength(),
             0.,0.);
        }
      else
        {
          sSideShifts[lUse].Set
            (0.,
             SM_ARE_SAME_TO_TOL(sStart.y,sDomain.GetVMin(),SM_EFF_ZERO_PARAM) ? sDomain.YLength() : -sDomain.YLength(),
             0.);
        }
    }

  // Independently choose either periodic side for each of the four trims.
  // Check all four vertexuses and their mates for each of the 16 combinations.
  ULONG lPreviousMask = 0;
  for(ULONG lMask = 0; lMask < 16; ++lMask)
    {
      for(ULONG lUse = 0; lUse < 4; ++lUse)
        {
          if((lMask ^ lPreviousMask) & (1u << lUse))
            {
              SmAxis2Placement sShift;
              sShift.Translate((lMask & (1u << lUse)) ? sSideShifts[lUse] : -sSideShifts[lUse]);
              SER(sEdgeuses[lUse]->GetUVTrimCurve()->Transform(sShift));
            }
        }
      lPreviousMask = lMask;
      for(ULONG lUse = 0; lUse < 4; ++lUse)
        {
          for(ULONG lSide = 0; lSide < 2; ++lSide)
            {
              SmEdgeuse *pEdgeuse = lSide ? sEdgeuses[lUse]->GetMate() : sEdgeuses[lUse];
              SmPoint2d  sCorner;
              SER(pEdgeuse->GetVertexuse()->ComputeUVPoint(sCorner,FALSE));
              if(sCorner.DistanceBetween(sExpectedCorners[lUse][lSide]) > SM_EFF_ZERO_PARAM) { return SM_ERR; }
            }
        }
    }
  return SM_SUCCESS;
}

// The same directed hole must have the same containment for every first edge.
// Build the complete seam topology explicitly; trims and stored loop roles are
// not inputs to the classification. Reversed winding must remain observable.
static SmStatus my_test_seam_touch_incidence()
{
  MYPRINTF(_T("\n********** Entered: my_test_seam_touch_incidence"));
  for (ULONG lKind = 0; lKind < 7; ++lKind)
  {
    // Full cylinder/torus, either one-sided cylinder boundary (optionally
    // segmented), and a torus patch meeting a corner of the UV domain.
    SmBoolean bTorus    = lKind == 1 || lKind == 6;
    SmBoolean bHighSide = lKind == 4 || lKind == 5;
    for (ULONG lRotation = 0; lRotation < 4; ++lRotation)
    {
      for (ULONG lReversed = 0; lReversed < 4; ++lReversed)
      {
        // Test reversed outer winding at each one-sided seam. The other cases
        // exercise complete containment with correct/reversed hole winding.
        if(lReversed >= 2 && ((lKind != 2 && lKind != 4) || lRotation != 0)) { continue; }
        SmContext         sContext;
        SmBrepConstructor sBuilder;
        SmBrep           *pBrep = sBuilder.StartBrep(sContext);
        SmObjDelete       sCleanup(pBrep);
        pBrep->SetTolerance(1e-7, FALSE);
        SmShell *pOuter = NULL, *pInner = NULL;
        SER(sBuilder.StartShell(pOuter, pInner));
        SmFace    *pFace = sBuilder.StartFace(SM_OT_SAME);
        SmSurface *pSurface = NULL;
        if (bTorus)
        {
          SmTorus *pTorus = NULL;
          SER(SmTorus::CreateCanonical(sContext, SmAxis2Placement(), 3., 1., pTorus));
          pSurface = pTorus;
        }
        else
        {
          SmCylinder *pCylinder = NULL;
          SER(SmCylinder::CreateCanonical(sContext, SmAxis2Placement(), .5, pCylinder));
          SmExtent2d sDomain = pCylinder->GetSTEPUVDomain();
          SER(pCylinder->AdjustSTEPUVDomain(SmExtent2d(sDomain.GetMin().x, 0., sDomain.GetMax().x, 3.)));
          pSurface = pCylinder;
        }
        SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
        SER(sBuilder.SetFaceSurface(pSurface, sDomain));
        SmLoop *apLoops[2];
        for (ULONG lLoop = 0; lLoop < 2; ++lLoop)
        {
          // Deliberately put the hole first, with an untrusted stored role.
          apLoops[lLoop] = sBuilder.StartLoop(SM_OT_SAME);
          SmPoint2d asUV[6] = { SmPoint2d(.3, .25), SmPoint2d(.3, .75), SmPoint2d(.45, .75), SmPoint2d(.45, .25) };
          if(bHighSide)
          {
            for(ULONG lIndex=0; lIndex<4; ++lIndex) { asUV[lIndex].x += .3; }
          }
          if(lKind == 6)
          {
            for(ULONG lIndex=0; lIndex<4; ++lIndex) { asUV[lIndex].y *= .8; }
          }
          if (lLoop)
          {
            double dLow  = bTorus ? 0. : .05;
            double dHigh = lKind == 6 ? .7 : lKind == 1 ? 1. : .95;
            double dLeft = bHighSide ? .3 : 0., dRight = lKind >= 2 && !bHighSide ? .7 : 1.;
            asUV[0] = SmPoint2d(dLeft, dLow);
            asUV[1] = SmPoint2d(dRight, dLow);
            asUV[2] = SmPoint2d(dRight, dHigh);
            asUV[3] = SmPoint2d(dLeft, dHigh);
          }
          ULONG lSegments = lLoop && (lKind == 3 || lKind == 5) ? 6UL : 4UL;
          if(lSegments == 6)
          {
            // Split both vertical boundaries at the sample height, so both
            // periodic images encounter a vertex between two seam segments.
            asUV[5] = SmPoint2d(asUV[0].x, .5);
            asUV[4] = asUV[3];
            asUV[3] = asUV[2];
            asUV[2] = SmPoint2d(asUV[1].x, .5);
          }
          SmVertex *apVertices[6];
          SmEdge   *apEdges[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
          for (ULONG lIndex = 0; lIndex < lSegments; ++lIndex)
          {
            asUV[lIndex] = sDomain.Evaluate(asUV[lIndex].x, asUV[lIndex].y);
            SmPoint3d sPoint;
            SER(pSurface->EvaluatePoint(asUV[lIndex], sPoint));
            apVertices[lIndex] = NULL;
            for (ULONG lPrevious = 0; lPrevious < lIndex; ++lPrevious)
            {
              if ((sPoint - apVertices[lPrevious]->GetPoint()).Length() < 1e-8)
              {
                apVertices[lIndex] = apVertices[lPrevious];
                break;
              }
            }
            if (!apVertices[lIndex])
            {
              apVertices[lIndex] = sBuilder.StartVertexOfLoop(sPoint);
            }
          }
          SmBoolean bReverse = (lReversed & (lLoop ? 2UL : 1UL)) != 0;
          for (ULONG lIndex = 0; lIndex < lSegments; ++lIndex)
          {
            ULONG   lFrom = (lRotation + (bReverse ? lSegments - lIndex : lIndex)) % lSegments;
            ULONG   lTo = (lFrom + (bReverse ? lSegments - 1 : 1)) % lSegments;
            ULONG   lSegment = bReverse ? lTo : lFrom;
            ULONG   lPartner = (lSegment + 2) % 4;
            SmEdge *pShared = lLoop && (lKind == 1 || (lKind == 0 && lSegment % 2)) ? apEdges[lPartner] : NULL;
            SmEdge *pEdge = sBuilder.StartEdge(pShared ? SM_OT_OPPOSITE : SM_OT_SAME, pShared, apVertices[lFrom], apVertices[lTo]);
            NER(pEdge);
            apEdges[lSegment] = pEdge;
            if (!pShared)
            {
              SmLine   sLine(asUV[lFrom], asUV[lTo], 2, &sContext);
              SmCurve *pCurve = new (sContext) SmCrvOnSurf
                (sLine,      // in : parameter-space boundary
                 *pSurface,  // in : supporting surface
                 NULL,       // in : natural surface domain
                 2,          // in : own copies of the curve and surface
                 &sContext); // in : allocation context
              SER(sBuilder.SetEdgeCurve(pCurve, FALSE));
              SER(sBuilder.SetLimits(SmVertexNode(0.), SmVertexNode(1.)));
            }
            SER(sBuilder.EndEdge());
          }
          SER(sBuilder.EndLoop());
        }
        SER(sBuilder.EndFace());
        SER(sBuilder.EndShell(FALSE));
        SER(sBuilder.EndBrep());
        for (ULONG lLoop = 0; lLoop < 2; ++lLoop)
        {
          SmTArray<SmEdgeuse*> sUses;
          apLoops[lLoop]->GetEdgeuses(sUses);
          for (ULONG lIndex = 0; lIndex < sUses.GetSize(); ++lIndex)
          {
            if (sUses[lIndex]->GetUVTrimCurve())
            {
              return SM_ERR;
            }
          }
        }
        SmFaceProps sProps(&sContext);
        SER(sProps.SetProps_Stage2(pFace, 0, SM_HO_ALL));
        // Full-period boundaries have two UV seam encounters and no periodic
        // crossing. The partial cylinder instead has a real trim crossing at
        // the periodic join and one inside the interval.
        SmLine                sSampleUV(sDomain.Evaluate(0., .5), sDomain.Evaluate(1., .5), 2, &sContext);
        SmCrvOnSurf           sSample(sSampleUV, *pSurface, &sDomain, 0, &sContext);
        SmExtent1d            sInterval = sSample.GetNaturalInterval();
        SmCurveClassification sClassification;
        sClassification.ReSet(&sSample, &sInterval, &sSampleUV, sProps.m_sZoneTol3d);
        SER(pFace->CurveOnClassify
          (TRUE,            // in : classify intervals with no boundary intersections
           sClassification, // i/o: sample and its boundary encounters
           TRUE,            // in : accommodate edge-face gaps
           TRUE,            // in : verify intersections in 3D
           NULL,            // out: curve modification flag unused
           FALSE));         // in : do not rely on UV trims
        SmTArray<SmLoop*> sTargets;
        sTargets.Add(apLoops[0]);
        sTargets.Add(apLoops[1]);
        SmTArray<ULONG>       sCounts;
        SmTArray<SmTouchData> sRaw, sDone, sEncounters;
        SER(sClassification.GetLoopTouches(sTargets,&sProps,sEncounters));
        if(sEncounters.GetSize() != (lKind >= 2 ? 10UL : 8UL) || sEncounters[0].m_eLoopDir != (bHighSide ? SM_CD_SEAM_NO_EDGEUSE : SM_CD_SEAM)
         || sEncounters[sEncounters.GetSize()-1].m_eLoopDir != ((lKind >= 2 && !bHighSide) ? SM_CD_SEAM_NO_EDGEUSE : SM_CD_SEAM)
         || sEncounters[0].m_lLoopIndx != 1 || sEncounters[sEncounters.GetSize()-1].m_lLoopIndx != 1
         || sEncounters[0].m_dParam >= sEncounters[sEncounters.GetSize()-1].m_dParam)
         { return SM_ERR; }
        SER(sClassification.GetLoopCrossings
         (sTargets, // in : hole and enclosing boundary
          &sProps,  // i/o: Stage 2 face properties
          &sCounts, // out: periodic crossing counts
          &sRaw,    // out: complete encounters
          &sDone)); // out: derived periodic crossings
        if(sCounts[0] != 2 || sCounts[1] != (lKind >= 2 ? 2UL : 0UL) || sRaw.GetSize() != sEncounters.GetSize())
         { return SM_ERR; }
        for(ULONG lIndex=0; lIndex<sRaw.GetSize(); ++lIndex)
         { if(!(sRaw[lIndex] == sEncounters[lIndex])) { return SM_ERR; } }
        if((lKind == 2 || lKind == 4) && lRotation == 0 && lReversed == 0)
        {
          // The same circle without a defining UV curve must select the same
          // endpoint images, even when a point drop chooses the other image.
          SmCircle sCircle
            (SmPoint3d(0.,0.,1.5), // in : center at half the cylinder height
             SmVector3d(1.,0.,0.), // in : zero-angle direction
             SmVector3d(0.,1.,0.), // in : positive angular direction
             SmExtent1d(0.,360.),  // in : one complete revolution
             .5,                  // in : cylinder radius
             3,                   // in : model-space curve
             &sContext);          // in : allocation context
          SmExtent1d            sCircleInterval = sCircle.GetNaturalInterval();
          SmCurveClassification sCircleClassification;
          SmTArray<SmTouchData>  sCircleTouches;
          sCircleClassification.ReSet(&sCircle,&sCircleInterval,NULL,sProps.m_sZoneTol3d);
          SER(pFace->CurveOnClassify
            (TRUE,                  // in : classify intervals with no boundary intersections
             sCircleClassification, // i/o: model-space sample and its boundary encounters
             TRUE,                  // in : accommodate edge-face gaps
             TRUE,                  // in : verify intersections in 3D
             NULL,                  // out: curve modification flag unused
             FALSE));               // in : do not rely on UV trims
          SER(sCircleClassification.GetLoopTouches(sTargets,&sProps,sCircleTouches));
          if(sCircleTouches.GetSize() != sEncounters.GetSize()
           || sCircleTouches[0].m_eLoopDir != sEncounters[0].m_eLoopDir
           || sCircleTouches[sCircleTouches.GetSize()-1].m_eLoopDir != sEncounters[sEncounters.GetSize()-1].m_eLoopDir)
            { return SM_ERR; }
        }
        if(lKind == 6 && lRotation == 0 && lReversed == 0)
        {
          // The patch uses (Umin,Vmin). This diagonal starts at (Umin,Vmax)
          // and ends at (Umax,Vmin): neither image has a bounding edgeuse.
          SmLine                sCornerUV(sDomain.Evaluate(0.,1.),sDomain.Evaluate(1.,0.),2,&sContext);
          SmCrvOnSurf           sCorner(sCornerUV,*pSurface,&sDomain,0,&sContext);
          SmExtent1d            sCornerInterval = sCorner.GetNaturalInterval();
          SmCurveClassification sCornerClassification;
          SmTArray<SmTouchData>  sCornerTouches;
          sCornerClassification.ReSet(&sCorner,&sCornerInterval,&sCornerUV,sProps.m_sZoneTol3d);
          SER(pFace->CurveOnClassify
            (TRUE,                  // in : classify intervals with no boundary intersections
             sCornerClassification, // i/o: diagonal sample and its boundary encounters
             TRUE,                  // in : accommodate edge-face gaps
             TRUE,                  // in : verify intersections in 3D
             NULL,                  // out: curve modification flag unused
             FALSE));               // in : do not rely on UV trims
          SER(sCornerClassification.GetLoopTouches(sTargets,&sProps,sCornerTouches));
          if(sCornerTouches.GetSize() < 4 || sCornerTouches[0].m_eLoopDir != SM_CD_SEAM_NO_EDGEUSE
           || sCornerTouches[sCornerTouches.GetSize()-1].m_eLoopDir != SM_CD_SEAM_NO_EDGEUSE)
            { return SM_ERR; }
        }
        if (lKind == 1 && lRotation == 0 && lReversed == 0)
        {
          // A sample coincident with the seam cannot anchor nesting.
          SmLine                sBoundaryUV(sDomain.Evaluate(0., 0.), sDomain.Evaluate(1., 0.), 2, &sContext);
          SmCrvOnSurf           sBoundary(sBoundaryUV, *pSurface, &sDomain, 0, &sContext);
          SmExtent1d            sBoundaryInterval = sBoundary.GetNaturalInterval();
          SmCurveClassification sAmbiguous;
          sAmbiguous.ReSet(&sBoundary, &sBoundaryInterval, &sBoundaryUV, sProps.m_sZoneTol3d);
          SER(pFace->CurveOnClassify
            (TRUE,       // in : classify intervals with no boundary intersections
             sAmbiguous, // i/o: sample and its boundary encounters
             TRUE,       // in : accommodate edge-face gaps
             TRUE,       // in : verify intersections in 3D
             NULL,       // out: curve modification flag unused
             FALSE));    // in : do not rely on UV trims
          // Coincidence is retained as evidence, not invented transverse crossings.
          SER(sAmbiguous.GetLoopTouches(sTargets,&sProps,sEncounters));
          if(sEncounters.GetSize() < 4 || sEncounters[0].m_eLoopDir != SM_CD_SEAM
           || sEncounters[sEncounters.GetSize()-1].m_eLoopDir != SM_CD_SEAM
           || (sEncounters[1].m_eLoopDir != SM_CD_ON
             && sEncounters[sEncounters.GetSize()-2].m_eLoopDir != SM_CD_ON))
           { return SM_ERR; }
          SER(sAmbiguous.GetLoopCrossings
           (sTargets, // in : hole and enclosing boundary
            &sProps,  // i/o: Stage 2 face properties
            &sCounts, // out: periodic crossing counts
            &sRaw,    // out: complete encounters
            &sDone)); // out: derived periodic crossings
          if(sDone.GetSize() != 0 || sRaw.GetSize() != sEncounters.GetSize()) { return SM_ERR; }
          for(ULONG lIndex=0; lIndex<sRaw.GetSize(); ++lIndex)
           { if(!(sRaw[lIndex] == sEncounters[lIndex])) { return SM_ERR; } }
        }
        if(lReversed & 2UL)
        {
          // Incidence must remain visible for a reversed outer boundary too.
          // Stage 3's vertex-sector interpretation of that invalid loop is
          // outside this test; the raw seam-side and periodic-count checks above
          // must succeed before winding is repaired.
          continue;
        }
        SmTArray<SmTouchData> sSavedU, sSavedV;
        sSavedU.Append(sProps.m_sRawTouchListU);
        sSavedV.Append(sProps.m_sRawTouchListV);
        SER(sProps.SetProps_Stage3(pFace, 0, SM_HO_ALL));
        if(sSavedU.GetSize() != sProps.m_sRawTouchListU.GetSize()
         || sSavedV.GetSize() != sProps.m_sRawTouchListV.GetSize()) { return SM_ERR; }
        for(ULONG lIndex=0; lIndex<sSavedU.GetSize(); ++lIndex)
         { if(!(sSavedU[lIndex] == sProps.m_sRawTouchListU[lIndex])) { return SM_ERR; } }
        for(ULONG lIndex=0; lIndex<sSavedV.GetSize(); ++lIndex)
         { if(!(sSavedV[lIndex] == sProps.m_sRawTouchListV[lIndex])) { return SM_ERR; } }
        if (sProps.m_sLoopProps.GetSize() != 2 || sProps.m_eNotYetFaceType != SM_HNF_NO_PROBS || sProps.m_bBadMissingSeam ||
          sProps.m_bBadCrossedSeam)
        {
          TCHAR sMessage[SM_TBLOCK_SIZE];
          smos_sprintf(sMessage,_T("\nUV containment setup failed: kind %lu rotation %lu reversed %lu"),lKind,lRotation,lReversed);
          MYPRINTF(sMessage);
          return SM_ERR;
        }
        for (ULONG lIndex = 0; lIndex < sProps.m_sLoopProps.GetSize(); ++lIndex)
        {
          const SmLoopProps &crLoopProps = sProps.m_sLoopProps[lIndex];
          SmBoolean         bHole = crLoopProps.m_cpLoop == apLoops[0];
          SmBoolean         bReverse = (lReversed & (bHole ? 1UL : 2UL)) != 0;
          SmOrientType      eExpected = (bHole != bReverse) ? SM_OT_OPPOSITE : SM_OT_SAME;
          if (crLoopProps.m_eContainmentType != (bHole ? SM_CMT_INNERLOOP : SM_CMT_OUTERLOOP) || crLoopProps.m_eLoopOrient != eExpected ||
            crLoopProps.m_bGoodOrient != !bReverse)
          {
            TCHAR sMessage[SM_TBLOCK_SIZE];
            smos_sprintf
              (sMessage,                       // out: diagnostic text
               _T("\nUV containment failed: kind %lu rotation %lu reversed %lu loop %lu role %d orient %d good %d"),
               lKind,                          // in : cylinder or torus
               lRotation,                      // in : first boundary edge
               lReversed,                      // in : reversed hole/outer winding bits
               lIndex,                         // in : reported loop
               crLoopProps.m_eContainmentType, // in : measured loop role
               crLoopProps.m_eLoopOrient,      // in : measured winding
               crLoopProps.m_bGoodOrient);     // in : winding agrees with role
            MYPRINTF(sMessage);
            return SM_ERR;
          }
        }
      }
    }
  }
  MYPRINTF(_T("\nUV containment at seams: 60 cases passed (56 containment, 4 reversed-outer incidence)"));
  return SM_SUCCESS;
}

// A bounded planar walk establishes containment independently of winding.
// Keep the loop inside the surface domain so its crossings are not endpoints.
static SmStatus my_test_planar_outer_loop_winding()
{
  MYPRINTF(_T("\n********** Entered: my_test_planar_outer_loop_winding"));
  for(ULONG lReversed=0; lReversed<2; ++lReversed)
    {
      SmContext         sContext;
      SmBrepConstructor sBuilder;
      SmBrep           *pBrep = sBuilder.StartBrep(sContext);
      NER(pBrep);
      SmObjDelete       sCleanup(pBrep);
      SmShell          *pOuter = NULL, *pInner = NULL;
      pBrep->SetTolerance(1e-7,FALSE);
      SER(sBuilder.StartShell(pOuter,pInner));
      SmFace  *pFace  = sBuilder.StartFace(SM_OT_SAME);
      SmPlane *pPlane = new(sContext) SmPlane
        (SmPoint3d(0,0,0),      // in : origin
         SmVector3d(1,0,0),     // in : U direction
         SmVector3d(0,1,0),     // in : V direction
         SmVector2d(1,1),       // in : parameter scale
         SmExtent2d(0,0,10,10), // in : surface domain enclosing the loop
         &sContext);           // in : allocation context
      SER(sBuilder.SetFaceSurface(pPlane,pPlane->GetNaturalUVDomain()));
      NER(sBuilder.StartLoop(SM_OT_SAME));
      SmPoint3d asPoints[4] = {SmPoint3d(2,2,0),SmPoint3d(8,2,0),SmPoint3d(8,8,0),SmPoint3d(2,8,0)};
      SmVertex *apVertices[4];
      for(ULONG lIndex=0; lIndex<4; ++lIndex)
        { apVertices[lIndex] = sBuilder.StartVertexOfLoop(asPoints[lIndex]); }
      for(ULONG lIndex=0; lIndex<4; ++lIndex)
        {
          ULONG lFrom = lReversed ? (4-lIndex)%4 : lIndex;
          ULONG lTo   = lReversed ? (3-lIndex)%4 : (lIndex+1)%4;
          NER(sBuilder.StartEdge(SM_OT_SAME,NULL,apVertices[lFrom],apVertices[lTo]));
          SER(sBuilder.SetEdgeCurve(new(sContext) SmLine(asPoints[lFrom],asPoints[lTo],3,&sContext),FALSE));
          SER(sBuilder.SetLimits(SmVertexNode(0.),SmVertexNode(1.)));
          SER(sBuilder.EndEdge());
        }
      SER(sBuilder.EndLoop());
      SER(sBuilder.EndFace());
      SER(sBuilder.EndShell(FALSE));
      SER(sBuilder.EndBrep());

      SmFaceProps sProps(&sContext);
      SER(sProps.SetProps_Stage3(pFace,0,SM_HO_ALL));
      if(   sProps.m_eNotYetFaceType != SM_HNF_NO_PROBS
         || sProps.m_pLooptreeRoot == NULL
         || sProps.m_sLoopProps.GetSize() != 1)
        { return SM_ERR; }
      const SmLoopProps &crLoopProps = sProps.m_sLoopProps[0];
      if(   crLoopProps.m_eContainmentType != SM_CMT_OUTERLOOP
         || crLoopProps.m_eLoopOrient != (lReversed ? SM_OT_OPPOSITE : SM_OT_SAME)
         || crLoopProps.m_bGoodOrient != (lReversed == 0))
        { return SM_ERR; }
    }
  MYPRINTF(_T("\nPlanar outer loop winding: two cases passed"));
  return SM_SUCCESS;
}

// Overlapping cylinder loops produce interleaved entries and exits rather than
// nesting. Stage 3 must reject that walk and discard its partially built tree.
static SmStatus my_test_failed_loop_containment()
{
  MYPRINTF(_T("\n********** Entered: my_test_failed_loop_containment"));
  SmContext         sContext;
  SmBrepConstructor sBuilder;
  SmBrep           *pBrep = sBuilder.StartBrep(sContext);
  NER(pBrep);
  SmObjDelete       sCleanup(pBrep);
  SmShell          *pOuter = NULL, *pInner = NULL;
  pBrep->SetTolerance(1e-7,FALSE);
  SER(sBuilder.StartShell(pOuter,pInner));
  SmFace     *pFace     = sBuilder.StartFace(SM_OT_SAME);
  SmCylinder *pSurface  = NULL;
  SER(SmCylinder::CreateCanonical(sContext,SmAxis2Placement(),.5,pSurface));
  SmExtent2d sSTEPDomain = pSurface->GetSTEPUVDomain();
  SER(pSurface->AdjustSTEPUVDomain(SmExtent2d(sSTEPDomain.GetUMin(),0.,sSTEPDomain.GetUMax(),3.)));
  SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
  SER(sBuilder.SetFaceSurface(pSurface,sDomain));
  for(ULONG lLoop=0; lLoop<2; ++lLoop)
    {
      NER(sBuilder.StartLoop(SM_OT_SAME));
      SmPoint2d asUV[4] = {SmPoint2d(.6,.25),SmPoint2d(.6,.75),SmPoint2d(.85,.75),SmPoint2d(.85,.25)};
      if(lLoop == 0)
        {
          asUV[0].Set(0.,.05); asUV[1].Set(.7,.05);
          asUV[2].Set(.7,.95); asUV[3].Set(0.,.95);
        }
      SmVertex *apVertices[4];
      for(ULONG lIndex=0; lIndex<4; ++lIndex)
        {
          asUV[lIndex] = sDomain.Evaluate(asUV[lIndex].x,asUV[lIndex].y);
          SmPoint3d sPoint;
          SER(pSurface->EvaluatePoint(asUV[lIndex],sPoint));
          apVertices[lIndex] = sBuilder.StartVertexOfLoop(sPoint);
        }
      for(ULONG lIndex=0; lIndex<4; ++lIndex)
        {
          ULONG lNext = (lIndex+1)%4;
          NER(sBuilder.StartEdge(SM_OT_SAME,NULL,apVertices[lIndex],apVertices[lNext]));
          SmLine   sLine(asUV[lIndex],asUV[lNext],2,&sContext);
          SmCurve *pCurve = new(sContext) SmCrvOnSurf
            (sLine,      // in : parameter-space boundary
             *pSurface,  // in : supporting surface
             NULL,       // in : natural surface domain
             2,          // in : own copies of the curve and surface
             &sContext); // in : allocation context
          SER(sBuilder.SetEdgeCurve(pCurve,FALSE));
          SER(sBuilder.SetLimits(SmVertexNode(0.),SmVertexNode(1.)));
          SER(sBuilder.EndEdge());
        }
      SER(sBuilder.EndLoop());
    }
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(FALSE));
  SER(sBuilder.EndBrep());

  // The invalid nesting also triggers two debug consistency assertions.
  MYPRINTF(_T("\nExpect inconsistent-loop classification diagnostics for the overlapping boundaries."));
  SmFaceProps sProps(&sContext);
  SmStatus    sStatus = sProps.SetProps_Stage3(pFace,0,SM_HO_ALL);
  if(   sStatus != SM_ERR_NOTYET_HEAL_FACE
     || sProps.m_eNotYetFaceType != SM_HNF_NO_LOOP_TREE_ROOT
     || sProps.m_pLooptreeRoot != NULL)
    { return SM_ERR; }
  MYPRINTF(_T("\nFailed loop containment: partial tree discarded"));
  return SM_SUCCESS;
}

static SmStatus my_test_loop_orientation_knot_sampling()
{
  MYPRINTF(_T("\n********** Entered: my_test_loop_orientation_knot_sampling"));
  // A simple concave CCW polygon of UV area .36. With the biased knots,
  // sampling the single edge every .05 misses the entire upper excursion
  // and gives a CW polygon instead. Also test reverse traversal and stale trims.
  for(ULONG lCase=0; lCase<13; ++lCase)
    {
      SmContext sContext;
      SmBrep   *pBrep = NULL;
      SmFace   *pFace = NULL;
      SmLoop   *pLoop = NULL;
      SER(sm_MakeLoopOrientationCase
        (sContext, // in : allocation context
         lCase,    // in : original knot-sampling case number
         pBrep,    // out: caller-owned brep
         pFace,    // out: face with the sampled boundary
         pLoop));  // out: loop whose orientation is measured
      SmObjDelete sCleanup(pBrep);
      SmTArray<SmEdgeuse*> sUses;
      pLoop->GetLoopuse()->GetEdgeuses(sUses);
      const SmBoolean bReverse = lCase == 2 || lCase == 4 || lCase == 7 || lCase == 8 || lCase >= 10;
      SmBSplineCurve *pOldTrim = sUses[0]->GetUVTrimCurve();
      SmOrientType eOrient = SM_OT_UNKNOWN;
      SER(pFace->ComputeLoopOrientation
        (pLoop,   // in : target loop
         eOrient, // out: measured orientation
         NULL,    // out: optional UV bounds
         TRUE));  // in : use defining 3D curves
      SmOrientType eExpected = lCase == 5 ? SM_OT_UNKNOWN : bReverse ? SM_OT_OPPOSITE : SM_OT_SAME;
      if(eOrient != eExpected || sUses[0]->GetUVTrimCurve() != pOldTrim) { return SM_ERR; }
    }
  MYPRINTF(_T("\nLoop orientation knot sampling: thirteen cases passed"));
  return SM_SUCCESS;
}

SmStatus my_test_faces()
{
  // Run every independent test and retain the first failure in either build mode.
  SmStatus sResult = SM_SUCCESS;

  PT_RUN(sResult, my_test_loop_orientation_periodic_winding());
  PT_RUN(sResult, my_test_loop_orientation_knot_sampling());
  // Scale each seam independently to exercise both adjacent derivative intervals.
  PT_RUN(sResult, my_test_doubly_periodic_vertexuse_corners(SmExtent1d(0.,1.),SmExtent1d(0.,1.)));
  PT_RUN(sResult, my_test_doubly_periodic_vertexuse_corners(SmExtent1d(0.,1e-4),SmExtent1d(0.,1.)));
  PT_RUN(sResult, my_test_doubly_periodic_vertexuse_corners(SmExtent1d(0.,1.),SmExtent1d(0.,1e-4)));
  PT_RUN(sResult, my_test_single_periodic_vertexuse(FALSE,SmExtent1d(0.,1.)));
  PT_RUN(sResult, my_test_single_periodic_vertexuse(TRUE,SmExtent1d(0.,1.)));
  PT_RUN(sResult, my_test_single_periodic_vertexuse(FALSE,SmExtent1d(0.,1e-4)));
  PT_RUN(sResult, my_test_single_periodic_vertexuse(TRUE,SmExtent1d(0.,1e-4)));
  PT_RUN(sResult, my_test_face_line_intersect_growth());
  PT_RUN(sResult, my_test_loop_uv_gap_neighborhood());
  PT_RUN(sResult, my_test_seam_touch_incidence());
  PT_RUN(sResult, my_test_planar_outer_loop_winding());
  PT_RUN(sResult, my_test_failed_loop_containment());
  PT_RUN(sResult, my_test_seam_uv_redistribution_at_poles());
  PT_RUN(sResult, my_test_get_loop_crossings());
  PT_RUN(sResult, my_test_face_loop_nesting());
  return sResult;
}
