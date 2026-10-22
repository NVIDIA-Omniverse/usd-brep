// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- smHealTest.cpp
* PURPOSE --- Test healing stages, property refresh, and repeated repairs.
**********************************************************************/

#include "StdAfx.h"
#include <smHealTest.h>
#include <smTestBrep.h>
#include <SmHealData.h>
#include <SmAssertArray.h>
#include <SmCrvOnSurf.h>
#include <SmFaceProps.h>
#include <SmPrimitiveCreation.h>
#include <SmTrimmingTools.h>
#include <SmTrackTopologyChanges.h>
#include <SmLine.h>

static SmStatus my_test_heal_infinite_region_with_plausible_void_flags()
{
  SmContext           sContext;
  SmBrep             *pBrep = new(sContext) SmBrep();
  SmObjDelete         sCleanBrep(pBrep);
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion());
  SER(sPrimitive.CreateBox(1.0,1.0,1.0,SmAxis2Placement()));

  SmTArray<SmRegion*> sRegions;
  pBrep->GetRegions(sRegions);
  AERN(sRegions.GetSize() == 2,SM_ERR);
  SmRegion *pExterior = sRegions[0];
  SmRegion *pInterior = sRegions[1];

  // Reproduce an inward-oriented imported shell: the wrong exterior is first
  // and marked void, while the actual exterior is marked material.
  SER(pBrep->ResetInfiniteRegion(pInterior,SM_VS_INFINITEREGION_SWAP));
  pBrep->GetRegions(sRegions);
  AERN(sRegions[0] == pInterior && pInterior->IsVoid() && !pExterior->IsVoid(),SM_ERR);

  // The first pass must correct the assignment; the next must preserve it.
  for(ULONG lPass=0; lPass<2; ++lPass)
    {
      SER(pBrep->HealBrep(SM_HO_ALL));
      pBrep->GetRegions(sRegions);
      AERN(sRegions.GetSize() == 2 && sRegions[0] == pExterior && sRegions[1] == pInterior,SM_ERR);
      AERN(pBrep->GetInfiniteRegion() == pExterior && pExterior->IsVoid() && !pInterior->IsVoid(),SM_ERR);
      AERN(pBrep->GetNumFaces() == 6 && pBrep->GetNumEdges() == 12 && pBrep->GetNumVertices() == 8,SM_ERR);

      SmAssertArray sReports;
      SER(pBrep->ValidatePointers(&sReports));
      AERN(sReports.GetSize() == 0,SM_ERR);
      AERN(pBrep->AssertValid(&sReports,SM_LEVEL_2,SM_WALK) && sReports.GetSize() == 0,SM_ERR);
    }
  return SM_SUCCESS;
}

static SmStatus my_test_move_seam_refreshes_face_props()
{
  // Exercise both the tracker consumer and the healer's own MoveSeam path.
  for(ULONG ii=0; ii<2; ii++)
    {
      SmContext sContext ;
      SmBrep * pBrep = new(sContext) SmBrep() ;
      SmObjDelete sCleanBrep(pBrep) ;
      SmBSplineSurface * pSurface = NULL ;
      SmFace * pFace = NULL ;
      SER(SmBSplineSurface::CreateConePatch(sContext, SmAxis2Placement(), 1.0, 1.0,
                                           45.0, 135.0, 1.0, SM_CO_QUADRATIC, pSurface)) ;
      SER(pBrep->CreateFaceFromSurface(pSurface, pSurface->GetNaturalUVDomain(), pFace)) ;

      // Keep the quarter-cylinder boundary, but place the full surface's seam inside it.
      SmAxis2Placement sSeamFrame ;
      sSeamFrame.SetCanonical(SmPoint3d(0,0,0), SmVector3d(0,1,0), SmVector3d(-1,0,0)) ;
      SER(SmBSplineSurface::CreateConePatch(sContext, sSeamFrame, 1.0, 1.0,
                                           0.0, 360.0, 1.0, SM_CO_QUADRATIC, pSurface)) ;
      SER(pBrep->ReplaceSurface(pFace, pSurface, FALSE, NULL, FALSE)) ;
      pFace->SetIsRectangularTrim(FALSE) ;

      SmCurveClassification sClassU, sClassV ;
      SmSurfParamType eCrossed, eMissing, eNearMiss ;
      if(   !pFace->HasSeamProblem(sClassU, sClassV, eCrossed, eMissing, eNearMiss)
         || eCrossed != SM_SP_U)
        { return SM_ERR ; }
      SmHealData sHealData ;
      SER(sHealData.HealBrep(pBrep, ii == 0 ? SM_HO_CACHE_FACEPROPS_2 : SM_HO_FIX_MOVESEAM)) ;
      if(sHealData.m_sTgtFaceProps.GetSize() != 1)
        { return SM_ERR ; }
      SmFaceProps * pProps = sHealData.m_sTgtFaceProps[0] ;

      if(ii == 0)
        {
          if(   pProps->m_cpSurface != pSurface
             || !pProps->m_bBadCrossedSeam)
            { return SM_ERR ; }
          SmFixSeamPlanType eMoveU, eMoveV ;
          SmTrackTopologyChanges sChanges(pBrep, TRUE) ;
          SER(pFace->MoveSeam(eMoveU, eMoveV, *pProps->m_pCrvClassU, *pProps->m_pCrvClassV,
                             pProps->m_eBadCrossedSeam, pProps->m_eBadMissingSeam,
                             pProps->m_eBadNearMissSeam, TRUE, FALSE, NULL, NULL, NULL, pProps)) ;
          sChanges.StopTracking() ;
          if(   (eMoveU == SM_FSP_NO_CHANGES && eMoveV == SM_FSP_NO_CHANGES)
             || !sChanges.IsInRmList(pFace)
             || !sChanges.IsInAddList(pFace))
            { return SM_ERR ; }
          SER(sHealData.UpdateTargetLists(sChanges, SM_HO_CACHE_FACEPROPS_2)) ;
        }

      if(   pFace->GetSurface() == pSurface
         || sHealData.m_sTgtFaceProps.GetSize() != 1)
        { return SM_ERR ; }
      pProps = sHealData.m_sTgtFaceProps[0] ;
      if(   ii == 1
         && (   pProps->m_eBeenThroughMoveSeam != SM_TRY_FIXED
             || sHealData.m_sRanFaceProps_MoveSeam.GetSize() != 1
             || sHealData.m_sRanFaceProps_MoveSeam[0] != pProps))
        { return SM_ERR ; }
      if(   pProps->m_pFace != pFace
         || pProps->m_cpSurface != pFace->GetSurface()
         || pProps->m_bBadCrossedSeam
         || pProps->m_bBadMissingSeam
         || pProps->m_bBadNearMissSeam
         || sHealData.m_sBadFaceProps_CrossedSeam.GetSize() != 0
         || sHealData.m_sBadFaceProps_MissingSeam.GetSize() != 0
         || !pBrep->AssertValid(NULL, SM_LEVEL_2, SM_WALK))
        { return SM_ERR ; }

      SmFixSeamPlanType eMoveU, eMoveV ;
      SmSurface * pHealedSurface = pFace->GetSurface() ;
      SmTrackTopologyChanges sUnchanged(pBrep, TRUE) ;
      SER(pFace->MoveSeam(eMoveU, eMoveV, *pProps->m_pCrvClassU, *pProps->m_pCrvClassV,
                         pProps->m_eBadCrossedSeam, pProps->m_eBadMissingSeam,
                         pProps->m_eBadNearMissSeam, TRUE, FALSE, NULL, NULL, NULL, pProps)) ;
      sUnchanged.StopTracking() ;
      if(   eMoveU != SM_FSP_NO_CHANGES || eMoveV != SM_FSP_NO_CHANGES
         || pFace->GetSurface() != pHealedSurface
         || sUnchanged.GetRmList()->GetSize() != 0
         || sUnchanged.GetAddList()->GetSize() != 0
         || sUnchanged.GetChgList()->GetSize() != 0)
        { return SM_ERR ; }
    }
  return SM_SUCCESS ;
}

static SmStatus my_test_missing_poles_refreshes_all_faces()
{
  SmContext sContext ;
  SmBrep * pBrep = new(sContext) SmBrep() ;
  SmObjDelete sCleanBrep(pBrep) ;
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion()) ;
  SER(sPrimitive.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement())) ;
  SmTArray<SmFace*> sFaces ;
  pBrep->GetFaces(sFaces) ;

  // Seed two cached missing-pole requests to exercise vertex-loop insertion and list refresh.
  SmTArray<SmVertex*> sTargetVertices ;
  SmTArray<SmEdge*> sTargetEdges ;
  SmTArray<SmFace*> sTargetFaces ;
  sTargetFaces.Add(sFaces[0]) ;
  sTargetFaces.Add(sFaces[1]) ;
  SmHealData sHealData(pBrep, sTargetVertices, sTargetEdges, sTargetFaces) ;
  for(ULONG ii=0; ii<2; ii++)
    {
      SmFaceProps * pProps = sHealData.m_sTgtFaceProps[ii] ;
      pProps->m_pFace = sFaces[ii] ;
      pProps->m_lBadMissingPoles = SM_SS_UMIN ;
      pProps->m_sPolePoints.SetSize(4) ;
      SmExtent2d sDomain = sFaces[ii]->GetSurface()->GetNaturalUVDomain() ;
      SmPoint2d sMid = (sDomain.GetMin() + sDomain.GetMax()) * 0.5 ;
      SER(sFaces[ii]->GetSurface()->EvaluatePoint(sMid, pProps->m_sPolePoints[0])) ;
      sHealData.m_sBadFaceProps_MissingPole.m_sProbArray.Add(pProps) ;
    }
  SmNewMarkAndLock sMark(&sContext) ;
  SER(sHealData.Fix_BadLoops(pBrep, sMark, SM_HO_FIX_BADLOOPS)) ;
  if(   sHealData.m_sNewVerticesOnPoles_FromFixBadLoops.GetSize() != 2
     || sHealData.m_sTgtVertices.GetSize() != 2
     || sHealData.m_sTgtFaces.GetSize() != 2
     || sHealData.m_sTgtFaceProps.GetSize() != 2
     || sHealData.m_sBadFaceProps_MissingPole.GetSize() != 0)
    { return SM_ERR ; }
  for(ULONG ii=0; ii<2; ii++)
    {
      SmTArray<SmLoop*> sLoops ;
      sFaces[ii]->GetLoops(sLoops) ;
      if(sLoops.GetSize() != 2)
        { return SM_ERR ; }
    }
  return SM_SUCCESS ;
}

static SmStatus my_test_missing_torus_seam_at_coincidence()
{
  MYPRINTF(_T("\n********** Entered: my_test_missing_torus_seam_at_coincidence")) ;

  // Clockwise UV rectangle: the retained face is the torus minus the patch.
  // Its top edge lies on V=1. Swap coordinates and reverse the traversal to
  // exercise the symmetric U-seam case. Reverse again for the bounded patch,
  // whose shared seam corner is outside the face and must stay unclassified.
  for(int iSwap=0; iSwap<2; iSwap++)
    for(int iPatch=0; iPatch<3; iPatch++)
      {
        SmContext sContext ;
        SmBrepConstructor sBuilder ;
        SmBrep * pBrep = sBuilder.StartBrep(sContext) ;
        NER(pBrep) ;
        SmObjDelete sCleanup(pBrep) ;
        pBrep->SetTolerance(1e-7, FALSE) ;
        SmShell * pOuter = NULL, * pInner = NULL ;
        SER(sBuilder.StartShell(pOuter, pInner)) ;
        SmFace * pFace = sBuilder.StartFace(SM_OT_SAME) ;
        NER(pFace) ;
        SmTorus * pTorus = NULL ;
        SER(SmTorus::CreateCanonical
          (sContext,           // in : allocation context
           SmAxis2Placement(), // in : torus placement
           2.0,                // in : major radius
           0.5,                // in : minor radius
           pTorus)) ;          // out: torus surface
        SER(sBuilder.SetFaceSurface(pTorus, pTorus->GetNaturalUVDomain())) ;
        NER(sBuilder.StartLoop(SM_OT_SAME)) ;
        SmPoint2d sUV[6] = {SmPoint2d(0.25,0.5), SmPoint2d(0.25,1.0),
                           SmPoint2d(0.5,1.0), SmPoint2d(0.5,0.5)} ;
        int iNumPoints = 4 ;
        SmBoolean bReverse = iSwap != (iPatch == 1) ;
        if(iPatch == 2)
          {
            // A patch crossing V=0: V has an interior FACE interval, but the
            // shared U/V seam corner is outside. U must not be marked missing.
            sUV[0] = SmPoint2d(0.25,0.75) ; sUV[1] = SmPoint2d(0.5,0.75) ;
            sUV[2] = SmPoint2d(0.5,1.0) ; sUV[3] = SmPoint2d(0.5,0.25) ;
            sUV[4] = SmPoint2d(0.25,0.25) ; sUV[5] = SmPoint2d(0.25,1.0) ;
            iNumPoints = 6 ;
            bReverse = iSwap != 0 ;
          }
        SmVertex * pVertices[6] ;
        for(int iPoint=0; iPoint<iNumPoints; iPoint++)
          {
            if(iSwap) { sUV[iPoint] = SmPoint2d(sUV[iPoint].y, sUV[iPoint].x) ; }
            SmPoint3d sPoint ;
            SER(pTorus->EvaluatePoint(sUV[iPoint], sPoint)) ;
            pVertices[iPoint] = sBuilder.StartVertexOfLoop(sPoint) ;
            NER(pVertices[iPoint]) ;
          }
        for(int iPoint=0; iPoint<iNumPoints; iPoint++)
          {
            int iStart = bReverse ? (iNumPoints-iPoint)%iNumPoints : iPoint ;
            int iEnd = bReverse ? (iNumPoints-1-iPoint)%iNumPoints : (iPoint+1)%iNumPoints ;
            NER(sBuilder.StartEdge
              (SM_OT_SAME,         // in : edgeuse orientation
               NULL,               // in : create a new edge
               pVertices[iStart],  // in : start vertex
               pVertices[iEnd])) ; // in : end vertex
            SmBSplineCurve * pCurve = NULL ;
            SmPoint2d sStart = sUV[iStart], sEnd = sUV[iEnd] ;
            if(iPatch == 2)
              {
                // Use V=0 for the two edges on the low side of the wrap.
                double & rdStart = iSwap ? sStart.x : sStart.y ;
                double & rdEnd = iSwap ? sEnd.x : sEnd.y ;
                if(rdStart == 1.0 && rdEnd == 0.25) { rdStart = 0.0 ; }
                if(rdEnd == 1.0 && rdStart == 0.25) { rdEnd = 0.0 ; }
              }
            // Use independent 3D curves, as in the imported STEP boundary.
            SmBoolean bConstU = sStart.x == sEnd.x;
            double dStart = bConstU ? sStart.y : sStart.x;
            double dEnd = bConstU ? sEnd.y : sEnd.x;
            SmExtent2d sDomain = pTorus->GetNaturalUVDomain();
            SmExtent1d sRange(smos_Min(dStart,dEnd),smos_Max(dStart,dEnd));
            if(bConstU) { sDomain.SetVInterval(sRange); }
            else { sDomain.SetUInterval(sRange); }
            SER(pTorus->CreateIsoParametricCurve
              (sContext,                      // in : allocation context
               bConstU ? SM_SP_U : SM_SP_V,   // in : constant parameter direction
               bConstU ? sStart.x : sStart.y, // in : constant parameter value
               0.0,                           // in : use default approximation tolerance
               pCurve,                        // out: independent 3D curve
               &sDomain));                    // in : trimmed surface domain
            // Reverse the NURBS definition without retaining analytic parameter metadata.
            SmBSplineCurve *pNurbs = new(sContext) SmBSplineCurve(3,pCurve->GetGwNurbPointer());
            delete pCurve;
            pCurve = pNurbs;
            if(dStart > dEnd)
              {
                SmExtent1d sNewInterval;
                SER(pCurve->ReverseParameterization(pCurve->GetNaturalInterval(),sNewInterval));
              }
            SER(sBuilder.SetEdgeCurve(pCurve, FALSE)) ;
            SER(sBuilder.SetLimits(SmVertexNode(pVertices[iStart]->GetPoint()), SmVertexNode(pVertices[iEnd]->GetPoint()))) ;
            SER(sBuilder.EndEdge()) ;
          }
        SER(sBuilder.EndLoop()) ;
        SER(sBuilder.EndFace()) ;
        SER(sBuilder.EndShell(FALSE)) ;
        SER(sBuilder.EndBrep()) ;

        SmCurveClassification sU, sV ;
        SmSurfParamType eCrossed, eMissing, eNear ;
        pFace->HasSeamProblem
          (sU,         // out: U seam classification
           sV,         // out: V seam classification
           eCrossed,   // out: illegally crossed seams
           eMissing,   // out: missing seams
           eNear) ;    // out: boundaries near seams
        if(iPatch == 2)
          {
            if(   eCrossed != SM_SP_NEITHER || eMissing != (iSwap ? SM_SP_U : SM_SP_V)
               || eNear != SM_SP_NEITHER)
              { return SM_ERR ; }
          }
        else if(iPatch == 1)
          {
            if(eCrossed != SM_SP_NEITHER || eMissing != SM_SP_NEITHER || eNear != SM_SP_NEITHER)
              { return SM_ERR ; }
          }
        else
          {
            if(eCrossed != SM_SP_BOTH || eMissing != SM_SP_BOTH || eNear != SM_SP_NEITHER)
              { return SM_ERR ; }
            SmCurveClassification & rMissing = iSwap ? sV : sU ;
            if(   rMissing.GetSize() != 1
               || rMissing[0].m_vStart.GetPointClass() != SM_PC_FACE
               || rMissing[0].m_vMid.GetPointClass() != SM_PC_FACE
               || rMissing[0].m_vEnd.GetPointClass() != SM_PC_FACE
               || rMissing[0].m_vStart.GetObject() != pFace
               || rMissing[0].m_vMid.GetObject() != pFace
               || rMissing[0].m_vEnd.GetObject() != pFace)
              { return SM_ERR ; }
          }

        // Classification must preserve the input boundary topology.
        if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != ULONG(iNumPoints) || pBrep->GetNumVertices() != ULONG(iNumPoints))
          { return SM_ERR ; }
        if(iPatch == 0 && iSwap == 0)
          {
            // This orientation matches the STEP fixture: seam movement leaves
            // both seams nonintersecting. The transposed case still intersects
            // a boundary and does not exercise the all-UNKNOWN fallback.
            // The missing flags must survive the canonical healer's refresh.
            SmHealData sHeal;
            SER(sHeal.HealBrep
              (pBrep,              // in : brep to heal
               SM_HO_FIX_MOVESEAM, // in : last healing stage to run
               NULL,               // in : all brep vertices
               NULL,               // in : all brep edges
               NULL,               // in : all brep faces
               FALSE));            // in : create UV trim curves
            pFace->HasSeamProblem
              (sU,       // out: U seam classification
               sV,       // out: V seam classification
               eCrossed, // out: illegally crossed seams
               eMissing, // out: missing seams
               eNear);   // out: boundaries near seams
            if(eMissing != SM_SP_BOTH) { return SM_ERR; }
            // Complete the same moved-boundary path as the analytic STEP fixture.
            SER(sHeal.HealBrep
              (pBrep,     // in : brep to heal
               SM_HO_ALL, // in : last healing stage to run
               NULL,      // in : all brep vertices
               NULL,      // in : all brep edges
               NULL,      // in : all brep faces
               TRUE));    // in : create UV trim curves
            SmTArray<SmLoop*> sLoops;
            pFace->GetLoops(sLoops);
            SmAssertArray sPointers,sValid;
            if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != 6 || pBrep->GetNumVertices() != 5
               || sLoops.GetSize() != 2
               || pBrep->ValidatePointers(&sPointers,SM_LEVEL_2,NULL) != SM_SUCCESS || sPointers.GetSize()
               || !pBrep->AssertValid(&sValid,SM_LEVEL_2,SM_WALK) || sValid.GetSize()) { return SM_ERR; }
          }
      }
  MYPRINTF(_T("\nHasSeamProblem torus coincidence: six cases passed")) ;
  return SM_SUCCESS ;
}

/***********************************************************************
PURPOSE --- Closed spheres have poles orthogonal to their periodic direction.
***********************************************************************/
static SmStatus my_test_closed_sphere_properties()
{
  MYPRINTF(_T("\n********** Entered: my_test_closed_sphere_properties"));
  for(ULONG lSwap=0; lSwap<2; ++lSwap)
    {
      SmContext  sContext;
      SmBrep    *pBrep = new(sContext) SmBrep();
      SmObjDelete sCleanup(pBrep);
      pBrep->SetTolerance(1e-7,FALSE);

      SmSphere *pSphere = NULL;
      SmShell  *pShell  = NULL;
      SmFace   *pFace   = NULL;
      SER(SmSphere::CreateCanonical(sContext,SmAxis2Placement(),2.0,pSphere));
      if(lSwap) { SER(pSphere->SwapUV()); }
      SER(pBrep->CreateFaceInRegionFromSurface
        (pBrep->GetInfiniteRegion(),    // in : region containing the face
         pSphere,                      // in : surface consumed by the face
         pSphere->GetNaturalUVDomain(), // in : full sphere domain
         pShell,                       // out: created shell
         pFace));                      // out: created face

      SmHealData sHeal;
      SER(sHeal.HealBrep(pBrep,SM_HO_CACHE_FACEPROPS_2));
      if(sHeal.m_sTgtFaceProps.GetSize() != 1 || !sHeal.m_sTgtFaceProps[0]->m_bClosedSurf)
        { return SM_ERR; }
    }
  return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- Rebuild a single-vertex sphere with its natural seam and poles.
***********************************************************************/
static SmStatus my_test_heal_single_vertex_sphere()
{
  MYPRINTF(_T("\n********** Entered: my_test_heal_single_vertex_sphere"));
  for(ULONG lSwap=0; lSwap<2; ++lSwap)
    {
      SmContext         sContext;
      SmBrepConstructor sBuilder;
      SmBrep           *pBrep = sBuilder.StartBrep(sContext);
      NER(pBrep);
      SmObjDelete sCleanup(pBrep);
      pBrep->SetTolerance(1e-7,FALSE);
      NER(sBuilder.StartRegion(TRUE,NULL));

      SmShell  *pOuter  = NULL;
      SmShell  *pInner  = NULL;
      SmSphere *pSphere = NULL;
      SER(sBuilder.StartShell(pOuter,pInner));
      // Swapping U/V reverses the natural surface normal.
      NER(sBuilder.StartFace(lSwap ? SM_OT_OPPOSITE : SM_OT_SAME));
      SER(SmSphere::CreateCanonical(sContext,SmAxis2Placement(),2.0,pSphere));
      if(lSwap) { SER(pSphere->SwapUV()); }
      SER(sBuilder.SetFaceSurface(pSphere,pSphere->GetNaturalUVDomain()));
      NER(sBuilder.StartAndEndSingleVertexLoop(SmPoint3d(0,0,2)));
      SER(sBuilder.EndFace());
      SER(sBuilder.EndShell(TRUE));
      SER(sBuilder.EndRegion());
      SER(sBuilder.EndBrep());
      if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != 0 || pBrep->GetNumVertices() != 1)
        { return SM_ERR; }

      SER(pBrep->HealBrep(SM_HO_ALL));
      if(   pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != 1 || pBrep->GetNumVertices() != 2
         || !pBrep->IsManifoldSolid())
        { return SM_ERR; }
      SmAssertArray sReports;
      SER(pBrep->ValidatePointers(&sReports));
      if(sReports.GetSize() || !pBrep->AssertValid()) { return SM_ERR; }
    }
  return SM_SUCCESS;
}

// Complete closed faces must retain an outer UV boundary on every healing pass.
static SmStatus my_test_closed_face_containment_repeated()
{
  MYPRINTF(_T("\n********** Entered: my_test_closed_face_containment_repeated"));
  for(ULONG lKind=0; lKind<2; ++lKind)
    {
      SmContext           sContext;
      SmBrep             *pBrep = new(sContext) SmBrep();
      SmObjDelete         sCleanup(pBrep);
      SmPrimitiveCreation sCreate(pBrep->GetInfiniteRegion());
      pBrep->SetTolerance(1e-7,FALSE);
      if(lKind == 0)
        {
          SER(sCreate.CreateSphere
            (2.0,                  // in : radius
             0.0,                  // in : start angle in degrees
             360.0,                // in : end angle in degrees
             SmAxis2Placement())); // in : placement
        }
      else
        {
          SER(sCreate.CreateTorus
            (3.0,                  // in : major radius
             1.0,                  // in : minor radius
             0.0,                  // in : start angle in degrees
             360.0,                // in : end angle in degrees
             SmAxis2Placement())); // in : placement
        }
      ULONG lFaces    = pBrep->GetNumFaces();
      ULONG lEdges    = pBrep->GetNumEdges();
      ULONG lVertices = pBrep->GetNumVertices();
      for(ULONG lPass=0; lPass<2; ++lPass)
        {
          SmHealData sHeal;
          SER(sHeal.HealBrep(pBrep,SM_HO_ALL));
          if(   sHeal.m_sNotYetFaceProps.GetSize() != 0
             || pBrep->GetNumFaces() != lFaces
             || pBrep->GetNumEdges() != lEdges
             || pBrep->GetNumVertices() != lVertices)
            { return SM_ERR; }
          SmAssertArray sReports;
          SER(pBrep->ValidatePointers(&sReports));
          if(sReports.GetSize() || !pBrep->AssertValid(&sReports,SM_LEVEL_2,SM_WALK) || sReports.GetSize())
            { return SM_ERR; }
        }
    }
  return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- Add natural boundaries without replacing vertex-only closed faces.
***********************************************************************/
static SmStatus my_test_heal_vertex_only_closed_surface
 (SmBoolean bTorus,        // in : TRUE = torus; FALSE = sphere
  ULONG     lCase,         // in : 0 = corner/pole, 1 = interior, 2 = shared contact, 3 = cavity, 4 = two vertices
  SmBoolean bSwapUV)       // in : swap surface parameters and reverse the outside faceuse
{
  TCHAR sLabel[SM_TBLOCK_SIZE];
  smos_sprintf(sLabel,_T("\nVertex-only closed face: torus=%d case=%lu swap=%d"),bTorus,lCase,bSwapUV);
  MYPRINTF(sLabel);
  SmContext         sContext;
  SmBrepConstructor sBuilder;
  SmBrep           *pBrep = sBuilder.StartBrep(sContext);
  NER(pBrep);
  SmObjDelete sCleanup(pBrep);
  pBrep->SetTolerance(1e-7,FALSE);

  SmBoolean  bShared   = lCase == 2;
  SmBoolean  bCavity   = lCase == 3;
  ULONG      lFaces    = bShared || bCavity ? 2 : 1;
  SmRegion  *pMaterial = NULL;
  SmRegion  *pVoid     = NULL;
  SmFace    *sFaces[2]    = {NULL,NULL};
  SmFaceuse *sFaceuses[4] = {NULL,NULL,NULL,NULL};
  SmShell   *sShells[4]   = {NULL,NULL,NULL,NULL};
  SmRegion  *sOwners[4]   = {NULL,NULL,NULL,NULL};
  for(ULONG lFace=0; lFace<lFaces; ++lFace)
    {
      SmRegion *pRegion = sBuilder.StartRegion(TRUE,bCavity ? pMaterial : NULL);
      NER(pRegion);
      if(lFace == 0) { pMaterial = pRegion; }
      if(bCavity && lFace == 1) { pVoid = pRegion; pVoid->SetIsVoid(TRUE); }
      SER(sBuilder.StartShell(sShells[2*lFace],sShells[2*lFace+1]));
      // The outside of each enclosed region includes the material around a cavity.
      sFaces[lFace] = sBuilder.StartFace(bSwapUV ? SM_OT_OPPOSITE : SM_OT_SAME);
      NER(sFaces[lFace]);

      SmSurface *pSurface = NULL;
      SmPoint3d  sPoint;
      if(bTorus)
        {
          SmTorus          *pTorus  = NULL;
          double            dMinor = bCavity && lFace == 1 ? .5 : 1.;
          SmAxis2Placement  sAxes;
          if(bShared) { sAxes.Translate(SmVector3d(8.*lFace,0.,0.)); }
          SER(SmTorus::CreateCanonical
            (sContext, // in : geometry context
             sAxes,    // in : placement
             3.,       // in : major radius
             dMinor,   // in : minor radius
             pTorus)); // out: surface
          pSurface = pTorus;
          sPoint.Set(3.+dMinor,0.,0.);
        }
      else
        {
          SmSphere         *pSphere = NULL;
          double            dRadius = bCavity && lFace == 1 ? 1. : 2.;
          SmAxis2Placement  sAxes;
          if(bShared) { sAxes.Translate(SmVector3d(0.,0.,4.*lFace)); }
          SER(SmSphere::CreateCanonical
            (sContext,  // in : geometry context
             sAxes,     // in : placement
             dRadius,   // in : radius
             pSphere)); // out: surface
          pSurface = pSphere;
          sPoint.Set(0.,0.,dRadius);
        }
      SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
      if(lCase == 1)
        {
          SmPoint2d sUV(sDomain.GetUMin()+.3*sDomain.XLength(),sDomain.GetVMin()+.6*sDomain.YLength());
          SER(pSurface->EvaluatePoint(sUV,sPoint));
        }
      if(bSwapUV) { SER(pSurface->SwapUV()); }
      SER(sBuilder.SetFaceSurface(pSurface,pSurface->GetNaturalUVDomain()));
      NER(sBuilder.StartAndEndSingleVertexLoop(sPoint));
      if(lCase == 4)
        {
          SmPoint3d sSecondPoint = bTorus ? SmPoint3d(2.,0.,0.) : SmPoint3d(0.,0.,-2.);
          NER(sBuilder.StartAndEndSingleVertexLoop(sSecondPoint));
        }
      SER(sBuilder.EndFace());
      SER(sBuilder.EndShell(FALSE));
      SER(sBuilder.EndRegion());
    }
  SER(sBuilder.EndBrep());
  SmTArray<SmVertex*> sOriginalVertices;
  pBrep->GetVertices(sOriginalVertices);
  if(bShared)
    {
      SER(pBrep->GlueVertices(sOriginalVertices[0],sOriginalVertices[1]));
      pBrep->GetVertices(sOriginalVertices);
    }
  // Gluing can combine shells. Record the graph that the healer receives.
  for(ULONG lFace=0; lFace<lFaces; ++lFace)
    {
      sFaces[lFace]->GetFaceuses(sFaceuses[2*lFace],sFaceuses[2*lFace+1]);
      for(ULONG lSide=2*lFace; lSide<2*lFace+2; ++lSide)
        {
          sShells[lSide] = sFaceuses[lSide]->GetShell();
          sOwners[lSide] = sShells[lSide]->GetRegion();
        }
      if(!sFaces[lFace]->IsClosedSurfaceWithVertexLoops() || sFaces[lFace]->IsDegenerate(TRUE))
        { return SM_ERR; }
    }
  ULONG lExpectedEdges = bTorus ? (bShared ? 5 : lCase == 4 ? 3 : 2*lFaces) : lFaces;
  ULONG lExpectedVertices = bTorus ? (bShared || lCase == 1 || lCase == 4 ? 2 : lFaces)
                                   : (bShared || lCase == 1 ? 3 : 2*lFaces);
  for(ULONG lPass=0; lPass<2; ++lPass)
    {
      SmHealData sHeal;
      SER(sHeal.HealBrep
        (pBrep,         // in : target BRep
         SM_HO_ALL,     // in : run the complete healer
         NULL,          // in : all vertices
         NULL,          // in : all edges
         NULL,          // in : all faces
         lPass != 0));  // in : also create optional UV trims on the second pass
      // Complete seam boundaries must remain classifiable on both healing passes.
      if(   pBrep->GetNumFaces() != lFaces
         || pBrep->GetNumEdges() != lExpectedEdges
         || pBrep->GetNumVertices() != lExpectedVertices
         || sHeal.m_sNotYetFaceProps.GetSize() != 0)
        { return SM_ERR; }
      SmTArray<SmFace*>   sCurrentFaces;
      SmTArray<SmVertex*> sCurrentVertices;
      pBrep->GetFaces(sCurrentFaces);
      pBrep->GetVertices(sCurrentVertices);
      for(ULONG lVertex=0; lVertex<sOriginalVertices.GetSize(); ++lVertex)
        { if(!sCurrentVertices.IsIn(sOriginalVertices[lVertex])) { return SM_ERR; } }
      for(ULONG lFace=0; lFace<lFaces; ++lFace)
        {
          if(!sCurrentFaces.IsIn(sFaces[lFace])) { return SM_ERR; }
          SmFaceuse *pUp = NULL, *pDown = NULL;
          sFaces[lFace]->GetFaceuses(pUp,pDown);
          if(pUp != sFaceuses[2*lFace] || pDown != sFaceuses[2*lFace+1]) { return SM_ERR; }
          for(ULONG lSide=2*lFace; lSide<2*lFace+2; ++lSide)
            {
              if(   sFaceuses[lSide]->GetShell() != sShells[lSide]
                 || sShells[lSide]->GetRegion() != sOwners[lSide]) { return SM_ERR; }
            }
          SmCurveClassification sU,sV;
          SmSurfParamType       eCrossed,eMissing,eNear;
          sFaces[lFace]->HasSeamProblem
            (sU,       // out: U seam classification
             sV,       // out: V seam classification
             eCrossed, // out: crossed seams
             eMissing, // out: missing seams
             eNear);   // out: near-miss seams
          if(eMissing != SM_SP_NEITHER || eCrossed != SM_SP_NEITHER) { return SM_ERR; }
        }
      if(bCavity && (pMaterial->IsVoid() || !pVoid->IsVoid())) { return SM_ERR; }
      SmAssertArray sReports;
      SER(pBrep->ValidatePointers(&sReports));
      if(sReports.GetSize() || !pBrep->AssertValid(&sReports,SM_LEVEL_2,SM_WALK) || sReports.GetSize())
        { return SM_ERR; }
    }
  return SM_SUCCESS;
}

static SmStatus my_test_vertex_only_plane_is_degenerate()
{
  SmContext         sContext;
  SmBrepConstructor sBuilder;
  SmBrep           *pBrep = sBuilder.StartBrep(sContext);
  NER(pBrep);
  SmObjDelete sCleanup(pBrep);
  SmShell *pOuter = NULL, *pInner = NULL;
  SER(sBuilder.StartShell(pOuter,pInner));
  SmFace *pFace = sBuilder.StartFace(SM_OT_SAME);
  NER(pFace);
  SmPlane *pPlane = NULL;
  SER(SmPlane::CreateCanonical(sContext,SmAxis2Placement(),pPlane));
  SER(sBuilder.SetFaceSurface(pPlane,SmExtent2d(SmPoint2d(-1.,-1.),SmPoint2d(1.,1.))));
  NER(sBuilder.StartAndEndSingleVertexLoop(SmPoint3d(0.,0.,0.)));
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(FALSE));
  SER(sBuilder.EndBrep());
  if(pFace->IsClosedSurfaceWithVertexLoops() || !pFace->IsDegenerate(TRUE)) { return SM_ERR; }
  SmHealData sHeal;
  SER(sHeal.HealBrep(pBrep,SM_HO_CACHE_DEGEN_FACES));
  if(   pBrep->GetNumFaces() != 1
     || sHeal.m_sBadFaceProps_DegenFaces.GetSize() != 1
     || sHeal.m_sBadFaceProps_DegenFaces[0]->m_bBadDegenFace != TRUE) { return SM_ERR; }
  for(ULONG lPass=0; lPass<2; ++lPass)
    {
      SER(pBrep->HealBrep(SM_HO_ALL));
      if(   pBrep->GetNumFaces() != 0
         || pBrep->GetNumEdges() != 0
         || pBrep->GetNumVertices() != 0) { return SM_ERR; }
      SmAssertArray sReports;
      SER(pBrep->ValidatePointers(&sReports));
      if(sReports.GetSize() || !pBrep->AssertValid(&sReports,SM_LEVEL_2,SM_WALK) || sReports.GetSize())
        { return SM_ERR; }
    }
  return SM_SUCCESS;
}

static SmStatus my_test_heal_vertex_only_closed_surfaces()
{
  // Run every independent test and retain the first failure in either build mode.
  SmStatus sResult = SM_SUCCESS;

  PT_RUN(sResult, my_test_vertex_only_plane_is_degenerate());
  for(ULONG lSurface=0; lSurface<2; ++lSurface)
    for(ULONG lCase=0; lCase<5; ++lCase)
      for(ULONG lSwap=0; lSwap<2; ++lSwap)
        {
          PT_RUN(sResult, my_test_heal_vertex_only_closed_surface(lSurface != 0,lCase,lSwap != 0));
        }
  return sResult;
}

static SmStatus my_test_healer_continues_after_pole_split_failure()
{
  MYPRINTF(_T("\n********** Entered: my_test_healer_continues_after_pole_split_failure (expected endpoint diagnostic)"));
  for(ULONG lCase=0; lCase<3; ++lCase)
    {
      SmContext         sContext;
      SmBrepConstructor sBuilder;
      SmBrep           *pBrep = sBuilder.StartBrep(sContext);
      SmObjDelete       sCleanup(pBrep);
      SmShell          *pOuter = NULL, *pInner = NULL;
      SmFace           *pFaces[2] = { NULL, NULL };
      SmEdge           *pEdges[2] = { NULL, NULL };
      SmVertex         *pFailedVertex = NULL;
      SER(sBuilder.StartShell(pOuter, pInner));
      for(ULONG lFace=0; lFace<2; ++lFace)
        {
          pFaces[lFace] = sBuilder.StartFace(SM_OT_SAME);
          NER(pFaces[lFace]);
          SmPoint3d        sOrigin(lFace == 0 || lCase == 2 ? 0.0 : 10.0, 0, 0);
          SmAxis2Placement sFrame;
          SER(sFrame.SetCanonical(sOrigin, SmVector3d(1,0,0), SmVector3d(0,1,0)));
          if(lCase == 2 && lFace == 1)
            { SER(sFrame.SetCanonical(sOrigin, SmVector3d(0,0,-1), SmVector3d(0,1,0))); }
          SmSphere *pSphere = NULL;
          SER(SmSphere::CreateCanonical(sContext, sFrame, 2.0, pSphere));
          SER(sBuilder.SetFaceSurface(pSphere, pSphere->GetNaturalUVDomain()));
          NER(sBuilder.StartLoop(SM_OT_SAME));
          if(lCase == 2 && lFace == 1)
            {
              // Same meridian, different sphere axes: this face has poles at +/-X.
              pEdges[1] = sBuilder.StartEdge(SM_OT_SAME, pEdges[0], NULL, NULL);
              NER(pEdges[1]);
              SER(sBuilder.EndEdge());
              SER(sBuilder.EndLoop());
              SER(sBuilder.EndFace());
              continue;
            }
          double     dRootTwo = sqrt(2.0);
          SmPoint3d  sCenter  = lFace == 0 ? sOrigin : SmPoint3d(11,0,1);
          double     dRadius  = lFace == 0 ? 2.0 : dRootTwo;
          SmVector3d sAxisX   = lFace == 0 ? SmVector3d(sin(1e-4),0,-cos(1e-4)) : SmVector3d(0,1,0);
          SmVector3d sAxisY   = lFace == 0 ? SmVector3d(cos(1e-4),0,sin(1e-4)) : SmVector3d(-1.0/dRootTwo,0,1.0/dRootTwo);
          SmPoint3d  sStart   = sCenter + dRadius*sAxisX;
          SmVertex  *pStart   = sBuilder.StartVertexOfLoop(sStart);
          if(lFace == 0) { pFailedVertex = pStart; }
          pEdges[lFace] = sBuilder.StartEdge(SM_OT_SAME, NULL, pStart, pStart);
          NER(pEdges[lFace]);
          SmCircle *pCircle = new(sContext) SmCircle
            (sCenter,          // in : center
             sAxisX,           // in : first circle axis
             sAxisY,           // in : second circle axis
             SmExtent1d(0,360), // in : angular interval in degrees
             dRadius,          // in : radius
             3,                // in : dimension
             &sContext);       // in : allocation context
          SER(sBuilder.SetEdgeCurve(pCircle, FALSE));
          SER(sBuilder.SetLimits(SmVertexNode(sStart), SmVertexNode(sStart)));
          SER(sBuilder.EndEdge());
          SER(sBuilder.EndLoop());
          SER(sBuilder.EndFace());
        }
      SER(sBuilder.EndShell(FALSE));
      SER(sBuilder.EndBrep());

      SmHealData sHealData;
      SER(sHealData.HealBrep(pBrep, SM_HO_CACHE_FACEPROPS_2));
      if(sHealData.m_sBadFaceProps_CrossedPole.GetSize() != 2) { return SM_ERR; }
      // Make failure ordering independent of the Brep's face enumeration order.
      SmFaceProps *pFailedProps = NULL, *pGoodProps = NULL;
      for(ULONG lFace=0; lFace<sHealData.m_sTgtFaceProps.GetSize(); ++lFace)
        {
          SmFaceProps *pProps = sHealData.m_sTgtFaceProps[lFace];
          if(pProps->m_pFace == pFaces[0]) { pFailedProps = pProps; }
          if(pProps->m_pFace == pFaces[1]) { pGoodProps = pProps; }
        }
      NER(pFailedProps);
      NER(pGoodProps);
      sHealData.m_sBadFaceProps_CrossedPole.ReSet();
      sHealData.m_sBadFaceProps_CrossedPole.Add(pFailedProps, TRUE);
      sHealData.m_sBadFaceProps_CrossedPole.Add(pGoodProps, TRUE);
      if(lCase < 2 && !pGoodProps->m_bBadCrossedSeam) { return SM_ERR; }
      if(lCase == 2 && pFailedProps->m_pCrossedPoleEdge != pGoodProps->m_pCrossedPoleEdge)
        { return SM_ERR; }

      // Expose the detector/splitter tolerance disagreement without changing its policy.
      pFailedVertex->SetTolerance(SmZoneTol3d(1e-3), FALSE);
      SmExtent1d sFailedInterval = pEdges[0]->GetInterval();
      SmTemporaryChangeValue<SmBoolean> sEditing(pBrep->m_bEditingEnabled, TRUE);
      SmNewMarkAndLock sMark(&sContext);
      // Also exercise a direct late call; its refresh may move the good face's seam.
      if(lCase == 1) { sHealData.m_eDoneHealOp = SM_HO_FIX_SPLITEDGE_ATSEAM; }
      SER(sHealData.Fix_SplitEdgesAtSeam(pBrep, sMark, SM_HO_FIX_SPLITEDGE_ATSEAM));
      if(lCase == 2)
        {
          // The first face's rejected parameter must not suppress either pole on
          // the second face, even though both cached candidates use one edge.
          SmTArray<SmVertex*> sVertices;
          SmBoolean bFoundNorth = FALSE, bFoundSouth = FALSE;
          pFaces[1]->GetVertices(sVertices);
          for(ULONG lVertex=0; lVertex<sVertices.GetSize(); ++lVertex)
            {
              bFoundNorth |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(2,0,0)) < 1e-6;
              bFoundSouth |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(-2,0,0)) < 1e-6;
            }
          if(!bFoundNorth || !bFoundSouth) { return SM_ERR; }
          continue;
        }
      if(   !(sFailedInterval == pEdges[0]->GetInterval())
         || sHealData.m_sBadFaceProps_CrossedPole.GetSize() != 1
         || !sHealData.m_sBadFaceProps_CrossedPole[0]->m_bBadCrossedPole
         || sHealData.m_sBadFaceProps_CrossedPole[0]->m_pCrossedPoleEdge != pEdges[0])
        { return SM_ERR; }

      SmBoolean bFoundPole = FALSE, bFoundSeam = FALSE;
      SmTArray<SmVertex*> sVertices;
      pFaces[1]->GetVertices(sVertices);
      for(ULONG lVertex=0; lVertex<sVertices.GetSize(); ++lVertex)
        {
          bFoundPole |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(10,0,2)) < 1e-6;
          bFoundSeam |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(12,0,0)) < 1e-6;
        }
      if(!bFoundPole || (lCase == 0 && !bFoundSeam)) { return SM_ERR; }
      for(ULONG lFace=0; lFace<sHealData.m_sTgtFaceProps.GetSize(); ++lFace)
        {
          SmFaceProps *pProps = sHealData.m_sTgtFaceProps[lFace];
          if(pProps->m_pFace == pFaces[1] && pProps->m_bBadCrossedSeam) { return SM_ERR; }
        }

      // Both attempts must survive property replacement and later loop-cache resets.
      if(sHealData.m_sRanFaceProps_SplitEdgesAtSeam.GetSize() != 2) { return SM_ERR; }
      SmBoolean bReportedFailure = FALSE, bReportedSuccess = FALSE;
      for(ULONG lFace=0; lFace<2; ++lFace)
        {
          SmFaceProps *pProps = sHealData.m_sRanFaceProps_SplitEdgesAtSeam[lFace];
          if(!sHealData.m_sTgtFaceProps.IsIn(pProps)) { return SM_ERR; }
          SmTriedLinkType eResult = pProps->m_eBeenThroughSplitEdgesAtSeam;
          bReportedFailure |= pProps->m_pFace == pFaces[0] && eResult == SM_TRY_NOFIX;
          bReportedSuccess |= pProps->m_pFace == pFaces[1] && eResult == SM_TRY_FIXED;
          pProps->ReSet(SM_PROPSTAGE_3);
          if(pProps->m_eBeenThroughSplitEdgesAtSeam != eResult) { return SM_ERR; }
        }
      if(!bReportedFailure || !bReportedSuccess) { return SM_ERR; }
    }
  return SM_SUCCESS;
}

static SmStatus my_test_healer_splits_boundary_at_pole()
{
  MYPRINTF(_T("\n********** Entered: my_test_healer_splits_boundary_at_pole"));
  // A meridian with one or two interior poles, then a small circle crossing
  // both the north pole and the seam away from that pole.
  for(ULONG lCase=0; lCase<4; ++lCase)
    {
      SmContext         sContext;
      SmBrepConstructor sBuilder;
      SmBrep           *pBrep = sBuilder.StartBrep(sContext);
      SmObjDelete       sCleanup(pBrep);
      SmShell          *pOuter = NULL, *pInner = NULL;
      SER(sBuilder.StartShell(pOuter, pInner));
      SmFace *pFace = sBuilder.StartFace(SM_OT_SAME);
      NER(pFace);
      SmSphere *pSphere = NULL;
      SER(SmSphere::CreateCanonical(sContext, SmAxis2Placement(), 2.0, pSphere));
      SER(sBuilder.SetFaceSurface(pSphere, pSphere->GetNaturalUVDomain()));
      NER(sBuilder.StartLoop(SM_OT_SAME));
      double     dRootTwo = sqrt(2.0);
      SmPoint3d  sCenter  = lCase == 2 ? SmPoint3d(1,0,1) : SmPoint3d(0,0,0);
      double     dRadius  = lCase == 2 ? dRootTwo : 2.0;
      SmVector3d sAxisX   = lCase == 0 ? SmVector3d(0,0,1) : SmVector3d(1,0,0);
      SmVector3d sAxisY   = lCase == 0 ? SmVector3d(1,0,0) : SmVector3d(0,0,-1);
      if(lCase == 2)
        { sAxisX = SmVector3d(0,1,0); sAxisY = SmVector3d(-1.0/dRootTwo,0,1.0/dRootTwo); }
      SmPoint3d sStart = sCenter + dRadius*sAxisX;
      SmVertex *pStart = sBuilder.StartVertexOfLoop(sStart);
      SmEdge   *pOriginalEdge = sBuilder.StartEdge(SM_OT_SAME, NULL, pStart, pStart);
      NER(pOriginalEdge);
      SmCircle *pCircle = new(sContext) SmCircle
        (sCenter,            // in : center
         sAxisX,             // in : first circle axis
         sAxisY,             // in : second circle axis
         SmExtent1d(0,360),   // in : angular interval in degrees
         dRadius,            // in : radius
         3,                  // in : dimension
         &sContext);         // in : allocation context
      SER(sBuilder.SetEdgeCurve(pCircle, FALSE));
      SER(sBuilder.SetLimits(SmVertexNode(sStart), SmVertexNode(sStart)));
      SER(sBuilder.EndEdge());
      SER(sBuilder.EndLoop());
      SER(sBuilder.EndFace());
      SER(sBuilder.EndShell(FALSE));
      SER(sBuilder.EndBrep());

      SmHealData sHealData;
      SER(sHealData.HealBrep(pBrep, SM_HO_CACHE_FACEPROPS_2));
      SmTArray<SmEdge*>   sEdges;
      SmTArray<SmVertex*> sVertices;
      pBrep->GetEdges(sEdges);
      pBrep->GetVertices(sVertices);
      if(   sEdges.GetSize() != 1 || sVertices.GetSize() != 1
         || sHealData.m_sBadFaceProps_CrossedPole.GetSize() != 1
         || sHealData.m_sBadFaceProps_CrossedPole[0]->m_pCrossedPoleEdge != pOriginalEdge)
        { return SM_ERR; }

      if(lCase == 2 && !sHealData.m_sTgtFaceProps[0]->m_bBadCrossedSeam)
        { return SM_ERR; }

      // Exercise the normal healer path for meridians. For the small circle,
      // preserve the current seam to test a seam crossing after the pole split.
      SmTemporaryChangeValue<SmBoolean> sEditing(pBrep->m_bEditingEnabled, TRUE);
      SmNewMarkAndLock sMark(&sContext);
      if(lCase == 3)
        {
          // Exercise a real error after MakeVertexSplitEdge changes the interval.
          class SmFailCopyCircle : public SmCircle
            {
            public:
              SmFailCopyCircle(const SmCircle &crCircle) : SmCircle(crCircle) { }
              SmStatus Copy
                (const SmContext &crContext,  // in : allocation context
                 SmCurve        *&rpCurve)   // out: copied curve
                const override
                {
                  SM_REF1(crContext);
                  rpCurve = NULL;
                  return SM_ERR;
                }
            };
          SmFailCopyCircle *pFailCurve = new(sContext) SmFailCopyCircle(*pCircle);
          pOriginalEdge->SetCurve(pFailCurve, TRUE);
          pFailCurve->SetOwner(pOriginalEdge);
          SmExtent1d sBefore = pOriginalEdge->GetInterval();
          SmStatus sStatus = sHealData.Fix_SplitEdgesAtSeam(pBrep, sMark, SM_HO_FIX_SPLITEDGE_ATSEAM);
          if(sStatus == SM_SUCCESS || sBefore == pOriginalEdge->GetInterval()) { return SM_ERR; }
          continue;
        }
      if(lCase < 2)
        { SER(sHealData.HealBrep(pBrep, SM_HO_FIX_SPLITEDGE_ATSEAM)); }
      else
        { SER(sHealData.Fix_SplitEdgesAtSeam(pBrep, sMark, SM_HO_FIX_SPLITEDGE_ATSEAM)); }
      if(   sHealData.m_sBadFaceProps_CrossedPole.GetSize() != 0
         || sHealData.m_sRanFaceProps_SplitEdgesAtSeam.GetSize() != 1
         || sHealData.m_sRanFaceProps_SplitEdgesAtSeam[0]->m_pFace != pFace
         || sHealData.m_sRanFaceProps_SplitEdgesAtSeam[0]->m_eBeenThroughSplitEdgesAtSeam != SM_TRY_FIXED)
        { return SM_ERR; }
      SmBoolean bFoundNorth = FALSE, bFoundSouth = FALSE, bFoundSeam = FALSE;
      sVertices.ReSet();
      pBrep->GetVertices(sVertices);
      for(ULONG lVertex=0; lVertex<sVertices.GetSize(); ++lVertex)
        {
          bFoundNorth |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(0,0,2)) < 1e-6;
          bFoundSouth |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(0,0,-2)) < 1e-6;
          bFoundSeam |= sVertices[lVertex]->GetPoint().DistanceBetween(SmPoint3d(2,0,0)) < 1e-6;
        }
      if(!bFoundNorth || (lCase < 2 && !bFoundSouth)) { return SM_ERR; }
      if(lCase == 2 && !bFoundSeam) { return SM_ERR; }
      for(ULONG lEdge=0; lEdge<sHealData.m_sTgtEdgeProps.GetSize(); ++lEdge)
        {
          SmEdgeProps *pProps = sHealData.m_sTgtEdgeProps[lEdge];
          if(!(pProps->m_sEdgeInterval == pProps->m_pEdge->GetInterval())) { return SM_ERR; }
        }
      // Owning face properties must be rebuilt too, not just the split edges.
      SmFaceProps sFreshProps(&sContext);
      SER(sFreshProps.SetProps_Stage2(pFace, 0, SM_HO_CACHE_FACEPROPS_2));
      SmFaceProps *pProps = sHealData.m_sTgtFaceProps[0];
      if(   pProps->m_bBadCrossedPole || pProps->m_pCrossedPoleEdge != NULL
         || pProps->m_lNoVertexPoles != sFreshProps.m_lNoVertexPoles
         || pProps->m_eBadCrossedSeam != sFreshProps.m_eBadCrossedSeam)
        { return SM_ERR; }
    }
  return SM_SUCCESS;
}

static SmStatus my_test_pole_crossing_endpoint_tolerances()
{
  MYPRINTF(_T("\n********** Entered: my_test_pole_crossing_endpoint_tolerances"));
  // A two-edge small circle crosses only the north pole. Put its nearby vertex
  // before or after the pole to exercise the start and end of the crossing edge.
  for(ULONG lEnd=0; lEnd<2; ++lEnd)
    for(ULONG lCase=0; lCase<3; ++lCase)
      {
        SmContext         sContext;
        SmBrepConstructor sBuilder;
        SmBrep           *pBrep = sBuilder.StartBrep(sContext);
        SmObjDelete       sCleanup(pBrep);
        SmShell          *pOuter = NULL, *pInner = NULL;
        SER(sBuilder.StartShell(pOuter, pInner));
        SmFace *pFace = sBuilder.StartFace(SM_OT_SAME);
        NER(pFace);
        // A nonperiodic hemisphere keeps seam classification out of this test.
        SmExtent2d sAngles(SmExtent1d(-90,90), SmExtent1d(-90,90));
        SmSphere *pSphere = new(sContext) SmSphere
          (SmPoint3d(0,0,0),        // in : sphere origin
           SmVector3d(1,0,0),       // in : first frame axis
           SmVector3d(0,1,0),       // in : second frame axis
           sAngles,                 // in : angular domain in degrees
           2.0,                    // in : radius
           FALSE,                  // in : keep U and V directions
           FALSE,                  // in : outward sphere
           NULL,                   // in : generate the meridian curve
           &sContext);             // in : allocation context
        SER(sBuilder.SetFaceSurface(pSphere, pSphere->GetNaturalUVDomain()));
        NER(sBuilder.StartLoop(SM_OT_SAME));

        double     dRadius  = sqrt(2.0);
        double     dAngle   = 180.0 + (lEnd == 0 ? -0.01 : 0.01);
        double     dRadians = SM_DEG2RAD(dAngle);
        SmPoint3d  sCenter(1,0,1), sFar(2,0,0);
        SmVector3d sAxisX(1.0/dRadius,0,-1.0/dRadius), sAxisY(0,1,0);
        SmPoint3d  sNear = sCenter + dRadius*(cos(dRadians)*sAxisX + sin(dRadians)*sAxisY);
        SmVertex  *pFar  = sBuilder.StartVertexOfLoop(sFar);
        SmVertex  *pNear = sBuilder.StartVertexOfLoop(sNear);
        SmEdge    *pEdges[2] = { NULL, NULL };
        for(ULONG lEdge=0; lEdge<2; ++lEdge)
          {
            pEdges[lEdge] = sBuilder.StartEdge
              (SM_OT_SAME,                // in : edgeuse orientation
               NULL,                      // in : create a new edge
               lEdge == 0 ? pFar : pNear,  // in : existing start vertex
               lEdge == 0 ? pNear : pFar); // in : existing end vertex
            NER(pEdges[lEdge]);
            SmExtent1d sArcAngles(lEdge == 0 ? 0.0 : dAngle, lEdge == 0 ? dAngle : 360.0);
            SmCircle *pCircle = new(sContext) SmCircle
              (sCenter,                       // in : center
               sAxisX,                        // in : first circle axis
               sAxisY,                        // in : second circle axis
               sArcAngles,                    // in : angular interval in degrees
               dRadius,                       // in : radius
               3,                             // in : dimension
               &sContext);                    // in : allocation context
            SER(sBuilder.SetEdgeCurve(pCircle, FALSE));
            SER(sBuilder.SetLimits(SmVertexNode(lEdge == 0 ? sFar : sNear),
                                   SmVertexNode(lEdge == 0 ? sNear : sFar)));
            SER(sBuilder.EndEdge());
          }
        SER(sBuilder.EndLoop());
        SER(sBuilder.EndFace());
        SER(sBuilder.EndShell(FALSE));
        SER(sBuilder.EndBrep());

        // A large vertex tolerance covers the pole. Small tolerances retain the
        // interior crossing; the last case needs both vertex and face tolerances.
        pFace->SetTolerance(SmZoneTol3d(lCase == 2 ? 1.5e-4 : 1e-7), FALSE);
        pNear->SetTolerance(SmZoneTol3d(lCase == 0 ? 1e-3 : lCase == 2 ? 1.5e-4 : 1e-7), FALSE);
        pFar->SetTolerance(SmZoneTol3d(1e-7), FALSE);
        for(ULONG lEdge=0; lEdge<2; ++lEdge)
          { pEdges[lEdge]->SetTolerance(SmZoneTol3d(1e-7), FALSE); }
        SmFaceProps sProps(&sContext);
        SER(sProps.SetProps_Stage2(pFace, 0, SM_HO_CACHE_FACEPROPS_2));
        SmBooleanUL bCrossing = lCase == 1;
        if(sProps.m_bBadCrossedPole != bCrossing) { return SM_ERR; }
        if(bCrossing)
          {
            SmPoint3d sCrossing;
            if(sProps.m_pCrossedPoleEdge != pEdges[lEnd == 0 ? 1 : 0]) { return SM_ERR; }
            SER(sProps.m_pCrossedPoleEdge->GetCurve()->EvaluatePoint(sProps.m_dCrossedPoleParameter, sCrossing));
            if(sCrossing.DistanceBetween(SmPoint3d(0,0,2)) > 1e-6) { return SM_ERR; }
          }
        else if(sProps.m_pCrossedPoleEdge != NULL) { return SM_ERR; }
      }
  return SM_SUCCESS;
}

static SmStatus my_test_missing_seams_from_loop_orientation()
{
  MYPRINTF(_T("\n********** Entered: my_test_missing_seams_from_loop_orientation"));
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
      // All boundaries lie strictly inside the UV rectangle. Only a CW loop
      // gives evidence that the completely nonintersecting seams are in the face.
      SmCurveClassification sU,sV;
      SmSurfParamType eCrossed,eMissing,eNear;
      pFace->HasSeamProblem
        (sU,       // out: U seam classification
         sV,       // out: V seam classification
         eCrossed, // out: illegally crossed seams
         eMissing, // out: missing seams
         eNear);   // out: boundaries near seams
      if(eCrossed != SM_SP_NEITHER || eMissing != (bReverse ? SM_SP_BOTH : SM_SP_NEITHER)
         || eNear != SM_SP_NEITHER) { return SM_ERR; }
      SmCurveClassification *pClasses[2] = {&sU,&sV};
      for(ULONG lPoint=0; lPoint<2; ++lPoint)
        {
          SmPointClassType eExpectedClass = bReverse ? SM_PC_FACE : SM_PC_UNKNOWN;
          if(pClasses[lPoint]->GetSize() != 1
             || (*pClasses[lPoint])[0].m_vStart.GetPointClass() != eExpectedClass
             || (*pClasses[lPoint])[0].m_vMid.GetPointClass() != eExpectedClass
             || (*pClasses[lPoint])[0].m_vEnd.GetPointClass() != eExpectedClass) { return SM_ERR; }
        }
      ULONG lExpectedCount = (lCase == 8 || lCase == 9) ? 5 : 1;
      if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != lExpectedCount || pBrep->GetNumVertices() != lExpectedCount)
        { return SM_ERR; }

      if(lCase == 7 || lCase == 8 || lCase >= 10)
        {
          // Exercise the first cut separately in both orders, then full healing
          // for a curved hole and multiple holes. Defining boundary edges survive.
          pBrep->m_bEditingEnabled = TRUE;
          if(lCase == 7 || lCase == 8)
            {
              // Stage 3 cannot classify this single-edge circle before seams
              // exist. Use four arcs for the end-to-end healer regressions;
              // the direct insertion cases below retain the single closed edge.
              SmEdge *pRemainder = sUses[0]->GetEdge();
              for(ULONG lIndex=1; lIndex<4; ++lIndex)
                {
                  SmEdge *pFirst = NULL,*pSecond = NULL;
                  SmVertex *pSplitVertex = NULL;
                  SER(pBrep->MakeVertexSplitEdge
                    (pRemainder,     // in : edge to split
                     lIndex*0.25,    // in : split parameter
                     pFirst,         // out: first edge
                     pSecond,        // out: second edge
                     pSplitVertex)); // out: split vertex
                  pRemainder = pFirst->GetInterval().GetMax() > lIndex*0.25 ? pFirst : pSecond;
                }
            }
          ULONG lNumEdges = pBrep->GetNumEdges(), lNumVertices = pBrep->GetNumVertices();
          SmTArray<SmLoop*> sOriginalLoops;
          pFace->GetLoops(sOriginalLoops);
          SER(pFace->RemoveUVTrimCurves());
          pBrep->m_bEditingEnabled = TRUE;
          if(lCase == 10 || lCase == 11)
            {
              SmTemporaryChangeValue<SmBoolean> sDoingBool(sContext.GetDoingBooleanRef(),TRUE);
              SmCurveClassification &rFirst = lCase == 10 ? sU : sV;
              SmCurveClassification &rSecond = lCase == 10 ? sV : sU;
              SmTArray<SmFace*> sNewFaces;
              SmTArray<SmEdge*> sNewEdges;
              SmTArray<SmVertex*> sNewVertices;
              SmTrackTopologyChanges sChanges(pBrep,TRUE);
              SER(rFirst.MergeIntervals
                (&sNewFaces,    // out: new faces
                 &sNewEdges,    // out: new edges
                 &sNewVertices, // out: new vertices
                 pFace));       // in : original face
              sChanges.StopTracking();
              if(sNewFaces.GetSize() != 0 || sNewEdges.GetSize() != 1 || sNewVertices.GetSize() != 1
                 || pBrep->GetNumFaces() != 1 || !sChanges.IsInRmList(pFace) || !sChanges.IsInAddList(pFace))
                { return SM_ERR; }
              SmTArray<SmLoop*> sAfterFirst;
              pFace->GetLoops(sAfterFirst);
              if(sAfterFirst.GetSize() != sOriginalLoops.GetSize()+2) { return SM_ERR; }
              SmTArray<SmEdgeuse*> sSeamUses;
              sNewEdges[0]->GetEdgeuses(sSeamUses);
              SmLoop *pSide = NULL;
              ULONG lNumSides = 0;
              for(ULONG lIndex=0; lIndex<sSeamUses.GetSize(); ++lIndex)
                if(sSeamUses[lIndex]->GetLoopuse()->GetFaceuse() == pFace->GetUpwardFaceuse())
                  {
                    SmLoop *pThisSide = sSeamUses[lIndex]->GetLoopuse()->GetLoop();
                    if(pThisSide == pSide) { return SM_ERR; }
                    pSide = pThisSide;
                    ++lNumSides;
                  }
              if(lNumSides != 2) { return SM_ERR; }
              rSecond[0].m_vStart.SetClassObject(SM_PC_VERTEX,sNewVertices[0]);
              rSecond[0].m_vEnd.SetClassObject(SM_PC_VERTEX,sNewVertices[0]);
              SER(rSecond.MergeIntervals
                (&sNewFaces,    // out: new faces
                 &sNewEdges,    // out: new edges
                 &sNewVertices, // out: new vertices
                 pFace));       // in : original face
              if(sNewFaces.GetSize() != 0 || sNewEdges.GetSize() != 1 || sNewVertices.GetSize() != 0)
                { return SM_ERR; }
            }
          else if(lCase == 12)
            {
              // Exercise SplitAtSeam's output accumulation for both insertions.
              SmTArray<SmFace*> sNewFaces;
              SmTArray<SmEdge*> sNewEdges;
              SmTArray<SmVertex*> sNewVertices;
              SER(pFace->SplitAtSeam
                (sU,              // i/o: U seam classification
                 sV,              // i/o: V seam classification
                 eCrossed,        // in : illegally crossed seams
                 eMissing,        // in : missing seams
                 eNear,           // in : boundaries near seams
                 &sNewFaces,      // out: new faces
                 &sNewEdges,      // out: new seam edges
                 &sNewVertices)); // out: new seam vertices
              if(sNewFaces.GetSize() || sNewEdges.GetSize() != 2 || sNewVertices.GetSize() != 1) { return SM_ERR; }
            }
          else
            {
              SmHealData sHeal;
              SER(sHeal.HealBrep
                (pBrep,     // in : brep to heal
                 SM_HO_ALL, // in : last healing stage to run
                 NULL,      // in : all brep vertices
                 NULL,      // in : all brep edges
                 NULL,      // in : all brep faces
                 FALSE));   // in : create UV trim curves
            }
          SmTArray<SmLoop*> sLoops;
          pFace->GetLoops(sLoops);
          if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != lNumEdges+2
             || pBrep->GetNumVertices() != lNumVertices+1 || sLoops.GetSize() != sOriginalLoops.GetSize()+1)
            { return SM_ERR; }
          for(ULONG lIndex=0; lIndex<sOriginalLoops.GetSize(); ++lIndex)
            if(sOriginalLoops[lIndex]->IsOuterLoop()) { return SM_ERR; }
          for(ULONG lIndex=0; lIndex<sLoops.GetSize(); ++lIndex)
            {
              SmOrientType eActual = SM_OT_UNKNOWN;
              SER(pFace->ComputeLoopOrientation
                (sLoops[lIndex], // in : target loop
                 eActual,        // out: measured orientation
                 NULL,           // out: optional UV bounds
                 TRUE));         // in : use defining 3D curves
              if(eActual != (lIndex == 0 ? SM_OT_SAME : SM_OT_OPPOSITE)
                 || sLoops[lIndex]->GetLoopuse()->GetOrientation() != eActual) { return SM_ERR; }
            }
          sUses.ReSet();
          sLoops[0]->GetLoopuse()->GetEdgeuses(sUses);
          if(sUses.GetSize() != 4) { return SM_ERR; }
          for(ULONG lIndex=0; lIndex<4; ++lIndex)
            {
              ULONG lNumSameEdge = 0;
              for(ULONG lNext=0; lNext<4; ++lNext)
                if(sUses[lIndex]->GetEdge() == sUses[lNext]->GetEdge())
                  {
                    ++lNumSameEdge;
                    if(lNext != lIndex && sUses[lIndex]->GetOrientation() == sUses[lNext]->GetOrientation()) { return SM_ERR; }
                  }
              if(lNumSameEdge != 2) { return SM_ERR; }
            }
          // Regeneration must agree with the defining topology. Repeated healing
          // must not create another face, seam, or seam-intersection vertex.
          SER(pFace->RemoveUVTrimCurves());
          SER(SmTrimmingTools::CreateUVTrimCurves(pFace));
          SmHealData sAgain;
          SER(sAgain.HealBrep
            (pBrep,     // in : brep to heal
             SM_HO_ALL, // in : last healing stage to run
             NULL,      // in : all brep vertices
             NULL,      // in : all brep edges
             NULL,      // in : all brep faces
             TRUE));    // in : create UV trim curves
          if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != lNumEdges+2
             || pBrep->GetNumVertices() != lNumVertices+1) { return SM_ERR; }
          SmAssertArray sPointers,sValid;
          if(pBrep->ValidatePointers(&sPointers,SM_LEVEL_2,NULL) != SM_SUCCESS || sPointers.GetSize()
             || !pBrep->AssertValid(&sValid,SM_LEVEL_2,SM_WALK) || sValid.GetSize()) { sPointers.Dump(); sValid.Dump(); return SM_ERR; }
          pFace->HasSeamProblem
            (sU,       // out: U seam classification
             sV,       // out: V seam classification
             eCrossed, // out: illegally crossed seams
             eMissing, // out: missing seams
             eNear);   // out: boundaries near seams
          if(eCrossed != SM_SP_NEITHER || eMissing != SM_SP_NEITHER || eNear != SM_SP_NEITHER) { return SM_ERR; }
        }
    }
  MYPRINTF(_T("\nNonseparating torus seams: five cases passed"));
  return SM_SUCCESS;
}

static SmStatus my_test_torus_seams_with_vertex_loops()
{
  MYPRINTF(_T("\n********** Entered: my_test_torus_seams_with_vertex_loops"));
  // Exercise manual insertion in both orders, SplitAtSeam, and HealBrep.
  // Cover isolated points, vertices on either or both seams, and an existing
  // corner. All existing vertices must survive.
  for(ULONG lCase=0; lCase<30; ++lCase)
    {
      SmContext sContext;
      SmObjDelete sCleanup; // Destroy seam classifications before their supporting brep.
      SmTorusVertexLoopTestCase sCase;
      SER(sm_MakeTorusVertexLoopCase(sContext,lCase,sCase));
      SmBrep *pBrep = sCase.m_pBrep;
      SmFace *pFace = sCase.m_pFace;
      sCleanup.SetObj(pBrep);
      SmTemporaryChangeValue<SmBoolean> sDoingBool(sContext.GetDoingBooleanRef(),TRUE);
      TCHAR sLabel[SM_TBLOCK_SIZE];
      smos_sprintf(sLabel,_T("\nTorus vertex-loop case %lu"),lCase);
      MYPRINTF(sLabel);
      ULONG lMode = lCase/10;
      ULONG lNumPoints = sCase.m_lNumPoints;
      SmVertex **pVertices = sCase.m_pVertices;
      ULONG lExpectedTotalEdges = sCase.m_lExpectedEdges;
      ULONG lExpectedVertices = sCase.m_lExpectedVertices;
      if(lMode < 2)
        {
          // Prepare independent inputs; topology checks direct insertion separately.
          SER(sm_InsertTorusVertexLoopSeams(sContext,lCase,sCase));
        }
      else
        {
          SmHealData sInitialHeal;
          SER(sInitialHeal.HealBrep
            (pBrep,     // in : brep to heal
             SM_HO_ALL, // in : last healing stage to run
             NULL,      // in : all brep vertices
             NULL,      // in : all brep edges
             NULL,      // in : all brep faces
             TRUE));    // in : create UV trim curves
        }
      if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != lExpectedTotalEdges
         || pBrep->GetNumVertices() != lExpectedVertices)
        { SER_MSG(SM_ERR,_T("Torus seam repair split the face or duplicated a seam vertex")); }
      // Completed seams must remain stable through another healing pass.
      SER(pFace->RemoveUVTrimCurves());
      SER(SmTrimmingTools::CreateUVTrimCurves(pFace));
      ULONG      lNumEdges = pBrep->GetNumEdges(), lNumVertices = pBrep->GetNumVertices();
      SmHealData sHeal;
      SER(sHeal.HealBrep
        (pBrep,     // in : brep to heal
         SM_HO_ALL, // in : last healing stage to run
         NULL,      // in : all brep vertices
         NULL,      // in : all brep edges
         NULL,      // in : all brep faces
         TRUE));    // in : create UV trim curves
      if(pBrep->GetNumFaces() != 1 || pBrep->GetNumEdges() != lNumEdges || pBrep->GetNumVertices() != lNumVertices)
        { return SM_ERR; }
      SmTArray<SmVertex*> sVertices;
      pBrep->GetVertices(sVertices);
      for(ULONG lPoint=0; lPoint<lNumPoints; ++lPoint)
        {
          if(!sVertices.IsIn(pVertices[lPoint])) { return SM_ERR; }
          SmTArray<SmEdge*> sEdges;
          pVertices[lPoint]->GetEdges(sEdges);
          ULONG lExpectedVertexEdges = lPoint == 0 ? 0 : 2;
          if(sEdges.GetSize() != lExpectedVertexEdges) { return SM_ERR; }
        }
      SmTArray<SmLoop*> sLoops;
      pFace->GetLoops(sLoops);
      if(sLoops.GetSize() != 3 || sLoops[0]->GetLoopuse()->IsVertexLoopuse()) { return SM_ERR; }
      SER(pFace->RemoveUVTrimCurves());
      SER(SmTrimmingTools::CreateUVTrimCurves(pFace));
      SmAssertArray sPointers,sValid;
      if(pBrep->ValidatePointers(&sPointers,SM_LEVEL_2,NULL) != SM_SUCCESS || sPointers.GetSize()
         || !pBrep->AssertValid(&sValid,SM_LEVEL_2,SM_WALK) || sValid.GetSize())
        { sPointers.Dump(); sValid.Dump(); return SM_ERR; }
    }
  MYPRINTF(_T("\nTorus seams with vertex loops: thirty cases passed"));
  return SM_SUCCESS;
}

SmStatus my_test_healing()
{
  // Run every independent test and retain the first failure in either build mode.
  SmStatus sResult = SM_SUCCESS;

  MYPRINTF(_T("\n********** Entered: my_test_healing"));

  PT_RUN(sResult, my_test_heal_infinite_region_with_plausible_void_flags());

  // Seam completion, degenerate boundaries, and stability through another pass.
  PT_RUN(sResult, my_test_missing_seams_from_loop_orientation());
  PT_RUN(sResult, my_test_torus_seams_with_vertex_loops());
  PT_RUN(sResult, my_test_missing_torus_seam_at_coincidence());
  PT_RUN(sResult, my_test_heal_single_vertex_sphere());
  PT_RUN(sResult, my_test_heal_vertex_only_closed_surfaces());
  PT_RUN(sResult, my_test_closed_face_containment_repeated());

  // Surface-pole detection, edge splitting, and continuation after a failed repair.
  PT_RUN(sResult, my_test_healer_splits_boundary_at_pole());
  PT_RUN(sResult, my_test_healer_continues_after_pole_split_failure());
  PT_RUN(sResult, my_test_pole_crossing_endpoint_tolerances());

  // Cached properties must describe the surviving topology after repairs.
  PT_RUN(sResult, my_test_closed_sphere_properties());
  PT_RUN(sResult, my_test_move_seam_refreshes_face_props());
  PT_RUN(sResult, my_test_missing_poles_refreshes_all_faces());
  return sResult;
}
