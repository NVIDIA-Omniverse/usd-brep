// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smtopo_test.cpp
* PURPOSE ---  Test topology functions.
*
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <SmTypes.h>
#include <smTopoTest.h>
#include <SmTopologyIntersector.h>
#include <SmTrimmingTools.h>
#include <SmShape.h>
#include <SmCrvOnSurf.h>
#include <SmBrepCutting.h>
#include <SmAssertArray.h>
#include <SmEdgeProps.h>
#include <SmFace.h>
#include <SmFaceProps.h>
#include <SmOwningTopology.h>
#include <SmOffsetCurve.h>
#include <SmOffsetSurface.h>
#include <SmSurfOfExtrusion.h>
#include <SmHealData.h>
#include <SmPrimitiveCreation.h>
#include <SmCylinder.h>
#include <SmTrackTopologyChanges.h>

#define SM_NEW_READER 1

class SmBrepValidationTestAccess
{
public:
  static SmTopology * SetEdgeuseOwner(SmEdgeuse * pEdgeuse, SmTopology * pOwner)
    {
      SmTopology * pOldOwner = pEdgeuse->m_pSorLU ;
      pEdgeuse->m_pSorLU = pOwner ;
      return pOldOwner ;
    }
} ;

static SmBoolean sm_HasAssertReport(const SmAssertArray & crAsserts, ULONG lTestIndex, const void * pOwner)
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

static SmBoolean sm_HasAssertReport(const SmAssertArray & crAsserts, ULONG lListIndex, ULONG lTestIndex, const void * pOwner)
{
  for(ULONG ii=0; ii<crAsserts.GetSize(); ii++)
    {
      if(   crAsserts[ii] != NULL
         && crAsserts[ii]->GetListIndex() == lListIndex
         && crAsserts[ii]->GetTestIndex() == lTestIndex
         && crAsserts[ii]->GetOwner() == pOwner)
        { return TRUE ; }
    }
  return FALSE ;
}

static SmStatus my_test_validate_required_pointers()
{
  SmContext sContext ;
  SmBrep * pEmpty = new(sContext) SmBrep() ;
  SmAssertArray sEmptyAsserts ;
  if(   pEmpty->ValidatePointers(&sEmptyAsserts, SM_LEVEL_0, NULL) != SM_SUCCESS
     || sEmptyAsserts.GetSize() != 0)
    {
      delete pEmpty ;
      return SM_ERR ;
    }
  delete pEmpty ;

  SmBrep * pWireBrep = new(sContext) SmBrep() ;
  SmShell * pWireShell = NULL ;
  SmVertex * pWireVertex = NULL ;
  if(pWireBrep->MakeShellVertex(pWireBrep->GetInfiniteRegion(), SmPoint3d(0,0,0), pWireShell, pWireVertex) != SM_SUCCESS)
    {
      delete pWireBrep ;
      return SM_ERR ;
    }
  SmAssertArray sWireAsserts ;
  if(   pWireBrep->ValidatePointers(&sWireAsserts, SM_LEVEL_0, NULL) != SM_SUCCESS
     || sWireAsserts.GetSize() != 0)
    {
      delete pWireBrep ;
      return SM_ERR ;
    }
  delete pWireBrep ;

  SmBrep * pBrep = new(sContext) SmBrep() ;
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion()) ;
  if(sPrimitive.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement()) != SM_SUCCESS)
    {
      delete pBrep ;
      return SM_ERR ;
    }

  SmAssertArray sValidAsserts ;
  if(   pBrep->ValidatePointers(&sValidAsserts, SM_LEVEL_2, NULL) != SM_SUCCESS
     || sValidAsserts.GetSize() != 0)
    {
      delete pBrep ;
      return SM_ERR ;
    }

  SmTArray<SmEdge*> sEdges ;
  pBrep->GetEdges(sEdges) ;
  SmEdgeuse * pEdgeuse = sEdges[0]->GetPrimaryEdgeuse() ;

  SmStatus eRtn = SM_SUCCESS ;
  {
    SmOwningTopology * pOldOwner = pEdgeuse->GetOwner() ;
    pEdgeuse->SetListOwner(NULL) ;

    SmAssertArray sNullOwnerAsserts ;
    if(   pBrep->ValidatePointers(&sNullOwnerAsserts, SM_LEVEL_0, NULL) != SM_ERR
       || !sm_HasAssertReport(sNullOwnerAsserts, SM_LIST_1, 135, pEdgeuse))
      { eRtn = SM_ERR ; }

    pEdgeuse->SetListOwner(pOldOwner) ;
  }

  {
    SmTArray<SmFace*> sFaces ;
    pBrep->GetFaces(sFaces) ;
    SmFace * pFace = sFaces[0] ;
    SmFaceuse * pFaceuse1 = NULL ;
    SmFaceuse * pFaceuse2 = NULL ;
    pFace->GetFaceuses(pFaceuse1, pFaceuse2) ;
    pFace->SetFaceuse(NULL) ;

    SmAssertArray sNullFaceuseAsserts ;
    if(   pBrep->ValidatePointers(&sNullFaceuseAsserts, SM_LEVEL_0, NULL) != SM_ERR
       || !sm_HasAssertReport(sNullFaceuseAsserts, SM_LIST_1, 122, pFace))
      { eRtn = SM_ERR ; }

    pFace->SetFaceuse(pFaceuse1) ;
  }

  {
    SmTArray<SmVertex*> sVertices ;
    pBrep->GetVertices(sVertices) ;
    SmVertex * pVertex = sVertices[0] ;
    SmOwningTopology * pOldEdgeuseOwner = pEdgeuse->GetOwner() ;
    SmOwningTopology * pOldVertexOwner = pVertex->GetOwner() ;
    pEdgeuse->SetListOwner(NULL) ;
    pVertex->SetListOwner(NULL) ;

    SmAssertArray sFailFastAsserts ;
    if(   pBrep->ValidatePointers(&sFailFastAsserts, SM_LEVEL_0, NULL) != SM_ERR
       || !sm_HasAssertReport(sFailFastAsserts, SM_LIST_1, 135, pEdgeuse)
       || sm_HasAssertReport(sFailFastAsserts, SM_LIST_1, 142, pVertex))
      { eRtn = SM_ERR ; }

    pVertex->SetListOwner(pOldVertexOwner) ;
    pEdgeuse->SetListOwner(pOldEdgeuseOwner) ;
  }

  delete pBrep ;
  return eRtn ;
}

static SmStatus my_test_stale_edgeuse_owner()
{
  SmContext sContext ;
  SmBrep * pBrep = new(sContext) SmBrep() ;
  SmBrep * pOtherBrep = new(sContext) SmBrep() ;
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion()) ;
  SmPrimitiveCreation sOtherPrimitive(pOtherBrep->GetInfiniteRegion()) ;
  SER(sPrimitive.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement())) ;
  SER(sOtherPrimitive.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement())) ;

  const SmAssertTestLevel sLevels[2] = { SM_LEVEL_0, SM_LEVEL_2 } ;
  SmAssertArray sValidAsserts ;
  if(   !pBrep->AssertValid(&sValidAsserts, SM_LEVEL_2, SM_WALK)
     || sValidAsserts.GetSize() != 0)
    {
      delete pBrep ;
      delete pOtherBrep ;
      return SM_ERR ;
    }

  SmTArray<SmEdge*> sEdges ;
  SmTArray<SmEdge*> sOtherEdges ;
  pBrep->GetEdges(sEdges) ;
  pOtherBrep->GetEdges(sOtherEdges) ;
  SmEdgeuse * pEdgeuse = sEdges[0]->GetPrimaryEdgeuse() ;
  SmLoopuse * pStaleOwner = sOtherEdges[0]->GetPrimaryEdgeuse()->GetLoopuse() ;
  SmTopology * pOldOwner = SmBrepValidationTestAccess::SetEdgeuseOwner(pEdgeuse, pStaleOwner) ;

  SmStatus eRtn = SM_SUCCESS ;
  for(ULONG ii=0; ii<2; ii++)
    {
      SmAssertArray sPointerAsserts ;
      if(   pBrep->ValidatePointers(&sPointerAsserts, sLevels[ii], NULL) != SM_ERR
         || !sm_HasAssertReport(sPointerAsserts, 98, pEdgeuse))
        { eRtn = SM_ERR ; }

      SmAssertArray sWalkAsserts ;
      if(   pBrep->AssertValid(&sWalkAsserts, sLevels[ii], SM_WALK)
         || !sm_HasAssertReport(sWalkAsserts, 98, pEdgeuse))
        { eRtn = SM_ERR ; }
    }

  SmBrepValidationTestAccess::SetEdgeuseOwner(pEdgeuse, pOldOwner) ;
  delete pBrep ;
  delete pOtherBrep ;
  return eRtn ;
}

static SmStatus my_test_squeeze_edge_tracks_removal()
{
  SmContext sContext ;
  SmBrep * pBrep = new(sContext) SmBrep() ;
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion()) ;
  SER(sPrimitive.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement())) ;

  SmTArray<SmEdge*> sEdges ;
  pBrep->GetEdges(sEdges) ;
  SmEdge * pRemovedEdge = sEdges[0] ;
  SmVertex * pSurvivingVertex = pRemovedEdge->GetVertex() ;

  SmStatus eRtn = SM_SUCCESS ;
  {
    SmHealData sHealData ;
    SmEdgeProps * pRemovedProps = new SmEdgeProps(&sContext) ;
    pRemovedProps->m_pEdge = pRemovedEdge ;
    sHealData.m_pBrep = pBrep ;
    sHealData.m_sTgtEdges.Add(pRemovedEdge) ;
    sHealData.m_sTgtEdgeProps.Add(pRemovedProps) ;

    SmTrackTopologyChanges sChanges(pBrep, TRUE) ;
    eRtn = pBrep->SqueezeEdge(pRemovedEdge, pSurvivingVertex) ;
    sChanges.StopTracking() ;

    ULONG lIndex ;
    if(   eRtn != SM_SUCCESS
       || !sChanges.GetRmList()->FindElement(pRemovedEdge, lIndex)
       || pBrep->IsLiveTopologyMember(pRemovedEdge)
       || sHealData.UpdateTargetLists(sChanges, SM_HO_FIX_COIN_VERTICES) != SM_SUCCESS
       || sHealData.m_sTgtEdges.GetSize() != 0
       || sHealData.m_sTgtEdgeProps.GetSize() != 0)
      { eRtn = SM_ERR ; }
  }

  delete pBrep ;
  return eRtn ;
}

static SmStatus my_test_tracks_changed_objects()
{
  SmContext sContext ;
  SmBrep * pBrep = new(sContext) SmBrep() ;
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion()) ;
  SER(sPrimitive.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement())) ;

  SmTArray<SmVertex*> sVertices ;
  SmTArray<SmFace*> sFaces ;
  SmTArray<SmLoop*> sLoops ;
  pBrep->GetVertices(sVertices) ;
  pBrep->GetFaces(sFaces) ;
  sFaces[0]->GetLoops(sLoops) ;

  SmStatus eRtn = SM_SUCCESS ;
  {
    SmTrackTopologyChanges sChanges(pBrep, TRUE) ;
    sVertices[0]->SetPoint(sVertices[0]->GetPoint()) ;
    sChanges.StopTracking() ;

    if(   sChanges.GetRmList()->GetSize() != 0
       || sChanges.GetAddList()->GetSize() != 0
       || sChanges.GetChgList()->GetSize() != 1
       || !sChanges.IsInChgList(sVertices[0]))
      { eRtn = SM_ERR ; }

    sChanges.StartTracking() ;
    SmSurface * pSurface = sFaces[0]->GetSurface() ;
    pSurface->Notify(SM_NO_POST_EDIT, pSurface, SM_NO_GET_OWNER(pSurface), NULL) ;
    if(!sChanges.IsInChgList(sFaces[0]))
      { eRtn = SM_ERR ; }

    if(   sLoops[0]->FlipLoopOrientation() != SM_SUCCESS
       || sLoops[0]->FlipLoopOrientation() != SM_SUCCESS)
      { eRtn = SM_ERR ; }
    sChanges.StopTracking() ;

    if(   sChanges.GetRmList()->GetSize() != 1
       || sChanges.GetAddList()->GetSize() != 1
       || sChanges.GetChgList()->GetSize() != 1
       || !sChanges.IsInRmList(sFaces[0])
       || !sChanges.IsInAddList(sFaces[0])
       || !sChanges.IsInChgList(sFaces[0])
       || !pBrep->AssertValid())
      { eRtn = SM_ERR ; }

    sChanges.StartTracking() ;
    if(sFaces[0]->RemoveLoop(sLoops[0]) != SM_SUCCESS)
      { eRtn = SM_ERR ; }
    if(   sLoops[0]->GetLoopuse()->GetFaceuse() != NULL
       || sLoops[0]->FlipLoopOrientation() != SM_SUCCESS
       || sLoops[0]->FlipLoopOrientation() != SM_SUCCESS)
      { eRtn = SM_ERR ; }
    if(sFaces[0]->InsertLoop(sLoops[0]) != SM_SUCCESS)
      { eRtn = SM_ERR ; }
    sChanges.StopTracking() ;
    if(!pBrep->AssertValid())
      { eRtn = SM_ERR ; }
  }

  delete pBrep ;
  return eRtn ;
}

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

static SmStatus my_test_tracks_nested_geometry_and_resets()
{
  SmContext sContext ;
  SmBrep * pBrep = new(sContext) SmBrep() ;
  SmObjDelete sCleanBrep(pBrep) ;
  SmPrimitiveCreation sPrimitive(pBrep->GetInfiniteRegion()) ;
  SER(sPrimitive.CreateRectangle(1.0, 1.0, SmAxis2Placement())) ;
  SmTArray<SmFace*> sFaces ;
  SmTArray<SmEdge*> sEdges ;
  pBrep->GetFaces(sFaces) ;
  pBrep->GetEdges(sEdges) ;
  SmFace * pFace = sFaces[0] ;
  SmEdge * pEdge = sEdges[0] ;

  SmCurve * pBaseCurve = NULL ;
  SER(pEdge->GetCurve()->Copy(sContext, pBaseCurve)) ;
  SmOffsetCurve * pInnerCurve = new(sContext) SmOffsetCurve(3, *pBaseCurve,
      pBaseCurve->GetNaturalInterval(), SmVector3d(0,0,1), 0.0, TRUE) ;
  SmOffsetCurve * pOuterCurve = new(sContext) SmOffsetCurve(3, *pInnerCurve,
      pInnerCurve->GetNaturalInterval(), SmVector3d(0,0,1), 0.0, TRUE) ;
  pOuterCurve->SetOwner(pEdge) ;
  pEdge->SetCurve(pOuterCurve, TRUE) ;

  SmBSplineCurve * pGenerator = NULL ;
  SER(SmBSplineCurve::CreateLineSegment(sContext, 3, SmPoint3d(0,0,0), SmPoint3d(1,0,0), pGenerator)) ;
  SmSurfOfExtrusion * pBaseSurface = NULL ;
  SER(SmSurfOfExtrusion::CreateCanonical(sContext, pGenerator, SmVector3d(0,1,0), pBaseSurface)) ;
  SER(pBaseSurface->AdjustSTEPUVDomain(SmExtent2d(0,0,1,1))) ;
  SmOffsetSurface * pInnerSurface = new(sContext) SmOffsetSurface(0.0, *pBaseSurface, TRUE) ;
  SmOffsetSurface * pOuterSurface = new(sContext) SmOffsetSurface(0.0, *pInnerSurface, TRUE) ;
  pFace->SetSurface(pOuterSurface, TRUE) ;

  SmTrackTopologyChanges sChanges(pBrep, TRUE) ;
  pBaseCurve->Notify(SM_NO_POST_EDIT, pBaseCurve, SM_NO_GET_OWNER(pBaseCurve), NULL) ;
  pBaseSurface->Notify(SM_NO_POST_EDIT, pBaseSurface, SM_NO_GET_OWNER(pBaseSurface), NULL) ;
  if(   pBaseCurve->GetEdge() != pEdge
     || pBaseSurface->GetFace() != pFace
     || !sChanges.IsInChgList(pEdge)
     || !sChanges.IsInChgList(pFace))
    { return SM_ERR ; }
#ifdef SM_DEBUG_CODE
  SmTopologyChangeCallback * pCallback = (SmTopologyChangeCallback *)sContext.GetSysNotifyCallback() ;
  if(pCallback->m_pChgListIndx->GetSize() != sChanges.GetChgList()->GetSize())
    { return SM_ERR ; }
#endif

  sChanges.ReSetChgList() ;
  if(sChanges.GetChgList()->GetSize() != 0)
    { return SM_ERR ; }
#ifdef SM_DEBUG_CODE
  if(pCallback->m_pChgListIndx->GetSize() != 0)
    { return SM_ERR ; }
#endif
  // A generator curve reaches its face through both curve and surface ownership.
  pGenerator->Notify(SM_NO_POST_EDIT, pGenerator, SM_NO_GET_OWNER(pGenerator), NULL) ;
  if(sChanges.GetChgList()->GetSize() != 1 || !sChanges.IsInChgList(pFace))
    { return SM_ERR ; }

  sChanges.StopTracking() ;
  sChanges.StartTracking() ;
  if(   sChanges.GetChgList()->GetSize() != 0
     || sChanges.GetRmList()->GetSize() != 0
     || sChanges.GetAddList()->GetSize() != 0
     || sChanges.GetNotifyEvents()->GetSize() != 0)
    { return SM_ERR ; }
#ifdef SM_DEBUG_CODE
  if(   pCallback->m_pRmListIndx->GetSize() != 0
     || pCallback->m_pAddListIndx->GetSize() != 0
     || pCallback->m_pChgListIndx->GetSize() != 0)
    { return SM_ERR ; }
#endif
  pBaseSurface->Notify(SM_NO_POST_EDIT, pBaseSurface, SM_NO_GET_OWNER(pBaseSurface), NULL) ;
  SmTemporaryChangeValue<SmBoolean> sEditing(pBrep->m_bEditingEnabled, TRUE) ;
  SER(pBrep->DeleteFace(pFace, TRUE)) ;
  sChanges.StopTracking() ;
  if(   !sChanges.IsInRmList(pFace)
     || sChanges.IsInChgList(pFace)
     || sChanges.IsInAddList(pFace)
     || pBrep->IsLiveTopologyMember(pFace))
    { return SM_ERR ; }
#ifdef SM_DEBUG_CODE
  if(   pCallback->m_pRmListIndx->GetSize() != sChanges.GetRmList()->GetSize()
     || pCallback->m_pAddListIndx->GetSize() != sChanges.GetAddList()->GetSize()
     || pCallback->m_pChgListIndx->GetSize() != sChanges.GetChgList()->GetSize())
    { return SM_ERR ; }
#endif
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


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_tsurf_cyl_creation_fast
  (const SmContext & crContext,
   SmBrep *& rpNewBrep,
   SmBoolean bDoFast)
{
    // Allocate a new brep object to hold the trimmed surface.
    SmBrep *pBrep = new (crContext) SmBrep();
    pBrep->SetTolerance(0.0001);

    // Create the surface for the face - a 10x10 plane
    SmBSplineSurface *pCyl = NULL ;
    SmAxis2Placement sRefFrame;
    SER(SmBSplineSurface::CreateConePatch(crContext,sRefFrame,10.0,
        5.0,0.0,180.0,10,SM_CO_QUADRATIC,pCyl));

    SmExtent2d sNewDomain(SmPoint2d(0,0),SmPoint2d(10,10));

    pCyl->Reparameterize(sNewDomain);

    // Declare arrays to hold curves and curve orientations
    SmTArray<SmBSplineCurve*> sUVCurves;
    SmTArray<SmOrientType> sCurveOrients;

    // Now create the outer loop - it is a square with a knotch in
    // the upper right hand corner.  The loop orientation needs to be
    // counter clockwise.

    SmBSplineCurve * pCurve = NULL ;
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(0,0,0),SmPoint3d(1,0,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(1,.5,0),SmPoint3d(1,0,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // curve orientation opposite to loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(1,.5,0),SmPoint3d(.5,.5,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.5,.5,0),SmPoint3d(.5,1,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.5,1,0),SmPoint3d(0,1,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(0,1,0),SmPoint3d(0,0,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    // Now create first inner loop - make it a clockwise list of 3
    // lines
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.1,.5,0),SmPoint3d(.1,.8,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.1,.8,0),SmPoint3d(.4,.5,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.4,.5,0),SmPoint3d(.1,.5,0),pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    // Now Create second inner loop which is a closed curve (circle)
    // Please note that the creation of the circle will be counter clockwise
    // therefore we have to reverse it to display it.
    sRefFrame.SetCanonical(SmPoint3d(.6,.25,0),SmVector3d(-1,0,0),SmVector3d(0,-1,0));
    SER(SmBSplineCurve::CreateCircleSegment(crContext,2,sRefFrame,.15,0.0,360.0,
        SM_CO_QUADRATIC,pCurve));
    sUVCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // Curve orientation opposite of loop

    // Now set up loops - count of the curve number in each loop
    SmTArray<ULONG> sLoops;
    sLoops.Add(6);  // 6 curves in outer loop
    sLoops.Add(3);  // 3 curves in first inner loop
    sLoops.Add(1);  // 1 curve in last inner loop


    // Now make the face in the infinite region of the brep.
    SmTArray<SmPoint3d> sLoopPoints;
    SmRegion *pNewRegion;
    SmShell *pNewShell;
    SmFace *pNewFace;

if ( bDoFast ) {
    SER(pBrep->MakeFaceWithCurvesFast(
        pBrep->GetInfiniteRegion(),   // Get the infinite region of the Brep
        sLoops,  // Loops
        sUVCurves, // UV curves
        sCurveOrients, // Orientation for each curve
        sLoopPoints,   // No single loop vertex points
        pCyl,        // Surface
        pCyl->GetNaturalUVDomain(), // Natural domain of surface
        SM_OT_SAME,    // Outer loop is input counterclockwise relative
                       // to the top of the surface.  
        pNewRegion, // Will be returned NULL,
        pNewShell,  // New shell created for the face
        pNewFace)); // New Face created with the three loops
    SmObjsDelete<SmBSplineCurve*> sCleanUV( &sUVCurves );

} else {

    SER(pBrep->MakeFaceWithCurves(
        pBrep->GetInfiniteRegion(),   // Get the infinite region of the Brep
        sLoops,  // Loops
        NULL,       // 3D curves
        &sUVCurves, // UV curves
        sCurveOrients, // Orientation for each curve
        sLoopPoints,   // No single loop vertex points
        pCyl,        // Surface
        pCyl->GetNaturalUVDomain(), // Natural domain of surface
        SM_OT_SAME,    // Outer loop is input counterclockwise relative
                       // to the top of the surface.  
        pNewRegion, // Will be returned NULL,
        pNewShell,  // New shell created for the face
        pNewFace)); // New Face created with the three loops
}

    rpNewBrep = pBrep;    

    return SM_SUCCESS;

} // end my_test_tsurf_cyl_creation_fast

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_tsurf_cyl_creation_2d
  (const SmContext & crContext,
   SmBrep *& rpNewBrep)
{
    // Allocate a new brep object to hold the trimmed surface.
    SmBrep *pBrep = new (crContext) SmBrep();
    pBrep->SetTolerance(0.0001);

    // Create the surface for the face - a 10x10 plane
    SmBSplineSurface *pCyl = NULL ;
    SmAxis2Placement sRefFrame;
    SER(SmBSplineSurface::CreateConePatch(crContext,sRefFrame,10.0,
        5.0,0.0,180.0,10,SM_CO_QUADRATIC,pCyl));

    SmExtent2d sNewDomain(SmPoint2d(0,0),SmPoint2d(10,10));

    pCyl->Reparameterize(sNewDomain);

    // Declare arrays to hold curves and curve orientations
    SmTArray<SmBSplineCurve*> s3DCurves;
    SmTArray<SmOrientType> sCurveOrients;

    // Now create the outer loop - it is a square with a knotch in
    // the upper right hand corner.  The loop orientation needs to be
    // counter clockwise.

    SmBSplineCurve * pCurve = NULL ;
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(0,0,0),SmPoint3d(1,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(1,.5,0),SmPoint3d(1,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // curve orientation opposite to loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(1,.5,0),SmPoint3d(.5,.5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.5,.5,0),SmPoint3d(.5,1,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.5,1,0),SmPoint3d(0,1,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(0,1,0),SmPoint3d(0,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    // Now create first inner loop - make it a clockwise list of 3
    // lines
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.1,.5,0),SmPoint3d(.1,.8,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.1,.8,0),SmPoint3d(.4,.5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(.4,.5,0),SmPoint3d(.1,.5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    // Now Create second inner loop which is a closed curve (circle)
    // Please note that the creation of the circle will be counter clockwise
    // therefore we have to reverse it to display it.
    sRefFrame.SetCanonical(SmPoint3d(.6,.25,0),SmVector3d(-1,0,0),SmVector3d(0,-1,0));
    SER(SmBSplineCurve::CreateCircleSegment(crContext,2,sRefFrame,.15,0.0,360.0,
        SM_CO_QUADRATIC,pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // Curve orientation opposite of loop

    // Now set up loops - count of the curve number in each loop
    SmTArray<ULONG> sLoops;
    sLoops.Add(6);  // 6 curves in outer loop
    sLoops.Add(3);  // 3 curves in first inner loop
    sLoops.Add(1);  // 1 curve in last inner loop


    // Now make the face in the infinite region of the brep.
    SmTArray<SmPoint3d> sLoopPoints;
    SmRegion *pNewRegion;
    SmShell *pNewShell;
    SmFace *pNewFace;

    SER(pBrep->MakeFaceWithCurves(
        pBrep->GetInfiniteRegion(),   // Get the infinite region of the Brep
        sLoops,  // Loops
        NULL,
        &s3DCurves, // 3D curves
        sCurveOrients, // Orientation for each curve
        sLoopPoints,   // No single loop vertex points
        pCyl,        // Surface
        pCyl->GetNaturalUVDomain(), // Natural domain of surface
        SM_OT_SAME,    // Outer loop is input counterclockwise relative
                       // to the top of the surface.  
        pNewRegion, // Will be returned NULL,
        pNewShell,  // New shell created for the face
        pNewFace)); // New Face created with the three loops

    rpNewBrep = pBrep;    

    return SM_SUCCESS;

} // end my_test_tsurf_cyl_creation_2d

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_tsurf_cyl_creation_3d
  (const SmContext & crContext,
   SmBrep *& rpNewBrep)
{
    // Allocate a new brep object to hold the trimmed surface.
    SmBrep *pBrep = new (crContext) SmBrep();
    pBrep->SetTolerance(0.0001);

    // Create the surface for the face - a 10x10 plane
    SmBSplineSurface *pCyl = NULL ;
    SmAxis2Placement sRefFrame;
    SER(SmBSplineSurface::CreateConePatch(crContext,sRefFrame,10.0,
        5.0,0.0,180.0,10,SM_CO_QUADRATIC,pCyl));

    SmExtent2d sNewDomain(SmPoint2d(0,0),SmPoint2d(10,10));

    pCyl->Reparameterize(sNewDomain);

    // Declare arrays to hold curves and curve orientations
    SmTArray<SmCurve*> s3DCurves;
    SmTArray<SmOrientType> sCurveOrients;

    // Now create the outer loop - it is a square with a knotch in
    // the upper right hand corner.  The loop orientation needs to be
    // counter clockwise.

    SmBSplineCurve * pCurve = NULL ;
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(0,0,0),SmPoint3d(10,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(10,5,0),SmPoint3d(10,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // curve orientation opposite to loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(10,5,0),SmPoint3d(5,5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(5,5,0),SmPoint3d(5,10,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(5,10,0),SmPoint3d(0,10,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(0,10,0),SmPoint3d(0,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    // Now create first inner loop - make it a clockwise list of 3
    // lines
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(1,5,0),SmPoint3d(1,8,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(1,8,0),SmPoint3d(4,5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        2,SmPoint3d(4,5,0),SmPoint3d(1,5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    // Now Create second inner loop which is a closed curve (circle)
    // Please note that the creation of the circle will be counter clockwise
    // therefore we have to reverse it to display it.
    sRefFrame.SetCanonical(SmPoint3d(6,2.5,0),SmVector3d(-1,0,0),SmVector3d(0,-1,0));
    SER(SmBSplineCurve::CreateCircleSegment(crContext,2,sRefFrame,1.5,0.0,360.0,
        SM_CO_QUADRATIC,pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // Curve orientation opposite of loop

    // Now set up loops - count of the curve number in each loop
    SmTArray<ULONG> sLoops;
    sLoops.Add(6);  // 6 curves in outer loop
    sLoops.Add(3);  // 3 curves in first inner loop
    sLoops.Add(1);  // 1 curve in last inner loop


    // Now make the face in the infinite region of the brep.
    SmTArray<SmPoint3d> sLoopPoints;
    SmRegion *pNewRegion;
    SmShell *pNewShell;
    SmFace *pNewFace;

    SER(pBrep->MakeFaceWithCurves(
        pBrep->GetInfiniteRegion(),   // Get the infinite region of the Brep
        sLoops,  // Loops
        &s3DCurves, // 3D curves
        NULL,
        sCurveOrients, // Orientation for each curve
        sLoopPoints,   // No single loop vertex points
        pCyl,        // Surface
        pCyl->GetNaturalUVDomain(), // Natural domain of surface
        SM_OT_SAME,    // Outer loop is input counterclockwise relative
                       // to the top of the surface.  
        pNewRegion, // Will be returned NULL,
        pNewShell,  // New shell created for the face
        pNewFace)); // New Face created with the three loops

    rpNewBrep = pBrep;    

    return SM_SUCCESS;

} // end my_test_tsurf_cyl_creation_3d





/***********************************************************************
PURPOSE --- Create analytic trimmed surfaces and do intersecting.

USAGE NOTES ---
 
***********************************************************************/
SmStatus my_test_analytic_tsurf_creation
  (const SmContext & crContext)
{
    // Allocate a new brep object to hold the trimmed surface.
    SmBrep *pBrep1 = new (crContext) SmBrep();
    SmObjDelete sClean1( pBrep1 );
    pBrep1->SetTolerance(0.00001);

    // Create the surface for the face #1 - a 10x10 plane
    SmPoint3d sOrigin(0.0,0.0,0.0);
    SmVector3d sX(1.0,0.0,0.0);
    SmVector3d sY(0.0,1.0,0.0);
    SmAxis2Placement sPos;
    sPos.SetCanonical(sOrigin,sX,sY);
    SmPlane * pNewPlane = NULL;
    SER(SmPlane::CreateCanonical(crContext,sPos,pNewPlane));
    SmExtent2d sUVDomain(SmPoint2d(-10.0,-10.0),SmPoint2d(10.0,10.0));
    SER(pNewPlane->AdjustSTEPUVDomain(sUVDomain));

    // Declare arrays to hold curves and curve orientations
    SmTArray<SmCurve*> s3DCurves;
    SmTArray<SmOrientType> sCurveOrients;

    SmPoint3d sLinePoint;
    SmVector3d sLineVector;
    SmLine * pLine = NULL;
    sLinePoint = SmPoint3d(-10.0,-10.0,0.0);
    sLineVector = SmVector3d(1.0,0.0,0.0);
    SER(SmLine::CreateCanonical(crContext,sLinePoint,sLineVector,pLine));
    SER(pLine->AdjustSTEPInterval(SmExtent1d(0.0,20.0)));
    s3DCurves.Add(pLine);
    sCurveOrients.Add(SM_OT_SAME);

    sLinePoint = SmPoint3d(10.0,-10.0,0.0);
    sLineVector = SmVector3d(0.0,1.0,0.0);
    SER(SmLine::CreateCanonical(crContext,sLinePoint,sLineVector,pLine));
    SER(pLine->AdjustSTEPInterval(SmExtent1d(0.0,20.0)));
    s3DCurves.Add(pLine);
    sCurveOrients.Add(SM_OT_SAME);

    sLinePoint = SmPoint3d(10.0,10.0,0.0);
    sLineVector = SmVector3d(-1.0,0.0,0.0);
    SER(SmLine::CreateCanonical(crContext,sLinePoint,sLineVector,pLine));
    SER(pLine->AdjustSTEPInterval(SmExtent1d(0.0,20.0)));
    s3DCurves.Add(pLine);
    sCurveOrients.Add(SM_OT_SAME);

    sLinePoint = SmPoint3d(-10.0,10.0,0.0);
    sLineVector = SmVector3d(0.0,-1.0,0.0);
    SER(SmLine::CreateCanonical(crContext,sLinePoint,sLineVector,pLine));
    SER(pLine->AdjustSTEPInterval(SmExtent1d(0.0,20.0)));
    s3DCurves.Add(pLine);
    sCurveOrients.Add(SM_OT_SAME);

    // Now make the face in the infinite region of the brep.
    SmTArray<ULONG> sLoops;
    sLoops.Add(4);  // 4 curves in outer loop
    SmTArray<SmPoint3d> sLoopPoints;
    SmRegion *pNewRegion;
    SmShell *pNewShell;
    SmFace *pNewFace;

    SER(pBrep1->MakeFaceWithCurves(pBrep1->GetInfiniteRegion(),
        sLoops,&s3DCurves,NULL,sCurveOrients,sLoopPoints,pNewPlane,
        pNewPlane->GetNaturalUVDomain(),SM_OT_SAME,pNewRegion,pNewShell,
        pNewFace)); // New Face created with the loop       

    // Allocate a new brep object to hold the trimmed surface.
    s3DCurves.ReSet();
    sCurveOrients.ReSet();

    SmBrep *pBrep2 = new (crContext) SmBrep();
    SmObjDelete sClean2( pBrep2 );
    pBrep2->SetTolerance(SM_ZONE_TOL_3D);

    // Create the surface for the face #2 - a cone
    sOrigin = SmPoint3d(0.0,0.0,0.0);
    sX = SmVector3d(1.0,0.0,0.0);
    sY = SmVector3d(0.0,1.0,0.0);
    sPos.SetCanonical(sOrigin,sX,sY);
    SmCylinder * pNewCylinder = NULL;
    SER(SmCylinder::CreateCanonical(crContext,sPos,2.0,pNewCylinder));
    sUVDomain = SmExtent2d(SmPoint2d(0.0,-1.0),SmPoint2d(360.0,1.0));
    SER(pNewCylinder->AdjustSTEPUVDomain(sUVDomain));

    sLinePoint = SmPoint3d(2.0,0.0,1.0);
    sLineVector = SmVector3d(0.0,0.0,-1.0);
    SER(SmLine::CreateCanonical(crContext,sLinePoint,sLineVector,pLine));
    SER(pLine->AdjustSTEPInterval(SmExtent1d(0.0,2.0)));
    s3DCurves.Add(pLine);
    sCurveOrients.Add(SM_OT_SAME);

    sOrigin = SmPoint3d(0.0,0.0,-1.0);
    sX = SmVector3d(1.0,0.0,0.0);
    sY = SmVector3d(0.0,1.0,0.0);
    sPos.SetCanonical(sOrigin,sX,sY);
    SmCircle * pCircle = NULL;
    SER(SmCircle::CreateCanonical(crContext,sPos,2.0,pCircle));
    SER(pCircle->AdjustSTEPInterval(SmExtent1d(0.0,90.0)));
    s3DCurves.Add(pCircle);
    sCurveOrients.Add(SM_OT_SAME);

    sLinePoint = SmPoint3d(0.0,2.0,-1.0);
    sLineVector = SmVector3d(0.0,0.0,1.0);
    SER(SmLine::CreateCanonical(crContext,sLinePoint,sLineVector,pLine));
    SER(pLine->AdjustSTEPInterval(SmExtent1d(0.0,2.0)));
    s3DCurves.Add(pLine);
    sCurveOrients.Add(SM_OT_SAME);

    sOrigin = SmPoint3d(0.0,0.0,1.0);
    sX = SmVector3d(0.0,1.0,0.0);
    sY = SmVector3d(1.0,0.0,0.0);
    sPos.SetCanonical(sOrigin,sX,sY);
    SER(SmCircle::CreateCanonical(crContext,sPos,2.0,pCircle));
    SER(pCircle->AdjustSTEPInterval(SmExtent1d(0.0,90.0)));
    s3DCurves.Add(pCircle);
    sCurveOrients.Add(SM_OT_SAME);

    SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, &crContext)->GetDoingBooleanRef(), TRUE );

    SER(pBrep2->MakeFaceWithCurves(pBrep1->GetInfiniteRegion(),
        sLoops,&s3DCurves,NULL,sCurveOrients,sLoopPoints,pNewCylinder,
        pNewCylinder->GetNaturalUVDomain(),SM_OT_SAME,pNewRegion,pNewShell,
        pNewFace)); // New Face created with the loop

    SmTopologyIntersector sTI(crContext,pBrep1,pBrep2,
        smos_Max(pBrep1->GetTolerance(),pBrep2->GetTolerance()),
        20.0*SM_PI/180.0);

    SER(sTI.IntersectInsertRelate());


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetColor(0,1,0);
        pBrep1->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        pBrep2->Draw();
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;

} // end my_test_analytic_tsurf_creation


    
 /***********************************************************************
 PURPOSE --- Create a trimmed surface and corresponding brep from a
 surface (Plane) and curves.  This trimmed surface will have two
 inner loops, one triangle and one circle.
    
  __________________
  |                |
  |                |
  |                |
  |   |\           |
  |   |  \         |
  |   |    \       |
  |   |     \      |
  |   |       \    |
  |   |_________\  |_________________
  |                                  |
  |                                  |
  |              /---\               |
  |             /     \              |
  |            |       |             |
  |            |       |             |
  |             \     /              |
  |               \__/               |
  |                                  |
  |__________________________________|

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_tsurf_creation
  (const SmContext & crContext,
   SmBrep *& rpNewBrep)
{
  // Allocate a new brep object to hold the trimmed surface.
  SmBrep *pBrep = new (crContext) SmBrep();
  pBrep->SetTolerance(0.0001);

  // Create the surface for the face - a 10x10 plane
  SmBSplineSurface *pPlane = NULL ;
  SER(SmBSplineSurface::CreateBilinearSurface(crContext,
      SmPoint3d(0,0,0),SmPoint3d(10,0,0),SmPoint3d(0,10,0),SmPoint3d(10,10,0),
      pPlane));

  SmSurface *pSurface;
  SER(pPlane->CopyAndAddAnalytics(crContext,pSurface));
  SM_ASSERT(pPlane != NULL) ; delete pPlane; pPlane = NULL ;
  pPlane = SM_CAST_PTR(SmBSplineSurface,pSurface);

  // Declare arrays to hold curves and curve orientations
  SmTArray<SmCurve*> s3DCurves;
  SmTArray<SmOrientType> sCurveOrients;

  // Now create the outer loop - it is a square with a knotch in
  // the upper right hand corner.  The loop orientation needs to be
  // counter clockwise.

  SmBSplineCurve * pCurve = NULL ;
  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(0,0,0),SmPoint3d(10,0,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(10,5,0),SmPoint3d(10,0,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_OPPOSITE);  // curve orientation opposite to loop

  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(10,5,0),SmPoint3d(5,5,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(5,5,0),SmPoint3d(5,10,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(5,10,0),SmPoint3d(0,10,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
  
  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(0,10,0),SmPoint3d(0,0,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
  
  // Now create first inner loop - make it a clockwise list of 3
  // lines
  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(1,5,0),SmPoint3d(1,8,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(1,8,0),SmPoint3d(4,5,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

  SER(SmBSplineCurve::CreateLineSegment(crContext,
      3,SmPoint3d(4,5,0),SmPoint3d(1,5,0),pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

  // Now Create second inner loop which is a closed curve (circle)
  // Please note that the creation of the circle will be counter clockwise
  // therefore we have to reverse it to display it.
  SmAxis2Placement sRefFrame;
  sRefFrame.SetCanonical(SmPoint3d(6,2.5,0),SmVector3d(-1,0,0),SmVector3d(0,-1,0));
  SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,1.5,0.0,360.0,
      SM_CO_QUADRATIC,pCurve));
  s3DCurves.Add(pCurve); pCurve = NULL ;
  sCurveOrients.Add(SM_OT_OPPOSITE);  // Curve orientation opposite of loop

  // Now set up loops - count of the curve number in each loop
  SmTArray<ULONG> sLoops;
  sLoops.Add(6);  // 6 curves in outer loop
  sLoops.Add(3);  // 3 curves in first inner loop
  sLoops.Add(1);  // 1 curve in last inner loop


  // Now make the face in the infinite region of the brep.
  SmTArray<SmPoint3d> sLoopPoints;
  SmRegion *pNewRegion;
  SmShell *pNewShell;
  SmFace *pNewFace;

  SER(pBrep->MakeFaceWithCurves
        (pBrep->GetInfiniteRegion(),   // Get the infinite region of the Brep
         sLoops,                       // Loops
         &s3DCurves,                   // 3D curves
         NULL,                         // No UV Curves
         sCurveOrients,                // Orientation for each curve
         sLoopPoints,                  // No single loop vertex points
         pPlane,                       // Surface
         pPlane->GetNaturalUVDomain(), // Natural domain of surface
         SM_OT_SAME,                   // Outer loop is input counterclockwise relative
                                       // to the top of the surface.  
         pNewRegion,                   // Will be returned NULL,
         pNewShell,                    // New shell created for the face
         pNewFace));                   // New Face created with the three loops


  rpNewBrep = pBrep;    

  return SM_SUCCESS;

} // end my_test_tsurf_creation


/***********************************************************************
 PURPOSE --- Create a trimmed surface and corresponding brep from a
 surface (Plane) and curves.  This trimmed surface will have two
 inner loops, one triangle and one circle.
    
  __________________
  |                |
  |                |
  |                |
  |   |\           |
  |   |  \         |
  |   |    \       |
  |   |     \      |
  |   |       \    |
  |   |_________\  |_________________
  |                                  |
  |                                  |
  |              /---\               |
  |             /     \              |
  |            |       |             |
  |            |       |             |
  |             \     /              |
  |               \__/               |
  |                                  |
  |__________________________________|

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_trimming_tools
  (const SmContext & crContext,
   SmBrep *& rpNewBrep)
{
    if (TRUE) {
    // Allocate a new brep object to hold the trimmed surface.
    SmBrep *pBrep = new (crContext) SmBrep();
    pBrep->SetTolerance(0.0001);

    // Create the surface for the face - a 10x10 plane
    SmBSplineSurface *pPlane = NULL ;
    SER(SmBSplineSurface::CreateBilinearSurface(crContext,
        SmPoint3d(0,0,0),SmPoint3d(10,0,0),SmPoint3d(0,10,0),SmPoint3d(10,10,0),
        pPlane));

    // Declare arrays to hold curves and curve orientations
    SmTArray<SmCurve*> s3DCurves;
    SmTArray<SmOrientType> sCurveOrients;

    // Now create the outer loop - it is a square with a knotch in
    // the upper right hand corner.  The loop orientation needs to be
    // counter clockwise.

    SmBSplineCurve * pCurve = NULL ;
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(0,0,0),SmPoint3d(10,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(10,5,0),SmPoint3d(10,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // curve orientation opposite to loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(10,5,0),SmPoint3d(5,5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(5,5,0),SmPoint3d(5,10,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(5,10,0),SmPoint3d(0,10,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(0,10,0),SmPoint3d(0,0,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop
    
    // Now create first inner loop - make it a clockwise list of 3
    // lines
    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(1,5,0),SmPoint3d(1,8,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(1,8,0),SmPoint3d(4,5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    SER(SmBSplineCurve::CreateLineSegment(crContext,
        3,SmPoint3d(4,5,0),SmPoint3d(1,5,0),pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_SAME);  // curve orientation same as loop

    // Now Create second inner loop which is a closed curve (circle)
    // Please note that the creation of the circle will be counter clockwise
    // therefore we have to reverse it to display it.
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical(SmPoint3d(6,2.5,0),SmVector3d(-1,0,0),SmVector3d(0,-1,0));
    SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,1.5,0.0,360.0,
        SM_CO_QUADRATIC,pCurve));
    s3DCurves.Add(pCurve);
    sCurveOrients.Add(SM_OT_OPPOSITE);  // Curve orientation opposite of loop

    // Now set up loops - count of the curve number in each loop
    SmTArray<ULONG> sLoops;
    sLoops.Add(6);  // 6 curves in outer loop
    sLoops.Add(3);  // 3 curves in first inner loop
    sLoops.Add(1);  // 1 curve in last inner loop


    // Now make the face in the infinite region of the brep.
    SmTArray<SmPoint3d> sLoopPoints;

    SER(SmTrimmingTools::TrimSurfaceWithModelSpaceCurves(
        crContext,pBrep->GetInfiniteRegion(),
        0.00001,
        sLoops,  // Loops
        s3DCurves, // 3D curves
        sCurveOrients, // Orientation for each curve
        sLoopPoints,   // No single loop vertex points
        pPlane,        // Surface
        pPlane->GetNaturalUVDomain(), // Natural domain of surface
        SM_OT_SAME,    // Outer loop is input counterclockwise relative
                       // to the top of the surface.  
        FALSE, TRUE, TRUE)); 


    // rpNewBrep = pBrep;   
    
    SM_ASSERT_VALID( pBrep );
    delete pBrep; pBrep = NULL;
    }

    if (TRUE) {
        // Allocate a new brep object to hold the trimmed surface.
        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance(0.0001);
        
        // Create the surface for the face - a 10x10 plane
        SmBSplineSurface *pPlane = NULL ;
        SER(SmBSplineSurface::CreateBilinearSurface(crContext,
            SmPoint3d(-10,-10,0),SmPoint3d(10,-10,0),SmPoint3d(-10,10,0),SmPoint3d(10,10,0),
            pPlane));
        
        // Declare arrays to hold curves and curve orientations
        SmTArray<SmCurve*> s3DCurves;
        SmTArray<SmOrientType> sCurveOrients;
        
        // Now create the outer loop - it is a square with a knotch in
        // the upper right hand corner.  The loop orientation needs to be
        // counter clockwise.
        
        // Now Create second inner loop which is a closed curve (circle)
        // Please note that the creation of the circle will be counter clockwise
        // therefore we have to reverse it to display it.
        SmBSplineCurve * pCurve = NULL ;
        SmAxis2Placement sRefFrame;
        sRefFrame.SetCanonical(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,5.0,0.0,360.0,
            SM_CO_QUADRATIC,pCurve));
        s3DCurves.Add(pCurve);
        sCurveOrients.Add(SM_OT_SAME);  // Outer Loop circle
        
        SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,3.0,0.0,360.0,
            SM_CO_QUADRATIC,pCurve));
        s3DCurves.Add(pCurve);
        sCurveOrients.Add(SM_OT_OPPOSITE);  // Inner Loop circle
        
        // Now set up loops - count of the curve number in each loop
        SmTArray<ULONG> sLoops;
        sLoops.Add(1);  // 1 curves in outer loop
        sLoops.Add(1);  // 1 curve in last inner loop
        
        
        // Now make the face in the infinite region of the brep.
        SmTArray<SmPoint3d> sLoopPoints;
        
        SER(SmTrimmingTools::TrimSurfaceWithModelSpaceCurves(
            crContext,pBrep->GetInfiniteRegion(),
            0.00001,
            sLoops,  // Loops
            s3DCurves, // 3D curves
            sCurveOrients, // Orientation for each curve
            sLoopPoints,   // No single loop vertex points
            pPlane,        // Surface
            pPlane->GetNaturalUVDomain(), // Natural domain of surface
            SM_OT_SAME,    // Outer loop is input counterclockwise relative
            // to the top of the surface.  
            FALSE, TRUE, TRUE)); 
        
        
        rpNewBrep = pBrep;   

        SM_ASSERT_VALID( pBrep );
    }
    return SM_SUCCESS;

} // end my_test_trimming_tools

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_tsurf_insert_edge()
{

    {
        SmContext crContext;
        SmBrep *pBrep;
        SER( my_test_tsurf_creation( crContext, pBrep ) );
        pBrep->m_bEditingEnabled = TRUE;
        SmObjDelete sCleanup( pBrep );
        SmPlaneCutter sCutter( SmPoint3d( 5.1, 5.1, 0.0 ),
                               SmVector3d( -1, -1, 0 ) );

        SmBrepCutting sCutting( pBrep );

        SER( sCutting.DoCut( &sCutter, SM_ZONE_TOL_3D, TRUE ) );
        pBrep->m_bEditingEnabled = FALSE;
#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            SmTArray<SmFace*> sFaces;
            pBrep->GetFaces( sFaces );
            for ( ULONG j = 0; j < sFaces.GetSize(); j++ )
            {
                sFaces[j]->DrawUV( 4 + j, 8 + j );
            }
        }
#endif

        SmPlaneCutter sCutter2( SmPoint3d( 6.5, 6.5, 0.0 ),
                                SmVector3d( 1, 1, 0 ) );

        pBrep->m_bEditingEnabled = TRUE;
        SER( sCutting.DoCut( &sCutter2, SM_ZONE_TOL_3D, TRUE ) );

        pBrep->m_bEditingEnabled = FALSE;
#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            SmTArray<SmFace*> sFaces;
            pBrep->GetFaces( sFaces );
            for ( ULONG i = 0; i < sFaces.GetSize(); i++ )
            {
                sFaces[i]->DrawUV( 4 + i, 8 + i );
            }
        }
#endif // SM_GFX_CODE
   }

   {
        SmContext crContext;
        SmTemporaryChangeValue< SmBoolean > sDoingBool( (&crContext)->GetDoingBooleanRef(), TRUE );
        SmBrep *pBrep;
        SER(my_test_tsurf_creation(crContext,pBrep));
        SmObjDelete sCleanup(pBrep);
// Remove Composites
//        pBrep->SetMakeComposites(TRUE);
        pBrep->m_bEditingEnabled = TRUE;
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        SmFace *pFace = sFaces[0];
        SmSurface *pSurface = pFace->GetSurface();

        SmBSplineCurve *pLine = NULL ;
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            SmPoint3d(0.0,5.5,0),SmPoint3d(10,5.5,0),
            pLine));
        SmTArray<SmCurve*> s3DCurves;
        SmObjsDelete<SmCurve*> sObjsClean(&s3DCurves);
        SmTArray<SmFace*> sNewFaces;
        s3DCurves.Add(pLine);

        SmTArray<SmEdge*> sEdges;
        SER(pBrep->MergeCurvesOnSurface(*pSurface,  // in : Surface must be owned by a face in this Brep.
                                         SM_ZONE_TOL_3D,    // in : 3D Tol for curves greater than MaxCrvSrfGap and MaxCrvTrimCrvGap.
                                         s3DCurves, // in : 3D Curves on surface not necessarily trimmed to face boundary.
                                                    //      Curves should not cross each other.
                                         NULL,      // in : associated UV Curves having a dim of 2.
                                         sNewFaces, // out: Newly created faces, if any.
                                         sEdges));  // out: Brep Edges for cr3DCurves (new and/or existing) 
                                                    //      ordered:[CurveParameritization]
                                                    // in : TRUE = use projection for intersections to allow for tolerances.
                                                    //      FALSE= no projections for faces already toleranced. default:[FALSE]
                                                    // in : OtherSurface when cr3DCurves are from surf/surf XSects. 
                                                    //      As curves are merged into Brep - surf/surf/surf XSects are
                                                    //      used for new verts and to improve Edge->Curve interpolation.
                                                    //      NULL to ignore, default:[NULL], NULL creates larger gaps.
                                                    // in : TRUE = try UVSpace Classification if 3Space try is dodgey (tolerant cases)
                                                    //      FALSE= don't try UVSpace because UVTrimCurves aren't yet valid, default:[TRUE]
        
        pBrep->m_bEditingEnabled = FALSE;
#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            for ( ULONG i = 0; i < sNewFaces.GetSize(); i++ )
            {
                sNewFaces[i]->DrawUV( 30, 10 );
            }
            pFace->DrawUV( 11, 33 );
            //        sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }
        return SM_SUCCESS;

#if 0 // unreachable code
   { // Test copy and transform of brep
        SmContext crContext;
        SmBrep *pBrep;
        SER(my_test_tsurf_creation(crContext,pBrep));
        SmObjDelete sCleanup(pBrep);
        SmBrep *pCopy = new (crContext) SmBrep(*pBrep);
        SmObjDelete sClenaup2(pCopy);
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(1.0,0.5,1.0));
        SmVector3d sScale(2.0,1.0,0.5);
        SER(pCopy->Transform(sA2P,&sScale));
#ifdef SM_GFX_CODE
//        smgfx_SetColor(0,0,0);
#endif
        SmBrep *pBrep2;
        SER(my_test_tsurf_creation(crContext,pBrep2));
        SmObjDelete sCleanup3(pBrep2);
        SER(pBrep->MergeBrep(*pBrep2));
#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            smgfx_SetColor( 1, 0, 0 );
            pCopy->Draw();
        }
#endif
 //       sm_GraphicsLoop();
   }


   {
        SmContext crContext;
        SmBrep *pBrep;
        SER(my_test_tsurf_creation(crContext,pBrep));
        SmObjDelete sCleanup(pBrep);
        SmTArray<SmVertex*> sVertices;
        pBrep->GetVertices(sVertices);
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        SmFace *pFace = sFaces[0];

        SmLoop *pNewLoop;
        SmVertex *pNewVertex;
        SER(pBrep->MakeVertexLoop(pFace,SmPoint3d(3.0,4.0,0.0),
            pNewLoop,pNewVertex));


        SmBSplineCurve *pLine = NULL ;
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            sVertices[8]->GetPoint(),pNewVertex->GetPoint(),
            pLine));
        SmEdge *pNewEdge;
        SmFace *pNewFace;
        SER(pBrep->MakeEdgeInFace(pFace,sVertices[8],pNewVertex,
            pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D, 
            pNewEdge, pNewLoop, pNewFace));

        
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            sVertices[6]->GetPoint(),pNewVertex->GetPoint(),
            pLine));
        SER(pBrep->MakeEdgeInFace(pFace,sVertices[6],pNewVertex,
            pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D,
            pNewEdge, pNewLoop, pNewFace));

#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            if ( pNewFace ) pNewFace->DrawUV( 30, 10 );
            pFace->DrawUV( 10, 30 );
            //     sm_GraphicsLoop();
        }
#endif
    }

    {
        SmContext crContext;
        SmBrep *pBrep;
        SER(my_test_tsurf_creation(crContext,pBrep));
        SmObjDelete sCleanup(pBrep);
        SmTArray<SmVertex*> sVertices;
        pBrep->GetVertices(sVertices);
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        SmFace *pFace = sFaces[0];

        SmLoop *pNewLoop;
        SmVertex *pNewVertex;
        SER(pBrep->MakeVertexLoop(pFace,SmPoint3d(3.0,4.0,0.0),
            pNewLoop,pNewVertex));


        SmBSplineCurve *pLine = NULL ;
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            sVertices[8]->GetPoint(),pNewVertex->GetPoint(),
            pLine));
        SmEdge *pNewEdge;
        SmFace *pNewFace;
        SER(pBrep->MakeEdgeInFace(pFace,sVertices[8],pNewVertex,
            pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D,
            pNewEdge, pNewLoop, pNewFace));

        
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            pNewVertex->GetPoint(),sVertices[6]->GetPoint(),
            pLine));
        SER(pBrep->MakeEdgeInFace(pFace,pNewVertex,sVertices[6],
            pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D,
            pNewEdge, pNewLoop, pNewFace));

#ifdef SM_GFX_CODE 
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            if ( pNewFace ) pNewFace->DrawUV( 30, 10 );
            pFace->DrawUV( 10, 30 );
            //     sm_GraphicsLoop();
        }
#endif
    }

    {
        SmContext crContext;
        SmBrep *pBrep;
        SER(my_test_tsurf_creation(crContext,pBrep));
        SmObjDelete sCleanup(pBrep);
        SmTArray<SmVertex*> sVertices;
        pBrep->GetVertices(sVertices);
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        SmFace *pFace = sFaces[0];
        SmBSplineCurve *pLine = NULL ;
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            sVertices[5]->GetPoint(),sVertices[6]->GetPoint(),
            pLine));
        SmEdge *pNewEdge;
        SmLoop *pNewLoop;
        SmFace *pNewFace;
        SER(pBrep->MakeEdgeInFace(pFace,sVertices[5],sVertices[6],
            pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D,
            pNewEdge, pNewLoop, pNewFace));
#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            if ( pNewFace ) pNewFace->DrawUV( 30, 10 );
            pFace->DrawUV( 10, 30 );
            //     sm_GraphicsLoop();
        }
#endif
    }

    {
        SmContext crContext;
        SmBrep *pBrep;
        SER(my_test_tsurf_creation(crContext,pBrep));
        SmObjDelete sCleanup(pBrep);
        SmTArray<SmVertex*> sVertices;
        pBrep->GetVertices(sVertices);
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        SmFace *pFace = sFaces[0];
        SmBSplineCurve *pLine = NULL ;
        SER(SmBSplineCurve::CreateLineSegment(crContext,3,
            sVertices[0]->GetPoint(),sVertices[3]->GetPoint(),
            pLine));
        SmEdge *pNewEdge;
        SmLoop *pNewLoop;
        SmFace *pNewFace;
        SER(pBrep->MakeEdgeInFace(pFace,sVertices[0],sVertices[3],
            pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D,
            pNewEdge, pNewLoop, pNewFace));
#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            pBrep->Draw();
            if ( pNewFace ) pNewFace->DrawUV( 30, 10 );
            pFace->DrawUV( 10, 30 );
            //      sm_GraphicsLoop();
        }
#endif
    }

    
    return SM_SUCCESS;
#endif // unreachable code

} // end my_test_tsurf_insert_edge



/***********************************************************************
PURPOSE --- This function tests planar section of the trimmed surfaces
    in a brep.  It computes a bunch of planes that span the box of the 
    brep.  It sections the brep with each plane and then draws the 3D 
    curves produced by the sectioning operation.

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_tsurf_section
  (const SmBrep * cpBrep,
   SmApproxTol3d dApproxTol,
   const SmVector3d & crPlaneNormal,
   ULONG lNumPlanes,
   ULONG & rlNumFound)
{
//    SmVector3d sXAx(-1.0,0,0),sYAx(0,1,0);
//    SmAxis2Placement sMirror;
//    SmPoint3d sMirrPnt(2280.8164281678269000,-1280.3485428928038000,4203.0064058561475000);
//    sMirror.SetCanonical(sMirrPnt,sXAx,sYAx);
//    SmBrep *pBrep = (SmBrep*)cpBrep;
//    pBrep->Mirror(sMirror);
//    pBrep->ValidatePointers();
//    return SM_SUCCESS;
    
    // Statistical stuff
    rlNumFound = 0;

    SmContext  sContext ;
    const SmContext *pContext = cpBrep->GetContext() ? cpBrep->GetContext() : &sContext ;
    SmExtent3d sBBox;
    SER(cpBrep->CalculateBoundingBox(sBBox)); // Get box of brep

    for (ULONG i=0; i<lNumPlanes; i++) {
        double dParam = (i*1.0) / (lNumPlanes-1.0);
        // Walk along the bounding box axis and get points for the plane
        SmPoint3d sPlanePoint = sBBox.Evaluate(dParam,dParam,dParam);
        SmTArray<SmCurve*> s3DCurves;  // Holds 3D model space curves
        SmTArray<SmCurve*> sUVCurves;  // Holds 2D parameter space curves

        // Create the planar section curves from the brep and the plane
        SER(cpBrep->CreatePlanarSectionCurves(*pContext,
                                              sPlanePoint,
                                              crPlaneNormal,
                                              &dApproxTol,
                                              NULL,
                                              &s3DCurves,
                                              &sUVCurves,
                                              NULL));

        // Draw the resulting section curves
        for (ULONG j=0; j<s3DCurves.GetSize(); j++) {
            rlNumFound ++;
            SmCurve *p3DCurve = s3DCurves[j] ;
            // SmCurve *pUVCurve = sUVCurves[j] ;
            SM_ASSERT(p3DCurve != NULL) ;  // not all intersections produce UVTrimCurves
#ifdef SM_GFX_CODE
            if ( smGet_DoGraphics() )
            {
                smgfx_SetColor( 1, 0, 0 );
                smgfx_ChangeColor();
                //            p3DCurve->DrawWDeriv(p3DCurve->GetNaturalInterval(),0);
                smgfx_SetLineWidth( 2.0 );
                ( SM_CAST_PTR( SmBSplineCurve, p3DCurve ) )->DrawWithKnots();
                smgfx_SetLineWidth( 1.0 );
                if ( FALSE )
                {
                    sUVCurves[j]->DrawWDeriv( sUVCurves[j]->GetNaturalInterval(), 0 );
                }
            }
#endif
            // Delete the curves once we are done drawing them
            SM_ASSERT(s3DCurves[j] != NULL) ; delete s3DCurves[j]; s3DCurves[j] = NULL ;
            if(sUVCurves[j]        != NULL) { delete sUVCurves[j]; sUVCurves[j] = NULL ; }
        }
    }

    return SM_SUCCESS;

} // end my_test_tsurf_section

/***********************************************************************
PURPOSE --- This function tests silhouette coputation of the trimmed surfaces
    in a brep.  It computes the untrimmed section curves and then trims
    them.

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_tsurf_silhouette
 (const SmBrep     * cpBrep,
  SmApproxTol3d      dApproxTol,
  const SmVector3d & crEyePointOrViewVector,
  SmBoolean          bPerspective,
  ULONG            & rlNumFound)
{
    // Statistical stuff
    rlNumFound = 0;

    const SmContext *pContext = cpBrep->GetContext();
    SmTArray<SmFace*> sFaces;
    cpBrep->GetFaces(sFaces);

    // Perform silhouette computation for each face in the Brep
    for (ULONG i=0; i<sFaces.GetSize(); i++) {
        SmFace *pFace = sFaces[i];
        SmTArray<SmCurve*> s3DCurves;  // Holds 3D model space curves
        SmTArray<SmCurve*> sUVCurves;  // Holds 2D parameter space curves
        SmObjsDelete<SmCurve*> sClean1(&s3DCurves);
        SmObjsDelete<SmCurve*> sClean2(&sUVCurves);

        SmBSplineSurface *pSurface = SM_CAST_PTR(SmBSplineSurface,pFace->GetSurface());
        NER(pSurface);
        SmExtent2d sUVDomain = pFace->GetUVDomain();
  
        double dAngleTol = SM_DEG2RAD(20.0);

        SmTArray<SmCurve*> sTrim3DCurves;
        SmTArray<SmCurve*> sTrimUVCurves;
        SmObjsDelete<SmCurve*> sClean3(&sTrim3DCurves);
        SmObjsDelete<SmCurve*> sClean4(&sTrimUVCurves);

        SmBoolean bHaveUntrimmed = FALSE;
        SmBoolean bHaveTrimmed = FALSE;
        SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pSurface); NER(pSC);
        if (pSC->HasCachedCurves(*pContext,SM_AS_TRIMMED_SILHOUETTE,dApproxTol,
            dAngleTol,&crEyePointOrViewVector,
            &bPerspective, NULL, NULL, NULL,
            &sTrim3DCurves,&sTrimUVCurves)) {
            bHaveTrimmed = TRUE;
        }
        else if (pSC->HasCachedCurves(*pContext,SM_AS_SILHOUETTE,dApproxTol,
            dAngleTol,&crEyePointOrViewVector,
            &bPerspective, NULL, NULL, NULL,
            &s3DCurves,&sUVCurves)) {
            bHaveUntrimmed = TRUE;
        }

        if (!bHaveUntrimmed && !bHaveTrimmed) {
            // Create the planar section curves from the brep and the plane
            SER(pSurface->CreateSilhouetteCurves(*pContext,
                                                 sUVDomain,
                                                 crEyePointOrViewVector, 
                                                 bPerspective,
                                                 &dApproxTol,
                                                 &dAngleTol,
                                                 &s3DCurves,
                                                 &sUVCurves));
        }

static SmBoolean bDoTrim = TRUE;
        if (bDoTrim && !bHaveTrimmed) {
            SER(pFace->CreateCurvesByTrimming(*pContext, 
                                              &dApproxTol,
                                              &s3DCurves, 
                                              &sUVCurves, 
                                              &sTrim3DCurves, 
                                              &sTrimUVCurves));
            // Register trimmed curves and create copy by getting them again
            SER(pSC->AddCurvesToCache(SM_AS_TRIMMED_SILHOUETTE,
                                      dApproxTol,dAngleTol,&crEyePointOrViewVector,&bPerspective,
                                      NULL,NULL,NULL,&sTrim3DCurves,&sTrimUVCurves));
            if (!pSC->HasCachedCurves(*pContext,SM_AS_TRIMMED_SILHOUETTE,dApproxTol,
                dAngleTol,&crEyePointOrViewVector,
                &bPerspective, NULL, NULL, NULL,
                &sTrim3DCurves,&sTrimUVCurves)) {
                SER(SM_ERR);
            }
        }
        else if (!bHaveTrimmed) {
            SER(pSC->AddCurvesToCache(SM_AS_SILHOUETTE,
                dApproxTol,dAngleTol,&crEyePointOrViewVector,&bPerspective,
                NULL,NULL,NULL,&s3DCurves,&sUVCurves));
            if (!pSC->HasCachedCurves(*pContext,SM_AS_SILHOUETTE,dApproxTol,
                dAngleTol,&crEyePointOrViewVector,
                &bPerspective, NULL, NULL, NULL,
                &s3DCurves,&sUVCurves)) {
                SER(SM_ERR);
            }
            sClean1.Clear();
            sClean2.Clear();
            sTrim3DCurves.Append(s3DCurves);
            sTrimUVCurves.Append(sUVCurves);
        }

        // Draw the resulting section curves
        for (ULONG j=0; j<sTrim3DCurves.GetSize(); j++) {
            rlNumFound ++;
#ifdef SM_GFX_CODE
            SmCurve *pCurve = sTrim3DCurves[j];
            if ( smGet_DoGraphics() )
            {
                smgfx_SetColor( 1, 0, 0 );
                smgfx_SetLineWidth( 2.0 );
                smgfx_ChangeColor();
                //            pCurve->DrawWDeriv(pCurve->GetNaturalInterval(),0);
                ( SM_CAST_PTR( SmBSplineCurve, pCurve ) )->DrawWithKnots();
                smgfx_SetLineWidth( 1.0 );
                if ( FALSE )
                {
                    sUVCurves[j]->DrawWDeriv( sUVCurves[j]->GetNaturalInterval(), 0 );
                }
            }
#endif
        }
    }

    return SM_SUCCESS;

} // end my_test_tsurf_silhouette




/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_tsurf_projection
  (const SmBrep * cpBrep,
   SmApproxTol3d  dApproxTol,
   ULONG          lNumCurves,
   ULONG        & rlNumFound)
{
    rlNumFound = 0;

    SmExtent3d sBBox;

    SmContext sContext;
    const SmContext *pContext = cpBrep->GetContext() ? cpBrep->GetContext() : &sContext ;

    SER(cpBrep->CalculateBoundingBox(sBBox));
    SmPoint3d sCent = sBBox.Evaluate(0.5,0.5,1.0);
    SmVector3d sDir = sBBox.Evaluate(0.5,0.5,0.0) - sCent;
    SmVector3d sWidVec = sBBox.Evaluate(1,1,1) - sBBox.Evaluate(0,0,1);
    double dMaxRadius = smos_Max(sWidVec.x,sWidVec.y);

    for (ULONG i=1; i<=lNumCurves; i++) {
        double dCurrRad = (i*1.0)/(lNumCurves) * (dMaxRadius/2.1);
        SmBSplineCurve * pCurve = my_create_circle(*pContext,dCurrRad,sCent,SM_CO_QUADRATIC,0,0,1);
        NER(pCurve);
        SmObjDelete sCurveCleanup(pCurve);
        SmTArray<SmCurve*> s3DCurves;
        SmTArray<SmCurve*> sUVCurves;
        SER(cpBrep->CreateParallelProjectionCurves(*pContext,
                                                   *pCurve,
                                                   SmVector3d(0,0,-1),
                                                   NULL,
                                                   &dApproxTol,
                                                   NULL,
                                                   &s3DCurves,
                                                   &sUVCurves,
                                                   NULL));
        for (ULONG j=0; j<s3DCurves.GetSize(); j++) {
            rlNumFound ++;
#ifdef SM_GFX_CODE
            SmCurve *pCrv = s3DCurves[j];
            if ( smGet_DoGraphics() )
            {
                smgfx_SetColor( 1, 0, 0 );
                pCrv->DrawWDeriv( pCrv->GetNaturalInterval(), 0 );
                if ( FALSE )
                {
                    sUVCurves[j]->DrawWDeriv( sUVCurves[j]->GetNaturalInterval(), 0 );
                }
            }
#endif
            SM_ASSERT(s3DCurves[j] != NULL) ; delete s3DCurves[j]; s3DCurves[j] = NULL ;
            SM_ASSERT(sUVCurves[j] != NULL) ; delete sUVCurves[j]; sUVCurves[j] = NULL ;
        }
    }

    return SM_SUCCESS;

} // end my_test_tsurf_projection


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_ssi
  (const SmBrep * cpBrep1,
   const SmBrep * cpBrep2,
   SmApproxTol3d  dApproxTol,
   ULONG        & rlNumFound)
{

    ULONG ii, jj;

    SmContext  sContext;
    const SmContext *pContext = cpBrep1->GetContext() ? cpBrep1->GetContext() : &sContext ;
    rlNumFound = 0;
    SmTArray<SmCurve*> s3DCurves;
    SmObjsDelete<SmCurve*> sClean1(&s3DCurves);
    SmTArray<SmCurve*> sUVCurves1;
    SmObjsDelete<SmCurve*> sClean2(&sUVCurves1);
    SmTArray<SmCurve*> sUVCurves2;
    SmObjsDelete<SmCurve*> sClean3(&sUVCurves2);
    SmTArray<double> sDeviations;
    SmTArray<SmTsectCurveType> sCurveTypes;
    SmTArray<SmFace*> sFaces1;
    cpBrep1->GetFaces(sFaces1);
    SmTArray<SmFace*> sFaces2;
    cpBrep2->GetFaces(sFaces2);
    for (ii=0; ii<sFaces1.GetSize(); ii++) {
        SmFace *pF1 = sFaces1[ii];
        for (jj=0; jj<sFaces2.GetSize(); jj++) {
            SmFace *pF2 = sFaces2[jj];
            SmBoolean bUseSurfaceEdges[2];
            bUseSurfaceEdges[0] = TRUE;
            bUseSurfaceEdges[1] = TRUE;
            SER(pF1->GetSurface()->GlobalSurfaceIntersect(*pContext,pF1->GetUVDomain(),
                                                          *pF2->GetSurface(),pF2->GetUVDomain(),bUseSurfaceEdges,
                                                          &dApproxTol,
                                                          NULL,
                                                          &s3DCurves,
                                                          &sUVCurves1,
                                                          &sUVCurves2,
                                                          &sCurveTypes,
                                                          &sDeviations));
            rlNumFound += s3DCurves.GetSize();
#ifdef SM_GFX_CODE
            if (smGet_DoGraphics()) {
                if (FALSE) {
                    smgfx_Erase();
                }

                smgfx_SetLook(2, 4, 1, 0, 0);
                for (ULONG kk = 0; kk < s3DCurves.GetSize(); kk++) {
                    (SM_CAST_PTR(SmBSplineCurve, s3DCurves[kk]))->DrawWithKnots();
                    sm_GraphicsLoop();
                }
            }
#endif

        }
    }

    return SM_SUCCESS;

} // end my_test_ssi   


/***********************************************************************
PURPOSE --- This function computes a grid of points in the box of a brep
    and invokes the topology solver to solve the given operation between
    a point and the brep with trimmed surfaces.  The operation may be
    minimization, maximization, normalization or intersection. 

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_tsurface_point_extrema
  (const SmBrep *cpBrep,
   SmSolverOperationType eSolverOperation,
   long lNumX,
   long lNumY,
   long lNumZ,
   const SmVector3d * cpOptVectors,
   ULONG & rlNumFound)
{
    // Statistical stuff
    rlNumFound = 0;
    ULONG lCount=0;

    // The following is a benchmark test for Quality Control Systems
    // It is currently turned off.
    if (TRUE) {
        SmTArray<SmPoint3d> sPoints;
        sPoints.Add(SmPoint3d(  2811.4739500810206, 590.21281639457879, 145.44695932094584 ));

        SmSolution aSols[10];
        SmSolutionArray sSolutions(10,aSols);

        TCHAR sBuff[SM_TBLOCK_SIZE];
        for (ULONG i=0; i<10; i++) {
            for (ULONG j=0; j<sPoints.GetSize(); j++) {
                SmPoint3d sClPt = sPoints[j];
                double dBestAnswer = SM_BIG_DOUBLE;
                SER(SmTopologySolver::BrepPointSolve(cpBrep,sClPt,SM_SO_MINIMIZE,
                    SM_SR_ALL,SM_ZONE_TOL_3D,dBestAnswer,NULL,sSolutions));
                if (sSolutions.GetSize() == 0) SER(SM_ERR);
                // Draw the result of the solve
                for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) {
                    rlNumFound ++;
                    SmSolution sSolution = sSolutions[kk];
                    SmPoint3d sFndPnt;
                    sSolution.GetPoint( 0, sFndPnt );
                    SmVector3d sTo = sFndPnt - sClPt;
#ifdef SM_GFX_CODE
              if(smGet_DoGraphics())
              {
                smgfx_SetColor( 1, 0, 0 );
                sTo.Draw( &sClPt );
                sClPt.Draw();
              }
#endif
            } // For ea
            SmSolution & rSol = sSolutions[0];
            if(i == 0)
            {
              SM_SPRINTF( sBuff, _T( "Point = %lf, %lf, %lf, Distance = %lf\n" ),
                  sClPt.x, sClPt.y, sClPt.z, rSol.m_vStart.m_dSolutionValue );
              smos_WriteBuffer( sBuff );
            }

          }

        }
        return SM_SUCCESS;
    }


    // Compute the bounding box of the Brep
    SmExtent3d sBBox;
    SER(cpBrep->CalculateBoundingBox(sBBox));

    if (eSolverOperation == SM_SO_RAYFIRE) {
        lNumZ = 1;
    }

#ifdef SM_GFX_CODE
//    smgfx_Open(smgfx_GetRuleColor(this));
#endif

    // Traverse X, Y, and Z directions and create corresponding delta points
    for (long i=0; i<lNumX; i++) {
        double dX = (sBBox.GetMax().x - sBBox.GetMin().x) / (lNumX-1+SM_EFF_ZERO);
        if (dX < SM_EFF_ZERO) i=lNumX;
        if (lNumX < 2) dX = 0.0;
        for (long j=0; j<lNumY; j++) {
            double dY = (sBBox.GetMax().y - sBBox.GetMin().y) / (lNumY-1+SM_EFF_ZERO);
            if (dY < SM_EFF_ZERO) j=lNumY;
            if (lNumY < 2) dY = 0.0;
            for (long k=0; k<lNumZ; k++) {
                double dZ = (sBBox.GetMax().z - sBBox.GetMin().z) / (lNumZ-1+SM_EFF_ZERO);
                if (dZ < SM_EFF_ZERO) k=lNumZ;
                if (lNumZ < 2) dZ = 0.0;
                // Here is the point in the box we are going to use in solve
                SmPoint3d sClPt( sBBox.GetMin().x + i * dX,
                                 sBBox.GetMin().y + j * dY,
                                 sBBox.GetMin().z + k * dZ);

                // Here is a fast way to initialize the SmSolutionArray without 
                // invoking a malloc/free.  
                SmSolution aSols[10];
                SmSolutionArray sSolutions(10,aSols);

                // Just use the default for the best answer so far.
                double dBestAnswer = SM_BIG_DOUBLE;
                if (eSolverOperation == SM_SO_MAXIMIZE ||
                    eSolverOperation == SM_SO_PROJECTED_MAXIMIZE) dBestAnswer = 0.0;


//                if (eSolverOperation == SM_SO_RAYFIRE) {
//                    double dScale = (1.0 + sClPt.GetMaxDimension()) * 2.0;
//                    SER(cpBrep->RayIntersection(sClPt,&cpOptVectors[0],SM_EFF_ZERO*dScale,
//                        SM_EFF_ZERO*dScale,sSolutions));
//                }
//                else {
                // Solve for the global operation with a brep and point
                SER(SmTopologySolver::BrepPointSolve(cpBrep,sClPt,eSolverOperation,
                    SM_SR_ALL,1.0e-2,dBestAnswer,cpOptVectors,sSolutions)); 
//                }
                // Note that minimization and maximization always find answers
                if (eSolverOperation == SM_SO_MINIMIZE ||
                    eSolverOperation == SM_SO_MAXIMIZE) {
                    SM_ASSERT(sSolutions.GetSize() > 0);
                    if (sSolutions.GetSize() < 1) {
#ifdef SM_GFX_CODE
                        if ( smGet_DoGraphics() )
                        {
                            //sm_GraphicsLoop();
                            smgfx_SetColor( 1, 0, 0 );
                            sClPt.Draw();
                            if ( FALSE )
                            {
                                smgfx_SetColor( 0, 0, 0 );
                                cpBrep->Draw();
                            }
                            //sm_GraphicsLoop();
                        }
#endif
              }
                }
                lCount++;
                // Draw the result of the solve
                for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) {
                    rlNumFound ++;
                    SmSolution sSolution = sSolutions[kk];
                    SmPoint3d sFndPnt;
                    sSolution.GetPoint( 0, sFndPnt );
                    SmVector3d sTo = sFndPnt - sClPt;
#ifdef SM_GFX_CODE
                    if ( smGet_DoGraphics() )
                    {
                        smgfx_SetColor( 1, 0, 0 );
                        sTo.Draw( &sClPt );
                        sClPt.Draw();
                    }
#endif
                } // For each answer
            }
        }
    }

#ifdef SM_GFX_CODE
//    smgfx_Close();
#endif
    // Write out statistical stuff

    return SM_SUCCESS;

} // end my_test_tsurface_point_extrema


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_tsurface_point_intersect
  (const SmBrep *cpBrep,
   long lNumU,
   long lNumV,
   ULONG & rlNumFound)
{

    SmSolution aSols[10];
    SmSolutionArray sSolutions(10,aSols);

    ULONG lCount=0;
    
#ifdef SM_GFX_CODE
//    smgfx_Open(smgfx_GetRuleColor(this));
#endif
    
    
    rlNumFound = 0;
    SmTArray<SmFace*> sFaces;
    cpBrep->GetFaces(sFaces);
    for (ULONG k=0; k<sFaces.GetSize(); k++) {
        SmFace *pFace = sFaces[k];
        SmExtent2d sUVDomain = pFace->GetUVDomain();
        SmSurface *pSurface = pFace->GetSurface();

        for (long i=0; i<lNumU; i++) {
            for (long j=0; j<lNumV; j++) {
                SmPoint2d sUV = sUVDomain.Evaluate( i/(lNumU-1.0), j/(lNumV-1.0) );
                SmPoint3d sEvalPnt;
                pSurface->EvaluatePoint(sUV,sEvalPnt);
                double dBestAnswer = 0.0;
                SER(SmTopologySolver::BrepPointSolve(cpBrep,sEvalPnt,SM_SO_INTERSECT,
                    SM_SR_ALL,SM_ZONE_TOL_3D,dBestAnswer,NULL,sSolutions));
                lCount++;
                if (sSolutions.GetSize() != 0) {
                    rlNumFound ++;
#ifdef SM_GFX_CODE
                    if ( smGet_DoGraphics() )
                    {
                        smgfx_SetColor( 1, 0, 0 );
                        sEvalPnt.Draw();
                        smgfx_SetColor( 0, 0, 0 );
                    }
#endif
                }
                
            }
        }
    }

#ifdef SM_GFX_CODE
//    smgfx_Close();
#endif

    return SM_SUCCESS;

} // end my_test_tsurface_point_intersect


/***********************************************************************
PURPOSE --- This function computes a grid of lines and invokes the topology
    solver to solve the given operation between a curve and a brep with 
    trimmed surfaces.

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_tsurface_curve_solve
  (const SmBrep *cpBrep, 
   SmSolverOperationType eSolverOp,
   long lNumX,
   long lNumY,
   const SmVector3d * cpOptVectors,
   ULONG & rlNumFound)
{
    // Statistical stuff
    rlNumFound = 0;

    ULONG lCount=0;

    rlNumFound = 0;

    // Compute the bounding box of the Brep
    SmExtent3d sBBox;
    SER(cpBrep->CalculateBoundingBox(sBBox));

    SmVector3d sVectors[3];
    sVectors[0] = SmVector3d(0,0,1);

    // Here we declare the solution outside of the loop to
    // prevent memory allocations every time in the inner loop.
    SmSolutionArray sSolutions;

#ifdef SM_GFX_CODE
//    smgfx_Open(smgfx_GetRuleColor(this));
#endif

    // Now create an X, Y grid of lines which go from the minimum to the
    // maximum Z and call the solver on them.
    for (long i=0; i<lNumX; i++) {
        double dX = (sBBox.GetMax().x - sBBox.GetMin().x) / (lNumX-1+SM_EFF_ZERO);
        if (dX < SM_EFF_ZERO) i=lNumX;
        if (lNumX < 2) dX = 0.0;
        for (long j=0; j<lNumY; j++) {
            double dY = (sBBox.GetMax().y - sBBox.GetMin().y) / (lNumY-1+SM_EFF_ZERO);
            double dSize = 0.0; //(dX*lNumX + dY*lNumY) * 10000.0;
            SmPoint3d sStartPt( sBBox.GetMin().x + (i) * dX,
                    sBBox.GetMin().y + (j) * dY,
                    sBBox.GetMin().z-dSize);
            SmPoint3d sEndPt( sBBox.GetMin().x + (i) * dX,
                sBBox.GetMin().y + (j) * dY,
                sBBox.GetMax().z+dSize);
            if (sBBox.GetMax().z - sBBox.GetMin().z < SM_EFF_ZERO) {
                sStartPt.z -= 1.0;
                sEndPt.z += 1.0;
            }
            
            // Create the line to use in the solver
            SmCurve *pLine = my_create_line(*cpBrep->GetContext(),sStartPt,sEndPt,0.2,0,0.3);
            SmObjDelete sCleanup(pLine);
          
            double dBestAnswer = SM_BIG_DOUBLE;
            if (eSolverOp == SM_SO_MAXIMIZE ||
                eSolverOp == SM_SO_PROJECTED_MAXIMIZE) dBestAnswer = 0.0;

            // Invoke the Brep/Curve solver.
            sSolutions.ReSet();  
            SER(SmTopologySolver::BrepCurveSolve(cpBrep,*pLine,pLine->GetNaturalInterval(),
                eSolverOp,SM_SR_ALL,SM_ZONE_TOL_3D,dBestAnswer,cpOptVectors,sSolutions));
            
            lCount++;
                    
            if (FALSE) {
                sSolutions.Dump();
            }

            // Draw results returned from the solver.
            for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) {
                rlNumFound++;
                SmSolution & rSolution = sSolutions[kk];
                SmPoint3d sFndPnt1;
                rSolution.GetPoint( 0, sFndPnt1 );
                SmPoint3d sFndPnt2;
                rSolution.GetPoint( 1, sFndPnt2 );
#ifdef SM_GFX_CODE
                if ( smGet_DoGraphics() )
                {
                    smgfx_SetColor( 1.0, 0.0, 0.0 );
                    sFndPnt1.Draw();
                    SmVector3d sDiff = sFndPnt1 - sFndPnt2;
                    sFndPnt2.Draw();
                    sDiff.Draw( &sFndPnt2 );
                    smgfx_SetColor( 0, 0, 0 );
                }
#endif
            }
        }
    }
    
    
#ifdef SM_GFX_CODE
//    smgfx_Close();
#endif
    
    return SM_SUCCESS;

} // end my_test_tsurface_curve_solve


/***********************************************************************
PURPOSE --- This function invokes the topology solver to solve for the 
   given operation between two breps which contain trimmed surfaces.

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_tsurface_tsurface_solve
  (const SmBrep *cpBrep1, const SmBrep *cpBrep2, 
   SmSolverOperationType eSolverOp,
   const SmVector3d * cpOptVectors,
   ULONG & rlNumFound)
{
    // Statistical stuff
    rlNumFound = 0;

    const SmContext *pContext = cpBrep1->GetContext();
    // Invoke the Brep/Brep topology solver
    SmSolutionArray sSolutions;
    double dBestAnswer = SM_BIG_DOUBLE;
    if (eSolverOp == SM_SO_MAXIMIZE ||
        eSolverOp == SM_SO_PROJECTED_MAXIMIZE) dBestAnswer = 0.0;

    SmVector3d sVectors[3];
    sVectors[0] = SmVector3d(0,0,1);
    sVectors[1] = SmVector3d(1,0,0);
    if (cpOptVectors) {
        sVectors[0] = cpOptVectors[0];
    }

    // Invoke the solver
    SER(SmTopologySolver::BrepBrepSolve(cpBrep1,cpBrep2,
        eSolverOp,SM_SR_ALL,SM_ZONE_TOL_3D,dBestAnswer,sVectors,sSolutions));
    
    if (FALSE) {
        sSolutions.Dump();
    }
    // Draw the points which were produced and a vector
    // between them.
    for (ULONG kk = 0; kk < sSolutions.GetSize(); kk++) {
        rlNumFound++;
        SmSolution & rSolution = sSolutions[kk];
        SmPoint3d sFndPnt1;
        rSolution.GetPoint( 0, sFndPnt1 );
        SmPoint3d sFndPnt2;
        rSolution.GetPoint( 1, sFndPnt2 );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (FALSE) {
                smgfx_Erase();
                smgfx_SetLook(1, 2, 0, 0, 1); cpBrep1->Draw(TRUE); sm_GraphicsLoop();
                smgfx_SetLook(1, 2, 0, 1, 0); cpBrep2->Draw(TRUE); sm_GraphicsLoop();
            }
            smgfx_SetLook(4, 6, 1, 0, 0); sFndPnt1.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(4, 6, 1, 0, 1); sFndPnt2.Draw(); sm_GraphicsLoop();
            SmVector3d sDiff = sFndPnt1 - sFndPnt2;
            smgfx_SetLook(1, 2, 0, 0, 0); sDiff.Draw(&sFndPnt2); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
    }
     
    // Do the same thing with the shape solver and make sure that
    // we get the same answers
    SmTArray<SmBrep*> sBreps;
    SmTArray<SmFace*> sFaces;
    SmTArray<SmEdge*> sEdges;
    SmTArray<SmVertex*> sVertices;
    sBreps.Add((SmBrep*)(void*)cpBrep1);
    SmShape *pShape1 = new (*pContext) SmShape(sBreps,sFaces,sEdges,sVertices); // increments an unused Mark value
    SmObjDelete sCleanShape(pShape1);
    sBreps.ReSet();
    sBreps.Add((SmBrep*)(void*)cpBrep2);
    SmShape *pShape2 = new (*pContext) SmShape(sBreps,sFaces,sEdges,sVertices); // increments an unused Mark value
    SmObjDelete sCleanShape2(pShape2);
    SER(SmTopologySolver::ShapeShapeSolve(pShape1,pShape2,
        eSolverOp,SM_SR_ALL,SM_ZONE_TOL_3D,dBestAnswer,sVectors,sSolutions));

    if (rlNumFound < 5) {
        SM_ASSERT(sSolutions.GetSize() == rlNumFound);
    }
    
    return SM_SUCCESS;

} // end my_test_tsurface_tsurface_solve


/***********************************************************************
PURPOSE --- This function tests point classification on all of the trimmed
   surfaces of a brep.  It creates a grid of lNumU by lNumV 2D points and
   calls the point classification on the face of the brep.

USAGE NOTES --- 
***********************************************************************/
PT_EXPORT SmStatus my_test_tsurface_point_classify
  (const SmBrep *cpBrep,
   long lNumU,
   long lNumV,
   ULONG & rlNumFound)
{
    // Statistical stuff
    rlNumFound = 0;


    // Get the faces of the brep
    SmTArray<SmFace*> sFaces;
    cpBrep->GetFaces(sFaces);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_Open(smgfx_GetRuleColor());
    }
#endif

    // For each face classify a grid of points
    for (ULONG kk=0; kk<sFaces.GetSize(); kk++) {
        const SmFace    * cpFace         = sFaces[kk];  
        const SmSurface & crSurface      = *cpFace->GetSurface();
        const SmExtent2d  crUVDomain     = cpFace->GetUVDomain();
        SmZoneTol3d       sFaceZoneTol3d = SmTol::GetZoneTol3d(cpFace) ;
        
        for (long i=0; i<lNumU; i++) {
            for (long j=0; j<lNumV; j++) {
                // Compute the 2D point
                SmPoint2d sUV = crUVDomain.Evaluate( i/(lNumU-1.0), j/(lNumV-1.0) );
                // Declare the point classification object which will 
                // contain the result of the classification.
                SmPointClassification sPointClass(sFaceZoneTol3d, cpBrep->GetContext()) ;

                // Do the actual work of classifying the point
                SER(cpFace->PointClassify(sUV,sFaceZoneTol3d, FALSE,TRUE,sPointClass));

                // If the point is not in the face continue on to the
                // next point
                if (sPointClass.GetPointClass() != SM_PC_FACE) continue;
                // If the point is in the face draw it
                rlNumFound ++;
                SmPoint3d sEvalPnt;
                crSurface.EvaluatePoint(sUV,sEvalPnt);
#ifdef SM_GFX_CODE
                if (smGet_DoGraphics()) {
                    smgfx_SetColor(1, 0, 0);
                    sEvalPnt.Draw();
                    smgfx_SetColor(0, 0, 0);
                }
#endif
            }
        }
    }

#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_Close();
    }
#endif

    return SM_SUCCESS;

} // end my_test_tsurface_point_classify

/***********************************************************************
PURPOSE --- This function classifies set of concentric ellipses in the 
   parameter space of a trimmed surface relative to the boundary of the 
   trimmed surface.

USAGE NOTES --- 
***********************************************************************/
PT_EXPORT SmStatus my_test_tsurface_curve_classify
  (const SmBrep *cpBrep, 
   ULONG lNum, 
   ULONG & rlNumFound)
{
    // Statistical stuff
    rlNumFound = 0;


#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_Open(smgfx_GetRuleColor());
    }
#endif
    // Get the faces of the brep
    SmContext sContext;
    SmTArray<SmFace*> sFaces;
    cpBrep->GetFaces(sFaces);
    // For each face create a set of concentric ellipses and 
    // classify them
    for (ULONG kk=0; kk<sFaces.GetSize(); kk++) {
        const SmFace    * cpFace   = sFaces[kk];  // Get the face
        const SmContext * pContext =   cpFace->GetContext() 
                                     ? cpFace->GetContext()
                                     : &sContext ;
        // Get the UV domain of the face
        SmExtent2d sUVDomain = cpFace->GetUVDomain();
        // Create a vector between the corners of the domain
        SmVector2d sDiff = sUVDomain.GetMax() - sUVDomain.GetMin();
        // Divide to get a vector that goes from the center to 
        // a corner of the domain.  This vector will be scaled 
        // and used to get the ellipse radii.
        sDiff = sDiff/2.0;

        // For each concentric ellipses
        for (ULONG i=1; i<lNum; i++) {
            // Get vector which defines radii of ellipse
            SmVector2d sVec = (i/((double)lNum)) * sDiff;
            SmBSplineCurve *pUVCurve = NULL ;
            // Make ellipse center at center of domain
            SmPoint3d sCenter(sUVDomain.GetMin() + sDiff);
            // Create a coordinate system using the center point
            SmAxis2Placement sA2P;
            sA2P.SetCanonical(sCenter,SmVector3d(1,0,0),SmVector3d(0,1,0));

            // Create an ellipse using the coordinate system and the 
            // radii vector.
            SmBSplineCurve::CreateEllipseSegment(*pContext,2,sA2P,sVec.x,sVec.y,0.0,360.0,
                                                  SM_CO_QUADRATIC,pUVCurve);
            // Automatic cleanup of curve
            SmObjDelete sCleanup(pUVCurve);
            // Initialize the curve classification object with the
            // domain of interest for the classification (all of it).
            SmCurveInterval pData[10];
            SmCurveClassification sCurveClass(pUVCurve,
                                              pUVCurve->GetNaturalInterval(),
                                              pUVCurve,
                                              SM_ZONE_TOL_3D*10.0,10,
                                              pData,
                                              FALSE,TRUE,
                                              cpFace->GetSurface());

            // Now do the real work of classifying the curve relative
            // to the face.  The resulting curve classification object
            // will tell you just about all you need to know about how
            // the curve interacts with the face.  However we only use
            // a little of that information to determine which portions 
            // of the curve are inside of the face to draw them.
            SE(cpFace->CurveOnClassify(TRUE,sCurveClass));

            // Draw the portions of the curve inside of the face
            for (ULONG ij=0; ij<sCurveClass.GetSize(); ij++) {
                SmCurveInterval & rCIvl = sCurveClass[ij];
                if (rCIvl.m_vMid.GetPointClass() == SM_PC_FACE) {
                    rlNumFound ++;
#ifdef SM_GFX_CODE
                    if (smGet_DoGraphics()) {
                        smgfx_SetColor(1, 0, 0);
                        // Create a temporary 3D curve which is defined by the
                        // mapping of the 2D curve through the surface to draw the results
                        // in 3D.
                        SmCrvOnSurf s3DCurve(*pUVCurve, *cpFace->GetSurface());
                        s3DCurve.SetContext(NULL);
                        s3DCurve.DrawWDeriv(rCIvl.m_vInterval, 0);
                        smgfx_SetColor(0, 0, 0);
                    }
#endif
                }
            }
        }
    }

#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_Close();
    }
#endif

    return SM_SUCCESS;

} // end my_test_tsurface_curve_classify


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_topo_regression_test
  (SmBrep *pBrep, 
   ULONG lPointClass,
   ULONG lPointMin,
   ULONG lPointMax,
   ULONG lPointNorm,
   ULONG lPointIntersect,
   ULONG lCurveClass,
   ULONG lCurveSolveInt,
   ULONG lCurveSolveMin,
   ULONG lCurveSolveMax,
   ULONG lCrvProjection,
   ULONG lCrvSection)
{
    ULONG lNumFound;
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_SetColor(0, 0, 0);
        pBrep->Draw();
    }
#endif

    SER(my_test_tsurface_point_classify(pBrep,33,33,lNumFound));
    SM_ASSERT(lNumFound == lPointClass);
    
    SER(my_test_tsurface_point_extrema(pBrep,SM_SO_MINIMIZE,
        10,10,2,NULL,lNumFound));
    SM_ASSERT(lNumFound == lPointMin);
    
    SER(my_test_tsurface_point_extrema(pBrep,SM_SO_MAXIMIZE,
        3,3,2,NULL,lNumFound));
    SM_ASSERT(lNumFound == lPointMax);
    
    SER(my_test_tsurface_point_extrema(pBrep,SM_SO_NORMALIZE,
        10,10,2,NULL,lNumFound));
    SM_ASSERT(lNumFound == lPointNorm);
    
    SER(my_test_tsurface_point_intersect(pBrep,33,33,lNumFound));
    SM_ASSERT(lNumFound == lPointIntersect);
    
    SER(my_test_tsurface_curve_classify(pBrep,20,lNumFound));
    SM_ASSERT(lNumFound == lCurveClass);
    
    SER(my_test_tsurface_curve_solve(pBrep,SM_SO_INTERSECT,
        5,5,NULL,lNumFound));
    SM_ASSERT(lNumFound == lCurveSolveInt);
    
    SER(my_test_tsurface_curve_solve(pBrep,SM_SO_MINIMIZE,
        5,5,NULL,lNumFound));
    SM_ASSERT(lNumFound == lCurveSolveMin);
    
    SER(my_test_tsurface_curve_solve(pBrep,SM_SO_MAXIMIZE,
        5,5,NULL,lNumFound));
    SM_ASSERT(lNumFound == lCurveSolveMax);
    
    SER(my_test_tsurf_projection(pBrep,0.0001,10,lNumFound));
    SM_ASSERT(lNumFound == lCrvProjection);

    SER(my_test_tsurf_section(pBrep,0.0001,SmVector3d(1,1,1),10,lNumFound));
    SM_ASSERT(lNumFound == lCrvSection);
    
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
    }
#endif

    return SM_SUCCESS;

} // end my_topo_regression_test

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
static SmBrep * sm_make_xy_sheet(const SmContext & crContext, double dSize)
{
  SmAxis2Placement sRefFrame ;
  sRefFrame.SetCanonical(SmPoint3d(0.0, 0.0, 0.0),
                         SmVector3d(1.0, 0.0, 0.0),
                         SmVector3d(0.0, 1.0, 0.0)) ;
  return SmPrimitiveCreation::CreateRectangle(crContext, 1.0e-5, dSize, dSize, sRefFrame) ;
}

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

// NURB uv box of the half-cylinder (radius dR about z, angles 0..180 from +x)
// between dZ0 and dZ1. Corners are dropped with the face midpoint as the guess
// so a corner on the seam stays on the face's side of it.
static SmExtent2d sm_coaxial_face_uv_box
  (SmSurface * pSurf,
   double      dR,
   double      dZ0,
   double      dZ1)
{
  SmExtent2d sBox ;
  SmExtent2d sDom = pSurf->GetNaturalUVDomain() ;
  SmBoolean bOk = FALSE ;
  SmBoolean bMulti = FALSE ;
  double dGap = 0.0 ;
  SmPoint2d sMid, sUV ;
  pSurf->DropPoint(SmPoint3d(0.0, dR, 0.5 * (dZ0 + dZ1)), sDom, NULL, bOk, sMid, dGap, bMulti) ;
  const double daX[2] = { dR, -dR } ;
  const double daZ[2] = { dZ0, dZ1 } ;
  for(int ii=0; ii<2; ii++)
    {
      for(int jj=0; jj<2; jj++)
        {
          pSurf->DropPoint(SmPoint3d(daX[ii], 0.0, daZ[jj]), sDom, &sMid, bOk, sUV, dGap, bMulti) ;
          sBox.AddPoint2d(sUV) ;
        }
    }
  return sBox ;
}

static SmStatus my_test_coaxial_cylinder_merge()
{
  // Two faces on one cylinder (r = 50 about z), both over 0..180 deg from +x:
  //   A  z in [-10, 10] on a 360 deg cylinder, seam at +x, height -20..20
  //   B  z in [ 10, 13] on a second cylinder, per variant
  // RemoveTopologicalEdgesAndVertices must leave one valid face. Variants 2 and 3
  // put B's seam away from A's; covering the other surface's natural domain fails
  // there, covering the other face's uv box does not.
  const double dR = 50.0 ;
  const SmVector3d sZ(0.0, 0.0, 1.0) ;
  struct Variant
    {
      SmVector3d sSeam ;
      double dStart ;
      double dEnd ;
      double dZ0 ;
      double dZ1 ;
    } ;
  const Variant aVariant[4] =
    {
      { SmVector3d( 1.0,  0.0, 0.0),   0.0, 180.0,   10.0,  13.0 },
      { SmVector3d( 0.0, -1.0, 0.0),   0.0, 356.0, -100.0, 100.0 },
      { SmVector3d( 0.0, -1.0, 0.0),   0.0, 360.0, -100.0, 100.0 },
      { SmVector3d( 1.0,  0.0, 0.0),   0.0, 360.0, -100.0, 100.0 }
    } ;

  for(int iVariant=0; iVariant<4; iVariant++)
    {
      const Variant & crV = aVariant[iVariant] ;
      SmContext sContext ;
      SmVector3d sXA(1.0, 0.0, 0.0) ;
      SmVector3d sYA = sZ * sXA ;
      SmCylinder * pA = new(sContext) SmCylinder(SmPoint3d(0.0, 0.0, -20.0), sXA, sYA, dR,
                                                0.0, 360.0, 40.0, FALSE, FALSE, FALSE, &sContext) ;
      SmVector3d sXB = crV.sSeam ;
      SmVector3d sYB = sZ * sXB ;
      SmCylinder * pB = new(sContext) SmCylinder(SmPoint3d(0.0, 0.0, crV.dZ0), sXB, sYB, dR,
                                                crV.dStart, crV.dEnd, crV.dZ1 - crV.dZ0,
                                                FALSE, FALSE, FALSE, &sContext) ;
      NER(pA) ;
      NER(pB) ;

      SmBrep * pBrep = new(sContext) SmBrep() ;
      SmObjDelete sClean(pBrep) ;
      pBrep->m_bEditingEnabled = TRUE ;
      SmFace * pFaceA = NULL ;
      SmFace * pFaceB = NULL ;
      SmExtent2d sBoxA = sm_coaxial_face_uv_box(pA, dR, -10.0, 10.0) ;
      SmExtent2d sBoxB = sm_coaxial_face_uv_box(pB, dR, 10.0, 13.0) ;
      SER(pBrep->CreateFaceFromSurface(pA, sBoxA, pFaceA)) ;
      SER(pBrep->CreateFaceFromSurface(pB, sBoxB, pFaceB)) ;
      SER(pBrep->StitchAndOrient()) ;

      SmTArray<SmFace*> sFaces ;
      pBrep->GetFaces(sFaces) ;
      if(sFaces.GetSize() != 2)
        { return SM_ERR ; }

      SER(pBrep->RemoveTopologicalEdgesAndVertices()) ;
      pBrep->GetFaces(sFaces) ;
      SmAssertArray sAsserts ;
      SmBoolean bValid = pBrep->AssertValid(&sAsserts, SM_LEVEL_2, SM_WALK, NULL) ;
      if(sFaces.GetSize() != 1 || !bValid || sAsserts.GetSize() != 0)
        { return SM_ERR ; }

      SmTArray<SmVertex*> sVertices ;
      pBrep->GetVertices(sVertices) ;
      SmExtent3d sBox ;
      for(ULONG ii=0; ii<sVertices.GetSize(); ii++)
        { sBox.AddPoint3d(sVertices[ii]->GetPoint()) ; }
      if(sVertices.GetSize() != 4)
        { return SM_ERR ; }
      if(   smos_Fabs(sBox.GetMin().x + 50.0) > 1.0e-4
         || smos_Fabs(sBox.GetMax().x - 50.0) > 1.0e-4
         || smos_Fabs(sBox.GetMin().y) > 1.0e-4
         || smos_Fabs(sBox.GetMax().y) > 1.0e-4
         || smos_Fabs(sBox.GetMin().z + 10.0) > 1.0e-4
         || smos_Fabs(sBox.GetMax().z - 13.0) > 1.0e-4)
        { return SM_ERR ; }
    }

  MYPRINTF(_T("\nOKAY: coaxial cylinder faces merge across different seams")) ;
  return SM_SUCCESS ;
}

/*******************************************************************//**
PURPOSE: Keep the mapped face domain through successive coaxial merges.

NOTES: Each surface is much larger than its face. Test both insertion orders
       and broad input domains, which require the trim-derived fallback.
***********************************************************************/
static SmStatus my_test_coaxial_merge_domain()
{
  for(int iReverse=0; iReverse<2; iReverse++)
    {
      for(int iBroad=0; iBroad<2; iBroad++)
        {
          SmContext sContext;
          SmBrep *pBrep = new(sContext) SmBrep();
          SmObjDelete sClean(pBrep);
          pBrep->m_bEditingEnabled = TRUE;
          for(int ii=0; ii<3; ii++)
            {
              int iFace = iReverse ? 2-ii : ii;
              SmVector3d sX = iFace == 1 ? SmVector3d(0.0,-1.0,0.0) : SmVector3d(1.0,0.0,0.0);
              SmVector3d sY = SmVector3d(0.0,0.0,1.0) * sX;
              SmCylinder *pSurface = new(sContext) SmCylinder(SmPoint3d(0.0,0.0,-100.0),
                  sX, sY, 50.0, 0.0, 360.0, 200.0, FALSE, FALSE, FALSE, &sContext);
              SmExtent2d sFaceDomain = sm_coaxial_face_uv_box(pSurface, 50.0, iFace, iFace+1.0);
              SmFace *pFace = NULL;
              SER(pBrep->CreateFaceFromSurface(pSurface, sFaceDomain, pFace));
              if(iBroad)
                { pFace->SetUVDomain(pSurface->GetNaturalUVDomain()); }
            }
          SER(pBrep->StitchAndOrient());

          // The second pass must not change the result or lose the saved bounds.
          for(int iPass=0; iPass<2; iPass++)
            {
              SER(pBrep->RemoveTopologicalEdgesAndVertices());
              SmTArray<SmFace*> sFaces;
              pBrep->GetFaces(sFaces);
              if(sFaces.GetSize() != 1)
                { return SM_ERR; }
              SmFace *pFace = sFaces[0];
              SmExtent2d sDomain = pFace->GetUVDomain();
              SmExtent2d sSTEPDomain;
              SER(pFace->GetSurface()->ConvertDomainFromNURBSToSTEP(sDomain, sSTEPDomain));
              // Expected angular width is 180 degrees and height is 3; allow
              // the existing coverage sampler's conservative expansion.
              if(sSTEPDomain.GetSize().x > 200.0 || sSTEPDomain.GetSize().y > 4.0)
                { return SM_ERR; }
              SmExtent2d sTrimDomain;
              SER(pFace->CalculateUVDomainFromUVTrimCurves(sTrimDomain, TRUE));
              if(!sTrimDomain.IsContainedBy(sDomain, 1.0e-7))
                { return SM_ERR; }
              SmAssertArray sAsserts;
              if(!pBrep->AssertValid(&sAsserts, SM_LEVEL_2, SM_WALK, NULL) || sAsserts.GetSize() != 0)
                { return SM_ERR; }
            }
        }
    }
  MYPRINTF(_T("\nOKAY: coaxial merges retain mapped face domains and refine broad inputs"));
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Preserve full-surface coverage and support optional domain subsets.
***********************************************************************/
static SmStatus my_test_surface_cover_domains()
{
  SmContext sContext;
  // Keep the legacy four-argument call: coincident planes cover, offset ones do not.
  for(int iCase=0; iCase<2; iCase++)
    {
      SmPlane sFirst(SmPoint3d(0,0,0), SmVector3d(1,0,0), SmVector3d(0,1,0),
                     SmVector2d(1,1), SmExtent2d(0,0,1,1), &sContext);
      SmPlane sOther(SmPoint3d(0,0,iCase), SmVector3d(1,0,0), SmVector3d(0,1,0),
                     SmVector2d(1,1), SmExtent2d(0,0,2,2), &sContext);
      SmSurface *pCover = NULL;
      double dMaxDist;
      SER(sFirst.CoverOtherCoincidentSurface(&sOther, 1.0e-7, dMaxDist, pCover));
      if(iCase == 0 ? (pCover != &sFirst || dMaxDist > 1.0e-7) : pCover != NULL)
        { return SM_ERR; }
    }
  // Full domains cross the other cylinder's seam; their face subsets do not.
  SmCylinder *pFirst = new(sContext) SmCylinder(SmPoint3d(0,0,-100), SmVector3d(1,0,0), SmVector3d(0,1,0),
                    50, 0, 360, 200, FALSE, FALSE, FALSE, &sContext);
  SmCylinder *pOther = new(sContext) SmCylinder(SmPoint3d(0,0,-100), SmVector3d(0,-1,0), SmVector3d(1,0,0),
                    50, 0, 360, 200, FALSE, FALSE, FALSE, &sContext);
  SmObjDelete sCleanFirst(pFirst), sCleanOther(pOther);
  SmSurface *pCover = NULL;
  double dMaxDist;
  SER(pFirst->CoverOtherCoincidentSurface(pOther, 1.0e-7, dMaxDist, pCover));
  if(pCover != NULL)
    { return SM_ERR; }
  SmExtent2d sFirstDomain = sm_coaxial_face_uv_box(pFirst, 50, -10, 10);
  SmExtent2d sOtherDomain = sm_coaxial_face_uv_box(pOther, 50, 10, 13);
  // Domain-limited coverage must also work without requesting the merged output.
  SER(pFirst->CoverOtherCoincidentSurface(pOther, 1.0e-7, dMaxDist, pCover,
                                        &sFirstDomain, &sOtherDomain));
  if(pCover != pFirst || dMaxDist > 1.0e-7)
    { return SM_ERR; }
  MYPRINTF(_T("\nOKAY: shared surface coverage supports natural domains and optional subsets"));
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Exercise end-tolerance and UV-interval repairs in ValidateGeometry.
***********************************************************************/
static SmStatus my_test_edge_geometry_repairs()
{
  // Observe the interval handed to the real distance routine. A straight trim
  // can still measure zero distance with stale bounds, so that numeric result
  // alone does not prove ValidateGeometry used the repaired interval.
  class IntervalCheckingCurve : public SmBSplineCurve
  {
  public:
    mutable ULONG m_lChecks = 0;
    mutable SmBoolean m_bIntervalsMatch = TRUE;
    IntervalCheckingCurve(const SmBSplineCurve &crCurve) : SmBSplineCurve(crCurve) {}
    SmStatus CurveMaxDistanceBetween(const SmExtent1d &crInterval, const SmCurve &crOther,
        double dOtherStart, double dOtherEnd, ULONG lNumSamples, double *pOptMaxDistance,
        double &rMaxDistance, double *pOptCurveT, double *pOptOtherT,
        SmBoolean *pOptIncorrectDir) const override
    {
      m_lChecks++;
      SmExtent1d sOtherInterval = crOther.GetNaturalInterval();
      m_bIntervalsMatch = m_bIntervalsMatch
          && SM_ARE_SAME(dOtherStart, sOtherInterval.GetMin())
          && SM_ARE_SAME(dOtherEnd, sOtherInterval.GetMax());
      return SmBSplineCurve::CurveMaxDistanceBetween(crInterval, crOther, dOtherStart,
          dOtherEnd, lNumSamples, pOptMaxDistance, rMaxDistance, pOptCurveT,
          pOptOtherT, pOptIncorrectDir);
    }
  };

  SmContext sContext;
  SmBrep *pBrep = new(sContext) SmBrep();
  SmObjDelete sClean(pBrep);
  SmShell *pShell = NULL;
  SmVertex *pStart = NULL, *pEnd = NULL;
  SmEdge *pEdge = NULL;
  SmBSplineCurve *pLine = NULL;
  SER(pBrep->MakeShellVertex(pBrep->GetInfiniteRegion(), SmPoint3d(0,0,0), pShell, pStart));
  SER(SmBSplineCurve::CreateLineSegment(sContext, 3, SmPoint3d(0,0,0), SmPoint3d(10,0,0), pLine));
  SER(pBrep->MakeWireEdgeVertex(pBrep->GetInfiniteRegion(), pStart, pLine,
      pLine->GetNaturalInterval(), SM_OT_SAME, SmPoint3d(10,0,0), pEdge, pEnd));
  pStart->SetTolerance(1.0e-4, FALSE);
  pEnd->SetTolerance(1.0e-4, FALSE);
  SER(pEdge->ValidateGeometry(FALSE));
  pEnd->SetPoint(SmPoint3d(10,1.0e-3,0));
  SER(pEdge->ValidateGeometry(TRUE));
  if(smos_Fabs(pEnd->GetTolerance() - 2.0e-3) > 1.0e-10
     || smos_Fabs(pStart->GetTolerance() - 1.0e-4) > 1.0e-10)
    { return SM_ERR; }
  SER(pEdge->ValidateGeometry(FALSE));
  MYPRINTF(_T("\nOKAY: end-gap repair uses the end distance without changing start tolerance"));

  SmBrep *pSheet = new(sContext) SmBrep();
  SmObjDelete sCleanSheet(pSheet);
  SmPlane *pPlane = new(sContext) SmPlane(SmPoint3d(0,0,0), SmVector3d(1,0,0),
      SmVector3d(0,1,0), SmVector2d(1,1), SmExtent2d(0,0,10,10), &sContext);
  SmFace *pFace = NULL;
  SER(pSheet->CreateFaceFromSurface(pPlane, pPlane->GetNaturalUVDomain(), pFace));
  SmTArray<SmLoopuse*> sLoopuses;
  pFace->GetUpwardLoopuses(sLoopuses);
  if(sLoopuses.GetSize() != 1)
    { return SM_ERR; }
  SmTArray<SmEdgeuse*> sEdgeuses;
  sLoopuses[0]->GetEdgeuses(sEdgeuses);
  if(sEdgeuses.GetSize() != 4)
    { return SM_ERR; }
  SmEdgeuse *pEU = sEdgeuses[0];
  pEdge = pEU->GetEdge();
  SmBSplineCurve *pOldCurve = SM_CAST_PTR(SmBSplineCurve, pEdge->GetCurve());
  NER(pOldCurve);
  IntervalCheckingCurve *pCheckingCurve = new(sContext) IntervalCheckingCurve(*pOldCurve);
  pEdge->SetCurve(pCheckingCurve, TRUE);
  pCheckingCurve->SetOwner(pEdge);
  SmBSplineCurve *pUV = NULL;
  SER(pEU->GetOrCreateUVTrimCurve(pUV));
  SER(pEdge->ValidateGeometry(FALSE));
  SER(pUV->EditParameterization(SmExtent1d(2,5)));
  pCheckingCurve->m_lChecks = 0;
  pCheckingCurve->m_bIntervalsMatch = TRUE;
  SER(pEdge->ValidateGeometry(TRUE));
  if(pCheckingCurve->m_lChecks == 0 || !pCheckingCurve->m_bIntervalsMatch
     || !SM_ARE_SAME(pUV->GetNaturalInterval().GetMin(), pEdge->GetInterval().GetMin())
     || !SM_ARE_SAME(pUV->GetNaturalInterval().GetMax(), pEdge->GetInterval().GetMax()))
    { return SM_ERR; }
  SmAssertArray sAsserts;
  if(!pSheet->AssertValid(&sAsserts, SM_LEVEL_0, SM_WALK, NULL) || sAsserts.GetSize() != 0)
    { return SM_ERR; }
  sAsserts.ReSet();
  if(!pSheet->AssertValid(&sAsserts, SM_LEVEL_2, SM_WALK, NULL) || sAsserts.GetSize() != 0)
    { return SM_ERR; }
  MYPRINTF(_T("\nOKAY: UV distance check receives the repaired trim-curve interval"));
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Join lines whose NURBS and analytic parameter directions differ.
***********************************************************************/
static SmStatus my_test_reversed_line_join()
{
  for(int iCase=0; iCase<16; iCase++)
    {
      SmContext sContext;
      SmBoolean bInsideOut = (iCase & 1) != 0;
      SmBoolean bOtherInsideOut = (iCase & 2) != 0;
      ULONG lJoinEndThis = (iCase >> 2) & 1;
      ULONG lJoinEndOther = (iCase >> 3) & 1;
      SmLine sLine(SmPoint3d(0,0,0), SmVector3d(1,0,0), SmExtent1d(2,7),
                   2.0, 3, &sContext, NULL, bInsideOut);
      SmPoint3d sStart, sEnd;
      sLine.GetEnds(sStart, sEnd);
      SmPoint3d sCommon = lJoinEndThis ? sEnd : sStart;
      SmPoint3d sFar(sCommon.x + (sCommon.x > 9.0 ? 6.0 : -6.0), 0, 0);
      SmPoint3d sOtherStart = lJoinEndOther ? sFar : sCommon;
      SmPoint3d sOtherEnd = lJoinEndOther ? sCommon : sFar;
      SmVector3d sDirection = bOtherInsideOut ? sOtherStart-sOtherEnd : sOtherEnd-sOtherStart;
      SmLine sOther(bOtherInsideOut ? sOtherEnd : sOtherStart, sDirection,
                    SmExtent1d(0,2), 3.0, 3, &sContext, NULL, bOtherInsideOut);
      double dTol = 1.0e-7;
      SER(sLine.JoinWith(lJoinEndThis, &sOther, lJoinEndOther, &dTol));
      SmPoint3d sNewStart, sNewEnd;
      sLine.GetEnds(sNewStart, sNewEnd);
      if(sNewStart.DistanceBetween(lJoinEndThis ? sStart : sFar) > dTol
         || sNewEnd.DistanceBetween(lJoinEndThis ? sFar : sEnd) > dTol)
        { return SM_ERR; }
      sOther.GetEnds(sNewStart, sNewEnd);
      if(sNewStart.DistanceBetween(sOtherStart) > dTol || sNewEnd.DistanceBetween(sOtherEnd) > dTol)
        { return SM_ERR; }
    }
  MYPRINTF(_T("\nOKAY: line joins preserve endpoints for both parameter directions"));
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Validate coaxial cleanup with swapped and reversed surface parameters.
***********************************************************************/
static SmStatus my_test_coaxial_merge_orientation()
{
  // Independently vary each surface's swap/inside-out flags and creation order.
  for(int iCase=0; iCase<32; iCase++)
    {
      SmContext sContext;
      SmBrep *pBrep = new(sContext) SmBrep();
      SmObjDelete sClean(pBrep);
      pBrep->m_bEditingEnabled = TRUE;
      for(int ii=0; ii<2; ii++)
        {
          int iFace = (iCase & 16) ? 1-ii : ii;
          SmVector3d sX = iFace ? SmVector3d(0,-1,0) : SmVector3d(1,0,0);
          SmVector3d sY = SmVector3d(0,0,1) * sX;
          SmBoolean bSwap = (iCase & (1 << iFace)) != 0;
          SmBoolean bInsideOut = (iCase & (4 << iFace)) != 0;
          SmCylinder *pSurface = new(sContext) SmCylinder(SmPoint3d(0,0,-100),
              sX, sY, 50.0, 0.0, 360.0, 200.0, bSwap, bInsideOut, FALSE, &sContext);
          SmExtent2d sDomain = sm_coaxial_face_uv_box(pSurface, 50.0, iFace ? 10.0 : -10.0,
                                                    iFace ? 13.0 : 10.0);
          SmExtent2d sSTEPDomain, sRoundTripDomain;
          SER(pSurface->ConvertDomainFromNURBSToSTEP(sDomain, sSTEPDomain));
          SER(pSurface->ConvertDomainFromSTEPToNURBS(sSTEPDomain, sRoundTripDomain));
          if(sDomain.GetMin().DistanceBetween(sRoundTripDomain.GetMin()) > 1.0e-7
             || sDomain.GetMax().DistanceBetween(sRoundTripDomain.GetMax()) > 1.0e-7)
            { return SM_ERR; }
          SmFace *pFace = NULL;
          SER(pBrep->CreateFaceFromSurface(pSurface, sDomain, pFace));
        }
      SER(pBrep->StitchAndOrient());
      // Validate before cleanup, after cleanup, and after an idempotent repeat.
      for(int iPass=0; iPass<3; iPass++)
        {
          if(iPass != 0)
            { SER(pBrep->RemoveTopologicalEdgesAndVertices()); }
          SmAssertArray sAsserts;
          if(!pBrep->AssertValid(&sAsserts, SM_LEVEL_0, SM_WALK, NULL) || sAsserts.GetSize() != 0)
            { return SM_ERR; }
          sAsserts.ReSet();
          if(!pBrep->AssertValid(&sAsserts, SM_LEVEL_2, SM_WALK, NULL) || sAsserts.GetSize() != 0)
            { return SM_ERR; }
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          if(sFaces.GetSize() != (iPass == 0 ? 2u : 1u))
            { return SM_ERR; }
          double dArea = 0.0;
          for(ULONG ii=0; ii<sFaces.GetSize(); ii++)
            {
              double dFaceArea = 0.0, dVolume = 0.0;
              SmTArray<SmVector3d> sMoments;
              SER(sFaces[ii]->ComputePreciseProperties(SM_OT_SAME, 1.0e-8, SmPoint3d(0,0,0),
                  10000, 0, dFaceArea, dVolume, sMoments, SM_PPF_AREA));
              dArea += dFaceArea;
            }
          // Allow numerical integration error; AssertValid separately checks
          // the edge/vertex/surface agreement at the modeling tolerance.
          double dExpectedArea = 50.0 * SM_PI * 23.0;
          if(smos_Fabs(dArea - dExpectedArea) > 1.0e-6 * dExpectedArea)
            { return SM_ERR; }
        }
    }
  MYPRINTF(_T("\nOKAY: coaxial cleanup preserves geometry for swapped/reversed surface parameters"));
  return SM_SUCCESS;
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
PURPOSE --- Optional outputs must not change the returned surface pole mask.
***********************************************************************/
static SmStatus my_test_poles_without_optional_outputs()
{
  MYPRINTF(_T("\n********** Entered: my_test_poles_without_optional_outputs"));
  for(ULONG lSwap=0; lSwap<2; ++lSwap)
    {
      SmContext sContext;
      SmSphere *pSphere = NULL;
      SER(SmSphere::CreateCanonical(sContext,SmAxis2Placement(),2.0,pSphere));
      SmObjDelete sCleanup(pSphere);
      if(lSwap) { SER(pSphere->SwapUV()); }

      ULONG lApprox   = SM_SS_NONE;
      ULONG lExpected = lSwap ? SM_SS_UMIN | SM_SS_UMAX : SM_SS_VMIN | SM_SS_VMAX;
      if(   pSphere->GetSingularities(NULL,NULL,&lApprox) != lExpected
         || pSphere->GetSingularities() != lExpected)
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
      NER(sBuilder.StartFace(SM_OT_SAME));
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

SmStatus my_test_topology()
{
  SmContext sContext;
  SmBrep   * pBrep = new(sContext) SmBrep();
  SmShell  * pNewShell;
  SmVertex * pNewVertex;

  MYPRINTF(_T("\n\n"));
  MYPRINTF(_T("********** Entered: my_test_topology"));

  SER(my_test_poles_without_optional_outputs());
  SER(my_test_closed_sphere_properties());
  SER(my_test_heal_single_vertex_sphere());
  // Scale each seam independently to exercise both adjacent derivative intervals.
  SER(my_test_doubly_periodic_vertexuse_corners(SmExtent1d(0.,1.),SmExtent1d(0.,1.)));
  SER(my_test_doubly_periodic_vertexuse_corners(SmExtent1d(0.,1e-4),SmExtent1d(0.,1.)));
  SER(my_test_doubly_periodic_vertexuse_corners(SmExtent1d(0.,1.),SmExtent1d(0.,1e-4)));
  SER(my_test_single_periodic_vertexuse(FALSE,SmExtent1d(0.,1.)));
  SER(my_test_single_periodic_vertexuse(TRUE,SmExtent1d(0.,1.)));
  SER(my_test_single_periodic_vertexuse(FALSE,SmExtent1d(0.,1e-4)));
  SER(my_test_single_periodic_vertexuse(TRUE,SmExtent1d(0.,1e-4)));
  SER(my_test_face_line_intersect_growth());
  SER(my_test_loop_uv_gap_neighborhood());
  SER(my_test_stale_edgeuse_owner()) ;
  SER(my_test_validate_required_pointers()) ;
  SER(my_test_squeeze_edge_tracks_removal()) ;
  SER(my_test_tracks_changed_objects()) ;
  SER(my_test_seam_uv_redistribution_at_poles()) ;
  SER(my_test_move_seam_refreshes_face_props()) ;
  SER(my_test_tracks_nested_geometry_and_resets()) ;
  SER(my_test_missing_poles_refreshes_all_faces()) ;
  SER(my_test_get_loop_crossings()) ;
  SER(my_test_face_loop_nesting()) ;
  SER(my_test_coaxial_cylinder_merge()) ;
  SER(my_test_coaxial_merge_domain()) ;
  SER(my_test_surface_cover_domains()) ;
  SER(my_test_edge_geometry_repairs()) ;
  SER(my_test_reversed_line_join()) ;
  SER(my_test_coaxial_merge_orientation()) ;

  // Test Shell vertex creation
  SER(pBrep->MakeShellVertex(pBrep->GetInfiniteRegion(),SmPoint3d(0,0,0),pNewShell,pNewVertex));
  SmVertex *pNewVertex2;
  SER(pBrep->MakeShellVertex(pBrep->GetInfiniteRegion(),SmPoint3d(1,1,1),pNewShell,pNewVertex2));

  SmTArray<SmEdgeuse*> sEdgeuses;

  // Test wire edge creation with vertex
  SmEdge         * pNewEdge;
  SmVertex       * pNewVertex3;
  SmBSplineCurve * pLine = NULL ;
  SER(SmBSplineCurve::CreateLineSegment(sContext,3,pNewVertex->GetPoint(),SmPoint3d(1,0,0),pLine));
  SER(pBrep->MakeWireEdgeVertex(pBrep->GetInfiniteRegion(),pNewVertex,
                                pLine,pLine->GetNaturalInterval(),
                                SM_OT_SAME,SmPoint3d(1,0,0),pNewEdge,pNewVertex3));
  sEdgeuses.Add(pNewEdge->GetPrimaryEdgeuse());

  // Test wire edge creation between vertices
  SER(SmBSplineCurve::CreateLineSegment(sContext,3,
                                        pNewVertex3->GetPoint(),
                                        pNewVertex2->GetPoint(),pLine));
  SER(pBrep->MakeWireEdge(pBrep->GetInfiniteRegion(),pNewVertex3,pNewVertex2,
                          pLine,pLine->GetNaturalInterval(),SM_OT_SAME,pNewEdge));
  sEdgeuses.Add(pNewEdge->GetPrimaryEdgeuse());

  SER(pBrep->MakeEdgeTopology(pBrep->GetInfiniteRegion(),pNewVertex2,pNewVertex,pNewEdge));
  SER(SmBSplineCurve::CreateLineSegment(sContext,3,pNewVertex2->GetPoint(),pNewVertex->GetPoint(),pLine));
  pNewEdge->SetCurve(pLine, TRUE ) ; // TRUE = delete preExisting Edge->Curve - expect none here
                                     // side effect: delete current pSurviveEdge->UVTrimCurve
  pLine->SetOwner( pNewEdge );  // memory leak otherwise
  pNewEdge->SetInterval(pLine->GetNaturalInterval());
  pNewEdge->SetOrientation(SM_OT_SAME);
  sEdgeuses.Add(pNewEdge->GetPrimaryEdgeuse());

  // Now Make a face
  SmTArray<ULONG> sLoops;
  sLoops.Add(3);
//    SmTArray<SmVertex*> sVertexLoops;
//    SmRegion *pNewR;
//    SmShell *pNewS;
//    SmFace *pNewF;
//    SER(pBrep->MakeFaceTopology(pBrep->GetInfiniteRegion(),sLoops,sEdgeuses,sVertexLoops  // increments unlocked mark value
//        pNewR,pNewS,pNewF));


  // Make another vertex and two edges
  SmVertex *pNewVertex4;
  // Test Shell vertex creation
  SER(pBrep->MakeShellVertexTopology(pBrep->GetInfiniteRegion(),pNewShell,pNewVertex4));
  pNewVertex4->SetPoint(SmPoint3d(0,1,0));

  SER(pBrep->MakeEdgeTopology(pBrep->GetInfiniteRegion(),pNewVertex2,pNewVertex4,pNewEdge));
  SER(SmBSplineCurve::CreateLineSegment(sContext,3,
                                        pNewVertex2->GetPoint(),
                                        pNewVertex4->GetPoint(),
                                        pLine));
  pNewEdge->SetCurve(pLine, TRUE ) ; // TRUE = delete preExisting Edge->Curve - expect none here
                                     // side effect: delete current pSurviveEdge->UVTrimCurve
  pLine->SetOwner( pNewEdge );  // memory leak otherwise
  pNewEdge->SetInterval(pLine->GetNaturalInterval());
  pNewEdge->SetOrientation(SM_OT_SAME);
  SmTArray<SmEdgeuse*> sEdgeuses2;
  SmEdgeuse *pEU = (SmEdgeuse*)sEdgeuses[2];
  sEdgeuses2.Add(pEU->GetMate());
  sEdgeuses2.Add(pNewEdge->GetPrimaryEdgeuse());

  SER(pBrep->MakeEdgeTopology(pBrep->GetInfiniteRegion(),pNewVertex4,pNewVertex,pNewEdge));
  SER(SmBSplineCurve::CreateLineSegment(sContext,3,
                                        pNewVertex4->GetPoint(),
                                        pNewVertex->GetPoint(),
                                        pLine));
  pNewEdge->SetCurve(pLine, TRUE ) ; // TRUE = delete preExisting Edge->Curve - expect none here
                                     // side effect: delete current pSurviveEdge->UVTrimCurve
  pLine->SetOwner( pNewEdge );  // memory leak otherwise
  pNewEdge->SetInterval(pLine->GetNaturalInterval());
  pNewEdge->SetOrientation(SM_OT_SAME);
  sEdgeuses2.Add(pNewEdge->GetPrimaryEdgeuse());

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      pBrep->Draw();
  }
#endif
  SM_ASSERT(pBrep != NULL) ; delete pBrep; pBrep = NULL ;
  return SM_SUCCESS;

} // end my_test_topology

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_trim_surfaces()
{
    SmContext sContext;
    
    if (TRUE) {
        SmBrep *pNewBrep;
        ULONG ii;

        clock_t start = clock();

        // Was 1000  iters: don't need to wait that long, 100 is plenty.
        // But sm_PrintTime() goes only to the nearest whole second,
        // and 1000 works well for that ... but adds almost a minute to the run.
        // So: Just do 100 then multiply the time by 10.
        for (ii=0; ii<100; ii++) {
            SER(my_test_tsurf_cyl_creation_fast(sContext,pNewBrep, TRUE ));
            delete pNewBrep; pNewBrep=NULL;
        }
        clock_t finish = clock();
        // Multiply the time by 10.
        finish = start + 10 * ( finish - start );
        sm_PrintTime(_T("\nFast Creation: "),start,finish);

        start = clock();

        for (ii=0; ii<100; ii++) {
            SER(my_test_tsurf_cyl_creation_fast(sContext,pNewBrep, FALSE ));
            delete pNewBrep; pNewBrep=NULL;
        }
        finish = clock();
        finish = start + 10 * ( finish - start );
        sm_PrintTime(_T("Not-Fast Creation: "),start,finish);

        start = clock();
 
        for (ii=0; ii<100; ii++) {
            SER(my_test_tsurf_cyl_creation_2d(sContext,pNewBrep));
            delete pNewBrep; pNewBrep=NULL;
        }
        finish = clock();
        finish = start + 10 * ( finish - start );
        sm_PrintTime(_T("Normal 2D Creation: "),start,finish);

        start = clock();
 
        for (ii=0; ii<100; ii++) {
            SER(my_test_tsurf_cyl_creation_3d(sContext,pNewBrep));
            delete pNewBrep; pNewBrep=NULL;
        }
        finish = clock();
        finish = start + 10 * ( finish - start );
        sm_PrintTime(_T("Normal 3D Creation: "),start,finish);
//        return SM_SUCCESS;
    }

    if (TRUE) {
        SmBrep *pBrep;
        my_test_trimming_tools(sContext,pBrep);
        // dumped in the test routine:    pBrep->Dump();
        SM_ASSERT(pBrep != NULL) ; delete pBrep; pBrep = NULL ;
    }


    if (TRUE) {
        SmStatus sm_status;
        
        SmBSplineSurface *sm_surf = NULL ;
        
        SmContext crContext;
        
        SE(SmBSplineSurface::CreateBilinearSurface(
            crContext,
            SmPoint3d(0,0,0),
            SmPoint3d(1,0,0),
            SmPoint3d(0,1,0),
            SmPoint3d(1,1,0),
            sm_surf));
        
        SmTArray<SmBSplineCurve*> sUVCurves;
        SmTArray<SmOrientType> sCurveOrients;
        
        SmTArray<SmPoint2d> sm_points;
        /*
        // case 1
        sm_points.Add(SmPoint2d(0,0));
        sm_points.Add(SmPoint2d(0.5,0));
        sm_points.Add(SmPoint2d(0.5,0.5));
        sm_points.Add(SmPoint2d(0,0.5));
        */
        // case 2
        sm_points.Add(SmPoint2d(0,0));
        sm_points.Add(SmPoint2d(0,0.5));
        sm_points.Add(SmPoint2d(0.5,0.5));
        sm_points.Add(SmPoint2d(0.5,0));
        
        for (ULONG i=0;i<sm_points.GetSize();i++)
        {
            ULONG i1 = i+1;
            if (i1>=sm_points.GetSize())
                i1 = 0;
            
            SmBSplineCurve * pCurve = NULL ;
            
            SE(SmBSplineCurve::CreateLineSegment(crContext,2,sm_points[i],sm_points[i1],pCurve));
            
            sUVCurves.Add(pCurve);
            sCurveOrients.Add(SM_OT_SAME);
        }
        
        SmBrep *sm_brep = new (crContext) SmBrep;
        SmObjDelete sClean( sm_brep );
        sm_brep->SetTolerance(0.001);
        
        SmTArray<ULONG> sCurveLoops;
        sCurveLoops.Add(sUVCurves.GetSize());
        
        SmTArray<SmPoint3d> sLoopPoints;
        SmRegion *pNewRegion;
        SmShell *pNewShell;
        SmFace *pNewFace;
        
        sm_status = sm_brep->MakeFaceWithCurves(
            sm_brep->GetInfiniteRegion(),
            sCurveLoops,
            NULL,
            &sUVCurves,
            sCurveOrients,
            sLoopPoints,
            sm_surf,
            sm_surf->GetNaturalUVDomain(),
            // case 1
            //� SM_OT_SAME,
            // case 2
            SM_OT_OPPOSITE,
            pNewRegion, pNewShell, pNewFace);

        SM_ASSERT(sm_status==SM_SUCCESS);
        SM_ASSERT_VALID( sm_brep );
    }

    
    if (FALSE) {
        SmBrep *pBrep = new (sContext) SmBrep();
        

        SER(pBrep->ReadFromFile(sContext,_T("blue.sm")));

        SmBSplineCurve *pCurve = NULL ;
        SER(SmBSplineCurve::ReadFromFile(sContext,_T("blueCurve1.sm"),pCurve));
        
        SmPoint3d sReferencePoint(944.604, 270.463, 1057.45);        
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep->Draw();
            smgfx_SetColor(1, 0, 0);
            pCurve->DrawWDeriv(pCurve->GetNaturalInterval(), 0);
            sReferencePoint.Draw();
            sm_GraphicsLoop();
        }
#endif
        
        SE(pBrep->ProjectAndTrim(*pCurve,             // can increment an unused Mark value
                                  SmVector3d(0,0,1),
                                  sReferencePoint,
                                  SM_TT_KEEP_POINT,
                                  NULL,NULL));
        
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            smgfx_SetColor(0, 0, 1);
            smgfx_SetLineWidth(4.0);
            pBrep->Draw();
            sm_GraphicsLoop();
        }
#endif
        return SM_SUCCESS;
    }

    SmBrep *pBrep;
    SE(my_test_tsurf_creation(sContext,pBrep));
    SmTArray<SmCurve*> s3DCurves;
    SmTArray<SmFace*> sFaces;
    SmTArray<SmSurface*> sSurfaces;
    // Create a curve and project it down to the brep
    SmBSplineCurve * pCurve = NULL;
    SmTArray<SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(2,-2,0.5));sPnts.Add(SmPoint3d(3,4,1.0));
    sPnts.Add(SmPoint3d(1,6,1.0));sPnts.Add(SmPoint3d(2,12,0.5));
    pCurve = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts,3,1,1,0);
    SmObjDelete sCU(pCurve);

    SmPoint3d sReferencePoint(-10,0.5,0.5);

#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_SetColor(0, 0, 0);
        pBrep->Draw();
        smgfx_SetColor(1, 0, 0);
        pCurve->DrawWDeriv(pCurve->GetNaturalInterval(), 0);
        sReferencePoint.Draw();
        //    sm_GraphicsLoop();
    }
#endif

    SE(pBrep->ProjectAndTrim(*pCurve,             // can increment an unused Mark value
                             SmVector3d(0,0,-1),
                             sReferencePoint,
                             SM_TT_KEEP_POINT,
                             NULL,NULL));

#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_Erase();
        smgfx_SetColor(0, 0, 1);
        smgfx_SetLineWidth(4.0);
        pBrep->Draw();
        //    sm_GraphicsLoop();
    }
#endif


//    SE(pBrep->CreateParallelProjectionCurves(sContext,*pCurve,
//       SmVector3d(0,0,-1),NULL,NULL,NULL,&s3DCurves,NULL,&sFaces));
//    for (ULONG j=0; j<sFaces.GetSize(); j++) {
//        sSurfaces.Add(sFaces[j]->GetSurface());
//    }

//    SE(pBrep->TrimSurfaces(s3DCurves,sSurfaces,sReferencePoint,SM_TT_DELETE_POINT));
//    SE(pBrep->TrimSurfaces(s3DCurves,sSurfaces,sReferencePoint,SM_TT_KEEP_POINT));
    SM_ASSERT_VALID( pBrep );
    SM_ASSERT(pBrep != NULL) ; delete pBrep; pBrep = NULL ;

    return SM_SUCCESS;

} // end my_test_trim_surfaces

/***********************************************************************
PURPOSE --- Stress test the various MakeFace routines in SmBrep

USAGE NOTES ---  This is a growing collection of tests inspired by the
    Alias connector
***********************************************************************/
PT_EXPORT SmStatus my_test_MakeFace()
{
    SmContext sContext;
    SmTArray<SmSurface*> sSurfaces;
    SmTArray<SmCurve*> sCurves;
    SmTArray<SmBrep*> sBreps;
    SmTArray<long> sTree;
    SmStatus eStat = SM_SUCCESS;

    // Import single surface, make a naturally trimmed face.
    // Surface is not an exact sphere, it has approximate poles
    {
        if (SM_SUCCESS == SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/MakeFace/ApproximateSphere.smp"), sCurves, sSurfaces, sTree, sBreps, SM_ASCII) )
        {
            SM_ASSERT(sSurfaces.GetSize() == 1 && sCurves.GetSize() == 4 && sBreps.GetSize() == 0);

            SmSurface* pSurface = sSurfaces[0];

            SmBrep* pBrep = new (sContext) SmBrep();
            SmFace* pNewFace;
            pBrep->CreateFaceFromSurface(pSurface, pSurface->GetNaturalUVDomain(), pNewFace);

            if (SM_SUCCESS != pBrep->ValidateCounts(1, 1, 2, 0, 0, 1, 2, 2))
            { eStat = SM_ERR; }
        }
        else
        { eStat = SM_ERR; }

    }

    return eStat;
} // end my_test_MakeFace()

/***********************************************************************
PURPOSE --- Stress test the various Brep building algorithms.
    Intended to test imports of non-solid models utilizing
        * MakeFaceWithCurves
        * MakeFacesWithCurves
        * StitchAndOrient
        * ...

USAGE NOTES ---  This is a growing collection of tests inspired by the
    Alias connector
***********************************************************************/
PT_EXPORT SmStatus my_test_brep_import()
{
    SmStatus eStat = SM_SUCCESS;

    if (SM_SUCCESS != my_test_MakeFace() )
    { eStat = SM_ERR; }

    return eStat;
} // end my_test_brep_import
