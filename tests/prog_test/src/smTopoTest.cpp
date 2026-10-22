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
#include <smFaceTest.h>
#include <smTestBrep.h>
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
#include <SmPrimitiveCreation.h>
#include <SmCylinder.h>
#include <SmProtoTopology.h>
#include <SmLine.h>
#include <SmTrackTopologyChanges.h>
#include <NL_Geometry.h>

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
#ifdef SM_DEBUG_CODE
    ++sHealData.m_lPropsAllocCount ; // Match the manually allocated property above.
#endif

    SmTrackTopologyChanges sChanges(pBrep, TRUE) ;
    eRtn = pBrep->SqueezeEdge(pRemovedEdge, pSurvivingVertex) ;
    sChanges.StopTracking() ;

    ULONG lIndex ;
    if(   eRtn != SM_SUCCESS
       || !sChanges.GetRmList()->FindElement(pRemovedEdge, lIndex)
       || pBrep->IsLiveTopologyMember(pRemovedEdge)
       || sHealData.UpdateTargetLists(sChanges, SM_HO_FIX_COIN_VERTICES) != SM_SUCCESS
       || sHealData.m_sTgtEdges.FindElement(pRemovedEdge, lIndex))
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

    SmHealData sHealData ;
    sHealData.m_pBrep = pBrep ;
    if(   sHealData.UpdateTargetLists(sChanges, SM_HO_NONE) != SM_SUCCESS
       || sHealData.m_sTgtVertices.GetSize() != 0
       || sHealData.m_sTgtEdges.GetSize() != 0
       || sHealData.m_sTgtFaces.GetSize() != 0)
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


static SmStatus my_test_separating_closed_edges()
{
  MYPRINTF(_T("\n********** Entered: my_test_separating_closed_edges"));
  for(ULONG lCase=0; lCase<2; ++lCase)
    {
      SmContext sContext;
      SmBrep *pBrep = lCase == 0 ? sm_make_xy_sheet(sContext,20.0) : new(sContext) SmBrep();
      SmObjDelete sCleanup(pBrep);
      pBrep->m_bEditingEnabled = TRUE;
      if(lCase == 1)
        {
          SmPrimitiveCreation sCreate(pBrep->GetInfiniteRegion());
          SER(sCreate.CreateCone
            (4.0,                  // in : height
             2.0,                  // in : base radius
             2.0,                  // in : top radius
             0.0,                  // in : start angle in degrees
             360.0,                // in : end angle in degrees
             SmAxis2Placement())); // in : placement
        }
      SmTArray<SmFace*> sFaces;
      pBrep->GetFaces(sFaces);
      SmFace *pFace = sFaces[0];
      if(lCase == 1)
        for(ULONG lPoint=0; lPoint<sFaces.GetSize(); ++lPoint)
          if(sFaces[lPoint]->GetSurface()->IsPeriodic(sFaces[lPoint]->GetSurface()->GetNaturalUVDomain(),SM_SP_U))
            { pFace = sFaces[lPoint]; break; }
      SmCurve *pCurve = NULL;
      if(lCase == 0)
        pCurve = new(sContext) SmCircle
          (SmPoint3d(10,10,0), // in : center
           SmVector3d(1,0,0),  // in : first axis
           SmVector3d(0,1,0),  // in : second axis
           SmExtent1d(0,360),  // in : angular interval in degrees
           2.0,                // in : radius
           3,                  // in : dimension
           &sContext);         // in : allocation context
      else
        {
          SmExtent2d sDomain = pFace->GetSurface()->GetNaturalUVDomain();
          double dV = (sDomain.GetMin().y+sDomain.GetMax().y)*0.5;
          SmLine sUV
            (SmPoint2d(sDomain.GetMin().x,dV), // in : start point
             SmPoint2d(sDomain.GetMax().x,dV), // in : end point
             2,                                // in : dimension
             &sContext);                       // in : allocation context
          pCurve = new(sContext) SmCrvOnSurf
            (sUV,                  // in : defining UV curve
             *pFace->GetSurface(), // in : base surface
             NULL,                 // in : natural surface domain
             2,                    // in : copy and own both inputs
             &sContext);           // in : allocation context
        }
      SmObjDelete sCurveCleanup(pCurve);
      SmCurveClassification sClass
        (pCurve,                       // in : defining curve
         pCurve->GetNaturalInterval(), // in : curve interval
         NULL,                         // in : no associated UV trim
         pBrep->GetTolerance(),        // in : source zone tolerance
         0,                            // in : no borrowed data capacity
         NULL,                         // in : no borrowed data
         FALSE,                        // in : keep curve ownership with caller
         TRUE,                         // in : curve is current
         pFace->GetSurface());         // in : base surface
      SER(pFace->CurveOnClassify
        (TRUE,   // in : classify nonintersecting curve points
         sClass, // i/o: curve classification
         FALSE,  // in : use standard intersection tolerance
         TRUE,   // in : check 3D intersection distances
         NULL,   // out: curve modification flag not needed
         TRUE)); // in : allow UV classification fallback
      SmTArray<SmFace*> sNewFaces;
      SmTArray<SmEdge*> sNewEdges;
      SmTArray<SmVertex*> sNewVertices;
      SER(sClass.MergeIntervals
        (&sNewFaces,    // out: new faces
         &sNewEdges,    // out: new edges
         &sNewVertices, // out: new vertices
         pFace));       // in : original face
      if(sNewFaces.GetSize() != 1 || pBrep->GetNumFaces() != sFaces.GetSize()+1) { return SM_ERR; }
      SmAssertArray sPointers,sValid;
      if(pBrep->ValidatePointers(&sPointers,SM_LEVEL_2,NULL) != SM_SUCCESS || sPointers.GetSize()
         || !pBrep->AssertValid(&sValid,SM_LEVEL_2,SM_WALK) || sValid.GetSize()) { return SM_ERR; }
    }
  MYPRINTF(_T("\nSeparating closed edges: plane and cylinder passed"));
  return SM_SUCCESS;
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

static SmBrep * sm_combine_protoedge_box(const SmContext & crCtx, double dX, double dY, double dZ)
{
  SmBrep * pBrep = new(crCtx) SmBrep() ;
  SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion()) ;
  SmAxis2Placement sFrame(SmPoint3d(dX, dY, dZ), SmVector3d(1.0, 0.0, 0.0), SmVector3d(0.0, 1.0, 0.0)) ;
  if(sPC.CreateBox(10.0, 10.0, 10.0, sFrame) != SM_SUCCESS)
    {
      delete pBrep ;
      return NULL ;
    }
  return pBrep ;
}

static SmEdgeDefinition * sm_combine_protoedge_ed(const SmContext & crCtx, SmProtoTopologyManager * pMgr,
                                                  SmProtoEdge * pPE, SmFace * pF1, SmFace * pF2)
{
  SmLine * pLine = new(crCtx) SmLine(SmPoint3d(0.0, 0.0, 10.0), SmPoint3d(10.0, 0.0, 10.0), 3, &crCtx) ;
  SmExtent1d sIvl = pLine->GetNaturalInterval() ;
  SmEdgeDefinition * pED = new(pMgr) SmEdgeDefinition(pLine, sIvl, pF1, pF2, 1.0e-6, NULL, NULL, pPE) ;
  pPE->AddEdgeDefinition(pED) ;
  return pED ;
}

static SmBoolean sm_combine_protoedge_pf_lists(SmProtoFace * pPF, SmProtoEdge * pPE)
{
  ULONG lIdx ;
  return pPF->GetCoincProtoEdges()->FindElement(pPE, lIdx) ;
}

/*******************************************************************//**
PURPOSE: CombineProtoEdge moves every EdgeDefinition and the coincident ProtoFace.

NOTES: Two ProtoEdges share the line (0,0,10)-(10,0,10). PE1 has one EdgeDefinition
       and PE2 has three, from different face pairs. A ProtoFace lists PE2 only.
       The range branch then combines, removes, and deletes PE2. Same-source overlaps
       never reach that branch, so this builds the proto topology directly.
***********************************************************************/
static SmStatus my_test_combine_protoedge()
{
  SmContext sCtx ;
  SmBrep * pA = sm_combine_protoedge_box(sCtx, 0.0, 0.0, 0.0) ;
  SmObjDelete sCleanA(pA) ;
  SmBrep * pB = sm_combine_protoedge_box(sCtx, 0.0, -5.0, 0.0) ;
  SmObjDelete sCleanB(pB) ;
  NER(pA) ;
  NER(pB) ;
  SmTArray<SmFace*> sFA ;
  SmTArray<SmFace*> sFB ;
  pA->GetFaces(sFA) ;
  pB->GetFaces(sFB) ;
  if(sFA.GetSize() < 3 || sFB.GetSize() < 3)
    { return SM_ERR ; }

  SmTopologyIntersector sTI(sCtx, pA, pB, 1.0e-6, 0.1) ;
  SmProtoTopologyManager * pMgr = new(pA) SmProtoTopologyManager(pA, pB, &sTI, 1.0e-3, 1.0e-6) ;
  SmObjDelete sCleanMgr(pMgr) ;

  SmProtoVertex * pPV0 = new(pMgr) SmProtoVertex(SmPoint3d(0.0, 0.0, 10.0), 1.0e-6, pMgr) ;
  SmProtoVertex * pPV1 = new(pMgr) SmProtoVertex(SmPoint3d(10.0, 0.0, 10.0), 1.0e-6, pMgr) ;
  pMgr->AddProtoVertex(pPV0) ;
  pMgr->AddProtoVertex(pPV1) ;

  SmExtent1d sIvl(0.0, 1.0) ;
  SmProtoEdge * pPE1 = new(pMgr) SmProtoEdge(pPV0, pPV1, sIvl, pMgr) ;
  SmProtoEdge * pPE2 = new(pMgr) SmProtoEdge(pPV0, pPV1, sIvl, pMgr) ;
  pMgr->AddProtoEdge(pPE1) ;
  pMgr->AddProtoEdge(pPE2) ;
  pPV0->AddProtoEdge(pPE1) ;
  pPV1->AddProtoEdge(pPE1) ;
  pPV0->AddProtoEdge(pPE2) ;
  pPV1->AddProtoEdge(pPE2) ;

  sm_combine_protoedge_ed(sCtx, pMgr, pPE1, sFA[0], sFB[0]) ;
  SmEdgeDefinition * apED2[3] ;
  apED2[0] = sm_combine_protoedge_ed(sCtx, pMgr, pPE2, sFA[1], sFB[0]) ;
  apED2[1] = sm_combine_protoedge_ed(sCtx, pMgr, pPE2, sFA[0], sFB[1]) ;
  apED2[2] = sm_combine_protoedge_ed(sCtx, pMgr, pPE2, sFA[1], sFB[1]) ;

  SmTArray<SmProtoEdge*> sNoPEs ;
  SmProtoFace * pPF = new(pMgr) SmProtoFace(sFA[2], sFB[2], sNoPEs, pMgr) ;
  pMgr->AddProtoFace(pPF) ;
  pPF->RelateCoincidentPF(pPE2) ;

  pPE1->CombineProtoEdge(pPE2) ;

  if(pPE1->GetNumEdgeDefinitions() != 4 || pPE2->GetNumEdgeDefinitions() != 0)
    { return SM_ERR ; }
  for(int ii=0; ii<3; ii++)
    {
      if(apED2[ii]->GetProtoEdge() != pPE1)
        { return SM_ERR ; }
    }
  if(   !sm_combine_protoedge_pf_lists(pPF, pPE1)
     || sm_combine_protoedge_pf_lists(pPF, pPE2)
     || pPE1->GetCoincProtoFaces()->GetSize() != 1)
    { return SM_ERR ; }

  SmProtoEdge * pDeleted = pPE2 ;
  pMgr->RemoveProtoEdge(pPE2) ;
  delete pPE2 ;
  pPE2 = NULL ;
  if(sm_combine_protoedge_pf_lists(pPF, pDeleted))
    { return SM_ERR ; }

  SmTArray<SmEdgeDefinition *> & rEDs = pPE1->GetEdgeDefinitions() ;
  for(ULONG ii=0; ii<rEDs.GetSize(); ii++)
    { rEDs[ii]->Destruct() ; }

  MYPRINTF(_T("\nOKAY: CombineProtoEdge keeps every EdgeDefinition and the ProtoFace link")) ;
  return SM_SUCCESS ;
}

static SmBoolean sm_box_near(const SmExtent3d & crBox, const SmPoint3d & crMin, const SmPoint3d & crMax)
{
  return    crBox.GetMin().DistanceBetween(crMin) < 1.0e-6
         && crBox.GetMax().DistanceBetween(crMax) < 1.0e-6 ;
}

static SmBoolean sm_overlap_pf_lists_live(SmProtoTopologyManager * pMgr, SmProtoFace * pPF)
{
  SmTArray<SmProtoEdge*> * pCoinPEs = pPF->GetCoincProtoEdges() ;
  for(ULONG ii=0; ii<pCoinPEs->GetSize(); ii++)
    {
      ULONG lIdx ;
      if(!pMgr->GetProtoEdges().FindElement((*pCoinPEs)[ii], lIdx))
        { return FALSE ; }
    }
  return TRUE ;
}

/*******************************************************************//**
PURPOSE: ResolveOverlappingProtoEdges refreshes every ProtoFace that lists
         a ProtoEdge of an overlapping group.

NOTES: Three overlapping ProtoEdges lie on the edge (0,0,10)-(10,0,10) of box A,
       which lies in the top face of box B. Both faces of that edge get a ProtoFace
       listing the group. The group is deleted and replaced by a ProtoEdge from the
       edge/face classification; both ProtoFaces must then list only live ProtoEdges.
       The check compares pointers only, so a stale entry never gets dereferenced.
***********************************************************************/
static SmStatus my_test_overlap_group_on_two_protofaces()
{
  SmContext sCtx ;
  SmBrep * pA = sm_combine_protoedge_box(sCtx, 0.0, 0.0, 0.0) ;
  SmObjDelete sCleanA(pA) ;
  SmBrep * pB = sm_combine_protoedge_box(sCtx, 0.0, -5.0, 0.0) ;
  SmObjDelete sCleanB(pB) ;
  NER(pA) ;
  NER(pB) ;

  SmEdge * pEdgeA = NULL ;
  SmTArray<SmEdge*> sEA ;
  pA->GetEdges(sEA) ;
  for(ULONG ii=0; ii<sEA.GetSize(); ii++)
    {
      SmExtent3d sBox ;
      sEA[ii]->CalculateTightBoundingBox(&sBox) ;
      if(sm_box_near(sBox, SmPoint3d(0.0, 0.0, 10.0), SmPoint3d(10.0, 0.0, 10.0)))
        { pEdgeA = sEA[ii] ; }
    }
  SmFace * pTopB = NULL ;
  SmTArray<SmFace*> sFB ;
  pB->GetFaces(sFB) ;
  for(ULONG ii=0; ii<sFB.GetSize(); ii++)
    {
      SmExtent3d sBox ;
      sFB[ii]->CalculateTightBoundingBox(sBox) ;
      if(sm_box_near(sBox, SmPoint3d(0.0, -5.0, 10.0), SmPoint3d(10.0, 5.0, 10.0)))
        { pTopB = sFB[ii] ; }
    }
  NER(pEdgeA) ;
  NER(pTopB) ;
  SmTArray<SmFace*> sEdgeFaces ;
  pEdgeA->GetFaces(sEdgeFaces) ;
  if(sEdgeFaces.GetSize() != 2)
    { return SM_ERR ; }

  SmTopologyIntersector sTI(sCtx, pA, pB, 1.0e-6, 0.1) ;
  SmProtoTopologyManager * pMgr = new(pA) SmProtoTopologyManager(pA, pB, &sTI, 1.0e-3, 1.0e-6) ;
  SmObjDelete sCleanMgr(pMgr) ;

  // overlapping spans along x: [0,10], [0,6], [4,10]
  const double daStart[3] = { 0.0, 0.0,  4.0 } ;
  const double daEnd  [3] = { 10.0, 6.0, 10.0 } ;
  SmProtoEdge * apPE[3] ;
  for(int ii=0; ii<3; ii++)
    {
      SmPoint3d sP0(daStart[ii], 0.0, 10.0) ;
      SmPoint3d sP1(daEnd[ii],   0.0, 10.0) ;
      SmProtoVertex * pPV0 = new(pMgr) SmProtoVertex(sP0, 1.0e-6, pMgr) ;
      SmProtoVertex * pPV1 = new(pMgr) SmProtoVertex(sP1, 1.0e-6, pMgr) ;
      pMgr->AddProtoVertex(pPV0) ;
      pMgr->AddProtoVertex(pPV1) ;

      SmLine * pLine = new(sCtx) SmLine(sP0, sP1, 3, &sCtx) ;
      SmExtent1d sIvl = pLine->GetNaturalInterval() ;
      apPE[ii] = new(pMgr) SmProtoEdge(pPV0, pPV1, sIvl, pMgr) ;
      pMgr->AddProtoEdge(apPE[ii]) ;
      pPV0->AddProtoEdge(apPE[ii]) ;
      pPV1->AddProtoEdge(apPE[ii]) ;
      SmEdgeDefinition * pED = new(pMgr) SmEdgeDefinition(pLine, sIvl, pEdgeA, pTopB, 1.0e-6, NULL, NULL, apPE[ii]) ;
      apPE[ii]->AddEdgeDefinition(pED) ;
    }
  for(int ii=0; ii<3; ii++)
    {
      for(int jj=0; jj<3; jj++)
        {
          if(ii != jj)
            { apPE[ii]->AddOverlappingPE(apPE[jj]) ; }
        }
    }

  SmTArray<SmProtoEdge*> sNoPEs ;
  SmProtoFace * apPF[2] ;
  for(int ii=0; ii<2; ii++)
    {
      apPF[ii] = new(pMgr) SmProtoFace(sEdgeFaces[ii], pTopB, sNoPEs, pMgr) ;
      pMgr->AddProtoFace(apPF[ii]) ;
      for(int jj=0; jj<3; jj++)
        { apPF[ii]->RelateCoincidentPF(apPE[jj]) ; }
    }

  SER(pMgr->ResolveOverlappingProtoEdges()) ;

  for(int ii=0; ii<2; ii++)
    {
      if(!sm_overlap_pf_lists_live(pMgr, apPF[ii]))
        {
          MYPRINTF(_T("\nERROR: ResolveOverlappingProtoEdges left a deleted ProtoEdge on a ProtoFace")) ;
          return SM_ERR ;
        }
    }
  SmTArray<SmProtoEdge*> * pCoin0 = apPF[0]->GetCoincProtoEdges() ;
  SmTArray<SmProtoEdge*> * pCoin1 = apPF[1]->GetCoincProtoEdges() ;
  if(pCoin0->GetSize() == 0 || pCoin0->GetSize() != pCoin1->GetSize())
    { return SM_ERR ; }
  for(ULONG ii=0; ii<pCoin0->GetSize(); ii++)
    {
      ULONG lIdx ;
      if(!pCoin1->FindElement((*pCoin0)[ii], lIdx))
        { return SM_ERR ; }
    }

  SmTArray<SmProtoEdge*> & rPEs = pMgr->GetProtoEdges() ;
  for(ULONG ii=0; ii<rPEs.GetSize(); ii++)
    {
      SmTArray<SmEdgeDefinition *> & rEDs = rPEs[ii]->GetEdgeDefinitions() ;
      for(ULONG jj=0; jj<rEDs.GetSize(); jj++)
        { rEDs[jj]->Destruct() ; }
    }

  MYPRINTF(_T("\nOKAY: ResolveOverlappingProtoEdges refreshes every ProtoFace of an overlapping group")) ;
  return SM_SUCCESS ;
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





// A vertex-only face can connect wire components that survive its deletion.
static SmStatus my_test_delete_vertex_face_with_wires
  (ULONG     lWireCount,           // in : number of separate attached wires
   SmBoolean bSqueeze,             // in : remove the face through SqueezeFace
   SmBoolean bDeleteEdgesVertices) // in : DeleteFace removes newly isolated topology
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
  SER(sBuilder.SetFaceSurface(pPlane,SmExtent2d(SmPoint2d(-1.,-1.),SmPoint2d(5.,1.))));
  for(ULONG lWire=0; lWire<lWireCount; ++lWire)
    { NER(sBuilder.StartAndEndSingleVertexLoop(SmPoint3d(4.*lWire,0.,0.))); }
  SER(sBuilder.EndFace());
  SER(sBuilder.EndShell(FALSE));
  SER(sBuilder.EndBrep());

  SmTemporaryChangeValue<SmBoolean> sEditing(pBrep->m_bEditingEnabled,TRUE);
  SmTArray<SmVertex*> sOriginalVertices, sWireEnds;
  SmTArray<SmEdge*>   sWires;
  pFace->GetVertices(sOriginalVertices);
  for(ULONG lWire=0; lWire<lWireCount; ++lWire)
    {
      SmVertex       *pStart = sOriginalVertices[lWire];
      SmVertex       *pEnd   = NULL;
      SmEdge         *pWire  = NULL;
      SmBSplineCurve *pLine  = NULL;
      SmPoint3d       sEnd   = pStart->GetPoint()+SmVector3d(0.,0.,2.);
      SER(SmBSplineCurve::CreateLineSegment(sContext,3,pStart->GetPoint(),sEnd,pLine));
      SER(pBrep->MakeWireEdgeVertex
        (pBrep->GetInfiniteRegion(),  // in : owning region
         pStart,                      // in : vertex shared with the face
         pLine,                       // in : wire curve
         pLine->GetNaturalInterval(), // in : curve interval
         SM_OT_SAME,                  // in : curve direction
         sEnd,                        // in : new endpoint
         pWire,                       // out: new edge
         pEnd));                      // out: new vertex
      sWires.Add(pWire);
      sWireEnds.Add(pEnd);
    }

  if(bSqueeze)
    {
      SER(pBrep->SqueezeFace
        (pFace,   // in : target face
         NULL,    // in : use the Brep stitching tolerance
         NULL,    // out: deleted vertices (unused)
         NULL,    // out: new edges (unused)
         NULL));  // out: deleted edges (unused)
    }
  else
    {
      SmTArray<SmTopology*> sDeletedTopology;
      SER(pBrep->DeleteFace
        (pFace,                  // in : target face
         bDeleteEdgesVertices,   // in : remove newly isolated topology
         TRUE,                   // in : update regions and shells
         &sDeletedTopology));    // out: deleted edges and vertices
      // The wires existed before deletion and remain attached to the vertices.
      if(sDeletedTopology.GetSize() != 0) { return SM_ERR; }
    }

  if(   pBrep->GetNumFaces() != 0
     || pBrep->GetNumEdges() != lWireCount
     || pBrep->GetNumVertices() != 2*lWireCount) { return SM_ERR; }
  SmTArray<SmShell*> sShells;
  pBrep->GetShells(sShells);
  if(sShells.GetSize() != lWireCount) { return SM_ERR; }
  for(ULONG lWire=0; lWire<lWireCount; ++lWire)
    {
      if(   !pBrep->IsLiveTopologyMember(sWires[lWire])
         || !pBrep->IsLiveTopologyMember(sOriginalVertices[lWire])
         || !pBrep->IsLiveTopologyMember(sWireEnds[lWire])) { return SM_ERR; }
      SmEdgeuse *pEdgeuse = sWires[lWire]->GetPrimaryEdgeuse();
      SmShell   *pShell  = pEdgeuse->GetShell();
      if(   !sWires[lWire]->IsWire()
         || !pShell->IsWireShell()
         || pShell->GetList() == NULL
         || pShell->GetRegion() != pBrep->GetInfiniteRegion()
         || pEdgeuse->GetMate()->GetShell() != pShell
         || sWires[lWire]->GetOtherVertex(sOriginalVertices[lWire]) != sWireEnds[lWire])
        { return SM_ERR; }
    }
  SmAssertArray sReports;
  SER(pBrep->ValidatePointers(&sReports));
  if(sReports.GetSize() || !pBrep->AssertValid(&sReports,SM_LEVEL_2,SM_WALK) || sReports.GetSize())
    { return SM_ERR; }
  return SM_SUCCESS;
}

static SmStatus my_test_delete_face_wire_shell_transition()
{
  // Run every independent test and retain the first failure in either build mode.
  SmStatus sResult = SM_SUCCESS;

  for(ULONG lWireCount=1; lWireCount<=2; ++lWireCount)
    {
      PT_RUN(sResult, my_test_delete_vertex_face_with_wires(lWireCount,FALSE,FALSE));
      PT_RUN(sResult, my_test_delete_vertex_face_with_wires(lWireCount,FALSE,TRUE));
      PT_RUN(sResult, my_test_delete_vertex_face_with_wires(lWireCount,TRUE,FALSE));
    }
  return sResult;
}


static SmStatus my_test_torus_seam_insertion_with_vertex_loops()
{
  MYPRINTF(_T("\n********** Entered: my_test_torus_seam_insertion_with_vertex_loops"));
  for(ULONG lCase=0; lCase<20; ++lCase)
    {
      SmContext sContext;
      SmObjDelete sCleanup; // Destroy seam classifications before their supporting brep.
      SmTorusVertexLoopTestCase sCase;
      SER(sm_MakeTorusVertexLoopCase(sContext,lCase,sCase));
      sCleanup.SetObj(sCase.m_pBrep);
      SER(sm_InsertTorusVertexLoopSeams(sContext,lCase,sCase));
    }
  MYPRINTF(_T("\nDirect torus seam insertion: twenty cases passed"));
  return SM_SUCCESS;
}

SmStatus my_test_topology()
{
  // Run every independent test and retain the first failure in either build mode.
  SmStatus sResult = SM_SUCCESS;

  SmContext sContext;
  SmBrep   * pBrep = new(sContext) SmBrep();
  SmShell  * pNewShell;
  SmVertex * pNewVertex;

  MYPRINTF(_T("\n\n"));
  MYPRINTF(_T("********** Entered: my_test_topology"));

  PT_RUN(sResult, my_test_faces());
  PT_RUN(sResult, my_test_torus_seam_insertion_with_vertex_loops());

  PT_RUN(sResult, my_test_delete_face_wire_shell_transition());
  PT_RUN(sResult, my_test_separating_closed_edges());

  PT_RUN(sResult, my_test_stale_edgeuse_owner());
  PT_RUN(sResult, my_test_validate_required_pointers());
  PT_RUN(sResult, my_test_squeeze_edge_tracks_removal());
  PT_RUN(sResult, my_test_tracks_changed_objects());
  PT_RUN(sResult, my_test_tracks_nested_geometry_and_resets());
  PT_RUN(sResult, my_test_coaxial_cylinder_merge());
  PT_RUN(sResult, my_test_combine_protoedge());
  PT_RUN(sResult, my_test_overlap_group_on_two_protofaces());
  PT_RUN(sResult, my_test_coaxial_merge_domain());
  PT_RUN(sResult, my_test_surface_cover_domains());
  PT_RUN(sResult, my_test_edge_geometry_repairs());
  PT_RUN(sResult, my_test_reversed_line_join());
  PT_RUN(sResult, my_test_coaxial_merge_orientation());

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
  return sResult;

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
