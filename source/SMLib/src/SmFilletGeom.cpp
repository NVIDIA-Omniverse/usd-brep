// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFilletGeom.cpp
* PURPOSE: Source file for SmFilletGeom object.
**********************************************************************/

#include "StdAfx.h"

#ifndef __SMFILLETGEOM_H__
#include <SmFilletGeom.h>
#endif

#ifndef __SMFILLETSOLVER_H__
#include <SmFilletSolver.h>
#endif

#ifndef __SMFILLETCORNER_H__
#include <SmFilletCorner.h>
#endif

#ifndef __SMFILLETINTERSECTOR_H__
#include <SmFilletIntersector.h>
#endif
// Remove Composites
// #ifndef __SMCEDGE_H__
// #include <SmCEdge.h>
// #endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#include <SmCone.h>
#include <SmPlane.h>
#include <SmLine.h>
#include <SmTrimmingTools.h>
#include <SmTopologyTraverser.h>
#include <SmAssertArray.h>
#include <SmAttribute.h>
#include <SmGeomUtility.h>
#include <nurbs.h>

#ifdef SM_GFX_CODE
  #include <SmGraphicsOutput.h>
#endif // SM_GFX_CODE


/*******************************************************************//**
    Static functions
***********************************************************************/

#if 0
/*******************************************************************//**
PURPOSE: Local debugging convenience: collect the FilletVertexuses from a FilletGeom.

NOTES: The passed-in array must be sized at least 4.
***********************************************************************/
static void sm_GetFVUs( SmFilletGeom *pFG, SmFilletVertexuse * apFVUs[4] )
{
  if ( pFG==NULL ) { return; }
  apFVUs[0] = pFG->GetRailVertexuse( 0,0 );
  apFVUs[1] = pFG->GetRailVertexuse( 0,1 );
  apFVUs[2] = pFG->GetRailVertexuse( 1,0 );
  apFVUs[3] = pFG->GetRailVertexuse( 1,1 );
  return;
} // end static sm_GetVUs
#endif

/*******************************************************************//**
PURPOSE: Evaluate point geometry of a vertex of type SM_FV_MATE

NOTES:  When given UV point is inside face->Surface->NaturalDomain
     face->Surface is used to calculate Point else use
     face->ExtendedSurface()
***********************************************************************/
static SmStatus sm_EvaluateMate
  (SmFace * pFace,            // in : face attached to vertexMate
   const SmPoint2d & crUV,    // in : UV point to evaluate
   SmPoint3d & rPnt)          // out: point = face->Surface(uvPoint) or
                              //              face->ExtendedSurface(uvPoint) as needed
{
    SmBSplineSurface * pSurface = SM_CAST_PTR(SmBSplineSurface,pFace->GetSurface());
    NER(pSurface);
    SmExtent2d sNaturalUVDomain = pSurface->GetNaturalUVDomain();
    if (sNaturalUVDomain.ContainsPoint2d(crUV))
      {
        SER(pSurface->EvaluatePoint(crUV,rPnt));
      }
    else
      {
        SER(pSurface->EvaluatePointOnExtendedSurface(crUV,rPnt));
      }

    return SM_SUCCESS;

} // end sm_EvaluateMate

/*******************************************************************//**
PURPOSE: Drop a point onto a surface. Tolerance may be adjusted when trying

NOTES:
  This routine is intended for target points that are actually on
  the surface, to within a tolerance.  (It uses SM_SO_INTERSECT and
  not SM_SO_MINIMIZE.)  So it will fail if the point is too far away.

  If this fails to find a solution, it will return SM_ERR, so don't
  use SER on it unless its failure warrants dropping what you're doing.

***********************************************************************/
static SmStatus sm_SurfaceDropPoint
  (SmSurface * pSurface,            // in : target surface
   SmPoint3d & rPnt,                // in : target point
   double dApproxTol,               // in :
   SmSolutionArray & rSolutions,    // out:
   int iDebugLevel)
{
  SmSurfaceCache *pSC1 = smsurf_GetSurfaceCache(pSurface); NER(pSC1);
  SmCacheCheckOutIn sCheckIO(pSC1);


#ifdef SM_DEBUG_CODE
  // draw Brep(blue), Surface(green), face(black), point(red)
  if( iDebugLevel > 0 )
    {
      SmFace *pFace = (SmFace *)pSurface->GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; pSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 1,0,0) ; rPnt.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  { // Makes the change values destructors occur before surface is checked back in
    // and possibly deleted.

    // Turn off point testing so GlobalPointSolve() will keep all point solutions
    //   without classifying the solution point against the trim boundaries.
    // Turn on boundary curve processing to force LocalSolve to look for drop points
    //   on boundary curves. This will find solutions where the curve comes close
    //   to the surface but does not actually intersect it.
    SmTemporaryChangeValue<SmBoolean> sStack1(pSC1->m_bPointTestEnabled,FALSE);
    SmTemporaryChangeValue<SmBoolean> sStack3(pSC1->m_bProcessBoundaryCurves,TRUE);

    rSolutions.ReSet();
    SmExtent2d sDomain = pSurface->GetNaturalUVDomain();

    // for a sequence of intersection tolerance distances
    double dDroppingTol = dApproxTol/100.0 ;
    for (ULONG i=0; i<5; i++)
      {
        // intersect the point with the surface
        SER( pSurface->GlobalPointSolve(sDomain,SM_SO_INTERSECT,
            rPnt,dDroppingTol,NULL,SM_SR_ALL,rSolutions ));
        if (rSolutions.GetSize() > 0)
          { break; }
        dDroppingTol *= 10.0;
      }
    if (rSolutions.GetSize() > 0)
      {

#ifdef SM_DEBUG_CODE
  // draw the dropped result (cyan)
  if( iDebugLevel > 0 )
    {
      SmPoint2d sUV( rSolutions[0].m_vStart[0], rSolutions[0].m_vStart[1] );
      SmPoint3d sResult;
      pSurface->EvaluatePoint( sUV, sResult );
      smgfx_SetLook(4,6, 0,1,1); sResult.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

        return SM_SUCCESS;
      }
  }

  // Unable to drop point
  return SM_ERR;

} // end sm_SurfaceDropPoint

/*******************************************************************//**
PURPOSE: Compute surface-surface intersection given a known start point,
            a start direction, and optionally an end point as well

NOTES:
  Note: if this fails to find a solution, it will return SM_ERR.
  So don't use SER on a call to this unless its failure warrants quitting.

***********************************************************************/
static SmStatus sm_TwoPntsIntersection
  (const SmContext & crContext,     // in : context for new object construction
   SmVertex        * pStartV,       // in : 1st point known to be on intersection curve
   SmVertex        * pOptEndV,      // in : 2nd point known to be on intersection curve,
                                    //      NULL to ignore
   SmVector3d      * pDirection,    // in (required) : expected general direction of intersection curve from 1st point
   SmVector3d      * pOptEndDirection, // in : expected intersection curve end
   SmSurface       * pSurf1,        // in : 1st intersecting surface
   SmSurface       * pSurf2,        // in : 2nd intersecting surface
   double            d3DTol,        // in : caller-prescribed 3d tolerance
   SmBSplineCurve *& rp3DCurve,     // out: 3d intersection curve
   SmBSplineCurve *& rpUVCurve1,    // out: associated UVTrimCurve on 1st surface
   SmBSplineCurve *& rpUVCurve2,    // out: associated UVTrimCurve on 1st surface
   int               iDebugLevel)
{
  // check input
  NER(pStartV);
  NER(pDirection);
  NER(pSurf1);
  NER(pSurf2);

  // init outputs
  rp3DCurve  = NULL;
  rpUVCurve1 = NULL;
  rpUVCurve2 = NULL;

#ifdef SM_DEBUG_CODE
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  // draw surfaces and end Points
  if ( iDebugLevel > 0  || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)pSurf1->GetFace() ;
      SmFace *pFace2 = (SmFace *)pSurf2->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE); } sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE); } sm_GraphicsLoop();

      smgfx_SetLook(1,2, 1,0,1) ; pSurf1->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0) ; pSurf2->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH); } sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH); } sm_GraphicsLoop();
      smgfx_SetLook(4,8, 1,0,0) ; pStartV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(4,8, 0,0,1) ; if (pOptEndV) { pOptEndV->Draw(); } sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // surface size locals
  SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
  SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();

  // Note: this used to calculate a separate tolerance for SSI (dSSITol).
  // That leads to inconsistencies however.  We should use the same
  // tolerance that the caller prescribes.
  // [bd; 060105, at rad==0.2, tol == Brep tol]
  //
  // SmExtent3d sBBox1, sBBox2;
  // SER(pSurf1->CalculateBoundingBox(sDomain1,&sBBox1));
  // SER(pSurf2->CalculateBoundingBox(sDomain2,&sBBox2));
  // double d1 = sBBox1.GetSize().GetMaxDimension();
  // double d2 = sBBox2.GetSize().GetMaxDimension();
  // double dSSITol = SM_ZONE_TOL_3D * smos_Min( d1, d2 );

  double dSSITol      = d3DTol;  // use caller-specified tol for SSI.
  double dAngleTol    = SM_DEG2RAD(30);
  double dDroppingTol = dSSITol;

  // Drop start point onto surf1 - watch for closed surfaces
  SmPoint3d sStartPnt = pStartV->GetPoint();
  SmTArray<SmPoint2d> sSurface1Points;
  SmTArray<SmPoint2d> sSurface2Points;
  SmSolution sData[16];
  SmSolutionArray sSolutions(16,sData);
  SmStatus eStat = sm_SurfaceDropPoint( pSurf1, sStartPnt, dDroppingTol, sSolutions, iDebugLevel );
  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
    {
      if ( eStat == SM_SUCCESS ) { eStat = SM_ERR_BAD_INTERSECTIONS; }
      SER_MSG( eStat, _T("sm_TwoPntsIntersection(): Suffered a sm_SurfaceDropPoint() failure"));
    }
  ULONG lIndex11 = 0;
  // Please note that it is possible that this point lies on the seam
  // of a closed surface.
  if (sSolutions.GetSize() == 2)
    {
      // Closed surface
      SmPoint2d sUV(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
      SmVector2d sUVDir;
      SER(pSurf1->DropVectors(sUV,TRUE,TRUE,1,pDirection,&sUVDir));
      SmPoint2d sTestUV = sUV+sUVDir;
      if (   (pSurf1->IsClosed(sDomain1,SM_SP_U) && sUVDir.x < 0.0)
          || (pSurf1->IsClosed(sDomain1,SM_SP_V) && sUVDir.y < 0.0 ))
        {
          lIndex11 = 1;
        }
    } // end 2 solutions for closed surfaces check

  SmPoint2d sUV11(sSolutions[lIndex11].m_vStart[0],sSolutions[lIndex11].m_vStart[1]);

  // Drop start point onto surf2 - watch for closed surfaces
  eStat = sm_SurfaceDropPoint( pSurf2, sStartPnt, dDroppingTol, sSolutions, iDebugLevel );
  if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
    {
      if ( eStat == SM_SUCCESS ) { eStat = SM_ERR_BAD_INTERSECTIONS; }
      SER_MSG( eStat, _T("sm_TwoPntsIntersection(): Suffered a sm_SurfaceDropPoint() failure"));
    }
  ULONG lIndex21 = 0;
  // Please note that it is possible that this point lies on the seam
  // of a closed surface.
  if (sSolutions.GetSize() == 2)
    {
      // Closed surface
      SmPoint2d sUV(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
      SmVector2d sUVDir;
      SER(pSurf2->DropVectors(sUV,TRUE,TRUE,1,pDirection,&sUVDir));
      SmPoint2d sTestUV = sUV+sUVDir;
      if (   (pSurf2->IsClosed(sDomain2,SM_SP_U) && sUVDir.x < 0.0)
          || (pSurf2->IsClosed(sDomain2,SM_SP_V) && sUVDir.y < 0.0 ))
        {
          lIndex21 = 1;
        }
    }
  SmPoint2d sUV21(sSolutions[lIndex21].m_vStart[0],sSolutions[lIndex21].m_vStart[1]);

  // add start points into surfacePoint arrays
  sSurface1Points.Add(sUV11);
  sSurface2Points.Add(sUV21);

  // when given a stop point - drop it onto the surfaces
  if (pOptEndV != NULL)
    {
      // Drop end point onto surf1 - watch for closed surfaces
      SmPoint3d sEndPnt = pOptEndV->GetPoint();
      eStat = sm_SurfaceDropPoint(pSurf1,sEndPnt,dDroppingTol,sSolutions,iDebugLevel );
      if ( eStat != SM_SUCCESS )
        { SER( eStat ); }
      ULONG lIndex12 = 0;
      if (sSolutions.GetSize() == 2)
        {
          // Closed surface
          for (ULONG ii=0; ii<2; ii++)
            {
              SmPoint2d sUV(sSolutions[ii].m_vStart[0],sSolutions[ii].m_vStart[1]);
              SmVector2d sUVDir;
              SER(pSurf1->DropVectors(sUV,TRUE,TRUE,1,pDirection,&sUVDir));
              if (sUVDir.Dot(sUV-sUV11) < 0.0)
                {
                  lIndex12 = 1;
                  break;
                }
            }
        }
      SmPoint2d sUV12(sSolutions[lIndex12].m_vStart[0],sSolutions[lIndex12].m_vStart[1]);


      // Drop end point onto surf2 - watch for closed surfaces
      SER(sm_SurfaceDropPoint(pSurf2,sEndPnt,dDroppingTol,sSolutions,iDebugLevel));
      ULONG lIndex22 = 0;
      if (sSolutions.GetSize() == 2)
        {
          // Closed surface
          for (ULONG ii=0; ii<2; ii++)
            {
              SmPoint2d sUV(sSolutions[ii].m_vStart[0],sSolutions[ii].m_vStart[1]);
              SmVector2d sUVDir;
              SER(pSurf2->DropVectors(sUV,TRUE,TRUE,1,pDirection,&sUVDir));
              if (sUVDir.Dot(sUV-sUV21) < 0.0)
                {
                  lIndex22 = 1;
                }
            }
        }
      SmPoint2d sUV22(sSolutions[lIndex22].m_vStart[0],sSolutions[lIndex22].m_vStart[1]);

      // add dropped endPoints to SurfacePoint arrays
      sSurface1Points.Add(sUV12);
      sSurface2Points.Add(sUV22);

    } // end pOptEndV existence check

  // This used to loop up to five times, doubling the tolerance each time,
  // hoping for a success.  That leads to inconsistencies however.
  // For example two points that the caller considers to be distinct
  // (at dSSITol) can be classified as coincident at 32 * dSSITol.
  // [bd; 060105, at rad==0.2, tol == Brep tol]

  double dDeviation = 0;
  SmTsectCurveType eCurveType;
  SER(pDirection->Unitize());

  SER( pSurf1->PointBasedSurfaceIntersect(crContext,
                                          sDomain1,
                                          *pSurf2,
                                          sDomain2,
                                          sSurface1Points,
                                          sSurface2Points,
                                          FALSE,
                                          FALSE,
                                          pDirection,
                                          pOptEndDirection,
                                          SM_CAST_APPROXTOL3D_PTR(&dSSITol),
                                          &dAngleTol,
                                          rp3DCurve,
                                          rpUVCurve1,
                                          rpUVCurve2,
                                          eCurveType,
                                          dDeviation));

  return SM_SUCCESS;

} // end sm_TwoPntsIntersection

/*******************************************************************//**
PURPOSE: Determine where the guess UV's are for 'PointOnPlaneSolve'

NOTES:
***********************************************************************/
static SmStatus sm_FindStepoffUV
  (SmFilletSolver * pFilSolver,
   double dCurveParam,     // in : must be a a param in the pFilSolver->GetEdgeuse(0 or 1)->GetInterval()
   double dStepoffDist,
   SmVector2d sUVs[2])
{
    for (ULONG i=0; i<2; i++) {
        SmEdgeuse * pEdgeuse = pFilSolver->GetEdgeuse(i);
        SmSurface *pSurface = pEdgeuse->GetFace()->GetSurface();
        // Step off the edge along the binormal direction
        SmPoint3d sSurfPnt;
        SmVector3d sBinVec;
        SER(pEdgeuse->EvaluateBinormalStepOff(dCurveParam,
            smos_Fabs(dStepoffDist),sSurfPnt,sBinVec));
        double dTol = pFilSolver->GetThisApproxTol3d();
        SmSolution sData[64];
        SmSolutionArray sSolutions(64,sData);
        // Make the surface think that it is a standalone surface
        SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pSurface); NER(pSC);
        SmCacheCheckOutIn sCheckIO(pSC);
        {
          // Make the surface think that it is a standalone surface.
          // Turn on boundary curve processing to force LocalSolve to look for drop points
          //   on boundary curves. This will find solutions where the curve comes close
          //   to the surface but does not actually intersect it.
          SmTemporaryChangeValue<SmBoolean> sChange1(pSC->m_bPointTestEnabled,FALSE);
          SmTemporaryChangeValue<SmBoolean> sChange2(pSC->m_bProcessBoundaryCurves,TRUE);
          SER(pSurface->GlobalPointSolve(pEdgeuse->GetFace()->GetUVDomain(),
              SM_SO_MINIMIZE,sSurfPnt,dTol,NULL,
              SM_SR_SINGLE,sSolutions));
        }
        if (sSolutions.GetSize() < 1) SER(SM_ERR);
        sUVs[i] = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
#ifdef SM_DEBUG_CODE
        if ( pFilSolver->DebugLevel() > 0 ) {
            smgfx_SetLook(2,4, 0,0,1); pEdgeuse->Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,6, 1,0,0); sSurfPnt.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(3,6, 0,1,0); sBinVec.Draw(&sSurfPnt); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    }

    return SM_SUCCESS;

} // end sm_FindStepoffUV

/*******************************************************************//**
PURPOSE: Decide which of two Faces contains a 3d point.

NOTES: This is like SmFace::Point3DClassify() but optimized for our situation:
   The two Faces were originally a single Face that was split.
   We call that SmFace method if we can't get an easier solution.
   The point might be on the splitting Edge, in which case we return both.
   Returns:
     0: Neither Face
     1: in pOrigFace only
     2: in pNewFace only
     3: in both Faces.
***********************************************************************/
static int sm_ClassifyFacePoint( const SmPoint3d & rPos, SmFace *pOrigFace, SmFace *pNewFace )
{
  SmBoolean bSuccess1 = FALSE, bSuccess2 = FALSE;
  SmPoint2d sUV1(0,0), sUV2(0,0);
  double dGap1 = 0.0, dGap2 = 0.0;
  SmBoolean bIsMulti;

  SmExtent2d sFaceDomain1 = pOrigFace->GetUVDomain();

//cbi_CEdge TODO: Maybe do this ahead of time.
  pOrigFace->CalculateUVDomainFromUVTrimCurves( sFaceDomain1 );

  // Use Normalize: it should be over the Face it's in, and don't bother
  // finding the closest approach if it's not.

  SmStatus eStat1 = pOrigFace->GetSurface()->DropPoint( rPos,
              sFaceDomain1, NULL, bSuccess1, sUV1, dGap1, bIsMulti, SM_SO_NORMALIZE );

  if ( bSuccess1 )
  {
      if ( ! sFaceDomain1.ContainsPoint2d( sUV1 ) )
        { bSuccess1 = FALSE; }
  }

  SmExtent2d sFaceDomain2 = pNewFace->GetUVDomain();

  pNewFace->CalculateUVDomainFromUVTrimCurves( sFaceDomain2 );

  SmStatus eStat2 = pNewFace->GetSurface()->DropPoint( rPos,
              sFaceDomain2, NULL, bSuccess2, sUV2, dGap2, bIsMulti, SM_SO_NORMALIZE );

  if ( bSuccess2 )
  {
      if ( ! sFaceDomain2.ContainsPoint2d( sUV2 ) )
        { bSuccess2 = FALSE; }
  }

  // Best situation: the point drops to one Face and not the other.
  if ( eStat1 == SM_SUCCESS  &&  eStat2 != SM_SUCCESS )
    { return 1; }
  if ( bSuccess1  &&  ! bSuccess2 )
    { return 1; }

  if ( eStat2 == SM_SUCCESS  &&  eStat1 != SM_SUCCESS )
    { return 2; }
  if ( bSuccess2  &&  ! bSuccess1 )
    { return 2; }

  // Ok, it didn't drop to one and not the other.
  // Note, here, eStat1 == eStat2, and bSuccess1 == bSuccess2.
  // If both succeeded, check gaps.
  if ( bSuccess1 ) // Note, both flags are the same at this point.
  {
      // Note, if our Rail Edge borders both Faces (i.e., already split),
      // then both gaps will be small.  Switch only if one is definitely bigger.
      if ( dGap2 > 0.0  &&  dGap1 > dGap2 * 20.0 )
        { return 2; }
      if ( dGap1 > 0.0  &&  dGap2 > dGap1 * 20.0 )
        { return 1; }
  }

  // If we're still here, we'll have to do the more expensive point-in-Face test.
  // If the drops succeeded, then we have a good uv value.
  SmPoint2d *pGuessUV = ( bSuccess1 ) ? &sUV1 : NULL;
  SmZoneTol3d sTol = SmTol::GetZoneTol3d( pOrigFace );
  SmPointClassification eClassif1, eClassif2;
  eStat1 = pOrigFace->Point3DClassify( rPos,
                              sTol,
                              TRUE,      // Check boundaries first
                              eClassif1, // out
                              pGuessUV,
                              FALSE,     // Target Loops only?
                              NULL,      // Target Loops
                              TRUE );    // Okay to work in uv space.

  bSuccess1 = ( eClassif1.GetPointClass() != SM_PC_UNKNOWN );

  pGuessUV = ( bSuccess2 ) ? &sUV2 : NULL;
  eStat2 = pNewFace->Point3DClassify( rPos,
                              sTol,
                              TRUE,      // Check boundaries first
                              eClassif2, // out
                              pGuessUV,
                              FALSE,     // Target Loops only?
                              NULL,      // Target Loops
                              TRUE );    // Okay to work in uv space.

  bSuccess2 = ( eClassif2.GetPointClass() != SM_PC_UNKNOWN );

  if ( eStat1 == SM_SUCCESS  &&  eStat2 != SM_SUCCESS )
    { return 1; }
  if ( bSuccess1  &&  ! bSuccess2 )
    { return 1; }

  if ( eStat2 == SM_SUCCESS  &&  eStat1 != SM_SUCCESS )
    { return 2; }
  if ( bSuccess2  &&  ! bSuccess1 )
    { return 2; }

  // Here again, bSuccess1 == bSuccess2.
  if ( bSuccess1 )
    { return 3; }

  // Hmm, this shouldn't happen: The point was presumably in the original Face before
  // it was split into pOrigFace and pNewFace.  Maybe put a breakpoint here.
  //WARN( _T("  *** Problem: Failure in sm_ClassifyFacePoint()\n" ));

  // This does happen: both DropPoint() succeed, both PointClassify() fail.
  // Point is on the surface, and within both Face domains, but it's not actually
  // within the boundaries of either Face (which means it was not within the boundaries
  // of the original pre-split Face).  [Fillet Reg 231]
  // Let the caller deal with it, they know more than we do.

  return 0;

} // end static sm_ClassifyFacePoint

/*******************************************************************//**
PURPOSE: Find the intersection curve of two surfaces.

NOTES:
   Try two-point-intersector first and then the global
   solver if the first method failed.  All parameters are needed
   except pEndV which can be NULL when it's not available
***********************************************************************/
SmStatus sm_SrfSrfIntersection
  (const SmContext & crContext,             // in : context for new object construction
   SmVertex        * pStartV,               // in : 1st point known to be on intersection curve
   SmVertex        * pEndV,                 // in : 2nd point known to be on intersection curve,
                                            //      NULL to ignore
   SmVector3d      * pOptDir,               // in : expected general direction of intersection curve from 1st point
   SmVector3d      * pOptEndDir,            // in   expected intersection end direction
   SmSurface       * pSurf1,                // in : 1st intersecting surface
   SmSurface       * pSurf2,                // in : 2nd intersecting surface
   SmApproxTol3d     dApproxTol,            // in : max allowed distance between xSect Curve and surfaces
   double            dAngleTol,             // in : max allowed angle between consecutive xSect curve segment tangents
   SmBSplineCurve *& rp3DCurve,             // out: 3d intersection curve
   SmBSplineCurve *& rpUVCurve1,            // out: associated UVTrimCurve on 1st surface
   SmBSplineCurve *& rpUVCurve2,            // out: associated UVTrimCurve on 2nd surface
   SmBoolean   bOptSkipTwoPntsIntersection, // in : TRUE = skip trying cheap sm_TwoPntsIntersection()
                                            //             before moving onto expensive general surf/surf xSect solver
                                            //      FALSE= try cheap sm_TwoPntsIntersection() before
                                            //             resorting to expensive general surf/surf xSect solver
   SmPoint3d       * pOptRefPoint,          // In : In case of multiple intersection curves, if neither pStartV
                                            //      nor pEndV is given, used to select the correct curve.
                                            //      Default: NULL
   int               iDebugLevel)           // in : iDebugLevel, <= 0 = No Debug output
{
  // check inputs
  NER(pSurf1);
  NER(pSurf2);

  // init outputs
  rp3DCurve  = NULL;
  rpUVCurve1 = NULL;
  rpUVCurve2 = NULL;

#ifdef SM_DEBUG_CODE
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  // draw surfaces and end Points
  if ( iDebugLevel > 0  || lCount == lDebugCount)
    {
      SmFace *pFace1 = (SmFace *)pSurf1->GetFace() ;
      SmFace *pFace2 = (SmFace *)pSurf2->GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE); } sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE); } sm_GraphicsLoop();

      smgfx_SetLook(1,2, 0,1,1) ; pSurf1->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0) ; pSurf2->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH); } sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH); } sm_GraphicsLoop();
      smgfx_SetLook(4,8, 1,0,0) ; if (pStartV) { pStartV->Draw(); } sm_GraphicsLoop();
      smgfx_SetLook(4,8, 0,0,1) ; if (pEndV) { pEndV->Draw(); } sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // surface and surf/surf intersection locals
  SmBoolean bNeedsMoreIntersections;
  SmTArray<SmCurve*> s3DCurves;
  SmTArray<SmCurve*> sSurface1UVCurves;
  SmTArray<SmCurve*> sSurface2UVCurves;

  SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
  SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();

  // Note: this used to calculate a separate tolerance for SSI.
  // That leads to inconsistencies however.  We should use the same
  // tolerance that the caller prescribes.
  // [bd; 060105, at rad==0.2, tol == Brep tol]
  //
  // // set tolerance to SM_ZONE_TOL_3D * smaller bounding box dimension
  // SmExtent3d sBBox1, sBBox2;
  // SER( pSurf1->CalculateBoundingBox( sDomain1, &sBBox1 ));
  // SER( pSurf2->CalculateBoundingBox( sDomain2, &sBBox2 ));
  // double d1 = sBBox1.GetSize().GetMaxDimension();
  // double d2 = sBBox2.GetSize().GetMaxDimension();
  // double sSSITol = SM_ZONE_TOL_3D * smos_Min( d1, d2 );

  SmApproxTol3d sSSITol = dApproxTol;

  // find intersection endPoints from naturalBoundary/Surface intersections
  SmBoolean bUseSurfaceEdges[2] = { TRUE, TRUE };

  // See if we have analytic cases
  if (pSurf1->GetType() == SmPlane_TYPE)
    {
      // get planar intersection
      SmPlane * pPlane = SM_CAST_PTR(SmPlane,pSurf1); NER(pPlane);
      pSurf2->IntersectWithPlane(crContext, sDomain2,
                                 *pPlane, sDomain1, bUseSurfaceEdges,
                                 &sSSITol, &dAngleTol, bNeedsMoreIntersections,
                                 &s3DCurves, &sSurface2UVCurves, &sSurface1UVCurves,
                                 NULL,NULL);
    }
  else if (pSurf2->GetType() == SmPlane_TYPE)
    {
      // get planar intersection
      SmPlane * pPlane = SM_CAST_PTR(SmPlane,pSurf2); NER(pPlane);
      pSurf1->IntersectWithPlane(crContext,sDomain1,
                                 *pPlane,sDomain2, bUseSurfaceEdges,
                                 &sSSITol, &dAngleTol, bNeedsMoreIntersections,
                                 &s3DCurves, &sSurface1UVCurves, &sSurface2UVCurves,
                                 NULL, NULL);
    }

  // Use two-pts intersector.
  // Note, pOptDir is a required argument to sm_TwoPntsIntersection().

  if (   s3DCurves.GetSize() == 0        //  when analytic intersectors gave no answer
      && !bOptSkipTwoPntsIntersection    //  and  asked
      && pStartV                         //  and  given a startPoint
      && pOptDir)                        //  and  given a startDirection
    {
      // get two-pt intersection
      // Note, an error return value here just means
      // that it couldn't find a solution, not that we should quit,
      // so don't SER here.
     sm_TwoPntsIntersection( crContext,
              pStartV, pEndV, pOptDir, pOptEndDir, pSurf1, pSurf2, sSSITol,
              rp3DCurve, rpUVCurve1, rpUVCurve2, iDebugLevel );

      // If an intersection was found, AND both start and end points
      // were given, check that both ends of the solution curve
      // are within 100*tol of the given start and end points.

      if (rp3DCurve != NULL)
        {
          // skip cases where pEndV was not given
          if (pEndV == NULL)
            { return SM_SUCCESS; }

          // Check start and end points vs. given points.
          SmPoint3d sP1, sP2;
          SmExtent1d sTrimIvl = rp3DCurve->GetNaturalInterval();
          SER(rp3DCurve->EvaluatePoint(sTrimIvl.GetMin(),sP1));
          SER(rp3DCurve->EvaluatePoint(sTrimIvl.GetMax(),sP2));

          if (   sP1.DistanceBetween( pStartV->GetPoint() ) < 100.0*dApproxTol
              && sP2.DistanceBetween( pEndV->GetPoint() )   < 100.0*dApproxTol)
            {
              // return success
              return SM_SUCCESS;
            }

          // Got a result, but apparently not the whole curve:
          //   clear output and move intersection curve
          //   to intersectionCurve array
          s3DCurves.Add(rp3DCurve);          rp3DCurve = NULL;
          sSurface1UVCurves.Add(rpUVCurve1); rpUVCurve1 = NULL;
          sSurface2UVCurves.Add(rpUVCurve2); rpUVCurve2 = NULL;

        } // end found intersection check
    } // end try two-pts intersector check


  // Arrive here if the planar intersectors did generate an answer,
  // or they didn't and the two-point intersector failed to generate
  // a satisfactory answer.


  // when planar intersectors did not generate an answer, and the two-point
  // intersector didn't find anything -- general surf/surf intersector
  if ( s3DCurves.GetSize() == 0 )
    {
      // gwc: switched to global intersection call - it does the same thing
      //      as current code but with more debugging tools available.
      SmBoolean bUseSrfEdges[2] = { TRUE, TRUE }; 

#ifndef SM_VALIDATE_INTERSECTORS
      SER(pSurf1->GlobalSurfaceIntersect
                    (crContext,
                     sDomain1,
                     *pSurf2,
                     sDomain2,
                     bUseSrfEdges,
                     &sSSITol,
                     &dAngleTol,
                     &s3DCurves,
                     &sSurface1UVCurves,
                     &sSurface2UVCurves,
                     NULL,NULL));
#else // SM_VALIDATE_INTERSECTIONS
      SER(pSurf1->GlobalSurfaceIntersectAndValidate
                    (crContext,
                     sDomain1,
                     *pSurf2,
                     sDomain2,
                     bUseSrfEdges,
                     &sSSITol,
                     &dAngleTol,
                     &s3DCurves,
                     &sSurface1UVCurves,
                     &sSurface2UVCurves,
                     NULL,NULL));
#endif // SM_VALIDATE_INTERSECTIONS

      // gwc: old removed code
      //      SmAdvSurfaceIntersector sSSI( *pSurf1, sDomain1, *pSurf2, sDomain2 );
      //      SER( sSSI.DoIntersection( crContext, &sSSITol, &dAngleTol,
      //           &s3DCurves, &sSurface1UVCurves, &sSurface2UVCurves, NULL, NULL ));
    }

  // Check every surf/surf intersection solution:
  // If pStartV and/or pEndV were given, they should be on the intersection curve.
  // If they're given, and they're not on the intersection curve, then it's not the
  // correct int curve.  If they are both Null, then we can check pOptRefPoint.
  // That might not be directly on the int curve, just closer to the correct int
  // curve than to any other int curve.
  // If all that fails, then we look for the longest intersection curve.
  ULONG lBestDist = 9999;
  ULONG lBestLen  = 9999;
  double dMinDist = SM_BIG_DOUBLE;
  double dDist = 0.0;
  double dMaxLen = 0.0;
  ULONG lCrvIndex;
  for ( lCrvIndex=0; lCrvIndex < s3DCurves.GetSize(); lCrvIndex++ )
    {
      SmCurve *pNew3DCurve  = s3DCurves[ lCrvIndex ];
      SmCurve *pNewUVCurve1 = sSurface1UVCurves[ lCrvIndex ];
      SmCurve *pNewUVCurve2 = sSurface2UVCurves[ lCrvIndex ];
      SmExtent1d sTrimIvl   = pNew3DCurve->GetNaturalInterval();
      double dParam=0.0, dParam1=0.0, dParam2=0.0;
      SmBoolean bReverseCurves = FALSE;
      SmBoolean bSuccess       = TRUE;

#ifdef SM_DEBUG_CODE
      // draw 3DCurve and start/end points
      if ( iDebugLevel > 0 )
        {
          smgfx_SetLook(1,2, 1,0,0);
          pNew3DCurve->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1);
          if (pStartV) { pStartV->Draw(); sm_GraphicsLoop(); }
          smgfx_SetLook(1,2, 0,1,0);
          if (pEndV) { pEndV->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
      // drop StartPoint to curve within dApproxTol
      if ( pStartV )
        {
          SmPoint3d sStartPnt = pStartV->GetPoint();
          SER(pNew3DCurve->DropPoint(sTrimIvl,   // in : target curve allowed domain
                                     sStartPnt,  // in : Point to drop to curve
                                     NULL,       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                     dApproxTol, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                     NULL,       // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                     bSuccess,   // out: TRUE = found a drop point
                                     dParam1,    // out: found drop curve param
                                     dDist,      // out: found drop distance
                                     SM_SO_INTERSECT )); // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior
        }

      // drop EndPoint to curve within dApproxTol
      if ( bSuccess && pEndV )
        {
          SmPoint3d sEndPnt = pEndV->GetPoint();
          SER( pNew3DCurve->DropPoint(sTrimIvl,   // in : target curve allowed domain
                                      sEndPnt,    // in : Point to drop to curve
                                      NULL,       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                  //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                  //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                      dApproxTol, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                  //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                  //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                  //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                      NULL,       // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                      bSuccess,   // out: TRUE = found a drop point
                                      dParam2,    // out: found drop curve param
                                      dDist,      // out: found drop distance
                                      SM_SO_INTERSECT )); // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                  //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                  //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                  //      default:[SM_SO_MINIMIZE] to preserve original behavior
        }

      // skip curves that are not close to endPoints
      if ( !bSuccess )
          { continue; }

      // when given an endPoint - trim intersection curves to endPoints
      if ( pEndV )
        {
          // see if intersection curve needs to be reversed
          if ( dParam1 > dParam2 )
            {
              bReverseCurves = TRUE;
              SM_SWAP(double,dParam1,dParam2);
            }
          sTrimIvl.SetMinMax( dParam1, dParam2 );
          SER( pNew3DCurve->Trim( sTrimIvl ));  // may snap sIvl by tol to existing knots
          if ( pNewUVCurve1 )
              SER( pNewUVCurve1->Trim( sTrimIvl ));  // may snap sIvl by tol to existing knots
          if ( pNewUVCurve2 )
              SER( pNewUVCurve2->Trim( sTrimIvl ));  // may snap sIvl by tol to existing knots
        }

      // when reversing curves - reverse curve parameterizations
      if ( bReverseCurves )
        {
          SER( pNew3DCurve->ReverseParameterization( sTrimIvl, sTrimIvl ));

          if ( pNewUVCurve1 )
              SER( pNewUVCurve1->ReverseParameterization( sTrimIvl, sTrimIvl ));
          if ( pNewUVCurve2 )
              SER( pNewUVCurve2->ReverseParameterization( sTrimIvl, sTrimIvl ));
        }

      // If no pStartV or pEndV, see if we were given a reference point.
      if ( pStartV == NULL && pEndV == NULL && pOptRefPoint != NULL )
        {
          SER( pNew3DCurve->DropPoint(sTrimIvl,   // in : target curve allowed domain
                                      *pOptRefPoint, // in : Point to drop to curve
                                      NULL,       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                      dApproxTol, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                      NULL,       // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                      bSuccess,   // out: TRUE = found a drop point
                                      dParam,     // out: found drop curve param
                                      dDist,      // out: found drop distance
                                      SM_SO_MINIMIZE )); // in : SM_SO_MINIMIZE: ref pt is not necessarily on the curve
          if ( bSuccess )
            {
              // save the best curve seen - best = closest to reference point
              if (dDist < dMinDist)
                {
                  dMinDist  = dDist;
                  lBestDist = lCrvIndex;
                }
            }
        }

      // save the best curve seen - best = longest that is close to points
      double dCrvLen = pNew3DCurve->ApproximateLength(pNew3DCurve->GetNaturalInterval(),5);
      if (dCrvLen > dMaxLen)
        {
          dMaxLen  = dCrvLen;
          lBestLen = lCrvIndex;
        }
    } // end checking every intersection curve for longest solution within dApproxTol of given endPoints

  // error: didn't find a curve within tol of endPoints
  if (lBestDist == 9999  &&  lBestLen == 9999)
    {
      return SM_ERR;
    }

  // arrive here after finding a longest curve within tol of given endPoints
  ULONG lBestIndex = ( lBestDist < 9999 ) ? lBestDist : lBestLen;

  // delete every solution curve except the best one
  for ( lCrvIndex = 0; lCrvIndex < s3DCurves.GetSize(); lCrvIndex++ )
    {
      if ( lCrvIndex == lBestIndex )
        { continue; }

      SmCurve *pNew3DCurve  = s3DCurves[ lCrvIndex ];
      SmCurve *pNewUVCurve1 = sSurface1UVCurves[ lCrvIndex ];
      SmCurve *pNewUVCurve2 = sSurface2UVCurves[ lCrvIndex ];
      if (pNew3DCurve)  { delete  pNew3DCurve;   pNew3DCurve  = NULL ; }
      if (pNewUVCurve1) { delete  pNewUVCurve1;  pNewUVCurve1 = NULL ; }
      if (pNewUVCurve2) { delete  pNewUVCurve2;  pNewUVCurve2 = NULL ; }
    }

  // set output - get curve's interval
  rp3DCurve  = SM_CAST_PTR( SmBSplineCurve, s3DCurves[ lBestIndex ] );
  rpUVCurve1 = SM_CAST_PTR( SmBSplineCurve, sSurface1UVCurves[ lBestIndex ] );
  rpUVCurve2 = SM_CAST_PTR( SmBSplineCurve, sSurface2UVCurves[ lBestIndex ] );
  SmExtent1d sTrimIvl = rp3DCurve->GetNaturalInterval();

  // generate surf1 UVTrimCurve if it's missing (some intersectors don't make UVTrimCurves)
  if (rpUVCurve1 == NULL)
    {
      double dMaxDist = 0;
      double dDeviation = 0;
      SmTArray<SmBSplineCurve*> sUVCurves;
      SER(pSurf1->DropCurve(crContext, sDomain1,
                           *rp3DCurve, sTrimIvl,
                            dApproxTol,
                            dMaxDist,     // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                            dDeviation,   // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                            sUVCurves));
      SM_ASSERT(sUVCurves.GetSize() == 1);
      if ( sUVCurves.GetSize() < 1 )
        { SER( SM_ERR ); }
      rpUVCurve1 = sUVCurves[0];
    }

  // generate surf2 UVTrimCurve if it's missing (some intersectors don't make UVTrimCurves)
  if (rpUVCurve2 == NULL)
    {
      double dMaxDist = 0;
      double dDeviation = 0;
      SmTArray<SmBSplineCurve*> sUVCurves;
      SER(pSurf2->DropCurve(crContext, sDomain2,
                            *rp3DCurve, sTrimIvl,
                             dApproxTol,
                             dMaxDist,     // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,   // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sUVCurves));
      SM_ASSERT(sUVCurves.GetSize() == 1);
      if ( sUVCurves.GetSize() < 1 )
        { SER( SM_ERR ); }
      rpUVCurve2 = sUVCurves[0];
    }

#ifdef SM_DEBUG_CODE
  // draw best xSect curve and start and end points
  if ( iDebugLevel > 0 )
    {
      smgfx_SetLook(1,2, 1,0,0); rp3DCurve->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,0,1); if (pStartV) { pStartV->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,1,0); if (pEndV) { pEndV->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end sm_SrfSrfIntersection

/*******************************************************************//**
PURPOSE: Interpolate two end points & tangents with either a degree 1, 2 or 3 curve

NOTES:
***********************************************************************/
static SmStatus sm_InterpolateEndptsAndTangents
  (const SmContext & crContext,
   SmPoint3d  & rStartPnt,
   SmPoint3d  & rEndPnt,
   SmVector3d & rStartVec,
   SmVector3d & rEndVec,
   ULONG lCurveDim,
   SmBSplineCurve *& rpCurve,
   int iDebugLevel = 0)
{
    if ( lCurveDim < 2 || lCurveDim > 3 )
      { lCurveDim = 3; }

    // Unitize tangents  // Don't quit just because of zero vectors.
    rStartVec.Unitize(); // SER(rStartVec.Unitize());
    rEndVec.Unitize();   // SER(rEndVec.Unitize());

#ifdef SM_DEBUG_CODE
    if ( iDebugLevel > 0 ) {
        smgfx_SetLook(1,2, 1,0,0);
        rStartPnt.Draw();           sm_GraphicsLoop();
        rStartVec.Draw(&rStartPnt); sm_GraphicsLoop();
        rEndPnt.Draw();             sm_GraphicsLoop();
        rEndVec.Draw(&rEndPnt);     sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#else
    SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

    rpCurve = NULL;
    // Note: IsColinearWith() returns TRUE if the points are coincident.
    // That's not the behavior we want, unless both vectors are zero as well.
    double dDistBetweenPts = rStartPnt.DistanceBetween(rEndPnt);

    if (   dDistBetweenPts > SM_EFF_ZERO
        && rStartVec.IsColinearWith( rEndVec, rStartPnt, rEndPnt ) == TRUE )
    {
        // Will create a line first
        SER( SmBSplineCurve::CreateLineSegment( crContext, lCurveDim,
            rStartPnt, rEndPnt, rpCurve ));
    }
    else
    {
        // Will attempt to make a conic
        double dT1, dT2;
        SER( smgu_LineLineClosestPoint( rStartPnt, rStartVec, rEndPnt, rEndVec,
            dT1, dT2 ));

        // If the intersection is between the points,
        // the intersection parameters will have opposite signs.
        SmBoolean bCanMakeConic = dT1*dT2 < -SM_EFF_ZERO;

        // Also don't make conic if it hits right next to one point.
        // That would create a tiny (zero) end tangent.  [bd 060424]
        // Since rStartVec and rEndVec are unit vectors, dT1 and dT2 are actual distances.
        bCanMakeConic &= smos_Fabs(dT1) > 0.1 * dDistBetweenPts;
        bCanMakeConic &= smos_Fabs(dT2) > 0.1 * dDistBetweenPts;

        if ( ! bCanMakeConic )
        {
            // Can not make conic curve, create cubic Hermite instead
            //rEndVec = -rEndVec;
            // First, Adjust the lengths of direction vectors
            dT1 = smos_Fabs(dT1);
            dT2 = smos_Fabs(dT2);
            // Don't allow zero-length vectors (coincident control points)
            if ( dT1 < dT2/10 )
                dT1 = dT2/10;
            if ( dT2 < dT1/10 )
                dT2 = dT1/10;
            double dSum = dT1 + dT2;
            rStartVec *= ( dDistBetweenPts * dT1 / dSum );
            rEndVec   *= ( dDistBetweenPts * dT2 / dSum );
            SmHermiteCurve sHerm( rStartPnt, rStartVec, rEndPnt, rEndVec, lCurveDim );
            sHerm.SetContext( NULL );
            SmPoint3d sP1, sP2, sP3, sP4;
            sHerm.GetBezierPoints( sP1, sP2, sP3, sP4 );
            SmPoint3d sData[4];
            SmTArray<SmPoint3d> sCntrlPoly(4,sData);
            sCntrlPoly.Add(sP1);
            sCntrlPoly.Add(sP2);
            sCntrlPoly.Add(sP3);
            sCntrlPoly.Add(sP4);
            double adKData[2];
            SmTArray<double> sKnots(2,adKData,2);
            sKnots[0] = 0.0; sKnots[1] = 1.0;
            ULONG alKMData[2];
            SmTArray<ULONG> sKnotMult(2,alKMData,2);
            sKnotMult[0] = 4; sKnotMult[1] = 4;
            SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,3,
                sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                SM_KT_UNSPECIFIED, NULL, NULL, rpCurve));
        }
        else
        {
            // Make conic
            SmPoint3d sMidPnt = rStartPnt + dT1 * rStartVec;
            SmPoint3d sData[3];
            SmTArray<SmPoint3d> sCntrlPoly(3,sData);
            sCntrlPoly.Add(rStartPnt);
            sCntrlPoly.Add(sMidPnt);
            sCntrlPoly.Add(rEndPnt);
            double adKData[2];
            SmTArray<double> sKnots(2,adKData,2);
            sKnots[0] = 0.0; sKnots[1] = 1.0;
            ULONG alKMData[2];
            SmTArray<ULONG> sKnotMult(2,alKMData,2);
            sKnotMult[0] = 3;
            sKnotMult[1] = 3;
            double adWData[3];
            SmTArray<double> sWeights(3,adWData,3);
            sWeights[0] = sWeights[2] = 1.0;
            // Calculate the middle weight
            sWeights[1] = 1.0;
            if (smos_Fabs(dT1+dT2) < SM_EFF_ZERO_SQRT) {
                // Make circular arc
                SmVector3d sVec = rEndPnt - rStartPnt;
                double dAngle;
                SER(sVec.AngleBetween(rStartVec,dAngle));
                sWeights[1] = cos(dAngle);
                SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                    sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                    SM_KT_UNSPECIFIED, &sWeights, NULL, rpCurve));
            }
            else
            {
                //sWeights[1] = 2.0; // Make Hyperbola
                // Make parabola: no weights.
                SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                    sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                    SM_KT_UNSPECIFIED, NULL, NULL, rpCurve));
            }
        } // end if making conic
    } // end if not making line

#ifdef SM_DEBUG_CODE
    if ( iDebugLevel > 0 ) {
        smgfx_SetLook(1,2, 1,0,0); rpCurve->DrawWithKnots(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end sm_InterpolateEndptsAndTangents

/*******************************************************************//**
    END - Static functions
***********************************************************************/

/*******************************************************************//**
PURPOSE: Constructor for SmFilletVertexuse

NOTES:
***********************************************************************/
SmFilletVertexuse::SmFilletVertexuse
  ()
{
    for (ULONG i=0; i<2; i++)
    {
        m_vTsectPnt.UVPos  (i).Set(-SM_BIG_DOUBLE,-SM_BIG_DOUBLE);
        m_vTsectPnt.UVDeriv(i).Set(-SM_BIG_DOUBLE,-SM_BIG_DOUBLE);
    }

} // end SmFilletVertexuse::SmFilletVertexuse constructor

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertFilletVertexuse_list[] =
{
  {SM_AT_UNKNOWN, _T("UNKNOWN"), _T("Not yet implemented") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmFilletVertexuse::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmVertexuse::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // todo - add SmFilletVertexuse checks here

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmFilletVertexuse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmFilletVertexuse::AssertHeal
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
//       return ( SmVertexuse::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmFilletVertexuse::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmFilletVertexuse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletVertexuse::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletVertexuse_TYPE == t) ? TRUE : SmVertexuse::IsKindOf( (t) ));
}

/*************************************************************
PURPOSE: Unimplemented Dump method to satisfy SM_COMMON.

NOTES: May be implemented if desired.
**************************************************************/
void SmFilletVertexuse::Dump (void) const
{ }

/*******************************************************************//**
PURPOSE: Constructor for the SmFilletVertex object.

NOTES:
***********************************************************************/
SmFilletVertex::SmFilletVertex
  (SmFilletCorner * cpCorner)
 : SmVertex(), 
   m_eType(SM_FV_UNKNOWN),
   m_vPointClass(SmTol::GetZoneTol3d(), m_cpContext),  // needs thought: What's the right ZoneTol3d here? 
   m_pMate(),
   m_eStatus(SM_FIL_UNPROCESSED),
   m_cpCorner(cpCorner)
{
  m_vPointClass.SetContext(GetContext()) ;
  m_pMate[0] = m_pMate[1] = NULL;

} // end SmFilletVertex::SmFilletVertex constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmFilletVertex object.

NOTES:
***********************************************************************/
SmFilletVertex::~SmFilletVertex
  ()
{
  // delete all vertexuses
  SmTArray<SmVertexuse*> sVertexuses;
  GetVertexuses(sVertexuses);
  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      SmFilletVertexuse * pVU = (SmFilletVertexuse*)sVertexuses[i];
      SM_ASSERT(pVU != NULL) ; delete pVU ; pVU = NULL ;
    }

  // clear mate pointers
  for (ULONG j=0; j<2; j++)
    {
      if (m_pMate[j])
        {
          if (this == m_pMate[j]->GetMate(0))
            {
              m_pMate[j]->SetMate(0,NULL);
            }
          else if (this == m_pMate[j]->GetMate(1))
            {
              m_pMate[j]->SetMate(1,NULL);
            }
        }
    }

} // end SmFilletVertex::~SmFilletVertex destructor

/*******************************************************************//**
PURPOSE: Get the name of this vertex

NOTES:
***********************************************************************/
void SmFilletVertex::GetName
  (TCHAR * pcMyName,
   size_t  lMyNameAllocSize)
{
    switch (m_eType) {
    case SM_FV_RAIL_X_RAIL:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_RAIL_X_RAIL"));
            break;
        }
    case SM_FV_RAIL_X_EDGEUSE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_RAIL_X_EDGEUSE"));
            break;
        }
    case SM_FV_ON_VERTEX:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_ON_VERTEX"));
            break;
        }
    case SM_FV_FILLET_X_FILLET:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_FILLET_X_FILLET"));
            break;
        }
    case SM_FV_RAIL_X_EXTENDED_EDGEUSE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_RAIL_X_EXTENDED_EDGEUSE"));
            break;
        }
    case SM_FV_FILLET_X2_FILLETS:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_FILLET_X2_FILLETS"));
            break;
        }
    case SM_FV_ON_CROSS_SECTION:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_ON_CROSS_SECTION"));
            break;
        }
    case SM_FV_SETBACK:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_SETBACK"));
            break;
        }
    case SM_FV_FILLET_X_EDGEUSE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_FILLET_X_EDGEUSE"));
            break;
        }
    case SM_FV_MATE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FV_MATE"));
            break;
        }
    default:
        break;
    }

} // end SmFilletVertex::GetName

/*******************************************************************//**
PURPOSE: Calculate the geometry of 'this' vertex at a corner.

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::CalcCornerVertGeom()
{
  if (IsProcessed()) return SM_SUCCESS;

  // switch on FilletVertex->m_eType
  switch (m_eType)
    {
      case SM_FV_RAIL_X_RAIL            : return CalcRailIntRail();
      case SM_FV_RAIL_X_EDGEUSE         : return CalcRailIntEdgeuse();
      case SM_FV_ON_VERTEX              : return CalcVertOnVert();
      case SM_FV_FILLET_X_FILLET        : return CalcFilletIntFillet();
      case SM_FV_RAIL_X_EXTENDED_EDGEUSE: return CalcRailIntExtendedEdgeuse();
      case SM_FV_FILLET_X2_FILLETS      : return CalcFilletInt2Fillets();
      case SM_FV_ON_CROSS_SECTION       : return CalcVertOnCrossSection();
      case SM_FV_SETBACK                : return CalcSetBackVert();
      case SM_FV_MATE                   : return CalcMateGeom();
      case SM_FV_FILLET_X_EDGEUSE       :
      default                           : break;
    } // end switch on FilletVertex->m_eType

  return SM_SUCCESS;

} // end SmFilletVertex::CalcCornerVertGeom

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_RAIL_X_RAIL.
            Two rail curves are in a common face; find their intersection
            in that face.

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::CalcRailIntRail
  ()
{
  // First, make sure we have a corner and exactly two FilletGeom's.
  NER(m_cpCorner);
  SmTArray<SmFilletGeom*> sGeoms;
  GetFilletGeoms(sGeoms);
  ULONG lTotalGeoms = sGeoms.GetSize();
  if (lTotalGeoms != 2) SER(SM_ERR);
  SmFilletGeom * pFilletGeom1 = sGeoms[0];
  SmFilletGeom * pFilletGeom2 = sGeoms[1];

  const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex();
  SmFilletSolver * pFilSolver1 = pFilletGeom1->GetFilletSolver();
  SmFilletSolver * pFilSolver2 = pFilletGeom2->GetFilletSolver();
  NER(pFilSolver1); NER(pFilSolver2);

  // Get the common face, and the indices of the rail curves.
  SmFace * pFace = (SmFace*)GetPointClassObject();  
  ULONG lRailIndex1 = pFilSolver1->FindIndexOfRailOnFace( pFace );
  ULONG lRailIndex2 = pFilSolver2->FindIndexOfRailOnFace( pFace );

  // Figure a guess uv for the intersection of the rail curves in the face.
  // Method: starting at the vertex common to both filleted edges,
  // find a point along each edge that is twice the fillet radius away
  // from the vertex.  Get the uv positions of those points in the common
  // face via the Edgeuse's uv trim curve.  The center of the fillet
  // (which is the intersection of the rails) should be somewhere near
  // the midpoint of those two points.

  SmPoint3d sUV[2];  // This loop sets these.

  // Locals for the loop.
  SmFilletSolver * pSolver[2] = { pFilSolver1, pFilSolver2 };
  ULONG lIndex[2] = { lRailIndex1, lRailIndex2 };

  for ( ULONG ii=0; ii<2; ii++ )
    {
      SmOffsetSurface * pOffsetSurf = pSolver[ii]->GetSurface( lIndex[ii] );
      double dOffsetDistance = smos_Fabs( pOffsetSurf->GetOffsetDistance() );
      SmEdgeuse *  pEU = pSolver[ii]->GetEdgeuse( lIndex[ii] );
      SmEdge *   pEdge = pEU->GetEdge();
      SmExtent1d  sIvl = pEdge->GetInterval();
      SmCurve * pCurve = pEdge->GetCurve();
      double dOffsetParam;

      SmVertex * pV = pEU->GetVertexuse()->GetVertex();
      if (   (pV == cpVertex && pEU->GetOrientation() == SM_OT_SAME)
          || (pV != cpVertex && pEU->GetOrientation() == SM_OT_OPPOSITE ))
        {
          SER( pCurve->FindParameterAtArcLength(
                  sIvl.GetMin(), 2.0*dOffsetDistance, dOffsetParam ));
        }
      else
        {
          SER( pCurve->FindParameterAtArcLength(
                  sIvl.GetMax(), -2.0*dOffsetDistance, dOffsetParam ));
        }
      SmBSplineCurve * pTrimCurve = NULL;
      SER( pEU->GetOrCreateUVTrimCurve( pTrimCurve ));
      NER( pTrimCurve );

      SER(pTrimCurve->EvaluatePoint( dOffsetParam, sUV[ii]) );

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 )
        {
          smgfx_SetLook(1,2, 1,0,1); pFace->DrawUV(4,4); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,1); pEU->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
          SmSurface * pSurf = pFace->GetSurface();
          SmPoint3d sCornerPt;
          SmPoint2d sTestUV = SmPoint2d(sUV[ii].x, sUV[ii].y);
          pSurf->EvaluatePoint(sTestUV,sCornerPt);
          smgfx_SetLook(4,6, 1,0,0); sCornerPt.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    }

  // Got uv's in the face at 2*radius offsets along each filleted edge.
  // Their midpoint will be our guess.
  SmPoint3d sMidUV = (sUV[0] + sUV[1])*0.5;
  SmPoint2d sUVGuess( sMidUV.x, sMidUV.y );

#ifdef SM_DEBUG_CODE
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if ( DebugLevel() > 0  || lDebugCount == lCount)
    {
      SmBrep    * pBrep = pFace->GetBrep() ;
      SmSurface * pSurf = pFace->GetSurface();
      SmPoint3d sCornerPt;
      pSurf->EvaluatePoint(sUVGuess, sCornerPt);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurf) pSurf->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; pFilSolver1->GetEdgeuse(lRailIndex1)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,1) ; pFilSolver2->GetEdgeuse(lRailIndex2)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(5,7, 0,1,0) ; sCornerPt.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Get the TsectPnts and call the solver.
  SmFilletVertexuse * pVU1       = GetVUAtRailEnd( pFilletGeom1 );
  SmFilletVertexuse * pVU2       = GetVUAtRailEnd( pFilletGeom2 );
  SmTsectPnt        & rTsectPnt1 = pVU1->GetTsectPnt();
  SmTsectPnt        & rTsectPnt2 = pVU2->GetTsectPnt();
  SmBoolean bFoundIntersection;

  SER( pFilSolver1->RailRailIntersect(sUVGuess, lRailIndex1,
                                      pFilSolver2, lRailIndex2,
                                      bFoundIntersection, rTsectPnt1, rTsectPnt2));

  if (!bFoundIntersection)
    {
      SetStatus(SM_FV_SOLVER_NOT_CONVERGE);
      SER(SM_ERR);
    }

  // Save the SolutionPoints
  SmPoint3d sPnt;
  SER(pFace->GetSurface()->EvaluatePoint(rTsectPnt1.UVPos(lRailIndex1),
                                         sPnt ));
  SetOriginalUV( rTsectPnt1.UVPos(lRailIndex1) );
  SetPoint     ( sPnt );
  SetStatus    ( SM_FIL_PROCESSED );

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 )
    {
      SmBrep    * pBrep = pFace->GetBrep() ;
      SmSurface * pSurf = pFace->GetSurface();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurf) pSurf->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; pFilSolver1->GetEdgeuse(lRailIndex1)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,1) ; pFilSolver2->GetEdgeuse(lRailIndex2)->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(5,7, 0,1,0) ; sPnt.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Calculate Mate if needed
  for ( ULONG j=0; j<2; j++ )
    {
      SmFilletVertex * pMate = GetMate(j);
      if ( pMate == NULL )
          continue;

      SmFace * pF = (SmFace*)pMate->GetPointClassObject();
      SmPoint3d sPt;
      SmFilletVertexuse * pVU = pMate->GetVUAtRailEnd( pFilletGeom1 );
      SmPoint2d sUVPt;
      if (pVU)
        {
          sUVPt = rTsectPnt1.UVPos( 1-lRailIndex1 );
        }
      else
        {
          pVU = pMate->GetVUAtRailEnd(pFilletGeom2);
          sUVPt = rTsectPnt2.UVPos( 1-lRailIndex2 );
        }
      SER( pF->GetSurface()->EvaluatePoint( sUVPt, sPt ) );
      pMate->SetOriginalUV( sUVPt );
      pMate->SetPoint     ( sPt );
      pMate->SetStatus    ( SM_FIL_PROCESSED );
    }

  return SM_SUCCESS;

} // end SmFilletVertex::CalcRailIntRail

/*******************************************************************//**
PURPOSE: If the given UV is on seam of the given surface,
     adjust it if the step-off direction is 'opposite'
     to the given rRefDirection.
     It is possible in the 4x2 cases that uv point lies on
     wrong side of the seam.

NOTES:
***********************************************************************/
static SmStatus sm_AdjustUVOnSeam
  (const SmSurface * pSurf,
   SmVector3d & rRefDirection,
   SmPoint2d & rUV)
{
    SmExtent2d sDomain = pSurf->GetNaturalUVDomain();
    SmPoint2d sMin = sDomain.GetMin();
    SmPoint2d sMax = sDomain.GetMax();

    SmVector3d sPnt, sDU, sDV;
    SER(pSurf->Evaluate1stDerivatives( rUV, TRUE, TRUE, sPnt, sDU, sDV ));

    if ( pSurf->IsClosed( sDomain, SM_SP_U ) )
      {
        double d2dTolerance = sDomain.XLength() / 1000;
        double dAngle;
        if ( smos_Fabs( rUV.x - sMin.x ) < d2dTolerance )
          {
            SER( sDU.AngleBetween( rRefDirection, dAngle ));
            if ( dAngle > SM_PI/2.0 )
              { rUV.x += sDomain.XLength(); }

          }  // end if close to low U

        else if ( smos_Fabs( rUV.x - sMax.x ) < d2dTolerance )
          {
            sDU = -sDU;  // Backwards from top of domain.
            SER( sDU.AngleBetween( rRefDirection, dAngle ));

            if ( dAngle > SM_PI/2.0 )
              { rUV.x -= sDomain.XLength(); }

          }  // end if close to high U
      } // end if closed in U

    if ( pSurf->IsClosed( sDomain, SM_SP_V ))
      {
        double d2dTolerance = sDomain.YLength() / 1000;
        double dAngle;
        if ( smos_Fabs( rUV.y - sMin.y ) < d2dTolerance )
          {
            SER( sDV.AngleBetween( rRefDirection, dAngle ));

            if ( dAngle > SM_PI/2.0 )
              { rUV.y += sDomain.YLength(); }

          }  // end if close to low V

        else if ( smos_Fabs( rUV.y - sMax.y ) < d2dTolerance )
          {
            sDV = -sDV;  // Backwards from top of domain.
            SER( sDV.AngleBetween( rRefDirection, dAngle ));

                if ( dAngle > SM_PI/2.0 )
                  { rUV.y -= sDomain.YLength(); }

          }  // end if close to high V
      } // end if closed in V

    return SM_SUCCESS;

} // end sm_AdjustUVOnSeam

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_RAIL_X_EDGEUSE.

NOTES:
  This function finds the point by using a NewtonRaphson iteration
  which moves along the SideEdge->UVTrimCurve and the FilletFace->Surface.
  This solution is found without referencing a FilletSurface and
  may be called prior to defining the filletSurface shape.

  effects: SmFilletVertex::SetPoint(sPnt);
           SmFilletVertex::SetOriginalTParam(dEdgeuseParameter);
           SmFilletVertex::SetStatus(SM_FIL_PROCESSED);
     where sPnt = Projection of sideEdge->Curve/RailEdge->Curve xSect point to sideEdgeCurve

     when  SmFilletVertex has mates (matching vertices on other railCurves)
           Mate::SetPoint(sPnt);
           Mate::SetOriginalTParam(dEdgeuseParameter);
           Mate::SetStatus(SM_FIL_PROCESSED);
     where sPnt = intersection point projected onto rail->surface

  This also has the side effect of modifying the SmFilletGeoms.
  It sets m_vUVCurvePV and m_dCurveParameter in the corresponding
  SmFilletVertexuse in each SmFilletGeom.
***********************************************************************/
SmStatus SmFilletVertex::CalcRailIntEdgeuse
 (SmFilletGeom * pOptFG)   // in : notNULL = ignore other FilletGeoms
                           //                attached to this filletVertex
                           //      NULL    = process 1st and last filletGeom
                           //                attached to this vertex
{
  NER( m_cpCorner );

  // get original Brep vertex
  const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex();

  // get this filletVertex->filletEdgeuse[type == SM_FE_RAIL]->FilletGeom(s)
  SmTArray<SmFilletGeom*> sGeoms;
  GetFilletGeoms( sGeoms );

  // check state - at least 1 railEdge must be connected to this vertex
  ULONG lTotalGeoms = sGeoms.GetSize();
  if ( lTotalGeoms < 1 )
    { SER( SM_ERR ); }

  // locals: pFilletGeom2 is set only when lTotalGeoms == 2
  SmFilletGeom * pFilletGeom1  = sGeoms[0];
  SmFilletGeom * pFilletGeom2  = (lTotalGeoms == 2)
                                 ? sGeoms[1]
                                 : NULL;
  SM_ASSERT(lTotalGeoms <= 2) ; // gwc_test: this code assumes this to be true from here on out

  // get handles to intersection curves:
  // originalBrep sideEdgeuse and FilletVertexuse connected to railEdge

  // get original Brep Edgeuse connected to this FilletVertex
  SmEdgeuse         * pSideEU1 = (SmEdgeuse*)GetPointClassObject();

  // get vertexuse that is connected to a pFilletGeom1->filletRailEdge
  SmFilletVertexuse * pVU1     = GetVUAtRailEnd(pFilletGeom1);

  // Set dGuessEdgeParameters to run from one edgeEnd to the other based on orienation
  double dEdgeuseParameter = 0.0;
  double dGuessEdgeParameters[9];
  SmEdge     * pSideEdge = pSideEU1->GetEdge();
  SmExtent1d   sIvl      = pSideEdge->GetInterval();

  // side edgeuse was carefully selected based on its orientation
  // See 'sm_FindClosedSideEdgeuse' in SmFilletCorner.cpp
  SmBoolean bIsClosed  = pSideEdge->IsClosed() ;
  SmBoolean bFromStart =   bIsClosed
                         ? ( pSideEU1 ->GetOrientation() == SM_OT_SAME )
                         : ( pSideEdge->GetStartVertex() == cpVertex );

  // set the guess sequence from the Start
  // Make the first guess be a bit intelligent: a fraction of the length based on radius.
  double dGuessParam = ( bFromStart ) ? sIvl.Evaluate(0.01) : sIvl.Evaluate(0.99);  // what it used to be...
  double dSideEdgeLength = pSideEdge->GetCurve()->ApproximateLength( sIvl, 5 );
  // Get the radius.
  SmOffsetSurface *pOffSrf = pFilletGeom1->GetOffsetSurface(0);
  if ( pOffSrf != NULL )
  {
      double dRad = smos_Fabs( pOffSrf->GetOffsetDistance() );
      if ( dSideEdgeLength > dRad / 2.0 )
      {
          double dFrac = dRad / dSideEdgeLength;
          if ( ! bFromStart )
            { dFrac = 1.0 - dFrac; }
          dGuessParam  = sIvl.Evaluate( dFrac );
      }
  }

  if (bFromStart)
    {
      dGuessEdgeParameters[0] = dGuessParam;
      dGuessEdgeParameters[1] = sIvl.Evaluate(0.05);
      dGuessEdgeParameters[2] = sIvl.Evaluate(0.1);
      dGuessEdgeParameters[3] = sIvl.Evaluate(0.23);
      dGuessEdgeParameters[4] = sIvl.Evaluate(0.5);
      dGuessEdgeParameters[5] = sIvl.Evaluate(0.77);
      dGuessEdgeParameters[6] = sIvl.Evaluate(0.9);
      dGuessEdgeParameters[7] = sIvl.Evaluate(0.95);
      dGuessEdgeParameters[8] = sIvl.Evaluate(0.99);
    }
  else // set the guess sequence from the End
    {
      dGuessEdgeParameters[0] = dGuessParam;
      dGuessEdgeParameters[1] = sIvl.Evaluate(0.95);
      dGuessEdgeParameters[2] = sIvl.Evaluate(0.9);
      dGuessEdgeParameters[3] = sIvl.Evaluate(0.77);
      dGuessEdgeParameters[4] = sIvl.Evaluate(0.5);
      dGuessEdgeParameters[5] = sIvl.Evaluate(0.23);
      dGuessEdgeParameters[6] = sIvl.Evaluate(0.1);
      dGuessEdgeParameters[7] = sIvl.Evaluate(0.05);
      dGuessEdgeParameters[8] = sIvl.Evaluate(0.01);

    } // end setting SideEdge gueuss sequence branches

  // Sometimes, we may just need to do one side
  // such as processing tangent roll-over
  if ( pOptFG != NULL )
    {
      if      ( pOptFG == pFilletGeom1 ) { pFilletGeom2 = NULL; }
      else if ( pOptFG == pFilletGeom2 ) { pFilletGeom1 = NULL; }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugFVUs=FALSE;
#endif // SM_DEBUG_CODE

  // For FilletGeom1: find the rail/sideEdge intersection,
  //                  place it on the sideEdge Curve, save it
  //                  and update any FilletVertexMates if needed
  if ( pFilletGeom1 != NULL )
    {
      ULONG lRailIndex = 0;
      pFilletGeom1->GetRailIndexOfSideEU( pSideEU1, lRailIndex );

#ifdef SM_DEBUG_CODE
      if ( bDebugFVUs )
      {   pFilletGeom1->DumpFilletVUs();
          pFilletGeom2->DumpFilletVUs();
      }

      // draw sideEdge(red), railEdge(green) and fillet offset surfaces(black)
      if ( DebugLevel() > 0 )
        {
          if ( FALSE )
          {
              smgfx_Erase();
              smgfx_SetLook( 1,1, 0,0,0 ); pSideEU1->GetBrep()->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
          smgfx_SetLook(3,1, 1,0,0); pSideEU1->Draw(); sm_GraphicsLoop();
          SmFilletSolver * pFilSolver1 = pFilletGeom1->GetFilletSolver();
          smgfx_SetLook(3,1, 0,1,0);pFilSolver1->GetEdgeuse(lRailIndex)->Draw();
          sm_GraphicsLoop();
          if ( DebugLevel() > 10 ) {
              smgfx_SetLook(1,1, 0,0,0) ;

              SmOffsetSurface *pOffSrf1 = pFilletGeom1->GetOffsetSurface(   lRailIndex );
              SmOffsetSurface *pOffSrf2 = pFilletGeom1->GetOffsetSurface( 1-lRailIndex );

              // draw base or offset surfaces
              static constexpr SmBoolean bBaseOnly = TRUE;
              if ( bBaseOnly )
              {
                  smgfx_SetLook( 1,2, 0,0,0 ); pOffSrf1->GetBaseSurface()->DrawUV(2,2); sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,1,0 ); pOffSrf2->GetBaseSurface()->DrawUV(2,2); sm_GraphicsLoop();
              }
              else
              {
                  smgfx_SetLook( 1,2, 0,0,0 ); pOffSrf1->DrawUV(2,2); sm_GraphicsLoop();
                  smgfx_SetLook( 1,2, 0,1,0 ); pOffSrf2->DrawUV(2,2); sm_GraphicsLoop();
              }
              sm_GraphicsLoop();
          }

          if ( DebugLevel() > 20 ) {
              SmCurve *pEdgeCrv   = pSideEU1->GetEdge()->GetCurve();
              SmSurface *pEdgeSrf = pSideEU1->GetFace()->GetSurface();
              SmExtent1d sDom = pSideEU1->GetEdge()->GetInterval();
              SmPoint3d sEUUVPt, sEU3dPt, sSrfPt;
              double dEdgeT, dGuess = 0;
              double dDist1, dMaxDist1 = -1;
              double dDist2, dMaxDist2 = -1;
              double dDist3, dMaxDist3 = -1;
              SmBoolean bOK;

              if ( pSideEU1->GetOrientation() == SM_OT_OPPOSITE ) { dGuess = 1; }
              dGuess = sDom.Evaluate( dGuess );
              for ( ULONG jjj = 0; jjj <= 16; jjj++ ) {
                  double t0 = (double)jjj / 16.0;
                  pSideEU1->NormalizedEvaluate( t0, FALSE, sEU3dPt );    // TRUE = UV Eval, FALSE = 3d Eval
                  pSideEU1->NormalizedEvaluate( t0, TRUE,  sEUUVPt );    // TRUE = UV Eval, FALSE = 3d Eval
                  pEdgeSrf->EvaluatePoint( SmPoint2d( sEUUVPt.x, sEUUVPt.y ), sSrfPt );
                  dDist1 = sEU3dPt.DistanceBetween( sSrfPt );
                  if ( dDist1 > dMaxDist1 )
                    { dMaxDist1 = dDist1; }

                  pEdgeCrv->DropPoint(sDom,      // in : target curve allowed domain
                                      sSrfPt,    // in : Point to drop to curve
                                      NULL,      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.                           
                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                      10,        // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                      &dGuess,   // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                      bOK,       // out: TRUE = found a drop point
                                      dEdgeT,    // out: found drop curve param
                                      dDist2) ;  // out: found drop distance
                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior
                  if ( bOK ) {
                      if ( dDist2 > dMaxDist2 )
                        { dMaxDist2 = dDist2; }
                      dGuess = dEdgeT;
                  }

                  SmPoint2d sUV, sUVGuess( sEUUVPt.x, sEUUVPt.y );
                  SmBoolean bIsMulti;
                  pEdgeSrf->DropPoint(sEU3dPt, 
                                      pEdgeSrf->GetNaturalUVDomain(),
                                      &sUVGuess, 
                                      bOK, 
                                      sUV, 
                                      dDist3,
                                      bIsMulti);
                  if ( bOK ) {
                      if ( dDist3 > dMaxDist3 )
                        { dMaxDist3 = dDist3; }
                  }
              }
              smgfx_Erase();
              smgfx_SetLook( 2,1, 1,0,0 ); pSideEU1->Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 2,1, 0,1,0 ); pEdgeCrv->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
              smgfx_SetLook( 1,1, 0,0,0 ); pSideEU1->GetBrep()->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
          } // end if DebugLevel() > 20 , checking precision of pSideEU1
        }
#endif // SM_DEBUG_CODE

      // for every sideEdge guess parameter - seek rail/edge intersection
      // Note: this sets fields in rTsectPnt1, which is owned by pVU1,
      // which is in pFilletGeom1, so we're modifying pFilletGeom1 here.
      // Also, it sets rTsectPnt1.m_dCurveParameter to dEdgeuserParameter,
      // which is the param on the side edge; it would normally be the
      // parameter along the fillet.  That is put into rTsectPnt1.m_adUserDoubles[0].

      SmBoolean bFoundIntersection = FALSE;
      SmTsectPnt & rTsectPnt1 = pVU1->GetTsectPnt();
      for ( ULONG ii=0; ii<9; ii++ )
        {
          // load rTsectPnt with the rail/curve intersection found as
          // point on sideEdge->UVTrimCurve and FilletFace->Surface

          // Temporarily change this to a constant-radius function.
          // See notes in the class header, in SmFilletExecutive.h.
          SmFilletSolver *pFSForChange = ( this->GetFilletCorner()->IsCalcAsConstRad() )
                                           ? pFilletGeom1->GetFilletSolver()
                                           : NULL;
          SmTemporaryRadiusChange sTempRad( pFSForChange, lRailIndex, pSideEU1 );

          SER( pFilletGeom1->RailEdgeuseIntersect(lRailIndex,
                                                  pSideEU1,
                                                  dGuessEdgeParameters[ii],
                                                  bFoundIntersection,
                                                  rTsectPnt1,
                                                  dEdgeuseParameter));
          if ( bFoundIntersection )
            { break ; }

        } // end iter every sideEdge guess parameter seeking rail/edge intersection

      // when no intersection was found
      // Try once more with a bigger extension. [B465]
      if ( !bFoundIntersection )
        {
          SmFilletSolver *pFS = pFilletGeom1->GetFilletSolver();
          double dExtFactor = pFS->GetExtensionFactor();
          pFS->SetExtensionFactor( 2 * dExtFactor );

          for ( ULONG ii=0; ii<9; ii++ )
            {
              // load rTsectPnt with the rail/curve intersection found as
              // point on sideEdge->UVTrimCurve and FilletFace->Surface

              SmTemporaryRadiusChange sTempRad( pFilletGeom1->GetFilletSolver(), lRailIndex, pSideEU1 );

              SER( pFilletGeom1->RailEdgeuseIntersect(lRailIndex,
                                                      pSideEU1,
                                                      dGuessEdgeParameters[ii],
                                                      bFoundIntersection,
                                                      rTsectPnt1,
                                                      dEdgeuseParameter)) ;
              if ( bFoundIntersection )
                  break;

            } // end iter every sideEdge guess parameter seekin rail/edge intersection
        } // end if didn't find intersection

      // when still not found
      if ( !bFoundIntersection )
        {
          // inform the public as needed
          SetStatus( SM_FV_NO_INT_RAIL_EU );

          // Sometimes the fillet can still succeed.  [iter 401,403]
          if ( m_cpCorner->IsBlendingCorner() )
              SER( SM_ERR );

          return SM_SUCCESS;
        }

      // force solution to lie on SideEdge->Curve and save that as FilletVertex Position

      // let sPnt = Rail->BaseSurface intersection point
      SmPoint3d         sPnt;
      SmPoint2d         sUV    = rTsectPnt1.UVPos(lRailIndex);
      const SmSurface * pSurf1 = pFilletGeom1->GetOffsetSurface(lRailIndex)->GetBaseSurface();
      SER( pSurf1->EvaluatePoint( sUV, sPnt ) );

      // let dGap = Projection distance from sPnt to SideEdge->Curve
      SmCurve  * pCurve = pSideEU1->GetEdge()->GetCurve();
      SmSolution sSolution;
      SmBoolean  bFound;
      SER( pCurve->LocalPointSolve(pCurve->GetNaturalInterval(), // in : search interval
                                   SM_SO_MINIMIZE,               // in : specify specific operation to optimize
                                   sPnt,                         // in : point specializing this search
                                   NULL,                         // in : Opt Max allowed solution distance (NULL to ignore) 
                                                                 //      for SM_SO_RAYFIRE
                                                                 //          SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                   NULL,                         // in : required curve/TestPoint desired dist (NULL when not used)
                                                                 //      for SM_SO_AT_DISTANCE
                                   NULL,                         // in : required Vector direction (NULL when not used)
                                                                 //      for SM_SO_RAYFIRE
                                                                 //          SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                   dEdgeuseParameter,            // in : search starting parameter
                                   bFound,                       // out: TRUE=converged,FALSE=didn't
                                   sSolution)) ;                 // out: solution container for solver

      //double dGap = sSolution.m_vStart.m_dSolutionValue;

      // set dEdgeuseParameter = Point on sideEdge->Curve to which sPnt projected
      dEdgeuseParameter = sSolution.m_vStart.m_adParameters[0];

#ifdef SM_DEBUG_CODE
// GWC_NOTE CHANGE_NEXT_LINE_TO_TRUE_TO_TRACK_APPROX_XSECT_RESULTS_WHERE_EXACT_ARE_EXPECTED GWC_LINE ;
SmBoolean bDebugMe = FALSE ;
      // check solution quality
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(pCurve) ;  
          SM_DUMP_AND_ASSERT_VALID(pFilletGeom1->GetFilletSurface()) ; 
           
          SmPoint3d sCheckPnt[2] ;
          pCurve->Evaluate(dEdgeuseParameter, 1, TRUE, sCheckPnt) ;
          sCheckPnt[1].Unitize() ;
          SmVector3d sGap = sPnt - sCheckPnt[0] ;
          double     dGapLen  = sGap.Length() ;
          // double     dGapPar  = sGap.Dot(sCheckPnt[1]) ; 
          // double     dGapPerp = smos_Sqrt(dGapLen * dGapLen - dGapPar * dGapPar) ;
          SM_ASSERT_MSG(dGapLen < SmTol::GetScaledZero(sPnt), _T("SmFillVertex::CalcRailEdgeuse: found tolerant Rail/Surf XSect - should be exact - review")) ;

          // gwc: I currently have partial knowledge of the fillet package - so the following note may be off.
          //      The above assert test should never fail.  I believe the vertex location should be
          //      at an exact intersection between the SideEdge and the Edgeuse.  The dGapLen size should be machine precision,
          //      but I believe the SideEdge is an intersection curve and when the SideEdge/FilletSurface intersection happens
          //      towards the middle of the intersection curve's span where the shape of the intersection curve deviates from
          //      the ideal shape by the max allowed Approximation tolerance, the final vertex location will  
          //      inherit the max allowed approximation tolerance used to define the intersection curve.
          //      I'm guessing that this gap can be driven down to machine precision if the intersection curve were
          //      to have a through point at the intersection point.  The Boolean package has been extended through 
          //      the SmCurveClassification class to
          //      have a refine point mechanism that adds through points to intersection curves at requested parameter values.  
          //      That mechanism should be used in this fillet package to introduce a through point at dEdgeuseParameter in the
          //      SideEdge curve shape.  Perhaps in the Solver code, like the method SmFilletSolver::RailEdgeuseIntersect().
          //      That should tighten up the gap between the SideEdge and the FilletSurface/Rail intersection.
        }
#endif // SM_DEBUG_CODE

      // set outputs: SmfilletVertex->Point
      //    and SmFilletVertex->PointClassification->CurveParameter
      SetPoint( sPnt );
      SetOriginalTParam( dEdgeuseParameter );

      if ( pFilletGeom1 == pFilletGeom2 )  // Seam cases
        {
          // If uv on the other side of the solver happens to be on a seam too,
          // then check to see if we need to adjust it.
          SmVector3d sBinPnt, sBinVec;

          SER( pSideEU1->EvaluateBinormal( dEdgeuseParameter, FALSE, sBinPnt, sBinVec ));

          const SmSurface * pOtherSurf = pFilletGeom1->GetOffsetSurface(1-lRailIndex)->GetBaseSurface();

          SER( sm_AdjustUVOnSeam( pOtherSurf, sBinVec, rTsectPnt1.UVPos(1-lRailIndex) ));
        }

#ifdef SM_DEBUG_CODE
      // draw Rail->Surface xSectPoint(red), sideEdge xSectPoint(green)
      // optionally draw rail->Surface(black), FilletOffsetSurface xSectPoints(blue)
      if ( DebugLevel() > 0 )
      { 
        // if(0)
        // { // draw rail->Surface
        //   smgfx_SetLook( 1, 2, 0, 0, 0 ); pSurf1->Draw(); sm_GraphicsLoop();
        // }

          smgfx_SetLook(1,5, 1,0,0);
          sPnt.Draw(); sm_GraphicsLoop();
          SmPoint3d sPnt1;
          pCurve->EvaluatePoint(dEdgeuseParameter,sPnt1);
          // double dGapSize = sPnt.DistanceBetween(sPnt1);
          smgfx_SetLook(1,8, 0,1,0); sPnt1.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
          // if (0)
          //   {
          //     SmPoint3d s3DPnt;
          //     smgfx_SetLook(1,10, 0,0,1);
          //     for (ULONG g=0; g<2; g++)
          //       {
          //         // draw FilletOffsetSurface xSectPoints
          //         SmPoint2d sUVPnt = rTsectPnt1.UVPos(g);
          //         pFilletGeom1->GetOffsetSurface(g)->EvaluatePoint(sUVPnt,s3DPnt);
          //         s3DPnt.Draw(); sm_GraphicsLoop();
          //       }
          //   }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Calculate FilletVertex->Mates if needed
      // There can be at most two of them.
      for ( ULONG j=0; j<2; j++ )
        {
          // get Mate = matching vertices on other railCurves
          SmFilletVertex * pMate = GetMate(j);
          if ( pMate == NULL )
            { continue; }

          SmFilletVertexuse * pVU = pMate->GetVUAtRailEnd( pFilletGeom1 );
          if ( !pVU )
            { continue; }

          SmFace *pFace = (SmFace*)pMate->GetPointClassObject();
          NER( pFace );
          SmPoint2d sUV1 = rTsectPnt1.UVPos(1-lRailIndex);
          SmPoint3d sPnt1;

          // compute sPnt1 = pFace->Surface(sUV1), use ExtendedSurface as needed
          SER( sm_EvaluateMate( pFace, sUV1, sPnt1 ));

#ifdef SM_DEBUG_CODE
          // draw Face(black), sPnt1(green)
          if ( DebugLevel() > 0 )
            {
              smgfx_SetLook(1,1, 0,0,0); pFace->DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(2,8, 0,1,0); sPnt1.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // set mate outputs
          pMate->SetOriginalUV( sUV1 ); // Sets this into the FV's PointClass object.
          pMate->SetPoint( sPnt1 );
          pMate->SetStatus( SM_FIL_PROCESSED );
          break;

        } // end iter over potential mates

      // done with this FilletVertex
      SetStatus( SM_FIL_PROCESSED );

#ifdef SM_DEBUG_CODE
      if ( bDebugFVUs )
        { pFilletGeom1->DumpFilletVUs(); }
#endif

    } // end pFilletGeom1 existence check

  // Process intersection for adjacent solver if presented
  if ( pFilletGeom2 )
    {

#ifdef SM_DEBUG_CODE
      if ( bDebugFVUs )
        { pFilletGeom2->DumpFilletVUs(); }
#endif

      //SmFilletSolver * pFilSolver2= pFilletGeom2->GetFilletSolver();
      SmEdgeuse * pSideEU2 = pSideEU1->GetRadial();
      ULONG lOtherRailIndex = 0;
      pFilletGeom2->GetRailIndexOfSideEU( pSideEU2, lOtherRailIndex );

      SmFilletVertexuse * pVU2 = GetVUAtRailEnd( pFilletGeom2, pVU1 );

      // for every sideEdge guess parameter - seek rail/edge intersection
      // Note: this sets fields in rTsectPnt2, which is owned by pVU2,
      // which is in pFilletGeom2, so we're modifying pFilletGeom2 here.

      SmBoolean bFoundIntersection = FALSE;
      SmTsectPnt & rTsectPnt2 = pVU2->GetTsectPnt();
      double dEdgeuseParameter2 = 0.0;

      for ( ULONG ii=0; ii<9; ii++ )
        {
          // Temporarily change this to a constant-radius function.
          // See notes in the class header, in SmFilletExecutive.h.
          SmTemporaryRadiusChange sTempRad( pFilletGeom2->GetFilletSolver(), lOtherRailIndex, pSideEU2 );

          SER( pFilletGeom2->RailEdgeuseIntersect(
               lOtherRailIndex,
               pSideEU2,
               dGuessEdgeParameters[ii],
               bFoundIntersection,
               rTsectPnt2,
               dEdgeuseParameter2 ));

          if ( bFoundIntersection )
              break;

        } // end iter every sideEdge guess parameter seekin rail/edge intersection

      // when no intersection was found
      if ( !bFoundIntersection )
        {
          // inform the public as needed
          SetStatus( SM_FV_SOLVER_NOT_CONVERGE );
          return SM_SUCCESS;
        }

      //
      if ( pFilletGeom1 == pFilletGeom2 )  // Seam cases
        {
          // If uv on the other side of the solver happens to be on a seam too,
          // then check to see if we need to adjust it.
          SmVector3d sBinPnt, sBinVec;
          SER( pSideEU2->EvaluateBinormal( dEdgeuseParameter2, FALSE,
              sBinPnt, sBinVec ));
          const SmSurface * pOtherSurf =
              pFilletGeom2->GetOffsetSurface( 1-lOtherRailIndex )->GetBaseSurface();
          SER( sm_AdjustUVOnSeam( pOtherSurf, sBinVec,
              rTsectPnt2.UVPos( 1-lOtherRailIndex )));
        }

#ifdef SM_DEBUG_CODE
      // draw sideEdge xSectPoint(red)
      // optionally draw rail->Surface(black), FilletOffsetSurface xSectPoints(blue)
      if ( DebugLevel() > 0 )
        {
          SmPoint3d sPnt2;
          pSideEU2->GetEdge()->GetCurve()->EvaluatePoint(dEdgeuseParameter2,sPnt2);
          smgfx_SetPointSize(10.0);
          smgfx_SetLook(1,4, 1,0,0);
          sPnt2.Draw(); sm_GraphicsLoop();
          // if (0)
          //   {
          //     SmPoint3d s3DPnt;
          //     for (ULONG g=0; g<2; g++)
          //       {
          //         // draw FilletOffsetSurface xSectPoints
          //         SmPoint2d sUVPnt = rTsectPnt2.UVPos(g);
          //         pFilletGeom2->GetOffsetSurface(g)->EvaluatePoint(sUVPnt,s3DPnt);
          //         smgfx_SetLook(1,4, 0,0,1);
          //         s3DPnt.Draw(); sm_GraphicsLoop();
          //       }
          //   }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Calculate Mate if needed
      // NOTE: will process MATE even when two rails failed to meet each other
      // at the same location. May need more error-handlings here
      // There can be at most two Mates.
      for ( ULONG jj=0; jj<2; jj++ )
        {
          SmFilletVertex * pMate = GetMate(jj);
          if (pMate == NULL || pMate->IsProcessed()) continue;

          SmFilletVertexuse * pVU = pMate->GetVUAtRailEnd(pFilletGeom2);
          if (!pVU) continue;

          SmFace    * pFace = (SmFace*)pMate->GetPointClassObject(); NER(pFace);
          SmPoint2d   sUV1  = rTsectPnt2.UVPos(1-lOtherRailIndex);
          SmPoint3d   sPnt1;

          // compute sPnt1 = pFace->Surface(sUV1), use ExtendedSurface as needed
          SER(sm_EvaluateMate(pFace,sUV1,sPnt1));

          // set mate outputs
          pMate->SetOriginalUV(sUV1);
          pMate->SetPoint(sPnt1);
          pMate->SetStatus(SM_FIL_PROCESSED);
          break;

        } // end iter over potential mates

      // done with this FilletVertex
      SetStatus( SM_FIL_PROCESSED );

#ifdef SM_DEBUG_CODE
      if ( bDebugFVUs )
        { pFilletGeom2->DumpFilletVUs(); }
#endif

    } // end pFilletGeom2 existence check

  return SM_SUCCESS;

} // end SmFilletVertex::CalcRailIntEdgeuse

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_ON_VERTEX.

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::CalcVertOnVert()
{
  SmVertex * pVert = (SmVertex*)GetPointClassObject();
  SetPoint(pVert->GetPoint());
  SetStatus(SM_FIL_PROCESSED);

  return SM_SUCCESS;

} // end SmFilletVertex::CalcVertOnVert

/*******************************************************************//**
PURPOSE: Compute fillet vertex which is an end point of the intersection
  curve of two fillets for Nx2-concave cases


NOTES:
***********************************************************************/
static SmStatus sm_CalcNx2VertFilletXFillet
  (SmFilletVertex * pFV)
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = pFV->DebugLevel();
#endif // SM_DEBUG_CODE

  SmPoint3d          sRefPnt;
  double             dApproxTol      = SM_BIG_DOUBLE ;
  double             dAngTol         = SM_BIG_DOUBLE ;

  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 2 line
  // rm : SmBSplineSurface * pFilletSurface1 = NULL ;
  // rm : SmBSplineSurface * pFilletSurface2 = NULL ;
  SM_FILLETSURF_TYPE * pFilletSurface1 = NULL ;
  SM_FILLETSURF_TYPE * pFilletSurface2 = NULL ;

  SmTArray<SmEdge*> sEdges;
  pFV->GetEdges(sEdges);
  for(ULONG i=0; i<sEdges.GetSize(); i++) 
    {
      SmFilletEdge * pFE = (SmFilletEdge*)sEdges[i] ;
      switch (pFE->GetFilletEdgeType()) 
        {
          case SM_FE_CROSS_SECTION:
            {
              SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFE->GetPrimaryEdgeuse() ;
              SmFilletGeom    * pFG     = pPrimEU->GetFilletGeom() ; NER(pFG) ;
              SmFilletSolver  * pFS     = pFG->GetFilletSolver() ;

              dApproxTol = smos_Min(dApproxTol,pFS->GetThisApproxTol3d()) ;
              dAngTol    = smos_Min(dAngTol,pFS->GetThisAngTolRad()) ;

              if(pFilletSurface1 == NULL) 
                { pFilletSurface1 = pFG->GetFilletSurface() ; }
              else 
                { pFilletSurface2 = pFG->GetFilletSurface() ; }
              break;
            }
          case SM_FE_FILLET_X_SIDE_FACE:
            {
              SmFilletVertex * pOtherFV = (SmFilletVertex*)pFE->GetOtherVertex(pFV);
              if(!pOtherFV->IsProcessed()) 
                { SER(SM_ERR) ; }
              sRefPnt = pOtherFV->GetPoint();
              break;
            }
          default:
            break;
        } // end switch
    } // end iter every sEdges

  // Do surface/surface intersection
  SmBSplineCurve * p3DCurve = NULL;
  SmBSplineCurve * pUVCurve1 = NULL;
  SmBSplineCurve * pUVCurve2 = NULL;
  SmPoint3d sRefPoint = pFV->GetPoint();
  const SmContext & crContext = pFV->GetFilletCorner()->GetContext();
  if(   SM_SUCCESS != sm_SrfSrfIntersection(crContext,       // in : context for new object construction
                                            NULL,            // in : 1st point known to be on intersection curve
                                            NULL,            // in : 2nd point known to be on intersection curve,
                                                             //      NULL to ignore
                                            NULL,            // in : expected general direction of intersection curve from 1st point
                                            NULL,            // in   expected intersection end direction
                                            pFilletSurface1, // in : 1st intersecting surface
                                            pFilletSurface2, // in : 2nd intersecting surface
                                            dApproxTol,      // in : max allowed distance between xSect Curve and surfaces
                                            dAngTol,         // in : max allowed angle between consecutive xSect curve segment tangents
                                            p3DCurve,        // out: 3d intersection curve
                                            pUVCurve1,       // out: associated UVTrimCurve on 1st surface
                                            pUVCurve2,       // out: associated UVTrimCurve on 2nd surface
                                            FALSE,           // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                             //             resorting to expensive general surf/surf xSect solver
                                            &sRefPoint,      // in : reference point
                                            iDebugLevel)     // in : iDebugLevel, 0 = No Debug output
     || !p3DCurve)
    {
      pFV->SetStatus(SM_FIL_SURF_INT_FAILURE);
      SER(SM_ERR);
    }

  SmObjDelete sDelete1(p3DCurve);
  SmObjDelete sDelete2(pUVCurve1);
  SmObjDelete sDelete3(pUVCurve2);

  SmPoint3d   sPnt1, sPnt2 ;
  SmExtent1d  sIvl      = p3DCurve->GetNaturalInterval() ;
  SER(p3DCurve->EvaluatePoint(sIvl.GetMin(), sPnt1)) ;
  SER(p3DCurve->EvaluatePoint(sIvl.GetMax(), sPnt2)) ;
  SmPoint3d   sVertGeom = sPnt1 ;

  if(sRefPnt.DistanceBetween(sPnt1) > sRefPnt.DistanceBetween(sPnt2)) 
    { sVertGeom = sPnt2 ; }

  pFV->SetPoint(sVertGeom) ;
  pFV->SetStatus(SM_FIL_PROCESSED) ;

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 ) 
    {
      smgfx_SetLook(1,3, 1,0,0); sVertGeom.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,6, 0,0,1); sRefPnt.Draw();   sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end sm_CalcNx2VertFilletXFillet

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_FILLET_X_FILLET.

NOTES:
  sets this FilletVertex->Point = Point on filletSurface/filletSurface xSectCurve

METHOD ---
  1. find filletEdge connected to this filletVertex which represents
    the filletSurface/filletSurface intersection.

  2. make sure it has a 3dCurve
    if (filletEdge->Curve == NULL) compute and set
      filletEdge->Curve,
      filletEdge->Edgeuses->UVCurves,
      filletEdge->Status = processed

  3. Set this FilletVertex->Point  = xSectCurve->Point
              FilletVertex->Status = processed

  returns SM_ERR when attempt to compute xSectCurve fails

***********************************************************************/
SmStatus SmFilletVertex::CalcFilletIntFillet
  ()
{
  // check state
  NER(m_cpCorner);

  // First, check if we have an adjusted Nx2 Concave corner
  if (   m_cpCorner->GetCornerType() == SM_FCR_N_x_2
   // && !m_cpCorner->IsBlendingCorner()
      && !m_cpCorner->IsBevelCorner())
    {
      SER(sm_CalcNx2VertFilletXFillet(this));
      return SM_SUCCESS;
    }

  // get CornerFillet->OrigVertex
  const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex();

  // Compute fillet vertex which is an end point of the intersection
  // curve of two fillets. Note: the other end of intersection
  // should have already been computed (e.g. from rail-rail intersection)

  // find the filletVertex on the other end of the intersection curve
  // by finding the filletEdge defined by the filletSurface/filletSurface intersection
  SmFilletEdge * pCurrEdge = NULL;
  SmTArray<SmEdge*> sEdges;
  GetEdges(sEdges);
  for (ULONG i=0; i<sEdges.GetSize(); i++)
    {
      SmFilletEdge * pFilEdge = (SmFilletEdge*)sEdges[i];
      if (pFilEdge->GetFilletEdgeType() == SM_FE_FILLET_X_FILLET)
        {
          pCurrEdge = pFilEdge;
          break;
        }
    }
  NER(pCurrEdge);

  // get filletFVertex->Point at other end of intersection curve
  SmFilletVertex * pOtherFV = (SmFilletVertex*)pCurrEdge->GetOtherVertex(this);
  if (!pOtherFV->IsProcessed())
    {
      SER(SM_ERR);
    }
  SmPoint3d sOtherPnt = pOtherFV->GetPoint();

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
  iDebugLevel = DebugLevel();
  if ( iDebugLevel > 0 )
    {
      sm_GraphicsLoop();
      smgfx_SetLook(4,6, 0,0,1); pOtherFV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); cpVertex->GetBrep()->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals from intersection FilletEdge
  SmFilletEdgeuse * pPrimEU   = (SmFilletEdgeuse*)pCurrEdge->GetPrimaryEdgeuse();
  SmFilletEdgeuse * pMateEU   = (SmFilletEdgeuse*)pPrimEU->GetMate();
  SmBSplineCurve  * p3DCurve  = SM_CAST_PTR(SmBSplineCurve,pCurrEdge->GetCurve());
  SmBSplineCurve  * pUVCurve1 = pPrimEU->GetUVTrimCurvePointer();
  SmBSplineCurve  * pUVCurve2 = pMateEU->GetUVTrimCurvePointer();

  // when intersection FilletEdge->Curve has not yet been computed
  if (p3DCurve == NULL)
    {
      // Do surface/surface intersection

      // get intersecting filletSurface FilletGeoms, FilletSolvers, and FilletSurfaces
      SmFilletGeom * pFilletGeom1 = pPrimEU->GetFilletGeom() ;
      SmFilletGeom * pFilletGeom2 = pMateEU->GetFilletGeom() ;
      NER(pFilletGeom1) ;
      NER(pFilletGeom2) ;
      SmFilletSolver   * pFilSolver1     = pFilletGeom1->GetFilletSolver() ;
      SmFilletSolver   * pFilSolver2     = pFilletGeom2->GetFilletSolver() ;

      // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 2 line
      // rm : SmBSplineSurface * pFilletSurface1 = pFilletGeom1->GetFilletSurface() ;
      // rm : SmBSplineSurface * pFilletSurface2 = pFilletGeom2->GetFilletSurface() ;
      SM_FILLETSURF_TYPE * pFilletSurface1 = pFilletGeom1->GetFilletSurface() ;
      SM_FILLETSURF_TYPE * pFilletSurface2 = pFilletGeom2->GetFilletSurface() ;

      // check state - filletSurface must have been computed by now
      if (   !pFilletSurface1
          || !pFilletSurface2)
        {
          return SM_SUCCESS;// Fillet surfaces have not yet been computed
        }

      // pick appoxTol and angTol as tightest filletSolver tolerances
      double dApproxTol = smos_Min(pFilSolver1->GetThisApproxTol3d(),
                                   pFilSolver2->GetThisApproxTol3d());
      double dAngTol    = smos_Min(pFilSolver1->GetThisAngTolRad(),
                                   pFilSolver2->GetThisAngTolRad());

      // set sDir = fillet chord (vector connecting intersection endPoints)
      SmVector3d sDir = cpVertex->GetPoint() - sOtherPnt;
      SmBoolean bSkipTwoPntsIntersection = TRUE;

      // find the surface/surface intersection curve knowing only 1 end-point
      if(SM_SUCCESS != sm_SrfSrfIntersection(m_cpCorner->GetContext(), // in : context for new object construction
                                             pOtherFV,                 // in : 1st point known to be on intersection curve
                                             NULL,                     // in : 2nd point known to be on intersection curve,
                                                                       //      NULL to ignore
                                             &sDir,                    // in : expected general direction of intersection curve from 1st point
                                             NULL,                     // in   expected intersection end direction
                                             pFilletSurface1,          // in : 1st intersecting surface
                                             pFilletSurface2,          // in : 2nd intersecting surface
                                             dApproxTol,               // in : max allowed distance between xSect Curve and surfaces
                                             dAngTol,                  // in : max allowed angle between consecutive xSect curve segment tangents
                                             p3DCurve,                 // out: 3d intersection curve
                                             pUVCurve1,                // out: associated UVTrimCurve on 1st surface
                                             pUVCurve2,                // out: associated UVTrimCurve on 2nd surface
                                             bSkipTwoPntsIntersection, // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                                       //             resorting to expensive general surf/surf xSect solver
                                             NULL,                     // in : reference point
                                             iDebugLevel )             // in : iDebugLevel, 0 = No Debug output
          || !p3DCurve)
        { // error: sm_SrfSrfIntersection() failed or failed to produce a p3DCurve
          this->SetStatus(SM_FIL_SURF_INT_FAILURE);
          pCurrEdge->SetStatus(SM_FIL_SURF_INT_FAILURE);
          SER(SM_ERR);
        }

      // save outputs: pCurrEdge->Curve, pCurrEdge->Edgeuses->UVCurves, pCurrEdge->Status = processed
      pCurrEdge->SetCurve(p3DCurve, FALSE) ; // FALSE = don't delete preExisting Curve - because pCurrEdge->Curve == NULL
                                             // side effect: delete current pEdge->UVTrimCurves
      p3DCurve->SetOwner(pCurrEdge);
      SmExtent1d sCrvIvl = p3DCurve->GetNaturalInterval();
      pCurrEdge->SetInterval(sCrvIvl);
      pPrimEU->SetUVCurve(pUVCurve1);
      pMateEU->SetUVCurve(pUVCurve2);
      pCurrEdge->SetStatus(SM_FIL_PROCESSED);

    } // end FilletEdge->3dCurve existence check

  // arrive here after guaranteeing that intersection curve is computed
  // and stored in pFilletEdge->Curve

  // get intersection curve endPoints
  SmPoint3d sPnt1, sPnt2;
  SmExtent1d sIvl = p3DCurve->GetNaturalInterval();
  SER(p3DCurve->EvaluatePoint(sIvl.GetMin(), sPnt1));
  SER(p3DCurve->EvaluatePoint(sIvl.GetMax(), sPnt2));

  // use xSect point closest to this vertex
  // to set vertex position and 3dCurve direction
  SmBoolean bOtherPointCloserToPnt1 = (sOtherPnt.DistanceBetween(sPnt1) < sOtherPnt.DistanceBetween(sPnt2)) ;
  SmPoint3d sVertGeom      = bOtherPointCloserToPnt1
                             ? sPnt2
                             : sPnt1;
  SmBoolean bReverseCurves =   (   (pCurrEdge->GetVertex() == this &&  bOtherPointCloserToPnt1)
                                || (pCurrEdge->GetVertex() != this && !bOtherPointCloserToPnt1))
                             ? TRUE
                             : FALSE ;

  // when 3dCurve needs to be reversed
  if (bReverseCurves)
    {
      // reverse it and its uvTrimCurves
      SER(p3DCurve->ReverseParameterization(sIvl,sIvl));
      SER(pUVCurve1->ReverseParameterization(sIvl,sIvl));
      SER(pUVCurve2->ReverseParameterization(sIvl,sIvl));
    }

  // set this filletVertex point and status values
  SetPoint(sVertGeom);
  SetStatus(SM_FIL_PROCESSED);

  // all done
  return SM_SUCCESS;

} // end SmFilletVertex::CalcFilletIntFillet

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_FILLET_X2_FILLETS.
    (i.e. intersection of three fillets at corner)

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::CalcFilletInt2Fillets
  ()
{
  NER(m_cpCorner) ;

  const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex() ;

  // Compute fillet vertex which is the intersecion of 3 incoming fillets
  SmBSplineCurve * pCurve1 = NULL ; //Intersection curve of fillet#1 & #2
  SmBSplineCurve * pCurve2 = NULL ; //Intersection curve of fillet#2 & #3
  double           dMinTol = SM_BIG_DOUBLE;
  SmTArray<SmEdge*> sEdges;
  GetEdges(sEdges);

  for (ULONG i=0; i<sEdges.GetSize(); i++) 
    {
      SmFilletEdge * pCurrEdge = (SmFilletEdge*)sEdges[i];
      if(pCurrEdge->GetFilletEdgeType() != SM_FE_FILLET_X_FILLET) 
        { continue ; }

      SmFilletVertex * pOtherFV = (SmFilletVertex*)pCurrEdge->GetOtherVertex(this);
      if (!pOtherFV->IsProcessed()) 
        { SER(SM_ERR) ; }

      SmPoint3d sOtherPnt = pOtherFV->GetPoint();

      // Do surface/surface intersection
      SmFilletEdgeuse * pPrimEU      = (SmFilletEdgeuse*)pCurrEdge->GetPrimaryEdgeuse() ;
      SmFilletEdgeuse * pMateEU      = (SmFilletEdgeuse*)pPrimEU->GetMate() ;
      SmFilletGeom    * pFilletGeom1 = pPrimEU->GetFilletGeom() ;
      SmFilletGeom    * pFilletGeom2 = pMateEU->GetFilletGeom() ;
      NER(pFilletGeom1) ; NER(pFilletGeom2) ;
      SmFilletSolver  * pFilSolver1  = pFilletGeom1->GetFilletSolver() ;
      SmFilletSolver  * pFilSolver2  = pFilletGeom2->GetFilletSolver() ;
        
      // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 2 line
      // rm : SmBSplineSurface * pFilletSurface1 = pFilletGeom1->GetFilletSurface() ;
      // rm : SmBSplineSurface * pFilletSurface2 = pFilletGeom2->GetFilletSurface() ;
      SM_FILLETSURF_TYPE * pFilletSurface1 = pFilletGeom1->GetFilletSurface() ;
      SM_FILLETSURF_TYPE * pFilletSurface2 = pFilletGeom2->GetFilletSurface() ;

      if(!pFilletSurface1 || !pFilletSurface2) 
        { return SM_SUCCESS ; } // Fillet surfaces have not yet been computed

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
      iDebugLevel = DebugLevel();
      if ( iDebugLevel > 0 ) 
        {
          smgfx_SetColor(1,1,0) ; cpVertex->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetColor(0,0,1) ; pOtherFV->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetColor(1,0,0) ; pFilletSurface1->DrawUV(0,0) ; sm_GraphicsLoop() ;
          smgfx_SetColor(1,0,0) ; pFilletSurface2->DrawUV(0,0) ; sm_GraphicsLoop() ;
          smgfx_SetColor(0,0,0) ; cpVertex->GetBrep()->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      double dApproxTol = smos_Min(pFilSolver1->GetThisApproxTol3d(),
                                   pFilSolver2->GetThisApproxTol3d());
      double dAngTol    = smos_Min(pFilSolver1->GetThisAngTolRad(),
                                   pFilSolver2->GetThisAngTolRad());
      if (dApproxTol < dMinTol) dMinTol = dApproxTol;
      SmBSplineCurve * pNew3DCurve = NULL;
      SmBSplineCurve * pUVCurve1 = NULL;
      SmBSplineCurve * pUVCurve2 = NULL;
      SmVector3d       sDir = cpVertex->GetPoint() - sOtherPnt;
      SmBoolean        bSkipTwoPntsIntersection = TRUE;

      if(SM_SUCCESS != sm_SrfSrfIntersection(m_cpCorner->GetContext(), // in : context for new object construction
                                             pOtherFV,                 // in : 1st point known to be on intersection curve
                                             NULL,                     // in : 2nd point known to be on intersection curve,
                                                                       //      NULL to ignore
                                             &sDir,                    // in : expected general direction of intersection curve from 1st point
                                             NULL,                     // in   expected intersection end direction
                                             pFilletSurface1,          // in : 1st intersecting surface
                                             pFilletSurface2,          // in : 2nd intersecting surface
                                             dApproxTol,               // in : max allowed distance between xSect Curve and surfaces
                                             dAngTol,                  // in : max allowed angle between consecutive xSect curve segment tangents
                                             pNew3DCurve,              // out: 3d intersection curve
                                             pUVCurve1,                // out: associated UVTrimCurve on 1st surface
                                             pUVCurve2,                // out: associated UVTrimCurve on 2nd surface
                                             bSkipTwoPntsIntersection, // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                                       //             resorting to expensive general surf/surf xSect solver
                                             NULL,                     // in : reference point
                                             iDebugLevel )             // in : iDebugLevel, 0 = No Debug output
          || pNew3DCurve == NULL )
        {
          this->SetStatus(SM_FIL_SURF_INT_FAILURE);
          pCurrEdge->SetStatus(SM_FIL_SURF_INT_FAILURE);
          SER(SM_ERR);
        }

      pCurrEdge->SetCurve(pNew3DCurve, TRUE); // side effect: delete current pCurrEdge->UVTrimCurves
      pNew3DCurve->SetOwner(pCurrEdge);
      SmExtent1d sCrvIvl = pNew3DCurve->GetNaturalInterval();
      pCurrEdge->SetInterval(sCrvIvl);
      pPrimEU->SetUVCurve(pUVCurve1);
      pMateEU->SetUVCurve(pUVCurve2);

      // Determine whether the curves need to be reversed
      SmExtent1d sIvl = pNew3DCurve->GetNaturalInterval();
      SmPoint3d sPnt1, sPnt2;
      SER(pNew3DCurve->EvaluatePoint(sIvl.GetMin(), sPnt1));
      SER(pNew3DCurve->EvaluatePoint(sIvl.GetMax(), sPnt2));
      SmBoolean bReverseCurves = FALSE;
      if (pCurrEdge->GetVertex() == this) 
        {
          if(sOtherPnt.DistanceBetween(sPnt1) < sOtherPnt.DistanceBetween(sPnt2)) 
            {  bReverseCurves = TRUE ; }
        }
      else 
        {
          if(sOtherPnt.DistanceBetween(sPnt1) < sOtherPnt.DistanceBetween(sPnt2)) 
            {
            }
          else 
            { bReverseCurves = TRUE ; }
        }

      //
      if (bReverseCurves) 
        {
          SER(pNew3DCurve->ReverseParameterization(sIvl,sIvl)) ;
          SER(pUVCurve1->ReverseParameterization(sIvl,sIvl)) ;
          SER(pUVCurve2->ReverseParameterization(sIvl,sIvl)) ;
        }

      if      (pCurve1 == NULL) pCurve1 = pNew3DCurve ;
      else if (pCurve2 == NULL) pCurve2 = pNew3DCurve ;
    }

  // Now find the common intersection of all the curves.
  SmSolution      aData[4];
  SmSolutionArray sSolutions(4,aData);

  //
  SER(pCurve1->GlobalCurveIntersect(pCurve1->GetNaturalInterval(),
                                   *pCurve2,
                                    pCurve2->GetNaturalInterval(),
                                    dMinTol,
                                    sSolutions)) ;
  if (sSolutions.GetSize() != 1) SER(SM_ERR) ;

  SmSolution & rSol = sSolutions[0];
  SmVector3d   sVertGeom;

  SER(pCurve1->EvaluatePoint(rSol.m_vStart[0],sVertGeom));
  SetPoint(sVertGeom);
  SetStatus(SM_FIL_PROCESSED);

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetColor(1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetColor(0,0,0) ; cpVertex->GetBrep()->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmFilletVertex::CalcFilletInt2Fillets

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_RAIL_X_EXTENDED_EDGEUSE.

NOTES:
  vertex will be attached to a filletEdge of types
  SM_FE_FILLET_X_SIDE_FACE or SM_FCR_3_x_2_MIXED.

  When vertex->FilletEdgeType == SM_FE_FILLET_X_SIDE_FACE
    Find FilletSurface/endSurface intersection curve
    set FilletEdge->SetCurve(3DCurve)
        FilletEdge->PrimaryEU->SetUVCurve(PUVCurve1)
        FilletEdge->PrimaryEU->Mate->SetUVCurve(PUVCurve2)
        3DCurve->SetOwner(pFilletEdge)

        FilletVertex->SetPoint(3DCurve->EndPoint)
        FilletVertex->SetStatus(SM_FIL_PROCESSED)

  else  vertex->FilletEdgeType == SM_FCR_3_x_2_MIXED
    get surface/surface intersection
        FilletVertex->SetPoint(surface/surface xSectCurve->EndPoint projection onto surface)
        FilletVertex->SetStatus(SM_FIL_PROCESSED)
***********************************************************************/
SmStatus SmFilletVertex::CalcRailIntExtendedEdgeuse()
{
  // locals
  SmBoolean b3x2MixedCornerCase =  (m_cpCorner && m_cpCorner->GetCornerType() == SM_FCR_3_x_2_MIXED)
                                  ? TRUE
                                  : FALSE;

  // Find adjacent fillet edge which was classifed
  //   as SM_FE_FILLET_X_SIDE_FACE in regular cases,
  //   or as SM_FE_CROSS_SECTION in 3x2MixedConvexity cases
  enum SmFilletEdgeType eFEType =  (b3x2MixedCornerCase)
                                  ? SM_FE_CROSS_SECTION
                                  : SM_FE_FILLET_X_SIDE_FACE ;
  SmFilletEdge    * pCurrEdge = NULL;
  SmTArray<SmEdge*> sEdges;
  GetEdges(sEdges);

  //
  for (ULONG i=0; i<sEdges.GetSize(); i++)
    {
      SmFilletEdge * pFilEdge = (SmFilletEdge*)sEdges[i];
      if (pFilEdge->GetFilletEdgeType() == eFEType)
        {
          pCurrEdge = pFilEdge;
          break;
        }
    } // end iter every vertex->edge searching for one with desired classification
  NER(pCurrEdge);

  // set up surface/surface intersection

  // get FilletSurface stored in filletEdge->PrimaryEU->FilletGeom->FilletSurface
  SmFilletEdgeuse * pPrimEU     = (SmFilletEdgeuse*)pCurrEdge->GetPrimaryEdgeuse();
  SmFilletEdgeuse * pMateEU     = (SmFilletEdgeuse*)pPrimEU->GetMate();
  SmFilletGeom    * pFilletGeom = pPrimEU->GetFilletGeom();
  NER(pFilletGeom);
  
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pFilletSurface = pFilletGeom->GetFilletSurface();
  SM_FILLETSURF_TYPE * pFilletSurface = pFilletGeom->GetFilletSurface();

  if (!pFilletSurface)
    {
      // Need fillet surface, will process it later
      return SM_SUCCESS;
    }

  // get OtherFaceSurface
  SmFilletVertex * pOtherFV  = NULL;
  SmFace         * pSideFace = NULL; // Face used for surface-surface-interection
  SmFace         * pOnFace   = NULL; // Face where 'THIS' is on(for 'SM_FCR_3_x_2_MIXED' cases only)
  if (b3x2MixedCornerCase)
    {
      // Will intersect fillet with a side-face of the original brep
      SmEdgeuse * pSideEU = (SmEdgeuse*)GetPointClassObject() ;
      NER(pSideEU) ;
      pSideFace = pSideEU->GetFace() ;
      pOnFace   = pSideEU->GetRadial()->GetFace() ;
      for (ULONG kk=0; kk<2; kk++)
        {
          SmEdgeuse * pEU = pFilletGeom->GetFilletSolver()->GetEdgeuse(kk) ;
          if (pEU->GetFace() == pSideFace)
            {
              SM_SWAP_PTR(SmFace,pSideFace,pOnFace) ;
              break ;
            }
        }
    } // end b3x2MixedCornerCase branch
  else // pCurrEdge == SM_FE_FILLET_X_SIDE_FACE branch
    {
      pOtherFV = (SmFilletVertex*)pCurrEdge->GetOtherVertex(this) ;

      // check state - the other filletVertex is already processed
      // That vertex will be on a sideEdge not an extendedSideEdge
      if (!pOtherFV->IsProcessed())
        {
          SER(SM_ERR) ;
        }

      // get face (sometimes its extended) upon which this filletEdge was originated
      pSideFace = pCurrEdge->GetOriginalFace() ;
    }
  NER(pSideFace) ;

  // get face->extendedSurface it exists
  SmSurface        * pSurface  = m_cpCorner->GetExtendedSurface(pSideFace);
  SmBSplineSurface * pSurface2 = SM_CAST_PTR(SmBSplineSurface,pSurface);
  NER(pSurface2);

  // locals surface/surface intersection
  SmFilletSolver * pFilSolver  = pFilletGeom->GetFilletSolver() ;
  double           dApproxTol  = pFilSolver->GetThisApproxTol3d() ;
  double           dAngTol     = pFilSolver->GetThisAngTolRad() ;
  SmBSplineCurve * pNew3DCurve = NULL ;
  SmBSplineCurve * pUVCurve1   = NULL ;
  SmBSplineCurve * pUVCurve2   = NULL ;

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  // do surface/surface intersection
  if (SM_SUCCESS != sm_SrfSrfIntersection(m_cpCorner->GetContext(), // in : context for new object construction
                                          pOtherFV,                 // in : 1st point known to be on intersection curve
                                          NULL,                     // in : 2nd point known to be on intersection curve,
                                                                    //      NULL to ignore
                                          NULL,                     // in : expected general direction of intersection curve from 1st point
                                          NULL,                     // in   expected intersection end direction
                                          pFilletSurface,           // in : 1st intersecting surface
                                          pSurface2,                // in : 2nd intersecting surface
                                          dApproxTol,               // in : max allowed distance between xSect Curve and surfaces
                                          dAngTol,                  // in : max allowed angle between consecutive xSect curve segment tangents
                                          pNew3DCurve,              // out: 3d intersection curve
                                          pUVCurve1,                // out: associated UVTrimCurve on 1st surface
                                          pUVCurve2,                // out: associated UVTrimCurve on 2nd surface
                                          FALSE,                    // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                                    //             resorting to expensive general surf/surf xSect solver
                                          NULL,                     // in : reference point
                                          iDebugLevel )             // in : iDebugLevel, 0 = No Debug output
      || !pNew3DCurve)
    {
      this->SetStatus(SM_FIL_SURF_INT_FAILURE);
      pCurrEdge->SetStatus(SM_FIL_SURF_INT_FAILURE);
      SER_MSG(SM_ERR, _T("CalcRailIntExtendedEdgeuse(): Suffered a sm_SrfSrfIntersection() failure"));
    }

#ifdef SM_DEBUG_CODE
  // draw
  if ( iDebugLevel > 0 )
    {
      smgfx_SetColor(1,0,0) ; pNew3DCurve->DrawWDeriv(pNew3DCurve->GetNaturalInterval(),0) ; sm_GraphicsLoop() ;
      smgfx_SetColor(0,1,1) ; pFilletSurface->DrawUV(4,4) ; sm_GraphicsLoop() ;
      smgfx_SetColor(0.5,1,0) ; pSideFace->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetColor(0.5,1,0) ; pSurface2->DrawUV(4,4) ; sm_GraphicsLoop() ;
      smgfx_SetColor(0.5,1,0) ; pSideFace->GetBrep()->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // get surface/surface xSectCurve->endPoints
  SmExtent1d sIvl = pNew3DCurve->GetNaturalInterval( );
  SmPoint3d sPnt1, sPnt2 ;
  SER(pNew3DCurve->EvaluatePoint(sIvl.GetMin(), sPnt1)) ;
  SER(pNew3DCurve->EvaluatePoint(sIvl.GetMax(), sPnt2)) ;

  //
  if (b3x2MixedCornerCase)
    {
      // Destroy intersection curves
      SM_ASSERT(pNew3DCurve != NULL) ; delete pNew3DCurve ; pNew3DCurve = NULL ;
      SM_ASSERT(pUVCurve1   != NULL) ; delete pUVCurve1 ;   pUVCurve1   = NULL ;
      SM_ASSERT(pUVCurve2   != NULL) ; delete pUVCurve2 ;   pUVCurve2   = NULL ;

      // Find which end point lies on the pOnFace
      SmSurface     * pSurf1 = pOnFace->GetSurface() ;
      SmSolutionArray sSolutions ;
      double          dTol   = pFilSolver->GetThisApproxTol3d() ;

      SER(pSurf1->GlobalPointSolve(pSurf1->GetNaturalUVDomain(),
                                   SM_SO_INTERSECT,
                                   sPnt1,
                                   10.0*dTol,
                                   NULL,
                                   SM_SR_ALL,
                                   sSolutions)) ;

      if (sSolutions.GetSize() > 0)
        {
          SetPoint(sPnt1);
          SetStatus(SM_FIL_PROCESSED) ;
        }
      else
        {
          SER(pSurf1->GlobalPointSolve(pSurf1->GetNaturalUVDomain(),
                                       SM_SO_INTERSECT,
                                       sPnt2,
                                       10.0*dTol,
                                       NULL,
                                       SM_SR_ALL,
                                       sSolutions)) ;
          if (sSolutions.GetSize() > 0)
            {
              SetPoint(sPnt2);
              SetStatus(SM_FIL_PROCESSED);
            }
          else
            {
              SER(SM_ERR);
            }
        }

      return SM_SUCCESS;

    } // end b3x2MixedCornerCase check

  // arrive here for SM_FE_FILLET_X_SIDE_FACE cases
  SmPoint3d sVertGeom      = sPnt1;
  SmBoolean bReverseCurves = FALSE;
  SmPoint3d sOtherPnt      = pOtherFV->GetPoint();

#ifdef SM_DEBUG_CODE
  // draw OtherFilletVertex(blue)
  if ( DebugLevel() > 0 )
    {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1); pOtherFV->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // make sure curve is parameterized from edge->StartVert to edge->EndVert
  if (pCurrEdge->GetVertex() == this)
    {
      if (sOtherPnt.DistanceBetween(sPnt1) < sOtherPnt.DistanceBetween(sPnt2))
        {
          sVertGeom      = sPnt2;
          bReverseCurves = TRUE;
        }
    }
  else
    {
      if (sOtherPnt.DistanceBetween(sPnt1) < sOtherPnt.DistanceBetween(sPnt2))
        {
          sVertGeom = sPnt2;
        }
      else
        {
          bReverseCurves = TRUE;
        }
    }

  // reverse FilletEdge parameterization when needed
  if (bReverseCurves)
    {
      SER(pNew3DCurve->ReverseParameterization(sIvl,sIvl));
      SER(pUVCurve1->ReverseParameterization(sIvl,sIvl));
      SER(pUVCurve2->ReverseParameterization(sIvl,sIvl));
    }

  // Set edge geometry
  pCurrEdge->SetCurve(pNew3DCurve, TRUE)  ; // side effect: delete current pCurrEdge->UVTrimCurves
  pNew3DCurve->SetOwner(pCurrEdge) ;
  pCurrEdge->SetInterval(sIvl) ;
  pPrimEU->SetUVCurve(pUVCurve1) ;
  pMateEU->SetUVCurve(pUVCurve2) ;

  // Set vertex geometry
  SetPoint(sVertGeom) ;
  SetStatus(SM_FIL_PROCESSED) ;

  // all done
  return SM_SUCCESS ;

} // end SmFilletVertex::CalcRailIntExtendedEdgeuse

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_ON_CROSS_SECTION,
    such as in corner case SmFillet1x1Corner. The vertex is on the cross-
    sectional plane at the filleted vertex.

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::CalcVertOnCrossSection
  (double * pOptEdgeParam)
{
    NER(m_cpCorner);
    const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex();

    // Determine where the cross-sectional plane is
    SmTArray<SmFilletGeom*> sGeoms;
    GetFilletGeoms(sGeoms);
    ULONG lTotalGeoms = sGeoms.GetSize();
    if (lTotalGeoms < 1) SER(SM_ERR);
    SmFilletGeom * pFilletGeom1 = sGeoms[0];
    SmFilletSolver * pFilSolver1 = pFilletGeom1->GetFilletSolver();
    NER(pFilSolver1);
    SmEdgeuse * pEU = pFilSolver1->GetEdgeuse(0);
    SmEdge * pEdge = pEU->GetEdge();
    SmExtent1d sIvl = pEdge->GetInterval();
    SmBoolean bVertAtEdgeStart = TRUE;
    double dParam = sIvl.GetMin();
    if (pOptEdgeParam) {
        dParam = *pOptEdgeParam;
    }
    else if (pEdge->GetStartVertex() != cpVertex) {
        dParam = sIvl.GetMax();
        bVertAtEdgeStart =  FALSE;
    }
    SmVector3d sPV[2];
    SmCurve * pCurve = pEdge->GetCurve(); NER(pCurve);
    SER(pCurve->Evaluate(dParam,1,TRUE,sPV));
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(1,0,0);
        pEU->Draw();
        sm_GraphicsLoop();
        smgfx_SetPointSize(6);
        sPV[0].Draw();
        sPV[1].Draw(&sPV[0]);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    SmPoint3d  sPlaneOrig = sPV[0];
    SmVector3d sPlaneNormal = sPV[1];
    SmFilletVertexuse * pVU1 = GetVUAtRailEnd(pFilletGeom1);
    SmTsectPnt & rTsectPnt = pVU1->GetTsectPnt();

    SmTArray<SmVertexuse*> sVertexuses;
    cpVertex->GetVertexuses(sVertexuses);
    SmVector2d sUVs[2];
    for (ULONG i=0; i<2; i++) {
        SmFace * pFace = pFilSolver1->GetEdgeuse(i)->GetFace();
        SmBoolean bFound = FALSE;
        for (ULONG jj=0; jj<sVertexuses.GetSize() && !bFound; jj++) {
            SmVertexuse * pVU = sVertexuses[jj];
            if (pVU->GetFaceuse()->GetFace() == pFace) {
                bFound = TRUE;
                pVU->ComputeUVPoint(sUVs[i]);
            }
        }
    }

    // Determine if we need to step off the edge to avoid
    // convergence problems (some surfaces may have colinear dU & dV)
    SmOffsetSurface * pOffsetSurf = pFilSolver1->GetSurface(0);
    double dStepoffDist = smos_Fabs(pOffsetSurf->GetOffsetDistance());
    double dCurveStepoffDist = dStepoffDist;
    double dCurveLen = pCurve->ApproximateLength(sIvl,10);
    SM_ASSERT(dCurveLen > SM_EFF_ZERO);
    double dMaxStepoffDist = dCurveLen*0.15;
    if (dCurveStepoffDist > dMaxStepoffDist) {
        dCurveStepoffDist = dMaxStepoffDist;
    }
    if (!bVertAtEdgeStart) {
        dCurveStepoffDist = -dCurveStepoffDist;
    }

    for (ULONG j=0; j<2; j++) {
        SmSurface * pSurf = pFilSolver1->GetSurface(j);
        NER(pSurf);
        SmVector3d sDU, sDV;
        SmVector3d sMat[2][2];
        SER(pSurf->Evaluate(sUVs[j],1,1,TRUE,TRUE,TRUE,sMat[0]));
        sDU = sMat[1][0];
        sDV = sMat[0][1];
        double dDeg;
        SER(sDU.AngleBetween(sDV,dDeg));
        dDeg = SM_RAD2DEG(dDeg);
        if (dDeg < 30 || dDeg > 150.0) {
            // May lead to potential failures, step off the boundary
            // Step off the vertex along the edge first
            SER(pCurve->FindParameterAtArcLength(dParam,
                2.0*dCurveStepoffDist,dParam));
            // Then step off the boundary edge
            SER(sm_FindStepoffUV(pFilSolver1,dParam,
                dStepoffDist,sUVs));
            break;
        }
    }

    SmExtent2d sDomain1 = pFilSolver1->GetSurface(0)->GetNaturalUVDomain();
    SmExtent2d sDomain2 = pFilSolver1->GetSurface(1)->GetNaturalUVDomain();
    SmBoolean bFoundSolution = FALSE;

    // If the solver doesn't succeed the first time, try stepping off
    // the vertex along the edge.  Loop a couple of times.
    for (ULONG k=0; k<3; k++)
      {
        // find filletPoint on given plane that satisfies geometry requirements
        // implemented in derived SmFilletSolver class
        SER( pFilSolver1->PointOnPlaneSolve(
                sPlaneOrig, sPlaneNormal, sDomain1, sDomain2, sUVs[0], sUVs[1],
                bFoundSolution, rTsectPnt ));
        if ( bFoundSolution )
            break;

        // No solution found there, step off the vertex along the edge
        // and try again.
        SER( pCurve->FindParameterAtArcLength( dParam,
            dCurveStepoffDist, dParam ));

        // Then step off the boundary edge
        SER( sm_FindStepoffUV( pFilSolver1, dParam,
            dStepoffDist, sUVs ));
      }

    if ( !bFoundSolution )
    {
        SetStatus( SM_FV_SOLVER_NOT_CONVERGE );
        return SM_SUCCESS;
    }

    SmFace * pFace1 = (SmFace*)GetPointClassObject();
    ULONG lRailIndex = 0;
    if (pEU->GetFace() != pFace1) {
        lRailIndex = 1;
    }
    SmVector2d sUV1 = rTsectPnt.UVPos(lRailIndex);
    SmPoint3d sPnt1;
    SER(pFace1->GetSurface()->EvaluatePoint(sUV1,sPnt1));
    SetOriginalUV(sUV1);
    SetPoint(sPnt1);
    SetStatus(SM_FIL_PROCESSED);

    ULONG lOtherRailIndex = 0;
    SmFilletVertexuse * pVU2 = NULL;
    if (lTotalGeoms ==1 ) {
        pVU2 = GetVUAtRailEnd(pFilletGeom1,pVU1);
        lOtherRailIndex = lRailIndex;
    }
    else {
        SmFilletGeom * pFilletGeom2 = sGeoms[1];
        pVU2 = GetVUAtRailEnd(pFilletGeom2,pVU1);
        SmEdgeuse * pEU2 = pFilletGeom2->GetFilletSolver()->GetEdgeuse(0);
        if (pEU2->GetFace() != pFace1) {
            lOtherRailIndex = 1;
        }
    }
    if (pVU2) {
        SmTsectPnt & rTsectPnt2 = pVU2->GetTsectPnt();
        rTsectPnt2.UVPos(lOtherRailIndex) = rTsectPnt.UVPos(lRailIndex);
        rTsectPnt2.UVPos(1-lOtherRailIndex) = rTsectPnt.UVPos(1-lRailIndex);
    }
    // Calculate Mate if needed
    SmFilletVertex * pMate = GetMate(0);
    if (pMate == NULL) return SM_SUCCESS;
    SmFilletVertexuse * pVU = pMate->GetVUAtRailEnd(pFilletGeom1);
    if (!pVU) return SM_SUCCESS;
    SmFace * pFace2 = (SmFace*)pMate->GetPointClassObject();
    SmVector2d sUV2 = rTsectPnt.UVPos(1-lRailIndex);
    SmPoint3d sPnt2;
    SER(pFace2->GetSurface()->EvaluatePoint(sUV2,sPnt2));
    pMate->SetOriginalUV(sUV2);
    pMate->SetPoint(sPnt2);
    pMate->SetStatus(SM_FIL_PROCESSED);
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(1,0,0);
        smgfx_SetPointSize(6);
        Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        pMate->Draw();
        smgfx_SetColor(0,0,0);
        cpVertex->GetBrep()->Draw();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmFilletVertex::CalcVertOnCrossSection

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_SETBACK.

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::CalcSetBackVert()
{
  NER(m_cpCorner);
  const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex();
  double dSetBackDist = m_cpCorner->GetSetBackDist();

  SmTArray<SmFilletGeom*> sGeoms;
  GetFilletGeoms(sGeoms);
  ULONG lTotalGeoms = sGeoms.GetSize();
  if (lTotalGeoms != 1) SER(SM_ERR);
  SmFilletGeom * pFilletGeom = sGeoms[0];
  SmFilletSolver * pFilSolver = pFilletGeom->GetFilletSolver();
  NER(pFilSolver);
  SmEdgeuse * pEU = pFilSolver->GetEdgeuse(0);
  SmEdge * pE = pEU->GetEdge();
  SmCurve * pCurve = pE->GetCurve(); NER(pCurve);
  SmExtent1d sIvl = pCurve->GetNaturalInterval();
  double dEndParam = SM_BIG_DOUBLE;
  SmVector3d sPV[2];
  // Walk along the curve for dSetBackDist
  if (pE->GetVertex() == cpVertex) {
      double dStartParam = sIvl.GetMin();
      SER( pCurve->FindParameterAtArcLength(
              dStartParam, dSetBackDist, dEndParam ));
      SER( pCurve->Evaluate( dEndParam, 1, TRUE, sPV ));
  }
  else {
      const SmContext & crContext = m_cpCorner->GetContext();
      SmCurve * pNewCurve = NULL;
      SER(pCurve->Copy( crContext, pNewCurve ));
      SmObjDelete sDelete( pNewCurve );
      SmExtent1d sNewIvl;
      SER( pNewCurve->ReverseParameterization( sIvl, sNewIvl ));
      double dStartParam = sNewIvl.GetMin();
      SER( pNewCurve->FindParameterAtArcLength(
              dStartParam, dSetBackDist, dEndParam ));
      SER( pNewCurve->Evaluate( dEndParam, 1, TRUE, sPV ));
  }
  SmPoint3d sPlaneOrig = sPV[0];
  SmVector3d sPlaneNormal = sPV[1];

  SmSurface * pSurf1 = pFilSolver->GetSurface(0);
  SmSurface * pSurf2 = pFilSolver->GetSurface(1);
  SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
  SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();
  SmBoolean bFoundSolution;
  SmFilletVertexuse * pVU = GetVUAtRailEnd(pFilletGeom);
  SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();

  SmSolutionArray sSolutions;
  double dTol = pFilSolver->GetThisApproxTol3d();
  SER(pSurf1->GlobalPointSolve(sDomain1,SM_SO_MINIMIZE,
      sPlaneOrig,dTol,NULL,SM_SR_ALL,sSolutions));
  if (sSolutions.GetSize() != 1) SER(SM_ERR);
  SmPoint2d sUV1 = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
  SER(pSurf2->GlobalPointSolve(sDomain2,SM_SO_MINIMIZE,
      sPlaneOrig,dTol,NULL,SM_SR_ALL,sSolutions));
  if (sSolutions.GetSize() != 1) SER(SM_ERR);
  SmPoint2d sUV2 = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

  // find filletPoint on given plane that satisfies geometry requirements
  // implemented in derived SmFilletSolver class

  SER( pFilSolver->PointOnPlaneSolve(
          sPlaneOrig, sPlaneNormal, sDomain1, sDomain2, sUV1, sUV2,
          bFoundSolution, rTsectPnt ));

  if ( !bFoundSolution )
  {
      SetStatus( SM_FV_SOLVER_NOT_CONVERGE );
      return SM_SUCCESS;
  }
  SmFace * pFace1 = SM_CAST_PTR(SmFace,GetPointClassObject()); NER(pFace1);
  ULONG lRailIndex = 1;
  if (pEU->GetFace() == pFace1) lRailIndex = 0;
  sUV1 = rTsectPnt.UVPos(lRailIndex);
  SmPoint3d sPnt1;
  SER(pFace1->GetSurface()->EvaluatePoint(sUV1,sPnt1));
  SetOriginalUV(sUV1);
  SetPoint(sPnt1);
  SetStatus(SM_FIL_PROCESSED);
  // Calculate Mate if needed
  SmFilletVertex * pMate = GetMate(0);
  if (pMate == NULL) return SM_SUCCESS;
  SmFilletVertexuse * pVU2 = pMate->GetVUAtRailEnd(pFilletGeom);
  if (!pVU2) return SM_SUCCESS;
  SmTsectPnt & rTsectPnt2 = pVU2->GetTsectPnt();
  for (ULONG j=0; j<2; j++) {
      rTsectPnt2.UVPos(j) = rTsectPnt.UVPos(j);
  }
  SmFace * pFace2 = (SmFace*)pMate->GetPointClassObject();
  sUV2 = rTsectPnt.UVPos(1-lRailIndex);
  SmPoint3d sPnt2;
  SER(pFace2->GetSurface()->EvaluatePoint(sUV2,sPnt2));
  pMate->SetOriginalUV(sUV2);
  pMate->SetPoint(sPnt2);
  pMate->SetStatus(SM_FIL_PROCESSED);
#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) {
      smgfx_SetLook(1,1, 1,1,0);
      Draw();
      sm_GraphicsLoop();
      smgfx_SetLook(1,1, 0,0,1);
      pMate->Draw();
      smgfx_SetLook(1,1, 0,0,0);
      cpVertex->GetBrep()->Draw();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmFilletVertex::CalcSetBackVert

/*******************************************************************//**
PURPOSE: Set SmFilletVertex::m_vPoint  = XSectPoint(FilletSurface, Orig BrepCurve) and
             SmFilletVertex::m_eStatus = SM_FIL_PROCESSED

NOTES: Calculates vertex geometry of type SM_FV_SETBACK.
  returns SM_SUCCESS when FilletSurface XSects OrigBrepCurve and sets SmFilletVertex::m_vPoint and m_eStatus
  returns SM_ERR     when FilletSurface/OrigBrepCurve XSect can't be found - no changes
***********************************************************************/
SmStatus SmFilletVertex::CalcCliffRailIntEdge
 // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
 // rm : (SmBSplineSurface * pFilletSurface,     // in : FilletSurface to examine
 (SM_FILLETSURF_TYPE * pFilletSurface,     // in : FilletSurface to examine
  double               dThisApproxTol3d) // in : DistTol3d to use in intersection
{
  NER(pFilletSurface);

  // get origBrep object mapping to this FilletVertex - known to be an Edgeuse
  SmEdgeuse     * pEU      = (SmEdgeuse*)GetPointClassObject();

  // locals
  SmCurve       * p3DCurve = pEU->GetEdge()->GetCurve();
  SmExtent1d      sIvl     = p3DCurve->GetNaturalInterval();
  SmSolution      aData[4];
  SmSolutionArray sSolutions(4,aData);

  // get FilletSurface/OrigBrepCurve intersection marking this FilletVertex Position3d
  SER(pFilletSurface->GlobalCurveIntersect(pFilletSurface->GetNaturalUVDomain(),
                                           *p3DCurve,
                                           sIvl,
                                           dThisApproxTol3d,
                                           sSolutions));
  if (sSolutions.GetSize() < 1)
    { SER(SM_ERR); }

  // side effect: Set SmFilletVertex::m_vPoint = XSectPoint(FilletSurface, Orig BrepCurve)
  SmPoint3d sPnt;
  SER(p3DCurve->EvaluatePoint(sSolutions[0].m_vStart[0],sPnt));
  SetPoint(sPnt);
  SetStatus(SM_FIL_PROCESSED);

  // all done
  return SM_SUCCESS;

} // end SmFilletVertex::CalcCliffRailIntEdge

/*******************************************************************//**
PURPOSE: Calculate vertex geometry of type SM_FV_MATE.

NOTES:
   If unable to calculate our geom, return SM_SUCCESS, but remain unprocessed.
***********************************************************************/
SmStatus SmFilletVertex::CalcMateGeom()
{
  // our mate (which is the 'real' fillet vertex) must be done first.
  SmFilletVertex *pMainFV = this->GetMate( 0 );
  NER( pMainFV );
  if ( ! pMainFV->IsProcessed() )
    { pMainFV->CalcCornerVertGeom(); }
  if ( ! pMainFV->IsProcessed() )
    { return SM_SUCCESS; }

  // Calculating Main's geom can cause us to be calculated also.
  if ( this->IsProcessed() )
    { return SM_SUCCESS; }

  // Just work from the first FilletGeom.
  // If there's ever a problem with that -- could try a second FG.
  SmTArray<SmFilletGeom*> sGeoms;
  GetFilletGeoms( sGeoms );

  // check state - at least 1 railEdge must be connected to this vertex
  if ( sGeoms.GetSize() < 1 )
    { SER( SM_ERR ); }

  SmFilletGeom *pFilletGeom = sGeoms[0];

  // We have to figure out which rail of the FilletGeom we're on.
  // We can use the FG's Solver's FindIndexOfRailXSideEdgeuse()
  // method on the opposite (our mate) FilletVertex.

  SmEdgeuse * pOtherSideEU = (SmEdgeuse*)( pMainFV->GetPointClassObject() );
  if ( pOtherSideEU == NULL )
    { return SM_SUCCESS; }

  SmFilletSolver * pFilSolver = pFilletGeom->GetFilletSolver();
  NER( pFilSolver );
  ULONG lRailIndex = pFilSolver->FindIndexOfRailXSideEdgeuse( pOtherSideEU );
  lRailIndex = 1 - lRailIndex; // We're on the opposite rail.

  SmFilletVertexuse * pMainVtxUse = pMainFV->GetVUAtRailEnd( pFilletGeom );
  SmTsectPnt & rTsectPnt = pMainVtxUse->GetTsectPnt();

  SmFilletVertexuse * pOurVU = GetVUAtRailEnd( pFilletGeom );
  if ( !pOurVU )
    { return SM_SUCCESS; }

  SmFace *pFace = (SmFace*)GetPointClassObject();
  NER( pFace );
  SmPoint2d sUV = rTsectPnt.UVPos(lRailIndex);
  SmPoint3d sPnt;

  // compute sPnt = pFace->Surface(sUV), use ExtendedSurface as needed
  SER( sm_EvaluateMate( pFace, sUV, sPnt ));

#ifdef SM_DEBUG_CODE
  // draw Face(black), sPnt(green)
  if ( DebugLevel() > 0 )
  {
            smgfx_SetLook(1,1, 0,0,0); pFace->DrawUV(1,1); sm_GraphicsLoop();
            smgfx_SetLook(1,8, 0,1,0); sPnt.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE
  // set mate outputs
  SetOriginalUV( sUV );
  SetPoint( sPnt );
  SetStatus( SM_FIL_PROCESSED );

  return SM_SUCCESS;
} // end SmFilletVertex::CalcMateGeom

/*******************************************************************//**
PURPOSE: When a Face is split, if this FilletVertex is on the orignal Face,
   see whether it is now on the new split Face.

NOTES:
***********************************************************************/
SmStatus SmFilletVertex::UpdateSplitFace( SmFace *pOrigFace, SmFace *pNewFace )
{
  SmPointClassification &rPC = this->GetPointClassification();
  SmPoint3d sPos = this->GetPoint();
  int iWhichFace = -1;  // Unset. values: 0: neither Face; 1: 1st Face; 2: 2nd Face; 3: both.

  SmFace *pPCFace = rPC.GetFaceObject();
  if ( pPCFace == pOrigFace )
  {
      iWhichFace = sm_ClassifyFacePoint( sPos, pOrigFace, pNewFace );
      if ( iWhichFace == 2 )
      {
          rPC.SetClassObject( SM_PC_FACE, pNewFace );
      }
  }

  // It also has another Face pointer:
  const SmFace *cpOtherFace = rPC.GetFace();
  if ( cpOtherFace == pOrigFace )
  {
      if ( iWhichFace < 0 )
        { iWhichFace = sm_ClassifyFacePoint( sPos, pOrigFace, pNewFace ); }
      if ( iWhichFace == 2 )
      {
         rPC.SetFace( pNewFace );
      }
  }

  return SM_SUCCESS;

} // end SmFilletVertex::UpdateSplitFace

/*******************************************************************//**
PURPOSE: Get all SmFilletGeoms associated with FilletEdges
          of type SM_FE_RAIL to which this vertex is connected.

NOTES:
***********************************************************************/
void SmFilletVertex::GetFilletGeoms
  (SmTArray<SmFilletGeom*> & rGeoms)    // out: list of FilletGeoms for this FilletVertex
{
  rGeoms.ReSet();

  // get vertexuses (to find edges connected to this vertex)
  SmTArray<SmVertexuse*> sVertexuses;
  GetVertexuses(sVertexuses);

  // for every vertexuse
  for (ULONG i=0; i<sVertexuses.GetSize(); i++)
    {
      // get FilletVvertexuse->FilletEdgeuse->FilletEdge
      SmVertexuse     * pVU = sVertexuses[i];
      SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pVU->GetEdgeuse();
      SmFilletEdge    * pE  = (SmFilletEdge*)pEU->GetEdge();

      // skip edges not of type SM_FE_RAIL
      if (pE->GetFilletEdgeType() != SM_FE_RAIL) continue;

      // accumlate all the FilletEdgeuse->FilletGeoms
      SmFilletGeom * pFilletGeom = pEU->GetFilletGeom();
      if (pFilletGeom)
        {
         rGeoms.Add(pFilletGeom);
        } // end found a keeper pFilletGeom check
    } // end iter all vertexuses

  // all done
  return;

} // end SmFilletVertex::GetFilletGeoms

// /*******************************************************************//**
// PURPOSE: Get object of point classification.
// 
// NOTES:
// ***********************************************************************/
// SmObject *SmFilletVertex::GetPointClassObject
//   ()
// {
//     return m_vPointClass.GetObject();
// 
// } // end SmFilletVertex::GetPointClassObject

/*******************************************************************//**
PURPOSE: Get SmFilletVertex->SmFilletVertexuse which connects to a
            SmFilletEdge rail (type == SM_FE_RAIL or SM_FE_CLIFF_RAIL)
            which is part of the input pOptFilletGeom (when given) and
                  is not equal to the pOptExculdeVU (when given).

NOTES:

METHOD ---
  search every SmFilletVertexuse attached to this SmFilletVertex for one that
  is attached to an SmFilletEdge of type SM_FE_RAIL or SM_FE_CLIFF_RAIL
  and return that SmFilletVertexuse.

  Skip any vertexuse whose pointer value equals the input pOptExculdeVU
  value or whose SmFilletGeom pointer value does not equal the input
  pOptFilletGeom value.
***********************************************************************/
SmFilletVertexuse * SmFilletVertex::GetVUAtRailEnd
  (SmFilletGeom      * pOptFilletGeom, // in : notNULL=only return vertexuse connected to rails in this filletGeom
                                       //         NULL=return 1st vertexuse connected to any rail
                                       //      default:[NULL]
   SmFilletVertexuse * pOptExcludeVU,  // in : specify a Vertexuse to exclude from search,
                                       //      NULL to ignore, default:[NULL]
   SmFilletEdge      * pOptFilletRail) // in : notNULL=only return vertexuse connected to this rail
                                       //      NULL to ignore, default:[NULL]
{
    // get all of this vertex's vertexuses
    SmTArray<SmVertexuse*> sVertexuses;
    GetVertexuses(sVertexuses);

    // for every vertexuse
    for (ULONG i=0; i<sVertexuses.GetSize(); i++)
      {
        // this vertexuse locals
        SmFilletVertexuse * pFilletVU = (SmFilletVertexuse*)sVertexuses[i];
        SmFilletEdgeuse   * pFilletEU = (SmFilletEdgeuse*)pFilletVU->GetEdgeuse();
        SmFilletEdge      * pFilletE  = (SmFilletEdge*)pFilletEU->GetEdge();

        // when edge is a RAIL or CLIFF_RAIL type
        if (   pFilletE->GetFilletEdgeType() == SM_FE_RAIL
            || pFilletE->GetFilletEdgeType() == SM_FE_CLIFF_RAIL)
          {
            // skip excluded filletVertices
            if (pOptExcludeVU && pFilletVU == pOptExcludeVU) continue;

            // skip excluded filletGeoms
            SmFilletGeom * pFG = pFilletEU->GetFilletGeom();
            if (pOptFilletGeom && pFG != pOptFilletGeom) continue;

            // skip excluded fillet Rails
            if (pOptFilletRail && pFilletE != pOptFilletRail) continue;

            return pFilletVU;
          }
      } // end iter every vertexuse

    return NULL;// Could not find it!

} // end SmFilletVertex::GetVUAtRailEnd

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertFilletVertex_list[] =
{
  {SM_AT_POINTER, _T("Context"), _T("m_pMate[0] shares the same context") },
  {SM_AT_POINTER, _T("Context"), _T("m_pMate[1] shares the same context") },
  {SM_AT_POINTER, _T("Context"), _T("m_cpCorner shares the same context") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmFilletVertex::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmVertex::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // SmFilletVertex and the objects it attaches to need to share common contexts
  if(m_pMate[0]) {
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pMate[0]->GetContext()), _T("") ) ;
  }
  if(m_pMate[1]) {
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (GetContext() == m_pMate[1]->GetContext()), _T("") ) ;
  }
  if(m_cpCorner) {
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (GetContext() == &m_cpCorner->GetContext()), _T("") ) ;
  }

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmFilletVertex::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmFilletVertex::AssertHeal
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
//       return ( SmVertex::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmFilletVertex::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmFilletVertex::AssertHeal
// end obsolete

/*************************************************************
PURPOSE: Dump.

NOTES:
**************************************************************/
void SmFilletVertex::DumpLevel
 (int           iDebugLevel, // NotUsed: in :
  const TCHAR * pcMsg )      // in :
 const
{
  SM_REF1(iDebugLevel) ; 
  if ( pcMsg != NULL ) { smos_WriteBuffer( pcMsg ); }

  TCHAR sBuff[SM_TBLOCK_SIZE];
  
  ULONG lIdxThis  = m_cpCorner->FindVertexIndex( this );
  ULONG lIdxMate1 = ( m_pMate[0] ) ? m_cpCorner->FindVertexIndex( m_pMate[0] ) : 9999;
  ULONG lIdxMate2 = ( m_pMate[1] ) ? m_cpCorner->FindVertexIndex( m_pMate[1] ) : 9999;
  ULONG lIdxCorn  = m_cpCorner->FindIndexInExec();
  smos_sprintf( sBuff,
      _T("\nDump of SmFilletVertex [%4ld]  Type %d,  Mates [%4ld], [%4ld], FilletCorner [%4ld], Status %d\n"),
      lIdxThis, m_eType, lIdxMate1, lIdxMate2, lIdxCorn, m_eStatus );
  smos_WriteBuffer(sBuff);
  m_vPointClass.Dump();

  SmVertex::Dump( FALSE ); // Unabbreviated: just more decimal places.

} // end SmFilletVertex::DumpLevel

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletVertex::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletVertex_TYPE == t) ? TRUE : SmVertex::IsKindOf( (t) ));
}


/*************************************************************
PURPOSE: Unimplemented Dump method to satisfy SM_COMMON.
**************************************************************/
void SmFilletVertex::Dump () const
{}

/*******************************************************************//**
PURPOSE: Return the index of this FilletVertex in our FilletCorner.

NOTES: Used in debugging.
    Returns error 9999 if 'this' is not in our FilletCorner.
***********************************************************************/
ULONG SmFilletVertex::FindIndexInCorner() const
{
  if ( m_cpCorner == NULL ) { return 9999; }
  return m_cpCorner->FindVertexIndex( this ) ;

} // end SmFilletVertex::FindIndexInCorner

/*******************************************************************//**
PURPOSE: Constructor for the Fillet Edgeuse.  It adds it into the
   Pseudo Brep.

NOTES:
***********************************************************************/
SmFilletEdgeuse::SmFilletEdgeuse
  (SmFilletBrep *pPseudoBrep)
 : SmEdgeuse(),
   m_pFilletGeom(NULL)
{
    SmRegion *pReg = pPseudoBrep->GetInfiniteRegion();
    SmTArray<SmShell*> sShells;
    pReg->GetShells(sShells);
    if (sShells.GetSize() == 0) {
        SmVertex *pNewV; // Just an artifact vertex
        SmShell *pShell;
        pPseudoBrep->MakeShellVertexTopology(pReg,pShell,pNewV);
        pReg->GetShells(sShells);
    }
    m_pSorLU = sShells[0];
    m_tEdgeuseType = SmShell_TYPE;

} // end SmFilletEdgeuse::SmFilletEdgeuse

/*******************************************************************//**
PURPOSE: Set the SmFilletGeom backpointer in an SmFilletEdgeuse

NOTES:
***********************************************************************/
void SmFilletEdgeuse::SetFilletGeom
  (SmFilletGeom * pFG)
{
  if (pFG)
    {
      pFG->AddReferencedFilletEdgeuse(this);
    }
  m_pFilletGeom = pFG;

} // end SmFilletEdgeuse::SetFilletGeom

/*******************************************************************//**
PURPOSE: Set the SmFilletGeom backpointer in an SmFilletEdgeuse

NOTES:
***********************************************************************/
void SmFilletEdgeuse::SetUVCurve
 (SmBSplineCurve * pUVTrimCurve,     // in : New Value to store in m_pUVTrimCurve
  SmBoolean        bDeleteOldCurve,  // in : TRUE = delete current m_pCurve if not NULL
                                     //      FALSE= don't delete current m_pCurve
                                     //      default:[FALSE] = previous behavior
  SmBoolean        bDbgWarnLeaks)    // in : TRUE = in Debug Mode - Warn when NonNULL m_pCurve is overwritten
                                     //      FALSE= don't, caller knows that might happen and it's not a problem
                                     //      default:[TRUE]
{
#ifdef SM_DEBUG_CODE
  if(pUVTrimCurve != NULL && pUVTrimCurve->GetDim() != 2)
    {
      SM_ASSERT_MSG(pUVTrimCurve == NULL || pUVTrimCurve->GetDim() == 2, _T("SmFilletEdgeuse::SetUVCurve passed a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  if(m_pUVTrimCurve != pUVTrimCurve)
    {
      if(m_pUVTrimCurve)
        { if(bDeleteOldCurve)    { delete m_pUVTrimCurve ; m_pUVTrimCurve = NULL ; }
          else if(bDbgWarnLeaks) {
                                   SM_DBG_WARN(_T("SmFilletEdgeuse::SetUVCurve - possible memory leak, replaced NonNUll m_pCurve value without deleting old m_pCurve")) ;
                                 }
        }
    }

  m_pUVTrimCurve = pUVTrimCurve;
  if(pUVTrimCurve) { pUVTrimCurve->SetOwner(this) ; }

} // end SmFilletEdgeuse::SetUVCurve

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertFilletEdgeuse_list[] =
{
  {SM_AT_UNKNOWN, _T("UNKNOWN"), _T("Not yet implemented") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmFilletEdgeuse::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmEdgeuse::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // todo - add SmFilletEdgeuse checks here

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmFilletEdgeuse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmFilletEdgeuse::AssertHeal
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
//       return ( SmEdgeuse::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmFilletEdgeuse::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmFilletEdgeuse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletEdgeuse::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletEdgeuse_TYPE == t) ? TRUE : SmEdgeuse::IsKindOf( (t) ));
}

/*************************************************************
PURPOSE: Unimplemented Dump method to satisfy SM_COMMON.

NOTES: May be implemented if desired.
**************************************************************/
void SmFilletEdgeuse::Dump( void ) const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // label
  smos_WriteBuffer( _T( "\nBegin SmFilletEdgeuse::Dump() derived from class SmEdgeuse" ) );

  // header
  smos_sprintf( sBuff, _T( "\nSmFilletEdgeuse object[0x%p] : " ), this );
  smos_sprintf( sBuffForFile, _T( "\nSmFilletEdgeuse object[%s] : " ), _T( "NotNULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // FilletGeom
  smos_sprintf( sBuff, _T( "m_pFilletGeom:[0x%p] " ), m_pFilletGeom );
  smos_sprintf( sBuffForFile, _T( "m_pFilletGeom:[%s] " ), m_pFilletGeom ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // Dump the contained FilletGeom
  if(m_pFilletGeom)
  {
    m_pFilletGeom->Dump();
  }

  // Dump the Root class
  SmEdgeuse::Dump();

  smos_WriteBuffer( _T( "\nEnd SmFilletEdgeuse::Dump() derived from class SmEdgeuse \n" ) );

} // end SmFilletEdgeuse::Dump

/*******************************************************************//**
    Static functions
***********************************************************************/

/*******************************************************************//**
PURPOSE: Create a cubic curve which interpolates two rails.

NOTES: Bulge Factor: A value between [0,1] which
  determine the smoothness of the curve. In general, bigger
  bulge factor resulted in 'sharper' corner
  curve while smaller bulge-factor results in more smoother curver
  (i.e. the risk of intersecting corner of original brep is increased)

***********************************************************************/
static SmStatus sm_CreateCubicRailRailInterpolant
  (const SmContext & crContext,
   SmFilletSolver * pFilSolver1,
   SmFilletSolver * pFilSolver2,
   SmFace * pFace,
   const SmVertex * cpCornerVert,
   SmPoint3d & rStartPnt,
   SmVector3d & rStartTangent,
   SmPoint3d & rEndPnt,
   SmVector3d & rEndTangent,
   double dBulgeFactor,
   SmBSplineCurve *& rp3DCurve,
   int iDebugLevel)
{
    rp3DCurve = NULL;
    // First, calculate the intersection of two rails
    ULONG lRailIndex = pFilSolver1->FindIndexOfRailOnFace(pFace);
    ULONG lOtherRailIndex = pFilSolver2->FindIndexOfRailOnFace(pFace);
    // Derive guess uv point here
    SmPoint2d sUVGuess;
    SmEdgeuse * pEU = pFilSolver1->GetEdgeuse(lRailIndex);
    if (pEU->GetVertexuse()->GetVertex() != cpCornerVert)
        pEU = pFilSolver2->GetEdgeuse(lOtherRailIndex);
    pEU->GetVertexuse()->ComputeUVPoint(sUVGuess);

#ifdef SM_DEBUG_CODE
    SmSurface * pSurf = pFace->GetSurface();

    if ( iDebugLevel > 0 ) {
        smgfx_SetColor(0,0,0);
        pFace->DrawUV(5,5);
        smgfx_SetColor(1,0,0);
        smgfx_SetLineWidth(4);
        pFilSolver1->GetEdgeuse(lRailIndex)->Draw();
        pFilSolver2->GetEdgeuse(lOtherRailIndex)->Draw();
        smgfx_SetLineWidth(2);
        smgfx_SetColor(0,0,1);
        smgfx_SetPointSize(8);
        cpCornerVert->Draw();
        smgfx_SetPointSize(4);
        sm_GraphicsLoop();
    }
#else
    SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE
    SmBoolean bFoundIntersection;
    SmTsectPnt sTsectPnt1, sTsectPnt2;
    SER(pFilSolver1->RailRailIntersect(sUVGuess,lRailIndex,pFilSolver2,
        lOtherRailIndex,bFoundIntersection,sTsectPnt1,sTsectPnt2));
    if (!bFoundIntersection) SER(SM_ERR);
    SmPoint2d sUVPnt = sTsectPnt1.UVPos(lRailIndex);
#ifdef SM_DEBUG_CODE
    if ( iDebugLevel > 0 ) {
        smgfx_SetColor(0,1,0);
        smgfx_SetPointSize(8);
        SmPoint3d sPnt;
        SER(pSurf->EvaluatePoint(sUVPnt,sPnt));
        sPnt.Draw();
        sm_GraphicsLoop();
        smgfx_SetPointSize(4);
        smgfx_SetColor(0,0,1);
        SmPoint2d sUV1(rStartPnt.x,rStartPnt.y);
        SER(pSurf->EvaluatePoint(sUV1,sPnt));
        sPnt.Draw();
        sm_GraphicsLoop();
        SmPoint2d sUV2(rEndPnt.x,rEndPnt.y);
        SER(pSurf->EvaluatePoint(sUV2,sPnt));
        sPnt.Draw();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SmVector3d sVec = sUVPnt - rStartPnt;
    double dLen = dBulgeFactor*sVec.Length();
    double dAngle;
    SER(rStartTangent.AngleBetween(sVec,dAngle));
    double dLen1 = smos_Fabs(dLen/cos(dAngle));
    SmPoint3d sP1 = rStartPnt + rStartTangent*dLen1;
    sVec = sUVPnt - rEndPnt;
    dLen = dBulgeFactor*sVec.Length();

    SER(rEndTangent.AngleBetween(sVec,dAngle));
    dLen1 = smos_Fabs(dLen/cos(dAngle));
    SmPoint3d sP2 = rEndPnt + rEndTangent*dLen1;

    SmPoint3d sData[5];
    SmTArray<SmPoint3d> sCntrlPoly(5,sData);
    sCntrlPoly.Add(rStartPnt);
    sCntrlPoly.Add(sP1);
    sCntrlPoly.Add(sUVPnt);
    sCntrlPoly.Add(sP2);
    sCntrlPoly.Add(rEndPnt);
    double adKData[3];
    SmTArray<double> sKnots(3,adKData,3);
    sKnots[0] = 0.0; sKnots[1] = 0.5; sKnots[2] = 1.0;
    ULONG alKMData[3];
    SmTArray<ULONG> sKnotMult(3,alKMData,3);
    sKnotMult[0] = 4;
    sKnotMult[1] = 1;
    sKnotMult[2] = 4;
    SER(SmBSplineCurve::CreateCanonical(crContext,2,2,
        sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
        SM_KT_UNSPECIFIED, NULL, NULL, rp3DCurve));

    return SM_SUCCESS;

} // end sm_CreateCubicRailRailInterpolant

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static void sm_CleanupNoise
  (SmPoint2d & rUV)
{
    rUV.x = smos_CleanUpNoise(rUV.x);
    rUV.y = smos_CleanUpNoise(rUV.y);

} // end sm_CleanupNoise

/*******************************************************************//**
PURPOSE: Create a uv-curve on a surface domain given start
     and end points.

NOTES:
     when bCreateTwoBoundaryCurves == TRUE, endPts are known to lie on a seam
       check surface and return SM_ERR if it's not closed in u
       and rpUVCurve1 and rpUVCurve2 are built and returned.
     when both end points lie on an isoParameter line - return IsoParameter curve in rpUVCurve1
       when endPoints are also found to lie on a seam - return IsoParameter curve in rpUVCurve2
     else
       return UVCurve found by intersecting a plane/surface intersection
         where the plane contains both startPoint and EndPoint and its surface normal
         direction is selected
           for pSurface->IsCone_TYPE  : in the plane containing the cone Zaxis and the Start/EndPt 3d line
           for all other surface types: in the plane containing a filletVertexuse->GetTsectPnt().CrvDeriv() vector
                                            and the Start/EndPt 3d line
     undefined behavior when
       ZAxis is parallel to sDir
       Start/EndPt are coincident
***********************************************************************/
static SmStatus sm_CreateUVCurve
  (const SmContext & crContext,                // in : context for new objects
   SmFilletVertex  * pStartFV,                 // in : Vertex marking UVTrimCurve start
   SmFilletVertex  * pEndFV,                   // in : Vertex marking UVTrimCurve end
   SmBoolean         bCreateTwoBoundaryCurves, // in : TRUE = curve is known to lie on seam of a closed surface.
   SmSurface       * pSurface,                 // in : Surface to map the UVTrimCurve
   double            dApproxTol,               // in : min dist between distinct points,
                                               //      accuracy of UVCurve approximations when needed
   SmBSplineCurve *& rp3DCurve,                // out: 3D curve only created when rpUVCurve1 is not a iso-curve
   SmBSplineCurve *& rpUVCurve1,               // out: Resulting uv-curve
   SmBSplineCurve *& rpUVCurve2)               // out: 2nd uv-curve, if it's a seam curve
{
  // init output
  rp3DCurve           = NULL ;
  rpUVCurve1          = NULL ;
  rpUVCurve2          = NULL ;

  // locals
  ULONG      ii ;
  int        iDebugLevel = 0;
  SmPoint3d  sStartPnt   = pStartFV->GetPoint();
  SmPoint3d  sEndPnt     = pEndFV->GetPoint();
  SmExtent2d sUVDomain   = pSurface->GetNaturalUVDomain();

#ifdef SM_DEBUG_CODE
  iDebugLevel = pStartFV->DebugLevel();
  if ( iDebugLevel > 0 )
    {
      // if(0)
      //   { smgfx_Erase() ; }
      smgfx_SetLook( 3,4, 1,0,0 ); pStartFV->Draw();      sm_GraphicsLoop();
      smgfx_SetLook( 3,4, 1,0,1 ); pEndFV->Draw();        sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,0,0 ); pSurface->DrawUV(0,0); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
// draw
if(bDebugMe)
  {
    SM_ASSERT_VALID(pSurface) ;

    SmFace *pFace = (SmFace *)pSurface->GetFace() ;
    SmBrep *pBrep =   pFace  ? pFace->GetBrep()
                    : pStartFV ? pStartFV->GetBrep()
                    : pEndFV ? pEndFV->GetBrep() : NULL ;

    smgfx_Erase() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
    smgfx_SetLook(3,4, 1,0,0 ); if(pStartFV) pStartFV->Draw(); sm_GraphicsLoop();
    smgfx_SetLook(5,6, 1,0,1 ); if(pEndFV) pEndFV->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // when asked - create 2 UVTrimCurves running along surface seam
  if ( bCreateTwoBoundaryCurves )
    {
      // Closed surface, create two boundary isocurves at uMin and uMax.

      // First double-check that the surface is indeed closed.
      //cbi  We'll allow for one corner being cut off (as could happen if
      //cbi  the seam is over a cliff edge), but currently not both.
      SmPoint2d uv00 = sUVDomain.GetMin();
      SmPoint2d uv11 = sUVDomain.GetMax();
      SmPoint2d uv01( uv00.x, uv11.y );
      SmPoint2d uv10( uv11.x, uv00.y );
      SmPoint3d sSrfPt00, sSrfPt01, sSrfPt10, sSrfPt11;

      // evaluate Vmin corners
      SER( pSurface->EvaluatePoint( uv00, sSrfPt00 ));
      SER( pSurface->EvaluatePoint( uv10, sSrfPt10 ));
      if ( sSrfPt00.DistanceBetween( sSrfPt10 ) > dApproxTol ) // start points on Umin and Umax isoparam curves are equal
          SER( SM_ERR );

      // evaluate Vmax corners
      SER( pSurface->EvaluatePoint( uv01, sSrfPt01 ));
      SER( pSurface->EvaluatePoint( uv11, sSrfPt11 ));
      if ( sSrfPt01.DistanceBetween( sSrfPt11 ) > dApproxTol ) // end points on Umin and Umax isoparam curves are equal
          SER( SM_ERR );

      // Now check which v-direction is indicated by Start & End verts
      int iWhichCornerCut = 0; // -1 for vMin, +1 for vMax
      int iDirection =  (sStartPnt.DistanceBetween( sSrfPt00 ) < dApproxTol) ? +1
                       :(sEndPnt.  DistanceBetween( sSrfPt00 ) < dApproxTol) ? -1
                       : 0 ;

      if ( iDirection == 0 )
        {
          // Perhaps the SMin corner has been cut off.  Check SMax corner.
          iDirection =  (sEndPnt  .DistanceBetween( sSrfPt01 ) < dApproxTol) ? +1
                       :(sStartPnt.DistanceBetween( sSrfPt01 ) < dApproxTol) ? -1
                       : 0 ;

          // Not currently handling it if both corners are cut off.
          // Of course, this could also indicate a serious error [such as
          // reg tests 2:201,214,215] where the fillet surface doesn't
          // close back to the beginning, although that should have been
          // caught above.
          if ( iDirection == 0 )
            { SER( SM_ERR ) ; }

          iWhichCornerCut = -1;
        }
      else
        {
          // We did succeed in getting a direction from sSrfPt00, so the
          // low-v corner is not cut off.  We need to determine whether
          // the high-v corner is ok.

          // (iDirection > 0): when start pt coincides with sSrfPt00; check end   pt vs. sSrfPt01
          // (iDirection < 0): when end   pt coincides with sSrfPt00; check start pt vs. sSrfPt01
          iWhichCornerCut =   ((iDirection > 0) && (sEndPnt.  DistanceBetween( sSrfPt01 ) > dApproxTol)) ? +1
                            : ((iDirection < 0) && (sStartPnt.DistanceBetween( sSrfPt01 ) > dApproxTol)) ? +1
                            : iWhichCornerCut ;
        }

      // Get the start and end points for the two uv curves.
      // First check whether a corner was cut, and update uv[ij]
      // by finding the parameter value along the rib corresponding
      // to the cutoff point (which is either sStartPnt or sEndPnt).

      // Get the point that's not at a vertex, and a guess parameter
      // for it on the rib curve.

      // Guess parameter: we could interpolate the cutoff point
      // between the rib start and end points, but we know that the rib
      // is circular and less than 180 degrees, so the midpoint will
      // converge with no problem.  We also know that the parameterization
      // of the fillet surface is [0-1] in v.
      double dGuessParam = 0.5;

      // when a corner has been cut - find the param value that trims the isoCurve to the cut corner
      if ( iWhichCornerCut != 0 )
        {
          SmPoint3d sPtToDrop;

          // (Slightly tricky logic in this one line, sorry.)
          //  when (iWhichCornerCut * iDirection > 0) then sPtToDrop = sEndPt
          //       (iWhichCornerCut * iDirection < 0) then sPtToDrop = sStartPt
          SM_ASSERT(iWhichCornerCut * iDirection == 1 || iWhichCornerCut * iDirection == -1) ;
          sPtToDrop =  (iWhichCornerCut * iDirection > 0) ? sEndPnt : sStartPnt ;

          // Extract the full-length constant start-u isoParameter rib curve.
          SmBSplineCurve *pRibIsoCurve = NULL;
          SER( pSurface->CreateIsoParametricCurve
                  ( crContext,       // in : context for created objects
                    SM_SP_U,         // in : Defines which parameter direction on surface to extract curve from
                                     //      SM_SP_U = create constant u isoParameter curve
                                     //      SM_SP_V = create constant v isoParameter curve
                    uv00.x,          // in : Defines parametric value at which to extract the curve.
                                     //      If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V
                                     //      then this is the V parameter
                    dApproxTol,      // in : passed to ApproximateCurve() when approximation is required.
                                     //      If set to 0.0, tolerance is set to curve length * 1.0e-4
                    pRibIsoCurve)) ; // out: 3d IsoParameterCurve
          NER( pRibIsoCurve );

          // Drop sPtToDrop to full curve to find trim param value for cut corner
          SmBoolean bSuccess = FALSE;
          double dDropParam, dDistToCurve;
          SER( pRibIsoCurve->DropPoint( pRibIsoCurve->GetNaturalInterval(), // in : target curve allowed domain
                                        sPtToDrop,                          // in : Point to drop to curve
                                        NULL,                               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                        dApproxTol,                         // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                       &dGuessParam,                        // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                        bSuccess,                           // out: TRUE = found a drop point
                                        dDropParam,                         // out: found drop curve param
                                        dDistToCurve)) ;                    // out: found drop distance
                                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
          if(!bSuccess)
            { SER( SM_ERR ) ; }

          // trim the UVTrimCurve on the seam side of the surface which has been cut
          if ( iWhichCornerCut < 0 ) { uv00.Set( uv00.x, dDropParam );
                                       uv10.Set( uv10.x, dDropParam );
                                     }
          else                       { uv01.Set( uv01.x, dDropParam );
                                       uv11.Set( uv11.x, dDropParam );
                                     }
        } // end a corner was cut check

      // arrive here after the 2 UVTrimCurve extents are set
      //   they are either full or trimmed to the cut point when
      //   one corner or the other of a surface has been cut

      // set up the desired IsoParameter intervals by setting the UVStart and UVEnd points
      SmPoint2d sStartUV0, sEndUV0;
      SmPoint2d sStartUV1, sEndUV1;
      if(iDirection > 0) { // run constant u IsoCurve from Surface v Min to Max
                           sStartUV0 = uv00;
                           sEndUV0   = uv01;
                           sStartUV1 = uv10;
                           sEndUV1   = uv11;
                         }
      else               { // run constant u IsoCurve from Surface v Max to Min
                           sStartUV0 = uv01;
                           sEndUV0   = uv00;
                           sStartUV1 = uv11;
                           sEndUV1   = uv10;
                         }

      // make Surface constant uMin isoParam curve
      SER_MSG( SmBSplineCurve::CreateLineSegment( crContext, 2, SmPoint3d( sStartUV0 ), SmPoint3d( sEndUV0 ), rpUVCurve1 ),
               _T("sm_CreateUVCurve(): Suffered an SmBSplineCurve::CreateLineSegment() failure"));
      NER(rpUVCurve1);

      // make Surface constant vMin isoParam curve
      SER_MSG( SmBSplineCurve::CreateLineSegment( crContext, 2, SmPoint3d( sStartUV1 ), SmPoint3d( sEndUV1 ), rpUVCurve2 ),
               _T("sm_CreateUVCurve(): Suffered an SmBSplineCurve::CreateLineSegment() failure"));
      NER(rpUVCurve2);

      // all done
      return SM_SUCCESS;

    }  // End of create-two-boundary-curves.

  // arrive here when desired UVTrimCurve is not known to sit on a Seam

  // locals
  SmSolutionArray sSolutions1;
  SmSolutionArray sSolutions2;
  ULONG lNumSolutions1 = 0;
  ULONG lNumSolutions2 = 0;

  // Compute dropping tolerance - start with a tight tolerance and relax that as needed
  SmExtent3d sBBox;
  SER(pSurface->CalculateBoundingBox(sUVDomain,&sBBox)) ;
  double dDroppingTol = 1.0e-9 * sBBox.GetSize().GetMaxDimension() ;

  // Param tolerance
  double dParamTol = (sUVDomain.GetSize().GetMaxDimension() + 1.0) * SM_EFF_ZERO;

  // try increasing tolerance sized drops of Start/End Pts to surface
  for (ULONG i=0; i<7; i++)
    {
      // Drop the start point
      if (lNumSolutions1 == 0) { SER(pSurface->GlobalPointSolve(sUVDomain,
                                                                SM_SO_INTERSECT,
                                                                sStartPnt,
                                                                dDroppingTol,
                                                                NULL,
                                                                SM_SR_ALL,
                                                                sSolutions1));
                               }

      // Drop the end point
      if (lNumSolutions2 == 0) { SER(pSurface->GlobalPointSolve(sUVDomain,
                                                                SM_SO_INTERSECT,
                                                                sEndPnt,
                                                                dDroppingTol,
                                                                NULL,
                                                                SM_SR_ALL,
                                                                sSolutions2));
                               }

      // increase the drop tolerance
      dDroppingTol   = 10.0*dDroppingTol;

      // quit iterating once both points drop to surface
      lNumSolutions1 = sSolutions1.GetSize();
      lNumSolutions2 = sSolutions2.GetSize();
      if (lNumSolutions1 > 0 && lNumSolutions2 > 0)
        {
          break;
        }
    } // end drop Start/End pt iter with larger drop tolerances

  // when DropPoint Intersection failed - try DropPoint Minimize
  if (lNumSolutions1 == 0) { // Try again with minimization
                             SER(pSurface->GlobalPointSolve(sUVDomain,
                                                            SM_SO_MINIMIZE,
                                                            sStartPnt,
                                                            dApproxTol,
                                                            NULL,
                                                            SM_SR_SINGLE,
                                                            sSolutions1));
                                 lNumSolutions1 = sSolutions1.GetSize();
                           }
  if (lNumSolutions2 == 0) { // Try again with minimization
                             SER(pSurface->GlobalPointSolve(sUVDomain,
                                                            SM_SO_MINIMIZE,
                                                            sEndPnt,
                                                            dApproxTol,
                                                            NULL,
                                                            SM_SR_SINGLE,
                                                            sSolutions2));
                                 lNumSolutions2 = sSolutions2.GetSize();
                           }

  // failure case: start or end pt failed to drop to surface
  if (lNumSolutions1 == 0 || lNumSolutions2 == 0)
    { SER(SM_ERR) ; }

  // when start point dropped to surface multiple times
  if (lNumSolutions1 > 1)
    {
      // sStartPnt could be a little bit off the surface.
      // Discard solutions within dParamTol of one another
      SmPoint2d sUV1(sSolutions1[0].m_vStart[0], sSolutions1[0].m_vStart[1]);
      for(ii=lNumSolutions1-1; ii>0; ii--)
        {
          SmPoint2d sUV2(sSolutions1[ii].m_vStart[0],sSolutions1[ii].m_vStart[1]);
          double dParamDist = sUV1.DistanceBetween(sUV2);
          if (dParamDist < dParamTol)
            {
              sSolutions1.RemoveAt(ii);
            }
        }
      lNumSolutions1 = sSolutions1.GetSize();

    } // end drop startPt multiple solutions check

  // when end point dropped to surface multiple times
  if (lNumSolutions2 > 1)
    {
      // sEndPnt could be a little bit off the surface.
      // Discard solutions within dParamTol of one another
      SmPoint2d sUV1(sSolutions2[0].m_vStart[0],sSolutions2[0].m_vStart[1]);
      for(ii=lNumSolutions2-1; ii>0; ii--)
        {
          SmPoint2d sUV2(sSolutions2[ii].m_vStart[0],sSolutions2[ii].m_vStart[1]);
          double dParamDist = sUV1.DistanceBetween(sUV2);
          if (dParamDist < dParamTol)
            {
              sSolutions2.RemoveAt(ii);
            }
        }
      lNumSolutions2 = sSolutions2.GetSize();

    } // end drop endPt multiple solutions check

  // Sort out dual(or multi-) solution cases
  SmPoint2d sStartUV(sSolutions1[0].m_vStart[0], sSolutions1[0].m_vStart[1]);
  SmPoint2d sEndUV  (sSolutions2[0].m_vStart[0], sSolutions2[0].m_vStart[1]);
  SmPoint2d sStartUV1;
  SmPoint2d sEndUV1;

  // when start point still dropped to surface multiple times
  if (lNumSolutions1 > 1)
    {
      sStartUV1 = SmPoint2d(sSolutions1[1].m_vStart[0], sSolutions1[1].m_vStart[1]);
      if (lNumSolutions1 == 2 && lNumSolutions2 == 1)
        {
          // Start point is on seam but end point is not (i.e. not a 'Seam-cross-section'
          // case where both lNumSolutions1 & lNumSolutions2 should be two.
          // Find the solution which is closest to sEndUV
          double dDistToEnd  = sStartUV.DistanceBetween(sEndUV);
          double dDistToEnd1 = sStartUV1.DistanceBetween(sEndUV);
          if (dDistToEnd1 < dDistToEnd)
            {
              sStartUV = sStartUV1;
            }
          lNumSolutions1 = 1;
        }
    } // end start point still dropped to surface multiple times check

  // when end point still dropped to surface multiple times
  if (lNumSolutions2 > 1)
    {
      sEndUV1 = SmPoint2d(sSolutions2[1].m_vStart[0], sSolutions2[1].m_vStart[1]);
      if (lNumSolutions2 == 2 && lNumSolutions1 == 1)
        {
          // Could be seam or degenerate cases, but not "Seam-cross-section" case
          // where both lNumSolutions1 & lNumSolutions2 should be two.
          // Find the solution which is closest to sStartUV
          double dDistToStart = sEndUV.DistanceBetween(sStartUV);
          double dDistToStart1 = sEndUV1.DistanceBetween(sStartUV);
          if (dDistToStart1 < dDistToStart)
            {
              sEndUV = sEndUV1;
            }
          lNumSolutions2 = 1;
        }
    } // end EndPoint still dropped to surface multiple times check

  // remember when Start and End Pts both dropped to a seam
  SmBoolean bMakeTwoCurves = (lNumSolutions1 >= 2 && lNumSolutions2 >= 2) ? TRUE : FALSE ;

  // when start point still dropped to surface multiple times
  if (lNumSolutions1 > 2)
    {
      if (lNumSolutions2 > 2) SER(SM_ERR); // Should not happen!!

      // Start point lies on degenerate edge
      if (smos_Fabs(sStartUV.x - sStartUV1.x) < dParamTol)
        {
          sStartUV.y = sEndUV.y;
          if (lNumSolutions2 == 2)
            {
              sStartUV1.y = sEndUV1.y;
            }
        }
      else if (smos_Fabs(sStartUV.y - sStartUV1.y) < dParamTol)
        {
          sStartUV.x = sEndUV.x;
          if (lNumSolutions2 == 2)
            {
              sStartUV1.x = sEndUV1.x;
            }
        }
      else SER(SM_ERR);
    } // end StartPoint still dropped to surface multiple times check

  // when end point still dropped to surface multiple times
  if (lNumSolutions2 > 2)
    {
      // End point lies on degenerate edge
      if (smos_Fabs(sEndUV.x - sEndUV1.x) < dParamTol)
        {
          sEndUV.y = sStartUV.y;
          if (lNumSolutions1 == 2)
            {
              sEndUV1.y = sStartUV1.y;
            }
        }
      else if (smos_Fabs(sEndUV.y - sEndUV1.y) < dParamTol)
        {
          sEndUV.x = sStartUV.x;
          if (lNumSolutions1 == 2)
            {
              sEndUV1.x = sStartUV1.x;
            }
        }
      else SER(SM_ERR);
    } // end EndPoint still dropped to surface multiple times check

  // when needed - Make the second UVcurve
  if (bMakeTwoCurves)
    {
      sm_CleanupNoise(sStartUV1);   // convert numbers of form 1.XXXX000000000X to 1.XXXX
      sm_CleanupNoise(sEndUV1);     // convert numbers of form 1.XXXX000000000X to 1.XXXX
      SER(SmBSplineCurve::CreateLineSegment(crContext,
                                            2,
                                            SmPoint3d(sStartUV1),
                                            SmPoint3d(sEndUV1),
                                            rpUVCurve2));
      NER(rpUVCurve2);
    }

  // Now, make the first curve

  sm_CleanupNoise(sStartUV);     // convert numbers of form 1.XXXX000000000X to 1.XXXX
  sm_CleanupNoise(sEndUV);       // convert numbers of form 1.XXXX000000000X to 1.XXXX

  // when curve is not an iso-curve, intersect the surface with a plane
  SmPoint2d sDomainSize = sUVDomain.GetSize();
  if(   smos_Fabs(sStartUV.x - sEndUV.x) > 0.01*sDomainSize.x
     && smos_Fabs(sStartUV.y - sEndUV.y) > 0.01*sDomainSize.y)
    {
      SmBSplineSurface * pBSP = SM_CAST_PTR(SmBSplineSurface, pSurface);

      // StartPt to EndPt 3d unit-vector
      SmVector3d sDir = sEndPnt - sStartPnt ;
      SER(sDir.Unitize());

      // plane locals: set PlaneNormal = (sZAxis * sDir) * sDir
      SmVector3d sPlaneNormal ;
      SmVector3d sZAxis(0.0,0.0,0.0);

      // set ZAxis =   pSurface->IsCone() ? Cone->GetZAxis : StartFV/EndFV vertexuse
      //             : StartFV/EndFV vertexuse ? pVU->GetTsectPnt().CrvDeriv()
      //             : [0 0 0]
      if (pBSP->IsKindOf(SmCone_TYPE))
        {
          SmCone * pCone = (SmCone*)pSurface;
          sZAxis = pCone->GetPosition().GetZAxis();
        }
      else // pick a likely ZAxis from the Start/EndFV->GetTsectPnt().CrvDeriv() values
        {
          // find a fillet vertexuse to target
          SmFilletVertexuse * pVU = NULL;

          // get a Vertexuse attached to the pStartFV
          SmTArray<SmFilletGeom*> sGeoms;
          pStartFV->GetFilletGeoms(sGeoms);
          if (sGeoms.GetSize() == 1)
            {
              pVU = pStartFV->GetVUAtRailEnd(sGeoms[0]);
            }
          else // get a vertexuse attached to the pEndFV
            {
              pEndFV->GetFilletGeoms(sGeoms);
              if (sGeoms.GetSize() == 1)
                {
                  pVU = pEndFV->GetVUAtRailEnd(sGeoms[0]);
                }
            }

          // when a vertexuse was found
          if (pVU)
            {
              SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();
              sZAxis = rTsectPnt.CrvDeriv();
            }

        } // end set ZAxis from filletVertexuse->GetTsectPnt().CrvDeriv() branch

      // when ZAxis was set for cones or from Start/End vertexuse->GetTsectPnt().CrvDeriv()
      if (sZAxis.Length() > SM_EFF_ZERO_SQRT)
        {
          // build plane with planeNormal perp to sDir in [sZAxis sDir] plane
          // where sDir = 3d unit-vec from StartPt to EndPt
          SmVector3d sViewVec = sZAxis   * sDir;
          sPlaneNormal        = sViewVec * sDir;
          SER(sPlaneNormal.Unitize());
          SmPlane * pPlane = new (crContext) SmPlane(sEndPnt,sPlaneNormal);
          NER(pPlane);
          SmObjDelete sDelete(pPlane);
          SER(pPlane->MakeNurb());

          // surface intersection initial tol
          SmApproxTol3d sSSITol = dApproxTol / 100;

          // iter a sequence of SrfSrfIntersections with increasing tolerance - until one works
          for (ULONG jj=0; jj<4; jj++)
            {
              SmBSplineCurve * pUVCurve2 = NULL;
              if(SM_SUCCESS == sm_SrfSrfIntersection(crContext,         // in : context for new object construction
                                                     pStartFV,          // in : 1st point known to be on intersection curve
                                                     pEndFV,            // in : 2nd point known to be on intersection curve,
                                                                        //      NULL to ignore
                                                    &sDir,              // in : expected general direction of intersection curve from 1st point
                                                     NULL,              // in   expected intersection end direction
                                                     pBSP,              // in : 1st intersecting surface
                                                     pPlane,            // in : 2nd intersecting surface
                                                     sSSITol,           // in : max allowed distance between xSect Curve and surfaces
                                                     30.0*SM_PI/180.0,  // in : max allowed angle between consecutive xSect curve segment tangents
                                                     rp3DCurve,         // out: 3d intersection curve
                                                     rpUVCurve1,        // out: associated UVTrimCurve on 1st surface
                                                     pUVCurve2,         // out: associated UVTrimCurve on 2nd surface
                                                     FALSE,             // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                                        //             resorting to expensive general surf/surf xSect solver
                                                     NULL,              // in : reference point
                                                     iDebugLevel))      // in : iDebugLevel, 0 = No Debug output
                {
                  // when Srf/Srf worked - don't need 2nd UVcurve
                  SM_ASSERT(pUVCurve2 != NULL) ; delete pUVCurve2 ; pUVCurve2 = NULL ;
#ifdef SM_DEBUG_CODE
                  if ( iDebugLevel > 0 )
                    {
                      smgfx_SetColor(1,0,0);
                      rp3DCurve->Draw();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                  // all done
                  return SM_SUCCESS;
                }

              // try Srf/Srf xsect again with larger tolerance
              sSSITol *= 10.0;

            } // end iter srf/srf xsects with increasing tols
        } // end Found a Z Axis from pSurface or start/end vertexuse->GetTsectPnt().CrvDeriv check

      // GWC: it seems that running through here would be a bug
      //      We know the curve is not an isoCurve but the nonIsoCurve build attempt failed
      // GWC: The only case in prog_test that ran through here created a LineSegment
      //      after this loop and that line segment worked fine.  Perhaps not a bug -
      // [prog_test: Fillet 408]
      //      SM_ASSERT_MSG(FALSE, _T("sm_CreateUVCurve - NonIsoParam UVCurve build branch failed - this is a bug")) ;

    } // end desired UVcurve is NOT an isoparam curve check

  // Otherwise, make an iso-curve
  SER(SmBSplineCurve::CreateLineSegment(crContext,
                                        2,
                                        SmPoint3d(sStartUV),
                                        SmPoint3d(sEndUV),
                                        rpUVCurve1));
  NER(rpUVCurve1);

  // all done
  return SM_SUCCESS;

} // end sm_CreateUVCurve

/*******************************************************************//**
PURPOSE: In some Nx2 tangent corners, rail curves from two adjacent fillets
     failed to meet at the same vertex locations. Will try to fix the
     geometry by matching the boundary control points of two fillets


NOTES: This currently assumes that the fillet surfaces have
     not been trimmed, that the end rib curves go all the way across the
     domains.  It should be enhanced to handle trimmed ribs.
***********************************************************************/
static SmStatus sm_ExamineNFixFilletGeoms
 (SmFilletGeom   * pFilletGeom1,
  SmBoolean        bAtStartOfGeom1,
  SmFilletGeom   * pFilletGeom2,
  SmBoolean        bAtStartOfGeom2,
  SmFilletVertex * pStartFV,
  SmFilletVertex * pEndFV,
  double           dDistTol,
  int              iDebugLevel)
{
#ifdef SM_DEBUG_CODE
  TCHAR sBuff[SM_TBLOCK_SIZE];
  if ( iDebugLevel > 0 ) 
    {
      ULONG lFGIdx1 = pFilletGeom1->FindIndexInSolver() ;
      ULONG lFGIdx2 = pFilletGeom2->FindIndexInSolver() ;
      ULONG lFSIdx1 = pFilletGeom1->GetFilletSolver()->FindIndexInExec() ;
      ULONG lFSIdx2 = pFilletGeom2->GetFilletSolver()->FindIndexInExec() ;

      ULONG lFVIdx1 = pStartFV->FindIndexInCorner() ;
      ULONG lFVIdx2 = pEndFV  ->FindIndexInCorner() ;
      ULONG lFCIdx1 = pStartFV->GetFilletCorner()->FindIndexInExec() ;
      ULONG lFCIdx2 = pEndFV  ->GetFilletCorner()->FindIndexInExec() ;
      smos_sprintf(sBuff, _T("  enter sm_ExamineNFixFilletGeoms: FGs [%4ld][%4ld] and [%4ld][%4ld], AtStart %3d %3d, FVs [%4ld][%4ld] and [%4ld][%4ld]\n"),
                 lFGIdx1, lFSIdx1, lFGIdx2, lFSIdx2,
                  bAtStartOfGeom1, bAtStartOfGeom2,
                  lFVIdx1, lFCIdx1, lFVIdx2, lFCIdx2 ) ;
      smos_WriteBuffer(sBuff) ;
    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  SmPoint3d sStartPnt = pStartFV->GetPoint();
  SmPoint3d sEndPnt   = pEndFV->GetPoint();
    
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 3 line
  // rm : SmBSplineSurface * pSurface1 = pFilletGeom1->GetFilletSurface();
  // rm : SmBSplineSurface * pSurface2 = pFilletGeom2->GetFilletSurface();
  // rm : SmBSplineSurface * apSurf[2] = { pSurface1, pSurface2 };
  SM_FILLETSURF_TYPE * pSurface1 = pFilletGeom1->GetFilletSurface();
  SM_FILLETSURF_TYPE * pSurface2 = pFilletGeom2->GetFilletSurface();
  SM_FILLETSURF_TYPE * apSurf[2] = { pSurface1, pSurface2 };
  NER(pSurface1) ; NER(pSurface2) ;

  // check state - only high degree bspline surfaces can be examined
  if(   pSurface1->GetDegree(SM_SP_U) < 3
     || pSurface2->GetDegree(SM_SP_U) < 3
     
     // GWC: Stop Fillets from switching SmCurve and SmSurface objects to SmBSplineCurve and SmBSplineSurface objs
     //      this method only works on BSplineSurface Fillets
     || !pSurface1->IsKindOf(SmBSplineSurface_TYPE) 
     || !pSurface2->IsKindOf(SmBSplineSurface_TYPE) ) 
    {
      // Can only fix non-analytic nurb fillets

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          smos_sprintf( sBuff, _T("%s"),_T("      quitting: analytic.\n"));
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE
      return SM_SUCCESS;
    } // end Surface Degree check

   // GWC: Stop Fillets from switching SmCurve and SmSurface objects to SmBSplineCurve and SmBSplineSurface objs
   SM_ASSERT_MSG(   pSurface1->IsKindOf(SmBSplineSurface_TYPE)
                 && pSurface2->IsKindOf(SmBSplineSurface_TYPE),
                 _T("sm_ExamineNFixFilletGeoms: Changing FilletSurfaces from SmBSplineSurface to any derived type of SmBSplineSurface - has stopped this method from running - review")) ; 

  // check state - only bspline surfaces can be examined
  if(   !pSurface1->IsKindOf(SmBSplineSurface_TYPE) 
     || !pSurface2->IsKindOf(SmBSplineSurface_TYPE) )
    {
#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          smos_sprintf( sBuff,_T("%s"), _T("      quitting: FilletSurfaces are not both BSplineSurfaces. \n"));
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE
      return SM_SUCCESS;
    } // end Surface Type check

  // Determine which fillet surface need be adjusted
  SmBoolean    abAtStartOfGeom[2] = { bAtStartOfGeom1, bAtStartOfGeom2 } ;
  SmBoolean    abAdjustSurf[2]    = { TRUE, TRUE } ;
  SmOrientType aeOrient[2]        = { SM_OT_SAME, SM_OT_SAME } ;

  // iter each Surface
  for (ULONG i=0; i<2; i++) 
    {
      SmExtent2d sDomain = apSurf[i]->GetNaturalUVDomain();
      SmPoint2d sUV1, sUV2;
      if (abAtStartOfGeom[i]) 
        {
          sUV1 = sDomain.Evaluate(0.0,0.0);
          sUV2 = sDomain.Evaluate(0.0,1.0);
        }
      else 
        {
          sUV1 = sDomain.Evaluate(1.0,0.0);
          sUV2 = sDomain.Evaluate(1.0,1.0);
        }

      SmPoint3d sPnt1, sPnt2;
      SER(apSurf[i]->EvaluatePoint(sUV1,sPnt1));
      SER(apSurf[i]->EvaluatePoint(sUV2,sPnt2));
      double dStartDist, dEndDist, dDist2;
      dStartDist = sStartPnt.DistanceBetween(sPnt1);
      dDist2 = sStartPnt.DistanceBetween(sPnt2);
      if (dStartDist < dDist2) 
        {
          dEndDist = sEndPnt.DistanceBetween(sPnt2);
        }
      else 
        {
          dStartDist = dDist2;
          dEndDist = sEndPnt.DistanceBetween(sPnt1);
          aeOrient[i] = SM_OT_OPPOSITE;
        }

      if (dStartDist < dDistTol && dEndDist < dDistTol) 
        {
          abAdjustSurf[i] = FALSE;
        }

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          smgfx_SetColor(1,0,0) ; sStartPnt.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetColor(1,0,0) ; sEndPnt.Draw() ; sm_GraphicsLoop() ;
          if ( iDebugLevel >= 10 ) 
            {
              smgfx_SetColor(0,0,0) ; apSurf[i]->DrawUV(0,0) ; sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    } // end iter both surfaces


#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 ) 
    {
      if      (!abAdjustSurf[0] && !abAdjustSurf[1]) { smos_sprintf( sBuff, _T("%s"), _T("      quitting: no adjustment needed.\n")) ; }
      else if ( abAdjustSurf[0] && abAdjustSurf[1] ) { smos_sprintf( sBuff, _T("%s"), _T("      quitting: Error: both surfaces need adjustment.\n")) ; }
      else                                           { smos_sprintf( sBuff, _T("%s"), _T("      Doing adjustment.\n")) ; }
      smos_WriteBuffer(sBuff);
    }
#endif // SM_DEBUG_CODE

  if (!abAdjustSurf[0] && !abAdjustSurf[1]) 
    {
      // Nothing need to be adjusted
      return SM_SUCCESS;
    }
  if (abAdjustSurf[0] && abAdjustSurf[1]) 
    {
      // Can not adjust both surfaces
      return SM_ERR;
    }

  // Copy one row of control points from the 'good'
  // fillet to the 'bad' fillet. Belows are some
  // low-level data access.
  gw_CPOINT * pFromCPOINT        = NULL;
  gw_CPOINT * pToCPOINT          = NULL;
  int         iFromCount         = 0 ;
  int         iToCount           = 0;
  SmBoolean   bCopyEndPointsOnly = FALSE;

  // for both surfaces
  for (ULONG j=0; j<2; j++) 
    {
      gw_SURFACE * pGwSurf = ((SmBSplineSurface *)apSurf[j])->GetOrCreateGwNurbPointer() ;
      gw_CNET    * pCNET   = pGwSurf->net;
      if (j == 0) 
        {
          iFromCount = pCNET->m;
        }
      else if (iFromCount != pCNET->m) 
        {
          bCopyEndPointsOnly = TRUE;
        }

      gw_CPOINT * pCPOINT = pCNET->Pw[0];
      if (!abAtStartOfGeom[j]) 
        {
          pCPOINT = pCNET->Pw[pCNET->n];
        }
      if (abAdjustSurf[j]) 
        {
          pToCPOINT = pCPOINT;
          iToCount  = pCNET->m;
        }
      else 
        {
          pFromCPOINT = pCPOINT;
          iFromCount  = pCNET->m;
        }
    }

  if (aeOrient[0] == aeOrient[1]) 
    {
      if (bCopyEndPointsOnly) 
        {
          pToCPOINT[0]        = pFromCPOINT[0];
          pToCPOINT[iToCount] = pFromCPOINT[iFromCount];
        }
      else 
        {
          for (int kk=0; kk<=iToCount; kk++) 
            {
              pToCPOINT[kk] = pFromCPOINT[kk];
            }
        }
    }
  else 
    {
      if (bCopyEndPointsOnly) 
        {
          pToCPOINT[0]        = pFromCPOINT[iFromCount];
          pToCPOINT[iToCount] = pFromCPOINT[0];
        }
      else 
        {
          for (int kk=0; kk<=iToCount; kk++) 
            {
              pToCPOINT[kk] = pFromCPOINT[iToCount-kk];
            }
        }
    }

  // Recreate rail curves
  SmFilletGeom * pAdjustFG = pFilletGeom1;
  if (abAdjustSurf[1]) 
    {
      pAdjustFG = pFilletGeom2;
    }
  
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pAdjustSurf = pAdjustFG->GetFilletSurface();
  SM_FILLETSURF_TYPE * pAdjustSurf = pAdjustFG->GetFilletSurface();

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 ) 
    {
      smgfx_SetLook(2,4, 1,0,0) ; sStartPnt.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,1,0) ; sEndPnt.Draw() ;   sm_GraphicsLoop() ;
      if(iDebugLevel > 10) 
        { smgfx_SetLook(1,1, 0,0,0) ; pAdjustSurf->DrawUV(0,0) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  SmExtent2d        sSurfDomain = pAdjustSurf->GetNaturalUVDomain();
  const SmContext * cpContext   = pAdjustSurf->GetContext();
  SmFilletSolver  * pAdjustFS   = pAdjustFG->GetFilletSolver();

  //
  for(ULONG k=0; k<2; k++)
    {
      SmFilletEdge * pRail         = pAdjustFG->GetRail(k);
      double         dIsoParameter = sSurfDomain.GetMin().y;
      if (k==1)
        {
          dIsoParameter = sSurfDomain.GetMax().y;
        }

      // Produce iso-curve for rail
      SmBSplineCurve * pIsoCrv = NULL;
      SER(pAdjustSurf->CreateIsoParametricCurve(*cpContext,
                                                 SM_SP_V,
                                                 dIsoParameter,
                                                 0.0,
                                                 pIsoCrv)) ;
      pRail->SetCurve(pIsoCrv, TRUE) ; // TRUE = delete preExisting Curve
                                       // side effect: delete current pEdge->UVTrimCurves
      pIsoCrv->SetOwner(pRail) ;

      // Project the new curve onto the surface of the original brep
      SmOffsetSurface * pOffsetSurf     = pAdjustFS->GetSurface(k) ; NER(pOffsetSurf) ;
      double            dOrigOffsetDist = pOffsetSurf->GetOffsetDistance() ;
      pOffsetSurf->SetOffsetDistance(0.0) ;

      SmTArray<SmBSplineCurve*> sUVCurves ;
      double dMaxDistToSurf = 0.0 ;
      double dDeviation ;

      // Drop the 3d curve onto the surface
      SmStatus eStatus = pOffsetSurf->DropCurve(*cpContext,
                                                 pOffsetSurf->GetNaturalUVDomain(),
                                                *pIsoCrv,
                                                 pIsoCrv->GetNaturalInterval(),
                                                 dDistTol,
                                                 dMaxDistToSurf, // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                                 dDeviation,     // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                                 sUVCurves) ;
      SmObjsDelete<SmBSplineCurve*> sDelCurves(&sUVCurves) ;
      pOffsetSurf->SetOffsetDistance(dOrigOffsetDist) ;
      if (eStatus != SM_SUCCESS)
        { continue ; }

      // Will replace the original UV-curve when we have only one result
      if (sUVCurves.GetSize() == 1)
        {
          sDelCurves.Clear();
          SmFilletEdgeuse * pPrimEU        = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse() ;
          SmFilletEdgeuse * pMateEU        = (SmFilletEdgeuse*)pPrimEU->GetMate() ;
          SmBSplineCurve  * pUVCrvToDelete = pMateEU->GetUVTrimCurvePointer() ;
          SmBSplineCurve  * pUVCurve       = sUVCurves[0] ;
          pMateEU->SetUVCurve(pUVCurve) ;
          pUVCurve->SetOwner(pMateEU) ;
          SmExtent1d sIvl = pUVCurve->GetNaturalInterval() ;    // gwc: Is this a bug?
                                                               //      check to see that this interval matches the edge interval
                                                               //      perhaps the edge interval needs to be set
          if (pUVCrvToDelete)  { delete pUVCrvToDelete; pUVCrvToDelete = NULL ; }
        }
    }

  // all done
  return SM_SUCCESS;

} // end sm_ExamineNFixFilletGeoms

/*******************************************************************//**
    END - Static functions
***********************************************************************/

/*******************************************************************//**
PURPOSE: Constructor for the SmFilletEdge object.

NOTES:
***********************************************************************/
SmFilletEdge::SmFilletEdge
  (SmFilletBrep   * pPseudoBrep,
   SmFilletCorner * cpCorner)
 : SmEdge(),
   m_eType(SM_FE_UNKNOWN),
   m_pOrigEdge(NULL),
   m_pOrigFace(NULL),
   m_pCurveClass(NULL),
   m_cpCorner(cpCorner),
   m_eStatus(SM_FIL_UNPROCESSED),
   m_pFilletBrepEdge1(NULL),
   m_pFilletBrepEdge2(NULL)
{
    pPseudoBrep->AddEdge(this);

} // end SmFilletEdge::SmFilletEdge

/*******************************************************************//**
PURPOSE: Destructor for the SmFilletEdge object.

NOTES:
***********************************************************************/
SmFilletEdge::~SmFilletEdge
  ()
{
  // delete the curveClassification
  if (m_pCurveClass) { delete m_pCurveClass; m_pCurveClass = NULL ; }

  // delete all connected edgeuses and the edgeuses' connected vertexuses
  SmTArray<SmEdgeuse*> sEdgeuses;
  GetEdgeuses(sEdgeuses);
  for (ULONG i=0; i<sEdgeuses.GetSize(); i++)
    {
      SmEdgeuse   * pEU = sEdgeuses[i];
      SmVertexuse * pVU = pEU->GetVertexuse();
      pVU->GetVertex()->Remove(pVU);
      SM_ASSERT(pEU != NULL) ; delete pEU ; pEU = NULL ;
      SM_ASSERT(pVU != NULL) ; delete pVU ; pVU = NULL ;
    }

} // end SmFilletEdge::~SmFilletEdge destructor

/*******************************************************************//**
PURPOSE: Get the name of this edge

NOTES:
***********************************************************************/
void SmFilletEdge::GetName
  (TCHAR * pcMyName,
  size_t    lMyNameAllocSize)
{
    switch (m_eType) {
    case SM_FE_CROSS_SECTION:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_CROSS_SECTION"));
            break;
        }
    case SM_FE_FILLET_X_FILLET:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_FILLET_X_FILLET"));
            break;
        }
    case SM_FE_FILLET_X_SIDE_FACE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_FILLET_X_SIDE_FACE"));
            break;
        }
    case SM_FE_RAIL_RAIL_INTERPOLATION:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_RAIL_RAIL_INTERPOLATION"));
            break;
        }
    case SM_FE_ON_EDGE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_ON_EDGE"));
            break;
        }
    case SM_FE_ON_EXTEND_EDGE:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_ON_EXTEND_EDGE"));
            break;
        }
    case SM_FE_FILLET_END:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_FILLET_END"));
            break;
        }
    case SM_FE_SETBACK_RAIL:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_SETBACK_RAIL"));
            break;
        }
    case SM_FE_RAIL_EXTENSION:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_RAIL_EXTENSION"));
            break;
        }
    case SM_FE_RAIL:
        {
            smos_WStrCpy(pcMyName,lMyNameAllocSize,_T("SM_FE_RAIL"));
            break;
        }
    default:
        break;
    }

} // end SmFilletEdge::GetName

/*******************************************************************//**
PURPOSE: Calculate the geometry of this edge at a filletCorner.

NOTES:
  Branch on SmFilletEdge->m_eType to pass the call along to one of
       CalcCrossSection(crContext,m_cpCorner);
       CalcFilletIntFillet(crContext,m_cpCorner);
       CalcFilletIntSideFace(crContext,m_cpCorner);
       CalcRailRailInterpolation(crContext,m_cpCorner);
       CalcEdgeOnEdge(crContext,m_cpCorner);
       CalcEdgeOnExtendedEdge(crContext,m_cpCorner);
       CalcFilletEnd(crContext,m_cpCorner);

***********************************************************************/
SmStatus SmFilletEdge::CalcCornerEdgeGeom
  ()
{
  // check state - filletEdge must have been generated by a filletCorner
  NER(m_cpCorner);

  // local
  const SmContext & crContext = m_cpCorner->GetContext();

#ifdef SM_DEBUG_CODE
  // draw filletCorner->originalVertex and Brep
  if ( DebugLevel() > 0 )
    {
      const SmVertex * cpVertex = m_cpCorner->GetFilletedVertex();

      // draw Brep
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); cpVertex->GetBrep()->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();

      // draw vertex
      smgfx_SetLook(1,5, 1,0,0); cpVertex->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // switch on FilletEdge->m_eType to pass the call along
  switch (m_eType)
    {
      case SM_FE_CROSS_SECTION           : return CalcCrossSection(crContext,m_cpCorner);
      case SM_FE_FILLET_X_FILLET         : return CalcFilletIntFillet(crContext,m_cpCorner);
      case SM_FE_FILLET_X_SIDE_FACE      : return CalcFilletIntSideFace(crContext,m_cpCorner);
      case SM_FE_RAIL_RAIL_INTERPOLATION : return CalcRailRailInterpolation(crContext,m_cpCorner);
      case SM_FE_ON_EDGE                 : return CalcEdgeOnEdge(crContext,m_cpCorner);
      case SM_FE_ON_EXTEND_EDGE          : return CalcEdgeOnExtendedEdge(crContext,m_cpCorner);
      case SM_FE_FILLET_END              : return CalcFilletEnd(crContext,m_cpCorner);
      case SM_FE_SETBACK_RAIL            :
      case SM_FE_RAIL_EXTENSION          : return CalcSetBackRail(crContext,m_cpCorner);
      case SM_FE_RAIL                    :
      case SM_FE_PRECOMPUTED             : // Special pre-computed edge, NOT a general type
                                           break;
      case SM_FE_BLENDING_RAIL           :
      case SM_FE_CLIFF_RAIL              :
      case SM_FE_SPLIT_FACE              :
      case SM_FE_UNKNOWN                 : break;

    } // end switch on FilletEdge->m_eType

  return SM_SUCCESS;

} // end SmFilletEdge::CalcCornerEdgeGeom

/*******************************************************************//**
PURPOSE: Adjust the edge geometry of type SM_FE_CROSS_SECTION.
    (cross section of a fillet)

NOTES: This fix is based on BSpline control point manipulations
***********************************************************************/
SmStatus SmFilletEdge::AdjustCrossSection
  (const SmContext & crContext,
   SmFilletCorner  * pCorner,
   SmVertex        * pFVOfDegenerateCurve)
{
  // locals
  SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
  SmFilletVertex * pEndFV   = (SmFilletVertex*)GetOtherVertex(pStartFV);
  if(   pFVOfDegenerateCurve != NULL 
     && (pStartFV == pFVOfDegenerateCurve || pEndFV == pFVOfDegenerateCurve)) 
    {
      // Process only the side which is across the degenerate edge
      return SM_SUCCESS;
    }

  SmFilletEdgeuse             * pPrimEU     = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
  SmFilletGeom                * pFilletGeom = pPrimEU->GetFilletGeom();
  NER(pFilletGeom) ;           
  SmFilletSolver              * pFS         = pFilletGeom->GetFilletSolver();
  SmFilletSurfaceGenerator    * pFSG        = pFS->GetFilletSurfaceGenerator();
  if (pFSG->GetFilletSurfaceGeneratorType() != SM_FSG_BLEND_CURVE) 
    {
      // Don't need to adjust cases SM_FSG_LINEAR & SM_FSG_CIRCULAR
      return SM_SUCCESS;
    }
  SmBlendCurveCrossSectionFSG * pBlendFSG    = (SmBlendCurveCrossSectionFSG*)pFSG;
  ULONG                         lContinuity  = pBlendFSG->GetContinuity(); // 1, 2, or 3 for G1, G2 or G3
  //dBlendScale = pBlendFSG->GetBlendScale();

  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pFilletSurface = pFilletGeom->GetFilletSurface();
  SM_FILLETSURF_TYPE * pFilletSurface = pFilletGeom->GetFilletSurface();
  SM_ASSERT_MSG(GetCurve()->IsKindOf(SmBSplineCurve_TYPE),_T("Moving from SmBSplineCurve to SmCurve caused this method to not run"));
  NER(pFilletSurface) ;

  SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,GetCurve()) ; NER(pCurve) ;

  //double dTol = pCorner->GetThisApproxTol3d();
  //double dDist = 0.0;
  //double dRadius = 0.0;
  //double dBlendScale = 1.0;
  //SmBoolean bConstantDistance = TRUE;
  //SmBoolean bConstantRadius = TRUE;
  //ULONG lFilletCrossSection;  // 0 - circular,
  //// 1 - Approx Circular, 2 - Linear, 3 - G1 Blend, 4 - G2 Blend, 5 - G3 Blend
  //SmBoolean bIsFilletCrossSection;
  //SER(pCurve->TestForFilletCrossSection(crContext,dTol,
  //    bIsFilletCrossSection,lFilletCrossSection,dDist,dRadius,dBlendScale));
  //if (lFilletCrossSection == 2) {
  //    return SM_SUCCESS; // Don't adjust linear curve
  //}

  // Get "incoming" rail-curve parameter at start vertex
  SmTArray<SmEdge*> sEdges;
  pStartFV->GetEdges(sEdges);
  SmFilletEdge * pRail1 = NULL;
  for (ULONG i=0; i<sEdges.GetSize(); i++) 
    {
      SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[i];
      if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL) 
        {
          SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pFilletE->GetPrimaryEdgeuse();
          SmFilletGeom * pOtherFG = pEU->GetFilletGeom();
          NER(pOtherFG);
          if (pOtherFG != pFilletGeom) 
            {
              pRail1 = pFilletE;
              break;
            }
        }
    }
  NER(pRail1);

  SmEdgeuse       * pEU1          = pRail1->GetPrimaryEdgeuse()->GetMate();
  SmOrientType      eOrient1      = pEU1->GetOrientation();
  SmCurve         * pCurve1       = pRail1->GetCurve();
  SmExtent1d        sIvl1         = pCurve1->GetNaturalInterval();
  SmBoolean         bGetStartTan1 = FALSE;
  double            dParam1       = sIvl1.GetMax();
  if(   (pRail1->GetVertex() != pStartFV && eOrient1 == SM_OT_SAME) 
     || (pRail1->GetVertex() == pStartFV && eOrient1 == SM_OT_OPPOSITE)) 
    {
      bGetStartTan1 = TRUE;
      dParam1 = sIvl1.GetMin();
    }

  // Get "incoming" rail-curve parameter at end vertex
  pEndFV->GetEdges(sEdges);
  SmFilletEdge * pRail2 = NULL;

  for (ULONG j=0; j<sEdges.GetSize(); j++) 
    {
      SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[j];
      if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL) 
        {
          SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pFilletE->GetPrimaryEdgeuse();
          SmFilletGeom * pOtherFG = pEU->GetFilletGeom();
          NER(pOtherFG);
          if (pOtherFG != pFilletGeom) 
            {
              pRail2 = pFilletE;
              break;
            }
        }
    }
  NER(pRail2);

  SmEdgeuse        * pEU2          = pRail2->GetPrimaryEdgeuse()->GetMate();
  SmOrientType       eOrient2      = pEU2->GetOrientation();
  SmCurve          * pCurve2       = pRail2->GetCurve();
  SmExtent1d         sIvl2         = pCurve2->GetNaturalInterval();
  SmBoolean          bGetStartTan2 = FALSE;
  double             dParam2       = sIvl2.GetMax();
  if(   (pRail2->GetVertex() != pEndFV && eOrient2 == SM_OT_SAME) 
     || (pRail2->GetVertex() == pEndFV && eOrient2 == SM_OT_OPPOSITE)) 
    {
      bGetStartTan2 = TRUE;
      dParam2 = sIvl2.GetMin();
    }

  // Compute tangents and higher order derivatives if necessary
  ULONG lNumDerivatives = lContinuity;
  // 1 - Get first derivative and point
  // 2 - produces 2nd derivative, 1st derivative and point.
  // 3 - produces 3rd derivative and lower derivatives

  // Calc tangent from the rail at start vertex
  SmPoint3d sPntVec1[4];
  SER(pCurve1->Evaluate(dParam1,lNumDerivatives,TRUE,sPntVec1));
  if (bGetStartTan1) 
    {
      for (ULONG ii=1; ii<=lNumDerivatives; ii++) 
        {
          sPntVec1[ii] = -sPntVec1[ii];
        }
    }

  // Calc tangent from the other rail
  SmPoint3d sPntVec2[4];
  SER(pCurve2->Evaluate(dParam2,lNumDerivatives,TRUE,sPntVec2));
  if (!bGetStartTan2) 
    {
      for (ULONG ii=1; ii<=lNumDerivatives; ii++) 
        {
          sPntVec2[ii] = -sPntVec2[ii];
        }
    }

  // Fair the 3d points & tangents
  SmTArray<SmPoint3d>  sPoints;
  SmTArray<SmVector3d> sTangents;
  SmTArray<SmVector3d> sHighDerivs;

  // locals for FairBlendCurveDerivatives()
  sPoints.Add(sPntVec1[0]);     // Rail1 EndPoint
  sPoints.Add(sPntVec2[0]);     // Rail2 EndPoint
  sTangents.Add(sPntVec1[1]);   // Rail1 End 1stDeriv
  sTangents.Add(sPntVec2[1]);   // Rail2 End 1stDeriv

  // for every higher order derivative
  for (ULONG k=2; k<=lNumDerivatives; k++) 
    {
      sHighDerivs.Add(sPntVec1[k]); // Rail1 End kth Deriv
      sHighDerivs.Add(sPntVec2[k]); // Rail2 End kth Deriv
    }

  // set smooth sTangents and optional sHighDerivs values
  SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints, sTangents, &sHighDerivs));

  sPntVec1[1] = sTangents[0];
  sPntVec2[1] = sTangents[1];
  ULONG lCount=0;
  for (ULONG m=2; m<=lNumDerivatives; m++) 
    {
      sPntVec1[m] = sHighDerivs[lCount++];
      sPntVec2[m] = sHighDerivs[lCount++];
    }

  // Drop derivative vectors onto fillet surface
  SmBSplineCurve * pPSCurve = pPrimEU->GetUVTrimCurve();
  SmExtent1d       sIvl = pPSCurve->GetNaturalInterval();
  SmPoint3d        sPnt ;
  SmVector2d       sUVDerivs1[3];
  SmVector2d       sUVDerivs2[3];

  SER(pPSCurve->EvaluatePoint(sIvl.GetMin(),sPnt)) ;
  SmPoint2d sUV1(sPnt.x,sPnt.y);
  SER(pPSCurve->EvaluatePoint(sIvl.GetMax(),sPnt)) ;
  SmPoint2d sUV2(sPnt.x,sPnt.y);
  SER(pFilletSurface->DropVectors(sUV1,TRUE,TRUE,lNumDerivatives,&sPntVec1[1],sUVDerivs1));
  SER(pFilletSurface->DropVectors(sUV2,TRUE,TRUE,lNumDerivatives,&sPntVec2[1],sUVDerivs2));

  if (   smos_Fabs(sUV1.x - sUV2.x) < SM_EFF_ZERO
      && smos_Fabs(sUVDerivs1[0].x) < SM_EFF_ZERO
      && smos_Fabs(sUVDerivs2[0].x) < SM_EFF_ZERO)
    {
      // Skip it since the iso-curve is exactly what we need
      return SM_SUCCESS;
    }

  // Recompute uv & 3d curves
  SmTArray<SmPoint3d>  sUVPoints;
  SmTArray<SmVector3d> sUVTangents;
  SmTArray<SmVector3d> sUVHighDerivs;

  sUVPoints.Add(SmPoint3d(sUV1));
  sUVPoints.Add(SmPoint3d(sUV2));
  sUVTangents.Add(SmPoint3d(sUVDerivs1[0]));
  sUVTangents.Add(SmPoint3d(sUVDerivs2[0]));

  for (ULONG n=1; n<lNumDerivatives; n++)
    {
      sUVHighDerivs.Add(sUVDerivs1[n]);
      sUVHighDerivs.Add(sUVDerivs2[n]);
    }
  SmBSplineCurve * pNewUVCurve = NULL;
  SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,
                                               SM_CP_UNIFORM,
                                               2/*Dim*/,
                                               3/*Deg*/,
                                               sUVPoints,
                                               sUVTangents,
                                               &sUVHighDerivs,
                                               FALSE,
                                               pNewUVCurve));

  SmBSplineCurve * pNew3DCurve = NULL;
  double dDistance = 0.0;

  //SmExtent3d sBBox;
  //SER(pNewUVCurve->CalculateBoundingBox(pNewUVCurve->GetNaturalInterval(),&sBBox));
  //SmExtent2d sCurveUVDomain(sBBox);
  //SmExtent2d sUVDomain = pFilletSurface->GetNaturalUVDomain();
  //SmBoolean bExtendingSurface = !sCurveUVDomain.IsContainedBy(sUVDomain);
  SmBoolean bExtendingSurface = TRUE;
  if (bExtendingSurface)
    {
      double             dSize       = pCurve->ApproximateLength(pCurve->GetNaturalInterval(),5);
      SmSurface        * pNewSurface = NULL;
      SmBSplineSurface * pNewBSS     = NULL;
      SER(pFilletSurface->CreateExtendedSurface(crContext,
                                                SM_SP_U,
                                                dSize/5.0,
                                                SM_CT_G1R,
                                                pNewSurface));
      SM_ASSERT(pFilletSurface != NULL) ; delete pFilletSurface ; pFilletSurface = NULL ;
      pNewBSS = SM_CAST_PTR( SmBSplineSurface, pNewSurface );
      pFilletGeom->SetFilletSurface(pNewBSS);
      pFilletSurface = pNewBSS;
    }

  SmApproxTol3d dTol = pCorner->GetThisApproxTol3d();
  SER(pFilletSurface->LiftCurve(crContext,
                                pFilletSurface->GetNaturalUVDomain(),
                               *pNewUVCurve,
                                pNewUVCurve->GetNaturalInterval(),
                                SM_CAST_DOUBLE(dTol/10.0),
                                dDistance,
                                pNew3DCurve));
  NER(pNew3DCurve);

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      // if (0) 
      //   { smgfx_SetColor(0,1,0); pFilletSurface->DrawUV(3,3); sm_GraphicsLoop(); }
      smgfx_SetColor(0,1,0); pCurve->Draw(); pPSCurve->Draw(); sm_GraphicsLoop(); 
      smgfx_SetColor(1,0,0); pNew3DCurve->Draw(); sm_GraphicsLoop(); 
      smgfx_SetColor(1,0,0); pNewUVCurve->Draw(); sm_GraphicsLoop(); 
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  SetCurve(pNew3DCurve, TRUE);  // side effect: delete current this->UVTrimCurves
  pNew3DCurve->SetOwner(this);
  pPrimEU->SetUVCurve(pNewUVCurve);

  SM_ASSERT(pPSCurve != NULL) ; delete pPSCurve ; pPSCurve = NULL ;
  SM_ASSERT(pCurve   != NULL) ; delete pCurve ;   pCurve   = NULL ;

  // all done
  return SM_SUCCESS;

} // end SmFilletEdge::AdjustCrossSection

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_CROSS_SECTION.
    (cross section of a fillet)

NOTES:
***********************************************************************/
SmStatus SmFilletEdge::CalcCrossSection
  (const SmContext & crContext,
   SmFilletCorner * pCorner)
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

    SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
    SmFilletVertex * pEndFV = (SmFilletVertex*)GetOtherVertex(pStartFV);

    SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
    SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
    SmFilletGeom * pFilletGeom1 = pPrimEU->GetFilletGeom();
    SmFilletGeom * pFilletGeom2 = pMateEU->GetFilletGeom();
    SmFilletSolver * pFilSolver1 = NULL;
    SmFilletSolver * pFilSolver2 = NULL;

    SmSurface * pSurface1 = NULL;
    SmSurface * pSurface2 = NULL;
    double dTol;
    if ( pFilletGeom1 )
    {
        pFilSolver1 = pFilletGeom1->GetFilletSolver();
        pSurface1 = pFilletGeom1->GetFilletSurface(); NER(pSurface1);

        if ( pFilletGeom2 != NULL  &&  pFilletGeom2 != pFilletGeom1 )
        {
            pFilSolver2 = pFilletGeom2->GetFilletSolver(); NER(pFilSolver2);
            pSurface2 = pFilletGeom2->GetFilletSurface();
            NER(pSurface2);
        }
        dTol = pFilSolver1->GetThisApproxTol3d();

        // Examine the connectivity between two adjacent fillet geoms.
        // They should meet precisely along their common edge.
        // If not, adjust the control points of one to match the other.
        // Note, this won't work for analytic surfaces:
        // sm_ExamineNFixFilletGeoms() will refuse to modify them.
        // Also, sm_ExamineNFixFilletGeoms() assumes that the surfaces
        // have not been trimmed, that the common edges go all the way
        // across the surface domain.  Surfaces can be trimmed if the fillet
        // has rolled over a cliff, so don't do the check on those.
        // (It should be enhanced to handle that.)

        if (    pCorner && pFilletGeom2
             && pFilletGeom1->GetFilletGeomType() != SM_FG_CLIFF_ROLLOVER
             && pFilletGeom2->GetFilletGeomType() != SM_FG_CLIFF_ROLLOVER )
        {
            SmBoolean bAtStartOfGeom1 = TRUE;
            SmBoolean bAtStartOfGeom2 = FALSE;
            SmBoolean bCheckConnectivity = FALSE;
            const SmVertex * cpVertex = pCorner->GetFilletedVertex();
            switch (pCorner->GetCornerType())
            {
            case SM_FCR_N_x_2:
                bAtStartOfGeom1 = ( pFilSolver1->GetVertex(0) == cpVertex );

                bAtStartOfGeom2 = ( pFilSolver2->GetVertex(0) == cpVertex );

                bCheckConnectivity = TRUE;
                break;

            case SM_FCR_N_x_1_CLOSED:
                if ( pCorner->IsBlendingCorner() )
                {
                    bCheckConnectivity = TRUE;
                }
                break;

            case SM_FCR_UNKNOWN:
            case SM_FCR_OPEN:
            case SM_FCR_DEGENERATE:
            case SM_FCR_1_x_1:
            case SM_FCR_2_x_2:
            case SM_FCR_3_x_2_MIXED:
            case SM_FCR_4_x_3:
            case SM_FCR_N_x_1:
            case SM_FCR_N_x_N:
            case SM_FCR_N_x_N_CONVEX:
            case SM_FCR_N_x_N_CONCAVE:
               break;
            }  // end case

            if ( bCheckConnectivity )
            {
                sm_ExamineNFixFilletGeoms(
                    pFilletGeom1, bAtStartOfGeom1,
                    pFilletGeom2, bAtStartOfGeom2,
                    pStartFV, pEndFV, dTol, iDebugLevel );
            }
        }
    }
    else
    {
        SmFace * pOrigFace = GetOriginalFace();  NER(pOrigFace);
        pSurface1 = pOrigFace->GetSurface();     NER(pSurface1);
        dTol = pOrigFace->GetTolerance();
    }
#ifdef SM_DEBUG_CODE
    TCHAR sBuff[SM_TBLOCK_SIZE];
    if ( iDebugLevel > 0 ) {
        smgfx_Erase();
        smgfx_SetLook(1,3, 1,0,0); pStartFV->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,3, 1,0,1); pEndFV->Draw(); sm_GraphicsLoop();
        if ( iDebugLevel > 10 ) {
            smgfx_SetLook(1,1, 0,0,0); pSurface1->DrawUV(0,0); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,1,1); if (pSurface2) pSurface2->DrawUV(0,0);
        }
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Drop start & end points onto pFilletSurface1, then
    // join them with a uv-curve
    SmBSplineCurve * p3DCurve = NULL;
    SmBSplineCurve * pUVCurve1 = NULL;
    SmBSplineCurve * pUVCurve2 = NULL;
    SmBoolean bCreateTwoBoundaryCurves = FALSE;

    // Need to create two UV curves when we had a closed-tangent fillet surface
    if (    pCorner != NULL
         && pCorner->GetCornerType() == SM_FCR_N_x_1_CLOSED
         && pFilletGeom1 == pFilletGeom2
         && pCorner->IsBlendingCorner() )
    {
        bCreateTwoBoundaryCurves = TRUE;
    }

    SER( sm_CreateUVCurve( crContext,
                           pStartFV,
                           pEndFV,
                           bCreateTwoBoundaryCurves,
                           pSurface1,
                           50.0*dTol,
                           p3DCurve,
                           pUVCurve1,
                           pUVCurve2 ));

    if (p3DCurve == NULL)
    {
        // Lift uv curve
        double dDist = 0.0;
        SER( pSurface1->LiftCurve( crContext,
                                   pSurface1->GetNaturalUVDomain(),
                                   *pUVCurve1,
                                   pUVCurve1->GetNaturalInterval(),
                                   dTol/10.0,
                                   dDist,
                                   p3DCurve ));
        NER(p3DCurve);
    }

#ifdef SM_DEBUG_CODE
    if ( iDebugLevel > 0 ) {
        smos_sprintf( sBuff,_T("%s"), _T("      after sm_CreateUVCurve:\n"));
        smos_WriteBuffer(sBuff);
        //ULONG lFGIdx1 = 9999, lFSIdx1 = 9999, lFGIdx2 = 9999, lFSIdx2 = 9999;
        if ( pFilletGeom1 ) {
            //lFGIdx1 = pFilletGeom1->FindIndexInSolver();
            //lFSIdx1 = pFilletGeom1->GetFilletSolver()->FindIndexInExec();
        }
        if ( pFilletGeom2 ) {
            //lFGIdx2 = pFilletGeom2->FindIndexInSolver();
            //lFSIdx2 = pFilletGeom2->GetFilletSolver()->FindIndexInExec();
        }
        ULONG lFVIdx1 = pStartFV->FindIndexInCorner();
        ULONG lFVIdx2 = pEndFV  ->FindIndexInCorner();
        ULONG lFCIdx1 = pStartFV->GetFilletCorner()->FindIndexInExec();
        ULONG lFCIdx2 = pEndFV  ->GetFilletCorner()->FindIndexInExec();

        smos_sprintf( sBuff, _T("         FVs [%4ld][%4ld] and [%4ld][%4ld], create two? %3d\n"),
                            lFVIdx1, lFCIdx1, lFVIdx2, lFCIdx2, bCreateTwoBoundaryCurves );
        smos_WriteBuffer(sBuff);
        smos_sprintf( sBuff, _T("         UV crv 1 %s, UV crv 2 %s, 3d crv %s.\n"),
                (pUVCurve1) ? _T("notNULL") : _T("NULL"),
                (pUVCurve2) ? _T("notNULL") : _T("NULL"),
                (p3DCurve ) ? _T("notNULL") : _T("NULL")  );
        smos_WriteBuffer(sBuff);
        if ( iDebugLevel > 20 ) {
            smos_sprintf( sBuff, _T("%s"),_T("         Dumps:\n"));
            smos_WriteBuffer(sBuff);
            SmBoolean bAbbrev = ( iDebugLevel < 60 );
            if ( pUVCurve1 != NULL ) { pUVCurve1->Dump( bAbbrev ); }
            if ( pUVCurve2 != NULL ) { pUVCurve2->Dump( bAbbrev ); }
            if ( p3DCurve  != NULL ) { p3DCurve ->Dump( bAbbrev ); }
        }
    }
#endif // SM_DEBUG_CODE

    SmObjDelete sDeleteUVCurve2;
    if ( pUVCurve2 )
    {
        sDeleteUVCurve2.SetObj(pUVCurve2);
    }
    else
    {
        // Got only one uv-curve. Reparametrize the 3d curve
        // with arc-length if it is not linear.
        //SmPoint3d sLinePt;
        //SmVector3d sLineVec;
        //if (!p3DCurve->IsLine(5,dTol,sLinePt,sLineVec)) {
        //    SER(p3DCurve->ReparametrizeWithArcLength());
        //    // Delete UV curve
        //    delete pUVCurve1;
        //    SmTArray<SmBSplineCurve*> sUVCurves;
        //    double dMaxDistToSurf = 0.0;
        //    double dDeviation;
        //    // Reproject 3d curve back onto the surface
        //    SER(pSurface1->DropCurve(crContext,pSurface1->GetNaturalUVDomain(),
        //                            *p3DCurve,p3DCurve->GetNaturalInterval(),dTol,
        //        dMaxDistToSurf,dDeviation,sUVCurves));
        //    SM_ASSERT(sUVCurves.GetSize() == 1);
        //    if ( sUVCurves.GetSize() < 1 )
        //      { SER( SM_ERR ); }
        //    pUVCurve1 = sUVCurves[0];
        //}
    }

    SetCurve( p3DCurve, TRUE ); // side effect: delete current this->UVTrimCurves
    p3DCurve->SetOwner( this );
    pPrimEU->SetUVCurve( pUVCurve1 );

#ifdef SM_DEBUG_CODE
    if ( iDebugLevel > 0 )
      {
        smgfx_SetLook(1,2, 1,0,0);
        p3DCurve->Draw();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // Need second UV curve when we had a closed fillet surface
    if (    pCorner != NULL
         && pCorner->GetCornerType() == SM_FCR_N_x_1_CLOSED
         && pFilletGeom1 == pFilletGeom2 )
    {
        NER(pUVCurve2);
        sDeleteUVCurve2.Clear();
        pMateEU->SetUVCurve(pUVCurve2);
        pFilletGeom1->AddSideFilletEdgeuse(pMateEU);
    }

    if ( pFilletGeom1 )
    {
        pFilletGeom1->AddSideFilletEdgeuse(pPrimEU);
    }

    // See if we need to add side curve for the other solver
    if ( pSurface2 )
    {
        dTol = pFilSolver2->GetThisApproxTol3d();
        SmBSplineCurve * p3DCrv = NULL;
        SmBSplineCurve * pUVCrv1 = NULL;
        SmBSplineCurve * pUVCrv2 = NULL;
        SER( sm_CreateUVCurve( crContext,
                               pStartFV,
                               pEndFV,
                               FALSE,
                               pSurface2,
                               50.0*dTol,
                               p3DCrv,
                               pUVCrv1,
                               pUVCrv2 ));

        if ( p3DCrv )
        {
            delete p3DCrv; p3DCrv = NULL ; // Duplicate curve
        }

        if ( pUVCrv2 )
        {
            // Unknown case, return error for now.
            //delete pUVCrv1; pUVCrv1 = NULL ;
            delete pUVCrv2; pUVCrv2 = NULL ;
            //SER(SM_ERR);
        }

        pMateEU->SetUVCurve( pUVCrv1 );
        pFilletGeom2->AddSideFilletEdgeuse( pMateEU );

#ifdef SM_DEBUG_CODE
        if ( iDebugLevel > 20 )
          {
            SmBSplineCurve * p3DCrv1 = NULL;
            double dDist;
            SER( pSurface2->LiftCurve( crContext,
                                       pSurface2->GetNaturalUVDomain(),
                                       *pUVCrv1,
                                       pUVCrv1->GetNaturalInterval(),
                                       dTol/10.0,
                                       dDist,
                                       p3DCrv1 ));
            NER(p3DCrv1);
            smgfx_SetColor(1,0,0);
            p3DCrv1->Draw();
            sm_GraphicsLoop();
            SM_ASSERT( p3DCrv1 != NULL );
            delete p3DCrv1; p3DCrv1 = NULL;
        }
#endif // SM_DEBUG_CODE
    }

    return SM_SUCCESS;

} // end SmFilletEdge::CalcCrossSection

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_FILLET_X_FILLET.
    (fillet-fillet-intersection)

NOTES:
 FilletEdge represents a FilletSurface/FilletSurface intersection.

 Set SmFilletEdge->Curve,
     SmFilletEdge->UVTrimCurves
     SmFilletEdge->Status = processed

METHOD ---
  1. add this FilletEdge->Edgeuses to FilletGeoms Side Edgeuse lists

  2. when just 1 FilletEdge->Vertex has not been processed
       pass the call to SmFilletVertex::CalcFilletIntFillet to
       find the Vertex-Point and Edge->Curve

  else
  3a. if SmFilletEdge->3DCurve == NULL
      - use 2 Pt intersection to find 3DCurve
      - set FilletEdge->Curve and FilletEdge->UVTrimCurves
  3b. else if SmFilletEdge->3DCurve != NULL
      - trim FilletEdge->Curve to FilletVertex points

  4. set FilletEdge->Status = processed

returns SM_ERR when
    a. neither FilletEdge->Vertex has been processed
    b. attempt to find FilletSurface/FilletSurface intersection fails

***********************************************************************/
SmStatus SmFilletEdge::CalcFilletIntFillet
  (const SmContext & crContext, // NotUsed: in : context for new object construction
   SmFilletCorner  * pCorner)   // in : filletCorner which generated this filletEdge
{
  SM_REF1(crContext) ;
  // check state
  SM_ASSERT(pCorner == m_cpCorner) ;

  // locals: get filletSurface FilletGeoms
  SmFilletEdgeuse * pPrimEU      = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
  SmFilletEdgeuse * pMateEU      = (SmFilletEdgeuse*)pPrimEU->GetMate();
  SmFilletGeom    * pFilletGeom1 = pPrimEU->GetFilletGeom();
  SmFilletGeom    * pFilletGeom2 = pMateEU->GetFilletGeom();
  NER(pFilletGeom1);NER(pFilletGeom2);

  // add this FilletEdge->Edgeuses to FilletGeoms Side Edgeuse lists
  pFilletGeom1->AddSideFilletEdgeuse(pPrimEU);
  pFilletGeom2->AddSideFilletEdgeuse(pMateEU);

  // get FilletEdge->Vertices
  SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
  SmFilletVertex * pEndFV   = (SmFilletVertex*)GetOtherVertex(pStartFV);

  // check state - at least one FilletEdge->Vertex is processed
  if (   !pStartFV->IsProcessed()
      && !pEndFV->IsProcessed())
    {
      SER(SM_ERR);
    }

  // when either vertex is not processed
  // pass the call to SmFilletVertex::CalcFilletIntFillet to
  //  compute and save SmEdge->Curve, SmEdge->UVCurves, and SmVertex->Point
  if (!pStartFV->IsProcessed())
    {
      SER(pStartFV->CalcFilletIntFillet());
      return SM_SUCCESS;
    }
  if (!pEndFV->IsProcessed())
    {
      SER(pEndFV->CalcFilletIntFillet());
      return SM_SUCCESS;
    }

  // Do surface/surface intersection

  // get intersecting filletSurfaces and FilletSolvers
  SmFilletSolver   * pFilSolver1     = pFilletGeom1->GetFilletSolver();
  SmFilletSolver   * pFilSolver2     = pFilletGeom2->GetFilletSolver();

  
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 2 line
  // rm : SmBSplineSurface * pFilletSurface1 = pFilletGeom1->GetFilletSurface();
  // rm : SmBSplineSurface * pFilletSurface2 = pFilletGeom2->GetFilletSurface();
  SM_FILLETSURF_TYPE * pFilletSurface1 = pFilletGeom1->GetFilletSurface();
  SM_FILLETSURF_TYPE * pFilletSurface2 = pFilletGeom2->GetFilletSurface();
  NER(pFilletSurface1);
  NER(pFilletSurface2);

  // get 3DCurve and UVTrimCurve pointers for storing upcoming intersections
  SmBSplineCurve * p3DCurve  = SM_CAST_PTR(SmBSplineCurve,GetCurve());
  SmBSplineCurve * pUVCurve1 = pPrimEU->GetUVTrimCurvePointer();
  SmBSplineCurve * pUVCurve2 = pMateEU->GetUVTrimCurvePointer();

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
  iDebugLevel = DebugLevel();
  // draw filletSurfaces, intersection curve endPoints, and 3dCurve (if there is one)
  if ( iDebugLevel > 0 )
    {
      smgfx_SetLook(3,6, 0,0,1); pFilletSurface1->DrawUV(0,0); sm_GraphicsLoop();
      smgfx_SetLook(3,6, 1,0,0); pFilletSurface2->DrawUV(0,0); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pStartFV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,1); pEndFV->Draw();   sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); if (p3DCurve) { p3DCurve->DrawWDeriv(p3DCurve->GetNaturalInterval(),0); } sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // pick appoxTol and angTol as tightest filletSolver tolerances
  double dApproxTol = smos_Min(pFilSolver1->GetThisApproxTol3d(),
                               pFilSolver2->GetThisApproxTol3d());
  double dAngTol    = smos_Min(pFilSolver1->GetThisAngTolRad(),
                               pFilSolver2->GetThisAngTolRad());

  // when 3D curve has not yet been calculated
  if ( p3DCurve == NULL )
    {
      // Use 2-pts intersector - which can try the global surf/surf intersector as a last resort
      SmPoint3d sStartPnt = pStartFV->GetPoint();
      SmPoint3d sEndPnt   = pEndFV->GetPoint();
      SmVector3d sDir     = sEndPnt - sStartPnt;
      if(SM_SUCCESS != sm_SrfSrfIntersection(m_cpCorner->GetContext(), // in : context for new object construction
                                             pStartFV,                 // in : 1st point known to be on intersection curve
                                             pEndFV,                   // in : 2nd point known to be on intersection curve,
                                                                       //      NULL to ignore
                                             &sDir,                    // in : expected general direction of intersection curve from 1st point
                                             &sDir,                    // in   expected intersection end direction
                                             pFilletSurface1,          // in : 1st intersecting surface
                                             pFilletSurface2,          // in : 2nd intersecting surface
                                             dApproxTol,               // in : max allowed distance between xSect Curve and surfaces
                                             dAngTol,                  // in : max allowed angle between consecutive xSect curve segment tangents
                                             p3DCurve,                 // out: 3d intersection curve
                                             pUVCurve1,                // out: associated UVTrimCurve on 1st surface
                                             pUVCurve2,                // out: associated UVTrimCurve on 2nd surface
                                             FALSE,                    // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                                       //             resorting to expensive general surf/surf xSect solver
                                             NULL,                     // in : reference point
                                             iDebugLevel)              // in : iDebugLevel, 0 = No Debug output
          || p3DCurve == NULL )
        { // error: sm_SrfSrfIntersection() failed or failed to produce a p3DCurve
          SetStatus(SM_FIL_SURF_INT_FAILURE);

          pCorner->RecordFilletError(SM_FILERR_BAD_INT,
                                     pFilletSurface1, pFilletSurface2,
                                     pStartFV, pEndFV,
                     _T("SmFilletEdge::CalcFilletIntFillet(): failure to calculate intersection of two fillet surfaces") );

          SER(SM_ERR);
        }

      // set SmFilletEdge Curve and UVTrimCurves
      SetCurve(p3DCurve, TRUE); // side effect: delete current this->UVTrimCurves
      p3DCurve->SetOwner(this);
      pPrimEU->SetUVCurve(pUVCurve1);
      pMateEU->SetUVCurve(pUVCurve2);
    } // end p3DCurve == NULL branch

  else // Trim the intersection curve by Start and End FilletPoint
    {
      SmExtent1d sTrimIvl = p3DCurve->GetNaturalInterval();
      SmBoolean bSuccess;

      // drop StartFV->Point to 3DCurve
      double dParam1, dParam2, dDist;
      if (   SM_SUCCESS != p3DCurve->DropPoint(sTrimIvl,             // in : target curve allowed domain
                                               pStartFV->GetPoint(), // in : Point to drop to curve
                                               NULL,                 // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                     //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                     //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                               dApproxTol*10.0,      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                     //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                     //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                     //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                               NULL,                 // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                               bSuccess,             // out: TRUE = found a drop point
                                               dParam1,              // out: found drop curve param
                                               dDist)                // out: found drop distance
                                                                     // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                     //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                     //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                     //      default:[SM_SO_MINIMIZE] to preserve original behavior
          || !bSuccess)
        { // error: DropPoint to Curve from StartFV failed
          SetStatus(SM_FE_END_PNT_DROP_FAILURE);
          SER(SM_ERR);
        }

      // drop EndFV->Point to 3DCurve
      if (   SM_SUCCESS != p3DCurve->DropPoint(sTrimIvl,           // in : target curve allowed domain
                                               pEndFV->GetPoint(), // in : Point to drop to curve
                                               NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                   //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                   //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                               dApproxTol*10.0,    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                   //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                   //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                   //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                               NULL,               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                               bSuccess,           // out: TRUE = found a drop point
                                               dParam2,            // out: found drop curve param
                                               dDist)              // out: found drop distance
                                                                   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                   //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                   //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                   //      default:[SM_SO_MINIMIZE] to preserve original behavior
          || !bSuccess)
        { // error: DropPoint to Curve from EndFV failed
          SetStatus(SM_FE_END_PNT_DROP_FAILURE);
          SER(SM_ERR);
        }

      // trim 3DCurve and UVTrimCurves to dropPoint parameter values
      sTrimIvl.SetMinMax(dParam1,dParam2);
      SER(p3DCurve->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots
      SER(pUVCurve1->Trim(sTrimIvl)); // may snap sIvl by tol to existing knots
      SER(pUVCurve2->Trim(sTrimIvl)); // may snap sIvl by tol to existing knots

    } // end p3DCurve exists branch

  return SM_SUCCESS;

} // end SmFilletEdge::CalcFilletIntFillet

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_FILLET_X_SIDE_FACE.
    (intersection of fillet & side face)

NOTES:
  when filletEdge->3DCurve == NULL
     compute FilletSurface/EndFace->Surface intersection and set
       FilletEdge->3DCurve
       FilletEdge->PrimEU->UVTrimCurve
       FilletEdge->PrimEU->Mate->UVTrimCurve
       3DCurve->Owner
  always set
      FilletEdge->PrimaryEdgeuse->pFilletGeom1->AddSideFilletEdgeuse(pPrimEU);

***********************************************************************/
SmStatus SmFilletEdge::CalcFilletIntSideFace
  (const SmContext & crContext,         // in : context for new object construction
   SmFilletCorner * pCorner)            // in : corner continaing this filletEdge
{
  // get filletEdge->vertices
  SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
  SmFilletVertex * pEndFV   = (SmFilletVertex*)GetOtherVertex(pStartFV);

  // check state - both vertices must be processed
  if (   !pStartFV->IsProcessed()
      || !pEndFV->IsProcessed())
    {
      return SM_ERR;
    }

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  // get vertex endPoints
  SmPoint3d sStartPnt = pStartFV->GetPoint();
  SmPoint3d sEndPnt   = pEndFV->GetPoint();

  // locals: PrimEU connected to FilletSurface, MateEU connected to endFace
  SmFilletEdgeuse * pPrimEU      = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
  SmFilletEdgeuse * pMateEU      = (SmFilletEdgeuse*)pPrimEU->GetMate();
  SmFilletGeom    * pFilletGeom1 = pPrimEU->GetFilletGeom();
  NER(pFilletGeom1);
  SmFilletSolver  * pFilSolver1  = pFilletGeom1->GetFilletSolver();
  SmBSplineCurve  * p3DCurve     = SM_CAST_PTR(SmBSplineCurve,GetCurve());
  SmBSplineCurve  * pUVCurve1    = pPrimEU->GetUVTrimCurve();
  SmBSplineCurve  * pUVCurve2    = pMateEU->GetUVTrimCurve();
  SmBSplineCurve  * pNew3DCurve  = NULL;

  // get tolerance
  double dApproxTol = pFilSolver1->GetThisApproxTol3d();

  // when filletEdge does not yet have a 3DCurve
  if (p3DCurve == NULL)
    {
      SmSurface * pSurface = NULL ;
      SmObjDelete sCleanup ;

      // get FilletSurface
      // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
      // rm : SmBSplineSurface * pFilletSurface1 = pFilletGeom1->GetFilletSurface();
      SM_FILLETSURF_TYPE * pFilletSurface1 = pFilletGeom1->GetFilletSurface();
      NER(pFilletSurface1);

      // when endFace->Surface is defined by sideFace->ExtendedSurface
      if(pCorner->IsExtendedSurfacePatch())
        {
          // get endFace->Surface = sideFace->ExtendedSurface
          SmFace * pFace = GetOriginalFace(); NER(pFace);
          pSurface       =  (pCorner)
                           ? pCorner->GetExtendedSurface(pFace)
                           : NULL ;
          if (pSurface == NULL)
            {
              pSurface = pFace->GetSurface();
            }
        } // end building extendedSurface endFace patch branch
      else // not building extendedSurface endFace patch branch
        {
          // get endFace->Surface = plane through corner->OriginalVertex, edge->EndPoints
          SmPoint3d  sOrigin  = pCorner->GetFilletedVertex()->GetPoint() ;
          SmVector3d sXVector = GetVertex()->GetPoint() - sOrigin ;
          SmVector3d sYVector = GetOtherVertex(GetVertex())->GetPoint() - sOrigin ;
          double     dXLen    = sXVector.Length() ;
          double     dYLen    = sYVector.Length() ;
          SmVector2d sUVScale(1.0,1.0) ;
          sXVector /=  dXLen ;
          sYVector /=  dYLen ;
          sYVector = sXVector * sYVector * sXVector ;
          sYVector.Unitize() ;
          SmExtent2d sUVDomain(-dXLen, -dYLen, 2*dXLen, 2*dYLen) ;

          pSurface = new(crContext) SmPlane(sOrigin, sXVector, sYVector, sUVScale, sUVDomain) ;
          ((SmPlane *)pSurface)->MakeNurb() ;
          sCleanup.SetObj(pSurface) ;

        } // end not building extendedSurface endFace patch branch

      // do Surface/surface xSect
      double     dAngTol = pFilSolver1->GetThisAngTolRad();
      SmVector3d sDir    = sEndPnt - sStartPnt;
      const SmContext & sContext = m_cpCorner->GetContext();
      if(SM_SUCCESS != sm_SrfSrfIntersection( sContext,       // in : context for new object construction
                                             pStartFV,        // in : 1st point known to be on intersection curve
                                             pEndFV,          // in : 2nd point known to be on intersection curve,
                                                              //      NULL to ignore
                                             &sDir,           // in : expected general direction of intersection curve from 1st point
                                             NULL,            // in   expected intersection end direction
                                             pFilletSurface1, // in : 1st intersecting surface
                                             pSurface,        // in : 2nd intersecting surface
                                             dApproxTol,      // in : max allowed distance between xSect Curve and surfaces
                                             dAngTol,         // in : max allowed angle between consecutive xSect curve segment tangents
                                             pNew3DCurve,     // out: 3d intersection curve
                                             pUVCurve1,       // out: associated UVTrimCurve on 1st surface
                                             pUVCurve2,       // out: associated UVTrimCurve on 2nd surface
                                             FALSE,           // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                              //             resorting to expensive general surf/surf xSect solver
                                             NULL,            // in : reference point
                                             iDebugLevel)     // in : iDebugLevel, 0 = No Debug output
          || !pNew3DCurve )
        {

#ifdef SM_DEBUG_CODE
          if ( iDebugLevel > 0 )
            {
              smgfx_SetLook(1,6, 0,0,1); pStartFV->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,6, 0,1,0); pEndFV->Draw(); smgfx_SetLook(1,6, 0,1,1);
              smgfx_SetLook(1,2, 1,0,1); pFilletSurface1->DrawUV(0,0); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0.5,1,0); pSurface->DrawUV(5,5); sm_GraphicsLoop();
              sm_GraphicsLoop();
              //pFace->GetBrep()->Draw();
            }
#endif // SM_DEBUG_CODE

          SetStatus(SM_FIL_SURF_INT_FAILURE);

          pCorner->RecordFilletError(
              SM_FILERR_BAD_INT, pFilletSurface1, pSurface,
              pStartFV, pEndFV,
              _T("SmFilletEdge::CalcFilletIntSideFace(): failure to calculate intersection of fillet surface with side face")
          );

          SER(SM_ERR);
        } // end surf/surf xsect failed check

      // save the 3DCurve
      p3DCurve = pNew3DCurve;
    }

  // when a new 3DCurve was constructed by surf/surf intersection
  if (pNew3DCurve)
    {
      // save 3DCurve and UVTrimCurves, and set the owner
      SetCurve(p3DCurve, TRUE); // side effect: delete current this->UVTrimCurves
      p3DCurve->SetOwner(this);
      pPrimEU->SetUVCurve(pUVCurve1);
      pMateEU->SetUVCurve(pUVCurve2);
    }

  // connect the sideFilletEdge to the FilletSurfaceFace
  pFilletGeom1->AddSideFilletEdgeuse(pPrimEU);

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 )
    {
      pUVCurve1->Dump();
      pUVCurve2->Dump();

      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook(3,5, 0,0,1); pStartFV->Draw(); sm_GraphicsLoop();
      pEndFV->Draw(); smgfx_SetLook(3,5, 0,0,1); sm_GraphicsLoop();
      smgfx_SetLook(1,3, 0,1,1); p3DCurve->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmFilletEdge::CalcFilletIntSideFace

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_RAIL_RAIL_INTERPOLATION.

NOTES:
***********************************************************************/
SmStatus SmFilletEdge::CalcBlendingRail
  (const SmContext & crContext)
{
    SmFace * pFace = GetOriginalFace(); NER(pFace);
    SmSurface * pSurf = pFace->GetSurface(); NER(pSurf);
    double dTol = pFace->GetTolerance();
    SmExtent2d sUVDomain = pSurf->GetNaturalUVDomain();
    SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
    SmFilletVertex * pEndFV = (SmFilletVertex*)GetOtherVertex(pStartFV);

    SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
    SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
    // Calc tangent from the rail at start vertex
    SmTArray<SmEdge*> sEdges;
    pStartFV->GetEdges(sEdges);
    SmFilletEdge * pRail1 = NULL;
    for (ULONG j=0; j<sEdges.GetSize(); j++) {
        SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[j];
        if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL) {
            pRail1 = pFilletE;
            break;
        }
    }
    NER(pRail1);

    SmPoint2d sUV1;
    SmBoolean bDropStartPoint = FALSE;
    if (pStartFV->GetFilletVertexType() == SM_FV_MATE) {
        // In normal cases, pStartFV & pEndFV should all be SM_FV_MATE
        sUV1 = pStartFV->GetOriginalUV();
    }
    else {
        // Special cases, such as in 4x2 concave case
        // Drop pStartFV onto pSurf
        SmSolutionArray sSolutions;
        SER(pSurf->GlobalPointSolve(sUVDomain,SM_SO_INTERSECT,
            pStartFV->GetPoint(),dTol,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() < 1) {
            SER(pSurf->GlobalPointSolve(sUVDomain,SM_SO_INTERSECT,
                pStartFV->GetPoint(),1.0e2*SM_EFF_ZERO,NULL,SM_SR_SINGLE,sSolutions));
        }
        if (sSolutions.GetSize() < 1) {
            SetStatus(SM_FE_END_PNT_DROP_FAILURE);
            SER(SM_ERR);
        }
        sUV1 = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
        bDropStartPoint = TRUE;
    }

    SmEdgeuse   * pEU1     = pRail1->GetPrimaryEdgeuse()->GetMate();
    SmOrientType  eOrient1 = pEU1->GetOrientation();
    SmCurve     * pCurve1  = pRail1->GetCurve();
    SmExtent1d    sIvl1    = pCurve1->GetNaturalInterval();
    SmVector3d    sTangent1;
    SmVector2d    sUVTangent1;
    SmPoint3d     sPntVec1[2];
    SmBoolean     bGetStartTan1 = FALSE;
    double dParam1 = sIvl1.GetMax();
    if ((pRail1->GetVertex() != pStartFV && eOrient1 == SM_OT_SAME) ||
        (pRail1->GetVertex() == pStartFV && eOrient1 == SM_OT_OPPOSITE)) {
        bGetStartTan1 = TRUE;
        dParam1 = sIvl1.GetMin();
    }
    SER(pCurve1->Evaluate(dParam1,1,TRUE,sPntVec1));
    sTangent1 = sPntVec1[1];
    SmBSplineCurve * pUVCurve1 = pEU1->GetUVTrimCurve();
    if (pUVCurve1 && !bDropStartPoint) {
        SER(pUVCurve1->Evaluate(dParam1,1,TRUE,sPntVec1));
        sUVTangent1 = SmPoint2d(sPntVec1[1].x,sPntVec1[1].y);
    }
    else {
        SER(pSurf->DropVectors(sUV1,TRUE,TRUE,1,&sTangent1,&sUVTangent1));
    }
    if (bGetStartTan1) {
        sTangent1 = -sTangent1;
        sUVTangent1 = -sUVTangent1;
    }

    // Calc tangent from the other rail
    pEndFV->GetEdges(sEdges);
    SmFilletEdge * pRail2 = NULL;
    for (ULONG k=0; k<sEdges.GetSize(); k++) {
        SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[k];
        if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL) {
            pRail2 = pFilletE;
            break;
        }
    }
    NER(pRail2);

    SmPoint2d sUV2;
    SmBoolean bDropEndPoint = FALSE;
    if (pEndFV->GetFilletVertexType() == SM_FV_MATE) {
        sUV2 = pEndFV->GetOriginalUV();
    }
    else {
        SmSolutionArray sSolutions;
        SER(pSurf->GlobalPointSolve(sUVDomain,SM_SO_INTERSECT,
            pEndFV->GetPoint(),SM_EFF_ZERO,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() < 1) {
            SER(pSurf->GlobalPointSolve(sUVDomain,SM_SO_INTERSECT,
                pEndFV->GetPoint(),1.0e2*SM_EFF_ZERO,NULL,SM_SR_SINGLE,sSolutions));
        }
        if (sSolutions.GetSize() < 1) {
            SetStatus(SM_FE_END_PNT_DROP_FAILURE);
            SER(SM_ERR);
        }
        sUV2 = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
        bDropEndPoint = TRUE;
    }

    SmEdgeuse   * pEU2     = pRail2->GetPrimaryEdgeuse()->GetMate();
    SmOrientType  eOrient2 = pEU2->GetOrientation();
    SmCurve     * pCurve2  = pRail2->GetCurve();
    SmExtent1d    sIvl2    = pCurve2->GetNaturalInterval();
    SmVector3d    sTangent2;
    SmVector2d    sUVTangent2;
    SmPoint3d     sPntVec2[2];
    SmBoolean     bGetStartTan2 = FALSE;
    double dParam2 = sIvl2.GetMax();
    if ((pRail2->GetVertex() != pEndFV && eOrient2 == SM_OT_SAME) ||
        (pRail2->GetVertex() == pEndFV && eOrient2 == SM_OT_OPPOSITE)) {
        bGetStartTan2 = TRUE;
        dParam2 = sIvl2.GetMin();
    }
    SER(pCurve2->Evaluate(dParam2,1,TRUE,sPntVec2));
    sTangent2 = sPntVec2[1];
    SmCurve * pUVCurve2 = pEU2->GetUVTrimCurve();
    if (pUVCurve2 && !bDropEndPoint) {
        SER(pUVCurve2->Evaluate(dParam2,1,TRUE,sPntVec2));
        sUVTangent2 = SmPoint2d(sPntVec2[1].x,sPntVec2[1].y);
    }
    else {
        SER(pSurf->DropVectors(sUV2,TRUE,TRUE,1,&sTangent2,&sUVTangent2));
    }
    if (bGetStartTan2) {
        sTangent2 = -sTangent2;
        sUVTangent2 = -sUVTangent2;
    }
    double dLen1 = sTangent1.Length();
    double dLen2 = sTangent2.Length();
    SM_ASSERT(dLen1 > dTol && dLen2 > dTol);
    sTangent1 = sTangent1 / dLen1;
    sUVTangent1 = sUVTangent1 / dLen1;
    sTangent2 = sTangent2 / dLen2;
    sUVTangent2 = sUVTangent2 / dLen2;

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(1,0,0);
        pRail1->Draw();
        pRail2->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        SmPoint3d sPnt1 = pStartFV->GetPoint();
        SmPoint3d sPnt2 = pEndFV->GetPoint();
        sPnt1.Draw();
        sPnt2.Draw();
        smgfx_SetColor(0,0,1);
        sTangent1.Draw(&sPnt1);
        sTangent2.Draw(&sPnt2);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SmPoint3d  sStartPnt;
    SmVector3d sStartVec;
    SmPoint3d  sEndPnt;
    SmVector3d sEndVec;
    ULONG lCurveDim;
    if (pSurf->GetDegree(SM_SP_U) == 1 &&
        pSurf->GetDegree(SM_SP_V) == 1) {
        sStartPnt = pStartFV->GetPoint();
        sEndPnt = pEndFV->GetPoint();
        sStartVec = sTangent1;
        sEndVec = sTangent2;
        lCurveDim = 3;
    }
    else {
        sStartPnt.Set(sUV1.x, sUV1.y, 0.0);
        sStartVec.Set(sUVTangent1.x, sUVTangent1.y, 0.0) ;

        sEndPnt.Set(sUV2.x, sUV2.y, 0.0) ;
        sEndVec.Set(sUVTangent2.x, sUVTangent2.y, 0.0) ;

        lCurveDim = 2;
    }
    // Will create a conic first
    SmBSplineCurve * pCurve = NULL;
    if (sStartVec.IsColinearWith(sEndVec,sStartPnt,sEndPnt) == TRUE) {
        SER(SmBSplineCurve::CreateLineSegment(crContext,lCurveDim,
            sStartPnt,sEndPnt,pCurve));
    }
    else {
        double dT1, dT2;
        SER(smgu_LineLineClosestPoint(sStartPnt,sStartVec,sEndPnt,sEndVec,dT1,dT2));
        if (dT1*dT2 < 0.0) {
            // Can not make quadratic UV curve, create Hermite instead
            sEndVec = -sEndVec;
            // First, Adjust the lengths of direction vectors
            double dLen = sStartPnt.DistanceBetween(sEndPnt);
            dT1 = smos_Fabs(dT1);
            dT2 = smos_Fabs(dT2);
            double dSum = dT1 + dT2;
            sStartVec = sStartVec*(dLen*dT1/dSum);
            sEndVec = sEndVec*(dLen*dT2/dSum);
            SmHermiteCurve sHerm(sStartPnt,sStartVec,sEndPnt,sEndVec,lCurveDim);
            sHerm.SetContext(NULL);
            SmPoint3d sP1, sP2, sP3, sP4;
            sHerm.GetBezierPoints(sP1, sP2, sP3, sP4);
            SmPoint3d sData[4];
            SmTArray<SmPoint3d> sCntrlPoly(4,sData);
            sCntrlPoly.Add(sP1);
            sCntrlPoly.Add(sP2);
            sCntrlPoly.Add(sP3);
            sCntrlPoly.Add(sP4);
            double adKData[2];
            SmTArray<double> sKnots(2,adKData,2);
            sKnots[0] = 0.0; sKnots[1] = 1.0;
            ULONG alKMData[2];
            SmTArray<ULONG> sKnotMult(2,alKMData,2);
            sKnotMult[0] = 4; sKnotMult[1] = 4;
            SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                SM_KT_UNSPECIFIED, NULL, NULL, pCurve));
        }
        else {
            SmPoint3d sMidPnt = sStartPnt + dT1 * sStartVec;

#ifdef SM_DEBUG_CODE
            if ( DebugLevel() > 0 ) {
                SmPoint3d sPP1 = sMidPnt;
                if (lCurveDim == 2) {
                    SmPoint2d sPUV(sMidPnt.x,sMidPnt.y);
                    pSurf->EvaluatePoint(sPUV,sPP1);
                }
                smgfx_SetColor(0,1,0);
                sPP1.Draw();
                sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

            SmPoint3d sData[3];
            SmTArray<SmPoint3d> sCntrlPoly(3,sData);
            sCntrlPoly.Add(sStartPnt);
            sCntrlPoly.Add(sMidPnt);
            sCntrlPoly.Add(sEndPnt);
            double adKData[2];
            SmTArray<double> sKnots(2,adKData,2);
            sKnots[0] = 0.0; sKnots[1] = 1.0;
            ULONG alKMData[2];
            SmTArray<ULONG> sKnotMult(2,alKMData,2);
            sKnotMult[0] = 3;
            sKnotMult[1] = 3;
            double adWData[3];
            SmTArray<double> sWeights(3,adWData,3);
            sWeights[0] = sWeights[2] = 1.0;
            // Calculate the middle weight
            sWeights[1] = 1.0;
            if (smos_Fabs(dT1-dT2) < SM_EFF_ZERO_SQRT) {
                // Make circular arc
                SmVector3d sVec = sEndPnt - sStartPnt;
                double dAngle;
                SER(sVec.AngleBetween(sStartVec,dAngle));
                sWeights[1] = cos(dAngle);
                SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                    sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                    SM_KT_UNSPECIFIED, &sWeights, NULL, pCurve));
            }
            else {
                // Make parabola
                SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                    sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                    SM_KT_UNSPECIFIED, NULL, NULL, pCurve));
            }
        }
    }

    SmBSplineCurve * pNew3DCurve = NULL;
    SmBSplineCurve * pUVCurve = NULL;
    SmExtent1d sIvl = pCurve->GetNaturalInterval();
    if (lCurveDim == 2) {
        pUVCurve = pCurve;
        double dAchieved = 0.0;
        SER(pSurf->LiftCurve(crContext,sUVDomain,*pUVCurve,
            sIvl,dTol,dAchieved,pNew3DCurve));
    }
    else {
        pNew3DCurve = pCurve;
        // Project 3D curve onto the plane
        double dMaxDistToSurf = 0.0, dDeviation = 0.0;
        SmTArray<SmBSplineCurve*> sUVCurves;
        SER(pSurf->DropCurve(crContext,sUVDomain,*pNew3DCurve,
                             sIvl,dTol,
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sUVCurves));
        if (sUVCurves.GetSize() < 1)
            SER(SM_ERR);
        pUVCurve = sUVCurves[0];
    }

    SetCurve(pNew3DCurve, TRUE); // side effect: delete current this->UVTrimCurves
    pNew3DCurve->SetOwner(this);
    pMateEU->SetUVCurve(pUVCurve);

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(0,1,1);
        pNew3DCurve->DrawWithKnots();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // all done
    return SM_SUCCESS;

} // end SmFilletEdge::CalcBlendingRail

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_RAIL_RAIL_INTERPOLATION.

NOTES:
***********************************************************************/
SmStatus SmFilletEdge::CalcRailRailInterpolation
  (const SmContext & crContext,
   SmFilletCorner  * pCorner)
{
  enum SmFilletSurfaceGeneratorType eFSGType = SM_FSG_CIRCULAR; // Default
  ULONG lNumDerivatives = 1; // 0 - linear; 1, 2, or 3 for G1, G2 or G3

  // when given a corner - use its properties to pick a lNumDerivatives value
  if (pCorner)
    {
      SmTArray<SmFilletEdge*> sFilletEdges;
      pCorner->GetFilletEdges(sFilletEdges);
      ULONG lTotalEdges = sFilletEdges.GetSize();
      if (lTotalEdges <= 4)
        {
          ULONG lIndex;
          if (!sFilletEdges.FindElement(this,lIndex))
            { SER(SM_ERR) ; }

          SmFilletEdge             * pTestEdge   = sFilletEdges[(lIndex+2)%lTotalEdges];
          if (pTestEdge->GetFilletEdgeType() != SM_FE_CROSS_SECTION)
            { SER(SM_ERR) ; }

          SmFilletEdgeuse          * pPrimEU     = (SmFilletEdgeuse*)pTestEdge->GetPrimaryEdgeuse();
          SmFilletGeom             * pFilletGeom = pPrimEU->GetFilletGeom();
          NER(pFilletGeom);
          SmFilletSolver           * pFS         = pFilletGeom->GetFilletSolver();

          SmFilletSurfaceGenerator * pFSG        = pFS->GetFilletSurfaceGenerator();
          eFSGType = pFSG->GetFilletSurfaceGeneratorType();
          switch (eFSGType)
            {
              case SM_FSG_LINEAR:      lNumDerivatives = 0;
                                       break;

              case SM_FSG_BLEND_CURVE: lNumDerivatives = ((SmBlendCurveCrossSectionFSG*)pFSG)->GetContinuity();
                                       break;

              case SM_FSG_CIRCULAR:
              default:                 lNumDerivatives = 1;
                                       break;
            }
        } // end corner has 4 edges check
    } // end when given a corner check

  SmFilletVertex   * pStartFV  = (SmFilletVertex*)GetVertex();
  SmFilletVertex   * pEndFV    = (SmFilletVertex*)GetOtherVertex(pStartFV);

  SmFace           * pFace     = GetOriginalFace(); NER(pFace);
  SmBSplineSurface * pSurf     = SM_CAST_PTR(SmBSplineSurface,pFace->GetSurface());
  NER(pSurf);
  SmExtent2d         sUVDomain = pSurf->GetNaturalUVDomain();
  double             dTol      = pFace->GetTolerance();

  // Determine if we want to work on 2D or 3D
  SmBoolean bProjectAndLift = FALSE; // TRUE if we have non-analytic rails without

  // uv curves. Very rare case
  SmBoolean bCreateCrvOn3D = FALSE;
  SmPlane *pPlane = NULL;
  if (SmPlane::IsNurbSurfacePlane(crContext,pSurf,pPlane))
    {
      delete pPlane; pPlane = NULL ;
      bCreateCrvOn3D = TRUE;
    }
  else if (   pStartFV->GetFilletVertexType() != SM_FV_MATE
           || pEndFV->GetFilletVertexType()   != SM_FV_MATE)
    {
      // Special 4x2 concave cases
      bCreateCrvOn3D = TRUE;
    }

  // Get "incoming" rail-curve parameter at start vertex
  SmTArray<SmEdge*> sEdges;
  pStartFV->GetEdges(sEdges);
  SmFilletEdge * pRail1 = NULL;
  for (ULONG j=0; j<sEdges.GetSize(); j++)
    {
      SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[j];
      if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL)
        {
          pRail1 = pFilletE;
          break;
        }
    }
  NER(pRail1);
  SmEdgeuse * pEU1    = pRail1->GetPrimaryEdgeuse()->GetMate();
  SmCurve   * pCurve1 = pEU1->GetUVTrimCurve();

  if (bCreateCrvOn3D==FALSE && pCurve1==NULL)
    {
      bCreateCrvOn3D = TRUE;
      bProjectAndLift = TRUE;
    }

  SmPoint3d sStartPnt;
  if (bCreateCrvOn3D)
    {
      // Process 3D curve
      pCurve1   = pRail1->GetCurve();
      sStartPnt = pStartFV->GetPoint();
    }
  else
    {
      // Process 2D curve. Normally, pStartFV & pEndFV should all be SM_FV_MATE
      sStartPnt = SmPoint3d(pStartFV->GetOriginalUV());
    }

  double dDroppingTol = dTol;
  if (!bCreateCrvOn3D)
    {
      dDroppingTol = (sUVDomain.GetSize().GetMaxDimension() + 1.0) * SM_EFF_ZERO;
    }

  double dParam1 = SM_BIG_DOUBLE;
  SmBoolean bSuccess = FALSE;

  for (double dScale=1.0; dScale<100.1; dScale=dScale*10.0)
    {
      double dDist;
      SER(pCurve1->DropPoint(pCurve1->GetNaturalInterval(), // in : target curve allowed domain
                             sStartPnt,                     // in : Point to drop to curve
                             NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                             dScale*dDroppingTol,           // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                             NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                             bSuccess,                      // out: TRUE = found a drop point
                             dParam1,                       // out: found drop curve param
                             dDist)) ;                      // out: found drop distance
                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
      if (bSuccess) break;
    }

  if (!bSuccess) /*dParam1 == SM_BIG_DOUBLE*/
    {
      SER(SM_ERR);
    }

  SmOrientType eOrient1     = pEU1->GetOrientation();
  SmBoolean    bSameOrient1 = TRUE;
  if (   (pRail1->GetVertex() != pStartFV && eOrient1 == SM_OT_SAME)
      || (pRail1->GetVertex() == pStartFV && eOrient1 == SM_OT_OPPOSITE))
    {
      bSameOrient1 = FALSE;
    }

  // Get "incoming" rail-curve parameter at end vertex
  pEndFV->GetEdges(sEdges);
  SmFilletEdge * pRail2 = NULL;
  for (ULONG k=0; k<sEdges.GetSize(); k++)
    {
      SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[k];
      if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL)
        {
          pRail2 = pFilletE;
          break;
        }
    }

  NER(pRail2);
  SmEdgeuse * pEU2    = pRail2->GetPrimaryEdgeuse()->GetMate();
  SmCurve   * pCurve2 = pEU2->GetUVTrimCurve();

  if (bCreateCrvOn3D==FALSE && pCurve2==NULL)
    {
      bCreateCrvOn3D = TRUE;
      bProjectAndLift = TRUE;
    }

  SmPoint3d sEndPnt;
  if (bCreateCrvOn3D)
    {
      // Process 3D curve
      pCurve2   = pRail2->GetCurve();
      sStartPnt = pStartFV->GetPoint();
      sEndPnt   = pEndFV->GetPoint();
    }
  else
    {
      // Process 2D curve. Normally, pStartFV & pEndFV should all be SM_FV_MATE
      sEndPnt = SmPoint3d(pEndFV->GetOriginalUV());
    }

  double dParam2 = SM_BIG_DOUBLE;
  for (double dScale2=1.0; dScale2<100.1; dScale2=dScale2*10.0)
    {
      double dDist;
      SER(pCurve2->DropPoint(pCurve2->GetNaturalInterval(), // in : target curve allowed domain
                             sEndPnt,                       // in : Point to drop to curve
                             NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                             dScale2*dDroppingTol,          // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                             NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                             bSuccess,                      // out: TRUE = found a drop point
                             dParam2,                       // out: found drop curve param
                             dDist)) ;                      // out: found drop distance
                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
      if (bSuccess) break;
    }
  if (!bSuccess) /*dParam2 == SM_BIG_DOUBLE*/
    {
      SER(SM_ERR);
    }
  SmOrientType eOrient2     = pEU2->GetOrientation();
  SmBoolean    bSameOrient2 = FALSE;
  if (   (pRail2->GetVertex() != pEndFV && eOrient2 == SM_OT_SAME)
      || (pRail2->GetVertex() == pEndFV && eOrient2 == SM_OT_OPPOSITE))
    {
      bSameOrient2 = TRUE;
    }

  // Compute tangents and higher order derivatives if necessary
  // lNumDerivatives:
  //   1: Circular & G1 - Get first derivative and point
  //   2: G2 - produces 2nd derivative, 1st derivative and point.
  //   3: G3 - produces 3rd derivative and lower derivatives

  // Calc tangent from the rail at start vertex
  SmPoint3d sPntVec1[4];
  SER(pCurve1->Evaluate(dParam1,lNumDerivatives,TRUE,sPntVec1));
  if (!bSameOrient1)
    {
      for (ULONG ii=1; ii<=lNumDerivatives; ii++)
        {
          sPntVec1[ii] = -sPntVec1[ii];
        }
    }

  // Calc tangent from the other rail
  SmPoint3d sPntVec2[4];
  SER(pCurve2->Evaluate(dParam2,lNumDerivatives,TRUE,sPntVec2));
  if (!bSameOrient2)
    {
      for (ULONG ii=1; ii<=lNumDerivatives; ii++)
        {
          sPntVec2[ii] = -sPntVec2[ii];
        }
    }

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
  iDebugLevel = DebugLevel();
  if ( iDebugLevel > 0 )
    {
      if (FALSE)
        { smgfx_SetColor(1,0,0); pRail1->Draw(); sm_GraphicsLoop();
          smgfx_SetColor(1,0,0); pRail2->Draw(); sm_GraphicsLoop();
        }
      smgfx_SetColor(0,0,1); sStartPnt.Draw(); sm_GraphicsLoop();
      smgfx_SetColor(0,0,1); sEndPnt.Draw(); sm_GraphicsLoop();
      if (lNumDerivatives > 0)
        { smgfx_SetColor(0,0,1); sPntVec1[1].Draw(&sStartPnt); sm_GraphicsLoop();
          smgfx_SetColor(0,0,1); sPntVec2[1].Draw(&sEndPnt); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  SmBSplineCurve * pNewCurve = NULL;
  if (eFSGType == SM_FSG_LINEAR)
    {
      // Linear cases
      SER(SmBSplineCurve::CreateLineSegment(crContext, 3/*dim*/,
                                            sStartPnt,
                                            sEndPnt,
                                            pNewCurve));
    }
  else if (TRUE/*eFSGType == SM_FSG_CIRCULAR*/)
    {
      // In G1 cases, we'll create 3d curve and then
      // drop it onto the surface
      SmVector3d sStartVec = sPntVec1[1];
      SmVector3d sEndVec = sPntVec2[1];
      ULONG lDimension = 3;
      SER(sm_InterpolateEndptsAndTangents(crContext,
                                          sStartPnt,
                                          sEndPnt,
                                          sStartVec,
                                          sEndVec,
                                          lDimension,
                                          pNewCurve,
                                          iDebugLevel ));
    }
  else
    {
      // G1, G2 & G3 cases
      // Fair the 3d points & tangents
      SmTArray<SmPoint3d> sPoints;
      sPoints.Add(sPntVec1[0]);
      sPoints.Add(sPntVec2[0]);
      SmTArray<SmVector3d> sTangents;
      sTangents.Add(sPntVec1[1]);
      sTangents.Add(sPntVec2[1]);
      SmTArray<SmVector3d> sHighDerivs;
      for (ULONG k=2; k<=lNumDerivatives; k++)
        {
          sHighDerivs.Add(sPntVec1[k]);
          sHighDerivs.Add(sPntVec2[k]);
        }
      SmBoolean bPreventInflection = TRUE;
      SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,
          sTangents,&sHighDerivs,bPreventInflection));
      // Create 3d curve
      ULONG lDegree = 3;
      if (lNumDerivatives == 2)
        {
          lDegree = 5;
        }
      else if (lNumDerivatives == 3)
        {
          lDegree = 7;
        }
      SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,
                                                   SM_CP_UNIFORM,3,lDegree,
                                                   sPoints,sTangents,
                                                  &sHighDerivs,TRUE,
                                                   pNewCurve));
    }

  SmBSplineCurve * p3DCurve = NULL;
  SmBSplineCurve * pUVCurve = NULL;
  if (bCreateCrvOn3D)
    {
      p3DCurve = pNewCurve;
      // Compute UV-curve by projecting 3d curve onto the surface
      SmTArray<SmBSplineCurve*> sUVCurves;
      double dMaxDistToSurf = 0.0;
      double dDeviation;
      SER(pSurf->DropCurve(crContext,sUVDomain,*p3DCurve,
                           p3DCurve->GetNaturalInterval(),dTol,
                           dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                           dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                           sUVCurves));
      SmObjsDelete<SmBSplineCurve*> sDelCurves(&sUVCurves);
      // Will replace the original UV-curve when we have only one result
      if (sUVCurves.GetSize() == 1)
        {
          sDelCurves.Clear();
          pUVCurve = sUVCurves[0];
        }
      else
        {
          SER(SM_ERR);
        }
    }
  else
    {
      pUVCurve = pNewCurve;
      pUVCurve->ConvertTo2D() ; // gwc: this changes z=0 to z=NL_NOZ values.  Is that a problem?
    }

  if (bCreateCrvOn3D == FALSE || bProjectAndLift == TRUE)
    {
      // Lift the 3d curve
      double dAchieved;
      SER(pSurf->LiftCurve(crContext,sUVDomain,*pUVCurve,
                           pUVCurve->GetNaturalInterval(),
                           dTol,
                           dAchieved,
                           p3DCurve));

      // Reparametrize the curve with arc-length for non-blend type
      if (eFSGType != SM_FSG_BLEND_CURVE)
        {
          SER(p3DCurve->ReparametrizeWithArcLength());
          SM_ASSERT(pUVCurve != NULL) ; delete pUVCurve ; pUVCurve = NULL ;
          // Drop the 3d curve again to get the UV-curve
          // Reproject 3d curve back onto the surface
          SmTArray<SmBSplineCurve*> sUVCurves;
          double dMaxDistToSurf = 0.0;
          double dDeviation;
          SER(pSurf->DropCurve(crContext,sUVDomain,*p3DCurve,
                               p3DCurve->GetNaturalInterval(),dTol,
                               dMaxDistToSurf, // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                               dDeviation,     // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                               sUVCurves));
          SmObjsDelete<SmBSplineCurve*> sDelCurves(&sUVCurves);
          if (sUVCurves.GetSize() == 1)
            {
              sDelCurves.Clear();
              pUVCurve = sUVCurves[0];
            }
          else
            {
              SER(SM_ERR);
            }
        } // end if (eFSGType != SM_FSG_BLEND_CURVE)
    } // end if (bCreateCrvOn3D == FALSE || bProjectAndLift == TRUE)

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 )
    {
      if (0)
        {
          SmBoolean bIsFilletCrossSection;
          ULONG lCrossSection;
          double dDistance, dRadius;
          double dBlendScale = 1.0;
          SER(p3DCurve->TestForFilletCrossSection(crContext,dTol,
              bIsFilletCrossSection,lCrossSection,dDistance,dRadius,dBlendScale));
        }
      smgfx_SetColor(1,0,0);
      p3DCurve->DrawWithKnots();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  if (pCorner && pCorner->GetCornerType() == SM_FCR_N_x_N_CONCAVE)
    {
      // Test and see if this edge intersect with the original corner
      SmFilletConcaveNxNCorner * pConcaveNxNCorner = (SmFilletConcaveNxNCorner*)pCorner;
      if (pConcaveNxNCorner->TestRailRailInterpolationCurve(p3DCurve))
        {
          // Corner vertex is on the opposite side of the curve
          SmTArray<SmFilletGeom*> sGeoms;
          pStartFV->GetFilletGeoms(sGeoms);
          if (sGeoms.GetSize() != 1) SER(SM_ERR);
          SmFilletSolver * pFilSolver1 = sGeoms[0]->GetFilletSolver();
          sGeoms.ReSet();// Need to reset it!!!
          pEndFV->GetFilletGeoms(sGeoms);
          if (sGeoms.GetSize() != 1) SER(SM_ERR);
          SmFilletSolver * pFilSolver2 = sGeoms[0]->GetFilletSolver();

          // Re-compute UV-curve
          SmExtent1d sIvl = pUVCurve->GetNaturalInterval();
          SmPoint3d  sPntVec[2];
          SER(pUVCurve->Evaluate(sIvl.GetMin(),1,TRUE,sPntVec));
          SmPoint3d  sStartUVPnt = sPntVec[0];
          SmVector3d sStartUVVec = sPntVec[1];
          SER(pUVCurve->Evaluate(sIvl.GetMax(),1,TRUE,sPntVec));
          SmPoint3d sEndUVPnt = sPntVec[0];
          SmVector3d sEndUVVec = -sPntVec[1];
          SM_ASSERT(pUVCurve != NULL) ; delete pUVCurve ; pUVCurve = NULL ;
          const SmVertex * cpVertex = pCorner->GetFilletedVertex();

          SER(sm_CreateCubicRailRailInterpolant(crContext,
                                                pFilSolver1,pFilSolver2,
                                                pFace,cpVertex,
                                                sStartUVPnt,sStartUVVec,
                                                sEndUVPnt,sEndUVVec,
                                                0.3,
                                                pUVCurve,
                                                iDebugLevel ));
          // Lift the UV-curve
          SM_ASSERT(p3DCurve != NULL) ; delete p3DCurve ; p3DCurve = NULL ;
          p3DCurve = NULL;
          double dAchieved;
          SER(pSurf->LiftCurve(crContext,sUVDomain,*pUVCurve,
                               pUVCurve->GetNaturalInterval(),
                               dTol,
                               dAchieved,
                               p3DCurve));
          SetStatus(SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE);
        }
    }

  // set p3DCurve as this FilletEdge's geometry
  SetCurve(p3DCurve, TRUE); // side effect: delete current this->UVTrimCurves
  p3DCurve->SetOwner(this);
  SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)GetPrimaryEdgeuse()->GetMate();
  pMateEU->SetUVCurve(pUVCurve);

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 )
    {
      smgfx_Erase();
      smgfx_SetColor(0,1,1);
      p3DCurve->DrawWithKnots();
      p3DCurve->Dump();
      smgfx_SetColor(0,0,0);
      pRail1->Draw();
      pRail2->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  return SM_SUCCESS;

} // end SmFilletEdge::CalcRailRailInterpolation

#if 0 // begin obsolete code block
/*******************************************************************//**
PURPOSE:

NOTES:    GWC: this is an obsolete code block which was rewritten
               as above method
***********************************************************************/
SmStatus SmFilletEdge::CalcRailRailInterpolation
  (const SmContext & crContext,
   SmFilletCorner * pCorner)
{
    SmFace * pFace = GetOriginalFace(); NER(pFace);
    SmSurface * pSurf = pFace->GetSurface(); NER(pSurf);
    double dTol = pFace->GetTolerance();
    SmExtent2d sUVDomain = pSurf->GetNaturalUVDomain();
    SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
    SmFilletVertex * pEndFV = (SmFilletVertex*)GetOtherVertex(pStartFV);
    SmPoint3d sStartPnt = pStartFV->GetPoint();
    SmPoint3d sEndPnt = pEndFV->GetPoint();

    SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
    SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();

    SmBoolean bSuccess;
    SmPoint3d sPntVec[2];
    // Calc tangent from the rail at start vertex
    SmTArray<SmEdge*> sEdges;
    pStartFV->GetEdges(sEdges);
    SmFilletEdge * pRail1 = NULL;
    for (ULONG j=0; j<sEdges.GetSize(); j++) {
        SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[j];
        if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL) {
            pRail1 = pFilletE;
            break;
        }
    }
    NER(pRail1);
    SmCurve * pCurve1 = pRail1->GetCurve();
    double dParam1;
    for (double dScale=1.0; dScale<100.1; dScale=dScale*10.0) {
        double dDist;
        SER(pCurve1->DropPoint(pCurve1->GetNaturalInterval(),  // in : target curve allowed domain
                               sStartPnt,                      // in : Point to drop to curve
                               NULL,                           // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                               //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                               //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dScale*dTol,                    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                               //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                               //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                               //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               NULL,                           // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               bSuccess,                       // out: TRUE = found a drop point
                               dParam1,                        // out: found drop curve param
                               dDist)) ;                       // out: found drop distance
                                                               // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                               //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                               //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                               //      default:[SM_SO_MINIMIZE] to preserve original behavior
        if (bSuccess) break;
    }
    if (!bSuccess) SER(SM_ERR);
    SER(pCurve1->Evaluate(dParam1,1,TRUE,sPntVec));
    SmVector3d    sStartVec = sPntVec[1];
    SmEdgeuse   * pEU1      = pRail1->GetPrimaryEdgeuse()->GetMate();
    SmOrientType  eOrient1  = pEU1->GetOrientation();
    if (pRail1->GetVertex() != pStartFV && eOrient1 == SM_OT_SAME ||
        pRail1->GetVertex() == pStartFV && eOrient1 == SM_OT_OPPOSITE) {
        sStartVec = -sStartVec;
    }

    // Calc tangent from the other rail
    pEndFV->GetEdges(sEdges);
    SmFilletEdge * pRail2 = NULL;
    for (ULONG k=0; k<sEdges.GetSize(); k++) {
        SmFilletEdge * pFilletE = (SmFilletEdge*)sEdges[k];
        if (pFilletE->GetFilletEdgeType() == SM_FE_RAIL) {
            pRail2 = pFilletE;
            break;
        }
    }
    NER(pRail2);
    SmCurve * pCurve2 = pRail2->GetCurve();
    double dParam2;
    for (double dScale2=1.0; dScale2<100.1; dScale2=dScale2*10.0) {
        double dDist;
        SER(pCurve2->DropPoint(pCurve2->GetNaturalInterval(), // in : target curve allowed domain
                               sEndPnt,                       // in : Point to drop to curve
                               NULL,                          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                              //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                              //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dScale2*dTol,                  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                              //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                              //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                              //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               NULL,                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               bSuccess,                      // out: TRUE = found a drop point
                               dParam2,                       // out: found drop curve param
                               dDist)) ;                      // out: found drop distance
                                                              // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                              //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                              //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                              //      default:[SM_SO_MINIMIZE] to preserve original behavior
        if (bSuccess) break;
    }
    if (!bSuccess) SER(SM_ERR);
    SER(pCurve2->Evaluate(dParam2,1,TRUE,sPntVec));
    SmVector3d    sEndVec  = sPntVec[1];
    SmEdgeuse   * pEU2     = pRail2->GetPrimaryEdgeuse()->GetMate();
    SmOrientType  eOrient2 = pEU2->GetOrientation();
    if (pRail2->GetVertex() != pEndFV && eOrient2 == SM_OT_SAME ||
        pRail2->GetVertex() == pEndFV && eOrient2 == SM_OT_OPPOSITE) {
        sEndVec = -sEndVec;
    }

    double dLen1 = sStartVec.Length();
    double dLen2 = sEndVec.Length();
    SM_ASSERT(dLen1 > dTol && dLen2 > dTol);
    // Unitize tangents
    sStartVec = sStartVec / dLen1;
    sEndVec = sEndVec / dLen2;

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        if (0) {
            smgfx_SetColor(1,0,0);
            pRail1->Draw();
            pRail2->Draw();
            sm_GraphicsLoop();
        }
        smgfx_SetColor(0,0,1);
        SmPoint3d sPnt1 = pStartFV->GetPoint();
        SmPoint3d sPnt2 = pEndFV->GetPoint();
        sPnt1.Draw();
        sPnt2.Draw();
        smgfx_SetColor(0,0,1);
        sStartVec.Draw(&sStartPnt);
        sEndVec.Draw(&sEndPnt);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SmBSplineCurve * pCurve = NULL;
    ULONG lCurveDim = 3;
    if (sStartVec.IsColinearWith(sEndVec,sStartPnt,sEndPnt) == TRUE) {
        // Will create a line first
        SER(SmBSplineCurve::CreateLineSegment(crContext,lCurveDim,
            sStartPnt,sEndPnt,pCurve));
    }
    else {
        // Will attempt to make a conic
        double dT1, dT2;
        SER(smgu_LineLineClosestPoint(sStartPnt,sStartVec,sEndPnt,sEndVec,dT1,dT2));
        if (dT1*dT2 < 0.0) {
            // Can not make quadratic UV curve, create Hermite instead
            sEndVec = -sEndVec;
            // First, Adjust the lengths of direction vectors
            double dLen = sStartPnt.DistanceBetween(sEndPnt);
            dT1 = smos_Fabs(dT1);
            dT2 = smos_Fabs(dT2);
            double dSum = dT1 + dT2;
            sStartVec = sStartVec*(dLen*dT1/dSum);
            sEndVec = sEndVec*(dLen*dT2/dSum);
            SmHermiteCurve sHerm(sStartPnt,sStartVec,sEndPnt,sEndVec,lCurveDim);
            sHerm.SetContext(NULL);
            SmPoint3d sP1, sP2, sP3, sP4;
            sHerm.GetBezierPoints(sP1, sP2, sP3, sP4);
            SmPoint3d sData[4];
            SmTArray<SmPoint3d> sCntrlPoly(4,sData);
            sCntrlPoly.Add(sP1);
            sCntrlPoly.Add(sP2);
            sCntrlPoly.Add(sP3);
            sCntrlPoly.Add(sP4);
            double adKData[2];
            SmTArray<double> sKnots(2,adKData,2);
            sKnots[0] = 0.0; sKnots[1] = 1.0;
            ULONG alKMData[2];
            SmTArray<ULONG> sKnotMult(2,alKMData,2);
            sKnotMult[0] = 4; sKnotMult[1] = 4;
            SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                SM_KT_UNSPECIFIED, NULL, NULL, pCurve));
        }
        else {
            // Make conic
            SmPoint3d sMidPnt = sStartPnt + dT1 * sStartVec;
#ifdef SM_DEBUG_CODE
            if ( DebugLevel() > 0 ) {
                SmPoint3d sPP1 = sMidPnt;
                if (lCurveDim == 2) {
                    SmPoint2d sPUV(sMidPnt.x,sMidPnt.y);
                    pSurf->EvaluatePoint(sPUV,sPP1);
                }
                smgfx_SetColor(0,1,0);
                sPP1.Draw();
                sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
            SmPoint3d sData[3];
            SmTArray<SmPoint3d> sCntrlPoly(3,sData);
            sCntrlPoly.Add(sStartPnt);
            sCntrlPoly.Add(sMidPnt);
            sCntrlPoly.Add(sEndPnt);
            double adKData[2];
            SmTArray<double> sKnots(2,adKData,2);
            sKnots[0] = 0.0; sKnots[1] = 1.0;
            ULONG alKMData[2];
            SmTArray<ULONG> sKnotMult(2,alKMData,2);
            sKnotMult[0] = 3;
            sKnotMult[1] = 3;
            double adWData[3];
            SmTArray<double> sWeights(3,adWData,3);
            sWeights[0] = sWeights[2] = 1.0;
            // Calculate the middle weight
            sWeights[1] = 1.0;
            if (smos_Fabs(dT1-dT2) < SM_EFF_ZERO_SQRT) {
                // Make circular arc
                SmVector3d sVec = sEndPnt - sStartPnt;
                double dAngle;
                SER(sVec.AngleBetween(sStartVec,dAngle));
                sWeights[1] = cos(dAngle);
                SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                    sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                    SM_KT_UNSPECIFIED, &sWeights, NULL, pCurve));
            }
            else {
                // Make parabola
                SER(SmBSplineCurve::CreateCanonical(crContext,lCurveDim,2,
                    sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots,
                    SM_KT_UNSPECIFIED, NULL, NULL, pCurve));
            }
        }
    }
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(1,0,0);
        pCurve->DrawWithKnots();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Compute UV-curve
    SmTArray<SmBSplineCurve*> sUVCurves;
    double dMaxDistToSurf = 0.0;
    double dDeviation;
    // Project 3d curve onto the surface
    SER(pSurf->DropCurve(crContext,sUVDomain,*pCurve,
                         pCurve->GetNaturalInterval(),dTol,
                         dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                         dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                         sUVCurves));
    SM_ASSERT(sUVCurves.GetSize() == 1);
    if ( sUVCurves.GetSize() < 1 )
      { SER( SM_ERR ); }
    SmBSplineCurve * pUVCurve = sUVCurves[0];
    SmBSplineCurve * pNew3DCurve = NULL;
    double dAchieved;
    SER(pSurf->LiftCurve(crContext,sUVDomain,*pUVCurve,
        pUVCurve->GetNaturalInterval(),dTol,dAchieved,pNew3DCurve));
    // Reparametrize the curve with arc-length if the curve is not linear
    SER(pNew3DCurve->ReparametrizeWithArcLength());
    SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
    pCurve = pNew3DCurve;
    SM_ASSERT(pUVCurve != NULL) ; delete pUVCurve ; pUVCurve = NULL ;
    // Drop the 3d curve again to get the UV-curve
    SmExtent1d sIvl = pCurve->GetNaturalInterval();
    // Reproject 3d curve back onto the surface
    SER(pSurf->DropCurve(crContext,sUVDomain,*pCurve,
                         sIvl,dTol,
                         dMaxDistToSurf,    // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                         dDeviation,        // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                         sUVCurves));
    SM_ASSERT(sUVCurves.GetSize() == 1);
    if ( sUVCurves.GetSize() < 1 )
      { SER( SM_ERR ); }
    pUVCurve = sUVCurves[0];

    if (pCorner && pCorner->GetCornerType() == SM_FCR_N_x_N_CONCAVE) {
        // Test and see if this edge intersect with the original corner
        const SmVertex * cpVertex = pCorner->GetFilletedVertex();
        // Check to see if the original corner 'intersects' the curve
        // Drop corner vertex onto pCurve
        SmPoint3d sCornerPnt = cpVertex->GetPoint();
        SmSolution sSData[8];
        SmSolutionArray sSolutions(8,sSData);
        SER(pCurve->GlobalPointSolve(sIvl,SM_SO_MINIMIZE,sCornerPnt,
                                     dTol,NULL,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() != 1) SER(SM_ERR);
        double dT = sSolutions[0].m_vStart[0];
        // Find the 2nd deriv. of the curve at dT
        SmVector3d sGeomVec[4];
        SER(pCurve->EvaluateGeometric(dT,2,TRUE,sGeomVec));
        SmVector3d sV = sCornerPnt - sGeomVec[0];
        SER(sGeomVec[2].Unitize());
        SER(sV.Unitize());
        if (sV.Dot(sGeomVec[2]) < 0.0) {
            // Corner vertex is on the opposite side of the curve
#ifdef SM_DEBUG_CODE
            if ( DebugLevel() > 0 ) {
                smgfx_SetColor(0,1,1);
                pCurve->DrawWithKnots();
                smgfx_SetColor(1,0,0);
                sGeomVec[0].Draw();
                sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
            SmTArray<SmFilletGeom*> sGeoms;
            pStartFV->GetFilletGeoms(sGeoms);
            if (sGeoms.GetSize() != 1) SER(SM_ERR);
            SmFilletSolver * pFilSolver1 = sGeoms[0]->GetFilletSolver();
            sGeoms.ReSet();// Need to reset it!!!
            pEndFV->GetFilletGeoms(sGeoms);
            if (sGeoms.GetSize() != 1) SER(SM_ERR);
            SmFilletSolver * pFilSolver2 = sGeoms[0]->GetFilletSolver();
            // Re-compute UV-curve
            SER(pUVCurve->Evaluate(sIvl.GetMin(),1,TRUE,sPntVec));
            SmPoint3d sStartUVPnt = sPntVec[0];
            SmVector3d sStartUVVec = sPntVec[1];
            SER(pUVCurve->Evaluate(sIvl.GetMax(),1,TRUE,sPntVec));
            SmPoint3d sEndUVPnt = sPntVec[0];
            SmVector3d sEndUVVec = -sPntVec[1];
            SM_ASSERT(pUVCurve != NULL) ; delete pUVCurve ; pUVCurve = NULL ;
            SER(sm_CreateCubicRailRailInterpolant(crContext,
                                                  pFilSolver1,pFilSolver2,pFace,cpVertex,
                                                  sStartUVPnt,sStartUVVec,
                                                  sEndUVPnt,sEndUVVec,
                                                  0.3,pUVCurve,
                                                  DebugLevel() ));
            // Lift the UV-curve
            SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            pCurve = NULL;
            double dAchieved;
            SER(pSurf->LiftCurve(crContext,sUVDomain,*pUVCurve,
                                 pUVCurve->GetNaturalInterval(),dTol,dAchieved,pCurve));
            SetStatus(SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE);
        }
    }

    SetCurve(pCurve, TRUE);
    pCurve->SetOwner(this);
    pMateEU->SetUVCurve(pUVCurve);

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(0,1,1);
        pCurve->DrawWithKnots();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    return SM_SUCCESS;

} // end SmFilletEdge::CalcRailRailInterpolation
#endif  // 0  - end obsolete code block

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_ON_EDGE.

NOTES: side effects: thisFilletEdge->SetCurve(ThisFilletEdge->OrigCurveCopy);
                     pNew3DCurve->SetOwner(this);

  where pNew3DCurve = copy of OriginalEdge() trimmed to start and end vertices.

  thisFilletEdge has no UVTrimCurves on exit
***********************************************************************/
SmStatus SmFilletEdge::CalcEdgeOnEdge
  (const SmContext & crContext,  // in : Contex for new object construction
   SmFilletCorner  * pCorner)    // NotUsed: in : Not used in this method, default:[NULL]
{
  SM_REF1(pCorner) ;
  // get filletEdge vertices
  SmFilletVertex * pStartFV    = (SmFilletVertex*)GetVertex();
  SmFilletVertex * pEndFV      = (SmFilletVertex*)GetOtherVertex(pStartFV);

  SmEdge         * pEdge       = GetOriginalEdge();

  // GWC: Stop Fillets from switching SmCurve objects to SmBSplineCurve objs
  SmBSplineCurve * pNew3DCurve = NULL ;
  SmCurve        * pCopyCurve  = NULL ;
  pEdge->GetCurve()->Copy(crContext, pCopyCurve) ; 
  pNew3DCurve = SM_CAST_PTR(SmBSplineCurve, pCopyCurve) ; NER(pNew3DCurve);
  //  
  //  SmBSplineCurve * pCurve      = SM_CAST_PTR(SmBSplineCurve, pEdge->GetCurve()); NER(pCurve);
  //  SmBSplineCurve * pNew3DCurve = new (crContext)SmBSplineCurve(*pCurve);

  // Trim ThisFilletEdge->OrigCurveCopy by Start and End FilletPoints
  SmExtent1d sTrimIvl = pEdge->GetInterval();  // was  pNew3DCurve->GetNaturalInterval();  [220629]
  SmBoolean  bSuccess;
  double     dParam1, dParam2, dDist;
  double     dTol = pEdge->GetTolerance();

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 )
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); if(GetOriginalBrep()) GetOriginalBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1); if(GetOriginalEdge()) GetOriginalEdge()->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0); if(GetOriginalFace()) GetOriginalFace()->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); pNew3DCurve->DrawWDeriv(pNew3DCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 0,0,1); pStartFV->GetPoint().Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 0,1,0); pEndFV->GetPoint().Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // drop startVertex to thisFilletEdge->OrigCurveCopy
  if (   SM_SUCCESS != pNew3DCurve->DropPoint(sTrimIvl,                   // in : target curve allowed domain
                                              pStartFV->GetPoint(),       // in : Point to drop to curve
                                              NULL,                       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                          //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                          //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                              dTol,                       // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                          //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                          //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                          //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                              NULL,  /* was 10* [B448] */ // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                              bSuccess,                   // out: TRUE = found a drop point
                                              dParam1,                    // out: found drop curve param
                                              dDist)                      // out: found drop distance
                                                                          // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                          //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                          //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                          //      default:[SM_SO_MINIMIZE] to preserve original behavior
      || !bSuccess)                                                       
    {
      SetStatus(SM_FE_END_PNT_DROP_FAILURE);
      SER(SM_ERR);
    }

  // drop endVertex to thisFilletEdge->OrigCurveCopy
  if (   SM_SUCCESS != pNew3DCurve->DropPoint(sTrimIvl,                    // in : target curve allowed domain
                                              pEndFV->GetPoint(),          // in : Point to drop to curve
                                              NULL,                        // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                           //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                           //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                              dTol,  /* was 10 * [B448] */ // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance. 
                                                                           //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                           //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                           //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                              NULL,                        // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                              bSuccess,                    // out: TRUE = found a drop point
                                              dParam2,                     // out: found drop curve param
                                              dDist)                       // out: found drop distance
                                                                           // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                           //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                           //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                           //      default:[SM_SO_MINIMIZE] to preserve original behavior
     ||  !bSuccess)
    {
      SetStatus(SM_FE_END_PNT_DROP_FAILURE);
      SER(SM_ERR);
    }

  // set 3DCurve->interval = ordered[startDropparameter, endDropParameter]
  if (dParam1 > dParam2) { sTrimIvl.SetMinMax(dParam2,dParam1); }
  else                   { sTrimIvl.SetMinMax(dParam1,dParam2); }
  SER(pNew3DCurve->Trim(sTrimIvl));   // may snap sIvl by tol to existing knots

  // reverse curve if needed
  if (dParam1 > dParam2)
    {
      SER(pNew3DCurve->ReverseParameterization(sTrimIvl,sTrimIvl));
    }

  // set Curve and Curve->Owner pointers
  SetCurve(pNew3DCurve, TRUE); // side effect: delete current this->UVTrimCurves
  pNew3DCurve->SetOwner(this);

  // all done
  return SM_SUCCESS;

} // end SmFilletEdge::CalcEdgeOnEdge

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_ON_EXTEND_EDGE.

NOTES:
***********************************************************************/
SmStatus SmFilletEdge::CalcEdgeOnExtendedEdge
  (const SmContext & crContext,
   SmFilletCorner  * pOptCorner)
{
    NER(pOptCorner);
    SmFilletVertex * pStartFV = (SmFilletVertex*)GetVertex();
    SmFilletVertex * pEndFV   = (SmFilletVertex*)GetOtherVertex(pStartFV);
    if (!pStartFV->IsProcessed() || !pEndFV->IsProcessed()) {
        SER(SM_ERR);
    }

    // Edge need to be extended by recomputing the SSI
    // Should have extended side-face now
    SmEdge * pEdge = GetOriginalEdge();
    SmTArray<SmFace*> sFaces;
    pEdge->GetFaces(sFaces);
    if (sFaces.GetSize() < 2) SER(SM_ERR);

    SmBSplineSurface * pSurface1 = NULL;
    SmBSplineSurface * pSurface2 = NULL;
    for (ULONG i=0; i<2; i++) {
        SmSurface * pSurf = pOptCorner->GetExtendedSurface(sFaces[i]);
        if (pSurf == NULL) {
            pSurf = sFaces[i]->GetSurface();
        }
        if (pSurface1 == NULL) {
            pSurface1 = SM_CAST_PTR(SmBSplineSurface,pSurf);
        }
        else {
            pSurface2 = SM_CAST_PTR(SmBSplineSurface,pSurf);
        }
    }

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        iDebugLevel = DebugLevel();
        smgfx_SetColor(0,1,1);
        pSurface1->DrawUV(4,4);
        sm_GraphicsLoop();
        smgfx_SetColor(0.5,1,0);
        sFaces[0]->Draw();
        sm_GraphicsLoop();
        pSurface2->DrawUV(4,4);
        sm_GraphicsLoop();
        sFaces[1]->Draw();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    double dTol = pEdge->GetTolerance();
    if (pOptCorner) {
        dTol = smos_Max(dTol,(double)pOptCorner->GetThisApproxTol3d());
    }

    SmPoint3d sStartPt = pStartFV->GetPoint();
    SmPoint3d sEndPt = pEndFV->GetPoint();
    SmVector3d sDir = sEndPt - sStartPt;
    SmBSplineCurve * pNew3DCurve = NULL;
    SmBSplineCurve * pUVCurve1 = NULL;
    SmBSplineCurve * pUVCurve2 = NULL;
    double dAngleTol = 2.0*SM_PI/180.0;;
    SER(sm_SrfSrfIntersection(crContext,      // in : context for new object construction
                              pStartFV,       // in : 1st point known to be on intersection curve
                              pEndFV,         // in : 2nd point known to be on intersection curve,
                                              //      NULL to ignore
                              &sDir,          // in : expected general direction of intersection curve from 1st point
                              NULL,           // in   expected intersection end direction
                              pSurface1,      // in : 1st intersecting surface
                              pSurface2,      // in : 2nd intersecting surface
                              dTol,           // in : max allowed distance between xSect Curve and surfaces
                              dAngleTol,      // in : max allowed angle between consecutive xSect curve segment tangents
                              pNew3DCurve,    // out: 3d intersection curve
                              pUVCurve1,      // out: associated UVTrimCurve on 1st surface
                              pUVCurve2,      // out: associated UVTrimCurve on 2nd surface
                              FALSE,          // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                              //             resorting to expensive general surf/surf xSect solver
                              NULL,           // in : reference point
                              iDebugLevel )); // in : iDebugLevel, 0 = No Debug output
    NER(pNew3DCurve);
    SmObjDelete sNewObj(pNew3DCurve);
    SM_ASSERT(pUVCurve1 != NULL) ; delete pUVCurve1 ; pUVCurve1 = NULL ;
    SM_ASSERT(pUVCurve2 != NULL) ; delete pUVCurve2 ; pUVCurve2 = NULL ;
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetColor(0,1,0);
        pNew3DCurve->Draw();
        smgfx_SetColor(1,0,0);
        smgfx_SetPointSize(10.0);
        pStartFV->GetPoint().Draw();
        pEndFV->GetPoint().Draw();
        smgfx_SetPointSize(4.0);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SetCurve(pNew3DCurve, TRUE); // side effect: delete current this->UVTrimCurves
    pNew3DCurve->SetOwner(this);
    sNewObj.Clear();

    return SM_SUCCESS;

} // end SmFilletEdge::CalcEdgeOnExtendedEdge

/*******************************************************************//**
PURPOSE: Calculate edge geometry of type SM_FE_FILLET_END.

NOTES:
***********************************************************************/
SmStatus SmFilletEdge::CalcFilletEnd
  (const SmContext & crContext,
   SmFilletCorner * pCorner)
{
  NER(pCorner);

  // Gather info.
  const SmVertex * cpVertex = pCorner->GetFilletedVertex();

  SmFilletVertex * pStartFV = (SmFilletVertex*)( this->GetVertex() );
  SmFilletVertex * pEndFV   = (SmFilletVertex*)( this->GetOtherVertex(pStartFV) );
  if ( ! pStartFV->IsProcessed() || ! pEndFV->IsProcessed() )
    { SER(SM_ERR); }

  SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)( this->GetPrimaryEdgeuse() );
  SmFilletGeom * pFilletGeom1 = pPrimEU->GetFilletGeom();
  NER( pFilletGeom1 );
  SmFilletSolver   * pFilSolver1    = pFilletGeom1->GetFilletSolver();

  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pFilletSurface = pFilletGeom1->GetFilletSurface();
  SM_FILLETSURF_TYPE * pFilletSurface = pFilletGeom1->GetFilletSurface();
  NER( pFilletSurface );

  // Drop two end points to fillet

  SmSolutionArray sSolutions1;
  SmSolutionArray sSolutions2;
  SmExtent2d sUVDomain = pFilletSurface->GetNaturalUVDomain();

  SmPoint3d sP1 = pStartFV->GetPoint();
  SER(pFilletSurface->GlobalPointSolve(sUVDomain, 
                                       SM_SO_MINIMIZE, 
                                       sP1, 
                                       SM_EFF_ZERO, 
                                       NULL, 
                                       SM_SR_ALL,
                                       sSolutions1 ));

  if ( sSolutions1.GetSize() < 1 )
    {
      SetStatus( SM_FE_END_PNT_DROP_FAILURE );
      SER( SM_ERR );
    }
  SmPoint2d sStartUV = SmPoint2d( sSolutions1[0].m_vStart[0], sSolutions1[0].m_vStart[1] );

  SmPoint3d sP2 = pEndFV->GetPoint();
  SER(pFilletSurface->GlobalPointSolve(sUVDomain, 
                                       SM_SO_MINIMIZE, 
                                       sP2, 
                                       SM_EFF_ZERO, 
                                       NULL, 
                                       SM_SR_ALL,
                                       sSolutions2 ));

  if ( sSolutions2.GetSize() < 1 )
    {
      SetStatus( SM_FE_END_PNT_DROP_FAILURE );
      SER( SM_ERR );
    }
  SmPoint2d sEndUV = SmPoint2d( sSolutions2[0].m_vStart[0], sSolutions2[0].m_vStart[1] );

  // Calc tangents along two side edges
  if ( pStartFV->GetPointClass() != SM_PC_EDGEUSE )
    { SER(SM_ERR); }
  if ( pEndFV->GetPointClass() != SM_PC_EDGEUSE )
    { SER(SM_ERR); }

  SmVector3d s3dTan1, s3dTan2;
  SmVector2d sUVTan1, sUVTan2;
  SmVector3d sPV[2];

  // Start:
  double      dT     = pStartFV->GetOriginalTParam();
  SmEdgeuse * pEU    = (SmEdgeuse*)pStartFV->GetPointClassObject(); NER(pEU);
  SmEdge    * pE     = pEU->GetEdge();
  SmCurve   * pCurve = pE->GetCurve(); NER(pCurve);

  SER(pCurve->Evaluate( dT, 1, TRUE, sPV ));
  if(sPV[1].LengthSquared() < SM_EFF_ZERO_SQ )
    { SER(SM_ERR); }
  s3dTan1 = sPV[1];

  SmOrientType eOrient = pE->GetPrimaryEdgeuse()->GetOrientation();
  SmVertex   * pV      = pE->GetVertex();
  if (   ( pV == cpVertex && eOrient == SM_OT_SAME )
      || ( pV != cpVertex && eOrient == SM_OT_OPPOSITE ) )
    { s3dTan1 = -s3dTan1; }

  // End:
  dT     = pEndFV->GetOriginalTParam();
  pEU    = (SmEdgeuse*)pEndFV->GetPointClassObject(); NER(pEU);
  pE     = pEU->GetEdge();
  pCurve = pE->GetCurve(); NER(pCurve);

  SER(pCurve->Evaluate( dT, 1, TRUE, sPV ));
  if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ )
    { SER(SM_ERR); }
  s3dTan2 = sPV[1];

  eOrient = pE->GetPrimaryEdgeuse()->GetOrientation();
  pV = pE->GetVertex();
  if (   ( pV != cpVertex && eOrient == SM_OT_SAME )
      || ( pV == cpVertex && eOrient == SM_OT_OPPOSITE ) )
    { s3dTan2 = -s3dTan2; }

  SER( pFilletSurface->DropVectors( sStartUV, TRUE, TRUE, 1, &s3dTan1, &sUVTan1 ));
  SER( pFilletSurface->DropVectors( sEndUV,   TRUE, TRUE, 1, &s3dTan2, &sUVTan2 ));

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetLook(1,2, 0,1,0); pFilletSurface->DrawUV(4,4); sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_SetLook(8,4, 1,0,0); sP1.Draw(); s3dTan1.Draw(&sP1); sm_GraphicsLoop();
      smgfx_SetLook(8,4, 0,0,1); sP2.Draw(); s3dTan2.Draw(&sP2); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Create a UV-space curve.
  SmPoint3d        sStartPnt( sStartUV );
  SmVector3d       sStartVec( sUVTan1 );
  SmPoint3d        sEndPnt  ( sEndUV );
  SmVector3d       sEndVec  ( sUVTan2 );
  SmBSplineCurve * pUVCurve = NULL;
  ULONG            lDimension = 2;

#ifdef SM_DEBUG_CODE
  SER(sm_InterpolateEndptsAndTangents(crContext, sStartPnt, sEndPnt, sStartVec, sEndVec, lDimension, pUVCurve, DebugLevel() ));  
#else  // no SM_DEBUG_CODE
  SER(sm_InterpolateEndptsAndTangents(crContext, sStartPnt, sEndPnt, sStartVec, sEndVec, lDimension, pUVCurve));
#endif // no SM_DEBUG_CODE

  NER( pUVCurve );

  double dDist = 0.0;
  SmBSplineCurve * pNew3DCurve = NULL;
  SER(pFilletSurface->LiftCurve(crContext, sUVDomain,
                               *pUVCurve,
                                pUVCurve->GetNaturalInterval(),
                                pFilSolver1->GetThisApproxTol3d()/10.0,
                                dDist,
                                pNew3DCurve ));
  NER( pNew3DCurve );

  this->SetCurve( pNew3DCurve, TRUE ); // side effect: delete current this->UVTrimCurves
  pNew3DCurve->SetOwner( this );
  pPrimEU->SetUVCurve( pUVCurve );
  pFilletGeom1->AddSideFilletEdgeuse( pPrimEU );

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetLook(3,5, 1,0,0); pNew3DCurve->DrawWDeriv( pNew3DCurve->GetNaturalInterval(), 0 ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmFilletEdge::CalcFilletEnd

/*******************************************************************//**
PURPOSE: Calculate and save geometry for this FilletEdge
         of type SM_FE_SETBACK_RAIL or SM_FE_RAIL_EXTENSION.

NOTES: FilletEdge has no geometry entering this method,
       i.e. m_pCurve, PrimEU->m_pUVTrimCurve, MateEU->m_pUVTrimCurve are NULL on entry.
       Build and Save SmCurves for
         this->Curve                   = trimmed copy of FilletEdge->FilletGeom->Rail->Curve
         this->PrimaryEU->UVTrimCurve, = trimmed copy of FilletEdge->FilletGeom->Rail->PrimEU->UVTrimCurve or
                                         new FilletSurface IsoParamCurve
         this->MateEU->UVTrimCurves    = trimmed copy of FilletEdge->FilletGeom->Rail->MateEU->UVTrimCurve
***********************************************************************/
SmStatus SmFilletEdge::CalcSetBackRail
  (const SmContext & crContext,
   SmFilletCorner  * pOptCorner)
{
  // locals
  SmFace          * pFace       = GetOriginalFace(); NER(pFace);
  SmFilletEdgeuse * pPrimEU     = (SmFilletEdgeuse*)GetPrimaryEdgeuse();
  SmFilletEdgeuse * pMateEU     = (SmFilletEdgeuse*)pPrimEU->GetMate();
  SmFilletGeom    * pFilletGeom = pPrimEU->GetFilletGeom(); NER(pFilletGeom);

  SmFilletSolver  * pFilSolver1 = pFilletGeom->GetFilletSolver();
  double            dApproxTol  = pFilSolver1->GetThisApproxTol3d();
  ULONG             lRailIndex  = pFilSolver1->FindIndexOfRailOnFace(pFace);
  SmFilletEdge    * pRail       = pFilletGeom->GetRail(lRailIndex);

  SmBSplineCurve  * pRailCurve  = SM_CAST_PTR(SmBSplineCurve, pRail->GetCurve()); NER(pRailCurve);

  SmExtent1d        sTrimIvl    = pRailCurve->GetNaturalInterval();

  SmFilletEdgeuse * pRailPrimEU = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
  SmFilletEdgeuse * pRailMateEU = (SmFilletEdgeuse*)pRailPrimEU->GetMate();
  SmBSplineCurve  * pRailUVCurve = pRailPrimEU->GetUVTrimCurvePointer();
  SmBSplineCurve  * pOrigUVCurve = pRailMateEU->GetUVTrimCurvePointer();

#ifdef SM_DEBUG_CODE
  // check state of This and pRail FilletEdges.
  //   Do they come from different Breps?  Both thisEdge and Rail seem to be in m_pPseudoBrep
  //   Do they both have geometry? thisEdge seems to have no geometry on input
  //                               Rail seems to have geometry
  //   So, this method seems to propagate geometry from Rail to ThisEdge.
  SmTArray<SmEdgeuse *> sEdgeuses ;
  //SmBrep         * pThisBrep            = this->GetBrep() ;
  //SmBrep         * pRailBrep            = pRail->GetBrep() ;

  // SmBSplineCurve * pThisCurve  = SM_CAST_PTR(SmBSplineCurve, this->GetCurve());
  this->GetEdgeuses(sEdgeuses) ;
  //SmBSplineCurve * pThisPrimUVTrimCurve = pPrimEU->GetUVTrimCurvePointer() ;
  //SmBSplineCurve * pThisMateUVTrimCurve = pMateEU->GetUVTrimCurvePointer() ;
  //SmBSplineCurve * pRailPrimUVTrimCurve = pRailPrimEU->GetUVTrimCurvePointer() ;
  //SmBSplineCurve * pRailMateUVTrimCurve = pRailPrimEU->GetMate()->GetUVTrimCurvePointer() ;

  if(0)
    {
      SM_DUMP_AND_ASSERT_VALID(pPrimEU) ;
      SM_DUMP_AND_ASSERT_VALID(pMateEU) ;
      SM_DUMP_AND_ASSERT_VALID(pRailPrimEU) ;
      SM_DUMP_AND_ASSERT_VALID(pRailMateEU) ;
    }
#endif // SM_DEBUG_CODE

  // Copy original curves
  SmBSplineCurve    * pNewOrigUVCurve = NULL;
  SmBSplineCurve    * pNewRailUVCurve = NULL;

  // GWC: Stop Fillets from switching SmCurve objects to SmBSplineCurve objs
  SmCurve           * pCopyCurve      = NULL ;
  SmBSplineCurve    * pNewRailCurve   = NULL ;
  pRailCurve->Copy(crContext, pCopyCurve) ;  
  pNewRailCurve = SM_CAST_PTR(SmBSplineCurve, pCopyCurve) ; NER(pNewRailCurve);
  //   
  //   SmBSplineCurve    * pNewRailCurve   = new (crContext) SmBSplineCurve(*pRailCurve);

  if (pOrigUVCurve)
    { pNewOrigUVCurve = new (crContext) SmBSplineCurve(*pOrigUVCurve); }
  if (pRailUVCurve)
    { pNewRailUVCurve = new (crContext) SmBSplineCurve(*pRailUVCurve); }
  else // RailUVCurve as the appropriate FilletSurface IsoParameterCurve
    {
      SmPoint3d sCurvePnt, sSurfPnt;

      SmSurface * pSurface   = pFilletGeom->GetFilletSurface();
      SmExtent2d  sUVDomain  = pSurface->GetNaturalUVDomain();

      // compare RailPoint to a FilletSurface Point to see if rail runs along sUVDomain.MinY IsoParamCurve
      double    dParam = sTrimIvl.Evaluate(0.34567);
      double    dIsoV  = sUVDomain.GetMin().y;
      SmPoint2d sTestUV(dParam, dIsoV);

      // evaluate a RailCurve point and a FilletSurface MinY point
      SER(pNewRailCurve->EvaluatePoint(dParam,sCurvePnt));
      SER(pSurface->EvaluatePoint(sTestUV,sSurfPnt));

      // Compare RailCurve point to FilletSurface MinY point
      if (sCurvePnt.DistanceBetween(sSurfPnt) > dApproxTol)
        {
          // evaluate FilletSurface MaxY point
          dIsoV     = sUVDomain.GetMax().y;
          sTestUV.y = dIsoV;
          SER(pSurface->EvaluatePoint(sTestUV,sSurfPnt));

          // Compare RailCurve point to FilletSurface MaxY point
          if (sCurvePnt.DistanceBetween(sSurfPnt) > dApproxTol)
            {
              // Rail is not syncronizing with the parametrization of the fillet surface
              SER(SM_ERR);
            }
        } // End comparing sample points to determine which FilletSurface IsoParamCurve maps to the RailCurve

      // FilletSurface IsoParamCurve end points
      SmPoint2d sStartUV = SmPoint2d(sTrimIvl.GetMin(),sTestUV.y);
      SmPoint2d sEndUV   = SmPoint2d(sTrimIvl.GetMax(),sTestUV.y);

      // Create UV-curve on fillet for this rail
      SER(SmBSplineCurve::CreateLineSegment(crContext,
                                            2,
                                            SmPoint3d(sStartUV),
                                            SmPoint3d(sEndUV),
                                            pNewRailUVCurve));
      NER(pNewRailUVCurve);
      SER(pNewRailUVCurve->EditParameterization(sTrimIvl));

    } // end build pNewRailUVCurve as appropriate FilletSurface IsoParamCurve branch

  // prevent leaks on SER()s
  SmObjDelete sClean0(pNewRailCurve  ) ;
  SmObjDelete sClean1(pNewOrigUVCurve) ;
  SmObjDelete sClean2(pNewRailUVCurve) ;

  // End FilletVertices for the FilletEdge
  SmFilletVertex * pV0 = (SmFilletVertex*)GetVertex();
  SmFilletVertex * pV1 = (SmFilletVertex*)GetOtherVertex(pV0);

  // check state - FilletVertices should be processed so that their Vertex->Points are set
  if (!pV0->IsProcessed()) SER(SM_ERR);
  if (!pV1->IsProcessed()) SER(SM_ERR);

  // End FilletVertex Points
  SmPoint3d sPnt0   = pV0->GetPoint();
  SmPoint3d sPnt1   = pV1->GetPoint();


  double    dParam1 = sTrimIvl.GetMin();
  double    dParam2 = sTrimIvl.GetMax();
  double    dDist;
  SmBoolean bSuccess;

  // Drop Vertex0 Point to RailCurve
  if(   SM_SUCCESS != pNewRailCurve->DropPoint(sTrimIvl,         // in : target curve allowed domain
                                               sPnt0,            // in : Point to drop to curve
                                               NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                               dApproxTol*10.0,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                               NULL,             // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                               bSuccess,         // out: TRUE = found a drop point
                                               dParam1,          // out: found drop curve param
                                               dDist)            // out: found drop distance
                                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior
                                                                
     || !bSuccess)
    {
      SetStatus(SM_FE_END_PNT_DROP_FAILURE);
      SER(SM_ERR);
    }

  // Drop Vertex1 Point to RailCurve
  if(   SM_SUCCESS != pNewRailCurve->DropPoint(sTrimIvl,         // in : target curve allowed domain
                                               sPnt1,            // in : Point to drop to curve
                                               NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                 //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                 //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                               dApproxTol*10.0,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                 //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                 //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                 //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                               NULL,             // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                               bSuccess,         // out: TRUE = found a drop point
                                               dParam2,          // out: found drop curve param
                                               dDist)            // out: found drop distance
                                                                 // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                 //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                 //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                 //      default:[SM_SO_MINIMIZE] to preserve original behavior
                                                                
     || !bSuccess)
    {
      SetStatus(SM_FE_END_PNT_DROP_FAILURE);
      SER(SM_ERR);
    }

  // set Rail NewCurve and UVTrimCurve and NewOrigUVCurve TrimIvls to found DropParam values
  SmBoolean bReverseCurves = FALSE ;
  if (dParam1 > dParam2) { bReverseCurves = TRUE ;
                            sTrimIvl.SetMinMax(dParam2,dParam1) ;
                         }
  else                   { sTrimIvl.SetMinMax(dParam1,dParam2) ; }
  SER(pNewRailCurve->Trim(sTrimIvl));     // may snap sIvl by tol to existing knots
  SER(pNewRailUVCurve->Trim(sTrimIvl));   // may snap sIvl by tol to existing knots
  if (pNewOrigUVCurve) { SER(pNewOrigUVCurve->Trim(sTrimIvl)); }  // may snap sIvl by tol to existing knots

  // when needed - reverse curve parameterizations
  if (bReverseCurves)
    {
      SmExtent1d sIvl = pNewRailCurve->GetNaturalInterval();
      SER(pNewRailCurve->ReverseParameterization(sIvl,sIvl));
      SER(pNewRailUVCurve->ReverseParameterization(sIvl,sIvl));
      if (pNewOrigUVCurve) { SER(pNewOrigUVCurve->ReverseParameterization(sIvl,sIvl)); }
    }

  // Set this FilletEdge->Curve and UVCurve values - no memory leaks, all new Curves are saved under thisEdge data pointers.
  SetCurve(pNewRailCurve, TRUE); // side effect: delete current this->UVTrimCurves
  pNewRailCurve->SetOwner(this);
  pPrimEU->SetUVCurve(pNewRailUVCurve);  // set this->UVTrimCurves
  pMateEU->SetUVCurve(pNewOrigUVCurve);  // set this->UVTrimCurves

  // saved the curves - make them permanent
  sClean0.Clear() ;
  sClean1.Clear() ;
  sClean2.Clear() ;

  // for Corner of type = SM_FCR_N_x_2,
  if(   pOptCorner
     && pOptCorner->GetCornerType() == SM_FCR_N_x_2)
    {
      double dLen = pNewRailCurve->ApproximateLength(sTrimIvl,5);
      if (dLen < pOptCorner->GetThisApproxTol3d())
        {
          // The edge is too small, try to squeeze it (later)
          SmFilletVertex * pDeleteFV = pV0;
          if (pDeleteFV->GetFilletVertexType() == SM_FV_FILLET_X_FILLET) { pDeleteFV = pV1; }
          if (pDeleteFV->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE)  { SER(SM_ERR); }
          pDeleteFV->SetStatus(SM_FV_TO_BE_DELETED);
          this->SetStatus(SM_FE_TO_BE_SQUEEZED);

          // all done
          return SM_SUCCESS;
        } // end NewCurve is short check
    } // end CornerTyupe is SM_FCR_N_x_2 check

  // add the PrimEU to the m_vSideEUs array
  pFilletGeom->AddSideFilletEdgeuse(pPrimEU);

  // all done
  return SM_SUCCESS;

} // end SmFilletEdge::CalcSetBackRail

/*******************************************************************//**
PURPOSE: If any of our FilletBrepEdges is listed in the input array,
            clear out its pointer.

NOTES: Called when those edges have been deleted.
***********************************************************************/
SmStatus SmFilletEdge::ClearFilletBrepEdges
 (const SmTArray<SmEdge*> & crEdges)
{
  ULONG i;
  for ( i = 0; i < crEdges.GetSize(); i++ )
  {
      if ( m_pFilletBrepEdge1 == crEdges[i] ) m_pFilletBrepEdge1 = NULL;
      if ( m_pFilletBrepEdge2 == crEdges[i] ) m_pFilletBrepEdge2 = NULL;
  }

  return SM_SUCCESS;

} // end SmFilletEdge::ClearFilletBrepEdges(

/*******************************************************************//**
PURPOSE: Match this filletEdge->Curve to an edge in the input edge
            array geometrically and set the m_pFilletBrepEdge pointer
            with it.  If no edge is found an error is returned.

NOTES: This function is helpful after stitching in which
       the original FilletEdge/BrepEdge mapping may be lost.

       In use: crEdges is always the set of edges in m_pFilletBrep.
***********************************************************************/
SmStatus SmFilletEdge::FindFilletBrepEdge
 (const SmTArray<SmEdge*> & crEdges,            // in : Array of edges to check
  SmBoolean                 bFindMultipleEdges) // in : TRUE = set m_pFilletBrepEdge1 and m_pFilletBrepEdge2 with
                                                //             1st 2 edges whose midPoints are within
                                                //             tolerance of FilletEdge->Curve
                                                //      FALSE= set m_pFilletBrepEdge1 with
                                                //             1st edge whose midPoint is within
                                                //             tolerance of FilletEdge->Curve->MidPoint
{
  // get this Fillet Edge's curve
  SmCurve * p3DCurve = GetCurve();
  NER(p3DCurve);
  if (p3DCurve->IsDegenerate())
    { return SM_SUCCESS; }

  // let sPnt = curve's midPoint
  SmExtent1d sIvl = p3DCurve->GetNaturalInterval();
  SmPoint3d sPnt;
  SER(p3DCurve->EvaluatePoint(sIvl.Evaluate(0.5),sPnt));

  // These values are for checking tolerance slop.  It's possible to have
  // a midpoint of a different curve within the Edge's tolerance of the edge.
  // If we find midpoints that are not right on, we will save the distances
  // and keep looking for a tight one.  [B332]
  SmEdge * pBestEdge1        = NULL, *pBestEdge2 = NULL;
  double   dDist, dBestDist1 = SM_BIG_DOUBLE, dBestDist2 = SM_BIG_DOUBLE;
  double   dScaledZero       = 10 * SM_EFF_ZERO * ( 1.0 + sPnt.GetMaxDimension() );

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 1,0,0 ); p3DCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 1,4, 0,0,1 ); sPnt.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // First just look for a matching midpoint.

  // for every input edge
  ULONG ii;
  for (ii=0; ii<crEdges.GetSize(); ii++)
    {
      SmEdge *pE = crEdges[ii];

      // let sEdgePnt = edge midPoint
      SmPoint3d sEdgePnt;
      SER(pE->GetCurve()->EvaluatePoint(pE->GetInterval().Evaluate(0.5),sEdgePnt));

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 ) 
        {
          smgfx_SetLook( 3,4, 0,0,0 ); pE->Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 3,6, 0,1,0 ); sEdgePnt.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // when looking for 1 edge branch
      if (!bFindMultipleEdges)
        {
          // when distance between midpoints is less than tolerance
          dDist = sEdgePnt.DistanceBetween(sPnt);
          if (dDist < pE->GetTolerance())
            {
              if ( dDist < dScaledZero )
                {
                  // Remember that this edge maps to this fillet edge.
                  m_pFilletBrepEdge1 = pE;
                  return SM_SUCCESS;
                }

              // Close, but not a great result.
              // Note this proximity and keep looking.
              if ( dDist < dBestDist1 )
              {
                  pBestEdge1 = pE;
                  dBestDist1 = dDist;
              }
              continue;
            }
        } // end looking for 1 edge branch

      else // looking for multiple edges
        {
          // Drop the point onto p3DCurve
          SmSolution sSData[4];
          SmSolutionArray sSolutions(4,sSData);
          SER(p3DCurve->GlobalPointSolve(sIvl,
                                         SM_SO_INTERSECT,
                                         sEdgePnt,
                                         pE->GetTolerance(),
                                         NULL,
                                         NULL,
                                         SM_SR_SINGLE,
                                         sSolutions));

          // when EdgePnt is within tolerance of FilletEdge->Curve
          if (sSolutions.GetSize() >= 1)
            {
              dDist = sSolutions[0].m_vStart.m_dSolutionValue;
              if ( dDist < dScaledZero )
                {
                  if (m_pFilletBrepEdge1 == NULL)
                    {
                      m_pFilletBrepEdge1 = pE;
                    }
                  else if (m_pFilletBrepEdge2 == NULL)
                    {
                      m_pFilletBrepEdge2 = pE;
                      break;  // Found two good ones, all done.
                    }
                  else
                    {
                      SM_DBG_WARN(_T("Unexpected branch in SmFilletEdge::FindFilletBrepEdge()\n"));
                    }
                } // end if dDist < dScaledZero
              else
                {
                  // Close, but not a great result.
                  // Note this proximity and keep looking.
                  if ( dDist < dBestDist1 )
                    {
                      dBestDist2 = dBestDist1;
                      dBestDist1 = dDist;
                      pBestEdge2 = pBestEdge1;
                      pBestEdge1 = pE;
                    }
                  else if ( dDist < dBestDist2 )
                    {
                      dBestDist2 = dDist;
                      pBestEdge2 = pE;
                    }
                }  // end else EdgePnt is (not) within scaled zero of Curve check
            }  // end EdgePnt is within tolerance of Curve check
        } // end looking for multiple edges branch
    } // end iter every input edge

  // every filletEdge should map to a Brep edge
  if ( m_pFilletBrepEdge1 )
    { return SM_SUCCESS; }

  // Didn't set it.  See if we have a close approach.
  m_pFilletBrepEdge1 = pBestEdge1;
  if ( bFindMultipleEdges )
    { m_pFilletBrepEdge2 = pBestEdge2; }

  // If that got it, we're done.
  if ( m_pFilletBrepEdge1 )
    { return SM_SUCCESS; }

  // Someday later we may need to do it again using closest point type of
  // functionality if the paramterizations don't match along the adjacent
  // edges which get combined when stitching.
  SE(SM_ERR);

  return SM_ERR;

} // end SmFilletEdge::FindFilletBrepEdge

/*******************************************************************//**
PURPOSE: Get NonNULL SmFilletEdge::m_pFilletBrepEdge1 and m_pFilletBrepEdge2.

NOTES: Only used when 'this' FilletEdge->GetBrep() == m_pPseudoBrep 
    (e.g. this FIlletEdge was made by a FilletCorner or FilletSolver)
    
    Typically, only one edge is associated with this. However,
    two edges could be associated with this when this was split
    during N-sided patches creation.
***********************************************************************/
SmStatus SmFilletEdge::GetFilletBrepEdges
 (SmTArray<SmEdge*> & rEdges) 
 const
{
  rEdges.ReSet();
  if (m_pFilletBrepEdge1) rEdges.Add(m_pFilletBrepEdge1);
  if (m_pFilletBrepEdge2) rEdges.Add(m_pFilletBrepEdge2);
  return SM_SUCCESS;

} // end SmFilletEdge::GetFilletBrepEdges

/*******************************************************************//**
PURPOSE: Make a fillet edge from a start vertex to an end vertex needed
            at a FilletCorner to terminate a fillet surface.

NOTES: 1. allocates 1 FilletEdge and 2 filletEdgeuse->FilletVertexuse
          pairs to connect the FilletEdge to the input FilletVertices.
       2. The new SmFilletEdge has no Curve or UVTrimCurve data.
***********************************************************************/
SmStatus SmFilletEdge::MakeFilletEdge
 (SmFilletBrep    * pPseudoBrep,     // in : target Brep to receive new topology objects
  SmFilletVertex  * pStartVertex,    // in : start of new FilletEdge
  SmFilletVertex  * pEndVertex,      // in : end   of new FilletEdge
  SmFilletEdge   *& rpNewFilletEdge, // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
  SmFilletCorner  * cpCorner)        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                     //      NULL to ignore, default:[NULL]
{
  // Check input
  NER(pStartVertex);
  NER(pEndVertex);

  // allocates 1 FilletEdge and 2 filletEdgeuse->FilletVertexuse pairs to connect
  // the FilletEdge to the input FilletVertices

  // allocate the FilletEdge
  SmFilletEdge * pNewE = new (pPseudoBrep) SmFilletEdge(pPseudoBrep,cpCorner);
  NER(pNewE); SmObjDelete sClean1(pNewE);

  // allocate 1st FilletEdgeuse
  SmFilletEdgeuse * pNewEU1 = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
  NER(pNewEU1); SmObjDelete sClean4(pNewEU1);

  // allocate 2nd FilletEdgeuse
  SmFilletEdgeuse * pNewEU2 = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
  NER(pNewEU2); SmObjDelete sClean5(pNewEU2);

  // allocate 1st FilletVertexuse - connect it to 1st FilletEdgeuse
  SmFilletVertexuse * pNewVU1 = new (pPseudoBrep) SmFilletVertexuse();
  NER(pNewVU1); SmObjDelete sClean2(pNewVU1);
  pNewVU1->SetProperty(pNewEU1);

  // allocate 2nd FilletVertexuse - connect it to 2nd FilletEdgeuse
  SmFilletVertexuse * pNewVU2 = new (pPseudoBrep) SmFilletVertexuse();
  NER(pNewVU2); SmObjDelete sClean3(pNewVU2);
  pNewVU2->SetProperty(pNewEU2);

  // connect FilletVertices to FilletEdge

  // Put VU1 into start vertex list and VU2 into end vertex
  SER(pStartVertex->PostInsert(pNewVU1));
  SER(pEndVertex->PostInsert(pNewVU2));

  // Put EU1 and EU2 into edge - Make EU1 the Primary Edgeuse
  // which determines the topological orientation of the edge.
  // The topological orientation of the edge goes from the
  // start vertex to the end vertex.
  pNewEU1->SetOrientation(SM_OT_SAME);
  pNewEU2->SetOrientation(SM_OT_OPPOSITE);
  SER(pNewE->PreInsert(pNewEU1));
  SER(pNewE->PostInsert(pNewEU2));

  // Attach edgeuses to vertexuses (2nd half of backpointer set)
  pNewEU1->SetFilletVertexuse(pNewVU1);
  pNewEU2->SetFilletVertexuse(pNewVU2);

  // Put edge into brep's edge list
  //SER(m_pPseudoBrep->m_pEdgeListHead->PostInsert(pNewE));

  // Clear cleanup objects
  sClean1.Clear(); sClean2.Clear(); sClean3.Clear();
  sClean4.Clear(); sClean5.Clear();

  // Assign output
  rpNewFilletEdge = pNewE;

  return SM_SUCCESS;

} // end SmFilletEdge::MakeFilletEdge


/*******************************************************************//**
    Static functions
***********************************************************************/

/*******************************************************************//**
PURPOSE: Decide whether this FilletEdge lies in one or the other of two Faces

NOTES:
***********************************************************************/
SmBoolean SmFilletEdge::ShouldSwapFaces( SmFace *pOrigFace, SmFace *pNewFace )
{
  // This FilletEdge should lie in one Face and not the other.
  // Drop its midpoint to both.
  // We have Face::Point3DClassify(), but that does the whole loop-containment thing.
  // Simplify: drop point to both and check domains.

  // Find a point to drop.  Our Domain might not be set; we may or may not have
  // a CurveClass; failing those we have an end Vertex in m_vCorner.
  SmPoint3d sTestPt;

  // Sometimes the midpoint can be inconclusive.  In that case, if we have a curve,
  // try the start and end points.  It's likely that the curve either
  // starts or ends on one Face or the other.
  // I'm going to use a goto statement: it's actually clearer than putting this
  // whole routine into a loop which might have 1 or 3 iterations.
  int iWhichTry = 1;

TryAgain:

  if ( ! ( m_vInterval.IsInit()  ||  m_vInterval.HasNegativeLength() ) )
  {
      double dParam =   ( iWhichTry == 1 ) ? m_vInterval.GetMid()
                      : ( iWhichTry == 2 ) ? m_vInterval.GetMin()
                      :                      m_vInterval.GetMax();
      this->GetCurve()->EvaluatePoint( dParam, sTestPt );
  }
  else if ( this->GetCurveClass() != NULL )
  {
      SmExtent1d sEdgeDomain = this->GetCurveClass()->GetInterval();
      double dParam =   ( iWhichTry == 1 ) ? sEdgeDomain.GetMid()
                      : ( iWhichTry == 2 ) ? sEdgeDomain.GetMin()
                      :                      sEdgeDomain.GetMax();
      this->GetCurve()->EvaluatePoint( dParam, sTestPt );
  }
  else if ( this->GetFilletCorner() != NULL )
  {
      SmFilletCorner *cpCorner = this->GetFilletCorner();
      const SmVertex *pBaseVtx = cpCorner->GetFilletedVertex();
      sTestPt = pBaseVtx->GetPoint();

      // We have no curve, only one point, so only one try:
      iWhichTry = 3;
  }
  else
    {
      // Shouldn't happen: set a breakpoint here.
      SM_DBG_WARN(_T("Possible error: unhandled case") );
      return FALSE;
    }

  int iWhichFace = sm_ClassifyFacePoint( sTestPt, pOrigFace, pNewFace );

  if ( iWhichFace == 1 )
    { return FALSE; }

  if ( iWhichFace == 2 )
    { return TRUE; }

  if ( iWhichFace == 3 )
    { return FALSE; }   // If in both, we don't have to switch m_pOrigFace.

  // This should not happen: the point does not classify to its own Face.
  // Sometimes Faces get extended, or sometimes there is geometric slop.
  // Try different points on the curve. 
  if ( iWhichFace == 0 )
    {
      iWhichTry++;
      if ( iWhichTry <= 3 )
        { goto TryAgain; }
    }

  return FALSE;

}  // end SmFilletEdge::ShouldSwapFaces

/*******************************************************************//**
PURPOSE: Deal with our Face being split.

NOTES: The previous behavior, using Composite Faces, was to check the underlying
   surface to see whether the FilletEdge in question was originally in a given
   Face, before the Face was split.  (All of the split Faces would share a
   common surface.)  That means that it would process the FilletEdge if it is
   on any of the Faces split from the original.  To preserve that behavior, we
   always record the the split Face: we have a list of them.  The only question
   is whether or not to replace m_pOrigFace with pNewFace.
***********************************************************************/
SmStatus SmFilletEdge::UpdateSplitFace( SmFace *pOrigFace, SmFace *pNewFace )
{
  SmFace *pRailFace = this->GetOriginalFace();

#ifdef SM_DEBUG_CODE
int iDebugLevel = -1;
  if ( iDebugLevel > 0 )
    {
      smgfx_Erase();
      SmBrep *pBrep = pOrigFace->GetBrep();
      smgfx_SetLook(1,2, 0,0,0); if( pBrep    ) pBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(5,8, 0,0,0);                this ->Draw();     sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); if( pOrigFace) pOrigFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,0); if( pNewFace ) pNewFace ->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); if( pRailFace) pRailFace->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  if ( pRailFace == pOrigFace )
  {
      // This FilletEdge was in the Face that got split.

      SmBoolean bSwap = this->ShouldSwapFaces( pOrigFace, pNewFace );
      if ( bSwap == TRUE )
        {
          this->SetOriginalFace( pNewFace );
          this->AddSplitFace( pOrigFace );  // pOrigFace is now a Split Face.
        }
      else // Always add, if our Orig Face == pOrigFace.
        { this->AddSplitFace( pNewFace ); }
  }

  return SM_SUCCESS;

} // end SmFilletEdge::UpdateSplitFace

/*******************************************************************//**
PURPOSE: Join 'Start' Fillet Vertex to 'End' Fillet Vertex by 3D curves
 (and their corresponding UV curves) obtained from 'SIDE' fillet edgeuses

NOTES:

  Find a sequence of edges in the rSideEUs array that allows moving from
  startFilletVertex to EndVilletVertex through edges that share common
  end vertices as:

  startFV to edge to commonVertex to nextEdge to nextCommonVertex to . . . EndFV

  if such a sequence is found, set rbMadeConnection = TRUE and use the
    edge sequence to load r3DCurves, rUVCurves, and rCurveOrients in sequence.
  else
    set rbMadeConnection = FALSE and return.

***********************************************************************/
static SmStatus JoinTwoFilletVertsByCurves
  (SmFilletGeom                * pFilletGeom,      // in : target filletGeom
   SmVertex                    * pStartFV,         // in : Start FilletPoint of FilletEdges which bounds FilletGeom
   SmVertex                    * pEndFV,           // in : End FilletPoint of FilletEdges which bounds FilletGeom
   SmTArray<SmFilletEdgeuse*>  & rSideEUs,         // in : Collection of all 'side' FilletEdgeuses of FilletGeom
   double                        dUVTol,           // in : UV-Point tolerance of the fillet surface
   SmTArray<SmCurve*>          & r3DCurves,        // i/o: OrderedEdge List1 3DCurves
   SmTArray<SmBSplineCurve*>   & rUVCurves,        // i/o: OrderedEdge List1 UVCurves
   SmTArray<SmOrientType>      & rCurveOrients,    // i/o: OrderedEdge List1 Orientations
   SmBoolean                   & rbMadeConnection, // out: TRUE = Success, m_vSideEUs from Start to End FilletVertex added to OrderedEdgeList 1
                                                   //      FALSE= Failure,
   int                           iDebugLevel)
{
  // init output
  rbMadeConnection = FALSE;

  // locals
  ULONG ii ;
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pFilletSurface = pFilletGeom->GetFilletSurface();
  SM_FILLETSURF_TYPE * pFilletSurface = pFilletGeom->GetFilletSurface();
  double             d3DTolerance   = pFilletGeom->GetFilletSolver()->GetThisApproxTol3d();

#ifdef SM_DEBUG_CODE
  ULONG di ;
  // draw start vertex(green) and end vertex (blue)
  if ( iDebugLevel > 0 )
    {
      SmFilletSolver    * pFilletSolver    = pFilletGeom->GetFilletSolver() ;
      SmFilletExecutive * pFilletExecutive = pFilletSolver ? pFilletSolver->GetFilletExecutive() : NULL ;
      SmBrep            * pTargetBrep      = pFilletExecutive ? pFilletExecutive->GetTargetBrep() : NULL ;
      SmBrep            * pFilletBrep      = pFilletExecutive ? pFilletExecutive->GetFilletBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pTargetBrep) pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if(pFilletBrep) pFilletBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,0) ; pStartFV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 0,0,1) ; pEndFV->Draw();   sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->Draw() ;
                                                                          sm_GraphicsLoop() ;
                                                                        }
      smgfx_SetLook(2,3, 0,1,1) ; for(di=0;di<rSideEUs.GetSize(); di++) { if(rSideEUs[di]) rSideEUs[di]->Draw() ;
                                                                          sm_GraphicsLoop() ;
                                                                        }
      sm_GraphicsLoop();
    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  // when all UVTrimCurves are present - do 2D testing else do more expensive 3D testing
  SmBoolean bDo2DTest = TRUE;
  for(ii=0;ii<rUVCurves.GetSize() && bDo2DTest;ii++)
    {
      if (rUVCurves[ii] == NULL)
        { bDo2DTest = FALSE; }
    }

  // Find the 3d or UV point at the end of the input OrderedEdge List
  SmOrientType eLastCurveOrient = rCurveOrients.GetLast();
  SmCurve    * pLastCurve       =   (bDo2DTest)
                                  ? rUVCurves.GetLast()
                                  : r3DCurves.GetLast() ;

  SmExtent1d   sIvl             = pLastCurve->GetNaturalInterval();
  double       dParam           =   (eLastCurveOrient == SM_OT_SAME)
                                  ? sIvl.GetMax()
                                  : sIvl.GetMin() ;
  SmPoint3d    sEndPnt;
  SER(pLastCurve->EvaluatePoint(dParam, sEndPnt));

  // Make a copy of side-EUs array
  SmTArray<SmFilletEdgeuse*> sCopySideEUs;
  sCopySideEUs.Append(rSideEUs);

  // until we connect to target EndFilletVertex
  SmVertex  * pCurrFV         = pStartFV;
  SmBoolean   bMadeConnection = TRUE;
  while (   pCurrFV != pEndFV
         && bMadeConnection)
    {
      bMadeConnection = FALSE;

      // for every SideEdgeuse
      for(ii=0; ii<sCopySideEUs.GetSize(); ii++)
        {
          SmEdge           * pCurrEdge    = sCopySideEUs[ii]->GetEdge();
          SmCurve          * pCurve       = pCurrEdge->GetCurve();
          SmVertex         * pEdgeStart   = pCurrEdge->GetVertex();
          SmVertex         * pEdgeEnd     = pCurrEdge->GetOtherVertex(pEdgeStart);
          SmBSplineCurve   * pSideUVCurve = sCopySideEUs[ii]->GetUVTrimCurve() ;

          // skip edges not connected to the current FilletVertex
          if (pEdgeStart != pCurrFV && pEdgeEnd != pCurrFV)
            { continue; }

          // Found 'NEXT' curve
          SmOrientType  eOrient ;
          SmVertex    * pNextFV  ;
          SmExtent1d    sSideIvl ;
          if (pSideUVCurve)  sSideIvl  = pSideUVCurve->GetNaturalInterval() ;
          if(pEdgeStart == pCurrFV) { eOrient = SM_OT_SAME ;
                                      pNextFV = pEdgeEnd ;
                                    }
          else                      { eOrient = SM_OT_OPPOSITE ;
                                      pNextFV = pEdgeStart ;
                                    }

#ifdef SM_DEBUG_CODE
          // draw SideEdge->Curve(red), SideEdge->UVTrimCurve(red)
          //      SiedEdge->EndVertices(green), LastCurve(blue)
          if ( pFilletGeom->DebugLevel() > 0 )
            {
              pLastCurve->Dump();
              if (pSideUVCurve) { pSideUVCurve->Dump(); }

              SmFilletSolver    * pFilletSolver    = pFilletGeom->GetFilletSolver() ;
              SmFilletExecutive * pFilletExecutive = pFilletSolver ? pFilletSolver->GetFilletExecutive() : NULL ;
              SmBrep            * pTargetBrep      = pFilletExecutive ? pFilletExecutive->GetTargetBrep() : NULL ;
              SmBrep            * pFilletBrep      = pFilletExecutive ? pFilletExecutive->GetFilletBrep() : NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pTargetBrep) pTargetBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; if(pFilletBrep) pFilletBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 0,1,0) ; pStartFV->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 0,0,1) ; pEndFV->Draw();   sm_GraphicsLoop();
              smgfx_SetLook(2,3, 1,0,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->Draw() ;
                                                                                  sm_GraphicsLoop() ;
                                                                                }
              smgfx_SetLook(2,3, 0,1,1) ; for(di=0;di<rSideEUs.GetSize(); di++) { if(rSideEUs[di]) rSideEUs[di]->GetEdge()->GetCurve()->Draw() ;
                                                                                  sm_GraphicsLoop() ;
                                                                                }
              smgfx_SetLook(4,5, 1,0,0); pCurve->DrawWDeriv(pCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 1,0,0); if(pSideUVCurve) pSideUVCurve->DrawWDeriv(sSideIvl,0); sm_GraphicsLoop();
              smgfx_SetLook(2,7, 0,1,0) ; pEdgeStart->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,7, 0,0,1) ; pEdgeEnd->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(5,8, 1,0,1) ; pLastCurve->DrawWDeriv(sIvl,0); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // when testing in 2D with UVTrimCurves - skip Curves with bad gaps - no skips in 3D
          if (bDo2DTest && pSideUVCurve)
            {
              // get ordered endParameters
              double dStartParam = sSideIvl.GetMin();
              double dEndParam   = sSideIvl.GetMax();
              if (eOrient == SM_OT_OPPOSITE)
                {
                  SM_SWAP(double,dEndParam,dStartParam);
                }

              // when distance between last and current curve endPoints is out of tolerance
              SmPoint3d sStartPnt;
              SER(pSideUVCurve->EvaluatePoint(dStartParam, sStartPnt));

              double dUVDist = sStartPnt.DistanceBetween(sEndPnt);

              if (dUVDist > dUVTol)
                {
                  // skip curves with large gaps
                  if (dUVDist > 20.0*dUVTol)
                    {
                      continue; // Too much of a gap
                    }

                  // for slightly out of tolerance gaps - test 3D distance between endPoints
                  SmPoint3d s3DStartPnt;
                  SmPoint3d s3DEndPnt;
                  SmPoint2d sUVStart(sStartPnt.x,sStartPnt.y);
                  SmPoint2d sUVEnd  (sEndPnt.x,  sEndPnt.y);
                  SER(pFilletSurface->EvaluatePoint(sUVStart,s3DStartPnt));
                  SER(pFilletSurface->EvaluatePoint(sUVEnd,s3DEndPnt));
                  double    dGap = s3DStartPnt.DistanceBetween(s3DEndPnt);

                  // reject gaps larger than 10*tolerance
                  if (dGap > 10.0*d3DTolerance)
                    {
                      SM_DBG_WARN(_T("JoinTwoFilletVertsByCurves(): Gap found between UV curves in making fillet face - Expect FilletGeom Curves to connect without gaps (this is a bug in the FilletGeom construction or perhaps a pole?) - continuing"));
                      continue;
                    }
                } // end 2D gap larger than tolerance check

              // found a keeper edge -increment EndPoint
              SER(pSideUVCurve->EvaluatePoint(dEndParam, sEndPnt));
            } // end 2DTest check

          // arrive here when ready to add this Edge to the OrderedEdge List
          // because we are working in 3D or having found an acceptable 2D curve

          // Add Edge to Arrays - prepare for next iteration
          r3DCurves.Add(pCurve) ;
          rUVCurves.Add(pSideUVCurve) ;
          rCurveOrients.Add(eOrient) ;
          sCopySideEUs.RemoveAt(ii) ;
          pCurrFV         = pNextFV ;
          bMadeConnection = TRUE ;

          break ;

        } // end iter every sideEdgeuse
    } // end while trying to connect to target endFilletVertex

  if (pCurrFV == pEndFV) { rbMadeConnection = TRUE; }

  // all done
  return SM_SUCCESS;

} // end JoinTwoFilletVertsByCurves

/*******************************************************************//**
PURPOSE: Join two uv-curves by uv-lines

NOTES: The end of UVTrimCurve1 is expected to coincide with the start
 of UVTrimCurve2 to tight tolerances.  When this is not true create a
 set of up to 3 isoparameter UVCurves to fill in the gap as follows

 +---------------------+
 |          *          |
 |           \         |
 | in :       \        |
 |   pUVCurve2 \       |
 |              \      |
 |               *=====* out: 3rd cap Curve created if NonDegenerate and added to rUVCurves array
 |                     " out: 2nd cap Curve created if NonDegenerate and added to rUVCurves array
 |    *--------*=======* out: 1st cap Curve created if NonDegenerate and added to rUVCurves array
 | in : pUVCurve1      |
 +---------------------+

 gwc:note The design of the cap curves can be improved at a later time.

***********************************************************************/
static SmStatus JoinVertsByCapping
  (const SmContext             & crContext,          // in : context for new object construction
   double                        dThisApproxTol3d,   // in :
   SmSurface                   * pSurface,           // in : FilletSurface
   SmBSplineCurve              * pUVCurve1,          // in : 1st target UVCurve
   SmOrientType                  eOrient1,           // in : and its orientation
   SmBSplineCurve              * pUVCurve2,          // in : 2nd target UVCurve
   SmOrientType                  eOrient2,           // in : and its orientation
   SmTArray<SmCurve*>          & r3DCurves,          // i/o: OrderedEdge List1 3DCurves
   SmTArray<SmBSplineCurve*>   & rUVCurves,          // i/o: OrderedEdge List1 UVCurves
   SmTArray<SmOrientType> &      rOrients,           // i/o: OrderedEdge List1 Orientations
   int                           iDebugLevel)
{
#ifdef SM_DEBUG_CODE
  // prettyPrint UVCurves
  if ( iDebugLevel > 0 )
    {
      pUVCurve1->Dump();
      pUVCurve2->Dump();
    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  // locals
  SmPoint3d        sPnt;
  SmExtent1d       sIvl1    = pUVCurve1->GetNaturalInterval();
  SmExtent1d       sIvl2    = pUVCurve2->GetNaturalInterval();
  double           dTVal1   =  (eOrient1 == SM_OT_OPPOSITE) ? sIvl1.GetMin() : sIvl1.GetMax();
  double           dTVal2   =  (eOrient2 == SM_OT_OPPOSITE) ? sIvl2.GetMin() : sIvl2.GetMax();
  SmExtent2d       sDomain  = pSurface->GetNaturalUVDomain();
  SmBSplineCurve * pUVCurve = NULL ;
  SmBSplineCurve * p3DCurve = NULL ;
  double           dMaxDist = 0.0;

  // let sUV1 = UVTrimCurve1 UV endPoint
  SER(pUVCurve1->EvaluatePoint(dTVal1,sPnt));
  SmPoint2d sUV1(sPnt.x,sPnt.y);

  // let sUV4 = UVTrimCurve2 UV endPoint
  SER(pUVCurve2->EvaluatePoint(dTVal2,sPnt));
  SmPoint2d sUV4(sPnt.x,sPnt.y);

  // let sUV2 = UVTrimCurve1->EndPoint moved to Surface V_Dir domain boundary
  // let sUV3 = UVTrimCurve2->EndPoint moved to Surface V_Dir domain boundary
  SmPoint2d sUV2((eOrient1 == SM_OT_OPPOSITE) ? sDomain.GetUMin() : sDomain.GetUMax(),
                 sUV1.y) ;
  SmPoint2d sUV3(sUV2.x,
                 sUV4.y) ;

  // when UVTrimCurve1->EndPoint is more than tol from Surface V_Dir boundary
  // make a curve to fill the gap
  if (sUV1.DistanceBetween(sUV2) > SM_EFF_ZERO_SQRT)
    {
      // gwc: I reviewed all the prog_test cases that run through this code - seems to give correct results
      // SM_DBG_WARN(_T("Adding Fillet Cap Edge 1, to be reviewed")) ;

      // create a isoParameter UV Line from UVTrimCurve1->EndPoint to the Surface domain boundary
      SER(SmBSplineCurve::CreateLineSegment(crContext,2,SmPoint3d(sUV1.x,sUV1.y,0.0),
          SmPoint3d(sUV2.x,sUV2.y,0.0),pUVCurve));

      // let p3DCurve = SurfaceProjection of new UV Line
      SER(pSurface->LiftCurve(crContext, sDomain,
                              *pUVCurve, pUVCurve->GetNaturalInterval(),
                              dThisApproxTol3d, dMaxDist,p3DCurve));

      // add the curves to the output arrays
      rUVCurves.Add(pUVCurve); pUVCurve = NULL ;
      r3DCurves.Add(p3DCurve); p3DCurve = NULL ;
      rOrients.Add(SM_OT_SAME);

    } // end need to fill UVTrimCurve1->EndPoint to Surface V_Dir boundary gap check

  // when distance between UVTrimCurve boundaryPoints projected to the surfaceDomain edge
  // is greater than tolerance, make a curve to fill the gap
  if (sUV2.DistanceBetween(sUV3) > SM_EFF_ZERO_SQRT)
    {
      // gwc: I reviewed all the prog_test cases that run through this code - seems to give correct results
      // SM_DBG_WARN(_T("Adding Fillet Cap Edge 2, to be reviewed")) ;

      // create a isoParameter UV Line between UVTrimCurve1 and UVTrimCurve2 projections to SurfaceDomainBoundary
      SER(SmBSplineCurve::CreateLineSegment(crContext, 2,
                                            SmPoint3d(sUV2.x,sUV2.y,0.0),
                                            SmPoint3d(sUV3.x,sUV3.y,0.0),
                                            pUVCurve));

      // let p3DCurve = SurfaceProjection of new UV Line
      SER(pSurface->LiftCurve(crContext, sDomain,
                              *pUVCurve, pUVCurve->GetNaturalInterval(),
                              dThisApproxTol3d, dMaxDist, p3DCurve));

      // add the curves to the output arrays
      rUVCurves.Add(pUVCurve); pUVCurve = NULL ;
      r3DCurves.Add(p3DCurve); p3DCurve = NULL ;
      rOrients.Add(SM_OT_SAME);
    }

  // when Surface V_Dir boundary is more than tolerance from UVTrimCurve2->EndPoint
  // make a curve to fill the gap
  if (sUV3.DistanceBetween(sUV4) > SM_EFF_ZERO_SQRT)
    {
      // gwc: I reviewed all the prog_test cases that run through this code - seems to give correct results
      // SM_DBG_WARN(_T("Adding Fillet Cap Edge 3, to be reviewed")) ;

      // create a isoParameter UV Line between UVTrimCurve1 and UVTrimCurve2 projections to SurfaceDomainBoundary
      SER(SmBSplineCurve::CreateLineSegment(crContext, 2,
                                            SmPoint3d(sUV3.x,sUV3.y,0.0),
                                            SmPoint3d(sUV4.x,sUV4.y,0.0),
                                            pUVCurve));

      // let p3DCurve = SurfaceProjection of new UV Line
      SER(pSurface->LiftCurve(crContext, sDomain,
                              *pUVCurve, pUVCurve->GetNaturalInterval(),
                              dThisApproxTol3d, dMaxDist, p3DCurve));

      // add the curves to the output arrays
      rUVCurves.Add(pUVCurve); pUVCurve = NULL ;
      r3DCurves.Add(p3DCurve); p3DCurve = NULL ;
      rOrients.Add(SM_OT_SAME);
    }

  // all done
  return SM_SUCCESS;

} // end JoinVertsByCapping

/*******************************************************************//**
    END - Static functions
***********************************************************************/

/*******************************************************************//**
PURPOSE: Constructor for the SmFilletGeom object.

NOTES:
***********************************************************************/
SmFilletGeom::SmFilletGeom
  (SmFilletSolver *pFilletSolver,
   SmFilletGeomType eType)
: m_eType(eType),
  m_pFilletSolver(pFilletSolver),
  m_pCenterLineCurve(NULL),
  m_pFilletSurface(NULL),
  m_bNeedSplit(FALSE),
  m_eStatus(SM_FIL_UNPROCESSED)
{
    for (ULONG i=0; i<2; i++) {
        m_pOffSurfs[i] = NULL;
        m_vRails[i]    = NULL;
        m_dDeviations[i] = 0.0;

        if ( pFilletSolver != NULL )
        {
            SmOffsetSurface *pOffSrf = pFilletSolver->GetSurface(i);
            if ( pOffSrf != NULL )
            {
                // SmOffsetSurface::Copy() returns an SmSurface*, not SmOffsetSurface*
                SmSurface *pNewSurf = NULL;
                pOffSrf->Copy( pFilletSolver->GetCreationContext(), pNewSurf );
                m_pOffSurfs[i] = SM_CAST_PTR(SmOffsetSurface, pNewSurf);
            }
            else
            { m_pOffSurfs[i] = NULL; }
        }
    }

} // end SmFilletGeom::SmFilletGeom constructor

/*******************************************************************//**
PURPOSE: Constructor for the SmFilletGeom object that copies an
    existing Fillet Geom object.

NOTES: Note that this contructor only works for the case
    where rail edges only are present.
***********************************************************************/
SmFilletGeom::SmFilletGeom
  (SmFilletGeom *pFilletGeomToCopy)
 : m_pCenterLineCurve(NULL),
   m_pFilletSurface(NULL),
   m_bNeedSplit(FALSE),
   m_eStatus(SM_FIL_UNPROCESSED)
{
    SmFilletSolver *pFilletSolver = pFilletGeomToCopy->m_pFilletSolver;
    m_pFilletSolver = pFilletSolver;

    m_pOffSurfs[0] = m_pOffSurfs[1] = NULL;

    // Create Rails
    SmFilletBrep * pBrep = pFilletSolver->m_pExecutive->GetPseudoBrep();
    for (ULONG i=0; i<2; i++) {
        SmFilletEdge * pRail = NULL;
        SE(MakeRailEdge(pBrep, i, pRail));
    }
    SE(Copy(pFilletGeomToCopy));
    m_eType = pFilletGeomToCopy->m_eType;

} // end SmFilletGeom::SmFilletGeom constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmFilletGeom object.

NOTES:
***********************************************************************/
SmFilletGeom::~SmFilletGeom()
{
  // delete centerLine
  if (m_pCenterLineCurve) { delete m_pCenterLineCurve; m_pCenterLineCurve = NULL ; }

  // delete Vertices
  ULONG k;
  for (k=0; k<m_vVertices.GetSize(); k++)
    {
      SM_ASSERT(m_vVertices[k] != NULL) ; delete m_vVertices[k] ; m_vVertices[k] = NULL ;
    }

  // delete offset surfaces
  if ( m_pOffSurfs[0] != NULL ) { delete m_pOffSurfs[0] ; m_pOffSurfs[0] = NULL; }
  if ( m_pOffSurfs[1] != NULL ) { delete m_pOffSurfs[1] ; m_pOffSurfs[1] = NULL; }

} // end SmFilletGeom::~SmFilletGeom destructor

/*******************************************************************//**
PURPOSE: Add a Side Edgeuse.

NOTES:
***********************************************************************/
void SmFilletGeom::AddSideFilletEdgeuse
 (SmFilletEdgeuse * pSideEdgeuse)
{
#ifdef SM_DEBUG_CODE
  SmBSplineCurve *pUVTrimCurve = pSideEdgeuse->GetUVTrimCurve() ;
  if(pUVTrimCurve != NULL && pUVTrimCurve->GetDim() != 2)
    {
      SM_ASSERT_MSG(pUVTrimCurve == NULL || pUVTrimCurve->GetDim() == 2, _T("SmFilletGeom::AddSideFilletEdgeuse passed a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  m_vSideEUs.AddUnique(pSideEdgeuse);

} // end SmFilletGeom::AddSideFilletEdgeuse

/*******************************************************************//**
PURPOSE: Copy geometry from a given pFilletGeomToCopy into this Geom.

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::Copy
  (SmFilletGeom * pFilletGeomToCopy)
{
  // check input
  NER(pFilletGeomToCopy);

  // check state
  if (m_pFilletSolver != pFilletGeomToCopy->m_pFilletSolver)
    { SER(SM_ERR); }

  // locals
  const SmContext & crContext = m_pFilletSolver->m_crContext;

  // copy FilletSurface and CenterLineCurve
  // GWC: Stop Fillets from switching SmCurve and SmSurface objects to SmBSplineCurve and SmBSplineSurface objs
  SmSurface * pCopySurface = NULL ;
  SmCurve   * pCopyCurve   = NULL ;
  if (pFilletGeomToCopy->m_pFilletSurface)   { pFilletGeomToCopy->m_pFilletSurface->Copy  (crContext, pCopySurface  ) ; }      
  if (pFilletGeomToCopy->m_pCenterLineCurve) { pFilletGeomToCopy->m_pCenterLineCurve->Copy(crContext, pCopyCurve) ; }
  m_pFilletSurface   = SM_CAST_PTR(SmBSplineSurface, pCopySurface) ; NER(m_pFilletSurface  ) ;
  m_pCenterLineCurve = SM_CAST_PTR(SmBSplineCurve,   pCopyCurve  ) ; NER(m_pCenterLineCurve) ;

  //   
  //   if (pFilletGeomToCopy->m_pFilletSurface)   { m_pFilletSurface   = new (crContext) SmBSplineSurface(*pFilletGeomToCopy->m_pFilletSurface); }
  //   if (pFilletGeomToCopy->m_pCenterLineCurve) { m_pCenterLineCurve = new (crContext) SmBSplineCurve  (*pFilletGeomToCopy->m_pCenterLineCurve); }

  // for both offset surfaces
  for (ULONG i=0; i<2; i++)
    {
      // Offset surfaces.  Base surfaces are not owned.
      SmOffsetSurface *pOtherOffSurf = pFilletGeomToCopy->GetOffsetSurface( i );

      // when given an OffsetSurface to copy - copy it
      if ( pOtherOffSurf != NULL )
        {
          SmSurface * pNewSurf = NULL;

          // SmOffsetSurface::Copy() returns an SmSurface*, not SmOffsetSurface*
          pOtherOffSurf->Copy( crContext, pNewSurf );
          SmOffsetSurface * pNewOffSurf = SM_CAST_PTR(SmOffsetSurface, pNewSurf);
          if(pNewOffSurf != NULL)
            { m_pOffSurfs[i] = pNewOffSurf; }
        }

      m_dDeviations[i] = pFilletGeomToCopy->m_dDeviations[i];

      // rails
      SmFilletEdge * pRailToCopy = pFilletGeomToCopy->GetRail(i);
      SmFilletEdge * pRail = GetRail(i);

      // Copy Rail parameters
      pRail->SetFilletEdgeType(pRailToCopy->GetFilletEdgeType());
      pRail->SetOriginalFace(pRailToCopy->GetOriginalFace());
      pRail->SetOriginalEdge(pRailToCopy->GetOriginalEdge());

      // Copy 3D curve for new rail
      SmBSplineCurve *pOrig3DCurve = SM_CAST_PTR(SmBSplineCurve,pRailToCopy->GetCurve());
      if (pOrig3DCurve)
        {
          // GWC: Stop Fillets from switching SmCurve and SmSurface objects to SmBSplineCurve and SmBSplineSurface objs
          SmCurve        * pCopyCrv = NULL ;
          SmBSplineCurve * p3DCurve   = NULL ;
          pOrig3DCurve->Copy(crContext, pCopyCrv) ; 
          p3DCurve = SM_CAST_PTR(SmBSplineCurve, pCopyCrv) ; NER(p3DCurve) ; 
          //   
          //   SmBSplineCurve *p3DCurve = new (crContext) SmBSplineCurve(*pOrig3DCurve);
      
          pRail->SetCurve(p3DCurve, TRUE) ;  // TRUE = delete preExisting Curve - don't expect a delete, expect pRail->Curve == NULL
                                             // side effect: delete current pEdge->UVTrimCurves(expect none at this point)
          p3DCurve->SetOwner(pRail);
          SmExtent1d sCrvIvl = p3DCurve->GetNaturalInterval();
          pRail->SetInterval(sCrvIvl);
        }

      // Copy UV curves for new rail
      SmFilletEdgeuse * pEUOrig      = (SmFilletEdgeuse*)pRailToCopy->GetPrimaryEdgeuse();
      SmBSplineCurve  * pOrigUVCurve = pEUOrig->GetUVTrimCurve();
      if (pOrigUVCurve)
        {
          SmBSplineCurve  * pUVCurve = new (crContext) SmBSplineCurve(*pOrigUVCurve);
          SmFilletEdgeuse * pEU      = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
          if (pEU) pEU->SetUVCurve(pUVCurve, TRUE) ; // TRUE = delete preExisting UVTrimCureve - don't expect delete, expect pEU->UVTrimCurve == NULL
        }

      pOrigUVCurve = pEUOrig->GetMate()->GetUVTrimCurve();
      if (pOrigUVCurve)
        {
          SmBSplineCurve  * pUVCurve = new (crContext) SmBSplineCurve(*pOrigUVCurve);
          SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse()->GetMate();
          if (pEU) pEU->SetUVCurve(pUVCurve, TRUE); // TRUE = delete preExisting UVTrimCureve - don't expect delete, expect pEU->UVTrimCurve == NULL
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::Copy

/*******************************************************************//**
PURPOSE: Set offset surface.

NOTES: Deletes what's there, if anything.
***********************************************************************/
SmStatus SmFilletGeom::SetOffsetSurface
 (ULONG lWhichSrf,
  SmOffsetSurface *pNewOffSrf )
{
    if ( lWhichSrf > 1 )
        { SER( SM_ERR_INVALID_INPUT ); }

    if ( m_pOffSurfs[ lWhichSrf ] != NULL )
    {
        delete m_pOffSurfs[ lWhichSrf ];
    }

    m_pOffSurfs[ lWhichSrf ] = pNewOffSrf;
    return SM_SUCCESS;

} // end SmFilletGeom::SetOffsetSurface

/*******************************************************************//**
PURPOSE: Find the tangent of an edge intersecting the point on the
     original brep.

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::FindEdgeTangentAtPoint
  (ULONG        lRailIndex,
   ULONG        lEndIndex,
   SmBoolean  & rbFoundTangent,
   SmVector3d & rTangent)
 const
{
  rbFoundTangent = FALSE;
  SmFilletEdge * pRail = m_vRails[lRailIndex];
  SmFilletVertex * pFV = (SmFilletVertex*)pRail->GetVertex();
  if (lEndIndex != 0)
      pFV = (SmFilletVertex*)pRail->GetOtherVertex(pFV);

  if (pFV->GetPointClass() == SM_PC_EDGE) {
      double dT = pFV->GetOriginalTParam();
      SmVector3d sPV[2];
      SmEdge *pE = (SmEdge*)pFV->GetPointClassObject(); NER(pE);
      SmCurve *pCurve = pE->GetCurve(); NER(pCurve);
      SER(pCurve->Evaluate(dT,1,TRUE,sPV));
      if (sPV[1].LengthSquared() > SM_EFF_ZERO_SQ) {
          sPV[1].Unitize();
          rTangent = sPV[1];
          rbFoundTangent = TRUE;
      }
  }
  else if (pFV->GetPointClass() == SM_PC_VERTEX) {
      // Someday we will look at all edges at this vertex and take
      // the best canidate.  However today we do nothing.
      return SM_SUCCESS;
  }
      return SM_SUCCESS;

} // end SmFilletGeom::FindEdgeTangentAtPoint

/*******************************************************************//**
PURPOSE: Create Face (and Edges and Verts) in pBrep from this 
         m_pFilletSurface with call to pBrep->MakeFaceWithCurves()

NOTES:
  The given input pBrep is always SmFilletExecutive::m_pFilletBrep.
  Face is created using the SmFilletGeom::Surface bounded by an
  ordered sequence of
    the FilletGeom->railCurves,
    the FilletGeom->filletCorner->FilletEdge->Curves,
    and possibly some gap filling short curves created as needed.
  The ordered sequence is found as part of this function's method.

***********************************************************************/
SmStatus SmFilletGeom::MakeFaceBrep
  (SmBrep * pBrep)       // in : target Brep to hold newFace
{
  // check state - must have a FilletSurface
  NER(m_pFilletSurface);

  // locals
  ULONG ii ;
  const SmContext * cpContext   = pBrep->GetContext();
  SmExtent2d        sUVDomain   = m_pFilletSurface->GetNaturalUVDomain();

  // Next: Build Loop EdgeOrder:[ RailMinV - end rib edges - RailMaxV - end rib edges]
  //       Rails can be several edges, if there was rollover.

  // make 1st loop edge the MinV rail curve: set these two vars:
  ULONG     lStartRailIndx = 0;       // 1st loop edge
  SmBoolean bUseUVCurves   = FALSE;   // flag to use UVTrimCurves or 3dCurves
                                      // only FALSE for FilletGeom type == SM_FG_CLIFF_SIDE_PATCH

  // when FilletGeom is not type SM_FG_CLIFF_SIDE_PATCH
  if ( m_eType != SM_FG_CLIFF_SIDE_PATCH )
    {
      // UVTrimCurves are available - work in cheaper UVSpace
      bUseUVCurves = TRUE;

      // See which rail is at low-v.  Check the midpoint of each.
      SmPoint3d sTestUV[2];
      for(ii=0;ii<2;ii++)
        {
          SmFilletEdge   * pThisRail = m_vRails[ii];
          SmEdgeuse      * pPrimEU   = pThisRail->GetPrimaryEdgeuse(); NER(pPrimEU);
          SmBSplineCurve * pUVCurve  = pPrimEU->GetUVTrimCurve();      NER(pUVCurve);
          SER( pUVCurve->EvaluatePoint(pThisRail->GetInterval().Evaluate(0.5), sTestUV[ii] ));
        }

      // make min V rail curve the 1st edge in the loop being built
      //   when lStartRailIndx = 0: Loop EdgeOrder:[Rail0 - end rib edges - Rail1 - end rib edges]
      //                       = 1: Loop EdgeOrder:[Rail1 - end rib edges - Rail0 - end rib edges]
      lStartRailIndx = (sTestUV[0].y < sTestUV[1].y) ? 0 : 1 ;

    } // end setting lStartRailIndx and bUseUVCurves

  // pick 2d or 3d tolerance depending on bUseUVCurve
  SmApproxTol3d sApproxTol3d = m_pFilletSolver->GetThisApproxTol3d();
  double d3dTol = (double)sApproxTol3d;  // 3D Tol
  double dUVTol = d3dTol;
  if ( bUseUVCurves )
  {
      SmPoint2d sUV( sUVDomain.GetMid() );
      SmVector2d sUVDir(1,0); // Use tol in u-direction of fillet surface at its midpoint.
      dUVTol = (double) SmTol::MapTo2d( sApproxTol3d, sUV, sUVDir, *m_pFilletSurface ); // 2D Tol
  }

  // local arrays for
  SmTArray<SmOrientType>        sCurveOrients1, sCurveOrients2, sCurveOrients3, sCurveOrients4;
  SmTArray<SmCurve*>            s3DCurves1,     s3DCurves2,     s3DCurves3,     s3DCurves4 ;
  SmTArray<SmBSplineCurve*>     sUVCurves1,     sUVCurves2,     sUVCurves3,     sUVCurves4 ;
  SmObjsDelete<SmCurve*>        sClean3D3(&s3DCurves3), sClean3D4(&s3DCurves4) ;
  SmObjsDelete<SmBSplineCurve*> sCleanUV3(&sUVCurves3), sCleanUV4(&sUVCurves4) ;

  // Load up s3DCurves1, sUVCurves1, and sCurveOrients1.
  // First the start rail:
  SmEdge                  * pRail0      = m_vRails[ lStartRailIndx ];
  SmEdgeuse               * pPrimEU     = pRail0->GetPrimaryEdgeuse(); NER(pPrimEU);
  SmBSplineCurve          * pUVCurve0   =   bUseUVCurves
                                          ? pPrimEU->GetUVTrimCurve()
                                          : NULL ;
  SmTArray<SmFilletEdge*> * pOtherRails =  (lStartRailIndx == 1)
                                          ? &m_vOtherRails2
                                          : &m_vOtherRails1 ;
  ULONG                     lTotal      = pOtherRails->GetSize();

  // Load orderedEdges list1 with MinV Rail, OrderedEdge List1 =[MinVRail]
  s3DCurves1.Add    ( pRail0->GetCurve() );
  sUVCurves1.Add    ( pUVCurve0  );
  sCurveOrients1.Add( SM_OT_SAME );

  // Get current start & end verts (in the correct order).
  SmVertex * pStartFV0=NULL, * pEndFV0=NULL ;
  if(pPrimEU->GetOrientation() ==  SM_OT_OPPOSITE)     { pEndFV0   = pRail0->GetVertex();
                                                         pStartFV0 = pRail0->GetOtherVertex( pStartFV0 );
                                                       }
  else /* old behavior: SM_OT_SAME or SM_OT_UNKNOWN */ { pStartFV0 = pRail0->GetVertex();
                                                         pEndFV0   = pRail0->GetOtherVertex( pStartFV0 );
                                                       }

  // Next Add MinV OtherRails to OrderedEdges list1, OrderedEdge List1 =[MinVRail(s)]
  // for every additional MinV Rail
  for (ii=0;ii<lTotal;ii++)
    {
      SmEdge * pOtherRail = (*pOtherRails)[ii];
      pPrimEU             = pOtherRail->GetPrimaryEdgeuse(); NER(pPrimEU);

      // add 3DCurve and UVTrimCurves to curve arrays
      s3DCurves1.Add    ( pOtherRail->GetCurve() );
      sUVCurves1.Add    ( pPrimEU->GetUVTrimCurve() );
      sCurveOrients1.Add( SM_OT_SAME );

      // for the last MinV Rail edge - update the pEndFV0 value
      if(ii == lTotal-1)
        {
          SM_ASSERT_MSG(pOtherRail->GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME,
                        _T("SmFilletGeom::MakeFaceBrep: Assumption that m_vOtherRail orientations == SM_OT_SAME not true - this is a bug")) ;
          pEndFV0 = pOtherRail->GetOtherVertex( pOtherRail->GetVertex() );
        }
    } // end iter every otherRails entry

  // Load MaxV rail(s) into OrderedEdges List2
  SmEdge                  * pRail1       = m_vRails[ 1-lStartRailIndx ];
  SmVertex                * pStartFV1    = pRail1->GetVertex();
  SmVertex                * pEndFV1      = pRail1->GetOtherVertex( pStartFV1 );
  SmTArray<SmFilletEdge*> * pOtherRails1 =  (lStartRailIndx == 1)
                                           ? &m_vOtherRails1
                                           : &m_vOtherRails2 ;
  ULONG                     lTotal1      = pOtherRails1->GetSize();

  // For this side go backwards: start with MaxV other rails, backwards, then to the main MaxV rail.
  for(ii=lTotal1;ii>=1;ii--)
    {
      SmEdge * pOtherRail = (*pOtherRails1)[ii-1]; // loop backwards
      pPrimEU = pOtherRail->GetPrimaryEdgeuse(); NER(pPrimEU);

      // add this Other MaxV Edge to the OrderedEdge List2
      s3DCurves2.Add    ( pOtherRail->GetCurve() );
      sUVCurves2.Add    ( pPrimEU->GetUVTrimCurve() );
      sCurveOrients2.Add( SM_OT_OPPOSITE );

      // for the first edge - update the EndVertex value
      if ( ii == 1 )
        {
          SM_ASSERT_MSG(pOtherRail->GetPrimaryEdgeuse()->GetOrientation() == SM_OT_SAME,
                        _T("SmFilletGeom::MakeFaceBrep: Assumption that m_vOtherRail orientations == SM_OT_SAME not true - this is a bug")) ;
          pEndFV1 = pOtherRail->GetOtherVertex( pOtherRail->GetVertex() );
        }
    } // end iter every otherRails1 entry

  // Finally load the MaxV rail into OrderedEdge List2.
  pPrimEU = pRail1->GetPrimaryEdgeuse(); NER(pPrimEU);
  s3DCurves2.Add    ( pRail1->GetCurve() );
  sUVCurves2.Add    ( pPrimEU->GetUVTrimCurve() );
  sCurveOrients2.Add( SM_OT_OPPOSITE );

  // order the MaxV Rail vertices
  if ( pPrimEU->GetOrientation() == SM_OT_OPPOSITE )
    {
      SM_SWAP( SmVertex*, pStartFV1, pEndFV1 );
    }

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  // Add a sequence of m_vSideEUs->Curves to OrderedEdge List1, OrderedEdge List1 =[MinVRail(s) - end rib edges]
  //   to traverse from pEndFV0 to pEndFV1 without creating any gaps.
  SmBoolean bMadeConnection;
  SER( JoinTwoFilletVertsByCurves( this,            // in : target filletGeom
                                   pEndFV0,         // in : Start FilletPoint of FilletEdges which bounds FilletGeom
                                   pEndFV1,         // in : End FilletPoint of FilletEdges which bounds FilletGeom
                                   m_vSideEUs,      // in : Collection of all 'side' FilletEdgeuses of FilletGeom
                                   dUVTol,          // in : UV-Point tolerance of the fillet surface
                                   s3DCurves1,      // i/o: OrderedEdge List1 3DCurves
                                   sUVCurves1,      // i/o: OrderedEdge List1 UVCurves
                                   sCurveOrients1,  // i/o: OrderedEdge List1 Orientations
                                   bMadeConnection, // out: TRUE = Success, m_vSideEUs from Start to End FilletVertex added to OrderedEdgeList 1
                                                    //      FALSE= Failure,
                                   iDebugLevel ));
  SM_ASSERT_MSG(s3DCurves1.GetSize() == sUVCurves1.GetSize(),     _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sUVCurves1 sizes differ after JoinTwoFilletVertsByCurves()")) ;
  SM_ASSERT_MSG(s3DCurves1.GetSize() == sCurveOrients1.GetSize(), _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sCurveOrients1 sizes differ after JoinTwoFilletVertsByCurves()")) ;

  // when railCurveEnd/FilletEdgeStart has too big a gap
  // create and add a set of capEdges to fill it in.
  m_bCapped[1] = FALSE;
  if ( bUseUVCurves && !bMadeConnection )
    {
      m_bCapped[1] = TRUE;

      // check for and fill in gaps between railEdge/filletEdge endPoints
      SER( JoinVertsByCapping(*cpContext,             // in : context for new object construction
                               sApproxTol3d,          // in :
                               m_pFilletSurface,      // in : FilletSurface
                               sUVCurves1.GetLast(),  // in : 1st target UVCurve
                               SM_OT_SAME,            // in : and its orientation
                               sUVCurves2[0],         // in : 2nd target UVCurve
                               SM_OT_SAME,            // in : and its orientation
                               s3DCurves3,            // i/o: OrderedEdge List1 3DCurves
                               sUVCurves3,            // i/o: OrderedEdge List1 UVCurves
                               sCurveOrients3,        // i/o: OrderedEdge List1 Orientations
                               iDebugLevel ));

      // add Gap filling edges to OrderedEdge List 1, OrderedEdge List1 =[MinVRail(s) - end rib edges + gapfillers]
      s3DCurves1.Append( s3DCurves3 );
      sUVCurves1.Append( sUVCurves3 );
      sCurveOrients1.Append( sCurveOrients3 );

      SM_ASSERT_MSG(s3DCurves1.GetSize() == sUVCurves1.GetSize(),     _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sUVCurves1 sizes differ after JoinVertsByCapping()")) ;
      SM_ASSERT_MSG(s3DCurves1.GetSize() == sCurveOrients1.GetSize(), _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sCurveOrients1 sizes differ after JoinVertsByCapping()")) ;
    }

  // Add MaxV rail sequence to growing OrderedEdge List1 =[MinVRail - end rib edges - MaxVRail]
  s3DCurves1.Append( s3DCurves2 );
  sUVCurves1.Append( sUVCurves2 );
  sCurveOrients1.Append( sCurveOrients2 );

  // Add in m_vSideEUs Edge sequence to traverse from pStartFV1 to pStartFV0, OrderedEdge List1 =[MinVRail - end rib edges - MaxVRail - end rib edges]
  SER( JoinTwoFilletVertsByCurves(this,            // in : target filletGeom
                                  pStartFV1,       // in : Start FilletPoint of FilletEdges which bounds FilletGeom
                                  pStartFV0,       // in : End FilletPoint of FilletEdges which bounds FilletGeom
                                  m_vSideEUs,      // in : Collection of all 'side' FilletEdgeuses of FilletGeom
                                  dUVTol,          // in : UV-Point tolerance of the fillet surface
                                  s3DCurves1,      // i/o: OrderedEdge List1 3DCurves
                                  sUVCurves1,      // i/o: OrderedEdge List1 UVCurves
                                  sCurveOrients1,  // i/o: OrderedEdge List1 Orientations
                                  bMadeConnection, // out: TRUE = Success, m_vSideEUs from Start to End FilletVertex added to OrderedEdgeList 1
                                                   //      FALSE= Failure,
                                  iDebugLevel  ));
  SM_ASSERT_MSG(s3DCurves1.GetSize() == sUVCurves1.GetSize(),     _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sUVCurves1 sizes differ after JoinTwoFilletVertsByCurves()")) ;
  SM_ASSERT_MSG(s3DCurves1.GetSize() == sCurveOrients1.GetSize(), _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sCurveOrients1 sizes differ after JoinTwoFilletVertsByCurves()")) ;

  // when railCurveEnd/FilletEdgeStart has too big a gap
  // create and add a set of capEdges to fill it in.
  m_bCapped[0] = FALSE;

  if(bUseUVCurves && !bMadeConnection)
    {
      m_bCapped[0] = TRUE;

      SER( JoinVertsByCapping( *cpContext,            // in : context for new object construction
                                sApproxTol3d,         // in :
                                m_pFilletSurface,     // in : FilletSurface
                                sUVCurves1.GetLast(), // in : 1st target UVCurve
                                SM_OT_OPPOSITE,       // in : and its orientation
                                sUVCurves1[0],        // in : 2nd target UVCurve
                                SM_OT_OPPOSITE,       // in : and its orientation
                                s3DCurves4,           // i/o: OrderedEdge List1 3DCurves
                                sUVCurves4,           // i/o: OrderedEdge List1 UVCurves
                                sCurveOrients4,       // i/o: OrderedEdge List1 Orientations
                                iDebugLevel ));

      // Add gap filling edge sequence to OrderedEdge List1, OrderedEdge List1 =[MinVRail - end rib edges - MaxVRail - end rib edges + gap filling edges]
      s3DCurves1.Append( s3DCurves4 );
      sUVCurves1.Append( sUVCurves4 );
      sCurveOrients1.Append( sCurveOrients4 );

      SM_ASSERT_MSG(s3DCurves1.GetSize() == sUVCurves1.GetSize(),     _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sUVCurves1 sizes differ after JoinVertsByCapping()")) ;
      SM_ASSERT_MSG(s3DCurves1.GetSize() == sCurveOrients1.GetSize(), _T("SmFilletGeom::MakeFaceBrep1 - s3DCurves1 and sCurveOrients1 sizes differ after JoinVertsByCapping()")) ;
    }

  // remove any degerenate curves added to the curve sequence
  for ( long m=s3DCurves1.GetSize()-1; m>=0; m-- )
    {
      SmCurve * pCurve = s3DCurves1[m];
      if ( pCurve->IsDegenerate() )
        {
          s3DCurves1.RemoveAt(m);
          sUVCurves1.RemoveAt(m);
          sCurveOrients1.RemoveAt(m);
        }
    }

  // set the number of curves forming this filletSurfaces boundary loop
  SmTArray<ULONG> sCurveLoops;
  sCurveLoops.Add( s3DCurves1.GetSize() );

  // turn off UseUVCurves if any UVCurve array member is set to NULL
  for ( ULONG k=0; k<sUVCurves1.GetSize(); k++ )
    {
      if ( sUVCurves1[k] == NULL )
        {
          bUseUVCurves = FALSE;
          break;
        }
    }

  // Make a copy of each 3D & UV curve
  for ( ULONG kkk=0; kkk<s3DCurves1.GetSize(); kkk++ )
    {
      SmBSplineCurve *pBSC = SM_CAST_PTR( SmBSplineCurve, s3DCurves1[kkk] );
      NER(pBSC);

      // copy 3DCurve
      const SmContext *pContext = pBrep->GetContext();
      SmCurve * pCopyCurve = NULL ; 
      // GWC: Stop Fillets from switching SmCurve and SmSurface objects to SmBSplineCurve and SmBSplineSurface objs
      pBSC->Copy(*pContext,pCopyCurve ) ;  s3DCurves1[kkk] = SM_CAST_PTR(SmBSplineCurve,pCopyCurve) ; NER(s3DCurves1[kkk]) ; 
      //
      //   s3DCurves1[kkk]           = new (*pContext) SmBSplineCurve(*pBSC);

      // skip copying uvTrimCurves when not using them
      if ( bUseUVCurves )
        {
          SmBSplineCurve *pUVBSC = sUVCurves1[kkk]; NER(pUVBSC);
          sUVCurves1[kkk]        = new (*pContext) SmBSplineCurve(*pUVBSC);
        }
    } // end iter every curve making copies

#ifdef SM_DEBUG_CODE
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  // draw
  if ( iDebugLevel > 0  || lCount == lDebugCount)
    {
      SmVector3d sNorm;
      SmPoint3d  sSurfPnt, sPnt;
      double     sCH = .1 ; 
      SmBoolean  bDrawUVIn3D = TRUE ;
      SmBoolean  bDrawUVIn2D = FALSE ;
      
      SER(m_pFilletSurface->EvaluateNormal(sUVDomain.Evaluate(0.0,0.1),TRUE,TRUE,sNorm));
      SER(m_pFilletSurface->EvaluatePoint(sUVDomain.Evaluate(0.0,0.1),sSurfPnt));

      smgfx_Erase() ;
      sCH = smgfx_SetLook(1,2, 0,0,1, .0001) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1, .0001 ) ; m_pFilletSurface->DrawUV(4,4); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,1) ;         sNorm.Draw(&sSurfPnt); sm_GraphicsLoop() ;

      for (ULONG kk=0; kk<s3DCurves1.GetSize(); kk++)
        {
          if(s3DCurves1[kk] == NULL)                                 { continue; }
          if(sUVCurves1[kk] == NULL && (bDrawUVIn2D || bDrawUVIn3D)) { continue; }

          SmExtent1d sIvl = s3DCurves1[kk]->GetNaturalInterval();
          if (sCurveOrients1[kk] == SM_OT_SAME) { SER(s3DCurves1[kk]->EvaluatePoint(sIvl.Evaluate(0.25), sPnt)); }
          else                                  { SER(s3DCurves1[kk]->EvaluatePoint(sIvl.Evaluate(0.75), sPnt)); }

          smgfx_ChangeColor(kk!=0) ; 
          smgfx_SetLineWidth(4) ; smgfx_SetPointSize(5) ; s3DCurves1[kk]->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLineWidth(7) ; smgfx_SetPointSize(8) ; smgfx_ChangeColor(TRUE) ; 
            if(bDrawUVIn2D) { sUVCurves1[kk]->Draw() ; } sm_GraphicsLoop() ; 
            if(bDrawUVIn3D) { SmCrvOnSurf s3DCrv(*sUVCurves1[kk],*m_pFilletSurface, NULL, 0, pBrep->GetContext());
                              s3DCrv.Draw(); sm_GraphicsLoop();
                            }
          smgfx_SetLook(9,10, 0,0,1); sPnt.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();

        } // end iter every s3DCurves1
      smgfx_SetChordHeight(sCH) ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Make Face
  SmFace * pNewFace = NULL ;
  SmTArray<SmPoint3d> sLoopPoints;

  if ( !bUseUVCurves )
    {
      SER( SmTrimmingTools::TrimSurfaceWithModelSpaceCurves
             (*cpContext,                 // in : new object construction
              pBrep->GetInfiniteRegion(), // in : Target region in existing Brep to receive new trimmed surface.
              d3dTol,                     // in : Tolerance assigned to Brep and passed to SplitAtSeams().
              sCurveLoops,                // in : crCurveLoops.GetSize() = number of loops to make
                                          //      crCurveLoops[i]        = number of curves in ith loop 
              s3DCurves1,                 // in : Curve array ordered by loop and neighbor.  
                                          //      1st loop is outer loop, subsequent optLoops are inner loops. 
                                          //      outer loop orientation = counter clockwise
                                          //      inner loop orientation = clockwise
                                          //      These curves are consumed by this method.
              sCurveOrients1,             // in : SM_OT_SAME     = curve's orientation in its loop is the same as it's parametric orientation
                                          //      SM_OT_OPPOSITE = curve's orientation in its loop is the oppositie of it's parametric orientation
              sLoopPoints,                // in : Points within the outer loop to become vertex loops.
              m_pFilletSurface,           // in : The surface to be trimmed. It's consumed by this operation.
              sUVDomain,                  // in : Domain of the surface used by the face.
              SM_OT_SAME,                 // in : SM_OT_SAME     = face oriented with the surface normal
                                          //      SM_OT_OPPOSITE = face oriented against the surface normal (negates the CurveOrientation requirements)
              TRUE,                       // in : Not used in this function.
              FALSE,                      // in : TRUE = 
                                          //      FALSE= 
              FALSE));                    // in : TRUE =
                                          //      FALSE=
    } // end not using UVTrimCurves branch
  else // using UVTrimCurves branch
    {
      SmRegion *pNewRegion = NULL;
      SmShell  *pNewShell = NULL;

      // make face bounded by sequenced curves
      SER( pBrep->MakeFaceWithCurves
             (pBrep->GetInfiniteRegion(),  // in : region to contain new topology objects
              sCurveLoops,                 // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
              &s3DCurves1,                 // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
              &sUVCurves1,                 // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
              sCurveOrients1,              // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
              sLoopPoints,                 // in : Point positions to build SmVertex VertexLoops
              m_pFilletSurface,            // in : new face->Surface
              sUVDomain,                   // in : domain of Surface used by face
              SM_OT_SAME,                  // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
              pNewRegion,                  // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
              pNewShell,                   // out: New shell if any.  Trimmed surfaces always create a new shell.
              pNewFace));                  // out: the new face
    } // end using UVTrimCurves branch

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 )
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; if(pNewFace) pNewFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; if(pNewFace) pNewFace->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::MakeFaceBrep

/*******************************************************************//**
PURPOSE: Get one of the offset surfaces.

NOTES:
  If this FilletGeom does not have the requested offset surface,
  return our FilletSolver's offset surface.
***********************************************************************/
SmOffsetSurface * SmFilletGeom::GetOffsetSurface
 (ULONG lRailIndex) 
 const
{
  SmOffsetSurface * pRetVal = m_pOffSurfs[lRailIndex];

  if ( pRetVal != NULL ) { return pRetVal; }
  else                   { return m_pFilletSolver->GetSurface(lRailIndex) ; }

} // end SmFilletGeom::GetOffsetSurface

/*******************************************************************//**
PURPOSE: Get the index of the rail of the given side-Edgeuse.

NOTES:
  This method requires that our rails are set.
  If not, this method will defer to our FilletSolver's method.

  Error status returns:
  - input is NULL
  - this FilletGeom has no rails (yet)
  - input is not one of our rail vertexuses
  lIdx is returned 0 on error.
***********************************************************************/
SmStatus SmFilletGeom::GetRailIndexOfSideEU
    (SmEdgeuse *pEU,  // in
    ULONG &lIdx )     // out
  const
{
    NER( pEU );

    lIdx = 0;
    SmFilletEdge * pRail0 = GetRail(0);
    SmFilletEdge * pRail1 = GetRail(1);
    if ( pRail0 == NULL || pRail1 == NULL )
    {
        lIdx = GetFilletSolver()->FindIndexOfRailXSideEdgeuse( pEU );
        return SM_SUCCESS;
    }

    lIdx = 2; // error condition, locally

    SmFace *pSideFace = pEU->GetFace();
    if ( pRail0->GetOriginalFace() == pSideFace )
        lIdx = 0;
    else if ( pRail1->GetOriginalFace() == pSideFace )
        lIdx = 1;

    if ( lIdx < 2 )
        return SM_SUCCESS;

    // Be nice: allow the other side of the given edgeuse
    SmEdgeuse *pRadialEU = pEU->GetRadial();
    NER( pRadialEU );

    pSideFace = pRadialEU->GetFace();
    if ( pRail0->GetOriginalFace() == pSideFace )
        lIdx = 0;
    else if ( pRail1->GetOriginalFace() == pSideFace )
        lIdx = 1;

    if ( lIdx > 1 )
    {
        lIdx = 0;
        return SM_ERR;
    }

    return SM_SUCCESS;

} // end SmFilletGeom::GetRailIndexOfSideEU

/*******************************************************************//**
PURPOSE: Get the index of the rail of the given FilletVertexuse.

NOTES:
  Error status returns:
  - input is NULL
  - input is not one of our rail vertexuses
  lIdx is returned 0 on error.
***********************************************************************/
SmStatus SmFilletGeom::GetRailIndexOfVertexuse
    (SmFilletVertexuse *pFVU,  // in
    ULONG &lIdx )              // out
  const
{
    NER( pFVU );

    lIdx = 2; // error condition
    if ( GetRailVertexuse(0,0) == pFVU || GetRailVertexuse(0,1) == pFVU )
        lIdx = 0;
    else if ( GetRailVertexuse(1,0) == pFVU || GetRailVertexuse(1,1) == pFVU )
        lIdx = 1;

    if ( lIdx < 2 )
        return SM_SUCCESS;

    lIdx = 0;
    return SM_ERR;

} // end SmFilletGeom::GetRailIndexOfVertexuse

/*******************************************************************//**
PURPOSE: Get the index of the rail of the given FilletVertex.

NOTES:
  Error status returns:
  - input is NULL
  - input is not one of our rail vertices
  lIdx is returned 0 on error.
***********************************************************************/
SmStatus SmFilletGeom::GetRailIndexOfVertex
    (SmFilletVertex *pFVtx, // in
    ULONG &lIdx )           // out
  const
{
    NER( pFVtx );

    // Find the FilletVertexuse of this FilletVertex
    SmFilletVertexuse *pFVUseTmp;
    SmFilletVertexuse *pFVUseToUse = NULL;
    ULONG i, j;
    for ( i=0; i<2; i++ )
    {
        for ( j=0; j<2; j++ )
        {
            pFVUseTmp = GetRailVertexuse( i, j );
            if ( pFVUseTmp->GetVertex() == pFVtx )
            {
                pFVUseToUse = pFVUseTmp;
                break;
            }
        }

        if ( pFVUseToUse != NULL )
            break;
    }

    return GetRailIndexOfVertexuse( pFVUseToUse, lIdx );

} // end SmFilletGeom::GetRailIndexOfVertex

/*******************************************************************//**
PURPOSE: Get the vertexuse corresponding the the edgeuse of the given
    rail at the given end.

NOTES:
***********************************************************************/
SmFilletVertex * SmFilletGeom::GetRailVertex
  (ULONG lRailIndex,
   ULONG lEndIndex)
 const
{
    SmFilletVertexuse * pVU = GetRailVertexuse(lRailIndex,lEndIndex);
    SmFilletVertex *pV = (SmFilletVertex*)pVU->GetVertex();
    return pV;

} // end SmFilletGeom::GetRailVertex

/*******************************************************************//**
PURPOSE: Get the vertexuse corresponding the the edgeuse of the given
    rail at the given end.

NOTES:
***********************************************************************/
SmFilletVertexuse * SmFilletGeom::GetRailVertexuse
  (ULONG lRailIndex,
   ULONG lEndIndex)
 const
{
    SM_ASSERT(lRailIndex < 2);
    SM_ASSERT(lEndIndex < 2);
    SmFilletEdge * pRail = m_vRails[lRailIndex];
    SM_ASSERT(pRail != NULL);

    SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
    SM_ASSERT(pEU != NULL);
    if (pEU->GetOrientation() == SM_OT_OPPOSITE) {
        pEU = (SmFilletEdgeuse*)pEU->GetMate();
    }
    if (lEndIndex != 0) {
        pEU = (SmFilletEdgeuse*)pEU->GetMate();
    }
    SmFilletVertexuse * pVU = (SmFilletVertexuse*)pEU->GetVertexuse();
    return pVU;

} // end SmFilletGeom::GetRailVertexuse

/*******************************************************************//**
PURPOSE: Set values of a corner FilletVertex.  Set 3d point,
    vertex type, optionally PointClassification, and status (to Processed).
    Also set the uv value in the FilletVertexUse.

NOTES:
    The corner to be set is indicated by lRailIndex and lEndIndex.
***********************************************************************/
SmStatus SmFilletGeom::SetUpRailPoint
 (ULONG                   lRailIndex,     // in : 0 or 1
  ULONG                   lEndIndex,      // in : 0 (start) or 1 (end)
  double                  dParameter,     // in : param along rail == u-param of fillet surface
                                          //      of this corner point
  SmPointClassification * pOptPointClass) // in :
{
  SmFilletVertexuse * pVU         = GetRailVertexuse(lRailIndex,lEndIndex);
  SmFilletVertex    * pFilletVert = (SmFilletVertex*)pVU->GetVertex();

  if(pFilletVert->IsProcessed())
    {
      return SM_SUCCESS;
    }

  SmFilletEdge * pRail      = GetRail(lRailIndex);
  SmCurve      * pRailCurve = pRail->GetCurve();
  SmPoint3d      s3DPnt;
  SER(pRailCurve->EvaluatePoint( dParameter, s3DPnt ));

  pFilletVert->SetFilletVertexType(SM_FV_UNKNOWN);
  if(pOptPointClass != NULL) 
    {
      pFilletVert->CopyPointClass(*pOptPointClass);

      if (pOptPointClass->GetPointClass() == SM_PC_EDGE) 
        { pFilletVert->SetFilletVertexType(SM_FV_RAIL_X_EDGEUSE); }

      if (pOptPointClass->GetPointClass() == SM_PC_VERTEX) 
        { pFilletVert->SetFilletVertexType(SM_FV_RAIL_X_VERTEX); }
    }

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      pRailCurve->Dump();

      smgfx_SetColor(0,0,1); pRail->Draw(); sm_GraphicsLoop();
      smgfx_SetColor(1,0,0); pFilletVert->Draw(); sm_GraphicsLoop();
      smgfx_SetColor(1,0,1); s3DPnt.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  pFilletVert->SetPoint(s3DPnt); //cbi: untrimmed pt: incorrect.

  // Get UV value (in the fillet surface) for this corner vertexuse
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface *pFilletSurface = GetFilletSurface();
  SM_FILLETSURF_TYPE *pFilletSurface = GetFilletSurface();
  NER(pFilletSurface);

  SmExtent2d sUVDomain = pFilletSurface->GetNaturalUVDomain();
  SmPoint2d  sUV(sUVDomain.Evaluate( (double)lEndIndex, (double)lRailIndex) );
  sUV.x = dParameter;

  pVU->SetUVPoint(sUV);

  pFilletVert->SetStatus(SM_FIL_PROCESSED);

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::SetUpRailPoint

/*******************************************************************//**
PURPOSE: Make a rail edge including its end vertices.

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::MakeRailEdge
  (SmFilletBrep  * pPseudoBrep,   // in :
   ULONG           lRailIndex,    // in :
   SmFilletEdge *& rpNewRail)     // out:
{
    // allocate new SmFilletEdge
    SmFilletEdge * pRail = new (pPseudoBrep) SmFilletEdge(pPseudoBrep);
    NER(pRail); SmObjDelete sClean1(pRail);
    pRail->SetFilletEdgeType(SM_FE_RAIL);

    // allocate 2 new SmFilletEdgeuses
    SmFilletEdgeuse *pNewEU1 = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
    NER(pNewEU1); SmObjDelete sClean2(pNewEU1);

    SmFilletEdgeuse *pNewEU2 = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
    NER(pNewEU2); SmObjDelete sClean3(pNewEU2);

    // allocate 2 new SmFilletVertexuses
    SmFilletVertexuse *pNewVU1 = new (pPseudoBrep) SmFilletVertexuse();
    NER(pNewVU1); SmObjDelete sClean4(pNewVU1);
    pNewVU1->SetProperty(pNewEU1);

    SmFilletVertexuse *pNewVU2 = new (pPseudoBrep) SmFilletVertexuse();
    NER(pNewVU2); SmObjDelete sClean5(pNewVU2);

    // allocate 2 new SmFilletVertices
    SmFilletVertex * pStartVert = new (pPseudoBrep) SmFilletVertex();
    NER(pStartVert); SmObjDelete sClean6(pStartVert);

    SmFilletVertex * pEndVert = new (pPseudoBrep) SmFilletVertex();
    NER(pEndVert); SmObjDelete sClean7(pEndVert);

     // Put VU1 into start vertex list and VU2 into end vertex
    SER(pStartVert->PostInsert(pNewVU1));
    SER(pEndVert->PostInsert(pNewVU2));
    pNewVU2->SetProperty(pNewEU2);
    SER(pRail->PreInsert(pNewEU1));
    SER(pRail->PostInsert(pNewEU2));
    pNewEU1->SetFilletVertexuse(pNewVU1);
    pNewEU2->SetFilletVertexuse(pNewVU2);

    // Clear cleanup objects
    sClean1.Clear(); sClean2.Clear(); sClean3.Clear();
    sClean4.Clear(); sClean5.Clear(); sClean6.Clear();
    sClean7.Clear();

    rpNewRail = pRail;
    SetRail(lRailIndex, pRail);
    m_vVertices.Add(pStartVert);
    m_vVertices.Add(pEndVert);

    return SM_SUCCESS;

} // end SmFilletGeom::MakeRailEdge

/*******************************************************************//**
PURPOSE: Make a FilletEdgeuse and a FilletVertexuse to connect a given
            filletVertex to one of this FilletGeom's Fillet Rails.
            The Fillet Rail can be specified either by
              - a railIndex (this->m_pFilletSolver->GetRail(RailIndex)),
              - an explicit SmFilletEdge pointer (pOptBelongingRail), or
              - by making a new one in this function.
            If the function makes a new SmFilletEdge, that SmFilletEdge will
            have no curve data.

NOTES: In general, orientation of edgeuse (optional) need not be
    specified except for the 'closed-rail' cases. User may assign SM_OT_SAME
    for the first edgeuse and SM_OT_OPPOSITE for the second edgeuse.

METHOD ---
  1. Make a new FilletEdgeuse and FilletVertexuse
  2. set new FilletEdgeuse and FilletVertexuse pointers to point at one another and to this SmFilletGeom
  3. Get the input target rail edge, pRailE
     either: pOptBelongingRail,
             GetRail(lRailIndex), or
             new SmFilletEdge
  4. Set topology pointers
        pRailE->m_pOrigFace     = Face being filleted (in case pRailE is newly constructed)
        Edgeuse->m_eOrientation = eOrient or determine SM_OT_SAME/SM_OT_OPPOSITE
  5. Add NewEdgeuse to pRailE SmOwningTopology's object list
     5a. if NewEdgeuse->m_eOrientation == SM_OT_SAME,     Add to beginning of list
     5b. if NewEdgeuse->m_eOrientation == SM_OT_OPPOSITE, Add to end       of list

SIDE EFFECTS ---
  1. Add new FilletVertexuse to input pFilletVertex vertexuses list
  2. Connect new Vertexuse to newEdgeuse
  3. set new FilletEdgeuse->m_pOrientation
  4. Add new FilletEdgeuse to targetRailEdge Edgeuses list

***********************************************************************/
SmStatus SmFilletGeom::MakeRailEdgeuse   // eff: connect 1 of 2 edge RailCurves to given filletVertex
  (SmFilletBrep     * pPseudoBrep,       // in : Brep recieving the new topology objects
   ULONG              lRailIndex,        // in : target railCurve being connected to filletVertex,
                                         //      (when pOptBelongingRail == NULL also indicates starting rail)
   SmFilletVertex   * pFilletVertex,     // in : target FilletVertex being connected to railCurve
   SmFilletEdgeuse *& rpNewEdgeuse,      // out: the new FilletEdgeuse - connected to a new FilletVertexuse connected to FilletVertex
   SmOrientType       eOrient,           // in : orientation to give new edgeuse
   SmFilletEdge     * pOptBelongingRail) // in : target rail edge, NULL to ignore
{
  // Check input
  SmFilletSolver * pFilletSolver = GetFilletSolver();
  NER(pFilletSolver);
  NER(pFilletVertex);

  // 1. allocate new FilletEdgeuse->FilletVertexuse pair
  SmFilletEdgeuse *pNewEU = new (pPseudoBrep) SmFilletEdgeuse(pPseudoBrep);
  NER(pNewEU); SmObjDelete sClean3(pNewEU);
  SmFilletVertexuse *pNewVU = new (pPseudoBrep) SmFilletVertexuse();
  NER(pNewVU); SmObjDelete sClean2(pNewVU);
  pNewVU->SetProperty(pNewEU);

  // 2. connect filletVertexuse to filletEdgeuse -
  //    - add filletVertexuse to filletVertex vertexuse list
  //    - set new FilletEdgeuse owner to this FilletGeom
  pNewEU->SetFilletVertexuse(pNewVU);
  SER(pFilletVertex->PostInsert(pNewVU));
  pNewEU->SetFilletGeom(this);

  // 3. Get or make rail FilletEdge - either pOptBelongingRail,
  //                                         GetRail(lRailIndex), or
  //                                        new SmFilletEdge
  SmFilletEdge * pRailE = pOptBelongingRail;
  if (pRailE == NULL)
    {
      pRailE = GetRail(lRailIndex);
    }
  SmObjDelete sClean1;
  if (pRailE == NULL)
    {
      // Create a new one
      pRailE = new (pPseudoBrep) SmFilletEdge(pPseudoBrep);
      NER(pRailE); sClean1.SetObj(pRailE);
      pRailE->SetFilletEdgeType(SM_FE_RAIL);
      SetRail(lRailIndex, pRailE);
    }

  // 4. set topology pointers
  // set pRailE->m_pOrigFace = Face being filleted (stored in pFilletSolver)
  // Note, this is already set, unless we made a new Edge.
  SmEdgeuse *pBaseEU = pFilletSolver->GetEdgeuse( lRailIndex );
  NER(pBaseEU);
  pRailE->SetOriginalFace( pBaseEU->GetFace() );

  // Set srf ptrs in FilletVertexuse's TsectPnt.
  SmTsectPnt &rTsect = pNewVU->GetTsectPnt();

  SM_ASSERT( rTsect.m_apUserPointer[0] == NULL );
  SM_ASSERT( rTsect.m_apUserPointer[1] == NULL );

  SmSurface *pSrfPtr = pBaseEU->GetFace()->GetSurface();
  rTsect.m_apUserPointer[ lRailIndex ] = (void*)pSrfPtr;

  SmEdgeuse *pBaseEU1 = pFilletSolver->GetEdgeuse( 1-lRailIndex );
  pSrfPtr = pBaseEU1->GetFace()->GetSurface();
  rTsect.m_apUserPointer[ 1-lRailIndex ] = (void*)pSrfPtr;

  // set new Edgeuse->m_eOrientation
  if (eOrient != SM_OT_UNKNOWN)
    {
      pNewEU->SetOrientation(eOrient);
    } // end eOrient given branch
  else
    { // eOrient == SM_OT_UNKNOWN
      pNewEU->SetOrientation(SM_OT_SAME);

      // when edge is not closed
      SmVertex *pBaseV = pBaseEU->GetVertexuse()->GetVertex();
      if (pBaseV != pBaseEU->GetMate()->GetVertexuse()->GetVertex())
        {
          // Non-closed rails
          // Get the original vertex corresponding to the pFilletVertex.
          SmFilletCorner * pCorner = pFilletVertex->GetFilletCorner();
          const SmVertex *pOrigV = pCorner->GetFilletedVertex();
          // set the newEU Orientation to match the how the target rail's
          // current edgeuse connects to the pOrigV vertex.
          if (   (pBaseEU->GetOrientation() == SM_OT_SAME     && pOrigV != pBaseV)
              || (pBaseEU->GetOrientation() == SM_OT_OPPOSITE && pOrigV == pBaseV))
            {
              pNewEU->SetOrientation(SM_OT_OPPOSITE);
            }
        } // end no closed check
    } // end eOrient == SM_OT_UNKNOWN branch

  // 5. Put FEU into rail edge as the mate of the pRailE
  // The first EU in the Edge's list should be SAME, the second OPP.

  if ( pNewEU->GetOrientation() == SM_OT_SAME )
    {
      SER( pRailE->PreInsert( pNewEU ));
    }
  else
    {
      SER( pRailE->PostInsert( pNewEU ));
    }

  // Clear cleanup objects
  sClean1.Clear(); sClean2.Clear(); sClean3.Clear();
  rpNewEdgeuse = pNewEU;

  return SM_SUCCESS;

} // end SmFilletGeom::MakeRailEdgeuse

/*******************************************************************//**
PURPOSE: Refine the fillet point using a solver based technique to
    get an accurate intersection with the fillet surface.

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::RefineFilletPoint
  (ULONG lRailIndex,      // NotUsed: in :
   ULONG lEndIndex)       // NotUsed: in :
{
  SM_REF2(lRailIndex, lEndIndex) ;
    SER(SM_ERR);
    return SM_SUCCESS;

} // end SmFilletGeom::RefineFilletPoint

/*******************************************************************//**
PURPOSE: Create an end curve given the trimming type and index of
        the end.

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::CreateEndCurve
  (ULONG                   lEndIndex,
   SmBoundaryTrimmingType  eTrimType,
   SmFilletBrep          * pPseudoBrep)
{
  // Create a line between the two UV points and lift the curve.
  SmSurface         * pFilletSurface = GetFilletSurface(); NER(pFilletSurface);
  SmEdgeuse         * pEU0           =  (lEndIndex == 0) ? m_vRails[0]->GetPrimaryEdgeuse()
                                                         : m_vRails[0]->GetPrimaryEdgeuse()->GetMate();
  SmEdgeuse         * pEU1           =  (lEndIndex == 0) ? m_vRails[1]->GetPrimaryEdgeuse()
                                                         : m_vRails[1]->GetPrimaryEdgeuse()->GetMate();
  SmFilletVertexuse * pVU0           = (SmFilletVertexuse*)pEU0->GetVertexuse();
  SmFilletVertexuse * pVU1           = (SmFilletVertexuse*)pEU1->GetVertexuse();
  SmFilletVertex    * pFVStart       = (SmFilletVertex*)pVU0->GetVertex();
  SmFilletVertex    * pFVEnd         = (SmFilletVertex*)pVU1->GetVertex();
  SmPoint2d           sPStart        = pVU0->GetUVPoint();
  SmPoint2d           sPEnd          = pVU1->GetUVPoint();
  SmBSplineCurve    * pUVCurve       = NULL ;

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 )
    {
      smgfx_SetLook(1,2, 0,0,1); pFVStart->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); pFVEnd->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,1); m_vRails[0]->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,1); m_vRails[1]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when eTrimType == BLEND
  if (eTrimType == SM_BT_BLEND)
    {
      SmVector2d sUVTan1(0,1), sUVTan2(0,1);
      SmBoolean  bFound1,      bFound2;
      SmVector3d sTangent1,    sTangent2;

      double     dDist = pFVStart->GetPoint().DistanceBetween(pFVEnd->GetPoint());

      // Find Edge Tangents
      SER(FindEdgeTangentAtPoint(0,lEndIndex,bFound1,sTangent1));
      SER(FindEdgeTangentAtPoint(1,lEndIndex,bFound2,sTangent2));

      // when 1st tangent was found
      if (bFound1)
        {
          sTangent1.Unitize();
          sTangent1 = sTangent1 * dDist;
          SER(pFilletSurface->DropVectors(sPStart,TRUE,TRUE,1,&sTangent1,&sUVTan1));
          if (sUVTan1.y < 0.0) { sUVTan1 = - sUVTan1; }
          if (sUVTan1.y < 0.1) { sUVTan1.x = 0.0; // If angle is too small then difficult to blend just skip it
                                 sUVTan1.y = 1.0;
                               }
        } // end bFound1 existence check

      // when second tangent was found
      if (bFound2)
        {
          sTangent2.Unitize();
          sTangent2 = sTangent2 * dDist;
          SER(pFilletSurface->DropVectors(sPEnd,TRUE,TRUE,1,&sTangent2,&sUVTan2));
          if (sUVTan2.y < 0.0) { sUVTan2 = - sUVTan2; }
          if (sUVTan2.y < 0.1) { sUVTan2.x = 0.0; // If angle is too small then difficult to blend just skip it
                                 sUVTan2.y = 1.0;
                               }
        } // end bFound2 existence check

      // when neither tangent was found - just make a line segment
      if (bFound1 == FALSE && bFound2 == FALSE)
        {
          SER(SmBSplineCurve::CreateLineSegment(m_pFilletSolver->m_crContext,2,
                                                SmPoint3d(sPStart),
                                                SmPoint3d(sPEnd),
                                                pUVCurve));
          NER(pUVCurve);
        }
      else // some tangent was found - make a hermite curve between the end points
        {
          sUVTan1 = m_pFilletSolver->m_dEdgeBlendFactor * sUVTan1 * 1.5;
          sUVTan2 = m_pFilletSolver->m_dEdgeBlendFactor * sUVTan2 * 1.5;

          // Now create a hermite curve in UV
          SmHermiteCurve sHerm(sPStart,sUVTan1,sPEnd,sUVTan2,2);
          sHerm.SetContext(NULL);
          SmTArray<double> sBreaks;
          sBreaks.Add(0.0); sBreaks.Add(1.0);
          double dAchievedTol;
          SER(sHerm.ApproximateCurve(m_pFilletSolver->m_crContext,
                                     sBreaks,
                                     m_pFilletSolver->GetThisApproxTol3d()/10.0,dAchievedTol,
                                     pUVCurve, // out: built with Dim == 3
                                     TRUE,     // in : bOptCreateAnalytics
                                     FALSE,    // in : bOptMatchParameterization
                                     FALSE)) ; // in : bJustCopyBSplines
          if(pUVCurve)
            { pUVCurve->ConvertTo2D() ; }

        } // end found a tangent branch
    } // end eTrimType == SM_BT_BLEND branch
  else // eTrimType != SM_BT_BLEND
    {
      // just make a line segment
      SER(SmBSplineCurve::CreateLineSegment(m_pFilletSolver->m_crContext,2,
                                            SmPoint3d(sPStart),
                                            SmPoint3d(sPEnd),
                                            pUVCurve));
      NER(pUVCurve);

    } // end eTrimType != SM_BT_BLEND branch

  // arrive here once pUVCurve has been made
  // next - Lift pUVCurve to p3DCurve
  SmObjDelete sClean(pUVCurve);

  double dMaxDistToSurface = 0.0;
  SmBSplineCurve *p3DCurve = NULL ;
  SER(pFilletSurface->LiftCurve(m_pFilletSolver->m_crContext,               // in : context for new object construction
                                pFilletSurface->GetNaturalUVDomain(),       // in : this surface limit
                               *pUVCurve,                                   // in : 2d Parameter curve defined in surface parameter space to lif
                                pUVCurve->GetNaturalInterval(),             // in : curve segment to lift
                                m_pFilletSolver->GetThisApproxTol3d()/4.0,  // in : max allowed distance between output curve and Surface
                                dMaxDistToSurface,                          // out: max dist from output curve to surface
                                p3DCurve));                                 // out: 3d Curve = Surface(crUVCurveToLift(crInterval))
                                                                            // in : FALSE= check Surface for C0 discontinuities -
                                                                            //             break lifted curve at each such point
                                                                            //      default:[FALSE]
  NER(p3DCurve);
  SmObjDelete sClean2(p3DCurve);

  // make pFilletEdge in pPseudoBrep
  SmFilletEdge *pFilletEdge = NULL;
  SER(SmFilletEdge::MakeFilletEdge(pPseudoBrep,   // in : target Brep to receive new topology objects
                                   pFVStart,      // in : start of new FilletEdge
                                   pFVEnd,        // in : end   of new FilletEdge
                                   pFilletEdge)); // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                                  // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                  //      NULL to ignore, default:[NULL]
  NER(pFilletEdge);
  SmObjDelete sClean3(pFilletEdge);

  // Set pFilletEdge values: FilletEdgeType, Curve, Interval, Edgeuse->UVTrimCurve
  pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
  sClean2.Clear();
  pFilletEdge->SetCurve(p3DCurve, TRUE) ; // TRUE = delete preExisting Curve - don't expect a delete, expect pFilletEdge->Curve == NULL
                                          // side effect: delete current pFilletEdge->UVTrimCurves(expect none at this point)
  p3DCurve->SetOwner(pFilletEdge);
  SmExtent1d sCrvIvl = p3DCurve->GetNaturalInterval();
  pFilletEdge->SetInterval(sCrvIvl);
  sClean.Clear();

  SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
  pEU->SetUVCurve(pUVCurve);
  sClean3.Clear();

  // save results in SmFilletGeom arrays
  m_vSideEUs.Add(pEU);
  m_vEdges.Add(pFilletEdge);

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::CreateEndCurve

/*******************************************************************//**
PURPOSE: Trim this FilletGeom to the indicated interval.

NOTES:
    Question: m_bCapped: if we're chopping off one or both ends,
    then that end would presumably not need capping.
    Should we check that?
***********************************************************************/
SmStatus SmFilletGeom::Trim( SmExtent1d &rTrimInterval )
{
  SmExtent1d sTempIvl;

  // Rails:
  SmCurve *pRailCurve = m_vRails[0]->GetCurve();
  if ( pRailCurve != NULL )
    {
      pRailCurve->Trim( rTrimInterval );   // may snap sIvl by tol to existing knots
      m_vRails[0]->GetInterval().Intersect( rTrimInterval, sTempIvl );
      m_vRails[0]->SetInterval( sTempIvl );
    }

  pRailCurve = m_vRails[1]->GetCurve();
  if ( pRailCurve != NULL )
    {
      pRailCurve->Trim( rTrimInterval );  // may snap sIvl by tol to existing knots
      m_vRails[1]->GetInterval().Intersect( rTrimInterval, sTempIvl );
      m_vRails[1]->SetInterval( sTempIvl );
    }

  // OtherRails:
  ULONG i;
  for ( i = 0; i < m_vOtherRails1.GetSize(); i++ )
    {
      pRailCurve = m_vOtherRails1[i]->GetCurve();
      if ( pRailCurve != NULL )
        {
          pRailCurve->Trim( rTrimInterval );  // may snap sIvl by tol to existing knots
          m_vOtherRails1[i]->GetInterval().Intersect( rTrimInterval, sTempIvl );
          m_vOtherRails1[i]->SetInterval( sTempIvl );
        }
    }
  for ( i = 0; i < m_vOtherRails2.GetSize(); i++ )
    {
      pRailCurve = m_vOtherRails2[i]->GetCurve();
      if ( pRailCurve != NULL )
        {
          pRailCurve->Trim( rTrimInterval ); // may snap sIvl by tol to existing knots
          m_vOtherRails2[i]->GetInterval().Intersect( rTrimInterval, sTempIvl );
          m_vOtherRails2[i]->SetInterval( sTempIvl );
        }
    }

  // Spine curve:
  if ( m_pCenterLineCurve != NULL )
    {
      m_pCenterLineCurve->Trim( rTrimInterval );  // may snap sIvl by tol to existing knots
    }

  // Fillet Surface:
  if ( m_pFilletSurface != NULL )
    {
      SmExtent2d sDomain = m_pFilletSurface->GetNaturalUVDomain();
      // Trimming is in the u-direction.
      SmExtent1d sUDomain = sDomain.GetUInterval();
      sUDomain.Intersect( rTrimInterval, sTempIvl );
      sDomain.SetUInterval( sTempIvl );

      // trim to domain
      //   GWC: The trimmed geometry will not change it's shape and
      //        I think that since the m_pFilletSurface is a BSplineSurface and not an Analytic
      //        that its parameterization will stay constant as well.  If that turns out
      //        not to be true, than callers of this function will need to delete
      //        and rebuild any UVTrimCurves associated with this surface.
      m_pFilletSurface->TrimWithDomain( sDomain );
    }

  return SM_SUCCESS;

} //end SmFilletGeom::Trim

/*******************************************************************//**
PURPOSE: Trim rail curves by the rail ends

NOTES: This function is called after
  1. filletCorner->FilletVertex->Positions are set
  2. FilletGeom->FilletSurface->Shape      is set
  3. FilletGeom->FilletRails->Shapes       are set
  4. FilletCorner->FilletEdge->Shapes      are set

  for side effects:
   for both rail curves
    1. Create V isoParameter UVTrimCurve line and store it as rail->PrimaryEdgeuse->UVTrimCurve
    2. Set rail->edgeuse->FilletVertexuse->UV FilletSurface points
    3. When Rail->endVertex->Points differ by more than tolerance
         Trim 3DCurve and both UVTrimCurves to interval
           defined by dropping endPoint vertices onto rail->Curve
       Else
         when rail is self-intersecting in 1 place
           trim rail to nonSelf-intersecting interval
         else
           case not handled yet
***********************************************************************/
SmStatus SmFilletGeom::TrimRailCurves
  ()
{
  // handle CliffRollover separately
  if (m_eType == SM_FG_CLIFF_ROLLOVER)
    {
      SER(TrimCliffRails());
      return SM_SUCCESS;
    }

  // check state
  NER(m_pFilletSurface);

  // locals: FilletSurface Natural domain and current tolerance
  SmExtent2d sUVDomain = m_pFilletSurface->GetNaturalUVDomain();
  double     dAppTol   = m_pFilletSolver->GetThisApproxTol3d();

  // Trim rail curves by their end FilletVerts

  // for both rail curves
  for (ULONG ii=0; ii<2; ii++)
    {
      // get Rail, rail->edgeuses, rail->vertices
      SmFilletEdge      * pRail   = GetRail(ii);
      SmFilletEdgeuse   * pPrimEU = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
      SmFilletEdgeuse   * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
      SmFilletVertexuse * pVU0    = (SmFilletVertexuse*)pPrimEU->GetVertexuse();
      SmFilletVertexuse * pVU1    = (SmFilletVertexuse*)pMateEU->GetVertexuse();
      SmFilletVertex    * pV0     = (SmFilletVertex*)pVU0->GetVertex();
      SmFilletVertex    * pV1     = (SmFilletVertex*)pVU1->GetVertex();

      // get rail->vertex->points
      // gwc: it's okay for a vertex to be unprocessed - some are status = SM_FV_NO_INT_RAIL_EU,
      // SM_ASSERT(pV0->IsProcessed() && pV1->IsProcessed()) ;  // gwc:test
      SmPoint3d sPnt0(-1000,9999,2000);
      if (pV0->IsProcessed()) { sPnt0 = pV0->GetPoint(); }
      SmPoint3d sPnt1(3456,8901,4521);
      if (pV1->IsProcessed()) { sPnt1 = pV1->GetPoint(); }

      // get rail->Curve
      SmBSplineCurve * p3DRailCurve = SM_CAST_PTR(SmBSplineCurve,pRail->GetCurve());
      NER(p3DRailCurve);

      // init Trim interval = 3DCurve->NaturalInterval
      SmExtent1d sTrimIvl = p3DRailCurve->GetNaturalInterval();

#ifdef SM_DEBUG_CODE
      // draw p3DRailCurve(red), 3DCurveSamplePoint(red), Vertex->Points(green), FilletSurface(black)
      if ( DebugLevel() > 0 )
        {
        if ( DebugLevel() > 5 )
          {
            m_pFilletSurface->Dump();
            p3DRailCurve->Dump();
          }

          smgfx_Erase();

          smgfx_SetLook(1,2, 1,0,0); p3DRailCurve->DrawWDeriv(sTrimIvl); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,0,1); p3DRailCurve->DrawAt(sTrimIvl.Evaluate(0.1),0); sm_GraphicsLoop();

          smgfx_SetLook(3,4, 0,1,0); if (pV0->IsProcessed()) sPnt0.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,1,0); if (pV1->IsProcessed()) sPnt1.Draw(); sm_GraphicsLoop();

          smgfx_SetLook(1,1, 0,0,1); m_pFilletSurface->DrawUV(0,0); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // make iso-V UV line and store it as rail->PrimaryEdgeuse->UVTrimCurve4
      double dIsoV;
      SmPoint2d sTestUV;

      // First, a check, to make sure that the parameterization of the rail
      // curve matches that of the fillet surface.
      // This test has the side effect of setting dIsoV and sTestUV.y
      // (the same value), which is one or the other rail of the
      // fillet surface (v = 0 or 1).

      // Pick a point out in the middle.
      double dParam = sTrimIvl.Evaluate(0.34567);
      SmPoint3d sCurvePnt,sSurfPnt;
      SER(p3DRailCurve->EvaluatePoint(dParam,sCurvePnt));

      // These two will be used later:
      dIsoV = sUVDomain.GetMin().y;
      sTestUV.Set(dParam,dIsoV);

      SER(m_pFilletSurface->EvaluatePoint(sTestUV,sSurfPnt));

      // If the distance between points on the rail curve and on the
      // fillet surface is bigger than app tol, check the other side
      // of the fillet surface.
      if ( sCurvePnt.DistanceBetween(sSurfPnt) > dAppTol )
        {
          // check the other edge of the fillet surface
          dIsoV     = sUVDomain.GetMax().y;
          sTestUV.y = dIsoV;
          SER(m_pFilletSurface->EvaluatePoint(sTestUV,sSurfPnt));

          // when surfaceMaxVPoint/CurvePoint distance is larger than AppTol
          if (sCurvePnt.DistanceBetween(sSurfPnt) > dAppTol)
            {
              // Rail is not syncronizing with the parametrization
              // of the fillet surface
              SER(SM_ERR);
            }
        }

      // Ok, they're in sync; proceed.

      // set rail->PrimaryEdgeuse->UVTrimCurve endPoint values
      SmPoint2d sStartUV = SmPoint2d(sTrimIvl.GetMin(),sTestUV.y);
      SmPoint2d sEndUV   = SmPoint2d(sTrimIvl.GetMax(),sTestUV.y);

      // clean out existing primaryEdgeuse->UVTrimCurve
      SmBSplineCurve *pUVCurve = pPrimEU->GetUVTrimCurvePointer();
      if (pUVCurve != NULL) { delete pUVCurve ; pUVCurve = NULL ; }
      const SmContext & crContext = m_pFilletSolver->GetCreationContext();

      // Create filletSurface UV-Min/Max Line for this rail->PrimaryEdgeuse->UVTrimCurve
      SER(SmBSplineCurve::CreateLineSegment(crContext,
                                            2,
                                            SmPoint3d(sStartUV),
                                            SmPoint3d(sEndUV),
                                            pUVCurve));
      NER(pUVCurve);
      SER(pUVCurve->EditParameterization(sTrimIvl));

      // Save UV curve in rail->PrimaryEdgeuse->UVTrimCurve
      pPrimEU->SetUVCurve(pUVCurve);

      // when the two trim points differ by more than tolerance
      double dParam1 = sTrimIvl.GetMin();
      double dParam2 = sTrimIvl.GetMax();

      // Note: for the start vertex, we DropPoint(), but for the end,
      // we do a global solve and check the number of solutions,
      // and then check for a closed curve.  Are we assuming the start
      // vertex will be out in the middle of the curve, but the end
      // vertex might be at a seam?  [bd 30 Nov 05]

      if (sPnt0.DistanceBetween(sPnt1) > dAppTol)
        {
          // When V0 is processed
          double dDist;
          SmBoolean bSuccess = FALSE;
          // gwc: it's okay for a vertex to be unprocessed - some are status = SM_FV_NO_INT_RAIL_EU,
          // SM_ASSERT(pV0->IsProcessed()) ; // gwc:test
          if (pV0->IsProcessed())
            {
              // drop V0->point to rail->Curve over sequence of ranges until we get a hit
              double dT = 0.0;
              double dDroppingTol = dAppTol;
              for (double dScale=10.0; dScale<100.0; dScale+=10.0)
                {
                  SmStatus eStat =
                      p3DRailCurve->DropPoint(sTrimIvl,     // in : target curve allowed domain
                                              sPnt0,        // in : Point to drop to curve
                                              NULL,         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                              dDroppingTol, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                              NULL,         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                              bSuccess,     // out: TRUE = found a drop point
                                              dT,           // out: found drop curve param
                                              dDist) ;      // out: found drop distance
                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
                  if ( eStat != SM_SUCCESS )
                  {
                      SM_DBG_WARN(_T("TrimRailCurve(): Suffered a SmCurve::DropPoint() failure") );
                      // SER( eStat );
                  }

                  if (bSuccess) break;
                  dDroppingTol = dAppTol*dScale;
                }
              if (!bSuccess)
                SER(SM_ERR);

              // clean up dT value from 3.000000001 to 3.0
              dT = smos_CleanUpNoise(dT);

              // set Vertexuse0->UVPoint on filletSurface
              SM_ASSERT(pUVCurve != NULL) ; // gwc:test
              if ( pUVCurve != NULL )
              {
                  SmPoint2d sUV0(dT,dIsoV);
                  pVU0->SetUVPoint(sUV0);
              }

              // save V0 to d3Curve dropPoint parameter
              if (pPrimEU->GetOrientation() == SM_OT_SAME)
                {
                  dParam1 = dT;
                }
              else
                {
                  SER(SM_ERR);//dParam2 = dT;
                }
            } // end pV1->IsProcessed() check

          // when V1 vertex is processed
          // gwc: it's okay for a vertex to be unprocessed - some are status = SM_FV_NO_INT_RAIL_EU,
          // SM_ASSERT(pV1->IsProcessed()) ; // gwc:test
          if (pV1->IsProcessed())
            {
              // find 3DCurve points close to V1
              SmSolution sData[8];
              SmSolutionArray sSolutions(8,sData);
              SER(p3DRailCurve->GlobalPointSolve(sTrimIvl,SM_SO_MINIMIZE, sPnt1,
                  dAppTol, NULL,NULL, SM_SR_ALL,sSolutions));

              // Note: previously, this assumed that if there were two
              // solutions, that means a closed curve, and that the
              // curve param of the second solution would be higher than
              // the first; and also that if there were more than two
              // solutions, then there must have been an error.
              // Don't assume those things, and just find the best solution.
              //   [fillet regression case 2:229; bd 30 Nov 05]:
              ULONG dNumSolutions = sSolutions.GetSize();
              if ( dNumSolutions < 1 )
              {
                  SER(SM_ERR);
              }

              dParam2 = sSolutions[0].m_vStart[0];

              if ( dNumSolutions > 1 )
              {
                  double dBestSolVal = sSolutions[0].m_vStart.m_dSolutionValue;
                  for ( ULONG iii = 1; iii < dNumSolutions; iii++ )
                  {
                      // Check only those beyond dParam1
                      if ( sSolutions[iii].m_vStart[0] > dParam1
                        && sSolutions[iii].m_vStart.m_dSolutionValue <= dBestSolVal )
                      {
                          dParam2 = sSolutions[iii].m_vStart[0];
                          dBestSolVal = sSolutions[iii].m_vStart.m_dSolutionValue;
                      }
                  }
              }




              // set Vertexuse1->UVPoint on filletSurface
              SM_ASSERT(pUVCurve != NULL) ; // gwc:test
              if (pUVCurve)
                {
                  SmPoint2d sUV1(dParam2,dIsoV);
                  pVU1->SetUVPoint(sUV1);
                }
            } // end V1 is processed check

          // clip the interval to drop points
          sTrimIvl.SetMinMax(dParam1,dParam2);

          // Trim 3D curve and both UVTrimCurves
          SER(p3DRailCurve->Trim(sTrimIvl)); // may snap sIvl by tol to existing knots
          if (pUVCurve)
            {
              SmBSplineCurve *pOrigFaceUVCurve = pMateEU->GetUVTrimCurvePointer();
              if (pOrigFaceUVCurve)
                {
                  SER(pOrigFaceUVCurve->Trim(sTrimIvl)); // may snap sIvl by tol to existing knots
                }
              SER(pUVCurve->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots
            }
        } // end EndVertex->points differ by more than tol branch
      else // EndVertex->points are the same
        {
          // Rail could be either closed, degenerate  or self-intersected
          // No further processing if closed or degenerate
          if (   p3DRailCurve->IsClosed(sTrimIvl,dAppTol)
              || p3DRailCurve->IsDegenerate())
            {
              continue;
            }

          // Check if rail self-intersect
          SmSolutionArray sSolutions;
          SER(p3DRailCurve->GlobalCurveSelfIntersect( sTrimIvl,
              dAppTol*10.0, sSolutions));

          // when there is just 1 selfIntersection solution
          if (sSolutions.GetSize() == 1)
            {
              // get nonSelfIntersecting interval
              SmSolution & rSol = sSolutions[0];
              dParam1           = rSol.m_vStart[0];
              dParam2           = rSol.m_vStart[1];
              sTrimIvl.SetMinMax(dParam1,dParam1);
              sTrimIvl.AddValue(dParam2);

              // trim curve and UVTrimCurves to interval
              SER(p3DRailCurve->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots
              SM_ASSERT(pUVCurve != NULL) ; // gwc:test
              if (pUVCurve)
                {
                  SmBSplineCurve *pOrigFaceUVCurve = pMateEU->GetUVTrimCurvePointer();
                  if (pOrigFaceUVCurve)
                      SER(pOrigFaceUVCurve->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots
                  SER(pUVCurve->Trim(sTrimIvl)); // may snap sIvl by tol to existing knots
                }
            } // end 1 solution found check
          else
            {
               SM_DBG_WARN(_T("more than 1 self intersection found on rail curve - case not handled yet")) ;
            }
        } // end rail->Vertex->EndPoint are the same branch

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 ) {
          smgfx_SetLook(4,4, 1,0,0); p3DRailCurve->DrawWDeriv(sTrimIvl); sm_GraphicsLoop();
          smgfx_SetLook( 2, 5, 0, 1, 0 ); if(pV0->IsProcessed()) { sPnt0.Draw(); sm_GraphicsLoop(); }
          if(pV1->IsProcessed()) { sPnt1.Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE
    } // end iter both rail curves

  return SM_SUCCESS;

} // end SmFilletGeom::TrimRailCurves

/*******************************************************************//**
PURPOSE: Trim rail curves of cliff-rollover fillet geom

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::TrimCliffRails()
{
  if ( m_eType != SM_FG_CLIFF_ROLLOVER )
    { SER(SM_ERR); }

  int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
  iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  const SmContext  & crContext      = m_pFilletSolver->GetCreationContext();
  double             dTolerance     = m_pFilletSolver->GetThisApproxTol3d();
  double             dAngleTol      = m_pFilletSolver->GetThisAngTolRad();

  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pFilletSurface = m_pFilletSurface; NER(pFilletSurface);
  SM_FILLETSURF_TYPE * pFilletSurface = m_pFilletSurface; NER(pFilletSurface);
  SmExtent2d         sFilSrfDomain  = pFilletSurface->GetNaturalUVDomain();

  for (ULONG lRailIdx=0; lRailIdx<2; lRailIdx++)
    {
      SmFilletEdgeuse * pRailEU              = (SmFilletEdgeuse*)m_vRails[ lRailIdx ]->GetPrimaryEdgeuse();
      SmFilletEdgeuse * pRailMateEU          = (SmFilletEdgeuse*)m_vRails[ lRailIdx ]->GetPrimaryEdgeuse()->GetMate() ;

      SmCurve         * pOrigRailCurve       = m_vRails[lRailIdx]->GetCurve(); NER(pOrigRailCurve);
      SmExtent1d        sOrigRailIvl         = pOrigRailCurve->GetNaturalInterval();
      SmBoolean         bDeleteOrigRailCurve = (pOrigRailCurve != NULL) ? TRUE : FALSE;

      // get UVCurve on BaseSurface
      SmBSplineCurve  * pBaseUVTrimCurve     = pRailMateEU->GetUVTrimCurvePointer();
      SmBoolean         bDeleteBaseUVCurve   = (pBaseUVTrimCurve != NULL) ? TRUE : FALSE;

      SmBSplineCurve  * pFilletUVCurve       = NULL;
      SmBoolean         bDeleteFilletUVCurve = FALSE ;

     // decouple and delete RailEU->m_pUVTrimCurve - it won't be used again
      pRailEU->SetUVCurve(NULL, TRUE) ; // TRUE = delete preExisting m_pUVTrimCurve

      // decouple RailMateEU from its UVTrimCurve wthout deleting it (we have it as pBaseUVTrimCurve)
      pRailMateEU->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting m_pUVTrimCurve 
                                                    // FALSE = no Leak Warnings

      // decouple thisRail->OrigRailCurve without deleting it (we have it as pOrigRailCurve)
      //  after SetUVCurve(NULL) calls to prevent deleting UVTrimCurves too soon.
      m_vRails[lRailIdx]->SetCurve(NULL, FALSE, FALSE) ;  // FALSE = don't delete existing m_pCurve, FALSE = no Leak Warnings
                                                          // Side Effect: deletes all UVTrimCurves.

      // begin scope - build pFilletUVCurve = IsoLineTrimCurve for pRailEU
        {
          // Find dIsoV: where IsoParamCurve FilletSurface(dT, dIsoV) is coincident with OrigRailCurve(dT)
          SmPoint3d sCurvePnt, sSurfPnt;
          double    dT    = sOrigRailIvl.Evaluate(0.34567);
          double    dIsoV = sFilSrfDomain.GetMin().y;
          SmPoint2d sTestUV(dT, dIsoV);

          // get RailCurve(dT) and FilletSurface(dT, dMinV) points
          SER( pOrigRailCurve->EvaluatePoint( dT, sCurvePnt ));
          SER( pFilletSurface->EvaluatePoint( sTestUV, sSurfPnt ));

          // when RailCurve(dT) and FilletSurface(dT, dMinV) points not the same
          if ( sCurvePnt.DistanceBetween( sSurfPnt ) > dTolerance)
            {
              // Get FilletSurface(dT, dMaxV)
              dIsoV     = sFilSrfDomain.GetMax().y;
              sTestUV.y = dIsoV;
              SER( pFilletSurface->EvaluatePoint(sTestUV, sSurfPnt ));

              // and when RailCurve(dT) and FilletSurface(dT, dMaxV) points not the same
              if ( sCurvePnt.DistanceBetween( sSurfPnt ) > dTolerance)
                {
                  // RailCurve not synchronized with FilletSurface Min or Max IsoParamCurve -
                  // Signal error and quit
                  SER(SM_ERR);
                }
            }

          // arrive here when IsoParamCurve FilletSurface(dT, dIsoV) coincident with OrigRailCurve(dT)

          // FilletSurface StartUV and EndUV points at ends of OrigRailCurve
          SmPoint2d sStartUV = SmPoint2d( sOrigRailIvl.GetMin(), dIsoV );
          SmPoint2d sEndUV   = SmPoint2d( sOrigRailIvl.GetMax(), dIsoV );

          // Create new LineSegment[StartUV EndUV] UVCurve
          SER( SmBSplineCurve::CreateLineSegment( crContext,
                                                  2,
                                                  SmPoint3d(sStartUV),
                                                  SmPoint3d(sEndUV),
                                                  pFilletUVCurve ));
          NER( pFilletUVCurve );
          SER( pFilletUVCurve->EditParameterization( sOrigRailIvl ));

        } // end scope - build pFilletUVCurve = IsoLineTrimCurve for pRailEU

      // mark FilletUVCurve for deletion - if it gets saved under an Edgeuse this value will be changed to FALSE
      bDeleteFilletUVCurve = pFilletUVCurve ? TRUE : FALSE ;

      // gwc - too soon to attach as UVTrimCurve - it will get deleted when OrigRail->SetCurve is called
      //  // set OrigRail->UVTrimCurve = LineSegment[StartUV EndUV]
      //  pRailEU->SetUVCurve( pFilletUVCurve );
      //  SmBoolean bDeleteFilletUVCurve = TRUE;

      // Collect all of the pieces of this rail curve.
      SmTArray<SmFilletEdge *> sAllCurvesThisRail;

      // The main rail ...
      sAllCurvesThisRail.Add( m_vRails[ lRailIdx ] );

      // ... plus the other pieces (mostly when Rail has been split into multiple segments when rollover occurs)
      if ( lRailIdx == 0 ) { sAllCurvesThisRail.Append( m_vOtherRails1 ); }
      else                 { sAllCurvesThisRail.Append( m_vOtherRails2 ); }

      // Loop on all of this side's rails
      for ( ULONG jj=0; jj<sAllCurvesThisRail.GetSize(); jj++ )
        {
          SmFilletEdge    * pRail     = sAllCurvesThisRail[jj];
          SmFilletEdgeType  eRailType = pRail->GetFilletEdgeType();
          SmFilletEdgeuse * pPrimEU   = (SmFilletEdgeuse*) pRail->GetPrimaryEdgeuse();
          SmFilletVertex  * pStartV   = (SmFilletVertex*)  pRail->GetVertex();
          SmFilletVertex  * pEndV     = (SmFilletVertex*)  pRail->GetOtherVertex(pStartV);
          SmFilletEdgeuse * pMateEU   = (SmFilletEdgeuse*) pPrimEU->GetMate();

          // when pStartV is UnProcessed SM_FV_CLIFF_RAIL_X_EDGE - move pStartV->m_vPoint to XSect
          if(    !pStartV->IsProcessed()
             &&  pStartV->GetFilletVertexType() == SM_FV_CLIFF_RAIL_X_EDGE )
            {
              // Set pStartV::m_vPoint  = XSectPoint(FilletSurface, Orig BrepCurve) and
              //     pStartV::m_eStatus = SM_FIL_PROCESSED
              SER( pStartV->CalcCliffRailIntEdge( pFilletSurface, dTolerance ));
            }

          // when pEndV is UnProcessed SM_FV_CLIFF_RAIL_X_EDGE - move pEndV->m_vPoint to XSect
          if(   !pEndV->IsProcessed()
             &&  pEndV->GetFilletVertexType() == SM_FV_CLIFF_RAIL_X_EDGE )
            {
              // Set pEndV::m_vPoint  = XSectPoint(FilletSurface, Orig BrepCurve) and
              //     pEndV::m_eStatus = SM_FIL_PROCESSED
              SER( pEndV->CalcCliffRailIntEdge( pFilletSurface, dTolerance ));
            }

          // get StartV and EndV Point3ds
          SmPoint3d sStartPnt = pStartV->GetPoint();
          SmPoint3d sEndPnt   = pEndV->GetPoint();

#ifdef SM_DEBUG_CODE
          if ( iDebugLevel > 0 )
            {
              smgfx_SetLook(1,2, 1,0,0); pRail->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); sStartPnt.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); sEndPnt.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

          // Branch on whether this RailCurve pieces
          // is running along the edge of the base face (the cliff)
          // or is the regular part of the rail (the original piece that's in the base face)
          if(eRailType == SM_FE_CLIFF_RAIL)
            {
              // Get side FilletGeom and side surface.
              SmFilletGeom     * pSideFG      =  (pStartV == pPrimEU->GetVertexuse()->GetVertex())
                                                ? pPrimEU->GetFilletGeom()
                                                : pMateEU->GetFilletGeom() ;
              
              // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 3 line
              // rm : SmBSplineSurface * pSideSurface =  (pSideFG && pSideFG != this)
              // rm :                                   ? pSideFG->GetFilletSurface()
              // rm :                                   : SM_CAST_PTR(SmBsplineSurface, pRail->GetOriginalFace()->GetSurface()) ;
              SM_FILLETSURF_TYPE * pSideSurface =  (pSideFG && pSideFG != this)
                                                  ? pSideFG->GetFilletSurface()
                                                  : SM_CAST_FILLETSURF_PTR(SM_FILLETSURF_TYPE, pRail->GetOriginalFace()->GetSurface()) ;
              NER( pSideSurface );

              // Intersect fillet with side face
              SmBSplineCurve * p3DIntCurve         = NULL;
              SmBSplineCurve * pNewFilletUVCurve   = NULL;
              SmBSplineCurve * pNewBaseUVTrimCurve = NULL;
              SmVector3d       sDir                = sEndPnt - sStartPnt;

              SER( sm_SrfSrfIntersection( crContext,              // in : context for new object construction
                                          pStartV,                // in : 1st point known to be on intersection curve
                                          pEndV,                  // in : 2nd point known to be on intersection curve,
                                                                  //      NULL to ignore
                                          &sDir,                  // in : expected general direction of intersection curve from 1st point
                                          NULL,                   // in   expected intersection end direction
                                          pFilletSurface,         // in : 1st intersecting surface
                                          pSideSurface,           // in : 2nd intersecting surface
                                          dTolerance,                   // in : max allowed distance between xSect Curve and surfaces
                                          dAngleTol,              // in : max allowed angle between consecutive xSect curve segment tangents
                                          p3DIntCurve,            // out: 3d intersection curve
                                          pNewFilletUVCurve,      // out: associated UVTrimCurve on 1st surface
                                          pNewBaseUVTrimCurve,    // out: associated UVTrimCurve on 2nd surface
                                          FALSE,                  // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                                  //             resorting to expensive general surf/surf xSect solver
                                          NULL,                   // in : reference point
                                          iDebugLevel ));         // in : iDebugLevel, 0 = No Debug output
              NER( p3DIntCurve );

              // set the Rail->Curve and Rail->UVTrimCurve
              if(pRail->GetCurve() != pOrigRailCurve) { pRail->SetCurve(p3DIntCurve, TRUE) ; // TRUE = delete preExisting Curve
                                                                                             // side effect: delete current pRail->UVTrimCurves(expect none at this point)
                                                      }
              else                                    { pRail->SetCurve(p3DIntCurve, FALSE) ; // FALSE = don't delete preExisting Curve
                                                                                              // side effect: delete current pRail->UVTrimCurves(expect none at this point)
                                                      }

              p3DIntCurve->SetOwner( pRail );
              pRail->SetInterval(  p3DIntCurve->GetNaturalInterval() );
              pPrimEU->SetUVCurve( pNewFilletUVCurve );
              pMateEU->SetUVCurve( NULL );

              // clean up - we don't use the uv curve in the base surface,
              //  but it should be there...
              SM_ASSERT( pNewBaseUVTrimCurve != NULL );
              delete pNewBaseUVTrimCurve; pNewBaseUVTrimCurve = NULL;

            } // end eRailType == SM_FE_CLIFF_RAIL branch

          else if ( eRailType == SM_FE_RAIL )
            {
              // It's the original piece, in the base face.
              // Copy the original curve and trim it

              // skip closed Rails - Set UVTrimCurves and they're done
              if ( pStartV == pEndV )
                {
                  // restore OrigRailCurve
                  pRail->SetCurve( pOrigRailCurve, FALSE) ; // FALSE = don't delete preExisting Curve
                                                            // side effect: delete current pRail->UVTrimCurves
                  pOrigRailCurve->SetOwner( pRail );

                  // Set UVTrimCurves
                  pPrimEU->SetUVCurve( pFilletUVCurve ) ;
                  pMateEU->SetUVCurve( pBaseUVTrimCurve ) ;

                  // remember not to delete original base UVTrimCurve and new IsoCurve UVTrimCurve - they're being used
                  bDeleteOrigRailCurve = FALSE ;
                  bDeleteFilletUVCurve = FALSE ;
                  bDeleteBaseUVCurve   = FALSE ;
                  continue ;
                }

              // locals
              SmExtent1d sTrimIvl = sOrigRailIvl;
              SmBoolean  bSuccess;
              double     dParam1 = sTrimIvl.GetMin();
              double     dParam2 = sTrimIvl.GetMax();
              double     dDist;
              double     dTol = m_pFilletSolver->GetThisApproxTol3d();

              // when pStartV->m_vPoint is set - Drop start XSectPoint(FilletSurface, OrigBrepCurve) onto RailCurve
              if ( pStartV->IsProcessed() )
                {
                  // gwc???: why use DropPoint for pStartV and GlobalPointSolve for pEndV?
                  if(   SM_SUCCESS != pOrigRailCurve->DropPoint(sTrimIvl,   // in : target curve allowed domain
                                                                sStartPnt,  // in : Point to drop to curve
                                                                NULL,       // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                                dTol,       // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                                NULL,       // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                                bSuccess,   // out: TRUE = found a drop point
                                                                dParam1,    // out: found drop curve param
                                                                dDist)      // out: found drop distance
                                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
                     || !bSuccess )
                    {
                      // Give it another chance with relaxed tol
                      // [bd 27Dec05 T1000B]
                      if(SM_SUCCESS != pOrigRailCurve->DropPoint(sTrimIvl,  // in : target curve allowed domain
                                                                 sStartPnt, // in : Point to drop to curve
                                                                 NULL,      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                                 10*dTol,   // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                                 NULL,      // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                                 bSuccess,  // out: TRUE = found a drop point
                                                                 dParam1,   // out: found drop curve param
                                                                 dDist)     // out: found drop distance
                                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
                         || !bSuccess )
                        { SER(SM_ERR); }
                    }
                } // end if pStartV->IsProcessed() - then Drop sStartPnt onto OrigRailCurve for dParam1 block

              // when pEndV->m_vPoint is set - Drop end XSectPoint(FilletSurface, OrigBrepCurve) onto RailCurve
              if ( pEndV->IsProcessed() )
                {
                  // gwc???: why use DropPoint for pStartV and GlobalPointSolve for pEndV?
                  SmSolution      sData[8];
                  SmSolutionArray sSolutions(8,sData);
                  SER( pOrigRailCurve->GlobalPointSolve(sTrimIvl, 
                                                        SM_SO_MINIMIZE,
                                                        sEndPnt, 
                                                        dTol, 
                                                        NULL, 
                                                        NULL, 
                                                        SM_SR_ALL, 
                                                        sSolutions)) ;
                  if ( sSolutions.GetSize() < 1 )
                      SER(SM_ERR);

                  // Two solutions could mean a closed curve, but maybe not.
                  // Sometimes we get multiple solutions that are very close
                  // to each other.  Take the best solution value that's
                  // beyond dParam1.
                  dParam2 = sSolutions[0].m_vStart[0];
                  double dBestSolValue = sSolutions[0].m_vStart.m_dSolutionValue;
                  for ( ULONG ii = 1; ii < sSolutions.GetSize(); ii++ )
                    {
                      if ( sSolutions[ii].m_vStart[0] > dParam1 )
                        {
                          if ( sSolutions[ii].m_vStart.m_dSolutionValue < dBestSolValue + SM_EFF_ZERO )
                            {
                              dParam2       = sSolutions[ii].m_vStart[0];
                              dBestSolValue = sSolutions[ii].m_vStart.m_dSolutionValue;
                            }
                        }
                    }
                }  // end if pEndV->IsProcessed() - then Drop sEndPnt onto OrigRailCurve for dParam2 block

              if ( dParam1 > dParam2 )
                { SER(SM_ERR); }

              // Set RailCurve = Trimmed[dParam1 dParam2] OrigCurve Copy
              sTrimIvl.SetMinMax( dParam1, dParam2 );
              SmCurve * pCopyCurve = NULL;
              SER( pOrigRailCurve->Copy( crContext, pCopyCurve ));
              SER( pCopyCurve->Trim( sTrimIvl ));  // may snap sIvl by tol to existing knots
              if(pRail->GetCurve() != pOrigRailCurve) { pRail->SetCurve(pCopyCurve, TRUE) ; // TRUE = delete preExisting m_pUVCurve
                                                                                            // side effect: delete current pNewE->UVTrimCurves - okay ptrs have been set to NULL

                                                      }
              else                                    {pRail->SetCurve(pCopyCurve, FALSE) ; // FALSE = don't delete preExisting m_pUVCurve
                                                                                            // side effect: delete current pNewE->UVTrimCurves - okay ptrs have been set to NULL

                                                      }
              pCopyCurve->SetOwner(pRail) ;
              pRail->SetInterval( sTrimIvl );

              // Set Rail->PrimEU->UVTrimCurve = FilletUVCurve trimmed[dParam1 dParam2] copy
              SmCurve * pCopyFilletUVCurve = NULL;
              SER( pFilletUVCurve->Copy( crContext, pCopyFilletUVCurve ));
              SER( pCopyFilletUVCurve->Trim( sTrimIvl )); // may snap sIvl by tol to existing knots
              pPrimEU->SetUVCurve( (SmBSplineCurve*)pCopyFilletUVCurve );

              // when pBaseUVTrimCurve exists
              if ( pBaseUVTrimCurve )
                {
                  // set Rail->MateEU->UVTrimCurve = BaseUVTrimCurve trimmed[dParam1 dParam2] copy
                  SmCurve * pCopyBaseUVCurve = NULL;
                  SER( pBaseUVTrimCurve->Copy( crContext, pCopyBaseUVCurve ));
                  SER( pCopyBaseUVCurve->Trim( sTrimIvl ));  // may snap sIvl by tol to existing knots
                  pMateEU->SetUVCurve( (SmBSplineCurve*)pCopyBaseUVCurve );
                }
            } // end eRailType == SM_FE_RAIL branch

#ifdef SM_DEBUG_CODE
            SM_ASSERT_BREAK_MSG(pRail->GetCurve() != NULL, _T("SmFilletGeom::TrimCliffRails - built a RailCurve with a m_pCurve == NULL value")) ;
#endif // SM_DEBUG_CODE

        } // end iter sAllCurvesThisRail

      // if we haven't used the original Rail->Curves and UVTrimCurves - delete them
      if ( bDeleteOrigRailCurve ) { delete pOrigRailCurve ;   pOrigRailCurve   = NULL ; }
      if ( bDeleteFilletUVCurve ) { delete pFilletUVCurve ;   pFilletUVCurve   = NULL ; }
      if ( bDeleteBaseUVCurve )   { delete pBaseUVTrimCurve ; pBaseUVTrimCurve = NULL ; }

    } // end iter both rails

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::TrimCliffRails

// Some static helpers, just to keep code less cluttered.
/*******************************************************************//**
PURPOSE: If a uv point is outside the domain, check whether it can be moved
   into the domain due to periodicity.  Move it if so, and return TRUE.

NOTES: 
***********************************************************************/
static SmBoolean sm_CheckSeamOut( const SmSurface *pSurf, SmPoint2d & rUV, double dTol, SmSurfParamType eClosure )
{
  if ( eClosure == SM_SP_NEITHER ) { return FALSE; }
  if ( rUV.x   <= -SM_BIG_DOUBLE ) { return FALSE; }
  if ( rUV.y   <= -SM_BIG_DOUBLE ) { return FALSE; }

  SmExtent2d sDom = pSurf->GetNaturalUVDomain();

  if ( sDom.ContainsPoint2d( rUV, dTol ) )
    { return FALSE; }

  SmBoolean bRet = FALSE;

  if ( eClosure == SM_SP_U || eClosure == SM_SP_BOTH )
    {
      SmExtent1d sDomU = sDom.GetUInterval();
      if ( rUV.x < sDomU.GetMin() - dTol )
        {
          rUV.x += sDomU.GetLength();
          bRet = TRUE;
        }
      if ( rUV.x > sDomU.GetMax() + dTol )
        {
          rUV.x -= sDomU.GetLength();
          bRet = TRUE;
        }
    }

  if ( eClosure == SM_SP_V || eClosure == SM_SP_BOTH )
    {
      SmExtent1d sDomV = sDom.GetVInterval();
      if ( rUV.y < sDomV.GetMin() - dTol )
        {
          rUV.y += sDomV.GetLength();
          bRet = TRUE;
        }
      if ( rUV.y > sDomV.GetMax() + dTol )
        {
          rUV.y -= sDomV.GetLength();
          bRet = TRUE;
        }
    }

  return bRet;

} // end static sm_CheckSeamOut

/*******************************************************************//**
PURPOSE: If a uv point is on a domain boundary, and the boundary is a seam,
   move it to the other side of the seam, and return TRUE.

NOTES: 
***********************************************************************/
static SmBoolean sm_CheckSeamOn( const SmSurface *pSurf, SmPoint2d & rUV, double dTol, SmSurfParamType eClosure )
{
  if ( eClosure == SM_SP_NEITHER ) { return FALSE; }
  if ( rUV.x   <= -SM_BIG_DOUBLE ) { return FALSE; }
  if ( rUV.y   <= -SM_BIG_DOUBLE ) { return FALSE; }

  SmExtent2d sDom = pSurf->GetNaturalUVDomain();

  SmBoolean bRet = FALSE;

  if ( eClosure == SM_SP_U || eClosure == SM_SP_BOTH )
    {
      SmExtent1d sDomU = sDom.GetUInterval();
      if ( smos_Fabs( rUV.x - sDomU.GetMin() ) < dTol )
        {
          rUV.x = sDomU.GetMax();
          bRet = TRUE;
        }
      else if ( smos_Fabs( rUV.x - sDomU.GetMax() ) < dTol )
        {
          rUV.x = sDomU.GetMin();
          bRet = TRUE;
        }
    }

  if ( eClosure == SM_SP_V || eClosure == SM_SP_BOTH )
    {
      SmExtent1d sDomV = sDom.GetVInterval();
      if ( smos_Fabs( rUV.y - sDomV.GetMin() ) < dTol )
        {
          rUV.y = sDomV.GetMax();
          bRet = TRUE;
        }
      else if ( smos_Fabs( rUV.y - sDomV.GetMax() ) < dTol )
        {
          rUV.y = sDomV.GetMin();
          bRet = TRUE;
        }
    }

  return bRet;

} // end static sm_CheckSeamOn

//cbi 398:

/*******************************************************************//**
PURPOSE: Check whether two TsectPnts are in the correct order, based on geometry.
   The one at lIdx1 should point towards the one at lIdx2.

NOTES: We just look at the 3d curve positions and tangents;
   could also look at UV pos and derivatives.
***********************************************************************/
static SmBoolean sm_IsOrderCorrect
 (SmTArray<SmTsectPnt*> & rTsectPnts, 
  ULONG                   lIdx1, 
  ULONG                   lIdx2 )
{
  SmTsectPnt *pTSP1 = rTsectPnts[ lIdx1 ];
  SmTsectPnt *pTSP2 = rTsectPnts[ lIdx2 ];

  SmVector3d sDiff( pTSP2->CrvPos() - pTSP1->CrvPos() );
  SmVector3d sTan ( pTSP1->CrvDeriv() );

  return ( sDiff.Dot( sTan ) > 0.0 );
}

// -----

//cbi 398:
//cbi maybe rename, since we're also checking order.

/*******************************************************************//**
PURPOSE: Move TsectPnt pointers within the array, IF the geometry indicates
   that that should be done.

NOTES: 
***********************************************************************/
static SmStatus sm_MoveTSP
 (SmTArray<SmTsectPnt*> & rTsectPnts, 
  ULONG                   lFrom,
  ULONG                   lTo )
{
  ULONG lSize = rTsectPnts.GetSize();
  if ( lFrom >= lSize || lTo >= lSize )
    { return SM_ERR; }

  if ( sm_IsOrderCorrect( rTsectPnts, smos_Min( lFrom, lTo ), smos_Max( lFrom, lTo ) ) )
    { return SM_SUCCESS; }

  // It's just pointers, so just swap them.
  if ( lSize == 2 )
    {
      SM_ASSERT( lFrom + lTo == 1 );

      SmTsectPnt *pTspTemp = rTsectPnts[ lFrom ];
      rTsectPnts[ lFrom ] = rTsectPnts[ lTo ];
      rTsectPnts[ lTo   ] = pTspTemp;
    }
  else
    {
      SmTsectPnt *pTspTemp = rTsectPnts[ lFrom ];
      rTsectPnts.RemoveAt( lFrom );
      rTsectPnts.InsertAt( lTo, pTspTemp );
    }

  return SM_SUCCESS;

} // end static sm_MoveTSP

/*******************************************************************//**
PURPOSE: Set surface closure flags.

NOTES: 
***********************************************************************/
static SmSurfParamType sm_SetSurfaceClosure( const SmSurface *pSurf, const SmExtent2d & rDomain )
{
  SmSurfParamType eRet = SM_SP_NEITHER;

  if ( pSurf->IsClosed( rDomain, SM_SP_U ) )
    { eRet = SM_SP_U; }
  if ( pSurf->IsClosed( rDomain, SM_SP_V ) )
    { eRet = ( eRet == SM_SP_NEITHER ) ? SM_SP_V : SM_SP_BOTH; }

  return eRet;
}

// End of static helpers.

/*******************************************************************//**
PURPOSE: Recreate the fillet surface (and fillet edges, if needed)
    Typically, this routine can be used to refine the fillet surfaces
    such that it will interpolate all the rail ends. (For example, when
    SmFilletGeom were split, it is often desirable to have the fillet
    surfaces pass through their rail ends).  Another use of it is to
    compute the fillet surface patch for each SmFilletGeom when roll-over
    occurred.

NOTES: User may want to specify 'optional' pMarchDir if the
    geometry hasn't yet been computed
***********************************************************************/
SmStatus SmFilletGeom::ReCalcFilletGeom
  (SmBoolean bIsAnalyticFillet,
   SmVector3d * pOptMarchDir,
   SmCurve * pOptRefCurve)
{
    SmFilletSolver * pFS = this->GetFilletSolver();
    const SmContext & crContext = pFS->GetCreationContext();

    SmCurve * pCurve = NULL;
    SmBoolean b3DTest = TRUE;
    SmObjDelete sDelete;
    if (m_eType == SM_FG_CORNER_FILLET)
    {
        pCurve = pOptRefCurve; NER(pCurve);
    }
    else {
        // When processing a tangent rollover, we're working on
        // different faces.  Since faces are stored in the FS
        // instead of FG, the caller temporarily swaps out the
        // faces in the FS.  However, the FS also contains the
        // original edge being filleted, and that's what we sort
        // along.  So, since we know what the caller is thinking,
        // we do things differently in the tangent-rollover case.

        if (m_eType != SM_FG_TANGENT_ROLLOVER)
          { b3DTest = FALSE; }

        SmEdge * pEdge = pFS->GetEdgeuse(0)->GetEdge();
        pCurve = pEdge->GetCurve(); NER(pCurve);
    }
    SmExtent1d sIvl = pCurve->GetNaturalInterval();
    SmBoolean bIsClosedSolverEdge = FALSE;
    if (m_eType != SM_FG_CORNER_FILLET &&
        pCurve->IsClosed(sIvl)) {
        bIsClosedSolverEdge = TRUE;
    }

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {


        if ( FALSE ) {
            smgfx_Erase();
            smgfx_SetLook( 1,2, 0,0,0 );
            this->GetFilletSolver()->GetFilletExecutive()->GetTargetBrep()->Draw(TRUE); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }

        smgfx_SetLook(3,3, 0,1,0); pCurve->DrawWDeriv(sIvl); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SmOffsetSurface * pSurf1 = pFS->GetSurface(0);
    SmOffsetSurface * pSurf2 = pFS->GetSurface(1);
    SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
    SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();
    double dTolerance    = pFS->m_dThisApproxTol3d;
    double dAngTol = pFS->GetThisAngTolRad();
    double dScale  = sIvl.GetLength();

    // Note, surface closure is relatively expensive to calculate.
    // If we need it (for seam checking), we'll calculate it once and pass flags.
    SmSurfParamType eSurfClosure1 = sm_SetSurfaceClosure( pSurf1, sDomain1 );
    SmSurfParamType eSurfClosure2 = sm_SetSurfaceClosure( pSurf2, sDomain2 );


    // Sort the SmTsectPnt's of each rail-end, the rails should
    // go in the same 'direction' as the solver edge
    //
    // Each rail has two Vertexuses, and each Vertexuse has
    // an SmTsectPnt with a uv point.  Sort along the rails.

    // Loop on both rails

    SmTsectPnt* sP1Data[64];
    SmTArray<SmTsectPnt*> sTsectPnts(64,sP1Data);
    SmTArray<double> sProjParams;
    ULONG lAllRailIdx, lThisFilEU;
    for ( ULONG jj=0; jj<2; jj++ )
    {
        // If working in 2d, use the uv curve of the original edgeuse.
        if (!b3DTest)
        {
            SmEdgeuse * pEUOnSurf = pFS->GetEdgeuse(jj); NER(pEUOnSurf);
            pCurve = pEUOnSurf->GetUVTrimCurve(); NER(pCurve);
        }

        // sAllRails will be all rails for this side:
        // thisRail plus extra if there is one.
        SmTArray<SmFilletEdge *> sAllRails;
        sAllRails.Add( m_vRails[jj] );
        if ( jj==0 ) {
            sAllRails.Append( m_vOtherRails1 );
        }
        else {
            sAllRails.Append( m_vOtherRails2 );
        }

        // Loop on all rails on this side: the main one and any extras.
        for ( lAllRailIdx=0; lAllRailIdx<sAllRails.GetSize(); lAllRailIdx++ )
        {
            SmFilletEdge * pRail= sAllRails[lAllRailIdx]; NER(pRail);
            SmTArray<SmEdgeuse*> sFilEUs;
            pRail->GetEdgeuses( sFilEUs );
            if ( sFilEUs.GetSize() != 2 ) { SER(SM_ERR); }

            // Loop on both edgeuses of this rail, sorting their Vertexuses along pCurve.
            for ( lThisFilEU=0; lThisFilEU<2; lThisFilEU++ )
            {
                SmEdgeuse * pEU = sFilEUs[lThisFilEU];
                SmFilletVertexuse * pVU = (SmFilletVertexuse*)pEU->GetVertexuse();
                SmFilletVertex * pFV = (SmFilletVertex*)pVU->GetVertex();
                if (pFV->GetStatus() == SM_FIL_UNPROCESSED ||
                    pFV->GetFilletVertexType() == SM_FV_MATE ||
                    pFV->GetFilletVertexType() == SM_FV_RAIL_X_EXTENDED_EDGEUSE) {
                    continue;
                }
                SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();


                // Check for seams.  When calculating the Fillet surface and rail geometry,
                // we extend our surfaces even in the seam directions (a little), and will
                // march right along the wrong side of seams.  [B398]
                double dTol = SM_EFF_ZERO;
                const SmSurface *pBaseSurf1 = pSurf1->GetBaseSurface();
                SmPoint2d sUV = rTsectPnt.UVPos(0);
                if ( sm_CheckSeamOut( pBaseSurf1, sUV, dTol, eSurfClosure1 ) )
                  { rTsectPnt.UVPos(0) = sUV; }

                const SmSurface *pBaseSurf2 = pSurf2->GetBaseSurface();
                sUV = rTsectPnt.UVPos(1);
                if ( sm_CheckSeamOut( pBaseSurf2, sUV, dTol, eSurfClosure2 ) )
                  { rTsectPnt.UVPos(1) = sUV; }



                SmPoint3d sPnt;

                // Check for rTsectPnt not set.
                if ( !b3DTest ) // Do a 2D test
                {
                    sPnt = SmPoint3d( rTsectPnt.UVPos( jj ) );
                    if (sPnt.x == -SM_BIG_DOUBLE || sPnt.y == -SM_BIG_DOUBLE) {
                        continue;
                    }
                }
                else // Do a 3D test
                {
                    if (    rTsectPnt.m_ePointType == SM_IP_UNKNOWN
                         || rTsectPnt.CrvPos().IsUndef()
                       )
                    {
                        SmPoint2d sUVPt = rTsectPnt.UVPos(0);
                        if (sUVPt.x == -SM_BIG_DOUBLE || sUVPt.y == -SM_BIG_DOUBLE) {
                            continue;
                        }
                        SER(pFS->SetupOffsetValues(rTsectPnt));
                        SER(pSurf1->EvaluatePoint( sUVPt,rTsectPnt.CrvPos()));
                    }
                    sPnt = rTsectPnt.CrvPos();
                }

                // Drop sPnt to pCurve, 2d or 3d
                SmSolution sSData[8];
                SmSolutionArray sSolutions(8,sSData);
                SER( pCurve->GlobalPointSolve(sIvl, SM_SO_MINIMIZE,
                     sPnt, dTol, NULL, NULL, SM_SR_SINGLE, sSolutions ));
                if ( sSolutions.GetSize() != 1 ) SER(SM_ERR);
                double dT = sSolutions[0].m_vStart[0];

#ifdef SM_DEBUG_CODE
                if ( DebugLevel() > 0 ) {
                    if ( FALSE ) {
                        smgfx_Erase();
                        smgfx_SetLook( 1,2, 0,0,0 );
                        this->GetFilletSolver()->GetFilletExecutive()->GetTargetBrep()->Draw(TRUE);
                        sm_GraphicsLoop();
                    }
                    SmPoint3d sP1, sP2;
                    SmPoint2d sUV1 = rTsectPnt.UVPos(0);
                    pFS->GetSurface(0)->GetBaseSurface()->EvaluatePoint(sUV1,sP1);
                    smgfx_SetLook( 2,5, 0,1,0 ); sP1.Draw(); sm_GraphicsLoop();
                    if ( pOptMarchDir ) {
                        static constexpr double dVecMag = 10;
                        SmVector3d sMultVec = dVecMag * (*pOptMarchDir);
                        sMultVec.Draw( &sP1 ); sm_GraphicsLoop();
                        sm_GraphicsLoop();
                    }
                    if (FALSE) {
                        smgfx_SetLook( 1,2, 1,1,0);
                        pFS->GetSurface(0)->DrawUV(1,1); sm_GraphicsLoop();
                        sm_GraphicsLoop();
                    }
                    SmPoint2d sUV2 = rTsectPnt.UVPos(1);
                    pFS->GetSurface(1)->GetBaseSurface()->EvaluatePoint(sUV2,sP2);
                    smgfx_SetLook( 2,5, 1,0,0);
                    sP2.Draw();
                    if (FALSE) {
                        smgfx_SetLook( 1,2, 1,0,1); pFS->GetSurface(1)->DrawUV(1,1); sm_GraphicsLoop();
                    }
                    sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

                // Adjust for point being off the end of the curve.
                if ( sIvl.IsValueOnBoundary( dT ) )
                {
                    // (Note, this can get bad values if on wrong side of a seam. [B398])
                    SmPoint3d sPV[2];
                    SER( pCurve->Evaluate( dT, 1,1, sPV ) );
                    double dAdjustment;
                    SER( smgu_LineClosestPoint( sPV[0], sPV[1], sPnt, dAdjustment ));
                    dT += dAdjustment;
                }

                if (    b3DTest
                     && bIsClosedSolverEdge
                     && sIvl.IsValueOnBoundary(dT) )
                {
                    // Do this logical test if dT is on boundary of a
                    // closed edge. (Please noted that the rail is already
                    // in the same direction as the solver)

                    if ( pEU->GetOrientation() == SM_OT_OPPOSITE )
                    {
                        dT = sIvl.GetMax();
                    }
                }

                // Decide if and where to insert this param
                ULONG lIndexOfInsert = sProjParams.GetSize();
                SmBoolean bDoInsertion = TRUE;
                for ( ULONG k=0; k<sProjParams.GetSize(); k++ )
                {
                    if ( smos_Fabs(dT-sProjParams[k]) < SM_EFF_ZERO_SQRT*dScale)
                    {
                        bDoInsertion = FALSE;
                        break;
                    }
                    else if (dT < sProjParams[k]) {
                        lIndexOfInsert = k;
                        break;
                    }
                }
                if ( bDoInsertion )
                {
                    sProjParams.InsertAt(lIndexOfInsert, dT);
                    rTsectPnt.m_dCurveParameter = -SM_BIG_DOUBLE;
                    sTsectPnts.InsertAt( lIndexOfInsert, &rTsectPnt );
                }
            } // end loop on both edgeuses of this rail
        } // end loop on all rails on this side
    } // end loop on both main rails, sorting SmTsectPnts

    // State:
    // - sProjParams: sorted parameters of all rail-ends (noted by Vertexuses);
    // - sTsectPnts contains rTsectPnt's of all Vertexuse's;
    // - each sTsectPnt
    // - for each Vertexuse, its rTsectPnt.m_dCurveParameter = -SM_BIG_DOUBLE;

    SmBoolean bFilletCreated = FALSE;
    if ( bIsAnalyticFillet && m_eType != SM_FG_TANGENT_ROLLOVER )
    {
        // Create analytics
        if (   pFS->CalcConeFilletGeom( sTsectPnts, this ) == SM_SUCCESS
            || pFS->CalcTorusFilletGeom( sTsectPnts, this ) == SM_SUCCESS )
            bFilletCreated = TRUE;
    }
    if ( !bFilletCreated )
    {
//        if (m_eType != SM_FG_CORNER_FILLET) {
            pFS->SetupOffsetExtension( pSurf1, sDomain1 );
            pFS->SetupOffsetExtension( pSurf2, sDomain2 );
//        }
        // Setup FilletIntersector
        SmFilletIntersector sFI(*pSurf1,sDomain1,
                                *pSurf2,sDomain2,
                                *pFS,this);

        SmBoolean bExtendBefore = FALSE;
        SmBoolean bExtendAfter = FALSE;
        if ( this == pFS->GetFirstFilletGeom() )
            bExtendBefore = pFS->m_bExtendBefore;
        if ( this == pFS->GetLastFilletGeom() )
            bExtendAfter = pFS->m_bExtendAfter;

        SmBSplineCurve *p3DCurve = NULL ;
        SmTsectCurveType eCurveType;
        double dDeviation;
        SmVector3d sDir;
        if (pOptMarchDir == NULL)
        {
            if (m_vRails[0]->GetCurve()) {
                SER( m_vRails[0]->GetEndTangent( TRUE, sDir ));
            }
            else {
                sDir = sTsectPnts[0]->CrvDeriv();
            }
            pOptMarchDir = &sDir;
        }
        SmBSplineCurve *pSurfaceUVCurves[2];

        // DoPointIntersection() can fail because of TsectPnts being on the
        // wrong sides of seams.  Unfortunately, at this point, it's not
        // obvious which side of the seam is correct.  All we can do is:
        // if the call fails, check whether any TsectPnts are on seams,
        // and if so, try again from the other side of the seam.
        // Note, jumping a seam can mean reordering the TsectPnts.


        SmBoolean bSuccess = TRUE;
        SmBoolean bDoneCheckingSeams = FALSE;
        int iTry = 0;
        while ( ! bDoneCheckingSeams )
        {

            // Note: this call puts all of its results directly into
            // 'this' FilletGeom, and not in the output arguments.
            SmStatus eStat = sFI.DoPointIntersection(crContext,
                                        sTsectPnts,
                                        bExtendBefore,
                                        bExtendAfter,
                                        pOptMarchDir,
                                        NULL,
                                        SM_CAST_APPROXTOL3D_PTR(&dTolerance ),
                                        &dAngTol,
                                        p3DCurve,
                                        pSurfaceUVCurves[0],
                                        pSurfaceUVCurves[1],
                                        eCurveType,
                                        dDeviation);

            if (eStat != SM_SUCCESS)
            {
                //if (this->m_pFilletSurface)
                //{
                //    delete this -> m_pFilletSurface;
                //    this->m_pFilletSurface = NULL;
                //}
                SER(eStat);
            }

            // DoPointIntersection inits TsectPnt curve param to -1,
            // and resets it >= 0 if it was successfully processed.
            // Check for errors here.

            bSuccess = TRUE;
            for ( ULONG ii=1; ii<sTsectPnts.GetSize(); ii++ )
            {
                SmTsectPnt * pTSP = sTsectPnts[ii];
                if ( pTSP->m_dCurveParameter < 0.0 )
                  { bSuccess = FALSE; break; }
            }

            if ( bSuccess )
              { break; }  // Break out of while not bDoneCheckingSeams

            if ( ! bSuccess ) //cbi ...
            {
                iTry++;

                // A seam would have to be the first or last TsectPnt.

                ULONG lLastTspIdx = sTsectPnts.GetSize() - 1;

                switch ( iTry )
                {
                  case 1: 
                    {
                      if ( sm_CheckSeamOn( pSurf1, sTsectPnts[0]->UVPos(0), dTolerance, eSurfClosure1 ) )
                        {
                          sm_MoveTSP( sTsectPnts, 0, lLastTspIdx );
                          break;
                        }
                      iTry++;  // if it wasn't on a seam, try the other one.
                    }
                  case 2: 
                    {
                      if ( sm_CheckSeamOn( pSurf1, sTsectPnts[lLastTspIdx]->UVPos(0), dTolerance, eSurfClosure1 ) )
                        {
                          sm_MoveTSP( sTsectPnts, lLastTspIdx , 0);
                          break;
                        }
                      iTry++;  // if it wasn't on a seam, try the other one.
                    }
                  case 3: 
                    {
                      if ( sm_CheckSeamOn( pSurf2, sTsectPnts[0]->UVPos(1), dTolerance, eSurfClosure2 ) )
                        {
                          sm_MoveTSP( sTsectPnts, 0, lLastTspIdx );
                          break;
                        }
                      iTry++;  // if it wasn't on a seam, try the other one.
                    }
                  case 4: 
                    {
                      if ( sm_CheckSeamOn( pSurf2, sTsectPnts[lLastTspIdx]->UVPos(1), dTolerance, eSurfClosure2 ) )
                        {
                          sm_MoveTSP( sTsectPnts, lLastTspIdx, 0 );
                          break;
                        }
                      bDoneCheckingSeams = TRUE;  // that's all.
                    }
                  default:
                    {
                      bDoneCheckingSeams = TRUE;  // that's all.
                      break;
                    }

                } // end switch
            } // end if not bSuccess

        } // end while not bDoneCheckingSeams

        if ( ! bSuccess )
          {
            return SM_ERR;
          }

    } // end if Not bFilletCreated


    // Calculate fillet edges of type SM_FE_CROSS_SECTION
    SmTArray<SmFilletEdge*> sEdges;
    this->GetFilletEdges( sEdges );

    for ( ULONG jj=0; jj<sEdges.GetSize(); jj++ )
    {
        SmFilletEdge * pE = sEdges[jj];
        if ( pE->GetFilletEdgeType() == SM_FE_CROSS_SECTION )
        {
            SER(pE->CalcCrossSection( crContext ));
        }
    }
    m_eStatus = SM_FIL_PROCESSED;

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetLook(1,2, 1,0,0);
        m_pFilletSurface->DrawUV(1,1);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmFilletGeom::ReCalcFilletGeom

/*******************************************************************//**
PURPOSE: Compute the fillet point corresponding to the intersection
     of a rail and an edge given a starting point on the edge.

NOTES:  This method assumes that pEdgeuse connects to the surface
     corresponding to the lRailIndex.
     rTsectPnt contains the edge parameter and other information about
     the intersection of the edge and the rail.

     This method sets rTsectPnt.m_dCurveParameter to rdEdgeuserParameter,
     which is nonstandard usage for that field.
***********************************************************************/
SmStatus SmFilletGeom::RailEdgeuseIntersect
 (ULONG        lRailIndex,           // in : target rail index
  SmEdgeuse  * pEdgeuse,             // in : target edgeuse->edge->curve to intersect
  double       dGuessEdgeParameter,  // in : Edgeuse->curve guess parameter
  SmBoolean  & rbFoundIntersection,  // out: TRUE=found an intersection
  SmTsectPnt & rTsectPnt,            // out: Contains solution
  double     & rdEdgeuseParameter)   // out: edgeuse Parameter of intersection
{
  SmFilletSolver *pFS = this->GetFilletSolver();
  NER( pFS );

  // Tell our FilletSolver to use our surfaces.
  // If there's anything there already, restore it when we're done.
  SmFilletGeom *pSavedFG = pFS->GetFilletGeomForSurfaces();

  pFS->SetFilletGeomForSurfaces( this );

  SmStatus eStat = pFS->RailEdgeuseIntersect(lRailIndex,           // in : target rail index
                                             pEdgeuse,             // in : target edgeuse->edge->curve to intersect
                                             dGuessEdgeParameter,  // in : Edgeuse->curve guess parameter
                                             rbFoundIntersection,  // out: TRUE=found an intersection
                                             rTsectPnt,            // out: Contains solution
                                             rdEdgeuseParameter) ; // out: edgeuse Parameter of intersection

  pFS->SetFilletGeomForSurfaces( pSavedFG );

  return eStat;

} // end SmFilletGeom::RailEdgeuseIntersect

/*******************************************************************//**
PURPOSE: Deal with original Faces being split in our rails and SideEUs.

NOTES:
   If either rail of this FilletGeom claims to be in pOrigFace, but it actually lies
   in pNewFace, then set its OriginalFace to be pNewFace instead.
***********************************************************************/
SmStatus SmFilletGeom::UpdateSplitFace( SmFace *pOrigFace, SmFace *pNewFace )
{
  SmFilletEdge *pThisRail = NULL;
  SmTArray< SmFilletEdge* > sAllRails;

  // For both rails:
  for ( ULONG ii=0; ii<2; ii++ )
  {
      // Collect all extra rails for this rail.
      sAllRails.ReSet();
      sAllRails.Add( this->GetRail( ii ) );
      sAllRails.Append( (ii==0) ? m_vOtherRails1 : m_vOtherRails2 );

      for ( ULONG kk=0; kk<sAllRails.GetSize(); kk++ )
      {
          pThisRail = sAllRails[kk];
          pThisRail->UpdateSplitFace( pOrigFace, pNewFace );

      } // end for sAllRails

  } // end for both rails

  // Also SideEUs.
  for ( ULONG ii=0; ii<m_vSideEUs.GetSize(); ii++ )
  {
      SmFilletEdgeuse *pFEU = m_vSideEUs[ii];
      SmFilletEdge    *pFE  = SM_CAST_PTR( SmFilletEdge, pFEU->GetOwner() );
      if ( pFE != NULL )
        { pFE->UpdateSplitFace( pOrigFace, pNewFace ); }
  }

  return SM_SUCCESS;

} // end SmFilletGeom::UpdateSplitFace

/*******************************************************************//**
PURPOSE: Insert the intersection topology of the fillet curves into
    the original m_pTargetBrep and set up the m_vTI Topology Intersector Relationships.

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::InsertIntersectionTopology
  ()
{
  // fillet locals
  ULONG ii, jj ;
  SmFilletSolver        *pFS            = m_pFilletSolver;
  SmFilletExecutive     *pFilExec       = pFS->GetFilletExecutive();
  SmTopologyIntersector &rTI            = pFilExec->GetTopologyIntersector();
  SmSurface             *pFilletSurface = GetFilletSurface();

  // locals for RegisterExistingSSI() call
  SmTArray<SmAObject*> sAllEdges;
  SmTArray<SmCurve*> s3DCurves, sUVCurves1, sUVCurves2;
  SmTArray<SmTsectCurveType> sTsectTypes;
  SmTArray<double> sDeviations;

  SmBoolean bTopologyWasDeleted = FALSE;
  SmTArray< SmEdge* > aDeletedEdges;
  SmTArray< SmFace* > aNewFacesBrep;

#ifdef SM_DEBUG_CODE
static ULONG lCount = 1 ; lCount++ ; 
static ULONG lDebugCount = 0 ;
  TCHAR sBuff[SM_TBLOCK_SIZE];
#endif // SM_DEBUG_CODE

  // Merge each rail into originating face
  for(ii=0; ii<2; ii++)
    {
      // Collect all extra rails for case ii
      SmTArray< SmFilletEdge* > sAllRails;
      sAllRails.Add( m_vRails[ii] );
      sAllRails.Append( (ii==0) ? m_vOtherRails1 : m_vOtherRails2 );

      // for every rail
      for (ULONG kk=0; kk<sAllRails.GetSize(); kk++)
        {
          // locals - rail, Face, Brep, and edges to merge
          SmFilletEdge &rThisRail = *sAllRails[kk];
          SmFace       *pBaseFace = rThisRail.GetOriginalFace();
          SmBrep       *pBaseBrep = pBaseFace ? pBaseFace->GetBrep() : NULL ;
          SmEdge       *pFillEdge = rThisRail.GetFilletBrepEdge();
//cbi: can't we do this before the loop?
          pFilExec->m_pAttributeE->GetUsers( sAllEdges );
          ULONG lIndex;

          // skip degenerate rails and cases with bad model consistency
          if (   rThisRail.GetCurve()->IsDegenerate()
              || !pBaseFace
              || !pFillEdge
              || !sAllEdges.FindElement(pFillEdge,lIndex))
            { continue; }

#ifdef SM_DEBUG_CODE
          // draw rail being merged(red), OriginalBrep(blue), FilletSurface(Green)
          if ( DebugLevel() > 0 || lCount == lDebugCount)
            {
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,0 ); pBaseBrep->Draw(TRUE) ; sm_GraphicsLoop() ;   
              smgfx_SetLook( 1,2, 0,1,0); pFilletSurface->Draw() ; pFilletSurface->DrawUV(10,10) ; sm_GraphicsLoop() ;
              smgfx_SetLook( 1,2, 0,1,1); pBaseFace->GetSurface()->Draw() ; pBaseFace->GetSurface()->DrawUV(10,10) ; sm_GraphicsLoop() ;
              smgfx_SetLook( 3,3, 1,0,0 ); rThisRail.GetCurve()->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook( 5,6, 0,0,1 ); if(pFillEdge) { pFillEdge->Draw(); } sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Enable editing in the Brep.
          SmTemporaryChangeValue<SmBoolean> TempEdit(pBaseBrep->m_bEditingEnabled, TRUE);

// Remove Composites
// #ifndef SM_NO_COMPOSITES
//          pBaseBrep->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES

          // merge rail curve into original Brep

          // Put an attribute on the base Face, to check whether it gets split.
          SmAttribute sFaceAttr( 777, SM_AB_STANDALONE_REFERENCE );
          pBaseFace->AddAttribute( &sFaceAttr );

          SmStatus eStat = rTI.MergeCurveOnSurfaces
             (pBaseFace->GetSurface(),        // in : 1st surface
              NULL,                           // in : 2nd surface
              pFillEdge->GetTolerance(),      // in : curve tolerance
              rThisRail.GetCurve(),           // in : 3d curve to merge
              rThisRail.GetOriginalUVCurve(), // in : opt associated surface1 UVTrimCurve
              NULL,                           // in : opt associated surface2 UVTrimCurve
              NULL,                           // in : opt existing edge which already corresponds
                                              //      to 3Dcurve on 1st surface
              pFillEdge,                      // in : opt existing edge which already corresponds
                                              //      to 3Dcurve on 2nd surface
              bTopologyWasDeleted,            // out: flag indicating whether anything was deleted
              &aDeletedEdges);                // out: edges deleted by squeezing vertices

          // If MergeCurveOnSurface deleted an edge that rThisRail points to,
          // clear out that pointer.
          rThisRail.ClearFilletBrepEdges( aDeletedEdges );

          // If any Faces were split, check whether objects sitting on the
          // original Face are now in the new split Face.
          pFilExec->UpdateTopologyChanges( &sFaceAttr, pBaseFace );

#ifdef SM_DEBUG_CODE
          // draw rail being merged(red), OriginalBrep(blue), FilletSurface(Green)
          if ( DebugLevel() > 10 || lCount == lDebugCount)
            {
              smos_sprintf(sBuff,_T("\n\nSmFilletGeom::InsertIntersectionTopology(): After MergeCurvOnSurface(), Rail # %ld,  %ld\n"), ii, kk);
              smos_WriteBuffer(sBuff);

              rTI.GetPrimaryBrep()->Dump();
              rTI.GetOtherBrep()->Dump();
              rTI.Dump();

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pBaseBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0); pFilletSurface->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0); pFilletSurface->DrawUV(10,10) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,5, 1,0,0); rThisRail.GetCurve()->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,0); if(pFillEdge) { pFillEdge->Draw(); } sm_GraphicsLoop() ;
              rTI.Draw( 3,4, 5,6 );
              sm_GraphicsLoop();

            }
#endif // SM_DEBUG_CODE

          if ( eStat != SM_SUCCESS )
            {
              //cbi: type?
              //cbi  Our own status is UNPROCESSED.  set that?
              // This avoid crash when calling this function when m_bEditingEnabled is false
              SmEdgeuse* eu = GetFilletSolver()->GetEdgeuse(0);
              if( eu != NULL ) 
                {
                  RecordFilletError(SM_FILERR_BAD_MERGE,
                                    eu->GetEdge(), // filleted edge
                                    pBaseFace, &rThisRail, pFilletSurface,
                                    _T("SmFilletGeom::InsertIntersectionTopology(): Failure to merge fillet rail curve into side face of Target Brep")
                                   );
                }
              SER( eStat );
            }

          // Right now Just register no intersections - because s3DCurves is empty
          SER( rTI.RegisterExistingSSI(pBaseFace->GetSurface(), // in : surf1
                                       pFilletSurface,          // in : surf2
                                       s3DCurves,               // in : 3d xsect curves
                                       sUVCurves1,              // in : associated surf1 UVTrimCurves
                                       sUVCurves2,              // in : associated surf2 UVTrimCurves
                                       sTsectTypes,             // in : associated intersection curve types
                                       sDeviations));           // in : associated intersection curve deviations


        } // end iter kk, every edge making up the current rail
    } // end iter ii, merge both rails into originating face

  // Merge every SideEU into the original model
  for(ii=0; ii<m_vSideEUs.GetSize(); ii++)
    {
      SmFilletEdge *pFE   = (SmFilletEdge*)m_vSideEUs[ii]->GetEdge();
      SmFace       *pFace = pFE->GetOriginalFace();
      SmEdge       *pEdge = pFE->GetFilletBrepEdge();
//cbi_CEdge: can't we do this before the loop?
      pFilExec->m_pAttributeE->GetUsers(sAllEdges);
      ULONG lIndex;

      // skip cases where the bounding edge is not on an originalFace
      if ( !pFace || !sAllEdges.FindElement(pEdge,lIndex) )
        { continue; }

#ifdef SM_DEBUG_CODE
      SmBrep       *pBrep = pFace ? pFace->GetBrep() : NULL ;
          // draw rail being merged(red), OriginalBrep(blue), FilletSurface(Green)
          if ( DebugLevel() > 0 || lCount == lDebugCount)
            {
              ULONG di ;
              for(di=0;di<m_vSideEUs.GetSize();di++)
                { if(m_vSideEUs[ii]) ((SmFilletEdge*)m_vSideEUs[ii])->Dump() ; }

              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,0 ); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;   
              smgfx_SetLook( 1,2, 0,1,0); if(pFilletSurface) pFilletSurface->Draw() ; pFilletSurface->DrawUV(10,10) ; sm_GraphicsLoop() ;
              smgfx_SetLook( 1,2, 0,1,1); if(pFace) pFace->GetSurface()->Draw() ; pFace->GetSurface()->DrawUV(10,10) ; sm_GraphicsLoop() ;
              smgfx_SetLook( 3,3, 1,0,0 ); for(di=0;di<m_vSideEUs.GetSize();di++)
                                             { if(m_vSideEUs[ii] && ((SmFilletEdge*)m_vSideEUs[ii])->GetCurve()) ((SmFilletEdge*)m_vSideEUs[ii])->GetCurve()->Draw() ; sm_GraphicsLoop() ; }
              smgfx_SetLook( 5,6, 0,0,1 ); if(pFE) { pFE->Draw(); } sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

// Remove Composites
// #ifndef SM_NO_COMPOSITES
//       // tell Brep to make composites when splitting edges and faces
//       pBrep->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES

      // Put an attribute on the base Face, to check whether it gets split.
      SmAttribute sFaceAttr( 778, SM_AB_STANDALONE_REFERENCE );
      pFace->AddAttribute( &sFaceAttr );

      SER(rTI.MergeCurveOnSurfaces(pFace->GetSurface(),       // in : 1st surface of surface/surface intersection
                                   NULL,                      // in : 2nd surface of surface/surface intersection
                                   pEdge->GetTolerance(),     // in : min length for non-degenerate curve. Degen Crv treated as pt.
                                   pFE->GetCurve(),           // in : surf/surf 3D xsect curve
                                   pFE->GetOriginalUVCurve(), // in : opt associated UV curve for 1st surface
                                   NULL,                      // in : opt associated UV curve for 2nd surface
                                   NULL,                      // in : opt already existing edge corresponding to 3DCurve on pSurface,
                                                              //      NULL to ignore
                                   pEdge,                     // in : opt already existing edge which corresponding to 3DCurve on pOtherSurface,
                                                              //      NULL to ignore
                                   bTopologyWasDeleted,       // out: TRUE = multiply mated Vertices or Edges squeezed in MergeCurveClasses().
                                                              //      FALSE= no Vertices or Edges squeezed
                                   &aDeletedEdges ));         // out: ptrs to deleted edges

      // If MergeCurveOnSurface deleted an edge that rThisRail points to,
      // clear out that pointer.
      pFE->ClearFilletBrepEdges( aDeletedEdges );

      // If any Faces were split, check whether objects sitting on the
      // original Face are now in the new split Face.
      pFilExec->UpdateTopologyChanges( &sFaceAttr, pFace );

      // Right now Just register no intersections - because s3DCurves is empty
      SER(rTI.RegisterExistingSSI(pFace->GetSurface(),  // in : surf1
                                  pFilletSurface,       // in : surf2
                                  s3DCurves,            // in : 3d xsect curves
                                  sUVCurves1,           // in : associated surf1 UVTrimCurves
                                  sUVCurves2,           // in : associated surf2 UVTrimCurves
                                  sTsectTypes,          // in : associated intersection curve types
                                  sDeviations));        // in : associated intersection curve deviations

#ifdef SM_DEBUG_CODE
      // draw bounding edge being merged(red), OriginalBrep(blue), FilletSurface(Green)
      if ( DebugLevel() > 10 )
        {
          smos_sprintf(sBuff,_T("\n\nSmFilletGeom::InsertIntersectionTopology(): After MergeCurvOnSurface(), SideEU # %ld\n"), ii);
          smos_WriteBuffer(sBuff);

          rTI.GetPrimaryBrep()->Dump();
          rTI.GetOtherBrep()->Dump();
          rTI.Dump();

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); if(pBrep) { pBrep->Draw(TRUE) ; }
                                     else      { rTI.GetPrimaryBrep()->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(0,1,0); pFilletSurface->Draw() ; pFilletSurface->DrawUV(10,10) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 1,0,0) ; pFE->GetCurve()->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 1,0,0) ; if(pEdge) { pEdge->Draw(); } sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end iter ii, every m_vSideEUs inserting edges into the original Brep

  // Register FilletSurface/OrigBrepSurface intersections
  // with all OriginalBrepSurfaces around both to-be-filleted-edge
  // vertices when those vertexCorners are not being capped.

  // for both to-be-filleted-edge vertices
  SmVertex *pV = NULL, *pV0 = NULL ;
  for(ii=0; ii<2; ii++)
    {
      pV0 = pV ;
      pV  = pFS->GetVertex(ii);

      // skip closed fillet edge 2nd vertices - its already done
      if(pV == pV0) { continue ; }

      // when the Original vertex exists and there is no filletCap surface
      if (pV && !m_bCapped[ii])
        {
          // for every OriginalFace connected to the vertex
          SmTArray<SmFace*> sVertFaces;
          pV->GetFaces(sVertFaces);
          for(jj=0; jj<sVertFaces.GetSize(); jj++)
            {
              SmFace *pVF = sVertFaces[jj];

#ifdef SM_DEBUG_CODE
              // draw vertex(red), OriginalBrep(blue), new FilletSurface(green), OriginalFace(black),
              if ( DebugLevel() > 10 )
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,3, 0,0,1); pVF->GetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,5, 1,0,0); pV->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,5, 0,1,0); pFilletSurface->DrawUV(10,10); sm_GraphicsLoop();
                  smgfx_SetLook(2,5, 0,0,0); pVF->Draw(SM_DM_CROSSHATCH,12,12); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
              // Right now Just register no intersections - because s3DCurves is empty
              SER(rTI.RegisterExistingSSI(pVF->GetSurface(),  // in : surf1
                                          pFilletSurface,     // in : surf2
                                          s3DCurves,          // in : 3d xsect curves
                                          sUVCurves1,         // in : associated surf1 UVTrimCurves
                                          sUVCurves2,         // in : associated surf2 UVTrimCurves
                                          sTsectTypes,        // in : associated intersection curve types
                                          sDeviations));      // in : associated intersection curve deviations
            } // end iter every OriginalVertex->Face
        } // end no filletCapSurface for this vertex check
    } // end iter both to-be-filleted-edge vertices

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::InsertIntersectionTopology

/*******************************************************************//**
PURPOSE: DoSurfaceFilleting helper function to match a context
         to a NewMarkAndLock value

NOTES: 1. returns NewMarkAndLock for given context or NULL for no match
       2. same function appears in both SmFilletGeom.cpp and
                                        SmFilletExecutive.cpp (yuk)
***********************************************************************/
SmNewMarkAndLock * sm_FindMarkForContext2
 ( SmTArray<SmNewMarkAndLock *> & rMarkLocks,  // in : list of marks to check
   const SmContext              * cpContext)   // in : context to find amongst marks
{
  ULONG ii ;
  for(ii=0;ii<rMarkLocks.GetSize();ii++)
    {
      if(cpContext == rMarkLocks[ii]->GetContext())
        { return( rMarkLocks[ii] ) ; }
    }

  return NULL ;

} // end sm_FindMarkForContext2

/*******************************************************************//**
PURPOSE: Trim the original surfaces in cases where the fillet edge
         has split the face and collects faces for FacesKept and FacesDelete lists.

NOTES: Collects faces bounded by edges which are marked:[eOrigMarkType, eFilletMarkType]
       does not increment Context::Mark values
***********************************************************************/
SmStatus SmFilletGeom::TrimOriginalSurfaces
 (SmTArray<SmFace*>            & rFilletBrepFacesKept, // out: accumulating list of faces in m_pFilletBrep to keep
  SmTArray<SmFace*>            & rOriginalFacesDelete, // out: accumulating list of faces from OrigBrep(s) to delete
  SmTArray<SmNewMarkAndLock *> & rMarkLocks)           // in : list of all marks for all contexts used in this fillet
                                                       //      the first entry is for SmFilletExecutive::m_crContext
{
#ifdef SM_DEBUG_CODE
  ULONG di ;
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj, kk, ll ; // loop indices
  SmFilletSolver        * pFS             = m_pFilletSolver;
  SmFilletExecutive     * pFE             = pFS->GetFilletExecutive();
  SmTopologyIntersector & rTI             = pFE->GetTopologyIntersector();
// Remove Composites
//   SmSurface             * pFilletSurface  = GetFilletSurface();
  SmMarkType              eFilletMarkType = rMarkLocks[0]->GetMarkType() ;
  SmBoolean               bBadTrim        = FALSE;

  SmTArray<SmFace*>    sDelFaces;
  SmTArray<SmFace*>    sCollectedFaces;
  SmTArray<SmEdgeuse*> sEdgeuses;
// Remove Composites
//  SmTArray<SmFace*>    sFGFaces;
//  SmTArray<SmEdge*>    sCEdges;

// Remove Composites
// //cbi_CEdge: without Composites, there is only one.
//   // get all m_pFilletBrep->Faces that use pFilletSurface
//   SmBrep::GetFacesOfSurface(pFilletSurface, sFGFaces);  // Faces from the FilletGeom in pFE->m_pFilletBrep

  // For both Fillet sides (both rails):
  for(ii=0;ii<2;ii++)
    {
      // get sAllRails on this side. Usually just m_vRails[ii], but also m_vOtherRails
      //                             if original rail was split into multiple segments by rollover
      SmTArray<SmFilletEdge*> sAllRails;
      sAllRails.Add   ( m_vRails[ii] );
      sAllRails.Append( ii==0 ? m_vOtherRails1 : m_vOtherRails2 );

      // Get the surface which the rail curve is on.
      //    GWC note: when called from SurfaceSurfaceFilleting() => DoSurfaceFilleting()
      //              the original surfaces can come from two different Breps (yuk)
      //              which means there may be 3 different Breps and 3 different contexts in play
      //              1. m_pFE->m_pFilletBrep in context m_pFE->m_crContext
      //              2. m_vRails[0]->OrigFace->Brep  and its context
      //              3. m_vRails[1]->OrigFace->Brep  and its context (usually the same but can be different)
      //                  often The OrigFaces all come from the same Brep - but that's not a rule
      //              This is important for marking - each mark has to correllate with its associated context.
      SmFace    * pOrigFace = m_vRails[ii]->GetOriginalFace();
      const SmTArray< SmFace *> & crSpltFaces = m_vRails[ii]->GetSplitFaces();

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 )
        {
          SmBrep *pOrigBrep = pOrigFace->GetBrep() ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; if(pOrigBrep) pOrigBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<sAllRails.GetSize();di++) sAllRails[di]->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; m_vRails[(ii+1)%2]->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; pOrigFace->DrawUV() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // for every rail on this fillet side - usually 1 sometimes more
      for ( jj=0; jj < sAllRails.GetSize(); jj++ )
        {
          SmFilletEdge * pRail = sAllRails[jj];

          // skip rails not attached to OrigBrep faces
          if (pRail->GetFilletEdgeType() != SM_FE_RAIL)
            {  continue; }

          // get the m_pFilletBrep->Edge mapped to this m_pPseudoBrep->pRail
          SmEdge *pFBEdge = pRail->m_pFilletBrepEdge1;

          // skip cases where m_pFilletBrep->Edge is missing - gwc: then is something wrong?
          if ( pFBEdge == NULL )
            { continue; }

          // Get mated m_pTargetBrep->Edge for this m_pFilletBrep->Edge
          SmEdge *pBrepEdge = (SmEdge*)rTI.GetBrepMate( pFBEdge );

          // when FilletBrep->Edge is not mated to an OrigBrep->Edge - see if it's mated to a composite sibling
          if ( pBrepEdge == NULL )
            {
              // Get FilletBrep->Edge->Curve
              SmCurve *pBrepEdgeCurve = pFBEdge->GetCurve();

              // skip FilletBrep->Edge->Curves owned by the FilletBrepEdge
              if ( pBrepEdgeCurve->GetOwner() == pFBEdge )
                { continue; }

// Remove Composites
//              //   Note: this is never hit in prog_test:
//              // arrive here when FilletBrep->Edge->Curve is part of a composite edge
//              SmCEdge *pCEdge = (SmCEdge*)pBrepEdgeCurve->GetOwner();
//              pCEdge->GetEdges( sCEdges );
//
//              // for every member of the composite edge - look for a mated Edge from OrigBrep
//              for ( kk=0; kk<sCEdges.GetSize(); kk++ )
//                {
//                  SmEdge *pNextBrepEdge = sCEdges[kk];
//                  SmEdge *pOtherSurfEdge = (SmEdge*)rTI.GetBrepMate( pNextBrepEdge );
//                  if ( pOtherSurfEdge )
//                    {
//                      pFBEdge   = pNextBrepEdge;
//                      pBrepEdge = pOtherSurfEdge;
//                      break;
//                    }
//                } // end iter kk, every pBrepEdgeCurve->OwningCEdge->member
            } // end if pBrepEdge Null, looking for CEdge etc.

          // skip m_pFilletBrep->Edges not mated to an Orig m_pTargetBrep->Edge
          if ( pBrepEdge == NULL )
            { continue; }

          // arrive here when pFBEdge   = m_pFilletBrep->Edge attached to fillet rail curve
          //                  pBrepEdge = mated Original m_pTargtBrep->Edge containing the OrigSurface being filleted

          SmEdgeuse *pPrimEU = pFBEdge->GetPrimaryEdgeuse();
          SmFace    *pFBFace = pPrimEU->GetFace();

          // Collect all faces reachable from pFBFace in m_pFilletBrep -- all new Fillet faces.
          SmTopologyTraverser sFBTraverser;
          SER( sFBTraverser.CollectFaces( pFBFace,             // in : Seed face (gets marked)
                                          sCollectedFaces,     // out: List of connected m_pFilletBrep->faces (Get marked)
                                          eFilletMarkType ));  // in : specify mark for target objects (not incremented)

#ifdef SM_DEBUG_CODE
          if ( DebugLevel() > 0 )
            {
              SmBrep *pOrigBrep = pOrigFace->GetBrep() ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2) ; if(pOrigBrep) pOrigBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<sAllRails.GetSize();di++) sAllRails[di]->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; pOrigFace->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,0,1) ; pFBEdge->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,1) ; pPrimEU->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; pFBFace->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,1) ; for(di=0;di<sCollectedFaces.GetSize();di++) sCollectedFaces[di]->DrawUV() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // accumulate output rFilletBrepFacesKept array += unique m_pFilletBrep->sCollectedFaces
          for(kk=0; kk<sCollectedFaces.GetSize(); kk++ )
            {
              // build unique list of KeptBrepFaces
              rFilletBrepFacesKept.AddUnique( sCollectedFaces[kk] );

            } // end iter kk, every sCollectedFaces building Unique list rFilletBrepFacesKept

          // Find the Face of the OrigBrep Edge that is 'inside' the
          // closed loop of common edges, which have been marked.
          // We find the binormal of the Fillet Edge, and compare
          // the binormals of the Brep Edge's Edgeuses.
          // They should be either parallel or anti-parallel.

          // get Orig m_pTargetBrep->Faces connected to the mated Orig m_pTargetBrep->Edge
          SmTArray<SmFace*>    sFaces1; 
          pBrepEdge->GetFaces( sFaces1 );

          // skip nonManifold orig m_pTargetBrep >Edges - not having two sides, they don't get trimmed
          if ( sFaces1.GetSize() != 2 )
            { continue; }

          // Get the m_pFilletBrep->Edge binormal pointing into the fillet
          SmExtent1d sIvl = pFBEdge->GetInterval();
          SmPoint3d  sMidPnt;
          SmVector3d sFilletBin;
          SER( pPrimEU->EvaluateBinormal(sIvl.Evaluate(0.5), // in : value within Edge->m_vInterval.
                                         FALSE,              // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                                             //      FALSE = get/create pUVTrimCurve to calc Surface points
                                         sMidPnt,            // out: 3D pt on m_pFilletBrep->Edge
                                         sFilletBin ));      // out: unit-vector pointing to m_pFilletBrep->Face interior from rBinormalPoint

          // Drop the m_pFilletBrep->Edge->MidPoint onto the Orig m_pTargetBrep->Edge->Curve (expect near zero drop distances)
          SmCurve  * pBrepEdgeCurve = pBrepEdge->GetCurve(); NER( pBrepEdgeCurve );
          SmBoolean  bSuccess;
          double     dDroppedParameter, dDistance;
          SmExtent1d sIvl2 = pBrepEdge->GetInterval();
          SER( pBrepEdgeCurve->DropPoint(sIvl2,                       // in : target curve allowed domain  
                                         sMidPnt,                     // in : Point to drop to curve 
                                         NULL,                        // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping. 
                                                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt. 
                                                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends. 
                                           pFBEdge->GetTolerance()    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance. 
                                         + pBrepEdge->GetTolerance(), //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT                                                                         
                                                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE 
                                                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT     
                                         NULL,                        // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve() 
                                         bSuccess,                    // out: TRUE = found a drop point 
                                         dDroppedParameter,           // out: found drop curve param 
                                         dDistance)) ;                // out: found drop distance
                                                                      // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol 
                                                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints 
                                                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior 
          if ( !bSuccess )
            { 
#ifdef SM_DEBUG_CODE
              if ( DebugLevel() > 0 )
                {
                  SmBrep *pOrigBrep = pOrigFace->GetBrep() ;

                  smgfx_Erase() ;
                  smgfx_SetLook(1,2) ; if(pOrigBrep) pOrigBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<sAllRails.GetSize();di++) sAllRails[di]->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 0,0,1) ; if(m_vRails[(ii+1)%2]) m_vRails[(ii+1)%2]->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; if(pOrigFace) pOrigFace->DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 1,0,0) ; sMidPnt.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 1,0,1) ; if(pBrepEdgeCurve) pBrepEdgeCurve->DrawParams(&sIvl2) ; sm_GraphicsLoop() ;

                  smgfx_SetLook(4,5, 0,0,1) ; pFBEdge->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 1,0,1) ; pPrimEU->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,0,0) ; pFBFace->DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 1,0,1) ; for(di=0;di<sCollectedFaces.GetSize();di++) sCollectedFaces[di]->DrawUV() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              SER(SM_ERR); 
            }

          // for every Orig m_pTargetBrep->pBrepEdge->Edgeuse - find the Face connected to the edgeuse no longer needed
          pBrepEdge->GetEdgeuses( sEdgeuses );
          for(kk=0;kk<sEdgeuses.GetSize();kk+=2)  // gwc: potential savings - we only need faces so only look at every other Edgeuse
                                                  //      later try changing kk++ to kk+=2
            {
              SmEdgeuse * pEU   = sEdgeuses[kk];

              // skip Orig m_pTargetBrep->pBrepEdge->Edgeuses not attached to the original Face(s).
              SmFace * pEUFace = pEU->GetFace();
              if ( ! ( pEUFace == pOrigFace || crSpltFaces.IsIn( pEUFace ) ) )
                { continue; }

              // Get OrigBrep->Edge->Binormal evaluation
              SmPoint3d  sPnt2;
              SmVector3d sBin2;
              SER(pEU->EvaluateBinormal(dDroppedParameter, // in : value within Edge->m_vInterval.
                                        FALSE,             // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                                           //      FALSE = get/create pUVTrimCurve to calc Surface points
                                        sPnt2,             // out: 3D pt on edge
                                        sBin2 ));          // out: unit-vector pointing to SmFace interior from rBinormalPoint
#ifdef SM_DEBUG_CODE
              if ( DebugLevel() > 0 )
                {
                  double dLen  = pEU->GetEdge()->GetCurve()->ApproximateLength(sIvl,5);
                  double dGain = 0.2 * dLen ;

                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,0,0 ); pEU->GetFace()->GetBrep()->Draw(TRUE); sm_GraphicsLoop();
                  smgfx_SetLook( 2,3, 0,0,1 ); pEU->GetFace()->DrawUV();          sm_GraphicsLoop();
                  smgfx_SetLook( 3,4, 1,0,0 ); pEU->Draw();                       sm_GraphicsLoop();
                  smgfx_SetLook( 5,6, 1,1,0 ); (dGain*sFilletBin).Draw(&sMidPnt); sm_GraphicsLoop(); // MidPt binormal into jjth rail->FilletBrep->Edge->PrimaryEU->Face
                  smgfx_SetLook( 7,8, 1,0,1 ); (dGain*sBin2).Draw(&sPnt2);        sm_GraphicsLoop(); // MidPt binormal into kkth m_pTargetBrep->Edge->Edgeuse->Face
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // when asked - reverse the binormal direction
              if ( pFS->GetReverseTrim() )
                { sBin2 = - sBin2; }

              // If this orig m_pTargetBrep->Edge->Edgeuse binormal points in the same dir as m_pFilletBrep Edge's binormal,
              // then the m_pTargetBrep->Edgeuse->Face is 'inside' the closed loop of common edges and
              // it (and all Faces connected to it) get marked for deletion.

              if ( sFilletBin.Dot( sBin2 ) > 0.2 ) // 0.2: just to avoid noise around 0.0.
                {
                  // Found a face that is 'inside' the closed loop
                  // of (marked) Edges in the target Brep.
                  // Collect all Faces inside the closed loop: those
                  // that can be traversed without crossing any marked Edges.
                  SmFace *pMatchedOrigFace = pEU->GetLoopuse()->GetLoop()->GetFace();
                  SmFace *pAdjOrigFace     = pEU->GetRadial()->GetFace();

                  SmTArray<SmFace*> sFaces2;
                  pBrepEdge->GetFaces( sFaces2 );
                  if ( sFaces2.GetSize() == 1 ) { continue; }

                  // get MarkType for MatchedOrigFace->Context
                  SmContext        * pOrigContext  = (SmContext *)pMatchedOrigFace->GetContext() ;
                  SmNewMarkAndLock * pOrigMarkLock = sm_FindMarkForContext2(rMarkLocks, pOrigContext) ;
                  SM_ASSERT_BREAK(pOrigMarkLock != NULL) ;
                  SmMarkType eOrigMarkType = pOrigMarkLock->GetMarkType() ;

                  // collect all faces connected to pMatchedOrigFace without stepping over marked vertices and edges
                  SmTopologyTraverser sTraverser;
                  SER( sTraverser.CollectFaces( pMatchedOrigFace,  // in : Seed face (gets marked)
                                                sCollectedFaces,   // out: List of connected faces (Get marked)
                                                eOrigMarkType ));  // in : specify mark for target objects (not incremented)
#ifdef SM_DEBUG_CODE
              if ( DebugLevel() > 0 )
                {
                  double dLen  = pEU->GetEdge()->GetCurve()->ApproximateLength(sIvl,5);
                  double dGain = 0.2 * dLen ;

                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,0,0 ); pEU->GetFace()->GetBrep()->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook( 3,4, 1,0,0 ); pEU->Draw();                       sm_GraphicsLoop();
                  smgfx_SetLook( 5,6, 1,1,0 ); (dGain*sFilletBin).Draw(&sMidPnt); sm_GraphicsLoop(); // MidPt binormal into jjth rail->FilletBrep->Edge->PrimaryEU->Face
                  smgfx_SetLook( 7,8, 1,0,1 ); (dGain*sBin2).Draw(&sPnt2);        sm_GraphicsLoop(); // MidPt binormal into kkth m_pTargetBrep->Edge->Edgeuse->Face
                  smgfx_SetLook( 2,3, 0,0,1 ); pMatchedOrigFace->DrawUV();          sm_GraphicsLoop();
                  smgfx_SetLook( 2,3, 1,0,1 ); for(di=0;di<sCollectedFaces.GetSize();di++) sCollectedFaces[di]->DrawUV() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

                  // for every sCollectedFaces, mark and add unique faces to sDelFaces list
                  for(ll=0;ll<sCollectedFaces.GetSize();ll++)
                    {
                      // check for faces that 'leaked out' of the closed loop - gwc: this is a problem case
                      if (sCollectedFaces[ll] == pAdjOrigFace)
                        {
                          bBadTrim = TRUE;
                        }

                      sDelFaces.AddUnique( sCollectedFaces[ll] );
                      sCollectedFaces[ll]->Mark(eOrigMarkType);

                    } // end iter ll, marking and adding unique sCollectedFaces to sDelFaces list
                } // end if binormals have generally the same direction - found a face inside the closed loop of edges
            } // end iter kk, every RailEdge->Edgeuse
        } // end iter jj, every RailEdge on this fillet side (usually only m_vRails[ii] but sometimes also m_vOtherRails_ii)
    } // end iter ii, both fillet sides (both rails)

  // when no connected OrigFaces 'leaked' outside the fillet boundaries
  if ( !bBadTrim )
    {
      // accumulate output, rOriginalFacesDelete += unique sDelFaces
      for ( ii=0; ii < sDelFaces.GetSize(); ii++ )
        {
          rOriginalFacesDelete.AddUnique( sDelFaces[ii] );
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::TrimOriginalSurfaces

/*******************************************************************//**
PURPOSE: Split a SmFilletGeom object into two topologically.

NOTES:
  Each fillet is broken up into a sequence of SmBsplineSurface FilletSurfaces.
  Each FilletSurface piece is associated with one SmFilletGeom.
  This function takes one SmFilletGeom with one FilletSurface
    and splits it into two child neighboring SmFilletGeoms each with their
    own child filletSurface, splitting the input parent's rail edges with
    new FilletVertices and adding a new Filletedge between those vertices to
    lie between the two child FilletSurfaces.

METHOD ---
  1. Create output value - a new SmFilletGeom
  2. Split both Rails with new topology objects
      from StartVert->pRailE->EndVert
      to   StartVert->pRailE->pNewVert->pNewRail->EndVert
  3. Connect the new rail-splitting vertices with a new FilletEdge,
     and set them as Mates to each other
    3a. connect this new connecting edge to the old and new FilletGeoms


***********************************************************************/
SmStatus SmFilletGeom::TopologySplit     // eff: split a single parent filletGeom into two connected FilletGeom children
  (SmFilletGeom    *& rpNewFilletGeom,   // out: newly created fillet geom
   SmFilletGeomType   eNewGeomType)      // in : specify type of new returned SmFilletGeom
{
  // Let the input parent SmFilletGeom end up as one of the 2 child SmFilletGeoms

    // 1. initialize output value - a new SmFilletGeom (the 2nd child geom)
    rpNewFilletGeom = new SmFilletGeom(m_pFilletSolver,eNewGeomType);

    // locals
    SmFilletBrep   * pPseudoBrep = m_pFilletSolver->GetFilletExecutive()->GetPseudoBrep();
    SmFilletVertex * pNewVert[2];    // new vertices to split the parent FilletGeom RailEdges
    SmFilletVertex * pStartVert[2];
    SmFilletVertex * pEndVert[2];

    // 2. for both rail Edges - split the railEdge topology
    //      from StartVert->pRailE->EndVert
    //      to   StartVert->pRailE->pNewVert->pNewRail->EndVert
    ULONG lThisRailIdx;
    for ( lThisRailIdx = 0; lThisRailIdx < 2; lThisRailIdx++ )
      {
        SmFilletEdge * pThisRail = GetRail(lThisRailIdx);

        // when a railcurve has been split into pieces due to rollover
        // get the last piece
        if (lThisRailIdx == 0 && m_vOtherRails1.GetSize() > 0)
          {
            pThisRail = m_vOtherRails1.GetLast();
          }
        else if (lThisRailIdx == 1 && m_vOtherRails2.GetSize() > 0)
          {
            pThisRail = m_vOtherRails2.GetLast();
          }

        // get pThisRail's vertices
        pStartVert[ lThisRailIdx ] = (SmFilletVertex*)pThisRail->GetVertex();
        pEndVert  [ lThisRailIdx ] = (SmFilletVertex*)pThisRail->GetOtherVertex(
                                        pStartVert[ lThisRailIdx ]);

        // Get the SmFilletVertexuse and the FilletEdgeuse attached to the end of this rail.
        // It must be on pThisRail: we will be removing it from pThisRail.
        SmFilletVertexuse *pEndVU = pEndVert[ lThisRailIdx ]->GetVUAtRailEnd(this, NULL, pThisRail);
        NER(pEndVU);
        SmFilletEdgeuse   * pEndEU = (SmFilletEdgeuse*)pEndVU->GetEdgeuse();

        // Its orientation will normally be OPPOSITE.
        if ( //pStartVert[lThisRailIdx] == pEndVert[lThisRailIdx] &&
             pEndEU->GetOrientation() != SM_OT_OPPOSITE )
          {
            // Typically, this might happen for closed rails.
            // Get the other vertexuse attached to a rail instead.
            pEndVU = pEndVert[ lThisRailIdx ]->GetVUAtRailEnd( this, pEndVU );
            NER(pEndVU);
            pEndEU = (SmFilletEdgeuse*)pEndVU->GetEdgeuse();
          }  // end orientation check

        // 2b. create a new SmFilletVertex
        pNewVert[ lThisRailIdx ] = new (pPseudoBrep) SmFilletVertex();
        SmFilletEdgeuse * pNewEdgeuse = NULL;

#ifdef SM_DEBUG_CODE
        if ( DebugLevel() > 0 ) {
            if (FALSE) {
                smgfx_Erase();
                smgfx_SetLook(1,2, 0,1,1); this->Draw(); sm_GraphicsLoop();
                sm_GraphicsLoop();
            }
            // Note, drawing 'this' draws things fat.
            smgfx_SetLook( 6, 8, 0,0,0); pThisRail->Draw(); sm_GraphicsLoop();
            smgfx_SetLook( 8,10, 0,0,1); pStartVert[lThisRailIdx]->Draw(); sm_GraphicsLoop();
            smgfx_SetLook( 8,10, 0,1,0); pEndVert  [lThisRailIdx]->Draw(); sm_GraphicsLoop();
            smgfx_SetLook(10,12, 1,0,0); pEndEU->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        // 2c. insert the pNewVert into the pThisRail
        //       making a new FilletEdge, pNewRail
        //       moving the endVert Edgeuse from pThisRail to pNewRail
        //       and creating new Vertexuse/Edgeuse connections to pNewVert
        //           for both pThisRail and pNewRail

        // First, remove the end EU from the rail edge,
        // then call MakeRailEdgeuse() to create a new one
        // starting at the new FilletVertex we just made, pNewVert.
        pThisRail->Remove( pEndEU );

        // hold the old Edgeuse's UVTrimCurve
        SmBSplineCurve *pUVCurve = pEndEU->GetUVTrimCurvePointer();
        pEndEU->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting m_pUVTrimCurve, FALSE = no Leak Warnings

        // connect pNewVert to pThisRail with new topology
        // as pNewVert->Vertexuse->Edgeuse->pRail
        // Since this is the EU starting at the End of the Rail edge,
        // its orientation will be OPPOSITE.

        SER( this->MakeRailEdgeuse( pPseudoBrep,
                                    lThisRailIdx,
                                    pNewVert[lThisRailIdx],
                                    pNewEdgeuse,
                                    SM_OT_OPPOSITE,
                                    pThisRail ));

        // move the old edgeuse's UVTrimcurve onto the new UVTrimcurve
        pNewEdgeuse->SetUVCurve( pUVCurve,
                                 TRUE );   // TRUE = delete preExisting m_pUVTrimCurve

        // Create new edgeuse and new edge in rpNewFilletGeom.
        // Here it makes a new Rail; none passed in.

        SER( rpNewFilletGeom->MakeRailEdgeuse( pPseudoBrep,
             lThisRailIdx, pNewVert[ lThisRailIdx ], pNewEdgeuse, SM_OT_SAME ));

        // Grab the newly created other Rail
        SmFilletEdge * pNewRail = (SmFilletEdge*)pNewEdgeuse->GetEdge();

        // Insert pThisRail's removed edgeuse to new rail
        SER( pNewRail->PostInsert( pEndEU ));
        pEndEU->SetFilletGeom( rpNewFilletGeom );
        ULONG lFoundIndex;
        if ( m_vRefEUs.FindElement( pEndEU, lFoundIndex ))
        {
             m_vRefEUs.RemoveAt( lFoundIndex );
        }

        pNewRail->SetFilletEdgeType( SM_FE_RAIL );
        pNewRail->SetOriginalFace( pThisRail->GetOriginalFace() );
        rpNewFilletGeom->SetRail( lThisRailIdx, pNewRail );
      } // end iter both rail edges

    // 3. connect the rail splitting vertices with a new FilletEdge and as Mates
    pNewVert[0]->SetFilletVertexType(SM_FV_RAIL_END);
    pNewVert[1]->SetFilletVertexType(SM_FV_MATE);
    SmFilletEdge * pDivideEdge = NULL;

    // build a FilletEdge between pNewVert[0] and pNewVert[1]
    //  - each vertex is on a different rail and needs to be
    //    connected to one another with a new edge.
    SER(SmFilletEdge::MakeFilletEdge(pPseudoBrep,   // in : target Brep to receive new topology objects
                                     pNewVert[0],   // in : start of new FilletEdge
                                     pNewVert[1],   // in : end   of new FilletEdge
                                     pDivideEdge)); // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                                    // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
    pDivideEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);

    SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pDivideEdge->GetPrimaryEdgeuse();
    SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
    SmFilletGeom * pOwningFG  = this;
    SmFilletGeom * pFG2 = rpNewFilletGeom;
    if (m_eType == SM_FG_BLENDS || m_eType == SM_FG_TANGENT_ROLLOVER)
      {
        pOwningFG = rpNewFilletGeom;
        pFG2 = this;
      }
    pPrimEU->SetFilletGeom( pOwningFG );

    // Primary edgeuse will be attached to the previous geom
    pOwningFG->m_vEdges.Add(pDivideEdge);

    // Mate edgeuse will always be attached to the new geom
    pFG2->AddSideFilletEdgeuse(pMateEU);

    //WARNING: Do NOT add this line:
    // pMateEU->SetFilletGeom(pFG2);

    // Also, the new FilletVertex's must be owned by a FilletCorner,
    // in order for CalcCornerVertGeom() to be called on them, at least
    // for the 4x2-tangent case.  The Rollover case works fine without
    // them here, and they can't be in both places (double-delete).
    // So put them into the FilletCorner, and not the FilletGeom.

    // pOwningFG->m_vVertices.Add(pNewVert[0]);
    // pOwningFG->m_vVertices.Add(pNewVert[1]);

    // Attach new FV's to FilletCorner at start or end?
    // They go into the corner that's associated with the rollover.
    // If we're moving from a regular (DEFAULT) FG onto a Tangent Rollover FG,
    // then attach them to the corner at the end,
    // else to the corner at the beginning.
    // The type of 'this' (m_eType) is the type at the beginning.
    // Note, if it's a single closed fillet, it's the same Vertex at both ends,
    // and there's only one Corner, so this wouldn't matter.

    ULONG lWhichEnd = 1;
    if ( m_eType == SM_FG_BLENDS || m_eType == SM_FG_TANGENT_ROLLOVER ) {
        lWhichEnd = 0;
    }
    SmVertex * pEndVtx = m_pFilletSolver->GetVertex( lWhichEnd );
    NER(pEndVtx);
    SmFilletExecutive * pFilExec   = m_pFilletSolver->GetFilletExecutive();
    SmFilletCorner * pFilletCorner = pFilExec->GetFilletCornerOfVertex(pEndVtx);

    // Stick them into that corner.
    pFilletCorner->AddFilletVertex( pNewVert[0] );
    pFilletCorner->AddFilletVertex( pNewVert[1] );
    pNewVert[0]->SetFilletCorner( pFilletCorner );
    pNewVert[1]->SetFilletCorner( pFilletCorner );

    // Get the end vertex of edge of original brep, and its FilletCorner,
    // to which the solver is associated.
    // Note, if lWhichEnd is 1, then we already have it.
    if ( lWhichEnd != 1 )
    {
        pEndVtx = m_pFilletSolver->GetVertex(1);
        NER(pEndVtx);
        pFilletCorner = pFilExec->GetFilletCornerOfVertex(pEndVtx);
    }

    SmFilletCornerType eCornerType = pFilletCorner->GetCornerType();

    // We are going to move those side edgeuses associated to
    // original fillet geom to new fillet geom
    for (int j=m_vRefEUs.GetSize()-1; j>=0; j--)
      {
        // Get Edgeuse and Edge pair
        SmFilletEdgeuse * pEU = m_vRefEUs[j];
        NER(pEU->GetFilletGeom());
        SmFilletEdge * pE = (SmFilletEdge*)pEU->GetEdge();

        // no work - no Edge or Edge not connected to filletCorner or
        if (pE == NULL) continue;
        if (pE->GetFilletCorner() != pFilletCorner) continue;
        if ((eCornerType == SM_FCR_N_x_1_CLOSED || eCornerType == SM_FCR_1_x_1)
          && pEU == pE->GetPrimaryEdgeuse())
            { continue; }

        // set this Edgeuse's FilletGeom pointer
        pEU->SetFilletGeom(rpNewFilletGeom);

        // delete the Edgeuse's UVTrimCurve
        pEU->SetUVCurve(NULL, TRUE) ; // TRUE = delete existing UVCurve

        // delete the Edgeuse->Mate's UVTrimCurve
        SmFilletEdgeuse * pMate = (SmFilletEdgeuse*)pEU->GetMate();
        pMate->SetUVTrimCurve(NULL, TRUE) ; // TRUE = delete preExisting m_pUVTrimCurve

        // remove the Edgeuse from the FilletGeom Edgeuse lists
        m_vRefEUs.RemoveAt(j);
        ULONG lIndex;
        if (m_vSideEUs.FindElement(pEU,lIndex)) {
            m_vSideEUs.RemoveAt(lIndex);
        }
      } // end iter orig FilletGeom referenced Edgeuse

    return SM_SUCCESS;

} // SmFilletGeom::TopologySplit

/*******************************************************************//**
PURPOSE: Calculate geometry of a 4-sided coons surface for the
    blending SmFilletGeom (such as when handling self-intersections)

NOTES:
***********************************************************************/
SmStatus SmFilletGeom::CalcBlendingGeom
  ()
{
    SmFilletSolver * pFilSolver = GetFilletSolver();
    const SmContext & crContext = pFilSolver->GetCreationContext();
    SmTArray<SmBSplineCurve*> sBdryCrvs;
    SmTArray<SmBSplineCurve*> sDerivCrvs;
    double dAchieved = 0.0;
    SmTArray<double> sBreaks;
    SmVector3d sTangent1;
    SmVector3d sTangent2;
    SmVector2d sUVTan1;
    SmVector2d sUVTan2;
    SmFilletEdge * pRail1 = m_vRails[0];
    SmFilletEdge * pRail2 = m_vRails[1];
    double dApproxTol3d = pFilSolver->GetThisApproxTol3d();
    double dTangencyTolRadians = pFilSolver->GetTangencyTolerance();

    // Derive Top & Bottom Curves
    // Recreate each rail curve by sampling points evenly spaced in arc length
    SmTArray<SmPoint3d> sUVPoints[2];
    for (ULONG i=0; i<2; i++)
    {
        SmFilletEdge * pRail = m_vRails[i];
        if (pRail->GetFilletEdgeType() != SM_FE_BLENDING_RAIL) {
            SER(SM_ERR);
        }
        SER(pRail->CalcBlendingRail(crContext));
        SmBSplineCurve  * pRailCurve = SM_CAST_PTR(SmBSplineCurve, pRail->GetCurve()); NER(pRailCurve);
        SmFilletEdgeuse * pPrimEU    = (SmFilletEdgeuse*)pRail->GetPrimaryEdgeuse();
        SmFilletEdgeuse * pMateEU    = (SmFilletEdgeuse*)pPrimEU->GetMate();
        SmBSplineCurve  * pUVCurve   = pMateEU->GetUVTrimCurve();
        NER(pUVCurve);
        SmFace * pFace = pRail->GetOriginalFace();
        NER(pFace);
        SmSurface * pBaseSurf = pFace->GetSurface();
        SmExtent1d sIvl = pRailCurve->GetNaturalInterval();
        double dCrvLen;
        double dAccuracy = dApproxTol3d / 10.0;
        SER(pRailCurve->Length( sIvl, dAccuracy, dCrvLen ));

        // Step along this rail curve, at points evenly spaced in arc length.
        ULONG lTotalSamples = 10;
        double dStep = dCrvLen / lTotalSamples;
        double dStartParam = sIvl.GetMin();
        SmPoint3d sUVPnt;
        SER( pUVCurve->EvaluatePoint( dStartParam, sUVPnt ));
        sUVPoints[i].Add( sUVPnt );

        for (ULONG ii=1; ii<lTotalSamples; ii++)
        {
            // Find param on curve at arc length dStep from previous point.
            double dEndParam;
            SER( pRailCurve->FindParameterAtArcLength(dStartParam, dStep, dEndParam ));
            dStartParam = dEndParam;

            // Find uv from the corresponding uv curve.
            SER( pUVCurve->EvaluatePoint( dStartParam, sUVPnt ));
            sUVPoints[i].Add( sUVPnt );

#ifdef SM_DEBUG_CODE
            if ( DebugLevel() > 0 ) {
                // if (0) {
                //     smgfx_SetLook(1,2, 1,0,0); pRailCurve->DrawWDeriv(sIvl); sm_GraphicsLoop();
                //     sm_GraphicsLoop();
                // }
                SmPoint2d sUV(sUVPnt.x,sUVPnt.y);
                SmPoint3d sPnt;
                pBaseSurf->EvaluatePoint(sUV,sPnt);
                smgfx_SetLook(1,4, 0,1,0); sPnt.Draw(); sm_GraphicsLoop();
                sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        }

        // Get the final point.
        SER(pUVCurve->EvaluatePoint( sIvl.GetMax(), sUVPnt ));
        sUVPoints[i].Add( sUVPnt );

        SmTArray<SmVector3d> sVectors;
        pRail->SetCurve(NULL, TRUE) ; // TRUE = delete preExisting Curve
                                      // side effect: delete current pRail->UVTrimCurves
        // SM_ASSERT(pRailCurve != NULL) ; delete pRailCurve ; pRailCurve = NULL ;              // already done
        // pMateEU->SetUVCurve(NULL, TRUE) ; // TRUE = delete preExisting Edgeuse->UVTrimCurve  // already done
        // // SM_ASSERT(pUVCurve != NULL) ; delete pUVCurve ; pUVCurve = NULL ;                 // already done
        pRailCurve = NULL ; // done with pRailCurve - it's been deleted
        pUVCurve   = NULL ; // done with pUVCurve - it's been deleted
        SER( SmBSplineCurve::CreateInterpolatingCurve( crContext,
                                                       SM_CP_CHORDLENGTH,
                                                       2, 2,
                                                       sUVPoints[i], sVectors,
                                                       NULL, TRUE,
                                                       pUVCurve ));

        // Lift it to 3D
        SmExtent2d sUVDomain = pBaseSurf->GetNaturalUVDomain();
        SER(pBaseSurf->LiftCurve(crContext,
                                 sUVDomain,
                                 *pUVCurve,
                                 sIvl,
                                 SM_CAST_APPROXTOL3D(pFace->GetTolerance()),
                                 dAchieved,
                                 pRailCurve));
        pRail->SetCurve(pRailCurve, TRUE) ; // TRUE = delete preExisting Curve - expect pRail->Curve == NULL
                                            // side effect: delete current pRail->UVTrimCurves
        pRailCurve->SetOwner(pRail);
        SmExtent1d sCrvIvl = pRailCurve->GetNaturalInterval();
        pRail->SetInterval(sCrvIvl);
        pMateEU->SetUVCurve(pUVCurve);
    }

    // Create ruled surface if we have liner cross sections
    if (pFilSolver->m_pFSG->GetFilletSurfaceGeneratorType() == SM_FSG_LINEAR) {
        SmBSplineSurface * pRuledSurface = NULL;
        SmBSplineCurve * pRail1Curve = SM_CAST_PTR(SmBSplineCurve,
            pRail1->GetCurve()); NER(pRail1Curve);
        SmBSplineCurve * pRail2Curve = SM_CAST_PTR(SmBSplineCurve,
            pRail2->GetCurve()); NER(pRail2Curve);
        SER(SmBSplineSurface::CreateRuledSurface(crContext,
            *pRail1Curve,*pRail2Curve,SM_SP_V,pRuledSurface));
#ifdef SM_DEBUG_CODE
        if ( DebugLevel() > 0 ) {
            smgfx_SetLook(3,4, 1,0,0); pRail1Curve->Draw(); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); pRail2Curve->Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pRuledSurface->DrawUV(1,1); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        this->m_pFilletSurface = pRuledSurface;
        return SM_SUCCESS;
    }

    // Find 2 side edges (with type SM_FE_CROSS_SECTION)
    SmFilletEdge * pStartEdge = NULL;
    SmFilletEdge * pEndEdge = NULL;
    SmTArray<SmEdge*> sEdges;
    SmVertex * pStartVert = m_vRails[0]->GetVertex();
    pStartVert->GetEdges(sEdges);
    for (ULONG k=0; k<sEdges.GetSize(); k++) {
        SmFilletEdge * pFE = (SmFilletEdge*)sEdges[k];
        if (pFE->GetFilletEdgeType() == SM_FE_CROSS_SECTION) {
            pStartEdge = pFE;
            break;
        }
    }
    SmVertex * pEndVert = m_vRails[0]->GetOtherVertex(pStartVert);
    pEndVert->GetEdges(sEdges);
    for (ULONG kk=0; kk<sEdges.GetSize(); kk++) {
        SmFilletEdge * pFE = (SmFilletEdge*)sEdges[kk];
        if (pFE->GetFilletEdgeType() == SM_FE_CROSS_SECTION) {
            pEndEdge = pFE;
            break;
        }
    }
    NER(pStartEdge); NER(pEndEdge);

    SmTArray<SmBSplineCurve*> sOrderedUV;
    SmTArray<SmCurve*>        sOrdered3D;
    SmTArray<SmOrientType>    sOrients;
    SmTArray<SmSurface*>      sOrderedSurfs;
    SmFilletEdge            * pArr[4];
    SmTArray<SmFilletEdge*>   sFilEdges(4,pArr);
    sFilEdges.Add(pStartEdge);
    sOrients.Add(SM_OT_OPPOSITE);
    sFilEdges.Add(pRail1);
    sOrients.Add(SM_OT_SAME);
    sFilEdges.Add(pEndEdge);
    sOrients.Add(SM_OT_SAME);
    sFilEdges.Add(pRail2);
    sOrients.Add(SM_OT_OPPOSITE);

    for (ULONG j=0; j<4; j++) {
        SmFilletEdge * pFE = sFilEdges[j];
        SmBSplineCurve * pPSCurve = NULL;
        SmSurface * pBaseSurface = NULL;
        SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFE->GetPrimaryEdgeuse();
        SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
        switch (pFE->GetFilletEdgeType()) {
        case SM_FE_BLENDING_RAIL:
        case SM_FE_RAIL_RAIL_INTERPOLATION:
        case SM_FE_SETBACK_RAIL:
            pPSCurve = pMateEU->GetUVTrimCurve();
            pBaseSurface = pFE->GetOriginalFace()->GetSurface();
            break;
        default:
            pPSCurve = pPrimEU->GetUVTrimCurve();
            SmFilletGeom * pFilletGeom = pPrimEU->GetFilletGeom();
            pBaseSurface = (SmSurface*)pFilletGeom->GetFilletSurface();
        }
        sOrderedUV.Add(pPSCurve);
        sOrdered3D.Add(pFE->GetCurve());
        sOrderedSurfs.Add(pBaseSurface);
    }
    SmTArray<SmSurface*> sSurfaces;
    SER(SmBSplineSurface::CreateCornerBlend(crContext,
                                            sOrdered3D,sOrients,sOrderedUV,sOrderedSurfs,
                                            dApproxTol3d,
                                            dTangencyTolRadians,
                                            sSurfaces,
                                            FALSE));

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        sSurfaces[0]->DrawUV(1,1);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    this->m_pFilletSurface = SM_CAST_PTR(SmBSplineSurface,sSurfaces[0]);

    return SM_SUCCESS;

} // end SmFilletGeom::CalcBlendingGeom

/*******************************************************************//**
PURPOSE: Make an SmFilletGeom object into a 'Gap filler'. Typically,
    when the solver failed to trace the whole run of the fillet due to
    boundary-clipping. Another small piece of fillet surface will then be
    generated to fill the GAP. For examples, in the 3x1-Closed Corners
    and 4x2-tangent(G1) corners, when two end points of the fillet edge
    which bounds the fillets did not lie on the same cross-section (i.e.
    'non-perpendicular'), then, as a result, leave a GAP in the corner.
    Therefore, we will need to continue tracing across the bounday to fill
    the GAP. This routine will make necessary settings to fill in the GAP.

NOTES:
    This method should only be called after a new SmFilletGeom was created
    and labeled as type: SM_FG_GAP_FILLER.

***********************************************************************/
SmStatus SmFilletGeom::MakeGapFiller()
{
  // locals
  SmFilletVertex   * pStartVert[2];
  SmFilletVertex   * pEndVert[2];
  SmFilletGeom     * pLastFG            = m_pFilletSolver->GetLastFilletGeom();
  
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface * pLastFilletSurface = pLastFG->GetFilletSurface();
  SM_FILLETSURF_TYPE * pLastFilletSurface = pLastFG->GetFilletSurface();

  for (ULONG i=0; i<2; i++) 
    {
      pStartVert[i] = (SmFilletVertex*)m_vRails[i]->GetVertex();
      pEndVert[i]   = (SmFilletVertex*)m_vRails[i]->GetOtherVertex(pStartVert[i]);
    }

  // Need to find which rail has the 'GAP'.
  // (Note: the other rail should touch the side edge)
  // Compute two corners of the existing fillet surface and
  // do a simple distance comparisons
  double     dTol    = m_pFilletSolver->GetThisApproxTol3d();
  SmExtent2d sDomain = pLastFilletSurface->GetNaturalUVDomain();
  SmPoint2d  sUV     = sDomain.GetMax();
  SmPoint3d  sSurfPnt, sSurfPnt1;
  SmPoint3d  sMatePnt;
  ULONG      lRailIndx = 99; // Index of rail end with NO gap

  SER(pLastFilletSurface->EvaluatePoint(sUV,sSurfPnt));
  sUV.y = sDomain.GetMin().y;
  SER(pLastFilletSurface->EvaluatePoint(sUV,sSurfPnt1));

  //
  for (ULONG jj=0; jj<2; jj++) 
    {
      SmPoint3d sVertPnt = pEndVert[jj]->GetPoint();
      double dDist = sVertPnt.DistanceBetween(sSurfPnt);
      if (dDist < dTol) 
        {
          sMatePnt = sSurfPnt1;
        }
      else 
        {
          double dDist1 = sVertPnt.DistanceBetween(sSurfPnt1);
          if (dDist1 > dTol) continue;
          sMatePnt = sSurfPnt;
        }
      lRailIndx = jj;
    }

  if(lRailIndx == 99) 
    { SER(SM_ERR); } // Unknown error

  // Do some adjustments so that topology will match with geometry
  pStartVert[1-lRailIndx]->SetFilletVertexType(SM_FV_MATE);
  pStartVert[1-lRailIndx]->SetPoint(sMatePnt);

  SmFilletVertexType eFVType = pEndVert[lRailIndx]->GetFilletVertexType();
  SM_ASSERT(eFVType == SM_FV_RAIL_X_EDGEUSE); // Only SM_FV_RAIL_X_EDGEUSE is handled here

  pStartVert[lRailIndx]->SetFilletVertexType(eFVType);
  pStartVert[lRailIndx]->SetPoint(pEndVert[lRailIndx]->GetPoint());

  pEndVert[lRailIndx]->SetFilletVertexType(SM_FV_MATE);
  pEndVert[lRailIndx]->SetMate(0,pEndVert[1-lRailIndx]);
  pEndVert[1-lRailIndx]->SetMate(0,pEndVert[lRailIndx]);

  // Need to calculate the new position of pEndVert[lRailIndx]
  // Since it is now the 'mate' of pEndVert[1-lRailIndx],
  // We can find its UV corrd. on the other side of the fillet geom
  SmTArray<SmVertexuse*> sVUs;
  SmFilletVertexuse    * pFoundVU = NULL; // Corresponding to the neighboring fillet geom

  pEndVert[1-lRailIndx]->GetVertexuses(sVUs);

  for (ULONG kk=0; kk<sVUs.GetSize() && !pFoundVU; kk++) 
    {
      SmFilletEdge * pE = (SmFilletEdge*)sVUs[kk]->GetEdgeuse()->GetEdge();
      if(   pE->GetFilletEdgeType() == SM_FE_RAIL
         && pE != m_vRails[1-lRailIndx]) 
        {
          pFoundVU = (SmFilletVertexuse*)sVUs[kk];
        }
    }
  NER(pFoundVU);

  SmTsectPnt & rTsectPnt = pFoundVU->GetTsectPnt();
  SmPoint2d    sUV2      = rTsectPnt.UVPos(lRailIndx);
  pEndVert[lRailIndx]->GetVertexuses(sVUs);
  SmFace * pFoundFace2 = NULL;

  //
  for (ULONG mm=0; mm<sVUs.GetSize() && !pFoundFace2; mm++) 
    {
      SmFilletEdge * pE = (SmFilletEdge*)sVUs[mm]->GetEdgeuse()->GetEdge();
      if(   pE->GetFilletEdgeType() == SM_FE_RAIL
         && pE != m_vRails[lRailIndx]) 
        {
          pFoundFace2 = pE->GetOriginalFace();
        }
    }
  NER(pFoundFace2);

  SmPoint3d sPnt2;
  SER(pFoundFace2->GetSurface()->EvaluatePoint(sUV2,sPnt2));
  pEndVert[lRailIndx]->SetPoint(sPnt2);
  //pEndVert[lRailIndx]->SetOriginalUV(sUV2);
  //pEndVert[lRailIndx]->SetPointClass(SM_PC_FACE, pFoundFace2)

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetLook(1,6, 1,0,0); pStartVert[  lRailIndx]->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,6, 1,0,0); pStartVert[1-lRailIndx]->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,6, 0,0,1); pEndVert[  lRailIndx]->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,6, 0,0,1); pEndVert[1-lRailIndx]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmFilletGeom::MakeGapFiller

/*******************************************************************//**
PURPOSE: Tell the FilletExecutive about a fillet error.

NOTES:
    This could arise from failure to imprint an edge onto a face.
***********************************************************************/
void SmFilletGeom::RecordFilletError
 (SmFilletErrorType  eType,
  SmEdge           * pFilletedEdge,
  SmFace           * pTargetFace,
  SmFilletEdge     * pRail,
  SmSurface        * pFilletSurface,
  const TCHAR      * cComment)
{
  // pass the call along to the FilletExecutive
  this->GetFilletSolver()->GetFilletExecutive()->NoteFilletError(eType,
                                                                 GetStatus(),
                                                                 pFilletedEdge,
                                                                 pTargetFace,
                                                                 pRail,
                                                                 pFilletSurface,
                                                                 (TCHAR *)cComment) ;
} // end SmFilletGeom::RecordFilletError

/*******************************************************************//**
PURPOSE: Debug dump for SM_COMMON_BASE declaration
***********************************************************************/
void SmFilletGeom::Dump() const
{
  // pass the call along
  DumpLevel(0, NULL) ;

} // end SmFilletGeom::Dump

/*******************************************************************//**
PURPOSE: Debug dump of FilletGeom = Geometry for 1 FilletSurface
         containing: 1 BackPtr to FilletSolver,
                     2 rail curves, (which might be split for rollovers)
                     1 center line, 
                     1 Fillet Surface,
                     2 Offset Surfaces (of the original Surfaces attached to Target Edge)
                     2 Offset Orientations directions
                     2 Deviations of Rail from Surface
                     2 Capped Flags
                     1 NeedsSplit Flag
                     Array of Edges other than rails owned by this FilletSurface
                     Array of Vertices owned by this FilletSurface
                     Array of all referenced Edgeuses
                     Array of all sideCurve edgeuses (excluding rails)

NOTES:  iDebugLevel >=  0 Pretty Print SmFilletGeom Summary
        iDebugLevel >= 20 Dump Nested Objects abbreviated
        iDebugLevel >= 60 Dump Nested Objects in Full
***********************************************************************/
void SmFilletGeom::DumpLevel( int iDebugLevel, const TCHAR * cpMsg ) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  ULONG i;

  // message
  if(cpMsg != NULL) smos_WriteBuffer( cpMsg );

  // label
  smos_WriteBuffer( _T( "\nBegin SmFilletGeom::DumpLevel()" ) );

  // header
  smos_sprintf( sBuff, _T( "\nSmFilletGeom object:[0x%p] : " ), this );
  smos_sprintf( sBuffForFile, _T( "\nSmFilletGeom object:[%s] : " ), _T( "NotNULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );


  // FilletSolver Type, Status, Index in Solver, Solver Index in Exec
  ULONG lFGIdx = this->FindIndexInSolver();
  ULONG lFSIdx = this->GetFilletSolver()->FindIndexInExec();
  smos_sprintf( sBuff, _T( "\n  IndexInSolver      : [%4ld]  Type:[%s]  Status:[%s]  Solver->IndexInExec:[%4ld]" ),
                     lFGIdx,
                       m_eType == SM_FG_BLENDS ? _T( "SM_FG_BLENDS" )
                     : m_eType == SM_FG_CLIFF_ROLLOVER ? _T( "SM_FG_CLIFF_ROLLOVER" )
                     : m_eType == SM_FG_TANGENT_ROLLOVER ? _T( "SM_FG_TANGENT_ROLLOVER" )
                     : m_eType == SM_FG_CLIFF_SIDE_PATCH ? _T( "SM_FG_CLIFF_SIDE_PATCH" )
                     : m_eType == SM_FG_GAP_FILLER ? _T( "SM_FG_GAP_FILLER" )
                     : m_eType == SM_FG_CORNER_FILLET ? _T( "SM_FG_CORNER_FILLET" )
                     : _T( "SM_FG_DEFAULT" ),
                       m_eStatus == SM_FIL_PROCESSED ? _T( "SM_FIL_PROCESSED" )
                     : m_eStatus == SM_FIL_FAILURE ? _T( "SM_FIL_FAILURE" )
                     : m_eStatus == SM_FIL_SURF_INT_FAILURE ? _T( "SM_FIL_SURF_INT_FAILURE" )
                     : m_eStatus == SM_FE_END_PNT_DROP_FAILURE ? _T( "SM_FE_END_PNT_DROP_FAILURE" )
                     : m_eStatus == SM_FE_TO_BE_SQUEEZED ? _T( "SM_FE_TO_BE_SQUEEZED" )
                     : m_eStatus == SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE ? _T( "SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE" )
                     : m_eStatus == SM_FV_NO_INT_RAIL_EU ? _T( "SM_FV_NO_INT_RAIL_EU" )
                     : m_eStatus == SM_FV_SOLVER_NOT_CONVERGE ? _T( "SM_FV_SOLVER_NOT_CONVERGE" )
                     : m_eStatus == SM_FV_TO_BE_DELETED ? _T( "SM_FV_TO_BE_DELETED" )
                     : m_eStatus == SM_FS_SURF_TRACING_FAILURE ? _T( "SM_FS_SURF_TRACING_FAILURE" )
                     : m_eStatus == SM_FS_SELF_INT_HANDLING_FAILURE ? _T( "SM_FS_SELF_INT_HANDLING_FAILURE" )
                     : m_eStatus == SM_FS_SURF_SKINNING_FAILURE ? _T( "SM_FS_SURF_SKINNING_FAILURE" )
                     : _T( "SM_FIL_UNPROCESSED" ),
                     lFSIdx );
  smos_WriteBuffer( sBuff );

  // Capped Ends
  smos_sprintf( sBuff, _T( "\n  Capped Start       : [%s]\n  Capped End         : [%s]" ),
                     m_bCapped[0] ? _T( "TRUE" ) : _T( "FALSE" ),
                     m_bCapped[1] ? _T( "TRUE" ) : _T( "FALSE" ) );
  smos_WriteBuffer( sBuff );

  // two main rails and rail->CurveTypes
  smos_sprintf( sBuff, _T( "\n  Two Main Rails     : [0x%p:Curve[%s] 0x%p:Curve[%s]]" ), m_vRails[0],
    (m_vRails[0] && m_vRails[0]->GetCurve()) ? m_vRails[0]->GetCurve()->GetClassString() : _T( "None" ),
                                                                                            m_vRails[1],
                                                                                        (m_vRails[1] && m_vRails[1]->GetCurve()) ? m_vRails[1]->GetCurve()->GetClassString() : _T( "None" ) );
  smos_sprintf( sBuffForFile, _T( "\n  Two Main Rails     : [%s:Curve[%s] %s:Curve[%s]]" ), (m_vRails[0]) ? _T( "notNULL" ) : _T( "NULL" ),
    (m_vRails[0] && m_vRails[0]->GetCurve()) ? m_vRails[0]->GetCurve()->GetClassString() : _T( "None" ),
(m_vRails[1]) ? _T( "notNULL" ) : _T( "NULL" ),
(m_vRails[1] && m_vRails[1]->GetCurve()) ? m_vRails[1]->GetCurve()->GetClassString() : _T( "None" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // CenterSpine Curve and type
  smos_sprintf( sBuff, _T( "\n  Center Spine Curve : [0x%p:Curve[%s]]" ), m_pCenterLineCurve,
                                                                           m_pCenterLineCurve ? m_pCenterLineCurve->GetClassString() : _T( "None" ) );
  smos_sprintf( sBuffForFile, _T( "\n  Center Spine Curve : [%s:Curve[%s]]" ), (m_pCenterLineCurve) ? _T( "notNULL" ) : _T( "NULL" ),
                                                                           m_pCenterLineCurve ? m_pCenterLineCurve->GetClassString() : _T( "None" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // number of OtherRails split for Rollover
  smos_sprintf( sBuff, _T( "\n  Other Rails1       : [%ld]\n  Other Rails2       : [%ld]\n  Owned Vertices     : [%ld]\n  Owned Edges        :[%ld]" ),
                     m_vOtherRails1.GetSize(),
                     m_vOtherRails2.GetSize(),
                     m_vVertices.GetSize(),
                     m_vEdges.GetSize() );
  smos_WriteBuffer( sBuff );

  // referenced Edgeuses
  smos_sprintf( sBuff, _T( "\n  Referenced Edgeuses: [%ld]\n  Other Side Edgeuses: [%ld]" ),
                     m_vRefEUs.GetSize(),
                     m_vSideEUs.GetSize() );
  smos_WriteBuffer( sBuff );

  // FilletSurface and Type
  smos_sprintf( sBuff, _T( "\n  Fillet Surface     : [0x%p:[%s]]\n" ), m_pFilletSurface,
                                                                          m_pFilletSurface ? m_pFilletSurface->GetClassString() : _T( "None" ) );
  smos_sprintf( sBuffForFile, _T( "\n  Fillet Surface     : [%s:[%s]]\n" ), (m_pFilletSurface) ? _T( "notNULL" ) : _T( "NULL" ),
                                                                          m_pFilletSurface ? m_pFilletSurface->GetClassString() : _T( "None" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // Done with summary - optionally output contained object dumps
  if(iDebugLevel > 20)
  {
    SmBoolean bAbbrev = (iDebugLevel < 60);

    // Rail 0 SmFilletEdge Dump
    if(m_vRails[0])
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump m_vRails[0]:[0x%p]" ), m_vRails[0] );
      smos_WriteBuffer( sBuff, sBuffForFile );
      m_vRails[0]->DumpLevel( iDebugLevel, cpMsg );
    }

    // Rail 1 SmFilletEdge Dump
    if(m_vRails[1])
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump m_vRails[1]:[0x%p]" ), m_vRails[1] );
      smos_WriteBuffer( sBuff, sBuffForFile );
      m_vRails[1]->DumpLevel( iDebugLevel, cpMsg );
    }

    // CenterSpine dump
    if(iDebugLevel > 20 && m_pCenterLineCurve != NULL)
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump Center Spine Curve:[0x%p]" ), m_pCenterLineCurve );
      smos_sprintf( sBuffForFile, _T( "\n  Begin Dump Center Spine Curve:[%s]" ), (m_pCenterLineCurve) ? _T( "notNULL" ) : _T( "NULL" ) );
      smos_WriteBuffer( sBuff, sBuffForFile );

      m_pCenterLineCurve->Dump( bAbbrev );
    }

    // OtherRails 1 Dumps
    if(m_vOtherRails1.GetSize() > 0)
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump %ld Other Rails1:" ), m_vOtherRails1.GetSize() );
      smos_WriteBuffer( sBuff );
      for(i = 0; i < m_vOtherRails1.GetSize(); i++)
      {
        m_vOtherRails1[i]->DumpLevel( iDebugLevel );
      }
    }

    // OtherRails2 Dumps
    if(m_vOtherRails2.GetSize() > 0)
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump %ld Other Rails2:" ), m_vOtherRails2.GetSize() );
      smos_WriteBuffer( sBuff );
      for(i = 0; i < m_vOtherRails2.GetSize(); i++)
      {
        m_vOtherRails2[i]->DumpLevel( iDebugLevel );
      }
    }

    // Vertices Dump
    if(m_vVertices.GetSize() > 0)
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump %ld Owned Vertices:" ), m_vVertices.GetSize() );
      smos_WriteBuffer( sBuff );
      for(i = 0; i < m_vVertices.GetSize(); i++)
      {
        m_vVertices[i]->DumpLevel( iDebugLevel );
      }
    }

    // FilletSurface Dump
    if(m_pFilletSurface != NULL)
    {
      smos_sprintf( sBuff, _T( "\n  Begin Dump Fillet Surface:[0x%p]" ), m_pFilletSurface );
      smos_sprintf( sBuffForFile, _T( "\n  Begin Dump Fillet Surface:[%s]" ), (m_pFilletSurface) ? _T( "notNULL" ) : _T( "NULL" ) );
      smos_WriteBuffer( sBuff, sBuffForFile );

      m_pFilletSurface->Dump( bAbbrev );
    }
  }


  // all done
  smos_WriteBuffer( _T( "End SmFilletGeom::DumpLevel()\n" ) );

} // end SmFilletGeom::DumpLevel()

#ifdef SM_DEBUG_CODE
/*******************************************************************//**
PURPOSE: Debugging convenience: Dump the FilletVertexuses of this FilletGeom.

NOTES: 
***********************************************************************/
void SmFilletGeom::DumpFilletVUs()
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("\nDump of FilletVertexuses in FilletGeom  0x%p :\n"), this );
  smos_WriteBuffer(sBuff);

  for ( ULONG ii=0; ii<2; ii++ )
  {
      for ( ULONG jj=0; jj<2; jj++ )
      {
          SmFilletVertexuse *pFVU = this->GetRailVertexuse( ii, jj );
          SmVertex          *pVtx = pFVU->GetVertex();
          SmFilletVertex    *pFV  = SM_CAST_PTR( SmFilletVertex, pVtx );

          SmFilletVertexType eFVType =  pFV->GetFilletVertexType();
          smos_sprintf( sBuff, _T("  Dump of pFVU [ %lu, %lu ],  0x%p , FilVtx 0x%p, Type %d\n" ), ii, jj, pFVU, pFV, eFVType );
          smos_WriteBuffer(sBuff);

          SmTsectPnt & rTsectPnt = pFVU->GetTsectPnt();
          SmPoint2d sUV1 = rTsectPnt.UVPos(0);
          SmPoint2d sUV2 = rTsectPnt.UVPos(1);
          smos_sprintf( sBuff, _T("    PointType %d , uv1 %lf %lf  UV2 %lf %lf\n" ), rTsectPnt.PointType(), sUV1.x, sUV1.y, sUV2.x, sUV2.y );
          smos_WriteBuffer(sBuff);
      }
  }

  return;

} // end DumpFilletVUs

#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Debug draw.
***********************************************************************/
void SmFilletGeom::Draw() const
{
#ifdef SM_DEBUG_CODE
  ULONG i;

  smgfx_SetLook(1,1, 1,1,0) ; if(m_pFilletSurface)   { m_pFilletSurface->DrawUV(3,3) ; }
  smgfx_SetLook(2,1, 1,0,0) ; if(m_pCenterLineCurve) { m_pCenterLineCurve->Draw() ; }
  smgfx_SetLook(5,1, 0,0,1) ; if(m_vRails[0])        { m_vRails[0]->Draw() ; }
  smgfx_SetLook(5,1, 0,0,1) ; if(m_vRails[1])        { m_vRails[1]->Draw() ; }

  smgfx_SetLook(5,1, 1,0,1) ; for(i=0;i<m_vOtherRails1.GetSize();i++) { m_vOtherRails1[i]->Draw() ; }
  smgfx_SetLook(5,1, 1,0,1) ; for(i=0;i<m_vOtherRails2.GetSize();i++) { m_vOtherRails2[i]->Draw() ; }

#endif // SM_DEBUG_CODE

} // end SmFilletGeom::Draw()

/*******************************************************************//**
PURPOSE: Return the index of this FilletGeom in our FilletSolver.

NOTES: Used in debugging.
    Returns error 9999 if 'this' is not in our FilletSolver.
***********************************************************************/
ULONG SmFilletGeom::FindIndexInSolver() const
{
  return m_pFilletSolver->FindFilGeomIndex( this );

} // end SmFilletGeom::FindIndexInSolver

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertFilletEdge_list[] =
{
  {SM_AT_UNKNOWN, _T("Unknown"), _T("Not yet implemented") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmFilletEdge::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmEdge::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // todo - add SmFilletEdge checks here

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmFilletEdge::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmFilletEdge::AssertHeal
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
//       return ( SmEdge::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmFilletEdge::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmFilletEdge::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Draw a rail

NOTES:
***********************************************************************/
SmDisplayList * SmFilletEdge::Draw
 (SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  // const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // output Edge Graphics
  if ( GetCurve() != NULL )
    { GetCurve()->DrawWDeriv(GetCurve()->GetNaturalInterval(),0,NULL,pOptGfxSet); }

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmFilletEdge::Draw

/*******************************************************************//**
PURPOSE: Debug dump for a FilletEdge

NOTES:
***********************************************************************/
void SmFilletEdge::DumpLevel( int iDebugLevel, const TCHAR * cpMsg )
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  if(cpMsg != NULL) smos_WriteBuffer( cpMsg );

  // label
  smos_WriteBuffer( _T( "\nBegin SmFilletEdge::DumpLevel() derived from class SmEdge" ) );

  // header
  smos_sprintf( sBuff, _T( "\nSmFilletEdge object[0x%p] : " ), this );
  smos_sprintf( sBuffForFile, _T( "\nSmFilletEdge object[%s] : " ), _T( "NotNULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );


  ULONG lFEIdx = FindIndexInCorner();
  ULONG lEdgeIdx = (m_pOrigEdge) ? m_pOrigEdge->GetEdgeNumberInBrep() : 9999;
  ULONG lFaceIdx = (m_pOrigFace) ? m_pOrigFace->GetFaceNumberInBrep() : 9999;

  smos_sprintf( sBuff, _T( "\n  IndexInCorner:[%4ld]  Type:[%d]  Status:[%d]  Deviation:[%lf]" ),
              lFEIdx, m_eType, m_eStatus, m_dDeviation );
  smos_WriteBuffer( sBuff );

  smos_sprintf( sBuff, _T( "\n  OrigEdgeNumInBrep:[%4ld], OrigFaceNumInBrep:[%4ld]" ), lEdgeIdx, lFaceIdx );
  smos_WriteBuffer( sBuff );

  smos_sprintf( sBuff,_T("%s"), _T( "\n  Domain Interval:  " ) );
  smos_WriteBuffer( sBuff );
  m_vInterval.Dump();

  smos_sprintf( sBuff, _T( "\n  OrigEdge:[0x%p]" ), m_pOrigEdge );
  smos_sprintf( sBuffForFile, _T( "\n  OrigEdge:[%s]" ), m_pOrigEdge ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  smos_sprintf( sBuff, _T( ", OrigFace:[0x%p] : " ), m_pOrigFace );
  smos_sprintf( sBuffForFile, _T( ", OrigFace:[%s] : " ), m_pOrigFace ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  if ( m_cpCorner ) {
      smos_sprintf( sBuff, _T( "\n  FilletCornerIndexInExec:[%4ld]" ), m_cpCorner->FindIndexInExec() );
  } else {
      smos_sprintf( sBuff,_T("%s"), _T( "\n  FilletCorner : NULL" ) );
  }
  smos_WriteBuffer( sBuff );

  smos_sprintf( sBuff, _T( ", CurveClassification:[0x%p] : " ), m_pCurveClass );
  smos_sprintf( sBuffForFile, _T( ", CurveClassification:[%s] : " ), m_pCurveClass ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // contained object dumps
  SmBoolean bAbbrev = (iDebugLevel < 60);
  if(iDebugLevel > 20)
  {
    if(m_pOrigEdge)
    {
      smos_sprintf( sBuff, _T( "\n  OrigEdge:[0x%p] : " ), m_pOrigEdge );
      smos_sprintf( sBuffForFile, _T( "\n  OrigEdge:[%s] : " ), m_pOrigEdge ? _T( "NotNULL" ) : _T( "NULL" ) );
      smos_WriteBuffer( sBuff, sBuffForFile );

      m_pOrigEdge->Dump( bAbbrev );
    }

    if(m_pOrigFace)
    {
      smos_sprintf( sBuff, _T( "\n  OrigFace:[0x%p] : " ), m_pOrigFace );
      smos_sprintf( sBuffForFile, _T( "\n  OrigFace:[%s] : " ), m_pOrigFace ? _T( "NotNULL" ) : _T( "NULL" ) );
      smos_WriteBuffer( sBuff, sBuffForFile );

      m_pOrigFace->Dump( bAbbrev );
    }

    if(m_pCurveClass)
    {
      smos_sprintf( sBuff, _T( "\n  CurveClassification:[0x%p] : " ), m_pCurveClass );
      smos_sprintf( sBuffForFile, _T( "\n  CurveClassification:[%s] : " ), m_pCurveClass ? _T( "NotNULL" ) : _T( "NULL" ) );
      smos_WriteBuffer( sBuff, sBuffForFile );

      m_pCurveClass->Dump();
    }


    // Dump the Root class
    smos_WriteBuffer( _T( "\n" ) );
    SmEdge::Dump( bAbbrev );

  } // end this existence check

  smos_WriteBuffer( _T( "End SmFilletEdge::DumpLevel() derived from SmEdge\n" ) );

} // end SmFilletEdge::DumpLevel

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletEdge::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletEdge_TYPE == t) ? TRUE : SmEdge::IsKindOf( (t) ));
}

/*************************************************************
PURPOSE: Unimplemented Dump method to satisfy SM_COMMON.

NOTES: May be implemented if desired.
**************************************************************/
void SmFilletEdge::Dump (void) const
{ }

/*******************************************************************//**
PURPOSE: Return the index of this FilletEdge in our FilletCorner.

NOTES: Used in debugging.
    Returns error 9999 if this is not in our FilletCorner.
***********************************************************************/
ULONG SmFilletEdge::FindIndexInCorner() const
{
  if ( m_cpCorner == NULL ) { return 9999; }
  return m_cpCorner->FindEdgeIndex( this );

} // end SmFilletEdge::FindIndexInCorner

/*******************************************************************//**
PURPOSE: Constructor for SmFilletBrep

NOTES:
***********************************************************************/
SmFilletBrep::SmFilletBrep( )
 : SmBrep()
{
    m_bEditingEnabled = TRUE;

} // end SmFilletBrep::SmFilletBrep constructor

/*******************************************************************//**
PURPOSE: Destructor for SmFilletBrep

NOTES:
***********************************************************************/
SmFilletBrep::~SmFilletBrep()
{

} // end SmFilletBrep::~SmFilletBrep destructor

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertFilletBrep_list[] =
{
  {SM_AT_UNKNOWN, _T("UNKNOWN"), _T("Not yet implemented") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmFilletBrep::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmBrep::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // todo - add SmFilletBrep checks here

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmFilletBrep::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmFilletBrep::AssertHeal
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
//       return ( SmBrep::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmFilletBrep::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmFilletBrep::AssertHeal
// end obsolete

#ifdef SM_DEBUG_CODE
/*******************************************************************//**
PURPOSE: DebugLevel() implementations
***********************************************************************/
int SmFilletVertex::DebugLevel()
  { return (m_cpCorner == NULL) ? 0 : m_cpCorner->DebugLevel(); }

int SmFilletEdgeuse::DebugLevel()
  { return (m_pFilletGeom == NULL) ? 0 : m_pFilletGeom->DebugLevel(); }

int SmFilletEdge::DebugLevel()
  { return (m_cpCorner == NULL) ? 0 : m_cpCorner->DebugLevel(); }

int SmFilletGeom::DebugLevel()
  { return (m_pFilletSolver == NULL) ? 0 : m_pFilletSolver->DebugLevel(); }

#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Return the SmObject this fillet vertex's point classification maps to.

USAGE NOTES---- Not inline: <windows.h> defines GetObject as GetObjectW, which
breaks a header call to it.
***********************************************************************/
SmObject * SmFilletVertex::GetPointClassObject() const
{ return m_vPointClass.GetObject(); }
