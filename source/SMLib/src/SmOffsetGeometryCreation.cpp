// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmOffsetGeometryCreation.cpp 
* PURPOSE   --- Source file for class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmOffsetGeometryCreation.h>
#include <SmOffsetExecutive.h>
#include <SmFilletSolver.h>
#include <SmGeomUtility.h>
#include <SmPlane.h>
// Remove Composites
// #include <SmCEdge.h> 
#include <SmAssertArray.h>
#include <SmCrvOnSurf.h>

/*******************************************************************//**
PURPOSE: Destructor for SmOffsetGeometryCreation.

NOTES: 
***********************************************************************/
SmOffsetGeometryCreation::~SmOffsetGeometryCreation() 
{ }

/*******************************************************************//**
PURPOSE: Create the offset surface corresponding to a faceuse.

NOTES: Approximate When necessary - Out Surf->Domain(s) may be trimmed 
       but not scaled.
***********************************************************************/
SmStatus SmLocalOperationGeometryCreation::FaceuseOffset
 (const SmContext       & crContext,             // in : context for new obj construction  
  const SmFaceuse       * pOriginalFaceuse,      // in : Faceuse->Surface to copy and offset  
  SmBoolean               bSkipSelfIntersection, // NotUsed: in : TRUE =  
                                                 //      FALSE=  
  SmSurface*            & rNewSurface,           // out: New Surface 
  SmSSIData             & rSelfIntersections)    // out: List of SelfXSect curves (when bSkipSelfIntersection == FALSE)
 const
{
  SM_REF1(bSkipSelfIntersection) ;
  // init output
  rSelfIntersections.m_v3DCurves.ReSet();
  rSelfIntersections.m_vUVCurves1.ReSet();
  rSelfIntersections.m_vUVCurves2.ReSet();
  rSelfIntersections.m_vCurveTypes.ReSet();
  rSelfIntersections.m_vDeviations.ReSet();

  // locals
  SmFace    *pFace = pOriginalFaceuse->GetFace();
  SmSurface *pSur  = pFace->GetSurface();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      SmBrep *pBrep = pFace->GetBrep() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop(); 
      smgfx_SetLook(1,2, 0,1,1) ; pSur->DrawUV(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; pFace->Draw(SM_DM_CROSSHATCH) ; 
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  ULONG lIndex = 0;
  if (!m_vFaces.FindElement(pFace,lIndex)) 
    {
      SER(SM_ERR);
    }

  rNewSurface = m_vSurfaces[lIndex];

  if (m_bMakeAnalytics)
  {
      SmSurface* pS = rNewSurface;
      SmExtent2d sOldDomain = pSur->GetNaturalUVDomain();
      pS->Reparameterize(sOldDomain);
      SmObjDelete sClean(pS);
      SmSurface* pAnalyticS = NULL;

      // Copy pS, when possible as an analytic surface
      SER(pS->CopyAndAddAnalytics(crContext, pAnalyticS));
      SmOffsetSurface* pOffset = SM_CAST_PTR(SmOffsetSurface, pAnalyticS);
      if (pOffset)
      {
          pOffset->SetAllowExtension(FALSE);
      }
      rNewSurface = pAnalyticS;
  }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      pSur->Dump();
      rNewSurface->Dump();
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmLocalOperationGeometryCreation::FaceuseOffset

/*******************************************************************//**
PURPOSE: Create the offset surface from a given faceuse->Face->Surface.
            Optionally check for and report self-intersections.

NOTES: Approximate When necessary - Out Surf->Domain(s) may be trimmed 
       but not scaled
***********************************************************************/
SmStatus SmOffsetGeometryCreation::FaceuseOffset
(
  const SmContext       & crContext,             // in : context for new obj construction  
  const SmFaceuse       * pOriginalFaceuse,      // in : Faceuse->Surface to copy and offset  
  SmBoolean               bSkipSelfIntersection, // in : TRUE =  
                                                 //      FALSE=  
  SmSurface *           & rNewSurface,           // out: New Surface  
  SmSSIData             & rSelfIntersections     // out: List of SelfXSect curves (when bSkipSelfIntersection == FALSE)
) const
{
  // init output
  rSelfIntersections.m_v3DCurves.ReSet();
  rSelfIntersections.m_vUVCurves1.ReSet();
  rSelfIntersections.m_vUVCurves2.ReSet();
  rSelfIntersections.m_vCurveTypes.ReSet();
  rSelfIntersections.m_vDeviations.ReSet();
  rNewSurface = NULL;

  // locals
  double     dSign =   (pOriginalFaceuse->GetOrientation() == SM_OT_OPPOSITE)
                     ? -1.0
                     :  1.0 ;
  SmFace    *pFace = pOriginalFaceuse->GetFace();
  SmSurface *pSur  = pFace->GetSurface();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount  = 0;
  lDebugCount ++;
  if (bDebugMe || lDebugCount == lCount) 
    {
      SmBrep *pBrep = pFace->GetBrep() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; pSur->DrawUV(9,9); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // construct an offset surface from this surface - 
  SER_DELETE(pSur->CreateOffsetSurface(crContext,               // in : context for new obj construction 
                                dSign*m_dOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal) 
                                m_dThisApproxTol3d/10.0, // in : Max Dist between ApproxOffsetSurface and ideal offset shape 
                                rNewSurface), // out: Offset Surf Approx, may be more than 1 when offsets have self-intersections
                                rNewSurface); // JLMCC hunting memory leaks

  if ( rNewSurface == NULL )
    { return SM_SUCCESS; }

  // when working with analytics - 
  // replace all current appropriate BSplineSurface
  //  with analytic copies when the surface is properly represented
  if(m_bMakeAnalytics)
  {
    SmSurface *pS = rNewSurface;

    // skip surfaces which are already analytics
    if(!(pS->IsAnalytic()))
    {
        SmSurface* pAnalyticS = NULL; // JLMCC editted to address memory leak
        //{
            // make this surface temporary
            SmObjDelete sClean(pS);

            // see if this surface is secretly an analytic
            //   if it is - copy it as an analytic surface
            //   if it isn't -
            // Copy pS, when possible as an analytic surface
            SER(pS->CopyAndAddAnalytics(crContext, pAnalyticS));
            SmOffsetSurface* pOffset = SM_CAST_PTR(SmOffsetSurface, pAnalyticS);

            if (pOffset)
            {
                pOffset->SetAllowExtension(FALSE);
            }
        //}
      // store the copy
      rNewSurface = pAnalyticS;
    }

  }  // end m_bMakeAnalytics check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      pSur->Dump();
      if (rNewSurface) 
        {
          rNewSurface->Dump();
          rNewSurface->DrawUV(2,2);
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // when only one offset surface was made and checking for self-intersections 
  if (   rNewSurface && !bSkipSelfIntersection) 
    {
      // self-intersection locals
      double dAngleTol       = 20.0 * SM_PI / 180.0;
      SmBoolean bUseSurfaceEdges[2];
      bUseSurfaceEdges[0]    = FALSE ;
      bUseSurfaceEdges[1]    = FALSE;
      SmSurface *pOffSurface = rNewSurface;
      SmExtent2d sUVDomain   = pOffSurface->GetNaturalUVDomain();

      // find self intersections
#ifndef SM_VALIDATE_INTERSECTORS
      SER(pOffSurface->GlobalSurfaceIntersect
                         (crContext,
                          sUVDomain,
                          *pOffSurface,
                          sUVDomain,
                          bUseSurfaceEdges,
                          SM_CAST_APPROXTOL3D_PTR(&m_dThisApproxTol3d),
                          &dAngleTol,
                          &rSelfIntersections.m_v3DCurves,
                          SM_REINTERPRET_CAST(SmTArray<SmCurve*> *,&rSelfIntersections.m_vUVCurves1),
                          SM_REINTERPRET_CAST(SmTArray<SmCurve*> *,&rSelfIntersections.m_vUVCurves2),
                          &rSelfIntersections.m_vCurveTypes,
                          &rSelfIntersections.m_vDeviations));
#else // no SM_VALIDATE_INTERSECTORS
      SER(pOffSurface->GlobalSurfaceIntersectAndValidate
                         (crContext,
                          sUVDomain,
                          *pOffSurface,
                          sUVDomain,
                          bUseSurfaceEdges,
                          SM_CAST_APPROXTOL3D_PTR(&m_dThisApproxTol3d),
                          &dAngleTol,
                          &rSelfIntersections.m_v3DCurves,
                          SM_REINTERPRET_CAST(SmTArray<SmCurve*> *,
                          &rSelfIntersections.m_vUVCurves1),
                          SM_REINTERPRET_CAST(SmTArray<SmCurve*> *,
                          &rSelfIntersections.m_vUVCurves2),
                          &rSelfIntersections.m_vCurveTypes,
                          &rSelfIntersections.m_vDeviations));
#endif // no SM_VALIDATE_INTERSECTORS   
    } // end self-intersection check

  // all done
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::FaceuseOffset

/*******************************************************************//**
PURPOSE: Create the offset geometry of a vertex given the bounding edges
    in the offset.

NOTES: 
***********************************************************************/
SmStatus SmOffsetGeometryCreation::VertexOffset
  (const SmContext          & crContext,          // in : context for new object construction 
   const SmVertex           * pOriginalVertex,    // in : Vertex on orig Brep being offset
   const SmTArray<SmEdge*>  & rBoundingEdges,     // in : 
   SmTArray<SmCurve*>       & r3DTrimmingCurves,  // out: 
   SmTArray<SmOrientType>   & rOrients,           // out: 
   SmSurface               *& rpNewSurface)       // out: 
{                          
  // init output
  NER(pOriginalVertex);
  rpNewSurface = NULL;
  r3DTrimmingCurves.ReSet();
  rOrients.ReSet();

  // locals
  double dTolerance = pOriginalVertex->GetBrep()->GetTolerance();
  SmTArray<SmBSplineCurve*> sDeleteCurves;
  SmObjsDelete<SmBSplineCurve*> sCleanCurves(&sDeleteCurves);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; pOriginalVertex->GetBrep()->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; pOriginalVertex->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,1) ; for(ULONG ii=0;ii<rBoundingEdges.GetSize();ii++)
                                    { rBoundingEdges[ii]->Draw() ; sm_GraphicsLoop(); }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // If we have no bounding edges or don't have 3, 
  // just create a spherical surface and hope that the merge will
  // take care of the rest.
  if (rBoundingEdges.GetSize() == 1) 
    {
      // Check for closed case
      SmEdge *pE = rBoundingEdges[0];
      SM_PTR_ARRAY(sFaces, SmFace, 16);  // SmTArray<SmFace *>
      pE->GetFaces(sFaces);
      SmFace *pF = sFaces[0];
      if (pE->IsSeam(*pF->GetSurface())) 
        {
          // no need for a vertex cap Surface
          return SM_SUCCESS;
        }
    } // end just 1 BoundingEdge check

  // Check for case where bounding edges are coincident.
  if (rBoundingEdges.GetSize() == 2) 
    {
      SmCurve *pCurve1 = rBoundingEdges[0]->GetCurve();
      SmCurve *pCurve2 = rBoundingEdges[1]->GetCurve();
      double dMaxDist;
      double dParam2Start = rBoundingEdges[1]->GetInterval().GetMin();
      double dParam2End   = rBoundingEdges[1]->GetInterval().GetMax();
      SER(pCurve1->CurveMaxDistanceBetween(rBoundingEdges[0]->GetInterval(),
                                           *pCurve2,
                                           dParam2Start,dParam2End,
                                           10, &dTolerance, dMaxDist));
      if (dMaxDist < dTolerance)
        {
          // Don't need to do anything here just quit because they are the same
          // geometric edges.
          return SM_SUCCESS;
        }

      // check to see ifthe curves have opposite parameterization
      SER(pCurve1->CurveMaxDistanceBetween(rBoundingEdges[0]->GetInterval(),
                                           *pCurve2,
                                           dParam2End, dParam2Start,
                                           10, &dTolerance, dMaxDist));
      if (dMaxDist < dTolerance)
        {
          // Don't need to do anything here just quit because they are the same
          // geometric edges.
          return SM_SUCCESS;
        }

    } // end 2 coincident bounding curve checks

  // when vertex does not have 3 bounding edges or is part of a lamina edge
  SmBSplineSurface *pSphere = NULL ;
  if (  rBoundingEdges.GetSize() != 3 
      || pOriginalVertex->IsLaminaVertex()) 
    {
      SmBSplineCurve *pArc = NULL;
      SmObjDelete     sClean;
      SmAxis2Placement sPlacement;
      sPlacement.Translate(pOriginalVertex->GetPoint());

      // when there is just 1 bounding edge
      if (rBoundingEdges.GetSize() == 1) 
        {
          SmBSplineCurve* pBlend = SM_CAST_PTR(SmBSplineCurve, rBoundingEdges[0]->GetCurve());
          if (pBlend->IsClosed(rBoundingEdges[0]->GetInterval())) 
            {
              SmAxis2Placement sRefFrame;
              double dArcRad, dStartAng, dEndAng;
              if (pBlend->IsArc(10, dTolerance,sRefFrame,dArcRad,dStartAng,dEndAng))
                {
                  SmVector3d sZAxis = sRefFrame.GetZAxis();
                  SmPoint3d sCenter = pOriginalVertex->GetPoint();
                  SmVector3d sVecToEdge = sRefFrame.GetOriginRef() - sCenter;
                  if (sVecToEdge.Dot(sZAxis) < 0.0) 
                    {
                      sZAxis = -sZAxis;
                    }
                  SmPoint3d sPoint1;
                  SER(pBlend->EvaluatePoint(pBlend->GetNaturalInterval().GetMin(),sPoint1));
                  SmPoint3d sPoint2 = sCenter + m_dOffsetDistance * sZAxis;
                  SER(SmBSplineCurve::CreateArcFromPoints(crContext,3,sCenter,sPoint1,sPoint2,
                      SM_CO_QUADRATIC,pArc));
                }
              sPlacement.SetCanonical(sPlacement.GetOriginRef(),sRefFrame.GetZAxis(),sRefFrame.GetXAxisRef());
            }
        } // end 1 bounding edge check

      // create the vertex endCap surface
      if (!pArc) 
        {
          SER_DELETE(SmBSplineCurve::CreateCircleSegment(crContext,3,sPlacement,m_dOffsetDistance,0,180.0, SM_CO_QUADRATIC, pArc),
                     pArc);
          SER_DELETE(SmBSplineSurface::CreateSurfOfRevolution(crContext,pArc,sPlacement.GetOriginRef(),
                                                       sPlacement.GetXAxisRef(),180.0,pSphere), pArc);
          sClean.SetObj(pArc);
        }
      else 
        {
          SER_DELETE(SmBSplineSurface::CreateSurfOfRevolution(crContext,pArc,sPlacement.GetOriginRef(),
                                                       sPlacement.GetXAxisRef(),360.0,pSphere), pArc);
          sClean.SetObj( pArc );
        }
      rpNewSurface = pSphere;
    }
  else // 3 boundary curves none of which are lamina 
    {
      // In this simple case we will assume that we are doing a constant 
      // radius offset and that all of the edges are arcs of the same radius
      SmTArray<SmBSplineCurve*> sCurves;
      for (ULONG i=0; i<rBoundingEdges.GetSize(); i++) 
        {
          SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, rBoundingEdges[i]->GetCurve());

          //Potential solution for CrvOnSurf edge curves (suspect)
          //SmBSplineCurve* pBSC;

          //// ApproximateCurve locals
          //SmExtent1d sCurveIvl = rBoundingEdges[i]->GetCurve()->GetNaturalInterval();
          //double dMaxGap3d;
          //SmTArray<double> sBreaks;
          //sBreaks.Add(sCurveIvl.GetMin());
          //sBreaks.Add(sCurveIvl.GetMax());

          //// Approximate the curve (copy BSplines)
          //rBoundingEdges[i]->GetCurve()->ApproximateCurve(crContext, sBreaks, SmTol::GetApproxTol3d(this)/2.0, dMaxGap3d,
          //                                                pBSC,
          //                                                FALSE, // in : bOptCreateAnalytics
          //                                                TRUE, // in : bOptMatchParameterization // JLMCC was FALSE
          //                                                TRUE); // in : bJustCopyBSplines

          NER(pBSC);
          sCurves.Add(pBSC);
        }

      // Please note that some of the arcs may not be complete - Check them
      // against the corresponding 3 planes and see if one of them needs to
      // be extended.
      if (sCurves.GetSize() == 3) 
        {
          // Check for shelling case
            {
              SmBSplineCurve *pShellArc = NULL;
              //SmBSplineCurve *pShellLines[3];
              ULONG lLineCount = 0;
              SmAxis2Placement sFrame;
              double dRadius, dStartAng, dEndAng;
              for (ULONG jj=0; jj<sCurves.GetSize(); jj++) 
                {
                  SmBSplineCurve *pLine = sCurves[jj];
                  SmVector3d sLineVector;
                  SmPoint3d sLinePoint;
                  if (pLine->IsArc(10, dTolerance,sFrame,dRadius,dStartAng,dEndAng))
                    {
                      pShellArc = pLine;
                    }
                  if (pLine->IsLine(10, dTolerance,sLinePoint,sLineVector))
                    {
                      //pShellLines[lLineCount] = pLine;
                      lLineCount++;
                    }
                }
              if (pShellArc && lLineCount == 2) 
                {
                  SmBSplineCurve *pPointCurve = NULL ;
                  SER(SmBSplineCurve::CreateDegenerateCurve(crContext,3,
                      sFrame.GetOriginRef(),pPointCurve));
                  SmObjDelete sCleanPC(pPointCurve);
                  // Now that we have the curves create the geometry for the surface.
                  SmBSplineSurface *pPlanarNurb = NULL ;
                  SER(SmBSplineSurface::CreateTrimmedPlaneFromRuled(crContext,
                      *pShellArc,*pPointCurve,r3DTrimmingCurves,
                      rOrients,pPlanarNurb));
                  NER(pPlanarNurb);
                  rpNewSurface = pPlanarNurb;
                  return SM_SUCCESS;
                }
            }

          // Create planes
          SmBoolean bShelling = FALSE;
          double dTol = GetThisApproxTol3d();
          SmVector3d sPlanePoints[3];
          SmVector3d sPlaneNormals[3];
          for (ULONG j=0; j<sCurves.GetSize(); j++) 
            {
              SmBSplineCurve *pArc = sCurves[j];
              SmAxis2Placement sFrame;
              double dRadius, dStartAng, dEndAng;
              // If we have a line then we probably are shelling 
              if (!pArc->IsArc(10,dTol,sFrame,dRadius,dStartAng,dEndAng)) 
                {
                  SER(SM_ERR);
                }
              ULONG lNext = (j+1) % sCurves.GetSize();
              SmBSplineCurve *pOther = sCurves[lNext];
              SmPoint3d sMid;
              SER(pOther->EvaluatePoint(pOther->GetNaturalInterval().Evaluate(0.5),sMid));
              sPlanePoints[j]  = sFrame.GetOriginRef();
              sPlaneNormals[j] = sFrame.GetZAxis();
              SmVector3d sVec = sPlanePoints[j] - sMid;
              if (sVec.Dot(sPlaneNormals[j]) < 0.0) 
                { sPlaneNormals[j] = -sPlaneNormals[j];   }
            }

          // If we are doing shelling just create a non tangential blend.
          if (bShelling) 
            {
              
            }

          // See if all arc points lie on 2 planes
          for (ULONG k=0; k<sCurves.GetSize(); k++) 
            {
              SmBSplineCurve *pArc = sCurves[k];
              SmPoint3d sStart, sEnd;
              pArc->GetEnds(sStart,sEnd);
              for (ULONG kk=0; kk<sCurves.GetSize(); kk++) 
                {
                  if (kk==k) continue;
                  SmPoint3d sProjPoint, sProjPoint2;
                  SER(smgu_PointProjectToPlane(sStart,sPlanePoints[kk],sPlaneNormals[kk],sProjPoint));
                  double dDist1 = sProjPoint.DistanceBetween(sStart);
                  SER(smgu_PointProjectToPlane(sEnd,sPlanePoints[kk],sPlaneNormals[kk],sProjPoint2));
                  double dDist2 = sProjPoint2.DistanceBetween(sEnd);
                  if (dDist1 > dTol && dDist2 > dTol) 
                    {
                      // Have a bad arc because at least one point needs to be on the
                      // plane.
                      ULONG lOther = (kk+1) % 3;
                      if (lOther == k) lOther = (lOther + 1) % 3;
                      
                      SmAxis2Placement sFrame;
                      double dRadius, dStartAng, dEndAng;
                      pArc->IsArc(10,dTol,sFrame,dRadius,dStartAng,dEndAng);
                      SmPoint3d sMid;
                      pArc->EvaluatePoint(pArc->GetNaturalInterval().Evaluate(0.5),sMid);

                      SmPoint3d sClosestPt1;
                        { // Find the closest point to the plane kk
                          ULONG lNumTsect;
                          SmPoint3d sTsectPoints[2];
                          double dClosestDist = SM_BIG_DOUBLE;
                          // Find the closest point to the midpoint of the original arc
                          SER(smgu_CirclePlaneIntersect(dTol,dRadius,sFrame.GetOriginRef(),sFrame.GetZAxis(),
                              sPlanePoints[kk],sPlaneNormals[kk],lNumTsect,sTsectPoints));
                          if (lNumTsect == 3) SER(SM_ERR);
                          for (ULONG mm=0; mm<lNumTsect; mm++) 
                            {
                              double dDist = sTsectPoints[mm].DistanceBetween(sMid);
                              if (dDist < dClosestDist) 
                                {
                                  dClosestDist = dDist;
                                  sClosestPt1 = sTsectPoints[mm];
                                }
                            }
                          if (dClosestDist > dRadius * 10) SER(SM_ERR);
                        }
                      SmPoint3d sClosestPt2;
                        {
                          ULONG lNumTsect;
                          SmPoint3d sTsectPoints[2];
                          double dClosestDist = SM_BIG_DOUBLE;
                          SER(smgu_CirclePlaneIntersect(dTol,dRadius,sFrame.GetOriginRef(),sFrame.GetZAxis(),
                              sPlanePoints[lOther],sPlaneNormals[lOther],lNumTsect,sTsectPoints));
                          if (lNumTsect == 3) SER(SM_ERR);
                          for (ULONG mm=0; mm<lNumTsect; mm++) 
                            {
                              double dDist = sTsectPoints[mm].DistanceBetween(sMid);
                              if (dDist < dClosestDist) 
                                {
                                  dClosestDist = dDist;
                                  sClosestPt2 = sTsectPoints[mm];
                                }
                            }
                          if (dClosestDist > dRadius * 10) SER(SM_ERR);
                        }

                      // Make a new arc using the intersection points.
                      SmBSplineCurve *pNewArc = NULL ;
                      SER(SmBSplineCurve::CreateArcFromPoints(crContext,3,sPlanePoints[0],
                          sClosestPt1,sClosestPt2,SM_CO_QUADRATIC,pNewArc));
                      sDeleteCurves.Add(pNewArc);
                      sCurves[k] = pNewArc;
#ifdef SM_DEBUG_CODE
                      if (bDebugMe) 
                        {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 1,0,0);
                          pArc->Draw();
                          sm_GraphicsLoop();
                          pArc->Dump();
                          smgfx_SetLineWidth(4.0);
                          smgfx_SetLook(1,2, 0,0,1);
                          pNewArc->Draw();
                          sm_GraphicsLoop();
                          pNewArc->Dump();
                          smgfx_SetLineWidth(2.0);
                        }
#endif // SM_DEBUG_CODE

                    }
                }
            }
        } // end exactly 3 curves check

      SmTArray<SmCurve*> sTrimCurves;
      SmBSplineSurface *pNewSurface = NULL ;
      if (SmBSplineSurface::CreateSphereFromArcs(crContext,sCurves, dTolerance,sTrimCurves,rOrients,pNewSurface) != SM_SUCCESS)
        {
          return SM_ERR;
        }
      NER(pNewSurface);
      rpNewSurface = pNewSurface;
      for (ULONG j=0; j<sTrimCurves.GetSize(); j++) 
        {
          SmBSplineCurve *pOrig = SM_CAST_PTR(SmBSplineCurve,sTrimCurves[j]);
          SmBSplineCurve *pNewBSC = new(crContext) SmBSplineCurve(*pOrig);
          r3DTrimmingCurves.Add(pNewBSC);
        }
    } // end given 3 boundary curves branch

  // all done
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::VertexOffset

/*******************************************************************//**
PURPOSE: Update the Surface Domain of the Face and possibly extend the
     surface of the face to cover it.

NOTES: 
***********************************************************************/
SmStatus SmOffsetGeometryCreation::UpdateFaceSurfaceDomain
  (SmFace * pFace,
   SmSurface * pExtendedSurface)
{
    SmExtent2d sFaceDomain;
    if (pFace->CalculateUVDomainFromUVTrimCurves(sFaceDomain) != SM_SUCCESS) {
        SER(SM_ERR);
    }

    SmExtent2d sSurfDomain = pFace->GetSurface()->GetNaturalUVDomain();
    SmPoint2d sMin = sFaceDomain.GetMin();
    SmPoint2d sMax = sFaceDomain.GetMax();
    if (smos_Fabs(sMin.x-sSurfDomain.GetMin().x) < SM_EFF_ZERO) {
        sMin.x = sSurfDomain.GetMin().x;
    }
    if (smos_Fabs(sMin.y-sSurfDomain.GetMin().y) < SM_EFF_ZERO) {
        sMin.y = sSurfDomain.GetMin().y;
    }
    if (smos_Fabs(sMax.x-sSurfDomain.GetMax().x) < SM_EFF_ZERO) {
        sMax.x = sSurfDomain.GetMax().x;
    }
    if (smos_Fabs(sMax.y-sSurfDomain.GetMax().y) < SM_EFF_ZERO) {
        sMax.y = sSurfDomain.GetMax().y;
    }
    SmExtent2d sNewDomain(sMin,sMax);
    pFace->SetUVDomain(sNewDomain);

    if (sNewDomain.IsContainedBy(sSurfDomain, SM_EFF_ZERO)) {
//        SmSurface *pSurf = pFace->GetSurface();
        return SM_SUCCESS; // no extension surface needed
    }

    // Need to replace it with the extended surface
    SmSurface *pExt = NULL;
    SER(pExtendedSurface->Copy(*pFace->GetContext(),pExt)); NER(pExt);
    SM_PTR_ARRAY(sModifiedFaces,SmFace,32);
    SER(pFace->GetBrep()->ReplaceSurface(pFace,pExt,FALSE,NULL));

    return SM_SUCCESS;

} // end SmOffsetGeometryCreation::UpdateFaceSurfaceDomain

/*******************************************************************//**
PURPOSE: Replace the curves of this edge and update vertices.  
      Recompute the orientation flag as well by recomputing it 
      based the relative orientation of the new curve and the old curve.

NOTES: When Geometry of 3DCurve == geometry of Edgeuse->Edge->Curve,
           leaves Edgeuse alone and deleted both 3DCurve and UVCurve.
       When Geometry of 3DCurve != geometry of Edgeuse->Edge->Curve,
           deletes Edgeuse->Edge->Curve and UVTrimCurves,
           sets    Edgeuse->Edge->Curve = 3DCurve  (may reverse parameterization for orientation)
           sets    Edgeuse->UVTrimCurve = UVCurve  (may reverse parameterization for orientation)
           sets    Edgeuse->StartVertexuse->Points = 3DCurve(0.0) eval
           sets    Edgeuse->EndVertexuse->Points   = 3DCurve(1.0) eval
             
***********************************************************************/
SmStatus SmOffsetGeometryCreation::ReplaceEdgeCurves
  (SmEdgeuse        * pEdgeuse,          // in : when rp3DCurve is a new shape, 
                                         //           - assigned rp3DCurve and rpUVCurve deleting any 
                                         //               preExisting Curve and UVTrimCurves.
                                         //      else - not changed.
   SmCurve         *& rp3DCurve,         // i/o: candidate 3dCurve for Edgeuse.    
   SmBSplineCurve  *& rpUVCurve,         // i/o: candidate UVTrimCurve for Edgeuse.
   const SmEdgeuse  * cpOptOrigEdgeuse,  // in : optional Edgeuse used to compute orientation of rp3DCurve to Edgeuse, NULL to ignore
   SmBoolean        & rbDelete3DCurve,   // o: if true, then caller should delete 3DCurve.
   SmBoolean        & rbDeleteUVCurve)   // o: if true, then caller should delete UVCurve.
{
  // locals
  SmEdge  * pEdge    = pEdgeuse->GetEdge() ;
  SmCurve * pEUCurve = pEdgeuse->GetEdge()->GetCurve() ;

  rbDelete3DCurve = FALSE;
  rbDeleteUVCurve = FALSE;

  // when given a 3dCurve with a new shape - Update pEdge->Curve = rp3DCurve
  if (rp3DCurve) 
    {
      SM_ASSERT_MSG(rp3DCurve != pEUCurve, _T("SmOffsetGeometryCreation::ReplaceEdgeCurves - asked to replace Curve with itself causing a stale pointer")) ; 
      SmPoint3d sPV[2], sPV2[2];

      // Get rp3DCurve and pEdgeuse->Curve endPoints
      SmPoint3d sEdgeuseStart,  sEdgeuseEnd ;
      SmPoint3d sCurveStart,    sCurveEnd ;

      // Edgesuse->Curve Start and End pts
      SER(pEUCurve->EvaluatePoint(pEdge->GetInterval().GetMin(),sEdgeuseStart));
      SER(pEUCurve->EvaluatePoint(pEdge->GetInterval().GetMax(),sEdgeuseEnd));

      // input 3DCurve- Start and End pts
      SER(rp3DCurve->EvaluatePoint(rp3DCurve->GetNaturalInterval().GetMin(),sCurveStart));
      SER(rp3DCurve->EvaluatePoint(rp3DCurve->GetNaturalInterval().GetMax(),sCurveEnd));

      // low work - Edgesuse->Curve and 3DCurve start and end Points are within tol
      if(   (sEdgeuseStart.DistanceBetween(sCurveStart) < pEdge->GetTolerance()) 
         && (sEdgeuseEnd.DistanceBetween(sCurveEnd)     < pEdge->GetTolerance())) 
        {
          // No change in ends of curve - don't bother using the inputs - delete them and get out.
          SM_ASSERT(rp3DCurve != NULL) ; rbDelete3DCurve = TRUE ;
          SM_ASSERT(rpUVCurve != NULL) ; rbDeleteUVCurve = TRUE ;

          // all done
          return SM_SUCCESS;
        } // end low work - input curves are same as existing Edgeuse->Curve

      // low work - Edgeuse->Curve and 3DCurve start and end Points are within tol with reverse parameterization.
      if(   (sEdgeuseStart.DistanceBetween(sCurveEnd) < pEdge->GetTolerance()) 
         && (sEdgeuseEnd.DistanceBetween(sCurveStart) < pEdge->GetTolerance()))
        {
          // No change in ends of curve - just delete the curves and get out
          SM_ASSERT(rp3DCurve != NULL) ; rbDelete3DCurve = TRUE ;
          SM_ASSERT(rpUVCurve != NULL) ; rbDeleteUVCurve = TRUE ;

          // all done
          return SM_SUCCESS;
        } // end low work - input curves are reverse parameterized existing edgeuse->Curve

      // Edgeuse MidPoint (use cpOptOrigEdgeuse if given)
      if(cpOptOrigEdgeuse == NULL) { double dParam = pEdge->GetInterval().Evaluate(0.5) ;
                                     SER(pEUCurve->Evaluate(dParam,1,TRUE,sPV)); 
                                   }
      else                         { SmCurve * pOrigCurve = cpOptOrigEdgeuse->GetEdge()->GetCurve() ;
                                     double    dParam     = cpOptOrigEdgeuse->GetEdge()->GetInterval().Evaluate(0.5) ;
                                     SER(pOrigCurve->Evaluate(dParam,1,TRUE,sPV));
                                   }
      
      // input 3DCurve midPoint
      SER(rp3DCurve->Evaluate(rp3DCurve->GetNaturalInterval().Evaluate(0.5),1,TRUE,sPV2));
      
      // when input and existing edge curves run in opposite directions - reverse 3DCurve and UVCurve
      if (sPV[1].Dot(sPV2[1]) < 0.0) 
        { 
          // Opposite orienation - reverse 3DCurve and UVCurve
          SmExtent1d sIvl;
          rp3DCurve->ReverseParameterization(rp3DCurve->GetNaturalInterval(),sIvl);
          rpUVCurve->ReverseParameterization(rpUVCurve->GetNaturalInterval(),sIvl);
        
        } // end Edge and Input curve running in opposite directions check
      
      // gwc: obsolete
      //  // delay curve delete until after the Edge->Curve and Curve->Owner pointers are changed
      //  SmObjDelete sCleanEdge ;

// Remove Composites
//       // when curve belongs to an Edge and not a CEdge
//       if (pEUCurve->GetOwner() == pEdge) 
//         {
// #ifdef SM_DEBUG_CODE
// SmBoolean bDebugMe = FALSE;
//           if (bDebugMe) 
//             {
//               pEUCurve->Dump();
//               rp3DCurve->Dump();
// 
//               smgfx_Erase();
//               smgfx_SetLook(1,2, 1,0,0); rp3DCurve->Draw(); sm_GraphicsLoop();
//               smgfx_SetLook(4,5, 0,0,1); pEUCurve->Draw(); sm_GraphicsLoop();
//               sm_GraphicsLoop();
//             }
// #endif // SM_DEBUG_CODE
// 
//           // gwc: obsolete
//           //   // delete the curve after updating Edge->Curve and Curve->Owner pointers
//           //   SM_ASSERT(pEUCurve != NULL) ; sCleanEdge.SetObj(pEUCurve) ; 
//           //   // SM_ASSERT(pEUCurve != NULL) ; delete pEUCurve ; pEUCurve = NULL ;
//         }  
//       else // Curve belongs to a CompositeEdge
//         { 
//           SmCEdge *pCompEdge = SM_CAST_PTR(SmCEdge,pEUCurve->GetOwner());
//           NER(pCompEdge);
//           SER(pCompEdge->RemoveEdge(pEdge)); // note: pEdge->Curve set to NULL not deleted, UVTrimCurves are deleted.
//         }
      
      // set input curves as Edge geometry
      pEdge->SetCurve(rp3DCurve, TRUE ) ; // TRUE = delete preExisting Edge->Curve - expect none here
                                          // side effect: delete current pSurviveEdge->UVTrimCurve
      pEUCurve = NULL ; // done with pEUCurve - it's been deleted
      rp3DCurve->SetOwner(pEdge);
      pEdge->SetInterval(rp3DCurve->GetNaturalInterval());
    
    } // end rp3DCurve existence check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmBSplineCurve *pOldUV = pEdgeuse->GetUVTrimCurve();
      if (pOldUV) 
        {
          pOldUV->Dump();
          rpUVCurve->Dump();

          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0); if(pOldUV) pOldUV->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 0,0,1); if(rpUVCurve) rpUVCurve->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // Update Edgeuse UVTrimCurve = rpUVCurve
  pEdgeuse->SetUVTrimCurve(rpUVCurve,pEdge->GetTolerance(),TRUE) ; // TRUE = delete existing UVTrimCurve
  
  // when given a 3DCurve - Update pEdgeuse Start and End Vertex->Positions = rp3DCurve->Evals
  if (rp3DCurve) 
    {
      SmPoint3d sPnt;
      
      // Update the geometry of the start vertex.
      SmVertex *pV = pEdgeuse->GetVertexuse()->GetVertex();
      SER(pEdgeuse->NormalizedEvaluate(0.0,FALSE,sPnt));  // TRUE = UV Eval, FALSE = 3d Eval
      pV->SetPoint(sPnt);

      // Update the geometry of the other vertex
      pV = pEdgeuse->GetMate()->GetVertexuse()->GetVertex();
      SER(pEdgeuse->NormalizedEvaluate(1.0,FALSE,sPnt));  // TRUE = UV Eval, FALSE = 3d Eval
      pV->SetPoint(sPnt);
    
    } // end given 3DCurve check

  // all done
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::ReplaceEdgeCurves
      
/*******************************************************************//**
PURPOSE: Trim the Extended Edge using various techniques.

NOTES: 
  Select trimPoints by generating candidate trim points by the following 
  techniques and selecting the TrimPoint on either end which is closest 
  to the BaseEdgeuse vertices.
  
      1) Use existing vertex of adjacent object if exists.
      2) Intersect curve with Extended surface of adjacent face (convex, concave edge)
      3) Intersection of edges extended in parameter space (tangent edge case)
      4) Take a wild guess??

***********************************************************************/
SmStatus SmOffsetGeometryCreation::TrimExtendedEdge
  (const SmContext   & crContext,            // in : context for new object construction 
   SmOffsetExecutive * cpOffsetExecutive,    // in : offset operation context data
   const SmEdgeuse   * pBaseEdgeuse,         // in : 1st target edgeuse
   const SmEdgeuse   * pOtherBaseEdgeuse,    // in : 2nd target edgeuse
   SmEdgeuse         * pOffsetEdgeuse,       // in : Offset of 1st target edgeuse
   SmEdgeuse         * pOtherOffsetEdgeuse,  // NotUsed: in : Offset of 2nd target edgeuse
   SmCurve           * p3DCurve,             // in : XSectCurve between edgeuse->Face->ExtendedSurfaces
   SmBSplineCurve    * pUVCurveBase,         // in : UVTrimCurve on 1st target edgeuse->Face->ExtendedSurface
   SmBSplineCurve    * pUVCurveOtherBase,    // in : UVTrimCurve on 2nd target edgeuse->Face->ExtendedSurface 
   SmBoolean         & bGoodEdge,            // out: TRUE = TrimPoints were found for the input 3DCurve
   SmExtent1d        & rTrimInterval,        // out: TrimInterval for the 3DCurve
   SmMarkType          eMarkType)            // in : uses without increment eMarkType value
{
  SM_REF2(pOffsetEdgeuse, pOtherOffsetEdgeuse) ;
  // init output
  bGoodEdge = FALSE;

  // locals
  SmExtent1d sTrimIvl;
  SmSolution sData[16];
  SmSolution sData2[16];
  SmSolutionArray sSolutions(16,sData);
  SmSolutionArray sSolutions2(16,sData2);
  double dTanTolDeg = cpOffsetExecutive->GetTangencyTolDegrees();
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      p3DCurve->Dump();
      pUVCurveBase->Dump();
      pUVCurveOtherBase->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0); p3DCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pOffsetEdgeuse->GetEdge()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pOffsetEdgeuse->GetEdge()->GetBrep()->Draw();  sm_GraphicsLoop();
      smgfx_SetLook(2,4, 0,1,0); pOffsetEdgeuse->GetFace()->DrawUV(4,4);  sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for two iterations - find a one 3DCurve trim point
  for (ULONG i=0; i<2; i++) 
    {
      sSolutions.ReSet();

      // Let's first try to take care of start vertex of pBaseEdgeuse
      SmVertex *pBaseVertex = NULL;
      SM_PTR_ARRAY(sOffAdjEUs,SmEdgeuse,     20);  // list of     
      SM_PTR_ARRAY(sTanEUs,   SmEdgeuse,     20);  // list of NextEdgeuses between tangent faces    
      SM_PTR_ARRAY(sTanUVs,   SmBSplineCurve,20);  // list of BaseCurve UVTrimCurves when NextEdgeuse is a tangent sector    

      // on the 1st pass
      if (i==0) 
        { 
          // start at BaseEdgeuse->StartVertex
          pBaseVertex     = pBaseEdgeuse->GetVertexuse()->GetVertex();
          SmEdgeuse *pBCW = pBaseEdgeuse->GetCWEdgeuse();

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pBaseEdgeuse->GetVertexuse()->GetVertex()->GetPoint().Draw();
              smgfx_SetLook(1,2, 0,0,1); pBaseEdgeuse->GetEdge()->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 1,0,0); pBCW->GetEdge()->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 1,0,0); p3DCurve->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); pOffsetEdgeuse->GetEdge()->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pOffsetEdgeuse->GetEdge()->GetBrep()->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pOtherBaseEdgeuse->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // See if base edgeuse is tangent and save it along with the curve on the
          // correct surface to be used during intersection.
          if (pBCW != pBaseEdgeuse) 
            {
              // When next edgeuse is either a tangentSector or lamina
              // set up to try trimming 3Dcurve to BCWEdgeuse/ExtendedUVCurve intersection point
              if (   pBCW->GetEdge()->IsLamina()
                  || pBCW->IsTangentSector(dTanTolDeg)) 
                {
                  // save it and this baseEdgeuse in the Tan lists
                  sTanEUs.Add(pBCW);
                  sTanUVs.Add(pUVCurveBase);
                }
              else // Look for intersecting surface
                { 
                  // set up to try trimming 3DCurve to OffAdjEU->Face->ExtendedSurface/3DCurve intersection
                  if (!pBaseVertex->AreEdgesTangent(pBaseEdgeuse->GetEdge(),pBCW->GetEdge(),1.0)) 
                    {
                      SmEdgeuse * pAdjEU    = pBCW->GetRadial();
                      SmEdgeuse * pOffAdjEU = cpOffsetExecutive->GetOffsetEdgeuse(pAdjEU); 
                      if (pOffAdjEU) { sOffAdjEUs.Add(pOffAdjEU); }
                      //                    else { sOffAdjEUs.Add(pAdjEU); }
                    }
                }
            }

          // See if otherBase edgeuse is tangent and save it along with the curve on the
          // correct surface to be used during intersection.
          if (pOtherBaseEdgeuse) 
            {
              SmEdgeuse *pOBCCW = pOtherBaseEdgeuse->GetCCWEdgeuse();
              if (pOBCCW != pOtherBaseEdgeuse) 
                {
                  // When next edgeuse is either a tengentSector or lamina
                  if (   pOBCCW->GetEdge()->IsLamina()
                      || pOBCCW->IsTangentSector(dTanTolDeg)) 
                    {
                      // save it and this baseEdgeuse in the Tan lists
                      sTanEUs.Add(pOBCCW);
                      sTanUVs.Add(pUVCurveOtherBase);
                    }
                  else 
                    { // Look for intersecting surface
                      SmVertex *pOtherBaseVertex = pBaseEdgeuse->GetEdge()->GetOtherVertex(pBaseVertex);
                      if (!pOtherBaseVertex->AreEdgesTangent(pOtherBaseEdgeuse->GetEdge(),pOBCCW->GetEdge(),1.0)) 
                        {
                          SmEdgeuse * pAdjEU2   = pOBCCW->GetRadial();
                          SmEdgeuse * pOffAdjEU = cpOffsetExecutive->GetOffsetEdgeuse(pAdjEU2); 
#ifdef SM_DEBUG_CODE
                          if (bDebugMe) 
                            {
                              smgfx_Erase();
                              smgfx_SetLook(1,2, 0,0,1) ; pOffAdjEU->Draw(); sm_GraphicsLoop();
                              smgfx_SetLook(1,2, 0,1,0) ; pOffAdjEU->GetFace()->Draw(); sm_GraphicsLoop();
                              sm_GraphicsLoop();
                            }
#endif // SM_DEBUG_CODE
                          if (pOffAdjEU) 
                            { 
                              sOffAdjEUs.Add(pOffAdjEU); 
                            }
                          //                        else { sOffAdjEUs.Add(pAdjEU2); }
                        }
                    }
                }
            }
        } // end is 1st iteration check
      else // 2nd iteration
        { 
          // start at BaseEdgeuse->EndVertex
          pBaseVertex      = pBaseEdgeuse->GetMate()->GetVertexuse()->GetVertex();
          SmEdgeuse *pBCCW = pBaseEdgeuse->GetCCWEdgeuse();

          // See if base edgeuse is tangent and save it along with the curve on the
          // correct surface to be used during intersection.
          if (pBCCW != pBaseEdgeuse) 
            {
              if (   pBCCW->GetEdge()->IsLamina()
                  || pBCCW->IsTangentSector(dTanTolDeg)) 
                {
                  // When next edgeuse is either a tengentSector or lamina
                  sTanEUs.Add(pBCCW);
                  sTanUVs.Add(pUVCurveBase);
                }
              else 
                { // Look for intersecting surface
                  if (!pBaseVertex->AreEdgesTangent(pBaseEdgeuse->GetEdge(),pBCCW->GetEdge(),1.0)) 
                    {
                      SmEdgeuse * pAdjEU = pBCCW->GetRadial();
                      SmEdgeuse *pOffAdjEU = cpOffsetExecutive->GetOffsetEdgeuse(pAdjEU); 
                      if (pOffAdjEU) { sOffAdjEUs.Add(pOffAdjEU); }
                      //                    else { sOffAdjEUs.Add(pAdjEU); }
                    }
                }
            }
          
          // See if otherBase edgeuse is tangent and save it along with the curve on the
          // correct surface to be used during intersection.
          if (pOtherBaseEdgeuse) 
            {
              SmEdgeuse *pOBCW = pOtherBaseEdgeuse->GetCWEdgeuse();
              if (pOBCW != pOtherBaseEdgeuse) 
                {
                  // When next edgeuse is either a tengentSector or lamina
                  if (   pOBCW->GetEdge()->IsLamina()
                      || pOBCW->IsTangentSector(dTanTolDeg)) 
                    {
                      // save it and this baseEdgeuse in the Tan lists
                      sTanEUs.Add(pOBCW);
                      sTanUVs.Add(pUVCurveOtherBase);
                    }
                  else 
                    { // Look for intersecting surface
                      SmVertex *pOtherBaseVertex = pBaseEdgeuse->GetEdge()->GetOtherVertex(pBaseVertex);
                      if (!pOtherBaseVertex->AreEdgesTangent(pOtherBaseEdgeuse->GetEdge(),pOBCW->GetEdge(),1.0)) 
                        {
                          SmEdgeuse * pAdjEU2 = pOBCW->GetRadial();
                          SmEdgeuse * pOffAdjEU = cpOffsetExecutive->GetOffsetEdgeuse(pAdjEU2); 
                          if (pOffAdjEU) { sOffAdjEUs.Add(pOffAdjEU); }
                          //                              else { sOffAdjEUs.Add(pAdjEU2); }
                        }
                    }
                }
            }
        } // end 2nd iteration branch
      
      // Tangent sector handling - extend parameter space and intersect it
      // for one or two curves.
      for (ULONG itry = 0; itry<sTanEUs.GetSize(); itry++) 
        {
          SmEdgeuse      * pTanEU   = sTanEUs[itry];
          SmBSplineCurve * pTanUV   = sTanUVs[itry];
          SmBSplineCurve * pUVCurve = pTanEU->GetUVTrimCurve();
#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_Erase();
              p3DCurve->Draw(); sm_GraphicsLoop();
              pBaseEdgeuse->Draw(); sm_GraphicsLoop();
              pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
              pTanEU->GetEdge()->Draw(); sm_GraphicsLoop();
              pTanEU->GetFace()->Draw(); sm_GraphicsLoop();
              pBaseEdgeuse->GetEdge()->GetBrep()->Draw(); sm_GraphicsLoop();
              pOffsetEdgeuse->GetEdge()->GetBrep()->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // If this has already been mated then we should just grab the point
          // of the vertex and 
          SmBoolean bFound3DVertex = FALSE;
          if (   pTanEU->IsMarked(eMarkType) 
              && pTanEU->GetMate()->IsMarked(eMarkType)) 
            {
              SM_PTR_ARRAY(sALLEUS,SmEdgeuse,32);
              pTanEU->GetEdge()->GetEdgeuses(sALLEUS);
              for (ULONG ieu=0; ieu<sALLEUS.GetSize(); ieu++) 
                {
                  // skip
                  SmEdgeuse *pOffEU = cpOffsetExecutive->GetOffsetEdgeuse(sALLEUS[ieu]); 
                  if (!pOffEU) { continue; }
                  SmVertex *pV = pOffEU->GetVertexuse()->GetVertex();

                  // get point on 3DCurve closest to pOffEu->StartVertex
                  double d3DTol = m_dThisApproxTol3d * 10.0;
                  SER(p3DCurve->GlobalPointSolve(p3DCurve->GetNaturalInterval(),SM_SO_MINIMIZE,
                      pV->GetPoint(),d3DTol,&d3DTol,NULL,SM_SR_ALL,sSolutions2));

                  // save solutions
                  if (sSolutions2.GetSize() > 0) 
                    {
                      sSolutions.Append(sSolutions2);
                      bFound3DVertex = TRUE;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
                      if (bDebugMe3) 
                        {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 0,0,1); pV->GetPoint().Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 1,0,0); p3DCurve->DrawAt(sSolutions2[0].m_vStart[0],0); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 0,0,0); p3DCurve->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
#endif // SM_DEBUG_CODE
                    } // end found a solution check
                  
                  // not get point on 3DCurve closest to pOffEd0->EndVertex
                  pV = pOffEU->GetEdge()->GetOtherVertex(pV);
                  if (!pV) continue; // Closed Curve Case
                  SER(p3DCurve->GlobalPointSolve(p3DCurve->GetNaturalInterval(),SM_SO_MINIMIZE,
                      pV->GetPoint(),d3DTol,&d3DTol,NULL,SM_SR_ALL,sSolutions2));

                  // save solutions
                  if (sSolutions2.GetSize() > 0) 
                    {
                      sSolutions.Append(sSolutions2);
                      bFound3DVertex = TRUE;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
                      if (bDebugMe3) 
                        {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 0,0,1); pV->GetPoint().Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 1,0,0); p3DCurve->DrawAt(sSolutions2[0].m_vStart[0],0); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 0,0,0); p3DCurve->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                        }
#endif // SM_DEBUG_CODE
                    }
                }
            } // end pTanEu and its mate are marked checks
          
          // If we didn't find a 3D vertex from the offset close enough then
          // we need to do UV curve extension and intersections to solve for
          // this tangency case.
          if ( ! bFound3DVertex  &&  pTanUV != NULL ) 
            {
              // get a temporary copy of the pTanEU->UVTrimCurve
              double dMaxDist = 0.0;
              if (!pUVCurve) { SER(pTanEU->CreateUVTrimCurve(dMaxDist,pUVCurve)); }
              else
              {
                  // Temp workaround to appease Linux gcc compiler
                  // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
                  // SER(pUVCurve->Copy(crContext,(SmCurve*&)pUVCurve));

                  SER(pUVCurve->Copy(crContext, pUVCurve));
              }
              SmObjDelete sCleanUV(pUVCurve);
              
              // set pUVCurve extension amounts
              double dCurveExtensionFactor = 10.0;  // Curve Extension Factor 
              SmExtent1d sIvl             = pUVCurve->GetNaturalInterval();
              double dExtensionParameter  = sIvl.GetMin() - dCurveExtensionFactor*sIvl.GetLength();
              double dExtensionParameter2 = sIvl.GetMax() + dCurveExtensionFactor*sIvl.GetLength();
              
              // extend both ends of pUVCurve
              SmBSplineCurve *pCurveExtended  = NULL ;
              SmBSplineCurve *pCurveExtended2 = NULL ;
              SmBoolean     bPreciseExtension = TRUE;  // Otherwise it adds another length of sIvl.
              SER(pUVCurve      ->CreateExtendedCurve(crContext,dExtensionParameter, SM_CT_G1_G2,pCurveExtended , bPreciseExtension )); 
              SER(pCurveExtended->CreateExtendedCurve(crContext,dExtensionParameter2,SM_CT_G1_G2,pCurveExtended2, bPreciseExtension )); 
              SmObjDelete sCleanExt(pCurveExtended);
              SmObjDelete sCleanExt2(pCurveExtended2);
              SER(pCurveExtended2->RemoveExtraKnots(SM_EFF_ZERO_SQRT/100.0));
              
              // intersect pTanUV/Extended pTanEU->UVTrimCurve
              SER(pTanUV->GlobalCurveIntersect(pTanUV->GetNaturalInterval(),*pCurveExtended2,
                  pCurveExtended2->GetNaturalInterval(), SM_EFF_ZERO_SQRT, sSolutions2));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
              if (bDebugMe2) 
                { sSolutions2.Dump(); }
#endif // SM_DEBUG_CODE

              // save solutions as possible trim Points
              sSolutions.Append(sSolutions2);
            }
        } // end iter every sTanEU/sTanUV pair of members
      
      
      // Now handle intersecting faces if there are any
      
      // for every sOffAdjEU - try letting trimPoint = OffAdjEu->ExtendedSurface/3DCurve intersection
      SmFace *pLastFace = NULL;
      for (ULONG jj=0; jj<sOffAdjEUs.GetSize(); jj++) 
        {
          SmEdgeuse * pOffAdjEU = sOffAdjEUs[jj];
          SmFace    * pF        = pOffAdjEU->GetFace();

          // skip LastFace
          if (pF == pLastFace) continue;

          // Extend pF->Surface
          pLastFace = pF;
          SmSurface *pExtSurface = NULL;
          SmExtent2d sExtDomain;
          double d3DTol = pF->GetBrep()->GetTolerance();
          SER(cpOffsetExecutive->GetCreateExtendedSurface(pF->GetSurface(),sExtDomain,pExtSurface));
          NER(pExtSurface);

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              SmBrep *pBrep = pF->GetBrep(); 

              smgfx_Erase();
              smgfx_SetLook(3,4, 1,0,0) ; p3DCurve->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,0) ; pBaseEdgeuse->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,0,1) ; pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,1) ; pOffAdjEU->GetEdge()->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0) ; pF->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1) ; pExtSurface->DrawUV(10,10); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // intersect pF->ExtendedSurface with 3DCurve
          SmExtent1d sCrvIvl = p3DCurve->GetNaturalInterval();
          if (pExtSurface->GlobalCurveIntersect(sExtDomain,*p3DCurve,sCrvIvl,d3DTol,sSolutions2) != SM_SUCCESS) 
            {
              continue;
            }

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              SmBrep *pBrep = pF->GetBrep();
              
              sSolutions2.Dump() ; 

              smgfx_Erase();
              smgfx_SetLook(3,4, 1,0,0) ; p3DCurve->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,0) ; pBaseEdgeuse->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,0,1) ; pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,1) ; pOffAdjEU->GetEdge()->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0) ; pF->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1) ; pExtSurface->DrawUV(10,10); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 1,0,0) ; sSolutions2.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Note that the adjacent surface intersection overrides tangent intersections
          // Test the solutions to see if they are not nearly tangent.
          for (ULONG kk=0; kk<sSolutions2.GetSize(); kk++) 
            {
              SmSolution & rSol = sSolutions2[kk];
              SmPoint3d sPV[2];
              p3DCurve->Evaluate(rSol.m_vStart[0],1,TRUE,sPV);
              SmPoint2d sUV(rSol.m_vStart[1],rSol.m_vStart[2]);
              SmVector3d sNormal;
              pExtSurface->EvaluateNormal(sUV,TRUE,TRUE,sNormal);
              sPV[1].Unitize();
              double dAngleRad = 0.0;
              SER(sPV[1].AngleBetween(sNormal,dAngleRad));
              double dAngleDeg = dAngleRad * 180.0/SM_PI;
              if (   dAngleDeg < 85 
                  || dAngleDeg > 95) 
                {
                  // save solution as possible trimPoint
                  sSolutions.Add(rSol);
                }
            }
        } // end iter every sOffAdjEU
      
      SmBoolean bFound = FALSE;
      double dFoundDist = SM_BIG_DOUBLE;
      double dFoundParam =0.0;
      //double dOtherFoundParam=0.0;

      // If we have no candidate TrimPoints found by intersecting the 3DCurve with other geometry,
      // get a candidate TrimPoint by dropping the Vertex->Point onto the 3DCurve
      if (sSolutions.GetSize() == 0) 
        {
          SmExtent1d sCrvIvl = p3DCurve->GetNaturalInterval();
          double d3DTol = pBaseEdgeuse->GetEdge()->GetBrep()->GetTolerance();
          SER(p3DCurve->GlobalPointSolve(sCrvIvl,SM_SO_MINIMIZE,pBaseVertex->GetPoint(),
                                         d3DTol, NULL, NULL, SM_SR_ALL,sSolutions));
        }

      // search every solution for the one closest to the BaseVertex to use as the trimPoint
      for (ULONG ii=0; ii<sSolutions.GetSize(); ii++) 
        {
          SmSolution &rSol = sSolutions[ii];
          SmPoint3d sPnt;
          SER(p3DCurve->EvaluatePoint(rSol.m_vStart[0],sPnt));

          // search for the best solution
          double dDistMoved = sPnt.DistanceBetween(pBaseVertex->GetPoint());
          if (   dDistMoved < dFoundDist
              && !SM_ARE_SAME(rSol.m_vStart[0],sTrimIvl.GetMin())) 
            {
              dFoundParam      = rSol.m_vStart[0];
              //dOtherFoundParam = rSol.m_vStart[1];
              dFoundDist       = dDistMoved;
              bFound           = TRUE;
            }

          // If solution is a range, e.g. coincident curves, check m_vEnd
          if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
            {
              SER(p3DCurve->EvaluatePoint(rSol.m_vEnd[0], sPnt));
              dDistMoved = sPnt.DistanceBetween(pBaseVertex->GetPoint());

            if (   dDistMoved < dFoundDist
                && !SM_ARE_SAME(rSol.m_vEnd[0],sTrimIvl.GetMin())) 
              {
                dFoundParam      = rSol.m_vEnd[0];
                dFoundDist       = dDistMoved;
                bFound           = TRUE;
              }
            }
        }

      // BadEdge when no trim point was found for this iteration
      if (!bFound) 
        {
          bGoodEdge = FALSE;
          return SM_SUCCESS;
        }

      // else set Interval
      sTrimIvl.AddValue(dFoundParam);
    
    } // end iter twice
  
  // cull zero length curves
  double dScaledZero = SM_EFF_ZERO * (1.0 + sTrimIvl.GetMaxDimension()) ;
  if(sTrimIvl.GetLength() > dScaledZero)
    {
      bGoodEdge     = TRUE;
      rTrimInterval = sTrimIvl;
    }
  else
    { bGoodEdge     = FALSE;
    }
  
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::TrimExtendedEdge

/*******************************************************************//**
PURPOSE:  Extend or Trim Lamina or Tangent Edges

NOTES: 
***********************************************************************/
SmStatus SmOffsetGeometryCreation::ExtendTrimEdges
  (const SmContext   & crContext,             // in : context for new object construction
   SmOffsetExecutive * cpOffsetExecutive,     // in : offset executive for this offset operation
   const SmEdgeuse   * pEdgeuse,              // in : edgeuse             from orig brep
   const SmEdgeuse   * pOtherEdgeuse,         // in : radial mate edgeuse from orig brep, NULL if lamina
   SmEdgeuse         * pOffsetEdgeuse,        // in : corresponding edgeuse             from offset brep            
   SmEdgeuse         * pOtherOffsetEdgeuse,   // in : corresponding radial mate edgeuse from offset brep, NULL if lamina
   SmMarkType          eMarkType)             // in : uses without incrementing eMarkType value
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      pEdgeuse->GetEdge()->GetCurve()->Dump();
      pEdgeuse->GetEdge()->Dump();

      smgfx_Erase();
      smgfx_SetLook(3,5, 1,0,0); pEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,1,1);
      if(pOtherEdgeuse) { pOtherEdgeuse->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook(3,5, 1,0,1); pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,0,1);
      if(pOtherOffsetEdgeuse) { pOtherOffsetEdgeuse->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook( 2, 4, 1, 0, 1 ); { pOffsetEdgeuse->GetFace()->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook(2,4, 0,1,1);
      if(pOtherOffsetEdgeuse) { pOtherOffsetEdgeuse->GetFace()->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 1,0,0); pOffsetEdgeuse->GetFace()->GetBrep()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); pEdgeuse->GetFace()->GetBrep()->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

  // locals
  SmSolution      sData[16];
  SmSolutionArray sSolutions(16,sData);
  SmSolution      sData2[16];
  SmSolutionArray sSolutions2(16,sData2);
  SmBSplineCurve *pUVCurve      = pEdgeuse->GetUVTrimCurve();
  SmBSplineCurve *pOtherUVCurve = NULL;
  SmBoolean       bUseRadUV     = FALSE;
  SmPoint3d       sOtherMid;

  // Let's first decide if edgeuse or radial UVTrimCurve is optimal TrimCurve to take.  
  // We like ISO curves the best.
  
  // when Edgeuse is not lamina - pick the best UVTrimCurve on Edgeuse and RadialEdgeuse
  if ( ! pEdgeuse->GetEdge()->IsLamina() )
    {
      SmBSplineCurve *pUVRad = pEdgeuse->GetRadial()->GetUVTrimCurve();
      
      // Edgeuse and RadialEdgeuse UVCurve bounding boxes and sizes
      SmExtent3d  sUVBBox, sUVRadBBox;
      SER(pUVCurve->CalculateBoundingBox(pUVCurve->GetNaturalInterval(),&sUVBBox));
      SER(pUVRad->CalculateBoundingBox(pUVCurve->GetNaturalInterval(),&sUVRadBBox));
      SmPoint3d sUVSize    = sUVBBox.GetSize();
      SmPoint3d sUVRadSize = sUVRadBBox.GetSize();

      double dMinUV    = smos_Min(sUVSize.x,sUVSize.y);
      double dMinRadUV = smos_Min(sUVRadSize.x,sUVRadSize.y);

      // let pUVCurve be the Edgeuse or RadialEdgeuse curve that's the most isoparameter like
      // But: if we use the radial uv curve, then we have to have pOtherOffsetEdgeuse,
      // otherwise pExtSurf will not get created (see below). [B473]
      if (   ( dMinRadUV < dMinUV-SM_EFF_ZERO )
          && ( pOtherOffsetEdgeuse != NULL    ) )
        {
          pOtherUVCurve = pUVCurve;
          pUVCurve      = pUVRad;
          bUseRadUV     = TRUE;
        }
      else
        {
          pOtherUVCurve = pUVRad;
          // pUVCurve  already set to pEdgeuse->GetUVTrimCurve();
          // bUseRadUV already set to FALSE
        }
      // refresh locals
      SmExtent1d sOrigIvl = pOtherUVCurve->GetNaturalInterval();
      SER(pOtherUVCurve->EvaluatePoint(sOrigIvl.Evaluate(0.5),sOtherMid));
    
    } // end edgeuse is not lamina check
  
  // Create a greatly extended curve and trim it to the bounding box of the extended surface.
  SmExtent1d       sIvl             = pUVCurve->GetNaturalInterval();
  double           dNewMin          = sIvl.GetMin() - 10.0*sIvl.GetLength(); 
  double           dNewMax          = sIvl.GetMax() + 10.0*sIvl.GetLength(); 
  SmBSplineCurve * pCurveExtended   = NULL ;
  SmBSplineCurve * pUVCurveExtended = NULL;
  SmBoolean bPreciseExtension = TRUE;  // Otherwise it adds another length of sIvl.

  // pCurveExtended = Extend the UVCurve to the NewMin parameter value
  SER(pUVCurve->CreateExtendedCurve(crContext,dNewMin,SM_CT_G1_G2,pCurveExtended, bPreciseExtension ));
  SmObjDelete sClean1(pCurveExtended);

  // pUVCurveExtended = ReExtend the Extended Curve to the NewMax parameter value
  SER(pCurveExtended->CreateExtendedCurve(crContext,dNewMax,SM_CT_G1_G2,pUVCurveExtended, bPreciseExtension ));
  SmObjDelete sClean2(pUVCurveExtended);

  // Clean up extended curve pUVCurveExtended
  SmExtent1d sExtIvl = pUVCurveExtended->GetNaturalInterval();
  SER(pUVCurveExtended->RemoveExtraKnots(SM_EFF_ZERO_SQRT/100.0));
  
  // locals for surface extension
  SmSurface * pExtSurf      = NULL;
  SmExtent2d  sExtDomain;
  SmSurface * pOtherExtSurf = NULL;
  SmExtent2d  sOtherExtDomain;

  // when using the Edgeuse UVTrimCurve
  if ( ! bUseRadUV )
    {
      // extend the surface
      SmSurface *pSurf = pOffsetEdgeuse->GetFace()->GetSurface();
      SER(cpOffsetExecutive->GetCreateExtendedSurface(pSurf,sExtDomain,pExtSurf));

      if ( ! pEdgeuse->GetEdge()->IsLamina()  &&  pOtherOffsetEdgeuse != NULL )
        {
          SmSurface *pSurf2 = pOtherOffsetEdgeuse->GetFace()->GetSurface();
          SER(cpOffsetExecutive->GetCreateExtendedSurface(pSurf2,sOtherExtDomain,pOtherExtSurf));
        }
    }
  else // when using the RadialEdgeuse UVTrimCurve
    {
      if ( pOffsetEdgeuse != NULL )
        {
          SmSurface *pSurf = pOffsetEdgeuse->GetFace()->GetSurface();
          SER( cpOffsetExecutive->GetCreateExtendedSurface( pSurf, sOtherExtDomain, pOtherExtSurf ));
        }

      if ( pOtherOffsetEdgeuse != NULL )
        {
          SmSurface *pSurf2 = pOtherOffsetEdgeuse->GetFace()->GetSurface();
          SER( cpOffsetExecutive->GetCreateExtendedSurface( pSurf2, sExtDomain, pExtSurf ));
        }
    }
  
  // arrive here when
  // pUVCurveExtended = pUVCurve extended to new Min and Max parameters
  //                    pUVCurve = most isoParameter UVTrimCurve of pEdgeuse and pRadialEdgeuse
  // pExtSurf         = bUseRadUV ? pOtherOffsetEdgeuse->Surf extension : pOffsetEdgesue->Surf extension
  // pExtDomein       = Domain of pExtSurf
  // pOtherExtSurf    = bUseRadUV ? pOffsetEdgeuse->Surf extension : pOtherOffsetEdgesue->Surf extension 
  //                    or NULL if Edge is Lamina
  // pOtherExtDomain  = Domain of pOtherExtSurf

  // Trim extended UVTrimCurve to pExtDomain
  SmExtent1d sTrimExtended;
  SER(SmSurface::FindUVCurveIvlInDomain(*pUVCurveExtended,
                                         sExtIvl,
                                         sIvl.Evaluate(0.5),
                                         sExtDomain,
                                         sTrimExtended));
  SER(pUVCurveExtended->Trim(sTrimExtended));  // may snap sIvl by tol to existing knots

  // refresh sExtIvl to trimmed Ivl
  sExtIvl = pUVCurveExtended->GetNaturalInterval();
  
  // Set pLiftedCurve = pUVCurveExtended projected through pExtSurf
  SmBSplineCurve *pLiftedCurve = NULL ;
  double dDistToSurf = 0.0;
  SER(pExtSurf->LiftCurve(crContext,                                // in : context for new object construction
                          sExtDomain,                               // in : this surface limit
                         *pUVCurveExtended,                         // in : 2d Parameter curve defined in surface parameter space to lift
                          sExtIvl,                                  // in : curve segment to lift
                          pEdgeuse->GetEdge()->GetTolerance()/3.0,  // in : max allowed distance between output curve and Surface
                          dDistToSurf,                              // out: max dist from output curve to surface
                          pLiftedCurve));                           // out: 3d Curve = Surface(crUVCurveToLift(crInterval))
                                                                    // in : default:[FALSE]= check Surface for C0 discontinuities -
                                                                    //                       break lifted curve at each such point
  SmObjDelete sCleanLifted(pLiftedCurve);                          
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      pLiftedCurve->Dump();
      pUVCurveExtended->Dump();

      smgfx_Erase();
      pLiftedCurve->Draw();     sm_GraphicsLoop();
      pExtSurf->DrawUV(10,10);  sm_GraphicsLoop();
      pUVCurveExtended->Draw(); sm_GraphicsLoop();
      if ( pOtherExtSurf ) { pOtherExtSurf->DrawUV(5,5); }   sm_GraphicsLoop();
      pOffsetEdgeuse->GetEdge()->GetBrep()->Draw();  sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // When the Edgeuse is not lamina - build the 2nd UVTrimCurve on the other Surface
  SmObjDelete sCleanOtherUV;
  if (pOtherExtSurf) 
    {
      SmTArray<SmBSplineCurve*> sUVCurves;
      SmObjsDelete<SmBSplineCurve*> sCleanUVS(&sUVCurves);
      double dDistToOtherSurf = 0.0, dDeviation = 0.0;

      // Drop Lifted Curve onto other extended surface to make 2nd UVTrimCurve
      SER(pOtherExtSurf->DropAndTrimCurve( crContext, sOtherExtDomain,
                                          *pLiftedCurve,
                                           pLiftedCurve->GetNaturalInterval(),
                                           pOffsetEdgeuse->GetEdge()->GetTolerance()/3.0,
                                           dDistToOtherSurf,
                                           dDeviation,
                                           sUVCurves, 
                                           TRUE, TRUE ));
//        SmBSplineCurve *pUVCurve2Found = NULL;
      ULONG lClosestIdx = 0;

      // When Lifted Curve dropped to OtherExtSurf
      if (sUVCurves.GetSize() != 0) 
        {
          // When LiftedCurve dropped in more than 1 piece to OtherExtSurf
          if (sUVCurves.GetSize() > 1) 
            {
              // Find the UVTrimCurve closest to the original - assume curve dropped to a seam
              double dMinDist = SM_BIG_DOUBLE;
              for (ULONG j=0; j<sUVCurves.GetSize(); j++) 
                {
                  SmBSplineCurve *pTest = sUVCurves[j];
                  SER(pTest->GlobalPointSolve(pTest->GetNaturalInterval(),SM_SO_MINIMIZE,
                                              sOtherMid,SM_EFF_ZERO_SQRT,
                                              NULL,NULL,
                                              SM_SR_SINGLE,
                                              sSolutions2));
                  SmSolution &rSol2 = sSolutions2[0];

                  // save the UVTrimCurve with the shortest drop
                  if (rSol2.m_vStart.m_dSolutionValue < dMinDist) 
                    {
                      lClosestIdx = j;
                      dMinDist = rSol2.m_vStart.m_dSolutionValue;
                    }
                } // end iter every DropCurve candidate - looking for best to use
            } // end LiftedCurve dropped in more than 1 piece check

          // Set pOtherUVCurve to best LiftedCurve dropped to OtherExtSurf UVTrimCurve
          pOtherUVCurve = sUVCurves[lClosestIdx];
          sCleanOtherUV.SetObj(pOtherUVCurve);

          // When using the RadialEdgeuse - swap the extended UVTrimCurves
          if (bUseRadUV) 
            {  // Swap them back
              pOtherUVCurve    = pUVCurveExtended;
              pUVCurveExtended = sUVCurves[lClosestIdx];
            }

          // clear the pointer
          sUVCurves[lClosestIdx] = NULL;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
          if (bDebugMe4) 
            {
              pOtherUVCurve->Dump();
              pUVCurveExtended->Dump();

              smgfx_Erase();
              pOtherUVCurve->Draw(); sm_GraphicsLoop();
              pUVCurveExtended->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end LiftedCurve dropped to OtherExtSurf check
      else // LiftedCurve failed to Drop to OtherExtSurf 
        {  // Unable to do dropping - just use original UV and copy it
          // Temp workaround to appease Linux gcc compiler
          // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
          // pOtherUVCurve->Copy(crContext,(SmCurve*&)pOtherUVCurve);
          pOtherUVCurve->Copy(crContext, pOtherUVCurve);
          sCleanOtherUV.SetObj(pOtherUVCurve);
          
          // When using the RadialEdgeuse - swap the extended UVTrimCurves
          if (bUseRadUV) 
            {  // Swap them back
              SmBSplineCurve *pTmp = pOtherUVCurve;
              pOtherUVCurve        = pUVCurveExtended;
              pUVCurveExtended     = pTmp;
            }
        } // end LiftedCurve failed to drop to OtherExtSurf branch
    } // end pOtherExtSurf existence check - working with a nonLamina edge (assumes manifold edge)
  
  // Now that we have created one or two UV curves and a 3D curve find trimming
  // interval and if we find a good one replace the geometry.
  SmBoolean bGoodEdgeCurve;
  SmExtent1d sTrimIvl;
  SER(TrimExtendedEdge(crContext,             // in : context for new object construction                     
                       cpOffsetExecutive,     // in : offset operation context data                           
                       pEdgeuse,              // in : 1st target edgeuse                                      
                       pOtherEdgeuse,         // in : 2nd target edgeuse                                      
                       pOffsetEdgeuse,        // in : Offset of 1st target edgeuse                            
                       pOtherOffsetEdgeuse,   // in : Offset of 2nd target edgeuse                            
                       pLiftedCurve,          // in : XSectCurve between edgeuse->Face->ExtendedSurfaces      
                       pUVCurveExtended,      // in : UVTrimCurve on 1st target edgeuse->Face->ExtendedSurface
                       pOtherUVCurve,         // in : UVTrimCurve on 2nd target edgeuse->Face->ExtendedSurface
                       bGoodEdgeCurve,        // out: TRUE = TrimPoints were found for the input 3DCurve      
                       sTrimIvl,              // out: TrimInterval for the 3DCurve 
                       eMarkType));           // in : uses without increment eMarkType value                         
                                         
  // when TrimExtendedEdge() call worked
  if (bGoodEdgeCurve) 
    {
      // Trim the 3 curves to the common interval
      sCleanLifted.Clear();
      sCleanOtherUV.Clear();
      sClean2.Clear();
      SER(pLiftedCurve->Trim(sTrimIvl));      // may snap sIvl by tol to existing knots
      SER(pUVCurveExtended->Trim(sTrimIvl));  // may snap sIvl by tol to existing knots
      if(pOtherUVCurve) { SER(pOtherUVCurve->Trim(sTrimIvl)); }  // may snap sIvl by tol to existing knots

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          pLiftedCurve->Dump();
          pUVCurveExtended->Dump();

          smgfx_Erase();
          pLiftedCurve->Draw(); sm_GraphicsLoop();
          pUVCurveExtended->Draw(); sm_GraphicsLoop();
          pOffsetEdgeuse->GetEdge()->GetBrep()->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      SmCurve *pLiftedRef  = pLiftedCurve ; 
      SmCurve *pLiftedCopy = NULL ;
      pLiftedCurve->Copy(crContext, pLiftedCopy);
      SmObjDelete sCleanLiftedCopy( pLiftedCopy );

      SmBoolean bDelete3DCurve = FALSE;
      SmBoolean bDeleteUVCurve = FALSE;
      
      // change the pOffsetEdgeuse geometry
      SmStatus Err = ReplaceEdgeCurves(pOffsetEdgeuse,    // in : when rp3DCurve is a new shape, 
                                               //           - assigned rp3DCurve and rpUVCurve deleting any 
                                               //               preExisting Curve and UVTrimCurves.
                                               //      else - not changed.
                            pLiftedRef,        // i/o: candidate 3dCurve for Edgeuse,     deleted and set to NULL when not stored in pEdgeuse  
                            pUVCurveExtended,  // i/o: candidate UVTrimCurve for Edgeuse, deleted and set to NULL when not stored in pEdgeuse
                            NULL,              // in : optional Edgeuse used to compute orientation of rp3DCurve to Edgeuse, NULL to ignore
                            bDelete3DCurve,
                            bDeleteUVCurve);

      if (bDelete3DCurve)
      {
          delete pLiftedRef;
          pLiftedRef = NULL;
      }

      if (bDeleteUVCurve)
      {
          delete pUVCurveExtended;
          pUVCurveExtended = NULL;
      }

      if (Err != SM_SUCCESS)
      {
          SER(Err);
      }

          
      // when appropriate - change the pOtherOffsetEdgeuse geometry
      if (pOtherOffsetEdgeuse) 
        { // Not Lamina
          if (!pOtherUVCurve) 
            { SER(SM_ERR); }
          // GWC:BUG?? These two Edgeuses point to the same edge - these two calls should
          //           have the same 3DCurve
          sCleanLiftedCopy.Clear();

          bDelete3DCurve = FALSE;
          bDeleteUVCurve = FALSE;

          SmStatus ErrOther = ReplaceEdgeCurves(pOtherOffsetEdgeuse, // in : when rp3DCurve is a new shape, 
                                                                //           - assigned rp3DCurve and rpUVCurve deleting any 
                                                                //               preExisting Curve and UVTrimCurves.
                                                                //      else - not changed.
                                           pLiftedCopy,         // i/o: candidate 3dCurve for Edgeuse,     deleted and set to NULL when not stored in pEdgeuse 
                                           pOtherUVCurve,       // i/o: candidate UVTrimCurve for Edgeuse, deleted and set to NULL when not stored in pEdgeuse 
                                           NULL,                // in : optional Edgeuse used to compute orientation of rp3DCurve to Edgeuse, NULL to ignore 
                                           bDelete3DCurve,
                                           bDeleteUVCurve);

          if (bDelete3DCurve)
          {
              delete pLiftedCopy;
              pLiftedCopy = NULL;
          }
         
          if (bDeleteUVCurve)
          {
              delete pOtherUVCurve;
              pOtherUVCurve = NULL;
          }
         
          if (ErrOther != SM_SUCCESS)
          {
                SER(ErrOther);
          }
        } // end pOtherOffsetEdgeuse existence check
    } // end TrimExtendedEdge() call worked branch
  else // TrimExtendedEdge() call did not work
    {
      SER(SM_ERR);
    }
  
  // all done
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::ExtendTrimEdges

/*******************************************************************//**
PURPOSE:  Mate a couple of edges by extension and intersection.

NOTES: 
***********************************************************************/
SmStatus SmOffsetGeometryCreation::MateEdges
  (const SmContext   & crContext,              // in : context for new object construction
   SmOffsetExecutive * cpOffsetExecutive,      // in : offset executive managing this offset
   const SmEdgeuse   * pEdgeuse,               // in : edgeuse from original brep
   const SmEdgeuse   * pOtherEdgeuse,          // in : radial mate of edgeuse from original brep
   SmEdgeuse         * pOffsetEdgeuse,         // in : edgeuse in offset brep mapped to OriginalEdgeuse
   SmEdgeuse         * pOtherOffsetEdgeuse,    // in : edgeuse in offset brep mapped to OtherOriginalEdgeuse
   SmMarkType          eMarkType)              // in : uses without incrementing eMarkType value
{
  // These can't be Null:
  NER( pEdgeuse );
  NER( pOtherEdgeuse );
  NER( pOffsetEdgeuse );
  NER( pOtherOffsetEdgeuse );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(2,4, 0,1,1) ; pEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,1) ; pOtherEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,4, 0,0,0) ; pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,1,0) ; pOtherOffsetEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1) ; pOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; pOtherOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  
  // Extend Surfaces of faces connected to current edge.
  SmSurface * pExtSurface1, * pExtSurface2;
  SmExtent2d  sDomain1,        sDomain2;
  SER(cpOffsetExecutive->GetCreateExtendedSurface(pOffsetEdgeuse->GetFace()->GetSurface(),
                                                  sDomain1, 
                                                  pExtSurface1));
  SER(cpOffsetExecutive->GetCreateExtendedSurface(pOtherOffsetEdgeuse->GetFace()->GetSurface(),
                                                  sDomain2, 
                                                  pExtSurface2));
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
       smgfx_Erase();
       smgfx_SetLook(1,2, 0,0,1) ; if(pExtSurface1) pExtSurface1->DrawUV(3,3); sm_GraphicsLoop();
       smgfx_SetLook(1,2, 0,1,0) ; if(pExtSurface2) pExtSurface2->DrawUV(4,4); sm_GraphicsLoop(); 
       sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  
  // locals for surf/surf intersection 
  ULONG ii ;
  SmApproxTol3d sSSITol             = pEdgeuse->GetEdge()->GetBrep()->GetTolerance()/4.0;
  double        d20DegRad           = SM_DEG2RAD(20.0) ;
  SmBoolean     bUseSurfaceEdges[2] = { TRUE, TRUE } ;

  // local arrays for Temp XSectCurve results - XSectCurves that get used get taken out of these arrays
  SmTArray<SmCurve*> s3DCurves;          SmObjsDelete<SmCurve*> sClean3D(&s3DCurves);
  SmTArray<SmCurve*> sSurface1UVCurves ; SmObjsDelete<SmCurve*> sCleanS1(&sSurface1UVCurves);
  SmTArray<SmCurve*> sSurface2UVCurves;  SmObjsDelete<SmCurve*> sCleanS2(&sSurface2UVCurves);

  // Intersect extended surfaces 
#ifndef SM_VALIDATE_INTERSECTORS
  SER(pExtSurface1->GlobalSurfaceIntersect(crContext,
                                           sDomain1,
                                           *pExtSurface2,sDomain2, 
                                           bUseSurfaceEdges, 
                                           &sSSITol,
                                           &d20DegRad, 
                                           &s3DCurves, 
                                           &sSurface1UVCurves, 
                                           &sSurface2UVCurves, 
                                           NULL, NULL));
#else // no SM_VALIDATE_INTERSECTORS
  SER(pExtSurface1->GlobalSurfaceIntersectAndValidate(crContext,
                                                      sDomain1,
                                                      *pExtSurface2, 
                                                      sDomain2, 
                                                      bUseSurfaceEdges, 
                                                      &sSSITol,
                                                      &d20DegRad, 
                                                      &s3DCurves, 
                                                      &sSurface1UVCurves, 
                                                      &sSurface2UVCurves, 
                                                      NULL, NULL));
#endif // no SM_VALIDATE_INTERSECTORS
  
  // Now see if we have an intersection curve that has proper trimming

  // locals for
  SmCurve        * p3DCurveFound  = NULL;
  SmBSplineCurve * pUVCurve1Found = NULL;
  SmBSplineCurve * pUVCurve2Found = NULL;
  double           dMinDist       = SM_BIG_DOUBLE;
  ULONG            lMinCurve      = 99999;
  SmExtent1d       sTrimIvl;
  
  // for every intersection solution
  for(ii=0;ii<s3DCurves.GetSize();ii++) 
    {
      SmBoolean bGoodEdgeCurve;

#ifdef SM_DEBUG_CODE
      // Draw faces(blue,green), extended surfaces, edgeuses, and xSectCurve(red)
      if (bDebugMe) 
        {
          s3DCurves[ii]->Dump();
          
          smgfx_Erase();
          smgfx_SetLook(2,4, 0,1,1) ; pEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,1) ; pOtherEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 0,0,0) ; pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,1,0) ; pOtherOffsetEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; pExtSurface1->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; pExtSurface2->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; pOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0) ; pOtherOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 1,0,0) ; s3DCurves[ii]->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // skip degenerate curve solutions
      if(s3DCurves[ii]->IsDegenerate())
        { continue ; }

      // get iith XSect 3D and UVTrim Curves
      SmCurve        * p3DCurve  = s3DCurves[ii];
      SmBSplineCurve * pUVCurve1 = SM_CAST_PTR(SmBSplineCurve, sSurface1UVCurves[ii]);
      SmBSplineCurve * pUVCurve2 = SM_CAST_PTR(SmBSplineCurve, sSurface2UVCurves[ii]);
      NER(pUVCurve1); // if UVTrimCurves are missing - we could drop curves rather than
      NER(pUVCurve2); //    signalling an error
      
      // Trim the extended Surf/Surf intersection
      SmExtent1d sMyTrimIvl;
      SER(TrimExtendedEdge(crContext,            // in : context for new object construction 
                           cpOffsetExecutive,    // in : offset operation context data
                           pEdgeuse,             // in : 1st target edgeuse
                           pOtherEdgeuse,        // in : 2nd target edgeuse
                           pOffsetEdgeuse,       // in : Offset of 1st target edgeuse
                           pOtherOffsetEdgeuse,  // in : Offset of 2nd target edgeuse
                           p3DCurve,             // in : XSectCurve between edgeuse->Face->ExtendedSurfaces
                           pUVCurve1,            // in : UVTrimCurve on 1st target edgeuse->Face->ExtendedSurface
                           pUVCurve2,            // in : UVTrimCurve on 2nd target edgeuse->Face->ExtendedSurface
                           bGoodEdgeCurve,       // out: TRUE = TrimPoints were found for the input 3DCurve
                           sMyTrimIvl,           // out: TrimInterval for the 3DCurve
                           eMarkType ));         // in : uses without increment eMarkType value

      // when intersection was successfully trimmed
      if (bGoodEdgeCurve) 
        {
          // when there is more than 1 intersection curve to check
          if (s3DCurves.GetSize() > 1) 
            {
              // save the trimmed XSectCurve that has the smallest change from the original 3DCurve.
              SmPoint3d sMidPnt, sOrigMid;
              SER(p3DCurve->EvaluatePoint(sMyTrimIvl.Evaluate(0.5),sMidPnt));
              SER(((SmEdgeuse *)pEdgeuse)->NormalizedEvaluate(0.5,FALSE,sOrigMid));  // TRUE = UV Eval, FALSE = 3d Eval

              double dDist = sMidPnt.DistanceBetween(sOrigMid);

              if (dDist < dMinDist) { lMinCurve = ii;
                                      dMinDist  = dDist;
                                      sTrimIvl  = sMyTrimIvl;
                                    }
            }
          else // only 1 curve to check branch - use this curve
            {
              sTrimIvl  = sMyTrimIvl;
              lMinCurve = 0;
            }
        }
    } // end iter every extended Surf/Surf intersection

  if (lMinCurve < 99999) 
    {

#ifdef SM_DEBUG_CODE
      // Draw faces(blue,green), extended surfaces, edgeuses, and xSectCurve(red)
      if (bDebugMe) 
        {
          s3DCurves[lMinCurve]->Dump();
          SmPoint3d sStartPoint, sEndPoint ;
          s3DCurves[lMinCurve]->EvaluatePoint(sTrimIvl.GetMin(), sStartPoint) ;
          s3DCurves[lMinCurve]->EvaluatePoint(sTrimIvl.GetMax(), sEndPoint) ;
          
          smgfx_Erase();
          smgfx_SetLook(2,4, 0,1,1) ; pEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,1) ; pOtherEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 0,0,0) ; pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,1,0) ; pOtherOffsetEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; pExtSurface1->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; pExtSurface2->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; pOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0) ; pOtherOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 1,0,0) ; s3DCurves[lMinCurve]->Draw(&sTrimIvl); sm_GraphicsLoop();
          smgfx_SetLook(6,7, 1,0,1) ; sStartPoint.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1,0,1) ; sEndPoint.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // arrive here after finding the XSectCurve result whose midpoint that is closest to the pEdgeuse->Curve->midPoint

      // locals
      p3DCurveFound  = s3DCurves[lMinCurve];
      pUVCurve1Found = SM_CAST_PTR(SmBSplineCurve,sSurface1UVCurves[lMinCurve]);
      pUVCurve2Found = SM_CAST_PTR(SmBSplineCurve,sSurface2UVCurves[lMinCurve]);
      NER(pUVCurve1Found);
      NER(pUVCurve2Found);

      // remove found Curves from their input arrays so that whey won't be deleted when leaving this method's scope
      s3DCurves.RemoveAt(lMinCurve,1);
      sSurface1UVCurves.RemoveAt(lMinCurve,1);
      sSurface2UVCurves.RemoveAt(lMinCurve,1);

      // Trim bound best XSectCurves to sTrimIvl
      SER(p3DCurveFound->Trim(sTrimIvl));    // may snap sIvl by tol to existing knots
      SER(pUVCurve1Found->Trim(sTrimIvl));   // may snap sIvl by tol to existing knots
      SER(pUVCurve2Found->Trim(sTrimIvl));   // may snap sIvl by tol to existing knots

#ifdef SM_DEBUG_CODE
      // Draw faces(blue,green), extended surfaces, edgeuses, and xSectCurve(red)
      if (bDebugMe) 
        {
          p3DCurveFound->Dump();
          SmPoint3d sStartPoint, sEndPoint ;
          p3DCurveFound->EvaluatePoint(sTrimIvl.GetMin(), sStartPoint) ;
          p3DCurveFound->EvaluatePoint(sTrimIvl.GetMax(), sEndPoint) ;
          
          smgfx_Erase();
          smgfx_SetLook(2,4, 0,1,1) ; pEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,1) ; pOtherEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 0,0,0) ; pOffsetEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,1,0) ; pOtherOffsetEdgeuse->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1) ; pExtSurface1->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; pExtSurface2->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; pOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0) ; pOtherOffsetEdgeuse->GetFace()->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 1,0,0) ; p3DCurveFound->Draw(&sTrimIvl); sm_GraphicsLoop();
          smgfx_SetLook(6,7, 1,0,1) ; sStartPoint.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1,0,1) ; sEndPoint.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end lMinCurve existence check

  // Error Exit - no XSectCurve found
  if (!p3DCurveFound) 
    { return SM_ERR; }
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
  if (bDebugMe4) 
    {
      SmBrep *pBrep = pOffsetEdgeuse->GetFace()->GetBrep() ;
      if(pBrep) { pBrep->Dump() ; SM_ASSERT_VALID(pBrep) ; }
      p3DCurveFound->Dump() ;     SM_ASSERT_VALID(p3DCurveFound) ;
      pUVCurve1Found->Dump() ;     SM_ASSERT_VALID(pUVCurve1Found) ;

      smgfx_Erase();
      smgfx_SetLook(3,4, 1,0,0) ; p3DCurveFound->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  
  // Replace Curve of edge and set its new domain.
  SmCurve *p3DCopy;
  p3DCurveFound->Copy(crContext,p3DCopy);

  SmBoolean bDelete3DCurve = FALSE;
  SmBoolean bDeleteUVCurve = FALSE;

  // when appropriate - change the pOffsetEdgeuse geometry
  SmStatus Err1 = ReplaceEdgeCurves(pOffsetEdgeuse,  // in : when rp3DCurve is a new shape, 
                                         //           - assigned rp3DCurve and rpUVCurve deleting any 
                                         //               preExisting Curve and UVTrimCurves.
                                         //      else - not changed.
                        p3DCurveFound,   // i/o: candidate 3dCurve for Edgeuse,     deleted and set to NULL when not stored in pEdgeuse  
                        pUVCurve1Found,  // i/o: candidate UVTrimCurve for Edgeuse, deleted and set to NULL when not stored in pEdgeuse  
                        pEdgeuse,         // in : optional Edgeuse used to compute orientation of rp3DCurve to Edgeuse, NULL to ignore  
                        bDelete3DCurve,
                        bDeleteUVCurve);

  if (bDelete3DCurve)
  {
      delete p3DCurveFound;
      p3DCurveFound = NULL;
  }

  if (bDeleteUVCurve)
  {
      delete pUVCurve1Found;
      pUVCurve1Found = NULL;
  }

  if (Err1 != SM_SUCCESS)
  {
    SER(Err1);
  }

  bDelete3DCurve = FALSE;
  bDeleteUVCurve = FALSE;

  // when appropriate - change the pOtherOffsetEdgeuse geometry 
  // GWC:BUG?? These two Edgeuses point to the same edge - these two calls should
  //           have the same 3DCurve                           
  //     It's ok: At this point they do not (yet) point to the same edge.
  SmStatus Err2 = ReplaceEdgeCurves(pOtherOffsetEdgeuse,  // in : when rp3DCurve is a new shape, 
                                              //           - assigned rp3DCurve and rpUVCurve deleting any 
                                              //               preExisting Curve and UVTrimCurves.
                                              //      else - not changed.
                        p3DCopy,              // i/o: candidate 3dCurve for Edgeuse.
                        pUVCurve2Found,       // i/o: candidate UVTrimCurve for Edgeuse.
                        pOtherEdgeuse,        // in : optional Edgeuse used to compute orientation of rp3DCurve to Edgeuse, NULL to ignore
                        bDelete3DCurve,
                        bDeleteUVCurve);

  if (bDelete3DCurve)
  {
      delete p3DCopy;
      p3DCopy = NULL;
  }

  if (bDeleteUVCurve)
  {
      delete pUVCurve2Found;
      pUVCurve2Found = NULL;
  }

  if (Err2 != SM_SUCCESS)
  {
        SER(Err2);
  }
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe7 = FALSE;
  if (bDebugMe7) 
    {
      ULONG di ;
      SmTArray<SmEdgeuse*> sEdgeuses;
      SmTArray<SmEdgeuse*> sOtherEdgeuses;
      pOffsetEdgeuse->GetFaceuse()->GetEdgeuses(sEdgeuses);
      pOtherOffsetEdgeuse->GetFaceuse()->GetEdgeuses(sOtherEdgeuses);

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0); p3DCurveFound->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pOffsetEdgeuse->GetFace()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pOtherOffsetEdgeuse->GetFace()->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); for(di=0;di<sEdgeuses.GetSize();di++) { if(sEdgeuses[di]) sEdgeuses[di]->Draw(); sm_GraphicsLoop();
                                                                         sm_GraphicsLoop(); 
                                                                       }
      smgfx_SetLook(1,2, 0,0,0); for(di=0;di<sOtherEdgeuses.GetSize();di++) { if(sOtherEdgeuses[di]) sOtherEdgeuses[di]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();                                                      sm_GraphicsLoop();
                                                                            }
    }
#endif // SM_DEBUG_CODE
  
  // all done
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::MateEdges

/*******************************************************************//**
PURPOSE: This method will look for and fill any tangency gaps between
     these two edgeuses.  If it finds them it will return TRUE.

NOTES: 
***********************************************************************/
SmBoolean SmOffsetGeometryCreation::HaveTangentGaps
  (ULONG lNumSamples,
   SmEdgeuse * cpOffsetEdgeuse,
   SmEdgeuse * cpOtherOffsetEdgeuse)
{
    SmEdge *pEdge = cpOffsetEdgeuse->GetEdge();
    double dTol = pEdge->GetTolerance();
    
    for (ULONG i=0; i<lNumSamples+1; i++) 
      {
        double dParam = (i*1.0) / (lNumSamples*1.0);
        SmPoint3d sPnt;
        cpOtherOffsetEdgeuse->NormalizedEvaluate(dParam,FALSE,sPnt);  // TRUE = UV Eval, FALSE = 3d Eval
        SmBoolean bSuccess;
        double dParameter, dDistance;
        pEdge->ClosestPoint(sPnt,bSuccess,dParameter,dDistance);
        if (bSuccess && dDistance > dTol) 
          {
            return TRUE;
          }
      }
    return FALSE;

} // end SmOffsetGeometryCreation::HaveTangentGaps

/*******************************************************************//**
PURPOSE: Create the offset surface corresponding to an edgeuse.

NOTES: 
***********************************************************************/
SmStatus SmOffsetGeometryCreation::EdgeuseOffset
  (const SmContext         & crContext,               // in : context for new object construction
   SmOffsetExecutive       * cpOffsetExecutive,       // in : offset executive managing this offset
   const SmEdgeuse         * cpOriginalEdgeuse,       // in : edgeuse from original brep
   const SmEdgeuse         * cpOtherOriginalEdgeuse,  // in : radial mate of edgeuse from original brep
   SmEdgeuse               * cpOffsetEdgeuse,         // in : edgeuse in offset brep mapped to OriginalEdgeuse
   SmEdgeuse               * cpOtherOffsetEdgeuse,    // in : edgeuse in offset brep mapped to OtherOriginalEdgeuse
   SmSurface              *& rpNewSurface,            // out: 
   SmTArray<SmCurve*>      & r3DTrimmingCurves,       // out: 
   SmTArray<SmOrientType>  & rTrimOrientations,       // out: 
   SmSSIData               & rSelfIntersections,      // out: currently not used - in future to be used
                                                      //      to find self intersections in rpNewSurface
   SmMarkType                eMarkType)               // in : uses without increment eMarkType Value
{
  // init output
  rpNewSurface = NULL;
  rSelfIntersections.m_v3DCurves.ReSet();
  rSelfIntersections.m_vUVCurves1.ReSet();
  rSelfIntersections.m_vUVCurves2.ReSet();
  rSelfIntersections.m_vCurveTypes.ReSet();
  rSelfIntersections.m_vDeviations.ReSet();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      SmFace *pFace1  = cpOriginalEdgeuse ? cpOriginalEdgeuse->GetLoopuse()->GetFaceuse()->GetFace() : NULL ;
      SmFace *pFace2  = cpOtherOriginalEdgeuse ? cpOtherOriginalEdgeuse->GetLoopuse()->GetFaceuse()->GetFace() : NULL ;
      SmFace *pOFace1 = cpOffsetEdgeuse ? cpOffsetEdgeuse->GetLoopuse()->GetFaceuse()->GetFace() : NULL ;
      SmFace *pOFace2 = cpOtherOffsetEdgeuse ? cpOtherOffsetEdgeuse->GetLoopuse()->GetFaceuse()->GetFace() : NULL ;

      SmBrep *pBrep1  = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2  =   pOFace1 ? pOFace1->GetBrep() 
                        : pOFace2 ? pOFace2->GetBrep() : NULL ;
      SM_DUMP_AND_ASSERT_VALID(pBrep1) ; 
      SM_DUMP_AND_ASSERT_VALID(pBrep2) ; 

      smgfx_Erase();
      smgfx_SetLook(1,2, 0, 0, 1); if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0, 1, 0); if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1, 0, 0); if(cpOriginalEdgeuse)      cpOriginalEdgeuse->Draw();      sm_GraphicsLoop();
      smgfx_SetLook(4,5, 1, 0, 1); if(cpOtherOriginalEdgeuse) cpOtherOriginalEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,.5, 0); if(cpOffsetEdgeuse)        cpOffsetEdgeuse->Draw();        sm_GraphicsLoop();
      smgfx_SetLook(6,7, 1, 0,.5); if(cpOtherOffsetEdgeuse)   cpOtherOffsetEdgeuse->Draw();   sm_GraphicsLoop();

      smgfx_SetLook(1,2, 0,1,1); if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0); if(pOFace1) pOFace1->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0); if(pOFace2) pOFace2->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // If we are extending convex edges, extend both offset surfaces to hit each other.
  if (cpOffsetExecutive->GetExtendConvexEdges()) 
    {
      SmBoolean bLamina = cpOriginalEdgeuse->GetEdge()->IsLamina();

      // No, we have to call ExtendTrimEdges() for Lamina Edges.  [cbi B01; B256 ok]
      //if ( bLamina )
      //  { return SM_SUCCESS; } // [B256]

      double    dTanTolDeg = cpOffsetExecutive->GetTangencyTolDegrees();
      SmBoolean bTanSec    = cpOriginalEdgeuse->IsTangentSector(dTanTolDeg);

      if ( bTanSec  ||  bLamina )
        {
          // Extend or Trim Lamina or Tangent Edges
          SmStatus eStat = ExtendTrimEdges(crContext,
                                           cpOffsetExecutive,
                                           cpOriginalEdgeuse, 
                                           cpOtherOriginalEdgeuse, 
                                           cpOffsetEdgeuse, 
                                           cpOtherOffsetEdgeuse,
                                           eMarkType);  // in : uses without incrementing eMarkType value
          if ( eStat != SM_SUCCESS )
            {
              MSG(_T("Problem Extending Lamina/Tangent Edge - Will be Fixed Later"));
            }

          return SM_SUCCESS;
        }

      // Mate a couple of edges by extension and intersection.
      if(SM_SUCCESS != MateEdges(crContext,
                                 cpOffsetExecutive,
                                 cpOriginalEdgeuse, 
                                 cpOtherOriginalEdgeuse, 
                                 cpOffsetEdgeuse, 
                                 cpOtherOffsetEdgeuse,
                                 eMarkType))   // in: uses without incrementing eMarkType value
        {
          MSG(_T("Problem Extending Convex Edge - Will be Fixed Later"));
        }
      return SM_SUCCESS;
    } // end when extending convex edges

  // Make sure sector being offset is convex otherwise just skip it
  if(   cpOriginalEdgeuse->GetRadial() == cpOtherOriginalEdgeuse 
     && !cpOriginalEdgeuse->IsConvexRadialSector(10)) 
    {
      if(   cpOriginalEdgeuse->IsTangentSector(3) 
         && cpOffsetEdgeuse 
         && cpOtherOffsetEdgeuse
         && HaveTangentGaps(10,cpOffsetEdgeuse,cpOtherOffsetEdgeuse)) 
        {
          return SM_ERR; // Figure out how to recover from this outside
        }
      return SM_SUCCESS;
    }

  // Now take care of case where we are insetting and hit the shell face
  // we need to extend and intersect the side face.
  // Note: if cpOtherOriginalEdgeuse was passed in NULL, then there's nothing
  // else we can do in this method.  That will happen for Lamina edges.
  if ( cpOtherOriginalEdgeuse == NULL )
    { return SM_SUCCESS; }

  SmFace *pOrigFace = cpOriginalEdgeuse->GetFace();
  SmFace *pOtherOrigFace = cpOtherOriginalEdgeuse->GetFace();
  SmTArray<SmFace*> sShellFaces;
  cpOffsetExecutive->GetShellFaces(sShellFaces);

  // when Orig or OtherOrig face is a ShellFace (not offset - just copied)
  // and  we are insetting
  ULONG lFound;
  if(   (   sShellFaces.FindElement(pOrigFace,lFound) 
         || sShellFaces.FindElement(pOtherOrigFace,lFound)) 
     && cpOffsetExecutive->GetInset() == TRUE) 
    {
      // when OrigFace is not a shell face and we have an OffsetEdgeuse
      if (   !sShellFaces.FindElement(pOrigFace,lFound)
          && cpOffsetEdgeuse) 
        {
          //
          SmEdge *pOffsetEdge = cpOffsetEdgeuse->GetEdge();
          if (pOffsetEdge->GetBrep()->MateLaminaEdgeToSurface(pOffsetEdge,
              *pOtherOrigFace->GetSurface(),1.5,10.0) != SM_SUCCESS) 
            {
              MSG(_T("Problem Mating Lamina Edge - Will be Fixed Later"));
            }
        }

      // when OtherFace is not a shell face and we have an OtherOffsetEdgeuse
      if (   !sShellFaces.FindElement(pOtherOrigFace,lFound) 
          && cpOtherOffsetEdgeuse) 
        {
          //
          SmEdge *pOtherOffsetEdge = cpOtherOffsetEdgeuse->GetEdge();
          if (pOtherOffsetEdge->GetBrep()->MateLaminaEdgeToSurface(pOtherOffsetEdge,
              *pOrigFace->GetSurface(),1.5,10.0) != SM_SUCCESS) 
                {
              MSG(_T("Problem Mating Lamina Edge- Will be Fixed Later"));
            }
        }

      // all done
      return SM_SUCCESS; // Don't need to build one if it is 
                         // an inset

    } // end inset on shell face check

  // let pCenterLine = temporary copy of OrigEU->Edge->Curve
  SmCurve        *pCopy;
  SmEdge         *pE          = cpOriginalEdgeuse->GetEdge();
  SmBSplineCurve* pCenterLine = SM_CAST_PTR(SmBSplineCurve, pE->GetCurve());
  NER(pCenterLine);
  //Potential solution for CrvOnSurf edge curve (suspect)
  //SmBSplineCurve* pCenterLine = pE->CreateTrimmedNURBSCurve(crContext);
  //NER(pCenterLine);
  SER(pCenterLine->Copy(crContext,pCopy)); NER(pCopy);
  SmObjDelete     sCleanCL(pCopy); 
  pCenterLine = SM_CAST_PTR(SmBSplineCurve,pCopy); NER(pCenterLine);

  // get OrigEU->Edge interval
  SmExtent1d sEdgeIvl = pE->GetInterval();

  // Let RailCurves = 3dCurve Copies of offsetEdge->Curves - trimmed to edge intervals 
  SmBSplineCurve *pRail1Curve = NULL;
  SmBSplineCurve *pRail2Curve = NULL;
  SmObjDelete sCleanRail1;
  SmObjDelete sCleanRail2;
  if (cpOffsetEdgeuse) 
    {
      pRail1Curve = cpOffsetEdgeuse->GetEdge()->CreateTrimmedNURBSCurve(crContext);
      sCleanRail1.SetObj(pRail1Curve);
    }
  if (cpOtherOffsetEdgeuse) 
    {
      pRail2Curve = cpOtherOffsetEdgeuse->GetEdge()->CreateTrimmedNURBSCurve(crContext);
      sCleanRail2.SetObj(pRail2Curve);
    }

  // When not given an OffsetEdgeuse - make a Rail1Curve by approximating OrigEU->UVTrimCurve Offset
  if (!pRail1Curve) 
    {
      // make temporary copy of the OrigEU->UVTrimCurve trimmed to sEdgeIvl
      SmCurve *pUVCurve = cpOriginalEdgeuse->GetUVTrimCurve(); NER(pUVCurve);
      SER(pUVCurve->Copy(crContext,pUVCurve)); NER(pUVCurve);
      SmObjDelete sCleanUV(pUVCurve);
      SER(pUVCurve->Trim(sEdgeIvl)); // may snap sIvl by tol to existing knots

      // make an offset surface from origEU->Face->Surface 
      SmSurface *pSurface = cpOriginalEdgeuse->GetFace()->GetSurface(); NER(pSurface);
      double     dSign    =  (cpOriginalEdgeuse->GetFaceuse()->GetOrientation() != SM_OT_SAME)
                            ? -1.0
                            :  1.0 ;
      SmOffsetSurface sOffset(dSign * m_dOffsetDistance,*pSurface,FALSE);
      sOffset.SetContext(pSurface->GetContext());

      // Make a 3dCurve from sOffsetSurface and pUVCurve
      SmCrvOnSurf sOffCurve(*pUVCurve,sOffset, NULL, 0, NULL);

      // get OtherRailCurve knots (or UVCurve knots if unavailable)
      SmTArray<double> sKnots;
      if (pRail2Curve) { SER(pRail2Curve->GetKnots(sKnots));
                       }
      else             { SER(pUVCurve->GetKnots(sKnots));
                       }

      // let pRail1Curve = approximation of the 3dCurve
      double dAchievedTol;
      SER(sOffCurve.ApproximateCurve(crContext,sKnots,
                                     m_dThisApproxTol3d,
                                     dAchievedTol, pRail1Curve,
                                     TRUE,     // in : bOptCreateAnalytics      
                                     FALSE,    // in : bOptMatchParameterization
                                     FALSE)) ; // in : bJustCopyBSplines        

      NER(pRail1Curve);
      sCleanRail1.SetObj(pRail1Curve);
    
    } // end not given an OffsetEdgeuse check

  // When not given an OtherOffsetEdgeuse - make a Rail2Curve by approximating OtherOrigEU->UVTrimCurve Offset
  if (!pRail2Curve) 
    {
      // make temporary copy of the OtherOrigEU->UVTrimCurve trimmed to sEdgeIvl
      SmCurve *pUVCurve = cpOtherOriginalEdgeuse->GetUVTrimCurve(); NER(pUVCurve);
      SER(pUVCurve->Copy(crContext,pUVCurve)); NER(pUVCurve);
      SmObjDelete sCleanUV(pUVCurve);
      SER(pUVCurve->Trim(sEdgeIvl));

      // make an offset surface from OtherOrigEU->Face->Surface 
      SmSurface *pSurface = cpOtherOriginalEdgeuse->GetFace()->GetSurface(); NER(pSurface);
      double dSign =  (cpOtherOriginalEdgeuse->GetFaceuse()->GetOrientation() != SM_OT_SAME)
                     ? -1.0
                     :  1.0;
      SmOffsetSurface sOffset(dSign * m_dOffsetDistance,*pSurface,FALSE);
      sOffset.SetContext(pSurface->GetContext());

      // Make a 3dCurve from sOffsetSurface and pUVCurve
      SmCrvOnSurf sOffCurve(*pUVCurve,sOffset, NULL, 0, NULL);

      // get RailCurve knots 
      SmTArray<double> sKnots;
      SER(pRail1Curve->GetKnots(sKnots));
      
      // let pRail2Curve = approximation of the 3dCurve
      double dAchievedTol;
      SER(sOffCurve.ApproximateCurve(crContext, sKnots,
                                     m_dThisApproxTol3d,
                                     dAchievedTol, pRail2Curve,
                                     TRUE,     // in : bOptCreateAnalytics      
                                     FALSE,    // in : bOptMatchParameterization
                                     FALSE)) ; // in : bJustCopyBSplines        

      NER(pRail2Curve);
      sCleanRail2.SetObj(pRail2Curve);
    
    } // end not given an OtherOffsetEdgeuse check

  // Make sure intervals of rail curves are the same
  SM_ASSERT(   pRail1Curve != NULL
            && pRail2Curve != NULL) ;
  SmExtent1d sIvl1 = pRail1Curve->GetNaturalInterval();
  SmExtent1d sIvl2 = pRail2Curve->GetNaturalInterval();
  if(   !sIvl1.IsValueOnBoundary(sIvl2.GetMin())
     || !sIvl1.IsValueOnBoundary(sIvl2.GetMax()) ) 
    {
      SER(SM_ERR); // Non syncronized edgeuses
    }

  // get Rail1Curve/Rail2Curve max distance
  double dMaxDist;
  SER(pRail1Curve->CurveMaxDistanceBetween(pRail1Curve->GetNaturalInterval(),
      *pRail2Curve,pRail2Curve->GetNaturalInterval().GetMin(),pRail2Curve->GetNaturalInterval().GetMax(),
      30,NULL,dMaxDist));

  // When railCurves are within 10*Tolerance of each other
  if (dMaxDist < pE->GetTolerance() * 10.0) 
    {
      // don't build a capSurface - just increase railCurve tolerances and return
#ifdef SM_USE_OLDTOL      
      SM_OLDTOL_LINE if (cpOffsetEdgeuse)      cpOffsetEdgeuse->GetEdge()->SetTolerance(dMaxDist*2.0);
      SM_OLDTOL_LINE if (cpOtherOffsetEdgeuse) cpOtherOffsetEdgeuse->GetEdge()->SetTolerance(dMaxDist*2.0);
#endif // SM_USE_OLDTOL
      return SM_SUCCESS;
    }

  // regularize the railCurve knot vectors by inserting knots into large spans 
  if (pRail1Curve->GetDegree() == 3) { SER(pRail1Curve->DampenKnotSpacing(2.0,3));
                                     }
  if (pRail2Curve->GetDegree() == 3) { SER(pRail2Curve->DampenKnotSpacing(2.0,3));
                                     }

  // The offset edgeuses must be syncronized but may only be a subset of
  // the original edge.  Therefore we do a trim just to make sure.

  // get Rail1Curve StartPoint
  SmExtent1d sRail1Ivl = pRail1Curve->GetNaturalInterval();
  SmPoint3d sPnt1 ;
  SER(pRail1Curve->EvaluatePoint(sRail1Ivl.GetMin(),sPnt1));

  // get CenterLine StartPoint
  SmExtent1d sCenterIvl = pCenterLine->GetNaturalInterval();
  SmPoint3d sPnt2 ;
  SER(pCenterLine->EvaluatePoint(sCenterIvl.GetMin(),sPnt2));

  // When StartPoints Gap != OffsetDistance to within tolerance
  if (smos_Fabs(sPnt1.DistanceBetween(sPnt2)-m_dOffsetDistance) > m_dThisApproxTol3d) 
    {
      // drop Rail1Curve StartPoint to CenterLine
      SmPoint3d sPV[2];
      SER(pRail1Curve->Evaluate(sRail1Ivl.GetMin(),1,TRUE,sPV));
      double dParam, dDist;
      SmBoolean bSuccess;
      SER(pCenterLine->DropPoint(sCenterIvl,             // in : target curve allowed domain
                                 sPV[0],                 // in : Point to drop to curve
                                 &sPV[1],                // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                         //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                         //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                 1.1*m_dOffsetDistance,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                         //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                         //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                         //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                 NULL,                   // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                 bSuccess,               // out: TRUE = found a drop point
                                 dParam,                 // out: found drop curve param
                                 dDist,                  // out: found drop distance
                                 SM_SO_INTERSECT)) ;     // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                         //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                         //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                         //      default:[SM_SO_MINIMIZE] to preserve original behavior
      // when Rail1Curve StartPoint dropped to CenterLine
      if (bSuccess) 
        {
          // Trim CenterLine to Start dropPoint
          sCenterIvl.SetMinMax(dParam,sCenterIvl.GetMax());
          pCenterLine->Trim(sCenterIvl);   // may snap sIvl by tol to existing knots
        }
    } // end Rail1/CenterLine StartPoint gapSize check

  // get Rail1 and CenterLine EndPoints
  SER(pRail1Curve->EvaluatePoint(sRail1Ivl.GetMax(),sPnt1));
  SER(pCenterLine->EvaluatePoint(sCenterIvl.GetMax(),sPnt2));

  // When EndPoints Gap != OffsetDistance to within tolerance
  if (smos_Fabs(sPnt1.DistanceBetween(sPnt2)-m_dOffsetDistance) > m_dThisApproxTol3d) 
    {
      // drop Rail1Curve EndPoint to CenterLine
      SmPoint3d sPV[2];
      SER(pRail1Curve->Evaluate(sRail1Ivl.GetMax(),1,TRUE,sPV));
      sPV[1] *= -1;   // "inward-pointing vector".

      double dParam, dDist;
      SmBoolean bSuccess;

      // Call DropPoint() with INTERSECT, and pass a maximum allowable distance.
      double dMaxDropDist = 1.1*m_dOffsetDistance;

      SER(pCenterLine->DropPoint(sCenterIvl, // in : target curve allowed domain
                                 sPV[0],     // in : Point to drop to curve
                                 &sPV[1],    // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                             //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                             //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                 dMaxDropDist, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                             //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                             //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                             //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                 NULL,       // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                 bSuccess,   // out: TRUE = found a drop point
                                 dParam,     // out: found drop curve param
                                 dDist,      // out: found drop distance
                                 SM_SO_INTERSECT )); // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                             //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                             //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                             //      default:[SM_SO_MINIMIZE] to preserve original behavior
      // when Rail1Curve EndPoint dropped to CenterLine
      if (bSuccess) 
        {
          // Trim CenterLine to End dropPoint
          sCenterIvl.SetMinMax(sCenterIvl.GetMin(),dParam);
          pCenterLine->Trim(sCenterIvl);  // may snap sIvl by tol to existing knots
        }
    } // end Rail1/CenterLine EndPoint gapSize check

  // when either railCurve is degree 3 or higher
  if(   pRail1Curve->GetDegree() > 2
     || pRail2Curve->GetDegree() > 2) 
    {
      // make rail1/rail2/centerLine curves compatible i.e. share a common knotVector
      SmBSplineCurve * apData[3];
      SmTArray<SmBSplineCurve*> sCurves(3,apData);
      sCurves.Add(pRail1Curve);
      sCurves.Add(pRail2Curve);
      sCurves.Add(pCenterLine);
      SER(SmBSplineCurve::MakeCurvesCompatible(sCurves,SM_EFF_ZERO_PARAM));
    }

  // railCurve's have been manicured - make a surface between them
  SmBSplineSurface *pNewSurface = NULL;

  // First handle case of one face being shelling face and just 
  // create a linear blend between the rails or extend the surface.
  if(   sShellFaces.FindElement(pOrigFace,lFound) 
     || sShellFaces.FindElement(pOtherOrigFace,lFound) ) 
    {
      // Now that we have the curves create the geometry for the surface.
      if (SmBSplineSurface::CreateTrimmedPlaneFromRuled(crContext,
                                                        *pRail1Curve, *pRail2Curve,
                                                        r3DTrimmingCurves, rTrimOrientations,
                                                        pNewSurface) != SM_SUCCESS) 
        {
          SER(SmBSplineSurface::CreateRuledSurface(crContext, *pRail1Curve, *pRail2Curve,
                                                   SM_SP_U, pNewSurface));
        }
    } // end one face is a shell face check

  // First do a quick check for the self intersecting torus situation.
  SmAxis2Placement sRefFrame;
  double dMajorRadius, dStartAngle, dEndAngle;
  if (   pNewSurface == NULL
      && pCenterLine->IsArc(10,m_dThisApproxTol3d,sRefFrame,dMajorRadius,dStartAngle,dEndAngle)) 
    {
      SmAxis2Placement sRefFrame2;
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          pCenterLine->Dump();
          pRail1Curve->Dump();
          pRail2Curve->Dump();
          double dW = 1 ;
          smgfx_SetLook(dW,2, 0,0,0) ; pCenterLine->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(dW,2, 0,0,0) ; pRail1Curve->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(dW,2, 0,0,0) ; pRail2Curve->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();

        }
#endif // SM_DEBUG_CODE

      double dRadiusR1, dRadiusR2, dStartAngle2, dEndAng2;
      if (   pRail1Curve->IsArc(10,m_dThisApproxTol3d,sRefFrame2,dRadiusR1,dStartAngle2,dEndAng2)
          && pRail2Curve->IsArc(10,m_dThisApproxTol3d,sRefFrame2,dRadiusR2,dStartAngle2,dEndAng2) ) 
        {
          // Have a torus situation
          SmPoint3d sCLPoint, sR1Point, sR2Point;
          SER(pCenterLine->EvaluatePoint(pCenterLine->GetNaturalInterval().Evaluate(0.0),sCLPoint));
          SER(pRail1Curve->EvaluatePoint(pRail1Curve->GetNaturalInterval().Evaluate(0.0),sR1Point));
          SER(pRail2Curve->EvaluatePoint(pRail2Curve->GetNaturalInterval().Evaluate(0.0),sR2Point));
          double dRad1 = sCLPoint.DistanceBetween(sR1Point);
          double dRad2 = sCLPoint.DistanceBetween(sR2Point);
          if (smos_Fabs(dRad1-dRad2) < SM_EFF_ZERO * (1.0 + sR1Point.GetMaxDimension())) 
            {
              double dMinorRadius = (dRad1+dRad2)/2.0;
              
              // Swap and start arc so that it corresponds to the curve which is
              // the same radius as the center line.
              if (smos_Fabs(dMajorRadius-dRadiusR2) < smos_Fabs(dMajorRadius-dRadiusR1)) 
                { 
                  sR1Point.Swap(sR2Point); 
                }
              SmBSplineCurve *pArc;
              SER(SmBSplineCurve::CreateArcFromPoints(crContext,3,sCLPoint,sR1Point,sR2Point,
                  SM_CO_QUADRATIC,pArc));
              SmObjDelete sCleanArc(pArc);
              
              // Make a ray going in the right direction
              SmVector3d sRayVector = sRefFrame.GetZAxis();
              SmPoint3d sRayPoint = sRefFrame.GetOriginRef() - (dMinorRadius+dMajorRadius)*sRayVector;
              
              if (dMajorRadius - dMinorRadius < -SM_EFF_ZERO) 
                {
                  // Self intersecting torus case - create a new point and a new circle
                  SmSolution sSData[16];
                  SmSolutionArray sSolutions(16,sSData);
                  SmExtent1d sArcIvl = pArc->GetNaturalInterval();
                  SER(pArc->GlobalPointSolve( sArcIvl,SM_SO_RAYFIRE,sRayPoint,
                      m_dThisApproxTol3d,NULL,&sRayVector,SM_SR_SINGLE,sSolutions));
                  if (sSolutions.GetSize() > 0) 
                    {
                      double dT = sSolutions[0].m_vStart[0];
                      if ( smos_Fabs( dT - sArcIvl.GetMin() ) > SM_EFF_ZERO_SQRT )
                        {
                          sArcIvl.SetMinMax( sArcIvl.GetMin(),dT );
                          SER(pArc->Trim( sArcIvl ));
                        }
                    }
                }
              // Now Create torus
              SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,pArc,sRefFrame.GetOriginRef(),
                  sRefFrame.GetZAxis(),dEndAngle-dStartAngle,pNewSurface));
            }
        }
    } // end quick check for self intersecting torus case

  // Check for case of cone or cylinder
  SmPoint3d sLinePt;
  SmVector3d sLineVec;
  if(   pNewSurface == NULL 
     && pCenterLine->IsLine(10,m_dThisApproxTol3d,sLinePt,sLineVec)) 
    {
      SmPoint3d sLinePt2, sLinePt3;
      SmVector3d sLineVec2, sLineVec3;
      if (pRail1Curve->IsLine(10,m_dThisApproxTol3d,sLinePt2,sLineVec2) &&
          pRail2Curve->IsLine(10,m_dThisApproxTol3d,sLinePt3,sLineVec3) ) 
        {
          SmPoint3d sCLPoint, sR1Point, sR2Point;
          SER(pCenterLine->EvaluatePoint(pCenterLine->GetNaturalInterval().Evaluate(0.0),sCLPoint));
          SER(pRail1Curve->EvaluatePoint(pRail1Curve->GetNaturalInterval().Evaluate(0.0),sR1Point));
          SER(pRail2Curve->EvaluatePoint(pRail2Curve->GetNaturalInterval().Evaluate(0.0),sR2Point));
          double dRad1 = sCLPoint.DistanceBetween(sR1Point);
          double dRad2 = sCLPoint.DistanceBetween(sR2Point);
          if (smos_Fabs(dRad1-dRad2) < SM_EFF_ZERO * (1.0 + sR1Point.GetMaxDimension())) 
            {
              SmBSplineCurve *pArc;
              SER(SmBSplineCurve::CreateArcFromPoints(crContext,3,sCLPoint,sR2Point,sR1Point,
                  SM_CO_QUADRATIC,pArc));
              SmObjDelete sCleanArc2(pArc);
              SmAxis2Placement sFrame;
              double dRadius, dStartAng, dEndAng;
//                double dHeight = sLineVec.Length();
              if (pArc->IsArc(10,m_dThisApproxTol3d,sFrame,dRadius,dStartAng,dEndAng)) 
                {
                  SER(SmBSplineSurface::CreateLinearSweep(crContext,*pArc,sLineVec,pNewSurface));
                }
            }
        }
    } // end cone or cylinder case

  // when case is not already handled - build a surface to fill the curves with circular cross section
  if (!pNewSurface) 
    {
      // Now that we have the curves create the geometry for the surface.
      SmCircularCrossSectionFSG sFSG(FALSE);
      SER(sFSG.CreateSurfaceFromCurves(crContext,
                                      *pCenterLine, *pRail1Curve, *pRail2Curve,
                                      m_dThisApproxTol3d,
                                      FALSE, NULL, NULL, NULL, NULL,
                                      pNewSurface));
    }

  // when working with analytics - turn NewSurface into Analytic when possible
  if (m_bMakeAnalytics) { SmObjDelete sClean(pNewSurface);
                          // Copy pNewSurface, when possible as an analytic surface
                          SER(pNewSurface->CopyAndAddAnalytics(crContext,rpNewSurface));
                        }
  else                  { rpNewSurface = pNewSurface;
                        }

#if 0
  double dAngleTol = 20.0 * SM_PI / 180.0;
  SmBoolean bUseSurfaceEdges[2];
  bUseSurfaceEdges[0] = bUseSurfaceEdges[1] = FALSE;
  SmExtent2d sUVDomain = rpNewSurface->GetNaturalUVDomain();
  SER(rpNewSurface->GlobalSurfaceIntersect(crContext,
                                           sUVDomain,
                                           *rpNewSurface,
                                           sUVDomain,
                                           bUseSurfaceEdges,
                                           SM_CAST_APPROXTOL3D_PTR(&m_dThisApproxTol3d),
                                           &dAngleTol,
                                           &rSelfIntersections.m_v3DCurves,
                                           SM_REINTERPRET_CAST(SmTArray<SmCurve*> *,
                                           &rSelfIntersections.m_vUVCurves1),
                                           SM_REINTERPRET_CAST(SmTArray<SmCurve*> *,
                                           &rSelfIntersections.m_vUVCurves2),
                                           &rSelfIntersections.m_vCurveTypes,
                                           &rSelfIntersections.m_vDeviations));
#endif // 0

  // all done
  return SM_SUCCESS;

} // end SmOffsetGeometryCreation::EdgeuseOffset





