// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVertexuse.cpp
* PURPOSE: Source file for SmVertexuse class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmVertexuse.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#include <SmSolutionArray.h>
#include <SmSurface.h>
#include <SmSurfaceCache.h>
#include <SmGraphicsExtern.h>
#include <SmGap.h>
#include <SmAssertArray.h>
#include <SmGraphicsOutput.h> 
#include <SmCrvOnSurf.h>
#include <SmCircle.h>    

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE
//
//    SmVertexuse * dbgVertexuse1 = NULL ;
//    SmVertexuse * dbgVertexuse2 = NULL ;
//
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmUVSectorIO::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  SmFace    * pFace       = (m_pVertexuse && m_pVertexuse->GetFaceuse()) ? m_pVertexuse->GetFaceuse()->GetFace() : NULL ;
  SmSurface * pSurface    = (pFace) ? pFace->GetSurface() : NULL ;
  SmExtent2d  sSurfDomain ; if(pSurface) { sSurfDomain = pSurface->GetNaturalUVDomain() ; } 

  smos_WriteBuffer(_T("\nBegin SmUVSectorIO Dump")) ; 

  // header
  smos_sprintf(sBuff,        _T("\n  Vertexuse:[0x%p], On Face:[0x%p], with Surface:[0x%p]"), m_pVertexuse, pFace, pSurface);
  smos_sprintf(sBuffForFile, _T("\n  Vertexuse:[%s], On Face:[%s], with Surface:[%s]"), 
                             m_pVertexuse ? _T("NotNULL") : _T("NULL"),
                             pFace        ? _T("NotNULL") : _T("NULL"),
                             pSurface     ? _T("NotNULL") : _T("NULL") ) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // no work - no surface
  if(pSurface == NULL)
    {
      smos_WriteBuffer(_T("\n  pSurface is NULL - no data to display")) ;
      smos_WriteBuffer(_T("\nEnd SmUVSectorIO Dump")) ; 
      return ;
    } // end pSurface is NULL check

  // Beg Sector data
  smos_sprintf(sBuff,        _T("\n  BegEdgeuse:[0x%p], Param:[%16.16lf], BegUVPoint:[%7.7lf,%7.7lf], BegUVTan:[%7.7lf, %7.7lf]"), 
                           m_pBegEdgeuse, 
                           m_dBegEdgeParam,
                           m_sBegEdgeUVPoint.x,
                           m_sBegEdgeUVPoint.y,
                           m_sBegEdgeUVTan.x,
                           m_sBegEdgeUVTan.y) ;
  smos_sprintf(sBuffForFile, _T("\n  BegEdgeuse:[%s], Param:[%16.16lf], BegUVPoint:[%7.7lf,%7.7lf], BegUVTan:[%7.7lf, %7.7lf]"), 
                           m_pBegEdgeuse ? _T("NotNULL") : _T("NULL"), 
                           m_dBegEdgeParam,
                           m_sBegEdgeUVPoint.x,
                           m_sBegEdgeUVPoint.y,
                           m_sBegEdgeUVTan.x,
                           m_sBegEdgeUVTan.y) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // End Sector data
  smos_sprintf(sBuff,        _T("\n  EndEdgeuse:[0x%p], Param:[%16.16lf], EndUVPoint:[%7.7lf,%7.7lf], EndUVTan:[%7.7lf, %7.7lf]"), 
                           m_pEndEdgeuse, 
                           m_dEndEdgeParam,
                           m_sEndEdgeUVPoint.x,
                           m_sEndEdgeUVPoint.y,
                           m_sEndEdgeUVTan.x,
                           m_sEndEdgeUVTan.y) ;
  smos_sprintf(sBuffForFile, _T("\n  EndEdgeuse:[%s], Param:[%16.16lf], EndUVPoint:[%7.7lf,%7.7lf], EndUVTan:[%7.7lf, %7.7lf]"), 
                           m_pEndEdgeuse ? _T("NotNULL") : _T("NULL"), 
                           m_dEndEdgeParam,
                           m_sEndEdgeUVPoint.x,
                           m_sEndEdgeUVPoint.y,
                           m_sEndEdgeUVTan.x,
                           m_sEndEdgeUVTan.y) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Sector angle data
  smos_sprintf(sBuff, _T("\n  m_bVertexOnPole:%s, m_dSectorAngDeg  :[%16.16lf]"), m_bVertexOnPole ? _T("[TRUE ]") : _T("[FALSE]"), m_dSectorAngDeg) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n  m_bVertexOnPole:%s, SurfNatInterval_U:[%16.16lf, %16.16lf], DelU:[%16.16lf]"), 
                      m_bVertexOnPole ? _T("[TRUE ]") : _T("[FALSE]"),
                      sSurfDomain.GetUMin(), sSurfDomain.GetUMax(), sSurfDomain.GetUMax() - sSurfDomain.GetUMin()) ;
  smos_sprintf(sBuff, _T("                             SectorInterval_U :[%16.16lf, %16.16lf], DelU:[%16.16lf]"), 
                      m_sBegEdgeUVPoint.x, m_sEndEdgeUVPoint.x, smos_Fabs(m_sBegEdgeUVPoint.x - m_sEndEdgeUVPoint.x)) ; smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff, _T("\n                           SurfNatInterval_V:[%16.16lf, %16.16lf], DelV:[%16.16lf]"), 
                      sSurfDomain.GetVMin(), sSurfDomain.GetVMax(), sSurfDomain.GetVMax() - sSurfDomain.GetVMin()) ;
  smos_sprintf(sBuff, _T("                             SectorInterval_V :[%16.16lf, %16.16lf], DelU:[%16.16lf]"), 
                      m_sBegEdgeUVPoint.y, m_sEndEdgeUVPoint.y, smos_Fabs(m_sBegEdgeUVPoint.y - m_sEndEdgeUVPoint.y)) ; smos_WriteBuffer(sBuff) ;

  // Sector Interval
  smos_sprintf(sBuff, _T("\n  SectorInterval      :[%16.16lf]"), m_dSectorAngDeg) ; smos_WriteBuffer(sBuff) ;

  // properties
  smos_WriteBuffer(_T("\n Properties")) ;
  smos_sprintf(sBuff, _T("\n  m_bVertexOnClosedU   :%s"), m_bVertexOnClosedU    ? _T("[TRUE]") : _T("[FALSE]")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n  m_bVertexOnClosedV   :%s"), m_bVertexOnClosedV    ? _T("[TRUE]") : _T("[FALSE]")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n  m_bVertexOnPole      :%s"), m_bVertexOnPole       ? _T("[TRUE]") : _T("[FALSE]")) ; smos_WriteBuffer(sBuff) ;

  // probs
  smos_WriteBuffer(_T("\n Problems")) ;
  smos_sprintf(sBuff, _T("\n  m_bVertexOnFlatCorner:%s"), m_bVertexOnFlatCorner ? _T("[TRUE] - Bad") : _T("[FALSE] - Okay")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n  m_bSectorMissingSeamU:%s"), m_bSectorMissingSeamU ? _T("[TRUE] - Bad") : _T("[FALSE] - Okay")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n  m_bSectorMissingSeamV:%s"), m_bSectorMissingSeamV ? _T("[TRUE] - Bad") : _T("[FALSE] - Okay")) ; smos_WriteBuffer(sBuff) ;

  smos_WriteBuffer(_T("\nEnd SmUVSectorIO Dump")) ; 

} // end SmUVSectorIO::Dump

/*******************************************************************//**
PURPOSE: Compute the UVPoint on the Vertexuse->Face->Surface
         closest to the vertexuse->vertex->point.

NOTES: When there is only one Vertex/Face connection there is only 
         one possible UVPoint value to return.
       When there are multiple Vertex/Face connections as in the cases:
         1. A Vertex on a pole where 1 3DPoint maps to a line of UVPoints, and
         2. A Vertex on a seam where 1 3DPoint maps to 2 or even 3 or 4 UVPoints
       And the Vertexuse is part of a Loop being an EdgeVertexuse type,
       the returned UVPoint is the one closest to the UVTrimCurve endpoints
       connected to this Vertexuse.  Otherwise, it returns any UVPoint that
       it can find.

***********************************************************************/
SmStatus SmVertexuse::ComputeUVPoint
 (SmPoint2d & rUVPoint,             // out: Projection of Vertexuse->Vertex->Point onto
                                    //      Vertexuse->Edgeuse->Loopuse->Faceuse->Face->Surface
  SmBoolean   bOKToMakeUVTrimCurve) // in : TRUE  = use old or make new UVTrimCurves to get LocalPointSolve() UVGuesses 
                                    //      FALSE = only use old UVTrimCurves - if missing call GlobalPointSolve() with no UVGuess
                                    //      default:[TRUE]
 const
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  //if (this->m_sUVPoint != SmPoint2d(SM_BIG_DOUBLE, SM_BIG_DOUBLE))
  //{
  //   rUVPoint = this->m_sUVPoint;
  //   return SM_SUCCESS;
  //}
  // Vertex and Face locals
  SmPoint2d   sUV;
  SmVertex  * pVertex     = GetVertex();
  SmPoint3d   sVertexPnt  = pVertex->GetPoint();
  SmFaceuse * pFaceuse    = GetFaceuse();         NER(pFaceuse);
  SmFace    * pFace       = pFaceuse->GetFace();  NER(pFace);
  SmSurface * pSurface    = pFace->GetSurface();  NER(pSurface);
  SmExtent2d  sUVDomain   = pFace->GetUVDomain();
  SmExtent2d  sNatDomain  = pSurface->GetNaturalUVDomain();
  // case EdgeVertexUse
  //   Get this (and Prev) UVTrimCurves connected to this Vertexuse
  //   project VertexPoint down to Face's Surface
  //    1. try Project Point to Surface with LocalPointSolve() with UVGuess = UVTrimCurve->NormalizedEvaluate()
  //    2. If that fails - try global project point to Face's Surface
  //    3. If that fails - signal error
  // note: this method was extended for healing to handle ClosedFaces with missing SeamEdges.
  //       For such incalid cases a vertexuse sitting on a Surface seam can be connected to
  //       This and Prev Edgeuses that lie on opposite sides of the seam boundary.
  //       Ex: the Vertex on the CircleTop to a CylinderFace without a SeamEdge.
  //       That Vertex is both the 
  //       This next section has been modified to use This and Prev Edgeuse 
  //       Start and End Tangents to figure out which side of the missing Seam Edge
  //       this Vertexuse lies.
  //       In the valid case of a SphereFace with its SeamEdge, The vertexuse can be
  //       sitting on a pole at the end of a Seam in which case the vertexuse
  //       is also attached to UVTrimCurves on opposite sides of the seam and
  //       the vertex maps to an entire line of UVPoints (along the singular boundary) and
  //       this method can only guess which of all those values to return.
  if (IsEdgeVertexuse())
    {
      // Edgeuse locals - thisEdgeuse->StartPt == CWEdgeuse->EndPt (with tolerances)
      SmEdgeuse *pEdgeuse     = GetEdgeuse() ; NER(pEdgeuse);
      SmEdgeuse *pPrevEdgeuse = pEdgeuse->GetCWEdgeuse() ; NER(pPrevEdgeuse);
      SmBoolean  bFoundAnswer = FALSE;
      SmSolution sSol ;

      // JGU - condition is obsolete. Instead, pass bOkToMakeUVTrimCurve to NormalizedEvaluate.
      // when Edgeuse and PrevEdgeuse already have a UVTrimCurve or when asked to build UVTrimCurves when needed
      //if(   (   pEdgeuse->GetUVTrimCurve()
      //       && pPrevEdgeuse->GetUVTrimCurve())
      //   || bOKToMakeUVTrimCurve)
        {
          // try quick solution based on this (and prev) Vertexuse->Edgeuse->UVTrimCurve data
          SmPoint3d sThisUVPnt, sThisUVTan;
          SmPoint3d sPrevUVPnt, sPrevUVTan;

          // get this SurfaceUV for ThisEdgeuse start point from UVTrimCurve
          SER(pEdgeuse->NormalizedEvaluate(0.0,TRUE,sThisUVPnt,&sThisUVTan, bOKToMakeUVTrimCurve));  // TRUE = UV Eval, FALSE = 3d Eval
          SmPoint2d sUVGuess(sThisUVPnt.x,sThisUVPnt.y);

          // get SurfaceUV for PrevEdgeuse end pointfrom UVTrimCurve
          SER(pPrevEdgeuse->NormalizedEvaluate(1.0,TRUE,sPrevUVPnt,&sPrevUVTan, bOKToMakeUVTrimCurve));  // TRUE = UV Eval, FALSE = 3d Eval

          // when PrevUV and ThisUV UVTrimCurve EndPts are far apart - expected rare case - check for singularities and seams 
          SmExtent2d sSurfUVDomain = pSurface->GetNaturalUVDomain() ;
          double     dDistU = smos_Fabs(sThisUVPnt.x - sPrevUVPnt.x) ;
          double     dDistV = smos_Fabs(sThisUVPnt.y - sPrevUVPnt.y) ;
          double     dDomainLengthU = sSurfUVDomain.XLength() ;
          double     dDomainLengthV = sSurfUVDomain.YLength() ;
          if(   dDistU > dDomainLengthU/4.0
             || dDistV > dDomainLengthV/4.0)
            {                                 
              // When the UVTrimCurves EndPts are far apart in UVSpace but close in 3DSpace that may be due to
              // the UVTrimCurves being on different sides of a Seam 
              //  (vertices on poles,
              //   vertices on missing SeamEdges,
              //   vertices on poles and a SeamEdge)  
              SmBoolean bClosedU = pSurface->IsClosed(sSurfUVDomain, SM_SP_U) ;
              SmBoolean bClosedV = pSurface->IsClosed(sSurfUVDomain, SM_SP_V) ;

              // check for seam with UVTrimCurves on conflicting sides problem - UVDist size of the UVDomain and Surface is closed
              if(   (bClosedU && SM_ARE_SAME_TO_TOL(dDistU, dDomainLengthU, SM_EFF_ZERO_PARAM * 1000))
                 || (bClosedV && SM_ARE_SAME_TO_TOL(dDistV, dDomainLengthV, SM_EFF_ZERO_PARAM * 1000)))
                {                                                                   
                  // for cases of just one closed direction (cylinders, spheres) - orthogonalize the endTans to cross the seam
                  if     ( bClosedU && !bClosedV) { sThisUVTan.y = 0.0 ; sPrevUVTan.y = 0.0 ; }
                  else if(!bClosedU &&  bClosedV) { sThisUVTan.x = 0.0 ; sPrevUVTan.x = 0.0 ; }

                  // skip cases where the EndTans are parallel to Seam (ex: vertex on pole at end of seam)
                  //  after the orthogonalization that can be checked by checking for degenerate tangents
                  SmXSectTol3d sXSectTol = SM_EFF_ZERO_PARAM * 1000;
                  if(   !SmTol::IsDegenerate(sThisUVTan, &sXSectTol)
                     && !SmTol::IsDegenerate(sPrevUVTan, &sXSectTol))
                    {

                      // looks like we have UVTrimCurves on different sides of a seam - use UVTans to pick the right side
                      // check that steps from UVTrimPoint End point are inside the surface domain - use the endpoint that
                      // has two valid steps.
                      SmPoint2d sThisStep_FromThis ; sThisStep_FromThis = (sThisUVPnt + .001 * sThisUVTan) ;
                      SmPoint2d sPrevStep_FromThis ; sPrevStep_FromThis = (sThisUVPnt + .001 * sPrevUVTan) ;

                      SmPoint2d sThisStep_FromPrev ; sThisStep_FromPrev = (sPrevUVPnt + .001 * sThisUVTan) ;
                      SmPoint2d sPrevStep_FromPrev ; sPrevStep_FromPrev = (sPrevUVPnt + .001 * sPrevUVTan) ;

#ifdef SM_DEBUG_CODE
                      if(bDebugMe)
                        {
                          SmEdgeuse * pThisRadial = pEdgeuse->GetRadial() ;
                          SmEdgeuse * pPrevRadial = pPrevEdgeuse->GetRadial() ;

                          SM_DUMP_AND_ASSERT_VALID(this) ;          // will be recursive - skip line if there is a problem in this method 
                          SM_DUMP_AND_ASSERT_VALID(pEdgeuse) ;      // will be recursive - skip line if there is a problem in this method 
                          SM_DUMP_AND_ASSERT_VALID(pPrevEdgeuse) ;  // will be recursive - skip line if there is a problem in this method 
                                                                                             
                          SM_DUMP_AND_ASSERT_VALID(pThisRadial) ;   // will be recursive - skip line if there is a problem in this method 
                          SM_DUMP_AND_ASSERT_VALID(pPrevRadial) ;   // will be recursive - skip line if there is a problem in this method 

                          SmBrep * pBrep = GetBrep() ;
                          SmPoint2d sThisUVPnt2d ; sThisUVPnt2d = sThisUVPnt ;
                          SmPoint2d sPrevUVPnt2d ; sPrevUVPnt2d = sPrevUVPnt ;
                          SmPoint3d sThisUVPnt3d ; pSurface->EvaluatePoint(sThisUVPnt2d, sThisUVPnt3d) ;
                          SmPoint3d sPrevUVPnt3d ; pSurface->EvaluatePoint(sPrevUVPnt2d, sPrevUVPnt3d) ; 
                          SmPoint3d sThisStep_FromThis3d ; pSurface->EvaluatePoint(sThisStep_FromThis, sThisStep_FromThis3d) ;
                          SmPoint3d sPrevStep_FromThis3d ; pSurface->EvaluatePoint(sPrevStep_FromThis, sPrevStep_FromThis3d) ;
                          SmPoint3d sThisStep_FromPrev3d ; pSurface->EvaluatePoint(sThisStep_FromPrev, sThisStep_FromPrev3d) ;
                          SmPoint3d sPrevStep_FromPrev3d ; pSurface->EvaluatePoint(sPrevStep_FromPrev, sPrevStep_FromPrev3d) ;                        

                          smgfx_Erase() ;
                          smgfx_SetLook(1,2,   0,0,1) ;  if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(6,7,   1,0,0) ;  if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2,   0,0,0) ;  if(pFace) pFace->Draw(SM_DM_CROSSHATCH,7,7) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2,   0,1,2) ;  if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;

                          smgfx_SetLook(1,2,   0,1,0) ;  if(pEdgeuse) pEdgeuse->Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2,   1,0,0) ;  if(pEdgeuse) pEdgeuse->DrawEdgeFaceTrimCurveGap() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2,   0,1,1) ;  if(pPrevEdgeuse) pPrevEdgeuse->Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(1,2,   0,1,0) ;  if(pPrevEdgeuse) pPrevEdgeuse->DrawEdgeFaceTrimCurveGap() ; sm_GraphicsLoop() ;

                          smgfx_SetLook(5,6,   0,1,0) ;  if(pSurface) sThisUVPnt3d.Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(7,8,   1,0,0) ;  if(pSurface) sPrevUVPnt3d.Draw() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(8,9,   0,1,1) ;  if(pSurface) sThisStep_FromThis3d.DrawPointToPoint(sThisUVPnt3d) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(8,9,   0,1,1) ;  if(pSurface) sPrevStep_FromThis3d.DrawPointToPoint(sThisUVPnt3d) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(10,11, 1,0,1) ;  if(pSurface) sThisStep_FromPrev3d.DrawPointToPoint(sPrevUVPnt3d) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(10,11, 1,0,1) ;  if(pSurface) sPrevStep_FromPrev3d.DrawPointToPoint(sPrevUVPnt3d) ; sm_GraphicsLoop() ;

                          smgfx_SetLook(4,5, .2,.2,.2) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawPoles() ; sm_GraphicsLoop() ;
                          smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
                          sm_GraphicsLoop() ;
                        }
#endif // SM_DEBUG_CODE
                      // when steps from sThisUVPnt are both in the SurfUVDomain
                      if(   sSurfUVDomain.ContainsPoint2d(sThisStep_FromThis, SM_EFF_ZERO_PARAM)
                         && sSurfUVDomain.ContainsPoint2d(sPrevStep_FromThis, SM_EFF_ZERO_PARAM))
                        {
                          // The Vertexuse is probably on the sThisUVPnt side of the seam
                          sUVGuess = sThisUVPnt ;

                          // check assumption
                          SM_ASSERT_MSG(   !sSurfUVDomain.ContainsPoint2d(sThisStep_FromPrev, SM_EFF_ZERO_PARAM)
                                        || !sSurfUVDomain.ContainsPoint2d(sPrevStep_FromPrev, SM_EFF_ZERO_PARAM),
                                        _T("SmVertexuse::ComputeUVPoint - OnTheSeam with conflicting UVTrimCurves assumption is wrong - check case")) ;
                        } // end UVTan Steps from ThisUVPnt are in UVDomain check

                      // else when steps from sPrevUVPnt are both in the SurfUVDomain
                      else if(   sSurfUVDomain.ContainsPoint2d(sThisStep_FromPrev, SM_EFF_ZERO_PARAM)
                              && sSurfUVDomain.ContainsPoint2d(sPrevStep_FromPrev, SM_EFF_ZERO_PARAM))
                        {
                          // The Vertexuse is probably on the sThisUVPnt side of the seam
                          sUVGuess = sPrevUVPnt ;

                          // check assumption
                          SM_ASSERT_MSG(   !sSurfUVDomain.ContainsPoint2d(sThisStep_FromThis, SM_EFF_ZERO_PARAM)
                                        || !sSurfUVDomain.ContainsPoint2d(sPrevStep_FromThis, SM_EFF_ZERO_PARAM),
                                        _T("SmVertexuse::ComputeUVPoint - OnTheSeam with conflicting UVTrimCurves assumption is wrong - check case")) ;
                        } // end UVTan Steps from PrevUVPnt are in UVDomain check

                      // else our assumptions and tests are not properly detecting a SeamCurve with UVTrimCurves on different sides
                      else
                        { SM_ASSERT_MSG(FALSE,
                                        _T("SmVertexuse::ComputeUVPoint - Tests for OnTheSeam with conflicting UVTrimCurves are wrong - check case")) ;
                        } 
                    } // end skip cases of UVTangents not crossing the seam
                } // end OnSeam with ConflictingUVTrimCurves problem case check
            } // end conflicting UVGuess values check

          // limit UV points surface domain
          // GWC:NOTE - we may have to relax this for vertices placed
          //            with a little tolerance
          sUVGuess = sUVDomain.ClampPoint2d(sUVGuess);

          // try to project vertex point down onto surface
          SER(pSurface->LocalPointSolve(pFace->GetUVDomain(),
                                        SM_SO_MINIMIZE,
                                        sVertexPnt,
                                        sUVGuess,
                                        bFoundAnswer,
                                        sSol));

          // If the found solution is more than half the surface domain from the TrimCurve generated guess
          // assume we jumped a seam and try GlobalPointSolve [B678]
          if ( bFoundAnswer
               &&  ( ( smos_Fabs( sSol.m_vStart[0] - sUVGuess.x ) > sNatDomain.XLength() / 2 )
                  || ( smos_Fabs( sSol.m_vStart[1] - sUVGuess.y ) > sNatDomain.YLength() / 2 ) ) )
          { bFoundAnswer = FALSE; }

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              Dump() ;
              sSol.Dump() ;
              SmPoint2d sSolUV(sSol.m_vStart.m_adParameters[0], sSol.m_vStart.m_adParameters[1]) ;
              SmPoint3d s3dGuess, sSolPoint ;
              SmVertexEdgeGap *pVertexEdgeGap = pEdgeuse->GetVertexEdgeGap() ;

              if(pVertexEdgeGap) { pVertexEdgeGap->Dump() ; }
              if(pSurface) { pSurface->Dump() ;
                             pSurface->EvaluatePoint(sUVGuess, s3dGuess) ; 
                             pSurface->EvaluatePoint(sSolUV, sSolPoint) ;
                           }
              SmBrep * pBrep =   pVertex ? pVertex->GetBrep()
                               : pFace   ? pFace->GetBrep()
                                         : NULL ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,7, 1,0,0) ; if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH,7,7) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; if(pEdgeuse) pEdgeuse->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; if(pEdgeuse) pEdgeuse->DrawEdgeFaceTrimCurveGap() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,2) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,0) ; if(pSurface) s3dGuess.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 0,0,1) ; if(pSurface) sSolPoint.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, .2,.2,.2) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE
        }

      // If local solve fails then go back and do a global
      // closest point on the surface.
      //   Local Solve fails when point is not above surface
      //   but lies off to one side of it.
      if(  !bFoundAnswer
         || sSol.m_vStart.m_dSolutionValue > pVertex->GetTolerance()+pFace->GetTolerance())
        {
          SmSolution        sData[4];
          SmSolutionArray   sSolutions(4,sData);
          SmSurfaceCache  * pSC = smsurf_GetSurfaceCache(pSurface); NER(pSC);
          SmCacheCheckOutIn sCheckIO(pSC);
            {
              // Make the surface think that it is a standalone surface
              // Turn on boundary curve processing to force LocalSolve to look for drop points
              //   on boundary curves.
              SmTemporaryChangeValue<SmBoolean> sChange(pSC->m_bHaveTSurfaceCache,FALSE);
              SmTemporaryChangeValue<SmBoolean> sChange1(pSC->m_bProcessBoundaryCurves,TRUE);

              // Turn off point testing if we can't use UVTrimCurves
              SmBoolean bPointTestEnabled = TRUE;
              if ( !bOKToMakeUVTrimCurve )
                {
                  bPointTestEnabled        = pSC->m_bPointTestEnabled;
                  pSC->m_bPointTestEnabled = FALSE;
                }

              SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(),
                                             SM_SO_MINIMIZE,
                                             sVertexPnt,
                                             pVertex->GetTolerance()+pFace->GetTolerance(),
                                             NULL,
                                             SM_SR_SINGLE,
                                             sSolutions));

              // Reset m_bPointTestEnabled
              if ( !bOKToMakeUVTrimCurve )
                { pSC->m_bPointTestEnabled = bPointTestEnabled; }
            }

          if (sSolutions.GetSize() < 1)
            { SER_MSG(SM_ERR, _T("VertexUse->Vertex->Point does not project to Surface")); }

          // Use UVTrimCurves to choose the right drop point [B678]
          if (sSolutions.GetSize() > 1)
          {
              // Locals
              SmPoint3d sThisUVPnt;
              SmPoint2d sSolPnt, sGuessPnt;
              double    dDist, dMinDist = SM_BIG_DOUBLE;

              // UVTrimCurve should be on the right side of the seam. Solution closest to it is the one we want
              SER( pEdgeuse->NormalizedEvaluate( 0.0, TRUE, sThisUVPnt, NULL, bOKToMakeUVTrimCurve ) );  // TRUE = UV Eval, FALSE = 3d Eval
              sGuessPnt.Set( sThisUVPnt.x, sThisUVPnt.y );
              for ( ULONG ii = 0; ii < sSolutions.GetSize(); ++ii )
              {
                  sSolPnt.Set(sSolutions[ii].m_vStart[0], sSolutions[ii].m_vStart[1]);
                  dDist = sSolPnt.DistanceBetweenSquared( sGuessPnt );
                  if ( dDist < dMinDist )
                  {
                      sSol = sSolutions[ii];
                      dMinDist = dDist;
                  }
              }
          }
          else
          { sSol = sSolutions[0]; }
        } // end local project point fail branch

      // when projection dist is larger than tolerance
      if (sSol.m_vStart.m_dSolutionValue > pVertex->GetTolerance() + pFace->GetTolerance())
        {
          // print error when projection distance is large
          if (sSol.m_vStart.m_dSolutionValue/1000.0 > pVertex->GetTolerance() + pFace->GetTolerance()) {
              SE(SM_ERR);
            }
          // increment the tolerance
          // Even though this vertex is too far from the face, this function
          // should not be modifying the tolerance.
          // pVertex->SetTolerance(sSol.m_vStart.m_dSolutionValue * 2.0);
        } // end projection dist > tolerance check

      // store the solution
      sUV = SmPoint2d(sSol.m_vStart[0],sSol.m_vStart[1]) ;

    } // end case EdgeVertexuse

  // case LoopVertexertexuse
  else if (IsLoopVertexuse())
    {
      // project point down to face's surface
      SmSolution        sData[10];
      SmSolutionArray   sSolutions(10,sData);
      SmSurfaceCache  * pSC = smsurf_GetSurfaceCache(pSurface); NER(pSC);

      SmCacheCheckOutIn sCheckIO(pSC);
        {
          // Make the surface think that it is a standalone surface.
          // Turn on boundary curve processing to force LocalSolve to look for drop points
          //   on boundary curves.
          SmTemporaryChangeValue<SmBoolean> sChange (pSC->m_bHaveTSurfaceCache,    FALSE);
          SmTemporaryChangeValue<SmBoolean> sChange1(pSC->m_bProcessBoundaryCurves,TRUE);
          SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(),
                                         SM_SO_INTERSECT,
                                         sVertexPnt,
                                         pVertex->GetTolerance()+pFace->GetTolerance(),
                                         NULL,
                                         SM_SR_ALL,
                                         sSolutions));

          // If that failed, try again with Minimize.  [090827]
          if (sSolutions.GetSize() < 1)
            {
              SM_DBG_WARN(_T("SmVertexuse::ComputeUVPoint(): Global Point Solve with INTERSECT failed.") );
              SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(),
                                             SM_SO_MINIMIZE,
                                             sVertexPnt,
                                             pVertex->GetTolerance()+pFace->GetTolerance(),
                                             NULL,
                                             SM_SR_ALL,
                                             sSolutions));
            }
        } // end scope for SmTemporaryChangeValue objects

      // should always be at least one solution
      if (sSolutions.GetSize() < 1)
        { return (SM_ERR); }

      // return only solution when just 1, or any solution when many (on a pole or seam)
      SmSolution & rSol = sSolutions[0];
      sUV = SmPoint2d(rSol.m_vStart[0],rSol.m_vStart[1]);

#ifdef SM_DEBUG_CODE
      SmSurfParamType eSurfParmType ;
      SM_ASSERT(   (sSolutions.GetSize() == 1)                            // typical
                || (   sSolutions.GetSize() == 2                          // solution on seam
                    && (   pSurface->IsSingularity(sUV, eSurfParmType)
                        || pSurface->IsOnSeam(sUV)))
                || (   sSolutions.GetSize() == 4                          // solution on closed torus vertex
                    && (   pSurface->IsOnSeam(sUV, NULL, &eSurfParmType))
                        && eSurfParmType == SM_SP_BOTH) ) ;
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          Dump() ;    // note: a Dump() call here is recursive - it calls ComputeUVPoint
          sSolutions.Dump() ;
          if(pSurface) pSurface->Dump() ;

          SmBrep * pBrep =   pVertex    ? pVertex->GetBrep()
                           : pFace ? pFace->GetBrep()
                                   : NULL ;
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7, 1,0,0) ; if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH,7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,2) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .2,.2,.2) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawPoles() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end case LoopVertexertexuse

  // default case: must be a ShellVertexuse
  else
    {
      SER(SM_ERR);
    } // end default case

  rUVPoint = sUV;

  return SM_SUCCESS;

} // end SmVertexuse::ComputeUVPoint

/*******************************************************************//**
PURPOSE: Compute the UVPoint and Start and End Sector UVTangents
         for the corner represented by this Vertexuse,  
         reports if the sector vertex sits on a natural boundary 
         singularity and/or if the sector spans a missing seam edge.

NOTES: Returns SM_SUCCESS when Vertexuse->Type == SmEdgeuse_TYPE and
       the edges are connected to a face forming a vertex sector
       on a face, otherwise returns SM_ERR

SINGULARITY/CLOSED NOTE --- Some surfaces map many UVPoints to 1 3DPoint.  
 This happens for surface singularities (an entire isoParameter line 
 maps to 1 3DPoint) and for closed surfaces (2, or even 3 or 4, 
 UVPoints map to 1 3DPoint).

 For Edgevertexuse on Singularity or Closed points
   Computed rUVPoint    : returns the UVPoint closest to the associated 
                          UVTrimCurve endPoint if possible. If not  
                          possible returns any UVPoint that can be found.
            OptUVPoint2 : returns the UVPoint on the other end of the
                          sectors Singularity or the matching Closed point
                          UVPoint.  (Extremely rare cases of closed
                          surfaces with 3 or 4 UVPoints for 1 3DPoint are
                          not fully reported)  gwc:closed tori aren't all that rare.

 For EdgeVertexuse types on singularities,
   Computed UVTan   : - one appropriate UVTan will point along the singularity.
   & dSectorUVAngDeg: - the other UVTan will be mapped to a UVDir found by dropping
                        the associated edge's 3DEndTangent to the UVPlane
                        of the surface.  
                      - For a vertex on a closed singularity
                        missing a seam edge - both UVTans will be in the direction
                        of the Singularity in UVSpace and the reported dSectorUVAngDeg
                        will be 180 degrees.

 Sector on Singularity note:  1 3d sector on a singularity can map to
 one or two UV sectors depending on the uvPoint locations associated with
 the edgeuses coming into the 3d sector.  When those UVPoints are the same
 there is only one UV Sector.  If they are different, then the 3d Sector
 really maps to 3 UVEdgeuses and 2 UVSectors. 
 
                Vertex on Singularity   
     UV Space:                              3D Space:
         2 UVEndPoints map to 1 3DPoint        1 Vertex has 1 3d Face Sector
               +----------------+                  + Single vertex
     outgoing-/ _/            \_ \-incoming       / \- incoming edge
     Edgeuse / UVSector   UVSector\edgeuse       /   \
            /   1             2                 /- outgoing Edge
     In this case one Vertex should have 2 vertexuses that map to different
     UVPoints.  Depending on the case, there will or won't be an actual
     degenerate 3dEdge between the two UVPoints with an associated UVTrimCurve 
     that runs along the singularity boundary.  When the degenerate 3DEdge 
     is not present, the case is treated as if the edge were present.
 ***********************************************************************/
SmStatus SmVertexuse::ComputeUVSector
 (SmUVSectorIO & rUVSectorIO,          // out: Optional Sector properties.  NULL to ignore, default:[NULL]
  SmBoolean      bOKToMakeUVTrimCurve) // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve
                                       //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve, default:[TRUE]
 const                                   
  // where SmUVSectorIO contains:    
  // SmVertexuse *  m_pVertexuse           // in : sector's defining Vertexuse
  // SmEdgeuse *    m_pBegEdgeuse          // out: pBegEdgeuse   = sector BegEdgeuse
  // SmEdgeuse *    m_pEndEdgeuse          // out: pEndEdgeuse   = sector EndEdgeuse
  // double         m_dBegEdgeParam        // out: dBegEdgeParam = sector BegEdge corner Param
  // double         m_dEndEdgeParam        // out: dEndEdgeParam = sector EndEdge corner Param
  // 
  // SmPoint2d      m_sBegEdgeUVPoint      // out: sBegEdgeUVPoint = UVPosition of Vertexuse marking the beg of this sector
  // SmPoint2d      m_sEndEdgeUVPoint      // out: sEndEdgeUVPoint = UVPosition of VertexUse marking the end of this sector
  //                                       //                  = if(GetVertexuseType() == SmEdgeuse_TYPE)
  //                                       //                      if(bVertexOnClosed)          Other UVPoint mapping to same 3dPoint
  //                                       //                      else if(bVertexOnSingulariy) Other end of the Singularity bndry Ivl 
  //                                       //                                                    mapping to this vertex
  //                                       //                      else                         Should be same UVPoint as UVBegEdgePoint.
  //                                       //                      else                         SetUninitialized()
  // SmPoint2d      m_sBegEdgeUVTan        // out: sBegEdgeUV1stDir = UV 1st Derivative from the SectorPoint along BegSector arm
  // SmPoint2d      m_sEndEdgeUVTan        // out: sEndEdgeUV1stDir = UV 1st Derivative from the SectorPoint along EndSector arm
  // double         m_dSectorAngDeg        // out: UVSpace rotation from BegTan to EndTan in the UVplane corner, range:[0 360]                                                              
  // 
  // double         m_bVertexOnClosedU     // out: bVertexOnClosedU:     TRUE = SectorVtx on closedU bndry, Pos[Umin,v] == Pos[Umax,v] for v smps
  // double         m_bVertexOnClosedV     // out: bVertexOnClosedV:     TRUE = SectorVtx on closedV bndry, Pos[u,Vmin] == Pos[u,Vmax] for u smps
  // SmBoolean      m_vVertexOnPole        // out: bVertexOnSingularity: TRUE = SectorVtx on Singularity
  // SmBoolean      m_bSectorMissingSeamU  // out: bSectorMissingSeamU: TRUE = Sector contains a missing Udir Seam 
  // SmBoolean      m_bSectorMissingSeamV  // out: bSectorMissingSeamV: TRUE = Sector contains a missing Vdir Seam 
{
  // init output - do this here because convenience class SmUVSectorIO has no methods.
    {
      rUVSectorIO.m_pVertexuse           = this ;
      rUVSectorIO.m_pBegEdgeuse          = NULL ;
      rUVSectorIO.m_pEndEdgeuse          = NULL ;
      rUVSectorIO.m_dBegEdgeParam        = SM_UNDEF_DOUBLE ;
      rUVSectorIO.m_dEndEdgeParam        = SM_UNDEF_DOUBLE ;

      rUVSectorIO.m_sBegEdgeUVPoint.SetUninitialized() ; 
      rUVSectorIO.m_sEndEdgeUVPoint.SetUninitialized() ; 
  
      rUVSectorIO.m_sBegEdgeUVTan.SetUninitialized() ; 
      rUVSectorIO.m_sEndEdgeUVTan.SetUninitialized() ; 
      rUVSectorIO.m_dSectorAngDeg        = SM_UNDEF_DOUBLE ; 
  
      rUVSectorIO.m_bVertexOnClosedU     = FALSE ;
      rUVSectorIO.m_bVertexOnClosedV     = FALSE ;
      rUVSectorIO.m_bVertexOnPole        = FALSE ;
      rUVSectorIO.m_bVertexOnFlatCorner  = FALSE ;

      rUVSectorIO.m_bSectorMissingSeamU  = FALSE ;
      rUVSectorIO.m_bSectorMissingSeamV  = FALSE ;
    } // end init rUVSectorIO values

  // no work - no sector
  if(GetVertexuseType() == SmShell_TYPE)
    { return(SM_ERR) ; }

  // locals
  // SmStatus    sRtn = SM_ERR ;
  SmVertex  * pVertex   = GetVertex() ;
  SmLoopuse * pLoopuse  = GetLoopuse() ;
  SmFaceuse * pFaceuse  = GetFaceuse() ;
  SmFace    * pFace     = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmSurface * pSurface  = pFace ? pFace->GetSurface() : NULL ;

  // no work - no geometry
  if(pVertex == NULL || pSurface == NULL)
    { return(SM_ERR) ; }

  // locals
  SmVector3d sVertexPnt = pVertex->GetPoint() ;
  SmExtent2d sUVDomain  = pFace->GetUVDomain();

  // compute the UV point - pass the call along
  ComputeUVPoint(rUVSectorIO.m_sBegEdgeUVPoint, bOKToMakeUVTrimCurve) ;

  // branch between Loop and Edge Vertices
  if(GetVertexuseType() == SmLoopuse_TYPE)
    {
      // return a full 360 deg face sector - Vertices on NatBndry will be EdgeVertices never LoopVertices => SecAng == 360. 
      rUVSectorIO.m_sBegEdgeUVTan.Set(1.0, 0.0) ;
      rUVSectorIO.m_sEndEdgeUVTan.Set(1.0, 0.0) ;
      rUVSectorIO.m_dSectorAngDeg = 360.0 ;

      // no edgeuses - leave edgeuse output in init state
      // sRtn = SM_ERR ;
    } // end Loopuse_TYPE vertices branch

  // Edgeuse type vertices branch
  else if(GetVertexuseType() == SmEdgeuse_TYPE)
    {
      //   A person walking along the Edgeuse from Start to End on the positive side
      //   of the faceuse will have the interior of the face to his lefthand side.
      //   That means Sector BegEU = pVU->EU and
      //                     EndEU = pVU->EU->CWEU  (clockwise for the edge order in the loop, ie. the loop's previous edgeuse)

      rUVSectorIO.m_pBegEdgeuse   = GetEdgeuse() ;
      rUVSectorIO.m_pEndEdgeuse   = rUVSectorIO.m_pBegEdgeuse ? rUVSectorIO.m_pBegEdgeuse->GetCornerMateEdgeuse(pVertex) : NULL ;
      SmVertexuse * pEndVertexuse =   (   rUVSectorIO.m_pEndEdgeuse 
                                       && rUVSectorIO.m_pEndEdgeuse->GetVertexuse()->GetVertex() == pVertex) 
                                    ? rUVSectorIO.m_pEndEdgeuse->GetVertexuse() 
                                    : rUVSectorIO.m_pEndEdgeuse->GetMate()->GetVertexuse() ;

      // no work - no face sector - attached to wires
      if(rUVSectorIO.m_pBegEdgeuse == NULL || rUVSectorIO.m_pEndEdgeuse == NULL)
        { return(SM_ERR) ; }      

      // locals
      SmEdge    * pBegEdge    = rUVSectorIO.m_pBegEdgeuse ? rUVSectorIO.m_pBegEdgeuse->GetEdge() : NULL ;
      SmEdge    * pEndEdge    = rUVSectorIO.m_pEndEdgeuse ? rUVSectorIO.m_pEndEdgeuse->GetEdge() : NULL ;
      SmCurve   * pBegCurve   = pBegEdge ? pBegEdge->GetCurve() : NULL ;
      SmCurve   * pEndCurve   = pEndEdge ? pEndEdge->GetCurve() : NULL ;

      // compute EndSector UVPoint - usually same as sBegUVPoint except for Vertices on Seams (with missing seam edges) or Singularities
      pEndVertexuse->ComputeUVPoint(rUVSectorIO.m_sEndEdgeUVPoint, bOKToMakeUVTrimCurve) ;
      SM_ASSERT_MSG(pEndVertexuse->GetVertex() == pVertex, 
                    _T("SmVertexuse::ComputeUVSector - found wrong Sector ending EndVertexuse - needs review")) ;

      // no work - no geometry
      if(pBegCurve == NULL || pEndCurve == NULL)
        { return(SM_ERR) ; }

      // locals
      double     dSectorAngRad ;
      SmVector3d sBegEdgePV[2], sEndEdgePV[2] ;
      SmExtent1d sBegEdgeIvl = pBegEdge->GetInterval() ;
      SmExtent1d sEndEdgeIvl = pEndEdge->GetInterval() ;

      // Begin and End EdgeParams
      rUVSectorIO.m_dBegEdgeParam = rUVSectorIO.m_pBegEdgeuse->GetOrientation() == SM_OT_SAME     ? sBegEdgeIvl.GetMin() : sBegEdgeIvl.GetMax() ;
      rUVSectorIO.m_dEndEdgeParam = rUVSectorIO.m_pEndEdgeuse->GetOrientation() == SM_OT_OPPOSITE ? sEndEdgeIvl.GetMin() : sEndEdgeIvl.GetMax() ;

      // Begin and End Tangent Vecs, Oriented From Vertex into Face
      pBegCurve->Evaluate(rUVSectorIO.m_dBegEdgeParam, 1, TRUE, sBegEdgePV, TRUE) ; // TRUE = NonZeroTangents
      pEndCurve->Evaluate(rUVSectorIO.m_dEndEdgeParam, 1, TRUE, sEndEdgePV, TRUE) ; // TRUE = NonZeroTangents
      if(rUVSectorIO.m_pBegEdgeuse->GetOrientation() == SM_OT_OPPOSITE) { sBegEdgePV[1] = -sBegEdgePV[1] ; }
      if(rUVSectorIO.m_pEndEdgeuse->GetOrientation() == SM_OT_SAME)     { sEndEdgePV[1] = -sEndEdgePV[1] ; }
      double dBegEdgeSpeed = sBegEdgePV[1].Length() ;
      double dEndEdgeSpeed = sEndEdgePV[1].Length() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          SmBrep *pBrep = GetBrep() ;
          SmExtent2d  sSurfaceDomain ;
          if(pSurface) sSurfaceDomain = pSurface->GetNaturalUVDomain() ;
          //SmSurfParamType eSingularDirection ;
          //SmBoolean bOnSingularity = pSurface->IsSingularity(rSectorUVPoint, eSingularDirection) ;
          //SmBoolean bOnClosed      = pSurface->IsOnSeam(rSectorUVPoint) ; 
          // SmBoolean bOnFlatCorner  = pSurface->IsFlatCorner(rSectorUVPoint) ;

          SM_DUMP_AND_ASSERT_VALID(pBegCurve) ;
          SM_DUMP_AND_ASSERT_VALID(pEndCurve) ;  
          SM_DUMP_AND_ASSERT_VALID(pSurface) ;

          smgfx_Erase() ;
          smgfx_SetLook( 1,2,  0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
          smgfx_SetLook( 2,3,  0, 1, 0) ; this->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook( 1,2,  0, 0, 0 ); if(pFace)    { pFace->DrawUV(); sm_GraphicsLoop(); }
          smgfx_SetLook( 1,2,  1, 1, 0 ); if(pSurface) { pSurface->DrawUV( 5, 5 ); sm_GraphicsLoop(); }
          smgfx_SetLook( 3,4, .3,.3,.3 ); if(pSurface) { pSurface->DrawSeams(); sm_GraphicsLoop(); }
          smgfx_SetLook( 5,6, .8,.4,.2 ); if(pSurface) { pSurface->DrawPoles(); sm_GraphicsLoop(); }
          smgfx_SetLook( 3,4,  1, 0, 0 ); if(pBegEdge) { pBegEdge->DrawParams(); sm_GraphicsLoop(); }
          smgfx_SetLook( 4,5,  1, 0, 1 ); if(pEndEdge) { pEndEdge->DrawParams(); sm_GraphicsLoop(); }
          smgfx_SetLook( 6,7,  0, 0, 0) ; sBegEdgePV[1].Draw(&sBegEdgePV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook( 6,7,  1, 0, 0) ; sEndEdgePV[1].Draw(&sEndEdgePV[0]) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // note: Even with NonZeroTangents, sBegPV[1] will be TolZero when on singularity in singularity direction
      if(   SmTol::IsZeroSpeed(dBegEdgeSpeed, sBegEdgePV[0].GetMaxDimension())
         || SmTol::IsZeroSpeed(dEndEdgeSpeed, sEndEdgePV[0].GetMaxDimension()))
        {
          SM_DBG_WARN(_T("SmVertexuse::ComputeUVSector - found UVSector with zero speed beg or End Sector Tangent vector.")) ;
        } // end Zero BegTang Speed branch

      // pole check locals
      SmSurfParamType eSingularDir = SM_SP_UNKNOWN ; // SM_SP_U = degenerate V isoParameter Curve.
                                                     //           where Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                     // SM_SP_V = degenerate U isoParameter Curve.
                                                     //           where Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                                                     // SM_SP_BOTH = surface is singular in both directions at UVTestPt.
                                                     // SM_SP_NEITHER = surface is not singular at UVTestPt.
      // IsBegVertexUV on a Surface Singularity
      rUVSectorIO.m_bVertexOnPole = pSurface->IsSingularity(rUVSectorIO.m_sBegEdgeUVPoint, eSingularDir, SM_APPROX_TOL_3D) ;
      if(rUVSectorIO.m_bVertexOnPole)
        {
#ifdef SM_DEBUG_CODE
// ULONG GWC_CHANGE_NEXT_LINE_TO_ZERO_AND_REMOVE_STATIC_VARIABLE ;
ULONG lDebugCntTgt = 0 ; 
/* static */ ULONG lDebugCnt = 1 ;
lDebugCnt++ ;
      if(bDebugMe || lDebugCntTgt == lDebugCnt)
        {
          SmBrep *pBrep = GetBrep() ;
          SmExtent2d  sSurfaceDomain ;
          if(pSurface) sSurfaceDomain = pSurface->GetNaturalUVDomain() ;
          //SmSurfParamType eSingularDirection ;
          //SmBoolean bOnSingularity = pSurface->IsSingularity(rSectorUVPoint, eSingularDirection) ;
          //SmBoolean bOnClosed      = pSurface->IsOnSeam(rSectorUVPoint) ; 
          // SmBoolean bOnFlatCorner  = pSurface->IsFlatCorner(rSectorUVPoint) ;

          SM_DUMP_AND_ASSERT_VALID(pBegCurve) ;
          SM_DUMP_AND_ASSERT_VALID(pEndCurve) ;  
          SM_DUMP_AND_ASSERT_VALID(pSurface) ;

          smgfx_Erase() ;
          smgfx_SetLook( 1,2,  0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
          smgfx_SetLook( 2,3,  0, 1, 0) ; this->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook( 1,2,  0, 0, 0 ); if(pFace)    { pFace->DrawUV(); sm_GraphicsLoop(); }
          smgfx_SetLook( 1,2,  1, 1, 0 ); if(pSurface) { pSurface->DrawUV( 5, 5 ); sm_GraphicsLoop(); }
          smgfx_SetLook( 3,4, .3,.3,.3 ); if(pSurface) { pSurface->DrawSeams(); sm_GraphicsLoop(); }
          smgfx_SetLook( 5,6, .8,.4,.2 ); if(pSurface) { pSurface->DrawPoles(); sm_GraphicsLoop(); }
          smgfx_SetLook( 3,4,  1, 0, 0 ); if(pBegEdge) { pBegEdge->DrawParams(); sm_GraphicsLoop(); }
          smgfx_SetLook( 4,5,  1, 0, 1 ); if(pEndEdge) { pEndEdge->DrawParams(); sm_GraphicsLoop(); }
          smgfx_SetLook( 6,7,  0, 0, 0) ; sBegEdgePV[1].Draw(&sBegEdgePV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook( 6,7,  1, 0, 0) ; sEndEdgePV[1].Draw(&sEndEdgePV[0]) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

          // Find DegenParam for BegEdge
          pSurface->FindDegenParamForDirection(rUVSectorIO.m_sBegEdgeUVPoint, // i/o: a point on the surface singularity                                       
                                               sBegEdgePV[1],                 // in : the direction to match with the NonSingular Surface tangent               
                                               TRUE,                          // in : TRUE  = Dir to match is from the pole heading out, as in starting a curve 
                                                                              //    : FALSE = Dir to match ends at the pole heading in, as in ending a curve    
                                               eSingularDir,                  // in : oneof SM_SP_U: Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                                              //    :       SM_SP_V: Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                                               SM_APPROX_TOL_3D) ;            // in : a loose tolerance; will try for a tight one.                              

          // Find DegenParam for EndEdge
          pSurface->FindDegenParamForDirection(rUVSectorIO.m_sEndEdgeUVPoint, // i/o: a point on the surface singularity                                       
                                               sEndEdgePV[1],                 // in : the direction to match with the NonSingular Surface tangent               
                                               TRUE,                          // in : TRUE  = Dir to match is from the pole heading out, as in starting a curve 
                                                                              //    : FALSE = Dir to match ends at the pole heading in, as in ending a curve    
                                               eSingularDir,                  // in : oneof SM_SP_U: Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                                              //    :       SM_SP_V: Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                                               SM_APPROX_TOL_3D) ;            // in : a loose tolerance; will try for a tight one.                              

        } // end Vertex on Pole check

      // Is BegVertexUV on a Surface FlatCorner (control points moved so TanU is parallel to TanV at corner)
      //   We have to check here because Flat corners cause upcoming DropVectors() calls to fail
      SmBoolean bOnFlatCorner  = pSurface->IsFlatCorner(rUVSectorIO.m_sBegEdgeUVPoint) ;

      if(!bOnFlatCorner)
        {
          // project BegEdge Tangent Vec down to UVPlane Corner and unitize
          pSurface->DropVectors(rUVSectorIO.m_sBegEdgeUVPoint, TRUE, TRUE, 1, &sBegEdgePV[1], &rUVSectorIO.m_sBegEdgeUVTan) ;
          rUVSectorIO.m_sBegEdgeUVTan.Unitize() ;

          // project EndEdge Tangent Vec down to UVPlane Corner and unitize
          pSurface->DropVectors(rUVSectorIO.m_sEndEdgeUVPoint, TRUE, TRUE, 1, &sEndEdgePV[1], &rUVSectorIO.m_sEndEdgeUVTan) ;
          rUVSectorIO.m_sEndEdgeUVTan.Unitize() ;
        } // end not on a flatCorner branch
      else // on a FlatCorner - look for special case of Edges running along SurfaceBoundaries
        {
          // Surface Eval
          SmVector3d sPoint, sDU, sDV ;
          pSurface->Evaluate1stDerivatives(rUVSectorIO.m_sBegEdgeUVPoint, TRUE, TRUE, sPoint, sDU, sDV) ; // always calculates NonZeroTangents

          // as needed negate sDU and sDV to point into the surface knowing rBedEdgeUVPoint is in a surface corner
          SmPoint2d sMidUV = sUVDomain.GetMid() ;
          SmPoint2d sUDir(1,0) ;
          SmPoint2d sVDir(0,1) ;
          SmBoolean bBeg_DUParallel, bAntiParallel ;
          SmBoolean bBeg_DVParallel ;
          SmBoolean bEnd_DUParallel ;
          SmBoolean bEnd_DVParallel ;
          if(sMidUV.x < rUVSectorIO.m_sBegEdgeUVPoint.x) { sDU = -sDU ; sUDir.Set(-1, 0) ; }
          if(sMidUV.y < rUVSectorIO.m_sBegEdgeUVPoint.y) { sDV = -sDV ; sVDir.Set( 0,-1) ; }

          // Expect Beg and End Edge tangents to align one each with sDU and sDV
          bBeg_DUParallel = (sBegEdgePV[1].IsParallelTo(sDU, SM_EFF_ZERO_DEG, &bAntiParallel)) && (bAntiParallel == FALSE) ;
          bBeg_DVParallel = (sBegEdgePV[1].IsParallelTo(sDV, SM_EFF_ZERO_DEG, &bAntiParallel)) && (bAntiParallel == FALSE) ;

          bEnd_DUParallel = (sEndEdgePV[1].IsParallelTo(sDU, SM_EFF_ZERO_DEG, &bAntiParallel)) && (bAntiParallel == FALSE) ;
          bEnd_DVParallel = (sEndEdgePV[1].IsParallelTo(sDV, SM_EFF_ZERO_DEG, &bAntiParallel)) && (bAntiParallel == FALSE) ;

          // Set Beg and End EdgeUVTan based on who lines up with whom (2 of the 4 possible Parallel booleans have to be true)
          SM_ASSERT_MSG((bBeg_DUParallel != bBeg_DVParallel) && (bEnd_DUParallel != bEnd_DVParallel) && (bBeg_DUParallel != bEnd_DUParallel),
                        _T("SmVertexuse::ComputeUVSector - Flat corner assumption that Beg and End Edge Tangents line up with Surf Du and DV tangents is false - an extension is needed here.")) ;

          if(bBeg_DUParallel) { rUVSectorIO.m_sBegEdgeUVTan = sUDir ; }
          if(bBeg_DVParallel) { rUVSectorIO.m_sBegEdgeUVTan = sVDir ; }
                                 
          if(bEnd_DUParallel) { rUVSectorIO.m_sEndEdgeUVTan = sUDir ; }
          if(bEnd_DVParallel) { rUVSectorIO.m_sEndEdgeUVTan = sVDir ; }

        } // end on a FlatCorner branch

        // arrive here when
        //  rBegSectorUV = vector from vertex into face from pBegEdgeuse = this->Edgeuse    
        //  rEndSectorUV = vector from vertex into face from pBegEdgeuse = this->Edgeuse->GetCornerMateEdgeuse(pVertex)
        //
        // when Loopuse->Orientation == SAME,     m_dSectorAngleDeg = Ang from rBegSectorUV to rEndSectorUV
        // when Loopuse->Orientation == OPPOSITE, m_dSectorAngleDeg = Compliment(m_dSectorAngleDeg) = Ang from rEndSectorUV to rBegSectorUV
        //
        // When faceuse->Orientation == SAME,
        //      faceuse->Orientation == OPPOSITE,

        // count Opposite orientations
        ULONG lOppositeCount =   pFaceuse->GetOrientation() == SM_OT_OPPOSITE ? 1 : 0
                               + pLoopuse->GetOrientation() == SM_OT_OPPOSITE ? 1 : 0 ;

      // UVSector AngDeg in SurfNormal coordinates
      if(     lOppositeCount == 0 
          ||  lOppositeCount == 2)   { rUVSectorIO.m_sBegEdgeUVTan.CCWAngleBetween(rUVSectorIO.m_sEndEdgeUVTan,dSectorAngRad) ; }
      else /* lOppositeCount == 1 */ { rUVSectorIO.m_sEndEdgeUVTan.CCWAngleBetween(rUVSectorIO.m_sBegEdgeUVTan,dSectorAngRad) ; }

          rUVSectorIO.m_dSectorAngDeg = SM_RAD2DEG(dSectorAngRad) ;

      // seam-ending-at-a-pole and Crack on face special cases - but not a single-edge closed-loop, eg. a circle.
      //   A closed sphere has one edge running from pole to pole on the seam being the
      //   natural trim boundaries for the face. This loop is expected to be an outerloop
      //   with orientation SM_OT_SAME.
      //     At those PoleVertices the rEndSectorUVTan and rBegSectorUVTan will be the same.
      //     The m_dSectorAngDeg returned by SmVector2d::CCWAngleBetween() will be 0.0 deg.
      //       When the seam loop is oriented SM_OT_SAME the m_dSectorAngDeg = 360.0 deg
      //       When the seam loop is oriented SM_OT_OPPOSITE the m_dSectorAngDeg = 0.0 deg (this is a database error needing fixing in the healer)
      //   A crack edge modeled within a face will have one edge or sequence of edges running from strut end to strut end in the middle of the face
      //     At those StrutVertices the rEndSectorUVTan and rBegSectorUVTan will be the same. The crack edge
      //     should be associated with a inner loop whose orientation is SM_OT_OPPOSITE.
      //       When the crack loop is oriented SM_OT_OPPOSITE the m_dSectorAngDeg = 360.0 deg.
      //       When the crack loop is oriented SM_OT_SAME the sectorAngDeg = 0.0 (this is a database error neegin fixing in the healer)
      //     note a crack on sphere can start and/or stop on a pole or a seam but the crack can not run along the natural surface boundary.
      // Circles and any single-edge closed-loop:  pBegEdgeuse->GetEdge() == pEndEdgeuse->GetEdge() is true and the m_dSectorAngDeg can be anything
      //      as already computed in m_dSectorAngDeg.  So, skip this special case fix.
      if(    rUVSectorIO.m_pBegEdgeuse            != rUVSectorIO.m_pEndEdgeuse             // true for struts and seam-ending-at-a-pole special cases, false for single-edge closed-loops
          && rUVSectorIO.m_pBegEdgeuse->GetEdge() == rUVSectorIO.m_pEndEdgeuse->GetEdge()) // true for struts and seam-ending-at-a-pole special cases, and single-edge closed-loops
        {
          SM_ASSERT_MSG(SM_IS_ZERO(rUVSectorIO.m_dSectorAngDeg), _T("SmVertexuse::ComputeUVSector: m_dSectorAngDeg is not zero when sampling a vertex at the end of a strut.  It should be. Case needs review.")) ;
          SM_ASSERT_MSG(   (   pLoopuse->GetOrientation() == SM_OT_OPPOSITE )
                        || (   pLoopuse->GetOrientation() == SM_OT_SAME
                            && rUVSectorIO.m_pBegEdgeuse->GetEdge()->IsSeam()),
                        _T("SmVertexuse:ComputeUVSector: special case is not an InnerLoop strut-end vertex or an OuterLoop Seam-end on-a-pole vertex as expected.  Case needs review")) ;

          // The vertex AngleSector should be 360.0 meaning that any direction from the vertex runs into the face.
          rUVSectorIO.m_dSectorAngDeg = 360.0 ;
        } // end a Vertex is a StrutEnd or a PoleVertex on-a-Seam special case check

      // begin scope: set additional output
        {
          // pOptIO->m_pBegEdgeuse     = pBegEdgeuse ;
          // pOptIO->m_pEndEdgeuse     = pEndEdgeuse ;
          // pOptIO->m_sBegEdgeUVPoint = rSectorUVPoint ;
          // already set in each branch // pOptIO->dBegEdgeParam   = dBegEdgeParam ;
          // already set in each branch // pOptIO->dEndEdgeParam   = dEndEdgeParam ;
          // already set in each branch // pOptIO->sEndEdgeUVPoint = sEndEdgeUVPoint ;

          // locals for Singularity and Closed checks
          SmSurfParamType     eSeamDir       = SM_SP_UNKNOWN ;
          //SmSurfParamType     eClosedParam   = SM_SP_UNKNOWN ;
          SmExtent2d          sSurfaceDomain = pSurface->GetNaturalUVDomain() ;
          SmTArray<SmPoint2d> sCrossSeamUVs ;

          // Is BegVertexUV on a Surface->Seam
          //     eSeamDir = oneof: SM_SP_NEITHER
          //                       SM_SP_U [check Pos[Umin,v] == Pos[Umax,v] for v samples]
          //                       SM_SP_V [check Pos[u,Vmin] == Pos[u,Vmax] for u samples]
          //                       SM_SP_BOTH
          SmBoolean bIsOnSeam  = pSurface->IsOnSeam(rUVSectorIO.m_sBegEdgeUVPoint, NULL, &eSeamDir, &sCrossSeamUVs) ;

          // Remember seams BegVertexUV is upon
          rUVSectorIO.m_bVertexOnClosedU     = bIsOnSeam && (eSeamDir == SM_SP_U || eSeamDir == SM_SP_BOTH) ;  

          rUVSectorIO.m_bVertexOnClosedV     = bIsOnSeam && (eSeamDir == SM_SP_V || eSeamDir == SM_SP_BOTH) ;

          // Is BegVertexUV on a Surface FlatCorner (control points moved so TanU is parallel to TanV at corner)
          //SmBoolean bOnFlatCorner  = pSurface->IsFlatCorner(rSectorUVPoint) ;


          // When Vertex is on a SurfaceSeam - check for Missing SeamEdge
          if(bIsOnSeam == TRUE)
            { 
              // note when Sector includes IsoParamU and/or IsoParamV lines 
              //   * Not every contained IsoParam Line will be a seam 
              //      - usually just one or the other (since VertexUV is known to be on a seam) except for tori
              //   * Commonly a Tangent will be coincident to a IsoParam line - 
              //       watch for tols causing sectors bounded by a IsoParam line being classified as containing a IsoParam Line.
              //   * Check for IsoParam containment in UVSpace
              //   * Check for Tangent parallel to IsoParam dir in 3dSpace
              //
              //  /*                       ^ - EndEdgeUVTan        */  /*                       ^ - BegEdgeUVTan    */
              //  /*                     /                         */  /*                     /                     */
              //  /*                   /                           */  /*                   /                       */
              //  /*                 /                             */  /*                 /                         */
              //  /*    Vertexuse -@ -------                       */  /*    Vertexuse -@                           */
              //  /*        in       \                             */  /*        in     | \                         */
              //  /*      UVSpace      \                           */  /*      UVSpace  |   \                       */
              //  /*                     \                         */  /*               |     \                     */
              //  /*                       v - BegEdgeUVTan        */  /*               |       v - EndEdgeUVTan    */
              //  /*                                               */  /*                                           */
              //  /*    Tangents on different sides of IsoU line   */  /*   Tangents on same sides of IsoV lines    */
              //
              //SmBoolean bHasIsoParamU =    (rBegSectorUVTan.x * rEndSectorUVTan.x) < 0.0                         // tangents on different sides of IsoU lines
              //                         && (smos_Fabs(rBegSectorUVTan.y) < smos_Fabs(rBegSectorUVTan.x * 100.))   // BegATan(y,x) > 0.5 degrees (not a IsoU line direction)
              //                         && (smos_Fabs(rEndSectorUVTan.y) < smos_Fabs(rEndSectorUVTan.x * 100.)) ; // EndATan(y,x) > 0.5 degrees (not a IsoU line direction)
              //SmBoolean bHasIsoParamV =    (rBegSectorUVTan.y * rEndSectorUVTan.y) < 0.0                         // tangents on different sides of IsoV lines
              //                         && (smos_Fabs(rBegSectorUVTan.x) < smos_Fabs(rBegSectorUVTan.y * 100.))   // BegATan(x,y) > 0.5 degrees (not a IsoV line direction)
              //                         && (smos_Fabs(rEndSectorUVTan.x) < smos_Fabs(rEndSectorUVTan.y * 100.)) ; // EndATan(x,y) > 0.5 degrees (not a IsoV line direction)

              // Prior test didn't work with [B651]
              // Vast scale differences in UV meant clear seam cross didn't satisfy ATan(x,y) > 0.5
              SmBoolean bHasIsoParamU = FALSE ;
              SmBoolean bHasIsoParamV = FALSE;
              SmPoint3d sBeg3d, sEnd3d;
              SmVector3d sBegDU, sBegDV, sEndDU, sEndDV;
              pFace->GetSurface()->Evaluate1stDerivatives( rUVSectorIO.m_sBegEdgeUVPoint, TRUE, TRUE, sBeg3d, sBegDU, sBegDV ); // rSectorUVPoint was rStartEdgeUVPoint
              pFace->GetSurface()->Evaluate1stDerivatives( rUVSectorIO.m_sEndEdgeUVPoint, TRUE, TRUE, sEnd3d, sEndDU, sEndDV );
              double dRadLimit = .5 / 180. * SM_PI;
              
              if ( eSeamDir == SM_SP_U || eSeamDir == SM_SP_BOTH )
              {
                  double dAng1, dAng2;
                  sBegEdgePV[1].AngleBetween( sBegDV, dAng1 );
                  sEndEdgePV[1].AngleBetween( sEndDV, dAng2 );

                  bHasIsoParamU = ( ( rUVSectorIO.m_sBegEdgeUVTan.x * rUVSectorIO.m_sEndEdgeUVTan.x ) < 0.0 )
                      && ( dAng1 > dRadLimit ) && ( dAng1 < SM_PI - dRadLimit )
                      && ( dAng2 > dRadLimit ) && ( dAng2 < SM_PI - dRadLimit );
              }
              if ( eSeamDir == SM_SP_V || eSeamDir == SM_SP_BOTH )
              {
                  double dAng1, dAng2;
                  sBegEdgePV[1].AngleBetween( sBegDU, dAng1 );
                  sEndEdgePV[1].AngleBetween( sEndDU, dAng2 );

                  bHasIsoParamV = ( ( rUVSectorIO.m_sBegEdgeUVTan.y * rUVSectorIO.m_sEndEdgeUVTan.y ) < 0.0 )
                      && ( dAng1 > dRadLimit ) && ( dAng1 < SM_PI - dRadLimit )
                      && ( dAng2 > dRadLimit ) && ( dAng2 < SM_PI - dRadLimit );
              }


               // sector contains a seam 
               //   when VertexUV is on a Seam
               //   and  that Seam's IsoParam Line dir is contained within the sector. 
              //          eSeamDir == SM_SP_U [check Pos[Umin,v] == Pos[Umax,v] for v samples]
              //          eSeamDir == SM_SP_V [check Pos[u,Vmin] == Pos[u,Vmax] for u samples]
               rUVSectorIO.m_bSectorMissingSeamU = bIsOnSeam && (eSeamDir == SM_SP_U || eSeamDir == SM_SP_BOTH) && bHasIsoParamU ;
               rUVSectorIO.m_bSectorMissingSeamV = bIsOnSeam && (eSeamDir == SM_SP_V || eSeamDir == SM_SP_BOTH) && bHasIsoParamV ;

             } // end Vertex is on a SurfaceSeam check - so look for missing SeamEdge
        } // end scope - fill in addition SectorData

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          Dump() ;
          if(pSurface) pSurface->Dump() ;

          SmBrep    * pBrep    =  pVertex ? pVertex->GetBrep()
                                : pFace   ? pFace->GetBrep()
                                          : NULL ;
          SmVector3d sBegEdgeBasePoint, sEndEdgeBasePoint, sSurfPoint, sSurfNormal, sFaceNormal ;
          pBegCurve->EvaluatePoint(rUVSectorIO.m_dBegEdgeParam, sBegEdgeBasePoint) ;
          pEndCurve->EvaluatePoint(rUVSectorIO.m_dEndEdgeParam, sEndEdgeBasePoint) ;
          pSurface->EvaluatePoint(rUVSectorIO.m_sBegEdgeUVPoint, sSurfPoint) ;
          pSurface->EvaluateNormal(rUVSectorIO.m_sBegEdgeUVPoint, TRUE, TRUE, sSurfNormal) ;
          sFaceNormal = (pFaceuse->GetOrientation() == SM_OT_SAME) ? sSurfNormal : -sSurfNormal ;
          smgfx_Erase() ;
          smgfx_SetLook(1,2,  0, 0, 1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7,  1, 0, 0) ; if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2,  0, 0, 0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH,7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2,  0, 1, 2) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .2,.2,.2) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawPoles() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .8,.2,.8) ; if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5,  1, 0, 0) ; sSurfNormal.Draw(&sSurfPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7,  0, 1, 1) ; sFaceNormal.Draw(&sSurfPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7,  0, 1, 0) ; sBegEdgePV[1].Draw(&sBegEdgeBasePoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(6,7,  0, 0, 1) ; sEndEdgePV[1].Draw(&sEndEdgeBasePoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3,  0, 1, 0) ; if(pLoopuse) pLoopuse->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4,  0, 1, 0) ; if(pBegCurve) pBegCurve->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4,  0, 0, 1) ; if(pEndCurve) pEndCurve->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5,  0, 1, 0) ; if(rUVSectorIO.m_pBegEdgeuse) rUVSectorIO.m_pBegEdgeuse->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5,  0, 0, 1) ; if(rUVSectorIO.m_pEndEdgeuse) rUVSectorIO.m_pEndEdgeuse->Draw() ; sm_GraphicsLoop() ;

          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // all done
      return(SM_SUCCESS) ;
    } // end case EdgeVertexuse

  else // vertexuse is an uninitialized state
    {
      return(SM_ERR) ;
    }

  // never reach here
  return(SM_ERR) ;

} // end SmVertexuse::ComputeUVSector

/*******************************************************************//**
PURPOSE: Get turning AngDeg from Beg to End of this Edgeuse accounting
         for orientation.

NOTES: 1. return AngDeg range:[-180 180]
       returns   0   Deg for parallel Beg and End 1st deriv vectors
               + 180 Deg for antiparallel 1st derivs
       2. TurningAngDeg = 180 - SectorAngDeg ;
       3. Gets SectorAngDeg from ComputeUVSector().
***********************************************************************/
double SmVertexuse::GetTurningAngDeg() const
{ 
  // locals
  double      dTurnAngDeg ;
  SmBoolean   bOKToMakeUVTrimCurve=TRUE ; // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve

  // sector properties
  SmUVSectorIO sUVSectorIO ;

  // Sector eval
  ComputeUVSector(sUVSectorIO,             // out: Optional Sector Properties. NULL to ignore, default:[NULL]
                  bOKToMakeUVTrimCurve) ;  // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve
                                           //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve, default:[TRUE]
  /*
   NoTurn:    InsideFace
                ^               ^
    PrevVU      |               |          SectorAngDeg  = 180
       @________+....@__________+....@     TurningAngDeg = 0
        CWEdgeuse   VU  Edgeuse   NextVU   
             End  <-- --> Start            TurningAng + SectorAng = 180
             Sector     Sector
        OutsideFace
                            @ NextVU
                           .
                          . 
   LeftTurn:         <-- +         OutsideFace
              InsideFace/ Edgeuse
                ^      /  ^ Start Sector
    PrevVU      |     /  /     SectorAngDeg  = 135   TurningAng + SectorAng = 180
       @________+....@  /      TurningAngDeg = 45
        CWEdgeuse   VU /
             End  <---
             Sector 
  
   RightTurn:
              InsideFace
                ^
    PrevVU      |     \
       @________+....@ \
        CWEdgeuse   VU\ \
       End Sector <--  \ v Start Sector
                Edgeuse +-->      SectorAngDeg  = 135   -TurningAng + 180 = SectorAng
                         .     TurningAngDeg = 45
              OutsideFace . 
                           @ NextVU
  */

  // turning angle
  dTurnAngDeg =  180 - sUVSectorIO.m_dSectorAngDeg ;    // left turns are positive turning angles

  // tolerance problem:  for strut edges with 180 degree turns, we should always be returning +180
  //                     for sliver corners we might want to retrun-179.9999 and 
  //                     for sliver corners with tol, tol might toggle Turning angle betwee + and - 180.
                                             
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // Draw Loop use in space
      SmBrep      * pBrep       = GetBrep() ;
      SmEdgeuse   * pEdgeuse    = GetEdgeuse() ;
      SmEdgeuse   * pCCWEdgeuse = pEdgeuse ? pEdgeuse->GetCCWEdgeuse() : NULL ; 
      SmEdgeuse   * pCWEdgeuse  = pEdgeuse ? pEdgeuse->GetCWEdgeuse() : NULL ;  

      smgfx_Erase() ; 
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(5,6, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ; // TRUE = add UVPlane graphics for loop on nearby plane
      smgfx_SetLook( 3, 4, 1, 0, 0 ); if(pEdgeuse) { pEdgeuse->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook( 3, 4, 0, 1, 0 ); if(pCCWEdgeuse) { pCCWEdgeuse->Draw(); sm_GraphicsLoop(); }
      smgfx_SetLook( 3, 4, 1, 0, 1 ); if(pCWEdgeuse) { pCWEdgeuse->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(dTurnAngDeg) ;

} // end SmVertexuse::GetTurningAngDeg

/*******************************************************************//**
PURPOSE: Get the parametric value of the vertexuse->Edgeuse->Edge EndPoint
  that is expected to connect to the vertexuse->vertex->point.

NOTES:
***********************************************************************/
SmStatus SmVertexuse::ComputeTPoint
  (double &dT)                          // out: Vertexuse->Edgeuse->Edge->EndParamValue
 const                                  //      expected to connect to vertexuse->Vertex->Point
{
  // for edgeuse type vertexuses
  if(IsEdgeVertexuse())
    {
      SmEdgeuse  *pEdgeuse = GetEdgeuse(); NER(pEdgeuse);
      SmEdge     *pEdge    = pEdgeuse->GetEdge() ;
      SmExtent1d  sIvl     = pEdge->GetInterval() ;

      // set output
      dT = (pEdgeuse->GetOrientation() == SM_OT_SAME) ? sIvl.GetMin()
                                                      : sIvl.GetMax() ;
    } // end IsEdgeVertexuse() check
  else // no TPoint to compute
    {
      SER(SM_ERR);
    } // end default case

  // all done
  return(SM_SUCCESS) ;

} // end SmVertexuse::ComputeTPoint

/*******************************************************************//**
PURPOSE: Get the edge of a vertexuse.

NOTES:
***********************************************************************/
SmEdgeuse * SmVertexuse::GetEdgeuse() const
{ SM_ASSERT(   (m_pSorLUorEU == NULL)
            || (IsShellVertexuse() && m_pSorLUorEU->IsKindOf(SmShell_TYPE))
            || (IsEdgeVertexuse()  && m_pSorLUorEU->IsKindOf(SmEdgeuse_TYPE))
            || (IsLoopVertexuse()  && m_pSorLUorEU->IsKindOf(SmLoopuse_TYPE))) ;

  return (  IsEdgeVertexuse() ? (SmEdgeuse*)m_pSorLUorEU : NULL ) ;

} // end SmVertexuse::GetEdgeuse

/*******************************************************************//**
PURPOSE: Get the faceuse associated with a vertexuse.

NOTES: returns SmFaceuse ptrs for SmLoopuse_TYPE and SmEdgeuse_TYPE Vertexuses
       returns NULL for SM_Shell_TYPE vertexuses not connected to a Face
***********************************************************************/
SmFaceuse * SmVertexuse::GetFaceuse() const
{
  // no face for shell vertices
  if (m_tVertexuseType == SmShell_TYPE)
    { /* gwc: removed - SE(SM_ERR); */ return NULL; }

  // for Edgeuse_type - return vertexuse->edgeuse->faceuse
  else if (m_tVertexuseType == SmEdgeuse_TYPE)
    {
      SmEdgeuse *pEUofVU = (SmEdgeuse*)m_pSorLUorEU;
      NERN(pEUofVU);
      return pEUofVU->GetFaceuse();
    }

  // for Loopuse_type - return vertexuse->loopuse->faceuse
  else if (m_tVertexuseType == SmLoopuse_TYPE)
    {
      SmLoopuse *pLU = (SmLoopuse*)m_pSorLUorEU;
      return pLU->GetFaceuse();
    }

  // error: unknown m_tVertexuseType
  SE(SM_ERR);
  return NULL;

} // end SmVertexuse::GetFaceuse

/*******************************************************************//**
PURPOSE: Get the shell of a vertexuse.

NOTES: The output value will always be found unless an error occurs.
***********************************************************************/
SmShell * SmVertexuse::GetShell() const
{
  // SmVertexuse->m_pSorLUorEU has a type that matches the value stored in m_pSorLUorEU
  SM_ASSERT(   (m_pSorLUorEU == NULL)
            || (m_tVertexuseType == SmEdgeuse_TYPE && m_pSorLUorEU->IsKindOf(SmEdgeuse_TYPE))
            || (m_tVertexuseType == SmLoopuse_TYPE && m_pSorLUorEU->IsKindOf(SmLoopuse_TYPE))
            || (m_tVertexuseType == SmShell_TYPE   && m_pSorLUorEU->IsKindOf(SmShell_TYPE))) ;

  // no work - no m_pSorLUorEU
  if(m_pSorLUorEU == NULL)
    { return (NULL); }

    SmShell *pShell = NULL;
  if (m_tVertexuseType == SmShell_TYPE)
    { pShell = (SmShell*)m_pSorLUorEU; }

  else if (m_tVertexuseType == SmEdgeuse_TYPE)
    {
      pShell = ((SmEdgeuse*)m_pSorLUorEU)->GetShell();
    }
  else if (m_tVertexuseType == SmLoopuse_TYPE)
    {
      SmFaceuse *pFU = ((SmLoopuse*)m_pSorLUorEU)->GetFaceuse();
        SM_ASSERT(pFU != NULL);
        pShell = pFU->GetShell();
    }

    SM_ASSERT(pShell != NULL);
    return pShell;

} // end SmVertexuse::GetShell

/*******************************************************************//**
PURPOSE: Get the Brep of a vertexuse.

NOTES: The output value will always be found unless an error occurs.
***********************************************************************/
SmBrep * SmVertexuse::GetBrep() const
{
  // SmVertexuse->m_pSorLUorEU has a type that matches the value stored in m_pSorLUorEU
  SM_ASSERT(   (m_pSorLUorEU == NULL)
            || (m_tVertexuseType == SmEdgeuse_TYPE && m_pSorLUorEU->IsKindOf(SmEdgeuse_TYPE))
            || (m_tVertexuseType == SmLoopuse_TYPE && m_pSorLUorEU->IsKindOf(SmLoopuse_TYPE))
            || (m_tVertexuseType == SmShell_TYPE   && m_pSorLUorEU->IsKindOf(SmShell_TYPE))) ;

  // no work - no m_pSorLUorEU
  if(m_pSorLUorEU == NULL)
    { return (NULL); }

  // chase pointers
  SmShell * pShell = GetShell() ;
  SmBrep  * pBrep  = pShell ? pShell->GetBrep() : NULL ;

  // all done
  return(pBrep) ;

} // end SmVertexuse::GetBrep

/*******************************************************************//**
PURPOSE: return TRUE when Vertex has a VertexEdge Gap

NOTES: No VertexEdge gaps when Vertexuse does not connect to an Edge with a curve
***********************************************************************/
SmBoolean SmVertexuse::HasVertexEdgeGap() const
{
  // No VertexEdgeGap when vertexuse does not connect to an edge with a curve
  if(   IsLoopVertexuse()
     || IsShellVertexuse()
     || GetEdgeuse() == NULL
     || GetEdgeuse()->GetEdge() == NULL
     || GetEdgeuse()->GetEdge()->GetCurve() == NULL) // for FilletVertices during Filleting
    { return( FALSE ) ; }

  return(TRUE) ;

} // end SmVertexuse::HasVertexEdgeGap

/*******************************************************************//**
PURPOSE: return TRUE when Vertex has a VertexFaceTrimCurve gap

NOTES: No VertexFaceTrimCurve gap when Vertexuse does not connect
       to an Edge that is connected to a Face
***********************************************************************/
SmBoolean SmVertexuse::HasVertexFaceTrimCurveGap() const
{
  SmEdgeuse *pEdgeuse = GetEdgeuse() ;

  // when vertexuse does not connect to an edgeuse connected to a face
  if(   IsLoopVertexuse()
     || IsShellVertexuse()
     || pEdgeuse == NULL
     || pEdgeuse->GetFace() == NULL
     || pEdgeuse->GetFace()->GetSurface() == NULL) // for FilletVertices during Filleting
    {
      // there is no gap
      return( FALSE ) ;
    }

  return(TRUE) ;

} // end SmVertexuse::HasVertexFaceTrimCurveGap

/*******************************************************************//**
PURPOSE: return TRUE when Vertex has a VertexFace gap

NOTES: No VertexFaceTrimCurve gap when Vertexuse does not connect
       to an Edge that is connected to a Face
***********************************************************************/
SmBoolean SmVertexuse::HasVertexFaceGap() const
{

  // when vertexuse does not connect to a face
  if( IsShellVertexuse()
     || (   IsKindOf(SmFilletVertex_TYPE)
         && (   GetFaceuse() == NULL
             || GetFaceuse()->GetFace() == NULL
             || GetFaceuse()->GetFace()->GetSurface() == NULL)))  // happens for SmFilletVertex during filleting
    {
      // No face: there is no gap.
      return( FALSE ) ;
    }

  // when vertexuse connects to a wire edge
  if ( IsEdgeVertexuse() || IsStrutVertexuse() )
    {
      // See if it connects to a wire edge.
      SmEdgeuse * pEU   = GetEdgeuse();
      SmEdge    * pEdge = pEU ? pEU->GetEdge() : NULL ;
      if ( pEdge && pEdge->IsWire() )
        {
          // No face for wire edge
          return( FALSE ) ;
        }
    }

  return(TRUE) ;

} // end SmVertexuse::HasVertexFaceGap

/*******************************************************************//**
PURPOSE: Check Vertex/Edge gap cached value against a freshly calculated one.

NOTES: return TRUE when they are the same, FALSE when they are the different
***********************************************************************/
SmBoolean SmVertexuse::IsVertexEdgeGapFresh
 (SmVertexEdgeGap & rVertexEdgeGap,       // out: calculated Vertex/Edge Gap
  SmVertexEdgeGap & rStoredVertexEdgeGap) // out: stored Vertex/Edge Gap
 const
{
  SmBoolean bRtn = TRUE ;

  if(IsVertexEdgeGapInit())
    {
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this->GetContext()) ;

      // calculate the gap without changing the cache
      ((SmVertexuse *)this)->CalcVertexEdgeGap( &rVertexEdgeGap, FALSE) ; // FALSE = don't change chache

      // check that fresh gap == cached gap
      bRtn = SM_ARE_SAME_TO_TOL(rVertexEdgeGap, m_sVertexEdgeGap3d, sXSectTol3d/1000.0) ;

      // finish output
      rStoredVertexEdgeGap = m_sVertexEdgeGap3d ;
    }

  return(bRtn) ;

} // end SmVertexuse::IsVertexEdgeGapFresh

/*******************************************************************//**
PURPOSE: Check Vertex/Face gap cached value against a freshly calculated one.

NOTES: return TRUE when they are the same, FALSE when they are the different
***********************************************************************/
SmBoolean SmVertexuse::IsVertexFaceGapFresh
 (SmVertexFaceGap & rVertexFaceGap,       // out: calculated Vertex/Face Gap
  SmVertexFaceGap & rStoredVertexFaceGap) // out: stored Vertex/Face Gap

 const
{
  SmBoolean bRtn = TRUE ;

  if(IsVertexFaceGapInit())
    {
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this->GetContext()) ;

      // calculate the gap without changing the cache
      ((SmVertexuse *)this)->CalcVertexFaceGap( &rVertexFaceGap, FALSE) ;

      // check that fresh gap == cached gap
      bRtn = SM_ARE_SAME_TO_TOL(rVertexFaceGap, m_sVertexFaceGap3d, sXSectTol3d/1000.0) ;

      // finish output
      rStoredVertexFaceGap = m_sVertexFaceGap3d ;
    }

  return(bRtn) ;

} // end SmVertexuse::IsVertexFaceGapFresh

/*******************************************************************//**
PURPOSE: Return True when Vertexuse is connected to given Target Topology object

NOTES: 1. the input Target cpConnectTgt may be NULL, or of type,
             SmVertex,
             SmEdge,
             SmLoop,
             SmFace,
             SmShell,
             SmRegion
        2. returns TRUE for NULL and for any other unsupported Topology TYPE
***********************************************************************/
SmBoolean SmVertexuse::IsConnectedTo
  ( const SmTopology *cpConnectTgt )    // in : target Topology
 const
{
  // no work - no Topo
  if(cpConnectTgt == NULL) 
    { return TRUE ; }

  // switch on cpConnectTgt type
  switch(cpConnectTgt->GetType())
    {
      case SmVertexuse_TYPE: { return( this == (SmVertexuse *)cpConnectTgt ) ; 
                             } break ;
      case SmVertex_TYPE   : { return( GetVertex() == (SmVertex *)cpConnectTgt ) ; 
                             } break ;
      case SmEdge_TYPE     : { return( GetEdgeuse() && GetEdgeuse()->GetEdge() == (SmEdge *)cpConnectTgt ) ;
                             } break ;
      case SmEdgeuse_TYPE  : { return( this == ((SmEdgeuse *)cpConnectTgt)->GetVertexuse() ) ;
                             } break ;
      case SmFace_TYPE     : { return( GetFaceuse() && GetFaceuse()->GetFace() == (SmFace *)cpConnectTgt ) ;
                             } break ;
      case SmFaceuse_TYPE  : { return( GetFaceuse() == (SmFaceuse *)cpConnectTgt ) ;
                             } break ;
      case SmLoop_TYPE     : { return( GetLoopuse() && GetLoopuse()->GetLoop() == (SmLoop *)cpConnectTgt ) ;
                             } break ;
      case SmLoopuse_TYPE  : { return( GetLoopuse() == (SmLoopuse *)cpConnectTgt ) ;
                             } break ;
      case SmShell_TYPE    : { return( GetShell() == (SmShell *)cpConnectTgt ) ;
                             } break ;
      case SmRegion_TYPE   : { return( GetShell() && GetShell()->GetRegion() == (SmRegion *)cpConnectTgt ) ;
                             } break ;
      default: break ;

    } // end switch on type

  // arrive here when cpConnectTgt is an unsupported type - return TRUE
  return(TRUE) ;

} // end SmVertexuse::IsConnectedTo

/*******************************************************************//**
PURPOSE: return Vertex/Edge gap from Vertex->Point to edge endPoint
         when it exists, else returns NULL

NOTES: 1. vertexuse->Vertex to vertexuse->edgeuse->Edge gap
       2. returns NULL for a Vertex that is not connected to an Edge
***********************************************************************/
SmVertexEdgeGap * SmVertexuse::GetVertexEdgeGap
 (SmBoolean  bForceCalc)     // in : TRUE = force gap evaluation, FALSE=use cache if available
 const                       //      default:[FALSE]
{
  // no work - no gap
  if(HasVertexEdgeGap() == FALSE)
    { return NULL ; }

  // when asked - refresh the cache
  if( bForceCalc || !m_sVertexEdgeGap3d.IsInit())
    {
      // refresh the Vertex/Edge gap cache
      CalcVertexEdgeGap((SmVertexEdgeGap *)NULL, TRUE) ;
    }

  // all done
  return(&m_sVertexEdgeGap3d) ;

} // end SmVertexuse::GetVertexEdgeGap

/*******************************************************************//**
PURPOSE: return gap from Vertex->Point to Face->surface
         when Vertex is connected to Face, else returns NULL

NOTES: 1. vertexuse->Vertex to vertexuse->...->Face->DropPoint gap
       2. returns NULL when Vertex is not Connected to a Face
***********************************************************************/
SmVertexFaceGap * SmVertexuse::GetVertexFaceGap
 (SmBoolean bForceCalc)     // in : TRUE = force gap evaluation, FALSE=use cache if available
 const                      //      default:[FALSE]
{
  // no work - no Gap
  if(HasVertexFaceGap() == FALSE)
    { return NULL ; }

  // when asked - refresh the cache
  if( bForceCalc || !m_sVertexFaceGap3d.IsInit())
    {
      // refresh the Vertex/Face gap cache
      CalcVertexFaceGap(NULL, TRUE) ;
    }

  // all done
  return(&m_sVertexFaceGap3d) ;

} // end SmVertexuse::GetVertexFaceGap

/*******************************************************************//**
PURPOSE: compute and/or cache Vertex/Edge gap from Vertex->Point to edge endPoint

NOTES: 1. caches gap in m_sVertexEdgeGap3d.
       2. When vertexuse does not connect to an edge, sets gaps to uninit
***********************************************************************/
SmStatus SmVertexuse::CalcVertexEdgeGap
 (SmVertexEdgeGap * pOptVertexEdgeGap, // out: calculated Vertex/Edge gap, Must pt to SmGap obj to fill, NULL to ignore, default:[NULL]
  SmBoolean         bCache)            // in : TRUE=cache calculated Vertex/Edge gap, FALSE=no change to cache, default:[TRUE]
  const
{
  // init output
  if(pOptVertexEdgeGap)
    { pOptVertexEdgeGap->ReSet() ; }

  if ( ! IsEdgeVertexuse() )
    { return SM_SUCCESS; }

  // get all vertices connected to the same Edge
  ULONG ii ;
  SmEdgeuse * pEdgeuse = GetEdgeuse() ;
  SmEdge    * pEdge    = pEdgeuse ? pEdgeuse->GetEdge() : NULL ;
  SmVertex  * pVertex  = GetVertex() ;
  SmTArray<SmVertexuse *> sVertexuses ;
  pVertex->GetVertexusesOfEdge(pEdge, sVertexuses) ;

  // when asked - clear cache
  if(bCache)
    { m_sVertexEdgeGap3d.ReSet() ;
      // keep equivalent vertexuses synchronized
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        { sVertexuses[ii]->m_sVertexEdgeGap3d.ReSet() ; }
    }

  // no work - no VertexEdge Gap
  if(!HasVertexEdgeGap())
    { return( SM_SUCCESS ) ; }

  // get edge EndPoint and param value
  double      dEdgeT =  (pEdgeuse->GetOrientation() == SM_OT_SAME)
                       ? pEdgeuse->GetEdge()->GetInterval().GetMin()
                       : pEdgeuse->GetEdge()->GetInterval().GetMax() ;

  // set output - the calls to SmVertexEdgeGap::Set() assign values to m_pVertexuse and m_dEdgeT
  //              and then call SmVertexEdgeGap::CacheGap() to compute a m_dGap3d value
  if(pOptVertexEdgeGap) { pOptVertexEdgeGap->Set((SmVertexuse *)this, dEdgeT); }  // side effect: Gap values are set
  if(bCache)            { m_sVertexEdgeGap3d.Set((SmVertexuse *)this, dEdgeT);    // side effect: Gap values are set
                          // keep equivalent vertexuses synchronized
                          for(ii=0;ii<sVertexuses.GetSize();ii++)
                            { sVertexuses[ii]->m_sVertexEdgeGap3d = m_sVertexEdgeGap3d ; }
                        }

  // all done
  return( SM_SUCCESS );

} // end SmVertexuse::CalcVertexEdgeGap

/*******************************************************************//**
PURPOSE: find and return gap from Vertex->Point to edgeuse
         endPoint (projected through surface)

NOTES: when vertexuse does not connect to an edge connected to a face
       returns an uninitialized Gap
***********************************************************************/
SmVertexFaceTrimCurveGap SmVertexuse::GetVertexFaceTrimCurveGap // rtn: Gap data for Vertex/FaceTrimCurve gap
  (SmBSplineCurve * pOptUVTrimCurve)                            // in : for internal use only
                                                                //      default:[NULL]
 const
{
  // init output
  SmVertexFaceTrimCurveGap sVertexFaceTrimCurveGap ;

  // locals
  SmEdgeuse      * pEdgeuse     = GetEdgeuse() ;
  SmBSplineCurve * pUVTrimCurve = pOptUVTrimCurve ;

  AER_MSG( pEdgeuse != NULL, _T( "Corrupted database: vertexuse with no edgeuse" ) );

  if(pUVTrimCurve == NULL)
    {
      // Use GetUVTrimCurve, not GetOrCreateUVTrimCurve.  If not there, don't try to create,
      // the topology might not be in good shape (as during read-in).  [B513]
      pUVTrimCurve = pEdgeuse->GetUVTrimCurve() ;
    }

  // when vertexuse does not connect to an edge connected to a face
  if(!HasVertexFaceTrimCurveGap())
    {
      // report a problem and return an uninitialized gap
      WARN(_T("SmVertexuse::GetVertexFaceTrimCurveGap - Vertexuse does not connect a Vertex to a Face - uninit gap returned")) ;
      return( sVertexFaceTrimCurveGap ) ;
    }

  if ( pUVTrimCurve != NULL )
    { 
      // get param
      double dEdgeT = (pEdgeuse->GetOrientation() == SM_OT_SAME)
                     ? pUVTrimCurve->GetNaturalInterval().GetMin()
                     : pUVTrimCurve->GetNaturalInterval().GetMax() ;

      // set output - the call to SmVertexFaceTrimCurveGap::Set() assign values to m_pVertexuse and m_dEdgeT
      //              and then call SmVertexFaceGap::CacheGap() to compute a m_dGap3d value
      sVertexFaceTrimCurveGap.Set( (SmVertexuse *)this, dEdgeT);
    }

  // all done
  return( sVertexFaceTrimCurveGap ) ;

} // end SmVertexuse::GetVertexFaceTrimCurveGap

/*******************************************************************//**
PURPOSE: compute and cache Vertex/Face gap from Vertex->Point to
            Face->Surface.

NOTES: 1. caches gap in m_sVertexFaceGap3d.
       2. When vertexuse does not connect to an Face,
          caches gap of type SM_NO_VERTEX_FACE
***********************************************************************/
SmStatus SmVertexuse::CalcVertexFaceGap
 (SmVertexFaceGap * pOptVertexFaceGap, // out: calculated Vertex/Face gap, Must pt to SmGap obj to fill, NULL to ignore, default:[NULL]
  SmBoolean         bCache)            // in : TRUE=cache calculated Vertex/Face gap, FALSE=no change to cache, default:[TRUE]
 const
{
  // init output
  if(pOptVertexFaceGap)
    { pOptVertexFaceGap->ReSet() ; }

  // no work - has no Face/Vertex gap
  if (!HasVertexFaceGap())
    { return SM_SUCCESS; }

  // get all vertices connected to the same Face
  ULONG ii ;
  SmFaceuse * pFaceuse = GetFaceuse() ;
  SmFace    * pFace    = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmVertex  * pVertex  = GetVertex() ;
  SmTArray<SmVertexuse *> sVertexuses ;
  pVertex->GetVertexusesOfFace(pFace, sVertexuses) ;

  // when asked - clear cache
  if(bCache)
    {
      m_sVertexFaceGap3d.ReSet() ;
      // keep equivalent vertexuses synchronized
      for(ii=0;ii<sVertexuses.GetSize();ii++)
        { sVertexuses[ii]->m_sVertexFaceGap3d.ReSet() ; }
    }

  SmPoint2d sUV ;
  SmStatus  sRtn = SM_SUCCESS ;

  // check state
  SM_ASSERT(   GetFaceuse() != NULL
            && GetFaceuse()->GetFace() != NULL
            && GetFaceuse()->GetFace()->GetSurface() != NULL) ;

  // project vertex to vertexuse->Face->Surface
  SER(sRtn = ComputeUVPoint(sUV, FALSE)) ; // FALSE = Use GlobalSolve without touching UVTrimCurve

  // set output - the calls to SmVertexFaceGap::Set() assign values to m_pVertexuse and m_sFaceUV
  //              and then call SmVertexFaceGap::CacheGap() to compute a m_dGap3d value
  if(pOptVertexFaceGap) { pOptVertexFaceGap->Set((SmVertexuse *)this, &sUV) ; }  // side effect: sets pOptVertexFaceGap::m_dGap3d
  if(bCache)            { m_sVertexFaceGap3d.Set((SmVertexuse *)this, &sUV) ;    // side effect: sets m_sVertexFaceGap3d::m_dGap3d
                          // keep equivalent vertexuses synchronized 
                          // Vertexuses aren't equivalent. Across a seam we have jumps in sUV [B678]
                          //for(ii=0;ii<sVertexuses.GetSize();ii++)
                          //  { sVertexuses[ii]->m_sVertexFaceGap3d = m_sVertexFaceGap3d ; }

                        }
  // all done
  return(sRtn) ;

} // end SmVertexuse::CalcVertexFaceGap

/*******************************************************************//**
PURPOSE: OBSOLETE methods scheduled for removal

NOTES:
***********************************************************************/
//   #ifdef SM_USE_OLDTOL
//     SM_OLDTOL_LINE // obsolete method - scheduled for removal
//     SM_OLDTOL_LINE SmStatus SmVertexuse::GetVertexEdgeGap       (SmGap &rVertexEdgeGap) const
//     SM_OLDTOL_LINE { SM_ASSERT_MSG(rVertexEdgeGap.IsKindOf(SmVertexEdgeGap_TYPE), _T("SmVertexuse::GetVertexEdgeGap - bad arg - not SmVertexEdgeGap_TYPE")) ;
//     SM_OLDTOL_LINE   if(rVertexEdgeGap.IsKindOf(SmVertexEdgeGap_TYPE))
//     SM_OLDTOL_LINE     { (SmVertexEdgeGap &)rVertexEdgeGap = ((SmVertexuse *)this)->GetVertexEdgeGap( FALSE ) ; }
//     SM_OLDTOL_LINE   return ( SM_SUCCESS ) ;
//     SM_OLDTOL_LINE }
//     SM_OLDTOL_LINE 
//     SM_OLDTOL_LINE // obsolete method - scheduled for removal
//     SM_OLDTOL_LINE SmStatus SmVertexuse::GetVertexUVTrimCurveGap(SmGap &rVertexFaceTrimCurveGap) const
//     SM_OLDTOL_LINE { SM_ASSERT_MSG(rVertexFaceTrimCurveGap.IsKindOf(SmVertexFaceTrimCurveGap_TYPE), _T("SmVertexuse::GetVertexFaceTrimCurveGap - bad arg - not SmVertexFaceTrimCurveGap_TYPE")) ;
//     SM_OLDTOL_LINE   if(rVertexFaceTrimCurveGap.IsKindOf(SmVertexFaceTrimCurveGap_TYPE))
//     SM_OLDTOL_LINE     { GetVertexFaceTrimCurveGap((SmVertexFaceTrimCurveGap &)rVertexFaceTrimCurveGap) ; }
//     SM_OLDTOL_LINE   return ( SM_SUCCESS ) ;
//     SM_OLDTOL_LINE }
//     SM_OLDTOL_LINE 
//     SM_OLDTOL_LINE // obsolete method - scheduled for removal
//     SM_OLDTOL_LINE SmStatus SmVertexuse::GetVertexFaceGap (SmGap &rVertexFaceGap) const
//     SM_OLDTOL_LINE { SM_ASSERT_MSG(rVertexFaceGap.IsKindOf(SmVertexFaceGap_TYPE), _T("SmVertexuse::GetVertexFaceGap - bad arg - not SmVertexFaceGap_TYPE")) ;
//     SM_OLDTOL_LINE   if(rVertexFaceGap.IsKindOf(SmVertexFaceGap_TYPE))
//     SM_OLDTOL_LINE     { (SmVertexFaceGap &)rVertexFaceGap = ((SmVertexuse *)this)->GetVertexFaceGap( FALSE ) ; }
//     SM_OLDTOL_LINE   return ( SM_SUCCESS ) ;
//     SM_OLDTOL_LINE }
//     
//   #endif // SM_USE_OLDTOL

/*******************************************************************//**
PURPOSE: Is the vertexuse part of a strut edge?

NOTES:  A strut is an edge not connected to another
  edge at one end.  It is tested for by checking its loop membership;
  if the next ClockWise Edgeuse comes from the same edge as this Edgeuse -
  it's a strut Edge.

  A strut Vertex points to a strut Edge.
***********************************************************************/
SmBoolean SmVertexuse::IsStrutVertexuse() const
{
  SmBoolean bRet = FALSE;
  if (IsEdgeVertexuse())
    {
      SmEdgeuse *pEU   = GetEdgeuse();
      SmEdgeuse *pCWEU = pEU->GetCWEdgeuse();
      if (pEU && pCWEU && pEU != pCWEU)
        {
          if (pEU->GetEdge() == pCWEU->GetEdge()) bRet = TRUE;
        }
    }
  return bRet;

} // end SmVertexuse::IsStrutVertexuse()

/*******************************************************************//**
PURPOSE: Does Vertexuse mark a pole on one of the surfaces to which it connects

NOTES:
***********************************************************************/
SmBoolean SmVertexuse::IsOnPole() const
{
  SmBoolean bOnPole = FALSE;

  // locals
  SmVertexFaceGap * pVertexFaceGap = NULL ;

  // walk topology from vertexuse to surface
  SmFaceuse *pFaceuse = this->GetFaceuse() ;
  SmFace    *pFace    = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmSurface *pSurface = pFace ? pFace->GetSurface() : NULL ;

  // when vertexuse connects to a face->Surface with poles - see if vertex is on pole
  if(   pSurface
     && pSurface->GetSingularities() 
     && HasVertexFaceGap())
    {
      // find the gap to the surface
      pVertexFaceGap = ((SmVertexuse *)this)->GetVertexFaceGap() ;
      const SmPoint2d sFaceUV = pVertexFaceGap->GetFaceUV() ;

      // when the vertex maps to a surface point that is a pole
      SmSurfParamType eSurfParmType ;
      if(   sFaceUV.IsInitialized()
         && pSurface->IsSingularity(sFaceUV, eSurfParmType) )
        {
          // remember it's on a pole
          bOnPole = TRUE ;
        }
    } // end pSurface exists check

  // all done
  return bOnPole;

} // end SmVertexuse::IsOnPole()

/*******************************************************************//**
PURPOSE: Does Vertexuse mark a seam on one of the surfaces to which it connects

NOTES:
***********************************************************************/
SmBoolean SmVertexuse::IsOnSeam() const
{
  // init return value
  SmBoolean bOnSeam = FALSE;

  // locals
  SmVertexFaceGap * pVertexFaceGap ;

  // walk topology from vertexuse to surface
  SmFaceuse *pFaceuse   = this->GetFaceuse();
  SmFace    *pFace      = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmSurface *pSurface   = pFace ? pFace->GetSurface() : NULL ;

  // no work - no surface
  if(pSurface == NULL)
    { return FALSE ; }

  // classify surface
  SmExtent2d sUVDomain  = pSurface->GetNaturalUVDomain() ;
  SmBoolean  bPeriodicU = pSurface->IsClosed(sUVDomain, SM_SP_U) ;
  SmBoolean  bPeriodicV = pSurface->IsClosed(sUVDomain, SM_SP_V) ;

  // when vertexuse connects to a face->Surface with seams - see if vertex is on seam
  if(   (bPeriodicU || bPeriodicV)
     && HasVertexFaceGap())
    {
      // find the gap to the surface to get a UV point
      pVertexFaceGap = ((SmVertexuse *)this)->GetVertexFaceGap() ;
      const SmPoint2d sFaceUV = pVertexFaceGap->GetFaceUV() ;

      // check if vertex maps to a surface point that is a seam
      bOnSeam = pSurface->IsOnSeam(sFaceUV) ;

    } // end pSurface exists check

  // all done
  return bOnSeam;

} // end SmVertexuse::IsOnSeam()

/*******************************************************************//**
PURPOSE: rtn TRUE when VU->Vertex is on a VU->EU->Face->Surface closed boundary
                  and that Boundary is within the VU->Sector

NOTES: For a valid Brep, this method should always return FALSE
  since every Seam boundary in a face is required to be represented
  explicitly by a Seam Edge.

  However, When importing Breps from other systems that do not support
  Seam Edges, Faces are temporarily made as part of the import process
  that have missing Seam edges.  Splitting every edge everytime the
  edge crosses a seam creates vertices that have sectors that contain
  the missing seam curve direction.
  
  Assuming all edges have been split when they cross seams, a missing
  seam edge can be detected solely by examining the properties of a 
  Vertexuse.  That happens when
    Vertexuse->VertexPoint projects to Vertexuse->Edgeuse->Face->Surface
   on a closed surface boundary.  And the begin and end UVtangents of
   the sector include that seam's UV direction. 
***********************************************************************/
SmBoolean SmVertexuse::IsSectorMissingSeam    // rtn: TRUE = Sector contains a SurfaceSeam not yet marked with a SeamEdge
 (SmBoolean * pOptSectorMissingSeamU,    // out: Optional, TRUE = Sector contains a missing Udir Seam that runs through sector vertex
  SmBoolean * pOptSectorMissingSeamV,    // out: Optional, TRUE = Sector contains a missing Vdir Seam that runs through sector vertex
  SmBoolean   bUVSpaceOkay)              // in : default:[TRUE] = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) <br>
                                         //      FALSE = don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] <br>
 const 
{ 
  // init output
  if(pOptSectorMissingSeamU) { *pOptSectorMissingSeamU = FALSE ; }
  if(pOptSectorMissingSeamV) { *pOptSectorMissingSeamV = FALSE ; }

  // locals
  SmUVSectorIO sUVSectorIO ; 

  // Compute Sector properties
  ComputeUVSector(sUVSectorIO,              // out: Optional Sector Properties. NULL to ignore. Default:[NULL]
                  bUVSpaceOkay) ;           // in : default:[TRUE]  = Use LocalSolve with guesses from UVTrimCurve
                                            //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve

  // set optional output
  if(pOptSectorMissingSeamU) { *pOptSectorMissingSeamU = sUVSectorIO.m_bSectorMissingSeamU ; }
  if(pOptSectorMissingSeamV) { *pOptSectorMissingSeamV = sUVSectorIO.m_bSectorMissingSeamV ; }

  // all done
  return(sUVSectorIO.m_bSectorMissingSeamU || sUVSectorIO.m_bSectorMissingSeamV) ;

} // end SmVertexuse::IsSectorMissingSeam 

/*******************************************************************//**
PURPOSE: Return TRUE when VertexSector data is self consistent.

NOTES: returns FALSE for Loopuse_TYPE and Shelluse_TYPE Vertexuses.

CHECKS: A VertexSector has the following Objects with Geometry at the corner

     3d Space Geometry   | UV Space Geometry
     --------------------+----------------------------
     Vertex->Point3d     |
     BegEdge->Curve3d    | BegEdgeuse->UVTrimCurve
     EndEdge->Curve3d    | EndEdgeuse->UVTrimCurve
     Face->Surface       |

The set of XSectTol3d values for all the various connected topology objects include:

     sXSectTol3d_VertFace       = SmTol::GetXSectTol3d(Face,    Vertex)      
     sXSectTol3d_FaceBegEdge    = SmTol::GetXSectTol3d(Face,    BegEdge)
     sXSectTol3d_FaceEndEdge    = SmTol::GetXSectTol3d(Face,    EndEdge)
     sXSectTol3d_BegEdgeEndEdge = SmTol::GetXSectTol3d(BegEdge, EndEdge)
     sXSectTol3d_VertBegEdge    = SmTol::GetXSectTol3d(BegEdge, Vertex)     
     sXSectTol3d_VertEndEdge    = SmTol::GetXSectTol3d(EndEdge, Vertex)     

The sector geometries can be evaluated to generate the following 3d and UV space points.
 (Note: Evaluated Point Names have no suffixes.)
     3d Space Points                    | UV Space Points
     -----------------------------------+----------------------------
     VertexPoint = Vertex->GetPoint     |
     BegCrvPoint = BegCurve3d(BegParam) | BegTrimCrvPoint = BegUVTrimCurve(BegParam)
     EndCrvPoint = EndCurve3d(EndParam) | EndTrimCrvPoint = EndUVTrimCurve(EndParam)

UV and 3d Space points can be Mapped and InvMapped into the other space.
(Note: Mapped Point Name suffixes are '3d' or '2d'
     UVSpace Mapped to 3d                             | 3d Space InvMapped to UV                      
     -------------------------------------------------+----------------------------------------------  
                                                      | VertexPoint2d = pSurf->DropPoint(VertexPoint) 
     BegTrimCrvPoint3d = pSurf->Eval(BegTrimCrvPoint) | BegCrvPoint2d = pSurf->DropPoint(BegCrvPoint)  
     EndTrimCrvPoint3d = pSurf->Eval(EndTrimCrvPoint) | EndCrvPoint2d = pSurf->DropPoint(EndCrvPoint)
     VertexPoint3d     = pSurf->Eval(VertexPoint2d)   |  
                         
Checks (point/point gap checks for directly connected geometry and Edge/Edge and Vertex/Face connections)
   1. Compare Evaluated Points in 3d
      CloserThan(VertexPoint, BegCrvPoint, sXSectTol3d_BegEdgeVert)
      CloserThan(VertexPoint, EndCrvPoint, sXSectTol3d_EndEdgeVert)
      CloserThan(BegCrvPoint, EndCrvPoint, sXSectTol3d_BegEdgeEndEdge)

   2. Compare Evaluated Points in UV
      CloserThan(BegTrimCrvPoint, EndTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_EndEdgeVert))

   3. Compare Evaluated Points with Mapped Points in 3d
      CloserThan(VertexPoint,       VertexPoint3d,     sXSectTol3d_FaceVert)
      CloserThan(VertexPoint,       BegTrimCrvPoint3d, sXSectTol3d_BegEdgeVert) 
      CloserThan(VertexPoint,       EndTrimCrvPoint3d, sXSectTol3d_EndEdgeVert) 
      CloserThan(BegCrvPoint,       BegTrimCrvPoint3d, SmTol::MapTo2d(sXSectTol3d_FaceBegEdge))
      CloserThan(EndCrvPoint,       EndTrimCrvPoint3d, SmTol::MapTo2d(sXSectTol3d_FaceEndEdge))
      CloserThan(BegTrimCrvPoint3d, EndTrimCrvPoint3d, sXSectTol3d_sEndEdgeBegEdge) <== typical loop connection test

   4. Compare Evaluated Points with Mapped Points in UV
      CloserThan(VertexPoint2d, BegTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_BegEdgeVert)) 
      CloserThan(VertexPoint2d, EndTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_EndEdgeVert)) 
      CloserThan(BegCrvPoint2d, BegTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_FaceBegEdge))
      CloserThan(EndCrvPoint2d, EndTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_FaceEndEdge))
   
   5. Containment check - UV points offset into the sector interior are inside the surface->NaturalUVDomain    
   
      UVPoints stepping into the Sector interior
      with:
        BegTrimCrvTan from BegUVTrimCurve->Eval(BegParam, BegTrimCrvPoint, BegTrimCrvTan) call,
        EndTrimCrvTan from EndUVTrimCurve->Eval(EndParam, EndTrimCrvPoint, EndTrimCrvTan) call,
      
      // single step points  
      BegTan_FromBegPt2d = BegCrvPoint2d + ParamTol * BegTrimCrvTan
      EndTan_FromBegPt2d = BegCrvPoint2d + ParamTol * EndTrimCrvTan
      BegTan_FromEndPt2d = EndCrvPoint2d + ParamTol * BegTrimCrvTan        
      EndTan_FromEndPt2d = EndCrvPoint2d + ParamTol * EndTrimCrvTan 
      
      // double step points
      TwoTan_FromBegPt2d = BegCrvPoint2d + ParamTol * (BegTrimCrvTan + EndTrimCrvTan)
      TwoTan_fromEndPt2d = EndCrvPoint2d + ParamTol * (BegTrimCrvTan + EndTrimCrvTan) 
      
      // when SectorAngle > 180 also check 
      OtherTwoTan_FromBegPt2d = BegCrvPoint2d - ParamTol * (BegTrimCrvTan + EndTrimCrvTan)
      OtherTwoTan_fromEndPt2d = EndCrvPoint2d - ParamTol * (BegTrimCrvTan + EndTrimCrvTan) 

      // check
      sUVDomain.Contains(BegTan_FromBegPt2d)
      sUVDomain.Contains(EndTan_FromBegPt2d)
      sUVDomain.Contains(BegTan_FromEndPt2d)
      sUVDomain.Contains(EndTan_FromEndPt2d)
      sUVDomain.Contains(OtherTwoTan_FromBegPt2d)
      sUVDomain.Contains(OtherTwoTan_fromEndPt2d)
***********************************************************************/
SmBoolean SmVertexuse::IsSectorGapFreeAndOriented() const
{
  // init return 
  SmBoolean bRtn = TRUE ;  // TRUE = okay

  // no work - not an EdgeuseTYPE
  if(!IsEdgeVertexuse() || GetEdgeuse() == NULL)
    { return(FALSE) ; }

  // no work - no Face or missing UVTrimCurves
  if(GetEdgeuse()->GetFace() == NULL)  
    { return(FALSE) ; }

  // no work - missing UVTrimCurves
  if(   GetEdgeuse()->GetUVTrimCurve()                 == NULL 
     || GetEdgeuse()->GetCWEdgeuse()                   == NULL   
     || GetEdgeuse()->GetCWEdgeuse()->GetUVTrimCurve() == NULL)  
    { return(FALSE) ; }

  // locals - connected topological objects and their geometries
  SmFace          * pFace             = GetEdgeuse()->GetFace() ;

  SmSurface       * pSurface          = pFace ? pFace->GetSurface() : NULL ;
  SmExtent2d        sUVDomain         = pSurface->GetNaturalUVDomain() ;
                  
  SmEdgeuse       * pBegEdgeuse       = GetEdgeuse() ;                
  //SmCurve         * pBegUVTrimCurve   = pBegEdgeuse->GetUVTrimCurve() ;
  //SmEdgeuse       * pBegMate          = pBegEdgeuse->GetMate() ;      
  SmEdge          * pBegEdge          = pBegEdgeuse->GetEdge() ;
  SmOrientType      eBegEdgeuseOrient = pBegEdgeuse->GetOrientation() ;
  //SmOrientType eBegMateOrient    = pBegMate   ->GetOrientation() ;

  SmEdgeuse       * pEndEdgeuse       = pBegEdgeuse->GetCWEdgeuse() ; 
  //SmCurve         * pEndUVTrimCurve   = pEndEdgeuse->GetUVTrimCurve() ;
  //SmEdgeuse       * pEndMate          = pEndEdgeuse->GetMate() ;      
  SmEdge          * pEndEdge          = pEndEdgeuse->GetEdge() ;
  //SmOrientType eEndEdgeuseOrient = pEndEdgeuse->GetOrientation() ;
  //SmOrientType eEndMateOrient    = pEndMate   ->GetOrientation() ;

  SmVertex        * pBegVertex  = GetVertex() ;
  SmVertex        * pEndVertex  = pEndEdgeuse->GetMate()->GetVertexuse()->GetVertex() ;   // End Vertex for EndEdgeuse

  // BegEdge and EndEdge Curve ParamVals 
  double dBegParam = eBegEdgeuseOrient == SM_OT_SAME ? pBegEdge->GetInterval().GetMin() 
                                                     : pBegEdge->GetInterval().GetMax() ;
  //double dEndParam = eEndEdgeuseOrient == SM_OT_SAME ? pEndEdge->GetInterval().GetMin() 
  //                                                   : pEndEdge->GetInterval().GetMax() ;

  // Tols
  SmXSectTol3d sXSectTol3d_FaceVert       = SmTol::GetXSectTol3d(pFace,    pBegVertex) ;       
  SmXSectTol3d sXSectTol3d_FaceBegEdge    = SmTol::GetXSectTol3d(pFace,    pBegEdge) ;
  SmXSectTol3d sXSectTol3d_FaceEndEdge    = SmTol::GetXSectTol3d(pFace,    pEndEdge) ;
  SmXSectTol3d sXSectTol3d_BegEdgeEndEdge = SmTol::GetXSectTol3d(pBegEdge, pEndEdge) ;
  SmXSectTol3d sXSectTol3d_BegEdgeVert    = SmTol::GetXSectTol3d(pBegEdge, pBegVertex) ;
  SmXSectTol3d sXSectTol3d_EndEdgeVert    = SmTol::GetXSectTol3d(pEndEdge, pBegVertex) ;

  // Evaluated 3d Space Points
  SmPoint3d sVertexPoint ;
  SmPoint3d sBegCrvPoint,  sEndCrvPoint ;
   
  sVertexPoint = pBegVertex->GetPoint() ;
  pBegEdge->GetCurve()->EvaluatePoint(dBegParam, sBegCrvPoint) ; 
  pEndEdge->GetCurve()->EvaluatePoint(dBegParam, sEndCrvPoint) ; 

  // Evaluated UV Space Points
  SmPoint2d sBegTrimCrvPoint,     sEndTrimCrvPoint,     sBegTrimCrvTan,     sEndTrimCrvTan ;
  SmPoint3d sBegTrimCrvPointIn3d, sEndTrimCrvPointIn3d, sBegTrimCrvTanIn3d, sEndTrimCrvTanIn3d ;
  pBegEdgeuse->NormalizedEvaluate(0.0, TRUE, sBegTrimCrvPointIn3d, &sBegTrimCrvTanIn3d) ; // TRUE = UV Eval, FALSE = 3d Eval
  pEndEdgeuse->NormalizedEvaluate(1.0, TRUE, sEndTrimCrvPointIn3d, &sEndTrimCrvTanIn3d) ; // TRUE = UV Eval, FALSE = 3d Eval

  sBegTrimCrvPoint = sBegTrimCrvPointIn3d ; 
  sBegTrimCrvTan   = sBegTrimCrvTanIn3d ; 
  sEndTrimCrvPoint = sEndTrimCrvPointIn3d ;    
  sEndTrimCrvTan   = sEndTrimCrvTanIn3d ;

  // sEndTrimCrvTan current points into the Vertex - negate it so it points out from the Vertex
  sEndTrimCrvTanIn3d *= -1.0 ; 
  sEndTrimCrvTan     *= -1.0 ;

  // 3d Points InvMapped to UV Space
  SmBoolean bSuccess ;
  SmBoolean bIsMulti;
  SmPoint2d sVertexPoint2d ; 
  SmPoint2d sBegCrvPoint2d, sEndCrvPoint2d ; 
  double    dFaceVert_Gap3d = 0.0;
  double    dFaceBegEdge_Gap3d = 0.0, dFaceEndEdge_Gap3d = 0.0;

  pSurface->DropPoint(sVertexPoint, sUVDomain, &sBegTrimCrvPoint, bSuccess, sVertexPoint2d, dFaceVert_Gap3d, bIsMulti) ;
  pSurface->DropPoint(sBegCrvPoint, sUVDomain, &sBegTrimCrvPoint, bSuccess, sBegCrvPoint2d, dFaceBegEdge_Gap3d, bIsMulti) ;
  pSurface->DropPoint(sEndCrvPoint, sUVDomain, &sEndTrimCrvPoint, bSuccess, sEndCrvPoint2d, dFaceEndEdge_Gap3d, bIsMulti) ;

  // UV Points Mapped to 3d Space
  SmPoint3d sVertexPoint3d ;
  SmPoint3d sBegTrimCrvPoint3d, sEndTrimCrvPoint3d ;

  pSurface->EvaluatePoint(sVertexPoint2d, sVertexPoint3d) ;
  pSurface->EvaluatePoint(sBegTrimCrvPoint, sBegTrimCrvPoint3d) ;
  pSurface->EvaluatePoint(sEndTrimCrvPoint, sEndTrimCrvPoint3d) ;

  // short names for SmTol::MapTo2d calls
  SmPoint2d sMapUV = sBegTrimCrvPoint ;  // for okay sectors - any of the UVPoints should work for mapping to 2d
  SmSurface &rSrf  = *pSurface ;

  // macros for debugging convenience - put bThisRtn, dGap, dTol, and lCnt in watch window
#ifdef SM_DEBUG_CODE
  ULONG   lCnt = 0 ;
  //double  dGap, dTol ;
  //SmBoolean bThisRtn ;
  #define SM_SAME_CHECK(a, b) ((a) == (b)) ;                                 /*bThisRtn = bRtn ;*/ if(bRtn == FALSE) { lCnt++ ; bRtn = TRUE ; }
  #define SM_GAP_CHECK(gap, tol) ((gap) < (tol)) ; /*dGap = gap ; dTol = tol ; bThisRtn = bRtn ;*/ if(bRtn == FALSE) { lCnt++ ; bRtn = TRUE ; } 
  #define SM_PTS_CHECK(pt1, pt2, tol) (pt1).CloserThan((tol), (pt2)) ; /*dGap = ((pt1)-(pt2)).Length() ; dTol = (tol) ; bThisRtn = bRtn ;*/ if(bRtn == FALSE) { lCnt++ ; bRtn = TRUE ; } 
  #define SM_INSIDE_CHECK(dom, pt) (dom).ContainsPoint2d(pt) ;               /*bThisRtn = bRtn ;*/ if(bRtn == FALSE) { lCnt++ ; bRtn = TRUE ; }
#else  // no SM_DEBUG_CODE
  #define SM_SAME_CHECK(a, b) ((a) == (b)) 
  #define SM_GAP_CHECK(gap, tol) ((gap) < (tol))    
  #define SM_PTS_CHECK(pt1, pt2, tol) (pt1).CloserThan((tol), (pt2)) 
  #define SM_INSIDE_CHECK(dom, pt) (dom).ContainsPoint2d(pt)  
#endif // SM_DEBUG_CODE
  // macros for debugging convenience - put bThisRtn, dGap, dTol, dGap<dTol, and lCnt in watch window

  // simple topology checks
  bRtn &= SM_SAME_CHECK(pBegVertex, pEndVertex) ;

  // FaceDrop Gap checks
  bRtn &= SM_GAP_CHECK(dFaceVert_Gap3d   , sXSectTol3d_FaceVert) ;
  bRtn &= SM_GAP_CHECK(dFaceBegEdge_Gap3d, sXSectTol3d_FaceBegEdge) ;
  bRtn &= SM_GAP_CHECK(dFaceEndEdge_Gap3d, sXSectTol3d_FaceEndEdge) ;

  // 1. Compare Evaluated Points in 3d
    {

      bRtn &= SM_PTS_CHECK(sVertexPoint, sBegCrvPoint, sXSectTol3d_BegEdgeVert) ;
      bRtn &= SM_PTS_CHECK(sVertexPoint, sEndCrvPoint, sXSectTol3d_EndEdgeVert) ;
      bRtn &= SM_PTS_CHECK(sBegCrvPoint, sEndCrvPoint, sXSectTol3d_BegEdgeEndEdge) ;  // loop EdgeEnd/EdgeEnd gap3d check
    } // end 1. Compare Evaluated Points in 3d scope

  // 2. Compare Evaluated Points in UV
    {
      bRtn &= SM_PTS_CHECK(sBegTrimCrvPoint, sEndTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_EndEdgeVert, sMapUV, rSrf)) ; 
    } // end 2. Compare Evaluated Points in UV scope

  // 3. Compare Evaluated Points with Mapped Points in 3d
    {
      bRtn &= SM_PTS_CHECK(sVertexPoint, sVertexPoint3d,     sXSectTol3d_FaceVert) ;         
      bRtn &= SM_PTS_CHECK(sVertexPoint, sBegTrimCrvPoint3d, sXSectTol3d_BegEdgeVert) ;           
      bRtn &= SM_PTS_CHECK(sVertexPoint, sEndTrimCrvPoint3d, sXSectTol3d_EndEdgeVert) ;           
                                                             
      bRtn &= SM_PTS_CHECK(sBegCrvPoint, sBegTrimCrvPoint3d, sXSectTol3d_FaceBegEdge) ;           
      bRtn &= SM_PTS_CHECK(sEndCrvPoint, sEndTrimCrvPoint3d,sXSectTol3d_FaceBegEdge) ;            

      bRtn &= SM_PTS_CHECK(sBegTrimCrvPoint3d, sEndTrimCrvPoint3d, sXSectTol3d_BegEdgeEndEdge) ; // typical loop connection test
    } // end 3. Compare Evaluated Points with Mapped Points in 3d scope

  // 4. Compare Evaluated Points with Mapped Points in UV
    {
      bRtn &= SM_PTS_CHECK(sVertexPoint2d, sBegTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_BegEdgeVert, sMapUV, rSrf)) ;
      bRtn &= SM_PTS_CHECK(sVertexPoint2d, sEndTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_EndEdgeVert, sMapUV, rSrf)) ;
      bRtn &= SM_PTS_CHECK(sBegCrvPoint2d, sBegTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_FaceBegEdge, sMapUV, rSrf)) ;
      bRtn &= SM_PTS_CHECK(sEndCrvPoint2d, sEndTrimCrvPoint, SmTol::MapTo2d(sXSectTol3d_FaceEndEdge, sMapUV, rSrf)) ;
    } // end 4. Compare Evaluated Points with Mapped Points in UV scope 

  // 5. Containment check - UV points offset into the sector interior are inside the surface->NaturalUVDomain
    {
      // single step points  
      SmPoint2d sBegTan_FromBegPt2d = sBegCrvPoint2d + SM_EFF_ZERO_PARAM * sBegTrimCrvTan ;
      SmPoint2d sEndTan_FromBegPt2d = sBegCrvPoint2d + SM_EFF_ZERO_PARAM * sEndTrimCrvTan ;
      SmPoint2d sBegTan_FromEndPt2d = sEndCrvPoint2d + SM_EFF_ZERO_PARAM * sBegTrimCrvTan ;       
      SmPoint2d sEndTan_FromEndPt2d = sEndCrvPoint2d + SM_EFF_ZERO_PARAM * sEndTrimCrvTan ;
      
      // check containment of step off points - CheckTol2d = 0.0 here because steps are tolerance sized
      bRtn &= SM_INSIDE_CHECK(sUVDomain, sBegTan_FromBegPt2d) ;
      bRtn &= SM_INSIDE_CHECK(sUVDomain, sEndTan_FromBegPt2d) ;
      bRtn &= SM_INSIDE_CHECK(sUVDomain, sBegTan_FromEndPt2d) ;
      bRtn &= SM_INSIDE_CHECK(sUVDomain, sEndTan_FromEndPt2d) ;

      // Sector Angle Deg in SurfNormal coordinates
      double dSectorUVAngRad = 0.0;
      sBegTrimCrvTan.Unitize() ;
      sEndTrimCrvTan.Unitize() ;
      sBegTrimCrvTan.CCWAngleBetween(sEndTrimCrvTan, dSectorUVAngRad) ; 

      // when SectorUVAngle <= 180  
      if(dSectorUVAngRad <= SM_PI)
        {
          // check double step points
          SmPoint2d sTwoTan_FromBegPt2d = sBegCrvPoint2d + SM_EFF_ZERO_PARAM * (sBegTrimCrvTan + sEndTrimCrvTan) ;
          SmPoint2d sTwoTan_fromEndPt2d = sEndCrvPoint2d + SM_EFF_ZERO_PARAM * (sBegTrimCrvTan + sEndTrimCrvTan) ;
          bRtn &= SM_INSIDE_CHECK(sUVDomain, sTwoTan_FromBegPt2d) ;
          bRtn &= SM_INSIDE_CHECK(sUVDomain, sTwoTan_fromEndPt2d) ;
        }
      else // when SectorUVAngle > 180 also check
        {
          // check double step points
          SmPoint2d sOtherTwoTan_FromBegPt2d = sBegCrvPoint2d - SM_EFF_ZERO_PARAM * (sBegTrimCrvTan + sEndTrimCrvTan) ;
          SmPoint2d sOtherTwoTan_fromEndPt2d = sEndCrvPoint2d - SM_EFF_ZERO_PARAM * (sBegTrimCrvTan + sEndTrimCrvTan) ;

          bRtn &= SM_INSIDE_CHECK(sUVDomain, sOtherTwoTan_FromBegPt2d) ;
          bRtn &= SM_INSIDE_CHECK(sUVDomain, sOtherTwoTan_fromEndPt2d) ;
        }
    } // end 5. Containment check - UV points offset into the sector interior are inside the surface->NaturalUVDomain scope

#ifdef SM_DEBUG_CODE 
  // lCnt = number of failed cases - okay when lCnt == 0.
  bRtn = (lCnt == 0) ; 
#endif // SM_DEBUG_CODE

  // all done - TRUE = okay, FALSE = problems
  return(bRtn) ;

#undef SM_GAP_CHECK
#undef SM_PTS_CHECK

} // end SmVertexuse::IsSectorGapFreeAndOriented

/*******************************************************************//**
PURPOSE: Get the loopuse of a vertexuse.

NOTES:
***********************************************************************/
SmLoopuse * SmVertexuse::GetLoopuse() const
{ // Can't get a loopuse from shell vertexuse
  SM_ASSERT(   (m_pSorLUorEU == NULL)
            || (m_tVertexuseType == SmEdgeuse_TYPE && m_pSorLUorEU->IsKindOf(SmEdgeuse_TYPE))
            || (m_tVertexuseType == SmLoopuse_TYPE && m_pSorLUorEU->IsKindOf(SmLoopuse_TYPE))
            || (m_tVertexuseType == SmShell_TYPE   && m_pSorLUorEU->IsKindOf(SmShell_TYPE))) ;

  return(  (m_tVertexuseType == SmEdgeuse_TYPE) ? GetEdgeuse()->GetLoopuse()
         : (m_tVertexuseType == SmLoopuse_TYPE) ? (SmLoopuse*)m_pSorLUorEU
         : NULL) ;

} // end SmVertexuse::GetLoopuse

/*******************************************************************//**
PURPOSE: Receive notification of things happening to the object and
take appropriate actions.

  NOTES: For example cleaning up the cache of an
object which is being deleted or edited.  Also clean up any attributes
and relations specific to this class which are not handled automatically
by construtors.
***********************************************************************/
void SmVertexuse::Notify               // expected calls: caller->Notify(Event, pData1, pData2, pData2)
 (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3
  SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
  SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL
  SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2
                                       // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep
                                       // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                       // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                       // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL
                                       // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL
                                       // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL
                                       // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL
                                       // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL
                                       // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL
                                       // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     
{
  switch (eNotifyOperation)
    {
      case SM_NO_PRE_EDIT              : break ;
                                       
      case SM_NO_POST_EDIT             : 
      case SM_NO_SPLIT_IN_BREP         :
      case SM_NO_CHANGE_GEOMETRY       :
      case SM_NO_SPLIT                 : { // Clear Cached Gap data
                                           m_sVertexEdgeGap3d.ReSet() ;
                                           m_sVertexFaceGap3d.ReSet() ;
                                         }
                                         break ;
                                       
      case SM_NO_ADD_TO_BREP           : break ;
      case SM_NO_MERGE_IN_BREP         : break ;
      case SM_NO_TRIM_NO_SPLIT_IN_BREP : break ;
      case SM_NO_COINCIDENT            : break ;
      case SM_NO_RM_FROM_BREP          : break ;
      case SM_NO_CHANGE_OWNER          : break ;
      case SM_NO_CONSTRUCTION          : break ;
      case SM_NO_COPY                  : break ;
      case SM_NO_MERGE                 : break ;
      case SM_NO_REG_PROPAGATION       : break ;
      case SM_NO_DESTRUCTION           : break ;
      case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmEdge::Notify - SM_NO_UNKNOWN event signalled")) ; }
                                         break ;
    }

  // Propagate notification up hierarchy
  SmTopology::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmVertexuse::Notify

/*************************************************************
PURPOSE: draw view of Vertex/Region, Vertex/Face , or Vertex/Edge connection

NOTES: 
**************************************************************/
SmDisplayList * SmVertexuse::Draw
 (SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // locals
  SM_TYPE            eType          = GetVertexuseType() ;
  //SmShell          * pShell         = GetShell() ;     // NotNULL for eType == SmShell_TYPE   (16013)
  //SmLoopuse        * pLoopuse       = GetLoopuse() ;   // NotNULL for eType == SmLoopuse_TYPE (16004)
  //SmEdgeuse        * pEdgeuse       = GetEdgeuse() ;   // NotNULL for eType == SmEdgeuse_TYPE (16005)
                   
  SmVertex         * pVertex        = GetVertex() ;
  SmFaceuse        * pFaceuse       = GetFaceuse() ;   // NotNULL for eType == SmLoopuse_TYPE || SmEdgeuse_TYPE
  SmFace           * pFace          = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmSurface        * pSurface       = pFace ? pFace->GetSurface() : NULL ;
  //SmBoolean          bSwapNormal    = pFaceuse ? (pFaceuse->GetOrientation() == SM_OT_OPPOSITE) : FALSE ;
  const SmContext  * cpContext      = GetContext() ; 
  SmCurve          * pSectorArc1=NULL, * pSectorArc2=NULL,* pSectorArc3=NULL, * pSectorArc4=NULL,* pSectorArc5=NULL ; 
  SmCrvOnSurf      * pMissingSeamLine1=NULL, * pMissingSeamLine2=NULL ;
  SmObjDelete        sCleanArc1, sCleanArc2, sCleanArc3, sCleanArc4, sCleanArc5 ;
  SmObjDelete        sCleanMissingSeam1, sCleanMissingSeam2 ;

  // no work - missing Vertex
  if(pVertex == NULL)
    { return(NULL) ; }

  // Local Points and Polylines to draw
  SmPoint3d           sVertexPoint = pVertex->GetPoint() ;
  SmTArray<SmPoint3d> sBegEdgePolyline, sEndEdgePolyline ;

  // When the Vertex connects to a Edgeuse - draw Vertex and EdgeuseTangent, EdgeuseBinormal, FaceuseNormal direction sequence
  if(eType == SmEdgeuse_TYPE)
    {
      // Draw values
      SmVector3d sBegEdgeBinormalPoint, sBegEdgeBinormal, sBegEdgeuseTangent, sBegEdgeFaceuseNormal ;
      SmVector3d sEndEdgeBinormalPoint, sEndEdgeBinormal, sEndEdgeuseTangent, sEndEdgeFaceuseNormal ;
      double             dBegEdgeSize, dEndEdgeSize ; 
      SmExtentPointType  eBegEdgeType, eEndEdgeType ; 
      SmExtent1d         sBegEdgeIvl,  sEndEdgeIvl ;

      // locals
      // SmPoint2d   sUVPoint, sUVBegEdgePoint, sUVEndEdgePoint ;
      // double      dSectorAngDeg ;
      SmUVSectorIO sUVSectorIO ;
      double       dSectorRadUV=FALSE ;
      SmPoint2d    sUVBegEdgePoint ;
      SmPoint2d    sUVEndEdgePoint ;
      // SmVector2d  sUVBegEdgeTan, sUVEndEdgeTan ;

      // Get Sector Geometry: sRtn == SM_SUCCESS for eTYpe == SmEdgeuse_TYPE, else == SM_ERR
      ComputeUVSector(sUVSectorIO,  // out: Optional Sector Properties. NULL to ignore. default:[NULL]
                      TRUE) ;       // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve
                                    //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve, default:[TRUE]
      // sUVPoint,        // out: Projection of Vertexuse->Vertex->Point onto
      //                  //      Vertexuse->Edgeuse->Loopuse->Faceuse->Face->Surface
      // sUVBegEdgeTan,   // out: BegEdgeFaceSector UVTan UnitVec, Oriented From rUVPoint into Face
      // sUVEndEdgeTan,   // out: EndEdgeFaceSector UVTan UnitVec, Oriented From rUVPoint into Face
      // dSectorAngDeg,   // out: UVSpace rotation from BegEdgeTan to EndEdgeTan in the UVplane corner

      NERN(sUVSectorIO.m_pBegEdgeuse) ; 
      NERN(sUVSectorIO.m_pEndEdgeuse) ; 

      // begin scope: set sBegEdgeBinormalPoint, sBegEdgeBinormal, sBegEdgeFaceuseNormal (The BegEdgein Sector direction)
        {
           // locals
           SmEdge * pBegEdge = sUVSectorIO.m_pBegEdgeuse->GetEdge() ;  
                    sBegEdgeIvl  = pBegEdge->GetInterval() ;  
                    eBegEdgeType = sBegEdgeIvl.ClassifyPoint(sUVSectorIO.m_dBegEdgeParam) ;
           SM_ASSERT_MSG(  (eBegEdgeType == SM_EP_START || eBegEdgeType == SM_EP_END), 
                         _T("SmVertexuse::Draw Err: Vertexuse->Edgeuse->Param not Start or EndEdge")) ;  

           // move BegEdgeParam 5% into the edge
           sUVSectorIO.m_dBegEdgeParam = (eBegEdgeType == SM_EP_START) ? sBegEdgeIvl.Evaluate(0.05) : sBegEdgeIvl.Evaluate(0.95) ;
       
           // GetEdgeuse binormal positions and directions
           sUVSectorIO.m_pBegEdgeuse->EvaluateBinormal(sUVSectorIO.m_dBegEdgeParam, 
                                                       TRUE, 
                                                       sBegEdgeBinormalPoint,         // out: 3D pt on edge
                                                       sBegEdgeBinormal,              // out: unit-vector pointing to SmFace interior from rBinormalPoint
                                                       &sBegEdgeuseTangent,           // out: opt non-unit Edgeuse tangent at rBinormalPoint, NULL to ignore, default:[NULL]                  
                                                       &sBegEdgeFaceuseNormal,        // out: opt unit Faceuse normal, NULL to ignore, default:[NULL]                                         
                                                       &sUVBegEdgePoint) ;            // out: opt Surface param value at edge point, NULL to ignore, default:[NULL]                           
                                                                                      // out: opt Surface Binormal param direction at edge point, not unitized, NULL to ignore, default:[NULL]
                                                                                      // 
           // size the Binormal and FaceuseNormal to (BinormalPoint-VertexPoint).Dist
           dBegEdgeSize = (sBegEdgeBinormalPoint - sVertexPoint).Length() ;

           // UVRadius
           dSectorRadUV = (sUVBegEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8 ; 

         } // end scope: setting sBegEdgeBinormalPoint, sBegEdgeBinormal, sBegEdgeFaceuseNormal

      // begin scope: set sEndEdgeBinormalPoint, sEndEdgeBinormal, sEndEdgeFaceuseNormal (The EndEdge Sector direction)
        {
           // locals
           SmEdge * pEndEdge = sUVSectorIO.m_pEndEdgeuse->GetEdge() ;  
                    sEndEdgeIvl  = pEndEdge->GetInterval() ;  
                    eEndEdgeType = sEndEdgeIvl.ClassifyPoint(sUVSectorIO.m_dEndEdgeParam) ;
           SM_ASSERT_MSG(  (eEndEdgeType == SM_EP_START || eEndEdgeType == SM_EP_END), 
                         _T("SmVertexuse::Draw Err: Vertexuse->Edgeuse->Param not Start or EndEdge")) ;  

           // move EndEdgeParam 5% into the edge
           sUVSectorIO.m_dEndEdgeParam = (eEndEdgeType == SM_EP_START) ? sEndEdgeIvl.Evaluate(0.05) : sEndEdgeIvl.Evaluate(0.95) ;
       
           // GetEdgeuse binormal positions and directions
           sUVSectorIO.m_pEndEdgeuse->EvaluateBinormal(sUVSectorIO.m_dEndEdgeParam, 
                                                  TRUE, 
                                                  sEndEdgeBinormalPoint, 
                                                  sEndEdgeBinormal, 
                                                  &sEndEdgeuseTangent, 
                                                  &sEndEdgeFaceuseNormal, 
                                                  &sUVEndEdgePoint) ;  
       
           // size the Binormal and FaceuseNormal to (BinormalPoint-VertexPoint).Dist
           dEndEdgeSize = (sEndEdgeBinormalPoint - sVertexPoint).Length() ;

           // UVRadius
           if(dSectorRadUV >  (sUVEndEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8)
             { dSectorRadUV = (sUVEndEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8 ; }

         } // end scope: setting sEndEdgeBinormalPoint, sEndEdgeBinormal, sEndEdgeFaceuseNormal  

        // size the output graphics
        double dSize = smos_Min(dBegEdgeSize, dEndEdgeSize) ;

        // When BegEdge 3d Step Size is bigger - try smaller BegEdge ParamStep to make 3d step sizes about the same size
        if(dBegEdgeSize > dEndEdgeSize)
          {
           double dStep = (dEndEdgeSize / dBegEdgeSize) * 0.05 ; 

           // move BegEdgeParam scaled 5% into the edge
           sUVSectorIO.m_dBegEdgeParam = (eBegEdgeType == SM_EP_START) ? sBegEdgeIvl.Evaluate(dStep) : sBegEdgeIvl.Evaluate(1.0 - dStep) ;
       
           // GetEdgeuse binormal positions and directions
           sUVSectorIO.m_pBegEdgeuse->EvaluateBinormal(sUVSectorIO.m_dBegEdgeParam, TRUE, sBegEdgeBinormalPoint, sBegEdgeBinormal, NULL, &sBegEdgeFaceuseNormal, &sUVBegEdgePoint) ;  
          
           // UVRadius
           if(dSectorRadUV >  (sUVBegEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8)
             { dSectorRadUV = (sUVBegEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8 ; }

          } // end reduce BegEdge sampling size

        // When EndEdge 3d Step Size is bigger - try smaller EndEdge ParamStep to make 3d step sizes about the same size 
        if(dEndEdgeSize > dBegEdgeSize)
          {                        
            double dStep = (dBegEdgeSize / dEndEdgeSize) * 0.05 ;
            
           // move EndEdgeParam scaled 5% into the edge
           sUVSectorIO.m_dEndEdgeParam = (eEndEdgeType == SM_EP_START) ? sEndEdgeIvl.Evaluate(dStep) : sEndEdgeIvl.Evaluate(1.0 - dStep) ;
       
           // GetEdgeuse binormal positions and directions
           sUVSectorIO.m_pEndEdgeuse->EvaluateBinormal(sUVSectorIO.m_dEndEdgeParam, TRUE, sEndEdgeBinormalPoint, sEndEdgeBinormal, NULL, &sEndEdgeFaceuseNormal, &sUVEndEdgePoint) ;  
             
           // UVRadius
           if(dSectorRadUV >  (sUVEndEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8)
             { dSectorRadUV = (sUVEndEdgePoint-sUVSectorIO.m_sBegEdgeUVPoint).Length() * .8 ; }

          } // end reduce EndEdge sampling size
            
        sBegEdgeBinormal      *= dSize ;                                              
        sEndEdgeBinormal      *= dSize ;                                              
        sBegEdgeFaceuseNormal *= dSize ;
        sEndEdgeFaceuseNormal *= dSize ;

        // Build the BegEdge output Polyline
        sBegEdgePolyline.Add(sVertexPoint) ;
        sBegEdgePolyline.Add(sBegEdgeBinormalPoint) ;
        sBegEdgePolyline.Add(sBegEdgePolyline[1] + sBegEdgeBinormal) ;
        sBegEdgePolyline.Add(sBegEdgePolyline[2] + sBegEdgeFaceuseNormal) ;

        // Build the EndEdge output Polyline
        sEndEdgePolyline.Add(sVertexPoint) ;
        sEndEdgePolyline.Add(sEndEdgeBinormalPoint) ;
        sEndEdgePolyline.Add(sEndEdgePolyline[1] + sEndEdgeBinormal) ;
        sEndEdgePolyline.Add(sEndEdgePolyline[2] + sEndEdgeFaceuseNormal) ;

        // Build ArcCurves in Sector - don't care if arcs run outside surface - that makes it easier to see database errors
        SmBoolean bOutDomain1 = FALSE, bOutDomain2 = FALSE, bOutDomain3 = FALSE, bOutDomain4 = FALSE, bOutDomain5 = FALSE ;
        pSurface->CreateArcOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVSectorIO.m_sBegEdgeUVTan, sUVSectorIO.m_dSectorAngDeg, 0.3*dSectorRadUV, bOutDomain1, pSectorArc1) ; 
        pSurface->CreateArcOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVSectorIO.m_sBegEdgeUVTan, sUVSectorIO.m_dSectorAngDeg, 0.6*dSectorRadUV, bOutDomain2, pSectorArc2) ; 
        pSurface->CreateArcOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVSectorIO.m_sBegEdgeUVTan, sUVSectorIO.m_dSectorAngDeg, 0.9*dSectorRadUV, bOutDomain3, pSectorArc3) ; 
        pSurface->CreateArcOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVSectorIO.m_sBegEdgeUVTan, sUVSectorIO.m_dSectorAngDeg, 1.2*dSectorRadUV, bOutDomain4, pSectorArc4) ; 
        pSurface->CreateArcOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVSectorIO.m_sBegEdgeUVTan, sUVSectorIO.m_dSectorAngDeg, 1.5*dSectorRadUV, bOutDomain5, pSectorArc5) ; 
        if(bOutDomain1 || bOutDomain2 || bOutDomain3 || bOutDomain4 || bOutDomain5)
          { // rebuild CrvOnSurf as SmCircle when CrvOnSurf UVCurve runs out of the Surface domain
            SmVector3d sX        = sBegEdgeBinormalPoint - sVertexPoint ;                                                          
            SmVector3d sY        = sEndEdgeFaceuseNormal * sX ; 
            SmVector3d sEndTan   = sEndEdgeBinormalPoint - sVertexPoint ;
            double     dAngRad3d ; 
            sBegEdgeFaceuseNormal.CCWAngleBetween(sX, sEndTan, dAngRad3d) ;
            double     dAngDeg3d = SM_RAD2DEG(dAngRad3d) ;
            if(dAngDeg3d < SM_EFF_ZERO_DEG) { dAngDeg3d += 360.0 ; }
            double     dRadius3d = smos_Min(sX.Length(), sEndTan.Length()) * .8 ;
            sX.Unitize() ;
            sY.Unitize() ; 
            SmExtent1d sAnalDomain(0.0, dAngDeg3d) ; 

            // replace CrvOnSurf curves whose UVTrimCircles run beyond the Surface's UVDomain with approximate 3d arcs
            if(bOutDomain1) { if(pSectorArc1) delete pSectorArc1 ; pSectorArc1 = new (cpContext) SmCircle(sVertexPoint, sX, sY, sAnalDomain, 0.3*dRadius3d) ; }
            if(bOutDomain2) { if(pSectorArc2) delete pSectorArc2 ; pSectorArc2 = new (cpContext) SmCircle(sVertexPoint, sX, sY, sAnalDomain, 0.6*dRadius3d) ; }
            if(bOutDomain3) { if(pSectorArc3) delete pSectorArc3 ; pSectorArc3 = new (cpContext) SmCircle(sVertexPoint, sX, sY, sAnalDomain, 0.9*dRadius3d) ; }
            if(bOutDomain4) { if(pSectorArc4) delete pSectorArc4 ; pSectorArc4 = new (cpContext) SmCircle(sVertexPoint, sX, sY, sAnalDomain, 1.2*dRadius3d) ; }
            if(bOutDomain5) { if(pSectorArc5) delete pSectorArc5 ; pSectorArc5 = new (cpContext) SmCircle(sVertexPoint, sX, sY, sAnalDomain, 1.5*dRadius3d) ; }
          } // end built a CrvOnSurf curve whose UVTrimCurve runs beyond the Surfaces UVDomain check
        sCleanArc1.SetObj(pSectorArc1) ; 
        sCleanArc2.SetObj(pSectorArc2) ; 
        sCleanArc2.SetObj(pSectorArc3) ; 
        sCleanArc2.SetObj(pSectorArc4) ; 
        sCleanArc2.SetObj(pSectorArc5) ; 

        // Build Missing Seam Curve                                    
        if(sUVSectorIO.m_bSectorMissingSeamU || sUVSectorIO.m_bSectorMissingSeamV)                                               
          {                                                            
            SmPoint2d sUVEndEdgePt      = sUVSectorIO.m_sBegEdgeUVPoint ;
            double    dBegEdgeAngRad    = smos_ArcTangent2(sUVSectorIO.m_sBegEdgeUVTan.y, sUVSectorIO.m_sBegEdgeUVTan.x) ;
            double    dSectorAngRad = SM_DEG2RAD(sUVSectorIO.m_dSectorAngDeg) ; 
            SmExtent2d sUVDomain      = pSurface->GetNaturalUVDomain() ; 

            if(sUVSectorIO.m_bSectorMissingSeamU) 
              { double dDir = (   (dBegEdgeAngRad < SM_PI/2.0   && (dBegEdgeAngRad+dSectorAngRad) > SM_PI/2.0) 
                               || (dBegEdgeAngRad < 5*SM_PI/2.0 && (dBegEdgeAngRad+dSectorAngRad) > 5*SM_2PI/2.0)) ? 1.0 : -1.0 ;
                sUVEndEdgePt.y += dDir * 1.2*dSectorRadUV ;
                sUVEndEdgePt.y = sUVDomain.GetVInterval().ClampValue(sUVEndEdgePt.y) ; 
                pSurface->CreateLineOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVEndEdgePt, pMissingSeamLine1) ;
                sCleanMissingSeam2.SetObj(pMissingSeamLine1) ;
              }

            if(sUVSectorIO.m_bSectorMissingSeamV) 
              { double dDir = (   (dBegEdgeAngRad < 0.0    && (dBegEdgeAngRad+dSectorAngRad) > 0) 
                               || (dBegEdgeAngRad < SM_2PI && (dBegEdgeAngRad+dSectorAngRad) > SM_2PI)) ? 1.0 : -1.0 ;
                sUVEndEdgePt.x += dDir * 1.2*dSectorRadUV ;
                sUVEndEdgePt.x = sUVDomain.GetUInterval().ClampValue(sUVEndEdgePt.x) ; 
                pSurface->CreateLineOnSurf(*cpContext, sUVSectorIO.m_sBegEdgeUVPoint, sUVEndEdgePt, pMissingSeamLine2) ;
                sCleanMissingSeam1.SetObj(pMissingSeamLine2) ;
              }
          } // end MissingSeam check

     } // end if eType == SmEdgeuse_TYPE

  // arrive here when all points and polylines to draw eType == SmEdgeuse_TYPE have been built
  // next - draw the graphics
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  double dLineWidth = smgfx_GetLineWidth() ;
  double dPointSize = smgfx_GetPointSize() ;
  SmVector3d sOutputColor = smgfx_GetOutputColor() ;
    {
      // Draw StartSector Vertex -> BegEdgeBinormalPoint -> BegEdgeBinormalVector -> BegEdgeFaceuseNormal Polylines 
      smgfx_DrawPoint(sVertexPoint.x, sVertexPoint.y, sVertexPoint.z, pOptGfxSet) ;
  
      // Start of sector color = Green
      SmVector3d sColor = smgfx_OutputColor(0.0, 1.0, 0.0, pOptGfxSet) ;
      if(sBegEdgePolyline.GetSize() > 0) { smgfx_DrawPolyline((double*)sBegEdgePolyline.GetDataArray(), sBegEdgePolyline.GetSize(), pOptGfxSet) ; }

      // EndEdge of sector color = Red
      smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
      if(sEndEdgePolyline.GetSize() > 0) { smgfx_DrawPolyline((double*)sEndEdgePolyline.GetDataArray(), sEndEdgePolyline.GetSize(), pOptGfxSet) ; }

      // sector interior crosshatch color = Cyan
      smgfx_SetLook(1, 2, 0.0, 1.0, 1.0, pOptGfxSet) ;
      if(pSectorArc1) { pSectorArc1->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }
      if(pSectorArc2) { pSectorArc2->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }
      if(pSectorArc3) { pSectorArc3->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }
      if(pSectorArc4) { pSectorArc4->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }
      if(pSectorArc5) { pSectorArc5->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }

      // missing Seam color = grey
      smgfx_SetLook(3, 4, .3, .3, .3) ;
      if(pMissingSeamLine1) { pMissingSeamLine1->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }
      if(pMissingSeamLine2) { pMissingSeamLine2->Draw(NULL, FALSE, NULL, pOptGfxSet) ; }
    }

  // restore Draw color and LineWidth
  smgfx_SetLook(dLineWidth, dPointSize, sOutputColor, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE

  return(pRtn) ;

} // end SmVertexuse::Draw

/*************************************************************
PURPOSE: draw micro view of Vertex/Region, Vertex/Face , or Vertex/Edge connection

NOTES: o - Draw Edge to high resolution
       o - Draw EdgeStrip on Face using Face->MicroColor
**************************************************************/
SmDisplayList * SmVertexuse::DrawMicro
 (SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // locals
  SM_TYPE          eType          = GetVertexuseType() ;
  SmFaceuse      * pFaceuse       = GetFaceuse() ;
  SmFace         * pFace          = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmSurface      * pSurface       = pFace ? pFace->GetSurface() : NULL ;
  SmVertex       * pVertex        = GetVertex() ;
  SmBoolean        bSwapNormal    = pFaceuse ? (pFaceuse->GetOrientation() == SM_OT_OPPOSITE) : FALSE ;

  // no work - missing Vertex
  if(pVertex == NULL)
    { return(NULL) ; }

  // locals
  ULONG ii ;
  // SmPoint2d   sUVPoint ;
  // SmVector2d  sUVBegTan,     sUVEndTan ;
  double      dLastBegEdgeParam=SM_BIG_DOUBLE, dLastEndEdgeParam=SM_BIG_DOUBLE ;
  SmUVSectorIO sUVSectorIO ;

  // Get Sector Geometry
  SmStatus sRtn = ComputeUVSector(sUVSectorIO,   // out: Optional Sector Properties.  NULL to ignore, default:[NULL]
                                  TRUE) ;        // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve
                                                 //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve
                                                 //      default:[TRUE]
  SM_REF1(sRtn) ;

  // sUVPoint      = sUVSectorIO.m_sBegEdgeUVPoint ;  // Projection of Vertexuse->Vertex->Point onto
  //                                                  // Vertexuse->Edgeuse->Loopuse->Faceuse->Face->Surface
  // sUVBegTan     = sUVSectorIO.m_sBegEdgeUVTan ; // BegFaceSector UVTan UnitVec, Oriented From rUVPoint into Face
  // sUVEndTan     = sUVSectorIO.m_sEndEdgeUVTan ; // EndFaceSector UVTan UnitVec, Oriented From rUVPoint into Face
  // dSectorAngDeg = sUVSectorIO.m_dSectorAngDeg ;    // UVSpace rotation from BegTan to EndTan in the UVplane corner
                                  
  // local points and polylines to draw
  SmPoint3d           sVertexPoint = pVertex->GetPoint() ;
  SmTArray<SmPoint3d> sBegEdgePoints, sEndEdgePoints, sFacePoints, sFaceNormals ;

  // 3d MicroDraw size
  SmApproxTol3d sApproxTol3d = (pFace) ? SmTol::GetApproxTol3d(pFace)
                                       : SmTol::GetApproxTol3d(GetContext()) ;

  // When the Vertex connects to a face - draw MicroFace shape
  if(   (eType == SmLoopuse_TYPE || eType == SmEdgeuse_TYPE)
     && (pSurface))
    {
      // check state
      SM_ASSERT_MSG(sRtn == SM_SUCCESS || eType == SmLoopuse_TYPE, _T("SmVertexuse::DrawMicro - ComputeUVSector didn't work as expected")) ;

      // locals
      SmPoint3d sFacePoint, sFaceNormal ;

      // compute step inside parameter distance = pFace->ApproxTol3d
      SmPoint2d sU( 0.0, 0.0 );
      SmPoint2d sV( 0.0, 1.0 );
      SmPoint2d sRotTan(sUVSectorIO.m_sBegEdgeUVTan);
      SmPoint2d sSmpPointUV ;

      double    dAngDeg = 0, dAngSign = (sUVSectorIO.m_dSectorAngDeg >= 0.0) ? 1.0 : -1.0 ;
      SmTol2d   sStepUVSizeU = SmTol::MapTo2d(sApproxTol3d, sUVSectorIO.m_sBegEdgeUVPoint, sU, *pSurface) ; // TRUE = use Consistent Tol Model
      SmTol2d   sStepUVSizeV = SmTol::MapTo2d(sApproxTol3d, sUVSectorIO.m_sBegEdgeUVPoint, sV, *pSurface) ; // TRUE = use Consistent Tol Model

      // build FaceCorner triangle strip - Point[0] = corner, Points[1,...] sampled fan circumference
        {
          pSurface->EvaluatePoint(sUVSectorIO.m_sBegEdgeUVPoint, sFacePoint) ;
          pSurface->EvaluateNormal(sUVSectorIO.m_sBegEdgeUVPoint, TRUE, TRUE, sFaceNormal) ;
          if(bSwapNormal) { sFaceNormal = -sFaceNormal ; }
          sFacePoints.Add(sFacePoint) ;
          sFaceNormals.Add(sFaceNormal) ;

          // Sample sector in 15 deg steps
          while(smos_Fabs(dAngDeg) <= smos_Fabs(sUVSectorIO.m_dSectorAngDeg))
            {
              // compute next Sample point = RotTan * StepUVSize
              sRotTan.Set(sRotTan.x * sStepUVSizeU, sRotTan.y * sStepUVSizeV) ;
              sSmpPointUV = sUVSectorIO.m_sBegEdgeUVPoint + sRotTan ;

              // build triangle strip - Point[0] = corner, Points[1,...] sampled fan
              pSurface->EvaluatePoint(sSmpPointUV, sFacePoint) ;
              pSurface->EvaluateNormal(sSmpPointUV, TRUE, TRUE, sFaceNormal) ;
              if(bSwapNormal) { sFaceNormal = -sFaceNormal ; }
              sFacePoints.Add(sFacePoint) ;
              sFaceNormals.Add(sFaceNormal) ;

              // exit case
              if(dAngDeg == sUVSectorIO.m_dSectorAngDeg)
                { break ; }

              // increment the SampleAng Deg
              dAngDeg += dAngSign*15.0 ;
              if(smos_Fabs(dAngDeg) >= smos_Fabs(sUVSectorIO.m_dSectorAngDeg) - SmTol::GetScaledZero(sUVSectorIO.m_dSectorAngDeg))
                { dAngDeg = sUVSectorIO.m_dSectorAngDeg ; }

              // increment Tan direction
              sRotTan = sUVSectorIO.m_sBegEdgeUVTan.Rotate(SM_DEG2RAD(dAngDeg)) ;

            } // end iter sampling sector circumference
        } // end Build FaceCorner triangle strip scope

      // When the Vertex also connects to Edges - draw MicroEdge shape
      if(eType == SmEdgeuse_TYPE)
        {
          // locals
          SmEdge      * pBegEdge                = sUVSectorIO.m_pBegEdgeuse->GetEdge() ;
          SmEdge      * pEndEdge                = sUVSectorIO.m_pEndEdgeuse->GetEdge() ;
          SmCurve     * pBegEdgeCurve           = pBegEdge->GetCurve() ;
          SmCurve     * pEndEdgeCurve           = pEndEdge->GetCurve() ;
          SmExtent1d    sBegEdgeIvl             = pBegEdge->GetInterval() ;
          SmExtent1d    sEndEdgeIvl             = pEndEdge->GetInterval() ;
          SmScaledZero  sBegEdgeIvlScaledZero   = SmTol::GetScaledZero(sBegEdgeIvl) ;
          SmScaledZero  sEndEdgeIvlScaledZero   = SmTol::GetScaledZero(sEndEdgeIvl) ;
          double        dBegEdgeIncSign         = (sUVSectorIO.m_pBegEdgeuse->GetOrientation() == SM_OT_SAME)     ? 1.0 : -1.0 ;
          double        dEndEdgeIncSign         = (sUVSectorIO.m_pEndEdgeuse->GetOrientation() == SM_OT_OPPOSITE) ? 1.0 : -1.0 ;
          SmVector3d    sBegEdgePoint, sEndEdgePoint ;

          // for several ApproxTol sides - sample the edges
          for(ii=0;ii<8;ii++)
            {
              // sample the curves
              if(dLastBegEdgeParam != sUVSectorIO.m_dBegEdgeParam) { pBegEdgeCurve->EvaluatePoint(sUVSectorIO.m_dBegEdgeParam, sBegEdgePoint) ;
                                                                     sBegEdgePoints.Add(sBegEdgePoint) ;
                                                                   }
              if(dLastEndEdgeParam != sUVSectorIO.m_dEndEdgeParam) { pEndEdgeCurve->EvaluatePoint(sUVSectorIO.m_dEndEdgeParam, sEndEdgePoint) ;
                                                                     sEndEdgePoints.Add(sEndEdgePoint) ;
                                                                   }

              // increment the param values, even sApproxTol3d 3d steps
              dLastBegEdgeParam = sUVSectorIO.m_dBegEdgeParam ;
              dLastEndEdgeParam = sUVSectorIO.m_dEndEdgeParam ;
              sUVSectorIO.m_dBegEdgeParam += dBegEdgeIncSign * SmTol::MapTo1d(sApproxTol3d, sUVSectorIO.m_dBegEdgeParam, *pBegEdgeCurve) ; // TRUE = use NewTol Model
              sUVSectorIO.m_dEndEdgeParam += dEndEdgeIncSign * SmTol::MapTo1d(sApproxTol3d, sUVSectorIO.m_dEndEdgeParam, *pEndEdgeCurve) ; // TRUE = use NewTol Model

              // for very short edges
              sBegEdgeIvl.SnapValue(sUVSectorIO.m_dBegEdgeParam, sBegEdgeIvlScaledZero) ;
              sEndEdgeIvl.SnapValue(sUVSectorIO.m_dEndEdgeParam, sEndEdgeIvlScaledZero) ;

            } // end iter side Edges getting MicroSample Points

        } // end Vertex also connects to an Edge check

    } // end Vertex connects to a Face check

  // arrive here when all points and polylines to draw have been built
  // next - draw the graphics

  // Get static s_Disp display parameters in file SmGraphicsExtern.cpp
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawMicro = TRUE ;

  // save current draw state for later restore
  SmVector3d sColor     = smgfx_GetOutputColor() ;
  double     dLineWidth = smgfx_GetOutputLineWidth(pOptGfxSet) ;
  double     dPointSize = smgfx_GetOutputPointSize(pOptGfxSet) ;

  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // draw Vertex color = black, size >= 8
  smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
  if(dPointSize < 8.0) { smgfx_OutputPointSize(8.0, pOptGfxSet) ; }

  // draw Vertex
  smgfx_DrawPoint(sVertexPoint.x, sVertexPoint.y, sVertexPoint.z, pOptGfxSet) ;

  // Draw Sector Edges, color = Reg, size >= 2
  if(   sBegEdgePoints.GetSize() > 1
     || sEndEdgePoints.GetSize() > 1)
    {
      smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
      if(dLineWidth < 4.0) { smgfx_OutputLineWidth(4.0, pOptGfxSet) ;}

      // Draw BegEdge
      if(sBegEdgePoints.GetSize() > 1)
        {
          smgfx_DrawPolyline((double *)sBegEdgePoints.GetDataArray(), sBegEdgePoints.GetSize(), pOptGfxSet) ;
        }

      // Draw EndEdge
      if(sEndEdgePoints.GetSize() > 1)
        {
          smgfx_DrawPolyline((double *)sEndEdgePoints.GetDataArray(), sEndEdgePoints.GetSize(), pOptGfxSet) ;
        }

    } // end Draw Sector Edge PolyLines

  // restore Draw color and LineWidth
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif
  return(pRtn) ;

} // end SmVertexuse::DrawMicro

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertVertexuse_list[] =
{
  /* 0 */ {SM_AT_POINTER,     _T("Bad Context Pointers"),   _T("this Vertexuse and its m_pSorLUorEU don't share the same context") },
  /* 1 */ {SM_AT_TYPE,        _T("Bad Type Value"),         _T("this Vertexuse->m_tVertexuseType does not match the m_pSorLUorEU->m_tObjType value") },
  /* 2 */ {SM_AT_CACHE,       _T("Bad Vertex/Edge Cache"),  _T("Vertexuse cached Vertex/Edge Gap value is stale") },
  /* 3 */ {SM_AT_CACHE,       _T("Bad Vertex/Face Cache"),  _T("Vertexuse cached Vertex/Face Gap value is stale") },
  /* 4 */ {SM_AT_GEOMETRIC,   _T("Bad Gap or Orientation"), _T("Vertexuse->Sector has UV and 3d gaps exceeding XSectTol or Sector nearby interior not inside FaceSrf->UVDomain") },
  /* 5 */ {SM_AT_TOPOLOGICAL, _T("Bad Missing Seam"),       _T("Vertexuse->Sector contains a FaceSrf->Seam not marked by SeamEdge") },
  /* 6 */ {SM_AT_POINTER,     _T("vertexuse/faceuse"),      _T("Vertexuse must connect to Faceuse for LoopuseTypes and an EdgeuseTypes not connected to wires") }
} ;

/*******************************************************************//**
AssertRule Predicate names and return values:
  predicate = SmBoolean sm_AssertTestClassNameRule#() ; Rtn: TRUE=OK,    FALSE=Bad,
***********************************************************************/

/*******************************************************************//**
PURPOSE: AssertTest Rule 2 - Vertexuse Vertex/Edge Gap cached and freshly computed values must be equal
NOTES:
  predicate = does Cached Vertex/Edge Gap == Calculated Vertex/Edge Gap.
  action    = Refresh Cache: Let Cached Vertex/Edge Gap = Calculated Vertex/Edge Gap
***********************************************************************/
SmBoolean sm_AssertTestVertexuse2             // rtn: TRUE = okay, FALSE = problem
 (const SmVertexuse * pVertexuse,             // in : Vertexuse to test
  SmVertexEdgeGap   * pOptVertexEdgeGap=NULL, // out: Calculated Vertex/Edge Gap, Must pt to SmGap obj to fill, NULL to ignore, default:[NULL]
  SmTol3d           * pOptOutTol3d=NULL,      // out: optional tolerance for failed assert reports
  double            * pOptOutVal=NULL)        // out: optional value for failed assert reports
{
  SmVertexEdgeGap sVertexEdgeGap, *pVertexEdgeGap = pOptVertexEdgeGap ? pOptVertexEdgeGap : &sVertexEdgeGap ;
  SmVertexEdgeGap sStoredVertexEdgeGap ;

  // Test = FALSE when calculated and stored VertexEdge Gaps differ
  SmBoolean bRtn = pVertexuse->IsVertexEdgeGapFresh(*pVertexEdgeGap, sStoredVertexEdgeGap) ;

  // set output
  if(pOptOutTol3d) { *pOptOutTol3d = SmTol::GetXSectTol3d(pVertexuse->GetContext())/1000.0 ; }
  if(pOptOutVal)   { *pOptOutVal   =   sStoredVertexEdgeGap.IsInit()
                                     ? *pVertexEdgeGap - sStoredVertexEdgeGap
                                     : 0.0 ;
                   }
  // all done
  return( bRtn ) ;

} // end sm_AssertTestVertexuse2

// obsolete
// /*******************************************************************//**
// PURPOSE: AssertHeal Rule 2 - When Vertexuse Vertex/Edge Gap is stale,
//                               recalculate cached Gap value.
// NOTES:
// ***********************************************************************/
// SmBoolean sm_AssertHealVertexuse2
//  (SmVertexuse    * pVertexuse,   // in :
//   SmAssertReport & rAReport,     // in :
//   SmAssertArray  * pAList)       // NotUsed: in :
// {
//   SM_REF1(pAList) ;
//   // remember Heal has run on this AssertReport
//   rAReport.m_eAssertType = SM_AT_HEALER ;
// 
//   // locals
//   SmVertexEdgeGap sVertexEdgeGap ;
// 
//   // check test - other fix functions may have already fixed this one
//   rAReport.m_bOK = sm_AssertTestVertexuse2(pVertexuse, &sVertexEdgeGap) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix broken Vertexuses - Refresh cached value.
//   pVertexuse->SetVertexEdgeGap(sVertexEdgeGap) ;
// 
//   // Good Citizenship - log all Object changes with pAList here:
//   //                      pAList->LogReplaceObject(),
//   //                      pAList->LogSplitObject(),
//   //                      pAList->LogMergeObject(),
//   //                      pAList->LogDeleteObject().
//   // No Object changes to log.
// 
//   // No Need to check - fix always works:  check change - fix worked when pNewBSplineSurface != NULL and NewSurface passes test
//   // rAReport.m_bOK = sm_AssertTestVertexuse2(pVertexuse) ;
//   rAReport.m_bOK = TRUE ;
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value fixed stale Vertex/Edge gap problem.") ; }
//   else               { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value did NOT fixed stale Vertex/Edge gap problem.") ; }
// 
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealVertexuse2
// end obsolete

/*******************************************************************//**
PURPOSE: AssertTest Rule 3 - Vertexuse Vertex/Face Gap cached and freshly computed values must be equal
NOTES:
  predicate = does Cached Vertex/Face Gap == Calculated Vertex/Face Gap.
  action    = Refresh Cache: Let Cached Vertex/Face Gap = Calculated Vertex/Face Gap
***********************************************************************/
SmBoolean sm_AssertTestVertexuse3             // rtn: TRUE = okay, FALSE = problem
 (const SmVertexuse * pVertexuse,             // in : Vertexuse to test
  SmVertexFaceGap   * pOptVertexFaceGap=NULL, // out: Calculated Vertex/Face Gap, Must pt to SmGap obj to fill, NULL to ignore, default:[NULL]
  SmTol3d           * pOptOutTol3d=NULL,      // out: optional tolerance for failed assert reports
  double            * pOptOutVal=NULL)        // out: optional value for failed assert reports

{
  SmVertexFaceGap sVertexFaceGap, *pVertexFaceGap = pOptVertexFaceGap ? pOptVertexFaceGap : &sVertexFaceGap ;
  SmVertexFaceGap sStoredVertexFaceGap ;

  // Test = FALSE when calculated and stored VertexFace Gaps differ
  SmBoolean bRtn = pVertexuse->IsVertexFaceGapFresh(*pVertexFaceGap, sStoredVertexFaceGap) ;

  // set output
  if(pOptOutTol3d) { *pOptOutTol3d = SmTol::GetXSectTol3d(pVertexuse->GetContext())/1000.0 ; }
  if(pOptOutVal)   { *pOptOutVal   =   sStoredVertexFaceGap.IsInit()
                                     ? *pVertexFaceGap - sStoredVertexFaceGap
                                     : 0.0 ;
                   }
  // all done
  return( bRtn ) ;


} // end sm_AssertTestVertexuse3

// obsolete
// /*******************************************************************//**
// PURPOSE: AssertHeal Rule 3 - When Vertexuse Vertex/Face Gap is stale,
//                               recalculate cached Gap value.
// NOTES:
// ***********************************************************************/
// SmBoolean sm_AssertHealVertexuse3
//  (SmVertexuse    * pVertexuse,   // in :
//   SmAssertReport & rAReport,     // in :
//   SmAssertArray  * pAList)       // NotUsed: in :
// {
//   SM_REF1(pAList) ;
//   // remember Heal has run on this AssertReport
//   rAReport.m_eAssertType = SM_AT_HEALER ;
// 
//   // locals
//   SmVertexFaceGap sVertexFaceGap ;
// 
//   // check test - other fix functions may have already fixed this one
//   rAReport.m_bOK = sm_AssertTestVertexuse3(pVertexuse, &sVertexFaceGap) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix broken Vertexuses - Refresh cached value.
//   pVertexuse->SetVertexFaceGap(sVertexFaceGap) ;
// 
//   // Good Citizenship - log all Object changes with pAList here:
//   //                      pAList->LogReplaceObject(),
//   //                      pAList->LogSplitObject(),
//   //                      pAList->LogMergeObject(),
//   //                      pAList->LogDeleteObject().
//   // No Object changes to log.
// 
//   // No Need to check - fix always works:  check change - fix worked when pNewBSplineSurface != NULL and NewSurface passes test
//   // rAReport.m_bOK = sm_AssertTestVertexuse3(pVertexuse) ;
//   rAReport.m_bOK = TRUE ;
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value fixed stale Vertex/Face gap problem.") ; }
//   else               { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value did NOT fixed stale Vertex/Face gap problem.") ; }
// 
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealVertexuse3
// end obsolete

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmVertexuse::AssertValid
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
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  /* 0 */ // SmVertexuse and the objects it attaches to need to share common contexts
  if(m_pSorLUorEU)
    {
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pSorLUorEU->GetContext() ), _T("") ) ;
    }

  /* 1 */ // SmVertexuse->m_tVertexuseType type has to match type value stored in m_pSorLUorEU object
  if(m_pSorLUorEU)
    {
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0,
                                       (   (m_pSorLUorEU == NULL)
                                        || (m_tVertexuseType == SmEdgeuse_TYPE && m_pSorLUorEU->IsKindOf(SmEdgeuse_TYPE))
                                        || (m_tVertexuseType == SmLoopuse_TYPE && m_pSorLUorEU->IsKindOf(SmLoopuse_TYPE))
                                        || (m_tVertexuseType == SmShell_TYPE   && m_pSorLUorEU->IsKindOf(SmShell_TYPE)) ), 
                                       _T("") ) ;
    }

#ifdef SM_USE_OLDTOL // Skip new fresh Gap cache value checks until ready to debug the gap and Notify changes
#ifdef SM_NMTLIB_7166
  /* 2 */ // check if Vertexuse cached Vertex/Edge Gap value is stale
  SmTol3d sOutTol3d ;
  double  dOutVal ;
  bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_2, sm_AssertTestVertexuse2(this, NULL, &sOutTol3d, &dOutVal), sOutTol3d, dOutVal, _T("")) ;

  /* 3 */ // check if Vertexuse cached Vertex/Face Gap value is stale
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_2, sm_AssertTestVertexuse3(this, NULL, &sOutTol3d, &dOutVal), sOutTol3d, dOutVal, _T("")) ;
#endif // SM_NMTLIB_7166
#endif // SM_USE_OLDTOL

// this tests shows up a million times when there are gaps between geometry objects.
// These reports should be culled to stop a bunch of cascading messages.
// For some reason - SmFace Test19 has been commented out - that should be used
//                   as one of the this tests cascading checks.
// When looking at a gap checks the mix of Owner and OtherOwner values are combinatorial.
//  The SmAssertArray::FindReport() has to be made smarter to handle all the possibilites.
//  Until we work on removing cascadind AssertReports - this test is just going to be commented out.
//  It's a good test.  It should be run. But we need a plan for cascading messages first.
  //    /* 4 */ // Vertexuse->Sector has UV and 3d gaps exceeding XSectTol or Sector nearby interior not inside FaceSrf->UVDomain
  //    // Skip cascading test - Test 4 is known to Cascade when SmVertex Test1 has failed for pVertexuse->Vertex
  //    if(   SM_ASSERT_REPORT_NOT_IN_ARRAY(SmVertex, 1, GetVertex())
  //       && (   GetEdgeuse() == NULL
  //           || SM_ASSERT_REPORT_NOT_IN_ARRAY(SmFace, 19, GetEdgeuse()->GetEdge()) )
  //      { 
  //        SmBoolean bGapFreeAndOriented = IsSectorGapFreeAndOriented() ;
  //        bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, bGapFreeAndOriented==TRUE, _T("")) ;
  //      }

  /* 5 */ // Vertexuse->Sector contains a FaceSrf->Seam not marked by SeamEdge
  SmBoolean bSectorMissingSeam = IsSectorMissingSeam() ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, bSectorMissingSeam==FALSE, _T("")) ;

  /* 6 */ // Vertexuse connects to Faceuse for LoopuseTypes and EdgeuseTypes not connected to wires.
  if(m_tVertexuseType == SmLoopuse_TYPE || m_tVertexuseType == SmEdgeuse_TYPE)
    {
      SmEdgeuse *pEdgeuse = GetEdgeuse() ;
      SmEdge    *pEdge    = pEdgeuse ? pEdgeuse->GetEdge() : NULL ;
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (GetFaceuse() != NULL || (pEdge && pEdge->IsWire())), _T("")) ;
    }
                 
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SM_PTR_ARRAY(sFaces, SmFace, 16) ;
      SM_PTR_ARRAY(sEdges, SmEdge, 16) ;
      SmVertex * pVertex = GetVertex() ;
      SmEdge   * pEdge   = GetEdgeuse() ? GetEdgeuse()->GetEdge() : NULL ;
      SmFace   * pFace   = GetFaceuse() ? GetFaceuse()->GetFace() : NULL ;
      SmBrep   * pBrep   = GetBrep() ;
      if(pVertex) { pVertex->GetEdges(sEdges) ; }
      if(pEdge)   { pEdge->GetFaces(sFaces) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,0) ; if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; for(di=0;di<sEdges.GetSize();di++) { if(sEdges[di]) sEdges[di]->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,1,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; for(di=0;di<sFaces.GetSize();di++) { if(sFaces[di]) sFaces[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmVertexuse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmVertexuse::AssertHeal
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
//       return ( SmTopology::AssertHeal(rAReport, pAList) ) ;
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
//       case 2 : bRtn = sm_AssertHealVertexuse2(this, rAReport, pAList); break;
//       case 3 : bRtn = sm_AssertHealVertexuse3(this, rAReport, pAList); break;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmVertexuse::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmVertexuse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmVertexuse::IsKindOf( SM_TYPE t ) const
{
  return ((SmVertexuse_TYPE == t) ? TRUE : SmTopology::IsKindOf( (t) ));
}


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVertexuse::Dump( void ) const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // label
  smos_WriteBuffer( _T( "\nBegin SmVertexuse::Dump()" ) );

  // header
  smos_sprintf( sBuff,        _T( "\nSmVertexuse:[0x%p]," ), this );
  smos_sprintf( sBuffForFile, _T( "\nSmVertexuse:[%s]," ), _T( "NotNULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );


  // type
  smos_sprintf( sBuff, _T( " Type:[%s]," ), (m_tVertexuseType == SmShell_TYPE)   ? _T( "Shell" ) :
                                          (m_tVertexuseType == SmLoopuse_TYPE) ? _T( "Loopuse" )
                                                                               : _T( "Edgeuse" ) );
  smos_WriteBuffer( sBuff );

  // owner Shell, Loopuse or Edgeuse
  smos_sprintf( sBuff,        _T( " Owning_%s:[0x%p]" ), m_pSorLUorEU->GetClassString(), m_pSorLUorEU );
  smos_sprintf( sBuffForFile, _T( " Owning_%s:[%s]" ), m_pSorLUorEU->GetClassString(), m_pSorLUorEU ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // UV point
  SmPoint2d sUV;
  SmStatus eStat = ComputeUVPoint( sUV );
  if(eStat == SM_SUCCESS) { smos_sprintf( sBuff, _T( "\n  UV:[%16.16lf  %16.16lf]" ), sUV.x, sUV.y ); }
  else { smos_sprintf( sBuff, _T("%s"), _T( "\n  UV:[unavailable]" ) ); }
  smos_WriteBuffer( sBuff );

  // cached Vertex/Edge gap
  if(m_sVertexEdgeGap3d.IsInit())
  {
    smos_sprintf( sBuff, _T( "\n  Vertex/Edge Gap:[%16.16lf]" ), m_sVertexEdgeGap3d.GetLength() );
  }
  else if(!HasVertexEdgeGap())
  {
    smos_sprintf( sBuff, _T("%s"),_T( "\n  Vertex/Edge Gap:[N/A - NoEdge]" ) );
  }
  else
  {
    smos_sprintf( sBuff, _T("%s"),_T( "\n  Vertex/Edge Gap:[Uninitialized]" ) );
  }
  smos_WriteBuffer( sBuff );

  // cached Vertex/Face gap
  if(m_sVertexFaceGap3d.IsInit())
  {
    smos_sprintf( sBuff, _T( "\n  Vertex/Face Gap:[%16.16lf]" ), m_sVertexFaceGap3d.GetLength() );
  }
  else if(!HasVertexFaceGap())
  {
    smos_sprintf( sBuff, _T("%s"),_T( "\n  Vertex/Face Gap:[N/A - NoFace]" ) );
  }
  else
  {
    smos_sprintf( sBuff, _T("%s"),_T( "\n  Vertex/Face Gap:[Uninitialized]" ) );
  }
  smos_WriteBuffer( sBuff );

  // base class dump
  SmTopology::Dump();


  smos_WriteBuffer( _T( "\nEnd SmVertexuse::Dump()\n" ) );

} // end SmVertexuse::Dump()
