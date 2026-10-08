// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmEdgeuse.cpp
* PURPOSE: Source file for SmEdgeuse class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmEdgeuse.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmEdge.h>
#include <SmSurfaceCache.h>
#include <SmLocalSolveNd.h>
#include <SmCrvOnSurf.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmBSplineSurface.h>
#include <SmGeomUtility.h>
#include <SmPlane.h>
#include <SmAxis2Placement.h>
#include <SmTopologyTraverser.h>
#include <SmGap.h>
#include <SmAssertArray.h>
// Remove Composites
//  #include <SmCFace.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE
//
//    SmEdgeuse * dbgEdgeuse1 = NULL ;
//    SmEdgeuse * dbgEdgeuse2 = NULL ;
//
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: SmEdgeuse Default constructor placed in .cpp file so
            that debugging break points can be set.

NOTES:
***********************************************************************/
SmEdgeuse::SmEdgeuse()
  : m_tEdgeuseType(SmUnknown_TYPE),
    m_pSorLU(NULL),
    m_pCCW(NULL),
    m_pCW(NULL),
    m_pVU(NULL),                   // RCLxx default- set to NULL on instance
    m_pUVTrimCurve(NULL),
    m_pDropCurveFail(NULL)
{
  m_eOrientation = SM_OT_UNKNOWN ;
  m_sMaxEdgeFaceGap3d.ReSet() ;
  m_sEdgeCCWEdgeGap3d.ReSet() ;

  // report construction at SmObject::Notify level - skip other levels
  SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);

} // end SmEdgeuse::SmEdgeuse Default Constructor

/*******************************************************************//**
PURPOSE: Destructor for the edgeuse - cleans up the trim curve.

NOTES:
***********************************************************************/
SmEdgeuse::~SmEdgeuse()
{
  Notify(SM_NO_DESTRUCTION, this, NULL, NULL);

  m_tEdgeuseType = SmUnknown_TYPE ;
  m_pSorLU       = NULL ;
  m_pCCW         = NULL ;
  m_pCW          = NULL ;
  m_pVU          = NULL ;
  m_eOrientation = SM_OT_UNKNOWN ;
  if (m_pUVTrimCurve) { delete m_pUVTrimCurve; m_pUVTrimCurve = NULL ; }
  if (m_pDropCurveFail) { delete m_pDropCurveFail; m_pDropCurveFail = NULL ; }
  m_sMaxEdgeFaceGap3d.ReSet() ;
  m_sEdgeCCWEdgeGap3d.ReSet() ;

} // end SmEdgeuse::~SmEdgeuse destructor

/*******************************************************************//**
PURPOSE: Define a curve(s) of pos, cross-tangent, and higher cross-derivative values from
      a surface boundary returned as a DerivativeSurface only to be evaluated along
      the v = 0 isoparameter line as DerivativeSurface->Evaluate(s,0) for 
      D, Dv, and possible Dvv, and Dvvv values.

NOTES:
  1.) The DerivSurface can be built in many ways from the SurfaceBoundary.  All the
      options that control this construction are placed in the input
      SmDerivSurfaceDefinition object.  Those options are documented in the declaration
      of that class.

  2.) the DerivSurface Dv, Dvv, and Dvvv are computed from Surface directional derivatives.
      The DirectionalDeriv direction can vary along the length of the Surface boundary in
      3 manners as specified by crDS.m_eTangencyType:
        SM_TT_NO_TANGENCY,     // dir for directionalDerivatives = binormal direction (out from surface)
        SM_TT_TAN_TO_EDGES,    // dir for directionalDerivatives = linear interp between neighbor end-UV-tangents
        SM_TT_MATCH_NEIGHBORS, // dir for directionalDerivatives = linear interp between avg of this and neighbor end-UV-tangents

  3.) Loop Corner management: Whencreating a sequence of DerivSurfaces for a sequence
      of edges from a loop use the SM_TT_MATCH_NEIGHBORS option to make sure that 
      the DerivSurfaces end up being water tight with one another at their 
      neighbor/neighbor boundaries.

RETURNS --- Returns SM_ERR for wire edges.
***********************************************************************/
SmStatus SmEdgeuse::CreateDerivativeSurface
  (SmDerivSurfDefinition & crDS,             // in : Construction parameters.
   SmBoolean               bIntoFace,        // in : TRUE = Derivs go into the surface, FALSE = they go out
   SmBSplineSurface     *& rpDerivSurface)   // out: the DerivSurface, use DerivSurface->Evaluate(s,0)
                                             //      to get D, Dv, Dvv, and Dvvv values.
{
  // init output
  rpDerivSurface = NULL;

  // check state - Doesn't work for wire edges.
  SER_MSG( GetEdge()->IsWire() ? SM_ERR : SM_SUCCESS, _T("Can't build Derivative Surface for a wire")) ;

  // locals 
  const SmContext *pContext     = GetContext() ;
  SmCurve         *pBaseCurve   = GetEdge()->GetCurve() ;
  SmSurface       *pSurface     = GetFace()->GetSurface() ;
  SmExtent1d       sEdgeIvl     = GetEdge()->GetInterval() ;

  // make sure the curve parameterization direction and bIntoFace agree
  if(GetOrientation() == SM_OT_OPPOSITE)
    {
      // negate bIntoFace
      bIntoFace = bIntoFace ? FALSE : TRUE ;

      // note: don't reverse the curve parameterization because that
      //       will change the direction of parameterization in rpDerivSurface.
    }

  // get UV dir for directional derivatives
  if(   crDS.m_eTangencyType == SM_TT_TAN_TO_EDGES
     || crDS.m_eTangencyType == SM_TT_MATCH_NEIGHBORS)
    {
      // loop neighbor UV end-tangents, TRUE = proj Edge Pts to surface
      SmVector3d sPnt3d, sVec3d1, sVec3d2 ;

      // neighbor pointers
      SmEdgeuse *pEUPrev = GetCWEdgeuse();
      SmEdgeuse *pEUNext = GetCCWEdgeuse();
      SmBoolean  bClosedEdge = (pEUPrev == this) ? TRUE : FALSE;

      // neighbor end tangents
      SER( pEUPrev->NormalizedEvaluate( 1.0, TRUE, sPnt3d, &sVec3d1 ));    // TRUE = UV Eval, FALSE = 3d Eval
      SER( pEUNext->NormalizedEvaluate( 0.0, TRUE, sPnt3d, &sVec3d2 ));    // TRUE = UV Eval, FALSE = 3d Eval
      SmVector2d sStartUVTangent(sVec3d1.x, sVec3d1.y) ;
      SmVector2d sEndUVTangent  (sVec3d2.x, sVec3d2.y) ;
      // JGU: Only mirror tangent when edge is not closed
      sEndUVTangent = (bClosedEdge) ? sEndUVTangent : - sEndUVTangent; 

      // flip end tangents when asked
      if(bIntoFace == TRUE)
        {
          sStartUVTangent = -sStartUVTangent ;
          sEndUVTangent   = -sEndUVTangent ; 
        }

      // when asked to match neighbor tangents - avg neighbor UV end-tangents with this edges UV end-tangents
      if ( crDS.m_eTangencyType == SM_TT_MATCH_NEIGHBORS )
        {
          // note: Don't average closed curve end-tangents

          // edge start loopCorner
          if(!bClosedEdge)
             // GWC: this check used to worry about lamina edges pEUNext->GetEdge()->IsLamina().
             //      I'm guessing that when the edge is not lamina 
             //      the desire is to match the dir for directional derivatives
             //      with the dir of some other edge that shares this same vertex. 
             //      However, I don't see which edge to use for that.  So assume that
             //      the 3 current cases for picking directions for directionalderivatives
             //      are enough and ignore the lamina state of the neighbors.
            {
              // this edgeuse start tangent
              SmVector3d sEUStartTan;
              SER(NormalizedEvaluate(0.0,TRUE,sPnt3d,&sEUStartTan));  // TRUE = UV Eval, FALSE = 3d Eval
              sEUStartTan = - sEUStartTan;

              // average this and prev edgeuse tangents together
              sEUStartTan.Unitize();
              sStartUVTangent.Unitize();
              if(bIntoFace == TRUE) { sStartUVTangent = - sStartUVTangent ; }
              sStartUVTangent.Set(sEUStartTan.x + sStartUVTangent.x, sEUStartTan.y + sStartUVTangent.y);
              sStartUVTangent.Unitize();
            }

          // edge end loopCorner
          if(!bClosedEdge)
            {
              // this edgeuse end tangent
              SmVector3d sEUEndTan;
              SER(NormalizedEvaluate(1.0,TRUE,sPnt3d,&sEUEndTan));  // TRUE = UV Eval, FALSE = 3d Eval

              // average this and next edgeuse tangents together
              sEUEndTan.Unitize();
              sEndUVTangent.Unitize();
              if(bIntoFace == TRUE) { sEndUVTangent = - sEndUVTangent ; }
              sEndUVTangent.Set(sEUEndTan.x + sEndUVTangent.x, sEUEndTan.y + sEndUVTangent.y) ;
              sEndUVTangent.Unitize();
            }

        } // end when asked to match same-loop neighbor end-tangent values check

      // arrive here when sStartUVTangent and sEndUVTangent values, used to pick the directionalDerivative dirs,
      //  have been set as
      //  1. the same-loop neighbor edgeuse end tangents or
      //  2. the average of the this end tangent with the same-loop neighbor edgeuse end-tangents,
      //       when asked to 'match.'

      // convert sStartUVTangent and sEndUVTangent values into angle values stored in crDS

      // start/end binormal evaluations
      SmVector3d sPnt, sVec ;
      SmVector2d sStartUVDir, sEndUVDir ;
      EvaluateBinormal(sEdgeIvl.GetMin(), TRUE, sPnt, sVec, NULL, NULL, NULL, &sStartUVDir) ;
      EvaluateBinormal(sEdgeIvl.GetMax(), TRUE, sPnt, sVec, NULL, NULL, NULL, &sEndUVDir) ;
      sStartUVDir.Unitize() ;
      sEndUVDir.Unitize() ;

      // compute and store angles
      SER(sStartUVDir.CCWAngleBetween(sStartUVTangent, crDS.m_dStartDerivDirAngleDeg)) ;
      SER(sEndUVDir.  CCWAngleBetween(sEndUVTangent  , crDS.m_dEndDerivDirAngleDeg  )) ;
      crDS.m_dStartDerivDirAngleDeg = SM_RAD2DEG(crDS.m_dStartDerivDirAngleDeg) ;
      crDS.m_dEndDerivDirAngleDeg   = SM_RAD2DEG(crDS.m_dEndDerivDirAngleDeg  ) ;

    } // end need to set crDS.m_dStartAngle and crDS.m_dEndAngle check

  // current limitation - needs to be relaxed
  SmBSplineCurve *pBSplineCurve = pBaseCurve->IsKindOf(SmBSplineCurve_TYPE) ? (SmBSplineCurve *)pBaseCurve : NULL ;

  // pass the call along
  SmStatus bRtn = crDS.CreateDerivativeSurface(*pContext,
                                               *pSurface,
                                               *pBSplineCurve,
                                                sEdgeIvl,
                                                bIntoFace,
                                                TRUE,
                                                GetEdge()->GetTolerance(),
                                                rpDerivSurface) ;
  SER(bRtn) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      // dump BaseCurve, tangent reference, and output Derivative surface
      pBaseCurve->Dump();
      if (crDS.m_pTangentReference) crDS.m_pTangentReference->Dump();
      rpDerivSurface->Dump();

      SmFace *pFace = GetFace() ;

      smgfx_Erase();
      smgfx_SetLook(2,3, 0,0,1) ; pBaseCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,1) ; pBaseCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,1) ; if (crDS.m_pTangentReference) crDS.m_pTangentReference->Draw();  sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; if(pFace) pFace->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpDerivSurface->Draw(TRUE); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpDerivSurface->DrawUV(10,10); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; rpDerivSurface->DrawAlong(SM_SP_V, 0.0, 1); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; rpDerivSurface->DrawAlong(SM_SP_V, 0.0, 2); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; rpDerivSurface->DrawAlong(SM_SP_V, 0.0, 3); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmEdgeuse::CreateDerivativeSurface

/*******************************************************************//**
PURPOSE: Is the edgeuse a strut edgeuse.  The edge is part
  of a loop and this end of the edge does not connect to another
  edge.

NOTES: Detected by traversing both the loop and radial/mate
  connections.  When the next loop member is also the radial partner
  of this edgeuse - then the edge is a strut on this end.
***********************************************************************/
SmBoolean SmEdgeuse::IsStrut() const
{ SmBoolean bRet = FALSE;
  if (m_tEdgeuseType == SmLoopuse_TYPE) 
    {
      if (GetRadial() == m_pCW) bRet = TRUE;
    }
  return bRet;
} // end SmEdgeuse::IsStrut()

/*******************************************************************//**
PURPOSE: Get the vertexuse at the start of an edgeuse.

NOTES: 
***********************************************************************/
SmVertexuse* SmEdgeuse::GetVertexuse() const 
{ SM_ASSERT(m_pVU != NULL); 
  return m_pVU; 
} // end SmEdgeuse::GetVertexuse()

/*******************************************************************//**
PURPOSE: Get the faceuse of an edgeuse.

NOTES: 
***********************************************************************/
SmFaceuse* SmEdgeuse::GetFaceuse() const
{
  SM_ASSERT( m_tEdgeuseType != SmLoopuse_TYPE || m_pSorLU != NULL) ;
  return(  (m_tEdgeuseType == SmLoopuse_TYPE)
         ? (m_pSorLU ? ((SmLoopuse*)m_pSorLU)->GetFaceuse() : NULL)
         : NULL) ;

} // end SmEdgeuse::GetFaceuse

/*******************************************************************//**
PURPOSE: Get the face of an edgeuse.

NOTES: 
***********************************************************************/
SmFace* SmEdgeuse::GetFace() const 
{
  SM_ASSERT(m_tEdgeuseType != SmLoopuse_TYPE || m_pSorLU != NULL) ;
  return(  (m_tEdgeuseType == SmLoopuse_TYPE)
         ? (m_pSorLU ? ((SmLoopuse*)m_pSorLU)->GetFaceuse()->GetFace() : NULL)
         : NULL) ;
} // end SmEdgeuse::GetFace()

/*******************************************************************//**
PURPOSE: Get the loopuse of an edgeuse.

NOTES: 
***********************************************************************/
SmLoopuse* SmEdgeuse::GetLoopuse() const
{
  SM_ASSERT(m_tEdgeuseType != SmLoopuse_TYPE || m_pSorLU != NULL) ;
  SM_ASSERT(   (m_pSorLU == NULL)
            || (m_tEdgeuseType == SmLoopuse_TYPE && m_pSorLU->IsKindOf(SmLoopuse_TYPE))
            || (m_tEdgeuseType == SmShell_TYPE   && m_pSorLU->IsKindOf(SmShell_TYPE))) ;
  return(  (m_tEdgeuseType == SmLoopuse_TYPE)
         ? (SmLoopuse*)m_pSorLU
         : NULL) ;
} // end SmEdgeuse::GetLoopuse()

/*******************************************************************//**
PURPOSE: Is the edgeuse part of a loop?

NOTES: 
***********************************************************************/
SmBoolean SmEdgeuse::IsLoopEdgeuse() const
{ return(  (m_tEdgeuseType == SmLoopuse_TYPE)
         ? TRUE
         : FALSE) ;
} // end SmEdgeuse::IsLoopEdgeuse()

/*******************************************************************//**
PURPOSE: Get the next edgeuse in a loop that connects through given vertex.

NOTES: returns NULL for 
***********************************************************************/
SmEdgeuse * SmEdgeuse::GetCornerMateEdgeuse
  (const SmVertex *pVertex)  // in : common vertex at edgeuse/edgeuse corner
 const
{ SM_ASSERT(IsLoopEdgeuse());
  return(  (IsLoopEdgeuse()               == FALSE)   ? NULL
         : (m_pVU->GetVertex()            == pVertex) ? m_pCW
         : (GetMate()->m_pVU->GetVertex() == pVertex) ? m_pCCW
         : NULL) ; 

} // end SmEdgeuse::GetCornerMateEdgeuse

/*******************************************************************//**
PURPOSE: Get turning AngDeg from Beg to End of this Edgeuse accounting
         for orientation.

NOTES: return AngDeg range:[-180 180]
       returns   0   Deg for parallel Beg and End 1st deriv vectors
               + 180 Deg for antiparallel 1st derivs

TurnAng can't be computed just from the edgeuse EndPoint Tangents.  
A curve path can turn left or right between any pair of end tangent 
values.  For the simplest curves (i.e. min TurnAng) the left or 
right direction of the turn will depend on the relative start 
and end point positions.  Even trickier curves (i.e. spiral curves encircling 
the end point) can traverse any number of revolutions between 
the end points. The turn angle has to be integrated along the length 
of the curve. But, the integration does not have to be done accurately 
all the possible turn angles are

   ActualTurnAng = CCWAng(StartDir, EndDir) +- n*360 ;

So do a rough integration and use that to pick n above and use
the start and end dir directions to get an exact value of the turning angle.
***********************************************************************/
double SmEdgeuse::GetTurningAngDeg() const
{ 
  // return value
  double     dTurnAngDeg ;

  // return Turning AngDeg - ParallelDirs = 0 deg turn, AntiParallel = 180 deg
  GetUVTrimCurve()->ApproximateLength(GetEdge()->GetInterval(), 8, &dTurnAngDeg) ;
  
  // adjust for Edgeuse orientation
  if(GetOrientation() == SM_OT_OPPOSITE)
    { dTurnAngDeg *= -1.0 ; } 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmVector3d sBegUV, sBegDir ;
      SmVector3d sEndUV, sEndDir ;

      // begin and end UV tangents
      NormalizedEvaluate(0.0, TRUE, sBegUV, &sBegDir) ; // TRUE = UVSpace Eval
      NormalizedEvaluate(1.0, TRUE, sEndUV, &sEndDir) ; // TRUE = UVSpace Eval

      // Draw Loop use in space
      SmBrep         * pBrep        = GetBrep() ;
      SmVertexuse    * pThisVU      = GetVertexuse() ;
      SmBSplineCurve * pUVTrimCurve = GetUVTrimCurve() ; 
      SmSurface      * pSurface     = GetFaceuse() && GetFaceuse()->GetFace() ? GetFaceuse()->GetFace()->GetSurface() : NULL ;
      SmEdgeuse      * pCWEdgeuse   = this->GetCWEdgeuse() ; 
      SmEdgeuse      * pCCWEdgeuse  = this->GetCCWEdgeuse() ; 
      SmVertexuse    * pOtherVU     = pCCWEdgeuse ? pCCWEdgeuse->GetVertexuse() : NULL ;
      SmEdge         * pEdge        = GetEdge() ;
      SmCurve        * pCurve       = pEdge ? pEdge->GetCurve() : NULL ;
      SmFace         * pFace        = GetFaceuse() ? GetFaceuse()->GetFace() : NULL ;
      SmCrvOnSurf sTrimOnSurf( *pUVTrimCurve, *pSurface) ;  
      SmPoint3d sBegPN[2], sEndPN[2], s1BegPN[2], s1EndPN[2], s2BegPN[2], s2EndPN[2] ;
      SmVector3d s1BegUV, s1BegDir, s2BegUV, s2BegDir ;
      SmVector3d s1EndUV, s1EndDir, s2EndUV, s2EndDir ;

      NormalizedEvaluate(0.01, TRUE, s1BegUV, &s1BegDir) ; // TRUE = UVSpace Eval
      NormalizedEvaluate(0.99, TRUE, s1EndUV, &s1EndDir) ; // TRUE = UVSpace Eval

      NormalizedEvaluate(0.02, TRUE, s2BegUV, &s2BegDir) ; // TRUE = UVSpace Eval
      NormalizedEvaluate(0.98, TRUE, s2EndUV, &s2EndDir) ; // TRUE = UVSpace Eval

      if(pSurface)
        {
          SmVector2d sBeg2d(sBegUV.x,sBegUV.y), sBegDir2d(sBegDir.x,sBegDir.y) ; 
          SmVector2d sEnd2d(sEndUV.x,sEndUV.y), sEndDir2d(sEndDir.x,sEndDir.y) ; 
          pSurface->EvaluateDirectionalDerivs(sBeg2d, sBegDir2d, 1, sBegPN) ; 
          pSurface->EvaluateDirectionalDerivs(sEnd2d, sEndDir2d, 1, sEndPN) ;

          sBeg2d.Set(s1BegUV.x,s1BegUV.y); sBegDir2d.Set(s1BegDir.x,s1BegDir.y) ;
          sEnd2d.Set(s1EndUV.x,s1EndUV.y); sEndDir2d.Set(s1EndDir.x,s1EndDir.y) ;
          pSurface->EvaluateDirectionalDerivs(sBeg2d, sBegDir2d, 1, s1BegPN) ; 
          pSurface->EvaluateDirectionalDerivs(sEnd2d, sEndDir2d, 1, s1EndPN) ;

          sBeg2d.Set(s2BegUV.x,s2BegUV.y); sBegDir2d.Set(s2BegDir.x,s2BegDir.y) ;
          sEnd2d.Set(s2EndUV.x,s2EndUV.y); sEndDir2d.Set(s2EndDir.x,s2EndDir.y) ;
          pSurface->EvaluateDirectionalDerivs(sBeg2d, sBegDir2d, 1, s2BegPN) ; 
          pSurface->EvaluateDirectionalDerivs(sEnd2d, sEndDir2d, 1, s2EndPN) ;
        } 

      // if(0)
      //   { ((SmEdgeuse *)this)->RebuildUVTrimCurve() ; }

      SM_DUMP_AND_ASSERT2_VALID(pCurve) ;

      smgfx_Erase() ; 
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(3,4, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ; // TRUE = add UVPlane graphics for loop on nearby plane
      smgfx_SetLook(3,4, 0,1,1) ; this->DrawMicro() ; sm_GraphicsLoop() ; // TRUE = add UVPlane graphics for loop on nearby plane
      smgfx_SetLook(5,6, 1,0,0) ; if(pEdge)        {pEdge->DrawParams() ; sm_GraphicsLoop() ;                                }
      smgfx_SetLook(5,6, 1,0,0) ; if(pEdge)        {pEdge->Draw() ; sm_GraphicsLoop() ;                                      }
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {pCurve->DrawPolygon() ; sm_GraphicsLoop() ;                              }
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {pCurve->DrawWDeriv(pCurve->GetNaturalInterval(), 1) ; sm_GraphicsLoop() ;}
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {pCurve->DrawSpeed(-1,100) ; sm_GraphicsLoop() ;                          }
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {pCurve->DrawCurvature(-25,100) ; sm_GraphicsLoop() ;                     }
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {sTrimOnSurf.DrawPolygon() ; sm_GraphicsLoop() ;                          }
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {sTrimOnSurf.DrawWDeriv(pCurve->GetNaturalInterval(), 1) ; sm_GraphicsLoop() ;}
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {sTrimOnSurf.DrawSpeed(-1,100) ; sm_GraphicsLoop() ;                          }
      smgfx_SetLook(5,6, 0,1,1) ; if(pCurve)       {sTrimOnSurf.DrawCurvature(-25,100) ; sm_GraphicsLoop() ;                     }
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace)        {pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;                          }
      smgfx_SetLook(6,7, 0,1,0) ; if(pSurface)     {sBegPN[1].Draw(&sBegPN[0]) ; sm_GraphicsLoop() ;                             }
      smgfx_SetLook(7,8, 1,0,0) ; if(pSurface)     {sEndPN[1].Draw(&sEndPN[0]) ; sm_GraphicsLoop() ;                             }
      smgfx_SetLook(6,7, 0,1,0) ; if(pSurface)     {s1BegPN[1].Draw(&s1BegPN[0]) ; sm_GraphicsLoop() ;                           }
      smgfx_SetLook(7,8, 1,0,0) ; if(pSurface)     {s1EndPN[1].Draw(&s1EndPN[0]) ; sm_GraphicsLoop() ;                           }
      smgfx_SetLook(6,7, 0,1,0) ; if(pSurface)     {s2BegPN[1].Draw(&s2BegPN[0]) ; sm_GraphicsLoop() ;                           }
      smgfx_SetLook(7,8, 1,0,0) ; if(pSurface)     {s2EndPN[1].Draw(&s2EndPN[0]) ; sm_GraphicsLoop() ;                           }
      smgfx_SetLook(5,6, 1,0,0) ; if(pThisVU)      {pThisVU->Draw() ; sm_GraphicsLoop() ;                                        }
      smgfx_SetLook(5,6, 0,1,0) ; if(pOtherVU)     {pOtherVU->Draw() ; sm_GraphicsLoop() ;                                       }
      smgfx_SetLook(3,4, 1,0,1) ; if(pCCWEdgeuse)  {pCCWEdgeuse->Draw() ; sm_GraphicsLoop() ;                                    }
      smgfx_SetLook(3,4, 0,1,0) ; if(pCWEdgeuse)   {pCWEdgeuse->Draw() ; sm_GraphicsLoop() ;                                     }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(dTurnAngDeg) ;

} // end SmEdgeuse::GetTurningAngDeg

/*******************************************************************//**
PURPOSE: Is this edgeuse a shell edgeuse (no face)?

NOTES: 
***********************************************************************/
SmBoolean SmEdgeuse::IsShellEdgeuse() const
{ return(  (m_tEdgeuseType == SmShell_TYPE)
         ? TRUE
         : FALSE) ;
} // end SmEdgeuse::IsShellEdgeuse()

/*******************************************************************//**
PURPOSE: Get the shell of an edgeuse.

NOTES: 
***********************************************************************/
SmShell* SmEdgeuse::GetShell() const
{ 
  SM_ASSERT(   (m_pSorLU == NULL)
            || (m_tEdgeuseType == SmLoopuse_TYPE && m_pSorLU->IsKindOf(SmLoopuse_TYPE))
            || (m_tEdgeuseType == SmShell_TYPE   && m_pSorLU->IsKindOf(SmShell_TYPE))) ;
  return(  (m_tEdgeuseType == SmShell_TYPE)
         ? (SmShell*)m_pSorLU
         : ( m_pSorLU ? ((SmLoopuse*)m_pSorLU)->SmLoopuse::GetShell() : NULL)) ;
} // end SmEdgeuse::GetShell()

/*******************************************************************//**
PURPOSE: Determine if this is a tangent dihedral sector to within the given tolerance.

NOTES: A dihedral sector is the space between two faces connected
 to a single edge.  Each sector on an edge is bounded by the face
 attached to an edgeuse and the face attached to that edgeuse's radial
 partner.  A sector is tangent if the binormal vectors into each of these
 faces are tangent along the length of the edge.
 
 A sector whose faces are coincident to one another (cusp -- sharp edge)
 is not a tangent sector.   However, that condition may be checked for by
 setting the input flag bCoincidenceTest to TRUE; in that case, this routine
 will return TRUE for cusp edges, but not for tangent edges.

METHOD ---
 This test samples an edge at 5 locations (.12, .34, .55, .78, .93)
 and if at each sample point the two binormal vectors (perpendicular
 to the edge pointing into the two faces bounding the sector) are
 tangent to one another, then the sector is called tangent.
***********************************************************************/
SmBoolean SmEdgeuse::IsTangentSector
  (double   dTangencyTolDeg,       // in : max deviation in degrees between
                                   //      two parallel vectors
  SmBoolean bCoincidenceTest,      // in : TRUE = Test for coincidence instead of tangency between faces
                                   //      FALSE= Test for tangency between faces
                                   //      default:[FALSE]
  SmBoolean bOKToMakeUVTrimCurve)  // in : TRUE=Make UVTrimCurve as needed, FALSE=only use 3d calcs when m_pUVTrimCurve==NULL
 const
{
  SmEdge *pEdge = GetEdge();
  if ( pEdge == NULL )
    { return FALSE; }

  // Lamina Edges have nothing to be tangent (or coincident) with.
  if ( pEdge->IsLamina() )
    { return FALSE; }

  // init an array of edge parameter values
  SmTArray<double> sParams;
  sParams.Add(0.34);
  sParams.Add(0.78);
  sParams.Add(0.55);
  sParams.Add(0.12);
  sParams.Add(0.93);

  const SmEdgeuse *pEU0 = this;
  const SmEdgeuse *pEU1 = pEU0->GetRadial();

  double dMaxDotValue = smos_Cosine( (90.0-dTangencyTolDeg) * SM_PI/180.0 );

  // for every parameter value
  ULONG ii;
  for (ii=0; ii<sParams.GetSize(); ii++)
    {
      // locals
      double dTestParam = pEdge->GetInterval().Evaluate(sParams[ii]);
      SmPoint3d sPnt, sPnt2;
      SmVector3d sBinVec2, sBinVec, sFaceuseNormal, sFaceuseNormal2;

      SmCurve  * pUVTrimCurve = GetUVTrimCurve() ;
      SmBoolean  bUse3DOnly   = !bOKToMakeUVTrimCurve && pUVTrimCurve == NULL ;

      // evaluate this edgeuses binormal
      // vector pointing into sector's 1st face
      SE(pEU0->EvaluateBinormal(dTestParam,        // in : value within Edge->m_vInterval.
                                bUse3DOnly,        // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                                   //      FALSE = get/create pUVTrimCurve to calc Surface points
                                sPnt,              // out: 3D pt on edge
                                sBinVec,           // out: unit-vector pointing to SmFace interior from rBinormalPoint
                                NULL,              // out: non-unit Edgeuse tangent at rBinormalPoint
                                &sFaceuseNormal)); // out: unit Faceuse normal. Points to enterior of bounded radial sector
                                                   // out: Surface param value at edge point, NULL to ignore, default:[NULL]
                                                   // out: Surface uv vector pointing inside the surface, not unitized
                                                   //      NULL to ignore, default:[NULL]

      // evaluate sector's other edgeuse's binormal
      // vector pointing into sector's 2nd face
      SE(pEU1->EvaluateBinormal(dTestParam,         // in : value within Edge->m_vInterval.
                                bUse3DOnly,         // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                                    //      FALSE = get/create pUVTrimCurve to calc Surface points
                                sPnt2,              // out: 3D pt on edge
                                sBinVec2,           // out: unit-vector pointing to SmFace interior from rBinormalPoint
                                NULL,               // out: non-unit Edgeuse tangent at rBinormalPoint
                                &sFaceuseNormal2)); // out: unit Faceuse normal. Points to enterior of bounded radial sector
                                                    // out: Surface param value at edge point, NULL to ignore, default:[NULL]
                                                    // out: Surface uv vector pointing inside the surface, not unitized
                                                    //      NULL to ignore, default:[NULL]
      // normalize the sample vectors
      SE(sFaceuseNormal.Unitize());
      SE(sFaceuseNormal2.Unitize());
      SE(sBinVec.Unitize());
      SE(sBinVec2.Unitize());

      // It used to return TRUE even for joints that double back on themselves rather than
      // continuing in the same direction. So now:
      // finds anything > 90 deg. avoids confusion between angles near 0.0 and angles near 180
      // added by LL 16 Sep 2004
      // Extended to allowed for testing for coincident faces RMB Jul 2007

      if (!bCoincidenceTest && sBinVec.Dot(sBinVec2) > 0.0)
        { return FALSE; }
      if ( bCoincidenceTest && sBinVec.Dot(sBinVec2) < 0.0)
        { return FALSE; }

      double dDot1 = sBinVec2.Dot(sFaceuseNormal);
      if (smos_Fabs(dDot1) > dMaxDotValue) {
          // sBinVec2 is not perpendicular to sFaceuseNormal to within tolerance
          return FALSE;
      }
      double dDot2 = sBinVec.Dot(sFaceuseNormal2);
      if (smos_Fabs(dDot2) > dMaxDotValue) {
          // sBinVec1 is no not perpendicular to sFaceuseNormal2 to within tolerance
          return FALSE;
      }
    } // end iter every parameter value

  // get here when all BinVecs are perpendicular (to within tolerance) of
  // all neighbor face normals at each sample point.

  // the sector is tangent
  return TRUE;

} // end SmEdgeuse::IsTangentSector

/*******************************************************************//**
PURPOSE: Compute and return the continuity between the two faces
  bounding this sector.  In this function cusps (sector angle of 0)
  are not considered continuous beyond C0.

NOTES: A sector is the space between two faces connected
 to a single edge.  Each sector on an edge is bounded by the face
 attached to an edgeuse and the face attached to that edgeuse's radial
 partner.

METHOD ---
 Returns the minimum continuity value found at 5 sample 
 locations (.12, .34, .55, .78, .93) along the Edgeuse.
***********************************************************************/
SmContinuityType SmEdgeuse::GetSectorContinuity
  (double dAngTolDeg)              // NotUsed: in : max deviation between parallel vectors in degrees between
                                   //      default:[SM_CONTINUITY_ANGLE]
 const
{
  SM_REF1(dAngTolDeg) ;
  // return value
  SmContinuityType eRtn = SM_CT_CINFINITY ;

  // init an array of edge normalized parameter values
  SmTArray<double> sParams;
  sParams.Add(0.34);
  sParams.Add(0.78);
  sParams.Add(0.55);
  sParams.Add(0.12);
  sParams.Add(0.93);

  // locals
  ULONG ii ;
  SmEdge          *pEdge     = GetEdge();
  SmExtent1d       sEdgeIvl  = pEdge->GetInterval() ;
  const SmEdgeuse *pEdgeuse1 = this;
  const SmEdgeuse *pEdgeuse2 = pEdgeuse1->GetRadial();
  SmSurface       *pSurface1 = pEdgeuse1->GetFace()->GetSurface() ;
  SmSurface       *pSurface2 = pEdgeuse2->GetFace()->GetSurface() ;

  SmPoint3d  sBinormal1,       sBinormal2 ;
  SmVector3d sBinormalVec1,    sBinormalVec2 ;
  SmVector2d sUV1,             sUV2 ;
  SmVector2d sUVDir1,          sUVDir2 ;
  SmVector3d aDirDerivs1[4],   aDirDerivs2[4] ;  // organized: [Pos, 1st Deriv, 2nd Deriv, 3rd Deriv ] 

  // for every parameter value
  for(ii=0; ii<sParams.GetSize(); ii++)
    {
      double dParam = sEdgeIvl.Evaluate(sParams[ii]);

      // Surface binormals
      SE_MSG(pEdgeuse1->EvaluateBinormal(dParam, TRUE, sBinormal1, sBinormalVec1, NULL, NULL, &sUV1, &sUVDir1),_T("Failed EvaluateBinormal")) ;
      SE_MSG(pEdgeuse2->EvaluateBinormal(dParam, TRUE, sBinormal2, sBinormalVec2, NULL, NULL, &sUV2, &sUVDir2),_T("Failed EvaluateBinormal")) ;
      sUVDir1.Unitize() ;
      sUVDir2.Unitize() ;

      // negate the UVdir for Surface2
      sUVDir2.Set(-sUVDir2.x, -sUVDir2.y) ;

      // Surface Directional Derivative Evaluations
      SE_MSG(pSurface1->EvaluateDirectionalDerivs(sUV1, sUVDir1, 3, aDirDerivs1),_T("Failed EvaluateDirectionalDerivs")) ;
      SE_MSG(pSurface2->EvaluateDirectionalDerivs(sUV2, sUVDir2, 3, aDirDerivs2),_T("Failed EvaluateDirectionalDerivs")) ;

      // Check continuity at this point
      SmContinuityType eContinuity ;
      SE_MSG(smgu_EvaluateCurveContinuity(aDirDerivs1[0],   aDirDerivs1[1],   aDirDerivs1[2],
                                          aDirDerivs2[0],   aDirDerivs2[1],   aDirDerivs2[2],
                                          eContinuity,      SM_CONTINUITY_ANGLE,
                                          &aDirDerivs1[0],  &aDirDerivs2[1]), _T("Failed smgu_EvaluateCurveContinuity")) ;
      // save the minimum continuity seen
      if(eContinuity < eRtn) eRtn = eContinuity ;

    } // end iter every sample point

  // all done
  return eRtn;

} // end SmEdgeuse::GetSectorContinuity

/*******************************************************************//**
PURPOSE: Check Edge/Face gap cached value against a freshly calculated one.

NOTES: return TRUE when they are the same, FALSE when they are the different
***********************************************************************/
SmBoolean SmEdgeuse::IsMaxEdgeFaceGapFresh
 (SmEdgeFaceGap & rCalcMaxEdgeFaceGap,     // out: copy of calculated Edge/Face Gap 
  SmEdgeFaceGap & rStoredMaxEdgeFaceGap)   // out: copy of stored Edge/Face Gap 
 const 
{
  SmBoolean bRtn = TRUE ; 
  if(IsMaxEdgeFaceGapInit())
    { 
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this->GetContext()) ;

      // calculate the gap without changing the cache
      ((SmEdgeuse *)this)->CalcMaxEdgeFaceGap( &rCalcMaxEdgeFaceGap, FALSE) ;

      // check that fresh gap == cached gap
      bRtn = SM_ARE_SAME_TO_TOL(rCalcMaxEdgeFaceGap, m_sMaxEdgeFaceGap3d, sXSectTol3d/1000.0) ;

      // finish output
      rStoredMaxEdgeFaceGap = m_sMaxEdgeFaceGap3d ;
    }
  return(bRtn) ; 

} // end SmEdgeuse::IsMaxEdgeFaceGapFresh

/*******************************************************************//**
PURPOSE: Check Edge/CCWEdge gap cached value against a freshly calculated one.

NOTES: return TRUE when they are the same, FALSE when they are the different
***********************************************************************/
SmBoolean SmEdgeuse::IsEdgeCCWEdgeGapFresh
 (SmEdgeEdgeGap & rCalcEdgeCCWEdgeGap,     // out: copy of calculated Edge/EdgeCCWEdge Gap 
  SmEdgeEdgeGap & rStoredEdgeCCWEdgeGap)   // out: copy of stored Edge/EdgeCCWEdge Gap 
 const 
{
  SmBoolean bRtn = TRUE ; 
  if(IsEdgeCCWEdgeGapInit())
    { 
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(this->GetContext()) ;

      // calculate the gap without changing the cache
      ((SmEdgeuse *)this)->CalcEdgeCCWEdgeGap( &rCalcEdgeCCWEdgeGap, FALSE) ;

      // check that fresh gap == cached gap
      bRtn = SM_ARE_SAME_TO_TOL(rCalcEdgeCCWEdgeGap, m_sEdgeCCWEdgeGap3d, sXSectTol3d/1000.0) ;

      // finish output
      rStoredEdgeCCWEdgeGap = m_sEdgeCCWEdgeGap3d ;
    }
  return(bRtn) ; 

} // end SmEdgeuse::IsEdgeCCWEdgeGapFresh

/*******************************************************************//**
PURPOSE: Return True when Edgeuse is connected to given Target Topology object

NOTES: 1. the input Target cpConnectTgt may be NULL, or of type,
             SmVertex,
             SmEdge,
             SmLoop,
             SmFace,
             SmShell,
             SmRegion
        2. returns TRUE for NULL and for any other unsupported Topology TYPE
***********************************************************************/
SmBoolean SmEdgeuse::IsConnectedTo
  ( const SmTopology *cpConnectTgt )    // in : target Topology
 const
{
  // no work - no Topo
  if(cpConnectTgt == NULL) 
    { return TRUE ; }

  // switch on cpConnectTgt type
  switch(cpConnectTgt->GetType())
    {
      case SmVertexuse_TYPE: { return( ((SmVertexuse *)cpConnectTgt)->GetEdgeuse() == this ) ; 
                             } break ;
      case SmVertex_TYPE   : { return( GetVertexuse() && GetVertexuse()->GetVertex() == (SmVertex *)cpConnectTgt ) ;
                             } break ;
      case SmEdgeuse_TYPE  : { return( (SmEdgeuse *)cpConnectTgt == this ) ;
                             } break ; 
      case SmEdge_TYPE     : { return( GetEdge() == (SmEdge *)cpConnectTgt ) ; 
                             } break ;
      case SmFace_TYPE     : { return( GetFace() == ((SmFace *)cpConnectTgt) ) ;
                             } break ;
      case SmFaceuse_TYPE  : { return( GetFaceuse() == ((SmFaceuse *)cpConnectTgt) ) ;
                             } break ;
      case SmLoop_TYPE     : { return( GetLoopuse() && GetLoopuse()->GetLoop() == (SmLoop *)cpConnectTgt ) ;
                             } break ;
      case SmLoopuse_TYPE  : { return( GetLoopuse() == (SmLoopuse *)cpConnectTgt ) ;
                             } break ;
      case SmShell_TYPE    : { return( GetShell() == (SmShell *)cpConnectTgt) ;
                             } break ;
      case SmRegion_TYPE   : { return( GetShell() && GetShell()->GetRegion() == (SmRegion *)cpConnectTgt ) ;
                             } break ;
      default: break ;

    } // end switch on type

  // arrive here when cpConnectTgt is an unsupported type - return TRUE
  return(TRUE) ;

} // end SmEdgeuse::IsConnectedTo

/*******************************************************************//**
PURPOSE: Return FALSE when Edgeuse is on and tangent to a Surface boundary 
and the edgeuse side bounding 'into' the face is pointing outside the SurfaceUVDomain.

NOTES: Ths predicate tests for an illegal database configuration.
       A heal function will have to be built that fixes this condition.
       rtn: TRUE = OK use of edgeuse  - its bounding a region in its Face->Surface.
            FALSE= bad use of edgeuse - its bounding a region off its Face->Surface.

       This method should not call any function that causes a UVTrimCurve to
       be constructed.  This method is here to work on databases with errors as part
       of the heal functions.  When a database has problems, depending on
       functions as database sensitive as CreateUVTrimCurve will often result
       in coupling database problems together.  As such this method does
       not call EvaluateBinormal() instead it samples the edgeuse's edge projection
       onto the edgeuse's surface to make its best guess to classify the edge as
       lieing on the surface's natural boundary and if so in what direction it is running.

       Attempts to handle cases of degenerate edges dropping to a single point which might
       be a pole on a surface (its illegal to have degenerate edges) and cases where
       edges cross unmarked surface seams (its only legal if an edge gets split at the
       surface seam).

       returns TRUE (OK) for Edgeuses not yet connected to Faces.
***********************************************************************/
SmBoolean SmEdgeuse::IsEdgeuseSideInSurface
 (ULONG       lSmpCount,             // in : Number of sample points along the Edgeuse->Edge->Curve, default:[6]
  SmBoolean * pOptPeriodicU,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]
  SmBoolean * pOptPeriodicV,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]
  SmBoolean * pOptOnNaturalBoundary) // in : TRUE=edgeuse is on SurfNatBndry, FALSE=Not, NULL to ignore, default:[NULL]
 const 
{ 
  // init return and ouput vals
  SmBoolean bRtn = TRUE ;
  if(pOptOnNaturalBoundary) { *pOptOnNaturalBoundary = TRUE ; }

  // locals
  ULONG ii ; 
  SmFace    * pFace     = GetFace() ;
  SmSurface * pSurface  = pFace ? pFace->GetSurface() : NULL ;
  SmEdge    * pEdge     = GetEdge() ;

  // no work - missing Face, Surface, or Edge
  if(pFace == NULL || pSurface == NULL || pEdge == NULL)
    { return TRUE ; }

  // classify periodic surface
  SmExtent2d          sUVDomain     = pSurface->GetNaturalUVDomain() ;
  SmBoolean           bPeriodicU    = pOptPeriodicU ? *pOptPeriodicU : pSurface->IsPeriodic(sUVDomain, SM_SP_U) ;
  SmBoolean           bPeriodicV    = pOptPeriodicV ? *pOptPeriodicV : pSurface->IsPeriodic(sUVDomain, SM_SP_V) ;

  // locals
  SmCurve           * pEdgeCurve    = pEdge->GetCurve() ;
  SmExtent1d          sEdgeInterval = pEdge->GetInterval() ;
  SmPoint3d           sEdgePoint, sUVTrimCurve3d ;
  SmBoolean           bOrientation  = GetOrientation() ;
  const SmPoint2d   * pUVGuessPoint = NULL ;
  SmTArray<SmPoint2d> sUVDropPoints ;     // UV point per SamplePoint
  SmTArray<ULONG>     sUVPointsClassify ; // (SM_SS_UMIN | SM_SS_VMIN | SM_SS_UMAX | SM_SS_VMAX) per SamplePoint
  ULONG               lUVClassify = (SM_SS_UMIN | SM_SS_VMIN | SM_SS_UMAX | SM_SS_VMAX) ;
  SmPoint2d           sUVDropPoint, sUVGuessPoint, sUDir(1,0), sVDir(0,1) ; 
  SmBoolean           bSuccess = false;
  double              dGap = 0.0;
  SmBoolean           bIsMulti;
  ULONG               lUpU=0, lDownU=0, lUpV=0, lDownV=0 ;
  SmExtent3d          sEdgeBox3d ;
  SmBSplineCurve    * pUVTrimCurve = GetUVTrimCurve() ; // rtn: UVTrimCurve from Edgeuse or Mate. NULL if none attached

  // Tol = 3d Surface Zone Tol mapped for each sample point to 2d through the surface function
  SmZoneTol3d sSurfZoneTol3d = SmTol::GetZoneTol3d(pSurface) ;

  // for every sample point - check for Edgeuse point on Surface boundary whose inside direction points out of the Surface domain
  for(ii=0;ii<lSmpCount;ii++)
    {
      // param
      double dNormParam =   (bOrientation == SM_OT_SAME) ? (  ii == 0 ? 0.0
                                                            : ii == lSmpCount - 1 ? 1.0
                                                            : (double)ii / (double)(lSmpCount - 1))
                                                         : (  ii == 0 ? 1.0
                                                            : ii == lSmpCount - 1 ? 0.0
                                                            : 1.0 - ((double)ii / (double)(lSmpCount-1))) ;
      double dParam     = sEdgeInterval.Evaluate(dNormParam) ;

      // Edge->3d Point
      pEdgeCurve->EvaluatePoint( dParam, sEdgePoint );

      // next Drop GuessUV = UVTrimCurve(dParam) or last Drop point 
      pUVGuessPoint = NULL;
      if(pUVTrimCurve != NULL)
      {
        pUVTrimCurve->EvaluatePoint( dParam, sUVTrimCurve3d );
        sUVGuessPoint = sUVTrimCurve3d;
        pUVGuessPoint = &sUVGuessPoint;
      }
      else if(ii > 0)
      {
        pUVGuessPoint = &(sUVDropPoints.GetAt( ii - 1 ));
      }

      // Drop Point
      pSurface->DropPoint(sEdgePoint, // in : target point to drop
                          sUVDomain, // in : target surface domain
                          pUVGuessPoint, // in : When given, uses only local solves
                          bSuccess, // out: TRUE=Point dropped successfully, FALSE=didn't
                          sUVDropPoint, // out: drop result UVPoint
                          dGap, // out: distance of found point to Pt to drop
                          bIsMulti, // in : SM_SO_MINIMIZE = allow nonNormal drops near boundaries
                          SM_SO_MINIMIZE); //      SM_SO_NORMALIZE= exclude nonNormal drops near boundaries
                                                          //      SM_SO_INTERSECT= point must be on surface, to Tol
      // Tol  2d mapping of Surface ZoneTol3d
      SmTol2d sTol2dU = SmTol::MapTo2d(sSurfZoneTol3d, sUVDropPoint, sUDir, *pSurface) ; 
      SmTol2d sTol2dV = SmTol::MapTo2d(sSurfZoneTol3d, sUVDropPoint, sVDir, *pSurface) ;

      // classify sUVPoint - to XSectTol3d numerical tolerances mapped to 2d (diff in U and V directions)
      ULONG sUVLocU = sUVDomain.GetPoint2dBoundaries(sUVDropPoint, 2*sTol2dU) & (SM_SS_UMIN | SM_SS_UMAX) ;
      ULONG sUVLocV = sUVDomain.GetPoint2dBoundaries(sUVDropPoint, 2*sTol2dV) & (SM_SS_VMIN | SM_SS_VMAX) ;
      ULONG sUVLoc  = sUVLocU | sUVLocV ;

      // save the drop and classify data per sample point
      sEdgeBox3d.AddPoint3d(sEdgePoint) ;
      sUVDropPoints.Add(sUVDropPoint) ;
      sUVPointsClassify.Add(sUVLoc) ; 

      // accumulate the classifications and the UV incremental step directions
      lUVClassify &= sUVLoc ;
      if(ii>0) 
        {
          if(sUVDropPoints[ii-1].x < sUVDropPoints[ii].x) { lUpU ++ ; }
          if(sUVDropPoints[ii-1].x > sUVDropPoints[ii].x) { lDownU ++ ; }
                                                              
          if(sUVDropPoints[ii-1].y < sUVDropPoints[ii].y) { lUpV ++ ; }
          if(sUVDropPoints[ii-1].y > sUVDropPoints[ii].y) { lDownV ++ ; }
        }

       // low work - found a classify point not on a boundary       
       if(lUVClassify == 0)
         { 
           // both sides of this edgeuse are inside at this point, so return TRUE
           if(pOptOnNaturalBoundary) { *pOptOnNaturalBoundary = FALSE ; }
           return(TRUE) ;
         }
    } // end iter dropping every sample point
     
  // arrive here when sUVClassify != 0, i.e. all sample points share some common boundary
  // the lUpU, lDownU, lUpV, and lDownV capture Edge direction, expect one out of sequence step when edge crosses an unmarked seam

  // any Edgeuse with an Edge which is degenerate (a database problem to be healed) is classified as having a side inside the surface
  if(sEdgeBox3d.GetMaxDimension() < 2.0 * sSurfZoneTol3d)
    {
      return(TRUE) ; 
    }

  // check direction for edges running a natural boundary to determine if the edgeuses inside-side is in or out of the surface domain
  //  note: 1. if edge crosses an unmarked seam (a database problem to be healed) One increment count will be in the wrong direction.
  //        Only require all but one of the increments to be in the right direction to classify edgeuse inside-side as in or out of surface domain
  //        2. When dropping to seams the drop points may be switched to the other side when the UVTrimCurve is missing

  //        3. When sampling along a seam without a UVTrimCurve we currently assume the curve can be on either side of the 
  //           seam and always return TRUE.  That's a problem. To detect an edgeuse running the wrong way on the seam we
  //           have to compare the start and ends of the sample sequence with the end of the last edgeuse in the loop
  //           and the beginning of the next edgeuse in the loop.  That check can be coded up later using
  //             pNextLoopEdgeuse = m_pCCW
  //             pLastLoopEdgeuse = m_pCW
  //             and dropped sample points near the end of LastLoopEdgeuse and the beginning of NextLoopEdgeuse
  //                 to determine which side of the seam this Edgeuse should be upon.  Then Straighten out the
  //                 lUVClassify values switching SM_SS_UMIN/SM_SS_UMAX or SM_SS_VMIN/SM_SS_VMAX values as appropriate
  //                 and then removing the 2nd half of every conditional below.

  if     (lUVClassify & SM_SS_UMIN) 
                                    { bRtn =    (lDownV >= lSmpCount - 2)  // IsoParam along const x = sUVDomain.MinU, V values should descend
                                             || (pUVTrimCurve == NULL && bPeriodicU && lUpV   >= lSmpCount - 2) ; } // assume samples on wrong side of seam
  else if(lUVClassify & SM_SS_UMAX) 
                                    { bRtn =    (lUpV   >= lSmpCount - 2)  // IsoParam along const x = sUVDomain.MaxU, V values should ascend 
                                             || (pUVTrimCurve == NULL && bPeriodicU && lDownV >= lSmpCount - 2) ; } // assume samples on wrong side of seam
  else if(lUVClassify & SM_SS_VMIN)          
                                    { bRtn =    (lUpU   >= lSmpCount - 2)  // IsoParam along const x = sUVDomain.MinV, U values should ascend 
                                             || (pUVTrimCurve == NULL && bPeriodicV && lDownU >= lSmpCount - 2) ; } // assume samples on wrong side of seam
  else if(lUVClassify & SM_SS_VMAX)          
                                    { bRtn =    (lDownU >= lSmpCount - 2)  // IsoParam along const x = sUVDomain.MaxV, V values should descend 
                                             || (pUVTrimCurve == NULL && bPeriodicV && lUpU   >= lSmpCount - 2) ; } // assume samples on wrong side of seam
  else                                    
                                    { /* something unexpected */ bRtn = TRUE ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

  if(bDebugMe)
    {
      SmBrep    * pBrep    = pEdge ? pEdge->GetBrep() : NULL ;
      SmFace    * pF    = GetFace() ;
      SmSurface * pSrf = pF ? pF->GetSurface() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep)    { pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;   }
      smgfx_SetLook(2,3, 1,0,0) ; if(pEdge)    { pEdge->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,1,1) ; if(pSrf)     { pSrf->DrawUV() ; sm_GraphicsLoop() ;  }
      smgfx_SetLook(4,5, 1,0,1) ; this->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ; 

} // end SmEdgeuse::IsEdgeuseSideInSurface


/*******************************************************************//**
PURPOSE: Static helper: Set the Distance and optionally the Deviation of
   a point-surface drop.

NOTES: 
   The Distance is the distance from the drop point to the normal plane
   (along the surface normal) and the Deviation is the distance in the
   normal plane from the surface point to the projection of the drop point.

   If the optional pDev is Null, rDist is just the 3d distance between
   the drop point and the surface point.
***********************************************************************/
static void sm_CalcDistAndDev( const SmPoint3d  &rPt,   // in: Point that was dropped
                               const SmSolution & rSol, // in: Point-drop solution
                               double & rDist,          // out: Dist from surface
                               double * pDev )          // out: Dist from normal
{
  if ( pDev == NULL )
  {
      rDist = rSol.m_vStart.m_dSolutionValue;
      return;
  }

  // pDev is not Null.

  rDist = -1.0;
  *pDev = -1.0;

  SmSurface *pSrf = SM_CAST_PTR( SmSurface, rSol.m_apObjects[0] );
  if ( pSrf == NULL ) { return; }

  // Find distance (along surface normal) and deviation (in normal plane).
  SmPoint2d sUV( rSol.m_vStart.m_adParameters[0], rSol.m_vStart.m_adParameters[1] );
  SmPoint3d sPt;
  SmVector3d sSu, sSv;
  pSrf->Evaluate1stDerivatives( sUV, TRUE, TRUE, sPt, sSu, sSv );

  SmVector3d sNorm( sSu * sSv );
  double dMag = sNorm.Length();
  if ( dMag > SM_EFF_ZERO )
    { sNorm /= dMag; }
  else
    {
      pSrf->EvaluateNormal( sUV, TRUE, TRUE, sNorm );
      if ( sNorm.Length() < 0.5 )
        { return; }
    }

  SmVector3d sDiff( rPt - sPt );
  double dDot = sDiff.Dot( sNorm );
  rDist = smos_Fabs( dDot );

  double d3dDist = sDiff.Length();

  // Pythagorean:
  *pDev = ( rDist >= d3dDist ) ? 0.0    // (Shouldn't happen, just a check for Sqrt.)
                               : smos_Sqrt( d3dDist*d3dDist - rDist*rDist );

  return;

} // end static sm_CalcDistAndDev

/*******************************************************************//**
PURPOSE: Decide whether uv curves represent a surface seam.

NOTES: 
   We could check this using SmEdge::IsSeam(), but that calls
   SmEdgeuse::NormalizedEvaluate(), which calls CreateUVTrimCurve().
***********************************************************************/
static SmBoolean sm_AreSeamCurves
 ( const SmEdgeuse                   * cpEdgeuse,         // NotUsed: in :
   const SmTArray< SmBSplineCurve* > & crUVCurves,        // in :
   const SmExtent1d                  & sEdgeDomain,       // in :
         SmBoolean                     bDropCurveCalled)  // in :
{
  SM_REF1(cpEdgeuse) ;
  if ( crUVCurves.GetSize() != 2 )
    { return FALSE; }

  if ( bDropCurveCalled )
    { return TRUE; }  // DropCurve() will never return two curves except for seams.

  // Check curve domains.
  SmCurve *pUVCurve1 = (SmCurve*)crUVCurves[0];
  SmCurve *pUVCurve2 = (SmCurve*)crUVCurves[1];
  SmExtent1d sDom1 = pUVCurve1->GetNaturalInterval();
  SmExtent1d sDom2 = pUVCurve2->GetNaturalInterval();
  if ( sDom1 == sDom2 ) // This would be a very tight agreement.
    {
      // The check vs. the Edge domain can have a little slop?  Say 1%.
      double dTol = sEdgeDomain.GetLength() / 100.0;
      if ( sDom1.AreEqual( sEdgeDomain, dTol ) )
        { return TRUE; }
    }

  // State: 2 uv curves; DropAndTrimCurve was called; all three domains don't agree.
  // Could also check:
  // - seam crossing would mean that sDom1 + sDom2 == sEdgeDomain,
  //    i.e., sDom1 is first part of sEdgeDomain and sDom2 is the rest.
  // - For a seam, corresponding points (say midpoint) on interiors of the two
  //    uv curves are on opposite sides of a seam in uv space, and coincident in 3d.
  // Or, the curve could leave the surface boundaries and then re-enter.
  // That would be the same as the seam-crossing, as far as we're concerned.
  // However, the domains argument stitches it up I think.
  // Not going to check for a partial coincidence with a seam.
  return FALSE;

} // end sm_AreSeamCurves

/*******************************************************************//**
PURPOSE: Create (but do not attach) a UVTrimCurve corresponding to this edgeuse.
   Ignore any UVTrimCurve that may be attached to this edgeuse.

RETURN: SmStatus SM_SUCCESS = Newly built or preexisting WellMade UVTrimCurve

NOTES:
   In the typical case, the UVTrimCurve is created with
   SmSurface::DropCurve(GetEdge()->GetCurve(), tol=GetEdge()->GetTolerance()/2.0) ;

   Special case code exists to handle
   1. curves on surface seams,
   2. failures of the DropCurve function,
   3. cases where the Edgeuses curve extends beyond the domain of the surface

   When no UVTrimCurve can be constructed for this edge return SM_ERR.

   In the prog_test test suite all calls to this function are
   handled either as the typical case or the seam case.

   The special cases are not expected to be used.

METHOD ---

  TRY 1: SmCrvOnSurf special case -
    let rpNewUVCurve = Copy of SmCrvOnSurf->GetUVCurve

  TRY 2:
    a. call SmSurface::DropCurve(); if that fails,
       call SmSurface::DropAndTrimCurve()
       When dropping fails - reverse curve parameterization and try again

       DropCurve typically returns just one UVTrimCurve but it
       can return two curves when the curve is exactly on a surface seam boundary.
       DropAndTrimCurve can return any number of curves when the curve enters and leaves
       the surface boundary one or more times.

    b. When two UVTrimCurves are generated, test for a shorter-than-tolerance
       seam crossing.  If either uv curve is shorter than tolerance (in 3-space),
       remove it and use the other.

    c. When two UVTrimCurves are generated, test for seam curves.
       The UVTrimCurve associated with this Edgeuse is used and the other
       is freed.  A general tolerance test applied at a near-midPoint
       is tried first to pick the best seam possible.  When that fails
       geometry tests on the binormal direction is used for near isoParameter
       curves.  If that fails, try DropAndTrimCurve.

    d. when there are more than 1 UVTrimCurve generated because the
       curve weaved in and out of the surface,

       1. use any fragment more than 90% of the curve's length
          when no such fragment exists - try again with DropAndTrimCurve.

       2. There are conditions under which the edge is trimmed back to
           where it leaves or enters the surface.  This should happen
           only for differences less than tolerance.

  TRY 3: When DropCurve and DropAndTrimCurve both fail -
     sample the curve 101 times and drop points to the surface.
     Return a TrimCurve constructed by interpolating the
     drop points.  The 3D distance between sample points is tested to see
     if the curve maps to a degenerate boundary of the Surface.

  When all 3 tries fail to construct a UVTrimCurve, SM_ERR is returned
     and rpNewUVCurve is set to NULL.

***********************************************************************/
SmStatus SmEdgeuse::CreateUVTrimCurve
( 
  double          & rdMaxDistToSurf,       // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
  SmBSplineCurve *& rpNewUVCurve,          // out: Edgeuse's UVTrimCurve
  double          * pdMaxDeviation,        // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                           //      Default: NULL
  SmDropCurveFail * pOptDropCurveFail      // out: Opt state at time of DropCurve fail or success, NULL to ignore, default:[NULL]
) const
{
  // always collect DropCurveData
  SmDropCurveFail sDropCurveFail, * pDropCurveFail = pOptDropCurveFail ? pOptDropCurveFail : &sDropCurveFail ;

  // init output
  rpNewUVCurve = NULL;
  pDropCurveFail->UnInit() ;
  pDropCurveFail->m_pBrep = GetBrep() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe  = FALSE ;
SmBoolean bDebugMe2 = FALSE ;
SmValidityCheckType eCheckFailed ;
  if(bDebugMe)
    {
      // leads to recursive call - GetFace()->Dump(FALSE) ;
      // leads to recursive call - GetEdge()->Dump(FALSE) ;

      SmEdge *pEdge = GetEdge() ;
      SmFace *pFace = GetFaceuse()->GetFace() ;
      SmBrep *pBrep = pEdge->GetBrep() ? pEdge->GetBrep() : pFace->GetBrep() ;

      pFace->GetSurface()->Dump() ;
      pEdge->GetCurve()->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,3,  0, 0, 1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3,  0, 1, 0) ; pFace->GetSurface()->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6,  0, 0, 1) ; pFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,5,  0, 1, 1) ; pFace->GetSurface()->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,6,  0, 1, 1) ; pFace->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,7,  1, 0, 0) ; pEdge->GetCurve()->DrawParams() ; sm_GraphicsLoop() ;
      // recursion: smgfx_SetLook(5,6,  0, 1, 1) ; this->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  SmEdge          *pEdge        = GetEdge();
  SmFaceuse       *pFU          = GetFaceuse(); NER(pFU);
  SmFace          *pF           = pFU->GetFace();
  SmSurface       *pSurface     = pF->GetSurface();
  const SmContext *pContext     = pF->GetContext();
  SmExtent2d       sFaceDomain  = pF->GetUVDomain();
  SmCurve         *pCurve       = SM_CAST_PTR( SmCurve, pEdge->GetCurve() );

  SmExtent1d       sEdgeDomain  = pEdge->GetInterval();
  SmCrvOnSurf     *pCrvOnSurf   = SM_CAST_PTR( SmCrvOnSurf, pEdge->GetCurve() );
  SmBSplineCurve  *pUVTrimCurve = GetUVTrimCurve() ;

  // DropCurve output arguments.
  // If we have a CrvOnSurf for a seam, we will populate sUVCurves directly and skip DropCurve
  SM_PTR_ARRAY(sUVCurves, SmBSplineCurve, 16);
  double dMaxDropApproxDev = 0.0;
  SmBoolean bHaveCurves = FALSE;

  if (pCrvOnSurf && ((pSurface == pCrvOnSurf->GetBaseSurface()) || ( *pSurface == *(pCrvOnSurf->GetBaseSurface()))) &&
      (GetOrientation() == GetEdge()->GetPrimaryEdgeuse()->GetOrientation()))
  {
      const SmBSplineCurve* pUVCurve = SM_CAST_PTR(SmBSplineCurve, pCrvOnSurf->GetUVCurve());
      SmExtent2d sSurfaceUV = pSurface->GetNaturalUVDomain();
      SmTol3d sTol = this->GetTolerance();
      

      SmPoint3d sStartPt(0.0, 0.0, 0.0), sEndPt(0.0, 0.0, 0.0), sOtherPoint(0.0, 0.0, 0.0);

      pUVCurve->GetEnds(sStartPt, sEndPt);

      SmExtent3d sUVBBox;
      pUVCurve->CalculateBoundingBox(pUVCurve->GetNaturalInterval(), &sUVBBox);

      // Checking here to ensure that if this is a seam, we make sure to pick the correct UV line.
      // If this is not a line, then we're good. Then check to see if we are along an edge of the
      // domain for a periodic surface, aka this edgeuse represents a seam.

      double sULength = sUVBBox.GetUInterval().GetLength();
      double sVLength = sUVBBox.GetVInterval().GetLength();
      //SmBoolean bUPeriodic = pSurface->IsPeriodic(sSurfaceUV, SM_SP_U);
      //SmBoolean bVPeriodic = pSurface->IsPeriodic(sSurfaceUV, SM_SP_V);
      //double sUMinDist = smos_Fabs(sStartPt.x - sSurfaceUV.GetUMin());
      //double sUMaxDist = smos_Fabs(sStartPt.x - sSurfaceUV.GetUMax());
      //double sVMinDist = smos_Fabs(sStartPt.y - sSurfaceUV.GetVMin());
      //double sVMaxDist = smos_Fabs(sStartPt.y - sSurfaceUV.GetVMax());

      if (sULength <  sTol &&
          ((smos_Fabs(sStartPt.x - sSurfaceUV.GetUMin()) <  sTol) ||
           (smos_Fabs(sStartPt.x - sSurfaceUV.GetUMax()) < sTol)) &&
               (pSurface->IsPeriodic(sSurfaceUV, SM_SP_U)))
           {
              // Must be a seam with constant U, so build the other possible UV curve.

              SmBSplineCurve* pOtherLine;

              sOtherPoint.x = (smos_Fabs(sStartPt.x - sSurfaceUV.GetUMin()) < sTol.val) ? sSurfaceUV.GetUMax() :
                                                                                          sSurfaceUV.GetUMin();
              sOtherPoint.y = sStartPt.y;
              // sOtherPoint.z = 0.0;
              //  SmVector3d sLineVec = pUVLine->GetLineVector();
              //  SmExtent1d sLineIvl= pUVLine->GetNaturalInterval();
              SmBSplineCurve* pCopyUVLine;

              SER(pUVCurve->Copy(*pContext, pOtherLine));
              NER(pOtherLine);

              SmAxis2Placement sTransformAxis;
              SmVector3d sXAxis(1.0, 0, 0);
              SmVector3d sYAxis(0, 1.0, 0);

              sTransformAxis.SetCanonical(sOtherPoint - sStartPt, sXAxis, sYAxis);

              pOtherLine->Transform(sTransformAxis);
              pOtherLine->ConvertTo2D();

              SER(pUVCurve->Copy(*pContext, pCopyUVLine));
              pCopyUVLine->ConvertTo2D();
              NER(pCopyUVLine);

              // SM_CAST_PTR(SmLine, pCopy2UVLine)->SetCanonical(sOtherPoint, sLineVec, pUVLine->GetLineScale(),
              // sLineIvl);

              // if (GetOrientation() != GetEdge()->GetPrimaryEdgeuse()->GetOrientation())
              //{
              //      SmExtent1d sNewIvl;
              //      pCopy1UVLine->ReverseParameterization(sLineIvl, sNewIvl);
              //      pCopy2UVLine->ReverseParameterization(sLineIvl, sNewIvl);
              // }

              sUVCurves.Add(SM_CAST_PTR(SmBSplineCurve, pOtherLine));
              sUVCurves.Add(SM_CAST_PTR(SmBSplineCurve, pCopyUVLine));

              pOtherLine->SetOwner((SmEdgeuse*)this);
              pCopyUVLine->SetOwner((SmEdgeuse*)this);

              bHaveCurves = TRUE;

              rdMaxDistToSurf = 0.0;
              if (pdMaxDeviation)
              {
                  *pdMaxDeviation = 0.0;
              }
           }
           else if (sVLength < sTol &&
                    ((smos_Fabs(sStartPt.y - sSurfaceUV.GetVMin()) < sTol) ||
                     (smos_Fabs(sStartPt.y - sSurfaceUV.GetVMax()) < sTol)) &&
                    (pSurface->IsPeriodic(sSurfaceUV, SM_SP_V)))
           {
              // Must be a seam with constant V, so build the other possible UV curve.

              SmBSplineCurve* pOtherLine;

              sOtherPoint.y = (smos_Fabs(sStartPt.y - sSurfaceUV.GetVMin()) < sTol.val) ? sSurfaceUV.GetVMax() :
                                                                                          sSurfaceUV.GetVMin();
              sOtherPoint.x = sStartPt.x;
              // sOtherPoint.z = 0.0;

              // SmVector3d sLineVec = pUVLine->GetLineVector();
              // SmExtent1d sLineIvl= pUVLine->GetNaturalInterval();
              SmBSplineCurve* pCopyUVLine;

              SER(pUVCurve->Copy(*pContext, pOtherLine));
              NER(pOtherLine);

              SmAxis2Placement sTransformAxis;
              SmVector3d sXAxis(1.0, 0, 0);
              SmVector3d sYAxis(0, 1.0, 0);
              sTransformAxis.SetCanonical(sOtherPoint - sStartPt, sXAxis, sYAxis);

              pOtherLine->Transform(sTransformAxis);
              pOtherLine->ConvertTo2D();

              SER(pUVCurve->Copy(*pContext, pCopyUVLine));
              pCopyUVLine->ConvertTo2D();
              NER(pCopyUVLine);

              // SM_CAST_PTR(SmLine, pCopy2UVLine)->SetCanonical(sOtherPoint, sLineVec, pUVLine->GetLineScale(),
              // sLineIvl);

              // if (GetOrientation() != GetEdge()->GetPrimaryEdgeuse()->GetOrientation())
              //{
              //      SmExtent1d sNewIvl;
              //      pCopy1UVLine->ReverseParameterization(sLineIvl, sNewIvl);
              //      pCopy2UVLine->ReverseParameterization(sLineIvl, sNewIvl);
              // }

              sUVCurves.Add(pOtherLine);
              sUVCurves.Add(pCopyUVLine);

              pOtherLine->SetOwner((SmEdgeuse*)this);
              pCopyUVLine->SetOwner((SmEdgeuse*)this);

              bHaveCurves = TRUE;

              rdMaxDistToSurf = 0.0;
              if (pdMaxDeviation)
              {
                  *pdMaxDeviation = 0.0;
              }
           }
           else // Not a seam, so just copy the curve
           {
              SmBSplineCurve* pCopyCurve;
              SER(pUVCurve->Copy(*pContext, pCopyCurve));
              pCopyCurve->ConvertTo2D();
              NER(pCopyCurve);

              // if (GetOrientation() != GetEdge()->GetPrimaryEdgeuse()->GetOrientation())
              //{
              //    SmExtent1d sNewIvl;
              //    SmExtent1d sOldIvl = pCopyCurve->GetNaturalInterval();
              //    pCopyCurve->ReverseParameterization(sOldIvl, sNewIvl);
              // }

              rpNewUVCurve = pCopyCurve;
              NER(rpNewUVCurve);
              rdMaxDistToSurf = 0.0;
              if (pdMaxDeviation)
              {
                  *pdMaxDeviation = 0.0;
              }
              rpNewUVCurve->SetOwner((SmEdgeuse*)this);

              // all done

#ifdef SM_DEBUG_CODE
              if (bDebugMe2)
              {
                  SM_ASSERT_VALID(rpNewUVCurve);
                  rpNewUVCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed);
                  rpNewUVCurve->PassesValidityCheck(SM_VC_ALL, eCheckFailed);
              }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
              if (rpNewUVCurve->GetDim() != 2)
              {
                  SM_ASSERT_MSG(
                      rpNewUVCurve->GetDim() == 2, _T("SmEdgeuse::CreateUVTrimCurve created a non-2D UVTrimCurve"));
              }
#endif // SM_DEBUG_CODE

              SM_SET_CREATE_UVTRIMCURVE_FAIL(SM_SUCCESS, SM_CTC_FROM_CRVONSURF);

              return SM_SUCCESS;
           }
      } // end low work - SmCrvOnSurf_TYPE check


#ifdef SM_DEBUG_CODE
  TCHAR sBuff[SM_TBLOCK_SIZE];
  SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3) 
    {
      SmEdge *pEdge1 = GetEdge() ;
      SmFace *pFace = GetFaceuse()->GetFace() ;
      SmBrep *pBrep = pEdge1->GetBrep() ? pEdge1->GetBrep() : pFace->GetBrep() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,3,  0, 0, 1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3,  0, 1, 0) ; pFace->GetSurface()->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, .1,.1,.1) ; pFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,5,  0, 1, 1) ; pFace->GetSurface()->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,6,  0, 1, 1) ; pFace->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,7,  1, 0, 0) ; pEdge1->GetCurve()->DrawParams() ; sm_GraphicsLoop() ;
      // recursion: smgfx_SetLook(5,6,  0, 1, 1) ; this->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
  SmBoolean bDebugMe7 = FALSE;
  if (bDebugMe7) 
    { SmEdge *pEdge2 = GetEdge() ;
      SmFace *pFace = GetFaceuse()->GetFace() ;
      smos_sprintf(sBuff, _T("\nSmEdgeuse::CreateUVTrimCurve(): Edge # %lu, Face # %lu\n"),
                  pEdge2->GetEdgeNumberInBrep(),
                  pFace->GetFaceNumberInBrep() );
      smos_WriteBuffer(sBuff);
      pEdge2->GetCurve()->Dump();
    }
#endif // SM_DEBUG_CODE

  // Do not trim to Face boundaries:
  SmTemporaryChangeValue<SmObject*> sTON; 
  //JLMCC for parallel code, to avoid data race
  if (pSurface->m_pOwner) {
    sTON.Set(pSurface->m_pOwner, NULL);
  }

  SmZoneTol3d sZoneTol3d = pEdge->GetTolerance() ;
  SmStatus lResult ;

  // iter 0 = DropCurve(), 1 = DropAndTrimCurve() only if DropCurve() fails 
  // iters 2 and 3 = same with bigger tolerance.
  // DropAndTrimCurve() will find curves that are just slightly too long,
  // extending just a bit across the surface boundary; DropCurve() returns
  // nothing in that case.
  ULONG ii, jj;
  for(ii=0;ii<4;ii++ )
    {

      // drop curve to surface
      SmBoolean bDropCurveCalled = FALSE;
    if (!bHaveCurves)
    {
              if (ii == 2)
              {
                  // No success at original tolerance.  Try a different tolerance based on gaps.
                  SmEdgeFaceGap* pEdgeFaceGap = GetMaxEdgeFaceGap();

                  // if the gap size says to increase tolerances, try that
                  // else double the tolerance
                  if (pEdgeFaceGap->GetLength() * 2.0 > sZoneTol3d)
                  {
                      sZoneTol3d = pEdgeFaceGap->GetLength() * 2.0;
                  } // 2.0 works well [B573]
                  else
                  {
                      sZoneTol3d *= 2.0;
                  }
              }

              // drop curve to surface

              if (ii == 0 || ii == 2)
              {
                  bDropCurveCalled = TRUE;
                  lResult = pSurface->DropCurve(*pContext,                 // in : context for new object construction
                                                sFaceDomain,               // in : domain of interest for this surface
                                                *pCurve,                   // in : Curve to project onto the surface
                                                sEdgeDomain,               // in : interval of interest for target curve
                                                (SmApproxTol3d)sZoneTol3d, // in : max allowed distance between drop
                                                                           //      point and m_SrfNormal line at drop point
                                                rdMaxDistToSurf,           // out: MaxDist(DropPt, 3dCurvePt), found by sampling,
                                                                           //      may be slightly less than actual max.
                                                dMaxDropApproxDev,         // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt),
                                                                           //      Quality Measure: expected to be close to 0.0
                                                sUVCurves,                 // out: 1 (or 2) curves constructed by projection.
                                                                           //      (2 curves for closed surfaces when cr3dCurve is
                                                                           //      coincident with seam)
                                                FALSE,                     // in : TRUE = return all drop curves that stay over the surface
                                                                           //      (those that wander off do not drop),
                                                                           //      FALSE= only return drop curves with drop distances less
                                                                           //      than dApproxTol FALSE preserves old behavior [B260]
                                                                           //      default:[TRUE]
                                                pDropCurveFail);           // out: Optional data container for DropCurve diagnostics
                                                                           //      and recovery-path reporting,
                                                                           //      NUll to ignore, default:[NULL]
              } // end (ii==0 || ii==2) - Try DropCurve branch
              else // (ii==1 || ii=3) - Try DropAndTrimCurve branch
              {
                  bDropCurveCalled = FALSE;
                  lResult = pSurface->DropAndTrimCurve(*pContext,                 // in : context for newly created geometry
                                                       sFaceDomain,               // in : Domain limits for accepting the dropped
                                                                                  //      curve
                                                       *pCurve,                   // in : Curve to project onto the surface
                                                       sEdgeDomain,               // in : cr3dCurve interval to drop
                                                       (SmApproxTol3d)sZoneTol3d, // in : This tolerance is used to
                                                                                  //      control the approximation of the
                                                                                  //      projected
                                                                                  //      curve v.s. theoretical exact
                                                                                  //      projection of the curve.  If
                                                                                  //      the projection goes outside of
                                                                                  //      the surface boundary by more
                                                                                  //      than this tolerance then no
                                                                                  //      curve is produced.
                                                       rdMaxDistToSurf,           // out: Max dist between 3dCurve and output
                                                                                  //      UVTrimCurve projected surface image.
                                                       dMaxDropApproxDev,         // out: Max dist 3dCurve SamplePoint was
                                                                                  //      dropped during UVTrimCurve creation.
                                                                                  //      In most cases it will be close to the
                                                                                  //      dApproxTol. In some cases it will be
                                                                                  //      much better than the dApproxTol.
                                                       sUVCurves,                 // out: Projected UVTrimCurve
                                                       FALSE,                     // in : TRUE = keep all drop curves
                                                                                  //      FALSE=Only keep curves when rdMaxDropToSurf <=
                                                                                  //      dApproxTol FALSE preserves old behavior [B260]
                                                                                  //      default:[TRUE]
                                                       TRUE,                      // in : TRUE=cr3dCurve is known to lie on the Surface
                                                                                  //      default:[FALSE]
                                                       pDropCurveFail);           // out: Optional data container for DropCurve diagnostics
                                                                                  //      and recovery-path reporting,
                                                                                  //      NUll to ignore, default:[NULL]
                  pDropCurveFail->m_pBrep = GetBrep();
              } // end (ii==1 || ii=3) - Try DropAndTrimCurve branch

#ifdef SM_DEBUG_CODE
              // draw
              if (bDebugMe) // draw and check quality of drop curves
              {
                  ULONG di;
                  SmEdge* pEdge3 = GetEdge();
                  SmFace* pFace = GetFaceuse()->GetFace();
                  SmBrep* pBrep = pEdge3->GetBrep() ? pEdge3->GetBrep() : pFace->GetBrep();
                  SmCurve* pEdgeCurve = pEdge3 ? pEdge3->GetCurve() : NULL;
                  SmSurface* pFaceSurface = pFace ? pFace->GetSurface() : NULL;
#    ifdef SM_USE_NEWTOL
                  SM_NEWTOL_LINE SmZoneTol3d sZoneTol3dDebug = pEdge3 ? pEdge3->GetTolerance() : 0.0;
#    else // SM_USE_OLDTOL
                  SM_OLDTOL_LINE SmZoneTol3d sZoneTol3dDebug = GetTolerance();
#    endif // SM_USE_OLDTOL
                  double dMaxEdgeFaceTrimCurveGap = 0.0;
                  double dMaxEdgeFaceGap = 0.0;

                  pDropCurveFail->Dump();

                  // get max gaps
                  if (pEdgeCurve && pFaceSurface)
                  {
                      for (di = 0; di < sUVCurves.GetSize(); di++)
                      {
                          SmCrvOnSurf sCrvOnSurf(*sUVCurves[di], *pFaceSurface, NULL, 0, pContext);

                          SmExtent1d sIvl = sUVCurves[di]->GetNaturalInterval();
                          ULONG lSampleCnt = 20;
                          SmCrvCrvGapFunction sEdgeUVTrimCurveGF(
                              (SmXSectTol3d)sZoneTol3dDebug, pEdgeCurve, sIvl, &sCrvOnSurf, lSampleCnt);
                          SmCrvSrfGapFunction sEdgeFaceGF(
                              (SmXSectTol3d)sZoneTol3dDebug, pEdgeCurve, sIvl, pFaceSurface, lSampleCnt);

                          // get max gap functions
                          SmGapSample* pMaxEdgeFaceTrimCurveGap;
                          sEdgeUVTrimCurveGF.GetMaxGapSample(pMaxEdgeFaceTrimCurveGap);
                          SmGapSample* pMaxEdgeFaceGap;
                          sEdgeFaceGF.GetMaxGapSample(pMaxEdgeFaceGap);
                          if (!pMaxEdgeFaceGap || !pMaxEdgeFaceTrimCurveGap)
                          {
                              continue;
                          }

                          double dThisEdgeFaceTrimCurveGap = pMaxEdgeFaceTrimCurveGap->GetLength();
                          double dThisEdgeFaceGap = pMaxEdgeFaceGap->GetLength();

                          if (dThisEdgeFaceTrimCurveGap > dMaxEdgeFaceTrimCurveGap)
                          {
                              dMaxEdgeFaceTrimCurveGap = dThisEdgeFaceTrimCurveGap;
                          }
                          if (dThisEdgeFaceGap > dMaxEdgeFaceGap)
                          {
                              dMaxEdgeFaceGap = dThisEdgeFaceGap;
                          }
                      }
                  }

                  smgfx_Erase();
                  smgfx_SetLook(1, 4, 0, 0, 1);
                  if (pBrep)
                      pBrep->Draw(TRUE);
                  sm_GraphicsLoop();
                  smgfx_SetLook(1, 4, 0, 1, 0);
                  pFace->GetSurface()->DrawUV(8, 8, FALSE, NULL, TRUE);
                  sm_GraphicsLoop();
                  smgfx_SetLook(3, 4, .1, .1, .1);
                  pFace->GetSurface()->DrawSeams();
                  sm_GraphicsLoop();
                  smgfx_SetLook(2, 5, 0, 1, 1);
                  pFace->GetSurface()->DrawParams();
                  sm_GraphicsLoop();
                  smgfx_SetLook(3, 5, 0, 1, 1);
                  pFace->Draw();
                  sm_GraphicsLoop();
                  for (di = 0; di < sUVCurves.GetSize(); di++)
                  {
                      SmCrvOnSurf sCrvOnSurf(*sUVCurves[di], *pFaceSurface);
                      smgfx_SetLook(4, 6, 0, 1, 1);
                      sCrvOnSurf.Draw(NULL, TRUE);
                      sm_GraphicsLoop();
                      smgfx_SetLook(4, 6, 0, .5, 1);
                      sCrvOnSurf.DrawSpeed(-10.0);
                      sm_GraphicsLoop();
                      smgfx_SetLook(4, 6, 0, 1, .5);
                      sCrvOnSurf.DrawParams();
                      sm_GraphicsLoop();
                  }
                  smgfx_SetLook(5, 7, 1, 0, 0);
                  pEdgeCurve->DrawParams();
                  sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }

              if (bDebugMe7)
              {
                  smos_sprintf(sBuff, _T("SmEdgeuse::CreateUVTrimCurve(): Success? %ld; created %lu curves:\n"),
                               lResult, sUVCurves.GetSize());
                  smos_WriteBuffer(sBuff);
                  ULONG kk;
                  for (kk = 0; kk < sUVCurves.GetSize(); kk++)
                  {
                      sUVCurves[kk]->Dump();
                  }
              }
#endif // SM_DEBUG_CODE

              // when dropCurve fails and Surface has singularities - it's likely DropCurve started at a surface
              // singularity
              if ((lResult != SM_SUCCESS || sUVCurves.GetSize() == 0) &&
                  (pSurface && pSurface->GetSingularities() != SM_SS_NONE))
              {
                  // create a temp curve copy with reverse parameterizaton
                  SmCurve* pRevCurve = NULL;
                  pCurve->Copy(*pContext, pRevCurve);
                  SmObjDelete sClean1(pRevCurve);

                  // Have to trim this, otherwise the reversals mess things up. [081031]
                  pRevCurve->Trim(sEdgeDomain); // may snap sIvl by tol to existing knots
                  SmExtent1d sNewIvl;
                  SER(pRevCurve->ReverseParameterization(sEdgeDomain, sNewIvl));

                  // drop reverse curve to surface
                  lResult = pSurface->DropCurve(*pContext, sFaceDomain, *pRevCurve, sNewIvl, (SmApproxTol3d)sZoneTol3d,
                                                rdMaxDistToSurf, // out: MaxDist(DropPt, 3dCurvePt), found by sampling,
                                                                 // may be slightly less than actual max.
                                                dMaxDropApproxDev, // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt),
                                                                   // Quality Measure: expected to be close to 0.0
                                                sUVCurves);

                  // when reverse curve dropping works
                  if (lResult == SM_SUCCESS && sUVCurves.GetSize() > 0)
                  {
                      // reverse array of sUVCurves
                      for (ULONG kkk = 0; kkk < sUVCurves.GetSize(); kkk++)
                      {
                          SER(sUVCurves[kkk]->ReverseParameterization(sNewIvl, sNewIvl));
                      }
                      pDropCurveFail->m_eCreatePathType = SM_CTC_REVERSEDROP_RECOVERY;
                      pDropCurveFail->m_pBrep = GetBrep();
                  } // end reverse curve dropping worked branch

                  // when dropReverseCurve fails - it might be that both end points sit on singularities
                  if (lResult != SM_SUCCESS)
                  {
                      // see if both curve ends drop to singularities
                      SmSurfParamType eSingularDir;
                      SmBoolean bStartSingularity = FALSE, bEndSingularity = FALSE;
                      SmBoolean bStartResult = FALSE, bEndResult = FALSE;
                      SmPoint2d sStartUV, sEndUV;
                      double dStartGap = 0.0, dEndGap = 0.0;
                      SmPoint3d sStartPoint, sEndPoint;
                      SmBoolean bIsMulti;
                      SmExtent2d sSurfaceUVDomain = pSurface->GetNaturalUVDomain();

                      // Curve EndPoints
                      pCurve->EvaluatePoint(sEdgeDomain.GetMin(), sStartPoint);
                      pCurve->EvaluatePoint(sEdgeDomain.GetMax(), sEndPoint);

                      // drop curve endpoints onto surface
                      pSurface->DropPoint(
                          sStartPoint, sSurfaceUVDomain, NULL, bStartResult, sStartUV, dStartGap, bIsMulti);
                      if (bStartResult)
                      {
                          bStartSingularity = pSurface->IsSingularity(sStartUV, eSingularDir);

                          if (bStartSingularity)
                          {
                              pSurface->DropPoint(
                                  sEndPoint, sSurfaceUVDomain, NULL, bEndResult, sEndUV, dEndGap, bIsMulti);
                              if (bEndResult)
                              {
                                  bEndSingularity = pSurface->IsSingularity(sEndUV, eSingularDir);
                              }
                          }
                      } // end OK to classify both endPoints check

                      // if both ends map to singulartities
                      if (bStartSingularity && bEndSingularity)
                      {
                          // get the edge half domains
                          SmExtent1d s1stHalfDomain(sEdgeDomain.GetMin(), sEdgeDomain.Evaluate(.5));
                          SmExtent1d s2ndHalfDomain(sEdgeDomain.Evaluate(.5), sEdgeDomain.GetMax());

                          // create a pair of temp curve copies for each half of the target curve
                          SmCurve *p1stHalfCurve, *p2ndHalfCurve;
                          pCurve->Copy(*pContext, p1stHalfCurve);
                          pCurve->Copy(*pContext, p2ndHalfCurve);

                          SmObjDelete sClean2(p1stHalfCurve);
                          SmObjDelete sClean3(p2ndHalfCurve);

                          // trim curves to their half of the original curve
                          p1stHalfCurve->Trim(s1stHalfDomain); // may snap sIvl by tol to existing knots
                          p2ndHalfCurve->Trim(s2ndHalfDomain); // may snap sIvl by tol to existing knots

                          // reverse the parameterization of the 1st half
                          SmExtent1d sNewInterval;
                          SER(p1stHalfCurve->ReverseParameterization(s1stHalfDomain, sNewInterval));

                          // drop the 1st half curve to surface
                          SmTArray<SmBSplineCurve*> s1stUVCurves;
                          SmObjsDelete<SmBSplineCurve*> sClean4(&s1stUVCurves);
                          SmStatus l1stResult = pSurface->DropCurve(*pContext, sFaceDomain, *p1stHalfCurve,
                                                                    sNewInterval, (SmApproxTol3d)sZoneTol3d,
                                                                    rdMaxDistToSurf, // out: MaxDist(DropPt, 3dCurvePt),
                                                                                     // found by sampling, may be
                                                                                     // slightly less than actual max.
                                                                    dMaxDropApproxDev, // out:
                                                                                       // MaxDist(DropPt->SrfNormalLine,
                                                                                       // 3dCurvePt), Quality Measure:
                                                                                       // expected to be close to 0.0
                                                                    s1stUVCurves);
                          // when 1st half dropped successfully  - carry on
                          if (l1stResult == SM_SUCCESS && s1stUVCurves.GetSize() == 1)
                          {
                              // drop the 2nd half curve to the surface
                              SmTArray<SmBSplineCurve*> s2ndUVCurves;
                              SmObjsDelete<SmBSplineCurve*> sClean5(&s2ndUVCurves);
                              SmStatus l2ndResult = pSurface->DropCurve(*pContext, sFaceDomain, *p2ndHalfCurve,
                                                                        s2ndHalfDomain, (SmApproxTol3d)sZoneTol3d,
                                                                        rdMaxDistToSurf, // out: MaxDist(DropPt,
                                                                                         // 3dCurvePt), found by
                                                                                         // sampling, may be slightly
                                                                                         // less than actual max.
                                                                        dMaxDropApproxDev, // out:
                                                                                           // MaxDist(DropPt->SrfNormalLine,
                                                                                           // 3dCurvePt), Quality
                                                                                           // Measure: expected to be
                                                                                           // close to 0.0
                                                                        s2ndUVCurves);
                              // when 2nd half dropped successfully - carry on
                              if (l2ndResult == SM_SUCCESS && s2ndUVCurves.GetSize() == 1)
                              {
                                  // restore parameterization of 1stHalf drop curve
                                  SER(s1stUVCurves[0]->ReverseParameterization(sNewInterval, sNewInterval));

                                  // stitch the two halves together
                                  SmBSplineCurve* pNewUVTrimCurve = NULL;
                                  s1stUVCurves.Add(s2ndUVCurves[0]);
                                  s2ndUVCurves.RemoveAt(0);
                                  SmStatus sRtn =
                                      SmBSplineCurve::CreateByJoining(*pContext, s1stUVCurves, NULL, pNewUVTrimCurve);

                                  // when stitching worked
                                  if (sRtn == SM_SUCCESS && pNewUVTrimCurve != NULL)
                                  {
                                      // check range of pNewUVTrimCurve
                                      SM_ASSERT(pNewUVTrimCurve->GetNaturalInterval().AreEqual(sEdgeDomain));

                                      // set build values
                                      s2ndUVCurves.Add(pNewUVTrimCurve);
                                      lResult = SM_SUCCESS;

                                      pDropCurveFail->m_eCreatePathType = SM_CTC_TWOSINGULARENDPTS_RECOVERY;
                                      pDropCurveFail->m_pBrep = GetBrep();
                                  }
                                  else // clean up
                                  {
                                      if (pNewUVTrimCurve)
                                      {
                                          delete pNewUVTrimCurve;
                                          pNewUVTrimCurve = NULL;
                                      }
                                  }
                              } // end 2ndHalf drop successful check
                          } // end 1stHalf drop successful check
                      } // end both curve ends drop to singularity check
                  } // end DropCurve hasn't worked yet check
              } // end curve drop failed check
    }
      // If no UVTrimCurves are made, try next iteration.
      if ( sUVCurves.GetSize() == 0 )
        { continue ; }

      SmObjsDelete<SmBSplineCurve*> sCleanArr(&sUVCurves);

      // Check for the DropCurve methods returning two curves.
      // The only legitimate ways for this to happen are (1) the curve runs along
      // a seam, or (2) it runs across a seam, but only by less than tolerance.
      // The second case can happen (hopefully only) during read-in: other systems
      // create Faces that span seams on closed surfaces; we split such Faces
      // at the seams when reading in.

      SmBoolean bOnSeam = sm_AreSeamCurves( this, sUVCurves, sEdgeDomain, bDropCurveCalled );

      // First check for the second case:
      // If either curve is shorter than tolerance, remove it.
      if ( sUVCurves.GetSize() == 2  &&  ! bOnSeam )
        {
          double dEdgeTol = pEdge->GetTolerance();
          SmVertex *pVtx  = pEdge->GetStartVertex();
          double dVtx1Tol = pVtx->GetTolerance();
          pVtx = pEdge->GetOtherVertex( pVtx );
          double dVtx2Tol = dVtx1Tol;
          if (pVtx != NULL)
            dVtx2Tol = pVtx->GetTolerance();

          // Remove only if one is much smaller than the other.  [B513]
          // Have a case where they're seam curves, but Face's tol is bigger than the surface.

          SmBSplineCurve *pUVCurve1 = sUVCurves[0];
          SmCrvOnSurf sCOS1( *pUVCurve1, *pSurface, NULL, 0, pContext );
          double dLen1 = sCOS1.ApproximateLength( pUVCurve1->GetNaturalInterval(), 5 );

          SmBSplineCurve *pUVCurve2 = sUVCurves[1];
          SmCrvOnSurf sCOS2( *pUVCurve2, *pSurface, NULL, 0, pContext );
          double dLen2 = sCOS2.ApproximateLength( pUVCurve2->GetNaturalInterval(), 5 );

          if ( dLen1 < dLen2 / 10.0 && dLen1 < dEdgeTol + dVtx1Tol )
            {
              sUVCurves.RemoveAt(0,1);
              SM_ASSERT(pUVCurve1 != NULL) ; delete pUVCurve1 ; pUVCurve1 = NULL ;
              pDropCurveFail->m_eCreatePathType = SM_CTC_SHORTPIECE_SEAMCROSS_RECOVERY ; 
              pDropCurveFail->m_pBrep = GetBrep() ;
            }
          else if ( dLen2 < dLen1 / 10.0 && dLen2 < dEdgeTol + dVtx2Tol )
            {
              sUVCurves.RemoveAt(1,1);
              SM_ASSERT(pUVCurve2 != NULL) ; delete pUVCurve2 ; pUVCurve2 = NULL ;
              pDropCurveFail->m_eCreatePathType = SM_CTC_SHORTPIECE_SEAMCROSS_RECOVERY ;
              pDropCurveFail->m_pBrep = GetBrep() ;
            }
        }  // end first 2-curve check: short seam crossing

      // If still two curves, do the seam check: have to decide which to keep.
      if ( sUVCurves.GetSize() == 2  &&  bOnSeam ) 
        {
          // Take care of seam curves by choosing the correct projected curve.
          // This should work for both continuous and discontinuous seam curves.
          SmCurve *pUVCurve1 = (SmCurve*)sUVCurves[0];
          SmCurve *pUVCurve2 = (SmCurve*)sUVCurves[1];

          // locals
          SmPoint3d sMidPnt;                       // near mid Edge Pt
          SmPoint3d sUVPnt1, sPnt1, sDU1, sDV1 ;   // near mid UVCurve1, then Surface Eval pos, 1stDU, 1stDV
          SmPoint3d sUVPnt2, sPnt2, sDU2, sDV2 ;   // near mid UVCurve2, then Surface Eval pos, 1stDU, 1stDV
          
          //double    dDist1 ;                       // dist(sMidPnt, sPnt1)  - EdgeMidPtDropToSurface to Surface(UVCurve1MidPt) dist
          //double    dDist2 ;                       // dist(sMidPnt, sPnt2)  - EdgeMidPtDropToSurface to Surface(UVCurve2MidPt) dist
          // note: for UVCurves running along either side of a seam, dDist1 and dDist2 will be smaller than tolerance
          //       for UVCurves running across seams, dDist1 and dDist2 will be larger for longer UVCurve segments
            {
              // Get this edgeuse's Position (3d) near edge's midPoint.
              double dMidParam = ( this->GetOrientation() == SM_OT_SAME ) ? 0.48765 : 1.0 - 0.48765;
              (SM_CONST_CAST( SmEdgeuse*, this ) )->NormalizedEvaluate( dMidParam, FALSE, sMidPnt );

              // evaluate NearMid 1st UVCurve Point clamped to face domain (for seams: coincident to sMidPoint, for crossing curves: not)
              // Use the uv curve's domain, not the Edge's.  On a seam, it's the same
              // as the Edge's, but if crossing, we need the curve's own midpoint;
              // Edge midpoint might not even be within its own domain.  [091030]
              SmExtent1d sUVCurveDomain = pUVCurve1->GetNaturalInterval();
              SER( pUVCurve1->EvaluatePoint( sUVCurveDomain.Evaluate(0.48765), sUVPnt1 ));
              SmPoint2d sUV1( sUVPnt1.x, sUVPnt1.y );
              sUV1 = sFaceDomain.ClampPoint2d( sUV1 );

              // evaluate NearMid 2nd UVCurve Point clamped to face domain (for seams: coincident to sMidPoint, for crossing curves: not)
              // Again, use uv curve's domain.
              sUVCurveDomain = pUVCurve2->GetNaturalInterval();
              SER( pUVCurve2->EvaluatePoint( sUVCurveDomain.Evaluate(0.48765), sUVPnt2 ));
              SmPoint2d sUV2( sUVPnt2.x, sUVPnt2.y );
              sUV2 = sFaceDomain.ClampPoint2d( sUV2 );

              // project NearMid UVCurve Points to 3Space through Edgeuse->Face->Surface
              SER(pSurface->Evaluate1stDerivatives( sUV1, TRUE, TRUE, sPnt1, sDU1, sDV1 ));
              SER(pSurface->Evaluate1stDerivatives( sUV2, TRUE, TRUE, sPnt2, sDU2, sDV2 ));

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  pUVCurve1->Dump();
                  pUVCurve2->Dump();

                  if ( FALSE )
                    { smgfx_Erase(); }
                  sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 1,0,0 ); sMidPnt.Draw();        sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 1,0,1 ); sPnt1.Draw();          sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 0,0,1 ); sDU1.Draw(&sPnt1);     sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 0,1,0 ); sDV1.Draw(&sPnt1);     sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 0,1,1 ); sPnt2.Draw();          sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 1,0,1 ); sDU2.Draw(&sPnt2);     sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 0,0,1 ); sDV2.Draw(&sPnt2);     sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 1,1,0 ); pSurface->DrawUV(5,5); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

            } // end evaluating UVCurve locals when 2 UVCurves are present


          // Note: the following test is quite ad-hoc.  It has been replaced
          // by the tolerance-based test above.

          // // get distances between facePoints and edge's midPoint
          // //  (zero for seam curves, NonZero for crossing curves)
          // dDist1 = sPnt1.DistanceBetween( sMidPnt );
          // dDist2 = sPnt2.DistanceBetween( sMidPnt );
          //
          // // When curve crosses seam by small amount, just delete the
          // //   small amount of UVCurve that crossed the seam.
          // // When dDist1 and dDist2 are both tiny and similar,
          // //   expect two UVCurves running down either side of a seam.
          // // When one is large and one is small - expect a curve crossing
          // //   a seam by a small amount.
          // // We are assuming that if an edge crosses a seam, it would be only a
          // // little bit (due to noise/tolerance issues), so one of dDist1 or
          // // dDist2 will be small, and the other large.
          // if(dDist1 < dDist2/100.0  &&  dDist2 > pEdge->GetTolerance()/1000.0)
          //   {
          //     // found a crossing curve: remove the short UVTrimCurve piece in index [1]
          //     sUVCurves.RemoveAt(1,1);
          //     SM_ASSERT(pUVCurve2 != NULL) ; delete pUVCurve2 ; pUVCurve2 = NULL ;
          //   }
          // else if(dDist2 < dDist1/100.0  &&  dDist1 > pEdge->GetTolerance()/1000.0)
          //   {
          //     // found a crossing curve: remove the short UVTrimCurve piece in index [0]
          //     sUVCurves.RemoveAt(0,1);
          //     SM_ASSERT(pUVCurve1 != NULL) ; delete pUVCurve1 ; pUVCurve1 = NULL ;
          //   }

          // When there are still 2 UVTrimCurves expect to have 2 UVCurves running down either side of a seam curve.
          // Find the right one for this Edgeuse.
          if ( sUVCurves.GetSize() == 2 )
            {
              SmBoolean bWorkIn3d = TRUE;
              // TRUE=proj Edge pts to Surface, FALSE=use pUVTrimCurve to calc Surface points
              // We can't use UVTrimCurves here, because that's what we're creating.
              // Also, if called with FALSE, EvaluateBinormal() could possibly
              // call this method, and possibly lead to infinite recursion.

              SmPoint3d sBinormal, sTangent ; // binormal, tangent dropped to Surface
              SER( EvaluateBinormal( sEdgeDomain.Evaluate(0.48765), bWorkIn3d,
                                     sMidPnt, sBinormal, &sTangent ));

              // local angle data - to determine which of two seam curves to keep
              double dAngBinU1 = 0.0, dAngBinV1 = 0.0, dMaxAngBinU = 0.0, dMinAngBinU = 0.0;
              double dAngBinU2 = 0.0, dAngBinV2 = 0.0, dMaxAngBinV = 0.0, dMinAngBinV = 0.0;
              double dAngTanU1 = 0.0, dAngTanU2 = 0.0, dAngTanV1 = 0.0, dAngTanV2 = 0.0;

                {
                  // Get angles between Binormal and the facePoints' U and V derivs.
                  SER( sDU1.AngleBetween( sBinormal, dAngBinU1 ));
                  SER( sDU2.AngleBetween( sBinormal, dAngBinU2 ));
                  if (dAngBinU1 > dAngBinU2) { dMaxAngBinU = dAngBinU1 ; dMinAngBinU = dAngBinU2; }
                  else                       { dMaxAngBinU = dAngBinU2 ; dMinAngBinU = dAngBinU1; }

                  SER( sDV1.AngleBetween( sBinormal, dAngBinV1 ));
                  SER( sDV2.AngleBetween( sBinormal, dAngBinV2 ));
                  if (dAngBinV1 > dAngBinV2) { dMaxAngBinV = dAngBinV1 ; dMinAngBinV = dAngBinV2 ; }
                  else                       { dMaxAngBinV = dAngBinV2 ; dMinAngBinV = dAngBinV1 ; }

                  // get angles between tangent and the facePoints' U and V tangents
                  SER( sDU1.AngleBetween( sTangent, dAngTanU1 ));
                  SER( sDV1.AngleBetween( sTangent, dAngTanV1 ));
                  SER( sDU2.AngleBetween( sTangent, dAngTanU2 ));
                  SER( sDV2.AngleBetween( sTangent, dAngTanV2 ));

                  // The angles with the edge-curve tangent are used only for
                  // checking which derivative lines up with the edge.
                  // For that, we don't care parallal or anti-parallel,
                  // so mirror them about 90 degrees so that 180 degrees maps to 0.
                  if ( dAngTanU1 > SM_PI/2.0 ) { dAngTanU1 = SM_PI - dAngTanU1; }
                  if ( dAngTanV1 > SM_PI/2.0 ) { dAngTanV1 = SM_PI - dAngTanV1; }
                  if ( dAngTanU2 > SM_PI/2.0 ) { dAngTanU2 = SM_PI - dAngTanU2; }
                  if ( dAngTanV2 > SM_PI/2.0 ) { dAngTanV2 = SM_PI - dAngTanV2; }

                } // end local angle data

              // The cross-boundary derivative points into the face at
              // the low end of the domain, and out of the face at the
              // high end.  The binormal always points into the face.
              // So check whether the CBD and the binormal are in the same
              // or opposite directions.
              // The CBD is the one that does not line up with the curve tangent.
              // We check that here, within two degrees.

              double dTwoDeg = 2.0 * SM_PI / 180.0;

              if ( dAngTanV1 < dTwoDeg || dAngTanV2 < dTwoDeg )
                {
                  // V lines up, so U is the cross-boundary deriv (if curve is a seam).

                  if(   (SM_PI - dMaxAngBinU < dMinAngBinU && sUVPnt1.x > sUVPnt2.x)  // keep max curve and max curve is 1st
                     || (SM_PI - dMaxAngBinU > dMinAngBinU && sUVPnt1.x < sUVPnt2.x)) // keep min curve and min curve is 1st
                    { // remove 2nd UVTrimCurve
                      sUVCurves.RemoveAt(1,1);
                      SM_ASSERT(pUVCurve2 != NULL) ; delete pUVCurve2 ; pUVCurve2 = NULL ;
                      pDropCurveFail->m_eCreatePathType = SM_CTC_PICKONESEAM_RECOVERY ;
                      pDropCurveFail->m_pBrep = GetBrep() ;
                    }
                  else
                    { // remove 1st UVTrimCurve
                      sUVCurves.RemoveAt(0,1);
                      SM_ASSERT(pUVCurve1 != NULL) ; delete pUVCurve1 ; pUVCurve1 = NULL ;
                      pDropCurveFail->m_eCreatePathType = SM_CTC_PICKONESEAM_RECOVERY ;
                      pDropCurveFail->m_pBrep = GetBrep() ;
                    }

                  // failed to choose one seam curve - try again with a different tolerance
                  if (sUVCurves.GetSize() > 1)
                    {
                      sCleanArr.Clear();
                      for ( jj=0; jj<sUVCurves.GetSize(); jj++ )
                        { delete( sUVCurves[jj] ); sUVCurves[jj] = NULL; }

                      sUVCurves.ReSet();
                      continue;
                    }
                } // end tangent vector within 2 degrees of either of Surface V tangent check

              else if ( dAngTanU1 < dTwoDeg || dAngTanU2 < dTwoDeg )
                {
                  // U lines up, so V is the cross-boundary deriv (if curve is a seam).

                  if(   (SM_PI - dMaxAngBinV < dMinAngBinV && sUVPnt1.y > sUVPnt2.y)  // keep max curve and max curve is 1st
                     || (SM_PI - dMaxAngBinV > dMinAngBinV && sUVPnt1.y < sUVPnt2.y)) // keep min curve and min curve is 1st
                    { // remove 2nd UVTrimCurve
                      sUVCurves.RemoveAt(1,1);
                      SM_ASSERT(pUVCurve2 != NULL) ; delete pUVCurve2 ; pUVCurve2 = NULL ;
                      pDropCurveFail->m_eCreatePathType = SM_CTC_PICKONESEAM_RECOVERY ;
                      pDropCurveFail->m_pBrep = GetBrep() ;
                    }
                  else
                    { // remove 1st UVTrimCurve
                      sUVCurves.RemoveAt(0,1);
                      SM_ASSERT(pUVCurve1 != NULL) ; delete pUVCurve1 ; pUVCurve1 = NULL ;
                      pDropCurveFail->m_eCreatePathType = SM_CTC_PICKONESEAM_RECOVERY ;
                      pDropCurveFail->m_pBrep = GetBrep() ;
                    }

                  // failed to choose one seam curve - try again with a different tolerance
                  if (sUVCurves.GetSize() > 1)
                    {
                      sCleanArr.Clear();
                      for ( jj=0; jj<sUVCurves.GetSize(); jj++ )
                        { delete( sUVCurves[jj] ); sUVCurves[jj] = NULL; }

                      sUVCurves.ReSet();
                      continue;
                    }
                } // end tangent vector is within 2 degrees of either Surface U Tangent vector check
            } // end 2nd 2 UVCurves check -- really was a seam
        } // end 2 UVCurves check -- looking for seam curves

      // no curves with DropCurve() (possibly curve crossing seam) - try again with DropAndTrimCurve() which splits curves across seams
      if (sUVCurves.GetSize() == 0)
        {
          continue;
        }

      // If we have more than one then we may have split them because they
      // cross the boundary.  Try to find one that is within 10% of the size
      // of the original one and use it.
      //   GWC: this is not a good idea - this creates gaps between the Edge and the UVTrimCurve.
      //                                  But it's not the worst idea either, it does not create
      //                                  gaps between the Edge and its vertices - leave this healing in this method for now.
      if ( sUVCurves.GetSize() > 1 )
        {
          double dParamSize = sEdgeDomain.GetLength();

          // for every found UVTrimCurve
          ULONG jjj, kkk;
          for ( kkk=0; kkk<sUVCurves.GetSize(); kkk++ )
            {
              SmBSplineCurve *pUVBSC = sUVCurves[kkk];
              double dUVParamSize = pUVBSC->GetNaturalInterval().GetLength();

              // if this UVTrimCurve param range is at least 90% of orig param range
              if ( smos_Fabs( dUVParamSize - dParamSize ) < 0.1 * dParamSize )
                {
                  // remove all the other curves
                  for ( jjj=0; jjj<sUVCurves.GetSize(); jjj++ )
                    {
                      if ( jjj != kkk )
                        { SM_ASSERT(sUVCurves[jjj] != NULL) ; delete sUVCurves[jjj]; sUVCurves[jjj] = NULL ; }
                      pDropCurveFail->m_eCreatePathType = SM_CTC_10PERCENT_SEAMCROSS_RECOVERY ;
                      pDropCurveFail->m_pBrep = GetBrep() ;
                    }

                  // set remaining UVcurve's param range to equal original curve
                  SmExtent1d sIvlUV = sEdgeDomain;
                  SER( pUVBSC->EditParameterization( sIvlUV ));

                  // set sUVCurves = this UVTrimCurve
                  sUVCurves.ReSet();
                  sUVCurves.Add( pUVBSC );
                  break;
                }
            } // end iter every UVTrimCurve

          // if we don't have 1 UVTrimCurve - try again with a different tolerance
          if ( sUVCurves.GetSize() != 1 )
            {
              sCleanArr.Clear();
              for ( jj=0; jj<sUVCurves.GetSize(); jj++ )
                { delete( sUVCurves[jj] ); sUVCurves[jj] = NULL; }

              sUVCurves.ReSet();

              continue;
            }
        } // end more than 1 UVTrimCurve due to splitting (not seams) check

      // State: sUVCurves.GetSize() == 1

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SmBSplineCurve *pUVCurve = sUVCurves[0] ; 
          SM_ASSERT_VALID( pUVCurve );
          pUVCurve->Dump();

          SmEdge *pEdgeDebug = GetEdge() ;
          SmFace *pFace = GetFaceuse()->GetFace() ;
          SmBrep *pBrep = pEdgeDebug->GetBrep() ? pEdgeDebug->GetBrep() : pFace->GetBrep() ;
          SmCrvOnSurf sCrvOnSurf(*pUVCurve, *pFace->GetSurface()) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; pFace->GetSurface()->DrawUV(3,3,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; pEdgeDebug->GetCurve()->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sCrvOnSurf.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // when single UVTrimCurve has same param range as edge - return success
      SmExtent1d sUVIvl = sUVCurves[0]->GetNaturalInterval();

      if (   SM_ARE_SAME( sUVIvl.GetMin(), sEdgeDomain.GetMin() )
          && SM_ARE_SAME( sUVIvl.GetMax(), sEdgeDomain.GetMax() ) )
        {
          sCleanArr.Clear();
          rpNewUVCurve = sUVCurves[0];
          rpNewUVCurve->SetOwner((SmEdgeuse *)this) ;
          if(pDropCurveFail->m_eCreatePathType == SM_CTC_UNKNOWN)
            { pDropCurveFail->m_eCreatePathType = SM_CTC_OKAY ;
              pDropCurveFail->m_pBrep = GetBrep() ;
            }

          // clean up Surface Cache side-effect of attaching 2nd UVTrimCurve
          if( pUVTrimCurve == NULL && GetUVTrimCurve() != NULL )
            {
              ((SmEdgeuse *)this)->SetUVTrimCurve(NULL,0.0,TRUE,NULL) ; // TRUE = delete existing UVTrimCurve
            }

          // successfully constructed and selected a UVTrimCurve for this Edgeuse - set output
          if ( pdMaxDeviation != NULL )
            { *pdMaxDeviation = dMaxDropApproxDev; }
          else
            { // Put the entire error into the one output.
              rdMaxDistToSurf = smos_Sqrt(   rdMaxDistToSurf  *  rdMaxDistToSurf
                                          + dMaxDropApproxDev * dMaxDropApproxDev );
            }

#ifdef SM_DEBUG_CODE
          if(bDebugMe2)
            {
              SM_ASSERT_VALID(rpNewUVCurve) ;
              rpNewUVCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed) ;
              rpNewUVCurve->PassesValidityCheck(SM_VC_ALL, eCheckFailed) ;
            }

          if(rpNewUVCurve->GetDim() != 2)
            {
              SM_ASSERT_MSG(rpNewUVCurve->GetDim() == 2, _T("SmEdgeuse::CreateUVTrimCurve created a non-2D UVTrimCurve")) ;
            }
#endif // SM_DEBUG_CODE

          // If we had to use a bigger tolerance, update the Edge's tolerance
          // according to the actual gaps.
          if ( ii > 1 )
            {
              double dError = ( pdMaxDeviation == NULL )
                                  ? rdMaxDistToSurf
                                  : smos_Sqrt (     rdMaxDistToSurf * rdMaxDistToSurf
                                                + dMaxDropApproxDev * dMaxDropApproxDev );
              SmEdge *pE = this->GetEdge();
              if ( pE ) { pE->SetTolerance( dError * 1.01 ); }
            }

          // all done
          return SM_SUCCESS;

        } // end single UVTrimCurve covers whole Edge param range branch

      else // single UVTrimCurve with shorter param range than edge
        {
          // Don't use 99% threshold (below). We'll use XSect tolerances instead. 
          // This makes sure that after trimming the Edge to match the TrimCurve,
          // the Edge will still intersect the Vertex.
          // Is this even still necessary?
          //// In this case: If the uv trim curve covers almost all (99%) of the
          //// Edge param range, then it's probably just a matter of noisy data
          //// and tolerances.  (We probably removed a shorter-than-tolerance
          //// second uv curve above.)  In that case, we will use the projected UV trim
          //// curve as is, and adjust the param range of the Edge to match it.
          ////
          //// Note, this trimming is a bit of a hack -- creating a uv trim curve
          //// for an Edgeuse shouldn't trim its Edge.  But it causes problems
          //// not to do it.  We will assume that it's within tolerance, as
          //// checked above.  [B484]
          //
          //// Otherwise: try again.
          ////static double dPerCent = 0.01; // 0.01 for the 99%;  try at 0.1 (10%).
          ////double dScale = dPerCent * sEdgeDomain.GetLength();

          // Don't edit the Edge unless we are keeping the UVCurve. 
          SmBoolean bEditUVMin = FALSE;
          SmBoolean bEditUVMax = FALSE;

          // Get the 2D end points of the Drop Curve
          SmPoint3d sUVStart, sUVEnd, sDropCurveStart3d, sDropCurveEnd3d;
          SmPoint2d sUVStart2d, sUVEnd2d;
          sUVCurves[0]->GetEnds( sUVStart, sUVEnd );
          sUVStart2d = sUVStart; 
          sUVEnd2d   = sUVEnd;

          // Get the 3D end points of the Drop Curve
          pSurface->EvaluatePoint( sUVStart2d, sDropCurveStart3d );
          pSurface->EvaluatePoint( sUVEnd2d, sDropCurveEnd3d );

          // Get the local XSect Tolerances
          SmVertex *pStartVertex, *pEndVertex;
          pEdge->GetVertices( pStartVertex, pEndVertex );
          SmXSectTol3d sStartXSectTol = SmTol::GetXSectTol3d( pEdge, pStartVertex );
          SmXSectTol3d sEndXSectTol   = SmTol::GetXSectTol3d( pEdge, pEndVertex );

          // when Edge/UVCurve min params differ
          if ( ! SM_ARE_SAME( sUVIvl.GetMin(), sEdgeDomain.GetMin() ))
            {
              //if ( smos_Fabs( sUVIvl.GetMin() - sEdgeDomain.GetMin() ) < dScale )
              if ( pStartVertex->GetPoint().DistanceBetween( sDropCurveStart3d ) < sStartXSectTol )
              { bEditUVMin = TRUE; }
              else // Edge StartParam differs by more than 1% of UVTrimCurve StartParam branch
                { // Curve may have stepped off the surface.
                  // Do not accept this uv curve.
                  sCleanArr.Clear();
                  SM_ASSERT(sUVCurves[0] != NULL) ; delete sUVCurves[0] ; sUVCurves[0] = NULL ;
                  continue;
                } // end Edge StartParam differs by more than 1% of UVTrimCurve StartParam branch
            } // end sUVIvl/sEdgeDomain min values are not the same check

          // when Edge/UVCurve max params differ
          if ( ! SM_ARE_SAME( sUVIvl.GetMax(), sEdgeDomain.GetMax() ))
            {
              //if ( smos_Fabs( sEdgeDomain.GetMax() - sUVIvl.GetMax()) < dScale )
              if ( pEndVertex->GetPoint().DistanceBetween( sDropCurveEnd3d ) < sEndXSectTol )
              { bEditUVMax = TRUE; }
              else // Edge EndParam differs by more than 1% of UVTrimCurve EndParam branch
                { // Curve may have stepped off the surface - try again with
                  // larger tolerance.
                  sCleanArr.Clear();
                  SM_ASSERT(sUVCurves[0] != NULL) ; delete sUVCurves[0] ; sUVCurves[0] = NULL ;
                  continue;
                } // end Edge EndParam differs by more than 1% of UVTrimCurve EndParam branch
            } // end sUVIvl/sEdgeDomain max values are not the same check

          // We are keeping the curve, edit as necessary
          if ( bEditUVMin )
          {
              // Sometimes the uv curve can actually come back longer,
              // due to noise.  In that case, trim the uv curve.
              if ( sUVIvl.GetMin() < sEdgeDomain.GetMin() )
              {
                  SmExtent1d sNewIvl( sEdgeDomain.GetMin(), sUVIvl.GetMax() );
                  sUVCurves[0]->Trim( sNewIvl );  // may snap sIvl by tol to existing knots
                  pDropCurveFail->m_eCreatePathType = SM_CTC_1PERCENT_UVTRIMCURVETRIM_RECOVERY1;
                  pDropCurveFail->m_pBrep = GetBrep();
              }
              else
              {
                  // change Edge parameter to match UVTrimCurve parameter
                  SmExtent1d sNewIvl( sUVIvl.GetMin(), sEdgeDomain.GetMax() );
                  pEdge->SetInterval( sNewIvl );
                  sEdgeDomain = pEdge->GetInterval();
                  pDropCurveFail->m_eCreatePathType = SM_CTC_1PERCENT_EDGETRIM_RECOVERY1;
                  pDropCurveFail->m_pBrep = GetBrep();

                  // Also have to update any other Edgeuses on this Edge.
                  SmTArray< SmEdgeuse* > sEUs;
                  pEdge->GetEdgeuses( sEUs );
                  ULONG lNumEUs = sEUs.GetSize();
                  SmEdgeuse *pThisEU;
                  for ( jj = 0; jj < lNumEUs; jj++ )
                  {
                      pThisEU = sEUs[jj];
                      if ( pThisEU == this ) { continue; }
                      SmCurve *pThisUVCurve = pThisEU->GetUVTrimCurvePointer();
                      if ( pThisUVCurve == NULL ) { continue; }
                      pThisUVCurve->Trim( sNewIvl ); // may snap sIvl by tol to existing knots
                  }
              } // end else, trim Edge instead of uv curve
          } // end Edge StartParam differs but by less than 1% of UVTrimCurve StartParam branch
          if ( bEditUVMax )
          {
              // Sometimes the uv curve can actually come back longer,
              // due to noise.  In that case, trim the uv curve.
              if ( sUVIvl.GetMax() > sEdgeDomain.GetMax() )
              {
                  SmExtent1d sNewIvl( sUVIvl.GetMin(), sEdgeDomain.GetMax() );
                  sUVCurves[0]->Trim( sNewIvl );   // may snap sIvl by tol to existing knots
                  pDropCurveFail->m_eCreatePathType = SM_CTC_1PERCENT_UVTRIMCURVETRIM_RECOVERY2;
                  pDropCurveFail->m_pBrep = GetBrep();
              }
              else
              {
                  // Change Edge parameter to match UVTrimCurve parameter.
                  SmExtent1d sNewIvl( sEdgeDomain.GetMin(), sUVIvl.GetMax() );
                  pEdge->SetInterval( sNewIvl );
                  sEdgeDomain = pEdge->GetInterval();
                  pDropCurveFail->m_eCreatePathType = SM_CTC_1PERCENT_EDGETRIM_RECOVERY2;
                  pDropCurveFail->m_pBrep = GetBrep();

                  // Also have to update any other Edgeuses on this Edge.
                  SmTArray< SmEdgeuse* > sEUs;
                  pEdge->GetEdgeuses( sEUs );
                  ULONG lNumEUs = sEUs.GetSize();
                  SmEdgeuse *pThisEU;
                  for ( jj = 0; jj < lNumEUs; jj++ )
                  {
                      pThisEU = sEUs[jj];
                      if ( pThisEU == this ) { continue; }
                      SmCurve *pThisUVCurve = pThisEU->GetUVTrimCurvePointer();
                      if ( pThisUVCurve == NULL ) { continue; }
                      pThisUVCurve->Trim( sNewIvl );  // may snap sIvl by tol to existing knots
                  }
              } // end else, trim Edge instead of uv curve
          } // end Edge EndParam differs but by less than 1% of UVTrimCurve EndParam branch

          // State: we have accepted the single UV Trim curve,
          // and probably adjusted the Edge's param range.
          sCleanArr.Clear();
          rpNewUVCurve = sUVCurves[0];
          rpNewUVCurve->SetOwner((SmEdgeuse *)this) ;

          // clean up Surface Cache side-effect of attaching 2nd UVTrimCurve
          if ( pUVTrimCurve == NULL && GetUVTrimCurve() != NULL )
            {
              ((SmEdgeuse *)this)->SetUVTrimCurve( NULL, 0.0, TRUE, NULL ) ; // TRUE = delete existing UVTrimCurve
            }

          // successfully constructed and set a UVTrimCurve for this Edgeuse

          if ( pdMaxDeviation != NULL )
            { *pdMaxDeviation = dMaxDropApproxDev; }
          else
            { // Put the entire error into the one output.
              rdMaxDistToSurf = smos_Sqrt(  rdMaxDistToSurf   * rdMaxDistToSurf
                                          + dMaxDropApproxDev * dMaxDropApproxDev );
            }

#ifdef SM_DEBUG_CODE
          if(bDebugMe2)
            {
              SM_ASSERT_VALID(rpNewUVCurve) ;
              rpNewUVCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed) ;
              rpNewUVCurve->PassesValidityCheck(SM_VC_ALL, eCheckFailed) ;
            }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
          if(rpNewUVCurve->GetDim() != 2)
            {
              SM_ASSERT_MSG(rpNewUVCurve->GetDim() == 2, _T("SmEdgeuse::CreateUVTrimCurve created a non-2D UVTrimCurve")) ;
            }
#endif // SM_DEBUG_CODE

          // If we had to use a bigger tolerance, update the Edge's tolerance
          // according to the actual gaps.
          if ( ii > 1 )
            {
              double dError = ( pdMaxDeviation == NULL )
                                  ? rdMaxDistToSurf
                                  : smos_Sqrt (     rdMaxDistToSurf * rdMaxDistToSurf
                                                + dMaxDropApproxDev * dMaxDropApproxDev );
              SmEdge *pE = this->GetEdge();
              if ( pE ) { pE->SetTolerance( dError * 1.01 ); }
            }

          return SM_SUCCESS;

        } // end single UVTrimCurve with shorter param range than edge

    } // end iters trying DropCurve and DropAndTrimCurve


  // If we make it to here all other attempts have failed to create a curve
  // our last attempt is to tessellate the curve and drop points.

  // It works better to quit here, without doing the
  // brute-force point-dropping algorithm.  It's often because the 3d curve
  // runs across a seam, in an incomplete model.  In this case, this will
  // create a uv curve that jumps wildly from one end of the domain to the
  // other, rendering it misleading and mostly useless.  In such a case,
  // the caller can deal with the lack of a uv trim curve appropriately.
  // [B513 B556 B557]

  // However, there are cases where this is necessary,
  // so we have to leave it in for now.

  double dMaxDist = 0.0;
  double dMaxDev  = 0.0;

//  // 0: Do the brute-force algorithm. 1: Don't, return success.  2: SER.
//static int siUVDropBruteForce=0; // 0: for now, go ahead and do it.
//  // 2 works best with what is currently in place, including HW.
//  if ( siUVDropBruteForce==1 )
//    { return SM_SUCCESS; }
//  else if ( siUVDropBruteForce==2 )
//    { SER( SM_ERR ); }
//  else

  // Ok, do the brute-force algorithm.
    {
// GWC:FEWER_SURFACE_COPIES_CHANGE - TODO see if surface copy can be replaced
//                                   with temporary change of m_pOwner = NULL
      
      WARN(_T("SmEdgeuse::CreateUVTrimCurve(): dropping 101 EdgePts-to-FaceSrf because EdgeCrv-to-FaceSrf Drop failed [mightBe:EdgeCrossesSeam, NonG1, other]") );

      // Make a temp copy of the surface
      SmSurface *pTmpSurface = NULL;
      pSurface->Copy(*pContext,pTmpSurface);
      SmObjDelete sClean(pTmpSurface);

      // locals for GlobalPointSolve
      SmSolutionArray sSolutions;
      SmTArray<SmPoint3d> sPoints;
      SmTArray<double> sParams;
      double dUVTol = SM_EFF_ZERO_SQRT * (sFaceDomain.GetSize().Length() + 1.0);

      double dThisDist, dThisDev = 0.0;
      double *pThisDevPtr = ( pdMaxDeviation == NULL ) ? NULL : &dThisDev;

      // for 101 points
      for ( ii=0; ii<=100; ii++ )
        {
          // evaluate the curve at evenly spaced parameter points
          double dParam = sEdgeDomain.Evaluate( ii/(100.0) );
          SmPoint3d sPV[2];
          SER(pCurve->Evaluate(dParam,1,TRUE,sPV));

          // find closest point on surface to sample point
          SER(pTmpSurface->GlobalPointSolve(sFaceDomain,SM_SO_MINIMIZE,sPV[0],
              pEdge->GetTolerance(),NULL,SM_SR_ALL,sSolutions));

          // when a closest point was found
          if (sSolutions.GetSize() > 0)
            {
              // get the surface UV Point from the solution array
              SmPoint3d sPnt(sSolutions[0].m_vStart[0],
                             sSolutions[0].m_vStart[1],
                             0.0);

              // save the first sample Point and subsequent ones that are
              // far enough apart from one another
              if (   sPoints.GetSize() == 0
                  || sPnt.DistanceBetween(sPoints.GetLast()) > dUVTol)
                {
                  sPoints.Add(sPnt);
                  sParams.Add(dParam);

                  sm_CalcDistAndDev( sPV[0], sSolutions[0], dThisDist, pThisDevPtr );
                  dMaxDist = smos_Max( dThisDist, dMaxDist );
                  dMaxDev  = smos_Max( dThisDev , dMaxDev  );  // (Note, dMaxDev is 0 if ptr is Null.)
                } // end SavePoint Check
            } // end found nearest surface point check
        } // end iter every sample point

      // when no sample points projected to the surface
      if (sParams.GetSize() < 1)
        {
          // Set these anyway, could be useful to the caller.
          if ( pdMaxDeviation != NULL )
            {
              rdMaxDistToSurf = dMaxDist;
              *pdMaxDeviation = dMaxDev;
            }
          else
            { rdMaxDistToSurf = smos_Sqrt( dMaxDist*dMaxDist + dMaxDev*dMaxDev ); }

          SER(SM_ERR);
        }

      // when just 1 sample point projected to the surface
      if (sParams.GetSize() == 1)
        {
          // create and use a degenerate curve as the UVTrimCurve
          SmBSplineCurve *pDegenCurve = NULL ;
          SER(SmBSplineCurve::CreateDegenerateCurve(*pContext,2,sPoints[0],pDegenCurve));
          NER(pDegenCurve);
          rpNewUVCurve = pDegenCurve;
          rpNewUVCurve->SetOwner((SmEdgeuse *)this) ;
          pDropCurveFail->m_eCreatePathType = SM_CTC_101POINTDROP_DEGENCURVE_RECOVERY ;
          pDropCurveFail->m_pBrep = GetBrep() ;

          // clean up Surface Cache side-effect of attaching 2nd UVTrimCurve
          if(pUVTrimCurve == NULL && GetUVTrimCurve() != NULL)
            {
              ((SmEdgeuse *)this)->SetUVTrimCurve(NULL,0.0,TRUE,NULL) ; // TRUE = delete existing UVTrimCurve
              
            }

          // successfully constructed a UVTrimCurve

          // Set these anyway, could be useful to the caller.
          if ( pdMaxDeviation != NULL )
            {
              rdMaxDistToSurf = dMaxDist;
              *pdMaxDeviation = dMaxDev;
            }
          else
            { rdMaxDistToSurf = smos_Sqrt( dMaxDist*dMaxDist + dMaxDev*dMaxDev ); }

#ifdef SM_DEBUG_CODE
          if(bDebugMe2)
            {
              SM_ASSERT_VALID(rpNewUVCurve) ;
              rpNewUVCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed) ;
              rpNewUVCurve->PassesValidityCheck(SM_VC_ALL, eCheckFailed) ;
            }

          if(rpNewUVCurve->GetDim() != 2)
            {
              SM_ASSERT_MSG(rpNewUVCurve->GetDim() == 2, _T("SmEdgeuse::CreateUVTrimCurve created a non-2D UVTrimCurve")) ;
            }
#endif // SM_DEBUG_CODE

          return SM_SUCCESS;

        } // end 1 sample point check

      // else have several sample points - construct UVTrimCurve
      // to interpolate sample points
      pDropCurveFail->m_eCreatePathType = SM_CTC_101POINTDROP_DEGENCURVE_RECOVERY ;
      pDropCurveFail->m_pBrep = GetBrep() ;

      SmBSplineCurve *pNewUVCurve = NULL ;
      SER(SmBSplineCurve::InterpolatePoints(*pContext,sPoints,
                                            &sParams,
                                             3,       // in : degree
                                             NULL,NULL,FALSE,
                                             SM_IT_CHORDLENGTH,
                                             pNewUVCurve));
      NER(pNewUVCurve);
      pNewUVCurve->ConvertTo2D() ;
      rpNewUVCurve = pNewUVCurve;
      rpNewUVCurve->SetOwner((SmEdgeuse *)this) ;

      // clean up Surface Cache side-effect of attaching 2nd UVTrimCurve
      if(pUVTrimCurve == NULL && GetUVTrimCurve() != NULL)
        {
          ((SmEdgeuse *)this)->SetUVTrimCurve(NULL,0.0,TRUE,NULL) ; // TRUE = delete existing UVTrimCurve
        }

      // successfully constructed a UVTrimCurve

#ifdef SM_DEBUG_CODE
      if(bDebugMe2)
        {
          SM_ASSERT_VALID(rpNewUVCurve) ;
          rpNewUVCurve->PassesValidityCheck(SM_VC_REVERSE_DIRECTION, eCheckFailed) ;
          rpNewUVCurve->PassesValidityCheck(SM_VC_ALL, eCheckFailed) ;
        }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
      if(rpNewUVCurve->GetDim() != 2)
        {
          SM_ASSERT_MSG(rpNewUVCurve->GetDim() == 2, _T("SmEdgeuse::CreateUVTrimCurve created a non-2D UVTrimCurve")) ;
        }
#endif // SM_DEBUG_CODE

      // Set these anyway, could be useful to the caller.
      if ( pdMaxDeviation != NULL )
        {
          rdMaxDistToSurf = dMaxDist;
          *pdMaxDeviation = dMaxDev;
        }
      else
        { rdMaxDistToSurf = smos_Sqrt( dMaxDist*dMaxDist + dMaxDev*dMaxDev ); }

      return SM_SUCCESS;

  } // end block

  // Unreachable code
  // all attempts to construct UVTrimCurve failed - return an error code
  // if(pDropCurveFail->m_eCreatePathType == SM_CTC_UNKNOWN)
  //   { pDropCurveFail->m_eCreatePathType = SM_CTC_ALLRECOVERIES_FAIL ; 
  //     pDropCurveFail->m_pBrep = GetBrep() ;
  //   }
  // 
  // // Set these anyway, could be useful to the caller.
  // if ( pdMaxDeviation != NULL )
  //   {
  //     rdMaxDistToSurf = dMaxDist;
  //     *pdMaxDeviation = dMaxDev;
  //   }
  // else
  //   { rdMaxDistToSurf = smos_Sqrt( dMaxDist*dMaxDist + dMaxDev*dMaxDev ); }
  // 
  // return SM_ERR;

} // end SmEdgeuse::CreateUVTrimCurve

/*******************************************************************//**
PURPOSE: Evaluate the Binormal point and vector at a given parameter
   on an edgeuse.

NOTES: The binormal is a unit vector which is perpendicular to the
   Edgeuse->Face->Surface->Normal and the
   Edgeuse->Edge->Tangent (as adjusted for its orientation).
   It is a vector pointing towards the inner part of the face starting
   from the Edgeuse->Edge->EvaluatePoint(dParameter) point.
   The edgeuse tangent is adjusted to conform to the orientation of the edgeuse.

   This method works only on edgeuses which belong to a face.
***********************************************************************/
SmStatus SmEdgeuse::EvaluateBinormal // eff: binormal = crossProduct(surfaceNormal,CurveTangent)
  (double       dParameter,          // in : value within Edge->m_vInterval.
   SmBoolean    bUse3DOnly,          // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                     //      FALSE = get/create pUVTrimCurve to calc Surface points
   SmPoint3d  & rBinormalPoint,      // out: 3D pt on edge
   SmVector3d & rBinormal,           // out: unit-vector pointing to SmFace interior from rBinormalPoint
   SmVector3d * pOptEdgeuseTangent,  // out: opt non-unit Edgeuse tangent at rBinormalPoint, NULL to ignore, default:[NULL]                  
   SmVector3d * pOptFaceuseNormal,   // out: opt unit Faceuse normal, NULL to ignore, default:[NULL]                                         
   SmPoint2d  * pOptUVParameter,     // out: opt Surface param value at edge point, NULL to ignore, default:[NULL]                           
   SmVector2d * pOptUVVector)        // out: opt Surface Binormal param direction at edge point, not unitized, NULL to ignore, default:[NULL]
  const
{
  // check state - only Edgeuses in loops have a binormal vector
  if (m_tEdgeuseType != SmLoopuse_TYPE)
    { SER(SM_ERR_INVALID_INPUT); }

  // check state - dParameter not within Edge->Ivl
  if(FALSE == this->GetEdge()->GetInterval().ContainsValue(dParameter, SM_EFF_ZERO))
    {
      // inform the public - this is a bug and should be examined.  A lot of these calls seem to be coming from CurveOnClassify
      // where Edges are found that stop near but not on the Curve being classified.
      //                    |
      //  Edge1    Vertex   |
      // ----------------#  @ - PointClassifications - one from Vertex/ClassifyCrv XSect 
      //                 |  |                          one from Edge2Coincidence/ClassifyCrv XSect
      //                 |  |                          one from Edge1End/ClassifyCrv XSect
      //          Edge2  |  |     (Evaluating Edge1 at extended point @ causes this method
      //                 |  |      to have problems due to dParameter being outside of Edge->Ivl.)
      //                 |  |      
      //                 |  |-ClassifyCurve within Tol of Edge2, Vertex, and the end of Edge1
      //                    |
      //
      // The classification in 
      // SmCurveClassification::InstersectEdges uses the Near approach conditions as NearMiss intersections 
      // and places the approximate intersection point directly on the curve 
      // being classified.  That means the approximate point is now within tolerance of the end of the edge.
      // The approximate point is used to create a PointClassification
      // and when that PointClassification gets classified to classify all the intervals in the associated CurveClassification
      // this method is called which justly complains that it's been given a param outside the domain of the associated Edge.
      // The classification sequence needs to be thought through so that either these NearMiss intesections get culled
      // or the PointClassify change how it stores the param value for the Edge choosing instead a param value which would
      // extend the edge to the classifyCurve another value which has been snapped back to the EdgeEnd.  
      // This might help: When IntersectEdges() finds a NearMiss intersecion that starts the long sequence that ends up
      // here - IntersectVertices() has already found an intersection with a Vertex that is also nearby.  One thing the classify
      // code could do is recognize earlier than this call that the Edge NearMiss intersection should be coincident with
      // that Vertex intersection and as such, ignored.  Alternatively perhaps the PointClassify
      // should store both param values and use the appropriate one at the appropriate time.  This issue requires further study
      // than its getting at this time.
      SM_DBG_WARN(_T("SmEdgeuse::EvaluateBinormal: Outside of Edge->Ivl CrvParam being snapped to Ivl - This bug needs work.")) ;

      // snap Param value to Edge->Ivl endPoint
      dParameter = this->GetEdge()->GetInterval().ClampValue(dParameter) ;
      
    } // end need to clamp dParameter value check

  // Init outputs.
  rBinormalPoint.Set( 0,0,0 );
  if ( pOptEdgeuseTangent != NULL ) { pOptEdgeuseTangent->Set( 0,0,0 ); }
  if ( pOptFaceuseNormal  != NULL ) { pOptFaceuseNormal ->Set( 0,0,0 ); }
  if ( pOptUVParameter    != NULL ) { pOptUVParameter   ->Set( 0,0   ); }
  if ( pOptUVVector       != NULL ) { pOptUVVector      ->Set( 0,0   ); }

  //locals
  SmObjDelete sUVCurveCleanUp;
  SmEdge     *pEdge    = GetEdge();
  SmExtent1d  sEdgeIvl = pEdge->GetInterval() ;
  SmCurve    *p3DCurve = pEdge->GetCurve(); NER(p3DCurve);

  // evaluate the unoriented curve position and tangent values
  SmVector3d sCurvePV[3];  // big enough for 2 derivs: for nonregular case.
  SER( p3DCurve->Evaluate( dParameter, 1, TRUE, sCurvePV ));

  // while tangent length is zero - step along curve looking for a nonDegenerate tangent direction
  double dStepParam = dParameter;
  ULONG lSteps = 0;
  while (sCurvePV[1].LengthSquared() < SM_EFF_ZERO_SQ && lSteps < 10)
    {
      lSteps ++;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          p3DCurve->Dump();

          smgfx_Erase();
          smgfx_SetLook(2,3, 1,0,0); sCurvePV[1].Draw(&sCurvePV[0]); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); p3DCurve->DrawParams(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // First: check second derivative.  If it's not zero, then its
      //        direction is what the first deriv's direction will be as soon as
      //        we leave the singularity. [bd, 13 Sep 05]

      // Try eval from right
      SER( p3DCurve->Evaluate( dParameter, 2, TRUE, sCurvePV ));
      if ( sCurvePV[2].LengthSquared() >= SM_EFF_ZERO_SQ )
        {
          // assume 2nd deriv points pos at edge beg where parameterization is expected to be increasing
          // and neg at edge ends where parameterization is expected to be decreasing
          sCurvePV[1] = (sEdgeIvl.GetMid() > dParameter) ? sCurvePV[2] : -sCurvePV[2] ;
          break;
        }

      // Else try eval from left
      SER( p3DCurve->Evaluate( dParameter, 2, FALSE, sCurvePV ));
      if ( sCurvePV[2].LengthSquared() >= SM_EFF_ZERO_SQ )
        {
          sCurvePV[1] = (sEdgeIvl.GetMid() > dParameter) ? sCurvePV[2] : -sCurvePV[2] ;
          break;
        }

      // pick sequence of new param values progressively farther away from orig param
      double dNextStep =  (lSteps % 2 == 0)
                        ? lSteps * +SM_EFF_ZERO_SQRT
                        : lSteps * -SM_EFF_ZERO_SQRT ;
      if     (sEdgeIvl.ContainsValue(dParameter + dNextStep)) { dStepParam = dParameter + dNextStep ; }
      else if(sEdgeIvl.ContainsValue(dParameter - dNextStep)) { dStepParam = dParameter - dNextStep ; }
      else { SER(SM_ERR) ; }

      // evaluate unoriented curve at modified param value
      SmVector3d sPV2[2];
      SER(p3DCurve->Evaluate(dStepParam,1,TRUE,sPV2));
      sCurvePV[1] = sPV2[1];

    } // end while tang vector length == 0.0 - seek nonDegenerate Tangent evals

  // Reverse orientation of curve tangent to conform to edgeuse tangent.
  if (GetOrientation() == SM_OT_OPPOSITE)
    {
      sCurvePV[1] = - sCurvePV[1];
    }

  // face and surface locals
  SmFace          * pFace            = GetFace();           SM_ASSERT(pFace != NULL);
  SmSurface       * pSurface         = pFace->GetSurface();
  SmExtent2d        sDomain          = pFace->GetUVDomain();
  SmBSplineCurve  * pUVTrimCurve     = NULL;
  SmDropCurveFail * pUVDropCurveFail = NULL ; 

  // If we already have a uv trim curve, then we can assume it's valid.  [B556]
  pUVTrimCurve     = this->GetUVTrimCurve();
  pUVDropCurveFail = this->GetDropCurveFail() ;

  // Work in UV Space when good pUVTrimCurve is available
  if(   (pUVTrimCurve != NULL)                                     //    When a UVTrimCurve has not yet been built
     && (   (pUVDropCurveFail == NULL)                             // and a previous UVTrimCurve try has yet to fail
         || (   pUVDropCurveFail->m_sFailStatus != SM_SUCCESS 
             && pUVDropCurveFail->m_sFailStatus != SM_ERR_UNKNOWN)))
    { bUse3DOnly = FALSE; }

  // when trying to work in UVSpace - make sure to get a UVTrimCurve
  if(   (pUVTrimCurve == NULL)               //     When a UVTrimCurve has not yet been built (when healing - might be couldn't)
     && (bUse3DOnly == FALSE)                // and when asked to try UVTrimCurves
     && (   (pUVDropCurveFail == NULL)       // and don't try building a UVTrimCurve known to be bad
         || (pUVDropCurveFail->m_sFailStatus != SM_ERR_UNKNOWN)))
    {
      // Get our uv trim curve.
      // Create it if necessary.  [B438]
      // Note: a bit of a hack.  (Besides the non-const cast.)
      // CreateUVTrimCurve() can call this method (EvaluateBinormal),
      // which would seem to invite infinite recursion.
      // However, that method calls this one only with bUse3DOnly set to TRUE,
      // so this will never happen.

      SmEdgeuse *pNonConstThis = SM_CONST_CAST( SmEdgeuse*, this );
      pNonConstThis->GetOrCreateUVTrimCurve( pUVTrimCurve );
    }

  SmPoint2d sUV;
  SmPoint2d sUVTest;

  SmBoolean bGotUV = FALSE;

  // For the following Local- and GlobalPointSolves,
  // use MINIMIZE.  (Was INTERSECT; don't see any reason for that.)
  SmSolverOperationType eSolverOp = SM_SO_MINIMIZE;

  // eval pUVTrimCurve on clamped param values and clamp resulting uv point
  if ( !bGotUV && pUVTrimCurve != NULL )
    {
      // clamp the parameter value
      dParameter = pUVTrimCurve->GetNaturalInterval().ClampValue( dParameter );

      // compute UVTrimCurve Point and clamp that to SurfaceDomain
      SmPoint3d sPnt;
      SER( pUVTrimCurve->EvaluatePoint( dParameter, sPnt ));
      sUV = sDomain.ClampPoint2d( SmPoint2d( sPnt.x, sPnt.y ));

      bGotUV = TRUE;

      // Double check the validity of this uv point.
      double dTol = pEdge->GetTolerance() * 2.0;

      if ( dTol > 0 )
        {
          // Allow for the 3d curve point being off the surface.  (It happens.)
          // Check the dist from the 3d curve point to the surface normal.
          SmVector3d sThisSrfPt, sThisSu, sThisSv, sThisNorm;
          pSurface->Evaluate1stDerivatives( sUV,        // in : target
                                            FALSE,      // in : from ULeft
                                            FALSE,      // in : from URight
                                            sThisSrfPt, // out: 
                                            sThisSu,    // out: 
                                            sThisSv );  // out: 

          // try a cheap eval for surface normal
          sThisNorm = sThisSu * sThisSv;
          double dLen = sThisNorm.Length();
          if ( dLen > pSurface->GetNormalLimit() )
            { sThisNorm /= dLen; }
          else // when cheap is degenerate - try expensive normal eval
            { pSurface->EvaluateNormal( sUV, FALSE, FALSE, sThisNorm ); } // sThisNorm is returned unitized 

          // When minDist(surfacePt/surfaceNorm line, EdgePoint) < dTol - use the UV point
          SmVector3d sGap( sCurvePV[0] - sThisSrfPt );
          double     dDot = sGap.Dot( sThisNorm );
          SmPoint3d  sNormPt( sThisSrfPt + dDot * sThisNorm );

          bGotUV = sNormPt.CloserThan( dTol, sCurvePV[0] );

          // When edge point is too far from surface normal line,
          // try a LocalSolve using sUV as a guess.
          // (Much cheaper than GlobalPointSolve, next.)
          if ( !bGotUV )
            {
              SmSolution sSol;
              pSurface->LocalPointSolve( sDomain, eSolverOp, sCurvePV[0], sUV, bGotUV, sSol );
              if ( bGotUV && sSol.m_vStart.m_dSolutionValue > dTol )
                { bGotUV = FALSE; }
              if ( bGotUV )
                { sUV.Set( sSol.m_vStart[0], sSol.m_vStart[1] ); }
            }
        } // end if double-checking result of pUVTrimCurve (dTol > 0)

    } // end if have pUVTrimCurve

  // arrive here with or without a sUV point marking the face point for the given edge point.
  //   May have a sUV point if(bUse3DOnly == TRUE) and UVTrimPoint projected to surface was close to edge point.

  // when the edgePoint isn't matched to a surface sUV point
  if ( !bGotUV )
    {
      // Drop edge point to surface to get uv point.

      // intersect locals
      double dTol = pEdge->GetTolerance();
      SmSolution      sSolData[3];
      SmSolutionArray sSolutions(3,sSolData);

      // get and lock the surface cache
      SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pSurface);  NER(pSC);
      SmCacheCheckOutIn sCheckIO(pSC);

      { // local scope for TemporaryChange variables

        // Turn off point testing so GlobalPointSolve() will keep all point solutions
        //   without classifying the solution point against the trim boundaries.
        SmTemporaryChangeValue<SmBoolean> sTCV(pSC->m_bPointTestEnabled,FALSE);

        // Turn on boundary curve processing to force LocalSolve to look for drop points
        //   on boundary curves. This will find solutions where the curve comes close
        //   to the surface but does not actually intersect it.
        SmTemporaryChangeValue<SmBoolean> sTBC(pSC->m_bProcessBoundaryCurves,TRUE);

        // This 'turns off point testing' when GPS does not use the cache:
        SmTemporaryChangeValue<SmObject*> sTON( pSurface->m_pOwner, NULL );

        // As does this: use surface's domain instead of Face's domain.
        SmExtent2d sSrfDomain = pSurface->GetNaturalUVDomain();

        // Find Surface sUV point for EdgePoint
        SER(pSurface->GlobalPointSolve(sSrfDomain,          // domain to search
                                       eSolverOp,
                                       sCurvePV[0],         // target point to work with
                                       dTol,
                                       NULL,
                                       SM_SR_ALL,
                                       sSolutions));
        if (sSolutions.GetSize() == 0 )
          {
            // try again with a bigger tolerance
            smos_WriteBuffer(_T("Tolerance Problem - increasing by 1000x\n"));
            dTol = dTol * 1000;
            SER(pSurface->GlobalPointSolve(sSrfDomain,
                                           eSolverOp,
                                           sCurvePV[0],
                                           dTol,
                                           NULL,
                                           SM_SR_ALL,
                                           sSolutions));
          } // end no solutions check

        // when there are no solutions
        if (sSolutions.GetSize() == 0)
          {
            // We should now have a UV curve, so use it unless indicated otherwise.
            if ( !bUse3DOnly )
              {
                // get sUVTrimCurve
                SmBSplineCurve *pUVTrimCrv = GetUVTrimCurve();
                if (!pUVTrimCrv)
                  {
                    double dDist;
                    // (See Hack warning, above.)
                    SER(CreateUVTrimCurve(dDist, pUVTrimCrv )); NER( pUVTrimCrv );
                    sUVCurveCleanUp.SetObj( pUVTrimCrv );
                  }

                // set sUV based on sUVTrimCurve eval clamped to surface domain
                SmPoint3d sPnt;
                dParameter = pUVTrimCrv->GetNaturalInterval().ClampValue(dParameter);
                SER( pUVTrimCrv->EvaluatePoint(dParameter,sPnt));
                sUV = sDomain.ClampPoint2d(SmPoint2d(sPnt.x,sPnt.y));
              }
            else // no hope - can't get edge point to project to surface
              {
                SER(SM_ERR);
              }
          } // end no solutions branch

        else if (sSolutions.GetSize() == 1)
          {
            // the solution is the sUV point
            SmSolution & rSol = sSolutions[0];
            sUV = SmPoint2d(rSol.m_vStart[0], rSol.m_vStart[1]);
          } // end 1 solution branch

        // If on a seam, doesn't matter which we use, the normal vector will be the same.
        else if (   sSolutions.GetSize() == 2
                 && pSurface->IsOnSeam( SmPoint2d( sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1] ) )
                 && pSurface->IsOnSeam( SmPoint2d( sSolutions[1].m_vStart[0], sSolutions[1].m_vStart[1] ) )
                )
          {
            // either solution can be the sUVPoint and we have no way of telling which to choose
            // Doesn't matter, normal is the same.
            SmSolution & rSol = sSolutions[0];
            sUV = SmPoint2d( rSol.m_vStart[0], rSol.m_vStart[1] );
          } // end 2 solutions on a seam branch

        else  // Multiple solutions or not seam.
          {
            // let sUV be the solution closest to the EdgePoint
            double dMinDist = SM_BIG_DOUBLE;
            for (ULONG i=0; i<sSolutions.GetSize(); i++)
              {
                SmSolution & rSol  = sSolutions[i];
                SmPoint2d    sUV2  = SmPoint2d(rSol.m_vStart[0],rSol.m_vStart[1]);
                double       dDist = rSol.m_vStart.m_dSolutionValue;
                if (dDist < dMinDist) { dMinDist = dDist;
                                        sUV = sUV2;
                                      }
              }  // end for all solutions found
          }  // end multiple-solutions-not-seam branch
      }  // end local scope for temp-change-values
    } // end no pUVTrimCurve to use to get a surface uv point branch


  // State: we have the surface parameter in sUV.

  // get oriented Faceuse Normal vector and Binormal, which is Normal <cross> edge tangent
  SmVector3d sNormal, sBinormal;

  // Deal with potential discontinuities in the curve or surface.
  // If we get a zero binormal, try small step-offs.  [230227_crease]
  double dCrvStep = sEdgeIvl.GetLength() / 1000.0;
  SmVector2d sSrfStep( sDomain.GetSize() / 1000.0 );
  SmPoint2d  sStepUV( sUV );
  double dTolUV = smos_Min( sSrfStep.x, sSrfStep.y ) / 2.0;

  // Don't re-evaluate the curve step-offs on each surface iteration.
  SmBoolean bHaveCrvEvalLo = FALSE, bHaveCrvEvalHi = FALSE; 
  SmVector3d sCurvePVLo[2], sCurvePVHi[2];

  // Loop over four step-off directions on the surface.
  // In each iteration, try two curve step-offs.
  for ( ULONG ii = 0; ii<5; ii++ )
  {
      switch ( ii ) {
        case 1: { sStepUV.Set( sUV.x + sSrfStep.x, sUV.y );  break; }
        case 2: { sStepUV.Set( sUV.x - sSrfStep.x, sUV.y );  break; }
        case 3: { sStepUV.Set( sUV.x, sUV.y + sSrfStep.y );  break; }
        case 4: { sStepUV.Set( sUV.x, sUV.y - sSrfStep.y );  break; }
      }

      // If the step puts us outside the basic domain, don't use it.
      // (But if the input uv is out, skipping it could skip everything.)
      if ( ii > 0  &&  ! sDomain.ContainsPoint2d( sStepUV, dTolUV ) )
        { continue; }

      SER( pSurface->EvaluateNormal( sStepUV, TRUE, TRUE, sNormal ));

      if (GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE)
        { sNormal = - sNormal; }

      // let binormal = crossProduct(Faceuse->Normal, Edgeuse->Normal).
      //    A person walking along the Edgeuse from Start to End will
      //    have the interior of the face to his lefthand side.
      sBinormal = sNormal * sCurvePV[1];

      if ( sBinormal.LengthSquared() > SM_EFF_ZERO_SQ )
        { break; } // we're good.

      // when Binormal is zero length - try curve eval just below given param point
      if ( sEdgeIvl.GetMin() < dParameter - dCrvStep )
        {
          if ( ! bHaveCrvEvalLo )
          {
              SER( p3DCurve->Evaluate( dParameter-dCrvStep, 1, TRUE, sCurvePVLo ));
              bHaveCrvEvalLo = TRUE;
          }
          sCurvePV[1] = sCurvePVLo[1];

          // Reverse orientation of curve tangent to conform to edgeuse tangent.
          if ( GetOrientation() == SM_OT_OPPOSITE )
            { sCurvePV[1] = - sCurvePV[1]; }

          sBinormal = sNormal * sCurvePV[1];

          if ( sBinormal.LengthSquared() > SM_EFF_ZERO_SQ )
            { break; } // we're good.
        }

      // when binormal is still zero length - try curve eval just above given param point
      if ( sEdgeIvl.GetMax() > dParameter + dCrvStep )
        {
          if ( ! bHaveCrvEvalHi )
          {
              SER( p3DCurve->Evaluate( dParameter+dCrvStep, 1, TRUE, sCurvePVHi ));
              bHaveCrvEvalHi = TRUE;
          }
          sCurvePV[1] = sCurvePVHi[1];

          // Reverse orientation of curve tangent to conform to edgeuse tangent.
          if ( GetOrientation() == SM_OT_OPPOSITE )
            { sCurvePV[1] = - sCurvePV[1]; }

          sBinormal = sNormal * sCurvePV[1];

          if ( sBinormal.LengthSquared() > SM_EFF_ZERO_SQ )
            { break; } // we're good.
        }

  } // end loop on surface-evaluation step-offs.

  // give up if binormal is still zero
  if (sBinormal.LengthSquared() < SM_EFF_ZERO_SQ)
    {
      SER(SM_ERR);
    }

  // normalize the return vector
  sBinormal.Unitize();
  rBinormalPoint = sCurvePV[0];
  rBinormal      = sBinormal;

  // store optional output values
  if (pOptEdgeuseTangent) { *pOptEdgeuseTangent = sCurvePV[1] ; }
  if (pOptFaceuseNormal)  { *pOptFaceuseNormal  = sNormal; }
  if (pOptUVParameter)    { *pOptUVParameter    = sUV ; }
  if (pOptUVVector)       {  pSurface->ComputeParamDirection(sUV, sBinormal, *pOptUVVector) ; 
                          }

  // all done
  return SM_SUCCESS;

} // end SmEdgeuse::EvaluateBinormal

/*******************************************************************//**
   Local stuff
***********************************************************************/

class SmFindStepOffENFO : public SmEvalNFunctionsObject
{
protected:
    double m_dStepOffDist;
    const SmPoint3d  & m_crPoint;
    const SmVector3d & m_crPlaneNormal;
    const SmSurface  & m_crSurface;
public:
    SmFindStepOffENFO(double dStepOffDist,
                      const SmPoint3d & crPoint,
                      const SmVector3d & crPlaneNormal,
                      const SmSurface & crSurface)
      : m_dStepOffDist(dStepOffDist),
        m_crPoint(crPoint),
        m_crPlaneNormal(crPlaneNormal),
        m_crSurface(crSurface) {}

    virtual ~SmFindStepOffENFO() {}

    virtual SmStatus Evaluate(const SmTArray<double> & crX,             // in : x of Ax=F
                              SmTArray<double>       & rF,              // out: F of Ax=F function values,
                              SmMatrix               * pOptJacobian,    // out: Partial derivatives of the functions.
                              SmBoolean              & rbFoundAnswer);  // out: Not always used, when used
                                                                        //      TRUE = converged (F members are within tolerance of 0.0
                                                                        //      FALSE= Not Used or Not Converged
} ; // end class SmFindStepOffENFO

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmFindStepOffENFO::Evaluate
  (const SmTArray<double> & crX,           // in : x of Ax=F
   SmTArray<double>       & rF,            // out: F of Ax=F function values,
   SmMatrix               * pOptJacobian,  // out: Partial derivatives of the functions.
   SmBoolean              & rbFoundAnswer) // out: TRUE =
                                           //      FALSE=

{
    SM_ASSERT(crX.GetSize() == 2);
    SM_ASSERT(rF.GetSize() == 2);
    if (pOptJacobian) {
        SM_ASSERT(pOptJacobian->GetNumRows() == 2);
        SM_ASSERT(pOptJacobian->GetNumColumns() == 2);
    }

    rbFoundAnswer = FALSE;
    SmPoint3d sPoint;
    SmVector3d sDU, sDV;
    SmPoint2d sUV(crX[0],crX[1]);
    SER(m_crSurface.Evaluate1stDerivatives(sUV,TRUE,TRUE,sPoint,sDU,sDV));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,1,0);
        m_crSurface.DrawAt(sUV,1);
        sm_GraphicsLoop();
    }
#endif

    double sDULenSq = sDU.LengthSquared();
    double sDVLenSq = sDV.LengthSquared();
    // if we have a zero length tangent then we must fail
    if (sDULenSq < SM_EFF_ZERO_SQ || sDVLenSq < SM_EFF_ZERO_SQ) {
        return SM_ERR;
    }

    SmVector3d sVecSMinusP = sPoint - m_crPoint;
#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        sVecSMinusP.Draw(&m_crPoint);
        sm_GraphicsLoop();
    }
#endif

    // Solve the following two equations.  What they do is find the
    // intersection of a plane and a surface at a given distance
    // from the original point.  It can also be thought of as finding
    // the intersection of a surface with a circle in the plane.
    // fun[0] = (s-p)(s-p) - dStepOffDist*dStepOffDist
    // fun[1] = (s . plane_normal) + plane_D
    //
    //  |2(s-p)du            2(s-p)dv            |
    //  |(du.N)              dv.N                |
    //
    double dPlaneD = - m_crPlaneNormal.Dot(m_crPoint);

    if (pOptJacobian) {
        (*pOptJacobian)[0][0] = 2.0 * sDU.Dot(sVecSMinusP);
        (*pOptJacobian)[0][1] = 2.0 * sDV.Dot(sVecSMinusP);
        (*pOptJacobian)[1][0] = sDU.Dot(m_crPlaneNormal);
        (*pOptJacobian)[1][1] = sDV.Dot(m_crPlaneNormal);
    }

    // Compute function values
    rF[0] = sVecSMinusP.Dot(sVecSMinusP) - m_dStepOffDist*m_dStepOffDist;
    rF[1] = sPoint.Dot(m_crPlaneNormal) + dPlaneD;

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        if (pOptJacobian) pOptJacobian->Dump();
        smos_WriteBuffer(_T(" X --- \n"));
        crX.Dump();
        smos_WriteBuffer(_T(" F --- \n"));
        rF.Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmFindStepOffENFO::Evaluate

/*******************************************************************//**
PURPOSE: Determine the binormal of a surface given a stepoff distance
    to step into the face from the edge.

NOTES:
    Let binormVector = CrossProduct(SurfaceNormal(StepOffPoint),
                                    CurveTangent(param))
     A person walking along the Edgeuse from Start to End on the positive side 
     of the faceuse will have the interior of the face to his lefthand side.

    StepOffPoint = point on surface dStepOffDistance from
                   CurvePoint(param) found by intersecting
                   1. Surface
                   2. Plane perp to curve at nearest point on surface to Curve(param)
                   3. Sphere of radius(StepOffDistance) centered on nearest point on surface to Curve(param)

    The function can be used in slightly different fashions by
    controlling the computation of the StepOffPoint.
    Use pOptCenterPoint to specify the plane point and the sphere's center.
    Use pOptPlaneNormal to specify the direction of the plane.

    Returns SM_ERR
       - for cascading errors when internal evaluators fail and
       - when the step into the surface steps off the face UVDomain.
         This happens for large StepOffDistances and sliver surfaces.
         NOTE: step points are not checked against the trimcurves of the
               face, so if a face is trimmed it is possible to get an
               answer back for a point that is within the UVDomain of the face
               but outside trim curves of that face.
***********************************************************************/
SmStatus SmEdgeuse::EvaluateBinormalStepOff  // eff: binormal=crossProduct(SurfaceNormal(StepOffPoint),CurveTangent(param))
  (double dParameter,                        // in : curve parameter
   double dStepOffDistance,                  // in : 3d distance to step (0.0 is ok)
   SmPoint3d  & rBinormalPoint,              // out: Surface position for given param and StepOffDistance
   SmVector3d & rBinormal,                   // out: rBinormal = CrossProduct(sNormal,sTangent at stepoff point
   SmVector3d * pOptEdgeuseTangent,          // out: curve tangent at param, NULL to ignore
   SmVector3d * pOptEdgeuseFaceNormal,       // out: surface normal at stepOffPoint, NULL to ignore
   SmPoint3d  * pOptCenterPoint,             // in : surface point at step off distance, NULL to ignore
                                             //      default:[NULL] = use surfacePoint nearest EdgePoint at given param
   SmVector3d * pOptPlaneNormal,             // in : define plane to measure step off distance, NULL to ignore
                                             //      default:[NULL] = use EdgeuseTangent at given param
   SmVector2d * pOptStepOffUV,               // out: found surface StepOff UVPoint, NULL to ignore default:[NULL]
   SmBoolean    bOKToMakeUVTrimCurve)        // in : default:[TRUE] = okay to build UVTrimCurves when missing
                                             //    : FALSE= Only use existing UVTrimCurves or work in 3d because UVTrimCurve Creation is dodgy during healing
 const
{
  SmPoint2d sUV;
  SmVector3d sTangent;

  // evaluate binormal at the edge - set outputs for stepOffDistance = 0.0
  SER(EvaluateBinormal(dParameter,           // in : value within Edge->m_vInterval.
                       !bOKToMakeUVTrimCurve, // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                             //      FALSE = get/create pUVTrimCurve to calc Surface points
                       rBinormalPoint,       // out: 3D pt on edge
                       rBinormal,            // out: unit-vector pointing to SmFace interior from rBinormalPoint
                       &sTangent,            // out: non-unit Edgeuse tangent at rBinormalPoint
                       pOptEdgeuseFaceNormal,// out: unit Faceuse normal at StepOff point
                       &sUV));               // out: Surface param value at edge point

  // capture remaining opt outputs
  if(pOptEdgeuseTangent)  { *pOptEdgeuseTangent = sTangent; }
  if(pOptStepOffUV)       { *pOptStepOffUV      = sUV ; }

  // low work - StepOffDistance = 0.0
  if (dStepOffDistance < SM_EFF_ZERO)
    {
      return SM_SUCCESS;
    }

  // edgeuse locals
  SmFace    *pFace    = GetFace(); NER(pFace);
  SmSurface *pSurface = pFace->GetSurface(); NER(pSurface);
  SmPoint3d sSurfPoint;

  // evaluate surface at edge point
  SER(pSurface->EvaluatePoint(sUV,sSurfPoint));

  // set startpoint = stepoffdistance from surfPoint in binormal direction
  SER(rBinormal.Unitize());
  SmPoint3d sStartPoint = sSurfPoint + dStepOffDistance * rBinormal;
  SmPoint2d sUVGuess[1];

  // drop start point to surface to get a guess point
  SER(pSurface->DropPointsToTangentPlane(sUV,TRUE,TRUE, 1,&sStartPoint,sUVGuess));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      pSurface->Dump() ;

      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      rBinormal.Draw(&sSurfPoint);
      sm_GraphicsLoop();
  }
#endif

  // Return an error if we step off the face UVDomain.
  if (!pFace->GetUVDomain().ContainsPoint2d(sUVGuess[0]))
    {
      // We may have a periodic surface or something so we should drop the
      // point just to find something.
      SmSolutionArray sSolutions;
      // Use NORMALIZE here: if the point is past the end of the Face,
      // we don't want it returning success with a boundary point with
      // large deviation from the normal: defeats the purpose. [B579]
      SER(pSurface->GlobalPointSolve(pFace->GetUVDomain(), SM_SO_NORMALIZE, sStartPoint,
          pFace->GetTolerance(),NULL,SM_SR_ALL,sSolutions));
      if(sSolutions.GetSize() > 0)
        {
          sUVGuess[0].x = sSolutions[0].m_vStart[0];
          sUVGuess[0].y = sSolutions[0].m_vStart[1];

          // still stepping off face UVDomain
          if(!pFace->GetUVDomain().ContainsPoint2d(sUVGuess[0]))
            {
              return SM_ERR ; 
            }
        }
      else // stepoffdistance stepped off face UVDomain
        {
          return SM_ERR;
        }
    } // end stepped off face check

  SmExtentNd sIntervals(2);
  SmBoolean           alPerData[2];
  SmTArray<SmBoolean> sPeriodicities(2,alPerData,2);
  double           adGuessData[2];
  SmTArray<double> sGuessT(2,adGuessData,2);
  double           adSolData[2];
  SmTArray<double> sSolutionVector(2,adSolData);

  SmExtent2d sUVDomain = pFace->GetUVDomain();
  sIntervals[0] = SmExtent1d(sUVDomain.GetMin().x,sUVDomain.GetMax().x);
  sIntervals[1] = SmExtent1d(sUVDomain.GetMin().y,sUVDomain.GetMax().y);

  // In general when doing local solver don't check for periodicity.
  sPeriodicities[0] = FALSE;
  sPeriodicities[1] = FALSE;

  sGuessT[0] = sUVGuess[0].x;
  sGuessT[1] = sUVGuess[0].y;
  SmBoolean bFoundSolution;

  // use local solver to find point on surface stepOffDistance from edge point in binormal direction.
  if (pOptCenterPoint) sSurfPoint   = *pOptCenterPoint;
  SmVector3d           sPlaneNormal = sTangent;
  if (pOptPlaneNormal) sPlaneNormal = *pOptPlaneNormal;

  SmFindStepOffENFO sEvalFun(dStepOffDistance,sSurfPoint,
                             sPlaneNormal,*pSurface);
  SmLocalSolveNd sLS(sEvalFun,          // in : Define Eqns to set to Zero (defines the DOF COUNT)
                     &sIntervals,       // in : NULL or sized:[N], N = this rEvalFun DOF count
                                        //      bounds on problem parameters.
                                        //      Set m_cpIntervals[i].SetUnbounded() for ivl whose prob params don't get clamped
                     &sPeriodicities) ; // in : NULL or sized:[N], N = this rEvalFun DOF count
                                        //      ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                        //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped

  sLS.SetBoundaryHandler(SM_BH_TOTAL_BOUNDARY_HITS,3);

  // intersect: 1. plane perp to curve (or pOptPlaneNormal) at sSurfacePoint (or pOptCenterPoint)
  //            2. surface
  //            3. Sphere with StepOffDistance radius centered on sSurfPoint
  // to find point on surface in binormal direction at a StepOffDistance from sSurfPoint.
  SER(sLS.SolveIt(sGuessT,SM_EFF_ZERO_SQRT*100.0,bFoundSolution,sSolutionVector));

  // If we didn't converge just take the guess point as good enough.
  if (!bFoundSolution)
    {
      SmPoint3d sUVPnt = sUVDomain.ClampPoint2d(sUVGuess[0]);
      sSolutionVector[0] = sUVPnt.x;
      sSolutionVector[1] = sUVPnt.y;
    }

  // evaluate stepoffPoint face normal - (negate from surf normal for opposite face orientations)
  SmPoint2d sUVFound(sSolutionVector[0],sSolutionVector[1]);

  SmVector3d sNormal;
  SER(pSurface->EvaluateNormal(sUVFound,TRUE,TRUE,sNormal));
  if (GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE)
    {
      sNormal = - sNormal;
    }

  // set output
  rBinormal = sNormal * sTangent;
  SER(pSurface->EvaluatePoint(sUVFound,rBinormalPoint));
  if(pOptEdgeuseFaceNormal) { *pOptEdgeuseFaceNormal = sNormal; }
  // pOptEdgeuseTangent value already set
  if(pOptStepOffUV)         { *pOptStepOffUV         = sUVFound ; }

  return SM_SUCCESS;

} // end SmEdgeuse::EvaluateBinormalStepOff

/*******************************************************************//**
PURPOSE: This method will get the edgeuse mate of the given edgeuse.

NOTES: The mate is the corresponding oppositely oriented edgeuse 
       on the other side of the face.
  The mate is stored next to this edgeuse on the Edgeuse->Edge's EdgeuseList
  if(ThisEdgeuse orient == PrimaryEdgeuse Orient) Mate = Next Edgeuse on Edge's Edgeuselist
  if(ThisEdgeuse orient != PrimaryEdgeuse Orient) Mate = Prev Edgeuse on Edge's Edgeuselist
***********************************************************************/
SmEdgeuse* SmEdgeuse::GetMate() const
{
   if (IsShellEdgeuse()) { return (SmEdgeuse*)GetNext(); }

   SmEdge    *pE      = GetEdge();
   SmEdgeuse *pPrimEU = pE->GetPrimaryEdgeuse();
   SmEdgeuse *pRet    = NULL;
   if (pPrimEU->GetOrientation() == GetOrientation())
     {
       pRet = (SmEdgeuse*)GetNext();
     }
   else
     {
       pRet = (SmEdgeuse*)GetLast();
     }
   return pRet;

} // end SmEdgeuse::GetMate

/*******************************************************************//**
PURPOSE: This method will get the radial edgeuse of the given edgeuse.

NOTES: The radial is the corresponding oppositely oriented edge on the adjacent
    face.  Note that the radial is on the opposite side as the mate.
***********************************************************************/
SmEdgeuse* SmEdgeuse::GetRadial() const
{
  if (IsShellEdgeuse()) { return (SmEdgeuse*)GetNext(); }

  SmEdge    *pE      = GetEdge();
  SmEdgeuse *pPrimEU = pE->GetPrimaryEdgeuse();
  SmEdgeuse *pRet    = NULL;
  if (pPrimEU->GetOrientation() == GetOrientation())
    {
      pRet = (SmEdgeuse*)GetLast();
    }
  else
    {
      pRet = (SmEdgeuse*)GetNext();
    }
  return pRet;

} // end SmEdgeuse::GetRadial


/*******************************************************************//**
PURPOSE: Determine if the edgeuse defines a convex radial sector.

NOTES: That is that the angle between the face connected to this
    edgeuse and the face connected to this edgeuse's radial partner
    is greater than 180 degrees at all sample points.
    We allow tested points to be slightly concave as specified by
        tolerance_degrees
    Users can set optional bOptStepOff to TRUE when edgeuse
    bounds two tangential(i.e. G1) surfaces. The default value is FALSE
***********************************************************************/
SmBoolean SmEdgeuse::IsConvexRadialSector
  (ULONG lNumSamples,           // in : number of evenly spaced internal samples (end points are not tested)
   SmBoolean bOptStepOff,       // in : TRUE  = test points offset (but near the edge)
                                //      FALSE = test points along edge
                                //      Can classify a G1 edge connected to faces
                                //      that quickly curve away from the tangent plane.
   double dTolDeg)              // tolerance in degrees (= 0.0)  Default = 0.0
  const
{
  // locals: radial partner, edge->Interval
  SmEdgeuse  * pRadial      = GetRadial();
  SmExtent1d   sIvl         = GetEdge()->GetInterval();
  double dStepOffDist=(bOptStepOff)? 20.0 * GetEdge()->GetTolerance():0.0;

  // convert to radians
  double dDotProductLimit = (dTolDeg != 0.0) ? cos( SM_PI*(90.0 - dTolDeg)/180.0 ) : -SM_EFF_ZERO;

  // for every requested sample point
  for (ULONG i=0; i<lNumSamples; i++)
    {
      // sample evenly skipping endPoints
      double dT = (i + 1.0) / (lNumSamples + 1.0);
      SmPoint3d  sPnt,  sPnt2;
      SmVector3d sBin,  sBin2;
      SmVector3d sNorm, sNorm2;

      // replaced EvaluateBinormal with EvaluateBinormalStepOff
      SER(EvaluateBinormalStepOff(sIvl.Evaluate(dT),dStepOffDist,sPnt,sBin,NULL,&sNorm));
      SER(pRadial->EvaluateBinormalStepOff(sIvl.Evaluate(dT),dStepOffDist,sPnt2,sBin2,NULL,&sNorm2));
      if ( dTolDeg != 0.0) {
         sBin.Unitize();
         sNorm.Unitize();
         sBin2.Unitize();
         sNorm2.Unitize();
      }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw thisEdgeuse BinormalVector and NormalVector(red), radialEdgeuse BinormalVector and NormalVector(blue)
  if(bDebugMe)
    {
      smgfx_SetLineWidth(3.0) ;
      smgfx_SetColor(1,0,0) ; sBin.Draw(&sPnt) ; sm_GraphicsLoop() ;
      smgfx_SetColor(0,1,0) ; sNorm.Draw(&sPnt) ; sm_GraphicsLoop() ;
      smgfx_SetColor(0,0,1) ; sBin2.Draw(&sPnt2) ; sm_GraphicsLoop() ;
      smgfx_SetColor(0,1,1) ; sNorm2.Draw(&sPnt2) ; sm_GraphicsLoop() ;
      smgfx_SetLineWidth(1.0) ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // when SurfaceNormal is in same direction of OtherSurface Binormal - its not convex
      if (   sNorm2.Dot(sBin) > dDotProductLimit
          && sNorm.Dot(sBin2) > dDotProductLimit)
        {
          return FALSE;
        }
    } // end iter every requested sample point

  return TRUE;

} // end SmEdgeuse::IsConvexRadialSector

/*******************************************************************//**
PURPOSE: see if UVTrimCurve ControlPolygon turns and follows a surface singularity boundary

RETURNS ---  TRUE  = Problem - UVTrimCurves should have all or just end 
                               Control Points on Surface Singularity boundaries
             FALSE = okay - no or just end ControlPoints are on Surface 
                               Singularity boundaries
***********************************************************************/
SmBoolean SmEdgeuse::HasDogLegNearPole
 (SmBoolean * pOptIndx)  // out: CPt Index at first turn, NULL to ignore, default:[NULL]
const
{
  SmBSplineCurve * pUVTrimCurve = GetUVTrimCurve()  ;

  // init return value
  SmBoolean bHasDogLegNearPole = FALSE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      pUVTrimCurve->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // no work - no UVTrimCurve to check
  if(pUVTrimCurve == NULL)
    { return FALSE ; }

  // locals
  SmFace    * pFace    = GetFace() ;
  SmSurface * pSurface = pFace ? pFace->GetSurface() : NULL ;

  // no work - no Surface to check
  if(pSurface == NULL)
    { return FALSE ; }

  // locals
  ULONG lSing = pSurface->GetSingularities() ; // rtn: SM_SS_NONE or an orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX

  // no work - no Singularities to check
  if(lSing == SM_SS_NONE)
    { return FALSE ; }

  // locals
  ULONG ii, jj ;
  SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain() ;
  double     dTolDistU = sUVDomain.XLength() / 50.0 ;
  double     dTolDistV = sUVDomain.YLength() / 50.0 ;
  double     dUMin     = sUVDomain.GetUMin() ;
  double     dUMax     = sUVDomain.GetUMax() ;
  double     dVMin     = sUVDomain.GetVMin() ;
  double     dVMax     = sUVDomain.GetVMax() ;

  // BSpline Control Points
  SM_OBJ_ARRAY(sControlPoints,   SmPoint3d, 64) ;
  SM_OBJ_ARRAY(sControlUVPoints, SmPoint2d, 64) ;
  pUVTrimCurve->GetPolygon(pUVTrimCurve->GetNaturalInterval(), sControlPoints) ;
  ULONG lNum = sControlPoints.GetSize() ;
  sControlUVPoints.SetSize(lNum) ;
  for(ii=0;ii<lNum;ii++)
    { sControlUVPoints[ii] = sControlPoints[ii] ; } 
  sControlUVPoints.SetSize(lNum) ;
  for(ii=0;ii<lNum;ii++)
    { sControlUVPoints[ii] = sControlPoints[ii] ; } 

  // Special case curves built by the intersectors and DropCurve (expected to be the common case for UVTrimCurves)
  if(pUVTrimCurve->IsIntersectorApprox())
    {
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          pUVTrimCurve->Dump() ;
        }
#endif // SM_DEBUG_CODE

      // tunable locals
      double dPerpVecDeg     = 35.0 ; 
      double dParallelVecDeg = 45.0 ; 

      // see if start/end is on a singularity
      for(ii=0;ii<2;ii++)
        {
          // iter 0=Start Seg, 1=End Seg
          ULONG lIndx = ii == 0 ? 0 : lNum - 4 ;

          // beg/end ControlPoint
          SmPoint3d *pCPt = &sControlPoints[lIndx+ii*3] ;

          // check for control points near constant U Singularity boundaries
          SmBoolean bOnSingU = (   ((lSing & SM_SS_UMIN) && (smos_Fabs(pCPt->x - dUMin)) < dTolDistU)
                                || ((lSing & SM_SS_UMAX) && (smos_Fabs(pCPt->x - dUMax)) < dTolDistU)) ;

          SmBoolean bOnSingV = (   ((lSing & SM_SS_VMIN) && (smos_Fabs(pCPt->y - dVMin)) < dTolDistV) 
                                || ((lSing & SM_SS_VMAX) && (smos_Fabs(pCPt->y - dVMax)) < dTolDistV)) ;

          // when start/end UVTrimCurve segment starts/ends on Pole and has a dog-leg shape 
          if(  (bOnSingU || bOnSingV)
             && smgu_IsDogLeg(sControlUVPoints[lIndx+0], sControlUVPoints[lIndx+1],
                              sControlUVPoints[lIndx+2], sControlUVPoints[lIndx+3],
                              dPerpVecDeg, dParallelVecDeg))
            {
              // set optional output
              if(pOptIndx)
                {
                  *pOptIndx = lIndx + 1 ;
                }
              bHasDogLegNearPole = TRUE ;
              break ;

            } // end beg/end pt on pole and dog-leg shape check
        } // end iter beg/end Pts on UVTrimCurve
    } // end curve is an "intersectorApprox" branch
  else
    { // UVTrimCurve was not created by an intersector or dropCurve
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          pUVTrimCurve->Dump() ;
        }
#endif // SM_DEBUG_CODE

      // for every control point - see if its near the singularity boundary
      for(ii=0;ii<lNum && bHasDogLegNearPole==FALSE;ii++)
        {
          SmPoint3d sCPt = sControlPoints[ii] ;

          // check for control points near constant U Singularity boundaries
          SmBoolean bDoUCheck = (   ((lSing & SM_SS_UMIN) && (smos_Fabs(sCPt.x - dUMin)) < dTolDistU)
                                 || ((lSing & SM_SS_UMAX) && (smos_Fabs(sCPt.x - dUMax)) < dTolDistU)) ;

          SmBoolean bDoVCheck = (   ((lSing & SM_SS_VMIN) && (smos_Fabs(sCPt.y - dVMin)) < dTolDistV) 
                                 || ((lSing & SM_SS_VMAX) && (smos_Fabs(sCPt.y - dVMax)) < dTolDistV)) ;

          // when control point is near singularity boundary
          if(bDoUCheck || bDoVCheck)
            { 
              // for U and V checks
              for(jj=0;jj<2;jj++)
                {
                  SmVector3d sSingVec((jj+1)%2, jj, 0.0) ;  // jj==0 ? [1 0 0] : [0 1 0]

                  // skip check when control point is not near constant U poles
                  if(jj == 0 && bDoUCheck == FALSE) 
                    { continue ; }

                  // skip check when control point is not near constant V poles
                  if(jj == 1 && bDoVCheck == FALSE) 
                    { continue ; }

                  // when control point is an interior control point
                  if(ii != 0 && ii < lNum-1)
                    {
                      SmVector3d sLastVec = sCPt - sControlPoints[ii-1] ;
                      SmVector3d sNextVec = sControlPoints[ii+1] - sCPt ;
                      double     dAngVecRad, dAngLastPoleRad, dAngNextPoleRad ;

                      // Angle in Control Polygon at target ControlPoint
                      sLastVec.AngleBetween(sNextVec, dAngVecRad) ;

                      // Angles between ControlPolygon Next/Last Vecs and the singularity direction
                      sSingVec.Set(1.0, 0.0, 0.0) ;
                      sLastVec.AngleBetween(sSingVec, dAngLastPoleRad) ; 
                      sNextVec.AngleBetween(sSingVec, dAngNextPoleRad) ;

                      double dAngLastPoleDeg = SM_RAD2DEG(dAngLastPoleRad) ;
                      double dAngNextPoleDeg = SM_RAD2DEG(dAngNextPoleRad) ;

                      // when ControlPolygon turns and one CtrlPolygon segment runs along the singularity and one does not
                      if(   dAngVecRad > SM_DEG2RAD(50.0)                                          // When control polygon bends significantly
                         && (   (   (   SM_IS_CONTAINED(dAngLastPoleDeg,   0,  10)     // and (  ( LastVec nearly parallel to singular boundary
                                     || SM_IS_CONTAINED(dAngLastPoleDeg, 170, 180))
                                 && (   SM_IS_CONTAINED(dAngNextPoleDeg,  35, 145)))   //          and NextVec is far from parallel)
                             || (   (   SM_IS_CONTAINED(dAngNextPoleDeg,   0,  10)     //      or ( NextVec nearly parallel to singular boundary
                                     || SM_IS_CONTAINED(dAngNextPoleDeg, 170, 180))
                                 && (   SM_IS_CONTAINED(dAngLastPoleDeg,  35, 145))))) //           and NextVec is far from parallel))
                        {
                          // gwc: this predicate needs something to distinguish between 
                          //      turns and circles. A turn would
                          //      be a 3d curve that's supposed to project to an IsoParam curve that ends on the pole
                          //      but whose last few control points near the pole were given bad UV
                          //      points due to confused DropPoint results near the pole.
                          //      A circle in UV space can be represented by a square of control points
                          //      with 4 right angle turns in the control polygon.  Using SmCrvOnSurf
                          //      its legal to build a UVCircle and place one of the sides of the
                          //      of the circle controlPolygon square on a singularity boundary.
                          //      Currently this local test counts such a UVCircle on a singularity boundary
                          //      as having a dogLeg near a pole when it does not.
                          //      Today I don't see a way to make this test distinguish between the
                          //      two cases.  It's possible for 3d point evaluations at the turn to end up
                          //      having similar geometric properties, but many cases will be
                          //      distinguished ex: dropCurves and IntersectionCurves made as piecewise
                          //      bezier curves will be different than a rational BSpline circle or ellipse.
                          //      It's still possible to build confusing cases that will trick this
                          //      test into producing a false positive, e.g. a rational polynomial general
                          //      UV BSplineCurve that swings through the pole on a somewhat circular arc.

                          // call it a turn and quit
                          bHasDogLegNearPole = TRUE ;

                          // set output
                          if(pOptIndx) { *pOptIndx = ii ; }
                          break ; 

                          // The following idea did not work to distinguish between curves with turns at poles
                          //  and a circle UVCurve tangent to a surface pole.  The following code is no
                          //  good but I left it here as a source for better ideas.  good luck.
                          //      // Check UVTrimCurve tangents near the target ControlPoint to try and distinguish
                          //      //   between UVTrimCurve turns and UVCircles that become tangent to a singularity boundary.
                          //      SmVector3d sNextPV[2], sLastPV[2] ;
                          //      double dLastParam = pUVTrimCurve->GetGrevilleAbscissa(ii-1) ;
                          //      double dThisParam = pUVTrimCurve->GetGrevilleAbscissa(ii) ;
                          //      double dNextParam = pUVTrimCurve->GetGrevilleAbscissa(ii+1) ;
                          //      pUVTrimCurve->Evaluate(.8*dThisParam + .2*dNextParam, 1, TRUE,  sNextPV) ; 
                          //      pUVTrimCurve->Evaluate(.8*dThisParam + .2*dLastParam, 1, FALSE, sLastPV) ; 
                          //      double dAngUVRad ;
                          //      sLastPV[1].AngleBetween(sNextPV[1], dAngUVRad) ;
                          //
                          //      // when the pUVTrimCurve tangent changes significantly near the target control point
                          //      if(dAngUVRad > SM_DEG2RAD(50.0))
                          //        { 
                          //        } // end pUVTrimCurve tangent has a turn at ControlPoint check
                        } // end ControlPolygon has turn at ControlPoint check
                    } // end interior control point check 
                } // end iter constant U then constant V singularity directions
            } // end control point near constant U or V singularity check

        } // end iter every control point looking for those that sit on singularities
    } // end UVTrimCurve not created by Surf/Surf Intersector or DropCurve branch

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      pUVTrimCurve->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bHasDogLegNearPole) ;

} // end SmEdgeuse::HasDogLegNearPole

/*******************************************************************//**
PURPOSE: Evaluate the edgeuse in either UV space (Eval UVTrimCurve
   for a Surface UV Value) or 3D (Eval Curve for an XYZ value) using a
   normalized parameter [0.0->1.0].

NOTES:  Evaluate Edgeuse->Edge->Curve(Param) to get 3d points
                 Evaluate Edgeuse->UVTrimCurve(Param) to get UV points
***********************************************************************/
SmStatus SmEdgeuse::NormalizedEvaluate
 (double       dNormalizedParameter, // in : 0 to 1 target param value (accounts for orientation)
  SmBoolean    bParameterSpaceEval,  // in : TRUE = Eval Edgeuse->UVTrimCurve for a Surface UV Point
                                     //      FALSE= Eval Edgeuse->Edge->Curve for a 3d Point
  SmPoint3d  & rPoint,               // out: UV or XYZ value along Edgeuse for param (accounts for orientation)
  SmVector3d * pOptFirstDeriv,       // out: 1st UV or XYZ deriv along Edgeuse at param point (accounts for orientation),
                                     //      NULL to ignore, default:[NULL]
  SmBoolean    bOKToMakeUVTrimCurve) // in : default:[TRUE] = okay to build UVTrimCurves when missing
                                     //    : FALSE= Only use existing UVTrimCurves or work in 3d because UVTrimCurve Creation is dodgy during healing
 const
{
  // locals
  SmCurve * p3dOrUVCurve=NULL ;
  SmExtent1d sIvl;

  // when looking for parameter space evaluations
  //   let p3dOrUVCurve = UVTrimCurve
  if (bParameterSpaceEval)
    {
      // fetch or create UVTrimCurve
      double dMaxDist = 0.0, dMaxDev = 0.0;
      SmCurve * pUVTrimCurve = NULL ;
      SmStatus  eStat = ((SmEdgeuse *)this)->GetOrCreateUVTrimCurve( pUVTrimCurve, &dMaxDist, &dMaxDev, bOKToMakeUVTrimCurve) ;
      p3dOrUVCurve = pUVTrimCurve ;

      if ( pUVTrimCurve == NULL )
        {
          // Could do: If a uv trim curve cannot be created, then evalute in 3d
          // and drop to the surface.  And don't forget pOptFirstDeriv.
          if(bOKToMakeUVTrimCurve == TRUE)
            {
              WARN(_T("SmEdgeuse::NormalizedEvaluate() could not create uv trim curve.") );
            }

          // Evaluate in 3d, then drop to surface.
          SmPoint3d  s3dPoint;
          SmVector3d s3dDeriv;
          SmVector3d *pOpt3dDeriv = ( pOptFirstDeriv != NULL ) ? &s3dDeriv : NULL;
          SER( this->NormalizedEvaluate( dNormalizedParameter, FALSE, s3dPoint, pOpt3dDeriv)) ; // FALSE = Eval Curve for an XYZ evaluation

          SmSurface *pSurf = this->GetFace()->GetSurface();  NER( pSurf );
          SmExtent2d sDomain = pSurf->GetNaturalUVDomain();
          SmBoolean bOk;
          SmBoolean bIsMulti;
          SmPoint2d sUV;
          double dGap;
          SER( pSurf->DropPoint( s3dPoint, sDomain, NULL, bOk, sUV, dGap, bIsMulti ));
          if ( ! bOk )
            { SER( SM_ERR ); }

          rPoint.Set( sUV.x, sUV.y, 0 );
          if ( pOptFirstDeriv != NULL )
            {
              SmVector3d sS, sSu, sSv;
              SmVector2d sUVDeriv;
              pSurf->Evaluate1stDerivatives( sUV, FALSE, FALSE, sS, sSu, sSv );
              eStat = smgu_VecLinCombTwoVectors( sSu, sSv, s3dDeriv, sUVDeriv );
              if ( eStat == SM_SUCCESS )
                { pOptFirstDeriv->Set( sUVDeriv.x, sUVDeriv.y, 0 ); }
              else
                {
                  // Probably a zero vector.  Set that one to zero, the other to the dot product.
                  if ( sSu.Length() < sSv.Length() )
                    {
                      double dDot = s3dDeriv.Dot( sSv );
                      pOptFirstDeriv->Set( 0.0, dDot, 0.0 );
                    }
                  else
                    {
                      double dDot = s3dDeriv.Dot( sSu );
                      pOptFirstDeriv->Set( dDot, 0.0, 0.0 );
                    }
                }
            }

          // All done.
          return SM_SUCCESS;

        } // end GetOrCreateUVTrimCurve() failure

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
if ( bDebugMe ) 
  {
#ifdef SM_USE_NEWTOL
      SM_NEWTOL_LINE double dTol = ( this->GetEdge() ) ? this->GetEdge()-> GetLocalZoneTol3d() : SM_BIG_DOUBLE;
#else // SM_USE_OLDTOL
    SM_OLDTOL_LINE double dTol = SM_BIG_DOUBLE;
    SM_OLDTOL_LINE if (this->GetEdge()) 
    SM_OLDTOL_LINE   {
    SM_OLDTOL_LINE     dTol = (double)this->GetEdge()->GetTolerance();
    SM_OLDTOL_LINE   }
#endif // SM_USE_OLDTOL

      if ( dMaxDist > 100 * dTol  ||  dMaxDev > 10 * dTol )
        {
          SM_ASSERT_MSG(FALSE, _T("SmEdgeuse::NormalizedEvaluate(): Suspicious curve-drop results"));
        }
  }
#endif // SM_DEBUG_CODE

      sIvl = pUVTrimCurve->GetNaturalInterval();

    } // end bParameterSpaceEval branch

  else // xyzSpace - let p3dOrUVCurve = 3DCurve and sIvl = Edge domain.
    {
      SmEdge *pE = GetEdge();        NER(pE);
      p3dOrUVCurve = pE->GetCurve();   NER(p3dOrUVCurve);
      sIvl       = pE->GetInterval();

    } // end !bParameterSpaceEval branch

  // currently - p3dOrUVCurve is set to either 3DCurve or UVTrimCurve

  // invert parameter for opposite running Edgeuses
  double dOrientedParam = dNormalizedParameter;
  if (GetOrientation() == SM_OT_OPPOSITE) 
    {
      dOrientedParam = 1.0 - dOrientedParam;
    }

  // get requested evaluation
  if (pOptFirstDeriv == NULL)
    {
      SER(p3dOrUVCurve->EvaluatePoint(sIvl.Evaluate(dOrientedParam),rPoint));
    }
  else
    {
      SmVector3d sPV[2];
      SER(p3dOrUVCurve->Evaluate(sIvl.Evaluate(dOrientedParam),1,TRUE,sPV));
      rPoint = sPV[0];

      // invert tangents for opposite running Edgeuses
      if (GetOrientation() == SM_OT_OPPOSITE) 
        {
          sPV[1] = -sPV[1];
      }
      (*pOptFirstDeriv) = sPV[1];
    }

  return SM_SUCCESS;

} // end SmEdgeuse::NormalizedEvaluate

/*******************************************************************//**
PURPOSE: Evaluate the edgeuse inUV space (Eval Edge 3d Pt and Drop to Surf)
   using a normalized parameter [0.0->1.0].

NOTES:  Evaluate Edgeuse->Edge->Curve(Param) to get 3d points
                 pSurface->DropPoint
***********************************************************************/
SmStatus SmEdgeuse::NormalizedEvaluateAndDropPt
 (double       dNormalizedParameter, // in : 0 to 1 target param value (accounts for orientation)
  SmPoint2d  & rPointUV,             // out: UV value along Edgeuse for param (accounts for orientation)
  SmVector2d * pOptFirstDerivUV)     // out: 1st UV deriv along Edgeuse at param point (accounts for orientation),
                                     //      NULL to ignore, default:[NULL]
 const
{
  // locals
  SmEdge    * pEdge     = GetEdge() ;              NER(pEdge) ;
  SmCurve   * pCurve3d  = pEdge->GetCurve() ;      NER(pCurve3d) ;
  SmLoopuse * pLoopuse  = GetLoopuse() ;           NER(pLoopuse) ;
  SmFaceuse * pFaceuse  = pLoopuse->GetFaceuse() ; NER(pFaceuse) ; 
  SmFace    * pFace     = pFaceuse->GetFace() ;    NER(pFace) ;
  SmSurface * pSurface  = pFace->GetSurface() ;    NER(pSurface) ;
  SmExtent1d  sIvl      = pEdge->GetInterval() ; 
  SmExtent2d  sUVDomain = pSurface->GetNaturalUVDomain() ;
  SmVector3d  sPV[2], sGuessPt ;
  SmVector2d  sGuessUV, * pGuessUV = NULL ;
  SmVector2d  sUV[2] ;
  SmBoolean   bSuccess ;
  double      dDropGap3d ; 

  // don't create UVTrimCurves - but if they are available use them for guess points
  SmCurve * pUVTrimCurve = GetUVTrimCurve() ; 
          
  // invert parameter for opposite running Edgeuses
  double dOrientedParam = (GetOrientation() == SM_OT_SAME)
                             ? dNormalizedParameter
                             : 1.0 - dNormalizedParameter ;

  // when UVTrimCurve exists - use it to get a DropPt guess
  if(pUVTrimCurve)
    {
      pUVTrimCurve->EvaluatePoint(dOrientedParam, sGuessPt) ;
      sGuessUV = sGuessPt ; 
      pGuessUV = &sGuessUV ;
    }
    SmBoolean bIsMulti;
  // Point evaluation
  if (pOptFirstDerivUV == NULL)
    {
      SER(pCurve3d->EvaluatePoint(sIvl.Evaluate(dOrientedParam), sPV[0])) ;
      SER(pSurface->DropPoint(sPV[0], sUVDomain, pGuessUV, bSuccess, rPointUV, dDropGap3d, bIsMulti)) ;  
    }
  else // Point and 1stDeriv evaluate
    {
      SER(pCurve3d->Evaluate(sIvl.Evaluate(dOrientedParam),1,TRUE,sPV)) ;
      SER(pSurface->DropPoint(sPV[0], sUVDomain, pGuessUV, bSuccess, sUV[0], dDropGap3d, bIsMulti)) ;  
      SER(pSurface->DropVectors(sUV[0], TRUE, TRUE, 1, &sPV[1], &sUV[1])) ; 

      // invert tangents for opposite running Edgeuses
      if (GetOrientation() == SM_OT_OPPOSITE) 
        { sUV[1] = -sUV[1] ; }

      // set output
      rPointUV          = sUV[0] ;
      *pOptFirstDerivUV = sUV[1] ;
    } // end Point and 1stDeriv branch

  // all done
  return SM_SUCCESS ;

} // end SmEdgeuse::NormalizedEvaluateAndDropPt

/*******************************************************************//**
PURPOSE: Get EdgeParam for given NormalizedParam accounting for orienations

NOTES:  
***********************************************************************/
double SmEdgeuse::GetEdgeParam
 (double dNormalizedParameter) 
 const
{
  // locals
  SmEdge   * pE   = GetEdge(); NER(pE);
  SmExtent1d sIvl = pE->GetInterval();

  // invert parameter for opposite running Edgeuses
  if (GetOrientation() == SM_OT_OPPOSITE) 
    {
      dNormalizedParameter = 1.0 - dNormalizedParameter ;
    }

  return( sIvl.Evaluate(dNormalizedParameter) ) ;

} // end SmEdgeuse::GetEdgeParam

#ifdef SM_DEBUG_CODE
/*******************************************************************//**
PURPOSE: Static debugging routine.
***********************************************************************/
static void sm_DebugCheckExtents( const SmEdgeuse *pEU, SmCurve *pUVTrimCurve )
{
static constexpr SmBoolean sbSkipCheck = FALSE;
  if ( sbSkipCheck ) { return; }

  if ( pEU          == NULL ) { return; }
  if ( pUVTrimCurve == NULL ) { return; }

  SmExtent1d   sUVTrimIvl    = pUVTrimCurve->GetNaturalInterval() ;
  SmEdge     * pEdge         = pEU->GetEdge() ;
  SmCurve    * pEdgeCurve    = pEdge ? pEdge->GetCurve() : NULL ;
  SmExtent1d   sEdgeIvl      = pEdge ? pEdge->GetInterval() : sUVTrimIvl ; 
  SmExtent1d   sEdgeCurveIvl = pEdgeCurve ? pEdgeCurve->GetNaturalInterval() : sUVTrimIvl ;
  // Use the same tolerance that Trim() uses to decide not to trim.  [B484]
  //double       dScaledZero   = SmTol::GetScaledZero(sEdgeCurveIvl) ;
  double       dScaledZeroMin = 100 * SmTol::GetScaledZero(sEdgeCurveIvl.GetMin() );
  double       dScaledZeroMax = 100 * SmTol::GetScaledZero(sEdgeCurveIvl.GetMax() );
  if (   smos_Fabs( sUVTrimIvl.GetMin() - sEdgeIvl.GetMin() ) > dScaledZeroMin
      || smos_Fabs( sUVTrimIvl.GetMax() - sEdgeIvl.GetMax() ) > dScaledZeroMax )
    { 
      double dMinDist = smos_Fabs( sUVTrimIvl.GetMin() - sEdgeIvl.GetMin() );
      double dMaxDist = smos_Fabs( sUVTrimIvl.GetMax() - sEdgeIvl.GetMax() );
      TCHAR  sBuff[SM_TBLOCK_SIZE];
      smos_sprintf( sBuff, _T("SmEdgeuse::GetOrCreateUVTrimCurve - fetched UVTrimCurve->Ivl != Edge->Ivl. Dist of Mins:[%.16f], Dist of Maxs:[%.16f]" ), dMinDist, dMaxDist );
      SM_ASSERT_MSG( FALSE, sBuff); 
    }
  if ( sUVTrimIvl.GetMin() < sEdgeCurveIvl.GetMin() - dScaledZeroMin
       || sUVTrimIvl.GetMax() > sEdgeCurveIvl.GetMax() + dScaledZeroMax )
    {
      double dMinDist = sUVTrimIvl.GetMin() - sEdgeCurveIvl.GetMin();
      double dMaxDist = sEdgeCurveIvl.GetMax() - sUVTrimIvl.GetMax() ;
      dMinDist = ( dMinDist > 0 ) ? 0 : -dMinDist;
      dMaxDist = ( dMaxDist > 0 ) ? 0 : -dMaxDist;
      TCHAR  sBuff[SM_TBLOCK_SIZE];
      smos_sprintf( sBuff, _T( "SmEdgeuse::GetOrCreateUVTrimCurve - fetched UVTrimCurve->Ivl NotContainedBy Edge->Curve->Ivl. Dist outside Min:[%.16f], Dist outside Max:[%.16f]" ), dMinDist, dMaxDist );
      SM_ASSERT_MSG( FALSE, sBuff );
    }

  // Check Face/Surface domains.
  SmFace    * pFace = pEU->GetFace();
  SmSurface * pSurf = pFace ? pFace->GetSurface() : NULL;
  if ( pSurf )
    {
      SmExtent2d sSurfDom = pSurf->GetNaturalUVDomain();
      SmExtent2d sFaceDom = pFace->GetUVDomain();
      double dDomainScale = 100 * SmTol::GetScaledZero( sSurfDom );
      if ( ! sFaceDom.IsContainedBy( sSurfDom, dDomainScale ) )
        { SM_ASSERT_MSG( FALSE, _T("SmEdgeuse::GetOrCreateUVTrimCurve - Face domain bigger than its surface domain.")); }

// Remove Composites
//      // Also check owner pointers - expect them to be SmFace, SmCFace, or NULL
//      // BUT: These can be violated temporarily, such as in MakeFace()
//      // and MakeFaceTopology().  So use this only during specific debugging.
//static SmBoolean sbDoFacePtrsCheck = FALSE;
//
//      if ( sbDoFacePtrsCheck )
//        {
//          SmObject * pOwnerObj = pSurf->GetOwner();
//          SmFace   * pOwner    = SM_CAST_PTR( SmFace, pOwnerObj ) ;
//
//          // if pOwner is a CFace
//          if ( pOwner != NULL  &&  pOwner != pFace )
//            {
//              // Should be a CFace.
//              SmCFace *pCF = SM_CAST_PTR( SmCFace, pOwner );
//              if ( pCF == NULL )
//                { SM_ASSERT_MSG( FALSE, _T("cbi Bad pointers 1.")); }
//              else if ( pCF->GetSurface() != pSurf )
//                { SM_ASSERT_MSG( FALSE, _T("cbi Bad pointers 2.")); }
//            }
//        }
    } // end if pSurf exists

static constexpr SmBoolean sbDoContainmentTest = FALSE; // very expensive...
static constexpr double sdDistLimit = 0.25;

  if ( pSurf != NULL && sbDoContainmentTest )
    {
      SmExtent2d sSurfDom = pSurf->GetNaturalUVDomain();
      SmExtent2d sFaceDom = pFace->GetUVDomain();

      SmTArray< SmVertex* > sVerts;
      pFace->GetVertices( sVerts );
      SmVertex *pV;
      SmPoint3d sPt;
      SmBoolean bOk;
      SmPoint2d sUV;
      double sDist;
      SmBoolean bIsMulti;

      ULONG ii, lNumV = sVerts.GetSize();
      for ( ii=0; ii<lNumV; ii++ )
        {
          pV  = sVerts[ii];
          sPt = pV->GetPoint();
          SmStatus eStat = pSurf->DropPoint(sPt, sSurfDom, NULL, bOk, sUV, sDist, bIsMulti, SM_SO_MINIMIZE);
          if ( eStat != SM_SUCCESS || bOk != TRUE || sDist > sdDistLimit )
            { SM_ASSERT_MSG( FALSE, _T("SmEdgeuse::GetOrCreateUVTrimCurve : Face Vertices not on surf.")); }
        }
    }
} // end static sm_DebugCheckExtents
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Get or Create the UVTrimCurve of an edgeuse

RETURN: SmStatus SM_SUCCESS = Newly built or preexisting WellMade UVTrimCurve

NOTES: When Edgeuse has no attached UVTrimCurve, creates and attaches
       one before returning it.
***********************************************************************/
SmStatus SmEdgeuse::GetOrCreateUVTrimCurve
 (SmCurve *& rpUVTrimCurve,   // out: UVTrimCurve attached to Edgeuse/Mate pair
  double   * pdOptDistToSurf, // out: for New UVTrimCurve: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                              //      for Old UVTrimCurve: -1.0
                              //      NULL to ignore, default:[NULL]
  double   * pdOptDeviation,  // out: for New UVTrimCurve: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                              //      for Old UVTrimCurve: -1.0
                              //      NULL to ignore, default:[NULL]
  SmBoolean  bUVSpaceOkay)    // in : default:[TRUE] = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases)
                              //    : FALSE= don't use UVSpace because UVTrimCurves are not known to be valid
{
  // init outputs
  if(pdOptDistToSurf) { *pdOptDistToSurf = -1.0 ; }
  if(pdOptDeviation ) { *pdOptDeviation  = -1.0 ; }

  // get cached UVTrimCurve (if any)
  rpUVTrimCurve = GetUVTrimCurve();

  // no work - no UVTrimCurve and not allowed to build one for fear of DropCurve failure during healing - return NULL
  if(rpUVTrimCurve == NULL && bUVSpaceOkay == FALSE)
    {
      // all done
      return(SM_SUCCESS) ;
    }

  // low work - UVTrimCurve already exists
  if (rpUVTrimCurve)
    { 
      // prevent infinite loop - only ask for FaceEdge Gap if it already exists
      if(pdOptDistToSurf && IsMaxEdgeFaceGapInit()) 
        { 
          SmEdgeFaceGap * pMaxEdgeFaceGap = GetMaxEdgeFaceGap() ; 
          *pdOptDistToSurf                = pMaxEdgeFaceGap ? pMaxEdgeFaceGap->GetLength() : 0.0 ; 
        }

#ifdef SM_DEBUG_CODE
  sm_DebugCheckExtents( this, rpUVTrimCurve );
#endif // SM_DEBUG_CODE

      return( m_pDropCurveFail ? m_pDropCurveFail->m_sFailStatus : SM_SUCCESS) ; 
    } // end low work - UVTrimCurve already exists

  // arrive here to build UVTrimCurve

  // locals
  double           dMaxDistToSurf;
  // double         * pdMaxDevPtr = ( pdOptDeviation ) ? &dMaxDeviation : NULL;
  SmBSplineCurve * pNewUVCurve = NULL ;

  // create a new UVTrimCurve: err return value = CreateUVTrimCurve err return value
  SER( CreateUVTrimCurve( dMaxDistToSurf, pNewUVCurve, pdOptDeviation ));

  SM_ASSERT_BREAK(dMaxDistToSurf < 0.250);
  if ( pdOptDeviation != NULL )
    { SM_ASSERT_BREAK( *pdOptDeviation < 0.250 ); }

  // Save max edge to projected TrimCurve distances.
  // If one is Null and the other is non-Null, return the whole distance in the non-Null one.
  // (If pdOptDistToSurf is non-Null and pdOptDeviation is Null, CreateUVTrimCurve takes care of that.)
  if( pdOptDistToSurf )
    { *pdOptDistToSurf = dMaxDistToSurf; }
  else if ( pdOptDeviation )
    { *pdOptDeviation = smos_Sqrt(   dMaxDistToSurf *  dMaxDistToSurf
                                  + *pdOptDeviation * *pdOptDeviation ); }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugMe2 = FALSE ;
  if(bDebugMe2) 
    { SmBoolean bStream = SmSetAssertValidStream(TRUE) ;
      if (   !SM_DUMP_AND_ASSERT_VALID(pNewUVCurve) 
          || !SM_DUMP_AND_ASSERT_VALID(this))
        { bDebugMe = bDebugMe ? TRUE : FALSE ; } // debug breakpoint
      SmSetAssertValidStream(bStream) ;
    }

  sm_DebugCheckExtents( this, pNewUVCurve );

  if(bDebugMe)
    {
      sm_DebugCheckExtents( this, pNewUVCurve );

      SmBoolean bStream = SmSetAssertValidStream(TRUE) ;
      SM_DUMP_AND_ASSERT_VALID(pNewUVCurve) ;
      SmSetAssertValidStream(bStream) ;

      SmEdge     * pEdge         = this->GetEdge() ;
      SmCurve    * pEdgeCurve    = pEdge ? pEdge->GetCurve() : NULL ;
      SmFace     * pFace         = this->GetFace() ;
      SmSurface  * pFaceSurface  = pFace ? pFace->GetSurface() : NULL ;

      // get edgeuse Edge/UVTrimCurve and Edge/Surface Max gaps
      if(pEdgeCurve && pFaceSurface)
        {
          SmCrvOnSurf sCrvOnSurf( *pNewUVCurve, *pFaceSurface) ;
          SmExtent1d  sEdgeIvl   = pEdge->GetInterval() ;
#ifdef SM_USE_NEWTOL
          SM_NEWTOL_LINE SmZoneTol3d sZoneTol3d = pEdge ? pEdge->GetTolerance() : 0.0 ;
#else // SM_USE_OLDTOL
          SM_OLDTOL_LINE SmZoneTol3d sZoneTol3d = GetTolerance() ;
#endif // SM_USE_OLDTOL
          ULONG       lSampleCnt = 20 ;
          SmCrvCrvGapFunction sEdgeUVTrimCurveGF( (SmXSectTol3d)sZoneTol3d, pEdgeCurve, sEdgeIvl, &sCrvOnSurf,   lSampleCnt) ;
          SmCrvSrfGapFunction sEdgeFaceGF       ( (SmXSectTol3d)sZoneTol3d, pEdgeCurve, sEdgeIvl,  pFaceSurface, lSampleCnt) ; 

          // get max gap functions
          SmGapSample *pMaxEdgeFaceTrimCurveGap; 
          sEdgeUVTrimCurveGF.GetMaxGapSample(pMaxEdgeFaceTrimCurveGap);
          SmGapSample *pMaxEdgeFaceGap;
          sEdgeFaceGF.GetMaxGapSample(pMaxEdgeFaceGap);

          //double dEdgeFaceTrimCurveGap = sMaxEdgeFaceTrimCurveGap.GetLength() ;
          //double dEdgeFaceGap          = sMaxEdgeFaceGap.GetLength() ;
        }
    }
#endif // SM_DEBUG_CODE

  // Fix duplicate UVTrimCurve Problem
  //    In the process of creating a UV trim curve a trimmed
  //     surface cache will be retrieved or created for the
  //     face's surface to which this Edgeuse is attached.
  //    Constructing this trimmmed surface cache may create
  //     and attach another UVTrimCurve to this edgeuse
  //     rendering our new one redundant.

  // A call to SetUVTrimCurve will delete the duplicate UVTrimCurve (pass a copy of DropCurveFail when it exists)
  SmDropCurveFail sOldDropCurveFail, *pTgtDropFail = NULL, *pOldDropCurveFail = GetDropCurveFail() ;
  if(pOldDropCurveFail) { sOldDropCurveFail = *pOldDropCurveFail ;
                          pTgtDropFail      = &sOldDropCurveFail ;
                        }
  SetUVTrimCurve(pNewUVCurve, dMaxDistToSurf, TRUE, pTgtDropFail) ; // TRUE = delete existing UVTrimCurve

  // set output
  rpUVTrimCurve = GetUVTrimCurve();

#ifdef SM_DEBUG_CODE
  if ( rpUVTrimCurve == NULL )
    {
      SM_ASSERT_MSG( FALSE, _T("SmEdgeuse::GetOrCreateUVTrimCurve returned a NULL UVTrimCurve")) ;
    }
  else if(rpUVTrimCurve->GetDim() != 2)
    {
      SM_ASSERT_MSG(rpUVTrimCurve->GetDim() == 2, _T("SmEdgeuse::GetOrCreateUVTrimCurve returned a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // SmEdgeuse::GetOrCreateUVTrimCurve

/*******************************************************************//**
PURPOSE:  required after VisualStudio 4.0 because tighter
             conversion rules won't allow compiler to convert
             a (SmBSplineCurve *&) to a (SmCurve *&)

RETURN: SmStatus SM_SUCCESS = Newly built or preexisting WellMade UVTrimCurve
NOTES:
***********************************************************************/
SmStatus SmEdgeuse::GetOrCreateUVTrimCurve
 (SmBSplineCurve *& rpBSC,           // out: UVTrimCurve attached to Edgeuse mated pair
  double          * pdOptDistToSurf, // out: New UVTrimCurve: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                     //      Old UVTrimCurve: -1.0
                                     //      NULL to ignore, default:[NULL]
  double          * pdOptDeviation,  // out: New UVTrimCurve: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                     //      Old UVTrimCurve: -1.0
                                     //      NULL to ignore, default:[NULL]
  SmBoolean         bUVSpaceOkay)    // in : default:[TRUE] = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases)
                                     //    : FALSE= don't use UVSpace because UVTrimCurves are not known to be valid
{
  return GetOrCreateUVTrimCurve((SmCurve * &) rpBSC, pdOptDistToSurf, pdOptDeviation, bUVSpaceOkay);

} // end SmEdgeuse::GetOrCreateUVTrimCurve

/*******************************************************************//**
PURPOSE: Compute a UVTrimCurve and its UVTrimCurve/Curve tolerance 
  and attach those to this SmEdgeuse replacing any existing values.

NOTES:
***********************************************************************/
SmStatus SmEdgeuse::RebuildUVTrimCurve() 
{
  // locals
  double dMaxDistanceToSurface ;
  SmBSplineCurve *pNewUVTrimCurve ;

  // create new TrimCurve
  SER(CreateUVTrimCurve(dMaxDistanceToSurface, pNewUVTrimCurve)) ;

  // Attach to Edgeuse - replace existing data when needed
  SetUVTrimCurve(pNewUVTrimCurve, dMaxDistanceToSurface, TRUE) ; // TRUE = delete existing UVTrimCurve

  // all done
  return(SM_SUCCESS) ;

} // end SmEdgeuse::RebuildUVTrimCurve

/*******************************************************************//**
PURPOSE: Set edgeuse's UVTrimCurve and UVTrimCurve's Owner, 
         calc and cache max Edge/FaceTrimCurve gap, and
           if (GetBrep()->m_bValidateOnly == TRUE)
             and if needed, increase the edgeuse->edge's tolerance to 
                            2 times the given dMaxDistanceToSurface.
NOTES:
  1. One UVTrimCurve is stored for every pair of mated Edgeuses
     on the Edgeuse whose orientation is SM_OT_SAME in the 
     m_pUVTrimCurve pointer.
  2. On the Edgeuse whose orientation is SM_OT_OPPOSITE the
     m_pUVTrimCurve pointer is set to NULL.
  3. When an Edgeuse has no Mate, a UVTrimCurve is being temporarily 
     placed onto a standalone Edgeuse and its m_pUVTrimCurve pointer is set
     without regard to orientation.
  4. The max Edge/FaceTrimCurve gap is stored in m_sMaxEdgeFaceTrimCurveGap3d
***********************************************************************/
void SmEdgeuse::SetUVTrimCurve
 (SmBSplineCurve * pUVTrimCurve,              ///< [in] : UVTrimCurve to save on SM_OT_SAME directed mated-pair member, NULL okay
                                              ///<      : Ownership changes to SmEdgeuse, caller should not delete
  double           dMaxDistanceToSurface,     ///< [in] : set edgeuse->edge->Tolerance >= 2*dMaxDistanceToSurface, 0.0 to ignore
  SmBoolean        bDeleteOldUVTrimCurveFlag, ///< [in] : TRUE =delete any existing UVTrimCurve, FALSE=Don't, default:[FALSE]
  SmDropCurveFail *pOptDropCurveFail)         ///< [in] : DropCurve() data when UVTrimCurve was created, NULL to ignore, default:[NULL]
                                              ///<      : Makes a deep copy - owner needs to manage the memory for this object.
 {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pUVTrimCurve) ;

      sm_DebugCheckExtents( this, pUVTrimCurve );
    }

  sm_DebugCheckExtents( this, pUVTrimCurve );

  if(pUVTrimCurve != NULL && pUVTrimCurve->GetDim() != 2)
    {
      SM_ASSERT_MSG(pUVTrimCurve == NULL || pUVTrimCurve->GetDim() == 2, _T("SmEdgeuse::SetUVTrimCurve passed a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  // locals
  //SmEdge         * pE              = GetEdge() ;
  SmEdgeuse       * pMate             = GetMate() ;
  SmBSplineCurve  * pOldUVTrimCurve   = GetUVTrimCurve() ;
  SmDropCurveFail * pOldDropCurveFail = GetDropCurveFail() ;
  //SmFace         * pF              = GetFace() ;

  // no work - pUVTrimCurve already saved
  if(pOldUVTrimCurve == pUVTrimCurve)
    { // when given a DropCurveFail - udpated current DropCurveFail values
      if(pOptDropCurveFail) 
       { pOptDropCurveFail->m_pBrep = GetBrep() ; 
        if(pOldDropCurveFail) { *pOldDropCurveFail = *pOptDropCurveFail ; }
         else                  {  pOldDropCurveFail = new SmDropCurveFail(*pOptDropCurveFail) ; }
       }
      // else - leave any pOldDropCurveFail in place
      return ; 
    } // end no work - pUVTrimCurve already saved

  // delete oldUVTrimCurve when asked and we have one
  if(bDeleteOldUVTrimCurveFlag && pOldUVTrimCurve) { delete pOldUVTrimCurve ; pOldUVTrimCurve = NULL ; }

  // delete oldDropCurveFail
  if(pOldDropCurveFail) { delete pOldDropCurveFail ; pOldDropCurveFail = NULL ; }

  // clear old UVTrimCurve pointers 
  this->m_pUVTrimCurve = NULL ;
  this->m_pDropCurveFail = NULL ; 
  if(pMate) { pMate->m_pUVTrimCurve = NULL ;
              pMate->m_pDropCurveFail = NULL ; 
            }

  // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
  // this->m_sMaxEdgeFaceTrimCurveGap3d.ReSet() ;
  // if(pMate) { pMate->m_sMaxEdgeFaceTrimCurveGap3d.ReSet() ; }

  // A new UVTrimCurve can change a Vertexuse UV point. So clear the old VertexFaceGaps. [B678]
  // Note: SmVertexuse::ComputeUVPoint will use UVTrimCurves to refine the UVPoint, in particular getting
  //       the correct side of the seam.
  GetVertexuse()->InitVertexFaceGap();
  if ( pMate)
  { pMate->GetVertexuse()->InitVertexFaceGap(); }

  // if given a New UVTrimCurve - save it and update m_sEdgeFaceTrimCurveGap
  if(pUVTrimCurve)
    {
      // store pUVTrimCurve pointer on SM_OT_SAME oriented edgeuse
      if(pMate)
        {
          if(m_eOrientation == SM_OT_SAME)
            {
              SM_ASSERT(pMate->m_eOrientation  == SM_OT_OPPOSITE) ;
              this ->m_pUVTrimCurve = pUVTrimCurve ;
              pMate->m_pUVTrimCurve = NULL ;
              pUVTrimCurve->SetOwner((SmEdgeuse *)this) ;
              if(pOptDropCurveFail) { if(this->m_pDropCurveFail) { *this->m_pDropCurveFail = *pOptDropCurveFail ; }
                                      else                       {  this->m_pDropCurveFail = new SmDropCurveFail(*pOptDropCurveFail) ; }
                                    }
              else                  { if(this->m_pDropCurveFail) {  delete this->m_pDropCurveFail ; this->m_pDropCurveFail = NULL ; }
                                    }
              pMate->m_pDropCurveFail = NULL ; 
            }
          else
            {
              SM_ASSERT(this ->m_eOrientation  == SM_OT_OPPOSITE) ;
              SM_ASSERT(pMate->m_eOrientation  == SM_OT_SAME) ;
              this ->m_pUVTrimCurve = NULL ;
              pMate->m_pUVTrimCurve = pUVTrimCurve ;
              pUVTrimCurve->SetOwner(pMate) ;
              if(pOptDropCurveFail) { if(pMate->m_pDropCurveFail) { *pMate->m_pDropCurveFail = *pOptDropCurveFail ; }
                                      else                        {  pMate->m_pDropCurveFail = new SmDropCurveFail(*pOptDropCurveFail) ; }
                                    }
              else                  { if(pMate->m_pDropCurveFail) {  delete pMate->m_pDropCurveFail ; pMate->m_pDropCurveFail = NULL ; }
                                    }
              this->m_pDropCurveFail = NULL ; 
            }
        } // end has mate check
      else // a UVTrimCurve is being temporarily placed onto a standalone Edgeuse
        {
          //      SM_ASSERT(m_eOrientation == SM_OT_SAME) ;
          this->m_pUVTrimCurve = pUVTrimCurve ;
          pUVTrimCurve->SetOwner((SmEdgeuse*)this) ;
          if(pOptDropCurveFail) { if(this->m_pDropCurveFail) { *this->m_pDropCurveFail = *pOptDropCurveFail ; }
                                  else                       {  this->m_pDropCurveFail = new SmDropCurveFail(*pOptDropCurveFail) ; }
                                }
          else                  { if(this->m_pDropCurveFail) {  delete this->m_pDropCurveFail ; this->m_pDropCurveFail = NULL ; }
                                }
        } // end has no mate check

      // gwc: The gaps are set up for lazy evaluation - no need to set them up here.
      //      In fact, since EdgeFaceTrimCurve gaps require UVTrimCurves, updating
      //      here leads to some convoluted infinite loops.  So, only clear
      //      the EdgeFaceTrimCurveGap when a new UVTrimCurve is saved - that's already done.
      //      so remove the following block of code

      // gwc: remove block
      // // next: update cached gaps: m_sMaxEdgeFaceTrimCurveGap3d, m_sMaxEdgeFaceGap3d
      // 
      // // When Edgeuse does not represent a Face/Edge connection
      // if(!pF || !pE)
      //   {
      //     // also clear the Edge/Face gap
      //     m_sMaxEdgeFaceGap3d.ReSet() ;
      //     if(pMate) { pMate->m_sMaxEdgeFaceGap3d.ReSet() ; }
      //   } 
      // else // find and cache the new max Edge/FaceTrimCurve gap
      //   {
      //     CalcMaxEdgeFaceTrimCurveGap(NULL,TRUE, pUVTrimCurve) ;
      // 
      //   //  // locals
      //   //  SmSurface    * pSurface    = pF->GetSurface();
      //   //  SmCurve      * pEdgeCurve  = pE->GetCurve();
      //   //  // SmXSectTol3d   sXSectTol3d = SmTol::GetXSectTol3d(pE, GetFace()) ;  // gwc: infinite loop
      //   //  SmXSectTol3d   sXSectTol3d = SmTol::GetXSectTol3d(*(pF->GetContext() ? pF->GetContext() : pE->GetContext())) ; 
      //   //
      //   //  // build projection of UVTrimCurve through pSurface into 3d Space
      //   //  SmCrvOnSurf sCrvOnSurf(*pUVTrimCurve,*pSurface, NULL, 0, GetContext());
      //   //
      //   //  // construct gap function
      //   //  ULONG               lSampleCnt = 20 ;
      //   //  SmExtent1d          sInterval = pE->GetInterval();
      //   //  SmCrvCrvGapFunction sEdgeFaceTrimCurveGF( sXSectTol3d.val, pEdgeCurve, sInterval, &sCrvOnSurf, lSampleCnt) ;
      //   //
      //   //  // get max gap
      //   //  SmGapSample sMaxEdgeFaceTrimCurveGap = sEdgeFaceTrimCurveGF.GetMaxGapSample() ;
      //   //
      //   //  // Set EdgeFaceTrimCurve gap
      //   //  m_sMaxEdgeFaceTrimCurveGap3d.Set(this, 
      //   //                                sMaxEdgeFaceTrimCurveGap.GetThisParam().x, 
      //   //                                sMaxEdgeFaceTrimCurveGap.GetOtherParam().x) ;
      // 
      //     // keep Mate gap current
      //     if(pMate) { pMate->m_sMaxEdgeFaceTrimCurveGap3d = m_sMaxEdgeFaceTrimCurveGap3d ; }
      //
      //     // when needed also compute the EdgeFace Gap
      //     if(m_sMaxEdgeFaceGap3d.IsInit() == FALSE)
      //       {
      //         CalcMaxEdgeFaceGap(NULL, TRUE) ;
      // 
      //        //  // construct gap function
      //        //  SmCrvSrfGapFunction sEdgeFaceGF( sXSectTol3d.val, pEdgeCurve, sInterval, pSurface, lSampleCnt) ;
      //        // 
      //        //  // get max gap
      //        //  SmGapSample sMaxEdgeFaceGap = sEdgeFaceGF.GetMaxGapSample() ;
      //        // 
      //        //  // Set EdgeFaceTrimCurve gap
      //        //  m_sMaxEdgeFaceGap3d.Set( this, 
      //        //                        sMaxEdgeFaceGap.GetThisParam().x, 
      //        //                       &sMaxEdgeFaceGap.GetOtherParam()) ;
      //        // 
      //         // keep Mate gap current
      //         if(pMate) { pMate->m_sMaxEdgeFaceGap3d = m_sMaxEdgeFaceGap3d ; }
      //
      //       } // end need to find and cache the max Edge/Face gap block 
      // 
      //     // if SmVertexuse ever caches the Vertex/EdgeFaceTrimCurve gap
      //     //   modify the following sample block of code appropriately
      //     //   to update the vertex/EdgeFaceTrimCurve cached gap when UVTrimCurve changes
      // 
      //     //  SmVertexuse * pVU = GetVertexuse() ;
      //     //  if(pVU && pUVTrimCurve)
      //     //    {
      //     //      SmVertexFaceTrimCurveGap sVertexFaceFrimCurveGap ;
      //     //      // the important part - pass the pUVTrimCurve to prevent infinite loop
      //     //      sVertexFaceFrimCurveGap = pVU->GetVertexFaceTrimCurveGap(pUVTrimCurve) ;
      //     //    }
      // 
      //   } // end find and cache the new max Edge/FaceTrimCurve gap block
      // gwc: end removed block of code

      // I don't like doubling this every time, I think something
      // just slightly bigger than 1.0 would be more appropriate.
      // Note, using 1.01 has little effect on prog_test, nothing bad.
      // We'll straighten it all out in the Tol project.
      static constexpr double sdTolFactor = 2.0;

      // if only validating, then do not allow tolerances to change
      if( this->GetBrep()->m_bValidateOnly == FALSE )
        {
          double dCurrTol = GetEdge()->GetTolerance();
          if (dCurrTol < sdTolFactor * dMaxDistanceToSurface)
            {

#ifdef SM_USE_OLDTOL
              SM_OLDTOL_LINE GetEdge()->SetTolerance( sdTolFactor * dMaxDistanceToSurface);
#endif // SM_USE_OLDTOL

            }
        } // end if not validating only (may modify)
    } // end need to set new UVTrimCurve value

#ifdef SM_DEBUG_CODE
  if (pUVTrimCurve && bDebugMe)
    {
      SM_ASSERT_VALID( this ) ;
      SmEdge *pE = GetEdge();
      SmFace *pF = GetFace();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if ( pF ) pF->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); if ( pE ) pE->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,0,1); this->Draw(); sm_GraphicsLoop();

      SmFaceuse *pFU = GetLoopuse()->GetFaceuse();
      SmTArray<SmEdgeuse*> sEdgeuses;
      pFU->GetEdgeuses(sEdgeuses);

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0); pUVTrimCurve->Draw(); sm_GraphicsLoop();
      for (ULONG i=0; i<sEdgeuses.GetSize(); i++)
        {
          SmEdgeuse      *pEU    = sEdgeuses[i];
          SmBSplineCurve *pUVCrv = pEU->GetUVTrimCurve();
          if (pUVCrv) { pUVCrv->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

} // end SmEdgeuse::SetUVTrimCurve

/*******************************************************************//**
PURPOSE: Set this Edgeuse->m_pUVTrimCurve to pUVCurve when pUVCurve
  is good enough, else does nothing.

NOTES: 1. Good enough - MaxDist(Surf(UVCurve), Edge->Curve) < Brep->Tol * 100.
                        or MaxDist < Edge->Length/10.0
       2. When updating tolerances, before updating an Edge tolerance
          it drops Edge->Curve back to Surface to see if it can
          find a better UVTrimCurve than the input UVCurve.
          It saves the better UVCurve as the UVTrimCurve and
          updates the Edge tolerance as needed.
***********************************************************************/
SmStatus SmEdgeuse::CheckAndSetUVTrimCurve
 (SmBSplineCurve  * pUVTrimCurve,       // in : target UVTrimCurve to check
  SmBoolean         bAdjustTolerances,  // in : TRUE = Adjust Edgeuse->Edge->Tol >= 2 MaxDistToSurf
                                        //      FALSE= don't
  double          & rdMaxDistToSurf,    // out: MaxDist between Surf(UVCurve) and Edge
  SmBoolean       & rbTrimReplaced,     // out: TRUE = pUVTrimCurve attaced to Edgeuse as UVTrimCurve, mem managed by Edgeuse
                                        //      FALSE= pUVTrimCurve no good, no changes made, pUVTrimCurve managed by caller
  SmDropCurveFail * pOptDropCurveFail)  ///< [in] : DropCurve() data when UVTrimCurve was created, NULL to ignore, default:[NULL]
                                        ///<      : Makes a deep copy - owner needs to manage the memory for this object.

{
  // init output
  rbTrimReplaced = FALSE;

  // locals
  SmEdgeuse      * pEU             = (SmEdgeuse*) this;
  SmEdgeuse      * pTrimOwner=NULL, * pDropOwner=NULL ; 
  SmBSplineCurve * pOldUVTrimCurve = GetUVTrimCurve (&pTrimOwner);
  SmSurface      * pSurface        = GetFace()->GetSurface();
  SmEdge         * pE              = GetEdge();
  SmCurve        * pEdgeCurve      = pE->GetCurve() ;
  double           dMaxDistToSurf;
  SmDropCurveFail  sOldDropCurveFail, *pOldDropCurveFail = GetDropCurveFail(&pDropOwner) ;
  SM_ASSERT_MSG(   (pOldUVTrimCurve == NULL || pOldDropCurveFail == NULL)
                || (pTrimOwner == pDropOwner),
                _T("SmEdgeuse::CheckAndSetUVTrimCurve error - something wrong, the UVTrimCurve and DropCurveFail objs have different owners")) ; 

  // save a copy of OldDropCurveFail data
  if(pOldDropCurveFail) { sOldDropCurveFail = *pOldDropCurveFail ; 
                          sOldDropCurveFail.m_pBrep = GetBrep() ;
                        }

  // attach pUVTrimCurve to this Edgeuse
  SetUVTrimCurve(pUVTrimCurve,0.0,FALSE,pOptDropCurveFail) ; // FALSE = don't delete existing UVTrimCurve

  // define the 3D curve for the UVCurve projected through the Surface
  SmCrvOnSurf s3DMap(*pUVTrimCurve,*pSurface, NULL, 0, GetContext());

  // find approx max dist between edge and 3d Projection of UVCurve through the Surface using 8 sample points
  SmBoolean bIncorrectDir;
  SmStatus sRtn = pEdgeCurve->CurveMaxDistanceBetween
             (pE->GetInterval(),                    // in : interval limit for this curve
              s3DMap,                               // in : other curve to test
              s3DMap.GetNaturalInterval().GetMin(), // in : OtherCurve param mapping to ThisCurve Interval.Min value
              s3DMap.GetNaturalInterval().GetMax(), // in : OtherCurve param mapping to ThisCurve Interval.Max value
              8,                                    // in : Min number of samples to take - it measures at least this many points
                                                    //      at a uniform spacing on the otherCurve finding the
                                                    //      corresponding ThisCurve closest points.
                                                    //      If 0 is given it will do its best to perform
                                                    //      a precise measurement and will be much slower.
              NULL,                                 // in : Max allowed gap.  Quit searching once this value is exceeded.
                                                    //      NULL to ignore.   Never quit search when NULL.
              dMaxDistToSurf,                       // out: Set to max gap size seen.
                                                    //        a. When lNumSamples == 0 This is the max curve/curve gap.
                                                    //        b. When pdOptMaxDistanceNeeded this is either the max sampled
                                                    //           curve/curve gap which is less than pdOptMaxDistanceNeeded or
                                                    //           the 1st gap seen larger than pdOptMaxDistanceNeeded.
              NULL,                                 // out: Curve param for returned MaxDistance Found, NULL to ignore.
                                                    //      default:[NULL]
              NULL,                                 // out: OtherCurve param for returned MaxDistance Found, NULL to ignore.
                                                    //      default:[NULL]
              &bIncorrectDir);                      // out: corresponding point found in incorrect order
                                                    //      default:[NULL]
  // when MaxDist was computed without hiccups
  if (sRtn == SM_SUCCESS && !bIncorrectDir)
    {
      // remember when EdgeCurve/UVTrimcurve distance is very close 
      SmBoolean bPreciseCurve =  (dMaxDistToSurf < pE->GetBrep()->GetTolerance() / 1000.00)
                              ? TRUE
                              : FALSE;

      // when the two curves are kind of close - mark the bUVGoodEnough bit as TRUE
      SmBoolean bUVGoodEnough =   (dMaxDistToSurf < pE->GetBrep()->GetTolerance()*100.0)
                              ? TRUE
                              : FALSE;

      // give bad curves one last chance to be ok
      if(!bUVGoodEnough)
        {
          // check approx pEdgeCurve length with 10 sample points
          double dEdgeLength = pEdgeCurve->ApproximateLength(pE->GetInterval(),10) ;

          // if max dist between curves is near the edge (based on edge length) - mark bUVGoodEnough
          if (dMaxDistToSurf < dEdgeLength/10.0)
            {
              bUVGoodEnough = TRUE;
            }
        } // end GoodEnough Checks

      // when this UVCurve is GoodEnough and the curve is not Precise
      if (bUVGoodEnough && !bPreciseCurve)
        {
          // find accurate max dist between edge and projected UVCurve
          SER(pEdgeCurve->CurveMaxDistanceBetween(pE->GetInterval(),
                                                  s3DMap,
                                                  s3DMap.GetNaturalInterval().GetMin(),
                                                  s3DMap.GetNaturalInterval().GetMax(),
                                                  0,
                                                  NULL,
                                                  dMaxDistToSurf,
                                                  NULL,
                                                  NULL,
                                                  &bIncorrectDir));
          if (bIncorrectDir) { bUVGoodEnough = FALSE; }
        }

      // When UVTrimCurve is good enough
      if (bUVGoodEnough)
        {
          // save the average and the max dist values for output
          rdMaxDistToSurf = dMaxDistToSurf;

          // when adjusting edge and face tolerances
          if (bAdjustTolerances)
            {
              double dCurrTol = pE->GetTolerance();

              // when edge tolerance is less than 2*max dist between curves
              if (dCurrTol < 2.0 * dMaxDistToSurf)
                {
                  // Try reprojecting to get a better UV curve
                  SmBSplineCurve *pTmpUVCrv = NULL ;
                  double dTmpMaxDistToSurf;

                  // create a new UV curve
                  SER(pEU->CreateUVTrimCurve(dTmpMaxDistToSurf,pTmpUVCrv));
                  SmObjDelete sCleanTmpUV(pTmpUVCrv);
                  if (dMaxDistToSurf > dTmpMaxDistToSurf * 2.0)
                    {
                      rdMaxDistToSurf = dMaxDistToSurf;
                      if (!bAdjustTolerances) dTmpMaxDistToSurf = 0.0;
                      sCleanTmpUV.Clear();
                      pEU->SetUVTrimCurve(pTmpUVCrv,dTmpMaxDistToSurf,TRUE) ; // TRUE = delete existing UVTrimCurve
                    }
                  else
                    {
#ifdef SM_USE_OLDTOL
                      SM_OLDTOL_LINE pE->SetTolerance(2.0*dMaxDistToSurf);
#endif // SM_USE_OLDTOL
                    }
                } // end current edge tolerance is less than this UVcurve tolerance check
            } // end adjusting tolerances check

          // done with this edgeuse - it has a good enough UVTrimCurve
          rbTrimReplaced = TRUE;
        } // end found UVTrimCurve is goodEnough check
    } // end Calc MaxDist without a hiccup check

  // when UVTrimCurve was replaced - delete OldUVTrimCurve
  if (rbTrimReplaced)
    { if(pOldUVTrimCurve) {delete pOldUVTrimCurve; pOldUVTrimCurve = NULL ; }}
  else // restore OldUVTrimCurve
    { // UV Curve was not very accurate - restore the old one
      SmDropCurveFail *pTgtDrop = pOldDropCurveFail ? &sOldDropCurveFail : NULL ; 
      pEU->SetUVTrimCurve(pOldUVTrimCurve,0.0,TRUE,pTgtDrop) ; // TRUE = delete existing UVTrimCurve
    } 

  return sRtn;

} // end SmEdgeuse::CheckAndSetUVTrimCurve

/*******************************************************************//**
PURPOSE: find and return max gap between Edgeuse->Edge->Curve and Face->Surface
         when Edge is connected Face, else returns NULL

NOTES: MaxFaceEdgeGap is Cached 
        - after a 1st expensive call - it's cheap to get.

TODO: extend SmSurface::GlobalCurveSolve to find the max gap between a curve and a surface
      extend SmGlobal::GlobalCurveSolve to find the max gap between a curve and a curve
      Use SmSurface::GlobalCurveSolve to set rEdgeuseGap
***********************************************************************/
SmEdgeFaceGap * SmEdgeuse::GetMaxEdgeFaceGap // rtn: ref to stored max gap between Edge->Curve and Face->Surface
 (SmBoolean     bForceCalc)                  // in : TRUE = force gap Calc, FALSE=use cache if available else calc
 const                                       //      default:[FALSE] 
{
  // no work - no Gap
  if(HasEdgeFaceGap() == FALSE)
    { return NULL ; }
 
  // when asked - refresh the m_sMaxEdgeFaceTrimCurveGap3d cache
  if( bForceCalc || !m_sMaxEdgeFaceGap3d.IsInit())
    {
      // refresh the Vertex/Edge gap cache
      CalcMaxEdgeFaceGap(NULL, TRUE) ;
    }

  // all done
  return(&m_sMaxEdgeFaceGap3d ) ;

} // end SmEdgeuse::GetMaxEdgeFaceGap

/*******************************************************************//**
PURPOSE: Find and return Gap between Edgeuse->Vertexuse->StartVertex and
         Edge->StartPoint when StartVertex is connected to Edge,
            else returns NULL.

NOTES:
***********************************************************************/
SmVertexEdgeGap * SmEdgeuse::GetVertexEdgeGap // rtn: gap between edgeuse->edgeStartPoint/edgeuse->StartVertexPoint
 (SmBoolean bForceCalc)                       // in : TRUE = force gap evaluation, FALSE=use cache if available
                                              //      default:[FALSE]
 const
{
  // Start Vertexuse
  SmVertexuse *pVertexuse = GetVertexuse() ;

  // fetch and return StartVertexuse/Edge gap
  return(pVertexuse->GetVertexEdgeGap(bForceCalc)) ;

} // end SmEdgeuse::GetVertexEdgeGap

/*******************************************************************//**
PURPOSE: find and return max gap between 
         Edgeuse->Edge->Curve->EndPt and CCWEdgeuse->Edge->Curve->StartPt

NOTES: EdgeCCWEdgeGap is Cached - after a 1st expensive call - it's cheap to get.
***********************************************************************/
SmEdgeEdgeGap * SmEdgeuse::GetEdgeCCWEdgeGap // rtn: stored gap between Edgeuse->Curve->EndPt and CCWEdgeuse->Curve->StartPt
 (SmBoolean bForceCalc)                      // in : TRUE = force gap cals, FALSE=use cache if available, else calc
 const                                       //      default:[FALSE] 
{
  // no work - no gap
  if(HasEdgeCCWEdgeGap() == FALSE)
    { return(NULL) ; }

  // when asked - refresh the m_sEdgeCCWEdgeTrimCurveGap3d cache
  if( bForceCalc || !m_sEdgeCCWEdgeGap3d.IsInit())
    {
      // refresh the Vertex/Edge gap cache
      CalcEdgeCCWEdgeGap(NULL, TRUE) ;
    }

  // all done
  return(&m_sEdgeCCWEdgeGap3d ) ;

} // end SmEdgeuse::GetEdgeCCWEdgeGap

/*******************************************************************//**
PURPOSE: find and return max gap between 
         Edgeuse->Edge->Curve->StartPt and CWEdgeuse->Edge->Curve->EndPt

NOTES: EdgeCWEdgeGap is Cached on the CWEdgeuse 
        - after a 1st expensive call - it's cheap to get.
***********************************************************************/
SmEdgeEdgeGap * SmEdgeuse::GetEdgeCWEdgeGap  // rtn: stored gap between Edgeuse->Curve->StartPt and CCWEdgeuse->Curve->EndPt
 (SmBoolean bForceCalc)                      // NotUsed: in : TRUE = force gap calc, FALSE=use cache if available, else calc
 const                                       //      default:[FALSE] 
{
  SM_REF1(bForceCalc) ;
  // get neighbor
  SmEdgeuse * pCWEdgeuse = GetCWEdgeuse() ;

  // pass the call along - check for no CWEdgeuse neighbor (Edgeuse not in a loop)
  return( pCWEdgeuse ? pCWEdgeuse->GetEdgeCCWEdgeGap() : NULL) ;

} // end SmEdgeuse::GetEdgeCWEdgeGap

// /*******************************************************************//**
// PURPOSE: find and return max gap between Edgeuse->Edge->Curve and Edgeuse->UVTrimCurve projection.
// 
// NOTES: MaxFaceTrimCurveGap is not cached - get this gap is expensive for every call.
// 
//  TODO: extend SmSurface::GlobalCurveSolve to find the max gap between a curve and a surface
//        extend SmGlobal::GlobalCurveSolve to find the max gap between a curve and a curve
//        Use SmSurface::GlobalCurveSolve to set rEdgeuseGap
// ***********************************************************************/
// SmEdgeFaceTrimCurveGap SmEdgeuse::GetMaxEdgeFaceTrimCurveGap 
//  () 
//  const
// {
//   SmEdgeFaceTrimCurveGap sMaxEdgeFaceTrimCurveGap ;
// 
//   // calc the EdgeTrimCurve/Face gap
//   CalcMaxEdgeFaceTrimCurveGap(sMaxEdgeFaceTrimCurveGap) ;
// 
//   // all done
//   return(sMaxEdgeFaceTrimCurveGap) ;
// 
// } // end SmEdgeuse::GetMaxEdgeFaceTrimCurveGap
// 
// /*******************************************************************//**
// PURPOSE: Find and return Gap between Edgeuse->Vertexuse->StartVertex and
//             associated edgeTrimCurve->StartPoint
// 
// RETURNS ---
// ***********************************************************************/
// SmVertexFaceTrimCurveGap  SmEdgeuse::GetVertexFaceTrimCurveGap // rtn: gap between edgeuse->UVTrimCurve->StartPt/edgeuse->StartVertexPoint 
//  ()
//  const
// {
//   // locals
//   SmVertexuse *pVertexuse = GetVertexuse() ;
// 
//   // fetch and return Vertexuse gap
//   return( pVertexuse->GetVertexFaceTrimCurveGap()) ; 
// 
// } // end SmEdgeuse::GetVertexFaceTrimCurveGap

/*******************************************************************//**
PURPOSE: Find and return Gap between Edgeuse->Vertexuse->EndVertex and
            associated edge->EndPoint when EdgeEnd connects to Vertex,
            else return NULL ;

NOTES: When OtherVertex does not exist - Signal Assert, return NULL
***********************************************************************/
SmVertexEdgeGap * SmEdgeuse::GetOtherVertexEdgeGap 
 (SmBoolean bForceCalc)  // in : TRUE = force gap evaluation, FALSE=use cache if available
                         //      default:[FALSE] 
 const
{
  // locals
  SmEdgeuse   * pMate           = GetMate() ;
  SmVertexuse * pOtherVertexuse = pMate ? pMate->GetVertexuse() : NULL ;

  // error - no OtherVertexuse
  SM_ASSERT_MSG_BREAK(pOtherVertexuse != NULL,
                      _T("SmEdgeuse::GetOtherVertexEdgeGap - Edgeuse->OtherVertexuse does not exist - returning NULL")) ;
  if(pOtherVertexuse == NULL)
    { return NULL ; }

  // fetch Vertexuse gap
  return( pOtherVertexuse->GetVertexEdgeGap(bForceCalc) ) ;

} // end SmEdgeuse::GetOtherVertexEdgeGap

/*******************************************************************//**
PURPOSE: Old model GetTolerance() - 

NOTES: Edgeuse ZoneTol3d inherited from its edge when owned, else
       SM_ZONE_TOL_3D
***********************************************************************/
SmZoneTol3d SmEdgeuse::GetTolerance()
 const
{
  return(GetEdge() ? GetEdge()->GetTolerance() : ((SmZoneTol3d)SM_ZONE_TOL_3D)) ;

} // end SmEdgeuse::GetTolerance

/*******************************************************************//**
PURPOSE: compute max Edge/FaceTrimCurve gap from 
         Edge->Curve to Face->Surface(UVTrimCurve)

NOTES: 1. computes and caches UVTrimCurve when uninitialized
       2. when edgeuse does not connect a curve to a surface, sets gaps to uninit
       3. if Edgeuse has no trim curve, return success (do not create one)
***********************************************************************/
SmStatus SmEdgeuse::CalcMaxEdgeFaceTrimCurveGap
 (SmEdgeFaceTrimCurveGap & rMaxEdgeFaceTrimCurveGap, // out: set to calculated Edge/FaceTrimCurve gap, NULL to ignore, default:[NULL]
  SmBSplineCurve         * pOptUVTrimCurve)          // in : for internal use only - stops infinite loop
 const                                               //      default:[NULL]
{
  // init output
  rMaxEdgeFaceTrimCurveGap.ReSet() ; 

  // Get Mate (Mates share the same gaps)
  // SmEdgeuse *pMate = GetMate() ;

  // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
  // // when asked - clear cache
  // if(bCache)
  //   { m_sMaxEdgeFaceTrimCurveGap3d.ReSet() ;
  //     if(pMate) 
  //       { pMate->m_sMaxEdgeFaceTrimCurveGap3d.ReSet() ; } 
  //   }

  // no work - no EdgeFaceTrimCurve Gap
  if(!HasEdgeFaceTrimCurveGap())
    { return( SM_SUCCESS ) ; }

  // check state
  SM_ASSERT_MSG(   GetFace()               != NULL
                && GetFace()->GetSurface() != NULL
                && GetEdge()               != NULL
                && GetEdge()->GetCurve()   != NULL,
                _T("SmEdgeuse::CalcMaxEdgeFaceTrimCurveGap: Edgeuse use not connected to a Curve and a Surface")) ;

  // Locals for Edge/UVTrimCurve gap calc
  SmEdge          * pEdge          = GetEdge() ;
  SmFace          * pFace          = GetFace() ;
  SmCurve         * pEdgeCurve     = pEdge->GetCurve();
  SmSurface       * pSurface       = pFace->GetSurface();
  SmBSplineCurve  * pUVCurve       = pOptUVTrimCurve ;
  const SmContext * pContext       =   GetContext() ? GetContext()
                                     : pEdge->GetContext() ? pEdge->GetContext()
                                     : pFace->GetContext() ? pFace->GetContext()
                                     : NULL ;

  // gwc: avoid infinite loop - don't call SmTol::GetXSectTol3d(pEdge,pFace) which comes back to this function
  SmXSectTol3d      sXSectTol3d    = SmTol::GetXSectTol3d(pContext) ;

  // When not given a UVTrimCurve
  if(pUVCurve == NULL)
    {
      // Retrieve UVTrimCurve.  If not there, don't try to create,
      // the topology might not be in good shape (as during read-in).  [B513]
      pUVCurve = GetUVTrimCurve();
      // SM_ASSERT_MSG(pUVCurve != NULL, _T("SmEdgeuse::CalcMaxEdgeFaceTrimCurveGap: UVTrimCurve construction failed for Edgeuse")) ;
      // NER( pUVCurve );
      if ( pUVCurve == NULL )
      { return SM_SUCCESS; }
    }

  // build projection of UVTrimCurve through pSurface into 3d Space
  SmCrvOnSurf sCrvOnSurf(*pUVCurve,*pSurface, NULL, 0, GetContext());

  // construct gap function
  ULONG               lSampleCnt = 20 ;
  SmExtent1d          sInterval = pEdge->GetInterval();
  SmCrvCrvGapFunction sEdgeFaceTrimCurveGF( sXSectTol3d, pEdgeCurve, sInterval, &sCrvOnSurf, lSampleCnt) ;

  // get max gap sample - forces sEdgeFaceTrimCurveGF lazy evaluation
  SmGapSample *pMaxGapSample;
  sEdgeFaceTrimCurveGF.GetMaxGapSample( pMaxGapSample ); NER( pMaxGapSample );

  // Max Gap locals
  // SmEdgeFaceTrimCurveGap   sMaxEdgeFaceTrimCurveGap3d ;
  // SmEdgeFaceTrimCurveGap * pMaxEdgeFaceTrimCurveGap3d = bCache ? &m_sMaxEdgeFaceTrimCurveGap3d 
  //                                                              : &sMaxEdgeFaceTrimCurveGap3d ;

  // Set EdgeFaceTrimCurve gap - SmEdgeFaceTrimCurveGap::Set() assigns m_pEdgeuse, m_dEdgeT, and m_dUVTrimCurveT vals
  //                             and calls SmVertexFaceGap::CacheGap() to compute a m_dGap3d value 
  // sMaxEdgeFaceTrimCurveGap3D.Set(this, 
  //                                sMaxGapSample.GetThisParam().x, 
  //                                sMaxGapSample.GetOtherParam().x) ;

  rMaxEdgeFaceTrimCurveGap.Set(this, 
                               pMaxGapSample->GetThisParam().x, 
                               pMaxGapSample->GetOtherParam().x) ;

  // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
  // // keep Mate gaps current
  // if(bCache && pMate) 
  //   { pMate->m_sMaxEdgeFaceTrimCurveGap3d = *pMaxEdgeFaceTrimCurveGap3d ;
  //     pMate->m_sMaxEdgeFaceTrimCurveGap3d.SetEdgeuse(pMate) ; 
  //   }
  // 
  // // set output
  // if(pMaxEdgeFaceTrimCurveGap) 
  //   { *pMaxEdgeFaceTrimCurveGap = sMaxEdgeFaceTrimCurveGap3d ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      sEdgeFaceTrimCurveGF.Dump() ;

      SmBrep *pBrep = GetBrep() ;
      pFace = GetFace() ;
      pEdge = GetEdge() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sEdgeFaceTrimCurveGF.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmEdgeuse::CalcMaxEdgeFaceTrimCurveGap

/*******************************************************************//**
PURPOSE: compute and/or cache max Edge/Face gap from 
         Edge->Curve to Face->Surface(UVTrimCurve)

NOTES: 1. caches gap in m_sMaxEdgeFaceGap3d and Mate->m_sMaxEdgeFaceGap3d
       2. computes and caches UVTrimCurve when uninitialized
       3. when edgeuse does not connect a curve to a surface, sets gaps to uninit
***********************************************************************/
SmStatus SmEdgeuse::CalcMaxEdgeFaceGap
 (SmEdgeFaceGap  * pOptMaxEdgeFaceGap, // out: calculated Edge/Face gap, must pt to SmGapObj to fill, NULL to ignore, default:[NULL]
  SmBoolean        bCache)             // in : TRUE=cache calculated Edge/Face gap, 
 const                                 //      FALSE=no change to cache, default:[TRUE]
{
  // init output
  if(pOptMaxEdgeFaceGap) 
    { pOptMaxEdgeFaceGap->ReSet() ; }

  // Get Mate (Mates share the same gaps)
  SmEdgeuse *pMate = GetMate() ;

  // when asked - clear cache
  if(bCache)
    { m_sMaxEdgeFaceGap3d.ReSet() ; 
      if(pMate) 
        { pMate->m_sMaxEdgeFaceGap3d.ReSet() ; }
    }

  // no work - no VertexEdge Gap
  if(!HasEdgeFaceGap())
    { return( SM_SUCCESS ) ; }

  // check state
  SM_ASSERT_MSG(   GetFace() != NULL
                && GetFace()->GetSurface() != NULL
                && GetEdge() != NULL
                && GetEdge()->GetCurve() != NULL,
                _T("SmEdgeuse::CalcMaxEdgeFaceGap: Edgeuse use not connected to a Curve and a Surface")) ;

  // Locals for Edge/UVTrimCurve gap calc
  SmEdge          * pEdge      = GetEdge();
  SmFace          * pFace      = GetFace();
  SmCurve         * pEdgeCurve = pEdge->GetCurve() ;
  SmSurface       * pSurface   = pFace->GetSurface() ;
  const SmContext * pContext   =   GetContext() ? GetContext()
                                 : pEdge->GetContext() ? pEdge->GetContext()
                                 : pFace->GetContext() ? pFace->GetContext()
                                 : NULL ;

  // gwc: avoid infinite loop - don't call SmTol::GetXSectTol3d(pEdge,pFace) which comes back to this function.
  // gwc: Tol, a temp value, can be any number, only used for sEdgeFaceGF construction then deleted.
  SmXSectTol3d        sXSectTol3d  = SmTol::GetXSectTol3d(pContext) ;

  // construct gap function
  ULONG               lSampleCnt = 20 ;
  SmExtent1d          sInterval  = pEdge->GetInterval();
  if(sInterval.GetMin() == sInterval.GetMax() )
    return SM_ERR;

  SmCrvSrfGapFunction sEdgeFaceGF( sXSectTol3d, pEdgeCurve, sInterval, pSurface, lSampleCnt) ; 

  // get max gap sample - forces sEdgeFaceGF lazy evaluation
  SmGapSample *pMaxGapSample = NULL;
  sEdgeFaceGF.GetMaxGapSample( pMaxGapSample ); NER( pMaxGapSample );

  // Max Gap locals
  SmEdgeFaceGap   sEdgeFaceGap3d ;
  SmEdgeFaceGap * pEdgeFaceGap3d = bCache ? &m_sMaxEdgeFaceGap3d 
                                          : &sEdgeFaceGap3d ;

  // set Edge/Face gap - SmEdgeFaceGap::Set() assigns m_pEdgeuse, m_dEdgeT, and m_sFaceUV vals
  //                             and calls SmVertexFaceGap::CacheGap() to compute a m_dGap3d value 
  pEdgeFaceGap3d->Set( this, 
                        pMaxGapSample->GetThisParam().x, 
                       &pMaxGapSample->GetOtherParam()) ;

  // keep Mate gaps current
  if(bCache && pMate) 
    { pMate->m_sMaxEdgeFaceGap3d = m_sMaxEdgeFaceGap3d ;
      pMate->m_sMaxEdgeFaceGap3d.SetEdgeuse(pMate) ; 
    }

  // set output
  if(pOptMaxEdgeFaceGap) 
    { *pOptMaxEdgeFaceGap = *pEdgeFaceGap3d ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      sEdgeFaceGF.Dump() ;

      SmBrep *pBrep = GetBrep() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; sEdgeFaceGF.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmEdgeuse::CalcMaxEdgeFaceGap

/*******************************************************************//**
PURPOSE: compute and/or cache Edge/CCWEdge gap 

NOTES: 1. caches gap in m_sEdgeCCWEdgeGap3d 
       2. when edgeuse does not connect a curve to a surface, sets gaps to uninit
***********************************************************************/
SmStatus SmEdgeuse::CalcEdgeCCWEdgeGap
 (SmEdgeEdgeGap  * pOptEdgeCCWEdgeGap, // out: calculated Edge/CCWEdge gap, must pt to SmGapObj to fill, NULL to ignore, default:[NULL]
  SmBoolean        bCache)             // in : TRUE=cache calculated Edge/CCWEdge gap in this edgeuse's m_sEdgeCCWEdgeGap3d,
 const                                 //      FALSE=no change to cache, default:[TRUE]
{
  // init output
  if(pOptEdgeCCWEdgeGap) 
    { pOptEdgeCCWEdgeGap->ReSet() ; }

  // locals
  SmEdgeuse * pCCWEdgeuse = GetCCWEdgeuse() ;

  // when asked - clear cache
  if(bCache)
    { m_sEdgeCCWEdgeGap3d.ReSet() ; }

  // no work - no VertexEdge Gap
  if(!HasEdgeCCWEdgeGap())
    { return( SM_SUCCESS ) ; }

  // check state
  SM_ASSERT_MSG(   GetEdge() != NULL
                && GetEdge()->GetCurve() != NULL
                && pCCWEdgeuse->GetEdge() != NULL
                && pCCWEdgeuse->GetEdge()->GetCurve() != NULL,
                _T("SmEdgeuse::CalcEdgeCCWEdgeGap: ThisEdgeuse or CCWEdgeuse not connected to curve")) ;

  // Locals for Edge/UVTrimCurve gap calc
  SmEdge          * pEdge         = GetEdge() ;
  SmExtent1d        sEdgeIvl      = pEdge->GetInterval() ;
  double            dEdgeT        = (GetOrientation() == SM_OT_SAME) ? sEdgeIvl.GetMax() : sEdgeIvl.GetMin() ; 

  SmEdge          * pCCWEdge      = pCCWEdgeuse->GetEdge() ;
  SmExtent1d        sCCWEdgeIvl   = pCCWEdge->GetInterval() ;
  double            dCCWEdgeT     = (pCCWEdgeuse->GetOrientation() == SM_OT_SAME) ? sCCWEdgeIvl.GetMin() : sCCWEdgeIvl.GetMax() ; 

  // const SmContext * pContext      =   GetContext() ? GetContext()
  //                                   : pEdge->GetContext() ? pEdge->GetContext()
  //                                   : pCCWEdge->GetContext() ? pCCWEdge->GetContext()
  //                                   : NULL ;

  // set m_sEdgeCCWEdgeGap3d
  if(bCache)
    { 
      // cache this Edge/CCWEdge Gap3d
      m_sEdgeCCWEdgeGap3d.SetEdgeEdgeGap(this, pCCWEdgeuse, dEdgeT, dCCWEdgeT) ;

    } // end Cache check

  // set output
  if(pOptEdgeCCWEdgeGap) 
    { pOptEdgeCCWEdgeGap->SetEdgeEdgeGap(this, pCCWEdgeuse, dEdgeT, dCCWEdgeT) ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmBrep *pBrep = GetBrep() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,1) ; if(pCCWEdge) pCCWEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_sEdgeCCWEdgeGap3d.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmEdgeuse::CalcEdgeCCWEdgeGap

/*******************************************************************//**
PURPOSE: Calculates the sector angle to the Radial EU at dParam, in radians

NOTES: Choose dParam in this->Edge->Ivl
***********************************************************************/
SmStatus SmEdgeuse::CalcSectorAngle ( 
  double    dParam,     ///< [in] :Edge parameter at which angle is calculated  <br>
  double & rdAngle      ///< [out]:Sector angle at dParam in radians            <br>
) 
{
    double      dAngle = SM_UNDEF_DOUBLE;
    SmPoint3d   sBinPoint1, sBinPoint2;
    SmVector3d  sBinormal1, sBinormal2, sNormal1;
    SmEdgeuse  *pEdgeuse2;

    pEdgeuse2 = GetRadial();

    // sBinormalAvg will always point into the acute angle between sBinormal1 and sBinormal2
    // If sBinormalAvg IsInsideSector( sBinormal1, sNormal1, sBinormal2 ), then the sector angle
    // is the angle returned by AngleBetween.  Else, the sector angle is 2pi - AngleBetween.
    SER( EvaluateBinormal( dParam, FALSE, sBinPoint1, sBinormal1, NULL, &sNormal1 ) );
    SER( pEdgeuse2->EvaluateBinormal( dParam, FALSE, sBinPoint2, sBinormal2 ) );

    SER( sBinormal1.AngleBetween( sBinormal2, dAngle ) );

    SmVector3d sBinormalAvg = ( sBinormal1 + sBinormal2 ) / 2.0;
    SmStatus eStat = sBinormalAvg.Unitize();
    // If eStat != SM_SUCCESS, then dAngle = pi
    if ( eStat == SM_SUCCESS )
    {
        if ( !sBinormalAvg.IsInsideSector( sBinormal1, sNormal1, sBinormal2 ) )
        { dAngle = SM_2PI - dAngle; }
    }

    // Set return
    rdAngle = dAngle;
    return SM_SUCCESS;

} // end SmEdgeuse::CalcSectorAngle

/*******************************************************************//**
PURPOSE: Gets all edgeuses connected to this one through CCW and CW
    pointers.

NOTES: eMarkType is used when this edgeuse is a wire.  The
       connected wires get collected and marked.  To work
       properly it should be a NewMark.
***********************************************************************/
SmStatus SmEdgeuse::GetConnectedEdgeuses
  (SmTArray<SmEdgeuse*> & rEdgeuses,   // out: list of connected edgeuses (not marked)
   SmMarkType             eMarkType)   // in : only used for wires: passed to CollectWireEdges (not incremented - should be a NewMark)
 const
{
  // init output
  rEdgeuses.ReSet();

  // when Edgeuse is part of a loop
  if (IsLoopEdgeuse())   { GetLoopuse()->GetEdgeuses(rEdgeuses) ;
                           return(SM_SUCCESS) ;
                         }

  // else edgeuse is part of a wire network
  SM_ASSERT(IsShellEdgeuse()==TRUE) ;

  SmTArray<SmEdge*>    sEdges ;
  SmTArray<SmEdgeuse*> sWireEdgeuses ;
  SmTopologyTraverser sTT ;
  SmStatus sRtn = sTT.CollectWireEdges(GetEdge(),   // in : target wire (gets marked)
                                       sEdges,      // out: all wires connected to target wire (get marked)
                                       eMarkType) ; // in : specify mark for target objects (not incremented)

  // for every wire
  for(ULONG ii=0;ii<sEdges.GetSize(); ii++)
    {
      sWireEdgeuses.ReSet() ;
      sEdges[ii]->GetEdgeuses(sWireEdgeuses) ;
      rEdgeuses.Append(sWireEdgeuses) ;
    }

  // all done
  return(sRtn) ;

} // end SmEdgeuse::GetConnectedEdgeuses

/*******************************************************************//**
PURPOSE: Receive notification of things happening to the object and
take appropriate actions.

  NOTES: For example cleaning up the cache of an
object which is being deleted or edited.  Also clean up any attributes
and relations specific to this class which are not handled automatically
by construtors.
***********************************************************************/
void SmEdgeuse::Notify                 // expected calls: caller->Notify(Event, pData1, pData2, pData2)
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
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     // 
{
  switch (eNotifyOperation) 
    {
      case SM_NO_PRE_EDIT              : break ;
                                       
      case SM_NO_SPLIT_IN_BREP         : 
      case SM_NO_CHANGE_GEOMETRY       : 
      case SM_NO_SPLIT                 : { // Clear UVTrimCurve
                                           SetUVTrimCurve(NULL,0.0,TRUE) ; // TRUE = delete existing UVTrimCurve
                                       
                                           // Clear Cached Gap data
                                           SmEdgeuse *pMate = GetMate() ;
                                           m_sMaxEdgeFaceGap3d.ReSet() ;
                                           pMate->m_sMaxEdgeFaceGap3d.ReSet() ;
                                       
                                           // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
                                           // m_sMaxEdgeFaceTrimCurveGap3d.ReSet() ;
                                           // pMate->m_sMaxEdgeFaceTrimCurveGap3d.ReSet() ;
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
      case SM_NO_POST_EDIT             : break ;
      case SM_NO_MERGE                 : break ;
      case SM_NO_REG_PROPAGATION       : break ;
      case SM_NO_DESTRUCTION           : break ;
      case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmEdge::Notify - SM_NO_UNKNOWN event signalled")) ; } 
                                         break ;
    }

  // Propagate notification up hierarchy
  SmTopology::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmEdgeuse::Notify

/*******************************************************************//**
PURPOSE: draw 80% of edge beginning at edgeuse start and add
   a binormal vector pointing into the face to which the edgeuse is connected
   and a normal vector showing this edgeuse's sector.

NOTES: When UVTrimCurve exists also draws CrvOnSurf(UVTrimCurve, Face->Surface).
  When the UVTrimCurve is good, the two renderings should superimpose and
  not be visible in the display.  When the UVTrimCurve is bad, the graphics
  will look very confused.  This is to help display silently corrupt UVTrimCurves.
***********************************************************************/
SmDisplayList * SmEdgeuse::Draw
  (double dScale,               // in : Set size of BiNorm and Norm Vectors
                                //      BiNormLength = .2 * CurveLength * dScale
                                //      NormLength   = .1 * CurveLength * dScale
                                //      default:[1.0]
   SmBoolean       bDrawUVTrimCurves, // in : TRUE = Draw UVTrimCurves when present, FALSE=don't
                                      //        note: when UVTrimCurves are good - they draw on top of the edges and aren't seen
                                      //              when UVTrimCurves are bad - they vary widely from the edges and indicate debugging is needed
                                      //      default:[TRUE]
   SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
  const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // locals
  double dT;
  SmEdge      * pEdge        = GetEdge() ;
  SmExtent1d    sIvl         = pEdge->GetInterval();
  SmCurve     * pEdgeCurve   = pEdge->GetCurve() ;
  SmCurve     * pUVTrimCurve = GetUVTrimCurve() ;
  SmSurface   * pFaceSurface = GetFace() ? GetFace()->GetSurface() : NULL ; 
  SmPoint3d     sBinPnt;
  SmVector3d    sBinVec, sBinNorm;

  // no work - no EdgeCurve - Happens during Filleting
  if ( pEdgeCurve == NULL )
    { return pRtn; } 

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // get interval from start to 80% to end
  if (m_eOrientation == SM_OT_SAME)         { dT = sIvl.Evaluate(0.8);
      sIvl.SetMinMax(sIvl.GetMin(),dT);
    }
  else if(m_eOrientation == SM_OT_OPPOSITE) { dT = sIvl.Evaluate(0.2);
      sIvl.SetMinMax(dT,sIvl.GetMax());
    }
  else                                      { SE(SM_ERR);
      dT = 0.0;
    }

  // evaluate and output binormal vector at 80% point
  if (! pEdge->IsWire())
    {
      SE(EvaluateBinormal(dT,FALSE,sBinPnt,sBinVec, NULL, &sBinNorm));
      double dLen  = pEdgeCurve->ApproximateLength(sIvl,5);
      double dGain =  dScale >= 0.0
                    ? 0.2 * dLen * dScale
                    : 0.2 * dLen ;
      sBinVec  = sBinPnt + dGain * sBinVec ;
      sBinNorm = sBinVec + 0.5 * dGain * sBinNorm ;

      // draw BiNorm Vector
      smgfx_OutputLine(sBinVec.x, sBinVec.y, sBinVec.z,
                       sBinPnt.x, sBinPnt.y, sBinPnt.z, pOptGfxSet) ;

      // draw Norm Vector
      smgfx_OutputLine(sBinVec.x,  sBinVec.y,  sBinVec.z,
                       sBinNorm.x, sBinNorm.y, sBinNorm.z, pOptGfxSet) ;
    }

  // output the EdgeCurve over given interval
  pEdgeCurve->DrawWDeriv(sIvl, 0, NULL, pOptGfxSet) ;

  // when UVTrimCurve and Face->Surface exist - output the FaceUVTrimCurve
  if(pUVTrimCurve && pFaceSurface && bDrawUVTrimCurves)
    {
      SmCrvOnSurf sCrvOnSurf(*pUVTrimCurve, *pFaceSurface) ;
      sCrvOnSurf.DrawWDeriv(sIvl, 0, NULL, pOptGfxSet) ;

    } // end UVTrimCurve and Face->Surface existence check

  // end displayList
  smgfx_OutputColor(sColor,pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(dScale, bDrawUVTrimCurves, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmEdgeuse::Draw

/*******************************************************************//**
PURPOSE: Draw Edgeuse (draw 80% of edge beginning at edgeuse start and add
                       a binormal vector pointing into the face to which the edgeuse is connected
                       and a normal vector showing this edgeuse's sector.)
         and the Edge/Face Gap function

NOTES:
***********************************************************************/
SmDisplayList * SmEdgeuse::DrawEdgeFaceTrimCurveGap
  (double dScale,               // in : Set size of BiNorm and Norm Vectors
                                //      BiNormLength = .2 * CurveLength * dScale
                                //      NormLength   = .1 * CurveLength * dScale
                                //      default:[1.0]
   SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
  const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL,NULL,FALSE,pOptGfxSet);

  // Draw the Edgeuse and projected UVTrimCurve
  Draw(dScale, TRUE, pOptGfxSet) ;    // TRUE = draw UVTrimCurves

  // locals
  // SmEdgeFaceTrimCurveGap & rEdgeFaceTrimCurveGap = ((SmEdgeuse*)this)->GetEdgeFaceTrimCurveGap(FALSE) ; // FALSE = don't force Gap calculation

  // end displayList
  smgfx_OutputColor(sColor,pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(dScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmEdgeuse::DrawEdgeFaceTrimCurveGap

/*************************************************************
PURPOSE: draw micro view of Edge/Face connection

NOTES: o - Draw Edge to high resolution
       o - Draw EdgeStrip on Face using Face->MicroColor
**************************************************************/
SmDisplayList * SmEdgeuse::DrawMicro
 (double        * pOptMicroParam, // in : optional Edge param to be center of Micro Drawing
  SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                  //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // locals
  SmFaceuse      * pFaceuse       = GetFaceuse() ;
  SmFace         * pFace          = pFaceuse ? pFaceuse->GetFace() : NULL ;
  SmSurface      * pSurface       = pFace ? pFace->GetSurface() : NULL ;
  SmEdge         * pEdge          = GetEdge() ;
  SmCurve        * pCurve         = pEdge ? pEdge->GetCurve() : NULL ;
  SmBoolean        bSwapNormal    = pFaceuse ? (pFaceuse->GetOrientation() == SM_OT_OPPOSITE) : FALSE ; 

  // no work - missing geometry
  if(pSurface == NULL || pCurve == NULL)
    { return(NULL) ; }

  // Micro Sample Locals
  SmTArray<SmPoint3d>  sEdgePoints,     sFacePoints,     sFaceNormals ;
  SmTArray<SmPoint3d>  sEndEdgePoints,  sEndFacePoints,  sEndFaceNormals ;
  SmTArray<SmPoint3d> *pTgtEdgePoints, *pTgtFacePoints, *pTgtFaceNormals ;

  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawMicro = TRUE ;
  if(pOptMicroParam && GetEdge()->GetInterval().ContainsValue(*pOptMicroParam))
    {
      sDisp.m_dMicroEdgeParam = *pOptMicroParam ; 
    }

  // begin Build Micro Samples scope
    {
      // locals
      ULONG ii ;
      SmExtent1d    sEdgeIvl        = pEdge->GetInterval() ;
      SmPoint3d     sEdgePoint, sEdgeTangent, sEdgeBinormal, sFaceNormal ;
      SmPoint3d     sSurfPoint, sSurfInsidePoint, sSurfNormal, sSurfInsideNormal ;
      SmPoint2d     sSurfUVPoint, sSurfUVBinorm, sSurfUVInsidePoint ;
      SmPoint2d     sU( 1.0, 0.0 ) ;
      SmPoint2d     sV( 0.0, 1.0 ) ;

      SmApproxTol3d sApproxTol3d    = SmTol::GetApproxTol3d(pFace) ;
      double        dMicroEdgeParam =   sDisp.m_dMicroEdgeParam != SM_BIG_DOUBLE
                                      ? sDisp.m_dMicroEdgeParam
                                      : sEdgeIvl.Evaluate(0.5) ;
      ULONG         lTotalCount     = sDisp.m_lMicroEdgeSampleCount ;
      ULONG         lSameSizeCount  = 5 ;
      double        dStepInc        = 2.0 ;
      double        dMinStep        = SmTol::MapTo1d(sApproxTol3d,     // in : 3d Tol value to map to Param Space
                                                     dMicroEdgeParam,  // in : point on Curve
                                                     *pCurve) ;        // in : Curve mapping ParamSpace to 3dSpace
      double        dMaxStep        = sEdgeIvl.GetLength() / (double)lTotalCount ;
      if(dMinStep > dMaxStep) { dMinStep = dMaxStep ; }
      double        dStep, dEndParam, dS ;

      // for two passes - 0 = [MicroEdgeParam IvlStart], 1 = [MicroEdgeParam IvlEnd] 
      for(ii=0;ii<2;ii++)
        {
          ULONG       jj         = 0 ;

          if(ii==0) { dS              = dMicroEdgeParam ;
                      pTgtEdgePoints  = &sEdgePoints ;
                      pTgtFacePoints  = &sFacePoints ;
                      pTgtFaceNormals = &sFaceNormals ;
                      dStep           = -dMinStep ; 
                      dEndParam       =  sEdgeIvl.GetMin() ;
                    }
          else      { dS              = sEdgeIvl.SnapValue(dMicroEdgeParam + dMinStep, dMinStep) ;
                      pTgtEdgePoints  = &sEndEdgePoints ; 
                      pTgtFacePoints  = &sEndFacePoints ;
                      pTgtFaceNormals = &sEndFaceNormals ;
                      dStep           =  dMinStep ; 
                      dEndParam       =  sEdgeIvl.GetMax() ;
                    }

          // sample the Curve in micro steps = ApproxTol3d
          while(sEdgeIvl.ContainsValue(dS))
            {
              // Curve evaluate EdgePt, DropSurfPt, and DropSurfBiNormDir 
              EvaluateBinormal(dS,               // in : value within Edge->m_vInterval.
                               TRUE,             // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                                 //      FALSE = get/create pUVTrimCurve to calc Surface points
                               sEdgePoint,       // out: 3D pt on edge
                               sEdgeBinormal,    // out: unit-vector pointing to SmFace interior from rBinormalPoint
                               &sEdgeTangent,    // out: non-unit Edgeuse tangent at rBinormalPoint
                               &sFaceNormal,     // out: unit Faceuse normal
                               &sSurfUVPoint,    // out: Surface param value at edge point
                               &sSurfUVBinorm) ; // out: Surface Binormal param direction at edge point, not unitized
 
              // compute step inside parameter distance
              sSurfUVBinorm.Unitize() ;
              SmTol2d sStepUVSizeU = SmTol::MapTo2d(sApproxTol3d, sSurfUVPoint, sU, *pSurface) ; // TRUE = use Consistent Tol Model
              SmTol2d sStepUVSizeV = SmTol::MapTo2d(sApproxTol3d, sSurfUVPoint, sV, *pSurface) ; // TRUE = use Consistent Tol Model
              sSurfUVInsidePoint.Set(sSurfUVPoint.x + sStepUVSizeU * sSurfUVBinorm.x,
                                     sSurfUVPoint.y + sStepUVSizeV * sSurfUVBinorm.y) ; 

              // Get Surface points
              pSurface->EvaluatePoint(sSurfUVPoint,              sSurfPoint) ;
              pSurface->EvaluateNormal(sSurfUVPoint, TRUE, TRUE, sSurfNormal) ;
              pSurface->EvaluatePoint(sSurfUVInsidePoint,        sSurfInsidePoint) ;
              pSurface->EvaluateNormal(sSurfUVInsidePoint, TRUE, TRUE, sSurfInsideNormal) ;
              if(bSwapNormal) { sSurfNormal       = -sSurfNormal ;
                                sSurfInsideNormal = -sSurfInsideNormal ;
                              }

              // load gfx arrays
              pTgtEdgePoints->Add(sEdgePoint) ;

              // when ii == 0 array is going to be reversed - add SurfInside points first
              if(ii==0) { pTgtFacePoints->Add(sSurfInsidePoint) ;
                          pTgtFacePoints->Add(sSurfPoint) ;

                          pTgtFaceNormals->Add(sSurfInsideNormal) ;
                          pTgtFaceNormals->Add(sSurfNormal) ;
                        }
              else // array used as loaded - add SurfEdge points first
                        { pTgtFacePoints->Add(sSurfPoint) ;
                          pTgtFacePoints->Add(sSurfInsidePoint) ;

                          pTgtFaceNormals->Add(sSurfNormal) ;
                          pTgtFaceNormals->Add(sSurfInsideNormal) ;
                        }

              // exit case
              if(dS == dEndParam)
                { break ; }

              // increment dS - start with dMinStep sized increments for lSameSizeCount samples
              //                then increase step size every lSameSizeCount by dStepInc multiple
              //                until step size == dMaxStep
              jj = (jj+1)%lSameSizeCount ;
              if(jj==0 && smos_Fabs(dStep) < dMaxStep) { dStep *= dStepInc ; 
                                                         if(smos_Fabs(dStep) > dMaxStep)
                                                           { dStep = smos_Sgn(dStep) * dMaxStep ; }
                                                       }
              dS = sEdgeIvl.SnapValue(dS + dStep, dStep) ; 

            } // end while sampling the Edge
        } // end end iter 2 passes - 0 = [MicroEdgeParam IvlStart], 1 = [MicroEdgeParam IvlEnd]

      // combine the two points sets into one, reversing the StartEdgePoints
      sEdgePoints .ReverseArray(0,sEdgePoints .GetSize()) ;
      sFacePoints .ReverseArray(0,sFacePoints .GetSize()) ;
      sFaceNormals.ReverseArray(0,sFaceNormals.GetSize()) ;
      sEdgePoints .Append(sEndEdgePoints ) ;
      sFacePoints .Append(sEndFacePoints ) ;
      sFaceNormals.Append(sEndFaceNormals) ;

    } // end Build Micro Samples scope

  // save current draw state for later restore
  SmVector3d sColor     = smgfx_GetOutputColor(pOptGfxSet) ;
  double     dLineWidth = smgfx_GetOutputLineWidth(pOptGfxSet) ;

  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Set MicroEdge color = red and LineWidth >= 2.0
  smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
  if(dLineWidth < 4.0) 
    { smgfx_OutputLineWidth(4.0, pOptGfxSet) ; }

  // Draw the MicroEdge 
  smgfx_DrawPolyline((double *)sEdgePoints.GetDataArray(), sEdgePoints.GetSize(), pOptGfxSet) ;

  // restore Draw color and LineWidth
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pOptMicroParam, pOptGfxSet);
#endif
  return(pRtn) ;

} // end SmEdgeuse::DrawMicro

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertEdgeuse_list[] =
{
 /*  0 */ {SM_AT_POINTER,     _T("Bad Context"),                 _T("Edgeuse and m_pSorLU have different contexts") },
 /*  1 */ {SM_AT_POINTER,     _T("Bad Context"),                 _T("Edgeuse and m_pCCW have different contexts") },
 /*  2 */ {SM_AT_POINTER,     _T("Bad Context"),                 _T("Edgeuse and m_pCW have different contexts") },
 /*  3 */ {SM_AT_POINTER,     _T("Bad Context"),                 _T("Edgeuse and m_pVU have different contexts") },
 /*  4 */ {SM_AT_POINTER,     _T("Bad Context"),                 _T("Edgeuse and m_pUVTrimCurve have different contexts") },

 /*  5 */ {SM_AT_TYPE,        _T("Bad TYPE"),                    _T("SmEdgeuse->m_tEdgeuseType and m_pSorLU have inconsistent type values") },
 /*  6 */ {SM_AT_POINTER,     _T("Bad UVTrimCurve Owner"),       _T("EdgeuseUVTrimCurve->Owner != Edgeuse") },
 /*  7 */ {SM_AT_GEOMETRIC,   _T("Bad UVTrimCurve"),             _T("Edgeuse->UVTrimCurve jumps across a periodic surface boundary") },

 /*  8 */ {SM_AT_POINTER,     _T("Bad RadialPair Shell"),        _T("An Edgeuse/RadialEdgeuse pair not attached to same Shell") },
 /*  9 */ {SM_AT_DIRECTION,   _T("Warning 0.0 Angle Sector"),    _T("A TangentSector Edgeuse/RadialEdgeuse pair of SurfaceNormals point in opposite directions") },
 /* 10 */ {SM_AT_DIRECTION,   _T("Bad EdgeSector Normals"),          _T("An Edge->Edgeuse/RadialEdgeuse pair of SurfaceNormals don't consistently point into the sector") },

 /* 11 */ {SM_AT_GEOMETRIC,   _T("Bad UVTrimCurve"),             _T("Edgeuse->UVTrimCurve has zero length") },

 /* 12 */ {SM_AT_TOPOLOGICAL, _T("Bad Loop  Connection"),        _T("Edgeuse->GetLoop() obj doesn't contain this Edgeuse") },
 /* 13 */ {SM_AT_TOPOLOGICAL, _T("Bad Shell Connection"),        _T("Edgeuse->GetShell() obj doesn't contain this Edgeuse") },
 /* 14 */ {SM_AT_TOPOLOGICAL, _T("Bad LoopNeighbor Connection"), _T("Edgeuse->m_pCCW->m_pCW Ptr != Edgeuse") },
 /* 15 */ {SM_AT_TOPOLOGICAL, _T("Bad LoopNeighbor Connection"), _T("Edgeuse->m_pCW->m_pCCW Ptr != Edgeuse") },
 /* 16 */ {SM_AT_TOPOLOGICAL, _T("Bad Vertex Connection"),       _T("Edgeuse->m_pVU->GetLoopuse() Ptr != Edgeuse") },

 /* 17 */ {SM_AT_TOPOLOGICAL, _T("Bad Loopuse Pointer"),         _T("Edgeuse m_tEdgeuseType is SmLoopuse_TYPE when m_pSorLU does not point to a Loopuse") },
 /* 18 */ {SM_AT_TOPOLOGICAL, _T("Bad Shell Pointer"),           _T("Edgeuse m_tEdgeuseType is SmShell_TYPE when m_pSorLU does not point to a Shell") },

 /* 19 */ {SM_AT_COINCIDENCE, _T("Bad Loop Connection Vertex"),  _T("Edgeuse/Edgeuse LoopNeighbor connection not made through a single Vertex (probably a coincident vertex pair)") },
 /* 20 */ {SM_AT_POLE,        _T("Bad UVTrimCurve Near Pole"),   _T("UVTrimCurve ControlPolygon turns sharply to be tangent to nearby surface pole UV boundary.  The bad turn causes tessellation and other problems") },

 /* 21 */ {SM_AT_CACHE,       _T("Bad Edge/Face Cache"),         _T("Edgeuse cached Edge/Face Gap value is stale") },

 /* 22 */ {SM_AT_POINTER,     _T("Bad MatedPair Face"),           _T("An Edgeuse/MateEdgeuse pair not attached to same Face")            },
 /* 23 */ {SM_AT_DIRECTION,   _T("Bad RadialPair Orientations"),  _T("An Edgeuse/RadialEdgeuse pair do not have different Orientations") },
 /* 24 */ {SM_AT_DIRECTION,   _T("Bad MatedPair Orientations"),   _T("An Edgeuse/MateEdgeuse pair do not have different Orientations") },

 /* 25 */ {SM_AT_PARAMETERIZATION, _T( "Bad UVTrimCurve Parameterization" ), _T( "An Edgeuse UVTrimCurve->Ivl != Edge->Ivl" ) },
 /* 26 */ {SM_AT_PARAMETERIZATION, _T( "Bad UVTrimCurve Parameterization" ), _T( "An Edgeuse UVTrimCurve->Ivl NotContainedBy Edge->Curve->Ivl") },
 /* 27 */ {SM_AT_GEOMETRIC,        _T( "Bad Binormal Location" ),            _T( "An Edgeuse/RadialEdgeuse pair do not evaluate to the same binormal location") }
  // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
  // /* 22 */ {SM_AT_CACHE,       _T("Bad Edge/FaceTrimCurve Cache"), _T("Edgeuse cached Edge/FaceTrimCurve Gap value is stale") },

} ; // end sAssertEdgeuse_list[]

/*******************************************************************//**
AssertRule Predicate names and return values:
  predicate = SmBoolean sm_AssertTestClassNameRule#() ; Rtn: TRUE=OK,    FALSE=Bad, 
***********************************************************************/

/*******************************************************************//**
PURPOSE: AssertTest Rule 21 - Edgeuse Edge/Face Gap cached and freshly computed values must be equal 
NOTES:
  predicate = does Cached Edge/Face Gap == Calculated Edge/Face Gap.
  action    = Refresh Cache: Let Cached Edge/Face Gap = Calculated Edge/Face Gap
***********************************************************************/
SmBoolean sm_AssertTestEdgeuse21          // rtn: TRUE = okay, FALSE = problem
 (const SmEdgeuse * pEdgeuse,             // in : Edgeuse to test
  SmEdgeFaceGap   * pOptEdgeFaceGap=NULL, // out: Calculated Edge/Face Gap, must pt to SmGapObj to fill, NULL to ignore, default:[NULL] 
  SmTol3d         * pOptOutTol3d=NULL,    // out: optional tolerance for failed assert reports
  double          * pOptOutVal=NULL)      // out: optional value for failed assert reports
{ 
  SmEdgeFaceGap sEdgeFaceGap, * pEdgeFaceGap =  pOptEdgeFaceGap 
                                              ? pOptEdgeFaceGap 
                                              : &sEdgeFaceGap ; 
  SmEdgeFaceGap sStoredEdgeFaceGap ;

  // Test = FALSE when pEdgeuse->Edges intersects pEdgeuse->PeriodicSurface->Seam
  SmBoolean bRtn = pEdgeuse->IsMaxEdgeFaceGapFresh(*pEdgeFaceGap, sStoredEdgeFaceGap) ;

  // set output
  if(pOptOutTol3d) { *pOptOutTol3d = SmTol::GetXSectTol3d(pEdgeuse->GetContext())/1000.0 ; }
  if(pOptOutVal)   { *pOptOutVal   =   sStoredEdgeFaceGap.IsInit()
                                     ? *pEdgeFaceGap - sStoredEdgeFaceGap 
                                     : 0.0 ; 
                   }
  // all done
  return( bRtn ) ;

} // end sm_AssertTestEdgeuse21

// /*******************************************************************//**
//  PURPOSE: AssertHeal Rule 21 - When Edgeuse Edge/Face Gap is stale, 
//                                recalculate cached Gap value.
//  NOTES:
//  ***********************************************************************/
//  SmBoolean sm_AssertHealEdgeuse21(SmEdgeuse *pEdgeuse, SmAssertReport & rAReport,  SmAssertArray * pAList) 
//  { 
//    SM_REF1(pAList) ;
//    // remember Heal has run on this AssertReport
//    rAReport.m_eAssertType = SM_AT_HEALER ;
//  
//    // locals
//    SmEdgeFaceGap sEdgeFaceGap ;
//  
//    // check test - other fix functions may have already fixed this one
//    rAReport.m_bOK = sm_AssertTestEdgeuse21(pEdgeuse, &sEdgeFaceGap) ;
//    if(TRUE == rAReport.m_bOK)
//      { return(rAReport.m_bOK) ; }
//  
//    // fix broken Edgeuses - Refresh cached value.
//    pEdgeuse->SetMaxEdgeFaceGap(sEdgeFaceGap) ;
//  
//    // No Need to check - fix always works:  check change - fix worked when pNewBSplineSurface != NULL and NewSurface passes test
//    // rAReport.m_bOK = sm_AssertTestEdgeuse21(pEdgeuse) ;
//    rAReport.m_bOK = TRUE ;
//  
//    // Good Citizenship - log all Object changes with pAList here:
//    //                      pAList->LogReplaceObject(), 
//    //                      pAList->LogSplitObject(),
//    //                      pAList->LogMergeObject(),
//    //                      pAList->LogDeleteObject().
//    // no object changes to log
//  
//    // all done
//    if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value fixed stale Edge/Face gap problem.") ; }
//    else               { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value did NOT fixed stale Edge/Face gap problem.") ; }
//  
//    return(rAReport.m_bOK) ;
//  
//  } // end sm_AssertHealEdgeuse21
// end obsolete
  
// GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
// /*******************************************************************//**
// PURPOSE: AssertTest Rule 22 - When Edgeuse Edge/FaceTrimCurve Gap is stale, 
//                               recalculate cached Gap value.
// NOTES:
//   predicate = does Cached Edge/FaceTrimCurve Gap == Calculated Edge/FaceTrimCurve Gap.
//   action    = Refresh Cache: Let Cached Edge/FaceTrimCurve Gap = Calculated Edge/FaceTrimCurve Gap
// ***********************************************************************/
// SmBoolean sm_AssertTestEdgeuse22    // rtn: TRUE = okay, FALSE = problem
//  (const SmEdgeuse          * pEdgeuse,                      // in : Edgeuse to test
//   SmEdgeFaceTrimCurveGap   * pOptEdgeFaceTrimCurveGap=NULL, // out: Calculated Edge/FaceTrimCurve Gap, NULL to ignore, default:[NULL] 
//   SmTol3d                  * pOptOutTol3d=NULL,             // out: optional tolerance for failed assert reports
//   double                   * pOptOutVal=NULL)               // out: optional value for failed assert reports
// { 
//   SmEdgeFaceTrimCurveGap sEdgeFaceTrimCurveGap, * pEdgeFaceTrimCurveGap =   pOptEdgeFaceTrimCurveGap 
//                                                                           ? pOptEdgeFaceTrimCurveGap 
//                                                                           : &sEdgeFaceTrimCurveGap ; 
//   SmEdgeFaceTrimCurveGap sStoredEdgeFaceTrimCurveGap ;
// 
//   // Test = FALSE when pEdgeuse->Edges intersects pEdgeuse->PeriodicSurface->Seam
//   SmBoolean bRtn = pEdgeuse->IsEdgeFaceTrimCurveGapFresh(pEdgeFaceTrimCurveGap, &sStoredEdgeFaceTrimCurveGap) ;
// 
//   // set output
//   if(pOptOutTol3d) { *pOptOutTol3d = SmTol::GetXSectTol3d(*pEdgeuse->GetContext())/1000.0 ; }
//   if(pOptOutVal)   { *pOptOutVal   =   sStoredEdgeFaceTrimCurveGap.IsInit()
//                                      ? *pEdgeFaceTrimCurveGap - sStoredEdgeFaceTrimCurveGap 
//                                      : 0.0 ; 
//                    }
//   // all done
//   return( bRtn ) ;

// } // end sm_AssertTestEdgeuse22
// 
// /*******************************************************************//**
// PURPOSE: AssertHeal Rule 22 - When Edgeuse Edge/FaceTrimCurve Gap is stale, 
//                               recalculate cached Gap value.
// NOTES:
// ***********************************************************************/
// SmBoolean sm_AssertHealEdgeuse22(SmEdgeuse *pEdgeuse, SmAssertReport & rAReport,  SmAssertArray * pAList) 
// { 
//   // remember Heal has run on this AssertReport
//   rAReport.m_eAssertType = SM_AT_HEALER ;
// 
//   // locals
//   SmEdgeFaceTrimCurveGap sEdgeFaceTrimCurveGap ;
// 
//   // check test - other fix functions may have already fixed this one
//   rAReport.m_bOK = sm_AssertTestEdgeuse22(pEdgeuse, &sEdgeFaceTrimCurveGap) ;
//   if(TRUE == rAReport.m_bOK)
//     { return(rAReport.m_bOK) ; }
// 
//   // fix broken Edgeuses - Refresh cached value.
//   pEdgeuse->SetEdgeFaceTrimCurveGap(sEdgeFaceTrimCurveGap) ;
// 
//   // No Need to check - fix always works:  check change - fix worked when pNewBSplineSurface != NULL and NewSurface passes test
//   // rAReport.m_bOK = sm_AssertTestEdgeuse22(pEdgeuse) ;
//   rAReport.m_bOK = TRUE ;
// 
//   // all done
//   if(rAReport.m_bOK) { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value fixed stale Edge/Face gap problem.") ; }
//   else               { rAReport.m_pHealMessage = _T("Refreshing Stored Gap Value did NOT fixed stale Edge/Face gap problem.") ; }
// 
//   return(rAReport.m_bOK) ;
// 
// } // end sm_AssertHealEdgeuse22
// 
/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmEdgeuse::AssertValid
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

  // Edgeuse and the objects it attaches to need to share common contexts
  if(m_pSorLU) { bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pSorLU->GetContext()), _T("") ) ; }
  if(m_pCCW)   { bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (GetContext() == m_pCCW->GetContext()), _T("") ) ; }
  if(m_pCW)    { bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (GetContext() == m_pCW->GetContext()), _T("") ) ; }
  if(m_pVU)    { bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (GetContext() == m_pVU->GetContext()), _T("") ) ; }
  if(m_pUVTrimCurve) { bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (GetContext() == m_pUVTrimCurve->GetContext()), _T("") ) ; }

  // SmEdgeuse->m_tEdgeuseType and m_pSorLU must have consistent type values
  if(m_pSorLU) { bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, 
                                                  SM_LEVEL_0, 
                                                  (   (m_pSorLU == NULL)
                                                   || (m_tEdgeuseType == SmLoopuse_TYPE && m_pSorLU->IsKindOf(SmLoopuse_TYPE))
                                                   || (m_tEdgeuseType == SmShell_TYPE   && m_pSorLU->IsKindOf(SmShell_TYPE))), 
                                                  _T("")) ;
               }

  // for edgeuses with UVTrimCurves on closed surfaces
  // check to make sure the UV trimCurve does not jump a periodic boundary
  if(m_pUVTrimCurve)
    {
      // get connected surface and its closed/open state in U and V directions
      SmFace      *pFace     = GetFace() ;
      SmEdge      *pEdge     = GetEdge() ;
      SmBrep      *pBrep     = GetBrep() ;
      SmCurve     *pCurve    = pEdge  ? pEdge->GetCurve()            : NULL;
      SmExtent1d   sUVTrimIvl= m_pUVTrimCurve->GetNaturalInterval();
      SmExtent1d   sEdgeIvl  = pEdge  ? pEdge->GetInterval()         : sUVTrimIvl;
      SmExtent1d   sECIvl    = pCurve ? pCurve->GetNaturalInterval() : sUVTrimIvl;
      SmSurface   *pSurface  = pFace  ? pFace->GetSurface()          : NULL ;
      SmExtent2d   sUVDomain ;
      if(pSurface) sUVDomain = pSurface->GetNaturalUVDomain() ;
      SmBoolean    bClosedU  = pSurface && pSurface->IsClosed(sUVDomain, SM_SP_U) ;
      SmBoolean    bClosedV  = pSurface && pSurface->IsClosed(sUVDomain, SM_SP_V) ;
      double       dTol3d    =   pEdge ? (double)pEdge->GetTolerance() 
                               : pFace ? (double)pFace->GetTolerance()
                               : pBrep ? (double)pBrep->GetTolerance()
                               : SM_ZONE_TOL_3D/10.0 ;

      // UVTrimCurve->Owner pointer must equal this Edgeuse pointer
      SmBoolean bCheck = m_pUVTrimCurve->GetOwner() == (SmObject *)this ;
      if(bCheck == FALSE)
        {
          SM_DBG_WARN(_T("SmEdgeuse::AssertValid: Found a TrimCurve without an Edgeuse owner")) ;
        }
       else // pSurface pointer seems to be valid
        {
          SmCrvOnSurf sCrvOnSurf(*m_pUVTrimCurve, *pSurface) ;
          double      dLength = sCrvOnSurf.ApproximateLength(sEdgeIvl, 5) ;

          /* 11 */ // UVTrimCurve cannot have zero length
          bRtn &= SM_ASSERT_VALUE_REPORT(11, SM_LEVEL_0, dLength >= dTol3d, dTol3d, dLength, _T("")) ;
        }

      /*  6 */
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (bCheck), _T("") ) ;

      /*  7 */ // see if the UVTrimCurve jumps across a periodic boundary inappropriately
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(7, SM_LEVEL_0, (m_pUVTrimCurve->AssertNoPeriodicJump(bClosedU, bClosedV, sUVDomain)), _T("") ) ;

      // Check UVTrimCurve parameterization. Should be equal to Edge interval and contained by Curve interval
      double       dScaledZeroMin = 100 * SmTol::GetScaledZero( sECIvl.GetMin() );
      double       dScaledZeroMax = 100 * SmTol::GetScaledZero( sECIvl.GetMax() );
      SmBoolean bEquality = !( smos_Fabs( sUVTrimIvl.GetMin() - sEdgeIvl.GetMin() ) > dScaledZeroMin
                              || smos_Fabs( sUVTrimIvl.GetMax() - sEdgeIvl.GetMax() ) > dScaledZeroMax );
      SmBoolean bContainment = !( sUVTrimIvl.GetMin() < sECIvl.GetMin() - dScaledZeroMin
                                 || sUVTrimIvl.GetMax() > sECIvl.GetMax() + dScaledZeroMax );

      /* 25 */
      bRtn &= SM_ASSERT_BOOLEAN_REPORT( 25, SM_LEVEL_0, bEquality, _T("") );

      /* 26 */
      bRtn &= SM_ASSERT_BOOLEAN_REPORT( 26, SM_LEVEL_0, bContainment, _T("") );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          m_pUVTrimCurve->Dump(FALSE) ;
          if(pSurface) pSurface->Dump(FALSE) ; 
        }
#endif // SM_DEBUG_CODE

    } // end m_pUVTrimCurve existence check

  /* 17 */ // check for Bad Loopuse Pointer, Edgeuse m_tEdgeuseType is SmLoopuse_TYPE when m_pSorLU does not point to a Loopuse
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(17, SM_LEVEL_0, (!IsLoopEdgeuse() || m_pSorLU->IsKindOf(SmLoopuse_TYPE)), _T("")) ;

  /* 12 */ // check for Bad Loop  Connection,  Edgeuse->GetLoopuse() obj doesn't contain this Edgeuse
  if(IsLoopEdgeuse() && m_pSorLU->IsKindOf(SmLoopuse_TYPE))
    {
      ULONG lIndx ;
      SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 32) ;
      GetLoopuse()->GetEdgeuses(sEdgeuses) ;
      SmBoolean bHas = sEdgeuses.FindElement((SmEdgeuse *)this, lIndx) ;
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(14, SM_LEVEL_0, (bHas == TRUE), _T("") ) ;
    }

  /* 18 */ // check for Bad Shell Pointer,  Edgeuse m_tEdgeuseType is SmShell_TYPE when m_pSorLU does not point to a Shell
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(18, SM_LEVEL_0, (!IsShellEdgeuse() || m_pSorLU->IsKindOf(SmShell_TYPE)), _T("")) ;

  /* 13 */ // check for Bad Shell Connection,  Edgeuse->GetShell() obj doesn't contain this Edgeuse
  if(IsShellEdgeuse() &&  m_pSorLU->IsKindOf(SmShell_TYPE))
    {
      ULONG lIndx ;
      SmBoolean bHas = FALSE ;

      // when Shell is a WireShell - this edgeuse should be on the linked list of owned topology
      if     (GetShell()->IsWireShell())
        { 
          SmEdge *pEdge = GetEdge() ;
          SM_PTR_ARRAY(sWireEdges, SmEdge, 32) ;
          GetShell()->GetWireEdges(sWireEdges) ; 
          bHas = sWireEdges.FindElement(pEdge, lIndx) ; 
        }
      else if(GetShell()->IsFaceuseShell())
        { // this is edgeuse is part of a wire edge attached to a FaceuseShell
          // gwc: I don't think there is a back pointer in this case. So let bHas = TRUE
          bHas = TRUE ; 
        }

      bRtn &= SM_ASSERT_BOOLEAN_REPORT(13, SM_LEVEL_0, (bHas == TRUE), _T("") ) ;
    }

  /* 14 */ // check for Bad LoopNeighbor Connection, Edgeuse->m_pCCW->m_pCW Ptr != Edgeuse
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(14, SM_LEVEL_0, (m_pCCW == NULL || m_pCCW->m_pCW == this), _T("") ) ;

  /* 15 */ // check for Bad LoopNeighbor Connection, Edgeuse->m_pCW->m_pCCW Ptr != Edgeuse
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(15, SM_LEVEL_0, (m_pCW == NULL || m_pCW->m_pCCW == this), _T("") ) ;

  /* 16 */ // check for Bad Vertex Connection,       Edgeuse->m_pVU->GetLoopuse() Ptr != Edgeuse
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(16, SM_LEVEL_0, (m_pVU == NULL || (m_pVU->IsEdgeVertexuse() == TRUE && m_pVU->GetEdgeuse() == this)), _T("")) ;

  /* 19 */ // check for Bad Loop Connection Vertex, Edgeuse LoopNeighbor connection not made through a single Vertex (probably a coincident vertex pair)
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(19, SM_LEVEL_0, (// the preconditions - a lot of them in this case 
                                                       m_pVU == NULL 
                                                    || m_pVU->GetVertex() == NULL
                                                    || GetEdge()->IsLamina()
                                                    || m_pCW == NULL 
                                                    || m_pCW->GetMate() == NULL 
                                                    || m_pCW->GetMate()->GetEdge() == NULL
                                                    || m_pCW->GetMate()->GetVertexuse() == NULL
                                                    || m_pCW->GetMate()->GetVertexuse()->GetVertex() == NULL
                                                    || m_pCW->GetMate()->GetEdge()->IsLamina() 
                                                    // the test
                                                    || ( m_pVU->GetVertex() == m_pCW->GetMate()->GetVertexuse()->GetVertex() )), 
                                                   _T("")) ; 

  // for SmLoopuse_TYPE edgeuses make sure of radial ordering
  if(m_tEdgeuseType == SmLoopuse_TYPE)
    {
      SmEdge    *pEdge   = GetEdge() ;
      SmExtent1d sExtent = pEdge->GetInterval() ;

      // test Manifold and Spine edges
      if(pEdge->IsManifold() || pEdge->IsSpine())
        {
          const SmEdgeuse *pTargetEdgeuse = this ;
          SmEdgeuse       *pRadialEdgeuse = pTargetEdgeuse->GetRadial() ;
          SmEdgeuse       *pMateEdgeuse   = pTargetEdgeuse->GetMate() ;

          // SAME-oriented edgeuses own mate/radial pair tests so each pair runs once.
          // Missing radial/mate is a failed pair check; geometry 9/10 needs both links.
          if( GetOrientation() == SM_OT_SAME )
            {
              /*  8 */ // check radial pair for consistent shell membership
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, (pRadialEdgeuse != NULL && pTargetEdgeuse->GetShell() == pRadialEdgeuse->GetShell()), _T("") ) ;
              
              /* 22 */ // check mate pair for consistent face membership
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(22, SM_LEVEL_0, (pMateEdgeuse != NULL && pTargetEdgeuse->GetFace() == pMateEdgeuse->GetFace()), _T("") ) ; 

              /* 23 */ // Edge->Edgeuse List Orientation order should be [... SAME OPPOSITE SAME OPPOSITE ...]
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(23, SM_LEVEL_0, (pRadialEdgeuse != NULL && pTargetEdgeuse->GetOrientation() != pRadialEdgeuse->GetOrientation()), _T("") ) ;

              /* 24 */ // Edge->Edgeuse List Orientation order should be [... SAME OPPOSITE SAME OPPOSITE ...]
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(24, SM_LEVEL_0, (pMateEdgeuse != NULL && pTargetEdgeuse->GetOrientation() != pMateEdgeuse->GetOrientation()), _T("") ) ;

              if( pRadialEdgeuse != NULL && pMateEdgeuse != NULL )
                {
                  SmPoint3d sTargetPnt, sTargetBiVec, sTargetFaceNormal, sTargetTangent ;
                  SmPoint3d sRadialPnt, sRadialBiVec, sRadialFaceNormal, sRadialTangent ;
    
                  // get BinormalEvaluations
                  pTargetEdgeuse->EvaluateBinormal(sExtent.Evaluate(.456), TRUE, sTargetPnt, sTargetBiVec, &sTargetTangent, &sTargetFaceNormal) ;
                  pRadialEdgeuse->EvaluateBinormal(sExtent.Evaluate(.456), TRUE, sRadialPnt, sRadialBiVec, &sRadialTangent, &sRadialFaceNormal) ;
    
                  // check for tangent sector orientation consistency
                  //  Both FaceNormals need to point into the sector
                  //  (Check using SM_EFF_ZERO, to allow for coincident faces.) - don't make new UVTrimCurves
                  SmBoolean bTangentSector = pTargetEdgeuse->IsTangentSector(5.0,
                                                                             FALSE,
                                                                             FALSE) ;
    
                  /*  9 */ // In a TangentSector both Faceuse Normals should point in the same direction - otherwise they are folded on top of one another.
                           // There are cases where a valid sector pinches out to a zero sector angle like a knife edge - so this is just a warning
                  bRtn &= SM_ASSERT_VALUE_REPORT(9, SM_LEVEL_0, (bTangentSector == FALSE || sTargetFaceNormal.Dot(sRadialFaceNormal) > -SM_EFF_ZERO), SM_EFF_ZERO, sTargetFaceNormal.Dot(sRadialFaceNormal), _T("")  ) ;
    
                  /* 10 */ // in a nonTangentSector both Faceuse Normals should point into the sector
                           //       when both Faceuse Normals point into the sectors
                           //       the crossProduct of the Binormals should point in the opposite
                           //       direction from the crossProduct of the FaceuseNormals
                  bRtn &= SM_ASSERT_VALUE_REPORT(10, SM_LEVEL_0, (bTangentSector == TRUE || (sTargetBiVec * sRadialBiVec).Dot(sTargetFaceNormal * sRadialFaceNormal) < SM_EFF_ZERO), SM_EFF_ZERO, (sTargetBiVec * sRadialBiVec).Dot(sTargetFaceNormal * sRadialFaceNormal), _T("") ) ;

                  /* 27 */ // check for consistent binormal location
                  bRtn &= SM_ASSERT_BOOLEAN_REPORT(27, SM_LEVEL_0, (sTargetPnt.DistanceBetween(sRadialPnt) < GetEdge()->GetTolerance()), _T("") ) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE ;
                  // draw 
                  if(bDebugMe1)
                    {
                      SmFace  *pThisFace   = this->GetFace();
                      SmFace  *pTargetFace = pTargetEdgeuse ? pTargetEdgeuse->GetFace() : NULL ;
                      SmFace  *pRadialFace = pRadialEdgeuse ? pRadialEdgeuse->GetFace() : NULL ;
                      SmBrep  *pBrep       = pEdge          ? pEdge->GetBrep() : NULL ;
    
                      smgfx_Erase() ;
                      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
                      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pThisFace) { pThisFace->Draw( SM_DM_CROSSHATCH, 3, 3 ); sm_GraphicsLoop(); }
                      smgfx_SetLook( 1, 2, 0, 1, 0 ); if(pTargetFace) { pTargetFace->Draw( SM_DM_CROSSHATCH, 5, 5 ); sm_GraphicsLoop(); }
                      smgfx_SetLook( 1, 2, 0, 1, 1 ); if(pRadialFace) { pRadialFace->Draw( SM_DM_CROSSHATCH, 7, 7 ); sm_GraphicsLoop(); }
    
                      smgfx_SetLook(3,4, 0,0,0) ; this->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,4, 0,1,0) ; if(pTargetEdgeuse) { pTargetEdgeuse->Draw() ; sm_GraphicsLoop() ;  }
                      smgfx_SetLook(3,4, 0,1,1) ; if(pRadialEdgeuse) { pRadialEdgeuse->Draw() ; sm_GraphicsLoop() ;  }
    
                      smgfx_SetLook(5,6, 1,0,0) ; sTargetPnt.Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, 1,0,0) ; sTargetFaceNormal.Draw(&sTargetPnt) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(7,8, 0,0,1) ; sRadialPnt.Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, 0,0,1) ; sRadialFaceNormal.Draw(&sRadialPnt) ; sm_GraphicsLoop() ;

                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE
                }
            }
       } // end Manifold or Spine Edge check
   } // end SmLoopuse_TYPE check

  // when Edgeuse has a UVTrimCurve - Check that it's valid
  if(m_pUVTrimCurve) 
    { 
      bRtn &= m_pUVTrimCurve->AssertValid(pAList, eTestLevel, SM_NO_WALK) ; 
    }

  /* 20 */ // The Control Polygon turns to follow a singular surface boundary.  That turn causes tessellation problems
  SmBoolean bHasDogLegNearPole = HasDogLegNearPole() ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(20, SM_LEVEL_0, (bHasDogLegNearPole == FALSE), _T("") ) ;

#ifdef SM_USE_OLDTOL // Skip new fresh Gap cache value checks until ready to debug the gap and Notify changes
#ifdef SM_NMTLIB_7166
  /* 21 */ // check if Edgeuse cached Edge/Face Gap value is stale
  SmTol3d sOutTol3d ;
  double  dOutVal ;
  bRtn &= SM_ASSERT_VALUE_REPORT(21, SM_LEVEL_2, sm_AssertTestEdgeuse21(this, NULL, &sOutTol3d, &dOutVal), sOutTol3d, dOutVal, _T("")) ;
  
  // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
  // /* 22 */ // check if Edgeuse cached Edge/FaceTrimCurve Gap value is stale
  // bRtn &= SM_ASSERT_VALUE_REPORT(22, SM_LEVEL_2, sm_AssertTestEdgeuse22(this, NULL, &sOutTol3d, &dOutVal), sOutTol3d, dOutVal, _T("")) ;
#endif // SM_NMTLIB_7166
#endif // SM_USE_OLDTOL

  // all done
  // SM_ASSERT(bRtn) ;

  return(bRtn) ;

} // end SmEdgeuse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmEdgeuse::AssertHeal
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
//       case 21 : bRtn = sm_AssertHealEdgeuse21(this, rAReport, pAList); break;
// 
//       // GWC - removed: no longer cache MaxEdge/FaceTrimCurve gaps
//       // case 22 : bRtn = sm_AssertHealEdgeuse22(this, rAReport, pAList); break;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmEdgeuse::AssertHeal fix not yet supported") ;
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmEdgeuse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Get the Brep of an edgeuse

NOTES: 
***********************************************************************/
SmBrep* SmEdgeuse::GetBrep() const 
{ 
  return ((SmEdge*)GetOwner())->GetBrep() ; 

} // end SmEdgeuse::GetBrep

/*******************************************************************//**
PURPOSE: Get (don't make) existing UVTrimCurve from an edgeuse or its mate. 

RETURN: Ptr to existing UVTrimCurve or NULL when UVTrimCurve is not attached.

NOTES: Moves pointer to SM_OT_SAME mate when appropriate 
***********************************************************************/
SmBSplineCurve* SmEdgeuse::GetUVTrimCurve
 (SmEdgeuse **pOptOwner)   // out: opt Owning Edgeuse of returned UVTrimCurve
const 
{
  SmBSplineCurve *pRet = m_pUVTrimCurve;
  if(pOptOwner) { *pOptOwner = (SmEdgeuse *)this ; }
  if(pRet == NULL)
   { SmEdgeuse *pMate = GetMate() ;
      if(pMate)  pRet  = pMate->m_pUVTrimCurve;
      if(pOptOwner) { *pOptOwner = pMate ; }
   }

#ifdef SM_DEBUG_CODE
  if(pRet != NULL && pRet->GetDim() != 2)
    {
      SM_ASSERT_MSG(pRet == NULL || pRet->GetDim() == 2, _T("SmEdgeuse::GetUVTrimCurve passed a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  return pRet;

} // end SmEdgeuse::GetUVTrimCurve

/*******************************************************************//**
PURPOSE: Get the DropCurveFail creation data from an edgeuse or its mate.

NOTES: Moves pointer to SM_OT_SAME mate when appropriate 
***********************************************************************/
SmDropCurveFail * SmEdgeuse::GetDropCurveFail
 (SmEdgeuse **pOptOwner)   // out: opt Owning Edgeuse of returned UVTrimCurve
const 
{
  SmDropCurveFail * pRet         = m_pDropCurveFail ;
  SmBSplineCurve  * pUVTrimCurve = m_pUVTrimCurve;
  if(pOptOwner) { *pOptOwner = (SmEdgeuse *)this ; }

  if(pUVTrimCurve == NULL)
    { SmEdgeuse *pMate = GetMate() ;
      if(pMate)  pRet  = pMate->m_pDropCurveFail;
      if(pOptOwner) { *pOptOwner = pMate ; }
    }

  return pRet;

} // end SmEdgeuse::GetDropCurveFail

/*******************************************************************//**
PURPOSE: Get the UV trim curve pointer from an edgeuse.

NOTES: 
***********************************************************************/
SmBSplineCurve* SmEdgeuse::GetUVTrimCurvePointer() const
{ 
#ifdef SM_DEBUG_CODE
  if(m_pUVTrimCurve != NULL && m_pUVTrimCurve->GetDim() != 2)
    {
      SM_ASSERT_MSG(m_pUVTrimCurve == NULL || m_pUVTrimCurve->GetDim() == 2, _T("SmEdgeuse::GetUVTrimCurvePointer passed a non-2D UVTrimCurve")) ;
    }
#endif // SM_DEBUG_CODE

  return m_pUVTrimCurve; 

} // end SmEdgeuse::GetUVTrimCurvePointer

/*******************************************************************//**
PURPOSE: Get Gap in Surface's 2d UVSpace between ThisEdgeuseStartPt and 
         CWEdgeEndPt

NOTES: 1. returns 0.0 when for NoGap Edgeuses ie. it's not a LoopEdgeuse 
       2. Uses cheap UVTrimCurves when available, else does a Pt3d to Surf drop
***********************************************************************/
double SmEdgeuse::GetStartPointUVLoopGap() const
{ 
  // check state - only Loopuse Edgeuses have End_to_End gaps
  if(FALSE == IsLoopEdgeuse() || m_pCW == NULL)
    { return 0.0 ; }

  // locals
  SmExtent1d       sThisEdgeIvl     = GetEdge()->GetInterval() ;
  double           dStartParam      = m_eOrientation == SM_OT_SAME ? sThisEdgeIvl.GetMin() : sThisEdgeIvl.GetMax() ; 
  SmExtent1d       sCCWEdgeIvl      = m_pCW->GetEdge()->GetInterval() ;
  double           dCWEndParam      = m_pCW->m_eOrientation == SM_OT_SAME ? sCCWEdgeIvl.GetMax() : sCCWEdgeIvl.GetMin() ;
  SmPoint3d        sStartUV, sCWEndUV ;
  SmBSplineCurve * pThisUVTrimCurve = GetUVTrimCurve() ;
  SmBSplineCurve * pCWUVTrimCurve   = m_pCW->GetUVTrimCurve() ;

  // note when UVTrimCurves exist
  SmBoolean bHaveUVTrimCurves = pThisUVTrimCurve && pCWUVTrimCurve ;

  // When UVTrimCurves are available - use them to get EndPt UV Values
  if(bHaveUVTrimCurves)
    {
      // Evaluate Edgeuse End UVPoints
      pThisUVTrimCurve->EvaluatePoint(dStartParam, sStartUV) ;  
      pCWUVTrimCurve->EvaluatePoint(dCWEndParam, sCWEndUV) ; 
    }
  else // Drop 3d Points to Surface
    {
      SmFace    * pFace    = GetFace() ; 
      SmSurface * pSurface = pFace ? pFace->GetSurface() : NULL ; 
      SmPoint3d   sStart3d, sCWEnd3d ;
      SmPoint2d   sStart2d, sCWEnd2d ; 
      SmBoolean   bSuccess ;
      SmBoolean   bIsMulti;
      double      dStartDropGap, dCWEndDropGap ; 

      // check state - must have a surface
      if(pSurface == NULL)
        return(0.0) ;

      // evaluate Edgeuse End 3dPoints
      GetEdge()->GetCurve()->EvaluatePoint(dStartParam, sStart3d) ;
      m_pCW->GetEdge()->GetCurve()->EvaluatePoint(dCWEndParam, sCWEnd3d) ;


      // drop point to Surface
      pSurface->DropPoint(sStart3d, pSurface->GetNaturalUVDomain(), NULL,      bSuccess, sStart2d, dStartDropGap, bIsMulti, SM_SO_MINIMIZE) ;  
      pSurface->DropPoint(sCWEnd3d, pSurface->GetNaturalUVDomain(), &sStart2d, bSuccess, sCWEnd2d, dCWEndDropGap, bIsMulti, SM_SO_MINIMIZE) ;  

      sStartUV = sStart2d ; 
      sCWEndUV = sCWEnd2d ; 
    } // end UVTrimCurve unavailble branch

  // compute and return the EdgeEnd to EdgeEnd gap
  double dRtn = sStartUV.DistanceBetween(sCWEndUV) ; 
  return(dRtn) ; 

} // end SmEdgeuse::GetStartPointUVLoopGap

/*******************************************************************//**
PURPOSE: Get Gap in Surface's 2d UVSpace between ThisEdgeuseEndUVPt and 
         CCWEdgeuseStartUVPt

NOTES: 1. returns 0.0 when for NoGap Edgeuses ie. it's not a LoopEdgeuse 
       2. Uses cheap UVTrimCurves when available, else does a Pt3d to Surf drop
***********************************************************************/
double SmEdgeuse::GetEndPointUVLoopGap() const
{ 
  // check state - only Loopuse Edgeuses have End_to_End gaps
  if(FALSE == IsLoopEdgeuse() || m_pCCW == NULL)
    { return 0.0 ; }

  // locals
  SmExtent1d       sThisEdgeIvl     = GetEdge()->GetInterval() ;
  double           dEndParam        = m_eOrientation == SM_OT_SAME ? sThisEdgeIvl.GetMax() : sThisEdgeIvl.GetMin() ; 
  SmExtent1d       sCCWEdgeIvl      = m_pCCW->GetEdge()->GetInterval() ;
  double           dCCWStartParam   = m_pCCW->m_eOrientation == SM_OT_SAME ? sCCWEdgeIvl.GetMin() : sCCWEdgeIvl.GetMax() ;
  SmPoint3d        sEndUV, sCCWStartUV ;
  SmBSplineCurve * pThisUVTrimCurve = GetUVTrimCurve() ;         // Get existing UVTrimCurve - don't build new
  SmBSplineCurve * pCCWUVTrimCurve  = m_pCCW->GetUVTrimCurve() ; // Get existing UVTrimCurve - don't build new

  // note when UVTrimCurves exist
  SmBoolean bHaveUVTrimCurves = pThisUVTrimCurve && pCCWUVTrimCurve ;

  // When UVTrimCurves are available - use them to get EndPt UV Values
  if(bHaveUVTrimCurves)
    {
      // Evaluate Edgeuse End UVPoints
      pThisUVTrimCurve->EvaluatePoint(dEndParam, sEndUV) ;  
      pCCWUVTrimCurve->EvaluatePoint(dCCWStartParam, sCCWStartUV) ; 
    }
  else // Drop 3d Points to Surface
    {
      SmFace    * pFace    = GetFace() ; 
      SmSurface * pSurface = pFace ? pFace->GetSurface() : NULL ; 
      SmPoint3d   sEnd3d, sCCWStart3d ;
      SmPoint2d   sEnd2d, sCCWStart2d ; 
      SmBoolean   bSuccess ;
      SmBoolean   bIsMulti;
      double      dEndDropGap, dCCWStartDropGap ; 

      // check state - must have a surface
      if(pSurface == NULL)
        return(0.0) ;

      // evaluate Edgeuse End 3dPoints
      GetEdge()->GetCurve()->EvaluatePoint(dEndParam, sEnd3d) ;
      m_pCCW->GetEdge()->GetCurve()->EvaluatePoint(dCCWStartParam, sCCWStart3d) ; 

      // drop point to Surface
      pSurface->DropPoint(sEnd3d, pSurface->GetNaturalUVDomain(), NULL,         bSuccess, sEnd2d, dEndDropGap, bIsMulti, SM_SO_MINIMIZE) ;  
      pSurface->DropPoint(sCCWStart3d, pSurface->GetNaturalUVDomain(), &sEnd2d, bSuccess, sCCWStart2d, dCCWStartDropGap, bIsMulti, SM_SO_MINIMIZE) ;  

      // load EndPt3d vals 
      sEndUV      = sEnd2d ; 
      sCCWStartUV = sCCWStart2d ; 
    } // end UVTrimCurve unavailble branch

  // compute and return the EdgeEnd to EdgeEnd Gap2d
  double dRtn = sEndUV.DistanceBetween(sCCWStartUV) ; 
  return(dRtn) ; 

} // end SmEdgeuse::GetEndPointUVLoopGap

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmEdgeuse::IsKindOf( SM_TYPE t ) const
{
  return ((SmEdgeuse_TYPE == t) ? TRUE : SmTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: pretty print edgeuse data

NOTES:
***********************************************************************/
void SmEdgeuse::Dump() const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // label
  smos_WriteBuffer( _T( "\nBegin SmEdgeuse::Dump()" ) );

  // header
  smos_sprintf( sBuff, _T( "\n SmEdgeuse:[0x%p]," ), this );
  smos_sprintf( sBuffForFile, _T( "\n SmEdgeuse:[%s]," ), _T( "NotNULL" ));
  smos_WriteBuffer( sBuff, sBuffForFile );


  // type
  smos_sprintf( sBuff, _T( " Type:[%s]," ), (m_tEdgeuseType == SmShell_TYPE) ? _T( "Shell" )
                                        : (m_tEdgeuseType == SmLoopuse_TYPE) ? _T( "Loopuse" )
                                        : (m_tEdgeuseType == SmUnknown_TYPE) ? _T( "Not yet Init" )
                                                                           : _T( "Unsupported TYPE" ) );
  smos_WriteBuffer( sBuff );

  // orientation
  smos_sprintf( sBuff, _T( " Orientation:[%s]" ), (m_eOrientation == SM_OT_SAME) ? _T( "Same" )
                                               : (m_eOrientation == SM_OT_OPPOSITE) ? _T( "Opposite" )
                                               : _T( "UnKnown" ) );
  smos_WriteBuffer( sBuff );

  // CCW, CW
  smos_sprintf( sBuff, _T( "\n  CCW :[0x%p], CW    :[0x%p]" ), m_pCCW, m_pCW );
  smos_sprintf( sBuffForFile, _T( "\n  CCW :[%s], CW    :[%s]" ), m_pCCW ? _T( "NotNULL" ) : _T( "NULL" ), m_pCW ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // Mate and Radial: (m_eOrientation==SM_OT_SAME) ? (Next Last) : (Last Next)
  smos_sprintf( sBuff, _T( "\n  Mate:[0x%p], Radial:[0x%p]" ), GetMate(), GetRadial() );
  smos_sprintf( sBuffForFile, _T( "\n  Mate:[%s], Radial:[%s]" ), GetMate() ? _T( "NotNULL" ) : _T( "NULL" ), GetRadial() ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );

  // base class dump
  SmTopology::Dump();

  // Nested Vertexuse
  smos_sprintf( sBuff, _T( "\n  Vertexuse:[0x%p]" ), m_pVU );
  smos_sprintf( sBuffForFile, _T( "\n  Vertexuse:[%s]" ), m_pVU ? _T( "NotNULL" ) : _T( "NULL" ) );
  smos_WriteBuffer( sBuff, sBuffForFile );
  m_pVU->Dump();

  // If m_pUVTrimCurve is Null, explain, and dump the other one.
  SmBSplineCurve *pUVTrimCurve = GetUVTrimCurve();
  if(m_pUVTrimCurve != NULL)
  {
    smos_sprintf( sBuff, _T( "\n  UVTrimCurve:[0x%p]" ), m_pUVTrimCurve );
    smos_sprintf( sBuffForFile, _T( "\n  UVTrimCurve:[%s]" ), m_pUVTrimCurve ? _T( "NotNULL" ) : _T( "NULL" ) );
    smos_WriteBuffer( sBuff, sBuffForFile );
  }
  else
  {
    smos_sprintf( sBuff, _T( "\n   UVTrimCurve is NULL and mate->UVTrimCurve:[0x%p]" ), pUVTrimCurve );
    smos_sprintf( sBuffForFile, _T( "\n   UVTrimCurve is NULL and mate->UVTrimCurve:[%s]" ), pUVTrimCurve ? _T( "NotNULL" ) : _T( "NULL" ) );
    smos_WriteBuffer( sBuff, sBuffForFile );
  }
  if(pUVTrimCurve) pUVTrimCurve->Dump();

  smos_WriteBuffer( _T( "\nEnd SmEdgeuse::Dump()\n" ) );

} // end SmEdgeuse::Dump( bAbbrev )
