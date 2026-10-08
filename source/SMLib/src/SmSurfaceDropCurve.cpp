// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceDropCurve.cpp
* PURPOSE: This file implements SmSurfaceDropCurve.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineSurface.h>
#include <SmBSplineCurve.h>
#include <SmSurfaceDropCurve.h>
#include <SmSolutionArray.h>
#include <SmGraphicsExtern.h>
#include <SmHermiteCurve.h>
#include <SmSurfaceCache.h>
#include <SmCurveCache.h>
#include <SmCacheMgr.h>
#include <SmGeomUtility.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmIsoCurve.h>
#include <SmCrvOnSurf.h>

#ifdef SM_DEBUG_CODE
#include <SmBrep.h>
#include <SmFace.h>
#include <SmAssertArray.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Construct a surface drop curve object.

NOTES: 
***********************************************************************/
SmSurfaceDropCurve::SmSurfaceDropCurve
 (const SmSurface      * cpSurface,      // in : target surface
  const SmExtent2d     & crUVDomain,     // in : allowed surface subdomain
  const SmCurve        & crCurveToDrop,  // in : target curve
  const SmExtent1d     & crInterval)     // in : Curve interval to drop
: SmSurfaceTracer(cpSurface,crUVDomain), 
  m_crCurveToDrop(crCurveToDrop),
  m_vInterval(crInterval), 
  m_dMaxDistanceToSurface(0.0),
  m_bCurveIsOnSurface(FALSE),
  m_pCurveKnotVector(NULL),
  m_pCurveBreaks(NULL) 
{
  // currently leaving the following uninitialized
  //  m_vWorkingInterval
  //  m_vWorkingUVDomain

} // end SmSurfaceDropCurve::SmSurfaceDropCurve constructor
    
/*******************************************************************//**
PURPOSE: Destructor for drop curve - clean up arrays

NOTES: 
***********************************************************************/
SmSurfaceDropCurve::~SmSurfaceDropCurve()
{
  if (m_pCurveKnotVector) { delete m_pCurveKnotVector; m_pCurveKnotVector = NULL ; }
  if (m_pCurveBreaks)     { delete m_pCurveBreaks;     m_pCurveBreaks     = NULL ; }

} // end SmSurfaceDropCurve::~SmSurfaceDropCurve destructor

/*******************************************************************//**
PURPOSE: Given SurfacePoint and CurvePoint 
         compute and save 
            SurfacePoint pos, 1st Derivs, and SurfaceNormal,
            DropCurve 3d pos, tangent, and
            DropUVCurve 2d pos, tangent.

NOTES: SmTracePnt objects have room for curve 2nd derivatives
                but they are not being set in this function.
***********************************************************************/
SmStatus SmSurfaceDropCurve::ComputePointValues
  (const SmPoint2d & crUV,        // in : target SurfacePoint
   SmTracePnt      & crCurrPnt,   // i/o: target CurvePoint, set with SurfacePoint properties
                                  //      in : m_dCurveParameter
                                  //      out: m_vSurfacePV[0][0] = Surf Position  for crUV
                                  //           m_vSurfacePV[1][0] = Surf 1stDerivU for crUV
                                  //           m_vSurfacePV[0][1] = Surf 1stDerivV for crUV
                                  //           m_vSurfacePV[2][2] = Surf Unit Normal    for crUV
                                  //           m_v3DCurvePV[0]    = Drop 3DCurve Pos (equals m_vSurfacePV[0][0])
                                  //           m_v3DCurvePV[1]    = Drop 3DCurve Tan (e3d CurveTan proj to SurfNorm plane)
                                  //           m_vUVCurvePV[0]    = UVCurve pos (equals crUV)
                                  //           m_vUVCurvePV[1]    = UVCurve Tan (3DCurveTan projected in 1stDeriv space)
                                  //           m_dSurfDropDist    = DropCurve(CurveParam) to DropSurfPoint dist in DropSurfNormLine dir
                                  //           m_dSurfNormLineDist=  DropCurve(CurveParam) to DropSurfNormLine min dist
   SmTracePnt      * pPrevPnt,    // in : Last intersection point on curve being stepped out, NULL to ignore
                                  //      When supplied used to handle singularity cases.
                                  //      default:[NULL]
   double          *)             // in : pdStepSize = NOT USED
                                  //      distance to step back from singularities to try and find 
                                  //      a nearby neighbor to use to computePointValues, NULL to ignore 
                                  //      default:[NULL] 
{
  // local
  SmVector3d sPV[2];

  // get Curve(crCurrPnt.CurveParam) position and tangent  
  SER(m_crCurveToDrop.Evaluate(crCurrPnt.m_dCurveParameter, 1,
                               m_vWorkingInterval.GetTLeftEval(crCurrPnt.m_dCurveParameter),
                               sPV));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw Brep(blue), Surface(yellow), Face(green), Curve(cyan), dropPoint(red)
  if(bDebugMe)
    {
      SmFace *pFace = m_cpSurface ? (SmFace *)m_cpSurface->GetFace() : NULL ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; m_cpSurface->DrawUV(4,4) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 0,1,1) ; m_crCurveToDrop.DrawWDeriv(m_vInterval) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; sPV[0].Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when curveToDrop tangent is degenerate
  if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      // move param a small step into the interval 
      double dParam =   (m_vWorkingInterval.GetTLeftEval(crCurrPnt.m_dCurveParameter))
                      ? crCurrPnt.m_dCurveParameter+SM_EFF_ZERO_SQRT
                      : crCurrPnt.m_dCurveParameter-SM_EFF_ZERO_SQRT ;

      // get Curve position and tangent for the moved param value
      SER(m_crCurveToDrop.Evaluate(dParam, 1,
                                   m_vWorkingInterval.GetTLeftEval(crCurrPnt.m_dCurveParameter),
                                   sPV));

    } // end degenerate Curve(param) point check
  
  // Evaluate SurfacePoint
  SmPoint3d sSrfEval[2][2];
  SER(m_cpSurface->Evaluate(crUV, 1, 1,
                            m_vWorkingUVDomain.GetULeftEval(crUV.x), 
                            m_vWorkingUVDomain.GetVLeftEval(crUV.y),
                            TRUE, &sSrfEval[0][0]));

  // output SurfacePoint position and tangent
  crCurrPnt.m_vSurfacePV[0][0] = sSrfEval[0][0];
  crCurrPnt.m_vSurfacePV[1][0] = sSrfEval[1][0];
  crCurrPnt.m_vSurfacePV[0][1] = sSrfEval[0][1];

  // get surfaceNormal
  SmVector3d sNorm =   crCurrPnt.m_vSurfacePV[1][0] 
                     * crCurrPnt.m_vSurfacePV[0][1];
  if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      SER(m_cpSurface->EvaluateNormal(crUV, TRUE, TRUE, sNorm));

      // signal error when unable to get a surfaceNormal
      if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) 
        {
          SER_MSG(SM_ERR, _T("SmSurfaceDropCurve::ComputePointValues) Unable to get SurfaceNormal at dropPoint"));
        }
    } // end singular surface check

  // output unitized surface normal in [2][2]
  sNorm.Unitize();
  crCurrPnt.m_vSurfacePV[2][2] = sNorm;

  // output DropCurve Position and Tangent (projected to surfaceTangentPlane)
  crCurrPnt.m_v3DCurvePV[0] = crCurrPnt.m_vSurfacePV[0][0];
  crCurrPnt.m_v3DCurvePV[1] = sPV[1].ProjectToPlane(sNorm);

  // when DropCurveTangent is degenerate
  if (crCurrPnt.m_v3DCurvePV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      // copy PreviousPoint Tangent Value or signal error
      if (pPrevPnt) { crCurrPnt.m_v3DCurvePV[1] = pPrevPnt->m_v3DCurvePV[1];
                    }
      else          { SER_MSG(SM_ERR, _T("SmSurfaceDropCurve::ComputePointValues, Curve Tangent degenerates when dropped to Surface at dropPoint"));
                    }
    } // end degenerate DropCurveTangent check
    
  // output DropUVCurve position 
  crCurrPnt.m_vUVCurvePV[0] = crUV;

  // output DropUVCurve tangent as projection of curve3DTangent to SurfaceTangentPlane
  //   when surface point is singular - return appropriate isoParam direction
  SmSurfParamType eSingDir ;
  if(!m_cpSurface->IsSingularity(crUV, eSingDir))
    { 
      // when surfPoint is not singular - project curve3dTangent to get curveUVTangent
      SER(smsurf_DropVectors(crCurrPnt.m_vSurfacePV[1][0],
                             crCurrPnt.m_vSurfacePV[0][1],
                             1,
                             &crCurrPnt.m_v3DCurvePV[1],
                             &crCurrPnt.m_vUVCurvePV[1]));
    }
  else // surfPoint is Singular - Drop a point to pick appropriate isoParam line to report
    {
      // when given a PrevPnt - use its tangent value
      if(pPrevPnt)
        {
          crCurrPnt.m_vUVCurvePV[1] = pPrevPnt->m_vUVCurvePV[1] ; 
        }
      else // not given a prev point
        {
          // get isoParam vector pointing into surf from singular boundary
          SmExtent2d sDomain = m_cpSurface->GetNaturalUVDomain() ;
          SmVector2d sUVTang ;
          if(!sDomain.IsPoint2dOnBoundary(crUV,SM_EFF_ZERO_SQRT, &sUVTang))
            {
              // error - singular surface point not on boundary
              SER(SM_ERR) ;
            }

          // drop a point a small step in the 3d tangent direction onto the surf
          // to set the curveUVPoint appropriately for a isoparam 1st step.
          SmSolutionArray sSolutions ;
          SmPoint2d sDropUV ;
          double dSize         = crCurrPnt.m_v3DCurvePV[1].Length() ;
          SmPoint3d sStepPoint =   crCurrPnt.m_v3DCurvePV[0] 
                                 + 0.0001/dSize 
                                 * crCurrPnt.m_v3DCurvePV[1] ;
          SER(m_cpSurface->GlobalPointSolve(m_cpSurface->GetNaturalUVDomain(),
                                            SM_SO_MINIMIZE, sStepPoint, .001, NULL,
                                            SM_SR_SINGLE, sSolutions)) ;
          // Get DropUVPoint
          // when drop point was not found - just use current crUVPoint value
          if(sSolutions.GetSize() == 0) { sDropUV = crUV ; }
          else                          { sDropUV.Set(sSolutions[0].m_vStart[0],
                                                      sSolutions[0].m_vStart[1]) ;
                                        }

          // set UVPoint so that UVTangent will be an isoParameter direction
          if(eSingDir == SM_SP_U) { crCurrPnt.m_vUVCurvePV[0].x = sDropUV.x ; }
          else                    { crCurrPnt.m_vUVCurvePV[0].y = sDropUV.y ; } 

          // set UVTangent for no PrevPnt case
          crCurrPnt.m_vUVCurvePV[1] = sUVTang ;
        } // end pPrevPnt == NULL check
    } // end SurfacePoint on SurfaceSingularity Check

  // when given a previous point snap UVDropCurves to isoParameter directions
  if (pPrevPnt) 
    {
      if (   smos_Fabs(pPrevPnt->m_vUVCurvePV[1].x) < SM_EFF_ZERO_SQRT
          && smos_Fabs(crCurrPnt.m_vUVCurvePV[1].x) < SM_EFF_ZERO_SQRT) 
        {             
          crCurrPnt.m_vUVCurvePV[1].x = 0.0;
          pPrevPnt->m_vUVCurvePV[1].x = 0.0;
        }
      if (   smos_Fabs(pPrevPnt->m_vUVCurvePV[1].y) < SM_EFF_ZERO_SQRT
          && smos_Fabs(crCurrPnt.m_vUVCurvePV[1].y) < SM_EFF_ZERO_SQRT) 
        {
          crCurrPnt.m_vUVCurvePV[1].y = 0.0;
          pPrevPnt->m_vUVCurvePV[1].y = 0.0;
        }
    } // end pPrevPnt check

  // calc Surf DropDist and NormLineDist
  smgu_LineClosestPoint(crCurrPnt.m_vSurfacePV[0][0],
                        crCurrPnt.m_vSurfacePV[2][2],
                        sPV[0],
                        crCurrPnt.m_dSurfDropDist) ;
  SmVector3d sLinePt = crCurrPnt.m_vSurfacePV[0][0] + crCurrPnt.m_dSurfDropDist * crCurrPnt.m_vSurfacePV[2][2] ;
  crCurrPnt.m_dSurfNormLineDist = (sLinePt - sPV[0]).Length() ; 
  
  // all done
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::ComputePointValues

/*******************************************************************//**
PURPOSE: Compute the step size for the next point given
         the current point and the last step size.

NOTES: step size is a parametric step size in 3D curve parameter space.

       NewStepSize is usually set equal to OldStepSize unless
       CurrPnt is on a boundary, then NewStepSize = 0.0 (no more steps), or
       NewStepSize would step beyond a boundary, then
          NewStepSize = truncated(OldStepSize).

       NewStepSize is checked only for excursions outside of surface 
       and curve boundaries. It is not checked here to see if the 
       NewStepSize produces a DropSpanSegment whose distance 
       to the true DropSegment's exceeds ApproxTol or not.
***********************************************************************/
SmStatus SmSurfaceDropCurve::ComputeStepSize
  (SmTracePnt & crCurrPnt,     // in : Last found DropPoint
   double       dOldStepSize,  // in : StepSize to last found DropPoint
   double     & rdNewStepSize) // out: StepSize to next Point to Drop. 
                               //      0.0 = no more steps, CurrPnt is on a boundary
                               //      truncated = OldStepSize would step beyond a boundary
                               //      OldStepSize
{
  double dLastT = crCurrPnt.m_dCurveParameter;

  // when LastT is on the 3dCurve's interval boundary - set StepSize to zero and quit
  if (smos_Fabs(dLastT - m_vWorkingInterval.GetMax()) < SM_EFF_ZERO_SQRT) 
    {
      rdNewStepSize = 0.0;
      return SM_SUCCESS;
    }
    
  // locals - knot indices of CurveToDrop knot interval containing CurrPnt.parameter
  ULONG lMin = 0;
  ULONG lMax = m_pCurveKnotVector->GetSize() - 1;

  // Find the CurveToDrop Knot interval containing CurrPnt.parameter.  
  while (lMax > lMin + 1) 
    {
      ULONG lMid = (lMin + lMax) / 2;
      if (dLastT >= (*m_pCurveKnotVector)[lMid]) 
        { lMin = lMid; }
      else 
        { lMax = lMid; }
    }

  // get CurveToDrop params for KnotIvl containing CurrPnt.param - clip to m_vWorkingInterval
  double dMin = (*m_pCurveKnotVector)[lMin];
  double dMax =   (*m_pCurveKnotVector)[lMax] > m_vWorkingInterval.GetMax()
                ? m_vWorkingInterval.GetMax()
                : (*m_pCurveKnotVector)[lMax] ;

  // If we hit the end of the span 
  if (smos_Fabs(dMax-dLastT) < SM_EFF_ZERO_SQRT) 
    {
      // if there are more CurveToDrop spans to explore - set NewStepSize to the end of the next span
      if (lMax < m_pCurveKnotVector->GetSize() - 1) 
        {
          rdNewStepSize = (*m_pCurveKnotVector)[lMax+1] - dMax;
          return SM_SUCCESS;
        }

      // else there are no more CurveTo Drop spans to explore - zero the step size and quit
      rdNewStepSize = 0.0;
      return SM_SUCCESS;
    }

  // If at the beginning of a span - set step size to step to end of entire span.
  if (smos_Fabs(dMin-dLastT) < SM_EFF_ZERO_SQRT) 
    { rdNewStepSize = dMax - dMin; }  // gwc: this looks like a bug since this side effect gets ignored.
                                      //      either rdNewStepSize should be dOldStepSize or 
                                      //             we need a return SM_SUCCESS here or maybe
                                      //             something else - I'm thinking about it

  // If next step goes beyond the current span - truncate it
  if (dLastT + dOldStepSize > dMax) 
    {
      rdNewStepSize = dMax - dLastT;
      return SM_SUCCESS;
    }

  // If we made it to here then the last point was in the middle of
  // a span and the step size does not take it past the end of the
  // span.
  rdNewStepSize =   (dOldStepSize > SM_EFF_ZERO)
                  ? dOldStepSize
                  : dMax - dLastT ;

  // all done
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::ComputeStepSize

/*******************************************************************//**
PURPOSE: Drop a curve that does not cross a boundary of the surface.

NOTES: 
   We are just dropping a single contiguous curve.  If it crossed a
   boundary, then we have already found everything there is to find.
   If the input arguments indicate that some solution has already been
   found, we do nothing.  Otherwise we just call DropCurve(), which is
   not designed for curves crossing boundaries.
***********************************************************************/
SmStatus SmSurfaceDropCurve::FindInteriorCurves
 (SmTArray<SmCurve*> & r3DCurves,         // NotUsed: out: not modified
  SmTArray<SmCurve*> & rSurfaceUVCurves,  // out: resulting Dropped UVCurves
  SmTArray< double > & rMaxDropDists,     // out: Max Drop dist from 3dCurve(s) to Surface(drop_UV).          
  SmTArray< double > & rDeviations)       // out: Max gap dist between 3dCurve(s) and Surface(UVTrimCurve(s)).
{
  SM_REF1(r3DCurves) ;
  if ( rSurfaceUVCurves.GetSize() > 0 )
    { return SM_SUCCESS; }

  SM_ASSERT( rMaxDropDists.GetSize() == 0 );
  SM_ASSERT( rDeviations  .GetSize() == 0 );
  rMaxDropDists.ReSet();
  rDeviations  .ReSet();

  SmTArray< SmBSplineCurve* > sUVBSplCurves;
  double dMaxDropDist = 0.0, dMaxDeviation = 0.0;

  SmStatus eStat = m_cpSurface->DropCurve( *m_cpContext,
                                            m_vUVDomain,
                                            m_crCurveToDrop,
                                            m_vInterval,
                                            m_dThisApproxTol3d,
                                            dMaxDropDist,        // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                            dMaxDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                            sUVBSplCurves,
                                            TRUE, NULL, FALSE);

  SER( eStat );

  ULONG ii, lNumCurves = sUVBSplCurves.GetSize();
  for ( ii=0; ii<lNumCurves; ii++ )
    {
      rSurfaceUVCurves.Add( sUVBSplCurves[ii] );
      rMaxDropDists   .Add( dMaxDropDist      );
      rDeviations     .Add( dMaxDeviation     );
    }

  return SM_SUCCESS;

} // end SmSurfaceDropCurve::FindInteriorCurves

/*******************************************************************//**
PURPOSE: Label a point as a good drop Point when the distance between
  the test point and the SurfaceNormal line starting at the Surface Point
  is less than the given dApproxTol

NOTES: 
***********************************************************************/
static SmStatus sm_TestPoint
 (const SmSurface  & crSurface,                   // in : tgt surface
  const SmExtent2d & crUVDomain,                  // in : working surface domain
  const SmPoint2d  & crUVToTest,                  // in : UVPoint to test
  const SmPoint3d  & cr3DPointToTest,             // in : 3dPoint dropped to surface to test
  double             dApproxTol,                  // in : max allowed dist between a dropPoint
                                                  //      and it's associated SurfaceNormal line.
  SmBoolean        & rbGoodPoint,                 // out: TRUE = 3dPt is within tol of the SurfaceNormal Line starting at the SurfacePoint.
                                                  //      FALSE= isn't
  double           & rdSurfDropDist,              // out: 3d dist from 3dPt to surface in SurfNormLine dir
  double           & rdSurfNormLineDist)          // out: 3d Dist from 3dPt to SurfNormLine
//  double           & rdDistToSurfNormLineSquared) // out: 3d Dist from 3dPt to SurfaceNormalLine
{
  // init output
  rbGoodPoint = FALSE;

  // get Srf[pos normal]
  SmPoint3d sSurfPnt;
  SmVector3d sDU, sDV;
  SER(crSurface.Evaluate1stDerivatives(crUVDomain.ClampPoint2d(crUVToTest),TRUE,TRUE, sSurfPnt,sDU,sDV));
  SmVector3d sNorm = sDU * sDV;
  double     dNormLengthSq = sNorm.LengthSquared() ;
  if(dNormLengthSq < SM_EFF_ZERO_SQ) 
    {
      SER(crSurface.EvaluateNormal(crUVDomain.ClampPoint2d(crUVToTest),TRUE,TRUE,sNorm));
    }
  else
    {
      sNorm /= smos_Sqrt(dNormLengthSq) ;
    }

  // get distance between 3dPoint and the SurfaceNormal line originating at the SurfacePoint
  double dLineT = 0; // sNorm is unit - dLineT = DropDist
  SER(smgu_LineClosestPoint(sSurfPnt,sNorm,cr3DPointToTest,dLineT));
  SmVector3d sLinePnt = sSurfPnt + dLineT * sNorm;
  SmVector3d sLineOff = sLinePnt - cr3DPointToTest;
  double     dSurfNormLineDist = sLineOff.Length();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); sSurfPnt.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); sNorm.Draw(&sSurfPnt); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,1,0); cr3DPointToTest.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 0,0,1); sLinePnt.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,8, 0,0,1); sLineOff.Draw(&cr3DPointToTest); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  
  // when 3dPt is within tol of the SurfaceNormal line
  if(dSurfNormLineDist < dApproxTol) 
    {
      // save Good output
      rbGoodPoint                 = TRUE;
      rdSurfDropDist              = dLineT ;
      rdSurfNormLineDist          = dSurfNormLineDist;
      // SmVector3d sDiff2           = cr3DPointToTest - sSurfPnt;
      // rdDistToSurfNormLineSquared = sDiff2.LengthSquared();

    }
  else 
    {
      // save Bad output
      rbGoodPoint = FALSE;
      rdSurfDropDist              = 0.0 ;
      rdSurfNormLineDist          = 0.0 ;
      // rdDistToSurfNormLineSquared = 0.0;
    }

  // all done
  return SM_SUCCESS;

} // end sm_TestPoint

/*******************************************************************//**
PURPOSE: Compute the actual UV point given a guess UV point on the
         curve being traced.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceDropCurve::LocalPointSolve
 (const SmTracePnt & crTracePnt,     // NotUsed: in : curr TracePnt
  const SmPoint2d  & crGuessUV,      // in : UVPnt guess for next TracePnt drop
  SmTracePnt       & rNextPnt,       // i/o: in : m_dCurveParameter
                                     //      out: m_vUVCurvePV        = guess for NextPntUV
                                     //           m_dSurfDropDist     = 3d dist from 3dPt to surface in SurfNormLine dir
                                     //           m_dSurfNormLineDist = 3d Dist from 3dPt to SurfNormLine
  SmBoolean        & rbFoundAnswer,  // out: TRUE = NextPnt3d less than m_dThisApproxTol3d from 
                                     //             SurfNormLine starting at NextPnt.SurfacePnt
  SmPoint2d        & rUVFound)       // out: UVPnt found by NR iteration 
  const
{
  SM_REF1(crTracePnt) ;
  // init output
  rbFoundAnswer = FALSE;

  // locals
  SmBoolean    bFoundAnswer;
  SmSolution   sSolution;
  SmSolution & rSolution = sSolution;
  SmPoint3d    sCurrPnt;

  // 3d Pnt to drop = CurveToDrop(NextPnt.CurveParam)
  SER(m_crCurveToDrop.EvaluatePoint(rNextPnt.m_dCurveParameter, sCurrPnt));

  // Pass the Drop 3dPnt to Surface call along
  SER(m_cpSurface->LocalPointSolve(m_vWorkingUVDomain,
                                   SM_SO_NORMALIZE,
                                   sCurrPnt,
                                   crGuessUV,
                                   bFoundAnswer,
                                   sSolution));

  // when Local DropPoint to Surface fails - try global DropPoint to Surface
  if (!bFoundAnswer) 
    { 
      // locals
      SmSolution      sData[4];
      SmSolutionArray sSolutions(4,sData);

      // global drop point to surface
      SER(m_cpSurface->GlobalPointSolve(m_vWorkingUVDomain,
                                        SM_SO_MINIMIZE,
                                        sCurrPnt,
                                        SM_EFF_ZERO,
                                        NULL,
                                        SM_SR_ALL,
                                        sSolutions));

      // Take closest answer in parameter space to our original estimate
      double dSmallDiff=0.0;
      for (ULONG jj=0; jj<sSolutions.GetSize(); jj++) 
        {
          SmSolution & rTmpSol = sSolutions[jj] ;
          double dDiff =   smos_Fabs(rTmpSol.m_vStart[0] - crGuessUV.x)
                         + smos_Fabs(rTmpSol.m_vStart[1] - crGuessUV.y) ;

          // save solution closest to guessUV value
          if(   jj==0
             || dDiff < dSmallDiff)                  
            { dSmallDiff = dDiff;
              rSolution  = rTmpSol;
            }
        } // end iter all global solutions looking for best solution
    } // end Local drop point to Surface failed check
    
  // solution locals
  SmPoint2d  sFoundPt(rSolution.m_vStart[0],rSolution.m_vStart[1]);
  SmVector2d sDiff = sFoundPt - crGuessUV;
  rNextPnt.m_vUVCurvePV[0] = sFoundPt;

  // if SolPoint skipped across surface seam in U dir - report no good sol found
  if(   smos_Fabs(sDiff.x) > m_vWorkingUVDomain.XLength() / 4.0 
     && m_cpSurface->IsClosed(m_vWorkingUVDomain,SM_SP_U)) 
    {
        rbFoundAnswer = FALSE;
        return SM_SUCCESS;
    }

  // if SolPoint skipped across surface seam in V dir - report no good sol found
  if(   smos_Fabs(sDiff.y) > m_vWorkingUVDomain.YLength() / 4.0 
     && m_cpSurface->IsClosed(m_vWorkingUVDomain,SM_SP_V)) 
    {
      rbFoundAnswer = FALSE;
      return SM_SUCCESS;
    }
  
  // Found Point is good when dist between test pt and SurfNormal line starting at SurfPnt is less than ApproxTol
  SmBoolean bIsGoodPoint;
  SER(sm_TestPoint(*m_cpSurface,                   // in : tgt surface
                   m_vWorkingUVDomain,             // in : working surface domain
                   sFoundPt,                       // in : UVPoint to test
                   sCurrPnt,                       // in : 3dPoint dropped to surface to test
                   m_dThisApproxTol3d,             // in : max allowed dist between a dropPoint
                                                   //      and it's associated SurfaceNormal line.
                   bIsGoodPoint,                   // out: TRUE = 3dPt is within tol of the SurfaceNormal Line starting at the SurfacePoint.
                                                   //      FALSE= isn't
                   rNextPnt.m_dSurfDropDist,       // out: 3d dist from 3dPt to surface in SurfNormLine dir
                   rNextPnt.m_dSurfNormLineDist)); // out: 3d Dist from 3dPt to SurfNormLine

  // when NextTracePnt is bad - report no good sol found
  if (!bIsGoodPoint) 
    {
      rbFoundAnswer = FALSE;
      return SM_SUCCESS;
    }

  // arrive here when good NextTracePnt is found - set output and return
  rbFoundAnswer = TRUE;
  rUVFound      = sFoundPt;
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::LocalPointSolve

/*******************************************************************//**
PURPOSE: Insert dNewBreak value into rBreaks, keeping rBreaks
            sorted and free of duplicates.

NOTES:
***********************************************************************/
void sm_AddBreak
 (SmTArray<double> & rBreaks,   // out: array of accumulated values
  double             dNewBreak) // in : target value
{

  double dScaledZero = SM_EFF_ZERO * 100.0 * (1.0 + smos_Fabs(dNewBreak));

  // Do an insertion sort of dNewBreak into rBreaks if it is 
  // not already in the array.

  // find newBreak values position in the rBreaks list
  ULONG ii, lInsertion = rBreaks.GetSize();
  for (ii=0; ii<rBreaks.GetSize(); ii++) 
    {
      // skip duplicate values
      if (smos_Fabs(dNewBreak-rBreaks[ii]) < dScaledZero) return; 

      // mark target index
      if (dNewBreak < rBreaks[ii]) 
        {
          lInsertion = ii;
          break;
        }
    }
  
  // insert new value
  rBreaks.InsertAt(lInsertion,dNewBreak,1);

} // end sm_AddBreak

/*******************************************************************//**
PURPOSE: Static helper for FindBoundaryStartPoints(): Test whether
   a curve-curve intersection is a grazing (tangent) intersection.

NOTES:  
***********************************************************************/
static SmBoolean sm_IsGrazingIntersection( const SmCurve* cpCurve1,
                                                 double dCurve1Param,
                                           const SmCurve* cpCurve2,
                                                 double dCurve2Param )
{
  // Method: check tangents.  If parallel, then it's a graze.
  SmVector3d sEval1[2], sEval2[2];
  cpCurve1->Evaluate( dCurve1Param, 1, FALSE, sEval1 );
  cpCurve2->Evaluate( dCurve2Param, 1, FALSE, sEval2  );
  if ( sEval1[1].IsParallelTo( sEval2[1] ) )
    { return TRUE; }

  return FALSE;

} // end sm_IsGrazingIntersection

/*******************************************************************//**
PURPOSE: 1. Place startPoints into m_vStartPoints.
            2. Place Curve Segments that drop to Surface IsoParameter Curves into rSurfaceUVCurves.
               note: StartPoints are BreakPoints starting a dropSegment (other than the isoParameter ones). 
                 BreakPoints include:   
                    a. Curve Start/End points,
                    b. Curve discontinuity points,
                    c. SM_ST_SINGLE_VALUE sol pts for Curve/SurfaceBoundary intersection points
                    d. SM_ST_SINGLE_VALUE sol pts for Curve/SurfaceDiscontinuity IsoParamCurve intersection points
                    e. SM_ST_RANGE_OF_VALUES sol end pts for Curve/SurfaceBoundary coincident ranges 
                    f. SM_ST_RANGE_OF_VALUES sol end pts for Curve/SurfaceDiscontinuity IsoParamCurve coincident ranges
METHOD ---
  1. Gather candidate start points as
     a. curve start/end Points
     b. curve/Surface boundary intersections
     c. curve/Surface isoparameter Line continuity boundary intersections
     d. curve discontinuity points.
  
  2. For every candidate point 
     a. if point marks an interval that drops to a Surface IsoCurve
        - make UVLine and add it to outputs
     b. if point starts an interval that drops into Surface (rather than out)
        - add point to m_vStartPoints array
     c. if point starts an interval that drops out of Surface
        - discard the point 

  NOTE: Curves segments that are coincident with surface boundary curves will generally
        have different parameterization than the surface boundary isoLines.  So,
        when coincident segments are found, just add their end points to the
        candidate start point array and let the upcoming DropCurve create
        a UVTrimCurve coincident with the SurfaceNaturalBoundary with 
        the proper parameterization.
***********************************************************************/
SmStatus SmSurfaceDropCurve::FindBoundaryStartPoints
  (SmTArray<SmCurve*> & r3DCurves,         // NotUsed: out: Not Used - drops m_crCurveToDrop - does not gen any 3d curves 
   SmTArray<SmCurve*> & rSurfaceUVCurves,  // out: all curve intervals that drop to SurfaceIsoCurves 
   SmTArray<double>   & rMaxDropDists,     // out: max m_crCurveToDrop SmpPoint to SurfDropPt dist for every output curve 
   SmTArray<double>   & rDeviations)       // out: max m_crCurveToDrop SmpPoint to DropSurfNormLine dist for every output curve
{
  SM_REF1(r3DCurves) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      // draw Brep(blue), TargetSurface(cyan), Face(black), TargetCurve(green), m_vStartPoints(red)
      if(bDebugMe)
        {
          m_cpSurface->Dump() ;
          m_crCurveToDrop.Dump() ;

          SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
          SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;
          SmTracePnt *pPnt1 = m_vStartPoints.GetFirstNode() ;
          SmTracePnt *pI    = NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 1,1,0) ; m_crCurveToDrop.DrawParams() ; sm_GraphicsLoop() ;
          ULONG mm ;
          smgfx_SetLook(3,5, 1,0,0) ; for(mm=0, pI=pPnt1; 
                                          pI && (mm == 0 || pI != pPnt1);
                                          mm++, pI = m_vStartPoints.GetNextNode(pI))
                                        {
                                          pI->Draw(m_cpSurface->GetContext()) ; sm_GraphicsLoop() ;
                                        } 
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

  // locals
  SmSolution sSols[4];
  SmSolutionArray sSolutions(4,sSols);
  SmBSplineCurve *pData[64];

  // For dropping points with GlobalPointSolve(): If the curve is on the surface,
  // then we put a limit on how far to allow a good drop, otherwise not.
  double *pdMaxDropDist = ( m_bCurveIsOnSurface ) ? &m_dThisApproxTol3d : NULL;


  // Copy TargetCurve knots into m_pCurveKnotVector
  if(m_pCurveKnotVector) { delete m_pCurveKnotVector ; m_pCurveKnotVector = NULL ; }
  m_pCurveKnotVector = new (*m_cpContext) SmTArray<double>(*m_cpContext);
  SE(m_crCurveToDrop.GetKnots(*m_pCurveKnotVector));

  // Place targetCurve interval start/end points into m_pCurveBreaks 
  if (m_pCurveBreaks) { delete m_pCurveBreaks;     m_pCurveBreaks     = NULL ; }
  m_pCurveBreaks = new (*m_cpContext) SmTArray<double>(*m_cpContext);
  sm_AddBreak(*m_pCurveBreaks,m_vInterval.GetMin());
  sm_AddBreak(*m_pCurveBreaks,m_vInterval.GetMax());

  // get Curve and Surface Caches
  SmSurfaceCache *pSC = smsurf_GetSurfaceCache(m_cpSurface); NER(pSC);
  SmCurveCache   *pCC = (SmCurveCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE,&m_crCurveToDrop); 
  NER(pCC);

  // select tolerance - from curve length or as given when Curve is on Surface 
  double dSize   = m_crCurveToDrop.ApproximateLength(m_vInterval,5);
  double dCCITol = (m_bCurveIsOnSurface)
                   ? this->m_dThisApproxTol3d
                   : dSize/100.0;

  // Add TargetSurface natural boundary curves to sIsoCurves array 
  SmBoolean bCurvesAreCached;
  SmTArray<SmBSplineCurve*> sIsoCurves(64,pData);
  SER(pSC->GetIsoBoundaryCurves(m_vUVDomain,dCCITol/3.0,bCurvesAreCached,sIsoCurves));

  // Automatic clean up of non-cached TargetSurface natural boundary curves
  SmTArray<SmBSplineCurve*> sTempCurves;
  if (!bCurvesAreCached) { sTempCurves.Append(sIsoCurves); }
  SmObjsDelete<SmBSplineCurve*> sCleanupCurves(&sTempCurves);

  // add every Surface U internal continuity boundary to sIsoCurves array
  const SmTArray<SmContinuityType> & rUConts = pSC->GetUContinuitiesArray();
  double adKnotData[256];
  SmTArray<double> sKnots(256,adKnotData);
  SER(m_cpSurface->GetKnots(SM_SP_U,sKnots));
  ULONG lNumConts = rUConts.GetSize();
  if (lNumConts > 0) 
    {
      for (ULONG ii=1; ii+1<lNumConts; ii++)  // note: can't say lNumConts-1
        {
          if (rUConts[ii] <= SM_CT_C0) 
            {
              SmBSplineCurve *pUIso = NULL ;
              SER(m_cpSurface->CreateIsoParametricCurve(*m_cpSurface->GetContext(),
                                                         SM_SP_U,
                                                         sKnots[ii],
                                                         dCCITol/3.0,
                                                         pUIso));
              sIsoCurves.Add(pUIso);
              sTempCurves.Add(pUIso) ;
            }
        }
    }

  // add every surface V internal continuity boundary to sIsoCurves array
  SER(m_cpSurface->GetKnots(SM_SP_V,sKnots));
  const SmTArray<SmContinuityType> & rVConts = pSC->GetVContinuitiesArray();
  lNumConts = rVConts.GetSize();
  if (lNumConts > 0) 
    {
    for (ULONG ii=1; ii+1<lNumConts; ii++)  // note: can't say lNumConts-1
      {
      if (rVConts[ii] <= SM_CT_C0) 
        {
          SmBSplineCurve *pVIso = NULL ;
          SER(m_cpSurface->CreateIsoParametricCurve(*m_cpSurface->GetContext(),
                                                     SM_SP_V,
                                                     sKnots[ii],
                                                     dCCITol/3.0,
                                                     pVIso));
          sIsoCurves.Add(pVIso);
          sTempCurves.Add(pVIso) ;
        }
      }
    }

  // Add every SurfaceIsoBoundary/TargetCurve intersection into m_pCurveBreaks array
  ULONG lNumIsos = sIsoCurves.GetSize();
  for (ULONG ii=0; ii<lNumIsos; ii++)
    {
      SmBSplineCurve *pIsoCurve = sIsoCurves[ii];
      if (pIsoCurve == NULL) { SM_ASSERT_ERR_MSG(_T("SmSurfaceDropCurve::FindBoundaryStartPoints: Encountered a pSurface->CreateIsoParametricCurve() output rpNewIsoCurve == NULL value. This is a bug in method CreateIsoParametricCurve()")) ; continue; }

      // when boundary curve is degenerate - intersect with point
      if (pIsoCurve->IsDegenerate()) 
        {
          SmPoint3d sPole;
          pIsoCurve->EvaluatePoint(pIsoCurve->GetNaturalInterval().GetMin(),sPole);
          SmBoolean bSuccess;
          double dParam, dDist;


          // Note: if the curve is not on the surface, then dropping the
          // degenerate-curve point to the curve is not correct.
          // For that case we would have to drop the pole point to the surface
          // (which will presumably be a singularity), get the surface normal
          // there, and intersect the curve with the line of the pole and normal.
          // Note that for a cone, getting the surface normal would mean finding
          // the proper direction for the singularity ... or, we would have to
          // essentially intersect the curve with the cone defined by the apex
          // and all of the surface normals there.
          // However, if the boundary curve is degenerate, then it must have
          // other boundary curves on either side of it, and hence its intersection
          // will be caught by those.  So call DropPoint with SM_SO_INTERSECT
          // to avoid an actual drop from off the curve.
          SER(m_crCurveToDrop.DropPoint(m_vInterval,        // in : target curve allowed domain
                                        sPole,              // in : Point to drop to curve
                                        NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                        dCCITol,            // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                        NULL,               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                        bSuccess,           // out: TRUE = found a drop point
                                        dParam,             // out: found drop curve param
                                        dDist,              // out: found drop distance
                                        SM_SO_INTERSECT)) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior

          if (bSuccess) 
            {
              sm_AddBreak(*m_pCurveBreaks,dParam);
            }
          continue;
        } // end degenerate surface boundary curve check

      // Call GlobalCurveIntersect() if the curve lies in the surface,
      // otherwise call GlobalCurveSolve() to do a projected intersection.
      if ( m_bCurveIsOnSurface )
        {
          // get surfaceBoundary/Curve intersections but no GlobalCoincidenceCheck
          SER(m_crCurveToDrop.GlobalCurveIntersect
                  (m_vInterval,                     // in : thisCurve's interval for intersection                         
                  *pIsoCurve,                       // in : target OtherCurve                                             
                   pIsoCurve->GetNaturalInterval(), // in : OtherCurve's interval for intersection                        
                   dCCITol,                         // in : max 3d distance between intersecting points                   
                   sSolutions,                      // out: Array of Found Solutions: sSol.m_vStart[0] = thisCurve param  
                                                    //                                sSol.m_vStart[1] = otherCurve param 
                   FALSE));                         // in : TRUE = Skip global coincidence check, default:[FALSE]                     

        }
      else // Curve might not be on the surface:
        {
          // Create SmCrvOnSurface.
          SmLine *pUVLine = NULL;
          SmPoint2d sUVMin = m_vUVDomain.GetMin();
          SmPoint2d sUVMax = m_vUVDomain.GetMax();
          SmPoint2d sPt0, sPt1;
          SmStatus eStat = SM_ERR;
          switch ( ii ) {
            case 0: {
              sPt0.Set( sUVMin.x, sUVMin.y ); sPt1.Set( sUVMin.x, sUVMax.y ); 
              eStat = SmLine::CreateLineSegment( *m_cpContext, 2, sPt0, sPt1, pUVLine );
              break;
            }
            case 1: {
              sPt0.Set( sUVMin.x, sUVMin.y ); sPt1.Set( sUVMax.x, sUVMin.y ); 
              eStat = SmLine::CreateLineSegment( *m_cpContext, 2, sPt0, sPt1, pUVLine );
              break;
            }
            case 2: {
              sPt0.Set( sUVMax.x, sUVMax.y ); sPt1.Set( sUVMax.x, sUVMin.y ); 
              eStat = SmLine::CreateLineSegment( *m_cpContext, 2, sPt0, sPt1, pUVLine );
              break;
            }
            case 3: {
              sPt0.Set( sUVMax.x, sUVMax.y ); sPt1.Set( sUVMin.x, sUVMax.y ); 
              eStat = SmLine::CreateLineSegment( *m_cpContext, 2, sPt0, sPt1, pUVLine );
              break;
            }
          } // end switch on which boundary

          if ( eStat != SM_SUCCESS ) { continue; }

          SmSurface *pNonConst = SM_CONST_CAST( SmSurface*, m_cpSurface );
          SmCrvOnSurf sSrfCrv( *pUVLine, *pNonConst, &m_vUVDomain, 0, m_cpContext );
          
          SER( m_crCurveToDrop.GlobalCurveSolve(
                   m_vInterval,
                   sSrfCrv,
                   sSrfCrv.GetNaturalInterval(),
                   SM_SO_SURFACE_PROJECTED_INTERSECT,
                   dCCITol,
                   NULL, NULL, SM_SR_ALL,
                   sSolutions ));

          // Any solutions returned will have sSrfCrv as their second m_apObject,
          // and that will be extinct in a moment.  We don't need that for
          // anything (other than debugging), so just null it out.
          ULONG jj, lNumSols = sSolutions.GetSize();
          for ( jj=0; jj<lNumSols; jj++ )
            { sSolutions[jj].m_apObjects[1] = NULL; }

          delete pUVLine; pUVLine = NULL;

        } // end if m_bCurveIsOnSurface

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          sSolutions.Dump() ; 

          SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
          SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;
          //SmTracePnt *pPnt1 = m_vStartPoints.GetFirstNode() ;
          //SmTracePnt *pI    = NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 0,1,0) ; m_crCurveToDrop.DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 1,0,1) ; pIsoCurve->Draw() ; sm_GraphicsLoop() ;
          if ( FALSE ) {
            smgfx_SetLook(4,6, 1,0,1) ; m_crCurveToDrop.DrawInspectTwoCurves(pIsoCurve) ; sm_GraphicsLoop() ;
          }
          smgfx_SetLook(5,8, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ; 
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
      
      // add every intersection point to the m_pCurveBreaks array
      for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) 
        {
          SmSolution & rSol = sSolutions[kk];

#ifdef SM_DEBUG_CODE
          // draw 
          if(bDebugMe)
            {
              rSol.Dump() ; 

              SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
              SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;
              //SmTracePnt *pPnt1 = m_vStartPoints.GetFirstNode() ;
              //SmTracePnt *pI    = NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 0,1,0) ; m_crCurveToDrop.DrawParams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,5, 1,0,1) ; pIsoCurve->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,6, 1,0,0) ; rSol.Draw() ; sm_GraphicsLoop() ; 
              sm_GraphicsLoop() ;

            }
#endif // SM_DEBUG_CODE
          SmBoolean  bContinue = FALSE;
          //SmVector3d sPVIso[2], sPVCrv[2];

          // look for coincident regions not connected to the CurveInterval endPoints
          if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES) 
            {
              // skip loose coincident solutions
              if (rSol.m_vStart.m_dSolutionValue > dCCITol / 100.00) 
                {
                  bContinue = TRUE;
                }
              else // look for coincident solutions not connected to curve endPoints  
                {
                  // when coincident solution is not connected to interval endPoint
                  // interval endPoints have already been placed on the m_pCurveBreaks arrays 
                  if(   smos_Fabs(rSol.m_vEnd[0]-m_vInterval.GetMin()) > SM_EFF_ZERO_SQRT/100.0
                     && smos_Fabs(rSol.m_vEnd[0]-m_vInterval.GetMax()) > SM_EFF_ZERO_SQRT/100.0) 
                    {
                      // GWC: always Add Range Solution endPoints 
                      sm_AddBreak(*m_pCurveBreaks,rSol.m_vEnd[0]);

                      // GWC:Replaced // when curves are not tangent at the end points
                      //              SmVector3d sPVIso[2], sPVCrv[2];
                      //              SER(m_crCurveToDrop.Evaluate(rSol.m_vEnd[0],1,TRUE,sPVCrv));
                      //              SER(pIsoCurve->Evaluate(rSol.m_vEnd[1],1,TRUE,sPVIso));
                      //              double dAngleRad;
                      //              SER(sPVCrv[1].AngleBetween(sPVIso[1],dAngleRad));
                      //              if (   dAngleRad > SM_DEG2RAD(2.0)
                      //                  && dAngleRad < SM_DEG2RAD(178.0) ) 
                      //                {
                      //                  // add the endPoint to the break array
                      //                  sm_AddBreak(*m_pCurveBreaks,rSol.m_vEnd[0]);
                      //                }
                    }
                } // end tight solution check
            } // end coincident interval solution check

          // skip intersections which start at interval endPoint
          // interval endPoints have already been added to the m_pCurveBreaks arrays
          if (smos_Fabs(rSol.m_vStart[0]-m_vInterval.GetMin()) < SM_EFF_ZERO_SQRT/100.0)
            { bContinue = TRUE; }
          if (smos_Fabs(rSol.m_vStart[0]-m_vInterval.GetMax()) < SM_EFF_ZERO_SQRT/100.0)
            { bContinue = TRUE; }

          // Skip grazing intersections.  [Reg_091216_S]
          if ( ! bContinue )
            {
              if ( sm_IsGrazingIntersection( &m_crCurveToDrop, rSol.m_vStart[0],
                                              pIsoCurve,       rSol.m_vStart[1] ) )
                { bContinue = TRUE; }
            }

          // when not skipping this solution
          if (!bContinue) 
            {
              // GWC: always Add Solution Start Points 
              sm_AddBreak(*m_pCurveBreaks,rSol.m_vStart[0]);

              // GWC:Replaced // Add intersection parameter to m_pCurveBreaks array
              //              // when boundaryCurve is not tangent to targetcurve 
              //              SER(m_crCurveToDrop.Evaluate(rSol.m_vStart[0],1,TRUE,sPVCrv));
              //              SER(pIsoCurve->Evaluate(rSol.m_vStart[1],1,TRUE,sPVIso));
              //              double dAngleRad;
              //              SER(sPVCrv[1].AngleBetween(sPVIso[1],dAngleRad));
              //              if (   dAngleRad > SM_DEG2RAD(2.0)
              //                  && dAngleRad < SM_DEG2RAD(178.0) ) 
              //                {
              //                  sm_AddBreak(*m_pCurveBreaks,rSol.m_vStart[0]);
              //                }

            } // end keep solution check
        } // end iter every intersection solution
    } // end intersecting every Surface NaturalBoundary with TargetCurve

  // Add every curve discontinuity point to breakpoints array
  SER(m_crCurveToDrop.GetKnots(sKnots));
  const SmTArray<SmContinuityType> & rConts = pCC->GetContinuitiesArray();
  if (rConts.GetSize() > 0) 
    {
      for (ULONG jj=1; jj+1<rConts.GetSize(); jj++)  // note: can't say rConts.GetSize()-1
        {
          if (   rConts[jj] <= SM_CT_C0
              && m_vInterval.ContainsValue(sKnots[jj])) 
            {
              sm_AddBreak(*m_pCurveBreaks,sKnots[jj]);
            }
        } // end iter every curve discontinuity
    } // end curve discontinuity point existence check

  // arrive here when m_pCurveBreaks array contains
  //   every SurfBoundary/TargetCurve intersection
  //   every SurfDiscontinuityIsoParameterLine/TargetCurve intersection
  //   CurveInterval Start/End points
  //   Curve discontinuity points 

#ifdef SM_DEBUG_CODE
  // draw Brep(blue), TargetSurface(cyan), Face(black), TargetCurve(green), m_vStartPoints(red)
  if(bDebugMe)
    {
      SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
      SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; m_crCurveToDrop.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,8, 1,0,1) ; if(m_pCurveBreaks)
                                    { for(ULONG ii=0;ii<m_pCurveBreaks->GetSize();ii++)
                                        {
                                          SmPoint3d sPoint ;
                                          m_crCurveToDrop.EvaluatePoint(m_pCurveBreaks->GetAt(ii),sPoint) ;
                                          sPoint.Draw() ; sm_GraphicsLoop() ;
                                    }   }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // for every CurveBreak interval (bounded by [ii-1,ii] BreakPoint pair)
  //    1. Find BreakPoints that start DropSegments
  //    2. when DropSegment drops to Surface IsoParameter Curve add segment to rSurfaceUVCurves. 
  //    3. else place BreakPoint into m_vStartPoints array.
  for (ULONG ii=1; ii<m_pCurveBreaks->GetSize(); ii++) 
    {
      // snap this and last breakpoint parameters to nearby curve knot values
      double dT0 = (*m_pCurveBreaks)[ii-1];
      double dT1 = (*m_pCurveBreaks)[ii];
      SER(m_crCurveToDrop.SnapToKnots(dT0,SM_EFF_ZERO_SQRT/100.0,dT0));
      SER(m_crCurveToDrop.SnapToKnots(dT1,SM_EFF_ZERO_SQRT/100.0,dT1));

      // Copy the snapped values back into the array:
      (*m_pCurveBreaks)[ii-1] = dT0;
      (*m_pCurveBreaks)[ii  ] = dT1;

      // skip intervals that have become zero length - no need for tolerance here
      if((*m_pCurveBreaks)[ii-1] == (*m_pCurveBreaks)[ii  ])
        { continue ; }
 
      // this interval
      SmExtent1d sIvl(dT0,dT1);

      // Find interval midPoint projection to Surface
      SmPoint3d sMid;
      SER(m_crCurveToDrop.EvaluatePoint(sIvl.Evaluate(0.5),sMid)); 
      SER(m_cpSurface->GlobalPointSolve
            (m_vUVDomain,             // in : Domain of surface to search for solutions
             SM_SO_NORMALIZE,         // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
             sMid,                    // in : Target point for the solve operation
             m_dThisApproxTol3d,      // in : Min dist between unique points for range solutions, max 3d-dist to be in surface domain
             pdMaxDropDist,           // in : Max/Min Drop distance for min/max and normalize operations.
                                      //      NULL to ignore.
             SM_SR_ALL,               // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
             sSolutions));            // out: array of problem solutions reported as surface UV parameter values

      // skip intervals whose midpoints don't drop to the surface
      if (sSolutions.GetSize() == 0) 
        { continue; }

//            // skip intervals whose midPoints are on boundary curves
//            //  assume interval is coincident with boundary
//            SmSolution & rSol = sSolutions[0];
//            SmPoint2d sPnt(rSol.m_vStart[0],rSol.m_vStart[1]);
//            if (m_vUVDomain.IsPoint2dOnBoundary(sPnt,SM_EFF_ZERO)) 
//              { continue; }

      // drop Interval StartPoint to Surface along surface normals
      m_vWorkingInterval = sIvl;
      SmPoint3d sStart;
      SER(m_crCurveToDrop.EvaluatePoint(sIvl.GetMin(),sStart));

      //double dDist = m_dThisApproxTol3d;
      SER(m_cpSurface->GlobalPointSolve      // eff: search surface for points whose normal vectors aim at the target point
              (m_vUVDomain,                  // in : Domain of surface to search for solutions
               SM_SO_INTERSECT,              // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
               sStart,                       // in : Target point for the solve operation
               m_dThisApproxTol3d,           // in : Min dist between unique points for range solutions, max 3d-dist to be in surface domain
               pdMaxDropDist,                // in : Max/Min Drop distance for min/max and normalize operations.
                                             //      NULL to ignore.
               SM_SR_ALL,                    // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
               sSolutions));                 // out: array of problem solutions reported as surface UV parameter values

      SmBoolean bAtSeam      = FALSE;
      SmBoolean bSurfUClosed = FALSE;
      SmBoolean bSurfVClosed = FALSE;

      // when StartPoint drops to exactly two surface points it may be on a seam
      if (sSolutions.GetSize() == 2) 
        {
          bSurfUClosed = m_cpSurface->IsClosed(m_vUVDomain,SM_SP_U);
          bSurfVClosed = m_cpSurface->IsClosed(m_vUVDomain,SM_SP_V);
          if ( bSurfUClosed || bSurfVClosed )
          { bAtSeam = TRUE; }
        } // end StartPoint dropped to two surface points check

      // when StartPoint dropped to more than 1 surface points and not a seam - we are probably on a surface singularity
      if (sSolutions.GetSize() > 1 && !bAtSeam) 
        {
          SER(m_crCurveToDrop.EvaluatePoint(sIvl.Evaluate(0.001),sStart));
          SER(m_cpSurface->GlobalPointSolve(m_vUVDomain,SM_SO_INTERSECT,sStart,
              m_dThisApproxTol3d, pdMaxDropDist,
              SM_SR_ALL,sSolutions));

          // snap solutions within 1% of interval boundaries to those boundaries
          SmExtent1d sDomainU = m_vUVDomain.GetUInterval();
          SmExtent1d sDomainV = m_vUVDomain.GetVInterval();
          for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) 
            {
              SmSolution & rSol = sSolutions[kk];
              double dUT = ( rSol.m_vStart[0] - sDomainU.GetMin() ) / sDomainU.GetLength();
              double dVT = ( rSol.m_vStart[1] - sDomainV.GetMin() ) / sDomainV.GetLength();

              if      (dUT < 0.01) { rSol.m_vStart[0] = sDomainU.GetMin(); }
              else if (dUT > 0.99) { rSol.m_vStart[0] = sDomainU.GetMax(); }
              if      (dVT < 0.01) { rSol.m_vStart[1] = sDomainV.GetMin(); }
              else if (dVT > 0.99) { rSol.m_vStart[1] = sDomainV.GetMax(); }

            } // end iter every solution snapping to interval boundaries
        } // when StartPoint dropped to more than two different surface points check

      // process all StartPoint drop points - treat multiple drop points on a seam as two curves
      SmPoint2d sUVLast;
      for (ULONG jj=0; jj<sSolutions.GetSize(); jj++) 
        {
          SmVector2d sUV;
          sUV.x = sSolutions[jj].m_vStart[0];
          sUV.y = sSolutions[jj].m_vStart[1];
          if (jj==0) 
            {
              sUVLast = sUV;
            }
          else // skip break points too close to last break point 
            {
              if (sUVLast.DistanceBetween(sUV) < SM_EFF_ZERO_SQRT*100.0) 
                {
                  continue; // Sorry points are too close no need to process
                            // separate curves.
                }
            }
          
          // If we start out at a singularity point then we need to step
          // off a little and try again.
          SmSurfParamType eSingularDirection;
          if (m_cpSurface->IsSingularity(sUV,eSingularDirection)) 
            {
              SER(m_crCurveToDrop.EvaluatePoint(sIvl.Evaluate(0.001),sStart));
              SmSolutionArray sTmpSolutions;
              SER(m_cpSurface->GlobalPointSolve(m_vUVDomain,SM_SO_MINIMIZE,sStart,SM_EFF_ZERO, pdMaxDropDist,
                  SM_SR_ALL,sTmpSolutions));
              if (sTmpSolutions.GetSize() > jj) 
                {
                  if (eSingularDirection == SM_SP_U) sUV.x = sTmpSolutions[jj].m_vStart[0];
                  if (eSingularDirection == SM_SP_V) sUV.y = sTmpSolutions[jj].m_vStart[1];
                }
              else if (sTmpSolutions.GetSize() > 0) 
                {
                  if (eSingularDirection == SM_SP_U) sUV.x = sTmpSolutions[0].m_vStart[0];
                  if (eSingularDirection == SM_SP_V) sUV.y = sTmpSolutions[0].m_vStart[1];
                }
              else 
                {
                  SER_MSG(SM_ERR, _T("SmSurfaceDropCurve::FindBoundaryPoints() - failed to recover from point dropping to surface singularity")); // we failed to resolve singularity problem
                }
            } // end StartPoint dropped to surface singularity point check
          
          // see if working curve interval drops to Surface isoParameterCurve with the same parameterization
          SmBoolean bSuccess;
          double dMaxDist=0.0, dDeviation=0.0;
          SmBSplineCurve *pIsoCurve = NULL;
          SmStatus eStat;
          SER(m_cpSurface->SnapToKnots(sUV,SM_EFF_ZERO_SQRT/100.0,sUV));
          SE (eStat = m_cpSurface->DropIsoCurve
                 (*m_cpContext,           // in : context for new object construction
                   m_vUVDomain,           // in : This Surface domain of interest
                   m_crCurveToDrop,       // in : Curve to drop
                   sIvl,                  // in : Curve interval of interest
                   sUV,                   // in : Surface UVPoint corresponding to start of3d curve.
                   m_dThisApproxTol3d,    // in : max allowed distance between drop point and surfNormal line at drop point
                   bSuccess,              // out: TRUE = dropped a curve
                   dMaxDist,              // out: DropDist = max sample distance found between cr3dCurve and Surface
                                          //      taken at cr3dCurve.NumberOfKnot evenly spaced samples
                   pIsoCurve,             // out: Pointer to newly allocated UVLine curve or
                                          //      NULL if no curve were constructed
                   &dDeviation));         // out: Max deviation from surface normal

          if (eStat != SM_SUCCESS )
          { continue; }

          // when interval drops to Surface isoParameterCurve
          if (bSuccess) 
            {
              ULONG kk = ii+1;

              // extend isoLine through as many breakPoints as possible
              while (kk < m_pCurveBreaks->GetSize()) 
                {
                  // extend interval to next breakPoint
                  SmBSplineCurve *pIsoCurve2 = NULL ;
                  SmExtent1d sIvl2(sIvl.GetMin(),(*m_pCurveBreaks)[kk]);
                  kk++;
                  double dMaxDist2 = 0.0, dDeviation2 = 0.0;

                  // see if extended interval drops to Surface isoParameterCurve
                  SER(m_cpSurface->DropIsoCurve(*m_cpContext,           // in : context for new object construction
                                                 m_vUVDomain,           // in : This Surface domain of interest
                                                 m_crCurveToDrop,       // in : Curve to drop 
                                                 sIvl2,                 // in : Curve interval of interest
                                                 sUV,                   // in : Surface UVPoint corresponding to start of3d curve.
                                                 m_dThisApproxTol3d,    // in : max allowed distance between drop point and surfNormal line at drop point 
                                                 bSuccess,              // out: TRUE = dropped a curve
                                                 dMaxDist2,             // out: DropDist = max sample distance found between cr3dCurve and Surface
                                                                        //      taken at cr3dCurve.NumberOfKnot evenly spaced samples
                                                 pIsoCurve2,            // out: Pointer to newly allocated UVLine curve or
                                                                        //      NULL if no curve were constructed
                                                 &dDeviation2 ));       // out: Max CurveToDrop smplPoint to DropSurfNormLine dist
                  // if it does replace old line with new and continue
                  if (bSuccess) 
                    {
                      SM_ASSERT(pIsoCurve != NULL) ; delete pIsoCurve ; pIsoCurve = NULL ;
                      pIsoCurve  = pIsoCurve2;
                      dMaxDist   = dMaxDist2;
                      dDeviation = dDeviation2;

                      // gwc:change need to increase breakPoint counter
                      ii++ ;
                    }
                  else // use old line and break
                    { break; }
                } // end extending isoLine through as many break points as possible

              // If there are more solutions to check, see if they're contained in this iso curve
              if ( sSolutions.GetSize() > jj + 1 )
              {
                  SmPoint3d sCurveStart, sCurveEnd;
                  SmExtent1d sIsoCurveIvl = pIsoCurve->GetNaturalInterval();
                  pIsoCurve->EvaluatePoint( sIsoCurveIvl.GetMin(), sCurveStart );
                  pIsoCurve->EvaluatePoint( sIsoCurveIvl.GetMax(), sCurveEnd );

                  SmExtent1d sIsoUIvl, sIsoVIvl;
                  sIsoUIvl.SetMinMax( smos_Min( sCurveStart[0], sCurveEnd[0] ), smos_Max( sCurveStart[0], sCurveEnd[0] ) );
                  sIsoVIvl.SetMinMax( smos_Min( sCurveStart[1], sCurveEnd[1] ), smos_Max( sCurveStart[1], sCurveEnd[1] ) );

                  // Remove other solutions that are on this curve
                  for ( kk = jj + 1; kk < sSolutions.GetSize(); kk++ )
                  {
                      double sNextU, sNextV;
                      sNextU = sSolutions[kk].m_vStart[0];
                      sNextV = sSolutions[kk].m_vStart[1];

                      // If the point drops to the curve, remove it from solutions to check
                      if ( sIsoUIvl.ContainsValue( sNextU ) && sIsoVIvl.ContainsValue( sNextV ) )
                      {
                          sSolutions.RemoveAt( kk );
                          kk--;
                      }
                  } // end removing solutions that are included in this isoparam curve
              } // end check for solutions contained in the iso curve

              // add IsoLine drop Curve to output
              rSurfaceUVCurves.Add( pIsoCurve  );
              rMaxDropDists   .Add( dMaxDist   );
              rDeviations     .Add( dDeviation );

              m_dMaxDistanceToSurface = smos_Max( m_dMaxDistanceToSurface, dMaxDist );
              continue;
            
            } // end interval dropped to isoCurve check
          
          // arrive here when interval StartPoint may be the start of a 
          // legitimate dropCurve segment.  Include point in m_vStartPoint
          // array when interval is determined to be running into the 
          // surface - skip points when the curve is running out of the surface.

          // drop point nearby interval startPoint to Surface
          SmPoint3d sNearStart ;
          SER(m_crCurveToDrop.EvaluatePoint(sIvl.Evaluate(0.001),sNearStart));
          SmSolutionArray sNearStartSolutions;
          SER(m_cpSurface->GlobalPointSolve(m_vUVDomain, 
                                            SM_SO_NORMALIZE,    
                                            sNearStart,
                                            SM_EFF_ZERO, 
                                            pdMaxDropDist,
                                            SM_SR_ALL, 
                                            sNearStartSolutions));

          // when NearStartPoint dropped to surface
          // build domain surfaceDomain containing StartPoint and NearStartPoint drop points
          if (sNearStartSolutions.GetSize() > 0) 
            {
              SmPoint2d sUVNearStart(sNearStartSolutions[0].m_vStart[0],
                                     sNearStartSolutions[0].m_vStart[1]);
              if(bAtSeam) 
                {
                  SmPoint2d sDomainSize = m_vUVDomain.GetSize();

                  // skip points that jump to other seam boundary
                  //if (   (bSurfUClosed && smos_Fabs(sUV.x-sUVNearStart.x) > 0.5*(sDomainSize.x))
                  //    || (bSurfVClosed && smos_Fabs(sUV.y-sUVNearStart.y) > 0.5*(sDomainSize.y))) 
                  //  {
                  //    continue;
                  //  }

                  // Jump UV point across the seam instead of skipping [B651]
                  if ( sNearStartSolutions.GetSize() > 1 )
                    {
                      SmBoolean bCheckUSeam = FALSE, bCheckVSeam = FALSE;

                      // Determine which seams we might want to jump
                      for ( ULONG kk = 1; kk < sNearStartSolutions.GetSize(); kk++ )
                        {
                          if( smos_Fabs(sNearStartSolutions[kk].m_vStart[0] - sNearStartSolutions[kk-1].m_vStart[0]) > SM_EFF_ZERO)
                          { bCheckUSeam = TRUE; }
                          if( smos_Fabs(sNearStartSolutions[kk].m_vStart[1] - sNearStartSolutions[kk-1].m_vStart[1]) > SM_EFF_ZERO)
                          { bCheckVSeam = TRUE; }
                        }

                      // Determine which seams this curve actually jumps
                      SmSolutionArray sNextStartSolutions;
                      SmBoolean bHave1Solution = FALSE;
                      double    dIters = 1.;
                      while ( dIters < 20 )
                        {
                          // Big jumps should be fine, just trying to find which side of the seam this interval runs on
                          SER( m_crCurveToDrop.EvaluatePoint( sIvl.Evaluate( 0.05 * dIters ), sNearStart ) );
                          SER( m_cpSurface->GlobalPointSolve( m_vUVDomain,
                                                              SM_SO_NORMALIZE,
                                                              sNearStart,
                                                              SM_EFF_ZERO,
                                                              pdMaxDropDist,
                                                              SM_SR_ALL,
                                                              sNextStartSolutions ) );

                          if (sNextStartSolutions.GetSize() == 1)
                          { 
                              bHave1Solution = TRUE; 
                              break;
                          }
                          if ( sNextStartSolutions.GetSize() == 0 )
                          {
                              SM_DBG_WARN( _T( "Dropping a curve across a seam, couldn't determine which way to cross seam." ) );
                              break;
                          }

                          dIters++;
                        }

                      if (!bHave1Solution )
                      { continue; }

                      // Jump the UV point across the seams as necessary
                      SmPoint2d sUVNextNearStart( sNextStartSolutions[0].m_vStart[0],
                                                  sNextStartSolutions[0].m_vStart[1] );
                      SmTArray< SmPoint2d> sNewUVs;
                      SmSurfParamType sSPU = SM_SP_U, sSPV = SM_SP_V;
                      if ( bCheckUSeam && bSurfUClosed && smos_Fabs( sUV.x - sUVNextNearStart.x ) > 0.5*( sDomainSize.x )
                           && m_cpSurface->IsOnSeam( sUV, NULL, &sSPU, &sNewUVs ) )
                      { sUV = sNewUVs[0]; }

                      if ( bCheckVSeam && bSurfVClosed && smos_Fabs( sUV.y - sUVNextNearStart.y ) > 0.5*( sDomainSize.y )
                           && m_cpSurface->IsOnSeam( sUV, NULL, &sSPV, &sNewUVs ) )
                      { sUV = sNewUVs[0]; }

                      // Update sUVNearStart to the correct solution, as determined by smallest l1 distance
                      double dDist = smos_Fabs( sUV[0] - sNearStart[0] ) + smos_Fabs( sUV[1] - sNearStart[1] );
                      for ( ULONG kk = 1; kk < sNearStartSolutions.GetSize(); kk++ )
                      {
                          SmPoint2d sUVStartTest( sNearStartSolutions[kk].m_vStart[0],
                                                  sNearStartSolutions[kk].m_vStart[1] );
                          double dTestDist = smos_Fabs( sUV[0] - sUVStartTest[0] ) + smos_Fabs( sUV[1] - sUVStartTest[1] );
                          if ( dDist > dTestDist )
                          {
                              dDist = dTestDist;
                              sUVNearStart = sUVStartTest;
                          }
                      }
                    }
                  else
                    {
                      SmTArray< SmPoint2d> sNewUVs;
                      SmSurfParamType sSPU = SM_SP_U, sSPV = SM_SP_V;
                      if ( bSurfUClosed && smos_Fabs( sUV.x - sUVNearStart.x ) > 0.5*( sDomainSize.x )
                           && m_cpSurface->IsOnSeam( sUV, NULL, &sSPU, &sNewUVs ) )
                      { sUV = sNewUVs[0]; }

                      if ( bSurfVClosed && smos_Fabs( sUV.y - sUVNearStart.y ) > 0.5*( sDomainSize.y )
                           && m_cpSurface->IsOnSeam( sUV, NULL, &sSPV, &sNewUVs ) )
                      { sUV = sNewUVs[0]; }
                    }

                }
              SmExtent2d sDom(sUVNearStart);  // Interval NearStartPoint drop to surface loc
              sDom.AddPoint2d(sUV);           // Interval StartPoint drop to surface loc
              m_vWorkingUVDomain = sDom;
            }

          // When startPoint and NearStartPoints drop to different Surface Points
          //   add point to m_vStartPoints array 
          // Otherwise, skip the point
          
          // skip segments whose NearStartPoint don't drop into the surface or
          // whose StartPoint and NearStartPoint drop to the same point.
          // When that happens here we expect a short outside segment to 
          //   be projecting to the natural boundary.  
          // Note: Curves that drop to isoParameter lines with the same parameterization have already been found.
          //       When the parameterization is different, the UVTrimCurve still needs to be dropped.
          if (   sNearStartSolutions.GetSize() == 0
              || (    m_vWorkingUVDomain.XLength() < SM_EFF_ZERO
                   && m_vWorkingUVDomain.YLength() < SM_EFF_ZERO))
             // GWC:removed:   || m_vWorkingUVDomain.XLength() < SM_EFF_ZERO
             // GWC:removed:   || m_vWorkingUVDomain.YLength() < SM_EFF_ZERO)
            {
              continue ;
            }  
          else // add point to m_vStartPoints array
            {
              // make intervalStartPoint a TracePoint 
              SmTracePnt *pTSP = (SmTracePnt*)m_vTrcPntMgr.GetNewElement();
              pTSP->m_dCurveParameter = sIvl.GetMin();
              SER(ComputePointValues(sUV,*pTSP));
              pTSP->m_ePointType = SM_TP_NORMAL;

              // add unique pTSP points to m_vStartPoints array 
              AddStartPoint(*pTSP);

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
                  SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;

                  // draw Brep(blue), TargetSurface(cyan), Face(black), TargetCurve(green), m_vStartPoints(red)
                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 0,1,0) ; m_crCurveToDrop.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,5, 1,0,0) ; pTSP->Draw(m_cpSurface->GetContext()) ; sm_GraphicsLoop() ; 
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE
            } // end start and near StartPoints project to different surfacePoints check
        } // end iter all startPoint drop to Surface solutions
    } // end iter projecting every curve BreakPoint to Surface

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
          SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;
          SmTracePnt *pPnt1 = m_vStartPoints.GetFirstNode() ;
          SmTracePnt *pI    = NULL ;
          ULONG       ii ;

          // draw Brep(blue), TargetSurface(cyan), Face(black), TargetCurve(green), m_vStartPoints(red)
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(3,6, 1,0,1) ; m_crCurveToDrop.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,8, 1,0,0) ; for(ii=0, pI=pPnt1; 
                                          pI && (ii == 0 || pI != pPnt1);
                                          ii++, pI = m_vStartPoints.GetNextNode(pI))
                                        {
                                          pI->Draw(m_cpSurface->GetContext()) ; sm_GraphicsLoop() ;
                                        } 
          smgfx_SetLook(4,6, 1,0,1) ; for(ii=0; ii < rSurfaceUVCurves.GetSize(); ii++)
                                        {
                                          SmCrvOnSurf sCrv(*rSurfaceUVCurves[ii], (SmSurface &)*m_cpSurface) ;
                                          sCrv.Draw() ; sm_GraphicsLoop() ;
                                        }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::FindBoundaryStartPoints

/*******************************************************************//**
PURPOSE: This method is the top level interface function for various
   tracing routines.

NOTES: This method works for surfaces which are C0.  It does
   this by first intersecting the curve to be dropped with the curves
   on the surface where there is a discontinuity and boundary curves.
   Therefore the original curve must lie on the surface near these 
   discontinuities for it to work correctly.  
***********************************************************************/
SmStatus SmSurfaceDropCurve::DoTrace
  (const SmContext     & crContext,          // in : context for new object construction 
   const SmApproxTol3d * pdOptApproxTol3d,   // in : max allowed deviation of TrimCurve from ideal Projection, NULL for m_dThisApproxTol3d
   const double        * pdOptAngTolRad,     // in : 
   SmTArray<SmCurve*>  * p3DCurves,          // out: Surface Curves - not always a 3d curve for every UVDropCurve
   SmTArray<SmCurve*>  * pSurfaceUVCurves,   // out: UVDropCurves (more than 1 if projected to seam or in/out of boundary)
   SmTArray<double>    * pMaxDropDists,      // out: max 3dCurveSmpPoint to DropSurfPoint dist for each p3DCurves
   SmTArray<double>    * pDeviations)        // out: max 3dCurveSmpPoint to DropSurfNormLine dist for each p3DCurves
{
  // set object values from input
  m_cpContext = &crContext;
  if (pdOptApproxTol3d) { m_dThisApproxTol3d = *pdOptApproxTol3d; }
  if (pdOptAngTolRad)          { m_dThisAngTolRad   = *pdOptAngTolRad; }

  // get and checkout surface cache
  SmSurfaceCache *pSC1 = smsurf_GetSurfaceCache(m_cpSurface); NER(pSC1);
  SmCacheCheckOutIn sCheckIO(pSC1);

  // temporarily Turn off point testing (Keep point solutions without checking trim boundaries)
  // temporarilyTurn on boundary curve testing (force drop points onto natural boundary curves)
    { 
      // Turn off point testing so GlobalPointSolve() will keep all point solutions
      //   without classifying the solution point against the trim boundaries.
      // Turn on boundary curve processing to force LocalSolve to look for drop points
      //   on boundary curves.
      SmTemporaryChangeValue<SmBoolean> sStack1(pSC1->m_bPointTestEnabled,FALSE);
      SmTemporaryChangeValue<SmBoolean> sStack3(pSC1->m_bProcessBoundaryCurves,TRUE);

      // locals
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmCurve*> sSurfaceUVCurves;
      SmTArray<double>   sMaxDropDists;
      SmTArray<double>   sDeviations;
  
      SmObjsDelete<SmCurve*> sCleanup3D(&s3DCurves);
      SmObjsDelete<SmCurve*> sCleanupUV(&sSurfaceUVCurves);

      // 1. Place startPoints into m_vStartPoints.
      // 2. Place Curve Segments that drop to Surface IsoParameter Curves into sSurfaceUVCurves.
      //    note: StartPoints are BreakPoints starting a dropSegment (other than the isoParameter ones). 
      //      BreakPoints include:   
      //         a. Curve Start/End points,
      //         b. Curve discontinuity points,
      //         c. Curve/SurfaceBoundary Intersection points, and
      //         d. Curve/SurfaceDiscontinuity IsoParameterLine Intersection points.
      SER(FindBoundaryStartPoints(s3DCurves,        // out: Not Used - drops m_crCurveToDrop - does not gen any 3d curves 
                                  sSurfaceUVCurves, // out: all curve intervals that drop to SurfaceIsoCurves  
                                  sMaxDropDists,    // out: max m_crCurveToDrop SmpPoint to SurfDropPt dist for every output curve  
                                  sDeviations));    // out: max m_crCurveToDrop SmpPoint to DropSurfNormLine dist for every output curve 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(m_cpSurface) ;

          SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
          SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;
          SmTracePnt *pPnt1 = m_vStartPoints.GetFirstNode() ;
          SmTracePnt *pI    = NULL ;
          ULONG       ii ;

          // draw Brep(blue), TargetSurface(cyan), Face(black), TargetCurve(green), m_vStartPoints(red)
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) { pFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(3,6, 1,0,1) ; m_crCurveToDrop.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,8, 1,0,0) ; for(ii=0, pI=pPnt1; 
                                          pI && (ii == 0 || pI != pPnt1);
                                          ii++, pI = m_vStartPoints.GetNextNode(pI))
                                        {
                                          pI->Draw(m_cpSurface->GetContext()) ; sm_GraphicsLoop() ;
                                        } 
          smgfx_SetLook(6,7, 1,0,1) ; for(ii=0; ii<sSurfaceUVCurves.GetSize(); ii++)
                                        {
                                          SmCrvOnSurf sCrv(*sSurfaceUVCurves[ii], (SmSurface &)*m_cpSurface) ;
                                          sCrv.Draw() ; sm_GraphicsLoop() ;
                                        }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Now trace a dropCurve starting at each point listed in m_vStartPoints
      SmTArray<SmTracePnt*> sActiveElem;
      m_vStartPoints.GetAllNodes(sActiveElem);
      for (ULONG i=0; i<sActiveElem.GetSize(); i++) 
        {
          SmTracePnt *pTSP = sActiveElem[i];
          NER(pTSP);
          m_dCurveTraceDirection = 1.0;

          // trace dropCurve starting at pTSP and add
          // results to s3DCurves, sSurfaceUVCurves, sMaxDropDists, and sDeviations
          SER(TraceCurve2(*pTSP,              // in : current start point
                           s3DCurves,         // out: not used when DropCurve works - 
                           sSurfaceUVCurves,  // out: UVDropCurve (more than 1 when dropping onto a seam or in/out of a boundary)
                           sMaxDropDists,     // out: associated max drop distances
                           sDeviations));     // out: associated deviation from normal
        }

      // arrive here after getting all dropCurve segments that start on curve break points

      // trace dropCurve segments that don't start on a BreakPoint
      // and add results to s3DCurves, sSurfaceUVCurves, sMaxDropDists, and sDeviations
      SER(FindInteriorCurves(s3DCurves,
                             sSurfaceUVCurves,
                             sMaxDropDists,
                             sDeviations));

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          for (ULONG jjj=0; jjj<s3DCurves.GetSize(); jjj++) 
            {
              SmCurve *pCurve = s3DCurves[jjj];
              pCurve->Dump();
              sm_GraphicsLoop();
              pCurve->DrawWDeriv(pCurve->GetNaturalInterval(),0);
              sm_GraphicsLoop();
            }
        }
#endif // SM_DEBUG_CODE

      // set outputs
      if (p3DCurves)        { p3DCurves->Append(s3DCurves);
                              sCleanup3D.Clear();
                            }
      if (pSurfaceUVCurves) { pSurfaceUVCurves->Append(sSurfaceUVCurves);
                              sCleanupUV.Clear();
                            }
      if (pMaxDropDists)    { pMaxDropDists->Append(sMaxDropDists);
                            }
      if (pDeviations)      { pDeviations->Append(sDeviations);
                            }
    
    } // end scope block for SurfaceCache temporary changes

  // all done
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::DoTrace

/*******************************************************************//**
PURPOSE: Trace a curve from the given starting point stopping either
    at a boundary or when it becomes a closed loop.

NOTES: 
  m_pCurveBreaks must be set prior to this call.
    When m_pCurveBreaks is empty - no Curves are output
  
  The curve is dropped over the interval
    [rStartPoint->m_dCurveParameter, 1st m_pCurveBreak value greater than StartParameter]
      
***********************************************************************/
SmStatus SmSurfaceDropCurve::TraceCurve2
  (SmTracePnt         & rStartPoint,      // in : 
   SmTArray<SmCurve*> & r3DCurves,        // out: used when dropCurve walks in/out of a boundary and SmSurfaceTracer::TraceCurve() is called
   SmTArray<SmCurve*> & rSurfaceUVCurves, // out: UVDropCurves (more than 1 when dropping to a seam or wlaking in/out of a boundary)
   SmTArray<double>   & rMaxDropDists,    // out: max drop distances
   SmTArray<double>   & rDeviations)      // out: deviations from normal
{
  m_vCurvePoints.Append(&rStartPoint);
  SmTracePnt *pStartTSP = &rStartPoint;

  // If there are curve breaks - replace dMaxT with first break after dMinT.
  for (ULONG i=0; i<m_pCurveBreaks->GetSize(); i++) 
    {
      double dMinT = pStartTSP->m_dCurveParameter;
      double dMaxT = (*m_pCurveBreaks)[i];

      // skip breaks at the start dMinT param value
      if(dMaxT < dMinT + SM_EFF_ZERO)
        { continue; }

      // let WorkingDomain = Surface G1 continuous SubDomain that contains the StartPoint
      m_vWorkingUVDomain = m_cpSurface->GetDomainWithContinuity(SM_CT_G1,
                                                                pStartTSP->m_vUVCurvePV[0],
                                                                pStartTSP->m_vUVCurvePV[1]);

      // exit case - working domain does not overlap the domain of interest
      if (m_vUVDomain.AreDisjoint(m_vWorkingUVDomain)) 
        { return SM_SUCCESS; }

      // limit the working domain to the domain of interest
      SER(m_vUVDomain.Intersect(m_vWorkingUVDomain, m_vWorkingUVDomain));
      
      // curve working interval
      m_vWorkingInterval = SmExtent1d(dMinT,dMaxT);

      // DropCurve locals
      double dMaxDistToSurface = 0.0, dTheoreticalDeviation = 0.0;
      SmTArray<SmBSplineCurve*> sUVCurves;
      SmObjsDelete<SmBSplineCurve*> sDelUV(&sUVCurves);

      // Drop Curve WorkingInterval to Surface WorkingUVDomain to AppoxTol 
      SmStatus sRtn = m_cpSurface->DropCurve(*m_cpContext,
                                              m_vWorkingUVDomain,
                                              m_crCurveToDrop,
                                              m_vWorkingInterval,
                                              m_dThisApproxTol3d,
                                              dMaxDistToSurface,      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                              dTheoreticalDeviation,  // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                              sUVCurves,
                                              !m_bCurveIsOnSurface); // Maybe reject off-surface curves.

      // avoid re-delete: The UV curves were already deleted in DropCurve if not successful
      if (sRtn != SM_SUCCESS) 
        { sDelUV.Clear(); }   
          
      // Look for seam case: remove one of the two seam curves.
      //cbi Really?  Don't we return both?
      if(sRtn == SM_SUCCESS && sUVCurves.GetSize() == 2) 
        {
          // Seam case let's find the one that starts closest to rStartPoint.
          SmBSplineCurve *pCrv1 = sUVCurves[0];
          SmBSplineCurve *pCrv2 = sUVCurves[1];
          SmPoint3d sPnt1, sPnt2;
          pCrv1->EvaluatePoint(pCrv1->GetNaturalInterval().GetMin(),sPnt1);
          pCrv2->EvaluatePoint(pCrv2->GetNaturalInterval().GetMin(),sPnt2);

          // when 1st solution is closer to StartPoint - remove last solution
          if (sPnt1.DistanceBetween(rStartPoint.m_vUVCurvePV[0]) < 
              sPnt2.DistanceBetween(rStartPoint.m_vUVCurvePV[0]) ) 
            {
              SM_ASSERT(sUVCurves.GetLast() != NULL) ; delete sUVCurves.GetLast() ; 
              sUVCurves.RemoveLast();
            }
          else // last solution is closer to StartPoint - remove first solution
            {
              SM_ASSERT(sUVCurves[0] != NULL) ; delete sUVCurves[0] ; sUVCurves[0] = NULL ;
              sUVCurves.RemoveAt(0,1);
            }
        } // end seam case check

      // when there was just one solution or we are now down to one solution - put it in output and exit
      if (sRtn == SM_SUCCESS && sUVCurves.GetSize() == 1) 
        {
          // set output
          rSurfaceUVCurves.Add(sUVCurves[0]);
          sUVCurves.RemoveAt(0,1);
          rMaxDropDists.Add( dMaxDistToSurface );
          rDeviations.Add( dTheoreticalDeviation );
//          // RMB This StartPoint was successful so delete it from CurvePoints
//          // This will insure that the last StartPoint is not used by FlushCurve
//          // below to build another crv seg back to the previous StartPoint
//          m_vCurvePoints.RemoveLast();
        }
      else // more than one solution - probably walking in/out of a boundary
        {
          // Call SmSurfaceTracer algorithm: (need to document what this does that DropCurve doesn't)
          if (SmSurfaceTracer::TraceCurve(rStartPoint,r3DCurves) == SM_SUCCESS) 
            {
              SER(FlushCurve( r3DCurves, rSurfaceUVCurves, rMaxDropDists, rDeviations, &m_crCurveToDrop ));
            }
        }

      // all done
      return SM_SUCCESS;
    } // end iter all curve breaks

  // all done
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::TraceCurve2

/*******************************************************************//**
PURPOSE: Determine if the point is on a curve.

NOTES: NOTE - always FALSE for Drop Curve
***********************************************************************/
SmBoolean SmSurfaceDropCurve::IsPointOnCurve
  (const SmPoint3d &,           // crPointToTest,
   const SmTArray<SmCurve*> &)  // cr3DCurves) 
 const
{
    return FALSE;

} // end SmSurfaceDropCurve::IsPointOnCurve


/*******************************************************************//**
PURPOSE: Compute the next point in the trace stepping algorithm.
    This algorithm will compute the point based on the step size and return
    a boolean indicating if the step size satisfies all of our stepping 
    criteria.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceDropCurve::ComputeNextPoint
 (SmTracePnt & rTracePnt,         // in : Curr Trace point
  double       dStepSize,         // in : Param StepSize for Curve being dropped
  SmTracePnt & rNextPnt,          // out: Next TracePnt = Dropped(Curve(TracePnt.m_dCurveParameter + dStepSize)
  SmBoolean  & rbFoundGoodPoint,  // out: TRUE = 
  double     & rdDeviationFound,  // out: distance from NextPnt3d to Surface
  double     & rdAngleFoundRad,   // out:
  SmBoolean  & rbClipped,         // out: TRUE = TracePnt.m_dCurveParameter + dStepSize > m_vWorkingInterval.GetMax
  SmBoolean  & rbBoundaryHit)     // out: TRUE = TracePnt on m_vUVDomain boundary and
                                  //       TracePnt.m_vUVCurvePnt + dStepSize * TracePnt.m_vUVCurveTangent is out of m_vUVDomain 
{
  // exit case: if end of a trace sequence is on a singularity - then stop
  if(   rTracePnt.m_ePointType == SM_TP_SINGULARITY 
     && m_vCurvePoints.GetFirstNode() != &rTracePnt) 
    {
      rbClipped        = TRUE;
      rbBoundaryHit    = TRUE;
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // Init output
  rbClipped             = FALSE;
  rbBoundaryHit         = FALSE;
  rNextPnt.m_ePointType = SM_TP_NORMAL;

  // locals
  SmBoolean bHitEnd      = FALSE;
  double    dTol         = SM_EFF_ZERO * (1.0 + m_vInterval.GetLength());
  double    dOldStepSize = dStepSize;
  
  // remember when dStepSize steps to or beyond 3dCurve m_vWorkingInterval.GetMax() 
  if (smos_Fabs(rTracePnt.m_dCurveParameter+dStepSize - m_vWorkingInterval.GetMax()) < dTol) 
    {
      bHitEnd   = TRUE;
      rbClipped = TRUE;
    }

  // when StepSize does not hit the end of the 3dCurve m_vWorkingInterval - see if StepSize hits Surface m_vUVDomain boundary
  if (!bHitEnd) 
    {
      // 1st pass: use first derivative of param space curves to get start point. 
      dStepSize = m_vUVDomain.ClipLine2d(rTracePnt.m_vUVCurvePV[0],
                                         rTracePnt.m_vUVCurvePV[1],
                                         dStepSize);
        
      // Try to straighten the vectors out if we are right on the
      // boundary and get a clip.  Perhaps it is pointing out just
      // a little bit and we can straighten it to go along the boundary.
        
      dTol = smos_Max(SM_EFF_ZERO_SQRT,dStepSize/100.0);
      
      // When StepSize was clipped by m_vUVDomain and TracePnt is on the boundary  
      if(   dStepSize < dOldStepSize 
         && m_vUVDomain.IsPoint2dOnBoundary(rTracePnt.m_vUVCurvePV[0],dTol)) 
        {
          SmVector2d sVec = rTracePnt.m_vUVCurvePV[1];

          // when TracePnt tangent is mostly in the v direction
          if(smos_Fabs(sVec.x)*100.0 < smos_Fabs(sVec.y)) 
            {
              // Let Tangent = v dir and reClip to m_vUVDomain 
              rTracePnt.m_vUVCurvePV[1].x = 0.0;
              dStepSize = m_vUVDomain.ClipLine2d(rTracePnt.m_vUVCurvePV[0],
                                                 rTracePnt.m_vUVCurvePV[1],
                                                 dOldStepSize);
            } // end TracePnt Tangent is mostly a v dir vector check
          else if (smos_Fabs(sVec.y)*100.0 < smos_Fabs(sVec.x)) 
            {
              // else TracePnt tangent is mostly in the u direction

              // Let Tangent = u dir and reClip to m_vUVDomain
              rTracePnt.m_vUVCurvePV[1].y = 0.0;
              dStepSize = m_vUVDomain.ClipLine2d(rTracePnt.m_vUVCurvePV[0],
                                                 rTracePnt.m_vUVCurvePV[1],
                                                 dOldStepSize);
            } // end TracePnt Tangent is mostly a u dir vector check
        } // end StepSize was clipped by m_vUVDomain and TracePnt is on the boundary check 
      
      // init output  
      rbClipped     = FALSE; // gwc: this looks like a bug - should this be moved below the upcoming exit check?
      rbBoundaryHit = FALSE;

      // exit case - StepSize clipped to zero length - last point is on boundary - no good next point found
      if (SM_IS_ZERO(dStepSize)) 
        {
          rbFoundGoodPoint = FALSE;
          rbBoundaryHit    = TRUE;
          return SM_SUCCESS;
        }
      
      // when StepSize is clipped by more than tolerance  
      if (dStepSize < dOldStepSize - SM_EFF_ZERO * dOldStepSize) 
        {
          // and TracePnt is on the boundary of m_vUVDomain
          if (!m_vUVDomain.IsPoint2dOnBoundary(rTracePnt.m_vUVCurvePV[0])) 
            {
              // remember that StepSize was clipped
              rbClipped = TRUE;
            }
          else // else - restore StepSize back to input StepSize - not a bug next step clamps next UVPnt to UVDomain
            {
              dStepSize = dOldStepSize;
            }
        } // end StepSize was clipped check
    } // end of StepSize does not hit the end of the 3dCurve m_vWorkingInterval check

  // Clamp sUV0 = TracePnt + StepSize * TracePnt.UVTangent to m_vUVDomain
  SmVector2d sUV0 = rTracePnt.m_vUVCurvePV[0] + dStepSize * rTracePnt.m_vUVCurvePV[1];
  sUV0 = m_vUVDomain.ClampPoint2d(sUV0);

  // locals
  SmVector2d sUV;
  SmSolution sSol;
  SmBoolean  bFoundAnswer = FALSE;
    
  // if dStepSize shortened a lot - reset input StepSize - computes next TracePnt param value
  if (dOldStepSize > dStepSize * 1.2) 
    {
      dOldStepSize = dStepSize;
    }

  // Set NextTracePnt.CurveParam = TracePnt.CurveParam + OldStepSize
  rNextPnt.m_dCurveParameter = rTracePnt.m_dCurveParameter + dOldStepSize;

  // arrive here when: sUV0 = Good UVPnt guess for dropping Next CurvePnt to Surface

  // Now that we have guess for the next point - do local iteration
  SER(LocalPointSolve(rTracePnt,    // in : curr TracePnt
                      sUV0,         // in : UVPnt guess for next TracePnt drop
                      rNextPnt,     // i/o: in : m_dCurveParameter
                                    //      out: m_vUVCurvePV        = guess for NextPntUV
                                    //           m_dSurfDropDist     = 3d dist from 3dPt to surface in SurfNormLine dir
                                    //           m_dSurfNormLineDist = 3d Dist from 3dPt to SurfNormLine
                      bFoundAnswer, // out: TRUE = NextPnt3d less than m_dThisApproxTol3d from 
                                    //             SurfNormLine starting at NextPnt.SurfacePnt
                      sUV));        // out: UVPnt found by NR iteration 

  // exit case: LocalPointSolve did not find a good NextPnt
  if (!bFoundAnswer) 
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

//  double d2DDist = sUV.DistanceBetween(sUV0);

  // fill out the rest of the rNextPnt member values
  SER(ComputePointValues(sUV,          // in : target SurfacePoint
                         rNextPnt,     // i/o: target CurvePoint, set with SurfacePoint properties
                         &rTracePnt)); // in : Last intersection point on curve being stepped out, NULL to ignore
                                       //      When supplied used to handle singularity cases.
                                       //      default:[NULL]

  // bad case: when UV Step to NextPnt goes to zero - report no good sol found
  SmVector2d sDiff0    = rNextPnt.m_vUVCurvePV[0] - rTracePnt.m_vUVCurvePV[0];
  SmVector2d sDiff1    = rTracePnt.m_vUVCurvePV[0] - sUV;
  double     dLengDiff = sDiff0.Length();
  double     dLengTan  = sDiff1.Length();

  if (dLengDiff < SM_EFF_ZERO || dLengTan < SM_EFF_ZERO) 
    {
      rbFoundGoodPoint = FALSE;
      rbBoundaryHit    = TRUE;
      return SM_SUCCESS;
    }

  // Now that we have a point - backfit step size because it may have been clipped

  // bad case: when NextStep turning angle gets too large - report no good sol found
  SER(rTracePnt.m_v3DCurvePV[1].AngleBetween(rNextPnt.m_v3DCurvePV[1],rdAngleFoundRad));
  if(rdAngleFoundRad > m_dThisAngTolRad)
    {
      if(dStepSize > SM_EFF_ZERO*100.0) // gwc: why have this minimum size requirement
        {
          rbFoundGoodPoint = FALSE;
          return SM_SUCCESS;
        }
#ifdef SM_DEBUG_CODE
      else // a place for a break point to see if this case ever comes up
        {
          SM_DBG_WARN(_T("SmSurfaceDropCurve::ComputeNextPoint - found a very small step with big turning angle that it thinks is okay - should be checked")) ;
        }
#endif // SM_DEBUG_CODE
    } // end big turning angle check

  // bad case: when TracePnt UVCurveTangent has zero length - report no good sol found
  if (rTracePnt.m_vUVCurvePV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // bad case: NextPnt.UVPnt doubles back on line(TracePnt.UVPnt, TracePnt.UVTan)
  //           This is also a violation of the angle tolerance.
  double dLineParam;
  SER(smgu_LineClosestPoint(rTracePnt.m_vUVCurvePV[0],
                            rTracePnt.m_vUVCurvePV[1],
                            rNextPnt.m_vUVCurvePV[0],
                            dLineParam));
  if (dLineParam < 0.0) 
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // Here we need to call a method to compute the span's deviation from 
  // the actual curve.  This requires knowledge of the operation.
  SER(ComputeSpanDeviation(rTracePnt,
                           rNextPnt,
                           dOldStepSize,
                           rdDeviationFound,
                           rbFoundGoodPoint));

  // exit case: If we have a singularity and it satisfies tolerance than we should stop here
  if (rNextPnt.m_ePointType == SM_TP_SINGULARITY) 
    {
      rbClipped = TRUE;
    }

  // found a good sol point - all done
  return SM_SUCCESS;

} // end SmSurfaceDropCurve::ComputeNextPoint

/*******************************************************************//**
PURPOSE: Test the current point to see if it is near an existing 
   start point or the start of itself (i.e. curve is closed).

NOTES: NOTE - not needed in drop curve
***********************************************************************/
SmStatus SmSurfaceDropCurve::TestPoint
  (SmTracePnt &,         // in : Point to test
   SmBoolean  & rbDone)  // out: FALSE = Not Done tracing Curve,
{
    rbDone = FALSE;
    return SM_SUCCESS;

} // end SmSurfaceDropCurve::TestPoint

/*******************************************************************//**
PURPOSE: Compute the deviation of the span given by a Hermite with
    the actual theoretical curve.

NOTES: Note that this is pure virtual method that must be
    implemented by the subclasses of SmSurfaceTracer.
***********************************************************************/
SmStatus SmSurfaceDropCurve::ComputeSpanDeviation
  (const SmTracePnt & crCurrPnt,
   const SmTracePnt & crNextPnt,
   double dStepSize,
   double & rdDeviationFound,
   SmBoolean & rbSatisfiesTolerances)
{
    // Create two parameter space Hermite curves and test some points
    // on the curves as projected into 3d by their corresponding surfaces.
    SmHermiteCurve sHermiteUV(crCurrPnt.m_vUVCurvePV[0],
        crCurrPnt.m_vUVCurvePV[1]*dStepSize,
        crNextPnt.m_vUVCurvePV[0],
        crNextPnt.m_vUVCurvePV[1]*dStepSize,
        3);
    sHermiteUV.SetContext(NULL);

//    SmHermiteCurve sHermite3D(crCurrPnt.m_v3DCurvePV[0],
//        crCurrPnt.m_v3DCurvePV[1]*dStepSize,
//        crNextPnt.m_v3DCurvePV[0],
//        crNextPnt.m_v3DCurvePV[1]*dStepSize,
//        3);


#define NUM_TEST_POINTS 5
    double dMaxDist = 0.0;
    double dTolSq = m_dThisApproxTol3d * m_dThisApproxTol3d;
    for (ULONG i=0; i<NUM_TEST_POINTS; i++) {
        double dParam = (i+1.0) / (NUM_TEST_POINTS+1.0);
        SmPoint3d sCrvPnt;
        SER(sHermiteUV.EvaluatePoint(dParam,sCrvPnt));
        SmPoint2d sUV(sCrvPnt.x,sCrvPnt.y);
        sUV = m_vUVDomain.ClampPoint2d(sUV);
        SER(m_cpSurface->EvaluatePoint(sUV,sCrvPnt));

        SmPoint2d sUVFound;
        SmBoolean bFoundAnswer;
        SmTracePnt sDummyTP;
        sDummyTP.m_dCurveParameter = crCurrPnt.m_dCurveParameter + dParam * dStepSize;
        SER(LocalPointSolve(crCurrPnt,    // in : curr TracePnt
                            sUV,          // in : UVPnt guess for next TracePnt drop
                            sDummyTP,     // i/o: in : m_dCurveParameter
                                          //      out: m_vUVCurvePV        = guess for NextPntUV
                                          //           m_dSurfDropDist     = 3d dist from 3dPt to surface in SurfNormLine dir
                                          //           m_dSurfNormLineDist = 3d Dist from 3dPt to SurfNormLine
                                          //           m_dDeviation = Drop dist from NextPnt3d to SurfNormLine
                            bFoundAnswer, // out: TRUE = NextPnt3d less than m_dThisApproxTol3d from 
                                          //             SurfNormLine starting at NextPnt.SurfacePnt
                            sUVFound));   // out: UVPnt found by NR iteration 
        if (!bFoundAnswer) {
            rbSatisfiesTolerances = FALSE;
            return SM_SUCCESS;
        }

        SmPoint3d sCrvPnt2, sCrvPnt3;
        SER(m_cpSurface->EvaluatePoint(sUVFound,sCrvPnt2));
//        SER(sHermite3D.EvaluatePoint(dParam,sCrvPnt3));
        sCrvPnt3 = sCrvPnt2; // Temp cludge
        double dDistSq = sCrvPnt3.DistanceBetweenSquared(sCrvPnt2);
        dDistSq = smos_Max(dDistSq,sCrvPnt.DistanceBetweenSquared(sCrvPnt2));
        if (dDistSq > dMaxDist) {
            dMaxDist = dDistSq;
            if (dMaxDist > dTolSq) {
                rbSatisfiesTolerances = FALSE;
                return SM_SUCCESS;
            }
        }
    }

    rdDeviationFound = smos_Sqrt(dMaxDist);
    rbSatisfiesTolerances = TRUE; // passed test
    return SM_SUCCESS;

} // end SmSurfaceDropCurve::ComputeSpanDeviation

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfaceDropCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfaceDropCurve_TYPE == t) ? TRUE : SmSurfaceTracer::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print   

NOTES: 
***********************************************************************/
void SmSurfaceDropCurve::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmSurfaceDropCurve::Dump()")) ;

  // dump base
  SmSurfaceTracer::Dump() ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmSurfaceDropCurve::Dump()\n")) ;

} // end SmSurfaceDropCurve::Dump



