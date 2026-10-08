// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceTracer.cpp
* PURPOSE: This file contains surface intersector methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineSurface.h>
#include <SmBSplineCurve.h>
#include <SmSurfaceTracer.h>
#include <SmSolutionArray.h>
#include <SmGraphicsExtern.h>
#include <SmHermiteCurve.h>
#include <SmSurfaceCache.h>
#include <SmCurveCache.h>
#include <SmCacheMgr.h>
#include <SmGeomUtility.h>

#ifdef SM_DEBUG_CODE
#include <SmFace.h>
#include <SmBrep.h>
#include <SmSurfaceDropCurve.h>
#include <SmSurfaceSilhouette.h>
#include <SmCrvOnSurf.h>
#include <SmAssertArray.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: pVectorOrigin == NULL, draw point icon
            pVectorOrigin != NULL, draw vector starting at pVectorOrigin

NOTES: 
***********************************************************************/
SmDisplayList * SmTracePnt::Draw
 (const SmContext * pContext)  // NotUsed: in : pContext
 const
{
  SM_REF1(pContext) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  // output 3D point and tangent
  SmVector3d sVecEnd = m_v3DCurvePV[0] + m_v3DCurvePV[1] ;
  smgfx_DrawPoint(m_v3DCurvePV[0].x,
                  m_v3DCurvePV[0].y,
                  m_v3DCurvePV[0].z);
  smgfx_DrawLine(m_v3DCurvePV[0].x,
                 m_v3DCurvePV[0].y,
                 m_v3DCurvePV[0].z,
                 sVecEnd.x, 
                 sVecEnd.y, 
                 sVecEnd.z);

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmTracePnt::Draw

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmTracePnt::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  
  smos_sprintf(sBuff,        _T("SmTracePnt 0x%p"), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("SmTracePnt"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff,        _T("\n  PointType    : %s"), 
               m_ePointType == SM_TP_NORMAL        ? _T("SM_TP_NORMAL       ")     
             : m_ePointType == SM_TP_SINGULARITY   ? _T("SM_TP_SINGULARITY  ")       
             : m_ePointType == SM_TP_TANGENT_POINT ? _T("SM_TP_TANGENT_POINT")      
             : m_ePointType == SM_TP_TANGENT_LINE  ? _T("SM_TP_TANGENT_LINE ") 
             : _T("ERROR")) ;
  smos_WriteBuffer(sBuff);

  smos_WriteBuffer(_T("\n  Curve Parameter : ")) ; smos_sprintf(sBuff, _T("%16.16lf"), m_dCurveParameter) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n  SurfDropDist    : ")) ; smos_sprintf(sBuff, _T("%16.16lf"), m_dSurfDropDist) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n  SurfNormLineDist: ")) ; smos_sprintf(sBuff, _T("%16.16lf"), m_dSurfNormLineDist) ; smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  UV Curve Pos    : ")) ; m_vUVCurvePV[0].Dump() ; 
  smos_WriteBuffer(_T("\n  UV Curve Tan    : ")) ; m_vUVCurvePV[1].Dump() ; 
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  3D Curve XYZ Pos: ")) ; m_v3DCurvePV[0].Dump() ; 
  smos_WriteBuffer(_T("\n  3D Curve XYZ Tan: ")) ; m_v3DCurvePV[1].Dump() ; 
  smos_WriteBuffer(_T("\n")) ;
  smos_WriteBuffer(_T("\n  Surf XYZ Pos    : ")) ; m_vSurfacePV[0][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf XYZ Tan U  : ")) ; m_vSurfacePV[1][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf XYZ Tan V  : ")) ; m_vSurfacePV[0][1].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf XYZ Twist  : ")) ; m_vSurfacePV[1][1].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf XYZ Crv U  : ")) ; m_vSurfacePV[2][0].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf XYZ Crv V  : ")) ; m_vSurfacePV[0][2].Dump() ; 
  smos_WriteBuffer(_T("\n  Surf XYZ Norm   : ")) ; m_vSurfacePV[2][2].Dump() ; 

} // end SmTracePnt::Dump

/*******************************************************************//**
PURPOSE: Construct a surface tracer object.

NOTES: 
***********************************************************************/
SmSurfaceTracer::SmSurfaceTracer
  (const SmSurface  * cpSurface, 
   const SmExtent2d & crUVDomain)
 : m_cpSurface(cpSurface), 
   m_vUVDomain(crUVDomain),
   m_dThisAngTolRad(20.0*SM_PI/180.0), 
   m_pStartPoint(NULL)
{
  // Compute a reasonable default tolerance for intersection based on the sizes
  // of the surfaces.  Note that this may be overridden during subsequent
  // operations.
  SmPoint3d sPMid, sP1, sP2;
  SE(cpSurface->EvaluatePoint(crUVDomain.GetMin(),sP1));
  SE(cpSurface->EvaluatePoint(crUVDomain.Evaluate(0.5,0.5),sPMid));
  SE(cpSurface->EvaluatePoint(crUVDomain.GetMax(),sP2));
  double dSize = sP1.DistanceBetween(sPMid) +
      sPMid.DistanceBetween(sP2);
  m_dThisApproxTol3d = dSize/1000.0;  // accuracy is 1/1000th size of surface

  m_vTrcPntMgr.Initialize(ALIGN_SIZE(sizeof(SmTracePnt)),100) ;

  SmSurfaceCache *pSC = smsurf_GetSurfaceCache(m_cpSurface);
  if (!pSC) { SE(SM_ERR); return; }
  SmTree *pTree = pSC->GetTree();

  m_eSolverOperation = SM_SO_INTERSECT;
  m_eSolutionRequested = SM_SR_NODES;
  m_lNumTrees = 1;
  m_dBestAnswerSoFarSq = 0.0;
  m_dAtDistance = 0.0;
  m_dAtDistanceSq = 0.0;
  m_lNumVariables = 2;
  m_apTrees[0] = pTree;
  m_d3dTolerance = 0.0;
  m_dCurveTraceDirection = 1.0;
  m_bCurveIsClosed = FALSE;

} // end SmSurfaceTracer constructor

/*******************************************************************//**
PURPOSE: Compute the step size for the next point given
   the current point and the last step size.

NOTES: Note that the step size is a parametric step size

       This is a virtual function and should be implemented by all derived classes.

       Default behavior provided by this function,
         step size =   IsZero(dOldStepSize) 
                     ? Min(m_vUVDomain.XLength(), m_vUVDomain.YLength()) * 1.0e-2
                     : dOldStepSize
***********************************************************************/
SmStatus SmSurfaceTracer::ComputeStepSize
 (SmTracePnt & crCurrPnt,     // NotUsed: in : TracePnt to step from
  double       dOldStepSize,  // in : last (or appropriate) TraceCurve UVstep size
  double     & rdNewStepSize) // out: TraceCurve UVstepSize from crCurrPnt to NextPnt
{
  SM_REF1(crCurrPnt) ;
  // Surface UVDomain of interest
  SmPoint2d sUVSize = m_vUVDomain.GetSize();

  // default step size - expect derived classes to compute this differently
  rdNewStepSize =   SM_IS_ZERO(dOldStepSize) 
                  ? smos_Min(sUVSize.x,sUVSize.y) * 1.0e-2
                  : dOldStepSize ;

  // all done
  return SM_SUCCESS;

} // end ComputeStepSize

/*******************************************************************//**
PURPOSE: Determine if the point is within m_dThisApproxTol3d*(4.0 or 10.0) of the curves in cr3DCurves.
                      
NOTES: returns TRUE if crPointToTest is on any of the cr3DCurves 
         when cr3DCurves is degenerate to within tol = m_dThisApproxTol3d*4.0
         whne cr3DCurves is not degne  to within tol = m_dThisApproxTol3d*10.0
***********************************************************************/
SmBoolean SmSurfaceTracer::IsPointOnCurve
 (const SmPoint3d          & crPointToTest, // in : point to test
  const SmTArray<SmCurve*> & cr3DCurves)    // in : Previously traced curve solutions
 const
{
  // locals
  SmBoolean bPointIsOnCurve = FALSE;
  ULONG jj, lNumCurves = cr3DCurves.GetSize();

  // for every previously trace curve
  for (jj=0; jj<lNumCurves; jj++)
    {
      SmCurve * pCurve = cr3DCurves[jj];
      double    dCrvParam, dDist;
      SmBoolean bSuccess;

      // When curve is degenerate - check Dist(CurvePnt, PointToTest)
      if (pCurve->IsDegenerate())
        {
          // Get CurvePnt3d
          SmExtent1d sIvl = pCurve->GetNaturalInterval();
          SmPoint3d sCrvPnt;
          if ( pCurve->EvaluatePoint(sIvl.GetMin(),sCrvPnt) != SM_SUCCESS)
            { continue; }

          // check Dist3d(PointToTest, CrvPnt) to m_dThisApproxTol3d*4.0
          if (sCrvPnt.DistanceBetween(crPointToTest) < m_dThisApproxTol3d*4.0)
            {
              bPointIsOnCurve = TRUE;
              break;
            }
        } // end current curve is degenerate branch

      else // current curve is not degenerate - check Dist3d with max of m_dThisApproxTol3d*10.0
        {
          if(SM_SUCCESS != pCurve->DropPoint(pCurve->GetNaturalInterval(), // in : target curve allowed domain
                                             crPointToTest,                // in : Point to drop to curve
                                             NULL,                         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                           //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                           //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                             m_dThisApproxTol3d*10.0,      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                           //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                           //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                           //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                             NULL,                         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                             bSuccess,                     // out: TRUE = found a drop point
                                             dCrvParam,                    // out: found drop curve param
                                             dDist,                        // out: found drop distance
                                             SM_SO_INTERSECT))             // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                           //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                           //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                           //      default:[SM_SO_MINIMIZE] to preserve original behavior
            { continue; }                                                  

          if (bSuccess)
            {
              bPointIsOnCurve = TRUE;
              break;
            }
        } // current curve is not degenerate branch
    } // end iter all previously traced curves

  // all done
  return bPointIsOnCurve;

} // end IsPointOnCurve

/*******************************************************************//**
PURPOSE: Compute the next point in the trace stepping algorithm.
    This algorithm will compute the point based on the step size and return
    a boolean indicating if the step size satisfies all of our stepping 
    criteria.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceTracer::ComputeNextPoint
 (SmTracePnt & rCurrPnt,         // in : Current TraceCurve TracePnt
  double       dStepSize,        // in : UVStepSize from rCurrPnt to rNextPnt
  SmTracePnt & rNextPnt,         // out: Next TraceCurve TracePnt
  SmBoolean  & rbFoundGoodPoint, // out: we were able to find a good point.
  double     & rdDeviationFound, // out:
  double     & rdAngleFoundRad,  // out:
  SmBoolean  & rbClipped,        // out: step would cross a boundary, and is clipped.
  SmBoolean  & rbBoundaryHit)    // out: CurrPnt was already on a boundary, and step is leaving.
{
  // init outputs
  rbFoundGoodPoint = FALSE;
  rdDeviationFound = 0.0;
  rdAngleFoundRad  = 0.0;
  rbClipped        = FALSE;
  rbBoundaryHit    = FALSE;

  // If we hit a singularity last time then we need to stop
  if (    rCurrPnt.m_ePointType         == SM_TP_SINGULARITY
       && m_vCurvePoints.GetFirstNode() != &rCurrPnt)
    {
      rbClipped        = TRUE;
      rbBoundaryHit    = TRUE;
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }
  
  // init output
  rNextPnt.m_ePointType = SM_TP_NORMAL;

  // Check whether the proposed step will cross a domain boundary.
  // First pass let's just use first derivative of parameter space
  // curves to get start point. 
  double dOldStepSize = dStepSize;
  dStepSize = m_vUVDomain.ClipLine2d( rCurrPnt.m_vUVCurvePV[0],
                                      rCurrPnt.m_vUVCurvePV[1], 
                                      dStepSize );

  if (dStepSize < dOldStepSize - SM_EFF_ZERO*dOldStepSize)
    {
      rbClipped = TRUE;
    }

  if ( SM_IS_ZERO( dStepSize ))
    {
      rbFoundGoodPoint = FALSE;
      rbBoundaryHit = TRUE;
      return SM_SUCCESS;
    }

  // Take the step in uv and clamp to domain.
  SmVector2d sGuessUV = rCurrPnt.m_vUVCurvePV[0] + dStepSize * rCurrPnt.m_vUVCurvePV[1];
  sGuessUV = m_vUVDomain.ClampPoint2d(sGuessUV);
    
  // Sometimes we need the next curve parameter to compute point values
  rNextPnt.m_dCurveParameter = rCurrPnt.m_dCurveParameter + dStepSize;

  // Now that we have guess for the next point - do local iteration
  SmVector2d sRefinedUV;
  SmBoolean bFoundAnswer = FALSE;
  // Note, LocalPointSolve() does not use or set rNextPnt, it just sets sRefinedUV.
  SER( LocalPointSolve(rCurrPnt, sGuessUV, rNextPnt, bFoundAnswer, sRefinedUV ));

  if ( !bFoundAnswer )
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  //double d2DDist = sRefinedUV.DistanceBetween(sGuessUV);

  // Put RefinedUV into rNextPnt, and evaluate it there.
  // Given           : SurfacePoint and CurvePoint, 
  // Compute and Save: - SurfacePoint pos, 1st Derivs, and SurfaceNormal,
  //                   - DropCurve   3d pos, tangent, and  
  //                   - DropUVCurve 2d pos, tangent.
  SER( ComputePointValues( sRefinedUV, rNextPnt, &rCurrPnt ));

  // Adjust step size because it may have been clipped.
  SmVector2d sDiff = rNextPnt.m_vUVCurvePV[0] - rCurrPnt.m_vUVCurvePV[0];
  double dStepActual = sDiff.Length();
  sDiff = sGuessUV - rCurrPnt.m_vUVCurvePV[0];
  double dStepPredicted = sDiff.Length();
  if ( dStepActual < SM_EFF_ZERO || dStepPredicted < SM_EFF_ZERO )
    {
      rbFoundGoodPoint = FALSE;
      rbBoundaryHit = TRUE;
      return SM_SUCCESS;
    }
  dStepSize = dStepSize * ( dStepActual / dStepPredicted );

  rNextPnt.m_dCurveParameter = rCurrPnt.m_dCurveParameter + dStepSize;

  // First do a quick angular test to see if we can reject this span
  SER( rCurrPnt.m_v3DCurvePV[1].AngleBetween( rNextPnt.m_v3DCurvePV[1], rdAngleFoundRad ));
  if ( rdAngleFoundRad > m_dThisAngTolRad && dStepSize > SM_EFF_ZERO*100.0 )
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }
  // Also make sure that the point is going in the same direction.
  // There are some cases where the next point could be on the
  // wrong side of the current point.  This is also a violation
  // of our angle tolerance.
  double dLineParam;
  SER( smgu_LineClosestPoint( rCurrPnt.m_vUVCurvePV[0],
                              rCurrPnt.m_vUVCurvePV[1],
                              rNextPnt.m_vUVCurvePV[0],
                              dLineParam ));
  if ( dLineParam < 0.0 )
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // Compute the deviation from the actual curve.
  // This requires knowledge of the operation.
  SER( ComputeSpanDeviation( rCurrPnt, 
                             rNextPnt, 
                             dStepSize, 
                             rdDeviationFound, 
                             rbFoundGoodPoint ));

  // If we have a singularity and it satisfies tolerance then we should stop here.
  if ( rNextPnt.m_ePointType == SM_TP_SINGULARITY )
    {
       rbClipped = TRUE;
    }

  return SM_SUCCESS;

} // end ComputeNextPoint

/*******************************************************************//**
PURPOSE: Add a point to the existing m_vCurvePoints curve points array.  

NOTES: 
***********************************************************************/
SmStatus SmSurfaceTracer::AddPointToCurve
  (SmTracePnt & rNextPnt)           // in : point to add to m_vCurvePoints
{
  m_vCurvePoints.Append( &rNextPnt );
  return SM_SUCCESS;

} // end SmSurfaceTracer::AddPointToCurve

/*******************************************************************//**
PURPOSE: Add given point to m_vStartPoints array if it isn't already
            on that array.

NOTES:
  Regular points are the same when they have the same position in uv.
  Points at singularities are the same when they have the same
    position and parallel tangents 
***********************************************************************/
SmStatus SmSurfaceTracer::AddStartPoint
  (SmTracePnt & rStartTSP)        // in : candidate start point 
{
  // get current start point set
  SmTracePnt *aData[50];
  SmTArray<SmTracePnt*> sStartPts(20,aData);
  m_vStartPoints.GetAllNodes(sStartPts);
  SmBoolean bFoundMatch = FALSE;

  // for every startpoint
  ULONG i, lNumPts = sStartPts.GetSize();
  for ( i=0; i<lNumPts; i++ )
    {
      // get dist to candidate point
      SmTracePnt *pTSP = sStartPts[i];
      double dDistUV1Sq = rStartTSP.m_vUVCurvePV[0].DistanceBetweenSquared(
                          pTSP->m_vUVCurvePV[0]);

      // when candidate points are near one another
      if ( dDistUV1Sq < SM_EFF_ZERO )
        {
          // when both points mark a surface singularity
          if (rStartTSP.m_ePointType == SM_TP_SINGULARITY) 
            {
              if (pTSP->m_ePointType == SM_TP_SINGULARITY) 
                {
                  // points are the same when tangents are the same
                  SmVector3d sCross = rStartTSP.m_v3DCurvePV[1] * pTSP->m_v3DCurvePV[1];
                  if (   sCross.LengthSquared() < SM_EFF_ZERO_SQ
                      && rStartTSP.m_v3DCurvePV[1].Dot(pTSP->m_v3DCurvePV[1]) > 0.0) 
                    {
                      bFoundMatch = TRUE;  // Yes the tangents match
                      break;
                    }
                }
            }
          else 
            { // not singularity just distance check is fine
              bFoundMatch = TRUE;
              break;
            }
        } // end candidate points are near one another check
    }

  // when candidate point is unique
  if (!bFoundMatch) 
    {
      // add it to the m_vStartPoints array
      m_vStartPoints.Append(&rStartTSP);
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfaceTracer::AddStartPoint

/*******************************************************************//**
PURPOSE: Test the current point to see if it is near an existing 
   start point or the start of itself (i.e. curve is closed).

NOTES: 1. When NextPnt is close 3d (3*m_dApproxiationTol) to its TraceCurve's start , 
            it's snapped to form a gap free closed trace.
       2. TracePnts in m_vStartPoints coincident (UV SM_EFF_ZERO) to NextPnt are removed
            from m_vStartPoints to stop SmSurfaceTracer from tracing the
            same curve twice - in opposite directions.
***********************************************************************/
SmStatus SmSurfaceTracer::TestPoint
 (SmTracePnt & rNextPnt, // in : Point to test
  SmBoolean  & rbDone)   // out: TRUE = Done tracing Curve,
                         //              when TestPnt is at a tangency or a singularity,
                         //                or close to this TraceCurve's StartPnt 
                         //                or on the SurfaceUVBoundary.
{
  // init output
  rbDone = FALSE;

  // If point is at a tangent point then quit right here.
  if ( rNextPnt.m_ePointType == SM_TP_TANGENT_POINT )
    {
      rbDone = TRUE;
      return SM_SUCCESS;
    }

  // If point is at a singularity point then quit right here.
  if ( rNextPnt.m_ePointType == SM_TP_SINGULARITY )
    {
      rbDone = TRUE;
      return SM_SUCCESS;
    }

  // If we have a start point check for a closed curve situation
  if ( m_pStartPoint != NULL )
    {
      // Don't check if we are only starting the curve
      SmTracePnt *pPrev1 = m_vCurvePoints.GetLastNode();
      if (pPrev1 != m_pStartPoint)
        {
          SmTracePnt *pPrev2 = m_vCurvePoints.GetPrevNode(pPrev1);
          if (pPrev2 != m_pStartPoint)
            {
              // There is an inherent problem in checking for the start point,
              // it has to do with the fact that the curve may not be within
              // the given tolerance of the theoretical curve depending upon 
              // the operation we are tracing.  However, today
              // we just put in a couple of fudge factors (2.0 and 3.0) below
              // to adjust the tolerance and try to catch those situations.
              // 

              // make the hermite curve for the last trace segment
              double dDeltaT = rNextPnt.m_dCurveParameter - pPrev1->m_dCurveParameter;
              SmHermiteCurve sHerm(pPrev1 ->m_v3DCurvePV[0], 
                                   pPrev1 ->m_v3DCurvePV[1]*dDeltaT,
                                   rNextPnt.m_v3DCurvePV[0], 
                                   rNextPnt.m_v3DCurvePV[1]*dDeltaT, 3);
              sHerm.SetContext( NULL );

              // get hermite curve's BBox
              SmExtent3d s3DBox;
              SER( sHerm.CalculateBoundingBox( sHerm.GetNaturalInterval(), &s3DBox, NULL ));

              // Build BBox centered on StartPoint sized 2*m_dThisApproxTol3d
              SmExtent3d sPntBox( m_pStartPoint->m_v3DCurvePV[0] );
              sPntBox.ExpandAbsolute( m_dThisApproxTol3d*2.0 );

              // when HermiteCurve BBox and StartPoint BBox intersect
              if ( !s3DBox.AreDisjoint( sPntBox ))
                {
                  // Looks like we have a candidate for the end point.
                  // Test it to see if the start point falls close enough to the Hermite curve.

                  // locals - HermiteCurve's midPoint Param
                  double     dGuess = sHerm.GetNaturalInterval().Evaluate(0.5);
                  SmSolution sSol;
                  SmBoolean  bSuccess;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
                  if (bDebugMe) 
                    {
                      sm_GraphicsLoop();
                      smgfx_SetLook(3,4, 1,0,0); m_pStartPoint->m_v3DCurvePV[0].Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,0,0); sHerm.DrawWDeriv(sHerm.GetNaturalInterval(),0); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE

                  // find closest HermiteCurve pt to StartPoint
                  SER( sHerm.LocalPointSolve( sHerm.GetNaturalInterval(), 
                                              SM_SO_MINIMIZE,
                                              m_pStartPoint->m_v3DCurvePV[0], 
                                              NULL, 
                                              NULL, 
                                              NULL,
                                              dGuess, 
                                              bSuccess, 
                                              sSol ));

                  // when Dist(StartPoint,HermiteCurve) < m_dThisApproxTol3d*3.0  GWC:does fudge factor of 3 mess things up?
                  if ( bSuccess && sSol.m_vStart.m_dSolutionValue < m_dThisApproxTol3d*3.0 )
                    {
                      // Have found loop around - 
                      // snap rNextPnt to StartPoint's UV position, GWC:
                      // gwc: the snap eliminates gap problems - this is probably okay
                      //      the snap happens without a change in rNextPnt parameter value. That's
                      //      an m_dThisApproxTol3d sized change in end point parameterization.
                      //      The param values at which the TraceCurve end point and the original Curve
                      //      may vary by as much as 3*m_dThisApproxTol3d - That might be a tolerance problem for Trace Curve.
                      SmPoint2d sUV = m_pStartPoint->m_vUVCurvePV[0];

                      // Given           : SurfacePoint(from m_pStartPoint) and CurvePoint(from rNextPnt), 
                      // Compute and Save: - SurfacePoint pos, 1st Derivs, and SurfaceNormal,
                      //                   - DropCurve   3d pos, tangent, and  
                      //                   - DropUVCurve 2d pos, tangent.
                      SER( ComputePointValues( sUV, rNextPnt ));
                      m_bCurveIsClosed = TRUE;
                      rbDone = TRUE;
                      return SM_SUCCESS;
                    } // end Dist(StartPoint,HermiteCurve) < m_dThisApproxTol3d*3.0 check
                } // end HermiteCurve BBox and StartPoint BBox intersect check
            } // end Previous TracePnt is not the StartPoint check  (only 1 pt in the trace so far)
        } // end rNextPoint is not the StartPoint check (rNextPoint is the 1st TracePnt)
    } // end m_pStartPoint existence check

  // is NextPnt.UVPnt on m_vUVDomain boundary (tol = SM_EFF_ZERO)
  SmBoolean bOnBoundary = m_vUVDomain.IsPoint2dOnBoundary( rNextPnt.m_vUVCurvePV[0] );

  // exit case - TracePnt is in the middle of the surface - not the end of this trace
  if ( !bOnBoundary )
    {
      rbDone = FALSE;
      return SM_SUCCESS;
    }

  // arrive here when rNextPnt is on a Surface boundary.
  rbDone = TRUE;
  SmTracePnt * aData[50];
  SmTArray<SmTracePnt*> sStartTSP(50,aData);

  // Get all StartPnts for this trace
  m_vStartPoints.GetAllNodes(sStartTSP);
  ULONG i, lNumPts = sStartTSP.GetSize();

  // for every StartPnt - remove  StartPoints coincident with rNextPnt from m_vStartPoints array
  //   this will prevent the tracer from finding the same curve solution two times, once in each direction.
  for ( i=0; i<lNumPts; i++ )
    {
      SmTracePnt *pTSP = sStartTSP[i];

      // get UVDist(rNextPnt.UVPnt, pTSP.UVPnt)
      double dDistUV = pTSP->m_vUVCurvePV[0].DistanceBetweenSquared(rNextPnt.m_vUVCurvePV[0] );

      // when UVDist < SM_EFF_ZERO - GWC: bug? should at least use ScaledZero, maybe should use ApproxTol 
      //                                  mapped to 3d.
      if ( dDistUV <= SM_EFF_ZERO )
        {
          m_vStartPoints.Remove( pTSP );
        }
    } // end iter every start point

  // all done
  return SM_SUCCESS;

} // end SmSurfaceTracer::TestPoint

/*******************************************************************//**
PURPOSE: Trace a curve from the given starting point stopping either
    at a boundary or when it becomes a closed loop.

NOTES: loads a sequence of SmTracePnts into m_vCurvePoints
***********************************************************************/
SmStatus SmSurfaceTracer::TraceCurve
 (SmTracePnt         & rStartPoint,  // in : StartPnt for this trace sequence
  SmTArray<SmCurve*> & r3DCurves)    // in : previously traced curves, prevents steeping a previously found solution
{
  // init output
  m_bCurveIsClosed = FALSE;

  //locals 
  ULONG     lCount = 0;
  SmPoint2d sUVPt  = rStartPoint.m_vUVCurvePV[0];

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmFace     *pFace = (SmFace *)m_cpSurface->GetFace() ;
      SmBrep     *pBrep = pFace ? pFace->GetBrep() : NULL ;
      SmTracePnt *pPnt1 = &rStartPoint ;
      SmTracePnt *pI    = NULL ;
      ULONG       ii ;

      // draw Brep(blue), TargetSurface(cyan), Face(black), TargetCurve(green), m_vStartPoints(red)
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ;    m_cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .3,.3,.3) ; m_cpSurface->DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, .3,.3,.3) ; m_cpSurface->DrawPoles() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH );  sm_GraphicsLoop(); }
      smgfx_SetLook( 2, 3, 1, 0, 0 ); if(this->IsKindOf( SmSurfaceDropCurve_TYPE )) { ((SmSurfaceDropCurve*)this)->GetCurve().Draw(); sm_GraphicsLoop(); }
 //     smgfx_SetLook(3,6, 1,0,1) ; m_crCurveToDrop.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,8, 1,0,0) ; for(ii=0, pI=pPnt1; 
                                      pI && (ii == 0 || pI != pPnt1);
                                      ii++, pI = m_vStartPoints.GetNextNode(pI))
                                    {
                                      pI->Draw(m_cpSurface->GetContext()) ; pI->Dump() ; sm_GraphicsLoop() ;
                                    } 
      smgfx_SetLook(6,7, 1,0,1) ; for(ii=0; ii<r3DCurves.GetSize(); ii++)
                                    {
                                      r3DCurves[ii]->Draw() ; sm_GraphicsLoop() ;
                                    }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Note that if we are tracing backward the start point is
  // already on the list.
  if (m_vCurvePoints.GetFirstNode() == NULL)
    {
      // Given           : SurfacePoint and CurvePoint, 
      // Compute and Save: - SurfacePoint pos, 1st Derivs, and SurfaceNormal,
      //                   - DropCurve   3d pos, tangent, and  
      //                   - DropUVCurve 2d pos, tangent.
      if (ComputePointValues(sUVPt, rStartPoint) != SM_SUCCESS)
        {
          return SM_SUCCESS;  // Bad start point just go to next one
        }

      // when StartPoint is a TangentPoint
      if (rStartPoint.m_ePointType == SM_TP_TANGENT_POINT)
        {
          // when StartPoint is not on an already traced curve          
          if (!IsPointOnCurve(rStartPoint.m_v3DCurvePV[0],r3DCurves) ) // TRUE = Pnt near cr3DCurve, tol = m_dThisApproxTol3d*(4.0 or 10.0)
            {
              // place one point in m_vCurvePoints and quit
              rStartPoint.m_dCurveParameter   = 0.0; // curves start at 0.0
              rStartPoint.m_dSurfDropDist     = 0.0;
              rStartPoint.m_dSurfNormLineDist = 0.0;
              m_vCurvePoints.Append(&rStartPoint);
              return SM_SUCCESS;
            }
          else // skip tracing this point
            { // Point is already found
              return SM_SUCCESS;
            }
        } // end StartPoint is a TangentPoint check

      // Set StartPoint Parameter and deviation to zero, Make StartPoint 1st m_vCurvePoints element
      rStartPoint.m_dCurveParameter   = 0.0; // curves start at 0.0
      rStartPoint.m_dSurfDropDist     = 0.0;
      rStartPoint.m_dSurfNormLineDist = 0.0;
      m_vCurvePoints.Append(&rStartPoint);

      // Check to see if first step is out of bounds -- If it is try reversing the direction.
      SmPoint2d sDelta = rStartPoint.m_vUVCurvePV[0] + SM_EFF_ZERO_SQRT * rStartPoint.m_vUVCurvePV[1];
      if (!m_vUVDomain.ContainsPoint2d(sDelta,SM_EFF_ZERO_SQRT/100000.0))
        {
          m_dCurveTraceDirection = -1.0;  // Reverse curve tracing direction
          SER(ReverseCurveDirection());
        }
    } // end if first m_vCurvePoints node is Null check

  // Compute the values in the start point and put it into the list.
  double    dStepSize       = 0.0,   dDeviation      = 0.0;
  SmBoolean bDone           = FALSE, bHitBoundary    = FALSE;
  ULONG     lSmallStepCount = 0,     lPointCount     = 0;

  // iterate until sequence of UVStepped TracePnts terminates.
  //  each iteration adds one TracePnt to m_vCurvePoints array
  while ( !bDone )
    {
      // count iterations
      lPointCount ++;

      // From the last SmTracePnt in m_vCurvePoints
      SmTracePnt *pTSP = m_vCurvePoints.GetLastNode();

      // compute the next UVstep size
      SER(ComputeStepSize(*pTSP, dStepSize, dStepSize));

      // locals for NextPnt computation
      SmTracePnt *pNextPnt = (SmTracePnt*)m_vTrcPntMgr.GetNewElement();
      pNextPnt->m_ePointType = SM_TP_NORMAL;
      SmBoolean bFoundGoodPoint ;
      double    dAngleFound;
      SmBoolean bClipped;

      // Given current TracePnt and UVStepSize, compute and classify NextPoint = CurrPnt + UVStepSize
      SER(ComputeNextPoint(*pTSP,
                           dStepSize,
                           *pNextPnt,
                           bFoundGoodPoint,
                           dDeviation,
                           dAngleFound,
                           bClipped,
                           bHitBoundary));
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,0,0) ; pNextPnt->Draw(m_cpSurface->GetContext()); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // TraceCurve exit case: TraceCurve hit a surface boundary
      if (bHitBoundary)
        { break; }

      // When NextTracePnt is bad, iterate until a good NextTracePnt = CurrTracePnt + UVStepSize is found. 
      //  decrease UVStepSize in each iteration until a good NextTracePnts is found
      SmBoolean bIncreasedStepSize = FALSE;
      while (!bFoundGoodPoint)
        {
          // cut UVStepSize in half
          dStepSize /= 2.0;

          // Seek Good NextPnt exit case: Can't find a good NextPoint no matter how small UVStepSize is made
          if (dStepSize < SM_EFF_ZERO_SQRT)
            {
              // We have hit something nasty here if we have enough
              // just quit and check things out down below.
              bDone = TRUE;
              break;
            }

          // remember that UVStepSize was changed
          bIncreasedStepSize = TRUE;
          
          // Given current TracePnt and new UVStepSize, compute and classify NextPoint = CurrPnt + UVStepSize
          if(SM_SUCCESS != ComputeNextPoint(*pTSP,dStepSize,*pNextPnt,
                                            bFoundGoodPoint,
                                            dDeviation,
                                            dAngleFound,
                                            bClipped,
                                            bHitBoundary))
            {
              bFoundGoodPoint = FALSE;
            }

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,0,0) ; pNextPnt->Draw(m_cpSurface->GetContext()); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end while looking for good NextTracePnt by cutting UVStepSize in half each iteration

      // TraceCurve exit case: couldn't find a good NextTracePnt 
      if (bDone)
        { break; }

      // arrive here when pNextPnt is computed and bFoundGoodPoint is TRUE

      // If we didn't go through loop to decrease step size then we should try
      // to increase the step size. (optimization to minimize number of TracePnts on TraceCurve)
      if (    !bClipped                                  // NextPnt is not a clipped point
           && !bIncreasedStepSize                        // did not reduce UVStepSize to find a good NextPnt
           && dDeviation  < m_dThisApproxTol3d / 8.0  // Deviation in next TraceIvl is small
           && dAngleFound < m_dThisAngTolRad / 2.0     // AngleChange in next TraceIvl is small
         )
        {
          // locals
          double     dLastCurveParam = pNextPnt->m_dCurveParameter;
          SmTracePnt sLast;

          // While good, unclipped NextPoints are being found for larger UVStepSizes
          while (bFoundGoodPoint && !bClipped)
            {
              // double UVStepSize 
              dStepSize *= 2.0;

              // remember the last NextPnt that was classified as a good TracePnt
              sLast = *pNextPnt;

              // compute next UVStepSize from recently doubled UVStepSize
              SER(ComputeStepSize(*pTSP,dStepSize,dStepSize));

              // compute and classify NextTracePnt = CurrTracePnt + UVStepSize * CurrTracePnt.UVCurveTangent
              SER(ComputeNextPoint(*pTSP,dStepSize,*pNextPnt,
                                    bFoundGoodPoint,
                                    dDeviation,
                                    dAngleFound,
                                    bClipped,
                                    bHitBoundary));

              // If actual step size does not increase - we probably hit a boundary
              // It is a good idea to stop if we are not getting further along the curve
              // in any case.
#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,0,0) ; pNextPnt->Draw(m_cpSurface->GetContext()); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // largerUVStepSize exit case: found good NextPnt didn't increase UVStepSize
              //  - hit a boundary or something like that.
              if(   bFoundGoodPoint
                 && pNextPnt->m_dCurveParameter <= dLastCurveParam + SM_EFF_ZERO) 
                {
                  bFoundGoodPoint = FALSE;
                  break;
                }

              // update the NextPnt CurveParam 
              dLastCurveParam = pNextPnt->m_dCurveParameter;

            } // end while increasing UVStepSize looking for max good step to NextPnt.

          // TraceCurve exit case: something occurred while increasing UVStepSize (currently not used)
          if (bDone)
            { break; }

          // when while doubling UVStepSize ended by finding a bad NextPoint
          if (! bFoundGoodPoint)
            {
              // set UVStepSize and NextPnt back to previous iteration - 
              // That's the last UVStepSize that yielded a Good NextPnt
              dStepSize /= 2.0;
              *pNextPnt = sLast;
//                SER(ComputeNextPoint(*pTSP,dStepSize,*pNextPnt,bFoundGoodPoint,
//                    dDeviation,dAngleFound,bClipped,bHitBoundary));
            }
        } // end OK to try increased UVStepSizes check

      // arrive here when dStepSize and pNextPnt are set with a good UVStepSize and a good TracePnt

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          sm_GraphicsLoop();
          smgfx_SetLook(4,8, 1,0,0) ; rStartPoint.m_v3DCurvePV[0].Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,8, 1,0,1) ; pNextPnt->Draw(m_cpSurface->GetContext()); pNextPnt->Dump() ; sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // If the point is the first point after a singularity or tangency
      // point then we need to test the second point to see if it is on
      // an existing curve.
      if ( lPointCount == 1 )
        {
          SmPoint3d sPtToTest = pNextPnt->m_v3DCurvePV[0];

          // The first point after starting could also be the last point
          // on a curve, and that last point could be on another curve,
          // without the interior of the curve coinciding with any other curve.
          // If that's the case, bClipped will be set.
          // In that case, find a midpoint and use that to check.
          // [B74]
          if ( bClipped )
            {
              // locals
              double     dHalfStep = dStepSize / 2.0;
              SmTracePnt sMid;
              SmBoolean  bFoundMid, bClippedMid, bHitBdryMid;
              double     dDevMid, dAngleMid;

              // compute MidTracePnt = CurrTracePnt + dHalfStep * UVCurve.Tangent
              SmStatus eStat = ComputeNextPoint( *pTSP, dHalfStep, sMid, 
                                                  bFoundMid,
                                                  dDevMid, 
                                                  dAngleMid, 
                                                  bClippedMid, 
                                                  bHitBdryMid );

              // when a good MidTracePnt was found - use that as the TracePnt to test
              if ( eStat == SM_SUCCESS && bFoundMid )
                {
                  sPtToTest = sMid.m_v3DCurvePV[0];
                }
            } // end clipped NextPnt check

          // See if TracePnt to test is on any of the already traced curves
          if ( IsPointOnCurve( sPtToTest, r3DCurves ) ) // TRUE = Pnt near cr3DCurve, tol = m_dThisApproxTol3d*(4.0 or 10.0)
            {
              // TraceCurve exit case: 1st point traced is on an already traced curve
              break;
            }
        } // end NextPnt is the first point after a singularity or tangency check
      
      // If step size gets too small - quit curve tracing
      // - we are probably approacing a problem point - quit tracing when close to the problem
      if (dStepSize < SM_EFF_ZERO_SQRT)
        {
          lSmallStepCount ++;
          bDone = TRUE;
        }
      else
        {
          lSmallStepCount = 0;  // Reset small step count 
        }

      // When StepSize is small - quit without adding point to m_vCurvePoints
      if (lSmallStepCount > 10 || bDone)
        {
          bDone = TRUE;
        }
      else // StepSize is just fine
        {
          // TestPoint looking for end Tracing conditions
          SER(TestPoint(*pNextPnt, // in : Point to test - gets snapped for close to closed trace cases
                         bDone));  // out: TRUE = Done tracing Curve,
                                   //              when TestPnt is at a tangency or a singularity,
                                   //                or close to this TraceCurve's StartPnt 
                                   //                or on the SurfaceUVBoundary.

          // add NextPnt to m_vCurvePoints array
          SER(AddPointToCurve(*pNextPnt));
        }

      // count the number of TraceSteps
      lCount++;

      // When TraceStep count gets too large - stop TraceCurve
      if (lCount > 1000)
        {
          bDone = TRUE;
        }
    } // end while TracingCurve as sequence of UVSteps loading TracePnts to m_vCurvePoints array

  // when last TracePnt on TraceCurve is on a singularity - Add TracePnt to m_vStartPoints array 
  // so the Curve on the other side of the singularity can also be traced.
  SmTracePnt *pTSP = m_vCurvePoints.GetLastNode();
  if (pTSP->m_ePointType == SM_TP_SINGULARITY && lCount > 1)
    {
      SmTracePnt * pTSP2 = (SmTracePnt*)m_vTrcPntMgr.GetNewElement();
      SmTracePnt & rTSP2 = *pTSP2;
      rTSP2 = *pTSP;
      AddStartPoint(rTSP2);
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfaceTracer::TraceCurve

/*******************************************************************//**
PURPOSE: Flush out the curve defined in the control point and knots arrays
            and create a B-Spine and put it into the curves array.

NOTES: - Creates a cubic Hermite curve (full knot multiplicities).
       - Hint: use pNewBSP->RemoveExtraKnots(SM_EFF_ZERO) ;
         after this call to reduce the number of knots and control points
         on the output curve.
***********************************************************************/
static SmStatus sm_FlushCrv
 (const SmContext     & crContext,  // in : context for new object construction
  ULONG                 lDimension, // in : for curve creation
  SmTArray<SmPoint3d> & rCntrlPoly, // in : all control points
  SmTArray<double>    & rKnots,     // in : unique knot values, no multiplicities
  SmTArray<SmCurve*>  & crCurves)   // out: appended with 1 new output curve
{   
  // no work - not enough points
  if(rKnots.GetSize() < 2) 
    { return SM_SUCCESS; }

  // Set up knot vector: cubic, full multiplicities.
  SmTArray<ULONG> sKnotMult( rKnots.GetSize(), NULL, rKnots.GetSize() );
  ULONG i, lNumMults = sKnotMult.GetSize();
  sKnotMult[0]                     = 4;
  for(i=1; i+1<lNumMults; i++) // note: can't say lNumMults-1
    { sKnotMult[i]                 = 3; }
  sKnotMult[sKnotMult.GetSize()-1] = 4;

  // Create the curve.
  SmBSplineCurve *pNewBSP = NULL;
  ULONG lDegree = 3;
  SER( SmBSplineCurve::CreateCanonical( crContext, lDimension, lDegree,
                                        rCntrlPoly, SM_CF_UNSPECIFIED, 
                                        sKnotMult, rKnots, 
                                        SM_KT_UNSPECIFIED, 
                                        NULL, NULL, pNewBSP ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      if (FALSE) 
        { pNewBSP->Dump(); }
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; pNewBSP->DrawPolygon(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

// Note that the following line will remove extra knots and
// decrease the size of the curve.  This can easily be done
// after drop curve
//    SER(pNewBSP->RemoveExtraKnots(SM_EFF_ZERO));

  // set output
  crCurves.Add(pNewBSP);

  // all done
  return SM_SUCCESS;

} // end sm_FlushCrv

/*******************************************************************//**
PURPOSE: Flush the curve which currently exists in m_vCurvePoints list.

NOTES: m_vCurvePoints is always emptied by this call.  When m_vCurvePoints contains
      - 1 tangent point    - output a degenerate curve
      - less than 2 points - return without outputting a curve
      - 2 points closer together than m_dThisApproxTol3d - return without outputting a curve
      - 2 or more points - output a piecewise Bezier interpolation of m_vCurvePoints
***********************************************************************/
SmStatus SmSurfaceTracer::FlushCurve
  (SmTArray<SmCurve*> & r3DCurves,        // out: add new 3d BSplineCurve = Bezier cubic interp of m_vCurvePoints
   SmTArray<SmCurve*> & rSurfaceUVCurves, // out: add new 2d UVTrimCurve  = Bezier cubic interp of m_vCurvePoints
   SmTArray<double>   & rMaxDropDists,    // out: When pOptCurve!=NULL, Max(Dot(pOptCurve(s)-NewCurve,SurfNorm), else 0.0
   SmTArray<double>   & rDeviations,      // out: Max(m_vCurvePoints.Deviation = Dist(
   const SmCurve      * pOptCurve)        // in : default:[NULL}
{
  SmTracePnt *aData[200];
  SmTArray<SmTracePnt*> sPnts(200,aData);

  // Grab the m_vCurvePoints list into an array and clear out the list.
  m_vCurvePoints.GetAllNodes(sPnts);
  while (m_vCurvePoints.GetLastNode())
    {
      m_vCurvePoints.RemoveLast(); // don't delete last SmTsectPoint. It gets deleted by m_vTrcPntMgr block manager.
    }

  SmPoint3d sCrvPt, sSrfPt;
  SmVector3d sNorm;
  double dDist = 0.0;

  // For single tangent point - create a degenerate curve.
  ULONG lNumPts = sPnts.GetSize();
  if(lNumPts == 1 && sPnts[0]->m_ePointType == SM_TP_TANGENT_POINT)
    {
      // 3d BSplineCurve locals
      SmTracePnt        * pTSPCurr = sPnts[0];
      SmBSplineCurve    * pNewBSP  = NULL ;
      double              adKData[2];
      ULONG               alKMData[2];
      SmPoint3d           sData[2];
      SmTArray<double>    sKnots(2,adKData,2);
      SmTArray<ULONG>     sKnotMult(2,alKMData,2);
      SmTArray<SmPoint3d> sCntrlPoly(2,sData);

      // let control polygon = two coincident points
      sCntrlPoly.Add(pTSPCurr->m_v3DCurvePV[0]);
      sCntrlPoly.Add(pTSPCurr->m_v3DCurvePV[0]);

      // let Param domain = [0 1]
      sKnots[0] = 0.0; sKnots[1] = 1.0;

      // let knot vector = [2 2]
      sKnotMult[0] = 2;
      sKnotMult[1] = 2;

      // construct the degenerate BSplineCurve
      SER(SmBSplineCurve::CreateCanonical(*m_cpContext,
                                          3,
                                          1,
                                          sCntrlPoly, 
                                          SM_CF_UNSPECIFIED, 
                                          sKnotMult, 
                                          sKnots, 
                                          SM_KT_UNSPECIFIED, 
                                          NULL, 
                                          NULL, 
                                          pNewBSP));
      // set output
      r3DCurves.Add(pNewBSP);
        
      // Now build UVTrimCurve - update Control polygon - reuse same parameters and knots
      sCntrlPoly[0] = SmPoint3d(pTSPCurr->m_vUVCurvePV[0]);
      sCntrlPoly[1] = SmPoint3d(pTSPCurr->m_vUVCurvePV[0]);

      // construct the degenerate UVTrimCurve
      SER(SmBSplineCurve::CreateCanonical(*m_cpContext,2,1,
                                           sCntrlPoly, 
                                           SM_CF_UNSPECIFIED, sKnotMult, sKnots, 
                                           SM_KT_UNSPECIFIED, 
                                           NULL, NULL, pNewBSP));

      // when given pOptCurve - Calculate max drop distance 
      // gwc: Drop dist is now precalculated and stored in pTSPCurr
      //  dDist = 0.0;
      //  if ( pOptCurve != NULL )
      //    {
      //      pOptCurve->EvaluatePoint( pTSPCurr->m_dCurveParameter, sCrvPt );
      //      sSrfPt = pTSPCurr->m_vSurfacePV[0][0];
      //      sNorm  = pTSPCurr->m_vSurfacePV[2][2];
      //      dDist  = smos_Fabs( ( sCrvPt - sSrfPt ).Dot( sNorm ) );
      //    }

      // set outputs
      rSurfaceUVCurves.Add(pNewBSP);
      rMaxDropDists.Add( pTSPCurr->m_dSurfDropDist );  // rMaxDropDists.Add( dDist );
      rDeviations.Add( pTSPCurr->m_dSurfNormLineDist );

      // all done
      return SM_SUCCESS;
    } // end single tangent point (output degenerate curve) check

  // arrive here when m_vCurvePoints does NOT contain 1 tangent point

  // exit case - less than 2 m_vCurvePoints - can't build a curve
  if ( lNumPts < 2 )
    {
      return SM_SUCCESS;
    }

  // exit case - 2 m_vCurvePoints closer than m_dThisApproxTol3d apart - don't build short curves
  if ( lNumPts == 2 )
    {
      SmTracePnt *pTSPLast = sPnts[0];
      SmTracePnt *pTSPCurr = sPnts[1];
      if ( pTSPLast->m_v3DCurvePV[0].DistanceBetween( pTSPCurr->m_v3DCurvePV[0] )
           < m_dThisApproxTol3d )
        {
          return SM_SUCCESS;
        }
    }

  // arrive here when m_vCurvePoints has 2 or more points separated by more than m_dThisApproxTol3d

  // locals
  ULONG i;
  SmTArray<double>    sKnots;
  SmTArray<SmPoint3d> s3DCtrlPts;
  SmTArray<SmPoint3d> sUVCtrlPts;
  double              dMaxDropDist  = sPnts[0]->m_dSurfDropDist ; // -1.0;
  double              dMaxDeviation = sPnts[0]->m_dSurfNormLineDist ;

  // init 3d BSplineCurve and 2d UVTrimCurve with 1st Knot and Control point
  sKnots.    Add(sPnts[0]->m_dCurveParameter) ;
  s3DCtrlPts.Add(sPnts[0]->m_v3DCurvePV[0]) ;
  sUVCtrlPts.Add(SmPoint3d(sPnts[0]->m_vUVCurvePV[0])) ;

  // When asked - calc Drop Distance ([CrvPt-SrfPt] comp in SurfNorm dir for 1st ControlPoint
  // gwc: already calculated DropDistance stored in sPnts[0]
  //  if(pOptCurve)
  //    {
  //      pOptCurve->EvaluatePoint( sPnts[0]->m_dCurveParameter, sCrvPt );
  //      sSrfPt = sPnts[0]->m_vSurfacePV[0][0];
  //      sNorm  = sPnts[0]->m_vSurfacePV[2][2];
  //      dDist  = smos_Fabs( ( sCrvPt - sSrfPt ).Dot( sNorm ) );
  //      if ( dDist > dMaxDropDist ) 
  //        { dMaxDropDist = dDist; }
  //    }

  // for every remaining m_vCurvePoint - build piecewise cubic bezier interpolation of m_vCurvePoints
  for(i=1; i<lNumPts; i++)
    {
      SmTracePnt * pTSPLast = sPnts[i-1];   
      SmTracePnt * pTSPCurr = sPnts[i];
      double       dScale   = pTSPCurr->m_dCurveParameter - pTSPLast->m_dCurveParameter;

      // save max deviation = dist of Curve Point from SurfNorm line anchored at Drop point
      if(pTSPCurr->m_dSurfNormLineDist > dMaxDeviation) 
        { dMaxDeviation = pTSPCurr->m_dSurfNormLineDist; }

      // Add the next knot
      sKnots.Add(pTSPCurr->m_dCurveParameter);

      // Cubic Bezier: inner points are offset from start and end points by derivatives / degree.
      // (Scope just so that we can use the same names for 3d and 2d points.)
        {
          // 3d cubic bezier control points
          SmPoint3d  sP1( pTSPLast->m_v3DCurvePV[0] );    
          SmPoint3d  sP4( pTSPCurr->m_v3DCurvePV[0] );
          SmVector3d sD1( pTSPLast->m_v3DCurvePV[1] * dScale );
          SmVector3d sD2( pTSPCurr->m_v3DCurvePV[1] * dScale );
          SmPoint3d  sP2 = sP1 + sD1 / 3.0;
          SmPoint3d  sP3 = sP4 - sD2 / 3.0;
        
          // add ControlPoints [2,3,4] - control point [1] is already in s3DCtrlPts
          s3DCtrlPts.Add( sP2 );
          s3DCtrlPts.Add( sP3 );
          s3DCtrlPts.Add( sP4 );

        } // end scope to distinguish between same named 3d and 2d points

        {
          // 2d UVTrimCurve Control points
          SmPoint2d  sP1( pTSPLast->m_vUVCurvePV[0] ); 
          SmPoint2d  sP4( pTSPCurr->m_vUVCurvePV[0] );
          SmVector2d sD1( pTSPLast->m_vUVCurvePV[1] * dScale );
          SmVector2d sD2( pTSPCurr->m_vUVCurvePV[1] * dScale );
          SmPoint2d  sP2 = sP1 + sD1 / 3.0;
          SmPoint2d  sP3 = sP4 - sD2 / 3.0;
        
          // add ControlPoints [2,3,4] - control point [1] is already in s3DCtrlPts
          sUVCtrlPts.Add(SmPoint3d( sP2 ));
          sUVCtrlPts.Add(SmPoint3d( sP3 ));
          sUVCtrlPts.Add(SmPoint3d( sP4 ));

          // Calc drop dists at Middle and End of each new Bezier segment.
          if(pOptCurve)
            {
              // Middle: we'll have to evaluate the curve and surface there.

              // Mid Segment CrvPoint
              double dT = ( pTSPLast->m_dCurveParameter + pTSPCurr->m_dCurveParameter ) / 2.0;
              pOptCurve->EvaluatePoint( dT, sCrvPt );

              // Mid Segement SrfPoint (SurfUV point is the midPoint of the Bezier UVTrimCurve)
              SmPoint2d sUV( (sP1 + 3*sP2 + 3*sP3 + sP4) / 8.0 );
              m_cpSurface->EvaluatePoint( sUV, sSrfPt );

              // When Dist3d(CrvPnt, SrfPnt) is greater than MaxDropDist
              dDist = sCrvPt.DistanceBetween( sSrfPt );
              if ( dDist > dMaxDropDist )
                {
                  // SrfPnt is not a DropPnt of CrvPnt.  It is the correlation
                  // point between pOptCurve and the dropped UVTrimCurve sharing
                  // common parameter values.
                  // In SMLib 3dCurves and their associated DropCurves are supposed
                  // to share a common parameterization.

                  // Dropping to the correlation point is a good measure of the
                  // accuracy of the drop curve.  As with Drop distance, we
                  // check the component of (CrvPnt - SrfPnt) in the surfNorm direction.
                  // Using the correlation point rather than the drop point should
                  // be close to the actual drop distance and saves iterating to find the drop point.
                  m_cpSurface->EvaluateNormal( sUV, FALSE, FALSE, sNorm );
                  dDist = smos_Fabs( ( sCrvPt - sSrfPt ).Dot( sNorm ) );
                  if ( dDist > dMaxDropDist ) 
                    { dMaxDropDist = dDist; }

#ifdef SM_DEBUG_CODE
                  // Double-check that assertion about the distance.
                  SmPoint2d sUVDrop;
                  double    dTempDist;
                  SmBoolean bFound;
                  SmBoolean bIsMulti;
                  SmStatus  eStat = m_cpSurface->DropPoint(sCrvPt, 
                                                           m_cpSurface->GetNaturalUVDomain(),
                                                           &sUV, 
                                                           bFound, sUVDrop, dTempDist, bIsMulti, SM_SO_MINIMIZE );
                  SmVector3d sDropPt, sDropNorm ;
                  double dDistToSurfNormLine ;
                  double dDropToSurfNormLine ; 
                  smgu_LineClosestPoint(sSrfPt, sNorm, sCrvPt, dDistToSurfNormLine) ;
                  m_cpSurface->EvaluatePoint(sUVDrop, sDropPt) ;
                  m_cpSurface->EvaluateNormal(sUVDrop, TRUE, TRUE, sDropNorm) ;
                  smgu_LineClosestPoint(sDropPt, sDropNorm, sCrvPt, dDropToSurfNormLine) ;

                  static constexpr double sdLimit = SM_ZONE_TOL_3D/10.0;
                  if ( eStat == SM_SUCCESS && bFound ) 
                    {
                      if ( smos_Fabs( dDist - dTempDist ) > sdLimit )
                        { SM_ASSERT_MSG( FALSE, _T("SmSurfaceTracer::FlushCurve() Max Drop Distance discrepancy")); }
                    }

SmBoolean bDebugMe = FALSE ;
                 if(bDebugMe)
                   {
                     SM_DUMP_AND_ASSERT_VALID(m_cpSurface) ;
                     SmPoint3d sDropPnt, sDropNormal ;
                     if(bFound) { m_cpSurface->EvaluatePoint(sUVDrop, sDropPnt) ;
                                  m_cpSurface->EvaluateNormal(sUVDrop, TRUE, TRUE, sDropNormal) ; 
                                }
                     SmTArray<SmCurve*> sTmpCurves ;
                     sm_FlushCrv( *m_cpContext, 3, s3DCtrlPts, sKnots, sTmpCurves ); // add new BSplineCurve3d to sTmpCurves
                     sm_FlushCrv( *m_cpContext, 2, sUVCtrlPts, sKnots, sTmpCurves ); // add new BSplineCurve2d to sTmpCurves

                     SmCurve    * pTmpCurve3d = sTmpCurves.GetSize() > 0 ? sTmpCurves[0] : NULL ;
                     SmCurve    * pTmpCurve2d = sTmpCurves.GetSize() > 1 ? sTmpCurves[1] : NULL ;
                     SmObjDelete  sClean1(pTmpCurve3d), sClean2(pTmpCurve2d) ;
                     if(FALSE)
                       { smgfx_Erase() ; }
                     smgfx_SetLook(1,2, 0,0,1) ; pOptCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
                     smgfx_SetLook(4,5, 0,0,1) ; sCrvPt.Draw() ; sm_GraphicsLoop() ;
                     smgfx_SetLook(2,5, 1,0,0) ; if(bFound) { sDropNormal.Draw(&sDropPnt) ; }
                     smgfx_SetLook(1,2, 0,1,0) ; if(pTmpCurve3d) pTmpCurve3d->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; 
                     smgfx_SetLook(3,4, .2,.2,.2) ; pTmpCurve3d->DrawInspectTwoCurves(pOptCurve) ; sm_GraphicsLoop() ;
                     smgfx_SetLook(1,2, 0,1,1) ; if(pTmpCurve2d) 
                       { SmCrvOnSurf sTrimCurve(*pTmpCurve2d, *(SmSurface*)m_cpSurface) ; sTrimCurve.Draw(NULL, TRUE) ; sm_GraphicsLoop() ; 
                         smgfx_SetLook(3,4, .2,.2,.2) ; sTrimCurve.DrawInspectTwoCurves(pOptCurve) ; sm_GraphicsLoop() ;
                       }
                     smgfx_SetLook(1,2, 0,1,1) ; m_cpSurface->DrawUV(12, 12, FALSE, NULL, TRUE) ; sm_GraphicsLoop() ; 
                     sm_GraphicsLoop() ; 
                   }
#endif // SM_DEBUG_CODE
                }

              // End Segment CrvPnt
              pOptCurve->EvaluatePoint( pTSPCurr->m_dCurveParameter, sCrvPt );

              // End Segment Drop distance
              sSrfPt = pTSPCurr->m_vSurfacePV[0][0];
              sNorm  = pTSPCurr->m_vSurfacePV[2][2];
              dDist  = smos_Fabs( ( sCrvPt - sSrfPt ).Dot( sNorm ) );
              if ( dDist > dMaxDropDist ) 
                { dMaxDropDist = dDist; }

            } // end pOptCurve existence check
        } // end scope for 2d points.
    } // end iter every remaining m_vCurvePoint building one Bezier segment per point

  // set output
  SER( sm_FlushCrv( *m_cpContext, 3, s3DCtrlPts, sKnots, r3DCurves ));        // add new BSplineCurve to r3DCurves
  SER( sm_FlushCrv( *m_cpContext, 2, sUVCtrlPts, sKnots, rSurfaceUVCurves )); // add new BSplineCurve to rSurfaceUVCurves
  rMaxDropDists.Add( dMaxDropDist  );
  rDeviations  .Add( dMaxDeviation );

  // all done
  return SM_SUCCESS;

} // end SmSurfaceTracer::FlushCurve

/*******************************************************************//**
PURPOSE: This method is the top level interface function for various
   tracing routines.

NOTES: This method currently only works on surfaces which are 
    at least G1 (smooth) within the corresponding surface domain.  It is 
    possible to intersect surfaces with discontinuities by making multiple
    calls to this method using different surface domains.
***********************************************************************/
SmStatus SmSurfaceTracer::DoTrace                // eff: Trace for SmSurfaceDropCurve and SmSurfaceSilhouette
 (const SmContext     & crContext,               // in : context for new object construction
  const SmApproxTol3d * pOptApproxTol3d,         // in : opt AppoxTol3d  val to store in m_dThisApproxTol3d
  const double        * pdOptAngTolRad,          // in : opt AngleTolRad val to store in m_dThisAngTolRad
  SmTArray<SmCurve*>  * p3DCurves,               // out: traced 3dcurves
  SmTArray<SmCurve*>  * pSurfaceUVCurves,        // out: traced UVCurves
  SmTArray<double>    * pMaxDropDists,           // out: max m_crCurveToDrop or SilhouetteCrv SmpPoint to SurfDropPt dist for every output curve 
  SmTArray<double>    * pDeviations)             // out: max m_crCurveToDrop or SilhouetteCrv SmpPoint to DropSurfNormLine dist for every output curve
{
  // store optional Tol values
  if ( pOptApproxTol3d ) { m_dThisApproxTol3d = *pOptApproxTol3d; }
  if ( pdOptAngTolRad ) { m_dThisAngTolRad    = *pdOptAngTolRad; }

  // locals
  SmSurfaceCache *pSC1 = smsurf_GetSurfaceCache(m_cpSurface); NER(pSC1);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      if ( FALSE )
        { smgfx_Erase(); }
      smgfx_SetLook( 1,1, 1,0,1 ); m_cpSurface->DrawUV(2,2); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Check to make sure internal surface continuities are better then C0
  if ( pSC1->GetSurfaceContinuity(m_vUVDomain) <= SM_CT_C0 )
    {
      SE( SM_ERR_INVALID_INPUT );  // Note, SER stops the entire process.
    }

  // check out cache pSC1 so it can't be deleted until sCheckIO goes out of scope
  SmCacheCheckOutIn sCheckIO(pSC1);

    { // Scope: Makes the change values destructors occur before surface cache is
      //        checked back in and possibly deleted.
    
      // Turn off point testing so GlobalPointSolve() will keep all point solutions
      //   without classifying the solution point against the trim boundaries.
      // Turn on boundary curve processing to force LocalSolve to look for drop points
      //   on boundary curves.
      SmTemporaryChangeValue<SmBoolean> sStack1(pSC1->m_bPointTestEnabled,FALSE);
      SmTemporaryChangeValue<SmBoolean> sStack3(pSC1->m_bProcessBoundaryCurves,TRUE);

      // set local context pointer
      m_cpContext = &crContext;

      // local Arrays
      SmTArray<SmCurve*> s3DCurves;        SmObjsDelete<SmCurve*> sCleanup3D(&s3DCurves);
      SmTArray<SmCurve*> sSurfaceUVCurves; SmObjsDelete<SmCurve*> sCleanupUV(&sSurfaceUVCurves); 
      SmTArray<double>   sMaxDropDists;     
      SmTArray<double>   sDeviations;
    
      // The first step in tracing is to find some start points on the boundary
      SER( FindBoundaryStartPoints( s3DCurves,         // out: 3d curves found by SmSurfaceSilhouett - not used by SmSurfaceDropCurve
                                    sSurfaceUVCurves,  // out: all curve intervals that drop to SurfaceIsoCurves 
                                    sMaxDropDists,     // out: max m_crCurveToDrop SmpPoint to SurfDropPt dist for every output curve 
                                    sDeviations ));    // out: max m_crCurveToDrop SmpPoint to DropSurfNormLine dist for every output curve

      // While m_vStartPoints has Pnts - trace 1 curve from each StartPnt
      while (m_vStartPoints.GetLastNode() != NULL)
        {
          SmTracePnt * pTSP = m_vStartPoints.GetLastNode(); NER(pTSP);
          m_vStartPoints.RemoveLast();

          // Init trace in tangent dir
          m_dCurveTraceDirection = 1.0;

          // trace curve: start in pos dir, switch to neg dir if needed
          if(SM_SUCCESS == TraceCurve(*pTSP,
                                       s3DCurves))
            {
              // reverse curves traced in neg direction
              if ( m_dCurveTraceDirection == -1.0 )
                { SER( ReverseCurveDirection()); }

              // output curve
              SER( FlushCurve( s3DCurves, 
                               sSurfaceUVCurves, 
                               sMaxDropDists, 
                               sDeviations ));
            } // end TraceCurve was a success check
        } // end m_vStartPoints has TracePnts to trace while loop

      // Now that we have found all boundary intersections,
      // look for non boundary intersections.
      SER( FindInteriorCurves( s3DCurves, sSurfaceUVCurves, sMaxDropDists, sDeviations ));

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          for (ULONG jjj=0; jjj<s3DCurves.GetSize(); jjj++) 
            { SmCurve * pCurve = s3DCurves[jjj];
              smgfx_SetLook(1,2, 1,0,0) ; pCurve->DrawWDeriv(pCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
            }
          sm_GraphicsLoop();
        }
#endif  // SM_DEBUG_CODE

      // when asked - save 3d traced curves - take them off the temp list
      if(p3DCurves)        { p3DCurves->Append( s3DCurves );
                             sCleanup3D.Clear();
                           }

      // when asked - save UV traced curves - take them off the temp list
      if(pSurfaceUVCurves) { pSurfaceUVCurves->Append( sSurfaceUVCurves );
                             sCleanupUV.Clear();
                           }

      // when asked - save MaxDropDistances and Deviations
      if(pMaxDropDists)   { pMaxDropDists->Append( sMaxDropDists ); }
      if(pDeviations)     { pDeviations->Append( sDeviations ); }

    } // end scope for TempChangeValues.

  // all done
  return SM_SUCCESS;
                       
} // end DoTrace

/*******************************************************************//**
PURPOSE: Reverse the orientation, parameterization and ordering of
    the curve being traced.

NOTES: If there are user fields which are dependent on orientation
    then you will need to implement this virtual method in the subclass
    of the specific tracer.
***********************************************************************/
SmStatus SmSurfaceTracer::ReverseCurveDirection()
{
    SmTracePnt *pLastPoint = m_vCurvePoints.GetLastNode();
    NER(pLastPoint);
    SmTracePnt *aData[200];
    SmTArray<SmTracePnt*> sPnts(200,aData);

    // Grab points in m_vCurvePoints list into an array, and empty the list.
    m_vCurvePoints.GetAllNodes(sPnts);
    m_vCurvePoints.Init();

    double dCurrParam = 0.0;
    ULONG i, lNumPts = sPnts.GetSize();
    for ( i=lNumPts; i>0; i-- )
    {
        SmTracePnt *pPnt = sPnts[i-1];
        pPnt->m_v3DCurvePV[1] = - pPnt->m_v3DCurvePV[1];
        pPnt->m_v3DCurvePV[2] = - pPnt->m_v3DCurvePV[2];
        pPnt->m_vUVCurvePV[1] = - pPnt->m_vUVCurvePV[1];
        pPnt->m_vUVCurvePV[2] = - pPnt->m_vUVCurvePV[2];
        double dDeltaParam = 0.0;
        if ( i > 1 )
        {
            dDeltaParam = pPnt->m_dCurveParameter - 
                    sPnts[i-2]->m_dCurveParameter;
        }
        pPnt->m_dCurveParameter = dCurrParam;
        dCurrParam = dCurrParam + dDeltaParam;
        m_vCurvePoints.Append( pPnt );
    }
    return SM_SUCCESS;

} // end ReverseCurveDirection

/*******************************************************************//**
PURPOSE: Compute the deviation of the span given by a Hermite with
    the actual theoretical curve.

NOTES: Note that this is pure virtual method that must be
    implemented by the subclasses of SmSurfaceTracer.
***********************************************************************/
SmStatus SmSurfaceTracer::ComputeSpanDeviation
 (const SmTracePnt & crCurrPnt,             // in :
  const SmTracePnt & crNextPnt,             // in :
  double             dStepSize,             // in :
  double           & rdDeviationFound,      // in :
  SmBoolean        & rbSatisfiesTolerances) // in :
{
    // Create two Hermite curves (parameter space and 3d) and test some points
    // on the curves as projected into 3d by their corresponding surfaces.
    SmHermiteCurve sHermiteUV(crCurrPnt.m_vUVCurvePV[0],
        crCurrPnt.m_vUVCurvePV[1]*dStepSize,
        crNextPnt.m_vUVCurvePV[0],
        crNextPnt.m_vUVCurvePV[1]*dStepSize,
        3);
    sHermiteUV.SetContext(NULL);

    SmHermiteCurve sHermite3D(crCurrPnt.m_v3DCurvePV[0],
        crCurrPnt.m_v3DCurvePV[1]*dStepSize,
        crNextPnt.m_v3DCurvePV[0],
        crNextPnt.m_v3DCurvePV[1]*dStepSize,
        3);
    sHermite3D.SetContext(NULL);

#define NUM_TEST_POINTS 5
    double dMaxDist = 0.0;
    double dTolSq = m_dThisApproxTol3d * m_dThisApproxTol3d;
    ULONG i;
    for ( i=0; i<NUM_TEST_POINTS; i++ )
    {
        double dParam = (i+1.0) / (NUM_TEST_POINTS+1.0);
        SmPoint3d sCrvPnt;
        SER( sHermiteUV.EvaluatePoint( dParam, sCrvPnt ));
        SmPoint2d sUV( sCrvPnt.x, sCrvPnt.y );
        sUV = m_vUVDomain.ClampPoint2d( sUV );
        SER( m_cpSurface->EvaluatePoint( sUV, sCrvPnt ));

        SmPoint2d sUVFound;
        SmBoolean bFoundAnswer;
        SmTracePnt sDummyTP;
        SER( LocalPointSolve( crCurrPnt, sUV, sDummyTP, bFoundAnswer, sUVFound ));
        if ( !bFoundAnswer )
        {
            rbSatisfiesTolerances = FALSE;
            return SM_SUCCESS;
        }
        SmPoint3d sCrvPnt2, sCrvPnt3;
        SER( m_cpSurface->EvaluatePoint( sUVFound, sCrvPnt2 ));
        SER( sHermite3D.EvaluatePoint( dParam, sCrvPnt3 ));
        double dDistSq = sCrvPnt3.DistanceBetweenSquared( sCrvPnt2 );
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(1,0,0);
            sCrvPnt.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,1);
            sCrvPnt2.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(1,0,1);
            sCrvPnt3.Draw();
            sm_GraphicsLoop();
            sHermite3D.DrawWDeriv(sHermite3D.GetNaturalInterval(),0);
        }
#endif
        if ( dDistSq > dMaxDist )
        {
            dMaxDist = dDistSq;
            if ( dMaxDist > dTolSq )
            {
                rbSatisfiesTolerances = FALSE;
                return SM_SUCCESS;
            }
        }
    }

    rdDeviationFound = smos_Sqrt( dMaxDist );
    rbSatisfiesTolerances = TRUE; // passed test

    return SM_SUCCESS;

} // end ComputeSpanDeviation

/*******************************************************************//**
PURPOSE: Compute the starting points of the curves which cross the
         boundary of the surface.

NOTES: accumulates StartPoints in m_vStartPoints
       virtual method used by SmSurfaceSilhouette
       virtual method overloaded by SmSurfaceDropCurve
***********************************************************************/
SmStatus SmSurfaceTracer::FindBoundaryStartPoints
 (SmTArray<SmCurve*> & r3DCurves,        // out: 3d curves found by SmSurfaceSilhouett - not used by SmSurfaceDropCurve
  SmTArray<SmCurve*> & rSurfaceUVCurves, // out: all curve intervals that drop to SurfaceIsoCurves 
  SmTArray<double>   & rMaxDropDists,    // out: max m_crCurveToDrop SmpPoint to SurfDropPt dist for every output curve 
  SmTArray<double>   & rDeviations)      // out: max m_crCurveToDrop SmpPoint to DropSurfNormLine dist for every output curve
{
  // locals
  SmBSplineCurve * pData[4];
  ULONG                       lSide, lCoord;
  SmBoolean                   bCurvesAreCached;
  SmTArray<SmBSplineCurve*>   sIsoCurves(4,pData);
  SmSurfaceCache            * pSC     = smsurf_GetSurfaceCache(m_cpSurface); NER(pSC);

  // Get the four natural boundary curves of the surface.
  SER( pSC->GetIsoBoundaryCurves(m_vUVDomain, 
                                 m_dThisApproxTol3d, 
                                 bCurvesAreCached, 
                                 sIsoCurves ));

  // Automatic clean up of non-cached curves
  SmTArray<SmBSplineCurve*>   * pCurves = bCurvesAreCached ? NULL : &sIsoCurves ;
  SmObjsDelete<SmBSplineCurve*> sCleanupCurves(pCurves);

  // Loop over four Surface Natural Bounding edges: lSide = low/high, lCoord = u/v - looking for StartPnts
  for(lSide=0;lSide<=1;lSide++)
    {
      double dNormParam = lSide;

      // for U and V directions: lSide = low/high, lCoord = u/v
      for(lCoord=0;lCoord<=1;lCoord++)
        {
          SmSurfParamType  eSurfParam = (lCoord==0) ? SM_SP_U : SM_SP_V;
          SmBSplineCurve * pNewBSC    = sIsoCurves[lSide*2+lCoord] ; NER(pNewBSC);
          SmExtent1d       sBSCIvl(m_vUVDomain.GetMin()[1-lCoord],m_vUVDomain.GetMax()[1-lCoord]);
          SmSolution       aData[20];
          SmSolutionArray  sSolutions(20,aData);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_SetLook(1,3, 1,0,0); pNewBSC->DrawWDeriv(sBSCIvl,0); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // skip degen IsoCurves
          if ( pNewBSC->IsDegenerate() )
            { continue; }

          // Check whether the entire boundary curve is a solution.
          SmBoolean bCurveIsSilhouette;
          double dParamVal = m_vUVDomain.Evaluate(dNormParam,dNormParam)[lCoord];
          SER(TestIsoCurve(eSurfParam,dParamVal,bCurveIsSilhouette));

          // when entire BoundaryCurve is a solution
          if(bCurveIsSilhouette)
            {
              // get near mid-Pnt
              SmPoint3d sTestPnt;
              double    dTestParam = sBSCIvl.Evaluate(0.45678);
              SER(pNewBSC->EvaluatePoint(dTestParam,sTestPnt));

              // If we already have this curve then skip it.
              if (IsPointOnCurve(sTestPnt, r3DCurves)) // TRUE = Pnt near cr3DCurve, tol = m_dThisApproxTol3d*(4.0 or 10.0)
                { continue; }

              // Add a copy of this solution curve to r3DCurves.
              SmBSplineCurve *pBSCResult = new (*m_cpContext) SmBSplineCurve(*pNewBSC);
              NER(pBSCResult);
              r3DCurves.Add(pBSCResult);

              // locals for upcoming DropCurve() call
              double dDistToSurf, dDeviation;
              SmTArray<SmBSplineCurve*> sUVCurves;

//cbi: we know this is a boundary iso-curve:
              SER(m_cpSurface->DropCurve(*m_cpContext,
                                          m_vUVDomain,
                                         *pBSCResult,
                                          pBSCResult->GetNaturalInterval(),
                                          m_dThisApproxTol3d, 
                                          dDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                          dDeviation,    // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                          sUVCurves,
                                          TRUE, NULL, FALSE));

              // When Curve dropped along a seam - save the current target side solution - delete the other
              if (sUVCurves.GetSize() == 2) 
                {
                  rSurfaceUVCurves.Add(sUVCurves[lSide]); 
                  SM_ASSERT(sUVCurves[1-lSide] != NULL) ; delete sUVCurves[1-lSide] ; sUVCurves[1-lSide] = NULL ;
                }
              else if (sUVCurves.GetSize() == 1) // save the solution
                {
                  rSurfaceUVCurves.Add(sUVCurves[0]); 
                }
              else // report an error
                {
                  SER(SM_ERR);
                }

              // set other outputs
              rMaxDropDists.Add(dDistToSurf);
              rDeviations  .Add(dDeviation);
            } // end entire boundary curve was a solution check

          // Note, we have to do the next check even if the whole curve
          // is a silhouette: picture a torus edge-on.  [B100]

          // Find curve start points along the curve.
          SER(FindSolutionsOnBoundaryCurve(*pNewBSC,   
                                            sBSCIvl,
                                            lSide,
                                            eSurfParam,
                                            sSolutions));

          // locals                                              
          ULONG     i, lNumSols = sSolutions.GetSize();
          SmPoint2d sUV;

          // for every solution
          for(i=0;i<lNumSols;i++)
            {
              SmSolution & rSol = sSolutions[i];
              sUV[lCoord]       = m_vUVDomain.Evaluate(dNormParam,dNormParam)[lCoord];
              sUV[1-lCoord]     = rSol.m_vStart[0];

              // when solution lies on solution curve - save it
              if ( DoesPointLieOnCurve( sUV, TRUE ) )
                {
                  SmTracePnt * pTSP     = (SmTracePnt*)m_vTrcPntMgr.GetNewElement();
                  pTSP->m_ePointType    = SM_TP_NORMAL;
                  pTSP->m_vUVCurvePV[0] = sUV;
                  AddStartPoint(*pTSP);
                }

              // for range solutions
              if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                {
                  sUV[lCoord]   = m_vUVDomain.Evaluate(dNormParam,dNormParam)[lCoord];
                  sUV[1-lCoord] = rSol.m_vEnd[0];


                  // when solution lies on solution curve - save it
                  if ( DoesPointLieOnCurve( sUV, TRUE ) )
                    {
                      SmTracePnt * pTSP     = (SmTracePnt*)m_vTrcPntMgr.GetNewElement();
                      pTSP->m_ePointType    = SM_TP_NORMAL;
                      pTSP->m_vUVCurvePV[0] = sUV;
                      AddStartPoint(*pTSP);
                    }
                } // end range soltuion check
            } // end iter every solution from FindSolutionsOnBoundaryCurve() - adding sols to m_vStartPoints
        } // end iter each coord (U/V)
    } // end iter each side (low/high)

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmTArray<SmTracePnt*> sActiveElem;
      m_vStartPoints.GetAllNodes(sActiveElem);

      for (ULONG i=0; i<sActiveElem.GetSize(); i++) 
        {
          SmTracePnt * pTSP = sActiveElem[i];
          SmPoint3d    sP1;
          m_cpSurface->EvaluatePoint( pTSP->m_vUVCurvePV[0],sP1 );

          smgfx_SetLook( 2,5, 1,0,0); sP1.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end FindBoundaryStartPoints

/*******************************************************************//**
PURPOSE: Determine if the isoparametric curve on the surface 
    satisfies the tracing conditions.
    
NOTES: 
    Also note that this interface does not work well for the situation
    where a boundary curve has only segments which satisfy the tracing
    condition.  It is an all or nothing proposition.
***********************************************************************/
SmStatus SmSurfaceTracer::TestIsoCurve
 (SmSurfParamType eSurfParam,            // in : oneof SM_SP_U=const U Cuve or SM_SP_V=const V Curve
  double          dParam,                // in : constant param value
  SmBoolean     & rbBoundaryIsSolution)  // out: TRUE = IsoParamCurve is solution, FALSE=not
{
#define NUM_ISO_TEST_POINTS 10

  // init output
  rbBoundaryIsSolution = FALSE;

  // get IsoParamCurve min and max UVPnts
  SmPoint2d sUV1( m_vUVDomain.GetMin() );
  SmPoint2d sUV2( m_vUVDomain.GetMax() );

  // Constant U curve
  if ( eSurfParam == SM_SP_U ) { sUV1.Set( dParam, sUV1.y );
                                 sUV2.Set( dParam, sUV2.y );
                               }
  else /* Constant V curve */  { sUV1.Set( sUV1.x, dParam );
                                 sUV2.Set( sUV2.x, dParam );
                               }
  // locals
  ULONG i;
  SmExtent2d sUVIso( sUV1, sUV2 );

  // for every sample point
  for(i=0;i<NUM_ISO_TEST_POINTS;i++)
    {
      double dT = i / (NUM_ISO_TEST_POINTS-1.0);
      if (dT > 1.0) { dT = 1.0; }
      SmPoint2d sUV = sUVIso.Evaluate(dT,dT);

      // Check whether this uv point lies on the solution curve being traced.
      if(!DoesPointLieOnCurve( sUV, FALSE ) ) // FALSE = don't refine it.
        {
          return SM_SUCCESS;
        }
    } // end iter every sample point

  // Each sample point passed, so it must be a solution curve.
  rbBoundaryIsSolution = TRUE;
  return SM_SUCCESS;

} // end SmSurfaceTracer::TestIsoCurve

/*******************************************************************//**
PURPOSE: Find and trace interior curves.

NOTES: For this method to work. Subclasses of the surface tracer
   should implement BranchMayContainAnswers.
***********************************************************************/
SmStatus SmSurfaceTracer::FindInteriorCurves
 (SmTArray<SmCurve*> & r3DCurves,        // i/o: already-found sols; Added to with new solutions
  SmTArray<SmCurve*> & rSurfaceUVCurves, // i/o: already-found sols; Added to with new solutions
  SmTArray<double>   & rMaxDropDists,    // i/o: already-found sols; Added to with new solutions
  SmTArray<double>   & rDeviations)      // i/o: already-found sols; Added to with new solutions
{
  // Find node pairs in the tree
  SmSolution      aData[100];
  SmSolutionArray sSolutions(100,aData);

  // Find pairs of tree nodes containing possible solutions from which StartPoints can be found
  SER( SolveIt( SM_SO_FIND, SM_SR_NODES, 0.0, 0.0, NULL, sSolutions ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3) 
    {
      sSolutions.Dump();
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG    i, lNumSols = sSolutions.GetSize();
  SmStatus eStat;

  // Now for each node pair do a test to see if produces a unique start point
  // Note that our first pass at this is naive and slow.
  for (i=0; i<lNumSols; i++)
    {
      SmSolution    & rSol       = sSolutions[i];

      // Surface TreeNode locals for this solution
      SmTreeNode    * pNode1     = rSol.m_apNodes[0];
      SM_ASSERT(pNode1->m_eAuxDataType == SM_AD_BEZIER_SURFACE);
      SmBezierPatch * pBezPatch1 = (SmBezierPatch*)pNode1->m_pData; 

      // skip unexpected cases in which TreeNode has no BezPatch
      if (pBezPatch1 == NULL )
        { continue; }

      const SmExtent2d &rUVDomain1 =  pBezPatch1->GetUVDomain();

      // skip TreeNodes disjoint from trace surface's specified UVSubDomain
      if ( rUVDomain1.AreDisjoint( m_vUVDomain ) )
        { continue; }

#ifdef SM_DEBUG_CODE
      const SmSurface  &sBSS1 = *pBezPatch1->mBA_pSurface;

SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          SmPoint3d sPnt;
          sBSS1.EvaluatePoint(rUVDomain1.Evaluate(0.5,0.5), sPnt);

          sm_GraphicsLoop();
          smgfx_ChangeColor(i != 0); sBSS1.DrawUV(3,3); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); pBezPatch1->GetPolarBox().Draw(sPnt); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // First let's do a fast test to see if the intersection of
      // these two nodes is close to any of the new 3D curves that
      // we have found.  If we pass this
      // test then we can go onto a little more stringent test.

      // first test locals
      ULONG   j, lNumCurves = r3DCurves.GetSize();
      SmBoolean  bIsNearCurve = FALSE;
      SmExtent3d sIntBBox = pNode1->m_sBBox;
      sIntBBox.ExpandAbsolute(m_dThisApproxTol3d);

      // for every 3D curve
      for (j=0; j<lNumCurves; j++)
        {          
          SmCurve      * pCurve = r3DCurves[j];

          // get curve->Cache
          SmCurveCache * pCC    = (SmCurveCache*) SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE, pCurve); 

          // skip unexpected curves withough curve caches
          if ( pCC == NULL )
            { continue; }

          SmTree *pTree = pCC->GetTree();

          // remember when curve->Cache BBox intersects Surface->Node BBox
          if ( pTree->IntersectsBox( sIntBBox ) )
            {
              bIsNearCurve = TRUE;
              break;
            }
        } // end iter every 3D curve

      // skip cases where curve->Cache BBox is near an existing Curve
      if ( bIsNearCurve ) 
        { continue; }
        
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2) 
        {
          SmPoint3d sPnt;
          sBSS1.EvaluatePoint(rUVDomain1.Evaluate(0.5,0.5),sPnt);

          sm_GraphicsLoop();
          smgfx_ChangeColor(j != 0); sBSS1.DrawUV(3,3); sm_GraphicsLoop();
          smgfx_SetColor(0,0,1); pBezPatch1->GetPolarBox().Draw(sPnt); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      SmPoint2d sMiddle = rUVDomain1.Evaluate(0.5,0.5);

      // Refine sMiddle to see if we can find a point on the curve
      SmTracePnt sPnt1, sPnt2;
      SmBoolean  bFound = FALSE;
      SmPoint2d  sFoundUV;
      sPnt1.m_v3DCurvePV[1].Set(0,0,0);
      eStat = LocalPointSolve( sPnt1, sMiddle, sPnt2, bFound, sFoundUV );
      if ( !bFound  ||  eStat != SM_SUCCESS )
        { continue; }  // No point found continue to next node

      // Now test start point to see if it drops to any of the existing 3D curves.
      SmPoint3d sSurfPnt;
      eStat = m_cpSurface->EvaluatePoint( sFoundUV, sSurfPnt );
      if ( eStat != SM_SUCCESS )
        { continue; }
      if ( IsPointOnCurve( sSurfPnt, r3DCurves )) // TRUE = Pnt near cr3DCurve, tol = m_dThisApproxTol3d*(4.0 or 10.0)
        { continue; }
            
      // If we made it this far then we have a point which should serve as
      // a good start point.  Load it up and trace it.
      SmTracePnt *pStartTSP = (SmTracePnt*)m_vTrcPntMgr.GetNewElement();
      pStartTSP->m_ePointType = SM_TP_NORMAL;

      // Given           : SurfacePoint and CurvePoint, 
      // Compute and Save: - SurfacePoint pos, 1st Derivs, and SurfaceNormal,
      //                   - DropCurve   3d pos, tangent, and  
      //                   - DropUVCurve 2d pos, tangent.
      if ( ComputePointValues( sFoundUV, *pStartTSP ) != SM_SUCCESS )
        { continue; }

      m_pStartPoint = pStartTSP;
      m_dCurveTraceDirection = 1.0;  // Forward
      eStat = TraceCurve( *pStartTSP, r3DCurves );
      if ( eStat != SM_SUCCESS )
        { continue; }

      // Trace the other direction if appropriate.
      if ( !m_bCurveIsClosed && pStartTSP->m_ePointType != SM_TP_SINGULARITY )
        {
          eStat = ReverseCurveDirection();
          // Note, if this failed, we want to call FlushCurve(),
          // mainly for its side effect of clearing out m_vCurvePoints,
          // but also to create a curve form whatever was found.
          // But then if the subsequent TraceCurve and ReverseCurveDirection
          // calls fail, the code will just continue.
          // (Formerly SER, which would abort the entire operation.)
          if ( eStat != SM_SUCCESS )
            {
              FlushCurve( r3DCurves, rSurfaceUVCurves, rMaxDropDists, rDeviations );
              continue;
            }

          m_dCurveTraceDirection = -1.0;
          pStartTSP              = m_vCurvePoints.GetLastNode();
          TraceCurve( *pStartTSP, r3DCurves );
          ReverseCurveDirection();
          m_dCurveTraceDirection = 1.0;
        }

      FlushCurve( r3DCurves, rSurfaceUVCurves, rMaxDropDists, rDeviations );
      m_pStartPoint = NULL;

    } // end iter each cache node solution looking for interior StartPoints to trace

  return SM_SUCCESS;

} // end SmSurfaceTracer::FindInteriorCurves

/*******************************************************************//**
PURPOSE: Compute the point and derivative values of the point.

NOTES: This is a pure virtual method that must be implemented
       by subclasses. Each subclass has to solve the problem:

  Given           : SurfacePoint and CurvePoint or its equivalent, 
  Compute and Save: - SurfacePoint pos, 1st Derivs, and SurfaceNormal,
                    - DropCurve   3d pos, tangent, and  
                    - DropUVCurve 2d pos, tangent.
***********************************************************************/
SmStatus SmSurfaceTracer::ComputePointValues
 (const SmPoint2d &, // in : target SurfacePoint
  SmTracePnt      &, // i/o: target CurvePoint, set with SurfacePoint properties
  SmTracePnt      *, // in : Last intersection point on curve being stepped out, NULL to ignore
                     //      When supplied used to handle singularity cases.
                     //      default:[NULL]
  double          *) // in : pdStepSize = NOT USED
                     //      distance to step back from singularities to try and find 
                     //      a nearby neighbor to use to computePointValues, NULL to ignore 
                     //      default:[NULL] 
{
  SE(SM_ERR);
  return SM_ERR;
} // SmSurfaceTracer::ComputePointValues

/*******************************************************************//**
PURPOSE: Compute the actual UV point given a guess UV point on the
    curve being traced.

NOTES: This is a pure virtual method that must be implemented
    by subclasses.
***********************************************************************/
SmStatus SmSurfaceTracer::LocalPointSolve
 (const SmTracePnt &,
  const SmPoint2d  &,
  SmTracePnt       &,
  SmBoolean        &,
  SmPoint2d        &) 
 const
{
  SE(SM_ERR);
  return SM_ERR;
} // end SmSurfaceTracer::LocalPointSolve

/*******************************************************************//**
PURPOSE: Find potential start points along a specified boundary curve.

NOTES: Note that the solutions will be refined by the calling
   routine.  This method does not need to find exact solutions but it
   would be good if it found solutions near the actual solutions on the
   surface.  

   Also note that this method is a pure virtual method and must be 
   overridden when subclassing SmSurfaceTracer.
***********************************************************************/
SmStatus SmSurfaceTracer::FindSolutionsOnBoundaryCurve
 (const SmBSplineCurve &,
  const SmExtent1d     &,
  ULONG , 
  SmSurfParamType ,
  SmSolutionArray      & ) 
 const
{
  SE(SM_ERR);
  return SM_ERR;

} // end SmSurfaceTracer::FindSolutionsOnBoundaryCurve  

/*******************************************************************//**
PURPOSE: Determine if the given point is a silhouette point on the
   surface.  Note that not all curves silhouettes are also surface
   silhouettes.

NOTES: This method is a pure virtual method and must be 
   overridden when subclassing SmSurfaceTracer.
***********************************************************************/
SmBoolean SmSurfaceTracer::DoesPointLieOnCurve
 (SmPoint2d & , SmBoolean )
 const
{
  SE(SM_ERR);
  return FALSE;

} // end SmSurfaceTracer::DoesPointLieOnCurve

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfaceTracer::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfaceTracer_TYPE == t) ? TRUE : SmGlobalSolver::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print   

NOTES: 
***********************************************************************/
void SmSurfaceTracer::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmSurfaceTracer::Dump()")) ;

  // dump base
  SmGlobalSolver::Dump() ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmSurfaceTracer::Dump()\n")) ;

} // end SmSurfaceTracer::Dump

