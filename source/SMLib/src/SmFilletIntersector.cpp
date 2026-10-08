// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFilletIntersector.cpp
* PURPOSE: This file contains surface intersector methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmFilletIntersector.h>
#include <SmGeomUtility.h>
#include <SmFilletExecutive.h>


/*******************************************************************//**
PURPOSE: Construct a fillet intersector object.

NOTES:
***********************************************************************/
SmFilletIntersector::SmFilletIntersector
  (const SmSurface  & crSurface1,
   const SmExtent2d & crUVDomain1,
   const SmSurface  & crSurface2,
   const SmExtent2d & crUVDomain2,
   SmFilletSolver   & rFilletSolver,                                           
   SmFilletGeom     * pCurrFilletGeom)                                         
 : SmAdvSurfaceIntersector(crSurface1,                                         
                           crUVDomain1,   
                           crSurface2,
                           crUVDomain2,
                           TRUE),         // TRUE = Constructing a FilletIntersector object.
                                          // GWC: It seems, SmFilletIntersector manages the intersection of 
                                          //   extended surfaces so that these crUVDomains are expected to exceed
                                          //   the size of these Surface->NaturalUVDomains.  Setting this flag 
                                          //   value to TRUE tells the SmSurfaceIntersector constructor to expect and
                                          //   allow that for this case. SmFilletIntersector documentation does not 
                                          //   yet explain how these extended UVDomains are exploited. 
                                          //   TODO: add more fillet documentation.
     m_rFilletSolver(rFilletSolver), 
     m_pCurrFilletGeom(pCurrFilletGeom),
     m_bCurrFilletTouchBoundary(FALSE), 
     m_bSelfIntersect(FALSE)
{
    m_cpContext = rFilletSolver.GetContext();
    // Will not try to extend boundary intersection point to catch
    // an end point which is also on the boundary. i.e. deactivate this feature
    m_bDoBoundaryPointOnCurveTest = FALSE;
}

/*******************************************************************//**
PURPOSE: Compute the point and derivative values of the point.

NOTES:
***********************************************************************/
SmStatus SmFilletIntersector::ComputePointValues
  (SmPoint2d aUVValues[2],                       // in : rail UVcurve point values
   SmTsectPnt & rTsectPnt,                       // out: intersection point gets updated surface values
   SmTsectPnt * pOptPreviousPnt,                 // in : last intersection point
   double * )                                    // in : pdStepSize = Not used in this function

{
  // pass the call along to the
  if (SM_SUCCESS != m_rFilletSolver.ComputePointValues
           (aUVValues,                 // in : rail UVcurve point values
            m_vUVDomain[0],            // in : fillet surface1 UVDomain
            m_vUVDomain[1],            // in : fillet surface2 UVDomain
            m_dCurveTraceDirection,    // in : offset-surface xsect curve trace direction [+/-1]
            rTsectPnt,                 // out: intersection point gets updated surface values
            pOptPreviousPnt))          // in : last intersection point
     { return SM_ERR; }

  return SM_SUCCESS;

} // end SmFilletIntersector::ComputePointValues

/*******************************************************************//**
PURPOSE: Compute the Surface and Curve point and derivative values 
         associated with the given rail curve Surface UVPoints.

NOTES: rTsectPnt.m_ePointType = SM_IP_CROSSING
       rTSectPnt.CurveDer()   = curve[param deltaParam] chord
       rTSectPnt.UVVectors    = set

       These values not set in rDeltaPnt
***********************************************************************/
SmStatus SmFilletIntersector::ComputePointValuesDelta
 (SmPoint2d    aUVValues[2],      // in : TgtParam      rail Surf UVPt values to evaluate
  SmPoint2d    aUVValuesDelta[2], // in : TgtDeltaParam rail Surf UVPt values to evaluate
  SmTsectPnt & rTsectPnt,         // out: container for TgtParam surface and Crv 3dPt, UVPos and derivative values
  SmTsectPnt & rDeltaPnt)         // out: container for TgtDeltaParam surface and Crv 3dPt, UVPos and derivative values
{
  // given m_pSurface[0,1] UV params - compute Surface pt, Crv pos, and deriv values
  SER(m_rFilletSolver.ComputeSurfaceValues(aUVValues,      // in : rail UVPoint values to evaluate
                                           rTsectPnt));    // out: container for surface point, position and derivative values
  SER(m_rFilletSolver.ComputeSurfaceValues(aUVValuesDelta, // in : rail UVPoint values to evaluate
                                           rDeltaPnt));    // out: container for surface point, position and derivative values

  // get curve[param deltaParam] chord (tangent approximation)
  SmVector3d sDiff = rDeltaPnt.CrvPos() - rTsectPnt.CrvPos();

  // when needed - negate the chord to match the parameter progression
  if (rDeltaPnt.m_dCurveParameter < rTsectPnt.m_dCurveParameter) 
    { sDiff = - sDiff; }

  // skip zero length chords
  if (sDiff.Length() < SM_EFF_ZERO) 
    { return SM_ERR; }

  // unitize and save the chord
  SER(sDiff.Unitize());
  rTsectPnt.CrvDeriv() = sDiff;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe)
    {
      smgfx_SetLook(1,1, 1,0,0);
      rDeltaPnt.CrvPos().Draw(); sm_GraphicsLoop(); sm_GraphicsLoop();
      rTsectPnt.CrvPos().Draw(); sm_GraphicsLoop(); sm_GraphicsLoop();
      rTsectPnt.CrvDeriv().Draw(&rTsectPnt.CrvPos()); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // set output
  rTsectPnt.m_ePointType = SM_IP_CROSSING;

  SER(m_rFilletSolver.ComputeUVVectors(rTsectPnt));
  
  // all done
  return SM_SUCCESS;

} // end SmFilletIntersector::ComputePointValuesDelta

/*******************************************************************//**
PURPOSE: Test the accuracy of a given span in terms of the 3D curves
    being on the surface and the satisfaction of angle tolerances.

NOTES:
***********************************************************************/
SmStatus SmFilletIntersector::TestSpanAccuracy
 (SmTsectPnt & rTsectPnt,         // in :
  SmTsectPnt & rNextTSP,          // in :
  SmTsectPnt * pOptMidPnt,        // in :
  SmBoolean  & rbFoundGoodPoint,  // out:
  double     & rdDeviationFound,  // NotUsed: out:
  double     & rdAngleFoundRad)   // out:
{
  SM_REF1(rdDeviationFound) ;
    double dStepSize = rNextTSP.m_dCurveParameter - rTsectPnt.m_dCurveParameter;

    if (rNextTSP.CrvDeriv().LengthSquared() < SM_EFF_ZERO_SQ) {
        SER(SM_ERR);
    }

    SER(rTsectPnt.CrvDeriv().AngleBetween(rNextTSP.CrvDeriv(),rdAngleFoundRad));
    // First do a quick angular test to see if we can reject this span
    if (rdAngleFoundRad > m_dThisAngTolRad && dStepSize > SM_EFF_ZERO*100.0) {
        if (rNextTSP.m_ePointType == SM_IP_SINGULARITY) {
            rNextTSP.m_ePointType = SM_IP_CROSSING;
        }
        rbFoundGoodPoint = FALSE;
        return SM_SUCCESS;
    }

    SmHermiteCurve sUVCrv0(rTsectPnt.UVPos(0),
                           rTsectPnt.UVDeriv(0)*dStepSize,
                           rNextTSP.UVPos(0),
                           rNextTSP.UVDeriv(0)*dStepSize,
                           2);
    sUVCrv0.SetContext(NULL);

    SmHermiteCurve sUVCrv1(rTsectPnt.UVPos(1),
                           rTsectPnt.UVDeriv(1)*dStepSize,
                           rNextTSP.UVPos(1),
                           rNextTSP.UVDeriv(1)*dStepSize,
                           2);
    sUVCrv1.SetContext(NULL);

    // First set surfaces back to the base surfaces if needed - this
    // makes the offsets zero.
    m_rFilletSolver.SetSurfacesToZeroOffset();

    SmCrvOnSurf sMap0(sUVCrv0,SM_CONST_CAST(SmSurface&,*m_cpSurface[0]),
                      &m_vUVDomain[0], 0, NULL);
//    sMap0.SetContext(NULL);

    SmCrvOnSurf sMap1(sUVCrv1,SM_CONST_CAST(SmSurface&,*m_cpSurface[1]),
                      &m_vUVDomain[1], 0, NULL);
//    sMap1.SetContext(NULL);

    SmPoint3d sPV0[2], sPV1[2];

    SER(sMap0.Evaluate(sMap0.GetNaturalInterval().GetMin(),1,TRUE,sPV0));
    SER(sMap0.Evaluate(sMap0.GetNaturalInterval().GetMax(),1,TRUE,sPV1));
#if 0
    if (bAdjustDerivatives) {
        SmPoint3d sPV0_A[2], sPV1_A[2];
        SmExtent1d sIvl = sMap0.GetNaturalInterval();
        SER(sMap0.Evaluate(sIvl.Evaluate(0.001),1,TRUE,sPV0_A));
        SER(sMap0.Evaluate(sIvl.Evaluate(0.999),1,TRUE,sPV1_A));
        sPV0[1] = sPV0_A[1];
        sPV1[1] = sPV1_A[1];
    }
#endif
    SmHermiteCurve s3DCrv0(sPV0[0],sPV0[1],sPV1[0],sPV1[1]);
    s3DCrv0.SetContext(NULL);
    SER(sMap1.Evaluate(sMap1.GetNaturalInterval().GetMin(),1,TRUE,sPV0));
    SER(sMap1.Evaluate(sMap1.GetNaturalInterval().GetMax(),1,TRUE,sPV1));
#if 0
    if (bAdjustDerivatives) {
        SmPoint3d sPV0_A[2], sPV1_A[2];
        SmExtent1d sIvl = sMap1.GetNaturalInterval();
        SER(sMap1.Evaluate(sIvl.Evaluate(0.001),1,TRUE,sPV0_A));
        SER(sMap1.Evaluate(sIvl.Evaluate(0.999),1,TRUE,sPV1_A));
        sPV0[1] = sPV0_A[1];
        sPV1[1] = sPV1_A[1];
    }
#endif

    SmHermiteCurve s3DCrv1(sPV0[0],sPV0[1],sPV1[0],sPV1[1]);
    s3DCrv1.SetContext(NULL);

#define NUM_TEST_POINTS 5
    double dParamsToTest[NUM_TEST_POINTS];
    dParamsToTest[0] = 0.5;
    dParamsToTest[1] = 0.17;
    dParamsToTest[2] = 0.83;
    dParamsToTest[3] = 0.33;
    dParamsToTest[4] = 0.67;

    double dMaxDistSq = 0.0;
    // Utilize the angle between the normals to adjust the tolerance.
    // This is required to get the approximation to with tolerance of the
    // TRUE implicit curve.  The previous method just found the answer
    // to where the points on the surfaces were within tolerance.
    double dMinTanAngle = smos_Min(rTsectPnt.m_dTangentPlaneAngleRad,
                                   rNextTSP.m_dTangentPlaneAngleRad);
    double dAdjustedTolerance = 2.0 * m_dThisApproxTol3d * smos_Sine(dMinTanAngle/2.0);
    // Don't let the adjusted tolerance get to be less then 1/10th of the actual
    // tolerance.  Otherwise we could get nearly infinite numbers of points near
    // tangency and singularity points.
    dAdjustedTolerance = smos_Max(dAdjustedTolerance,m_dThisApproxTol3d/1.0);
    // Don't let points get farther away than the tolerance either
    dAdjustedTolerance = smos_Min(m_dThisApproxTol3d,dAdjustedTolerance);

    double dTolSq = dAdjustedTolerance*dAdjustedTolerance;
    if (pOptMidPnt) {
        SmPoint3d sTestMid1, sTestMid2;
        SER(m_cpSurface[0]->EvaluatePoint( pOptMidPnt->UVPos(0), sTestMid1 ));
        SER(m_cpSurface[1]->EvaluatePoint( pOptMidPnt->UVPos(1), sTestMid2 ));
        SmPoint3d sCrvPnt0, sCrvPnt1;
        SmSolution sSol;
        SmBoolean bFoundAnswer;
        SER(s3DCrv0.LocalPointSolve(s3DCrv0.GetNaturalInterval(),SM_SO_MINIMIZE,
            sTestMid1,NULL,NULL,NULL,0.5,bFoundAnswer,sSol));
        if (!bFoundAnswer) { sSol.m_vStart[0] = 0.5; }
        SER(s3DCrv0.EvaluatePoint(sSol.m_vStart[0],sCrvPnt0));

        SER(s3DCrv1.LocalPointSolve(s3DCrv1.GetNaturalInterval(),SM_SO_MINIMIZE,
            sTestMid2,NULL,NULL,NULL,0.5,bFoundAnswer,sSol));
        if (!bFoundAnswer) { sSol.m_vStart[0] = 0.5; }
        SER(s3DCrv1.EvaluatePoint(sSol.m_vStart[0],sCrvPnt1));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3=FALSE;
        if (bDebugMe3) {
            smgfx_SetLook(1,1, 1,0,0);
            m_cpSurface[0]->DrawUV(2,2); sm_GraphicsLoop();
            m_cpSurface[1]->DrawUV(2,2); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,0,1);
            s3DCrv0.Draw(); sm_GraphicsLoop();
            sMap0.Draw();   sm_GraphicsLoop();
            s3DCrv1.Draw(); sm_GraphicsLoop();
            sMap1.Draw();   sm_GraphicsLoop();
            sTestMid1.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,1,0); sCrvPnt0.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,0,1); sTestMid2.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 1,1,0); sCrvPnt1.Draw();
            sm_GraphicsLoop();
        }
#endif
        double dBigTolSq = dAdjustedTolerance*100.0;
        dBigTolSq = dBigTolSq * dBigTolSq;
        double dDist1 = sCrvPnt0.DistanceBetweenSquared(sTestMid1);
        double dDist2 = sCrvPnt1.DistanceBetweenSquared(sTestMid2);
        if (dDist1 > dBigTolSq || dDist2 > dBigTolSq) {
            rbFoundGoodPoint = FALSE;
            m_rFilletSolver.ReloadSurfaceOffsets();
            return SM_SUCCESS;
        }
    }


    for (ULONG i=0; i<NUM_TEST_POINTS; i++) {
        double dParam = dParamsToTest[i];
        SmPoint3d sUV3d0, sUV3d1;
        SER(sUVCrv0.EvaluatePoint(dParam,sUV3d0));
        SER(sUVCrv1.EvaluatePoint(dParam,sUV3d1));
        SmPoint2d sUV0(sUV3d0.x,sUV3d0.y);
        SmPoint2d sUV1(sUV3d1.x,sUV3d1.y);
        sUV0 = m_vUVDomain[0].ClampPoint2d(sUV0);
        sUV1 = m_vUVDomain[1].ClampPoint2d(sUV1);
        SmPoint3d sPnt0, sPnt1, sCrvPnt;
        SER(m_cpSurface[0]->EvaluatePoint(sUV0,sPnt0));
        SER(m_cpSurface[1]->EvaluatePoint(sUV1,sPnt1));
        SmPoint3d sCrvPnt0, sCrvPnt1;
        SER(s3DCrv0.EvaluatePoint(dParam,sCrvPnt0));
        SER(s3DCrv1.EvaluatePoint(dParam,sCrvPnt1));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
        if (bDebugMe) {
            smgfx_SetLook(1,1, 1,0,0);
            m_cpSurface[0]->DrawUV(2,2); sm_GraphicsLoop();
            m_cpSurface[1]->DrawUV(2,2); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,0,1);
            s3DCrv0.Draw(); sm_GraphicsLoop();
            sMap0.Draw();   sm_GraphicsLoop();
            s3DCrv1.Draw(); sm_GraphicsLoop();
            sMap1.Draw();   sm_GraphicsLoop();
            sPnt0.Draw();   sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,1,0); sCrvPnt0.Draw(); sm_GraphicsLoop();
            smgfx_SetLook(1,1, 0,0,1); sPnt1.Draw();    sm_GraphicsLoop();
            smgfx_SetLook(1,1, 1,1,0); sCrvPnt1.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        double dDistSq = sPnt0.DistanceBetweenSquared(sCrvPnt0);
        double dDistSq2 = sPnt1.DistanceBetweenSquared(sCrvPnt1);
        dDistSq = smos_Max(dDistSq,dDistSq2);
        if (dDistSq > dTolSq && dStepSize > SM_EFF_ZERO*100.0) {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
            if (bDebugMe2) {
                sm_GraphicsLoop();
                smgfx_SetLook(1,1, 1,0,1); sMap0.Draw();   sm_GraphicsLoop();
                smgfx_SetLook(1,1, 1,0,0); s3DCrv0.Draw(); sm_GraphicsLoop();
                smgfx_SetLook(1,1, 0,0,1); sMap1.Draw();   sm_GraphicsLoop();
                smgfx_SetLook(1,1, 0,1,1); s3DCrv1.Draw(); sm_GraphicsLoop();
            }
#endif

            if (rNextTSP.m_ePointType == SM_IP_SINGULARITY) {
                rNextTSP.m_ePointType = SM_IP_CROSSING;
            }
            rbFoundGoodPoint = FALSE;
            m_rFilletSolver.ReloadSurfaceOffsets();
            return SM_SUCCESS;
        }
        if (dDistSq > dMaxDistSq) dMaxDistSq = dDistSq;
    }

    m_rFilletSolver.ReloadSurfaceOffsets();
    rbFoundGoodPoint = TRUE;
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Compute the next point in the intersection stepping algorithm.
    This algorithm will compute the point based on the step size and return
    a boolean indicating if the step size satisfies all of our stepping
    criteria.

NOTES:
***********************************************************************/
SmStatus SmFilletIntersector::ComputeNextPoint
  (SmTsectPnt & rTsectPnt,          // in : last computed xsect point positions and derivs
   double       dStepSize,          // in : size for next step (reduced when a boundary is hit)
   SmTsectPnt & rNextTSP,           // out: next xsect point
   SmBoolean  & rbFoundGoodPoint,   // out: TRUE = Found next point that passed angle and dist checks
                                    //      FALSE= Didn't
   double     & rdDeviationFound,   // out: max approx_3Dcurve/approx_surfTrimCurve dist
                                    //        of 5 test points between TsectPnt and NextTSP
   double     & rdAngleFoundRad,    // out: angleBetween(TsectPnt.3DTangent, NextTSP.3DTangent)
   SmBoolean  & rbClipped,          // out: TRUE=proposed stepSize was reduced to force
                                    //         NextTSP to be in both Surfaces
   SmBoolean  & rbBoundaryHit)      // out: TRUE=TsectPnt is already on a boundary and the
                                    //         NextTSP direction steps off one of the surfaces.
{
  // init output
  rbBoundaryHit = FALSE;
  rbClipped     = FALSE;

// NOTE: remove this min stepsize check.
// The caller's algorithm knows what stepsizes are important to it.
// [bd 04Jan06: 051103, and fillet regression tests 1:111 and 2:226]

//  // pick a minStepSize
//  double dMinStepSize =
//      (rTsectPnt.m_dCurveParameter < m_dThisApproxTol3d * 10.0)
//          ? SM_EFF_ZERO * 100.0      // give curve starts a small MinStepSize
//          : m_dThisApproxTol3d * 10.0;
//
//  // Don't allow too small steps.  Just stop here and try something else.
//  if (dStepSize < dMinStepSize)
//    {
//      rbFoundGoodPoint = FALSE;
//      return SM_SUCCESS;
//    }


// NOTE - need to move most of this stuff down to StepSolve
  // local
  SmBoolean bFoundAnswer;

  // Now that we have start point -
  //  when rTsectPnt.m_ePointType == SM_IP_CROSSING,
  //    do local surface intersection at startPoint + dStepSize
  //  else no next point - set rbFoundGoodPoint = FALSE and return
  //
  // Cases that have no next point include:
  //  SM_IP_TANGENT_CURVE : point on tangent curve
  //  SM_IP_SINGULARITY   : point on singular curve
  //  SM_IP_TANGENT_POINT : point between tangent surfaces
  //
  // Bottom line, SM_IP_CROSSING is the only case that can have a next point.

  if ( rTsectPnt.m_ePointType != SM_IP_CROSSING )
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // solve for the next SmTsectPt - save step output values
  SmBoolean bClipped, bBoundaryHit;
  if (   m_rFilletSolver.StepSolve( rTsectPnt,      dStepSize, 
                                    m_vUVDomain[0], m_vUVDomain[1],
                                    bFoundAnswer,   bClipped, 
                                    bBoundaryHit,   rNextTSP)
      != SM_SUCCESS)
    { bFoundAnswer = FALSE ; }

  // Set return values
  rbClipped     = bClipped;
  rbBoundaryHit = bBoundaryHit;

  // quit when NR iteration can't find a next step value
  if ( !bFoundAnswer ) { rbFoundGoodPoint = FALSE;
                         return SM_SUCCESS;
                       }

  // Get here after setting rNextTSP in StepSolve().
  // StepSolve() has set only the uv positions on the two surfaces;
  // now we fill in the uv tangents.

  // compute rNextTSP point values -
  SmPoint2d sUVs[2];
  sUVs[0] = rNextTSP.UVPos(0);
  sUVs[1] = rNextTSP.UVPos(1);
  SmStatus eStat = ComputePointValues( sUVs, rNextTSP, &rTsectPnt, &dStepSize );

  // stop criteria: fail to compute derivs when exiting a surface boundary - trace is done
  if ( eStat != SM_SUCCESS )
    {
      rbBoundaryHit    = TRUE; // causes termination of current tracing
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // stop criteria: next point is a tangent or coincident point - trace is done
  if (   rNextTSP.m_ePointType == SM_IP_TANGENT_POINT
      || rNextTSP.m_ePointType == SM_IP_COINCIDENCE)
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // stop criteria: next point stepsize is zero
  if (rNextTSP.CrvDeriv().LengthSquared() < SM_EFF_ZERO_SQ)
    {
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // get change in 3dCurve angle for this step
  SER(rTsectPnt.CrvDeriv().AngleBetween(rNextTSP.CrvDeriv(),rdAngleFoundRad));

  // when the change in tangent angle is too large - 
  //   reject span over a large enough stepSize,
  //   accept span to on small steps to stop endless recursion.
  //     note: although surfaces are not supposed to have cusps, when they do,
  //           accepting very small steps here allows the algorithm to walk 
  //           through surface cusps where the tangent flips by 180 degrees.
  if(rdAngleFoundRad > m_dThisAngTolRad)
    {
      // if step size is nearly 180 degrees - check to see if fillet radius is bigger than surface radius at rails
      if(rdAngleFoundRad > 3.0)
        {
          // check curvature against fillet radius at current point
          // rdAngleFoundRad = rdAngleFoundRad ;
        }

      // if step size is still large enough to split
      if(dStepSize > SM_EFF_ZERO*1000.0)
        {
    #ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe=FALSE;
          if (bDebugMe) 
            {
              smgfx_SetLook(1,1, 1,0,0);
              rTsectPnt.CrvDeriv().Draw(&rTsectPnt.CrvPos());
              rTsectPnt.CrvPos().Draw();
              sm_GraphicsLoop();
              smgfx_SetLook(1,1, 0,1,0);
              rNextTSP.CrvDeriv().Draw(&rNextTSP.CrvPos());
              rNextTSP.CrvPos().Draw();
              sm_GraphicsLoop();
            }
    #endif

          // failing point when sample pts are distinct - stop
          double dTol = SM_EFF_ZERO * 1000.0 * (1.0 + rNextTSP.CrvPos().GetMaxDimension());
          if (rTsectPnt.CrvPos().DistanceBetween(rNextTSP.CrvPos()) > dTol)
            {
              if (rNextTSP.m_ePointType == SM_IP_SINGULARITY)
                {
                  rNextTSP.m_ePointType = SM_IP_CROSSING;
                }
              rbFoundGoodPoint = FALSE;
              return SM_SUCCESS;
            }
        } // end large enough step size
    } // end too much angle change check

  // Also make sure that the point is going in the same direction
  // There are some cases where the next point could be on the
  // wrong side of the current point.  This is also a violation
  // of our angle tolerance.
  double dLineParam;
  SER(smgu_LineClosestPoint(rTsectPnt.UVPos(0),
                            rTsectPnt.UVDeriv(0),
                            rNextTSP.UVPos(0),
                            dLineParam));
  if (dLineParam < 0.0)
    {
      if (rNextTSP.m_ePointType == SM_IP_SINGULARITY)
        {
          rNextTSP.m_ePointType = SM_IP_CROSSING;
        }
      rbFoundGoodPoint = FALSE;
      return SM_SUCCESS;
    }

  // set next SmTsectPt parameter values = lastParameter + dStepSize
  rNextTSP.m_dCurveParameter = rTsectPnt.m_dCurveParameter + dStepSize;



  // Next: check whether the curve(s) between current and next point
  // are within tolerance.

  // Create two parameter space Hermite curves and test some points
  // on the curves as projected into 3d by their corresponding surfaces.

  // But first - set deriviatves with finite differences
  //   when m_bClippedByThroughPoint == TRUE
  //   and the magnitude of either uv derivative has changed substantially
  //     since the last point.

  SmBoolean bAdjustDerivatives = FALSE; // Keep track of this for later.
  if ( m_bClippedByThroughPoint )
    {
      ULONG lWhichSurf;
      for ( lWhichSurf=0; lWhichSurf<2; lWhichSurf++ )
        {
          double dStartUVLen = rTsectPnt.UVDeriv(lWhichSurf).Length();
          double dEndUVLen   = rNextTSP. UVDeriv(lWhichSurf).Length();

          if (   dStartUVLen > 3.0*dEndUVLen
              || dEndUVLen   > 3.0*dStartUVLen )
            {
              // Compute derivatives by making a small step
              SmTsectPnt sSmallStep;
              double dStep = 1.0e-5;
              SmBoolean bFoundGoodPoint;
              SER( m_rFilletSolver.StepSolve( rTsectPnt, dStep, m_vUVDomain[0],
                   m_vUVDomain[1], bFoundGoodPoint, bClipped, bBoundaryHit, sSmallStep ));

              if ( bFoundGoodPoint )
                {
                  // Got uv's, fill in the rest.
                  SmPoint2d sUVArray[2];
                  sUVArray[0] = rNextTSP.UVPos(0);
                  sUVArray[1] = rNextTSP.UVPos(1);
                  if ( ComputePointValues( sUVArray, sSmallStep, NULL, NULL ) != SM_SUCCESS )
                    {
                      break; // If either doesn't work, forget this idea.
                    }

                  // Set the deriv of the 3d curve using finite difference.
                  SmVector3d sDiff     = sSmallStep.CrvPos() - rTsectPnt.CrvPos();
                  rTsectPnt.CrvDeriv() = sDiff / dStep;
                  SER( rTsectPnt.CrvDeriv().Unitize() );

                  // Set the derivs of the uv curves on both surfaces.
                  ULONG jSrf;
                  for ( jSrf=0; jSrf<=1; jSrf++ )
                    {
                      SmVector2d sNewDeriv;
                      SER( smsurf_DropVectors(
                           sSmallStep.SrfDu(jSrf),
                           sSmallStep.SrfDv(jSrf),
                           1, &rTsectPnt.CrvDeriv(), &sNewDeriv ));

                      // Set it only if shorter.  (?)
                      double dOrigLen = rTsectPnt.UVDeriv(jSrf).Length();
                      if ( sNewDeriv.Length() < dOrigLen )
                        {
                          rTsectPnt.UVDeriv(jSrf) = sNewDeriv;
                        }
                    }
                  bAdjustDerivatives = TRUE; // Remember for later.

                } // end bFoundGoodPoint (StepSolve worked).

              // If we did this for the first surf,
              // no need to do it again for the second.
              break;

            } // end derivative magnitudes changed substantially
        } // end iter on both surfaces

      // Determine if we need to scale end derivs to avoid bad Hermit
      //SmPoint3d sStartPnt   = rTsectPnt.CrvPos();
      //SmPoint3d sEndPnt     = rNextTSP.CrvPos();
      //double dCurveLeng     = sStartPnt.DistanceBetween(sEndPnt);
      //SmPoint3d sStart3DTan = dStepSize*rTsectPnt.CrvDeriv();
      //SmPoint3d sEnd3DTan   = dStepSize*rNextTSP.CrvDeriv();
      //double dStartVecLen   = sStart3DTan.Length();
      //double dEndVecLen     = sEnd3DTan.Length();
      //if (dStartVecLen/3.0 > dCurveLeng*0.67)
      //  {
      //    double dScale = dCurveLeng/dStartVecLen;
      //    rTsectPnt.UVDeriv(0) = dScale*rTsectPnt.UVDeriv(0);
      //    rTsectPnt.UVDeriv(1) = dScale*rTsectPnt.UVDeriv(1);
      //    rTsectPnt.CrvDeriv() = dScale*rTsectPnt.CrvDeriv();
      //  }
      //if (dEndVecLen/3.0 > dCurveLeng*0.67)
      //  {
      //    double dScale = dCurveLeng/dEndVecLen;
      //    rNextTSP.UVDeriv(0) = dScale*rNextTSP.UVDeriv(0);
      //    rNextTSP.UVDeriv(1) = dScale*rNextTSP.UVDeriv(1);
      //    rNextTSP.CrvDeriv() = dScale*rNextTSP.CrvDeriv();
      //  }
    } // end m_bClippedByThroughPoint == TRUE check


  // Ok, now we've got the derivatives all squared away,
  // proceed with testing the curves in between the points.

  // create Surf1 Hermite UVTrimCurve
  SmHermiteCurve sUVCrv0(rTsectPnt.UVPos(0), rTsectPnt.UVDeriv(0)*dStepSize,
                          rNextTSP.UVPos(0),  rNextTSP.UVDeriv(0)*dStepSize, 2);
  sUVCrv0.SetContext(NULL);

  // create Surf2 Hermite UVTrimCurve
  SmHermiteCurve sUVCrv1(rTsectPnt.UVPos(1), rTsectPnt.UVDeriv(1)*dStepSize,
                          rNextTSP.UVPos(1),  rNextTSP.UVDeriv(1)*dStepSize, 2);
  sUVCrv1.SetContext(NULL);

  // Set OffsetSurface offset distance to zero to get baseSurface evaluations.
  m_rFilletSolver.SetSurfacesToZeroOffset();

  // use UVTrimCurves on baseSurfaces to define 3d railCurves for this step
  SmCrvOnSurf sMap0(sUVCrv0,SM_CONST_CAST(SmSurface&,*m_cpSurface[0]),
                    &m_vUVDomain[0], 0, NULL);
//  sMap0.SetContext(NULL);

  SmCrvOnSurf sMap1(sUVCrv1,SM_CONST_CAST(SmSurface&,*m_cpSurface[1]),
                    &m_vUVDomain[1], 0, NULL);
//  sMap1.SetContext(NULL);

  SmPoint3d sPV0[2], sPV1[2];

  // evaluate Surf0 step railCurve endPoint positions and derivs
  SER( sMap0.Evaluate( sMap0.GetNaturalInterval().GetMin(), 1, TRUE, sPV0 ));
  SER( sMap0.Evaluate( sMap0.GetNaturalInterval().GetMax(), 1, TRUE, sPV1 ));

  // If we had to find the uv curve derivs by divided differences (above),
  // set 3d curve derivs from nearby internal points.
  if (bAdjustDerivatives)
    {
      SmPoint3d sPV0_A[2], sPV1_A[2];
      SmExtent1d sIvl = sMap0.GetNaturalInterval();
      SER(sMap0.Evaluate(sIvl.Evaluate(0.001),1,TRUE,sPV0_A));
      SER(sMap0.Evaluate(sIvl.Evaluate(0.999),1,TRUE,sPV1_A));
      sPV0[1] = sPV0_A[1];
      sPV1[1] = sPV1_A[1];
    }

  // build Hermite Surf0 3dCurve rail curve
  SmHermiteCurve s3DCrv0(sPV0[0],sPV0[1],sPV1[0],sPV1[1]);
  s3DCrv0.SetContext(NULL);

  // evaluate Surf1 step railCurve endPoint positions and derivs
  SER(sMap1.Evaluate(sMap1.GetNaturalInterval().GetMin(),1,TRUE,sPV0));
  SER(sMap1.Evaluate(sMap1.GetNaturalInterval().GetMax(),1,TRUE,sPV1));

  // If we had to find the uv curve derivs by divided differences (above),
  // set 3d curve derivs from nearby internal points.
  if (bAdjustDerivatives)
    {
      SmPoint3d sPV0_A[2], sPV1_A[2];
      SmExtent1d sIvl = sMap1.GetNaturalInterval();
      SER(sMap1.Evaluate(sIvl.Evaluate(0.001),1,TRUE,sPV0_A));
      SER(sMap1.Evaluate(sIvl.Evaluate(0.999),1,TRUE,sPV1_A));
      sPV0[1] = sPV0_A[1];
      sPV1[1] = sPV1_A[1];
    }

  // build Hermite Surf1 3dCurve rail curve
  SmHermiteCurve s3DCrv1(sPV0[0],sPV0[1],sPV1[0],sPV1[1]);
  s3DCrv1.SetContext(NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
      if (bDebugMe) {
          if ( FALSE ) {
              smgfx_SetLook(1,1, 0,0,1); m_cpSurface[0]->DrawUV(2,2); sm_GraphicsLoop();
              smgfx_SetLook(1,1, 0,1,0); m_cpSurface[1]->DrawUV(2,2); sm_GraphicsLoop(); sm_GraphicsLoop();
          }
          smgfx_SetLook(1,1, 0,0,1 ); s3DCrv0.Draw();  sm_GraphicsLoop();
          smgfx_SetLook(1,1, 1,0,0 ); sMap0.Draw();    sm_GraphicsLoop();
          smgfx_SetLook(1,1, 0,1,1 ); s3DCrv1.Draw();  sm_GraphicsLoop();
          smgfx_SetLook(1,1, 1,0,1 ); sMap1.Draw();    sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif

#define NUM_TEST_POINTS 5
  double dParamsToTest[NUM_TEST_POINTS];
  dParamsToTest[0] = 0.5;
  dParamsToTest[1] = 0.17;
  dParamsToTest[2] = 0.83;
  dParamsToTest[3] = 0.33;
  dParamsToTest[4] = 0.67;

  // (NOTE about setting the tolerance according to the angle:
  //  See also the discussion in SmSurfaceIntersector::TestSpanAccuracy().
  //  These are not identical, but if you change one, check the other.)

  // Utilize the angle between the normals to adjust the tolerance.
  // This is required to get the approximation to within tolerance of the
  // true implicit curve, and not just to where the points on the surfaces
  // are within tolerance of each other.

  // (NOTE: originally this adjusted tol was adjusted to be no less than 1/10
  //  the original tol, but that 1/10 was changed to 1.0, which means it was
  //  adjusted to be no less than the original tol.  But then it was adjusted
  //  to be no greater than the original tol (which makes sense), so it ends up
  //  always being identical to the original tol.  Someday we might get a case
  //  that indicates that we should reinstate this test, but in the meantime
  //  just comment it all out: won't change anything.  [bd 5/13/09] )

  double dAdjustedTolerance = m_dThisApproxTol3d;

//  double dMinTanAngle = smos_Min( rTsectPnt.m_dTangentPlaneAngleRad,
//                                   rNextTSP.m_dTangentPlaneAngleRad );
//  double dAdjustedTolerance = 2.0 * m_dThisApproxTol3d * smos_Sine(dMinTanAngle/2.0);
//  // Don't let the adjusted tolerance get to be less then 1/10th of the actual
//  // tolerance.  Otherwise we could get nearly infinite numbers of points near
//  // tangency and singularity points.
//  dAdjustedTolerance = smos_Max(dAdjustedTolerance,m_dThisApproxTol3d/1.0);
//  // Don't let points get farther away than the tolerance either
//  dAdjustedTolerance = smos_Min(m_dThisApproxTol3d,dAdjustedTolerance);

  if ( bAdjustDerivatives )
    {
      dAdjustedTolerance *= 10.0;
    }
  double dTolSq = dAdjustedTolerance*dAdjustedTolerance;

  // for every test point
  double dMaxDistSq = 0.0;
  ULONG i;
  for ( i=0; i<NUM_TEST_POINTS; i++ )
    {
      double dParam = dParamsToTest[i];

      // get UVPnts from UVTrimCurves clamped to surf domain
      SmPoint3d sUV3d0, sUV3d1;
      SER(sUVCrv0.EvaluatePoint(dParam,sUV3d0));
      SER(sUVCrv1.EvaluatePoint(dParam,sUV3d1));
      SmPoint2d sUV0(sUV3d0.x,sUV3d0.y);
      SmPoint2d sUV1(sUV3d1.x,sUV3d1.y);
      sUV0 = m_vUVDomain[0].ClampPoint2d(sUV0);
      sUV1 = m_vUVDomain[1].ClampPoint2d(sUV1);

      // evaluate the associated surface points
      SmPoint3d sPnt0, sPnt1, sCrvPnt;
      SER(m_cpSurface[0]->EvaluatePoint(sUV0,sPnt0));
      SER(m_cpSurface[1]->EvaluatePoint(sUV1,sPnt1));

      // evaluate the associated 3d RailCurve points
      SmPoint3d sCrvPnt0, sCrvPnt1;
      SER(s3DCrv0.EvaluatePoint(dParam,sCrvPnt0));
      SER(s3DCrv1.EvaluatePoint(dParam,sCrvPnt1));

#ifdef SM_DEBUG_CODE
      if (bDebugMe) {
          smgfx_SetLook(1,4, 0,0,1 ); sCrvPnt0.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,4, 1,0,0 ); sPnt0.Draw();    sm_GraphicsLoop();
          smgfx_SetLook(1,4, 0,1,1 ); sCrvPnt1.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,4, 1,0,1 ); sPnt1.Draw();    sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif
      // get distance between 3dCurve(param) points and Surf(UVTrimCurve(param)) points
      double dDistSq  = sPnt0.DistanceBetweenSquared(sCrvPnt0);
      double dDistSq2 = sPnt1.DistanceBetweenSquared(sCrvPnt1);
      dDistSq = smos_Max(dDistSq,dDistSq2);

      // when max dDistSq value is too large on a long enough step - reject this span
      if (   dDistSq   > dTolSq
          && dStepSize > SM_EFF_ZERO*100.0)
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2) {
              sm_GraphicsLoop();
              smgfx_SetLook(1,1, 1,0,1); sMap0.Draw();   sm_GraphicsLoop();
              smgfx_SetLook(1,1, 1,0,0); s3DCrv0.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,1, 0,0,1); sMap1.Draw();   sm_GraphicsLoop();
              smgfx_SetLook(1,1, 0,1,1); s3DCrv1.Draw(); sm_GraphicsLoop();
          }
#endif

          if (rNextTSP.m_ePointType == SM_IP_SINGULARITY)
            {
              rNextTSP.m_ePointType = SM_IP_CROSSING;
            }
          rbFoundGoodPoint = FALSE;
          m_rFilletSolver.ReloadSurfaceOffsets();
          return SM_SUCCESS;
        } // end reject this next point check

      // save the max deviation
      if (dDistSq > dMaxDistSq) dMaxDistSq = dDistSq;

      // (NOTE: I don't know why this next test would be here,
      //  but it's almost certainly incorrect.
      //  (dTmpStepSize = dStepSize * dParam ? should be '+', I'd imagine).
      //  Also, it indicates results that appear to be way off when the
      //  curves are actually quite good, so it must be doing something wrong.
      //  And, it does not do anything in practice.
      //  It's hit only in fillet reg test 306 (as is), and in
      //  tests 10, 301, 306 and 307 with * changed to +,
      //  and has no effect on their outcome.  Removing it. [bd 5/13/09] )

//      // when the fillet offset distance can change (FALSE for SmConstantRadiusFS derived objects)
//      if (m_rFilletSolver.SurfaceOffsetsCanChange())
//        {
//          SmTsectPnt sTmpTSP;
//          double dTmpStepSize = dStepSize * dParam;  <- should be +
//
//          // get next step for dTmpStepSize
//          SER(m_rFilletSolver.StepSolve(rTsectPnt, dTmpStepSize,
//                                        m_vUVDomain[0], m_vUVDomain[1],
//                                        bFoundAnswer, bClipped, bBoundaryHit,
//                                        sTmpTSP));
//          if (bFoundAnswer)
//            {
//              // get railCurve points from baseSurfaces
//              SmPoint2d sTmpUVs[2];
//              sTmpUVs[0] = sTmpTSP.UVPos(0);
//              sTmpUVs[1] = sTmpTSP.UVPos(1);
//              SmPoint3d sTmpPnt0, sTmpPnt1;
//              m_rFilletSolver.SetSurfacesToZeroOffset();
//              SER(m_cpSurface[0]->EvaluatePoint(sTmpUVs[0],sTmpPnt0));
//              SER(m_cpSurface[1]->EvaluatePoint(sTmpUVs[1],sTmpPnt1));
//
//              // project Surf0 point onto the 3dRailCurve0 to get deviation
//              SmBoolean bFoundAnswer;
//              SmSolution sSol;
//              SER(s3DCrv0.LocalPointSolve(s3DCrv0.GetNaturalInterval(),SM_SO_MINIMIZE,
//                  sTmpPnt0,NULL,NULL,NULL,dParam,bFoundAnswer,sSol));
//              double dTmpDistSq = 0.0;
//              if (bFoundAnswer)
//                {
//                  dTmpDistSq = sSol.m_vStart.m_dSolutionValue;
//                  dTmpDistSq = dTmpDistSq*dTmpDistSq;
//                }
//
//              // project Surf1 point onto the 3dRailCurve1 to get deviations
//              SER(s3DCrv1.LocalPointSolve(s3DCrv1.GetNaturalInterval(),SM_SO_MINIMIZE,
//                  sTmpPnt1,NULL,NULL,NULL,dParam,bFoundAnswer,sSol));
//              double dTmpDistSq2 = 0.0;
//              if (bFoundAnswer) {
//                  dTmpDistSq2 = sSol.m_vStart.m_dSolutionValue;
//                  dTmpDistSq2 = dTmpDistSq2*dTmpDistSq2;
//              }
//#ifdef SM_DEBUG_CODE
//SmBoolean bDebugMe3 = FALSE;
//              if (bDebugMe3) {
//                  smgfx_SetLook(1,1, 1,0,0);
//                  sTmpPnt0.Draw(); sm_GraphicsLoop();
//                  sTmpPnt1.Draw(); sm_GraphicsLoop();
//                  smgfx_SetLook(1,1, 0,0,1);
//                  sPnt0.Draw(); sm_GraphicsLoop();
//                  sPnt1.Draw(); sm_GraphicsLoop();
//              }
//#endif
//              // when deviation > tol reject this next point
//              if (smos_Max(dTmpDistSq,dTmpDistSq2) > dTolSq)
//                {
//                  m_rFilletSolver.ReloadSurfaceOffsets();
//                  rbFoundGoodPoint = FALSE;
//                  return SM_SUCCESS;
//                } // end reject next point check
//            } // end tmpNextStep was found check
//        } // end can change fillet offsetSurface offset distance check

  } // end iter every testpoint

  // arrive here after every test point passed tolerance check

  // set output and return FoundGoodPoint == TRUE
  rdDeviationFound = smos_Sqrt(dMaxDistSq);
  rbFoundGoodPoint = TRUE; // passed test
  m_rFilletSolver.ReloadSurfaceOffsets();

#ifdef SM_DEBUG_CODE
  // check for curvature of base surface, in the direction we're moving.
  SmBoolean bDebugMe4 = FALSE;
  SmBoolean bDebugMe5 = FALSE;
  SmBoolean bDebugMe6 = FALSE;
  static double dMaxKappa = 0;
  SmVector3d sNorm, sCurvatureVec, sDummyVec;
  SmPoint3d sBasePt, sOffPt;
  SmOffsetSurface *pOffSrf;
  double dCurvature;
  // Surf 0 first:
  if ( bDebugMe4 ) {
      pOffSrf = SM_CAST_PTR( SmOffsetSurface, m_cpSurface[0] );
      if ( pOffSrf != NULL ) {
          pOffSrf->GetBaseSurface()->EvaluatePoint( rNextTSP.UVPos(0), sBasePt );
          pOffSrf->EvaluatePoint( rNextTSP.UVPos(0), sOffPt );
          SmVector3d sOffVec( sOffPt - sBasePt );

          pOffSrf->GetBaseSurface()->EvaluateNormalSection( rNextTSP.UVPos(0),
              TRUE, TRUE, rNextTSP.CrvDeriv(),
              sDummyVec, dCurvature, sCurvatureVec );

          if ( dCurvature < 0 ) {
              sCurvatureVec = - sCurvatureVec;
              dCurvature    = - dCurvature;
          }

          if ( sOffVec.Dot( sCurvatureVec ) > 0 ) { // offsetting 'inside'
              double dOffDist = sOffPt.DistanceBetween( sBasePt );
              // dCurvature is 1/radius, so check:
              if ( dOffDist * dCurvature > 1.0 + SM_EFF_ZERO )
                {
                  sDummyVec.Set(0,0,0); // just for breakpoint
                }
              if ( dCurvature > dMaxKappa ) {
                  dMaxKappa = dCurvature;

              }
          }
          if ( bDebugMe6 && dCurvature > SM_EFF_ZERO ) {
            sCurvatureVec /= dCurvature;
            smgfx_SetLook(1,4, 0,1,0); sCurvatureVec.Draw(&sBasePt); sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
      }
  }
      // Now Surf 1:
  if ( bDebugMe5 ) {
      pOffSrf = SM_CAST_PTR( SmOffsetSurface, m_cpSurface[1] );
      if ( pOffSrf != NULL ) {
          pOffSrf->GetBaseSurface()->EvaluatePoint( rNextTSP.UVPos(1), sBasePt );
          pOffSrf->EvaluatePoint( rNextTSP.UVPos(1), sOffPt );
          SmVector3d sOffVec( sOffPt - sBasePt );

          pOffSrf->GetBaseSurface()->EvaluateNormalSection( rNextTSP.UVPos(1),
              TRUE, TRUE, rNextTSP.CrvDeriv(),
              sDummyVec, dCurvature, sCurvatureVec );

          if ( dCurvature < 0 ) {
              sCurvatureVec = - sCurvatureVec;
              dCurvature    = - dCurvature;
          }

          if ( sOffVec.Dot( sCurvatureVec ) > 0 ) { // offsetting 'inside'
              double dOffDist = sOffPt.DistanceBetween( sBasePt );
              // dCurvature is 1/radius, so check:
              if ( dOffDist * dCurvature > 1.0 + SM_EFF_ZERO )
                {
                  sDummyVec.Set(0,0,0); // just for breakpoint
                }
              if ( dCurvature > dMaxKappa ) {
                  dMaxKappa = dCurvature;
              }
          }

          if ( bDebugMe6 && dCurvature > SM_EFF_ZERO ) {
            sCurvatureVec /= dCurvature;
            smgfx_SetLook(2,4, 0,1,0); sCurvatureVec.Draw(&sBasePt); sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
      }
  }
#endif

  // all done
  return SM_SUCCESS;

} // end SmFilletIntersector::ComputeNextPoint

/*******************************************************************//**
    Static functions
***********************************************************************/

/*******************************************************************//**
PURPOSE: Flush out the curve defined in the control point and knots arrays
         and create a B-Spline and put it into the curves array.

NOTES:
***********************************************************************/
static SmStatus sm_FlushCrv
 (const SmContext           & crContext,    // in :
  ULONG                       lDimension,   // in :
  SmTArray<SmPoint3d>       & rCntrlPoly,   // in :
  SmTArray<double>          & rKnots,       // in :
  SmTArray<SmBSplineCurve*> & crCurves)     // out:
{
  // check input
  if (rKnots.GetSize() == 0) 
    { return SM_SUCCESS; }

  // locals for BSplineCurve
  SmBSplineCurve * pNewBSP = NULL ;
  ULONG            lDegree = 3;
  SmTArray<ULONG> sKnotMult(rKnots.GetSize(), NULL, rKnots.GetSize());
  sKnotMult[0]                     = 4;
  sKnotMult[sKnotMult.GetSize()-1] = 4;
  for(ULONG i=1; i+1<sKnotMult.GetSize(); i++)  // note: can't say sKnotMult.GetSize()-1
    { sKnotMult[i] = 3 ; }

  // build the BSplineCurve
  SER(SmBSplineCurve::CreateCanonical(crContext,
                                      lDimension,
                                      lDegree,
                                      rCntrlPoly, 
                                      SM_CF_UNSPECIFIED, 
                                      sKnotMult, 
                                      rKnots,
                                      SM_KT_UNSPECIFIED, 
                                      NULL, 
                                      NULL, 
                                      pNewBSP));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      if (FALSE) 
        { pNewBSP->Dump() ; }

      smgfx_SetLook(1,2, 0,0,1) ; pNewBSP->DrawPolygon(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // set output
  crCurves.Add(pNewBSP) ;

  // all done
  return SM_SUCCESS ;

} // end sm_FlushCrv

/*******************************************************************//**
PURPOSE: Find a guess mid-point between two existing (and accurate)
         SmTsectPnt's derived from offset-surface's intersection
         Basically, prorate two SmTsectPnt's into a third..

NOTES:
***********************************************************************/
static void sm_FindGuessMidTsectPnt
 (SmTsectPnt * pCurrPnt, // in
  SmTsectPnt * pNextPnt, // in
  double       dF0,      // in: fraction from CurrPt to NextPt
  SmTsectPnt * pMidPnt)  // out
{
  SM_ASSERT(pCurrPnt && pNextPnt && pMidPnt);
  double dF1 = 1 - dF0;

  // position:
  pMidPnt->CrvPos()   = dF1 * pCurrPnt->CrvPos() + dF0 * pNextPnt->CrvPos();

  // 1st deriv:
  pMidPnt->CrvDeriv() =  dF1 * pCurrPnt->CrvDeriv() + dF0 * pNextPnt->CrvDeriv();
  double dLen         =  dF1 * pCurrPnt->CrvDeriv().Length()
                       + dF0 * pNextPnt->CrvDeriv().Length();
  pMidPnt->CrvDeriv().Unitize();
  pMidPnt->CrvDeriv() = pMidPnt->CrvDeriv() * dLen;

  // Surface uv points:
  for (ULONG i=0; i<2; i++)
    {
      pMidPnt->UVPos(i) =   dF1 * pCurrPnt->UVPos(i)
                          + dF0 * pNextPnt->UVPos(i);

      pMidPnt->UVDeriv(i) =   dF1 * pCurrPnt->UVDeriv(i)
                            + dF0 * pNextPnt->UVDeriv(i);
    }

  // curve param:
  pMidPnt->m_dCurveParameter =   dF1 * pCurrPnt->m_dCurveParameter
                               + dF0 * pNextPnt->m_dCurveParameter;

  // deviation: be pessimistic, better safe than sorry.
  // pMidPnt->m_dDeviation = (pCurrPnt->m_dDeviation +
  //     pNextPnt->m_dDeviation) / 2.0;
  pMidPnt->m_dDeviation = smos_Max(pCurrPnt->m_dDeviation, pNextPnt->m_dDeviation );

} // end sm_FindGuessMidTsectPnt

/*******************************************************************//**
    END - Static functions
***********************************************************************/


/*******************************************************************//**
PURPOSE: Flush the curve which currently exists in m_vCurvePoints
    list to define the fillet centerLine, rail UVCurves, rail 3dCurves,
    and a fillet surface.

NOTES:
  sets 1. pFilletGeom->SetFilletSurface   = New Fillet Surface
       2. pFilletGeom->m_pCenterLineCurve = Offset Surface/Surface Intersection Curve
       3. pFilletGeom->GetRail(0)->SetCurve(pCurve1)                                = new Rail 3dCurve 1
       4. pFilletGeom->GetRail(1)->SetCurve(pCurve2)                                = new Rail 3dCurve 2
       5. pFilletGeom->GetRail(0)->GetPrimaryEdgeuse()->GetMate()->SetUVCurve(pUV1) = new Rail UVCurve 1
       6. pFilletGeom->GetRail(1)->GetPrimaryEdgeuse()->GetMate()->SetUVCurve(pUV2) = new Rail UVCurve 2
  where target pFilletGeom =   m_pCurrFilletGeom
                             ? m_pCurrFilletGeom
                             : m_rFilletSolver.GetLastFilletGeom();
METHOD ---
  1. Get all offset-surface xsect curve sample points from m_vCurvePoints
  2. remember if offset-surface xsect curve runs to end of either fillet-surface domain
  3. remove short spans at the end of the offset-surface xsect curve
  4. Build knot array from curve sample points
  5. Add knots to array so that every span is similarly sized to its neighbors
     5a. For each new knot - compute new centerline and rail curve points
  6. build offset-surface xsect bspline curve, rail UVCurves and rail 3DCurves all
       control-point arrays with the same parameterization
  7. Convert control-point and knot arrays into SmBsplineCurve objects
  8. Select target pFilletGeom = m_pCurrFilletGeom or m_rFilletSolver.GetLastFilletGeom()
  9. Correct self-intersecting rail curves
  10. Generate the fillet surface
  11. Correct self-intersecting fillet-surface
  12. Save FilletSurface data

SIDE EFFECTS ---
  1. Modifies: m_rFilletSolver.GetLastFilletGeom()
     1a. pFilletGeom->m_pFilletSurface    = newly created fillet surface
     1b. pFilletGeom->m_pCenterLineCurve  = newly created CenterLine
     1c. pFilletGeom->Rail[i]->Edgeuse->Mate->UVCurve = newly created rail UVCurves
     1d. pFilletGeom->Rail[i]->Curve      = newly created rail 3D Curves

***********************************************************************/
SmStatus SmFilletIntersector::FlushCurve
 (SmTArray<SmCurve*>         & r3DCurves,          // NotUsed: in : 
  SmTArray<SmCurve*>         & rSurface1UVCurves,  // NotUsed: in : 
  SmTArray<SmCurve*>         & rSurface2UVCurves,  // NotUsed: in : 
  SmTArray<SmTsectCurveType> & rCurveTypes,        // out:
  SmTArray<double>           & rDeviations)        // out:
{
  SM_REF3(r3DCurves, rSurface1UVCurves, rSurface2UVCurves) ;
  // locals
  SmTsectPnt               *aData[200];
  SmTsectPnt               *bData[50];
  SmTArray<SmTsectPnt*>     sPnts(200,aData);
  SmTArray<SmTsectPnt*>     sDeletePnts(50,bData);
  SmObjsDelete<SmTsectPnt*> sObjs(&sDeletePnts);

  // 1. offset surface sample points
  m_vCurvePoints.GetAllNodes(sPnts);

  // 2. Remember if the end of current fillet touched a boundary on
  //    either surface UVDomain
  SmTsectPnt * pLastTsectPnt = m_vCurvePoints.GetLastNode(); NER(pLastTsectPnt);
  m_bCurrFilletTouchBoundary = FALSE;

  // for both surfaces
  for (ULONG lSrf=0; lSrf<=1; lSrf++)
    {
      // get tolerance proportional to max UV coordinate value
      double dSize = smos_4Max(smos_Fabs(m_vUVDomain[lSrf].GetMin().x),
                               smos_Fabs(m_vUVDomain[lSrf].GetMin().y),
                               smos_Fabs(m_vUVDomain[lSrf].GetMax().x),
                               smos_Fabs(m_vUVDomain[lSrf].GetMax().y)) ;
      double dTol = SM_EFF_ZERO * (1.0 + dSize) * 100.0;

      // remember if lastTsectPnt is on this surface's (extended) uv domain boundary
      //  GWC: do we want to remember if the curve ended on a surface's original natural domain boundary 
      if ( m_vUVDomain[lSrf].IsPoint2dOnBoundary( pLastTsectPnt->UVPos(lSrf), dTol ))
        {
          m_bCurrFilletTouchBoundary = TRUE;
          break;
        }
    } // end iter both surfaces

  // Clear out curve point list that's been copied into sPnts
  while (m_vCurvePoints.GetLastNode()) 
    { m_vCurvePoints.RemoveLast() ; }

  // no work - not enough xsect points to flush a curve
  if (sPnts.GetSize() < 2) return SM_SUCCESS;

  // new curve locals
  SmTArray<double> sKnots;
  SmPoint3d sData3D[100];
  SmPoint3d sData3DSur1[100];
  SmPoint3d sData3DSur2[100];
  SmPoint3d sDataUV1[100];
  SmPoint3d sDataUV2[100];
  SmTArray<SmPoint3d> s3DCtrlPts(100,sData3D);
  SmTArray<SmPoint3d> s3DCtrlPtsSur1(100,sData3DSur1);
  SmTArray<SmPoint3d> s3DCtrlPtsSur2(100,sData3DSur2);
  SmTArray<SmPoint3d> sUVCtrlPts0(100,sDataUV1);
  SmTArray<SmPoint3d> sUVCtrlPts1(100,sDataUV2);

  SmTArray<SmPoint3d> * apUVCtrlPts[2];
  SmTArray<SmPoint3d> * ap3DCtrlPts[2];
  apUVCtrlPts[0] = &sUVCtrlPts0;          // rail 1 UVcurve
  apUVCtrlPts[1] = &sUVCtrlPts1;          // rail 2 UVCurve
  ap3DCtrlPts[0] = &s3DCtrlPtsSur1;       // rail 1 3DCurve
  ap3DCtrlPts[1] = &s3DCtrlPtsSur2;       // rail 2 3DCurve

  // locals for fillet target surfaces
  SmSurface * pSur[2] ;
  pSur[0] = m_rFilletSolver.GetSurface(0) ; // 1st fillet surface
  pSur[1] = m_rFilletSolver.GetSurface(1) ; // 2nd fillet surface

  SmTsectCurveType eCurveType    = SM_TC_CROSSING;
  double           dMaxDeviation = 0.0;
  double           dMaxAngle     = 0.0;

  // 3. If the final span is tiny, remove it by consolidating it with
  // the previous span.
  // Note, it's very possible that that point corresponds to a knot
  // that was found earlier, and could end up being the actual end
  // of the curve.  Tried leaving out this check, and in practice it works
  // either way, but leaving the test in results in a somewhat reduced
  // control point count.
  // Also tried it using a ratio of 1000 instead of 10, because I've
  // never found sudden changes in span size to be a problem: same result,
  // just a few more control points.  So just leave it at 10.
  // [bd 6/6/06]

  if(sPnts.GetSize() > 2)
    {
      SmTsectPnt * pLTSP   = sPnts[sPnts.GetSize()-1];
      SmTsectPnt * pNLTSP  = sPnts[sPnts.GetSize()-2];
      SmTsectPnt * pNNLTSP = sPnts[sPnts.GetSize()-3];
      double       dLT     = pLTSP->m_dCurveParameter;
      double       dNLT    = pNLTSP->m_dCurveParameter;
      double       dNNLT   = pNNLTSP->m_dCurveParameter;
      double       dRatio  = (dNLT - dNNLT) / (dLT - dNLT);
      if ( dRatio > 10.0 )
        {
          // If the ratio is too big remove the next to last point
          sPnts[sPnts.GetSize()-2] = pLTSP;
          sPnts.RemoveLast();
        }
    } // end sPnts.GetSize() > 2 check

  // 4. Build array of knots
  for (ULONG k=0; k<sPnts.GetSize(); k++)
    {
      sKnots.Add( sPnts[k]->m_dCurveParameter );
    }

  // 5. add knots so that each span is similiarly sized
  SmBoolean bDone               = FALSE;
  double    dMaximumDeltaFactor = 2.8;
  ULONG     lOrigNumKnots       = sKnots.GetSize();

  // continue to iter until spans are uniformly sized
  while (!bDone)
    {
      bDone = TRUE ;

      // end condition: if knot count is more than tripled
      if(sKnots.GetSize() > 3.0 * lOrigNumKnots) 
        { break ; }

      // iter every span
      for(ULONG j=0; j+1<sKnots.GetSize(); j++) // note: can't say sKnots.GetSize()-1
        {
          // see if this span should be split
          ULONG bSplit = FALSE;

          double dSpanSize = sKnots[j+1] - sKnots[j];

          if(j > 0) // Look at size of previous span
            {
              double dLastSpanSize = sKnots[j] - sKnots[j-1];
              if(dSpanSize/dLastSpanSize > dMaximumDeltaFactor)
                { bSplit = TRUE ; }
              
            }

          if(j<sKnots.GetSize()-2) // Look at size of next span
            {
              double dNextSpanSize = sKnots[j+2] - sKnots[j+1];
              if(dSpanSize/dNextSpanSize > dMaximumDeltaFactor) 
                { bSplit = TRUE ; }
            }

          if(!bSplit)
            { 
              // don't recheck spans already checked
              // lCnt = j ;
              continue; 
            }

          // If we need to split, find the fraction of the way,
          // from start to end, where we should put the split point
          // to get a ratio of dMaximumDeltaFactor.

          double dFrac = 0.5;  // midpoint
          // Note: in practice, it works better just to split
          // at the midpoint. [Fillet reg 2:229]
          // if ( bSplitStart && bSplitEnd )
          //     dFrac = 0.5;  // both adjacent spans are tiny: midpoint
          // else if ( bSplitStart )
          //     dFrac = 1.0 / ( 1 + dMaximumDeltaFactor );
          // else if ( bSplitEnd )
          //     dFrac = dMaximumDeltaFactor / ( 1 + dMaximumDeltaFactor );
          // else
          //     continue;  // no splitting.

          // Split the span.
          // add a mid-span knot value to knot array
          bDone             = FALSE;
          double dSplitKnot = (1-dFrac) * sKnots[j] + dFrac * sKnots[j+1];
          sKnots.InsertAt(j+1,dSplitKnot);

          // 5a. Find 3D intersection point for new knot value

          // setup: declare new SmTsectPnt,
          //        get a mid-span initial guess
          //        define a 3D plane perpendicular to the xsect curve at the mid-point
          //        get mid-span UV values for both fillet surfaces
          SmTsectPnt * pMidPnt      = new SmTsectPnt;
          sm_FindGuessMidTsectPnt( sPnts[j], sPnts[j+1], dFrac, pMidPnt) ;

          SmPoint3d    sPlaneOrig   = pMidPnt->CrvPos() ;
          SmVector3d   sPlaneNormal = pMidPnt->CrvDeriv() ;
          SmPoint2d    sUV1         = pMidPnt->UVPos(0) ;
          SmPoint2d    sUV2         = pMidPnt->UVPos(1) ;
          SmBoolean    bFoundSolution ;
          SER(sPlaneNormal.Unitize()) ;

          // find filletPoint on given plane satisfying geometry
          //   requirements implemented in derived SmFilletSolver class
          SER(m_rFilletSolver.PointOnPlaneSolve(sPlaneOrig,
                                                sPlaneNormal,
                                                m_vUVDomain[0],
                                                m_vUVDomain[1],
                                                sUV1,
                                                sUV2,
                                                bFoundSolution,
                                                *pMidPnt)) ;
          if(!bFoundSolution) 
            { SER(SM_ERR) ; }

          // save the rail UVCurve points for the new mid-span point
          SmPoint2d sUVs[2];
          sUVs[0] = pMidPnt->UVPos(0);
          sUVs[1] = pMidPnt->UVPos(1);

          // get the rail 3DCurve sample values
          SER(m_rFilletSolver.ComputePointValues(sUVs,
                                                 m_vUVDomain[0],
                                                 m_vUVDomain[1],
                                                 m_dCurveTraceDirection,
                                                 *pMidPnt,
                                                 sPnts[j])) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              SmPoint3d sP1 = sPnts[j]->CrvPos();
              SmPoint3d sP2 = sPnts[j+1]->CrvPos();
              SmPoint3d sP3 = pMidPnt->CrvPos();

              smgfx_SetLook(1,1, 1,0,0) ; sPnts[j]->CrvDeriv().Draw(&sP1) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,1, 1,0,0) ; sPnts[j+1]->CrvDeriv().Draw(&sP2) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,1, 0,0,1) ; pMidPnt->CrvDeriv().Draw(&sP3) ; sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // add the completed mid-span point to the sPnts array
          sPnts.InsertAt(j+1,pMidPnt);
          sDeletePnts.Add(pMidPnt);
          j++;
        } // end iter every span
    } // end while spans are not uniformly sized

  // set the stored fillet-surface OffsetDistances to 0.0
  m_rFilletSolver.SetSurfacesToZeroOffset();

  // 6. for every offset-surface xsect curve sample point span
  // build center-line, rail UVCurves, and rail 3DCurves control-point arrays
  // all with the same parameterization
  for(ULONG i=1;i<sPnts.GetSize();i++)
    {
      // get span end points
      SmTsectPnt *pTSPLast = sPnts[i-1];
      SmTsectPnt *pTSPCurr = sPnts[i];

      // save max angle between offset surfaces along xsect curve sample points
      dMaxAngle = smos_Max(dMaxAngle,pTSPCurr->m_dTangentPlaneAngleRad);

      // If we have a tangent curve set that curve type
      if(   i==1
         && pTSPCurr->m_ePointType == SM_IP_TANGENT_CURVE) 
        {
          eCurveType = SM_TC_TANGENT;
        }

      // Collect maximum deviation
      if (pTSPLast->m_dDeviation > dMaxDeviation) dMaxDeviation = pTSPLast->m_dDeviation;
      if (pTSPCurr->m_dDeviation > dMaxDeviation) dMaxDeviation = pTSPCurr->m_dDeviation;

      // let dScale = span parameter length
      double dScale = pTSPCurr->m_dCurveParameter - pTSPLast->m_dCurveParameter;

      // for the CenterLine
      // define sP1 = span start point
      //        sP2 = bezier control point
      //        sP3 = bezier control point
      //        sP4 = span end   point
      //        sD1 = span start tangent vector (length = span parameter length)
      //        sD2 = span end   tangent vector (length = span parameter length)
      SmPoint3d  sP1(pTSPLast->CrvPos());
      SmPoint3d  sP4(pTSPCurr->CrvPos());
      SmVector3d sD1(pTSPLast->CrvDeriv()*dScale);
      SmVector3d sD2(pTSPCurr->CrvDeriv()*dScale);

      // Note that the quick and easy way to convert from a Hermite
      // into a Bezier is to divide the vectors in the Hermite by the
      // degree (3) and add/subtract from the first/last vertices.
      SmPoint3d sP2 = sP1 + sD1 / 3.0;
      SmPoint3d sP3 = sP4 - sD2 / 3.0 ;

      // for the first span - add the start control point to the s3DCtrlPts array
      if(s3DCtrlPts.GetSize() == 0) 
        { s3DCtrlPts.Add(sP1) ; }

      // now add the span's last 3 control points to the s3DCtrlPts array
      s3DCtrlPts.Add(sP2) ;
      s3DCtrlPts.Add(sP3) ;
      s3DCtrlPts.Add(sP4) ;

      // for both surface rail curves - build the rail curves
      for(ULONG lSrf=0; lSrf<=1; lSrf++)
        {
          // define sUV1 = span start point
          //        sUV2 = bezier control point
          //        sUV3 = bezier control point
          //        sUV4 = span end   point
          //        sDer1 = span start tangent vector (length = span parameter length)
          //        sDer2 = span end   tangent vector (length = span parameter length)
          SmPoint2d sUV1(pTSPLast->UVPos(lSrf)) ;
          SmPoint2d sUV4(pTSPCurr->UVPos(lSrf)) ;
          SmVector2d sDer1(pTSPLast->UVDeriv(lSrf)*dScale) ;
          SmVector2d sDer2(pTSPCurr->UVDeriv(lSrf)*dScale) ;

          // Convert from Hermite to Bezier.
          SmPoint2d sUV2 = sUV1 + sDer1 / 3.0 ;
          SmPoint2d sUV3 = sUV4 - sDer2 / 3.0 ;

          // for the first span - add the start control point to the s3DCtrlPts array
          if(apUVCtrlPts[lSrf]->GetSize() == 0) 
            { apUVCtrlPts[lSrf]->Add(SmPoint3d(sUV1)) ; }

          // now add the span's last 3 control points to the s3DCtrlPts array
          apUVCtrlPts[lSrf]->Add(SmPoint3d(sUV2)) ;
          apUVCtrlPts[lSrf]->Add(SmPoint3d(sUV3)) ;
          apUVCtrlPts[lSrf]->Add(SmPoint3d(sUV4)) ;

          // Now lift the point and derivatives of UV curves into 3D on the zero offset surface.
          SmPoint3d  sPnt ;
          SmVector3d sDU, sDV ;
          SER(pSur[lSrf]->Evaluate1stDerivatives(pTSPLast->UVPos(lSrf),TRUE,TRUE,sPnt,sDU,sDV)) ;
          SmPoint3d sP13D(sPnt) ;
          SmPoint3d sD13D(sDU*sDer1.x + sDV*sDer1.y) ;

          SER(pSur[lSrf]->Evaluate1stDerivatives(pTSPCurr->UVPos(lSrf),TRUE,TRUE,sPnt,sDU,sDV)) ;
          SmPoint3d sP43D(sPnt) ;
          SmPoint3d sD23D(sDU*sDer2.x + sDV*sDer2.y) ;

          // Convert from Hermite to Bezier.
          SmPoint3d sP23D = sP13D + sD13D / 3.0 ;
          SmPoint3d sP33D = sP43D - sD23D / 3.0 ;
          if(ap3DCtrlPts[lSrf]->GetSize() == 0) 
            { ap3DCtrlPts[lSrf]->Add(SmPoint3d(sP13D)) ; }

          ap3DCtrlPts[lSrf]->Add(SmPoint3d(sP23D));
          ap3DCtrlPts[lSrf]->Add(SmPoint3d(sP33D));
          ap3DCtrlPts[lSrf]->Add(SmPoint3d(sP43D));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1=FALSE;
          if (bDebugMe1) 
            {
              smgfx_SetLook(2,4, 1,0,0);
              sP13D.Draw();       sm_GraphicsLoop();
              sD13D.Draw(&sP13D); sm_GraphicsLoop();
              sP23D.Draw();       sm_GraphicsLoop();
              sP33D.Draw();       sm_GraphicsLoop();
            (-sD23D).Draw(&sP43D); sm_GraphicsLoop();
              sP43D.Draw();       sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end iter both surface rail curves
    } // end iter every point

  // locals
  SmBSplineCurve * sCData1[16];
  SmBSplineCurve * sCData2[16];
  SmBSplineCurve * sCData3[16];
  SmTArray<SmBSplineCurve*> s3DCurves(16,sCData1);
  SmTArray<SmBSplineCurve*> sSurface1UVCurves(16,sCData2);
  SmTArray<SmBSplineCurve*> sSurface2UVCurves(16,sCData3);

  // 7. convert control-point/knot Arrays into SmBsplineCurves
  SER(sm_FlushCrv(*m_cpContext, 3, s3DCtrlPts,     sKnots, s3DCurves));
  SER(sm_FlushCrv(*m_cpContext, 2, sUVCtrlPts0,    sKnots, sSurface1UVCurves));
  SER(sm_FlushCrv(*m_cpContext, 2, sUVCtrlPts1,    sKnots, sSurface2UVCurves));
  SER(sm_FlushCrv(*m_cpContext, 3, s3DCtrlPtsSur1, sKnots, s3DCurves));
  SER(sm_FlushCrv(*m_cpContext, 3, s3DCtrlPtsSur2, sKnots, s3DCurves));
  rDeviations.Add(dMaxDeviation);
  rCurveTypes.Add(eCurveType);

  // 8. Select target pFilletGeom to Modify
  SmFilletGeom * pFilletGeom =   m_pCurrFilletGeom
                               ? m_pCurrFilletGeom
                               : m_rFilletSolver.GetLastFilletGeom();

  // get the self-intersection handler
  SmSelfIntersectionHandler * pSelfIntHandler = m_rFilletSolver.GetSelfIntersectionHandler();

  // 9. when self-intersection checking/correcting is on
  // GWC:SELFINTERSECT_REPLACED_ONE_LINE
  if (pSelfIntHandler != NULL)
  // if (m_bSelfIntersect)
    {
      if (pSelfIntHandler->GetAlgorithmType() == SM_HA_SMOOTHING)
        {
          // Already have a surface we will now try to recreate it
          // by doing some work to combine separate intersection curves
          // into a single set. If the handler will perform a smoothing
          // operation, then the following will recreate rails and
          // surfaces with self int. been smoothed out.
            
          // GWC: Stop Fillets from switching SmCurve and SmSurface objects to SmBSplineCurve and SmBSplineSurface objs
          if(SM_SUCCESS != pSelfIntHandler->SmoothOutIntersection(pFilletGeom,
                                                                  s3DCurves,
                                                                  sSurface1UVCurves,
                                                                  sSurface2UVCurves)) 
            {
              m_rFilletSolver.SetStatus(SM_FS_SELF_INT_HANDLING_FAILURE);
              SER(SM_ERR);
            }
        } // end algorithmType == SM_HA_SMOOTHING check
    } // end m_bSelfIntersect == TRUE check

  // get (the potentially modified) rail curves, their UVTrimCurves, and the Centerline
  SmBSplineCurve *pUV1 = sSurface1UVCurves.GetLast(); NER(pUV1);
  sSurface1UVCurves.RemoveLast();
  SmBSplineCurve *pUV2 = sSurface2UVCurves.GetLast(); NER(pUV2);
  sSurface2UVCurves.RemoveLast();

  SmBSplineCurve * pCurve2 = s3DCurves.GetLast(); NER(pCurve2);
  s3DCurves.RemoveLast();
  SmBSplineCurve * pCurve1 = s3DCurves.GetLast(); NER(pCurve1);
  s3DCurves.RemoveLast();
  SmBSplineCurve *pCenterLine = s3DCurves.GetLast(); NER(pCenterLine);
  s3DCurves.RemoveLast();
  
  SmBSplineSurface *pFilletSurface = NULL;
 

  // 10. Generate the fillet surface from the rail curves and centerline
  if (m_rFilletSolver.m_pFSG)
    {
      // generate the Surface
      if(SM_SUCCESS != m_rFilletSolver.m_pFSG->CreateSurface(m_rFilletSolver,
                                                             pCenterLine, 
                                                             pCurve1, 
                                                             pCurve2, 
                                                             sPnts,
                                                             pSur[0], 
                                                             pUV1, 
                                                             pSur[1], 
                                                             pUV2,
                                                             pFilletSurface))
        {
          // if (pFilletSurface) {delete pFilletSurface; pFilletSurface = NULL;} // JLMCC hunting memory leaks
          m_rFilletSolver.SetStatus( SM_FS_SURF_SKINNING_FAILURE );
          SER( SM_ERR );
        }

#ifdef SM_DEBUG_CODE
      if(m_rFilletSolver.DebugLevel() > 5) 
        {
          smgfx_Erase();
          smgfx_SetLook(1,1, 0,0,0) ; m_rFilletSolver.GetFilletExecutive()->GetTargetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,3, 1,0,0) ; pCurve1->DrawWDeriv(pCurve1->GetNaturalInterval()) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,3, 1,0,0) ; pCurve2->DrawWDeriv(pCurve2->GetNaturalInterval()) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,3, 0,0,1) ; pCenterLine->DrawWDeriv(pCenterLine->GetNaturalInterval()) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,2, 1,1,0) ; pFilletSurface->DrawUV(0,0) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    } // m_rFilletSolver.m_pFSG

  // 11. Correct self-intersecting fillet-surface 
  //     NOT YET IMPLEMENTED - needs work
static constexpr SmBoolean bTempTrySelfIntHandler = FALSE ;

  if (pFilletSurface)
    {
      if(   pSelfIntHandler != NULL
         && bTempTrySelfIntHandler)   // GWC:TEMP to skip this incomplete section
      // if (m_bSelfIntersect)        // GWC:When done - this is the actual check needed.  It saves time, only running when 
      //                              //      known to have a problem.
        {
          // TRUE when intersect handler is type SmMakeSurfaceBlendSIH - supports SmSelfIntersectionHandler::CreateBlends
          // FALSE when intersect handle is type SmMakeCurveBlendSIH - supports SmSelfIntersectHandler::SmoothOutIntersection
          if (pSelfIntHandler->GetAlgorithmType() == SM_HA_BLENDING)
            {
              // Create blending-surface patch between original
              // and newly created surface
              SmFilletGeom * pBlendFilletGeom  = NULL;
              SmFilletGeom * pNewFilletGeom    = NULL;
              SmBoolean      bSelfIntersecting = FALSE ;
              SmPoint2d      sStartPoint ;
              SmPoint2d      sEndPoint ;
              double         dFilletRadius     = 1.0 ;

              // check for a self-intersecting surface 
              //   - warning not implemented yet (bSelfIntersecting always FALSE)
              pSelfIntHandler->IsSelfIntersecting(*m_cpContext,
                                                  pFilletSurface,
                                                  pFilletGeom->GetFilletSolver()->GetThisApproxTol3d(),
                                                  pCenterLine,
                                                  pCurve1,
                                                  pCurve2,
                                                  dFilletRadius,
                                                  bSelfIntersecting,  // in the future - use this when needed
                                                  sStartPoint,
                                                  sEndPoint) ;

              // split topology of pFilletGeom into 3 connected neighbors, pFilletGeom => pBlendFilletGeom => pNewFilletGeom  
              //    (geometry is not yet set)
              if(   SM_SUCCESS != pFilletGeom->TopologySplit(pBlendFilletGeom,SM_FG_BLENDS)      // 1st topology split
                 || SM_SUCCESS != pBlendFilletGeom->TopologySplit(pNewFilletGeom,SM_FG_DEFAULT)) // 2nd topology split
                {
                  // Exit Error - one of the FilletGeom TopologySplits failed 
                  m_rFilletSolver.SetStatus(SM_FS_SELF_INT_HANDLING_FAILURE) ;
                  SER(SM_ERR) ;
                }

              // set geometry of 3 connected neighbor fillets just made by
              // trimming out and blending over a self intersecting portion of a fillet curve
              //   input: prev section in pOrigFilletGeom, 
              //          next section in input arguments pNewFilletSurface, pNewFillet, ...
              //   output: pOrigFilletGeom set with geometry for prev section trimmed back to OK section
              //           pBlendFilletGeom set with geometry for blend section
              //           pNewFilletGeom set with geometry for prev section trimmed back to OK section 
              // same algorithm as SmoothOutIntersection - but sets outputs into geometry of 3 different fillet geoms and updates fillet surface
              if(SM_SUCCESS != pSelfIntHandler->CreateBlends(pFilletGeom,      // i/o: orig FilletGeom                     - beg of output FilletSequence
                                                             pBlendFilletGeom, // i/o: FilletGeom after 1 TopologySplit()  - mid of output FilletSequence
                                                             pNewFilletGeom,   // i/o: FilletGeom after 2 topologySplits() - end of output FilletSequence
                                                             pFilletSurface,   // in : Fillet surface with potential self-intersections
                                                             pCenterLine,      // in : Fillet surface center line
                                                             pCurve1,          // in : 1st 3d rail curve
                                                             pCurve2,          // in : 2nd 3d rail curve
                                                             pUV1,             // in : 1st UV rail curve
                                                             pUV2))            // in : 2nd UV rail curve
                {
                  m_rFilletSolver.SetStatus(SM_FS_SELF_INT_HANDLING_FAILURE);
                  SER(SM_ERR);
                }

              // add the newly created blended filletGeoms to the FilletSolver geom list
              m_rFilletSolver.m_vFilletGeoms.Add(pBlendFilletGeom) ;
              m_rFilletSolver.m_vFilletGeoms.Add(pNewFilletGeom) ;

              return SM_SUCCESS ;

            } // end SelfIntersectingHandler algorithm == SM_HA_BLENDING check
          // should have a section for SM_HA_SMOOTHING
          // else // SM_HA_SMOOTHING branch
          //   {
          //     // need branch here
          //   }
        } // end check/correct SelfIntersection branch

// GWC:SELFINTERSECT-CHANGE removed this branch which seems to be incomplete
//                          and is only running now because of other GWC:SELFINTERSECT
//                          changes.  The original behavior was to be removed
   //   else
   //     { // no check/correct SelfIntersection branch
   //       SmFilletGeom * pNewFilletGeom = NULL;
   //
   //       //
   //       SER(pFilletGeom->TopologySplit(pNewFilletGeom,SM_FG_GAP_FILLER));
   //       SER(pNewFilletGeom->MakeGapFiller());
   //
   //       // add the splitGeom to the FilletSolver geom list
   //       m_rFilletSolver.m_vFilletGeoms.Add(pNewFilletGeom);
   //       pFilletGeom = pNewFilletGeom;
   //
   //     } // end no check/correct SlefIntersection branch
    } // end fillet surface created check

  // 12. Save Fillet Surface and rail curve data

  // Save filletSurface in target filletGeom Structure
  if(pFilletGeom->m_pFilletSurface != NULL)
    { delete pFilletGeom->m_pFilletSurface; pFilletGeom->m_pFilletSurface = NULL ; }
  pFilletGeom->SetFilletSurface(pFilletSurface) ;

  // Save Centerline curve in target filletGeom Structure
  SmCurve * pCrv = pFilletGeom->GetCenterLineCurve() ;
  if (pCrv && pCrv != pCenterLine) { delete pCrv; pCrv = NULL ; }
  pFilletGeom->m_pCenterLineCurve = pCenterLine ;

  // Rail1 locals
  SmFilletEdge    * pRail1   = pFilletGeom->GetRail(0) ;
  SmFilletEdgeuse * pPrimEU1 = (SmFilletEdgeuse*)pRail1->GetPrimaryEdgeuse() ;
  SmFilletEdgeuse * pMateEU1 = (SmFilletEdgeuse*)pPrimEU1->GetMate() ;

  // Save pCurve1 in rail(1)->Curve
  pRail1->SetCurve(pCurve1, TRUE) ;  // TRUE = also delete preExisting m_pCurve   
                                     // side effect: delete current pRail1->UVTrimCurves
  pCurve1->SetOwner(pRail1) ;
  SmExtent1d sCrvIvl = pCurve1->GetNaturalInterval() ;
  pRail1->SetInterval(sCrvIvl) ;
    
  // Save pUV1 in rail(1)->Edgeuse->Mate->UVCurve
  // pBS = pMateEU1->GetUVTrimCurvePointer();  // no longer needed - gets deleted in SetUVCurve
  pMateEU1->SetUVCurve(pUV1, TRUE) ; // TRUE = also delete preExisting UVTrimCurve - expect none here
  pUV1->SetOwner(pMateEU1) ;
  // if (pBS && pBS != pUV1) { delete pBS; pBS = NULL ; }  // no longer needed - already deleted
    
  // Rail2 locals
  SmFilletEdge    * pRail2   = pFilletGeom->GetRail(1) ;
  SmFilletEdgeuse * pPrimEU2 = (SmFilletEdgeuse*)pRail2->GetPrimaryEdgeuse() ;
  SmFilletEdgeuse * pMateEU2 = (SmFilletEdgeuse*)pPrimEU2->GetMate() ;

  // Save pCurve2 in rail(2)->Curve
  pRail2->SetCurve(pCurve2, TRUE) ;  // TRUE = also delete preExisting m_pCurve 
                                     // side effect: delete current pRail1->UVTrimCurves
  pCurve2->SetOwner(pRail2) ;
  sCrvIvl = pCurve2->GetNaturalInterval() ;
  pRail2->SetInterval(sCrvIvl) ;
    
  // Save pUV2 in rail(2)->Edgeuse->Mate->UVCurve
  // pBS = pMateEU2->GetUVTrimCurvePointer();  // no longer needed - gets deleted in SetUVCurve
  pMateEU2->SetUVCurve(pUV2, TRUE) ; // TRUE = also delete preExisting UVTrimCurve - expect none here
  pUV2->SetOwner(pMateEU2) ;
  // if (pBS && pBS != pUV2) { delete pBS; pBS = NULL ; }  // no longer needed - already deleted
    
  // restore FilletSolver offset distances in offset Surfaces stored in the FilletSolver
  m_rFilletSolver.ReloadSurfaceOffsets() ;

  // all done
  return SM_SUCCESS ;

} // end SmFilletIntersector::FlushCurve

/*******************************************************************//**
PURPOSE: Compute the UV values of Surface/Surface/Plane intersection with
         given a guess point on the two surfaces.

NOTES:
***********************************************************************/
SmStatus SmFilletIntersector::EvaluateLawPoint
 (double              dParameter,              // NotUsed: in : filletCurve parameter - used to compute current fillet radius
  const SmPoint2d   & crUV0,                   // in : guess uv point on m_cpSurface[0]
  const SmPoint2d   & crUV1,                   // in : guess uv point on m_cpSurface[1] 
  const SmFilletLaw & crLawCurve,              // NotUsed: in : FilletLaw to compute fillet-radius for every Curve param 
  SmBoolean           bLawOrient,              // NotUsed: in : bReverseOrientation: TRUE = parameters run from interval end to interval start
  const SmExtent1d  & crPointCurveInterval,    // NotUsed: in : crCurveInterval = =interval defining range of fillet edge 
  double              dSurfaceOrientations[2], // NotUsed: in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values. 
  const SmPoint3d   & rPlaneOrigin,            // in : Origin of Plane(origin, normal)
  const SmVector3d  & rPlaneNormal,            // in : Normal of Plane(origin, normal)
  double            & rdCurveParam,            // out: Curve Param
  SmPoint2d           sUVs[2])                 // out: Surf Params of m_pSurface[0]/m_pSurface[0]/plane XSect result
{
  SM_REF5(dParameter, crLawCurve, bLawOrient, crPointCurveInterval, dSurfaceOrientations) ;
  SmTsectPnt sTmpPnt;
  SmBoolean  bFoundSolution;

  // find filletPoint on given plane
  //  satisfying geometry requirements implemented in derived SmFilletSolver class
  SER(m_rFilletSolver.PointOnPlaneSolve(rPlaneOrigin,
                                        rPlaneNormal,
                                        m_vUVDomain[0],
                                        m_vUVDomain[1],
                                        crUV0,
                                        crUV1,
                                        bFoundSolution,
                                        sTmpPnt));

  // set output
  sUVs[0]      = sTmpPnt.UVPos(0);
  sUVs[1]      = sTmpPnt.UVPos(1);
  rdCurveParam = sTmpPnt.m_adUserDoubles[0];

  // all done
  return SM_SUCCESS;

} // end SmFilletIntersector::EvaluateLawPoint

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletIntersector::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletIntersector_TYPE == t) ? TRUE : SmAdvSurfaceIntersector::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print   

NOTES: 
***********************************************************************/
void SmFilletIntersector::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmFilletIntersector::Dump()")) ;

  // dump base
  SmAdvSurfaceIntersector::Dump() ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmFilletIntersector::Dump()\n")) ;

} // end SmFilletIntersector::Dump



