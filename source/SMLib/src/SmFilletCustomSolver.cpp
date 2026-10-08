// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCustomFilletSolver.cpp 
* PURPOSE: Source code file for SmFilletSolver object.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineSurface.h>
#include <SmFilletCustomSolver.h>
#include <SmAxis2Placement.h>
#include <SmGeomUtility.h>
#include <SmSurfOfExtrusion.h>


/*******************************************************************//**
PURPOSE: Constructor for 'Eiffel Tower' Fillet object where fillet
    radii are different for two offset surfaces

NOTES: The input pEdgeuse corresponds to the surface whose offset
    distance is dFilletRadius1. The cross-sections of the fillet
    (i.e. 'Eiffel tower') are determined by: (1) two different offset radii,
    dFilletRadius1 & dFilletRadius2 (where dFilletRadius1 > dFilletRadius2)
    (2) a given swept angle of the circular arc whose radius is dFilletRadius1.

Example:

        // Define a fillet surface generator
        SmEiffelTowerCrossSectionFSG sFSGEiffelTower;

        // Define solver object
        SmEiffelTowerFS * pFS = new(crContext) SmEiffelTowerFS(crContext,
            pBrep->GetTolerance(),30.0*SM_PI/180.0,2.0*SM_PI/180.0,
            dBallRadius1,dBallRadius2,dAngleInDegrees*SM_PI/180.0,pEdgeuse);

        // Associate fillet-surface generator to fillet solver
        pFS->SetFilletSurfaceGenerator(&sFSGEiffelTower);

***********************************************************************/
SmEiffelTowerFS::SmEiffelTowerFS(const SmContext & crContext,
                                 double dThisApproxTol3d,
                                 double dAngleTolerance,
                                 double dTangencyTolerance,
                                 double dFilletRadius1,
                                 double dFilletRadius2,
                                 double dTowerAngle, // in Degrees
                                 SmEdgeuse *pEdgeuse) 
: SmConstantRadiusAssistedFS(crContext,dThisApproxTol3d,dAngleTolerance,
    dTangencyTolerance,dFilletRadius1,dFilletRadius2,pEdgeuse),
  m_dTowerAngle(dTowerAngle)
{
}


/*******************************************************************//**
PURPOSE: Load the initial values for the solver.

NOTES: 
***********************************************************************/
SmStatus SmEiffelTowerFS::LoadInitialValues(ULONG & rbDoSurf2Calcs,
                                            ULONG lRailIndex,
                                            const SmSurface & crSurface1,
                                            ULONG lSurface1Index,
                                            SmSurface *& rpSurface2,
                                            ULONG & rlSurface2Index,
                                            SmExtentNd & rIntervals,
                                            SmTArray<SmBoolean> & rPeriodicities,
                                            SmTArray<double> & rGuessT)
{
    if ( rbDoSurf2Calcs == 0 ) {
        // Don't really need to do anything because both surfaces have already
        // been loaded.
        return SM_SUCCESS;
    }

    ULONG lOtherRailIndex = 1 - lRailIndex;
    rpSurface2 = GetSurface(lOtherRailIndex);

    SmPoint2d sUV1(rGuessT[lSurface1Index],rGuessT[lSurface1Index+1]);
    SmPoint2d sUV2;
    SmPoint3d sPnt;
    SER(crSurface1.EvaluatePoint(sUV1,sPnt));
    const SmSurface *pBase1 = ((SmOffsetSurface&)crSurface1).GetBaseSurface();
    const SmSurface *pBase2 = ((SmOffsetSurface*)rpSurface2)->GetBaseSurface();
    SmExtent2d sDomain1 = crSurface1.GetNaturalUVDomain();
    SmExtent2d sDomain2 = rpSurface2->GetNaturalUVDomain();

    // Now find the UV guess on the other Rail's surface
    SmSolution sSData[16];
    SmSolutionArray sSolutions(16,sSData);
    SmCurve * p3DFilletEdgeCurve = NULL;
    double dFilletEdgeT = 0.0;

    // If we have edgeuses then use them to compute a point on the other surfaces
    // If not then drop the point to the other surface.
    SmEdgeuse *pOtherEU = m_pEdgeuses[lOtherRailIndex];
    if ( pOtherEU != NULL && pOtherEU->GetUVTrimCurve() == NULL )
      { pOtherEU = NULL; }

    // But: if we are rolling on a different face (e.g., doing a rollover),
    // then this is the wrong Edgeuse, and we don't have an appropriate one to use.
    if ( pOtherEU != NULL )
      {
        SmFace *pF = pOtherEU->GetFace();
        if ( pF != NULL && pF->GetSurface() != pBase2 )
          { pOtherEU = NULL; }
      }

    if ( pOtherEU != NULL )
      {
        p3DFilletEdgeCurve = m_pEdgeuses[lOtherRailIndex]->GetEdge()->GetCurve();
        SmExtent1d sIvl = m_pEdgeuses[lOtherRailIndex]->GetEdge()->GetInterval();
        SER(p3DFilletEdgeCurve->GlobalPointSolve(sIvl,SM_SO_MINIMIZE,sPnt,
            m_dThisApproxTol3d,NULL,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() != 1) SER(SM_ERR);
        dFilletEdgeT = sSolutions[0].m_vStart[0];

        // If the curve is closed, and the solution is within tol of an
        // end point, snap the solution to whichever end (start or finish)
        // such that the direction that moves towards the interior of the curve
        // also moves generally towards the interior of the surface.

        if ( p3DFilletEdgeCurve->IsClosed( sIvl ) )
        {
            SmPoint3d s3DPnt;
            SER( p3DFilletEdgeCurve->EvaluatePoint( dFilletEdgeT, s3DPnt ));
            SmVector3d sInwardVector;
            SER( crSurface1.ComputeInwardVector( sUV1, sInwardVector ));
            double dDist, dParam;
            SmBoolean bSuccess;
            SER( p3DFilletEdgeCurve->DropPoint(sIvl,               // in : target curve allowed domain
                                               s3DPnt,             // in : Point to drop to curve
                                               &sInwardVector,     // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                   //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                   //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                               m_dThisApproxTol3d, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance. 
                                                                   //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                   //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                   //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                               NULL,               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                               bSuccess,           // out: TRUE = found a drop point
                                               dParam,             // out: found drop curve param
                                               dDist)) ;           // out: found drop distance
                                                                   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                   //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                   //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                   //      default:[SM_SO_MINIMIZE] to preserve original behavior
            if ( bSuccess )
                dFilletEdgeT = dParam;
        }

        // Now determine edgeuses, surfaces and orientations for current 
        // iteration of the solver.
        SmBSplineCurve *pOtherUVCurve = m_pEdgeuses[lOtherRailIndex]->GetUVTrimCurve();
        SmPoint3d s2DPnt;
        SER(pOtherUVCurve->EvaluatePoint(dFilletEdgeT,s2DPnt));
        sUV2.x = s2DPnt.x;
        sUV2.y = s2DPnt.y;

        // Before we add it drop point to other surface to get a better guess
        SmBoolean  bFound;
        SmSolution sSol;
        SER(rpSurface2->LocalPointSolve(sDomain2,SM_SO_MINIMIZE,sPnt,
            sUV2,bFound,sSol));
        if (bFound) {
            double dDist = sSol.m_vStart.m_dSolutionValue;
            if (dDist < (smos_Fabs(m_dFilletRadii[0]) + smos_Fabs(m_dFilletRadii[1])) ) {
                sUV2.x = sSol.m_vStart[0];
                sUV2.y = sSol.m_vStart[1];
            }
        }
    }
    else {
        SER(pBase2->GlobalPointSolve(pBase2->GetNaturalUVDomain(),
            SM_SO_MINIMIZE,sPnt,SM_EFF_ZERO_SQ,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() != 1) SER(SM_ERR);

        sUV2.x = sSolutions[0].m_vStart[0];
        sUV2.y = sSolutions[0].m_vStart[1];
    }

    SmVector3d  sNormal1, sNormal2;
    SER(pBase1->EvaluateNormal(sUV1,TRUE,TRUE,sNormal1));
    SER(pBase2->EvaluateNormal(sUV2,TRUE,TRUE,sNormal2));
    SER(sNormal1.Unitize());
    SER(sNormal2.Unitize());
    SmPoint3d  sPlaneOrig;
    SmVector3d sPlaneNormal;

    if (p3DFilletEdgeCurve) {
        SmVector3d sPV[2];
        SER(p3DFilletEdgeCurve->Evaluate(dFilletEdgeT,1,TRUE,sPV));
        sPlaneOrig = sPV[0];
        sPlaneNormal = sPV[1];
    }
    else {
        sPlaneOrig = sPnt;
        sPlaneNormal = sNormal1*sNormal2;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
    if (bDebugMe0) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        sPlaneNormal.Draw(&sPlaneOrig);
        sm_GraphicsLoop();
        SmPoint3d sTmpPnt, sTmpPnt2;
        smgfx_SetColor(1,0,0);
        pBase1->EvaluatePoint(sUV1,sTmpPnt);
        sTmpPnt.Draw();
        sm_GraphicsLoop();
        pBase2->EvaluatePoint(sUV2,sTmpPnt2);
        sTmpPnt2.Draw();
        sm_GraphicsLoop();
    }
#endif

    SmPoint2d sMin, sMax;
    sMin.x = rIntervals[lSurface1Index].GetMin();
    sMin.y = rIntervals[lSurface1Index+1].GetMin();
    sMax.x = rIntervals[lSurface1Index].GetMax();
    sMax.y = rIntervals[lSurface1Index+1].GetMax();
    sDomain1.SetMinMax(sMin,sMax);
    SmVector2d sSize2 = sDomain2.GetSize();
    sSize2 = m_dSurfaceExtensionFactor * sSize2;
    sMin.x = sDomain2.GetMin().x - sSize2.x;
    sMin.y = sDomain2.GetMin().y - sSize2.y;
    sMax.x = sDomain2.GetMax().x + sSize2.x;
    sMax.y = sDomain2.GetMax().y + sSize2.y;
    sDomain2.SetMinMax(sMin,sMax);

    double dRadDiff = m_dFilletRadii[0] - m_dFilletRadii[1];
    // Will first solve for the regular tangent case on the cross-section plane
    SmBoolean bFoundSolution = FALSE;
    double dScaleFactor = 1.2;
    SmTsectPnt sTsectPnt;
    if (lRailIndex == 0) {
        SER(GuessPointOnPlaneSolve(sPlaneOrig,sPlaneNormal,
                                            sDomain1,sDomain2,
                                            sUV1,sUV2,
                                            bFoundSolution,
                                            sTsectPnt));
        if (!bFoundSolution) SER(SM_ERR);
        sUV1.x = sTsectPnt.UVPos(0).x;
        sUV1.y = sTsectPnt.UVPos(0).y;
        sUV2.x = sTsectPnt.UVPos(1).x;
        sUV2.y = sTsectPnt.UVPos(1).y;
        // Then we want to move sUV1 along sNormal2 for a (3D-)distance
        // of (R1-R2)*sin(m_dTowerAngle) as our final guess sUV1
        // Similarily, move sUV2 along -sNormal1 for a distance of
        // (R1-R2)*cos(m_dTowerAngle) as our final guess sUV2
        double dLen1 = m_dOrientations[1]*dScaleFactor*dRadDiff*smos_Sine(m_dTowerAngle);
        SmVector3d sV1 = dLen1*sNormal2;
        SmVector2d sDeltaUV1;
        SER(pBase1->DropVectors(sUV1,TRUE,TRUE,1,&sV1,&sDeltaUV1));
        sUV1 = sUV1 + sDeltaUV1;
        double dLen2 = -m_dOrientations[0]*dRadDiff*smos_Cosine(m_dTowerAngle);
        SmVector3d sV2 = dLen2*sNormal1;
        SmVector2d sDeltaUV2;
        SER(pBase2->DropVectors(sUV2,TRUE,TRUE,1,&sV2,&sDeltaUV2));
        sUV2 = sUV2 + sDeltaUV2;
    }
    else {
        SER(GuessPointOnPlaneSolve(sPlaneOrig,sPlaneNormal,
                                            sDomain2,sDomain1,
                                            sUV2,sUV1,
                                            bFoundSolution,
                                            sTsectPnt));
        if (!bFoundSolution) SER(SM_ERR);
        sUV1.x = sTsectPnt.UVPos(1).x;
        sUV1.y = sTsectPnt.UVPos(1).y;
        sUV2.x = sTsectPnt.UVPos(0).x;
        sUV2.y = sTsectPnt.UVPos(0).y;
        // Then we want to move sUV2 along sNormal1 for a (3D-)distance
        // of (R1-R2)*sin(m_dTowerAngle) as our final guess sUV2
        // Similarily, move sUV2 along -sNormal1 for a distance of
        // (R1-R2)*cos(m_dTowerAngle) as our final guess sUV2
        double dLen2 = m_dOrientations[1]*dScaleFactor*dRadDiff*smos_Sine(m_dTowerAngle);
        SmVector3d sV2 = dLen2*sNormal1;
        SmVector2d sDeltaUV2;
        SER(pBase2->DropVectors(sUV2,TRUE,TRUE,1,&sV2,&sDeltaUV2));
        sUV2 = sUV2 + sDeltaUV2;
        double dLen1 = -m_dOrientations[0]*dRadDiff*smos_Cosine(m_dTowerAngle);
        SmVector3d sV1 = dLen1*sNormal2;
        SmVector2d sDeltaUV1;
        SER(pBase1->DropVectors(sUV1,TRUE,TRUE,1,&sV1,&sDeltaUV1));
        sUV1 = sUV1 + sDeltaUV1;
    }

#ifdef SM_DEBUG_CODE
    if (bDebugMe0) {
        SmPoint3d sTmpPnt;
        pBase1->EvaluatePoint(sUV1,sTmpPnt);
        smgfx_SetColor(0,0,1);
        sTmpPnt.Draw();
        sm_GraphicsLoop();
        SmPoint3d sTmpPnt2;
        pBase2->EvaluatePoint(sUV2,sTmpPnt2);
        smgfx_SetColor(1,0,0);
        sTmpPnt2.Draw();
        sm_GraphicsLoop();
    }
#endif

    rIntervals[rGuessT.GetSize()] = SmExtent1d(sDomain2.GetMin().x,sDomain2.GetMax().x);
    rIntervals[rGuessT.GetSize()+1] = SmExtent1d(sDomain2.GetMin().y,sDomain2.GetMax().y);

    rPeriodicities.Add(FALSE);
    rPeriodicities.Add(FALSE);
    
    rlSurface2Index = rGuessT.GetSize();
    rGuessT[lSurface1Index]   = sUV1.x;
    rGuessT[lSurface1Index+1] = sUV1.y;

    rGuessT.Add(sUV2.x);
    rGuessT.Add(sUV2.y);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        m_pEdgeuses[0]->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        sPnt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        ((SmOffsetSurface*)rpSurface2)->GetBaseSurface()->DrawUV(5,5);
        SmPoint3d s3DPnt;
        rpSurface2->EvaluatePoint(sUV2,s3DPnt);
        smgfx_SetPointSize(8);
        s3DPnt.Draw();
        smgfx_SetPointSize(4);
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Load the current Jacobian matrix in an incremental way
    and update the parameters.

NOTES:

  With: crX = [u1 v1 u2 v2]
         F  = OffsetSurf1.Position(u1, v1) 
         G  = OffsetSurf2.Position(u2, v2)
        nF  = BaseSurf1.UnitSurfaceNormal(u1, v1)
        nG  = BaseSurf2.UnitSurfaceNormal(u2, v2)
     dDiffR = m_dFilletRadii[0] - m_dFilletRadii[1]
         L  = m_dDistance, specified distance between rail curves
  Sets: 
    
  rF[]                 = done elsewhere
  rF[rlNumEquations+0] = (F-G).(F-G) - diffR*diffR           - Distance between f & g = diffR
  rF[rlNumEquations+1] = (nFxnG).(F-G)                       - nF, nG, F, G are coplanar
                          nF     (F-G)                                                    
  rF[rlNumEquations+2] = ---- . ------- - cos(m_dTowerAngle) - Angle between nF & (F-G) = m_dTowerAngle
                         |nF|    |F-G|                                                    

  pOptJacobian[][]                 = done elsewhere
                                     [          df                   df                     dg                   dg      ]
  pOptJacobian[rlNumEquations+0][] = [  2*(f-g).---          2*(f-g).---           -2*(f-g).---         -2*(f-g).---     ]
                                     [          duf                  dvf                    dug                  dvg     ]
                                     [  dN3                  dN3          df       dN3          dg      dN3          dg  ]
  pOptJacobian[rlNumEquations+1][] = [  ---.(f-g)+N3.duf     ---.(f-g)+N3.---      ---.(f-g)-N3.---     ---.(f-g)-N3.--- ]
                                     [  duf                  dvf          dvf      dug          dug     dvg          dvg ]
                                     [                                                                                   ]
  pOptJacobian[rlNumEquations+2][] = [ (dUnitNF/duf).UnitV+  (dUnitNF/dvf).UnitV+   UnitNF.              UnitNF.         ]
                                     [   UnitNF.(dUnitV/duf)   UnitNF.(dUnitV/dvf)    (dUnitV/dug)         (dUnitV/dvg)  ]

 
***********************************************************************/
SmStatus SmEiffelTowerFS::LoadJacobian
  (ULONG                  & rlNumEquations,       // i/o: number of equations to skip when setting
                                                  //      outputs rF and pOptJacobian
   ULONG                  & rlNumParameters,      // i/o: Gets incremented in an odd way to
                                                  //      to point to the input UVPnts.
   ULONG                    lRailIndex,           // in : Specifies rail index of first surface.
   SmSurface             *& rpSurface1,           // in : used only to map which crX[] uvPnt to surface1
   ULONG                  & rlSurf1Offset,        // i/o: index of rpSurface1 uvPnt in crX - ignored if rpSurface1 == NULL
   SmSurface             *& rpSurface2,           // in : used only to map which crX[] uvPnt to surface2
   ULONG                  & rlSurf2Offset,        // i/o: index of rpSurface2 uvPnt in crX - ignored if rpSurface2 == NULL
   const SmTArray<double> & crX,                  // in : current solution, [u1 v1 u2 v2 radius]
   SmTArray<double>       & rF,                   // out: sets rF[rlNumEquations] to rF[rlNumEquations+3] values
   SmMatrix               * pOptJacobian,         // out: sets pOptJacobian[rlNumEquations][] to pOptJacobian[rlNumEquations+3][] rows
   SmBoolean              & rbFoundAnswer)        // out: TRUE = all rF values less than scaled m_dConversionTol
{
  // init output
  rbFoundAnswer = FALSE;

  // Always need to reset offset surface offset distance prior to solving
  GetSurface(0)->SetOffsetDistance(m_dFilletRadii[0]*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(m_dFilletRadii[1]*m_dOrientations[1]);

  // get lSurf1Off index into crX array for lSurf1 parameters
  ULONG lSurf1Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 0 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() == 
      GetSurface(0)->GetBaseSurface())) {
      lSurf1Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 1 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      GetSurface(0)->GetBaseSurface())) {
      lSurf1Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }

  // get BaseSurface1 F point values for given crX values
  SmVector3d sF,sDUF,sDVF,sDUVF,sDUUF,sDVVF;
  SmPoint2d sUV(crX[lSurf1Off],crX[lSurf1Off+1]);
  GetSurface(0)->SetOffsetDistance(0.0);
  SER(GetSurface(0)->Evaluate2ndDerivatives(sUV,TRUE,TRUE,sF,
      sDUF,sDVF,sDUVF,sDUUF,sDVVF));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,1,0);
      GetSurface(0)->DrawAt(sUV,1);
      sm_GraphicsLoop();
  }
#endif

  // get lSurf2Off index into crX array for lSurf2 parameters
  ULONG lSurf2Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 1 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() ==
      GetSurface(1)->GetBaseSurface())) {
      lSurf2Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 0 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      GetSurface(1)->GetBaseSurface())) {
      lSurf2Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }

  // get BaseSurface2 G point values for given crX values
  SmPoint2d sUV2(crX[lSurf2Off],crX[lSurf2Off+1]);
  SmVector3d sG,sDUG,sDVG,sDUVG,sDUUG,sDVVG;
  GetSurface(1)->SetOffsetDistance(0.0);
  SER(GetSurface(1)->Evaluate2ndDerivatives(sUV2,TRUE,TRUE,sG,
      sDUG,sDVG,sDUVG,sDUUG,sDVVG));
#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      GetSurface(1)->DrawAt(sUV2,1);
      sm_GraphicsLoop();
  }
#endif

  // check surface tangent magnitudes 
  double sDUFLenSq = sDUF.LengthSquared();
  double sDVFLenSq = sDVF.LengthSquared();

  double sDUGLenSq = sDUG.LengthSquared();
  double sDVGLenSq = sDVG.LengthSquared();

  // failure: zero length tangent
  if (   sDUFLenSq < SM_EFF_ZERO_SQ 
      || sDVFLenSq < SM_EFF_ZERO_SQ
      || sDUGLenSq < SM_EFF_ZERO_SQ 
      || sDVGLenSq < SM_EFF_ZERO_SQ) 
    { return SM_ERR; }

  // get baseSurface SurfaceNormals at current solution points
  SmVector3d sNF = sDUF * sDVF; // Normal on base surface #1
  SmVector3d sNG = sDUG * sDVG; // Normal on base surface #2

  // failure: zero length SurfaceNormal
  if (   sNF.LengthSquared() < SM_EFF_ZERO_SQ
      || sNG.LengthSquared() < SM_EFF_ZERO_SQ) 
    { SER(SM_ERR); }

  // get derivatives of SurfaceNormals
  SmVector3d sNFdu = sDUF.DerivativeOfCrossProduct(sDVF,sDUUF,sDUVF);
  SmVector3d sNFdv = sDUF.DerivativeOfCrossProduct(sDVF,sDUVF,sDVVF);
  SmVector3d sNGdu = sDUG.DerivativeOfCrossProduct(sDVG,sDUUG,sDUVG);
  SmVector3d sNGdv = sDUG.DerivativeOfCrossProduct(sDVG,sDUVG,sDVVG);

  // get fillet-curve tangent direction and its derivatives 
  // from base surface surface normal values
  SmVector3d sN3    = sNF*sNG;
  SmVector3d sN3duf = sNFdu * sNG;
  SmVector3d sN3dvf = sNFdv * sNG;
  SmVector3d sN3dug = sNF * sNGdu;
  SmVector3d sN3dvg = sNF * sNGdv;

  // normalize SurfaceNormal derivatives
  SmVector3d sUnitNFdu = m_dOrientations[0]*sNF.UnitizedDerivative(sNFdu);
  SmVector3d sUnitNFdv = m_dOrientations[0]*sNF.UnitizedDerivative(sNFdv);
  SmVector3d sUnitNF   = m_dOrientations[0]*sNF;
  SER(sUnitNF.Unitize());

  // evaluate offsetSurface F and G point positions
  GetSurface(0)->SetOffsetDistance(m_dFilletRadii[0]*m_dOrientations[0]);
  SER(GetSurface(0)->Evaluate1stDerivatives(sUV,TRUE,TRUE,sF,sDUF,sDVF));
  GetSurface(1)->SetOffsetDistance(m_dFilletRadii[1]*m_dOrientations[1]);
  SER(GetSurface(1)->Evaluate1stDerivatives(sUV2,TRUE,TRUE,sG,sDUG,sDVG));

  // set sV = distance between offsetSurface points
  SmVector3d sV = sF - sG;
  SmVector3d sUnitVduf = sV.UnitizedDerivative( sDUF);
  SmVector3d sUnitVdvf = sV.UnitizedDerivative( sDVF);
  SmVector3d sUnitVdug = sV.UnitizedDerivative(-sDUG);
  SmVector3d sUnitVdvg = sV.UnitizedDerivative(-sDVG);
  SmVector3d sUnitV = sV;
  SER(sUnitV.Unitize());

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      sV.Draw(&sG);
      sm_GraphicsLoop();
  }
#endif

  double dDiffR = m_dFilletRadii[0] - m_dFilletRadii[1];
  // Intersection simply tries to minimize the following three equations.
  //
  // fun[i+0] = (f-g).(f-g) - diffR*diffR      - Distance between f & g = diffR
  //
  // fun[i+1] = (NFxNG).(f-g)                  - NF, NG, f, g are coplanar
  //
  //             NF     (f-g)
  // fun[i+2] = ---- . ------- - cos(alpha)    - Angle between NF & (f-g) = alpha
  //            |NF|    |f-g|
  //
  // It produces the following Jacobian
  //  |         df                   df                     dg                   dg     |
  //  | 2*(f-g).---          2*(f-g).---           -2*(f-g).---         -2*(f-g).---    |
  //  |         duf                  dvf                    dug                  dvg    |
  //  | dN3                  dN3          df       dN3          dg      dN3          dg | 
  //  | ---.(f-g)+N3.duf     ---.(f-g)+N3.---      ---.(f-g)-N3.---     ---.(f-g)-N3.---|
  //  | duf                  dvf          dvf      dug          dug     dvg          dvg|
  //  |                                                                                 |
  //  |(dUnitNF/duf).UnitV+  (dUnitNF/dvf).UnitV+   UnitNF.              UnitNF.        |
  //  |  UnitNF.(dUnitV/duf)   UnitNF.(dUnitV/dvf)    (dUnitV/dug)         (dUnitV/dvg) |
  
  if (pOptJacobian) 
    {
      ULONG i = rlNumEquations;
      (*pOptJacobian)[i][lSurf1Off]   =  2.0 * sV.Dot(sDUF);
      (*pOptJacobian)[i][lSurf1Off+1] =  2.0 * sV.Dot(sDVF);
      (*pOptJacobian)[i][lSurf2Off]   = -2.0 * sV.Dot(sDUG);
      (*pOptJacobian)[i][lSurf2Off+1] = -2.0 * sV.Dot(sDVG);

      i++; // Move to next equation;
      (*pOptJacobian)[i][lSurf1Off]   = sN3duf.Dot(sV) + sN3.Dot(sDUF);
      (*pOptJacobian)[i][lSurf1Off+1] = sN3dvf.Dot(sV) + sN3.Dot(sDVF);
      (*pOptJacobian)[i][lSurf2Off]   = sN3dug.Dot(sV) - sN3.Dot(sDUG);
      (*pOptJacobian)[i][lSurf2Off+1] = sN3dvg.Dot(sV) - sN3.Dot(sDVG);

      i++;
      (*pOptJacobian)[i][lSurf1Off]   = sUnitNFdu.Dot(sUnitV) + sUnitNF.Dot(sUnitVduf);
      (*pOptJacobian)[i][lSurf1Off+1] = sUnitNFdv.Dot(sUnitV) + sUnitNF.Dot(sUnitVdvf);
      (*pOptJacobian)[i][lSurf2Off]   = sUnitNF.Dot(sUnitVdug);
      (*pOptJacobian)[i][lSurf2Off+1] = sUnitNF.Dot(sUnitVdvg);
    }
  
  // Compute function values
  ULONG lInitialIndex  = rlNumEquations;
  rF[rlNumEquations++] = sV.Dot(sV) - dDiffR*dDiffR;
  rF[rlNumEquations++] = sN3.Dot(sV);
  rF[rlNumEquations++] = sUnitNF.Dot(sUnitV) - smos_Cosine(m_dTowerAngle);

  //double dM = sN3.Length() * sV.Length();

  // See if we have converged
  double dScaledTol = GetConversionTol() * (1.0 + sF.GetMaxDimension());
  if (   smos_Fabs(rF[lInitialIndex])   < dScaledTol 
      && smos_Fabs(rF[lInitialIndex+1]) < dScaledTol 
      && smos_Fabs(rF[lInitialIndex+2]) < dScaledTol) 
    {
      rbFoundAnswer = TRUE;
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      if (pOptJacobian) pOptJacobian->Dump();
      smos_WriteBuffer(_T(" F --- \n"));
      rF.Dump();
  }
#endif

  return SM_SUCCESS;

} // end SmEiffelTowerFS::LoadJacobian


/*******************************************************************//**
PURPOSE: Create a fillet surface with a circular cross section.  This
    is a lofted surface or in some special cases a cylinder or a torus.

NOTES: 
***********************************************************************/
SmStatus SmEiffelTowerCrossSectionFSG::CreateSurface(SmFilletSolver & rFilletSolver,
                                                     SmBSplineCurve * pCenterLine,
                                                     SmBSplineCurve * pRail1Curve,
                                                     SmBSplineCurve * pRail2Curve,
                                                     const SmTArray<SmTsectPnt*> & crFilletPoints,
                                                     SmSurface *,  
                                                     SmBSplineCurve *,
                                                     SmSurface *,
                                                     SmBSplineCurve *,
                                                     SmBSplineSurface *& rpFilletSurface)
{
    const SmContext & crContext = rFilletSolver.GetCreationContext();
    double dThisApproxTol3d = rFilletSolver.GetThisApproxTol3d();
    SmBoolean bNeedG1Continuity = TRUE;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
    if (bDebugMe) {
        smgfx_SetColor(0,0,1);
        pCenterLine->DrawWithKnots();
        pCenterLine->Dump();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        pRail1Curve->DrawWithKnots();
        pRail1Curve->Dump();
        sm_GraphicsLoop();
        smgfx_SetColor(0,1,0);
        pRail2Curve->DrawWithKnots();
        pRail2Curve->Dump();
        sm_GraphicsLoop();
    }
#else
    SM_REF1(pCenterLine);
#endif

    SmTArray<double> sBreaks;
    SmTArray<double> sKnots;
    SER(pRail1Curve->GetKnots(sKnots));
    SmTArray<double> sKnots2;
    SER(pRail2Curve->GetKnots(sKnots2));
    ULONG lTotalsCrossSections = sKnots.GetSize();
    if (lTotalsCrossSections != sKnots2.GetSize() ||
        lTotalsCrossSections != crFilletPoints.GetSize()) {
        SER(SM_ERR); // Curves not compatible
    }
    
    SmTArray<SmBSplineCurve*> sCrossSections(lTotalsCrossSections);
    SmObjsDelete<SmBSplineCurve*> sCleanCrossSections(&sCrossSections);
    SmTArray<double> sParametersOfCrossSections;
    
    double dAnalyticTol = 1.0e-8;
    for (ULONG i=0; i<lTotalsCrossSections; i++) {
        SmTsectPnt *pTSP = crFilletPoints[i];
        double dT = sKnots[i];
        sParametersOfCrossSections.Add(dT);
        //SmVector3d sCentPV[2];
        SmPoint3d sP1 = pTSP->SrfPos(0);
        SmPoint3d sP2 = pTSP->SrfPos(1);
                
        SmPoint3d sS1, sS2;
        SER(pRail1Curve->EvaluatePoint(dT,sS1));
        SER(pRail2Curve->EvaluatePoint(dT,sS2));
        SmBSplineCurve * pCrossSection = NULL;
        if (sS1.DistanceBetween(sS2) < dThisApproxTol3d) {
            // Create Point curve
            SER(SmBSplineCurve::CreatePointCurve(crContext,sP1,pCrossSection));  /* parameterized from 0 to 1 */
           // ULONG lNumCtrlPts = pCrossSection->GetNumberOfUniqueKnots();
            sCrossSections.Add(pCrossSection);
            continue;
        }
        
        SmVector3d sV = sS2 - sS1;
        SmVector3d sV1 = sS1 - sP1;
        SmVector3d sZVec = sV1 * sV;
        double dRadius = sV1.Length();
        sZVec.Unitize();

        // Project sP2 onto plane with normal sZVec
        sP2 = sP2.ProjectPointToPlane(sP1,sZVec);
        SmVector3d sV3 = sP2 - sP1;
        double dAngle;
        SER(sZVec.CCWAngleBetween(sV1,sV3,dAngle));

        SmAxis2Placement sPlacement;
        SER(sV1.Unitize());
        SmVector3d sYVec = sZVec * sV1;
        SER(sPlacement.SetCanonical(sP1,sV1,sYVec));
        SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sPlacement,
            dRadius,0.0,dAngle*180.0/SM_PI,SM_CO_QUADRATIC,pCrossSection));
#ifdef SM_DEBUG_CODE
        if (bDebugMe) {
            smgfx_SetColor(1,0,0);
            pCrossSection->DrawWithKnots();
            sm_GraphicsLoop();
        }
#endif
        // Will try to make the second arc begin with
        // the end point of the first arc
        SmExtent1d sIvl = pCrossSection->GetNaturalInterval();
        SmPoint3d sP3;
        SER(pCrossSection->EvaluatePoint(sIvl.GetMax(),sP3));
        SmVector3d sVecP3S2 = sS2 - sP3;
        SmPoint3d sMidP3S2 = (sS2 + sP3) / 2.0;
        SmVector3d sBisector = sZVec * sVecP3S2;
        SER(sBisector.Unitize());

        ULONG lNumIntersections;
        SmPoint3d aPoints[2];
        SER(smgu_SegmentSegmentIntersect(sP3,sP1,sMidP3S2,
            sMidP3S2 + sBisector*dRadius,dAnalyticTol,
            lNumIntersections,aPoints));
        if (lNumIntersections != 1) SER(SM_ERR);
        sP2 = aPoints[0];
        SmVector3d sV2 = sS2 - sP2;
        sV3 = sP3 - sP2;
        // Compute center of the arc #2
        double dRadius2 = sV3.Length();

        SmBSplineCurve * pArc2 = NULL;
        double dAngle2;
        SER(sZVec.CCWAngleBetween(sV3,sV2,dAngle2));
        if (dAngle2 > SM_EFF_ZERO_SQRT) {
            sYVec = sZVec * sV3;
            SER(sYVec.Unitize());
            SER(sV3.Unitize());
            SmAxis2Placement sPlacement2;
            SER(sPlacement2.SetCanonical(sP2,sV3,sYVec));
            SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sPlacement2,
                dRadius2,0.0,dAngle2*180.0/SM_PI,SM_CO_QUADRATIC,pArc2));
            SmObjDelete sDelete(pArc2);
#ifdef SM_DEBUG_CODE
            if (bDebugMe) {
                smgfx_SetColor(0,0,1);
                pArc2->DrawWithKnots();
                sm_GraphicsLoop();
            }
#endif
            // Join pCrossSection with pArc2
            SER(pCrossSection->JoinWith(1,pArc2,0));
        }

#ifdef SM_DEBUG_CODE
        if (bDebugMe) {
            smgfx_SetColor(1,0,0);
            pCrossSection->DrawWithKnots();
            sm_GraphicsLoop();
        }
#endif
        sCrossSections.Add(pCrossSection);
    } // for i

    SmBSplineSurface *pFilletSurface = NULL;

    SmPoint3d sLinePoint;
    SmVector3d sLineVector;
    SmPoint3d sLinePoint2;
    SmVector3d sLineVector2;
    SmBoolean bSweep = FALSE;
    if (pRail1Curve->IsLine(11,dAnalyticTol,sLinePoint,sLineVector) &&
        pRail2Curve->IsLine(11,dAnalyticTol,sLinePoint2,sLineVector2)) {
        double dAngle;
        SER(sLineVector2.AngleBetween(sLineVector,dAngle));
        if (dAngle < SM_EFF_ZERO_SQRT) {
            bSweep = TRUE;
        }
    }
    
    if (bSweep) {
        // Create linear sweep surface
        SmBSplineCurve *pStart = sCrossSections[0];
        SmBSplineCurve *pEnd = sCrossSections.GetLast();
        SER(SmBSplineSurface::CreateRuledSurface(crContext,*pStart,*pEnd,
            SM_SP_U,pFilletSurface));
        SmExtent1d sIvl = pRail1Curve->GetNaturalInterval();
        SmExtent2d sDomain(SmPoint2d(sIvl.GetMin(),0.0),
            SmPoint2d(sIvl.GetMax(),1.0));
        SER(pFilletSurface->Reparameterize(sDomain));
    }
    else {
        // See if we can create a surface of revolution
        double dRadius,dStartAng,dEndAng;
        SmAxis2Placement sRefFrame;
        double dRadius2, dStartAng2, dEndAng2;
        SmAxis2Placement sRefFrame2;
        if (pRail1Curve->IsArc(11,dAnalyticTol,sRefFrame,dRadius,dStartAng,dEndAng) &&
            pRail2Curve->IsArc(11,dAnalyticTol,sRefFrame2,dRadius2,dStartAng2,dEndAng2)) {
            // Create surface of revolution
            SmBSplineCurve *pGenCurve = sCrossSections[0];
            SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,pGenCurve,
                sRefFrame.GetOriginRef(),sRefFrame.GetZAxis(),dEndAng,pFilletSurface));
            SmExtent1d sIvl = pRail1Curve->GetNaturalInterval();
            SmExtent2d sDomain(SmPoint2d(sIvl.GetMin(),0.0),
                SmPoint2d(sIvl.GetMax(),1.0));
            SER(pFilletSurface->Reparameterize(sDomain));
        }
        else {
            // Create skinned surface
            SmBSplineSurface * pDerivSurfs[2] = { NULL, NULL };
            if (bNeedG1Continuity) {
                SmExtent1d sIvl = pRail1Curve->GetNaturalInterval();
                for (ULONG i=0; i<2; i++) {
                    // We need to provide cross-boundary derivative surface
                    // for skinned-surface creation.
                    // Create extrusion surface using end cross-section
                    // curve and the tangent vector of centerline.
                    // NOTE: Will work ONLY with constant-radius filleting.
                    SmSurfOfExtrusion * pSurf = NULL;
                    SmCurve *pNewCurve = NULL;
                    SmBSplineCurve * pGenCurve = sCrossSections[0];
                    double dT = sIvl.GetMin();
                    if (i == 1) {
                        pGenCurve = sCrossSections.GetLast();
                        dT = sIvl.GetMax();
                    }
                    SmVector3d sPV[2];
                    SER(pRail1Curve->Evaluate(dT,1,TRUE,sPV));
                    SER(pGenCurve->Copy(crContext,pNewCurve));
                    pGenCurve = (SmBSplineCurve*)pNewCurve;
                    SmExtent1d sCrvIvl = pGenCurve->GetNaturalInterval();
                    SER(SmSurfOfExtrusion::CreateCanonical(crContext,pGenCurve,
                        sPV[1],pSurf));
                    SmPoint2d sUVMin = SmPoint2d(sCrvIvl.GetMin(),0.0);
                    SmPoint2d sUVMax = SmPoint2d(sCrvIvl.GetMax(),1.0);
                    pSurf->AdjustSTEPUVDomain(SmExtent2d(sUVMin,sUVMax));
                    pDerivSurfs[i] = pSurf;
                }
                SmTArray<double> sKnotsLocal;
                pRail1Curve->GetKnots( sKnotsLocal );
                double dStartKnotMid = (sKnotsLocal[0] + sKnotsLocal[1]) / 2.0;
                pRail1Curve->InsertOneKnot(dStartKnotMid,1);
                pRail2Curve->InsertOneKnot(dStartKnotMid,1);
                double dEndKnotMid = (sKnotsLocal[sKnotsLocal.GetSize()-2] + sKnotsLocal.GetLast()) / 2.0;
                pRail1Curve->InsertOneKnot(dEndKnotMid,1);
                pRail2Curve->InsertOneKnot(dEndKnotMid,1);
            }
            SER(SmBSplineSurface::CreateSkinnedSurface(crContext,
                sCrossSections,FALSE,SM_SP_U,dThisApproxTol3d,
                pRail1Curve,pRail2Curve,FALSE,&sParametersOfCrossSections,
                pDerivSurfs,pFilletSurface));
            for (ULONG ii=0; ii<2; ii++) {
                if (pDerivSurfs[ii])  { delete pDerivSurfs[ii]; pDerivSurfs[ii] = NULL ; }
            }
        }
    }
    
    rpFilletSurface = pFilletSurface;
    
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2=FALSE;
    if (bDebugMe2) {
        smgfx_SetColor(1,0,0);
        pRail1Curve->DrawWDeriv(pRail1Curve->GetNaturalInterval());
        pRail2Curve->DrawWDeriv(pRail2Curve->GetNaturalInterval());
        smgfx_SetColor(0,0,1);
        // if (0) {
        //     pCenterLine->DrawWDeriv(pCenterLine->GetNaturalInterval());
        // }
        sm_GraphicsLoop();
        pFilletSurface->DrawUV(4,4);
        pFilletSurface->Dump();
        sm_GraphicsLoop();
    }
#endif
    
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Constructor for Constant Radius Fillet object.

NOTES: The input pEdgeuse corresponds to the surface which
    intersected with the rolling ball at a given secant angle
***********************************************************************/
SmSecantConstantRadiusFS::SmSecantConstantRadiusFS(const SmContext & crContext,
                                                   double dThisApproxTol3d,
                                                   double dAngleTolerance,
                                                   double dTangencyTolerance,
                                                   double dFilletRadius,
                                                   double dSecantAngle,
                                                   SmEdgeuse *pEdgeuse)
: SmConstantRadiusAssistedFS(crContext,dThisApproxTol3d,dAngleTolerance,
    dTangencyTolerance,dFilletRadius,dFilletRadius,pEdgeuse),
  m_dSecantAngle(dSecantAngle)
{
}


/*******************************************************************//**
PURPOSE: Load the initial values for the solver.

NOTES: 
***********************************************************************/
SmStatus SmSecantConstantRadiusFS::LoadInitialValues
  (ULONG & rbDoSurf2Calcs,
   ULONG lRailIndex,
   const SmSurface & crSurface1,
   ULONG lSurface1Index,
   SmSurface *& rpSurface2,
   ULONG & rlSurface2Index,
   SmExtentNd & rIntervals,
   SmTArray<SmBoolean> & rPeriodicities,
   SmTArray<double> & rGuessT)
{
    if ( rbDoSurf2Calcs == 0 ) {
        // Don't really need to do anything because both surfaces have already
        // been loaded.
        return SM_SUCCESS;
    }

    ULONG lOtherRailIndex = 1 - lRailIndex;
    rpSurface2 = GetSurface(lOtherRailIndex);

    SmPoint2d sUV1(rGuessT[lSurface1Index],rGuessT[lSurface1Index+1]);
    SmPoint2d sUV2;
    SmPoint3d sPnt;
    SER(crSurface1.EvaluatePoint(sUV1,sPnt));
    const SmSurface *pBase1 = ((SmOffsetSurface&)crSurface1).GetBaseSurface();
    const SmSurface *pBase2 = ((SmOffsetSurface*)rpSurface2)->GetBaseSurface();
    SmExtent2d sDomain1 = crSurface1.GetNaturalUVDomain();
    SmExtent2d sDomain2 = rpSurface2->GetNaturalUVDomain();

    // Now find the UV guess on the other Rail's surface
    SmSolution sSData[16];
    SmSolutionArray sSolutions(16,sSData);
    SmCurve * p3DFilletEdgeCurve = NULL;
    double dFilletEdgeT=0.0;

    // If we have edgeuses then use them to compute a point on the other surfaces
    // If not then drop the point to the other surface.
    SmEdgeuse *pOtherEU = m_pEdgeuses[lOtherRailIndex];
    if ( pOtherEU != NULL && pOtherEU->GetUVTrimCurve() == NULL )
      { pOtherEU = NULL; }

    // But: if we are rolling on a different face (e.g., doing a rollover),
    // then this is the wrong Edgeuse, and we don't have an appropriate one to use.
    if ( pOtherEU != NULL )
      {
        SmFace *pF = pOtherEU->GetFace();
        if ( pF != NULL && pF->GetSurface() != pBase2 )
          { pOtherEU = NULL; }
      }

    if ( pOtherEU != NULL )
    {
        p3DFilletEdgeCurve = m_pEdgeuses[lOtherRailIndex]->GetEdge()->GetCurve();
        SmExtent1d sIvl = m_pEdgeuses[lOtherRailIndex]->GetEdge()->GetInterval();
        SER(p3DFilletEdgeCurve->GlobalPointSolve(sIvl,SM_SO_MINIMIZE,sPnt,
            m_dThisApproxTol3d,NULL,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() != 1) SER(SM_ERR);
        dFilletEdgeT = sSolutions[0].m_vStart[0];

        // If the curve is closed, and the solution is within tol of an
        // end point, snap the solution to whichever end (start or finish)
        // such that the direction that moves towards the interior of the curve
        // also moves generally towards the interior of the surface.

        if ( p3DFilletEdgeCurve->IsClosed( sIvl ) )
        {
            SmPoint3d s3DPnt;
            SER( p3DFilletEdgeCurve->EvaluatePoint( dFilletEdgeT, s3DPnt ));
            SmVector3d sInwardVector;
            SER( crSurface1.ComputeInwardVector( sUV1, sInwardVector ));
            double dDist, dParam;
            SmBoolean bSuccess;
            SER( p3DFilletEdgeCurve->DropPoint(sIvl,                // in : target curve allowed domain
                                               s3DPnt,              // in : Point to drop to curve
                                               &sInwardVector,      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                    //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                    //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                               m_dThisApproxTol3d,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                    //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                    //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                               NULL,                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                               bSuccess,            // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                               dParam,              // out: TRUE = found a drop point
                                               dDist)) ;            // out: found drop curve param
                                                                    // out: found drop distance
                                                                    // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                    //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                    //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
            if ( bSuccess )                                         //      default:[SM_SO_MINIMIZE] to preserve original behavior
                dFilletEdgeT = dParam;
        }
        // Now determine edgeuses, surfaces and orientations for current 
        // iteration of the solver.
        SmBSplineCurve *pOtherUVCurve = m_pEdgeuses[lOtherRailIndex]->GetUVTrimCurve();
        SmPoint3d s2DPnt;
        SER(pOtherUVCurve->EvaluatePoint(dFilletEdgeT,s2DPnt));
        sUV2.x = s2DPnt.x;
        sUV2.y = s2DPnt.y;

        // Before we add it drop point to other surface to get a better guess
        SmBoolean  bFound;
        SmSolution sSol;
        SER(rpSurface2->LocalPointSolve(sDomain2,SM_SO_MINIMIZE,sPnt,
            sUV2,bFound,sSol));
        if (bFound) {
            double dDist = sSol.m_vStart.m_dSolutionValue;
            if (dDist < (smos_Fabs(m_dFilletRadii[0]) + smos_Fabs(m_dFilletRadii[1])) ) {
                sUV2.x = sSol.m_vStart[0];
                sUV2.y = sSol.m_vStart[1];
            }
        }
    }
    else {
        SER(pBase2->GlobalPointSolve(pBase2->GetNaturalUVDomain(),
            SM_SO_MINIMIZE,sPnt,SM_EFF_ZERO_SQ,NULL,SM_SR_SINGLE,sSolutions));
        if (sSolutions.GetSize() != 1) SER(SM_ERR);

        sUV2.x = sSolutions[0].m_vStart[0];
        sUV2.y = sSolutions[0].m_vStart[1];
    }

    SmVector3d  sNormal1, sNormal2;
    SER(pBase1->EvaluateNormal(sUV1,TRUE,TRUE,sNormal1));
    SER(pBase2->EvaluateNormal(sUV2,TRUE,TRUE,sNormal2));
    SER(sNormal1.Unitize());
    SER(sNormal2.Unitize());
    SmPoint3d  sPlaneOrig;
    SmVector3d sPlaneNormal;

    if (p3DFilletEdgeCurve) {
        SmVector3d sPV[2];
        SER(p3DFilletEdgeCurve->Evaluate(dFilletEdgeT,1,TRUE,sPV));
        sPlaneOrig = sPV[0];
        sPlaneNormal = sPV[1];
    }
    else {
        sPlaneOrig = sPnt;
        sPlaneNormal = sNormal1*sNormal2;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
    if (bDebugMe0) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        sPlaneNormal.Draw(&sPlaneOrig);
        sm_GraphicsLoop();
        SmPoint3d sTmpPnt, sTmpPnt2;
        smgfx_SetColor(1,0,0);
        pBase1->EvaluatePoint(sUV1,sTmpPnt);
        sTmpPnt.Draw();
        sm_GraphicsLoop();
        pBase2->EvaluatePoint(sUV2,sTmpPnt2);
        sTmpPnt2.Draw();
        sm_GraphicsLoop();
    }
#endif

    SmPoint2d sMin, sMax;
    sMin.x = rIntervals[lSurface1Index].GetMin();
    sMin.y = rIntervals[lSurface1Index+1].GetMin();
    sMax.x = rIntervals[lSurface1Index].GetMax();
    sMax.y = rIntervals[lSurface1Index+1].GetMax();
    sDomain1.SetMinMax(sMin,sMax);
    SmVector2d sSize2 = sDomain2.GetSize();
    sSize2 = m_dSurfaceExtensionFactor * sSize2;
    sMin.x = sDomain2.GetMin().x - sSize2.x;
    sMin.y = sDomain2.GetMin().y - sSize2.y;
    sMax.x = sDomain2.GetMax().x + sSize2.x;
    sMax.y = sDomain2.GetMax().y + sSize2.y;
    sDomain2.SetMinMax(sMin,sMax);

    // Will first solve for the regular tangent case on the cross-section plane
    SmBoolean bFoundSolution = FALSE;
    SmTsectPnt sTsectPnt;
    if (lRailIndex == 0) {
        SER(GuessPointOnPlaneSolve(sPlaneOrig,sPlaneNormal,
                                            sDomain1,sDomain2,
                                            sUV1,sUV2,
                                            bFoundSolution,
                                            sTsectPnt));
        if (!bFoundSolution) SER(SM_ERR);
        sUV1.x = sTsectPnt.UVPos(0).x;
        sUV1.y = sTsectPnt.UVPos(0).y;
        sUV2.x = sTsectPnt.UVPos(1).x;
        sUV2.y = sTsectPnt.UVPos(1).y;
        // Now we have a point corresponding to sUV2 where the ball touches
        // surface2 tangentially. Then we want to move sUV2 along -sNormal1 for
        // a (3-d)distance of 2*Radius*sin(theta) as our final guess point
        double dLen = -m_dOrientations[1]*m_dFilletRadii[1]*smos_Sine(m_dSecantAngle);
        SmVector3d sV = dLen*sNormal1;
        SmVector2d sDeltaUV;
        SER(pBase2->DropVectors(sUV2,TRUE,TRUE,1,&sV,&sDeltaUV));
        sUV2 = sUV2 + sDeltaUV*2.0;
    }
    else {
        SER(GuessPointOnPlaneSolve(sPlaneOrig,sPlaneNormal,
                                            sDomain2,sDomain1,
                                            sUV2,sUV1,
                                            bFoundSolution,
                                            sTsectPnt));
        if (!bFoundSolution) SER(SM_ERR);
        sUV1.x = sTsectPnt.UVPos(1).x;
        sUV1.y = sTsectPnt.UVPos(1).y;
        sUV2.x = sTsectPnt.UVPos(0).x;
        sUV2.y = sTsectPnt.UVPos(0).y;
        // Now we have a point corresponding to sUV1 where the ball touches
        // surface1 tangentially. Then we want to move sUV1 along -sNormal2 for
        // a (3-d)distance of 2*Radius*sin(theta) as our final guess point
        double dLen = -m_dOrientations[0]*m_dFilletRadii[0]*smos_Sine(m_dSecantAngle);
        SmVector3d sV = dLen*sNormal2;
        SmVector2d sDeltaUV;
        SER(pBase1->DropVectors(sUV1,TRUE,TRUE,1,&sV,&sDeltaUV));
        sUV1 = sUV1 + sDeltaUV*2.0;
    }

#ifdef SM_DEBUG_CODE
    if (bDebugMe0) {
        SmPoint3d sTmpPnt;
        pBase1->EvaluatePoint(sUV1,sTmpPnt);
        smgfx_SetColor(0,0,1);
        sTmpPnt.Draw();
        sm_GraphicsLoop();
        SmPoint3d sTmpPnt2;
        pBase2->EvaluatePoint(sUV2,sTmpPnt2);
        smgfx_SetColor(1,0,0);
        sTmpPnt2.Draw();
        sm_GraphicsLoop();
    }
#endif

    rIntervals[rGuessT.GetSize()] = SmExtent1d(sDomain2.GetMin().x,sDomain2.GetMax().x);
    rIntervals[rGuessT.GetSize()+1] = SmExtent1d(sDomain2.GetMin().y,sDomain2.GetMax().y);

    rPeriodicities.Add(FALSE);
    rPeriodicities.Add(FALSE);
    
    rlSurface2Index = rGuessT.GetSize();
    rGuessT[lSurface1Index]   = sUV1.x;
    rGuessT[lSurface1Index+1] = sUV1.y;

    rGuessT.Add(sUV2.x);
    rGuessT.Add(sUV2.y);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        m_pEdgeuses[0]->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        sPnt.Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        ((SmOffsetSurface*)rpSurface2)->GetBaseSurface()->DrawUV(5,5);
        SmPoint3d s3DPnt;
        rpSurface2->EvaluatePoint(sUV2,s3DPnt);
        smgfx_SetPointSize(8);
        s3DPnt.Draw();
        smgfx_SetPointSize(4);
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Load the current Jacobian matrix in an incremental way
    and update the parameters.

NOTES:
  With: crX = [u1 v1 u2 v2]
         F   = OffsetSurf1.Position(u1, v1) 
         G2  = BaseSurf2.Position(u2, v2)
         V2  = F-G2
         N1  = BaseSurf1.SurfaceNormal
         N2  = BaseSurf2.SurfaceNormal
         N3  = crossProduct(N1,N2)  - fillet center-curve direction

  Sets: 

  rF[]                 = done elsewhere
  rF[rlNumEquations+0] = V2.V2 - m_dFilletRadii[0]*m_dFilletRadii[1] - Distance between f and g2        
  rF[rlNumEquations+1] = V2.N2 - m_dFilletRadii[1]*smos_Cosine(m_dSecantAngle)                         
  rF[rlNumEquations+2] = V2.N3                                       - V2 is perpendicular to N3(=N1xN2)
                                          

  pOptJacobian[][]                 = done elsewhere
  pOptJacobian[rlNumEquations+0][] = [ 2*V2.duf   2*V2.dvf   -2*V2.dug2   -2*V2.dvg2 ]
  pOptJacobian[rlNumEquations+1][] = [ duf.sN2    dvf.sN2    -dug2.sN2    -dvg2.sN2  ]
  pOptJacobian[rlNumEquations+2][] = [ duf.sN3    dvf.sN3    -dug2.sN3    -dvg2.sN3  ]

  
***********************************************************************/
SmStatus SmSecantConstantRadiusFS::LoadJacobian
  (ULONG                  & rlNumEquations,       // i/o: number of equations to skip when setting
                                                  //      outputs rF and pOptJacobian
   ULONG                  & rlNumParameters,      // i/o: Gets incremented in an odd way to
                                                  //      to point to input UVPnts 
   ULONG                    lRailIndex,           // in : Specifies rail index of first surface.
   SmSurface             *& rpSurface1,           // in : used only to map which crX[] uvPnt to surface1
   ULONG                  & rlSurf1Offset,        // in : index of rpSurface1 uvPnt in crX - ignored if rpSurface1 == NULL
   SmSurface             *& rpSurface2,           // in : used only to map which crX[] uvPnt to surface2
   ULONG                  & rlSurf2Offset,        // in : index of rpSurface2 uvPnt in crX - ignored if rpSurface2 == NULL
   const SmTArray<double> & crX,                  // in : current solution, [u1 v1 u2 v2 radius]
   SmTArray<double>       & rF,                   // out: sets rF[rlNumEquations] to rF[rlNumEquations+3] values
   SmMatrix               * pOptJacobian,         // out: sets pOptJacobian[rlNumEquations][] to pOptJacobian[rlNumEquations+3][] rows
   SmBoolean              & rbFoundAnswer)        // out: TRUE = all rF values less than scaled m_dConversionTol
{
  // init outputs
  rbFoundAnswer = FALSE;

  // Always need to reset offsetSurface offset distances prior to solving
  GetSurface(0)->SetOffsetDistance(m_dFilletRadii[0]*m_dOrientations[0]);
  GetSurface(1)->SetOffsetDistance(m_dFilletRadii[1]*m_dOrientations[1]);

  // get lSurf1Off index into crX array for lSurf1 parameters
  ULONG lSurf1Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 0 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() == 
      GetSurface(0)->GetBaseSurface())) {
      lSurf1Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 1 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      GetSurface(0)->GetBaseSurface())) {
      lSurf1Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }

  // get baseSurface1 F2 point values for given crX values
  SmPoint3d sF2;
  SmVector3d sDUF2, sDVF2;
  SmPoint2d sUV(crX[lSurf1Off],crX[lSurf1Off+1]);
  GetSurface(0)->SetOffsetDistance(0.0);
  SER(GetSurface(0)->Evaluate1stDerivatives(sUV,TRUE,TRUE,sF2,sDUF2,sDVF2));

  // get baseSurface1 surface normal and check its size
  SmVector3d sN = sDUF2 * sDVF2;
  if (sN.LengthSquared() < SM_EFF_ZERO_SQ) 
    { SER(SM_ERR); }
  SER(sN.Unitize());
  sN = sN*m_dOrientations[0];

  // get offsetSurface1 F point values for given crX
  SmPoint3d sF;
  SmVector3d sDUF, sDVF;
  GetSurface(0)->SetOffsetDistance(m_dFilletRadii[0]*m_dOrientations[0]);
  SER(GetSurface(0)->Evaluate1stDerivatives(sUV,TRUE,TRUE,sF,sDUF,sDVF));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,1,0);
      GetSurface(0)->DrawAt(sUV,1);
      sm_GraphicsLoop();
  }
#endif

  // get lSurf2Off index into crX array for lSurf2 parameters
  ULONG lSurf2Off = rlNumParameters;
  rlNumParameters = rlNumParameters + 2;
  if (lRailIndex == 1 && (rpSurface1 != NULL &&
      ((SmOffsetSurface*)rpSurface1)->GetBaseSurface() ==
      GetSurface(1)->GetBaseSurface())) {
      lSurf2Off = rlSurf1Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  if (lRailIndex == 0 && (rpSurface2 != NULL &&
      ((SmOffsetSurface*)rpSurface2)->GetBaseSurface() ==
      GetSurface(1)->GetBaseSurface())) {
      lSurf2Off = rlSurf2Offset;
      rlNumParameters = rlNumParameters - 2;
  }
  
  // get baseSurface2 G2 point values for given crX values
  SmPoint3d sG2;
  SmVector3d sDUG2, sDVG2;
  SmPoint2d sUV2(crX[lSurf2Off],crX[lSurf2Off+1]);
  GetSurface(1)->SetOffsetDistance(0.0);
  SER(GetSurface(1)->Evaluate1stDerivatives(sUV2,TRUE,TRUE,sG2,sDUG2,sDVG2));

  // get baseSurface surface normal value
  SmVector3d sN2 = sDUG2 * sDVG2;
  if (sN2.LengthSquared() < SM_EFF_ZERO_SQ) { SER(SM_ERR); }
  SER(sN2.Unitize());
  sN2 = sN2*m_dOrientations[1];

  // restore offsetSurface2 offset distance
  GetSurface(1)->SetOffsetDistance(m_dFilletRadii[1]*m_dOrientations[1]);
#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      GetSurface(1)->DrawAt(sUV2,1);
      sm_GraphicsLoop();
  }
#endif

  // check surface tangent magnitudes 
  double sDUFLenSq = sDUF.LengthSquared();
  double sDVFLenSq = sDVF.LengthSquared();

  double sDUG2LenSq = sDUG2.LengthSquared();
  double sDVG2LenSq = sDVG2.LengthSquared();

  // failure: zero length tangents
  if (   sDUFLenSq  < SM_EFF_ZERO_SQ 
      || sDVFLenSq  < SM_EFF_ZERO_SQ
      || sDUG2LenSq < SM_EFF_ZERO_SQ 
      || sDVG2LenSq < SM_EFF_ZERO_SQ) 
    { return SM_ERR; }

  // set sDiff = distance between OffsetSurface1 and baseSurface2
  SmVector3d sDiff = sF - sG2;

  // set sN3 = crossProduct(BaseSurface1Normal, BaseSurface2Normal) 
  SmVector3d sN3   = sN*sN2;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(1,0,0);
      sDiff.Draw(&sG2);
      sm_GraphicsLoop();
  }
#endif

  // Intersection simply tries to zero the following three equations.
  // It is essentially intersecting the three surfaces by intersecting
  // tangent planes. Let V2 = f-g2
  // fun[i+0] = V2.V2 - rad*rad    - Distance between f and g2
  // fun[i+1] = V2.N2 - rad*cos(secant-angle)
  // fun[i+2] = V2.N3              - V2 is perpendicular to N3(=N1xN2)
  // 
  // Where f is offset-surface1, g2 is base-surface2
  //
  // It produces the following Jacobian
  //  | 2*V2.duf   2*V2.dvf   -2*V2.dug2   -2*V2.dvg2 |
  //  | duf.sN2    dvf.sN2    -dug2.sN2    -dvg2.sN2  |
  //  | duf.sN3    dvf.sN3    -dug2.sN3    -dvg2.sN3  |

  
  if (pOptJacobian) 
    {
      ULONG i = rlNumEquations;
      (*pOptJacobian)[i][lSurf1Off]   =  2.0 * sDiff.Dot(sDUF);
      (*pOptJacobian)[i][lSurf1Off+1] =  2.0 * sDiff.Dot(sDVF);
      (*pOptJacobian)[i][lSurf2Off]   = -2.0 * sDiff.Dot(sDUG2);
      (*pOptJacobian)[i][lSurf2Off+1] = -2.0 * sDiff.Dot(sDVG2);

      i++; // Move to next equation;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.Dot(sN2);
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.Dot(sN2);
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG2.Dot(sN2);
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG2.Dot(sN2);

      i++;
      (*pOptJacobian)[i][lSurf1Off]   =   sDUF.Dot(sN3);
      (*pOptJacobian)[i][lSurf1Off+1] =   sDVF.Dot(sN3);
      (*pOptJacobian)[i][lSurf2Off]   = - sDUG2.Dot(sN3);
      (*pOptJacobian)[i][lSurf2Off+1] = - sDVG2.Dot(sN3);
    }
  
  // Compute function values
  rF[rlNumEquations++] = sDiff.Dot(sDiff) - m_dFilletRadii[0]*m_dFilletRadii[1];
  rF[rlNumEquations++] = sDiff.Dot(sN2)   - m_dFilletRadii[1]*smos_Cosine(m_dSecantAngle);
  rF[rlNumEquations++] = sDiff.Dot(sN3);

  // See if we have converged
  double dScaledTol = GetConversionTol() * (1.0 + sF.GetMaxDimension());
  if (   smos_Fabs(sDiff.x) < dScaledTol
      && smos_Fabs(sDiff.y) < dScaledTol
      && smos_Fabs(sDiff.z) < dScaledTol) 
    {
      rbFoundAnswer = TRUE;
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) {
      if (pOptJacobian) pOptJacobian->Dump();
      smos_WriteBuffer(_T(" F --- \n"));
      rF.Dump();
  }
#endif

  // all done
  return SM_SUCCESS;

} // end SmSecantConstantRadiusFS::LoadJacobian



