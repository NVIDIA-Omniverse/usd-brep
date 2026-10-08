// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- sm_api.cpp
* PURPOSE --- Implementation for sm_api.h unit test lists
*
* 
* REVISION HISTORY:
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include "SM_API_test.h" 
#include "SmApiGeneral.h"
#include "SmApiIntersectors.h"
#include "SmApiQueries.h"

/***********************************************************************
PURPOSE ---  run all sms unit test that we want included in prog_test.  

// in : TRUE = display graphics within regressions
//      FALSE= don't

RETURNS --- 0 = all unit tests succeeded
            1 = one or more unit tests failed.
***********************************************************************/
SmStatus run_sm_api(SmBoolean bDoGraphics)
{
  int       iCnt    = 0 ; 
  TCHAR sBuff[SM_TBLOCK_SIZE];

  SmStatus stat;
  ULONG nSuccessFul = 0, nFailed = 0;

  // Set global settings
  smSet_DoGraphics(bDoGraphics);
  smSet_OutputLong(FALSE);
  smSet_OutputThin(FALSE);
  smSet_OutputDebugLog(FALSE);
  smSet_OutputApiLog(FALSE);

  

  MYPRINTF(_T("\n\n********** API TESTS ************\n"));

  MYPRINTF(_T("\n\n\n\n*************** Start TestSmBreps ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmBoolean ************\n"));

  stat = TestSmBoolean();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBoolean Successful ************\n"), NULL);
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBoolean NOT successful ************\n"), NULL);
  }

  MYPRINTF(_T("\n\n******************** Start TestSmBooleanBoundsExample ************\n"));
  stat = TestSmBooleanBoundsExample();
  iCnt++;
  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBooleanBoundsExample Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBooleanBoundsExample NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmMassProperties ************\n"));

  stat = TestSmMassProperties();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMassProperties Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMassProperties NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmPrecisePropertySelection ************\n"));
  stat = TestSmPrecisePropertySelection();
  iCnt++;
  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPrecisePropertySelection Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPrecisePropertySelection NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmBrepDistance ************\n"));

  stat = TestSmBrepDistance();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBrepDistance Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBrepDistance NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmMaterialCensus ************\n"));

  stat = TestSmMaterialCensus();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMaterialCensus Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMaterialCensus NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmMergeBreps   ************\n"));

  stat = TestSmMergeBreps();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMergeBreps Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMergeBreps NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmTessellate ************\n"));

  stat = TestSmTessellate();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTessellate Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTessellate NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmIntersectBrepWithPlane ************\n"));

  stat = TestSmIntersectBrepWithPlane();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectBrepWithPlane Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectBrepWithPlane NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmProjectBrepOntoPlane ************\n"));

  stat = TestSmProjectBrepOntoPlane();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectBrepOntoPlane Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectBrepOntoPlane NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSilhouetteCurves ************\n"));

  stat = TestSmCreateSilhouetteCurves();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSilhouetteCurves Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSilhouetteCurves NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCut ************\n"));

  stat = TestSmCut();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCut Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCut NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmProjectAndTrim ************\n"));

  stat = TestSmProjectAndTrim();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectAndTrim Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectAndTrim NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmProjectCurve ************\n"));

  stat = TestSmProjectCurve();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectCurve Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectCurve NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCurveSweep ************\n"));
  stat = TestSmCurveSweep();
  iCnt++;
  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCurveSweep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCurveSweep NOT successful ************\n"));
  }

  // MYPRINTF(_T("\n\n******************** Start TestSmCurveSweepFromFaces ************\n"));
  // stat = TestSmCurveSweepFromFaces();
  // iCnt++;
  // if (stat == SM_SUCCESS) {
  //     nSuccessFul++;
  //     smos_WriteBuffer(_T("\n\n******************** End TestSmCurveSweepFromFaces Successful ************\n"));
  // }
  // else {
  //     nFailed++;
  //     smos_WriteBuffer(_T("\n\n******************** End TestSmCurveSweepFromFaces NOT successful ************\n"));
  // }

  MYPRINTF(_T("\n\n******************** Start TestSmTaperExtrude ************\n"));

  stat = TestSmTaperExtrude();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTaperExtrude Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTaperExtrude NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmNonManifoldSweep ************\n"));

  stat = TestSmNonManifoldSweep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmNonManifoldSweep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmNonManifoldSweep NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmNonManifoldRotationalSweep ************\n"));

  stat = TestSmNonManifoldRotationalSweep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmNonManifoldRotationalSweep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmNonManifoldRotationalSweep NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmBooleanWithOptions ************\n"));

  stat = TestSmBooleanWithOptions();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBooleanWithOptions Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBooleanWithOptions NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmNonManifoldBoolean ************\n"));

  stat = TestSmNonManifoldBoolean();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmNonManifoldBoolean Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmNonManifoldBoolean NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmDeleteFaceKeepsInfiniteRegion ************\n"));

  stat = TestSmDeleteFaceKeepsInfiniteRegion();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmDeleteFaceKeepsInfiniteRegion Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmDeleteFaceKeepsInfiniteRegion NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmPiecewiseMerge ************\n"));

  stat = TestSmPiecewiseMerge();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPiecewiseMerge Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPiecewiseMerge NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmBooleanLists ************\n"));

  stat = TestSmBooleanLists();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBooleanLists Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBooleanLists NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmShellBrepFull ************\n"));

  stat = TestSmShellBrepFull();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmShellBrepFull Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmShellBrepFull NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmOffsetBrepFull ************\n"));

  stat = TestSmOffsetBrepFull();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmOffsetBrepFull Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmOffsetBrepFull NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmStitchIntoSolid ************\n"));

  stat = TestSmStitchIntoSolid();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmStitchIntoSolid Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmStitchIntoSolid NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmStitchIntoShell ************\n"));

  stat = TestSmStitchIntoShell();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmStitchIntoShell Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmStitchIntoShell NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmUnifyNormals ************\n"));

  stat = TestSmUnifyNormals();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmUnifyNormals Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmUnifyNormals NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmAdvancedStitch ************\n"));

  stat = TestSmAdvancedStitch();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmAdvancedStitch Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmAdvancedStitch NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n\n\n*************** Start TestSmCurves ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmCreateLineSegment ************\n"));

  stat = TestSmCreateLineSegment();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLineSegment Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLineSegment NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateCircle ************\n"));

  stat = TestSmCreateCircle();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCircle Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCircle NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmInvalidInputCodes ************\n"));

  stat = TestSmInvalidInputCodes();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmInvalidInputCodes Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmInvalidInputCodes NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateArc ************\n"));

  stat = TestSmCreateArc();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateArc Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateArc NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateHelixInvalidInputs ************\n"));

  stat = TestSmCreateHelixInvalidInputs();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateHelixInvalidInputs Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateHelixInvalidInputs NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateCurveInvalidInputs ************\n"));

  stat = TestSmCreateCurveInvalidInputs();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCurveInvalidInputs Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCurveInvalidInputs NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateCurve ************\n"));

  stat = TestSmCreateCurve();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCurve Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCurve NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateCanonicalCurve ************\n"));

  stat = TestSmCreateCanonicalCurve();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCanonicalCurve Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCanonicalCurve NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmOffsetCurve ************\n"));

  stat = TestSmOffsetCurve();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmOffsetCurve Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmOffsetCurve NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmDropCurveToSrf ************\n"));

  stat = TestSmDropCurveToSrf();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmDropCurveToSrf Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmDropCurveToSrf NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateRectangle ************\n"));

  stat = TestSmCreateRectangle();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRectangle Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRectangle NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateRegularPolygon ************\n"));

  stat = TestSmCreateRegularPolygon();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRegularPolygon Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRegularPolygon NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmProjectCurveToSurface ************\n"));

  stat = TestSmProjectCurveToSurface();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectCurveToSurface Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmProjectCurveToSurface NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmLiftUVCurve ************\n"));

  stat = TestSmLiftUVCurve();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmLiftUVCurve Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmLiftUVCurve NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmMakeCurvesCompatible ************\n"));

  stat = TestSmMakeCurvesCompatible();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMakeCurvesCompatible Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmMakeCurvesCompatible NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmOrderCurves ************\n"));

  stat = TestSmOrderCurves();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmOrderCurves Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmOrderCurves NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmRemoveCurveKnots ************\n"));

  stat = TestSmRemoveCurveKnots();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmRemoveCurveKnots Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmRemoveCurveKnots NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmFillets ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmCircularFillet ************\n"));

  stat = TestSmCircularFillet();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCircularFillet Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCircularFillet NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmChamferFillet ************\n"));

  stat = TestSmChamferFillet();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmChamferFillet Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmChamferFillet NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmFilletEdges ************\n"));

  stat = TestSmFilletEdges();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFilletEdges Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFilletEdges NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmFilletEdgesPerEdge ************\n"));

  stat = TestSmFilletEdgesPerEdge();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFilletEdgesPerEdge Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFilletEdgesPerEdge NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmVariableRadiusFillet ************\n"));

  stat = TestSmVariableRadiusFillet();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmVariableRadiusFillet Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmVariableRadiusFillet NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmRemoveFillet ************\n"));

  stat = TestSmRemoveFillet();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmRemoveFillet Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmRemoveFillet NOT successful ************\n"));
  }

  

  MYPRINTF(_T("\n\n******************** Start TestSmSurfaceSurfaceFillet ************\n"));

  stat = TestSmSurfaceSurfaceFillet();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSurfaceSurfaceFillet Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSurfaceSurfaceFillet NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmFilletPreview ************\n"));

  stat = TestSmFilletPreview();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFilletPreview Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFilletPreview NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmSetBevelCorners ************\n"));

  stat = TestSmSetBevelCorners();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSetBevelCorners Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSetBevelCorners NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n\n\n*************** Start TestSmGeneral ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmTransform ************\n"));

  stat = TestSmTransform();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTransform Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTransform NOT successful ************\n"));
  }

   MYPRINTF(_T("\n\n******************** Start TestSmScale ************\n"));

  stat = TestSmScale();
   iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmScale Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmScale NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmPolyBrepMassPropertiesWinding ************\n"));

  stat = TestSmPolyBrepMassPropertiesWinding();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPolyBrepMassPropertiesWinding Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPolyBrepMassPropertiesWinding NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmPrincipalDirectionPairing ************\n"));

  stat = TestSmPrincipalDirectionPairing();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPrincipalDirectionPairing Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPrincipalDirectionPairing NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmConeSTEPCrossDerivative ************\n"));

  stat = TestSmConeSTEPCrossDerivative();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmConeSTEPCrossDerivative Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmConeSTEPCrossDerivative NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmBrepCopyRepresentation ************\n"));
  stat = TestSmBrepCopyRepresentation();
  iCnt++;
  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBrepCopyRepresentation Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmBrepCopyRepresentation NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmSTEPDerivativeUnits ************\n"));

  stat = TestSmSTEPDerivativeUnits();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSTEPDerivativeUnits Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSTEPDerivativeUnits NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmSTEPDerivativesExample ************\n"));
  stat = TestSmSTEPDerivativesExample();
  iCnt++;
  if( stat == SM_SUCCESS )
  {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSTEPDerivativesExample Successful ************\n"));
  }
  else
  {
      smos_WriteBuffer(_T("\n\n******************** End TestSmSTEPDerivativesExample NOT successful ************\n"));
      nFailed++;
  }

  MYPRINTF(_T("\n\n******************** Start TestSmPlanarFacesFailureCleanup ************\n"));

  stat = TestSmPlanarFacesFailureCleanup();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPlanarFacesFailureCleanup Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmPlanarFacesFailureCleanup NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmEdgeCurveOwnership ************\n"));

  stat = TestSmEdgeCurveOwnership();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** TestSmEdgeCurveOwnership Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** TestSmEdgeCurveOwnership NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCurveSweepFailureCleanup ************\n"));

  stat = TestSmCurveSweepFailureCleanup();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCurveSweepFailureCleanup Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCurveSweepFailureCleanup NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmRotate ************\n"));

  stat = TestSmRotate();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmRotate Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmRotate NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmStatusAssertMacros ************\n"));

  stat = TestSmStatusAssertMacros();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmStatusAssertMacros Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmStatusAssertMacros NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmTranslate ************ \n"));

  stat = TestSmTranslate();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTranslate Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTranslate NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n\n\n*************** Start TestSmHeal ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmHealBrep ************\n"));

  stat = TestSmHealBrep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmHealBrep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmHealBrep NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmImportExport ************\n"));

  stat = TestSmImportExport();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmImportExport Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmImportExport NOT successful ************\n"));
  }

  // TestSmIntersectors

  MYPRINTF(_T("\n\n******************** Start TestSmIntersectCurves ************\n"));

  stat = TestSmIntersectCurves();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurves Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurves NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmIntersectSurfaces ************\n"));

  stat = TestSmIntersectSurfaces();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectSurfaces Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectSurfaces NOT successful ************\n"));
  }


  MYPRINTF(_T("\n\n******************** Start TestSmIntersectBrepWithPlane ************\n"));

  stat = TestSmIntersectBrepWithPlane();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectBrepWithPlane Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectBrepWithPlane NOT successful ************\n"));
  }


   MYPRINTF(_T("\n\n******************** Start TestSmIntersectBreps ************\n"));

  stat = TestSmIntersectBreps();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectBreps Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectBreps NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmIntersectCurveSurface ************\n"));

  stat = TestSmIntersectCurveSurface();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurveSurface Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurveSurface NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmIntersectCurveFace ************\n"));

  stat = TestSmIntersectCurveFace();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurveFace Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurveFace NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmIntersectCurveBrep ************\n"));

  stat = TestSmIntersectCurveBrep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurveBrep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmIntersectCurveBrep NOT successful ************\n"));
  }

  // TestSmPolygons


  // TestSmPrimitives
  MYPRINTF(_T("\n\n\n\n*************** Start TestSmPrimitives ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmCreateBox ************\n"));

  stat = TestSmCreateBox();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateBox Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateBox NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSphere ************\n"));

  stat = TestSmCreateSphere();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSphere Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSphere NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateCone ************\n"));

  stat = TestSmCreateCone();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCone Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateCone NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateTorus ************\n"));

  stat = TestSmCreateTorus();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateTorus Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateTorus NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreatePlane ************\n"));

  stat = TestSmCreatePlane();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreatePlane Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreatePlane NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreatePlanarCircle ************\n"));

  stat = TestSmCreatePlanarCircle();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreatePlanarCircle Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreatePlanarCircle NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateRotationalSweep ************\n"));

  stat = TestSmCreateRotationalSweep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRotationalSweep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRotationalSweep NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateLinearSweep ************\n"));

  stat = TestSmCreateLinearSweep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLinearSweep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLinearSweep NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateLinearSweepPlane ************\n"));

  stat = TestSmCreateLinearSweepPlane();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLinearSweepPlane Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLinearSweepPlane NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateLinearSweepCylinder ************\n"));

  stat = TestSmCreateLinearSweepCylinder();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLinearSweepCylinder Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateLinearSweepCylinder NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateDraftSweep ************\n"));


  stat = TestSmCreateDraftSweep();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateDraftSweep Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateDraftSweep NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSwungPrimitive ************\n"));

  stat = TestSmCreateSwungPrimitive();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSwungPrimitive Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSwungPrimitive NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSkinPrimitive ************\n"));

  stat = TestSmCreateSkinPrimitive();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSkinPrimitive Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSkinPrimitive NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSkinFromFaces ************\n"));

  stat = TestSmCreateSkinFromFaces();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSkinFromFaces Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSkinFromFaces NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n\n\n*************** Start TestSmSurfaces ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurface ************\n"));

  stat = TestSmCreateSurface();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurface Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurface NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceFromPoints ************\n"));

  stat = TestSmCreateSurfaceFromPoints();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceFromPoints Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceFromPoints NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceFromRandomPoints ************\n"));

  stat = TestSmCreateSurfaceFromRandomPoints();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceFromRandomPoints Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceFromRandomPoints NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceFromCornerPoints ************\n"));

  stat = TestSmCreateSurfaceFromCornerPoints();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceFromCornerPoints Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceFromCornerPoints NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateExtrude ************\n"));

  stat = TestSmCreateExtrude();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateExtrude Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateExtrude NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateRuledSurface ************\n"));

  stat = TestSmCreateRuledSurface();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRuledSurface Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateRuledSurface NOT successful ************\n"));
  }

    MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceRevolution ************\n"));

  stat = TestSmCreateSurfaceRevolution();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolution Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolution NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceRevolutionSphere ************\n"));

  stat = TestSmCreateSurfaceRevolutionSphere();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionSphere Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionSphere NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceRevolutionTorus ************\n"));

  stat = TestSmCreateSurfaceRevolutionTorus();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionTorus Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionTorus NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceRevolutionCylinder ************\n"));

  stat = TestSmCreateSurfaceRevolutionCylinder();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionCylinder Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionCylinder NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceRevolutionCone ************\n"));

  stat = TestSmCreateSurfaceRevolutionCone();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionCone Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionCone NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSurfaceRevolutionTruncCone ************\n"));

  stat = TestSmCreateSurfaceRevolutionTruncCone();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionTruncCone Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSurfaceRevolutionTruncCone NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateOffsetSurface ************\n"));

  stat = TestSmCreateOffsetSurface();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateOffsetSurface Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateOffsetSurface NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreateSkinSurface ************\n"));

  stat = TestSmCreateSkinSurface();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSkinSurface Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreateSkinSurface NOT successful ************\n"));
  }

  // TestSmTrimmedSurface
  MYPRINTF(_T("\n\n\n\n*************** Start TestSmTrimmedSurfaces ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmTrimSurfaceWith3dCurves ************\n"));

  stat = TestSmTrimSurfaceWith3dCurves();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTrimSurfaceWith3dCurves Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTrimSurfaceWith3dCurves NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmTrimProjectParallel ************\n"));

  stat = TestSmTrimProjectParallel();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTrimProjectParallel Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmTrimProjectParallel NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmCreatePlanarFaces ************\n"));

  stat = TestSmCreatePlanarFaces();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreatePlanarFaces Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmCreatePlanarFaces NOT successful ************\n"));
  }

 

  

  MYPRINTF(_T("\n\n\n\n*************** Start Sweep / Utility Tests ************\n"));
  MYPRINTF(_T("\n\n******************** Start TestSmSweepAlongPlanarPath ************\n"));

  stat = TestSmSweepAlongPlanarPath();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSweepAlongPlanarPath Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmSweepAlongPlanarPath NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmGetClosestPoint ************\n"));

  stat = TestSmGetClosestPoint();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmGetClosestPoint Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmGetClosestPoint NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmGetEdges ************\n"));

  stat = TestSmGetEdges();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmGetEdges Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmGetEdges NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmGetFaces ************\n"));

  stat = TestSmGetFaces();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmGetFaces Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmGetFaces NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmFindEdge ************\n"));

  stat = TestSmFindEdge();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFindEdge Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFindEdge NOT successful ************\n"));
  }

  MYPRINTF(_T("\n\n******************** Start TestSmFindFaces ************\n"));

  stat = TestSmFindFaces();
  iCnt++;

  if (stat == SM_SUCCESS) {
      nSuccessFul++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFindFaces Successful ************\n"));
  }
  else {
      nFailed++;
      smos_WriteBuffer(_T("\n\n******************** End TestSmFindFaces NOT successful ************\n"));
  }



  MYPRINTF(_T("\n\n*******************************************************************\n"));
  MYPRINTF(_T("*******************************************************************\n"));
  MYPRINTF(_T("*******************************************************************\n"));
  MYPRINTF(_T("***********************    END OF API-TEST    *********************\n"));
  MYPRINTF(_T("*******************************************************************\n"));
  MYPRINTF(_T("*******************************************************************\n"));
  MYPRINTF(_T("*******************************************************************\n\n\n"));

  //TCHAR sBuff[SM_TBLOCK_SIZE];
  SM_SPRINTF(sBuff, _T("%s"), _T("************************************************\n"));
  MYPRINTF(sBuff);
  SM_SPRINTF(sBuff, _T("%s"), _T("****************      RESULTS      *************\n"));
  MYPRINTF(sBuff);
  SM_SPRINTF(sBuff, _T("%s"), _T("************************************************\n"));
  MYPRINTF(sBuff);
  SM_SPRINTF(sBuff, _T("**************** %ld Successful    *************\n"), nSuccessFul);
  MYPRINTF(sBuff);
  SM_SPRINTF(sBuff, _T("**************** %ld Failed        *************\n"), nFailed);
  MYPRINTF(sBuff);


  // inform the public
  if(nFailed == 0) {
      SM_SPRINTF(sBuff,   _T("\n  API Test Final Result = Success: All %d API tests are still Working\n"), iCnt) ;
      smos_WriteBuffer(sBuff) ; 
  }
  else {
      SM_SPRINTF(sBuff,   _T("\n  API Test Final Result = Failure: [%lu of %d] API tests failed\n"), nFailed, iCnt) ;
      smos_WriteBuffer(sBuff) ;
  }

  if (nFailed) { MYPRINTF(_T("\n\n************************* End API Test with PROBLEMS ************\n")); }
  else { MYPRINTF(_T("\n\n************************* End API Test with SUCCESS ************\n")); }

  // all done
  return(nFailed == 0 ? 0 : 1) ;

} // end run_sms_unitz

#ifdef SM_API_TEST_STANDALONE

#include <csignal>
#include <cstdio>
#include <cstdlib>

static void CrashHandler(int sig)
{
    const char* name = (sig == SIGSEGV) ? "SIGSEGV" :
                       (sig == SIGABRT) ? "SIGABRT" :
                       (sig == SIGFPE)  ? "SIGFPE"  : "UNKNOWN";
    fprintf(stderr, "\n*** CRASH: signal %s (%d) ***\n", name, sig);
    std::_Exit(sig + 128);
}

int main(int /*argc*/, char* /*argv*/[])
{
    std::signal(SIGSEGV, CrashHandler);
    std::signal(SIGABRT, CrashHandler);
    std::signal(SIGFPE,  CrashHandler);

    printf("\nSM_API Standalone Test Runner\n");
    printf("=============================\n\n");

    SmStatus result = run_sm_api(FALSE);

    printf("\n=============================\n");
    if (result == 0)
        printf("RESULT: PASS\n");
    else
        printf("RESULT: FAIL (code %ld)\n", result);

    return (int)result;
}

#endif // SM_API_TEST_STANDALONE
