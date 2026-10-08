// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- sm_api.h
* PURPOSE --- Header file sms api tests
*
* 
**********************************************************************/
/*___*/

#ifndef SMS_API_TEST_H
#define SMS_API_TEST_H       

#if defined(_WIN32)
#  define API_EXPORT __declspec( dllexport )
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define API_EXPORT __attribute__((visibility("default")))
#else
#  define API_EXPORT
#endif

// TestSmBreps
API_EXPORT SmStatus TestSmBoolean();
API_EXPORT SmStatus TestSmBooleanBoundsExample();
API_EXPORT SmStatus TestSmMassProperties();
API_EXPORT SmStatus TestSmPrecisePropertySelection();
API_EXPORT SmStatus TestSmBrepDistance();
API_EXPORT SmStatus TestSmMaterialCensus();
API_EXPORT SmStatus TestSmMergeBreps();
API_EXPORT SmStatus TestSmTessellate();
API_EXPORT SmStatus TestSmProjectBrepOntoPlane();
API_EXPORT SmStatus TestSmCut();
API_EXPORT SmStatus TestSmProjectAndTrim();
API_EXPORT SmStatus TestSmProjectCurve();
API_EXPORT SmStatus TestSmCurveSweep();
API_EXPORT SmStatus TestSmCurveSweepFromFaces();
API_EXPORT SmStatus TestSmTaperExtrude();
API_EXPORT SmStatus TestSmNonManifoldSweep();
API_EXPORT SmStatus TestSmNonManifoldRotationalSweep();
API_EXPORT SmStatus TestSmBooleanWithOptions();
API_EXPORT SmStatus TestSmNonManifoldBoolean();
API_EXPORT SmStatus TestSmDeleteFaceKeepsInfiniteRegion();
API_EXPORT SmStatus TestSmPiecewiseMerge();
API_EXPORT SmStatus TestSmBooleanLists();
API_EXPORT SmStatus TestSmShellBrepFull();
API_EXPORT SmStatus TestSmOffsetBrepFull();
API_EXPORT SmStatus TestSmStitchIntoSolid();
API_EXPORT SmStatus TestSmStitchIntoShell();
API_EXPORT SmStatus TestSmUnifyNormals();
API_EXPORT SmStatus TestSmAdvancedStitch();
API_EXPORT SmStatus TestSmSweepAlongPlanarPath();
API_EXPORT SmStatus TestSmCreateSilhouetteCurves();

// TestSmErrorCodes
API_EXPORT SmStatus TestSmInvalidInputCodes();

// TestSmCurves
API_EXPORT SmStatus TestSmCreateLineSegment();
API_EXPORT SmStatus TestSmCreateCircle();
API_EXPORT SmStatus TestSmCreateArc();
API_EXPORT SmStatus TestSmCreateCurve();
API_EXPORT SmStatus TestSmCreateCanonicalCurve();
API_EXPORT SmStatus TestSmOffsetCurve();
API_EXPORT SmStatus TestSmDropCurveToSrf();
API_EXPORT SmStatus TestSmCreateRectangle();
API_EXPORT SmStatus TestSmCreateRegularPolygon();
API_EXPORT SmStatus TestSmProjectCurveToSurface();
API_EXPORT SmStatus TestSmLiftUVCurve();
API_EXPORT SmStatus TestSmMakeCurvesCompatible();
API_EXPORT SmStatus TestSmOrderCurves();
API_EXPORT SmStatus TestSmRemoveCurveKnots();
API_EXPORT SmStatus TestSmCreateHelixInvalidInputs();
API_EXPORT SmStatus TestSmCreateCurveInvalidInputs();

// TestSmFillets
API_EXPORT SmStatus TestSmCircularFillet();
API_EXPORT SmStatus TestSmChamferFillet();
API_EXPORT SmStatus TestSmFilletEdges();
API_EXPORT SmStatus TestSmFilletEdgesPerEdge();
API_EXPORT SmStatus TestSmVariableRadiusFillet();
API_EXPORT SmStatus TestSmSurfaceSurfaceFillet();
API_EXPORT SmStatus TestSmFilletPreview();
API_EXPORT SmStatus TestSmSetBevelCorners();
API_EXPORT SmStatus TestSmRemoveFillet();


// TestSmGeneral
API_EXPORT SmStatus TestSmBrepCopyRepresentation();
API_EXPORT SmStatus TestSmTransform();
API_EXPORT SmStatus TestSmScale();
API_EXPORT SmStatus TestSmRotate();
API_EXPORT SmStatus TestSmStatusAssertMacros();
API_EXPORT SmStatus TestSmTranslate();
API_EXPORT SmStatus TestSmPolyBrepMassPropertiesWinding();
API_EXPORT SmStatus TestSmPrincipalDirectionPairing();
API_EXPORT SmStatus TestSmSTEPDerivativeUnits();
API_EXPORT SmStatus TestSmSTEPDerivativesExample();
API_EXPORT SmStatus TestSmConeSTEPCrossDerivative();
API_EXPORT SmStatus TestSmPlanarFacesFailureCleanup();
API_EXPORT SmStatus TestSmEdgeCurveOwnership();
API_EXPORT SmStatus TestSmCurveSweepFailureCleanup();


// TestSmHeal
API_EXPORT SmStatus TestSmHealBrep();

// TestSmImportExport
API_EXPORT SmStatus TestSmImportExport();

// TestSmIntersectors
API_EXPORT SmStatus TestSmIntersectCurves();
API_EXPORT SmStatus TestSmIntersectSurfaces();
API_EXPORT SmStatus TestSmIntersectBrepWithPlane();
API_EXPORT SmStatus TestSmIntersectBreps();
API_EXPORT SmStatus TestSmIntersectCurveSurface();
API_EXPORT SmStatus TestSmIntersectCurveFace();
API_EXPORT SmStatus TestSmIntersectCurveBrep();


// TestSmPolygons

// TestSmPrimitives
API_EXPORT SmStatus TestSmCreateSphere();
API_EXPORT SmStatus TestSmCreateBox();
API_EXPORT SmStatus TestSmCreateCone();
API_EXPORT SmStatus TestSmCreateTorus();
API_EXPORT SmStatus TestSmCreatePlane();
API_EXPORT SmStatus TestSmCreatePlanarCircle();
API_EXPORT SmStatus TestSmCreateRotationalSweep();
API_EXPORT SmStatus TestSmCreateLinearSweep();
API_EXPORT SmStatus TestSmCreateLinearSweepPlane();
API_EXPORT SmStatus TestSmCreateLinearSweepCylinder();
API_EXPORT SmStatus TestSmCreateDraftSweep();
API_EXPORT SmStatus TestSmCreateSwungPrimitive();
API_EXPORT SmStatus TestSmCreateSkinPrimitive();
API_EXPORT SmStatus TestSmCreateSkinFromFaces();


// TestSmSurfaces
API_EXPORT SmStatus TestSmCreateSurface();
API_EXPORT SmStatus TestSmCreateSurfaceFromPoints();
API_EXPORT SmStatus TestSmCreateSurfaceFromRandomPoints();
API_EXPORT SmStatus TestSmCreateSurfaceFromCornerPoints();
API_EXPORT SmStatus TestSmCreateExtrude();
API_EXPORT SmStatus TestSmCreateRuledSurface();
API_EXPORT SmStatus TestSmCreateSurfaceRevolution();
API_EXPORT SmStatus TestSmCreateSurfaceRevolutionSphere();
API_EXPORT SmStatus TestSmCreateSurfaceRevolutionTorus();
API_EXPORT SmStatus TestSmCreateSurfaceRevolutionCylinder();
API_EXPORT SmStatus TestSmCreateSurfaceRevolutionCone();
API_EXPORT SmStatus TestSmCreateSurfaceRevolutionTruncCone();
API_EXPORT SmStatus TestSmCreateOffsetSurface();
API_EXPORT SmStatus TestSmCreateSkinSurface();
 


// TestSmTrimmedSurfaces
API_EXPORT SmStatus TestSmTrimSurfaceWith3dCurves();
API_EXPORT SmStatus TestSmTrimProjectParallel();
API_EXPORT SmStatus TestSmCreatePlanarFaces();

// TestSmUtilities
API_EXPORT SmStatus TestSmGetClosestPoint();
API_EXPORT SmStatus TestSmGetEdges();
API_EXPORT SmStatus TestSmGetFaces();
API_EXPORT SmStatus TestSmFindEdge();
API_EXPORT SmStatus TestSmFindFaces();

API_EXPORT SmStatus run_sm_api  (SmBoolean bDoGraphics = FALSE);

#endif // SMS_API_TEST_H

