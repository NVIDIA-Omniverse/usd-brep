// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*******************************************************************
// Undistributed, user managed test file
// You should no longer need to modify any files under source 
// control to get testing flexibility
// The only thing that cannot change is the Test() function signature
//*******************************************************************

#include "StdAfx.h"

#include <SmApiGeneral.h>
#include <SmApiCurves.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmApiBrep.h>
#include <SmBSplineCurve.h>
#include <SmBrep.h>
#include <SmPlane.h>
#include <SmCurve.h>
#include <SmLine.h>
#include <SmPrimitiveCreation.h>
#include <SmSurface.h>

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSphere()
{

  SmApiCreateContext();

  SmVector3d sPositon( 0.0, 0.0, 0.0 );

  SmBrep* pResult = NULL;
  SmStatus stat = SmApiCreateSphere( sPositon, 10, pResult);
  if( stat != SM_SUCCESS || pResult == NULL )
    return SM_ERR;

  // A failing primitive must clear a stale output rather than leave it
  // untouched or hand back a half-built Brep, and must free what it allocated.
  SmApiStatus eStat;
  SmBrep* pOut;
  pOut = pResult; eStat = SmApiCreateSphere( sPositon, 0.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreateCone( sPositon, 1.0, 1.0, 0.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreateCylinder( sPositon, 0.0, 5.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreateTorus( sPositon, 5.0, 0.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreatePartialSphere( sPositon, 0.0, 0.0, 90.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreatePartialCone( sPositon, 1.0, 1.0, 0.0, 0.0, 90.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreatePartialTorus( sPositon, 5.0, 0.0, 0.0, 90.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreateCylindricalBox( sPositon, 5.0, 3.0, 1.0, 0.0, 90.0, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;
  pOut = pResult; eStat = SmApiCreateConeEllipticEnds( 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, TRUE, pOut );
  if( eStat == SM_SUCCESS || pOut != NULL ) return SM_ERR;

  // A zero radius is rejected up front with the output cleared (it used to
  // segfault SmApiCreateSphereNoPole). A negative radius stays valid: it is an
  // inside-out sphere with the same solid.
  pOut = pResult;
  if( SmApiCreateSphere( sPositon, 0.0, pOut ) != SM_ERR_INVALID_INPUT || pOut != NULL )
      return SM_ERR;
  pOut = pResult;
  if( SmApiCreateSphereNoPole( sPositon, 0.0, pOut ) != SM_ERR_INVALID_INPUT || pOut != NULL )
      return SM_ERR;
  SmBrep* pInsideOut = NULL;
  if( SmApiCreateSphereNoPole( sPositon, -5.0, pInsideOut ) != SM_SUCCESS || pInsideOut == NULL )
      return SM_ERR;
  delete pInsideOut;

  delete pResult;
  return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateBox()
{

    SmApiCreateContext();

    SmVector3d sPositon(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateBox(sPositon, 5, 10, 20, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // Zero or negative sizes are rejected (they used to succeed).
    SmBrep* pOut = pResult;
    if( SmApiCreateBox(sPositon, 0, 10, 20, pOut) != SM_ERR_INVALID_INPUT || pOut != NULL )
        return SM_ERR;
    pOut = pResult;
    if( SmApiCreateBox(sPositon, 5, -10, 20, pOut) != SM_ERR_INVALID_INPUT || pOut != NULL )
        return SM_ERR;

    // Inside radius above outside radius used to return status 0.
    pOut = pResult;
    if( SmApiCreateCylindricalBox(sPositon, 10, 5, 2, 0, 90, pOut) != SM_ERR_INVALID_INPUT || pOut != NULL )
        return SM_ERR;

    // Invalid angles are rejected too: start outside [-360, 360], end not after
    // start, and a sweep over 360 degrees.
    const double aBadAngles[][2] = { { 400.0, 450.0 }, { 90.0, 10.0 }, { 45.0, 45.0 }, { 0.0, 400.0 } };
    for( int ii = 0; ii < 4; ii++ )
    {
        pOut = pResult;
        if( SmApiCreateCylindricalBox(sPositon, 10, 2, 5, aBadAngles[ii][0], aBadAngles[ii][1], pOut) != SM_ERR_INVALID_INPUT
            || pOut != NULL )
            return SM_ERR;
    }

    delete pResult;
    return( SM_SUCCESS );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateCone()
{

    SmApiCreateContext();

    SmVector3d sPositon(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateCone(sPositon, 5, 10, 20, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // Degenerate cone and cylinder surfaces fail with the output cleared.
    SmSurface* pConeSrf = NULL;
    if( SmApiCreateCone(sPositon, 5, 10, 20, pConeSrf) != SM_SUCCESS || pConeSrf == NULL )
        return SM_ERR;
    SmSurface* pFailedCone = pConeSrf;
    SmSurface* pFailedCyl = pConeSrf;
    SmStatus coneStat = SmApiCreateCone(sPositon, 0, 0, 20, pFailedCone);
    SmStatus cylStat = SmApiCreateCylinder(sPositon, 0, 20, pFailedCyl);
    delete pConeSrf;
    if( coneStat == SM_SUCCESS || pFailedCone != NULL
        || cylStat == SM_SUCCESS || pFailedCyl != NULL )
        return SM_ERR;

    // Negative radii and non-positive heights are rejected with the output
    // cleared. A negative radius used to succeed; a negative height on the
    // cylinder surface overload used to hang in AdjustSTEPUVDomain.
    SmBrep* pOut = pResult;
    if( SmApiCreateCone(sPositon, -1, 10, 20, pOut) != SM_ERR_INVALID_INPUT || pOut != NULL )
        return SM_ERR;
    pOut = pResult;
    if( SmApiCreateCylinder(sPositon, -1, 20, pOut) != SM_ERR_INVALID_INPUT || pOut != NULL )
        return SM_ERR;

    SmSurface* pSurface = NULL;
    SER( SmApiCreateCylinder(sPositon, 5, 20, pSurface) );
    SmSurface* pOutSrf = pSurface;
    if( SmApiCreateCylinder(sPositon, 5, -20, pOutSrf) != SM_ERR_INVALID_INPUT || pOutSrf != NULL )
        return SM_ERR;
    pOutSrf = pSurface;
    if( SmApiCreateCone(sPositon, 5, 10, -20, pOutSrf) != SM_ERR_INVALID_INPUT || pOutSrf != NULL )
        return SM_ERR;

    delete pSurface;
    delete pResult;
    return( SM_SUCCESS );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateTorus()
{

    SmApiCreateContext();

    SmVector3d sPositon(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateTorus(sPositon, 10, 5, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreatePlane()
{

    SmApiCreateContext();

    SmVector3d sPositon(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreatePlane(sPositon, 10, 15, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // Degenerate dimensions fail with the output cleared, for both overloads.
    SmBrep* pFailedBrep = pResult;
    if( SmApiCreatePlane(sPositon, 0, 0, pFailedBrep) == SM_SUCCESS || pFailedBrep != NULL )
        return SM_ERR;

    SmPlane* pPlaneSrf = NULL;
    if( SmApiCreatePlane(sPositon, 10, 15, pPlaneSrf) != SM_SUCCESS || pPlaneSrf == NULL )
        return SM_ERR;
    SmPlane* pFailedSrf = pPlaneSrf;
    stat = SmApiCreatePlane(sPositon, 0, 0, pFailedSrf);
    delete pPlaneSrf;
    if( stat == SM_SUCCESS || pFailedSrf != NULL )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreatePlanarCircle()
{

    SmApiCreateContext();

    SmVector3d sPositon(0.0,0.0,0.0);
   
    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreatePlanarCircle(sPositon, 10, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    SmBrep* pFailed = pResult;
    if( SmApiCreatePlanarCircle(sPositon, 0, pFailed) == SM_SUCCESS || pFailed != NULL )
        return SM_ERR;

    return( SM_SUCCESS );
}

//*************************************************************************
// Result should be an SmSphere
//*************************************************************************

SmStatus TestSmCreateRotationalSweep()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0,0.0,0.0);
    
    SmBSplineCurve* pSemiCircle = NULL;
    SmApiCreateArc(sCenter, 10.0, 0.0, 180.0, pSemiCircle);

    SmVector3d sAxis( 1,0,0 );
    double dAngle = 360.0;

    SmTArray<SmCurve*> sCurvesToSweep;
    sCurvesToSweep.Add( pSemiCircle );

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateRotationalSweep(sCurvesToSweep, sCenter, sAxis, dAngle, 0, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // Failing sweeps must clear a stale output and free the Brep they built.
    SmTArray<SmPoint3d> sDegenPoints;
    for( int ii = 0; ii < 4; ii++ )
        sDegenPoints.Add( SmPoint3d(1.0, 1.0, 1.0) );
    SmBSplineCurve* pDegen = NULL;
    SER( SmApiCreateCurve(sDegenPoints, pDegen) );
    SmTArray<SmCurve*> sDegenCurves;
    sDegenCurves.Add( pDegen );

    SmBrep* pOut = pResult;
    SmVector3d sZAxis( 0,0,1 );
    SmApiStatus eStat = SmApiCreateRotationalSweep(sDegenCurves, sCenter, sZAxis, 90.0, 0, pOut);
    if( eStat == SM_SUCCESS || pOut != NULL )
        return SM_ERR;

    pOut = pResult;
    eStat = SmApiCreateLinearSweepWithRepetitions(sCurvesToSweep, sZAxis, 5.0, 0, TRUE, pOut);
    if( eStat == SM_SUCCESS || pOut != NULL )
        return SM_ERR;

    delete pDegen;
    delete pResult;
    return( SM_SUCCESS );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateLinearSweep()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0,0.0,0.0);
    
    SmBSplineCurve* pSemiCircle = NULL;
    SmApiCreateArc(sCenter, 10.0, 0.0, 180.0, pSemiCircle);

    SmVector3d sStartPt(-10.0,0.0,0.0);
    SmVector3d sEndPt(10.0,0.0,0.0);

    SmLine* pLine = NULL;
    SmApiCreateLineSegment( sStartPt, sEndPt, pLine);

    SmVector3d sDir( 0,0,1 );
    double dDist = 10.0;

    SmTArray<SmCurve*> sCurvesToSweep;
    sCurvesToSweep.Add( pSemiCircle );
    sCurvesToSweep.Add( pLine );

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateLinearSweep(sCurvesToSweep, sDir, dDist, 1, pResult);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateLinearSweepPlane()
{

    SmApiCreateContext();

    SmVector3d sStartPt(0.0,0.0,0.0);
    SmVector3d sEndPt(4.0,2.0,0.0);
    
    SmLine* pLine = NULL;
    SmApiCreateLineSegment( sStartPt, sEndPt, pLine);

    SmVector3d sDir( 0,0,1 );
    double dDist = 10.0;

    SmTArray<SmCurve*> sCurvesToSweep;
    sCurvesToSweep.Add( pLine );

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateLinearSweep(sCurvesToSweep, sDir, dDist, 0, pResult);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateLinearSweepCylinder()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0,0.0,0.0);
    
    SmBSplineCurve* pCircle = NULL;
    SmApiCreateArc(sCenter, 10.0, 0.0, 360.0, pCircle);

    SmVector3d sDir( 0,0,1 );
    double dDist = 10.0;

    SmTArray<SmCurve*> sCurvesToSweep;
    sCurvesToSweep.Add( pCircle );

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateLinearSweep(sCurvesToSweep, sDir, dDist, 1, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateDraftSweep()
{

    SmApiCreateContext();

    SmVector3d sCorner1(0.0,0.0,0.0);
    SmVector3d sCorner2(10.0,0.0,0.0);
    SmVector3d sCorner3(10.0,20.0,0.0);
    SmVector3d sCorner4(0.0,20.0,0.0);
    
    SmLine* pLine = NULL;
    SmApiCreateLineSegment( sCorner1, sCorner2, pLine);

    SmTArray<SmCurve*> sCurvesToSweep;
    sCurvesToSweep.Add( pLine );

    SmApiCreateLineSegment( sCorner2, sCorner3, pLine);
    sCurvesToSweep.Add( pLine );

    SmApiCreateLineSegment( sCorner3, sCorner4, pLine);
    sCurvesToSweep.Add( pLine );

    SmApiCreateLineSegment( sCorner4, sCorner1, pLine);
    sCurvesToSweep.Add( pLine );

    double dHeight = 10.0;
    double dAngle = -5.0;

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateDraftSweep(sCurvesToSweep, dHeight, dAngle, 1, 3, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSwungPrimitive()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0, 0.0, 0.0);

    SmBSplineCurve* pXYArc = NULL;
    SmApiCreateArc(sCenter, 5.0, 0.0, 180.0, pXYArc);

    SmBSplineCurve* pXZArc = NULL;
    SmApiCreateArc(sCenter, 8.0, 0.0, 180.0, pXZArc);

    if (pXYArc == NULL || pXZArc == NULL)
    {
        delete pXYArc;
        delete pXZArc;
        return SM_ERR_NULL_POINTER;
    }

    SmTArray<SmCurve*> sXYCurves;
    sXYCurves.Add(pXYArc);

    SmTArray<SmCurve*> sXZCurves;
    sXZCurves.Add(pXZArc);

    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateSwungPrimitive(sXYCurves, sXZCurves, 1.0, pResult);


    return(stat);
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSkinPrimitive()
{
    SmApiCreateContext();

    struct CurveSnapshot
    {
        SmExtent1d m_sInterval;
        SmPoint3d m_sMidPoint;
        double m_dLength;
    };

    auto SnapshotCurve = [](SmCurve* pCurve, CurveSnapshot& rSnapshot) -> SmStatus
    {
        if( pCurve == NULL )
            return SM_ERR_NULL_POINTER;
        rSnapshot.m_sInterval = pCurve->GetNaturalInterval();
        SER( pCurve->EvaluatePoint(rSnapshot.m_sInterval.Evaluate(0.5), rSnapshot.m_sMidPoint) );
        rSnapshot.m_dLength = pCurve->ApproximateLength(rSnapshot.m_sInterval, 20);
        return SM_SUCCESS;
    };

    auto CurveIsUnchanged = [&SnapshotCurve](SmCurve* pCurve, const CurveSnapshot& rExpected) -> SmBoolean
    {
        CurveSnapshot sActual;
        if( SnapshotCurve(pCurve, sActual) != SM_SUCCESS )
            return FALSE;
        return fabs(sActual.m_sInterval.GetMin() - rExpected.m_sInterval.GetMin()) < 1.0e-12 &&
               fabs(sActual.m_sInterval.GetMax() - rExpected.m_sInterval.GetMax()) < 1.0e-12 &&
               sActual.m_sMidPoint.DistanceBetween(rExpected.m_sMidPoint) < 1.0e-8 &&
               fabs(sActual.m_dLength - rExpected.m_dLength) < 1.0e-8;
    };

    SmVector3d sCenter1(0.0, 0.0, 0.0);
    SmVector3d sCenter2(0.0, 0.0, 5.0);
    SmVector3d sCenter3(0.0, 0.0, 10.0);

    SmBSplineCurve* pCircle1 = NULL;
    SER( SmApiCreateArc(sCenter1, 5.0, 0.0, 360.0, pCircle1) );
    SmObjDelete sCircle1Cleanup(pCircle1);

    SmBSplineCurve* pCircle2 = NULL;
    SER( SmApiCreateArc(sCenter2, 8.0, 0.0, 360.0, pCircle2) );
    SmObjDelete sCircle2Cleanup(pCircle2);

    SmBSplineCurve* pCircle3 = NULL;
    SER( SmApiCreateArc(sCenter3, 3.0, 0.0, 360.0, pCircle3) );
    SmObjDelete sCircle3Cleanup(pCircle3);

    CurveSnapshot sCircle1Snapshot, sCircle2Snapshot, sCircle3Snapshot;
    SER( SnapshotCurve(pCircle1, sCircle1Snapshot) );
    SER( SnapshotCurve(pCircle2, sCircle2Snapshot) );
    SER( SnapshotCurve(pCircle3, sCircle3Snapshot) );

    SmTArray<ULONG> sCurvesPerProfile;
    sCurvesPerProfile.Add(1);
    sCurvesPerProfile.Add(1);
    sCurvesPerProfile.Add(1);

    SmTArray<const SmCurve*> sAllCurves;
    sAllCurves.Add(pCircle1);
    sAllCurves.Add(pCircle2);
    sAllCurves.Add(pCircle3);

    SmBrep* pResult = NULL;
    SmObjDelete sResultCleanup;
    SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;
    SER( SmApiCreateSkinPrimitive(sCurvesPerProfile, sAllCurves, 3, TRUE, pResult,
                                  &sStartFaces, &sSideFaces, &sEndFaces) );
    sResultCleanup.SetObj(pResult);

    if( pResult == NULL || !pResult->IsManifoldSolid() ||
        sStartFaces.GetSize() != 1 || sSideFaces.GetSize() != 1 || sEndFaces.GetSize() != 1 ||
        sAllCurves.GetSize() != 3 || sAllCurves[0] != pCircle1 || sAllCurves[1] != pCircle2 ||
        sAllCurves[2] != pCircle3 ||
        !CurveIsUnchanged(pCircle1, sCircle1Snapshot) ||
        !CurveIsUnchanged(pCircle2, sCircle2Snapshot) ||
        !CurveIsUnchanged(pCircle3, sCircle3Snapshot) )
        return SM_ERR;

    // Linear skins may be split at intermediate profiles.  Those profile
    // boundaries guide the loft but must not be treated as end caps.
    SmBrep* pLinearResult = NULL;
    SmObjDelete sLinearResultCleanup;
    SmTArray<SmFace*> sLinearStartFaces, sLinearSideFaces, sLinearEndFaces;
    SER( SmApiCreateSkinPrimitive(sCurvesPerProfile, sAllCurves, 1, TRUE, pLinearResult,
                                  &sLinearStartFaces, &sLinearSideFaces, &sLinearEndFaces) );
    sLinearResultCleanup.SetObj(pLinearResult);

    double dLinearVolume = 0.0;
    SER( SmApiBrepComputeVolume(pLinearResult, 1.0e-6, dLinearVolume) );
    const double dExpectedLinearVolume = 1130.0 * SM_PI / 3.0;
    if( pLinearResult == NULL || !pLinearResult->IsManifoldSolid() ||
        sLinearStartFaces.GetSize() == 0 || sLinearSideFaces.GetSize() == 0 ||
        sLinearEndFaces.GetSize() == 0 ||
        fabs(dLinearVolume-dExpectedLinearVolume) > dExpectedLinearVolume*1.0e-4 )
        return SM_ERR;

    // Exercise a failure after the first closed profile has already been
    // accepted by the legacy kernel. Both caller-owned curves must survive.
    SmBSplineCurve* pFailureCircle = NULL;
    SER( SmApiCreateArc(sCenter1, 3.0, 0.0, 360.0, pFailureCircle) );
    SmObjDelete sFailureCircleCleanup(pFailureCircle);

    SmPoint3d sLineStart(0.0, 0.0, 10.0);
    SmPoint3d sLineEnd(2.0, 0.0, 10.0);
    SmLine* pFailureLine = NULL;
    SER( SmApiCreateLineSegment(sLineStart, sLineEnd, pFailureLine) );
    SmObjDelete sFailureLineCleanup(pFailureLine);

    CurveSnapshot sFailureCircleSnapshot, sFailureLineSnapshot;
    SER( SnapshotCurve(pFailureCircle, sFailureCircleSnapshot) );
    SER( SnapshotCurve(pFailureLine, sFailureLineSnapshot) );

    SmTArray<const SmCurve*> sFailureCurves;
    sFailureCurves.Add(pFailureCircle);
    sFailureCurves.Add(pFailureLine);
    SmTArray<ULONG> sFailureCurvesPerProfile;
    sFailureCurvesPerProfile.Add(1);
    sFailureCurvesPerProfile.Add(1);
    SmTArray<SmFace*> sFailureStartFaces, sFailureSideFaces, sFailureEndFaces;
    sFailureStartFaces.Add(sStartFaces[0]);
    sFailureSideFaces.Add(sSideFaces[0]);
    sFailureEndFaces.Add(sEndFaces[0]);

    SmBrep* pSentinel = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sSentinelCleanup(pSentinel);
    SmBrep* pFailureResult = pSentinel;
    SmStatus eFailureStat = SmApiCreateSkinPrimitive(sFailureCurvesPerProfile, sFailureCurves, 1, TRUE,
                                                     pFailureResult, &sFailureStartFaces,
                                                     &sFailureSideFaces, &sFailureEndFaces);

    if( eFailureStat == SM_SUCCESS || pFailureResult != NULL ||
        sFailureStartFaces.GetSize() != 0 || sFailureSideFaces.GetSize() != 0 ||
        sFailureEndFaces.GetSize() != 0 ||
        sFailureCurves[0] != pFailureCircle || sFailureCurves[1] != pFailureLine ||
        !CurveIsUnchanged(pFailureCircle, sFailureCircleSnapshot) ||
        !CurveIsUnchanged(pFailureLine, sFailureLineSnapshot) )
        return SM_ERR;

    // Matching concentric loops produce an annular solid.
    SmBSplineCurve* pInner1 = NULL;
    SmBSplineCurve* pInner2 = NULL;
    SmBSplineCurve* pInner3 = NULL;
    SER( SmApiCreateArc(sCenter1, 2.0, 0.0, 360.0, pInner1) );
    SmObjDelete sInner1Cleanup(pInner1);
    SER( SmApiCreateArc(sCenter2, 4.0, 0.0, 360.0, pInner2) );
    SmObjDelete sInner2Cleanup(pInner2);
    SER( SmApiCreateArc(sCenter3, 1.0, 0.0, 360.0, pInner3) );
    SmObjDelete sInner3Cleanup(pInner3);

    SmTArray<ULONG> sMultiLoopCurvesPerProfile;
    sMultiLoopCurvesPerProfile.Add(2);
    sMultiLoopCurvesPerProfile.Add(2);
    sMultiLoopCurvesPerProfile.Add(2);
    SmTArray<const SmCurve*> sMultiLoopCurves;
    sMultiLoopCurves.Add(pCircle1);
    sMultiLoopCurves.Add(pInner1);
    sMultiLoopCurves.Add(pCircle2);
    sMultiLoopCurves.Add(pInner2);
    sMultiLoopCurves.Add(pCircle3);
    sMultiLoopCurves.Add(pInner3);

    SmBrep* pMultiLoopResult = NULL;
    SmTArray<SmFace*> sMultiLoopStartFaces, sMultiLoopSideFaces, sMultiLoopEndFaces;
    SmStatus eMultiLoopStat = SmApiCreateSkinPrimitive(
        sMultiLoopCurvesPerProfile, sMultiLoopCurves, 1, TRUE, pMultiLoopResult,
        &sMultiLoopStartFaces, &sMultiLoopSideFaces, &sMultiLoopEndFaces);
    SmObjDelete sMultiLoopResultCleanup(pMultiLoopResult);

    double dMultiLoopVolume = 0.0;
    if( eMultiLoopStat == SM_SUCCESS )
        { SER( SmApiBrepComputeVolume(pMultiLoopResult, 1.0e-6, dMultiLoopVolume) ); }
    const double dExpectedMultiLoopVolume = 295.0 * SM_PI;
    if( eMultiLoopStat != SM_SUCCESS || pMultiLoopResult == NULL ||
        !pMultiLoopResult->IsManifoldSolid() ||
        sMultiLoopStartFaces.GetSize() == 0 || sMultiLoopSideFaces.GetSize() == 0 ||
        sMultiLoopEndFaces.GetSize() == 0 ||
        fabs(dMultiLoopVolume-dExpectedMultiLoopVolume) > dExpectedMultiLoopVolume*1.0e-4 )
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSkinFromFaces()
{

    SmApiCreateContext();

    SmVector3d sCenter1(0.0, 0.0, 0.0);
    SmVector3d sCenter2(0.0, 0.0, 10.0);

    SmBrep* pPlane1 = NULL;
    SmApiCreatePlanarCircle(sCenter1, 5.0, pPlane1);

    SmBrep* pPlane2 = NULL;
    SmApiCreatePlanarCircle(sCenter2, 8.0, pPlane2);

    if (pPlane1 == NULL || pPlane2 == NULL)
    {
        delete pPlane1;
        delete pPlane2;
        return SM_ERR_NULL_POINTER;
    }

    SmTArray<SmFace*> sFaces1;
    pPlane1->GetFaces(sFaces1);
    SmTArray<SmFace*> sFaces2;
    pPlane2->GetFaces(sFaces2);

    SmTArray<SmFace*> sInputFaces;
    if (sFaces1.GetSize() > 0) sInputFaces.Add(sFaces1[0]);
    if (sFaces2.GetSize() > 0) sInputFaces.Add(sFaces2[0]);

    SmTArray<SmFace*> sNewFaces;
    SmBrep* pResult = NULL;
    SmStatus stat = SmApiCreateSkinFromFaces(sInputFaces, 3, sNewFaces, pResult);


    return(stat);
}


//*************************************************************************
// 
//*************************************************************************


SmStatus TestSmPrimitives()
{
    SmStatus stat = TestSmCreateBox();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSphere();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateCone();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateTorus();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreatePlane();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreatePlanarCircle();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateRotationalSweep();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateDraftSweep();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateLinearSweep();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateLinearSweepPlane();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateLinearSweepCylinder();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSwungPrimitive();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSkinPrimitive();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSkinFromFaces();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}
