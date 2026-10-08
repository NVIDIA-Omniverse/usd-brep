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
#include <SmApiSurfaces.h> 
#include <SmApiPrimitives.h>
#include <SmApiIntersectors.h>
#include <SmApiBrep.h>
#include <SmApiFillets.h>
#include <SmApiQueries.h>
#include <SmCurve.h>

static SmBoolean ValidateCurveOnSurface( SmCurve* pCrv, SmSurface* pSrf,
                                         double dEps = 0.01 )
{
    SmExtent1d sIvl = pCrv->GetNaturalInterval();
    double dStart = sIvl.GetMin();
    double dEnd   = sIvl.GetMax();
    const int nSamples = 5;

    for( int ii = 0; ii <= nSamples; ii++ )
    {
        double t = dStart + (dEnd - dStart) * ii / nSamples;
        SmPoint3d sPt;
        if( pCrv->EvaluatePoint( t, sPt ) != SM_SUCCESS )
            return FALSE;

        SmPoint3d sClosest;
        SmTArray<SmPoint2d> sUVs;
        double dDist = 0;
        if( SmApiSurfaceClosestPoint( pSrf, sPt, sClosest, sUVs, dDist ) != SM_SUCCESS )
            return FALSE;
        if( sUVs.GetSize() < 1 || dDist > dEps )
            return FALSE;
    }
    return TRUE;
}

static SmBoolean ValidateCurveOnBrep( SmCurve* pCrv, SmBrep* pBrep,
                                      double dEps = 0.01 )
{
    SmExtent1d sIvl = pCrv->GetNaturalInterval();
    double dStart = sIvl.GetMin();
    double dEnd   = sIvl.GetMax();
    const int nSamples = 5;

    for( int ii = 0; ii <= nSamples; ii++ )
    {
        double t = dStart + (dEnd - dStart) * ii / nSamples;
        SmPoint3d sPt;
        if( pCrv->EvaluatePoint( t, sPt ) != SM_SUCCESS )
            return FALSE;

        SmPoint3d sClosest;
        double dDist = 0;
        if( SmApiGetClosestPoint( pBrep, sPt, sClosest, dDist ) != SM_SUCCESS )
            return FALSE;
        if( dDist > dEps )
            return FALSE;
    }
    return TRUE;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmIntersectCurves()
{

    SmApiCreateContext();

    SmTArray<SmPoint3d> sPoints1;
    SmVector3d sPoint1(0.0,0.0,0.0);
    sPoints1.Add( sPoint1 );
    SmVector3d sPoint2(10.0,-3.0,0.0);
    sPoints1.Add( sPoint2 );
    SmVector3d sPoint3(20.0,3.0,0.0);
    sPoints1.Add( sPoint3 );
    SmVector3d sPoint4(30.0,-6.0,0.0);
    sPoints1.Add( sPoint4 );
   
    SmBSplineCurve* pCurve1 = NULL;
    long stat = SmApiCreateCurve(sPoints1, pCurve1);

    SmTArray<SmPoint3d> sPoints2;
    SmVector3d sPoint5(10.0,-5.0,0.0);
    sPoints2.Add( sPoint5 );
    SmVector3d sPoint6(15.0,-3.0,0.0);
    sPoints2.Add( sPoint6 );
    SmVector3d sPoint7(10.0,3.0,0.0);
    sPoints2.Add( sPoint7 );
    SmVector3d sPoint8(20.0,6.0,0.0);
    sPoints2.Add( sPoint8 );
   
    SmBSplineCurve* pCurve2 = NULL;
    stat = SmApiCreateCurve(sPoints2, pCurve2);

    SmTArray<SmVector3d> intPoints;
    stat = SmApiIntersectCurves(pCurve1, pCurve2, &intPoints, NULL, NULL);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmIntersectSurfaces()
{

    SmApiCreateContext();

    SmTArray<SmPoint3d>rPointsList;
    rPointsList.Add(SmPoint3d(-9.99999999999999, -90.00000000000001, -40.00000000000000));       //   [0][0] =>> 0
    rPointsList.Add(SmPoint3d(-15.00993828742204, -90.00000000000000, -37.49503085628898));       //   [0][1] =>> 1
    rPointsList.Add(SmPoint3d(-20.89918488675400, -90.00000000000000, -29.46066919081585));       //   [0][2] =>> 2
    rPointsList.Add(SmPoint3d(-29.86166135600070, -90.00000000000000, -24.45419962835159));       //   [0][3] =>> 3
    rPointsList.Add(SmPoint3d(-41.04540877725864, -90.00000000000000, -21.58500943904096));       //   [0][4] =>> 4
    rPointsList.Add(SmPoint3d(-54.08153368920120, -90.00000000000000, -19.79899179084966));       //   [0][5] =>> 5
    rPointsList.Add(SmPoint3d(-62.62953406581700, -90.00000000000000, -23.21819194149598));       //   [0][6] =>> 6
    rPointsList.Add(SmPoint3d(-9.99999999999999, -20.00000000000001, -50.00000000000001));       //   [1][0] =>> 7
    rPointsList.Add(SmPoint3d(-14.17192594490645, -20.00000000000001, -47.91403702754678));       //   [1][1] =>> 8
    rPointsList.Add(SmPoint3d(-18.11902186410478, -19.99999999999999, -39.11280302897903));       //   [1][2] =>> 9
    rPointsList.Add(SmPoint3d(-25.35399362720083, -20.00000000000004, -33.42503955402192));       //   [1][3] =>> 10
    rPointsList.Add(SmPoint3d(-35.09449053271035, -20.00000000000002, -29.66201132684917));       //   [1][4] =>> 11
    rPointsList.Add(SmPoint3d(-47.19784042704145, -20.00000000000001, -26.67879014901961));       //   [1][5] =>> 12
    rPointsList.Add(SmPoint3d(-55.15544087898040, -20.00000000000001, -29.86183032979519));       //   [1][6] =>> 13
    rPointsList.Add(SmPoint3d(-9.99999999999999, -10.00000000000001, -20.00000000000001));       //   [2][0] =>> 14
    rPointsList.Add(SmPoint3d(-16.68596297245324, -10.00000000000001, -16.65701851377339));       //   [2][1] =>> 15
    rPointsList.Add(SmPoint3d(-26.45951093205240, -10.00000000000001, -10.15640151448953));       //   [2][2] =>> 16
    rPointsList.Add(SmPoint3d(-38.87699681360041, -10.00000000000001, -6.51251977701096));       //   [2][3] =>> 17
    rPointsList.Add(SmPoint3d(-52.94724526635518, -10.00000000000001, -5.43100566342460));       //   [2][4] =>> 18
    rPointsList.Add(SmPoint3d(-67.84892021352071, -10.00000000000001, -6.03939507450981));       //   [2][5] =>> 19
    rPointsList.Add(SmPoint3d(-77.57772043949019, -10.00000000000001, -9.93091516489760));       //   [2][6] =>> 20
    rPointsList.Add(SmPoint3d(-10.00000000000001, -39.99999999999999, 39.99999999999999));       //   [3][0] =>> 21
    rPointsList.Add(SmPoint3d(-21.71403702754681, -39.99999999999999, 45.85701851377338));       //   [3][1] =>> 22
    rPointsList.Add(SmPoint3d(-43.14048906794761, -39.99999999999999, 47.75640151448948));       //   [3][2] =>> 23
    rPointsList.Add(SmPoint3d(-65.92300318639960, -40.00000000000000, 47.31251977701093));       //   [3][3] =>> 24
    rPointsList.Add(SmPoint3d(-88.65275473364481, -40.00000000000000, 43.03100566342455));       //   [3][4] =>> 25
    rPointsList.Add(SmPoint3d(-109.15107978647926, -39.99999999999999, 35.23939507450980));       //   [3][5] =>> 26
    rPointsList.Add(SmPoint3d(-122.42227956050978, -39.99999999999999, 29.93091516489760));       //   [3][6] =>> 27
    rPointsList.Add(SmPoint3d(-10.00000000000000, -90.00000000000000, 10.00000000000000));       //   [4][0] =>> 28
    rPointsList.Add(SmPoint3d(-19.20000000000002, -90.00000000000000, 14.60000000000001));       //   [4][1] =>> 29
    rPointsList.Add(SmPoint3d(-34.80000000000001, -90.00000000000000, 18.80000000000000));       //   [4][2] =>> 30
    rPointsList.Add(SmPoint3d(-52.40000000000000, -90.00000000000000, 20.40000000000000));       //   [4][3] =>> 31
    rPointsList.Add(SmPoint3d(-70.80000000000000, -90.00000000000000, 18.80000000000000));       //   [4][4] =>> 32
    rPointsList.Add(SmPoint3d(-88.50000000000000, -90.00000000000000, 14.60000000000000));       //   [4][5] =>> 33
    rPointsList.Add(SmPoint3d(-99.99999999999999, -90.00000000000000, 10.00000000000001));       //   [4][6] =>> 34

    SmSurface* pSrf1 = NULL;
    SmStatus stat = SmApiCreateSurfaceFromOrderedPoints(rPointsList, 5, 7, pSrf1);
    if( stat != SM_SUCCESS || pSrf1 == NULL ) {
        MYPRINTF(_T("\nError in SmApiCreateSurfaceFromPoints"));
        return( stat );
    }

    SmLine* pLine = NULL;
    SmVector3d sLinePt1( -50,-40,0 );
    SmVector3d sLinePt2( -50,-40,100 );
    stat = SmApiCreateLineSegment( sLinePt1, sLinePt2, pLine);
    if( stat != SM_SUCCESS || pLine == NULL ) {
        MYPRINTF(_T("\nError in SmApiCreateLineSegment"));
        return( stat );
    }

    SmPoint3d sCenter( 0,0,0 );
    SmVector3d sAxis( 0,0,1 );

    SmSurface* pSrf2 = NULL;
    stat = SmApiCreateSurfaceRevolution(pLine, sCenter, sAxis, 360.0, pSrf2);
    if( stat != SM_SUCCESS || pSrf2 == NULL ) {
        MYPRINTF(_T("\nError in SmApiCreateSurfaceRevolution"));
        return( stat );
    }

    SmTArray<SmCurve*> p3DCurves;
    stat = SmApiIntersectSurfaces(pSrf1, pSrf2, &p3DCurves);
    if( stat != SM_SUCCESS || p3DCurves.GetSize() == 0 ) {
        MYPRINTF(_T("\nError in SmApiIntersectSurfaces"));
        return( stat );
    }
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmIntersectBrepWithPlane()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pBrep = NULL;
    SmApiCreateBox(sPositon1, 5, 10, 15, pBrep);

    SmApiCircularFillet(pBrep, 0.5);

    SmTArray<SmCurve*>sSectionCrvs;
    SmVector3d sPlanePt(0, 0, 7);
    SmVector3d sPlaneNrm(0, 0, 1);
    SmStatus stat = SmApiIntersectBrepWithPlane(pBrep, sPlanePt, sPlaneNrm, sSectionCrvs);
    if( stat != SM_SUCCESS )
        return( stat );

    // A non-unit normal gives the same section and is left as the caller passed it.
    SmVector3d sLongNrm(0, 0, 5);
    SmTArray<SmCurve*> sLongCrvs;
    if( SmApiIntersectBrepWithPlane(pBrep, sPlanePt, sLongNrm, sLongCrvs) != SM_SUCCESS
        || sLongCrvs.GetSize() != sSectionCrvs.GetSize()
        || sLongNrm.z != 5.0 )
        stat = SM_ERR;

    // A zero-length normal is rejected rather than silently treated as +Z.
    SmVector3d sZeroNrm(0, 0, 0);
    SmTArray<SmCurve*> sZeroCrvs;
    if( stat == SM_SUCCESS
        && ( SmApiIntersectBrepWithPlane(pBrep, sPlanePt, sZeroNrm, sZeroCrvs) != SM_ERR_INVALID_INPUT
             || sZeroCrvs.GetSize() != 0 ) )
        stat = SM_ERR;

    // Sections are appended: curves already in the caller's array are kept.
    SmTArray<SmCurve*> sAppended;
    SmPoint3d sKeptStart(0, 0, 0), sKeptEnd(1, 0, 0);
    SmLine* pKept = NULL;
    SmApiCreateLineSegment(sKeptStart, sKeptEnd, pKept);
    sAppended.Add(pKept);
    if( stat == SM_SUCCESS
        && ( SmApiIntersectBrepWithPlane(pBrep, sPlanePt, sPlaneNrm, sAppended) != SM_SUCCESS
             || sAppended.GetSize() != sSectionCrvs.GetSize() + 1
             || sAppended[0] != pKept ) )
        stat = SM_ERR;
    for( ULONG ii = 0; ii < sAppended.GetSize(); ii++ )
        delete sAppended[ii];

    if( stat == SM_SUCCESS
        && SmApiIntersectBrepWithPlane(NULL, sPlanePt, sPlaneNrm, sZeroCrvs) != SM_ERR_INVALID_INPUT )
        stat = SM_ERR;

    for( ULONG ii = 0; ii < sSectionCrvs.GetSize(); ii++ )
        delete sSectionCrvs[ii];
    for( ULONG ii = 0; ii < sLongCrvs.GetSize(); ii++ )
        delete sLongCrvs[ii];
    delete pBrep;

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmIntersectBreps()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pResult1 = NULL;
    SmStatus stat = SmApiCreateBox(sPositon1, 5, 10, 15, pResult1);

    SmVector3d sPositon2(3.0,5.0,0.0);

    SmBrep* pResult2 = NULL;
    stat = SmApiCreateSphere(sPositon2, 10, pResult2);

    SmTArray<SmCurve*> sCurves3d;
    SmTArray<SmPoint3d> sPoints3d;
    stat = SmApiIntersectBreps(pResult1, pResult2, sCurves3d, &sPoints3d);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmIntersectCurveSurface()
{
    SmApiCreateContext();
    SmStatus retStat = SM_SUCCESS;

    // --- Transverse intersection: circle crosses tilted plane ---
    {
        SmPoint3d sCenter(5.0, 5.0, 0.0);
        SmBSplineCurve* pCircle = NULL;
        SmApiCreateCircle(sCenter, 3.0, pCircle);
        if (!pCircle)
            return SM_ERR;

        SmVector3d c1(0,0,-5), c2(10,0,-5), c3(0,10,5), c4(10,10,5);
        SmSurface* pSrf = NULL;
        SmApiCreateSurfaceFromCornerPoints(c1, c2, c3, c4, pSrf);
        if (!pSrf)
        {
            delete pCircle;
            return SM_ERR;
        }

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        SmStatus stat = SmApiIntersectCurveSurface(pCircle, pSrf, sCurves, sPoints);

        if( stat != SM_SUCCESS )
            retStat = SM_ERR;
        else if( sPoints.GetSize() == 0 && sCurves.GetSize() == 0 )
            retStat = SM_ERR;

        for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
        {
            if( !ValidateCurveOnSurface( sCurves[ii], pSrf ) )
                retStat = SM_ERR;
            delete sCurves[ii];
        }

        for( ULONG ii = 0; ii < sPoints.GetSize(); ii++ )
        {
            SmPoint3d sClosest;
            SmTArray<SmPoint2d> sUVs;
            double dDist = 0;
            SmStatus eClosestStatus =
                SmApiSurfaceClosestPoint( pSrf, sPoints[ii], sClosest, sUVs, dDist );
            if( eClosestStatus != SM_SUCCESS || sUVs.GetSize() < 1 || dDist > 0.01 )
                retStat = SM_ERR;
        }

        delete pCircle;
        delete pSrf;
    }

    if( retStat != SM_SUCCESS )
        return retStat;

    // --- Coincident intersection: circle lies on a planar surface ---
    {
        SmPoint3d sCenter(5.0, 5.0, 0.0);
        SmBSplineCurve* pCircle = NULL;
        SmApiCreateCircle(sCenter, 2.0, pCircle);
        if (!pCircle)
            return SM_ERR;

        SmVector3d p1(0,0,0), p2(10,0,0), p3(0,10,0), p4(10,10,0);
        SmSurface* pSrf = NULL;
        SmApiCreateSurfaceFromCornerPoints(p1, p2, p3, p4, pSrf);
        if (!pSrf)
        {
            delete pCircle;
            return SM_ERR;
        }

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        SmStatus stat = SmApiIntersectCurveSurface(pCircle, pSrf, sCurves, sPoints);

        if( stat != SM_SUCCESS )
            retStat = SM_ERR;
        else if( sCurves.GetSize() == 0 )
            retStat = SM_ERR;

        for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
        {
            if( !ValidateCurveOnSurface( sCurves[ii], pSrf ) )
                retStat = SM_ERR;
            delete sCurves[ii];
        }

        delete pCircle;
        delete pSrf;
    }

    return retStat;
}

//*************************************************************************
// 
//*************************************************************************

//*************************************************************************
// A 10-box with a radius-2 through-hole along Z at (5,5), and its top face.
//*************************************************************************

static SmBrep* sm_CreateHoledBox( SmFace*& rpTopFace )
{
    rpTopFace = NULL;
    SmVector3d sBoxOrigin(0.0, 0.0, 0.0);
    SmVector3d sCylOrigin(5.0, 5.0, -1.0);
    SmBrep* pBox = NULL;
    SmBrep* pCyl = NULL;
    SmApiCreateBox(sBoxOrigin, 10, 10, 10, pBox);
    SmApiCreateCylinder(sCylOrigin, 2.0, 12.0, pCyl);
    SmBrep* pHoled = NULL;
    if (!pBox || !pCyl || SmApiBooleanDifference(pBox, pCyl, pHoled) != SM_SUCCESS || !pHoled)
        return NULL;

    SmTArray<SmFace*> sFaces;
    SmApiGetFaces(pHoled, sFaces);
    for (ULONG ii = 0; ii < sFaces.GetSize(); ii++)
    {
        SmPoint3d sMin, sMax;
        if (SmApiFaceBoundingBox(sFaces[ii], TRUE, sMin, sMax) == SM_SUCCESS && sMin.z > 9.9)
            rpTopFace = sFaces[ii];
    }
    return pHoled;
}

SmStatus TestSmIntersectCurveFace()
{
    SmApiCreateContext();
    SmStatus retStat = SM_SUCCESS;

    // --- Transverse: line through box faces ---
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
        if (!pBox)
            return SM_ERR;

        SmTArray<SmFace*> sFaces;
        pBox->GetFaces(sFaces);
        if (sFaces.GetSize() < 1)
        {
            delete pBox;
            return SM_ERR;
        }

        SmPoint3d sStart(-5.0, 5.0, 5.0);
        SmPoint3d sEnd(15.0, 5.0, 5.0);
        SmLine* pLine = NULL;
        SmApiCreateLineSegment(sStart, sEnd, pLine);
        if (!pLine)
        {
            delete pBox;
            return SM_ERR;
        }

        SmBoolean bFoundHit = FALSE;
        for (ULONG ff = 0; ff < sFaces.GetSize(); ff++)
        {
            SmTArray<SmCurve*> sCurves;
            SmTArray<SmPoint3d> sPoints;
            SmStatus fStat = SmApiIntersectCurveFace(pLine, sFaces[ff], sCurves, sPoints);
            if (fStat == SM_SUCCESS && (sCurves.GetSize() > 0 || sPoints.GetSize() > 0))
            {
                bFoundHit = TRUE;

                SmSurface* pFaceSrf = sFaces[ff]->GetSurface();
                for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
                {
                    if( pFaceSrf && !ValidateCurveOnSurface( sCurves[ii], pFaceSrf ) )
                        retStat = SM_ERR;
                }

                for( ULONG ii = 0; ii < sPoints.GetSize(); ii++ )
                {
                    if( pFaceSrf )
                    {
                        SmPoint3d sClosest;
                        SmTArray<SmPoint2d> sUVs;
                        double dDist = 0;
                        SmStatus eClosestStatus =
                            SmApiSurfaceClosestPoint( pFaceSrf, sPoints[ii], sClosest, sUVs, dDist );
                        if( eClosestStatus != SM_SUCCESS || sUVs.GetSize() < 1 || dDist > 0.01 )
                            retStat = SM_ERR;
                    }
                }
            }
            for (ULONG ii = 0; ii < sCurves.GetSize(); ii++)
                delete sCurves[ii];
        }

        if( !bFoundHit )
            retStat = SM_ERR;

        delete pLine;
        delete pBox;
    }

    if( retStat != SM_SUCCESS )
        return retStat;

    // --- Coincident: circle lying on a box face (z=10 top face) ---
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
        if (!pBox)
            return SM_ERR;

        SmTArray<SmFace*> sFaces;
        pBox->GetFaces(sFaces);

        SmPoint3d sCenter(5.0, 5.0, 10.0);
        SmBSplineCurve* pCircle = NULL;
        SmApiCreateCircle(sCenter, 2.0, pCircle);
        if (!pCircle)
        {
            delete pBox;
            return SM_ERR;
        }

        SmBoolean bFoundCoincident = FALSE;
        for (ULONG ff = 0; ff < sFaces.GetSize(); ff++)
        {
            SmTArray<SmCurve*> sCurves;
            SmTArray<SmPoint3d> sPoints;
            SmStatus fStat = SmApiIntersectCurveFace(pCircle, sFaces[ff], sCurves, sPoints);
            if (fStat == SM_SUCCESS && sCurves.GetSize() > 0)
            {
                bFoundCoincident = TRUE;

                SmSurface* pFaceSrf = sFaces[ff]->GetSurface();
                for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
                {
                    if( pFaceSrf && !ValidateCurveOnSurface( sCurves[ii], pFaceSrf ) )
                        retStat = SM_ERR;
                }
            }
            for (ULONG ii = 0; ii < sCurves.GetSize(); ii++)
                delete sCurves[ii];
        }

        if( !bFoundCoincident )
            retStat = SM_ERR;

        delete pCircle;
        delete pBox;
    }

    if( retStat != SM_SUCCESS )
        return retStat;

    // --- Trimmed face: hits inside the hole are not on the face ---
    {
        SmFace* pTop = NULL;
        SmBrep* pHoled = sm_CreateHoledBox( pTop );
        if (!pHoled || !pTop)
            return SM_ERR;

        SmPoint3d sAxisStart(5.0, 5.0, -5.0), sAxisEnd(5.0, 5.0, 15.0);
        SmPoint3d sOffStart(1.0, 1.0, -5.0), sOffEnd(1.0, 1.0, 15.0);
        SmLine* pAxis = NULL;
        SmLine* pOff = NULL;
        SmApiCreateLineSegment(sAxisStart, sAxisEnd, pAxis);
        SmApiCreateLineSegment(sOffStart, sOffEnd, pOff);

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        SmApiIntersectCurveFace(pAxis, pTop, sCurves, sPoints);
        if (sPoints.GetSize() != 0 || sCurves.GetSize() != 0)
            retStat = SM_ERR;

        SmTArray<SmPoint3d> sOffPoints;
        if (   SmApiIntersectCurveFace(pOff, pTop, sCurves, sOffPoints) != SM_SUCCESS
            || sOffPoints.GetSize() != 1
            || sOffPoints[0].DistanceBetween(SmPoint3d(1.0, 1.0, 10.0)) > 1.0e-6)
            retStat = SM_ERR;

        for (ULONG ii = 0; ii < sCurves.GetSize(); ii++)
            delete sCurves[ii];
        delete pAxis;
        delete pOff;
        delete pHoled;
    }

    return retStat;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmIntersectCurveBrep()
{
    SmApiCreateContext();
    SmStatus retStat = SM_SUCCESS;

    // --- Transverse: line through box, expect 2 intersection points ---
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
        if (!pBox)
            return SM_ERR;

        SmPoint3d sStart(-5.0, 5.0, 5.0);
        SmPoint3d sEnd(15.0, 5.0, 5.0);
        SmLine* pLine = NULL;
        SmApiCreateLineSegment(sStart, sEnd, pLine);
        if (!pLine)
        {
            delete pBox;
            return SM_ERR;
        }

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        SmStatus stat = SmApiIntersectCurveBrep(pLine, pBox, sCurves, sPoints);

        if( stat != SM_SUCCESS )
            retStat = SM_ERR;
        else if( sPoints.GetSize() < 2 )
            retStat = SM_ERR;

        for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
        {
            if( !ValidateCurveOnBrep( sCurves[ii], pBox ) )
                retStat = SM_ERR;
            delete sCurves[ii];
        }

        for( ULONG ii = 0; ii < sPoints.GetSize(); ii++ )
        {
            SmPoint3d sClosest;
            double dDist = 0;
            SmApiGetClosestPoint( pBox, sPoints[ii], sClosest, dDist );
            if( dDist > 0.01 )
                retStat = SM_ERR;
        }

        delete pLine;
        delete pBox;
    }

    if( retStat != SM_SUCCESS )
        return retStat;

    // --- Coincident: line segment along a box edge (z=0, y=0 edge) ---
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
        if (!pBox)
            return SM_ERR;

        SmPoint3d sStart(2.0, 0.0, 0.0);
        SmPoint3d sEnd(8.0, 0.0, 0.0);
        SmLine* pLine = NULL;
        SmApiCreateLineSegment(sStart, sEnd, pLine);
        if (!pLine)
        {
            delete pBox;
            return SM_ERR;
        }

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        SmStatus stat = SmApiIntersectCurveBrep(pLine, pBox, sCurves, sPoints);

        if( stat == SM_SUCCESS && (sCurves.GetSize() > 0 || sPoints.GetSize() > 0) )
        {
            for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
            {
                if( !ValidateCurveOnBrep( sCurves[ii], pBox ) )
                    retStat = SM_ERR;
            }
        }
        else
        {
            retStat = SM_ERR;
        }

        for( ULONG ii = 0; ii < sCurves.GetSize(); ii++ )
            delete sCurves[ii];
        delete pLine;
        delete pBox;
    }

    if( retStat != SM_SUCCESS )
        return retStat;

    // --- A line down the hole axis misses the holed box entirely ---
    {
        SmFace* pTop = NULL;
        SmBrep* pHoled = sm_CreateHoledBox( pTop );
        if (!pHoled)
            return SM_ERR;

        SmPoint3d sStart(5.0, 5.0, -5.0), sEnd(5.0, 5.0, 15.0);
        SmLine* pAxis = NULL;
        SmApiCreateLineSegment(sStart, sEnd, pAxis);

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        SmApiIntersectCurveBrep(pAxis, pHoled, sCurves, sPoints);
        if (sPoints.GetSize() != 0 || sCurves.GetSize() != 0)
            retStat = SM_ERR;

        for (ULONG ii = 0; ii < sCurves.GetSize(); ii++)
            delete sCurves[ii];
        delete pAxis;
        delete pHoled;
    }

    // --- A line crossing two box edges reports each crossing once ---
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
        SmPoint3d sStart(-5.0, -5.0, 5.0), sEnd(15.0, 15.0, 5.0);
        SmLine* pDiag = NULL;
        SmApiCreateLineSegment(sStart, sEnd, pDiag);
        if (!pBox || !pDiag)
            return SM_ERR;

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        if (   SmApiIntersectCurveBrep(pDiag, pBox, sCurves, sPoints) != SM_SUCCESS
            || sPoints.GetSize() != 2)
            retStat = SM_ERR;

        for (ULONG ii = 0; ii < sCurves.GetSize(); ii++)
            delete sCurves[ii];
        delete pDiag;
        delete pBox;
    }

    // --- A segment lying along a box edge is one overlap, not one per face ---
    {
        SmVector3d sOrigin(0.0, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
        SmPoint3d sStart(2.0, 0.0, 0.0), sEnd(8.0, 0.0, 0.0);
        SmLine* pAlong = NULL;
        SmApiCreateLineSegment(sStart, sEnd, pAlong);
        if (!pBox || !pAlong)
            return SM_ERR;

        SmTArray<SmCurve*> sCurves;
        SmTArray<SmPoint3d> sPoints;
        if (   SmApiIntersectCurveBrep(pAlong, pBox, sCurves, sPoints) != SM_SUCCESS
            || sCurves.GetSize() != 1)
            retStat = SM_ERR;

        for (ULONG ii = 0; ii < sCurves.GetSize(); ii++)
            delete sCurves[ii];
        delete pAlong;
        delete pBox;
    }

    return retStat;
}


SmStatus TestSmIntersectors()
{

    SmStatus stat = TestSmIntersectCurves();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmIntersectBrepWithPlane();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmIntersectBreps();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmIntersectCurveSurface();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmIntersectCurveFace();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmIntersectCurveBrep();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}
