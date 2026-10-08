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
#include <SmApiIntersectors.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmMessages.h>
#include <SmBrep.h>
#include <SmFace.h>

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurface()
{

    SmApiCreateContext();

    SmTArray<SmPoint3d>rControlPointsList;
    rControlPointsList.Add(SmPoint3d(-9.99999999999999, -90.00000000000001, -40.00000000000000));       //   [0][0] =>> 0
    rControlPointsList.Add(SmPoint3d(-15.00993828742204, -90.00000000000000, -37.49503085628898));       //   [0][1] =>> 1
    rControlPointsList.Add(SmPoint3d(-20.89918488675400, -90.00000000000000, -29.46066919081585));       //   [0][2] =>> 2
    rControlPointsList.Add(SmPoint3d(-29.86166135600070, -90.00000000000000, -24.45419962835159));       //   [0][3] =>> 3
    rControlPointsList.Add(SmPoint3d(-41.04540877725864, -90.00000000000000, -21.58500943904096));       //   [0][4] =>> 4
    rControlPointsList.Add(SmPoint3d(-54.08153368920120, -90.00000000000000, -19.79899179084966));       //   [0][5] =>> 5
    rControlPointsList.Add(SmPoint3d(-62.62953406581700, -90.00000000000000, -23.21819194149598));       //   [0][6] =>> 6
    rControlPointsList.Add(SmPoint3d(-9.99999999999999, -20.00000000000001, -50.00000000000001));       //   [1][0] =>> 7
    rControlPointsList.Add(SmPoint3d(-14.17192594490645, -20.00000000000001, -47.91403702754678));       //   [1][1] =>> 8
    rControlPointsList.Add(SmPoint3d(-18.11902186410478, -19.99999999999999, -39.11280302897903));       //   [1][2] =>> 9
    rControlPointsList.Add(SmPoint3d(-25.35399362720083, -20.00000000000004, -33.42503955402192));       //   [1][3] =>> 10
    rControlPointsList.Add(SmPoint3d(-35.09449053271035, -20.00000000000002, -29.66201132684917));       //   [1][4] =>> 11
    rControlPointsList.Add(SmPoint3d(-47.19784042704145, -20.00000000000001, -26.67879014901961));       //   [1][5] =>> 12
    rControlPointsList.Add(SmPoint3d(-55.15544087898040, -20.00000000000001, -29.86183032979519));       //   [1][6] =>> 13
    rControlPointsList.Add(SmPoint3d(-9.99999999999999, -10.00000000000001, -20.00000000000001));       //   [2][0] =>> 14
    rControlPointsList.Add(SmPoint3d(-16.68596297245324, -10.00000000000001, -16.65701851377339));       //   [2][1] =>> 15
    rControlPointsList.Add(SmPoint3d(-26.45951093205240, -10.00000000000001, -10.15640151448953));       //   [2][2] =>> 16
    rControlPointsList.Add(SmPoint3d(-38.87699681360041, -10.00000000000001, -6.51251977701096));       //   [2][3] =>> 17
    rControlPointsList.Add(SmPoint3d(-52.94724526635518, -10.00000000000001, -5.43100566342460));       //   [2][4] =>> 18
    rControlPointsList.Add(SmPoint3d(-67.84892021352071, -10.00000000000001, -6.03939507450981));       //   [2][5] =>> 19
    rControlPointsList.Add(SmPoint3d(-77.57772043949019, -10.00000000000001, -9.93091516489760));       //   [2][6] =>> 20
    rControlPointsList.Add(SmPoint3d(-10.00000000000001, -39.99999999999999, 39.99999999999999));       //   [3][0] =>> 21
    rControlPointsList.Add(SmPoint3d(-21.71403702754681, -39.99999999999999, 45.85701851377338));       //   [3][1] =>> 22
    rControlPointsList.Add(SmPoint3d(-43.14048906794761, -39.99999999999999, 47.75640151448948));       //   [3][2] =>> 23
    rControlPointsList.Add(SmPoint3d(-65.92300318639960, -40.00000000000000, 47.31251977701093));       //   [3][3] =>> 24
    rControlPointsList.Add(SmPoint3d(-88.65275473364481, -40.00000000000000, 43.03100566342455));       //   [3][4] =>> 25
    rControlPointsList.Add(SmPoint3d(-109.15107978647926, -39.99999999999999, 35.23939507450980));       //   [3][5] =>> 26
    rControlPointsList.Add(SmPoint3d(-122.42227956050978, -39.99999999999999, 29.93091516489760));       //   [3][6] =>> 27
    rControlPointsList.Add(SmPoint3d(-10.00000000000000, -90.00000000000000, 10.00000000000000));       //   [4][0] =>> 28
    rControlPointsList.Add(SmPoint3d(-19.20000000000002, -90.00000000000000, 14.60000000000001));       //   [4][1] =>> 29
    rControlPointsList.Add(SmPoint3d(-34.80000000000001, -90.00000000000000, 18.80000000000000));       //   [4][2] =>> 30
    rControlPointsList.Add(SmPoint3d(-52.40000000000000, -90.00000000000000, 20.40000000000000));       //   [4][3] =>> 31
    rControlPointsList.Add(SmPoint3d(-70.80000000000000, -90.00000000000000, 18.80000000000000));       //   [4][4] =>> 32
    rControlPointsList.Add(SmPoint3d(-88.50000000000000, -90.00000000000000, 14.60000000000000));       //   [4][5] =>> 33
    rControlPointsList.Add(SmPoint3d(-99.99999999999999, -90.00000000000000, 10.00000000000001));       //   [4][6] =>> 34

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurface( rControlPointsList, 5, 7, pResult);
    if( stat != SM_SUCCESS )
        return( stat );

    // Fewer than 4 rows or columns used to underflow the knot counts.
    SmTArray<SmPoint3d> sTwoRows;
    for( ULONG ii = 0; ii < 2 * 7; ii++ )
        sTwoRows.Add( rControlPointsList[ii] );
    SmSurface* pTooFew = NULL;
    if( SmApiCreateSurface( sTwoRows, 2, 7, pTooFew ) != SM_ERR_INVALID_INPUT || pTooFew != NULL
        || SmApiCreateSurface( sTwoRows, 7, 2, pTooFew ) != SM_ERR_INVALID_INPUT || pTooFew != NULL )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceFromPoints()
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
   
    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceFromOrderedPoints(rPointsList, 5, 7, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceFromRandomPoints()
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

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceFromRandomPoints(rPointsList, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceFromCornerPoints()
{

    SmApiCreateContext();

    SmVector3d sCorner1(0.0,0.0,0.0);
    SmVector3d sCorner2(10.0,0.0,0.0);
    SmVector3d sCorner3(0.0,12.0,0.0);
    SmVector3d sCorner4(10.0,12.0,0.0);
   
    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceFromCornerPoints(sCorner1, sCorner2, sCorner3, sCorner4, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // The evaluators reject a NaN parameter and a NULL surface instead of
    // returning SM_SUCCESS with NaN or unset outputs.
    const double dNaN = std::numeric_limits<double>::quiet_NaN();
    SmVector2d sNaNUV( dNaN, 0.0 );
    SmVector3d sPt, sNrm, sDU, sDV;
    if(   SmApiEvaluateSurfacePoint( pResult, sNaNUV, sPt ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateSurfaceNormal( pResult, sNaNUV, sNrm ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateSurfaceDerivatives( pResult, sNaNUV, sPt, sDU, sDV ) != SM_ERR_INVALID_INPUT )
        stat = SM_ERR;

    // Outside the parameter domain is invalid input too.
    SmVector2d sOutUV( pResult->GetNaturalUVDomain().GetMax().x + 1.0, 0.0 );
    if(   SmApiEvaluateSurfacePoint( pResult, sOutUV, sPt ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateSurfaceNormal( pResult, sOutUV, sNrm ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateSurfaceDerivatives( pResult, sOutUV, sPt, sDU, sDV ) != SM_ERR_INVALID_INPUT )
        stat = SM_ERR;

    SmVector2d sUV( 0.5, 0.5 );
    if(   SmApiEvaluateSurfacePoint( NULL, sUV, sPt ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateSurfaceNormal( NULL, sUV, sNrm ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateSurfaceDerivatives( NULL, sUV, sPt, sDU, sDV ) != SM_ERR_INVALID_INPUT )
        stat = SM_ERR;

    delete pResult;
    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateExtrude()
{

    SmApiCreateContext();

    SmVector3d pt1(0.0,0.0,0.0);
    SmVector3d pt2(4.0,2.0,0.0);
    SmVector3d pt3(6.0,-2.0,0.0);
    SmVector3d pt4(8.0,2.0,0.0);
    SmVector3d pt5(10.0,4.0,0.0);
    SmVector3d pt6(12.0,0.0,0.0);

    SmTArray<SmPoint3d> points;
    points.Add( pt1 );
    points.Add( pt2 );
    points.Add( pt3 );
    points.Add( pt4 );
    points.Add( pt5 );
    points.Add( pt6 );

    SmBSplineCurve* pNewCurve = NULL;
    SmApiCreateCurve(points, pNewCurve);

    SmVector3d dirVector( 0,0,1 );
   
    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateExtrude(pNewCurve, dirVector, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // Kernel failures must reach the caller with the output cleared, for both
    // the linear extrude (zero vector) and the path sweep (zero-length path).
    SmVector3d sZeroVector( 0,0,0 );
    SmSurface* pFailed = pResult;
    if( SmApiCreateExtrude(pNewCurve, sZeroVector, pFailed) == SM_SUCCESS || pFailed != NULL )
        return SM_ERR;

    SmTArray<SmPoint3d> sDegenPoints;
    for( int ii = 0; ii < 4; ii++ )
        sDegenPoints.Add( SmPoint3d(5.0, 5.0, 5.0) );
    SmBSplineCurve* pDegenPath = NULL;
    SER( SmApiCreateCurve(sDegenPoints, pDegenPath) );
    pFailed = pResult;
    stat = SmApiCreateSweep(pNewCurve, pDegenPath, pFailed);
    delete pDegenPath;
    if( stat == SM_SUCCESS || pFailed != NULL )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateRuledSurface()
{

    SmApiCreateContext();

    SmVector3d pt1(0.0,0.0,0.0);
    SmVector3d pt2(4.0,2.0,0.0);
    SmVector3d pt3(6.0,-2.0,0.0);
    SmVector3d pt4(8.0,2.0,0.0);
    SmVector3d pt5(10.0,4.0,0.0);
    SmVector3d pt6(12.0,0.0,0.0);

    SmTArray<SmPoint3d> points;
    points.Add( pt1 );
    points.Add( pt2 );
    points.Add( pt3 );
    points.Add( pt4 );
    points.Add( pt5 );
    points.Add( pt6 );

    SmBSplineCurve* pCrv1 = NULL;
    SmApiCreateCurve(points, pCrv1);

    SmVector3d pt10(0.0,0.0,10.0);
    SmVector3d pt20(4.0,2.0,10.0);
    SmVector3d pt30(6.0,-2.0,10.0);
    SmVector3d pt40(8.0,2.0,10.0);
    SmVector3d pt50(10.0,4.0,10.0);
    SmVector3d pt60(12.0,0.0,10.0);

    SmTArray<SmPoint3d> points2;
    points2.Add( pt10 );
    points2.Add( pt20 );
    points2.Add( pt30 );
    points2.Add( pt40 );
    points2.Add( pt50 );
    points2.Add( pt60 );

    SmBSplineCurve* pCrv2 = NULL;
    SmApiCreateCurve(points2, pCrv2);
   
    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateRuledSurface(pCrv1, pCrv2, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceRevolution()
{


    SmApiCreateContext();

    SmVector3d pt1(0.0,3.0,0.0);
    SmVector3d pt2(4.0,5.0,0.0);
    SmVector3d pt3(6.0,1.0,0.0);
    SmVector3d pt4(8.0,5.0,0.0);
    SmVector3d pt5(10.0,7.0,0.0);
    SmVector3d pt6(12.0,3.0,0.0);

    SmTArray<SmPoint3d> points;
    points.Add( pt1 );
    points.Add( pt2 );
    points.Add( pt3 );
    points.Add( pt4 );
    points.Add( pt5 );
    points.Add( pt6 );

    SmBSplineCurve* pCrv = NULL;
    SmApiCreateCurve(points, pCrv);

    SmVector3d sOrigin( 0,0,0 );
    SmVector3d sAxis( 1,0,0 );
    double dAngle = 180.0;

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceRevolution(pCrv, sOrigin, sAxis, dAngle, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // A zero-length generatrix is rejected by the kernel; the failure must
    // reach the caller and the output must be cleared.
    SmTArray<SmPoint3d> sDegenPoints;
    for( int ii = 0; ii < 4; ii++ )
        sDegenPoints.Add( SmPoint3d(0.0, 5.0, 0.0) );
    SmBSplineCurve* pDegenCrv = NULL;
    SER( SmApiCreateCurve(sDegenPoints, pDegenCrv) );

    SmSurface* pDegenResult = pResult;
    stat = SmApiCreateSurfaceRevolution(pDegenCrv, sOrigin, sAxis, dAngle, pDegenResult);
    delete pDegenCrv;
    if( stat == SM_SUCCESS || pDegenResult != NULL )
        return SM_ERR;

    SmSurface* pNullCurveResult = pResult;
    if( SmApiCreateSurfaceRevolution(NULL, sOrigin, sAxis, dAngle, pNullCurveResult) == SM_SUCCESS
        || pNullCurveResult != NULL )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceRevolutionSphere()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0,0.0,0.0);
    
    SmBSplineCurve* pSemiCircle = NULL;
    SmApiCreateArc(sCenter, 10.0, 0.0, 180.0, pSemiCircle);

    SmVector3d sBasePt(-10.0,0.0,0.0);
    SmVector3d sAxis( 1,0,0 );
    double dAngle = 360.0;

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceRevolution(pSemiCircle, sBasePt, sAxis, dAngle, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceRevolutionTorus()
{

    SmApiCreateContext();

    SmVector3d sCenter(50.0,0.0,0.0);
    
    SmBSplineCurve* pCircle = NULL;
    SmApiCreateArc(sCenter, 5.0, 0.0, 360.0, pCircle);

    // Rotate circle about x axis 90 so circle is in x-z plane
    SmVector3d sRotAxis(1, 0, 0);
    SmApiRotate(pCircle, sCenter, sRotAxis, 90.0);

    sCenter.Set( 0.0,0.0,0.0 );
    SmVector3d sAxis( 0,0,1 );
    double dAngle = 360.0;

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceRevolution(pCircle, sCenter, sAxis, dAngle, pResult);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceRevolutionCylinder()
{

    SmApiCreateContext();

    SmVector3d sStartPt(5.0,0.0,0.0);
    SmVector3d sEndPt(5.0,0.0,10.0);
    
    SmLine* pLine = NULL;
    SmApiCreateLineSegment( sStartPt, sEndPt, pLine);

    SmVector3d sCenter(0.0,0.0,0.0);
    SmVector3d sAxis( 0,0,1 );
    double dAngle = 360.0;

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceRevolution((SmBSplineCurve*)pLine, sCenter, sAxis, dAngle, pResult);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceRevolutionCone()
{

    SmApiCreateContext();

    SmVector3d sStartPt(5.0,0.0,0.0);
    SmVector3d sEndPt(0.0,0.0,10.0);
    
    SmLine* pLine = NULL;
    SmApiCreateLineSegment( sStartPt, sEndPt, pLine);

    SmVector3d sCenter(0.0,0.0,0.0);
    SmVector3d sAxis( 0,0,1 );
    double dAngle = 360.0;

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceRevolution((SmBSplineCurve*)pLine, sCenter, sAxis, dAngle, pResult);
    

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSurfaceRevolutionTruncCone()
{

    SmApiCreateContext();

    SmVector3d sStartPt(5.0,0.0,0.0);
    SmVector3d sEndPt(3.0,0.0,10.0);
    
    SmLine* pLine = NULL;
    SmApiCreateLineSegment( sStartPt, sEndPt, pLine);

    SmVector3d sCenter(0.0,0.0,0.0);
    SmVector3d sAxis( 0,0,1 );
    double dAngle = 360.0;

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSurfaceRevolution((SmBSplineCurve*)pLine, sCenter, sAxis, dAngle, pResult);
  

    return( stat );
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateOffsetSurface()
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

    SmSurface* pSrfToOffset = NULL;
    SmApiCreateSurfaceFromOrderedPoints(rPointsList, 5, 7, pSrfToOffset);

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateOffsetSurface(pSrfToOffset, -12.0, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // Offsetting a radius-10 sphere inward by 20 collapses it: the kernel
    // yields no surface, which must be reported as a failure.
    SmVector3d sCenter( 0,0,0 );
    SmBrep* pSphere = NULL;
    SER( SmApiCreateSphere(sCenter, 10.0, pSphere) );
    SmTArray<SmFace*> sFaces;
    SER( SmApiGetFaces(pSphere, sFaces) );
    if( sFaces.GetSize() == 0 )
        return SM_ERR;

    SmSurface* pNullInput = pResult;
    if( SmApiCreateOffsetSurface(NULL, 1.0, pNullInput) != SM_ERR_INVALID_INPUT || pNullInput != NULL )
        return SM_ERR;

    SmSurface* pCollapsed = pResult;
    stat = SmApiCreateOffsetSurface(sFaces[0]->GetSurface(), -20.0, pCollapsed);
    delete pSphere;
    if( stat == SM_SUCCESS || pCollapsed != NULL )
        return SM_ERR;

    // The torus offset kernel reports success without writing its output when
    // the minor radius collapses; a stale caller pointer must not survive.
    SmVector3d sTorusCenter( 0,0,0 );
    SmSurface* pTorus = NULL;
    SER( SmApiCreateTorus(sTorusCenter, 10.0, 2.0, pTorus) );
    SmSurface* pTorusOffset = pResult;
    stat = SmApiCreateOffsetSurface(pTorus, -5.0, pTorusOffset);
    delete pTorus;
    if( stat == SM_SUCCESS || pTorusOffset != NULL )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmCreateSkinSurface()
{
    SmApiCreateContext();

    SmBSplineCurve* pCircle1 = NULL;
    SmBSplineCurve* pCircle2 = NULL;
    SmBSplineCurve* pCircle3 = NULL;
    SmPoint3d sC1(0, 0, 0), sC2(0, 0, 5), sC3(0, 0, 10);
    SmApiCreateCircle(sC1, 5.0, pCircle1);
    SmApiCreateCircle(sC2, 3.0, pCircle2);
    SmApiCreateCircle(sC3, 5.0, pCircle3);
    if (!pCircle1 || !pCircle2 || !pCircle3)
        return SM_ERR;

    SmTArray<SmBSplineCurve*> sProfiles;
    sProfiles.Add(pCircle1);
    sProfiles.Add(pCircle2);
    sProfiles.Add(pCircle3);

    SmSurface* pResult = NULL;
    SmStatus stat = SmApiCreateSkin(sProfiles, pResult);
    if (stat != SM_SUCCESS)
        return stat;
    if (!pResult)
        return SM_ERR;

    return SM_SUCCESS;
}


SmStatus TestSmSurfaces()
{
    SmStatus stat = TestSmCreateSurfaceRevolutionSphere();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSurfaceRevolutionTorus();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSurfaceRevolutionCylinder();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSurfaceRevolutionCone();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateSurfaceRevolutionTruncCone();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}


// 
#if 0
SmStatus SmTestConeOffset()
{

	// Method 0 = SmBsplineSurface 1 = SmSurfOfRevolution 2 = SmCone
	// Base Radius
	// Top Radius
	// Height
	// Offset Distance

	SmStatus stat = SM_SUCCESS;
	// Full cone toward outside
	stat &= SmConeOffset(0, 4, 0, 10, 1.0);
	stat &= SmConeOffset(1, 4, 0, 10, 1.0);
	stat &= SmConeOffset(2, 4, 0, 10, 1.0);

	// Full cone toward inside
	stat &= SmConeOffset(0, 4, 0, 10, -1.0);	// Broken
	stat &= SmConeOffset(1, 4, 0, 10, -1.0); 
	stat &= SmConeOffset(2, 4, 0, 10, -1.0);

	// Full upside down cone toward outside
	stat = SmConeOffset(0, 0, 4, 10, 1.0);
	stat = SmConeOffset(1, 0, 4, 10, 1.0);
	stat = SmConeOffset(2, 0, 4, 10, 1.0);

	// Full upside down cone toward inside
	stat = SmConeOffset(0, 0, 4, 10, -1.0); // Broken
	stat = SmConeOffset(1, 0, 4, 10, -1.0); 
	stat = SmConeOffset(2, 0, 4, 10, -1.0);

	// truncated cone toward outside
	stat = SmConeOffset(0, 4, 1, 10, 1.0);
	stat = SmConeOffset(1, 4, 1, 10, 1.0);
	stat = SmConeOffset(2, 4, 1, 10, 1.0);

	// upside down truncated cone toward outside
	stat = SmConeOffset(0, 1, 4, 10, 1.0);
	stat = SmConeOffset(1, 1, 4, 10, 1.0);
	stat = SmConeOffset(2, 1, 4, 10, 1.0);

	// truncated cone toward inside
	stat = SmConeOffset(0, 4, 1, 10, -1.5);	// Broken
	stat = SmConeOffset(1, 4, 1, 10, -1.0);
	stat = SmConeOffset(2, 4, 1, 10, -1.0);

	// upside down truncated cone toward inside
	stat = SmConeOffset(0, 1, 4, 10, -1.0);
	stat = SmConeOffset(1, 1, 4, 10, -1.0);
	stat = SmConeOffset(2, 1, 4, 10, -1.0);

	return(stat);

}
#endif



#if 0
SM_EXPORT int SmTestSphereOffset(int method, double dRadius, double dOffset)
{
	SmContext sContext;

	SmBSplineSurface* pSrf = NULL;
	SmAxis2Placement sFrame;

	switch (method)
	{
		// SmBSplineSurface
	case 0:
	{
		SmBSplineSurface::CreateSurfOfRevolution( (sContext, sFrame, dRadius, 0.0, 360.0, dHeight, SM_CO_QUADRATIC, pSrf);
		// CreateSphereFromArcs
	}
	break;

	// SmSurfOfRevolution
	case 1:
	{
		SmVector3d sOrigin(0, 0, 0);
		SmVector3d zAxis(0, 0, dHeight);

		SmVector3d sStartPt(dBaseRadius, 0, 0);
		SmVector3d sEndPt(dTopRadius, 0, dHeight);

		SmLine* pLine = NULL;
		SmLine::CreateArcSegment(sContext, 3, sStartPt, sEndPt, pLine);

		SmSurfOfRevolution* pSrfOfRev = NULL;
		SmSurfOfRevolution::CreateCanonical(sContext, pLine, sOrigin, zAxis, pSrfOfRev);

		pSrf = pSrfOfRev;
	}
	break;

	// SmSphere
	case 2:
	{
		SmSphere* pSphere = NULL;
		SmSphere::CreateCanonical(sContext, sFrame, dRadius, pSphere);
		pSrf = pSphere;
	}

	break;

	default:
		break;

	}

	SM_ASSERT_VALID(pSrf);
	smgfx_Erase(); smgfx_SetLook(2, 2, 0, 0, 1);
	pSrf->Draw(); sm_GraphicsLoop();
	//UserTest::Draw(pSrf);

	double dApproxTol = 0.0001;
	SmTArray<SmSurface*>pOffSrfs;
	SmStatus stat = pSrf->CreateOffsetSurface(sContext, dOffset, dApproxTol, pOffSrfs);

	if (stat == SM_SUCCESS) {
		// Might result in more than one surface
		// But draw only first one to see if we got the primary one
		//for (ULONG ii = 0; ii < pOffSrfs.GetSize(); ii++) {
		ULONG ii = 0;
		smgfx_SetLook(2, 2, 1, 0, 0); pOffSrfs[ii]->Draw(); sm_GraphicsLoop();
		UserTest::Draw(pOffSrfs[ii]);
		//}
	}
	else {
		return SM_ERR;
	}

	return stat;
}
#endif
