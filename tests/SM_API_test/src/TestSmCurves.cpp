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
#include <SmMessages.h>   // SM_ERR_INVALID_INPUT
#include <SmMath.h>       // SM_EFF_ZERO
#include <SmBSplineCurve.h>
#include <limits>

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

//*************************************************************************
// 
//*************************************************************************
// Knot type tag stored on a B-spline curve.
static SmKnotType sm_GetKnotType( const SmBSplineCurve& crCurve )
{
    ULONG lDim = 0, lDeg = 0;
    SmTArray<SmPoint3d> sCPs;
    SmBSplineCurveForm eForm;
    SmTArray<ULONG> sMults;
    SmTArray<double> sKnots, sWeights;
    SmKnotType eKnotType = SM_KT_UNKOWN;
    crCurve.GetCanonical( lDim, lDeg, sCPs, eForm, sMults, sKnots, eKnotType, sWeights );
    return eKnotType;
}

SmStatus TestSmCreateLineSegment()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
    SmVector3d sPositon2(10.0,10.0,0.0);
   
    SmLine* pLine = NULL;
    SmStatus stat = SmApiCreateLineSegment( sPositon1,  sPositon2, pLine);
    if( stat != SM_SUCCESS || pLine == NULL )
        return SM_ERR;

    // A zero direction used to produce a NaN line with SM_SUCCESS.
    SmVector3d sZeroDir(0.0,0.0,0.0);
    SmLine* pOutLine = pLine;
    if( SmApiCreateLine( sPositon1, sZeroDir, pOutLine ) != SM_ERR_INVALID_INPUT || pOutLine != NULL )
        return SM_ERR;

    // A negative ellipse radius used to succeed.
    SmEllipse* pEllipse = (SmEllipse*)pLine;
    if( SmApiCreateEllipse( sPositon1, -2.0, 1.0, pEllipse ) != SM_ERR_INVALID_INPUT || pEllipse != NULL )
        return SM_ERR;

    // Each radius is checked only along its own axis. At y = 1e16 doubles are
    // 2 apart, so a 0.5 radius vanishes in Y but not in X.
    SmPoint3d sFarCenter( 0.0, 1.0e16, 0.0 );
    SmEllipse* pFarEllipse = NULL;
    if( SmApiCreateEllipse( sFarCenter, 0.5, 1.0e10, pFarEllipse ) != SM_SUCCESS || pFarEllipse == NULL )
        return SM_ERR;
    delete pFarEllipse;
    pFarEllipse = NULL;
    if( SmApiCreateEllipse( sFarCenter, 1.0e10, 0.5, pFarEllipse ) != SM_ERR_INVALID_INPUT || pFarEllipse != NULL )
        return SM_ERR;

    delete pLine;
    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreateCircle()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0,0.0,0.0);

    double dRadius = 10.0;
   
    SmBSplineCurve* pResult = NULL;
    long stat = SmApiCreateCircle( sCenter, dRadius, pResult);
    

    return( stat );
}


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreateArc()
{

    SmApiCreateContext();

    SmVector3d sCenter(0.0,0.0,0.0);

    double dRadius = 10.0;
   
    SmBSplineCurve* pResult = NULL;
    long stat = SmApiCreateArc(sCenter, dRadius, 0, 135.0, pResult);
    if( stat != SM_SUCCESS || pResult == NULL )
        return SM_ERR;

    // SmApiEvaluateCurve rejects a NaN parameter and a NULL curve, and
    // leaves the outputs unwritten.
    const SmVector3d sSentinel( 7.0, 8.0, 9.0 );
    SmVector3d sPt = sSentinel, sD1 = sSentinel, sD2 = sSentinel;
    SmVector3d* pPt = &sPt;
    SmVector3d* pD1 = &sD1;
    SmVector3d* pD2 = &sD2;
    const double dPastEnd = pResult->GetNaturalInterval().GetMax() + 1.0;
    if(   SmApiEvaluateCurve( pResult, std::numeric_limits<double>::quiet_NaN(), pPt, pD1, pD2 ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateCurve( pResult, dPastEnd, pPt, pD1, pD2 ) != SM_ERR_INVALID_INPUT
       || SmApiEvaluateCurve( NULL, 0.0, pPt, pD1, pD2 ) != SM_ERR_INVALID_INPUT
       || !(sPt == sSentinel) || !(sD1 == sSentinel) || !(sD2 == sSentinel) )
        stat = SM_ERR;

    delete pResult;
    return( stat );
}


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreateCurve()
{

    SmApiCreateContext();

    SmTArray<SmPoint3d> sPoints;
    SmVector3d sPoint1(0.0,0.0,0.0);
    sPoints.Add( sPoint1 );
    SmVector3d sPoint2(10.0,-3.0,0.0);
    sPoints.Add( sPoint2 );
    SmVector3d sPoint3(20.0,3.0,0.0);
    sPoints.Add( sPoint3 );
    SmVector3d sPoint4(30.0,-6.0,0.0);
    sPoints.Add( sPoint4 );
   
    SmBSplineCurve* pResult = NULL;
    long stat = SmApiCreateCurve(sPoints, pResult);
    if( stat != SM_SUCCESS )
        return( stat );

    // The interior knots are not evenly spaced, so the curve must not be tagged uniform.
    if( sm_GetKnotType(*pResult) != SM_KT_UNSPECIFIED )
        return SM_ERR;

    // Fewer than 4 points used to underflow the knot count and loop ~forever.
    for( ULONG lKeep = 0; lKeep < 4; lKeep++ )
    {
        SmTArray<SmPoint3d> sFew;
        for( ULONG ii = 0; ii < lKeep; ii++ )
            sFew.Add( sPoints[ii] );
        SmBSplineCurve* pFew = NULL;
        if( SmApiCreateCurve(sFew, pFew) != SM_ERR_INVALID_INPUT || pFew != NULL )
            return SM_ERR;
    }

    return( SM_SUCCESS );
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreateCanonicalCurve()
{

    SmApiCreateContext();

    SmTArray < SmPoint3d> sCPnts;
    sCPnts.Add(SmPoint3d(0, 0, 0));
    sCPnts.Add(SmPoint3d(0, 0.2, 0));
    sCPnts.Add(SmPoint3d(0, 0.4, 0.2));
    sCPnts.Add(SmPoint3d(0.2, 0.6, - 0.3));
    sCPnts.Add(SmPoint3d(0, 0.8, 0));
    sCPnts.Add(SmPoint3d(0, 0.9, 0.2));
    sCPnts.Add(SmPoint3d(0, 1.0, 0.5));

    SmTArray < ULONG> sKnotMult;
    sKnotMult.Add(4);
    sKnotMult.Add(1);
    sKnotMult.Add(1);
    sKnotMult.Add(1);
    sKnotMult.Add(4);

    SmTArray < double> sKnots;
    sKnots.Add(0.0);
    sKnots.Add(0.3);
    sKnots.Add(0.6);
    sKnots.Add(0.8);
    sKnots.Add(1.0);

    SmTArray < double> sWeights;
    sWeights.Add(1.5);
    sWeights.Add(0.5);
    sWeights.Add(1.5);
    sWeights.Add(0.5);
    sWeights.Add(1.5);
    sWeights.Add(0.5);
    sWeights.Add(1.5);
   
    SmBSplineCurve* pResult = NULL;
    long stat = SmApiCreateCanonicalCurve(sCPnts, sKnots, sKnotMult, &sWeights, 3, SM_CF_UNSPECIFIED, pResult);
    if( stat != SM_SUCCESS )
        return( stat );

    // Caller-supplied knots (here 0, .3, .6, .8, 1) are not tagged uniform.
    if( sm_GetKnotType(*pResult) != SM_KT_UNSPECIFIED )
        return SM_ERR;

    // Too few control points for the degree: rejected by the kernel instead of
    // hanging in the (removed) unused knot computation.
    SmTArray<SmPoint3d> sTwoPnts;
    sTwoPnts.Add(SmPoint3d(0, 0, 0));
    sTwoPnts.Add(SmPoint3d(1, 0, 0));
    SmTArray<double> sEndKnots;
    sEndKnots.Add(0.0);
    sEndKnots.Add(1.0);
    SmTArray<ULONG> sEndMult;
    sEndMult.Add(4);
    sEndMult.Add(4);
    SmBSplineCurve* pTooFew = NULL;
    if( SmApiCreateCanonicalCurve(sTwoPnts, sEndKnots, sEndMult, NULL, 3, SM_CF_UNSPECIFIED, pTooFew) == SM_SUCCESS )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmOffsetCurve()
{

    SmApiCreateContext();

    SmTArray<SmPoint3d> sPoints;
    SmVector3d sPoint1(0.0,0.0,0.0);
    sPoints.Add( sPoint1 );
    SmVector3d sPoint2(10.0,-3.0,0.0);
    sPoints.Add( sPoint2 );
    SmVector3d sPoint3(20.0,3.0,0.0);
    sPoints.Add( sPoint3 );
    SmVector3d sPoint4(30.0,-6.0,0.0);
    sPoints.Add( sPoint4 );
   
    SmBSplineCurve* pCurve = NULL;
    long stat = SmApiCreateCurve(sPoints, pCurve);
  
    SmBSplineCurve* pResult = NULL;
    stat = SmApiOffsetCurve(pCurve, 3.0, pResult);
    if( stat != SM_SUCCESS )
        return( stat );

    // A straight line has no plane of its own; it is offset in the XY plane.
    SmTArray<SmPoint3d> sLinePts;
    for( int ii = 0; ii < 4; ii++ )
        sLinePts.Add( SmPoint3d(ii * 10.0, 0.0, 0.0) );
    SmBSplineCurve* pLine = NULL;
    SER( SmApiCreateCurve(sLinePts, pLine) );
    SmBSplineCurve* pLineOffset = NULL;
    stat = SmApiOffsetCurve(pLine, 2.0, pLineOffset);
    delete pLine;
    if( stat != SM_SUCCESS || pLineOffset == NULL )
        return SM_ERR;
    SmPoint3d sOffStart, sOffEnd;
    pLineOffset->GetEnds( sOffStart, sOffEnd );
    delete pLineOffset;
    if( fabs(sOffStart.z) > 1.0e-9 || fabs(fabs(sOffStart.y) - 2.0) > 1.0e-6
        || fabs(sOffEnd.z) > 1.0e-9 || fabs(fabs(sOffEnd.y) - 2.0) > 1.0e-6 )
        return SM_ERR;

    // A line parallel to Z is offset in the XZ plane: the ends move 2 along X
    // and keep their Y and Z.
    SmTArray<SmPoint3d> sVertPts;
    for( int ii = 0; ii < 4; ii++ )
        sVertPts.Add( SmPoint3d(0.0, 0.0, ii * 10.0) );
    SmBSplineCurve* pVert = NULL;
    SER( SmApiCreateCurve(sVertPts, pVert) );
    SmBSplineCurve* pVertOffset = NULL;
    stat = SmApiOffsetCurve(pVert, 2.0, pVertOffset);
    delete pVert;
    if( stat != SM_SUCCESS || pVertOffset == NULL )
        return SM_ERR;
    pVertOffset->GetEnds( sOffStart, sOffEnd );
    delete pVertOffset;
    if( fabs(fabs(sOffStart.x) - 2.0) > 1.0e-6 || fabs(sOffStart.y) > 1.0e-9 || fabs(sOffStart.z) > 1.0e-6
        || fabs(fabs(sOffEnd.x) - 2.0) > 1.0e-6 || fabs(sOffEnd.y) > 1.0e-9 || fabs(sOffEnd.z - 30.0) > 1.0e-6 )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmDropCurveToSrf()
{

    SmApiCreateContext();

    SmTArray<SmPoint3d> sPoints;
    SmVector3d sPoint1(-40.0,0.0,0.0);
    sPoints.Add( sPoint1 );
    SmVector3d sPoint2(-30.0,-3.0,0.0);
    sPoints.Add( sPoint2 );
    SmVector3d sPoint3(-20.0,3.0,0.0);
    sPoints.Add( sPoint3 );
    SmVector3d sPoint4(-10.0,-6.0,0.0);
    sPoints.Add( sPoint4 );
   
    SmBSplineCurve* pCurve = NULL;
    long stat = SmApiCreateCurve(sPoints, pCurve);
   
    
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

    SmSurface* pSurface = NULL;
    SmApiCreateSurface(rControlPointsList, 5, 7, pSurface);

    SmTArray<SmBSplineCurve*> sArrayCurves2d;
    SmTArray<SmBSplineCurve*> sArrayCurves3d;

    stat = SmApiDropCurveToSrf(pCurve, pSurface, &sArrayCurves2d, &sArrayCurves3d);
    if( stat != SM_SUCCESS )
        return( stat );
    const ULONG lNumDropped = sArrayCurves2d.GetSize();
    if( lNumDropped == 0 || sArrayCurves3d.GetSize() != lNumDropped )
        return SM_ERR;

    // 3D output without 2D output used to dereference the NULL 2D array.
    SmTArray<SmBSplineCurve*> sOnly3d;
    SER( SmApiDropCurveToSrf(pCurve, pSurface, NULL, &sOnly3d) );
    if( sOnly3d.GetSize() != lNumDropped )
        return SM_ERR;

    // Non-empty output arrays are appended to; the lift loop used to index the
    // internal UV curves by the caller's 2D array size.
    SER( SmApiDropCurveToSrf(pCurve, pSurface, &sArrayCurves2d, &sArrayCurves3d) );
    if( sArrayCurves2d.GetSize() != 2 * lNumDropped || sArrayCurves3d.GetSize() != 2 * lNumDropped )
        return SM_ERR;
    for( ULONG ii = 0; ii < sArrayCurves3d.GetSize(); ii++ )
    {
        if( sArrayCurves3d[ii] == NULL )
            return SM_ERR;
        for( ULONG jj = 0; jj < ii; jj++ )
            if( sArrayCurves3d[jj] == sArrayCurves3d[ii] )
                return SM_ERR;
    }

    SmTArray<SmBSplineCurve*> sUnused;
    if( SmApiDropCurveToSrf(NULL, pSurface, &sUnused, &sUnused) != SM_ERR_INVALID_INPUT
        || SmApiDropCurveToSrf(pCurve, NULL, &sUnused, &sUnused) != SM_ERR_INVALID_INPUT )
        return SM_ERR;

    return( SM_SUCCESS );
}


//*************************************************************************
// 
//*************************************************************************


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreateRectangle()
{
    SmApiCreateContext();

    SmPoint3d sCenter(0.0, 0.0, 0.0);
    SmTArray<SmBSplineCurve*> sResult;
    SmStatus stat = SmApiCreateRectangle(sCenter, 10.0, 6.0, sResult);
    if (stat != SM_SUCCESS)
        return stat;
    if (sResult.GetSize() != 4)
        return SM_ERR;

    // Far from the origin a 1e-9 width rounds to zero: the first (length)
    // segment is built, then the width segment fails. The failed call must
    // leave the caller's array as it was (still the 4 segments above).
    SmPoint3d sFarCenter(0.0, 1.0e10, 0.0);
    if (SmApiCreateRectangle(sFarCenter, 10.0, 1.0e-9, sResult) == SM_SUCCESS)
        return SM_ERR;
    if (sResult.GetSize() != 4)
        return SM_ERR;

    // NaN and infinite sizes are rejected instead of building NaN segments.
    const double dNaN = std::numeric_limits<double>::quiet_NaN();
    const double dInf = std::numeric_limits<double>::infinity();
    if (   SmApiCreateRectangle(sCenter, dNaN, 6.0, sResult) != SM_ERR_INVALID_INPUT
        || SmApiCreateRectangle(sCenter, 10.0, dInf, sResult) != SM_ERR_INVALID_INPUT)
        return SM_ERR;
    if (sResult.GetSize() != 4)
        return SM_ERR;

    for (ULONG ii = 0; ii < sResult.GetSize(); ii++)
        delete sResult[ii];
    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreateRegularPolygon()
{
    SmApiCreateContext();

    SmPoint3d sCenter(0.0, 0.0, 0.0);
    SmTArray<SmBSplineCurve*> sResult;
    SmStatus stat = SmApiCreateRegularPolygon(sCenter, 6, 5.0, sResult);
    if (stat != SM_SUCCESS)
        return stat;
    if (sResult.GetSize() != 6)
        return SM_ERR;

    // Far from the origin a tiny radius makes some edges collapse in double
    // precision: this pentagon builds two segments, then the third fails.
    // The failed call must leave the caller's array as it was.
    SmPoint3d sFarCenter(1.0e10, 1.0e10, 0.0);
    if (SmApiCreateRegularPolygon(sFarCenter, 5, 1.5e-6, sResult) == SM_SUCCESS)
        return SM_ERR;
    if (sResult.GetSize() != 6)
        return SM_ERR;

    // NaN and infinite radii are rejected instead of building NaN segments.
    if (   SmApiCreateRegularPolygon(sCenter, 6, std::numeric_limits<double>::quiet_NaN(), sResult) != SM_ERR_INVALID_INPUT
        || SmApiCreateRegularPolygon(sCenter, 6, std::numeric_limits<double>::infinity(), sResult) != SM_ERR_INVALID_INPUT)
        return SM_ERR;
    if (sResult.GetSize() != 6)
        return SM_ERR;

    for (ULONG ii = 0; ii < sResult.GetSize(); ii++)
        delete sResult[ii];
    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmProjectCurveToSurface()
{
    SmApiCreateContext();

    SmTArray<SmPoint3d> sPts;
    sPts.Add(SmPoint3d(1.0, 1.0, 2.0));
    sPts.Add(SmPoint3d(3.0, 5.0, 2.0));
    sPts.Add(SmPoint3d(7.0, 3.0, 2.0));
    sPts.Add(SmPoint3d(9.0, 8.0, 2.0));

    SmBSplineCurve* pCurve = NULL;
    SmApiCreateCurve(sPts, pCurve);
    if (!pCurve)
        return SM_ERR;

    SmVector3d c1(0,0,0), c2(10,0,0), c3(0,10,0), c4(10,10,0);
    SmSurface* pSrf = NULL;
    SmApiCreateSurfaceFromCornerPoints(c1, c2, c3, c4, pSrf);
    if (!pSrf)
        return SM_ERR;

    SmTArray<SmBSplineCurve*> sCurves2d, sCurves3d;
    SmStatus stat = SmApiProjectCurve(pCurve, pSrf, &sCurves2d, &sCurves3d);
    if (stat != SM_SUCCESS)
        return stat;
    if (sCurves3d.GetSize() < 1)
        return SM_ERR;

    // 3D output only; outputs are appended to, one 3D curve per UV curve.
    const ULONG lNum = sCurves2d.GetSize();
    SmTArray<SmBSplineCurve*> sOnly3d;
    SER( SmApiProjectCurve(pCurve, pSrf, NULL, &sOnly3d) );
    if (sOnly3d.GetSize() != lNum || sCurves3d.GetSize() != lNum)
        return SM_ERR;
    SER( SmApiProjectCurve(pCurve, pSrf, &sCurves2d, &sCurves3d) );
    if (sCurves2d.GetSize() != 2 * lNum || sCurves3d.GetSize() != 2 * lNum)
        return SM_ERR;

    SmTArray<SmBSplineCurve*> sUnused;
    if (SmApiProjectCurve(NULL, pSrf, &sUnused, &sUnused) != SM_ERR_INVALID_INPUT
        || SmApiProjectCurve(pCurve, NULL, &sUnused, &sUnused) != SM_ERR_INVALID_INPUT)
        return SM_ERR;

    // A curve above the surface that crosses its boundary must project to the
    // part inside the surface. With the "curve is on the surface" kernel flag
    // both functions returned no curves at all for this case.
    SmTArray<SmPoint3d> sCrossPts;
    for (int ii = 0; ii < 8; ii++)
    {
        double dX = -5.0 + 20.0 * ii / 7.0;
        sCrossPts.Add(SmPoint3d(dX, 5.0 + 1.5 * sin(dX), 0.5));
    }
    SmBSplineCurve* pCross = NULL;
    SER( SmApiCreateCurve(sCrossPts, pCross) );
    for (int iFunc = 0; iFunc < 2; iFunc++)
    {
        SmTArray<SmBSplineCurve*> sCross3d;
        stat = (iFunc == 0) ? SmApiProjectCurve(pCross, pSrf, NULL, &sCross3d)
                            : SmApiDropCurveToSrf(pCross, pSrf, NULL, &sCross3d);
        if (stat != SM_SUCCESS || sCross3d.GetSize() != 1)
            return SM_ERR;
        SmExtent1d sIvl = sCross3d[0]->GetNaturalInterval();
        SmPoint3d sStart, sEnd;
        sCross3d[0]->EvaluatePoint(sIvl.GetMin(), sStart);
        sCross3d[0]->EvaluatePoint(sIvl.GetMax(), sEnd);
        if (fabs(sStart.x) > 1.0e-3 || sEnd.x < 9.5 || sEnd.x > 10.0 + 1.0e-3
            || fabs(sStart.z) > 1.0e-6 || fabs(sEnd.z) > 1.0e-6)
            return SM_ERR;
        delete sCross3d[0];
    }
    delete pCross;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmLiftUVCurve()
{
    SmApiCreateContext();

    SmVector3d c1(0,0,0), c2(10,0,0), c3(0,10,0), c4(10,10,0);
    SmSurface* pSrf = NULL;
    SmApiCreateSurfaceFromCornerPoints(c1, c2, c3, c4, pSrf);
    if (!pSrf)
        return SM_ERR;

    SmTArray<SmPoint3d> sPts;
    sPts.Add(SmPoint3d(1.0, 1.0, 0.0));
    sPts.Add(SmPoint3d(5.0, 8.0, 0.0));
    sPts.Add(SmPoint3d(9.0, 5.0, 0.0));
    sPts.Add(SmPoint3d(9.0, 1.0, 0.0));

    SmBSplineCurve* pCurve3d = NULL;
    SmApiCreateCurve(sPts, pCurve3d);
    if (!pCurve3d)
        return SM_ERR;

    SmTArray<SmBSplineCurve*> sCurvesUV;
    SmStatus stat = SmApiProjectCurve(pCurve3d, pSrf, &sCurvesUV, NULL);
    if (stat != SM_SUCCESS || sCurvesUV.GetSize() < 1)
        return SM_ERR;

    SmTArray<SmBSplineCurve*> sLifted3d;
    stat = SmApiLiftUVCurveFromSrf(sCurvesUV[0], pSrf, &sLifted3d);
    if (stat != SM_SUCCESS)
        return stat;
    if (sLifted3d.GetSize() < 1)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmMakeCurvesCompatible()
{
    SmApiCreateContext();

    SmBSplineCurve* pCircle1 = NULL;
    SmBSplineCurve* pCircle2 = NULL;
    SmPoint3d sC1(0,0,0), sC2(0,0,5);
    SmApiCreateCircle(sC1, 5.0, pCircle1);
    SmApiCreateCircle(sC2, 3.0, pCircle2);
    if (!pCircle1 || !pCircle2)
        return SM_ERR;

    SmTArray<SmBSplineCurve*> sCurves;
    sCurves.Add(pCircle1);
    sCurves.Add(pCircle2);

    SmStatus stat = SmApiMakeCurvesCompatible(sCurves);
    return stat;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmOrderCurves()
{
    SmApiCreateContext();

    SmPoint3d sCenter(0.0, 0.0, 0.0);
    SmTArray<SmBSplineCurve*> sRectSegments;
    SmApiCreateRectangle(sCenter, 10.0, 6.0, sRectSegments);
    if (sRectSegments.GetSize() != 4)
        return SM_ERR;

    SmStatus stat = SmApiOrderCurves(sRectSegments);
    if (stat != SM_SUCCESS)
        return stat;
    if (sRectSegments.GetSize() != 4)
        return SM_ERR;

    return SM_SUCCESS;
}


//*************************************************************************
// Every out-of-domain create_helix input must be rejected with the exact
// status SM_ERR_INVALID_INPUT, and the output pointer must be left NULL
// (rpResult is only valid on SM_SUCCESS). Python cannot observe the output
// pointer on the failure path, so that half of the contract is asserted here:
// the output is seeded with a non-null sentinel before every call.
//*************************************************************************
SmStatus TestSmCreateHelixInvalidInputs()
{
    SmApiCreateContext();

    const double kInf     = std::numeric_limits<double>::infinity();
    const double kNaN     = std::numeric_limits<double>::quiet_NaN();
    const double kSub     = std::numeric_limits<double>::denorm_min();
    const double kDblMax  = DBL_MAX;
    const double kMaxMag  = 1.0e18;         // SM_IS_VALID_DOUBLE finite cap
    const double kMaxTurns = 994.0 / 8.0;   // (NL_CCPLIM - 6) / 8 == 124.25
    const double kMinTurns = 1.0e-7;
    const double kMinTol   = 1.0e-6;

    struct HelixCase
    {
        const char* pLabel;
        double ox, oy, oz;               // origin
        double h, r0, r1, turns, tol;    // scalars
    };

    // Base valid values are {0,0,0}, h=10, r0=r1=2, turns=5, tol=1e-3; each row
    // overrides exactly the field(s) it exercises.
    const std::vector<HelixCase> sCases = {
        // NaN / +/-Inf on every origin coordinate.
        { "origin.x=+inf",  kInf, 0, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.x=-inf", -kInf, 0, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.x=nan",   kNaN, 0, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.y=+inf",  0, kInf, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.y=-inf",  0,-kInf, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.y=nan",   0, kNaN, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.z=+inf",  0, 0, kInf, 10, 2, 2, 5, 1e-3 },
        { "origin.z=-inf",  0, 0,-kInf, 10, 2, 2, 5, 1e-3 },
        { "origin.z=nan",   0, 0, kNaN, 10, 2, 2, 5, 1e-3 },
        // NaN / +/-Inf on every scalar.
        { "height=+inf", 0, 0, 0,  kInf, 2, 2, 5, 1e-3 },
        { "height=-inf", 0, 0, 0, -kInf, 2, 2, 5, 1e-3 },
        { "height=nan",  0, 0, 0,  kNaN, 2, 2, 5, 1e-3 },
        { "r0=+inf", 0, 0, 0, 10,  kInf, 2, 5, 1e-3 },
        { "r0=-inf", 0, 0, 0, 10, -kInf, 2, 5, 1e-3 },
        { "r0=nan",  0, 0, 0, 10,  kNaN, 2, 5, 1e-3 },
        { "r1=+inf", 0, 0, 0, 10, 2,  kInf, 5, 1e-3 },
        { "r1=-inf", 0, 0, 0, 10, 2, -kInf, 5, 1e-3 },
        { "r1=nan",  0, 0, 0, 10, 2,  kNaN, 5, 1e-3 },
        { "turns=+inf", 0, 0, 0, 10, 2, 2,  kInf, 1e-3 },
        { "turns=-inf", 0, 0, 0, 10, 2, 2, -kInf, 1e-3 },
        { "turns=nan",  0, 0, 0, 10, 2, 2,  kNaN, 1e-3 },
        { "tol=+inf", 0, 0, 0, 10, 2, 2, 5,  kInf },
        { "tol=-inf", 0, 0, 0, 10, 2, 2, 5, -kInf },
        { "tol=nan",  0, 0, 0, 10, 2, 2, 5,  kNaN },
        // Zero / negative height and turns.
        { "height=0", 0, 0, 0,  0.0, 2, 2, 5, 1e-3 },
        { "height<0", 0, 0, 0, -1.0, 2, 2, 5, 1e-3 },
        { "turns=0",  0, 0, 0, 10, 2, 2,  0.0, 1e-3 },
        { "turns<0",  0, 0, 0, 10, 2, 2, -1.0, 1e-3 },
        // Radius contract: each single zero, both zero, and negatives rejected.
        { "r0=0",     0, 0, 0, 10, 0.0, 2,   5, 1e-3 },
        { "r1=0",     0, 0, 0, 10, 2,   0.0, 5, 1e-3 },
        { "both_r=0", 0, 0, 0, 10, 0.0, 0.0, 5, 1e-3 },
        { "r0<0",     0, 0, 0, 10, -1.0, 2,  5, 1e-3 },
        { "r1<0",     0, 0, 0, 10, 2, -1.0,  5, 1e-3 },
        // Subnormal values (below the 1e-7 / 1e-6 floors).
        { "height=sub", 0, 0, 0, kSub, 2, 2, 5, 1e-3 },
        { "r0=sub",     0, 0, 0, 10, kSub, 2, 5, 1e-3 },
        { "turns=sub",  0, 0, 0, 10, 2, 2, kSub, 1e-3 },
        { "tol=sub",    0, 0, 0, 10, 2, 2, 5, kSub },
        // Tolerance: zero, negative, and just below the 1e-6 floor.
        { "tol=0",  0, 0, 0, 10, 2, 2, 5,  0.0 },
        { "tol<0",  0, 0, 0, 10, 2, 2, 5, -1.0 },
        { "tol=just_below_min", 0, 0, 0, 10, 2, 2, 5, std::nextafter(kMinTol, 0.0) },
        { "tol=1e-9", 0, 0, 0, 10, 2, 2, 5, 1e-9 },
        // Turns just outside min/max, huge, and DBL_MAX.
        { "turns=just_below_min", 0, 0, 0, 10, 2, 2, std::nextafter(kMinTurns, 0.0), 1e-3 },
        { "turns=just_above_max", 0, 0, 0, 10, 2, 2, std::nextafter(kMaxTurns, kInf), 1e-3 },
        { "turns=125",     0, 0, 0, 10, 2, 2, 125.0,   1e-3 },
        { "turns=1e6",     0, 0, 0, 10, 2, 2, 1.0e6,   1e-3 },
        { "turns=DBL_MAX", 0, 0, 0, 10, 2, 2, kDblMax, 1e-3 },
        // Magnitudes just outside the finite cap (1e18) and at DBL_MAX; a finite
        // but huge origin whose derived extent overflows the cap.
        { "height=just_above_max", 0, 0, 0, std::nextafter(kMaxMag, kInf), 2, 2, 5, 1e-3 },
        { "height=1e19",    0, 0, 0, 1.0e19,  2, 2, 5, 1e-3 },
        { "height=DBL_MAX", 0, 0, 0, kDblMax, 2, 2, 5, 1e-3 },
        { "r0=1e19",        0, 0, 0, 10, 1.0e19,  2, 5, 1e-3 },
        { "r0=DBL_MAX",     0, 0, 0, 10, kDblMax, 2, 5, 1e-3 },
        { "tol=DBL_MAX",    0, 0, 0, 10, 2, 2, 5, kDblMax },
        { "origin.x=1e19",    1.0e19,  0, 0, 10, 2, 2, 5, 1e-3 },
        { "origin.x=DBL_MAX", kDblMax, 0, 0, 10, 2, 2, 5, 1e-3 },
    };

    // A valid, non-null address used to seed the output before every call; the
    // contract is that SmApiCreateHelix resets rpResult to NULL on failure.
    static char sSentinel = 0;
    SmBSplineCurve* const pSentinel = reinterpret_cast<SmBSplineCurve*>(&sSentinel);

    SmStatus rtn = SM_SUCCESS;
    for (size_t i = 0; i < sCases.size(); ++i)
    {
        const HelixCase& c = sCases[i];
        SmPoint3d sOrigin(c.ox, c.oy, c.oz);

        SmBSplineCurve* pOut = pSentinel;
        SmStatus stat = SmApiCreateHelix(sOrigin, c.h, c.r0, c.r1, c.turns,
                                         TRUE, c.tol, pOut);

        if (stat != SM_ERR_INVALID_INPUT || pOut != NULL)
        {
            fprintf(stderr,
                    "  TestSmCreateHelixInvalidInputs FAIL [%s]: status=%ld out=%p"
                    " (expected status=%ld out=NULL)\n",
                    c.pLabel, (long)stat, (void*)pOut, (long)SM_ERR_INVALID_INPUT);
            rtn = SM_ERR;
        }
    }

    return rtn;
}


namespace
{
// A valid-domain curve whose left or right evaluation fails. This distinguishes
// propagation of a kernel error from the API's invalid-input preflight.
class SmContinuityFailureLine : public SmLine
{
public:
    explicit SmContinuityFailureLine( SmBoolean bFailFromLeft )
        : SmLine(SmPoint3d(0, 0, 0), SmPoint3d(10, 0, 0), 3, SmApiGetOrCreateContext()),
          m_bFailFromLeft(bFailFromLeft)
    {
    }

    SmStatus Evaluate( double dParameter, ULONG lNumDerivatives, SmBoolean bFromLeft,
                       SmVector3d aPointAndDerivatives[], SmBoolean bNonZeroTangents ) const override
    {
        if( bFromLeft == m_bFailFromLeft )
            return SM_ERR_NOT_CONVERGING;
        return SmLine::Evaluate(dParameter, lNumDerivatives, bFromLeft,
                                aPointAndDerivatives, bNonZeroTangents);
    }

private:
    SmBoolean m_bFailFromLeft;
};
}

//*************************************************************************
// Output-pointer contract for invalid inputs: SmApiCreateCircle and
// SmApiCreateCurveInterpPoints must return SM_ERR_INVALID_INPUT and set the
// caller's output pointer to NULL even when it was non-NULL on entry. Python
// cannot observe the output-pointer contract, so it is verified here.
//*************************************************************************
SmStatus TestSmCreateCurveInvalidInputs()
{
    SmApiCreateContext();

    const double dNan = std::numeric_limits<double>::quiet_NaN();
    const double dInf = std::numeric_limits<double>::infinity();

    // Non-NULL sentinel: a distinct address, never a valid heap object.
    static char sSentinelByte = 0;
    SmCircle*       const pSentCircle  = reinterpret_cast<SmCircle*>(&sSentinelByte);
    SmBSplineCurve* const pSentBSpline = reinterpret_cast<SmBSplineCurve*>(&sSentinelByte);
    SmCurve*        const pSentCurve   = reinterpret_cast<SmCurve*>(&sSentinelByte);

    struct CircleCase { double x, y, z, r; };
    const CircleCase kCircleCases[] = {
        { dNan, 0.0, 0.0, 5.0 },
        { dInf, 0.0, 0.0, 5.0 },
        { 0.0, dNan, 0.0, 5.0 },
        { 0.0, 0.0, dInf, 5.0 },
        { 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, -5.0 },
        { 0.0, 0.0, 0.0, dNan },
        { 0.0, 0.0, 0.0, dInf },
        // Exact minimum-radius boundary: SmCircle::CreateCanonical requires
        // radius > SM_EFF_ZERO, so radius == SM_EFF_ZERO and anything below it
        // must be rejected as invalid input (not a generic kernel SM_ERR).
        { 0.0, 0.0, 0.0, SM_EFF_ZERO },
        { 0.0, 0.0, 0.0, 0.5 * SM_EFF_ZERO },
        // Large-center / small-radius: the extents collapse onto the center in
        // double precision (1e18 + 1 == 1e18), which would otherwise yield a
        // radius()==1 circle with length()==0.
        { 1.0e18, 0.0, 0.0, 1.0 },
        { 1.0e18, 0.0, 0.0, 1.0e-6 },
        { 0.0, 1.0e18, 0.0, 1.0 },
        // Center coordinate outside the SM_IS_VALID_DOUBLE range [-1e18, 1e18].
        { 2.0e18, 0.0, 0.0, 5.0 },
    };

    for (unsigned i = 0; i < sizeof(kCircleCases)/sizeof(kCircleCases[0]); i++)
    {
        SmPoint3d sCenter(kCircleCases[i].x, kCircleCases[i].y, kCircleCases[i].z);
        double dRadius = kCircleCases[i].r;

        SmCircle* pCircle = pSentCircle;
        if (SmApiCreateCircle(sCenter, dRadius, pCircle) != SM_ERR_INVALID_INPUT)
            return SM_ERR;
        if (pCircle != NULL)
            return SM_ERR;

        SmBSplineCurve* pBSpline = pSentBSpline;
        if (SmApiCreateCircle(sCenter, dRadius, pBSpline) != SM_ERR_INVALID_INPUT)
            return SM_ERR;
        if (pBSpline != NULL)
            return SM_ERR;
    }

    // Positive boundary: a radius just above SM_EFF_ZERO at a small center must
    // still succeed, so the minimum-radius guard does not over-reject valid
    // small circles. Both overloads must accept it.
    {
        SmPoint3d sCenter(0.0, 0.0, 0.0);
        const double dJustAbove = 2.0 * SM_EFF_ZERO;

        SmCircle* pCircle = NULL;
        if (SmApiCreateCircle(sCenter, dJustAbove, pCircle) != SM_SUCCESS)
            return SM_ERR;
        if (pCircle == NULL)
            return SM_ERR;

        SmBSplineCurve* pBSpline = NULL;
        if (SmApiCreateCircle(sCenter, dJustAbove, pBSpline) != SM_SUCCESS)
            return SM_ERR;
        if (pBSpline == NULL)
            return SM_ERR;
    }

    // Interpolating curve: fewer than 4 points, and a non-finite coordinate.
    {
        SmTArray<SmPoint3d> sFew;
        sFew.Add(SmPoint3d(0, 0, 0));
        sFew.Add(SmPoint3d(1, 1, 0));
        sFew.Add(SmPoint3d(2, 0, 0));   // only 3 points

        SmCurve* pResult = pSentCurve;
        if (SmApiCreateCurveInterpPoints(sFew, pResult) != SM_ERR_INVALID_INPUT)
            return SM_ERR;
        if (pResult != NULL)
            return SM_ERR;

        SmTArray<SmPoint3d> sNan;
        sNan.Add(SmPoint3d(0, 0, 0));
        sNan.Add(SmPoint3d(1, 1, 0));
        sNan.Add(SmPoint3d(dNan, 0, 0));
        sNan.Add(SmPoint3d(3, 1, 0));

        pResult = pSentCurve;
        if (SmApiCreateCurveInterpPoints(sNan, pResult,
                                         SM_CP_CENTRIPETAL) != SM_ERR_INVALID_INPUT)
            return SM_ERR;
        if (pResult != NULL)
            return SM_ERR;
    }

    // Interpolating curve: unknown parameterization enum values (raw C++ casts
    // below and above the valid SM_CP_* range) must be rejected. The kernel
    // silently maps unknown values to NL_UNIFORM, so this contract is enforced
    // at the SM_API boundary rather than in the kernel.
    {
        SmTArray<SmPoint3d> sPts;
        sPts.Add(SmPoint3d(0, 0, 0));
        sPts.Add(SmPoint3d(1, 1, 0));
        sPts.Add(SmPoint3d(2, 0, 0));
        sPts.Add(SmPoint3d(3, 1, 0));

        const SmCurveParameterizationType kBadParams[] = {
            static_cast<SmCurveParameterizationType>(-1),   // below range
            static_cast<SmCurveParameterizationType>(3),    // just above range
            static_cast<SmCurveParameterizationType>(999),  // far above range
        };

        for (unsigned i = 0; i < sizeof(kBadParams)/sizeof(kBadParams[0]); i++)
        {
            SmCurve* pResult = pSentCurve;
            if (SmApiCreateCurveInterpPoints(sPts, pResult, kBadParams[i]) != SM_ERR_INVALID_INPUT)
                return SM_ERR;
            if (pResult != NULL)
                return SM_ERR;
        }
    }

    // Distance-based modes reject coincident successive points as
    // SM_ERR_INVALID_INPUT; UNIFORM must still accept the same input.
    {
        // Single coincident pair (points 1 and 2 identical).
        SmTArray<SmPoint3d> sDup;
        sDup.Add(SmPoint3d(0, 0, 0));
        sDup.Add(SmPoint3d(1, 1, 0));
        sDup.Add(SmPoint3d(1, 1, 0));
        sDup.Add(SmPoint3d(3, 1, 0));

        // Pair separated by less than SM_EFF_ZERO counts as coincident.
        SmTArray<SmPoint3d> sNearDup;
        sNearDup.Add(SmPoint3d(0, 0, 0));
        sNearDup.Add(SmPoint3d(1, 1, 0));
        sNearDup.Add(SmPoint3d(1, 1, 0.5 * SM_EFF_ZERO));
        sNearDup.Add(SmPoint3d(3, 1, 0));

        // Fully degenerate: all points coincident.
        SmTArray<SmPoint3d> sAllSame;
        for (int i = 0; i < 4; i++)
            sAllSame.Add(SmPoint3d(2, 2, 2));

        const SmCurveParameterizationType kDistModes[] = {
            SM_CP_CHORDLENGTH,
            SM_CP_CENTRIPETAL,
        };

        for (unsigned m = 0; m < sizeof(kDistModes)/sizeof(kDistModes[0]); m++)
        {
            SmTArray<SmPoint3d>* const kBadPtSets[] = { &sDup, &sNearDup, &sAllSame };
            for (unsigned s = 0; s < sizeof(kBadPtSets)/sizeof(kBadPtSets[0]); s++)
            {
                SmCurve* pResult = pSentCurve;
                if (SmApiCreateCurveInterpPoints(*kBadPtSets[s], pResult,
                                                 kDistModes[m]) != SM_ERR_INVALID_INPUT)
                    return SM_ERR;
                if (pResult != NULL)
                    return SM_ERR;
            }
        }

        // UNIFORM must not over-reject the duplicated input.
        SmCurve* pUniform = NULL;
        if (SmApiCreateCurveInterpPoints(sDup, pUniform, SM_CP_UNIFORM) != SM_SUCCESS)
            return SM_ERR;
        if (pUniform == NULL)
            return SM_ERR;
    }

    // SmApiEvaluateContinuity rejects a NULL curve, a NaN parameter (a NaN
    // used to slip past the domain check and reach the kernel) and a parameter
    // outside the domain as invalid input.
    {
        SmPoint3d sStart(0.0, 0.0, 0.0), sEnd(10.0, 0.0, 0.0);
        SmLine* pLine = NULL;
        if (SmApiCreateLineSegment(sStart, sEnd, pLine) != SM_SUCCESS || pLine == NULL)
            return SM_ERR;
        SmContinuityType eType = SM_CT_UNDEFINED;
        SmStatus eMid = SmApiEvaluateContinuity(pLine, 0.5, eType);
        SmStatus eNan = SmApiEvaluateContinuity(pLine, dNan, eType);
        SmStatus eNull = SmApiEvaluateContinuity(NULL, 0.5, eType);
        SmStatus ePast = SmApiEvaluateContinuity(pLine, pLine->GetNaturalInterval().GetMax() + 1.0, eType);
        delete pLine;
        if (   eMid != SM_SUCCESS || eNan != SM_ERR_INVALID_INPUT
            || eNull != SM_ERR_INVALID_INPUT || ePast != SM_ERR_INVALID_INPUT)
            return SM_ERR;
    }

    // Fail each side independently at a valid parameter. Ignoring the kernel
    // status used to report success with no continuity result in both cases.
    const SmBoolean aFailFromLeft[] = { TRUE, FALSE };
    for( SmBoolean bFailFromLeft : aFailFromLeft )
    {
        SmContinuityFailureLine sCurve(bFailFromLeft);
        SmContinuityType eType = SM_CT_UNDEFINED;
        if( SmApiEvaluateContinuity(&sCurve, 0.5, eType) != SM_ERR_NOT_CONVERGING )
            return SM_ERR;
    }

    return SM_SUCCESS;
}


SmStatus TestSmRemoveCurveKnots()
{
    SmApiCreateContext();

    SmTArray<SmPoint3d> sPts;
    sPts.Add(SmPoint3d(0.0, 0.0, 0.0));
    sPts.Add(SmPoint3d(1.0, 0.0, 0.0));
    sPts.Add(SmPoint3d(2.0, 0.0, 0.0));

    SmTArray<double> knots;
    knots.Add(0.0);
    knots.Add(0.5);
    knots.Add(1.0);
    SmTArray<ULONG> multiplicities;
    multiplicities.Add(2);
    multiplicities.Add(1);
    multiplicities.Add(2);

    SmBSplineCurve* pCurve = NULL;
    SER(SmApiCreateCanonicalCurve(sPts, knots, multiplicities, NULL, 1, SM_CF_UNSPECIFIED, pCurve));
    if (!pCurve)
        return SM_ERR;

    SmStatus stat = SmApiRemoveCurveKnots(pCurve);
    SmTArray<double> remainingKnots;
    if (stat == SM_SUCCESS)
        stat = pCurve->GetKnotsAll(remainingKnots);
    // A degree-one line needs only its four clamped endpoint knots.
    if (stat == SM_SUCCESS && remainingKnots.GetSize() != 4)
        stat = SM_ERR;
    delete pCurve;
    return stat;
}


SmStatus TestSmCurves()
{

    SmStatus stat = TestSmCreateLineSegment();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateCircle();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateArc();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateCurve();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateCanonicalCurve();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmOffsetCurve();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmIntersectCurves();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmDropCurveToSrf();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateRectangle();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateRegularPolygon();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmProjectCurveToSurface();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmLiftUVCurve();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmMakeCurvesCompatible();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmOrderCurves();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmRemoveCurveKnots();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateHelixInvalidInputs();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreateCurveInvalidInputs();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}
