// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*******************************************************************
// Undistributed, user managed test file
// You should no longer need to modify any files under source 
// control to get testing flexibility
// The only thing that cannot change is the Test() function signature
//*******************************************************************

#include "StdAfx.h"

#include <SmMessages.h>
#include <SmApiGeneral.h>

#include <SmApiCurves.h>
#include <SmApiSurfaces.h>
#include <SmApiTrimmedSurfaces.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmBrep.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmSurface.h>


//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmTrimSurfaceWith3dCurves()
{
    
    //SmApiCreateContext();

    SmPoint3d c1( 0,0,0 );
    SmPoint3d c2( 10,0,0 );
    SmPoint3d c3( 10,10,0 );
    SmPoint3d c4( 0,10,0 );

    SmSurface* pSurface = NULL;
    SER( SmApiCreateSurfaceFromCornerPoints(c1, c2, c4, c3, pSurface) );

 
    SmTArray<SmCurve*> sTrimmingCurves;

    SmPoint3d p0_0(0,0,0), p10_0(10,0,0), p10_5(10,5,0), p5_5(5,5,0);
    SmPoint3d p5_10(5,10,0), p0_10(0,10,0);
    SmPoint3d p1_5(1,5,0), p1_8(1,8,0), p4_5(4,5,0);
    SmPoint3d p6_25(6,2.5,0);

    SmLine* pLine = NULL ;
    SER( SmApiCreateLineSegment(p0_0, p10_0, pLine) );
    sTrimmingCurves.Add( pLine );
               
    SER( SmApiCreateLineSegment( p10_0, p10_5, pLine) );
    sTrimmingCurves.Add( pLine );

    SER( SmApiCreateLineSegment( p10_5, p5_5, pLine) );
    sTrimmingCurves.Add( pLine );

    SER( SmApiCreateLineSegment( p5_5, p5_10, pLine) );
    sTrimmingCurves.Add( pLine );

    SER( SmApiCreateLineSegment( p5_10, p0_10, pLine) );
    sTrimmingCurves.Add( pLine );
 
    SER( SmApiCreateLineSegment( p0_10, p0_0, pLine) );
    sTrimmingCurves.Add( pLine );
 
    
    // Now create first inner loop - make it a clockwise list of 3
    // lines
    SmApiCreateLineSegment( p1_5, p1_8, pLine);
    sTrimmingCurves.Add( pLine );

    SmApiCreateLineSegment( p1_8, p4_5, pLine);
    sTrimmingCurves.Add( pLine );

    SmApiCreateLineSegment( p4_5, p1_5, pLine);
    sTrimmingCurves.Add( pLine );

    SmBSplineCurve* pCircle = NULL;
    SmApiCreateCircle( p6_25, 1.5, pCircle);
    sTrimmingCurves.Add( pCircle );

    //sCurveOrients.Add(SM_OT_OPPOSITE);  // Curve orientation opposite of loop

    // Now set up loops - count of the curve number in each loop
    SmTArray<ULONG> sLoops;
    sLoops.Add(6);  // 6 curves in outer loop
    sLoops.Add(3);  // 3 curves in first inner loop
    sLoops.Add(1);  // 1 curve in last inner loop

    SmBrep* pResult = NULL;
    SER( SmApiTrimSurfaceWith3dCurves( pSurface, sLoops, sTrimmingCurves, pResult) );

    SmObjDelete sResultCleanup( pResult );
    SmTArray<SmFace*> sFaces;
    SER( SmApiGetFaces( pResult, sFaces ) );
    if( sFaces.GetSize() != 1 )
        return SM_ERR;

    SmFace* pFace = sFaces[0];
    SmTArray<ULONG> sEdgeCountsPerLoop;
    SmTArray<SmEdge*> sBoundaryEdges;
    SmTArray<SmOrientType> sBoundaryOrientations;
    SER( SmApiFaceGetBoundaryLoops(
        pFace, sEdgeCountsPerLoop, sBoundaryEdges, sBoundaryOrientations ) );

    if(   sEdgeCountsPerLoop.GetSize() != 3
       || sEdgeCountsPerLoop[0] != 6
       || sEdgeCountsPerLoop[1] != 3
       || sEdgeCountsPerLoop[2] != 1
       || sBoundaryEdges.GetSize() != 10
       || sBoundaryOrientations.GetSize() != sBoundaryEdges.GetSize() )
        return SM_ERR;

    SmTArray<SmLoop*> sOuterLoops;
    SmTArray<SmEdge*> sOuterEdges;
    SmTArray<SmVertex*> sOuterVertices;
    pFace->GetOuterLoops( sOuterLoops );
    pFace->GetOuterLoopEdges( sOuterEdges );
    pFace->GetOuterLoopVertices( sOuterVertices );
    if(   sOuterLoops.GetSize() != 1
       || sOuterEdges.GetSize() != 6
       || sOuterVertices.GetSize() != 6 )
        return SM_ERR;

    ULONG lEdgeIndex = 0;
    for( ULONG ii = 0; ii < sEdgeCountsPerLoop.GetSize(); ii++ )
    {
        ULONG lLoopStart = lEdgeIndex;
        ULONG lLoopEdgeCount = sEdgeCountsPerLoop[ii];
        for( ULONG jj = 0; jj < lLoopEdgeCount; jj++, lEdgeIndex++ )
        {
            SmEdge* pEdge = sBoundaryEdges[lEdgeIndex];
            SmOrientType eOrientation = sBoundaryOrientations[lEdgeIndex];
            if(   pEdge == NULL
               || pEdge->GetBrep() != pResult
               || (eOrientation != SM_OT_SAME && eOrientation != SM_OT_OPPOSITE) )
                return SM_ERR;

            ULONG lNextIndex = lLoopStart + ((jj + 1) % lLoopEdgeCount);
            SmEdge* pNextEdge = sBoundaryEdges[lNextIndex];
            if( pNextEdge == NULL )
                return SM_ERR;

            SmOrientType eNextOrientation = sBoundaryOrientations[lNextIndex];
            SmVertex* pEnd = eOrientation == SM_OT_SAME
                ? pEdge->GetEndVertex()
                : pEdge->GetStartVertex();
            SmVertex* pNextStart = eNextOrientation == SM_OT_SAME
                ? pNextEdge->GetStartVertex()
                : pNextEdge->GetEndVertex();
            if( pEnd == NULL || pEnd != pNextStart )
                return SM_ERR;
        }
    }

    if( lEdgeIndex != sBoundaryEdges.GetSize() )
        return SM_ERR;

    // Failure clears every parallel output, so callers never observe a
    // partially valid grouping.
    if(   SmApiFaceGetBoundaryLoops(
              NULL, sEdgeCountsPerLoop, sBoundaryEdges, sBoundaryOrientations ) != SM_ERR_INVALID_INPUT
       || sEdgeCountsPerLoop.GetSize() != 0
       || sBoundaryEdges.GetSize() != 0
       || sBoundaryOrientations.GetSize() != 0 )
        return SM_ERR;
 
    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmTrimProjectParallel()
{
    
    SmApiCreateContext();

    SmPoint3d c1( 0,0,0 );
    SmPoint3d c2( 25,0,0 );
    SmPoint3d c3( 25,25,0 );
    SmPoint3d c4( 0,25,0 );

    SmSurface* pSurface = NULL;
    SER( SmApiCreateSurfaceFromCornerPoints(c1, c2, c4, c3, pSurface) );

    SmVector3d sDir( 0,0,-1 );
    SmPoint3d sRefPt( 5,5,0 );

    // A surface owned by a Brep cannot be consumed independently without
    // invalidating its owning topology.
    SmBrep* pBox = NULL;
    SER( SmApiCreateBox(c1, 10, 10, 10, pBox) );
    SmObjDelete sBoxCleanup(pBox);

    SmTArray<SmFace*> sFaces;
    SER( SmApiGetFaces(pBox, sFaces) );
    if( sFaces.GetSize() == 0 || sFaces[0]->GetSurface() == NULL )
        return SM_ERR;

    SmFace* pOwnedFace = sFaces[0];
    SmSurface* pOwnedSurface = pOwnedFace->GetSurface();
    SmTArray<SmEdge*> sEdgesBefore;
    SmTArray<SmVertex*> sVerticesBefore;
    SER( SmApiGetEdges(pBox, sEdgesBefore) );
    pBox->GetVertices(sVerticesBefore);
    SmExtent2d sOwnedDomain = pOwnedSurface->GetNaturalUVDomain();
    SmVector2d sOwnedUV(0.5*(sOwnedDomain.GetMin().x+sOwnedDomain.GetMax().x),
                        0.5*(sOwnedDomain.GetMin().y+sOwnedDomain.GetMax().y));
    SmPoint3d sOwnedPointBefore;
    SER( SmApiEvaluateSurfacePoint(pOwnedSurface, sOwnedUV, sOwnedPointBefore) );

    double dBoxVolumeBefore = 0.0;
    SER( SmApiBrepComputeVolume(pBox, 1.0e-4, dBoxVolumeBefore) );

    SmLine* pOwnedTestLine = NULL;
    SmPoint3d sOwnedLP1(-5,5,15), sOwnedLP2(15,5,15);
    SER( SmApiCreateLineSegment(sOwnedLP1, sOwnedLP2, pOwnedTestLine) );
    SmObjDelete sOwnedLineCleanup(pOwnedTestLine);

    SmBrep* pRejected = pBox;
    SmStatus eOwnedStatus = SmApiTrimProjectParallel(
        pOwnedSurface, pOwnedTestLine, sDir, sRefPt,
        SM_TT_KEEP_POINT, pRejected);
    if( eOwnedStatus != SM_ERR_INVALID_INPUT || pRejected != NULL ) {
        // A regression may already have moved the surface into another Brep.
        // Avoid deleting an ambiguously shared topology graph while reporting it.
        sBoxCleanup.Clear();
        return SM_ERR;
    }

    SmPoint3d sOwnedPointAfter;
    SER( SmApiEvaluateSurfacePoint(pOwnedSurface, sOwnedUV, sOwnedPointAfter) );
    double dBoxVolumeAfter = 0.0;
    SER( SmApiBrepComputeVolume(pBox, 1.0e-4, dBoxVolumeAfter) );
    SmTArray<SmFace*> sFacesAfter;
    SmTArray<SmEdge*> sEdgesAfter;
    SmTArray<SmVertex*> sVerticesAfter;
    SER( SmApiGetFaces(pBox, sFacesAfter) );
    SER( SmApiGetEdges(pBox, sEdgesAfter) );
    pBox->GetVertices(sVerticesAfter);
    if( sOwnedPointAfter.DistanceBetween(sOwnedPointBefore) > 1.0e-8 ||
        fabs(dBoxVolumeAfter-dBoxVolumeBefore) > 1.0e-8 ||
        pOwnedSurface->GetOwner() != pOwnedFace ||
        pOwnedFace->GetSurface() != pOwnedSurface ||
        pOwnedFace->GetBrep() != pBox ||
        sFacesAfter.GetSize() != sFaces.GetSize() ||
        sEdgesAfter.GetSize() != sEdgesBefore.GetSize() ||
        sVerticesAfter.GetSize() != sVerticesBefore.GetSize() ||
        !pBox->AssertValid() )
        return SM_ERR;

    // A kernel trim failure reports the kernel's status, clears the output,
    // and leaves the Brep input with the caller.
    SmVector3d sZeroDir( 0,0,0 );
    SmBrep* pFailed = pBox;
    SmStatus eTrimStatus = SmApiTrimProjectParallel(
        pBox, pOwnedTestLine, sZeroDir, sRefPt, SM_TT_KEEP_POINT, pFailed);
    if( eTrimStatus != SM_ERR_INVALID_INPUT || pFailed != NULL || !pBox->AssertValid() )
        return SM_ERR;

    // A rejected standalone surface stays standalone, so the caller can retry
    // with it (Trim 1 below does exactly that).
    SmBrep* pRejectedSrf = pBox;
    if( SmApiTrimProjectParallel(pSurface, pOwnedTestLine, sZeroDir, sRefPt,
                                 SM_TT_KEEP_POINT, pRejectedSrf) != SM_ERR_INVALID_INPUT
        || pRejectedSrf != NULL || pSurface->GetOwner() != NULL )
        return SM_ERR;

    // A degenerate (collinear-corner) surface cannot become a face. That
    // failure must be reported, not turned into a faceless "trimmed" Brep,
    // and the surface must stay with the caller.
    SmPoint3d sD0(0,0,0), sD1(1,0,0), sD2(2,0,0), sD3(3,0,0);
    SmSurface* pDegenSrf = NULL;
    SER( SmApiCreateSurfaceFromCornerPoints(sD0, sD1, sD2, sD3, pDegenSrf) );
    SmTArray<SmPoint3d> sDegenCrvPts;
    sDegenCrvPts.Add( SmPoint3d(-5,5,15) );
    sDegenCrvPts.Add( SmPoint3d( 2,5,15) );
    sDegenCrvPts.Add( SmPoint3d( 8,5,15) );
    sDegenCrvPts.Add( SmPoint3d(15,5,15) );
    SmBSplineCurve* pDegenCrv = NULL;
    SER( SmApiCreateCurve(sDegenCrvPts, pDegenCrv) );
    SmPoint3d sDegenRefPt(1,1,1);
    SmBrep* pDegenResult = pBox;
    SmStatus eDegenStat = SmApiTrimProjectParallel(
        pDegenSrf, pDegenCrv, sDir, sDegenRefPt, SM_TT_KEEP_POINT, pDegenResult);
    delete pDegenCrv;
    SmBoolean bDegenStandalone = ( pDegenSrf->GetOwner() == NULL );
    if( bDegenStandalone )
        delete pDegenSrf;
    if( eDegenStat == SM_SUCCESS || pDegenResult != NULL || !bDegenStandalone )
        return SM_ERR;

    // Trim 1: horizontal cut at y=15, keep the bottom half (y < 15)
    SmLine* pLine = NULL ;
    SmPoint3d sLP1(-10,15,10), sLP2(30,15,10);
    SER( SmApiCreateLineSegment( sLP1, sLP2, pLine) );

    SmBrep* pResult = NULL;
    SER( SmApiTrimProjectParallel(pSurface, pLine, sDir, sRefPt, SM_TT_KEEP_POINT, pResult) );

    // Trim 2: vertical cut at x=15, keep the left side (x < 15)
    SmPoint3d sLP3v(15,-10,10), sLP4v(15,30,10);
    SER( SmApiCreateLineSegment( sLP3v, sLP4v, pLine) );
    SER( SmApiTrimProjectParallel(pResult, pLine, sDir, sRefPt, SM_TT_KEEP_POINT, pResult) );


    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************
SmStatus TestSmCreatePlanarFaces()
{
    
    SmApiCreateContext();

    SmPoint3d c1( 0,0,0 );
    SmPoint3d c2( 10,0,0 );
    SmPoint3d c3( 10,10,0 );
    SmPoint3d c4( 0,10,0 );

    SmTArray<SmCurve*> sTrimmingCurves;

    SmPoint3d p0_0(0,0,0), p10_0(10,0,0), p10_5(10,5,0), p5_5(5,5,0);
    SmPoint3d p5_10(5,10,0), p0_10(0,10,0);
    SmPoint3d p1_5(1,5,0), p1_8(1,8,0), p4_5(4,5,0);
    SmPoint3d p6_25(6,2.5,0);

    SmLine* pLine = NULL ;
    SER( SmApiCreateLineSegment( p0_0, p10_0, pLine) );
    sTrimmingCurves.Add( pLine );
               
    SER( SmApiCreateLineSegment( p10_0, p10_5, pLine) );
    sTrimmingCurves.Add( pLine );

    SER( SmApiCreateLineSegment( p10_5, p5_5, pLine) );
    sTrimmingCurves.Add( pLine );

    SER( SmApiCreateLineSegment( p5_5, p5_10, pLine) );
    sTrimmingCurves.Add( pLine );

    SER( SmApiCreateLineSegment( p5_10, p0_10, pLine) );
    sTrimmingCurves.Add( pLine );
 
    SER( SmApiCreateLineSegment( p0_10, p0_0, pLine) );
    sTrimmingCurves.Add( pLine );
 
    
    // Now create first inner loop - make it a clockwise list of 3 lines
    SmApiCreateLineSegment( p1_5, p1_8, pLine);
    sTrimmingCurves.Add( pLine );

    SmApiCreateLineSegment( p1_8, p4_5, pLine);
    sTrimmingCurves.Add( pLine );

    SmApiCreateLineSegment( p4_5, p1_5, pLine);
    sTrimmingCurves.Add( pLine );

    SmBSplineCurve* pCircle = NULL;
    SmApiCreateCircle( p6_25, 1.5, pCircle);
    sTrimmingCurves.Add( pCircle );

    SmBrep* pResult = NULL;
    SER( SmApiCreatePlanarFaces(sTrimmingCurves, pResult) );
    
 
    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************


SmStatus TestSmTrimmedSurfaces()
{
    SmStatus stat = TestSmTrimSurfaceWith3dCurves();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmTrimProjectParallel();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCreatePlanarFaces();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}
