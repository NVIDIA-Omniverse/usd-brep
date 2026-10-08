// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StdAfx.h"
#include "SmApiQueries.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include <SmMapPtrToPtr.h>
#include <SmVertex.h>
#include <SmEdge.h>
#include <SmEdgeuse.h>
#include <SmFace.h>
#include <SmLoopuse.h>
#include <SmRegion.h>
#include <SmAttribute.h>
#include <SmCurve.h>
#include <SmSurface.h>
#include <SmBrep.h>
#include <SmRegion.h>
#include <SmSolutionArray.h>
#include <SmTopologySolver.h>
#include <SmTol.h>
#include <SmTools.h>
#include <SmPointClass.h>
#include <SmShape.h>

/*******************************************************************//**
PURPOSE --- Find the closest point on a brep to a given point

NOTES ---   Convenience wrapper around SmTopologySolver::BrepPointSolve
            with SM_SO_MINIMIZE and SM_SR_SINGLE.

***********************************************************************/
SmApiStatus SmApiGetClosestPoint
(
    const SmBrep* pBrep,                          ///< [in ]: Brep to query
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on brep
    SmPoint3d& rClosestPoint,               ///< [out]: Closest point on brep
    double& rdDistance                      ///< [out]: Distance from input point to closest point
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SmSolutionArray sSolutions;
    double dBigDist = 1.0e+20;

    SmStatus status = SmTopologySolver::BrepPointSolve(
        pBrep,
        crPoint,
        SM_SO_MINIMIZE,
        SM_SR_SINGLE,
        SM_ZONE_TOL_3D,
        dBigDist * dBigDist,
        NULL,
        sSolutions );

    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        return ( status != SM_SUCCESS ) ? status : SM_ERR;

    SmSolution& rSol = sSolutions[0];
    rdDistance = rSol.m_vStart.m_dSolutionValue;
    SER( rSol.GetPoint( 0, rClosestPoint ) );

    return SM_SUCCESS;

} // End SmApiGetClosestPoint

// Count how many sample points of pSample fall strictly inside pTarget's solid
// material (rInside) vs strictly outside it (rOutside). Samples are pSample's
// vertices plus one interior point per face; the per-face points add coverage
// where the vertices are sparse or land on pTarget's boundary (a sphere, for
// instance, carries only its two pole vertices). SM_PC_REGION covers BOTH the
// solid interior and the exterior/void, so "inside" is SM_PC_REGION with a
// non-void region; boundary hits count as neither (a shared boundary is not
// containment).
static void sm_CountContainedSamples
(
    const SmBrep* pSample,
    const SmBrep* pTarget,
    SmZoneTol3d   sZoneTol,
    int&          rInside,
    int&          rOutside
)
{
    rInside = 0;
    rOutside = 0;

    SmTArray<SmPoint3d> sPts;
    SmTArray<SmVertex*> sVerts;  pSample->GetVertices( sVerts );
    for( ULONG ii = 0; ii < sVerts.GetSize(); ++ii )
        sPts.Add( sVerts[ii]->GetPoint() );
    SmTArray<SmFace*> sFaces;  pSample->GetFaces( sFaces );
    for( ULONG ii = 0; ii < sFaces.GetSize(); ++ii )
    {
        SmPoint3d sFacePt;
        if( sFaces[ii]->CalculateAnInternalPoint( sFacePt ) == SM_SUCCESS )
            sPts.Add( sFacePt );
    }

    for( ULONG ii = 0; ii < sPts.GetSize(); ++ii )
    {
        SmPointClassification sPC;
        if( pTarget->Point3DClassify( sPts[ii], sZoneTol, FALSE, sPC ) != SM_SUCCESS )
            continue;
        if( sPC.GetPointClass() == SM_PC_REGION )
        {
            SmRegion* pReg = (SmRegion*)sPC.GetObject();
            if( pReg != NULL && !pReg->IsVoid() )  ++rInside;
            else                                   ++rOutside;
        }
        else if( sPC.GetPointClass() == SM_PC_UNKNOWN ) ++rOutside;
    }
}

// TRUE if crPt is strictly inside pTarget's solid material.
static SmBoolean sm_PointInSolid( const SmBrep* pTarget, const SmPoint3d& crPt, SmZoneTol3d sZoneTol )
{
    SmPointClassification sPC;
    if( pTarget->Point3DClassify( crPt, sZoneTol, FALSE, sPC ) != SM_SUCCESS )
        return FALSE;
    if( sPC.GetPointClass() != SM_PC_REGION )
        return FALSE;
    SmRegion* pReg = (SmRegion*)sPC.GetObject();
    return ( pReg != NULL && !pReg->IsVoid() ) ? TRUE : FALSE;
}


/**********************************************************************//**
PURPOSE --- Classify the geometric relationship between two Breps.

NOTES ---   Combines the boundary-to-boundary minimum distance (BrepBrepSolve,
    SM_SO_MINIMIZE) with a Point3DClassify containment test to report one of:
      0 SM_BREP_REL_SEPARATE          disjoint; rdDistance = surface gap (> 0)
      1 SM_BREP_REL_TOUCHING          boundaries meet, interiors do not; rdDistance = 0
      2 SM_BREP_REL_INTERPENETRATING  interiors overlap;                 rdDistance = 0
      3 SM_BREP_REL_A_CONTAINS_B      B fully inside A, no contact;       rdDistance = wall gap (> 0)
      4 SM_BREP_REL_B_CONTAINS_A      A fully inside B, no contact;       rdDistance = wall gap (> 0)

    "Contains" means inside the other Brep's solid material.  A Brep lying
    inside a void/cavity region of another is SEPARATE, not contained.

    SEPARATE/A_CONTAINS_B/B_CONTAINS_A is exact: disjoint boundaries mean every
    point of one Brep shares one classification against the other, so the
    BrepBrepSolve closest-approach point pair is a sufficient witness.

    TOUCHING vs INTERPENETRATING still uses vertex/face-point sampling, so an
    overlap missed by every sample may under-report as TOUCHING. Reliable for
    typical CAD assemblies.
***********************************************************************/
SmApiStatus SmApiBrepRelationship
(
    const SmBrep* pBrepA,
    const SmBrep* pBrepB,
    int&    rRelationship,
    double& rdDistance
)
{
    if( pBrepA == NULL || pBrepB == NULL )
        { return SM_ERR_INVALID_INPUT; }

    // 1) boundary-to-boundary minimum distance
    SmSolutionArray sSolutions;
    double dBigDist = 1.0e+20;
    SmStatus status = SmTopologySolver::BrepBrepSolve(
        pBrepA, pBrepB, SM_SO_MINIMIZE, SM_SR_SINGLE,
        SM_ZONE_TOL_3D, dBigDist * dBigDist, NULL, sSolutions );
    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        { return ( status != SM_SUCCESS ) ? status : SM_ERR; }
    const double dSurfaceDist = sSolutions[0].m_vStart.m_dSolutionValue;

    const double dTol = pBrepA->GetTolerance() + pBrepB->GetTolerance();
    SmZoneTol3d sZoneTol = SmTol::GetZoneTol3d( (const SmContext*)NULL );

    // 2) decide.  rdDistance is the raw surface gap in every arrangement; the
    // relationship label says what it means (an external gap when SEPARATE, the
    // wall clearance when one contains the other, 0 when the boundaries meet).
    if( dSurfaceDist > dTol )
    {
        // Disjoint boundaries -> separate or containment.  Classification is
        // exact via the closest-approach points already computed above (see
        // docstring), not sampling.
        rdDistance = dSurfaceDist;

        SmPoint3d sPtOnA, sPtOnB;
        if( sSolutions[0].GetPoint( 0, sPtOnA ) == SM_SUCCESS &&
            sSolutions[0].GetPoint( 1, sPtOnB ) == SM_SUCCESS )
        {
            SmBoolean bBinA = sm_PointInSolid( pBrepA, sPtOnB, sZoneTol );  // B's point vs A
            SmBoolean bAinB = sm_PointInSolid( pBrepB, sPtOnA, sZoneTol );  // A's point vs B

            if( bBinA )       { rRelationship = 3; }  // A contains B
            else if( bAinB )  { rRelationship = 4; }  // B contains A
            else              { rRelationship = 0; }  // separate
        }
        else
        {
            // Fallback: point extraction failed, use sampling instead.
            int nBin = 0, nBout = 0, nAin = 0, nAout = 0;
            sm_CountContainedSamples( pBrepB, pBrepA, sZoneTol, nBin, nBout );  // B's points vs A
            sm_CountContainedSamples( pBrepA, pBrepB, sZoneTol, nAin, nAout );  // A's points vs B
            if( nBin > 0 && nBout == 0 )       { rRelationship = 3; }  // A contains B
            else if( nAin > 0 && nAout == 0 )  { rRelationship = 4; }  // B contains A
            else                               { rRelationship = 0; }  // separate
        }
    }
    else
    {
        // Boundaries meet -> touching or interpenetrating (distance 0), decided
        // by sampling (heuristic -- see docstring).
        rdDistance = 0.0;

        int nBin = 0, nBout = 0, nAin = 0, nAout = 0;
        sm_CountContainedSamples( pBrepB, pBrepA, sZoneTol, nBin, nBout );  // B's points vs A
        sm_CountContainedSamples( pBrepA, pBrepB, sZoneTol, nAin, nAout );  // A's points vs B
        if( nBin > 0 || nAin > 0 )         { rRelationship = 2; }  // interpenetrating
        else                               { rRelationship = 1; }  // touching
    }
    return SM_SUCCESS;

} // End SmApiBrepRelationship

/**********************************************************************//**
PURPOSE --- Distance between two Breps' boundary surfaces: the gap when they
    are disjoint (the wall clearance when one is nested inside the other), 0.0
    when they touch or interpenetrate.

NOTES ---   Thin wrapper over SmApiBrepRelationship returning only its distance
    component, so the two never disagree.  Use SmApiBrepRelationship when the
    number needs the arrangement (separate vs containment) to be meaningful.
***********************************************************************/
SmApiStatus SmApiBrepDistance
(
    const SmBrep* pBrepA,
    const SmBrep* pBrepB,
    double& rdDistance
)
{
    int iRelationship = 0;
    return SmApiBrepRelationship( pBrepA, pBrepB, iRelationship, rdDistance );

} // End SmApiBrepDistance

/*******************************************************************//**
PURPOSE --- Get all edges from a brep

NOTES ---   Returns SM_ERR if the brep has no edges.

***********************************************************************/
SmApiStatus SmApiGetEdges
(
    SmBrep* pBrep,                          ///< [in ]: Brep to query
    SmTArray<SmEdge*>& rEdges              ///< [out]: Edges of brep
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    pBrep->GetEdges( rEdges );

    if( rEdges.GetSize() == 0 )
        return SM_ERR;

    return SM_SUCCESS;

} // End SmApiGetEdges

/*******************************************************************//**
PURPOSE --- Get all faces from a brep

NOTES ---   Returns SM_ERR if the brep has no faces.

***********************************************************************/
SmApiStatus SmApiGetFaces
(
    SmBrep* pBrep,                          ///< [in ]: Brep to query
    SmTArray<SmFace*>& rFaces              ///< [out]: Faces of brep
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    pBrep->GetFaces( rFaces );

    if( rFaces.GetSize() == 0 )
        return SM_ERR;

    return SM_SUCCESS;

} // End SmApiGetFaces

/*******************************************************************//**
PURPOSE --- Get a face's ordered boundary-edge occurrences, grouped by loop.

NOTES ---   Uses the upward faceuse so the first loop is the outer loop.  Each
    loop's edgeuses are returned in SMLib traversal order.  Unlike GetEdges,
    this deliberately preserves repeated occurrences of an edge at a seam.
    The starting occurrence within a closed loop and the order of inner loops
    are unspecified.  Edge pointers are borrowed from the face's owning Brep.
    All outputs are cleared before the query and remain empty on failure.

***********************************************************************/
SmApiStatus SmApiFaceGetBoundaryLoops
(
    SmFace* pFace,
    SmTArray<ULONG>& rEdgeCountsPerLoop,
    SmTArray<SmEdge*>& rEdges,
    SmTArray<SmOrientType>& rOrientations
)
{
    rEdgeCountsPerLoop.ReSet();
    rEdges.ReSet();
    rOrientations.ReSet();

    if( pFace == NULL )
        return SM_ERR_INVALID_INPUT;

    SmTArray<SmLoopuse*> sLoopuses;
    pFace->GetUpwardLoopuses( sLoopuses );
    if( sLoopuses.GetSize() == 0 )
        return SM_ERR;

    for( ULONG ii = 0; ii < sLoopuses.GetSize(); ii++ )
    {
        SmLoopuse* pLoopuse = sLoopuses[ii];
        if( pLoopuse == NULL )
        {
            rEdgeCountsPerLoop.ReSet();
            rEdges.ReSet();
            rOrientations.ReSet();
            return SM_ERR;
        }

        SmTArray<SmEdgeuse*> sEdgeuses;
        pLoopuse->GetEdgeuses( sEdgeuses );
        rEdgeCountsPerLoop.Add( sEdgeuses.GetSize() );

        for( ULONG jj = 0; jj < sEdgeuses.GetSize(); jj++ )
        {
            SmEdgeuse* pEdgeuse = sEdgeuses[jj];
            SmEdge* pEdge = pEdgeuse == NULL ? NULL : pEdgeuse->GetEdge();
            SmOrientType eOrientation = pEdgeuse == NULL
                ? SM_OT_UNKNOWN
                : pEdgeuse->GetOrientation();
            if(   pEdge == NULL
               || (eOrientation != SM_OT_SAME && eOrientation != SM_OT_OPPOSITE) )
            {
                rEdgeCountsPerLoop.ReSet();
                rEdges.ReSet();
                rOrientations.ReSet();
                return SM_ERR;
            }

            rEdges.Add( pEdge );
            rOrientations.Add( eOrientation );
        }
    }

    return SM_SUCCESS;

} // End SmApiFaceGetBoundaryLoops

/*******************************************************************//**
PURPOSE --- Find the edge in a brep closest to a given 3D point

NOTES ---   Uses SmTools::FindEdge to locate the edge and parameter.

***********************************************************************/
SmApiStatus SmApiFindEdge
(
    SmBrep* pBrep,                          ///< [in ]: Brep to search
    const SmPoint3d& crPoint,                     ///< [in ]: Point on or near edge
    ULONG& rlEdgeIndex,                     ///< [out]: Index of found edge in brep
    double& rdParam                        ///< [out]: Parameter on found edge
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SmStatus stat = SmTools::FindEdge( pBrep, crPoint, rlEdgeIndex, rdParam );

    if( stat != SM_SUCCESS )
        return stat;

    return SM_SUCCESS;

} // End SmApiFindEdge

/*******************************************************************//**
PURPOSE --- Find the face in a brep closest to a given 3D point

NOTES ---   Uses SmTools::FindFace to locate the face.

***********************************************************************/
SmApiStatus SmApiFindFaces
(
    SmBrep* pBrep,                          ///< [in ]: Brep to search
    const SmPoint3d& crPoint,                     ///< [in ]: Point on or near face
    SmFace*& rpFace                        ///< [out]: Pointer to found face
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    rpFace = NULL;
    SmStatus stat = SmTools::FindFace( pBrep, crPoint, rpFace );

    if( stat != SM_SUCCESS || rpFace == NULL )
        return stat != SM_SUCCESS ? stat : SM_ERR;

    return SM_SUCCESS;

} // End SmApiFindFaces

/*******************************************************************//**
PURPOSE --- Find the single closest point on a curve to a given point

NOTES ---   Uses SmCurve::GlobalPointSolve with SM_SO_MINIMIZE and
            SM_SR_SINGLE to find the best closest point.

***********************************************************************/
SmApiStatus SmApiCurveClosestPoint
(
    const SmCurve* pCurve,             ///< [in ]: Curve to query
    const SmPoint3d& crPoint,          ///< [in ]: Point to find closest on curve
    SmPoint3d& rClosestPoint,    ///< [out]: Closest point on curve
    double& rdParameter,         ///< [out]: Curve parameter at closest point
    double& rdDistance           ///< [out]: Distance from input point to closest point
)
{
    if( !pCurve )
        return SM_ERR_INVALID_INPUT;

    SmSolutionArray sSolutions;

    SmStatus status = pCurve->GlobalPointSolve(
        pCurve->GetNaturalInterval(),
        SM_SO_MINIMIZE,
        crPoint,
        SM_ZONE_TOL_3D,
        NULL,
        NULL,
        SM_SR_SINGLE,
        sSolutions );

    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        return ( status != SM_SUCCESS ) ? status : SM_ERR;

    SmSolution& rSol = sSolutions[0];
    const double dParameter = rSol.m_vStart[0];
    SmPoint3d sPoint;
    SER( pCurve->EvaluatePoint( dParameter, sPoint ) );
    rClosestPoint = sPoint;
    rdParameter = dParameter;
    rdDistance = rSol.m_vStart.m_dSolutionValue;

    return SM_SUCCESS;

} // SmApiCurveClosestPoint

/*******************************************************************//**
PURPOSE --- Find all closest points on a curve to a given point

NOTES ---   Uses SmCurve::GlobalPointSolve with SM_SO_MINIMIZE and
            SM_SR_ALL to find all solutions within tolerance of the global
            minimum.

***********************************************************************/
SmApiStatus SmApiCurveClosestPointAll
(
    const SmCurve* pCurve,                ///< [in ]: Curve to query
    const SmPoint3d& crPoint,             ///< [in ]: Point to find closest on curve
    SmTArray<SmPoint3d>& rPoints,   ///< [out]: All closest points on curve
    SmTArray<double>& rParameters,  ///< [out]: Curve parameters at each closest point
    SmTArray<double>& rDistances   ///< [out]: Distances from input point to each closest point
)
{
    if( !pCurve )
        return SM_ERR_INVALID_INPUT;

    SmSolutionArray sSolutions;

    SmStatus status = pCurve->GlobalPointSolve(
        pCurve->GetNaturalInterval(),
        SM_SO_MINIMIZE,
        crPoint,
        SM_ZONE_TOL_3D,
        NULL,
        NULL,
        SM_SR_ALL,
        sSolutions );

    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        return ( status != SM_SUCCESS ) ? status : SM_ERR;

    SmPoint3d sPt;
    for( ULONG ii = 0; ii < sSolutions.GetSize(); ii++ )
    {
        SmSolution& rSol = sSolutions[ii];
        double dParam = rSol.m_vStart[0];
        double dDist  = rSol.m_vStart.m_dSolutionValue;

        SER( pCurve->EvaluatePoint( dParam, sPt ) );
        rPoints.Add( sPt );
        rParameters.Add( dParam );
        rDistances.Add( dDist );
    }

    return SM_SUCCESS;

} // SmApiCurveClosestPointAll

/*******************************************************************//**
PURPOSE --- Find one closest point on a surface and its known UV representations

NOTES ---   Uses SmSurface::DropPoint with SM_SO_MINIMIZE, preserving the
            established solve path while requesting its full solution array.
            Analytic paths can report other equally close spatial points, so
            this wrapper selects one best spatial result and retains only the
            returned UV values that evaluate to that point.  It does not
            promise to discover every tied closest point or every preimage of
            the selected point.  The search domain and returned UVs use the
            natural/NURBS parameterization expected by EvaluatePoint.

***********************************************************************/
SmApiStatus SmApiSurfaceClosestPoint
(
    const SmSurface* pSurface,             ///< [in ]: Surface to query
    const SmPoint3d& crPoint,              ///< [in ]: Point to find closest on surface
    SmPoint3d& rClosestPoint,        ///< [out]: One closest point on surface
    SmTArray<SmPoint2d>& rUVs,       ///< [out]: Known UV representations of rClosestPoint
    double& rdDistance               ///< [out]: Distance from input point to rClosestPoint
)
{
    rUVs.ReSet();

    if( !pSurface )
        return SM_ERR_INVALID_INPUT;

    SmBoolean bSuccess = FALSE;
    SmBoolean bIsMultivalued = FALSE;
    SmPoint2d sUnusedUV;
    double dUnusedDistance = SM_BIG_DOUBLE;
    SmSolutionArray sSolutions;

    SmStatus status = pSurface->DropPoint(
        crPoint,
        pSurface->GetNaturalUVDomain(),
        NULL,
        bSuccess,
        sUnusedUV,
        dUnusedDistance,
        bIsMultivalued,
        SM_SO_MINIMIZE,
        NULL,
        &sSolutions );

    if( status != SM_SUCCESS || !bSuccess || sSolutions.GetSize() < 1 )
        return ( status != SM_SUCCESS ) ? status : SM_ERR;

    SmZoneTol3d sPointTol = SmTol::GetZoneTol3d( pSurface );

    ULONG lBestSolution = 0;
    for( ULONG ii = 1; ii < sSolutions.GetSize(); ii++ )
    {
        if( sSolutions[ii].m_vStart.m_dSolutionValue <
            sSolutions[lBestSolution].m_vStart.m_dSolutionValue )
        {
            lBestSolution = ii;
        }
    }

    SmSolution& rBestSolution = sSolutions[lBestSolution];
    SmPoint2d sSelectedUV;
    sSelectedUV.Set( rBestSolution.m_vStart[0], rBestSolution.m_vStart[1] );
    rdDistance = rBestSolution.m_vStart.m_dSolutionValue;
    SER( pSurface->EvaluatePoint( sSelectedUV, rClosestPoint ) );

    for( ULONG ii = 0; ii < sSolutions.GetSize(); ii++ )
    {
        SmPoint2d sUV( sSolutions[ii].m_vStart[0], sSolutions[ii].m_vStart[1] );
        SmPoint3d sPoint;
        if( pSurface->EvaluatePoint( sUV, sPoint ) != SM_SUCCESS )
            continue;

        if( sPoint.CloserThan( sPointTol, rClosestPoint ) )
            rUVs.Add( sUV );
    }

    if( rUVs.GetSize() < 1 )
        return SM_ERR;

    return SM_SUCCESS;

} // SmApiSurfaceClosestPoint

/*******************************************************************//**
PURPOSE --- Find the single closest point on a brep to a given point

NOTES ---   Uses SmTopologySolver::BrepPointSolve with SM_SO_MINIMIZE
            and SM_SR_SINGLE to find the closest point across all
            faces and edges of the brep.

***********************************************************************/
SmApiStatus SmApiBrepClosestPoint
(
    const SmBrep* pBrep,               ///< [in ]: Brep to query
    const SmPoint3d& crPoint,          ///< [in ]: Point to find closest on brep
    SmPoint3d& rClosestPoint,    ///< [out]: Closest point on brep
    double& rdDistance           ///< [out]: Distance from input point to closest point
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SmSolutionArray sSolutions;
    double dBigDist = 1.0e+20;

    SmStatus status = SmTopologySolver::BrepPointSolve(
        pBrep,
        crPoint,
        SM_SO_MINIMIZE,
        SM_SR_SINGLE,
        SM_ZONE_TOL_3D,
        dBigDist * dBigDist,
        NULL,
        sSolutions );

    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        return ( status != SM_SUCCESS ) ? status : SM_ERR;

    SmSolution& rSol = sSolutions[0];
    rdDistance = rSol.m_vStart.m_dSolutionValue;

    SER( rSol.GetPoint( 0, rClosestPoint ) );

    return SM_SUCCESS;

} // SmApiBrepClosestPoint

/*******************************************************************//**
PURPOSE --- Find all closest points on a brep to a given point

NOTES ---   Uses SmTopologySolver::BrepPointSolve with SM_SO_MINIMIZE
            and SM_SR_ALL to find all solutions within tolerance of the global
            minimum across all faces and edges of the brep.

***********************************************************************/
SmApiStatus SmApiBrepClosestPointAll
(
    const SmBrep* pBrep,                   ///< [in ]: Brep to query
    const SmPoint3d& crPoint,              ///< [in ]: Point to find closest on brep
    SmTArray<SmPoint3d>& rPoints,    ///< [out]: All closest points on brep
    SmTArray<double>& rDistances    ///< [out]: Distances from input point to each closest point
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SmSolutionArray sSolutions;
    double dBigDist = 1.0e+20;

    SmStatus status = SmTopologySolver::BrepPointSolve(
        pBrep,
        crPoint,
        SM_SO_MINIMIZE,
        SM_SR_ALL,
        SM_ZONE_TOL_3D,
        dBigDist * dBigDist,
        NULL,
        sSolutions );

    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        return ( status != SM_SUCCESS ) ? status : SM_ERR;

    SmPoint3d sPt;
    for( ULONG ii = 0; ii < sSolutions.GetSize(); ii++ )
    {
        SmSolution& rSol = sSolutions[ii];
        double dDist = rSol.m_vStart.m_dSolutionValue;

        SER( rSol.GetPoint( 0, sPt ) );
        rPoints.Add( sPt );
        rDistances.Add( dDist );
    }

    return SM_SUCCESS;

} // SmApiBrepClosestPointAll


/*******************************************************************//**
PURPOSE --- Query whether a Brep is a manifold solid.

USAGE NOTES --- A manifold solid has at least two regions, only manifold
    edges, and no vertex shells.

    Migrated from SmApiBrep.cpp on the rationale that read-only Brep
    properties belong with the rest of the SmApiQueries surface.
***********************************************************************/
SmApiStatus SmApiBrepIsManifoldSolid
(
    const SmBrep* pBrep,
    SmBoolean & rbIsManifoldSolid
)
{
    if( pBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    rbIsManifoldSolid = pBrep->IsManifoldSolid();

    return SM_SUCCESS;

} // End SmApiBrepIsManifoldSolid

/*******************************************************************//**
PURPOSE --- Count a Brep's material (solid) regions and enclosed void
    cavities, counting nested shells unambiguously.

USAGE NOTES --- SMLib stores each connected region separately, so nesting is
    explicit.  Walks SmBrep::GetRegions once, classifying by IsVoid():
    solids are IsVoid()==FALSE; voids are IsVoid()==TRUE minus the single
    infinite region (always a void).  Read-only; the counts reflect the
    brep's current void flags (maintained by construction/boolean/heal).
***********************************************************************/
SmApiStatus SmApiBrepMaterialCensus
(
    const SmBrep*  pBrep,
    long &   rlSolidCount,
    long &   rlVoidCount
)
{
    if( pBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmTArray<SmRegion*> sRegions;
    pBrep->GetRegions( sRegions );

    long lSolids = 0;
    long lVoids  = 0;
    ULONG lCount = sRegions.GetSize();
    for( ULONG i = 0; i < lCount; i++ )
    {
        SmRegion* pRegion = sRegions.GetAt( i );
        if( pRegion == NULL )
            continue;

        if( pRegion->IsVoid() )
        {
            if( !pRegion->IsInfiniteRegion() )
                lVoids++;
        }
        else
        {
            lSolids++;
        }
    }

    rlSolidCount = lSolids;
    rlVoidCount  = lVoids;

    return SM_SUCCESS;

} // End SmApiBrepMaterialCensus

/*******************************************************************//**
PURPOSE --- Shared single-pass core: one SmBrep::ComputePreciseProperties
    integration about crOrigin (raw area/volume/mass + 8-slot moment vector;
    see SmBrep.h).  The caller owns density and validation.  Both volume and
    mass-properties entries route through here so the kernel call is in one
    place.
***********************************************************************/
static SmApiStatus sm_BrepComputePropertiesAbout
(
    const SmBrep*                  pBrep,
    double                         dRelativeAccuracy,
    const SmMassPropertyBehavior & crBehavior,
    const SmPoint3d &              crOrigin,
    double &                       rdArea,
    double &                       rdVolume,
    double &                       rdMass,
    SmTArray<SmVector3d> &         rMoments
)
{
    rMoments.SetSize(8);
    return pBrep->ComputePreciseProperties( dRelativeAccuracy, crOrigin, crBehavior,
                                            rdArea, rdVolume, rdMass, rMoments );
}

/*******************************************************************//**
PURPOSE --- Compute the enclosed volume of a Brep using analytic
    integration over trimmed NURBS boundaries.

USAGE NOTES --- Accuracy is clamped to [1e-8, 1e-1]; non-finite values are
    rejected.  The Brep should be a closed manifold solid.  Integration uses
    the bounding-box midpoint and the mass-properties core.
***********************************************************************/
SmApiStatus SmApiBrepComputeVolume
(
    const SmBrep*  pBrep,
    double   dRelativeAccuracy,
    double & rdVolume
)
{
    if( pBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    if( !SM_IS_VALID_DOUBLE( dRelativeAccuracy ) )
        { return SM_ERR_INVALID_INPUT; }
    if( dRelativeAccuracy < 1.0e-8 ) { dRelativeAccuracy = 1.0e-8; }
    if( dRelativeAccuracy > 1.0e-1 ) { dRelativeAccuracy = 1.0e-1; }

    SmExtent3d sBBox;
    SER( pBrep->CalculateBoundingBox(sBBox) );

    SmMassPropertyBehavior sBehavior;
    double dArea = 0.0, dMass = 0.0;
    SmTArray<SmVector3d> sMoments;
    SER( sm_BrepComputePropertiesAbout( pBrep, dRelativeAccuracy, sBehavior,
                                        sBBox.GetMid(),
                                        dArea, rdVolume, dMass, sMoments ) );

    return SM_SUCCESS;

} // End SmApiBrepComputeVolume

/*******************************************************************//**
PURPOSE --- Compute the total surface area of a Brep: the sum of its
    trimmed face areas.

USAGE NOTES --- Each face goes through SmApiFaceComputeArea, so the
    accuracy envelope and per-face orientation handling are the same as
    for single faces; non-finite accuracy is rejected.  Unlike
    SmApiBrepComputeMassProperties, the Brep does not have to be a closed
    solid: sheet bodies and open shells are accepted.  A Brep with no
    faces has area 0.
***********************************************************************/
SmApiStatus SmApiBrepComputeArea
(
    SmBrep*  pBrep,
    double   dRelativeAccuracy,
    double & rdArea
)
{
    if( pBrep == NULL )
        { return SM_ERR; }

    if( !SM_IS_VALID_DOUBLE( dRelativeAccuracy ) )
        { return SM_ERR_INVALID_INPUT; }

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );

    double dArea = 0.0;
    for( ULONG ii = 0; ii < sFaces.GetSize(); ii++ )
    {
        double dFaceArea = 0.0;
        SER( SmApiFaceComputeArea( sFaces[ii], dRelativeAccuracy, dFaceArea ) );
        dArea += dFaceArea;
    }
    rdArea = dArea;

    return SM_SUCCESS;

} // End SmApiBrepComputeArea

/*******************************************************************//**
PURPOSE --- Compute area, volume, mass, centroid, and the mass moments /
    products of inertia of a Brep about a caller-supplied origin.

USAGE NOTES --- A single ComputePreciseProperties integration about crOrigin.
    Moments/products are about axes through crOrigin (parallel to world axes);
    rCentroid = crOrigin + staticMoment/mass, independent of crOrigin.  For
    centroidal inertia, call again passing the returned rCentroid as crOrigin.
    Accuracy is clamped to [1e-4, 1e-1].  Returns SM_ERR_INVALID_INPUT for a
    non-finite accuracy/origin, a non-finite / sub-SM_EFF_ZERO density, an
    SM_AI_MASS_PROPERTIES-attributed input (would override the density), or a
    non-solid / zero-mass Brep.
***********************************************************************/
SmApiStatus SmApiBrepComputeMassProperties
(
    const SmBrep*     pBrep,
    double            dRelativeAccuracy,
    double            dDensity,
    const SmPoint3d & crOrigin,
    double &          rdArea,
    double &          rdVolume,
    double &          rdMass,
    SmPoint3d &       rCentroid,
    SmVector3d &      rMomentsOfInertia,
    SmVector3d &      rProductsOfInertia
)
{
    if( pBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    // Reject non-finite accuracy before clamping (NaN slips past both ordered
    // comparisons); finite out-of-range values still clamp to [1e-4, 1e-1].
    if( !SM_IS_VALID_DOUBLE( dRelativeAccuracy ) )
        { return SM_ERR_INVALID_INPUT; }
    if( dRelativeAccuracy < 1.0e-4 ) { dRelativeAccuracy = 1.0e-4; }
    if( dRelativeAccuracy > 1.0e-1 ) { dRelativeAccuracy = 1.0e-1; }

    // The kernel floors any density below SM_EFF_ZERO to 1.0, so a sub-epsilon
    // (or non-positive / non-finite) value would give a misleading unit-density
    // mass; reject it here since the kernel does not.
    if( !SM_IS_VALID_DOUBLE( dDensity ) || dDensity < SM_EFF_ZERO )
        { return SM_ERR_INVALID_INPUT; }

    // A non-finite origin would propagate into every moment.
    if(    !SM_IS_VALID_DOUBLE( crOrigin.x )
        || !SM_IS_VALID_DOUBLE( crOrigin.y )
        || !SM_IS_VALID_DOUBLE( crOrigin.z ) )
        { return SM_ERR_INVALID_INPUT; }

    // A per-entity SM_AI_MASS_PROPERTIES attribute overrides dDensity in the
    // kernel (attribute-weighted result, breaking mass == density * volume).
    {
        ULONG lMassAttrId = SM_AI_MASS_PROPERTIES;
        SmTArray<SmFace*>   sAttrFaces;
        SmTArray<SmEdge*>   sAttrEdges;
        SmTArray<SmVertex*> sAttrVertices;
        pBrep->GetFaces   ( sAttrFaces,    &lMassAttrId );
        pBrep->GetEdges   ( sAttrEdges,    &lMassAttrId );
        pBrep->GetVertices( sAttrVertices, &lMassAttrId );
        if(    sAttrFaces.GetSize()    != 0
            || sAttrEdges.GetSize()    != 0
            || sAttrVertices.GetSize() != 0 )
            { return SM_ERR_INVALID_INPUT; }
    }

    // Only defined for a closed manifold solid; a sheet body would otherwise
    // return an all-zero result with crOrigin as its "centroid".
    if( !pBrep->IsManifoldSolid() )
        { return SM_ERR_INVALID_INPUT; }

    SmMassPropertyBehavior sBehavior( dDensity );

    double dArea = 0.0, dVolume = 0.0, dMass = 0.0;
    SmTArray<SmVector3d> sMoments;
    SER( sm_BrepComputePropertiesAbout( pBrep, dRelativeAccuracy, sBehavior, crOrigin,
                                        dArea, dVolume, dMass, sMoments ) );

    // Reject a degenerate zero/non-finite mass or non-positive volume before
    // the centroid division makes the result meaningless.
    if(    !SM_IS_VALID_DOUBLE( dMass )   || dMass   < SM_EFF_ZERO
        || !SM_IS_VALID_DOUBLE( dVolume ) || dVolume < SM_EFF_ZERO )
        { return SM_ERR_INVALID_INPUT; }

    rdArea             = dArea;
    rdVolume           = dVolume;
    rdMass             = dMass;
    // sMoments[4] is the density-weighted static moment about crOrigin; divide
    // by mass to recover the geometric centroid.
    rCentroid          = crOrigin + ( sMoments[4] / dMass );
    // [7] = mass moments of inertia about crOrigin (integral of (y^2+z^2) dm,
    // ...); [6] = products as raw positive integrals (yz, zx, xy dm), whose
    // negatives are the inertia-tensor off-diagonals.
    rMomentsOfInertia  = sMoments[7];
    rProductsOfInertia = sMoments[6];

    return SM_SUCCESS;

} // End SmApiBrepComputeMassProperties

/*******************************************************************//**
PURPOSE --- Compute the axis-aligned bounding box of a Brep.

USAGE NOTES --- bTight=FALSE returns the union of vertex / edge / face
    boxes (fast, slightly larger than minimal).  bTight=TRUE calls
    CalculateTightBoundingBox which evaluates each face's surface at
    sample points -- expensive on large Breps; prefer FALSE unless you
    need a minimal box.
***********************************************************************/
SmApiStatus SmApiBrepBoundingBox
(
    const SmBrep* pBrep,
    SmBoolean   bTight,
    SmPoint3d & rMin,
    SmPoint3d & rMax
)
{
    if( pBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmExtent3d sBBox;

    if( bTight )
        { SER( pBrep->CalculateTightBoundingBox( sBBox ) ); }
    else
        { SER( pBrep->CalculateBoundingBox( sBBox ) ); }

    rMin = sBBox.GetMin();
    rMax = sBBox.GetMax();

    return SM_SUCCESS;

} // End SmApiBrepBoundingBox

/*******************************************************************//**
PURPOSE --- Deep-copy a Brep into a fresh handle owned by the SM_API
    context.

USAGE NOTES --- Wraps SmBrep's copy constructor.  The result is a
    standalone Brep with independent topology -- mutating it does not
    affect the source.  Allocated against SmApiGetOrCreateContext() so
    its lifetime tracks the context, matching every other SM_API
    creator. Surface and curve types are preserved; copying does not
    promote NURBS geometry to analytics.
***********************************************************************/
SmApiStatus SmApiBrepCopy
(
    const SmBrep*  pBrep,
    SmBrep*& rpResult
)
{
    if( pBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    rpResult = new (*SmApiGetOrCreateContext()) SmBrep(
        *pBrep, // in : crBrepToCopy: source Brep
        NULL,   // out: pOptCopyBrepMap: copy-to-source topology map, not requested
        NULL,   // out: pOptReverseMap: source-to-copy topology map, not requested
        TRUE,   // NotUsed: bCopyAttributes: currently ignored by the constructor
        FALSE   // in : bAddAnalytics: FALSE preserves source surface types
    );

    if( rpResult == NULL )
        { return SM_ERR; }

    return SM_SUCCESS;

} // End SmApiBrepCopy

/*******************************************************************//**
PURPOSE --- Compute the axis-aligned bounding box of a single face.

USAGE NOTES --- bTight=TRUE samples the face's surface inside its trim
    boundary for a minimal box; FALSE returns the loose face-level box.
    Use FALSE in tight loops over many faces.
***********************************************************************/
SmApiStatus SmApiFaceBoundingBox
(
    const SmFace* pFace,
    SmBoolean   bTight,
    SmPoint3d & rMin,
    SmPoint3d & rMax
)
{
    if( pFace == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmExtent3d sBBox;

    if( bTight )
        { SER( pFace->CalculateTightBoundingBox( sBBox ) ); }
    else
        { SER( pFace->CalculateBoundingBox( sBBox ) ); }

    rMin = sBBox.GetMin();
    rMax = sBBox.GetMax();

    return SM_SUCCESS;

} // End SmApiFaceBoundingBox

/*******************************************************************//**
PURPOSE --- Rough area estimate for SmFace::ComputePreciseProperties.

USAGE NOTES --- Must be positive: a zero estimate makes the integration
    fail on faces with a pole.
***********************************************************************/
static double sm_FaceAreaEstimate( const SmExtent3d & crBBox )
{
    SmVector3d sBoxSize = crBBox.GetSize();
    double dSize = smos_Max( sBoxSize.z, smos_Max(sBoxSize.x, sBoxSize.y) );
    return dSize * dSize / 100.0;
}

/*******************************************************************//**
PURPOSE --- Compute the surface area of a face within its trim
    boundary using numerical integration.

USAGE NOTES --- dRelativeAccuracy is clamped to [1e-4, 1e-1] to match
    SmApiBrepComputeVolume.  Origin of integration is the face's
    bounding-box midpoint for numerical conditioning.  The returned
    area is positive regardless of face orientation -- it is a
    geometric, not a signed, quantity. Area-only precise integration skips
    volume and moment components, retaining the area accuracy control.
***********************************************************************/
SmApiStatus SmApiFaceComputeArea
(
    const SmFace*  pFace,
    double   dRelativeAccuracy,
    double & rdArea
)
{
    if( pFace == NULL )
        { return SM_ERR_INVALID_INPUT; }

    if( dRelativeAccuracy < 1.0e-4 ) { dRelativeAccuracy = 1.0e-4; }
    if( dRelativeAccuracy > 1.0e-1 ) { dRelativeAccuracy = 1.0e-1; }

    SmExtent3d sBBox;
    SER( pFace->CalculateBoundingBox( sBBox ) );
    SmPoint3d sOrigin = sBBox.GetMid();
    double dEstimatedArea = sm_FaceAreaEstimate( sBBox );

    double dDeltaVolume = 0.0;
    SmTArray<SmVector3d> sDeltaMoments;

    SER( pFace->ComputePreciseProperties( SM_OT_SAME,
                                          dRelativeAccuracy,
                                          sOrigin,
                                          dEstimatedArea,
                                          /*dFaceThickness*/   0.0,
                                          rdArea,
                                          dDeltaVolume,
                                          sDeltaMoments,
                                          SM_PPF_AREA ) );

    if( rdArea < 0.0 )
        { rdArea = -rdArea; }

    return SM_SUCCESS;

} // End SmApiFaceComputeArea

/*******************************************************************//**
PURPOSE --- Return a 3D point that lies strictly inside a trimmed face.

USAGE NOTES --- Wraps SmFace::CalculateAnInternalPoint, which evaluates the
    owning surface at a UV point strictly interior to the face's trim loops.
    The result therefore lies on the owning surface and inside the trims.
    Useful as a seed point for ray tests, region classification, or picking.
***********************************************************************/
SmApiStatus SmApiFaceInternalPoint
(
    const SmFace*     pFace,
    SmPoint3d & rPoint
)
{
    if( pFace == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SER( pFace->CalculateAnInternalPoint( rPoint ) );

    return SM_SUCCESS;

} // End SmApiFaceInternalPoint

/*******************************************************************//**
PURPOSE --- Classify a UV parameter against a trimmed face's boundary.

USAGE NOTES --- Wraps SmFace::PointClassify.  rClassification is an
    SmPointClassificationType: SM_PC_FACE when the UV is inside the trim,
    SM_PC_UNKNOWN when outside (including inside a hole), and
    SM_PC_EDGE / SM_PC_VERTEX when on the boundary.  This is trim-aware:
    a UV inside an inner loop (hole) classifies as outside.
***********************************************************************/
SmApiStatus SmApiFaceClassifyUV
(
    const SmFace*           pFace,
    const SmPoint2d & crUV,
    int &             rClassification
)
{
    if( pFace == NULL )
        { return SM_ERR_INVALID_INPUT; }

    // NOTE: SmFace::PointClassify ignores its tolerance argument (see the
    // SM_REF3 in SmFace.cpp) and classifies purely by UV-domain containment
    // plus loop containment, using an internal UV-domain-relative epsilon.
    // The ZoneTol3d passed here is therefore inert; it is supplied only as the
    // documented "if none, use SmTol::GetZoneTol3d" default.  It is NOT used to
    // size a UV boundary snap zone, so there is no 3D-vs-UV tolerance mismatch.
    SmPointClassification sPC;
    SER( pFace->PointClassify( crUV,
                               SmTol::GetZoneTol3d( pFace ),
                               /*bDoBoundaryIntersections*/ FALSE,
                               /*bUseUVClassification*/     TRUE,
                               sPC ) );

    rClassification = (int)sPC.GetPointClass();

    return SM_SUCCESS;

} // End SmApiFaceClassifyUV

/*******************************************************************//**
PURPOSE --- Find the closest point on a single trimmed face to a given
    point.

USAGE NOTES --- Trim-aware, unlike SmApiSurfaceClosestPoint's
    SmSurface::DropPoint: scopes an SmShape to just this face and runs
    SmTopologySolver::ShapePointSolve (same machinery as
    SmApiGetClosestPoint, restricted to one face).
***********************************************************************/
SmApiStatus SmApiFaceClosestPoint
(
    SmFace*           pFace,
    const SmPoint3d & crPoint,
    SmPoint3d &       rClosestPoint,
    double &          rdDistance
)
{
    if( pFace == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmTArray<SmBrep*>   sBreps;
    SmTArray<SmFace*>   sFaces;
    SmTArray<SmEdge*>   sEdges;
    SmTArray<SmVertex*> sVertices;
    sFaces.SetSize(1);
    sFaces[0] = pFace;

    SmShape sShape( sBreps, sFaces, sEdges, sVertices );
    sShape.SetContext( pFace->GetContext() );

    SmSolutionArray sSolutions;
    double dBigDist = 1.0e+20;

    SmStatus status = SmTopologySolver::ShapePointSolve(
        &sShape,
        crPoint,
        SM_SO_MINIMIZE,
        SM_SR_SINGLE,
        SmTol::GetZoneTol3d( pFace ),
        dBigDist * dBigDist,
        NULL,
        sSolutions );

    if( status != SM_SUCCESS || sSolutions.GetSize() < 1 )
        { return ( status != SM_SUCCESS ) ? status : SM_ERR; }

    SmSolution& rSol = sSolutions[0];
    rdDistance = rSol.m_vStart.m_dSolutionValue;
    SER( rSol.GetPoint( 0, rClosestPoint ) );

    return SM_SUCCESS;

} // End SmApiFaceClosestPoint

/*******************************************************************//**
PURPOSE --- Compute the surface area and area centroid of a trimmed
    face.

USAGE NOTES --- Integrates only area and area first moments in one pass;
    centroid = origin + area first moment / area (rDeltaMoments[0]).
***********************************************************************/
SmApiStatus SmApiFaceComputeCentroid
(
    const SmFace*     pFace,
    double      dRelativeAccuracy,
    double &    rdArea,
    SmPoint3d & rCentroid
)
{
    if( pFace == NULL )
        { return SM_ERR_INVALID_INPUT; }

    if( dRelativeAccuracy < 1.0e-4 ) { dRelativeAccuracy = 1.0e-4; }
    if( dRelativeAccuracy > 1.0e-1 ) { dRelativeAccuracy = 1.0e-1; }

    SmExtent3d sBBox;
    SER( pFace->CalculateBoundingBox( sBBox ) );
    SmPoint3d sOrigin = sBBox.GetMid();

    double dDeltaVolume = 0.0;
    SmTArray<SmVector3d> sDeltaMoments;

    SER( pFace->ComputePreciseProperties( SM_OT_SAME,
                                          dRelativeAccuracy,
                                          sOrigin,
                                          sm_FaceAreaEstimate( sBBox ),
                                          /*dFaceThickness*/   0.0,
                                          rdArea,
                                          dDeltaVolume,
                                          sDeltaMoments,
                                          SM_PPF_AREA | SM_PPF_AREA_FIRST_MOMENTS ) );

    // Degenerate zero-area trim: division below would be meaningless.
    double dAbsArea = ( rdArea < 0.0 ) ? -rdArea : rdArea;
    if( dAbsArea < SM_EFF_ZERO )
        { return SM_ERR_INVALID_INPUT; }

    rCentroid = sOrigin + ( sDeltaMoments[0] / rdArea );
    rdArea    = dAbsArea;

    return SM_SUCCESS;

} // End SmApiFaceComputeCentroid

/*******************************************************************//**
PURPOSE --- Compute the axis-aligned bounding box of a single edge.

USAGE NOTES --- bTight=TRUE evaluates the edge's curve at sample points
    for a minimal box; FALSE returns the loose curve-control-polygon
    box.  Use FALSE for cheap clipping queries.
***********************************************************************/
SmApiStatus SmApiEdgeBoundingBox
(
    const SmEdge* pEdge,
    SmBoolean   bTight,
    SmPoint3d & rMin,
    SmPoint3d & rMax
)
{
    if( pEdge == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmExtent3d sBBox;

    if( bTight )
        { SER( pEdge->CalculateTightBoundingBox( &sBBox ) ); }
    else
        { SER( pEdge->CalculateBoundingBox( &sBBox ) ); }

    rMin = sBBox.GetMin();
    rMax = sBBox.GetMax();

    return SM_SUCCESS;

} // End SmApiEdgeBoundingBox

/*******************************************************************//**
PURPOSE --- Compute the 3D arc length of an edge over its parametric
    interval.

USAGE NOTES --- Wraps SmCurve::Length on the edge's underlying curve
    using the edge's GetInterval() to clip to the edge's portion of
    the curve.  dDesiredAccuracy is an absolute distance tolerance for
    the integration -- typical value is 1e-6 in modeling units.
***********************************************************************/
SmApiStatus SmApiEdgeComputeLength
(
    const SmEdge*  pEdge,
    double   dDesiredAccuracy,
    double & rdLength
)
{
    if( pEdge == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmCurve* pCurve = pEdge->GetCurve();
    if( pCurve == NULL )
        { return SM_ERR; }

    SmExtent1d sInterval = pEdge->GetInterval();

    SER( pCurve->Length( sInterval, dDesiredAccuracy, rdLength ) );

    return SM_SUCCESS;

} // End SmApiEdgeComputeLength

/*******************************************************************//**
PURPOSE --- Find the closest point on an edge's trimmed curve interval
    to a given point.

USAGE NOTES --- Wraps SmEdge::ClosestPoint, already scoped to
    GetInterval() rather than the curve's full natural range.
***********************************************************************/
SmApiStatus SmApiEdgeClosestPoint
(
    SmEdge*           pEdge,
    const SmPoint3d & crPoint,
    SmPoint3d &       rClosestPoint,
    double &          rdParameter,
    double &          rdDistance
)
{
    if( pEdge == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmBoolean bSuccess = FALSE;
    SER( pEdge->ClosestPoint( crPoint, bSuccess, rdParameter, rdDistance ) );

    if( !bSuccess )
        { return SM_ERR; }

    SmCurve* pCurve = pEdge->GetCurve();
    if( pCurve == NULL )
        { return SM_ERR; }

    SER( pCurve->EvaluatePoint( rdParameter, rClosestPoint ) );

    return SM_SUCCESS;

} // End SmApiEdgeClosestPoint

/*******************************************************************//**
PURPOSE --- Get the parametric interval an edge occupies on its
    underlying curve.

USAGE NOTES --- Wraps SmEdge::GetInterval(); distinct from the curve's
    own full natural range.
***********************************************************************/
SmApiStatus SmApiEdgeParameterRange
(
    const SmEdge*  pEdge,
    double & rdMin,
    double & rdMax
)
{
    if( pEdge == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmExtent1d sInterval = pEdge->GetInterval();
    rdMin = sInterval.GetMin();
    rdMax = sInterval.GetMax();

    return SM_SUCCESS;

} // End SmApiEdgeParameterRange

/*******************************************************************//**
PURPOSE --- Classify a curve parameter against an edge's trimmed
    interval.

USAGE NOTES --- SM_PC_VERTEX when the 3D point at dParameter is within
    the endpoint Vertex's own tolerance (SmTol::GetZoneTol3d), not by
    raw parameter comparison. SM_PC_EDGE within the interval, else
    SM_PC_UNKNOWN.
***********************************************************************/
SmApiStatus SmApiEdgeClassifyParameter
(
    const SmEdge*  pEdge,
    double   dParameter,
    int &    rClassification
)
{
    if( pEdge == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmCurve* pCurve = pEdge->GetCurve();
    if( pCurve == NULL )
        { return SM_ERR; }

    SmPoint3d sEvalPt;
    if( pCurve->EvaluatePoint( dParameter, sEvalPt ) == SM_SUCCESS )
    {
        SmVertex* pStart = pEdge->GetStartVertex();
        SmVertex* pEnd   = pEdge->GetEndVertex();
        if( pStart && sEvalPt.DistanceBetween( pStart->GetPoint() ) < SmTol::GetZoneTol3d( pStart ) )
            { rClassification = (int)SM_PC_VERTEX; return SM_SUCCESS; }
        if( pEnd && sEvalPt.DistanceBetween( pEnd->GetPoint() ) < SmTol::GetZoneTol3d( pEnd ) )
            { rClassification = (int)SM_PC_VERTEX; return SM_SUCCESS; }
    }

    SmExtent1d sInterval = pEdge->GetInterval();
    if( sInterval.ContainsValue( dParameter ) )
        { rClassification = (int)SM_PC_EDGE; }
    else
        { rClassification = (int)SM_PC_UNKNOWN; }

    return SM_SUCCESS;

} // End SmApiEdgeClassifyParameter

/*******************************************************************//**
PURPOSE --- Compute the unit tangent vector of an edge's curve at a
    given parameter.

USAGE NOTES --- Wraps SmCurve::Evaluate then unitizes the tangent.  The
    vector follows the curve's natural direction, which is opposite the edge's
    start->end traversal sense on a reversed edge.
***********************************************************************/
SmApiStatus SmApiEdgeTangent
(
    const SmEdge*      pEdge,
    double       dParameter,
    SmVector3d & rTangent
)
{
    if( pEdge == NULL )
        { return SM_ERR_INVALID_INPUT; }

    SmCurve* pCurve = pEdge->GetCurve();
    if( pCurve == NULL )
        { return SM_ERR; }

    SmVector3d aPointAndDerivatives[2];
    SER( pCurve->Evaluate( dParameter, 1, TRUE, aPointAndDerivatives ) );

    rTangent = aPointAndDerivatives[1];
    SER( rTangent.Unitize() );

    return SM_SUCCESS;

} // End SmApiEdgeTangent

/*******************************************************************//**
PURPOSE --- Get the 3D position of a vertex.

USAGE NOTES --- Trivial wrapper over SmVertex::GetPoint, exposed for
    SM_API symmetry so callers do not need to reach into kernel
    headers for the most common vertex query.
***********************************************************************/
SmApiStatus SmApiVertexGetPoint
(
    const SmVertex*   pVertex,
    SmPoint3d & rPoint
)
{
    if( pVertex == NULL )
        { return SM_ERR_INVALID_INPUT; }

    rPoint = pVertex->GetPoint();

    return SM_SUCCESS;

} // End SmApiVertexGetPoint
