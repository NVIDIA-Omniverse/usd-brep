// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmIntersectors.cpp

PURPOSE: 
    Contains popular high level "C" type functions that intersect two objects.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib
**********************************************************************/

#include "StdAfx.h"

#include "SmApiIntersectors.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include <SmCurve.h>
#include <SmSurface.h>
#include <SmFace.h>
#include <SmBrep.h>
#include <SmTol.h>
#include <SmPointClass.h>

#if SM_DEBUG_CODE
static const SmVector3d s_kBlack(0, 0, 0);
static const SmVector3d s_kBlue (0, 0, 1);
static const SmVector3d s_kGreen(0, 1, 0);
static const SmVector3d s_kRed  (1, 0, 0);
#endif

/*******************************************************************//**
PURPOSE --- Intersect two curves 

NOTES ---  If both curves are the same, use self intersection
***********************************************************************/
SmApiStatus SmApiIntersectCurves
( 
    SmCurve* pCurve1,                       ///< [in ]: Curve to intersect
    SmCurve* pCurve2,                       ///< [in ]: Curve to intersect
    SmTArray<SmPoint3d>* p3DPoints,         ///< [out]: Resulting intersection points
    SmTArray<double>* parametersOnCurve1,   ///< [out]: Resulting parameters on first curve
    SmTArray<double>* parametersOnCurve2   ///< [out]: Resulting parameters on second curve    
)       
{

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pCurve1, &s_kBlack, 1, 1 );
        SmApiDraw( pCurve2, &s_kBlack, 0, 0 );
    }
#endif

    SmSolutionArray sSolutions;
    double dTol = 0.001;

    SmStatus stat = SM_SUCCESS;
    if( pCurve1 == pCurve2 )
    {
        stat = pCurve1->GlobalCurveSelfIntersect( pCurve1->GetNaturalInterval(), dTol, sSolutions );
    }
    else
    {
        stat = pCurve1->GlobalCurveIntersect( pCurve1->GetNaturalInterval(),
                                      *pCurve2, pCurve2->GetNaturalInterval(), dTol, sSolutions );
    }
    if( stat != SM_SUCCESS )
        return( stat );

    // If no intersections return error
    if ( sSolutions.GetSize() < 1 ) 
        return (SM_ERR);

    SmPoint3d sPt;

    // Extract info from solution array
    for( ULONG ii = 0; ii < sSolutions.GetSize(); ii++ )
    {
        SmSolution& rSol = sSolutions[ii];

        if( parametersOnCurve1 )
            parametersOnCurve1->Add( rSol.m_vStart[0] );

        if( parametersOnCurve2 )
            parametersOnCurve2->Add( rSol.m_vStart[1] );

        // get all points on curve1 (3d points on curve2 are the same)
        if( p3DPoints )
        {
            pCurve1->EvaluatePoint ( rSol.m_vStart[0], sPt );
            p3DPoints->Add( sPt );
        }
    }

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( *p3DPoints, &s_kRed, 0, 0 );
    }
#endif

    return( SM_SUCCESS );

} // End SmApiIntersectCurves

/*******************************************************************//**
PURPOSE --- Intersect two surfaces 

NOTES ---  This can be expanded to return uv curves on each surface
***********************************************************************/
SmApiStatus SmApiIntersectSurfaces
( 
    SmSurface* pSurface1,                 ///< [in ]: Surface to intersect
    SmSurface* pSurface2,                 ///< [in ]: Surface to intersect
    SmTArray<SmCurve*> * pCurves3d       ///< [out]: Resulting 3d curves
)
{
    // check input
    if( pSurface1 == NULL || pSurface2 == NULL || pCurves3d == NULL )
        return( SM_ERR_INVALID_INPUT );

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pSurface1, &s_kBlack, 1, 1 );
        SmApiDraw( pSurface2, &s_kBlack, 0, 0 );
    }
#endif

    pCurves3d->RemoveAll();

    const SmContext* pContext = pSurface1->GetContext();

    SmBoolean bUseSurfaceEdges[2];
    bUseSurfaceEdges[0] = true;
    bUseSurfaceEdges[1] = true;

    SmApiStatus stat = pSurface1->GlobalSurfaceIntersect( *pContext, pSurface1->GetNaturalUVDomain(), 
                                       *pSurface2, pSurface2->GetNaturalUVDomain(),
                                       bUseSurfaceEdges, NULL, NULL, pCurves3d,
                                       NULL, NULL, NULL, NULL );
    // check result
    if( stat != SM_SUCCESS || pCurves3d->GetSize() == 0 )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( *pCurves3d, &s_kBlue, 0, 0 );
    }
#endif

    return( SM_SUCCESS );

} // End SmApiIntersectSurfaces

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmApiStatus SmApiIntersectFaces
(
    SmFace* pFace1,                      ///< [in ]: Pointer to first face
    SmFace* pFace2,                      ///< [in ]: Pointer to other face
    SmTArray<SmCurve*> & r3DCurves,      ///< [out]: Resulting 3d intersection curves
    SmTArray<SmPoint3d> * pOpt3DPoints  // (out, opt) Resulting 3d intersection points
)
{
 #if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pFace1, &s_kBlack, 1, 1 );
        SmApiDraw( pFace2, &s_kBlack, 0, 0 );
        
    }
#endif
    
    const SmContext* pContext = pFace1->GetContext();
    double dTol = pFace1->GetTolerance();
    double dAngTol = 20.0;

    SmStatus stat = pFace1->FaceIntersect( *pContext, pFace2, dTol, dAngTol, r3DCurves, NULL, NULL, pOpt3DPoints, NULL, NULL );
    if( stat != SM_SUCCESS )
    {
        // The kernel appends curves and points while processing intersection
        // segments. A later segment can fail after earlier output was added.
        SmObjsDelete<SmCurve*> cleanupCurves(&r3DCurves);
        if( pOpt3DPoints )
            pOpt3DPoints->RemoveAll();
        return( stat );
    }

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( r3DCurves, &s_kGreen, 0, 0 );
        if( pOpt3DPoints )
            SmApiDraw( *pOpt3DPoints, &s_kRed, 0, 0 );
    }
#endif

    return( SM_SUCCESS );

} // End SmApiIntersectFaces

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmApiStatus SmApiIntersectBreps
(
    SmBrep* pBrep1,                      ///< [in ]: Pointer to brep
    SmBrep* pBrep2,                      ///< [in ]: Pointer to other brep
    SmTArray<SmCurve*> & rCurves3d,      ///< [out]: Resulting 3d intersection curves
    SmTArray<SmPoint3d> * pOptPoints3d  ///< [out]: Resulting 3d intersection points (optional)
)
{
  if(pBrep1 == NULL || pBrep2 == NULL)
    return(SM_ERR_INVALID_INPUT);

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep1, NULL, 1, 1 );
    SmApiDraw( pBrep2, NULL, 0, 0 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = pBrep1->GetTolerance();
  double dAngTol = 20.0;

  SmApiStatus stat = pBrep1->BrepIntersect( *pContext, pBrep2, dTol, dAngTol, rCurves3d, pOptPoints3d );

  // check result
  if(stat != SM_SUCCESS || (rCurves3d.GetSize() == 0 && (pOptPoints3d == NULL || pOptPoints3d->GetSize() == 0)))
    return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rCurves3d, &s_kBlue, 0, 0 );
    SmApiDraw( *pOptPoints3d, &s_kRed, 0, 0 );
  }
#endif

  return(SM_SUCCESS);

} // End SmApiIntersectBreps

/*******************************************************************//**
PURPOSE --- Intersect a brep with a plane  

USAGE NOTES --- The result is an array of section curves

***********************************************************************/
SmApiStatus SmApiIntersectBrepWithPlane
(
    SmBrep*  pBrep,                         ///< [in ]: Pointer to brep
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane
    const SmVector3d& rPlaneNormal,         ///< [in ]: Normal to define direction of plane
    SmTArray<SmCurve*>& rSectionCurves     ///< [out]: Resulting section curves
)
{
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif

  if( pBrep == NULL )
    return( SM_ERR_INVALID_INPUT );

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmVector3d sPlaneNormal = rPlaneNormal;
  if( sPlaneNormal.Unitize() != SM_SUCCESS )
    return( SM_ERR_INVALID_INPUT );

  // The kernel resets the array it is given, so collect into a local one and
  // append, keeping any curves already in the caller's array.
  SmTArray<SmCurve*> sSectionCurves;
  SmStatus stat = pBrep->CreatePlanarSectionCurves( *pContext, rPlanePt, sPlaneNormal, NULL, NULL, &sSectionCurves, NULL, NULL, false );
  if( stat != SM_SUCCESS )
    return( stat );
  for( ULONG ii = 0; ii < sSectionCurves.GetSize(); ii++ )
    rSectionCurves.Add( sSectionCurves[ii] );

#ifdef SM_ASSERT_VALID
  for(ULONG ii = 0; ii < rSectionCurves.GetSize(); ii++)
    SM_ASSERT_VALID( rSectionCurves[ii] );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rSectionCurves, &s_kBlue, 0, 0 );
  }
#endif


  return(SM_SUCCESS);

} // End SmApiIntersectBrepWithPlane

/*******************************************************************//**
PURPOSE --- Append the intersections of a curve with a surface, optionally
            restricted to a trimmed face.

NOTES ---   With pOptFace, hits whose surface UV lies outside the face's trim
            loops (holes, trimmed-away regions) are dropped; overlap ranges are
            classified at their UV midpoint.  With bSkipDuplicates, a point
            within dTol of one already appended by this call is skipped, so a
            curve crossing a shared edge is reported once; likewise an overlap
            whose ends match one already appended (a curve lying along a shared
            edge).
***********************************************************************/
static SmStatus sm_HasOwnCurveWithEnds
(
    const SmTArray<SmCurve*>& crCurves,
    ULONG                     lFirstOwnCurve,
    const SmPoint3d&          crStart,
    const SmPoint3d&          crEnd,
    double                    dTol,
    SmBoolean&                rbFound
)
{
    rbFound = FALSE;
    for( ULONG ii = lFirstOwnCurve; ii < crCurves.GetSize(); ii++ )
    {
        SmExtent1d sIvl = crCurves[ii]->GetNaturalInterval();
        SmPoint3d sStart, sEnd;
        SER( crCurves[ii]->EvaluatePoint( sIvl.GetMin(), sStart ) );
        SER( crCurves[ii]->EvaluatePoint( sIvl.GetMax(), sEnd ) );
        if(    ( sStart.DistanceBetween( crStart ) <= dTol && sEnd.DistanceBetween( crEnd ) <= dTol )
            || ( sStart.DistanceBetween( crEnd ) <= dTol && sEnd.DistanceBetween( crStart ) <= dTol ) )
        {
            rbFound = TRUE;
            break;
        }
    }
    return SM_SUCCESS;
}

// Remove only this call's outputs; preserve caller-owned entries on failure.
static void sm_RestoreCurveSurfaceHits
(
    SmTArray<SmCurve*>&  rCurves,
    SmTArray<SmPoint3d>& rPoints,
    ULONG                lFirstOwnCurve,
    ULONG                lFirstOwnPoint
)
{
    for( ULONG ii = lFirstOwnCurve; ii < rCurves.GetSize(); ++ii )
        delete rCurves[ii];
    rCurves.SetSize(lFirstOwnCurve);
    rPoints.SetSize(lFirstOwnPoint);
}

static SmStatus sm_AppendCurveSurfaceHits
(
    const SmContext&     crContext,
    SmCurve*             pCurve,
    SmSurface*           pSurface,
    const SmExtent2d&    crUVDomain,
    SmFace*              pOptFace,
    double               dTol,
    SmBoolean            bSkipDuplicates,
    ULONG                lFirstOwnCurve,
    ULONG                lFirstOwnPoint,
    SmTArray<SmCurve*>&  rCurves,
    SmTArray<SmPoint3d>& rPoints
)
{
    SmSolutionArray sSolutions;
    SmStatus stat = pSurface->GlobalCurveIntersect(
        crUVDomain, *pCurve, pCurve->GetNaturalInterval(), dTol, sSolutions );
    if( stat != SM_SUCCESS )
        return stat;

    SmPoint3d sPt;
    for( ULONG ii = 0; ii < sSolutions.GetSize(); ii++ )
    {
        SmSolution& rSol = sSolutions[ii];
        SmBoolean bRange = ( rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES );

        if( pOptFace )
        {
            SmPoint2d sUV( rSol.m_vStart[1], rSol.m_vStart[2] );
            if( bRange )
                sUV = SmPoint2d( 0.5 * ( rSol.m_vStart[1] + rSol.m_vEnd[1] ),
                                 0.5 * ( rSol.m_vStart[2] + rSol.m_vEnd[2] ) );
            SmPointClassification sPC;
            SmStatus classStat = pOptFace->PointClassify(
                sUV, SmTol::GetZoneTol3d( pOptFace ), FALSE, TRUE, sPC );
            if( classStat != SM_SUCCESS )
                return classStat;
            if( sPC.GetPointClass() == SM_PC_UNKNOWN )
                continue;
        }

        if( bRange )
        {
            if( bSkipDuplicates )
            {
                SmPoint3d sStart, sEnd;
                SER( pCurve->EvaluatePoint( rSol.m_vStart[0], sStart ) );
                SER( pCurve->EvaluatePoint( rSol.m_vEnd[0], sEnd ) );
                SmBoolean bDuplicate = FALSE;
                SER( sm_HasOwnCurveWithEnds( rCurves, lFirstOwnCurve, sStart, sEnd, dTol, bDuplicate ) );
                if( bDuplicate )
                    continue;
            }

            SmCurve* pTrimmedCurve = NULL;
            SmStatus copyStat = pCurve->Copy( crContext, pTrimmedCurve );
            if( copyStat != SM_SUCCESS || pTrimmedCurve == NULL )
            {
                delete pTrimmedCurve;
                return copyStat != SM_SUCCESS ? copyStat : SM_ERR;
            }
            SmExtent1d sIvl( rSol.m_vStart[0], rSol.m_vEnd[0] );
            SmStatus trimStat = pTrimmedCurve->Trim( sIvl );
            if( trimStat != SM_SUCCESS )
            {
                delete pTrimmedCurve;
                return trimStat;
            }
            rCurves.Add( pTrimmedCurve );
            continue;
        }

        SER( pCurve->EvaluatePoint( rSol.m_vStart[0], sPt ) );
        SmBoolean bDuplicate = FALSE;
        if( bSkipDuplicates )
            for( ULONG jj = lFirstOwnPoint; jj < rPoints.GetSize() && !bDuplicate; jj++ )
                bDuplicate = ( rPoints[jj].DistanceBetween( sPt ) <= dTol );
        if( !bDuplicate )
            rPoints.Add( sPt );
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Intersect a curve and a surface

USAGE NOTES ---

***********************************************************************/
SmApiStatus SmApiIntersectCurveSurface
(
    SmCurve* pCurve,                        ///< [in ]: Pointer to curve                                              <br>
    SmSurface*  pSurface,                   ///< [in ]: Pointer to surface                                            <br>
    SmTArray<SmCurve*>& rCurves,            ///< [out]: Resulting intersection curves                                 <br>
    SmTArray<SmPoint3d>& rPoints           ///< [out]: Resulting intersection points                                 <br>
)
{
    if( pCurve == NULL || pSurface == NULL )
        return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pCurve, NULL, 1, 1 );
    SmApiDraw( pSurface, NULL, 0, 0 );
  }
#endif

    double dTol = 0.001;
    const ULONG lFirstOwnCurve = rCurves.GetSize();
    const ULONG lFirstOwnPoint = rPoints.GetSize();
    SmStatus stat = sm_AppendCurveSurfaceHits( *SmApiGetOrCreateContext(), pCurve, pSurface,
        pSurface->GetNaturalUVDomain(), NULL, dTol, FALSE,
        lFirstOwnCurve, lFirstOwnPoint, rCurves, rPoints );
    if( stat != SM_SUCCESS )
    {
        sm_RestoreCurveSurfaceHits(rCurves, rPoints, lFirstOwnCurve, lFirstOwnPoint);
        return stat;
    }

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rPoints, &s_kRed, 0, 0 );
  }
#endif

    if( rPoints.GetSize() == lFirstOwnPoint && rCurves.GetSize() == lFirstOwnCurve )
        return SM_ERR;

    return SM_SUCCESS;

} // End SmIntersectCurveSurface

/*******************************************************************//**
PURPOSE --- Intersect a curve and a face  

USAGE NOTES ---

***********************************************************************/
SmApiStatus SmApiIntersectCurveFace
(
    SmCurve* pCurve,                        ///< [in ]: Pointer to curve                                              <br>
    SmFace*  pFace,                         ///< [in ]: Pointer to surface                                            <br>
    SmTArray<SmCurve*>& rCurves,            ///< [out]: Resulting intersection curves                                 <br>
    SmTArray<SmPoint3d>& rPoints           ///< [out]: Resulting intersection points                                 <br>
)
{
    if( pCurve == NULL || pFace == NULL )
        return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pCurve, NULL, 1, 1 );
    SmApiDraw( pFace, NULL, 0, 0 );
  }
#endif

    SmSurface* pSurface = pFace->GetSurface();
    if( pSurface == NULL )
        return SM_ERR;

    double dTol = 0.001;
    const ULONG lFirstOwnCurve = rCurves.GetSize();
    const ULONG lFirstOwnPoint = rPoints.GetSize();
    SmStatus stat = sm_AppendCurveSurfaceHits( *SmApiGetOrCreateContext(), pCurve, pSurface,
        pFace->GetUVDomain(), pFace, dTol, FALSE,
        lFirstOwnCurve, lFirstOwnPoint, rCurves, rPoints );
    if( stat != SM_SUCCESS )
    {
        sm_RestoreCurveSurfaceHits(rCurves, rPoints, lFirstOwnCurve, lFirstOwnPoint);
        return stat;
    }

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rPoints, &s_kRed, 0, 0 );
  }
#endif

    if( rPoints.GetSize() == lFirstOwnPoint && rCurves.GetSize() == lFirstOwnCurve )
        return SM_ERR;

    return SM_SUCCESS;

} // End SmIntersectCurveFace

/*******************************************************************//**
PURPOSE --- Intersect a curve and a brep  

USAGE NOTES ---

***********************************************************************/
SmApiStatus SmApiIntersectCurveBrep
(
    SmCurve* pCurve,                        ///< [in ]: Pointer to curve                                              <br>
    SmBrep*  pBrep,                         ///< [in ]: Pointer to surface                                            <br>
    SmTArray<SmCurve*>& rCurves,            ///< [out]: Resulting intersection curves                                 <br>
    SmTArray<SmPoint3d>& rPoints           ///< [out]: Resulting intersection points                                 <br>
)
{
    if( pCurve == NULL || pBrep == NULL )
        return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pCurve, NULL, 1, 1 );
    SmApiDraw( pBrep, NULL, 0, 0 );
  }
#endif

    const SmContext& ctx = *SmApiGetOrCreateContext();

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );

    double dTol = 0.001;
    const ULONG lFirstOwnCurve = rCurves.GetSize();
    const ULONG lFirstOwnPoint = rPoints.GetSize();

    for( ULONG ff = 0; ff < sFaces.GetSize(); ff++ )
    {
        SmFace* pFace = sFaces[ff];
        SmSurface* pSurface = pFace->GetSurface();
        if( pSurface == NULL )
            continue;

        SmStatus faceStat = sm_AppendCurveSurfaceHits( ctx, pCurve, pSurface,
            pFace->GetUVDomain(), pFace, dTol, TRUE, lFirstOwnCurve, lFirstOwnPoint,
            rCurves, rPoints );
        if( faceStat != SM_SUCCESS )
        {
            // Drop the hits this call already appended rather than return a
            // partial result.
            sm_RestoreCurveSurfaceHits(rCurves, rPoints, lFirstOwnCurve, lFirstOwnPoint);
            return faceStat;
        }
    }

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rPoints, &s_kRed, 0, 0 );
  }
#endif

    if( rPoints.GetSize() == lFirstOwnPoint && rCurves.GetSize() == lFirstOwnCurve )
        return SM_ERR;

    return SM_SUCCESS;

} // End SmIntersectCurveBrep
