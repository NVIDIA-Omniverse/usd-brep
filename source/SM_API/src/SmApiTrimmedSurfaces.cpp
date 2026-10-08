// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmTrimmedSurfaces.cpp

PURPOSE: 
    Contains popular, high level "C" type functions that create 
    primitive objects (cube, sphere, etc).

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/

#include "StdAfx.h"

#include "SmApiTrimmedSurfaces.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include "SmBrep.h"
#include "SmEdge.h"
#include "SmFace.h"
#include "SmSurface.h"
#include "SmTrimmingTools.h"
#include "SmMapPtrToPtr.h"

#if SM_DEBUG_CODE
static const SmVector3d s_kBlack(0, 0, 0);
static const SmVector3d s_kBlue (0, 0, 1);
static const SmVector3d s_kGreen(0, 1, 0);
#endif

/*******************************************************************//**
PURPOSE --- Trim a surface with model space curves  

NOTES ---  Assume all curves are oriented correctly
           The outer loop of curves must be first
           All loops must be head to tail

***********************************************************************/
SmApiStatus SmApiTrimSurfaceWith3dCurves
( 
    SmSurface* pSurface,             ///< [in ]: Surface to trim (consumed)
    const SmTArray<ULONG>& crCurveLoops,   ///< [in ]: Number of curves in each loop
    const SmTArray<SmCurve*>& cr3DCurves,  ///< [in ]: Trimming curves in head to tail order
    SmBrep*& rpResult               ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pSurface, &s_kBlack, 1, 1 );
        SmApiDraw( cr3DCurves, &s_kGreen, 1, 1 );
    }
#endif

    SmBrep* pBrep = NULL;

    SmBoolean bFirstLoopIsOuterLoop = TRUE;
    SmBoolean bLoopsAreOrientedCorrectly = TRUE;
    SmBoolean bLoopMayCrossASeam = TRUE;

    double dTol = 0.01;

    ULONG lNumCurves = cr3DCurves.GetSize();

    // Require all curves to already be in the right direction
    SmTArray<SmOrientType> sCurveOrientations;
    for( ULONG ii = 0; ii < lNumCurves; ii++ ) {
        sCurveOrientations.Add(SM_OT_SAME); 
    }

    SmTArray<SmPoint3d> loopPoints;

    SmExtent2d domain = pSurface->GetNaturalUVDomain();

    SER( SmTrimmingTools::TrimSurfaceWithModelSpaceCurves( *SmApiGetOrCreateContext(), dTol, crCurveLoops, cr3DCurves, sCurveOrientations, 
        loopPoints, pSurface, domain, SM_OT_SAME, bFirstLoopIsOuterLoop, bLoopsAreOrientedCorrectly, bLoopMayCrossASeam, pBrep ) );


#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    rpResult = pBrep;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiTrimSurfaceWith3dCurves


/*******************************************************************//**
PURPOSE --- Trim a surface by projecting curves in a specific direction
            And selecting the area to keep by reference point

NOTES ---   Reference point options:
            SM_TT_KEEP_POINT  : Keep the face that contains the ref point
            SM_TT_DELETE_POINT: Delete the face that contains the ref point
            SM_TT_SPLIT       : Split the object keeping both sides

***********************************************************************/
SmApiStatus SmApiTrimProjectParallel
( 
    SmObject* pObject,                 ///< [in ]: Either SmSurface or SmBrep to trim (consumed on success)
    SmBSplineCurve* crCurveToProject,  ///< [in ]: Curve to project to the faces
    const SmVector3d& crProjectionDirection, ///< [in ]: Defines direction of projection
    const SmPoint3d & crReferencePoint,      ///< [in ]: Used only if trimType is SM_TT_KEEP_POINT or SM_TT_DELETE_POINT.
    SmTrimType eTrimType,              ///< [in ]: SM_TT_KEEP_POINT, SM_TT_DELETE_POINT, SM_TT_SPLIT (see above)
    SmBrep*& rpResult                 ///< [out]: Resulting SmBrep
)                  
{
    rpResult = NULL;
    if( pObject == NULL || crCurveToProject == NULL )
        return SM_ERR_INVALID_INPUT;

    if( !pObject->IsKindOf(SmSurface_TYPE) && !pObject->IsKindOf(SmBrep_TYPE) )
        return SM_ERR_INVALID_INPUT;

    // A surface owned by a Brep is only one part of that Brep's geometry.
    // Consuming it independently would leave the owning topology inconsistent.
    if( pObject->IsKindOf(SmSurface_TYPE) &&
        ((SmSurface*)pObject)->GetOwner() != NULL )
        return SM_ERR_INVALID_INPUT;

    // A standalone surface is attached to a temporary Brep below and cannot be
    // detached again if the trim fails, so reject a zero direction up front.
    if( pObject->IsKindOf(SmSurface_TYPE) &&
        crProjectionDirection.IsZero() )
        return SM_ERR_INVALID_INPUT;
    
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        if( pObject->GetType() == SmBrep_TYPE ) {
            SmApiDraw( (SmBrep*)pObject, &s_kBlack, 1, 1 );
        }
        else {
            SmApiDraw( (SmSurface*)pObject, &s_kBlack, 1, 1 );
        }
        
        SmApiDraw( crCurveToProject, &s_kGreen, 1, 1 );
    }
#endif

    // create new brep from surface
    SmBrep* pBrep = NULL;

    if( pObject->IsKindOf( SmSurface_TYPE ) ) {
            pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
            SmSurface* pSrf = (SmSurface*)pObject;
            SmFace* pFace = NULL;
            SmStatus faceStat = pBrep->CreateFaceFromSurface( pSrf, pSrf->GetNaturalUVDomain(), pFace );
            if( faceStat != SM_SUCCESS )
            {
                // Free the temporary Brep only while it cannot own the caller's
                // surface; once the surface is attached, deleting the Brep would
                // delete it too.
                if( pSrf->GetOwner() == NULL )
                    delete pBrep;
                return faceStat;
            }
    }
    else if( pObject->IsKindOf( SmBrep_TYPE ) ) {
        pBrep = (SmBrep*)pObject;   // do we need to copy here?
    }
  

    // now project and trim
    SmApiStatus stat = pBrep->ProjectAndTrim( *crCurveToProject, crProjectionDirection,
                                       crReferencePoint, eTrimType, NULL, NULL, NULL );

    if(stat != SM_SUCCESS) return(stat);

    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 1, 1 );
    }
#endif


    return( SM_SUCCESS );
}


/*******************************************************************//**
PURPOSE --- Given a set of coplanar 3d curves, this function creates a face
            or a set of faces in the given region of the brep, on the plane 
            of the curves.  The face will be bounded by the given curves. 

NOTES --- The curves can be given in any order, with any orientation.
   They must not intersect each other though, and each start/end point
   must be coincident with exactly one other start/end point.

***********************************************************************/
SmApiStatus SmApiCreatePlanarFaces
( 
    const SmTArray<SmCurve*>& crPlanarCurves,  ///< [in ]: Planar bounding curves
    SmBrep*& rpResult                   ///< [out]: Resulting SmBrep
)
{

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( crPlanarCurves, &s_kBlack, 1, 1 );
    }
#endif

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmRegion* pRegion = pBrep->GetInfiniteRegion();

    double dTol = 0.01;

    SmTArray<SmFace*> sNewfaces;
    const SmStatus eStat = pBrep->CreatePlanarFacesWith3DCurves( pRegion, crPlanarCurves, dTol, sNewfaces );
    if( eStat != SM_SUCCESS )
    {
        // Free the Brep so a failure does not leak it. A late kernel failure can
        // leave some input curves already held by its edges, but the caller owns
        // the curves on failure. Detach them without deleting them; SetCurve
        // also clears their owner back-pointers. Skip geometry notifications
        // because the entire partial Brep is about to be destroyed.
        SmTArray<SmEdge*> sEdges;
        pBrep->GetEdges( sEdges );
        for( ULONG ii = 0; ii < sEdges.GetSize(); ++ii )
        {
            SmCurve* pEdgeCurve = sEdges[ii]->GetCurve();
            for( ULONG jj = 0; jj < crPlanarCurves.GetSize(); ++jj )
            {
                if( pEdgeCurve != crPlanarCurves[jj] )
                    continue;
                sEdges[ii]->SetCurve( NULL, FALSE, FALSE, FALSE );
                break;
            }
        }
        delete pBrep;
        rpResult = NULL;
        SER( eStat );
    }


#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    rpResult = pBrep;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 1, 1 );
    }
#endif


#if 0
#endif

    return( SM_SUCCESS );
} // SmApiCreatePlanarFaces
