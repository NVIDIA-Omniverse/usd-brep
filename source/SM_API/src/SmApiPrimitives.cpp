// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmPrimitives.cpp

PURPOSE: 
    Contains popular high level "C" type functions that create 
    primitive objects (cube, sphere, etc).

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/

#include "SmTypes.h"
#include "StdAfx.h"

#include "SmApiPrimitives.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include "SmBrep.h"
#include "SmCurve.h"
#include "SmPrimitiveCreation.h"
#include <SmLine.h>
#include <SmSurface.h>
#include <SmPlane.h>
#include <SmCone.h>
#include <SmCylinder.h>
#include <SmSphere.h>
#include <SmTorus.h>
#include <SmTol.h>

/*******************************************************************//**
PURPOSE --- True for a finite dimension (radius, length, height) above zero.
***********************************************************************/
static bool SmApiIsPositiveDim( double dValue )
{
    return SM_IS_VALID_DOUBLE( dValue ) && dValue > SM_EFF_ZERO;
}

/*******************************************************************//**
PURPOSE --- True for a finite, non-zero sphere radius. A negative radius is a
            valid inside-out (reversed parameterization) sphere.
***********************************************************************/
static bool SmApiIsNonZeroRadius( double dRadius )
{
    return SM_IS_VALID_DOUBLE( dRadius ) && smos_Fabs( dRadius ) > SM_EFF_ZERO;
}

/*******************************************************************//**
PURPOSE --- Valid cone dimensions: finite radii that are not negative and
            not both zero, and a positive height.
***********************************************************************/
static bool SmApiConeDimsValid( double dRadiusBase, double dRadiusTop, double dHeight )
{
    if( !SM_IS_VALID_DOUBLE( dRadiusBase ) || !SM_IS_VALID_DOUBLE( dRadiusTop ) )
        return false;
    if( dRadiusBase < 0.0 || dRadiusTop < 0.0 )
        return false;
    if( dRadiusBase <= SM_EFF_ZERO && dRadiusTop <= SM_EFF_ZERO )
        return false;
    return SmApiIsPositiveDim( dHeight );
}

/*******************************************************************//**
PURPOSE --- Validate the public skin-profile structure before any input
            curve is copied or passed to the consuming kernel operation.
***********************************************************************/
static SmApiStatus SmApiValidateSkinPrimitiveInputs
(
    const SmTArray<ULONG>&          rCurvesInEachProfile,
    const SmTArray<const SmCurve*>& r3DCurves,
    ULONG                           lDegree
)
{
    if( rCurvesInEachProfile.GetSize() < 2 || lDegree < 1 || lDegree > 3 )
        return SM_ERR_INVALID_INPUT;

    ULONG lCurveCount = 0;
    for( ULONG ii = 0; ii < rCurvesInEachProfile.GetSize(); ii++ )
    {
        const ULONG lProfileCurveCount = rCurvesInEachProfile[ii];
        if( lProfileCurveCount == 0 || lProfileCurveCount > r3DCurves.GetSize() - lCurveCount )
            return SM_ERR_INVALID_INPUT;
        lCurveCount += lProfileCurveCount;
    }

    if( lCurveCount != r3DCurves.GetSize() )
        return SM_ERR_INVALID_INPUT;

    for( ULONG ii = 0; ii < r3DCurves.GetSize(); ii++ )
        if( r3DCurves[ii] == NULL )
            return SM_ERR_INVALID_INPUT;

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Deep-copy caller-owned skin profiles for the consuming legacy
            SmPrimitiveCreation::CreateSkinPrimitive operation.
***********************************************************************/
static SmApiStatus SmApiCopySkinPrimitiveCurves
(
    const SmContext&                crContext,
    const SmTArray<const SmCurve*>& rInputCurves,
    SmTArray<SmCurve*>&             rCurveCopies
)
{
    rCurveCopies.ReSet();
    for( ULONG ii = 0; ii < rInputCurves.GetSize(); ii++ )
    {
        SmCurve* pCopy = NULL;
        SmStatus eStat = rInputCurves[ii]->Copy( crContext, pCopy );
        if( eStat != SM_SUCCESS )
        {
            delete pCopy;
            return eStat;
        }
        if( pCopy == NULL )
            return SM_ERR_NULL_POINTER;
        rCurveCopies.Add( pCopy );
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Create a planar rectangular surface as a brep  

NOTES ---   Assume x-y plane
            Reference point at corner

***********************************************************************/   
SmApiStatus SmApiCreatePlane
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of bottom left corner of plane
    double dX,                   ///< [in ]: Distance along X axis of frame
    double dY,                   ///< [in ]: Distance along Y axis of frame
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    double dTol = 1.0e-5;
    rpResult = SmPrimitiveCreation::CreateRectangle(*SmApiGetOrCreateContext(), dTol, dX, dY, sRefFrame);
    if( rpResult == NULL || rpResult->GetNumFaces() == 0 )
    {
        delete rpResult;
        rpResult = NULL;
        return( SM_ERR );
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = TRUE ;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // End SmCreatePlane

/*******************************************************************//**
PURPOSE --- Create a planar rectangular surface as a bounded SmPlane

NOTES ---   Assume x-y plane
            Reference point at corner

***********************************************************************/   
SmApiStatus SmApiCreatePlane
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of minU-minV corner
    double dLength,              ///< [in ]: Length along x-axis
    double dWidth,               ///< [in ]: Width along y-axis
    SmPlane*& rpResult          ///< [out]: Resulting SmPlane
)
{
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmPlane* pPlane = NULL;
    long status = SmPlane::CreateCanonical(*SmApiGetOrCreateContext(), sRefFrame, pPlane); 

    if( status != SM_SUCCESS || pPlane == NULL ) {
        delete pPlane;
        rpResult = NULL;
        return ( status != SM_SUCCESS ) ? status : SM_ERR;
    }

    SmExtent2d sExtent;
    sExtent.SetMinMax( 0, 0, dLength, dWidth );
    status = pPlane->AdjustSTEPUVDomain( sExtent );
    if( status != SM_SUCCESS ) {
        delete pPlane;
        rpResult = NULL;
        return status;
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pPlane );
#endif

    rpResult = pPlane;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = TRUE ;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreatePlane

/*******************************************************************//**
PURPOSE --- Create a planar circular plane as a brep  

NOTES ---   Assume x-y plane

***********************************************************************/   
SmApiStatus SmApiCreatePlanarCircle
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of bottom left corner of plane
    double dRadius,              ///< [in ]: Radius of circle
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    double dTol = 1.0e-5;
    rpResult = SmPrimitiveCreation::CreateCircle(*SmApiGetOrCreateContext(), dTol, dRadius, sRefFrame);
    if( rpResult == NULL || rpResult->GetNumFaces() == 0 )
    {
        delete rpResult;
        rpResult = NULL;
        return( SM_ERR );
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = TRUE ;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // End SmApiCreatePlanarCircle


/*******************************************************************//**
PURPOSE --- Create a six-sided box or cube solid.  

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateBox
( 
    const SmVector3d& crOrigin,       ///< [in ]: Position at lower left corner of the solid
    double dLength,             ///< [in ]: Distance along the X axis
    double dWidth,              ///< [in ]: Distance along the Y axis
    double dHeight,             ///< [in ]: Distance along the Z axis
    SmBrep*& rpResult          ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    if( !SmApiIsPositiveDim( dLength ) || !SmApiIsPositiveDim( dWidth ) || !SmApiIsPositiveDim( dHeight ) )
        return( SM_ERR_INVALID_INPUT );

    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    rpResult = NULL;
    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateBox( dLength, dWidth, dHeight, sRefFrame ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID(pBrep);
#endif


#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreateBox


/*******************************************************************//**
PURPOSE --- Create a solid sphere with poles.  

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateSphere
( 
    const SmVector3d& crOrigin,      ///< [in ]: Center of sphere
    double dRadius,            ///< [in ]: Distance radius of sphere
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    if( !SmApiIsNonZeroRadius( dRadius ) )
        return( SM_ERR_INVALID_INPUT );

    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateSphere( dRadius, 0.0, 360.0, sRefFrame ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreateSphere

/*******************************************************************//**
PURPOSE --- Create a solid cylinder  

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateCylinder
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of base
    double dRadius,              ///< [in ]: Radius
    double dHeight,              ///< [in ]: Distance bewteen base and top
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    return SmApiCreateCone(crOrigin, dRadius, dRadius, dHeight, rpResult);

} // SmApiCreateCylinder

/*******************************************************************//**
PURPOSE --- Create a cylinder surface

NOTES ---   Caps not created

***********************************************************************/   
SmApiStatus SmApiCreateCylinder
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of base
    double dRadius,              ///< [in ]: Radius
    double dHeight,              ///< [in ]: Distance bewteen base and top
    SmSurface*& rpResult        ///< [out]: Resulting Cylinder 
)
{
    rpResult = NULL;
    if( !SmApiIsPositiveDim( dRadius ) || !SmApiIsPositiveDim( dHeight ) )
        return( SM_ERR_INVALID_INPUT );

    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmCylinder* pCylinder = NULL;
    long status = SmCylinder::CreateCanonical(*SmApiGetOrCreateContext(), sRefFrame, dRadius, pCylinder); 

    if( status != SM_SUCCESS || pCylinder == NULL ) {
        delete pCylinder;
        rpResult = NULL;
        return ( status != SM_SUCCESS ) ? status : SM_ERR;
    }

    SmExtent2d sExtent;
    sExtent.SetMinMax( 0, 0, 360.0, dHeight );
    status = pCylinder->AdjustSTEPUVDomain( sExtent );
    if( status != SM_SUCCESS ) {
        delete pCylinder;
        rpResult = NULL;
        return status;
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCylinder );
#endif

    rpResult = pCylinder;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreateCylinder

/*******************************************************************//**
PURPOSE --- Create a solid cone  

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateCone
( 
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of base
    double dRadiusBase,          ///< [in ]: Radius of base
    double dRadiusTop,           ///< [in ]: Radius of top
    double dHeight,              ///< [in ]: Distance bewteen base and top
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    if( !SmApiConeDimsValid( dRadiusBase, dRadiusTop, dHeight ) )
        return( SM_ERR_INVALID_INPUT );

    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateCone( dHeight, dRadiusBase, dRadiusTop, 0.0, 360.0, sRefFrame ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreateCone

/*******************************************************************//**
PURPOSE --- Create a cone surface

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateCone
( 
    const SmVector3d& crOrigin,      ///< [in ]: Position of center of base
    double dRadiusBase,        ///< [in ]: Radius at base
    double dRadiusTop,         ///< [in ]: Radius at top (for truncated cones)
    double dHeight,            ///< [in ]: Height
    SmSurface*& rpResult      ///< [out]: Resulting SmCone
)
{
    rpResult = NULL;
    if( !SmApiConeDimsValid( dRadiusBase, dRadiusTop, dHeight ) )
        return( SM_ERR_INVALID_INPUT );

    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmCone* pCone = NULL;
    long status = SmCone::CreateCanonical(*SmApiGetOrCreateContext(), sRefFrame, dRadiusBase, dRadiusTop, dHeight, pCone); 

    if( status != SM_SUCCESS || pCone == NULL ) {
        delete pCone;
        rpResult = NULL;
        return ( status != SM_SUCCESS ) ? status : SM_ERR;
    }

    SmExtent2d sExtent;
    sExtent.SetMinMax( 0, 0, 360.0, dHeight );
    status = pCone->AdjustSTEPUVDomain( sExtent );
    if( status != SM_SUCCESS ) {
        delete pCone;
        rpResult = NULL;
        return status;
    }

    rpResult = pCone;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreateCone

/*******************************************************************//**
PURPOSE --- Create a solid torus   

NOTES ---

***********************************************************************/   
SmApiStatus SmApiCreateTorus 
(
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of torus
    double dRadiusMajor,         ///< [in ]: Major Radius
    double dRadiusMinor,         ///< [in ]: Minor Radius
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateTorus( dRadiusMajor, dRadiusMinor, 0.0, 360.0, sRefFrame ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( SM_SUCCESS );

} // SmApiCreateTorus

/*******************************************************************//**
PURPOSE --- Create a surface torus   

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateTorus 
(
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of torus
    double dRadiusMajor,         ///< [in ]: Major Radius
    double dRadiusMinor,         ///< [in ]: Minor Radius
    SmSurface*& rpResult        ///< [out]: Resulting SmTorus
)
{
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmTorus* pTorus = NULL;
    SmApiStatus status = SmTorus::CreateCanonical(*SmApiGetOrCreateContext(), sRefFrame, dRadiusMajor, dRadiusMinor, pTorus);
    if( status != SM_SUCCESS || pTorus == NULL ) {
        delete pTorus;
        rpResult = NULL;
        return ( status != SM_SUCCESS ) ? status : SM_ERR;
    }

    rpResult = pTorus;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif


    return( status );

} // SmApiCreateTorus

/*******************************************************************//**
PURPOSE --- Create a pyramid with a square base and an apex point

NOTES --- Square base lies in the XY plane, centered on crBaseCenter.
          Apex is dHeight above the base center along +Z. On failure,
          rpResult is set to NULL and the partially built brep is deleted.

***********************************************************************/
SmApiStatus SmApiCreatePyramid
(
    const SmVector3d& crBaseCenter,  ///< [in ]: Position of center of base                                  <br>
    double dLength,            ///< [in ]: Length and Width                                            <br>
    double dHeight,            ///< [in ]: Height at apex                                              <br>
    SmBrep*& rpResult         ///< [out]: Resulting SmBrep                                            <br>
)
{
  rpResult = NULL;

  if( dLength <= 0.0 || dHeight <= 0.0 )
    return( SM_ERR_INVALID_INPUT );

  const SmContext* pContext = SmApiGetOrCreateContext();
  const double dHalf = dLength / 2.0;

  // Square base corners (CCW in the XY plane) and the apex above the center.
  SmPoint3d sBase[4];
  sBase[0] = crBaseCenter + SmVector3d( -dHalf, -dHalf, 0.0 );
  sBase[1] = crBaseCenter + SmVector3d(  dHalf, -dHalf, 0.0 );
  sBase[2] = crBaseCenter + SmVector3d(  dHalf,  dHalf, 0.0 );
  sBase[3] = crBaseCenter + SmVector3d( -dHalf,  dHalf, 0.0 );
  SmPoint3d sApex = crBaseCenter + SmVector3d( 0.0, 0.0, dHeight );

  SmBrep* pBrep = new (*pContext) SmBrep();
  SmRegion* pRegion = pBrep->GetInfiniteRegion();
  const double dTol = 1.0e-5;

  // Build the square base + four triangular side faces and stitch them into a
  // closed, consistently oriented solid. Wrapped in a lambda so any SER()
  // early-return is caught below and pBrep is cleaned up rather than leaked.
  auto buildPyramid = [&]() -> SmStatus
  {
    // Square base face: c0 -> c1 -> c2 -> c3.
    {
      SmTArray<SmCurve*> sLoop;
      for( int ii = 0; ii < 4; ii++ )
      {
        SmLine* pLine = NULL;
        SER( SmLine::CreateLineSegment( *pContext, 3, sBase[ii], sBase[(ii + 1) % 4], pLine ) );
        sLoop.Add( pLine );
      }
      SmFace* pFace = NULL;
      SER( pBrep->CreatePlanarFaceWith3DCurves( pRegion, sLoop, dTol, pFace ) );
    }

    // Four triangular side faces: c_i -> c_{i+1} -> apex.
    for( int ii = 0; ii < 4; ii++ )
    {
      const SmPoint3d& sP0 = sBase[ii];
      const SmPoint3d& sP1 = sBase[(ii + 1) % 4];

      SmTArray<SmCurve*> sLoop;
      SmLine* pLine = NULL;
      SER( SmLine::CreateLineSegment( *pContext, 3, sP0,   sP1,   pLine ) ); sLoop.Add( pLine );
      SER( SmLine::CreateLineSegment( *pContext, 3, sP1,   sApex, pLine ) ); sLoop.Add( pLine );
      SER( SmLine::CreateLineSegment( *pContext, 3, sApex, sP0,   pLine ) ); sLoop.Add( pLine );

      SmFace* pFace = NULL;
      SER( pBrep->CreatePlanarFaceWith3DCurves( pRegion, sLoop, dTol, pFace ) );
    }

    SER( pBrep->StitchAndOrient() );

    return( SM_SUCCESS );
  };

  SmStatus stat = buildPyramid();
  if( stat != SM_SUCCESS )
  {
    delete pBrep;
    rpResult = NULL;
    return( stat );
  }

  rpResult = pBrep;

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( rpResult, NULL, 1, 1 );
  }
#endif


  return SM_SUCCESS;

} // End SmApiCreatePyramid

/*******************************************************************//**
PURPOSE --- Create a cylindrical box (hollow cylinder sector)

NOTES ---   Defined by inside/outside radii and start/end angles

***********************************************************************/
SmApiStatus SmApiCreateCylindricalBox
(
    const SmVector3d& crOrigin,        ///< [in ]: Position of origin on Z axis at base
    double dLength,              ///< [in ]: Length along Z axis
    double dInsideRadius,        ///< [in ]: Inside radius from Z axis
    double dOutsideRadius,       ///< [in ]: Outside radius from Z axis
    double dStartAngleDeg,       ///< [in ]: Start angle from X axis in XY plane
    double dEndAngleDeg,         ///< [in ]: End angle from X axis in XY plane
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    if( !SmApiIsPositiveDim( dLength ) || !SmApiIsPositiveDim( dInsideRadius ) ||
        !SM_IS_VALID_DOUBLE( dOutsideRadius ) || dOutsideRadius <= dInsideRadius + SM_EFF_ZERO ||
        !SM_IS_VALID_DOUBLE( dStartAngleDeg ) || !SM_IS_VALID_DOUBLE( dEndAngleDeg ) ||
        dStartAngleDeg < -360.0 || dStartAngleDeg > 360.0 ||
        dEndAngleDeg <= dStartAngleDeg + SM_EFF_ZERO || dEndAngleDeg - dStartAngleDeg > 360.0 )
        return( SM_ERR_INVALID_INPUT );

    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    rpResult = NULL;
    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateCylindricalBox( dLength, dInsideRadius, dOutsideRadius, dStartAngleDeg, dEndAngleDeg, sRefFrame ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreateCylindricalBox

/*******************************************************************//**
PURPOSE --- Create a partial sphere with start and end angles

NOTES ---

***********************************************************************/
SmApiStatus SmApiCreatePartialSphere
(
    const SmVector3d& crOrigin,        ///< [in ]: Center of sphere
    double dRadius,              ///< [in ]: Radius of sphere
    double dStartAngleDeg,       ///< [in ]: Start angle in XY plane from X axis
    double dEndAngleDeg,         ///< [in ]: End angle in XY plane from X axis
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateSphere( dRadius, dStartAngleDeg, dEndAngleDeg, sRefFrame ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreatePartialSphere

/*******************************************************************//**
PURPOSE --- Create a sphere without singularities at the poles

NOTES ---   Uses SmPrimitiveCreation::CreateSphereNoPole which
            constructs a sphere from patches that avoid degenerate
            vertices at the poles.

***********************************************************************/
SmApiStatus SmApiCreateSphereNoPole
(
    const SmVector3d& crCenter,        ///< [in ]: Center of sphere
    double dRadius,              ///< [in ]: Radius of sphere
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    if( !SmApiIsNonZeroRadius( dRadius ) )
        return( SM_ERR_INVALID_INPUT );

    const SmContext* pContext = SmApiGetOrCreateContext();

    rpResult = NULL;
    SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SER( SmPrimitiveCreation::CreateSphereNoPole( pContext, dRadius, crCenter, SM_ZONE_TOL_3D, pBrep ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreateSphereNoPole

/*******************************************************************//**
PURPOSE --- Create a partial cone with start and end angles

NOTES ---

***********************************************************************/
SmApiStatus SmApiCreatePartialCone
(
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of base
    double dRadiusBase,          ///< [in ]: Radius at base
    double dRadiusTop,           ///< [in ]: Radius at top
    double dHeight,              ///< [in ]: Height of cone
    double dStartAngleDeg,       ///< [in ]: Start angle in XY plane from X axis
    double dEndAngleDeg,         ///< [in ]: End angle in XY plane from X axis
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateCone( dHeight, dRadiusBase, dRadiusTop, dStartAngleDeg, dEndAngleDeg, sRefFrame ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreatePartialCone

/*******************************************************************//**
PURPOSE --- Create a cone with elliptic ends and independent tilts

NOTES ---

***********************************************************************/
SmApiStatus SmApiCreateConeEllipticEnds
(
    double dFrontRadius,         ///< [in ]: Radius at front end
    double dRearRadius,          ///< [in ]: Radius at rear end
    double dHeight,              ///< [in ]: Distance between end-ellipse centers
    double dFrontYTiltDeg,       ///< [in ]: Front end tilt about X axis
    double dRearYTiltDeg,        ///< [in ]: Rear end tilt about X axis
    double dFrontXTiltDeg,       ///< [in ]: Front end tilt about Y axis
    double dRearXTiltDeg,        ///< [in ]: Rear end tilt about Y axis
    SmBoolean bEndCaps,          ///< [in ]: TRUE = close off end caps
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SER( SmPrimitiveCreation::CreateConeEllipticEnds(
        dFrontRadius, dRearRadius, dHeight,
        dFrontYTiltDeg, dRearYTiltDeg,
        dFrontXTiltDeg, dRearXTiltDeg,
        bEndCaps, pBrep ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    if( pBrep )
        SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe && pBrep ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreateConeEllipticEnds

/*******************************************************************//**
PURPOSE --- Create a partial cylinder with start and end angles

NOTES ---   Delegates to SmApiCreatePartialCone with equal radii

***********************************************************************/
SmApiStatus SmApiCreatePartialCylinder
(
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of base
    double dRadius,              ///< [in ]: Radius
    double dHeight,              ///< [in ]: Height
    double dStartAngleDeg,       ///< [in ]: Start angle in XY plane from X axis
    double dEndAngleDeg,         ///< [in ]: End angle in XY plane from X axis
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    return SmApiCreatePartialCone( crOrigin, dRadius, dRadius, dHeight, dStartAngleDeg, dEndAngleDeg, rpResult);

} // SmApiCreatePartialCylinder

/*******************************************************************//**
PURPOSE --- Create a partial torus with start and end angles

NOTES ---

***********************************************************************/
SmApiStatus SmApiCreatePartialTorus
(
    const SmVector3d& crOrigin,        ///< [in ]: Position of center of torus
    double dRadiusMajor,         ///< [in ]: Major radius
    double dRadiusMinor,         ///< [in ]: Minor radius
    double dStartAngleDeg,       ///< [in ]: Start angle in XY plane from X axis
    double dEndAngleDeg,         ///< [in ]: End angle in XY plane from X axis
    SmBrep*& rpResult           ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmVector3d sXaxis(1.0,0.0,0.0);
    SmVector3d sYaxis(0.0,1.0,0.0);
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, sXaxis, sYaxis );

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER( sPC.CreateTorus( dRadiusMajor, dRadiusMinor, dStartAngleDeg, dEndAngleDeg, sRefFrame ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreatePartialTorus

/*******************************************************************//**
PURPOSE --- Offset a set of planar curves

NOTES ---   Curves must lie in a plane and form at least one closed loop.
            The offset can be to the left, right, or both sides of the
            input curves.

***********************************************************************/
SmApiStatus SmApiCreateOffsetProfile
(
    const SmTArray<SmCurve*>& rPlanarCurves,  ///< [in ]: Planar curves forming at least one closed loop
    double dOffsetDistance,              ///< [in ]: Offset distance
    ULONG lOffsetType,                  ///< [in ]: 1=LEFT, 2=RIGHT, 3=BOTH
    SmBoolean bRoundCorners,            ///< [in ]: TRUE = round expanding corners
    SmBoolean bShellResult,             ///< [in ]: TRUE = shell result (LEFT or RIGHT only)
    SmBrep*& rpResult                  ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    const SmContext& crContext = *SmApiGetOrCreateContext();

    // OffsetProfile consumes its curve array on success, so pass private
    // copies -- otherwise borrowed curves (e.g. from a Face's edges) get
    // pulled out of their source Brep's topology.
    SmTArray<SmCurve*> sCurveCopies;
    SmObjsDelete<SmCurve*> sCopyCleanup( &sCurveCopies );
    for( ULONG ii = 0; ii < rPlanarCurves.GetSize(); ii++ )
    {
        if( rPlanarCurves[ii] == NULL )
            return SM_ERR_INVALID_INPUT;
        SmCurve* pCopy = NULL;
        SmStatus eCopyStat = rPlanarCurves[ii]->Copy( crContext, pCopy );
        if( eCopyStat != SM_SUCCESS )
        {
            delete pCopy;
            return eCopyStat;
        }
        if( pCopy == NULL )
            return SM_ERR_NULL_POINTER;
        sCurveCopies.Add( pCopy );
    }

    // Keep cleanup ownership separate from the working array passed to the
    // consuming kernel operation, then clear only entries whose ownership
    // transferred to its partial or complete result.
    SmTArray<SmCurve*> sOwnedCurveCopies(sCurveCopies);
    sCopyCleanup.Clear();
    SmObjsDelete<SmCurve*> sOwnedCopyCleanup( &sOwnedCurveCopies );

    SmTArray<SmBoolean> sInputOwnershipTransferred;
    SmStatus eStat = SmPrimitiveCreation::OffsetProfile(
        crContext, sCurveCopies, SM_ZONE_TOL_3D, dOffsetDistance, lOffsetType,
        bRoundCorners, bShellResult, rpResult, sInputOwnershipTransferred );

    const ULONG lOwnershipCount = sInputOwnershipTransferred.GetSize();
    const ULONG lCopyCount = sOwnedCurveCopies.GetSize();
    const ULONG lKnownOwnershipCount = lOwnershipCount < lCopyCount ? lOwnershipCount : lCopyCount;
    for( ULONG ii = 0; ii < lKnownOwnershipCount; ii++ )
        if( sInputOwnershipTransferred[ii] )
            sOwnedCurveCopies[ii] = NULL;

    if( lOwnershipCount != lCopyCount )
    {
        // Ownership is indeterminate for copies without a report entry. Do not
        // risk deleting a curve already transferred to the partial result.
        for( ULONG ii = lKnownOwnershipCount; ii < lCopyCount; ii++ )
            sOwnedCurveCopies[ii] = NULL;
        SER( SM_ERR );
    }

    SER( eStat );

#ifdef SM_ASSERT_VALID
    if( rpResult )
        SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe && rpResult ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif

    return( SM_SUCCESS );

} // SmApiCreateOffsetProfile

/*******************************************************************//**
PURPOSE --- Create a blend surface between two boundary edges

NOTES ---   Uses SmPrimitiveCreation::CreateBlendPrimitive
            TRUE = run blend between start of Edge1->Curve to start of Edge2->Curve
            FALSE = run blend between start of edge1->Curve to end of Edge2->Curve 
***********************************************************************/
SmApiStatus SmApiCreateBlendPrimitive
(
    SmBrep* pBrep,                ///< [in ]: Brep containing the boundary edges
    SmEdge* pEdge1,               ///< [in ]: First boundary edge (start of blend)
    SmFace* pFace1,               ///< [in ]: Target start blend surface 
    SmEdge* pEdge2,               ///< [in ]: End surface boundary curve
    SmFace* pFace2,               ///< [in ]: Target end blend surface 
    SmBoolean bSameDirCurves,     ///< [in ]: TRUE = run blend between start of Edge1->Curve to start of Edge2->Curve
                                  ///< [in ]: FALSE = run blend between start of Edge1->Curve to end of Edge2->Curve 
    SmFace*& rpBlendFace          ///< [out]: Resulting blend face
)
{
    if( !pBrep || !pEdge1 || !pFace1 || !pEdge2 || !pFace2 )
        return SM_ERR_INVALID_INPUT;

    rpBlendFace = NULL;

    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SER( sPC.CreateBlendPrimitive( pFace1, pEdge1, pFace2, pEdge2, bSameDirCurves, NULL, rpBlendFace ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif

    return SM_SUCCESS;

} // SmApiCreateBlendPrimitive

/*******************************************************************//**
PURPOSE --- Create a swung solid from XY and XZ profile curves

NOTES ---   Uses SmPrimitiveCreation::CreateSwungPrimitive
***********************************************************************/
SmApiStatus SmApiCreateSwungPrimitive
(
    SmTArray<SmCurve*>& rXYCurves,      ///< [in ]: Curves defining the XY profile
    SmTArray<SmCurve*>& rXZCurves,      ///< [in ]: Curves defining the XZ profile
    double dScale,                      ///< [in ]: Scale factor
    SmBrep*& rpResult                  ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SER( sPC.CreateSwungPrimitive( rXYCurves, rXZCurves, dScale, SM_ZONE_TOL_3D ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif

    return SM_SUCCESS;

} // SmApiCreateSwungPrimitive

/*******************************************************************//**
PURPOSE --- Create a lofted solid (skin) from a set of profile curves

NOTES ---   Full version with per-profile curve counts, degree control,
            and optional face output maps.
***********************************************************************/
SmApiStatus SmApiCreateSkinPrimitive
(
    const SmTArray<ULONG>& rCurvesInEachProfile,       ///< [in ]: Number of curves in each cross-section profile
    const SmTArray<const SmCurve*>& r3DCurves,         ///< [in ]: All profile curve segments concatenated
    ULONG lDegree,                                     ///< [in ]: Degree in loft direction: 1=linear, 2=quad, 3=cubic
    SmBoolean bCapEnds,                                ///< [in ]: TRUE = build end caps
    SmBrep*& rpResult,                                 ///< [out]: Resulting SmBrep
    SmTArray<SmFace*>* pOptStartFaces,                 ///< [opt]: Start cap faces (NULL to ignore)
    SmTArray<SmFace*>* pOptSideFaces,                  ///< [opt]: Side loft faces (NULL to ignore)
    SmTArray<SmFace*>* pOptEndFaces                    ///< [opt]: End cap faces (NULL to ignore)
)
{
    rpResult = NULL;
    if( pOptStartFaces ) pOptStartFaces->ReSet();
    if( pOptSideFaces  ) pOptSideFaces->ReSet();
    if( pOptEndFaces   ) pOptEndFaces->ReSet();

    SmApiStatus eStat = SmApiValidateSkinPrimitiveInputs( rCurvesInEachProfile, r3DCurves, lDegree );
    SER( eStat );

    const SmContext* pContext = SmApiGetOrCreateContext();
    SmTArray<SmCurve*> sCurveCopies;
    SmObjsDelete<SmCurve*> sPartialCopyCleanup( &sCurveCopies );
    eStat = SmApiCopySkinPrimitiveCurves( *pContext, r3DCurves, sCurveCopies );
    if( eStat != SM_SUCCESS )
        return eStat;

    // The working array may be nulled by the legacy open-profile path. Keep a
    // separate ownership array so unconsumed private copies remain reclaimable.
    SmTArray<SmCurve*> sOwnedCurveCopies(sCurveCopies);
    sPartialCopyCleanup.Clear();
    SmObjsDelete<SmCurve*> sCurveCopyCleanup( &sOwnedCurveCopies );

    SmBrep* pResult = new (*pContext) SmBrep();
    if( pResult == NULL )
        return SM_ERR_OUT_OF_MEMORY;
    SmObjDelete sResultCleanup( pResult );

    SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;

    SmPrimitiveCreation sPC( pResult->GetInfiniteRegion() );

    SmTArray<SmBoolean> sInputOwnershipTransferred;
    eStat = sPC.CreateSkinPrimitive(rCurvesInEachProfile, sCurveCopies, 0.0, lDegree, bCapEnds,
                                    sStartFaces, sSideFaces, sEndFaces, sInputOwnershipTransferred);

    const ULONG lOwnershipCount = sInputOwnershipTransferred.GetSize();
    const ULONG lCopyCount = sOwnedCurveCopies.GetSize();
    const ULONG lKnownOwnershipCount = lOwnershipCount < lCopyCount ? lOwnershipCount : lCopyCount;
    for( ULONG ii = 0; ii < lKnownOwnershipCount; ii++ )
        if( sInputOwnershipTransferred[ii] )
            sOwnedCurveCopies[ii] = NULL;

    if( lOwnershipCount != lCopyCount )
    {
        // Ownership is indeterminate for copies without a report entry. Do not
        // risk deleting a curve already transferred to the partial result.
        for( ULONG ii = lKnownOwnershipCount; ii < lCopyCount; ii++ )
            sOwnedCurveCopies[ii] = NULL;
        SER( SM_ERR );
    }

    SER( eStat );

    if( pOptStartFaces ) *pOptStartFaces = sStartFaces;
    if( pOptSideFaces  ) *pOptSideFaces  = sSideFaces;
    if( pOptEndFaces   ) *pOptEndFaces   = sEndFaces;

    sResultCleanup.Clear();
    rpResult = pResult;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( rpResult, NULL, 1, 1 );
    }
#endif

    return SM_SUCCESS;

} // SmApiCreateSkinPrimitive

/*******************************************************************//**
PURPOSE --- Create a lofted solid between matching loops on face sequences

NOTES ---   Uses SmPrimitiveCreation::CreateSkinFromFaces
***********************************************************************/
SmApiStatus SmApiCreateSkinFromFaces
(
    SmTArray<SmFace*>& rFaces,              ///< [in ]: Sequence of faces whose loops to loft between
    ULONG lDegree,                          ///< [in ]: Degree in loft direction: 1=linear, 2=quad, 3=cubic
    SmTArray<SmFace*>& rNewFaces,           ///< [out]: Resulting lofted faces
    SmBrep*& rpResult                      ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;
    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmTArray<SmBSplineCurve*> sEmptyCurves;

    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SmStatus eStat = sPC.CreateSkinFromFaces( rFaces, sEmptyCurves, 0.0, lDegree, rNewFaces );
    if( eStat != SM_SUCCESS )
    {
        rNewFaces.ReSet();
        return eStat;
    }

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( pBrep, NULL, 1, 1 );
    }
#endif

    return SM_SUCCESS;

} // SmApiCreateSkinFromFaces
