// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiPolygons.cpp

PURPOSE: 
    Contains high level "C" type functions that operate on SmPolyBreps.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmMerge, etc
**********************************************************************/

#include "StdAfx.h"

#include "SmApiGeneral.h"
#include "SmApiPolygons.h"
#include "SmPoly.h"
#include "SmPolyMerge.h"

#if SM_DEBUG_CODE
static const SmVector3d s_kBlue (0, 0, 1);
static const SmVector3d s_kGreen(0, 1, 0);
static const SmVector3d s_kRed  (1, 0, 0);
#endif

/*******************************************************************//**
PURPOSE --- Boolean two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiPolyBoolean
(
    SmPolyBrep*  pPolyBrep1,              ///< [in ]: Pointer to primary brep
    SmPolyBrep*  pPolyBrep2,              ///< [in ]: Pointer to second brep
    SmPolyBooleanOperationType operation, ///< [in ]: specify boolean operation
    SmPolyBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
  rpResult = NULL;
  if( pPolyBrep1 == NULL || pPolyBrep2 == NULL || pPolyBrep1 == pPolyBrep2 )
    return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pPolyBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pPolyBrep2, &s_kGreen, 0, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());
  double dAngleTol = 20.0*SM_PI/180.0;

  SmPolyMerge sPolyMerge( *pContext, pPolyBrep1, pPolyBrep2, dTol, dAngleTol );
  SER( sPolyMerge.ManifoldBoolean( operation, rpResult ) );
 
#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif


  return(SM_SUCCESS);

} // End SmApiPolyBoolean

/*******************************************************************//**
PURPOSE --- Boolean Union two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiPolyBooleanUnion
(
    SmPolyBrep*  pPolyBrep1,              ///< [in ]: Pointer to primary brep
    SmPolyBrep*  pPolyBrep2,              ///< [in ]: Pointer to second brep
    SmPolyBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
  rpResult = NULL;
  if( pPolyBrep1 == NULL || pPolyBrep2 == NULL || pPolyBrep1 == pPolyBrep2 )
    return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pPolyBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pPolyBrep2, &s_kGreen, 0, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());
  double dAngleTol = 20.0*SM_PI/180.0;

  SmPolyMerge sPolyMerge( *pContext, pPolyBrep1, pPolyBrep2, dTol, dAngleTol );
  SER( sPolyMerge.ManifoldBoolean( SM_PBO_UNION, rpResult ) );
  
#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif


  return(SM_SUCCESS);

} // End SmApiPolyBooleanUnion

/*******************************************************************//**
PURPOSE --- Boolean Union two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiPolyBooleanDifference
(
    SmPolyBrep*  pPolyBrep1,          ///< [in ]: Pointer to primary brep
    SmPolyBrep*  pPolyBrep2,          ///< [in ]: Pointer to second brep
    SmPolyBrep*& rpResult            ///< [out]: Resulting SmBrep
)
{
  rpResult = NULL;
  if( pPolyBrep1 == NULL || pPolyBrep2 == NULL || pPolyBrep1 == pPolyBrep2 )
    return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pPolyBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pPolyBrep2, &s_kGreen, 0, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());
  double dAngleTol = 20.0*SM_PI/180.0;

  SmPolyMerge sPolyMerge( *pContext, pPolyBrep1, pPolyBrep2, dTol, dAngleTol );
  SER( sPolyMerge.ManifoldBoolean( SM_PBO_DIFFERENCE, rpResult ) );


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif


  return(SM_SUCCESS);

} // End SmApiPolyBooleanDifference

/*******************************************************************//**
PURPOSE --- Boolean Intersection two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiPolyBooleanIntersection
(
    SmPolyBrep*  pPolyBrep1,              ///< [in ]: Pointer to primary brep
    SmPolyBrep*  pPolyBrep2,              ///< [in ]: Pointer to second brep
    SmPolyBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
  rpResult = NULL;
  if( pPolyBrep1 == NULL || pPolyBrep2 == NULL || pPolyBrep1 == pPolyBrep2 )
    return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pPolyBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pPolyBrep2, &s_kGreen, 0, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());
  double dAngleTol = 20.0*SM_PI/180.0;

  SmPolyMerge sPolyMerge( *pContext, pPolyBrep1, pPolyBrep2, dTol, dAngleTol );
  SER( sPolyMerge.ManifoldBoolean( SM_PBO_INTERSECTION, rpResult ) );


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif


  return(SM_SUCCESS);

} // End SmPolyBooleanIntersection


/*******************************************************************//**
PURPOSE --- Boolean Merge two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiPolyBooleanMerge
(
    SmPolyBrep*  pPolyBrep1,                  ///< [in ]: Pointer to primary brep
    SmPolyBrep*  pPolyBrep2,                  ///< [in ]: Pointer to second brep
    SmPolyBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
  rpResult = NULL;
  if( pPolyBrep1 == NULL || pPolyBrep2 == NULL || pPolyBrep1 == pPolyBrep2 )
    return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pPolyBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pPolyBrep2, &s_kGreen, 0, 1 );
  }
#endif

  const SmContext* pContext = SmApiGetOrCreateContext();
  double dTol = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());
  double dAngleTol = 20.0*SM_PI/180.0;

  SmPolyMerge sPolyMerge( *pContext, pPolyBrep1, pPolyBrep2, dTol, dAngleTol );
  SER( sPolyMerge.ManifoldBoolean( SM_PBO_MERGE, rpResult ) );

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif


  return(SM_SUCCESS);

} // End SmApiPolyBooleanMerge

/*******************************************************************//**
PURPOSE --- Query whether a PolyBrep is a manifold solid.

USAGE NOTES --- A manifold solid PolyBrep has all manifold edges
    (every edge shared by exactly two faces). No lamina, wire, or
    spine edges.
***********************************************************************/
SmApiStatus SmApiPolyBrepIsManifoldSolid
(
    const SmPolyBrep* pPolyBrep,          ///< [in ]: Pointer to poly brep
    SmBoolean & rbIsManifoldSolid         ///< [out]: TRUE = manifold solid, FALSE = not
)
{
  if( pPolyBrep == NULL )
    { return SM_ERR_INVALID_INPUT; }

  rbIsManifoldSolid = pPolyBrep->IsManifoldSolid();

  return SM_SUCCESS;

} // End SmApiPolyBrepIsManifoldSolid

/*******************************************************************//**
PURPOSE --- Compute the enclosed volume of a PolyBrep by summing
    per-face tetrahedra contributions.

USAGE NOTES --- The PolyBrep should be a closed manifold mesh for the
    volume to be meaningful.
***********************************************************************/
SmApiStatus SmApiPolyBrepComputeVolume
(
    const SmPolyBrep*  pPolyBrep,         ///< [in ]: Pointer to poly brep
    double     & rdVolume                 ///< [out]: Enclosed volume
)
{
  if( pPolyBrep == NULL )
    { return SM_ERR_INVALID_INPUT; }

  double dArea = 0.0;
  SmPoint3d  sBarycenter;
  SmVector3d aMoments[2];

  SmExtent3d sBBox;
  SER( pPolyBrep->CalculateBoundingBox(sBBox) );
  SmPoint3d sOrigin = sBBox.GetMid();

  SER( pPolyBrep->ComputeProperties( sOrigin,
                                     dArea,
                                     rdVolume,
                                     sBarycenter,
                                     aMoments,
                                     SM_MPT_VOLUME ) );

  return SM_SUCCESS;

} // End SmApiPolyBrepComputeVolume

/*******************************************************************//**
PURPOSE --- Compute area, volume, mass, centroid, and the mass moments /
    products of inertia of a PolyBrep about a caller-supplied origin.

USAGE NOTES --- Mesh counterpart of SmApiBrepComputeMassProperties, with the
    same outputs and conventions.  One SmPolyBrep::ComputeProperties pass
    about crOrigin; the result is exact for the mesh, so there is no accuracy
    parameter.  Validation matches the Brep entry point (density, origin,
    closed manifold solid, non-degenerate volume).  Unlike
    SmApiPolyBrepComputeVolume, which returns a signed volume, the result does
    not depend on the mesh's winding direction.

    SmPolyBrep::ComputeProperties labels aMoments[0] "moments of inertia", but
    it accumulates the per-axis second moments (integral of x^2, y^2, z^2 dV);
    aMoments[1] holds the products (integral of xy, yz, zx dV).  Both are
    geometric (no density) and taken about crOrigin.
***********************************************************************/
SmApiStatus SmApiPolyBrepComputeMassProperties
(
    const SmPolyBrep* pPolyBrep,
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
    if( pPolyBrep == NULL )
        { return SM_ERR_INVALID_INPUT; }

    if( !SM_IS_VALID_DOUBLE( dDensity ) || dDensity < SM_EFF_ZERO )
        { return SM_ERR_INVALID_INPUT; }

    if(    !SM_IS_VALID_DOUBLE( crOrigin.x )
        || !SM_IS_VALID_DOUBLE( crOrigin.y )
        || !SM_IS_VALID_DOUBLE( crOrigin.z ) )
        { return SM_ERR_INVALID_INPUT; }

    // Only defined for a closed manifold mesh; an open one gives a volume that
    // depends on crOrigin.
    if( !pPolyBrep->IsManifoldSolid() )
        { return SM_ERR_INVALID_INPUT; }

    double     dArea = 0.0, dVolume = 0.0;
    SmPoint3d  sBarycenter;
    SmVector3d aMoments[2];
    SER( pPolyBrep->ComputeProperties( crOrigin, dArea, dVolume, sBarycenter,
                                       aMoments, SM_MPT_ALL ) );

    if( !SM_IS_VALID_DOUBLE( dVolume ) )
        { return SM_ERR_INVALID_INPUT; }

    // A closed mesh with globally reversed winding (as imported, or after a
    // mirroring transform) passes IsManifoldSolid, but every signed integral
    // comes back negated: volume, second moments and products.  Area and the
    // barycenter (a ratio) are unaffected.  Mixed winding is not manifold and
    // was rejected above, so a negative volume here is a clean global flip.
    if( dVolume < 0.0 )
    {
        dVolume     = -dVolume;
        aMoments[0] = aMoments[0] * -1.0;
        aMoments[1] = aMoments[1] * -1.0;
    }

    if( dVolume < SM_EFF_ZERO )
        { return SM_ERR_INVALID_INPUT; }

    // Second moments about the planes through crOrigin, density-weighted.
    const SmVector3d sSecond = aMoments[0] * dDensity;

    rdArea             = dArea;
    rdVolume           = dVolume;
    rdMass             = dDensity * dVolume;
    // The kernel already returns the barycenter as a world position.
    rCentroid          = sBarycenter;
    // Ixx = integral of (y^2 + z^2) dm, cyclically.
    rMomentsOfInertia.Set( sSecond.y + sSecond.z,
                           sSecond.x + sSecond.z,
                           sSecond.x + sSecond.y );
    // Reorder the kernel's (xy, yz, zx) to the Brep API's (yz, zx, xy).
    rProductsOfInertia.Set( aMoments[1].y * dDensity,
                            aMoments[1].z * dDensity,
                            aMoments[1].x * dDensity );

    return SM_SUCCESS;

} // End SmApiPolyBrepComputeMassProperties

/*******************************************************************//**
PURPOSE --- Deep-copy a PolyBrep into a fresh handle owned by the SM_API
    context.

USAGE NOTES --- Wraps SmPolyBrep's copy constructor.  The result is a
    standalone mesh with independent topology -- mutating it does not
    affect the source.  Allocated against SmApiGetOrCreateContext() so its
    lifetime tracks the context, matching SmApiBrepCopy and every other
    SM_API creator.
***********************************************************************/
SmApiStatus SmApiPolyBrepCopy
(
    const SmPolyBrep*  pPolyBrep,               ///< [in ]: Source PolyBrep (not modified)
    SmPolyBrep*& rpResult                 ///< [out]: Newly allocated deep copy
)
{
  if( pPolyBrep == NULL )
    { return SM_ERR_INVALID_INPUT; }

  rpResult = new (*SmApiGetOrCreateContext()) SmPolyBrep( *pPolyBrep );

  if( rpResult == NULL )
    { return SM_ERR; }

  return SM_SUCCESS;

} // End SmApiPolyBrepCopy

