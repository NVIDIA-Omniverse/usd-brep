// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfTypes.h
* PURPOSE: Declaration of surface and volume types.
**********************************************************************/

#ifndef __SMSURF_TYPES_H__
#define __SMSURF_TYPES_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

/*******************************************************************//**
PURPOSE: This flag defines the location where a point might be
    on a surface

NOTES: 
***********************************************************************/
enum SmLocationType 
{
  SM_LT_POLE,     // The point is at POLE, i.e. generator curve touch axis
  SM_LT_U_SEAM,   // On SEAM of a full-revolution surface
  SM_LT_V_SEAM,   // On SEAM of closed-generator-curve surface
  SM_LT_UV_SEAM,  // SM_LT_U_SEAM plus SM_LT_V_SEAM (eg.full-torus)
  SM_LT_INTERIOR, // Neither at the pole nor on the seam
  SM_LT_EXTERIOR  // Point not on surface or out of domain limits
} ;

/*******************************************************************//**
PURPOSE: This enum defines the type of point defined by the local surface
    geometry.

NOTES: 
***********************************************************************/
enum SmLocalSurfaceType {
    SM_LS_PLANAR,       // U and V isoParam Curves are planar at a point
    SM_LS_ELLIPTICAL,   // U and V isoParam Curves bend in same direction at a point (a bowl or hill)
    SM_LS_HYPERBOLIC,   // U and V isoparam Curves bend in different directions at a point (a saddle)
    SM_LS_CYLINDRICAL   // only one of U or V isopara Curve is planar at a point
};

/*******************************************************************//**
PURPOSE: This enum defines the side of the surface relative to the
    tangent plane with the natural surface normal.

NOTES: 
***********************************************************************/
enum SmSurfaceSideType {
    SM_SS_ON,        // isoParam Curve is in plane perp to normal at point
    SM_SS_ABOVE,     // isoParam Curve bends up towards surface normal at point
    SM_SS_BELOW      // isoParam Curve bends down away from surface normal at point
};



/*******************************************************************//**
PURPOSE: This enum defines the type of surface which is represented
   by the B-Spline surface.  Note that although we currently do not use
   these values for anything we plan to in the future.  It would be a 
   good idea not to loose this information.

NOTES: SM_SF_UNSPECIFIED is the default value you should use 
   for most NURBS that are not a specific type.
***********************************************************************/
enum SmBSplineSurfaceForm {
    SM_SF_PLANE_SURF,
    SM_SF_CYLINDRICAL_SURF,
    SM_SF_CONICAL_SURF,
    SM_SF_SPHERICAL_SURF,
    SM_SF_SURF_OF_REVOLUTION,
    SM_SF_RULED_SURF,
    SM_SF_GENERALIZED_CONE,
    SM_SF_QUADRIC_SURF,
    SM_SF_SURF_OF_LINEAR_EXTRUSION,
    SM_SF_UNSPECIFIED,
    SM_SF_POLYNOMIAL,
    SM_SF_HELICAL_SWEEP
};

class SmSurface;
class SmBSplineSurface;
class SmPlane;
class SmSphere;
class SmCylinder;
class SmTorus;
class SmCone;
class SmSurfOfRevolution;
class SmSurfOfExtrusion;

class SmSurfaceCache;

class SmIsoCurve;
class SmCrvOnSurf;
class SmTangentField;

class SmOffsetSurface;
class SmSTEPSurface;

#define SmSurface_TYPE              (SURF_BASE_TYPE + 1)   /* SURF_BASE_TYPE:[15000] */
#define SmBSplineSurface_TYPE       (SURF_BASE_TYPE + 2)
#define SmPlane_TYPE                (SURF_BASE_TYPE + 11)
#define SmCone_TYPE                 (SURF_BASE_TYPE + 12)
#define SmCylinder_TYPE             (SURF_BASE_TYPE + 13)
#define SmSphere_TYPE               (SURF_BASE_TYPE + 14)
#define SmTorus_TYPE                (SURF_BASE_TYPE + 15)
#define SmSurfOfRevolution_TYPE     (SURF_BASE_TYPE + 20)
#define SmSurfOfExtrusion_TYPE      (SURF_BASE_TYPE + 30)
#define SmBlendSurface_TYPE         (SURF_BASE_TYPE + 40)
#define SmCurveBoundedSurface_TYPE  (SURF_BASE_TYPE + 45)   /* gwc: moved from (TOPO_BASE_TYPE + 100) in SmTopoTypes.h */

#define SmSurfaceCache_TYPE         (SURF_BASE_TYPE + 100)
#define SmTrimSrfCache_TYPE         (SURF_BASE_TYPE + 120)  /* gwc: moved from SmSurf_types.h and changed value to make unique */
#define SmTessSrfCache_TYPE         (SURF_BASE_TYPE + 150)  /* gwc: moved from SmTess.h and changed value to make unique */
#define SmSurfaceTessDriver_TYPE    (SURF_BASE_TYPE + 152)
#define SmViewBasedTessDriver_TYPE  (SURF_BASE_TYPE + 154)

#define SmSrfInVolume_TYPE          (SURF_BASE_TYPE + 218)  /* SURF_BASE_TYPE:[15000] */
#define SmOffsetSurface_TYPE        (SURF_BASE_TYPE + 400)
#define SmSTEPSurface_TYPE          (SURF_BASE_TYPE + 410)
                                    
#define SM_SURF_TYPENAME(a) \
  (a) == SmSurface_TYPE              ? _T("SmSurface            ") \
: (a) == SmBSplineSurface_TYPE       ? _T("SmBSplineSurface     ") \
: (a) == SmPlane_TYPE                ? _T("SmPlane              ") \
: (a) == SmCone_TYPE                 ? _T("SmCone               ") \
: (a) == SmCylinder_TYPE             ? _T("SmCylinder           ") \
: (a) == SmSphere_TYPE               ? _T("SmSphere             ") \
: (a) == SmTorus_TYPE                ? _T("SmTorus              ") \
: (a) == SmSurfOfRevolution_TYPE     ? _T("SmSurfOfRevolution   ") \
: (a) == SmSurfOfExtrusion_TYPE      ? _T("SmSurfOfExtrusion    ") \
: (a) == SmBlendSurface_TYPE         ? _T("SmBlendSurface       ") \
: (a) == SmCurveBoundedSurface_TYPE  ? _T("SmCurveBoundedSurface") \
: (a) == SmSurfaceCache_TYPE         ? _T("SmSurfaceCache       ") \
: (a) == SmTrimSrfCache_TYPE         ? _T("SmTrimSrfCache       ") \
: (a) == SmTessSrfCache_TYPE         ? _T("SmTessSrfCache       ") \
: (a) == SmSurfaceTessDriver_TYPE    ? _T("SmSurfaceTessDriver  ") \
: (a) == SmViewBasedTessDriver_TYPE  ? _T("SmViewBasedTessDriver") \
: (a) == SmSrfInVolume_TYPE          ? _T("SmSrfInVolume        ") \
: (a) == SmOffsetSurface_TYPE        ? _T("SmOffsetSurface      ") \
: (a) == SmSTEPSurface_TYPE          ? _T("SmSTEPSurface        ") \
: _T("Not a SURFACE_BASE_TYPE" )                                   


#endif // __SMSURF_TYPES_H__

