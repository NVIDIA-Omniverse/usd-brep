// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCurveTypes.h
* PURPOSE: Declaration of curve types.
**********************************************************************/

#ifndef __SMCURV_TYPES_H__
#define __SMCURV_TYPES_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

/*******************************************************************//**
PURPOSE: This flag defines the location where a point might be
    on a curve

NOTES: 
***********************************************************************/
enum SmCurveLocationType 
{
  SM_CL_SEAM,     // Location On seam of a closed curve
  SM_CL_INTERIOR, // Location On curve
  SM_CL_EXTERIOR, // Location not On curve
  SM_CL_UNKNOWN
} ;

/*******************************************************************//**
PURPOSE: Define curve parameterization types for interpolation

NOTES: 
***********************************************************************/
enum SmCurveParameterizationType {
    SM_CP_UNIFORM = 0,
    SM_CP_CHORDLENGTH = 1,
    SM_CP_CENTRIPETAL = 2
};


/*******************************************************************//**
PURPOSE: This enum defines what direction offsets are to be produced
   relative to the original curve as seen from the positive side of the
   offset plane.

NOTES: 
***********************************************************************/
enum SmOffsetDirectionType {
    SM_OD_RIGHT_HAND_SIDE,
    SM_OD_LEFT_HAND_SIDE,
    SM_OD_BOTH_SIDES,
    SM_OD_UNKNOWN
};

/*******************************************************************//**
PURPOSE: This enum defines what type of corner to produce when doing
    curve filleting of composites.

NOTES: 
***********************************************************************/
enum SmOffsetCornerType {
    SM_OC_LINEAR_EXTENSION,
    SM_OC_FILLET_CORNER,
    SM_OC_LINEAR_CHAMFER
};

/*******************************************************************//**
PURPOSE: This enum defines what type of control points are being given
   to the canonical B-Spline curve/surface constructors.

NOTES: 
***********************************************************************/
enum SmControlPointFormType {
    SM_CP_NON_RATIONAL,         // (Euclidian) Non-rational control points - no W
    SM_CP_HOMOGENEOUS_RATIONAL, // Control points need be divided by W to produce Euclidian points
    SM_CP_EUCLIDIAN_RATIONAL    // Control points have already been divided by W and are Euclidian
};

/*******************************************************************//**
PURPOSE: This enum defines what type of input knots are being given
   to the canonical B-Spline curve/surface constructors.

NOTES: 
***********************************************************************/
enum SmEndKnotFormType {
    SM_EK_CLAMPPED,             // End knot has degree + 1 multiplicities
    SM_EK_UNCLAMPPED            // End knot has degree multiplicities
};

/*******************************************************************//**
PURPOSE: This enum specifies what type of approximation algorithm
   is being used.

NOTES: 
***********************************************************************/
enum SmApproxAlgorithmType {
    SM_AA_HERMITE
};


/*******************************************************************//**
PURPOSE: This enum helps to define which sort of validity check
   is to be done on a curve/surface.  Note that right now most of these
   are not supported.

NOTES: 
***********************************************************************/
enum SmValidityCheckType {
    SM_VC_NONE,
    SM_VC_ALL,
    SM_VC_DEFINITION,

    SM_VC_SELF_INTERSECTION,    // Not supported
    SM_VC_CUSPS,                // Not supported
    SM_VC_SMOOTH,               // Not supported
    SM_VC_REVERSE_DIRECTION,
    SM_VC_SINGULARITY
} ;

/*******************************************************************//**
PURPOSE: This enum defines what kind of knots are used.  Right now
   we are not using this information anywhere within the software.
   It exists primarily for STEP compatability reasons.

NOTES: 
***********************************************************************/
enum SmKnotType {
    SM_KT_UNIFORM_KNOTS,
    SM_KT_UNSPECIFIED,
    SM_KT_QUASI_UNIFORM_KNOTS,
    SM_KT_PIECEWISE_BEZIER_KNOTS,
    SM_KT_UNKOWN
} ;

/*******************************************************************//**
PURPOSE: This enum is used to specify which curve property is to
    be extracted.

NOTES: 
  To add a new solver type: see the description in the header
  comments for class SmCurvePropertyEFO, in file SmCurve.cpp.
***********************************************************************/
enum SmCurvePropertyType
{
  SM_CP_PERPENDICULAR_TO_VECTOR,  // Find curve points where the tangent is perpendicular to the
                                  // given vector vector.  One vector is given.
  
  SM_CP_INFLECTION_POINTS,        // Find points where second derivative vanishes
  
  SM_CP_RADIUS_OF_CURVATURE,      // Find points where radius of curvature equal to given value - 
                                  //      requires value input
  
  SM_CP_FIRST_DERIVATIVE_LENGTH,  // Find points where first derivative is of given length - 
                                  //      requires value input
  
  SM_CP_PARALLEL_TO_VECTOR,       // Find points where tangent parallel to vector -
                                  //      requires vector input
  
  SM_CP_SILHOUETTE_VECTOR,        // Find parallel projection silhouette points relative to a view vector - 
                                  //      requires vector input
  
  SM_CP_SILHOUETTE_POINT,         // Find perspective projection silhouette points relative to eye point -
                                  //      requires vector input
  
  SM_CP_X_NORMAL,                 // Find points where X value of tangent goes through zero
                                  //      ignores points on YZ planar curve segments so an YZ planar curve will have
                                  //      zero points normal to the X axis rather than the entire curve being normal. 
  
  SM_CP_Y_NORMAL,                 // Find points where Y value of tangent goes through zero
                                  //      ignores points on XZ planar curve segments so an XZ planar curve will have
                                  //      zero points normal to the Y axis rather than the entire curve being normal. 
  
  SM_CP_Z_NORMAL,                 // Find points where Z value of tangent goes through zero
                                  //      ignores points on XY planar curve segments so an XY planar curve will have
                                  //      zero points normal to the Z axis rather than the entire curve being normal. 
  
  SM_CP_XYZ_NORMAL,               // Find points where X, Y or Z value of tangent becomes zero
  
  SM_CP_MINIMIZE_FIRST_DERIVATIVE, // Find minimum first derivative points
  
  SM_CP_MAXIMIZE_FIRST_DERIVATIVE, // Find maximum first derivative points
  
  SM_CP_MINIMIZE_RADIUS_OF_CURVATURE, // Find minimum radius of curvature points
  
  SM_CP_MAXIMIZE_RADIUS_OF_CURVATURE, // Find maximum radius of curvature points
  
  SM_CP_PLANE_CLASSIFY,              // Find segments of a curve on the positive side of plane
                                     // equation.  The plane is defined by a normal vector (unitized) and
                                     // and the plane equation D value input as a value.  The plane equation
                                     // is Ax + By + Cz + D = 0.  Where A,B,C is a unitized plane normal and
                                     // D the negative value of any point on the plane doted with the normal.
  
  SM_CP_PLANE_INTERSECTION,          // Find points of intersection with plane equation
                                     // The plane is defined by a normal vector (unitized) and
                                     // and the plane equation D value input as a value
  
  SM_CP_MINIMIZE_ANGLE_TO_PLANE,     // Minimize the angle made by a line through a point on
                                     // the plane to a point on a curve.
                                     // Three vectors are required for input.  The first vector defines
                                     // the pivot point of the line. The second vector is a 
                                     // a normal vector defining the plane of projection.
                                     // The third vector defines the normal to the plane to which the angle
                                     // is to be measured.  
                                     // Both normal vectors must be unitized.  The third
                                     // vector must be perpendicular to the projection plane normal.
                                     // This method assumes that the object does not cross the plane.
                                     // If it does you should utilize SM_CP_PLANE_INTERSECTION to compute
                                     // the crossings which would produce a zero angle.
  
  SM_CP_MINIMIZE_DIRECTED_ANGLE,     // Minimize the projected angle made between a vector from a
                                     // point on the projection plane to the curve relative to a 
                                     // reference vector in the projection plane.  
                                     // The angle produced will be positive if the vector is clockwise 
                                     // from the projection vector and between zero and 180 degrees.
                                     // The angle produced will be negative if the vector is in the 
                                     // 0 to -180 range.
                                     // Note that this is a TRUE minimization and that negative values
                                     // will override positive values and that negative values with greater
                                     // magnitude will override negative values of smaller magnitude.
                                     // Three vectors are required for input.  The first vector defines
                                     // the pivot point of the line. The second vector defines the
                                     // the projection plane in which the angle is to be measured.
                                     // The third vector is a vector in the projection plane tp which the
                                     // angle is measured. This vector defines the zero angle.  
                                     // The two vectors must be unitized and perpendicular to each other.
                                     // This method assumes that the object does not cross the plane.
                                     // If it does you should utilize SM_CP_PLANE_INTERSECTION to compute
                                     // the crossings which would produce negative 180 degree result.
  
  SM_CP_MINIMIZE_CCW_ANGLE,          // Minimize the counter clock wise angle made between a vector from a
                                     // point on the projection plane to the curve relative to a 
                                     // reference vector in the projection plane.  
                                     // The angle produced will always be positive and between 0 and 360
                                     // degrees.
                                     // Three vectors are required for input.  The first vector defines
                                     // the pivot point of the line. The second vector defines the
                                     // the projection plane in which the angle is to be measured.
                                     // The third vector is a vector in the projection plane tp which the
                                     // angle is measured.
                                     // The two vectors must be unitized and perpendicular to each other.
                                     // This method assumes that the object does not cross the line defined
                                     // by the point and vector.
  
  SM_CP_PROJECTED_POINT_MINIMIZE,    // Minimize the distance between a point and a curve as viewed
                                     // from a parallel projection.  The first vector contains the 
                                     // point to minimize and the second vector is used to define the 
                                     // normal to a projection plane.
  
  SM_CP_PROJECTED_POINT_MAXIMIZE,    // Maximize the distance between a point and a curve as viewed 
                                     // from a parallel projection.  The first vector contains the 
                                     // point to minimize and the second vector is used to define the 
                                     // normal to a projection plane.
  
  SM_CP_CYLINDER_INTERSECTION,       // Find points of intersection with an infinite cylinder
                                     // The cylinder is defined by a point on the axis(the first vector),
                                     // a center axis vector (the second vector), and the radius (the
                                     // property value)
  
  SM_CP_DISTANCE_TO_POINT,           // Find points where the curve is a given distance from a given
                                     // point.  Essentially, intersect the curve with an ideal sphere,
                                     // returning only the curve parameter.
  
  SM_CP_TANGENT_TO_CIRCLE,           // Given a point on a circle (of unknown radius) and a tangent to
                                     // the circle, find a point on this curve that is also tangent to
                                     // the circle.  (Picture: keeping the circle point and tangent fixed,
                                     // grow the circle until it touches the curve.)
                                     // Requires the point on the circle, and the tangent vector there.
  
  SM_CP_POINT_INVERSION,             // Given a point on the curve, find the corresponding parameter
  
  SM_CP_MAXIMA_TO_LINE,              // find points where distance to given line is either a maximum or a minimum
                                     // i.e. where the curve becomes tangent to any of a family of cylinders
                                     //      centered on the given line.
  
  SM_CP_MAXIMA_TO_PLANE,             // find points where distance to a given plane is either a maximum or a minimum
                                     // i.e. where the curve becomes tangent to any of a family of planes
                                     //      all parallel to the given plane. 
                                     //      (same as SM_CP_PERPENDICULAR_TO_VECTOR when vec = PlaneNormal)
  SM_CP_PROJECTED_TANGENT_THROUGH_POINT // find points where curve tangent contains a given point
} ; // end enum SmCurvePropertyType

/*******************************************************************//**
PURPOSE: This enum defines what type of interpolation algorithm is to
    be used.  Currently this type is not used.

NOTES: 
***********************************************************************/
enum SmInterpolationType
{
 SM_IT_UNIFORM,
 SM_IT_CHORDLENGTH,
 SM_IT_CENTRIPETAL
} ;

/*******************************************************************//**
PURPOSE: This enum defines behavior choices for an input tgt values that
         can be unconstrained, the same, or specified.

NOTES: 1. Tri-state input values to methods and function can be passed
          as a (enum, value) pair where the enum value specifies how
          the algorithm will use the input value as:
            SPECIFIED     = use input value as specified in the documentation.
            SAME          = 1st: set input value = current values. 
                            2nd: use input value as specified in the documentation.
            UNCONSTRAINED = ignore input value.
       2. Bi-state input values don't need an enum partner.  They can be passed
          as a single pointer value as:
            NotNULL       = use input value as specified in the documentation
            NULL          = ignore input value.
***********************************************************************/
enum SmInValueType
{
 SM_IV_SPECIFIED,      // use input value as specified in the documentation.
 SM_IV_SAME,           // 1st: set input value according to documented rule.
                       // 2nd: use input value as specified in the documentation.
 SM_IV_UNCONSTRAINED,  // ignore input value.
} ;

/*******************************************************************//**
PURPOSE: This enum is used to define the underlying type for a B-Spline
   curve. 

NOTES: 
***********************************************************************/
enum SmBSplineCurveForm
{
  SM_CF_POLYLINE_FORM,
  SM_CF_CIRCULAR_ARC,
  SM_CF_ELLIPTIC_ARC,
  SM_CF_PARABOLIC_ARC,
  SM_CF_HYPERBOLIC_ARC,
  SM_CF_HELICAL_ARC,
  SM_CF_UNSPECIFIED
} ;

/*******************************************************************//**
PURPOSE: This enum defines which type of circular parameterization to
   use when creating circles, ellipse, cylinders, and other objects with
   a rational quadric cross section.

NOTES: 
***********************************************************************/
enum SmNurbCircleParam {
    SM_CO_QUADRATIC,    // degree 2 
    SM_CO_QUINTIC       // degree 5
};


class SmCurve;
class SmLine;
class SmConic;
class SmCircle;
class SmEllipse;
class SmParabola;
class SmHyperbola;
class SmCompositeCurve;
class SmBSplineCurve;
class SmHermiteCurve;
class SmOffsetCurve;
class SmProjectedCurve;
class SmOffsetMapAttribute;
class SmCompositeCurveRegion;
/*class SmCCRegionCache;*/

class SmCurveCache;

#define SmCurve_TYPE                 (CURV_BASE_TYPE + 1)   /* CURV_BASE_TYPE:[14000] */
#define SmLine_TYPE                  (CURV_BASE_TYPE + 10)
#define SmConic_TYPE                 (CURV_BASE_TYPE + 20)
#define SmCircle_TYPE                (CURV_BASE_TYPE + 21)
#define SmEllipse_TYPE               (CURV_BASE_TYPE + 22)
#define SmParabola_TYPE              (CURV_BASE_TYPE + 23)
#define SmHyperbola_TYPE             (CURV_BASE_TYPE + 24)
#define SmCompositeCurve_TYPE        (CURV_BASE_TYPE + 32)
#define SmBSplineCurve_TYPE          (CURV_BASE_TYPE + 33)
#define SmHermiteCurve_TYPE          (CURV_BASE_TYPE + 34)
#define SmOffsetCurve_TYPE           (CURV_BASE_TYPE + 36)
#define SmProjectedCurve_TYPE        (CURV_BASE_TYPE + 37)
#define SmCompositeCurveRegion_TYPE  (CURV_BASE_TYPE + 38)
#define SmCompositeCurveSegment_TYPE (CURV_BASE_TYPE + 40)
#define SmIsoCurve_TYPE              (CURV_BASE_TYPE + 45)  /* gwc: moved frm SmSurfTypes.h  (SURF_BASE_TYPE + 200) */
#define SmCrvOnSurf_TYPE             (CURV_BASE_TYPE + 48)  /* gwc: moved from SmSurfTypes.h (SURF_BASE_TYPE + 210) */
#define SmTangentField_TYPE          (CURV_BASE_TYPE + 50)  /* gwc: moved from SmSurfTypes.h (SURF_BASE_TYPE + 211) */
#define SmCrvInVolume_TYPE           (CURV_BASE_TYPE + 53)

/*#define SmCCRegionCache_TYPE        (CURV_BASE_TYPE + 39)*/

#define SmCurveCache_TYPE           (CURV_BASE_TYPE + 100)  /* CURV_BASE_TYPE:[14000] */
#define SmOffsetMapAttribute_TYPE   (CURV_BASE_TYPE + 200)

#define SM_CURV_TYPENAME(a) \
  (a) == SmCurve_TYPE                 ? _T("SmCurve                ") \
: (a) == SmLine_TYPE                  ? _T("SmLine                 ") \
: (a) == SmConic_TYPE                 ? _T("SmConic                ") \
: (a) == SmCircle_TYPE                ? _T("SmCircle               ") \
: (a) == SmEllipse_TYPE               ? _T("SmEllipse              ") \
: (a) == SmParabola_TYPE              ? _T("SmParabola             ") \
: (a) == SmHyperbola_TYPE             ? _T("SmHyperbola            ") \
: (a) == SmCompositeCurve_TYPE        ? _T("SmCompositeCurve       ") \
: (a) == SmBSplineCurve_TYPE          ? _T("SmBSplineCurve         ") \
: (a) == SmHermiteCurve_TYPE          ? _T("SmHermiteCurve         ") \
: (a) == SmOffsetCurve_TYPE           ? _T("SmOffsetCurve          ") \
: (a) == SmProjectedCurve_TYPE        ? _T("SmProjectedCurve       ") \
: (a) == SmCompositeCurveRegion_TYPE  ? _T("SmCompositeCurveRegion ") \
: (a) == SmCompositeCurveSegment_TYPE ? _T("SmCompositeCurveSegment") \
: (a) == SmIsoCurve_TYPE              ? _T("SmIsoCurve             ") \
: (a) == SmCrvOnSurf_TYPE             ? _T("SmCrvOnSurf            ") \
: (a) == SmTangentField_TYPE          ? _T("SmTangentField         ") \
: (a) == SmCrvInVolume_TYPE           ? _T("SmCrvInVolume          ") \
: (a) == SmCurveCache_TYPE            ? _T("SmCurveCache           ") \
: (a) == SmOffsetMapAttribute_TYPE    ? _T("SmOffsetMapAttribute   ") \
: _T("Not a SURFACE_BASE_TYPE" )                                   

#endif // !__SMCURV_TYPES_H__


