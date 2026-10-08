// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCoreTypes.h
* PURPOSE: Declaration of core types.
**********************************************************************/

#ifndef __SMCORE_TYPES_H__
#define __SMCORE_TYPES_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#include <float.h>    // for DBL_MAX


/*******************************************************************//**
PURPOSE: The following classes are basically structures with no
    virtual methods and no inheritance.

NOTES:
***********************************************************************/
class SmVector2d;
class SmVector3d;
class SmAxis2Placement;
class SmExtent1d;
class SmExtent2d;
class SmExtent3d;
class SmPseudoBox;
class SmPolarBox;
class SmPeriodicExtent1d;
class SmPointSequence;
class SmPointSet3d;
class SmPointGrid;

#ifndef NL_NOZ
  #define NL_NOZ  DBL_MAX  // same as in NLib
#endif
#ifndef NL_NOW
  #define NL_NOW -DBL_MAX  // same as in NLib
#endif

#ifndef NL_BIGD
  #define NL_BIGD DBL_MAX  // same as in NLib
#endif

/*******************************************************************//**
PURPOSE: SmPoint2d is another name for SmVector2d.  The following
    define is used primarily to make documentation clearer.  When you see
    a point it should be treated as a point not a vector.

NOTES:
***********************************************************************/
#define SmPoint2d SmVector2d

/*******************************************************************//**
PURPOSE: SmPoint3d is another name for SmVector3d.  The following
    define is used primarily to make documentation clearer.  When you see
    a point it should be treated as a point not a vector.

NOTES:
***********************************************************************/
#define SmPoint3d SmVector3d

template<class KeyType, class ValueType> class SmMapPtrToPtr;
template<class KeyType, class ValueType> class SmMapPtrToPtrs;
class SmSolution;
class SmSolutionArray;
template<class TYPE> class SmTArray;
template<class TYPE> class SmTList;
template<class TYPE1,class TYPE2> class SmRelation;
class SmAObject;
class SmAttribute;
class SmLongAttribute;
class SmPointerAttribute;
class SmVector3dAttribute;
class SmTagAttribute;
class SmTree;
class SmTreeNode;
class SmTreeVertex;
template<class TYPE> class SmBucketSort;

#define SmMapPtrToPtr_TYPE           (CONT_BASE_TYPE + 3)
#define SmMapPtrToPtrs_TYPE          (CONT_BASE_TYPE + 400)
#define SmMapTypeToType_TYPE         (CONT_BASE_TYPE + 7)
#define SmSolutionArray_TYPE         (CONT_BASE_TYPE + 20)
#define SmTArray_TYPE                (CONT_BASE_TYPE + 40)
#define SmSArray_TYPE                (CONT_BASE_TYPE + 45)
#define SmTList_TYPE                 (CONT_BASE_TYPE + 50)
#define SmRelation_TYPE              (CONT_BASE_TYPE + 55)

#define SmAObject_TYPE               (CONT_BASE_TYPE + 60) // note: 62 and 64 are used below

#define SmAttribute_TYPE             (CONT_BASE_TYPE + 70)
#define SmLongAttribute_TYPE         (CONT_BASE_TYPE + 71)
#define SmVector3dAttribute_TYPE     (CONT_BASE_TYPE + 72)

#define SmGenericAttribute_TYPE      (CONT_BASE_TYPE + 73)  // example of list values to be copied and changed for users needs
#define SmPointerAttribute_TYPE      (CONT_BASE_TYPE + 74)
#define SmPointerListAttribute_TYPE  (CONT_BASE_TYPE + 62)  // note: 62 is out of sequence
#define SmMergeRegionAttribute_TYPE  (CONT_BASE_TYPE + 64)  // note: 64 is out of sequence - for merge operator only
#define SmTagAttribute_TYPE          (CONT_BASE_TYPE + 76)
#define SmColorAttribute_TYPE        (CONT_BASE_TYPE + 66)  // note: 66 is out of sequence

#define SmBucketSort_TYPE            (CONT_BASE_TYPE + 75)
#define SmPointGrid_TYPE             (CONT_BASE_TYPE + 77)
#define SmPointSequence_TYPE         (CONT_BASE_TYPE + 78)
#define SmPointSet3d_TYPE            (CONT_BASE_TYPE + 80)

#define SmAxis2Placement_TYPE        (CONT_BASE_TYPE + 82)
#define SmPolarConversion_TYPE       (CONT_BASE_TYPE + 84)
#define SmVector2d_TYPE              (CONT_BASE_TYPE + 85)
#define SmPoint2d_TYPE               (CONT_BASE_TYPE + 85)
#define SmVector3d_TYPE              (CONT_BASE_TYPE + 86)

#define SmPoint3d_TYPE               (CONT_BASE_TYPE + 86)
#define SmExtent1d_TYPE              (CONT_BASE_TYPE + 88)
#define SmExtent2d_TYPE              (CONT_BASE_TYPE + 90)
#define SmExtent3d_TYPE              (CONT_BASE_TYPE + 92)
#define SmFilletBrep_TYPE            (CONT_BASE_TYPE + 94)

#define SmPeriodicExtent1d_TYPE      (CONT_BASE_TYPE + 96)
#define SmPolarBox_TYPE              (CONT_BASE_TYPE + 98)
#define SmPseudoBox_TYPE             (CONT_BASE_TYPE + 102)
#define SmDerivSurfDefinition_TYPE   (CONT_BASE_TYPE + 104)

#define SmCacheObj_TYPE              (CONT_BASE_TYPE + 106)
#define SmTree_TYPE                  (CONT_BASE_TYPE + 150)  // gwc: changed from CURVE_BASE_TYPE to CONT_BASE_TYPE to make unique
#define SmTreeNode_TYPE              (CONT_BASE_TYPE + 151)  
#define SmHashTable_TYPE             (CONT_BASE_TYPE + 152)  
#define SmObjsInVoxels_TYPE          (CONT_BASE_TYPE + 153)  
#define SmGridElement_TYPE           (CONT_BASE_TYPE + 155)
#define SmSurfaceGridElement_TYPE    (CONT_BASE_TYPE + 159)

// TreeNodeData types
#define SmTreeNodeData_TYPE          (CONT_BASE_TYPE + 160)
#define SmObjectList_TYPE            (CONT_BASE_TYPE + 162)
#define SmBezierAux1d_TYPE           (CONT_BASE_TYPE + 164)
#define SmBezierSpan_TYPE            (CONT_BASE_TYPE + 165)
#define SmBezierAux2d_TYPE           (CONT_BASE_TYPE + 167)
#define SmBezierPatch_TYPE           (CONT_BASE_TYPE + 168)


#define SmPolygonGridElement_TYPE    (CONT_BASE_TYPE + 213)
#define SmHierarchyGridElement_TYPE  (CONT_BASE_TYPE + 217)
#define SmUserGridElement_TYPE       (CONT_BASE_TYPE + 221)
#define SmTrimmingTools_TYPE         (CONT_BASE_TYPE + 225)
#define SmPointClassification_TYPE   (CONT_BASE_TYPE + 308)

#define SmCurveClassification_TYPE   (CONT_BASE_TYPE + 312)
#define SmCurveInterval_TYPE         (CONT_BASE_TYPE + 313)
#define SmSpaceBend_TYPE             (CONT_BASE_TYPE + 316)
#define SmSpaceUnbend_TYPE           (CONT_BASE_TYPE + 318)
#define SmAssertArray_TYPE           (CONT_BASE_TYPE + 320)
#define SmAssertReport_TYPE          (CONT_BASE_TYPE + 324)

#define SmMergeOptions_TYPE          (CONT_BASE_TYPE + 328)

#define SmFirstUserAttribute_TYPE    (CONT_BASE_TYPE + 100000)

// Solver types
#define SmGlobalSolver_TYPE          (SOLV_BASE_TYPE +  4)
#define SmCCGlobalSolver_TYPE        (SOLV_BASE_TYPE +  8)
#define SmCCIGlobalSolver_TYPE       (SOLV_BASE_TYPE + 12)
#define SmCPGlobalSolver_TYPE        (SOLV_BASE_TYPE + 16)
#define SmCSGlobalSolver_TYPE        (SOLV_BASE_TYPE + 20)
#define SmCSIGlobalSolver_TYPE       (SOLV_BASE_TYPE + 24)
#define SmCurvePropertyGS_TYPE       (SOLV_BASE_TYPE + 28)
#define SmLSIGlobalSolver_TYPE       (SOLV_BASE_TYPE + 32)
#define SmSPGlobalSolver_TYPE        (SOLV_BASE_TYPE + 36)
#define SmSSGlobalSolver_TYPE        (SOLV_BASE_TYPE + 40)
#define SmSurfaceIntersector_TYPE    (SOLV_BASE_TYPE + 44)
#define SmAdvSurfaceIntersector_TYPE (SOLV_BASE_TYPE + 48)
#define SmFilletIntersector_TYPE     (SOLV_BASE_TYPE + 52)
#define SmSurfaceTracer_TYPE         (SOLV_BASE_TYPE + 56)
#define SmSurfaceDropCurve_TYPE      (SOLV_BASE_TYPE + 60)
#define SmSurfaceSilhouette_TYPE     (SOLV_BASE_TYPE + 64)
#define SmTopologySolver_TYPE        (SOLV_BASE_TYPE + 68)

/******************************************************************//**
PURPOSE: This enum defines what type of solution is being looked for
    by the various solver algorithms.

NOTES: The solver input SmSolutionRequestedType enum values
  SM_SR_SINGLE and SM_SR_ALL determine whether a single global
  best answer is returned or all local solutions.

  When minimizing and maximizing, all answers within tolerance
  of the global minimum value are returned.  This is different from
  returning all local extrema values.

  To add a new solver type: see the description in the header
  comments for class SmCurvePropertyEFO, in file SmCurve.cpp.
***********************************************************************/
enum SmSolverOperationType
{
  SM_SO_INTERSECT,           // Intersect the objects
  SM_SO_INTERSECT_WIREFRAME, // in SmTopologySolver::BrepCurveSolve limits Curve/Brep intersections to vertices and edges skipping faces
                                   // This option is no longer being used in SMLib.
  SM_SO_INTERSECTION_TEST,   // Test for possible intersection between two spatial decomposition trees
                             // by intersecting the bounding boxes of all leafNode pairs.
                             //   output: 1. Store number of leafNode pairs with intersecting bounding pseudoBoxes.
                             //           2. Store union of all intersecting leafNode bounding box intersections.
                             //           3. Store union of all intersecting leafNode intervals.
                             //   rSolutions[0].m_vStart[6]   = number of leafNode pairs with intersecting bounding boxes
                             //   rSolutions[0].m_vStart[0-5] = BoundingBoxUnion Min/Max XYZ Point Values
                             //   rSolutions[0].m_vEnd[0-3]   = Curve0/Curve1 Intersecting IntervalUnions
                             //   rSolutions[0].m_vEnd[0-7]   = Surf0/Surf1 Intersecting UVDomainUnions
  SM_SO_MINIMIZE,            // Minimize distance between the objects
  SM_SO_MAXIMIZE,            // Maximize distance between the objects
  SM_SO_NORMALIZE,           // Find points which project along Curve and Surface normals
  SM_SO_AT_DISTANCE,         // Find points which are at a given distance.
  SM_SO_FIND,                // Just looking for something
  SM_SO_RAYFIRE,             // This is a minimization of intersections
                             // along a 3D unitized vector also known as a ray firing.
                             // Rayfire is used in conjunction with the Point based
                             // solvers.  The point which defines the start of the ray
                             // is passed in to these solvers as a separate argument.
                             // The vector for the ray is passed in as the first
                             // element of the optional vectors array 'm_cpOptVectors[0]'.
  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, // Minimize 3D signed distance along a given vector.
                                     // Finds the value which is closest to negative infinity
                                     // including negative values.  The 3D vector along which
                                     // the bodies are moved is 'm_cpOptVectors[0]'.  Basically
                                     // this operation finds the first contact point between
                                     // two objects if the first object is moving to the other
                                     // from a great distance away along the given vector.
  SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE, // Maximize 3D signed distance along a given vector.
                                     // Note, this one is currently used by SmSurface global
                                     // and local point solve methods, without
                                     // a point: the direction vector is passed in
                                     // as the test point.

  // The following solvers are all projected solvers.
  //  For most, the projection plane is always the first element of the optional vectors array 'm_cpOptVectors[0]'
  // Any other vectors or points passed in are detailed below.
  SM_SO_SIGNED_DIRECTED_MINIMIZE, // Minimize 2D projected signed distance along a given vector
                                  // Finds the largest magnitude negative value
                                  // The direction vector is passed in as 'm_cpOptVectors[1]'
                                  // Basically this operation finds the first contact point
                                  // (projected) between two objects if the first object is
                                  // moving to the other from a great distance away along
                                  // the given vector.
  SM_SO_DIRECTED_MINIMIZE,        // Minimize 2D projected absolute distance along a given vector
                                  // The direction vector is passed in as 'm_cpOptVectors[1]'
  SM_SO_DIRECTED_MAXIMIZE,        // Maximize 2D projected absolute distance along a given vector
                                  // The direction vector is passed in as 'm_cpOptVectors[1]'
  SM_SO_PROJECTED_MINIMIZE,       // Minimize 2D distance relative to a projection direction
  SM_SO_PROJECTED_MAXIMIZE,       // Maximize 2D distance relative to a projection direction
  SM_SO_PROJECTED_INTERSECT,      // Intersect the projected objects (2D) - used to get silhoette intersections
                                  //   m_cpOptVectors[0] = unit normal to view plane,
  SM_SO_ROTATED_PROJECTED_INTERSECT, // Intersect curves rotated about a common axis - used for rotational sweep checks.
                                     //   m_cpOptVectors[0] = pt on rotation axis,
                                     //   m_cpOptVectors[1] = rotation axis vec,
                                     //   m_cpOptVectors[2] = XAxis of proj Plane
  SM_SO_PERSPECTIVE_INTERSECT,       // Intersect curves perspective objects - used to get silhoette intersections in perspective
                                     //   m_cpOptVectors[0] = pt on view plane,
                                     //   m_cpOptVectors[1] = unit normal to view plane,
                                     //   m_cpOptVectors[2] = eye point
  SM_SO_SURFACE_PROJECTED_INTERSECT, // Intersect the first curve projected to the surface of the
                                     // second curve, which must be an SmCrvOnSurf.
  SM_SO_ANGLE_MINIMIZE,           // Find minimum absolute angle to a plane relative to point on the plane
                                  // Note - that this solver is not currently implemented.
  SM_SO_SIGNED_ANGLE_MINIMIZE,    // Find minimum angle relative to a -180 to 180 degree
                                  // angular measurement (positive is counter clockwise relative to
                                  // the reference vector in the projection plane).
                                  // The reference vector is passed in as 'm_cpOptVectors[1]'
  SM_SO_PROJECTED_TANGENCY,       // Find the projected tangency points of two curves.  These are the
                                  // point where the projection of the line segment between the points
                                  // on the two curves is parallel to the projection of the tangents
                                  // at those two points
  SM_SO_SIGNED_PIVOT_MINIMIZE,    // Find min angle A such that when curve1 is rotated about
                                  // pivot pt by A, it just touches curve2 when viewed along projection vector
                                  // Will consider only one side of Pivot pt specified by a direction vector.
                                  // The pivot point is passed in as 'm_cpOptVectors[1]'
                                  // The reference direction is passed in as 'm_cpOptVectors[2]'
  SM_SO_PROJECTED_TANGENT_THROUGH_POINT  // Find points on the curve whose tangents contain a given
                                         // point, projected to a plane.

} ; // end enum SmSolverOperationType

/*******************************************************************//**
PURPOSE: The Boolean Operation Type defines what sort of operation
         is being performed by SmMerge::ManifoldBoolean

NOTES: The two Brep objects being operated upon by SmMerge::ManifoldBoolean
       are stashed within SmMerge.m_vTI.m_pBrep  (called Solids A)
                   and    SmMerge.m_vTI.m_pOther (called Solids B)

       Different OperationTypes modify the m_pBrep and m_pOther values in
       different ways as indicated in the notes below.
***********************************************************************/
enum SmBooleanOperationType
{
  SM_BO_UNKNOWN,          // an error operation                                | 
  SM_BO_UNION,            // Union of solids A and B                           | rpResult = m_vTI.m_pBrep, delete m_vTI.m_pOther
  SM_BO_INTERSECTION,     // Intersection of solids A and B                    | rpResult = m_vTI.m_pBrep, delete m_vTI.m_pOther
  SM_BO_DIFFERENCE,       // Difference = A minus B                            | rpResult = m_vTI.m_pBrep, delete m_vTI.m_pOther
  SM_BO_EXCLUSIVE_OR,     // ExclusiveOr = (A Union B) - (A intersect B)       | rpResult = m_vTI.m_pBrep, delete m_vTI.m_pOther
  SM_BO_MERGE,            // Merge Operation = Merge B into A                  | rpResult = m_vTI.m_pBrep, delete m_vTI.m_pOther 
  SM_BO_PARTIAL_MERGE,    // Merge subset of otherBrep parts into Brep.        | rpResult = m_vTI.m_pBrep, save   m_vTI.m_pOther
                          //  (list otherBrep subset in m_vTI.m_pSubset)       |
                          //   Does not delete the m_vTI.m_pOther Brep.        |
  SM_BO_IMPRINT,          // Imprint (A Intersect B) results                   | rpResult = m_vTI.m_pBrep, save   m_vTI.m_pOther
                          //   as Edges and Vertices in A                      |
  SM_BO_IMPRINT_CLASSIFY, // Imprint and mark Faces that would be              | rpResult = m_vTI.m_pBrep, save   m_vTI.m_pOther
                          // deleted with attribute SM_AI_BOOLEAN_DELETE.      |
                          //  Does not delete the other Brep.                  |
                          //  BUT don't use this - rather use oneof            |
                          //       SM_BO_UNION,                                |
                          //       SM_BO_INTERSECTION,                         |
                          //       SM_BO_DIFFERENCE,                           |
                          //       SM_BO_EXCLUSIVE_OR,                         |
                          //       SM_BO_MERGE and set                         |
                          //  SmMerge::m_bImprintAndClassifyFaces==TRUE        |
  SM_BO_EXTRACT_SEPARATE, // After merging m_pBrep with m_pOther,              | output = m_vOneBrepPer_OrigBrepConnectedFaceSet array of Breps
                          // place disjoint geometry into new separate         |          m_vOneBrepPer_OtherBrepConnectedFaceSet array of Breps
                          //    Brep objects                                   |      delete m_vTI.m_pBrep, delete m_vTI.m_pOther 
                          //   When using SM_BO_EXTRACT_SEPARATE in            |      rpResult not used 
                          //   SmMerge::ManifoldBoolean(), output is           |
                          //   two or more new Breps placed in                 |
                          //   SmMerge::m_vOneBrepPer_OrigBrepConnectedFaceSet |
                          //   SmMerge::m_vOneBrepPer_OtherBrepConnectedFaceSet|
                          //   normal output rpResult is not used.             |
  SM_BO_SLICE             // Slice = Slice a solid with a sheet and            | rpResult = m_vTI.m_pBrep, delete m_vTI.m_pOther
                          //         keep upper half.                          |
                          //         When m_vTI.m_pOther is not an             |
                          //           oriented sheet make no changes          |
                          //           and return SM_ERR.                      |
                          //         When Slice does not split a region        |
                          //        (just nicks it) behave as                  |
                          //           SM_BO_DIFFERENCE.                 
};    

/*******************************************************************//**
PURPOSE: Classify a pair of SmSolution objects

NOTES:
***********************************************************************/
enum SmSolutionPairType
{  SM_SP_IDENTICAL,       // two solutions have same type, object counts,
                          //     object pointer values, parameter counts,
                          //     and parameter values.
                          //
   SM_SP_DIFFCOUNTS,      // two solutions have different type, object count
                          //     or parameter count.
   SM_SP_DIFFOBJECTS,     // two solutions have different object pointers
   SM_SP_DIFFPARAMETERS   // two solutions only have different parameter values.

} ; // enum SmSolutionPairType


/*******************************************************************//**
PURPOSE: This enum defines what category the operation belongs to

NOTES:
***********************************************************************/
enum SmOperationCategory
{
  SM_OPERATION_MINIMIZE,    // Some kind of minimization
  SM_OPERATION_MAXIMIZE,    // Some kind of maximization
  SM_OPERATION_SIGNED,      // Signed operations (eg. SM_SO_SIGNED_ANGLE_MINIMIZE)
  SM_OPERATION_OTHER        // All other kinds of operations

} ; // end enum SmOperationCategory

/*******************************************************************//**
PURPOSE: This enum defines what type of solutions to produce during the
   global solver algorithms.

NOTES:
***********************************************************************/
enum SmSolutionRequestedType
{
  SM_SR_SINGLE,           // Only produce one solution - the best
  SM_SR_ALL,              // Find all function vals satisfying solutions to given tol
  SM_SR_FIND_AMBIGUITIES, // not currently used
  SM_SR_NODES             // Only find the nodes in the tree where solutions may exist

} ; // end enum SmSolutionRequestedType

/*******************************************************************//**
PURPOSE: This enum defines which criteria to use when sorting answers
    that are added to the global solution.

NOTES:
***********************************************************************/
enum SmSortKeyType
{
  SM_SK_BY_SOLUTION_VALUE,       // Only sort by value
  SM_SK_BY_FIRST_PARAMETER,      // Only sort by first parameter
  SM_SK_BY_FIRST_TWO_PARAMETERS, // Sort by first two parameters - second used to resolve ties
  SM_SK_BY_VALUE_WITH_PARAMETER, // Sort by value - using parameters to resolve ties
  SM_SK_BY_ALL_PARAMETERS        // Use all parameters up to point of resolving ties

} ; // end enum SmSortKeyType

/*******************************************************************//**
PURPOSE: This enum defines the special case color rules used
            to select a color under special circumstances.

NOTES:
***********************************************************************/
enum SmColorRuleType
{
  SM_CR_STANDARD,              // if stackColor - return stack color
                               // else if specialRule - process special rule
                               // else if object->Geometry and geometry->ColorAttribute - return colorAttribute
                               // else if object->ColorAttritbute - return colorAttribute
                               // else if object->Brep and Brep->ColorAttribute - return ColorAttribute
                               // else return defaultColor
  SM_CR_SHADING,               // when returning a default color
                               //      return sDefaultShadingColor
  SM_CR_OBJPROPERTY,           // Called from SmBrep::Draw(), SmBrep::Output Graphics
                               //             SmFace::Draw(), etc.
                               //   used to apply special colors to lamina edges,
                               //   wire edges and the like
  SM_CR_CURVECONTROLPOLYGON,   // use the object attribute color or the curve control points and control polygon color
  SM_CR_SURFACECONTROLPOLYGON, // use the object attribute color or the surface control points and control net color
  SM_CR_VOLUMECONTROLPOLYGON,  // use the object attribute color or the volume control points and control mesh color
  SM_CR_SURFACENORMAL,         // use the SurfaceNormal color
  SM_CR_NEG_SURFACENORMAL,     // 2nd surfaceNormal color for normals pointing into screen
  SM_CR_SURFACENORMAL_INFINITE,// when drawing TypedShells - use this color for a shell that bounds an infinite region
  SM_CR_SURFACENORMAL_SOLID,   // when drawing TypedShells - use this color for a shell that bounds a solid region
  SM_CR_SURFACENORMAL_VOID,    // when drawing TypedShells - use this color for a shell that bounds a void region
  SM_CR_CURVATURE,             // use curvature comb color
  SM_CR_CURVATURE2,            // use curvature comb color 2 (when drawing U and V isoparam Curves with curvature on surfaces)
  SM_CR_SPEED,                 // use speed comb color
  SM_CR_KNOT,                  // use the knot color
  SM_CR_VARYCROSSHATCH,        // use the varied crosshatch line color
  SM_CR_HIGHCOUNTCURVE,        // use the high controlPoint count curve color
  SM_CR_BASESURFACE,           // use the BaseSurface Color for SmOffsetSurface base surface rendering
  SM_CR_EXTENDEDSURFACE,       // use the ExtendedSurface Color for SmOffsetSurface extended surface rendering
  SM_CR_NEIGHBOR,              // use the Neighbor Color for drawing neighbors in DrawNeighbors functions
  SM_CR_NEIGHBORMATE,          // use the Neighbor Color for drawing neighbor mates in DrawNeighbors functions
  SM_CR_HOTPOINT,              // use the hot point Color for drawing pickable hot points
  SM_CR_HOTPOINTFILL,          // use the hot point fill Color to draw non-pickable potions of the hot point display
  SM_CR_HOTPOINTSELECT,        // use the hot point select color for drawing selected hot points
  SM_CR_HOTPOINTHIGHLIGHT,     // use the hot point highlight color for drawing highlighted hot points
  SM_CR_TRACKTARGET,           // use the track target color to display track mode orientation graphics
  SM_CR_TRACKINFO,             // use the track info color to display none track mode tracking orientation graphics
  SM_CR_TESTGEOMETRY,          // use the Test Geometry color to display geometry under interactive construction
  SM_CR_GAPPOINT,              // use the gap color to display gap points
  SM_CR_VERTEX_EDGEGAP,        // use the gap color to display vertexEdge gap lines
  SM_CR_VERTEX_FACEGAP,        // use the gap color to display VertexFace gap lines
  SM_CR_EDGE_FACEGAP,          // use the gap color to display EdgeFace gap lines
  SM_CR_EDGE_EDGEGAP,          // use the gap color to display EdgeEdge gap lines
  SM_CR_VERTEX_UVTRIMCURVEGAP, // use the gap color to display VertexUVTrimCurve gap lines
  SM_CR_EDGE_UVTRIMCURVEGAP,   // use the gap color to display EdgeUVTrimCurve gap lines
  SM_CR_IN_TOL_GAP,            // use the color for In Tolerance GapFunction gap segments
  SM_CR_OUT_TOL_GAP,           // use the color for Out of Tolerance GapFunction gap segments

  SM_CR_UNDEFINED

} ; // end enum SmColorRuleType

/*******************************************************************//**
PURPOSE: This enum is used to control final positioning of the part after unbending.
NOTES:
 ***********************************************************************/
enum SmUnbendOrientTYPE
{
    SM_UO_ORIENT_UNKNOWN,
    SM_UO_FIXED_BEFORE_UNBENDING,  // topology in front of unbend is fixed, inside topology unbends, after topology is rotated into position
    SM_UO_CENTER_UNBENDING,        // topology on either side of unbend is rotated in equal and opposite directions
    SM_UO_FIXED_AFTER_UNBENDING    // topology after unbend is fixed, inside topology unbends, before topology is rotated into position

}; // end enum SmUnbendOrientTYPE

/*******************************************************************//**
PURPOSE: This enum is used to control final positioning of the part after bending.

NOTES:
***********************************************************************/
enum SmBendOrientTYPE
{
    SM_BO_ORIENT_UNKNOWN,
    SM_BO_FIXED_BEFORE_BENDING,  // topology in front of bend is fixed - inside bends, after topology is rotated into position
    SM_BO_CENTER_BENDING,        // topology on either side of bend is rotated in equal and opposite directions
    SM_BO_FIXED_AFTER_BENDING    // topology after bend is fixed - inside topology bends, before topology is rotated into position

}; // end enum SmBendOrientTYPE

/*******************************************************************//**
PURPOSE: List of object cache types.

NOTES: 
***********************************************************************/
enum SmObjectCacheType {
    SM_OC_CURVE     = 0,
    SM_OC_SURFACE   = 1,
    SM_OC_TRIMSRF   = 2,
    SM_OC_BREP      = 3,
    SM_OC_CACHE_TYPE_COUNT,
    SM_OC_UNKNOWN
};
#endif // !__SMCORE_TYPES_H__

