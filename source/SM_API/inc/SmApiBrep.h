// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


/**********************************************************************
FILE NAME: SmBrep.h

PURPOSE: 
    Contains high level "C" type functions that operate on SmBreps.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmMerge, etc
**********************************************************************/


#ifndef __SmApiBrep_H__
#define __SmApiBrep_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmMerge.h>
#include <SmPrimitiveCreation.h>
#include <SmFilletSolver.h>
#include <SmTArray.h>
#include <SmApiTessellationParams.h>

class SmContext;
class SmBrep;
class SmPolyBrep;
class SmEdge;
class SmFace;
class SmVertex;
class SmSurface;
class SmBSplineCurve;
class SmSweepOptions;


/// Two-Brep Booleans (SmApiBoolean, SmApiBooleanUnion, SmApiBooleanDifference,
/// SmApiBooleanIntersection, SmApiBooleanMerge, SmApiBooleanWithCurves,
/// SmApiBooleanWithOptions, SmApiNonManifoldBoolean), in addition to the per-parameter notes:
///  - pBrep2 is not deleted for SM_BO_PARTIAL_MERGE, SM_BO_IMPRINT_CLASSIFY, or
///    SmApiBooleanWithOptions with bKeepOtherBrep = TRUE. SM_BO_IMPRINT does delete it.
///  - A NULL operand, pBrep1 == pBrep2, or SM_BO_EXTRACT_SEPARATE (which has no single
///    result) returns SM_ERR_INVALID_INPUT with both inputs untouched.
///  - rpResult is NULL on failure.
SMAPI_EXPORT SmApiStatus SmApiBoolean( 
    SmBrep*  pBrep1,                      ///< [in/out]: Modified in place and returned as rpResult on success;     <br>
                                           ///<           remains allocated but may be modified on failure          <br>
    SmBrep*  pBrep2,                      ///< [in ]: Deleted on success; remains allocated but may be modified     <br>
                                           ///<        on failure                                                    <br>
    SmBooleanOperationType lOperation,    ///< [in ]: specify boolean operation                                     <br>
    SmBrep*& rpResult                    ///< [out]: Aliases pBrep1 on success; NULL on failure                     <br>
);

SMAPI_EXPORT SmApiStatus SmApiBooleanUnion( 
    SmBrep*  pBrep1,                      ///< [in/out]: Modified in place and returned as rpResult on success;      <br>
                                           ///<           remains allocated but may be modified on failure           <br>
    SmBrep*  pBrep2,                      ///< [in ]: Deleted on success; remains allocated but may be modified      <br>
                                           ///<        on failure                                                     <br>
    SmBrep*& rpResult                    ///< [out]: Aliases pBrep1 on success; NULL on failure                      <br>
);

SMAPI_EXPORT SmApiStatus SmApiBooleanDifference( 
    SmBrep*  pBrep1,                      ///< [in/out]: Modified in place and returned as rpResult on success;      <br>
                                           ///<           remains allocated but may be modified on failure           <br>
    SmBrep*  pBrep2,                      ///< [in ]: Deleted on success; remains allocated but may be modified      <br>
                                           ///<        on failure                                                     <br>
    SmBrep*& rpResult                    ///< [out]: Aliases pBrep1 on success; NULL on failure                      <br>
);

SMAPI_EXPORT SmApiStatus SmApiBooleanIntersection( 
    SmBrep*  pBrep1,                       ///< [in/out]: Modified in place and returned as rpResult on success;      <br>
                                            ///<           remains allocated but may be modified on failure           <br>
    SmBrep*  pBrep2,                       ///< [in ]: Deleted on success; remains allocated but may be modified      <br>
                                            ///<        on failure                                                     <br>
    SmBrep*& rpResult                     ///< [out]: Aliases pBrep1 on success; NULL on failure                      <br>
);

SMAPI_EXPORT SmApiStatus SmApiBooleanMerge( 
    SmBrep*  pBrep1,                        ///< [in/out]: Modified in place and returned as rpResult on success;     <br>
                                             ///<           remains allocated but may be modified on failure          <br>
    SmBrep*  pBrep2,                        ///< [in ]: Deleted on success; remains allocated but may be modified     <br>
                                             ///<        on failure                                                    <br>
    SmBrep*& rpResult                      ///< [out]: Aliases pBrep1 on success; NULL on failure                     <br>
);                                          
                                            
SMAPI_EXPORT SmApiStatus SmApiMergeBreps(                                           
    SmTArray<SmBrep*>& rBreps,              ///< [in/out]: Two or more unique, non-NULL Breps; consumed after validation <br>
    SmBooleanOperationType lOperation,      ///< [in ]: UNION, INTERSECTION, DIFFERENCE, or MERGE                       <br>
    SmBrep*& rpResult                      ///< [out]: Resulting SmBrep; NULL on failure                               <br>
); 

SMAPI_EXPORT SmApiStatus SmApiBooleanWithCurves(
    SmBrep*  pBrep1,                      ///< [in/out]: Modified in place and returned as rpResult on success;     <br>
                                           ///<           remains allocated but may be modified on failure          <br>
    SmBrep*  pBrep2,                      ///< [in ]: Deleted on success; remains allocated but may be modified     <br>
                                           ///<        on failure                                                    <br>
    SmBooleanOperationType lOperation,    ///< [in ]: specify boolean operation                                     <br>
    SmBrep*& rpResult,                    ///< [out]: Aliases pBrep1 on success; NULL on failure                     <br>
    SmTArray<SmEdge*>& rIntersectionEdges ///< [out]: Intersection edges from the boolean (on first brep)         <br>
);

/// If either input has zero faces, returns SM_SUCCESS without assigning
/// rpResult or deleting pBrep2.
SMAPI_EXPORT SmApiStatus SmApiBoolean2d(
  SmBrep*  pBrep1,                        ///< [in/out]: Primary Brep; modified and returned on non-empty success    <br>
  SmBrep*  pBrep2,                        ///< [in ]: Second Brep; deleted on non-empty success                      <br>
  Sm2DBooleanOperationType eOpType,       ///< [in ]: Operation Type                                                    <br>
                                          ///<      : SM_2D_UNION,        = Union of 2D regions A and B             <br>
                                          ///<      : SM_2D_INTERSECTION, = Intersection of 2D regions A and B      <br>
                                          ///<      : SM_2D_DIFFERENCE,   = Difference - A minus B                  <br>
                                          ///<      : SM_2D_EXCLUSIVE_OR, = XOR = (A union B) - (A intersect B)     <br> 
                                          ///<      :  SM_2D_MERGE         = Merge Operation  <br>                  <br>
  SmBrep*& rpResult                      ///< [out]: Modified pBrep1 on non-empty success; otherwise unchanged      <br>
);

/// Tessellate a brep into a SmPolyBrep mesh using the supplied quality controls.
/// Rejects failed faces unless bAllowPartial is TRUE and pFailures or pReport is supplied.
/// Output errors always fail. The caller owns successful output; errors leave it null.
/// pFailures is cleared on entry and retained on error, using input GetFaces() indices.
SMAPI_EXPORT SmApiStatus SmApiTessellate(
    SmBrep* pBrep,                          ///< [in ]: Pointer to brep
    SmPolyBrep*& rpResult,                  ///< [out]: Resulting SmPolyBrep
    const SmTessellationParams& rParams = SmTessellationParams(), ///< [in ]: Quality controls
    SmBoolean bAllowPartial = FALSE,        ///< [in ]: Retain partial meshes when some faces fail
    SmTArray<SmTessellationFailure>* pFailures = nullptr, ///< [out]: First failure per original face
    SmTessellationReport* pReport = nullptr ///< [out]: Nonfatal output topology diagnostics
);

/// Sample trimmed edges; finite, positive angle in degrees. Brep is unchanged.
/// Replaces flat points, counts and borrowed edges.
/// Missing/failed curves: zero counts and SM_ERR; valid samples are retained.
/// Degenerate curves retain one point.
SMAPI_EXPORT SmApiStatus SmApiTessellateBoundaries(
    SmBrep* pBrep,
    double dAngleTolDeg,
    SmTArray<SmPoint3d>& rPoints,
    SmTArray<ULONG>& rVertexCounts,
    SmTArray<SmEdge*>& rEdges
);

/// Replaces every analytic / periodic surface in @p pBrep (cylinder, cone,
/// sphere, torus, plane) with an equivalent NURBS surface in place. No-op on
/// already-NURBS surfaces.
///
/// Analytic surfaces, lines and circles are copied exactly from their NURBS
/// form. Offset surfaces and other edge curves are approximated, and face UV
/// trim curves are removed (later operations regenerate them).
///
/// This operation can replace and delete Surface and Curve objects owned by
/// @p pBrep. Callers must discard any cached pointers to Brep-owned geometry
/// before the call, then reacquire them from their surviving Face or Edge.
SMAPI_EXPORT SmApiStatus SmApiTurnToNurbs(
    SmBrep*  pBrep                          ///< [in/out]: Pointer to brep                                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiProjectBrepOntoPlane(
    SmBrep*  pBrep,                         ///< [in ]: Pointer to brep                                               <br>
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane                             <br>
    const SmVector3d& rPlaneNormal,         ///< [in ]: Direction of projection; must be non-zero, need not be unit    <br>
    SmTArray<SmCurve*>& rProjectionCurves  ///< [out]: Resulting projected curves, appended; on failure the array    <br>
                                           ///<        is left as it was on entry                                    <br>
);

SMAPI_EXPORT SmApiStatus SmApiCut(
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep                                           <br>
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane                            <br>
    const SmVector3d& rPlaneNormal         ///< [in ]: Plane normal; must be non-zero, need not be unit             <br>
);

SMAPI_EXPORT SmApiStatus SmApiProjectAndTrim( 
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep                                           <br>
    SmBSplineCurve* rCurveToProject,        ///< [in ]:                                                              <br>
    const SmVector3d& rProjectionDir,       ///< [in ]: Direction of projection; must be non-zero, need not be unit  <br>
    const SmPoint3d& rRefPt                ///< [in ]: Point on the side of the trim to keep                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSilhouetteCurves(
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep
    const SmVector3d& rPlanePt,             ///< [in ]: Unused (debug draw only); silhouette uses rPlaneNormal
    const SmVector3d& rPlaneNormal,         ///< [in ]: Direction of projection; must be non-zero, need not be unit
    SmTArray<SmCurve*>& rProjectionCurves  ///< [in/out]: New caller-owned curves appended on success; unchanged on failure
);

SMAPI_EXPORT SmApiStatus SmApiProjectCurve(
    SmBrep* pBrep,                          ///< [in ]: Pointer to brep                                             <br>
    SmBSplineCurve* pCurveToProject,        ///< [in ]: Pointer to curve to project                                 <br>
    const SmVector3d& projectionVector,    ///< [in ]: Non-zero projection direction; need not be unit             <br>
    SmTArray<SmCurve*> & r3DCurves         ///< [in/out]: New caller-owned curves appended on success. Unchanged   <br>
                                         ///<           on failure, including a projection producing no curves.  <br>
);

SMAPI_EXPORT SmApiStatus SmApiStitch(
    SmBrep* pBrep                          ///< [in ]: Pointer to brep                                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiStitch(
    SmBrep* pBrep,                          ///< [in ]: Pointer to brep                                             <br>
    SmFace* pFaceToStitch                  ///< [in ]: Pointer to curve to project                                 <br>
);

// Family of Sweep Functions
// These all create SmBrep.
// For simple sweep functions that create a single surface, refer to SmSurfaces

/// rCurvesToSweep are copied, not consumed: the caller keeps ownership and the
/// curves are unchanged. The caller owns the new rpResult.
SMAPI_EXPORT SmApiStatus SmApiCreateLinearSweep( 
    const SmTArray<SmCurve*>& rCurvesToSweep,  ///< [in ]: Curves to sweep                                           <br>
    const SmVector3d& rSweepDir,               ///< [in ]: Sweep direction                                           <br>
    double dSweepDist,                   ///< [in ]: Sweep distance                                            <br>
    SmBoolean bCapEnds,                  ///< [in ]: Cap the ends of the result                                <br>
    SmBrep*& rpResult                   ///< [out]: Resulting SmBrep                                          <br>
) ;


/// rCurvesToSweep are copied, not consumed: the caller keeps ownership and the
/// curves are unchanged. The caller owns the new rpResult.
SMAPI_EXPORT SmApiStatus SmApiCreateRotationalSweep( 
    const SmTArray<SmCurve*>& rCurvesToSweep,  ///< [in ]: Curves to sweep                                            <br>
    const SmPoint3d& rBasePt,                  ///< [in ]: Base point of axis                                         <br>
    const SmVector3d& rAxis,                   ///< [in ]: Direction of axis                                          <br>
    double dAngleDeg,                    ///< [in ]: Angle of rotation                                          <br>
    SmBoolean bCapEnds,                  ///< [in ]: Cap the ends of the result                                 <br>
    SmBrep*& rpResult                   ///< [out]: Resulting SmBrep                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateDraftSweep( 
    const SmTArray<SmCurve*>& rCurvesToSweep, ///< [in ]: Curves to sweep                                           <br>
    double dHeight,                     ///< [in ]: Height of sweep or extrusion                              <br>
    double dAngleDeg,                   ///< [in ]: Draft Angle                                               <br>
    SmBoolean bCornerType,              ///< [in ]: 0 = SM_OC_LINEAR_EXTENSION, 1 = SM_OC_FILLET_CORNER       <br>
    int iCapEnds,                       ///< [in ]: 0 = none, 1 = at curves, 2 = at offset, 3 = both ends     <br>
    SmBrep*& rpResult                  ///< [out]: Resulting SmBrep                                          <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreatePipeSweep( 
    double              dPipeRadius,    ///< [in ]: Radius                                             <br>
    SmBSplineCurve    * pPathCurve,     ///< [in ]: Path Curve                                         <br>
    SmBoolean           bCapEnds,       ///< [in ]: 
    SmBrep*&            rpResult        ///< [out]: Resulting SmBrep                                          <br>
);  

/**
 * Pure producer that sweeps profile boundary curves along a planar path.
 *
 * Simple, coplanar closed profile loops define planar regions by even/odd
 * nesting.  For an open path, BOTH endpoint caps request a nested solid;
 * NONE, START, or END leave a sheet or open shell.  A closed path has no
 * endpoint caps and requests a nested solid when the swept boundary closes.
 * Completed manifold boundaries are classified from the infinite void region
 * inward, alternating bounded material and void regions at each shell.
 * Open profile wires remain sheet geometry.  Tangent-discontinuous path joins
 * use sharp, mitered transitions; this function does not add rounded corners.
 */
SMAPI_EXPORT SmApiStatus SmApiCreateSweepAlongPlanarPath( 
    SmTArray<SmCurve*>& rProfileCurves, ///< [in ]: Coplanar profile boundary curves                         <br>
    SmTArray<SmCurve*>& rPathCurves,    ///< [in ]: Ordered, endpoint-connected curves forming an open or     <br>
                                        ///<        closed planar path                                         <br>
    SmBoolean           bMoveProfile,   ///< [in ]: TRUE anchors the profile bounding-box center to the path  <br>
                                        ///<        start; FALSE anchors the first profile-curve point there   <br>
    int iCapEnds,                       ///< [in ]: 0 = none, 1 = profile/start, 2 = far/path, 3 = both ends; <br>
                                        ///<        ignored for closed paths; other values return             <br>
                                        ///<        SM_ERR_INVALID_INPUT                                      <br>
    SmBrep*& rpResult                  ///< [out]: New SmBrep; inputs unchanged; NULL on failure               <br>
);

// Advanced Sweep Functions

/// The profile, path, and scale curves are not consumed; the caller keeps
/// ownership and all inputs remain unchanged. The caller owns the new rpResult
/// on success; on failure rpResult is NULL (any partially built Brep is deleted).
/// The same applies to SmApiCreateCurveSweepFromFaces and
/// SmApiCreateCurveSweepFromEdges.
SMAPI_EXPORT SmApiStatus SmApiCreateCurveSweep(
    SmTArray<SmCurve*>& rProfileCurves,   ///< [in ]: Profile curves to sweep along path                        <br>
    SmBSplineCurve* pPathCurve,           ///< [in ]: Path curve                                                <br>
    SmBSplineCurve* pScaleReference,      ///< [in ]: Scale reference curve (NULL for none)                     <br>
    SmBSplineCurve* pScaleCurve,          ///< [in ]: Scale curve (NULL for none)                               <br>
    SmSweepOptions* pOptions,             ///< [in ]: Sweep options (NULL for defaults)                         <br>
    SmBrep*& rpResult,                    ///< [out]: Resulting SmBrep                                          <br>
    SmTArray<SmFace*>* pOptStartFaces,    ///< [opt]: Start cap faces (NULL to ignore)                         <br>
    SmTArray<SmFace*>* pOptSideFaces,     ///< [opt]: Side faces (NULL to ignore)                              <br>
    SmTArray<SmFace*>* pOptEndFaces      ///< [opt]: End cap faces (NULL to ignore)                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCurveSweepFromFaces(
    SmTArray<SmFace*>& rFaces,            ///< [in ]: Faces whose edge loops to sweep                          <br>
    SmBSplineCurve* pPathCurve,           ///< [in ]: Path curve                                                <br>
    SmBSplineCurve* pScaleReference,      ///< [in ]: Scale reference curve (NULL for none)                     <br>
    SmBSplineCurve* pScaleCurve,          ///< [in ]: Scale curve (NULL for none)                               <br>
    SmSweepOptions* pOptions,             ///< [in ]: Sweep options (NULL for defaults)                         <br>
    SmBrep*& rpResult,                    ///< [out]: Resulting SmBrep                                          <br>
    SmTArray<SmFace*>* pOptStartFaces,    ///< [opt]: Start cap faces (NULL to ignore)                         <br>
    SmTArray<SmFace*>* pOptSideFaces,     ///< [opt]: Side faces (NULL to ignore)                              <br>
    SmTArray<SmFace*>* pOptEndFaces      ///< [opt]: End cap faces (NULL to ignore)                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCurveSweepFromEdges(
    SmTArray<SmEdge*>& rEdges,            ///< [in ]: Edges whose curves to sweep                              <br>
    SmBSplineCurve* pPathCurve,           ///< [in ]: Path curve                                                <br>
    SmBSplineCurve* pScaleReference,      ///< [in ]: Scale reference curve (NULL for none)                     <br>
    SmBSplineCurve* pScaleCurve,          ///< [in ]: Scale curve (NULL for none)                               <br>
    SmSweepOptions* pOptions,             ///< [in ]: Sweep options (NULL for defaults)                         <br>
    SmBrep*& rpResult,                    ///< [out]: Resulting SmBrep                                          <br>
    SmTArray<SmFace*>* pOptStartFaces,    ///< [opt]: Start cap faces (NULL to ignore)                         <br>
    SmTArray<SmFace*>* pOptSideFaces,     ///< [opt]: Side faces (NULL to ignore)                              <br>
    SmTArray<SmFace*>* pOptEndFaces      ///< [opt]: End cap faces (NULL to ignore)                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateTaperExtrude(
    const SmTArray<SmCurve*>& rCurves,          ///< [in ]: Coplanar curves to extrude                               <br>
    double dHeight,                       ///< [in ]: Extrusion height (+/- for direction)                     <br>
    double dDraftAngleDeg,                ///< [in ]: Draft angle: 0-90 inner, 90-180 outer                    <br>
    int iEndCaps,                         ///< [in ]: 0=none, 1=at curves, 2=at offset, 3=both ends            <br>
    SmBoolean bFilletCorner,              ///< [in ]: TRUE = fillet corners, FALSE = linear extension           <br>
    SmBrep*& rpResult                    ///< [out]: Resulting SmBrep                                          <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateLinearSweepWithRepetitions(
    const SmTArray<SmCurve*>& rCurvesToSweep,   ///< [in ]: Curves to sweep                                         <br>
    const SmVector3d& rSweepDir,                ///< [in ]: Sweep direction                                          <br>
    double dSweepDist,                    ///< [in ]: Sweep distance                                           <br>
    ULONG nRepetitions,                   ///< [in ]: Number of end-to-end sweeps                              <br>
    SmBoolean bCapEnds,                   ///< [in ]: Cap the ends of the result                               <br>
    SmBrep*& rpResult                    ///< [out]: Resulting SmBrep                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateRotationalSweepWithRepetitions(
    const SmTArray<SmCurve*>& rCurvesToSweep,   ///< [in ]: Curves to sweep                                         <br>
    const SmPoint3d& rBasePt,                   ///< [in ]: Base point of axis                                       <br>
    const SmVector3d& rAxis,                    ///< [in ]: Direction of axis                                        <br>
    double dAngleDeg,                     ///< [in ]: Angle of rotation                                        <br>
    ULONG nRepetitions,                   ///< [in ]: Number of end-to-end revolutions                         <br>
    SmBoolean bCapEnds,                   ///< [in ]: Cap the ends of the result                               <br>
    SmBrep*& rpResult                    ///< [out]: Resulting SmBrep                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiNonManifoldSweep(
    SmBrep* pBrepToSweep,                 ///< [in/out]: Brep containing topology to sweep; modified in place  <br>
    SmBrep* pRegionBrep,                  ///< [in ]: Target region's Brep. NULL or pBrepToSweep; any other   <br>
                                          ///<      : Brep returns SM_ERR_INVALID_INPUT, because only          <br>
                                          ///<      : pBrepToSweep is stitched, validated and returned.        <br>
                                          ///<      : SmTopologySweep::DoSweep still accepts a distinct Brep.  <br>
    const SmVector3d& rSweepDir,                ///< [in ]: Sweep direction vector                                  <br>
    double dSweepDist,                    ///< [in ]: Sweep distance                                          <br>
    SmBoolean bDoStitching,               ///< [in ]: TRUE = stitch after sweep                               <br>
    SmBoolean bDoMerge,                   ///< [in ]: TRUE = merge (catches self-intersections)               <br>
    SmTArray<SmFace*>* pOptFaces,         ///< [opt]: Faces to sweep into solids, NULL to ignore faces        <br>
    SmTArray<SmEdge*>* pOptEdges,         ///< [opt]: Edges to sweep into faces, NULL to ignore edges         <br>
    SmTArray<SmVertex*>* pOptVertices    ///< [opt]: Vertices to sweep into edges, NULL to ignore vertices;  <br>
                                          ///<      : only when all three are NULL is the whole Brep swept.  <br>
                                          ///<      : Every entry must be live topology of pBrepToSweep; a    <br>
                                          ///<      : null, foreign or stale handle returns                   <br>
                                          ///<      : SM_ERR_INVALID_INPUT and nothing is modified.           <br>
);

SMAPI_EXPORT SmApiStatus SmApiNonManifoldRotationalSweep(
    SmBrep* pBrepToSweep,                 ///< [in/out]: Brep containing topology to sweep; modified in place  <br>
    SmBrep* pRegionBrep,                  ///< [in ]: Target region's Brep. NULL or pBrepToSweep; any other   <br>
                                          ///<      : Brep returns SM_ERR_INVALID_INPUT, because only          <br>
                                          ///<      : pBrepToSweep is stitched, validated and returned.        <br>
                                          ///<      : SmTopologySweep::DoSweep still accepts a distinct Brep.  <br>
    const SmPoint3d& rBasePt,                   ///< [in ]: Rotation axis base point                                <br>
    const SmVector3d& rAxis,                    ///< [in ]: Rotation axis direction                                 <br>
    double dAngleDeg,                     ///< [in ]: Angle of rotation in degrees                            <br>
    SmBoolean bDoStitching,               ///< [in ]: TRUE = stitch after sweep                               <br>
    SmBoolean bDoMerge,                   ///< [in ]: TRUE = merge (catches self-intersections)               <br>
    SmTArray<SmFace*>* pOptFaces,         ///< [opt]: Faces to sweep into solids, NULL to ignore faces        <br>
    SmTArray<SmEdge*>* pOptEdges,         ///< [opt]: Edges to sweep into faces, NULL to ignore edges         <br>
    SmTArray<SmVertex*>* pOptVertices    ///< [opt]: Vertices to sweep into edges, NULL to ignore vertices;  <br>
                                          ///<      : only when all three are NULL is the whole Brep swept.  <br>
                                          ///<      : Every entry must be live topology of pBrepToSweep; a    <br>
                                          ///<      : null, foreign or stale handle returns                   <br>
                                          ///<      : SM_ERR_INVALID_INPUT and nothing is modified.           <br>
);

// Advanced Boolean Functions

SMAPI_EXPORT SmApiStatus SmApiBooleanWithOptions(
    SmBrep* pBrep1,                       ///< [in/out]: Modified in place and returned as rpResult on success;
                                          ///<           remains allocated but may be modified on failure <br>
    SmBrep* pBrep2,                       ///< [in ]: Deleted on success unless kept (see the Boolean notes
                                          ///<        above); remains allocated but may be modified on failure <br>
    SmBooleanOperationType eOperation,    ///< [in ]: Boolean operation type                                  <br>
    SmBoolean bCookieCutter,              ///< [in ]: TRUE = only remove faces from BrepA                     <br>
    SmBoolean bImprinting,                ///< [in ]: TRUE = imprint intersection on BrepA and quit           <br>
    SmBoolean bBooleanPostProcess,        ///< [in ]: TRUE = remove topological edges/vertices after boolean  <br>
    SmBoolean bImprintAndClassifyFaces,   ///< [in ]: TRUE = imprint and mark faces to be deleted             <br>
    SmBoolean bKeepOtherBrep,             ///< [in ]: TRUE = do not delete BrepB                              <br>
    SmBrep*& rpResult                    ///< [out]: Resulting SmBrep                                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiNonManifoldBoolean(
    SmBrep* pBrep1,                       ///< [in/out]: Modified in place and returned as rpResult on success;
                                          ///<           remains allocated but may be modified on failure <br>
    SmBrep* pBrep2,                       ///< [in ]: Deleted on success unless kept (see the Boolean notes
                                          ///<        above); remains allocated but may be modified on failure <br>
    SmBooleanOperationType eOperation,    ///< [in ]: Boolean operation type                                  <br>
    SmBrep*& rpResult                    ///< [out]: Resulting SmBrep                                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiPiecewiseMerge(
    SmBrep* pBrep1,                       ///< [in/out]: Primary brep; modified and returned as rpResult  <br>
    SmBrep* pBrep2,                       ///< [in/out]: Second brep; imprinted but not deleted; must    <br>
                                          ///<          differ from pBrep1                               <br>
    SmBoolean bCreateNewBrepsForFaces,    ///< [in ]: TRUE = treat each other face as a sheet face            <br>
    SmBrep*& rpResult                    ///< [out]: Resulting SmBrep                                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiBooleanLists(
    const SmTArray<SmBrep*>& rBreps1,           ///< [in ]: First list of breps                                    <br>
    const SmTArray<SmSurface*>& rSurfaces1,     ///< [in ]: First list of surfaces (for planar 2D)                 <br>
    const SmTArray<SmBrep*>& rBreps2,           ///< [in ]: Second list of breps                                   <br>
    const SmTArray<SmSurface*>& rSurfaces2,     ///< [in ]: Second list of surfaces (for planar 2D)                <br>
    int operation,                        ///< [in ]: 0=AND, 1=OR, 2=XOR, 3=A-B, 4=B-A                      <br>
    SmTArray<SmBrep*>& rResultBreps,      ///< [out]: Output breps                                           <br>
    SmTArray<SmSurface*>& rResultSurfaces///< [out]: Output surfaces (if all inputs 2D)                     <br>
);

SMAPI_EXPORT SmApiStatus SmApiBooleanTrees(
    SmTArray<SmBrep*>& rBreps,            ///< [i/o]: Array of breps; result plus unused at end              <br>
    SmTArray<long>& rPostFixTrees        ///< [in ]: CSG tree in postfix notation                           <br>
);

// Advanced Fillet Functions

SMAPI_EXPORT SmApiStatus SmApiFilletEdges(
    SmBrep* pBrep,                        ///< [in/out]: Brep to fillet                                      <br>
    SmTArray<SmEdge*>& rEdges,            ///< [in ]: Edges to fillet                                        <br>
    double dRadius,                       ///< [in ]: Fillet radius                                          <br>
    ULONG lXSectType,                     ///< [in ]: 0=linear, 1=circular, 2=blend curve                    <br>
    ULONG lContinuity,                    ///< [in ]: 1=G1, 2=G2, 3=G3 for blend curves; three-edge corner   <br>
                                          ///<        boundaries guarantee only G1 regardless of this value  <br>
    double dThumbweight                  ///< [in ]: Thumbweight for blend (default 1.0)                    <br>
);

SMAPI_EXPORT SmApiStatus SmApiFilletEdgesPerEdge(
    SmBrep* pBrep,                                          ///< [in/out]: Brep to fillet                                  <br>
    SmTArray<SmEdge*>& rEdges,                              ///< [in ]: Edges to fillet                                    <br>
    SmTArray<double>& rRadii,                               ///< [in ]: Per-edge radius values                             <br>
    SmTArray<SmFilletSurfaceGeneratorType>& rXSectTypes,    ///< [in ]: Per-edge cross section types                       <br>
    ULONG lContinuity,                                      ///< [in ]: 1=G1, 2=G2, 3=G3; three-edge corner boundaries    <br>
                                                            ///<        guarantee only G1 regardless of this value          <br>
    double dThumbweight                                    ///< [in ]: Thumbweight for blend (default 1.0)                <br>
);

SMAPI_EXPORT SmApiStatus SmApiVariableRadiusFillet(
    SmBrep* pBrep,                        ///< [in/out]: Brep to fillet                                      <br>
    SmTArray<SmEdge*>& rEdges,            ///< [in ]: Edges to fillet                                        <br>
    double dStartRadius,                  ///< [in ]: Radius at start of edge                                <br>
    double dEndRadius,                    ///< [in ]: Radius at end of edge                                  <br>
    ULONG lXSectType,                     ///< [in ]: 0=linear, 1=circular, 2=blend curve                    <br>
    ULONG lContinuity                    ///< [in ]: 1=G1, 2=G2, 3=G3 for blend curves; three-edge corner  <br>
                                          ///<        boundaries guarantee only G1 regardless of this value  <br>
);

SMAPI_EXPORT SmApiStatus SmApiSurfaceSurfaceFillet(
    SmSurface* pSurface1,                 ///< [in ]: First surface                                          <br>
    SmSurface* pSurface2,                 ///< [in ]: Second surface                                         <br>
    double dRadius1,                      ///< [in ]: Signed radius from surface 1                           <br>
    double dRadius2,                      ///< [in ]: Signed radius from surface 2                           <br>
    double dTolerance,                    ///< [in ]: Max dist from rails to base surfaces                   <br>
    SmBrep*& rpResult,                    ///< [out]: Brep with fillet surface(s)                            <br>
    ULONG lXSectType,                     ///< [in ]: 0=linear, 1=approx circular, 2=circular                <br>
    SmBoolean bMirror,                    ///< [in ]: TRUE = mirror fillet (inside-out)                      <br>
    SmBoolean bComplement                ///< [in ]: TRUE = complement circular cross section               <br>
);

SMAPI_EXPORT SmApiStatus SmApiFilletPreview(
    SmBrep* pBrep,                        ///< [in ]: Brep with fillet setup                                 <br>
    SmTArray<SmEdge*>& rEdges,            ///< [in ]: Edges to preview fillet for                            <br>
    double dRadius,                       ///< [in ]: Fillet radius                                          <br>
    SmTArray<SmSurface*>& rPreviewSurfaces ///< [out]: Preview fillet surfaces                              <br>
);

SMAPI_EXPORT SmApiStatus SmApiSetBevelCorners(
    SmBrep* pBrep,                        ///< [in/out]: Brep to fillet                                      <br>
    SmTArray<SmEdge*>& rEdges,            ///< [in ]: Edges to fillet                                        <br>
    double dRadius,                       ///< [in ]: Fillet radius                                          <br>
    SmTArray<SmVertex*>& rBevelVertices  ///< [in ]: Vertices to bevel                                     <br>
);

// Advanced Offset / Shell Functions

SMAPI_EXPORT SmApiStatus SmApiShellBrepFull(
    const SmBrep* pBrep,                  ///< [in ]: Brep to shell; left unchanged                          <br>
    double dOffsetDistance,                ///< [in ]: Offset distance                                        <br>
    SmBoolean bDoExtendedOffset,          ///< [in ]: TRUE = extend/intersect convex edges                   <br>
    SmBoolean bDoSelfInt,                 ///< [in ]: TRUE = handle self-intersections                       <br>
    SmBoolean bCreateOffsetSolid,         ///< [in ]: For sheet bodies, TRUE = ruled surfaces between lamina  <br>
                                          ///<        edges; manifold inputs always return Boolean wall volume <br>
    const SmTArray<const SmFace*>& rFacesToShell, ///< [in ]: Faces from pBrep to shell; left unchanged        <br>
    SmBrep*& rpResult                    ///< [out]: New shell, or NULL on failure                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiOffsetBrepFull(
    SmBrep* pBrep,                        ///< [in ]: Brep to offset                                         <br>
    double dOffsetDistance,                ///< [in ]: Offset distance                                        <br>
    SmBoolean bDoExtendedOffset,          ///< [in ]: TRUE = extend/intersect convex edges                   <br>
    SmBoolean bDoSelfInt,                 ///< [in ]: TRUE = handle self-intersections                       <br>
    SmBrep*& rpResult                    ///< [out]: Resulting offset                                       <br>
);

// Advanced Stitching Functions

SMAPI_EXPORT SmApiStatus SmApiStitchIntoSolid(
    SmBrep* pBrep,                        ///< [in ]: Brep to stitch                                         <br>
    SmBoolean& rbProducesASolid,          ///< [out]: TRUE if result is manifold solid                       <br>
    ULONG& rlStitchedEdges,               ///< [out]: Number of edges stitched                               <br>
    double& rdMaxVertexGap,               ///< [out]: Max vertex gap                                         <br>
    double& rdMaxEdgeGap                 ///< [out]: Max edge gap                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiStitchIntoShell(
    SmBrep* pBrep,                        ///< [in ]: Brep to stitch                                         <br>
    SmBoolean& rbShellIsWellFormed,       ///< [in ]: Stitching mode, not a result: pass TRUE only for a     <br>
                                          ///<        well-formed shell (e.g. from a solid modeler); FALSE  <br>
                                          ///<        glues edges/vertices and ignores other problems.      <br>
                                          ///<        Not written.                                          <br>
    double dMaxStitchingRatio,            ///< [in ]: Max allowed separation, an absolute distance in model  <br>
                                          ///<        units (not a ratio)                                   <br>
    ULONG& rlStitchedEdges,               ///< [out]: Number of edges stitched                               <br>
    double& rdMaxVertexGap,               ///< [out]: Max vertex gap                                         <br>
    double& rdMaxEdgeGap                 ///< [out]: Max edge gap                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiUnifyNormals(
    SmBrep* pBrep,                        ///< [in ]: Brep to orient                                         <br>
    SmFace* pStartFace,                   ///< [in ]: Face to use as orientation reference                   <br>
    ULONG lNumSamples,                    ///< [in ]: Number of samples for ray classification               <br>
    SmTArray<SmFace*>& rFlippedFaces     ///< [out]: Faces whose normals were flipped                      <br>
);

SMAPI_EXPORT SmApiStatus SmApiSimpleFaceStitch(
    SmBrep* pBrep,                        ///< [in ]: Brep to stitch                                         <br>
    const SmTArray<SmFace*>& rFacesToKeep,      ///< [in ]: Faces to keep                                         <br>
    const SmTArray<SmFace*>& rFacesToDelete,    ///< [in ]: Matching faces to delete (glued to keeps)             <br>
    ULONG& rlStitchedEdges,               ///< [out]: Number of edges stitched                               <br>
    double& rdMaxVertexGap,               ///< [out]: Max vertex gap                                         <br>
    double& rdMaxEdgeGap                 ///< [out]: Max edge gap                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiStitchAdvanced(
    SmBrep* pBrep,                        ///< [in ]: Brep to stitch                                         <br>
    double dStitchTol3d,                  ///< [in ]: 3D tolerance for stitching                             <br>
    SmBoolean bSqueezeSmallEdges,         ///< [in ]: TRUE = remove edges shorter than tolerance             <br>
    SmBoolean bSplitEdgesWithVertices,    ///< [in ]: TRUE = split edges at nearby vertices                  <br>
    SmBoolean bMakingManifoldSolid,       ///< [in ]: TRUE = only glue lamina edge pairs                    <br>
    SmBoolean bRemoveLaminarSlivers,      ///< [in ]: TRUE = remove sliver faces before stitching           <br>
    ULONG& rlStitchedEdges,               ///< [out]: Number of edges stitched                               <br>
    ULONG& rlLaminaEdges,                 ///< [out]: Number of lamina edges remaining                      <br>
    double& rdMaxVertexGap,               ///< [out]: Max vertex gap found                                  <br>
    double& rdMaxEdgeGap                 ///< [out]: Max edge gap found                                    <br>
);


#endif
