// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiQueries.h

PURPOSE:
    Read-only geometric / topological queries on Brep / Face / Edge /
    Vertex / Surface / Curve.  Three loose subgroups, all answer
    "tell me X about this object":

      * Closest-point and topology search:
          SmApiBrepClosestPoint / -All, SmApiCurveClosestPoint / -All,
          SmApiSurfaceClosestPoint, SmApiGetClosestPoint,
          SmApiFindEdge, SmApiFindFaces.
      * Topology accessors:
          SmApiGetEdges, SmApiGetFaces, SmApiFaceGetBoundaryLoops,
          SmApiVertexGetPoint.
      * Geometric / topological properties:
          SmApiBrepIsManifoldSolid, SmApiBrepMaterialCensus,
          SmApiBrepComputeVolume, SmApiBrepComputeArea,
          SmApiBrepBoundingBox, SmApiBrepCopy,
          SmApiFaceBoundingBox, SmApiFaceComputeArea,
          SmApiEdgeBoundingBox, SmApiEdgeComputeLength.

**********************************************************************/

#ifndef __SM_API_QUERIES_H__
#define __SM_API_QUERIES_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmTArray.h>

class SmPoint3d;
class SmPoint2d;
class SmCurve;
class SmSurface;
class SmBrep;
class SmEdge;
class SmFace;
class SmVertex;


SMAPI_EXPORT SmApiStatus SmApiGetClosestPoint
(
    const SmBrep* pBrep,                          ///< [in ]: Brep to query                                               <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on brep                               <br>
    SmPoint3d& rClosestPoint,               ///< [out]: Closest point on brep                                       <br>
    double& rdDistance                      ///< [out]: Distance from input point to closest point                  <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepRelationship
(
    const SmBrep* pBrepA,                         ///< [in ]: First Brep                                                  <br>
    const SmBrep* pBrepB,                         ///< [in ]: Second Brep                                                 <br>
    int&    rRelationship,                  ///< [out]: 0 separate,1 touching,2 interpenetrating,3 A contains B,4 B in A <br>
    double& rdDistance                      ///< [out]: surface gap; wall clearance when one contains the other; 0 when touching/interpenetrating <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepDistance
(
    const SmBrep* pBrepA,                         ///< [in ]: First Brep                                                  <br>
    const SmBrep* pBrepB,                         ///< [in ]: Second Brep                                                 <br>
    double& rdDistance                      ///< [out]: surface gap; wall clearance when one contains the other; 0 when touching/interpenetrating <br>
);

SMAPI_EXPORT SmApiStatus SmApiGetEdges
(
    SmBrep* pBrep,                          ///< [in ]: Brep to query                                               <br>
    SmTArray<SmEdge*>& rEdges              ///< [out]: Edges of brep                                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiGetFaces
(
    SmBrep* pBrep,                          ///< [in ]: Brep to query                                               <br>
    SmTArray<SmFace*>& rFaces              ///< [out]: Faces of brep                                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiFaceGetBoundaryLoops
(
    SmFace* pFace,                          ///< [in ]: Face to query                                               <br>
    SmTArray<ULONG>& rEdgeCountsPerLoop,   ///< [out]: Edge-occurrence count for each loop, outer loop first;      <br>
                                           ///<        later-loop order is unspecified; a vertex-only loop          <br>
                                           ///<        contributes a zero count                                     <br>
    SmTArray<SmEdge*>& rEdges,             ///< [out]: Borrowed Brep-owned edges, flattened loop by loop in         <br>
                                           ///<        traversal order; each loop's starting occurrence is          <br>
                                           ///<        unspecified; repeated seam occurrences are retained;         <br>
                                           ///<        invalid after the owning Brep's topology changes             <br>
    SmTArray<SmOrientType>& rOrientations ///< [out]: Orientation per edge occurrence; SM_OT_SAME traverses        <br>
                                           ///<        edge start/min to end/max, SM_OT_OPPOSITE traverses reverse   <br>
);

/// Returns the edge nearest to crPoint. There is no distance limit: a point far from every edge
/// still returns the nearest one, so check the distance if the point may be off the Brep.
SMAPI_EXPORT SmApiStatus SmApiFindEdge
(
    SmBrep* pBrep,                          ///< [in ]: Brep to search                                              <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point on or near edge                                       <br>
    ULONG& rlEdgeIndex,                     ///< [out]: Index of found edge in brep                                 <br>
    double& rdParam                        ///< [out]: Parameter on found edge                                     <br>
);

SMAPI_EXPORT SmApiStatus SmApiFindFaces
(
    SmBrep* pBrep,                          ///< [in ]: Brep to search                                              <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point on or near face                                       <br>
    SmFace*& rpFace                        ///< [out]: Pointer to found face                                       <br>
);


/// Returns SM_ERR_INVALID_INPUT for a NULL curve, propagates solver/evaluation
/// failures, and returns SM_ERR if the solve succeeds without any solution.
SMAPI_EXPORT SmApiStatus SmApiCurveClosestPoint
(
    const SmCurve* pCurve,                        ///< [in ]: Curve to query                                              <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on curve                              <br>
    SmPoint3d& rClosestPoint,               ///< [out]: Closest point on curve                                      <br>
    double& rdParameter,                    ///< [out]: Curve parameter at closest point                            <br>
    double& rdDistance                      ///< [out]: Distance from input point to closest point                  <br>
);

/// Same status rules as SmApiCurveClosestPoint. Appends aligned result entries;
/// an evaluation failure can leave a successfully evaluated prefix appended.
SMAPI_EXPORT SmApiStatus SmApiCurveClosestPointAll
(
    const SmCurve* pCurve,                        ///< [in ]: Curve to query                                              <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on curve                              <br>
    SmTArray<SmPoint3d>& rPoints,           ///< [out]: All closest points on curve                                 <br>
    SmTArray<double>& rParameters,          ///< [out]: Curve parameters at each closest point                      <br>
    SmTArray<double>& rDistances           ///< [out]: Distances from input point to each closest point            <br>
);

SMAPI_EXPORT SmApiStatus SmApiSurfaceClosestPoint
(
    const SmSurface* pSurface,                    ///< [in ]: Surface to query                                            <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on surface                            <br>
    SmPoint3d& rClosestPoint,               ///< [out]: One globally closest point on surface                       <br>
    SmTArray<SmPoint2d>& rUVs,              ///< [out]: Nonempty solver-reported natural/NURBS UVs for rClosestPoint; <br>
                                            ///<        may not contain every preimage                              <br>
    double& rdDistance                      ///< [out]: Distance from input point to rClosestPoint                  <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepClosestPoint
(
    const SmBrep* pBrep,                          ///< [in ]: Brep to query                                               <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on brep                               <br>
    SmPoint3d& rClosestPoint,               ///< [out]: Closest point on brep                                       <br>
    double& rdDistance                      ///< [out]: Distance from input point to closest point                  <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepClosestPointAll
(
    const SmBrep* pBrep,                          ///< [in ]: Brep to query                                               <br>
    const SmPoint3d& crPoint,                     ///< [in ]: Point to find closest on brep                               <br>
    SmTArray<SmPoint3d>& rPoints,           ///< [out]: All closest points on brep                                  <br>
    SmTArray<double>& rDistances           ///< [out]: Distances from input point to each closest point            <br>
);


// ============================================================================
//   Geometric / topological properties
// ============================================================================
//
// Read-only properties of a Brep / Face / Edge / Vertex.  None of these
// modify their input.  Bounding boxes wrap SmObject::CalculateBoundingBox
// (and CalculateTightBoundingBox when bTight=TRUE).  Area and length wrap
// the kernel's numerical-integration entry points.  Area uses a
// [1e-4, 1e-1] relative-accuracy envelope; volume uses [1e-8, 1e-1].
// SmApiBrepCopy uses SmBrep's copy constructor allocated against the
// SM_API context, with analytic recognition disabled so geometry types
// are preserved (including an explicit conversion to NURBS).
//
// For whole-solid area+volume+mass+centroid+inertia use one
// SmApiBrepComputeMassProperties call (SmApiBrepComputeVolume shares its
// core). Mass properties integrate all quantities in one pass; per-face
// area and centroid queries integrate only the quantities needed for their
// results. Cost is governed by dRelativeAccuracy. For interactive preview,
// use a coarse dRelativeAccuracy (~1e-1 is much cheaper than 1e-3),
// or skip exact integration entirely: the SmApiTessellate PolyBrep you already
// build for rendering yields an approximate volume for a fraction of the cost
// (closed manifold mesh only -- meaningless for open/partial tessellations).
// SmApiBrepBoundingBox bTight=FALSE is a cheap loose box (bTight=TRUE is
// expensive).

SMAPI_EXPORT SmApiStatus SmApiBrepIsManifoldSolid(
    const SmBrep* pBrep,                  ///< [in ]: Pointer to brep                                               <br>
    SmBoolean & rbIsManifoldSolid         ///< [out]: TRUE = manifold solid, FALSE = not                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepMaterialCensus(
    const SmBrep*  pBrep,                       ///< [in ]: Pointer to brep (read-only)                                   <br>
    long &   rlSolidCount,                ///< [out]: Number of material (solid) regions; excludes the             <br>
                                          ///<        infinite region and every void cavity                          <br>
    long &   rlVoidCount                  ///< [out]: Number of enclosed void cavities; excludes the single         <br>
                                          ///<        infinite (unbounded) region                                    <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepComputeVolume(
    const SmBrep*  pBrep,                       ///< [in ]: Pointer to brep                                               <br>
    double   dRelativeAccuracy,           ///< [in ]: Relative accuracy clamped to [1.0e-8, 1.0e-1]; non-finite     <br>
                                          ///<        values return SM_ERR_INVALID_INPUT                            <br>
    double & rdVolume                     ///< [out]: Enclosed volume                                                <br>
);

/// Total surface area: the sum of SmApiFaceComputeArea over every face, with
/// the same accuracy envelope.  Works on sheet bodies and open shells, which
/// SmApiBrepComputeMassProperties rejects.
SMAPI_EXPORT SmApiStatus SmApiBrepComputeArea(
    SmBrep*  pBrep,                       ///< [in ]: Pointer to brep (read-only)                                   <br>
    double   dRelativeAccuracy,           ///< [in ]: Relative accuracy, same envelope as SmApiFaceComputeArea;     <br>
                                          ///<        non-finite values return SM_ERR_INVALID_INPUT                 <br>
    double & rdArea                       ///< [out]: Total surface area of all faces                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepComputeMassProperties(
    const SmBrep* pBrep,                   ///< [in ]: Brep to query (closed manifold solid; read-only)              <br>
    double       dRelativeAccuracy,        ///< [in ]: Relative accuracy, clamped to [1e-4, 1e-1]; non-finite is      <br>
                                           ///<        rejected with SM_ERR_INVALID_INPUT                            <br>
    double       dDensity,                 ///< [in ]: Uniform mass density (mass = density * volume); must be a      <br>
                                           ///<        valid double >= SM_EFF_ZERO (1e-12), else                     <br>
                                           ///<        SM_ERR_INVALID_INPUT                                           <br>
    const SmPoint3d & crOrigin,            ///< [in ]: Origin the moments/products are taken about (axes parallel    <br>
                                           ///<        to world axes); pass the centroid for centroidal inertia.     <br>
                                           ///<        Must be finite, else SM_ERR_INVALID_INPUT                      <br>
    double &     rdArea,                   ///< [out]: Total surface area                                            <br>
    double &     rdVolume,                 ///< [out]: Enclosed volume                                               <br>
    double &     rdMass,                   ///< [out]: Mass = dDensity * volume                                      <br>
    SmPoint3d &  rCentroid,                ///< [out]: Mass centroid (centre of gravity) in world coordinates;       <br>
                                           ///<        independent of crOrigin                                       <br>
    SmVector3d & rMomentsOfInertia,        ///< [out]: Mass moments of inertia (Ixx, Iyy, Izz) about axes through    <br>
                                           ///<        crOrigin parallel to the world axes                           <br>
    SmVector3d & rProductsOfInertia        ///< [out]: Mass products of inertia about crOrigin as raw positive       <br>
                                           ///<        integrals (Iyz, Izx, Ixy); tensor off-diagonals are negatives <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepBoundingBox(
    const SmBrep* pBrep,                  ///< [in ]: Pointer to brep                                               <br>
    SmBoolean   bTight,                   ///< [in ]: FALSE = fast loose union of vertex/edge/face boxes;           <br>
                                          ///<        TRUE  = call CalculateTightBoundingBox (expensive)            <br>
    SmPoint3d & rMin,                     ///< [out]: Min corner of axis-aligned bounding box                       <br>
    SmPoint3d & rMax                      ///< [out]: Max corner of axis-aligned bounding box                       <br>
);

SMAPI_EXPORT SmApiStatus SmApiBrepCopy(
    const SmBrep*  pBrep,                 ///< [in ]: Source Brep (not modified)                                    <br>
    SmBrep*& rpResult                     ///< [out]: Newly allocated deep copy in the SM_API context               <br>
);

SMAPI_EXPORT SmApiStatus SmApiFaceBoundingBox(
    const SmFace* pFace,                  ///< [in ]: Pointer to face                                               <br>
    SmBoolean   bTight,                   ///< [in ]: FALSE = fast loose box; TRUE = tight (expensive)              <br>
    SmPoint3d & rMin,                     ///< [out]: Min corner of axis-aligned bounding box                       <br>
    SmPoint3d & rMax                      ///< [out]: Max corner of axis-aligned bounding box                       <br>
);

/// Single-face area.  For whole-solid area+volume+centroid+inertia, use
/// SmApiBrepComputeMassProperties (one call; also yields solid volume).
/// This area-only query skips volume and moment integration.
SMAPI_EXPORT SmApiStatus SmApiFaceComputeArea(
    const SmFace*  pFace,                 ///< [in ]: Pointer to face                                               <br>
    double   dRelativeAccuracy,           ///< [in ]: Relative accuracy in [1e-4, 1e-1]                             <br>
    double & rdArea                       ///< [out]: Surface area of the face within its trim boundary             <br>
);

SMAPI_EXPORT SmApiStatus SmApiFaceInternalPoint(
    const SmFace*     pFace,                    ///< [in ]: Pointer to face                                               <br>
    SmPoint3d & rPoint                    ///< [out]: Point on the owning surface, strictly inside the face trims    <br>
);

SMAPI_EXPORT SmApiStatus SmApiFaceClassifyUV(
    const SmFace*           pFace,              ///< [in ]: Pointer to face                                               <br>
    const SmPoint2d & crUV,               ///< [in ]: UV parameter on the parent surface to classify                <br>
    int &             rClassification     ///< [out]: SmPointClassificationType: SM_PC_FACE (inside),               <br>
                                          ///<        SM_PC_UNKNOWN (outside), SM_PC_EDGE / SM_PC_VERTEX (boundary)  <br>
);

SMAPI_EXPORT SmApiStatus SmApiFaceClosestPoint(
    SmFace*           pFace,              ///< [in ]: Pointer to face                                               <br>
    const SmPoint3d & crPoint,            ///< [in ]: Query point                                                   <br>
    SmPoint3d &       rClosestPoint,      ///< [out]: Closest point on the trimmed face                             <br>
    double &          rdDistance          ///< [out]: Distance from crPoint to rClosestPoint                        <br>
);

/// Single-face area centroid (integrates only area and area first moments). For a
/// whole-solid mass centroid, use SmApiBrepComputeMassProperties.
SMAPI_EXPORT SmApiStatus SmApiFaceComputeCentroid(
    const SmFace*     pFace,                    ///< [in ]: Pointer to face                                               <br>
    double      dRelativeAccuracy,        ///< [in ]: Relative accuracy in [1e-4, 1e-1]                             <br>
    double &    rdArea,                   ///< [out]: Surface area of the face within its trim boundary             <br>
    SmPoint3d & rCentroid                 ///< [out]: Area centroid of the trimmed face                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiEdgeBoundingBox(
    const SmEdge* pEdge,                  ///< [in ]: Pointer to edge                                               <br>
    SmBoolean   bTight,                   ///< [in ]: FALSE = fast loose box; TRUE = tight (expensive)              <br>
    SmPoint3d & rMin,                     ///< [out]: Min corner of axis-aligned bounding box                       <br>
    SmPoint3d & rMax                      ///< [out]: Max corner of axis-aligned bounding box                       <br>
);

SMAPI_EXPORT SmApiStatus SmApiEdgeComputeLength(
    const SmEdge*  pEdge,                 ///< [in ]: Pointer to edge                                               <br>
    double   dDesiredAccuracy,            ///< [in ]: Absolute accuracy of the arc-length integration               <br>
    double & rdLength                     ///< [out]: 3D arc length of the edge over its parametric interval        <br>
);

SMAPI_EXPORT SmApiStatus SmApiEdgeClosestPoint(
    SmEdge*           pEdge,              ///< [in ]: Pointer to edge                                               <br>
    const SmPoint3d & crPoint,            ///< [in ]: Query point                                                   <br>
    SmPoint3d &       rClosestPoint,      ///< [out]: Closest point on the edge's trimmed curve interval             <br>
    double &          rdParameter,        ///< [out]: Curve parameter of the closest point                          <br>
    double &          rdDistance          ///< [out]: Distance from crPoint to rClosestPoint                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiEdgeParameterRange(
    const SmEdge*  pEdge,                       ///< [in ]: Pointer to edge                                               <br>
    double & rdMin,                       ///< [out]: Low end of the edge's parametric interval on its curve        <br>
    double & rdMax                        ///< [out]: High end of the edge's parametric interval on its curve       <br>
);

SMAPI_EXPORT SmApiStatus SmApiEdgeClassifyParameter(
    const SmEdge*  pEdge,                       ///< [in ]: Pointer to edge                                               <br>
    double   dParameter,                  ///< [in ]: Curve parameter to classify                                   <br>
    int &    rClassification              ///< [out]: SmPointClassificationType: SM_PC_EDGE (interior),             <br>
                                          ///<        SM_PC_VERTEX (at start/end), SM_PC_UNKNOWN (outside interval)  <br>
);

SMAPI_EXPORT SmApiStatus SmApiEdgeTangent(
    const SmEdge*      pEdge,                   ///< [in ]: Pointer to edge                                               <br>
    double       dParameter,              ///< [in ]: Curve parameter, within the edge's interval                   <br>
    SmVector3d & rTangent                 ///< [out]: Unit tangent vector at dParameter                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiVertexGetPoint(
    const SmVertex*   pVertex,            ///< [in ]: Pointer to vertex                                             <br>
    SmPoint3d & rPoint                    ///< [out]: Vertex 3D position                                            <br>
);

#endif // __SM_API_QUERIES_H__
