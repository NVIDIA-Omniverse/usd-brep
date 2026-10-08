// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTopoTypes.h
* PURPOSE: Type file for topology subcomponent.
**********************************************************************/

#ifndef __SMTOPO_TYPES_H__
#define __SMTOPO_TYPES_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

class SmTopology;
class SmOwningTopology;
class SmRegion;
class SmShell;
class SmFace;
// Remove Composites
// class SmCFace;
class SmLoop;
class SmEdge;
// Remove Composites
// class SmCEdge;
class SmVertex;
class SmFaceuse;
class SmLoopuse;
class SmEdgeuse;
class SmVertexuse;
class SmSAGObject;
class SmAssembly;
class SmAssemblyInstance;
class SmCurveBoundedSurface;
class SmCurveClassification;
class SmLineSegClassification;
class SmTrimSrfCache;
class SmBrepCache;
class SmShape;
class SmBrepData;
class SmFilletVertexuse;
class SmFilletVertex;
class SmFilletEdgeuse;
class SmFilletEdge;
class SmFilletCorner;
class SmFilletSolver;
class SmFilletLaw;
class SmLinearFilletLaw;
class SmBSplineFilletLaw;
class SmConstantRadiusFS;
class SmConstantRadiusAssistedFS;
class SmSurfaceSurfaceFS;
class SmConstantDistanceFS;
class SmCurveBasedFS;
class SmVariableRadiusFS;
class SmFeature;
class SmFeatureLoop;
class SmProtoTopologyManager;

#define SmTopology_TYPE              (TOPO_BASE_TYPE + 1)
#define SmFace_TYPE                  (TOPO_BASE_TYPE + 2)
// Remove Composites
// #define SmCFace_TYPE                 (TOPO_BASE_TYPE + 23)
#define SmLoop_TYPE                  (TOPO_BASE_TYPE + 3)
#define SmLoopuse_TYPE               (TOPO_BASE_TYPE + 4)
#define SmEdgeuse_TYPE               (TOPO_BASE_TYPE + 5)
#define SmVertexuse_TYPE             (TOPO_BASE_TYPE + 6)
#define SmOwningTopology_TYPE        (TOPO_BASE_TYPE + 10)
#define SmBrep_TYPE                  (TOPO_BASE_TYPE + 11)
#define SmRegion_TYPE                (TOPO_BASE_TYPE + 12)
#define SmShell_TYPE                 (TOPO_BASE_TYPE + 13)
#define SmEdge_TYPE                  (TOPO_BASE_TYPE + 16)
// Remove Composites
// #define SmCEdge_TYPE                 (TOPO_BASE_TYPE + 17)
#define SmVertex_TYPE                (TOPO_BASE_TYPE + 21)
#define SmFaceuse_TYPE               (TOPO_BASE_TYPE + 22)

#define SmFilletEdgeuse_TYPE         (TOPO_BASE_TYPE + 31)  /* TOPO_BASE_TYPE:[16000] */
#define SmFilletVertexuse_TYPE       (TOPO_BASE_TYPE + 32)
#define SmFilletEdge_TYPE            (TOPO_BASE_TYPE + 33)
#define SmFilletVertex_TYPE          (TOPO_BASE_TYPE + 34)
#define SmFilletGeom_TYPE            (TOPO_BASE_TYPE + 35)
#define SmFilletCorner_TYPE          (TOPO_BASE_TYPE + 36)
#define SmFilletSolver_TYPE          (TOPO_BASE_TYPE + 37)
#define SmVariableRadiusFS_TYPE      (TOPO_BASE_TYPE + 38)
#define SmFilletLaw_TYPE             (TOPO_BASE_TYPE + 39)
#define SmLinearFilletLaw_TYPE       (TOPO_BASE_TYPE + 40)
#define SmBSplineFilletLaw_TYPE      (TOPO_BASE_TYPE + 41)

#define SmConstantRadiusFS_TYPE      (TOPO_BASE_TYPE + 42)
#define SmConstantRadiusAssistedFS_TYPE (TOPO_BASE_TYPE + 43)
#define SmSurfaceSurfaceFS_TYPE      (TOPO_BASE_TYPE + 44)
#define SmConstantDistanceFS_TYPE    (TOPO_BASE_TYPE + 45)
#define SmCurveBasedFS_TYPE          (TOPO_BASE_TYPE + 46)

#define SmSAGObject_TYPE             (TOPO_BASE_TYPE + 50)
#define SmAssembly_TYPE              (TOPO_BASE_TYPE + 55)
#define SmAssemblyInstance_TYPE      (TOPO_BASE_TYPE + 56)

#define SmFeature_TYPE               (TOPO_BASE_TYPE + 60)
#define SmFeatureLoop_TYPE           (TOPO_BASE_TYPE + 61)

#define SmProbArray_TYPE             (TOPO_BASE_TYPE + 70)
#define SmPairProbArray_TYPE         (TOPO_BASE_TYPE + 72)
#define SmMixedProbArray_TYPE        (TOPO_BASE_TYPE + 74)
#define SmTriedProbArray_TYPE        (TOPO_BASE_TYPE + 76)

#define SmObjProps_TYPE              (TOPO_BASE_TYPE + 80)
#define SmFaceProps_TYPE             (TOPO_BASE_TYPE + 82)
#define SmLoopProps_TYPE             (TOPO_BASE_TYPE + 84)
#define SmEdgeProps_TYPE             (TOPO_BASE_TYPE + 86)
#define SmVertexProps_TYPE           (TOPO_BASE_TYPE + 88)

// retired numbers - these were improperly assigned to the wrong geometry type and then moved. Don't reuse these numbers
//      SmCurveBoundedSurface_TYPE   (TOPO_BASE_TYPE + 100)  // retired number
   
#define SmBrepCache_TYPE             (TOPO_BASE_TYPE + 400)  /* TOPO_BASE_TYPE:[16000] */
#define SmShape_TYPE                 (TOPO_BASE_TYPE + 500)
#define SmBrepData_TYPE              (TOPO_BASE_TYPE + 600)
#define SmCutter_TYPE                (TOPO_BASE_TYPE + 620)
#define SmPlaneCutter_TYPE           (TOPO_BASE_TYPE + 621)
#define SmSurfaceBiLinearCutter_TYPE (TOPO_BASE_TYPE + 622)

// SmGap types                       
#define SmGap_TYPE                    (TOPO_BASE_TYPE + 640)
#define SmVertexEdgeGap_TYPE          (TOPO_BASE_TYPE + 642)
#define SmVertexFaceTrimCurveGap_TYPE (TOPO_BASE_TYPE + 644)
#define SmVertexFaceGap_TYPE          (TOPO_BASE_TYPE + 646)
#define SmEdgeEdgeGap_TYPE            (TOPO_BASE_TYPE + 647)
#define SmEdgeFaceTrimCurveGap_TYPE   (TOPO_BASE_TYPE + 648)
#define SmEdgeFaceGap_TYPE            (TOPO_BASE_TYPE + 650)

#define SmGapFunction_TYPE            (TOPO_BASE_TYPE + 660)  /* TOPO_BASE_TYPE:[16000] */
#define SmCrvPtGapFunction_TYPE       (TOPO_BASE_TYPE + 662)
#define SmCrvCrvGapFunction_TYPE      (TOPO_BASE_TYPE + 664)
#define SmCrvSrfGapFunction_TYPE      (TOPO_BASE_TYPE + 666)
#define SmSrfPtGapFunction_TYPE       (TOPO_BASE_TYPE + 668)
#define SmSrfCrvGapFunction_TYPE      (TOPO_BASE_TYPE + 670)
#define SmSrfSrfGapFunction_TYPE      (TOPO_BASE_TYPE + 672)
#define SmProtoTopologyManager_TYPE   (TOPO_BASE_TYPE + 680)

#define SmUnknown_TYPE               (TOPO_BASE_TYPE + 999)  // gwc:changed to make unique

#define SM_TOPO_TYPENAME(a) \
  (a) == SmTopology_TYPE            ? _T("SmTopology")            \
: (a) == SmFace_TYPE                ? _T("SmFace")                \
/* Remove Composites            */                                \
/* : (a) == SmCFace_TYPE              ? _T("SmCFace")  */         \
: (a) == SmLoop_TYPE                ? _T("SmLoop")                \
: (a) == SmLoopuse_TYPE             ? _T("SmLoopuse")             \
: (a) == SmEdgeuse_TYPE             ? _T("SmEdgeuse")             \
: (a) == SmVertexuse_TYPE           ? _T("SmVertexuse")           \
: (a) == SmOwningTopology_TYPE      ? _T("SmOwningTopology")      \
: (a) == SmBrep_TYPE                ? _T("SmBrep")                \
: (a) == SmRegion_TYPE              ? _T("SmRegion")              \
: (a) == SmShell_TYPE               ? _T("SmShell")               \
: (a) == SmEdge_TYPE                ? _T("SmEdge")                \
/* Remove Composites            */                                \
/* : (a) == SmCEdge_TYPE               ? _T("SmCEdge")  */        \
: (a) == SmVertex_TYPE              ? _T("SmVertex")              \
: (a) == SmFaceuse_TYPE             ? _T("SmFaceuse")             \
: (a) == SmSAGObject_TYPE           ? _T("SmSAGObject")           \
: (a) == SmAssembly_TYPE            ? _T("SmAssembly")            \
: (a) == SmAssemblyInstance_TYPE    ? _T("SmAssemblyInstance")    \
: (a) == SmCurveBoundedSurface_TYPE ? _T("SmCurveBoundedSurface") \
: (a) == SmCurveClassification_TYPE ? _T("SmCurveClass")          \
: (a) == SmCurveInterval_TYPE       ? _T("SmCurveInterval")       \
: (a) == SmPointClassification_TYPE ? _T("SmPointClass")          \
: (a) == SmProbArray_TYPE           ? _T("SmProbArray")         \
: (a) == SmPairProbArray_TYPE       ? _T("SmPairProbArray")     \
: (a) == SmMixedProbArray_TYPE      ? _T("SmMixedProbArray")    \
: (a) == SmTriedProbArray_TYPE      ? _T("SmTriedProbArray")    \
: (a) == SmBrepCache_TYPE           ? _T("SmBrepCache")           \
: (a) == SmShape_TYPE               ? _T("SmShape")               \
: (a) == SmBrepData_TYPE            ? _T("SmBrepData")            \
: (a) == SmCutter_TYPE              ? _T("SmCutter")              \
: (a) == SmPlaneCutter_TYPE         ? _T("SmPlaneCutter")         \
: (a) == SmTArray_TYPE              ? _T("SmTArray")            \
: (a) == SmGap_TYPE                    ? _T("SmGap")                  \
: (a) == SmVertexEdgeGap_TYPE          ? _T("SmVertex/Edge_Gap")        \
: (a) == SmVertexFaceTrimCurveGap_TYPE ? _T("SmVertex/UVTrimCurve_Gap") \
: (a) == SmVertexFaceGap_TYPE          ? _T("SmVertex/Face_Gap")        \
: (a) == SmEdgeFaceTrimCurveGap_TYPE   ? _T("SmEdge/UVTrimCurve_Gap")   \
: (a) == SmEdgeFaceGap_TYPE            ? _T("SmEdge/Face_Gap")          \
: (a) == SmProtoTopologyManager_TYPE   ? _T("SmProtoTopologyManager")   \
: (a) == SmUnknown_TYPE             ? _T("Unknown")             \
: _T("Not a TOPO_BASE_TYPE" )                                   

class SmPolyVertex;
class SmPolyEdge;
class SmPolyLoop;
class SmPolyFace;
class SmPolyShell;
class SmPolyRegion;
class SmPolyBrep;
class SmPolyBrepData;

// obsolete - not used
// class SmSSVertex;
// class SmSSEdge;
// class SmSSFace;
// class SmSSShell;
// class SmSSRegion;
// class SmSSBrep;
// class SmPointGraph;
// class SmSSAttr;
// class SmSSVertexAttr;
// class SmSSEdgeAttr;
// class SmSSFaceAttr;
// class SmSSExec;
// class SmSSEvaluator;

// moved to SmTypes.h -  #define POLY_BASE_TYPE       (TOPO_BASE_TYPE + 2000)
#define SmPolyBrep_TYPE       POLY_BASE_TYPE + 1
#define SmPolyRegion_TYPE     POLY_BASE_TYPE + 2
#define SmPolyShell_TYPE      POLY_BASE_TYPE + 3
#define SmCPolyFace_TYPE      POLY_BASE_TYPE + 4
#define SmPolyFace_TYPE       POLY_BASE_TYPE + 5
#define SmPolyLoop_TYPE       POLY_BASE_TYPE + 6
#define SmPolyEdge_TYPE       POLY_BASE_TYPE + 7
#define SmPolyVertex_TYPE     POLY_BASE_TYPE + 8
                              
// obsolete - not used
// #define SmSSBrep_TYPE         POLY_BASE_TYPE + 30
// #define SmSSRegion_TYPE       POLY_BASE_TYPE + 31
// #define SmSSShell_TYPE        POLY_BASE_TYPE + 32
// #define SmSSAttr_TYPE         POLY_BASE_TYPE + 33
// #define SmSSFaceAttr_TYPE     POLY_BASE_TYPE + 34
// #define SmSSEdgeAttr_TYPE     POLY_BASE_TYPE + 35
// #define SmSSVertexAttr_TYPE   POLY_BASE_TYPE + 36
// 
// #define SmPointGraph_TYPE POLY_BASE_TYPE + 52

/*******************************************************************//**
PURPOSE: This object determines the behavior of mass property 
    computations.

NOTES: 
***********************************************************************/
class SmMassPropertyBehavior
{
 public:
  double     m_dDensity;                // Default density - may be overridden by
                                        // an attribute for mass properties on individual objects.
                                        // default:[1.0]
  double     m_dFaceThickness;          // Default thickness assigned to faces
                                        // for which volumetric computation is performed as if they are a thin
                                        // solid. May be overridden by attributes on individual faces.
                                        // Note that this computation is only accurate if the thickness is
                                        // relatively small compared to the size of the face.
                                        // default:[0.0]
  double     m_dEdgeCrossSectionRadius; // Default cross section radius of
                                        // edges for which volumetric computation is performed as if they are a
                                        // solid tube.  May be overridden by attributes on individual edges.
                                        // Note that this computation is only accurate if the area is
                                        // relatively small compared to the size of the edge.
                                        // default:[0.0]
  double     m_dVertexSphereRadius;     // Default radius for vertices for which 
                                        // volumetric computation is performed as if it was a small sphere.
                                        // default:[0.0]
  ULONG      m_lFaceAreaBehavior;       // Describes how face area computations behave
                                        // 0: Default, measures area only for those faces which lie between a
                                        //     solid region and a void region.
                                        // 1: Measures each face one time
                                        // 2: Measures each face 2 times (how much paint goes on a building if
                                        //     each wall is represented by a single face)
                                        // default:[0]
  ULONG      m_lFaceVolumeBehavior;     // Describe how a face is used to compute volumes.  
                                        // 0: Default, uses faces which bound non void regions to compute the
                                        //               volume of the regions.  All other faces must either use
                                        //               an attribute or the face thickness must be non-zero for them
                                        //               to enter into the computation.
                                        // 1: Assumes all faces represent a thin shelled solid and that no volumes
                                        //               exist.  Note that a face thickness must be specified.
                                        // default:[0]

 public:
  // constructor                                         
  SmMassPropertyBehavior(double dDensity = 1.0,
                         double dFaceThickness = 0.0,
                         double dEdgeCrossSectionRadius = 0.0,
                         double dVertexSphereRadius = 0.0,
                         ULONG lFaceAreaBehavior = 0,
                         ULONG lFaceVolumeBehavior = 0) 
                       : m_dDensity(dDensity), 
                         m_dFaceThickness(dFaceThickness), 
                         m_dEdgeCrossSectionRadius(dEdgeCrossSectionRadius), 
                         m_dVertexSphereRadius(dVertexSphereRadius),
                         m_lFaceAreaBehavior(lFaceAreaBehavior), 
                         m_lFaceVolumeBehavior(lFaceVolumeBehavior) 
                       { }

} ; // end class SmMassPropertyBehavior

/*******************************************************************//**
PURPOSE: This enum defines the topology types of Edges.

NOTES: 
***********************************************************************/
enum SmEdgeTopoType 
{
    SM_ET_UNKNOWN  =  0,   // Point not yet classified
    SM_ET_WIRE     =  1,   // Wire     Edges
    SM_ET_LAMINA   =  2,   // Lamina   Edges
    SM_ET_MANIFOLD =  4,   // Manifold Edges
    SM_ET_SPINE    =  8,   // Spine    Edges
    SM_ET_ALL      = 15,   // All      Edges
};    

/*******************************************************************//**
PURPOSE: This enum determines which mass property quantities are to be computed.

NOTES: 
***********************************************************************/
enum SmMassPropertiesType {
    SM_MPT_AREA,      // compute area only
    SM_MPT_VOLUME,    // compute area and volume only
    SM_MPT_CENTROID,  // compute area, volume and centroid
    SM_MPT_MOMENTS,   // compute area, volume, centroid and moments of inertia
    SM_MPT_ALL,       // compute all properties: same as SM_MPT_MOMENTS
};


#endif // !__SMTOPO_TYPES_H__

