// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTess.h
* PURPOSE: Header file for SmTess object.
**********************************************************************/

#ifndef __SMTESS_H__
#define __SMTESS_H__

#include <SmSurfTypes.h>
#include <SmCoreTypes.h>
#include <SmCurveTypes.h>
#include <SmVector3d.h>
#include <SmTopoTypes.h>
#include <SmCurveTypes.h>
#include <SmAxis2Placement.h>
#include <SmTrimSrfCache.h>
#include <SmRelation.h>
#include <SmPoly.h>
#include <SmFace.h>
// Remove Composites
// #include <SmCEdge.h>
#include <SmVertexuse.h>
#include <SmEdgeuse.h>
#include <SmAttribute.h>

class SmCurveTessDriver;
class SmSurfaceTessDriver;
class SmTess;
class SmTessSrfCache;

struct StackData
    {
      SmTreeNode * m_pNode;
      SmPolyEdge * m_cornerEdge [4];
      SmPolyEdge * m_minMaxSplitEdgeRadialPair [2];
      StackData () { }
      StackData (SmTreeNode * pNode,
                 SmPolyEdge * const cornerEdge [4])
               : m_pNode (pNode)
               {
                 for (ULONG i = 0; i < 4; ++i) { m_cornerEdge [i] = cornerEdge[i]; }
                 for (ULONG i = 1; i < 2; ++i) { m_minMaxSplitEdgeRadialPair[i] = NULL; }
               }

      StackData (SmPolyEdge* const minMaxSplitEdgeRadialPair [2])
               : m_pNode (NULL)
               {
                 for (ULONG i = 0; i < 4; ++i) { m_cornerEdge [i] = NULL; }
                 for (ULONG i = 0; i < 2; ++i) { m_minMaxSplitEdgeRadialPair [i] = minMaxSplitEdgeRadialPair [i]; }
               }
} ;

struct SmPolyEdgeProp {
    double m_dAspectRatio1 ;
    double m_dAspectRatio2 ;
    double m_dPELengthUV ;
    double m_dPELength3d ;
} ;

SM_RELATION_TEMPLATE_PREDECLARATION(SmFace,SmTessSrfCache)

/*******************************************************************//**
PURPOSE: This object is an attribute which will contain EU and VU
    pointers back to the Tessellation Brep.

NOTES:
***********************************************************************/
class SM_EXPORT SmPolyToEUVUAttr : public SmAttribute
{
  friend class SmTess;
protected:
  SmEdgeuse *m_pEdgeuse;
  SmVertexuse *m_pVertexuse;

public:
  SmPolyToEUVUAttr
  (
    ULONG lAttributeID,
    SmAttributeBehaviorType eBehavior = SM_AB_COPY
  )
    : SmAttribute(lAttributeID,eBehavior),
      m_pEdgeuse(NULL), m_pVertexuse(NULL) {}

  virtual ~SmPolyToEUVUAttr() {}

  virtual SmAttribute * MakeCopy(const SmContext & crContext) const;

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const
  { 
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed(lAllocated) ;
    rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
    return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
  }

} ; // end class SmPolyToEUVUAttr

/*******************************************************************//**
PURPOSE: This object is the high level object which contains the
   surface tessellation subdivision cache.

NOTES: This is used to break a Surface into a set of
   polygons - used in SMLib to make rendering outputs.

   SMLib does NOT store SmTessSrfCaches in the global cache queues.
   SMLib uses these to generate polygons for rendering, when doing so,
      SmTessSrfCache caches are stored within the SmTess::m_vCache relation.
***********************************************************************/
class SM_EXPORT SmTessSrfCache : public SmTrimSrfCache
{
  friend class SmTree;
  friend class SmTess;
  friend class SmSurfaceTessDriver;


protected:
  // inherited from SmSurfaceCache
  //   SmTree                      * m_pTree                       // The surface subdivision tree
  //   const SmSurface             * m_cpSurface;                  // Surface being subdivided into a piecewise bezier surface
  //   SmExtent2d                    m_sUVDomain;                  // Surface Extent for the Cache
  //                                                               //      (SmSurfaceCache = Natural UV Domain)
  //                                                               //      (SmTrimSrfCache = Trim Surface UV Domain)
  //   SmTArray<SmContinuityType>  * m_pUContinuities;             //
  //   SmTArray<SmContinuityType>  * m_pVContinuities;             //
  //   double                        m_dChordHeightTolerance;      // max allowed control-Point dist to patch basePlane,       0.0 = ignore
  //   double                        m_dAngTolRad;                 // max allowed controlPolygon endTangent angles (radians),  0.0 = ignore
  //   double                        m_dAspectRatio3D;             // max allowed element basepolygon aspect ratio,            0.0 = ignore
  //   double                        m_dMaximumSideLength3D;       // max allowed basePolygon side length in u or v direction, 0.0 = ignore
  //   double                        m_dMinimumSideLength3D;       // polygon side length stopping size for subdivision,       0.0 = ignore, 0.0 = ignore
  //                                                               //   note: the smallest patch sizes might run to about 1/2 of m_dMinimumSideLength3D.
  //   double                        m_dMinimumSideLengthRatioUV;  // Subdivison stops once node get smaller than this size, 0.0 = ignore

  SmFace              * m_pFace;                               // ptr to Face with TrimSurface to tessellate
  SmPolyBrep          * mTS_pPolyBrep;                         // Poly Brep tessellation of the TrimmedSurface (the polygons)
                                                               //  This is a UV tessellation

  SmSurfaceTessDriver * m_pSurfaceTessDriver;                  // left over from old style caching to preserve user interface.
    // contains:                                               // Could just be a boolean now, except its used to hold tessellation tolerances.
    //   SmSurfaceTessDriver::m_vViewVector;                   // NULL    = tessellate using standard SmSurfaceCache tessellation tolerances
    //   SmSurfaceTessDriver::m_dSilhouetteChordHeight;        // notNULL = tessellate subdivision nodes on the silhouette boundary to
    //   SmSurfaceTessDriver::m_dSilhouetteAngleToleranceDeg;  //           tighter tessellation tolerances

  // gwc: not used
  // SmBSplineSurface    * m_pCompositeBezier;

public:
  // constructor
  SmTessSrfCache
  (
    SmFace              * pFace,                 ///< [in ]: ptr to Face with TrimSurface to tessellate                                              <br>
    SmPolyBrep          * pPolyBrep,             ///< [in ]: PolyBrep tessellation of the TrimmedSurface (the polygons)                              <br>
    SmSurfaceTessDriver * pSurfaceTessDriver,    ///< [in ]: contains: m_vViewVector, m_dSilhouetteChordHeight, m_dSilhouetteAngleToleranceDeg       <br>
    double                dChordHeightTolerance, ///< [in ]: max allowed control-Point dist to patch basePlane,       0.0 = ignore                   <br>
    double                dAngleTolerance,       ///< [in ]: max allowed controlPolygon endTangent angles (radians),  0.0 = ignore                   <br>
    double                dMaxEdgeLength3D,      ///< [in ]: max allowed basePolygon side length in u or v direction, 0.0 = ignore                   <br>
    double                dMinEdgeLength3D,      ///< [in ]: polygon side length stopping size for subdivision,       0.0 = ignore                   <br>
    double                dMinEdgeLengthRatioUV, ///< [in ]: Subdivison stops once node get smaller than this size,   0.0 = ignore                   <br>
    double                dMaxAspectRatio        ///< [in ]: max allowed element basepolygon aspect ratio,            0.0 = ignore                   <br>
  );

  virtual ~SmTessSrfCache();

  virtual SmStatus BuildTree(SmMemBlockMgr *pOptBezierBlock=NULL);

  SmStatus BuildTreeWithSubdivision(SmBSplineSurface *pBSS);


  SmFace     * GetFace()        const { return m_pFace; }
  SmPolyBrep * GetTS_PolyBrep() const { return mTS_pPolyBrep ; }

  SmStatus ImplantUVSegment(SmPolyEdge* pSegment);

  SmStatus RefineMeshBoundaries
  (
    ULONG lEdgeCountThreshold,          /// [in ]: number of PolyEdges in node before split
    ULONG lEdgeThresholdGap,            /// [in ]: threshold for gap in PolyEdge distributions between halves of boundary node
    double dNeighborThreshold,          /// [in ]: threshold for subdivision of neighbors to divided boundary nodes
    double dNeighborThresholdGrowth,    /// [in ]: growth rate for threshold for subdivision of neighbors to divided boundary nodes   
    SmBoolean bIgnoreBoundaryNeighbors  /// [in ]: Determines whether we reevaluate boundary nodes which are neighbors of
                                        //      subdivided interior nodes.
  );

  SmStatus FindUVNodesOfClass
  (
    const SmExtent2d      & crUVDomain,         ///< [in ]: culling domain                                                                      <br>
    const SmNodeClassType * cpOptTypeToFind,    ///< [in ]: opt culling nodetype, NULL to ignore                                                <br>
    SmTArray<SmTreeNode*> & rNodes              ///< [out]: List of m_pTree nodes intersecting crUVDomain and optionally of type cpTypeToFind   <br>
  ) const;                                          

  // label each subdivision node as in/out/on face boundaries.
  //   This used to be based upon 2d raycasting but now uses loop containment.
  //   As such it can be called with m_bPointTestEnabled on or off.
  SmStatus MarkFaceContainment();

  virtual SmStatus TestAgainstTolerances
  (
    SmTreeNode      * pNode,                       ///< [in ]: node to test                                                     <br>
    SmBoolean       & rbNeedsSubdivision,          ///< [out]: TRUE = patch fails some tessellation test                        <br>
    SmSurfParamType & reSubdivisionDirection       ///< [out]: suggested direction to split bad elements SM_SP_U or SM_SP_V     <br>
  );

  // Draw mTS_pPolyBrep Tessellation polygon boundaries in 2d on plane or projected through the owner surface into 3d space
  SmDisplayList * DrawTessellation2D
  (
    SmPlane       * pOptOutPlane=NULL,      ///< [in ]: Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                  <br>
    SmGfxArraySet * pOptGfxSet=NULL         ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.    <br>
  ) const;

  SmDisplayList * DrawTessellation3D(SmGfxArraySet * pOptGfxSet=NULL) const; ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.  <br>
                                                                             ///<         : NULL to ignore. default:[NULL]                                                <br>
  // inherited from SmSurfaceCache
  // // Draw UV Subdivision boundares in z=0 plane
  // SmDisplayList * DrawSubdivision2D
  //   (SmBoolean       bDrawGeometry=FALSE,              ///< [in ]: TRUE= Also draw Face->UVTrimCurves and UVVertexPts on plane                      <br>
  //    SmBoolean       bOnlyDrawNodesWithVertices=FALSE, ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face vertices                     <br>
  //    SmBoolean       bOnlyDrawNodesWithEdges=FALSE,    ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face edges                        <br>
  //    SmPlane       * pOptOutPlane=NULL,                ///< [in ]: Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                       <br>
  //    SmGfxArraySet * pOptGfxSet=NULL) const;           ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.  <br>
  //
  // // Draw Subdivision block BBoxes in 3d
  // SmDisplayList * DrawSubdivision3D
  //   (SmBoolean       bOnlyDrawNodesWithVertices=FALSE, ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face vertices                    <br>
  //    SmBoolean       bOnlyDrawNodesWithEdges=FALSE,    ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face edges                       <br>
  //    SmGfxArraySet * pOptGfxSet=NULL) const;           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.        <br>

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmTessSrfCache,SmTrimSrfCache,SmTessSrfCache_TYPE);

} ; // end class SmTessSrfCache

/*******************************************************************//**
PURPOSE: Object which controls how curve is tessellated.  May be
    subclassed to provide different kinds of controls and tessellation
    algorithms.  For example one possible subclass would be to tessellate
    based on some viewing information such as the eye point and pixels
    on the screen.  In most situations the virtual methods Initialize
    and TestStep are the ones which will need to be implemented on
    the users new class.

NOTES: Values of ZERO for the tolerances will negate the use
    of that particular tolerance.

Kinds of Tessellation:

  The method SmCurveTessDriver::TessellateCurve() does the actual work.
  It supports several different kinds of tessellation as described next.

    if(m_bEvalBasedTessellation == FALSE)
      {
        SmCurve::TessellateByBisection() - Generate a set of curve segments
          that pass a set of geometry tests: not too long,
                                             chord height tolerance,
                                             end tangents don't exceed angle tol.
          Segments that fail are iteratively split in half.
          Segments that succeed are extended until they fail, then the last largest
          good segment is added to the output.
      }
    if(   m_bEvalBasedTessellation == FALSE
       && Curve is a BSpline
       && TessellateByBisection fails)
      {
        SmCurveCache::Tessellate() - decompose each BSpline into its natural spans.
           Iteratively split those spans in half until each segment passes
           the geometry tests.  No lengthening of segments is tried.
      }
    if(   Curve is Degenerate)
      {
        return single tessellation point.
      }
    if(   Curve is a BSpline
       && m_dMaxDist3dBetweenPts > 0.0)
      {
        SmBSplineCurve::EquallySpacedPoints()
          find a set of equally spaced points along the curve in Image Space.
      }
    try SmCurve::TessellateByBisection() one more time and quit.
    else
      {
        Iterate to find largest good segments one at a time from beginning to end.
        Geometry tests include: points near m_dMaxDist3dBetweenPts spacing
                                within m_dChordHeight tolerance,
                                angle between endpoint tangents within m_dAngTolDeg,
                                ratio (parametric tangent * StepSize)/(3d Dist) is close to one,
                                  (guarding against unexposed spikes in the shape.)

      }

***********************************************************************/
class SM_EXPORT SmCurveTessDriver
{
  friend class SmTess;
private:
  SmBoolean m_bEvalBasedTessellation; // FALSE = SmCurve::TessellateByBisection()
                                      //          if that fails - tessellate by SmCurveCache spatial decomposition
                                      // TRUE  = SmBSplineCurve::EquallySpacedPoints()
                                      // default:[FALSE]

  ULONG     m_lMinSegNumber;          // Min number of segments in tess polygon,   0=ignore
  double    m_dChordHeight;           // Max dist3D between geom and tess segment, 0=ignore
  double    m_dAngTolDeg;             // Max angDeg between tess tangents,         0=ignore
  double    m_dMaxDist3dBetweenPts;   // Max dist3D between tess pts,              0=ignore
  double    m_dMinParamRatio;         // Min SegParamLength/DomainLength ratio,    0=ignore

public:
  SmCurveTessDriver
  (
    ULONG     lMinSegNumber        = 0,
    double    dChordHeight         = 0.0,
    double    dAngTolDeg           = 8.0,
    double    dMaxDist3DBetweenPts = 0.0,
    double    dMinParamRatio       = 0.00001,
    SmBoolean bEvaluatorBasedTessellation = FALSE
  )
  : m_bEvalBasedTessellation(bEvaluatorBasedTessellation),
    m_lMinSegNumber         (lMinSegNumber),
    m_dChordHeight          (dChordHeight),
    m_dAngTolDeg            (dAngTolDeg),
    m_dMaxDist3dBetweenPts  (dMaxDist3DBetweenPts),
    m_dMinParamRatio        (dMinParamRatio)
  {}

  virtual ~SmCurveTessDriver() {}

 virtual SmStatus Initialize
 (
   const SmCurve    & crCurve,                  // NotUsed: in :
   const SmExtent1d & crInterval,               // in :
   SmVector3d         aInitialPntAndDerivs[3],  // in :
   double           & rdFirstStepEstimate,      // out:
   double           & rdMinStepSize,            // out:
   double           & rdMaxStepSize             // out:
 );

 virtual SmStatus TestStep
 (
   double                     dCurrentParameter,  // NotUsed: in :
   double                     dParamStepSize,     // in :
   SmVector3d                 sCurPVV[3],         // in :
   SmVector3d                 sNextPVV[3],        // in :
   SmTessStepTestResultType & reResult            // out:
 );

 virtual SmStatus TessellateCurve
 (
   const SmCurve       & crCurve,          ///< [in ]: target curve                                                 <br>
   const SmExtent1d    & crInterval,       ///< [in ]: target interval                                              <br>
   SmTArray<double>    * pParameters,      ///< [out]: tessellation parameters                                      <br>
   SmTArray<SmPoint3d> * pPoints = NULL    ///< [out]: associated tessellation points, includes curve endPoints     <br>
 );

} ; // end class SmCurveTessDriver

/*******************************************************************//**
PURPOSE: Object which controls how surfaces are subdivided during
    tessellation.  May be subclassed to provide different kinds of controls.

NOTES:
***********************************************************************/
class SM_EXPORT SmSurfaceTessDriver
{
  friend class SmTess;
protected:
  SmBoolean m_bEvalBasedTessellation; //

  double    m_dChordHeight;           // max allowed Element ControlPoint to BasePlane3d dist3d,      0.0 = ignore
  double    m_dAngTolDeg;             // max allowed Element ControlPolygon turning ang3d (radians),  0.0 = ignore
  double    m_dMaxAspectRatio;        // max allowed Element BasePolygon aspect ratio3d,              0.0 = ignore
  double    m_dMaxEdgeLength3D;       // max allowed Element BasePolygon side length3d,               0.0 = ignore
  double    m_dMinEdgeLength3D;       // min allowed Element BasePolygon side length3d,               0.0 = ignore, 0.0 = ignore
  double    m_dMinEdgeLengthRatioUV;  // min allowed Element (side lengthUV/origUVDomain.Size) ratio, 0.0 = ignore
                                      //   note: smallest elements run about 1/2 of m_dMinimumSideLength3D when used.
                                      //   note: smallest elements run about 1/2 of m_dMinimumSideLengthRatioUV when used.
public:
  SmSurfaceTessDriver
  (
    double dChordHeight          = 0.0,
    double dAngTolDeg            = 25.0,
    double dMaxEdgeLength3D      = 0.0,
    double dMinEdgeLength3D      = 0.0,
    double dMinEdgeLengthRatioUV = 0.001,
    double dMaxAspectRatio       = 0.0
  )
  : m_bEvalBasedTessellation(FALSE),
    m_dChordHeight          (dChordHeight),
    m_dAngTolDeg            (dAngTolDeg),
    m_dMaxAspectRatio       (dMaxAspectRatio),
    m_dMaxEdgeLength3D      (dMaxEdgeLength3D),
    m_dMinEdgeLength3D      (dMinEdgeLength3D),
    m_dMinEdgeLengthRatioUV (dMinEdgeLengthRatioUV)
  { }

  virtual ~SmSurfaceTessDriver() { }
  virtual  SM_TYPE   GetType()           const { return SmSurfaceTessDriver_TYPE; }

 //      virtual SmStatus TestAgainstTolerances(SmTessSrfCache  * pCache,
 //                                             SmTree          * pTree,
 //                                             SmTreeNode      * pNode,
 //                                             SmBoolean       & rbNeedsSubdivision,
 //                                             SmSurfParamType & reSubdivisionDirection);

} ; // end class SmSurfaceTessDriver

/*******************************************************************//**
PURPOSE: This is a new class which enables us to build a view
         based tessellation.

NOTES: All surface nodes within a Surface subdivision tree are subdivided
       to the tolerances stored in the SmSurfaceCache based on tests
       in SmSurfaceCache::TestAgainstTolerances.

       Surface nodes that are part of the silhouette boundary
       for this m_vViewVector, are further subdivided to the tighter
       tolerances stored in this SmViewBasedTessDriver.
***********************************************************************/
class SM_EXPORT SmViewBasedTessDriver : public SmSurfaceTessDriver
{
private:
  SmVector3d m_vViewVector;                   // define view orientation for tessellation
  double     m_dSilhouetteChordHeight;        // tessellation tolerance for subdivision nodes on silhouette boundaries
                                              //   - assumed to be tighter than the normal SmSurfaceCache::m_dChordHeightTolerance
  double     m_dSilhouetteAngleToleranceDeg;  // tessellation tolerance for subdivision nodes on silhouette boundaries
                                              //   - assumed to be tighter than the normal SmSurfaceCache::m_dAngTolRad
public:
  SmViewBasedTessDriver
  (
    const SmVector3d & crViewVector,
    double dSilhouetteChordHeight       = 0.0,
    double dSilhouetteAngleToleranceDeg = 5.0,
    double dChordHeight                 = 0.0,
    double dAngTolDeg                   = 20.0,
    double dMaxEdgeLength3D             = 0.0,
    double dMinEdgeLength3D             = 0.0,
    double dMinEdgeLengthRatioUV        = 0.001,
    double dMaxAspectRatio              = 0.0
  )
    : SmSurfaceTessDriver(dChordHeight,
                          dAngTolDeg,
                          dMaxEdgeLength3D,
                          dMinEdgeLength3D,
                          dMinEdgeLengthRatioUV,
                          dMaxAspectRatio),
      m_vViewVector                 (crViewVector),
      m_dSilhouetteChordHeight      (dSilhouetteChordHeight),
      m_dSilhouetteAngleToleranceDeg(dSilhouetteAngleToleranceDeg)
    { }

  virtual ~SmViewBasedTessDriver() { }
  virtual  SM_TYPE   GetType()           const { return SmViewBasedTessDriver_TYPE; }

  const SmVector3d &GetViewVector()               const { return m_vViewVector ; }
  double            GetSilhouetteChordHeight()    const { return m_dSilhouetteChordHeight ; }
  double            GetSilhouetteAngleTolerance() const { return m_dSilhouetteAngleToleranceDeg ; }

  //        virtual SmStatus TestAgainstTolerances(SmTessSrfCache  * pCache,
  //                                               SmTree          * pTree,
  //                                               SmTreeNode      * pNode,
  //                                               SmBoolean       & rbNeedsSubdivision,
  //                                               SmSurfParamType & reSubdivisionDirection);

} ; // end class SmViewBasedTessDriver

/*******************************************************************//**
PURPOSE: This data structure controls how interior vertices are
    smoothed after triangulation.  It seaks to smooth out the ratio
    of polygon areas.

NOTES: Values control SmTess::SmoothPolygons() method
***********************************************************************/
class SM_EXPORT SmSmoothingData
{
public:
  ULONG              m_lSmoothingPasses;     // Number of smoothing iterations per SmTess::SmoothPolygons() call.
                                             // default:[0] - no smoothing done

  double             m_dSmoothingStepSize;   // control how far vertices are moved towards centroid of vertices of all faces attached to vertex
                                             //   examples: 0.0 - no movement of vertices to improve (a silly value)
                                             //             0.5 - move 1/2 way toward centroid each iteration (a little damping stablizes the routine)
                                             //             1.0 - move to average of surrounding vertices each iteration
                                             // default:[0.0] - don't move vertices when smoothing

  double             m_dMinSmoothingRatio;   // only used when SmTess::m_bAdvancingFront == FALSE,
                                             //   stop smoothing a vertex location once its Max/Min attached
                                             //   triangle area ratio < m_dMinSmoothingRation.
                                             //     examples: 1.0 or less - smooth every vertex no matter what
                                             //               2.0-4.0 - smooth lots of things
                                             //               10.0 - smooth only areas with really small polygons
                                             // default:[1.0] - no damping, move to vertex towards centroid every pass

  ULONG              m_lSmoothingTechnique;  // 1 - move vertex towards the centroid of adjacent vertices
                                             // no other techniques yet implemented
                                             // default:[1] - only implemented technique

  // constructor - default arguments turn off smoothing.  Change SmoothingPasses and SmoothingStepSize values
  //               to force the system to smooth meshes with calls to SmTess::SmoothPolygons()
  SmSmoothingData()
   : m_lSmoothingPasses(0),
     m_dSmoothingStepSize(0.0),
     m_dMinSmoothingRatio(1.0),
     m_lSmoothingTechnique(1)
   { }

} ; // end class SmSmoothingData

/*******************************************************************//**
PURPOSE: The Tess (Tessellation) object provides the
    ability to tessellate a brep object.

NOTES:
***********************************************************************/
class SM_EXPORT SmTess
{
private:
  const SmContext & m_crContext ;        // context for new object construction

  SmBoolean         m_bAdvancingFront ;  // TRUE = in AddQuadBoundaries calls TriangulateFaceAF(), many subdivision side effects
                                         // FALSE= in AddQuadBoundaries calls TriangulateFace(), many subdivision side effects
                                         // default:[FALSE]

  SmSmoothingData   m_vSmoothingData ;   // control parameters for SmTess::SmoothPolygons() and passed to SmPolyBrep::SmoothPolygons()
  // ULONG             m_lTriangulateFaceLevel ;  // recursion depth counter in SmTess::TriangulateFace (recursion stops at level == 15).

  SmBrep              * m_pTessBrep ;    // Brep to tessellate - modified in place, deleted when SmTess is deleted
                                         //   its edges are split, its faces are divided in place until
                                         //   this brep is ready for a compatible tessellation.
  SmCurveTessDriver   & m_rCurveTess ;   // Control how curves are divided. Contains:
                                         //   SmBoolean m_bEvalBasedTessellation;       // FALSE = SmCurve::TessellateByBisection()
                                         //                                             //          if that fails - tessellate by SmCurveCache spatial decomposition
                                         //                                             // TRUE  = SmBSplineCurve::EquallySpacedPoints()
                                         //                                             // default:[FALSE]
                                         //
                                         //   ULONG  m_lMinSegNumber;                   // min number of segments in tess polygon,     0=ignore
                                         //   double m_dChordHeight;                    // max distance between geom and tess segment, 0=ignore
                                         //   double m_dAngTolDeg;                      // max angle between tess tangents,            0=ignore
                                         //   double m_dMaxDist3dBetweenPts;            // max distance between tess pts,              0=ignore
                                         //   double m_dMinParamRatio;                  // Limits the smallness of the stepsize
                                         //
  SmSurfaceTessDriver & m_rSurfaceTess ; // Control how surfaces are divided. Contains:
                                         //   SmBoolean m_bEvalBasedTessellation;       //
                                         //   double    m_dChordHeight;                 // max distance between geom and tess segment, 0=ignore
                                         //   double    m_dAngTolDeg;                   // max angle between tess tangents,            0=ignore
                                         //   double    m_dMaxEdgeLength3D;             // Maximum length of polygon edge in 3D,       0=ignore
                                         //   double    m_dMinEdgeLength3D;             // Minimum length of a polygon edge in 3D ,    0=ignore
                                         //   double    m_dMinEdgeLengthRatioUV;        // Minimu size in u or v of a polygon edge as a
                                         //                                             // fraction of the domain size.
                                         //                                             // of the surface relative to the UV size.     0=ignore
                                         //   double    m_dMaxAspectRatio;              // Max subdivision rectangle ratio size,
                                         //                                             // AspectRatioS = SizeOfLargest/sizeOfSmallest.0=ignore
  SmRelation<SmFace,
             SmTessSrfCache> m_vCache ;  // map m_pTessBrep->faces to m_pTessBrep->face->Surface->SmTessSrfCache caches.
                                         //   note: Each SmTessSrfCache cache holds the UV polygons of that face's tessellation.
                                         //         Fetch the polygon model of a face with,
                                         //           m_vCache->GetSecond(pFace)->mTS_pPolyBrep.
                                         //         Use the PolyBrep vertices as UV points to evaluate
                                         //         tessellation points from the pFace->Surface().

  SmPolyBrep               * m_p3DPolyBrep ;        // built by OutputPolygons() when
                                                    //    SmPolygonOutputCallback::m_eOutputType == SM_PO_CREATE_POLYBREP
  // gwc: obsolete
  //  SmMapPtrToPtr            * m_pBrepToPolyBrepMap ; // scratch for OutputToPolyBrep
  SmBoolean                  m_bCheckLicense ;      //
  SmTessAlgorithmType        m_eTessAlgorithm;      // default - SM_TA_FASTER_TESSELLATION
                                                    //        or SM_TA_FEWER_POLYGONS;
public:

  // constructor
  SmTess(const SmContext     & crContext,
         SmCurveTessDriver   & rCurveTess,
         SmSurfaceTessDriver & rSurfaceTess);

  // destructor
 ~SmTess();

  SmStatus     AddQuadBoundaries        (SmFace *pFace);

  SmStatus     BuildLaminaLoop          
  (
    SmTArray<SmPolyEdge*> & rPolyEdgesOfLoop,     ///< [in,out]: Input edges of loops - replaced by new edges   <br>
    SmPolyLoop           *& rpNewPolyLoop         ///< [out]: NewLoop                                           <br>
  ) const;                                                                                                  

  SmStatus     CheckPolygonNesting      
  (
    SmPolyFace * pPolygon,                ///< [in ]: Polyface to check                                                          <br>
    SmBoolean  & rbLoopRefinementDone     ///< [out]: TRUE = Edges were split when Points in one inner loop were                 <br>
                                          ///<      : found to be outside the outer loop or inside one of the other inner loops. <br>
  );

  SmStatus     CheckAgainstSurfaceCache 
  (
    SmTessSrfCache  * pTessCache,               ///< [in ]: Cache with subdivision tree to interogate                          <br>
    const SmPoint3d & crStartPoint,             ///< [in ]: start of UV segment to review                                      <br>
    const SmPoint3d & crEndPoint,               ///< [in ]: end of UV segment to review                                        <br>
    double            dSubdivisionRatio,        ///< [in ]: allowed size variation before segment needs split                  <br>
    SmBoolean         bIsInteriorEdge,          ///< [in ]: TRUE = interior edge - don't check 3d lengths                      <br>
                                                ///<      : FALSE= exterior edge - do check 3d lengths                         <br>
    ULONG           & rlNumberSubdivisions,     ///< [out]: number of splits Start/End UV segment needs                        <br>
                                                ///<      : so segment sizes are about the smallest xsecting                   <br>
                                                ///<      : cache subdivision tree leaf node size.                             <br>
    SmBoolean         bFirstSplitOnly = FALSE   ///< [in ]: for optimization only: TRUE = return after finding the first split <br>
                                                ///<      : FALSE = Check all nodes for max Split, quit when splitCnt > 100.   <br>
                                                ///<      : default:[FALSE]                                                    <br>
  )  const ;

  SmStatus     CheckPolygonIntersections 
  (
    SmPolyFace * pPolygon,                      ///< [in ]: PolyFace to check                                            <br>
    SmBoolean  & rbRefinementDone               ///< [out]: TRUE = PolyFace modified when intersecting edges are found.  <br>
  );

  SmStatus     ClampAndSplitEdge         
  (
    SmEdge           * pEdge,                   ///< [in ]: edge to be split       <br>
    SmTArray<double> & rParameters              ///< [in ]: where to split edge    <br>
  );                                                                          

  SmStatus CreateFaceTessCache(SmFace* pFace);

  SmStatus CreateFaceTessCache(SmFace* pFace, SmTessSrfCache*& pSC);

  SmStatus     SplitFaceEdgesToUVSize    (SmFace * pFace);  // renamed from CreateFaceUVSegments()

  SmStatus     FindInsideOutTopoVertices 
  (
    const SmPolyEdge        * cpSeg1,
    const SmPolyEdge        *  cpSeg2,
    SmTArray<SmPolyVertex*> & rSeg1BadVerts,
    SmTArray<SmPolyVertex*> & rSeg2BadVerts
  ) const;

  SmStatus     FindIsoLineSplits         
  (
    SmPolyEdge       * pEdge,               ///< NotUsed: [in ]:                                                    <br>
    SmTessSrfCache   * pSurfaceCache,       ///< [in ]:                                                    <br>
    const SmPoint3d  & crLinePoint,         ///< [in ]: The line point and vector define the entire        <br>
                                            ///<      : UV curve to be split.  The interval is just        <br>
                                            ///<      : the mapping between the 0-1 parameterization       <br>
                                            ///<      : of the line and the actual parameterization        <br>
                                            ///<      : of the UV curve.                                   <br>
    const SmVector3d & crLineVector,        ///< [in ]:                                                    <br>
    const SmExtent1d & crLineInterval,      ///< [in ]:                                                    <br>
    SmTArray<double> & rTSplits             ///< [out]:                                                    <br>
  ) const;

  SmStatus     GetOrCreatePolyVertex     
  (
    const SmPolyVertex         * pVertexIn2D,           ///< [in ]: PolyVertex to map to m_p3DPolyBrep PolyVertices                                  <br>
    const SmTArray<SmPoint3d>  & cr3DPoints,            ///< [in ]: assoc 3dPoint    in cr3DPoints    [pVertexIn2D->m_lIndexValue]                   <br>
    const SmTArray<SmPoint2d>  & crUVPoints,            ///< [in ]: assoc UVPoint    in crUVPoints    [pVertexIn2D->m_lIndexValue]                   <br>
    const SmTArray<SmVector3d> & cr3DNormals,           ///< [in ]: assoc 3dNormal   in cr3DNormals   [pVertexIn2D->m_lIndexValue]                   <br>
    const SmTArray<SmVertex*>  & crBrepVertices,        ///< NotUsed: [in ]: assoc BrepVertex in crBrepVertices[pVertexIn2D->m_lIndexValue]                   <br>
    SmPolyVertex              *& rpNew3DVertex,         ///< [out]: PolyVertex mapped to pVertexIn2D->BrepVertex in m_pBrepToPolyBrepMap             <br>
    SmPolyVertAuxData         *& rpNewVertexAux         ///< [out]: associated Aux Data (UVPnt & Normal - to be attached to appropriate SmPolyEdge)  <br>
  );

  SmPolyBrep     * Get3DPolyBrep         ()                             { return m_p3DPolyBrep ; }
  SmTessSrfCache * GetTessSrfCacheOfFace (SmFace * pFace) const         { return m_vCache.GetSecond(pFace) ; }
  SmPolyBrep     * GetTessBrepOfFace     (SmFace * pFace) const         { SmTessSrfCache *pSC = m_vCache.GetSecond(pFace);
                                                                          return( pSC ? pSC->GetTS_PolyBrep() : NULL) ;
                                                                        }

  static SmPoint3d Get3DPoint            
  (
    const SmSurface * cpSurface,  // rtn: cpSurface->EvaluatePoint(crUVPoint)
    const SmPoint3d & crUVPoint
  );

  void         Set3DPolyBrep             (SmPolyBrep *p3DPolyBrep)   { m_p3DPolyBrep = p3DPolyBrep; }

  SmStatus     IsoEdgeSubdivide          (SmEdge *pEdgeToSubdivide);

  SmBoolean    IsQuadSplittingEdge       
  (
    SmPolyEdge  * pEdge,                  ///< [in ]: target PolyEdge    <br>
    SmPolyEdge *& rpBottomEdge,           ///< [out]:                    <br>
    SmPolyEdge *& rpRightEdge,            ///< [out]:                    <br>
    SmPolyEdge *& rpTopEdge,              ///< [out]:                    <br>
    SmPolyEdge *& rpLeftEdge              ///< [out]:                    <br>
  ) const;

  SmStatus     MakePolygon               
  (
    SmFace      * pFace,                  ///< [in ]: face to approximate as a polygon                                                  <br>
    double        dTolerance,             ///< [in ]: 3d tol used for creating Poly topo                                                <br>
    SmPolyBrep *& rpPolyBrep,             ///< [out]: UV PolyBrep containing                                                            <br>
                                          ///<      : 1 SmPolyFace per simple outer pFace->Loop                                         <br>
                                          ///<      :     (complex pFace->Loops are broken into simple Loops yielding a few PolyFaces)  <br>
                                          ///<      : 1 inner SmPolyLoop for each inner pFace->Loop,                                    <br>
                                          ///<      : 1 SmPolyEdge for each pFace->Loop->Edge,                                          <br>
                                          ///<      : 1 SmPolyVertex for each pFace->Loop->Vertex.                                      <br>
                                          ///<      : No Inner PolyVerts or PolyEdges at this time.                                     <br>
                                          ///<      : PolyLoops not broken up into triangles at this time.                              <br>
    SmPolyFace *& rpPolyFace              ///< [out]: UV PolyFace polygon approximation to input pFace                                  <br>
  );

  // user call: Output tessellation polygons for drawing, printing,
  //            or making a polyBrep after the polygons have been
  //            built by a call to DoTessellation().

  // helper method: output one PFace->Surface's (SmTessSrfCache)pSC->mTS_pPolyBrep pPolyEdges as polylines through rPolygonOutput class
  SmStatus     OutputFaceWireframe       
  (
    SmFace                  * pFace,            ///< [in ]:                                                    <br>
    SmPolygonOutputCallback & rPolygonOutput,   ///< [in ]: Chooses how and where to output the polygons       <br>
    SmGfxArraySet           * pOptGfxSet=NULL   ///<      :note: increments unlocked mark value                <br>
  ); 

  // helper method: duplicate one PFace->Surface's (SmTessSrfCache)pSC->mTS_pPolyBrep->pPolyTopology objs in m_p3DPolyBrep.
  //   1. m_p3DPolyBrep->m_bOKBackPtrs == TRUE. During tessellation back ptrs are current.
  //      After, back ptrs may become stale
  //   2. Good for aggregating several pFace tessellations into one m_p3DPolyBrep SmPolyBrep.
  //   3. Called by OutputFacePolygons(pFace, rPolygonOutput) when rPolygonOutput->m_eOutputType == SM_PO_CREATE_POLYBREP
  //   m_pTessBrep         = not modified.
  //   m_vCache            = not modified
  //   pSC->mTS_pPolyBrep  = not modified
  //   m_p3DPolyBrep       = linear approx of m_pTessBrep with one SmPolyEdge for every m_pTessBrep->Edge
  //                         and many SmPolyFaces for every m_pTessBrep->Face
  SmStatus     OutputToPolyBrep          
  (
    SmFace                     * pFace,           ///< [in ]: target face                                                                                <br>
    SmPolygonOutputCallback    & rPolygonOutput,  ///< NotUsed: [in ]: Chooses how and where to output the polygons                                               <br>
    SmPolyShell               ** ppOptPolyShell,  ///< [in ]: NULL to ignore, default:[NULL], else                                                       <br>
                                                  ///<      :  ppOptPolyShell NULL    = every SmPolyFace gets its own SmPolyShell and SmPolyRegion       <br>
                                                  ///<      : *ppOptPolyShell NotNULL = owner of any new SmPolyFaces constructed                         <br>
                                                  ///<      : *ppOptPolyShell NULL    = create and save ptr to new PolyShell for New SmPolyFaces of pFace<br>
    const SmTArray<SmPoint3d>  & cr3DPoints,      ///< [in ]: 3dPoint list                                                                               <br>
    const SmTArray<SmPoint2d>  & crUVPoints,      ///< [in ]: associated UVPoints                                                                        <br>
    const SmTArray<SmVector3d> & cr3DNormals,     ///< [in ]: associated 3dNormals                                                                       <br>
    const SmTArray<SmVertex*>  & crBrepVertices   ///< [in ]: associated BrepVertex or NULL=none                                                         <br>
  );

  // user call: Build and store the polygons of a target Brep tessellation
  //  o. the pBrepUsedForTessellation Brep is stored in m_pTessBrep.
  //  o. each m_pTessBrep edge may be split as needed for compatible meshes.
  //  o. each m_pTessBrepface is tessellated.
  //    - a SmSrfTessCache is built for each face and stored in m_vCache which
  //         contains SmSrfTessCache::mTS_pPolyBrep which is a UVDomain
  //         tessellation of the Face->Surface.
  //  o. The polygons can be fetched through sTess.OutputPolygons().
  SmStatus      DoTessellation
  (
    SmBrep    * pBrepUsedForTessellation,       ///< [in ]: Brep to tessellate. Brep is saved in SmTess, then modified       <br>
                                                ///<      : with split edges, and then deleted when SmTess is deleted.       <br>
                                                ///<      : Typically this is a copy of the Brep to tessellate.              <br>
    SmBoolean & rbSomeFacesFailedToTessellate   ///< [out]: TRUE = some faces failed to tessellate                           <br>
  );

  // Tessellate Phase 1 - tessellate all Brep->Edges preparing for water tight face tessellations in Phase 2.
  //   m_pTessBrep       = orig target Brep modified with edge splits.
  //   m_vCache          = RelatePairs<pFace,pSC> pSC = SmTessSrfCache for pFace->Surface
  //   pSC->mTS_PolyBrep = m_pTessBrep linear approximation for each m_pTessBrep->PFace
  //                       with 1 PolyLoop per Loop, and 1 PolyEdge per Edge.
  //   m_p3DPolyBrep     = NULL.
  SmStatus      Phase1SetupBrep
  (
    SmBrep            * pBrepUsedForTessellation,       ///< [in ]: Brep to tessellate. Brep is saved in SmTess, then modified     <br>
                                                        ///<      : with split edges, and then deleted when SmTess is deleted.     <br>
                                                        ///<      : Typically this is a copy of the Brep to tessellate.            <br>
    SmBoolean         & rbSomeFacesFailedToTessellate,  ///< [out]: TRUE = a problem face was encountered                          <br>
    SmTArray<SmFace*> * pOptFacesToTessellate = NULL,   ///< [in ]: Only Set up to tessellate these faces for now.                 <br>
    SmTArray<SmStatus>* pFaceStatuses = NULL // out: First error per setup face; SM_SUCCESS for successful faces
  );                                                                                                                          

  // Tessellate Phase 2 - for 1 pFace: add compatible Face->Surface tessellation to existing mTS_pPolyBrep Face->Edge->Curve tessellations.
  //   m_pTessBrep         = not modified.
  //   m_vCache            = pSC->TreeNodes of RelatePair<pFace,pSC> is loaded with pFace->UVTrimCurves
  //                         and labeled in/out/on pFace.
  //   pSC->mTS_pPolyBrep  = gets additional new SmPolyEdges and SmPolyVertices
  //   m_p3DPolyBrep       = NULL ;
  SmStatus      Phase2CreateFacePolygons(SmFace * pFace);

  // Interface method:
  //  note 1. Output all PFace->Surface's (SmTessSrfCache)pSC->mTS_pPolyBrep pPolyFaces as polygons
  //          through rPolygonOutput class after Phase2 Tessellation has been run on all faces.
  //  note 2. output to m_p3DPolyBrep by setting rPolygonOutput->m_eOutputType == SM_PO_CREATE_POLYBREP
  SmStatus     OutputPolygons
  (
    SmPolygonOutputCallback & rPolygonOutput,  ///< [in ]: Chooses how and where to output the polygons    <br>
    SmGfxArraySet           * pOptGfxSet=NULL  ///<      :                                                 <br>
  );                                                                                                     

  // Interface method:
  //  note 1. output just 1 PFace->Surface's (SmTessSrfCache)pSC->mTS_pPolyBrep pPolyFaces as polygons through rPolygonOutput class
  //          after Phase2 Tessellation has been run on PFace.
  //  note 2. output to m_p3DPolyBrep by setting rPolygonOutput->m_eOutputType == SM_PO_CREATE_POLYBREP
  //  note 3. when outputting to m_p3DPolyBrep, after last OutputFacePolygons call, call m_p3DPolyBrep->SetOKBackPtrs(FALSE) ;
  SmStatus     OutputFacePolygons
  (
    SmFace                  * pFace,                   ///< [in ]: target face
    SmPolygonOutputCallback & rPolygonOutput,          ///< [in ]: Chooses how and where to output the polygons                                                  <br>
    SmPolyShell            ** pOptPolyShell=NULL,      ///< [in ]: only used when rPolygonOutput.GetOutputType() == SM_PO_CREATE_POLYBREP                        <br>
                                                       ///<      :  ppOptPolyShell NULL    = every new SmPolyFace gets a new SmPolyShell and SmPolyRegion        <br>
                                                       ///<      :  *ppOptPolyShell NotNULL = owner of any new SmPolyFaces constructed                           <br>
                                                       ///<      :  *ppOptPolyShell NULL    = create and save ptr to new PolyShell for New SmPolyFaces of pFace  <br>
                                                       ///<      : NULL to ignore, default:[NULL]                                                                <br>
    SmGfxArraySet           * pOptGfxSet=NULL          ///< [in,out]: only used when rPolygonOutput.GetOutputType() != SM_PO_CREATE_POLYBREP                     <br>
                                                       ///<      : used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.                  <br>
                                                       ///<      : NULL to ignore. default:[NULL]                                                                <br>

  );

  // Tessellate Phase 3 - for 1 pFace: clean up temp tessellation data structures
  //   m_pTessBrep        = not modified.
  //   m_vCache           = remove RelatePair<pFace,pSC> and delete SmTessSrfCache pSC
  //   pSC->mTS_pPolyBrep = deleted
  //   m_p3DPolyBrep      = not modified.
  SmStatus      Phase3DeleteFacePolygons(SmFace * pFace);

  SmStatus      MakePolyBrepFromFaceEdges
  (
    SmFace    * pFace,
    SmBoolean & rbRefinementDone
  );

  SmStatus      RemoveTopologicalVertex (SmPolyVertex * pVertexToRemove);

  static SmVector3d ScaleUVVecTo3DLength
  (
    const SmSurface  * cpSurface,
    const SmPoint3d  & crUVPoint,
    double             d3DLength,
    const SmVector3d & crUVVector
  );

  void          SetAdvancingFront       (SmBoolean bAdvancingFront)         { m_bAdvancingFront = bAdvancingFront; }

  void          SetSmoothingData        (const SmSmoothingData & crSmoothingData);

  void          SetTessellationAlgorithm(SmTessAlgorithmType eTessAlgorithm) { m_eTessAlgorithm = eTessAlgorithm; }

  void          SetChecker              (SmBoolean bChecker)                 { m_bCheckLicense = bChecker; }

  SmStatus      SmoothPolygons          (SmPolyBrep *pPolyBrep);

  SmStatus      SplitSegmentOfLoop      
  (
    SmPolyLoop      * pLoop,
    const SmPoint3d & crTestPoint,
    SmBoolean       & rbSplitDone
  );

  SmStatus      SubdivideManifoldEdge   
  (
    SmPolyEdge * pEdgeToSplit,              ///< [in ]: PolyEdge to split                                              <br>
    ULONG        lNumberSplits,             ///< [in ]: number of splits required                                      <br>
    SmTree     * pVertexTree,               ///< [in ]: spatial tree of <PolyEdge,PolyEdge->StartPt position> pairs    <br>
    SmBoolean    bReturnPolyEdges,          ///< [in ]: flag for sending out the newly constructed PolyEdges.          <br>
    SmTArray<SmPolyEdge*> & sNewPolyEdges   ///< [out]: newly constructed PolyEdges, including the polyedge passed in. <br>
  );

  SmStatus      TessellateEdge          (SmEdge * pEdgeToTessellate);

  SmStatus      TraceLaminaLoop         
  (
    SmPolyEdge            * pStartEdge,          ///< [in ]:      <br>
    SmTessSrfCache        * pTessCache,          ///< [in ]:      <br>
    SmTArray<SmPolyEdge*> & rEdgesOfLoop         ///< [out]:      <br>
  );  

  SmStatus      TraceQuads              
  (
    SmSurfParamType           eDirection,                  ///< [in ]: SM_SP_U = hold V constant, move in positive U dir.         <br>
                                                           ///<      : SM_SP_V = hold U constant, move in positive V direction.   <br>
    SmPolyFace              * pStartFace,                  ///< [in ]:                                                            <br>
    SmBoolean                 bForce3DPlanarity,           ///< [in ]:                                                            <br>
    SmTArray<SmPolyFace*>   & rTracedFaces,                ///< [out]:                                                            <br>
    SmTArray<SmPolyVertex*> & rMinVerts,                   ///< [out]:                                                            <br>
    SmTArray<SmPolyVertex*> & rMaxVerts,                   ///< [out]:                                                            <br>
    SmMarkType                eMarkType                    ///< [in ]: checks without incrementing or assigning eMarkTypeValue    <br>
  ) ; 


  // subdivide a PolyFace into a set of triangular PolyFaces in PolyFace's PolyBrep
  SmStatus      TriangulateFace         (SmPolyFace * pPolyFace);

  // triangulateFace() helper method
  SmStatus      TriangulateSingleFace   
  (
    SmTArray<SmPolyFace*> & rPolyFaceStack,           ///< [in,out]: Top member gets triangulated,                                    <br>
                                                      ///<         : MakeManifoldEdge() generated children get added to stack.        <br>
    SmTArray<SmPolyEdge*> & rPolyEdgesToSubdivide,    ///< [in,out]: new poly edges which should subdivided                           <br>
    SmTArray<ULONG>       & rPolyEdgeSubdivisions,    ///< [in,out]: number of subdivisions for members of rPolyEdgesToSubdivide      <br>
    SmTree               *& rpVertexTree,             ///< [in,out]: Spatial tree of PolyFace PolyVertices.                           <br>
                                                      ///<         : New PolyVertices are added.                                      <br>
    SmNewMarkAndLock      & rMarkLock                 ///< [in ]: increments MarkLock mark value when rpVertexTree is NULL on input   <br>
  );   

  // Add PolyLines to divide PolyFace with more than 3 PolyEdges into a set of triangular 3 edge PolyFaces. (AF = AdvancingFront)
  SmStatus      TriangulateFaceAF       
  (
    SmPolyFace            * pFace,           ///< [in ]:      <br>
    SmFace                * pBrepFace,       ///< [in ]:      <br>
    double                  dQuadSize,       ///< [in ]:      <br>
    SmTArray<SmPolyFace*> & rNewFaces        ///< [out]:      <br>
  );

  // Split <PolyEdge, Edge> pairs in Phase1SetupBrep. Keeps m_pTessBrep Edge to mTS_pPolyBrep PolyEdge correspondence.
  // Doesn't split Segments when SplitPoint is near EndPoints.
  SmStatus      TrySplitSegment         
  (
    SmPolyEdge      * pEdgeToSplit,       ///< [in ]: UV PolyEdge to split                                                 <br> 
    const SmPoint3d & crPointToSplit,     ///< [in ]: target UV Split Point                                                <br>
    SmBoolean       & rbSegmentIsSplit    ///< [out]: TRUE = pPEdgeToSplit and pPEdgeToSplit->Edgeuse->Edge where split    <br>
                                          ///<      : FALSE= no Edges or PolyEdges were split
  );

  void          SetPolyEdgeEU           (SmPolyEdge * pPolyEdge, SmEdgeuse  * pEdgeuse);

  void          SetPolyEdgeVU           (SmPolyEdge  * cpPolyEdge, SmVertexuse * cpVertexuse);

  SmEdgeuse   * GetPolyEdgeEU           (const SmPolyEdge * cpPolyEdge) const;

  SmVertexuse * GetPolyEdgeVU           (const SmPolyEdge * cpPolyEdge) const;

  void          SetPolyEdgeVertexuse    (SmPolyEdge  * cpPolyEdge, SmVertexuse * cpVertexuse);

  SmVertexuse * GetPolyEdgeVertexuse    (const SmPolyEdge * cpPolyEdge) const;

  const TCHAR * GetTypeString() const  { return _T("SmTess") ; }

} ; // end class SmTess


#endif // !__SMTESS_H__
