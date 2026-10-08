// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfaceCache.h
* PURPOSE: Header file for SmSurfaceCache object.
**********************************************************************/

#ifndef __SMSURFACECACHE_H__
#define __SMSURFACECACHE_H__

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMPSEUDOBOX_H__
#include <SmPseudoBox.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

#ifndef __SMPOLARBOX_H__
#include <SmPolarBox.h>
#endif

#ifndef __SMTREE_H__
#include <SmTree.h>
#endif

#ifndef __SMTLIST_H__
#include <SmTList.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#ifndef __SMCACHEMGR_H__
#include <SmCacheMgr.h>
#endif

// gwc: added following dependency when the type hiding in SmBezierPatch was removed.
//      If we modify SmBezierPatch to just have a pointer to a gw_SURFACE rather
//      than contain one, we can remove this compile dependency.
//#ifndef _NURBS_H_INCLUDED
//#include <nurbs.h>
//#endif

// forward declarations
class SmVertexuse;  // external class declaration
class SmEdgeuse;    // external class declaration
//      typedef struct gw_surface gw_SURFACE ;


/*******************************************************************//**
PURPOSE: This enum defines where the node lies relative to the 
    trim boundary of a trimmed surface.

NOTES: 
***********************************************************************/
enum SmNodeClassType {
    SM_NC_UNKNOWN,      // node has yet to be classified - not used once tree is classified
          
    SM_NC_INSIDE,       // leaf   node: node's entire uvDomain is inside trim boundaries.
                        // parent node: all node's descendants are inside nodes.
                            
    SM_NC_OUTSIDE,      // leaf   node: node's entire uvDomain is outside trim boundaries.
                        // parent node: all node's descendants are outside nodes.
                             
    SM_NC_ON_BOUNDARY   // leaf   node: node's uvDomain is close enough or intersects a trim boundary.
                        // parent node: at least one of the node's descendants is a boundary node.  
};

/*******************************************************************//**
PURPOSE: This enumerates the possible results of the boundary plane test
            in surface/surface intersection (the result of testing the 
            bounding planes of the boundary curves against each other).

NOTES: 
***********************************************************************/
enum SmBoundaryPlaneTestResult {
    SM_BR_GRAZE,     // one pair of boundary curve bounding planes is
                     // coincident, and the surfaces are on the opposite 
                     // sides of this plane
    SM_BR_DISJOINT,  // The surfaces are definitely disjoint
    SM_BR_UNKNOWN    // Intersection possible: more work needed
};


/*******************************************************************//**
PURPOSE: This object is used to store the bounding planes of 
            a rectilinear UV subDomain of a surface when such exists. 

  The 4 isoParameter boundary curves of the UVDomain are tested, 
    the following is stored for each
      1. Is Curve Degenerate
      2. Is Curve Planar 
         and does Surface lie completely to one side of the surface
         2a. Plane base point
         2b. Plane normal - pointing outside of surface
         2c. Plane Tolerance 
            
NOTES: 
Example:

  The ordering of the elements of the arrays:
                -------2-------
                |             |
                3             1
                |             |
            u0v0--------0----- u0v1   
***********************************************************************/
#define SM_PB_SAMPLECOUNT 5

class SM_EXPORT SmPatchBoundaryBoundingPlanes
{
protected:
    const SmSurface* m_pSurface;                    // Surface back pointer
    SmExtent2d       m_sUVDomain;                   // 2D surface domain bounding box
    SmBoolean        m_bPlanesComputed;             // 1=yes,0=no
    SmBoolean        m_bBoundaryCurvePlanar[4];     // 1=yes, Surface->UVBoundaryCurve[i] is planar and 
                                                    //        surface lies completely to one side of that plane. 
                                                    // 0=no
    SmBoolean        m_bBoundaryCurveDegenerate[4]; // 1=yes, Surface->UVBoundaryCurve[i] is degenerate.
                                                    // 0=no
    SmPoint3d        m_vBasePoint[4];               // base point for each bounding plane
    SmPoint3d        m_vNormalVector[4];            // associated normal vec for each bounding plane
    SmPoint3d        m_vSamplePoint[4][SM_PB_SAMPLECOUNT];  // Surface->UVBoundaryCurve[i] Sample Points 
                                                            // to define BoundaryCurve's bounding Box on bounding_plane
    double           m_dPlaneTolerance[4];          // computed max dist between boundary_curve and bounding_plane

    // compute and save the ComputeBoundingPlanes parameters 

    // eff: compute all member values
    SmStatus ComputeBoundingPlanes(void);  

    // eff: NULL Constructor
    SmPatchBoundaryBoundingPlanes();                
public:

    // constructor and destructor
    SmPatchBoundaryBoundingPlanes(const SmSurface*, const SmExtent2d&);
   ~SmPatchBoundaryBoundingPlanes();

    SmBoolean  ArePlanesComputed() const { return(m_bPlanesComputed) ; }

    // retrieve surface bounding plane (if there is one) for given direction and side
    SmStatus GetBoundingPlane
    (
      SmSurfParamType,                        ///< [in]: oneof SM_SP_U or SM_SP_V                                                   <br>
      ULONG,                                  ///< [in]: 0 for Min, 1 for Max                                                       <br>
      SmBoolean   & rePlaneExistsArg,         ///< [out]: TRUE=3d Projection of ith bounding isoParameter curve is planar           <br>
                                              ///<      : and surface lies completely to one side of that plane.                    <br>
      SmBoolean   & rsBoundaryDegenerateArg,  ///< [out]: TRUE=3d Projection of ith bounding isoParameter curve is degenerate       <br>
      SmPoint3d   & rsPlaneBasePointArg,      ///< [out]: base point of bounding plane                                              <br>
      SmVector3d  & rsPlaneNormalArg,         ///< [out]: normal vec of bounding plane                                              <br>
      SmVector3d *& pSamplePoint,             ///< [out]: Curve Sample Points to define Curve's bounding Box on bounding_plane      <br>
                                              ///<      : sized:[SM_PB_SAMPLECOUNT]                                                 <br>
      double      & rdToleranceArg            ///< [out]: max deviation from plane                                                  <br>
    );

    // retrieve surface bounding plane (if there is one) for given boundary index
    SmStatus GetNthBoundingPlane
    (
      ULONG,                                   ///< [in ]: An iterator index, [0->3]                                       <br>
      SmBoolean   & rePlaneExistsArg,          ///< [out]: TRUE = ith boundary curve is planar and                         <br>
                                               ///<      : Surface lies to one side of plane                               <br>
      SmBoolean   & rsBoundaryDegenerateArg,   ///< [out]: TRUE = ith boundary curve is degenerate                         <br>
      SmPoint3d   & rsPlaneBasePointArg,       ///< [out]: plane point, only set when rbPlaneExistsArg == TRUE             <br>
      SmVector3d  & rsPlaneNormalArg,          ///< [out]: plane normal, only set when rbPlaneExistsArg == TRUE            <br>
      SmVector3d *& pSamplePoint,              ///< [out]: pointer to array of boundary curve sample points.               <br>
                                               ///<      : no new memory allocated, just referenced.                       <br>
                                               ///<      : sized:[SM_PB_SAMPLECOUNT]                                       <br>
      double      & rdToleranceArg             ///< [out]: max distance from boundary curve control polygon to plane       <br>
    );

    // build 3d isoParameter Curve for given boundary index
    SmStatus CreateBoundaryCurveForIndex
    (
      ULONG,                                   ///< [in ]: 0-3                     <br>
      SmBSplineCurve*&                         ///< [out]: newly constructed curve <br>
    );

    // get UVPoints bounding given boundary index
    SmStatus GetBoundaryUVEndPoints
    (
      ULONG,                     ///< [in]: 0-3                 <br>
      SmPoint2d &rMin,           ///< [in]: Start UV EndPoint   <br>
      SmPoint2d &rMax            ///< [in]: End UV EndPoint     <br>
    ) ;

    const SmSurface *GetSurface()       { return m_pSurface ; }
    
    // draw finite bounding planes in neighborhood of surface BBox 
    SmDisplayList * Draw() const ;

} ; // end class SmPatchBoundaryBoundingPlanes

/*******************************************************************//**
PURPOSE: This object is used to represent a vertex list in the 
   SmBezierAux2d object.

NOTES: 
***********************************************************************/
class SM_EXPORT SmVertexList : public SmListNode
{
 public:
  SmVertexuse    * m_pVU = NULL ;
  SmPoint2d        m_vUVPoint;
  SmVector2d       m_vUVTol;

 public:
  virtual void Dump() const ;

} ; // end class SmVertexList

/*******************************************************************//**
PURPOSE: This object is used to represent a edgeuse list in the SmBezierAux2d
   object.

NOTES: static class object - don't add virtual methods to SmCurveNode.
***********************************************************************/
class SM_EXPORT SmEdgeuseList : public SmListNode
{
public:
  SmEdgeuse        * m_pEU;        
  // not used: SmTreeNode       * m_pCurveNode;

  SmEdgeuseList() : m_pEU(NULL) { }

  ~SmEdgeuseList()
  {
    SmListNode::ReSet(); // clear Next/Last ptrs
    m_pEU = NULL;
  }

  void Dump() const ;

} ; // end SmEdgeuseList

/*******************************************************************/ /**
 PURPOSE: This object is used to represent a PolyEdge list in the SmBezierAux2d
    object.

 ***********************************************************************/
class SM_EXPORT SmPolyEdgeList : public SmListNode
{
public:
  SmPolyEdge* m_pPolyEdge;

  SmPolyEdgeList() : m_pPolyEdge(NULL) { }

  ~SmPolyEdgeList()
  {
    SmListNode::ReSet(); // clear Next/Last ptrs
    m_pPolyEdge = NULL;
  }

    void Dump() const ;

}; // end SmPolyEdgeList

SM_TLIST_TEMPLATE_PREDECLARATION(SmVertexList) ;
SM_TLIST_TEMPLATE_PREDECLARATION(SmEdgeuseList) ;
SM_TLIST_TEMPLATE_PREDECLARATION(SmPolyEdgeList);

/*******************************************************************//**
PURPOSE: This object is used to represent auxillary data in the
    tree nodes used by surface subdivision.

    Each node in the subdivision tree represents a sub-domain of a surface
    For each node in the subdivision tree - store
     For general use:
       child split direction (u or v),
       run-time status bit to let algorithms track visits to a particular node
       back pointer to target surface
       this node's sub-domain box

     For tessellations:
       Node-to-Trim_boundary classification in/on/out of trim boundary
       SmPolyFace back pointer (each SmPolyFace is one facet of a tessellation)
       ChordHeight flag 1=sub-division satisfies tol or all its sub-nodes do
       AngleTol    flag 1=sub-divison satisfies angle tol
       List of tessellation vertices within this node
       List of tessellation edges within this node

NOTES:
    A surface decomposition has two kinds of nodes (tracked in three ways)

    1.) m_eAuxDataType == SM_AD_BEZIER_SURFACE, m_pData_TYPE = SmBezierPatch
      The key decomposition nodes have a pointer to a SmBezierPatch object derived
      from SmBezierAux2d that has a set of nested bounding boxes and
      neighbor connectivity data.
      
    2.) m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD, m_pData_TYPE = SmBezierAux2d
      Depending on the decomposition tree history a set of initial sibling
      SM_AD_BEZIER_SURFACE decomposition nodes may need to be organized under a tree top.
      When this is done those nodes have a pointer to an SmBezierAux2d type
      and their bounding boxes are computed as the union of their children bounding boxes.

    3.) m_eAuxDataType == SM_AD_AUX_DATA, m_pData_TYPE = SmBezierAux2d
      When trim edges are added to the surface cache, relatively flat decomposition
      pieces may be further subdivided so that each trim edge is resonably fitted
      with decomposition nodes in UV space.  When this is done those nodes have
      a pointer to an SmBezierAux2d type and their bounding boxes are just set equal
      to the bounding box of the first ancester of type SM_AD_BEZIER_SURFACE.  Perhaps
      if those bounding boxes (the pseudo box in particular) become cheap to compute
      then it would be worth while to compute those extra bounding boxes.

  static class object - don't add virtual methods to SmBezierAux2d
***********************************************************************/
class SM_EXPORT SmBezierAux2d : public SmTreeNodeData
{
private:
  SmPolarBox m_sPolarBox; // This bounds the direction of the surface normals
public:
  SmSurfParamType       m_eSplitDir = SM_SP_UNKNOWN ; // Direction children are split, u or v
                                                      //   SM_SP_U = split u direction into left/right children
                                                      //   SM_SP_V = Split v direction into bot/top children
                                                      
  SmBoolean             m_bIsMarked = FALSE ;         // Mark to see if traversed yet.
                                                      //  rule: parent node is marked after both children have been marked

  SmBoolean             m_bIsTessOnly = FALSE ;       // Flag to distinguish nodes of the tree defined purely by the geometric
                                                      // tessellation constraints.
                                                      //  rule: = TRUE for nodes which built for tessellation, after initial tree construction.
                                                      //        = FALSE for nodes arising from initial construction.   
                                                      
  SmNodeClassType       m_eNodeClass = SM_NC_UNKNOWN ;// Classification of this node relative to the trimmed
                                                      // surface boundary: SM_NC_INSIDE
                                                      //                   SM_NC_OUTSIDE
                                                      //                   SM_NC_ON_BOUNDARY
                                                      //  rule: parent inside : all descendants are inside
                                                      //        parent outside: all descendants are outside
                                                      //        parent on     : at least one descendant is on boundary
                                                      
  SmObject            * m_pFace = NULL ;              // Corresponding SmPolyFace (used for tessellation) if m_eNodeClass is inside
                                                      //  rule: leaf nodes   : Face pointer if Surface is owned by a face
                                                      //        parent nodes : not set left at [NULL]
                                                      
  SmExtent2d            m_sUVDomain;                  // Parametric domain of the surface
// GWCTreeVertexTemp
//  ULONG                 m_lStartTreeVertexIndx;     // Index of Tree Vertex marking upper left corner of the m_sUVDomain.
//                                                    // uninitialized                 == SM_UNDEF_ULONG
//                                                    // for internal nodes (No value) == SM_BIG_ULONG
//                                                    // for leaf nodes                == index into SmTree::m_sTreeVertices for start SmTreeVertex
  SmBoolean             m_hasInsidePart = FALSE ;     // the node or one of its children is SM_NC_INSIDE
  SmTreeNode*           m_CornerNode [4] = {NULL} ;   // BL, BR, TR, TL node of subtree


  // variable length data
  SmTList<SmVertexList> m_vVertexList;           // List of vertices in this UV domain (SmVertexList == SmVertexuse pointer)
                                                 //  rule: leaf nodes   : number of Vertexuses in node's UVDomain
                                                 //        parent nodes : not set left at [0]

  SmTList<SmEdgeuseList>  m_sEdgeuseList;        // List of edgeuse pointers in this UV domain (SmEdgeuseList == SmEdgeuse pointer)
                                                 //  rule: leaf nodes   : number of Edgeuses in node's UVDomain
                                                 //        parent nodes : not set left at [0]

  SmTList<SmPolyEdgeList> m_sPolyEdgeList;       // List of PolyEdge pointers in this UV domain (SmPolyEdyeList == SmPolyEdge pointer)
                                                 //  rule: leaf nodes   : PolyEdges in node's UVDomain
                                                 //        parent nodes : not set left at [0]

  const SmSurface     * mBA_pSurface = NULL ;    // Used by SmTessSrfCache::BuildTree() during construction.
                                                 //   Corresponding surface that has been subdivided to
                                                 //   have the given Domain of this node. This is not the m_pFace->Surface but
                                                 //   a temporary copy stored in SmSurfaceCache::m_sSubdivisionSurfaces.
                                                 // 
                                                 // After construction these pointers are reset to the owner surface which
                                                 //     was subdivided.

  // construction only members
  SmBoolean             m_bChordHeightSatisfied = FALSE ; // If TRUE all sub surfaces satisfy their chord height, only used in construction
  SmBoolean             m_bAngleTolSatisfied    = FALSE ; // If TRUE all sub surfaces satisfy their angle tol, only used in construction

 public:
  // constructor
  SmBezierAux2d() : m_eSplitDir(SM_SP_UNKNOWN),  // typically not used - These are allocated through a SmMemBlockMgr object
                    m_bIsMarked(FALSE),
                    m_bIsTessOnly(FALSE),
                    m_eNodeClass(SM_NC_UNKNOWN),
                    m_pFace(NULL),
// GWCTreeVertexTemp
//                    m_lStartTreeVertexIndx(SM_UNDEF_ULONG),
                    mBA_pSurface(NULL),
                    m_bChordHeightSatisfied(FALSE),
                    m_bAngleTolSatisfied(FALSE)
                  { }

  // simple data access
  const SmExtent2d  & GetUVDomain() const          { return m_sUVDomain ; }
  //SmPolarBox        & GetPolarBox()                { return(m_sPolarBox) ; }
  const SmPolarBox  * GetPolarBoxPtr() const       { return(&m_sPolarBox) ; } 

// GWCTreeVertexTemp
//  void   SetStartTreeVertexIndx(ULONG indx)        { m_lStartTreeVertexIndx = indx ; }

  // no virtual functions allowed by memory block manager

  SmPolarBox& GetPolarBox();

  SmStatus BuildPolarBox();

} ; // end class SmBezierAux2d

/*******************************************************************//**
PURPOSE: This object is the primary representation for the decomposition
    of a surface.  It is a non-virtual object which is typically allocated
    in blocks of more then one by the SmMemBlockMgr.

NOTES:
  static class object - don't add virtual methods to SmBezierPatch  
***********************************************************************/
class SM_EXPORT SmBezierPatch : public SmBezierAux2d
{
  friend class SmSurfaceCache;
  friend class SmSurfaceTessDriver;

  // inherited:
  // SmBezierAux2d::m_eSplitDir;              // Direction children are split, u or v                                      
  // SmBezierAux2d::m_bIsMarked;              // Mark to see if traversed yet                                              
  // SmBezierAux2d::m_eNodeClass;             // Classification of this node relative to the trimmed                       
  //                                          // surface boundary (inside/outside/on boundary)                             
  // SmBezierAux2d::m_pFace;                  // Corresponding SmPolyFace (used for tessellation) if m_eNodeClass is inside
  // SmBezierAux2d::m_pSurface;               // Corresponding surface that has been subdivided to                         
  //                                          // have the given Domain of this node.                                       
  // SmBezierAux2d::m_sUVDomain;              // Parametric domain of the surface 
  // SmBezierAux2d::m_sPolarBox;              // Bounding Box of surface normal vectors                                         
  // SmBezierAux2d::m_vVertexList;            // List of vertices in this UV domain                                        
  // SmBezierAux2d::m_sEdgeuseList;           // List of curve tree pointers in this UV domain
  // SmBezierAux2d::m_sPolyEdgeList;          // List of PolyEdge pointers in this UV domain                            
  // 
  // SmBezierAux2d::m_bChordHeightSatisfied;  // If TRUE all sub surfaces satisfy their chord height, for construction only                       
  // SmBezierAux2d::m_bAngleTolSatisfied;     // If TRUE all sub surfaces satisfy their angle tol, for construction only                          
// GWCTreeVertexTemp
  // SmBezierAux2d::m_lStartTreeVertexIndx;   // Index of Tree Vertex marking upper left corner of the m_sUVDomain.

public:
  // Cached Bezier Patch properties computed for the Bezier Patch below
  // gwc:not used - SmContinuityType  m_eInternalContinuity;
  gw_SURFACE      * m_pBezier = NULL ;                      // The subdivision bezier patch during construction - set to NULL after construction.
  SmContinuityType  m_eUCurveConts[2] = {SM_CT_UNDEFINED} ; // Continuity at min and max U of patch
  SmContinuityType  m_eVCurveConts[2] = {SM_CT_UNDEFINED} ; // Continuity at min and max V of patch

private:
  SmPseudoBox       m_sPseudoBox;         // A tight bounding box - parallapiped
public:                                   

  // simple data access
  SmPseudoBox & GetPseudoBox()         { return(m_sPseudoBox) ; }
  gw_SURFACE  * GetBezierPtr()         { return m_pBezier; }

  // no virtual functions allowed by memory block manager

} ; // end class SmBezierPatch

/*******************************************************************//**
PURPOSE: Defines the type of data kept in the surface cache.

NOTES: 
***********************************************************************/
enum SmAltSrfCacheType {
  SM_AS_SILHOUETTE,          // contains silhouette curves trimmed
                             // to surface natural domain.
  SM_AS_TRIMMED_SILHOUETTE   // contains silhouette curves trimmed
                             // to face boundaries.
} ;

/*******************************************************************//**
PURPOSE: This class is a virtual super class which enables additional
    data to be associated with a surface cache and automatically destroyed.

NOTES: 
***********************************************************************/
class SM_EXPORT SmAltSrfCache : public SmObject
{
 protected:
  SmAltSrfCacheType         m_eSrfCacheType;
  double                    m_dApproxTol;
  double                    m_dAngleTol;
  SmVector3d                m_vUserVec;
  SmBoolean                 m_bUserBool;
  long                      m_lUserLong;
  double                    m_dUserDouble;
  const void             *  m_cpUserPointer;
  SmTArray<SmCurve*>     *  m_pCached3DCurves;
  SmTArray<SmCurve*>     *  m_pCachedUVCurves;

 public:
  SmAltSrfCache
  (
    const SmTArray<SmCurve*> * cp3DCurvesToCache,
    const SmTArray<SmCurve*> * cpUVCurvesToCache,
    SmAltSrfCacheType eSrfCacheType,
    double dApproxTol,
    double dAngleTol,
    const SmVector3d * cpUserVec = NULL,
    const SmBoolean       * cpUserBool = NULL,
    const long       * cpUserLong = NULL,
    const double     * cpUserDouble = NULL,
    const void       * cpUserPointer = NULL
  );

  virtual ~SmAltSrfCache();

  SmBoolean MatchCache
  (
    SmAltSrfCacheType eSrfCacheType,                 ///< [in ]: cache parameter to compare    <br>
    double dApproxTol,                               ///< [in ]: cache parameter to compare    <br>
    double dAngleTol,                                ///< [in ]: cache parameter to compare    <br>
    const SmVector3d * cpUserVec = NULL,             ///< [in ]: cache parameter to compare    <br>
    const SmBoolean       * cpUserBool = NULL,       ///< [in ]: cache parameter to compare    <br>
    const long       * cpUserLong = NULL,            ///< [in ]: cache parameter to compare    <br>
    const double     * cpUserDouble = NULL,          ///< [in ]: cache parameter to compare    <br>
    const void       * cpUserPointer = NULL          ///< [in ]: cache parameter to compare    <br>
  ) const;

  SmStatus GetCopyOfCurves
  (
    const SmContext & crContext,                           ///< [in ]: context for new object construction  <br>
    SmTArray<SmCurve*> * pCopyOf3DCurvesInCache,           ///< [out]: Optional 3D Curve copy output array, <br>
                                                           ///<      : NULL to skip curve copies.           <br>
    SmTArray<SmCurve*> * pCopyOfUVCurvesInCache = NULL     ///< [out]: Optional UVCurve copy output array,  <br>
                                                           ///<      : NULL to skip curve copies.           <br>
  ) const;

} ; // end class SmAltSrfCache

/*******************************************************************//**
PURPOSE: This is the number of alternate surface caches (SmAltSrfCache)
    that are allowed to be attached to a normal surface cache (SmSurfaceCache).

NOTES: 
***********************************************************************/
#define SM_MAX_ALT_CACHES 5

enum SmTessAlgorithmType 
{ SM_TA_FASTER_TESSELLATION,  // Default 
  SM_TA_FEWER_POLYGONS
};

/*******************************************************************//**
PURPOSE: This object is the high level object which contains the
   decompostion of a surface.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmSurfaceCache : public SmCacheObj
{
  friend class SmTree;
  friend class SmSurface;
protected:
  ULONG                         m_lNumAltCaches;                      // size of m_apAltSrfCaches array
  SmAltSrfCache               * m_apAltSrfCaches[SM_MAX_ALT_CACHES];  // array of extra data objects that can be 
                                                                      //   stored with surface cache.  used for storing 
                                                                      //   silhouette and trimmed silhouette curves.
                                                                      //   silhouette curves are used to solve surfance/point 
                                                                      //   projection problems.
  
  SmTree                      * m_pTree;                              //   A surface decomposition binary tree where each node
                                                                      //   represents a sub-domain of the surface.  Each node contains
                                                                      //     1. UVBounding  Box - under m_pData ptr
                                                                      //     2. 3d Bounding Box
                                                                      //     3. 3d Pseudo   Box  - (optional) under m_pData ptr
                                                                      //     4. 3d Polar    Box   - under m_pData ptr
                                                                      //   Each parent node has just 2 children
                                                                      //    When pParentNode->m_pData->m_eSplitDir = 
                                                                      //      SM_SP_U: child1 = lower u domain half, left  side
                                                                      //               child2 = upper u domain half, right side
                                                                      //      SM_SP_V: child1 = lower v domain half, bot   side
                                                                      //               child2 = upper v domain half, top   side

  const SmSurface             * m_cpSurface = NULL;                   // Associated Surface Object
  SmExtent2d                    m_sUVDomain;                          // Surface Extent for the Cache 
                                                                      //      (SmSurfaceCache = Natural UV Domain)
                                                                      //      (SmTrimSrfCache = Trim Surface UV Domain)

  SmTArray<SmContinuityType>  * m_pUContinuities = NULL;              //      
  SmTArray<SmContinuityType>  * m_pVContinuities = NULL;              //      

  ULONG                         m_lBezierSize;                        // size of SmBezierPatch->m_pNurb with knot and CP arrays in bytes
  ULONG                         m_lTotalBezSize;                      // size of SmBezierPatch (containing Bezier->m_pNurb) in bytes

  SmMemBlockMgr                 m_sBezMgr;                            // memory for SmBezierPatch objects stored with decomposition 
                                                                      //            tree nodes of type SM_AD_BEZIER_SURFACE 
  SmMemBlockMgr                 m_sAuxMgr;                            // memory for SmBezierAux2d objects stored with decomposition 
                                                                      //            tree nodes of types SM_AD_BEZIER_SURFACE_HEAD 
                                                                      //            and SM_AD_AUX_DATA

  SmMemBlockMgr                 m_sVLMgr;                             // used by SmTrimSrfCache, memory for SmVertexLists stored with tree nodes                    
  SmMemBlockMgr                 m_sEdgeuseListMgr;                    // used by SmTrimSrfCache and SmTessSrfCache, memory for SmEdgeuseLists stored with tree nodes
  SmMemBlockMgr                 m_sPolyEdgeListMgr;                   // used by SmTrimSrfCache and SmTessSrfCache, memory for SmPolyEdgeLists stored with tree nodes

  // tessellation control                                                                      
  double                        m_dChordHeightTolerance;              // max allowed Element ControlPoint to BasePlane3d dist3d,      0.0 = ignore 
  double                        m_dAngTolRad;                         // max allowed Element ControlPolygon turning ang3d (radians),  0.0 = ignore 
  double                        m_dAspectRatio3D;                     // max allowed Element BasePolygon aspect ratio3d,              0.0 = ignore 
  double                        m_dMaximumSideLength3D;               // max allowed Element BasePolygon side length3d,               0.0 = ignore 
  double                        m_dMinimumSideLength3D;               // min allowed Element BasePolygon side length3d,               0.0 = ignore, 0.0 = ignore
  double                        m_dMinimumSideLengthRatioUV;          // min allowed Element (side lengthUV/origUVDomain.Size) ratio, 0.0 = ignore 
                                                                      //   note: smallest elements run about 1/2 of m_dMinimumSideLength3D when used.
                                                                      //   note: smallest elements run about 1/2 of m_dMinimumSideLengthRatioUV when used.


  double                        m_dIsoCurveTolerance;                 // temp value, Last ApproxTol3d value used to produce an IsoParamCurve by GetIsoBoundaryCurves()

  // Optimization for SmTessSrfCache::BuildTree().  When node angle
  //    goes below m_dStartFastSubdivisionAngleDeg value 
  //    AND m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION,
  //    Then switch to SubdivideByBlock() from SubdivideNode() in SubdivideToTolerances().
  //    Switch state is stored in m_bSwitchToFastSubdivision.
  SmTessAlgorithmType           m_eTessellationAlgorithm;             // default: SM_TA_FASTER_TESSELLATION use optimization for SmTessSrfCache::BuildTree() where appropriate
                                                                      //          SM_TA_FEWER_POLYGONS      = Never used in SMLib
  double                        m_dStartFastSubdivisionAngleDeg;      // patch angle below which SmTessSrfCache::BuildTree() subdivision switches
                                                                      //       from SubdivideNode() to SubdivideByBlock() in SubdivideToTolerances().
  SmBoolean                     m_bSwitchToFastSubdivision;           // Value set in TestAgainstTolerances when node division is from SmTessSrfCache::BuildTree()
                                                                      //       and the node U and V dir turning angles are both less than m_dStartFastSubdivisionAngleDeg.
                                                                      //          TRUE  = switch from SubdivideNode() to SubdivideByBlock() in SubdivideToTolerances().
                                                                      // default: FALSE = not yet time to switch.

  //
  double                        m_dTrimSubdivisionFactor;            // When UVTrimCurves are implanted into Surface Cache, LeafNodes are
                                                                     // subdivided until contained UVTrimCurve-segment Control-polygon BBox
                                                                     // widths are less than UVDomainSize/m_dTrimSubdivisionFactor ;
                                                                     // default:[20], bounded between [10, 100].
                                                                     //  10 = (1/10  UVDomainSize) Smaller Cache, slower to classify (minimum)
                                                                     // 100 = (1/100 UVDomainSize) Larger Cache, faster to classify (maximum)

  SmBSplineCurve              * m_apIsoBoundaryCurves[4]; // Surface naturalBoundary IsoParameter Curves

  SmTArray<SmSurface*>          m_sSubdivisionSurfaces;   // pointers to temporary subdivision surface copies created and stored
                                                          // within the subdivided nodes but freed at the end of BuildTree(). 
                                                          // Only used during construction by SmTessSrfCache::BuildTree().
                                                          // left empty after constuction

  SmPatchBoundaryBoundingPlanes m_sBoundingPlanes;        // array of boundary planes defined when 3D region boundary curve is
                                                          //   planar and the surface lies completely to one side of the plane.

public:
  SmBoolean                     m_bHaveTSurfaceCache;     // Initial: FALSE, Set to True by SmTrimSrfCache::BuildTree when 
                                                          //                 Subdivision Tree is augmented with object list 
                                                          //                 pointers to face boundary UVTrimCurves and Vertices.
  SmBoolean                     m_bFaceWasModified;       // Initial: FALSE
  SmBoolean                     m_bProcessBoundaryCurves; // default: TRUE  = Do extra work in local solvers to explicitly look 
                                                          //                  for solver solutions on surface boundary curves.  Helpful
                                                          //                    1. when the boundary curves mark surface discontinuities 
                                                          //                       to force local solvers to find good endPoints. 
                                                          //                    2. allows SM_SO_INTERSECT/SM_SO_MINIMIZE/SM_SO_MAXIMIZE solvers
                                                          //                       to find 'near miss' solutions when the actual solution  
                                                          //                       point does not project down a normal vector to the surface.
                                                          //          FALSE = turned off when boundaries are known to be continuous for efficiency
                                                          //                  or when the boundaries are not being used - TrimSurfaces and TessSurfaces.
                                                          //          NOTE: A TRUE value does more work but generates all solutions,
                                                          //                A False value can miss solutions when used on a stand alone surface.
  SmBoolean                     m_bPointTestEnabled;      //          TRUE  = Do Point testing in GlobalPointSolve() and some LocalSolve 
                                                          //                  functions when working with an SmTrimSrfCache.  
                                                          //                  (no PointTests with SmSurfaceCaches)
                                                          //                  Only Keep solution points that are within surface trim boundaries.
                                                          // default: FALSE = No Point Testing = Keep solution points without checking trim boundaries.
  SmBoolean                     m_bFaceContainmentDone;   // Initial: FALSE = Not Done
                                                          //            2   = Under Construction
                                                          //          TRUE  = complete
                               
public:

  // constructor
  SmSurfaceCache
  (
    const SmSurface  & crSurface,
    const SmExtent2d & crUVDomain,
    double             dChordHeightTolerance = 0.0,
    double             dAngTolRad            = 0.0,
    double             dAspectRatio3D        = 0.0,
    double             dMaxSideLength3D      = 0.0,
    double             dMinSideLength3D      = 0.0,
    double             dMinSideLengthRatioUV = 0.001
  );
  
  // destructor               
  virtual ~SmSurfaceCache();

  void SetTrimSubdivisionFactor(double dSubdivisionFactor);

  void SetTessellationAlgorithm(SmTessAlgorithmType eTessAlg) { m_eTessellationAlgorithm = eTessAlg; }

  SmStatus ComputeNetConstants
  (
    const SmSurface  * pSurface,                 ///< [in ]: optional surface expected to be nonNULL for non SmBSplineSurface types             <br>
    const SmExtent2d * pUVDomain,                ///< [in ]: only used when pSurface != NULL                                                    <br>
    const gw_SURFACE * cpSur,                    ///< [in ]: Shape being tested                                                                 <br>
    SmZoneTol3d      & rZoneTol3d,
    double           * pdUChordHeight,           ///< [out]: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore             <br>
    double           * pdVChordHeight,           ///< [out]: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore             <br>
    double           * pdUAngleDeg,              ///< [out]: max Udir polygon endTangent angle, NULL to ignore                                  <br>
    double           * pdVAngleDeg,              ///< [out]: max Vdir polygon endTangent angle, NULL to ignore                                  <br>
    double           * pdUVChordHeight           ///< [out]: max UVdir diagonal controlPoint ist from polygon BasePlane dist, NULL to ignore    <br>
  ) ; 


  // find tree nodes whose uv domain intersect given UVDomain
  SmStatus FindUVNodes
  (
    const SmExtent2d      & crUVDomain,             ///< [in ]: test domain                                               <br>
    SmTArray<SmTreeNode*> & rNodes,                 ///< [out]: all nodes that intersect the test domain                  <br>
    SmBoolean               bCheckForPoles = FALSE  ///< [in ]: TRUE = expand search to include singular sides for poles  <br>
  ) const;                                             

  SmTreeNode * FindUnmarkedLeafNode
  (
    SmNodeClassType         eNodeClassToFind,      ///< [in ]: Type to search for              <br>x
    SmTArray<SmTreeNode*> & rStack                 ///< [in,out]: Stack of Nodes to search     <br>x
                                                   ///<      : Left in an unknown state        <br>x
  ) const;

  // label each subdivision node as in/out/on face boundaries.
  //   This used to be based upon 2d raycasting but now uses loop containment.
  //   As such it can be called with m_bPointTestEnabled on or off.
  SmStatus MarkFaceContainment();

  // walk Child/Parent pointers to get all leaf nodes connected that share the same m_eNodeClass value
  void GetConnectedRegion
  (
    SmTreeNode             *pStartLeafNode,                   ///< [in ]: start node - must be a leaf node                                  <br>
    SmTArray<SmTreeNode *> &rConnectedNodes,                  ///< [out]: connected region list including pStartLeafNode                    <br>
    SmTreeNode            *&rpClassifiedNeighborLeafNode      ///< [out]: first neighbor found with a                                       <br>
                                                              ///<      : m_eNodeClass == SM_NC_INSIDE or SM_NC_OUTSIDE classification      <br>
  ) const ;


  // set rbPointIsOk = !m_bPointTestEnabled ? TRUE : m_sUVDomain.ContainsPoint2d(crUVPoint) ;
  virtual SmStatus PointTest
  (
    const SmPoint2d & crUVPoint,                 ///< [in ]: TargetPoint                                               <br>
    SmBoolean       & rbPointIsOk,               ///< [out]: TRUE = point is passes this TrimBoundary Check            <br>
    SmZoneTol3d     * p3dTolerance = NULL,       ///< [in ]: p3dTolerance = max dist for counting a near miss a hit    <br>
    SmObject        **pOptObject = NULL          ///< [out]: Optional Pointer to Topology Object coincident with point,<br>
  ) const;

  // for derived classes.  Rtn SM_ERR if called.  Currently only implementation is SmTrimSrfCache::PointClassify
  virtual SmStatus PointClassify
  (
    const SmPoint2d           & crUVPoint,        ///< [in ]: Point to classify                                          <br>
    SmZoneTol3d                 sSrcZoneTol3d,    ///< [in ]: Obj ZoneTol3d assoc with UVPoint, not this face            <br>
                                                  ///<      :(if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL)    <br>
    SmPointClassificationType & eClassification,  ///< [out]: Type of object coincident with point                       <br>
                                                  ///<      : oneof SM_PC_VERTEX                                         <br>
                                                  ///<      :       SM_PC_EDGE                                           <br>
                                                  ///<      :       SM_PC_FACE                                           <br>
                                                  ///<      :       SM_PC_UNKNOWN                                        <br>
    SmObject                 *& rpObjectInOrOn    ///< [out]: object coincident with point                               <br>
  ) const;

  void SetProcessBoundaryCurves(SmBoolean bProcessBoundaryCurves)  { m_bProcessBoundaryCurves = bProcessBoundaryCurves; }

  SmBoolean GetProcessBoundaryCurves() const                       { return m_bProcessBoundaryCurves; }

  ULONG GetPatchCount() const                                      { return m_sBezMgr.GetNumActiveElements(); }

  SmBezierPatch * GetPatchAt
  (
    ULONG lUIndex,
    ULONG lVIndex,
    ULONG l1stGenerationUCount,
    ULONG l1stGenerationVCount
  ) const
  {
    SM_ASSERT( lUIndex < l1stGenerationUCount && lVIndex < l1stGenerationVCount );
    ULONG lIndex = lUIndex*l1stGenerationVCount + lVIndex;
    SM_ASSERT( lIndex < GetPatchCount() );
    return (SmBezierPatch*)((SmSurfaceCache*)this)->m_sBezMgr.GetAt( lIndex );
  }

  SmTree * GetTree();

  SmStatus GetCorners
  (
    gw_SURFACE       * pGWSurface,      ///< [in ]: optional Surface to check                                 <br>
    const SmExtent2d & crUVDomain,      ///< [in ]: surface subDomain                                         <br>
                                        ///<      : expected to be the natural domain when pSurface != NULL   <br>
    SmPoint3d        & rU0V0Point,      ///< [out]: Surface(u0, v0) corner 3d point                           <br>
    SmPoint3d        & rU1V0Point,      ///< [out]: Surface(u1, v0) corner 3d point                           <br>
    SmPoint3d        & rU1V1Point,      ///< [out]: Surface(u1, v1) corner 3d point                           <br>
    SmPoint3d        & rU0V1Point,      ///< [out]: Surface(u0, v1) corner 3d point                           <br>
    SmExtent2d       & rUVDomain        ///< [out]: surface domain for 4 corner points                        <br>
  );

  // return lowest continuity across any internal knot boundary in given domain
  SmContinuityType GetSurfaceContinuity(const SmExtent2d & crUVDomain);

  const SmTArray<SmContinuityType> & GetUContinuitiesArray() const { return *m_pUContinuities; }
  const SmTArray<SmContinuityType> & GetVContinuitiesArray() const { return *m_pVContinuities; }

  SmStatus GetIsoBoundaryCurves
  (
    const SmExtent2d          & crUVDomain,                ///< [in ]: target Surface UVDomain                                         <br>
    double                      d3DTolerance,              ///< [in ]: max dist between returned isoParameter Curves and Surface       <br>
    SmBoolean                 & rbCurvesAreCached,         ///< [out]: TRUE = returned curves will be deleted with surface cache       <br>
                                                           ///<      : FALSE= user should delete output curves after done using them.  <br>
    SmTArray<SmBSplineCurve*> & rIsoCurves                 ///< [out]: IsoParameter Curves bounding input crUVDomain                   <br>
                                                           ///<      : ordered: [0] - minimum const U curve   //       3               <br>
                                                           ///<      :          [1] - minimum const V curve   //       |               <br>
                                                           ///<      :          [2] - maximum const U curve   //     +----+            <br>
                                                           ///<      :          [3] - maximum const V curve   //     |    |            <br>
                                                           ///<      :                                        //   0-|    |-2          <br>
                                                           ///<      :                                        //     +----+            <br>
                                                           ///<      :                                        //       |               <br>
                                                           ///<      :                                        //       1               <br>
  );

  SmExtent2d GetUVDomain()                                         { return m_sUVDomain; }

  SmStatus SubdivideByBlock
  (
    SmTree     * pTree,                           ///< [in ]: Tree under construction    <br>
    SmTreeNode * pNode                            ///< [in ]: target node                <br>
  );

  SmStatus SubdivideNode
  (
    SmTreeNode      * pNode,                      ///< [in ]: target node                                                         <br>
    SmSurfParamType   eSubdivideDirection,        ///< [in ]: saved in pNode->m_pData->m_eSplitDir.                               <br>
                                                  ///<      : oneof SM_SP_U: Child1 = left, Child2 = right                        <br>
                                                  ///<      :       SM_SP_V: Child1 = bot,  Child2 = top                          <br>
    SmBoolean         bOnlySubdivideAuxData,      ///< [in ]: TRUE=no Bezier patch shape to split                                 <br>
                                                  ///<      : only set to TRUE when inserting edgeuses into the subdivision Tree  <br>
    SmMemBlockMgr   * pBezierBlock                ///< [in ]: scratch memory to hold Bezier patches during construction           <br>
  );

  SmStatus SubdivideToTolerances
  (
    SmTree        * pTree,                        ///< [in,out]: Tree to modify                                              <br>
    SmTreeNode    * pNode,                        ///< [in ]: candidate for subdivsion                                       <br>
    SmMemBlockMgr * pBezierBlock,                 ///< [in ]: memory for gw_SURFACEs                                         <br>
                                                  ///<      : note: pNode->m_eAuxDataType equals either                      <br>
                                                  ///<      :       SM_AD_BEZIER_SURFACE (m_pData = (SmBezierPatch *)) or    <br>
                                                  ///<      :       SM_AD_AUX_OBJECT     (m_pData = (SmBezierAux2d *))       <br>
    ULONG           lRecursionDepth = 0           // (for debugging)                                                         
  );
                                                                  
  // BuildTree helper functions
  SmStatus BuildTreeBase
  (
    SmTArray<SmTreeNode*> & rNodes,               ///< [out]: array of tree leaf nodes            <br>
    ULONG &rl1stGenerationUCount,                 ///< [out]: number of U entries in rNodes       <br>
    ULONG &rl1stGenerationVCount,                 ///< [out]: number of V entries in rNodes       <br>
    SmMemBlockMgr *pBezierBlock                   ///< [in ]: Bezier Surface Patch memory         <br>
  ) ;     

  SmStatus BuildTreeTop
  (
    SmTArray<SmTreeNode*> & rNodes,               ///< [in ]: array of 1st generation subdivision tree nodes   <br>
    ULONG &rl1stGenerationUCount,                 ///< [in ]: number of U entries in rNodes                    <br>
    ULONG &rl1stGenerationVCount                  ///< [in ]: number of V entries in rNodes                    <br>
  );
                        
  SmBrep          * GetBrep()    const ; 
  SmFace          * GetFace()    const ;

  const SmSurface * GetSurface() const           { return m_cpSurface; }

  SmStatus LookAhead
  (
    SmExtent2d        sUVDomain,          ///< [in ]: sub domain for this look        <br>
    ULONG             lMoveCnt,           ///< [in ]: number of moves to look ahead   <br>
    SmSurfParamType & eBestSplitDir,      ///< [out]: oneof of: SM_SP_U, SM_SP_V      <br>
    double          & dBestArea           ///< [out]:                                 <br>
  );

  virtual SmStatus TestAgainstTolerances
  (
    SmTreeNode      * pNode,                   ///< [in ]: node to test                                    <br>
    SmBoolean       & rbNeedsSubdivision,      ///< [out]: TRUE = patch fails some tessellation test       <br>
    SmSurfParamType & reSubdivisionDirection   ///< [out]: suggested direction to split bad elements       <br>
                                               ///<      : oneof: SM_SP_U: Child1 = left, Child2 = right   <br>
                                               ///<      :        SM_SP_V: Child1 = bot,  Child2 = top     <br>
  ); 
                                                                                    
                                                                  
  SmStatus ComputeSizeTolerances
  (
    const SmPoint3d & crU0V0Point,  ///< [in ]: 3d corner of Surface(u0, v0) of target quad                            <br>
    const SmPoint3d & crU1V0Point,  ///< [in ]: 3d corner of Surface(u1, v0) of target quad                            <br>
    const SmPoint3d & crU1V1Point,  ///< [in ]: 3d corner of Surface(u1, v1) of target quad                            <br>
    const SmPoint3d & crU0V1Point,  ///< [in ]: 3d corner of Surface(u0, v1) of target quad                            <br>
    SmVector2d & rAspectRatio,      ///< [out]: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max < dScaledZero, then 0.5   <br>
    SmVector2d & rMaxSideLength3D,  ///< [out]: [dMaxU, dMaxV] in 3d,                                                  <br>
    SmVector2d & rMinSideLength3D   ///< [out]: [dMinU, dMinV] in 3d, except Min < dScaledZero, then Max               <br>
  );

  SmStatus ComputeSizeTolerances
  ( 
    const SmSurface  * pSur,        ///< [in ]: Target Surface                                                         <br>
    const SmExtent2d & crDomain,    ///< [in ]: Target Surface sub UVDomain                                            <br>
    SmVector2d & rAspectRatio,      ///< [out]: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max < dScaledZero, then 0.5   <br>
    SmVector2d & rMaxSideLength3D,  ///< [out]: [dMaxU, dMaxV] in 3d,                                                  <br>
    SmVector2d & rMinSideLength3D   ///< [out]: [dMinU, dMinV] in 3d, except Min < dScaledZero, then Max               <br>
  );                                                                                                                

  // build surface spatial decomposition tree.
  //   Each leaf node contains a BezierPatch of original surface's sub-domain that meets tolerances
  //   Each parent node has 2 children
  //    When pParentNode->m_pData->m_eSplitDir = SM_SP_U: child1 = lower u domain half, left  side
  //                                                      child2 = upper u domain half, right side
  //                                             SM_SP_V: child1 = lower v domain half, bot   side
  //                                                      child2 = upper v domain half, top   side
  // always called immediately after SmSurfaceCache construction
  virtual SmStatus BuildTree(SmMemBlockMgr *pOptBezierBlock=NULL);

  // only called by SmTrimSrfCache::BuildTree (directly and through SubdivideNode)
  virtual SmStatus ImplantEdgeuse
  (
    SmEdgeuse * pEdgeuse,
    SmBoolean bDoSubdivision,
    SmExtent2d & rFaceDomain,
    SmTreeNode *pOptTreeNode = NULL
  )
  {
    SM_REF4(pEdgeuse, bDoSubdivision, rFaceDomain, pOptTreeNode) ;
    ERR( SM_ERR );
    return SM_ERR;
  }

  // only called by SmTrimSrfCache::BuildTree (directly and through SubdivideNode)
  virtual SmStatus ImplantPolyEdge(SmPolyEdge* pPolyEdge,
                                   SmTreeNode* pNode)
  {
    SM_REF2(pPolyEdge, pNode) ;
    ERR(SM_ERR);
    return SM_ERR;
  }

  SmBoolean HasCachedCurves
  (
    const SmContext    & crContext,                         ///< [in ]: context for constructor                                           <br>
    SmAltSrfCacheType    eSrfCacheType,                     ///< [in ]: oneof                                                             <br>
                                                            ///<      : SM_AS_SILHOUETTE         = contains silhouette curves trimmed     <br>
                                                            ///<      :                            to surface natural domain.             <br>
                                                            ///<      : SM_AS_TRIMMED_SILHOUETTE = contains silhouette curves trimmed     <br>
                                                            ///<      :                            to face boundaries.                    <br>
    double               dApproxTol,                        ///< [in ]: cache parameter to compare                                        <br>
    double               dAngleTol,                         ///< [in ]: cache parameter to compare                                        <br>
    const SmVector3d   * cpUserVec,                         ///< [in ]: cache parameter to compare                                        <br>
    const SmBoolean    * cpUserBool,                        ///< [in ]: cache parameter to compare                                        <br>
    const long         * cpUserLong,                        ///< [in ]: cache parameter to compare                                        <br>
    const double       * cpUserDouble,                      ///< [in ]: cache parameter to compare                                        <br>
    const void         * cpUserPointer,                     ///< [in ]: cache parameter to compare                                        <br>
    SmTArray<SmCurve*> * pCopyOf3DCurvesInCache,            ///< [out]: Optional Copies of silhouette 3D curves,                          <br>
                                                            ///<      : NULL to skip curve copying                                        <br>
    SmTArray<SmCurve*> * pCopyOfUVCurvesInCache = NULL      ///< [out]: Optional Copies of silhouette UVTrimCurves,                       <br>
                                                            ///<      : NULL to skip curve copying                                        <br>
  ) const;

  SmStatus AddCurvesToCache
  (
    SmAltSrfCacheType          eSrfCacheType,
    double                     dApproxTol,
    double                     dAngleTol,
    const SmVector3d         * cpUserVec,
    const SmBoolean          * cpUserBool,
    const long               * cpUserLong,
    const double             * cpUserDouble,
    const void               * cpUserPointer,
    const SmTArray<SmCurve*> * cp3DCurvesToCache,
    const SmTArray<SmCurve*> * cpUVCurvesToCache = NULL
  );

  SmPatchBoundaryBoundingPlanes & GetPatchBoundaryBoundingPlanes( void )
  {
    return m_sBoundingPlanes;
  }

  // utilities

  ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                          <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                      <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                  <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                            <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]<br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Draw decomposition BezierSurface patch outlines, 
  //      optional TreeNode bounding boxes,
  //  and optional Surface BoundaryPlanes
  SmDisplayList * Draw
  (
    SmBoolean bDrawTree=FALSE, 
    SmBoolean bDrawBoundingPlanes=FALSE,
    SmBoolean bOnlyDrawNodesWithVertices=FALSE,
    SmBoolean bOnlyDrawNodesWithEdges=FALSE,
    SmBoolean bOnlyDrawLeafNodes=FALSE
  ) const;

  // Draw UV Subdivision boundares in z=0 plane
  SmDisplayList * DrawSubdivision2D
  (
    SmBoolean       bDrawGeometry=FALSE,               ///< [in ]: TRUE= Also draw Face->UVTrimCurves and UVVertexPts on plane                      <br>
    SmBoolean       bOnlyDrawNodesWithVertices=FALSE,  ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face vertices                     <br>
    SmBoolean       bOnlyDrawNodesWithEdges=FALSE,     ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face edges                        <br>
    SmPlane       * pOptOutPlane=NULL,                 ///< [in ]: Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                       <br>
    SmGfxArraySet * pOptGfxSet=NULL                    ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.  <br>
  ) const;
    
  // Draw Subdivision block BBoxes in 3d                                                                        
  SmDisplayList * DrawSubdivision3D
  (
    SmBoolean       bOnlyDrawNodesWithVertices=FALSE, ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face vertices                   <br>
    SmBoolean       bOnlyDrawNodesWithEdges=FALSE,    ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face edges                      <br>
    SmGfxArraySet * pOptGfxSet=NULL                   ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.<br>
  ) const;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmSurfaceCache,SmObject,SmSurfaceCache_TYPE);            

} ; // end class SmSurfaceCache



#endif // __SMSURFACECACHE_H__

