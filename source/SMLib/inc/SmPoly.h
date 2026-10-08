// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPoly.h 
* PURPOSE: Header file for SmPoly* objects - used to represent polygons.
**********************************************************************/

#ifndef __SMPOLY_H__
#define __SMPOLY_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored to debug Linux builds

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMSAGOBJECT_H__
#include <SmSAGObject.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTOLERANCE_H__
#include <SmTol.h>
#endif

// forward declaration

class SmPolyFace ;
class SmRayTracer ;
class SmCopyPolyBrepMap ;
class SmCPolyFace; // needed for gcc4.x
class SmPlane;  // for graphics

/*******************************************************************//**
PURPOSE: This enum defines where a point lies relative to a polygon.

NOTES:
***********************************************************************/
enum SmPolyContainmentType 
{ 
  SM_PCT_UNKNOWN,
  SM_PCT_INSIDE,
  SM_PCT_OUTSIDE,
  SM_PCT_ON_BOUNDARY
};

/*******************************************************************//**
PURPOSE: This type defines what type of polygon file will be read.

NOTES: 
***********************************************************************/
enum SmPolygonFileType 
{ 
  SM_PF_RAW  // Raw triangle file
};

/*******************************************************************//**
PURPOSE: This type defines the vertex classifications.

NOTES: 
***********************************************************************/
enum SmPolyVertexType 
{ 
  SM_PV_UNKNOWN,    // We don't know what type of vertex this is yet
  SM_PV_3D_POINT,   // This vertex contains a 3D point - usually this
                    // is the Head vertex and there is only one of these in a
                    // vertex cluster.  
  SM_PV_UV_POINT,   // This vertex contains a 2D UV Point.  There can be more
                    // than one of these at a vertex cluster.  Ideally there
                    // should be one per original surface.  
  SM_PV_SURF_NORMAL // This vertex contains a 3D surface normal.  There can
                    // be more than one of these at a vertex cluster.  Ideally
                    // there should be one per original suface.
};

/*******************************************************************//**
PURPOSE: What type of a vertex is this?

NOTES: 
***********************************************************************/
enum SmPolyVertexClass 
{ 
  SM_PVC_NOT_COMPUTED = 0,   // Not yet computed
  SM_PVC_UNKNOWN      = 1,   // We don't know, vertex connected to spine edge or
                             //                vertex is nonManifold 
  SM_PVC_SIMPLE       = 2,   // Simple interior vertex, not connected to lamina, feature, or silhouette edges
  SM_PVC_BOUNDARY     = 3,   // Boundary of mesh - has lamina edges
  SM_PVC_INTERIOR     = 4,   // Interior point between two feature edges
  SM_PVC_CORNER       = 5    // Interior point between three or more feature edges
};

/*******************************************************************//**
PURPOSE: This class describes the position of a point in the grid
    and if the point was found in the grid.

NOTES: 
***********************************************************************/
class SM_EXPORT SmTessGridPoint
{
public:
  SmBoolean      m_bPointOnGrid;
  ULONG          m_lU;
  ULONG          m_lV;
  SmPoint3d      m_vPnt;
  SmPolyVertex * m_pPolyVertex;

  // constructor
  SmTessGridPoint() : m_bPointOnGrid(FALSE), 
                      m_lU(0), 
                      m_lV(0), 
                      m_pPolyVertex(NULL) 
                    { }

} ; // end class SmTessGridPoint

/*******************************************************************//**
PURPOSE: Interface class to allow both 3D and 2D version of 
   Tessellation to occur.  It can be used on 3D polygons and 2D polygons
   in the parameter space of a surface

NOTES: Base type for SmUVTessCallback
***********************************************************************/
class SM_EXPORT SmTessCallback
{
public:
  // constructor, destructor
  SmTessCallback() { }
  virtual ~SmTessCallback() { }

  // simple access
  virtual SmPoint3d  Get3DPoint   (const SmPoint3d & crInPoint) const { return crInPoint; }

  virtual double     Get3DLength  (const SmPoint3d & crStart, const SmPoint3d & crEnd) const 
  {  return Get3DPoint(crStart).DistanceBetween(Get3DPoint(crEnd)); }

  virtual SmVector3d GetNormal    (SmPolyFace *pPFace) const ;

  virtual ULONG      GetAge       (SmPolyEdge *)       const { return 0; }

  virtual ULONG      GetInitialAge(SmPolyEdge *)       const { return 0; }


  virtual SmPoint3d  PointAt3DDistInUV
  (
    const SmPoint3d  & crMid,     ///< [in] :                            <br>
    const SmVector3d & crBin,     ///< [in] : Assumes crBin is unitized  <br>
    double dQuadSize              ///< [in] :                            <br>
  )  const          
  { return crMid + dQuadSize * crBin; } 
                     
  virtual SmBoolean  IsEdgeShortEnough
  (
    const SmPoint3d &,           ///< [in] : crStartPt,           <br>
    const SmPoint3d &,           ///< [in] : crEndPt,             <br>
    double                       ///< [in] : dRatio               <br>
  )  const
  { return TRUE; }
                     
  virtual SmStatus Tweak
  (
    const SmPoint3d &,              ///< [in] : crStartPt,        <br>
    const SmPoint3d & crEndPt,      ///< [in] :                   <br>
    const SmPoint3d &,              ///< [in] : crOtherStartPt,   <br>
    const SmPoint3d &,              ///< [in] : crOtherEndPt,     <br>
    SmTessGridPoint & rNewEndPoint  ///< [out]:                   <br>
  )           
  { 
    rNewEndPoint.m_vPnt = crEndPt;    
    rNewEndPoint.m_bPointOnGrid = FALSE; 
    return SM_SUCCESS; 
  }
                     
  virtual SmStatus TweakQuad
  (
    const SmPoint3d       &,              ///< [in] : crStartPt,    <br>
    const SmTessGridPoint & crStartTop,   ///< [in] :               <br>
    const SmPoint3d       &,              ///< [in] : crEndPt,      <br>
    const SmTessGridPoint & crEndTop,     ///< [in] :               <br>
    SmPoint3d             & rNewStartTop, ///< [out]:               <br>
    SmPoint3d             & rNewEndTop    ///< [out]:               <br>
  )  
  { 
    rNewStartTop = crStartTop.m_vPnt; 
    rNewEndTop   = crEndTop.m_vPnt; 
    return SM_SUCCESS;  
  }

  virtual SmPoint3d  EdgeChopping
  (
    double             dQuadSize,         ///< [in] :        <br>
    double             dOrigSize,         ///< [in] :        <br>
    double             dLengthFactor,     ///< [in] :        <br>
    const SmPoint3d  & crStart,           ///< [in] :        <br>
    const SmVector3d & crEdgeVector,      ///< [in] :        <br>
    double           & rdFinalSize        ///< [in] :        <br>
  )   
  { 
    SmPoint3d sPoint;
    rdFinalSize = dOrigSize;
    while (TRUE) 
      {
        sPoint = PointAt3DDistInUV(crStart,crEdgeVector,rdFinalSize);
        if (IsEdgeShortEnough(crStart,sPoint,dLengthFactor)) 
          { break; }
        if (rdFinalSize < dQuadSize / 100.0) break;
        rdFinalSize = rdFinalSize / 2.0;
      }
    return sPoint;
  }
                     
  virtual SmPoint3d  FindPointNear
  (
    const SmPoint3d & crTest,           ///< [in] :
    ULONG           & rlUIndex,         ///< [in] :
    ULONG           & rlVIndex          ///< [in] :
  )  const
  { 
    rlUIndex = 0; 
    rlVIndex = 0; 
    return crTest; 
  }
                     
  virtual SmDisplayList * Draw(void) { return(NULL) ; }                
    
} ; // end class SmTessCallback

/*******************************************************************//**
PURPOSE: The following is the auxillary data at a vertex.  It contains
    normal information and a texture coordinate.

NOTES: This class is declared as final. Can't derive a new class from this
class without removing final.
***********************************************************************/
class SM_EXPORT SmPolyVertAuxData final : public SmObject
{
protected:
  ULONG      m_lNumUsers;   // Keeps track of the number of times it is referenced
  ULONG      m_lIndexValue; // a temp val - commonly set up and used to index into indexed arrays
public:
  SmVector3d m_vNormal;     // UnitNormal of original surface
  SmPoint2d  m_vUV;         // UV coordinate of original surface or texture coordinate

public:
  // constructor
  SmPolyVertAuxData() : m_lNumUsers(0),
                        m_lIndexValue(SM_UNDEF_ULONG),   // i.e. not yet assigned.
                        m_vNormal(0.0,0.0,0.0), 
                        m_vUV(0.0,0.0) 
                      { }

  // destructor - don't call destructor - call RemoveUser() ;

  // simple access
  ULONG GetNumUsers  () const            { return m_lNumUsers ; }

  ULONG GetIndexValue() const            { SM_ASSERT_BREAK(m_lIndexValue != SM_UNDEF_ULONG) ; return m_lIndexValue; }

  void  SetIndexValue(ULONG lIndexValue) { SM_ASSERT_BREAK(lIndexValue != SM_UNDEF_ULONG) ; m_lIndexValue = lIndexValue; }

  SmBoolean IsIndexValueInit()           { return(m_lIndexValue != SM_UNDEF_ULONG) ; }

  // UseCount management
  void  AddUser()                        { m_lNumUsers++; }
  void  RemoveUser()                     { m_lNumUsers--; DeleteIfNoUsers(); }

  // internal destructor - delete if m_lNumUsers == 0, when deleted, return NULL else return this
  SmPolyVertAuxData * DeleteIfNoUsers()
  {
    if(m_lNumUsers == 0)
    {
      m_lNumUsers = SM_UNDEF_ULONG;
      m_lIndexValue = SM_UNDEF_ULONG;
      m_vNormal.SetUninitialized();
      m_vUV.SetUninitialized();
      delete this;
      return NULL;
    }
    return this;
  }

} ; // end class SmPolyVertAuxData

/*******************************************************************//**
PURPOSE: Represents a polygon vertex.  

NOTES: A theoretical polygon vertex can consist of a linked list 
    of these polygon vertex objects with one of them being the head 
    vertex.  The types of information required depends on what type of
    these are kept around (see SmPolyVertexType).  It may contain
    a 3D vertex,

    This class is declared as final. Can't derive a new class from this
    class without removing final.
 ***********************************************************************/
class SM_EXPORT SmPolyVertex final : public SmTopology
{
  friend class SmPolyEdge;
  friend class SmPolyLoop;
  friend class SmPolyFace;
  friend class SmPolyShell;
  friend class SmPolyBrep;
  friend class SmTess;
  friend class SmPolyDecimate;
  friend class SmPolyMerge;

protected:
  // inherited:
  // SmTopology::m_pListOwner     -  SmPolyBrep::m_pPolyVertexListHead
  // SmTopology::m_pNext          -  linked list of SmPolyVertex's in SmPolyBrep's list
  // SmTopology::m_pLast          -  linked list of SmPolyVertex's in SmPolyBrep's list

  SmZoneTol3d             m_sZoneTol3d;
  SmPoint3d               m_vPoint;                 // This point may be a 3D point or a 2D UV point
  SmPolyEdge            * m_pPolyEdgeList = NULL;   // List head of edges emanating from this vertex.
  ULONG                   m_lIndexValue;            // a temp val - commonly set up and used to index into indexed arrays
  SmVertex              * m_pOriginalVertex = NULL; // back pointer to the original SmVertex that generated this PolyVertex

  // Coordinated lists. m_vNormals[ii] is the normal at m_vPoint on either m_vFaces[ii]->GetOriginalFace()->GetSurface() (if SmPolyFace)
  //                                                                    or m_vFaces[ii]->GetSurface()                    (if SmFace)
  // m_vFaces first set with SmPolyFace* during SmTess::OutputToPolyBrep.
  // m_vFaces ypdated to SmFace* in SmBrep::ConvertToPolyBrep, after PolyFace original faces are mapped to 'this' brep
  SmTArray<SmTopology*>   m_vFaces;         // List of PolyFaces or Faces connected to this PolyVertex
  SmTArray<SmVector3d>    m_vNormals;       // List of Normals that correspond to m_vFaces->Surfaces at m_vPoint
  SmTArray<ULONG>         m_vIndices;       // m_vIndices is a temp val used in coordinating mesh export

  public:
#ifdef USE_DEBUG_COUNTER
  ULONG          m_lDebugCount; // Used for debugging
#endif

public:
  // constructor:    SmPolyVertex *pPolyVertex = new (pPolyBrep) SmPolyVertex(args...) ;
  SmPolyVertex
  (
    const SmPoint3d & crVertexPoint,  ///< [in] :        <br>
    SmZoneTol3d       dZoneTol3d      ///< [in] :        <br>
  ) ;

  // destructor
  virtual ~SmPolyVertex() ;

  // edit PolyEdge linked list
  void         AddPolyEdge    (SmPolyEdge *pEdgeToStartAtVertex);

  SmStatus     RemovePolyEdge (SmPolyEdge *pEdgeStartingAtThisVertex) ;

  // simple access         
  SmPolyBrep * GetPolyBrep()              const ;

  SmPoint3d    GetPoint()                 const { return m_vPoint; }

  SmPoint3d  & GetPoint()                       { return m_vPoint; }

  SmZoneTol3d  GetTolerance()             const { return m_sZoneTol3d; }

  SmEdgeuse  * GetOriginalEdgeuse()       const ;  // rtn: 1st OriginalEdgeuse stored on any connected PolyEdge

  SmEdgeuse  * GetEdgeuse()               const { return(GetOriginalEdgeuse()) ; }  // for backward compatibility

  SmVertex   * GetOriginalVertex()        const { return m_pOriginalVertex; }

  SmVertex   * GetVertex()                const { return(GetOriginalVertex()) ; } // for backward compatibility

  SmBoolean    GetOKBackPtrs()            const ;
                                         
  ULONG        GetIndexValue()            const { SM_ASSERT_BREAK( m_lIndexValue != SM_UNDEF_ULONG ); return m_lIndexValue; }

  void         GetPolyFaces               (SmTArray<SmPolyFace*> & rFaces) const; // PolyFaces connected to PolyVertex

  void         GetOrderedPolyFaces        (SmTArray<SmPolyFace*> & rFaces) const; // CCW ordered PolyFaces connected to PolyVertex
                                                                                  // watch out for clusters
                                            
  void         GetPolyEdges               (SmTArray<SmPolyEdge*> & rEdges) const; // starting and ending PolyEdges

  void         GetStartingPolyEdges       (SmTArray<SmPolyEdge*> & rEdges) const; // starting PolyEdges 

  void         GetPolyEdgesOfFace         (const SmPolyFace      * crPolyFace, SmTArray<SmPolyEdge*> & rEdges ) const;    // starting PolyEdges connected to PolyFace
                                           
  void         GetOrderedStartingPolyEdges(SmTArray<SmPolyEdge*> & rEdges) const; // starting PolyEdges in CCW order

  SmPolyEdge * GetFirstPolyEdge           ()                               const  { return m_pPolyEdgeList; }

  SmPolyEdge * GetCWPolyEdge              (SmPolyEdge            * pEdge)  const; 

  SmPolyEdge * GetCCWPolyEdge             (SmPolyEdge            * pEdge)  const;

  void         GetAdjacentPolyVertices        (SmTArray<SmPolyVertex*> & rVertices) const; // PolyVertices connected to this through a PolyEdge
  
  void         GetOrderedAdjacentPolyVertices (SmTArray<SmPolyVertex*> & rVertices) const; // CCW ordered PolyVertices connected to this through a PolyEdge

  // Accessors to Face / Normal / Index lists
  SmTArray<SmTopology*> & GetFacesRef() { return m_vFaces; }

  SmTArray<SmVector3d>  & GetNormalsRef() { return m_vNormals; }

  SmTArray<ULONG>       & GetIndicesRef() { return m_vIndices; }

  void                    AddFaceToList(SmTopology * pFace) { m_vFaces.Add(pFace); };

  void                    AddFaceNormal(const SmVector3d & crNormal) { m_vNormals.Add(crNormal); };

  void                    AddFaceIndex(const ULONG& crIndex) { m_vIndices.Add(crIndex); }

  // internal use only - these use temp UserIndex2 values set up by other methods
  SmPolyEdge * GetFirstUnmarkedEdgeInCluster(const SmTArray<SmPolyEdge *> & rEdges)    const;
  
  void         AddEdgeCluster (SmPolyEdge * firstClusterEdge, SmTArray<SmPolyEdge *> & rEdges) const;

  // simple access
  void SetTolerance     (SmZoneTol3d sZoneTol3d,                    // in :
                         SmBoolean   bUpdateOnlyIfLarger = TRUE) ;  // NotUsed: in :

  void SetPoint         (const SmPoint3d & crNewPoint);

  void SetOriginalVertex
  (
    SmVertex * pOriginalVertex,          ///< [in] : BrepVertex origin for this PolyVertex                         <br>
    SmBoolean  bPropagateAttribs=FALSE   ///< [in] : TRUE  = Copy Original BrepVertexAttribs to this PolyVertex    <br>
                                         //      FALSE = don't, default:[FALSE]                                    <br>
  );                                                    

  void SetIndexValue( ULONG lIndexValue )
  {
    SM_ASSERT_BREAK( lIndexValue != SM_UNDEF_ULONG );
    m_lIndexValue = lIndexValue;
  }

  void UnInitIndexValue() { m_lIndexValue = SM_UNDEF_ULONG; }

  SmStatus     FindPolyEdgeTowards        
  (
    SmPolyVertex           * pOtherVertex,            ///< [in] :   <br>
    SmPolyEdge            *& rpEdgeBetweenVertices    ///< [out]:   <br>
  ) const;

  SmStatus     FindPolyEdgeBetween        
  (
    SmPolyVertex           * pOtherVertex,            ///< [in] : 2nd PolyEdge bounding Vertex                                                      <br>
    SmPolyEdge            *& rpEdgeBetweenVertices    ///< [out]: 1st PolyEdge found between PolyVerts:[this pOtherVertex] or NULL for none found   <br>
  ) const;

  SmStatus     FindPolyEdgesBetween       
  (
    SmPolyVertex           * pOtherVertex,      ///< [in] : other vertex target                                     <br>
    SmTArray<SmPolyEdge*>  & rEdgesBetween      ///< [out]: list of all PolyEdges between This and Other vertex     <br>
  ) const;

  SmStatus     FindPolyEdgeOrFaceBetween  
  (
    SmPolyVertex           * pOtherVertex,            ///< [in] :      <br>
    SmPolyEdge            *& rpEdgeBetweenVertices,   ///< [out]:      <br>
    SmPolyFace            *& rpFaceBetweenVertices    ///< [out]:      <br>
  ) const;

  SmStatus     FindPolyFaceSharingVertices
  (
    SmPolyVertex           * pVertex1,                ///< [in] :     <br>
    SmPolyVertex           * pVertex2,                ///< [in] :     <br>
    SmPolyFace            *& rpFaceWithVertices       ///< [out]:     <br>
  )    const;
                 
  // predicates
  SmBoolean IsIndexValueInit()            { return(m_lIndexValue != SM_UNDEF_ULONG) ; }

  SmBoolean IsBoundaryVertex()    const ; //

  SmBoolean IsSingleVertexLoop()  const ; // TRUE = FirstPolyEdgeAtVertex is a OnePolyEdgeLong PolyLoop

  SmBoolean IsTopologicalVertex() const ; // TRUE = single edge vertex loop

  SmBoolean IsLaminaVertex()      const ; // TRUE = any connected PolyEdge is lamina

  SmBoolean IsSpineVertex()       const ; // TRUE = any connected PolyEdge is Spine

  SmBoolean IsManifoldVertex()    const ; // TRUE = all connected PolyEdges are Manifold

  SmBoolean Is2d()                const { return m_vPoint.Is2d() ; }  // sometimes Z=0.0 for 2d which will be missed.

  SmBoolean Is2dOrZero()          const { return m_vPoint.Is2d() || m_vPoint.z == 0.0 ; }       

  SmStatus  CalculateBoundingBox (SmExtent3d & rEdgeBBox) const; // Expanded by 'this' tolerance

  // overloaded new and delete - allocate SmPolyVertex memory from SmPolyBrep::m_VMgr SmMemBlockMgr memory
  void *operator new (size_t size, SmPolyBrep * pObjectToGetContext);   // NotUsed: in : size
  void  operator delete(void* ptr) { SM_REF1(ptr); }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  // utilities
  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
  )  const;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  SmDisplayList   * Draw
  (
    SmBoolean       bDrawRaw=TRUE,           ///< [in] : TRUE = draw PolyLoop->PolyEdge->StartPts
    SmBoolean       bDrawIn3d=FALSE,         ///< [in] : TRUE = draw OrigSurf(PolyLoop->PolyEdge->StartPts)                                             <br>
    SmPlane       * pOptOutPlane=NULL,       ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane                                <br>
    SmGfxArraySet * pOptGfxSet=NULL          ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore.<br>
  ) const ; 
                                                                 

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyVertex,SmTopology,SmPolyVertex_TYPE);                                    

} ; // end class SmPolyVertex

/*******************************************************************//**
PURPOSE: Represents a polygon edge.

NOTES: see SmPolyBrep Model Topology and Shape notes for overview.
                                     
       The shape of every PolyEdge is defined by the
       PolyEdge's participation in the owning PolyLoop's doubly linked list
       of ordered PolyEdges as the line:[GetStartPoint(), GetEndPoint()]
                                        parameterized from 0.0 to 1.0
         where: GetStartPoint()    = PolyEdge->m_pOriginalVertex->m_pPoint
                GetEndPoint()      = GetEndPolyVertex()->m_pPoint
                GetEndPolyVertex() = either GetCCWEdge()->m_pOriginalVertex->m_pPoint
                                         or GetPolyLoop()->m_pLastEndPolyVertex
                                     as determined by the PolyLoop state and 
                                     the position of PolyEdge in the PolyLoop edge list.

       A PolyLoop's double linked PolyEdge list is ordered as a 
       CCW walk around the owning PolyFace's polygon. 
       That sequence defines the positive normal of the PolyFace
       so that an observer oriented in the positive normal direction
       on a polygon face walking in the direction of PolyEdge
       will have the interior of the polygon on his left hand side.

       GetCCWPolyEdge() = next edge in the PolyLoop's ordered PolyEdge list.
       GetCWPolyEdge()  = last edge in the PolyLoop's ordered PolyEdge list.

       This class is declared as final. Can't derive a new class from this
       class without removing final.
        
***********************************************************************/
class SM_EXPORT SmPolyEdge final : public SmTopology
{
  friend class SmPolyVertex;
  friend class SmPolyLoop;
  friend class SmPolyFace;
  friend class SmPolyShell;
  friend class SmPolyBrep;
  friend class SmTess;
  friend class SmPolyDecimate;

protected:
  // inherited:
  //      SmTopology::m_pListOwner      -  ptr to SmPolyLoop
  //      SmTopology::m_pNext           -  Next (CCW) SmPolyEdge in this SmPolyLoop
  //      SmTopology::m_pLast           -  Prev ( CW) SmPolyEdge in this SmPolyLoop

  SmPolyEdge        * m_pNextRadialE;     // Like SmEdgeuses around the same SmEdge.
  SmPolyEdge        * m_pLastRadialE;     //  Lamina if only one: m_pNextRadialE == this.
                                          //  Manifold if exactly two.
                                          // Note that m_pPolyVertex->m_vPoint may be a 2D or 3D value.
                                          //  RadialE's have to have matching m_pPolyVertex->m_vPoints, so a PolyEdge 
                                          //  on a seam in 2D is not the RadialE of a PolyEdge on the other side of the seam. 
                                          //  But a Strut PolyEdge in 2D does have a RadialE.
  SmZoneTol3d         m_sZoneTol3d;     

  // The following fields correspond to a 'Vertexuse' at the start of the edge.
  SmPolyEdge        * m_pNextEdgeAtV;     // Circular Linked list of edges emanating from this vertex.
  SmPolyEdge        * m_pPrevEdgeAtV;     //  Often a member of this list will share a common PolyLoop
                                          //    with one of the radial partners of its neighbor members
                                          //    representing a vertex sector that is within some PolyFace.
                                          //  However, when working with multi-VertexClusters and lamina PolyEdges
                                          //    it is possible to find consecutive members of this list with
                                          //    no VertexSector between them.
  
  //   consecutive members usually have radial partners that bound a corner sector in a PolyLoop, however
                                          //   when working with multi-VertexClusters consecutive members don't always have
                                          //   to have radial partners be part of a common PolyLoop
                                          
  SmPolyVertex      * m_pPolyVertex;      // Start PolyVertex for this PolyEdge
                                          // note: PolyEdges are lines that end at the next CCW PolyEdge->StartPoint in the owning PolyLoop.
                                          //       PolyEdgeShape        = PolyLine[GetStartPoint(), GetEndPoint()]
                                          //       GetStartPoint()      = GetStartPolyVertex()->m_vPoint
                                          //       GetEndPoint()        = GetEndPolyVertex()->m_vPoint
                                          //       GetStartPolyVertex() = PolyEdge->m_vPolyVertex
                                          //       GetEndPolyVertex()   = from PolyLoop data, either PolyEdge->GetCCWPolyEdge()->GetStartPolyVertex() 
                                          //                                                      or PolyLoop->m_pLastEndPolyVertex
                                          // The PolyEdge->PolyLine moves in the CCW direction around the PolyFace enclosed by this PolyEdge->PolyLoop
                                          
  SmPolyVertAuxData * m_pAuxData;         // stores a normal and a UV value for the associated m_pPolyVertex
                                          
  SmEdgeuse         * m_pOriginalEdgeuse; // in SmTess, backpointer to SmBrep object being tessellated by this edge
  ULONG               m_lIndexValue;      // a temp val - commonly set up and used to index into indexed arrays

public:
#ifdef USE_DEBUG_COUNTER
  ULONG               m_lDebugCount; // Used for debugging
#endif

  SmBoolean           m_bVisibility;    // Visibility flag
  ULONG               m_lUserLong1;
  ULONG               m_lUserLong2;

public:
  // constructor:    SmPolyEdge *pPolyEdge = new (pPolyBrep) SmPolyEdge(dZoneTol3d) ;
  SmPolyEdge(SmZoneTol3d dZoneTol3d);

  // destructor
  virtual ~SmPolyEdge();

  // measures
  double     Length              () const                        { return GetStartPoint().DistanceBetween(GetEndPoint()); }

  SmStatus   CalculateBoundingBox(SmExtent3d & rEdgeBBox) const; // Expanded by Vertices' tolerances

  SmVector3d ComputeBinormal     () const;                       // vec perp to PolyEdge pointing into PolyFace

  double     ComputeSectorAngle  () const;                       // dihedral [0, 2Pi] AngRad from this->PolyFace to CCW radial partner

  SmStatus   ComputeAspectRatio  (double & rdAspectRatio) const; // Height/longestEdge of TRI[StartPt, EndPt, CCWEdge->StartPt]
                                                                 // GWC: odd that a PolyEdge returns a Polygon value 
                                                                 //      Perhaps this should be rewritten as a PolyFace method
                                                                 //      needs review
  // simple access
  SmPolyBrep        * GetPolyBrep        () const ;
  SmPolyFace        * GetPolyFace        () const ;
  SmPolyLoop        * GetPolyLoop        () const ;
                                         
  SmEdgeuse         * GetOriginalEdgeuse () const { return m_pOriginalEdgeuse; }
  SmEdgeuse         * GetEdgeuse         () const { return(GetOriginalEdgeuse()) ; } // for backward compatibility
  SmBoolean           GetOKBackPtrs      () const ;
                                         
  SmZoneTol3d         GetTolerance       () const { return m_sZoneTol3d; }
  ULONG               GetIndexValue      () const { SM_ASSERT_BREAK(m_lIndexValue != SM_UNDEF_ULONG) ; return m_lIndexValue; }
  SmPolyVertAuxData * GetAuxData         () const { return m_pAuxData; }
                                         
  SmPoint3d           GetStartPoint      () const { return m_pPolyVertex->m_vPoint ; }
  SmPoint3d           GetEndPoint        () const { return (GetEndPolyVertex()->m_vPoint) ; }
  SmVector3d          GetVec             () const { return (GetEndPolyVertex()->m_vPoint - m_pPolyVertex->m_vPoint) ; }
  SmStatus            GetEnds            (SmPoint3d & rStart,     SmPoint3d & rEnd)        const ;
  SmStatus            GetLine            (SmPoint3d & rLinePoint, SmPoint3d & rLineVector) const ;
  double              GetLength          () const { return Length() ; }
  double              GetLengthSq        () const { return GetStartPoint().DistanceBetweenSquared(GetEndPoint()); }
  double              GetDihedralAngRad  (SmVector3d *pOptDiAngColor=NULL) const ; // rtn: [0-2Pi] AngRad to Radial partner & 2Pi when lamina
                    
  SmPolyVertex      * GetStartPolyVertex () const { return m_pPolyVertex; }
  SmPolyVertex      * GetEndPolyVertex   () const ; // note: EndVertex = NextEdgeInLoop->StartVertex except for last edges on open Loops 
  SmPolyVertex      * GetOtherPolyVertex (const SmPolyVertex *cpVertex) const { return (cpVertex != m_pPolyVertex) ? m_pPolyVertex : GetEndPolyVertex() ; }
                                         
  SmPolyEdge        * GetCCWPolyEdge     () const { return (SmPolyEdge*)m_pNext ; }  // PolyLoop CCW Neighbor
  SmPolyEdge        * GetCWPolyEdge      () const { return (SmPolyEdge*)m_pLast ; }  // PolyLoop CW  Neighbor
  SmPolyEdge        * GetCCWNonDegeneratePolyEdge() const ;  // Next NonDegenerate PolyLoop CCW Neighbor
  SmPolyEdge        * GetCWNonDegeneratePolyEdge () const ;  // Next NonDegenerate PolyLoop CW  Neighbor
  SmPolyEdge        * GetNextPolyEdgeAtV () const { return m_pNextEdgeAtV; }         // Vertex Neighbor (usually CCW neighbor except for multiple cluster cases)
  SmPolyEdge        * GetRadial() const
  {
    return  (m_pNextRadialE != this) // Radial CCW Neighbor
      ? m_pNextRadialE               // GWC: shouldn't it return itself for lamina case
      : NULL;
  }

  void         GetAllRadials       (SmTArray<SmPolyEdge*> & rEdges) const;     // Radial Neighbors including this (1 per PolyFace, unlike Breps)
  SmPolyEdge * GetSymmetricPolyEdge() const ;                                 // Find PolyEdge:[EndVertex, StartVertex] through vertex connections

  // Find side of pRadialEdge->PolyFace that bounds sector starting at <this->PolyFace, eThisSide>
  SmStatus     GetTopologicalRadial
  (
    SmOrientType    eThisSide,   ///< [in] : oneof SM_OT_SAME     = FaceSide in Normal direction                       <br>
                                 ///<      :       SM_OT_OPPOSITE = FaceSide opposite Normal direction                 <br>
    SmPolyEdge   *& pRadialEdge, ///< [out]: Appropriate Radial neighbor bounding same radial sector                   <br>
    SmOrientType  & eRadialSide, ///< [out]: Orientation of radial neighbor pointing into same radial sector as input  <br>
    double        & rdAngle      ///< [out]: dihedral angle across radial sector                                       <br>
  ) const;       
                    
  // backward compatibility
  void AddAuxData        (SmPolyVertAuxData *pAuxData) { SetAuxData(pAuxData) ; }
    
  // simple access
  void SetTolerance      (SmZoneTol3d dTol )            { m_sZoneTol3d = dTol; }
  void SetOriginalEdgeuse(SmEdgeuse *pOriginalEdgeuse, SmBoolean  bPropagateAttribs=FALSE) ;
  void SetIndexValue     (ULONG lIndexValue)           { SM_ASSERT_BREAK(lIndexValue != SM_UNDEF_ULONG) ; m_lIndexValue = lIndexValue; }
  void SetAuxData        (SmPolyVertAuxData *pAuxData)
  {
    if(m_pAuxData == pAuxData) return;
    SmPolyVertAuxData *pAuxDataOld = m_pAuxData;
    m_pAuxData = pAuxData;
    if(pAuxData) { pAuxData->AddUser(); }
    if(pAuxDataOld) { pAuxDataOld->RemoveUser(); }
  }

  // predicates
  // GWC: I don't think PolyTopology supports wire edges - PLoops are always in PFaces, PEdges are always in PFaces
  SmBoolean IsLamina             () const ; // NextRadialPolyEdge == ThisPolyEdge
  SmBoolean IsManifold           () const ; // NextRadialPolyEdge->NextRadialPolyEdge == ThisPolyEdge
  SmBoolean IsSpine              () const ; // NextRadialPolyEdge->NextRadialPolyEdge != ThisPolyEdge
  SmBoolean IsStrut              () const ; // Manifold && CWPolyEdge == ThisPolyEdge
  SmBoolean IsBoundary           () const { return(GetSymmetricPolyEdge() == NULL); }
  SmBoolean Is2d                 () const { return m_pPolyVertex->Is2d() ; }   // sometimes Z=0.0 for 2d which will be missed.      
  SmBoolean Is2dOrZero           () const; 
  SmBoolean IsDegenerate         (SmXSectTol3d * pOptXSectTol3d = NULL) const ; // return TRUE = PolyEdge->Length < ScaledZero, gwc: that's very tight - maybe another tolerance
  SmBoolean IsIndexValueInit     () const { return(m_lIndexValue != SM_UNDEF_ULONG) ; }
  SmBoolean IsLocalMinimaOrMaxima() const;
  SmBoolean IsRadialPartnerOf    (const SmPolyEdge * pOther ) const;
  SmBoolean IsConcaveCorner      (const SmVector3d & crPolyNormal)  const;
  SmBoolean IsConvexCorner       
  (
    const SmVector3d & crPolyNormal,              ///< [in] : NormalVec of PolyEdge->PolyLoop->PolyFace owner                                   <br>
    double           & rdAngleDeg,                ///< [out]: inside AngleDeg at this vertex in PolyEdge->PolyLoop                              <br>
                                                  ///<      : normally in UV Space, but in 3D space when corner is on Surface Pole              <br>
    double           & rdChordLength,             ///< [out]: UVDist:[PrevVertPt, NextVertPt]  in PolyEdge->PolyLoop                            <br>
    double           & rdAngleDeg3d,              ///< [out]: only used when PolyEdge->PolyFace->OriginalFace exists, corner angle in 3d space  <br>
                                                  ///<      : otherwise set to 0.0                                                              <br>
    SmPolyEdge       * pOptCWPolyEdge = NULL      ///< [in] : PolyEdge to use as CWPolyEdge to This PolyEdge                                    <br>
                                                  ///<      : used to skip over Degenerate PolyEdge neighborss NULL to ignore.                  <br>
  ) const;

  SmBoolean Is3DConvexCorner     
  (
    double           & rdAngleDeg,                ///< [out]: inside angle of PEdge->Start corner in PEdge->PolyLoop                         <br>
    SmPolyEdge       * pOptCWPolyEdge = NULL      ///< [in] : PolyEdge to use as CWPolyEdge to This PolyEdge.                                <br>
                                                  ///<      : Used to skip over Degenerate PolyEdge neighbors NULL to ignore. default:[NULL] <br>
  ) const;
               

  SmBoolean IsCoincidentWith     
  (
    SmZoneTol3d    dZoneTol3d,               ///< NotUsed: [in] :                                          <br>
    SmPolyEdge   * pOther,                   ///< [in] : target edge to compare                                            <br>
    SmOrientType & reOrient,                 ///< [out]: coincident edge orientations, oneof: SM_OT_SAME, SM_OT_OPPOSITE   <br>
    double       * dOptDeviation = NULL      ///< [out]: max deviation between coincident edge end-points, NULL=ignore     <br>
  ) const;

  // Intersect two UV segments 
  SmStatus IntersectSegment
  (
    const SmPoint3d & crStart,                     ///< [in] :     <br>
    const SmPoint3d & crEnd,                       ///< [in] :     <br>
    SmZoneTol3d       dZoneTol3d,                  ///< [in] :     <br>
    ULONG           & rlNumIntersections,          ///< [out]:     <br>
    double            aParametersThis[2],          ///< [out]:     <br>
    double            aParametersOther[2],         ///< [out]:     <br>
    double            aDeviations[2]               ///< [out]:     <br>
  ) const;

  // Drop and Clamp point to the nearest PolyEdge point
  SmStatus DropPoint
  (
    const SmPoint3d & crPoint,                   ///< [in] : TestPoint                                             <br>
    double          & rdParameter,               ///< [out]: 0.0 to 1.0, exterior ProjPts are clamped to endPts.   <br>
    double          & rdDistance                 ///< [out]: TestPoint/PolyEdgePoint distance                      <br>
  ) const;

  // evaluate PolyEdge for [0,1] normalizedParameter
  SmStatus EvaluatePoint
  (
    double      dNormalizedParameter,            ///< [in] : a value from 0.0 to 1.0                <br>
    SmPoint3d & rPoint3d                         ///< [out]: set to Uninitialized for bad params    <br>
  ) const;
                     
  // internal func: Make this a lamina edge (i.e. detach it from its radials.)
  void     MakeLamina();

  // Replace a PolyEdge in a PolyBrep with 'this'
  SmStatus ReplacePolyEdge( SmPolyEdge * pOther );

  // Swap an edge with its 'm_pNextRadialEdge' element in the linked list.
  SmStatus SwapWithNext();

  // overloaded new and delete - allocate SmPolyEdge memory from SmPolyBrep::m_EMgr SmMemBlockMgr memory
  void *operator new (size_t size, SmPolyBrep * pObjectToGetContext);   // NotUsed: in : size
  void  operator delete(void* ptr) { SM_REF1(ptr); }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  SmDisplayList * Draw
  (
    SmBoolean       bDrawRaw=TRUE,           ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                        <br>
    SmBoolean       bDrawIn3d=FALSE,         ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf, default:[FALSE]   <br>
    SmPlane       * pOptOutPlane=NULL,       ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]       <br>
    SmGfxArraySet * pOptGfxSet=NULL          ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.       <br>
  ) const ; 
   

  SmDisplayList * Draw3D(SmGfxArraySet *pOptGfxSet=NULL) const    { return( Draw(FALSE, TRUE, NULL, pOptGfxSet) ) ; }

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                          <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                      <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                  <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                            <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]<br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyEdge,SmTopology,SmPolyEdge_TYPE);
                      
}; // end class SmPolyEdge

/*******************************************************************//**
PURPOSE: Represents a polygon loop.

NOTES: This class is declared as final. Can't derive a new class from this
class without removing final.
***********************************************************************/
class SM_EXPORT SmPolyLoop final : public SmOwningTopology
{
  friend class SmPolyVertex;
  friend class SmPolyEdge;
  friend class SmPolyFace;
  friend class SmPolyShell;
  friend class SmPolyBrep;
  friend class SmPolyDecimate;
  friend class SmTess;
protected:
  // inherited:
  // SmTopology::      m_pListOwner           -  SmPolyFace
  // SmTopology::      m_pNext, m_pLast       -  linked list of SmPolyLoops in SmPolyFace
  // SmOwningTopology::m_pList                -  head of doubly linked list of SmPolyEdge
  // SmOwningTopology::m_lListSize            -  number of SmPolyEdges in doubly linked list 
                                              
  SmOrientType         m_eOrientation;        // SM_OT_SAME = outer loop,  SM_OT_OPPOSITE = inner loop
  SmBoolean            m_bClosed;             // TRUE  = closed loop, 
                                              // FALSE = open loop
  SmPoint3d            m_sLastEndPoint;       // a scratch val - last endPoint  in loop - used only in AddPolyEdge() Loop construction
  SmPolyVertex        *m_pLastEndPolyVertex;  // a scratch val - last endVertex in loop - used only in AddPolyEdge() Loop construction   

public:
  // constructor:    SmPolyLoop *pPolyLoop = new (pPolyBrep) SmPolyLoop(pPolyFace) ;
  SmPolyLoop(SmPolyFace * pPolyFace = NULL);

  // assignment operator
  SmPolyLoop & operator=( const SmPolyLoop & crOther );

  // make PolyLoop from list of 3D points
  static SmStatus MakeFromLineSegments
  (
    SmTArray<SmPoint3d> & rLineSegPnts,
    SmZoneTol3d           dZoneTol3d,
    SmPolyBrep          * pPolyBrep,
    SmPolyLoop         *& rpPolyLoop
  );

  // Make PolyLoop from sequence of Line EndPoints
  //   see: SmPolyFace::StartPolyEdgeLoop()    - called once                                                  
  //        SmPolyLoop::AddPolyEdge()          - called once per line segment
  //        SmPolyLoop::FinishPolyEdgeLoop()   - called once

  // create SmPolyEdge added to ordered PolyEdge list. 
  SmStatus AddPolyEdge
  (
    SmZoneTol3d                dZoneTol3d,                  ///< [in] : min dist between distinct points  
    const SmPoint3d          & crStartPoint,                ///< [in] : Line start position                                                                         <br>
    const SmPoint3d          & crEndPoint,                  ///< [in] : Line end position                                                                           <br>
    SmPolyVertex             * pOptStartPolyVertex,         ///< [in] : when m_pLastEndPolyVertex NotNULL (set on last call through pOptEndPolyVertex),             <br>
                                                            ///<      : m_pLastEndPolyVertex is Start PolyVertex for PolyLoop->PolyEdge                             <br>
                                                            ///<      : else: pOptStartPolyVertex NotNULL = Start PolyVertex for PolyLoop->PolyEdge,                <br>
                                                            ///<      :                           NULL    = create New PolyVertex for 1stPolyEdge                   <br>
    SmPolyVertex             * pOptEndPolyVertex,           ///< [in] : NotNULL = stored in m_pLastEndPolyVertex to be                                              <br>
                                                            ///<      : Start PolyVertex for next AddPolyEdge() call.                                               <br>
    SmPolyBrep               * pOptBrep,                    ///< [in] : provides context for new obj construction and                                               <br>
                                                            ///<      : accumulates new PolyVertices on its m_pVertexListHead list                                  <br>
    SmPolyEdge              *& rpNewEdge,                   ///< [out]: new edge, stitched to radial partners when pOptStartPolyVertex and pOptEndPolyVertex are NotNULL<br>
    SmPolyVertex            ** pOptNewStartVertex=NULL,     ///< [out]: when given, set to Start PolyVertex for NewEdge (always NonNULL)                            <br>
    SmPolyVertex            ** pOptNewEndVertex=NULL,       ///< [out]: when given, set to End   PolyVertex for NewEdge (NULL except when pOptEndVertex != NULL)    <br>
    SmVertex                 * pOptStartVertex=NULL,        ///< [in] : Opt StartVertex to store as OriginalVertex for NewStartPolyVertex                           <br>
    SmVertex                 * pOptEndVertex=NULL,          ///< [in] : Opt EndVertex   to store as OriginalVertex for NewEndPolyVertex                             <br>
    SmTArray<SmPolyVertex*>  * pDeletedPolyVertices = NULL, ///< [out]: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]<br>
    SmTArray<SmPolyVertex*>  * pSurvivingVertices = NULL    ///< [out]: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]    <br>
  ) ;                                     
                 
  // Finish closed PolyEdge list after all AddPolyEdge() calls. 
  SmStatus FinishPolyEdgeLoop(SmPolyBrep * pOptBrep=NULL); // NotUsed: in : pOptBrep

  // replace pOther with this PolyLoop in pOther->PolyBrep (pOther removed from PolyBrep, not deleted)
  SmStatus ReplacePolyLoop( SmPolyLoop * pOther );

  // Split PolyLoop: origLoop left with PolyEdges before pSplitAtPolyEdge, NewLoop
  SmStatus SplitPolyLoop
  (
    SmPolyEdge  * pSplitAtPolyEdge,  ///< [in] : First PolyEdge to place in second child    <br>
    SmPolyLoop *& pNewPolyLoop       ///< [in] : New PolyLoop, NULL on input                <br>
  ) ;    

  SmStatus RemoveLastPolyEdge() ;

  // destructor
  virtual ~SmPolyLoop()
  {
    m_eOrientation = SM_OT_UNKNOWN;
    m_bClosed = FALSE;
    m_sLastEndPoint.SetUninitialized();
    m_pLastEndPolyVertex = NULL;
  }

  // properties                                                      
  SmStatus CalculateBoundingBox(SmExtent3d & rLoopBBox) const;

  SmStatus ContainsPoint
  (
    const SmPoint3d        & crPointToTest,       ///< [in] :               <br>
    SmPolyContainmentType  & rbInside,            ///< [out]: the result    <br>
    double                   dTol  = SM_EFF_ZERO, ///< [in] : opt           <br>
    SmBoolean                bIn3D = FALSE        ///< [in] : opt           <br>
  ) const;

  SmStatus ContainsPolyLoop
  (
    SmPolyLoop             * pPolyLoopToTest,     ///< [in] :              <br>
    SmPolyContainmentType  & rbInside,            ///< [out]: the result   <br>
    double                   dTol  = SM_EFF_ZERO, ///< [in] : opt          <br>
    SmBoolean                bIn3D = FALSE        ///< [in] : opt          <br>
  ) const;

  SmStatus OverlapsPolyLoop
  (
    SmPolyLoop             * pPolyLoopToTest,     ///< [in] :               <br>
    SmPolyContainmentType  & rbInside,            ///< [out]: the result    <br>
    double                   dTol  = SM_EFF_ZERO, ///< [in] : opt           <br>
    SmBoolean                bIn3D = FALSE        ///< [in] : opt           <br>
  ) const;

  SmStatus ComputeLoopOrientation
  (
    const SmVector3d  & crReferenceUpVector,      ///< [in] :     <br>
    SmOrientType      & reLoopOrient,             ///< [out]:     <br>
    double            & rdLoopArea                ///< [out]:     <br>
  ) const;

  inline SmStatus UpdateListAddTriangle(SmTopology* pListStart, SmTopology* pSecondLeg, SmTopology* pThirdLeg);
  inline SmStatus UpdateListRemoveTriangle(SmTopology* pListStart);

  // simple access
  SmPolyBrep        * GetPolyBrep()          const ; 
  SmPolyFace        * GetPolyFace()          const { return (SmPolyFace *)m_pListOwner; }
  SmPolyEdge        * GetFirstPolyEdge()     const { return (SmPolyEdge *)m_pList; }
  SmPolyEdge        * GetLastPolyEdge()      const { return ((SmPolyEdge *)m_pList)->GetCWPolyEdge() ; }
  SmOrientType        GetOrientation()       const { return m_eOrientation; }
  SmPolyVertex      * GetLastEndPolyVertex() const { return m_pLastEndPolyVertex ; }
                      
  void                GetPolyEdges             (SmTArray<SmPolyEdge*>   & rPolyEdges) const;
  void                GetNonDegeneratePolyEdges(SmTArray<SmPolyEdge*>   & rPolyEdges) const;
  void                GetPolyVertices          (SmTArray<SmPolyVertex*> & rPolyVertices) const;
  void                GetPoints                (SmTArray<SmPoint3d>     & rPoints) const;
                      
  // simple access
  void              SetOrientation(SmOrientType eLoopOrient)    { m_eOrientation       = eLoopOrient; }
  void              SetLastEndPoint(SmPoint3d sLastEndPoint)         { m_sLastEndPoint      = sLastEndPoint ; }
  void              SetLastEndPolyVertex(SmPolyVertex *pPolyVertex)  { m_pLastEndPolyVertex = pPolyVertex ; }

  // predicates
  SmBoolean         IsPolyVertexLoop() const;
  SmBoolean         Is2d        () const { return GetFirstPolyEdge()->Is2d() ; }       // sometimes Z=0.0 for 2d which will be missed.
  SmBoolean         Is2dOrZero  () const { return GetFirstPolyEdge()->Is2dOrZero() ; } 
  SmBoolean         IsClosed    () const { return m_bClosed ; }      
  
  // utilities
  SmDisplayList * Draw
  (
    SmBoolean       bDrawRaw=TRUE,           ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                         <br>
    SmBoolean       bDrawIn3D=FALSE,         ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf, default:[FALSE]    <br>
    SmPlane       * pOptOutPlane=NULL,       ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]        <br>
    SmGfxArraySet * pOptGfxSet=NULL          ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.        <br>
  ) const;     

  void *operator  new (size_t size, SmPolyBrep * pObjectToGetContext);
  void  operator  delete(void *ptr) { SM_REF1(ptr); }

#ifndef SM_BORLAND
  void *operator  new(size_t size);
  void  operator  delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
  )  const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyLoop,SmOwningTopology,SmPolyLoop_TYPE);

} ; // end class SmPolyLoop

/*******************************************************************//**
PURPOSE: Represents a polygon face.

NOTES: see SmPolyBrep Model Topology and Shape notes for overview.

       SmPolyFace Derived Classes:  SmCPolyFace
                                     
       When the set of polygon vertices are planar and non-degenerate
       then the polygon normal is unique.  Otherwise, an effective
       normal is computed based on the accumulative positions of all
       the polygon vertices.
***********************************************************************/
class SM_EXPORT SmPolyFace : public SmOwningTopology
{
  friend class SmTriangleBag;
  friend class SmPolyVertex;
  friend class SmPolyEdge;
  friend class SmPolyLoop;
  friend class SmCPolyFace;
  friend class SmPolyShell;
  friend class SmPolyBrep;
  friend class SmPolyDecimate;
  friend class SmPolyMerge;
  friend class SmTess;
  friend class SmTessSrfCache;

protected:
  // inherited:
  // SmTopology::m_pListOwner       - SmPolyShell
  // SmTopology::m_pNext, m_pLast   - linked list of SmPolyFaces in SmPolyShell
  // SmOwningTopology::m_pList      - head of ordered SmPolyLoop list (1st outerloop, rest innerloops)
  // SmOwningTopology::m_pListSize  - number of PolyLoops in this PolyFace

  SmCPolyFace  * m_pCPolyFace;                    // back pointer to PolyCFace containing this PolyFace
  SmFace       * m_pOriginalFace = NULL;          // back pointer to SmBrep Face tessellated to generate this SmPolyFace
  SmZoneTol3d    m_sZoneTol3d;

  // m_vNormal and m_dMaxDeviation Set by SmPolyFace construtor or SmPolyFace::GetNormal() calls 
  SmVector3d     m_vNormal;        // normal to Face's Plane in RawSpace,            uninit:[SM_UNDEF_DOUBLE,SM_UNDEF_DOUBLE,SM_UNDEF_DOUBLE]
  double         m_dMaxDeviation;  // Max dist from Face's Plane to Face's vertices, uninit:[-SM_BIG_DOUBLE]

  // m_vSphereCenter and m_dSphereRadius Set by SmPolyFace::ComputeSphereBound() call
  SmPoint3d      m_vSphereCenter;        // Center of Bounding Sphere for this Face, uninit:[SM_UNDEF_DOUBLE,SM_UNDEF_DOUBLE,SM_UNDEF_DOUBLE]
  double         m_dSphereRadius = 0.0;  // Radius of Bounding Sphere for this Face, uninit:[0.0]

  ULONG          m_lIndexValue;    // a temp val,                                    uninit:[SM_UNDEF_ULONG]
                                   // commonly set up and used to index into indexed arrays

public:
#ifdef USE_DEBUG_COUNTER
  ULONG                 m_lDebugCount;
#endif
  ULONG                 m_lUserLong1;
  ULONG                 m_lUserLong2;
  ULONG                 m_lSmoothGroup;

protected:
  // private constructor:   SmPolyFace *pPolyFace = new (pPolyBrep) SmPolyFace() ;
  SmPolyFace();

public:
  // constructor:    SmPolyFace *pPolyFace = new (pPolyBrep) SmPolyFace(args,..) ;
  SmPolyFace
  (                                                                                                       
    SmZoneTol3d    dZoneTol3d,              ///< [in] : tol to store with new PolyFace                           <br>
    SmPolyBrep   * pOptPolyBrep   = NULL,   ///< [in] : if(pOptPolyShell==NULL && pOptPolyShell==NULL)           <br>
                                            ///<      : NotNULL = Make new PolyRegion in pOptPolyBrep,           <br>
                                            ///<      :           Make new PolyShell in NewPolyRegion,           <br>
                                            ///<      :           Make this PolyFace in NewPolyShell             <br>
                                            ///<      : NULL    = error                                          <br>
    SmPolyRegion * pOptPolyRegion = NULL,   ///< [in] : if(pOptPolyShell==NULL)                                  <br>
                                            ///<      : NotNull = make new PolyShell in pOptPolyRegion,          <br>
                                            ///<      :           make this PolyFace in NewPolyShell             <br>
    SmPolyShell  * pOptPolyShell  = NULL,   ///< [in] : NotNULL = make this PolyFace in pOptPolyShell            <br>
    SmVector3d   * pOptFaceNormal = NULL    ///< [in] : Normal for Face when known, NULL=ignore, default:[NULL]  <br>
  ) ; 

  // destructor
  virtual ~SmPolyFace();

  // assignment operator
  SmPolyFace & operator=( const SmPolyFace & crOther );

  // add a single vertex loop to the polygon
  SmStatus AddSinglePolyVertexLoop
  (
    const SmPoint3d & crPolyVertexPoint,      ///< [in] : 3d point for new vertex                                                 <br>
    SmZoneTol3d       dZoneTol3d,             ///< [in] : tolerance to assign to new vertex and edge                              <br>
    SmPolyVertex   *& rpNewPolyVertex,        ///< [out]: PolyVertex used for this SingleVertexLoop                               <br>
    SmPolyEdge     *& rpSingleVertexPolyEdge, ///< [out]: The degenerate edge needed to connect the PolyVertex to its PolyLoop    <br>
    SmPolyLoop     *& rpVertexPolyLoop        ///< [out]: The PolyLoop                                                            <br>
  );     

  // create PolyLoop ready to receive ordered sequence of SmPolyEdges. // see SmPolyLoop::AddPolyEdge()
  SmStatus StartPolyEdgeLoop
  (
    SmPolyLoop *& rpNewPolyLoop        ///< [out]: the new PolyLoop       <br>
  );             

  // remove and delete a PolyLoop
  SmStatus RemovePolyLoop
  (
    SmPolyLoop  *& rpPolyLoop,         ///< [in,out]: PolyLoop to delete. Deletes its PolyEdges and no longer used PolyVertices   <br>
    SmPolyVertex * pSavePolyVertex     ///< [in] : don't delete this PolyVertex when its PolyEdgeCount equals zero                <br>
  ) ;  
   
  // measures
  virtual SmStatus CalculateBoundingBox(SmExtent3d & rFaceBBox) const;      // Expanded by Vertices' tolerances

  SmStatus         ComputeSphereBound  
  (
    SmBoolean   bForceUpdate,           ///< [in] :  calc and cache containing Sphere    <br>
    SmPoint3d & rSphereCenter,          ///< [out]:                                      <br>
    double    & rdSphereRadius          ///< [out]:                                      <br>
  ) ;
                   
  // simple access
  SmPolyBrep          * GetPolyBrep()      const ; 
  SmPolyShell         * GetPolyShell()     const { return (SmPolyShell*)m_pListOwner; }
  virtual SmCPolyFace * GetCPolyFace()     { return m_pCPolyFace; }
  SmFace              * GetOriginalFace()  const { return m_pOriginalFace; }   // always OK to use ptr as a label, stale when GetOKBackPtrs() == FALSE
  virtual SmBoolean     GetOKBackPtrs()    const ;
                        
  SmPolyLoop          * GetOuterPolyLoop() const { return (SmPolyLoop *)m_pList; }
  virtual void          GetPolyLoops             (SmTArray<SmPolyLoop  *> & rPolyLoops)    const;
  virtual void          GetPolyEdges             (SmTArray<SmPolyEdge  *> & rPolyEdges)    const;
  virtual SmStatus      GetNonDegeneratePolyEdges(SmTArray<SmPolyEdge  *> & rPolyEdges)    const;
  virtual SmStatus      GetDegeneratePolyEdges   (SmTArray<SmPolyEdge  *> & rPolyEdges)    const;
  virtual void          GetPolyVertices          (SmTArray<SmPolyVertex*> & rPolyVertices) const;
  virtual void          GetPolyFacesOfComposite  (SmTArray<SmPolyFace * > & rPolyFaces)    const;

  SmZoneTol3d           GetTolerance()    const { return m_sZoneTol3d; }
  double                GetMaxDeviation() const { return m_dMaxDeviation; }

  virtual SmPoint3d     GetPoint     ()   const;
  SmPoint3d             GetCentroid  ()   const;
  virtual SmVector3d    GetNormal    
  (
    SmBoolean   bForceRecompute = FALSE,    ///< [in] : TRUE = always recompute m_vNormal & m_dMaxDeviation
    SmBoolean   bUpdateTolerances = TRUE,   ///< [in] : TRUE = make PolyEdge & PolyVertex dTols >= 2 * m_dMaxDeviation
    SmBoolean * pbOptIsDegenerate = NULL    ///< [out]: TRUE = degenerate polygon (sets m_vNormal[1,1,1]), NULL to ignore
  ) const;

  virtual SmVector3d    GetNormal3d  (SmSurface *pOptSurface=NULL) const ;   // when pOptSurface != NULL && pOrigSurf != NULL, rtn 3d space normal

  double                GetMaxDihedralAngRad(SmPolyEdge ** pOptPolyEdge  =NULL,   // rtn: [0-2Pi] ang for max edge->Dihedral angle - lamina edges = 2Pi
                                             SmVector3d  * pOptDiAngColor=NULL) const ;    

  void                  GetPolyFaceDetail(ULONG & rlLaminaCnt,           // rtn: Lamina, Manifold, and Spine PolyEdge counts
                                          ULONG & rlSpineCnt,
                                          ULONG & rlManifoldCnt) const;

  ULONG                 GetIndexValue() const            { SM_ASSERT_BREAK(m_lIndexValue != SM_UNDEF_ULONG) ; return m_lIndexValue; }


  // simple access
  void    SetTolerance   (SmZoneTol3d dTol )    { m_sZoneTol3d    = dTol; }
  void    SetNormalUnInit()                    { m_dMaxDeviation = -SM_BIG_DOUBLE; /* ForceRecompute flag for GetNormal() */ }
  void    SetCPolyFace   (SmCPolyFace * pCF)   { m_pCPolyFace = pCF; }
  void    SetOriginalFace(SmFace  * pOriginalFace, 
                          SmBoolean bPropagateAttribs=FALSE) ;
  void    SetIndexValue  (ULONG lIndexValue)   { SM_ASSERT_BREAK(lIndexValue != SM_UNDEF_ULONG) ; m_lIndexValue = lIndexValue; }
  void    SetLongValue   (ULONG lIndexValue)   { SetIndexValue(lIndexValue) ; } 

  // predicates
  SmBoolean IsDegenerate      (SmBoolean bRecomputeNormal=FALSE);
  SmBoolean IsPlanar          () const ;
  SmBoolean IsTriangle        () const ; // TRUE = 1 loop, 3 NonDegenerate PolyEdges
  SmBoolean IsBoundaryPolyFace() const ;
  SmBoolean IsIndexValueInit  ()       { return(m_lIndexValue != SM_UNDEF_ULONG) ; }
  SmBoolean Is2d              () const { return ((SmPolyLoop *)m_pList) ? ((SmPolyLoop *)m_pList)->Is2d() : FALSE ; }   // sometimes Z=0.0 for 2d which will be missed.      
  SmBoolean Is2dOrZero        () const { return ((SmPolyLoop *)m_pList) ? ((SmPolyLoop *)m_pList)->Is2dOrZero() : FALSE ; } 
  
  // calc properties

  // closest PolyFace PolyVertex to TestPoint
  SmStatus         ClosestVertex       
  (
    const SmPoint3d & crTestPoint,                 ///< [in] : target point                                          <br>
    double            dMaxDistance,                ///< [in] : search limit max distance                             <br>
    SmPolyEdge     *& rpPolyEdgeOfClosestVertex    ///< [out]: PolyEdge containing closest PolyVertex to TestPoint   <br>
  ) const;                                                                                                       

  // Determine if a PolyFace lies within the same plane as another - DOES NOT CHECK FOR OVERLAP
  virtual SmStatus CoincidenceCheck
  (
    const SmPolyFace & crFace,                      ///< [in] : target surface to compare                                    <br>
    double             d3DTolerance,                ///< [in] : max allowed distance sample points on coincident surfaces    <br>
    SmBoolean        & rbAreCoincident,             ///< [out]: TRUE = surfaces are coincident for some                      <br>
    double           & rMaxDistanceBetween          ///< [out]: Max distance seen between matched sample surface points      <br>
  ) const;

  // Classify the segment intersecting an edge of a face
  SmStatus EdgeLineSegOnClassify
  (
    const SmPolyEdge * cpEdge,                     ///< [in] :      <br>
    const SmVector3d & crLineVec,                  ///< [in] :      <br>
    SmBoolean        & rbIsInsideFace              ///< [out]:      <br>
  ) const;

  SmStatus FindVertexSector
  (
    const SmPolyVertex * cpVertex,                 ///< [in] : Target PolyVertex                                                                             <br>
    const SmVector3d   & crLineVec,                ///< [in] : Vector to classify                                                                            <br>
    SmBoolean            bValidateSector,          ///< [in] : TRUE = test all sectors and validate that vec is contained within returned sector.            <br>
                                                   ///<      : FALSE= Return Vertice's 1st sector without validation for optimization.                       <br>
                                                   ///<      :        Only use FALSE if vector is known to belong to vertice's 1st sector in this face.      <br>
                                                   ///<      :        Useful for something like a triangle corner when Vec has been built to be              <br>
                                                   ///<      :        inside the triangle and all we really need is the value of the rpEdgeOfSector pointer. <br>
    SmPolyEdge        *& rpEdgeOfSector            ///< [out]:  PolyEdge starting boundary of the Vertex sector containing LineVec.                          <br>
  ) const;

  SmStatus  FindClosestPointToLoop
  (
    SmPolyLoop  * pLoop,                                  ///< [in] : PolyLoop to check                                               <br>
    SmPolyEdge  * pOptLoopEdge,                           ///< [in] : NotNull = Find closest point to just this PolyLoopEdge,         <br>
                                                          ///<      : NULL    = Find closest point to any PolyLoopEdge                <br>
    SmPolyEdge *& rpLoopEdge,                             ///< [out]: PolyLoopEdge closest to NeighborLoopEdge                        <br>
    SmPolyEdge *& rpOtherLoopEdge,                        ///< [out]: NeighborLoop->PolyEdge with start-pt closest to PolyLoopEdge    <br>
    SmTArray<SmPolyEdge*> * pOptNonDegenPolyEdges = NULL  ///< [in] : Optional array of nondegenerate PolyEdges from this face.       <br>
                                                          ///<      : NULL to ignore. Default is NULL.                                <br>
  ) const; 

  // get PolyFace vertices in Tri[P0,P1,P2]
  SmStatus FindVerticesInTriangle
  (
    const SmPoint3d       & crP0,                     ///< [in] : P0 of Tri[P0,P1,P2]                                          <br>
    const SmPoint3d       & crP1,                     ///< [in] : P1 of Tri[P0,P1,P2]                                          <br>
    const SmPoint3d       & crP2,                     ///< [in] : P2 of Tri[P0,P1,P2]                                          <br>
    SmTree                * pVertexTree,              ///< [in] : vertex spatial tree, can narrow search if present            <br>
    SmTArray<SmPolyEdge*> & rInsideVertices,          ///< [out]: PolyEdges with StartPts in        Tri[P0,P1,P2]              <br>
    SmPolyEdge           *& rpVertexClosestToV0V1,    ///< [out]: PolyEdge  with StartPt closest to PEdge[P0,P1]               <br>
    SmTArray<SmPolyEdge*> & rInsideVertices_3d,       ///< [out]: PolyEdges with StartPts           Tri[pOrigSurf(P0,P1,P2)]   <br>
    SmPolyEdge           *& rpVertexClosestToV0V1_3d, ///< [out]: PolyEdge  with StartPt closest to PEdge[pOrigSurf(P0,P1)]    <br>
    SmBoolean               bWorkIn3D,                ///< [in] : TRUE=Do 3D tests, load xx_3d outputs, FALSE=Skip 3D tests    <br>
    SmMarkType              eMarkType                 ///< [in] : checks without incrementing eMarkType Value                  <br>
  ) const ;

  // get PolyFace vertices in Tri[V0->StartPt,V1->StartPt,V2->StartPt] - calls FindVerticesInTriangle
  SmStatus FindInsideVertices
  (
    SmPolyEdge            * pV0,                      ///< [in] : V0 of Tri[P0=V0->StartPt, P1=V1->StartPt, P2=V2->StartPt]       <br>
    SmPolyEdge            * pV1,                      ///< [in] : V1 of Tri[P0=V0->StartPt, P1=V1->StartPt, P2=V2->StartPt]       <br>
    SmPolyEdge            * pV2,                      ///< [in] : V2 of Tri[P0=V0->StartPt, P1=V1->StartPt, P2=V2->StartPt]       <br>
    SmTree                * pVertexTree,              ///< [in] : vertex spatial tree, can narrow search if present               <br>
    SmTArray<SmPolyEdge*> & rInsideVertices,          ///< [out]: PolyEdges with StartPts in        Tri[P0,P1,P2]                 <br>
    SmPolyEdge           *& rpVertexClosestToV0V1,    ///< [out]: PolyEdge  with StartPt closest to PEdge[P0,P1]                  <br>
    SmTArray<SmPolyEdge*> & rInsideVertices_3d,       ///< [out]: PolyEdges with StartPts           Tri[pOrigSurf(P0,P1,P2)]      <br>
    SmPolyEdge           *& rpVertexClosestToV0V1_3d, ///< [out]: PolyEdge  with StartPt closest to PEdge[pOrigSurf(P0,P1)]       <br>
    SmBoolean               bWorkIn3D,                ///< [in] : TRUE=Do 3D tests, load xx_3d outputs, FALSE=Skip 3D tests       <br>
    SmMarkType              eMarkType                 ///< [in] : checks without incrementing eMarkType Value                     <br>
  ) const ;

  static SmStatus FindSplitLine
  (
      SmTArray<SmPolyEdge*>& rLoopEdges,    ///< [in] : ordered target edges forming a polygon to                   <br>
                                            ///<        be split                                                    <br>
      SmVector3d& rAveragePlaneNormal,      ///< [in] : plane normal of the target loop                             <br>
      double* pdOptMinAspectRatio,          ///< [in] : reject splits that make childLoop's with aspect             <br>
                                            ///<        ratios less than this                                       <br>
                                            ///<      : AspectRatio = dMinDist/dLenSplitLine                        <br>
                                            ///<        dMinDist = Min(Dist from SplitPlane to                      <br>
                                            ///<        LoopPoint).                                                 <br>
                                            ///<        NULL to ignore                                              <br>
      SmBoolean* pbOptUseLooseSideCheck,    ///< [in] : TRUE = only reject splits with child                        <br>
                                            ///<        loops that touch the split line                             <br>
                                            ///<      : FALSE= reject splits with child loops                       <br>
                                            ///<        that touch or cross the splitPlane                          <br>
                                            ///<      : NULL = use FALSE                                            <br>
      SmBoolean& rbSuccess,                 ///< [out]: TRUE = found a split line that satisfies split              <br>
                                            ///<        constraints, FALSE=didn't                                   <br>
      double& rdMaxAspectRatio,             ///< [out]: Aspect ratio associated with split, when found.             <br>
                                            ///<        Else, -SM_BIG_DOUBLE.                                       <br>
      ULONG& rlLineStartIndex,              ///< [out]: SplitLine Start =                                           <br>
                                            ///<        rLoopEdges[rlLineStartIndex]->GetStartPoint()               <br>
      ULONG& rlLineEndIndex                 ///< [out]: SplitLine End   =                                           <br>
                                            ///<        rLoopEdges[rlLineStartIndex]->GetStartPoint()               <br>
  );

  // XSect segment with PolyFace->PolyEdge lines
  SmStatus IntersectSegment
  (
    const SmPoint3d       & crStart,                  ///< [in] : segment start point                                <br>
    const SmPoint3d       & crEnd,                    ///< [in] : segment end point                                  <br>
    SmZoneTol3d             dZoneTol3d,               ///< [in] : ignore distance for xsects near inpoints           <br>
    SmTArray<SmPoint3d>   & rIntersections,           ///< [out]: array of all intersection points                   <br>
    SmTArray<SmPolyEdge*> & rIntersectionEdges,       ///< [out]: associated edge for each intersection              <br>
    SmTree                * pOptEdgeTree = NULL,      ///< [in] : optional edge tree to cull intersection candidates <br>
    SmTArray<SmPolyEdge*> * pOptEdgesToCheck = NULL   ///< [in] : optional array to specify intersection candidates. <br>
                                                      ///<      : NULL to ignore. Default is NULL                    <br>
  ) const;

  // get XSect Line between the two planes of two PolyFaces, no XSect when XSect line is outside of either PolyFace->BBox
  SmStatus IntersectWithPolyFace
  (
    const SmPolyFace & crOtherFace,           ///< [in] :           <br>
    SmPoint3d        & rLinePnt,              ///< [in] :           <br>
    SmVector3d       & rLineVec               ///< [in] :           <br>
  ) const;

  SmStatus LineSegOnClassify
  (
    SmBoolean                 bDoPointClassify,    ///< [in] :      <br>
    SmLineSegClassification & rClassification      ///< [in] :      <br>
  ) const;

  SmStatus MakeEdgeToSingeVertexLoop
  (
    SmPolyVertex * pLoopVert,            ///< [in] : Vertex in existing Loop                                          <br>
    SmPolyVertex * pSingleVertexL,       ///< [in] : Single Vertex Loop                                               <br>
    SmPolyEdge  *& rpNewEdge             ///< [out]: The PolyEdge of the manifold polyEdge pair that connects         <br>
                                         ///<      : from pSingleVertexL to pLoopVert.  Its radial partner            <br>
                                         ///<      : runs in the opposite direction from pLoopVert to pSingleVertexL  <br>
  );

  SmStatus MakeManifoldEdge
  (
    SmPolyEdge    * pStartVertexEdge,                ///< [in] : start of new edge = pStartVertexEdge->GetStartPolyVertex()                                         <br>
    SmPolyEdge    * pEndVertexEdge,                  ///< [in] : end of new edge   = pEndVertexEdge->GetStartPolyVertex()                                           <br>
    SmPolyEdge   *& rpNewEdge,                       ///< [out]: Edge going from Start to End (a radial mateEdge running from End to Start is also made)            <br>
    SmPolyLoop   *& rpNewLoop,                       ///< [out]: New PolyLoop or NULL when no new polyLoop was made                                                 <br>
    SmPolyFace   *& rpNewFace,                       ///< [out]: New PolyFace or NULL when no new polyFace was made                                                 <br>
    SmBoolean       bDoSingularEdges = FALSE,        ///< [in] : TRUE = Split Singular Edge and connect to new PolyVertex if that can make NewEdge an IsoParamCurve <br>
                                                     ///<      : FALSE= don't                                                                                       <br>
    SmPolyEdge   ** pOptNewSingularEdge = NULL,      ///< [out]: New SingularEdge or NULL when no Singular Edge found needing splitting                             <br>
                                                     ///<      : (a radial mateEdge running from End to Start is also made)                                         <br>
    SmPolyVertex ** pOptNewSingularVertex = NULL,    ///< [out]: New SingularVertex or NULL when no Singular Edge found needing splitting                           <br>
    SmPolyEdge    * pOptEdgeForTriangle = NULL       ///< [in] : Optional third edge for triangle, along with NewEdge and StartVertexEdge.
                                                     ///<        Optimizes updating topology. NULL to ignore. Default is NULL.
  ) ;

  SmStatus MakeVertexSplitPolyEdge
  (
    SmPolyEdge      * pEdgeToSplit,                  ///< [in] : PolyEdge to split                          <br>
    const SmPoint3d & crSplitPoint,                  ///< [in] : Point split location                       <br>
    SmPolyEdge     *& rpNewEdge,                     ///< [out]: new PolyEdge (and new radial partners)     <br>
    SmPolyVertex   *& rpNewVertex                    ///< [out]: new PolyVertex                             <br>
  ); 

  SmStatus OrderPolyLoops() ; // make sure 1st PolyLoop is orient==SM_OT_SAME and the rest are SM_OT_OPPOSITE

  SmStatus PointInLoopTest
  (
    SmPolyLoop      * pLoopToTest,            ///< [in] :      <br>
    const SmPoint3d & crPointToTest,          ///< [in] :      <br>
    SmBoolean       & rbInside,               ///< [in] :      <br>
    SmBoolean         bIn3D = FALSE           ///< [in] :      <br>
  ) const;

  SmStatus PointInPolygon
  (
    const SmPoint3d & crPointToTest,      ///< [in] : target point                                               <br>
    SmBoolean       & rbInside,           ///< [out]: TRUE = point is inside or on face boundary, FALSE = not    <br>
    SmBoolean         bIn3D = FALSE       ///< [in] : TRUE =                                                     <br>
  ) const;

  SmStatus ReplacePolyFace( SmPolyFace * pOther,   // in : 
                            SmPolyBrep * pBrep) ;  // in : NotUsed: 

  SmStatus ReverseOrientation();
    
  // Helper method for SmPolyBrep::ComputeProperties() to get MassProps as integral over surface boundary
  SmStatus ComputeProperties
  (
    const SmPoint3d    & crOriginOfComputation,  ///< [in] : see usage notes                            <br>
    double             & rdDeltaArea,            ///< [out]: area of face                               <br>
    double             & rdDeltaVolume,          ///< [out]: volume 'between' face and origin           <br>
    SmPoint3d          & rDeltaBarycenter,       ///< [out]: centroid times delta volume                <br>
    SmVector3d           aDeltaMoments[2],       ///< [out]: [0] Second moments, int x2,y2,z2 dV        <br>
                                                 ///<      : [1] Products, int xy,yz,zx dV              <br>
    SmMassPropertiesType eWhichProps=SM_MPT_ALL  ///< oneof: SM_MPT_AREA      These are sequential.     <br>
                                                 ///<      : SM_MPT_VOLUME    All prior values of       <br>
                                                 ///<      : SM_MPT_CENTROID  requested value are       <br>
                                                 ///<      : SM_MPT_MOMENTS   computed and returned     <br>
                                                 ///<      : SM_MPT_ALL. default:[SM_MPT_ALL]           <br>
  ) const;                                        

  SmStatus TriangulateNonPlanar(SmTArray<SmPolyFace*> & rNewTriangleTriangles);

  // This is an alternative to SmTess class Tessellation
  SmStatus TriangulatePlanar   (SmTArray<SmPolyFace*> & rNewTriangleFaces) ; 

  SmStatus TessellateWithQuads 
  (
    double                  dQuadSize,              ///< [in] :             <br>
    SmTessCallback        & rTessCB,                ///< [in] :             <br>
    SmTArray<SmPolyFace*> & rNewFaces,              ///< [out]:             <br>
    SmTArray<SmPolyFace*> & rNewInteriorFaces       ///< [out]:             <br>
  ); 

  // overloaded new and delete - allocate SmPolyFace memory from SmPolyBrep::m_FMgr SmMemBlockMgr memory
  void *operator new (size_t size, SmPolyBrep * pObjectToGetContext);  // NotUsed: in : size
  void  operator delete(void *ptr) { SM_REF1(ptr); }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  SmDisplayList * Draw     
  (
    SmBoolean       bDrawRaw=TRUE,           ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                                <br>
    SmBoolean       bDrawIn3D=FALSE,         ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf, default:[FALSE]           <br>
    SmBoolean       bDrawFaceCenter=FALSE,   ///< [in] :                                                                                               <br>
    SmBoolean       bAddToUIPickList=FALSE,  ///< [in] : TRUE = add to display list so this can be picked, FALSE=don't.                                <br>
    SmPlane       * pOptOutPlane=NULL,       ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]               <br>
    SmGfxArraySet * pOptGfxSet=NULL,         ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.               <br>
    SmEdgeTopoType  eEdgeTopoType=SM_ET_ALL  ///< [in] : one of SM_ET_WIRE, SM_ET_LAMINA, SM_ET_MANIFOLD, SM_ET_SPINE, SM_ET_ALL                       <br>
  ) const ;                                   

  SmDisplayList * DrawDebug
  (
    SmBoolean       bDrawRaw=TRUE,          ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                                    <br>
    SmBoolean       bDrawIn3D=FALSE,        ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf                                <br>
    SmBoolean       bAllNormals=TRUE,       ///< [in] : TRUE = Draw Every Edge Normal, FALSE = draw only the first                                        <br>
    SmBoolean       bFlipNormals=FALSE,     ///< [in] : TRUE = Flip Edge Normals, FALSE=don't, default:[FALSE]                                            <br>
    SmPlane       * pOptOutPlane=NULL,      ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane                                   <br>
    SmGfxArraySet * pOptGfxSet=NULL         ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore    <br>
  ) const; 
                                                                    

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyFace,SmOwningTopology,SmPolyFace_TYPE);

} ; // end class SmPolyFace

/*******************************************************************/ /**
 PURPOSE: Specialization of SmOwningTopology::UpdateList for the case in which an
 SmPolyLoop has been constructed and contains precisely the three listed PolyEdges.

 NOTES: Utilized in MakeManifoldEdge to improve performance of the tessellator during
 SmTess::TriangulateSingleFace. 

 ***********************************************************************/
inline SmStatus SmPolyLoop::UpdateListAddTriangle(SmTopology* pListStart,
                                                  SmTopology* pSecondLeg,
                                                  SmTopology* pThirdLeg)
{
  // all three PolyEdges must be in the doubly-linked list.
  SM_ASSERT(pListStart == pListStart->GetNext()->GetNext()->GetNext());
  SM_ASSERT(pSecondLeg == pSecondLeg->GetNext()->GetNext()->GetNext());
  SM_ASSERT(pThirdLeg == pThirdLeg->GetNext()->GetNext()->GetNext());

  m_lListSize = 3;
  m_pList = pListStart;

  pListStart->SetListOwner(this);
  pSecondLeg->SetListOwner(this);
  pThirdLeg ->SetListOwner(this);

  return SM_SUCCESS;

} // end SmPolyLoop::UpdateListAddTriangle

/*******************************************************************/ /**
 PURPOSE: Specialization of SmOwningTopology::UpdateList for the case in which an
 SmPolyLoop has been editted to remove a triangle.

 NOTES: This is used in MakeManifoldEdge to improve tessellator performance. In that application,
 pListStart has already been inserted into the list, along with the radial mate. SmPolyLoop::UpdateListAddTriangle
 is used to define ownership in the new PolyLoop, this method corrects m_lListSize for the original PolyLoop.
 ***********************************************************************/
inline SmStatus SmPolyLoop::UpdateListRemoveTriangle(SmTopology* pListStart)
{
  SM_ASSERT(m_lListSize > 3);
  SM_ASSERT(pListStart->GetNext()->GetOwner() == this);
  SM_ASSERT(pListStart->GetLast()->GetOwner() == this);
  SM_ASSERT(pListStart->GetOwner() == this);

  m_lListSize -= 3;
  m_pList = pListStart;

  return SM_SUCCESS;

} // end SmPolyLoop::UpdateListRemoveTriangle

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyFace*) ;

/*******************************************************************//**
PURPOSE: Represents a composite polyface.

NOTES: This class is declared as final. Can't derive a new class from this
class without removing final.
***********************************************************************/
class SM_EXPORT SmCPolyFace final : public SmPolyFace
{
  friend class SmPolyVertex;
  friend class SmPolyEdge;
  friend class SmPolyLoop;
  friend class SmPolyFace;
  friend class SmPolyShell;
  friend class SmPolyBrep;
  friend class SmPolyMerge;

protected:
  // inherited:
  // SmTopology::m_pListOwner     - SmPolyBrep::m_pCPolyFaceListHead
  // SmTopology::m_pNext, m_pLast - linked list of SmPolyCFace's in SmPolyBrep's list
  // SmOwningTopology::m_pList    - <not used>
  // SmPolyFace::m_pCPolyFace     - points back to 'this'

  SmTArray<SmPolyFace*> m_vPolyFaces;

public:
  // constructor:    SmCPolyFace *pCPolyFace = new (pPolyBrep) SmCPolyFace(pPolyFace) ;
  SmCPolyFace(SmPolyFace * pPolyFace);

  // destructor
  virtual ~SmCPolyFace();

  // manage m_vPolyFaces list
  SmStatus AddPolyFace(SmPolyFace *pFaceToAdd, SmBoolean bAllowDuplicates=FALSE);
  SmStatus RemovePolyFace(SmPolyFace * pPolyFace);
  SmStatus ReplaceCPolyFace( SmCPolyFace *pNewCPFace );

  // measures
  virtual SmStatus CalculateBoundingBox(SmExtent3d & rFaceBBox) const;  // Expanded by Vertices' tolerances

  // simple access
  virtual SmCPolyFace * GetCPolyFace() { return this; }
  virtual void          GetPolyFacesOfComposite  (SmTArray<SmPolyFace*> & rPolyFaces)   const;
  virtual void          GetPolyLoops             (SmTArray<SmPolyLoop*> & rPolyLoops)   const;
  virtual void          GetPolyEdges             (SmTArray<SmPolyEdge*> & rPolyEdges)   const;
  virtual SmStatus      GetNonDegeneratePolyEdges(SmTArray<SmPolyEdge  *> & rPolyEdges) const;
  virtual SmStatus      GetDegeneratePolyEdges   (SmTArray<SmPolyEdge  *> & rPolyEdges)    const;
  virtual SmBoolean     GetOKBackPtrs() const ;

  virtual SmPoint3d     GetPoint() const;
  virtual SmVector3d    GetNormal
  (
    SmBoolean bForceRecompute = FALSE,        ///< [in]:
    SmBoolean bUpdateTolerances = TRUE,       ///< [in]:
    SmBoolean * pbIsDegenerate = NULL         ///< [in]:
  ) const;

  virtual SmVector3d    GetNormal3d(SmSurface *pOptSurface=NULL) const ;   // when pOptSurface != NULL && pOrigSurf != NULL, rtn 3d space normal
  
  // predicates
  SmBoolean Is2d()       const { return m_vPolyFaces.GetSize() > 0 ? m_vPolyFaces[0]->Is2d() : FALSE ; }   // sometimes Z=0.0 for 2d which will be missed.    

  SmBoolean Is2dOrZero() const { return   (m_vPolyFaces.GetSize() == 0) ? FALSE
                                        : (m_vPolyFaces.GetSize() == 1) ? m_vPolyFaces[0]->Is2dOrZero() 
                                        : (m_vPolyFaces[0]->Is2dOrZero() && m_vPolyFaces[1]->Is2dOrZero()) ; 
                               }
                          
  // overloaded new and delete - allocate SmCPolyFace memory from SmPolyBrep::m_CFMgr SmMemBlockMgr memory
  void *operator new (size_t size, SmPolyBrep * pObjectToGetContext);   // NotUsed: in : size
  void  operator delete(void *ptr) { SM_REF1(ptr); }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  SmDisplayList   * Draw
  (
    SmBoolean       bDrawRaw=TRUE,          ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                                     <br>
    SmBoolean       bDrawIn3D=FALSE,        ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf, default:[FALSE]                <br>
    SmPlane       * pOptOutPlane=NULL,      ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                    <br>
    SmGfxArraySet * pOptGfxSet=NULL         ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore.    <br>
  ) const; 

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
  )  const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCPolyFace,SmPolyFace,SmCPolyFace_TYPE);

} ; // end class SmCPolyFace

/*******************************************************************//**
PURPOSE: Represents a polygon shell.

NOTES: This class is declared as final. Can't derive a new class from this
class without removing final.
***********************************************************************/
class SM_EXPORT SmPolyShell final : public SmOwningTopology
{
  friend class SmPolyVertex;
  friend class SmPolyEdge;
  friend class SmPolyLoop;
  friend class SmPolyFace;
  friend class SmPolyRegion;
  friend class SmPolyBrep;
  friend class SmObjsDelete<SmPolyShell*>;

protected:
  // inherited:
  // SmTopology::m_pListOwner       - SmPolyRegion
  // SmTopology::m_pNext, m_pLast   - linked list of SmPolyShells in SmPolyRegion
  // SmOwningTopology::m_pList      - head of doubly linked SmPolyFace list
    
  // private destructor
  virtual ~SmPolyShell() { }

public:
  // constructor:    SmPolyShell *pPolyShell = new (pPolyBrep) SmPolyShell(pPolyRegion) ;
  SmPolyShell(SmPolyRegion * pPolyRegion = NULL);

  void           GetPolyFaces (SmTArray<SmPolyFace*> & rPolyFaces) const;
  SmPolyRegion * GetPolyRegion() const;
  SmPolyBrep   * GetPolyBrep  () const;

  // overloaded new and delete - allocate SmPolyShell memory from SmPolyBrep::m_SMgr SmMemBlockMgr memory
  void *operator new (size_t size, SmPolyBrep * pObjectToGetContext);   // NotUsed: in : size
  void  operator delete(void *ptr) { SM_REF1(ptr); }
  
  // predicates
  SmBoolean Is2d()       const { return ((SmPolyFace *)m_pList) ? ((SmPolyFace *)m_pList)->Is2d() : FALSE ; }   // sometimes Z=0.0 for 2d which will be missed.       
  SmBoolean Is2dOrZero() const { return ((SmPolyFace *)m_pList) ? ((SmPolyFace *)m_pList)->Is2dOrZero() : FALSE ; }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  SmDisplayList * Draw
  (
    SmBoolean       bDrawRaw=TRUE,          ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                                  <br>
    SmBoolean       bDrawIn3D=FALSE,        ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf, default:[FALSE]             <br>
    SmPlane       * pOptOutPlane=NULL,      ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                 <br>
    SmGfxArraySet * pOptGfxSet=NULL         ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore. <br>
  ) const; 
                   
  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,            ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
     SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                               ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
     SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
     SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyShell,SmOwningTopology,SmPolyShell_TYPE);

} ; // end class SmPolyShell

/*******************************************************************//**
PURPOSE: Represents a polygon volume which may be a collection of 
    an outer and inner shells.

NOTES: This class is declared as final. Can't derive a new class from this
class without removing final.
***********************************************************************/
class SM_EXPORT SmPolyRegion final : public SmOwningTopology
{
  friend class SmPolyVertex;
  friend class SmPolyEdge;
  friend class SmPolyLoop;
  friend class SmPolyFace;
  friend class SmPolyShell;
  friend class SmPolyBrep;
  friend class SmObjsDelete<SmPolyRegion*>;

  // inherited:
  // SmTopology::m_pListOwner       - SmPolyBrep
  // SmTopology::m_pNext, m_pLast   - doubly linked list of SmPolyBrep->SmPolyRegions
  // SmOwningTopology::m_pList      - head of ordered SmPolyShell list
  //                                     (1st outershell, rest innershells)

protected:
  // private destructor
  virtual ~SmPolyRegion() { }

public:
  // constructor:    SmPolyRegion *pPolyRegion = new (pPolyBrep) SmPolyRegion(pPolyBrep) ;
  SmPolyRegion(SmPolyBrep *pPolyBrep);

  void         GetPolyShells(SmTArray<SmPolyShell*> & rPolyShells) const;
  SmPolyBrep * GetPolyBrep() const;

  // predicates
  SmBoolean Is2d()       const { return ((SmPolyShell *)m_pList) ? ((SmPolyShell *)m_pList)->Is2d() : FALSE ; }   // sometimes Z=0.0 for 2d which will be missed.       
  SmBoolean Is2dOrZero() const { return ((SmPolyShell *)m_pList) ? ((SmPolyShell *)m_pList)->Is2dOrZero() : FALSE ; }

  // overloaded new and delete - allocate SmPolyRegion memory from SmPolyBrep::m_RMgr SmMemBlockMgr memory
  void *operator new (size_t size, SmPolyBrep * pObjectToGetContext);   // NotUsed: in : size
  void  operator delete(void *ptr) { SM_REF1(ptr); }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, SmPolyBrep * pObjectToGetContext) { SM_REF2(ptr, pObjectToGetContext); }
#endif // no SM_BORLAND

  SmDisplayList * Draw
  (
    SmBoolean       bDrawRaw=TRUE,          ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                                    <br>
    SmBoolean       bDrawIn3D=FALSE,        ///< [in] : TRUE = if Is2dOrZero(), draw pts after projecting through OrigSurf, default:[FALSE]               <br>
    SmPlane       * pOptOutPlane=NULL,      ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                   <br>
    SmGfxArraySet * pOptGfxSet=NULL         ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore.   <br>
  ) const; 

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
  ) const ;                                                                                                                                             

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyRegion,SmOwningTopology,SmPolyRegion_TYPE);

} ; // end class SmPolyRegion

/*******************************************************************//**
PURPOSE: Represents a polygonal brep.

NOTES: 

NOTES: SmPolyBrep Model

TOPOLOGY: 

PolyVertex owned by PolyBrep->m_pPolyVertexListHead :[m_pListOwner],       neighbors:[PolyVertex->m_pNext, PolyVertex->m_pLast]
           has an ordered list of PolyEdges         :[m_pPolyEdgeList],    members  :[PolyEdge->m_pNextEdgeAtV, PolyEdge->m_pPrevEdgeAtV]
                                                                         
PolyEdge   owned by a PolyLoop                      :[m_pListOwner],       neighbors:[PolyEdge->m_pNext, PolyEdge->m_pLast]
           has a start PolyVertex                   :[m_pPolyVertex]
           has radial partners                      :[m_pNextRadialE, m_pLastRadialE]
           has StartPolyVertex partners             :[m_pNextEdgeAtV, m_pPrevEdgeAtV]
           gets end PolyVertex from PolyLoop data   : either PolyEdge->GetCCWPolyEdge()->GetStartPolyVertex()
                                                          or PolyEdge->GetPolyLoop()->m_pLastEndPolyVertex
                                                      as decided in PolyEdge->GetEndPolyVertex()
                                                    
PolyLoop   owned by a PolyFace                      :[m_pListOwner],        neighbors:[PolyLoop->m_pNext, PolyLoop->m_pLast]
           owns a CCW ordered list of PolyEdges     :[PolyLoop->m_pList],   members  :[PolyEdge->m_pNext (CCW), PolyEdge->m_pLast (CW)]
           Last PolyVertex when m_bClosed == FALSE  :[PolyLoop->m_pLastEndPolyVertex]
                                                    
PolyFace   owned by a PolyShell                     :[m_pListOwner],        neighbors:[PolyFace->m_pNext, PolyFace->m_pLast] 
           owns an ordered list of PolyLoops,       :[PolyFace->m_pList]    members  :[PolyLoop->m_pNext, PolyLoop->m_pLast]
              (1st outerloop, rest innerloops)      
           may be contained by a PolyCFace          :[PolyFace->m_pCPolyFace]
                                                    
PolyCFace  owned by SmPolyBrep->m_pCPolyFaceListHead:[m_pListOwner],        neighbors:[PolyCFace->m_pNext, PolyCFace->m_pLast]
           has a list of PolyFaces                  :[SmTArray<SmPolyFace *> m_vPolyFaces] 
                                                    
PolyShell  owned by a PolyRegion                    :[m_pListOwner],        neighbors:[PolyShell->m_pNext, PolyShell->m_pLast] 
           owns an ordered list of PolyFaces        :[PolyShell->m_pList],  members  :[PolyFace->m_pNext, PolyFace->m_pLast]
              (1st outershell, rest innershells)    
                                                    
PolyRegion owned by a PolyBrep                      :[m_pListOwner],        neighbors:[PolyRegion->m_pNext, PolyRegion->m_pLast] 
           owns a list of PolyShells,   ListHead    :[PolyRegion->m_pList], members  :[PolyShell->m_pNext, PolyShell->m_pLast]
                                                    
PolyBrep   is not owned                             :[m_pListOwner = UnDefined]
           owns a list of PolyRegions,  ListHead    :[PolyBrep->m_pList],   members:[PolyRegion->m_pNext, PolyRegion->m_pLast]
                                                    


SHAPE: All PolyBrep shapes are defined by a set of vertex point locations in 2d or 3d
 
 Shapes: (GWC: all the degenerate shape ideas are my current guesses - these assumptions need to be verified)
   PolyVertex =    Point      [GetPoint()]   <== the only stored geometry in the PolyBrep model

   PolyEdge   =    The union of a Pair of points + all the points on a straight line between the points
                     Line       [GetStartPoint(), GetEndPoint() = GetCCWEdge()->GetStartPoint()]. 
                     Point      when PolyEdge is degenerate, GetStartPoint() == GetEndPoint().

   PolyLoop   =    PolyLine = a sequence of connected points and the lines between those points
                                 [GetPoints() = Ordered PolyEdge->PolyVertex->GetPoints(), GetPoints().GetSize() > 2] 
                     Polygon boundary        when PolyLine is closed and GetPoints().GetSize() > 2  and vertices are not collinear or coincident
                     degenerate Polygon      when PolyLine is closed and (GetPoints().GetSize() < 2 or vertices are collinear or coincident)
                     PolyWire (made up name) when PolyLine is open   and GetPoints().GetSize() > 2
                     Line                    when PolyLine is open   and GetPoints().GetSize() == 2
                     Point                   when PolyLine is open and GetPoints().GetSize() == 1
              
        In a manifold model of connected polygons (a PolySheet or a Polyhedron) every
        PolyLine[PolyVertex1, PolyVertex2] will be represented in two PolyEdges in two different PolyLoops.
        One represented   by SmPolyEdge[StartPoint = PolyVertex1]
        and the other     by SmPolyEdge[StartPoint = PolyVertex2] 
          Those two edges are connected when they are radial neighbors to one another,
                    otherwise those edges are coincident.  
          When more than two PolyLines share the same end points, PolyLine[PolyVerte1, PolyVertex2]
             Each use of the PolyLine is represented by one SmPolyEdge in its own SmPolyLoop 
                (or repeated use in a single PolyLoop for complicated cases.) 
             Those SmPolyEdges form a spine edge when they all are radial neighbors to one another.
             Otherwise they form a set of coincident PolyLines. 
             I guess in a valid model no coincident PolyLines are allowed.  They all must
             be radial neighbors. A set of coincident PolyLines can be found and made radial partners
             with a call to SmPolyBrep::Stitch()
                              
   PolyFace   =   when its 1st PolyLoop is closed [the union of all its PolyLoop shapes + all the internal points to the polygon (an area)] 
                  when its 1st PolyLoop is open   [the union of all its PolyLoop shapes (a piecewise line)]
                  polygon     when 1st PolyFace->PolyLoop is closed polygon 
                                           shape = Union PolyLine shapes + all points internal to that boundary.
                                           gwc: That's well defined for planar polygons.  But what are the
                                           internal points for a nonplanar polygon?
                                          with holes when subsequent PolyFace->PolyLoops are closed polygons
                                          with internal PolyLines when subsequent PolyFace->PolyLoops are open
                   PolyLine   [When 1st PolyLoop is open - subsequent PolyFace->PolyLoops don't make sense in this situation]

   PolyShell  = This could be two different concepts because which faces belong to a PolyShell are
                not enforced by the member relationships of the SmPolyFace and SmPolyShell class objects.

                Possibility 1:
                If this is different than the SmBrep world this could be the union of all the PolyEdge shapes that
                are connected together through shared PolyVertices and PolyEdges.  Let's call
                this complex of points a 'PolyWeb.'

                        PolyWeb(made up name)  =  The set of all points connected together by any combination of PolyEdges.
                             This happens when PolyWires and Polygons intersect at edges and vertices.
                               [not modeled directly.
                                I think the set can be found by looking at the list of all PolyEdges 
                                that start at the vertices of an initial PolyLoop. 
                                From those PolyEdges additional PolyLoops can be found. From those
                                additional PolyLoops a list of additional PolyVertices can be found.
                                After which recursively check the list of additional PolyVertices 
                                until no new PolyLoops are found.  That final accumulated list of
                                PolyVertices (organized into PolyEdges and PolyLoops) forms a
                                single connected PolyWeb. Some of the PolyVertices can be organized
                                into PolyFaces which can then have holes and internal PolyWires.
                                The PolyLoops of those internal holes and wires also have to be
                                added to the list of PolyLoops connected to the PolyShell.
                            
                                In the language of SmBrep models,
                                this definition in cases where Spine edges are present would gather together the
                                set of all 'BrepShells' into one PolyWeb complex.

                         In the end a PolyWeb could enclose any number of closed region spaces.

                  Possibility 2:
                  If this is the same as the SmBrep world a SmPolyShell could be the set of all PolyFaces that are
                  found through a recursive visit of the SmEdge->Radial neighbor relationships.  Starting
                  with a single SmPolyFace polygon, Recurse as

                     PolyFace -to-> PolyLoops -to-> PolyEdges -to-> RadialNeighbor 
                                  -to-> PolyLoops -to-> Neighbor PolyFaces -to-> Recurse

                     In addition, the PolyWires connected through shared PolyVertices would have to be added.
                     In the end the set of collected faces would bound a closed region when closed.
                          
                     For this definition of a PolyShell we can further define a Regularized PolyShell which
                     consists of only those PolyFaces on the boundary of an enclosed region. 

                     
                 more notes:  [Depending on what PolyBrep model shapes we want to model we'll have to do different things.
                               Some of the cases that will have to be examined include:
                               1. A single manifold polyhedron 
                               2. A single manifold sheet
                               3. A nonmanifold polyhedron with Spine edges to other polygons
                               4. A nonmanifold polyhedron with cluster vertices to other PolyWires
                               5. A nonmanifold PolySheet with Spine edges to other polygons
                               6. A nonmanifold PolySheet with cluster vertices to other PolyWires
                               7. A Shell PolyWire
                               8. A Shell Point
                               9. MultiRegion models

                               In the current SmPolyBrep data model I can see how to model Spine edges and Cluster vertices.
                               But I don't see how a set of PolyFaces can be organized into a set of PolyShells that
                               subdivide space.  Consider a 3 region model, a box split internally in two by a 
                               sheet.  Are those 3 regions enclosed in 3 PolyShells?  In which case
                               All the PolyFaces would have to be loaded into two PolyShells.  That can't be done with
                               this model because each PolyFace has one PolyShell owner (not two).
                               Does that mean that even though the PolyTopology model can represent spine
                               edges that they should not be used?  That's something for Bill and George to discuss.
                                     
                               An alternative would be that the PolyFace of a PolyShell are defined
                               to be all those PolyFace gathered in a recursive walk of the Topology graph

                               PolyFace -to-> PolyEdge -to-> RadialPolyEdge -to-> NeighborPolyFace -to-> recursion

                               until that recursion comes to an end. That set of Polygons when closed would
                               bound a closed region of points.  In the simple 3 region model described above
                               This recursion would gather together 3 different sets of PolyFaces where
                               every PolyFace would be listed in exactly two of the collected lists.

                               If we want to model general nonManifold PolyModels we'll have to change
                               the PolyCode to allow for a PolyFace to have two owners, a Positive normal
                               owner and a negative normal owner.

                               If we don't want to model general manifold objects, we'll have to 
                               come up with rules for use of the PolyModel objects to prevent combining
                               into invalid shapes.  A simple rule would be just manifold models.
                               A slightly more complicated rule would be manifold models plus sheets.
                               However, that gets into trouble with the Boolean operator.  What happens
                               when a Polygon is added to a sheet that ends up splitting an existing region.
                               That's a subdivision of space which can not be modeled correctly by the current
                               SmPolyBrep topology model.

                               We need to decide what set of models to represent in the SmPolyBrep.
                               Note when the SmPolyBrep is used to model a bag of triangles (as we do
                               for rendering) the problems about shells and regions don't come up.
                               They do become important when working on the meaning of a Boolean and
                               when building models up in bits and pieces as we do in the Brep world.

   Regularized PolyShell = This is a made up name for the concept of collecting the subset of a Shell that
                           forms a closed boundary of one simply closed region of points in space.
                           If we stored two Shell uses for each SmPolyFace it can quickly be computed
                           as all those Polygon faces that are used only once by a PolyShell.  This
                           would remove all the PolyLines and the sheet PolyFaces.

                           With single Shell ownership for PolyFaces this can only be computed by
                           recursion through the PolyEdge RadialPartner relationships as described above.
                                      
   PolyRegion =   [ union of all shell shapes + the simply connected point regions bounded by the regularized PolyShells]
                          When the first shell is closed, it's the outer boundary of the Region point set.
                             subsequent closed shells are internal hole boundaries to the region.
                             subsequent open Shells are internal sheet models. 
                           When the first shell is open, the region is all space with internal sheet boundaries.

                           Without two Shell owners per PolyFace, we run into modeling ambiguities for
                           regions that partition space, i.e. the 3 region model example mentioned above.

                         
   PolyBrep   =  [ union of all PolyRegion shapes ]
                
   PolyCFace  =    Varying    [union of any set of member PolyFace shapes]

   This class is declared as final. Can't derive a new class from this class without removing final.

***********************************************************************/
class SM_EXPORT SmPolyBrep final : public SmSAGObject
{
  friend class SmPolyVertex;
  friend class SmPolyEdge;
  friend class SmPolyLoop;
  friend class SmPolyFace;
  friend class SmCPolyFace;
  friend class SmPolyShell;
  friend class SmPolyRegion;
  friend class SmPolyMerge;
  friend class SmPolyDecimate;
  friend class SmTess;
  friend class SmTessSrfCache;

  // inherited:
  // SmTopology::m_pListOwner       - <not used>
  // SmTopology::m_pNext, m_pLast   - <not used>
  // SmOwningTopology::m_pList      - head of doubly linked SmPolyRegion list

protected:
  SmOwningTopology    * m_pPolyVertexListHead = NULL;  // PolyBrep heap object that points to 1st member of vertex list
  SmOwningTopology    * m_pCPolyFaceListHead = NULL;   // PolyBrep heap object that points to 1st member of Composite face list
// SmTree              * m_pPolyTree;                  // not used
  SmZoneTol3d           m_sZoneTol3d;           
                                                
  SmArenaMemBlockMgr    m_sMgr;                 // Manages memory

  SmBoolean             m_bOwnsContext;         // TRUE = delete context on destruction, FALSE=don't
  SmBoolean             m_bOKBackPtrs;          // During tessellation backpointers
                                                //   SmPolyFace::m_pOriginalFace (the pOriginalFace), 
                                                //   SmPolyEdge::m_pOriginalEdgeuse, and 
                                                //   SmPolyVertex::m_pOriginalVertex 
                                                // pointers are valid and this value is TRUE.
                                                // After tessellation the back pointers are stale and this value is FALSE.

public:
  // constructor:    SmPolyBrep *pPolyBrep = new (crContext) SmPolyBrep(args...) ;
  SmPolyBrep(SmZoneTol3d dZoneTol3d,
             // If we are using memory-block management,
             // this informs the numbers of poly-elements
             // that should fit into the first block.
             ULONG lEstimatedPolyElementNum=32);

  // copy constructor:    SmPolyBrep *pPolyBrep = new (crContext) SmPolyBrep(crPolyBrepToCopy,..) ;
  SmPolyBrep
  (
    const SmPolyBrep  & crPolyBrepToCopy,
    SmCopyPolyBrepMap * pOptCopyPolyBrepMap = NULL,
    SmCopyPolyBrepMap * pOptReverseMap      = NULL
  ) ;
    
  // destructor
  virtual ~SmPolyBrep();

  SmStatus CalculateBoundingBox
  (
    SmExtent3d          & rNormalBox,            ///< [out]: Expanded by Vertices' tolerances                                           <br>
    SmPseudoBox         * pOptPseudoBox=NULL,    ///< [out]: optional tight data aligned PseudoBox, NULL to ignore                      <br>
                                                 ///<      : not guaranteed to minSized                                                 <br>
                                                 ///<      : not expanded                                                               <br>
    SmTArray<SmPoint3d> * pOptPolyPoints=NULL,   ///< [in,out]: when GetSize() > 0, used as is, else loaded with all PolyVertex Points  <br>
                                                 ///<      : when passed, empty or not, rNormalBox is not expanded                      <br>
                                                 ///<      : NULL to ignore, default:[NULL]                                             <br>
    SmPoint3d           * pOptCentroid=NULL,     ///< [out]: optional geometric average point, NULL to ignore                           <br>
    SmVector3d  * pOptInputPseudoBoxNormal=NULL  ///< [in] : when given and when building a pseudo box,                                 <br>
                                                 ///<      : used to set the basis[2] direction of the pseudoBox                        <br>
  ) const ;                                      
  
  SmStatus CleanSolidMesh(SmBoolean bOptDoDecimate=FALSE);

  // set all PolyFace->m_pOrigFace and PolyEdge->m_pOriginalEdgeuse ptrs to NULL
  SmStatus ClearOrigBrepBackPointers() ;

  static SmStatus CreateBox
  (
    const SmContext        & crContext,                 ///< [in] :                                                                    <br>
    SmZoneTol3d              dZoneTol3d,                ///< [in] : Tolerance for this PolyBrep                                        <br>
    double                   dXSize,                    ///< [in] : Size in positive X direction of the crPosition.                    <br>
    double                   dYSize,                    ///< [in] : Size in the positive Y direction                                   <br>
    double                   dZSize,                    ///< [in] : Size in the positive Z direction                                   <br>
    const SmAxis2Placement & crPosition,                ///< [in] : The position defines the placement of the lower corner of the box. <br>
                                                        ///< [in] : Note that the entire box must lie in the positive quadrents.       <br>
    SmPolyBrep            *& rpNewBrep,                 ///< [out]: Newly created brep                                                 <br>
    SmTArray<SmPolyFace*>  * pOptPolyFaces = NULL       ///< [out]: Optional faces in the following order:                             <br>
                                                        ///<      : [0] - face on X,Y plane                                            <br>
                                                        ///<      : [1] - face on X,Z plane                                            <br>
                                                        ///<      : [2] - face parallel to the Y,Z plane                               <br>
                                                        ///<      : [3] - face parallel to the X,Z plane                               <br>
                                                        ///<      : [4] - face on the Y,Z plane                                        <br>
                                                        ///<      : [5] - face parallel to the X,Y plane                               <br>
  );

  static SmStatus CreateLinearSweepSolid
  (
    const SmContext           & crContext,              ///< [in] :        <br>
    SmZoneTol3d                 dZoneTol3d,             ///< [in] :        <br>
    const SmTArray<SmPoint3d> & crPoints,               ///< [in] :        <br>
    const SmVector3d          & crSweepVector,          ///< [in] :        <br>
    SmPolyBrep               *& rpNewBrep               ///< [out]:        <br>
  );

  SmStatus CreatePoly
  (
    SmPolyShell               * pShell,           ///< [in] : Shell in which the face belongs or NULL if we need to create a new shell.  <br>
    const SmTArray<SmPoint3d> & rPnts,            ///< [in] :                                                                            <br>
    SmPolyFace               *& rpPolyFace        ///< [out]:                                                                            <br>
  );

  SmStatus CreatePolyFace
  (
    SmPolyRegion                   * pNewRegion, 
    SmPolyShell                    * pNewShell,             ///< [in] : opt: shell for new face.                             <br>
    const SmTArray<SmPolyVertex *> & crVertexList,          ///< [in] : Vertices for the new face.                           <br>
    SmPolyFace                    *& rpNewFace,             ///< [out]: The resulting face.                                  <br>
    SmVector3d                     * pOptNormal = NULL      ///< [in] : Optional Normal to NewFace if known, NULL to ignore  <br>
  );

  SmStatus CreatePolyTriangle
  (
    SmPolyRegion * pNewRegion,        ///< [in] :                                           <br>
    SmPolyShell  * pNewShell,         ///< [in] : opt Shell in which to create the Face     <br>
    SmPolyVertex * v0,                ///< [in] : Vertex 0 of the new triangle.             <br>
    SmPolyVertex * v1,                ///< [in] : Vertex 1 of the new triangle.             <br>
    SmPolyVertex * v2,                ///< [in] : Vertex 2 of the new triangle.             <br>
    SmPolyFace  *& newFace            ///< [out]: The resulting triangle.                   <br>
  );

  SmStatus CreatePolyQuad
  (
    SmPolyRegion * pNewRegion, 
    SmPolyShell  * pNewShell,         ///< [in] : opt Shell in which to create the Face     <br>
    SmPolyVertex * v0,                ///< [in] : Vertex 0 of the new quad.                 <br>
    SmPolyVertex * v1,                ///< [in] : Vertex 1 of the new quad.                 <br>
    SmPolyVertex * v2,                ///< [in] : Vertex 2 of the new quad.                 <br>
    SmPolyVertex * v3,                ///< [in] : Vertex 3 of the new quad.                 <br>
    SmPolyFace * & newFace            ///< [out]: The resulting quad.                       <br>
  );

  SmStatus CreateNonPlanarPoly
  (
    SmPolyShell               * pShell,     ///< [in] : Shell in which the face belongs or NULL if we need to create a new shell.    <br>
    const SmTArray<SmPoint3d> & rPnts,      ///< [in] :                                                                              <br>
    SmPolyFace               *& rpPolyFace  ///< [out]:                                                                              <br>
  );

  // Add a SmPolyRegion->SmPolyShell->SmPolyFace->SmPolyLoop->SmPolyEdge[4]
  // complex to this PolyBrep.  
  SmStatus CreateRectangle
  (
    SmZoneTol3d              dZoneTol3d,             ///< [in] : min distance between points                                          <br>
    double                   dXSize,                 ///< [in] : Size along X axis of position                                        <br>
    double                   dYSize,                 ///< [in] : Size along Y axis of position                                        <br>
    const SmAxis2Placement & crPosition,             ///< [in] : Rectangle bottom left is at origin and lies                          <br>
                                                     ///<      : in the X, Y plane in the positive quadrant.                          <br>
    SmPolyFace            *& rpPolyFace,             ///< [out]: new SmPolyFace rectangle with its own SmPolyShell and SmPolyRegion   <br>
    SmTArray<SmPolyEdge*>  * pOptEdges = NULL,       ///< [out]: Edges are in counter clockwise order                                 <br>
                                                     ///<      : starting at 0,0 going to x,0, then                                   <br>
                                                     ///<      : x,y, then 0,y, then back to 0,0                                      <br>
    SmTree                 * pOptVertTree = NULL     ///< [out]: If this is given look for existing vertices when adding edges.       <br>
  );

  SmStatus CreateTriStripFaces
  (
    const SmTArray<SmPoint3d> & rTriStripPnts,        ///< [in] : Array of  TriStrip points                                                                <br>
    SmPolyShell               * pOptShell = NULL,     ///< [in] : Optional input shell in which the faces belong or NULL if we need to create a new shell. <br>
    SmTArray<SmPolyFace*>     * pOptNewFaces = NULL   ///< [out]: Optional output array that contains tristrip faces                                       <br>
  );

  SmStatus CombineCoincidentVertices
  (
    SmPolyVertex            * pSurvivingVertex,                ///< [in] : PolyVertex to remain after combining                                              <br>
    SmPolyVertex            * pVertexToDelete,                 ///< [in] : PolyVertex to delete after combining - stale after this call                      <br>
    SmTArray<SmPolyEdge*>   & rDeletedEdges,                   ///< [out]: List of stale deleted PolyEdge pointers                                           <br>
                                                               ///<      : (when Verts connect to a commonPolyFace but not a CommonPolyEdge,                 <br>
                                                               ///<      : these PolyEdges are 1st created and then deleted in this method)                  <br>
    SmTArray<SmPolyFace*>   & rFacesUpdated,                   ///< [out]: List of PolyFaces attached to deleted PolyEdges                                   <br>
                                                               ///<      : (some of these may be stale)                                                      <br>
    SmTArray<SmPolyVertex*> * pOptDeletedPolyVertices=NULL,    ///< [out]: optional array of additional deleted PolyVertices, NULL to ignore, default:[NULL] <br>
    SmTArray<SmPolyVertex*> * pOptSurvivingPolyVertices=NULL   ///< [out]: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]        <br>
  ); 

  SmStatus CollectManifoldFaceSheet
  (
    SmPolyFace            * pStartFace,           ///< [in] :                                   <br>
    SmTArray<SmPolyFace*> & rCollectedFaces,      ///< [out]:                                   <br>
    SmMarkType              eMarkType,            ///< [in] : not incremented                   <br>
    SmTArray<SmPolyEdge*> * pLaminaEdges = NULL,  ///< [out]: boundary edges; default NULL      <br>
    SmTArray<SmPolyEdge*> * pSpineEdges = NULL    ///< [out]: boundary edges; default NULL      <br>
  ); 

  SmStatus CollectPolyFaceShell
  ( 
    SmPolyFace             * pStartFace,         ///< [in] : A face in the shell to traverse                                        <br>
    SmOrientType             eStartSide,         ///< [in] : SM_OT_SAME     = traverse from Normal side of StartFace                <br>
                                                 ///<      : SM_OT_OPPOSITE = traverse from Opposite side of StartFace              <br>
    SmTArray<SmPolyFace*>  & rpCollectedFaces,   ///< [out]: All PolyFaces connected to pStartFace through sided edge connections   <br>
    SmTArray<SmOrientType> & reCollectedSides,   ///< [out]: associated SM_OT_SAME     = Normal points into this shell's region     <br>
                                                 ///<      :           SM_OT_OPPOSITE = Opposite points into this shell's region    <br>
    SmMarkType               eSameMarkType,      ///< [in] : Mark associated for eSide = Same                                       <br>
    SmMarkType               eOppositeMarkType   ///< [in] : Mark associated for eSide = Opposite                                   <br>
  );                       

  // note: increments two unlocked mark values for nonManifold PolyBreps
  SmStatus CollectPolyFaceShells
  ( 
    SmTArray< SmTArray<SmPolyFace*> *>  & rpCollectedFaces,            ///< [out]: Array of FaceArrays, Each FaceArray makes a shell.                         <br>
                                                                       ///<      : Nested arrays are allocated in this method                                 <br>
    SmTArray< SmTArray<SmOrientType> *> & reCollectedSides,            ///< [out]: associated SM_OT_SAME     = PFace Normal points into this shell's region   <br>
                                                                       ///<      :            SM_OT_OPPOSITE = PFace Opposite points into this shell's region <br>
    SmBoolean                             bMakeOuterShells = FALSE     ///< [in] : TRUE = Add Inner and Outer Shells to output                                <br>
                                                                       ///<      : FALSE= Only add Inner Shells to output,                                    <br>
  );                                     

  SmStatus ComputeAreaCentroid
  (
    double    & rdArea,
    SmPoint3d & rCentroid
  );

  SmStatus ComputeProperties
  (
    const SmPoint3d    & crOriginOfComputation,        ///< [in] : see usage notes                             <br>
    double             & rdArea,                       ///< [out]: area of face                                <br>
    double             & rdVolume,                     ///< [out]: volume 'between' face and origin            <br>
    SmPoint3d          & rdBarycenter,                 ///< [out]: centroid times delta volume                 <br>
    SmVector3d           aMoments[2],                  ///< [out]: [0] Second moments, int x2,y2,z2 dV         <br>
                                                       ///< [out]: [1] Products, int xy,yz,zx dV               <br>
    SmMassPropertiesType eWhichProps = SM_MPT_ALL      ///< [in] : see Usage Notes, above.                     <br>
  ) const;

  SmStatus DeletePolyFace
  (
    SmPolyFace              * pPolyFace,                      ///< [in] : PolyFace to delete                                                   <br>
    SmTArray<SmPolyLoop*>   * pOptDeletedPolyLoops=NULL,      ///< [out]: opt Array of deleted PolyLoops,    NULL to ignore, default:[NULL]    <br>
    SmTArray<SmPolyEdge*>   * pOptDeletedPolyEdges=NULL,      ///< [out]: opt Array of deleted PolyEdges,    NULL to ignore, default:[NULL]    <br>
    SmTArray<SmPolyVertex*> * pOptDeletedPolyVertices=NULL    ///< [out]: opt Array of deleted PolyVertices, NULL to ignore, default:[NULL]    <br>
  ) ; 


  SmStatus DeletePolyLoop
  (
    SmPolyLoop              * pPolyLoop,                      ///< [in] : PolyLoop to delete
    SmTArray<SmPolyEdge*>   * pOptDeletedPolyEdges=NULL,      ///< [out]: opt Array of deleted PolyEdges,    NULL to ignore, default:[NULL]    <br>
    SmTArray<SmPolyVertex*> * pOptDeletedPolyVertices=NULL    ///< [out]: opt Array of deleted PolyVertices, NULL to ignore, default:[NULL]    <br>
  ) ; 


  SmStatus DeleteStandalonePolyVertex(SmPolyVertex * pPolyVertex);

  SmStatus DeleteTopologicalVertex
  (
    SmPolyVertex          * pPolyVertexToRemove,       ///< [in] :                                         <br>
    SmTArray<SmPolyEdge*> & rRemovedEdges              ///< [out]: list of edges deleted (stale pointers)  <br>
  );

  SmStatus DeleteVertexLoop(SmPolyVertex * pPolyVertex) ;


  // Get plane/PolyFace intersection line segments 
  SmStatus DoSectioning
  (
    const SmPoint3d     & crPlaneOrigin,   ///< [in] :                                                        <br>
    const SmVector3d    & crPlaneNormal,   ///< [in] :                                                        <br>
    SmZoneTol3d           dZoneTol3d,      ///< [in] :                                                        <br>
    SmTArray<SmPoint3d> * pLineSegPnts,    ///< [out]: each pair of points marks one segment of               <br>
                                           ///<      : intersection between the plane and a polyBrep PolyFace <br>
    SmTArray<SmPoint3d> * pTangentSegPnts  ///< [out]:                                                        <br>
  ) const;

  // get position and normal of ray intersection with PolyBrep faces
  SmStatus RayIntersection
  (
    const SmPoint3d  & crRayPoint,        ///< [in] : fired ray's base point                                                                          <br>
    const SmVector3d * cpOptRayVector,    ///< [in] : fired ray's direction or NULL,                                                                  <br>
                                          ///<      : NULL = intersect PolyBrep with crRayPoint.                                                      <br>
    double             dRayTolerance,     ///< [in] : If the ray misses by a small distance, this value determines                                    <br>
                                          ///<      : how far away it will still be a 'hit'                                                           <br>
                                          ///<      : Note: that this value is 'added' to the tolerance                                               <br>
                                          ///<      : on the brep entities.  Make this value 0.0 if                                                   <br>
                                          ///<      : the tolerance on the PolyBrep objects is acceptable.                                            <br>
    double             dRayStepoffValue,  ///< [in] : How far down the ray should we go before accepting an answer.                                   <br>
                                          ///<      : This should be a parameter value relative to the ray point and vector.                          <br>
                                          ///<      : A good value for this is something like                                                         <br>
                                          ///<      : 2.0 * SM_EFF_ZERO * (1.0 + crRayPoint.GetMaxDimension())                                        <br>
    SmRayTracer     ** pOptRayTracer,     ///< [in,out]: Optional RayTracer cache.                                                                    <br>
                                          ///<      : if not NULL { if *pOptRayTracer == NULL, build a RayTracer and store it here.                   <br>
                                          ///<      :                  Caller must delete this object when done with it.                              <br>
                                          ///<      :               if *pOptRayTracer != NULL, skip build use this ray tracer as is.                  <br>
                                          ///<      :             }                                                                                   <br>
                                          ///<      : if NULL     { build local RayTracer, use it once, and delete it }                               <br>
                                          ///<      : When anything goes wrong, *ppOptRayTracer is set to NULL and anything it pointed to is deleted. <br>
    SmSolutionArray  & rSolutions         ///< [out]: The solution is returned in a non standard form because PolyFaces are not parameterized.        <br>
                                          ///<      : rSolution.m_SolutionType  = SM_ST_SINGLE_VALUE                                                  <br>
                                          ///<      : rSolution.m_lNumVariables = 1 ;                                                                 <br>
                                          ///<      : rSolution.m_vStart[0]     = dHitT;         // ray parameter                                     <br>
                                          ///<      : rSolution.m_vStart[1]     = SM_BIG_DOUBLE;                                                      <br>
                                          ///<      : rSolution.m_vStart[2]     = SM_BIG_DOUBLE;                                                      <br>
                                          ///<      : rSolution.m_vStart[3]     = sHitPnt.x;     // ray/face intersection point                       <br>
                                          ///<      : rSolution.m_vStart[4]     = sHitPnt.y;                                                          <br>
                                          ///<      : rSolution.m_vStart[5]     = sHitPnt.z;                                                          <br>
                                          ///<      : rSolution.m_vStart[6]     = sHitNormal.x ; // face normal at xsect                              <br>
                                          ///<      : rSolution.m_vStart[7]     = sHitNormal.y ;                                                      <br>
                                          ///<      : rSolution.m_vStart[8]     = sHitNormal.z ;                                                      <br>
                                          ///<      : rSolution.m_lNumObjects   = 1;                                                                  <br>
                                          ///<      : rSolution.m_apObjects[0]  = pHitFace;                                                           <br>
                                          ///<      : rSolution.m_apObjects[1]  = pHitFace;                                                           <br>
  ) const;                                                                                                                                
                                          
  SmStatus FillLaminaHoles();

  SmStatus FillPlanarHole
  (
    SmPolyShell           * pShell,
    SmTArray<SmPolyEdge*> & rEdges,
    SmPolyFace           *& rpNewFace
  );

  SmStatus FixTriangulation();

  SmPolyFace   * GetFirstPolyFace  () const;
  SmPolyVertex * GetFirstPolyVertex() const;
  SmZoneTol3d    GetTolerance      () const { return m_sZoneTol3d; }
  SmBoolean      GetOKBackPtrs     () const { return m_bOKBackPtrs; }
  void           GetAuxDataList    (SmTArray<SmPolyVertAuxData*> & rAuxDataList)           const;
  void           GetPolyRegions    (SmTArray<SmPolyRegion*>      & rPolyRegions)           const;
  void           GetPolyShells     (SmTArray<SmPolyShell*>       & rPolyShells)            const;
  void           GetPolyFaces      (SmTArray<SmPolyFace*>        & rPolyFaces,             
                                    SmFace                       * pOptCommonFace=NULL)    const;
  void           GetPolyEdges      (SmTArray<SmPolyEdge*>        & rPolyEdges,             
                                    SmEdgeuse                    * pOptCommonEdgeuse=NULL) const;
  void           GetNonDegeneratePolyEdges(SmTArray<SmPolyEdge*> & rPolyEdges,             
                                           SmEdgeuse             * pOptCommonEdgeuse=NULL) const;
  void           GetPolyVertices   (SmTArray<SmPolyVertex*>      & rPolyVertices)          const;
  void           GetPolyPoints     (SmTArray<SmPoint3d>          & rPolyPoints)            const;
  void           GetCPolyFaces     (SmTArray<SmCPolyFace*>       & rCPolyFaces)            const;

  SmStatus GetLaminaPolyEdges     (SmTArray<SmPolyEdge*> & rLaminaPolyEdges) const;
  SmStatus GetManifoldPolyEdges   (SmTArray<SmPolyEdge*> & rManifoldPolyEdges) const;
  SmStatus GetSpinePolyEdges      (SmTArray<SmPolyEdge*> & rSpinePolyEdges) const;
  SmStatus GetNonManifoldPolyEdges(SmTArray<SmPolyEdge*> & rNonManifoldPolyEdges) const;

  ULONG GetNumberLaminaPolyEdges() const ;
  ULONG GetNumberSpinePolyEdges() const ;
  ULONG GetNumberManifoldPolyEdges() const ;

  // look for intersecting PolyEdges that share the same m_pOriginalFace pointer value
  SmBoolean GetIntersectingPolyEdges 
  (
    SmTArray<SmPolyEdge *> & rPolyEdges1,        ///< [out]: Edge1 of Edge1/Edge2 intersection pairs                              <br>
    SmTArray<SmPolyEdge *> & rPolyEdges2,        ///< [out]: Edge2 of Edge1/Edge2 intersection pairs                              <br>
    double                   dNearBy = 0.001     ///< [in] : Distance at which PolyLine nearest points are considered a problem   <br>
                                                 ///<      : This is not a typical Tolerance, this is meant to allow              <br>
                                                 ///<      : out of plane tessellations to be treated as nearly planar.           <br>
  ) const ; 

  SmBoolean GetCoincidentPolyVertices
  (
    SmTArray<SmPolyVertex *> & rPolyVertices1,   ///< [out]: Vertex1 of Vertex1/Vertex2 coincident pairs    <br>
    SmTArray<SmPolyVertex *> & rPolyVertices2    ///< [out]: Vertex2 of Vertex1/Vertex2 coincident pairs    <br>
  ) const ;

  SmBoolean GetCoincidentTopology    
  (
    double dTol,
    SmTArray< SmTopology * > & rObjs1,          ///< [out]: associated lists of coincident entities     <br>
    SmTArray< SmTopology * > & rObjs2           ///< [out]:                                             <br>
  ) const;

  // Glue two PolyEdges of a PolyBrep without deleting any PolyEdges by making them radial partners
  SmStatus GlueEdges
  (
    SmPolyEdge              * pEdge,                      ///< [in] : Target PolyEdge1                                                                              <br>
    SmPolyEdge              * pOtherEdge,                 ///< [in] : Target PolyEdge2                                                                              <br>
    SmOrientType              eOrient,                    ///< [in] : oneof SM_OT_SAME, SM_OT_OPPOSITE                                                              <br>
    SmTArray<SmPolyEdge*>   & rEdgesBetweenGluedVertices, ///< [out]: PolyEdges between glued vertices (made zero length by gluing)                                 <br>
    SmTArray<SmPolyVertex*> * pDeletedVertices = NULL,    ///< [out]: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]  <br>
    SmTArray<SmPolyVertex*> * pSurvivingVertices=NULL     ///< [out]: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]      <br>
  ) ;  

  SmStatus GlueVertices
  (                                                       ///< [in] : vertex to keep                                         <br>
    SmPolyVertex          * pVertexToKeep,                ///< [in] : vertex to delete                                       <br>
    SmPolyVertex          * pVertexToDelete,              ///< [out]: all edges between target vertices                      <br>
    SmTArray<SmPolyEdge*> & rEdgesBetweenVertices         ///<      : after this call those edges will be zero length with   <br>
                                                          ///<      : edge->StartVertex == edge->EndVertex - not deleted     <br>

  );

  SmBoolean IsManifoldSolid() const; 

  // sometimes Z=0.0 for 2d which will be missed.
  SmBoolean Is2d           () const { return  m_pPolyVertexListHead ? ((SmPolyVertex *)m_pPolyVertexListHead->GetList())->Is2d() : FALSE ;  }    

  SmBoolean Is2dOrZero     () const ; 

  SmBoolean IsOuterFaceSheet
  ( 
    const SmTArray<SmPolyFace*>  & raFaces,
    const SmTArray<SmOrientType> & raSides,
    SmRayTracer                 ** pOptRayTracer=NULL 
  );

  SmBoolean IsBagOfPolygons(SmTArray<SmPolyEdge *> *pOptPolyEdges=NULL) const ; // TRUE= most PolyEdges are lamina (Brep likely a tessellation output needing topology)                                    

  SmStatus LinearSweepFace
  (
    const SmPolyFace      * pFaceToSweep,     ///< [in] : Base face to sweep                           <br>
    const SmVector3d      & crSweepVec,       ///< [in] : Vector giving sweep direction and length.    <br>
    SmBoolean               bMakeEndCap,      ///< [in] : If true then construct a cap face            <br>
    SmTArray<SmPolyFace*> * pPolyFaces        ///< [out]: Resulting faces                              <br>
  );

  SmStatus MakeEmptyPolyFace
  (
    SmZoneTol3d    dZoneTol3d,                ///< [in] :                                                  <br>
    SmPolyRegion * pPolyRegion,               ///< [in] : If not NULL poly face goes into this region      <br>
    SmPolyShell  * pPolyShell,                ///< [in] : If not NULL poly face goes into this shell       <br>
                                              ///< [in] : If both are NULL poly face goes into poly brep   <br>
    SmPolyFace  *& rpNewPolyFace,             ///< [out]:                                                  <br>
    SmVector3d   * pOptNormal=NULL            ///< [out]:                                                  <br>
  );

  SmStatus MakeEdgeInFace
  (
    SmPolyFace   * pFace,                   ///< [in] : target PolyFace                                                                              <br>
    SmPolyVertex * pStartVertex,            ///< [in] : Start of New manifold PolyEdge pair                                                          <br>
    SmPolyVertex * pEndVertex,              ///< [in] : End of New manifold PolyEdge pair                                                            <br>
    double dEdgeTolerance,                  ///< NotUsed: [in] : NewEdge assigned tolerance                                                                   <br>
    SmPolyEdge *& rpNewEdge,                ///< [out]: one of the PolyEdges of the new manifold PolyEdge pair.                                      <br>
                                            ///<      : The PolyEdge pair can be any mix of new or reused PolyEdges.                                 <br>
    SmPolyLoop *& rpNewLoop,                ///< [out]: Created when NewEdge splits an existing PolyLoop (vertices start connected to same PolyLoop) <br>
    SmPolyFace *& rpNewFace                 ///< [out]: Created when PolyLoop is split to contain one of the split children.                         <br>
                                            ///<      : pFace contains the other split child.                                                        <br>
  );

  SmStatus MakeEdgesRatioCompliant
  (
    double dRatio,                         ///< [in] : 1.0 - same ratio or smaller of all edges to smallest     <br>
                                           ///<      : typically you would never use 1.0.  You might use 1.1.   <br>
                                           ///<      : 2.0 - make edges less than 2x the smallest               <br>
    ULONG lMaxEdgeNumber                   ///< [in] : If this is not zero then only set this value             <br>
                                           ///<      : the polygon has this many edges or less.  3 -            <br>
                                           ///<      : only do this to triangles.                               <br>
  );

  SmStatus MakeLoopsFromPoints
  
  (SmTArray<SmPoint3d> & rLineSegPnts,
   SmZoneTol3d dZoneTol3d,
   SmTArray<SmPolyLoop*> & rLoops
  );

  SmStatus MakePolyBrepsFromShells
  ( 
    SmTArray< SmPolyBrep* > & rNewBreps,   ///< [out]: 1 new PolyBrep for every Shell found in This PolyBrep       <br>
    SmBoolean bMakeOuterShells = FALSE     ///< [in] : TRUE =make PolyBreps for inner and outer shells             <br>
                                           ///<      : FALSE=Make PolyBreps only for inner shells                  <br>
  );  
                                                                        

  SmStatus MakePolyBrepsFromShells                   // note: increments two unlocked mark values for nonManifold PolyBreps
  (
    SmTArray< SmTArray<SmPolyFace*> *>  & rpFaces,   ///< [in] : Array of FaceArrays, Each FaceArray makes a shell.                                       <br>
    SmTArray< SmTArray<SmOrientType> *> & reSides,   ///< NotUsed: [in] : associated SM_OT_SAME     = PFace Normal points into this shell's region                 <br>
                                                     ///<      :            SM_OT_OPPOSITE = PFace Opposite points into this shell's region               <br>
    SmTArray< SmPolyBrep* >             & rNewBreps  ///< [out]: 1 new PolyBrep for every Shell in rpCollectedFaceLists                                   <br>
  );                
                                                                                        
  
  SmStatus MakeTopologyFromData
  (
    SmPolyBrepData         * pPolyBrepData,             ///< [in] : sequential data to copy                                                  <br>
    SmTArray<SmAttribute*> & rAllAttributes,            ///< [in] : made by FromPolyBrep() With pPolyBrepData                                <br>
    SmCopyPolyBrepMap      * pOptIndexObjectMap=NULL,   ///< [in] : made by FromPolyBrep() With pPolyBrepData, NULL to ignore                <br>
    SmCopyPolyBrepMap      * pOptCopyPolyBrepMap=NULL,  ///< [out]: A map from copyPolyBrep topology Obj ptrs to orig ptrs, NULL to ignore   <br>
    SmCopyPolyBrepMap      * pOptReverseMap=NULL        ///< [out]: A map from orig topology Obj ptrs to copyPolyBrep ptrs, NULL to ignore   <br>
  ) ;     
             
  void Make2d() {SM_PTR_ARRAY(sVs,SmPolyVertex,512) ; GetPolyVertices(sVs) ; for(ULONG i=0;i<sVs.GetSize();i++) { sVs[i]->GetPoint().Make2d() ; }}
  void Make3d() {SM_PTR_ARRAY(sVs,SmPolyVertex,512) ; GetPolyVertices(sVs) ; for(ULONG i=0;i<sVs.GetSize();i++) { sVs[i]->GetPoint().Make3d() ; }}

  void MergeShells
  (
    SmPolyShell * pShell,
    SmPolyShell * pOtherShell
  );

  SmStatus Mirror(const SmAxis2Placement & crMirrorPlane);

  // Move the faces from this into target brep
  SmStatus MoveFaces
  (
    SmTArray<SmPolyFace*> & rFacesToMove,             ///< [in] :                                                                   <br>
    SmPolyBrep            * pTargetBrep,              ///< [in] :                                                                   <br>
    SmMarkType              eMarkType=SM_MT_NOMARK    ///< [in] : uses without incrementing eMarkType value. marks all moved faces  <br>
  );
                                                                       

  // Copy the faces from this into target brep                                                           
  SmStatus CopyFaces
  (
    SmTArray<SmPolyFace*> & rFacesToMove,             ///< [in] :                                                                    <br>
    SmPolyBrep            * pTargetBrep,              ///< [in] :                                                                    <br>
    SmMarkType              eMarkType=SM_MT_NOMARK    ///< [in] : uses without incrementing eMarkType value. marks all copied faces  <br>
  ) ; 
                                                                       
  virtual void Notify
  (
    SmNotifyOperation eNotifyOperation,
    SmObject * pData1,
    SmObject * pData2,
    SmObject * pData3
  ) ;

  SmStatus OrderRadialEdges( SmPolyEdge *pEdge );

  SmStatus OrientedStitch
  (
    SmZoneTol3d dZoneTol3d,                   ///< [in] : min dist between distinct points       <br>
    SmBoolean   bEnableEdgeSplitting,         ///< [in] : TRUE =                                 <br>
    ULONG     & rlNumberEdgesStitched,            ///< [out]: Number of EdgePairs stitched               <br>
    ULONG     & rlNumberLaminaRemaining,      ///< [out]: Number of remaining lamina edges       <br>
    SmBoolean   bKeepTriangles = FALSE,       ///< [in] : default:[FALSE]                        <br>
    double    * pdOptMinStitchAngDeg = NULL,      ///< [in] : default:[NULL]                         <br>
    SmBoolean   bManifoldStitching = TRUE        ///< [in] : default:[TRUE]                         <br>
  );  

  SmStatus OrientedStitchWithCleanup
  (
    SmBoolean bIsSolidMesh,                      ///< [in] : FALSE = postprocess NonManifold objects with CleanSolidMesh(), TRUE = don't  <br>
    double  * pdOptMinStitchAngDeg = NULL,  ///< [out]: passed to OrientedStitch(), default:[NULL]                                      <br>
    SmBoolean bOptKeepTriangles = FALSE,         ///< [out]: passed to OrientedStitch(), default:[FALSE]                                     <br>
    SmBoolean bOptDoDecimation = FALSE           ///< [out]: passed to CleanSolidMesh(), default:[FALSE]                                  <br>
  );

  SmStatus OutputBrepMesh
  (
    SmTArray<ULONG>      & rPolygonVertexCounts,        ///< [out]: This array will contain the count for the number                      <br>
                                                        ///<      : of points in each polygon.  Typically this will be                    <br>
                                                        ///<      : 3.  The polygon vertices are accessed using the                       <br>
                                                        ///<      : following indices.  For example if the first polygon                  <br>
                                                        ///<      : has 3 vertices and the second has 4.  The first 3                     <br>
                                                        ///<      : elements of the indices array will index the points                   <br>
                                                        ///<      : used for the first polygon and the next four will be                  <br>
                                                        ///<      : for the second polygon.                                               <br>
    SmTArray<ULONG>      & rPolygonVertexIndices,       ///< [out]: Index for the vertices of each polygon.                               <br>
    SmTArray<ULONG>      & rPolygonNormalIndices,       ///< [out]: Index for the surface normals each polygon.                           <br>
    SmTArray<SmPoint3d>  & rPolygon3DPoints,            ///< [out]: ith edge->vertex loc    = rPolygon3DPoints[rPolygonVertexIndices[ii]] <br>
    SmTArray<SmVector3d> & rSurfaceNormals,             ///< [out]: ith edge->vertex normal = rSurfaceNormals [rPolygonNormalIndices[ii]] <br>
    SmTArray<SmFace*>    & rBrepFaces                   ///< [out]: Brep face corresponding to each polygon                               <br>
  );

  static SmStatus PolygonalBoolean
  (
    const SmContext        & crContext,                 ///< [in] :                                                           <br>
    SmTArray<SmPolyBrep *> & sPolyBreps,                ///< [in] :                                                           <br>
    ULONG                    lOperation,                ///< [in] : 0 - union, 1 - intersection, 2 - difference, 3 - merge    <br>
    SmBoolean                bImprinting,               ///< [in] : Imprint instead of full Boolean                           <br>
    SmBoolean                bCookieCutter,             ///< [in] : Cookie cutter instead of full Boolean                     <br>
    SmBoolean                bCleanupCoplanarOperands,  ///< [in] : Clean coplanar faces before boolean                       <br>
    SmBoolean                bCleanupCoplanarResults,   ///< [in] : Clean coplanar after boolean                              <br>
    SmPolyBrep            *& rpResult                   ///< [out]:                                                           <br>
  );                                                                                                               

  static SmStatus ReadFromSTLFile
  (
    const SmContext & crContext,                    ///< [in] : context for new object construction                              <br>
    const TCHAR     * cInputFileName,               ///< [in] : target file                                                      <br>
    SmPolyBrep     *& rpNewPolyBrep,                ///< [out]: PolyBrep created from InputFile data                             <br>
    SmFileType        eType = SM_ASCII,             ///< [in] : one of SM_ASCII or SM_BINARY                                     <br>
    SmBoolean         bReverseOrientation = FALSE,  ///< [in] : TRUE = Reverse triangle point orders upon read, FALSE = don't    <br>
    ULONG             lOptConnectivityType = 1      ///< [in] :  1 - No duplicate edges or vertices(Default).                    <br>
                                                    ///<      :  2 - Duplicate edges but no duplicate vertices.                  <br>
                                                    ///<      :  3 - Duplicate edges and duplicate vertices.                     <br>
  );    
                                                                                  
  static SmStatus ReadFromASCII
  (
    const SmContext & crContext,                         ///< [in] :        <br>
    const TCHAR     * cInputFileName,                    ///< [in] :        <br>
    SmPolyBrep     *& rpNewPolyBrep,                     ///< [in] :        <br>
    SmPolygonFileType eType,                             ///< [in] :        <br>
    SmBoolean         bReverseOrientation = FALSE,       ///< [in] :        <br>
    double          * pdOptMinStitchAngDeg = NULL,           ///< [in] :        <br>
    SmBoolean         bReadSolidMesh = FALSE             ///< [in] :        <br>
  );

  SmStatus ReadFromFile
  (                                                      ///< [in] :        <br>
    const SmContext & crContext,                         ///< [in] :        <br>
    const TCHAR     * cInputFileName,                    ///< [in] :        <br>
    SmFileType        eType                              ///< [in] :        <br>
  );

  SmBoolean HasDegenerateFaces( ULONG& rNumDegenFaces ) const;

  SmStatus RemoveDegenerateFaces();

  SmStatus RemoveVerticesBetweenColinearEdges();

  SmBoolean RemoveEdgeBetweenCoplanarFaces
  (
    SmPolyEdge * pPolyEdge,
    SmTArray<SmPolyEdge*> & rRemovedEdges
  );

  SmStatus RemoveCoincidentFaces();

  SmStatus RemoveInternalFaces();

  SmStatus RemoveSmallDetails(double d3DTolerance);

  void SetTolerance(SmZoneTol3d dZoneTol3d,                    // in :
                    SmBoolean   bUpdateOnlyIfLarger = TRUE ) ; // NotUsed: in :

  void SetOwnsContext (SmBoolean bOwnsContext)  { m_bOwnsContext  = bOwnsContext; }

  void SetOKBackPtrs  (SmBoolean bOKBackPtrs)   { m_bOKBackPtrs   = bOKBackPtrs; }

  SmStatus Stitch
  (
    SmZoneTol3d dZoneTol3d,                   ///< [in] : min dist between unnique points                                                       <br>
    SmBoolean   bManifoldStitching,           ///< [in] : TRUE  = don't allow spine edges, FALSE=do                                             <br>
    SmBoolean   bEnableEdgeSplitting,         ///< [in] : TRUE  = split edges when partially overlapping.                                       <br>
                                              ///<      : FALSE =                                                                               <br>
    SmBoolean   bTryTopologyStitchingFirst,   ///< [in] : TRUE  = Stitch PolyEdges sharing common PolyVertices prior to coincident stitching    <br>
                                              ///<      : FALSE = Stitch PolyEdges with coincident vertices to tolerance first                  <br>
                                              ///<      : Saves a lot of time when vertices are known to be stitched (STL files)                <br>
    ULONG     & rlNumberEdgesStitched,        ///< [out]: Number of edges stitched                                                              <br>
    ULONG     & rlNumberLaminaRemaining,      ///< [out]: Number of remaining lamina edges                                                      <br>
    SmBoolean   bKeepTriangles = FALSE        ///< [in] : TRUE  = retriangulte after splitting an edge keeping mesh all triangles               <br>
                                              ///<      : FALSE = no retriangulation after edge split                                           <br>
  );  

  SmStatus StitchEdgesTopological
  ( 
    SmTArray< SmPolyEdge* > & rEdges,                       ///< [in] : just so we don't have to collect them on every call                                           <br>
    SmZoneTol3d               dZoneTol3d,                   ///< NotUsed: [in] : min dist between distinct points                                                     <br>
    SmBoolean                 bManifoldStitching,           ///< [in] : TRUE = only stitch lamina PolyEdges into manifold PolyEdges                                   <br>
                                                            ///<      : FALSE= Allow manifold PolyEdges to be stitched into spine PolyEdges                           <br>
    ULONG                   & rnNumEdgesStitched,           ///< [out]: number of PolyEdges stitched with a call to GlueEdges()                                       <br>
    SmBoolean               & rbFailedManifoldStitch        ///< [out]: FALSE = All laminia PolyEdges stitched into manifold Polyedges,                               <br>
                                                            ///<      : TRUE  = found one or more lamina PolyEdges that did not stitch into manifold edges            <br>
  );

  SmStatus StitchEdgeTopological
  (
    SmPolyEdge * pPolyEdge,                                 ///< [in] : PolyEdge to match                                                           <br>
    SmBoolean    bManifoldStitching,                        ///< [in] : TRUE = only stitch lamina PolyEdges into manifold PolyEdges                 <br>
                                                            ///<      : FALSE= Allow manifold PolyEdges to be stitched into spine PolyEdges         <br>
    ULONG      & rlNumberEdgesStitched,                     ///< [out]: number of PolyEdges stitched with a call to GlueEdges()                     <br>
    SmTArray<SmPolyVertex*> * pDeletedPolyVertices = NULL,  ///< [out]: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]  <br>
    SmTArray<SmPolyVertex*> * pSurvivingVertices = NULL     ///< [out]: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]      <br>
  ) ; 

  SmStatus SimpleMerge
  (
    SmTArray<SmPolyFace*> & rFacesToMove,   ///< [in] : PolyFace list in this PolyBrep to move                     <br>
    SmPolyBrep            * pTargetBrep     ///< [out]: PolyBrep receiving faces from this PolyBrep                <br>
  ) const;

  SmStatus SmoothPolygons
  (
    SmTArray<SmPolyFace*> & crPolysToSmooth,      ///< [in] : List of PolyFaces to smooth                             <br>
    ULONG                   lSmoothingPasses,     ///< [in] : Number of iterations before quitting                    <br>
    double                  dSmoothingStepSize    ///< [in] : 1.0 = Move Vertex to Centroid each iteration            <br>
                                                  ///<      : 0.5 = Mover vertex half way to Centroid each iteration  <br>
  );  

  SmStatus SplitEdgesNearVertices
  (
    SmPolyFace              * pFace,               ///< [in] :        <br>
    double                    d3DTolerance,        ///< [in] :        <br>
    SmTArray<SmPolyVertex*> & rNewVertices         ///< [out]:        <br>
  );

  SmStatus SqueezeEdge
  (
    SmPolyEdge   * pEdgeToSqueeze,               ///< [in] : edge to remove                                     <br>
    SmPolyVertex * pSurvivingVertex,             ///< [in] : vertex to keep                                     <br>
    SmPolyVertex **pOptStaleDeletedVertex=NULL   ///< [out]: stale pointer to deleted vertex, NULL to ignore    <br>
  ); 

  // squeeze face when it contains only two PolyEdges and glue any attached radial partners
  //   gwc - should be extended to more general cases: face too small, sliver face, etc.
  SmStatus SqueezeFaceIfDegenerate
  (
    SmPolyFace             * pPolyFace,                        ///< [in] : target PolyFace to delete                                                       <br>
    double                    dNewTolerance,                   ///< NotUsed: [in] : not used                                                                        <br>
    SmBoolean               & rbWasSqueezed,                   ///< [out]: TRUE=PolyFace was squeezed, FALSE=not                                           <br>
    SmPolyEdge             *& rpEdge1Del,                      ///< [out]: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object   <br>
    SmPolyEdge             *& rpEdge2Del,                      ///< [out]: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object   <br>
    SmPolyEdge             *& rpEdgeSurvive1,                  ///< [out]: Surviving radial partner to deleted PolyEdge1, NULL=none                        <br>
    SmPolyEdge             *& rpEdgeSurvive2,                  ///< [out]: Surviving radial partner to deleted PolyEdge2, NULL=none                        <br>
    SmTArray<SmPolyVertex*> * pOptDeletedPolyVertices=NULL,    ///< [out]: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]          <br>
    SmTArray<SmPolyVertex*> * pOptSurvivingPolyVertices=NULL   ///< [out]: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]      <br>
  ); 

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  SmStatus TessellateWithQuads
  (
    double    dMaxQuadSize,            ///< [in] : approximate Max allowed tessellation PolyEdge length                                             <br>
    SmBoolean bCleanCoplanarEdges,     ///< [in] : TRUE = merge coplanar faces before tessellating, FALSE=don't                                     <br>
    SmBoolean bCleanOnlyInvisible,     ///< [in] : used when bCleanCoplanarEdges == TRUE,                                                           <br>
                                       ///<      : TRUE = only merge coPlanar faces across Invisible PolyEdges, FALSE = merge across all PolyEdges  <br>
    SmBoolean bDoSmoothing             ///< [in] : TRUE  = call SmoothPolygons() after TessellateWithQuads() on each PolyFace                       <br>
                                       ///< [in] : FALSE = don't 
  );

  SmStatus TrimWithPlane
  (
    const SmPoint3d  & crPlanePoint,          ///< [in] :        <br>
    const SmVector3d & crPlaneNormal,         ///< [in] :        <br>
    SmBoolean          bKeepPositiveSide,     ///< [in] :        <br>
    SmPolyBrep      *& rpResult               ///< [out]:        <br>
  );

  SmStatus ValidatePointers() const;

  SmStatus WriteToFile
  (
    const std::string& cOutputFileName,    ///< [in] :      <br>
    SmFileType eType,                      ///< [in] :      <br>
    SmBoolean bNewFile = TRUE              ///< [in] :      <br>
  ) const;

  SmStatus WriteToFile
  (
    const TCHAR * cOutputFileName,             ///< [in] :      <br>
    SmFileType    eType,                       ///< [in] :      <br>
    SmBoolean     bNewFile=TRUE                ///< [in] :      <br>
  ) const;

  SmStatus WriteToSTLFile
  (
    const TCHAR * cOutputFileName,              ///< [in] :    <br>
    SmFileType    eFileType                     ///< [in] :    <br>
  );

  SmStatus WriteToSTLFile
  (
    std::ostream & outputStream,                ///< [in] :     <br>
    SmFileType     eFileType                    ///< [in] :     <br>
  );

  // overloaded new and delete - allocate SmPolyBrep memory from overloaded SmObject::operator new
  void *operator new(size_t size, const SmContext & crContext);         // NotUsed: in : size
  void  operator delete(void *ptr) { smos_Free(ptr); ptr = NULL ; }

#ifndef SM_BORLAND
  void *operator new(size_t size);
  void  operator delete(void *ptr, const SmContext &) { smos_Free(ptr); ptr = NULL ; }
#endif // no SM_BORLAND

  SmDisplayList * Draw
  (
    SmBoolean       bAddToUIPickList=FALSE,   ///< [in] : TRUE = add PolyBrep to display list so it can be picked, FALSE= don't, default:[FALSE] <br>
    SmBoolean       bDrawRaw=TRUE,            ///< [in] : TRUE = draw pts in NativeSpace, default:[TRUE]                                         <br>
    SmBoolean       bDrawIn3D=FALSE,          ///< [in] : TRUE = draw pts after projecting through OrigSurf, default:[FALSE]                     <br>
    SmPlane       * pOptOutPlane=NULL,        ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]        <br>
    SmGfxArraySet * pOptGfxSet=NULL,          ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.        <br>
                                              ///<      : NULL to ignore. default:[NULL]                                                         <br>
    SmEdgeTopoType  eEdgeTopoType=SM_ET_ALL   ///< [in] : orof SM_ET_WIRE, SM_ET_LAMINA, SM_ET_MANIFOLD, SM_ET_SPINE, SM_ET_ALL                  <br>
  ) const ;

  SmDisplayList * DrawCoincidentVertices
  (
    SmBoolean       bDrawRaw=TRUE,            ///< [in] : TRUE = draw pts in NativeSpace, FALSE=don't, default:[TRUE]                                    <br>
    SmBoolean       bDrawIn3D=FALSE,          ///< [in] : TRUE = draw pts projected through PolyFace->OrigSurfs, FALSE=don't default:[FALSE]             <br>
    SmPlane       * pOptOutPlane=NULL,        ///< [in] : When bDrawRaw==TRUE, Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                <br>
    SmGfxArraySet * pOptGfxSet=NULL           ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore.<br> 
  ) const ;                                    

  SmDisplayList * DrawWithVertexNormals (
    SmBoolean       bAddToUIPickList=FALSE,   ///< [in] : TRUE = add PolyBrep to display list so it can be picked, FALSE= don't, default:[FALSE] <br>
    SmGfxArraySet * pOptGfxSet=NULL           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                              //      NULL to ignore. default:[NULL]
  ) const ;

  SmDisplayList * DrawWithFaceNormals (
    SmBoolean       bAddToUIPickList=FALSE,   ///< [in] : TRUE = add PolyBrep to display list so it can be picked, FALSE= don't, default:[FALSE] <br>
    SmBoolean       bForceRecompute=FALSE,    // in : TRUE = always recompute m_vNormal & m_dMaxDeviation          
                                              //      default:[FALSE]
    SmGfxArraySet * pOptGfxSet=NULL           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                              //      NULL to ignore. default:[NULL]
  ) const ;

  virtual void    Dump(ULONG) const;
  virtual void    Dump(TCHAR * message) const;

  void            DumpPolyFaces(ULONG lSampleFrequency=10,    SmBoolean bWithAttribs=TRUE) const ;  
  void            DumpPolyEdges(ULONG lSampleFrequency=10,    SmBoolean bWithAttribs=TRUE) const ;
  void            DumpPolyVertices(ULONG lSampleFrequency=10, SmBoolean bWithAttribs=TRUE) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPolyBrep,SmSAGObject,SmPolyBrep_TYPE);


} ; // end class SmPolyBrep

/*******************************************************************//**
PURPOSE: A structure to map the pointers of one copied PolyBrep topology
            to the source PolyBrep's topology object.

NOTES: this class is optionally used by SmPolyBrep Copy constructor
            and internally by SmPolyBrep::Merge() to remember how the
            topology objects of a copied PolyBrep map the the topology objects
            of the original PolyBrep.  Once either brep is modified the
            map will be obsolete.

            This class is declared as final. Can't derive a new class from
            this class without removing final.
 ***********************************************************************/
class SM_EXPORT SmCopyPolyBrepMap final : public SmObject
{
protected:
  SmPolyBrep  * m_pOrigPolyBrep ;        // Target PolyBrep
  SmPolyBrep  * m_pCopyPolyBrep ;        // Copy of Target PolyBrep

  // pointer maps for every topology kind of object
  SmMapPtrToPtr<SmPolyRegion, SmPolyRegion> m_sMapPolyRegions ;      // map Copy PolyRegions    to Orig PolyRegions
  SmMapPtrToPtr<SmPolyShell, SmPolyShell>   m_sMapPolyShells ;       // map Copy PolyShells     to Orig PolyShells
  SmMapPtrToPtr<SmPolyFace, SmPolyFace>     m_sMapPolyFaces ;        // map Copy PolyFaces      to Orig PolyFaces
  SmMapPtrToPtr<SmCPolyFace, SmCPolyFace>   m_sMapCPolyFaces ;       // map Copy CPolyFaces     to Orig PolyCFaces
  SmMapPtrToPtr<SmPolyLoop, SmPolyLoop>     m_sMapPolyLoops ;        // map Copy PolyLoops      to Orig PolyLoops
  SmMapPtrToPtr<SmPolyEdge, SmPolyEdge>     m_sMapPolyEdges ;        // map Copy PolyEdges      to Orig PolyEdges
  SmMapPtrToPtr<SmPolyVertex, SmPolyVertex> m_sMapPolyVertexs ;      // map Copy Polyvertices   to Orig PolyVertices

  SmTArray<SmPolyRegion*> m_sPolyRegionsIndex;  // map Copy PolyRegions  index  to Orig PolyRegions
  SmTArray<SmPolyShell*>  m_sPolyShellsIndex;   // map Copy PolyShells   index  to Orig PolyShells
  SmTArray<SmPolyFace*>   m_sPolyFacesIndex;    // map Copy PolyFaces    index  to Orig PolyFaces    
  SmTArray<SmCPolyFace*>  m_sCPolyFacesIndex;   // map Copy CPolyFaces   index  to Orig PolyCFaces   
  SmTArray<SmPolyLoop*>   m_sPolyLoopsIndex;    // map Copy PolyLoops    index  to Orig PolyLoops    
  SmTArray<SmPolyEdge*>   m_sPolyEdgesIndex;    // map Copy PolyEdges    index  to Orig PolyEdges    
  SmTArray<SmPolyVertex*> m_sPolyVertexsIndex;  // map Copy Polyvertices index  to Orig PolyVertices

public:  
  // constructor
  SmCopyPolyBrepMap()          : m_pOrigPolyBrep(NULL),                        
                                 m_pCopyPolyBrep(NULL)
                               { }                     
  
  // destructor                                             
  virtual ~SmCopyPolyBrepMap() { m_pOrigPolyBrep = NULL ;                           
                                 m_pCopyPolyBrep = NULL ;
                                 m_sMapPolyRegions.RemoveAll() ; 
                                 m_sMapPolyShells .RemoveAll() ; 
                                 m_sMapPolyFaces  .RemoveAll() ; 
                                 m_sMapCPolyFaces .RemoveAll() ; 
                                 m_sMapPolyLoops  .RemoveAll() ; 
                                 m_sMapPolyEdges  .RemoveAll() ; 
                                 m_sMapPolyVertexs.RemoveAll() ; 
                                 m_sPolyRegionsIndex.RemoveAll() ;
                                 m_sPolyShellsIndex .RemoveAll() ;
                                 m_sPolyFacesIndex  .RemoveAll() ;
                                 m_sCPolyFacesIndex .RemoveAll() ;
                                 m_sPolyLoopsIndex  .RemoveAll() ;
                                 m_sPolyEdgesIndex  .RemoveAll() ;
                                 m_sPolyVertexsIndex.RemoveAll() ;
                               }                     

  // simple access
  void        SetPolyBreps(SmPolyBrep *pOrigPolyBrep, 
                           SmPolyBrep *pCopyPolyBrep)    { m_pOrigPolyBrep = pOrigPolyBrep ;
                                                           m_pCopyPolyBrep = pCopyPolyBrep ;
                                                         }
  void        SetOrigPolyBrep(SmPolyBrep *pOrigPolyBrep) { m_pOrigPolyBrep = pOrigPolyBrep ; }
  void        SetCopyPolyBrep(SmPolyBrep *pCopyPolyBrep) { m_pCopyPolyBrep = pCopyPolyBrep ; }
  SmPolyBrep *GetOrigPolyBrep() const                    { return m_pOrigPolyBrep ; }
  SmPolyBrep *GetCopyPolyBrep() const                    { return m_pCopyPolyBrep ; }
  void        SetAt(SmTopology *pCopyObj, SmTopology *pOrigObj) ;
  void        SetAt(ULONG       lIndex,   SmTopology *pOrigObj) ;
  SmTopology *GetAt(SmTopology *pCopyObj) const ;
  SmTopology *GetAt(ULONG lIndex, SM_TYPE lType) const ;
  void        SetIndexSize(ULONG lSize, SM_TYPE lType);

  void Dump() const ;
  void Dump(SmBoolean bFull) const ;
  
} ; // end class SmCopyPolyBrepMap  

/*******************************************************************//**
PURPOSE: This represents a single triangle in a very compact form.

NOTES: 
  static class object - don't add virtual methods to SmTriangle
***********************************************************************/
class SM_EXPORT SmTriangle 
{
public:
    SmPoint3d m_sPoints[3];
} ;

// GWC:BIND_TEMPLATES_MOVE  BIND_TEMPLATES_MOVE  

/*******************************************************************//**
PURPOSE: Represents a very compact memory contiguous bag of 
    triangles.  You can't do much with triangle bags except sort
    with them.  

NOTES: This class is declared as final. Can't derive a new class from this
class without removing final.
***********************************************************************/
class SM_EXPORT SmTriangleBag final : public SmAObject
{
protected:
    SmTArray<SmTriangle*> m_vTriangles;
    SmMemBlockMgr         m_vTriMgr;    // manages block of SmTriangle objects
    SmExtent3d            m_vBBox;

public:
    // constructor
    SmTriangleBag(ULONG lNumTriPerBlock=1000);

    // destructor
    virtual ~SmTriangleBag();

    SmStatus AddTriangle
    (
      const SmPoint3d & rP1,           ///< [in] :      <br>
      const SmPoint3d & rP2,           ///< [in] :      <br>
      const SmPoint3d & rP3,           ///< [in] :      <br>
      SmTriangle     *& rpNewTriangle  ///< [out]:      <br>
    );

    SmStatus GetTriangle
    (
      long index,                     ///< [in] :      <br>
      SmPoint3d & rP1,                ///< [out]:      <br>
      SmPoint3d & rP2,                ///< [out]:      <br>
      SmPoint3d & rP3                 ///< [out]:      <br>
    ) const ;

    ULONG GetNumberTriangles() const { return m_vTriangles.GetSize(); }

    SmExtent3d GetBBox() { return m_vBBox; }

    SmStatus SplitBag
    (
      const SmContext & crContext,           ///< [in] :    <br>
      const SmExtent3d & crBBox,             ///< [in] :    <br>
      SmTriangleBag *& rpNewBagInBox,        ///< [out]:    <br>
      SmTriangleBag *& rpNewBagNotInBox      ///< [out]:    <br>
    );

    static SmStatus ReadFromSTLFile
    (
      const SmContext & crContext,           ///< [in] :    <br>
      const TCHAR * cInputFileName,          ///< [in] :    <br>
      SmTriangleBag *& rpNewTriangleBag,     ///< [in] :    <br>
      SmFileType eType = SM_ASCII            ///< [in] :    <br>
    );

    SmStatus CreatePolyBrep
    (
      const SmContext & crContext,          ///< [in] :     <br>
      SmPolyBrep *& rpNewPolyBrep           ///< [out]:     <br>
    ) const;

} ; // end class SmTriangleBag

// Misc. inlines that need to go first because called from
// other inlines.

// SmPolyVertex inlines 

// SmPolyEdge inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyEdge::SmPolyEdge(SmZoneTol3d dZoneTol3d) 
: m_pNextRadialE(NULL), 
  m_pLastRadialE(NULL), 
  m_sZoneTol3d(dZoneTol3d), 
  m_pNextEdgeAtV(NULL), 
  m_pPrevEdgeAtV(NULL), 
  m_pPolyVertex(NULL),
  m_pAuxData(NULL), 
  m_pOriginalEdgeuse(NULL),
  m_bVisibility(1),
  m_lUserLong1(0),
  m_lUserLong2(0)
{ 
  // inherited:
  // m_pListOwner = NULL ;
  // m_pNext      = NULL ;
  // m_pLast      = NULL ;

  m_pNextRadialE = this;
  m_pLastRadialE = this; 
  m_lIndexValue = SM_UNDEF_ULONG;

} // end SmPolyEdge::SmPolyEdge constructor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmStatus SmPolyEdge::GetEnds(SmPoint3d & rStart, SmPoint3d & rEnd) const
{ 
  rStart = m_pPolyVertex->m_vPoint; 
  rEnd   = GetEndPoint(); 
  return SM_SUCCESS; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyLoop * SmPolyEdge::GetPolyLoop() const 
{ 
  return SM_CAST_PTR(SmPolyLoop,m_pListOwner); 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyFace * SmPolyEdge::GetPolyFace() const
{ 
  return( m_pListOwner ? ((SmPolyLoop *)m_pListOwner)->GetPolyFace() : NULL ) ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyEdge::IsLamina() const 
{ 
  return (m_pNextRadialE == this) ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyEdge::IsManifold() const 
{ 
  return(   m_pNextRadialE != this && m_pNextRadialE->m_pNextRadialE == this) ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyEdge::IsSpine() const 
{ 
  return (   m_pNextRadialE != this
          && m_pLastRadialE != this
          && m_pNextRadialE != m_pLastRadialE ) ; 
}

// SmPolyFace inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyBrep * SmPolyFace::GetPolyBrep() const 
{ 
  return ( m_pListOwner ? ((SmPolyShell*)m_pListOwner)->GetPolyBrep() : NULL ) ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyFace::GetOKBackPtrs() const 
{ 
  return GetPolyBrep() ? GetPolyBrep()->GetOKBackPtrs() : FALSE ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmCPolyFace::GetOKBackPtrs() const 
{ 
  return m_vPolyFaces.GetSize() > 0 ? m_vPolyFaces[0]->GetOKBackPtrs() : FALSE ; 
}

/*******************************************************************//**
PURPOSE: Create and return a PolyLoop ready to receive sequence of PolyEdges

NOTES: Creates an empty PolyLoop properly attached to its owner PolyFace

  To assemble the PolyEdges of a PolyLoop in a given PolyFace do:
    SmPolyLoop *pPolyLoop = NULL ;
    pPolyFace->StartPolyEdgeLoop(pPolyLoop) ;
    for(ii=0;ii<lNumEdges;ii++)
      { pPolyLoop->AddPolyEdge(...) ; }
    pPolyLoop->FinishPolyEdgeLoop(pPolyBrep) ;
***********************************************************************/
inline SmStatus SmPolyFace::StartPolyEdgeLoop(SmPolyLoop *& rpNewPolyLoop)   
{ 
  rpNewPolyLoop = new (this->GetPolyBrep()) SmPolyLoop(this); 
  return SM_SUCCESS; 

} // end SmPolyFace::StartPolyEdgeLoop

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyFace::GetPolyLoops(SmTArray<SmPolyLoop*> & rPolyLoops) const
{ 
  GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rPolyLoops)); 
}

// SmPolyLoop inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyBrep * SmPolyLoop::GetPolyBrep() const 
{ 
  return( GetPolyFace() ? GetPolyFace()->GetPolyBrep() : NULL ) ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyLoop::IsPolyVertexLoop() const
{ 
  return(   ((SmPolyEdge *)m_pList)->GetStartPolyVertex() 
         == ((SmPolyEdge *)m_pList)->GetEndPolyVertex() ) ; 
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyLoop::GetPolyEdges(SmTArray<SmPolyEdge*> & rPolyEdges) const
{ 
  GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rPolyEdges)); 
}

// SmPolyShell inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyShell::SmPolyShell(SmPolyRegion * pPolyRegion)
{ 
  // inherited:     init:    after PostInsert
  // m_pList      = NULL ;
  // m_lListSize  = 0 ;
  // m_pListOwner = NULL     = pPolyRegion ;
  // m_pNext      = NULL     = NextShell or this when only one shell in pPolyRegion ;
  // m_pLast      = NULL     = LastShell or this when only one shell in pPolyRegion ;

  if (pPolyRegion) 
    { SE(pPolyRegion->PostInsert(this)); } 

} // end SmPolyShell::SmPolyShell constructor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyShell::GetPolyFaces(SmTArray<SmPolyFace*> & rPolyFaces) const
{ 
  GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rPolyFaces)); 
}

// SmPolyRegion inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyRegion::SmPolyRegion(SmPolyBrep *pPolyBrep)
{ 
  // inherited:     init:    after PostInsert
  // m_pList      = NULL ;
  // m_lListSize  = 0 ;   
  // m_pListOwner = NULL ;   = pPolyBrep ;  
  // m_pNext      = NULL ;   = NextRegion or this when only one region in pPolyBrep ;
  // m_pLast      = NULL ;   = LastRegion or this when only one region in pPolyBrep ;

  if(pPolyBrep)
    { pPolyBrep->PostInsert(this); }

} // end SmPolyRegion::SmPolyRegion constructor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyRegion::GetPolyShells( SmTArray<SmPolyShell*> & rPolyShells ) const
{
  GetAll( SM_REINTERPRET_CAST( SmTArray<class SmTopology*>&, rPolyShells ) );
}

// SmPolyBrep inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyBrep::GetPolyRegions( SmTArray<SmPolyRegion*> & rPolyRegions ) const
{
  GetAll( SM_REINTERPRET_CAST( SmTArray<class SmTopology*>&, rPolyRegions ) );
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyBrep::GetPolyVertices( SmTArray<SmPolyVertex*> & rPolyVertices ) const
{
  m_pPolyVertexListHead->GetAll( SM_REINTERPRET_CAST( SmTArray<class SmTopology*>&, rPolyVertices ) );
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyBrep::GetPolyPoints(SmTArray<SmPoint3d> & rPolyPoints) const
{ 
  SM_PTR_ARRAY   (sPolyVertices, SmPolyVertex, 512) ; 
  GetPolyVertices(sPolyVertices) ;
  rPolyPoints.SetSize(sPolyVertices.GetSize()) ;
  for(ULONG ii = 0; ii < sPolyVertices.GetSize(); ii++)
  {
    rPolyPoints[ii] = sPolyVertices[ii]->GetPoint();
  }
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmPolyBrep::GetCPolyFaces( SmTArray<SmCPolyFace*> & rCPolyFaces ) const
{
  m_pCPolyFaceListHead->GetAll( SM_REINTERPRET_CAST( SmTArray<class SmTopology*>&, rCPolyFaces ) );
}


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmStatus SmPolyBrep::MakeEmptyPolyFace
(
  SmZoneTol3d dZoneTol3d,
  SmPolyRegion * pPolyRegion,   // If not NULL poly face goes into this region
  SmPolyShell  * pPolyShell,    // If not NULL poly face goes into this shell
                                // If both are NULL poly face goes into poly brep
  SmPolyFace  *& rpNewPolyFace,
  SmVector3d   * pOptNormal
)
{
  rpNewPolyFace = new(this) SmPolyFace(dZoneTol3d,this,pPolyRegion,pPolyShell,pOptNormal);
  NER(rpNewPolyFace);
  return SM_SUCCESS;
}

// Dependent inlines

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyBrep * SmPolyRegion::GetPolyBrep() const
{
  return (SmPolyBrep*)m_pListOwner;
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyRegion * SmPolyShell::GetPolyRegion() const
{
  return (SmPolyRegion*)m_pListOwner;
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyBrep * SmPolyShell::GetPolyBrep() const
{
  return GetPolyRegion()->GetPolyBrep();
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyBrep * SmPolyEdge::GetPolyBrep() const
{
  return(m_pListOwner ? ((SmPolyLoop*)m_pListOwner)->GetPolyBrep() : NULL);
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyEdge::GetOKBackPtrs() const
{
  return GetPolyBrep() ? GetPolyBrep()->GetOKBackPtrs() : FALSE;
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmPolyBrep * SmPolyVertex::GetPolyBrep() const
{
  return(m_pPolyEdgeList ? m_pPolyEdgeList->GetPolyBrep() : NULL);
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmBoolean SmPolyVertex::GetOKBackPtrs() const
{
  return GetPolyBrep() ? GetPolyBrep()->GetOKBackPtrs() : FALSE;
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmVector3d SmTessCallback::GetNormal( SmPolyFace *pPFace ) const
{
  return pPFace->GetNormal( TRUE, FALSE );
}

#endif // !__SMPOLY_H__
