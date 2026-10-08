// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTree.h
* PURPOSE: Header file for SmTree object.
**********************************************************************/

#ifndef __SMTREE_H__
#define __SMTREE_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#include <SmNurbs.h>  // typedef  struct surface gw_SURFACE;
class SmBrep ;
class SmPolyVertex ;
class SmGfxArraySet ;
class SmBezierAux2d ;
class SmBezierPatch ;
class SmBezierAux1d ;
class SmBezierSpan ;
class SmObjectList ;

/*******************************************************************//**
PURPOSE: This enum defines what type of auxillary data, if any, 
         is attached to a given tree node in SmTreeNode::m_pData. 
            
NOTES:   Auxillary data is now derived from base class SmTreeNodeData 
         and SmTreeNode::m_pData is a SmTreeNodeData pointer.
         SmTreeNode::m_pData used to be a void * pointer and thus the
         need for the SmAuxDataType label.
***********************************************************************/
enum SmAuxDataType 
{ SM_AD_NONE,                 // NULL                   for uninit PointTrees and CurveCaches without aux data
  SM_AD_BEZIER_CURVE,         // class SmBezierSpan,    for SmCurveCache::BuildTree leaf nodes
  SM_AD_CURVE_DATA,           // class SmBezierAux1d,   for SmCurveCache::BuildTree internal nodes
  SM_AD_BEZIER_SURFACE,       // class SmBezierPatch,   for SmSurfaceCache::SubdivideNode: mark decomposition nodes
                              //                              that have bounding boxes and neighbor continuities.
  SM_AD_BEZIER_SURFACE_HEAD,  // class SmBezierAux2d,   for SmSurfaceCache::BuildTreeTop internal nodes
                              //                            Mark organize-only ancesters of actual decomposition nodes.
  SM_AD_SURFACE_DATA,         // not used in NMTlib         
  SM_AD_AUX_DATA,             // class SmBezierAux2d,   for SmSurfaceCache::SubdivideNodeFast() and
                              //                            SmSurfaceCache::SubdivideByBlock()
                              //                            SmTessSrfCache::BuildTreeWithSubdivision()
                              //                            surface sub-division trees
                              //                            Mark non-decomposition descendants of actual decomposition nodes
                              //                              created when adding trim curves to SurfaceCaches
  SM_AD_CUBEAUX_DATA,         // . . .                  not used in NMTlib
  SM_AD_OBJECT_LIST           // class SmObjectList     for single node PointTrees,
                              // NULL                   for spatial trees
} ;

/*******************************************************************//**
PURPOSE: A common base class for all tree node data blocks.

NOTES:  Every tree node points to a data block.  Different
  kinds of trees point to different kinds of tree node data.
  This base class contains no data or methods, but is used to
  associate the various kinds of tree node data to one another.

  static class object - don't add virtual methods to SmTreeNodeData
***********************************************************************/
class SM_EXPORT SmTreeNodeData
{
public:
    SmTreeNode  * m_pOwningTreeNode = NULL ; // back pointer to tree node which owns this data

  // no virtual functions allowed by memory block manager

} ; // end class SmTreeNodeData

/*******************************************************************//**
PURPOSE: The SmObjectList represents one item in a linked list of SmObjects
   contained within a SmTreeNode object as part of a SmTree Spatial Decomposition.

NOTES: This class is used as a SmTreeNode's auxiliary data when a SmTree
  Spatial Decomposition tree is being used to decompose a space full
  of SmObjects.  Typically used for SmBrep Vertex, Edge, and Face Cache Trees.

       Each SmObjectList object stores
         o. The SmObject pointer
         o. The SmObject's boundingBox
         o. The linked list's Next SmObjectList pointer.

  static class object - don't add virtual methods to SmObjectList
***********************************************************************/
class SM_EXPORT SmObjectList : public SmTreeNodeData
{
public:
  SmObjectList  * m_pNext = NULL ;
  SmExtent3d      m_sBBox;
  SmObject      * m_pObject = NULL ;

  void GetObjectList
  (
    SmTArray<SmObject *> &rObjectList,
    SmTArray<SmExtent3d> *pOptBBoxList = NULL
  )
  {
    rObjectList.ReSet();
    if(pOptBBoxList) pOptBBoxList->ReSet();
    SmObjectList *pOL = this;
    for(; pOL != NULL; pOL = pOL->m_pNext)
    {
      rObjectList.Add( pOL->m_pObject );
      if(pOptBBoxList) pOptBBoxList->Add( pOL->m_sBBox );
    }
  }

  // rtn: TRUE = Found obj in linked list, FALSE = didn't
  SmBoolean FindObject
  (
    SmObject     * pObject,                
    SmObjectList *& pPrevObjectList        ///< [in,out]: Prev SmObjectList member pointing to found ObjectList.  <br>
                                           ///<         : set to NULL when this SmObjectList contains pObject     <br>
                                           ///<         : or pObject not found                                    <br>
  )
  {
    pPrevObjectList = NULL;
    SmObjectList *pThisListObjectList = this;
    for(; pThisListObjectList != NULL;)
    {
      if(pThisListObjectList->m_pObject == pObject)
      {
        return(TRUE);
      }
      // set for next iter
      pPrevObjectList = pThisListObjectList;
      pThisListObjectList = pThisListObjectList->m_pNext;

    } // iter all pFoundNode's linke SmObjectList members
    return(FALSE);
  }

  SmBrep * GetBrep() const ; 

  // eff: draw extent's rectilinear solid outline
  SmDisplayList * Draw(const SmContext *pContext=NULL) const; // NotUsed: in: pContext 

} ; // end class SmObjectList

enum SmNodeGeomType 
{
  SM_NG_NONE,        // initialization default
  SM_NG_DEFAULT,     // used for all trees except the following
  SM_NG_LINE_SEG,    // label for curve and polyedge tree nodes   - helps SmRayTracer::FireRay function
  SM_NG_POLYGON      // label for surface and polyface tree nodes - helps SmRayTracer::FireRay function 
};

/*******************************************************************//**
PURPOSE: The tree node is an object used to create and store hierarchial
   information about something which has been broken down into smaller 
   pieces.

NOTES: This class cannot have virtual methods since SmGlobalSolver
       and SmPolySolver manages arrays of SmTreeNodes using smos_MemSet().
***********************************************************************/
class SM_EXPORT SmTreeNode
{
 public:
  SmTree         * m_pTree   = NULL ; // binary tree
  SmTreeNode     * m_pParent = NULL ; // family relations
  SmTreeNode     * m_pChild1 = NULL ; // Bot or Left      
  SmTreeNode     * m_pChild2 = NULL ; // Top or Right     
                  
  SmExtent3d       m_sBBox;         // this node's bounding box
                  
  SmAuxDataType    m_eAuxDataType = SM_AD_NONE ;  // specify the data type stored in m_pData
  SmTreeNodeData * m_pData = NULL ;               // auxillary data whose type is given by m_eAuxDataType.
                                                  // one of: SmBezierSpan*     for SM_AD_BEZIER_CURVE
                                                  //         SmBezierAux1d*    for SM_AD_CURVE_DATA
                                                  //                        
                                                  //         SmBezierPatch*    for SM_AD_BEZIER_SURFACE  
                                                  //         SmBezierAux2d*    for SM_AD_BEZIER_SURFACE_HEAD 
                                                  //         SmBezierAux2d*    for SM_AD_AUX_DATA
                                                  //         SmObjectList*     for SM_AD_OBJECT_LIST 
                                                  //         NULL              for SM_AD_NONE   
                                                                                           
  SmNodeGeomType   m_eGeomType = SM_NG_NONE ;     // specify the geometry type represented by this node
                                                  // oneof: SM_NG_DEFAULT,     // used for all trees except the following
                                                  //        SM_NG_LINE_SEG,    // label for curve and polyedge tree nodes - helps SmRayTracer::FireRay function
                                                  //        SM_NG_POLYGON      // label for surface and polyface tree nodes - helps SmRayTracer::FireRay function 
 public:
  // constructor
  SmTreeNode() : m_pTree(NULL),   
                 m_pParent(NULL), 
                 m_pChild1(NULL), 
                 m_pChild2(NULL), 
                 m_eAuxDataType(SM_AD_NONE),
                 m_pData(NULL),
                 m_eGeomType(SM_NG_NONE)
               { }

  void ReSet() { m_pTree        = NULL ; 
                 m_pParent      = NULL ; 
                 m_pChild1      = NULL ; 
                 m_pChild2      = NULL ; 
                 m_eAuxDataType = SM_AD_NONE ;
                 m_pData        = NULL ;
                 m_eGeomType    = SM_NG_NONE ;
               }

  SmBoolean IsDescendant( const SmTreeNode *pAncestor ) const
  {
    return((pAncestor == NULL) ? FALSE
         : (this == pAncestor) ? TRUE
         : (m_pParent == NULL) ? FALSE
         : (m_pParent->IsDescendant( pAncestor )));
  }

  const SmExtent3d & GetBoundingBox()        { return(m_sBBox) ; }

  // return SmCurveCache, SmSurfaceCache, SmBrepCache, or SmTessSrfCache when Tree is built for a cache, else return NULL
  const SmObject   * GetOwner() const ;

  // return SmCurve or SmSurface pointer when TreeNode is from SmCurveCache or SmSurfaceCache, else returns NULL
  const SmObject   * GetOwnerObject() const ;

  // Get array of all SmObjectList objects in this node
  // method:   when m_eAuxDataType == SM_AD_OBJECT_LIST
  //           add SmObjectList linked-list members under m_pData into rObjectsInNode 
  SmStatus           GetObjectList(SmTArray<SmObjectList*> & rObjectsInNode) const;

  // get all generations below this TreeNode. If TreeNode is a leaf node rOffspring is returned empty
  void               GetOffspring(SmTArray<SmTreeNode *> &rOffspring) const ;

  // walk Child/Parent pointers to get immediate neighbors, else return unmodified list.
  void               GetNeighbors(SmTArray<SmTreeNode *> &rNeighbors) const ;

  // get neighbors in just one direction 
  //   (NOTE: Only call functions with default values. These args are used internally to manage recursion.)
  void GetLeftNeighbors 
  (
    SmTArray<SmTreeNode *> &rNeighbors,        ///< [in ]: list to which neighobrs are added                      <br>
    ULONG lSplitBits = 0,                      ///< [in ]: track splits to get from current node to target node   <br>
    ULONG lSplitCount = 0                      ///< [in ]: split depth starting at 0, default:[0]                 <br>
  ) const ;

  void GetRightNeighbors
  (
    SmTArray<SmTreeNode *> &rNeighbors,        ///< [in ]: list to which neighobrs are added                      <br>
    ULONG lSplitBits = 0,                      ///< [in ]: track splits to get from current node to target node   <br>
    ULONG lSplitCount = 0                      ///< [in ]: split depth starting at 0, default:[0]                 <br>
  ) const ;

  void GetBotNeighbors  
  (
    SmTArray<SmTreeNode *> &rNeighbors,       ///< [in ]: list to which neighobrs are added                       <br>
    ULONG lSplitBits = 0,                     ///< [in ]: track splits to get from current node to target node    <br>
    ULONG lSplitCount = 0                     ///< [in ]: split depth starting at 0, default:[0]                  <br>
  ) const ;

  void GetTopNeighbors  
  (
    SmTArray<SmTreeNode *> &rNeighbors,      ///< [in ]: list to which neighobrs are added                         <br>
    ULONG lSplitBits = 0,                    ///< [in ]: track splits to get from current node to target node      <br>
    ULONG lSplitCount = 0                    ///< [in ]: split depth starting at 0, default:[0]                    <br>
  ) const ;

  // walk Child/Parent pointers to get all generations below the most distant ancestor to this node - gets whole family tree
  void               GetFamily(SmTArray<SmTreeNode *> &rNeighbors) const ;

  // for surface subdivisions return upper left most leaf node descendant, otherwise return Child1 most descendant
  SmTreeNode       * GetUpperLeftMostDescendant() const ;

  // GWC: Added as a workaround to make sure every node can compute a BBox.
  //      NOTE: This function goes away if we enforce an every node gets a shape rule.
  gw_SURFACE       * GetFirstBezierPatch() const ;
                                                  
  SmBezierAux2d    * GetBezierAux2d() const
  {
    return((m_eAuxDataType == SM_AD_BEZIER_SURFACE
            || m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD
            || m_eAuxDataType == SM_AD_AUX_DATA) ? (SmBezierAux2d *)m_pData : NULL);
  }

  SmBezierPatch    * GetBezierPatch() const
  {
    return((m_eAuxDataType == SM_AD_BEZIER_SURFACE) ? (SmBezierPatch *)m_pData : NULL);
  }

  SmBezierAux1d    * GetBezierAux1d() const
  {
    return((m_eAuxDataType == SM_AD_BEZIER_CURVE
            || m_eAuxDataType == SM_AD_CURVE_DATA) ? (SmBezierAux1d *)m_pData : NULL);
  }

  SmBezierSpan     * GetBezierSpan()  const
  {
    return((m_eAuxDataType == SM_AD_BEZIER_CURVE) ? (SmBezierSpan *)m_pData : NULL);
  }

  SmObjectList     * GetObjectList()  const
  {
    return((m_eAuxDataType == SM_AD_OBJECT_LIST) ? (SmObjectList *)m_pData : NULL);
  }


  // after changing a node's m_bIsMarked or m_eNodeClass call these functions to update its ancestors or descendants
  void               PropagateToParents() ;   // rules: ParentMarked: when all children Marked, 
                                              //        ParentIn    : when all children in, 
                                              //        ParentOut   : when all children out, 
                                              //        ParentOn    : when any child on or children a mix of in/out
  void               PropagateToChildren() ;  // rule: all children get parent's mark

 protected:
  // the following are protected because they are used internally for the GetXxxNeighbors functions
  //  and the lTargetSplitBits value is non-intuitive
  void GetRightSide(SmTArray<SmTreeNode *> &rNeighbors, 
                    ULONG lTargetSplitBits, ULONG lTargetSplitCount, 
                    ULONG lSplitBits = 0,   ULONG lSplitCount = 0) const ;                    
  void GetLeftSide (SmTArray<SmTreeNode *> &rNeighbors,                  
                    ULONG lTargetSplitBits, ULONG lTargetSplitCount, 
                    ULONG lSplitBits = 0,   ULONG lSplitCount = 0) const ;                    
  void GetBotSide  (SmTArray<SmTreeNode *> &rNeighbors,                  
                    ULONG lTargetSplitBits, ULONG lTargetSplitCount, 
                    ULONG lSplitBits = 0,   ULONG lSplitCount = 0) const ;                    
  void GetTopSide  (SmTArray<SmTreeNode *> &rNeighbors,                  
                    ULONG lTargetSplitBits, ULONG lTargetSplitCount, 
                    ULONG lSplitBits = 0,   ULONG lSplitCount = 0) const ;                    

 public:

  // recursive dump of this node and all its offspring
  void Dump
  (
    int lDepth=0,                    ///< [in ]: Recursive depth - used to add spacing                   <br>
                                     ///<      : to show tree structures as indented list.               <br>
                                     ///<      : default:[0]                                             <br>
    SmBoolean bRecurse=TRUE,         ///< [in ]: TRUE=recurse to all children, FALSE=don't               <br>                       
    int *lParentCount=NULL,          ///< [in,out]: accumulative count of parent nodes, NULL to ignore   <br>
    int *lLeafCount=NULL             ///< [in,out]: accumulative count of leaf nodes, NULL to ignore     <br>
  ) const ;     

  // dump just this node and its parent and children family members
  void            DumpFamily() const ;                    

  // recrusive draw of this node's bounding box and all its offspring
  SmDisplayList * DrawBoundingBox
  (
    double red=0.0,     
    double green= 0.3,    
    double blue=.7, 
    double red_inc=0.0, double green_inc=0.0, double blue_inc=0.0,
    double lineWidth=1.0,
    const SmContext * pContext=NULL,
    SmGfxArraySet   * pOptGfxSet=NULL
  ) const ;

  // draw just this node's 3d bounding box and mid point
  SmDisplayList * Draw
  (
    SmBoolean         bAutoColor=TRUE,      ///< [in ]: TRUE = color surf nodes by classification type                                    <br>
                                            ///<      : FALSE= use current color values                                                   <br>
    const SmContext * pContext = NULL,      ///< [in ]:                                                                                   <br>
    SmGfxArraySet   * pOptGfxSet = NULL     ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.   <br>
                                            ///<      : NULL to ignore. default:[NULL]                                                    <br>
  ) const ;

  const TCHAR   * GetTypeString() const { return _T("SmTreeNode") ; }

} ; // end class SmTreeNode

/*******************************************************************//**
PURPOSE: Represent the vertex position and connectivity information in a
         decomposition Tree.
 
 NOTES: The rectangle shaped leaf nodes of a decomposition tree are bounded
 by vertical and horizontal edges whose locations are defined by the
 corners of each node's UVDomain bounding box.  Those corner locations
 are called TreeVertices. Up to 4 different Tree Nodes can connect to one
 common TreeVertex.  When outputting a decomposition tree as
 a surface tessellation every TreeVertex ends up being a PolyBrep PolyVertex
 and sequences of PolyVertices that bound a single TreeNode end up being 
 the ordered PolyVertices of a PolyBrep PolyLoop.

 An array of SmTreeVertex objects is stored with each decomposition tree
 containing enough position and connectivity information to create 
 the PolyVertices and  PolyLoops needed to make a valid PolyBrep 
 tessellation model of a surface. Without the SmTreeVertex data, SmPolyBreps
 are created with coincident PolyVertices and then glued together
 with a call to SmPolyBrep::Stitch().  Running Stitch() is always expensive and when the
 surface has a 3d shape problem (like a bow-tie, or a folded sheet) can
 be confused resulting in invalid SmPolyBrep topology graphs.

  VERTEX CONNECTION MODEL: Vertex Index is its position in the SmTree:m_lNextVertex list. 
          2           Vertex Neighbor Indices: 0 = Below Neighbor Index      
        ^ | |                                  1 = Right Neighbor Index 
 pNode2 | | | pNode1                           2 = Above Neighbor Index    
     ---/ | \--->                              3 = Left  Neighbor Index  
   3 -----*------ 1          
     <--\ | /----     Next connections: When coming from right(1) next Vertex index is down  m_lNextIndex[0]
 pNode3 | | | pNode0                    When coming from above(2) next Vertex index is right m_lNextIndex[1] 
        | | v                           When coming from left (3) next Vertex index is up    m_lNextIndex[2]
          0                             When coming from below(0) next Vertex index is left  m_lNextIndex[3]
                    
                    SM_BIG_ULONG = Unused neighbor, e.g m_lNextIndex[0] == SM_BIG_ULONG means no neighbor below.

                    When seeking a NextVertex Neighbor and the 1st next connection is unused (index == SM_BIG_ULONG),
                    increment the search in the CW direction by one position.  It's a bug if that connection
                    is also unused. 
                    EXAMPLE: Find NextVertex index when coming from the right:
                               if      BelowIndex  != SM_BIG_ULONG, return BelowIndex
                               else if LeftIndex   != SM_BIG_ULONG, return LeftIndex
                               else    SignalError.

 EXAMPLE: Decomposition Tree with 14 vertices and 7 Nodes

  0        1    2    3        Node|  Boundary Vertices - start in upper right - move CCW
   *--------*----*----*         0 |  0  7  8  4  1
   |        |    |    |         1 |  1  4  5  2
   |       4|   5|   6|         2 |  2  5  6  3
   |        *----*----*         3 |  4  8  9  5
   |        |    |    |         4 |  5  9 10  6
  7|       8|   9|  10|         5 |  7 11 12  8
   *--------*----*----*         6 |  8 12 13 10  9
   |        |         |        
 11|      12|       13|    Vertex | m_lNextVertex table of vertex to vertex connections    
   *--------*---------*         0 |  7  1  -  -
                                1 |  4  2  -  0
                                2 |  5  3  -  1
                                3 |  6  -  -  2
                                4 |  8  5  1  -
                                5 |  9  6  2  4
                                6 | 10  -  3  5
                                7 | 11  8  0  -
                                8 | 12  9  4  7
                                9 |  - 10  5  8
                               10 | 13  -  6  9
                               11 |  - 12  7  -
                               12 |  - 13  8 11
                               13 |  -  - 10 12


   1. Every Node is bounded by 4 or more vertices.
   2. Every Vertex is connected to 4 or less vertices.
   3. When traversing the sequence of vertices that bound
      a node, the nextVertex after the current node is
      determined by the direction from which the current
      vertex is approached and the number and locations
      of that vertex to its neighbors.
      Example:  When coming to Vertex 9:
        From the left  (vertex  8) next Vertex is 5
        From above     (vertex  5) next Vertex is 10
        from the right (vertex 10) next Vertex is 8
   4. The connectivity model for the Nodes of the 
      DecompositionTree is stored as a list (one item for
      each Tree Vertex) of fixed size SmTreeVertex objects 
      which store their position (UV coordinates) and 4 
      connections to immediate neighbor vertices 
      (below, right, above, left).
   5. Vertex Neighbor connections are stored as ULONG
      indices rather than pointers. That concentrates all
      memory management for the connectivity graph to one
      SmTArray<ULONG> object contained within the SmTree object.
      One new item in the SmTArray<UONG> object is
      added each time one parent node is subdivided into 
      two children.
   6. The scheme allows each leaf node to generate its
      bounding loop of TreeVertices but does not allow
      internal nodes to generate their bounding loop data
      directly from the connectivity model.
   7. Unused Vertex connections are labeled with SM_BIG_ULONG values.
   8. The SplitNode() method manages the writing of the connectivity
       data for each new node created in splitting a parent
       into two children.

 static class object - don't add virtual methods to SmTreeVertex.  
 Virtual methods conflict with the methods in SmTArray<SmTreeVertex>
 that use memset() to clear memory - with SmTreeVertex virtual methods the
 virtual pointer tables get corrupted by SmTArray<SmTreeVertex>::ReSet() 
 calls and the like.
 ***********************************************************************/
class SM_EXPORT SmTreeVertex // : public SmObject - GWC: don't inherit from SmObject - needs to be static
{
 protected:

  SmPoint2d      m_vUVPoint ;      // UV Position of this Tree Vertex
                                   
  SmPolyVertex * m_pPolyVertex ;   // temp - used during tessellation, i.e. mapping a surface subdivision tree to a PolyBrep
                                   // Points to the SmPolyBrep->SmPolyVertex object created for this one TreeVertex.
                                   // Otherwise NULL.
                                   
  ULONG          m_lNextIndex[4] = {SM_UNDEF_ULONG} ; // connected vertex indices, ordered:[Below, Right, Above, Left]
                                                      //           NextIndices for Loop traversals:
                                                      //            0 = coming from right(1) next Vertex is Below(0) 
                                                      //            1 = coming from above(2) next Vertex is Right(1)
                                                      //            2 = coming from left (3) next Vertex is Above(2)
                                                      //            3 = coming from below(0) next Vertex is Left (3)
                                                      // 
                                                      // When a connection is unused (SM_BIG_ULONG) the
                                                      // next connection increments in the CW direction by one.
                                                      // e.g. if coming from below the Next Vertex is to the left and 
                                                      //   its index value is stored in m_lNextIndex[3]. If that dir 
                                                      //    is not used, m_lNextIndex[3] == SM_BIG_ULONG
                                                      //    try          m_lNextIndex[2], if that is == SM_BIG_ULONG
                                                      //                 that's a bug.

  SmTreeNode  * m_pTreeNode[4] = {NULL} ;   //  m_pTreeNode[0] = TreeNode to Below Right corner of this vertex
                                            //  m_pTreeNode[1] = TreeNode to Above Right corner of this vertex 
                                            //  m_pTreeNode[2] = TreeNode to Above Left  corner of this vertex 
                                            //  m_pTreeNode[3] = TreeNode to Below Left  corner of this vertex 
                                            //    NULL == not used
 public:                    

  // constructor
  SmTreeVertex
  (
    double     dUVPointU=SM_UNDEF_DOUBLE,       ///< [in ]: Tree Vertex U Position, SM_UNDEF_DOUBLE = uninit, default:[SM_UNDEF_DOUBLE]       <br>
    double     dUVPointV=SM_UNDEF_DOUBLE,       ///< [in ]: Tree Vertex V Position, SM_UNDEF_DOUBLE = uninit, default:[SM_UNDEF_DOUBLE]       <br>
    ULONG      lVertexBelowIndx  =SM_BIG_ULONG, ///< [in ]: Below   Tree neighbor indx, SM_BIG_ULONG=Not connected, default:[SM_BIG_ULONG]    <br>
    ULONG      lVertexToRightIndx=SM_BIG_ULONG, ///< [in ]: ToRight Tree neighbor indx, SM_BIG_ULONG=Not connected, default:[SM_BIG_ULONG]    <br>
    ULONG      lVertexAboveIndx  =SM_BIG_ULONG, ///< [in ]: Above   Tree neighbor indx, SM_BIG_ULONG=Not connected, default:[SM_BIG_ULONG]    <br>
    ULONG      lVertexToLeftIndx =SM_BIG_ULONG, ///< [in ]: ToLeft  Tree neighbor indx, SM_BIG_ULONG=Not connected, default:[SM_BIG_ULONG]    <br>
    SmTreeNode * pNodeToBelowRight=NULL,
    SmTreeNode * pNodeToAboveRight=NULL,
    SmTreeNode * pNodeToAboveLeft =NULL,
    SmTreeNode * pNodeToBelowLeft =NULL
  )
  { m_vUVPoint.x = dUVPointU ;
    m_vUVPoint.y = dUVPointV ;
    m_lNextIndex[0] = lVertexBelowIndx ;
    m_lNextIndex[1] = lVertexToRightIndx ;
    m_lNextIndex[2] = lVertexAboveIndx ;
    m_lNextIndex[3] = lVertexToLeftIndx ;
    m_pTreeNode[0] = pNodeToBelowRight ;
    m_pTreeNode[1] = pNodeToAboveRight ;
    m_pTreeNode[2] = pNodeToAboveLeft ;
    m_pTreeNode[3] = pNodeToBelowLeft ;
  }

  // operator =
  SmTreeVertex &operator=( const SmTreeVertex &crOther )
  {
    if(&crOther == this) return *this;
    // base values
    // m_cpContext     = crOther.m_cpContext ;

    // member values
    m_vUVPoint = crOther.m_vUVPoint;
    m_lNextIndex[0] = crOther.m_lNextIndex[0];
    m_lNextIndex[1] = crOther.m_lNextIndex[1];
    m_lNextIndex[2] = crOther.m_lNextIndex[2];
    m_lNextIndex[3] = crOther.m_lNextIndex[3];
    m_pTreeNode[0] = crOther.m_pTreeNode[0];
    m_pTreeNode[1] = crOther.m_pTreeNode[1];
    m_pTreeNode[2] = crOther.m_pTreeNode[2];
    m_pTreeNode[3] = crOther.m_pTreeNode[3];

    return *this;
  }

  // operator ==
  SmBoolean operator==( const SmTreeVertex &crOther )
  {
    if(&crOther == this) return TRUE;
    if(m_vUVPoint != crOther.m_vUVPoint)      return FALSE;
    if(m_lNextIndex[0] != crOther.m_lNextIndex[0]) return FALSE;
    if(m_lNextIndex[1] != crOther.m_lNextIndex[1]) return FALSE;
    if(m_lNextIndex[2] != crOther.m_lNextIndex[2]) return FALSE;
    if(m_lNextIndex[3] != crOther.m_lNextIndex[3]) return FALSE;
    if(m_pTreeNode[0] != crOther.m_pTreeNode[0])  return FALSE;
    if(m_pTreeNode[1] != crOther.m_pTreeNode[1])  return FALSE;
    if(m_pTreeNode[2] != crOther.m_pTreeNode[2])  return FALSE;
    if(m_pTreeNode[3] != crOther.m_pTreeNode[3])  return FALSE;
    return(TRUE);
  }

  // destructor
 ~SmTreeVertex() { }

  // get access     
  const SmPoint2d & GetUVPoint() const
  {
    return(m_vUVPoint);
  }

  void GetUVPoint( double &rPointU, double &rPointV ) const
  {
    rPointU = m_vUVPoint.x; 
    rPointV = m_vUVPoint.y;
  }


  SmPolyVertex * GetPolyVertex()  const
  {
    return m_pPolyVertex;
  }

  ULONG GetConnect
  ( 
    ULONG lDir      ///< [in ]: 0=Below, 1=Right, 2=Above, 3=Left       <br>
  ) const 
  {
    SM_ASSERT( lDir < 4 );
    return m_lNextIndex[lDir];
  }

  SmTreeNode * GetTreeNode
  ( 
    ULONG lDir      ///< [in ]: 0=BelowRight, 1=AboveRight, 2=AboveLeft, 3=BelowLeft    <br>
  ) const 
  {
    SM_ASSERT( lDir < 4 );
    return m_pTreeNode[lDir];
  }

  // init/set access 
  void SetUVPoint( double dUVPointU, double dUVPointV )
  {
    m_vUVPoint.x = dUVPointU;
    m_vUVPoint.y = dUVPointV;
  }

  void SetPolyVertex( SmPolyVertex *pPolyVertex )
  {
    m_pPolyVertex = pPolyVertex;
  }

  void InitConnects()
  {
    m_lNextIndex[0] = SM_BIG_ULONG;
    m_lNextIndex[1] = SM_BIG_ULONG;
    m_lNextIndex[2] = SM_BIG_ULONG;
    m_lNextIndex[3] = SM_BIG_ULONG;
    m_pTreeNode[0] = NULL;
    m_pTreeNode[1] = NULL;
    m_pTreeNode[2] = NULL;
    m_pTreeNode[3] = NULL;
  }

  void SetConnect
  ( 
    ULONG lDir,                   ///< [in ]: 0=Below, 1=Right, 2=Above, 3=Left  <br>
    ULONG lNextIndex              ///< [in ]: TreeVertex index to connect with   <br>
  )
  {
    SM_ASSERT( lDir < 4 );
    m_lNextIndex[lDir] = lNextIndex;
  }

  void SetTreeNode
  (
    ULONG lDir,                      ///< [in ]: 0=BelowLeft, 1=AboveLeft, 2=AboveRight, 3=BelowRight   <br>
    SmTreeNode *pTreeNode
  )
  {
    SM_ASSERT_BREAK( lDir < 4 );
    m_pTreeNode[lDir] = pTreeNode;
  }

  void SetTreeNodes
  (
    SmTreeNode *pToBelowRightNode,  ///< [in ]: TreeNode to BelowRight corner of target TreeVertex      <br>
    SmTreeNode *pToAboveRightNode,  ///< [in ]: TreeNode to AboveRight corner of target TreeVertex      <br>
    SmTreeNode *pToAboveLeftNode,   ///< [in ]: TreeNode to AboveLeft corner of target TreeVertex       <br>
    SmTreeNode *pToBelowLeftNode    ///< [in ]: TreeNode to BelowLeft corner of target TreeVertex       <br>
  )
  {
    m_pTreeNode[0] = pToBelowRightNode;
    m_pTreeNode[1] = pToAboveRightNode;
    m_pTreeNode[2] = pToAboveLeftNode;
    m_pTreeNode[3] = pToBelowLeftNode;
  }

  // utilities
  void Dump() const ;

} ; // end class SmTreeVertex

SM_TARRAY_TEMPLATE_PREDECLARATION(SmTreeVertex) ;

/*******************************************************************//**
PURPOSE: The tree object is used to hierarchially and spatially 
   decompose curves, surfaces, etc. into smaller subelements.
   The tree may then be traversed quickly by the solver when looking
   for solutions.

Decomposition Trees:
   A rectangular UV Space region can be decomposed into a set of tessellating
   rectangular nodes whose area is specified by a UVDomain BoundingBox.  Spatial
   decompositions of Surface UVDomains are used to divide a surface up into
   piecewise nearly linear patches that are then used for global solves (each
   piece is close enough to linear so that Newton-Raphson searches find solutions)
   and the basis for Face and Surface tessellations.
    1. Each node of the tree is associated with a bounding box.
    2. Each node's m_eAuxDataType == SM_AD_BEZIER_SURFACE or SM_AD_BEZIER_SURFACE_HEAD
    3. The parent's bounding box is the union of its two children's bounding box.
    4. Children bounding boxes tessellate their parent's UVDomain.  They don't overlap.
    5. The set of all leaf nodes completely tessellates the root node's UVDomain.

Spatial Trees:  
   A set of objects can be combined into a binary spatial tree.
    1. Each node of the tree is associated with a bounding box.
    2. Each node's m_eAuxDataType == SM_AD_OBJECT_LIST
    3. The parent's bounding box is a union of its two children's bounding boxes.
    4. Each object stored in the tree is associated with its own bounding box.
    5. Each object stored in the tree is stored in the node whose
         bounding box is the smallest one in the tree that completely contains
         the object's bounding box.  An object may end up stored in a leaf
         or an internal node.

    4. Each internal and leaf node is associated with a linked-list of 
       SmObjectList objects stored in m_pData.
       4.a. Each SmObjectList object contains a next pointer, a bounding box,
            and a pointer to an SmObject.

    5. The number of objects in a node's linked-list is limited by m_lMaxElementsPerNode.
    6. The minimum size of a leaf node is limited by m_dMinSizeRatio.  When a leaf node
         becomes so small that it can no longer be split, its link-list length may
         exceed the m_lMaxElementsPerNode limit.
    7. A node is split when its linked-list length first exceeds m_lMaxElementsPerNode and
         it becomes an internal node and never gets split again.  
         Objects whose bounding boxes are small enough to fit in an internal node's 
         bounding box but are too large to fit in one or the other of its children's 
         bounding boxes are stored in the internal node's ObjectList list.  An internal 
         node's link-list length may exceed the m_lMaxElementsPerNode limit.
    8. The size of a child node's bounding box is computed from the parent's as
         a reduction in size as given by m_dNodeReduction value.  When
         m_dNodeReduction = .5, both children bounding boxes will be 1/2 the size of
         the parents.  For smaller reductions, the childrens bounding boxes will
         overlap slightly.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmTree : public SmObject
{
  friend class SmCurveCache;
  friend class SmSurfaceCache;
  friend class SmTrimSrfCache;
  friend class SmTessSrfCache;
//    friend class SmMarchingCubes;
//    friend class SmCCRegionCache;
protected:
  ULONG         m_lMaxElementsPerNode;     // How many elements allowed per node before split
                                           // note: 1. used only for object spatial trees where
                                           //          each node's aux data type = SM_AD_OBJECT_LIST
                                           //          and             m_pData   = linked list of SmObjectList objects.
                                           //       2. When an added object increases a SmTreeNode's linked-list 
                                           //          length beyond this value, the node is split into two children
                                           //          nodes.
  double        m_dMinSizeRatio;           // Minimum size - for example 100 = 1/100th of original size
                                           // is the smallest node size.
                                           // note: 1. used only for object spatial trees. 
                                           //       2. prevents a node from being split if the children
                                           //          nodes will end up being too small
  double        m_dNodeReduction;          // Should be a number between 0.5 - equal node splits to
                                           // to 0.45 which is a slight overlap in the middle of the
                                           // original node by the two children.
                                           // note: 1. used only for object spatial trees.
                                           //       2. When a node is split into 2 children nodes
                                           //          each child node is given a bounding box that
                                           //          is about 1/2 the size of the parent box found by
                                           //          dividing the parent box's longest dimension.
                                           //          This number specifies the actual reduction size of 
                                           //          each child box.
                                             
  SmObject    * m_pOwner;                  // in SmCurveCache::m_pTree        set to owning SmCurveCache   object in SmCurveCache::BuildTree
                                           //    SmSurfaceCache::m_pTree      set to owning SmSurfaceCache object in SmSurfaceCache::BuildTreeBase
                                           //                                                                        SmTessSrfCache::BuildTreeWithSubdivision
                                           //    SmBrepCache::m_pVertexTree   set to owning SmBrepCache    object\  /SmBrepCache::BuildPolyTrees
                                           //    SmBrepCache::m_pCurveTree    set to owning SmBrepCache    object-in-    and
                                           //    SmBrepCache::m_pSurfaceTree  set to owning SmBrepCache    object/  \SmBrepCache::BuildTrees
                                           // in which case the value is the SmCurveCache, SmSurfaceCache, or SmBrepCache object.
                                             
  SmTreeNode  * m_pTopNode;                // ptr to this tree's top SmTreeNode object 
                                           //  each node contains: binary-tree pointers
                                           //                      bounding box
                                           //                      auxillary data type and pointer 
                                           //                      geometry type data
                                             
  SmMemBlockMgr m_sNodeMgr;                // memory for all SmTreeNode objects in this SmTree
                                           //  hint: number of nodes in this tree = m_sNodeMgr->m_lNumActiveElements
  SmMemBlockMgr m_sListMgr;                // memory for all SmObjectList objects in this SmTree
                                           //  note: 1. each SmTreeNode can store a linked list of
                                           //           SmObjectList objects under its m_pData pointer when
                                           //           its m_eAuxDataType == SM_AD_OBJECT_LIST
                                           //        2. Both leaf nodes and internal nodes can have a linked list of SmObjectList objects.
                                           //  hint: total number of objects in a spatial tree = m_sListMgr->m_lNumActiveElements

  SmTArray<SmTreeVertex> m_sTreeVertices ; // used for decomposition trees only - tree vertex position and 
                                           // conectivity data for connectivity model - see notes for
                                           // class SmTreeVertex.
public:
    // constructor: create an empty tree with a back pointer to the owner
    SmTree(SmObject *pOwner)                        
     : m_lMaxElementsPerNode(10),                  // note: Currently only used for starting the decomposition trees
       m_dMinSizeRatio(100.0),                     //       in the SmCurveCache and SmSurfaceCache objects.         
       m_dNodeReduction(0.4545),
       m_pOwner(pOwner),                           
       m_pTopNode(NULL)
     { }                

    // constructor: create a one node tree containing one point of given size
    SmTree                                        
    (
      const SmPoint3d & crPoint,                  ///< [in ]: center of tree's single node's bounding box                        <br>
      double        dSizeOfPoint,                 ///< [in ]: amount tree's single node's bounding box is expanded               <br>
      SmTreeNode   *pExistingNode=NULL,           ///< [in ]: prevent memory allocation by supplying one node to use             <br>
      SmObjectList *pOptExistingObjectList=NULL   ///< [in ]: when supplied, set node's m_eAuxDataType == SM_AD_OBJECT_LIST and  <br>
                                                  ///<      : set node's m_pData = pOptExistingObjectList                        <br>
    );

    // constructor: create a spatial tree of objects - each node has a spatial extent and a list of contained objects 
    SmTree                                                                          
      (const SmExtent3d & crSpatialBox,            ///< [in ]: bounding box expected to contain all objs placed in tree, automatically increased when too small  <br>
       ULONG              lNumObjectsPerBlock=64,  ///< [in ]: number of SmObjectList objs per block in block memory manager, m_sListMgr                         <br>
       ULONG              lNumNodesPreBlock = 16); ///< [in ]: number of SmTreeNode objs per block in block memory manager, m_sNodeMgr                           <br>

    // destructor
    virtual ~SmTree() { }

    // predicates
    SmBoolean       IntersectsBox(const SmExtent3d & cr3DBox) const;

    // Owner      : SmCurveCache, SmSurfaceCache, SmBrepCache or SmSmTessCache when Tree is built for a cache, else return NULL
    SmObject      * GetOwner()           const       { return m_pOwner; }        

    // OwnerObject: SmCurve or SmSurface when Tree is built for SmCurveCache or SmSurfaceCache, else return NULL
    const SmObject* GetOwnerObject()     const ;                                
    void            SetOwnerObject(SmObject *pOwner) { m_pOwner = pOwner ; }

    // Spatial Trees

    // add element to spatial tree 
      // side effect: When needed, SpatialTree->TopNode->BBox expanded to include crOBjectExtent.  (expensive)
      //              When cpObject == NULL, just the BBox expansion occurs if needed.
      // (save cost - avoid expansion - set init crSpatialBox size to include all objects added to Spatial Tree - but no bigger) 
    SmStatus       AddToSpatialTree
    (
      const SmExtent3d & crObjectExtent,         ///< [in ]: bounding box of object to place in tree                                                                        <br>
      SmObject         * cpObject,               ///< [in ]: object to place in tree, NULL=just check extent size and return                                                <br>
      SmNodeGeomType     eType = SM_NG_DEFAULT   ///< [in ]: oneof SM_NG_NONE,        // initialization default                                                             <br>
                                                 ///<      :       SM_NG_DEFAULT,     // used for all trees except the following                                            <br>
                                                 ///<      :       SM_NG_LINE_SEG,    // label for curve and polyedge tree nodes   - helps SmRayTracer::FireRay function    <br>
                                                 ///<      :       SM_NG_POLYGON      // label for surface and polyface tree nodes - helps SmRayTracer::FireRay function    <br>
                                                 ///<      :                          //     only used by SmBrepCache::BuildTrees()                                         <br>
    );     
              
    // remove SmObjectList obj containing pObject from Spatial Tree
    // rtn: TRUE when object was found and removed, else rtn FALSE
    SmBoolean      RmFromSpatialTree                                
    (
      const SmExtent3d & crObjectExtent,          ///< [in ]: bounding box of object to remove from tree                            <br>
      SmObject         * pObject                  ///< [in ]: object to remove from tree, NULL=just check extent size and return    <br>
    );

    SmStatus       GetObjectList     
    (
      SmTArray<SmObjectList*> & rObjectsInNode    ///< [out]: array of all ObjectList items found in this tree.              <br>
    ) const; 

    SmStatus       GetObjectListInBox
    (
      const SmExtent3d        & crBBox,             ///< [in ]: target extent                                                  <br>
      SmTArray<SmObjectList*> & rObjectsInBBox      ///< [out]: array of ObjectLists whose BBoxes intersect the target extent  <br>
    ) const;

    SmStatus       GetObjectsInBox   
    (
      const SmExtent3d    & crBBox,                 ///< [out]: all Objects whose BBoxes XSect BBox     <br>
      SmTArray<SmObject*> & rObjectsInBBox
    )     const;

    // TreeNodes
    ULONG          GetNodeCount()                                  const  { return(m_sNodeMgr.GetNumActiveElements()) ; }                
    SmTreeNode   * GetTopNode()                                    const  { return m_pTopNode; }                
    SmTreeNode   * FindNode(const SmExtent3d & crBox)              const; // find smallest treeNode which completely contains given box
    void           GetAllTreeNodes(SmTArray<SmTreeNode*> & rNodes) const; // get all nodes stored in this tree                
    SmTreeNode   * GetFirstLeafNode()                              const; // for iterations: Get 1st and next nodes                
    SmTreeNode   * GetNextLeafNode(SmTreeNode *pCurrentNode)       const; // for iterations: Get 1st and next nodes   
    void           ClearNodeMarks() ;                                     // set all surface subdivision node->SmBezierAux2d->m_bMarked states to FALSE

    void           SetMinSizeRatio (double dMinSizeRatio)                 { m_dMinSizeRatio  = dMinSizeRatio; }                
    void           SetNodeReduction(double dNodeReduction)                { m_dNodeReduction = dNodeReduction; }                
    void           SetMaxElementsPerNode(ULONG lMaxElementsPerNode)       { m_lMaxElementsPerNode = lMaxElementsPerNode; }                

// GWCTreeVertexTemp
//      // TreeVertices (only for SurfaceSubdivisions) 
//      void              ClearAllPolyVertices()                       { for(ULONG ii=0;ii<m_sTreeVertices.GetSize();ii++)
//                                                                         { m_sTreeVertices[ii].SetPolyVertex(NULL) ; }
//                                                                     }
//      ULONG             GetTreeVertexCount()                 const   { return(m_sTreeVertices.GetSize()) ; }
//      void              GetTreeVertexLoop(ULONG              indx,         ///< [in ]: Above Left TreeNode TreeVertex index, SM_BIG_ULONG = use rTreeNode->GetStartTreeVertexIndx() 
//                                          const SmTreeNode & rTreeNode,    ///< [in ]: node to trace - leaf or parent
//                                          SmTArray<ULONG>  & rVertexLoop)  ///< [out]: TreeVertex CCW index loop starting with Above Left corner indx
//                                         const ;
//  
//      SmPolyVertex    * GetTreeVertexPolyVertex(ULONG indx)  const   { SM_ASSERT_BREAK(indx < m_sTreeVertices.GetSize()) ;
//                                                                      return m_sTreeVertices[indx].GetPolyVertex() ;
//                                                                     } 
//      const SmPoint2d & GetTreeVertexUVPoint(ULONG indx)     const   { SM_ASSERT_BREAK(indx < m_sTreeVertices.GetSize()) ;
//                                                                       return m_sTreeVertices[indx].GetUVPoint() ; 
//                                                                     } 
//      void              SetTreeVertexPolyVertex(ULONG indx,
//                                       SmPolyVertex * pPolyVertex)   { SM_ASSERT_BREAK(indx < m_sTreeVertices.GetSize()) ;
//                                                                       return m_sTreeVertices[indx].SetPolyVertex(pPolyVertex) ;
//                                                                     } 
//      // Build TreeVertexMaps when splitting a parent into an array of children - increment m_sTreeVertices as needed
//      SmStatus          BuildTreeVertexMaps
//             (SmTreeNode * pNode,                            ///< [in ]: Parent Node being split before TreeVertices entries are updated or NULL for none
//              SmTArray<double> & rUSplits,                   ///< [in ]: rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//              SmTArray<double> & rVSplits,                   ///< [in ]: rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//              SmTArray<ULONG>  & rTreeVertexMap,             ///< [out]: Index for every U/V split TreeVertex intersection
//                                                             //       access Array[i,j]: k = ArrayIJToK(i,j,lNumSplitsU,lNumSplitsV)
//                                                             //       sized:[lNumSplitsU * lNumSplitsV]
//              SmTArray<ULONG>  & rTreeVertexPre,             ///< [out]:  closest TreeVertexIndex to This between This and Last U/V SplitPoints
//                                                             //       SM_BIG_ULONG = no TreeVertices between This and Last U/V SplitPoint.
//                                                             //       access Array[i,j]: k = BorderIJToK(i,j,UVDir,lNumSplitsU,lNumSplitsV)
//                                                             //       sized:[2 * (lNumSplitsU + lNumSplitsV)]
//              SmTArray<ULONG>  & rTreeVertexPost) ;          ///< [out]:  closest TreeVertexIndex to This between This and Next U/V SplitPoints
//                                                             //       SM_BIG_ULONG = no TreeVertices between This and Next U/V SplitPoint.
//                                                             //       access Array[i,j]: k = BorderIJToK(i,j,UVDir,lNumSplitsU,lNumSplitsV)
//                                                             //       sized:[2 * (lNumSplitsU + lNumSplitsV)]
//            
//      // Set TreeVertex Loop - only give corner TreeVertices
//      ULONG          SetTreeVertexLoop                       // rtn: TreeVertex Index of AboveLeft NodeLoop Start
//              (SmTreeNode       * pTreeNode,                 ///< [in ]: TreeNode owner of this TreeVertex Loop
//               SmExtent2d       & rUVDomain,                 ///< [in ]: UVDomain for Node with this loop
//               ULONG              iiIndx,                    ///< [in ]: Above Left Corner i of index[i,j] of new Node TreeVertex Loop
//               ULONG              jjIndx,                    ///< [in ]: Above Left Corner j of index[i,j] of new Node TreeVertex Loop 
//               SmTArray<double> & rUSplits,                  ///< [in ]: rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//               SmTArray<double> & rVSplits,                  ///< [in ]: rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//               SmTArray<ULONG>  & rTreeVertexMap,            ///< [in ]: Array made by BuildTreeVertexMaps call        
//               SmTArray<ULONG>  & rTreeVertexPre,            ///< [in ]: Array Made By BuildTreeVertexMaps call
//               SmTArray<ULONG>  & rTreeVertexPost) ;         ///< [in ]: Array Made By BuildTreeVertexMaps call
//                                                                                                                            
//      ULONG          SetRootTreeVertexLoop                   // rtn: TreeVertex Index of AboveLeft NodeLoop Start
//              (SmExtent2d       & rUVDomain,                 ///< [in ]: UVDomain for Node with this loop
//               ULONG              lALIndx,                   ///< [in ]: Above Left  corner of new Loop:[AL BL BR AR]
//               ULONG              lBLIndx,                   ///< [in ]: Below Left  corner of new Loop:[AL BL BR AR]
//               ULONG              lBRIndx,                   ///< [in ]: Below Right corner of new Loop:[AL BL BR AR]
//               ULONG              lARIndx,                   ///< [in ]: Above Right corner of new Loop:[AL BL BR AR]
//               SmTreeNode       * pTopNode) ;                ///< [in ]: pNode being bounded by this TreeVertex loop
//                                                                                                                                       
//      void          SetTreeVertexNodes
//              (ULONG       lIndx,                            ///< [in ]: Index of target TreeVertex in m_sTreeVertices
//               SmTreeNode *pToBelowRightNode,                ///< [in ]: TreeNode to BelowRight corner of target TreeVertex
//               SmTreeNode *pToAboveRightNode,                ///< [in ]: TreeNode to AboveRight corner of target TreeVertex
//               SmTreeNode *pToAboveLeftNode,                 ///< [in ]: TreeNode to AboveLeft corner of target TreeVertex
//               SmTreeNode *pToBelowLeftNode)                 ///< [in ]: TreeNode to BelowLeft corner of target TreeVertex
//                                                             { SM_ASSERT_BREAK(lIndx < m_sTreeVertices.GetSize()) ;
//                                                               m_sTreeVertices[lIndx].SetTreeNodes(pToBelowRightNode,
//                                                                                                   pToAboveRightNode,
//                                                                                                   pToAboveLeftNode, 
//                                                                                                   pToBelowLeftNode) ;
//                                                             }
//  

// GWCTreeVertexTemp
//      // BEGIN TREEVERTEX INTERNAL HELPER METHODS
//   protected:
//  
//      // Set all treeNode pointers for one connected loop
//      void SetVertexLooptreeNodes(ULONG        indx,             ///< [in ]: AboveLeft corner TreeVertex index of desired loop
//                                  SmTreeNode * pLeafTreeNode) ;  ///< [in ]: TreeNode being circumscribed by this TreeVertex Loop
//   
//      // set m_sTreeVertices array size
//      void IncTreeVertexCount(ULONG lInc) ; // eff: inc m_sTreeVertices length, init new member connections to NotUsed
//      void SetTreeVertexCount(ULONG lCnt) ; // eff: set m_sTreeVertices length, init new member connections to NotUsed
//  
//      // New TreeVertex Index maps, New Indices, Border PreConnects, Border PostConnects
//  
//      // internal index access to all TreeVertexIndex array sTreeVertexMap
//      ULONG ArrayIJToK(ULONG iiIndx,                         ///< [in ]: ii of TreeVertex Array(ii,jj)
//                       ULONG jjIndx,                         ///< [in ]: jj of TreeVertex Array(ii,jj)
//                       ULONG lNumSplitsU,                    ///< [in ]: Total Number of U Splits in TreeVertex Array(ii,jj)
//                       ULONG lNumSplitsV) const              ///< [in ]: Total Number of V Splits in TreeVertex Array(ii,jj)
//                                                             { SM_ASSERT(iiIndx < lNumSplitsU) ;
//                                                               SM_ASSERT(jjIndx < lNumSplitsV) ;
//                                                               return ( jjIndx + iiIndx * lNumSplitsV) ; 
//                                                             } 
//                                                             
//      // index access to border Last/Next TreeVertexIndex arrays sTreeVertexPre and sTreeVertexPost
//      ULONG BorderIJToK(ULONG iiIndx,                        ///< [in ]: ii of TreeVertex Array(ii,jj)
//                        ULONG jjIndx,                        ///< [in ]: jj of TreeVertex Array(ii,jj)
//                        SmSurfParamType UVDir,               ///< [in ]: SM_SP_U=on V Min/Max in U dir, SM_SP_V=on U Min/Max in V dir.
//                        ULONG lNumSplitsU,                   ///< [in ]: Total Number of U Splits in TreeVertex Array(ii,jj)
//                        ULONG lNumSplitsV) const            ///< [in ]: Total Number of V Splits in TreeVertex Array(ii,jj)
//                                                             { SM_ASSERT(   (iiIndx == 0 || iiIndx == lNumSplitsU - 1)  
//                                                                         || (jjIndx == 0 || jjIndx == lNumSplitsV - 1)) ;
//                                                               return (  ((UVDir)==SM_SP_V) 
//                                                                       ? ((iiIndx == 0) ? jjIndx : jjIndx + lNumSplitsV)  
//                                                                       : (  2*lNumSplitsV 
//                                                                          + ((jjIndx == 0) ? iiIndx 
//                                                                                           : iiIndx + lNumSplitsU)) ) ;
//                                                             }
//      ULONG GetTreeVertexIndex                 
//             (ULONG iiIndx,                                  ///< [in ]: ii of TreeVertex Array(ii,jj)
//              ULONG jjIndx,                                  ///< [in ]: jj of TreeVertex Array(ii,jj)
//              ULONG lNumSplitsU,                             ///< [in ]: Total Number of U Splits in TreeVertex Array(ii,jj)
//              ULONG lNumSplitsV,                             ///< [in ]: Total Number of V Splits in TreeVertex Array(ii,jj)
//              SmTArray<ULONG> & rTreeVertexMap) const        ///< [in ]: Array made by BuildTreeVertexMaps call        
//                                                             { return( rTreeVertexMap[ArrayIJToK(iiIndx,jjIndx,lNumSplitsU,lNumSplitsV)] ) ; }
//      ULONG GetPreTreeVertexIndex
//             (ULONG iiIndx,                                  ///< [in ]: ii of TreeVertex Array(ii,jj)
//              ULONG jjIndx,                                  ///< [in ]: jj of TreeVertex Array(ii,jj)
//              ULONG lCornerIndx,                             ///< [in ]: oneof 0=ALCorner,1=BLCorner,2=BRCorner,3=ARCorner
//              ULONG lNumSplitsU,                             ///< [in ]: Total Number of U Splits in TreeVertex Array(ii,jj)
//              ULONG lNumSplitsV,                             ///< [in ]: Total Number of V Splits in TreeVertex Array(ii,jj)
//              SmTArray<ULONG> & rTreeVertexMap,              ///< [in ]: Array made by BuildTreeVertexMaps call        
//              SmTArray<ULONG> & rTreeVertexPre,              ///< [in ]: Array Made By BuildTreeVertexMaps call
//              SmTArray<ULONG> & rTreeVertexPost) const ;     ///< [in ]: Array Made By BuildTreeVertexMaps call
//  
//      ULONG GetPostTreeVertexIndex
//             (ULONG iiIndx,                                  ///< [in ]: ii of TreeVertex Array(ii,jj)
//              ULONG jjIndx,                                  ///< [in ]: jj of TreeVertex Array(ii,jj)
//              ULONG lCornerIndx,                             ///< [in ]: oneof 0=ALCorner,1=BLCorner,2=BRCorner,3=ARCorner
//              ULONG lNumSplitsU,                             ///< [in ]: Total Number of U Splits in TreeVertex Array(ii,jj)
//              ULONG lNumSplitsV,                             ///< [in ]: Total Number of V Splits in TreeVertex Array(ii,jj)
//              SmTArray<ULONG> & rTreeVertexMap,              ///< [in ]: Array made by BuildTreeVertexMaps call        
//              SmTArray<ULONG> & rTreeVertexPre,              ///< [in ]: Array Made By BuildTreeVertexMaps call
//              SmTArray<ULONG> & rTreeVertexPost) const ;     ///< [in ]: Array Made By BuildTreeVertexMaps call
//      
//      // more TreeVertex helper methods                                                            
//      ULONG                    ToDir2FromDir(ULONG lFromDir) const      { return((lFromDir+2)%4) ; }
//      SmTArray<SmTreeVertex> & GetTreeVertexArray()                     { return( m_sTreeVertices ) ; }
//      SmTreeVertex           & GetTreeVertex(ULONG  indx)               { return( m_sTreeVertices[indx] ) ; }
//      ULONG                    GetNextTreeVertex                        // rtn: next TreeVertex index in a CCW Loop traversal
//                                 (ULONG              indx,              ///< [in ]: Index of Vertex to query
//                                  ULONG              lFromDir,          ///< [in ]: 0=FromBelow, 1=FromRight, 2=FromAbove, 3=FromLeft 
//                                  ULONG            & lToDir,            ///< [out]: 0=ToBelow,   1=ToRight,   2=ToAbove,   3=ToLeft
//                                  const SmTreeNode * pOptTreeNode=NULL) ///< [in ]: Optional node to trace - use for parent nodes, NULL=trace leaf node
//                                 const ;                                
//                                                                        
//      ULONG                    FindCoincidentTreeVertexIndx             // rtn: TreeVertex indx coincident with UVPoint, SM_BIG_ULONG = No Coin Vert
//                                 (double dUVPointU,                     ///< [in ]: UVPoint U position
//                                  double dUVPointV,                     ///< [in ]: UVPoint V position
//                                  SmTArray<ULONG> * pOptVertexLoop)     ///< [in ]: opt list of indices to search, NULL=search all
//                                 const ;                                
//                                                                        
//      void                     FindNeighborTreeVertexIndx               // rtn: TreeVertex Indx next to UVPoint in given direction, SM_BUG_ULONG = No Next Vert
//                                 (double            dUVPointU,          ///< [in ]: UVPoint U position
//                                  double            dUVPointV,          ///< [in ]: UVPoint V position
//                                  SmTArray<ULONG> * pOptVertexLoop,     ///< [in ]: opt list of indices to search, NULL=search all
//                                  SmSurfParamType   eSurfDir,           ///< [in ]: oneof SM_SP_U or SM_SP_V
//                                  double            dLastParam,         ///< [in ]: Last SearchInterval:[LastParam UVPointParam], SM_BIG_DOUBLE to ignore
//                                  double            dNextParam,         ///< [in ]: Next SearchInterval:[UVPointParam NextParam], SM_BIG_DOUBLE to ignore
//                                  ULONG           & rLastIndx,          ///< [out]: Indx closest to UVPoint in given direction, SM_BIG_ULONG = None
//                                  ULONG           & rNextIndx)          ///< [out]: Indx closest to UVPoint in given direction, SM_BIG_ULONG = None
//                                 const ;                                
//                                                                        
//      void                     SetTreeVertex                            // eff: init TreeVertex[indx] UVPos - grow TreeVertex array size if needed.
//                                 (ULONG  indx,                          ///< [in ]: indx of TreeVertex to init
//                                  double dUVPointU,                     ///< [in ]: U of PointUV
//                                  double dUVPointV) ;                   ///< [in ]: V of PointUV
//                                                                        
//      void                     ConnectTreeVertex                 // eff: from PrevIndx <-> lNextVertexIndx 
//                                                                 //      to   PrevIndx <-> indx <-> lNextVertexIndx
//                                  (ULONG indx,                   ///< [in ]: Index of TreeVertex to connect
//                                   ULONG lDir,                   ///< [in ]: ToDir oneof:0=Below,1=Right,2=Above,3=Left
//                                   ULONG lNextVertexIndx) ;      ///< [in ]: TreeVertex indx in the given direction
//      
//      void DumpTreeVertices() const ;
//      SmBoolean AssertTreeVertices() const ;
//  
//      // end TreeVertex helper methods
 public:

    // maintenance
    ULONG             GetMemoryUsed(ULONG &rlMemoryAllocated) const ;
                    
    SmDisplayList   * DrawNodeBoundingBoxes(double red=0,       double green=.3,      double blue=.7,
                                            double red_inc=0.0, double green_inc=0.0, double blue_inc=0.0) ;

    // return TRUE when object is tested as valid
    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                                ///<      : default:[SM_LEVEL_0]                                                                               <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< NotUsed: [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
    )  const ;

    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmTree,SmObject,SmTree_TYPE);

} ; // end class SmTree


#endif // !__SMTREE_H__


