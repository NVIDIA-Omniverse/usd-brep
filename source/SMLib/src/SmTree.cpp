// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTree.cpp
* PURPOSE: Source file for SmTree methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmTree.h>
#include <SmGraphicsExtern.h>
#include <SmSArray.h>
#include <SmMath.h>   // includes SmTol.h
#include <SmGraphicsOutput.h>
#include <SmCurveCache.h>
#include <SmSurfaceCache.h>
#include <SmBSplineSurface.h>

#include <SmBrep.h>
#include <SmFace.h>
#include <SmEdge.h>
#include <SmVertex.h>
#include <SmBrepCache.h>

#include <SmSurface.h>
#include <SmCurve.h>
#include <SmCurveCache.h>
#include <SmSurfaceCache.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmBrep * SmObjectList::GetBrep()
 const
{
  if(m_pObject == NULL)                             return (NULL) ; 

  else if(m_pObject->IsKindOf(SmBrep_TYPE))         return ((SmBrep *)m_pObject)->GetBrep() ; 
  else if(m_pObject->IsKindOf(SmFace_TYPE))         return ((SmFace *)m_pObject)->GetBrep() ;
  else if(m_pObject->IsKindOf(SmSurface_TYPE))      return (  ((SmSurface *)m_pObject)->GetOwner() 
                                                            ? ((SmFace *)((SmSurface *)m_pObject)->GetOwner())->GetBrep()
                                                            : NULL ) ;
  else if(m_pObject->IsKindOf(SmEdge_TYPE))         return ((SmEdge *)m_pObject)->GetBrep() ;
  else if(m_pObject->IsKindOf(SmCurve_TYPE))        return (  ((SmCurve *)m_pObject)->GetOwner() 
                                                            ? (  ((SmCurve *)m_pObject)->GetOwner()->IsKindOf(SmEdge_TYPE)
                                                               ? ((SmEdge *)((SmCurve *)m_pObject)->GetEdge())->GetBrep()
                                                               : NULL)
                                                            : NULL ) ;
  else if(m_pObject->IsKindOf(SmVertex_TYPE))       return ((SmVertex *)m_pObject)->GetBrep() ;
  else if(m_pObject->IsKindOf(SmBrepCache_TYPE))    return ((SmBrepCache *)m_pObject)->GetBrep() ;
  else if(m_pObject->IsKindOf(SmCurveCache_TYPE))   return ((SmCurveCache *)m_pObject)->GetBrep() ;
  else if(m_pObject->IsKindOf(SmSurfaceCache_TYPE)) return ((SmSurfaceCache *)m_pObject)->GetBrep() ;
  else return(NULL) ;

} // end SmObjectList::GetBrep

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmObjectList::Draw
  (const SmContext *pContext)   // NotUsed: in : pContext
 const
{
  SM_REF1(pContext) ; 
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  smgfx_Open(smgfx_GetRuleColor());
  smgfx_OutputColor(smgfx_GetColor()) ;

  m_sBBox.Draw() ;

  if(m_pObject != NULL)
    {
      if(m_pObject->IsKindOf(SmSurface_TYPE))      ((SmSurface *)m_pObject)->Draw() ;
      else if(m_pObject->IsKindOf(SmCurve_TYPE))        ((SmCurve *)m_pObject)->Draw() ;
      else if(m_pObject->IsKindOf(SmBrep_TYPE))         ((SmBrep *)m_pObject)->Draw() ; 
      else if(m_pObject->IsKindOf(SmFace_TYPE))         ((SmFace *)m_pObject)->Draw() ;
      else if(m_pObject->IsKindOf(SmEdge_TYPE))         ((SmEdge *)m_pObject)->Draw() ;
      else if(m_pObject->IsKindOf(SmVertex_TYPE))       ((SmVertex *)m_pObject)->Draw() ;
      else if(m_pObject->IsKindOf(SmBrepCache_TYPE))    ((SmBrepCache *)m_pObject)->Draw() ;

      
      else if(m_pObject->IsKindOf(SmCurveCache_TYPE))   ((SmCurveCache *)m_pObject)->Draw() ;
      else if(m_pObject->IsKindOf(SmSurfaceCache_TYPE)) ((SmSurfaceCache *)m_pObject)->Draw() ;
    }

  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmObjectList::Draw

/*******************************************************************//**
PURPOSE: Constructor which creates a tree corresponding to  a
   single point of a given size.  This tree will only have one node.

NOTES: Note that an optimization may be used to reduce
   memory allocation by sending in an existing node.  That way 
   we can construct a PointTree without any memory allocation.
***********************************************************************/
SmTree::SmTree
  (const SmPoint3d & crPoint,                 // in : center tree bounding box on this point      
   double            dPointSize,              // in : 1/2 size of box in all directions
   SmTreeNode       *pOptExistingNode,        // in : Point tree into array of already allocated TreeNodes
                                              //      NULL=use m_sNodeMgr to alloc 1 starter node - all future
                                              //           node growth will be allocated inside m_sNodeMgr.
                                              //      default:[NULL]
   SmObjectList     *pOptExistingObjectList)  // in : When not NULL,
                                              //      Set the top tree node's type to SM_AD_OBJECT_LIST
                                              //      and set this Object List's bounding box to a point centered
                                              //      on crPoint.
                                              //      NULL = don't touch the top Node
                                              //      default:[NULL]
 : m_lMaxElementsPerNode(10), 
   m_dMinSizeRatio(100.0), 
   m_dNodeReduction(0.4545), 
   m_pOwner(NULL)
{
    SmTreeNode *pNode;
    if (pOptExistingNode) { pNode = pOptExistingNode;
                          }
    else                  { m_sNodeMgr.Initialize(ALIGN_SIZE(sizeof(SmTreeNode)),5);
                            pNode = (SmTreeNode*)m_sNodeMgr.GetAt(0);
                          }
    pNode->m_eAuxDataType = SM_AD_NONE;
    pNode->m_pData        = NULL;
    pNode->m_pTree        = this;
    pNode->m_pChild1      = NULL;
    pNode->m_pChild2      = NULL;
    pNode->m_pParent      = NULL;
    pNode->m_sBBox        = SmExtent3d(crPoint);
    pNode->m_sBBox.ExpandAbsolute(dPointSize);
    m_pTopNode = pNode;

    if (pOptExistingObjectList) 
      {
        pNode->m_pData                    = pOptExistingObjectList;
        pNode->m_eAuxDataType             = SM_AD_OBJECT_LIST;
        pOptExistingObjectList->m_pNext   = NULL;
        pOptExistingObjectList->m_sBBox   = SmExtent3d(crPoint);
        pOptExistingObjectList->m_pObject = NULL;
      }

} // end SmTree::SmTree single point constructor

/*******************************************************************//**
PURPOSE: Constructor to be used if the tree is going to be a spatial
    tree.

NOTES: The bounding box should be larger than that of any of 
    the elements to be added to the spatial tree later on.

RETURNS - a Single Node Object Tree with its Bounding Box = crSpatialBox
***********************************************************************/
SmTree::SmTree
  (const SmExtent3d & crSpatialBox,        // in : bounding box large enough to hold all objects in tree
   ULONG              lNumObjectsPerBlock, // in : A good size for this is 1/10 number of
                                           //      expected objects.
   ULONG              lNumNodesPerBlock)   // in : A good size for this is 1/100 number of expected
                                           //      objects.
 : m_lMaxElementsPerNode(10), 
   m_dMinSizeRatio(100.0), 
   m_dNodeReduction(0.4545),
   m_pOwner(NULL),
   m_pTopNode(NULL)
{
  // init memory managers - (size of element, number of elements per allocation block, a mem pool pointer[if any] )
  //   no allocation of memory yet - just storing of block size parameters.
  m_sListMgr.Initialize(ALIGN_SIZE(sizeof(SmObjectList)),lNumObjectsPerBlock);
  m_sNodeMgr.Initialize(ALIGN_SIZE(sizeof(SmTreeNode)),  lNumNodesPerBlock);

  // allocate 1st block of nodes and use the 1st as the TopNode 
  m_pTopNode = (SmTreeNode*)m_sNodeMgr.GetNewElement()  ;
  SmTreeNode *pNode = m_pTopNode ; 

  // Set top Node - Empty Object node with no children but a bounding box = crSpatialBox
  pNode->m_eAuxDataType = SM_AD_OBJECT_LIST;
  pNode->m_pData        = NULL;
  pNode->m_pTree        = this;
  pNode->m_pChild1      = NULL;
  pNode->m_pChild2      = NULL;
  pNode->m_pParent      = NULL;
  pNode->m_sBBox        = crSpatialBox;

} // end SmTree::SmTree constructor

/*******************************************************************//**
PURPOSE: Return Object for which the treeNode->tree was built when possible
            else return NULL

NOTES: Currently only Trees built for curves and surfaces return
  pointers to the original SmCurve or SmSurface.  All other cases
  return pointers to NULL
***********************************************************************/
const SmObject *SmTreeNode::GetOwnerObject() const
{ return( m_pTree ? m_pTree->GetOwnerObject() : NULL) ; }

/*******************************************************************//**
PURPOSE: Return Object for which the tree was built when possible
            else return NULL

NOTES: Currently only Trees built for curves and surfaces return
  pointers to the original SmCurve or SmSurface.  All other cases
  return pointers to NULL
***********************************************************************/
const SmObject *SmTree::GetOwnerObject() const
{
  // when the tree has an Owner
  //   it will be either an SmCurveCache or a SmSurfaceCache
  if(m_pOwner)
    {
      if(m_pOwner->IsKindOf(SmCurveCache_TYPE))
        { 
          // return CurveCache's Curve
          return( ((SmCurveCache *)m_pOwner)->GetCurve()) ;
        }

      if(m_pOwner->IsKindOf(SmSurfaceCache_TYPE))
        { 
          // return SurfaceCache's Surface
          return( ((SmSurfaceCache *)m_pOwner)->GetSurface()) ;
        }
    } // else owner check

  // all done
  return(NULL) ;

} // end SmTree::GetOwnerObject

/*******************************************************************//**
PURPOSE: Add an element to the spatial tree.  Split tree nodes as 
    needed.

NOTES: 
  1. Avoid adding objects outside of the bounding box of the tree.
     When building a tree take the time to compute the bounding box in
     advance and send that extent to the tree's initial constructor.

     Relying on this function to find the bounding box is very EXPENSIVE.
     Each time an object with a bounding box outside the tree's bounding box
     is added, the old tree is deleted with all that memory being freed, 
     and a new tree, with a bounding box big enough to hold the both the
     old tree and the new object is constructed with all its new memory
     newly allocated.  Try not to use this function this way. 
***********************************************************************/
SmStatus SmTree::AddToSpatialTree
  (const SmExtent3d & crObjectExtent,  // in : bounding box of object to place in tree
   SmObject         * pObject,         // in : object to place in tree, NULL=just check extent size and return
   SmNodeGeomType     eType)           // in : oneof SM_NG_NONE,        // initialization default
                                       //            SM_NG_DEFAULT,     // used for all trees except the following
                                       //            SM_NG_LINE_SEG,    // label for curve and polyedge tree nodes   - helps SmRayTracer::FireRay function
                                       //            SM_NG_POLYGON      // label for surface and polyface tree nodes - helps SmRayTracer::FireRay function 
                                       //                               //     only used by SmBrepCache::BuildTrees()
{
  // locals 
  SmTreeNode *pTop = GetTopNode();

  // when object extent is outside tree's bounding box,
  //   Rebuild tree with a larger boundingBox.
  //    - very expensive, too be avoided.
  if (!crObjectExtent.IsContainedBy(pTop->m_sBBox, SM_EFF_ZERO)) 
    {
      // rebuild the tree with the appropriate size and continue.

#ifdef SM_DEBUG_CODE
      if(pObject != NULL)
        {
          // gwc:todo  Turn on the following warning to find places
          //           in SMLib where we can stop running into this
          //           unneeded and expensive condition.
          //
          // inform the debugging public that uncontrolled expansions are happening
          //SM_DBG_WARN(_T("AddToSpatialTree: forced tree rebuild to increase BBox. Avoid this. Construct tree with precomputed right-sized BBox")) ;
        }
#endif // SM_DEBUG_CODE

      // get all tree node ObjectList items
      SmTArray<SmObjectList*> sAllObjects;
      SER(GetObjectList(sAllObjects));

      // locals 
      SmExtent3d sUnion = crObjectExtent;
      SmTArray<SmObject*>  sObjects   (sAllObjects.GetSize());
      SmTArray<SmExtent3d> sObjExtents(sAllObjects.GetSize());

      // for every ObjectList
      for (ULONG i=0; i<sAllObjects.GetSize(); i++) 
        {
          SmObjectList *pObjList = sAllObjects[i];

          // build arrays of Objects and their bounding boxes 
          sObjects.Add(pObjList->m_pObject);
          sObjExtents.Add(pObjList->m_sBBox);

          // get union of all object bounding boxes
          sUnion.Union(pObjList->m_sBBox,sUnion);
        }

      // increase the size of the unioned bounding-box 
      SmVector3d sSize = sUnion.GetSize();
      double dTol = SM_EFF_ZERO_SQ * sSize.GetMaxDimension();
      sUnion.ExpandAbsolute(dTol);
      sUnion.AddPoint3d(sUnion.GetMax()+0.15*sSize);
      sUnion.AddPoint3d(sUnion.GetMin()-0.15*sSize);

      // empty the current tree
      m_sListMgr.ReSet(); // Get rid of all list nodes and start over.
      m_sNodeMgr.ReSet(); // Get rid of all tree nodes and start over.

      // init the tree with one top node
      m_pTopNode = (SmTreeNode*)m_sNodeMgr.GetNewElement();
      SmTreeNode *pNode = m_pTopNode;

      // init the topNode to be an empty Object list node
      pNode->m_eAuxDataType = SM_AD_OBJECT_LIST;
      pNode->m_pData     = NULL;
      pNode->m_pTree     = this;
      pNode->m_pChild1   = NULL;
      pNode->m_pChild2   = NULL;
      pNode->m_pParent   = NULL;
      pNode->m_sBBox     = sUnion;
      pNode->m_eGeomType = eType;

      // Rebuild the tree
      for (ULONG j=0; j<sObjects.GetSize(); j++) 
        {
          AddToSpatialTree(sObjExtents[j],sObjects[j],eType);
        }

    } // end disjoint new element bounding box check   

  // quit when asked to just check the extent
  if(pObject == NULL)
    { return SM_SUCCESS; }

  // find smallest leaf or internal node that completely contains the input box.
  SmTreeNode *pFoundNode = FindNode(crObjectExtent);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      smgfx_SetLook(1,2, 0,0,1); crObjectExtent.Draw(NULL); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); pFoundNode->m_sBBox.Draw(NULL); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // see if pFoundNode already contains pObject
  SmObjectList * pPrevOL = NULL ;
  SmBoolean bContainsObject =   pFoundNode->m_pData
                              ? ((SmObjectList*)pFoundNode->m_pData) ->FindObject(pObject, pPrevOL) 
                              : FALSE ;
  
  // low work - pFoundNode already contains pObject
  if(bContainsObject)
    {
      // Expand the current BBox to include the new box.  [B318]
      // Note, could check boxing to see whether it's necessary,
      // but it's quicker and easier just to do it: Union is easy.
      // Note, considered having a list of BBoxes for each Object.
      // That would result in the boxing being more efficient (in terms of
      // what gets boxed and what doesn't), but implementation would be
      // very messy: it's assumed everywhere that there is one BBox for
      // each object, and moreover, those data members are public and
      // used all over the place.

      // FindObject() returns the ObjectList previous to the one containing
      // the passed-in pObject (for convenience of list manipulation), or NULL
      // if it's the first in the list.  Here's how we get the proper one:
      SmObjectList *pThisOL = ( pPrevOL == NULL )
                             ? (SmObjectList*)(pFoundNode->m_pData)
                             : pPrevOL->m_pNext;
      SM_ASSERT( pThisOL != NULL );
      if ( pThisOL != NULL ) // (shouldn't be Null...)
        { pThisOL->m_sBBox.Union( crObjectExtent, pThisOL->m_sBBox ); }

      // all done
      return SM_SUCCESS ;
    }

  // allocate and init a new ListObject item for this object-
  SmObjectList *pNewListNode = (SmObjectList*)m_sListMgr.GetNewElement();
  pNewListNode->m_pObject    = pObject;
  pNewListNode->m_sBBox      = crObjectExtent;

  // insert new ObjectList item into front of FoundNode's linked-list of ObjectList items 
  pNewListNode->m_pNext      = (SmObjectList*)pFoundNode->m_pData;
  pFoundNode->m_pData        = pNewListNode;
  
  // get length of FoundNode's linked-list of ObjectList items
  ULONG lCount = 0;
  SmObjectList *pListNode = (SmObjectList*)pFoundNode->m_pData;
  while (pListNode) 
    {
      lCount++;
      pListNode = pListNode->m_pNext;
    }

  // decide when to split a list which may be too long
  //  don't split: 1. internal nodes - they're already split
  //               2. leaf nodes whose list of ObjectList items is short
  //               3. leaf nodes too small to split
  //  do    split: 1. leaf nodes with long ObjectList lists that are big enough
  SmBoolean bSplit = TRUE;
  if( pFoundNode->m_pChild1 || lCount < m_lMaxElementsPerNode) 
    {
      bSplit = FALSE;
    }
  else 
    {
      // get top and current node box sizes
      SmExtent3d sTopExt   = GetTopNode()->m_sBBox;
      SmExtent3d sCurrExt  = pFoundNode->m_sBBox;
      double     dTopDist  = sTopExt.GetMax().DistanceBetween(sTopExt.GetMin());
      double     dCurrDist = sCurrExt.GetMax().DistanceBetween(sCurrExt.GetMin());

      // Do not let low level nodes get smaller than 1/lMinSizeRatio of top node
      if (dTopDist > dCurrDist*m_dMinSizeRatio) 
        {
          bSplit = FALSE;
        }
      else 
        {
          bSplit = TRUE;
          // If make it here then we should split this node and put objects
          // into the leaf nodes.
        }
    } // end consider splitting a leaf node branch

  // when splitting a leaf node
  if (bSplit) 
    {
      // construct two new children
      SmExtent3d sCurrExt = pFoundNode->m_sBBox;
      SmTreeNode *pChild1 = (SmTreeNode*)m_sNodeMgr.GetNewElement();
      SmTreeNode *pChild2 = (SmTreeNode*)m_sNodeMgr.GetNewElement();

      // set child 1 pointers
      pFoundNode->m_pChild1   = pChild1;
      pChild1->m_pParent      = pFoundNode;
      pChild1->m_pTree        = pFoundNode->m_pTree;
      pChild1->m_pChild1      = NULL;
      pChild1->m_pChild2      = NULL;
      pChild1->m_eAuxDataType = SM_AD_OBJECT_LIST;
      pChild1->m_pTree        = pFoundNode->m_pTree;
      pChild1->m_pData        = NULL;
      pChild1->m_eGeomType    = eType;

      // set child 2 pointers
      pFoundNode->m_pChild2   = pChild2;
      pChild2->m_pParent      = pFoundNode;
      pChild2->m_pTree        = pFoundNode->m_pTree;
      pChild2->m_pChild1      = NULL;
      pChild2->m_pChild2      = NULL;
      pChild2->m_eAuxDataType = SM_AD_OBJECT_LIST;
      pChild2->m_pTree        = pFoundNode->m_pTree;
      pChild2->m_pData        = NULL;
      pChild2->m_eGeomType    = eType;

      // compute bounding boxes for child1 and child2
      //   split parent_box along its longest axis then set child boxes as
      //   child1_box = Parent_min to 
      //                  Parent_min 
      //                + (1.0-NodeReduction) * (Parent_max - Parent_min)
      //   child2_box = Parent_max to
      //                  Parent_max
      //                - (1.0-NodeReduction) * (Parent_max - Parent_min)
      SmVector3d sDiff = sCurrExt.GetMax() - sCurrExt.GetMin();
      SmPoint3d sNewMinPnt = sCurrExt.GetMin(); 
      SmPoint3d sNewMaxPnt = sCurrExt.GetMax();
      if (sDiff.x >= sDiff.y && sDiff.x >= sDiff.z) 
        {
          sNewMaxPnt.x = sNewMaxPnt.x - sDiff.x*m_dNodeReduction;
          sNewMinPnt.x = sNewMinPnt.x + sDiff.x*m_dNodeReduction;
        }
      else if (sDiff.y >= sDiff.x && sDiff.y >= sDiff.z) 
        {
          sNewMaxPnt.y = sNewMaxPnt.y - sDiff.y*m_dNodeReduction;
          sNewMinPnt.y = sNewMinPnt.y + sDiff.y*m_dNodeReduction;
        }
      else 
        {
          sNewMaxPnt.z = sNewMaxPnt.z - sDiff.z*m_dNodeReduction;
          sNewMinPnt.z = sNewMinPnt.z + sDiff.z*m_dNodeReduction;
        }
      pChild1->m_sBBox.SetMinMax(sCurrExt.GetMin(),sNewMaxPnt);
      pChild2->m_sBBox.SetMinMax(sNewMinPnt,sCurrExt.GetMax());

      // Arrive here when One node is expanded into three, a parent and its two children.
      // Place old parent's ObjectList items into the smallest node whose BBox 
      //    completely contains the object's BBox

      // get all of old parent node's ObjectList items
      SmObjectList *aData[200];
      SmTArray<SmObjectList*> sObjLists(200,aData);
      SER(pFoundNode->GetObjectList(sObjLists));

      // clear out old parent's ObjectList List
      pFoundNode->m_pData = NULL; 

      // for every ObjectList
      for (ULONG i=0; i<sObjLists.GetSize(); i++) 
        {
          SmObjectList *pLNode = sObjLists[i];
          pLNode->m_pNext = NULL;

          // get smallest node completely containing this object's BBox
          SmTreeNode *pFndNode = FindNode(pLNode->m_sBBox);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              sm_GraphicsLoop();
              smgfx_SetColor(0,0,1);
              pLNode->m_sBBox.Draw(pObject->GetContext());
              smgfx_SetColor(1,0,0);
              pFndNode->m_sBBox.Draw(pObject->GetContext());
              sm_GraphicsLoop();
            }
#endif
          // add this object to the found node's object list
          pLNode->m_pNext   = (SmObjectList*)pFndNode->m_pData;
          pFndNode->m_pData = pLNode;
        } // end iter every ObjectList item
    } // end need to Split a Node check

  // all done
  return SM_SUCCESS;

} // end SmTree::AddToSpatialTree

/*******************************************************************//**
PURPOSE: Remove an object from an object spatial tree.  

NOTES: 
  1. to avoid a general search of the tree - finds the node that
     contains the given ObjectExtent and then finds and removes
     the SmListObject from the linked list of SmListObjects stored
     under the pFoundNode->m_pData pointer.
  2. If the FoundNode does not contain the Object it also checks
     the FoundNode's parent generations until the Object is found.
     It does not look in the FoundNode's children.
***********************************************************************/
SmBoolean SmTree::RmFromSpatialTree    // rtn: TRUE when object was found and removed, else rtn FALSE
  (const SmExtent3d & crObjectExtent,  // in : bounding box of object to remove from tree
   SmObject         * pObject)         // in : object to remove from tree, NULL=just check extent size and return
{
  // check state - quit if this is not an object tree
  if( m_pTopNode == NULL || m_pTopNode->m_eAuxDataType != SM_AD_OBJECT_LIST)
    {
      return(FALSE) ;
    }

  // find smallest leaf or internal node that completely contains the input box.
  SmTreeNode *pFoundNode = FindNode(crObjectExtent) ;

  // when no node is found - nothing to do
  if(pFoundNode == NULL)
    {
      return(FALSE) ;
    }

  // head of pFoundNode linked SmObjectList
  SmObjectList *pPrevListNode = NULL ;
  SmObjectList *pThisListNode =  pFoundNode->m_eAuxDataType == SM_AD_OBJECT_LIST 
                               ? (SmObjectList *)pFoundNode->m_pData 
                               : NULL ;

  // when FoundNode's SmObjectList contains pObject
  if(pThisListNode && pThisListNode->FindObject(pObject, pPrevListNode))
    {
      // remove the found SmObjectList from the linked list
      if(pPrevListNode) { pPrevListNode->m_pNext = pPrevListNode->m_pNext->m_pNext ; }
      else              { pFoundNode->m_pData    = ((SmObjectList *)pFoundNode->m_pData)->m_pNext ; }

      // all done - don't delete the SmListObject because we're using the SmMemBlockMgr
      return(TRUE) ;
    }

  // arrive here when object was not found where it ought to be in the tree
  // it may be in a parent node left there by mistake when its bounding box changed size
  SmTreeNode *pParentNode = pFoundNode->m_pParent ;
  for(;pParentNode!=NULL;pParentNode = pParentNode->m_pParent)
    {
      pPrevListNode = NULL ;               
      pThisListNode =   pParentNode->m_eAuxDataType == SM_AD_OBJECT_LIST 
                      ? (SmObjectList *)pParentNode->m_pData 
                      : NULL ;

      // when FoundNode's SmObjectList contains pObject
      if(pThisListNode && pThisListNode->FindObject(pObject, pPrevListNode))
        {
          // remove the found SmObjectList from the linked list
          if(pPrevListNode) { pPrevListNode->m_pNext = pPrevListNode->m_pNext->m_pNext ; }
          else              { pParentNode->m_pData   = ((SmObjectList *)pParentNode->m_pData)->m_pNext ; }

          // all done - don't delete the SmListObject because we're using the SmMemBlockMgr
          return(TRUE) ;
        }
    } // end searching every parent of pFound node for object

  // arrive here when object was not found in FoundNode nor FoundNode's parents
  // we could look in the children nodes but we'll just quit
  return(FALSE) ;

} // end SmTree::RmFromSpatialTree

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmTree::GetAllTreeNodes( SmTArray<SmTreeNode*> & rNodes ) const
{
  rNodes.ReSet();
  SmTArray<void*> sNodes;
  m_sNodeMgr.GetActiveElements( sNodes );
  for(ULONG i = 0; i<sNodes.GetSize(); i++)
  {
    rNodes.Add( (SmTreeNode*)sNodes[i] );
  }
} // end SmTree::GetAllTreeNodes


/*******************************************************************//**
PURPOSE: Set all treeNode pointers for one connected loop

NOTES: 
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTree::SetVertexLooptreeNodes
//   (ULONG        indx,        // in : AboveLeft corner TreeVertex index of desired loop
//    SmTreeNode * pLeafNode)   // in : TreeNode being circumscribed by this TreeVertex Loop
//  { 
//    // Update the TreeNode ptrs
//    ULONG ii, lToDir = 0 ;
//    SmTArray<ULONG> sVertexLoop ;
//    GetTreeVertexLoop(indx, *pLeafNode, sVertexLoop) ;
//    ULONG lNum = sVertexLoop.GetSize() ;
//  
//    // walk the VertexLoop - compute node pointers to update based on the directions between vertices.
//    //  When moving straight through a vertex - update two Node ptrs. (nodes to the left of the enter and exit directions)
//    //  when making a CCW turn at a vertex - update one Node ptrs.    (node to the left of the exit direction)
//    // Increment lToDir at each turn to decide which Vertex Node ptrs to update
//    for(ii=0;ii<lNum;ii++)
//      {
//        // case CCW Turn at Vertex: when Starting or CCW Connect Index == Next Loop Index
//        SmBoolean bTurn =    (ii == 0) 
//                          || (m_sTreeVertices[sVertexLoop[ii]].GetConnect((lToDir+1)%4) == sVertexLoop[(ii+1)%lNum]) ;
//  
//        // Assert other legal case: Straight through Vertex, straight Connect Index == Next Loop Index
//        SM_ASSERT(bTurn == TRUE || m_sTreeVertices[sVertexLoop[ii]].GetConnect(lToDir) == sVertexLoop[(ii+1)%lNum]) ;
//        
//        // when turning - increment lToDir
//        if(ii != 0 && bTurn) 
//          { lToDir ++ ; }
//  
//        // When turning or going straight set Node to the left of the exit direction (see picture)
//        m_sTreeVertices[sVertexLoop[ii]].SetTreeNode(lToDir, pLeafNode) ;
//  
//        // When going straight also set Node to the left of the enter direction (see picture)
//        if(!bTurn) m_sTreeVertices[sVertexLoop[ii]].SetTreeNode((lToDir+1)%4, pLeafNode) ;
//  
//        // VertexLoop [0 1 2 3 4 5 6 7];  
//        // 
//        //              Vertex Indices      TreeNode Ptr Indices
//        //              0     7     6       to set to pLeafNode
//        //               +-----+-----+        +-----+-----+
//        //               |           |        |0   3 0   3|
//        // Traversal  |  |           |        |           |
//        // Direction  | 1|          5|        |1         2|
//        //            |  +           +        +           +
//        //            V  |           |        |0         3|
//        //               |           | ^      |           |
//        //            | 2|    3     4| |      |1   2 1   2|
//        //            |  +-----+-----+ |      +-----+-----+
//        //            +--->        ----+
//  
//        // Loops are commonly made by subdivision - When subdividing a node
//        // the new vertices need TreeNode Pointers both inside and outside
//        // the new VertexLoops. Help SubdivideNode out by setting outside
//        // TreeNode values when appropriate.  This works here if all nodes
//        // are made through subdivision.
//  
//        // propogating right side TreeNode values when appropriate
//        ULONG lThisIndx = sVertexLoop[ii] ;
//        ULONG lNextIndx = m_sTreeVertices[lThisIndx].GetConnect(lToDir) ;
//        SmTreeNode *pThisRightSide = m_sTreeVertices[lThisIndx].GetTreeNode((lToDir+3)%4) ;
//        SmTreeNode *pNextRightSide = m_sTreeVertices[lNextIndx].GetTreeNode((lToDir+2)%4) ;
//  
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//  TCHAR sBuff[SM_TBLOCK_SIZE] ;
//        if(bDebugMe)
//          {
//            DumpTreeVertices() ;
//            smos_sprintf(sBuff, _T("ThisIndx = %ld,  NextIndx = %ld, pThisRightSide = 0x%p, pNextRightSide = 0x%p\n"),
//                              lThisIndx, lNextIndx, pThisRightSide, pNextRightSide) ;
//            smos_WriteBuffer(sBuff);
//          }
//  #endif // SM_DEBUG_CODE
//  
//        // propogate Right side NonNULL values to partner
//        if(pThisRightSide == NULL && pNextRightSide != NULL) { m_sTreeVertices[lThisIndx].SetTreeNode((lToDir+3)%4, pNextRightSide) ; 
//                                                               pThisRightSide = pNextRightSide ;
//                                                             }
//        if(pThisRightSide != NULL && pNextRightSide == NULL) { m_sTreeVertices[lNextIndx].SetTreeNode((lToDir+2)%4, pThisRightSide) ; 
//                                                               pNextRightSide = pThisRightSide ; 
//                                                             }
//  
//        // when RightSide TreeNode value is NonNULL
//        if(pNextRightSide != NULL)
//          {
//  
//            // propogate Right side NonNULL values across right side straight connections
//            if(m_sTreeVertices[lThisIndx].GetConnect((lToDir+3)%4) == SM_BIG_ULONG) // turn connect not used
//              {
//                 m_sTreeVertices[lThisIndx].SetTreeNode((lToDir+2)%4, pNextRightSide) ; // set Prev quadrant TreeNode on ThisNode
//              }
//            if(m_sTreeVertices[lNextIndx].GetConnect((lToDir+3)%4) == SM_BIG_ULONG) // turn connect not used
//              {
//                 m_sTreeVertices[lNextIndx].SetTreeNode((lToDir+3)%4, pNextRightSide) ; // set Next quadrant TreeNode on NextNode
//              }
//          } // end pNextRightSide existence check
//  
//        // When going straight on left side also set Node to the left of the enter direction (see picture)
//        if(!bTurn) 
//          {
//            ULONG lPrevIndx = m_sTreeVertices[lThisIndx].GetConnect((lToDir+2)%4) ;
//            SmTreeNode *pThisRightSide = m_sTreeVertices[lThisIndx].GetTreeNode((lToDir+2)%4) ;
//            SmTreeNode *pPrevRightSide = m_sTreeVertices[lPrevIndx].GetTreeNode((lToDir+3)%4) ;
//  
//            // propogate Right side NonNULL values to partner
//            if(pThisRightSide == NULL && pPrevRightSide != NULL) { m_sTreeVertices[lThisIndx].SetTreeNode((lToDir+2)%4, pPrevRightSide) ; }
//            if(pThisRightSide != NULL && pPrevRightSide == NULL) { m_sTreeVertices[lPrevIndx].SetTreeNode((lToDir+3)%4, pThisRightSide) ; }
//          }
//  
//  #ifdef SM_DEBUG_CODE
//        if(bDebugMe)
//          {
//            DumpTreeVertices() ;
//            smos_sprintf(sBuff, _T("ThisIndx = %ld,  NextIndx = %ld, pThisRightSide = 0x%p, pNextRightSide = 0x%p\n"),
//                              lThisIndx, lNextIndx, pThisRightSide, pNextRightSide) ;
//            smos_WriteBuffer(sBuff);
//          }
//  #endif // SM_DEBUG_CODE
//  
//      } // end iter every Vertex bounding this node
//  } // end SmTree::SetVertexLooptreeNodes 

/*******************************************************************//**
PURPOSE: Increment m_sTreeVertices list length initializing all
         new member connection values to NotUsed (SM_BIG_ULONG)

NOTES:
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTree::IncTreeVertexCount(ULONG lInc) 
//  { 
//    // locals
//    ULONG ii ;
//    ULONG lOldSize = m_sTreeVertices.GetSize() ;
//    ULONG lNewSize = lOldSize + lInc ;
//  
//    // Set the m_sTreeVertices array size
//    m_sTreeVertices.SetSize(lNewSize) ;
//    
//    // init all new member connections and TreeNodes to NotUsed (SM_BIG_ULONG, NULL) 
//    for(ii=lOldSize;ii<lNewSize;ii++) 
//      { 
//        m_sTreeVertices[ii].InitConnects() ; 
//      }
//  
//  } // end SmTree::IncTreeVertexCount

/*******************************************************************//**
PURPOSE: Set m_sTreeVertices list length initializing any
         new member connection values to NotUsed (SM_BIG_ULONG)

NOTES:
***********************************************************************/
// GWCTreeVertexTemp
// void SmTree::SetTreeVertexCount(ULONG lCnt) 
// { 
//   // when incrementing m_sTreeVertices length
//   if(GetTreeVertexCount() < lCnt)
//     { 
//       ULONG lInc = lCnt - GetTreeVertexCount() ;
// 
//       // pass the increment call along to init all connection values to NotUsed (SM_BIG_ULONG)
//       IncTreeVertexCount(lInc) ;
//     }
//   else // shorten the List length (not expected to be called)
//     { 
//       m_sTreeVertices.SetSize(lCnt) ; 
//     }
// 
// } // end SmTree::SetTreeVertexCount

/*******************************************************************//**
PURPOSE: Make Sure TreeVertex entry is allocated and set its UVPoint

NOTES:
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTree::SetTreeVertex
//    (ULONG  indx,        // eff: init TreeVertex[indx] UVPos - grow TreeVertex array size if needed.
//     double dUVPointU,      
//     double dUVPointV)   
//  { 
//    // allocate TreeVertices when needed - init all Connects to NotUsed (SM_BIG_ULONG)
//    if(indx >= GetTreeVertexCount()) 
//      { SetTreeVertexCount(indx+1) ; }
//    
//    // Set Tgt TreeVertex UVPoint
//    m_sTreeVertices[indx].SetUVPoint(dUVPointU, dUVPointV) ;
//       
//  } // end SmTree::SetTreeVertex

/*******************************************************************//**
PURPOSE: Return Next TreeVertex Index while traversing a node Loop 
         in CCW direction given current TreeVertex Index and a traversal direction.

NOTES: From the m_sTreeVertex[indx] TreeVertex point of view
          FromDir = vertex branch used to arrive at this vertex on a CCW traversal
          ToDir   = vertex branch used to leave this vertex on a CCW traversal
       
      The Next TreeVertex depends on the state at this TreeVertex:
        If StraightThrough this Vertex connection is being used
          {
           If (The leaf nodes bounded by this Vertex (to the left)
               on both the from and the to branches are both descended
               from the pOptTreeNode parent being traced,
             { ToDir = straight through direction }

           else if CW Turn connection is being used
             { ToDir = CW Turn direction }

           else (an error state) let,
             { ToDir = Straight Through Direction }
          }
        else
          { ToDir = CW Turn direction }

      When data structure is corrupt or uninitialized and no next direction can be computed,
      SM_BIG_ULONG is returned.
***********************************************************************/
// GWCTreeVertexTemp
//  ULONG SmTree::GetNextTreeVertex
//   (ULONG              indx,          // in : Index of Vertex to query
//    ULONG              lFromDir,      // in : 0=FromBelow, 1=FromRight, 2=FromAbove, 3=FromLeft 
//    ULONG            & lToDir,        // out: 0=ToBelow,   1=ToRight,   2=ToAbove,   3=ToLeft
//    const SmTreeNode * pOptTreeNode)  // in : Optional node to trace - use for parent nodes, NULL=trace leaf node
//   const                          
//  { 
//    // check state - valid index
//    SM_ASSERT(indx < m_sTreeVertices.GetSize()) ;
//  
//    // locals - FromNode = LeafNode bounded at Vertex[indx] to the left of the FromDir
//    SmTreeNode *pFromNode       = m_sTreeVertices[indx].GetTreeNode((lFromDir+3)%4) ;
//  
//    // if pOptTreeNode is NULL - set it to the current leaf node value
//    if(pOptTreeNode == NULL) 
//      { pOptTreeNode = pFromNode ; }
//  
//    // check state - FromNode is descendant of pOptTreeNode
//    SmBoolean   bFromDescendant = pFromNode ? pFromNode->IsDescendant(pOptTreeNode) : FALSE ;
//  
//    // When straight ahead is being used
//    if(m_sTreeVertices[indx].GetConnect((lFromDir+2)%4) != SM_BIG_ULONG)
//      {
//        // ToNode = LeafNode bounded at Vertex[indx] to the left of the straigth through ToDir
//        SmTreeNode *pToNode       = m_sTreeVertices[indx].GetTreeNode((lFromDir+2)%4) ;
//        SmBoolean   bToDescendant = pToNode ? pToNode->IsDescendant(pOptTreeNode) : FALSE ;
//  
//        // when both from and to nodes are descended from pOptTreeNode 
//        //      ToDir = straight through this TreeVertex,
//        // else if CW Turn is being used 
//        //      ToDir = CW turn at this TreeVertex
//        // else ToDir = straight through this Vertex (this is an error condition) 
//        lToDir =   (   (bFromDescendant && bToDescendant) 
//                    || m_sTreeVertices[indx].GetConnect((lFromDir+3)%4) == SM_BIG_ULONG)
//                 ? (lFromDir+2)%4    // Straight ahead
//                 : (lFromDir+3)%4 ;  // CW Turn
//      }
//    else // straight ahead not used - Set ToDir to a CW turn at TreeVertex
//      {
//        lToDir = (lFromDir+3)%4 ; // CW Turn
//      }
//  
//  
//    // return NextTreeVertex when available - else return err = SM_BIG_ULONG
//    return( lToDir == SM_BIG_ULONG ? SM_BIG_ULONG
//                                   : m_sTreeVertices[indx].GetConnect(lToDir)) ;
//  
//  } // end SmTree::GetNextTreeVertex

/*******************************************************************//**
PURPOSE: Create a doubly linked TreeVertex to TreeVertex connection

NOTES: 0.) Assumes m_sTreeVertices[indx].m_vUVPoint has been set
                   m_sTreeVertices[lNextVertexIndx].m_vUVPoint has been set

       1.) This method always sets the connections 
           from indx to lNextVertexIndx and
           from lNextVertexIndx to indx to create the relationship
               indx <-> lNextVertexIndx

       2.) This method is to be used to insert connected nodes into
           existing connected TreeVertex graphs.  So if lNextVertexIndx
           is already connected to another TreeVertex in the given direction
           additional connects are set to change 
               from PrevIndx <-> lNextVertexIndx 
               to   PrevIndx <-> indx <-> lNextVertexIndx

               from indx <-> NextIndx
               to   indx <-> lNextVertexIndx <-> NextIndx
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTree::ConnectTreeVertex
//   (ULONG indx,                      // in : Index of TreeVertex to connect
//    ULONG lDir,                      // in : ToDir oneof:0=Below,1=Right,2=Above,3=Left
//    ULONG lNextVertexIndx)           // in : TreeVertex indx in the given direction
//  { 
//    SM_ASSERT(indx            < m_sTreeVertices.GetSize()) ;
//    SM_ASSERT(lNextVertexIndx < m_sTreeVertices.GetSize()) ;
//    SM_ASSERT(lDir            < 4) ;
//  
//    // remember the lNextVertexIndx PrevIndx
//    ULONG lNext_PrevIndx = m_sTreeVertices[lNextVertexIndx].GetConnect((lDir+2)%4) ;
//    ULONG lIndx_NextIndx = m_sTreeVertices[indx].GetConnect(lDir) ;
//  
//    // no work - already connected
//    if(   lNext_PrevIndx  == indx
//       && lIndx_NextIndx  == lNextVertexIndx)
//      { return ; }
//  
//    // check state - indx and lNextVertexIndx are ordered
//    SM_ASSERT_MSG(   (lDir == 0 && m_sTreeVertices[indx].GetUVPoint().y > m_sTreeVertices[lNextVertexIndx].GetUVPoint().y)
//                  || (lDir == 1 && m_sTreeVertices[indx].GetUVPoint().x < m_sTreeVertices[lNextVertexIndx].GetUVPoint().x)
//                  || (lDir == 2 && m_sTreeVertices[indx].GetUVPoint().y < m_sTreeVertices[lNextVertexIndx].GetUVPoint().y)
//                  || (lDir == 3 && m_sTreeVertices[indx].GetUVPoint().x > m_sTreeVertices[lNextVertexIndx].GetUVPoint().x),
//                  _T("SmTree::ConnectTreeVertex - m_sTreeVertices[indx] not properly ordered with m_sTreeVertices[lNextVertexIndx]")) ;
//  
//    // check state - expect at least one of the inputs to be unconnected
//    SM_ASSERT(   lNext_PrevIndx == SM_BIG_ULONG    // gwc: I believe that it will be an error when
//              || lIndx_NextIndx == SM_BIG_ULONG) ; //      both vertices are already part of the topology graph.
//                                                    //      If this happens this function needs to be extended
//                                                    //      to handle that case.
//  
//    // set indx <-> lNextVertexIndx connections 
//    m_sTreeVertices[indx].SetConnect(lDir, lNextVertexIndx) ;
//    m_sTreeVertices[lNextVertexIndx].SetConnect((lDir + 2)%4, indx) ;
//  
//    // when needed - set PrevIndx <-> indx connections
//    if(lNext_PrevIndx != SM_BIG_ULONG)
//      { 
//        // check state - indx and lNextVertexIndx are ordered
//        SM_ASSERT_MSG(   (lDir == 0 && m_sTreeVertices[lNext_PrevIndx].GetUVPoint().y > m_sTreeVertices[indx].GetUVPoint().y)
//                      || (lDir == 1 && m_sTreeVertices[lNext_PrevIndx].GetUVPoint().x < m_sTreeVertices[indx].GetUVPoint().x)
//                      || (lDir == 2 && m_sTreeVertices[lNext_PrevIndx].GetUVPoint().y < m_sTreeVertices[indx].GetUVPoint().y)
//                      || (lDir == 3 && m_sTreeVertices[lNext_PrevIndx].GetUVPoint().x > m_sTreeVertices[indx].GetUVPoint().x),
//                      _T("SmTree::ConnectTreeVertex - m_sTreeVertices[lNext_PrevIndx] not properly ordered with m_sTreeVertices[indx]")) ;
//  
//        m_sTreeVertices[lNext_PrevIndx].SetConnect(lDir, indx) ;
//        m_sTreeVertices[indx].SetConnect((lDir + 2)%4, lNext_PrevIndx) ;
//      }
//  
//    // when needed - set lNextVertexIndx <-> NextIndx connections
//    if(lIndx_NextIndx != SM_BIG_ULONG)
//      { 
//        // check state - indx and lNextVertexIndx are ordered
//        SM_ASSERT_MSG(   (lDir == 0 && m_sTreeVertices[lNextVertexIndx].GetUVPoint().y > m_sTreeVertices[lIndx_NextIndx].GetUVPoint().y)
//                      || (lDir == 1 && m_sTreeVertices[lNextVertexIndx].GetUVPoint().x < m_sTreeVertices[lIndx_NextIndx].GetUVPoint().x)
//                      || (lDir == 2 && m_sTreeVertices[lNextVertexIndx].GetUVPoint().y < m_sTreeVertices[lIndx_NextIndx].GetUVPoint().y)
//                      || (lDir == 3 && m_sTreeVertices[lNextVertexIndx].GetUVPoint().x > m_sTreeVertices[lIndx_NextIndx].GetUVPoint().x),
//                      _T("SmTree::ConnectTreeVertex - m_sTreeVertices[lNextVertexIndx] not properly ordered with m_sTreeVertices[lIndx_NextIndx]")) ;
//  
//        m_sTreeVertices[lNextVertexIndx].SetConnect(lDir, lIndx_NextIndx) ;
//        m_sTreeVertices[lIndx_NextIndx].SetConnect((lDir + 2)%4, lNextVertexIndx) ;
//      }
//  
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe = FALSE ;
//    if(bDebugMe)
//      {
//        DumpTreeVertices() ;
//      }
//  #endif // SM_DEBUG_CODE
//  
//   } // end SmTree::ConnectTreeVertex

/*******************************************************************//**
PURPOSE: Get TreeVertexLoop index sequence in CCW direction starting with 
         Above Left TreeNode corner circumscribing rTreeNode

NOTES: This method gets called while the TreeVertex Model and the TreeNode 
       models are being built.  Tell the method where to start in one of 
       two different ways.

       if(rTreeNode and its parents all have set m_pData->m_lStartTreeVertexIndx
       set indx == SM_BIG_ULONG.

       If the treeNodes are not yet complete pass in the valid indx number
       for rTreeNode's Above Left corner vertex.
***********************************************************************/
// GWCTreeVertexTemp
// void SmTree::GetTreeVertexLoop
//  (ULONG              indx,         // in : Above Left TreeNode TreeVertex index, SM_BIG_ULONG = use rTreeNode->GetStartTreeVertexIndx() 
//   const SmTreeNode & rTreeNode,    // in : node to trace - leaf or parent node
//   SmTArray<ULONG>  & rVertexLoop)  // out: TreeVertex CCW index loop starting with Above Left corner indx
//  const
// { 
//   // init output
//   rVertexLoop.ReSet() ;
// 
//   // Get Above left most TreeVertex index into SmTree::m_sTreeVertices 
//   //   Surface Subdivision Nodes 
//   //         Leaf node indx     = ((SmBezierAux2d *)m_pData)->m_lStartTreeVertexIndx
//   //         interior node indx = inherited from UpperLeft most descendant
//   //   Other Nodes = SM_BIG_ULONG
//   if(indx == SM_BIG_ULONG)
//     { 
//       indx = rTreeNode.GetStartTreeVertexIndx() ;
//     }
// 
//   // no work - no NodeLoop to trace
//   if(indx == SM_BIG_ULONG)
//     { return ; }
// 
//   // check state - bad index
//   SM_ASSERT(indx < m_sTreeVertices.GetSize()) ;
// 
//   // init NodeLoop with AboveLeft Corner index
//   rVertexLoop.Add(indx) ;
// 
//   // iter local inits
//   ULONG lToDir, lFromDir = 1 ; // From Right
//   ULONG lNextTreeVertex = GetNextTreeVertex(indx, lFromDir, lToDir, &rTreeNode) ;
//   SM_ASSERT(lToDir == 0) ; // ToBelow
// 
//   // while loop TreeVerts remain
//   while(indx != lNextTreeVertex)
//     {
//       SM_ASSERT(lNextTreeVertex < m_sTreeVertices.GetSize()) ;
// 
//       // accumulate Loop output
//       rVertexLoop.Add(lNextTreeVertex) ;
// 
//       // convert a ToDir to a FromDir for next fetch
//       lFromDir = ToDir2FromDir(lToDir) ;
// 
//       // fetch next TreeVertex index in loop
//       lNextTreeVertex=GetNextTreeVertex(lNextTreeVertex, lFromDir, lToDir, &rTreeNode) ;
//     } // end Whil gathering Loop TreeVertices
// 
// } // end SmTree::GetTreeVertexLoop
// 
/*******************************************************************//**
PURPOSE: Insert a 4 corner TreeVertex loop into the growing
         TreeVertex topology graph model.

NOTES: 1.) Only the corner TreeVertices and their immediate Pre and Post
           neighbors (when traveling in a CCW direction) are needed
           to properly update the SmTree::m_sTreeVertices connectivity model.
           Assuming every split is recorded as they are made in sequence.

       2.) When the New Node only has the 4 corner TreeVertices then
           all the pre/post neighbors are the other corners being passed
           in within this call.  However, when the new Node loop includes
           existing side nodes then the neighbor indices will be a
           mix of existing TreeVertex indices and new corner TreeVertex
           indices as needed.
***********************************************************************/
// GWCTreeVertexTemp
// ULONG SmTree::SetTreeVertexLoop       // rtn: TreeVertex Index of AboveLeft NodeLoop Start
// (SmTreeNode       * pTreeNode,        // in : TreeNode owner of this TreeVertex Loop
//  SmExtent2d       & rUVDomain,        // in : UVDomain for Node with this loop
//  ULONG              iiIndx,           // in : Above Left Corner i of index[i,j] of new Node TreeVertex Loop
//  ULONG              jjIndx,           // in : Above Left Corner j of index[i,j] of new Node TreeVertex Loop 
//  SmTArray<double> & rUSplits,         // in : rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//  SmTArray<double> & rVSplits,         // in : rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//  SmTArray<ULONG>  & rTreeVertexMap,   // in : Array made by BuildTreeVertexMaps call        
//  SmTArray<ULONG>  & rTreeVertexPre,   // in : Array Made By BuildTreeVertexMaps call
//  SmTArray<ULONG>  & rTreeVertexPost)  // in : Array Made By BuildTreeVertexMaps call
// { 
//   SM_ASSERT(   m_pTopNode == NULL 
//             || (   (   m_pTopNode->m_eAuxDataType == SM_AD_AUX_DATA
//                     || m_pTopNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE     
//                     || m_pTopNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD)
//                 && (   rUVDomain.IsContainedBy( ((SmBezierAux2d*)m_pTopNode->m_pData)->GetUVDomain(), SM_EFF_ZERO)))) ;
// 
//   // locals
//   ULONG           lNumSplitsU  = rUSplits.GetSize() ;       // in : Total Number of U Splits in TreeVertex Array(ii,jj)
//   ULONG           lNumSplitsV  = rVSplits.GetSize() ;       // in : Total Number of V Splits in TreeVertex Array(ii,jj)
//   SmBezierAux2d * pBezierAux2d = pTreeNode->GetBezierAux2d() ;
// 
//   // corner TreeVertex indices 
//   ULONG lALIndx = GetTreeVertexIndex(iiIndx  ,jjIndx  ,lNumSplitsU,lNumSplitsV,rTreeVertexMap) ;
//   ULONG lBLIndx = GetTreeVertexIndex(iiIndx  ,jjIndx-1,lNumSplitsU,lNumSplitsV,rTreeVertexMap) ;
//   ULONG lBRIndx = GetTreeVertexIndex(iiIndx+1,jjIndx-1,lNumSplitsU,lNumSplitsV,rTreeVertexMap) ;
//   ULONG lARIndx = GetTreeVertexIndex(iiIndx+1,jjIndx  ,lNumSplitsU,lNumSplitsV,rTreeVertexMap) ;
// 
//   // Make sure the TreeVertex list holds a TreeVertex with set UVPoint for the given indices
//   SetTreeVertex(lALIndx, rUVDomain.GetUMin(), rUVDomain.GetVMax()) ; 
//   SetTreeVertex(lBLIndx, rUVDomain.GetUMin(), rUVDomain.GetVMin()) ; 
//   SetTreeVertex(lBRIndx, rUVDomain.GetUMax(), rUVDomain.GetVMin()) ; 
//   SetTreeVertex(lARIndx, rUVDomain.GetUMax(), rUVDomain.GetVMax()) ;
//   
//   // get the neighbor TreeVertices from the various TreeVertexMaps - set connections between the 4 corner loops and their immediate neighbors
//   ULONG lALIndx_ToRight = GetPreTreeVertexIndex (iiIndx  ,jjIndx  , 0, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ;  // in :   RightNeighbor TreeVertexIndex of AboveLeft Corner
//   ULONG lALIndx_ToBelow = GetPostTreeVertexIndex(iiIndx  ,jjIndx  , 0, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ; // in :   BelowNeighbor TreeVertexIndex of AboveLeft Corner
//                                                                                                                  
//   ULONG lBLIndx_ToAbove = GetPreTreeVertexIndex (iiIndx  ,jjIndx-1, 1, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ;  // in :   AboveNeighbor TreeVertexIndex of BelowLeft Corner
//   ULONG lBLIndx_ToRight = GetPostTreeVertexIndex(iiIndx  ,jjIndx-1, 1, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ; // in :   RightNeighbor TreeVertexIndex of BelowLeft Corner
//                                                                                                                  
//   ULONG lBRIndx_ToLeft  = GetPreTreeVertexIndex (iiIndx+1,jjIndx-1, 2, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ;  // in :   LeftNeighbor  TreeVertexIndex of BelowRight Corner
//   ULONG lBRIndx_ToAbove = GetPostTreeVertexIndex(iiIndx+1,jjIndx-1, 2, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ; // in :   AboveNeighbor TreeVertexIndex of BelowRight Corner
//                                                                                                                  
//   ULONG lARIndx_ToBelow = GetPreTreeVertexIndex (iiIndx+1,jjIndx  , 3, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ;  // in :    BelowNeighbor  TreeVertexIndex of AboveRight Corner
//   ULONG lARIndx_ToLeft  = GetPostTreeVertexIndex(iiIndx+1,jjIndx  , 3, lNumSplitsU, lNumSplitsV, rTreeVertexMap, rTreeVertexPre, rTreeVertexPost) ; // in :    LeftNeighbor TreeVertexIndex of AboveRight Corner
// 
//   // Set connections between the 4 corner loops and their immediate neighbors
//   ConnectTreeVertex(lALIndx, 1, lALIndx_ToRight) ; if(lALIndx_ToRight != lARIndx) { ConnectTreeVertex(lARIndx, 3, lARIndx_ToLeft ) ; }
//   ConnectTreeVertex(lBLIndx, 2, lBLIndx_ToAbove) ; if(lBLIndx_ToAbove != lALIndx) { ConnectTreeVertex(lALIndx, 0, lALIndx_ToBelow) ; }
//   ConnectTreeVertex(lBRIndx, 3, lBRIndx_ToLeft ) ; if(lBRIndx_ToLeft  != lBLIndx) { ConnectTreeVertex(lBLIndx, 1, lBLIndx_ToRight) ; }
//   ConnectTreeVertex(lARIndx, 0, lARIndx_ToBelow) ; if(lARIndx_ToBelow != lBRIndx) { ConnectTreeVertex(lBRIndx, 2, lBRIndx_ToAbove) ; }
// 
//   // Set all treeNode pointers for one connected loop
//   SetVertexLooptreeNodes(lALIndx,      // in : AboveLeft corner TreeVertex index of desired loop
//                          pTreeNode) ;  // in : TreeNode owner of circumscribed by this TreeVertex Loop
//    // all done
//   return( lALIndx ) ; 
// 
// } // end SmTree::SetTreeVertexLoop

/*******************************************************************//**
PURPOSE: Insert a 4 corner TreeVertex loop into the growing
         TreeVertex topology graph model.

NOTES: Used to start a root node where all 4 TreeVertices are New TreeVertices
***********************************************************************/
// GWCTreeVertexTemp
// ULONG SmTree::SetRootTreeVertexLoop  // rtn: TreeVertex Index of AboveLeft NodeLoop Start
// (SmExtent2d       & rUVDomain,       // in : UVDomain for Node with this loop
//  ULONG              lALIndx,         // in : Above Left  corner of new Loop:[AL BL BR AR]
//  ULONG              lBLIndx,         // in : Below Left  corner of new Loop:[AL BL BR AR]
//  ULONG              lBRIndx,         // in : Below Right corner of new Loop:[AL BL BR AR]
//  ULONG              lARIndx,         // in : Above Right corner of new Loop:[AL BL BR AR]
//  SmTreeNode       * pTopNode)        // in : pNode being bounded by this TreeVertex loop
// { 
//   SM_ASSERT(   m_pTopNode == NULL 
//             || (   (   m_pTopNode->m_eAuxDataType == SM_AD_AUX_DATA
//                     || m_pTopNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE     
//                     || m_pTopNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD)
//                 && (   rUVDomain.IsContainedBy( ((SmBezierAux2d*)m_pTopNode->m_pData)->GetUVDomain(), SM_EFF_ZERO)))) ;
// 
//   // size m_sTreeVertices
//   ULONG lMaxIndx = smos_4Max(lALIndx, lBLIndx, lBRIndx, lARIndx) ;
//   if(lMaxIndx >= m_sTreeVertices.GetSize())
//     {
//       m_sTreeVertices.SetSize(lMaxIndx+1) ;
//       for(ULONG ii=0;ii<=lMaxIndx;ii++)
//         {
//           m_sTreeVertices[ii].InitConnects() ;
//         }
//     }
//       
//   // Make sure the TreeVertex list holds a TreeVertex with set UVPoint for the given indices
//   SetTreeVertex(lALIndx, rUVDomain.GetUMin(), rUVDomain.GetVMax()) ; 
//   SetTreeVertex(lBLIndx, rUVDomain.GetUMin(), rUVDomain.GetVMin()) ; 
//   SetTreeVertex(lBRIndx, rUVDomain.GetUMax(), rUVDomain.GetVMin()) ; 
//   SetTreeVertex(lARIndx, rUVDomain.GetUMax(), rUVDomain.GetVMax()) ;
// 
//   // Set connections between the 4 corner loops and their immediate neighbors
//   ConnectTreeVertex(lALIndx, 1, lARIndx) ; // ConnectTreeVertex(lALIndx, 0, lBLIndx) ; 
//   ConnectTreeVertex(lBLIndx, 2, lALIndx) ; // ConnectTreeVertex(lBLIndx, 1, lBRIndx) ; 
//   ConnectTreeVertex(lBRIndx, 3, lBLIndx) ; // ConnectTreeVertex(lBRIndx, 2, lARIndx) ; 
//   ConnectTreeVertex(lARIndx, 0, lBRIndx) ; // ConnectTreeVertex(lARIndx, 3, lALIndx) ;
//   
//   // Set TreeNode pack pointers
//   SetTreeVertexNodes(lALIndx, pTopNode, NULL,     NULL,     NULL) ; 
//   SetTreeVertexNodes(lBLIndx, NULL,     pTopNode, NULL,     NULL) ; 
//   SetTreeVertexNodes(lBRIndx, NULL,     NULL,     pTopNode, NULL) ; 
//   SetTreeVertexNodes(lARIndx, NULL,     NULL,     NULL,     pTopNode) ; 
// 
//   // all done
//   return( lALIndx ) ; 
// 
// } // end SmTree::SetRootTreeVertexLoop

/*******************************************************************//**
PURPOSE: Fetch the previous TreeVertex Index in a TreeVertexLoop for
  a specified TreeVertex Index given the TreeVertex Maps 
  created by BuildTreeVertexMaps() that map all the TreeVertices 
  to be used to create a set of new subdivision nodes from a parent node.

NOTES: 
***********************************************************************/
// GWCTreeVertexTemp
//  ULONG SmTree::GetPreTreeVertexIndex
//   (ULONG iiIndx,                           // in : ii of TreeVertex Array(ii,jj)
//    ULONG jjIndx,                           // in : jj of TreeVertex Array(ii,jj)
//    ULONG lCornerIndx,                      // in : oneof 0=ALCorner,1=BLCorner,2=BRCorner,3=ARCorner
//    ULONG lNumSplitsU,                      // in : Total Number of U Splits in TreeVertex Array(ii,jj)
//    ULONG lNumSplitsV,                      // in : Total Number of V Splits in TreeVertex Array(ii,jj)
//    SmTArray<ULONG> & rTreeVertexMap,       // in : Array made by BuildTreeVertexMaps call        
//    SmTArray<ULONG> & rTreeVertexPre,       // in : Array Made By BuildTreeVertexMaps call
//    SmTArray<ULONG> & rTreeVertexPost)      // in : Array Made By BuildTreeVertexMaps call
//   const
//  {
//    // return value
//    ULONG lPreIndex = SM_BIG_ULONG ;
//  
//    // switch on the subdivision node corner being queired
//    switch(lCornerIndx)
//      {
//        case 0 : // Above Left Corner - fetch Previous (Right=Post) TreeVertex Index in TreeVertexLoop for this Node
//                 lPreIndex =   (   iiIndx == lNumSplitsU-1 
//                                || jjIndx == 0)             ? SM_BIG_ULONG
//                             : (   jjIndx == lNumSplitsV - 1      
//                                && SM_BIG_ULONG != rTreeVertexPost[BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)])  
//                                                            ? rTreeVertexPost[BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)]
//                             :                                rTreeVertexMap[ArrayIJToK(iiIndx+1, jjIndx, lNumSplitsU, lNumSplitsV)] ;
//                 break ;                                    
//        case 1 : // Below Left Corner - fetch Previous (Above=Post) TreeVertex Index in TreeVertexLoop for this Node
//                 lPreIndex =   (   iiIndx == lNumSplitsU-1 
//                                || jjIndx == lNumSplitsV-1) ? SM_BIG_ULONG
//                             : (   iiIndx == 0
//                                && SM_BIG_ULONG != rTreeVertexPost[BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)])                  
//                                                            ? rTreeVertexPost[BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)]
//                             :                                rTreeVertexMap[ArrayIJToK(iiIndx, jjIndx+1, lNumSplitsU, lNumSplitsV)] ;
//                 break ;
//        case 2 : // Below Right Corner - fetch Previous (Left=Pre) TreeVertex Index in TreeVertexLoop for this Node
//                 lPreIndex =   (   iiIndx == 0 
//                                || jjIndx == lNumSplitsV-1) ? SM_BIG_ULONG
//                             : (   jjIndx == 0
//                                && SM_BIG_ULONG != rTreeVertexPre[BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)])                  
//                                                            ? rTreeVertexPre[BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)]
//                             :                                rTreeVertexMap[ArrayIJToK(iiIndx-1, jjIndx, lNumSplitsU, lNumSplitsV)] ;
//                 break ;                                    
//        case 3 : // Above Right Corner - fetch Previous (Below=Pre) TreeVertex Index in TreeVertexLoop for this Node
//                 lPreIndex =   (   iiIndx == 0 
//                                || jjIndx == 0)             ? SM_BIG_ULONG
//                             : (   iiIndx == lNumSplitsU-1
//                                && SM_BIG_ULONG != rTreeVertexPre[BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)])      
//                                                            ? rTreeVertexPre[BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)]
//                             :                                rTreeVertexMap[ArrayIJToK(iiIndx, jjIndx-1, lNumSplitsU, lNumSplitsV)] ;
//                 break ;
//      } // end switch on Corner Indx
//  
//    // all done
//    return(lPreIndex) ;
//  
//  } // end SmTree::GetPreTreeVertexIndex

/*******************************************************************//**
PURPOSE: Fetch the Next TreeVertex Index in a TreeVertexLoop for
  a specified TreeVertex Index given the TreeVertex Maps 
  created by BuildTreeVertexMaps() that map all the TreeVertices 
  to be used to create a set of new subdivision nodes from a parent node.

NOTES: 
***********************************************************************/
// GWCTreeVertexTemp
// ULONG SmTree::GetPostTreeVertexIndex
//  (ULONG iiIndx,                           // in : ii of TreeVertex Array(ii,jj)
//   ULONG jjIndx,                           // in : jj of TreeVertex Array(ii,jj)
//   ULONG lCornerIndx,                      // in : oneof 0=ALCorner,1=BLCorner,2=BRCorner,3=ARCorner
//   ULONG lNumSplitsU,                      // in : Total Number of U Splits in TreeVertex Array(ii,jj)
//   ULONG lNumSplitsV,                      // in : Total Number of V Splits in TreeVertex Array(ii,jj)
//   SmTArray<ULONG> & rTreeVertexMap,       // in : Array made by BuildTreeVertexMaps call        
//   SmTArray<ULONG> & rTreeVertexPre,       // in : Array Made By BuildTreeVertexMaps call
//   SmTArray<ULONG> & rTreeVertexPost)      // in : Array Made By BuildTreeVertexMaps call
//  const
// {
//   // return value
//   ULONG lPreIndex = SM_BIG_ULONG ;
// 
//   // switch on the subdivision node corner being queired
//   switch(lCornerIndx)
//     {
//       case 0 : // Above Left Corner - fetch Next (Down==Pre) TreeVertex Index in TreeVertexLoop for this Node
//                lPreIndex =   (   iiIndx == lNumSplitsU-1 
//                               || jjIndx == 0)             ? SM_BIG_ULONG
//                            : (   iiIndx == 0
//                               && SM_BIG_ULONG != rTreeVertexPre[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)])
//                                                           ? rTreeVertexPre[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)]
//                            :                                rTreeVertexMap[SmTree::ArrayIJToK(iiIndx, jjIndx-1, lNumSplitsU, lNumSplitsV)] ;
//                break ;                                    
//       case 1 : // Below Left Corner - fetch Next (right=Post) TreeVertex Index in TreeVertexLoop for this Node
//                lPreIndex =   (   iiIndx == lNumSplitsU-1 
//                               || jjIndx == lNumSplitsV-1) ? SM_BIG_ULONG
//                            : (   jjIndx == 0
//                               && SM_BIG_ULONG != rTreeVertexPost[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)])                
//                                                           ? rTreeVertexPost[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)]
//                            :                                rTreeVertexMap[SmTree::ArrayIJToK(iiIndx+1, jjIndx, lNumSplitsU, lNumSplitsV)] ;
//                break ;
//       case 2 : // Below Right Corner - fetch Next (Above=Post) TreeVertex Index in TreeVertexLoop for this Node
//                lPreIndex =   (   iiIndx == 0 
//                               || jjIndx == lNumSplitsV-1) ? SM_BIG_ULONG
//                            : (   iiIndx == lNumSplitsU-1
//                               && SM_BIG_ULONG != rTreeVertexPost[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)])    
//                                                           ? rTreeVertexPost[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_V, lNumSplitsU, lNumSplitsV)]
//                            :                                rTreeVertexMap[SmTree::ArrayIJToK(iiIndx, jjIndx+1, lNumSplitsU, lNumSplitsV)] ;
//                break ;                                    
//       case 3 : // Above Right Corner - fetch Next (Left=Pre) TreeVertex Index in TreeVertexLoop for this Node
//                lPreIndex =   (   iiIndx == 0 
//                               || jjIndx == 0)             ? SM_BIG_ULONG
//                            : (   jjIndx == lNumSplitsV-1
//                               && SM_BIG_ULONG != rTreeVertexPre[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)])    
//                                                           ? rTreeVertexPre[SmTree::BorderIJToK(iiIndx, jjIndx, SM_SP_U, lNumSplitsU, lNumSplitsV)]
//                            :                                rTreeVertexMap[SmTree::ArrayIJToK(iiIndx-1, jjIndx, lNumSplitsU, lNumSplitsV)] ;
//                break ;
//     } // end switch on Corner Indx
// 
//   // all done
//   return(lPreIndex) ;
// 
// } // end SmTree::GetPostTreeVertexIndex

/*******************************************************************//**
PURPOSE: Given a Parent Start TreeVertexIndex and arrays for
  U and V splits, build maps of the new ChildNode TreeVertex Indices
  and the new TreeVertex Last/Next neighbors - increment m_sTreeVertices as needed.

NOTES: With no initial TreeVertices Builds TreeVertex Index Scheme as
                         4   9   14  19
   +------------+         +---+----+---+
   |            |        3|  8|  13| 18|
   |            |         +---+----+---+
   |            |        2|  7|  12| 17|
   |            |   ==>   +---+----+---+
   |            |        1|  6|  11| 16|
   |            |         +---+----+---+
   |            |        0|  5|  10| 15|
   +------------+         +---+----+---+
   
 With existing TreeVertices adds new TreeVertices around the old 
  0   4        3         0   4   17   3
   +---+--------+         +---+----+---+  GWC: notice how prexisting corner and mid-edge     
   |            |        8| 12|  16| 21|       TreeVertices propogate to the final TreeVertex    
  5|            |        5+---+----+---+       map for the subdivided block.  Some of those     
   +            |        7+ 11|  15| 20|       are coincident (0 1 2 3 4) with new vertices and     
   |            |   ==>   +---+----+---+       some aren't (5 6).  The existence of preexisting     
   |            |        6| 10|  14| 19|       TreeVertices causes the final TreeVertex numbering    
   |            |         +---+----+---+       to be irregular.  So a TreeVertex Index map    
  1|           2|        1|  9|  13| 18|       is used rather than a regular numbering scheme.    
   +------------+         +---+----+---+
   
       0.) A parent node can be split into 2 children (all new TreeVertices
           are on the parent's original Node boundary) or into an
           array of Child Nodes (new TreeVertices are created on
           both the Parent's original Node Boundary and within the
           original parent's interior.)
       
       1.) Uses existing TreeVertex indices when existing TreeVertices
           are coincident with new ChildNode TreeVertices.

       2.) Includes existing TreeVertex indices where appropriate
           in the Next/Last neighbor connections for the New TreeVertex indices.

       3.) Only those new TreeVertices on the original parent node's
           loop boundary can be connected to existing Tree Boundaries.
           So, all Next and Last connections for internal New Tree Vertices
           are just their neighbors in the sTreeVertexMap.

       4.) Next and Last neighbors for TreeVertices on the boundary
           of the parent's original node in the direction of the Boundary
           are stored in the rTreeVertexPre and rTreeVertexPost arrays.
           Their connections into the interior of the Array are
           just their neighbors in the rTreeVertexMap.
***********************************************************************/
// GWCTreeVertexTemp
// SmStatus SmTree::BuildTreeVertexMaps
//  (SmTreeNode       * pNode,            // in : Parent Node being split before TreeVertices entries are updated or NULL for none
//   SmTArray<double> & rUSplits,         // in : rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//   SmTArray<double> & rVSplits,         // in : rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
// 
//   SmTArray<ULONG>  & rTreeVertexMap,   // out: Index for every U/V split TreeVertex intersection
//                                        //       access Array[i,j]: k = ArrayIJToK(i,j,lNumSplitsU,lNumSplitsV)
//                                        //       sized:[lNumSplitsU * lNumSplitsV]
//   SmTArray<ULONG>  & rTreeVertexPre,   // out: Last TreeVertexIndex between SplitPoints for every Border TreeVertexIndex
//                                        //       SM_BIG_ULONG = no TreeVertices between This and Last SplitPoint.
//                                        //       access Array[i,j]: k = BorderIJToK(i,j,UVDir,lNumSplitsU,lNumSplitsV)
//                                        //       sized:[2 * (lNumSplitsU + lNumSplitsV)]
//   SmTArray<ULONG>  & rTreeVertexPost)  // out: Next TreeVertexIndex between SplitPoints for every Border TreeVertexIndex
//                                        //       SM_BIG_ULONG = no TreeVertices between This and Next SplitPoint.
//                                        //       access Array[i,j]: k = BorderIJToK(i,j,UVDir,lNumSplitsU,lNumSplitsV)
//                                        //       sized:[2 * (lNumSplitsU + lNumSplitsV)]
// {
//   // locals
//   ULONG ii, jj, idx ;
//   ULONG lNumSplitsU   = rUSplits.GetSize() ;
//   ULONG lNumSplitsV   = rVSplits.GetSize() ;
// 
//   // pNode locals 
//   SmBezierAux2d * pAux = NULL ;
//   ULONG           lParentStartTreeVertexIndx = SM_BIG_ULONG;
//   SmTArray<ULONG> sNodeLoop ;
//   if(pNode) { pAux                       = pNode->GetBezierAux2d() ;
//               lParentStartTreeVertexIndx = pNode->GetStartTreeVertexIndx() ;
//               GetTreeVertexLoop(SM_BIG_ULONG, *pNode, sNodeLoop) ;
//             }
//     
//   // bad state - pNode and pNode is not a kindof Surface Subdivision TreeNode
//   if(   pNode != NULL 
//      && pAux  == NULL)
//     {
//       // set outputs
//       rTreeVertexMap .SetSize(0) ;
//       rTreeVertexPre .SetSize(0) ;
//       rTreeVertexPost.SetSize(0) ;
//       return(SM_ERR) ;
//     }
//   
//   // init outputs
//   rTreeVertexMap .SetSize(lNumSplitsU * lNumSplitsV) ;
//   rTreeVertexPre .SetSize(2 * (lNumSplitsU + lNumSplitsV)) ;
//   rTreeVertexPost.SetSize(2 * (lNumSplitsU + lNumSplitsV)) ;
// 
//   // locals
//   ULONG lOldTreeVertexCount = GetTreeVertexCount() ;
//   ULONG lTreeVertexCount    = lOldTreeVertexCount ;
// 
//   ULONG lNodeLoopSize = sNodeLoop.GetSize() ;
//   ULONG lLastVIndx, lNextVIndx ;
//   ULONG lLastUIndx, lNextUIndx ;
// 
//   // Load sTreeVertexMap with mix of existing and new TreeVertex Indices - count the NewVertices
//   for(ii=0,idx=0;ii<lNumSplitsU;ii++)
//     {
//       double    dU       = rUSplits[ii] ;
//       double    dLastU   = ii > 0               ? rUSplits[ii-1] : SM_BIG_DOUBLE ;
//       double    dNextU   = ii < lNumSplitsU - 1 ? rUSplits[ii+1] : SM_BIG_DOUBLE ;
//       SmBoolean lBorderU = (ii == 0 || ii == lNumSplitsU - 1) ;
// 
//       for(jj=0;jj<lNumSplitsV;jj++,idx++)
//         {
//           double    dV       = rVSplits[jj] ;
//           double    dLastV   = jj > 0               ? rVSplits[jj-1] : SM_BIG_DOUBLE ;
//           double    dNextV   = jj < lNumSplitsV - 1 ? rVSplits[jj+1] : SM_BIG_DOUBLE ;
//           SmBoolean lBorderV = (jj == 0 || jj == lNumSplitsV - 1) ;
// 
//           // look for coincident, Next, and Last Border TreeVertices
//           ULONG lTgtTreeVertexIndx  = SM_BIG_ULONG ;
//           ULONG lNextTreeVertexIndx = SM_BIG_ULONG ;
//           ULONG lPrevTreeVertexIndx = SM_BIG_ULONG ;
// 
//           // when on the border - look for Coincident, Next, and Last original NodeLoop TreeVertices
//           if(lBorderU || lBorderV)
//             {
//               // look for coincident old TreeVertices in the origin Node Loop
//               lTgtTreeVertexIndx = FindCoincidentTreeVertexIndx(dU, dV, &sNodeLoop) ;
//               
//               // when on U Border - look for NonCoincident Next and Last old TreeVertices in the V direction in the original Node Loop
//               if(lBorderU)
//                 { 
//                   lLastVIndx = SM_BIG_ULONG ;
//                   lNextVIndx = SM_BIG_ULONG ;
// 
//                   if(lNodeLoopSize > 4) // Last/Next VIndx = closest TreeVertexIndex between split points, SM_BIG_ULONG = no skipped TreeVertices between split points
//                     { FindNeighborTreeVertexIndx(dU, dV, &sNodeLoop, SM_SP_V, dLastV, dNextV, lLastVIndx, lNextVIndx) ; }
// 
//                   // convert [ii jj] values to a border [bi] value
//                   ULONG bi = BorderIJToK(ii, jj, SM_SP_V, lNumSplitsU, lNumSplitsV) ;
//                   
//                   // remember any finds
//                   rTreeVertexPre [bi] = lLastVIndx ; 
//                   rTreeVertexPost[bi] = lNextVIndx ;
//                 }   
// 
//               // when on V Border - look for NonCoincident Next and Last old TreeVertices in the U direction in the original Node Loop
//               if(lBorderV)
//                 { 
//                   lLastUIndx = SM_BIG_ULONG ;
//                   lNextUIndx = SM_BIG_ULONG ;
// 
//                   if(lNodeLoopSize > 4) // Last/Next UIndx = closest TreeVertexIndex between split points, SM_BIG_ULONG = no skipped TreeVertices between split points
//                     { FindNeighborTreeVertexIndx(dU, dV, &sNodeLoop, SM_SP_U, dLastU, dNextU, lLastUIndx, lNextUIndx) ; } 
//                   
//                   // convert [ii jj] values to a border [bi] value
//                   ULONG bi = BorderIJToK(ii, jj, SM_SP_U, lNumSplitsU, lNumSplitsV) ;
//                   
//                   // remember any finds
//                   rTreeVertexPre [bi] = lLastUIndx ; 
//                   rTreeVertexPost[bi] = lNextUIndx ;
//                 }   
//             
//             } // end when on the border check
// 
//           // set the next TreeVertex Index - it's either an old coincident index or a new assigned index
//           if(lTgtTreeVertexIndx != SM_BIG_ULONG)
//             {  // an old coincident TreeVertex
//                rTreeVertexMap[idx] = lTgtTreeVertexIndx ; // old TreeVertex index
//             }
//           else // a new TreeVertex index
//             { 
//               rTreeVertexMap[idx] = lTreeVertexCount ;    // a new TreeVertex index - increment TreeVertexCount
//               lTreeVertexCount++ ;
//             }
// 
//         } // end iter every V Split
//     } // end iter every U Split
// 
//   // set TreeVertex increment size
//   ULONG lIncTreeVertexCount = lTreeVertexCount - lOldTreeVertexCount ;
// 
//   // side effect : increase the number of TreeVertices
//   // note: This call is here to simplify the user interface making
//   //       the size of the Vertex Array an internally managed detail.
//   IncTreeVertexCount(lIncTreeVertexCount) ;
// 
//   // all done
//   return(SM_SUCCESS) ; 
// 
// } // end SmTree::BuildTreeVertexMaps

/*******************************************************************//**
PURPOSE: Find the index of the TreeVertex that is coincident with given
         UVPoint

NOTES: 1.) When given an optional TreeVertex index list only check
           the given indices.

       2.) return SM_BIG_ULONG for no coincident TreeVertex found.
***********************************************************************/
// GWCTreeVertexTemp
//  ULONG SmTree::FindCoincidentTreeVertexIndx
//   (double dUVPointU,                 // in : UVPoint U position
//    double dUVPointV,                 // in : UVPoint V position
//    SmTArray<ULONG> * pOptVertexLoop) // in : opt list of indices to search, NULL=search all
//   const
//  { 
//    // locals
//    ULONG ii, iTgt, iCnt = pOptVertexLoop ? pOptVertexLoop->GetSize() : m_sTreeVertices.GetSize() ;
//    double dTgtPointU, dTgtPointV ;
//  
//    // for every TreeVertex (or every given TreeVertex) - search for coincidence
//    for(ii=0;ii<iCnt;ii++)
//      { 
//        iTgt = pOptVertexLoop ? pOptVertexLoop->GetAt(ii) : ii ;
//        m_sTreeVertices[iTgt].GetUVPoint(dTgtPointU, dTgtPointV) ;
//  
//        // when TreeVertex is coincident to TgtPointUV - return its indx
//        if(   SM_IS_ZERO(dTgtPointU - dUVPointU)
//           && SM_IS_ZERO(dTgtPointV - dUVPointV))
//          { return(iTgt) ; }
//  
//      } // end iter every TreeVertex to search
//  
//    // arrive here when a coincident TreeVertex was not found
//    return(SM_BIG_ULONG) ;
//  
//  } // end SmTree::FindCoincidentTreeVertexIndx
  
/*******************************************************************//**
PURPOSE: Find the indices of the TreeVertices within a given U/V direction  
    interval that are nearest, without being coincident with given UVPoint.

NOTES: 1.) When given an optional TreeVertex index list only check
           the given indices.

       2.) return SM_BIG_ULONG for no neighbors within specified interval.
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTree::FindNeighborTreeVertexIndx
//   (double            dUVPointU,        // in : UVPoint U position
//    double            dUVPointV,        // in : UVPoint V position
//    SmTArray<ULONG> * pOptVertexLoop,   // in : opt list of indices to search, NULL=search all
//    SmSurfParamType   eSurfDir,         // in : oneof SM_SP_U or SM_SP_V
//    double            dLastParam,       // in : Last SearchInterval:[LastParam UVPointParam], SM_BIG_DOUBLE to ignore
//    double            dNextParam,       // in : Next SearchInterval:[UVPointParam NextParam], SM_BIG_DOUBLE to ignore
//    ULONG           & rLastIndx,        // out: Indx closest to UVPoint in given direction, SM_BIG_ULONG = None
//    ULONG           & rNextIndx)        // out: Indx closest to UVPoint in given direction, SM_BIG_ULONG = None
//   const
//  { 
//    // init output 
//    rLastIndx = SM_BIG_ULONG ;
//    rNextIndx = SM_BIG_ULONG ;
//  
//    // no work - no intervals to check
//    if(   dLastParam == SM_BIG_DOUBLE
//       && dNextParam == SM_BIG_DOUBLE)
//      { return ; }
//  
//    // locals
//    ULONG  ii, iTgt, iCnt = pOptVertexLoop ? pOptVertexLoop->GetSize() : m_sTreeVertices.GetSize() ;
//    double dTgtPointU,  dTgtPointV ;
//    double dLastPointU, dLastPointV ;
//    double dNextPointU, dNextPointV ;
//  
//    // Look in U dir - V param must be coincident
//    if(eSurfDir == SM_SP_U)
//      {
//        // for every TreeVertex
//        for(ii=0;ii<iCnt;ii++)
//          {
//            iTgt = pOptVertexLoop ? pOptVertexLoop->GetAt(ii) : ii ;
//            m_sTreeVertices[iTgt].GetUVPoint(dTgtPointU, dTgtPointV) ;
//  
//            // find a Coincident V value in the given intervals
//            if(SM_IS_ZERO(dTgtPointV - dUVPointV))
//              {
//                // when asked, Find the nearest last (left) TreeVertex Index
//                if(   dLastParam != SM_BIG_DOUBLE 
//                   && SM_IS_BETWEEN_TO_TOL(dTgtPointU, dUVPointU, dLastParam, SM_EFF_ZERO)
//                   && (   rLastIndx == SM_BIG_ULONG
//                       || dLastPointU < dTgtPointU))
//                  {
//                    // store the best Last TreeVertex Index
//                    rLastIndx   = iTgt ;
//                    dLastPointU = dTgtPointU ;
//                    dLastPointV = dTgtPointV ;
//                  } 
//  
//                // when asked, Find the nearest Next (right) TreeVertex Index
//                if(   dNextParam != SM_BIG_DOUBLE 
//                   && SM_IS_BETWEEN_TO_TOL(dTgtPointU, dUVPointU, dNextParam, SM_EFF_ZERO)
//                   && (   rNextIndx == SM_BIG_ULONG
//                       || dNextPointU > dTgtPointU))
//                  {
//                    // store the best Next TreeVertex Index
//                    rNextIndx   = iTgt ;
//                    dNextPointU = dTgtPointU ;
//                    dNextPointV = dTgtPointV ;
//                  } 
//  
//              } // end Found a coincident V check
//          } // end iter every TreeVertex
//      } // end Look in U dir (left and right)
//    else // Look in V dir (above and below)
//      {
//        // for every TreeVertex
//        for(ii=0;ii<iCnt;ii++)
//          {
//            iTgt = pOptVertexLoop ? pOptVertexLoop->GetAt(ii) : ii ;
//            m_sTreeVertices[iTgt].GetUVPoint(dTgtPointU, dTgtPointV) ;
//  
//            // find a Coincident U value in the given intervals
//            if(SM_IS_ZERO(dTgtPointU - dUVPointU))
//              {
//                // when asked, Find the nearest last (Below) TreeVertex Index
//                if(   dLastParam != SM_BIG_DOUBLE 
//                   && SM_IS_BETWEEN_TO_TOL(dTgtPointV, dUVPointV, dLastParam, SM_EFF_ZERO)
//                   && (   rLastIndx == SM_BIG_ULONG
//                       || dLastPointV < dTgtPointV))
//                  {
//                    // store the best Last TreeVertex Index
//                    rLastIndx   = iTgt ;
//                    dLastPointU = dTgtPointU ;
//                    dLastPointV = dTgtPointV ;
//                  } 
//  
//                // when asked, Find the nearest Next (Above) TreeVertex Index
//                if(   dNextParam != SM_BIG_DOUBLE
//                   && SM_IS_BETWEEN_TO_TOL(dTgtPointV, dUVPointV, dNextParam, SM_EFF_ZERO)
//                   && (   rNextIndx == SM_BIG_ULONG
//                       || dNextPointV > dTgtPointV))
//                  {
//                    // store the best Next TreeVertex Index
//                    rNextIndx   = iTgt ;
//                    dNextPointU = dTgtPointU ;
//                    dNextPointV = dTgtPointV ;
//                  } 
//  
//              } // end Found a coincident U check
//          } // end iter every TreeVertex
//      } // end Look in V dir (above and below)
//  
//  } // end SmTree::FindNeighborTreeVertexIndx

/*******************************************************************//**
PURPOSE: Find and return the smallest node in the tree which completely
            contains the input box.

NOTES: 
  1. Function can return either a leaf or an internal node.
  2. NULL is returned when the input box does not fit within the TopNode bounding box.  
***********************************************************************/
SmTreeNode * SmTree::FindNode
  (const SmExtent3d & crBox)   // in : Box to classify against the nodes of this Tree
 const
{
  // locals - use a treeNode array as a stack
  SmTreeNode *aData[20];
  SmTArray<SmTreeNode*> sStack(20,aData);

  // init stack with TopNode
  sStack.Add(GetTopNode());
  SmTreeNode *pNode = sStack.GetLast();

  // walk the tree until an exit case 
  while (TRUE) 
    {
      // exit case - containing leaf node
      if (!pNode->m_pChild1) 
        {
          return pNode;
        }
      else // Have children - check them against the box
        { 
          // when child contains crBox - move to that node and iterate
          if      (crBox.IsContainedBy(pNode->m_pChild1->m_sBBox, SM_EFF_ZERO)) { pNode = pNode->m_pChild1 ; }
          else if (crBox.IsContainedBy(pNode->m_pChild2->m_sBBox, SM_EFF_ZERO)) { pNode = pNode->m_pChild2 ; }
          else // exit case - the children are too small - just return this internal node
            {
              // error check - this only happens if the crBox does not fit into the tree BBox
              if (!crBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO)) 
                {
                  return NULL;
                }
              // return the internal node that completely contains crBox
              return pNode;
            } // end exit case - found containing internal node
        } // end internal node branch
    } // end while walking the tree looking for an exit case

} // end SmTree::FindNode

/*******************************************************************//**
PURPOSE: The first leaf node of this tree.

NOTES: Return NULL if no nodes in the tree.
***********************************************************************/
SmTreeNode * SmTree::GetFirstLeafNode() const
{
    if (m_pTopNode == NULL) return NULL;
    SmTreeNode *pRet = m_pTopNode;
    while (pRet->m_pChild1 != NULL) 
      {
        pRet = pRet->m_pChild1;
      }
    return pRet;

} // end SmTree::GetFirstLeafNode
 

/*******************************************************************//**
PURPOSE: Get the next leaf node in this tree moving to the right 
    and down.  

NOTES: The current node must be a leaf node. Assumes each parent
    has two children.
***********************************************************************/
SmTreeNode * SmTree::GetNextLeafNode
  (SmTreeNode *pCurrentLeafNode) 
 const
{
    if (pCurrentLeafNode == NULL) return NULL;
    SmTreeNode *pRet = NULL;
    // First go up the tree until we reach point where we
    // can go left.
    SmTreeNode *pNode = pCurrentLeafNode;
    while (pNode->m_pParent != NULL) 
      {
        SmTreeNode *pParent = pNode->m_pParent;
        if (pParent->m_pChild1 == pNode) 
          {
            // Have found stopping point to start back down
            pNode = pParent->m_pChild2;
            break;
          }
        pNode = pParent; // Keep moving up we must have come
        // form the right hand side of this parent.
      }

    if (pNode->m_pParent == NULL) 
      {
        return NULL;
      }

    pRet = pNode;
    while (pRet->m_pChild1 != NULL) 
      {
        pRet = pRet->m_pChild1;
      }
    return pRet;

} // end SmTree::GetNextLeafNode

/*******************************************************************//**
PURPOSE: set all tree node->m_bMarked states to FALSE

NOTES: 
***********************************************************************/
void SmTree::ClearNodeMarks()
{
  // pass call along
  if(m_pTopNode)
    {
      ((SmBezierAux2d *)m_pTopNode->m_pData)->m_bIsMarked = FALSE ; 
       m_pTopNode->PropagateToChildren() ;
    }

} // end SmTree::ClearNodeMarks

/*******************************************************************//**
PURPOSE: Display the bounding boxes of the spatial tree.  
  
  For each deeper child level, each bounding box is drawn with
    color     += [red_inc, green_inc, blue_inc].
    lineWidth += 0.75

NOTES: 
***********************************************************************/
SmDisplayList * SmTree::DrawNodeBoundingBoxes
  (double red,     double green,     double blue,
   double red_inc, double green_inc, double blue_inc)
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  // create or use currently open DisplayList
  smgfx_Open(smgfx_GetRuleColor(this));

  // save the starting color and linewidth
  SmVector3d sColor         = smgfx_GetColor() ;
  double     linewidth_init = smgfx_GetLineWidth() ;
#endif

#ifdef SM_DEBUG_CODE
  // gather some statistics on this tree
  ULONG node_count = GetNodeCount() ; 
  if(node_count <= 0) SE(SM_ERR) ;

#endif // SM_DEBUG_CODE

  // walk the tree recursively - 
  // Draw each node's bounding box
  // For each deeper child level, draw each bounding box  
  //   with incremented color and linewidth
  GetTopNode()->DrawBoundingBox(red,     green,     blue,
                                red_inc, green_inc, blue_inc,
                                0.75) ;
#ifdef SM_GFX_OUTPUT_CODE
  // restore color and linewidth parameters
  smgfx_OutputColor(sColor) ;
  smgfx_OutputLineWidth(linewidth_init) ;

  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmTree::DrawNodeBoundingBoxes

/*******************************************************************//**
PURPOSE: return an array of all SmObjectList objects found 
            in this TreeNode's linked-list of SmObjectLists.

NOTES: This is a data conversion operation
                converting from a linked-list to an array.
***********************************************************************/
SmStatus SmTreeNode::GetObjectList
  (SmTArray<SmObjectList*> & rObjectsInNode) // out: array of SmObjectList objects in this treeNode
                                             //      note: SmObjectList contains
                                             //             next and object pointers and a bounding box
 const
{
  if (m_eAuxDataType != SM_AD_OBJECT_LIST) SER(SM_ERR_INVALID_INPUT);

  rObjectsInNode.ReSet();
  SmObjectList *pObjectList = (SmObjectList*)m_pData;
  while (pObjectList) 
    {
      rObjectsInNode.Add(pObjectList);
      pObjectList = pObjectList->m_pNext;
    }
  return SM_SUCCESS;

} // end SmTreeNode::GetObjectList

/*******************************************************************//**
PURPOSE: Get the object list for the entire tree.

NOTES: Add every ObjectList from each node's linked-list of
  of ObjectLists into the output array.
***********************************************************************/
SmStatus SmTree::GetObjectList
  (SmTArray<SmObjectList*> & rObjectsInNode)  // out: array of all ObjectList items
                                              //      found in this tree.
 const
{
  // init output
  rObjectsInNode.ReSet();

  // get array of all Nodes in tree
  void *sData[2048];
  SmTArray<void*> sNodes(2048,sData);
  m_sNodeMgr.GetActiveElements(sNodes);

  // for every node
  for (ULONG i=0; i<sNodes.GetSize(); i++) 
    {
      SmTreeNode *pNode = (SmTreeNode*)sNodes[i];

      // skip nodes without object lists
      if (pNode->m_eAuxDataType != SM_AD_OBJECT_LIST) 
        continue;

      // add each ObjectList in the linked-list to the output array
      SmObjectList *pObjectList = (SmObjectList*)pNode->m_pData;
      while (pObjectList) 
        {
          rObjectsInNode.Add(pObjectList);
          pObjectList = pObjectList->m_pNext;
        }
    } // end iter every array

  // all done
  return SM_SUCCESS;

} // end SmTree::GetObjectList

/*******************************************************************//**
PURPOSE: Get all objects in the tree whose BBoxes intersect a given bounding box.

NOTES: 
***********************************************************************/
SmStatus SmTree::GetObjectsInBox
  (const SmExtent3d    & crBBox,         // in : target extent
   SmTArray<SmObject*> & rObjectsInBBox) // out: array of Objects whose BBoxes intersect the target extent
  const
{
  // init output
  rObjectsInBBox.ReSet();

  // locals: Node Object Array and a Node Stack
  SmObjectList * sData[512];
  SmTArray<SmObjectList*> sObjsInNode(512,sData);
  SmTreeNode *sData2[64];
  SmTArray<SmTreeNode*> sStack(64,sData2);

  // init stack with topNode
  sStack.Add(GetTopNode());

  // while walking the tree
  while (sStack.GetSize() > 0) 
    {
      SmTreeNode *pNode = sStack.GetLast();
      sStack.RemoveLast();

      // when Node BBox intersects Target BBox
      if (!pNode->m_sBBox.AreDisjoint(crBBox)) 
        {
          // for every ObjectList in this node
          SM_ASSERT(pNode->m_eAuxDataType == SM_AD_OBJECT_LIST) ;
          SmObjectList *pObjectList = (SmObjectList*)pNode->m_pData;
          while (pObjectList) 
            {
              // when the Object's BBox intersect the input BBox
              if (!pObjectList->m_sBBox.AreDisjoint(crBBox)) 
                {
                  // add it to the output
                  rObjectsInBBox.Add(pObjectList->m_pObject);
                }
              pObjectList = pObjectList->m_pNext;
            }

          // add children to the stack for further study
          if (pNode->m_pChild1 != NULL) 
            {
              sStack.Add(pNode->m_pChild1);
              sStack.Add(pNode->m_pChild2);
            }
        } // end intersecting BBox check
    } // end processing the stack

  // all done
  return SM_SUCCESS;

} // end SmTree::GetObjectsInBox


/*******************************************************************//**
PURPOSE: Get the ObjectList elements in the tree which intersect a 
     given bounding box.

NOTES: Do not delete any of the objects in the rObjectsInBBox
     array.  They are owned by the tree.
***********************************************************************/
SmStatus SmTree::GetObjectListInBox
  (const SmExtent3d        & crBBox,              // in : target extent
   SmTArray<SmObjectList*> & rObjectListsInBBox)  // out: array of ObjectLists whose BBoxes intersect the target extent
  const
{
  // init output
  rObjectListsInBBox.ReSet();

  // locals: Node Object Array and a Node Stack
  SmObjectList * sData[512];
  SmTArray<SmObjectList*> sObjsInNode(512,sData);
  SmTreeNode *sData2[64];
  SmTArray<SmTreeNode*> sStack(64,sData2);

  // init stack with topNode
  sStack.Add(GetTopNode());

  // while walking the tree
  while (sStack.GetSize() > 0) 
    {
      SmTreeNode *pNode = sStack.GetLast();
      sStack.RemoveLast();

      // when Node BBox intersects Target BBox
      if (!pNode->m_sBBox.AreDisjoint(crBBox)) 
        {
          // for every ObjectList in this node
          SM_ASSERT(pNode->m_eAuxDataType == SM_AD_OBJECT_LIST) ;
          SmObjectList *pObjectList = (SmObjectList*)pNode->m_pData;
          while (pObjectList) 
            {
              // when the Object's BBox intersect the input BBox
              if (!pObjectList->m_sBBox.AreDisjoint(crBBox)) 
                {
                  // add it to the output
                  rObjectListsInBBox.Add(pObjectList);
                }
              pObjectList = pObjectList->m_pNext;
            }

          // add children to the stack for further study
          if (pNode->m_pChild1 != NULL) 
            {
              sStack.Add(pNode->m_pChild1);
              sStack.Add(pNode->m_pChild2);
            }
        } // end intersecting BBox check
    } // end processing the stack

  // all done
  return SM_SUCCESS;

} // end SmTree::GetObjectListInBox

/*******************************************************************//**
PURPOSE: Determine if the tree intersects the given 3D bounding box.

NOTES: 
***********************************************************************/
SmBoolean SmTree::IntersectsBox
  (const SmExtent3d & cr3DBox) 
 const
{
    SmTreeNode *sData[40];
    SmTArray<SmTreeNode*> sStack(40,sData);
    sStack.Add(GetTopNode());
    while (sStack.GetSize() > 0) 
      {
        SmTreeNode *pNode = sStack.GetLast();
        sStack.RemoveLast();
        if (!pNode->m_sBBox.AreDisjoint(cr3DBox)) 
          {
            if (pNode->m_pChild1 == NULL) 
              {
                return TRUE;
              }
            else 
              {
                sStack.Add(pNode->m_pChild1);
                sStack.Add(pNode->m_pChild2);
              }
          }
      }

    return FALSE;

} // end SmTree::IntersectsBox

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
ULONG SmTree::GetMemoryUsed
  (ULONG &rlMemoryAllocated)
   const
{
  ULONG lNodeAlloc=0, lListAlloc=0 ;
  ULONG lUsed =   sizeof(SmTree) 
                + m_sNodeMgr.GetMemoryUsed(lNodeAlloc) 
                + m_sListMgr.GetMemoryUsed(lListAlloc) ;

  // set output
  rlMemoryAllocated =   sizeof(SmTree) 
                      + lNodeAlloc
                      + lListAlloc ;
  
  // all done
  return(lUsed) ;

} // end SmTree::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Check m_sTreeVertices array for consistent connections
  and a sparsley sampled regular grid.

NOTES: 
RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
// GWCTreeVertexTemp
// SmBoolean SmTree::AssertTreeVertices()  const
// {
//   // return value
//   SmBoolean bRtn = TRUE ;
// 
//   ULONG ii, jj ;
//   double dOtherU, dOtherV, dTgtU, dTgtV ;
// 
//   // for every TreeVertex - check connectivity consistency,
//   //                        monotonically aligned UV Points,
//   //                        TreeNode Ptr consistency
//   for(ii=0;bRtn && ii<m_sTreeVertices.GetSize();ii++)
//     {
//       const SmTreeVertex &rTreeVertex = m_sTreeVertices[ii] ;
//       rTreeVertex.GetUVPoint(dTgtU, dTgtV) ;
// 
//       // for every connect direction
//       for(jj=0;bRtn && jj<4;jj++)
//         {
//           // check Connection: 0=Below, 1=Right, 2=Above, 3=Left
//           if(rTreeVertex.GetConnect(jj) != SM_BIG_ULONG)
//             {
//               const SmTreeVertex &rOther = m_sTreeVertices[rTreeVertex.GetConnect(jj)] ;
//               rOther.GetUVPoint(dOtherU, dOtherV) ;
// 
//               // check: aligned and monotonic
//               bRtn &= (  (jj==0) ? ( (dOtherV < dTgtV) && SM_ARE_SAME(dOtherU, dTgtU) )  // Below
//                        : (jj==1) ? ( (dOtherU > dTgtU) && SM_ARE_SAME(dOtherV, dTgtV) )  // Right
//                        : (jj==2) ? ( (dOtherV > dTgtV) && SM_ARE_SAME(dOtherU, dTgtU) )  // Above
//                        : (jj==3) ? ( (dOtherU < dTgtU) && SM_ARE_SAME(dOtherV, dTgtV) )  // Left
//                                  : FALSE) ;
// 
//               // check: consistent back pointer
//               bRtn &= (ii == rOther.GetConnect((jj+2)%4) ) ;
// 
//               // check: consistent TreeNode pointers 
//               //   Vert(dir) == Vert: Test Vert(NodeDir) == Vert(NodeDir) && Vert(NodeDir) == Vert(NodeDir)
//               //    When A(jj=0) = B: Test( A(0) == B(1) && A(3) == B(2) )                                 
//               //    When A(jj=1) = B: Test( A(1) == B(2) && A(0) == B(3) ) noting the pattern in these relationships yields                                
//               //    When A(jj=2) = B: Test( A(2) == B(3) && A(1) == B(0) )                                 
//               //    When A(jj=3) = B: Test( A(3) == B(0) && A(2) == B(1) ) 
//               bRtn &= (   (rTreeVertex.GetTreeNode(jj)       == rOther.GetTreeNode((jj+1)%4)) 
//                        && (rTreeVertex.GetTreeNode((jj+3)%4) == rOther.GetTreeNode((jj+2)%4))) ;
// 
//             } // end used connection branch
//           else // connection in this direction not used
//             {
//               // test consistent TreeNode pointers, for each unused connection two TreeNodes ptrs have to be the same
//               bRtn &= rTreeVertex.GetTreeNode(jj) == rTreeVertex.GetTreeNode((jj+3)%4) ; 
// 
//             }
//         } // end iter all 4 connect directions
//     } // end iter every TreeVertex
// 
// #ifdef SM_DEBUG_CODE
// SmBoolean bDebugMe = FALSE ;
//   if(bDebugMe)
//     {
//       DumpTreeVertices() ;
//     }
// #endif // SM_DEBUG_CODE
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmTree::AssertTreeVertices 

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertTree_list[] =
{
  /*  0 */ {SM_AT_BOX,         _T("Bad TreeNode BBox3d size"),                 _T("TreeNode BBox3d is zero or negative for NonDegenerate Curve/Surface") },
  /*  1 */ {SM_AT_BOX,         _T("Bad TreeNode BBox3d containment"),          _T("TreeNode BBox3d not contained by Parent BBox3d") },
  /*  2 */ {SM_AT_BOX,         _T("Bad TreeNode Child1 BBox3d"),               _T("Tree ParentNode BBox3d does not contain pChild1 BBox3d") },
  /*  3 */ {SM_AT_BOX,         _T("Bad TreeNode Child2 BBox3d"),               _T("Tree ParentNode BBox3d does not contain pChild2 BBox3d") },
  /*  4 */ {SM_AT_DOMAIN,      _T("Bad TreeNode UVDomain containment"),        _T("TreeNode UVDomain not contained by Parent UVdomain") },
  /*  5 */ {SM_AT_DOMAIN,      _T("Bad TreeNode Child1 UVDomain"),             _T("TreeNode UVDomain does not contain pChild1 UVDomain") },
  /*  6 */ {SM_AT_DOMAIN,      _T("Bad TreeNode Child2 UVDomain"),             _T("TreeNode UVDomain does not contain pChild2 UVDomain") },
  /*  7 */ {SM_AT_DOMAIN,      _T("Bad TreeNode Bezier UVDomain containment"), _T("TreeNode Bezier UVDomain not contained by Parent Bezier UVdomain") },
  /*  8 */ {SM_AT_DOMAIN,      _T("Bad TreeNode Child1 Bezier UVDomain"),      _T("TreeNode Bezier UVDomain does not contain pChild1 Bezier UVDomain") },
  /*  9 */ {SM_AT_BOX,         _T("Bad TreeNode Child2 Bezier UVDomain"),      _T("TreeNode Bezier UVDomain does not contain pChild2 Bezier UVDomain") },
  /* 10 */ {SM_AT_BOX,         _T("Bad TreeNode Pseudo Box Size"),             _T("TreeNode PseudoBox3d is zero or negative for NonDegenerate Curve/Surface") },  
  /* 11 */ {SM_AT_BOX,         _T("Bad TreeNode Pseudo Box containment"),      _T("TreeNode PseudoBox3d Volume is greater than Parent PseudoBox3d Volume") },
  /* 12 */ {SM_AT_BOX,         _T("Bad TreeNode Child1 Pseudo Box"),           _T("TreeNode PseudoBox3d Volume is less than pChild1 PseudoBox3d Volume") },
  /* 13 */ {SM_AT_BOX,         _T("Bad TreeNode Child2 Pseudo Box"),           _T("TreeNode PseudoBox3d Volume is less than pChild2 PseudoBox3d Volume") },
  /* 14 */ {SM_AT_TOPOLOGICAL, _T("Bad TreeVertices"),                         _T("Bad SubdivisionTree Vertex Connectivity Graph") },
  /* 15 */ {SM_AT_TOPOLOGICAL, _T("Bad Tree LeafNode Start Index"),            _T("Tree LeafNode has bad StartIndex Value") }
} ;

/*******************************************************************//**
PURPOSE: Virtual method used to determine validity of objects.

NOTES: For spatial trees - checks that all the bounding boxes
                fit within one another including
                  UVDomain,
                  BBox,
                  PseudoBox,
                  PolarBox.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmTree::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // NotUsed: in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests);

  // return value
  SmBoolean bRtn = TRUE ;
  SmBoolean bOK ;

  // locals
  SmTArray<const SmTreeNode *> sQueue ;

  // owner object locals
  const SmObject  *pObject  = GetOwnerObject() ;
  SmCurve         *pCurve   = SM_CAST_PTR(SmCurve, pObject) ;
  SmSurface       *pSurface = SM_CAST_PTR(SmSurface, pObject) ;
  SmBoolean   bIsDegenerate =   pCurve   ? pCurve->IsDegenerate()
                              : pSurface ? pSurface->IsDegeneratePoint()
                              : FALSE ;

// GWCTreeVertexTemp
//    // TreeVertices are curently only used for Surface Subdivisions but should be a valid empty structure for all other cases
//    bRtn &= SM_ASSERT_BOOLEAN_REPORT(14, SM_LEVEL_0, AssertTreeVertices(), _T("")) ;

  // when m_pOwner is set this is a spatial tree, a curve decomposition tree or a surface decomposition tree - test bounding boxes
  if(m_pOwner)
    {
      // init the queue
      if(m_pTopNode)
        { sQueue.Add(m_pTopNode) ; }

      // while nodes are in the queue
      while(bRtn && sQueue.GetSize() > 0)
        {
          const SmTreeNode *pNode   = sQueue.GetLast() ;
          const SmTreeNode *pParent = pNode->m_pParent ;
          const SmTreeNode *pChild1 = pNode->m_pChild1 ;
          const SmTreeNode *pChild2 = pNode->m_pChild2 ;

          // set up the queue for further iterations
          sQueue.RemoveLast() ;
          if(pChild1) { sQueue.Add(pChild1) ; }
          if(pChild2) { sQueue.Add(pChild2) ; }

          // all bounding boxes should be non-negative
          bRtn &= pNode->m_sBBox.AssertValid(pAList) ;

          // all bounding boxes of nonDegenerate shapes should be non-zero
          bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (bIsDegenerate == TRUE || pNode->m_sBBox.GetMaxDimension() > SM_EFF_ZERO), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

          // when the node has a parent - its BBox should be contained
          bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (pParent == NULL || pNode->m_sBBox.IsContainedBy(pParent->m_sBBox, SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")  ) ;

          // when the node is a parent - its BBox should contain the children's
          bOK = pChild1 == NULL || pChild1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO) ;
          bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (bOK), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;
          bOK = pChild2 == NULL || pChild2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO) ;
          bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (bOK), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          if(bDebugMe)
            {
              if(pParent) pParent->m_sBBox.Dump() ;
              pNode->m_sBBox.Dump() ;                      
              if(pChild1) pChild1->m_sBBox.Dump() ;
              if(pChild2) pChild2->m_sBBox.Dump() ;

              Dump() ;
              pNode->DumpFamily() ;        

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pCurve) pCurve->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,4, 0,1,0) ; pNode->m_sBBox.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,6, 1,0,1) ; if(pParent) pParent->m_sBBox.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,8, 0,1,1) ; if(pChild1) pChild1->m_sBBox.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,8, 0,1,1) ; if(pChild2) pChild2->m_sBBox.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;

            }
#endif // SM_DEBUG_CODE

          // When the Node has an SmBezierAux2d m_pData Object from a surface decomposition
          if(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
             || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
             || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD)
            {
              SmBezierAux2d     * pAux       = (SmBezierAux2d *)pNode->m_pData ;
              SmBezierAux2d     * pParentAux = pParent ? (SmBezierAux2d *)pParent->m_pData : NULL ;
              SmBezierAux2d     * pChild1Aux = pChild1 ? (SmBezierAux2d *)pChild1->m_pData : NULL ;
              SmBezierAux2d     * pChild2Aux = pChild2 ? (SmBezierAux2d *)pChild2->m_pData : NULL ;
              const SmExtent2d  & rUVDomain  = pAux->GetUVDomain() ;
              //const SmPolarBox  * pChild1PolarBox = pChild1Aux == NULL ? NULL : pChild1Aux->GetPolarBoxPtr() ;
              //const SmPolarBox  * pChild2PolarBox = pChild2Aux == NULL ? NULL : pChild2Aux->GetPolarBoxPtr() ;
              const SmPolarBox  & rPolarBox  = pAux->GetPolarBox() ;
              
              // check the the UVDomains nest
              bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, (pParent    == NULL || rUVDomain.IsContainedBy(pParentAux->GetUVDomain(), SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, (pChild1Aux == NULL || pChild1Aux->GetUVDomain().IsContainedBy(rUVDomain, SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, (pChild2Aux == NULL || pChild2Aux->GetUVDomain().IsContainedBy(rUVDomain, SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("") ) ;

              // check the PolarBox within the SmBezierAux2d data type
              bRtn &= rPolarBox.AssertValid(pAList) ;

              //      // polar boxes don't have to nest because they each have their own coordinate system
              //      // but as a rule child polar boxes should be smaller or equal than parent boxes
              //      double dPolarThisArea   = rPolarBox.GetPolarArea() ;
              //      double dPolarParentArea = pParent ? pParentAux->GetPolarBox().GetPolarArea() : 2.0 * dPolarThisArea ;
              //      double dPolar1Area      = pChild1Aux ? pChild1PolarBox->GetPolarArea() : 0.0 ;
              //      double dPolar2Area      = pChild2Aux ? pChild2PolarBox->GetPolarArea() : 0.0 ;
              //      
              //      bRtn &= (dPolarParentArea >= dPolarThisArea - SM_EFF_ZERO_RAD) ;
              //      bRtn &= (dPolar1Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
              //      bRtn &= (dPolar2Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;

              // when the node has a parent - its UVDomain and Polar BBox should be contained
              //      bRtn &= (   pParent == NULL
              //               || rPolarBox.IsContainedBy(pParentAux->GetPolarBox())) ;

              // when the node is a parent - its UVDomain and Polar BBox should contain the children's
              //      bRtn &= (   pChild1Aux == NULL
              //               || pChild1PolarBox->IsContainedBy(rPolarBox)) ;

              //      bRtn &= (   pChild2Aux == NULL
              //               || pChild2PolarBox->IsContainedBy(rPolarBox)) ;

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  if(pParentAux) pParentAux->GetPolarBox().Dump() ;
                  rPolarBox.Dump() ;                      
                  if(pChild1Aux) pChild1Aux->GetPolarBox().Dump() ;
                  if(pChild2Aux) pChild2Aux->GetPolarBox().Dump() ;

                  pNode->DumpFamily() ;        
                                             
                  SmPoint3d  sCenter  = pNode->m_sBBox.GetMid() ;
                  smgfx_SetRotationCenter(sCenter) ;
                  
                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pCurve) pCurve->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 0,1,0) ; pNode->m_sBBox.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 0,0,1) ; rPolarBox.Draw(sCenter) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(4,6, 1,0,1) ; if(pParentAux) pParentAux->GetPolarBox().Draw(sCenter) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 0,1,1) ; if(pChild1Aux) pChild1Aux->GetPolarBox().Draw(sCenter) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 0,1,1) ; if(pChild2Aux) pChild2Aux->GetPolarBox().Draw(sCenter) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;

                }
#endif // SM_DEBUG_CODE

// GWCTreeVertexTemp
//                /* 15 */ // Tree LeafNode has bad StartIndex Value
//                bRtn &= SM_ASSERT_BOOLEAN_REPORT(15, SM_LEVEL_0,    (pChild1 != NULL && pChild2 != NULL)
//                                                                 || (   pAux->m_lStartTreeVertexIndx != SM_BIG_ULONG 
//                                                                     && pAux->m_lStartTreeVertexIndx != SM_UNDEF_ULONG
//                                                                     && pAux->m_lStartTreeVertexIndx < m_sTreeVertices.GetSize()), 
//                                                               _T("")) ;

            } // end Node has an SmBezierAux2d m_pData Object from a surface decomposition existence check

         // check the PseudoBox within the SmBezierPatch data type
          if(   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
            {
              SmBezierPatch     * pBezier       = (SmBezierPatch *)pNode->m_pData ;
              SmBezierPatch     * pParentBezier =   (   pParent 
                                                     && pParent->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
                                                  ? (SmBezierPatch *)pParent->m_pData : NULL ;
              SmBezierPatch     * pChild1Bezier =   (   pChild1 
                                                     && pChild1->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
                                                  ? (SmBezierPatch *)pChild1->m_pData : NULL ;
              SmBezierPatch     * pChild2Bezier =   (   pChild2 
                                                     && pChild2->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
                                                  ? (SmBezierPatch *)pChild2->m_pData : NULL ;
              const SmExtent2d  & rUVDomain     = pBezier->GetUVDomain() ;
              const SmPseudoBox & rPseudoBox    = pBezier->GetPseudoBox() ;

              // check that UVDomains nest
              bRtn &= SM_ASSERT_VALUE_REPORT(7, SM_LEVEL_0, (pParentBezier == NULL || rUVDomain.IsContainedBy(pParentBezier->GetUVDomain(), SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(8, SM_LEVEL_0, (pChild1Bezier == NULL || pChild1Bezier->GetUVDomain().IsContainedBy(rUVDomain, SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(9, SM_LEVEL_0, (pChild2Bezier == NULL || pChild2Bezier->GetUVDomain().IsContainedBy(rUVDomain, SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;


              // test the Pseudo box - non negative volume
              bRtn &= rPseudoBox.AssertValid(pAList) ;

              // all bounding boxes of nonDegenerate shapes should be non-zero
              bRtn &= SM_ASSERT_VALUE_REPORT(10, SM_LEVEL_0, (bIsDegenerate == TRUE || rPseudoBox.GetMaxDimension() > SM_EFF_ZERO), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;


              // pseudo boxes don't have to nest because they each have their own coordinate system
              // but as a rule child pseudo boxes should be smaller or equal than parent boxes
              double dPseudoThisVolume   = rPseudoBox.GetVolume() ;
              double dPseudoParentVolume = pParentBezier ? pParentBezier->GetPseudoBox().GetVolume() : 2.0 * dPseudoThisVolume ;
              double dPseudo1Volume      = pChild1Bezier ? pChild1Bezier->GetPseudoBox().GetVolume() : 0.0 ;
              double dPseudo2Volume      = pChild2Bezier ? pChild2Bezier->GetPseudoBox().GetVolume() : 0.0 ;

              bRtn &= SM_ASSERT_VALUE_REPORT(11, SM_LEVEL_0, (dPseudoParentVolume >= dPseudoThisVolume - SM_EFF_ZERO), SM_EFF_ZERO, dPseudoParentVolume - dPseudoThisVolume, _T("")) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(12, SM_LEVEL_0, (dPseudo1Volume      <= dPseudoThisVolume + SM_EFF_ZERO), SM_EFF_ZERO, dPseudo1Volume      - dPseudoThisVolume, _T("")) ;
              bRtn &= SM_ASSERT_VALUE_REPORT(13, SM_LEVEL_0, (dPseudo2Volume      <= dPseudoThisVolume + SM_EFF_ZERO), SM_EFF_ZERO, dPseudo2Volume      - dPseudoThisVolume, _T("")) ;

              // gwc: removed nested test - pseudo boxes do not nest in a contained fashion as
              //        do nested bounding boxes.
              //      // when the node has a parent - its PsuedoBox should be contained
              //      bRtn &= (   pParent == NULL
              //               || pParentBezier->GetPseudoBox().IsContainedBy(rPseudoBox)) ;
              //      
              //      // when the node is a parent - its UVDomain and Pseudo BBox should contain the children's
              //      bRtn &= (   pChild1Bezier == NULL
              //               || pChild1Bezier->GetPseudoBox().IsContainedBy(rPseudoBox)) ;
              //      
              //      bRtn &= (   pChild2Bezier == NULL
              //               || pChild2Bezier->GetPseudoBox().IsContainedBy(rPseudoBox)) ;

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pCurve) pCurve->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,2, 0,0,0) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 0,1,0) ; pNode->m_sBBox.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 0,0,1) ; rPseudoBox.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(4,6, 1,0,1) ; if(pParentBezier) pParentBezier->GetPseudoBox().Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 0,1,1) ; if(pChild1Bezier) pChild1Bezier->GetPseudoBox().Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(6,8, 0,1,1) ; if(pChild2Bezier) pChild2Bezier->GetPseudoBox().Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;

                }
#endif // SM_DEBUG_CODE
            } // end need to validate polar box check

          // a place to put in a debug break
          if(bRtn == FALSE)
            {
              SM_ASSERT(bRtn == TRUE) ;
            }
        
        } // end iter every TreeNode 

    } // end m_pOwner existence check

  // all done
  return bRtn ; 

} // end SmTree::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmTree::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmObject::AssertHeal(rAReport, pAList) ) ;
//     }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmTree::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmTree::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTree::IsKindOf( SM_TYPE t ) const
{
  return ((SmTree_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmTree::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\nBegin Dump of SmTree = 0x%p "),this);
  smos_sprintf(sBuffForFile,_T("\nBegin Dump of SmTree = %s "), _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // TreeNode Count
  if(m_sNodeMgr.GetNumActiveElements() > 0)
    {
      smos_sprintf(sBuff,_T("\n   Tree Node Count       = %ld "),m_sNodeMgr.GetNumActiveElements());
      smos_WriteBuffer(sBuff) ;
    }

  // TreeVertex Count
  if(m_sTreeVertices.GetSize() > 0)
    {
      smos_sprintf(sBuff,_T("\n   Tree Vertex Count     = %ld "),m_sTreeVertices.GetSize());
      smos_WriteBuffer(sBuff) ;
    }

  // ListNode Count
  if(m_sListMgr.GetNumActiveElements() > 0)
    {
      smos_sprintf(sBuff,_T("\n   List Node Count       = %ld "),m_sListMgr.GetNumActiveElements());
      smos_WriteBuffer(sBuff) ;
    }

  // m_pOwner
  smos_sprintf(sBuff,_T("\n   m_pOwner              = 0x%p "),m_pOwner);
  smos_sprintf(sBuffForFile,_T("\n   m_pOwner = %s "),m_pOwner ? _T("notNULL") : _T("NULL"));     smos_WriteBuffer(sBuff, sBuffForFile);
  
  smos_sprintf(sBuff,_T("%s"),_T("     Only set by SmCurveCache and SmSurfaceCache - "));                  smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T("in which case the value is the cache being created"));                   smos_WriteBuffer(sBuff) ;
  
  // MaxElementsPerNode  
  smos_sprintf(sBuff,_T("\n   m_lMaxElementsPerNode = %ld "),m_lMaxElementsPerNode);              smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T("     max elems per node before split - "));                              smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T("for object spatial trees only"));                                        smos_WriteBuffer(sBuff) ;

  // MinSizeRatio
  smos_sprintf(sBuff,_T("\n   m_dMinSizeRatio       = %lf "),m_dMinSizeRatio);                    smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T("     min node size, i.e. 100 = 1/100 of orig size - "));                 smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T("prevents a node from being split - object spatial trees only"));         smos_WriteBuffer(sBuff) ;

  // NodeReduction
  smos_sprintf(sBuff,_T("\n   m_dNodeReduction      = %lf "),m_dNodeReduction);                   smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T("     amount of overlap in children; .5=equal node spacing, "));          smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,_T("%s"),_T(".45=slight overlap - object spatial trees only"));                       smos_WriteBuffer(sBuff) ;

  // display the node tree
  smos_WriteBuffer(_T("\nTree Node Hierarchy:")) ;
  if(m_pTopNode) m_pTopNode->Dump(0) ;

  // display the vertex tree connection table
    {
      //header
      smos_WriteBuffer(_T("\nTree Vertex Connections")) ;
      if(m_sTreeVertices.GetSize() > 0) { smos_WriteBuffer(_T("\n Vertex | Below Right Above Left: UV Position")) ;
                                        }
      else                              { smos_WriteBuffer(_T(" - NONE")) ; }

      // TreeVertex members
      for(ULONG ii=0;ii<m_sTreeVertices.GetSize();ii++)
        {
          // member label
          smos_sprintf(sBuff,_T("\n  %5lu | "), ii) ; smos_WriteBuffer(sBuff) ;

          // member data
          SmTreeVertex sTreeVertex = m_sTreeVertices[ii] ;
          sTreeVertex.Dump() ;
        }
    } // end VertexTree

  smos_sprintf(sBuff,_T("\nEnd Dump of SmTree = 0x%p "),this);
  smos_sprintf(sBuffForFile,_T("\nEnd Dump of SmTree = %s "),_T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmTree::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTree::DumpTreeVertices(void) const
//  {
//    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
//  
//    // TreeVertices Dump header
//    smos_sprintf(sBuff,_T("\nDump of SmTree = 0x%p TreeVertices"),this);
//    smos_sprintf(sBuffForFile,_T("\nDump of SmTree = %s TreeVertices"),this ? _T("notNULL") : _T("NULL"));
//    smos_WriteBuffer(sBuff, sBuffForFile);
//  
//    // TreeVertex Count
//    smos_sprintf(sBuff,_T("\n Tree Vertex Connections: TreeVertex Cnt = %ld"),m_sTreeVertices.GetSize()) ;
//    smos_WriteBuffer(sBuff) ;
//  
//    // TreeVertex table header
//    if(m_sTreeVertices.GetSize() > 0) { smos_WriteBuffer(_T("\n Vertex | Below Right Above Left: UV Position - Nodes:BelowRight AboveRight AboveLeft BelowLeft ")) ; }
//    else                              { smos_WriteBuffer(_T(" - NONE")) ; }
//  
//    // TreeVertex table elements
//    for(ULONG ii=0;ii<m_sTreeVertices.GetSize();ii++)
//      {
//        smos_sprintf(sBuff,_T("\n  %5d | "), ii) ;
//        smos_WriteBuffer(sBuff) ;
//        SmTreeVertex sTreeVertex = m_sTreeVertices[ii] ;
//        sTreeVertex.Dump() ;
//      }
//  
//  } // end SmTree::DumpTreeVertices

/*******************************************************************//**
PURPOSE: find and return a shape pointer to be used by this node.

NOTES: 
  right now its possible to build a surface node without its own shape definition.
  Under such circumstances return the first ancestor with a shape definition.

  In the future: we may choose to enforce an every node has a surface rule
  in which case the need for this function will go away.
***********************************************************************/
gw_SURFACE *SmTreeNode::GetFirstBezierPatch() const
{
  // check the AuxData Object for type SmBezier Patch
  //  loaded with a temporary Bezier Surface Patch
  if(   m_eAuxDataType == SM_AD_BEZIER_SURFACE
     && ((SmBezierPatch *)m_pData)->GetBezierPtr()) 

    { 
      return( ((SmBezierPatch *)m_pData)->GetBezierPtr() ) ; 
    }

  // check AuxData object for type SmBezierAux2d
  if(   m_eAuxDataType == SM_AD_BEZIER_SURFACE
     || m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD
     || m_eAuxDataType == SM_AD_AUX_DATA)     
    { 
      SmSurface *pOwnerSurface = (SmSurface *)GetOwnerObject() ;
      SM_ASSERT(   pOwnerSurface == NULL
                || pOwnerSurface->IsKindOf(SmSurface_TYPE)) ;

      // if the AuxData has a temporary Subdivision Surface pointer - use it
      //  mBA_pSurface will be in one of 3 states
      //    1. NULL
      //    2. Loaded with a temporary Subdivision Surface that should be used
      //    3. Loaded with the OwnerSurface - which should not be used
      if(   ((SmBezierAux2d *)m_pData)->mBA_pSurface
         && (   m_pParent == NULL
             || pOwnerSurface != ((SmBezierAux2d *)m_pData)->mBA_pSurface))
        {
#ifdef SM_DEBUG_CODE         
          if(!((SmBezierAux2d *)m_pData)->mBA_pSurface->IsKindOf(SmBSplineSurface_TYPE))
            {
              SM_ASSERT(((SmBezierAux2d *)m_pData)->mBA_pSurface->IsKindOf(SmBSplineSurface_TYPE)) ;
            }
#endif // SM_DEBUG_CODE
          return( ((SmBSplineSurface *)((SmBezierAux2d *)m_pData)->mBA_pSurface)->GetGwNurbPointer() ) ; 
        }

      // else try to get the parent's shape - skip case 3.
      else if(   m_pParent
              && pOwnerSurface != ((SmBezierAux2d *)m_pData)->mBA_pSurface) 
        { 
          return( m_pParent->GetFirstBezierPatch() ) ; 
        }

      // no shape available for this node
      return(NULL) ;
    }

  // no surface shape for this node - not a surface type
  return(NULL) ;

} // end SmTreeNode::GetFirstBezierPatch

/*******************************************************************//**
PURPOSE: Return the desired leaf node found starting a traversal from this ancestor

NOTES: For Surface subdivisino TreeNodes - return UpperLeft Most descendant
       For all other cases               - return child1 most descendant
***********************************************************************/
SmTreeNode * SmTreeNode::GetUpperLeftMostDescendant() const            
{
  // Arrived at a leaf node - return result
  if(m_pChild1 == NULL) return (SmTreeNode *)this ;

  // locals
  SmBezierAux2d *pAux = GetBezierAux2d() ;
                                    
  // for surface subdivision cases (pAux is defined) - descend based on split direction
  if(pAux) 
    { return(pAux->m_eSplitDir==SM_SP_U ? m_pChild1 : m_pChild2) ; }

  // else descend through m_pChild1
  return(m_pChild1) ;
                                      
} // end SmTreeNode::GetUpperLeftMostDescendant

/*******************************************************************//**
PURPOSE: return SmCurveCache, SmSurfaceCache, SmBrepCache, or SmTessSrfCache 
            when Tree is built for a cache, else return NULL

NOTES: 
***********************************************************************/
const SmObject * SmTreeNode::GetOwner() const         
{ 
  return m_pTree->GetOwner() ;
   
} // end SmTreeNode::GetOwner

/*******************************************************************//**
PURPOSE: propagate m_bIsMarked and m_eNodeClass changes to
            this node's ancestors

NOTES: 
***********************************************************************/
void SmTreeNode::PropagateToParents()
{
  // no work - node not a surface subdivision node
  if(   m_eAuxDataType != SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType != SM_AD_AUX_DATA)
    { return ; }

  // no work - no parent
  if(m_pParent == NULL)
    { return ; }

  // locals
  SmBoolean       bChanged = FALSE ;
  SmTreeNode     *pSibling    = (m_pParent->m_pChild1 == this) ? m_pParent->m_pChild2 : m_pParent->m_pChild1 ;
  SmBezierAux2d  *pParentAux  = (SmBezierAux2d *)m_pParent->m_pData ;
  SmBezierAux2d  *pThisAux    = (SmBezierAux2d *)m_pData ;
  SmBezierAux2d  *pSiblingAux = (SmBezierAux2d *)pSibling->m_pData ;

  // propagate mark to parent when both siblings are marked
  if(pThisAux->m_bIsMarked && pSiblingAux->m_bIsMarked) 
    { 
      if(pParentAux->m_bIsMarked == FALSE)
        {
          pParentAux->m_bIsMarked = TRUE ;
          bChanged = TRUE ;
        }
    }

  // propagate m_eNodeClass changes
  if(pThisAux->m_eNodeClass == SM_NC_ON_BOUNDARY)   
    { 
      if(pParentAux->m_eNodeClass != SM_NC_ON_BOUNDARY)
        { 
          pParentAux->m_eNodeClass = SM_NC_ON_BOUNDARY ;
          bChanged = TRUE ;
        }
    }
  else if(   pThisAux->m_eNodeClass    == SM_NC_INSIDE
          && pSiblingAux->m_eNodeClass == SM_NC_INSIDE)
    {
      if(pParentAux->m_eNodeClass != SM_NC_INSIDE)
        {
          pParentAux->m_eNodeClass = SM_NC_INSIDE ;
          pParentAux->m_pFace      = pThisAux->m_pFace ;
          SM_ASSERT(pThisAux->m_pFace == pSiblingAux->m_pFace) ;
          bChanged = TRUE ;
        }
    }
  else if(   pThisAux->m_eNodeClass    == SM_NC_OUTSIDE 
          && pSiblingAux->m_eNodeClass == SM_NC_INSIDE)
    { 
      if(pParentAux->m_eNodeClass != SM_NC_OUTSIDE)
        {
          pParentAux->m_eNodeClass = SM_NC_OUTSIDE ;
          bChanged = TRUE ;
        }
    }

  // move onto grandparents
  if(bChanged) { m_pParent->PropagateToParents() ; }

} // end SmTreeNode::PropagateToParents

/*******************************************************************//**
PURPOSE: propagate this node's->SmBezierAux2d->m_bIsMarked value
         to all its descendants

NOTES: 
***********************************************************************/
void SmTreeNode::PropagateToChildren()
{      

  // no work - node not a curve or surface subdivision node
  if(   this->m_eAuxDataType != SM_AD_BEZIER_SURFACE                     
     && this->m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD 
     && this->m_eAuxDataType != SM_AD_AUX_DATA)
    { return ; }

  // locals
  SmTreeNode * pNode     = NULL ;
  SmBoolean    bIsMarked = ((SmBezierAux2d *)this->m_pData)->m_bIsMarked ;

  // local queue
  SM_PTR_ARRAY(sQueue, SmTreeNode, 64) ;
  if(this->m_pChild1) sQueue.Push(this->m_pChild1) ;
  if(this->m_pChild2) sQueue.Push(this->m_pChild2) ;

  // while nodes are in the queue
  while(sQueue.Pop(pNode) > 0)
    {
      // clear the mark
      SmBezierAux2d * pAux  = (SmBezierAux2d *)(pNode->m_pData) ;
      pAux->m_bIsMarked = bIsMarked ;

      // Put children in the queue
      if(pNode->m_pChild1) sQueue.Push(pNode->m_pChild1) ;
      if(pNode->m_pChild2) sQueue.Push(pNode->m_pChild2) ;
    }

} // end SmTreeNode::PropagateToChildren

/*******************************************************************//**
PURPOSE: Get all offspring of this TreeNode in an unspecified order 

NOTES: 
***********************************************************************/
void SmTreeNode::GetOffspring
 (SmTArray<SmTreeNode *> &rOffspring)  // in : list to contain offspring
  const
{
  // init output
  rOffspring.ReSet() ;

  // local queue
  if(m_pChild1) rOffspring.Add(m_pChild1) ;
  if(m_pChild2) rOffspring.Add(m_pChild2) ;

  for(ULONG ii=0;ii<rOffspring.GetSize();ii++)
    {
      SmTreeNode *pTreeNode = rOffspring[ii] ;

      // place the next generation in the output
      if(pTreeNode->m_pChild1) rOffspring.Add(pTreeNode->m_pChild1) ;
      if(pTreeNode->m_pChild2) rOffspring.Add(pTreeNode->m_pChild2) ;

    } // end while TreeNodes are still spawning children

} // end SmTreeNode::GetOffspring

/*******************************************************************//**
PURPOSE: Get whole family tree 

NOTES: Walk to most distant ancestor of this node and call pAncestor->GetOffSpring
***********************************************************************/
void SmTreeNode::GetFamily
 (SmTArray<SmTreeNode *> &rNodes)  // out: list of this family's nodes
  const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  SmTreeNode *pAncestor = (SmTreeNode *)this ; 

  while(pAncestor->m_pParent != NULL)
    { pAncestor = pAncestor->m_pParent ; }

  // gather Ancestor offspring
  pAncestor->GetOffspring(rNodes) ;

  // Add ancestor 
  rNodes.InsertAt(0, pAncestor) ;

} // end SmTreeNode::GetFamily

/*******************************************************************//**
PURPOSE: Get all immediate left, right, bot, and top leaf-node neighbors 

NOTES: 
***********************************************************************/
void SmTreeNode::GetNeighbors
 (SmTArray<SmTreeNode *> &rNeighbors)  // in : list to which neighobrs are added
  const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // get left, right, bot, and top neighbors
  GetLeftNeighbors (rNeighbors) ;
  GetRightNeighbors(rNeighbors) ;
  GetBotNeighbors  (rNeighbors) ;
  GetTopNeighbors  (rNeighbors) ;

} // end SmTreeNode::GetNeighbors

/*******************************************************************//**
PURPOSE: Get Above left most TreeVertex index into SmTree::m_sTreeVertices 

NOTES: Surface Subdivision Nodes 
             Leaf node indx     = ((SmBezierAux2d *)m_pData)->m_lStartTreeVertexIndx
             interior node indx = inherited from UpperLeft most descendant
       Other Nodes = SM_BIG_ULONG
***********************************************************************/
// GWCTreeVertexTemp
//  ULONG SmTreeNode::GetStartTreeVertexIndx()    
//   const  
//  {
//    // locals
//    SmTreeNode    * pNode = GetUpperLeftMostDescendant() ;
//    SmBezierAux2d * pAux  = GetBezierAux2d() ;
//  
//    // for surface subdivision nodes - return Indx, otherwise return no Indx
//    return(pAux ? pAux->m_lStartTreeVertexIndx : SM_BIG_ULONG) ;
//  
//  } // end SmTreeNode::GetStartTreeVertexIndx

/*******************************************************************//**
PURPOSE: Get TreeVertexLoop index sequence in CCW direction starting with 
         Above Left TreeNode corner circumscribing this TreeNode 

NOTES: 
***********************************************************************/
// GWCTreeVertexTemp
//  void SmTreeNode::GetVertexLoop    
//   (ULONG            indx,          // in : Above Left TreeNode TreeVertex index, SM_BIG_ULONG = use rTreeNode->GetStartTreeVertexIndx() 
//    SmTArray<ULONG> & rVertexLoop)  // out: CCW closed loop of TreeVertex indices starting with Above Left corner indx
//   const  
//  { 
//    // Pass the call along
//    m_pTree->GetTreeVertexLoop(indx, *this, rVertexLoop) ;
//  
//  } // end SmTreeNode::GetVertexLoop

/*******************************************************************//**
NOTES: Neighbor lSplitBits and lSplitCount arguments.
  The recursive functions:  
                           
      GetLeftNeighbors()   GetRightSide()                     
      GetRightNeighbors()  GetLeftSide()                      
      GetBotNeighbors()    GetBotSide()                                       
      GetTopNeighbors()    GetTopSide()                      
                          
  All use a bitarray and a count to associate a descendant node
  with an ancestor node to figure out the relative positions
  of each.  These functions only track splits in
  one direction or the other at a time, so the use of the bitArray and
  the count don't count all levels between the ancestor and the
  descendant - just those in the direction of interest.
  
  lSplitCount = number of splits in the direction of interest between the
                ancestor and the descendant.
  
  lBitArray   = store 1 bit for each split
                1 = upper child
                0 = lower child                         
                          

   Example: lSplitCount = 3, lSplitBits = 0b101 ; Descendant is 3 splits from the ancestor.  
   
   If we are tracking horizontal splits one would find the position of the descendant 
   by starting at the ancestor node and walking the tree as follows:

   rootNode->Child2_OfFirstHorizontalSplit->Child1_OfNextHorizontalSplit->Child2_OfNextHorizontalSplit

   Because we are only interested in splits in one direction, all the offspring of the
   targeted descendant accessable through vertical splits count as well.

***********************************************************************/


/*******************************************************************//**
PURPOSE: Get all left leaf-node neighbors 

NOTES: 
***********************************************************************/
void SmTreeNode::GetLeftNeighbors
 (SmTArray<SmTreeNode *> &rNeighbors, // in : list to which neighobrs are added
  ULONG lSplitBits,                   // in : track splits to get from current node to target node
  ULONG lSplitCount)                  // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is not a sibling   - done - no neighbors to get (case handled above)
  // this is a left  sibling - get all left neighbors of the parent using current lSplitBits
  // this is a right sibling - get all appropriate right members of the left sibling through a downward tree walk
  // this is a bot   sibling - get all left neighbors of the parent padding lSplitBits to seek lower half neighbors
  // this is a top   sibling - get all left neighbors of the parent padding lSplitBits to seek upper half neighbors
  
  // this is not a sibling - there are no neighbors to gather
  if(m_pParent == NULL) { return ; }

  // locals
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pParent->m_pData))->m_eSplitDir ;
  SmBoolean       bLeft     = (eSplitDir == SM_SP_U && m_pParent->m_pChild1 == this) ;
  SmBoolean       bRight    = (eSplitDir == SM_SP_U && m_pParent->m_pChild2 == this) ;
  SmBoolean       bBot      = (eSplitDir == SM_SP_V && m_pParent->m_pChild1 == this) ;
  SmBoolean       bTop      = (eSplitDir == SM_SP_V && m_pParent->m_pChild2 == this) ;
                            
  // recursion branches
  if     (bLeft ) { m_pParent->GetLeftNeighbors       (rNeighbors, lSplitBits, lSplitCount) ;  }
  else if(bRight) { m_pParent->m_pChild1->GetRightSide(rNeighbors, lSplitBits, lSplitCount) ;  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
                    if(bDebugMe)
                      {
                        m_pParent->Dump() ;
                      }
#endif // SM_DEBUG_CODE
                  }
  else if(bBot  ) { m_pParent->GetLeftNeighbors(rNeighbors, lSplitBits | ( 0 << lSplitCount), lSplitCount+1) ; }
  else if(bTop  ) { m_pParent->GetLeftNeighbors(rNeighbors, lSplitBits | ( 1 << lSplitCount), lSplitCount+1) ; }
  else            { WARN(_T("bad SmTreeNode::GetLeftNeighbors entry - - Node has bad child/parent SplitDir data")) ; }

} // end SmTreeNode::GetLeftNeighbors

/*******************************************************************//**
PURPOSE: Get all right leaf-node neighbors 

NOTES: 
***********************************************************************/
void SmTreeNode::GetRightNeighbors
 (SmTArray<SmTreeNode *> &rNeighbors, // in : list to which neighobrs are added
  ULONG lSplitBits,                   // in : track splits to get from current node to target node
  ULONG lSplitCount)                  // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is not a sibling   - done - no neighbors to get (case handled above)
  // this is a left  sibling - get all appropriate left members of the right sibling through a downward tree walk
  // this is a right sibling - get all right neighbors of the parent using current lSplitBits
  // this is a bot   sibling - get all right neighbors of the parent padding lSplitBits to seek lower half neighbors
  // this is a top   sibling - get all right neighbors of the parent padding lSplitBits to seek upper half neighbors
  
  // this is not a sibling - there are no neighbors to gather
  if(m_pParent == NULL) { return ; }

  // locals
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pParent->m_pData))->m_eSplitDir ;
  SmBoolean       bLeft     = (eSplitDir == SM_SP_U && m_pParent->m_pChild1 == this) ;
  SmBoolean       bRight    = (eSplitDir == SM_SP_U && m_pParent->m_pChild2 == this) ;
  SmBoolean       bBot      = (eSplitDir == SM_SP_V && m_pParent->m_pChild1 == this) ;
  SmBoolean       bTop      = (eSplitDir == SM_SP_V && m_pParent->m_pChild2 == this) ;
                            
  // recursion branches
  if     (bLeft ) { m_pParent->m_pChild2->GetLeftSide(rNeighbors, lSplitBits, lSplitCount) ;  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
                    if(bDebugMe)
                      {
                        m_pParent->Dump() ;
                      }
#endif // SM_DEBUG_CODE
                  }
  else if(bRight) { m_pParent->GetRightNeighbors     (rNeighbors, lSplitBits, lSplitCount) ; }
  else if(bBot  ) { m_pParent->GetRightNeighbors(rNeighbors, lSplitBits | ( 0 << lSplitCount), lSplitCount+1) ; }
  else if(bTop  ) { m_pParent->GetRightNeighbors(rNeighbors, lSplitBits | ( 1 << lSplitCount), lSplitCount+1) ; }
  else            { WARN(_T("bad SmTreeNode::GetRightNeighbors entry - - Node has bad child/parent SplitDir data")) ; }

} // end SmTreeNode::GetRightNeighbors

/*******************************************************************//**
PURPOSE: Get all bot leaf-node neighbors 

NOTES: 
***********************************************************************/
void SmTreeNode::GetBotNeighbors
 (SmTArray<SmTreeNode *> &rNeighbors, // in : list to which neighobrs are added
  ULONG lSplitBits,                   // in : track splits to get from current node to target node
  ULONG lSplitCount)                  // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is not a sibling   - done - no neighbors to get (case handled above)
  // this is a left   sibling - get all bot neighbors of the parent padding lSplitBits to seek lower half neighbors
  // this is a right  sibling - get all bot neighbors of the parent padding lSplitBits to seek upper half neighbors
  // this is a bot    sibling - get all bot neighbors of the parent using current lSplitBits
  // this is a top    sibling - get all appropriate top members of the bot sibling through a downward tree walk
  
  // this is not a sibling - there are no neighbors to gather
  if(m_pParent == NULL) { return ; }

  // locals
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pParent->m_pData))->m_eSplitDir ;
  SmBoolean       bLeft     = (eSplitDir == SM_SP_U && m_pParent->m_pChild1 == this) ;
  SmBoolean       bRight    = (eSplitDir == SM_SP_U && m_pParent->m_pChild2 == this) ;
  SmBoolean       bBot      = (eSplitDir == SM_SP_V && m_pParent->m_pChild1 == this) ;
  SmBoolean       bTop      = (eSplitDir == SM_SP_V && m_pParent->m_pChild2 == this) ;
                            
  // recursion branches
  if     (bLeft ) { m_pParent->GetBotNeighbors(rNeighbors, lSplitBits | ( 0 << lSplitCount), lSplitCount+1) ; }
  else if(bRight) { m_pParent->GetBotNeighbors(rNeighbors, lSplitBits | ( 1 << lSplitCount), lSplitCount+1) ; }
  else if(bBot  ) { m_pParent->GetBotNeighbors      (rNeighbors, lSplitBits, lSplitCount) ; }
  else if(bTop  ) { m_pParent->m_pChild1->GetTopSide(rNeighbors, lSplitBits, lSplitCount) ;  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
                    if(bDebugMe)
                      {
                        m_pParent->Dump() ;
                      }
#endif // SM_DEBUG_CODE
                  }
  else            { WARN(_T("bad SmTreeNode::GetBotNeighbors entry - - Node has bad child/parent SplitDir data")) ; }

} // end SmTreeNode::GetBotNeighbors

/*******************************************************************//**
PURPOSE: Get all top leaf-node neighbors 

NOTES: 
***********************************************************************/
void SmTreeNode::GetTopNeighbors
 (SmTArray<SmTreeNode *> &rNeighbors, // in : list to which neighobrs are added
  ULONG lSplitBits,                   // in : track splits to get from current node to target node
  ULONG lSplitCount)                  // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is not a sibling - done - no neighbors to get (case handled above)
  // this is a left  sibling - get all top neighbors of the parent padding lSplitBits to seek lower half neighbors
  // this is a right sibling - get all top neighbors of the parent padding lSplitBits to seek upper half neighbors
  // this is a bot   sibling - get all appropriate bot members of the top sibling through a downward tree walk
  // this is a top   sibling - get all top neighbors of the parent using current lSplitBits
  
  // this is not a sibling - there are no neighbors to gather
  if(m_pParent == NULL) { return ; }

  // locals
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pParent->m_pData))->m_eSplitDir ;
  SmBoolean       bLeft     = (eSplitDir == SM_SP_U && m_pParent->m_pChild1 == this) ;
  SmBoolean       bRight    = (eSplitDir == SM_SP_U && m_pParent->m_pChild2 == this) ;
  SmBoolean       bBot      = (eSplitDir == SM_SP_V && m_pParent->m_pChild1 == this) ;
  SmBoolean       bTop      = (eSplitDir == SM_SP_V && m_pParent->m_pChild2 == this) ;
                            
  // recursion branches
  if     (bLeft ) { m_pParent->GetTopNeighbors(rNeighbors, lSplitBits | ( 0 << lSplitCount), lSplitCount+1) ; }
  else if(bRight) { m_pParent->GetTopNeighbors(rNeighbors, lSplitBits | ( 1 << lSplitCount), lSplitCount+1) ; }
  else if(bBot  ) { m_pParent->m_pChild2->GetBotSide(rNeighbors, lSplitBits, lSplitCount) ;  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
                    if(bDebugMe)
                      {
                        m_pParent->Dump() ;
                      }
#endif // SM_DEBUG_CODE
                  }
  else if(bTop  ) { m_pParent->GetTopNeighbors      (rNeighbors, lSplitBits, lSplitCount) ; }
  else            { WARN(_T("bad SmTreeNode::GetTopNeighbors entry - - Node has bad child/parent SplitDir data")) ; }

} // end SmTreeNode::GetTopNeighbors

/*******************************************************************//**
PURPOSE: Get all right side descendants that line up
            with the target mask.

NOTES: 
***********************************************************************/
void SmTreeNode::GetRightSide
 (SmTArray<SmTreeNode *> &rNeighbors,  // in : list to which neighobrs are added
  ULONG lTargetSplitBits,              // in : specify desired sibling(s) after multiple horizontal splits
  ULONG lTargetSplitCount,             // in : number of horizontal splits
  ULONG lSplitBits,                    // in : track splits to this nodes from root
  ULONG lSplitCount)                   // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is a leaf node - add this to rNeighbors 
  // this is a vertically split parent   - get right side nodes from right child
  // this is a horizontally split parent - use mask to get all right side nodes from 
  //                                       just top child
  //                                       just bot child
  //                                       both bot and top children

  // local
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pData))->m_eSplitDir ;
  
  // leaf node
  if(m_pChild1 == NULL) 
    { 
      SM_ASSERT(m_pChild2 == NULL) ;
      rNeighbors.Add((SmTreeNode *)this) ;
    }

  // vertical split  -  has right/left children
  else if(eSplitDir == SM_SP_U) 
    { 
      m_pChild2->GetRightSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, lSplitBits, lSplitCount) ; 
    }

  // horizontal split -  has bot/top children
  else if(eSplitDir == SM_SP_V) 
    { 
      // classify the children
      SmBoolean bChild1 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 0) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      SmBoolean bChild2 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 1) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      if(bChild1) m_pChild1->GetRightSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 0, lSplitCount+1) ; 
      if(bChild2) m_pChild2->GetRightSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 1, lSplitCount+1) ; 
    }

  else // error           
    { 
      WARN(_T("bad SmTreeNode::GetRightSide entry - - Node has bad child/parent SplitDir data")) ; 
    }

} // end SmTreeNode::GetRightSide  

/*******************************************************************//**
PURPOSE: Get all left side descendants that line up
            with the target mask.

NOTES: 
***********************************************************************/
void SmTreeNode::GetLeftSide
 (SmTArray<SmTreeNode *> &rNeighbors,  // in : list to which neighobrs are added
  ULONG lTargetSplitBits,              // in : specify desired sibling(s) after multiple horizontal splits
  ULONG lTargetSplitCount,             // in : number of horizontal splits
  ULONG lSplitBits,                    // in : track splits to this nodes from root
  ULONG lSplitCount)                   // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is a leaf node - add this to rNeighbors 
  // this is a vertically split parent   - get left side nodes from left child
  // this is a horizontally split parent - use mask to get all left side nodes from 
  //                                       just top child
  //                                       just bot child
  //                                       both bot and top children

  // local
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pData))->m_eSplitDir ;
  
  // leaf node
  if(m_pChild1 == NULL) 
    { 
      SM_ASSERT(m_pChild2 == NULL) ;
      rNeighbors.Add((SmTreeNode *)this) ;
    }

  // vertical split -  has right/left children
  else if(eSplitDir == SM_SP_U) 
    { 
      m_pChild1->GetLeftSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, lSplitBits, lSplitCount) ; 
    }

  // horizontal split -  has bot/top children
  else if(eSplitDir == SM_SP_V) 
    { 
      // classify the children
      SmBoolean bChild1 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 0) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      SmBoolean bChild2 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 1) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      if(bChild1) m_pChild1->GetLeftSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 0, lSplitCount+1) ; 
      if(bChild2) m_pChild2->GetLeftSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 1, lSplitCount+1) ; 
    }

  else // error           
    { 
      WARN(_T("bad SmTreeNode::GetLeftSide entry - - Node has bad child/parent SplitDir data")) ; 
    }

} // end SmTreeNode::GetLeftSide 
 
/*******************************************************************//**
PURPOSE: Get all top side descendants that line up
            with the target mask.

NOTES: 
***********************************************************************/
void SmTreeNode::GetTopSide
 (SmTArray<SmTreeNode *> &rNeighbors,  // in : list to which neighobrs are added
  ULONG lTargetSplitBits,              // in : specify desired sibling(s) after multiple horizontal splits
  ULONG lTargetSplitCount,             // in : number of horizontal splits
  ULONG lSplitBits,                    // in : track splits to this nodes from root
  ULONG lSplitCount)                   // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;                               

  // method:
  // this is a leaf node - add this to rNeighbors 
  // this is a horizontally split parent - get top side nodes from top child
  // this is a vertically split parent - use mask to get all top side nodes from 
  //                                       just right child
  //                                       just left child
  //                                       lefth left and right children

  // local
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pData))->m_eSplitDir ;
  
  // leaf node
  if(m_pChild1 == NULL) 
    { 
      SM_ASSERT(m_pChild2 == NULL) ;
      rNeighbors.Add((SmTreeNode *)this) ;
    }

  // horizontal split  -  has top/bot children
  else if(eSplitDir == SM_SP_V) 
    { 
      m_pChild2->GetTopSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, lSplitBits, lSplitCount) ; 
    }      

  // vertical split -  has left/right children
  else if(eSplitDir == SM_SP_U) 
    { 
      // classify the children
      SmBoolean bChild1 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 0) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      SmBoolean bChild2 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 1) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      if(bChild1) m_pChild1->GetTopSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 0, lSplitCount+1) ; 
      if(bChild2) m_pChild2->GetTopSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 1, lSplitCount+1) ; 
    }

  else // error           
    { 
      WARN(_T("bad SmTreeNode::GetTopSide entry - - Node has bad child/parent SplitDir data")) ; 
    }

} // end SmTreeNode::GetTopSide  

/*******************************************************************//**
PURPOSE: Get all bot side descendants that line up
            with the target mask.

NOTES: 
***********************************************************************/
void SmTreeNode::GetBotSide
 (SmTArray<SmTreeNode *> &rNeighbors,  // in : list to which neighobrs are added
  ULONG lTargetSplitBits,              // in : specify desired sibling(s) after multiple horizontal splits
  ULONG lTargetSplitCount,             // in : number of horizontal splits
  ULONG lSplitBits,                    // in : track splits to this nodes from root
  ULONG lSplitCount)                   // in : split depth starting at 0, default:[0]
   const
{
  // no work - node is not from a surface subdivision tree
  if(   m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // ensure initial values
  if(lSplitCount == 0) lSplitBits = 0 ;

  // method:
  // this is a leaf node - add this to rNeighbors 
  // this is a horizontally split parent   - get bot side nodes from bot child
  // this is a vertically split parent - use mask to get all bot side nodes from 
  //                                       just right child
  //                                       just left child
  //                                       lefth left and right children

  // local
  SmSurfParamType eSplitDir = ((SmBezierAux2d *)(m_pData))->m_eSplitDir ;
  
  // leaf node
  if(m_pChild1 == NULL) 
    { 
      SM_ASSERT(m_pChild2 == NULL) ;
      rNeighbors.Add((SmTreeNode *)this) ;
    }

  // horizontal split -  has top/bot children
  else if(eSplitDir == SM_SP_V) 
    { 
      m_pChild1->GetBotSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, lSplitBits, lSplitCount) ; 
    }
          
  // vertical split -  has left/right children
  else if(eSplitDir == SM_SP_U) 
    { 
      // classify the children
      SmBoolean bChild1 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 0) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      SmBoolean bChild2 =   ((lSplitCount + 1) > lTargetSplitCount) ? TRUE
                          : (((lSplitBits << 1) | 1) == (lTargetSplitBits >> (lTargetSplitCount - (lSplitCount + 1)))) ;

      if(bChild1) m_pChild1->GetBotSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 0, lSplitCount+1) ; 
      if(bChild2) m_pChild2->GetBotSide(rNeighbors, lTargetSplitBits, lTargetSplitCount, (lSplitBits << 1) | 1, lSplitCount+1) ; 
    }

  else // error           
    { 
      WARN(_T("bad SmTreeNode::GetBotSide entry - - Node has bad child/parent SplitDir data")) ; 
    }

} // end SmTreeNode::GetBotSide  

/*******************************************************************//**
PURPOSE: Display the current bounding box, and for any children recursivley
  ask that they be displayed in an incremented line size and color.

  For each deeper child level, each bounding box is drawn with
    color     += [red_inc, green_inc, blue_inc].
    lineWidth += 0.75

NOTES: 
***********************************************************************/
SmDisplayList * SmTreeNode::DrawBoundingBox
  (double red,     double green,     double blue, 
   double red_inc, double green_inc, double blue_inc,
   double lineWidth,
   const SmContext * pContext,
   SmGfxArraySet   * pOptGfxSet)               // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                               //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE  
     // open or use currently open displayList 
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  // set the color and linewidth
  smgfx_OutputColor(red, green, blue, pOptGfxSet) ;
  smgfx_OutputLineWidth(lineWidth, pOptGfxSet) ;
#endif

  // draw this node's bounding box and midPoint in current color
  Draw(FALSE, pContext, pOptGfxSet) ;

  // for any children
  if(m_pChild1 || m_pChild2)
    {
      // increment the line color
      double color_scale  = smos_Sqrt(red*red + green*green + blue*blue) ;
      red   += red_inc ;  
      green += green_inc ;
      blue  += blue_inc ;
      color_scale /= smos_Sqrt(red*red + green*green + blue*blue) ;
      red   *= color_scale ;
      green *= color_scale ;
      blue  *= color_scale ; 

      if(red  <0.0) red   = 0.0 ; 
      if(red  >1.0) red   = 1.0 ;
      if(green<0.0) green = 0.0 ; 
      if(green>1.0) green = 1.0 ;
      if(blue <0.0) blue  = 0.0 ; 
      if(blue >1.0) blue  = 1.0 ;

      // increment the line width
      lineWidth += 0.75 ;

      // pass the call along to the children
      if(m_pChild1) { m_pChild1->DrawBoundingBox(red,     green,     blue,
                                                 red_inc, green_inc, blue_inc,
                                                 lineWidth, pContext, pOptGfxSet) ;
                    }
      if(m_pChild2) { m_pChild2->DrawBoundingBox(red,     green,     blue,
                                                 red_inc, green_inc, blue_inc,
                                                 lineWidth, pContext, pOptGfxSet) ;
                    }
    } // end child check

#ifdef SM_GFX_OUTPUT_CODE  
     // open or use currently open displayList 
   pRtn = smgfx_Close(pOptGfxSet) ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmTreeNode::DrawBoundingBox

/*******************************************************************//**
PURPOSE: Draw SmTrimSrfCache subdivision tree leafnode bounding boxes.

NOTES:  leafnode bounding boxes are assigned the following colors
  to show in/out/on classification.

  MidPoint Colors
  -------------------
  red   = ON_BOUNDARY
  green = INSIDE
  blue  = OUTSIDE
  black = UNKNOWN

***********************************************************************/
SmDisplayList * SmTreeNode::Draw
  (SmBoolean         bAutoColor,   // in : TRUE = color surf nodes by classification type
                                   //      FALSE= use current color values
   const SmContext * pContext,     // in :
   SmGfxArraySet   * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]

 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // save current color
  SmVector3d sColor = smgfx_GetColor() ;

  // open display list
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  // draw every SmTrimSrfCache leafnode boundary and midPoint
  // midPoint color red   = ON_BOUNDARY
  //                green = INSIDE
  //                blue  = OUTSIDE
  //                black = UNKNOWN
  SmPoint3d sMidPoint ;
  SmPoint2d sPointUV ;

  const SmTreeNode *pNode = this ;
  SmBezierAux2d    *pAux  =   (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                               || pNode->m_eAuxDataType == SM_AD_AUX_DATA
                               || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD)
                            ? (SmBezierAux2d *)m_pData
                            : NULL ;

  // color red   = ON_BOUNDARY
  //       green = INSIDE
  //       blue  = OUTSIDE
  //       black = UNKNOWN
  SmVector3d sLastColor = smgfx_GetColor() ;
  if     (bAutoColor && pAux && pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) { smgfx_SetColor(1,0,0, pOptGfxSet); }
  else if(bAutoColor && pAux && pAux->m_eNodeClass == SM_NC_INSIDE)      { smgfx_SetColor(0,1,0, pOptGfxSet); }
  else if(bAutoColor && pAux && pAux->m_eNodeClass == SM_NC_OUTSIDE)     { smgfx_SetColor(0,0,1, pOptGfxSet); }
  else if(bAutoColor && pAux && pAux->m_eNodeClass == SM_NC_UNKNOWN)     { smgfx_SetColor(0,0,0, pOptGfxSet); }
 
  // bounding box
  pNode->m_sBBox.Draw(pContext, NULL, pOptGfxSet) ;

  // draw node midPoint

  // if Surface subdivision nodes
  if(pAux) 
    {
      // add center point - evaluate through uv_domain because some BBoxes are
      //   just equal to their parent's bbox
      // GWC TODO: Once the child BBox problem is fixed - just
      //           evaluate the BBox center - its a lot cheaper.
      sPointUV  = pAux->m_sUVDomain.Evaluate(.5,.5) ;
      SmSurface *pSurface = (SmSurface *)pNode->GetOwnerObject() ;
      SM_ASSERT(pNode->GetOwnerObject()->IsKindOf(SmSurface_TYPE)) ;

      pSurface->EvaluatePoint(sPointUV, sMidPoint) ;

      sMidPoint.Draw(NULL, pContext, pOptGfxSet) ;
    }
  else // just draw bbox mid point
    {
      pNode->m_sBBox.GetMid().Draw(NULL, pContext, pOptGfxSet) ;
    }

  if(bAutoColor) smgfx_SetColor(sColor, pOptGfxSet);

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bAutoColor, pContext, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmTreeNode::Draw

/*******************************************************************//**
PURPOSE: Debug formatted print 

NOTES: 
***********************************************************************/
void SmTreeNode::Dump
  (int lDepth,         // in : Recursive depth - used to add spacing
                       //      to show tree structures as indented list.
                       //      default:[0]
   SmBoolean bRecurse, // in : TRUE=recurse to children, FALSE=don't
   int *lParentCount,  // i/o: accumulative count of parent nodes, NULL to ignore
   int *lLeafCount)    // i/o: accumulative count of leaf nodes, NULL to ignore
   const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[SM_TBLOCK_SIZE];
  TCHAR sAuxType[9][128]   = { _T("SM_AD_NONE"),
                               _T("SM_AD_BEZIER_CURVE"),
                               _T("SM_AD_CURVE_DATA"),
                               _T("SM_AD_BEZIER_SURFACE = Decomposition node with neighbor connectivity data"),
                               _T("SM_AD_BEZIER_SURFACE_HEAD = Organizing ancester of top level SM_AD_BEZIER_SURFACE siblings"),
                               _T("SM_AD_SURFACE_DATA"),
                               _T("SM_AD_AUX_DATA = UV refined descendant of SM_AD_BEZIER_SURFACE for trim curve tracking"),
                               _T("SM_AD_CUBEAUX_DATA"),
                               _T("SM_AD_OBJECT_LIST") };
  TCHAR sGeomType[4][32]  = { _T("SM_NG_NONE"),
                             _T("SM_NG_DEFAULT"),
                             _T("SM_NG_LINE_SEG"),
                             _T("SM_NG_POLYGON") };
  TCHAR sSplitType[3][32] = { _T("splitDir = SM_SP_U"),
                             _T("splitDir = SM_SP_V"),
                             _T("splitDir = SM_SP_UNKNONWN") };
  TCHAR sContType[14][32]  = { _T("SM_NOT_USED"),
                              _T("SM_CT_DISCONTINUOUS"),
                              _T("SM_CT_C0"),
                              _T("SM_CT_G1"),
                              _T("SM_CT_G1R"),
                              _T("SM_CT_G1_G2"),
                              _T("SM_CT_G1_G2_G3"),
                              _T("SM_CT_C1"),
                              _T("SM_CT_C1_G2"),
                              _T("SM_CT_C1_G2_G3"),
                              _T("SM_CT_C1_C2"),
                              _T("SM_CT_C1_C2_G3"),
                              _T("SM_CT_C1_C2_C3"),
                              _T("SM_CT_CINFINITY") };
  TCHAR sNodeClassType[4][32] = { _T("SM_NC_UNKNOWN"),
                                 _T("SM_NC_INSIDE"),
                                 _T("SM_NC_OUTSIDE"),
                                 _T("SM_NC_ON_BOUNDARY") };

  // set indent string equal to depth of recursive call
  sIndent[0] = sIndent[1] = sIndent[2] = sIndent[3] = ' ' ;
  int i, i2;
  for(i=0,i2=4;i<=lDepth;i++,i2+=2)
    {
      sIndent[i2  ] = ' ' ;
      sIndent[i2+1] = ' ' ;                                       
    }                                              
  sIndent[i2] = '\0' ;

  // document this node
  if(m_pChild1 || m_pChild2) // parent
    { 
      SmBezierAux2d *pAux =   (   m_eAuxDataType == SM_AD_BEZIER_SURFACE     
                               || m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD
                               || m_eAuxDataType == SM_AD_AUX_DATA)
                            ? (SmBezierAux2d *)m_pData 
                            : NULL ;
      smos_sprintf(sBuff,_T("\n%s %d Parent Node AuxDataType = %s, GeomType = %s, %s"), 
                          sIndent, 
                          lDepth, 
                          sAuxType[m_eAuxDataType], 
                          sGeomType[m_eGeomType], 
                          pAux ? sSplitType[pAux->m_eSplitDir] : _T("")) ;
      smos_WriteBuffer(sBuff) ;
    }
  else // leaf node - has no children
    { smos_sprintf(sBuff,_T("\n%s %d Leaf Node AuxDataType = %s, GeomType = %s"), 
                 sIndent, lDepth, sAuxType[m_eAuxDataType], sGeomType[m_eGeomType]) ;
      smos_WriteBuffer(sBuff) ;
    }

  // domain space bounding box when applicable
  switch(m_eAuxDataType)
    {
      case SM_AD_BEZIER_CURVE        :
      case SM_AD_CURVE_DATA          : { SmBezierSpan *pBezSpan = (SmBezierSpan *)m_pData ;
                                         SmPoint3d sStartPt, sEndPt ;
                                         SmExtent1d sIvl ;
                                         pBezSpan->GetEnds((const SmCurveCache &)*GetOwner(), sStartPt, sEndPt, sIvl) ;
                                         smos_sprintf(sBuff,_T("\n%s         Interval= [%16.16lf, %16.16lf], LeftMost = [%ld, %s], RightMost = [%s, %ld]"), 
                                                    sIndent,               
                                                    sIvl.GetMin(),
                                                    sIvl.GetMax(), 
                                                    pBezSpan->m_lLeft, 
                                                    sContType[pBezSpan->m_aeConts[0]],
                                                    sContType[pBezSpan->m_aeConts[1]],
                                                    pBezSpan->m_lRight) ;
                                         smos_WriteBuffer(sBuff) ;
                                       } break ;

      case SM_AD_BEZIER_SURFACE      :
      case SM_AD_BEZIER_SURFACE_HEAD :
      case SM_AD_AUX_DATA            : { SmBezierAux2d *pAux = (SmBezierAux2d *)m_pData ;
                                         const SmExtent2d &rUVDomain = pAux->GetUVDomain() ;
                                         smos_sprintf(sBuff,
                                                    _T("\n%s         in/out = [%s] ,"),
                                                    sIndent,
                                                    sNodeClassType[pAux->m_eNodeClass]) ; 
                                         smos_WriteBuffer(sBuff) ;
                                         rUVDomain.Dump() ;   // ends with new line

                                         if(m_eAuxDataType == SM_AD_BEZIER_SURFACE)
                                           {
                                             SmBezierPatch *pBez = (SmBezierPatch *)m_pData ;
                                             smos_sprintf(sBuff,
                                                        _T("%s         cont: left = [%s], right = [%s], bot = [%s], top = [%s], "),
                                                        sIndent,
                                                        sContType[pBez->m_eUCurveConts[0]],
                                                        sContType[pBez->m_eUCurveConts[1]],
                                                        sContType[pBez->m_eVCurveConts[0]],
                                                        sContType[pBez->m_eVCurveConts[1]]) ;
                                             smos_WriteBuffer(sBuff) ;
                                            }
                                          else
                                            { smos_sprintf(sBuff, _T("%s         "), sIndent) ;
                                              smos_WriteBuffer(sBuff) ;
                                            }
                                         smos_sprintf(sBuff,        _T("mBA_pSurface = 0x%p"), pAux->mBA_pSurface) ;
                                         smos_sprintf(sBuffForFile, _T("mBA_pSurface = %s"), pAux->mBA_pSurface ? _T("NotNULL") : _T("NULL")) ;
                                         smos_WriteBuffer(sBuff,sBuffForFile) ;
                                       } break ;
                                        
      case SM_AD_SURFACE_DATA        :
      case SM_AD_CUBEAUX_DATA        : break ;
      case SM_AD_OBJECT_LIST         : { SmTArray<SmObject *> sObjectList ;
                                         SmTArray<SmExtent3d> sBBoxList ;
                                         ((SmObjectList*)(*this).m_pData)->GetObjectList(sObjectList, &sBBoxList) ;
                                         smos_sprintf(sBuff,_T("\n%s  ObjCnt[%ld]"), sIndent, sObjectList.GetSize()) ;
                                         smos_WriteBuffer(sBuff) ;
                                         ULONG ii ;
                                         for(ii=0;ii<sObjectList.GetSize();ii++)
                                           { // object identifier
                                             smos_sprintf(sBuff,_T("\n%s    Obj[%ld] = 0x%p[%s], BBox: "), 
                                                        sIndent, ii,
                                                        sObjectList[ii],
                                                        sObjectList[ii] ? sObjectList[ii]->GetTypeString() : _T("NULL") ) ;
                                             smos_sprintf(sBuffForFile,_T("\n%s    Obj[%ld] = %s[%s], BBox: "), 
                                                        sIndent, ii,
                                                        sObjectList[ii] ? _T("NotNULL") : _T("NULL") ,
                                                        sObjectList[ii] ? sObjectList[ii]->GetTypeString() : _T("NULL") ) ;
                                             smos_WriteBuffer(sBuff,sBuffForFile) ;

                                             // and the bounding box
                                             SmExtent3d sBBox = sBBoxList[ii] ;
                                             SmPoint3d  sMin  = sBBox.GetMin() ;
                                             SmPoint3d  sMax  = sBBox.GetMax() ;
                                             SmVector3d sLen  = sMax - sMin ;
                                             smos_sprintf(sBuff,_T("[%lf, %lf, %lf] [%lf, %lf, %lf], ln:[%lf, %lf, %lf]"),
                                                        sMin.x,sMin.y,sMin.z,
                                                        sMax.x,sMax.y,sMax.z,
                                                        sLen.x,sLen.y,sLen.z) ;
                                             smos_WriteBuffer(sBuff);
                                           }
                                       } // end case SM_AD_OBJECT_LIST
                                       break ; 
      case SM_AD_NONE:
          break;
    } // end switch(m_eAuxDataType)
                                         
  // image space bounding box
  smos_sprintf(sBuff,_T("\n%s         BBoxMin = [%16.16lf %16.16lf %16.16lf]"), 
             sIndent, m_sBBox.GetMin().x, m_sBBox.GetMin().y, m_sBBox.GetMin().z) ;
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff,_T("\n%s         BBoxMax = [%16.16lf %16.16lf %16.16lf]"), 
             sIndent, m_sBBox.GetMax().x, m_sBBox.GetMax().y, m_sBBox.GetMax().z) ;
  smos_WriteBuffer(sBuff) ;

  // recurse to the children
  if(bRecurse)
    {
      if(m_pChild1) { m_pChild1->Dump(lDepth+1,bRecurse,lParentCount,lLeafCount) ; }
      if(m_pChild2) { m_pChild2->Dump(lDepth+1,bRecurse,lParentCount,lLeafCount) ; 
                      if(lParentCount) *lParentCount += 1 ; 
                    }
      else          { if(lLeafCount) *lLeafCount += 1 ; 
                    }
    }

  // output node counts
  if(   lDepth == 0
     && lParentCount
     && lLeafCount)
    {
      smos_sprintf(sBuff,_T("\n  # internal Nodes = %d, # Leaf Nodes = %d"), 
                 *lParentCount, *lLeafCount) ;
      smos_WriteBuffer(sBuff) ;
    }
} // end SmTreeNode::Dump

/*******************************************************************//**
PURPOSE: Debug formatted print of this node and its immediate family members

NOTES: 
***********************************************************************/
void SmTreeNode::DumpFamily
  ()    
 const
{
  int lDepth = 0 ;

  if(m_pParent) { m_pParent->Dump(lDepth, FALSE) ; lDepth++ ; }
  this->Dump(1, FALSE) ; lDepth++ ;
  if(m_pChild1) { m_pChild1->Dump(2, FALSE) ; }
  if(m_pChild2) { m_pChild2->Dump(2, FALSE) ; }

} // end SmTreeNode::DumpFamily

/*******************************************************************//**
PURPOSE: Debug formatted print 

NOTES: writes one line report with no line feeds:
   [indx0 indx1 indx2 indx3] Pos: [UV.x,UV.y]
***********************************************************************/
void SmTreeVertex::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBelow[16], sRight[16], sAbove[16], sLeft[16] ;

  // below
  if(m_lNextIndex[0] != SM_BIG_ULONG) { smos_snprintf(sBelow, 16, _T("%5lu"), m_lNextIndex[0]) ; }
  else                                { smos_snprintf(sBelow, 16, _T("%s"),_T("    -")) ; }

  // right
  if(m_lNextIndex[1] != SM_BIG_ULONG) { smos_snprintf(sRight, 16, _T("%5lu"), m_lNextIndex[1]) ; }
  else                                { smos_snprintf(sRight, 16, _T("%s"),_T("    -")) ; }

  // above
  if(m_lNextIndex[2] != SM_BIG_ULONG) { smos_snprintf(sAbove, 16, _T("%5lu"), m_lNextIndex[2]) ; }
  else                                { smos_snprintf(sAbove, 16, _T("%s"),_T("    -")) ; }

  // left
  if(m_lNextIndex[3] != SM_BIG_ULONG) { smos_snprintf(sLeft, 16, _T("%5lu"), m_lNextIndex[3]) ; }
  else                                { smos_snprintf(sLeft, 16, _T("%s"),_T("    -")) ; }

  // indices and UV
  smos_sprintf(sBuff,_T("[%s %s %s %s] Pos: "),
             sBelow, sRight, sAbove, sLeft) ;
  smos_WriteBuffer(sBuff) ;
  m_vUVPoint.Dump() ;

  // Nodes
  smos_sprintf(sBuff,_T(" Nodes:[BR:0x%p AR:0x%p AL:0x%p BL:0x%p] "),
             m_pTreeNode[0], m_pTreeNode[1], m_pTreeNode[2], m_pTreeNode[3]) ;

  smos_WriteBuffer(sBuff) ;

} // end SmTreeVertex::Dump
