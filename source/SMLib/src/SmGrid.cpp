// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGrid.cpp
* PURPOSE: Source file for SmGrid methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmGrid.h>
#include <SmGraphicsExtern.h>  

#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmSurfaceCache.h>

#ifdef SM_DEBUG_CODE
#include <SmFace.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Expand the surface hierarchy and put sub-elements of the surface
            into the actual grid elements.

NOTES:
  1. Call BuildTree() on the surfaceCache
  2. For Every leaf node
     a. Make a SmGridElement containing a pointer to the leaf-node and
        a summary of the leaf-node's bounding box
     b. For every GridVoxel that intersects the LeafNode's bounding box,
         add a pointer to the SmGridElement to the Voxel->m_pElements list 
     //      c. Evaluate and store leaf-nodes midPoint position, tangent, and
     //         surface normal data in the SurfaceCache's SmBezierPatch data
     //         stored in the pNode->m_pData slot.
     //      d. When compiled with FIX_ME and USING_NLIB convert leaf-node's
     //         representation from bezier to power basis. (don't know why - yet)

***********************************************************************/
SmStatus SmSurfaceHierarchyElement::ExpandHierarchy
  (SmGrid * pGrid)
{
  // build or fetch surface cache from global cache queue
  // gwc note: for efficiency, SmRayTracer::AddSurfaceToGrid() 
  //           makes a ExpandHierarchy() call immediately after
  //           building the SurfaceCache so the next call should
  //           only fetch the cache not build it. 
  SmSurfaceCache *pSurfaceCache = smsurf_GetSurfaceCache(m_pSurface) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
      SmFace *pFace = (SmFace *)m_pSurface->GetFace() ;
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; pGrid->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; m_pSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,0) ; pSurfaceCache->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // For every Surface subdivision tree node
  SmTArray<SmTreeNode*> sAllTreeNodes;
  SmTree *pTree = pSurfaceCache->GetTree();
  pTree->GetAllTreeNodes(sAllTreeNodes);
  for (ULONG i=0; i<sAllTreeNodes.GetSize(); i++) 
    {
      SmTreeNode *pNode = sAllTreeNodes[i];

      // skip non-leaf nodes
      // gwc state: currently set up to add the lowest node in the hierarchy that
      //            is of type SM_AD_BEZIER_SURFACE and either has no children
      //            or has children not of type SM_AD_BEZIER_SURFACE.
      //            Note, this does not have to be leaf nodes - sometimes
      //            its mid level nodes.
      // gwc query: is this the way we want it? where do SM_AD_AUX_DATA
      //            children get added to parents?  Why does this bit
      //            of code know about that?
      if(   (pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE) 
         || (   pNode->m_pChild1 != NULL
             && pNode->m_pChild1->m_eAuxDataType == SM_AD_BEZIER_SURFACE)) 
        { continue; }

      // skip leaf-nodes classified outside of associated trim boundaries
      SmBezierPatch *pBezPatch = (SmBezierPatch*)pNode->m_pData; 
      NER(pBezPatch);
      if (pBezPatch->m_eNodeClass == SM_NC_OUTSIDE) 
        { 
#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              SmExtent3d sBBox = pNode->m_sBBox;
              // smgfx_ChangeColor(i != 0); sBBox.Draw(NULL); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE
          continue; 
        }

      // expand leaf-node bounding box by tolerance
      SmExtent3d sBBox = pNode->m_sBBox;
      sBBox.ExpandAbsolute(m_dExpansionTolerance);

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          smgfx_ChangeColor(i != 0); sBBox.Draw(NULL); sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // make an SmGridElement object containing a pointer to pNode 
      // Then for every voxel that intersects the pNode's bounding box,        
      //    add a pointer to the SmGridElement to the voxel's m_pElements list.
      pGrid->AddElement(pNode,
                        SM_GE_SURFACE,
                        sBBox);

    } // end iter every Surface subdivision Tree node

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; pGrid->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; m_pSurface->DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmSurfaceHierarchyElement::ExpandHierarchy

/*******************************************************************//**
PURPOSE: Reproduce pre SHORTRAYTRACER behavior of returning a void*
            pointer for all objects stored in the grid. 

NOTES: This is not exactly pre SHORTRAYTRACER behavior because
  the SmSurfaceGridElement now stores a pointer to a SmSurface object
  rather than a pointer to a SmSurfaceCache->SmTreeNode object.

  The SmUserGridElement      still stores a void * pointer,
  the SmPolygonGridElement   still stores a SmTreeNode * pointer, and
  the SmHierarchyGridElement still stores a SmHierarchyElement * pointer.
***********************************************************************/
void * SmGridElement::GetElement() const // rtn: SmSurface *          for SmSurfaceGridElement
                                         //      SmTreeNode *         for SmPolygonGridElement
                                         //      SmHierarchyElement * for SmHierarchyElement
                                         //      void *               for SmUserGridElement
{ 
  // Return derived type 
  switch(m_lType)
    {
      case SmSurfaceGridElement_TYPE   : return( ((SmSurfaceGridElement   *)this)->GetElement() ) ; 
      case SmPolygonGridElement_TYPE   : return( ((SmPolygonGridElement   *)this)->GetElement() ) ; 
      case SmHierarchyGridElement_TYPE : return( ((SmHierarchyGridElement *)this)->GetElement() ) ; 
      case SmUserGridElement_TYPE      : return( ((SmUserGridElement      *)this)->GetElement() ) ; 
      default : SM_DBG_WARN(_T("Use of unsupported type in SmGridElement::GetElement\n")) ;
                return(NULL) ;
    } // end switch on type

} // end SmGridElement::GetElement

/*******************************************************************//***
PURPOSE: return pointer to stored SmSurface object when possible,
  else return NULL

NOTES: Both SmGridElements of derived types SmSurfaceGridElement and
  SmHierarchyGridElement store pointers to SmSurface objects.
************************************************************************/
SmSurface* SmGridElement::GetSurface() const 
{ 
  // return contained surface for approriate derived types
  return(  m_lType == SmSurfaceGridElement_TYPE   ? ((SmSurfaceGridElement *)this)->GetElement()
         : m_lType == SmHierarchyGridElement_TYPE ? ((SmHierarchyGridElement *)this)->GetElement()->GetSurface()
         : NULL ) ; 

} // end SmGridElement::GetSurface     

/*******************************************************************//***
PURPOSE: return pointer to stored SmSurface subdivision leaf node UVDomain
  when possible, else return NULL

NOTES: Only SmGridElements of derived type SmSurfaceGridElement 
   stores the details of a SmSurface subdivision leaf node.
************************************************************************/
SmExtent2d* SmGridElement::GetLeafNodeUVDomain() const 
{ 
  // return contained surface for approriate derived types
  return(  m_lType == SmSurfaceGridElement_TYPE   
         ? &((SmSurfaceGridElement *)this)->m_sUVDomain
         : NULL ) ; 

} // end SmGridElement::GetLeafNodeUVDomain     

/*******************************************************************//***
PURPOSE: return pointer to stored SmSurface subdivision leaf node Bounding Box
  when possible, else return NULL

NOTES: Only SmGridElements of derived type SmSurfaceGridElement 
   stores the details of a SmSurface subdivision leaf node.
************************************************************************/
SmExtent3d* SmGridElement::GetLeafNodeBBox() const 
{ 
  // return contained surface for approriate derived types
  return(  m_lType == SmSurfaceGridElement_TYPE   
         ? &((SmSurfaceGridElement *)this)->m_sBBox
         : NULL ) ; 

} // end SmGridElement::GetLeafNodeBBox

/*******************************************************************//***
PURPOSE: return pointer to stored SmSurface subdivision leaf node Pseudo Box
  when possible, else return NULL

NOTES: Only SmGridElements of derived type SmSurfaceGridElement 
   stores the details of a SmSurface subdivision leaf node.
************************************************************************/
SmPseudoBox* SmGridElement::GetLeafNodePseudoBox() const 
{ 
  // return contained surface for approriate derived types
  return(  m_lType == SmSurfaceGridElement_TYPE   
         ? &((SmSurfaceGridElement *)this)->m_sPseudoBox
         : NULL ) ; 

} // end SmGridElement::GetLeafNodePseudoBox

/*******************************************************************//***
PURPOSE: return pointer to stored SmSurface subdivision leaf node Polar Box
  when possible, else return NULL

NOTES: Only SmGridElements of derived type SmSurfaceGridElement 
   stores the details of a SmSurface subdivision leaf node.
************************************************************************/
SmPolarBox* SmGridElement::GetLeafNodePolarBox() const 
{ 
  // return contained surface for approriate derived types
  return(  m_lType == SmSurfaceGridElement_TYPE   
         ? &((SmSurfaceGridElement *)this)->m_sPolarBox
         : NULL ) ; 

} // end SmGridElement::GetLeafNodePolarBox

/*******************************************************************//**
PURPOSE: Set all SmSurfaceGridElement Values from pSurfaceTreeNode 

NOTES: 
***********************************************************************/
void SmSurfaceGridElement::Init(SmTreeNode *pSurfTreeNode)
{
  // SHORTRAYTRACER - remove 1 line when done with changes
//  m_pSurfTreeNode = pSurfTreeNode ;

  // check state
  SM_ASSERT(pSurfTreeNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE) ;
  SM_ASSERT(pSurfTreeNode->GetOwnerObject() != NULL) ;

  // locals
  SmBezierPatch *pBezierPatch = (SmBezierPatch *)pSurfTreeNode->m_pData ;

  // init parent data
  m_lMark      = 0 ;
  m_lType      = SmSurfaceGridElement_TYPE ;

  // copy all data to be saved from the pSurfTreeNode ;
  m_pSurface   = (const SmSurface *)pSurfTreeNode->GetOwnerObject() ;
  m_sUVDomain  = pBezierPatch->m_sUVDomain ;
  m_sBBox      = pSurfTreeNode->m_sBBox ;    
  m_sPseudoBox = pBezierPatch->GetPseudoBox() ;
  m_sPolarBox  = pBezierPatch->GetPolarBox() ;

  // SHORTRAYTRACER - temporary experiment: following can be computed at run time or cached

    // Precomputation of derivative information for this little patch
    SmPoint2d sUVMid = m_sUVDomain.Evaluate(0.5,0.5);
    SmVector3d sEval[2][2];
    SE(m_pSurface->Evaluate(sUVMid,1,1,TRUE,TRUE,TRUE,sEval[0])); 
    m_sMidPoint  = sEval[0][0];
    m_sMidDU     = sEval[1][0];
    m_sMidDV     = sEval[0][1];
    m_sMidNormal = m_sMidDU * m_sMidDV;
    if (m_sMidNormal.LengthSquared() > SM_EFF_ZERO) 
      {
        SE(m_sMidNormal.Unitize());
      }
    else 
      {
        SE(m_pSurface->EvaluateNormal(sUVMid,TRUE,TRUE,m_sMidNormal));
      }

} // end SmSurfaceGridElement::Init

/*******************************************************************//**
PURPOSE: Constructor for the SmGrid object.  It sets up the
    bounding box for the grid and the number of elements along each
    axis for the grid.

NOTES: 
***********************************************************************/
SmGrid::SmGrid
  (const SmExtent3d & crGridBBox,
   ULONG  lSize[3])
 : m_sBBox(crGridBBox), 
   m_lCurrentMark(0)
{
  m_lSize[0] = lSize[0];
  m_lSize[1] = lSize[1];
  m_lSize[2] = lSize[2];

  m_sVoxelSize = crGridBBox.GetSize();
  m_sVoxelSize.x = m_sVoxelSize.x / lSize[0];
  m_sVoxelSize.y = m_sVoxelSize.y / lSize[1];
  m_sVoxelSize.z = m_sVoxelSize.z / lSize[2];

  m_dVoxelTolerance = SM_EFF_ZERO_SQRT * (1.0 + m_sVoxelSize.GetMaxDimension()) / 100.0;
  const SmContext *pContext = GetContext();

  // voxel memory block
  ULONG lTotalSize = m_lSize[0] * m_lSize[1] * m_lSize[2];

  // okay to use smos_Calloc on static class (SmVoxel) objects.
  SmVoxel *  pVArray  = (SmVoxel*) smos_Calloc(1, sizeof(SmVoxel) *lTotalSize);
  if (pVArray == NULL) { SE(SM_ERR); return; }

  // Ptrs to Voxel Vector memory block
  ULONG lXYSize = m_lSize[0] * m_lSize[1];

  // okay to use smos_Calloc on base (pointer) objects.
  SmVoxel ** pXYArray = (SmVoxel**)smos_Calloc(1, sizeof(SmVoxel*)*lXYSize);
  if (pXYArray == NULL) { SE(SM_ERR); return; }

  // Ptr to Voxel Array memory block
  // okay to use smos_Calloc on base (pointer) objects.
  m_aVoxels = (SmVoxel***)smos_Calloc(1, sizeof(SmVoxel**)*m_lSize[0]);
  
  // set up the voxel array memory pointers
  ULONG lZCount = 0;
  for (long ix=0; ix<m_lSize[0]; ix++) 
    {
      m_aVoxels[ix] = &pXYArray[ix*m_lSize[1]];
      for (long iy=0; iy<m_lSize[1]; iy++) 
        {
          m_aVoxels[ix][iy] = &pVArray[lZCount];
          lZCount = lZCount + m_lSize[2];
          for (long iz=0; iz<m_lSize[2]; iz++) 
            {
              m_aVoxels[ix][iy][iz].Init(this);
            }
        }
    }

  // initialize memory management for grid associated objects
    m_sSurfaceElementMgr.Initialize(ALIGN_SIZE(sizeof(SmSurfaceGridElement)),lTotalSize*10);
    m_sUserElementMgr.Initialize(ALIGN_SIZE(sizeof(SmUserGridElement)),lTotalSize*10);
    m_sPolygonElementMgr.Initialize(ALIGN_SIZE(sizeof(SmPolygonGridElement)),lTotalSize*10);

  m_pHierarchyElements = new (*pContext) SmTArray<SmHierarchyElement*>(*pContext);

} // end SmGrid::SmGrid default constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmGrid

NOTES: 
***********************************************************************/
SmGrid::~SmGrid()
{
  for (long ix=0; ix<m_lSize[0]; ix++) 
    {
      for (long iy=0; iy<m_lSize[1]; iy++) 
        {
          for (long iz=0; iz<m_lSize[2]; iz++) 
            {
              if (m_aVoxels[ix][iy][iz].m_pElements) 
                {
                  delete m_aVoxels[ix][iy][iz].m_pElements; m_aVoxels[ix][iy][iz].m_pElements = NULL ;
                }
              if (m_aVoxels[ix][iy][iz].m_pHierarchyElements) 
                {
                  delete m_aVoxels[ix][iy][iz].m_pHierarchyElements; m_aVoxels[ix][iy][iz].m_pHierarchyElements = NULL ;
                }
            }
        }
    }
  smos_Free(m_aVoxels[0][0]); m_aVoxels[0][0] = NULL ;
  smos_Free(m_aVoxels[0]);    m_aVoxels[0]    = NULL ;
  smos_Free(m_aVoxels);       m_aVoxels       = NULL ;
  
  for (ULONG i=0; i<m_pHierarchyElements->GetSize(); i++) 
    {
      SM_ASSERT((*m_pHierarchyElements)[i] != NULL) ; delete (*m_pHierarchyElements)[i] ; (*m_pHierarchyElements)[i] = NULL ;
    }
  SM_ASSERT(m_pHierarchyElements != NULL) ; delete m_pHierarchyElements ; m_pHierarchyElements = NULL ;


} // end SmGrid::~SmGrid destructor


/*******************************************************************//**
PURPOSE: Find the voxels in the given bounding box.

NOTES: 
***********************************************************************/
SmStatus SmGrid::FindVoxelsInBox
  (const SmExtent3d   & crBBox,   // in : bounding box to classify      
   SmTArray<SmVoxel*> & rVoxels)  // out: list of every voxel intersecting
                                  //      input crBBox
  const
{
  // init output
  rVoxels.ReSet();

  // increment BBOx by tolerance
  SmExtent3d sBBox = crBBox;
  sBBox.ExpandAbsolute(m_dVoxelTolerance);

  // get voxels containing min/max corners of the bounding box 
  long lMinAddress[3], lMaxAddress[3];
  SmBoolean bOutsideGrid;
  SER(GetVoxelAddress(crBBox.GetMin(),lMinAddress,bOutsideGrid));
//    if (bOutsideGrid) SER(SM_ERR);
  SER(GetVoxelAddress(crBBox.GetMax(),lMaxAddress,bOutsideGrid));
//    if (bOutsideGrid) SER(SM_ERR);

  // add every voxel between the min/max point containing voxels to output list
  for (long ix=lMinAddress[0]; ix<=lMaxAddress[0]; ix++) 
    {
      for (long iy=lMinAddress[1]; iy<=lMaxAddress[1]; iy++) 
        {
          for (long iz=lMinAddress[2]; iz<=lMaxAddress[2]; iz++) 
            {
              rVoxels.Add(&m_aVoxels[ix][iy][iz]);
            }
        }
    } // end setting voxel list

  // all done
  return SM_SUCCESS;

} // end SmGrid::FindVoxelsInBox


/*******************************************************************//**
PURPOSE: Add an hierarchial element to the grid by placing a pointer to the
  element in every grid->voxel->m_pElements list of a voxel
  that intersects the element's bounding box. 

NOTES: 
  A hierarchy element is currently always a surface/SurfaceCache pair which can be
  broken down into a spatial decomposition tree where each leaf-node
  represents one small bezier patch of the surface.
  The ExpandHierarchy() function can be called to build the surface
  decomposition tree and to add each leaf node as a gridElement to the
  grid->Voxel element lists.

  The grid will now own the hierarchy element and be responsible for deleting it.

METHOD ---
  1. Set pHierarchyElement->m_vGridElement BBox and m_pElement data
  2. Get all voxels that intersect crElementBox
  3. For every intersecting voxel -
       add pHierarchyElement to Voxel->m_pHierarchyElements list
***********************************************************************/
SmStatus SmGrid::AddHierarchyElement
  (SmHierarchyElement * pHierarchyElement,  // in : SmSurfaceHierarchyElement to be added to the grid->Voxel lists
   const SmExtent3d   & crElementBBox)      // in : Bounding Box of the HierarchyElement
{
  // get context for new object construction
  const SmContext * pContext = GetContext();

  // set the pHierarchyElement->m_vGridElement BBox and set its m_pElement = pHierarchyElement
  SmHierarchyGridElement & rGE = pHierarchyElement->m_vGridElement;

  rGE.m_sBBox = crElementBBox;
  rGE.m_sBBox.ExpandAbsolute(pHierarchyElement->m_dExpansionTolerance);
  rGE.m_sBBox.ComputeSphereBound(rGE.m_sSphereCenter,rGE.m_dSphereRadius);
  rGE.m_pHierarchyElement = pHierarchyElement;

  // add pHierarchyElement to m_pHierarchyElements list for later clean up.
  m_pHierarchyElements->Add(pHierarchyElement); 

  // get all voxels that intersect the input bounding box
  SmVoxel *sVData[256];
  SmTArray<SmVoxel*> sVoxels(256,sVData);
  SER(FindVoxelsInBox(crElementBBox,sVoxels));
  
  // for every intersecting voxel - add pointer to pHierarchyElement to pVoxel->m_pHierarchyElements list
  for (ULONG i=0; i<sVoxels.GetSize(); i++) 
    {
      SmVoxel * pVoxel = sVoxels[i];
      if (pVoxel->m_pHierarchyElements == NULL) 
        {
          pVoxel->m_pHierarchyElements = new(*pContext) SmTArray<SmHierarchyElement*>(*pContext);
          NER(pVoxel->m_pHierarchyElements);
        }
      pVoxel->m_pHierarchyElements->Add(pHierarchyElement);
    
    }

  // all done
  return SM_SUCCESS;

} // end SmGrid::AddHierarchyElement


/*******************************************************************//**
PURPOSE: Add an element to the grid by placing a pointer to the
  element in every grid->voxel->m_pElements list of a voxel
  that intersects the element's bounding box.

NOTES: 
  make an SmGridElement object containing a pointer to pNode and BoundingBox data. 
  Then for every voxel that intersects the pNode's bounding box,
     add a pointer to the SmGridElement to the voxel's m_pElements list.

***********************************************************************/
/*******************************************************************//**
PURPOSE: Same function when SHORTRAYTRACER defined.

NOTES: 
***********************************************************************/
SmStatus SmGrid::AddElement
  (void             * pElement,         // in : type = SmTreeNode * from Surface decomposition for SM_GE_SURFACE
                                        //             SmTreeNode * from PolyBrep decomposition for SM_GE_POLYBREP
                                        //             void *       from User Application
   SmGridElementType  eGridElementType, // in : oneof  SM_GE_SURFACE   for SmTreeNode * from Surface decomposition
                                        //             SM_GE_POLYBREP  for SmTreeNode * from PolyBrep decomposition
                                        //             SM_GE_USERDATA  for void *       from User Application
   const SmExtent3d & crElementBBox)    // in : BoundingBox for Element
{
  // check state
  SM_ASSERT(   eGridElementType == SM_GE_SURFACE
            || eGridElementType == SM_GE_POLYBREP
            || eGridElementType == SM_GE_USERDATA) ;

  // locals
  SmGridElement *pGE = NULL ;

  // allocate and init properly derived SmGridElement structure
  switch(eGridElementType)
    {
      case SM_GE_SURFACE  : { SmSurfaceGridElement *pSGE = (SmSurfaceGridElement*)m_sSurfaceElementMgr.GetNewElement() ;
                              pSGE->Init((SmTreeNode *)pElement) ;
                              pGE = pSGE ;
                            }
                            break ;

      case SM_GE_POLYBREP : { SmPolygonGridElement *pPGE = (SmPolygonGridElement*)m_sPolygonElementMgr.GetNewElement() ;
                              pPGE->Init((SmTreeNode *)pElement) ;
                              pGE = pPGE ;
                            }
                            break ;

      case SM_GE_USERDATA : { SmUserGridElement *pUGE = (SmUserGridElement*)m_sUserElementMgr.GetNewElement() ;
                              pUGE->Init((void *)pElement) ;
                              pGE = pUGE ;
                            }
                            break ;
      default:              SM_DBG_WARN(_T("Unexpected Type seen in eGridElementType switch\n")) ;
                            break ;
    } // end switch on eGridElementType

  // load root GridElement data with element and bbox data
  crElementBBox.ComputeSphereBound(pGE->m_sSphereCenter,pGE->m_dSphereRadius);
  pGE->m_sBBox    = crElementBBox;

  // find all voxels that intersect Element's bounding box
  SmVoxel *sVData[256];
  SmTArray<SmVoxel*> sVoxels(256,sVData);
  SER(FindVoxelsInBox(crElementBBox,sVoxels));
  
  // add GridElement pointer to every hit voxel's m_pElements list
  for (ULONG i=0; i<sVoxels.GetSize(); i++) 
    {
      SmVoxel * pVoxel = sVoxels[i];
      if (pVoxel->m_pElements == NULL) 
        {
          const SmContext *pContext = GetContext();
          pVoxel->m_pElements = new(*pContext) SmTArray<SmGridElement*>(*pContext);
          NER(pVoxel->m_pElements);
        }
      pVoxel->m_pElements->Add(pGE);
    }

  // all done
  return SM_SUCCESS;

} // end SmGrid::AddElement

/*******************************************************************//**
PURPOSE: Initialize the 3DDDA data with 1st GridVoxel/Ray intersection
            data setting up the stepping algorithm.

NOTES: Does no setup and sets rbHitsNothing == TRUE when
            Ray does not intersect the Grid's BoundingBox

SIDE EFFECTS ---
   m_dRayT          = Ray entering Parameter Value
   m_sCurrPoss      = GridVoxel/Ray intersection point
   m_dNextT         = Param steps to next X, Y, and Z voxel plane boundaries
   m_dDeltaT        = Param steps to move between X, Y, and Z voxel plane boundaries
   m_lStepX,Y,Z     = voxel address increment value as ray moves from grid voxel to grid voxel
   m_lOutX,Y,Z      = voxel grid exit address index for X Y and Z for when ray finally exits grid
   m_sDeltaXVec,Y,Z = 3d vector steps between X,Y, and Z planes
   m_lCurrentMark   = Gets incremented
***********************************************************************/
SmStatus SmGrid::Setup3DDDA
  (const SmPoint3d  & crRayStart,      // in : ray start point
   const SmVector3d & crRayVector,     // in : ray direction (non-zero)
   SmBoolean        & rbHitsNothing)   // out: TRUE = ray does not hit any of the grid voxels
                                       //      FALSE= ray hits voxels but may or may not hit element data
{
  rbHitsNothing = FALSE;

  // check input
  if (crRayVector.LengthSquared() < SM_EFF_ZERO) 
    { SER(SM_ERR); }

  // store unitized ray in Grid
  m_sRayPoint  = crRayStart;
  m_sRayVector = crRayVector;
  SER(m_sRayVector.Unitize());

  // set m_dRayT = Ray's param where it enters BoundingBox or Containing Voxel
  ULONG lNumFound;
  double dTEnter = 0.0, dTExit = 0.0;
  // when Ray starts in Grid Bounding Box
  if (m_sBBox.ContainsPoint3d(m_sRayPoint)) 
    {
      // get Voxel indices containing RayPoint
      long      lVoxelAdd[3];
      SmBoolean bOutsideGrid;
      SER(GetVoxelAddress(m_sRayPoint,lVoxelAdd,bOutsideGrid));
      if (bOutsideGrid) SER(SM_ERR);
      SmExtent3d sVoxelBox = GetVoxelBBox(lVoxelAdd);

      // intersect ray with containing Voxel
      SER(sVoxelBox.IntersectLine(m_sRayPoint,
                                  m_sRayVector,
                                  lNumFound,
                                  dTEnter,
                                  dTExit));
      // store entering param value
      m_dRayT = dTEnter;
    }
  else // else ray starts outside of Grid Bounding Box
    {
      // intersect ray with GridBoundingBox
      SER(m_sBBox.IntersectLine(m_sRayPoint,
                                m_sRayVector,
                                lNumFound,
                                dTEnter,
                                dTExit));
      // quit when ray misses GridBoundingBox
      if (lNumFound == 0) { rbHitsNothing = TRUE;
                            return SM_SUCCESS;
                          }
      // store entering param value
      m_dRayT = dTEnter;
    }

  // arrive here when
  // m_dRayT      = start param where ray enters Grid or containing voxel
  // m_sRayPoint  = ray start point
  // m_sRayVector = ray unit direction

  // Get start Point (and test point = startPoint + tolerance)
  SmPoint3d sTestPnt = m_sRayPoint + (m_dRayT + m_dVoxelTolerance)*m_sRayVector;
  m_sCurrPoss        = m_sRayPoint + m_dRayT * m_sRayVector;
  SmBoolean bOutsideGrid;

  // get voxel containing start point
  SER(GetVoxelAddress(sTestPnt,m_lAddress,bOutsideGrid));
  if (bOutsideGrid) { // quit when ray fails to hit a voxel
                      rbHitsNothing = TRUE;
                      return SM_SUCCESS;
                    }

  // get voxel bounding box
  SmExtent3d sVoxelBox = GetVoxelBBox(m_lAddress);

  // set X direction step to next X plane boundary
  if (smos_Fabs(m_sRayVector.x) < SM_EFF_ZERO) 
    {
      m_dNextT.x  = SM_BIG_DOUBLE;
      m_dDeltaT.x = 0.0;
    }
  else if (m_sRayVector.x < 0.0) 
    {
      m_dNextT.x  = m_dRayT + (sVoxelBox.GetMin().x - m_sCurrPoss.x) / m_sRayVector.x;
      m_dDeltaT.x = m_sVoxelSize.x / - m_sRayVector.x;
      m_lStepX    = -1; // Increment for voxel address in X
      m_lOutX     = -1; // When we are outside of grid the X address will be
    }
  else // m_sRayVector.x > 0.0
    { 
      m_dNextT.x  = m_dRayT + (sVoxelBox.GetMax().x - m_sCurrPoss.x) / m_sRayVector.x;
      m_dDeltaT.x = m_sVoxelSize.x / m_sRayVector.x;
      m_lStepX    = 1;
      m_lOutX     = m_lSize[0];
    }

  // set Y direction step to next Y plane boundary
  if (smos_Fabs(m_sRayVector.y) < SM_EFF_ZERO) 
    {
      m_dNextT.y = SM_BIG_DOUBLE;
      m_dDeltaT.y = 0.0;
    }
  else if (m_sRayVector.y < 0.0) 
    {
      m_dNextT.y  = m_dRayT + (sVoxelBox.GetMin().y - m_sCurrPoss.y) / m_sRayVector.y;
      m_dDeltaT.y = m_sVoxelSize.y / - m_sRayVector.y;
      m_lStepY    = -1; // Increment for voxel address in Y
      m_lOutY     = -1; // When we are outside of grid the Y address will be
    }
  else 
    { // m_sRayVector.x > 0.0
      m_dNextT.y  = m_dRayT + (sVoxelBox.GetMax().y - m_sCurrPoss.y) / m_sRayVector.y;
      m_dDeltaT.y = m_sVoxelSize.y / m_sRayVector.y;
      m_lStepY    = 1;
      m_lOutY     = m_lSize[1];
    }

  // set Z direction step to next Z plane boundary
  if (smos_Fabs(m_sRayVector.z) < SM_EFF_ZERO) 
    {
      m_dNextT.z  = SM_BIG_DOUBLE;
      m_dDeltaT.z = 0.0;
    }
  else if (m_sRayVector.z < 0.0) 
    {
      m_dNextT.z  = m_dRayT + (sVoxelBox.GetMin().z - m_sCurrPoss.z) / m_sRayVector.z;
      m_dDeltaT.z = m_sVoxelSize.z / - m_sRayVector.z;
      m_lStepZ    = -1; // Increment for voxel address in Z
      m_lOutZ     = -1; // When we are outside of grid the Z address will be
    }
  else 
    { // m_sRayVector.x > 0.0
      m_dNextT.z  = m_dRayT + (sVoxelBox.GetMax().z - m_sCurrPoss.z) / m_sRayVector.z;
      m_dDeltaT.z = m_sVoxelSize.z / m_sRayVector.z;
      m_lStepZ    = 1;
      m_lOutZ     = m_lSize[2];
    }

  // Set up delta vectors and Next intersections
  m_sDeltaXVec = m_sRayVector * m_dDeltaT.x;
  m_sDeltaYVec = m_sRayVector * m_dDeltaT.y;
  m_sDeltaZVec = m_sRayVector * m_dDeltaT.z;

  // Set up Next Intersection points for X, Y, Z
  if (m_dNextT.x < SM_BIG_DOUBLE) m_sNextX = m_sRayPoint + m_dNextT.x * m_sRayVector;
  if (m_dNextT.y < SM_BIG_DOUBLE) m_sNextY = m_sRayPoint + m_dNextT.y * m_sRayVector;
  if (m_dNextT.z < SM_BIG_DOUBLE) m_sNextZ = m_sRayPoint + m_dNextT.z * m_sRayVector;

  // increment the grid's mark
  m_lCurrentMark++;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // m_sRayPoint  = RayStart Point
  // m_sRayVector = Ray Unit Direction
  // m_dRayT      = param value where ray enters 1st grid element (can be neg when point starts in a voxel)
  // m_sCurrPoss  = Point position for m_dRayT
  // sTestPnt     = point position stepped from m_sCurrPoss by tolerance
  // sVoxelBox    = Voxel Box of 1st intersection grid element
  // m_dNextT.x   = param of next X-plane xsect
  // m_dNextT.y   = param of next y-plane xsect (usually not the same point as x-plane xsect)
  // m_dNextT.z   = param of next z-plane xsect (usually not the same point as x or y plane xsects)
  // m_dDeltaT.x  = delta param value to move from one x plane to the next
  // m_dDeltaT.y  = delta param value to move from one y plane to the next
  // m_dDeltaT.z  = delta param value to move from one z plane to the next
  // m_sDeltaXVec = RayVector set to length to move from one x plane to the next
  // m_sDeltaYVec = RayVector set to length to move from one y plane to the next
  // m_sDeltaZVec = RayVector set to length to move from one z plane to the next
  // m_sNextX     = Next XPlane intersection Point
  // m_sNextY     = Next YPlane intersection Point
  // m_sNextZ     = Next ZPlane intersection Point
  // m_lAddress   = index to current voxel box as in, SmExtent3d sVoxelBox = GetVoxelBBox(m_lVoxelAdd);
  if (bDebugMe) 
    {
      smgfx_Erase();
//            for(ULONG ii=0; ii<m_sSurfaces.GetSize(); ii++)
//              { smgfx_SetLook(1,4, 0,1,1) ; m_sSurfaces[ii]->DrawUV() ; sm_GraphicsLoop() ; }
//      
//            for(ULONG jj=0; jj<m_sPolyBreps.GetSize(); jj++)
//              { smgfx_SetLook(1,4, 0,1,1) ; m_sPolyBreps[jj]->Draw() ; sm_GraphicsLoop() ; }


      smgfx_SetLook(1,2, 1,1,0); this->Draw(TRUE,TRUE,TRUE,TRUE) ; sm_GraphicsLoop() ;  // draw grid with voxels
      smgfx_SetLook(1,2, 1,1,0); this->Draw(TRUE,TRUE,TRUE,FALSE) ; sm_GraphicsLoop() ;   // draw grid without voxels
      // the ray 
      smgfx_SetLook(1,4,  1,0,0); crRayStart.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, .5,1,0); crRayVector.Draw(&crRayStart); sm_GraphicsLoop() ;
      // 1st voxel intersected by ray
      smgfx_SetLook(2,3, 0,1,1); sVoxelBox.Draw() ; sm_GraphicsLoop() ;
      // 1st Voxel entry point
      smgfx_SetLook(3,4, 0,0,1); m_sCurrPoss.Draw() ; sm_GraphicsLoop() ;
      // possible voxel exit points - next X,Y,Z Plane intersection points
      smgfx_SetLook(3,4, 0,1, 0); m_sNextX.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,.2); m_sNextY.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,.4); m_sNextZ.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmGrid::Setup3DDDA

/*******************************************************************//**
PURPOSE: Add element, dTEnter, dTExit values to lists sorted by
  increasing dTEnter values.

NOTES:
***********************************************************************/
static void sm_InsertionSort
  (SmGridElement            * pElem,                   // in : element 
   double                     dTEnter,                 // in : enter param value to associate with element
   double                     dTExit,                  // in : exit param value to associate with element
   SmTArray<SmGridElement*> & rSortedElementsInVoxel,  // i/o: list of elements sorted by dTEnter value    
   SmTArray<double>         & rMinDistances,           // i/o: associated dTEnter value    
   SmTArray<double>         & rMaxDistances)           // i/o: associated dTExit value    
{
  // find index for element based on dTEnter sort value
  ULONG lInsertAt = rMinDistances.GetSize(); // if largest, it goes at end.
  for (ULONG i=0; i<rMinDistances.GetSize(); i++) 
    {
      if (dTEnter < rMinDistances[i]) 
        {
          lInsertAt = i;
          break;
        }
    }

  // add sorted element data to lists
  rSortedElementsInVoxel.InsertAt(lInsertAt,pElem);
  rMinDistances.InsertAt(lInsertAt,dTEnter);
  rMaxDistances.InsertAt(lInsertAt,dTExit);

} // end sm_InsertionSort

/*******************************************************************//**
PURPOSE: traverse ray through voxels until the ray passes close
            enough to an element embeded within the voxels so that
            the ray intersects the element's bounding box or the
            ray exits the grid.

            Loads the output arrays element pointers and ray intervals
            of the element->BBox/Ray intersections.

NOTES:
  An ElementBBox/Ray intersection may or may not contain a element/ray intersection.

  May traverse through many voxels looking for the next
            ray/ElementBBox intersection.

SIDE EFFECTS ---
  Expands (Build SurfaceCache) 
  and embeds (inserts SmGridElements into Grid for every Leafnode)
   for all HierarchyElements (Surfaces).

  Increments the following SmGrid values to the voxel containing the next
  ElementBBox/Ray intersection. 
     SmGrid->m_lAddress
     SmGrid->m_dRayT
     SmGrid->m_sCurrPoss
     SmGrid->m_NextT
     SmGrid->m_sNextX, Y, and Z.

GWC NOTE ---
  Expands and Embeds all Surface data into the Voxels creating 
    pCurrentVoxel->m_pElements entries.

  Uses just the m_pElement summary data to decide when a ray intersects
    a pElement->BoundingBox and adds those hits to the output arrays.
***********************************************************************/
SmStatus SmGrid::Step3DDDA
  (double                     dMinFoundT,              // in : Ray parameter value for current 1st ray/element intersection    
   SmBoolean                & rbExitedGrid,            // out: TRUE = Ray has exited the grid
                                                       //      FALSE= Ray has entered another voxel    
   long [3],                                           // in : lCurrentVoxel now stored in SmGrid object
   SmTArray<SmGridElement*> & rSortedElementsInVoxel,  // out: elements whose BBoxes intersect the ray, sorted by increasing dTEnter values
   SmTArray<double>         & rMinDistances,           // out: associated ElementBBox/Ray intersection dTEnter values
   SmTArray<double>         & rMaxDistances)           // out: associated ElementBBox/Ray intersection dTExit values
                                                       // note: uses without incrementing SmGrid::m_lCurrentMark value
{
  // init output
  rbExitedGrid = FALSE;
  rSortedElementsInVoxel.ReSet();
  rMinDistances.ReSet();
  rMaxDistances.ReSet();

 // loop until ray hits an object or exits the grid
 for(;;) 
   {
      // get Voxel for current m_lAddress
      SmVoxel *pVoxel = GetVoxel(m_lAddress);

#ifdef SM_DEBUG_CODE
     // draw Voxel(Black), Ray (start=red,dir=green,end=yellow), Grid(yellow)
SmBoolean bDebugMe = FALSE;
     if (bDebugMe) 
       { 
         smgfx_Erase() ;
         smgfx_SetLook(1,1, 1,1,0); Draw(TRUE,TRUE,TRUE,FALSE); sm_GraphicsLoop();
         sm_GraphicsLoop();
       }
#endif  // SM_DEBUG_CODE

      // gwc: no longer need next loop - Hierarchy elements are expanded when surfaces are added
      //          with SmRayTracer::AddSurfaceToGrid
      //      
      //      // Expand and embed all HierarchyElements in this voxel
      //      if (pVoxel->m_pHierarchyElements) 
      //        {
      //          SmBoolean bDoneExpanding = FALSE;
      //          SmTArray<SmHierarchyElement*> & rHierarchyElements = *pVoxel->m_pHierarchyElements;
      //      
      //          // Keep going until all hierarchy elements in this voxel are expanded
      //          // we have to do this because of the possibility of recursive nesting.
      //          while (!bDoneExpanding) 
      //            {
      //              bDoneExpanding = TRUE;
      //              for (ULONG jj=0; jj<rHierarchyElements.GetSize(); jj++) 
      //                {
      //                  SmHierarchyElement *pHier = rHierarchyElements[jj];
      //                  SM_ASSERT( pHier->m_bHasBeenExpanded == TRUE) ;
      //                  if (!pHier->m_bHasBeenExpanded) 
      //                    {
      //                      bDoneExpanding = FALSE;
      //                      // current hierarchy elements are always SMSurface Objects
      //                      // - build Surf spatial decomposition Tree, and 
      //                      //   embed every decomposition leaf-node into GridVoxels that contain it
      //                      pHier->ExpandHierarchy(this);
      //                   }
      //               }
      //           } // While !done expanding
      //      
      //       } // end Voxel has HierarchyElements check

      // when voxel contains elements
      if (pVoxel->m_pElements != NULL) 
        {
          // for every Element in this voxel
          // GWC Note: uses just SmGridElement summary info to build output lists
          SmTArray<SmGridElement*> & rElements = *pVoxel->m_pElements;
          for (ULONG i=0; i<rElements.GetSize(); i++) 
            {
              SmGridElement *pGridElem = (SmGridElement*)rElements[i];

              // skip elements already traversed during this ray cast. 
              if (pGridElem->m_lMark == m_lCurrentMark) 
                { continue; }

              // Get ray/ElementCenter closest distance
              SmVector3d sVecToCenter = pGridElem->m_sSphereCenter - m_sRayPoint;
              double     dCenterT     = sVecToCenter.Dot(m_sRayVector);
              SmPoint3d  sPointOnRay  = m_sRayPoint + dCenterT * m_sRayVector;
              SmVector3d dVecTo       = sPointOnRay - pGridElem->m_sSphereCenter;
              double     dDistSq      = dVecTo.LengthSquared();

              // skip rays which are too far from the element's center
              if (dDistSq > pGridElem->m_dSphereRadius*pGridElem->m_dSphereRadius) 
                { continue; }

              // intersect ray with element bounding box
              ULONG lNumFound;
              double dTEnter=0.0, dTExit=0.0;
              SER(pGridElem->m_sBBox.IntersectLine(m_sRayPoint, m_sRayVector,
                                                   lNumFound, dTEnter, dTExit));

              // skip rays that don't intersect bbox or intersect at a negative parameter value.
              if (lNumFound == 0) 
                { continue ; }
              if (dTExit < -SM_EFF_ZERO) 
                { continue ; }

              // Skip intersections that happen to start after some other found ray intersection point 
              if (dTEnter > dMinFoundT) 
                { continue ; }

              // add the bbox/ray intersection interval to the output lists
              pGridElem->m_lMark = m_lCurrentMark;
              sm_InsertionSort(pGridElem, dTEnter, dTExit,
                                  rSortedElementsInVoxel, rMinDistances, rMaxDistances);

            } // end iter every element in the voxel
        } // end voxel has elements check

      // Step to next voxel - either an increment in X, Y, or Z index

      // Step in X 
      if (   m_dNextT.x < m_dNextT.y 
          && m_dNextT.x < m_dNextT.z) 
        {
          // Step in X
          m_lAddress[0] += m_lStepX;
          if (m_lAddress[0] == m_lOutX) 
            {
              rbExitedGrid = TRUE;
              return SM_SUCCESS;
            }
          m_dRayT     = m_dNextT.x;
          m_dNextT.x += m_dDeltaT.x;
          m_sCurrPoss = m_sNextX;
          m_sNextX    = m_sNextX + m_sDeltaXVec;
        }
      // Step in Y
      else if (m_dNextT.y < m_dNextT.z) 
        {
          // Step in Y
          m_lAddress[1] += m_lStepY; 
          if (m_lAddress[1] == m_lOutY) 
            {
              rbExitedGrid = TRUE;
              return SM_SUCCESS;
            }
          m_dRayT     = m_dNextT.y;
          m_dNextT.y += m_dDeltaT.y;
          m_sCurrPoss = m_sNextY;
          m_sNextY    = m_sNextY + m_sDeltaYVec;

        }
      // Step in Z
      else 
        { // Step in Z
          // Step in X
          m_lAddress[2] += m_lStepZ;
          if (m_lAddress[2] == m_lOutZ) 
            {
              rbExitedGrid = TRUE;
              return SM_SUCCESS;
            }
          m_dRayT     = m_dNextT.z;
          m_dNextT.z += m_dDeltaT.z;
          m_sCurrPoss = m_sNextZ;
          m_sNextZ    = m_sNextZ + m_sDeltaZVec;
        }

      if (rSortedElementsInVoxel.GetSize() > 0) break;
    
    } // end infinite loop

  return SM_SUCCESS;

} // end SmGrid::Step3DDDA

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmGrid.

NOTES:
***********************************************************************/
ULONG SmGrid::GetMemoryUsed        // rtn: smaller size of actually used memory in bytes
   (ULONG &rlMemoryAllocated)
  const
{
  // locals
  ULONG lThisUsed, lThisAllocated ;

  // This Memory + Voxel and Voxel Array memory
  ULONG lThisMemory =   sizeof(*this)
                      + sizeof(SmVoxel**) * m_lSize[0] 
                      + sizeof(SmVoxel*)  * m_lSize[0] * m_lSize[1]
                      + sizeof(SmVoxel)   * m_lSize[0] * m_lSize[1] * m_lSize[2] ;

  // within each voxel is an array of SmHierarchyElement ptrs and 
  // an array of SmGridElement ptrs.
  ULONG lVoxelArrayUsed = 0, lVoxelArrayAllocated = 0 ;
  for (long ix=0; ix<m_lSize[0]; ix++) 
    {
      for (long iy=0; iy<m_lSize[1]; iy++) 
        {
          for (long iz=0; iz<m_lSize[2]; iz++) 
            {
              if (m_aVoxels[ix][iy][iz].m_pElements) 
                {
                  lThisUsed = m_aVoxels[ix][iy][iz].m_pElements->GetMemoryUsed(lThisAllocated) ;
                  
                  lVoxelArrayUsed      += lThisUsed ;
                  lVoxelArrayAllocated += lThisAllocated ;
                }
              if (m_aVoxels[ix][iy][iz].m_pHierarchyElements) 
                {
                  lThisUsed = m_aVoxels[ix][iy][iz].m_pHierarchyElements->GetMemoryUsed(lThisAllocated) ;
                  
                  lVoxelArrayUsed      += lThisUsed ;
                  lVoxelArrayAllocated += lThisAllocated ;
                }
            }
        }
    }

  // All SmGridMemory Memory referenced by Voxels is stored in m_sElementMgr
  ULONG lGridElementAllocated ; 
  ULONG lthisElementAllocated ;
  ULONG lGridElementUsed       = m_sSurfaceElementMgr.GetMemoryUsed(lGridElementAllocated) ;

        lGridElementUsed      += m_sUserElementMgr.GetMemoryUsed(lthisElementAllocated) ;
        lGridElementAllocated += lthisElementAllocated ;
        lGridElementUsed      += m_sPolygonElementMgr.GetMemoryUsed(lthisElementAllocated) ;
        lGridElementAllocated += lthisElementAllocated ;


  // All SmHierarchyElement Memory referency by Voxels is listed in m_pHierarchyElements
  ULONG lHierarchyElementAllocated = 0 ;
  ULONG lHierarchyElementUsed      = 0 ;

  if(m_pHierarchyElements)
    {
      // Pointer + Array memory
      lHierarchyElementUsed = m_pHierarchyElements->GetMemoryUsed(lHierarchyElementAllocated) ;

      // Add in member Object memory
      lHierarchyElementUsed      +=  m_pHierarchyElements->GetSize() * sizeof(SmHierarchyElement) ;
      lHierarchyElementAllocated +=  m_pHierarchyElements->GetSize() * sizeof(SmHierarchyElement) ;
    }

  // Get totals
  rlMemoryAllocated =   lThisMemory
                      + lVoxelArrayAllocated
                      + lGridElementAllocated
                      + lHierarchyElementAllocated ;
  lThisUsed         =   lThisMemory
                      + lVoxelArrayUsed
                      + lGridElementUsed
                      + lHierarchyElementUsed ;

  // all done
  return(lThisUsed) ;

} // end SmGrid::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Fine the voxel address of this 3D point.

NOTES: Even if it is outside of the grid.  We will find the
   closest element in the grid.  
***********************************************************************/
SmStatus SmGrid::GetVoxelAddress( const SmPoint3d & crPoint,
                                  long lVoxelAddress[3],
                                  SmBoolean & rbOutsideGrid) const
{

    SmPoint3d sNormalizedParameters = (crPoint - m_sBBox.GetMin());
    SmVector3d sVec = m_sBBox.GetSize();
    sNormalizedParameters.x /= sVec.x;
    sNormalizedParameters.y /= sVec.y;
    sNormalizedParameters.z /= sVec.z;

    rbOutsideGrid = FALSE;
    if (sNormalizedParameters.x < 0.0 || sNormalizedParameters.x > 1.0) { 
        rbOutsideGrid = TRUE;
    }
    if (sNormalizedParameters.y < 0.0 || sNormalizedParameters.y > 1.0) {
        rbOutsideGrid = TRUE;
    }
    if (sNormalizedParameters.z < 0.0 || sNormalizedParameters.z > 1.0) {
        rbOutsideGrid = TRUE;
    }
    
    if (sNormalizedParameters.x < 0.0) {
        sNormalizedParameters.x = 0.0;
    }
    else if (sNormalizedParameters.x > 1.0) { 
        sNormalizedParameters.x = 1.0;
    }
    if (sNormalizedParameters.y < 0.0) {
        sNormalizedParameters.y = 0.0;
    }
    else if (sNormalizedParameters.y > 1.0) { 
        sNormalizedParameters.y = 1.0;
    }
    if (sNormalizedParameters.z < 0.0) {
        sNormalizedParameters.z = 0.0;
    }
    else if (sNormalizedParameters.z > 1.0) { 
        sNormalizedParameters.z = 1.0;
    }
        
    if (sNormalizedParameters.x > 1.0-SM_EFF_ZERO) {
        sNormalizedParameters.x -= SM_EFF_ZERO;
    }
    if (sNormalizedParameters.y > 1.0-SM_EFF_ZERO) {
        sNormalizedParameters.y -= SM_EFF_ZERO;
    }
    if (sNormalizedParameters.z > 1.0-SM_EFF_ZERO) {
        sNormalizedParameters.z -= SM_EFF_ZERO;
    }
    lVoxelAddress[0] = (ULONG) (sNormalizedParameters.x * m_lSize[0]);
    lVoxelAddress[1] = (ULONG) (sNormalizedParameters.y * m_lSize[1]);
    lVoxelAddress[2] = (ULONG) (sNormalizedParameters.z * m_lSize[2]);

    if(lVoxelAddress[0] == m_lSize[0]) lVoxelAddress[0] = lVoxelAddress[0] - 1 ;
    if(lVoxelAddress[1] == m_lSize[1]) lVoxelAddress[1] = lVoxelAddress[1] - 1 ;
    if(lVoxelAddress[2] == m_lSize[2]) lVoxelAddress[2] = lVoxelAddress[2] - 1 ;

    return SM_SUCCESS;

} // end SmGrid::GetVoxelAddress


/*******************************************************************//**
PURPOSE: Draw Grid Voxels containing expanded Hierarchy element data,
                 Grid BBounding Box,
                 Grid Hierarchy Elements (the surfaces), and
                 Grid Stepping data.

NOTES:
***********************************************************************/
SmDisplayList * SmGrid::Draw
  (SmBoolean bDrawVoxels,            // in : TRUE = draw voxels that contain geometry, FALSE = don't
                                     //      default:[TRUE] 
   SmBoolean bDrawVoxelGeometry,     // in : TRUE = Draw bounded geometry in target voxel, FALSE = don't
                                     //      default:[TRUE]
   SmBoolean bDrawSteppingData,      // in : TRUE = Draw ray Current Point, extent in current voxel, FALSE = don't
                                     //      default:[TRUE]
   SmBoolean bDrawHierarchyElements) // in : TRUE = Draw contained hierarchy elements (the surfaces), FALSe = don't
                                     //      default:[TRUE]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  // Only when not asked to skip voxels (they can hide the stepping data)
  if(bDrawVoxels)
    {
      // for every voxel
      long lVoxelAddress[3];
      for (long ix=0; ix<m_lSize[0]; ix++) 
        {
          lVoxelAddress[0] = ix;
          for (long iy=0; iy<m_lSize[1]; iy++) 
            {
              lVoxelAddress[1] = iy;
               for (long iz=0; iz<m_lSize[2]; iz++) 
                {
                  lVoxelAddress[2] = iz;

                  // skip voxels that don't contain geometry
                  //  - this will be true for all voxels prior to embedding the elements
                  //    which currently gets done as a side effect of SmRayTracer::AddSurfaceToGrid()
                  //    when it calls SmSurfaceHierarchyElement::ExpandHierarchy()
                  SmVoxel *pVoxel = GetVoxel(lVoxelAddress);
                  if(   pVoxel->m_pElements==NULL 
                     || pVoxel->m_pElements->GetSize() == 0) 
                    { continue ; }

                  SmExtent3d sBBox = GetVoxelBBox(lVoxelAddress);

                  // draw voxel boxes containing element shapes
                  sBBox.Draw() ;
                }
            }
        } // end iter every voxel
    } // end not asked to skip voxels check

  // draw the Grid Bounding Box
  smgfx_SetLook(2,4, 0,0,0) ; m_sBBox.Draw() ; 

  // draw current voxel and optionally, voxel contents
  if(   m_lAddress[0] >= 0 && m_lAddress[0] < m_lSize[0]
     && m_lAddress[1] >= 0 && m_lAddress[1] < m_lSize[1]
     && m_lAddress[2] >= 0 && m_lAddress[2] < m_lSize[2])
    {
      SmExtent3d sVoxelBox = GetVoxelBBox(m_lAddress);
      smgfx_SetLook(3,5, 0,0,0) ; sVoxelBox.Draw() ; 

      // when asked - draw bounded voxel geometry
      if(bDrawVoxelGeometry)
        {
          SmVoxel *pVoxel = GetVoxel(m_lAddress);
          smgfx_SetLook(1,2, 0,1,1) ; pVoxel->DrawGridElements() ;
        }
    }

  // draw the hierarchy elements
  if(bDrawHierarchyElements)
    {
      for(ULONG ii=0;ii<m_pHierarchyElements->GetSize();ii++)
        {
          // there are currently only surface hierarchy elements
          SmSurfaceHierarchyElement *pElement = (SmSurfaceHierarchyElement *)(*m_pHierarchyElements)[ii] ;

          // Draw the hiearchy elements
          smgfx_SetLook(1,2, 0,1,1) ; pElement->GetSurface()->DrawUV() ;

        } // end iter every hierarchy element
    }

  // draw the stepping data when initialized - won't be before first ray trace
  if(bDrawSteppingData)
    {
      // the ray
      smgfx_SetLook(2,5, 0,0,1) ; if(m_sRayPoint.IsInitialized()) m_sRayPoint.Draw() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(m_sRayVector.IsInitialized() && m_sRayPoint.IsInitialized()) (20.0 *m_sRayVector).Draw(&m_sRayPoint) ; 

      // entry point to current voxel - can be assoc with neg param when RayPoint is in a voxel
      smgfx_SetLook(3,5, 1,0,1) ; if(m_sCurrPoss.IsInitialized()) m_sCurrPoss.Draw() ; 

      if(m_sNextX.IsInitialized() && m_sNextY.IsInitialized() && m_sNextZ.IsInitialized())
        {
          // possible voxel exit points - next X,Y,Z Plane intersection points
          smgfx_SetLook(3,5, 0,1, 0); m_sNextX.Draw() ; 
          smgfx_SetLook(3,5, 0,1,.2); m_sNextY.Draw() ; 
          smgfx_SetLook(3,5, 0,1,.4); m_sNextZ.Draw() ; 
        }

      // Current voxel segment
      double dT = smos_3Min(m_dNextT.x, m_dNextT.y, m_dNextT.z) ;
      SmPoint3d sNextPoint = m_sRayPoint + dT * m_sRayVector ;
      smgfx_SetLook(2,3, 1,.5,0) ; if(m_sCurrPoss.IsInitialized()) (sNextPoint - m_sCurrPoss).Draw(&m_sCurrPoss) ;
    } 

  // all done
  pRtn = smgfx_Close() ;

#else
  SM_REF4(bDrawVoxels, bDrawVoxelGeometry, bDrawSteppingData, bDrawHierarchyElements);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGrid::Draw

/*******************************************************************//**
PURPOSE:  Draw bounding box of target voxel identified by current
   m_lAddress value.

NOTES:
***********************************************************************/
SmDisplayList * SmGrid::DrawVoxel
(long [3])                // lVoxelAddress
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  SmExtent3d sBBox = GetVoxelBBox(m_lAddress);
  sBBox.Draw(GetContext());

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGrid::DrawVoxel

/*******************************************************************//**
PURPOSE: Draw bounding boxes of all hierarchy elements, surfaces, in 
   target voxel identified by current m_lAddress value.

NOTES:
***********************************************************************/
SmDisplayList * SmGrid::DrawVoxelHierarchyElements()
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  SmVoxel *pVoxel = GetVoxel(m_lAddress);
  pVoxel->DrawHierarchyElements() ;

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGrid::DrawVoxelHierarchyElements

/*******************************************************************//**
PURPOSE: Draw bounding boxes of all grid elements, surface->LeafNode, in 
   target voxel identified by current m_lAddress value.

NOTES:
***********************************************************************/
SmDisplayList * SmGrid::DrawVoxelGridElements()
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  SmVoxel *pVoxel = GetVoxel(m_lAddress);
  pVoxel->DrawGridElements() ;

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGrid::DrawVoxelHierarchyElements

/*******************************************************************//**
PURPOSE: Draw bounding boxes of all hierarchy elements (surfaces) 
   whose BBox intersects this Voxel's BBox.

NOTES:
***********************************************************************/
SmDisplayList * SmVoxel::DrawHierarchyElements()
{
  SmDisplayList *pRtn = NULL ;

  if ( m_pHierarchyElements == NULL ) { return pRtn; }

#ifdef SM_GFX_CODE
  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());
  for(ULONG ii=0;ii<m_pHierarchyElements->GetSize();ii++)
    {
      // For now: 
      // All SmHierarchyElement objects are of derived type SmSurfaceHierarchyElement
      //   which stores an SmSurface pointer and a SmHierarchyGridElement pointer
      //   
      // Different SmGridElement derived types store different data:
      //           DerivedType       | SurfacePtr | LeafNode Data
      //    -------------------------+------------+--------------
      //    SmSurfaceGridElement     |  has one   |  has some
      //    SmHierarchyGridElement   |  has one   |    none
      SmHierarchyElement     *pHierarchyElement     = m_pHierarchyElements->GetAt(ii) ;
      SmHierarchyGridElement &rHierarchyGridElement = pHierarchyElement->GetHierarchyGridElement() ;
      SmSurface              *pSurface              = pHierarchyElement->GetSurface() ;
      SmExtent2d             *pUVDomain             = rHierarchyGridElement.GetLeafNodeUVDomain() ;
                            
      rHierarchyGridElement.GetBoundingBox().Draw() ;
      if(pSurface) pSurface->DrawUV(8,8,FALSE,pUVDomain,TRUE) ;  // expect pUVDomain == NULL
    }

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVoxel::DrawHierarchyElements

/*******************************************************************//**
PURPOSE: Draw bounding boxes of all hierarchy elements (surfaces) 
   whose BBox intersects this Voxel's BBox.

NOTES:
***********************************************************************/
SmDisplayList * SmVoxel::DrawGridElements()
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  if(m_pElements)
    {
      for(ULONG ii=0;ii<m_pElements->GetSize();ii++)
        {
          // Different derived types store different data:
          //           DerivedType       | SurfacePtr | LeafNode Data
          //    -------------------------+------------+--------------
          //    SmSurfaceGridElement     |  has one   |   none
          //    SmHierarchyGridElement   |  has one   |  has some
          SmGridElement * pGridElement = m_pElements->GetAt(ii) ;
          SmSurface     * pSurface     = pGridElement->GetSurface() ;
          SmExtent2d    * pUVDomain    = pGridElement->GetLeafNodeUVDomain() ;
          
          pGridElement->GetBoundingBox().Draw() ;
          if(pSurface) pSurface->DrawUV(8,8,FALSE,pUVDomain,TRUE) ; // expect pUVDomain != NULL
        }
    }

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVoxel::DrawGridElements


