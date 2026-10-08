// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBrepCache.cpp
* PURPOSE:
**********************************************************************/

 
#include "StdAfx.h"

#include <SmBrepCache.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmTree.h>
#include <SmPoly.h>
#include <SmEdge.h>
// Remove Composites
// #include <SmCEdge.h>
#include <SmFace.h>
// Remove Composites
// #include <SmCFace.h>

/*******************************************************************//**
PURPOSE: Constructor for the Brep cache object.

NOTES: 
***********************************************************************/
SmBrepCache::SmBrepCache
  (const SmSAGObject * cpBrepOrPolyBrep) // in : Brep or PolyBrep target Object
:  m_cpBrep(NULL),
   m_cpPolyBrep(NULL),
   m_pVertexTree(NULL), 
   m_pCurveTree(NULL), 
   m_pSurfaceTree(NULL)
{
  // see if object is a brep
  m_cpBrep = SM_CAST_PTR(SmBrep,cpBrepOrPolyBrep);

#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; 
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  if(m_cpBrep) 
    { 
      lCount++ ;
      ((SmBrep *)m_cpBrep)->m_lBrepCacheCount++ ; 
      smos_sprintf(sBuff,_T("\nConstructed Brep Cache [Brep = 0x%p, rep = %ld] iter: %ld"), 
              m_cpBrep, m_cpBrep->m_lBrepCacheCount, lCount) ;
      MYPRINTF(sBuff) ;
    }
#endif

  // if Object is a Brep store it in m_cpBrep
  if (m_cpBrep == NULL) 
    {
      // else if Object is a PolyBrep store it in m_cpPolyBrep
      m_cpPolyBrep = SM_CAST_PTR(SmPolyBrep,cpBrepOrPolyBrep);

      if (m_cpPolyBrep == NULL) 
        {
          // else its neither - signal an error
          SE(SM_ERR);
        } // end not a PolyBrep check
    } // end not a Brep check

} // end SmBrepCache::SmBrepCache constructor

/*******************************************************************//**
PURPOSE: Destructor for the Brep cache object.

NOTES: 
***********************************************************************/
SmBrepCache::~SmBrepCache()
{
#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  smos_sprintf(sBuff,_T("\nDestructed Brep Cache [Brep = 0x%p] iter: %ld"), 
          m_cpBrep, lCount) ;
  MYPRINTF(sBuff) ;
#endif

    if (m_pVertexTree)  { delete m_pVertexTree; m_pVertexTree = NULL ; }
    if (m_pCurveTree)   { delete m_pCurveTree; m_pCurveTree = NULL ; }
    if (m_pSurfaceTree) { delete m_pSurfaceTree; m_pSurfaceTree = NULL ; }

} // end SmBrepCache::~SmBrepCache destructor

/*******************************************************************//**
PURPOSE: Make the trees of a set of SmPolyFace's.

NOTES: 
***********************************************************************/
SmStatus SmBrepCache::BuildPolyTrees(SmTArray<SmPolyFace*> & rPolyFaces)
{

    SM_ASSERT(m_pVertexTree == NULL);
    SM_ASSERT(m_pCurveTree == NULL);
    SM_ASSERT(m_pSurfaceTree == NULL);

    SmExtent3d sBBox;
    SmTArray<SmPolyVertex*> sVertices;
    SmTArray<SmPolyEdge*> sEdges;
    for (ULONG i=0; i<rPolyFaces.GetSize(); i++) {
        SmPolyFace * pFace = rPolyFaces[i];
        // Calculate bounding box of face
        SmExtent3d sFaceBBox;
        pFace->CalculateBoundingBox(sFaceBBox);
        if (i==0) { sBBox = sFaceBBox; }
        else { sBBox.Union(sFaceBBox,sBBox); }
        // Collect all edges
        SmTArray<SmPolyEdge*> sFaceEdges;
        pFace->GetPolyEdges(sFaceEdges);
        sEdges.Append(sFaceEdges);
    }
    double dBBoxSize = sBBox.GetSize().Length();
    sBBox.ExpandAbsolute(0.1*dBBoxSize);

    SmTree *pVertexTree = new (*GetContext()) SmTree(sBBox);
    pVertexTree->SetOwnerObject(this) ;
    SmObjDelete sClean1(pVertexTree);
   
    SmTree *pCurveTree = new (*GetContext()) SmTree(sBBox);
    pCurveTree->SetOwnerObject(this) ;
    SmObjDelete sClean2(pCurveTree);
    
    SmTree *pSurfaceTree = new (*GetContext()) SmTree(sBBox);
    pSurfaceTree->SetOwnerObject(this) ;
    SmObjDelete sClean3(pSurfaceTree);

    // Now load the Curves owned by single Edges 
    for (ULONG j=0; j<sEdges.GetSize(); j++) {
        SmPolyEdge *pE = sEdges[j];
        SmExtent3d sEBBox;
        SER(pE->CalculateBoundingBox(sEBBox));
        sEBBox.ExpandAbsolute(pE->GetTolerance());
        SER(pCurveTree->AddToSpatialTree(sEBBox,pE,SM_NG_LINE_SEG));

        // Load the VertexTree;
        SmPolyVertex *pStartV = pE->GetStartPolyVertex();
        SmExtent3d sPBBox1(pStartV->GetPoint());
        sPBBox1.ExpandAbsolute(pStartV->GetTolerance());
        SER(pVertexTree->AddToSpatialTree(sPBBox1,pStartV));
        SmPolyVertex *pEndV = pE->GetEndPolyVertex();
        SmExtent3d sPBBox2(pEndV->GetPoint());
        sPBBox2.ExpandAbsolute(pEndV->GetTolerance());
        SER(pVertexTree->AddToSpatialTree(sPBBox2,pEndV));
    }
    // Now load the FaceTree;
    for (ULONG k=0; k<rPolyFaces.GetSize(); k++) {
        SmPolyFace *pFace = rPolyFaces[k];
        SmExtent3d sFBBox;
        SER(pFace->CalculateBoundingBox(sFBBox));
        SER(pSurfaceTree->AddToSpatialTree(sFBBox,pFace,SM_NG_POLYGON));
    }
    // Set output and return
    sClean1.Clear();
    sClean2.Clear();
    sClean3.Clear();
    m_pVertexTree = pVertexTree;
    m_pCurveTree = pCurveTree;
    m_pSurfaceTree = pSurfaceTree;

    // all done
    return SM_SUCCESS;

} // end SmBrepCache::BuildPolyTrees


/*******************************************************************//**
PURPOSE: Make the trees of the Brep Cache.

  The SmBrepCache contains 3 spatial trees of bounding boxes
  for:  1. vertices,  m_pVertexTree
        2. curves,    m_pCurveTree
        3. surfaces,  m_pSurfaceTree

NOTES: 
***********************************************************************/
SmStatus SmBrepCache::BuildTrees()
{
  // check state
  SM_ASSERT(   m_cpBrep     != NULL 
            || m_cpPolyBrep != NULL);

  SM_ASSERT(m_pVertexTree  == NULL);
  SM_ASSERT(m_pCurveTree   == NULL);
  SM_ASSERT(m_pSurfaceTree == NULL);

  // get a containing bounding box
  //  for SmBrep     Objects: box is Centered on and twice the size of the SmBrep BoundingBox
  //  for SmPolyBrep Objects: box is the SmPolyBrep BoundingBox
  SmExtent3d sBrepBBox;

  // when CacheObject's Object is a Brep
  if (m_cpBrep) // SmBrep 
    {
      // Build Box centered on and larger than the Brep Bounding Box
      SER(m_cpBrep->CalculateBoundingBox(sBrepBBox));
      sBrepBBox.ExpandAbsolute(m_cpBrep->GetTolerance()*100.0);
      SmVector3d sSize = sBrepBBox.GetSize();
      sBrepBBox.AddPoint3d(sBrepBBox.GetMax()+0.15*sSize);
      sBrepBBox.AddPoint3d(sBrepBBox.GetMin()-0.15*sSize);
    } // end SmBrep branch
  else // SmPolyBrep (and m_cpBrep == NULL)
    { // Build Box centered on and larger than the Brep Bounding Box
      SER(m_cpPolyBrep->CalculateBoundingBox(sBrepBBox));
      sBrepBBox.ExpandAbsolute(m_cpPolyBrep->GetTolerance()*100.0);
    } // end SmPolyBrep Branch


  // allocate and init Vertex Tree 
  SmTree *pVertexTree  = new (*GetContext()) SmTree(sBrepBBox);
  pVertexTree->SetOwnerObject(this) ;
  SmObjDelete sClean1(pVertexTree);

  // allocate and init CurveTree (pNode->m_eGeomType == SM_NG_LINE_SEG)
  SmTree *pCurveTree   = new (*GetContext()) SmTree(sBrepBBox);
  pCurveTree->GetTopNode()->m_eGeomType = SM_NG_LINE_SEG;
  pCurveTree->SetOwnerObject(this) ;
  SmObjDelete sClean2(pCurveTree);

  // allocate and init SurfaceTree (pNode->m_eGeomType == SM_NG_POLYGON)
  SmTree *pSurfaceTree = new (*GetContext()) SmTree(sBrepBBox);
  pSurfaceTree->GetTopNode()->m_eGeomType = SM_NG_POLYGON;
  pSurfaceTree->SetOwnerObject(this) ;
  SmObjDelete sClean3(pSurfaceTree);

  if (m_cpPolyBrep) 
    {
      // Now load the PolyBrep VertexTree;
      SmTArray<SmPolyVertex*> sVertices;
      m_cpPolyBrep->GetPolyVertices(sVertices);
      for (ULONG i=0; i<sVertices.GetSize(); i++) 
        {
          SmPolyVertex *pV = sVertices[i];
          SmExtent3d sVBBox(pV->GetPoint());
          sVBBox.ExpandAbsolute(pV->GetTolerance());
          SER(pVertexTree->AddToSpatialTree(sVBBox,pV));
        }

      // Now load the PolyBrep Curves owned by single Edges 
      SmTArray<SmPolyEdge*> sEdges;
      m_cpPolyBrep->GetPolyEdges(sEdges);
      for (ULONG j=0; j<sEdges.GetSize(); j++) 
        {
          SmPolyEdge *pE = sEdges[j];
          SmExtent3d sEBBox;
          SER(pE->CalculateBoundingBox(sEBBox));
          sEBBox.ExpandAbsolute(pE->GetTolerance());
          SER(pCurveTree->AddToSpatialTree(sEBBox,pE,SM_NG_LINE_SEG));
        }

      // Now load the PolyBrep FaceTree;
      SmTArray<SmPolyFace*> sPolyFaces;
      m_cpPolyBrep->GetPolyFaces(sPolyFaces);
      for (ULONG k=0; k<sPolyFaces.GetSize(); k++) 
        {
          SmPolyFace *pFace = sPolyFaces[k];
          SmExtent3d sFBBox;
          SER(pFace->CalculateBoundingBox(sFBBox));
          SER(pSurfaceTree->AddToSpatialTree(sFBBox,pFace,SM_NG_POLYGON));
        }

      // Set output and return
      sClean1.Clear();
      sClean2.Clear();
      sClean3.Clear();
      m_pVertexTree  = pVertexTree;
      m_pCurveTree   = pCurveTree;
      m_pSurfaceTree = pSurfaceTree;

      // all done
      return SM_SUCCESS;

    } // end SmPolyBrep Object Check

  // arrive here for SmBreps

  // Now load the SmBrep VertexTree;
  SmTArray<SmVertex*> sVertices;
  m_cpBrep->GetVertices(sVertices);
  for (ULONG i=0; i<sVertices.GetSize(); i++) 
    {
      SmVertex *pV = sVertices[i];
      SmExtent3d sVBBox(pV->GetPoint());
      sVBBox.ExpandAbsolute(pV->GetTolerance());
      SER(pVertexTree->AddToSpatialTree(sVBBox,pV));
    } 

  // Now load the SmBrep Curves owned by single Edges 
  SmTArray<SmEdge*> sEdges;
  m_cpBrep->GetEdges(sEdges);
  for (ULONG j=0; j<sEdges.GetSize(); j++) 
    {
      SmEdge *pE = sEdges[j];
      SmExtent3d sEBBox;
      SmCurve *pCurve = pE->GetCurve();

      // when curve is owned by this edge (else its owned by a CEdge)
      if (pCurve->GetOwner() == pE) 
        {
          SER(pCurve->CalculateBoundingBox(pE->GetInterval(),&sEBBox));
          sEBBox.ExpandAbsolute(pE->GetTolerance());
          SER(pCurveTree->AddToSpatialTree(sEBBox,pCurve));

        } // end curve is owned by this Edge check
    } // end iter every edge

// Remove Composites
//  // Now load SmBrep curves owned by composite edges
//  SmTArray<SmCEdge*> sCEdges;
//  m_cpBrep->GetCEdges(sCEdges);
//  for (ULONG jj=0; jj<sCEdges.GetSize(); jj++) 
//    {
//      SmCEdge *pCE = sCEdges[jj];
//      SmExtent3d sCEBBox;
//      SmCurve *pCurve = pCE->GetCurve();
//      SER(pCurve->CalculateBoundingBox(pCE->GetInterval(),&sCEBBox));
//      sCEBBox.ExpandAbsolute(pCE->GetTolerance());
//      SER(pCurveTree->AddToSpatialTree(sCEBBox,pCurve));
//    }

  // Now load the SmBrep Surfaces owned by single Faces
  SmTArray<SmFace*> sFaces;
  m_cpBrep->GetFaces(sFaces);
  for (ULONG k=0; k<sFaces.GetSize(); k++) 
    {
      SmFace *pF = sFaces[k];
      SmExtent3d sFBBox;
      SmSurface *pSurface = pF->GetSurface();

      // when surface is owned by this face (else its owned by some CFace)
      if (pSurface->GetOwner() == pF) 
        {
          SER(pF->CalculateBoundingBox(sFBBox));
          sFBBox.ExpandAbsolute(pF->GetTolerance());
          SER(pSurfaceTree->AddToSpatialTree(sFBBox,pSurface));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetColor(1,0,0);
              pF->Draw();
              sm_GraphicsLoop();
              smgfx_SetColor(0,0,0);
              Draw();
            }
#endif
        } // end surface is owned by Face check
    } // end iter every face

// Remove Composites
//  // Now load the SmBrep Surfaces owned by Composite Faces
//  SmTArray<SmCFace*> sCFaces;
//  m_cpBrep->GetCFaces(sCFaces);
//  for (ULONG kk=0; kk<sCFaces.GetSize(); kk++) 
//    {
//      SmCFace *pCF = sCFaces[kk];
//      SmExtent3d sCFBBox;
//      SmSurface *pSurface = pCF->GetSurface();
//      SER(pSurface->CalculateBoundingBox(pCF->GetUVDomain(),&sCFBBox));
//      sCFBBox.ExpandAbsolute(pCF->GetTolerance());
//      SER(pSurfaceTree->AddToSpatialTree(sCFBBox,pSurface));
//    }

  // Set output and return
  sClean1.Clear();
  sClean2.Clear();
  sClean3.Clear();
  m_pVertexTree  = pVertexTree;
  m_pCurveTree   = pCurveTree;
  m_pSurfaceTree = pSurfaceTree;

  // all done
  return SM_SUCCESS;

} // end SmBrepCache::BuildTrees

/*******************************************************************//**
PURPOSE: Size the Spatial Tree Bounding Boxes to include all
  the bounding boxes of the included items.

NOTES: 
  1. The bounding edges and vertices of the input faces and the bounding
     vertices of edges are automatically checked. So when working with just
     faces or edges there is no need to add their boundary geometry to the
     input arrays.
  2. Composite Geometry.  When working with composite geometry, entire curves
     and surfaces are placed into the Spatial Trees, not just the sub-domain
     that applies to a specific face or edge.  So in the code you will
     see that one works with face->GetSurface()->GetOwner() and
     edge->GetCurve()->GetOwner() to make sure that we work with composite
     faces and edges and not one of their components.

NOTE --- Increasing the size of a Spatial Tree is done
 by deleting the old tree and resizing a new one.  It's expensive.
 To avoid doing this multiple times, its best to make a list of all
 the objects to be placed in the Brep Cache, get the union of their
 bounding boxes, and then change the Spatial Tree bounding box size
 just once.
***********************************************************************/
SmStatus SmBrepCache::ReSizeTrees
  (SmTArray<SmFace*> *pFaces,       // in : array of faces to size m_pSurfaceTree.
                                    //       face->Edges size m_pCurveTree, and
                                    //       face->Vertices size m_pVertexTree.
   SmTArray<SmEdge*> *pEdges,       // in : array of edges to size m_pCurveTree.
                                    //       edge->Vertices size m_pVertexTree.
   SmTArray<SmVertex*> *pVertices)  // in : array of vertices to size m_pVertexTree.
{
  // locals
  ULONG ii, jj ;
  SmFace   *pFace ;
  SmEdge   *pEdge ;
  SmVertex *pVertex ;
  SmTArray<SmEdge *> sEdges ;
  SmExtent3d sSurfaceBox, sCurveBox ;
  SmExtent3d sSurfaceUnionBox, sCurveUnionBox, sVertexUnionBox ; 
  double dTol = 0.0 ;

  SM_ASSERT(m_pVertexTree  != NULL);
  SM_ASSERT(m_pCurveTree   != NULL);
  SM_ASSERT(m_pSurfaceTree != NULL);

  // Faces 
  for(ii=0;pFaces && ii<pFaces->GetSize();ii++)
    {
      pFace = pFaces->GetAt(ii) ;

      // get union bounding box for all face->surfaces
      ((SmFace *)pFace->GetSurface()->GetFace())->CalculateBoundingBox(sSurfaceBox) ;
      sSurfaceUnionBox.Union(sSurfaceBox, sSurfaceUnionBox) ;

      // get max tolerance
      if(pFace->GetTolerance() > dTol) dTol = pFace->GetTolerance() ;

      // for every face->edge and face->edge->vertex
      pFace->GetEdges(sEdges) ;
      for(jj=0;jj<sEdges.GetSize();jj++)
        {
          pEdge = sEdges[jj] ;
          if(pEdge->GetTolerance() > dTol) dTol = pEdge->GetTolerance() ;

          // get union bounding box for all face->edge->Curves
          ((SmEdge*)pEdge->GetCurve()->GetEdge())->CalculateBoundingBox(&sCurveBox) ;
          sCurveUnionBox.Union(sCurveBox, sCurveUnionBox) ;

          // get union bounding box for all face->vertices
          pVertex = pEdge->GetVertex() ;
          if(pVertex->GetTolerance() > dTol) dTol = pVertex->GetTolerance() ;
          sVertexUnionBox.AddPoint3d(pVertex->GetPoint()) ;

          pVertex = pEdge->GetOtherVertex(pVertex) ;
          if(pVertex->GetTolerance() > dTol) dTol = pVertex->GetTolerance() ;
          sVertexUnionBox.AddPoint3d(pVertex->GetPoint()) ;
        }

    } // end iter every face

  // Edges
  for(ii=0;pEdges && ii<pEdges->GetSize();ii++)
    {
      pEdge = pEdges->GetAt(ii) ;
      if(pEdge->GetTolerance() > dTol) dTol = pEdge->GetTolerance() ;

       // get union bounding box for all edge->curves
      ((SmEdge *)pEdge->GetCurve()->GetEdge())->CalculateBoundingBox(&sCurveBox) ;
      sCurveUnionBox.Union(sCurveBox, sCurveUnionBox) ;

      // get union bounding box for all edge->vertices
      pVertex = pEdge->GetVertex() ;
      if(pVertex->GetTolerance() > dTol) dTol = pVertex->GetTolerance() ;
      sVertexUnionBox.AddPoint3d(pVertex->GetPoint()) ;

      pVertex = pEdge->GetOtherVertex(pVertex) ;
      if(pVertex->GetTolerance() > dTol) dTol = pVertex->GetTolerance() ;
      sVertexUnionBox.AddPoint3d(pVertex->GetPoint()) ;

    } // end iter every edge

  // Vertices
  for(ii=0;pVertices && ii<pVertices->GetSize();ii++)
    {
      pVertex = pVertices->GetAt(ii) ;

      // get union bounding box for all Vertices
      if(pVertex->GetTolerance() > dTol) dTol = pVertex->GetTolerance() ;
      sVertexUnionBox.AddPoint3d(pVertex->GetPoint()) ;
    
    } // end iter every vertex

  
  // arrive here after bbox unions are complete - now update tree sizes as needed

  // increase pSurfaceTree BBox 
  if(FALSE == sSurfaceUnionBox.HasNegativeVolume())
    {
      sSurfaceUnionBox.ExpandAbsolute(100.0*dTol) ;
      if(FALSE == sSurfaceUnionBox.IsContainedBy(m_pSurfaceTree->GetTopNode()->GetBoundingBox()))
        { 
          m_pSurfaceTree->AddToSpatialTree(sSurfaceUnionBox, NULL) ; 
        }
    }

  // increase pCurveTree BBox 
  if(FALSE == sCurveUnionBox.HasNegativeVolume())
    {
      sCurveUnionBox.ExpandAbsolute(100.0*dTol) ;
      if(FALSE == sCurveUnionBox.IsContainedBy(m_pCurveTree->GetTopNode()->GetBoundingBox()))
        { 
          m_pCurveTree->AddToSpatialTree(sCurveUnionBox, NULL) ; 
        }
    }

  // increase pVertexTree BBox
  if(FALSE == sVertexUnionBox.HasNegativeVolume()) 
    {
      sVertexUnionBox.ExpandAbsolute(100.0*dTol) ;
      if(FALSE == sVertexUnionBox.IsContainedBy(m_pVertexTree->GetTopNode()->GetBoundingBox(), SM_EFF_ZERO))
        { 
          m_pVertexTree->AddToSpatialTree(sVertexUnionBox, NULL) ; 
        }
    }

  // all done
  return(SM_SUCCESS) ;


} // end SmBrepCache::BuildTrees


/*******************************************************************//**
PURPOSE: Get the composite bounding box of all of the subentities of
   the Brep cache.

NOTES: 
***********************************************************************/
SmStatus SmBrepCache::GetBBox(SmExtent3d & rBBox) const
{
  // locals
  SmTArray<SmObjectList*> sObjects(256);
  SmExtent3d sUnion;

  // get BRep Vertex list
  SER(m_pVertexTree->GetObjectList(sObjects));

  // Give Empty Breps a bounding box at the origin out to 1,1,1
  if (sObjects.GetSize() == 0) 
    {
      rBBox.AddPoint3d(SmPoint3d(0,0,0));
      rBBox.AddPoint3d(SmPoint3d(1,1,1));
      return SM_SUCCESS;
    }

  // vertices
  ULONG i;
  sUnion = sObjects[0]->m_sBBox;
  for (i=1; i<sObjects.GetSize(); i++) 
    {
      SmObjectList *pObjList = sObjects[i];
      sUnion.Union(pObjList->m_sBBox,sUnion);
    }

  // curves
  SER(m_pCurveTree->GetObjectList(sObjects));
  for (i=0; i<sObjects.GetSize(); i++) 
    {
      SmObjectList *pObjList = sObjects[i];
      sUnion.Union(pObjList->m_sBBox,sUnion);
    }

  // surfaces
  SER(m_pSurfaceTree->GetObjectList(sObjects));
  for (i=0; i<sObjects.GetSize(); i++) 
    {
      SmObjectList *pObjList = sObjects[i];
      sUnion.Union(pObjList->m_sBBox,sUnion);
    }

  // set output
  rBBox = sUnion;

  // all done
  return SM_SUCCESS;

} // end SmBrepCache::GetBBox

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the Brep cache.

NOTES: 
***********************************************************************/
ULONG SmBrepCache::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated)   // out: bigger size of all allocated memory in bytes
  const
{
   ULONG lVertexTreeAlloc=0, lCurveTreeAlloc=0, lSurfaceTreeAlloc=0 ;

   ULONG lUsed =   sizeof(SmBrepCache) 
                 + m_pVertexTree-> GetMemoryUsed(lVertexTreeAlloc) 
                 + m_pCurveTree->  GetMemoryUsed(lCurveTreeAlloc) 
                 + m_pSurfaceTree->GetMemoryUsed(lSurfaceTreeAlloc); 

   // set output
   rlMemoryAllocated =  sizeof(SmBrepCache)
                      + lVertexTreeAlloc  
                      + lCurveTreeAlloc   
                      + lSurfaceTreeAlloc ;

   // all done
   return lUsed ;

} // end SmBrepCache::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Display the surface, edge, and vertex trees

NOTES: 
***********************************************************************/

void SmBrepCache::Draw
  (int DrawFlag)         // orof: 1=draw vertex tree bounding box hierarchy
                         //       2=draw curve tree
                         //       4=draw surface tree
  const
{
  // when asked draw the vertex, curve, and surface tree bounding box hierarchys
  if( (DrawFlag & 1) && m_pVertexTree  != NULL )
      m_pVertexTree->DrawNodeBoundingBoxes( 0,.4,.4,  0,.1, 0);
  if( (DrawFlag & 2) && m_pCurveTree   != NULL )
      m_pCurveTree->DrawNodeBoundingBoxes(.4,.4, 0, .1, 0, 0);
  if( (DrawFlag & 4) && m_pSurfaceTree != NULL )
      m_pSurfaceTree->DrawNodeBoundingBoxes(.4, 0,.4,  0, 0,.1);

} // end SmBrepCache::Draw

/*******************************************************************//**
PURPOSE: 
NOTES:
***********************************************************************/
void SmBrepCache::Dump(void) const
{
    smos_WriteBuffer(_T("SmBrepCache\n"));
}

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmBrepCache::IsKindOf( SM_TYPE t ) const 
{ 
  return ((SmBrepCache_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) )); 
}
