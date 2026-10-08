// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmShapeSolver.cpp
* PURPOSE:
**********************************************************************/

#include "StdAfx.h"

#include <SmShape.h>
#include <SmTree.h>
#include <SmCurve.h>
#include <SmSurface.h>
#include <SmSolutionArray.h>
#include <SmCacheMgrBrep.h>
#include <SmBrepCache.h>
#include <SmGraphicsExtern.h>

/*******************************************************************//**
PURPOSE: Constructor for the Shape object.  Note that a shape needs
    to be constructed all at once.  Loading a brep automatically loads
    all of its faces, edges, and vertices.  Loading a face automatically
    loads all of its edges and vertices.  Loading an edge automatically
    loads its vertices.  Therefore you do not need to duplicate the lower
    level items in the lists if they are included by a higher level item.
    For example you do not need to put a vertex into the list if the face,
    edge, or brep to which it belongs are in one of the lists already.

NOTES: The edges/faces that make up a shape must not be part
    of a composite but must own their geometry.  The reason for this is
    that we utilize the geometry as the access key.
***********************************************************************/
SmShape::SmShape
  (const SmTArray<SmBrep*>   & crBreps,      // in : add these Brep faces, edges and vertices to Shape
   const SmTArray<SmFace*>   & crFaces,      // in : add these faces and their edges and vertices to Shape
   const SmTArray<SmEdge*>   & crEdges,      // in : add these edges and their vertices to Shape
   const SmTArray<SmVertex*> & crVertices)   // in : add these vertices to Shape
{
  // locals
  ULONG ii, i, j, k ;
  SmExtent3d sBBoxUnion;
  SmTopology *pTopo = NULL;
  SmTArray<SmVertex*> sVertices;
  SmTArray<SmEdge*>   sEdges;
  SmTArray<SmFace*>   sFaces;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); for(ii=0;ii<crBreps.   GetSize();ii++) { crBreps   [ii]->Draw(TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,1,1); for(ii=0;ii<crFaces.   GetSize();ii++) { crFaces   [ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(2,3, 1,0,1); for(ii=0;ii<crEdges.   GetSize();ii++) { crEdges   [ii]->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,0); for(ii=0;ii<crVertices.GetSize();ii++) { crVertices[ii]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif

  // precompute the SmShape bounding box - for efficiecy its important to get this box size correct.
  //   composite geometry note:
  //     SmShape has two parts - a list of topology objects (vertices, edges, faces, breps)
  //                             and a list of geometry (points, curves, surfaces)
  //       Geometry is placed into spatial trees that get sent to the solvers.
  //       When working with composite geometry - complete shapes are placed in the tree.
  //       Later the solver only uses those parts of the shapes which belong to a listed topology object.
  //     So, the size of this pre-computed BBox must include the size of the composite geometry
  //       and not just the size of the geometry directly associated with the list of input
  //       topology objects.  In the code you will notice bounding boxes are computed for geometry
  //       owners.  Those topology objects get the larger composite sizes.

  // Compute the union of all of the extents of the input objects
  for (ii=0; ii<crBreps.GetSize(); ii++) 
    {
      SmBrep *pBrep = crBreps[ii];
      pTopo = pBrep->GetInfiniteRegion();
      SmExtent3d sBBox;
      pBrep->CalculateBoundingBox(sBBox);
      sBBoxUnion.Union(sBBox,sBBoxUnion);
    }

  // union in face bounding boxes
  for (ii=0; ii<crFaces.GetSize(); ii++) 
    {
      SmFace *pFace = crFaces[ii];
      pTopo = pFace;
      m_vFaces.AddUnique(pFace);
      SmExtent3d sBBox;
      SE(((SmFace *)pFace->GetSurface()->GetFace())->CalculateBoundingBox(sBBox));
      sBBoxUnion.Union(sBBox,sBBoxUnion);

      // Load edge->curves of the face
      pFace->GetEdges(sEdges);
      for (j=0; j<sEdges.GetSize(); j++) 
        {
          SmEdge *pE = sEdges[j];
          SE(((SmEdge *)pE->GetCurve()->GetOwner())->CalculateBoundingBox(&sBBox));
          sBBoxUnion.Union(sBBox,sBBoxUnion);
        }
    }

  // union in edge bounding boxes
  for (ii=0; ii<crEdges.GetSize(); ii++) 
    {
      SmEdge *pEdge = crEdges[ii];
      pTopo = pEdge;
      m_vEdges.AddUnique(pEdge);
      SmExtent3d sBBox;
      SE(((SmEdge *)pEdge->GetCurve()->GetOwner())->CalculateBoundingBox(&sBBox));
      sBBoxUnion.Union(sBBox,sBBoxUnion);
    }

  // union in vertex bounding boxes
  for (ii=0; ii<crVertices.GetSize(); ii++) 
    {
      SmVertex *pVertex = crVertices[ii];
      pTopo = pVertex;
      m_vVertices.AddUnique(pVertex);
      SmExtent3d sBBox(pVertex->GetPoint());
      sBBoxUnion.Union(sBBox,sBBoxUnion);
    }

  // find context
  const SmContext * cpContext = pTopo ? pTopo->GetContext() : NULL ;

  // signal an error for empty shapes
  if (pTopo == NULL) { SE_MSG(SM_ERR, _T("SmShape::SmShape constructor called with no Topology Objects to contain"));
                       sBBoxUnion.SetMinMax(0,0,0, 1,1,1) ; // specify BBox to be anything reasonable
                     }

  // init trees for each topology item to be stored
  m_pVertexTree  = new (*cpContext) SmTree(sBBoxUnion);
  m_pCurveTree   = new (*cpContext) SmTree(sBBoxUnion);
  m_pSurfaceTree = new (*cpContext) SmTree(sBBoxUnion);
  SM_ASSERT(m_pVertexTree  != NULL);
  SM_ASSERT(m_pCurveTree   != NULL);
  SM_ASSERT(m_pSurfaceTree != NULL);

  // Now add topology item->shapes in a unique way to the appropriate trees
  // Now load the VertexTree;

  if(   crBreps.GetSize() > 0
     || crFaces.GetSize() > 0
     || crEdges.GetSize() > 0
     || crVertices.GetSize() > 0)
    {
      // fetch and increment an unused mark
      SmNewMarkAndLock sMarkLock(cpContext, SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
      SmMarkType       eMarkType = sMarkLock.GetMarkType() ;

      for (ii=0; ii<crBreps.GetSize(); ii++) 
        {
          SmBrep *pBrep = crBreps[ii];
          // Load Brep vertices into vertex tree
          pBrep->GetVertices(sVertices);
          for (i=0; i<sVertices.GetSize(); i++) 
            {
              SmVertex *pV = sVertices[i];
              if (pV->IsMarked(eMarkType)) continue;
              m_vVertices.AddUnique(pV);
              pV->Mark(eMarkType);
              SmExtent3d sVBBox(pV->GetPoint());
              SE(m_pVertexTree->AddToSpatialTree(sVBBox,pV));
            }

          // Now load brep edges into the Edge Tree
          pBrep->GetEdges(sEdges);
          for (j=0; j<sEdges.GetSize(); j++) 
            {
              SmEdge *pE = sEdges[j];
              if (pE->IsMarked(eMarkType)) continue;
              m_vEdges.AddUnique(pE);
              pE->Mark(eMarkType);
              SmExtent3d sEBBox;
              SmCurve *pCurve = pE->GetCurve();

              // gwc: pEdge will be pE for regular edges but will be the
              //      composite edge for composite edges.
              //      Why the difference here?
              SmEdge *pEdge = SM_REINTERPRET_CAST(SmEdge*,pCurve->GetOwner());
              SE(pCurve->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox));
              SE(m_pCurveTree->AddToSpatialTree(sEBBox,pCurve));
            }

          // Now load the brep faces into the Face Tree
          pBrep->GetFaces(sFaces);
          for (k=0; k<sFaces.GetSize(); k++) 
            {
              SmFace *pF = sFaces[k];
              if (pF->IsMarked(eMarkType)) continue;
              m_vFaces.AddUnique(pF);
              pF->Mark(eMarkType);
              SmExtent3d sFBBox;
              SmSurface *pSurface = pF->GetSurface();

              // gwc: pFace will be pF for regular faces but will be the
              //      composite Face for composite Faces.
              //      Why the difference here?
              SmFace *pFace = SM_REINTERPRET_CAST(SmFace*,pSurface->GetOwner());
              SE(pFace->CalculateBoundingBox(sFBBox));
              SE(m_pSurfaceTree->AddToSpatialTree(sFBBox,pSurface));
            }
        } // end iter every Brep loading its vertices, edge->curves, and face->surfaces to trees

      // Load individual face->Surfaces
      for (ii=0; ii<crFaces.GetSize(); ii++) 
        {
          SmFace *pFace = crFaces[ii];
          if (pFace->IsMarked(eMarkType)) continue;
          pFace->Mark(eMarkType);

          // Load the face
          SmExtent3d sFBBox;
          SmSurface *pSurface = pFace->GetSurface();
      
          // gwc: pFace will be pF for regular faces but will be the
          //      composite Face for composite Faces.
          //      Why the difference here?
          SmFace *pFace2 = SM_REINTERPRET_CAST(SmFace*,pSurface->GetOwner());
          SE(pFace2->CalculateBoundingBox(sFBBox));
          SE(m_pSurfaceTree->AddToSpatialTree(sFBBox,pSurface));

          // Load edge->curves of the face
          pFace->GetEdges(sEdges);
          for (j=0; j<sEdges.GetSize(); j++) 
            {
              SmEdge *pE = sEdges[j];
              if (pE->IsMarked(eMarkType)) continue;
              m_vEdges.AddUnique(pE);
              pE->Mark(eMarkType);
              SmExtent3d sEBBox;
              SmCurve *pCurve = pE->GetCurve();
          
              // gwc: pEdge will be pE for regular edges but will be the
              //      composite edge for composite edges.
              //      Why the difference here?
              SmEdge *pEdge2 = SM_REINTERPRET_CAST(SmEdge*,pCurve->GetOwner());
              SE(pCurve->CalculateBoundingBox(pEdge2->GetInterval(),&sEBBox));
              SE(m_pCurveTree->AddToSpatialTree(sEBBox,pCurve));
            }

          // Load vertices of the face
          pFace->GetVertices(sVertices);
          for (i=0; i<sVertices.GetSize(); i++) 
            {
              SmVertex *pV = sVertices[i];
              if (pV->IsMarked(eMarkType)) continue;
              m_vVertices.AddUnique(pV);
              pV->Mark(eMarkType);
              SmExtent3d sVBBox(pV->GetPoint());
              SE(m_pVertexTree->AddToSpatialTree(sVBBox,pV));
            }

        } // end iter every face loading their vertices, edge->curves, and face->surfaces to trees

      // Now load edge objects and adjacent vertices
      for (ii=0; ii<crEdges.GetSize(); ii++) 
        {
          SmEdge *pEdge = crEdges[ii];
          if (pEdge->IsMarked(eMarkType)) continue;
          pEdge->Mark(eMarkType);
          SmExtent3d sEBBox;
          SmCurve *pCurve = pEdge->GetCurve();

          // gwc: pEdge will be pE for regular edges but will be the
          //      composite edge for composite edges.
          //      Why the difference here?
          SmEdge *pEdge2 = SM_REINTERPRET_CAST(SmEdge*,pCurve->GetOwner());
          SE(pCurve->CalculateBoundingBox(pEdge2->GetInterval(),&sEBBox));
          SE(m_pCurveTree->AddToSpatialTree(sEBBox,pCurve));

          // Load vertices of the edge
          pEdge->GetVertices(sVertices);
          for (i=0; i<sVertices.GetSize(); i++) 
            {
              SmVertex *pV = sVertices[i];
              if (pV->IsMarked(eMarkType)) continue;
              m_vVertices.AddUnique(pV);
              pV->Mark(eMarkType);
              SmExtent3d sVBBox(pV->GetPoint());
              SE(m_pVertexTree->AddToSpatialTree(sVBBox,pV));
            }
        } // end iter every edge loading their vertices and edge->curves to trees

      // Now load vertex objects
      for (ii=0; ii<crVertices.GetSize(); ii++) 
        {
          SmVertex *pVertex = crVertices[ii];
          if (pVertex->IsMarked(eMarkType)) continue;
          pVertex->Mark(eMarkType);
          SmExtent3d sVBBox(pVertex->GetPoint());
          SE(m_pVertexTree->AddToSpatialTree(sVBBox,pVertex));
        } // end iter every vertex loading each into the vertex tree
    } // end geometry to organize existence check

} // end SmShape::SmShape default constructor

/*******************************************************************//**
PURPOSE: Destructor for the Brep cache object.

NOTES: 
***********************************************************************/
SmShape::~SmShape()
{
    if (m_pVertexTree)  { delete m_pVertexTree;  m_pVertexTree  = NULL ; }
    if (m_pCurveTree)   { delete m_pCurveTree;   m_pCurveTree   = NULL ; }
    if (m_pSurfaceTree) { delete m_pSurfaceTree; m_pSurfaceTree = NULL ; }

} // end SmShape::~SmShape destructor

/*******************************************************************//**
PURPOSE: Return TRUE if target topology object is in Shape

NOTES: Only Faces, Edges, and Vertices are in the shape.
                If the shape was originally made from a Brep, that
                Brep has been broken down into its component
                faces, edges, and vertices.
***********************************************************************/
SmBoolean SmShape::IsInShape
  (SmTopology *pObj)        // in : target object to search for
 const
{
  SmBoolean bRtn = FALSE ;
  ULONG lFoundIndex ;

  // check apprpriate list for pObject Type
  if     (pObj->IsKindOf(SmFace_TYPE))   { bRtn = m_vFaces.   FindElement((SmFace*)  pObj, lFoundIndex) ; }   
  else if(pObj->IsKindOf(SmEdge_TYPE))   { bRtn = m_vEdges.   FindElement((SmEdge*)  pObj, lFoundIndex) ; }   
  else if(pObj->IsKindOf(SmVertex_TYPE)) { bRtn = m_vVertices.FindElement((SmVertex*)pObj, lFoundIndex) ; }

  // all done
  return(bRtn) ;

} // end SmShape::~SmShape destructor

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmShape::IsKindOf( SM_TYPE t ) const
{
  return ((SmShape_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump a shape - does nothing for now

NOTES: 
***********************************************************************/
void SmShape::Dump(void) const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // output entity counts
  SM_SPRINTF(sBuff,       
             _T("\nSmShape = 0x%p,  # Faces = %ld, # Edges = %ld, # Vertices = %ld\n"),
             this, m_vFaces.GetSize(),m_vEdges.GetSize(),m_vVertices.GetSize());
  SM_SPRINTF(sBuffForFile,
             _T("\nBrep = %s,  # Faces = %ld, # Edges = %ld, # Vertices = %ld\n"),
             _T("notNULL"), m_vFaces.GetSize(),m_vEdges.GetSize(),m_vVertices.GetSize());
  smos_WriteBuffer(sBuff, sBuffForFile);

  // output the spatial trees
  if(m_pVertexTree)  { smos_WriteBuffer(_T("\nVertex Tree -")) ;
                       m_pVertexTree->Dump() ;
                     }
  if(m_pCurveTree)   { smos_WriteBuffer(_T("\n\nCurve Tree -")) ;
                       m_pCurveTree->Dump() ;  
                     }
  if(m_pSurfaceTree) { smos_WriteBuffer(_T("\n\nSurface Tree -")) ;
                       m_pSurfaceTree->Dump() ;
                     }

} // end SmShape::Dump

/*******************************************************************//**
PURPOSE: Draw a shape - does nothing for now

NOTES: 
***********************************************************************/
void SmShape::Draw() const
{ 

} // end SmShape::Draw

