// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmShape.h
* PURPOSE: Header file for SmShape object.
**********************************************************************/

#ifndef __SMSHAPE_H__
#define __SMSHAPE_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

/*******************************************************************//**
PURPOSE: This class is used to represent a shape object.  A shape
   is a collection of topology objects such as edges, faces, and  vertices.
   The topology objects can come from different Breps or all from the 
   same Brep.  You don't have to include all topology items of a single
   Brep.

NOTES:
   When a Brep is added to a shape, all its faces, edges, and vertices
        are automatically added as well. 
   When a Face is added to a shape, all its edges and vertices
        are automatically added as well.
   When an Edge is added to a shape, its vertices
        are automatically added as well.

***********************************************************************/
class SM_EXPORT SmShape : public SmObject
{
  friend class SmTree;
private:
  SmTree            * m_pVertexTree;      // Spatial tree of all vertices     
  SmTree            * m_pCurveTree;       // Spatial tree of all edge->curves     
  SmTree            * m_pSurfaceTree;     // Spatial tree of all face->surfaces   
    
  SmTArray<SmFace*>   m_vFaces;           // Shape face list     
  SmTArray<SmEdge*>   m_vEdges;           // Shape edge list - augmented with all face->edges     
  SmTArray<SmVertex*> m_vVertices;        // Shape vertex list - augemented with all face->edges->vertices and edge->vertices     

public:
  // constructor/destructor - increments an unused Mark value
  SmShape(const SmTArray<SmBrep*>   & crBreps,     // in : add these Brep faces, edges and vertices to Shape    
          const SmTArray<SmFace*>   & crFaces,     // in : add these faces and their edges and vertices to Shape
          const SmTArray<SmEdge*>   & crEdges,     // in : add these edges and their vertices to Shape          
          const SmTArray<SmVertex*> & crVertices); // in : add these vertices to Shape                          
  virtual ~SmShape();

  // simple data access
  SmTree * GetVertexTree()                          const { return m_pVertexTree; }
  SmTree * GetCurveTree()                           const { return m_pCurveTree; }
  SmTree * GetSurfaceTree()                         const { return m_pSurfaceTree; }
  void GetEdges(SmTArray<SmEdge*> & rEdges)         const { rEdges.ReSet(); 
                                                            rEdges.Append(m_vEdges); 
                                                          }
  void GetFaces(SmTArray<SmFace*> & rFaces)         const { rFaces.ReSet(); 
                                                            rFaces.Append(m_vFaces); 
                                                          }
  void GetVertices(SmTArray<SmVertex*> & rVertices) const { rVertices.ReSet();  
                                                            rVertices.Append(m_vVertices);
                                                          }
  SmBoolean IsInShape(SmTopology *pObj)             const ;

  // Defines GetType(), IsKindOf(), virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmShape,SmObject,SmShape_TYPE);
  virtual void Draw(void) const;

} ; // end class SmShape

#endif // !__SMSHAPE_H__

