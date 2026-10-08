// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBrepCache.h
* PURPOSE: Header file for SmBrepCache object.
**********************************************************************/

#ifndef __SmBrepCache_H__
#define __SmBrepCache_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMSAGOBJECT_H__
#include <SmSAGObject.h>
#endif

#ifndef __SMCACHEMGR_H__
#include <SmCacheMgr.h>
#endif

/*******************************************************************//**
PURPOSE: The Brep cache object contains a spatial decomposition of 
    the vertices, edges and faces of the Brep.

NOTES: 
***********************************************************************/
class SM_EXPORT SmBrepCache : public SmCacheObj
{
    friend class SmTree;               
private:                               
    const SmBrep     * m_cpBrep;       // NotNULL when associated object is a Brep, m_cpPolyBrep is NULL
    const SmPolyBrep * m_cpPolyBrep;   // NotNULL when associated object is a PolyBrep, m_cpBrep is NULL

    SmTree           * m_pVertexTree;  // A spatialTree of vertex  bounding boxes
    SmTree           * m_pCurveTree;   // A spatialTree of curve   bounding boxes
    SmTree           * m_pSurfaceTree; // A spatialTree of surface bounding boxes

public:
    SmBrepCache(const SmSAGObject * cpBrepOrPolyBrep);
    virtual ~SmBrepCache();

    SmBrep     * GetBrep() const        { return (SmBrep *)m_cpBrep; }
    SmTree     * GetVertexTree()  const { return m_pVertexTree; }
    SmTree     * GetCurveTree()   const { return m_pCurveTree; }
    SmTree     * GetSurfaceTree() const { return m_pSurfaceTree; }

    SmStatus     BuildPolyTrees(SmTArray<SmPolyFace*> & rPolyFaces);

    SmStatus BuildTrees();

    SmStatus GetBBox(SmExtent3d & crBBox) const;

    // resize the SpatialTree Bounding boxes to the union of their current size and the input args.
    SmStatus ReSizeTrees
    (
      SmTArray<SmFace*> *pFaces, 
      SmTArray<SmEdge*> *pEdges, 
      SmTArray<SmVertex*> *pVertices
    ) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmBrepCache,SmObject,SmBrepCache_TYPE);
    
    ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ;

    virtual void Draw(int DrawFlag=7) const;  // orof: 1=draw vertex tree bounding box hierarchy
                                              //       2=draw curve tree
                                              //       4=draw surface tree
     
} ; // end class SmBrepCache

#endif // !__SmBrepCache_H__


