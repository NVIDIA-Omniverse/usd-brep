// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTrimSrfCache.h
* PURPOSE: Header file for SmTrimSrfCache object.
**********************************************************************/

#ifndef __SMTRIMSRFCACHE_H__
#define __SMTRIMSRFCACHE_H__

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
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

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

#ifndef __SMPOLARBOX_H__
#include <SmPolarBox.h>
#endif

#ifndef __SMTREE_H__
#include <SmTree.h>
#endif

#ifndef __SMTOPO_TYPES_H_
#include <SmTopoTypes.h>
#endif

#ifndef __SMSURFACECACHE_H__
#include <SmSurfaceCache.h>
#endif

class SmPlane ;  // for Draw() method

/*******************************************************************//**
PURPOSE: This object is the high level object which contains the
   decompostion of a trimmed surface.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmTrimSrfCache : public SmSurfaceCache
{
    friend class SmTree;
private:

public:
    SmTrimSrfCache
    (
      const SmSurface & crSurface,
      double            dChordHeightTolerance = 0.0,
      double            dAngleTolDeg          = 0.0,
      double            dAspectRatio3D        = 0.0,
      double            dMaxSideLength3D      = 0.0,
      double            dMinSideLength3D      = 0.0,
      double            dMinSideLengthRatioUV = 0.001
    );

    virtual ~SmTrimSrfCache();

    // build SmSurfaceCache Subdivision tree then
    virtual SmStatus BuildTree(SmMemBlockMgr *pOptBezierBlock=NULL);

    virtual SmStatus PointTest
    (
      const SmPoint2d & crUVPoint,                ///< [in ]: UVPoint to test                                            <br>
      SmBoolean       & rbPointIsOk,              ///< [out]: TRUE = PointTest is off,                                   <br>
                                                  ///<      :        node is in/on face boundary                         <br>
                                                  ///<      :        or test cannot be made                              <br>
                                                  ///<      : FALSE= Point is outside of face boundary                   <br>
      SmZoneTol3d     * pOptSrcZoneTol3d = NULL,  ///< [in ]: Obj ZoneTol3d assoc with crUVPoint,                        <br>
      SmObject        **pOptObject = NULL         ///< [out]: Optional Pointer to Topology Object coincident with point  <br>
    ) const;

    virtual SmStatus PointClassify
    (
      const SmPoint2d           & crUVPoint,       ///< [in ]: Point to classify                                          <br>
      SmZoneTol3d                 sSrcZoneTol3d,   ///< [in ]: Obj ZoneTol3d assoc with UVPoint, not this face            <br>
                                                   ///<      : (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL)   <br>
      SmPointClassificationType & eClassification, ///< [out]: Type of object coincident with point                       <br>
      SmObject                 *& rpObjectInOrOn   ///<      : oneof SM_PC_VERTEX                                         <br>
                                                   ///<      :       SM_PC_EDGE                                           <br>
                                                   ///<      :       SM_PC_FACE                                           <br>
                                                   ///<      :       SM_PC_UNKNOWN                                        <br>
                                                   ///< [out]: object coincident with point
    )  const;

    ULONG    GetMemoryUsed(ULONG &rlMemoryAllocated) const ;

    // only called by SmTrimSrfCache::BuildTree (directly and through SubdivideNode)
    virtual SmStatus ImplantEdgeuse
    (
      SmEdgeuse * pEdgeuse,                         ///< [in ]: geometry to embed into surface spatial decomposition                 <br>
      SmBoolean bDoSubdivision,                     ///< [in ]: TRUE  = Split nodes with large uv-domains and classify the children  <br>
                                                    ///<      : FALSE = don't split nodes - just classify them as they are           <br>
      SmExtent2d & rFaceDomain,                     ///< [out]: expanded to include the uv_domain of all                             <br>
                                                    ///<      : nodes classified as on the boundary                                  <br>
      SmTreeNode *pOptTreeNode = NULL               ///< [in ]: when given, only implant edgeuse in this node                        <br>
                                                    ///<      : default:[NULL]                                                       <br>
    );

    // only called by SmTrimSrfCache::BuildTree (through SubdivideNode)
    virtual SmStatus ImplantPolyEdge
    (
      SmPolyEdge* pPolyEdge,                        ///< [in ]: PolyEdge to embed into surface spatial decomposition                <br>
      SmTreeNode* pNode                             ///< [in ]: Node to embed the above PolyEdge into                               <br>                                                    
    );

    SmStatus ImplantVertexuse(SmVertexuse * pVertexuse, SmExtent2d & rFaceDomain);

    // inherited from SmSurfaceCache 
    // // Draw UV Subdivision boundares in z=0 plane
    // SmDisplayList * DrawSubdivision2D
    //   (SmBoolean       bDrawGeometry=FALSE,              ///< [in ]: TRUE= Also draw Face->UVTrimCurves and UVVertexPts on plane            
    //    SmBoolean       bOnlyDrawNodesWithVertices=FALSE, ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face vertices          
    //    SmBoolean       bOnlyDrawNodesWithEdges=FALSE,    ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face edges
    //    SmPlane       * pOptOutPlane=NULL,                ///< [in ]: Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
    //    SmGfxArraySet * pOptGfxSet=NULL) const;           ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
    //   
    // // Draw Subdivision block BBoxes in 3d                                                                        
    // SmDisplayList * DrawSubdivision3D
    //   (SmBoolean       bOnlyDrawNodesWithVertices=FALSE, ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face vertices         
    //    SmBoolean       bOnlyDrawNodesWithEdges=FALSE,    ///< [in ]: TRUE= Draw only Subdivision Nodes that contain Face edges
    //    SmGfxArraySet * pOptGfxSet=NULL) const;           ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmTrimSrfCache,SmSurfaceCache,SmTrimSrfCache_TYPE);

} ; // end class SmTrimSrfCache

// GWC:BIND_TEMPLATES_MOVE  SM_TARRAY_TEMPLATE_PREDECLARATION(SmTrimSrfCache*) ;

#endif // !__SMTRIMSRFCACHE_H__


