// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTrimSrfCache.cpp
* PURPOSE: Source file for SmTrimSrfCache methods. 
**********************************************************************/

#include "StdAfx.h"

#include <SmTrimSrfCache.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMLOOPUSE_H__
#include <SmLoopuse.h>
#endif

#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMTESS_H__
#include <SmTess.h>
#endif

#include <SmCurveCache.h>
#include <SmAssertArray.h>


/*******************************************************************//**
PURPOSE: When m_bPointTestEnabled == TRUE
                 and TSurfaceCache is current
            determine if a point lies inside of the boundary of the
            surface->Owner's trim boundaries.

NOTES:
    Returns TRUE when
      SurfaceCache->m_bPointTestEnabled == FALSE, 
      Point is in Surface, or
      whenever no determination can be made because,
       the cache is not a TrimSrfCache,
       the cache is out of date,
       the face has been modified since the cache was last updated.
  
***********************************************************************/
SmStatus SmTrimSrfCache::PointTest
  (const SmPoint2d & crUVPoint,      // in : UVPoint to test
   SmBoolean & rbPointIsOk,          // out: TRUE = PointTest is off, 
                                     //             node is in/on face boundary
                                     //             or test cannot be made
                                     //      FALSE= Point is outside of face boundary
   SmZoneTol3d *pOptZoneTol3d,       // in : Obj ZoneTol3d assoc with crUVPoint, 
   SmObject **pOptObject)            // out: Optional Pointer to Topology Object coincident with point,
                                     //      NULL to ignore, default:[NULL]
  const
{
  // init output
  if(pOptObject) { *pOptObject = NULL ; }
  rbPointIsOk = TRUE;

  // when testing is off or cache is not current
  if (   ( !m_bPointTestEnabled  )
      || (  m_bFaceWasModified   )
//    || ( !m_bHaveTSurfaceCache ) This now works without the surface cache.
     )
    {
      // return TRUE so this point won't be culled
      rbPointIsOk = TRUE;
    }
  else // return FALSE = outside face, TRUE = in/on face
    {
      // classify point (with optional tolerance) against face
      SmFace              * pFace          = GetFace() ;
      SmZoneTol3d           sSrcZoneTol3d  = pOptZoneTol3d ? *pOptZoneTol3d
                                             : SmTol::GetZoneTol3d(pFace) ; 
      SmPointClassification sPC(sSrcZoneTol3d, GetContext()) ;     // this tol is unlikely to be correct - check it

      sPC.SetSrcZoneTol3d(sSrcZoneTol3d); 

      SER(pFace->PointClassify(crUVPoint,sSrcZoneTol3d,FALSE,TRUE,sPC));

      // Point is ok when it classified to a topology object
      rbPointIsOk =  (   sPC.GetPointClass() == SM_PC_FACE
                      || sPC.GetPointClass() == SM_PC_EDGE
                      || sPC.GetPointClass() == SM_PC_VERTEX) 
                    ? TRUE
                    : FALSE ;
      if(pOptObject) { *pOptObject = sPC.GetObject() ; }

    } // end need to classify point branch

  // all done
  return SM_SUCCESS;

} // end SmTrimSrfCache::PointTest


/*******************************************************************//**
PURPOSE: Tessellate a surface using either chord height and/or angular
    tessellation tolerance and produce a Bezier representation of the surface.
    Then take the parameter space curves and vertices and put them into the
    cache.

NOTES: calls SmSurfaceCache::BuildTree() to build the subdivision Tree.
    Then if (GetFace()->GetBrep()->m_bEditingEnabled == FALSE)
      { ensures every edge has a UVTrimCurve ;
        implants every UVTrimCurve into the Subdivision tree ; uses m_sEdgeuseListMgr ;
        implants every Vertex into the Subdivision tree ;      uses m_sVLMgr ;
        marks every subdivision node as in/on/out of face boundary
        side effects that make this different than SmSurfaceCache
         1. Subdivision Nodes augmented with Curve Object lists.
         2. Subdivision Nodes augmented with Surface Object lists.
         3. m_sUVDomain reduced to TrimSurface boundaries.
         4. m_bProcessBoundaryCurves  = FALSE;
         5. m_bPointTestEnabled       = TRUE; 
         6. m_bHaveTSurfaceCache      = TRUE;
         7. m_bFaceContainmentDone    = TRUE;  
      }

***********************************************************************/
SmStatus SmTrimSrfCache::BuildTree
  (SmMemBlockMgr *)                 // in : only declared to be a virtual function of SmSurfaceCache::BuildTree
{
  // 
  //std::lock_guard<std::recursive_mutex> lock((GetSurface().mCacheMutex));
  // prevent bad recursion back into BuildTree
  SmTemporaryChangeValue<SmBoolean> sTCVFaceContainment(m_bFaceContainmentDone, 2);

  // Build m_ptree surface subdivision tree when it does not exist
  SmBoolean bCleanUpBuildData = TRUE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  SmMemBlockMgr sBezierBlock ;
  if(m_pTree == NULL) 
    { 
      SER(SmSurfaceCache::BuildTree(&sBezierBlock));

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { m_pTree->Dump() ;
          SM_ASSERT_VALID(this) ; 
        }
#endif      
    }
  else
    {
      // The temporary array sBezierBlock will be empty and all the
      //  TreeNode->m_pData->m_pBezier pointers will be NULL.  This
      //  no longer causes a problem for the bounding box computation made
      //  during node subdivision due to UVTrimCurve insertion because
      //  a suitable surface has been found for all bounding box computation cases.
      bCleanUpBuildData = FALSE ;
    } 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { m_pTree->Dump() ;
      SM_ASSERT_VALID(this) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; this->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; m_pTree->DrawNodeBoundingBoxes(0, .3, .7, .1, .1 ,.1) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // skip inserting UVTrimCurves and classifying node containment
  //   when Surface is not owned by a face (no UVTrimCurves to process)
  //     or Brep Editing is enabled        (face data structure may be in an intermediate state)
  //     or PointTestEnabled is turned off
  if (   GetFace() == NULL
      || GetFace()->GetBrep()->m_bEditingEnabled == TRUE)
   // gwc:removed to support one surface cache changes   || m_bPointTestEnabled == FALSE)
    {
       // set internal state to run solvers for a standalone surface
       m_bProcessBoundaryCurves  = TRUE;
       m_bPointTestEnabled       = FALSE;
       m_bHaveTSurfaceCache      = FALSE;
    }
   else // insert UVTrimCurves and classifying node containment
        //   because editing is disabled in the brep
        //   and the surface is owned by a face
        //   and PointTesting is enabled
    {
      // prevent undesired recursion to BuildTree - turn off m_bPointTestEnabled
      // This will allow future GetTree() calls to return a Tree with or without
      //   implanted UVTrimCurves
      SmTemporaryChangeValue<SmBoolean> sTCVPointTestEnabled(m_bPointTestEnabled, FALSE);  
      SmTemporaryChangeValue<SmBoolean> sTCVEditingEnabled(GetFace()->GetBrep()->m_bEditingEnabled, TRUE);

      // create UVTrimCurve for every face->edge
      //  gwc note: UVTrimCurves are constructed with SmSurface::DropCurve()  
      double dMeanCS = 0.0, dMaxCS = 0.0, dMeanV = 0.0, dMaxV = 0.0, dMeanCC = 0.0, dMaxCC = 0.0;
      SER(GetFace()->CreateUVTrimCurves(TRUE,NULL,NULL,
                                        dMeanCS,
                                        dMaxCS,
                                        dMeanV,
                                        dMaxV,
                                        dMeanCC,
                                        dMaxCC));

      // Now implant edgeuses and vertexuses into the cache tree

      // locals
      SmLoopuse* sData1[64];
      SmTArray<SmLoopuse*> sLoopuses(64,sData1);
      GetFace()->GetUpwardLoopuses(sLoopuses);
      SmVertexuse * sData2[64];
      SmTArray<SmVertexuse*> sVUses(64,sData2);
      SmEdgeuse * sData3[64];
      SmTArray<SmEdgeuse*> sEUses(64,sData3);
      m_sVLMgr.Initialize(ALIGN_SIZE(sizeof(SmVertexList)),30) ;
      m_sEdgeuseListMgr.Initialize(ALIGN_SIZE(sizeof(SmEdgeuseList)),30) ;

      SmExtent2d sFaceDomain;
      // do subdivision on nonPlanar faces.
      // gwc: consider only doing uni-directional subdivision on 
      //        surfaces with one linear direction like cones, cylinders, and simple sweep surfaces
      SmBoolean bDoSubdivision = m_cpSurface->IsKindOf(SmPlane_TYPE)
                                 ? FALSE
                                 : TRUE ; // TRUE = split large UVDomain subdivision nodes that get UVCurves or Vertices

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      m_pTree->Dump() ;
      SM_ASSERT_VALID(this) ;
    }
#endif 

      // for every loopuse - implant edgeuses into TrimSurfaceCache
      for (ULONG i=0; i<sLoopuses.GetSize(); i++) 
        {
          SmLoopuse *pLU = (SmLoopuse*)sLoopuses[i];

          // for every loopuse->edgeuse
          pLU->GetEdgeuses(sEUses);
          for (ULONG j=0; j<sEUses.GetSize(); j++) 
            {
              // implant edgeuse into TrimSurfaceCache - add a SmVertexList pointer to every
              //    Surface subdivision node's SmBezierAux2d::m_sEdgeuseList whose m_sUVDomain
              //    is intersected by the bounding box of the UVCurve.  This means many
              //      leaf nodes and parent nodes may end up getting a pointer to this one curve.
              //  
              //  side effects: 1. set nodes close to or intersecting curve as 
              //                      SM_NC_ON_BOUNDARY and marked.
              //                2. set all ancestors of an SM_NC_ON_BOUNDARY leaf node as
              //                      SM_NC_ON_BOUNDARY. 
              //                3.    set all parents with two marked children as marked.
              //                4. if(bDoSubdivision == TRUE) split nodes with large UVDomains and classify the children,
              //                      else don't split. 
              SER(ImplantEdgeuse((SmEdgeuse*)sEUses[j],bDoSubdivision,sFaceDomain));

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                { 
                  m_pTree->Dump();
                  SM_ASSERT_VALID(this);
                  this->DrawSubdivision2D( TRUE );
                }
#endif 
            } // end iter every Edgeuse
        } // end iter every Loopuse


      // for every Loopuse - implant every vertexuse into the TrimSurfCache
      for (ULONG j=0; j<sLoopuses.GetSize(); j++) 
        {
          SmLoopuse *pLU = (SmLoopuse*)sLoopuses[j];

          // for every Loopuse->vertexuse
          pLU->GetVertexuses(sVUses);
          for (ULONG k=0; k<sVUses.GetSize(); k++) 
            {
              // implant vertexuses into TrimSurfaceCache
              //  side effects: 1. set nodes intersecting vertex as 
              //                      SM_NC_ON_BOUNDARY and marked.
              //                2. set all ancestors of an SM_NC_ON_BOUNDARY leaf node as
              //                      SM_NC_ON_BOUNDARY. 
              //                3.    set all parents with two marked children as marked.
              SER(ImplantVertexuse((SmVertexuse*)sVUses[k],sFaceDomain));

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                { 
                  m_pTree->Dump();
                  SM_ASSERT_VALID(this);
                  this->DrawSubdivision2D( TRUE );
                }
#endif 

            } // end iter every vertexuse
        } // end iter every Loopuse

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { m_pTree->Dump() ;
          SM_ASSERT_VALID(this) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; this->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; m_pTree->DrawNodeBoundingBoxes(0, .3, .7, .1, .1 ,.1) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Here is where we should perhaps do additional subdivision
      // of the surface to align with edges and maximize containment
      // SER(EdgeBasedSubdivision());

      // set SurfaceCacheDomain as SurfaceNaturalDomain/FaceDomain intersection
      SmExtent2d sSurfaceDomain = GetFace()->GetSurface()->GetNaturalUVDomain();
      SER(sSurfaceDomain.Intersect(sFaceDomain,m_sUVDomain));

      // When the surface has UVTrimCurves
      if(sLoopuses.GetSize() > 0) 
        { 
          // mark state to run Solvers for TrimSurfaces
          //   skip finding natural UVTrimCurve/target solutions.
          m_bProcessBoundaryCurves  = FALSE; 
          m_bPointTestEnabled       = TRUE;
          m_bHaveTSurfaceCache      = TRUE;

          // Now determine containment of individual nodes in the tree relative to the face.

          // Classify every TrimSurfCache Node not on a boundary as INSIDE or OUTSIDE
          SER(MarkFaceContainment());          // gwc: switched from ray tracing to loop containment 
                                               //      so can be run with or without m_bPointTestEnabled
          // set a permament value into m_bFaceContainment
          sTCVFaceContainment.Clear();
          m_bFaceContainmentDone    = TRUE;

        }
      else // there are no UVTrimCurves - treat as a standalone surface
        {
          // run solvers looking for natural UVTrimCurve/target solutions.   
          // required to get IS_SO_MINIMIZE/IS_SO_MAXIMIZE and SM_SO_INTERSECT 
          // 'near miss' solutions.
          m_bProcessBoundaryCurves  = TRUE;
          m_bPointTestEnabled       = FALSE;
          m_bHaveTSurfaceCache      = FALSE;
        }

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        { m_pTree->Dump() ;
          SM_ASSERT_VALID(m_pTree) ;
          SM_ASSERT_VALID(this) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; GetFace()->Draw(); sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,0) ; this->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          // smgfx_SetLook(1,2, 1,0,0) ; m_pTree->DrawNodeBoundingBoxes(0, .3, .7, .1, .1 ,.1) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif

    } // end Add TrimFace data (m_bEditingEnabled == FALSE) branch

  // Clean up NURBS and Bezier pointers that become stale when m_sSubdisionSurface and sBezierBlock are freed
  if(bCleanUpBuildData)
    {
      // need to 
      //   Free all m_sSubdivisionSurface memory.
      //   Set SmBezierAux2d::mBA_pSurface pointers to source surface.
      //   Set SmBezierPatch::m_pBezier pointers to NULL - they are construction only data.
        {
          SmObjsDelete<SmSurface*> sClean(&m_sSubdivisionSurfaces);

          ULONG ii ;
          ULONG lBezCount = m_sBezMgr.GetNumActiveElements() ;
          ULONG lAuxCount = m_sAuxMgr.GetNumActiveElements() ;
          for(ii=0;ii<lBezCount;ii++) { SmBezierPatch *pBezPatch = (SmBezierPatch *)m_sBezMgr.GetAt(ii) ;
                                        pBezPatch->mBA_pSurface  = m_cpSurface ;
                                        pBezPatch->m_pBezier     = NULL ;
                                      }
          for(ii=0;ii<lAuxCount;ii++) { SmBezierAux2d *pAux = (SmBezierAux2d *)m_sAuxMgr.GetAt(ii) ;
                                        pAux->mBA_pSurface  = m_cpSurface ;
                                      }
        }
      m_sSubdivisionSurfaces.ReSet();

    } // end need to clean up check

  // all done
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SM_ASSERT_VALID(m_pTree) ;
      m_pTree->Dump() ;
      SmTArray<SmTreeNode*> sNodes ;
      m_pTree->GetAllTreeNodes(sNodes) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; this->SmSurfaceCache::Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; this->Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif
  return SM_SUCCESS;

} // end SmTrimSrfCache::BuildTree

/*******************************************************************//**
PURPOSE: Implant the edgeuse information into the surface cache in 
            such a way as it is accessible during surface cache traversal.

NOTES: 
  expands rFaceExtent to include the uvDomains of all nodes found on the boundary

  side effects: 1. set nodes close to or intersecting curve as 
                      SM_NC_ON_BOUNDARY and marked and adds curve to node's curve list.
                2. set all ancestors of an SM_NC_ON_BOUNDARY leaf node as
                      SM_NC_ON_BOUNDARY. 
                3.    set all parents with two marked children as marked.
                4. if(bDoSubdivision == TRUE) split nodes with large UVDomains and classify the children,
                      else don't split. 

***********************************************************************/
SmStatus SmTrimSrfCache::ImplantEdgeuse
  (SmEdgeuse  * pEdgeuse,          // in : geometry to embed into surface spatial decomposition      
   SmBoolean    bDoSubdivision,    // in : TRUE  = Split nodes with large uv-domains and classify the children
                                   //      FALSE = don't split nodes - just classify them as they are
   SmExtent2d & rFaceExtent,       // out: expanded to include the uv_domain of all
                                   //      nodes classified as on the boundary
   SmTreeNode * pOptTreeNode)      // in : when given, only implant edgeuse in this node
                                   //      default:[NULL]
{
  // check state - to prevent bad recursion
  SM_ASSERT(m_bPointTestEnabled == FALSE) ;

  // locals
  SmBoolean  bUVCurveIsLine = FALSE;
  SmPoint2d  sLinePoint;
  SmVector2d sLineVec;
  SmPoint3d  sPnt;
  SmVector3d sVec;
  //const SmContext *pContext = pEdgeuse->GetContext();
  SmBSplineCurve  *pEUCurve = SM_CAST_PTR(SmBSplineCurve,pEdgeuse->GetUVTrimCurve());
  NER(pEUCurve);

  // set up for poles
  ULONG      lPoles = 0 ;
  SmExtent1d sUIvl, sVIvl ;
  double     dUTol, dVTol ;
    {
      const SmSurface *cpSurface = GetSurface() ;
      SmExtent2d       sUVDomain = cpSurface->GetNaturalUVDomain() ;

      // save natural domain sizes
      sUIvl = sUVDomain.GetUInterval() ;
      sVIvl = sUVDomain.GetVInterval() ;
      dUTol = sUIvl.GetLength() / 100 ;
      dVTol = sVIvl.GetLength() / 100 ;

      // list m_sUVDomain boundaries near SrfNaturalDomain boundaries
      lPoles =   ((smos_Fabs( m_sUVDomain.GetMin().x - sUIvl.GetMin() ) < dUTol) ? 1 : 0)
               + ((smos_Fabs( m_sUVDomain.GetMin().y - sVIvl.GetMin() ) < dVTol) ? 2 : 0)
               + ((smos_Fabs( m_sUVDomain.GetMax().x - sUIvl.GetMax() ) < dUTol) ? 4 : 0)
               + ((smos_Fabs( m_sUVDomain.GetMax().y - sVIvl.GetMax() ) < dVTol) ? 8 : 0);

      // eliminate boundaries which are not poles
      if(lPoles)
        {
          ULONG lSrfPoles  = cpSurface->GetSingularities() ;
          lPoles &= lSrfPoles ;
        }
   } // end set up for poles scope

  // sCurveBBox = EUCurve 2D Bounding Box expanded by tolerance
  SmExtent3d  sBBox;
  SER(pEUCurve->CalculateBoundingBox(pEUCurve->GetNaturalInterval(), &sBBox));
  SmExtent2d sCurveBBox( sBBox.GetMin().x, sBBox.GetMin().y,
                         sBBox.GetMax().x, sBBox.GetMax().y);
  sCurveBBox.ExpandAbsolute((1.0 + sCurveBBox.GetSize().Length()) * SM_EFF_ZERO_SQRT);

  // when EUCurve is a line - get its local values
  if (pEUCurve->IsLine(4,SM_ZONE_TOL_3D/10.0,sPnt,sVec)) 
    {
      sLinePoint.Set(sPnt.x,sPnt.y);
      sLineVec.Set(sVec.x,sVec.y);
      bUVCurveIsLine = TRUE;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      pEUCurve->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0); pEUCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); this->DrawSubdivision2D(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // Compute OffAxisTolerance - maximum size of non largest component
  //                            of the bounding box of the curve segment.

  SmVector2d sDiff  = m_sUVDomain.GetSize();
  double     dScale = 1.0;

  SmBoolean bClampNodeGrowth = FALSE;
  if (sDiff.x > sDiff.y*20.0) { bClampNodeGrowth = TRUE;
                                dScale = sDiff.x / sDiff.y;
                              }
  if (sDiff.y > sDiff.x*20.0) { bClampNodeGrowth = TRUE;
                                dScale = sDiff.x / sDiff.y;
                              }
  // Scale things to smaller size if one dimension of UV domain is
  // very large compared to the other.
  if      (sDiff.x > sDiff.y*100.0) sDiff.x = sDiff.x / 100.0;
  else if (sDiff.x > sDiff.y*10.0 ) sDiff.x = sDiff.y*10.0;

  if      (sDiff.y > sDiff.x*100.0) sDiff.y = sDiff.y / 100.0;
  else if (sDiff.y > sDiff.x*10.0 ) sDiff.y = sDiff.x*10.0;

  //
  double dUVSize             = sDiff.Length();
  double dOffAxisTolerance   = (dUVSize / m_dTrimSubdivisionFactor) ;
  double dMinNodeSizeSquared = dOffAxisTolerance * dOffAxisTolerance ;
  
  // increase dMinNodeSizeSquared when working on very long lines
  if (bUVCurveIsLine) // gwc: changed to run under more circumstances
  //      if (bUVCurveIsLine && pContext->GetDoingBoolean()) // only called through SmSurface::GetTree() from SmMerge::ManifoldBoolean()
  //                                                         // after tree construction
  //                                                         // has been delayed in SmCacheMgrTSrf::CacheMakeOrValidate().
    {
      double dLineSize    = sCurveBBox.GetSize().Length() / 10.0;
      dMinNodeSizeSquared = smos_Max(dMinNodeSizeSquared, dLineSize*dLineSize);
    }

  // sMinSize
  SmVector2d sMinSize(0,0);
  if (bClampNodeGrowth) 
    {
      SmVector2d sDiff2 = m_sUVDomain.GetSize();
      sMinSize = sDiff2 / m_dTrimSubdivisionFactor;
    }

  // find all tree nodes that intersect sCurveBBox - use pOptTreeNode if given
  SmTreeNode *pData[256];
  SmTArray<SmTreeNode*> sNodes(256,pData);
  if(pOptTreeNode == NULL) { SER(FindUVNodes(sCurveBBox, sNodes, TRUE)); } // TRUE = include nodes near common poles
  else                     { sNodes.Add(pOptTreeNode) ; }

  // no work - no nodes
  if (sNodes.GetSize() == 0) 
    { return SM_SUCCESS; }

  // Don't need chord height if we have OffAxisTolerance
              
  double dChordHeightTol = 0.0;
          
  // Pick an angular tolerance - need to experiment a little to see
  // which values are optimal for the various global solver algorithms
  // which we will utilize.
              
  double dAngularTol = 20.0;  // 20 degrees might be ok
              
  // Create the curve cache for nonLinear EUCurves
          
  SmExtent1d sIvl = pEUCurve->GetNaturalInterval();
  SmCurveCache *pCurveCache = NULL;
  SmObjDelete sCleanUpCache;

  // for non-linear EUCurves - build a CurveCache
  if (!bUVCurveIsLine) 
    {
      pCurveCache = new(*GetContext()) SmCurveCache
        (*pEUCurve,          // in : target curve
          sIvl,              // in : target interval
          TRUE,              // in : only used when bComputeAuxTreeData == TRUE
                             //      FALSE = Simplify Tree where Parent Node's pass all tessellation tests
                             //      TRUE  = don't 
          dChordHeightTol,   // in : max leafNode control-polygon vertex to baseline distance, 0 = ignore
          dAngularTol,       // in : max leafNode control-polygon vertex angle sum,            0 = ignore
          dOffAxisTolerance, // in : max leafNode off axis control-polygon BBox size,          0 = ignore
                             //        subdivides leaves into axis aligned near-linear segments.
          0,                 // in : limits max element size, 0 = ignore
          TRUE);             // in : TRUE = propagate child properties up to parent nodes.
                             //      FALSE= don't 
                             // in : max leafNode 3d control-polygon baseline size,  0 = ignore
                             // in : min (leafNode Interval)/(Curve Interval) ratio, 0 = ignore
      NER(pCurveCache);
      sCleanUpCache.SetObj(pCurveCache);
      SER(pCurveCache->Tessellate()); // does actual work of creating decomposition tree data
                                      // This cache is not stored in the global cache queue
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          pEUCurve->Dump();
          pCurveCache->Dump() ;
        }
#endif // SM_DEBUG_CODE
    }

  // for every node that intersects the EUCurve bounding box
  SmTArray<SmTreeNode*> sStack;
  for (ULONG i=0; i<sNodes.GetSize(); i++) 
    {
      SmTreeNode *pNode = (SmTreeNode*)sNodes[i];

      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA 
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE 
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      // init stack with current node
      sStack.Add(pNode);

      // while nodes exist in the stack
      while (sStack.GetSize() > 0) 
        {
          pNode = sStack.GetLast();
          sStack.RemoveLast();

          // skip non-leaf nodes
          if (pNode->m_pChild1 != NULL) { continue; }

          // sNodeBBox = this node's 2D bounding box
          SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;
          SmExtent3d sNodeBBox(SmPoint3d(pAux->m_sUVDomain.GetMin()),
                               SmPoint3d(pAux->m_sUVDomain.GetMax()));

          // nodes by position
          SmBoolean bKeepForPosition = FALSE ;

          // when working with nonlinear curves - keep nodes that intersect the curve cache
          if(pCurveCache)
            {
              bKeepForPosition = pCurveCache->GetTree()->IntersectsBox(sNodeBBox) ;
            }

          // when working with lines - skip nodes too far from line
          if ( ! pCurveCache )
            {
              // First do bounding box check (uv).
              bKeepForPosition = ! sCurveBBox.AreDisjoint( sNodeBBox );

              // BBoxes intersect, check against line.
              if ( bKeepForPosition )
                {
                  SmBoolean  bFoundInterval;
                  SmExtent1d sTrimmedInterval;
                  SER (pAux->m_sUVDomain.IntersectWithInfiniteLine( sLinePoint, sLineVec, bFoundInterval, sTrimmedInterval ));
                  bKeepForPosition = bFoundInterval ;
                }
            } // end working with lines branch

          // also keep nodes near poles
          SmBoolean bKeepForPole = FALSE ;
          if(lPoles && !bKeepForPosition)
            { bKeepForPole =   ((lPoles & SM_SS_UMIN) && (smos_Fabs(pAux->m_sUVDomain.GetMin().x-sUIvl.GetMin()) < dUTol))
                            || ((lPoles & SM_SS_UMAX) && (smos_Fabs(pAux->m_sUVDomain.GetMax().x-sUIvl.GetMax()) < dUTol))
                            || ((lPoles & SM_SS_VMIN) && (smos_Fabs(pAux->m_sUVDomain.GetMin().y-sVIvl.GetMin()) < dVTol))
                            || ((lPoles & SM_SS_VMAX) && (smos_Fabs(pAux->m_sUVDomain.GetMax().y-sVIvl.GetMax()) < dVTol)) ;
            }

          // skip nodes too far away from this node's bounding box
          if(   ! bKeepForPosition
             && ! bKeepForPole) 
            { continue; }

          // arrive here when we found a node near enough the curve to count as on the boundary

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              SmTArray<SmEdgeuseList*>  sEdgeuseList ;  pAux->m_sEdgeuseList.GetAllNodes(sEdgeuseList) ;  
              SmTArray<SmVertexList*> sVertexList ; pAux->m_vVertexList.GetAllNodes(sVertexList) ;

              sEdgeuseList.Dump() ;
              sVertexList.Dump() ;

              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,0 ); this->DrawSubdivision2D( TRUE ); sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 0,0,1); sNodeBBox.Draw(GetContext()); sm_GraphicsLoop();
              smgfx_SetLook( 1,2, 1,0,0); sCurveBBox.Draw(GetContext()); sm_GraphicsLoop();
              sm_GraphicsLoop();
            } 
#endif
          // When not doing subdividing, when node is small enough, // or just kept for pole
          SmVector2d sUVDiff( pAux->m_sUVDomain.GetSize() );
          if(   bDoSubdivision == FALSE
             || sUVDiff.LengthSquared() < dMinNodeSizeSquared
             || sUVDiff.x < sMinSize.x 
             || sUVDiff.y < sMinSize.y)
  // GWC: try allow subdividing pole nodes           || (bKeepForPosition == FALSE && bKeepForPole == TRUE))
            {
              // mark the AuxData as on the boundary
              pAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
              pAux->m_bIsMarked  = TRUE ;

              // add the Edgeuse to the AuxData's curve list
              SmEdgeuseList *pEdgeuseList   = (SmEdgeuseList*)m_sEdgeuseListMgr.GetNewElement();
              pEdgeuseList->m_pEU = pEdgeuse;
              // not used: pCL->m_pCurveNode  = NULL;
              pAux->m_sEdgeuseList.Prepend(pEdgeuseList);

              // propagate state change to ancestors
              pNode->PropagateToParents() ;

            }
          else // the node is large enough to be subdivided before being marked
            {  
              // subdivide the node's largest UV direction 
              SmSurfParamType eSubdivideDirection = (sUVDiff.x > sUVDiff.y * dScale) ? SM_SP_U : SM_SP_V ;

              // promote leaf node into a parent->Child(1,2) triplet
              SER(SubdivideNode(pNode, eSubdivideDirection, TRUE, NULL)); // TRUE = don't split bezier patch

              // no need here to compute parent BBoxes from children boxes
              // because the new children are not building BezierPatches from coarse surface approximations.

              // put the children on the stack
              sStack.Add(pNode->m_pChild1);
              sStack.Add(pNode->m_pChild2);
            }

        } // While newly subdivided children are on the stack
    } // end iter every treeNode that intersects the curve's bounding box

  // Update the domain of the face extents
  //      SmVector2d sSize = rFaceExtent.GetSize();   // GWC: changed init check to remove Assert Failure
  if (rFaceExtent.IsInit() || rFaceExtent.HasNegativeArea()) { rFaceExtent = sCurveBBox; }
  else                                                       { sCurveBBox.Union(rFaceExtent,rFaceExtent); }

  // all done
  return SM_SUCCESS;

} // end SmTrimSrfCache::ImplantEdgeuse


/*******************************************************************/ /**
 PURPOSE: Implant the PolyEdge information into the surface cache. This replicates the
    embedding done by SmTesssSrfCache::ImplantUVSegment, but for a fixed node, for use
    in tree editing post-construction

 NOTES:
   reports nothing if the PolyEdge and the node do not intersect

   side effects: Set pNode as SM_NC_ON_BOUNDARY and adds PolyEdge to node's PolyEdge list.

 ***********************************************************************/
SmStatus SmTrimSrfCache::ImplantPolyEdge(SmPolyEdge* pPolyEdge,  // in : polyedge to embed into surface spatial
                                                                 // decomposition
                                         SmTreeNode* pNode)      // in : node to implant the above polyedge into
{
// check state - to prevent bad recursion
SM_ASSERT(m_bPointTestEnabled == FALSE);

// get segment endpoints
SmPoint2d sStartPt(pPolyEdge->GetStartPoint().x, pPolyEdge->GetStartPoint().y);
SmPoint2d sEndPt(pPolyEdge->GetEndPoint().x, pPolyEdge->GetEndPoint().y);

// Tolerance: get uv space tols, expand the uv domain of the node by tols
// the below follows the behavior of ImplantUVSegment
SmTol3d sTol3d = pPolyEdge->GetTolerance();
const SmSurface* pSurf = pPolyEdge->GetPolyFace()->GetOriginalFace()->GetSurface();
SmPoint2d sMidUV((sStartPt + sEndPt) / 2.0);
SmVector2d sDirUV(1, 0);
SmTol2d sTolU = SmTol::MapTo2d(sTol3d, sMidUV, sDirUV, *pSurf);
sDirUV.Set(0, 1);
SmTol2d sTolV = SmTol::MapTo2d(sTol3d, sMidUV, sDirUV, *pSurf);
SmVector2d sTolExpansion(sTolU, sTolV);

SmBezierAux2d* pAux = (SmBezierAux2d*)pNode->m_pData;
SmExtent2d sDomain(pAux->m_sUVDomain);
SmExtent2d sDomainExp(pAux->m_sUVDomain);
sDomainExp.ExpandAbsolute(sTolExpansion * 10.0);

// locals for intersection test with node domain
SmVector2d sLineVec = sEndPt - sStartPt;
SmBoolean bIsDegenerate = smos_Fabs(sLineVec.x) <= sTolU && smos_Fabs(sLineVec.y) <= sTolV;
SmBoolean bFoundInterval;
SmExtent1d sNodePolyEdgeIntersection;
SmExtent1d sUnit(0., 1.);
SmExtent1d sIntersectionNode;
SmExtent1d sIntersectionUnit;

// testing to see if node contains the start/end points of PolyEdge or PolyEdge intersects the domain
// this follows the behavior of ImplantUVSegment.
SmBoolean bEmbedPolyEdge = TRUE;

if (!sDomainExp.ContainsPoint2d(sStartPt) && !sDomainExp.ContainsPoint2d(sEndPt))
  {
    if (bIsDegenerate)
        {
          bEmbedPolyEdge = FALSE;
        }
    else
        {
        SER(sDomain.IntersectWithInfiniteLine(sStartPt, sLineVec, bFoundInterval, sIntersectionNode));
        if (!bFoundInterval)
          {
            bEmbedPolyEdge = FALSE;
          }
        else if (sIntersectionNode.Intersect(sUnit, sIntersectionUnit) == SM_ERR)
          {
            bEmbedPolyEdge = FALSE;
          }
        }
  }

if (bEmbedPolyEdge)
  {
    // Add segment to this node's m_sPolyEdgeList
    pAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
    SmPolyEdgeList* pPolyEdgeList = (SmPolyEdgeList*)m_sPolyEdgeListMgr.GetNewElement();
    pPolyEdgeList->m_pPolyEdge = pPolyEdge;
    pAux->m_sPolyEdgeList.Prepend(pPolyEdgeList);
  }

return SM_SUCCESS;

} // end SmTrimSrfCache::ImplantPolyEdge


/*******************************************************************//**
PURPOSE: Implant the vertexuse information into the surface cache
    in such a ways as it is accessible during traversal.

NOTES: 
***********************************************************************/
SmStatus SmTrimSrfCache::ImplantVertexuse
  (SmVertexuse * pVertexuse,     // in : vertexuse to implant
   SmExtent2d & rFaceExtent)     // out: expand faceExtent to include node's marked as SM_NC_ON_BOUNDARY
{
  // Find the UV point on the surface and corresponding UV tolerance for the vertex.
  // Find the corresponding tolerance in parameter space U and V directions.

  // locals: UVPoint, Vertex, 3D Surface Properties
  SmPoint2d sUV;
  SmPoint3d sPnt;
  SmVector3d sDU,sDV;
  SER(pVertexuse->ComputeUVPoint(sUV));
  SER(m_cpSurface->Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
  SmVertex *pV = pVertexuse->GetVertex();

  // get surface u and v speeds
  SmVector2d sUVTol;
  double dDULen = sDU.Length();
  double dDVLen = sDV.Length();

  // The following few lines of code keep the parameter space tolerance
  // from getting too long and skinny.  That sometimes occurs near poles.
  if (dDULen < dDVLen/10.0) 
    {
      dDULen = dDVLen;
    }
  if (dDVLen < dDULen/10.0) 
    {
      dDVLen = dDULen;
    }

  // pick u and v tolerance
  sUVTol.x = pV->GetTolerance()/dDULen;
  sUVTol.y = pV->GetTolerance()/dDVLen;

  // expand the UV tolerances when near surface poles orof:[SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX]
  const SmSurface *cpSurface = GetSurface() ;
  SmExtent2d      sUVDomain  = cpSurface->GetNaturalUVDomain() ;
  SmExtent1d      sUIvl      = sUVDomain.GetUInterval() ;
  SmExtent1d      sVIvl      = sUVDomain.GetVInterval() ;
  double          dUTol      = sUIvl.GetLength() / 100 ;
  double          dVTol      = sVIvl.GetLength() / 100 ;
  
  // Get boundaries close to sUV
  ULONG lPoles =   ((smos_Fabs(sUV.x-sUIvl.GetMin()) < dUTol) ? 1 : 0 )
                 + ((smos_Fabs(sUV.y-sVIvl.GetMin()) < dVTol) ? 2 : 0 )
                 + ((smos_Fabs(sUV.x-sUIvl.GetMax()) < dUTol) ? 4 : 0 )
                 + ((smos_Fabs(sUV.y-sVIvl.GetMax()) < dVTol) ? 8 : 0 ) ;

  // when sUV is close enough to boundaries for a pole check
  if(lPoles != SM_SS_NONE)
    {
      // only check those boundaries which are poles
      lPoles &=  cpSurface->GetSingularities() ;

      // expand tolerance to include entire degenerate sides when appropriate
      // when UV point is in constant U singular side
      if(   (lPoles & SM_SS_UMIN) 
         || (lPoles & SM_SS_UMAX))
        { 
          // expand sUVTol.y to include all values
          sUVTol.y = sVIvl.GetLength() ;
        }

      // when UV point is in constant V singular side
      if(   (lPoles & SM_SS_VMIN) 
         || (lPoles & SM_SS_VMAX))
        { 
          // expand sUVTol.y to include all values
          sUVTol.x = sUIvl.GetLength() ;
        }
    } // end sUV near enough to edges to check for poles

  // set point extent to include the point and its tolerance
  SmExtent2d sUVExt(sUV-sUVTol,sUV+sUVTol);

  // get all nodes that intersect the point extent
  SmTreeNode *pData[16];
  SmTArray<SmTreeNode*> sNodes(16,pData);
  SER(FindUVNodes(sUVExt,sNodes,FALSE));  // FALSE = don't check for poles (work is done in this method)

  SM_ASSERT(sNodes.GetSize() > 0);

  // for every intersecting node
  for (ULONG i=0; i<sNodes.GetSize(); i++) 
    {
      SmTreeNode *pNode = (SmTreeNode*)sNodes[i];

      // skip non-leaf nodes
      if (pNode->m_pChild1 != NULL) 
        { continue; }

      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      // local
      SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;

      // mark node as on boundary
      pAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
      pAux->m_bIsMarked  = TRUE ;

      // add vertexuse to node's vertex list
      SmVertexList *pVL = (SmVertexList*)m_sVLMgr.GetNewElement();
      pVL->m_pVU = pVertexuse;
      pVL->m_vUVPoint = sUV;
      pVL->m_vUVTol = sUVTol;
      pAux->m_vVertexList.Prepend(pVL);
    
      // propagate state change to ancestors
      pNode->PropagateToParents() ;

    } // end iter every intersecting node

  // Update the domain of the face extents
  SmVector2d sSize = rFaceExtent.GetSize();
  if (sSize.x < 0.0 || sSize.y < 0.0) { rFaceExtent = sUVExt; }
  else                                { sUVExt.Union(rFaceExtent,rFaceExtent); }
  
  // all done    
  return SM_SUCCESS;

} // end SmTrimSrfCache::ImplantVertexuse


/*******************************************************************//**
PURPOSE: Constructor for SmTrimSrfCache.  It allows input of data used
    during tessellation of the cache.  Note that this constructor only
    inputs values.  The actual creation of cache data is done in the 
    Tessellate method.

NOTES: 
***********************************************************************/
SmTrimSrfCache::SmTrimSrfCache
  (const SmSurface & crSurface,
   double  dChordHeightTolerance,
   double  dAngleTolerance,
   double  dAspectRatio3D,
   double  dMaxSideLength3D,
   double  dMinSideLength3D,
   double  dMinSideLengthRatioUV)
 : SmSurfaceCache(crSurface,
                  crSurface.GetNaturalUVDomain(),
                  dChordHeightTolerance,
                  dAngleTolerance,
                  dAspectRatio3D,
                  dMaxSideLength3D,
                  dMinSideLength3D,
                  dMinSideLengthRatioUV) 
{

  // set state bits anticipating a Trim Surface cache with UVTrimCurves.
  // BuildTree() will change these values when there are no UVTrimCurves.
  m_bFaceWasModified       = FALSE ;
  m_bProcessBoundaryCurves = FALSE ;  // solvers skip naturalBoundaryCurve/target solutions
                                      // needs to be true for standalone surfaces.
  m_bPointTestEnabled      = TRUE  ;  // solvers onlyc accept point solutions within trim boundaries.
                                      // may be false for standalone surfaces.
  m_bHaveTSurfaceCache     = FALSE ;  // Set to true after adding UVTrimCurves to cache
  m_bFaceContainmentDone   = FALSE ;  // Set to true after classifying nodes as in/on/out of UVTrimCurves
                                      //  Also used during construction to track internal state.

#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // count caches made and inform the debugging public
  crSurface.m_lTrimSrfCacheCount++ ;
  const SmFace *cpFace = (SmFace *)crSurface.GetFace() ;
  smos_sprintf(sBuff,_T("\nConstructed TrimSurface Cache [Face = 0x%p, Surface = 0x%p, rep = %ld] iter: %ld") 
                   cpFace, 
                   &crSurface, 
                   crSurface.m_lTrimSrfCacheCount, 
                   lCount) ;
  MYPRINTF(sBuff) ;
#endif

} // end SmTrimSrfCache::SmTrimSrfCache constructor

/*******************************************************************//**
PURPOSE: Destructor for the trimmed surface cache.

NOTES: 
***********************************************************************/
SmTrimSrfCache::~SmTrimSrfCache()
{
#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  smos_sprintf(sBuff,_T("\nDestructed TrimSurface Cache [Surface = 0x%p] iter: %ld"), 
          &m_crSurface, lCount) ;
  MYPRINTF(sBuff) ;
#endif // SM_DEBUG_CACHE_CPP



} // end SmTrimSrfCache::~SmTrimSrfCache destructor

/*******************************************************************//**
PURPOSE: Classify a point relative to the trimmed surface.

NOTES: 
***********************************************************************/
SmStatus SmTrimSrfCache::PointClassify
 (const SmPoint2d           & crUVPoint,          // in : Point to classify
  SmZoneTol3d                 sSrcZoneTol3d,      // in : Obj ZoneTol3d assoc with UVPoint, not this face 
                                                  //     (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL)
  SmPointClassificationType & eClassification,    // out: Type of object coincident with point
                                                  //      oneof SM_PC_VERTEX
                                                  //            SM_PC_EDGE
                                                  //            SM_PC_FACE
                                                  //            SM_PC_UNKNOWN
  SmObject *& rpObjectInOrOn)                     // out: object coincident with point
 const
{ 
  // init output
  eClassification = SM_PC_UNKNOWN;
  rpObjectInOrOn  = NULL;

  // local classification object
  SmFace    * pFace          = GetFace() ;
  SmPointClassification sPointClass(sSrcZoneTol3d, GetContext()) ;  

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if(bDebugMe || lCount == lDebugCount)
    {
      SmPoint3d sPoint ;
      SmFace *pFaceFromSrf = (SmFace *)m_cpSurface->GetFace() ;
      SmBrep *pBrep = pFaceFromSrf ? pFaceFromSrf->GetBrep() : NULL ;
      m_cpSurface->EvaluatePoint(crUVPoint, sPoint) ;
      
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,1,0) ; if(m_cpSurface) m_cpSurface->DrawUV(4,4) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,0,0) ; if(pFaceFromSrf) pFaceFromSrf->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 1,0,0) ; sPoint.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Classify the Point against the surface->Face
  SER(pFace->PointClassify(crUVPoint, sSrcZoneTol3d, FALSE, TRUE, sPointClass));

  // set output
  eClassification = sPointClass.GetPointClass();
  rpObjectInOrOn  =  (eClassification != SM_PC_UNKNOWN)
                    ? sPointClass.GetObject()
                    : NULL ;


  // all done
  return SM_SUCCESS;

} // end SmTrimSrfCache::PointClassify

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTrimSrfCache::IsKindOf( SM_TYPE t ) const
{
  return ((SmTrimSrfCache_TYPE == t) ? TRUE : SmSurfaceCache::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Debug formatted print.

NOTES: 
***********************************************************************/
void SmTrimSrfCache::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  SmTArray<void*> sNodes;

  smos_WriteBuffer(_T("\nBegin SmTrimSrfCache Dump()"));

  m_pTree->m_sNodeMgr.GetActiveElements(sNodes);
  smos_sprintf(sBuff,       _T("\nSmTrimSrfCache = 0x%p, Number of Nodes = %ld"),this,sNodes.GetSize());
  smos_sprintf(sBuffForFile,_T("\nSmTrimSrfCache = %s, Number of Nodes = %ld"), _T("notNULL"),sNodes.GetSize());
  smos_WriteBuffer(sBuff, sBuffForFile);

  this->SmSurfaceCache::Dump() ;

  smos_WriteBuffer(_T("\nEnd SmTrimSrfCache Dump()"));

} // end SmTrimSrfCache::Dump

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the Trim surface cache.

NOTES: SmTrimSrfCache has no members - all memory is held in the
 base class SmSurfaceCache.
***********************************************************************/
ULONG SmTrimSrfCache::GetMemoryUsed
  (ULONG &rlMemoryAllocated) 
 const
{
  // pass the call along. only the base SmSurfaceCache holds member memory
  return( SmSurfaceCache::GetMemoryUsed(rlMemoryAllocated) ) ;

} // end SmTrimSrfCache::GetMemoryUsed
