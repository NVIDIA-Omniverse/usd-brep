// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCutter.cpp
* PURPOSE: Source file for SmCutter object and some of its subclasses.
**********************************************************************/

#include "StdAfx.h"
#include <SmCutter.h> 

#ifndef __SMLOOP_H__
 #include <SmLoop.h>
#endif // __SMLOOP_H__

#include <SmBrepCutting.h> 
#include <SmCFace.h>
#include <SmBSplineSurface.h>
#include <SmBSplineCurve.h>
#include <SmAttribute.h>
#include <SmGraphicsExtern.h>
#include <SmMerge.h>

/*******************************************************************//**
PURPOSE: Constructor for a plane cutting operation.

NOTES: 
***********************************************************************/
SmPlaneCutter::SmPlaneCutter
  (const SmPoint3d  & crPlanePoint,    // in : point on infinite plane
   const SmVector3d & crPlaneNormal)   // in : plane normal pointing in direction
                                       //      of halfPlane to be removed from targetBrep
 : m_vPlanePoint(crPlanePoint), 
   m_vPlaneNormal(crPlaneNormal)
{
  SM_ASSERT(m_vPlaneNormal.LengthSquared() > SM_EFF_ZERO_SQ);
  m_vPlaneNormal.Unitize();
  
} // end SmPlaneCutter::SmPlaneCutter constructor

/*******************************************************************//**
PURPOSE: Merge the results of the intersection of a Plane and the
    Brep to be cut into the Brep in the form of new edges and vertices.

NOTES: 
***********************************************************************/
SmStatus SmPlaneCutter::MergeIntersection
 (SmBrep             * pTgtBrep,         // in : SmBrep to cut with this plane
  double               dXSectTol3d,      // in : Min dist between unique points
  SmTArray<SmFace *> * pOptSubsetFaces)  // optional subset of faces, NULL to ignore, default:[NULL]
{
// Remove Composites
//  // allow composites
// #ifndef SM_NO_COMPOSITES   //cbi_CEdge: 22
//  SmTemporaryChangeValue<SmBoolean> sChange(pTgtBrep->m_bMakeComposites,TRUE);
// #endif // SM_NO_COMPOSITES

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

  // init state
  m_apEdgesInCutter.ReSet() ;

  // locals
  ULONG ii, jj ;
  const SmContext * pContext = pTgtBrep->GetContext() ;
  SmSurface       * pPlane   = NULL ;
  SmExtent3d        sBBox ;
  SmExtent3d        sFaceBox ;
  SmTArray<SmFace*> sFaces ;
  SmTArray<SmEdge*> sEdgesInBrep ;

  // make temporary nurb plane which is larger than the Brep BBox.
  SER(pTgtBrep->CalculateBoundingBox(sBBox));
  SER(SmSurface::CreatePlaneFromBBox(*pTgtBrep->GetContext(), // in : context for new object construction 
                                     sBBox,                   // in : Bounding box to create plane through. 
                                     m_vPlanePoint,           // in : Defines base point of an infinite plane 
                                     m_vPlaneNormal,          // in : Defines the normal of an infinite plane 
                                     pPlane)) ;               // out: newly constructed plane limited by plane/BBox intersection 
                                                              //      or NULL when plane does not intersect BBox 
  SmObjDelete CleanUp(pPlane);

  // get target Brep faces
  if (pOptSubsetFaces != NULL) { for(ii=0;ii<pOptSubsetFaces->GetSize();ii++)
                                   { sFaces.Add((*pOptSubsetFaces)[ii]) ; }
                               }
  else                         { pTgtBrep->GetFaces(sFaces) ; }

  // split every target Brep face with this plane
  for(ii=0;ii<sFaces.GetSize();ii++) 
    {
      // locals for plane/surface intersections
      SmFace          * pFace     = sFaces[ii];
      SmSurface       * pSurface  = pFace->GetSurface();
      SmExtent2d        sUVDomain = pFace->GetUVDomain();
      SmTArray<SmFace*> sNewFaces;

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1 ); if(pTgtBrep) pTgtBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,0,1 ); if(pFace) pFace->DrawUV(); sm_GraphicsLoop();
          smgfx_SetLook( 1,4, 0,1,0 ); m_vPlanePoint.Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 3,4, 1,0,1 ); m_vPlaneNormal.Draw(&m_vPlanePoint); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Skip Faces far from cutting plane
      pFace->CalculateBoundingBox( sFaceBox );
      double dDist = sFaceBox.DistanceToPlane( m_vPlanePoint, m_vPlaneNormal );
      if(smos_Fabs(dDist) > dXSectTol3d)
        { continue; }

      // locals for temp geometry
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmCurve*> sUVCurves;
      SmObjsDelete<SmCurve*> sClean3D(&s3DCurves);
      SmObjsDelete<SmCurve*> sCleanUV(&sUVCurves);

      // XSect plane with Face->Surface
      SmApproxTol3d sApproxTol3d = SM_XSECT_TO_APPROXTOL3D(dXSectTol3d) ;
      SER(pSurface->CreatePlanarSectionCurves(*pContext,      // in : context for new object construction  
                                              sUVDomain,      // in : surface domain of interest  
                                              m_vPlanePoint,  // in : Defines base point of an infinite sectioning plane  
                                              m_vPlaneNormal, // in : Defines the unit normal of an infinite sectioning plane  
                                              &sApproxTol3d,  // in :  
                                              NULL,           // in :  
                                              &s3DCurves,     // out: 3D curves produced by sectioning operation  
                                              &sUVCurves));   // out: this surface UVTrimcurves produced by sectioning operation  
      // no work - no cuts
      if ( s3DCurves.GetSize() < 1 )
        { continue; }

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          ULONG di ;
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1 ); if(pTgtBrep) pTgtBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,0,1 ); if(pFace) pFace->DrawUV(); sm_GraphicsLoop();
          smgfx_SetLook( 1,4, 0,1,0 ); m_vPlanePoint.Draw(); sm_GraphicsLoop();
          smgfx_SetLook( 3,4, 1,0,1 ); m_vPlaneNormal.Draw(&m_vPlanePoint); sm_GraphicsLoop();
          smgfx_SetLook( 5,6, 1,0,0 ); for(di=0;di<s3DCurves.GetSize();di++) 
                                         { s3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // locals for intersectionCurve/Surface merges - need SmBSplineCurves to pass in, not SmCurves.
      SmTArray<SmCurve*> sUVBSPCurves;
      for (ULONG k=0; k<sUVCurves.GetSize(); k++) 
        {
          sUVBSPCurves.Add(sUVCurves[k]);
        }

      // Now classify and merge intersections into the Brep
      SmStatus eStat = pTgtBrep->MergeCurvesOnSurface(*pSurface,      // in : Surface must be owned by a face in this Brep.
                                                       dXSectTol3d,   // in : 3D Tol for curves greater than MaxCrvSrfGap and MaxCrvTrimCrvGap.
                                                       s3DCurves,     // in : 3D Curves on surface not necessarily trimmed to face boundary.
                                                                      //      Curves should not cross each other.
                                                       &sUVBSPCurves, // in : associated UV Curves having a dim of 2. 
                                                       sNewFaces,     // out: Newly created faces, if any.
                                                       sEdgesInBrep,  // out: Brep Edges for cr3DCurves (new and/or existing) 
                                                                      //      ordered:[CurveParameritization]
                                                       FALSE,         // in : TRUE = use projection for intersections to allow for tolerances.
                                                                      //      FALSE= no projections for faces already toleranced. default:[FALSE]
                                                       pPlane);       // in : OtherSurface when cr3DCurves are from surf/surf XSects. 
                                                                      //      As curves are merged into Brep - surf/surf/surf XSects are
                                                                      //      used for new verts and to improve Edge->Curve interpolation.
                                                                      //      NULL to ignore, default:[NULL], NULL creates larger gaps.
                                                                      // in : TRUE = try UVSpace Classification if 3Space try is dodgey (tolerant cases)
                                                                      //      FALSE= don't try UVSpace because UVTrimCurves aren't yet valid, default:[TRUE]
                                                    
      // skip Merge calls that failed - Don't abort the entire operation because of a failure here [070310]
      if(eStat != SM_SUCCESS)
        { continue; }

      // accumulate list of Edges added to TgtBrp
      m_apEdgesInCutter.Append( sEdgesInBrep );

    } // end iter every target Brep face

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      pTgtBrep->Dump(SM_BD_POINTS); 

      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); if(pTgtBrep) pTgtBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 1,4, 0,1,0 ); m_vPlanePoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 3,4, 1,0,1 ); m_vPlaneNormal.Draw(&m_vPlanePoint); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // wire edges

  // get modified targetBrep edges
  SmTArray<SmEdge*> sWireEdges;
  pTgtBrep->GetWireEdges(sWireEdges);

  // split every targetBrep wireEdge with this plane
  for(ii=0;ii<sWireEdges.GetSize();ii++) 
    {
      SmEdge *pWireEdge = sWireEdges[ii];

      // for wire edges
      if (pWireEdge->IsWire()) 
        {
          SmSolution sData[16];
          SmSolutionArray sSolutions(16,sData);
          SmCurve * pCurve  = pWireEdge->GetCurve();
          double    dPlaneD = - m_vPlanePoint.Dot(m_vPlaneNormal);

          // find wire/plane intersection points
          SER(pCurve->GlobalPropertyAnalysis(pWireEdge->GetInterval(),  // in : ThisCurve target interval 
                                             SM_CP_PLANE_INTERSECTION,  // in : The property of the curve to extract or find on the curve 
                                             &dPlaneD,                  // in : Value used to specify a particular property. 
                                             &m_vPlaneNormal,           // in : Vectors or points used to define a particular property. 
                                                                        //      If there is a point, that should be the first value in the array 
                                                                        //      If there is a normal (projection), it will come before all other vectors 
                                             dXSectTol3d,               // in : 3D Tolerance used to determine when two answers are equivalent 
                                             sSolutions)) ;             // out: solutions 

          // For every solution - merge points not on the cutplane into TgtBrep
          for(jj=0;jj<sSolutions.GetSize();jj++) 
            {
              SmSolution & rSol           = sSolutions[jj];
              SmVertex   * pV1            = pWireEdge->GetVertex();
              SmVertex   * pV2            = pWireEdge->GetOtherVertex(pV1);
              SmCurve    * pWireEdgeCurve = pWireEdge->GetCurve();
              
              // get edge endPoints, Curve, and SolutionPoint
              SmPoint3d sNewVPnt;
              SER(pWireEdgeCurve->EvaluatePoint(rSol.m_vStart[0], sNewVPnt));

              // when solutionPoint is not close to endPoints
              if (   (  sNewVPnt.DistanceBetween(pV1->GetPoint()) > dXSectTol3d + pV1->GetTolerance()) 
                  && (  sNewVPnt.DistanceBetween(pV2->GetPoint()) > dXSectTol3d + pV2->GetTolerance())) 
                {
                  // split the wire edge with the solutionPoint
                  SmEdge * pNewE1 = NULL ;
                  SmEdge * pNewE2 = NULL ;
                  SmVertex *pNewV = NULL ;
                  SER(pTgtBrep->MakeVertexSplitEdge(pWireEdge,        // in : target edge 
                                                    rSol.m_vStart[0], // in : target parameter 
                                                    pNewE1,           // out: new Edge1 (by chance == pEdgeToSplit) or NULL 
                                                    pNewE2,           // out: new Edge2 (by chance == newly allocated edge) or NULL 
                                                    pNewV));          // out: new vertex (newly allocated) or nearby endVertex or NULL 
#ifdef SM_USE_NEWTOL
                  SM_NEWTOL_LINE pNewV->ClearLocalZoneTol3d( ) ;
#else // SM_USE_OLDTOL
                  SM_OLDTOL_LINE pNewV->SetTolerance(SM_XSECT_TO_ZONETOL3D(dXSectTol3d)) ;
#endif // SM_USE_OLDTOL
                  // Change Edge to point to second edge in split - this
                  // is because solutions are sorted in ascending order.
                  pWireEdge = pNewE2;

                } // end solutionPoint not close to endPoint check
            } // end iter every plane/wire intersection point
        } // end Is Wire check
    } // end iter every edge

  // all done
  return SM_SUCCESS;

} // end SmPlaneCutter::MergeIntersection

/*******************************************************************//**
PURPOSE: Classify all faces, edges, and vertices and determine which
    ones need to be deleted.  Basically for the plane we just have to
    test to see if the point from the given topology is on the positive
    side of the plane equation.  If it is than we can add it to the list
    to be deleted.  

NOTES: increments an unlocked mark value
***********************************************************************/
SmStatus SmPlaneCutter::Classification     
 (SmBrep              * pTgtBrep,          // in : target Brep
  double                dXSectTol3d,       // in : Min dist between unique points
  SmTArray<SmFace*>   & rFacesToRemove,    // out: list of faces    to delete from targetBrep
  SmTArray<SmEdge*>   & rEdgesToRemove,    // out: list of edges    to delete from targetBrep
  SmTArray<SmVertex*> & rVerticesToRemove) // out: list of vertices to delete from targetBrep
{
  SM_ASSERT(pTgtBrep != NULL);

  // init output
  rFacesToRemove.ReSet();
  rEdgesToRemove.ReSet();
  rVerticesToRemove.ReSet();

  // Set a new mark increment and lock an unlocked mark - we are using marking to prevent retracing things
  SmNewMarkAndLock sMarkLock( pTgtBrep->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // locals
  ULONG ii, jj ;
  SmTArray<SmFace*>   sFaces;
  SmTArray<SmEdge*>   sEdges;
  SmTArray<SmVertex*> sVertices;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if ( bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every vertex
  pTgtBrep->GetVertices(sVertices);
  for(ii=0;ii<sVertices.GetSize();ii++) 
    {
      SmVertex *pVertex = sVertices[ii];

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 1,0,0); pVertex->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // get vector from point to plane point
      SmVector3d sVecToPoint = pVertex->GetPoint() - m_vPlanePoint;
      double     dSignedDist = sVecToPoint.Dot( m_vPlaneNormal );

      // if the vertex is on the cutting plane AND ONLY on the one face then remove the vertex
      // This happens when the cutting plane did not split a face
      if (smos_Fabs(dSignedDist) < dXSectTol3d)
        {
          sFaces.RemoveAll();
          pVertex->GetFaces(sFaces);
          if (sFaces.GetSize() == 1 ) 
            {
              rVerticesToRemove.Add(pVertex);
          
#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,4, 1,0,0); pVertex->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            }
          continue;
        } // end Vertex To cutting plane dist < XSextTol check

      // This vertex is definitely above or below the plane.
      // Classify it and all of its connected edges and faces.
      
      // Classify vertex:
      if(dSignedDist > 0.0) 
        {
          // Above the plane: place vertex on remove list
          rVerticesToRemove.Add(pVertex) ;
      
#ifdef SM_DEBUG_CODE
          if(bDebugMe) 
            { 
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,4, 1,0,0); pVertex->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end point above the plane branch 

      // Classify all edges attached to pVertex:
      pVertex->GetEdges(sEdges);
      for(jj=0;jj<sEdges.GetSize();jj++) 
        {
          SmEdge *pEdge = sEdges[jj];

          // skip already processed edges
          if (pEdge->IsMarked(eMarkType)) 
            { continue; }

          // mark edge
          pEdge->Mark(eMarkType);  // Mark the edge
        
          // When edge is attached to vertex above the cutting plane - add edge to remove list
          if (dSignedDist > 0.0) 
            {
              rEdgesToRemove.Add(pEdge);
          
#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(4,5, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            } // end edge attached to removeVertex check
        } // end iter all edges attached to pVertex
      
      // Classify all Faces attached to pVertex:
      sFaces.RemoveAll();
      pVertex->GetFaces(sFaces);
      for(jj=0;jj<sFaces.GetSize();jj++) 
        {
          SmFace *pThisFace = sFaces[jj];
          if (pThisFace->IsMarked(eMarkType)) 
            { continue; }
          pThisFace->Mark(eMarkType);
      
          // When face is connected to a vertex above the cutting plane - mark plane for deletion
          if ( dSignedDist > 0.0 ) 
            { 
              // Add attribute to mark it deleted for future use
              SmLongAttribute *pDeleteAttribute = new (pTgtBrep->GetContext()) SmLongAttribute( SM_AI_BOOLEAN_DELETE, 1, SM_AB_REFERENCE );
              pThisFace->AddAttribute( pDeleteAttribute );
              rFacesToRemove.Add(pThisFace); 
      
#ifdef SM_DEBUG_CODE
              if(bDebugMe) 
                { 
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(4,5, 1,0,0); pThisFace->Draw(SM_DM_CROSSHATCH, 12, 12); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            } // end face attached to a deleteVertex check
        } // end iter all faces attached pVertex
    } // end iter every vertex.

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    { 
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Now look at every edge that wasn't classified from vertex connections.
  pTgtBrep->GetEdges(sEdges);
  for(ii=0;ii<sEdges.GetSize();ii++) 
    {
      SmEdge *pEdge = sEdges[ii];  
              
      // Skip marked edges: already classified in the Vertex loop.
      if (pEdge->IsMarked(eMarkType))
        { continue; }

      pEdge->Mark(eMarkType);

      // evaluate a nearMid Edge Point - project that to the plane
      SmCurve *pCurve = pEdge->GetCurve();
      double   dParam = pEdge->GetInterval().Evaluate(0.45678);  
      SmPoint3d sPnt;
      SER(pCurve->EvaluatePoint(dParam,sPnt));
      SmVector3d sVecToPoint = sPnt - m_vPlanePoint;
      double dSignedDist = sVecToPoint.Dot( m_vPlaneNormal );

      // Be loose with this tolerance, because they will be caught when checking faces.
      if ( smos_Fabs( dSignedDist ) < pEdge->GetTolerance() + dXSectTol3d )
        {
          // Try one more interior point to make sure.
          dParam = pEdge->GetInterval().Evaluate(0.65432);
          SER(pCurve->EvaluatePoint(dParam,sPnt));
          sVecToPoint = sPnt - m_vPlanePoint;
          dSignedDist = sVecToPoint.Dot( m_vPlaneNormal );

          // Edge is contained in plane: if edge has only one face then cutting plane did not split a face 
          // so remove the edge
          if ( smos_Fabs( dSignedDist ) < pEdge->GetTolerance() + dXSectTol3d )
            {
              sFaces.RemoveAll();
              pEdge->GetFaces(sFaces);
              if (sFaces.GetSize() == 1) 
                { 
                  
                  // Check for previously applied attribute on face to remove
                  SmAttribute* pAttr = sFaces[0]->FindAttribute( SM_AI_BOOLEAN_DELETE );
                  if(pAttr != NULL)
                    {
                      rEdgesToRemove.Add( pEdge );
                    }
                  
#ifdef SM_DEBUG_CODE
                  if (bDebugMe) 
                    {
                      smgfx_Erase();
                      smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(2,3, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                }
              continue;
            }
        }

      // This Edge is definitely above or below the plane.
      // Classify it and all of its connected faces.

      // Classify this edge:
      if ( dSignedDist > 0.0 )
        {
          // Above the plane: remove this Edge.
          rEdgesToRemove.Add(pEdge);

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            { 
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        }

      // Classify all Faces attached to this Edge:
      pEdge->GetFaces(sFaces);
      for(jj=0;jj<sFaces.GetSize();jj++) 
        {
          SmFace *pThisFace = sFaces[jj];
          if (pThisFace->IsMarked(eMarkType)) 
           { continue ; }

          pThisFace->Mark(eMarkType);

          // Classify this face:
          if ( dSignedDist > 0.0 ) 
            { 
              rFacesToRemove.Add(pThisFace); 
            }

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            { 
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 1,0,0); pThisFace->Draw(SM_DM_CROSSHATCH, 12, 12); sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end iter all faces attached to pEdge
    } // end iter every edge

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Finally we have to pick up faces which have no edge or vertices
  // that are not on the plane.  These are the dome cases where 
  // a dome of a face is cut with the plane.  Since all of their
  // edges and vertices are in the plane, we'll look at the
  // binormals of the edges: pointing up or down w.r.t the plane.

  pTgtBrep->GetFaces( sFaces );
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      SmFace *pFace = sFaces[ii];

      // skip faces already marked
      if(pFace->IsMarked( eMarkType )) continue;

      // remember this face has been processed
      pFace->Mark( eMarkType );

      // for every edgeuse on the face's upward faceuse
      SmFaceuse *pFU = pFace->GetUpwardFaceuse();
      SmTArray<SmEdgeuse*> sEdgeuses;
      pFU->GetEdgeuses( sEdgeuses );
      for(jj=0;jj<sEdgeuses.GetSize();jj++)
        {
          SmEdgeuse *pEU = sEdgeuses[jj];

          // get edgeuse midPoint binormal vector
          SmEdge    *pEdge = pEU->GetEdge();
          SmPoint3d  sBinPoint;
          SmVector3d sBinormalVector;
          SER( pEU->EvaluateBinormal( pEdge->GetInterval().Evaluate( 0.5 ),  // in : value within Edge->m_vInterval.                                                  
                                      FALSE,                                 // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve  
                                                                             // in : FALSE = get/create pUVTrimCurve to calc Surface points                           
                                      sBinPoint,                             // out: 3D pt on edge                                                                    
                                      sBinormalVector ) );                   // out: unit-vector pointing to SmFace interior from rBinormalPoint                      

          // classify the binormal direction against the plane's normal direction
          double dDot = sBinormalVector.Dot( m_vPlaneNormal );

          // skip edges where the binormal vector is nearly tangent to the plane
          if(smos_Fabs( dDot ) < SM_EFF_ZERO_SQRT)
            {
              // too close to call try another edge
              continue;
            }

          // when face is above plane
          if(dDot > 0.0)
            {
              rFacesToRemove.Add( pFace );

#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 1,0,0); pFace->Draw( SM_DM_CROSSHATCH, 12, 12 ); sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            }

          break;  // go to next face

        } // end iter every upward faceuse->edgeuse 

      // arrive here when all edgesuse midPoint binormals are tangent to the plane.
      // gwc:todo Add check for face/plane coincidence check

    } // end iter every face

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); for(ii=0;ii<rFacesToRemove.GetSize();ii++) 
                                    { rFacesToRemove[ii]->Draw(SM_DM_CROSSHATCH, 12, 12); sm_GraphicsLoop() ; }
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); pTgtBrep->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<rEdgesToRemove.GetSize();ii++) 
                                   { rEdgesToRemove[ii]->Draw(); sm_GraphicsLoop() ; }

      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPlaneCutter::Classification

/********************************************************
PURPOSE: Draw cutter plane

NOTES:
********************************************************/

void SmPlaneCutter::Draw
  (SmBrep *pTgtBrep)       // in : Target Brep to size plane icon
  const
{
  SmCutter::Draw( pTgtBrep );

  // build the plane as a surface
  SmExtent3d sBrepToCutBBox ;
  SmSurface *pCutPlane ;
  if(pTgtBrep) { pTgtBrep->CalculateBoundingBox(sBrepToCutBBox) ;
                 }
  else           { sBrepToCutBBox.SetMinMax(SmPoint3d(-10.0, -10.0, -10.0), 
                                            SmPoint3d(10.0, 10.0, 10.0)) ;
                 }
  SmSurface::CreatePlaneFromBBox(*pTgtBrep->GetContext(), 
                                 sBrepToCutBBox, 
                                 m_vPlanePoint, 
                                 m_vPlaneNormal, 
                                 pCutPlane) ;
 
  // draw and delete the surface
  if(pCutPlane) { pCutPlane->Draw() ;
                  delete pCutPlane ; 
                  pCutPlane = NULL ; 
                }

} // end SmPlaneCutter    


/*******************************************************************//**
PURPOSE: Constructor for a BiLinear Surface cutting operation.

NOTES: 

  see  CreateBilinearSurface(*pContext, m_vP0, m_vP1, m_vP3, m_vP2, m_vSrf);
  to get order of points to define surface.
  Order of P0->P3 is in cw order around uv domain
***********************************************************************/
SmSurfaceBiLinearCutter::SmSurfaceBiLinearCutter
  (const SmPoint3d  & P0,
   const SmPoint3d  & P1,
   const SmPoint3d  & P2,
   const SmPoint3d  & P3)
{
  m_vP0 = P0;
  m_vP1 = P1;
  m_vP2 = P2;
  m_vP3 = P3;
  m_vSrf = NULL;
  m_pCutterAttribute = NULL ;

} // end SmSurfaceBiLinearCutter constructor

/*******************************************************************//**
PURPOSE: Destructor for a BiLinear Surface cutting operation.

NOTES: 
***********************************************************************/
SmSurfaceBiLinearCutter::~SmSurfaceBiLinearCutter()
{
  if(m_vSrf && m_vSrf->GetOwner() == NULL) { delete m_vSrf ; m_vSrf = NULL ; }
  if(m_pCutterAttribute) { delete m_pCutterAttribute ; m_pCutterAttribute = NULL ; }

} // end ~SmSurfaceBiLinearCutter

/*******************************************************************//**
PURPOSE: Merge the results of the intersection of a BiLinear Surface and the
    Brep to be cut into the Brep in the form of new edges and vertices.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceBiLinearCutter::MergeIntersection
  (SmBrep             * pTgtBrep,        // in : SmBrep to cut with this plane
   double               dXSectTol3d,     // in : Min dist between unique points
   SmTArray<SmFace *> * pOptSubsetFaces) // NotUsed: in : optional subset of faces, NULL to ignore, default:[NULL]
{
  SM_REF1(pOptSubsetFaces) ;

  // locals
  ULONG ii ;
  const SmContext *pContext= pTgtBrep->GetContext() ;
  SmTArray <SmFace *> sBrep2Faces ;
    
  // Define cutting brep as BiLinear surface
  SmBrep *pBrep2 = new (*pContext) SmBrep(); 
  SmObjDelete sClean( pBrep2 );
  pBrep2->SetTolerance(dXSectTol3d);
  
  // note order of points in BiLinear (not ccw or cw)
  SmBSplineSurface::CreateBilinearSurface(*pContext, m_vP0, m_vP1, m_vP3, m_vP2, m_vSrf);
  SmBSplineSurface * pSrf = new ( *pContext ) SmBSplineSurface( *m_vSrf );
  SmFace *pF0 = NULL;
  pBrep2->CreateFaceFromSurface(pSrf, pSrf->GetNaturalUVDomain(), pF0) ;

  // Get Cutter Faces
  pBrep2->GetFaces(sBrep2Faces) ;
  SM_ASSERT_MSG(sBrep2Faces.GetSize() == 1, _T("SmSurfaceBiLinearCutter::MergeIntersection - assumption that cutter has just one face is wrong - change attribute type from SmAttribute to SmPointAttribute to track individual faces uniquely")) ;

// Remove Composites - Added next line to replace tracking merged Faces that came from Cut Face with attributes instead of composite faces
  // create attribute that will track merged Cutter face children in the TgtBrep
  m_pCutterAttribute = new (pContext) SmAttribute(SM_AI_REPLACE_COMPOSITES,     // oneof of the SM_AI_ macro type names. A unique id for each attribute type - not each attribute. 
                                                  SM_AB_STANDALONE_REFERENCE) ; // oneof SM_AB_COPY                  - one geom obj per attrib obj   - auto deleted  
                                                                                //       SM_AB_REFERENCE             - many geom objs per attrib obj - auto deleted  
                                                                                //       SM_AB_STANDALONE_COPY       - same as COPY      - not deleted if user_count goes to zero  
                                                                                //       SM_AB_STANDALONE_REFERENCE  - same as REFERENCE - not deleted if user_count goes to zero  
                                                                                //       SM_AB_TEMP                  - Not Copied, Not Persistent  
                                                  
  // for every Brep2 Face - add a SM_AI_REPLACE_COMPOSITES attribute
  for(ii=0;ii<sBrep2Faces.GetSize();ii++)
    {
      SmFace * pFace = sBrep2Faces[ii] ;
      pFace->AddAttribute(m_pCutterAttribute) ; 
    } // end adding attribute to every Cutter Brep Face
  
  // merge parameters
  double dAngleTol = 20.0*SM_PI/180.0 ;
  double approxTol = 0.0001 ;
  SmBrep* pResult ;
  
  // merge executive object
  SmMerge sMerge(*pContext, pTgtBrep, pBrep2, approxTol, dAngleTol) ;

  // Do the boolean merge
  sMerge.NonManifoldBoolean(SM_BO_PARTIAL_MERGE, pResult) ;
  
  // expect large cutter face to be split into small children - reduce face sizes to their trim boundaries for stability 
  pTgtBrep->ShrinkGeometry() ; 
  
  // all done
  return(SM_SUCCESS) ;

} // end SmSurfaceBiLinearCutter::MergeIntersection

/*******************************************************************//**
PURPOSE: Classify all faces and determine which ones need to be deleted.  
    Basically for the bilinear cutter we just have to test to see if 
    the point from the given topology is on the positive side of the 
    surface from the projection of the point down to  the surface.
    If it is then we can add it to the list of faces to be deleted.  

NOTES: 
    This behaves differently from SmPlaneCutter in that
    It leaves 'inner loop' faces from the cut surface
***********************************************************************/
SmStatus SmSurfaceBiLinearCutter::Classification
 (SmBrep                * pTgtBrep,           // in : target Brep
  double                  dXSectTol3d,        // in : Min dist between unique points
  SmTArray < SmFace*>   & rFacesToRemove,     // out: Faces    to delete from TgtBrep
  SmTArray < SmEdge*>   & rEdgesToRemove,     // out: Edges    to delete from TgtBrep
  SmTArray < SmVertex*> & rVerticesToRemove)  // out: Vertices to delete from TgtBrep
{
  SM_ASSERT(pTgtBrep != NULL);

  // init output
  rFacesToRemove.ReSet();
  rEdgesToRemove.ReSet();
  rVerticesToRemove.ReSet();

  // set Brep editing
  pTgtBrep->m_bEditingEnabled = TRUE ; 
    
  // locals
  ULONG ii, jj, kk ;
  SmExtent2d              sUVDomainSrf = m_vSrf->GetNaturalUVDomain();
  SmFace                * pCutterFace  = NULL ;
  SmPoint3d               Pnt ;
  SmTArray <SmFace *>     sFaces ;        
  SmTArray <SmFace *>     sFaces1 ;
  SmTArray <SmAObject *>  sCutterAObjects;      // will hold all m_pCutterAttribute Users which are expected to all be SmFaces ptrs
  SmTArray <SmFace *>     sCutterFaces;      // will hold all m_pCutterAttribute Users which are expected to all be SmFaces ptrs
  SmTArray <SmFace *>     sCommonFaces;
  SmTArray <SmFace *>     sFacesToDelete;   
  SmTArray <SmFace *>     sFacesToDelete1;  
  SmTArray <SmEdge *>     sEdges;
  SmTArray <SmEdge *>     sEdges1;
  SmTArray <SmEdgeuse *>  sEdgeuses;
  SmTArray <SmLoop *>     sLoops;    
  SmTArray <SmLoop *>     sLoops1;     
  SmSolution aData[10]; 
  SmSolutionArray         sSolutions(10, aData);
  SmTArray <SmVertex *>   sVerts;
  SmTArray <SmVertex *>   sVertices ;
  pTgtBrep->GetVertices  (sVertices) ;

  // check state - SmSurfaceBiLinearCutter should have an attribute
  SER_MSG((m_pCutterAttribute != NULL) ? SM_SUCCESS : SM_ERR_ASSERT_FAILURE, 
          _T("SmSurfaceBiLinearCutter::Classification error - m_pCutterAttribute is NULL - this should be set in SmSurfaceBiLinearCutter::MergeIntersection.")) ; 

  // find all the faces in TgtBrep that came from the cutter face
  m_pCutterAttribute->GetUsers( (SmTArray<SmAObject *> &)sCutterAObjects ) ;

  // low work - CutterFace did not intersect any TgtBrep geometry - Brep is either all cut or all saved
  SmBoolean bNoCuts = sCutterAObjects.GetSize() == 0 ; 
  SM_REF1(bNoCuts) ;
  SM_ASSERT_MSG(bNoCuts == FALSE, _T("SmSurfaceBiLinearCutter::Classification: error - current implementation does not work correctly when bNoCuts = TRUE ;  needs rewrite")) ;
   
  // build sCutterFaces - for strong type casting
  for(ii=0;ii<sCutterAObjects.GetSize();ii++)
    {
      if(sCutterAObjects[ii]->GetType() == SmFace_TYPE)
        { sCutterFaces.Add((SmFace *)sCutterAObjects[ii]) ; }
    }

  // find any outer-most child face piece of the original cutter face - the piece of the cutter face outside the TgtBrep is going to be removed
  for(ii=0;ii<sVertices.GetSize();ii++)
    {
      SmVertex * pVertex = sVertices[ii] ;
      Pnt                = pVertex->GetPoint() ;

      // skip vertices that are not on the original cutting surface corner
      if(   Pnt.x != m_vP0.x    // gwc: this is an exact check - should there be a tolerance
         || Pnt.y != m_vP0.y    //      maybe not because this position was copied into the TgtBrep.
         || Pnt.z != m_vP0.z)   //      Here the position is being used as a name and not a geometry location.
        { continue ; }          //      maybe this all could be replaced with a simple collection of current pTgtBrep faces
                                //      that came from the original cutter face brep - that could be done with an attribute.

      // get faces attached to vertex known to be a corner of the original cutter face
      pVertex->GetFaces(sFaces) ;

      // check state - the cutter fadce corner should only be attached to one face - a piece of the original cutter face
      if (sFaces.GetSize() != 1)  // more than one face for that vertex
        { continue; }

      // remember face so it can be removed
      pCutterFace = sFaces[0] ;  
      break ;
    } // end iter all vertices
    
  //check state - should have found a outer-most piece of the original cutter face
  if (pCutterFace == NULL)
    { 
      // error, must be a face with that vertex point
      return 1 ; 
    } 

  // Add 1ast outer-most cutter face child to Remove list
  rFacesToRemove.Add(pCutterFace);
    
  // add face to find neighbor faces to remove queue 
  sFacesToDelete1.Add(pCutterFace);
    
  // while there are faces in the sFacesToDelete1 queue - 
  while (sFacesToDelete1.GetSize() > 0)
    {
      // move the faces to delete from list1 to list
      sFacesToDelete.ReSet();
      sFacesToDelete.Append(sFacesToDelete1) ;
      sFacesToDelete1.ReSet();

      // for every candidate faceToDelete - don't delete faces inside original Brep loops
      for(ii=0;ii<sFacesToDelete.GetSize();ii++)
        {
          SmFace *pFaceToDelete = sFacesToDelete[ii];
          pFaceToDelete->GetLoops(sLoops);

          // for every FaceToDelete->Loop - find neighbor faces
          for(jj=0;jj<sLoops.GetSize();jj++)
            {
              sLoops[jj]->GetEdges(sEdges);
              SmEdge * pLoop1stEdge = sEdges[0];

              // find the 1st Edge's neighboring faces that were part of the original cutter face
              pLoop1stEdge->GetFaces(sFaces);
              sFaces.FindCommonElements(sCutterFaces, sCommonFaces);
                
              // skip edges not a topologicial edge of the original cutter face
              if (sCommonFaces.GetSize() != 2)
                { return 1 ; }

              // Find the neighbor face to pFaceToDelete that's part of the original cutter face
              SmFace * pFaceToUse = (sCommonFaces[0] == pFaceToDelete) ? sCommonFaces[1] : sCommonFaces[0];
                
              // pFaceToUse is a face that is not to be deleted but if it has inner loops those define faces to delete
              pFaceToUse->GetLoops(sLoops1);
                
              // the faces of these inner loops are to be removed
              for(kk=1;kk<sLoops1.GetSize();kk++)
                {
                  sLoops1[kk]->GetEdges(sEdges1);
                  pLoop1stEdge = sEdges1[0];

                  // find the neighboring face
                  pLoop1stEdge->GetFaces(sFaces1);
                  SmTArray < SmFace *> sCommonFaces1;
                  sFaces1.FindCommonElements(sCutterFaces, sCommonFaces);
                    
                  // skip edges not a topologicial edge of the original cutter face
                  if (sCommonFaces.GetSize() != 2)
                    { return 1 ; }

                  // Find the neighbor face to pFaceToUse that's part of the original cutter face
                  SmFace *pFaceToDelete1 =  (sCommonFaces[0] == pFaceToUse) ? sCommonFaces[1] : sCommonFaces[0];
                    
                  // pFaceToDelete1 is a face that is to be deleted 
                  sFacesToDelete1.Add(pFaceToDelete1); 
                  
                  // add it to the face queue so if it has inner loops those define faces to keep
                  rFacesToRemove.Add(pFaceToDelete1) ;

                } // end iter all inner loop1s
            } // end iter all loops
        } // end iter all faces to delete
    } // end iter all faces to delete1

  // we need to identify and remove those faces which are on top(or bottom) 
  // side of the cutting surface
    
  // Get  a vertex from each face, drop to the surface, and see what side it is on wrt
  //   the surface normal at the drop point.......if unknown ( =ON) get the next vertex
  pTgtBrep->GetFaces(sFaces);

  // for every TgtBrep face - add faces on the positive side of the cutter face to the remove list
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      // no work - current face is the pCutterFace - already on the removed list
      if(pCutterFace == sFaces[ii])
        { continue ; } 
        
      // locals
      SmFace * pFace = sFaces[ii] ;
      pFace->GetVertices(sVerts);
      SmBoolean AllOn = TRUE;

      // for every Face->Vertex - classify vertex by dropping it to cutting face
      for(jj=0;jj<sVerts.GetSize(); jj++)
        {
          SmVertex * pVertex = sVerts[jj];
          SmPoint3d  sVPoint = pVertex->GetPoint();
           
          // find the closest point on the cutting surface
          double dTol = pVertex->GetTolerance() + dXSectTol3d;          
          SmSolution Sol;
          
          // drop VertexPoint to cutting surface
          m_vSrf->GlobalPointSolve(sUVDomainSrf,   // in : Domain of surface to search for solutions                                                      
                                   SM_SO_MINIMIZE, // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT                        
                                   sVPoint,        // in : Target point for the solve operation                                                           
                                   dXSectTol3d,    // in : Obj ZoneTol3d assoc with Target Point, if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL) 
                                   NULL,           // in : Max/Min Drop distance for min/max and normalize operations. NULL to ignore.                    
                                   SM_SR_SINGLE,   // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                                    
                                   sSolutions) ;   // out: array of problem solutions reported as surface UV parameter values                             
          // drop distance
          Sol = sSolutions[0];
          double Dist = smos_Fabs(Sol.m_vStart.m_dSolutionValue) ;
          
          // skip vertices that are 'on' the cutting plane - faces on Plane have already been classified
          if (Dist < dTol)
            { continue ; }
          
          // remember that a Face->Vertex not on the cutting surface was found
          AllOn = FALSE;  
          
          // Drop SurfacePt and Drop Vector
          SmPoint3d SrfPoint;
          SmPoint2d sUV(Sol.m_vStart[0], Sol.m_vStart[1]);
          m_vSrf->EvaluatePoint(sUV, SrfPoint);
          SmVector3d Vec1 = sVPoint - SrfPoint;
          Vec1.Unitize();
          
          // Drop SurfacePt Normal
          SmPoint3d SrfNorm;
          m_vSrf->EvaluateNormal(sUV, TRUE, TRUE, SrfNorm);
          SrfNorm.Unitize();
          
          // sign of Dot Product(DropVec, SrfNormal) classifies the Vertex as Above or Below cutting plane
          double dDot = Vec1.Dot(SrfNorm);
          
          // remove faces on positive side of the cutting plane
          if(dDot > 0.0)
            {
              rFacesToRemove.Add(pFace); 
            }
          
          // done with this face
          break ;

        } // end iter all face vertices looking for one to classify the face

    // when all Face Vertices are on the cutting surface - face may stilol be removed - think Dome rising above the cutting plane
    if (AllOn) 
      {
         SmFaceuse * pFU = pFace->GetUpwardFaceuse();
         pFU->GetEdgeuses(sEdgeuses);

         // for every edgeuse on the face's upward faceuse
         for(jj=0;jj<sEdgeuses.GetSize();jj++) 
           {
             SmEdgeuse * pEU   = sEdgeuses[jj];
             SmEdge    * pEdge = pEU->GetEdge();
             SmPoint3d   sEPoint;
             SmVector3d  sBinormalVector;

             // get edgeuse midPoint binormal vector
             SER(pEU->EvaluateBinormal(pEdge->GetInterval().Evaluate(0.5), // in : value within Edge->m_vInterval. 
                                       FALSE,                              // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve  
                                                                           //      FALSE = get/create pUVTrimCurve to calc Surface points 
                                       sEPoint,                            // out: 3D pt on edge 
                                       sBinormalVector) ) ;                // out: unit-vector pointing to SmFace interior from rBinormalPoint 

             // skip edges that don't generate a NonZero Binormal vector
             if (sBinormalVector.Unitize() != SM_SUCCESS) 
               { continue ; }
           
             // find the closest point on the cutting surface          
             m_vSrf->GlobalPointSolve(sUVDomainSrf,   // in : Domain of surface to search for solutions                                                      
                                      SM_SO_MINIMIZE, // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT                        
                                      sEPoint,        // in : Target point for the solve operation                                                           
                                      dXSectTol3d,    // in : Obj ZoneTol3d assoc with Target Point, if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL) 
                                      NULL,           // in : Max/Min Drop distance for min/max and normalize operations. NULL to ignore.                    
                                      SM_SR_SINGLE,   // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                                    
                                      sSolutions);    // out: array of problem solutions reported as surface UV parameter values
                                      
             // Cutting Surface Normal at closest point                         
             SmSolution Sol = sSolutions[0];
             SmPoint2d sUV(Sol.m_vStart[0], Sol.m_vStart[1]);
             SmPoint3d SrfNormal;
             m_vSrf->EvaluateNormal(sUV, TRUE, TRUE,SrfNormal);

             // skip PointDrops that don't generate a NonZero cutting surface normal vector
             if (SrfNormal.Unitize() != SM_SUCCESS) 
               { continue ; }
           
             // classify the binormal direction against the plane's normal direction
             double dDot = sBinormalVector.Dot(SrfNormal);
           
             // remove faces that are on pos side of the cutting plane
             if(dDot > 0.0)
               {
                 rFacesToRemove.Add(pFace); 
               }

             // done with this face
             break ;

           } // end iter jj, every upward faceuse->Edgeuse

      } // end all face vertices are on the cutting surface check - watch for domes rising above the cutting surface to be removed
  } // end iter every TgtBrep Face making Keep/Delete classifications
       
  // all done
  return SM_SUCCESS ;

} // end SmSurfaceBiLinearCutter::Classification

/********************************************************
PURPOSE: Draw cutter bilinear Surface

NOTES: gwc: draw function needs to be rewritten to current draw function style
********************************************************/
void SmSurfaceBiLinearCutter::Draw
  (SmBrep *pTgtBrep)       // in : Target Brep to size plane icon
  const
{
  SmCutter::Draw(pTgtBrep) ;
  if(m_vSrf) { m_vSrf->Draw() ; }
  
} // end SmSurfaceBiLinearCutter    

/********************************************************
PURPOSE: Draw generic cutter

NOTES: gwc: draw function needs to be rewritten to current draw function style
********************************************************/
void SmCutter::Draw
  (SmBrep *pTgtBrep) // NotUsed: in :
  const
{
  SM_REF1(pTgtBrep) ;

  for(ULONG ii=0;ii<m_apEdgesInCutter.GetSize();ii++)
    {
      m_apEdgesInCutter[ii]->Draw() ;
    }

} // end SmCutter::Draw

