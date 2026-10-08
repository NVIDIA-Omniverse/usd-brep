// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************/
/* FILE NAME --- SmStitch.cpp                                           */
/* PURPOSE: Source file for SmStitch class methods.                     */
/*********************************************************************/

#include "StdAfx.h"

#include <SmStitch.h>

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

#include <SmTree.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>


//#define SM_VALIDATE_TOPOLOGY 1

/*******************************************************************//**
PURPOSE: Default mechanism for selection of topology to keep.

NOTES:
  Input SmTopology objects are expected to be two coincident
  objects of the same type, vertices or edges, else always
  just keep element1. 

METHOD ---
  1. When vertices are close Keep vertex with most connected edges.
  2. when vertices are not within Brep tolerance of one another
       keep the one connected to the most edges to be kept
  3. when edges - Keep the edge with the most faces or the least number of knots
      If the Distance between the edges is somewhat large then do some
      computational comparisons to see which is the better 3D edge to
      keep.
***********************************************************************/
SmStatus SmStitchCallback::SelectTopologyToKeep
  (SmTopology * cpTopologyElement1,          // in : 1st of two coincident objects to stitch together
   SmTopology * cpTopologyElement2,          // in : 2nd of two coincident objects to stitch together
   double       dDistBetween,                // in : max distance between coincident objects
   double     & rdMaxDistance,               // out: max distance found between two target objects
   SmBoolean  & rbKeepElement1)              // out: TRUE = keep TopologyElement1
                                             //      FALSE= keep TopologyElement2
{
  ULONG ii, jj ;

  // init output
  rbKeepElement1 = TRUE;
  rdMaxDistance  = dDistBetween;
  // check for vertex
  SmVertex *pVertex1 = SM_CAST_PTR(SmVertex,cpTopologyElement1);

  // Keep vertex which has the most selected edges.
  if (pVertex1) 
    {
      // 2nd element is expected to be a vertex
      SmVertex *pVertex2 = SM_CAST_PTR(SmVertex,cpTopologyElement2);
      NER(pVertex2);

      // get edges connected to pVertex1
      SmEdge *sEData1[64];
      SmTArray<SmEdge*> sVEdges1(64,sEData1);
      pVertex1->GetEdges(sVEdges1);
      ULONG pV1EdgeCount = 0;

      // get edges connected to pVertex2
      SmEdge *sEData2[64];
      SmTArray<SmEdge*> sVEdges2(64,sEData2);
      pVertex2->GetEdges(sVEdges2);
      ULONG pV2EdgeCount = 0;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          ULONG di ; 
          smgfx_Erase();
          smgfx_SetLook(5,6, 1,0,0) ; pVertex1->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 1,0,1) ; for(di=0; di<sVEdges1.GetSize(); di++) 
                                       { sVEdges1[di]->Draw(); sm_GraphicsLoop(); }
          
          smgfx_SetLook(6,7, 0,1,0) ; pVertex2->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,.5,1); for(di=0; di<sVEdges2.GetSize(); di++) 
                                       { sVEdges2[di]->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // get Brep
      SmBrep *pBrep = pVertex1->GetBrep();

      // when distance between vertices is much larger than Brep Tolerance
      if (dDistBetween > pBrep->GetTolerance() * 10.0) 
        {
          // for every v1 edge
          for(ii=0; ii<sVEdges1.GetSize(); ii++) 
            {
              SmEdge *pV1Edge = sVEdges1[ii];
              if (pV1Edge->IsLamina()) 
                {
                  // for every v2->edge
                  for(jj=0; jj<sVEdges2.GetSize(); jj++) 
                    {
                      SmEdge *pV2Edge = sVEdges2[jj];

                      // skip edges between v1 and v2
                      if (pV2Edge == pV1Edge) 
                        { continue ; }
                      
                      // skip edges which don't connect to common vertices
                      //  edges connecting to a common vertex get stitched: see which one is to be saved
                      if (pV2Edge->GetOtherVertex(pVertex2) != pV1Edge->GetOtherVertex(pVertex1)) 
                        { continue ; }

                      // get curve/curve max distance
                      double dMaxDist = 0.0;
                      double dDistToCurve = 0.0;
                      double dTStart2 = pV2Edge->GetInterval().GetMin();
                      double dTEnd2   = pV2Edge->GetInterval().GetMax();
                      double dTol1    = pV1Edge->GetTolerance();
                      SER(pV1Edge->GetCurve()->CurveMaxDistanceBetween
                               ( pV1Edge->GetInterval(), // in : interval limit for this curve                           
                                *pV2Edge->GetCurve(),    // in : other curve to test                                     
                                 dTStart2,               // in : OtherCurve param mapping to ThisCurve Interval.Min value
                                 dTEnd2,                 // in : OtherCurve param mapping to ThisCurve Interval.Max value
                                 10,                     // in : Min number of samples to take - it measures at least this many points
                                &dTol1,                  // in : Max allowed gap.  Quit searching once this value is exceeded.
                                 dDistToCurve));         // out: Set to max gap size seen.

                      // skip coincident curves
                      if (dDistToCurve < dTol1/1000.0) 
                        { continue; } // Same curve doesn't make any difference

                      // skip curves more than tol apart; they don't get stitched
                      if (dDistToCurve > dTol1) 
                        { continue; }

                      // count up the vertex with the most edges to keep
                      SmBoolean bKeepEdge1;
                      SER(SelectTopologyToKeep(pV1Edge,      // in : 1st of two coincident objects to stitch together 
                                               pV2Edge,      // in : 2nd of two coincident objects to stitch together 
                                               dDistToCurve, // in : max distance between coincident objects       
                                               dMaxDist,     // out: max distance found between two target objects 
                                               bKeepEdge1)); // out: TRUE = keep TopologyElement1                  
                                                             //      FALSE= keep TopologyElement2                  
                      if (bKeepEdge1) 
                        { pV1EdgeCount ++; }

                    } // end iter every v2->edge
                } // end IsLamina branch
              else 
                {
                  // count all nonWire-nonLamina edges
                  if (!pV1Edge->IsWire()) 
                    { pV1EdgeCount ++; }

                } // end not Lamina edge branch
            } // end iter every V1->Edge

          // for every v2->edge - count all nonWire-nonLamina edges and preferred Lamina Edges
          for(ii=0; ii<sVEdges2.GetSize(); ii++) 
            {
              SmEdge *pV2Edge = sVEdges2[ii];
              if (pV2Edge->IsLamina()) 
                {
                  for(jj=0; jj<sVEdges1.GetSize(); jj++) 
                    {
                      // for every v1->edge
                      SmEdge *pV1Edge = sVEdges1[jj];

                      // skip edges between v1 and v2
                      if (pV2Edge == pV1Edge) 
                        { continue; }

                      // skip edges which don't connect to common vertices
                      //  edges connecting to a common vertex get stitched: see which one is to be saved
                      if (pV2Edge->GetOtherVertex(pVertex2) != pV1Edge->GetOtherVertex(pVertex1)) 
                        { continue; }

                      // get curve/curve max distance
                      double dDistToCurve = 0.0;
                      double dTStart2 = pV2Edge->GetInterval().GetMin();
                      double dTEnd2   = pV2Edge->GetInterval().GetMax();
                      double dTol1    = pV1Edge->GetTolerance();
                      SER(pV1Edge->GetCurve()->CurveMaxDistanceBetween
                              ( pV1Edge->GetInterval(),  // in : interval limit for this curve                                                                            
                               *pV2Edge->GetCurve(),     // in : other curve to test                                                                                      
                                dTStart2,                // in : OtherCurve param mapping to ThisCurve Interval.Min value                                                 
                                dTEnd2,                  // in : OtherCurve param mapping to ThisCurve Interval.Max value                                                 
                                10,                      // in : Min number of samples to take - it measures at least this many points                                    
                               &dTol1,                   // in : Max allowed gap.  Quit searching once this value is exceeded.                                            
                                dDistToCurve));          // out: Set to max gap size seen.                                                                                
                                                                                                                                                                          
                      // skip coincident curves
                      if (dDistToCurve < dTol1/1000.0) 
                        { continue; } // Same curve doesn't make any difference 

                      // skip curves more than tol apart; they don't get stitched
                      if (dDistToCurve > dTol1) 
                        { continue; }

                      // count up the vertex with the most edges to keep
                      SmBoolean bKeepElement1;
                      double dMaxDist;
                      SER(SelectTopologyToKeep(pV1Edge,          // in : 1st of two coincident objects to stitch together 
                                               pV2Edge,          // in : 2nd of two coincident objects to stitch together 
 /* GWC: why isn't this dDistToCurve */        dDistBetween*2.0, // in : max distance between coincident objects       
                                               dMaxDist,         // out: max distance found between two target objects 
                                               bKeepElement1));  // out: TRUE = keep TopologyElement1                  
                                                                 //      FALSE= keep TopologyElement2 
                      if(bKeepElement1==FALSE)                 
                        {                     
                          pV2EdgeCount ++;    
                        }                     
                    } // end iter every v1->edge
                }  // end IsLamina branch
              else 
                {
                  // count all nonWire-nonLamina edges
                  if (!pV2Edge->IsWire()) 
                    {
                      pV2EdgeCount ++;
                    }
                } // end not Lamina edge branch
            } // end iter every V2->Edge

          // keep the vertex with the highest keep-edge count
          if (pV2EdgeCount > pV1EdgeCount) 
            {
              rbKeepElement1 = FALSE;
            }
        } // end distance between vertices is larger than Brep Tolerance * 10 branch
      else // vertices are within 10*BrepTol
        {
          // keep the vertex with the most edges
          if (sVEdges2.GetSize() > sVEdges1.GetSize()) 
            {
              rbKeepElement1 = FALSE;
            }
        } // end vertices are within 10*BrepTol branch

      // all done
      return SM_SUCCESS;

    } // end 1st element is a vertex check
  
  // Keep the edge with the most faces or the least number of knots.
  // If the Distance between the edges is somewhat large then do some
  // computational comparisons to see which is the better 3D edge to keep.

  SmEdge *pEdge1 = SM_CAST_PTR(SmEdge,cpTopologyElement1);
  if ( pEdge1 != NULL )
    {
      // 2nd element is expected to be an edge
      SmEdge *pEdge2 = SM_CAST_PTR(SmEdge,cpTopologyElement2);
      NER(pEdge2);

      // get pEdge1 faces
      SmFace *sFData1[64];
      SmTArray<SmFace*> sEFaces1(64,sFData1);
      pEdge1->GetFaces(sEFaces1);
      
      // get pEdge2 faces
      SmFace *sFData2[64];
      SmTArray<SmFace*> sEFaces2(64,sFData2);
      pEdge2->GetFaces(sEFaces2);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe    = FALSE ;
static ULONG     lCount      = 0 ; lCount++ ;
static ULONG     lDebugCount = 0 ;
  // draw 
  if(bDebugMe || lDebugCount == lCount)
    {
      SM_DUMP_AND_ASSERT_VALID(pEdge1) ;
      SM_DUMP_AND_ASSERT_VALID(pEdge2) ;

      ULONG di ;
      SmTArray<SmEdge *> sFEdges1 ; if(sEFaces1.GetSize() > 0) { sEFaces1[0]->GetEdges(sFEdges1) ; }
      SmTArray<SmEdge *> sFEdges2 ; if(sEFaces2.GetSize() > 0) { sEFaces2[0]->GetEdges(sFEdges2) ; }

      SmFace    * pFace1    = sEFaces1.GetSize() > 0 ? sEFaces1[0] : NULL ;
      SmFace    * pFace2    = sEFaces2.GetSize() > 0 ? sEFaces2[0] : NULL ;
      SmSurface * pSurface1 = pFace1 ? pFace1->GetSurface() : NULL ;
      SmSurface * pSurface2 = pFace2 ? pFace2->GetSurface() : NULL ;
      SmCurve   * pCurve1   = pEdge1 ? pEdge1->GetCurve() : NULL ;
      SmCurve   * pCurve2   = pEdge2 ? pEdge2->GetCurve() : NULL ;
      SmBrep    * pBrep     =   pFace1 ? pFace1->GetBrep() 
                              : pFace2 ? pFace2->GetBrep() 
                              : pEdge1 ? pEdge1->GetBrep()
                              : pEdge2 ? pEdge2->GetBrep() : NULL ; 
      SM_DUMP_AND_ASSERT_VALID(pCurve1) ;
      SM_DUMP_AND_ASSERT_VALID(pCurve2) ;
      SM_DUMP_AND_ASSERT_VALID(pSurface1) ;
      SM_DUMP_AND_ASSERT_VALID(pSurface2) ;
                  
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface1) pSurface1->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pSurface2) pSurface2->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pCurve1) pCurve1->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pCurve2) pCurve2->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; if(pEdge1) pEdge1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 1,0,1) ; if(pEdge2) pEdge2->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,1) ; for(di=0;di<sFEdges1.GetSize();di++)
                                     { sFEdges1[di]->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(2,3,.2,1,0) ; for(di=0;di<sFEdges2.GetSize();di++)
                                     { sFEdges2[di]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // When edge tolerances are smaller than the specified max gap size
      // check distances between edges and otherEdge->faces to see if they can be stitched
      if (dDistBetween > (pEdge1->GetTolerance() + pEdge2->GetTolerance()) / 3.0) 
      {
          double dE1DistToE2Faces = 0.0;
          double dE2DistToE1Faces = 0.0;

          // Find the distance from pEdge1->Curve to each face in sEFaces2.
            // locals
            double dMaxDistToSurface;
            ULONG lNumSamples = 0;  // 0: do a precise measurement
           
            // Find the distance from pEdge1->Curve to each face in sEFaces2.
            for(ii=0; ii<sEFaces2.GetSize(); ii++) 
              {
                SmFace    * pEFace2   = sEFaces2[ii];
                SmSurface * pSurface2 = pEFace2->GetSurface();
           
                //SmBSplineCurve * pCurve1 = SM_CAST_PTR(SmBSplineCurve,pEdge1->GetCurve());
                SmCurve* pCurve1 =pEdge1->GetCurve();
                if ( !pCurve1 ) { break; }
           
                pCurve1->SurfaceMaxDistanceBetween(
                           pEdge1->GetInterval(),  // in : curve domain of interest
                          *pSurface2,              // in :
                           pEFace2->GetUVDomain(), // in : surface domain of interest
                           lNumSamples,            // in : 0 means do a precise measurement
                           NULL,                   // in : optional maximum distance allowed
                           dMaxDistToSurface );    // out: MaxDist(DropPt, 3dCurvePt)
           
                // remember the largest edge->Curve otherEdge->Face->Surface drop distance
                if (dMaxDistToSurface > dE1DistToE2Faces) 
                  { 
                    dE1DistToE2Faces = dMaxDistToSurface;
                  }
              } // end drop pEdge1->Curve to each face in sEFaces2
           
            // Find the distance from pEdge2->Curve to each face in sEFaces1.
            for(ii=0; ii<sEFaces1.GetSize(); ii++) 
              {
                SmFace    * pEFace1   = sEFaces1[ii];
                SmSurface * pSurface1 = pEFace1->GetSurface();
           
                //SmBSplineCurve * pCurve2 = SM_CAST_PTR(SmBSplineCurve,pEdge2->GetCurve());
                SmCurve* pCurve2 = pEdge2->GetCurve();
                if ( !pCurve2 ) { break; }
           
                pCurve2->SurfaceMaxDistanceBetween(
                           pEdge2->GetInterval(),  // in : curve domain of interest
                          *pSurface1,              // in :
                           pEFace1->GetUVDomain(), // in : surface domain of interest
                           lNumSamples,            // in : 0 means do a precise measurement
                           &dE1DistToE2Faces, // in : optional maximum distance allowed
                           dMaxDistToSurface );    // out: MaxDist(DropPt, 3dCurvePt)
           
           
                // remember the largest edge->Curve otherEdge->Face->Surface drop distance
                if (dMaxDistToSurface > dE2DistToE1Faces) 
                  { 
                    dE2DistToE1Faces = dMaxDistToSurface;
                  }
              } // end drop pEdge2->Curve to each face in sEFaces1
            // keep the edge with the smallest maxDropDistance as long as edge dropped to every neighbor face
            if(   dE2DistToE1Faces < dE1DistToE2Faces 
               && dE2DistToE1Faces != SM_BIG_DOUBLE) 
              {
                rdMaxDistance  = dE2DistToE1Faces;
                rbKeepElement1 = FALSE;
                return SM_SUCCESS;
              }
            else if (dE1DistToE2Faces != SM_BIG_DOUBLE) 
              {
                rdMaxDistance = dE1DistToE2Faces;
                return SM_SUCCESS;
              }
      } // end edges have a combined tolerance less than dDistBetween check

      // Otherwise just do the standard simple selection of the best edge
      //  defined as the edge with the most faces or the curve with the fewest knots
      if (sEFaces2.GetSize() > sEFaces1.GetSize()) 
        {
          rbKeepElement1 = FALSE;
          return SM_SUCCESS;
        }
      if (sEFaces1.GetSize() > sEFaces2.GetSize()) 
        {
          return SM_SUCCESS;
        }

      // pEdge1 knots 
      SmCurve * pCurve1 = pEdge1->GetCurve();
      double sDData1[256];
      SmTArray<double> sKnots1(256,sDData1);
      pCurve1->GetKnots(sKnots1);

      // pEdge2 knots
      SmCurve * pCurve2 = pEdge2->GetCurve();
      double sDData2[256];
      SmTArray<double> sKnots2(256,sDData2);
      pCurve2->GetKnots(sKnots2);

      // when knot counts vary - keep the smallest count 
      if(sKnots2.GetSize() != sKnots1.GetSize()) 
        {
          rbKeepElement1 = sKnots1.GetSize() < sKnots2.GetSize() ? TRUE : FALSE ;
        }

      // all done
      return SM_SUCCESS;

    } // end 1st element is an edge check

  // all done
  return SM_SUCCESS;

} // end SmStitchCallback::SelectTopologyToKeep

/***********************************************************************
Test to see whether a parameter value is in the interior of an Edge, to tolerance.
If trouble (return SM_ERR), returns bInterior == FALSE.
***********************************************************************/
static SmStatus sm_CheckInteriorEdge(
        const SmEdge     * pEdge,
              double       dT,
              double       dTol,
              SmBoolean  & bInterior
    )
{
  bInterior = FALSE;

  // Check common simple case first.
  // Note, negative tol excludes the ends.
  SmExtent1d sEdgeDomain = pEdge->GetInterval();
  if ( ! sEdgeDomain.ContainsValue( dT, -SM_EFF_ZERO ) )
    { return SM_SUCCESS; }

  SmPoint3d sCrvPt[2];
  SmCurve *pCurve = pEdge->GetCurve();
  SER( pCurve->Evaluate( dT, 1, FALSE, sCrvPt ) );
  double dParamLen = sCrvPt[1].Length();
  if ( dParamLen > 0.1 )
  {
      double dParamTol = dTol / dParamLen;

      // Because the param length can vary, use this test
      // only as a rough guide: if it's anywhere close,
      // go on to the 3d test.  So use a bigger tolerance.
      dParamTol *= 20.0;
      if ( sEdgeDomain.ContainsValue( dT, -dParamTol ) )
      {
          bInterior = TRUE;
          return SM_SUCCESS;
      }
  }
  // Work in 3d.
  // This is also more reliable in case of vertex/edge tolerance.

  SmVertex *pV = pEdge->GetVertex();
  double dDist = sCrvPt[0].DistanceBetween( pV->GetPoint() );
  if ( dDist <= dTol )
    { return SM_SUCCESS; }

  pV = pEdge->GetOtherVertex( pV );
  dDist = sCrvPt[0].DistanceBetween( pV->GetPoint() );
  if ( dDist <= dTol )
    { return SM_SUCCESS; }

  bInterior = TRUE;
  return SM_SUCCESS;

} // end sm_CheckInteriorEdge


// A local enum, for sm_TestAndSplitEdge (static is NOT allowed).
enum sReasonType
{
  NO_REASON,
  FAR_FROM_GEOM,
  CLOSE_TO_BOUNDARY
};

/*******************************************************************//**
PURPOSE: Static helper: Test to see if an edge can be split by this point
    
NOTES: 
***********************************************************************/
static SmStatus sm_TestAndSplitEdge(SmEdge      * pEdge,
                              const SmPoint3d   & crPoint,      // in
                                    double        dTol,         // in: Vertex tol + Stitch tol
                                    sReasonType & reReason,     // out
                                    double      & rdDistToEdge, // out: always set, whether split or not
                                    SmVertex   *& rpNewVertex,  // out
                                    SmEdge     *& rpNewEdge)    // out
{
    // Init outputs
    rdDistToEdge = SM_BIG_DOUBLE;
    rpNewVertex  = NULL;
    rpNewEdge    = NULL;
    reReason     = NO_REASON;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_Erase();
        smgfx_SetLook( 2,4, 1,0,0); crPoint.Draw(); sm_GraphicsLoop();
        smgfx_SetLook( 2,3, 0,1,1); pEdge ->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif
    SmCurve *pCurve = pEdge->GetCurve(); NER(pCurve);
    SmBoolean bSuccess;
    SmExtent1d sEdgeDomain = pEdge->GetInterval();
    double dParam, dThisTol = dTol;  // This can't be too big.  [B448]
    SER(pCurve->DropPoint(sEdgeDomain,      // in : target curve allowed domain
                          crPoint,          // in : Point to drop to curve
                          NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                          dThisTol,         // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                          NULL,             // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                          bSuccess,         // out: TRUE = found a drop point
                          dParam,           // out: found drop curve param
                          rdDistToEdge)) ;  // out: found drop distance
                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
    // Check far from curve.
    dThisTol = smos_Max( dTol, (double)pEdge->GetTolerance() ); //cbi  dTol + pEdge->GetTolerance();
    if ( !bSuccess || rdDistToEdge > dThisTol )
    {
        reReason = FAR_FROM_GEOM;
        return SM_SUCCESS;
    }

    // Check on the interior.
    SmBoolean bInterior = FALSE;
    sm_CheckInteriorEdge( pEdge, dParam, dThisTol, bInterior );
    if ( bInterior == FALSE )
    {
        reReason = CLOSE_TO_BOUNDARY;
        return SM_SUCCESS;
    }

    // Must have a point which can split the Edge.
    SmEdge *pE1 = NULL, *pE2 = NULL;
    SmVertex *pNewV = NULL;
    if (pEdge->GetBrep()->MakeVertexSplitEdge(pEdge,dParam,pE1,pE2,pNewV) != SM_SUCCESS) {
        SM_ASSERT( FALSE );
        reReason = CLOSE_TO_BOUNDARY; // most probable reason; don't really care too much.
        return SM_SUCCESS;
    }
    NER(pNewV);
    rpNewVertex = pNewV;
    rpNewEdge = pE2;
    if (pE2 == pEdge) {
        rpNewEdge = pE1;
    }
    return SM_SUCCESS;

} // end sm_TestAndSplitEdge

/*******************************************************************//**
PURPOSE: Stitch face pairs of a Brep
    
NOTES: 
  This routine is never called, and its utility is questionable.
  The only scenario in which it will work is if cpFacesToDelete
  are manifold faces in pBrepToStitch, and cpFacesToKeep are lamina
  versions of cpFacesToDelete, which are geometrically equivalent
  at the boundaries.  See comments after the RemoveFaces() call.
  Following are the original comments.

    This is designed to stitch two topologically distinct 
    but, at the boundaries, geometrically adjacent faces from a brep  
    by gluing the adjacent vertices and edges. For each face pair, 
    one face is kept and the other deleted.
    Note: there is no checking for and squeezing of micro edges

    increments unlocked mark value
***********************************************************************/
SmStatus SmStitch::DoSimpleFaceStitch
 (SmBrep                  * pBrepToStitch,
  const SmTArray<SmFace*> & cpFacesToKeep,
  const SmTArray<SmFace*> & cpFacesToDelete,
  double                  & rdMaxVertexGap,
  double                  & rdMaxEdgeGap,
  ULONG                   & rlStitchedEdges)
{   
    rdMaxVertexGap = 0.0;
    rdMaxEdgeGap = 0.0;
    rlStitchedEdges = 0;

    ULONG nFPairs = cpFacesToKeep.GetSize();
    if (nFPairs == 0 || nFPairs != cpFacesToDelete.GetSize()) 
        return (SM_ERR);

    // The given Faces must be part of the given Brep.
    SmBrep *pBrep0 = (cpFacesToKeep[0]->GetBrep());
    if (!pBrep0 || pBrep0 != pBrepToStitch) 
        return (SM_ERR);
    SmBrep *pBrep1 = (cpFacesToDelete[0]->GetBrep());
    if (!pBrep1 || pBrep1 != pBrepToStitch)
        return (SM_ERR);
    
    SmTArray <SmVertex *> sKVertices;
    SmTArray <SmVertex *> sDVertices;
    SmTArray <SmEdge *> sKEdges;
    SmTArray <SmEdge *> sDEdges;
    ULONG nVerts, nEdges;
    for(ULONG iPair=0; iPair<nFPairs; iPair++){
        // need the vertices and edges for this pair of faces
        SmFace *pFaceToKeep = cpFacesToKeep[iPair];
        SmFace *pFaceToDelete = cpFacesToDelete[iPair];

        // Need to find the pairs of vertices that are to be glued together
        SmTArray <SmVertex *> sKVerts;  // vertices of this FaceToKeep
        pFaceToKeep->GetVertices(sKVerts);

        SmTArray < SmVertex *> sDVerts;  // vertices of this FaceToDelete
        pFaceToDelete->GetVertices(sDVerts);
    
        // must have the same number of vertices for each face
        nVerts = sKVerts.GetSize();
        if (nVerts != sDVerts.GetSize())
            return (SM_ERR); // faces not compatible

        double Vtol = m_dStitchTol3d; // may want a Vtol and an Etol
        ULONG nFound = 0;
        ULONG iK;
        for(iK = 0; iK < nVerts; iK++)
        {
            SmVertex *pKVertex = sKVerts[iK];
            SmPoint3d pKVpt = pKVertex->GetPoint();
            for(ULONG iD = 0; iD < nVerts; iD++)
            {
                SmVertex *pDVertex = sDVerts[iD];
                SmPoint3d pDVpt = pDVertex->GetPoint();
                double Vdist = pKVpt.DistanceBetween(pDVpt);
                if (Vdist < Vtol)
                {
                    // found the neighbor of this vertex
                    if (rdMaxVertexGap < Vdist)
                        rdMaxVertexGap = Vdist;
                    sKVertices.Add(pKVertex);
                    sDVertices.Add(pDVertex);
                    nFound++;
                    iD = nVerts; // don't need to test further
                }
            }
        }

        // must have nVert pairs
        if (nFound != nVerts)
            return (SM_ERR); // vertices not all in tolerance
        // end of Vertex tests
       
        // start of Edge tests
        SmTArray <SmEdge *> sKEds;  // edges of this FaceToKeep
        pFaceToKeep->GetEdges(sKEds);

        SmTArray <SmEdge *> sDEds;  // edges of this FaceToDelete
        pFaceToDelete->GetEdges(sDEds);
    
        // must have the same number of edges for each face
        // and the same number of edges as verts
        nEdges = sKEds.GetSize();
        if (nEdges != sDEds.GetSize() || nEdges != nVerts) 
            return (SM_ERR); 
    
        //double Etol = m_dStitchTol3d; // may want a Vtol and an Etol
    
        nFound = 0;
        for(iK = 0; iK < nEdges; iK++)
        {
            SmEdge *pKEdge = sKEds[iK];
            SmVertex *pKV0 = pKEdge->GetVertex();
            SmVertex *pKV1 = pKEdge->GetOtherVertex(pKV0);

            // determine the corresponding DVerts
            ULONG lIndex;
            if (!sKVertices.FindElement(pKV0, lIndex))return (SM_ERR);
            SmVertex *pDKV0 = sDVertices[lIndex];

            if (!sKVertices.FindElement(pKV1, lIndex)) return (SM_ERR);               
            SmVertex *pDKV1 = sDVertices[lIndex];

            SmCurve *pKCurve  = pKEdge->GetCurve();

            for(ULONG iD = 0; iD < nEdges; iD++)
            {
                SmEdge *pDEdge = sDEds[iD];
                SmCurve *pDCurve  = pDEdge->GetCurve();
                SmVertex *pDV0 = pDEdge->GetVertex();
                SmVertex *pDV1 = pDEdge->GetOtherVertex(pDV0);
                // need the edge with verts DKV0 and DKV1
                if ((pDKV0 == pDV0 && pDKV1 == pDV1) ||
                    (pDKV0 == pDV1 && pDKV1 == pDV0))
                { // found an edge with matching vertices
                    nFound++;
                    // compare this pair of edges
                    double dKStart, dKEnd, dDStart, dDEnd;
                    double dDistToCurve;
                    dKStart = pKEdge->GetInterval().GetMin();
                    dKEnd =   pKEdge->GetInterval().GetMax();
                    dDStart = pDEdge->GetInterval().GetMin();
                    dDEnd =   pDEdge->GetInterval().GetMax();
                    SmPoint3d sKpt0, sKpt1, sDpt0, sDpt1;
                    SER(pKCurve->EvaluatePoint(dKStart, sKpt0));
                    SER(pKCurve->EvaluatePoint(dKEnd,   sKpt1));
                    SER(pDCurve->EvaluatePoint(dDStart, sDpt0));
                    SER(pDCurve->EvaluatePoint(dDEnd,   sDpt1));
                    if (sKpt0.DistanceBetween(sDpt0) < Vtol &&
                        sKpt1.DistanceBetween(sDpt1) < Vtol)
                    {
                        // do nothing, continue on to check dist.   continue;
                    }
                    else if (sKpt0.DistanceBetween(sDpt1) < Vtol &&
                            sKpt1.DistanceBetween(sDpt0) < Vtol)
                    {
                        double dTemp = dDStart;
                        dDStart = dDEnd;
                        dDEnd = dTemp;
                    }
                    else
                      { return (SM_ERR); }

                    double dTol = smos_Max( m_dStitchTol3d,
                                            (double)( pKEdge->GetTolerance() + pDEdge->GetTolerance() ));
                   
                    SE(pKCurve->CurveMaxDistanceBetween(pKEdge->GetInterval(), 
                                    *pDCurve, dDStart, dDEnd, 10, &dTol, dDistToCurve));
                    if (dDistToCurve > dTol)
                      { return (SM_ERR); } // edges not in tol
                    if (rdMaxEdgeGap < dDistToCurve)
                      { rdMaxEdgeGap = dDistToCurve; }
                    iD = nEdges; // don't need to test any further
                }
            }
        }
        // must have nEdge pairs
        if (nFound != nEdges)  
          { return (SM_ERR); }
    
    } // end of this face pair

    // the vertex and edge pairs have been found and they 
    // are in tolerance so delete FacesToDelete. 

    pBrepToStitch->RemoveFaces(cpFacesToDelete);  // increments unlocked mark value

    // Note: this routine is never called, and the following is
    // presumably incorrect.  If the Edges and Vertices connected
    // to cpFacesToDelete were Lamina, then the RemoveFaces() call
    // will delete them, and trying to stitch back in deleted entities
    // will obviously cause problems.  But if they were not Lamina,
    // then they must have been Manifold, connected to another Face.
    // If that other Face is the face to keep, then the Edges and
    // Vertices would be the same entities, so stitching makes no sense.
    // The only remaining scenario is if the Delete Faces were part
    // of a Manifold solid, and the Keep Faces are laminar copies
    // of the Delete Faces.  In that case, this routine is simply
    // replacing the Delete Faces with the Keep Faces.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
    if (bDebugMe3) {
        pBrepToStitch->Dump();
        pBrepToStitch->Draw();
    }
#endif

    // now follow the procedure that is used in DoStitching
    // for stitching vertex and edge pairs

    // increment and lock an unlocked mark - gwc: don't see where marks are used in this method - could delete this section
    SmNewMarkAndLock sMarkLock( pBrepToStitch->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
    SmMarkType eMarkType = sMarkLock.GetMarkType() ;

    nVerts = sKVertices.GetSize();
    ULONG j;
    for(j=0; j<nVerts; j++) {
        SmVertex *pV = sKVertices[j];
        pV->Mark(eMarkType);
        pV = sDVertices[j];
        pV->Mark(eMarkType);
    }
    nEdges = sKEdges.GetSize();
    for(j=0; j<nEdges; j++) {
        SmEdge *pE = sKEdges[j];
        pE->Mark(eMarkType);
        pE = sDEdges[j];
        pE->Mark(eMarkType);
    }

    SmTemporaryChangeValue<SmBoolean> sChange(pBrepToStitch->m_bEditingEnabled,TRUE);
    pBrepToStitch->Notify(SM_NO_PRE_EDIT, pBrepToStitch, NULL, NULL);

    // glue the coincident vertices

    SM_PTR_ARRAY(sCommonEdges, SmEdge, 64); // SmTArray<SmEdge *?
    SM_PTR_ARRAY(sWireEdges,   SmEdge, 64); // SmTArray<SmEdge *?
    pBrepToStitch->GetWireEdges(sWireEdges);

    for(ULONG kV=0; kV<nVerts; kV++) {
        SmBoolean bKeepFirst;
        SmVertex *pV0 = sKVertices[kV];
        SmVertex *pV1 = sDVertices[kV];
        if (  ! m_bMakingManifoldSolid
            ||
              ( pV0->IsLaminaVertex() && pV1->IsLaminaVertex() )
           )
          {
            double dDistToV = pV0->GetPoint().DistanceBetween(pV1->GetPoint());
            double dMaxDistance;
            SER(m_rStitchCallback.SelectTopologyToKeep(pV0,pV1,dDistToV,dMaxDistance,bKeepFirst));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe5 = FALSE;
            if (bDebugMe5) {
                smgfx_Erase();
                smgfx_SetColor(1,0,0);
                pV0->Draw();
                sm_GraphicsLoop();
                smgfx_SetColor(0,0,1);
                pV1->Draw();
                sm_GraphicsLoop();
                pBrepToStitch->Draw(); 
            }
#endif

            double dTol = smos_Max( m_dStitchTol3d,
                                    (double)( pV0->GetTolerance() + pV1->GetTolerance() ));
            if (dMaxDistance > dTol ) {
                SER(SM_ERR); // Don't stitch it if distance gets too big.
            }
            ULONG bDoGlue = TRUE;
            ULONG bDoSqueeze = FALSE;
            SmEdge *pSqueezeEdge = NULL;
            pV0->GetCommonEdges(pV1,sCommonEdges);
            for(ULONG lll=0; lll<sCommonEdges.GetSize(); lll++) {
                SmEdge *pE = sCommonEdges[lll];
                double dEdgeLength = pE->GetCurve()->ApproximateLength(pE->GetInterval(),10);
                if (dEdgeLength < 2.0 * dDistToV) {
                    if (bDoSqueeze) {
                        // Too many edges between vertices
                        bDoSqueeze = FALSE;
                        bDoGlue = FALSE;
                        break;
                    }
                    if (pE->IsLamina() || pE->IsWire()) {
                        bDoSqueeze = TRUE;
                        bDoGlue = FALSE;
                        pSqueezeEdge = pE;
                    }
                    else {
                        bDoSqueeze = FALSE;
                        bDoGlue = FALSE;
                    }
                }
            }

            if (   bDoSqueeze 
       && m_bSqueezeSmallEdges == FALSE) {
                continue;
            }
            if (bKeepFirst) {
                if (bDoSqueeze) {
                    SER(pBrepToStitch->SqueezeEdge(pSqueezeEdge,pV0));
                }
                else if (bDoGlue) {
                    SER(pBrepToStitch->GlueVertices(pV0,pV1,&sWireEdges));
                }
            }
            else {
                if (bDoSqueeze) {
                    SER(pBrepToStitch->SqueezeEdge(pSqueezeEdge,pV1));
                }
                else if (bDoGlue) {
                    SER(pBrepToStitch->GlueVertices(pV1,pV0,&sWireEdges));
                }
                pV0 = pV1;
            }
#ifdef SM_VALIDATE_TOPOLOGY
            pBrepToStitch->ValidatePointers();
#endif
        }
    } // for kV
 
    // All the vertices are stitched so now stitch the edge pairs
    SmTArray<SmFace*> sEdgeFaces, sModifiedFaces;

    for(ULONG kE = 0 ; kE<nEdges ; kE++){
        SmEdge *pE0 = sKEdges[kE];
        SmEdge *pE1 = sDEdges[kE]; // Both edges have same vertices
        SmCurve *pE0Curve = pE0->GetCurve();
        SmCurve *pE1Curve = pE1->GetCurve();
        // Need to figure out dTStart and dTEnd and relative orientation of curves.
        SmPoint3d sMidPnt;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
//            smgfx_Erase();
            sm_GraphicsLoop();
            smgfx_SetLineWidth(3.0);
            smgfx_SetColor(0,0,1);
            pE1->Draw();
            smgfx_SetColor(1,0,0);
            smgfx_SetLineWidth(5.0);
            pE0->Draw();
        }
#endif
        double dTol = smos_Max( m_dStitchTol3d, (pE0->GetTolerance() + pE1->GetTolerance()) );
        SER(pE0Curve->EvaluatePoint(pE0->GetInterval().Evaluate(0.5),sMidPnt));
        SmBoolean bSuccess;
        double dDropParam, dDistToCurve;
        SER(pE1Curve->DropPoint(pE1->GetInterval(), // in : target curve allowed domain
                                sMidPnt,            // in : Point to drop to curve
                                NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                    //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                    //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                dTol,               // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                    //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                    //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                    //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                NULL,               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                bSuccess,           // out: TRUE = found a drop point
                                dDropParam,         // out: found drop curve param
                                dDistToCurve)) ;    // out: found drop distance
                                                    // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                    //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                    //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                    //      default:[SM_SO_MINIMIZE] to preserve original behavior
        if (!bSuccess) SER(SM_ERR);
        SmVector3d sPV[2], sPVTest[2];
        SER(pE0Curve->Evaluate(pE0->GetInterval().Evaluate(0.5),1,TRUE,sPV));
        SER(pE1Curve->Evaluate(dDropParam,1,TRUE,sPVTest));
        SmOrientType eRelativeOrientation = SM_OT_SAME;
        double dTStart = pE1->GetInterval().GetMin();
        double dTEnd = pE1->GetInterval().GetMax();
        if (sPV[1].Dot(sPVTest[1]) < 0.0) {
            eRelativeOrientation = SM_OT_OPPOSITE;
            double dTTmp = dTStart;
            dTStart = dTEnd;
            dTEnd = dTTmp;
        }
        SER(pE0Curve->CurveMaxDistanceBetween(pE0->GetInterval(),*pE1Curve,
                            dTStart,dTEnd,5,&dTol,dDistToCurve));
        if (dDistToCurve > dTol) SER(SM_ERR);
        if (!m_bFastEdgeCompare) {
            // Passed an initial quick test with 5 points now do a precise measurement
            SER(pE0Curve->CurveMaxDistanceBetween(pE0->GetInterval(),*pE1Curve,
                                dTStart,dTEnd,0,&dTol,dDistToCurve));
            if (dDistToCurve > dTol) SER(SM_ERR);
        }
        // made it here so glue edges
        if (dDistToCurve > rdMaxEdgeGap) rdMaxEdgeGap = dDistToCurve;
                        
        SmBoolean bKeepFirst;
        double dMaxDistance;
        SER(m_rStitchCallback.SelectTopologyToKeep(pE0,pE1,dDistToCurve,dMaxDistance,bKeepFirst));

        if (dMaxDistance > dTol) {
            SER(SM_ERR); // can't stitch, error
        }
        if (bKeepFirst) {
            pE1->GetFaces(sEdgeFaces);
            for(ULONG mm=0; mm<sEdgeFaces.GetSize(); mm++) {
                sModifiedFaces.AddUnique(sEdgeFaces[mm]);
            }
            SER(pBrepToStitch->GlueEdgesGeneral(pE0,eRelativeOrientation,dDistToCurve,
                                m_bDoRegionNesting,pE1));
        }
        else {
            pE0->GetFaces(sEdgeFaces);
            for(ULONG mm=0; mm<sEdgeFaces.GetSize(); mm++) {
                sModifiedFaces.AddUnique(sEdgeFaces[mm]);
            }
            SER(pBrepToStitch->GlueEdgesGeneral(pE1,eRelativeOrientation,dDistToCurve,
                                m_bDoRegionNesting,pE0));
        }
        rlStitchedEdges ++;
    } // for kE

    
#ifdef SM_VALIDATE_TOPOLOGY
    if (m_bValidateResult)
    {
        SER(pBrepToStitch->ValidatePointers());
    }
#endif

    return SM_SUCCESS;

} // end DoSimpleFaceStitch

/*******************************************************************//**
PURPOSE: Stitch the edges of a set of topologically disjoint but 
    geometrically adjacent faces together.  This converts a Brep from
    a bag of trimmed surfaces into an open shell or possibly a solid.
    This method is very sensitive to the input data quality.  Currently
    the edges of the trimmed surfaces need to have a one-to-one 
    correspondance.  It works best for trimmed surfaces which originate
    from a solid or constructed specifically for the purpose of creating
    a solid.

NOTES:
    1. It is possible to invoke this method multiple times with the
       same Brep increasing the tolerance until all edges are stitched.
       This technique causes closer topology pairs to be stitched preserving
       smaller tolerances than looser pairs.
       The tolerance specifying the max 3d distance between coincident
       geometry is stored in m_dStitchTol3d.

    2. The stitching behavior depends on the boolean state stored in this 
       SmStitch Object. The significant behavior switches include:

       m_bSqueezeSmallEdges      == TRUE remove short edges from the topology graph
       m_bIgnoreProblems         == TRUE continue stitching after a glue edge pair attempt failed.
       m_bFastEdgeCompare        == TRUE  test edge coincidence with fast 5 point check
                                    FALSE test edge coincidence with precise algorithm 
       m_bSplitEdgesWithVertices == TRUE split all edges that lie within tolerance of a vertex
       m_bMakingManifoldSolid    == TRUE do not create spine edges:
                                         only squeeze lamina edges or glue lamina edge pairs
                                         possibly leaving coincident manifold edges in the model
                                    FALSE squeeze or glue all edges leaving no coincident edges
                                          in the model but possibly building a nonManifold model.
                                          NOTE: m_bMakingManifoldSolid == TRUE does not tell the function
                                                to return a manifold model.
SEQUENCE:
 I. Preprocess
  1. Remove laminar slivers: SmBrep::DeleteFace()
  2. Remove OpenEdges shorter than m_dStitchTol3d: SmBrep::SqueezeEdge()
II. Stitch coincident geometry
  Loop until no topology changes:
    1. when asked, split edges at coincident vertices.  This can create coincident vertices
       that will be processed in the next step.
    2. stitch Coincident vertices after sorting vertices on a common projection line for performance
    3. Find and stitch coincident edges: after stitching vertices, all coincident edges share common vertices
  
About Tolerances:
Tolerance handling in this file has been unified.  The various tolerances
that were used for various things are documented in older versions of this
file (7/11/14).  The tolerance now used is
  max( sum of topologies's tols, m_dStitchTol3d ).
In this file, every use of a tolerance involves two topological entities,
and the tolerance used is the usual sum of their tolerances,
unless overridden by the user-prescribed m_dStitchTol3d.

  increments unlocked mark value
***********************************************************************/
SmStatus SmStitch::DoStitching
  (SmBrep                    * pBrepToStitch,           // in : target Brep
   const SmTArray<SmVertex*> * cpOptCandidateVertices,  // in : only glue coincident vertices on this list, 
                                                        //      NULL = do all vertices
   const SmTArray<SmEdge*>   * cpOptCandidateEdges,     // in : only glue coincident edge pairs on this list, 
                                                        //      NULL = do all edges
   ULONG                     & rlStitchedEdges,         // out: number of edges stitched
   ULONG                     & rlLaminaEdges,           // out: number of lamina edges remaining after stitch
   double                    & rdMaxVertexGap,          // out: max gap found between coincident vertices considered for gluing
   double                    & rdMaxEdgeGap,            // out: max gap found between coincident edges    considered for gluing
                                                        //      NOTE: some coincident vertex and edge pairs do not get glued due to
                                                        //            a. gaps exceeding tolerances
                                                        //            b. geometries not listed within optional candidate lists
                                                        //            c. a failure within the glue edge function
                                                        //            d. not being lamina when m_bMakingManifoldSolid == TRUE
   double                    * pdMinUnstitchedVertGap,  // out: min gap found between vertices that did not get glued
                                                        //      default:[NULL], NULL to ignore.
   double                    * pdMinUnstitchedEdgeGap ) // out: min gap found between edges that did not get glued
                                                        //      default:[NULL], NULL to ignore.
{
  // init output
  rdMaxVertexGap = 0.0;
  rdMaxEdgeGap   = 0.0;
  rlLaminaEdges  = 0;
  rlStitchedEdges    = 0;
  if ( pdMinUnstitchedVertGap != NULL ) { *pdMinUnstitchedVertGap = SM_BIG_DOUBLE; }
  if ( pdMinUnstitchedEdgeGap   != NULL ) { *pdMinUnstitchedEdgeGap   = SM_BIG_DOUBLE; }

  // locals
  ULONG ii, jj; 
   
  // display input for debugging
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe)  // draw faces, face->edges, face->vertices in various orders to allow
    {            //  an inspection of what may lie on top of what
      SM_DUMP_AND_ASSERT_VALID(pBrepToStitch) ;

      SmTArray<SmFace*>   sFaces ;     pBrepToStitch->GetFaces   (sFaces) ;
      SmTArray<SmEdge*>   sEdges ;     pBrepToStitch->GetEdges   (sEdges) ;
      SmTArray<SmVertex*> sVertices ;  pBrepToStitch->GetVertices(sVertices) ;
      SmTArray<SmEdge*>   sFaceEdges ;
      SmTArray<SmVertex*> sFaceVertices ;
      
      smgfx_Erase();
      smgfx_SetLook(1,3) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(ii=0;ii<sFaces.GetSize();ii++) 
        {
          smgfx_SetLook(1,2, 0,1,1) ; sFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; 
          sFaces[ii]->GetEdges(sFaceEdges) ;  sFaceEdges.Dump() ;
          sFaces[ii]->GetVertices(sFaceVertices) ; sFaceVertices.Dump() ;
          for(jj=0;jj<sFaceEdges.GetSize();jj++)
            { smgfx_SetLook(2,3, 1,0,0) ; sFaceEdges[jj]->Draw() ; sm_GraphicsLoop() ; }
          for(jj=0;jj<sFaceVertices.GetSize();jj++)
            { smgfx_SetLook(2,6, 1,0,1) ; sFaceVertices[jj]->Draw() ; sm_GraphicsLoop() ; }
        }
      smgfx_SetLook(3,4, 0,1,0) ; for(ii=0;ii<sEdges.GetSize();ii++)
                                    { sEdges[ii]->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1,0,0) ; for(ii=0;ii<sVertices.GetSize();ii++)
                                    { sVertices[ii]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

   
  // set Brep to allow modifications
  SmTemporaryChangeValue< SmBoolean > sChange(pBrepToStitch->m_bEditingEnabled,TRUE);
  const SmContext *pContext = pBrepToStitch->GetContext();
  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, pContext)->GetDoingBooleanRef(), TRUE );
  pBrepToStitch->Notify(SM_NO_PRE_EDIT, pBrepToStitch, NULL, NULL);
 
  // when asked find and remove sliver laminar faces
  if(m_bRemoveLaminarSlivers)
    {
      SmZoneTol3d sZoneTol3d(m_dStitchTol3d / 2.);
      SmTArray<SmFace*> sFaces ;
      pBrepToStitch->GetFaces(sFaces) ;

      for(ii=0;ii<sFaces.GetSize();ii++)
        {
          SmFace *pFace = sFaces[ii] ;

          // when face is a lamina sliver
          if(   pFace->IsLamina()
             && pFace->IsDegenerate(TRUE, &sZoneTol3d))
           {
             // delete Face and its boundary edges and vertices from the Brep
             pBrepToStitch->DeleteFace(pFace, TRUE) ;
           }
        } // end iter every face
    } // end remove laminar sliver face check

  // increment and lock an unlocked mark - gwc: don't see where marks are used - could remove this section
  //  or we could make sCandidateVertices.FindElement(pSurvV,lIndex) run faster.

  /*
  SmNewMarkAndLock sMarkLock( pBrepToStitch->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;
  
  // mark every candidate vertex
  if (cpOptCandidateVertices)
    {
      for(jj=0; jj<cpOptCandidateVertices->GetSize(); jj++) 
        {
          SmVertex *pV = (*cpOptCandidateVertices)[jj];
          pV->Mark(eMarkType);
        }
    }

  // mark every candidate edge
  if (cpOptCandidateEdges)
    {
      for(jj=0; jj<cpOptCandidateEdges->GetSize(); jj++) 
        {
          SmEdge *pE = (*cpOptCandidateEdges)[jj];
          pE->Mark(eMarkType);
        }
    }
  */
  // locals
  SmTArray<SmEdge*>   sCommonEdges;
  SmTArray<SmEdge*>   sCandidateEdges;
  SmTArray<SmVertex*> sCandidateVertices;
  SmTArray<SmVertex*> sVertices;
  SmTArray<SmVertex*> sCoinVertices;
  SmTArray<SmEdge*>   sVEdges;
  SmTArray<SmFace*>   sEdgeFaces;
  SmTArray<SmFace*>   sModifiedFaces;
  SmTArray<SmVertex*> sSortedVerts;
  SmTArray<SmBoolean> sIsLaminaVertex;
  SmTArray<double>    sSortParameter;
  SmBoolean           bHasWireVertex = FALSE;

  // init candidate edges and vertices with the optional input arrays
  if (cpOptCandidateEdges) 
    {
      sCandidateEdges.Append(*cpOptCandidateEdges);
    }
  if (cpOptCandidateVertices) 
    {
      sCandidateVertices.Append(*cpOptCandidateVertices);
    }

  // holder for min length edge seen in BrepToStitch
  double dMinEdgeLength = SM_BIG_DOUBLE;

  // when asked - squeeze out edges shorter than m_dStitchTol3d
  if (m_bSqueezeSmallEdges) 
    {
      // iter every edge
      SmTArray<SmEdge*>   sEdges;
      pBrepToStitch->GetEdges(sEdges);
      for(ii=0; ii<sEdges.GetSize(); ii++) 
        {
          SmEdge *pE = sEdges[ii];

          // only check lamina edges when making a ManifoldSolid 
          if ( m_bMakingManifoldSolid && !pE->IsLamina() )
            { continue; }

          // get edge->curve approximate length
          SmCurve *pCurve  = pE->GetCurve();
          double   dLength = pCurve->ApproximateLength(pE->GetInterval(),5);

          // when edge is shorter than stitching tolerance and open
          if (   dLength < m_dStitchTol3d 
              && !pE->IsClosed()) 
            {
              // add every face connected to this edge to the sModifiedFaces list 
              pE->GetFaces(sEdgeFaces);
              for(jj=0; jj<sEdgeFaces.GetSize(); jj++) 
                {
                  sModifiedFaces.AddUnique(sEdgeFaces[jj]);
                }

              // get edge's vertices
              SmVertex *pSurvV = pE->GetVertex();
              SmVertex *pDelV  = pE->GetOtherVertex(pSurvV);

              // when given optCandidateVertices
              if (cpOptCandidateVertices) 
                {
                  // skip vertices if neither is on the optCandidateVertex list
                  ULONG lIndex;
                  if (   !sCandidateVertices.FindElement(pSurvV,lIndex)
                      && !sCandidateVertices.FindElement(pDelV, lIndex) )
                    { continue; }
                }

#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  SM_DUMP_AND_ASSERT_VALID(pBrepToStitch) ;

                  SmVertex *pVertex = pE->GetStartVertex() ;

                  // just for fun look for sliver faces
                  SmTArray<SmFace*> sFaces ;
                  pE->GetFaces(sFaces) ;
                  for(jj=0;jj<sFaces.GetSize();jj++)
                    {
                      SmFace *pFace = sFaces[jj] ;
                      // SmBoolean bDegenerate = pFace->IsDegenerate() ;

                      smgfx_Erase();
                      smgfx_SetLook(1,2) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,4, 1,0,0) ; if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,4, 1,0,0) ; if(pE) pE->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,6, 1,0,1) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop() ;
                    }
                }

#endif // SM_DEBUG_CODE
                  // remove and delete edge from topology graph by squeezing [vertex-edge-vertex] into [vertex] 
                  SER(pBrepToStitch->SqueezeEdge(pE,pSurvV));

                  if ( cpOptCandidateVertices )
                    { sCandidateVertices.AddUnique( pSurvV ); } // This becomes a candidate.

#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  SM_DUMP_AND_ASSERT_VALID(pBrepToStitch) ;
                  
                  SmVertex *pVertex = pSurvV ;

                  smgfx_Erase();
                  smgfx_SetLook(1,2) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,4, 1,0,0) ; if(pVertex) pVertex->Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                   
                }

#endif // SM_DEBUG_CODE

            } // end short and open edge branch
          else // edge was long or closed
            {
              // if the edge was short - tighten the dMinEdgeLength
              if (dLength < dMinEdgeLength) 
                {
                  dMinEdgeLength = dLength;
                }
            } // end not a short and open edge branch 

        } // end iter every edge
    } // end m_bSqueezeSmallEdges == TRUE check

  // arrive here after
  // 1. Remove laminar slivers: SmBrep::DeleteFace()
  // 2. if given, Mark:[eMarkType] every candidate vertex and edge and init sCandidateVertices and sCandidateEdges lists
  // 2. Remove OpenEdges shorter than m_dStitchTol3d: SmBrep::SqueezeEdge()

  // locals
  SmVertex* sCoinVData[16]; 
  SmTArray<SmVertex*> sCoinV(16, (SmVertex**)sCoinVData) ;

  // for 10 iterations or until all work is done
  // I. If asked, find Vertex/Edge intersections that split edges.
  //    This can create coincident-vertex pairs that will be processed in step II.
  //
  // II. Stitch coincident vertex sets
  //   1. Sort lamina vertices by their projection onto a common line: sVertices, sSortParameter, sSortedIndex
  //   2. For every vertex, 
  //        place coincident vertex set into sCoinVertices list
  //          2a. When given cpOptCandidateVertices, remove sCoinVertices members not on the list
  //          2b. when m_bMakingManifoldSolid == TRUE  - only glue pairs of Lamina vertices   
  //                   m_bMakingManifoldSolid == FALSE - glue all pairs of coincident vertices
  //             2b1. pick vertex to save with SelectTopologyToKeep()
  //             2b2. glue   : combine two coincident vertices not connected with a common edge
  //                           SmBrep::GlueVertices()
  //                  squeeze: combine two coincident vertices and delete edge when            
  //                               vertices are connected by a common wire or lamina short edge. 
  //                           SmBrep::SqueezeEdge() 
  //             2b3. place surviving vertex back on sCandidateVertices  
  //
  // III. Stitch coincident edge sets - because vertices are stitched all stitchable edges will share common vertices
  //   1. For every vertex, get all edges connected to iter vertex[ii]
  //      1a. For every vertex[ii]->edge[jj] vertex[ii]->edge[kk] pair
  //          1a1. when m_bMakingManifoldSolid == TRUE skip nonLamina edges
  //          1a2. skip edges pairs that don't share common end vertices
  //          1a3. when given cpOptCandidateEdges, skip edges not on the list
  //          1a4. skip edge pairs further apart than dTol = m_dStitchTol3d + edge[jj]->GetTolerance() + edge[kk]->GetTolerance()
  //              - quick check mid point drop
  //              - quick check nonLine curves at 5 sample drop points
  //              - when asked, do precise distance check with a call to SmCurve::CurveMaxDistanceBetween()
  //          1a5. stitch remaining edge pairs
  //              - SelectTopologyToKeep()
  //              - build sModifiedFaces list
  //              - call SmBrep::GlueEdgesGeneral()
  //              - place surviving edge on sCandidateEdges list
  ULONG lCount = 0;
  SmBoolean bMoreWorkNeeded = TRUE;
  while(   bMoreWorkNeeded 
        && lCount < 10) 
    {
      lCount ++;
      bMoreWorkNeeded = FALSE;

      // Check for Vertices coincident with Edges if asked.
      // NOTE: it is more efficient to do this first in the big loop,
      // before looking for coincident vertices.  This will create coincident
      // vertices, which would then be caught by that check.
      // Note also, this should obviate the need for the bMoreWorkNeeded loop altogether.

      if ( m_bSplitEdgesWithVertices )
        {
          // rebuild SmBrep values -- stored ones might be out of date.

          // get modified Brep Bounding box
          SmExtent3d sBrepBBox;
          pBrepToStitch->CalculateBoundingBox(sBrepBBox);
      
          // build modified Brep Edge spatial tree
          SmTArray<SmEdge*>   sEdges; 
          pBrepToStitch->GetEdges(sEdges);
          ULONG lNumPerNode = smos_Max(500,sEdges.GetSize()/10);
          SmTree *pEdgeTree = new (*pBrepToStitch->GetContext()) SmTree(sBrepBBox,lNumPerNode,lNumPerNode/10);
          NER(pEdgeTree);
          SmObjDelete sCleanTree(pEdgeTree);
      
          // load modified Brep Edge spatial tree with edges
          for(ULONG iii=0; iii<sEdges.GetSize(); iii++) 
            {
              SmExtent3d sEBBox;
              SmEdge  *pEdge  = sEdges[iii];
              SmCurve *pCurve = pEdge->GetCurve();
              SER(pCurve->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox));
              SER(pEdgeTree->AddToSpatialTree(sEBBox,pEdge));
            }

          SmTArray<SmObject*> sObjects;
          // was:  sCoinVertices.ReSet();
          pBrepToStitch->GetVertices(sVertices);

          // Check only if either vertex or edge is on candidate list.
          SmBoolean bCandidateVtx = TRUE, bCandidateEdge = TRUE;
          ULONG lFoundIndex;

          // for every vertex
          for(ULONG kkk=0; kkk<sVertices.GetSize(); kkk++) 
            {
              SmVertex *pV = sVertices[kkk];

              // If we are making a manifold solid only stitch lamina and wire topology. [B311]
              if ( m_bMakingManifoldSolid && ! (pV->IsLaminaVertex() || pV->IsWireVertex() ) )
                { continue; }

              // Check only if either vertex or edge is on candidate list.
              if (     cpOptCandidateVertices != NULL
                  && ! sCandidateVertices.FindElement( pV, lFoundIndex) )
                { bCandidateVtx = FALSE; }

              // for every edge near this vertex
              SmExtent3d sVBBox(pV->GetPoint());
                sVBBox.ExpandAbsolute(m_dStitchTol3d);
              SER(pEdgeTree->GetObjectsInBox(sVBBox,sObjects));
              for(ULONG lll=0; lll<sObjects.GetSize(); lll++) 
                {
                  SmEdge *pEdge = SM_CAST_PTR(SmEdge,sObjects[lll]);

                  // If we are making a manifold solid only stitch lamina and wire topology. [B311]
                  if ( m_bMakingManifoldSolid && ! (pEdge->IsLamina() || pEdge->IsWire() ))
                    { continue; }

                  // Check only if either vertex or edge is on candidate list.
                  if (     cpOptCandidateEdges != NULL
                      && ! sCandidateEdges.FindElement( pEdge, lFoundIndex) )
                    { bCandidateEdge = FALSE; }

                  if ( !bCandidateVtx && !bCandidateEdge )
                    { continue; }

                  // split edges when vertex is within tolerance
                  double dTol = smos_Max( m_dStitchTol3d,
                                          (double)(pV->GetTolerance() + pEdge->GetTolerance() ));
                  SmVertex *pNewVertex;
                  SmEdge   *pNewEdge;
                  sReasonType eReason;
                  double      dDistToEdge;
                  SER(sm_TestAndSplitEdge( pEdge, 
                                           pV->GetPoint(), 
                                           dTol,
                                           eReason, 
                                           dDistToEdge, 
                                           pNewVertex, 
                                           pNewEdge ));

                  // when split created a new vertex
                  if (pNewVertex) 
                    {
                      // set new vertex tolerance and add it to coincident vertex list
                      double dNewTol = smos_Max(pV->GetTolerance(),pNewVertex->GetTolerance());
#ifdef SM_USE_OLDTOL
                      SM_OLDTOL_LINE pNewVertex->SetTolerance(dNewTol);
#endif // SM_USE_OLDTOL
                      bMoreWorkNeeded = TRUE;  // was:  sCoinVertices.Add(pNewVertex);

                      sCandidateVertices.Add( pNewVertex );
                      if ( !bCandidateVtx ) { sCandidateVertices.Add( pV ); }
                    }

                  // when split created a new edge
                  if (pNewEdge) 
                    {
                      // add new edge->curve to edge spatial tree
                      SmExtent3d sEBBox;
                      SmCurve *pCurve = pNewEdge->GetCurve();
                      SER(pCurve->CalculateBoundingBox(pNewEdge->GetInterval(),&sEBBox));
                      SER(pEdgeTree->AddToSpatialTree(sEBBox,pNewEdge));

                      sCandidateEdges.Add( pNewEdge );
                      if ( !bCandidateEdge ) { sCandidateEdges.Add( pEdge ); }
                    }
                } // end iter every edge near the vertex
            } // end iter every vertex looking for edges to split
        } // end if m_bSplitEdgesWithVertices == TRUE check


      // Now glue coincident vertices, including those created in the Vertex/Edge intersections.
      //  (squeeze coincident vertices connected by one short common edge).
    
      // Here we sort the vertices along the vector
      // (1,1,1) to make this operation fast.

      //cbi A better optimization: if we have cpOptCandidateVertices,
      //cbi sort them too, first, and reject any other verts outside the range
      //cbi of those in cpOptCandidateVertices.

      //SmPoint3d sPnt(0,0,0);
      SmVector3d sVec(1,1,1);
      // Unitize the vector so that the parameter corresponds to 3d distance.
      SER( sVec.Unitize() );

      sSortedVerts.ReSet();        

      // Precompute lamina flag and sort value
      sIsLaminaVertex.ReSet();
      sSortParameter.ReSet();

      SmTArray<SmEdge*> sWireEdges;
      pBrepToStitch->GetWireEdges(sWireEdges);

      // for every vertex - label lamina vertices and 
      //     store common line projection parameter for later sorting
      pBrepToStitch->GetVertices(sVertices);
      ULONG lNumVerts = sVertices.GetSize();
      for(ii=0; ii<lNumVerts; ii++) 
        {
          SmVertex *pV = sVertices[ii];

          // lamina vertex := a vertex with one or more lamina edges
          if (pV->IsLaminaVertex()) { sIsLaminaVertex.Add(TRUE);
                                    }
          else                      { sIsLaminaVertex.Add(FALSE);
                                    }
          // wire vertex := a vertex with only wire edges
          if ( pV->IsWireVertex() ) { bHasWireVertex = TRUE;
                                    }

          // get and save parameter of projecting vertex to (1,1,1) line
          double dTest;
          // Note: this is just the dot product of sVec and the vertex point:
          // SER(smgu_LineClosestPoint(sPnt,sVec,pV->GetPoint(),dTest));
          dTest = sVec.Dot( pV->GetPoint() );

          sSortParameter.Add(dTest);

        } // end iter every vertex - getting sort parameter and lamina labels
      
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          sIsLaminaVertex.Dump() ;
          sSortParameter.Dump() ;
        }
#endif // SM_DEBUG_CODE

      // build sSortedIndex array to store vertex indices sorted based
      //   on their projection parameter to the (1,1,1) line.
      SmTArray<ULONG> sSortedIndex;
      for(ii=0; ii<sVertices.GetSize(); ii++) 
        {
          double dTest = sSortParameter[ii];

          // Sort only lamina vertices when making manifold solids, unless there's a wire vertex (which could be stitched to any vertex)
          if ( !bHasWireVertex && m_bMakingManifoldSolid && ! sIsLaminaVertex[ii] )
            { continue; }

          // init sSortedIndex with 1st entry
          if (sSortedIndex.GetSize() == 0) 
            {
              sSortedIndex.Add(ii); // Put first valid vertex into index list
              continue;
            }

          // Binary search.
          ULONG lMin = 0;
          ULONG lMax = sSortedIndex.GetSize() - 1;
          if (dTest >= sSortParameter[sSortedIndex.GetLast()]) 
            {
              sSortedIndex.Add(ii);
            }
          else if (dTest <= sSortParameter[sSortedIndex[0]]) 
            {
              sSortedIndex.InsertAt(0,ii);
            }
          else 
            {
              while (lMin != lMax-1 && lMin != lMax) 
                {
                  ULONG lMid = (lMin+lMax) / 2;
                  if (dTest > sSortParameter[sSortedIndex[lMid]]) 
                    {
                      lMin = lMid;
                    }
                  else 
                    {
                      lMax = lMid;
                    }
                }
              sSortedIndex.InsertAt(lMax,ii);
            }
        } // end iter every vertex - building sSortedIndex Array

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          sIsLaminaVertex.Dump() ;
          sSortParameter.Dump() ;
          sSortedIndex.Dump() ;
        }
#endif // SM_DEBUG_CODE

      // arrive here after the following arrays have been set
      // 1. sVertices      = list of all target vertices to stitch
      // 2. sSortParameter = associated projection param to the 1,1,1 line
      // 3. sSortedIndex   = ordered list of sVertices indices based on the sSortParameter list

      // for every vertex - build list of coincident vertices into sCoinVertices,
      //                  - then stitch (glue or squeeze) coincident vertices

      for(ii=0; ii<sSortedIndex.GetSize(); ii++) 
        {
          SmVertex *pV = sVertices[sSortedIndex[ii]];
          
          // skip vertices already processed
          if (pV == NULL) { continue; }

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              smgfx_SetLook(2,5, 1,0,0); pV->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // mark this vertex
          sVertices[sSortedIndex[ii]] = NULL;

          double dVParam = sSortParameter[sSortedIndex[ii]];
          
          // load sCoinVertices with all vertices coincident with pV
          sCoinVertices.ReSet();
          for(jj=ii+1; jj<sSortedIndex.GetSize(); jj++) 
            {
              SmVertex *pTestV   = sVertices[sSortedIndex[jj]];
              double    dTVParam = sSortParameter[sSortedIndex[jj]];

              // skip vertices already tested
              if (pTestV == NULL) continue;

              double dTestTol = smos_Max(m_dStitchTol3d,
                                         (double)(pTestV->GetTolerance() + pV->GetTolerance()));

              // done looking when pTestV is too far along the vector
              if (dTVParam-dVParam > dTestTol)
                { break; }

              // get 3d distance between vertex pairs and tolerance
              double dDist    = pV->GetPoint().DistanceBetween(pTestV->GetPoint());

              // when vertices are within tolerance
              if (dDist < dTestTol) 
                {
                  // add vertex to the coincident list - save biggest coincident vertex gap value
                  sCoinVertices.Add(pTestV);
                  sVertices[sSortedIndex[jj]] = NULL;
                  if ( dDist > rdMaxVertexGap ) { rdMaxVertexGap = dDist; }
                }
              else if ( pdMinUnstitchedVertGap != NULL && dDist < *pdMinUnstitchedVertGap )
                {
                  *pdMinUnstitchedVertGap = dDist;
                }
              
            } // end iter sorted vertices until no more coincidences are found
          
          // during iter ii on sVertices array, arrive here after
          // 1. all vertices coincident to sVertices[ii] are in list sCoinVertices

#ifdef SM_DEBUG_CODE // list all vertices coincident with sVertices[sSortedIndex[ii]]
          if(bDebugMe)
            { sCoinVertices.Dump() ; }
#endif // SM_DEBUG_CODE

          // when stitching is restricted to a cpOptCandidateVertices list and pV is not a targeted vertex,
          //  - remove vertices from sCoinVertices not on the allowed list.
          //  - and don't bother if there are no coincident vertices.
          // Note: if rejected for this reason, it doesn't factor into pdMinUnstitchedVertGap.
          ULONG lNumCoin = sCoinVertices.GetSize();
          if ( cpOptCandidateVertices != NULL  && lNumCoin > 0 )
            {
              // Remove all sCoinVertices vertices not on the cpOptCandidateVertices list.
              // Also allow it if pV itself is on the Candidate list.
              ULONG lFoundIndex;
              if ( ! sCandidateVertices.FindElement( pV, lFoundIndex) )
                {
                  // Copy the to-be-kept ones into sCoinV.
                  sCoinV.ReSet();
                  for ( jj=0; jj<sCoinVertices.GetSize(); jj++ )
                    {
                      if ( sCandidateVertices.FindElement(sCoinVertices[jj],lFoundIndex) )
                        {
                          sCoinV.Add( sCoinVertices[jj] );
                        }
                    }
                  // And copy the survivors back into sCoinVertices.
                  sCoinVertices.ReSet();
                  sCoinVertices.Append(sCoinV);
            
                } // end pV is not in candidate list
            } // end cpOptCandidateVertices check
          
#ifdef SM_DEBUG_CODE // list all vertices coincident with sVertices[sSortedIndex[ii]]
          if(bDebugMe)
            { sCoinVertices.Dump(); }
#endif // SM_DEBUG_CODE

          // Now glue (or squeeze) every vertex that is coincident with pV.
          for(jj=0; jj<sCoinVertices.GetSize(); jj++) 
            {
              SmBoolean bKeepFirst;

              // when     m_bMakingManifoldSolid - only glue pairs of Lamina vertices or wire vertices
              // when not m_bMakingManifoldSolid - glue all pairs of coincident vertices
              if (   ! m_bMakingManifoldSolid
                  || (( pV->IsLaminaVertex() && sCoinVertices[jj]->IsLaminaVertex() )
                    || (pV->IsWireVertex() || sCoinVertices[jj]->IsWireVertex()) ) )
                 
                {
                  double dDistToV = pV->GetPoint().DistanceBetween(sCoinVertices[jj]->GetPoint());

                  // "No Vertex tolerances greater than the min edge length" is a nice rule, but
                  // doesn't seem to work well in practice
                  //// skip vertices farther apart than the shortest squeezed edge
                  //// NOTE: if m_bSqueezeSmallEdges is False, dMinEdgeLength is SM_BIG_DOUBLE.
                  //if (dDistToV > dMinEdgeLength) 
                  //  {
                  //    if ( pdMinUnstitchedVertGap != NULL && dDistToV < *pdMinUnstitchedVertGap )
                  //      { *pdMinUnstitchedVertGap = dDistToV; }

                  //    continue;
                  //  }

                  // select the vertex to keep
                  double dMaxDistance;
                  SER(m_rStitchCallback.SelectTopologyToKeep(pV,sCoinVertices[jj],dDistToV,dMaxDistance,bKeepFirst));

#ifdef SM_DEBUG_CODE
                  if (bDebugMe) 
                    {
                      smgfx_Erase();
                      smgfx_SetLook(1,3) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,5, 1,0,0); pV->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(3,7, 0,0,1); sCoinVertices[jj]->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE
                  // skip vertices farther apart than allowed stitching tolerance + vertex tolerances
                  double dTol = smos_Max( m_dStitchTol3d,
                                          (double)( pV->GetTolerance() + sCoinVertices[jj]->GetTolerance() ));
                  if (dMaxDistance > dTol )
                    {
                      if ( pdMinUnstitchedVertGap != NULL && dDistToV < *pdMinUnstitchedVertGap )
                        { *pdMinUnstitchedVertGap = dDistToV; }

                      continue; // Don't stitch it if distance gets too big.
                    }

                  // squeeze/glue state
                  // glue   : combine two coincident vertices not connected with a common edge
                  // squeeze: combine two coincident vertices and delete edge when 
                  //              vertices are connected by a common short edge.
                  ULONG   bDoGlue      = TRUE;
                  ULONG   bDoSqueeze   = FALSE;
                  SmEdge *pSqueezeEdge = NULL;

                  // don't glue when vertices have common edges 
                  // - and squeeze only if common vertices are connected by one short edge

                  // for every common edge between the vertices
                  pV->GetCommonEdges(sCoinVertices[jj],sCommonEdges);
                  for(ULONG kk=0; kk<sCommonEdges.GetSize(); kk++) 
                    {
                      // get edge->curve approximate length
                      SmEdge *pE          = sCommonEdges[kk];
                      double  dEdgeLength = pE->GetCurve()->ApproximateLength(pE->GetInterval(),10);

                      // if edge is short
                      if (dEdgeLength < 2.0 * dDistToV) 
                        {
                          // bDoSqueeze is False the first time around,
                          // so it it's True, there must have been another
                          // short edge connecting the two vertices.
                          // Don't squeeze or glue a vertex pair when it has
                          // more than one short common edge.
                          if(bDoSqueeze)      { bDoSqueeze = FALSE;
                                                bDoGlue    = FALSE;
                                                break;
                                              }

                          // squeeze lamina and wire edges - don't squeeze manifold edges
                          if(   pE->IsLamina() 
                             || pE->IsWire()) { bDoSqueeze   = TRUE;
                                                bDoGlue      = FALSE;
                                                pSqueezeEdge = pE;
                                              }
                          else                { bDoSqueeze   = FALSE;
                                                bDoGlue      = FALSE;
                                              }

                        } // end edge is short check
                    } // end iter kk every edge connecting these two vertices

                  // skip squeezing vertices when m_bSqueezeSmallEdges is set to FALSE
                  if ( bDoSqueeze && m_bSqueezeSmallEdges == FALSE )
                    {
                      continue;
                    }

                  // squeeze or glue vertices together (make pV the retained vertex)
                  if (bKeepFirst) 
                    {
                      if (bDoSqueeze)   
                        { SER(pBrepToStitch->SqueezeEdge(pSqueezeEdge,pV)) ; }
                      else if (bDoGlue) 
                        { SER(pBrepToStitch->GlueVertices(pV,sCoinVertices[jj],&sWireEdges)) ; }
                    } // end keep first vertex branch
                  else // keep second vertex branch
                    {
                      if (bDoSqueeze)   
                        { SER(pBrepToStitch->SqueezeEdge(pSqueezeEdge,sCoinVertices[jj])) ;}
                      else if (bDoGlue) 
                        { SER(pBrepToStitch->GlueVertices(sCoinVertices[jj],pV,&sWireEdges)) ;}

                      pV = sCoinVertices[jj];

                    } // end keep second vertex branch

#ifdef SM_DEBUG_CODE
                  if (bDebugMe)  // draw faces, face->edges, face->vertices in various orders to allow
                    {            //  an inspection of what may lie on top of what
                      SM_DUMP_AND_ASSERT_VALID(pBrepToStitch) ;

                      SmTArray<SmEdge*> sDebugEdges;
                      pV->GetEdges(sDebugEdges);

                      smgfx_Erase();
                      smgfx_SetLook(1,3) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,5, 1,0,0); pV->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(2,4, .1, .1, 1.);
                      for (ULONG di = 0; di < sDebugEdges.GetSize(); ++di)
                      { sDebugEdges[di]->Draw(); sm_GraphicsLoop();}
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE
                  // The surviving vertex (which is pV) should be a candidate.
                  if ( cpOptCandidateVertices != NULL )
                    { sCandidateVertices.AddUnique( pV ); }

                } // end skip nonLamina vertices when making manifold solids check 
            } // end iter jj every coin vertex gluing (or squeezing) every coincident vertex 
        } // end iter ii every sorted vertex index

      // arrive here after squeezing/gluing all coincident vertex sets

#ifdef SM_DEBUG_CODE
  if (bDebugMe)  // draw faces, face->edges, face->vertices in various orders to allow
    {            //  an inspection of what may lie on top of what
      SM_DUMP_AND_ASSERT_VALID(pBrepToStitch) ;

      SmTArray<SmFace*>   sFaces ;     pBrepToStitch->GetFaces   (sFaces) ;
      SmTArray<SmEdge*>   sEdges ;     pBrepToStitch->GetEdges   (sEdges) ;
      SmTArray<SmVertex*> sVerts ;     pBrepToStitch->GetVertices(sVerts) ;
      SmTArray<SmEdge*>   sFaceEdges ;
      SmTArray<SmVertex*> sFaceVertices ;
      
      smgfx_Erase();
      smgfx_SetLook(1,3) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; for(ii=0;ii<sFaces.GetSize();ii++)
                                    { sFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; 
                                      sFaces[ii]->GetEdges(sFaceEdges) ;  sFaceEdges.Dump() ;
                                      sFaces[ii]->GetVertices(sFaceVertices) ; sFaceVertices.Dump() ;
                                      for(jj=0;jj<sFaceEdges.GetSize();jj++)
                                        { smgfx_SetLook(2,3, 1,0,0) ; sFaceEdges[jj]->Draw() ; sm_GraphicsLoop() ; }
                                      for(jj=0;jj<sFaceVertices.GetSize();jj++)
                                        { smgfx_SetLook(2,6, 1,0,1) ; sFaceVertices[jj]->Draw() ; sm_GraphicsLoop() ; }
                                    }
      smgfx_SetLook(3,4, 0,1,0) ; for(ii=0;ii<sEdges.GetSize();ii++)
                                    { sEdges[ii]->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1,0,0) ; for(ii=0;ii<sVerts.GetSize();ii++)
                                    { sVerts[ii]->Draw() ; sm_GraphicsLoop() ; }
      pBrepToStitch->Dump() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

//cbi: optimization: do we have to look over all verts in the Brep?
//cbi  Couldn't we keep track of the ones we glued?

      // next: glue edges 
      //   because vertices have been glued - all edge pairs to be glued will be
      //   connected to a common vertices at both their ends.
      pBrepToStitch->GetVertices(sVertices);
      for(ii=0; ii<sVertices.GetSize(); ii++) 
        {
          SmVertex *pV = sVertices[ii];
          SmBoolean bDoneWithVertex = FALSE;

          // while using this vertex to find glueable edge pairs
          while (!bDoneWithVertex) 
            {
              bDoneWithVertex = TRUE;

              // get all edges connected to this vertex
              pV->GetEdges(sVEdges);

#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,3) ; pBrepToStitch->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,6, 1,0,0) ; pV->Draw(); sm_GraphicsLoop();
                  for(jj=0; jj<sVEdges.GetSize(); jj++) 
                    { smgfx_SetLook(3+jj, 7+jj, 
                                    (double)((jj+0)%3)/2.0,
                                    (double)((jj+1)%3)/2.0,
                                    (double)((jj+2)%3)/2.0) ; sVEdges[jj]->Draw(); sm_GraphicsLoop(); 
                    }
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              // for every vertex->edge[jj]
              for(jj=0; jj<sVEdges.GetSize(); jj++) 
                {
                  SmEdge *pE = sVEdges[jj];

                  // skip edges already processed
                  if (pE==NULL) 
                    { continue; }

                  // If we are making a manifold solid only stitch lamina and wire edges
                  if ( m_bMakingManifoldSolid && !(pE->IsLamina() || pE->IsWire()))
                    { continue; } 

                  // iter every other vertex->edge[kk]
                  for(ULONG kk=jj+1; kk<sVEdges.GetSize(); kk++) 
                    {
                      SmEdge *pTestE = sVEdges[kk];

                      // skip edges already processed
                      if (pTestE == NULL) 
                        { continue; }

                      // If we are making a manifold solid only stitch lamina and wire edges
                      if ( m_bMakingManifoldSolid && !(pTestE->IsLamina() || pTestE->IsWire()))
                        { continue; }

                      // skip edges that don't share common vertices on both ends
                      if (pE->GetOtherVertex(pV) != pTestE->GetOtherVertex(pV)) 
                        { continue; }
                      
                      // Skip this one if there is a Candidate list
                      // and neither the edge nor its mate is on it.
                      if (cpOptCandidateEdges) 
                        {
                          ULONG lFoundIndex;
                          if (   (!sCandidateEdges.FindElement(pTestE,lFoundIndex))
                              && (!sCandidateEdges.FindElement(pE,lFoundIndex)) ) 
                            { continue; }
                        }
                      
                      // Both edges have same vertices if make it here
                      SmCurve *pECurve  = pE->GetCurve();
                      SmCurve *pTECurve = pTestE->GetCurve();

#ifdef SM_DEBUG_CODE
                      if (bDebugMe) 
                        {
                          SmTArray<SmFace*> sDrawF1, sDrawF2;
                          pE->GetFaces(sDrawF1);   
                          pTestE->GetFaces(sDrawF2);
                          
                          smgfx_Erase();
                          smgfx_SetLook(1,2) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(4,6, 1,0,0); pE->Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(6,8, 0,0,1); pTestE->Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(2,3, 0,1,1); for(ULONG mmm=0; mmm<sDrawF1.GetSize(); mmm++) 
                                                       { sDrawF1[mmm]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop(); }
                          sm_GraphicsLoop();
                          smgfx_SetLook(3,4, 1,0,1); for(ULONG nnn=0; nnn<sDrawF2.GetSize(); nnn++) 
                                                       { sDrawF2[nnn]->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop(); }
                          sm_GraphicsLoop();
                        }   
#endif // SM_DEBUG_CODE

                      // Need to figure out dTStart and dTEnd and relative 
                      // orientation of curves.
                      SmPoint3d sMidPnt;

                      // select new tolerance
                      double dTol = smos_Max( m_dStitchTol3d,
                                              (double)( pTestE->GetTolerance() + pE->GetTolerance() ));

                      // locals
                      SER(pECurve->EvaluatePoint(pE->GetInterval().Evaluate(0.5),sMidPnt));
                      SmBoolean bSuccess;
                      double dDropParam, dDistToCurve;

                      // sometimes failure here due to bad 3d curves or overlapping faces

                      SmExtent1d sTEIvl  = pTestE->GetInterval();
                      double dGuessParam = sTEIvl.GetMid();

                      // This tolerance tells DropPoint to quit looking if the solution
                      // is going to be larger than this value.
                      // If we're finding pdMinUnstitchedEdgeGap, collect every solution.
                      double dDropTol = ( pdMinUnstitchedEdgeGap == NULL ) ? dTol : SM_BIG_DOUBLE;

                      // skip edge pair if pE->midPoint is not within tolerance of pTestE

                      SmStatus eStat = pTECurve->DropPoint( sTEIvl,         // in : target curve allowed domain
                                                            sMidPnt,        // in : Point to drop to curve
                                                            NULL,           // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                            //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                            //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                            dDropTol,       // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                            //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                            //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                            //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                           &dGuessParam,    // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                            bSuccess,       // out: TRUE = found a drop point
                                                            dDropParam,     // out: found drop curve param
                                                            dDistToCurve) ; // out: found drop distance
                                                                            // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                            //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                            //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                            //      default:[SM_SO_MINIMIZE] to preserve original behavior
                      if ( !bSuccess || eStat != SM_SUCCESS )
                        { continue; }

                      if ( dDistToCurve > dTol )
                        {
                          if ( pdMinUnstitchedEdgeGap != NULL && dDistToCurve < *pdMinUnstitchedEdgeGap )
                            { *pdMinUnstitchedEdgeGap = dDistToCurve; }

                          continue;
                        }

                      // Check relative orientation of the edge curves.
                      SmVector3d sPV[2], sPVTest[2];
                      SER(pECurve->Evaluate(pE->GetInterval().Evaluate(0.5),1,TRUE,sPV));
                      SER(pTECurve->Evaluate(dDropParam,1,TRUE,sPVTest));

                      SmOrientType eRelativeOrientation = SM_OT_SAME;
                      double dTStart = pTestE->GetInterval().GetMin();
                      double dTEnd   = pTestE->GetInterval().GetMax();
                      if (sPV[1].Dot(sPVTest[1]) < 0.0) 
                        {
                          eRelativeOrientation = SM_OT_OPPOSITE;
                          double dTTmp = dTStart;
                          dTStart      = dTEnd;
                          dTEnd        = dTTmp;
                        }
                      
                      // skip edge pairs not within tolerance of one another
                      SmBoolean bEdgesAreLines = (pECurve->GetDegree()  == 1 &&  pECurve->GetNumberNaturalKnots() == 4) 
                                             &&  (pTECurve->GetDegree() == 1 && pTECurve->GetNumberNaturalKnots() == 4);
                      dDistToCurve = 0.0;
                      if (!bEdgesAreLines) // if edges not lines then do complex test
                        {
                          // do a quick 5 point distance check
                          SER(pECurve->CurveMaxDistanceBetween(pE->GetInterval(),
                                                              *pTECurve,
                                                               dTStart,
                                                               dTEnd,
                                                               5,
                                                              &dTol,
                                                               dDistToCurve));
                          if (dDistToCurve > dTol) 
                            {
                              if ( pdMinUnstitchedEdgeGap != NULL && dDistToCurve < *pdMinUnstitchedEdgeGap )
                                { *pdMinUnstitchedEdgeGap = dDistToCurve; }

                              continue;
                            }

                          // when asked - do a precise distance check
                          if ( ! m_bFastEdgeCompare ) 
                            {
                              // Passed an initial quick test with 5 points now do a precise measurement
                              SER(pECurve->CurveMaxDistanceBetween(pE->GetInterval(),  // in : interval limit for this curve                                         
                                                                  *pTECurve,           // in : other curve to test                                                   
                                                                   dTStart,            // in : OtherCurve param mapping to ThisCurve Interval.Min value              
                                                                   dTEnd,              // in : OtherCurve param mapping to ThisCurve Interval.Max value              
                                                                   0,                  // in : Min number of samples to take. 0 = slow precise measure
                                                                  &dTol,               // in : Max allowed gap.  Quit searching once this value is exceeded.
                                                                   dDistToCurve));     // out: Set to max gap size seen.
                                                                                       //        a. When lNumSamples == 0 This is the max curve/curve gap.
                              if (dDistToCurve > dTol) 
                                {
                                  if ( pdMinUnstitchedEdgeGap != NULL && dDistToCurve < *pdMinUnstitchedEdgeGap )
                                    { *pdMinUnstitchedEdgeGap = dDistToCurve; }

                                  continue;
                                }
                            }  // end SlowEdgeCompare check
                        } // end edges are not lines check
                          
                      // If make it here then we need to glue edges

                      // save the maxEdgeGap value
                      if ( dDistToCurve > rdMaxEdgeGap ) { rdMaxEdgeGap = dDistToCurve; }
                      
                      // skip edges farther apart than the dMaxDistance
                      SmBoolean bKeepFirst;
                      double dMaxDistance;
                      SER(m_rStitchCallback.SelectTopologyToKeep(pE,           // in : 1st of two coincident objects to stitch together
                                                              pTestE,       // in : 2nd of two coincident objects to stitch together
                                                              dTol,         // in : max allowed distance between coincident objects      
                                                              dMaxDistance, // out: max distance found between two target objects
                                                              bKeepFirst)); // out: TRUE = keep TopologyElement1                 
                                                                            //      FALSE= keep TopologyElement2                 

                      if ( dMaxDistance > dTol )
                        {
                          if ( pdMinUnstitchedEdgeGap != NULL && dMaxDistance < *pdMinUnstitchedEdgeGap )
                            { *pdMinUnstitchedEdgeGap = dMaxDistance; }

                          continue;
                        }

                      // branch on which edge to keep
                      if (bKeepFirst) 
                        {
                          // place every pTestE->Face on ModifiedFaces list
                          pTestE->GetFaces(sEdgeFaces);
                          for(ULONG mm=0; mm<sEdgeFaces.GetSize(); mm++) 
                            {
                              sModifiedFaces.AddUnique(sEdgeFaces[mm]);
                            }

                          // glue pE/pTestE edge pair - keep pE
                          if (pBrepToStitch->GlueEdgesGeneral(pE,
                                                           eRelativeOrientation,
                                                           dDistToCurve,
                                                           m_bDoRegionNesting,
                                                           pTestE) == SM_SUCCESS) 
                            {
                              sVEdges[kk]     = NULL;
                              bDoneWithVertex = FALSE;
                            }
                          else if (!m_bIgnoreProblems) 
                            {
                              SER(SM_ERR); 
                            }
                          else 
                            {
                              SmTArray<SmEdge *> tmpEdges;
                              pBrepToStitch->GetEdges(tmpEdges);
                              ULONG foundIndex;
                              if (tmpEdges.FindElement(sVEdges[kk], foundIndex) == FALSE)
                                sVEdges[kk] = NULL;
                            }
                        } // end keep pE branch
                      else // keep pTestE branch
                        {
                          // place every pE->Face on ModifiedFaces list
                          pE->GetFaces(sEdgeFaces);
                          for(ULONG mm=0; mm<sEdgeFaces.GetSize(); mm++) 
                            {
                              sModifiedFaces.AddUnique(sEdgeFaces[mm]);
                            }

                          // glue pE/pTestE edge pair - keep pTestE
                          if (pBrepToStitch->GlueEdgesGeneral(pTestE,
                                                           eRelativeOrientation,
                                                           dDistToCurve,
                                                           m_bDoRegionNesting,
                                                           pE) == SM_SUCCESS) 
                            {
                              sVEdges[jj] = pTestE;
                              sVEdges[kk] = NULL;
                              pE = pTestE;
                              bDoneWithVertex = FALSE;
                            }
                          else if (!m_bIgnoreProblems) 
                            {
                              SER(SM_ERR); 
                            }
                          else {
                              SmTArray<SmEdge *> tmpEdges;
                              pBrepToStitch->GetEdges(tmpEdges);
                              ULONG foundIndex;
                              if (tmpEdges.FindElement(sVEdges[jj], foundIndex) == FALSE)
                                sVEdges[jj] = NULL;
                            }
                        } // end keep pTestE branch
                      
                      // add the surviving edge, pE, to list of candidate edges
                      if (cpOptCandidateEdges) { sCandidateEdges.Add(pE); }
                      
                      // increment the number of glue edge attempts
                      rlStitchedEdges ++;

                      if ( !bDoneWithVertex ) { break; }

                    } // end iter every vertex->edge[kk]

                  if (!bDoneWithVertex) break; 

                } // end iter every vertex->edge[jj]
            } // while (!bDoneWithVertex) 
        } // end iter every vertex[ii] - looking for edges to glue

      // arrive here after all coincident vertices and edges are glued.
      // next: when asked split edges at coincident vertices

#ifdef SM_DEBUG_CODE
      if (bDebugMe)  // draw faces, face->edges, face->vertices in various orders to allow
        {            //  an inspection of what may lie on top of what
          SM_DUMP_AND_ASSERT_VALID(pBrepToStitch) ;

          SmTArray<SmFace*>   sFaces ;     pBrepToStitch->GetFaces   (sFaces) ;
          SmTArray<SmEdge*>   sEdges ;     pBrepToStitch->GetEdges   (sEdges) ;
          SmTArray<SmVertex*> sVerts ;     pBrepToStitch->GetVertices(sVerts) ;
          SmTArray<SmEdge*>   sFaceEdges ;
          SmTArray<SmVertex*> sFaceVertices ;
      
          smgfx_Erase();
          smgfx_SetLook(1,3) ; pBrepToStitch->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; for(ii=0;ii<sFaces.GetSize();ii++)
                                        { sFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; 
                                          sFaces[ii]->GetEdges(sFaceEdges) ;  sFaceEdges.Dump() ;
                                          sFaces[ii]->GetVertices(sFaceVertices) ; sFaceVertices.Dump() ;
                                          for(jj=0;jj<sFaceEdges.GetSize();jj++)
                                            { smgfx_SetLook(2,3, 1,0,0) ; sFaceEdges[jj]->Draw() ; sm_GraphicsLoop() ; }
                                          for(jj=0;jj<sFaceVertices.GetSize();jj++)
                                            { smgfx_SetLook(2,6, 1,0,1) ; sFaceVertices[jj]->Draw() ; sm_GraphicsLoop() ; }
                                        }
          smgfx_SetLook(3,4, 0,1,0) ; for(ii=0;ii<sEdges.GetSize();ii++)
                                        { sEdges[ii]->Draw() ; sm_GraphicsLoop() ; }
          smgfx_SetLook(5,6, 1,0,0) ; for(ii=0;ii<sVerts.GetSize();ii++)
                                        { sVerts[ii]->Draw() ; sm_GraphicsLoop() ; }
          pBrepToStitch->Dump();
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end while(bMoreWorkNeeded && lCount < 10)

  // arrive here after 
  // 1. sliver faces removed and, when asked, short lamina edgea are squeezed
  // 2. vertices and edges are glued 
  // 3. when asked, edges are split at coincident vertices

  // count the number of lamina edges in modified Brep
    SmTArray<SmEdge*>   sEdges; 
    pBrepToStitch->GetEdges(sEdges);
  for(ULONG ll=0; ll<sEdges.GetSize(); ll++) 
    {
      SmEdge *pEdge = sEdges[ll];
      if (pEdge->IsLamina()) 
        {
#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              if ( FALSE ) {
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,0,0 ); pBrepToStitch->Draw(TRUE); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
              smgfx_SetLook(6,8, 1,0,0); pEdge->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif  
          rlLaminaEdges++;
        }
    }

  // if edges were stitched - notifiy all modified Brep faces of impending change
  if (rlStitchedEdges > 0) 
    {
      pBrepToStitch->Notify(SM_NO_PRE_EDIT, pBrepToStitch, NULL, NULL);
      SmTArray<SmFace*> sFaces;
      pBrepToStitch->GetFaces(sFaces);
      for(ULONG jjj=0; jjj<sFaces.GetSize(); jjj++)
        {
          SmFace *pF = sFaces[jjj];
          pF->Notify(SM_NO_PRE_EDIT, pF, SM_NO_GET_BREP(pF), NULL);
        }
    }
  
  SmTArray<SmRegion*> sRegions;
  
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SM_DUMP_AND_ASSERT_VALID(pBrepToStitch);
    }
#endif // SM_DEBUG_CODE

  // when asked - find and set the infinite region by classifying a ray from a point outside the Brep
  if (m_bDoRegionNesting) 
    {
      pBrepToStitch->GetRegions(sRegions);
      if (sRegions.GetSize() > 1) 
        {
          SmTemporaryChangeValue<SmBoolean> sChange1(pBrepToStitch->m_bEditingEnabled,FALSE);
          SER(pBrepToStitch->FindAndSetInfiniteRegion());
        }
    }
  
  // for every modified face - remove UVTrimCurves

  {
      //// This was removed for efficiency, as the edited edges are already refreshed and
      //// we want to preserve the trim curves if possible, specifically for
      //// CrvOnSurf edges.
      //for (ii = 0; ii < sModifiedFaces.GetSize(); ii++)
      //{
      //    SmFace* pF = sModifiedFaces[ii];
      //    //double dAveCSGap, dMaxCSGap, dAveVGap, dMaxVGap, dAveUVGap, dMaxUVGap;

      //    // delete UVTrimCurves
      //    pF->RemoveUVTrimCurves();

      //    //// rebuild UVTrimCurves
      //    //pF->CreateUVTrimCurves(TRUE, NULL, NULL, dAveCSGap, dMaxCSGap, dAveVGap, dMaxVGap, dAveUVGap, dMaxUVGap);
      //} // end iter every sModifiedFace
  }
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SM_DUMP_AND_ASSERT_VALID(pBrepToStitch);
    }
#endif // SM_DEBUG_CODE
  
  // all done
  return SM_SUCCESS;

} // end SmStitch::DoStitching

/*******************************************************************//**
PURPOSE: Select a ring around the current set of faces by looking for
     adjacencies at the edges.

NOTES: increments unlocked mark value 
***********************************************************************/
SmStatus SmStitch::SelectRing
 (SmBrep                  * pBrepToOrient,
  const SmTArray<SmFace*> & crOriginalFaces,
  ULONG                     lNumSamples,
  SmTArray<SmFace*>       & rRingFaces,
  SmTArray<SmBoolean>     & rSameOrientation)
{   
    rRingFaces.ReSet();
    rSameOrientation.ReSet();

    // increment and lock an unlocked mark
    SmNewMarkAndLock sMarkLock( pBrepToOrient->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
    SmMarkType eMarkType = sMarkLock.GetMarkType() ;

    SmExtent3d sBrepBBox;
    pBrepToOrient->CalculateBoundingBox(sBrepBBox,TRUE);
    sBrepBBox.ExpandAbsolute(pBrepToOrient->GetTolerance()*100.0);

    SM_PTR_ARRAY(sFStack, SmFace, 256);  // SmTArray<SmFace *>
    SM_PTR_ARRAY(sEdges,  SmEdge, 256);  // SmTArray<SmEdge *>
        
    pBrepToOrient->GetEdges(sEdges);
    ULONG lNumPerNode = smos_Max(500,sEdges.GetSize()/10);
    SmTree *pEdgeTree = new (*pBrepToOrient->GetContext()) SmTree(sBrepBBox,lNumPerNode,lNumPerNode/10);
    NER(pEdgeTree);
    SmObjDelete sCleanTree(pEdgeTree);
        
    for(ULONG iii=0; iii<sEdges.GetSize(); iii++) {
        SmEdge *pEdge = sEdges[iii];
        SmExtent3d sEBBox;
        SmCurve *pCurve = pEdge->GetCurve();
        SER(pCurve->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox));
        SER(pEdgeTree->AddToSpatialTree(sEBBox,pEdge));
    }

    for(ULONG j=0; j<crOriginalFaces.GetSize(); j++) {
        crOriginalFaces[j]->Mark(eMarkType);
    }
    sFStack.Append(crOriginalFaces);

    SmEdgeuse *sEUData[256];
    SmTArray<SmEdgeuse*> sEdgeuses(256,sEUData);

    SmObject *sOData[256];
    SmTArray<SmObject*> sObjects(256,sOData);

    while (sFStack.GetSize() > 0) {
        SmFace *pF = sFStack.GetLast();
        sFStack.RemoveLast();

        SmFaceuse *pFU = pF->GetUpwardFaceuse();
        pFU->GetEdgeuses(sEdgeuses);
        for(ULONG i=0; i<sEdgeuses.GetSize(); i++) {
            SmEdgeuse *pEU = sEdgeuses[i];
            if (pEU->GetEdge()->IsManifold()) {
                // we are already stitched just orient it the easy way.
                SmEdgeuse *pRadialEU = pEU->GetRadial();
                SmFaceuse *pFUse = pRadialEU->GetFaceuse();
                SmFace *pRadF = pFUse->GetFace();
                if (pRadF->IsMarked(eMarkType)) continue;
                // Check orientation
                if (pFUse->GetOrientation() == SM_OT_OPPOSITE) {
                    rSameOrientation.Add(FALSE);
                }
                else {
                    rSameOrientation.Add(TRUE);
                }
                pRadF->Mark(eMarkType);
                rRingFaces.Add(pRadF);
                continue;
            }
            for(ULONG j=0; j<lNumSamples; j++) {
                double dParam = (j+1.0) / (lNumSamples + 2.0);
                SmPoint3d sPnt, sDir;
                SmExtent1d sOrigIvl = pEU->GetEdge()->GetInterval();
                pEU->NormalizedEvaluate(dParam,FALSE,sPnt,&sDir);  // TRUE = UV Eval, FALSE = 3d Eval
                SmExtent3d sVBBox(sPnt);
                sVBBox.ExpandAbsolute(m_dStitchTol3d);
                SER(pEdgeTree->GetObjectsInBox(sVBBox,sObjects));
                for(ULONG lll=0; lll<sObjects.GetSize(); lll++) {
                    SmEdge *pEdge = SM_CAST_PTR(SmEdge,sObjects[lll]);
                    if (!pEdge->IsLamina()) continue;
                    SmEdgeuse *pPrimEU = pEdge->GetPrimaryEdgeuse();
                    // If already processed this face continue
                    if (pPrimEU->GetFace()->IsMarked(eMarkType)) continue;
                    SmExtent1d sIvl = pEdge->GetInterval();
                    SmBoolean bSuccess;
                    double dParam2 = 0.0, dDist = 0.0;
                    SER(pEdge->GetCurve()->DropPoint(sIvl,               // in : target curve allowed domain
                                                     sPnt,               // in : Point to drop to curve
                                                     NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                         //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                         //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                     m_dStitchTol3d, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                         //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                         //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                         //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                     NULL,               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                     bSuccess,           // out: TRUE = found a drop point
                                                     dParam2,            // out: found drop curve param
                                                     dDist)) ;           // out: found drop distance
                                                                         // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                         //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                         //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                         //      default:[SM_SO_MINIMIZE] to preserve original behavior
                    if (bSuccess) {
                        SmPoint3d sPV[2];
                        double dNormParam = 0.0;
                        sIvl.Inversion(dParam2,dNormParam);
                        if (dNormParam < 0.02 || dNormParam > 0.98) continue;
                        SER(pEdge->GetCurve()->Evaluate(dParam2,1,TRUE,sPV));
                        //SmBoolean bSameOrient = TRUE;
                        double dAngleRad = 0.0;
                        sPV[1].AngleBetween(sDir,dAngleRad);
                        double dAngleDeg = dAngleRad * 180.0 / SM_PI;
                        // Make sure we are going in either same or opposite direction
                        if (dAngleDeg > 5 && dAngleDeg < 175) continue;
                        if (sPV[1].Dot(sDir) > 0.0) {
                            pPrimEU = pPrimEU->GetMate();
                        }
                        // Here we should have opposite oriented edgeuses
                        // pPrimEU's faceuse should be on side of positive normal
                        // Let's limit the unification to cases where the faces have
                        // a nice angle between them.
                        SmPoint3d sPt;
                        SmVector3d sBinVec, sBinOrig;
                        SER(pPrimEU->EvaluateBinormal(dParam2,FALSE,sPt,sBinVec));
                        SER(pEU->EvaluateBinormal(sOrigIvl.Evaluate(dParam),FALSE,sPt,sBinOrig));

                        // Only stitch cases where angle between binormals is > 90 degrees
                        SmFace *pFOther = pPrimEU->GetFaceuse()->GetFace();
                        if (pPrimEU->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
                            rSameOrientation.Add(FALSE);
                        }
                        else {
                            rSameOrientation.Add(TRUE);
                        }
                        pFOther->Mark(eMarkType);
                        rRingFaces.Add(pFOther);
                    }
                }
            }
        }
    }
    
    return SM_SUCCESS;

} // end SmStitch::SelectRing

/*******************************************************************//**
PURPOSE: Orient Faces of UnStitched Model consistently relative to a starting
     face.

NOTES: increments unlocked mark value 
***********************************************************************/
SmStatus SmStitch::UnifyNormals
 (SmBrep            * pBrepToOrient,
  SmFace            * pStartFace,
  ULONG               lNumSamples,
  SmTArray<SmFace*> & rFlippedFaces)
{
    // increment and lock an unlocked mark
    SmNewMarkAndLock sMarkLock( pBrepToOrient->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
    SmMarkType eMarkType = sMarkLock.GetMarkType() ;

    SmExtent3d sBrepBBox;
    pBrepToOrient->CalculateBoundingBox(sBrepBBox);

    SM_PTR_ARRAY(sFStack, SmFace, 256);  // SmTArray<SmFace *>
    SM_PTR_ARRAY(sEdges,  SmEdge, 256);  // SmTArray<SmEdge *>
        
    pBrepToOrient->GetEdges(sEdges);
    ULONG lNumPerNode = smos_Max(500,sEdges.GetSize()/10);
    SmTree *pEdgeTree = new (*pBrepToOrient->GetContext()) SmTree(sBrepBBox,lNumPerNode,lNumPerNode/10);
    NER(pEdgeTree);
    SmObjDelete sCleanTree(pEdgeTree);
        
    for(ULONG iii=0; iii<sEdges.GetSize(); iii++) {
        SmEdge *pEdge = sEdges[iii];
        SmExtent3d sEBBox;
        SmCurve *pCurve = pEdge->GetCurve();
        SER(pCurve->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox));
        SER(pEdgeTree->AddToSpatialTree(sEBBox,pEdge));
    }
        
    sFStack.Add(pStartFace);
    pStartFace->Mark(eMarkType);

    SmEdgeuse *sEUData[256];
    SmTArray<SmEdgeuse*> sEdgeuses(256,sEUData);

    SmObject *sOData[256];
    SmTArray<SmObject*> sObjects(256,sOData);

    while (sFStack.GetSize() > 0) {
        SmFace *pF = sFStack.GetLast();
        sFStack.RemoveLast();

        SmFaceuse *pFU = pF->GetUpwardFaceuse();
        pFU->GetEdgeuses(sEdgeuses);
        for(ULONG i=0; i<sEdgeuses.GetSize(); i++) {
            SmEdgeuse *pEU = sEdgeuses[i];
            if (pEU->GetEdge()->IsManifold()) {
                // we are already stitched just orient it the easy way.
                SmEdgeuse *pRadialEU = pEU->GetRadial();
                SmFaceuse *pFUse = pRadialEU->GetFaceuse();
                SmFace *pRadF = pFUse->GetFace();
                if (pRadF->IsMarked(eMarkType)) continue;
                if (pFUse->GetOrientation() == SM_OT_OPPOSITE) {
                    rFlippedFaces.Add(pRadF);
                    SER(pRadF->SwapUV());
                }
                // Only oriented faces get onto stack and get marked
                pRadF->Mark(eMarkType);
                sFStack.Add(pRadF);
                continue;
            }
            for(ULONG j=0; j<lNumSamples; j++) {
                double dParam = (j+1.0) / (lNumSamples + 2.0);
                SmPoint3d sPnt, sDir;
                SmExtent1d sOrigIvl = pEU->GetEdge()->GetInterval();
                pEU->NormalizedEvaluate(dParam,FALSE,sPnt,&sDir);  // TRUE = UV Eval, FALSE = 3d Eval
                SmExtent3d sVBBox(sPnt);
                sVBBox.ExpandAbsolute(m_dStitchTol3d);
                SER(pEdgeTree->GetObjectsInBox(sVBBox,sObjects));
                for(ULONG lll=0; lll<sObjects.GetSize(); lll++) {
                    SmEdge *pEdge = SM_CAST_PTR(SmEdge,sObjects[lll]);
                    if (!pEdge->IsLamina()) continue;
                    SmEdgeuse *pPrimEU = pEdge->GetPrimaryEdgeuse();
                    // If already processed this face continue
                    if (pPrimEU->GetFace()->IsMarked(eMarkType)) continue;
                    SmExtent1d sIvl = pEdge->GetInterval();
                    SmBoolean bSuccess;
                    double dParam2 = 0.0, dDist = 0.0;
                    SER(pEdge->GetCurve()->DropPoint(sIvl,                // in : target curve allowed domain
                                                     sPnt,                // in : Point to drop to curve
                                                     NULL,                // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                          //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                          //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                                     m_dStitchTol3d,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                          //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                          //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                          //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                                     NULL,                // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                                     bSuccess,            // out: TRUE = found a drop point
                                                     dParam2,             // out: found drop curve param
                                                     dDist)) ;            // out: found drop distance
                                                                          // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                          //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                          //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                          //      default:[SM_SO_MINIMIZE] to preserve original behavior
                    if (bSuccess) {
                        SmPoint3d sPV[2];
                        double dNormParam = 0.0;
                        sIvl.Inversion(dParam2,dNormParam);
                        if (dNormParam < 0.02 || dNormParam > 0.98) continue;
                        SER(pEdge->GetCurve()->Evaluate(dParam2,1,TRUE,sPV));
                        //SmBoolean bSameOrient = TRUE;
                        double dAngleRad = 0.0;
                        sPV[1].AngleBetween(sDir,dAngleRad);
                        double dAngleDeg = dAngleRad * 180.0 / SM_PI;
                        // Make sure we are going in either same or opposite direction
                        if (dAngleDeg > 5 && dAngleDeg < 175) continue;
                        if (sPV[1].Dot(sDir) > 0.0) {
                            pPrimEU = pPrimEU->GetMate();
                        }
                        // Here we should have opposite oriented edgeuses
                        // pPrimEU's faceuse should be on side of positive normal
                        // Let's limit the unification to cases where the faces have
                        // a nice angle between them.
                        SmPoint3d sPt;
                        SmVector3d sBinVec, sBinOrig;
                        SER(pPrimEU->EvaluateBinormal(dParam2,FALSE,sPt,sBinVec));
                        SER(pEU->EvaluateBinormal(sOrigIvl.Evaluate(dParam),FALSE,sPt,sBinOrig));

                        // Only stitch cases where angle between binormals is > 90 degrees
                        if (sBinVec.Dot(sBinOrig) > 0.0) continue;
                        SmFace *pFOther = pPrimEU->GetFaceuse()->GetFace();
                        if (pPrimEU->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) {
                            rFlippedFaces.Add(pFOther);
                            SER(pFOther->SwapUV());
                        }
                        pFOther->Mark(eMarkType);
                        sFStack.Add(pFOther);

                    } // end if DropPoint succeeded
                } // end for each object in box
            } // end for each sample point
        } // end for each Edgeuse
    } // end while stack is not empty
    
    return SM_SUCCESS;
} // end SmStitch::UnifyNormals

/*******************************************************************//**
PURPOSE: Stitch this Brep together expecting a manifold solid to result. 
    Manifold solids will have only two faces sharing a given edge and will
    not have separate bodies touching at a vertex or along an edge.

NOTES: Edges with gaps of more than 1/200th of the model size
    will not be stitched.
***********************************************************************/
SmStatus SmStitch::StitchIntoSolid
  (SmBrep    * pBrepToStitch,     // in :
   SmBoolean & rbProducesASolid,  // out:
   ULONG     & rlStitchedEdges,   // out:
   double    & rdMaxVertexGap,    // out:
   double    & rdMaxEdgeGap)      // out:
{
  rbProducesASolid = FALSE;
  rlStitchedEdges = 0;
  rdMaxVertexGap = 0.0;
  rdMaxEdgeGap = 0.0;

  SmExtent3d sBBox;
  SER(pBrepToStitch->CalculateBoundingBox(sBBox, TRUE)); // TRUE: Approx is fine [B259]
  SmVector3d sSize = sBBox.GetSize();
  double dDist = sSize.Length();
  // This tol will be iteratively increased:
  double dStitchTol3d = pBrepToStitch->GetTolerance() / 10.0;

  // The following code is just to make life easier when doing graphics.
//    SmVector3d sVecToCenter = - sBBox.Evaluate(0.5,0.5,0.5);
//    SmAxis2Placement sMoveToOrigin;
//    sMoveToOrigin.Translate(sVecToCenter);
//    pBrepToStitch->Transform(sMoveToOrigin);

  // First try stitching down to 1/10000 of the size of the part.
  // This should get most of the simple stuff stitched together.
  ULONG lNumStitched;
  ULONG lNumLamina = 0;
  while (dStitchTol3d < dDist / 200.0) {
      SmStitchCallback sStitchCallback;
      SmStitch sStitch(sStitchCallback,dStitchTol3d,TRUE,FALSE);
#ifdef FIX_ME
#endif
      sStitch.m_bFastEdgeCompare = TRUE;
      sStitch.m_bSplitEdgesWithVertices = FALSE;
    
      sStitch.m_bMakingManifoldSolid = TRUE;
//        sStitch.m_bSqueezeSmallEdges = TRUE;
      double dMaxVGap = 0.0, dMaxEGap = 0.0;
      SER(sStitch.DoStitching(pBrepToStitch,NULL,NULL,lNumStitched,lNumLamina,dMaxVGap,dMaxEGap));
      if (dMaxVGap > rdMaxVertexGap) {
          rdMaxVertexGap = dMaxVGap;
      }
      if (dMaxEGap > rdMaxEdgeGap) {
          rdMaxEdgeGap = dMaxEGap;
      }
      rlStitchedEdges += lNumStitched;
      if (lNumLamina == 0) {
          break;
      }
      dStitchTol3d = 2.0 * dStitchTol3d;
  }

  if (lNumLamina == 0) {
      SmTArray<SmRegion*> sRegions;
      pBrepToStitch->GetRegions(sRegions);
      if (sRegions.GetSize() > 1) {
          rbProducesASolid = TRUE;
      }
  }

  return SM_SUCCESS;

} // end SmStitch::StitchIntoSolid

/*******************************************************************//**
PURPOSE: Stitch this Brep together expecting a shell to result. 

NOTES: 
   If you set bShellIsWellFormed to FALSE it will basically
   glue together edges and vertices and ignore other things
   and could result in errors such as duplicate faces, if the
   Brep's topology is not well formed.
***********************************************************************/
SmStatus SmStitch::StitchIntoShell
  (SmBrep    * pBrepToStitch,         // in : target Brep
   SmBoolean   bShellIsWellFormed, // in : Only pass TRUE if you know that
                                   //      the shell is of very good quality,
                                   //      e.g., produced from a solid modeler.
                                   //      Surface based models usually need this
                                   //      to be FALSE
   double      dMaxStitchTol3d,      // in : Max allowable separation between topology
                                   //      objects that will get stitched here.
   ULONG     & rlStitchedEdges,        // out:
   double    & rdMaxVertexGap,     // out:
   double    & rdMaxEdgeGap)       // out:
{
  // init output
  rlStitchedEdges    = 0;
  rdMaxVertexGap = 0.0;
  rdMaxEdgeGap   = 0.0;

  // First stitch with tighter tolerances.  This helps to ensure that
  // the proper things get stitched.  Note that DoStitching() also uses the
  // tolerances associated with the topology objects being stitched.
  // Starting with 1/100 and doubling each time will mean seven iterations.
  double dStitchTol3d = dMaxStitchTol3d / 100.0;

  SmStitchCallback sStitchCallback;
  SmStitch         sStitch( sStitchCallback, dStitchTol3d, TRUE, FALSE );
  sStitch.m_bDoRegionNesting = FALSE;
  sStitch.m_bFastEdgeCompare = TRUE;
  if ( bShellIsWellFormed )
    { sStitch.m_bIgnoreProblems = FALSE; }
  else
    { sStitch.m_bIgnoreProblems = TRUE; }

  sStitch.m_bValidateResult    = FALSE;
  sStitch.m_bSqueezeSmallEdges = TRUE;
  double dMaxVGap = 0.0, dMaxEGap = 0.0;

  ULONG lNumStitched;
  ULONG lNumLamina = 0;

  while ( dStitchTol3d < dMaxStitchTol3d )
    {
      if ( dStitchTol3d > dMaxStitchTol3d / 2.0 )
        { dStitchTol3d = dMaxStitchTol3d; }

      SER(sStitch.DoStitching( pBrepToStitch, NULL, NULL, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap ));

      if ( dMaxVGap > rdMaxVertexGap ) { rdMaxVertexGap = dMaxVGap; }
      if ( dMaxEGap > rdMaxEdgeGap   ) { rdMaxEdgeGap   = dMaxEGap; }
      rlStitchedEdges += lNumStitched;

      if (lNumLamina == 0)
        { break; }

      dStitchTol3d = 2.0 * dStitchTol3d;
      sStitch.SetStitchTol3d( dStitchTol3d );

    } // end while dStitchTol3d < dMaxStitchTol3d

  return SM_SUCCESS;

} // end SmStitch::StitchIntoShell



/*********************************************************************
//
// The following three methods are intended for internal use.
//
***********************************************************************/

/*******************************************************************//**
PURPOSE: Given a list of vertices in a Brep,
   glue any pairs that are within m_dStitchTol3d tolerance.

NOTES:  
  All vertices must be part of a single SmBrep.

  Returns a list of the processed vertices -- the surviving one of each
  glued pair, and the original list with the processed vertices removed.

  This routine does some 'boxing' by projecting along a line.

INPUTS ---
  rVertexList     Vertices to possibly glue.

OUTPUTS ---
  rVertexList     Has all glued verticess set to NULL
  rProcessedVerts Contains all the glued verts that were set to Null in rVertexList.
                  Note: added to, not reset.
  Side effect:    Coincident verticess are glued, in the Brep.

***********************************************************************/
SmStatus SmStitch::StitchVertexPairs
(
    SmTArray< SmVertex* > & rVertexList,     // i/o: kept vertices to pair and glue - returned with glued vertices set to NULL
    SmTArray< SmVertex* > & rSurvivingVerts, // out: added to, not reset.
    SmTArray< SmVertex* > & rDeletedVerts,   // out: List of deleted vertices (stale pointers)
    SmTArray< SmEdge*> & rDeletedEdges,      // out: Edges that are squeezed out (list of stale pointers)
    double    & rdMaxVertexGap,              // out: largest gap seen between paired vertices
    double    & rdMinUnstitchedVertGap         // out: smallets gap seen between unpaired vertices
)
{
  // Init outputs.
  rdMaxVertexGap = 0;
  rdMinUnstitchedVertGap = SM_BIG_DOUBLE;
  rSurvivingVerts.ReSet();

  ULONG lNumVerts = rVertexList.GetSize();
  if ( lNumVerts < 1 ) { return SM_SUCCESS; }

  // Get Brep pointer from the first non-null entry.
  SmBrep *pOwningBrep = NULL;
  for ( ULONG ii = 0; ii < lNumVerts; ii++ )
  {
      if ( rVertexList[ii] != NULL )
      {
          pOwningBrep = rVertexList[ii]->GetBrep();
          break;
      }
  }
  if ( pOwningBrep == NULL ) { SER(SM_ERR); }  // we'll flag this as an error.

  // As in DoStitching(), sort vertices in one dimension, which speeds things up considerably.
  // Sort on (1,2,3), which should separate things better than (1,1,1).
  // And unitize the vector, so that parameter differences correspond to 3d distances.
  // Make separate lists for Verts1 and Verts2.

  SmPoint3d sPnt( 0,0,0 );
  SmVector3d sVec( 1,2,3 );
  sVec.Unitize();

  SmTArray< SmVertex* > sSortedVerts;
  SmTArray< double    > sSortParameter;
  SmTArray< SmBoolean > sIsLaminaVertex;

  // Precompute lamina flag and sort value.

  // Do VertexList1 first.
  // Also keep track of min and max param values, for "boxing out"
  // vertices in VertexList2.
  double dMinParam =  SM_BIG_DOUBLE;
  double dMaxParam = -SM_BIG_DOUBLE;
  double dTest;

  for( ULONG ii=0; ii<lNumVerts; ii++ )
  {
      SmVertex *pV = rVertexList[ii];

      if ( pV == NULL )
        {
          sIsLaminaVertex.Add( FALSE );
          sSortParameter.Add( 0.0 );
          continue;
        }
      sIsLaminaVertex.Add( pV->IsLaminaVertex() );

      // get and save parameter of projecting vertex to the line
      // Note, since sPnt is the origin, that call is just this dot product:
      // SER( smgu_LineClosestPoint( sPnt, sVec, pV->GetPoint(), dTest ));
      dTest = sVec.Dot( pV->GetPoint() );

      sSortParameter.Add( dTest );

      if ( dTest < dMinParam ) { dMinParam = dTest; }
      if ( dTest > dMaxParam ) { dMaxParam = dTest; }

  } // end iter every vertex - getting sort parameter and lamina labels

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
      rVertexList.Dump() ;
      sIsLaminaVertex.Dump() ;
      sSortParameter.Dump() ;
  }
#endif

  // For sorting VertexList2, we can "box out" those that are
  // outside the range of VertexList1's parameters.
  // Expand the range a bit.
  dMinParam -= 2.0 * m_dStitchTol3d;
  dMaxParam += 2.0 * m_dStitchTol3d;

  // Now sort the lists by projection param.
  SmTArray<ULONG> sSortedIndex;

  for ( ULONG ii=0; ii<lNumVerts; ii++ )
    {
      if ( rVertexList[ii] == NULL )
        { continue; }

      // // skip nonLamina edges when making manifold solids
      // if ( m_bMakingManifoldSolid && ! sIsLaminaVertex1[ii] )
      //   { continue; }

      dTest = sSortParameter[ii];

      // init sSortedIndex with 1st entry
      if (sSortedIndex.GetSize() == 0)
        {
          sSortedIndex.Add(ii);
          continue;
        }

      // init for binary sort to find index for this entry in sorted list
      ULONG lMin = 0;
      ULONG lMax = sSortedIndex.GetSize() - 1;

      // add entry to end to list when its Param is largest seen
      if ( dTest >= sSortParameter[ sSortedIndex.GetLast() ] )
        {
          sSortedIndex.Add(ii);
        }

      // add entry to begin of list when its param is smallest seen
      else if ( dTest <= sSortParameter[sSortedIndex[0]] )
        {
          sSortedIndex.InsertAt(0,ii);
        }

      // use binary search to find index for current entry
      else
        {
          // Binary search.
          while (lMin != lMax-1 && lMin != lMax)
            {
              ULONG lMid = (lMin+lMax) / 2;
              if ( dTest > sSortParameter[ sSortedIndex[lMid] ] )
                { lMin = lMid; }
              else
                { lMax = lMid; }
            }
          sSortedIndex.InsertAt(lMax,ii);
        }
    } // end iter sorting every vertex

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sSortedIndex.Dump();
    }
#endif // SM_DEBUG_CODE

  // Now both lists are sorted by projection on the line.
  // Now we march down the lists in order:
  // for each entry in List1,
  //   Step along list2 collecting vertices within tol,
  //     until the list2 parameter is past the list1 parameter.
  //   If more than one is found, pick the closest.
  //     Also check for closest List1 vertex to this List2 vertex.
  //   Glue.

  SmTArray<SmVertex*> sCoinVertices;
  SmTArray<SmEdge*>   sCommonEdges;
      // arrive here after the following arrays have been set
      // 1. sVertices      = list of all target vertices to stitch
      // 2. sSortParameter = associated projection param to the 1,1,1 line
      // 3. sSortedIndex   = ordered list of rVertexList indices based on the sSortParameter list

      // for every vertex - build list of coincident vertices into sCoinVertices,
      //                  - then stitch (glue or squeeze) coincident vertices 
  for ( ULONG ii = 0; ii < sSortedIndex.GetSize(); ii++ )
  {
      SmVertex *pV = rVertexList[sSortedIndex[ii]];

      // skip vertices already processed
      if ( pV == NULL ) { continue; }

#ifdef SM_DEBUG_CODE
      if ( bDebugMe )
      {
          smgfx_SetLook( 2, 5, 1, 0, 0 ); pV->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      double dVParam = sSortParameter[sSortedIndex[ii]];

      // load sCoinVertices with all vertices coincident with pV
      sCoinVertices.ReSet();
      for ( ULONG jj = ii + 1; jj < sSortedIndex.GetSize(); jj++ )
      {
          SmVertex *pTestV = rVertexList[sSortedIndex[jj]];
          double    dTVParam = sSortParameter[sSortedIndex[jj]];

          // skip vertices already tested
          if ( pTestV == NULL ) continue;

          double dTestTol = smos_Max( m_dStitchTol3d,
              (double) ( pTestV->GetTolerance() + pV->GetTolerance() ) );

          // done looking when pTestV is too far along the vector
          // Since dParam's are projected to (1,2,3), need to be 3x tol to ensure no coincidence
          if ( dTVParam - dVParam > dTestTol * 3.0 )
          { break; }

          // get 3d distance between vertex pairs and tolerance
          double dDist = pV->GetPoint().DistanceBetween( pTestV->GetPoint() );

          // when vertices are within tolerance
          if ( dDist < dTestTol )
          {
              // add vertex to the coincident list - save biggest coincident vertex gap value
              sCoinVertices.Add( pTestV );
              rVertexList[sSortedIndex[jj]] = NULL;
              if ( dDist > rdMaxVertexGap ) { rdMaxVertexGap = dDist; }
          }
          else if ( dDist < rdMinUnstitchedVertGap )
          {
              rdMinUnstitchedVertGap = dDist;
          }
      } // end iter sorted vertices until no more coincidences are found

    // during iter ii on rVertexList array, arrive here after
    // 1. all vertices coincident to rVertexList[ii] are in list sCoinVertices

#ifdef SM_DEBUG_CODE // list all vertices coincident with rVertexList[sSortedIndex[ii]]
      if ( bDebugMe )
      { sCoinVertices.Dump(); }
#endif // SM_DEBUG_CODE

      ULONG lNumCoin = sCoinVertices.GetSize();

      // Now glue (or squeeze) every vertex that is coincident with pV.
      for ( ULONG jj = 0; jj < lNumCoin; jj++ )
      {
          SmBoolean bKeepFirst;

          if ( pV == sCoinVertices[jj] )
          {
              rVertexList[sSortedIndex[ii]] = NULL;
              continue;
          }

          // when     m_bMakingManifoldSolid - only glue pairs of Lamina vertices or wire vertices
          // when not m_bMakingManifoldSolid - glue all pairs of coincident vertices
          if ( !m_bMakingManifoldSolid
               || ( ( pV->IsLaminaVertex() && sCoinVertices[jj]->IsLaminaVertex() )
                    || ( pV->IsWireVertex() || sCoinVertices[jj]->IsWireVertex() ) ) )

          {
              //// skip vertices farther apart than the shortest squeezed edge
              //// NOTE: if m_bSqueezeSmallEdges is False, dMinEdgeLength is SM_BIG_DOUBLE.
              //double dDistToV = pV->GetPoint().DistanceBetween(sCoinVertices[jj]->GetPoint());
              //if (dDistToV > dMinEdgeLength) 
              //  {
              //    if ( dDistToV < rdMinUnstitchedVertGap )
              //      { rdMinUnstitchedVertGap = dDistToV; }

              //    continue;
              //  }

              // select the vertex to keep
              double dMaxDistance;
              double dDistToV = pV->GetPoint().DistanceBetween( sCoinVertices[jj]->GetPoint() );
              SER( m_rStitchCallback.SelectTopologyToKeep( pV, sCoinVertices[jj], dDistToV, dMaxDistance, bKeepFirst ) );

#ifdef SM_DEBUG_CODE
              if ( bDebugMe )
              {
                  smgfx_Erase();
                  smgfx_SetLook( 1, 3 ); pOwningBrep->Draw( TRUE ); sm_GraphicsLoop();
                  smgfx_SetLook( 1, 5, 1, 0, 0 ); pV->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook( 3, 7, 0, 0, 1 ); sCoinVertices[jj]->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE
              // skip vertices farther apart than allowed stitching tolerance + vertex tolerances
              double dTol = smos_Max( m_dStitchTol3d,
                  (double) ( pV->GetTolerance() + sCoinVertices[jj]->GetTolerance() ) );
              if ( dMaxDistance > dTol )
              {
                  if ( dDistToV < rdMinUnstitchedVertGap )
                  { rdMinUnstitchedVertGap = dDistToV; }

                  continue; // Don't stitch it if distance gets too big.
              }

              // squeeze/glue state
              // glue   : combine two coincident vertices not connected with a common edge
              // squeeze: combine two coincident vertices and delete edge when 
              //              vertices are connected by a common short edge.
              ULONG   bDoGlue = TRUE;
              ULONG   bDoSqueeze = FALSE;
              SmEdge *pSqueezeEdge = NULL;

              // don't glue when vertices have common edges 
              // - and squeeze only if common vertices are connected by one short edge

              // for every common edge between the vertices
              pV->GetCommonEdges( sCoinVertices[jj], sCommonEdges );
              for ( ULONG kk = 0; kk < sCommonEdges.GetSize(); kk++ )
              {
                  // get edge->curve approximate length
                  SmEdge *pE = sCommonEdges[kk];
                  double  dEdgeLength = pE->GetCurve()->ApproximateLength( pE->GetInterval(), 10 );

                  // if edge is short
                  if ( dEdgeLength < 2.0 * dDistToV )
                  {
                      // bDoSqueeze is False the first time around,
                      // so it it's True, there must have been another
                      // short edge connecting the two vertices.
                      // Don't squeeze or glue a vertex pair when it has
                      // more than one short common edge.
                      if ( bDoSqueeze )
                      {
                          bDoSqueeze = FALSE;
                          bDoGlue = FALSE;
                          break;
                      }

                      // squeeze lamina and wire edges - don't squeeze manifold edges
                      if ( pE->IsLamina()
                           || pE->IsWire() )
                      {
                          bDoSqueeze = TRUE;
                          bDoGlue = FALSE;
                          pSqueezeEdge = pE;
                      }
                      else
                      {
                          bDoSqueeze = FALSE;
                          bDoGlue = FALSE;
                      }

                  } // end edge is short check
              } // end iter kk every edge connecting these two vertices

            // skip squeezing vertices when m_bSqueezeSmallEdges is set to FALSE
              if ( bDoSqueeze && m_bSqueezeSmallEdges == FALSE )
              {
                  continue;
              }

              // squeeze or glue vertices together (make pV the retained vertex)
              if ( bKeepFirst )
              {
                  if ( bDoSqueeze )
                  { SER( pOwningBrep->SqueezeEdge( pSqueezeEdge, pV ) ); }
                  else if ( bDoGlue )
                  { SER( pOwningBrep->GlueVertices( pV, sCoinVertices[jj], NULL ) ); }
                  rDeletedVerts.Add( sCoinVertices[jj] );
              } // end keep first vertex branch
              else // keep second vertex branch
              {
                  if ( bDoSqueeze )
                  { SER( pOwningBrep->SqueezeEdge( pSqueezeEdge, sCoinVertices[jj] ) ); }
                  else if ( bDoGlue )
                  { SER( pOwningBrep->GlueVertices( sCoinVertices[jj], pV, NULL ) ); }
                  rDeletedVerts.Add( pV );

                  pV = sCoinVertices[jj];

              } // end keep second vertex branch

              // update output
              if ( pSqueezeEdge ) rDeletedEdges.Add( pSqueezeEdge );
              rVertexList[sSortedIndex[ii]] = NULL;
#ifdef SM_DEBUG_CODE
              if ( bDebugMe )  // draw faces, face->edges, face->vertices in various orders to allow
              {            //  an inspection of what may lie on top of what
                  SM_DUMP_AND_ASSERT_VALID( pOwningBrep );
              }
#endif // SM_DEBUG_CODE
          } // end skip nonLamina vertices when making manifold solids check 
      } // end iter jj every coin vertex gluing (or squeezing) every coincident vertex 

      rSurvivingVerts.Add( pV );
  } // end iter ii every sorted vertex index

  // arrive here after squeezing/gluing all coincident vertex sets

  return SM_SUCCESS;

} // end SmStitch::StitchVertexPairs
/*******************************************************************//**
PURPOSE: Given two lists of vertices in a Brep,
   Glue any pairs (one vertex in each list) that are within 
   m_dStitchTol3d tolerance.

NOTES:  
  All vertices must be part of a single SmBrep.

  Returns a list of the processed vertices -- the surviving one of each
  glued pair, and the original lists with the processed vertices removed.

  This routine does some 'boxing' by projecting along a line.
  Any vertices in rVertexList2 that are not within the range of those
  in rVertexList1 will be removed from rVertexList2 -- not just set
  to Null, rVertexList2 will be smaller.
  If there is no overlap at all, then rVertexList2 will be returned empty.

INPUTS ---
  rVertexList1   If glued, these vertices will be kept.
  rVertexList2   If glued, these vertices will be deleted.

OUTPUTS ---
  rVertexList1    Has all glued verticess set to NULL
  rVertexList2    All glued verts set to NULL -- and they've been deleted.
                  rVertexList2 may be returned smaller, or even empty.
  rProcessedVerts Contains all the glued verts that were set to Null in rVertexList1.
                  Note: added to, not reset.
  Side effect:    Coincident verticess are glued, in the Brep.

***********************************************************************/
SmStatus SmStitch::StitchVertexPairs(
        SmTArray< SmVertex* > & rVertexList1,    // i/o: kept vertices to pair and glue - returned with glued vertices set to NULL
        SmTArray< SmVertex* > & rVertexList2,    // i/o: deleted vertices to pair and glue - returned with glued vertices set to NULL
        SmTArray< SmVertex* > & rProcessedVerts, // out: added to, not reset.
        SmTArray< SmVertex* > & rDeletedVerts,   // out: pointers to deleted vertices (Stale!)
        double    & rdMaxVertexGap,              // out: largest gap seen between paired vertices
        double    & rdMinUnstitchedVertGap         // out: smallets gap seen between unpaired vertices
    )
{
  // Init outputs.
  rdMaxVertexGap = 0;
  rdMinUnstitchedVertGap = SM_BIG_DOUBLE;
  rProcessedVerts.ReSet();
  rDeletedVerts.ReSet();

  ULONG lNumVerts1 = rVertexList1.GetSize();
  if ( lNumVerts1 < 1 ) { return SM_SUCCESS; }

  ULONG lNumVerts2 = rVertexList2.GetSize();
  if ( lNumVerts2 < 1 ) { return SM_SUCCESS; }

  // Get Brep pointer from the first non-null entry.
  SmBrep *pOwningBrep = NULL;
  for ( ULONG ii = 0; ii < lNumVerts1; ii++ )
  {
      if ( rVertexList1[ii] != NULL )
      {
          pOwningBrep = rVertexList1[ii]->GetBrep();
          break;
      }
  }
  if ( pOwningBrep == NULL ) { return SM_ERR; }  // we'll flag this as an error.

  // As in DoStitching(), sort vertices in one dimension, which speeds things up considerably.
  // Sort on (1,2,3), which should separate things better than (1,1,1).
  // And unitize the vector, so that parameter differences correspond to 3d distances.
  // Make separate lists for Verts1 and Verts2.

  SmPoint3d sPnt( 0,0,0 );
  SmVector3d sVec( 1,2,3 );
  sVec.Unitize();

  SmTArray< SmVertex* > sSortedVerts1,    sSortedVerts2;
  SmTArray< double    > sSortParameter1,  sSortParameter2;
  SmTArray< SmBoolean > sIsLaminaVertex1, sIsLaminaVertex2;

  // Precompute lamina flag and sort value.

  // Do VertexList1 first.
  // Also keep track of min and max param values, for "boxing out"
  // vertices in VertexList2.
  double dMinParam =  SM_BIG_DOUBLE;
  double dMaxParam = -SM_BIG_DOUBLE;
  double dTest;

  for( ULONG ii=0; ii<lNumVerts1; ii++ )
  {
      SmVertex *pV = rVertexList1[ii];

      if ( pV == NULL )
        {
          sIsLaminaVertex1.Add( FALSE );
          sSortParameter1.Add( 0.0 );
          continue;
        }
      sIsLaminaVertex1.Add( pV->IsLaminaVertex() );

      // get and save parameter of projecting vertex to the line
      // Note, since sPnt is the origin, that call is just this dot product:
      // SER( smgu_LineClosestPoint( sPnt, sVec, pV->GetPoint(), dTest ));
      dTest = sVec.Dot( pV->GetPoint() );

      sSortParameter1.Add( dTest );

      if ( dTest < dMinParam ) { dMinParam = dTest; }
      if ( dTest > dMaxParam ) { dMaxParam = dTest; }

  } // end iter every vertex 1 - getting sort parameter and lamina labels

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
      rVertexList1.Dump() ;
      sIsLaminaVertex1.Dump() ;
      sSortParameter1.Dump() ;
  }
#endif

  // For sorting VertexList2, we can "box out" those that are
  // outside the range of VertexList1's parameters.
  // Expand the range a bit.
  dMinParam -= 2.0 * m_dStitchTol3d;
  dMaxParam += 2.0 * m_dStitchTol3d;

  // Also keep track of which vertices from List2 we're using.
  SmTArray< SmVertex* > sUsingVerts2;
  SmTArray< ULONG     > sUsingIndices2;

  for( ULONG ii=0; ii<lNumVerts2; ii++ )
  {
      SmVertex *pV = rVertexList2[ii];

      if ( pV == NULL )
        {
          sUsingVerts2.Add( NULL );
          sUsingIndices2.Add( 0 );
          sIsLaminaVertex2.Add( FALSE );
          sSortParameter2.Add( 0.0 );
          continue;
        }

      // get and save parameter of projecting vertex to the line
      // (Dot product, as above.)
      dTest = sVec.Dot( pV->GetPoint() );

      // skip vertices that can't possibly pair up
      if ( dTest < dMinParam ) { continue; }
      if ( dTest > dMaxParam ) { continue; }

      // add potentially pairing vertex to check list
      sUsingVerts2.Add( pV );
      sUsingIndices2.Add( ii );

      sSortParameter2.Add( dTest );

      sIsLaminaVertex2.Add( pV->IsLaminaVertex() );

  } // end iter every vertex 2 - getting sort parameter and lamina labels

  ULONG lNumUsing = sUsingVerts2.GetSize();
  if ( lNumUsing != lNumVerts2 )
  {
      // rVertexList2 = sUsingVerts2;  No: the caller is still using the other ones.
      lNumVerts2   = lNumUsing;
  }

  // no work - no possible pairing candidates
  if ( lNumUsing < 1 )
    { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
  {
      rVertexList2.Dump() ;
      sUsingVerts2.Dump() ;
      sIsLaminaVertex2.Dump() ;
      sSortParameter2.Dump() ;
  }
#endif


  // Now sort the lists by projection param.
  SmTArray<ULONG> sSortedIndex1, sSortedIndex2;

  for ( ULONG ii=0; ii<lNumVerts1; ii++ )
    {
      if ( rVertexList1[ii] == NULL )
        { continue; }

      // // skip nonLamina edges when making manifold solids
      // if ( m_bMakingManifoldSolid && ! sIsLaminaVertex1[ii] )
      //   { continue; }

      dTest = sSortParameter1[ii];

      // init sSortedIndex with 1st entry
      if (sSortedIndex1.GetSize() == 0)
        {
          sSortedIndex1.Add(ii);
          continue;
        }

      // init for binary sort to find index for this entry in sorted list
      ULONG lMin = 0;
      ULONG lMax = sSortedIndex1.GetSize() - 1;

      // add entry to end to list when its Param is largest seen
      if ( dTest >= sSortParameter1[ sSortedIndex1.GetLast() ] )
        {
          sSortedIndex1.Add(ii);
        }

      // add entry to begin of list when its param is smallest seen
      else if ( dTest <= sSortParameter1[sSortedIndex1[0]] )
        {
          sSortedIndex1.InsertAt(0,ii);
        }

      // use binary search to find index for current entry
      else
        {
          // Binary search.
          while (lMin != lMax-1 && lMin != lMax)
            {
              ULONG lMid = (lMin+lMax) / 2;
              if ( dTest > sSortParameter1[ sSortedIndex1[lMid] ] )
                { lMin = lMid; }
              else
                { lMax = lMid; }
            }
          sSortedIndex1.InsertAt(lMax,ii);
        }
    } // end iter sorting every vertex1

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sSortedIndex1.Dump();
    }
#endif // SM_DEBUG_CODE


  // Same for List2.
  for ( ULONG ii=0; ii<lNumVerts2; ii++ )
    {
      if ( rVertexList2[ii] == NULL )
        { continue; }

      // // skip nonLamina edges when making manifold solids
      // if ( m_bMakingManifoldSolid && ! sIsLaminaVertex2[ii] )
      //   { continue; }

      dTest = sSortParameter2[ii];

      // init sSortedIndex with 1st entry
      if (sSortedIndex2.GetSize() == 0)
        {
          sSortedIndex2.Add(ii);
          continue;
        }

      // init for binary sort to find index for this entry in sorted list
      ULONG lMin = 0;
      ULONG lMax = sSortedIndex2.GetSize() - 1;

      // add entry to end to list when its Param is largest seen
      if ( dTest >= sSortParameter2[ sSortedIndex2.GetLast() ] )
        {
          sSortedIndex2.Add(ii);
        }

      // add entry to begin of list when its param is smallest seen
      else if ( dTest <= sSortParameter2[sSortedIndex2[0]] )
        {
          sSortedIndex2.InsertAt(0,ii);
        }

      // use binary search to find index for current entry
      else
        {
          // Binary search.
          while (lMin != lMax-1 && lMin != lMax)
            {
              ULONG lMid = (lMin+lMax) / 2;
              if ( dTest > sSortParameter2[ sSortedIndex2[lMid] ] )
                { lMin = lMid; }
              else
                { lMax = lMid; }
            }
          sSortedIndex2.InsertAt(lMax,ii);
        }
    } // end iter sorting every vertex2

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      sSortedIndex2.Dump();
    }
#endif // SM_DEBUG_CODE


  // Now both lists are sorted by projection on the line.
  // Now we march down the lists in order:
  // for each entry in List1,
  //   Step along list2 collecting vertices within tol,
  //     until the list2 parameter is past the list1 parameter.
  //   If more than one is found, pick the closest.
  //     Also check for closest List1 vertex to this List2 vertex.
  //   Glue.

  // Store the index [jj] values.
  SmTArray< ULONG > sCoin2jj;

  ULONG liiToGlue = 0;  // In case we glue something other than [ii].

  // So that we don't have to start at the beginning of list2 each time:
  ULONG lStartjj = 0;

  // for every Keep Vertex
  for ( ULONG ii = 0; ii < lNumVerts1; ii++ )
  {
      // If we glued any vertex other than [ii] last time (see below),
      // then we might not be finished with that one.
      if ( ii > 0  &&  ii != liiToGlue+1 )
        { ii--; }

      liiToGlue = ii;

      // target keep vertex
      SmVertex *pVtx1 = rVertexList1[ sSortedIndex1[ii] ];

      // no work - vertex already processed
      if ( pVtx1 == NULL )
        { continue; }

      double dParam1 = sSortParameter1[ sSortedIndex1[ii] ];

      // locals
      SmPoint3d sThisPt = pVtx1->GetPoint();

      // process the next set of delete vertices looking for coincidence
      sCoin2jj.ReSet();
      for ( ULONG jj = lStartjj; jj < lNumVerts2; jj++ )
      {
          // target delete vertex
          SmVertex *pVtx2   = sUsingVerts2[ sSortedIndex2[jj] ];
          double    dParam2 = sSortParameter2[ sSortedIndex2[jj] ];

          // skip already processed vertices
          if ( pVtx2 == NULL )
            { continue; }

          // remember to march past vertices who already missed their chance to be coincident
          if ( dParam2 < dParam1 - 2.0*m_dStitchTol3d )
          {
              lStartjj = jj+1; // done with this and all before it.
              continue;
          }

          // stop searching when delete vertex passes the target keep vertex
          if ( dParam2 > dParam1 + 2.0*m_dStitchTol3d )
            { break; }

          // KeepToDelete vertex gap
          double dDist = sThisPt.DistanceBetween( pVtx2->GetPoint() );

          // collect all delete vertices within tolerance of Keep vertex
          if ( dDist < m_dStitchTol3d )
          {
              // First just collect them.
              sCoin2jj.Add( jj );
          }
          else // look for minimum gap of unglued vertices
          {
              rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dDist );
          }
      } // end inner loop on VertexList2, populating coincident vertex indices in sCoin2jj.

      // arrive here when sCoin2jj contains list of all DeleteVertices within tol of the KeepVertex

      // If we found any vertices coincident to pVtx1,
      // find the closest one and glue.
      ULONG lNumCoin = sCoin2jj.GetSize();
      if ( lNumCoin > 0 )
      {
          // Find the closest deleteVertex and Glue it to pVtx1.
          ULONG ljjToGlue = sCoin2jj[0];

          SmVertex *pVtx2ToGlue = sUsingVerts2[ sSortedIndex2[ ljjToGlue ] ];

          double dMinDist = sThisPt.DistanceBetween( pVtx2ToGlue->GetPoint() );

          // when there are multiple candidates
          if ( lNumCoin > 1 )
          {
              for ( ULONG kk = 1; kk < lNumCoin; kk++ )
              {
                  // distance between candidateDeleteVertex and KeepVertex
                  ULONG lThisjj = sCoin2jj[kk];
                  SmVertex *pThisVtx = sUsingVerts2[ sSortedIndex2[ lThisjj ] ];
                  double dThisDist = sThisPt.DistanceBetween( pThisVtx->GetPoint() );

                  // remember the closest DeleteVertex
                  if ( dThisDist < dMinDist )
                  {
                    ljjToGlue = lThisjj;
                    dMinDist  = dThisDist;
                  }
                  else // remember smallest gap between unstitched vertices
                  {
                      rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisDist );
                  }
              } // end iter every candidate Delete Vertex
          } // end if lNumCoin > 1, so find best.

          // Ok, now we've found the closest vertex in List 2
          // to this [ii] vertex in List 1; it's the one at ljjToGlue.
          // Annoyingly, it's possible that there are nearby vertices
          // in List 1 that are closer to this List 2 vertex.
          // We have to check for that.
          //
          // On the bright side, having them sorted makes it easier.
          // We search List 2 in both directions starting at ljjToGlue.

          pVtx2ToGlue        = sUsingVerts2   [ sSortedIndex2[ ljjToGlue ] ];
          SmPoint3d sPoint2  = pVtx2ToGlue->GetPoint();
          double    dParam2  = sSortParameter2[ sSortedIndex2[ ljjToGlue ] ];

          // Search downward first.
          // (Unfortunately, ULONG's can't be negative, so we have to do this:)
          if ( liiToGlue > 0 )
          {
              for ( ULONG iii = liiToGlue-1; iii >= 0; iii-- )
              {
                  double dParam3 = sSortParameter1[ sSortedIndex1[ iii ] ];

                  if ( dParam3 < dParam2 - 2.0*m_dStitchTol3d )
                    { break; }

                  SmVertex *pThisVtx1 = rVertexList1[ sSortedIndex1[ iii ] ];
                  if ( pThisVtx1 == NULL )
                  {
                      if ( iii == 0 ) { break; }    // (Loop can't subtract from 0.)
                      else           { continue; }
                  }

                  double dThisDist = sPoint2.DistanceBetween( pThisVtx1->GetPoint() );

                  if ( dThisDist < dMinDist )
                  {
                      liiToGlue = iii;
                      dMinDist = dThisDist;
                  }

                  if ( iii == 0 ) { break; } // (Can't go negative.)
              }
          }

          // Now search upward.
          for ( ULONG iii = liiToGlue+1; iii < lNumVerts1; iii++ )
          {
              double dParam4 = sSortParameter1[ sSortedIndex1[ iii ] ];

              if ( dParam4 > dParam2 + 2.0*m_dStitchTol3d )
                { break; }

              SmVertex *pThisVtx1 = rVertexList1[ sSortedIndex1[ iii ] ];
              if ( pThisVtx1 == NULL ) { continue; }

              double dThisDist = sPoint2.DistanceBetween( pThisVtx1->GetPoint() );

              if ( dThisDist < dMinDist )
              {
                  liiToGlue = iii;
                  dMinDist = dThisDist;
              }
          }

          // remeber when we found a closer keep vertex
          if ( liiToGlue != ii )
          {
              pVtx1 = rVertexList1[ sSortedIndex1[ liiToGlue ] ];
          }


          // Now glue pVtx1 and pVtx2ToGlue.  pVtx2ToGlue will be deleted.
          // First set the position of the surviving vertex to be
          // the midpoint of the two ... this should help to keep tolerances
          // from growing excessively.
          // Actually, do it only if they are far enough apart.
          SmBoolean bCheckGaps = FALSE;
          if ( dMinDist > ( pVtx2ToGlue->GetTolerance() + pVtx2ToGlue->GetTolerance() ) / 4.0 )
          {
              SmPoint3d sMidPt = 0.5 * ( pVtx1->GetPoint() + pVtx2ToGlue->GetPoint() );
              pVtx1->SetPoint( sMidPt );
              bCheckGaps = TRUE;
          }

          double dOldTol = pVtx2ToGlue->GetTolerance();
          SmStatus eStat = pOwningBrep->GlueVertices( pVtx1, pVtx2ToGlue );

          // when GlueVertices fail
          if ( eStat != SM_SUCCESS )
          {
              // Prevent infinite loop: if the glue failed,
              // don't mess with these two vertices anymore.
              rVertexList1[ sSortedIndex1[ liiToGlue ] ] = NULL;
              sUsingVerts2[ sSortedIndex2[ ljjToGlue ] ] = NULL;

              if ( m_bIgnoreProblems ) { continue;     }
              else                     { SER( eStat ); }
          }

          SmTol::UpdateObjectTolerance( pVtx1, dMinDist, dOldTol, bCheckGaps );

          rdMaxVertexGap = smos_Max( rdMaxVertexGap, dMinDist );

          // Remove from our vertex list, and add to Processed list.
          rProcessedVerts.Add( pVtx1 );
          rDeletedVerts  .Add( pVtx2ToGlue );
          rVertexList1[ sSortedIndex1[ liiToGlue ] ] = NULL;
          sUsingVerts2[ sSortedIndex2[ ljjToGlue ] ] = NULL;

      } // end if lNumCoin > 0, and glue.

  } // end loop on all VertexList1, gluing when close.

  // Since we used a copy of rVertxList2, Null out the vertices we processed.
  for ( ULONG ii = 0; ii < sUsingVerts2.GetSize(); ii++ )
  {
      if ( sUsingVerts2[ii] == NULL )
        { rVertexList2[ sUsingIndices2[ii] ] = NULL; }
  }

  return SM_SUCCESS;

} // end SmStitch::StitchVertexPairs

/*******************************************************************//**
PURPOSE: Split edges in a Brep that are close to given vertices
   in their interiors.

NOTES:  
   Find any edges that are within tolerance of each vertex.
   If the vertex is in the interior of the edge
   (not within tolerance of either end),
   split the edge (creating a new vertex and a new edge),
   and glue the two vertices together.
   Return a list of the processed vertices -- the surviving one
   of each glued pair,
   and the original list with the processed vertices removed.

INPUTS ---
  rVerts       vertex list
  pEdgeTree    edge spatial tree

OUTPUTS ---
  Vertex list has all glued verts set to NULL.
  rProcessedVerts has all the glued verts that were set to Null in vertex list.
  All new Vertices are glued to existing Vertices, i.e. in the end there are no new Vertices.
  Edge tree gets updated with any new edges.

  Side effect: close edges were split in the Brep.
***********************************************************************/
SmStatus SmStitch::SplitEdgesWithVertices(
    SmTArray< SmVertex* > & rVerts,           ///< [in,out]: Vertices they may split Edges. Entries set to Null when used.       <br>
    SmTArray< SmEdge  * > & rEdges,           ///< [in,out]: In: Edges to split. Out: SplitEdges and NewEdges (i.e. all Edges)   <br>
    SmTArray< SmVertex* > & rProcessedVerts,  ///< [out]: Vertices that split an Edge.                                           <br>
    SmTArray< SmEdge  * > & rSplitEdges,      ///< [out]: Edges that were split                                                  <br>
    SmTArray< SmEdge  * > & rNewEdges,        ///< [out]: Edges that are created by the split                                    <br>
    double & rdMaxVertexGap,                  ///< [out]:                                                                        <br>
    double & rdMinUnstitchedVertGap             ///< [out]:                                                                        <br>
    )
{
  // Init outputs.
  rdMaxVertexGap = 0;
  rdMinUnstitchedVertGap = SM_BIG_DOUBLE;
  rProcessedVerts.ReSet();
  rSplitEdges    .ReSet();
  rNewEdges      .ReSet();

  // Locals.
  ULONG ii, jj;
  SmVertex   *pV;
  SmVertex   *pNewVertex;
  SmEdge     *pNewEdge;
  double      dDistToEdge;
  sReasonType eReason;
  SmStatus    eStat;

  ULONG lNumVerts = rVerts.GetSize();
  if ( lNumVerts < 1 ) { return SM_SUCCESS; }
  ULONG lNumEdges = rEdges.GetSize();
  if ( lNumEdges < 1 ) { return SM_SUCCESS; }


  // For each vertex in the list, look for close edges.
  for ( ii = 0; ii < lNumVerts; ii++ )
  {
      pV = rVerts[ii];
      if ( pV == NULL ) { continue; }

      // Do not do Manifold vertices if indicated.
      // We do allow Wire vertices though.
      if ( m_bMakingManifoldSolid  &&  ! pV->IsLaminaVertex() &&  ! pV->IsWireVertex() )
        { continue; }

      SmPoint3d sPvPoint = pV->GetPoint();

      // Find all edges close to this vertex.
      SmExtent3d sVBBox( pV->GetPoint() );
      sVBBox.ExpandAbsolute( pV->GetTolerance() );  //cbi was tol * 20
      for ( jj = 0; jj < lNumEdges; jj++ )
      {
          SmEdge *pEdge = rEdges[jj];
          if ( pEdge == NULL ) { continue; }

          // Again, do only Lamina edges if indicated.
          if ( m_bMakingManifoldSolid  &&  ! pEdge->IsLamina() &&  ! pEdge->IsWire() )
            { continue; }

          double dThisTol = smos_Max( m_dStitchTol3d,
                                      (double) pV->GetTolerance() + (double) pEdge->GetTolerance() );

          eStat = sm_TestAndSplitEdge( pEdge, sPvPoint, dThisTol, eReason,
                                       dDistToEdge, pNewVertex, pNewEdge );

          if ( eStat != SM_SUCCESS )
          {
              if ( m_bIgnoreProblems ) { continue;     }
              else                     { SER( eStat ); }
          }

          // when split created a new vertex
          if ( pNewVertex != NULL )
          {
              // Glue the new vertex to pV, and switch lists.
              SmBrep *pBrep = pV->GetBrep();
              if ( pBrep == NULL )
              {
                  if ( m_bIgnoreProblems ) { continue;     }
                  else                     { SER( eStat ); }
              }

              // First set the position of the surviving vertex to be
              // the midpoint of the two ... this should help to keep tolerances
              // from growing excessively.
              SmPoint3d sMidPt = 0.5 * ( pV->GetPoint() + pNewVertex->GetPoint() );
              pV->SetPoint( sMidPt );

              pBrep->GlueVertices( pV, pNewVertex );
              rProcessedVerts.Add( pV );
              rVerts[ii] = NULL;

              // Set max distance.
              rdMaxVertexGap = smos_Max( rdMaxVertexGap, dDistToEdge );

              SmTol::UpdateObjectTolerance( pV,    dDistToEdge );
              SmTol::UpdateObjectTolerance( pEdge, dDistToEdge );
          }
          else
          {
              // Set min distance, if it didn't split due to being
              // too far away (as opposed to being at a vertex).
              if ( eReason == FAR_FROM_GEOM )
              {
                  rdMinUnstitchedVertGap = smos_Min ( rdMinUnstitchedVertGap, dDistToEdge );
              }
          }

          // Add the split edge and the new edge to the output lists.
          if ( pNewEdge != NULL )
          {
//            SmExtent3d sEBBox;
//            SmCurve *pCurve = pNewEdge->GetCurve();
//            SER( pCurve->CalculateBoundingBox( pNewEdge->GetInterval(), &sEBBox ));
//            SER( pEdgeTree->AddToSpatialTree( sEBBox, pNewEdge ));

              rSplitEdges.Add( pEdge );
              rNewEdges.Add( pNewEdge );

              SmTol::UpdateObjectTolerance( pNewEdge, dDistToEdge, pEdge->GetTolerance() );

              // Also add to main Edges list: might still need processing in this routine.
              rEdges.Add( pNewEdge );
              lNumEdges = rEdges.GetSize();
          }

      } // end for all close edges to pV (jj)
  } // end loop on unprocessed vertices looking for close edges in Brep to split

  return SM_SUCCESS;

} // end SplitEdgesWithVertices

/*******************************************************************//**
PURPOSE: For each vertex in a given list, glue any pairs of coincident edges
   incident on the vertex, that also share another vertex in the list.

NOTES:
   Find any pairs of edges incident on each vertex that have common other-vertices.
   If the edge pair are coincident, glue the two edges together.
   Return a list of the processed edges -- the surviving one of each glued pair,
   and the original list with the processed vertices removed.

   In SmStitch::DoStitching, they keep track of Faces that have any of
   their edges modified, and CreateUVTrimCurves() on them.
   This routine does not do that.
   If desired, that could be done by creating trim curves on
   all Faces connected to the edges returned in rProcessedEdges.

***********************************************************************/
SmStatus SmStitch::StitchEdgesOfVertices
 (SmTArray< SmVertex* > & rVerts,              // in :  Check these Vertices for coincident edges to stitch   
  SmTArray< SmEdge  * > & rGluedEdges,         // out:  Edges that survived being glued (extant)           
  SmTArray< SmEdge  * > & rDeletedEdges,       // out: Edges that gluing deleted (stale pointers)         
  SmTArray< SmVertex* > & rProcessedVerts,     // NotUsed: out: Vertices that had Edges glued                      
  double                & rdMaxEdgeGap,        // out:
  double                & rdMinUnstitchedEdgeGap)  // out:
{
  SM_REF1(rProcessedVerts) ;
  // Init outputs.
  rdMaxEdgeGap   = 0.0;
  rdMinUnstitchedEdgeGap = SM_BIG_DOUBLE;

  // Locals.
  SmTArray< SmEdge* > sVEdges;
  SmVertex *pV;
  ULONG ii, jj, kk;

  ULONG lNumVerts = rVerts.GetSize();
  for ( ii = 0; ii < lNumVerts; ii++ )
  {
      pV = rVerts[ii];

      if ( pV == NULL ) { continue; }  // Can this happen?  Don't really care.

      //
      // About the following double-loop on all edges of this vertex pV:
      // A vertex could easily have more than one pair of edges that
      // have to be glued.
      //
      // If we could say that gluing two edges takes them out of consideration,
      // then we could just set them both to Null, and continue in the outer edge-loop.
      // But if (m_bMakingManifoldSolid==False), then we're allowed to make spine edges,
      // and we have to keep looking at more coincident edges.
      //
      // Also, about SelectTopologyToKeep() : we really don't care here, because
      // they're both ending up in the same place, so we can keep whichever is easier.
      //
      // So here's the logic for this:
      //   After gluing:
      //   First, always preserve [jj] and delete [kk]  (As noted, it doesn't matter.)
      //   Set [kk] to Null.
      //   If we can make spine edges (m_bMakingManifoldSolid==False), then
      //     continue on with the next kk (inner loop)
      //   else
      //     set [jj] to Null (although it doesn't matter, because:)
      //     go on to the next jj.
      //
      // Bottom line, we don't need the DoneWithThisVertex thing that's in DoStitching().
      //

      pV->GetEdges( sVEdges );

      ULONG lNumVEdges = sVEdges.GetSize();

      // Outer loop on all of this vertex's Edges:
      for ( jj = 0; jj+1 < lNumVEdges; jj++ ) // note, can't say 'lNumVEdges-1' : unsigned.
      {
          SmEdge *pE = sVEdges[jj];

          if ( pE == NULL ) { continue; }

          // If we are making a manifold solid only stitch lamina edges.
          // Note, don't check IsLamina, just avoid making spine edges.  // [100216]
          if ( m_bMakingManifoldSolid /* && !pE->IsLamina() */ )
            {
              SmTArray< SmEdgeuse* > sEUs;
              pE->GetEdgeuses( sEUs );
              if ( sEUs.GetSize() >= 4 )
                { continue; }
            }

          // Inner loop on all of this vertex's other Edges:
          for ( kk = jj+1; kk < lNumVEdges; kk++ )
          {
              SmEdge *pTestE = sVEdges[kk];

              if ( pTestE == NULL ) { continue; }


              // If we are making a manifold solid only stitch lamina edges
              // Don't check IsLamina, just avoid making spine edges.  // [100216]
              if ( m_bMakingManifoldSolid /* && !pTestE->IsLamina() */ )
                {
                  SmTArray< SmEdgeuse* > sEUs;
                  pTestE->GetEdgeuses( sEUs );
                  if ( sEUs.GetSize() >= 4 )
                    { continue; }
                }

              // skip edges that don't share common vertices on both ends
              if (pE->GetOtherVertex(pV) != pTestE->GetOtherVertex(pV))
                { continue; }


              // Found a pair of edges with the same two vertices.
              // Check for coincidence.
              // First check midpoints.
              SmCurve *pECurve  = pE->GetCurve();
              SmCurve *pTECurve = pTestE->GetCurve();

              SmPoint3d sMidPnt;
              SER(pECurve->EvaluatePoint(pE->GetInterval().Evaluate(0.5),sMidPnt));

              // select new tolerance
              double dThisTol = smos_Max( m_dStitchTol3d,
                                          (double)(pTestE->GetTolerance() + pE->GetTolerance()) );

              // locals
              SmBoolean bSuccess;
              double dDropParam, dDistToCurve;

              SmExtent1d sTEIvl  = pTestE->GetInterval();
              double dGuessParam = sTEIvl.GetMid();

              // This tolerance tells DropPoint to quit looking if the solution
              // is going to be larger than this value.
              // But to find rdMinUnstitchedEdgeGap, collect every solution.
              double dDropTol = SM_BIG_DOUBLE;

              // skip edge pair if pE->midPoint is not within tolerance of pTestE
              SER( pTECurve->DropPoint(sTEIvl,           // in : target curve allowed domain
                                       sMidPnt,          // in : Point to drop to curve
                                       NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                         //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                         //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                       dDropTol,         // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                         //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                         //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                         //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                       &dGuessParam,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                       bSuccess,         // out: TRUE = found a drop point
                                       dDropParam,       // out: found drop curve param
                                       dDistToCurve )) ; // out: found drop distance
                                                         // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                         //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                         //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                         //      default:[SM_SO_MINIMIZE] to preserve original behavior
              if (!bSuccess)
                { continue; }

              if ( dDistToCurve > dThisTol )
              {
                  rdMinUnstitchedEdgeGap = smos_Min( dDistToCurve, rdMinUnstitchedEdgeGap );
                  continue;
              }

              // Ok, the midpoints are close.

              // Need to figure out the relative orientation of the two curves,
              // for getting dTStart and dTEnd for the CurveMaxDistanceBetween
              // check, and for the edge-gluing call.

              // get midPoint tangents
              SmVector3d sPV[2], sPVTest[2];
              SER( pECurve ->Evaluate( pE->GetInterval().Evaluate(0.5), 1, TRUE, sPV ));
              SER( pTECurve->Evaluate( dDropParam,                      1, TRUE, sPVTest ));

              // Set dTStart and dTEnd according to relative orientations.
              SmOrientType eRelativeOrientation = SM_OT_SAME;
              double dTStart = pTestE->GetInterval().GetMin();
              double dTEnd   = pTestE->GetInterval().GetMax();
              if ( sPV[1].Dot( sPVTest[1] ) < 0.0)
              {
                  eRelativeOrientation = SM_OT_OPPOSITE;
                  double dTTmp = dTStart;
                  dTStart      = dTEnd;
                  dTEnd        = dTTmp;
              }

              // Skip edge pairs not within tolerance of one another.
              dDistToCurve = 0.0;

              // If both edges are lines then we don't have to do the harder test.
              SmBoolean bEdgesAreLines =
                        ( pECurve->GetDegree()  == 1 &&  pECurve->GetNumberNaturalKnots() == 4 )
                    &&  ( pTECurve->GetDegree() == 1 && pTECurve->GetNumberNaturalKnots() == 4 );

              if ( !bEdgesAreLines )
              {
                  // do a quick 5 point distance check
                  SER(pECurve->CurveMaxDistanceBetween(pE->GetInterval(),*pTECurve,
                    dTStart,dTEnd,5,&dThisTol,dDistToCurve));
                  if (dDistToCurve > dThisTol)
                  {
                      rdMinUnstitchedEdgeGap = smos_Min( dDistToCurve, rdMinUnstitchedEdgeGap );
                      continue;
                  }

                  // do a precise distance check
                  if ( ! m_bFastEdgeCompare )
                  {
                      // Passed an initial quick test with 5 points now do a precise measurement
                      SER(pECurve->CurveMaxDistanceBetween(pE->GetInterval(),*pTECurve,
                          dTStart,dTEnd,0,&dThisTol,dDistToCurve));
                      if (dDistToCurve > dThisTol)
                      {
                          rdMinUnstitchedEdgeGap = smos_Min( dDistToCurve, rdMinUnstitchedEdgeGap );
                          continue;
                      }
                  }
              }

              // If make it here then we need to glue edges

              // save the maxEdgeGap value
              if ( dDistToCurve > rdMaxEdgeGap ) { rdMaxEdgeGap = dDistToCurve; }

              // Note, in DoStitching(), they select which edge to keep here.
              // We don't care, we just keep pE, delete pTestE.

              SmBrep *pBrep = pE->GetBrep();
              if ( pBrep == NULL )
              {
                  if ( m_bIgnoreProblems ) { continue;      }
                  else                     { SER( SM_ERR ); }
              }

              // glue pE/pTestE edge pair - keep pE
              double dOldTol = pTestE->GetTolerance();
              if ( pBrep->GlueEdgesGeneral( pE, eRelativeOrientation, dDistToCurve,
                     m_bDoRegionNesting, pTestE ) == SM_SUCCESS )
              {
                  rGluedEdges  .Add( pE );
                  rDeletedEdges.Add( pTestE );

                  SmTol::UpdateObjectTolerance( pE, dDistToCurve, dOldTol );

                  sVEdges[kk] = NULL;  // per notes above (above the outer edge loop, jj).

                  // Now about the logic:
                  if ( m_bMakingManifoldSolid == FALSE )
                  {
                      // spine edges are allowed: just go on to the next inner-loop [kk]
                      continue;
                  }
                  else
                  {
                      // spine edges not allowed, so we're done with both of these edges.
                      // go on to the next outer-loop [jj]
                      sVEdges[jj] = NULL;  // per notes above: unnecessary, but...
                      break;
                  }

              } // end if GlueEdgesGeneral succeeded.

              else if (!m_bIgnoreProblems)
              {
                     SER(SM_ERR);
              }
              // else {
              //      SmTArray<SmEdge *> tmpEdges;
              //        pBrep->GetEdges(tmpEdges);
              //        ULONG foundIndex;
              //        if (tmpEdges.FindElement(sVEdges[kk], foundIndex) == FALSE)
              //          sVEdges[kk] = NULL;
              // }

          } // end inner loop on all other edges (kk)
      } // end outer loop on all edges of this pV (jj)

  } // end for all processed vertices

  return SM_SUCCESS;

} // end StitchEdgesOfVertices
