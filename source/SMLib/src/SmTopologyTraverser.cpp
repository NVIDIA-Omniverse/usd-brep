// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTopologyTraverser.cpp
* PURPOSE: Source file for SmTopologyTraverser object.
**********************************************************************/

#include "StdAfx.h"

#include <SmTopologyTraverser.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmBSplineCurve.h>
#include <SmTopologySolver.h>
#include <SmGraphicsExtern.h>
#include <SmAttribute.h>

/*******************************************************************//**
PURPOSE: Gather all faces connected to target faces through edges.
   eMarkType all faces and the edges and vertices that are connected to those
   faces.  These marks can be checked with pObject->IsMarked(eMarkType).

NOTES:  If you use this standalone then you will need to do
    a SmTopology::NewMark(eMarkType) prior to calling it.
***********************************************************************/
static SmStatus sm_CollectFaces
 (SmFace               * pStartFace,                    // in : seed face
  SmTArray<SmFace*>    & rCollectedFaces,               // out: unmarked faces conned to seed through unmarked edges
  SmMarkType             eMarkType,                     // in : mark type to check, not incremented, default:[SM_MT_MARK2]
  SmTArray<SmEdge*>    * pOptCollectedEdges,            // in : optional unmarked edges connected to any CollectedFace
  SmTArray<SmVertex *> * pOptCollectedVertices)         // in : optional unmarked vertices connected to any CollectedFace
{
  // init output
  rCollectedFaces.ReSet();
  if(pOptCollectedEdges)    { pOptCollectedEdges->ReSet() ; }
  if(pOptCollectedVertices) { pOptCollectedVertices->ReSet() ; }

  // allocate a Face Pointer Stack
  SmEdge * sData[64];
  SmFace * sData2[64];
  SmFace * sFData[64];
  SmTArray<SmEdge*> sEdges(64,sData);    // edges connected to a target face
  SmTArray<SmFace*> sFaces(64,sData2);
  SmTArray<SmFace*> sStack(64,sFData);   // stack of faces to process

  // init Face Stack with input
  sStack.Add(pStartFace);

  // while faces are on the stack
  while (sStack.GetSize() > 0) 
    {
      // get next StartFace
      SmFace *pFaceStart = sStack.GetLast();
      sStack.RemoveLast();
      
      // skip processed faces
      if (pFaceStart->IsMarked(eMarkType)) continue;

      // place face in output and mark it
      rCollectedFaces.Add( pFaceStart );
      pFaceStart->Mark(eMarkType);
      
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          ULONG ii ;
          SmBrep *pBrep = pFaceStart->GetBrep() ;
          SmTArray<SmEdge *> sBrepEdges ;
          if(pBrep) { pBrep->GetEdges( sBrepEdges ) ; }

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,1) ; for(ii=0;ii<sBrepEdges.GetSize();ii++)
                                        { if(sBrepEdges[ii]->IsMarked(eMarkType)) { sBrepEdges[ii]->Draw() ; } sm_GraphicsLoop() ; }
          smgfx_SetLook(1,2, 0,1,1) ; for(ii=0;ii<rCollectedFaces.GetSize();ii++)
                                        { rCollectedFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
          smgfx_SetLook(2,3, 0,1,0) ; pFaceStart->Draw(SM_DM_CROSSHATCH,10,10) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; for(ii=0;ii<sStack.GetSize();ii++)
                                        { sStack[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop();
        }
#endif
      
      // Now try to traverse adjacent edges to pick up
      // surrounding faces.

      // for every edge connected to current face
      pFaceStart->GetEdges(sEdges);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          // skip marked edges
          SmEdge *pEdge = sEdges[i];
          if (pEdge->IsMarked(eMarkType)) continue;

          // when asked - collect the edges
          if(pOptCollectedEdges) { pOptCollectedEdges->Add(pEdge) ; }

          // mark edge
          pEdge->Mark(eMarkType);

          // Mark edge->Vertices - when asked - collect the Vertices
          SmVertex *pV1 = pEdge->GetVertex();
          SmVertex *pV2 = pEdge->GetOtherVertex(pV1);

          if(pOptCollectedVertices && !pV1->IsMarked(eMarkType)) { pOptCollectedVertices->Add(pV1) ; }
          pV1->Mark(eMarkType);
          
          if(pOptCollectedVertices && !pV2->IsMarked(eMarkType)) { pOptCollectedVertices->Add(pV2) ; }
          pV2->Mark(eMarkType);

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              ULONG ii ;
              SmBrep *pBrep = pFaceStart->GetBrep() ;
              SmTArray<SmEdge *> sBrepEdges;
              if(pBrep) { pBrep->GetEdges( sBrepEdges ) ; }

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,1) ; for(ii=0;ii<sBrepEdges.GetSize();ii++)
                                            { if(sBrepEdges[ii]->IsMarked(eMarkType)) { sBrepEdges[ii]->Draw() ; } sm_GraphicsLoop() ; }
              smgfx_SetLook(1,2, 0,1,1) ; for(ii=0;ii<rCollectedFaces.GetSize();ii++)
                                            { rCollectedFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
              smgfx_SetLook(2,3, 0,1,0) ; pFaceStart->Draw(SM_DM_CROSSHATCH,10,10) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,1,0) ; for(ii=0;ii<sStack.GetSize();ii++)
                                            { sStack[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop();
            }
#endif
          // Add all unmarked faces connected to edge to face stack
          pEdge->GetFaces(sFaces);
          for (ULONG j=0; j<sFaces.GetSize(); j++) 
            {
              SmFace *pF = sFaces[j];
              if (!pF->IsMarked(eMarkType)) 
                {
                  sStack.AddUnique(pF);
                }
            } // end iter all edge->Faces
        } // end iter all face->edges
    } // end while faces are on the stack check

  return SM_SUCCESS;

} // end sm_CollectFaces

/*******************************************************************//**
PURPOSE: Collect all faces connected to StartFace through shared
   edge connections (which are not marked (eMarkType)).
   
   Mark the collected faces (eMarkType) and their edges and vertices.

NOTES: If you use this standalone then you will need to do
       a SmTopology::NewMark(eMarkType) prior to calling it.
***********************************************************************/
SmStatus  SmTopologyTraverser::CollectFaces
 (SmFace               * pStartFace,                    // in : seed face
  SmTArray<SmFace*>    & rCollectedFaces,               // out: unmarked faces conned to seed through unmarked edges
  SmMarkType             eMarkType,                     // in : mark type to check, not incremented, default:[SM_MT_MARK2]
  SmTArray<SmEdge*>    * pOptCollectedEdges,            // in : optional unmarked edges connected to any CollectedFace
                                                        //      NULL to ignore, default:[NULL]
  SmTArray<SmVertex *> * pOptCollectedVertices)         // in : optional unmarked vertices connected to any CollectedFace
                                                        //      NULL to ignore, default:[NULL]
{
  // init input
  rCollectedFaces.ReSet() ;

  // Sanity check
  if ( pStartFace == NULL ) { return SM_SUCCESS; }

  // no work - face already marked
  if (pStartFace->IsMarked(eMarkType)) { return SM_SUCCESS; }

  // pass the call along
  SER(sm_CollectFaces(pStartFace,
                      rCollectedFaces,
                      eMarkType,
                      pOptCollectedEdges,
                      pOptCollectedVertices)) ;

  // all done
  return SM_SUCCESS ;

} // end SmTopologyTraverser::CollectFaces

/*******************************************************************//**
PURPOSE: Mark (Mark(eMarkType)) and Collect all wires connected 
   to pStartWireEdge through edge/vertex connections.

NOTES: 

METHOD --- Recursively finds all edges connected to the input
   edge's vertices that are wire edges.  Uses eMarkType to skip wires 
   already processed.  Recursion stops when no more unmarked 
   vertices connected to edges are connected to more wire edges.

***********************************************************************/
static SmStatus sm_CollectWires
 (SmEdge               * pStartWireEdge,                 // in : target wire (gets marked)              
  SmTArray<SmEdge*>    & rCollectedEdges,                // out: unmarked wire edges connected to pStartWireEdge that are wires (get marked)              
  SmMarkType             eMarkType,                      // in : mark type to check, not incremented, default:[SM_MT_MARK2]
  SmTArray<SmVertex *> * pOptCollectedVertices)          // in : optional unmarked vertices connected to any CollectedEdge
                                                         //      NULL to ignore, default:[NULL]
{
  if ( pStartWireEdge == NULL )
      return SM_SUCCESS;

  // skip marked edges and edges that aren't wires
  if (    pStartWireEdge->IsMarked(eMarkType)
      || !pStartWireEdge->IsWire())  return SM_SUCCESS;

  // init CollectedEdges list with input Edge
  rCollectedEdges.Add(pStartWireEdge);

  // mark the edge
  pStartWireEdge->Mark(eMarkType);

  // Now try to traverse adjacent edges to pick up
  // surrounding wire edges
  SmEdge * sData[64];
  SmTArray<SmEdge*> sEdges(64,sData);
  SmVertex *pV1 = pStartWireEdge->GetVertex();
  SmVertex *pV2 = pStartWireEdge->GetOtherVertex(pV1);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); pStartWireEdge->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,4, 0,1,1); pV1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,0,1); pV2->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif
  if (   (!pV1->IsMarked(eMarkType)) 
      && pV1->IsWireVertex()) 
    {
      // when asked - collect the Vertices
      if(pOptCollectedVertices) { pOptCollectedVertices->Add(pV1) ; }

      pV1->Mark(eMarkType);
      pV1->GetEdges(sEdges);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          SmEdge *pEdge = sEdges[i];
          if(pEdge != pStartWireEdge) 
            { SER(sm_CollectWires(pEdge, rCollectedEdges, eMarkType, pOptCollectedVertices)); }
        }
    }

  if (   (!pV2->IsMarked(eMarkType)) 
      && pV2->IsWireVertex()) 
    {
      // when asked - collect the Vertices
      if(pOptCollectedVertices) { pOptCollectedVertices->Add(pV2) ; }

      pV2->Mark(eMarkType);
      pV2->GetEdges(sEdges);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          SmEdge *pEdge = sEdges[i];
          if(pEdge != pStartWireEdge) 
            { SER(sm_CollectWires(pEdge, rCollectedEdges, eMarkType, pOptCollectedVertices)); }
        }
    }

  return SM_SUCCESS;

} // end sm_CollectWires

/*******************************************************************//**
PURPOSE: Mark (isMarked(eMarkType)) and Collect a set of wire edges
     connected to a target wireEdge through vertex/Edge connections.

NOTES: If you use this standalone then you will need to do
    a SmTopology::NewMark(eMarkType) prior to calling it.
***********************************************************************/
SmStatus SmTopologyTraverser::CollectWireEdges
 (SmEdge               * pStartWireEdge,                 // in : target wire (gets marked)           
  SmTArray<SmEdge*>    & rCollectedEdges,                // out: all wires connected to target wire (get marked)
  SmMarkType             eMarkType,                      // in : specify mark for target objects (not incremented)
  SmTArray<SmVertex *> * pOptCollectedVertices)          // in : optional unmarked vertices connected to any CollectedEdge
                                                         //      NULL to ignore, default:[NULL]
{
  // init output
  rCollectedEdges.ReSet() ;

  // pass the call along
  SER(sm_CollectWires(pStartWireEdge,                    // in : target wire (gets marked)
                      rCollectedEdges,                   // out: unmarked wire edges connected to pStartWireEdge that are wires (get marked)              
                      eMarkType,                         // in : specify mark for target objects (not incremented)
                      pOptCollectedVertices)) ;          // in : optional unmarked vertices connected to any CollectedEdge
                                                         //      NULL to ignore, default:[NULL]
  return SM_SUCCESS ;

} // end SmTopologyTraverser::CollectWireEdges

/*******************************************************************//**
PURPOSE: Mark (Mark(eMarkType)) and Collect all lamina connected 
   to pStartLaminEdge through edge/vertex connections.

NOTES: 

METHOD --- Recursively finds all edges connected to the input
   edge's vertices that are lamina edges.  Uses eMarkType to skip laminas 
   already processed.  Recursion stops when no more unmarked 
   vertices connected to edges are connected to more lamina edges.

***********************************************************************/
static SmStatus sm_CollectLaminas
 (SmEdge               * pStartLaminaEdge,               // in : target lamina (gets marked)              
  SmTArray<SmEdge*>    & rCollectedEdges,                // out: unmarked wire edges connected to pStartWireEdge that are wires (get marked)              
  SmMarkType             eMarkType,                      // in : mark type to check, not incremented, default:[SM_MT_MARK2]
  SmTArray<SmVertex *> * pOptCollectedVertices)          // in : optional unmarked vertices connected to any CollectedEdge
                                                         //      NULL to ignore, default:[NULL]
{
  if ( pStartLaminaEdge == NULL )
      return SM_SUCCESS;

  // skip marked edges and edges that aren't laminas
  if (    pStartLaminaEdge->IsMarked(eMarkType) || !pStartLaminaEdge->IsLamina())  
    return SM_SUCCESS;

  // init CollectedEdges list with input Edge
  rCollectedEdges.Add(pStartLaminaEdge);

  // mark the edge
  pStartLaminaEdge->Mark(eMarkType);

  // Now try to traverse adjacent edges to pick up
  // surrounding wire edges
  SmEdge * sData[64];
  SmTArray<SmEdge*> sEdges(64,sData);
  SmVertex *pV1 = pStartLaminaEdge->GetVertex();
  SmVertex *pV2 = pStartLaminaEdge->GetOtherVertex(pV1);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); pStartLaminaEdge->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,4, 0,1,1); pV1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,0,1); pV2->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  if (   (!pV1->IsMarked(eMarkType)) && pV1->IsLaminaVertex()) 
    {
      // when asked - collect the Vertices
      if(pOptCollectedVertices) { pOptCollectedVertices->Add(pV1) ; }

      pV1->Mark(eMarkType);
      pV1->GetEdges(sEdges);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          SmEdge *pEdge = sEdges[i];
          if(pEdge != pStartLaminaEdge) 
            { SER(sm_CollectLaminas(pEdge, rCollectedEdges, eMarkType, pOptCollectedVertices)); }
        }
    }

  if (   (!pV2->IsMarked(eMarkType)) && pV2->IsLaminaVertex()) 
    {
      // when asked - collect the Vertices
      if(pOptCollectedVertices) { pOptCollectedVertices->Add(pV2) ; }

      pV2->Mark(eMarkType);
      pV2->GetEdges(sEdges);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          SmEdge *pEdge = sEdges[i];
          if(pEdge != pStartLaminaEdge) 
            { SER(sm_CollectLaminas(pEdge, rCollectedEdges, eMarkType, pOptCollectedVertices)); }
        }
    }

  return SM_SUCCESS;

} // end sm_CollectLaminas

/*******************************************************************//**
PURPOSE: Mark (isMarked(eMarkType)) and Collect a set of lamina edges
     connected to a target laminaEdge through vertex/Edge connections.

NOTES: If you use this standalone then you will need to do
    a SmTopology::NewMark(eMarkType) prior to calling it.
***********************************************************************/
SmStatus SmTopologyTraverser::CollectLaminaEdges
 (SmEdge               * pStartLaminaEdge,               // in : target lamina (gets marked)           
  SmTArray<SmEdge*>    & rCollectedEdges,                // out: all wires connected to target wire (get marked)
  SmMarkType             eMarkType,                      // in : specify mark for target objects (not incremented)
  SmTArray<SmVertex *> * pOptCollectedVertices)          // in : optional unmarked vertices connected to any CollectedEdge
                                                         //      NULL to ignore, default:[NULL]
{
  // init output
  rCollectedEdges.ReSet() ;

  // pass the call along
  SER(sm_CollectLaminas(pStartLaminaEdge,                // in : target wire (gets marked)
                      rCollectedEdges,                   // out: unmarked wire edges connected to pStartLaminaEdge that are wires (get marked)              
                      eMarkType,                         // in : specify mark for target objects (not incremented)
                      pOptCollectedVertices)) ;          // in : optional unmarked vertices connected to any CollectedEdge
                                                         //      NULL to ignore, default:[NULL]
  return SM_SUCCESS ;

} // end SmTopologyTraverser::CollectLaminaEdges

/*******************************************************************//**
PURPOSE: Collect all curves in the same loop as a given curve and 
            (optional) face

NOTES: 
***********************************************************************/
SmStatus  SmTopologyTraverser::CollectCurveLoop
   (  const SmCurve *pCrv,                 // in  Curve 
      const SmFace *pFace,                 // in (optional) Face (or NULL)
      SmTArray<SmCurve *> &rCurveLoop)     // list of curves (including pCrv) in same loop
{
    ULONG i;
    rCurveLoop.ReSet();
    SmEdge *pEdge = (SmEdge *)pCrv->GetEdge();
    if (!pEdge) return(SM_ERR);
    
    SmTArray < SmEdgeuse *> sEdgeuses;
    pEdge->GetEdgeuses(sEdgeuses);
    if (sEdgeuses.GetSize() < 1) return (SM_ERR);
    
    //which Edgeuse do I select?
    SmEdgeuse *pEU  = sEdgeuses[0];
    if ( pFace != NULL) 
    {
      for (i=0; i<sEdgeuses.GetSize(); i++)
      {
        pEU  = sEdgeuses[i];
        if (pEU == NULL) 
             return(SM_ERR);
        SmFace *pEUFace = pEU->GetFace();
        if ( pEUFace == pFace) break;
      }
    }
    SmLoopuse *pLU  = pEU->GetLoopuse();
    if (pLU == NULL) 
        return(SM_ERR);
    SmLoop    *pLoop = pLU->GetLoop();
    if (pLoop == NULL) 
        return(SM_ERR);

    SmTArray<SmEdge *> sEdges;
    pLoop->GetEdges(sEdges);
    if ( sEdges.GetSize() < 1)
        return (SM_ERR);
    
    for (i = 0; i < sEdges.GetSize(); i++)
    {
        SmCurve *pCurve = sEdges[i]->GetCurve();
        rCurveLoop.Add(pCurve);
    }

    return (SM_SUCCESS);

} // end SmTopologyTraverser::CollectCurveLoop

/*******************************************************************//**
PURPOSE: Collect all regions connected to the given Region through Faceuses.

NOTES: 
  - Does not return pStartRegion in the list.
  - NO USE of marks.
***********************************************************************/
SmStatus  SmTopologyTraverser::CollectRegions
   (  const SmRegion *pStartRegion,      // in
      SmTArray<SmRegion *> &rAllRegions) // out: list of connected Regions (not including pStartRegion)
{
  rAllRegions.ReSet();

  // Use a Stack of Regions.
  SmTArray< SmRegion* > sRegionStack;
  SmTArray< SmRegion* > sRegionMates;
  SmTArray< SmShell * > sShells;
  SmRegion *pThisReg;
  ULONG lIdx; 
  pStartRegion->GetAdjacentRegions( sRegionMates );

  // Init stack to adjacent Regions of pStartRegion.
  sRegionStack = sRegionMates;

  while ( sRegionStack.GetSize() > 0 )
    {
      if ( ! sRegionStack.Pop( pThisReg ) ) // Retrieves and removes Last.
        { continue; }
      if ( pThisReg == pStartRegion )
        { continue; }

      pThisReg->GetShells( sShells );
      ULONG ii, lNumShells = sShells.GetSize();
      for ( ii = 0; ii < lNumShells; ii++ )
        {
          SmRegion *pRegMate = sShells[ii]->GetRegion();
          if ( pRegMate == NULL )         { continue; }
          if ( pRegMate == pStartRegion ) { continue; }
          if ( rAllRegions.FindElement( pRegMate, lIdx ) )
            { continue; }  // Already processed.

          rAllRegions.Add( pRegMate );

          pRegMate->GetAdjacentRegions( sRegionMates );
          ULONG jj, lNumMates = sRegionMates.GetSize();
          for ( jj = 0; jj < lNumMates; jj++ )
            {
              SmRegion *pReg = sRegionMates[jj];
              if (      pReg != pStartRegion
                   && ! sRegionStack.FindElement( pReg, lIdx ) )
                { 
                  sRegionStack.Push( pReg );
                }
            } // end for each mate Region.
        } // end for each Shell in this Region from the stack
    } // end while stack not empty

  return SM_SUCCESS;

} // end SmTopologyTraverser::CollectRegions

/*******************************************************************//**
PURPOSE: Determine if the shell specified by this faceuse is
   closed.  This method will traverse the faceuse and adjacent
   faceuses until it is either done or comes back to itself.

NOTES:  Calls rMark.NewMark() and Marks every faceuse that
        is traversed.  After this call marks can be checked 
        with pFaceuse->IsMarked(rMark.GetMarkType()).

METHOD ---
  For a Brep under construction, we're not necessarily looking for watertight.
  Here we crawl the shell, and if we traverse to the mate of the given faceuse,
  we declare it open; it's closed iff traversal never reaches the mate.
***********************************************************************/
SmStatus SmTopologyTraverser::ClosureTraversal
 (SmFaceuse              * pStartFaceuse, // in : target faceuse (gets marked)
  SmBoolean              & rbIsClosed,    // out: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't
  SmTArray < SmFaceuse*> * pOptFaceuses,  // out: List of all connected faces (get marked), NULL to ignore.
  SmNewMarkAndLock       & rMark)         // in : specify mark for target objects (incremented)
{
  // initialize outputs
  if (pOptFaceuses)
      pOptFaceuses->ReSet();

  // A face with any lamina edges is not closed.
  // Check every edge connected to the faceuse->face.
  if(pStartFaceuse->GetFace()->HasLaminaEdge())
    {
      rbIsClosed = FALSE;
      return SM_SUCCESS;
    }

  // increment the context's current mark value
  rMark.NewMark();
    
  // pass the call along
  SER( FaceuseTraversal( pStartFaceuse, // in : target faceuse (gets marked)                              
                         rbIsClosed,    // out: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't
                         pOptFaceuses,  // out: List of all connected faces (get marked), NULL to ignore.
                         rMark ));      // in : specify mark for target objects (not incremented)

  return SM_SUCCESS;

} // end SmTopologyTraverser::ClosureTraversal

/*******************************************************************//**
PURPOSE: Order an array of Edges into topological chains

NOTES:   Each Edge can share a vertex with only one other Edge in 
         the list.

         rChainStart provides indices of where each chain starts. 
         rChainStart.GetSize() == Number of chains
***********************************************************************/
SmStatus SmTopologyTraverser::FixEdgeChainList( SmTArray<SmEdge*>  & rEdges,      // i/o: List of edges to be checked
                                                SmTArray<ULONG>    & rChainStart) // out: 
{
    // No work
    if ( rEdges.GetSize() == 0)
    { 
        rChainStart.ReSet();
        return SM_SUCCESS;
    }

    // Locals
    ULONG               lChainStart = 0;
    SmTArray<SmEdge*>   sUnusedEdges( rEdges ), sChains; 
    SmEdge            * pEdge = sUnusedEdges[0];         // Arbitrary starting edge
    SmVertex          * pVertex, *pStartVertex;
    pEdge->GetVertices( pStartVertex, pVertex );         // Arbitrary assignments

    // Reset input
    rEdges.ReSet();
    rChainStart.ReSet();
    rChainStart.Add( 0 );

    // Check the StartVertex for too many chain Edges
    SmTArray<SmEdge*> sLocalEdges;            // Edges connected to pVertex
    SmTArray<SmEdge*> sConnectedEdges;        // Edges in rEdges connected to pVertex
    pStartVertex->GetEdges( sLocalEdges );
    sUnusedEdges.FindCommonElements( sLocalEdges, sConnectedEdges );
    if ( sConnectedEdges.GetSize() > 2 )
    { SER_MSG( SM_ERR, _T( "Edge chains cannot intersect (e.g., no figure-8 allowed)" ) ); }

    // Work through edges until none are left
    while ( sUnusedEdges.GetSize() > 0 )
    {
        ULONG             lEdgeID = 0;  // ID of pEdge in varying SmTArray<SmEdge*> 
        lChainStart++;

        // Get edges connected to this vertex
        pVertex->GetEdges( sLocalEdges );          

        sLocalEdges.FindElement( pEdge, lEdgeID );
        sLocalEdges.RemoveAt( lEdgeID );

        // Find Edges to be chained that connect to pVertex
        sUnusedEdges.FindCommonElements( sLocalEdges, sConnectedEdges );

        if ( sConnectedEdges.GetSize() > 1 )
        { SER_MSG( SM_ERR, _T( "Edge chains cannot intersect (e.g., no figure-8 allowed)" ) ); }

        // Remove Edge from list sUnusedEdges
        sUnusedEdges.FindElement( pEdge, lEdgeID );
        sUnusedEdges.RemoveAt( lEdgeID );

        // Add to output
        rEdges.Add( pEdge );

        // Update edge and vertex for the next iteration
        if ( sConnectedEdges.GetSize() == 1 ) // found one other loop edge on this vertex
        {
            pEdge = sConnectedEdges[0];
            pVertex = pEdge->GetOtherVertex( pVertex );

            continue;
        }

        // Start a new chain if needed
        if (sUnusedEdges.GetSize() > 0 )
        {
            pEdge = sUnusedEdges[0];
            pEdge->GetVertices( pStartVertex, pVertex );
            rChainStart.Add( lChainStart );

            // Check the StartVertex for too many chain Edges
            pStartVertex->GetEdges( sLocalEdges );
            sUnusedEdges.FindCommonElements( sLocalEdges, sConnectedEdges );
            if ( sConnectedEdges.GetSize() > 2 )
            { SER_MSG( SM_ERR, _T( "Edge chains cannot intersect (e.g., no figure-8 allowed)" ) ); }
        }
    }

    return( SM_SUCCESS );

} // end SmTopologyTraverser::FixEdgeChainList

/*******************************************************************//**
PURPOSE: Traverse an array of edges to determine if they form a 
         single closed loop, with no spurious edges.

NOTES:   If the loop is, e.g., a figure-8 then rIsClosed == FALSE.
         If there is a closed loop with an unconnected edge, or an 
         edge sharing only one vertex with the loop, then 
         rIsClosed == FALSE.
***********************************************************************/
SmStatus SmTopologyTraverser::EdgeLoopTraversal(const SmTArray<SmEdge*>  & rEdges,    // in : List of edges to be checked
                                                      SmBoolean          & rIsClosed, // out: TRUE  if rEdges forms a closed loop 
                                                      SmTArray<ULONG>    * pOrder)    // out: Optional ordering of edges
{
  // Locals
  SmTArray<SmVertex*> sVertices;                       // Unique vertices used by rEdges
  SmTArray<SmEdge*>   sUnusedEdges(rEdges);            // List of edges, diminished as we work through loop
  SmEdge*             pEdge = rEdges[0];               // Arbitrary starting edge
  SmVertex*           pVertex = pEdge->GetEndVertex(); // Arbitrary starting vertex
  
  // Start collecting data
  pEdge->GetVertices(sVertices);
  if ( pOrder != NULL ) { pOrder->Add(0); }

  // Quick test for a single closed loop edge. Single vertex implies closed.
  if ( sUnusedEdges.GetSize() == 1 )
    {
      if ( sVertices.GetSize() == 1 )
        {
          rIsClosed = TRUE;
          return(SM_SUCCESS);
        }
      else
        { 
          rIsClosed = FALSE;
          return(SM_SUCCESS); 
        }
    }

  // Only want one vertex, so when loop closes we still add unique
  sVertices.ReSet();
  sVertices.Add(pVertex);

  // Next, work through edges until one is left
  while (sUnusedEdges.GetSize() > 1)
    {
      ULONG             lEdgeID = 0;            // ID of pEdge in varying SmTArray<SmEdge*>
      SmTArray<SmEdge*> sLocalEdges;            // Edges connected to pVertex
      SmTArray<SmEdge*> sConnectedEdges;        // Edges in rEdges connected to pVertex

      pVertex->GetEdges(sLocalEdges);           // Get edges connected to this vertex

      sLocalEdges.FindElement(pEdge, lEdgeID);
      sLocalEdges.RemoveAt(lEdgeID);

      sUnusedEdges.FindCommonElements(sLocalEdges, sConnectedEdges);

      if ( sConnectedEdges.GetSize() == 1 ) // found one other loop edge on this vertex
        {
          // Remove Edge from list sUnusedEdges
          sUnusedEdges.FindElement(pEdge, lEdgeID);
          sUnusedEdges.RemoveAt(lEdgeID);

          // Update edge and vertex for the next iteration
          pEdge = sConnectedEdges[0];
          pVertex = pEdge->GetOtherVertex(pVertex); 

          rEdges.FindElement(pEdge, lEdgeID);
          if ( pOrder != NULL ) { pOrder->Add(lEdgeID); }

          // repeated vertices here == multiple closed loops
          if ( !sVertices.AddUnique(pVertex) )
            {
              rIsClosed = FALSE;
              return(SM_SUCCESS);
            }
        }
      else // Loop is broken
        {
          rIsClosed = FALSE;
          return(SM_SUCCESS);
        }
    }

  // Arrive here when all previous edges were connected end-to-end with no repeated vertices
  // If the last vertex is the first vertex, then the loop is closed.
  if ( pVertex == rEdges[0]->GetStartVertex() )
    {
      rIsClosed = TRUE;
      return(SM_SUCCESS);
    }

  rIsClosed = FALSE;
  return(SM_SUCCESS);
  
} // end SmTopologyTraverser::EdgeLoopTraversal

/*******************************************************************//**
PURPOSE: Traverse all unmarked faceuses connected to Start Faceuse through
         unmarked edgeuse/faceuse connections 

NOTES:
    0. considered open when pFaceuseArg->Mate is on the closure list
    1. if pFaceuseArg is marked on entry, nothing happens 
    2. Marks pFaceuseArg and all unmarked Faceuses connected to it.
    3. Places all newly marked faceuses in pOptFaceuses when given. 
    4. To use this standalone pass in an SmNewMarkAndLock object 
         example: 
           SmNewMarkAndLock sMarkLock(SM_MT_ALLMARKS) ;
           FaceuseTraversal(pFaceuse, bIsClosed, pOptFaceuses, sMarkLock) ;
***********************************************************************/
SmStatus SmTopologyTraverser::FaceuseTraversal
  (SmFaceuse            * pFaceuseArg,  // in : target faceuse (gets marked)                                      
   SmBoolean            & rbIsClosed,   // out: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't                                     
   SmTArray<SmFaceuse*> * pOptFaceuses, // out: List of all connected faces (get marked), NULL to ignore.
   SmNewMarkAndLock     & rMark)        // in : specify mark for target objects (not incremented)
{   
  // initialize outputs
  rbIsClosed = TRUE;
  if ( pOptFaceuses != NULL ) { pOptFaceuses->ReSet(); }

#ifdef SM_DEBUG_CODE
  SmBrep *pBrep = pFaceuseArg->GetBrep();
#endif

  // initialize a recursion stack of Faceuses
  SmTArray<SmFaceuse*> sStack;
  sStack.Add(pFaceuseArg);
  
  SmTArray<SmLoopuse*> sLoopuses;
  SmTArray<SmEdgeuse*> sEdgeuses;
  
  // while there are faces on the stack
  while (sStack.GetSize() > 0) 
    {
      // pop the recursion stack
      SmFaceuse *pFaceuse = sStack.GetLast();
      sStack.RemoveLast();

      // skip faceuses already marked
      if (!pFaceuse || pFaceuse->IsMarked(rMark.GetMarkType())) 
        { continue; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      if ( FALSE ) {
          smgfx_Erase();
          if ( pBrep ) {
              smgfx_SetLook( 1,2, 0,0,0 ); pBrep->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
      }
      smgfx_SetLook( 1,2, 0,0,0 ); pFaceuse->GetFace()->DrawUV(5,5); sm_GraphicsLoop();
      smgfx_SetLook( 2,3, 0,0,1 ); pFaceuse->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

      // when the faceuse's mate matches the startFaceuse - we know its an open shell
      if ( pFaceuse->GetMate() == pFaceuseArg) 
        {
          rbIsClosed = FALSE;

          // when there is no request to gather connected faceuses - were done
          if(pOptFaceuses == NULL) 
            { return SM_SUCCESS; }

        } // end open shell test
      
      // accumulate connected faceuses
      if (pOptFaceuses) { pOptFaceuses->Add(pFaceuse); }
      
      // mark the faceuse
      pFaceuse->Mark(rMark.GetMarkType());

      // for every faceuse->loopuse
      pFaceuse->GetLoopuses(sLoopuses);
      for (ULONG j=0; j<sLoopuses.GetSize(); j++) 
        {
          SmLoopuse *pLU = sLoopuses[j];

          // for every faceuse->loopuse->edgeuse
          pLU->GetEdgeuses(sEdgeuses);
          for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
            {
              SmEdgeuse *pEU = sEdgeuses[i];

              // add the edgeuse's unmarked radial neighbor's faceuse to the stack
              SmEdgeuse *pEURadial = pEU->GetRadial();
              SmFaceuse *pFURadial = pEURadial->GetFaceuse();
              if(!pFURadial->IsMarked(rMark.GetMarkType()))
                { sStack.Add(pFURadial) ; }


#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              sm_GraphicsLoop();
              if ( FALSE ) {
                  smgfx_Erase();
                  if ( pBrep ) {
                      smgfx_SetLook( 1,2, 0,0,0 ); pBrep->Draw(TRUE); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                  }
              }
              smgfx_SetLook( 2,4, 0,1,1 ); pEU->Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 0,1,0 ); pEURadial->Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 1,2, 0,0,1 ); pFURadial->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
            } // end iter every faceuse->loopuse->edgeuse
        } // end iter every faceuse->loopuse
    } // end while faceuses left on the recursion stack

  return SM_SUCCESS;

} // end SmTopologyTraverser::FaceuseTraversal

/*******************************************************************//**
PURPOSE: Transfer ShellToRemove Children to ShellToReplace
  so that ShellToRemove is empty and is prepared to be removed
  from the Topology graph and deleted.

NOTES: 
  Move all edgeuses and faceuses from shell being removed 
  to shell being saved 
    - setting all edgeuse->m_pSorLU pointers = pShellToReplace
    - setting all  
***********************************************************************/
SmStatus SmTopologyTraverser::RemoveShell
  (SmShell *pShellToRemove,    // in : target shell being removed
   SmShell *pShellToReplace)   // in : owner shell to contain removeShell children
{
  SER(pShellToRemove->TransferChildrenTo(pShellToReplace,NULL));
  return SM_SUCCESS;

} // end SmTopologyTraverser::RemoveShell

/*******************************************************************//**
PURPOSE: Replace the shells of all faceuses which can be traversed 
    by connectivity through edgeuses (not vertexuses)

NOTES:  rMark value incremented 
***********************************************************************/
SmStatus SmTopologyTraverser::FaceuseShellReplacement
  (SmFaceuse            * pStartFaceuse,       // in : target faceuse (gets marked)
   SmShell              * pNewShell,           // in : new Shell to hold connected faceuses
   SmTArray<SmFaceuse*> & rReplacedFaceuses,   // in : list of faceuses moved to NewShell (get marked)
   SmNewMarkAndLock     & rMark)               // in : specify mark for target objects (incremented)
{
  rMark.NewMark();
  SmBoolean bClosed;

  // get all connected faceuses starting with StartFaceuse
  SER(FaceuseTraversal(pStartFaceuse,      // in : target faceuse (gets marked)                              
                       bClosed,            // out: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't                                           
                       &rReplacedFaceuses, // out: List of all connected faces (get marked), NULL to ignore.
                       rMark));            // in : specify mark for target objects (not incremented)

  // for every connected faceuse
  for (ULONG ii=0; ii<rReplacedFaceuses.GetSize(); ii++) 
    {
      SmFaceuse *pFU       = rReplacedFaceuses[ii];
      SmShell   *pOldShell = pFU->GetShell();
      if ( pNewShell != pOldShell )
        {
          pOldShell->Remove(pFU);
          pNewShell->PostInsert(pFU);
        }
    } // end iter every faceuse

  return SM_SUCCESS;

} // end SmTopologyTraverser::FaceuseShellReplacement

/*******************************************************************//**
PURPOSE: Mark all vertexuses, vertices, faceuses, and faces connected
   to StartVertexuse through the target shell with the context's 
   current mark value.  
   Optionally load faceuses and edgeuses that get marked into a list.

NOTES: 
  NOTE that the shell's edgeuse and edges do not get 
  marked - but that they can be placed into the optional edgeuse's list.

***********************************************************************/
static SmStatus sm_ShellConnectivity
  (SmVertexuse          * pStartVertexuse,       // in : target vertex  (gets marked)
   SmShell              * pShellToTraverse,      // in : target shell
   SmTArray<SmEdgeuse*> * pOptEdgeusesTouched,   // out: optional list of edgeuses in this shell (not marked)
   SmTArray<SmFaceuse*> * pOptFaceusesTouched,   // out: optional list of faces in this shell (get marked)
   SmNewMarkAndLock     & rMark)                 // in : specify mark for target objects  (not incremented)
{

  // mark the start vertex
  pStartVertexuse->Mark(rMark.GetMarkType());

  // init a recursion stack with the startVertexuse
  SmVertexuse *sVUStackData[1024];
  SmTArray<SmVertexuse*> sRecursionStack(1024,sVUStackData);
  sRecursionStack.Add(pStartVertexuse);

  // local lists
  SmVertexuse * sVUData[256];
  SmEdgeuse   * sEUData[256];
  SmLoopuse   * sLUData[32];
  SmFaceuse   * sFUData[32];
  SmVertex    * sVData[16];
  SmTArray<SmVertexuse*> sVertexuses(256,sVUData);
  SmTArray<SmEdgeuse*>   sEdgeuses(256,sEUData);
  SmTArray<SmLoopuse*>   sLoopuses(32,sLUData);
  SmTArray<SmFaceuse*>   sFaceuses(32,sFUData);
  SmTArray<SmVertex*>    sVertices(16,sVData);

  // while the recursion stack contains vertexuses objects
  while (sRecursionStack.GetSize() > 0) 
    {
      // get the current recusionStack target vertexuse - pop the stack
      SmVertexuse *pCurrVertexuse = sRecursionStack.GetLast();
      sRecursionStack.RemoveLast();

      // refresh the Faceuse list
      sFaceuses.ReSet(); 
      
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          sm_GraphicsLoop();
          pCurrVertexuse->GetVertex()->Draw();
          smgfx_SetColor(1,0,0);
          smgfx_SetLineWidth(3);
          pCurrVertexuse->GetEdgeuse()->GetEdge()->Draw();
          sm_GraphicsLoop();
        }
#endif

      // If the vertex is part of a face then add that face to the
      // potential propagation list. 

      // Set pFU with any face to which Vertexuse is connected
      // When vertexuse is connected to Edgeuse 
      //   - see if mate's vertexuse should be added to the recursion list 
      SmFaceuse *pFU = NULL;
      if (pCurrVertexuse->IsEdgeVertexuse()) 
        {
          // Propagate results to other end of edgeuse of vertexuse
          SmEdgeuse *pEU = pCurrVertexuse->GetEdgeuse(); NER(pEU);
          if (pOptEdgeusesTouched) pOptEdgeusesTouched->Add(pEU);

          // when the pEU Mate is in the target Shell
          SmEdgeuse *pEUMate = pEU->GetMate(); NER(pEUMate);
          if (pEUMate->GetShell() == pShellToTraverse) 
            {
              // mark and place on the recursion list its Vertexuse 
              SmVertexuse *pVUMate = pEUMate->GetVertexuse(); NER(pVUMate);
              if (!pVUMate->IsMarked(rMark.GetMarkType())) 
                {
                  pVUMate->Mark(rMark.GetMarkType());
                  sRecursionStack.Add(pVUMate);
                }
            }
          // Now get vertexuse->edgeuse->faceuse
          if (pEU->IsLoopEdgeuse()) 
            {
              pFU = pCurrVertexuse->GetFaceuse();
            }
        }
      else if (pCurrVertexuse->IsLoopVertexuse()) 
        { 
          // get vertexuse->faceuse
          pFU = pCurrVertexuse->GetFaceuse(); 
        }

      // when the Vertexuse is connected to a Faceuse (could be connected to a shell)
      if (pFU) 
        {
          // mark the Faceuse and add it to the optional Faceuse list
          if (!pFU->IsMarked(rMark.GetMarkType())) 
            {
              pFU->Mark(rMark.GetMarkType());
              if (pOptFaceusesTouched) pOptFaceusesTouched->Add(pFU);
            }
          // mark the Face and add its faceuse to the local Faceuse's list
          if (!pFU->GetFace()->IsMarked(rMark.GetMarkType())) 
            {
              pFU->GetFace()->Mark(rMark.GetMarkType());
              sFaceuses.Add(pCurrVertexuse->GetFaceuse());
            }
        }

      // Now add all other vertexuses attached to this vertexuses->vertex,
      // that also belong to the target shell, to the recursion stack.
      SmVertex *pV1 = pCurrVertexuse->GetVertex();
      if (!pV1->IsMarked(rMark.GetMarkType())) 
        {
          // mark the CurrVertexuse->Vertex 
          pV1->Mark(rMark.GetMarkType());

          // place all unmarked vertex->vertexuses attached to target shell on the recursion stack
          pV1->GetVertexuses(sVertexuses);
          for (ULONG i=0; i<sVertexuses.GetSize(); i++) 
            {
              SmVertexuse *pVU = sVertexuses[i];
              if (!pVU->IsMarked(rMark.GetMarkType()) && pVU->GetShell() == pShellToTraverse) 
                {
                  pVU->Mark(rMark.GetMarkType());
                  sRecursionStack.Add(pVU);
                }
            }
        }

      // Now pick up vertexuses of faceuses which are on the shell
      // This is required because the edges do not connect across a face
      // from one loop to the next.

      // for every Faceuse list member (expected only to be the faceuse attached to this vertex)
      for (ULONG jj=0; jj<sFaceuses.GetSize(); jj++) 
        {
          SmFaceuse *pFaceU = sFaceuses[jj];

          // for every Faceuse->Loopuse
          pFaceU->GetLoopuses(sLoopuses);
          for (ULONG i=0; i<sLoopuses.GetSize(); i++) 
            {
              SmLoopuse *pLU2 = sLoopuses[i];

              // for every Faceuse->Loopuse->Vertexuse
              pLU2->GetVertexuses(sVertexuses);
              for (ULONG j=0; j<sVertexuses.GetSize(); j++) 
                {
                  SmVertexuse *pVU = sVertexuses[j];
                  // for unprocessed vertices attached to the target shell
                  if (!pVU->IsMarked(rMark.GetMarkType()) && pVU->GetShell() == pShellToTraverse) 
                    {
                      // mark the vertexuse and add it to the recursion stack
                      pVU->Mark(rMark.GetMarkType());
                      sRecursionStack.Add(pVU);
                    } // end not marked and attached to target shell check
                } // end iter every Faceuse->Loopuse->Vertexuse
            } // end iter every Faceuse->Loopuse
        } // end iter every Faceuse list member
    }  // end while recursion stack

  return SM_SUCCESS;

} // end sm_ShellConnectivity

/*******************************************************************//**
PURPOSE: Traverse a shell through vertex/face connections marking
    every vertexuse, vertex, faceuse, and face connected to the StartVertex
    through the ShellToTraverse.
    Optionally collect Shell edgeuses and/or faceuses touched during the traversal.  

NOTES: Please note that this is not an enclosure traversal 
    routine (see ClosureTraversal). After this call you can check a 
    given item to see if it is marked (e.g. pVU->IsMarked(eMarkType))
    to see if it was touched during this traversal.

    side-effect: increment rMark mark value
***********************************************************************/
SmStatus SmTopologyTraverser::ShellTraversal
  (SmVertex *pStartVertex,                      // in : target vertex
   SmShell  *pShellToTraverse,                  // in : target shell
   SmTArray<SmEdgeuse*> * pOptEdgeusesTouched,  // out: optional list of edges in shell (not marked)
   SmTArray<SmFaceuse*> * pOptFaceusesTouched,  // out: optional list of faces in shell (get marked)
   SmNewMarkAndLock     & rMark)                // in : specify mark for target objects (incremented)
{
  // init outputs
  if (pOptEdgeusesTouched) pOptEdgeusesTouched->ReSet();
  if (pOptFaceusesTouched) pOptFaceusesTouched->ReSet();
  

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;

SmBrep* pBrep = pStartVertex->GetBrep();

  if (bDebugMe) 
    {
      SER(pBrep->ValidatePointers());
      smgfx_Erase();
      smgfx_SetLineWidth(1.0);
      smgfx_SetColor(0,0,0);
      pBrep->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // increment the mark value
  rMark.NewMark();

  // get all vertex->vertexuses
  SmVertexuse * sVUData[32];
  SmTArray<SmVertexuse*> sVertexuses(32,sVUData);
  pStartVertex->GetVertexuses(sVertexuses);

  // for every vertexuse
  for (ULONG i=0; i<sVertexuses.GetSize(); i++) 
    {
      SmVertexuse *pVU = sVertexuses[i];

      // when the Vertexuse->Shell is the target Shell
      if (pVU->GetShell() == pShellToTraverse) 
        {
          // mark and add all of the shell's connected Faces and Wires to th
          SER(sm_ShellConnectivity(pVU,                 // in : target vertex  (gets marked)
                                   pShellToTraverse,    // in : target shell
                                   pOptEdgeusesTouched, // out: optional list of edgeuses in this shell (not marked)
                                   pOptFaceusesTouched, // out: optional list of faces in this shell (get marked)
                                   rMark));             // in : specify mark for target objects  (not incremented)
          break;
        }
    } // end iter every vertex use

  return SM_SUCCESS;

} // end SmTopologyTraverser::ShellTraversal

static SmBoolean IsOdd( ULONG lNum ) { return ( lNum  / 2 ) * 2  != lNum; }
/*******************************************************************//**
PURPOSE: Determine whether a closed collection of Faceuses forms
         an inner or outer shell.

NOTES: InnerShell = shell is isomorphic to sphere's inside facing shell 
       OuterShell = shell is isomorphic to sphere's outside facing shell

                    +----------------+
                    |                |-> OuterShell
                    |-> InnerShell <-|
       OuterShell <-|                | 
                    +----------------+

       The Faces of the Faceuses must form a closed manifold.
       That is not explicitly checked for, and if not closed, 
       this ray-casting method may get confused by not finding 
       enough ray intersections (the rays run out the cracks)
       or may return an invalid answer. When confused,
       rbIsInner is set to UNSURE and this routine returns SM_ERR.

       IsInnerShell() was originally implemented by offsetting 
       faceuses in both directions and seeing which bounding box was bigger.  
       That failed on a certain case, and prog_test subsequently 
       found counterexamples to any reasonable implementation of 
       both that faceuse-offset boxing method, as well as
       an Edgeuse-convexity algorithm.  [B335]

METHOD ---
   Fire rays in both directions from points in faces, and count intersections.
***********************************************************************/
SmStatus SmTopologyTraverser::IsInnerShell
  ( SmTArray< SmFaceuse* > & rFaceuses,   // in : Facesuses of a closed shell to be checked
    SmBoolean              & rbIsInner )  // out: TRUE = is isomorphic to sphere's inside facing shell 
                                          //      FALSE= is isomorphic to sphere's outside facing shell
{
  // Init output
  rbIsInner = UNSURE;

  ULONG lNumFUs = rFaceuses.GetSize();
  if ( lNumFUs < 1 )
    { return SM_ERR; }

  ULONG lNumSamples = 3; // # rayfires per source face
  if ( lNumFUs < 6 ) { lNumSamples *= 2; } // More if fewer FUs.
  if ( lNumFUs < 3 ) { lNumSamples *= 2; }

  SmTArray<SmPoint2d> sUVs;
  SmTArray<SmPoint3d> sPtVecs;
  SmExtent2d sTargetDomain;

  SmSolutionArray sSolutions;

  // If we get enough hits without any misses,
  // we can assume it's inner.
  ULONG lRayHitCount = 0;

  // Also fire rays backwards.  Since it takes only a miss or two
  // to decide 'out', this should speed up 'in' cases.
  ULONG lBackHitCount = 0;

  SmStatus eStat;
  ULONG ii, jj, kk, ll;

  // Fire a ray from points in each Face.
  // Note, this will usually find an answer in the first face in this outer loop.
  for ( ii=0; ii<lNumFUs; ii++ )
  {
      SmFaceuse * pSourceFU   = rFaceuses[ii];
      SmFace    * pSourceFace = pSourceFU->GetFace();

      SER( pSourceFace->GetPointsInFace( lNumSamples, sUVs, sPtVecs ));
      ULONG lNumRays = sPtVecs.GetSize() / 2;

      // Flip rays for Opposite Faceuses.
      if ( pSourceFU->GetOrientation() == SM_OT_OPPOSITE )
      {
          for ( jj=0; jj<lNumRays; jj++ )
            { sPtVecs[2*jj+1] *= -1; }
      }

      // For each ray, fire at all Faces:
      // Note that we have to test against all Faces before deciding anything.
      // Again, this will usually find an answer on the first ray.
      for ( jj=0; jj<lNumRays; jj++ )
      {
          lRayHitCount = lBackHitCount = 0; // Reset for each ray.

          // Fire this ray at all Faces.

          for ( kk=0; kk<lNumFUs; kk++ )
          {
              ULONG lRayHitFace  = 0;
              ULONG lBackHitFace = 0;

              SmFaceuse * pTargetFU   = rFaceuses[kk];
              SmFace    * pTargetFace = pTargetFU->GetFace();

              sTargetDomain = pTargetFace->GetUVDomain();
              double dTol   = pTargetFace->GetTolerance();

              // Fire a ray from this Faceuse.
              eStat = pTargetFace->GlobalLineIntersect( sTargetDomain,
                                                        sPtVecs[2*jj], sPtVecs[2*jj+1],
                                                        NULL, FALSE, dTol,
                                                        sSolutions );

              if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
                { continue; }

              // count all forward and backward intersections, skip self intersection
              for ( ll=0; ll<sSolutions.GetSize(); ll++ )
                {
                  SmSolution & rSol = sSolutions[ll];
                  double dLineParam = rSol.m_vStart[0]; // Vectors are unit, so line param is dist.
                  if      ( dLineParam < -dTol ) { lBackHitFace++; }
                  else if ( dLineParam >  dTol ) { lRayHitFace++; }

                } // end for each solution

              lRayHitCount  += lRayHitFace;
              lBackHitCount += lBackHitFace;

          } // end for each target faceuse with this ray

          // Done firing this ray at all Faces.  Check hits.

          // A simple yes/no is good enough.
          if ( lRayHitCount==1 && lBackHitCount==0 ) { rbIsInner = TRUE; 
                                                       return SM_SUCCESS; 
                                                     }
          if ( lRayHitCount==0 && lBackHitCount==1 ) { rbIsInner = FALSE; 
                                                       return SM_SUCCESS; 
                                                     }

          // An even number of hits should mean Outside, and odd should mean Inside.
          SmBoolean bRayCountOdd  = IsOdd( lRayHitCount  );
          SmBoolean bBackCountOdd = IsOdd( lBackHitCount );

          if ( bRayCountOdd==TRUE && bBackCountOdd==FALSE ) { rbIsInner = TRUE; 
                                                              return SM_SUCCESS; 
                                                            }

          if ( bRayCountOdd==FALSE && bBackCountOdd==TRUE ) { rbIsInner = FALSE; 
                                                               return SM_SUCCESS; 
                                                            }

          // Couldn't find anything conclusive, try the next ray.

      } // end for each ray-fire from this pSourceFace
  } // end for each source faceuse

  // Shouldn't reach here.
  WARN( _T("Warning: Possible problem in SmTopologyTraverser::IsInnerShell()" ));

  if ( lRayHitCount > lBackHitCount )
  {
      if ( lBackHitCount == 0 && IsOdd( lRayHitCount ) )
        { rbIsInner = TRUE; return SM_SUCCESS; }
  }
  if ( lBackHitCount > lRayHitCount )
  {
      if ( lRayHitCount == 0 && IsOdd( lBackHitCount ) )
        { rbIsInner = FALSE; return SM_SUCCESS; }
  }

  // Really shouldn't reach here.
  WARN( _T("Error: SmTopologyTraverser::IsInnerShell() returning Error" ));

  rbIsInner = UNSURE;
  return SM_ERR;

} // end SmTopologyTraverser::IsInnerShell

/************************* FACE NUMBERING BASED ON CREASE ANGLE ***********************/

// First some local static variables and routines.

// this is really a constant -  unaffected by threads
     // used as attribute id to define a FaceNumber
     // faces that are un-numbered have no attribute or attribute value of -1
static SM_THREAD_LOCAL int SM_AIH_FACE_NUMBER = 9518; 

static int GetFaceNumber(SmFace *pF)
{
    // return face number for this face or -1 if none
    SmLongAttribute *Attribute =(SmLongAttribute *) pF->FindAttribute(SM_AIH_FACE_NUMBER);
    if (Attribute)
    {
        return Attribute->GetValue();
    }
    
    return (-1);
} // end static GetFaceNumber

static void PutFaceNumber(SmContext &rContext, SmFace *pF, int Num)
{   
    // If there is not an attribute, make one and set it.
    SmLongAttribute *Attribute =(SmLongAttribute *) pF->FindAttribute(SM_AIH_FACE_NUMBER);
    if (!Attribute)
        Attribute = new(rContext) SmLongAttribute(SM_AIH_FACE_NUMBER, SM_AB_COPY);  
    else
        pF->RemoveAttribute(Attribute, TRUE); // do not delete it
         
    Attribute->SetValue(Num);
    pF->AddAttribute(Attribute);
    
    return;
} // end static PutFaceNumber

static SmFace *FindOtherFace(SmEdge *pE, SmFace *pF)
{
    // find other face across edge
    SmTArray < SmFace*> rFaces; 
    pE->GetFaces(rFaces);
    if (rFaces.GetSize() < 2)
        return NULL;  // lamina edge
    if (rFaces.GetSize() > 2)
        return (NULL); // non-manifold edge too confusing
    SmFace *pF1 = rFaces[0];
    
    return ((pF1 == pF) ? rFaces[1]: pF1);
} // end static FindOtherface

// Return TRUE if computed crease angle exceeds input dAngle (degrees)
// Also return TRUE very short edge, or non-manifold edge
static SmBoolean EdgeCrease
 (SmFace * pF,              // NotUsed: in : pF
 SmEdge  * pE,              // in : 
 SmFace  * pF2,             // NotUsed: in : pF2
 double    dAngle,          // in : 
 double    MinEdgeSizeSq)   // in : 
{
 SM_REF2(pF, pF2) ;
    // check if vertex distance apart is too small to measure crease angle
    if (! pE->IsClosed())  // unless the edge is closed
    {
        SmTArray < SmVertex *> sVertices;
        pE->GetVertices(sVertices);
        SmPoint3d V1 = sVertices[0]->GetPoint();
        SmPoint3d V2 = sVertices[1]->GetPoint();
        if (V1.DistanceBetweenSquared(V2) < MinEdgeSizeSq)
            return (TRUE);
    }
    
   
    SmPoint3d s3UV;
    SmPoint2d s2UV;
    SmPoint3d sNorm0, sNorm1;
    double TValues[5] = { 0.5123, 0.2487, 0.76, 0.043, 0.982 };
    SmTArray < SmEdgeuse*>pEdgeuses;
    pE->GetEdgeuses(pEdgeuses);
    if (pEdgeuses.GetSize() != 4) return TRUE;  // only manifold non-lamina case
    SmEdgeuse *pEU0 = pEdgeuses[0];
    SmEdgeuse *pEU1 = pEdgeuses[2];
    
    SmBSplineCurve* bs0 = NULL;
    pEU0->GetOrCreateUVTrimCurve( bs0 );

    SmBSplineCurve* bs1 = NULL;
    pEU1->GetOrCreateUVTrimCurve( bs1 );

    if( bs0 == NULL || bs1 == NULL )
        return( TRUE );
    
    SmExtent1d sUVIv0 = bs0->GetNaturalInterval();
    SmExtent1d sUVIv1 = bs1->GetNaturalInterval();  
    for (int nt = 0; nt < 5; nt ++)  // 5 sample points
    {
        double t0 = sUVIv0.Evaluate(TValues[nt]);
        bs0->EvaluatePoint(t0, s3UV);
        SmSurface *pSrf0 = pEU0->GetFace()->GetSurface();
        s2UV.x = s3UV.x;  s2UV.y = s3UV.y;
        //SmPoint3d sPnt0;
        //pSrf0->EvaluatePoint(s2UV,sPnt0);
        pSrf0->EvaluateNormal(s2UV, TRUE, TRUE, sNorm0);
        
        double t1 = sUVIv1.Evaluate(TValues[nt]);
        bs1->EvaluatePoint(t1, s3UV);
        SmSurface *pSrf1 = pEU1->GetFace()->GetSurface();
        s2UV.x = s3UV.x;  s2UV.y = s3UV.y;
        //SmPoint3d sPnt1;
        //pSrf1->EvaluatePoint(s2UV,sPnt1);
        pSrf1->EvaluateNormal(s2UV, TRUE, TRUE, sNorm1);        
        double dAngleBetween = 0.0;
        sNorm0.AngleBetween(sNorm1, dAngleBetween);
        double dAngleDeg = dAngleBetween * 180.0 / SM_PI;
        if (dAngleDeg > dAngle)
            return (TRUE);
    }
    
    return (FALSE);
} // end static EdgeCrease


// Recursive Routine to traverse all adjacent faces that pass crease angle test
static void TestAdjacent(SmContext & rContext, SmFace *pF,  
                         int & rlNumberFree, int lNewNumber, int lOldNumber, 
                         double angle, SmTArray <SmFace *> & rFaces, double MinEdgeSizeSq)

{
    SmTArray <SmEdge *> sEdges;
    pF->GetEdges(sEdges);
    for (ULONG ne = 0; ne < sEdges.GetSize(); ne++)
    {
        SmEdge *pE = sEdges[ne];
        
        SmFace *pF2 = FindOtherFace(pE, pF);
        if (pF2 == NULL)  // only 1 face OR non-manifold edge (>2 faces)
            continue;
        if (GetFaceNumber(pF2) > -1)
            continue;
        if (EdgeCrease(pF, pE, pF2, angle, MinEdgeSizeSq))
            continue;
        
        PutFaceNumber(rContext, pF2, lNewNumber);
        rlNumberFree --;
        
        // recurse for each connected face (where crease angle is shallow)
        TestAdjacent(rContext, pF2, rlNumberFree, lNewNumber, lOldNumber, angle, rFaces, MinEdgeSizeSq);
    }
        
    // This face has been processed, remove it from list
    for (ULONG i = 0; i < rFaces.GetSize(); i++)
    {
        if (rFaces[i] == pF)
        {
            rFaces[i] = NULL;
            break;
        }
    }
    return;

} // end static TestAdjacent

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmTopologyTraverser::NumberFacesByCreaseAngle  
(  
  SmBrep   *pBrep,
  double   angle,         // Max angle between faces for same face-number
  double   MinEdgeSize,   // Dont test creases less than this length
  int      FaceNumberId,  // Attribute number to use for facenumbering (9518)
  int      *pNewNumber    // in:  Next Number to use for next face group  > -1
                          // out: next number updated for every new face number created 
)      
{
    ULONG nf, i;
    int lNumberFree = 0;
    int lNewNumber = *pNewNumber;

    // Set local statics from these arguments
    double MinEdgeSizeSq = MinEdgeSize*MinEdgeSize;
    SM_AIH_FACE_NUMBER  = FaceNumberId;  // 9518
   
    SmContext *pContext = (SmContext *) pBrep->GetContext();
    SmTArray < SmFace *> sFaces;
    pBrep->GetFaces(sFaces);
    
    // Establish all chains from any existing numbers
    for (nf =0; nf < sFaces.GetSize(); nf++)
    {
        SmFace *pF = sFaces[nf];
        if (pF == NULL)
            continue;
        int lOldNumber = GetFaceNumber(pF);
        if (lOldNumber == -1)
        {
            lNumberFree++;
            continue;
        }
        TestAdjacent(*pContext, pF, lNumberFree, lOldNumber, lOldNumber, angle, sFaces, MinEdgeSizeSq);
    }
    
    if (lNumberFree == 0)
        return (SM_SUCCESS);
    
    // Start off with New Numbers
    for (nf =0; nf < sFaces.GetSize(); nf++)
    {
        SmFace *pF = sFaces[nf];
        if (pF == NULL)
            continue;
        int lOldNumber = GetFaceNumber(pF);
        if (lOldNumber == -1)
        {
            PutFaceNumber(*pContext, pF, lNewNumber);
            lNumberFree--;
            TestAdjacent(*pContext, pF, lNumberFree, lNewNumber, lOldNumber, angle, sFaces, MinEdgeSizeSq);
            lNewNumber++;
        }
    }
    
   // Check results.
    SM_ASSERT(lNumberFree == 0);
    for (i = 0; i < sFaces.GetSize(); i++)
    {
        SM_ASSERT(sFaces[i] == NULL);
    }


    // Check results: There should be no un-numbered faces
    pBrep->GetFaces(sFaces);
    for (i = 0; i < sFaces.GetSize(); i++)
    {
        SmFace *pF = sFaces[i];
        SM_ASSERT(sFaces[i] != NULL);
        SmLongAttribute *Attribute =(SmLongAttribute *)pF->FindAttribute(SM_AIH_FACE_NUMBER);
        SM_ASSERT(Attribute != NULL);
        if (Attribute)
        {
            SM_ASSERT(Attribute->GetValue() < lNewNumber);
        }
    }


    // set the new higher max facenumber
    *pNewNumber= lNewNumber;
    return (SM_SUCCESS);

} // end NumberFacesByCreaseAngle

/************************* end FACE NUMBERING BASED ON CREASE ANGLE ***********************/


/*******************************************************************//**
PURPOSE: Walk the topology graph down from the input object adding
 pointers to every Topology object (SmRegion, SmShell, SmFace, SmLoop, SmEdge, SmVertex)
 encountered along the way to the output arrays. 

NOTES: 
 When eMarkType == SM_MT_NOMARK - All objects under rObject are added to output. 
 When eMarkType != SM_MT_NOMARK - All unmarked objects under rObject are added to output.
                                  Note control the mark state prior to this call with
                                   { SmNewMarkAndLock sMarkLock(pContext) ;             // increment and lock any unlocked mark
                                     SmMarkType eMarkType = sMarkLock.GetMarkType() ;   // fetch the mark that was locked
                                     sTopologyTraverser.GetSubTopology(...,eMarkType) ; // Tell GetSubTopology to use eMarkType marks
                                   }
 0. The input object is not added to any of the output lists.
 1. The output arrays may be preloaded with entities.
       when bReSetArrays == TRUE  - ReSetting the output arrays loses these entries.
       when bReSetArrays == FALSE - These entries remain in the output lists.
          and when eMarkType == SM_MT_NOMARK, all subTopology under these entities are found 
                                              and added to the unique entity output arrays.
          and when eMarkType != SM_MT_NOMARK, only the subTopology under the unmarked entities are
                                              found and added to the unique entity output arrays.
 2. The output arrays are unique, that is the objects on them are listed only once.
 3. The output arrays are optional.  Set any  output array pointer to NULL to ignore them.
 4. The input object can be one of SmBrep, SmRegion, SmShell, SmFace, SmLoop, SmEdge, SmVertex
      but cannot be SmFaceuse, SmLoopuse, SmEdgeuse, SmVertexuse.
    When the input argument is of an invalid type the output lists are
    cleared (bReSetArrays == TRUE) or left alone (bReSetArrays==FALSE) and SM_ERR is returned.
 5. No objects are under an SmVertex, one can call this function with an input vertex but
    no topology objects will be added to the output

***********************************************************************/
SmStatus SmTopologyTraverser::GetSubTopology
 (const SmTopology     &rObject,         // in : A member of an existing tree (not a use object)
  SmTArray<SmRegion *> *pRegions,        // i/o: opt unique regions list,  NULL to ignore, default:[NULL]
  SmTArray<SmShell  *> *pShells,         // i/o: opt unique shells list,   NULL to ignore, default:[NULL]
  SmTArray<SmFace   *> *pFaces,          // i/o: opt unique faces list,    NULL to ignore, default:[NULL]
  SmTArray<SmLoop   *> *pLoops,          // i/o: opt unique loops list,    NULL to ignore, default:[NULL]
  SmTArray<SmEdge   *> *pEdges,          // i/o: opt unique edges list,    NULL to ignore, default:[NULL]
  SmTArray<SmVertex *> *pVertices,       // i/o: opt unique vertices list, NULL to ignore, default:[NULL]
  SmBoolean             bReSetArrays,    // in : TRUE = reset arrays, FALSE = accumulate within arrays, Default:[TRUE]
  SmMarkType            eMarkType)       // in : mark type to check, SM_MT_NOMARK = no mark checks,
                                         //      not incremented, default:[SM_MT_NOMARK]
{
  // no work - no output requested
  if(   pRegions  == NULL
     && pShells   == NULL 
     && pFaces    == NULL 
     && pLoops    == NULL 
     && pEdges    == NULL  
     && pVertices == NULL) 
    { return(SM_SUCCESS) ; }

  // locals
  ULONG ii, jj ;
  SM_TYPE   lType    = rObject.GetType() ;
  SmBoolean bMarking = eMarkType != SM_MT_NOMARK ;
  
  // check for valid input object type
  SmBoolean bValidType = (   lType == SmBrep_TYPE
                          || lType == SmRegion_TYPE
                          || lType == SmShell_TYPE 
                          || lType == SmFace_TYPE  
                          || lType == SmLoop_TYPE  
                          || lType == SmEdge_TYPE  
                          || lType == SmVertex_TYPE) ;

  // clear output when asked, for Breps, and bad inputs
  if(bReSetArrays || lType == SmBrep_TYPE || !bValidType)
    {
      if(pRegions ) pRegions ->ReSet() ;
      if(pShells  ) pShells  ->ReSet() ;
      if(pFaces   ) pFaces   ->ReSet() ;
      if(pLoops   ) pLoops   ->ReSet() ;
      if(pEdges   ) pEdges   ->ReSet() ;
      if(pVertices) pVertices->ReSet() ;
    }

  // error - not a valid input object type
  if(!bValidType)
    { return(SM_ERR) ; }

  // special case SmBreps - they have stored lists of contained objects
  if(lType == SmBrep_TYPE)
    {
      // set up working arrays - needed for skipped output arrays
      SmTArray<SmRegion *> sRegions;
      // SmTArray<SmRegion *> sRegions,  *pMyRegions  = pRegions  ? pRegions  : &sRegions ;
      SmTArray<SmShell  *> sShells;
      // SmTArray<SmShell  *> sShells,   *pMyShells   = pShells   ? pShells   : &sShells ;
      SmTArray<SmFace   *> sFaces,    *pMyFaces    = pFaces    ? pFaces    : &sFaces ;
      SmTArray<SmLoop   *> sLoops,    *pMyLoops    = pLoops    ? pLoops    : &sLoops ;
      SmTArray<SmEdge   *> sEdges;
      // SmTArray<SmEdge   *> sEdges,    *pMyEdges    = pEdges    ? pEdges    : &sEdges ;
      SmTArray<SmVertex *> sVertices;
      // SmTArray<SmVertex *> sVertices, *pMyVertices = pVertices ? pVertices : &sVertices ;

      const SmBrep *pBrep = (const SmBrep *) &rObject ; 

      // use macro to get types
      if(pRegions)  pBrep->GetRegions (*pRegions) ; 
      if(pShells)   pBrep->GetShells  (*pShells) ; 
      if(pFaces || pLoops) pBrep->GetFaces(*pMyFaces) ;
      if(pLoops) { SmTArray<SmLoop *> sFaceLoops ;
                   for(ii=0;ii<pMyFaces->GetSize();ii++)
                    { pMyFaces->GetAt(ii)->GetLoops(sFaceLoops) ;
                      for(jj=0;jj<sFaceLoops.GetSize();jj++)
                        { if(sFaceLoops[jj]) 
                            { pMyLoops->Add(sFaceLoops[jj]) ; }
                        }
                    }
                 }
      if(pEdges)    pBrep->GetEdges   (*pEdges) ; 
      if(pVertices) pBrep->GetVertices(*pVertices) ;
      
      if(eMarkType != SM_MT_NOMARK)
        {
          // for every output array - remove the marked entries
          if(pRegions)  { for(ii=0;ii<pRegions->GetSize();ii++)  
                            { if(pRegions->GetAt(ii)->IsMarked(eMarkType))  { pRegions->SetAt(ii, NULL) ; }
                              else                                          { pRegions->GetAt(ii)->Mark(eMarkType) ; }
                            }
                          pRegions->CompressZeros() ;
                        }                                          
          if(pShells)   { for(ii=0;ii<pShells->GetSize();ii++)   
                            { if(pShells->GetAt(ii)->IsMarked(eMarkType))   { pShells->SetAt(ii, NULL) ; }
                              else                                          { pShells->GetAt(ii)->Mark(eMarkType) ; }
                            }
                          pShells->CompressZeros() ;                                          
                        }                                          
          if(pFaces)    { for(ii=0;ii<pFaces->GetSize();ii++)    
                            { if(pFaces->GetAt(ii)->IsMarked(eMarkType))    { pFaces->SetAt(ii, NULL) ; }
                              else                                          { pFaces->GetAt(ii)->Mark(eMarkType) ; }
                            } 
                          pFaces->CompressZeros() ;                                         
                        }                                          
          if(pLoops)    { for(ii=0;ii<pLoops->GetSize();ii++)    
                            { if(pLoops->GetAt(ii)->IsMarked(eMarkType))    { pLoops->SetAt(ii, NULL) ; }
                              else                                          { pLoops->GetAt(ii)->Mark(eMarkType) ; }
                            }
                          pLoops->CompressZeros() ;                                          
                        }                                          
          if(pEdges)    { for(ii=0;ii<pEdges->GetSize();ii++)    
                            { if(pEdges->GetAt(ii)->IsMarked(eMarkType))    { pEdges->SetAt(ii, NULL) ; }
                              else                                          { pEdges->GetAt(ii)->Mark(eMarkType) ; }
                            }
                          pEdges->CompressZeros() ;                                          
                        }                                          
          if(pVertices) { for(ii=0;ii<pVertices->GetSize();ii++) 
                            { if(pVertices->GetAt(ii)->IsMarked(eMarkType)) { pVertices->SetAt(ii, NULL) ; }
                              else                                          { pVertices->GetAt(ii)->Mark(eMarkType) ; }
                            }
                          pVertices->CompressZeros() ;
                        }                                          

        } // end need to cull marked elems

      // all done
      return(SM_SUCCESS) ;
    } // end lType == SmBrep_TYPE check

  // set a work bit Array - used to quit when work is done
  ULONG lMoreWork =   (pRegions  ? ( 1 << 0) : 0)
                    + (pShells   ? ( 1 << 1) : 0)
                    + (pFaces    ? ( 1 << 2) : 0)
                    + (pLoops    ? ( 1 << 3) : 0)
                    + (pEdges    ? ( 1 << 4) : 0)
                    + (pVertices ? ( 1 << 5) : 0) ;

  // no more work check
  if(lMoreWork == 0) 
    { return(SM_SUCCESS) ; }

  // use marks to prevent duplicate output entries
  SmNewMarkAndLock sMarkLock(rObject.GetContext(),
                             SM_MT_ALLMARKS & (~eMarkType)) ; // increment and lock any unlocked mark not equal to eMarkType
  SmMarkType       eWorkMarkType = sMarkLock.GetMarkType() ;   // fetch the mark that was locked

  SM_ASSERT_MSG(eMarkType != eWorkMarkType, _T("SmTopologyTraverser::GetSubTopology eWorkMarkType is not unique - this is a bug")) // caller forgot to lock the target mark 
  
  // local arrays
  SmTArray<SmRegion *> sRegions,  *pMyRegions = NULL ;
  SmTArray<SmShell  *> sShells,   *pMyShells = NULL;
  SmTArray<SmFace   *> sFaces,    *pMyFaces = NULL;
  SmTArray<SmLoop   *> sLoops,    *pMyLoops = NULL;
  SmTArray<SmEdge   *> sEdges,    *pMyEdges = NULL;
  SmTArray<SmVertex *> sVertices, *pMyVertices = NULL;

  // When marking work in local arrays
  if(bMarking)
    {
      // work in local arrays. Later, copy to output
      if(pRegions ) { pMyRegions  = &sRegions ;  sRegions .Append(*pRegions ) ; for(ii=0;ii<sRegions .GetSize();ii++) { sRegions [ii]->Mark(eMarkType) ; }}
      if(pShells  ) { pMyShells   = &sShells ;   sShells  .Append(*pShells  ) ; for(ii=0;ii<sShells  .GetSize();ii++) { sShells  [ii]->Mark(eMarkType) ; }}
      if(pFaces   ) { pMyFaces    = &sFaces ;    sFaces   .Append(*pFaces   ) ; for(ii=0;ii<sFaces   .GetSize();ii++) { sFaces   [ii]->Mark(eMarkType) ; }}
      if(pLoops   ) { pMyLoops    = &sLoops ;    sLoops   .Append(*pLoops   ) ; for(ii=0;ii<sLoops   .GetSize();ii++) { sLoops   [ii]->Mark(eMarkType) ; }}
      if(pEdges   ) { pMyEdges    = &sEdges ;    sEdges   .Append(*pEdges   ) ; for(ii=0;ii<sEdges   .GetSize();ii++) { sEdges   [ii]->Mark(eMarkType) ; }}
      if(pVertices) { pMyVertices = &sVertices ; sVertices.Append(*pVertices) ; for(ii=0;ii<sVertices.GetSize();ii++) { sVertices[ii]->Mark(eMarkType) ; }}
    }
  else // load up the output directly
    {
      // set up working arrays - needed for skipped output arrays
      pMyRegions  = pRegions  ? pRegions  : &sRegions ;
      pMyShells   = pShells   ? pShells   : &sShells ;
      pMyFaces    = pFaces    ? pFaces    : &sFaces ;
      pMyLoops    = pLoops    ? pLoops    : &sLoops ;
      pMyEdges    = pEdges    ? pEdges    : &sEdges ;
      pMyVertices = pVertices ? pVertices : &sVertices ;
    }

  // WorkMark all the input
  for(ii=0;ii<pMyRegions ->GetSize();ii++) { pMyRegions ->GetAt(ii)->Mark(eWorkMarkType) ; }
  for(ii=0;ii<pMyShells  ->GetSize();ii++) { pMyShells  ->GetAt(ii)->Mark(eWorkMarkType) ; }
  for(ii=0;ii<pMyFaces   ->GetSize();ii++) { pMyFaces   ->GetAt(ii)->Mark(eWorkMarkType) ; }
  for(ii=0;ii<pMyLoops   ->GetSize();ii++) { pMyLoops   ->GetAt(ii)->Mark(eWorkMarkType) ; }
  for(ii=0;ii<pMyEdges   ->GetSize();ii++) { pMyEdges   ->GetAt(ii)->Mark(eWorkMarkType) ; }
  for(ii=0;ii<pMyVertices->GetSize();ii++) { pMyVertices->GetAt(ii)->Mark(eWorkMarkType) ; }
  
  // Load input object temporarily into its topology array - marked or unmarked
  switch(lType)
    { case SmRegion_TYPE : pMyRegions->InsertAt (0, (SmRegion *) &rObject, 1) ; break ;
      case SmShell_TYPE  : pMyShells->InsertAt  (0, (SmShell  *) &rObject, 1) ; break ;
      case SmFace_TYPE   : pMyFaces->InsertAt   (0, (SmFace   *) &rObject, 1) ; break ;  
      case SmLoop_TYPE   : pMyLoops->InsertAt   (0, (SmLoop   *) &rObject, 1) ; break ;  
      case SmEdge_TYPE   : pMyEdges->InsertAt   (0, (SmEdge   *) &rObject, 1) ; break ;  
      case SmVertex_TYPE : pMyVertices->InsertAt(0, (SmVertex *) &rObject, 1) ; break ;
      default: ;
    }

  // Regions - already done in the SmBrep special case branch

  // done with regions - clear that bit in the lMoreWork bit array
  lMoreWork &= ~( 1 << 0 ) ;

  // For all input regions - Get Shells
  if(lMoreWork)
    {
      SmTArray<SmShell *> sRegionShells ;

      // iter all regions - gathering shells
      for(ii=0;ii<pMyRegions->GetSize();ii++)
        {
          pMyRegions->GetAt(ii)->GetShells(sRegionShells) ;

          for(jj=0;jj<sRegionShells.GetSize();jj++)
            {
              if(!sRegionShells[jj]->IsMarked(eWorkMarkType)) 
                { 
                  pMyShells->Add(sRegionShells[jj]) ;
                  sRegionShells[jj]->Mark(eWorkMarkType) ;
                } // Accumulate unWorked Shells
            } // end iter sRegions->Shells 
        } // end iter all regions - gathering shells
    } // end lMoreWork need to iter regions check

  // done getting shells - clear that bit in the lMoreWork bit array
  lMoreWork &= ~( 1 << 1 ) ; 

  // For all shells - get Faces, Edges, and Vertices
  if(lMoreWork)
    {
      SmTArray<SmFaceuse *> sShellFaceuses ;
      SmTArray<SmEdge    *> sShellEdges ; 
      SmTArray<SmVertex  *> sShellVertices ;

      // iter all shells - gathering faces, wire edges, and shell vertices
      for(ii=0;ii<pMyShells->GetSize();ii++)
        {
          SmShell *pShell = pMyShells->GetAt(ii) ;

          // faces through faceuses
          pShell->GetFaceuses( sShellFaceuses ) ;
          for(jj=0;jj<sShellFaceuses.GetSize();jj++)
            { if(   sShellFaceuses[jj]->GetFace()
                 && !sShellFaceuses[jj]->GetFace()->IsMarked(eWorkMarkType)) 
                { pMyFaces->Add(sShellFaceuses[jj]->GetFace()) ; 
                  sShellFaceuses[jj]->Mark(eWorkMarkType) ;
                }
            }

          // wire edges
          pShell->GetWireEdges( sShellEdges ) ;
          for(jj=0;jj<sShellEdges.GetSize();jj++)
            { if(   sShellEdges[jj]
                 && !sShellEdges[jj]->IsMarked(eWorkMarkType))
                { pMyEdges->Add(sShellEdges[jj]) ; 
                  sShellEdges[jj]->Mark(eWorkMarkType) ;
                }
            }

          // shell vertices
          if(   pShell->GetVertex()
             && !pShell->GetVertex()->IsMarked(eWorkMarkType) )
            { pMyVertices->Add(pShell->GetVertex()) ; 
              pShell->GetVertex()->Mark(eWorkMarkType) ;
            }

        } // end iter all shells - gathering faces, wire edges, and shell vertices 
    } // end lMoreWork need to iter shells check

  // done getting faces - clear that bit in the lMoreWork bit array
  lMoreWork &= ~( 1 << 2 ) ; 

  // For all Faces - get Loops
  if(lMoreWork)
    {
      SmTArray<SmLoop *> sFaceLoops ; 

      // iter all faces - gathering loops
      for(ii=0;ii<pMyFaces->GetSize();ii++)
        {
          SmFace *pFace = pMyFaces->GetAt(ii) ;

          // face->loops
          pFace->GetLoops(sFaceLoops) ;
          for(jj=0;jj<sFaceLoops.GetSize();jj++)
            { if(   sFaceLoops[jj]
                 && !sFaceLoops[jj]->IsMarked(eWorkMarkType)) 
              { pMyLoops->Add(sFaceLoops[jj]) ; 
                sFaceLoops[jj]->Mark(eWorkMarkType) ;
              } 
            }
        } // end iter all faces - gathering loops
    } // end lMoreWork need to iter faces check

  // done getting loops - clear that bit in the lMoreWork bit array
  lMoreWork &= ~( 1 << 3 ) ; 

  // For all Loops - get Edges and Vertices
  if(lMoreWork)
    {
      SmTArray<SmEdge   *> sLoopEdges ; 
      SmTArray<SmVertex *> sLoopVertices ; 

      // for every loop
      for(ii=0;ii<pMyLoops->GetSize();ii++)
        {
          SmLoop *pLoop = pMyLoops->GetAt(ii) ;

          // Loop->Edges
          pLoop->GetEdges(sLoopEdges) ;
          for(jj=0;jj<sLoopEdges.GetSize();jj++)
            { if(   sLoopEdges[jj]
                 && !sLoopEdges[jj]->IsMarked(eWorkMarkType)) 
              { pMyEdges->Add(sLoopEdges[jj]) ;
                sLoopEdges[jj]->Mark(eWorkMarkType) ; 
              } 
            }

          // Loop->Vertices
          pLoop->GetVertices(sLoopVertices) ;
          for(jj=0;jj<sLoopVertices.GetSize();jj++)
            { if(   sLoopVertices[jj]
                 && !sLoopVertices[jj]->IsMarked(eWorkMarkType))
              { pMyVertices->Add(sLoopVertices[jj]) ;
                sLoopVertices[jj]->Mark(eWorkMarkType) ; 
              } 
            }
        } // end iter all loops - gathering edges and vertices
    } // end lMoreWork need to iter Loops check

  // done getting edges - clear that bit in the lMoreWork bit array
  lMoreWork &= ~( 1 << 4 ) ; 

  // For all Edges - get Vertices
  if(lMoreWork)
    {
      SmTArray<SmVertex *> sEdgeVertices ; 

      // for all edges
      for(ii=0;ii<pMyEdges->GetSize();ii++)
        {
          SmEdge *pEdge = pMyEdges->GetAt(ii) ;

          // Edge->Vertices
          pEdge->GetVertices(sEdgeVertices) ;
          for(jj=0;jj<sEdgeVertices.GetSize();jj++)
            { if(   sEdgeVertices[jj]
                 && !sEdgeVertices[jj]->IsMarked(eWorkMarkType)) 
              { pMyVertices->Add(sEdgeVertices[jj]) ;
                sEdgeVertices[jj]->Mark(eWorkMarkType) ; 
              }
            }
        } // end iter all edges gathering vertices
    } // end lMoreWork need to iter edges check

  // done getting vertices - clear that bit in the lMoreWork bit array
  lMoreWork &= ~( 1 << 5 ) ; 

  SM_ASSERT(lMoreWork == 0) ;

  // arrive here when all topology added to accumulation arrays without regard to marks
  // next - remove the pObject from whatever array it was added to temporarily

 switch(lType)                        
   { case SmRegion_TYPE : pMyRegions-> RemoveAt(0) ; break ;
     case SmShell_TYPE  : pMyShells->  RemoveAt(0) ; break ;
     case SmFace_TYPE   : pMyFaces->   RemoveAt(0) ; break ;
     case SmLoop_TYPE   : pMyLoops->   RemoveAt(0) ; break ;
     case SmEdge_TYPE   : pMyEdges->   RemoveAt(0) ; break ;
     case SmVertex_TYPE : pMyVertices->RemoveAt(0) ; break ;
     default: ;
   }

  // when using marks - copy unmarked objects to the output arrays
  if(bMarking)
    {
#define MOVE_TO_OUTPUT(pWorkArray, pOutArray) \
      if(pOutArray) \
        { for(ii=0;ii<pWorkArray->GetSize();ii++)             \
            {                                                 \
              if(!pWorkArray->GetAt(ii)->IsMarked(eMarkType)) \
                {                                             \
                  pOutArray->Add(pWorkArray ->GetAt(ii)) ;    \
                  pWorkArray->GetAt(ii)->Mark(eMarkType) ;    \
        }   }   }                                              

      // copy unmarked output to input arrays
      MOVE_TO_OUTPUT(pMyRegions , pRegions ) ;
      MOVE_TO_OUTPUT(pMyShells  , pShells  ) ;
      MOVE_TO_OUTPUT(pMyFaces   , pFaces   ) ;
      MOVE_TO_OUTPUT(pMyLoops   , pLoops   ) ;
      MOVE_TO_OUTPUT(pMyEdges   , pEdges   ) ;
      MOVE_TO_OUTPUT(pMyVertices, pVertices) ;

#undef MOVE_TO_OUTPUT
    } // end unsing Marks check

  // all done
  return(SM_SUCCESS) ;

} // end SmTopologyTraverser::GetSubTopology

/*******************************************************************//**
PURPOSE: Walk the topology graph down from the set of input objects adding
 pointers to every Topology object (SmRegion, SmShell, SmFace, SmLoop, SmEdge, SmVertex)
 encountered along the way to the output arrays. 

NOTES: 
 When eMarkType == SM_MT_NOMARK - very expensive AddUnique calls are made to build unique output arrays
 When eMarkType != SM_MT_NOMARK - much cheaper marks are used to build unique output arrays, only
                                  unmarked elems are added to the output arrays.  When added, elems
                                  are marked to prevent them from being added multiple times.
                                  Note control the mark state prior to this call with
                                  SmTopology::NewMark()
                                  SmTopology::Mark() and
                                  SmTopology::UnMark().

 0. The input list may only contain one SmBrep object or a list of other topology types
 1. The output arrays are unique, that is the objects on them are listed only once.
 2. The output arrays are optional.  Set any  output array pointer to NULL to ignore them.
 3. The input object can be one of SmBrep, SmRegion, SmShell, SmFace, SmLoop, SmEdge, SmVertex
      but cannot be SmFaceuse, SmLoopuse, SmEdgeuse, SmVertexuse.
    When the input argument is of an invalid type the output lists are
    cleared (bRecursing ==FALSE) or left alone (bRecursing==TRUE) and SM_ERR is returned.
 4. No objects are under a SmVertex, one can call this function with an input vertex but the
    no topology objects will be added to the output

***********************************************************************/
SmStatus SmTopologyTraverser::GetSubTopologies
 (SmTArray<SmTopology *> & rObjects,        // in : members of an existing tree (not a use object)
  SmTArray<SmRegion *>   * pRegions,        // out: opt unique regions list,  NULL to ignore, default:[NULL]
  SmTArray<SmShell  *>   * pShells,         // out: opt unique shells list,   NULL to ignore, default:[NULL]
  SmTArray<SmFace   *>   * pFaces,          // out: opt unique faces list,    NULL to ignore, default:[NULL]
  SmTArray<SmLoop   *>   * pLoops,          // out: opt unique loops list,    NULL to ignore, default:[NULL]
  SmTArray<SmEdge   *>   * pEdges,          // out: opt unique edges list,    NULL to ignore, default:[NULL]
  SmTArray<SmVertex *>   * pVertices,       // out: opt unique vertices list, NULL to ignore, default:[NULL]
  SmBoolean                bReSetArrays,    // in : TRUE = reset arrays, FALSE = accumulate within arrays, Default:[TRUE]
  SmMarkType               eMarkType)       // in : mark type to check, SM_MT_NOMARK = no mark checks,
                                            //      not incremented, default:[SM_MT_NOMARK]
{
  // no work - no input
  if(rObjects.GetSize() == 0) 
    { return(SM_SUCCESS) ; }

  // no work - no output requested
  if(   pRegions  == NULL
     && pShells   == NULL 
     && pFaces    == NULL 
     && pLoops    == NULL 
     && pEdges    == NULL  
     && pVertices == NULL) 
    { return(SM_SUCCESS) ; }

  // init output when asked
  if(bReSetArrays)
    {
      if(pRegions ) pRegions ->ReSet() ;
      if(pShells  ) pShells  ->ReSet() ;
      if(pFaces   ) pFaces   ->ReSet() ;
      if(pLoops   ) pLoops   ->ReSet() ;
      if(pEdges   ) pEdges   ->ReSet() ;
      if(pVertices) pVertices->ReSet() ;
    }

  // set up working arrays - needed for missing output arrays
  SmTArray<SmVertex *> sVertices, *pMyVertices = pVertices ? pVertices : &sVertices ;
  SmTArray<SmEdge   *> sEdges,    *pMyEdges    = pEdges    ? pEdges    : &sEdges    ;
  SmTArray<SmLoop   *> sLoops,    *pMyLoops    = pLoops    ? pLoops    : &sLoops    ;
  SmTArray<SmFace   *> sFaces,    *pMyFaces    = pFaces    ? pFaces    : &sFaces    ;
  SmTArray<SmShell  *> sShells,   *pMyShells   = pShells   ? pShells   : &sShells   ;
  SmTArray<SmRegion *> sRegions,  *pMyRegions  = pRegions  ? pRegions  : &sRegions  ;

  // locals 
  ULONG ii ;
  SmBrep *pBrep = NULL ;

  // when rObjects is not already one of the output arrays
  if(   &rObjects != (SmTArray<SmTopology *> *)pRegions
     && &rObjects != (SmTArray<SmTopology *> *)pShells  
     && &rObjects != (SmTArray<SmTopology *> *)pFaces   
     && &rObjects != (SmTArray<SmTopology *> *)pLoops   
     && &rObjects != (SmTArray<SmTopology *> *)pEdges   
     && &rObjects != (SmTArray<SmTopology *> *)pVertices)
    {
      // load the rObjects into the output arrays
      for(ii=0;ii<rObjects.GetSize();ii++)
        {
          SmTopology *pObject = rObjects[ii] ;
          SM_TYPE     lType   = pObject->GetType() ;
  
          switch(lType)
            { case SmBrep_TYPE   : if(pBrep != NULL) { return(SM_ERR_INVALID_INPUT) ; }
                                   else              { pBrep = (SmBrep*) pObject ; break ; }
              case SmRegion_TYPE : pMyRegions->Add ((SmRegion *) pObject) ; break ;
              case SmShell_TYPE  : pMyShells->Add  ((SmShell  *) pObject) ; break ;
              case SmFace_TYPE   : pMyFaces->Add   ((SmFace   *) pObject) ; break ;  
              case SmLoop_TYPE   : pMyLoops->Add   ((SmLoop   *) pObject) ; break ;  
              case SmEdge_TYPE   : pMyEdges->Add   ((SmEdge   *) pObject) ; break ;  
              case SmVertex_TYPE : pMyVertices->Add((SmVertex *) pObject) ; break ;
              default: ;
            }
        } // end loading output arrays
    } // end need to distribute intput array check

  // pass the call along
  return( GetSubTopology( pBrep ? *(SmTopology *)pBrep : *rObjects[0],
                          pMyRegions,  
                          pMyShells,
                          pMyFaces,
                          pMyLoops,
                          pMyEdges,            
                          pMyVertices,
                          FALSE,
                          eMarkType) ) ;
                           
} // end SmTopologyTraverser::GetSubTopologies
