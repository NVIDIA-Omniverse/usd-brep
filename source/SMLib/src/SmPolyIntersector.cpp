// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolyIntersector.cpp 
* PURPOSE: Source file for SmPolyIntersector object.
**********************************************************************/

#include "StdAfx.h"

#include <SmPolyIntersector.h>
#include <SmPolySolver.h>
#include <SmExtent3d.h>
#include <SmTree.h>
#include <SmGraphicsExtern.h>
#include <SmGeomUtility.h>
#include <SmContext.h>

//#define SM_VALIDATE_TOPOLOGY 1
//#define SM_VALIDATE_INTERSECTIONS 1

#ifdef SM_DEBUG_CODE
static ULONG s_lMLSOFCount = 0;
#endif                    

/*******************************************************************//**
PURPOSE: Destructor for the topology intersector.

NOTES: 
***********************************************************************/
SmPolyIntersector::~SmPolyIntersector()
{
    if (m_pBToO) { delete m_pBToO; m_pBToO = NULL ; }
    if (m_pOToB) { delete m_pOToB; m_pOToB = NULL ; }
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmPolyIntersector::SmPolyIntersector
  (const SmContext & crContext)
 : m_bPolyDeleted(FALSE),
   m_crContext(crContext), 
   m_pBrep(NULL), 
   m_pOther(NULL),
   m_pBToO(NULL), 
   m_pOToB(NULL),
   m_bSwappedOrder(FALSE),
   m_dThisApproxTol3d(0.0),
   m_dThisAngTolRad(0.0), 
   m_bTrimWithPlane(FALSE)  
{
}

/*******************************************************************//**
PURPOSE: Constructor for the topology intersector.

NOTES: 
***********************************************************************/
SmPolyIntersector::SmPolyIntersector
  (const SmContext & crContext,
   SmPolyBrep * pPolyBrep, 
   SmPolyBrep *pOther,
   double dThisApproxTol3d,
   double dThisAngTolRad)
 : m_bPolyDeleted(FALSE),
   m_crContext(crContext), 
   m_pBrep(pPolyBrep), 
   m_pOther(pOther),
   m_bSwappedOrder(FALSE), 
   m_dThisApproxTol3d(dThisApproxTol3d),
   m_dThisAngTolRad(dThisAngTolRad),
   m_bTrimWithPlane(FALSE)
{
  m_pBToO = new(crContext) SmMapPtrToPtr<SmObject, SmObject>;
  m_pOToB = new(crContext) SmMapPtrToPtr<SmObject, SmObject>;

} // SmPolyIntersector::SmPolyIntersector constructor

/*******************************************************************//**
PURPOSE: Split near neighoring PolyEdges which are within tol of pVertex.

NOTES: Near Neighboring PolyEdges are the PolyEdges directly 
       connected to the PolyEdges that start at pVertex,
       so PolyEdges one away from connecting directly to pVertex.
***********************************************************************/
SmStatus SmPolyIntersector::SplitEdgesAroundVertex
 (SmPolyVertex            * pVertex,       // in : PolyVertex to check
  SmTArray<SmPolyVertex*> & rNewVertices,  // out: New Vertices created by splitting edges
  SmTArray<SmPolyEdge*>   & rNewEdges)     // out: New Edges created by splitting edges
{
  // init output
  rNewVertices.ReSet();
  rNewEdges.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmPoint3d    sVPnt     = pVertex->GetPoint() ;
  SmPolyEdge * pTestE    = NULL;

  // get polyedges starting at the input polyVertex
  SmTArray<SmPolyEdge*>  sNewEdges ;
  SM_PTR_ARRAY(sVertEdges,SmPolyEdge,32) ;
  pVertex->GetStartingPolyEdges(sVertEdges) ;
  ULONG        lNumEdges = sVertEdges.GetSize() ;

  // for every polyEdge starting at the PolyVertex
  for(ii=0; ii<lNumEdges; ii++)
    {
      SmPolyEdge * pE = sVertEdges[ii];

      // Look at the first edge just past the adjacent edge, in each direction.
      for(jj=0; jj<2; jj++)
        {
          if (jj == 0)
            {
              pTestE = pE->GetCCWPolyEdge();
              if ( pTestE == pE->GetCWPolyEdge() )
                { continue; }
            }
          else
            {
              pTestE = pE->GetCWPolyEdge() ? pE->GetCWPolyEdge()->GetCWPolyEdge() : NULL ;
              if ( pTestE == pE )
                { continue; }
            }

          if ( pTestE == NULL ) // [B348]
            { continue; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              pVertex->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 1,0,0); pE->Draw();     sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); pTestE->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // get dist from pVertex to the edge just pass the pVertex adjacent edge.
          double dParam, dDist;
          SER( pTestE->DropPoint( sVPnt, dParam, dDist ));
          double dTol = pVertex->GetTolerance() + pTestE->GetTolerance();

          // if that dist is greater than tol - skip splitting
          if (dDist > dTol)
            { continue; }

          // Note: split it at the vertex point, not at the drop point on
          // the edge.  That can change the geometry enough so that the line
          // segment will intersect the same edge again, far enough away from
          // the new vertex, and can result in an essentially infinite loop.  [B380]

          double dDistStart = sVPnt.DistanceBetween( pTestE->GetStartPoint() );
          double dDistEnd   = sVPnt.DistanceBetween( pTestE->GetEndPoint()   );

          // when pVertex is more than tol from both ends of the edge after the adjacent edge
          if ( dDistStart > dTol && dDistEnd > dTol )
            {
              SmPolyFace   * pPF = pTestE->GetPolyFace(); NER(pPF);
              SmPolyEdge   * pNewEdge = NULL;
              SmPolyVertex * pNewVertex = NULL;

              // split the Edge at the pVertex point.
              SER(pPF->MakeVertexSplitPolyEdge(pTestE,       // in : PolyEdge to split
                                               sVPnt,        // in : Point split location
                                               pNewEdge,     // out: new PolyEdge (and new radial partners)
                                               pNewVertex)); // out: new PolyVertex
              if (pNewVertex == NULL)
                { continue; }

              // update the output lists
              rNewVertices.Add(pNewVertex);
              pNewEdge->GetAllRadials(sNewEdges) ;
              rNewEdges.Append(sNewEdges) ; 
            } // end dist to TgtEdge ends is larger than tol check
        } // end for two edges (CCW and CW-CW)
    } // end iter every PolyEdge starting at the PolyVertex

  return SM_SUCCESS;

} // end SmPolyIntersector::SplitEdgesAroundVertex

/*******************************************************************//**
PURPOSE: Connect and Combine PolyVertices expected to be related 
   to one another to PolyVertices or PolyEdges found close to them
   in their respective PolyBreps.
  A kind of poor man's complex corner idea.

NOTES: Only checks PolyEdges and PolyVertices that can be
  found through topology connections and relationships for
  being near to the input tgtVerts. Reducing the number
  of PolyObjects checked for merging increasing the speed
  of this check.
   
  1. Split PolyEdges within tol of TgtVerts and 
      then Combine new SplitVerts with TgtVerts - Squeeze any degenerate faces
  2. When pBrepVertex is mated to something other than pOtherVertex or
         pOtherVertex is mated to something other than pBrepVertex
      then Remove the relationship
           assume related vertices are close to one another and combine them
           squeeze any resulting degenerate faces
           remove/update relationships for deleted PolyEdges and PolyVertices
  3. Make sure that relationships for PolyEdges connected to given vertices 
      of given relation<pVertex, pOtherVertex> that connect to related vertices
      are also related. 
    
  no work: 1. When pBrepVertex->GetOtherMate() == pOtherVertex
                   and  pOtherVertex->GetBrepMate() == pBrepMate
                   - already done and everything okay
                2. When pBrepBertex->GetOtherMate() != NULL
                   or   pOtherVertex->GetBrepMate() != NULL
                   - signal "attempt to squeeze already-related vertices; skipping."
                returns SM_SUCCESS

  GWC: I spent a lot of time stepping through this heuristic and
       I'm not sure that it is going to adequately protect the
       boolean operator from making degenerate faces or from 
       making sure the relationships list is properly up to date.
       In short, I think this is a suspecious subroutine.
***********************************************************************/
SmStatus SmPolyIntersector::CheckForSqueezeVertices
 (SmPolyVertex            * pBrepVertex,      // in : PolyVertex from m_pBrep
  SmPolyVertex            * pOtherVertex,     // in : PolyVertex from m_pOther
  SmTArray<SmPolyVertex*> & rDeletedV,        // i/o: accumulating list of stale PolyVertices
  SmTArray<SmPolyVertex*> & rSurvivingV,      // i/o: associated list of PolyVertices replacing stale PolyVertices (can have NULL entries)
  SmTArray<SmPolyEdge*>   & rDeletedE,        // i/o: accumulating list of stale PolyEdges
  SmTArray<SmPolyEdge*>   & rSurvivingE,      // i/o: associated list of PolyEdges replacing stale PolyEdges(NULL=no replacement)
  SmBoolean                 bCheckCloserMate) // in : TRUE  = Check if current BrepVertex and OtherVertex mates are closer than 
                                              //              than BrepVertex and OtherVertex are to eachother
                                              //      FALSE = Don't.
                                              //      Use FALSE when squeezing degenerate edge
{
  // Make sure tolerances of related vertices are the same.
  if (pOtherVertex->GetTolerance() > 1.01 * pBrepVertex->GetTolerance())
    { pBrepVertex->SetTolerance( pOtherVertex->GetTolerance() ); }

  if (pBrepVertex->GetTolerance() > 1.01 * pOtherVertex->GetTolerance())
    { pOtherVertex->SetTolerance( pBrepVertex->GetTolerance() ); }

  //SmPolyEdge *pOtherEdgeSurvive = NULL;
  //SmPolyEdge *pBrepEdgeSurvive  = NULL;
  ULONG ii, jj;

  // get related mates of input PolyVerts
  SmPolyVertex *pMateOfBrepV  = (SmPolyVertex*)GetOtherMate( pBrepVertex ); // m_pBToO
  SmPolyVertex *pMateOfOtherV = (SmPolyVertex*)GetBrepMate( pOtherVertex ); // m_pOToB

  // If they're already properly related, don't do anything.
  if (   pMateOfBrepV  && pMateOfBrepV  == pOtherVertex
      && pMateOfOtherV && pMateOfOtherV == pBrepVertex)
    { return SM_SUCCESS; }

    { // begin scope to check for mismatched existing relationships amongst input polyVerts

      // gwc?: this heuristic seems off to me because it's order dependent.  
      //       If the boolean tries to relate one pBrep PolyVertex to 
      //       two different pOther Vertices where the first pOther PolyVertex is closer
      //       to the pBrep PolyVertex than the 2nd, 
      //       this check signals a warning and returns if the first relationship happens
      //       to be made first, otherwise this heuristic does nothing and allows processing to continue.
      //       Is that the desired behavior?
      //static SmBoolean sbCheckCloserMate = TRUE;  // [B449]
      //       Moved to function argument [B643]
      if ( bCheckCloserMate )
        {
          // Also, if they're already "properly" related to other vertices,
          // don't do anything.  At least for now, define "properly" related
          // as being physically closer than the current pair.  [B198]
          // Note, "improper" relationships are dealt with later in this routine.
          // Check each one individually.
          if ( pMateOfBrepV != NULL )
            {
              double dOldDist = pBrepVertex->GetPoint().DistanceBetween( pMateOfBrepV->GetPoint() );
              double dNewDist = pBrepVertex->GetPoint().DistanceBetween( pOtherVertex->GetPoint() );
              if ( dOldDist < dNewDist - SM_EFF_ZERO )
                {
                  WARN(_T("SmPolyIntersector::CheckForSqueezeVertices: attempt to squeeze already-related vertices; skipping."));
                  return SM_SUCCESS;
                }
            }
          if ( pMateOfOtherV != NULL )
            {
              double dOldDist = pOtherVertex->GetPoint().DistanceBetween( pMateOfOtherV->GetPoint() );
              double dNewDist = pOtherVertex->GetPoint().DistanceBetween( pBrepVertex->GetPoint() );
              if ( dOldDist < dNewDist - SM_EFF_ZERO )
                {
                  WARN(_T("SmPolyIntersector::CheckForSqueezeVertices: attempt to squeeze already-related vertices; skipping."));
                  return SM_SUCCESS;
                }
            }
        }  // end if sbCheckCloserMate
    } // end scope to check for mismatched existing relationships amongst input polyVerts

  // locals
  SM_PTR_ARRAY(sNewBrepVertices,SmPolyVertex,32);
  SM_PTR_ARRAY(sNewOtherVertices,SmPolyVertex,32);
  SM_PTR_ARRAY(sNewBrepEdges,SmPolyEdge,32);
  SM_PTR_ARRAY(sNewOtherEdges,SmPolyEdge,32);

  // GWC: I don't see why the next call sequence is being made? Perhaps a heuristic to prevent building degenerate geometry?
  // Check for pBrepVertex dropping onto near neighboring edges (edges connected to Edges that start at the vertex)
  // Split pBrep near neighoring PolyEdges which are within tol of pBrepVertex.
  SER(SplitEdgesAroundVertex(pBrepVertex,         // in : PolyVertex to check
                             sNewBrepVertices,    // out: New Vertices created by splitting edges
                             sNewBrepEdges));     // out: New Edges created by splitting edges

  // when pBrep PolyEdges were split because they were close to pBrepVertex - combine new Verts with original
  if (sNewBrepVertices.GetSize() > 0)
    {
      SM_PTR_ARRAY(sChangedFaces,SmPolyFace, 32) ;
      SM_PTR_ARRAY(sDeletedEdges,SmPolyEdge, 32) ;
      SM_PTR_ARRAY(sDeletedV,    SmPolyVertex, 16) ;
      SM_PTR_ARRAY(sSurvivingV,  SmPolyVertex, 16) ;

      // GWC: why only combine the 1st Vert - if this is a good idea why not all of them? - check for that case
      SM_ASSERT_MSG(sNewBrepVertices.GetSize() <= 1, 
                    _T("SmPolyIntersector::CheckForSqueezeVertices found a more than 1 spit Edge case - code may need a loop to merge all verts")) ;
      
      // merge the original vert with the NewVert
      SER(m_pBrep->CombineCoincidentVertices(pBrepVertex,         // in : PolyVertex to remain after combining
                                             sNewBrepVertices[0], // in : PolyVertex to delete after combining - stale after this call
                                             sDeletedEdges,       // out: List of stale deleted PolyEdge pointers
                                                                  //       (when Verts connect to a commonPolyFace but not a CommonPolyEdge,
                                                                  //        these PolyEdges are 1st created and then deleted in this method)
                                             sChangedFaces,       // out: List of PolyFaces attached to deleted PolyEdges
                                                                  //       (some of these may be stale)
                                             &sDeletedV,          // out: optional array of additional deleted PolyVertices, NULL to ignore, default:[NULL]
                                             &sSurvivingV)) ;     // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]
      
      // accumulate output
      rDeletedV.Add(sNewBrepVertices[0]) ;
      rSurvivingV.Add(pBrepVertex) ;

      rDeletedV.Append(sDeletedV) ;
      rSurvivingV.Append(sSurvivingV) ;

      rDeletedE.Append(sDeletedEdges) ; // there are no surviving Edges from this call
      for(ii=0;ii<sDeletedEdges.GetSize();ii++)
        { rSurvivingE.Add(NULL) ; }
       
      // iter every changed PolyFace - squeezing those that are degenerate
      ULONG lNumFaces = sChangedFaces.GetSize();
      for(ii=0; ii<lNumFaces; ii++)
        {
          SmPolyFace *pF = sChangedFaces[ii];

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetLook(3,5, 1,0,0); pBrepVertex->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pF->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          SmPolyEdge *pDelEdge1 = NULL, *pDelEdge2 = NULL, *pEdgeSurvive1 = NULL, *pEdgeSurvive2 = NULL;
          SmBoolean   bFaceWasSqueezed = FALSE ;
          SmTArray<SmPolyVertex*> sDeletedVertices ; 
          SmTArray<SmPolyVertex*> sSurvivingVertices ; 

          // squeeze face when it contains only two PolyEdges and glue any attached radial partners
          //   gwc - should be extended to more general cases: face too small, sliver face, etc.
          SER(pF->GetPolyBrep()->SqueezeFaceIfDegenerate
               (pF,                          // in : target PolyFace to delete
                pBrepVertex->GetTolerance(), // in : not used
                bFaceWasSqueezed,            // out: TRUE=PolyFace was squeezed, FALSE=not
                pDelEdge1,                   // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                pDelEdge2,                   // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                pEdgeSurvive1,               // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                pEdgeSurvive2,               // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                &sDeletedVertices,           // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                &sSurvivingVertices)) ;      // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

          if (bFaceWasSqueezed)
            {
              // accumlate output
              rDeletedE.Add(pDelEdge1);
              rDeletedE.Add(pDelEdge2);
              rSurvivingE.Add(pEdgeSurvive1);
              rSurvivingE.Add(pEdgeSurvive2);
              m_vDeletedFaces.Add(pF);
              rDeletedV.Append(sDeletedVertices) ;
              rSurvivingV.Append(sSurvivingVertices) ;

              // replace relationships of pDelEdge1 stale pointers
              // pBrepEdgeSurvive = pEdgeSurvive1;
              SmPolyEdge *pDeletedOtherEdge = (SmPolyEdge*)GetOtherMate(pDelEdge1); // m_pBToO
              if (pDeletedOtherEdge)
                {
                  SER(RemoveRelationship(pDelEdge1,pDeletedOtherEdge));
                  if (pEdgeSurvive1 && GetOtherMate(pEdgeSurvive1) == NULL) // m_pBToO
                    {
                      SER(Relate(pEdgeSurvive1,pDeletedOtherEdge));
                    }
                }

              // replace relationships of pDelEdge2 stale pointers
              pDeletedOtherEdge = (SmPolyEdge*)GetOtherMate(pDelEdge2); // m_pBToO
              if (pDeletedOtherEdge)
                {
                  SER(RemoveRelationship(pDelEdge2,pDeletedOtherEdge));
                  if (pEdgeSurvive2 && GetOtherMate(pEdgeSurvive2) == NULL)  // m_pBToO
                    {
                      SER(Relate(pEdgeSurvive2,pDeletedOtherEdge));
                    }   
                }

              // replace relationships of sDeletedVertices stale pointers
              for(jj=0;jj<sDeletedVertices.GetSize();jj++)
                {
                  SmPolyVertex *pDeletedOtherVertex = (SmPolyVertex*)GetOtherMate(sDeletedVertices[jj]); // m_pBToO
                  if(pDeletedOtherVertex)
                    {
                      SER(RemoveRelationship(sDeletedVertices[jj],pDeletedOtherVertex));
                      if(sSurvivingVertices[jj] && GetOtherMate(sSurvivingVertices[jj]) == NULL) // m_pBToO
                        {
                          SER(Relate(sSurvivingVertices[jj], pDeletedOtherVertex));
                        }
                    } // end DeletedVertex has a relationship check  
                } // end iter every DeletedVertex looking for relationships to delete or replace
            } // end polyFace was squeezed check
        } // end iter all affected faces looking for degenerate ones
    } // end when pBrep PolyEdges were split because they were close to pBrepVertex check
  
  // this heuristic was coded for just one nearby edge being split - but any number might be
  //   if we run into a case where many edges exist - update the above code to be a loop that catches all the split edges 
  if (sNewBrepVertices.GetSize() > 1) 
    { SE_MSG(SM_ERR, _T("Heuristic was written for a single case but needs to be expanded into a loop to handle a many case")); } // Just warn for now
  
  // Check for pOtherVertex dropping onto near neighboring edges (edges connected to Edges that start at the vertex)
  // Split pOther near neighoring PolyEdges which are within tol of pOtherVertex.
  SER(SplitEdgesAroundVertex(pOtherVertex,        // in : PolyVertex to check
                             sNewOtherVertices,   // out: New Vertices created by splitting edges
                             sNewOtherEdges));    // out: New Edges created by splitting edges

  // when pBrep PolyEdges were split because they were close to pBrepVertex - combine new Verts with original
  if (sNewOtherVertices.GetSize() > 0)
    {
      SM_PTR_ARRAY(sChangedFaces,SmPolyFace,32);
      SM_PTR_ARRAY(sDeletedEdges,SmPolyEdge,32);
      SM_PTR_ARRAY(sDeletedV,    SmPolyVertex, 16) ;
      SM_PTR_ARRAY(sSurvivingV,  SmPolyVertex, 16) ;

      // GWC: why only combine the 1st Vert - if this is a good idea why not all of them? - check for that case
      SM_ASSERT_MSG(sNewBrepVertices.GetSize() <= 1, 
                    _T("SmPolyIntersector::CheckForSqueezeVertices found a more than 1 spit Edge case - code may need a loop to merge all verts")) ;
      
      SER(m_pOther->CombineCoincidentVertices(pOtherVertex,         // in : PolyVertex to remain after combining
                                              sNewOtherVertices[0], // in : PolyVertex to delete after combining - stale after this call
                                              sDeletedEdges,        // out: List of stale deleted PolyEdge pointers
                                                                    //       (when Verts connect to a commonPolyFace but not a CommonPolyEdge,
                                                                    //        these PolyEdges are 1st created and then deleted in this method)
                                              sChangedFaces,        // out: List of PolyFaces attached to deleted PolyEdges
                                                                    //       (some of these may be stale)
                                              &sDeletedV,           // out: optional array of additional deleted PolyVertices, NULL to ignore, default:[NULL]
                                              &sSurvivingV)) ;      // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

      // accumulate output
      rDeletedV.Add(sNewOtherVertices[0]) ;
      rSurvivingV.Add(pOtherVertex) ;

      rDeletedV.Append(sDeletedV) ;
      rSurvivingV.Append(sSurvivingV) ;

      rDeletedE.Append(sDeletedEdges) ;  // there are no surviving Edges from this call
      for(ii=0;ii<sDeletedEdges.GetSize();ii++)
        { rSurvivingE.Add(NULL) ; }
       
      // iter every changed PolyFace - squeezing those that are degenerate
      ULONG lNumFaces = sChangedFaces.GetSize();
      for(ii=0; ii<lNumFaces; ii++)
        {
          SmPolyFace * pF = sChangedFaces[ii];
          SmPolyEdge *pDelEdge1 = NULL, *pDelEdge2 = NULL, *pEdgeSurvive1 = NULL, *pEdgeSurvive2 = NULL;
          SmBoolean   bFaceWasSqueezed = FALSE;
          SmTArray<SmPolyVertex*> sDeletedVertices ; 
          SmTArray<SmPolyVertex*> sSurvivingVertices ; 

          // squeeze face when it contains only two PolyEdges and glue any attached radial partners
          //   gwc - should be extended to more general cases: face too small, sliver face, etc.
          SER(pF->GetPolyBrep()->SqueezeFaceIfDegenerate
                (pF,                           // in : target PolyFace to delete
                 pOtherVertex->GetTolerance(), // in : not used
                 bFaceWasSqueezed,             // out: TRUE=PolyFace was squeezed, FALSE=not
                 pDelEdge1,                    // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                 pDelEdge2,                    // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                 pEdgeSurvive1,                // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                 pEdgeSurvive2,                // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                 &sDeletedVertices,            // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                 &sSurvivingVertices)) ;       // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

          if (bFaceWasSqueezed)
            {
              // accumlate output
              rDeletedE.Add(pDelEdge1);
              rDeletedE.Add(pDelEdge2);
              rSurvivingE.Add(pEdgeSurvive1) ;
              rSurvivingE.Add(pEdgeSurvive2) ;
              m_vOtherDeletedFaces.Add(pF);
              rDeletedV.Append(sDeletedVertices) ;
              rSurvivingV.Append(sSurvivingVertices) ;

              // replace relationships of pDelEdge1 stale pointers
              // pOtherEdgeSurvive = pEdgeSurvive1;
              SmPolyEdge *pDeletedBrepEdge = (SmPolyEdge*)GetBrepMate(pDelEdge1); // m_pOToB
              if (pDeletedBrepEdge)
                {
                  SER(RemoveRelationship(pDeletedBrepEdge,pDelEdge1));
                  if (pEdgeSurvive1 && GetBrepMate(pEdgeSurvive1) == NULL) // m_pOToB
                    {
                      SER(Relate(pDeletedBrepEdge,pEdgeSurvive1));
                    }
                }
              
              // replace relationships of pDelEdge2 stale pointers
              pDeletedBrepEdge = (SmPolyEdge*)GetBrepMate(pDelEdge2); // m_pOToB
              if (pDeletedBrepEdge)
                {
                  SER(RemoveRelationship(pDeletedBrepEdge,pDelEdge2));
                  if (pEdgeSurvive2 && GetBrepMate(pEdgeSurvive2) == NULL) // m_pOToB
                    {
                      SER(Relate(pDeletedBrepEdge,pEdgeSurvive2));
                    }
                }

              // replace relationships of sDeletedVertices stale pointers
              for(jj=0;jj<sDeletedVertices.GetSize();jj++)
                {
                  SmPolyVertex *pDeletedBrepVertex = (SmPolyVertex*)GetBrepMate(sDeletedVertices[jj]); // m_pOToB
                  if(pDeletedBrepVertex)
                    {
                      SER(RemoveRelationship(pDeletedBrepVertex, sDeletedVertices[jj]));
                      if(sSurvivingVertices[jj] && GetBrepMate(sSurvivingVertices[jj]) == NULL) // m_pOToB
                        {
                          SER(Relate(pDeletedBrepVertex, sSurvivingVertices[jj]));
                        }
                    } // end DeletedVertex has a relationship check  
                } // end iter every DeletedVertex looking for relationships to delete or replace
            } // end polyFace was squeezed check
        } // end iter all affected faces looking for degenerate ones
    } // end when pOther PolyEdges were split because they were close to pOtherVertex check

  // this heuristic was coded for just one nearby edge being split - but any number might be
  //   if we run into a case where many edges exist - update the above code to be a loop that catches all the split edges 
  if (sNewOtherVertices.GetSize() > 1) 
    { SE_MSG(SM_ERR, _T("Heuristic was written for a single case but needs to be expanded into a loop to handle a many case")); } // Just warn for now
  
  // when pBrepVertex had a pre-existing, conflicting relationship - remove the relationship and squeeze the vertices
  if(   pMateOfBrepV != NULL 
     && pMateOfBrepV != pOtherVertex)
    {

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0); pMateOfBrepV->GetPoint().Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); pOtherVertex->GetPoint().Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 1,0,1); pBrepVertex->GetPoint().Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); this->m_pBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); this->m_pOther->Draw(); sm_GraphicsLoop();
          DrawRelatedObjects(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // remove stale undesired vertex relationship
      SER(RemoveRelationship(pBrepVertex,pMateOfBrepV));

      // pMateOfBrepV and rpOtherVertex are probably close - see if they can be squeezed
      double dDist = pMateOfBrepV->GetPoint().DistanceBetween(pOtherVertex->GetPoint());
      double dTol  = pMateOfBrepV->GetTolerance() + pOtherVertex->GetTolerance();
      if (dDist > dTol * 50.0) 
        { SER(SM_ERR); }

      SM_PTR_ARRAY(sFaces,SmPolyFace,32);
      SM_PTR_ARRAY(sDeletedEdges,SmPolyEdge,32);
      SM_PTR_ARRAY(sDeletedV,    SmPolyVertex, 16) ;
      SM_PTR_ARRAY(sSurvivingV,  SmPolyVertex, 16) ;

      SER(m_pOther->CombineCoincidentVertices(pOtherVertex,     // in : PolyVertex to remain after combining
                                              pMateOfBrepV,     // in : PolyVertex to delete after combining - stale after this call
                                              sDeletedEdges,    // out: List of stale deleted PolyEdge pointers
                                                                //       (when Verts connect to a commonPolyFace but not a CommonPolyEdge,
                                                                //        these PolyEdges are 1st created and then deleted in this method)
                                              sFaces,           // out: List of PolyFaces attached to deleted PolyEdges
                                                                //       (some of these may be stale)
                                              &sDeletedV,       // out: optional array of additional deleted PolyVertices, NULL to ignore, default:[NULL]
                                              &sSurvivingV)) ;  // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

      // accumulate output - stale vertex relationship already removed
      rDeletedV.Add(pMateOfBrepV);
      rSurvivingV.Add(pOtherVertex);

      rDeletedV.Append(sDeletedV) ;
      rSurvivingV.Append(sSurvivingV) ;

      // for every deleted edge
      ULONG lNumDeletedEdges = sDeletedEdges.GetSize();
      for (ii=0; ii<lNumDeletedEdges; ii++)
        {
          SmPolyEdge *pDeletedOtherEdge = sDeletedEdges[ii];
          if (pDeletedOtherEdge) 
            {
              // accumulate output
              rDeletedE.Add(pDeletedOtherEdge);// there are no surviving Edges from this call
              rSurvivingE.Add(NULL) ;

              // and remove obsolete relationships
              SmPolyEdge *pDeletedBrepEdge = (SmPolyEdge*)GetBrepMate(pDeletedOtherEdge); // m_pOToB
              SER(RemoveRelationship(pDeletedBrepEdge,pDeletedOtherEdge));
            }
        } // end iter every DeleteEdge - updating output and removing stale relationships

      // iter every changed PolyFace - squeezing PolyFaces that are degenerate
      ULONG lNumFaces = sFaces.GetSize();
      for (ii=0; ii<lNumFaces; ii++) 
        {
          SmPolyFace * pF = sFaces[ii];
          SmPolyEdge * pDelEdge1 = NULL, *pDelEdge2 = NULL, *pEdgeSurvive1 = NULL, *pEdgeSurvive2 = NULL;
          SmBoolean    bFaceWasSqueezed = FALSE;
          SmTArray<SmPolyVertex*> sDeletedVertices ; 
          SmTArray<SmPolyVertex*> sSurvivingVertices ; 

          // squeeze face when it contains only two PolyEdges and glue any attached radial partners
          //   gwc - should be extended to more general cases: face too small, sliver face, etc.
          SER(pF->GetPolyBrep()->SqueezeFaceIfDegenerate
               (pF,                           // in : target PolyFace to delete
                pOtherVertex->GetTolerance(), // in : not used
                bFaceWasSqueezed,             // out: TRUE=PolyFace was squeezed, FALSE=not
                pDelEdge1,                    // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                pDelEdge2,                    // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                pEdgeSurvive1,                // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                pEdgeSurvive2,               // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                &sDeletedVertices,           // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                &sSurvivingVertices)) ;      // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

          if (bFaceWasSqueezed) 
            {
              // accumlate output
              rDeletedE.Add(pDelEdge1);
              rDeletedE.Add(pDelEdge2);
              rSurvivingE.Add(pEdgeSurvive1) ;
              rSurvivingE.Add(pEdgeSurvive2) ;
              m_vOtherDeletedFaces.Add(pF);
              rDeletedV.Append(sDeletedVertices) ;
              rSurvivingV.Append(sSurvivingVertices) ;

              // replace relationships of pDelEdge1 stale pointers
              // pOtherEdgeSurvive = pEdgeSurvive1;
              SmPolyEdge *pDeletedBrepEdge = (SmPolyEdge*)GetBrepMate(pDelEdge1); // m_pOToB
              if (pDeletedBrepEdge) 
                {
                  SER(RemoveRelationship(pDeletedBrepEdge,pDelEdge1));
                  if (pEdgeSurvive1 && GetBrepMate(pEdgeSurvive1) == NULL) 
                    {
                      SER(Relate(pDeletedBrepEdge,pEdgeSurvive1));
                    }
                }

              // replace relationships of pDelEdge2 stale pointers
              pDeletedBrepEdge = (SmPolyEdge*)GetBrepMate(pDelEdge2); // m_pOToB
              if (pDeletedBrepEdge) 
                {
                  SER(RemoveRelationship(pDeletedBrepEdge,pDelEdge2));
                  if (pEdgeSurvive2 && GetBrepMate(pEdgeSurvive2) == NULL) // m_pOToB
                    {
                      SER(Relate(pDeletedBrepEdge,pEdgeSurvive2));
                    }
                }

              // replace relationships of sDeletedVertices stale pointers
              for(jj=0;jj<sDeletedVertices.GetSize();jj++)
                {
                  SmPolyVertex *pDeletedBrepVertex = (SmPolyVertex*)GetBrepMate(sDeletedVertices[jj]); // m_pOToB
                  if(pDeletedBrepVertex)
                    {
                      SER(RemoveRelationship(pDeletedBrepVertex,sDeletedVertices[jj]));
                      if(sSurvivingVertices[jj] && GetBrepMate(sSurvivingVertices[jj]) == NULL) // m_pOToB
                        {
                          SER(Relate(pDeletedBrepVertex, sSurvivingVertices[jj]));
                        }
                    } // end DeletedVertex has a relationship check  
                } // end iter every DeletedVertex looking for relationships to delete or replace
            } // end polyFace was squeezed check
        } // end iter all affected faces looking to squeeze degenerate ones

      // Fix relationships for PolyEdges connected to given vertices of given relation<pVertex, pOtherVertex>
      SmBoolean bFixed;
      SER(FixRelationsAtVertex(pBrepVertex,  // in : Brep vertex   of relationship(pVertex, pOtherVertex)
                               pOtherVertex, // in : Other vertex  of relationship(pVertex, pOtherVertex)
                               bFixed));     // out: TRUE = made changes, FALSE = didn't

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); this->m_pBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); this->m_pOther->Draw(); sm_GraphicsLoop();
          DrawRelatedObjects(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

  } // end pBrepVertex had a pre-existing, conflicting relationship check 

  // when pOtherVertex had a pre-existing, conflicting relationship - remove the relationship and squeeze the vertices
  if ( pMateOfOtherV != NULL && pMateOfOtherV != pBrepVertex )
    {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        { 
          Dump() ;
          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0); pMateOfOtherV->GetPoint().Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); pOtherVertex->GetPoint().Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,1); pBrepVertex->GetPoint().Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); this->m_pBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); this->m_pOther->Draw(); sm_GraphicsLoop();
          DrawRelatedObjects(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // remove stale undesired vertex relationship
      SER(RemoveRelationship(pMateOfOtherV,pOtherVertex));
      
      // pMateOfOtherV and pBrepVertex are probably close - see if they can be squeezed
      double dDist = pMateOfOtherV->GetPoint().DistanceBetween(pBrepVertex->GetPoint());
      double dTol  = pMateOfOtherV->GetTolerance() + pBrepVertex->GetTolerance();
      if (dDist > dTol * 50.0)
        { SER(SM_ERR); }

      SM_PTR_ARRAY(sFaces,SmPolyFace,32);
      SM_PTR_ARRAY(sDeletedEdges,SmPolyEdge,32);
      SM_PTR_ARRAY(sDeletedV,    SmPolyVertex, 16) ;
      SM_PTR_ARRAY(sSurvivingV,  SmPolyVertex, 16) ;

      SER(m_pBrep->CombineCoincidentVertices(pBrepVertex,     // in : PolyVertex to remain after combining
                                             pMateOfOtherV,   // in : PolyVertex to delete after combining - stale after this call
                                             sDeletedEdges,   // out: List of stale deleted PolyEdge pointers
                                                              //       (when Verts connect to a commonPolyFace but not a CommonPolyEdge,
                                             sFaces,          //        these PolyEdges are 1st created and then deleted in this method)
                                                              // out: List of PolyFaces attached to deleted PolyEdges
                                                              //       (some of these may be stale)
                                             &sDeletedV,      // out: optional array of additional deleted PolyVertices, NULL to ignore, default:[NULL]
                                             &sSurvivingV)) ; // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

      // accumulate output - stale vertex relationship already removed
      rSurvivingV.Add(pBrepVertex);
      rDeletedV.Add(pMateOfOtherV);

      rDeletedV.Append(sDeletedV) ;
      rSurvivingV.Append(sSurvivingV) ;

      // for every deleted edge
      ULONG lNumDeletedEdges = sDeletedEdges.GetSize();
      for (ii=0; ii<lNumDeletedEdges; ii++) 
        {
          SmPolyEdge *pDeletedBrepEdge = sDeletedEdges[ii];
          if (pDeletedBrepEdge) 
            {
              // accumulate output
              rDeletedE.Add(pDeletedBrepEdge); // there are no surviving Edges from this call
              rSurvivingE.Add(NULL) ;

              // and remove obsolete relationships
              SmPolyEdge *pDeletedOtherEdge = (SmPolyEdge*)GetOtherMate(pDeletedBrepEdge); // m_pBToO
              SER(RemoveRelationship(pDeletedBrepEdge,pDeletedOtherEdge));
            }
        } // end iter every DeleteEdge - updating output and removing stale relationships

      // iter every changed PolyFace - squeezing PolyFaces that are degenerate
      ULONG lNumFaces = sFaces.GetSize();
      for (ii=0; ii<lNumFaces; ii++)
        {
          SmPolyFace * pF = sFaces[ii];
          SmPolyEdge * pDelEdge1 = NULL, *pDelEdge2 = NULL, *pEdgeSurvive1 = NULL, *pEdgeSurvive2 = NULL;
          SmBoolean    bFaceWasSqueezed = FALSE;
          SmTArray<SmPolyVertex*> sDeletedVertices ; 
          SmTArray<SmPolyVertex*> sSurvivingVertices ; 

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pBrepVertex->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pF->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // squeeze face when it contains only two PolyEdges and glue any attached radial partners
          //   gwc - should be extended to more general cases: face too small, sliver face, etc.
          SER(pF->GetPolyBrep()->SqueezeFaceIfDegenerate
               (pF,                           // in : target PolyFace to delete
                pBrepVertex->GetTolerance(),  // in : not used
                bFaceWasSqueezed,             // out: TRUE=PolyFace was squeezed, FALSE=not
                pDelEdge1,                    // out: STALE pointer to deleted PolyEdge1, NULL=none deleted, don't use as an object
                pDelEdge2,                    // out: STALE pointer to deleted PolyEdge2, NULL=none deleted, don't use as an object
                pEdgeSurvive1,                // out: Surviving radial partner to deleted PolyEdge1, NULL=none
                pEdgeSurvive2,               // out: Surviving radial partner to deleted PolyEdge2, NULL=none
                &sDeletedVertices,           // out: optional array of deleted PolyVertices, NULL to ignore, default:[NULL]
                &sSurvivingVertices)) ;      // out: optional associated surviving PolyVertices, NULL to ignore, default:[NULL]

          if (bFaceWasSqueezed) 
            {
              rDeletedE.Add(pDelEdge1);
              rDeletedE.Add(pDelEdge2);
              rSurvivingE.Add(pEdgeSurvive1);
              rSurvivingE.Add(pEdgeSurvive2);
              m_vDeletedFaces.Add(pF);
              rDeletedV.Append(sDeletedVertices) ;
              rSurvivingV.Append(sSurvivingVertices) ;

              // replace relationships of pDelEdge1 stale pointers
              //pBrepEdgeSurvive = pEdgeSurvive1;
              SmPolyEdge *pDeletedOtherEdge = (SmPolyEdge*)GetOtherMate(pDelEdge1); // m_pBToO
              if (pDeletedOtherEdge) 
                {
                  SER(RemoveRelationship(pDelEdge1,pDeletedOtherEdge));
                  if (pEdgeSurvive1 && GetOtherMate(pEdgeSurvive1) == NULL) // m_pBToO
                    {
                      SER(Relate(pEdgeSurvive1,pDeletedOtherEdge));
                    }
                }

              // replace relationships of pDelEdge2 stale pointers
              pDeletedOtherEdge = (SmPolyEdge*)GetOtherMate(pDelEdge2);  // m_pBToO
              if (pDeletedOtherEdge) 
                {
                  SER(RemoveRelationship(pDelEdge2,pDeletedOtherEdge));
                  if (pEdgeSurvive2 && GetOtherMate(pEdgeSurvive2) == NULL) // m_pBToO
                    {
                      SER(Relate(pEdgeSurvive2,pDeletedOtherEdge));
                    }   
                }

              // replace relationships of sDeletedVertices stale pointers
              for(jj=0;jj<sDeletedVertices.GetSize();jj++)
                {
                  SmPolyVertex *pDeletedOtherVertex = (SmPolyVertex*)GetOtherMate(sDeletedVertices[jj]); // m_pBToO
                  if(pDeletedOtherVertex)
                    {
                      SER(RemoveRelationship(sDeletedVertices[jj], pDeletedOtherVertex));
                      if(sSurvivingVertices[jj] && GetOtherMate(sSurvivingVertices[jj]) == NULL) // m_pBToO
                        {
                          SER(Relate(sSurvivingVertices[jj], pDeletedOtherVertex));
                        }
                    } // end DeletedVertex has a relationship check  
                } // end iter every DeletedVertex looking for relationships to delete or replace
            } // end polyFace was squeezed check
        } // end iter all affected faces looking to squeeze degenerate ones

      // Fix relationships for PolyEdges connected to given vertices of given relation<pVertex, pOtherVertex>
      SmBoolean bFixed;
      SER(FixRelationsAtVertex(pBrepVertex,  // in : Brep vertex   of relationship(pVertex, pOtherVertex)
                               pOtherVertex, // in : Other vertex  of relationship(pVertex, pOtherVertex)
                               bFixed));     // out: TRUE = made changes, FALSE = didn't
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0); this->m_pBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); this->m_pOther->Draw(); sm_GraphicsLoop();
          DrawRelatedObjects(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end if pOtherVertex had a conflicting relationship.

  // all done
  return SM_SUCCESS;

} // end SmPolyIntersector::CheckForSqueezeVertices

/*******************************************************************//**
PURPOSE: Remove Deleted faces and adjacent edges/vertices from
    the merge PolyBrep.

NOTES: 
***********************************************************************/
SmStatus SmPolyIntersector::DeleteFaces(const SmTArray<SmPolyFace*> & crFaces)
{
  // Remove Deleted portion of the PolyBrep

  SmPolyBrep *pPolyBrep = NULL;

  SmPolyEdge *sPEData[128];
  SmTArray<SmPolyEdge*> sEdges(128,sPEData);

  ULONG ii;
  for (ii=0; ii<crFaces.GetSize(); ii++) {
      SmPolyFace *pFace = crFaces[ii];
      SmPolyFace *pOFace = (SmPolyFace*)GetOtherMate(pFace); // m_pBToO
      if (pOFace) {
          RemoveRelationship(pFace,pOFace);
      }
      pPolyBrep = pFace->GetPolyBrep();
      pFace->GetPolyEdges(sEdges);
      for (ULONG kk=0; kk<sEdges.GetSize(); kk++)
      {
          SmPolyEdge * pEdge = sEdges[kk];
          // If this is the last edge at this vertex then remove
          // vertex relationships.
          if (pEdge->GetNextPolyEdgeAtV() == pEdge)
          {
              SmPolyVertex *pV = pEdge->GetStartPolyVertex();
              SmPolyVertex *pOV = SM_REINTERPRET_CAST(SmPolyVertex*,GetOtherMate(pV)); // m_pBToO
              if (pOV) {
                  RemoveRelationship(pV,pOV);
              }
          }
          SmPolyEdge * pRadialE = pEdge->GetRadial();
          if (pRadialE == pEdge) {
              pRadialE = NULL; 
          }
          if (pRadialE && pRadialE->GetPolyFace() == pFace) {
              pRadialE = NULL;
          }
          SmPolyEdge * pOEdge = SM_REINTERPRET_CAST(SmPolyEdge*,GetOtherMate(pEdge)); // m_pBToO
          if (pOEdge) {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
              if (bDebugMe0) {
                  smgfx_SetLook(1,2, 1,0,0);
                  pEdge->Draw();
                  sm_GraphicsLoop();
              }
#endif
              RemoveRelationship(pEdge,pOEdge);
              if (pRadialE) {
                  Relate(pRadialE,pOEdge);
              }
          }
      }
      pPolyBrep->DeletePolyFace(pFace);
  }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      pPolyBrep->Dump();
      pPolyBrep->ValidatePointers();
      DrawRelatedObjects();
      sm_GraphicsLoop();
  }
#endif

  return SM_SUCCESS;

} // end DeleteFaces

/*******************************************************************//**
PURPOSE: Fix relationships for PolyEdges connected to given vertices 
         of given relation<pVertex, pOtherVertex> 

NOTES: Makes sure that all Brep PolyEdges connected to pVertex
       have a relationship to the associated Other PolyEdges
       connected to pOtherVertex when those PolyEdge endpoints
       have relationships. 
***********************************************************************/
SmStatus SmPolyIntersector::FixRelationsAtVertex
 (SmPolyVertex * pVertex,      // in : Brep vertex   of relationship(pVertex, pOtherVertex)
  SmPolyVertex * pOtherVertex, // in : Other vertex  of relationship(pVertex, pOtherVertex)
  SmBoolean    & rbFixed)      // out: TRUE = made changes, FALSE = didn't
{
  // locals
  ULONG j ;
  SM_PTR_ARRAY(sVertEdges,SmPolyEdge,64);
  pVertex->GetStartingPolyEdges(sVertEdges);

  // for every PolyEdge starting a pVertex
  for(j=0; j<sVertEdges.GetSize(); j++) 
    {
      SmPolyEdge   * pE          = sVertEdges[j];
      SmPolyVertex * pV2         = pE->GetEndPolyVertex();
      SmPolyFace   * pCommonFace = NULL;
      SmPolyEdge   * pCommonEdge = NULL;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          if (j==0) 
            {
              smgfx_SetLook(3,5, 0,0,1); pVertex->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(3,5, 0,1,0); pOtherVertex->GetPoint().Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
          smgfx_SetLook(2,4, 1,0,0); pE->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // find any PolyEdge or PolyFace connected to both bVertex and pV2 (in pBrep)   
      pVertex->FindPolyEdgeOrFaceBetween(pV2,pCommonEdge,pCommonFace);
      
      // skip cases not connected by a common PolyEdge (expect to find the pvertex<->pE<->pV2 structure 
      if (pCommonEdge == NULL) 
        { continue; }
      
      // look for relations for CommonEdge and pV2  
      SmPolyEdge   * pOtherEdge = (SmPolyEdge*)GetOtherMate(pCommonEdge); // m_pBToO
      SmPolyVertex * pOtherV1   = pOtherVertex;
      SmPolyVertex * pOtherV2   = (SmPolyVertex*)GetOtherMate(pV2); // m_pBToO

      // skip cases where PolyVertices have no relationships
      if (pOtherV2 == NULL || pOtherV1 == NULL) 
        { continue; }
        
      // find any PolyEdge or PolyFace connected to both pOtherV1 and pOtherV2 (in pOther)   
      SmPolyFace * pOtherCommonFace = NULL;
      SmPolyEdge * pOtherCommonEdge = NULL;
      pOtherV1->FindPolyEdgeOrFaceBetween(pOtherV2,pOtherCommonEdge,pOtherCommonFace);
        
      // skip cases without a commonEdge in pOther (expect to find the pOtherV1<->pOtherEdge<->pOtherV2 structure
      if (pOtherCommonEdge == NULL) 
        { continue; }

      // skip cases where OtherEdge and OherCommonEdge are the same - everything is okay
      if (pOtherEdge == pOtherCommonEdge) 
        { continue; }

      // else remove the OtherEdge relationship
      if (pOtherEdge) 
        { RemoveRelationship(pCommonEdge,pOtherEdge); }
      
      // make the desired realtionship<pCommonEdge, pOtherCommonEdge>  
      Relate(pCommonEdge,pOtherCommonEdge);

      // update output
      rbFixed = TRUE;

    } // end iter every PolyEdge starting a pVertex

  // all done
  return SM_SUCCESS;

} // end SmPolyIntersector::FixRelationsAtVertex

/*******************************************************************//**
PURPOSE: Fix gaps in the intersection loop by squeezing together 
    vertices.

NOTES: The vertices in the array are from m_pBrep.
***********************************************************************/
SmStatus SmPolyIntersector::FixGaps
 (SmTArray<SmPolyVertex*> & crSingleVertices,
  SmBoolean               & rbModifiedTopology)
{
    rbModifiedTopology = FALSE;
    // Look for gaps at about 100 times the tolerance of the vertex
    // and under.
    SM_PTR_ARRAY(sFaces,SmPolyFace,32);
    SM_PTR_ARRAY(sDeletedEdges,SmPolyEdge,32);
    SM_PTR_ARRAY(sDeletedOtherEdges,SmPolyEdge,32);

    ULONG ii, jj, lNumSingleVerts = crSingleVertices.GetSize();
    SmStatus eStat;

    // The best way to fix a gap is to look at adjacent edges 
    // Now let's try to connect the dots and get non-connected edges connected.
    {
        for (ii=0; ii<lNumSingleVerts; ii++)
        {
            SmPolyVertex *pV1 = crSingleVertices[ii];
            if (!pV1) continue;
            SmPolyVertex *pV1Other = (SmPolyVertex*)GetOtherMate(pV1); // m_pBToO
            if (!pV1Other) continue;
            SmBoolean bFixed = FALSE;
            // Fix relationships for PolyEdges connected to given vertices of given relation<pVertex, pOtherVertex>
            SER(FixRelationsAtVertex(pV1,      // in : Brep vertex   of relationship(pVertex, pOtherVertex)
                                     pV1Other, // in : Other vertex  of relationship(pVertex, pOtherVertex)
                                     bFixed)); // out: TRUE = made changes, FALSE = didn't
            if (bFixed) { 
                rbModifiedTopology = TRUE; 
//                crSingleVertices[i] = NULL;
            }
        }
    }


    for (ii=0; ii<lNumSingleVerts; ii++)
    {
        SmPolyVertex *pV1 = crSingleVertices[ii];
        if (!pV1) continue;
        double dMinDistBetween = SM_BIG_DOUBLE;
        SmPolyVertex *pV2Found = NULL;
        ULONG lFound=0;
        for (jj=ii+1; jj<lNumSingleVerts; jj++) 
          {
            SmPolyVertex *pV2 = crSingleVertices[jj];
            if (!pV2) { continue; }
            double dDistBetween = pV1->GetPoint().DistanceBetween(pV2->GetPoint());
            double dTol = 100.0 * (pV1->GetTolerance() + pV2->GetTolerance());
            if (   dDistBetween < dMinDistBetween 
                && dDistBetween < dTol) 
              {
                dMinDistBetween = dDistBetween;
                pV2Found = pV2;
                lFound = jj;
              }
          } // end inner loop on single vertices

        if (pV2Found)
        {
            SmPolyVertex *pV2 = pV2Found;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                smgfx_Erase();
                smgfx_SetLook(2,4, 0,1,1); pV1->GetPoint().Draw(); sm_GraphicsLoop();
                smgfx_SetLook(2,4, 1,0,1); pV2->GetPoint().Draw(); sm_GraphicsLoop();
                smgfx_SetLook(1,2, 0,0,0); m_pBrep->Draw();  sm_GraphicsLoop();
                smgfx_SetLook(1,2, 0,0,1); m_pOther->Draw(); sm_GraphicsLoop();
                DrawRelatedObjects(); sm_GraphicsLoop();
                sm_GraphicsLoop();
            }
#endif
            // Have a good canidate for squeezing
            SmPolyVertex *pOtherV1 = (SmPolyVertex*)GetOtherMate(pV1); NER(pOtherV1); // m_pBToO
            SmPolyVertex *pOtherV2 = (SmPolyVertex*)GetOtherMate(pV2); NER(pOtherV2); // m_pBToO
            
            crSingleVertices[ii] = NULL;
            crSingleVertices[lFound] = NULL;
            SER(m_pBrep->CombineCoincidentVertices (pV1,               // in : PolyVertex to remain after combining
                                                    pV2,               // in : PolyVertex to delete after combining - stale after this call
                                                    sDeletedEdges,     // out: List of stale deleted PolyEdge pointers
                                                                       //       (when Verts connect to a commonPolyFace but not a CommonPolyEdge,
                                                                       //        these PolyEdges are 1st created and then deleted in this method)
                                                    sFaces));          // out: List of PolyFaces attached to deleted PolyEdges
                                                                       //       (some of these may be stale)
            SER(m_pOther->CombineCoincidentVertices(pOtherV1,          // in : PolyVertex to remain after combining
                                                    pOtherV2,          // in : PolyVertex to delete after combining - stale after this call
                                                    sDeletedOtherEdges,// out: List of stale deleted PolyEdge pointers
                                                                       //       (when Verts connect to a commonPolyFace but not a CommonPolyEdge,
                                                                       //        these PolyEdges are 1st created and then deleted in this method)
                                                    sFaces));          // out: List of PolyFaces attached to deleted PolyEdges
                                                                       //       (some of these may be stale)
            RemoveRelationship(pV2,pOtherV2);
            
            ULONG kk, lNumDelEdges = sDeletedEdges.GetSize();
            for (kk=0; kk<lNumDelEdges; kk++) {
                SmPolyEdge *pOtherE = (SmPolyEdge*)GetOtherMate(sDeletedEdges[kk]); // m_pBToO
                if (pOtherE) {
                    RemoveRelationship(sDeletedEdges[kk],pOtherE);
                }
            }
            
            rbModifiedTopology = TRUE;

        } // end if pV2Found, a close vertex.
    } // end outer loop on single vertices


    // Now let's try to connect the dots and get non-connected edges connected.
    {
        for (ii=0; ii<lNumSingleVerts; ii++)
        {
            SmPolyVertex *pV1 = crSingleVertices[ii];
            if (!pV1) { continue; }
            double dMinDistBetween = SM_BIG_DOUBLE;
            SmPolyVertex *pV2Found = NULL;
            //ULONG lFound;
            for (jj=ii+1; jj<lNumSingleVerts; jj++) {
                SmPolyVertex *pV2 = crSingleVertices[jj];
                if (!pV2) continue;
                double dDistBetween = pV1->GetPoint().DistanceBetween(pV2->GetPoint());
                if (dDistBetween < dMinDistBetween)
                {
                    dMinDistBetween = dDistBetween;
                    pV2Found = pV2;
                    //lFound = jj;
                }
            }
            if (!pV2Found) { continue; }
            
            SmPolyVertex *pV2 = pV2Found;
            SmPolyFace *pCommonFace = NULL;
            SmPolyEdge *pCommonEdge = NULL;
            pV1->FindPolyEdgeOrFaceBetween(pV2,pCommonEdge,pCommonFace);

            if (pCommonEdge == NULL && pCommonFace == NULL) { continue; }
            
            SmPolyVertex *pOtherV1 = (SmPolyVertex*)GetOtherMate(pV1); NER(pOtherV1); // m_pBToO
            SmPolyVertex *pOtherV2 = (SmPolyVertex*)GetOtherMate(pV2); NER(pOtherV2); // m_pBToO

            SmPolyFace *pOtherCommonFace = NULL;
            SmPolyEdge *pOtherCommonEdge = NULL;
            pOtherV1->FindPolyEdgeOrFaceBetween(pOtherV2,pOtherCommonEdge,pOtherCommonFace);

            if (pOtherCommonEdge == NULL && pOtherCommonFace == NULL) { continue; }

            // If make it here we have a common edge or face for both.
            if (pCommonFace)
            {
                SmPolyBrep *pBrep = pCommonFace->GetPolyBrep();
                SmPolyLoop *pNewL = NULL;
                SmPolyFace *pNewF = NULL;
                double dEdgeTol = pCommonFace->GetTolerance();
                eStat = pBrep->MakeEdgeInFace(pCommonFace,pV1,pV2,
                    dEdgeTol,pCommonEdge,pNewL,pNewF);
                SER( eStat ); // SER is necessary. [B105]
            }
            if (pOtherCommonFace)
            {
                SmPolyBrep *pBrep = pOtherCommonFace->GetPolyBrep();
                SmPolyLoop *pNewL = NULL;
                SmPolyFace *pNewF = NULL;
                double dEdgeTol = pOtherCommonFace->GetTolerance();
                eStat = pBrep->MakeEdgeInFace(pOtherCommonFace,pOtherV1,pOtherV2,
                    dEdgeTol,pOtherCommonEdge,pNewL,pNewF);
                SER( eStat ); // SER is necessary. [B105]
            }
            if (pOtherCommonEdge && pCommonEdge) {
                Relate(pCommonEdge,pOtherCommonEdge);
                rbModifiedTopology = TRUE;
            }
            else {
                SE(SM_ERR);
                continue;
            }
            
        } // end loop on all single vertices
    } // end scope, connecting the dots


    return SM_SUCCESS;

} // end SmPolyIntersector::FixGaps

/*******************************************************************//**
PURPOSE: Remove the relationship between a PolyBrep entity and the
    other entity.

NOTES: 
***********************************************************************/
SmStatus SmPolyIntersector::RemoveRelationship
 (SmTopology * pPolyBrepEntity,
  SmTopology * pOtherEntity)
{
  m_pBToO->Remove(pPolyBrepEntity);
  m_pOToB->Remove(pOtherEntity);
  return SM_SUCCESS;

} // end SmPolyIntersector::RemoveRelationship

/*******************************************************************//**
PURPOSE: Get the Other PolyBrep entity from the related PolyBrep topology entity.
   If there is no relationship NULL is returned.

NOTES: fetches relation from m_pBToO
***********************************************************************/
SmObject * SmPolyIntersector::GetOtherMate
 (SmObject * pPolyBrepEntity) 
 const
{
  if (m_pBToO) 
    { return m_pBToO->At(pPolyBrepEntity); }
  else 
    { return NULL; }

} // end SmPolyIntersector::GetOtherMate

/*******************************************************************//**
PURPOSE: Get the PolyBrep entity from the related Other PolyBrep topology entity.
   If there is no relationship NULL is returned.

NOTES: fetches relation from m_pOToB
***********************************************************************/
SmObject * SmPolyIntersector::GetBrepMate
 (SmObject * pOtherEntity) 
 const
{
  if (m_pOToB) 
    { return m_pOToB->At(pOtherEntity); }
  else 
    { return NULL; }

} // end SmPolyIntersector::FixGaps

/*******************************************************************//**
PURPOSE: Get a list of the edges which are common edges between the
    two PolyBreps.

NOTES: The other pointer may be deleted. 
                Stale pointers may remain for history
***********************************************************************/
void SmPolyIntersector::GetCommonEdges
 (SmTArray<SmPolyEdge*> & rEdges, 
  SmTArray<SmPolyEdge*> & rOtherEdges) 
 const
{
  rEdges.ReSet();
  rOtherEdges.ReSet();

  SmPolyEdge *sPEData[256];
  SmTArray<SmPolyEdge*> sEdges(256,sPEData);
  m_pBrep->GetPolyEdges(sEdges);

  ULONG ii;
  for(ii=0; ii<sEdges.GetSize(); ii++) 
    {
      SmPolyEdge *pEdge = sEdges[ii];
      SmPolyEdge *pOEdge = (SmPolyEdge*)GetOtherMate(pEdge); // m_pBToO 
      if (pOEdge && pOEdge->GetPolyLoop()) 
        {  // This fixes a crash when pOEdge exists but it has no owner.
           // There may be a better solution.  [120910]
          rEdges.Add(pEdge);
          rOtherEdges.Add(pOEdge);
        }
    }
} // end SmPolyIntersector::GetCommonEdges

/*******************************************************************//**
PURPOSE: Get a list of the vertices which are common vertices between the
    two PolyBreps.

NOTES: 
***********************************************************************/
void SmPolyIntersector::GetCommonVertices
 (SmTArray<SmPolyVertex*> & rVertices, 
  SmTArray<SmPolyVertex*> & rOtherVertices) 
 const
{
  rVertices.ReSet();
  rOtherVertices.ReSet();

  SmPolyVertex *sPVData[256];
  SmTArray<SmPolyVertex*> sVertices(256,sPVData);
  m_pBrep->GetPolyVertices(sVertices);

  ULONG ii;
  for (ii=0; ii<sVertices.GetSize(); ii++) 
    {
      SmPolyVertex *pVertex = sVertices[ii];
      SmPolyVertex *pOVertex = (SmPolyVertex*)GetOtherMate(pVertex); // m_pBToO
      if (pOVertex) 
        {
          rVertices.Add(pVertex);
          rOtherVertices.Add(pOVertex);
        }
    }
} // end SmPolyIntersector::GetCommonVertices

/*******************************************************************//**
PURPOSE: Intersect the entities of these two PolyBreps, insert them into
    the corresponding topologies and set up the relationships.

NOTES: 
***********************************************************************/
SmStatus SmPolyIntersector::IntersectInsertRelate()
{
  // Go with a really large cache for this operation.

#ifdef SM_VALIDATE_TOPOLOGY
  smos_WriteBuffer(_T("SmPolyIntersector - Validate Topology is ON!"));
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if ( bDebugMe ) 
    {
      m_pBrep->Dump();
      m_pOther->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0); m_pOther->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG idx;
  SmExtent3d     sBBox1, sBBox2 ;
  SmExtent3d     sFBBox, sCommonBox, sBBoxF2 ;
  SmExtent3d     sIntersectBBox;
  SmPolyFace   * sPFData[64];
  SmObjectList * sOLData1[64];
  SmTArray<SmPolyFace*>   sPolyFaces(64,sPFData);
  SmTArray<SmObjectList*> sObjs1(64,sOLData1);

  m_pBrep->GetPolyFaces(sPolyFaces);
  m_pBrep->CalculateBoundingBox(sBBox1);
  m_pOther->CalculateBoundingBox(sBBox2);

  ULONG       lNumPerBlock = smos_Max(sPolyFaces.GetSize()/10,100);
  SmTree    * pFaceTree1   = new (m_crContext) SmTree(sBBox1,lNumPerBlock,lNumPerBlock/10);    
  SmObjDelete sClean(pFaceTree1);

  // for every PolyFace - get BBox and add to SpatialTree
  for (ULONG k=0; k<sPolyFaces.GetSize(); k++) 
    {
      SmPolyFace *pPF = sPolyFaces[k];
      SER(pPF->CalculateBoundingBox(sFBBox));
      SER(pFaceTree1->AddToSpatialTree(sFBBox,pPF,SM_NG_POLYGON));
    }

  // get common intersection = Intersect(sBBox1, sBBox2)
  if (sBBox1.AreDisjoint(sBBox2)) 
    { return SM_SUCCESS; }
  SER(sBBox1.Intersect(sBBox2,sCommonBox));
  sCommonBox.ExpandAbsolute(m_dThisApproxTol3d); //cbi already was expanded...

  // get Other PolyFaces
  m_pOther->GetPolyFaces(sPolyFaces);
  ULONG ii, lNumFaces = sPolyFaces.GetSize(); 

  // for Every OtherPolyFace do intersections
  for (ii=0; ii<lNumFaces; ii++)
    {
      SmPolyFace *pFace2 = sPolyFaces[ii];
      if (pFace2 == NULL) 
        { continue; }

      if (m_vOtherDeletedFaces.FindElement(pFace2,idx)) 
        {
          m_vOtherDeletedFaces.RemoveAt(idx,1); //cbi: why?
          sPolyFaces[ii] = NULL;
          continue;
        }

      pFace2->CalculateBoundingBox(sBBoxF2);
      if (sBBoxF2.AreDisjoint(sCommonBox)) 
        { continue; }

      //
      if(m_bTrimWithPlane )
        {
          // Determine if the spherical bounds intersected with trimming plane
          SmPoint3d sSphCenter;
          double    dSphRadius;
          double    dDistToPlane;

          sBBoxF2.ComputeSphereBound(sSphCenter,dSphRadius);
          SER(smgu_PlanePointDistance(m_vTrimPlanePoint,
                                      m_vTrimPlaneNormal,
                                      sSphCenter,
                                      dDistToPlane));
          if ( dDistToPlane > dSphRadius+SM_EFF_ZERO )
            { continue; }
        }

      //
      pFaceTree1->GetObjectListInBox(sBBoxF2,sObjs1);
      ULONG jj, lNumObjs = sObjs1.GetSize();

      // for every object in sBBoxF2
      for (jj=0; jj<lNumObjs; jj++)
       {
          SmObjectList * pOL = sObjs1[jj];
          if (pOL->m_pObject == NULL)
            { continue; }

          //
          if (m_vDeletedFaces.FindElement((SmPolyFace*)pOL->m_pObject,idx))
            {
              pOL->m_pObject = NULL;
              continue;
            }

          SmPolyFace * pPF1= SM_CAST_PTR(SmPolyFace,pOL->m_pObject);
          if (!pPF1)
            { continue; }

          //
          SER( pOL->m_sBBox.Intersect( sBBoxF2, sIntersectBBox ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE; // do any debug at all
SmBoolean bDebugMe3 = FALSE; // draw PolyBreps
int       iDebugMe4 = 0;     // dump 'this' and Draw() (if 1) or DrawRelatedObjects() (if 2).
SmBoolean bDrawDebug= FALSE; // Draw() or DrawDebug()
ULONG lStop_ii = SM_BIG_ULONG;
ULONG lStop_jj = SM_BIG_ULONG;
          if (bDebugMe2 || ii == lStop_ii || jj == lStop_jj ) 
            {
              SmPolyFace * pCFace      = pPF1->GetCPolyFace() ? pPF1->GetCPolyFace() : pPF1 ;
              SmPolyFace * pCOtherFace = pFace2->GetCPolyFace() ? pFace2->GetCPolyFace() : pFace2 ;

              if ( iDebugMe4 > 0 )
               { this->Dump( TRUE ); }

              if ( bDebugMe3 ) 
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1); m_pBrep->Draw(TRUE); sm_GraphicsLoop();
                  smgfx_SetLook(2,4, 0,1,0); m_pOther->Draw(TRUE); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
              if ( iDebugMe4 == 1 ) 
                {
                  SmVector3d sColor1 ( 0,1,1 );
                  SmVector3d sColor2 ( 1,0,1 );
                  SmVector3d sOffset1( 0,1,0 );
                  SmVector3d sOffset2( 0,1.1,0 );
                  smgfx_SetLook(4,7, 1,0,0);
                  this->Draw( &sOffset1, &sOffset2, &sColor1, &sColor2 ); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
              if ( iDebugMe4 == 2 ) 
                {
                  smgfx_SetLook(2,4, 1,0,0); this->DrawRelatedObjects(); sm_GraphicsLoop();
                }
              if ( bDrawDebug )
                {
                  smgfx_SetLook(2,4, 1,0,1); pCFace     ->DrawDebug(); sm_GraphicsLoop();
                  smgfx_SetLook(3,5, 0,1,1); pCOtherFace->DrawDebug(); sm_GraphicsLoop();
                }
              else
                {
                  smgfx_SetLook(2,4, 1,0,1); pCFace     ->Draw(1,1,1,1); sm_GraphicsLoop();
                  smgfx_SetLook(3,5, 0,1,1); pCOtherFace->Draw(1,1,1,1); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

          if(SM_SUCCESS != IIRFaces( pPF1, pFace2, &sIntersectBBox ))
            { continue; }

          if ( m_vOtherDeletedFaces.FindElement( pFace2, idx ) )
            {
              pFace2 = sPolyFaces[ii] = NULL;
              break;    // out of inner for loop (jj)
            }
        } // end iter every object in sBBoxF2
    } // end iter every OtherPolyface

  // Done Surface/Surface Intersection
  return SM_SUCCESS;

} // end SmPolyIntersector::IntersectInsertRelate()

/*******************************************************************//**
PURPOSE: Merge two LineSegClassifications into the corresponding PolyBreps.  

NOTES: this method does all of the homoginization, syncronization and 
       clean up of the classifications prior to merge.
***********************************************************************/
SmStatus SmPolyIntersector::MergeLineSegClasses
 (SmLineSegClassification & rClass1,             // i/o: homogenized, CleanupAndValidate
  SmLineSegClassification & rClass2,             // i/o:
  SmBoolean               & rbDeletedTopology)   // out:
{
#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      rClass1.Dump();
      rClass2.Dump();
      this->Dump(TRUE) ; 

      smgfx_Erase();
      smgfx_SetLook(2,4, 1,0,0); rClass1  .Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 1,0,1); rClass2  .Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); m_pBrep ->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0); m_pOther->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  rbDeletedTopology = FALSE;

  // Syncronize the two curve classifications
  if(SM_SUCCESS != rClass1.Homogenize(rClass2)) 
    { return SM_ERR; }
        
  // We will come back and do this later
  if(SM_SUCCESS != rClass1.CleanupAndValidate(rClass2)) 
    { return SM_ERR; }

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lCount == lDebugCount) 
    {
      rClass1.Dump();
      rClass2.Dump();
      this->Dump(TRUE) ;
    }
#endif // SM_DEBUG_CODE

  // Now check for special case where there is one classification
  // and it is on a related edge.
//    if (rClass1.GetSize() == 1) {
//        SmLineSegInterval & rIvl1 = rClass1[0];
//        SmLineSegInterval & rIvl2 = rClass2[0];
//        SmPolyPointClassification & rMid1 = rIvl1.m_vMid;
//        SmPolyPointClassification & rMid2 = rIvl2.m_vMid;
//
//        if (rMid1.GetPointClass() == SM_PPC_EDGE &&
//            (rIvl1.m_vStart.GetPointClass() == SM_PPC_EDGE ||
//            rIvl1.m_vEnd.GetPointClass() == SM_PPC_EDGE) ) {
//            double dLeng = rClass1[0].m_vInterval.GetLength();
//            dLeng *= rClass1.m_cLineVec.Length();
//            SmPolyEdge *pE = (SmPolyEdge*)rMid1.GetObject();
//            double dLeng2 = pE->GetStartPoint().DistanceBetween(pE->GetEndPoint());
//            // Syncronize the two curve classifications
//            if (dLeng < dLeng2 / 20.0) {
//                return SM_SUCCESS;
//            }
//        }
//        if (rMid2.GetPointClass() == SM_PPC_EDGE &&
//            (rIvl2.m_vStart.GetPointClass() == SM_PPC_EDGE ||
//            rIvl2.m_vEnd.GetPointClass() == SM_PPC_EDGE) ) {
//            double dLeng = rClass2[0].m_vInterval.GetLength();
//            dLeng *= rClass2.m_cLineVec.Length();
//            SmPolyEdge *pE = (SmPolyEdge*)rMid2.GetObject();
//            double dLeng2 = pE->GetStartPoint().DistanceBetween(pE->GetEndPoint());
//            if (dLeng < dLeng2 / 20.0) {
//                return SM_SUCCESS;
//            }
//        }
//    }

  // locals
  SmPolyEdge * sEData[16];
  SmPolyEdge * sEDataOther[16];
  SmTArray<SmPolyEdge*> sNewEdges(16,sEData);
  SmTArray<SmPolyEdge*> sNewEdgesOther(16,sEDataOther);

  // Now merge classification data into each PolyBrep
  if(SM_SUCCESS != rClass1.MergeClassifications
                     (rClass2,          // in : classification of curve against Other brep
                      NULL,             // out: New Brep  Faces made by merge
                      NULL,             // out: New Other Faces made by merge
                      &sNewEdges,       // out: New Brep  Edges made by merge
                      &sNewEdgesOther)) // out: New Other Edges made by merge
    { return SM_ERR; }

#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lCount == lDebugCount) 
    {
      rClass1.Dump();
      rClass2.Dump();
      this->Dump(TRUE) ;
    }
#endif // SM_DEBUG_CODE


  // Look for edges which are related and may have been split.
  // If found try to match them up back again .

  // locals
  SM_PTR_ARRAY(sNewEdges2,SmPolyEdge,64);
  SM_PTR_ARRAY(sNewEdgesOther2,SmPolyEdge,64);
  sNewEdges2.Append(sNewEdges);
  sNewEdgesOther2.Append(sNewEdgesOther);
  ULONG ii, lNumEdges  = sNewEdges.GetSize();
  ULONG lNumEdgesOther = sNewEdgesOther.GetSize();

  for (ii=0; ii<lNumEdges+lNumEdgesOther; ii++)
    {
      // for the Edge and its mate
      SmPolyEdge * pE      = NULL;
      SmPolyEdge * pOtherE = NULL;
      if(ii<lNumEdges) { pE      = sNewEdges[ii];
                         pOtherE = (SmPolyEdge*)GetOtherMate(pE); // m_pBToO
                       }
      else             { pOtherE = sNewEdgesOther[ ii - lNumEdges ];
                         pE      = (SmPolyEdge*)GetBrepMate(pOtherE); // m_pOToB
                       }

      // remove relationship<pE,pE->Mate> and add pE and pOtherE to edges2 lists
      if(pE != NULL && pOtherE != NULL) 
        {
          RemoveRelationship(pE,pOtherE);
          sNewEdges2.AddUnique(pE);
          sNewEdgesOther2.AddUnique(pOtherE);
        }

      // for Edge->Radial partner and its mate
      pOtherE = NULL;
      pE      = NULL;
      if ( ii < lNumEdges ) { pE      = sNewEdges[ii]->GetRadial();
                              pOtherE = pE ? (SmPolyEdge*)GetOtherMate(pE) : NULL ; // m_pBToO
                            }
      else                  { pOtherE = sNewEdgesOther[ ii - lNumEdges ]->GetRadial();
                              pE      = pOtherE ? (SmPolyEdge*)GetBrepMate(pOtherE) : NULL ;  // m_pOToB
                            }
        
      // remove relationship<pE->Radial,pE->Radial->mate> and add pE and pOtherE to edges2 lists
      if (pE != NULL && pOtherE != NULL) 
        {
          RemoveRelationship(pE,pOtherE);
          sNewEdges2.AddUnique(pE);
          sNewEdgesOther2.AddUnique(pOtherE);

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetLook(2,4, 0,0,1); pE->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,4, 0,1,0); pOtherE->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
              SM_DUMP_TARRAY( sNewEdges      );
              SM_DUMP_TARRAY( sNewEdgesOther );
            }
#endif // SM_DEBUG_CODE

        } // end need to remove relationship<pE->Radial,pE->Radial->mate> check
    } // end ii loop on all sNewEdges and sNewEdgesOther to move relationships into edge lists

  ULONG jj, lNumEdges2  = sNewEdges2.GetSize();
  ULONG lNumEdgesOther2 = sNewEdgesOther2.GetSize();

  // look for <Edge, OtherEdge> coincident pairs

  // For every Edge placed into the edge lists
  //  see about relating coincident Edges returned from MergeClassifications()
  //  plus the ones that might have been added because of split edges.

  // for every Other edge - check for <edge, OtherEdge> coincidence (Coincident midpoint and endpoints)
  for(ii=0; ii<lNumEdges2; ii++)
    {
      SmPolyEdge * pE       = sNewEdges2[ii];
      SmPoint3d    sEndPnt  = pE->GetEndPoint();
      SmPoint3d    sStartPt = pE->GetStartPoint();
      SmPoint3d    sMidPnt;
      pE->EvaluatePoint(0.5,sMidPnt);

      // for every OtherEdge - drop edge midoint onto other edge looking for possible coincidence
      for (jj=0; jj<lNumEdgesOther2; jj++)
        {
          SmPolyEdge * pOE = sNewEdgesOther2[jj];
          double dParam, dDist;

          // drop edge->MidPoint onto OtherEdge
          SER(pOE->DropPoint(sMidPnt,dParam,dDist));
          double dTol = pOE->GetTolerance() + pE->GetTolerance();

          // when midPoint drop dist was small
          if (dDist < dTol) 
            {
              // and end points are close together
              if(   sEndPnt.DistanceBetween(pOE->GetEndPoint()) < dTol
                 && sStartPt.DistanceBetween(pOE->GetStartPoint()) < dTol) 
                {
                  // remember the coincidence
                  if (Relate(pE,pOE) != SM_SUCCESS) 
                    { MSG(_T("Possible Problem Relating Edges")); }
                }

              // else when start/end endpoint pairs are close together
              if(   sStartPt.DistanceBetween(pOE->GetEndPoint()) < dTol
                 && sEndPnt.DistanceBetween(pOE->GetStartPoint()) < dTol) 
                {
                  // remember the coincidence
                  if (Relate(pE,pOE) != SM_SUCCESS) 
                    { MSG(_T("Possible Problem Relating Edges")); }
                } // end end points are within tol check
            } // end midpoints are within tol check
        } // end iter every OtherEdge
    } // end iter every Edge looking for coincident pairs

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3) 
    {
      rClass1.Dump();
      rClass2.Dump();
      this->Dump(TRUE) ;

      smgfx_Erase();
      smgfx_SetLook(2,4, 1,0,0); DrawRelatedObjects(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE


  // for every LineSegInterval - find and relate coincident start-Vertices, mid-Edges, and end-Vertices

  // locals
  ULONG lIdx, lNumIvls  = rClass1.GetSize();
  SmBoolean bDidLastEnd = FALSE;
  SM_PTR_ARRAY(sDeletedV, SmPolyVertex, 64) ;
  SM_PTR_ARRAY(sSurvivingV, SmPolyVertex, 64) ;
  SM_PTR_ARRAY(sDeletedEdges, SmPolyEdge, 64) ;  
  SM_PTR_ARRAY(sSurvivingEdges, SmPolyEdge, 64) ;  

  for (jj=0; jj<lNumIvls; jj++)
    {
      SmLineSegInterval & rClassIvl1 = rClass1[jj];
      SmLineSegInterval & rClassIvl2 = rClass2[jj];

      // Check Start vertices.
      SmPolyVertex * pStartV = NULL;
      SmPolyVertex * pEndV   = NULL;
      if(   rClassIvl1.m_vStart.GetPointClass() == SM_PPC_VERTEX
         && rClassIvl2.m_vStart.GetPointClass() == SM_PPC_VERTEX)
        {
          pStartV = (SmPolyVertex*)rClassIvl1.m_vStart.GetObject();

          if ( sDeletedV.FindElement( pStartV, lIdx ) )
            { pStartV = NULL; }  // [B109]

          // If we already did the start vtx as the end of the previous segment,
          // skip it.
          if (!bDidLastEnd)
            {
              SmPolyVertex * pV1 = (SmPolyVertex*)rClassIvl1.m_vStart.GetObject();
              SmPolyVertex * pV2 = (SmPolyVertex*)rClassIvl2.m_vStart.GetObject();

              // First, if either vertex has been deleted,
              // replace it with its survivor.
              ULONG lFoundIndex;
              while (sDeletedV.FindElement(pV1,lFoundIndex)) 
                {
                  pV1 = sSurvivingV[lFoundIndex];
                  rClassIvl1.m_vStart.SetClassObject(SM_PPC_VERTEX, pV1);
                }

              while (sDeletedV.FindElement(pV2,lFoundIndex)) 
                {
                  pV2 = sSurvivingV[lFoundIndex];
                  rClassIvl2.m_vStart.SetClassObject(SM_PPC_VERTEX, pV2);
                }

              // only relate vertices that still exist
              if(pV1 != NULL && pV2 != NULL)
                {
                  // Update tolerances.
                  double dDist = pV1->GetPoint().DistanceBetween(pV2->GetPoint());
                  if (dDist*2.0 > pV1->GetTolerance()) { pV1->SetTolerance(dDist*2.0); }
                  if (dDist*2.0 > pV2->GetTolerance()) { pV2->SetTolerance(dDist*2.0); }

                  // squeeze pBrep  PolyVertices near to pV1 with PV1 and
                  // squeeze pOther PolyVertices near to pV2 with pV2
                  CheckForSqueezeVertices( pV1,               // in : PolyVertex from m_pBrep
                                           pV2,               // in : PolyVertex from m_pOther
                                           sDeletedV,         // i/o: accumulating list of stale PolyVertices
                                           sSurvivingV,       // i/o: associated list of PolyVertices replacing stale PolyVertices (NULL entry = no replacement)
                                           sDeletedEdges,     // i/o: accumulating list of stale PolyEdges
                                           sSurvivingEdges,   // i/o: accumulating list of PolyEdges replacing stale ptrs (NULL entry = no replacement)
                                           TRUE );            // in : TRUE  = Check if current BrepVertex and OtherVertex mates are closer than 
                                                              //              than BrepVertex and OtherVertex are to eachother
                                                              //      FALSE = Don't.
                                                              //      Use FALSE when squeezing degenerate edge

                  if(sDeletedEdges.GetSize() != 0)
                    { rbDeletedTopology = TRUE; }

#ifdef SM_VALIDATE_TOPOLOGY
                  m_pBrep->ValidatePointers();
                  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

                  while (sDeletedV.FindElement(pV1,lFoundIndex)) 
                    {
                      pV1 = sSurvivingV[lFoundIndex];
                      rClassIvl1.m_vStart.SetClassObject(SM_PPC_VERTEX, pV1);
                    }

                  while (sDeletedV.FindElement(pV2,lFoundIndex)) 
                    {
                      pV2 = sSurvivingV[lFoundIndex];
                      rClassIvl2.m_vStart.SetClassObject(SM_PPC_VERTEX, pV2);
                    }

                  pStartV = pV1;

                  if ( pV1 != NULL && pV2 != NULL )
                    {
                      if(Relate(pV1,pV2) != SM_SUCCESS) 
                        { MSG(_T("Possible Problem Relating Vertices")); }
                    }

                } // end both PolyVertices still exist check

#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 1,0,0); pV1->GetPoint().Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,4, 0,1,1); pV2->GetPoint().Draw(); sm_GraphicsLoop();
                  SmPolyVertex *pVOther = (SmPolyVertex*)GetOtherMate(pV1); // m_pBToO
                  smgfx_SetLook(4,6, 0,0,0);
                  if (pVOther) { pVOther->GetPoint().Draw(); } sm_GraphicsLoop();
                  smgfx_SetLook(6,8, 0,1,0);
                  SmPolyVertex *pBV = (SmPolyVertex*)GetBrepMate(pV2); // m_pOToB
                  if (pBV) { pBV->GetPoint().Draw(); } sm_GraphicsLoop();
                  sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            } // end not already done on last iteration check
        } // end both classification ivls start on PolyVertices check

      // Check Mid Edges.
      if (   rClassIvl1.m_vMid.GetPointClass() == SM_PPC_EDGE
          && rClassIvl2.m_vMid.GetPointClass() == SM_PPC_EDGE)
        {
          SmPolyEdge *pE1 = (SmPolyEdge*)rClassIvl1.m_vMid.GetObject();
          SmPolyEdge *pE2 = (SmPolyEdge*)rClassIvl2.m_vMid.GetObject();

#ifdef SM_DEBUG_CODE
#ifdef USE_DEBUG_COUNTER
          if (pE1->m_lDebugCount == 0) pE1->m_lDebugCount = s_lMLSOFCount;
          if (pE2->m_lDebugCount == 0) pE2->m_lDebugCount = s_lMLSOFCount;        
#endif // USE_DEBUG_COUNTER
#endif // SM_DEBUG_CODE

          // First, if either Edge has been deleted,
          // replace it with its survivor.
          ULONG lFoundIndex;
          while (sDeletedEdges.FindElement(pE1,lFoundIndex)) 
            {
              pE1 = sSurvivingEdges[lFoundIndex];
              rClassIvl1.m_vMid.SetClassObject(SM_PPC_VERTEX, pE1);
            }

          while (sDeletedEdges.FindElement(pE2,lFoundIndex)) 
            {
              pE2 = sSurvivingEdges[lFoundIndex];
              rClassIvl2.m_vMid.SetClassObject(SM_PPC_VERTEX, pE2);
            }

          // only relate PolyEdges that have not been deleted
          if(pE1 != NULL && pE2 != NULL) 
            {
             if(SM_SUCCESS != Relate(pE1,pE2)) 
               { MSG(_T("Possible Problem Relating Edges")); }
            }

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              SmPolyEdge * pBE     = (SmPolyEdge*)GetBrepMate(pE2);  // m_pOToB
              SmPolyEdge * pEOther = (SmPolyEdge*)GetOtherMate(pE1); // m_pBToO

              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pE1->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(3,4, 0,0,1); pE2->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,1,0); if(pEOther) pEOther->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();

              smgfx_Erase();
              smgfx_SetLook(3,4, 1,0,0); if(pBE) pBE->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); if(m_pBrep ) m_pBrep ->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,0); if(m_pOther) m_pOther->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end both classification ivl mids classify to PolyEdges check 

      // Check End vertices.
      if (   rClassIvl1.m_vEnd.GetPointClass() == SM_PPC_VERTEX
          && rClassIvl2.m_vEnd.GetPointClass() == SM_PPC_VERTEX)
        {
          SmPolyVertex * pV1 = (SmPolyVertex*)rClassIvl1.m_vEnd.GetObject();
          SmPolyVertex * pV2 = (SmPolyVertex*)rClassIvl2.m_vEnd.GetObject();

          ULONG lFoundIndex;
          while(sDeletedV.FindElement(pV1,lFoundIndex)) 
            {
              pV1 = sSurvivingV[lFoundIndex];
              rClassIvl1.m_vEnd.SetClassObject(SM_PPC_VERTEX, pV1);
            }

          while(sDeletedV.FindElement(pV2,lFoundIndex)) 
            {
              pV2 = sSurvivingV[lFoundIndex];
              rClassIvl2.m_vEnd.SetClassObject(SM_PPC_VERTEX, pV2);
            }

          // only relate vertices that still exist
          if(pV1 != NULL && pV2 != NULL)
            {
              double dDist = pV1->GetPoint().DistanceBetween(pV2->GetPoint());
              if (dDist*2.0 > pV1->GetTolerance()) { pV1->SetTolerance(dDist*2.0); }
              if (dDist*2.0 > pV2->GetTolerance()) { pV2->SetTolerance(dDist*2.0); }

#ifdef SM_DEBUG_CODE
#ifdef USE_DEBUG_COUNTER
              if (pV1->m_lDebugCount == 0) { pV1->m_lDebugCount = s_lMLSOFCount; }
              if (pV2->m_lDebugCount == 0) { pV2->m_lDebugCount = s_lMLSOFCount; }
#endif // USE_DEBUG_COUNTER
#endif // SM_DEBUG_CODE

              // squeeze pBrep PolyVertices near to pV1 with PV1 and
              // squeeze pOther PolyVertices near to pV2 with pV2
              CheckForSqueezeVertices( pV1,               // in : PolyVertex from m_pBrep
                                       pV2,               // in : PolyVertex from m_pOther
                                       sDeletedV,         // i/o: accumulating list of stale PolyVertices
                                       sSurvivingV,       // i/o: associated list of PolyVertices replacing stale PolyVertices (can have NULL entries)
                                       sDeletedEdges,     // i/o: accumulating list of stale PolyEdges
                                       sSurvivingEdges,   // i/o: accumulating list of PolyEdges replacing stale ptrs (NULL entry = no replacement)
                                       TRUE );            // in : TRUE  = Check if current BrepVertex and OtherVertex mates are closer than 
                                                          //              than BrepVertex and OtherVertex are to eachother
                                                          //      FALSE = Don't.
                                                          //      Use FALSE when squeezing degenerate edge

              while(sDeletedV.FindElement(pV1,lFoundIndex)) 
                {
                  pV1 = sSurvivingV[lFoundIndex];
                  rClassIvl1.m_vEnd.SetClassObject(SM_PPC_VERTEX, pV1);
                }

              while(sDeletedV.FindElement(pV2,lFoundIndex)) 
                {
                  pV2 = sSurvivingV[lFoundIndex];
                  rClassIvl2.m_vEnd.SetClassObject(SM_PPC_VERTEX, pV2);
                }

              pEndV = pV2;

              if ( pV1 != NULL && pV2 != NULL )
                {
                  if(Relate(pV1,pV2) != SM_SUCCESS) 
                    { MSG(_T("Possible Problem Relating Vertices")); }
                }

            } // end both PolyVertices still exist check

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              rClass1.Dump() ;
              rClass2.Dump() ;
              Dump() ;
              SmPolyVertex * pVOther = (SmPolyVertex*)GetOtherMate(pV1); // m_pBToO
              SmPolyVertex * pBV     = (SmPolyVertex*)GetBrepMate(pV2);  // m_pOToB

              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pV1->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,4, 0,1,1); pV2->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,6, 0,0,0); if(pVOther) pVOther->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(6,8, 0,1,0); if(pBV) pBV->GetPoint().Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Signal not to check the next Start vtx.
          bDidLastEnd = TRUE;
        }
      else 
        {
          bDidLastEnd = FALSE;
        }

      // Check for Degenerate edge - pStartV from pBrep, pEndV from pOther
      if ( pStartV && pEndV )
        {
          if(   !sDeletedV.FindElement( pStartV, lIdx )  // [B109]
             && !sDeletedV.FindElement( pEndV,   lIdx ) )
            {
              double dDist = pStartV->GetPoint().DistanceBetween(pEndV->GetPoint());

              if (dDist < pStartV->GetTolerance()+pEndV->GetTolerance())
                {
                  // squeeze pBrep PolyVertices near to pV1 with PV1 and
                  // squeeze pOther PolyVertices near to pV2 with pV2
                  CheckForSqueezeVertices( pStartV,           // in : PolyVertex from m_pBrep
                                           pEndV,             // in : PolyVertex from m_pOther
                                           sDeletedV,         // i/o: accumulating list of stale PolyVertices
                                           sSurvivingV,       // i/o: associated list of PolyVertices replacing stale PolyVertices (can have NULL entries)
                                           sDeletedEdges,     // i/o: accumulating list of stale PolyEdges
                                           sSurvivingEdges,   // i/o: accumulating list of PolyEdges replacing stale ptrs (NULL entry = no replacement)
                                           FALSE );           // in : TRUE  = Check if current BrepVertex and OtherVertex mates are closer than 
                                                              //              than BrepVertex and OtherVertex are to eachother
                                                              //      FALSE = Don't.
                                                              //      Use FALSE when squeezing degenerate edge



                }
            }
        }
      if(   sDeletedEdges.GetSize() > 0
         || sDeletedV    .GetSize() > 0)
        {
          rbDeletedTopology = TRUE;
        }
    } // End iter every LineSegClassification interval

#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

  return SM_SUCCESS;

} // end SmPolyIntersector::MergeLineSegClasses

/*******************************************************************//**
PURPOSE:  Set a line-seg class to be as if the line-seg is the same as the edge.

NOTES:
***********************************************************************/
static SmStatus sm_SetEdgeClass
 (SmLineSegClassification & rClass, // i/o:
  SmPolyEdge              * pEdge)  // in :
{
  SmLineSegInterval & rIvl = rClass[0];
  rIvl.m_vStart.SetClassObject(SM_PPC_VERTEX, pEdge->GetStartPolyVertex());
  rIvl.m_vStart.SetDeviation(0.0);

  rIvl.m_vMid.SetClassObject(SM_PPC_EDGE, pEdge);
  rIvl.m_vMid.SetDeviation(0.0);
  rIvl.m_vMid.SetTParam(0.5);

  rIvl.m_vEnd.SetClassObject(SM_PPC_VERTEX, pEdge->GetEndPolyVertex());
  rIvl.m_vEnd.SetDeviation(0.0);

  return SM_SUCCESS;
} // end static sm_SetEdgeClass

/*******************************************************************//**
PURPOSE: Merge a line segment which is on two faces of two different Breps.

NOTES: The face and other face may be NULL if the 
    corresponding brep edge or other brep edge are not NULL.
    This is an optimization that allows us to skip classification
    if we already know the curve represents an edge in one of the
    breps.
***********************************************************************/
SmStatus SmPolyIntersector::MergeLineSegOnFaces
 (SmPolyFace       * pFace,
  SmPolyFace       * pOtherFace,
  double             dTolerance,
  const SmPoint3d  & crLinePnt,
  const SmVector3d & crLineVec,
  SmPolyEdge       * pBrepEdge,
  SmPolyEdge       * pOtherEdge,
  SmBoolean        & rbDeletedTopology)
{
#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif

  rbDeletedTopology = FALSE;

  if (pFace == NULL || pOtherFace == NULL)
    { SER(SM_ERR); }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe    = FALSE;
static ULONG     lDebugCount = 0 ;
  s_lMLSOFCount ++;
  if (bDebugMe || s_lMLSOFCount == lDebugCount ) 
    {
      smgfx_Erase();
      smgfx_SetLook(3,5, 1,0,0); crLineVec.Draw(&crLinePnt); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); if(pBrepEdge) pBrepEdge->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_SetLook(2,3, 0,0,1); if (pOtherFace) pOtherFace->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,0,1); if (pOtherEdge) pOtherEdge->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0);
      if(bDebugMe)
      {
        if(pFace && pFace->GetType() == SmPolyFace_TYPE) { pFace->GetPolyBrep()->Draw(); sm_GraphicsLoop(); }
        smgfx_SetLook( 1, 2, 1, 1, 0 ); if(pOtherEdge) { pOtherEdge->GetPolyBrep()->Draw(); sm_GraphicsLoop(); }
        sm_GraphicsLoop();
      }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    
  // Classify the line seg on each face
  SmLineSegInterval sLSI1[16];
  SmLineSegInterval sLSI2[16];
  SmLineSegClassification sClass1(crLinePnt,crLineVec,dTolerance,16,sLSI1);
  SmLineSegClassification sClass2(crLinePnt,crLineVec,dTolerance,16,sLSI2);

  if (pBrepEdge)  { SER(sm_SetEdgeClass(sClass1,pBrepEdge)); }
  else            { SER(pFace->LineSegOnClassify(TRUE,sClass1)); }
    
  if (pOtherEdge) { SER(sm_SetEdgeClass(sClass2,pOtherEdge)); }
  else            { SER(pOtherFace->LineSegOnClassify(TRUE,sClass2)); }
    
#if 0            
  double dNewTol = dTolerance;
  SmPoint3d sNewLinePnt;
  SmVector3d sNewLineVec;
  SmBoolean bCreateNewLine = FALSE;
  //        SER(sClass1.TestForPartialCoincidentEdges(sClass2,dNewTol,
  //            bCreateNewLine,sNewLinePnt,sNewLineVec));
  //        if (dNewTol < 0.0) SER(SM_ERR);
  if (bCreateNewLine) {
      SmLineSegClassification sNewClass1(sNewLinePnt,sNewLineVec,dNewTol);
      SER(pFace->LineSegOnClassify(TRUE,sNewClass1));
        
      SmLineSegClassification sNewClass2(sNewLinePnt,sNewLineVec,dNewTol);
      SER(pOtherFace->LineSegOnClassify(TRUE,sNewClass2));
        
      SER(MergeLineSegClasses(sNewClass1,sNewClass2,rbDeletedTopology));
      // We still have some 3D curve left over Classify again 
      SmLineSegClassification sClass12(p3DCurve,dCurveTolerance);
      if (pBrepEdge) {
          SER(sm_SetEdgeClass(sClass12,pBrepEdge));
      }
      else {
          SER(pFace->LineSegOnClassify(TRUE,sClass12));
      }
      SmLineSegClassification sClass22(p3DCurve,,dCurveTolerance);
      if (pOtherEdge) {
          SER(sm_SetEdgeClass(sClass22,pOtherEdge));
      }
      else {
          SER(pOtherFace->LineSegOnClassify(TRUE,sClass22));
      }
      SER(MergeLineSegClasses(sClass12,sClass22,FALSE,rbDeletedTopology));
      return SM_SUCCESS;
  }
#endif // 0
    
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3) 
    {
      sClass1.Dump() ;
      sClass2.Dump() ;
      GetBrep()->Dump() ; 
      GetOther()->Dump() ;
      this->Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0); DrawRelatedObjects(); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,1,1); sClass1  .Draw(); sm_GraphicsLoop();
      smgfx_SetLook(4,6, 1,0,1); sClass2  .Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); m_pBrep ->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0); m_pOther->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    
  // merge intervals from Classifications into m_pBrep and m_pOther
  SER(MergeLineSegClasses(sClass1,sClass2,rbDeletedTopology));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      GetBrep()->Dump() ; 
      GetOther()->Dump() ;
      this->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0); this->DrawRelatedObjects(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,0,1); m_pOther->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

  return SM_SUCCESS;

} // end SmPolyIntersector::MergeLineSegOnFaces

/*******************************************************************//**
PURPOSE: Merge coincident faces by projecting edges of each onto the other.

NOTES: Someday we may want to add vertices to this process.

       increments unlocked marks in pBFace and pOFace contexts
***********************************************************************/
SmStatus SmPolyIntersector::MergeCoincidentFaces
 (SmPolyFace * pBFace,
  SmPolyFace * pOFace)
{
  SmPolyFace * pCFace  = pBFace->GetCPolyFace() ? pBFace->GetCPolyFace() : pBFace ;
  SmPolyFace * pCOFace = pOFace->GetCPolyFace() ? pOFace->GetCPolyFace() : pOFace ;

  SmPolyEdge * sEData1[64];
  SmPolyEdge * sEData2[64];
  SmTArray<SmPolyEdge*> sEdgesB(64,sEData1);
  SmTArray<SmPolyEdge*> sEdgesO(64,sEData2);
    
  // Compute the bounding boxes and intersections
  SmExtent3d sThisFaceBBox, sOtherFaceBBox;
  SER(pCFace->CalculateBoundingBox(sThisFaceBBox));
  SER(pCOFace->CalculateBoundingBox(sOtherFaceBBox));
  sThisFaceBBox.ExpandAbsolute(m_dThisApproxTol3d);
  if (sThisFaceBBox.AreDisjoint(sOtherFaceBBox)) 
    {
      // Sorry we don't intersect.
      return SM_SUCCESS;
    }

  // intersect the bounding boxes
  SmExtent3d sIntersectionBBox;
  SER(sThisFaceBBox.Intersect(sOtherFaceBBox,sIntersectionBBox));
    
  SmBoolean bDeletedTopology = TRUE;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); pCFace ->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(4,6, 1,0,0); pCOFace->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
   }
#endif // SM_DEBUG_CODE

  // increment the context->m_lCurrentMark2 value
  SmNewMarkAndLock sBrepMarkLock ( m_pBrep->GetContext(),  SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmNewMarkAndLock sOtherMarkLock( m_pOther->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eBrepMarkType  = sBrepMarkLock.GetMarkType() ;
  SmMarkType eOtherMarkType = sOtherMarkLock.GetMarkType() ;

  ULONG ii, idx;

  //
  while (bDeletedTopology)
    {
      bDeletedTopology = FALSE;

      // MergeLineSegOnFaces() can delete faces.
      if ( m_vDeletedFaces.FindElement( pBFace, idx ) )
        { break; }
      if ( m_vOtherDeletedFaces.FindElement( pOFace, idx ) )
        { break; }

      pCFace = pBFace->GetCPolyFace() ? pBFace->GetCPolyFace() : pBFace ; 
      pCFace->GetPolyEdges(sEdgesB);
      ULONG lNumEdgesB = sEdgesB.GetSize();

      //
      for (ii=0; ii<lNumEdgesB; ii++)
        {
          SmPolyEdge *pBEdge = sEdgesB[ii];

          // If marked, skip it.
          if (pBEdge->IsMarked(eBrepMarkType)) { continue; }

          // If already related, skip it.
          SmPolyEdge *pMate = (SmPolyEdge*)GetOtherMate( pBEdge ); // m_pBToO

          // Also check all radial partners.
          SmPolyEdge *pRad = pBEdge->GetRadial();
          while ( pMate == NULL && pRad != NULL && pRad != pBEdge )
            {
              pMate = (SmPolyEdge*)GetOtherMate( pRad ); // m_pBToO
              pRad = pRad->GetRadial();
            }
          if (pMate)
            { continue; } // Already mated.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
          if (bDebugMe3) 
            {
              smgfx_Erase();
              smgfx_SetLook(3,5, 1,0,0); pBEdge->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,6, 0,0,1); pOFace->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
            
          SmExtent3d sEdgeBBox;
          SER(pBEdge->CalculateBoundingBox(sEdgeBBox));
          sEdgeBBox.ExpandAbsolute(pBEdge->GetTolerance()); //cbi Already was expanded.

          // skip disjoint BBoxes
          if (sEdgeBBox.AreDisjoint(sIntersectionBBox)) 
            {
              pBEdge->Mark(eBrepMarkType);
              continue;
            }

          double dTol = pOFace->GetTolerance() + pBEdge->GetTolerance();
          dTol = smos_Max(dTol,m_dThisApproxTol3d);

          SmPoint3d  sLinePnt;
          SmVector3d sLineVec;
          pBEdge->GetLine(sLinePnt,sLineVec);
            
          ULONG lNumPolyBrepEdges = sEdgesB.GetSize();
          pCFace  = pBFace->GetCPolyFace() ? pBFace->GetCPolyFace() : pBFace ;
          pCOFace = pOFace->GetCPolyFace() ? pOFace->GetCPolyFace() : pOFace ;

          //
          SER(MergeLineSegOnFaces(pCFace, 
                                  pCOFace, 
                                  dTol,
                                  sLinePnt, 
                                  sLineVec, 
                                  pBEdge, 
                                  NULL, 
                                  bDeletedTopology));

          if (bDeletedTopology)
            { break; }  // out of loop on ii, sEdgesB

          // Check to make sure there are the same number of edges otherwise we have
          // to jump out here because pBEdge may no longer correspond to the geometry.
          pCFace->GetPolyEdges(sEdgesB);
          if (sEdgesB.GetSize() != lNumPolyBrepEdges) 
            { bDeletedTopology = TRUE; }

          if (bDeletedTopology) { break; }  // out of loop on ii, sEdgesB
          pBEdge->Mark(eBrepMarkType);  // Have completed working with pBEdge mark it
        }
        
      if (bDeletedTopology) { continue; }
        
      pCOFace = pOFace->GetCPolyFace() ? pOFace->GetCPolyFace() : pOFace ;
      pCOFace->GetPolyEdges(sEdgesO);
      ULONG lNumEdgesO = sEdgesO.GetSize();

      //
      for (ii=0; ii<lNumEdgesO; ii++)
        {
          SmPolyEdge * pOEdge = sEdgesO[ii];
          if (pOEdge->IsMarked(eOtherMarkType)) { continue; }
          SmPolyEdge *pBEdge = (SmPolyEdge*)GetBrepMate(pOEdge); // m_pOToB
          if (pBEdge) { continue; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
          if (bDebugMe3)
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pOEdge->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,6, 0,0,1); pBFace->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          SmExtent3d sEdgeBBox;
          SER(pOEdge->CalculateBoundingBox(sEdgeBBox));
          sEdgeBBox.ExpandAbsolute(pOEdge->GetTolerance());

          // skip disjoint BBoxes
          if (sEdgeBBox.AreDisjoint(sIntersectionBBox)) 
            {
              pOEdge->Mark(eOtherMarkType);
              continue;
            }
            
          SmPoint3d  sLinePnt;
          SmVector3d sLineVec;
          pOEdge->GetLine(sLinePnt,sLineVec);
            
          double dTol = pBFace->GetTolerance() + pOEdge->GetTolerance();
          dTol = smos_Max(dTol,m_dThisApproxTol3d);
          ULONG lOEdgesCount = sEdgesO.GetSize();
            
          pCFace  = pBFace->GetCPolyFace() ? pBFace->GetCPolyFace() : pBFace ;
          pCOFace = pOFace->GetCPolyFace() ? pOFace->GetCPolyFace() : pOFace ;
          SER(MergeLineSegOnFaces(pCFace, 
                                  pCOFace,
                                  dTol,
                                  sLinePnt, 
                                  sLineVec, 
                                  NULL, 
                                  pOEdge, 
                                  bDeletedTopology));

          if (bDeletedTopology) { break; }  // out of loop on ii, sEdgesO

          // Check to make sure edge count is same otherwise jump out
          pCOFace->GetPolyEdges(sEdgesO);
          if (sEdgesO.GetSize() != lOEdgesCount) 
            {
              bDeletedTopology = TRUE;
            }

          if (bDeletedTopology) { break; }  // out of loop on ii, sEdgesO
          pOEdge->Mark(eOtherMarkType);
      }
  } // While (bDeletedTopology)    
    
#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif
    
  return SM_SUCCESS;

} // end SmPolyIntersector::MergeCoincidentFaces

/*******************************************************************//**
PURPOSE: Intersect, Insert and Relate two PolyFaces from the corresponding PolyBreps.

NOTES: 
***********************************************************************/
SmStatus SmPolyIntersector::IIRFaces
 (SmPolyFace       * pFace, 
  SmPolyFace       * pOtherFace,
  const SmExtent3d * cpIntersectionBBox)
{
  SmPolyFace * pCFace      = pFace->GetCPolyFace() ? pFace->GetCPolyFace() : pFace ;
  SmPolyFace * pCOtherFace = pOtherFace->GetCPolyFace() ? pOtherFace->GetCPolyFace() : pOtherFace ;

  // Compute tolerance.  Use sum, not max.  [B156]
  double dTol = pCFace->GetTolerance() + pCOtherFace->GetTolerance();
  dTol        = smos_Max(dTol, m_dThisApproxTol3d);  //[B209]

  // look for intersecting bounding boxes
  SmExtent3d sIntersectionBBox;
  if (!cpIntersectionBBox) 
    {
      SmExtent3d sThisFaceBBox, sOtherFaceBBox;

      // Compute the bounding boxes and intersections
      SER(pCFace->CalculateBoundingBox(sThisFaceBBox));
      SER(pCOtherFace->CalculateBoundingBox(sOtherFaceBBox));
      sThisFaceBBox.ExpandAbsolute(dTol);
      if (sThisFaceBBox.AreDisjoint(sOtherFaceBBox)) 
        {
          // Sorry we don't intersect.
          return SM_SUCCESS;
        }
      SER(sThisFaceBBox.Intersect(sOtherFaceBBox,sIntersectionBBox));
    }
  else 
    {
      sIntersectionBBox = *cpIntersectionBBox;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
ULONG lCount      = 1 ; lCount++ ;
ULONG lDebugCount = 0 ;
SmBoolean bDrawDebug=FALSE;
  if (bDebugMe || lCount == lDebugCount) 
    {
      m_pBrep->ValidatePointers();
      m_pOther->ValidatePointers();
      this->Dump();

      smgfx_Erase();
      if ( bDrawDebug )
        { this->DrawRelatedObjects();                          sm_GraphicsLoop();
          smgfx_SetLook(2,4, 0,0,1); pCFace     ->DrawDebug(); sm_GraphicsLoop();
          smgfx_SetLook(3,5, 0,1,0); pCOtherFace->DrawDebug(); sm_GraphicsLoop();
        }

      else
        { smgfx_SetLook(2,4, 0,0,1); pCFace     ->Draw(1,1,1,1); sm_GraphicsLoop();
          smgfx_SetLook(3,5, 0,1,0); pCOtherFace->Draw(1,1,1,1); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();

      if (bDebugMe) 
        { smgfx_SetLook(1,2, 0,0,0); m_pBrep ->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,1); m_pOther->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // Intersect surfaces
  SmBoolean bAreCoincident = FALSE;
  double    dCoincidentDistance = 0.0;
  SER(pFace->CoincidenceCheck(*pOtherFace,
                               dTol,
                               bAreCoincident,
                               dCoincidentDistance));
  SmPoint3d  sLinePnt;
  SmVector3d sLineVec;
  if (!bAreCoincident) 
    {
      // intersect the faces
      if(SM_SUCCESS != pCFace->IntersectWithPolyFace(*pCOtherFace,
                                                     sLinePnt, 
                                                     sLineVec)) 
        {
          // This is just the parallel face situation - or the intersection curve
          // does not intersect the bounding box of the faces.
          return SM_SUCCESS;
          // WARN(_T("PolyFace/PolyFace Intersection Error - Proceeding"));
        }
    }

  if(bAreCoincident) 
    {
      SER(MergeCoincidentFaces(pCFace,pCOtherFace));  // increments unlocked marks in pBFace and pOFace contexts
    }
  else 
    {
      SmExtent3d sLineSegBBox(sLinePnt);
      sLineSegBBox.AddPoint3d(sLinePnt+sLineVec);
      sLineSegBBox.ExpandAbsolute(dTol);
      if (sLineSegBBox.AreDisjoint(sIntersectionBBox)) 
        { return SM_SUCCESS; }        
        
#ifdef SM_DEBUG_CODE
// SmBoolean bDebugMe1 = FALSE;
      if (bDebugMe) 
        {
          smgfx_Erase() ;
          smgfx_SetLook(4,6, 1,0,0); sLineVec.Draw(&sLinePnt); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

constexpr double cbiFactor = 4.0;  //cbi: at least 2, probably 4. No bigger than 45. [B464]
      // Test for small edge cases
      double dLeng = sLineVec.Length();
      if (dLeng < cbiFactor * dTol) 
        {
          WARN(_T("Removing a small intersection curve"));
          return SM_SUCCESS;
        }
        
      SmBoolean bDeletedTopology;
      if(SM_SUCCESS != MergeLineSegOnFaces(pCFace,
                                           pCOtherFace,
                                           dTol,
                                           sLinePnt,
                                           sLineVec,
                                           NULL,
                                           NULL,
                                           bDeletedTopology) )
        {
          if (bDeletedTopology) { m_bPolyDeleted = TRUE; }

          SER(MergeLineSegOnFaces(pCFace,
                                  pCOtherFace,
                                  dTol,
                                  sLinePnt,
                                  sLineVec,
                                  NULL,
                                  NULL,
                                  bDeletedTopology));
          if (bDeletedTopology) { m_bPolyDeleted = TRUE; }
        }

      if (bDeletedTopology) m_bPolyDeleted = TRUE;
#ifdef SM_VALIDATE_INTERSECTIONS
      Validate();
#endif

    }

#ifdef SM_VALIDATE_TOPOLOGY
  m_pBrep->ValidatePointers();
  m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
    
  return SM_SUCCESS;

} // end SmPolyIntersector::IIRFaces

/*******************************************************************//**
PURPOSE: Relate these two topology objects.

NOTES: 
***********************************************************************/
#if 0
SmStatus SmPolyIntersector::Relate(SmObject *pBrepTopo, 
                                   SmObject *pOtherTopo)
{

NOTE: ifdef-ed out version: the only difference is that here,
in case of a conflict (already mated to a different entity),
for PolyVertices and PolyEdges only, that pre-existing relation
is removed and replaced with the requested one.

    SmObject *pOth = GetOtherMate(pBrepTopo); // m_pBToO
    if (pBrepTopo->IsKindOf(SmPolyFace_TYPE) &&
        pOth && pOth != pOtherTopo) {
        return SM_ERR;
    }
    SmObject *pBrp = GetBrepMate(pOtherTopo);  // m_pOToB
    if (pOtherTopo->IsKindOf(SmPolyFace_TYPE) &&
        pBrp && pBrp != pBrepTopo) {
        return SM_ERR;
    }

    if (pBrepTopo->IsKindOf(SmPolyVertex_TYPE)) {
        SmPolyVertex *pV1 = SM_CAST_PTR(SmPolyVertex,pBrepTopo); NER(pV1);
        SmPolyVertex *pV2 = SM_CAST_PTR(SmPolyVertex,pOtherTopo); NER(pV2);
        SmPolyVertex *pOtherV = (SmPolyVertex*)GetOtherMate(pV1); // m_pBToO
        if (pOtherV) {
            if (pOtherV != pV2) {
                RemoveRelationship(pV1,pOtherV);
                SE(SM_ERR);
            }
        }
        SmPolyVertex *pV = (SmPolyVertex*)GetBrepMate(pV2); // m_pOToB
        if (pV) {
            if (pV != pV1) {
                RemoveRelationship(pV,pV2);
                SE(SM_ERR);
            }
        }
        if (pV1->GetTolerance() < pV2->GetTolerance()) { pV1->SetTolerance(pV2->GetTolerance());}
        if (pV2->GetTolerance() < pV1->GetTolerance()) { pV2->SetTolerance(pV1->GetTolerance());}
    }

    if (pBrepTopo->IsKindOf(SmPolyEdge_TYPE)) {
        SmPolyEdge *pE1 = SM_CAST_PTR(SmPolyEdge,pBrepTopo); NER(pE1);
        SmPolyEdge *pE2 = SM_CAST_PTR(SmPolyEdge,pOtherTopo); NER(pE2);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
        if (bDebugMe1) {
            pE1->Dump();
            pE2->Dump();
            smgfx_ChangeLook();
            smgfx_SetLook( 3,5, 0,1,0 ); pE1->Draw(); sm_GraphicsLoop();
            smgfx_SetLook( 4,6, 1,0,1 ); pE2->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        // See if they have been already related
        SmPolyEdge *pE = pE1;
        SmPolyEdge *pOtherE = (SmPolyEdge*)GetOtherMate(pE); // m_pBToO
        if (!pOtherE) {
            pE = pE->GetRadial();
            pOtherE = (SmPolyEdge*)GetOtherMate(pE); // m_pBToO
        }
        if (pOtherE) {
            if (pOtherE != pE2 && pOtherE != pE2->GetRadial()) {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
                if (bDebugMe2) {
                    pE->Dump();
                    pE2->Dump();
                    pOtherE->Dump();

                    smgfx_Erase();
                    smgfx_ChangeLook(); pE->Draw();       sm_GraphicsLoop();
                    smgfx_ChangeLook(); pOtherE->Draw();  sm_GraphicsLoop();
                    smgfx_ChangeLook(); pE2->Draw();      sm_GraphicsLoop();
                    SmPolyFace *pFE2 = pE2->GetPolyFace();
                    smgfx_ChangeLook(); pFE2->Draw();     sm_GraphicsLoop();
                    SmPolyFace *pFOtherE = pOtherE->GetPolyFace();
                    smgfx_ChangeLook(); pFOtherE->Draw(); sm_GraphicsLoop();
                    sm_GraphicsLoop();
                }
#endif
                RemoveRelationship(pE,pOtherE);
                SE(SM_ERR);
            }
        }
        pOtherE = pE2;
        pE = (SmPolyEdge*)GetBrepMate(pOtherE); // m_pOToB
        if (!pE) {
            pOtherE = pOtherE->GetRadial();
            pE = (SmPolyEdge*)GetBrepMate(pOtherE); // m_pOToB
        }
        if (pE) {
            if (pE != pE1 && pE != pE1->GetRadial()) {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
                if (bDebugMe3) {
                    pE->Dump();
                    pOtherE->Dump();
                    pE1->Dump();

                    smgfx_Erase();
                    smgfx_ChangeLook(); pE->Draw();       sm_GraphicsLoop();
                    smgfx_ChangeLook(); pOtherE->Draw();  sm_GraphicsLoop();
                    smgfx_ChangeLook(); pE1->Draw();      sm_GraphicsLoop();
                    SmPolyFace *pFE1 = pE1->GetPolyFace();
                    smgfx_ChangeLook(); pFE1->Draw();     sm_GraphicsLoop();
                    SmPolyFace *pFOtherE = pOtherE->GetPolyFace();
                    smgfx_ChangeLook(); pFOtherE->Draw(); sm_GraphicsLoop();
                    sm_GraphicsLoop();
                }
#endif

                RemoveRelationship(pE,pOtherE);
                SE(SM_ERR);
            }
        }
        if (pE1->GetTolerance() < pE2->GetTolerance()) { pE1->SetTolerance(pE2->GetTolerance()); }
        if (pE2->GetTolerance() < pE1->GetTolerance()) { pE2->SetTolerance(pE1->GetTolerance()); }
    }

    m_pBToO->SetAt(pBrepTopo,pOtherTopo);
    m_pOToB->SetAt(pOtherTopo,pBrepTopo);

#ifdef SM_VALIDATE_INTERSECTIONS
    Validate();
#endif

    return SM_SUCCESS;
} // end of ifdef-ed out version of SmPolyIntersector::Relate

#else // not 0 branch
/*******************************************************************//**
PURPOSE: Relate these two topology objects.

NOTES: makes unique entries in
        m_pBToO->SetAt(pBrepTopo,pOtherTopo);
        m_pOToB->SetAt(pOtherTopo,pBrepTopo);
       and returns SM_SUCCESS 
       otherwise makes no entries and returns SM_ERR
***********************************************************************/
SmStatus SmPolyIntersector::Relate
 (SmObject * pBrepTopo,   // in : m_pBrep  obj1 of desired relation<obj1 obj2>
  SmObject * pOtherTopo)  // in : m_pOther obj2 of desired relation<obj1 obj2>
{
  NER( pBrepTopo );  // ... would cause crash.
  NER( pOtherTopo );

  // First grab both mates.
  SmObject *pOth = GetOtherMate(pBrepTopo); // m_pBToO
  SmObject *pBrp = GetBrepMate(pOtherTopo); // m_pOToB

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe0 = FALSE;
  if (bDebugMe0) 
    { TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("Relate: 0x%p  0x%p ; mates 0x%p  0x%p\n"), pBrepTopo, pOtherTopo, pOth, pBrp );
      smos_WriteBuffer(sBuff);

      if ( FALSE ) 
        { pBrepTopo->Dump();
          pOtherTopo->Dump();
        }
    }
#endif // SM_DEBUG_CODE

  // If the inputs are Faces, and either one is already
  // mated to a different object, return an error.
  if ( pBrepTopo->IsKindOf(SmPolyFace_TYPE)  && pOth != NULL && pOth != pOtherTopo )
    { return SM_ERR; }

  if ( pOtherTopo->IsKindOf(SmPolyFace_TYPE) && pBrp != NULL && pBrp != pBrepTopo  )
    { return SM_ERR; }

  // For vertices: check mating conflicts,
  // and also set both tolerances to the larger.
  if (pBrepTopo->IsKindOf(SmPolyVertex_TYPE))
    {
      SmPolyVertex * pV1 = SM_CAST_PTR(SmPolyVertex,pBrepTopo); NER(pV1);
      SmPolyVertex * pV2 = SM_CAST_PTR(SmPolyVertex,pOtherTopo); NER(pV2);

      // Check mating conflicts: already mated to a different vertex.
      // In that case, signal error and return ok.
      // (If this ever happens, investigate and possibly change this behavior.)
      SmPolyVertex *pOtherV = (SmPolyVertex*)GetOtherMate(pV1); // m_pBToO
      if (pOtherV) 
        {
          if (pOtherV != pV2)
            { SER(SM_ERR); }
          return SM_SUCCESS;
        }
      SmPolyVertex *pBrepV = (SmPolyVertex*)GetBrepMate(pV2); // m_pOToB
      if (pBrepV) 
        {
          if (pBrepV != pV1)
            { SER(SM_ERR); }
          return SM_SUCCESS;
        }

      if (pV1->GetTolerance() < pV2->GetTolerance()) { pV1->SetTolerance(pV2->GetTolerance());}
      if (pV2->GetTolerance() < pV1->GetTolerance()) { pV2->SetTolerance(pV1->GetTolerance());}
    }

  // For edges, same as vertices: check mating conflicts,
  // and also set both tolerances to the larger.
  // But with PolyEdges, the relationship can be recorded with
  // either the edge itself or any of its radial partners.
  if ( pBrepTopo->IsKindOf(SmPolyEdge_TYPE) )
    {
      SmPolyEdge *pE1 = SM_CAST_PTR(SmPolyEdge,pBrepTopo ); NER(pE1);
      SmPolyEdge *pE2 = SM_CAST_PTR(SmPolyEdge,pOtherTopo); NER(pE2);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
      if (bDebugMe1) 
        {
          smgfx_SetLook(2,4, 0,0,1); pE1->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,6, 0,1,0); pE2->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Check whether pE1 (or any of its radials) has a mate.
      SmPolyEdge *pOtherE = (SmPolyEdge*)GetOtherMate(pE1); // m_pBToO

      SmPolyEdge *pRad = pE1->GetRadial();
      // if ( pOtherE == NULL )
      while ( pOtherE == NULL && pRad != NULL && pRad != pE1 )
        {
          pOtherE = (SmPolyEdge*)GetOtherMate( pRad ); // m_pBToO
          pRad = pRad->GetRadial();
        }


      if ( pOtherE != NULL )
        {
          // Already mated.
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2) 
            {
              pE1->Dump();
              pE2->Dump();
              pOtherE->Dump();

              SmPolyFace *pFE2 = pE2->GetPolyFace();
              SmPolyFace *pFOtherE = pOtherE->GetPolyFace();

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pE1->Draw();      sm_GraphicsLoop();
              smgfx_SetLook(2,4, 0,1,0); pE2->Draw();      sm_GraphicsLoop();
              smgfx_SetLook(3,5, 1,0,0); pOtherE->Draw();  sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pFE2->Draw();     sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pFOtherE->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE
          if (pOtherE != pE2 && ! pOtherE->IsRadialPartnerOf( pE2 ) ) 
            {
              // Mate was not pE2 (or its radial): error.
              // Note, this can happen on spine edges, without
              // resulting in any problems. [B158]
              SE( SM_ERR );
            }
          return SM_SUCCESS; // Mate was already pE2 (or its radial): done.
        }

      // Check whether pE2 (or any of its radials) has a mate.
      SmPolyEdge *pBrepE = (SmPolyEdge*)GetBrepMate(pE2); // m_pOToB

      pRad = pE2->GetRadial();
      // if ( pBrepE == NULL )
      while ( pBrepE == NULL && pRad != NULL && pRad != pE2 )
        {
          pBrepE = (SmPolyEdge*)GetBrepMate( pRad ); // m_pOToB
          pRad = pRad->GetRadial();
        }

      if ( pBrepE != NULL )
        {
          if (pBrepE != pE1 && ! pBrepE->IsRadialPartnerOf( pE1 ))
            {
              // Mate was not pE1 (or its radial): error.
              // Note, this can happen on spine edges, without
              // resulting in any problems. [B158]
              SE( SM_ERR );
            }
          return SM_SUCCESS; // Mate was already pE2 (or its radial): done.
        }

      // Ok, neither had a mate.

      if (pE1->GetTolerance() < pE2->GetTolerance()) { pE1->SetTolerance(pE2->GetTolerance()); }
      if (pE2->GetTolerance() < pE1->GetTolerance()) { pE2->SetTolerance(pE1->GetTolerance()); }

    } // end case SmPolyEdge

  // All ok, relate them.
  m_pBToO->Insert(pBrepTopo,pOtherTopo);
  m_pOToB->Insert(pOtherTopo,pBrepTopo);

#ifdef SM_VALIDATE_INTERSECTIONS
  Validate();
#endif

  return SM_SUCCESS;

} // end SmPolyIntersector::Relate

#endif // not 0 branch


/*******************************************************************//**
PURPOSE: Validate relationships.

NOTES: 
***********************************************************************/
void SmPolyIntersector::Validate()
{
    SmTArray<SmObject*> sObjs1, sObjs2;
    m_pBToO->GetAllKeyValuePairs(sObjs1,sObjs2);

    ULONG ii;
    for (ii=0; ii<sObjs1.GetSize(); ii++) {
        SmObject * pObj1 = sObjs1[ii];
        SmObject * pObj2 = sObjs2[ii];
        if (!pObj1 || !pObj2) continue;
        if (pObj1->GetType() != pObj2->GetType()) {
            SE(SM_ERR);
        }
        if (pObj1->IsKindOf(SmPolyVertex_TYPE)) {
            SmPolyVertex *pV1 = SM_CAST_PTR(SmPolyVertex,pObj1);
            SmPolyVertex *pV2 = SM_CAST_PTR(SmPolyVertex,pObj2);
            double dDist = pV1->GetPoint().DistanceBetween(pV2->GetPoint());
            if (dDist > pV1->GetTolerance() + pV2->GetTolerance()) {
                SE(SM_ERR);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
                if (bDebugMe) {
                    pV1->Dump();
                    pV2->Dump();

                    smgfx_Erase(); smgfx_SetLook(1,2, 1,0,0); pV1->GetPoint().Draw();
                    sm_GraphicsLoop(); smgfx_SetLook(1,2, 0,0,1); pV2->GetPoint().Draw();
                    sm_GraphicsLoop(); smgfx_SetLook(1,2, 0,0,0); m_pBrep->Draw();
                    sm_GraphicsLoop(); smgfx_SetLook(1,2, 0,1,1); m_pOther->Draw();
                    sm_GraphicsLoop();
                }
#endif

            }
        }
        else if (pObj1->IsKindOf(SmPolyEdge_TYPE)) {
            SmPolyEdge *pE1 = SM_CAST_PTR(SmPolyEdge,pObj1);
            SmPolyEdge *pE2 = SM_CAST_PTR(SmPolyEdge,pObj2);
            SmPoint3d sMidPnt, sMidPntOther;
            pE1->EvaluatePoint(0.5,sMidPnt);
            pE2->EvaluatePoint(0.5,sMidPntOther);
            if (sMidPnt.DistanceBetween(sMidPntOther) > pE1->GetTolerance() +
                pE2->GetTolerance()) {
                SE(SM_ERR);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
                if (bDebugMe2) {
                    pE1->Dump();
                    pE2->Dump();

                    smgfx_Erase();
                    smgfx_SetLook(1,2, 1,0,0); pE1->Draw(); sm_GraphicsLoop();
                    smgfx_SetLook(1,2, 0,0,1); pE2->Draw(); sm_GraphicsLoop();
                    smgfx_SetLook(1,2, 0,0,0); m_pBrep->Draw();  sm_GraphicsLoop();
                    smgfx_SetLook(1,2, 0,1,1); m_pOther->Draw(); sm_GraphicsLoop();
                    sm_GraphicsLoop();
                }
#endif
            }
        }
    }
} // end SmPolyIntersector::Validate()

/*******************************************************************//**
PURPOSE:  Dump SmPolyIntersector Summary

NOTES: Currently just does the entity maps.
***********************************************************************/
void SmPolyIntersector::Dump
  (SmBoolean bDumpMapObjects)  // NotUsed: in : TRUE = Dump every map object
                               //      FALSE= don't
 const
{
  SM_REF1(bDumpMapObjects) ;
  // Dump in format "type: [key] -> value"

  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  static int iDumpLevel = 1; // Pass in as argument?  I think this is more convenient.

  smos_sprintf(sBuff, _T("%s"),_T("\nSmPolyIntersector Dump") );
  smos_WriteBuffer(sBuff);

  // output mapped entity list
  smos_sprintf(sBuff,_T("\n # Entity Maps = %ld" ),m_pBToO->Count());
  smos_WriteBuffer(sBuff);

#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif

  SmTArray<SmObject*> sKeys, sVals;
  m_pBToO->GetAllKeyValuePairs( sKeys, sVals );

  for ( ULONG ii = 0; ii < sKeys.GetSize(); ii++ )
  {
      SmObject *pThisObject  = sKeys[ii];
      SmObject *pOtherObject = sVals[ii];
      SM_ASSERT(pThisObject != NULL && pOtherObject != NULL);

      long lType    = pThisObject->GetType() ;
      double dDist;

      // check for widely separated vertices
      if(   lType == SmPolyVertex_TYPE
         && m_pOther)
        { dDist = ((SmPolyVertex *)pThisObject)->GetPoint().DistanceBetween(((SmPolyVertex *)pThisObject)->GetPoint()) ;
        }
      else
        { dDist = 0.0 ;
        }

      // output the relationship
      smos_sprintf(sBuff,       _T("\n\t%s : [0x%p] = 0x%p"),
                   lType == SmRegion_TYPE ? _T(" Region ")
                 : lType == SmPolyFace_TYPE   ? _T(" PolyFace   ")
                 : lType == SmCPolyFace_TYPE  ? _T(" CPolyFace  ")
                 : lType == SmPolyEdge_TYPE   ? _T(" PolyEdge   ")
                 : lType == SmPolyVertex_TYPE ? _T(" PolyVertex ")
                 : _T(" Other"),
                 pThisObject, 
                 pOtherObject);
      smos_sprintf(sBuffForFile,  _T("\n\t%s : [%s] = %s"),
                   lType == SmRegion_TYPE ? _T(" Region ")
                 : lType == SmPolyFace_TYPE   ? _T(" PolyFace   ")
                 : lType == SmCPolyFace_TYPE  ? _T(" CPolyFace  ")
                 : lType == SmPolyEdge_TYPE   ? _T(" PolyEdge   ")
                 : lType == SmPolyVertex_TYPE ? _T(" PolyVertex ")
                 : _T(" Other"),
                 pThisObject  ? _T("notNULL") : _T("NULL"), 
                 pOtherObject ? _T("notNULL") : _T("NULL"));
      smos_WriteBuffer(sBuff, sBuffForFile);

      // add in data for widely spaced vertices
      if(dDist > SM_EFF_ZERO * 2.0)
        {
          smos_sprintf(sBuff,_T(" Vert/Vert Dist = %16.16lf"), dDist);
          smos_WriteBuffer(sBuff);
        }
      if ( iDumpLevel >= 10 )
        {
          SmPoint3d sS1, sS2, sE1, sE2;
          if ( lType == SmPolyVertex_TYPE )
            {
              SmPolyVertex *pThisVtx = (( SmPolyVertex* )pThisObject );
              sS1 = pThisVtx->GetPoint();
              smos_sprintf(sBuff,_T("\n      [%16.12lf, %16.12lf, %16.12lf] "), sS1.x,sS1.y,sS1.z);
              smos_WriteBuffer(sBuff, sBuffForFile);

              SmPolyVertex *pOtherVtx = (( SmPolyVertex* )pOtherObject );
              sS2 = pOtherVtx->GetPoint();
              smos_sprintf(sBuff,_T("\n      [%16.12lf, %16.12lf, %16.12lf] "), sS2.x,sS2.y,sS2.z); 
              smos_WriteBuffer(sBuff, sBuffForFile);
            }
          else if ( lType == SmPolyEdge_TYPE )
            {
	      SmPolyEdge *pThisPolyEdge = ( SmPolyEdge* )pThisObject;
              SmPolyVertex *pThisStartVtx = pThisPolyEdge->GetStartPolyVertex();
              sS1 = pThisStartVtx->GetPoint();

              smos_sprintf(sBuff,_T("\n      S1: 0x%p : [%16.12lf, %16.12lf, %16.12lf] "), pThisStartVtx, sS1.x,sS1.y,sS1.z);
              smos_WriteBuffer(sBuff, sBuffForFile);

              SmPolyVertex *pThisEndVtx = pThisPolyEdge->GetEndPolyVertex();
              sE1 = (( SmPolyEdge* )pThisObject )->GetEndPoint();
              smos_sprintf(sBuff,_T("\n      E1: 0x%p : [%16.12lf, %16.12lf, %16.12lf] "), pThisEndVtx, sE1.x,sE1.y,sE1.z);
              smos_WriteBuffer(sBuff, sBuffForFile);

              SmPolyVertex *pOtherStartVtx = (( SmPolyEdge* )pOtherObject )->GetStartPolyVertex();
              sS2 = (( SmPolyEdge* )pOtherObject )->GetStartPoint();
              smos_sprintf(sBuff,_T("\n      S2: 0x%p : [%16.12lf, %16.12lf, %16.12lf] "), pOtherStartVtx, sS2.x,sS2.y,sS2.z);
              smos_WriteBuffer(sBuff, sBuffForFile);

              SmPolyVertex *pOtherEndVtx = (( SmPolyEdge* )pOtherObject )->GetEndPolyVertex();
              sE2 = (( SmPolyEdge* )pOtherObject )->GetEndPoint();
              smos_sprintf(sBuff,_T("\n      E2: 0x%p : [%16.12lf, %16.12lf, %16.12lf] "), pOtherEndVtx, sE2.x,sE2.y,sE2.z);
              smos_WriteBuffer(sBuff, sBuffForFile);
            }
        }

    } // end for each pair

  // Output count again at end, so it's easy to see.
  if(m_pBToO->Count() > 0)
    {
      smos_sprintf(sBuff,_T("\n # Entity Maps = %ld" ),m_pBToO->Count());
      smos_WriteBuffer(sBuff);
    }

  // That's enough for now.
  smos_WriteBuffer(_T("\n")) ;

} // end SmPolyIntersector::Dump()

/*******************************************************************//**
PURPOSE: Draw related topology objects.

NOTES: 
***********************************************************************/
void SmPolyIntersector::Draw(
              SmVector3d *pOffset1,
              SmVector3d *pOffset2,
              SmVector3d *pColor1,
              SmVector3d *pColor2
    )
{
#ifdef SM_GFX_CODE
    // Locals

    SmTArray<SmObject*> sObjs1, sObjs2;
    m_pBToO->GetAllKeyValuePairs(sObjs1,sObjs2);

    ULONG ii, lNumObjs = sObjs1.GetSize();
    for (ii=0; ii<lNumObjs; ii++) {
        SmObject * pObj1 = sObjs1[ii];
        SmObject * pObj2 = sObjs2[ii];
        if (!pObj1 || !pObj2) { continue; }
        if (pObj1->GetType() != pObj2->GetType()) {
            SE(SM_ERR);
        }

        SmPoint3d  sPt1, sPt2;
        if (pObj1->IsKindOf(SmPolyVertex_TYPE)) {
            SmPolyVertex *pV1 = SM_CAST_PTR(SmPolyVertex,pObj1);
            sPt1 = pV1->GetPoint();
            if ( pOffset1 != NULL ) { sPt1 += *pOffset1; }
            if ( pColor1  != NULL ) { smgfx_SetColor( *pColor1 ); }
            sPt1.Draw();

            SmPolyVertex *pV2 = SM_CAST_PTR(SmPolyVertex,pObj2);
            sPt2 = pV2->GetPoint();
            if ( pOffset2 != NULL ) { sPt2 += *pOffset2; }
            if ( pColor2  != NULL ) { smgfx_SetColor( *pColor2 ); }
            sPt2.Draw();
        }
        else if (pObj1->IsKindOf(SmPolyEdge_TYPE)) {
            SmPolyEdge *pE1 = SM_CAST_PTR(SmPolyEdge,pObj1);
            SmPolyEdge *pE2 = SM_CAST_PTR(SmPolyEdge,pObj2);

            SmVector3d sVec1;
            pE1->GetLine( sPt1, sVec1 );
            if ( pOffset1 != NULL ) { sPt1 += *pOffset1; }
            if ( pColor1  != NULL ) { smgfx_SetColor( *pColor1 ); }
            sVec1.Draw( &sPt1 );

            SmVector3d sVec2;
            pE2->GetLine( sPt2, sVec2 );
            if ( pOffset2 != NULL ) { sPt2 += *pOffset2; }
            if ( pColor2  != NULL ) { smgfx_SetColor( *pColor2 ); }
            sVec2.Draw( &sPt2 );
        }
    }
#else
  SM_REF4(pOffset1, pOffset2, pColor1, pColor2);
#endif // SM_GFX_CODE

} // end Draw

/*******************************************************************//**
PURPOSE: Draw related topology objects.

NOTES:
   Similar to Draw() except it can draw immediately (call sm_GraphicsLoop()),
   and it uses its own draw colors and sizes.
***********************************************************************/
void SmPolyIntersector::DrawRelatedObjects()
{
#ifdef SM_GFX_CODE
    SmTArray<SmObject*> sObjs1, sObjs2;
    m_pBToO->GetAllKeyValuePairs(sObjs1,sObjs2);

    smgfx_SetLook(1,2, 0,0,0);
//    m_pBrep->Draw();
//    m_pOther->Draw();
    sm_GraphicsLoop();

static constexpr SmBoolean bDrawNow = FALSE;

    for (ULONG ii=0; ii< sObjs1.GetSize(); ii++) {
        SmObject * pObj1 = sObjs1[ii];
        SmObject * pObj2 = sObjs2[ii];
        if (!pObj1 || !pObj2) { continue; }
        if (pObj1->GetType() != pObj2->GetType()) {
            SE(SM_ERR);
        }
        if (pObj1->IsKindOf(SmPolyVertex_TYPE)) {
            SmPolyVertex *pV1 = SM_CAST_PTR(SmPolyVertex,pObj1);
            SmPolyVertex *pV2 = SM_CAST_PTR(SmPolyVertex,pObj2);
            smgfx_SetLook(1,2, 1,0,0); pV1->GetPoint().Draw();
            if ( bDrawNow ) { sm_GraphicsLoop(); }
            smgfx_SetLook(3,4, 0,0,1); pV2->GetPoint().Draw();
            if ( bDrawNow ) { sm_GraphicsLoop(); }
        }
        else if (pObj1->IsKindOf(SmPolyEdge_TYPE)) {
            SmPolyEdge *pE1 = SM_CAST_PTR(SmPolyEdge,pObj1);
            SmPolyEdge *pE2 = SM_CAST_PTR(SmPolyEdge,pObj2);
            smgfx_SetLook(1,2, 1,0,0); pE1->Draw();
            if ( bDrawNow ) { sm_GraphicsLoop(); }
            smgfx_SetLook(3,4, 0,0,1); pE2->Draw();
            if ( bDrawNow ) { sm_GraphicsLoop(); }
        }
    }
#endif // SM_GFX_CODE

} // end DrawRelatedObjects
