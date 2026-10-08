// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTopologySweeep.cpp
* PURPOSE   --- Implementation of class members
**********************************************************************/

#include "StdAfx.h"

#include <SmTopologySweep.h>

#include <SmSurface.h>
#include <SmCurve.h>
#include <SmTopologySweep.h>
#include <SmSweepGeometryCreation.h>
#include <SmBSplineCurve.h>
#include <SmHCR.h>
#include <SmStitch.h>
#include <SmTopologyTraverser.h>
#include <SmGraphicsExtern.h>
#include <SmAssertArray.h>
#include <SmAttribute.h>
#include <SmMerge.h>


/*******************************************************************//**
PURPOSE: Constructor for Sweep object. 
            Needs an SmSweepGeometryCreation object which is already constructed,
            and knows the geometry (translation, rot, etc) of the sweep.

NOTES: 
***********************************************************************/
SmTopologySweep::SmTopologySweep
  (SmSweepGeometryCreation & rGeometryCreationArg,
   SmBoolean                 bDoStitching,
   SmBoolean                 ,                         // bClosedSweepArg,
   SmBoolean                 bDoSelfIntersectionArg,
   SmBoolean                 bDoMergeArg,
   SmBoolean                 bDoTaggingArg,
   long                      lTagID)
 : m_pGeometryCreation(&rGeometryCreationArg),
   m_bDoMerge(bDoMergeArg),
   m_bDoStitching(bDoStitching),
   m_bDoTagging(bDoTaggingArg),
   m_lTagID(lTagID),
   m_nRepetitions(1),
   m_bDoSelfIntersectionCheck(bDoSelfIntersectionArg)
{
  // Not yet supported:
  SM_ASSERT( !bDoSelfIntersectionArg );

} // end SmTopologySweep::SmTopologySweep constructor

/*******************************************************************//**
PURPOSE: Set the repetition count

NOTES: 
***********************************************************************/
void SmTopologySweep::SetRepetitions(ULONG nRepetitionsArg) 
{
  if(nRepetitionsArg == 0) 
    {
       SE(SM_ERR);
    }
  m_nRepetitions = nRepetitionsArg;

} // end SmTopologySweep::SetRepetitions

/*******************************************************************//**
PURPOSE: Set the mapping

NOTES: 
***********************************************************************/
void SmTopologySweep::SetHigher
  (SmTopology *pFromArg, 
   SmTopology *pToArg)
{
    // If FromArg is a vertex that has already been swept high,
    // ToArg need to be stitched with the image in such a way,
    // that the image should be kept, ToArg should be killed.
    // In  all other cases just record the map.
    
    SmVertex* pFromVtx = SM_CAST_PTR(SmVertex, pFromArg);

    SmEdge* pVEImage = NULL;
    if ( pFromVtx )
    {
        SmEdge* pToE = SM_CAST_PTR( SmEdge, pToArg );
        SM_ASSERT( pToE != NULL );
        pVEImage = GetToHigher( pFromVtx );
    } 

    // Note: there's no check for Edge already mapped.
    // Will overwrite an Edge's map if called twice with the same Edge.
    if ( !pVEImage )
    {
        m_vMapToHigher.Insert( pFromArg, pToArg );
        m_vMapFromLower.Insert( pToArg, pFromArg );
    }

} // end SmTopologySweep::SetHigher

/*******************************************************************//**
PURPOSE: Set the mapping when not already mapped

NOTES: Checks to see if the (FromArg, ToArg) pair is already in the
       maps m_vMapToSame and m_vMapFromSame.  If not, adds the pair.
***********************************************************************/
void SmTopologySweep::SetSame
  (SmTopology *pFromArg, 
   SmTopology *pToArg)
{ 
  // locals
  SmVertex* pToVtx   = SM_CAST_PTR(SmVertex, pToArg) ;
  SmEdge*   pToE     = SM_CAST_PTR(SmEdge  , pToArg) ;
  SmFace*   pToF     = SM_CAST_PTR(SmFace  , pToArg) ;

  SmVertex* pFromVtx   = SM_CAST_PTR(SmVertex, pFromArg) ;
  SmEdge*   pFromE     = SM_CAST_PTR(SmEdge  , pFromArg) ;
  SmFace*   pFromF     = SM_CAST_PTR(SmFace  , pFromArg) ;

  // Check for already mapped.
  SmVertex * pVImage = NULL ;
  SmEdge   * pEImage = NULL ;
  SmFace   * pFImage = NULL ;
  if      (pToVtx) { pVImage = GetToSame( pFromVtx ) ; }
  else if (pToE )  { pEImage = GetToSame( pFromE ) ; }
  else             { SM_ASSERT(pToF != NULL);
                     pFImage = GetToSame( pFromF );
                     SM_ASSERT(!pFImage ); // Faces should not already be mapped.
                   }

  // when not already mapped
  if ( !pVImage && !pEImage && !pFImage ) 
    {
       // update the maps
       m_vMapToSame.Insert(pFromArg,pToArg);
       m_vMapFromSame.Insert(pToArg,pFromArg);
    }

} // end SmTopologySweep::SetSame

/*******************************************************************//**
PURPOSE: Select the topology to keep and update the topology sweep
    relationships.

NOTES: 
    Generally, keep the entities that are recorded in our entity maps.
***********************************************************************/
SmStatus SmSweepStitchCallback::SelectTopologyToKeep
  (SmTopology * pTopologyElement1,      // in :
   SmTopology * pTopologyElement2,      // in :
   double       dDistTol,               // NotUsed: in :
   double     & rdMaxDistFound,         // out:
   SmBoolean  & rbKeepElement1)         // out:
{
  SM_REF1(dDistTol) ; 
    // The idea is to keep the maps intact. Select the topology_to_keep
    // in such a way that the maps need not be changed.

    // Note, 'From' variables are in the original Brep,
    // and 'To' variables are newly created (swept) entities.

    rdMaxDistFound = 0; // Not sure whether this will make a difference...

    rbKeepElement1 = TRUE;
    SmTopology* pTE1 = pTopologyElement1;
    SmTopology* pTE2 = pTopologyElement2;

    SmTopology* pToSame1 = m_pTopologySweep->GetToSamePriv( pTE1 );
    SmTopology* pToSame2 = m_pTopologySweep->GetToSamePriv( pTE2 );
    // If and only if pToSame is not NULL, then the object is in the
    // argument list of the sweep. 

    SmTopology* pFromSame1 = m_pTopologySweep->GetFromSamePriv( pTE1 );
    SmTopology* pFromSame2 = m_pTopologySweep->GetFromSamePriv( pTE2 );

    // ---------------------------------------------------------------------
    // Check: stitching an orig-brep object with a swept object.
    // ---------------------------------------------------------------------
    //   
    // First, the case of both objects belonging to the original brep.
    // NB the following 'if' does not detect all these cases, only those,
    // when the objects to glue are in the argument list of the sweep.
    // The original brep may contain many more objects, but we do not
    // know about them.
    // 
    // A problem, if the Stitch wants to stitch two objects of the original brep.
    //
    // We have assumed in the header, that this must not be the case: 
    // the original brep must have no stitchable parts. 

    if (pToSame1 && pToSame2 )
      { SER(SM_ERR); }
   
    // Again, what we can test here
    // is if an object created by the sweep is attempted to be stitched with an 
    // argument of the sweep - the original brep may contain many more 
    // objects. If someone thinks we should extend the check for all 
    // objects of the original brep, this object should be changed to keep 
    // such a list.
    //
    // Normally this is what we want to be done with objects and their copies.
    // but: 
    // -hof- observes, that the uncontrolled stitch may cause the problem of
    // a swept object coinciding by chance with an object of the original 
    // brep, of which is not a copy created by the sweep, causing
    // unwanted stitch, see figure:
    // 
    //        V2--------------
    //        |
    //        |
    //        ---------------* V1
    //
    //  If we sweep vertices with the sweep vect being (V2-V1)
    //  
    // If in the above case we allow the stitch to take place, the maps get 
    // messed up.
    // 
    // The commented out line helps to detect some of such cases:
    // go ahead only if the object, which has been created by the sweep,
    // has no preimage (eg in edge-high, the base-edge need to be stitched with
    // its copy - the copy has no preimage). 

  if (pToSame2 && !pToSame1) 
    {
        // SmBoolean bGoAhead = !pFromSame1;
        // if (bGoAhead )  else try to stop the swew

        // A special case: if in rot-edge-high, the surf is created,
        // but a v/e remains in place, the following will be TRUE:
      if (pToSame2 == pTE1) 
        {
           SM_ASSERT( pFromSame1 == pTE2); // a bug in the code if this fails
           m_pTopologySweep->RemoveFromToSamePriv(pTE2);
           m_pTopologySweep->RemoveFromFromSamePriv(pTE1);
           m_pTopologySweep->SetSame(pTopologyElement2, pTopologyElement2);
        }

        rbKeepElement1 = FALSE;
    }

    // see comment above 
  if (pToSame1 && !pToSame2) 
    {
        // SmBoolean bGoAhead = !pFromSame2;
        // if (bGoAhead ) else try to stop the swew

        // A special case: if in rot-edge-high, the surf is created,
        // but a v/e remains in place, the following will be TRUE:
      if (pToSame1 == pTE2) 
        {
           SM_ASSERT( pFromSame2 == pTE1); // a bug in the code if this fails
           m_pTopologySweep->RemoveFromToSamePriv(pTE1);
           m_pTopologySweep->RemoveFromFromSamePriv(pTE2);
           m_pTopologySweep->SetSame(pTopologyElement1, pTopologyElement1);
        }

        rbKeepElement1 = TRUE;
    }

    if (pToSame1 || pToSame2 )
      { return SM_SUCCESS; } // We have processed it.


    // ---------------------------------------------------------------------
    // Check: stitching two swept objects.
    // ---------------------------------------------------------------------

    // It is impossible that they come from different originals, 
    // because then these latter should coincide, which contradicts 
    // our assumption about the original brep being OK.

    // If both have preimages:
    // An error if the preimages are the same (contradicts with the MapToSame).
    // As said above, they can not come from different originals.
  if( pFromSame1 && pFromSame2 ) 
    {
        SER(SM_ERR);
    }

    // Only one of them has a preimage: this is OK, keep 
    // the one which has (so that the maps are unchanged). 
    // As said, it is impossible that they come from different originals.
  if( !pFromSame1 && pFromSame2 ) 
    {
        rbKeepElement1 = FALSE; // TE2 has a preimage, keep it
    }

    // See comment above 
  if( pFromSame1 && !pFromSame2 ) 
    {
        rbKeepElement1 = TRUE;  // TE1 has a preimage, keep it
    }

    if (pFromSame1 || pFromSame2 )
      { return SM_SUCCESS; } // We have processed it.


    //
    // Now we repeat for the higher/lower maps.
    // Here we can be simpler, just check for existence.
    // [As far as I can tell.  bd, Nov 2007]
    //

    void *pFromLower1 = m_pTopologySweep->GetFromLowerPriv( pTE1 );
    void *pFromLower2 = m_pTopologySweep->GetFromLowerPriv( pTE2 );

    if ( pFromLower1 && pFromLower2 )
    {
        SER( SM_ERR );  // Both are swept entities, and are coincident.
    }
    else if (  pFromLower1 && !pFromLower2 )
    {
        rbKeepElement1 = TRUE;
    }
    else if ( !pFromLower1 &&  pFromLower2 )
    {
        rbKeepElement1 = FALSE;
    }
    else
    {
        // no 'higher' maps, check lower pre-images.

        void *pToHigher1 = m_pTopologySweep->GetToHigherPriv( pTE1 );
        void *pToHigher2 = m_pTopologySweep->GetToHigherPriv( pTE2 );

        if ( pToHigher1 && pToHigher2 )
        {
            SER( SM_ERR );  // Both were from orignal, and are coincident.
        }
        else if (  pToHigher1 && !pToHigher2 )
        {
            rbKeepElement1 = TRUE;
        }
        else if ( !pToHigher1 &&  pToHigher2 )
        {
            rbKeepElement1 = FALSE;
        }
    }

    return SM_SUCCESS;

} // end SmSweepStitchCallback::SelectTopologyToKeep

/*******************************************************************//**
PURPOSE: For repeated sweep, update the argument arrays so that they 
         contain the result of the previous sweep

NOTES:
***********************************************************************/
static SmStatus sm_UpdateArgumentArrays
 (SmTArray<SmFace*>   * pFacesToSweepHigherArg,
  SmTArray<SmEdge*>   * pEdgesToSweepHigherArg,
  SmTArray<SmVertex*> * pVerticesToSweepHigherArg,
  SmTArray<SmFace*>   * pFacesToSweepSameArg,
  SmTArray<SmEdge*>   * pEdgesToSweepSameArg,
  SmTArray<SmVertex*> * pVerticesToSweepSameArg,
  SmTopologySweep     * pTopologySweepArg)
{
  if (pFacesToSweepHigherArg) 
    {
      SmTArray<SmFace*>& rFacesToSweepHigher = *pFacesToSweepHigherArg;
      ULONG nFaceCount = rFacesToSweepHigher.GetSize();
      for (ULONG k=0; k<nFaceCount; k++) 
        {
        
          SmFace *pFace = rFacesToSweepHigher[k];
          SmFace *pImage = pTopologySweepArg->GetToSame(pFace);
          // NER(pImage); // must exist: unless the sweep produced nothing
                          // in which case nothing to map to high
          if (pImage) 
              rFacesToSweepHigher.SetAt(k,pImage);
          else
              rFacesToSweepHigher.SetAt(k,pFace); // include again so that
                                                  // the arg list is the same
        }
    }
  if (pEdgesToSweepHigherArg) 
    {
      SmTArray<SmEdge*>& rEdgesToSweepHigher = *pEdgesToSweepHigherArg;
      ULONG nEdgeCount = rEdgesToSweepHigher.GetSize();
      for (ULONG k=0; k<nEdgeCount; k++) 
        {
        
          SmEdge *pEdge = rEdgesToSweepHigher[k];
          SmEdge *pImage = pTopologySweepArg->GetToSame(pEdge);
          // NER(pImage); // must exist: unless the sweep produced nothing
          if (pImage) 
              rEdgesToSweepHigher.SetAt(k,pImage);
          else
              rEdgesToSweepHigher.SetAt(k,pEdge);
        }
    }
  if (pVerticesToSweepHigherArg) 
    {
      SmTArray<SmVertex*>& rVerticesToSweepHigher = 
          *pVerticesToSweepHigherArg;
      ULONG nVertexCount = rVerticesToSweepHigher.GetSize();
      for (ULONG k=0; k<nVertexCount; k++) 
        {
        
          SmVertex *pVertex = rVerticesToSweepHigher[k];
          SmVertex *pImage = pTopologySweepArg->GetToSame(pVertex);
          // NER(pImage); // must exist: unless the sweep produced nothing
          if (pImage) 
              rVerticesToSweepHigher.SetAt(k,pImage);
          else
              rVerticesToSweepHigher.SetAt(k,pVertex);
        }
    }
  if (pFacesToSweepSameArg) 
    {
      SmTArray<SmFace*>& rFacesToSweepSame = *pFacesToSweepSameArg;
      ULONG nFaceCount = rFacesToSweepSame.GetSize();
      for (ULONG k=0; k<nFaceCount; k++) 
        {
        
          SmFace *pFace = rFacesToSweepSame[k];
          SmFace *pImage = pTopologySweepArg->GetToSame(pFace);
          // NER(pImage); // must exist: unless the sweep produced nothing
          if (pImage) 
              rFacesToSweepSame.SetAt(k,pImage);
          else
              rFacesToSweepSame.SetAt(k,pFace);
        }
    }
  if (pEdgesToSweepSameArg) 
    {
      SmTArray<SmEdge*>& rEdgesToSweepSame = *pEdgesToSweepSameArg;
      ULONG nEdgeCount = rEdgesToSweepSame.GetSize();
      for (ULONG k=0; k<nEdgeCount; k++) 
        {
        
          SmEdge *pEdge = rEdgesToSweepSame[k];
          SmEdge *pImage = pTopologySweepArg->GetToSame(pEdge);
          // NER(pImage); // must exist: unless the sweep produced nothing
          if (pImage) 
              rEdgesToSweepSame.SetAt(k,pImage);
          else
              rEdgesToSweepSame.SetAt(k,pEdge);
        }
    }
  if (pVerticesToSweepSameArg) 
    {
      SmTArray<SmVertex*>& rVerticesToSweepSame = 
          *pVerticesToSweepSameArg;
      ULONG nVertexCount = rVerticesToSweepSame.GetSize();
      for (ULONG k=0; k<nVertexCount; k++) 
        {
        
          SmVertex *pVertex = rVerticesToSweepSame[k];
          SmVertex *pImage = pTopologySweepArg->GetToSame(pVertex);
          // NER(pImage); // must exist: unless the sweep produced nothing
          if (pImage) 
              rVerticesToSweepSame.SetAt(k,pImage);
          else
              rVerticesToSweepSame.SetAt(k,pVertex);
        }
    }
  return SM_SUCCESS;

} // end sm_UpdateArgumentArrays

/*******************************************************************//**
PURPOSE: Ensure input Faces (and their Edges) and input Edges have a SM_AI_TAG attribute

NOTES: 
  1. Ensure every face and its Edges has a tag.
  2. Ensure every edge has a tag.
     When adding a tag to an edge, copy the edge->Curve->Tag if it exists,
     else create a new ordinal tag.
  3. don't tag vertices.  
***********************************************************************/
SmStatus SmTopologySweep::TagProfile
  (SmTArray<SmFace*>   * pOptFacesHigher,    // in : Faces and their edges to tag
   SmTArray<SmEdge*>   * pOptEdgesHigher,    // in : Edges to tag
   SmTArray<SmVertex*> * pOptVerticesHigher, // NotUsed: in : pOptVerticesHigher
   SmTArray<SmFace*>   * pOptFacesSame,      // in : more Faces and their edges to tag
   SmTArray<SmEdge*>   * pOptEdgesSame,      // in : more Edges to tag
   SmTArray<SmVertex*> * pOptVerticesSame)   // NotUsed: in : pOptVerticesSame
{
  SM_REF2(pOptVerticesHigher, pOptVerticesSame) ; 
  // accumulate edge inputs
  SmTArray<SmEdge*> sEdges;
  if (pOptEdgesHigher) { sEdges.Append(*pOptEdgesHigher); }
  if (pOptEdgesSame)   { sEdges.Append(*pOptEdgesSame); }

  // accumulate face inputs
  SmTArray<SmFace*> sFaces;
  if (pOptFacesHigher) { sFaces.Append(*pOptFacesHigher); }
  if (pOptFacesSame)   { sFaces.Append(*pOptFacesSame); }
  
  // for every face - accumulate face->edges and 
  SmTArray<SmEdge*> sFaceEdges;
  for (ULONG i=0; i<sFaces.GetSize(); i++) 
    {
      // accumulate face->edges
      SmFace *pF = sFaces[i];
      pF->GetEdges(sFaceEdges);
      for (ULONG ii=0; ii<sFaceEdges.GetSize(); ii++) 
        {
          sEdges.AddUnique(sFaceEdges[ii]);
        }

      // ensure face has a tag attribute
      SmAttribute *pFTag = pF->FindAttribute(SM_AI_TAG);
      if (!pFTag) 
        {
          SmTagAttribute *pNewTag = new (*pF->GetContext()) SmTagAttribute(SM_AI_TAG, 
                                                                           this->m_lTagID, 
                                                                           SM_TI_START_OF_SWEEP-i-1);
          pF->AddAttribute(pNewTag);
        }
    } // end iter every face - getting edges and adding tag attributes

  // for every edge - make sure it has a tag attribute
  for (ULONG j=0; j<sEdges.GetSize(); j++) 
    {
      SmEdge      *pE    = sEdges[j];
      SmAttribute *pETag = pE->FindAttribute(SM_AI_TAG);

      // when edge has NO tag attribute
      if (!pETag) 
        {
          SmCurve *pCurve = pE->GetCurve();

          // when edge->curve has a tag, copy it, else make a new tag
          SmTagAttribute *pCTag = (SmTagAttribute*)pCurve->FindAttribute(SM_AI_TAG);
          SmTagAttribute *pNewTag =  (pCTag)
                                    ? new (*pE->GetContext()) SmTagAttribute(SM_AI_TAG, 
                                                                             this->m_lTagID,
                                                                             0,
                                                                             SM_TI_GENERATOR_CURVE,
                                                                             pCTag->m_lPrimaryID,
                                                                             pCTag->m_lPrimaryOrdinal)
                                    : new (*pE->GetContext()) SmTagAttribute(SM_AI_TAG, 
                                                                             this->m_lTagID, 
                                                                             j+1) ;
          pE->AddAttribute(pNewTag);

        } // end NO tag attribute check
    } // end iter every edge
  
  // all done
  return SM_SUCCESS;

} // end SmTopologySweep::TagProfile

#ifdef SM_DEBUG_CODE                         
  #define DBG_VALIDATE_POINTERS pSweepBrep->ValidatePointers() ; \
                                if(bDebugMe) { pSweepBrep->Dump() ; }
#else
  #define DBG_VALIDATE_POINTERS 
#endif 


/*******************************************************************//**
PURPOSE:     The method which does the sweep.
- Rotational: if axis pierces body, possible failure, probable undesired results.

NOTES: 
   Note: sweep topologies need to be specified in the input argument arrays
         only if they don't occur at a higher level.  
   For example: the sweep of a single face to a shell (a higher dimension)
                will automatically sweep adjacent edges and vertices.
                So include the face in pOptFacesToSweepHigherArg list but
                  don't add the face's bounding edges and vertices into
                  any of the other input arguments - they will be swept just
                  because they are attached to the face.

   If the dimensionality of the sweep for a given object, as 
   implied by a higher-level object, contradicts with that of a 
   lower-level object, the higher-one is assumed without warnings.

   SweepEdgesHigh: The orientation of the new face is determined
   from the edge and the sweep (the new loop goes from 
   edge_start_vtx->end_end_vtx->sweep_vect).

   Note: there are some issues yet to be resolved with closed sweeps
   where there is more than one face or a face with more than one loop.
 
  About whole-Brep sweeps:
   If any of the six optional arrays is specified, then the topology
   in the given arrays (Faces, Edges, Vertices) will be swept.
   But if all six arrays are NULL, then the whole Brep will be swept.
   That behavior is encapsulated in DoAdvSweep().

   For users without SMLib, the whole-Brep sweep simply sweeps all Faces,
   Edges, and Vertices.  This will give reasonable results if all of the
   topology is 'visible' from the direction of the start of the sweep.
   Any manifold Edges, between two visible Faces, will be swept into
   internal Faces.  Any topology that is 'hidden' in the sweep direction
   will be swept as well, which could result in clashing topology that
   is likely not the desired result.  In this case, consider selecting
   specific topology to sweep.

   increments unlocked mark value
***********************************************************************/
SmStatus SmTopologySweep::DoSweep
 (SmBrep             * pBrepToSweep,                  // in : target Brep to sweep, always required
  SmRegion           * pRegionToSweepInto,            // in : The SmBrep of the region can be the
                                                      //      BrepToSweep or another SmBrep.
                                                      //      If no region given, the result will
                                                      //      be merged into BrepToSweepArg.
  SmTArray<SmFace*>   * pOptFacesToSweepHigherArg,    // in : sweep these faces into solids,   NULL to ingore, default:[NULL]
  SmTArray<SmEdge*>   * pOptEdgesToSweepHigherArg,    // in : sweep these edges into faces,    NULL to ignore, default:[NULL]
  SmTArray<SmVertex*> * pOptVerticesToSweepHigherArg, // in : sweep these vertices into edges, NULL to ignore, default:[NULL]
  SmTArray<SmFace*>   * pOptFacesToSweepSameArg,      // in : copy these faces to their swept position,    NULL to ignore, default:[NULL]
  SmTArray<SmEdge*>   * pOptEdgesToSweepSameArg,      // in : copy these edges to their swept position,    NULL to ignore, default:[NULL]
  SmTArray<SmVertex*> * pOptVerticesToSweepSameArg)   // in : copy these vertices to their swept position, NULL to ignore, default:[NULL]
                                                      // note: when all 6 input array ptrs are NULL, 
                                                      //       the whole Brep is swept. Otherwise only sweep 
                                                      //       the specified objects and their boundaries.
{
  // check input - must have a Brep with topology to sweep
  NER( pBrepToSweep );

  // The declared contract allows a null region, meaning merge the result into
  // pBrepToSweep, but every use below dereferences it. Normalize here so direct
  // kernel callers get the documented behaviour instead of a crash.
  if( !pRegionToSweepInto )
    pRegionToSweepInto = pBrepToSweep->GetInfiniteRegion();
  NER( pRegionToSweepInto );

  // when no topology is targeted to be swept - remember to sweep the whole Brep
  SmBoolean bSweepWholeBrep = (   !pOptFacesToSweepHigherArg  
                               && !pOptEdgesToSweepHigherArg  
                               && !pOptVerticesToSweepHigherArg  
                               && !pOptFacesToSweepSameArg  
                               && !pOptEdgesToSweepSameArg  
                               && !pOptVerticesToSweepSameArg);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE ;
  if ( bDebugMe )  // draw input
    {
      SM_DUMP_AND_ASSERT_VALID(pBrepToSweep) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; pBrepToSweep->Draw(1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Locals
  // Keep the owner independently of its regions: stitching and merging can
  // delete regions while the Brep survives.
  SmBrep * const pBrepToSweepInto = pRegionToSweepInto->GetBrep();
  const SmBoolean bSweepIntoInfiniteRegion =
      pRegionToSweepInto == pBrepToSweepInto->GetInfiniteRegion();
  const SmContext * cpRegionContext = pBrepToSweepInto->GetContext();
  const SmContext * cpBrepContext   = pBrepToSweep->GetContext(); // The two contexts will generally be the same, but not necessarily:

  SmTemporaryChangeValue< SmBoolean > sDoingBoolReg( SM_CONST_CAST(SmContext*, cpRegionContext)->GetDoingBooleanRef(), TRUE );
  SmTemporaryChangeValue< SmBoolean > sDoingBoolBrp( SM_CONST_CAST(SmContext*, cpBrepContext  )->GetDoingBooleanRef(), TRUE );
  SmBrep          * pNewBrep        = NULL;

  // Locals only used in whole-Brep sweep branch
  SmBrep             * pWireBrep      = NULL; // temp Brep to hold all pBrepToSweep WireEdges and ShellVerts
  SmTArray< SmFace* >  sSceneFaces;
  SmBoolean            bSweepIsClosed ;
  SmAxis2Placement     sRefFrame ;                // forward and inverted coordinate system for HCR
  SmAxis2Placement     sRefFrameInv ;
  
  // When merging - build sweep geometry into new tmp Brep 
  if (m_bDoMerge) 
    {
      pNewBrep = new (*cpRegionContext) SmBrep();
      // gwc: removed next line - sets pNewBrep->Tol = *cpRegionContext::ZoneTol3d
      //    pNewBrep->SetTolerance(pRegionToSweepInto->GetBrep()->GetTolerance());
    }
  SmObjDelete sCleanBrep(pNewBrep);

#ifdef SM_DEBUG_CODE
  // Validate through the surviving owner, not a region that stitching may delete.
  SmBrep * const pSweepBrep = pNewBrep ? pNewBrep : pBrepToSweepInto;
#endif

  // for every sweep m_nRepetitions
  for(ULONG ii=0;ii<m_nRepetitions;ii++)
    {
      // For repeated sweeps 
      if(ii > 0)
        {
          // update argument arrays to contain previous sweep results
          SER(sm_UpdateArgumentArrays(pOptFacesToSweepHigherArg,
                                      pOptEdgesToSweepHigherArg,
                                      pOptVerticesToSweepHigherArg,
                                      pOptFacesToSweepSameArg,
                                      pOptEdgesToSweepSameArg,
                                      pOptVerticesToSweepSameArg,
                                      this));
        } // end repeated sweep check
      
      // Reacquire the infinite region after the previous repetition's topology
      // changes. Preserve an explicitly supplied finite region.
      if ( bSweepIntoInfiniteRegion )
        { pRegionToSweepInto = pBrepToSweepInto->GetInfiniteRegion(); }

      // locals
      SmRegion * pRegion = (pNewBrep) ? pNewBrep->GetInfiniteRegion() : pRegionToSweepInto ;
      
      // check state - Rotational sweeps - signal error if next incremental sweep >= 360 degrees
      //                putting the check here allows as many legal step sweeps to complete
      //                prior to finding the one that's too big and causes an error.
      SER( m_pGeometryCreation->IsSweepClosed(ii+1, bSweepIsClosed));
      
      // When sweeping the entire Brep
      if(bSweepWholeBrep) 
        {

          // Method:
          // 1: Find the silhouette of the Brep in the direction of the start of the sweep.
          // 2: Imprint those silhouette curves as Edges on the Brep.
          // 3: Sweep those silhouette Edges into Faces.
          // 4: Translate the back-facing Faces from the start of the sweep to the end.
          // 5: Stitch it back together.
          // 6: Sweep original WireEdges and ShellVerts in separate Brep
          // 7: Merge two Breps to make final result

          // begin scope - build forward and inverted coordinate systems for HCR
            { 
              // Get an approximate center point for the Brep
              SmExtent3d sBBox;
              pBrepToSweep->CalculateBoundingBox( sBBox );
              SmPoint3d sCentroid = sBBox.GetMid();

              // get StartCoordinate system
              SmAxis2Placement sStartCoordSys;
              SER( m_pGeometryCreation->GetStartCoordSystem( sCentroid, sStartCoordSys ));

              // invert CoordinateSystem for HCR.
              // Also have to set its origin to zero first. (Why?)
              sRefFrame.SetCanonical(SmPoint3d(0,0,0),               // in : coordinate origin
                                     sStartCoordSys.GetXAxisRef(),   // in : coordinate X Axis (should be perp to Y axis)
                                     sStartCoordSys.GetYAxisRef()) ; // in : coordinate Y Axis (should be perp to X axis)
              sRefFrame.Invert(sRefFrameInv) ;

#ifdef SM_DEBUG_CODE
              if ( bDebugMe ) 
                {
                  SmVector3d sStartNorm = sStartCoordSys.GetZAxis();
                  SmVector3d sRefNorm   = sRefFrame.GetZAxis();
                  SmVector3d sInvNorm   = sRefFrameInv.GetZAxis();

                  smgfx_Erase();
                  smgfx_SetLook(1,4, 0,0,1); if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(1,4, 0,0,1); sStartCoordSys.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,4, 0,1,0); sRefFrame.Draw();      sm_GraphicsLoop();
                  smgfx_SetLook(1,4, 1,0,1); sRefFrameInv.Draw();   sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

            } // end scope - build forward and inverted coordinate systems for HCR

          // begin scope - remove all interior faces from pBrepToSweep
            {
              SmTArray<SmFace*>    sFaces, sRegFaces, sDelFaces;
              SmTArray<SmFaceuse*> sFUs;
              SmTArray<SmShell*>   sInfRegShells;

              SmRegion           * pInf = pBrepToSweep->GetInfiniteRegion();

              // get InfiniteRegion shells
              pInf->GetShells(sInfRegShells);

              // begin scope for sMarkLock used to linearize gather face code
                {
                  SmNewMarkAndLock sMarkLock(cpBrepContext) ;            // increment and lock any unlocked mark
                  SmMarkType       eMarkType = sMarkLock.GetMarkType() ; // fetch the mark that was locked

                  // for every InfiniteRegion shell - place faces bordering the InfiniteRegion into sRegFaces
                  for(ULONG jj=0;jj<sInfRegShells.GetSize();jj++)
                    {
                      sInfRegShells[jj]->GetFaceuses( sFUs );

                      for ( ULONG kk=0; kk<sFUs.GetSize(); kk++ )
                        {
                          SmFace *pFace = sFUs[kk]->GetFace();

                          // skip processed faces
                          if(pFace->IsMarked(eMarkType))
                            { continue ; }
                          pFace->Mark(eMarkType) ; 

                          // accumulate unique list of Region bordering faces
                          sRegFaces.Add( pFace );
                        }
                    } // end iter jj, adding every InfiniteRegion bordering Face to sRegFaces

                  // Place all faces not in sRegFace into sDelFaces list.
                  pBrepToSweep->GetFaces(sFaces) ;
                  for ( ULONG jj=0; jj<sFaces.GetSize(); jj++ )
                    {
                      SmFace * pFace = sFaces[jj] ;

                      // skip processed faces
                      if(pFace->IsMarked(eMarkType))
                        { continue ; }

                      // add pFace to sDelFaces list
                      sDelFaces.Add( sFaces[jj] );
                    } // end iter jj, adding all faces not in sRegFaces to sDelFaces
                } // end scope for sMarkLock used to linearize gather face code

              // remove sDelFacest faces from pBrepToSweep
              SmTemporaryChangeValue<SmBoolean> sChange( pBrepToSweep->m_bEditingEnabled, TRUE );
              pBrepToSweep->RemoveFaces( sDelFaces );  // increments unlocked mark value

            } // end scope - remove all interior faces from pBrepToSweep

          // arrive here when - pBrepToSweep only contains faces to sweep - all other faces have been removed
          //                  - forward and inverted coordinate system for HCR are set
          //                     sRefFrame
          //                     sRefFrameInv

          // Step 1: Get the silhouette curves.
          //         For a rotation, error if the centroid is on the axis.
          //           Actually, it wouldn't work at all that well if the Brep overlapped the axis at all.

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,4, 0,0,1); if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,4, 1,0,1); sRefFrameInv.Draw();   sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // hidden curve object locals
          double    dHiddenCurveTolerance = 0.001;
          SmBoolean bShowSeams  = FALSE; // Do not want to sweep seam edges ...
          SmBoolean bShowSmooth = FALSE; //  ... nor smooth edges.

          // the HiddenCurve object
          SmHCR sHCR(*cpBrepContext,              // in : new object construction
                      pBrepToSweep,            // in : Original Brep - never modified
                      sRefFrameInv,               // in : eye direction runs parallel to this vector
                      dHiddenCurveTolerance,      // in : dist tol  that limits segment sizes when walking silhouette curves
                      SM_DEG2RAD(5.0), // in : angle tol that limits segment sizes when walking silhouette curves
                      bShowSeams,                 // in : TRUE=marks seams as visible edges, FALSE=lets seams be invisible                                  
                      bShowSmooth );              // in : TRUE=marks manifold edges that are G1 between their faces as visible, FALSE=lets them be invisible
          sHCR.SetHCRForSilhouettes( TRUE );

          // Calculate silhouettes, make sHCR.m_pSceneBrep with silhouette edges
          sHCR.ComputeGlobalVisibility(); // increments unlocked mark value

          // pSceneBrep locals
          SmBrep *pSceneBrep = sHCR.GetSceneBrep();
          pSceneBrep->GetFaces( sSceneFaces );

 #ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              SM_DUMP_AND_ASSERT_VALID(pSceneBrep) ;

              smgfx_Erase();
              smgfx_SetLook(1,4, 0,0,1); if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,4, 0,1,0); if(pSceneBrep) pSceneBrep->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Step 2: Imprint silhouette curves as Edges onto pBrepToSweep, and put WireEdges and ShellVertices to the side.
          //         The imprint is already done in the HCR->pSceneBrep, use that.
          //         Replace the main Brep with the HCR's Brep.
          //           2a. Copy pBrepToSweep WireEdges and ShellVertices into new pWireBrep.
          //           2b. Gut pBrepToSweep.
          //           2c. Copy sHCR.m_pSceneBrep into pBrepToSweep.
          //           2d. Transform pBrepToSweep back to real space. 
          // 
          //         Note: To prevent sweeping WireEdges and ShellVertices into the interior of the final swept solid
          //           1. sweep sihouette edges into faces and build a manifold Brep with front and back faces
          //           2. sweep WireEdges and ShellVertices in another Brep.
          //           3. Boolean Union the two Breps 

          // locals
          SmTArray< SmEdge*   > sWireEdges;
          SmTArray< SmVertex* > sShellVerts;
          pBrepToSweep->GetWireEdges    ( sWireEdges  );
          pBrepToSweep->GetShellVertices( sShellVerts );

          // when WireEdges and ShellVerts exist - copy them to pWireBrep
          if ( sWireEdges.GetSize() + sShellVerts.GetSize() > 0 )
            {
              // The wire Brep stays in model space: no HCR at all for that.
              pWireBrep = new (*cpRegionContext) SmBrep();

              // copy WireEdges and ShellVerts from pBrepToSweep to temp pWireBrep 
              SmTArray< SmEdge*   > sNewBrepEdges;
              SmTArray< SmVertex* > sNewBrepVerts;
              pBrepToSweep->CopyEdges(    sWireEdges,  pWireBrep, &sNewBrepEdges );
              pBrepToSweep->CopyVertices( sShellVerts, pWireBrep, &sNewBrepVerts );

            } // end WireEdges and ShellVerts existence check

          // prepare pBrepToSweep for topology graph changes
          SmTemporaryChangeValue<SmBoolean> sChange( pBrepToSweep->m_bEditingEnabled, TRUE );

          // 2a. Empty pBrepToSweep of all faces, edges, and vertices.
          //     Note: this deletes Regions, so refresh pRegionToSweepInto if pRegionToSweepInto is the InfiniteRegion.
          SmBoolean bRefreshRegion = (pRegionToSweepInto == pBrepToSweep->GetInfiniteRegion()) ;
          SER( pBrepToSweep->RemoveAllTopology() );
          if ( bRefreshRegion )
            {
              pRegionToSweepInto = pBrepToSweep->GetInfiniteRegion();

              // Without a temporary merge Brep, pRegion aliases pRegionToSweepInto and so
              // goes stale on the same deletion. It, not pRegionToSweepInto, is what the
              // DoSweep*Higher/Same calls below are handed, so refresh it here too.
              if ( !pNewBrep )
                { pRegion = pRegionToSweepInto; }
            }

          // 2b. Copy pSceneBrep->Faces -- with the silhouette topology -- to pBrepToSweep.
          //     Note: 1. use SmMerge obj to keep maps between pSceneBrep and pBrepToSweep Edges
          //           2. Allow pBrepToSweep to contain composites (since they are in pSceneBrep)
          SmMerge sMergeObj(*cpRegionContext, 
                             pBrepToSweep, 
                             pSceneBrep,      
                             pBrepToSweep->GetTolerance(), 
                             20.0*SM_PI/180.0) ;

// Remove Composites
// #ifndef SM_NO_COMPOSITES   //cbi_CEdge: 21
//           pBrepToSweep->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES

          pSceneBrep->CopyFaces(sSceneFaces, 
                                pBrepToSweep, 
                                NULL, 
                                TRUE, 
                                &sMergeObj );

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              SM_DUMP_AND_ASSERT_VALID(pBrepToSweep) ; // pBrepToSweep and pSceneBrep are temporarily colocated
              SM_DUMP_AND_ASSERT_VALID(pSceneBrep) ;

              smgfx_Erase();
              smgfx_SetLook(1,4, 0,0,1); if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,4, 0,1,0); if(pSceneBrep) pSceneBrep->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          //  2c. Transform pBrepToSweep back from the HCR's space.
          pBrepToSweep->Transform( sRefFrame );

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,4, 0,0,1); if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,4, 0,1,0); if(pSceneBrep) pSceneBrep->Draw(TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Step 3: Sweep silhouette Edges into Faces.

          // HCR's Sil edges, and their mappings to pBrepToSweep.
          SmTArray <SmEdge*>      sEdgesToSweep ;
          SmTArray< SmEdge* >     sSilEdges ;
          SmTopologyIntersector & rTI = sMergeObj.GetTopologyIntersector() ;
          sHCR.GetSilhouetteEdges(sSilEdges) ;

          // for every silhouette edge
          for(ULONG jj=0;jj<sSilEdges.GetSize();jj++)
            {
              SmEdge   * pThisSilEdge = sSilEdges[jj] ;
              SmObject * pObj         = rTI.GetThisMate(pThisSilEdge) ;
              SmEdge   * pNewEdge     = SM_CAST_PTR(SmEdge, pObj);

              // accumulate pBrepToSweep copied silhouette edges to sweep
              sEdgesToSweep.Add( pNewEdge );

#ifdef SM_DEBUG_CODE
              if ( bDebugMe ) 
                {
                  if(jj == 0)
                    { smgfx_Erase();
                      smgfx_SetLook(1,4, 0,0,1) ; if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,4, 0,1,0) ; if(pSceneBrep) pSceneBrep->Draw(TRUE); sm_GraphicsLoop();
                    }
                  smgfx_SetLook(3,6, 1,0,0) ; pThisSilEdge->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

            } // end iter every silhouette edge building sEdgesToSweep list

          // low work - no SweepEdges, WireEdges, or ShellVerts that need further processing
          if(   sEdgesToSweep.GetSize() == 0 
             && sWireEdges.   GetSize() == 0 
             && sShellVerts.  GetSize() == 0)
            { return SM_SUCCESS; }

          // when asked - tag wire edges
          if (m_bDoTagging) 
            {
              // Add tags to edges.
              // Edges copy edge->curve tags when they exist.

              SmTArray< SmEdge* > sAllEdges( sWireEdges );
              sAllEdges.Append( sWireEdges );

              // Ensure AllEdges->Edges have a SM_AI_TAG attribute (gwc: ShellVerts not getting tagged - is that needed?)
              SER(TagProfile( NULL, &sAllEdges, &sShellVerts, NULL, NULL, NULL ));
            }

          // Sweep Edges into new Faces placed in pRegion
          //     note: This puts the new faces into pRegionToSweepInto->GetBrep()
          //           (not into pBrepToSweepInto).
          DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                // if(bDebugMe) pRegion->GetBrep()->Dump() 
          SER( DoSweepEdgesHigher( pRegion, sEdgesToSweep ));

          DBG_VALIDATE_POINTERS

          // Step 4: Translate the back-facing Faces from the start of the sweep to the end.
          //         4a. Collect all back-facing faces.
          //         4b. CopyFaces() into new temp Brep.
          //         4c. Remove back-facing faces from main Brep.
          //         4d. Transform temp Brep to the end of the sweep.
          //         4e. Merge temp Brep back into main Brep.

          // for closed sweeps, only faces from the swept sihouette edges remain,
          //       the original faces are deleted from the pBrepToSweep,
          //       otherwise original faces end up in the swept result interior.
          if ( bSweepIsClosed )
            {
              SmTArray< SmFace* > sFaces;
              pBrepToSweep->GetFaces( sFaces );
              pBrepToSweep->RemoveFaces( sFaces ); // increments unlocked mark value
            }

          // else doing a open sweep branch
          // when EdgesToSweep exists - find and move the back-facing faces that they bound
          else if ( sEdgesToSweep.GetSize() > 0 )
            {
              // locals
              SmTArray< SmFace* > sBackFaces, sTheseBackFaces;

              // increment and lock an unlocked mark
              SmNewMarkAndLock sMarkLock( sEdgesToSweep[0]->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
              SmMarkType       eMarkType = sMarkLock.GetMarkType() ;

              // Mark all Silhouette edges - to bound connected face searches
              for(ULONG jj=0;jj<sEdgesToSweep.GetSize();jj++)
                { 
                  sEdgesToSweep[jj]->Mark(eMarkType) ; 
                }

              // while unprocessed starter faces can be found - seek an unprocessed Back-Facing face
              while(TRUE)
                {
                  SmFace * pStartFace = NULL, * pThisStartFace = NULL ;
                  for(ULONG jj=0;jj<sEdgesToSweep.GetSize();jj++)
                    { 
                      SmEdge *pEdge = sEdgesToSweep[jj];

                      // Find SilhouetteEdge->Back-Face by comparing the Edge->Edgeuse->Binormal against the viewing direction 
                      SmVector3d  sViewDir = sRefFrame.GetZAxis();
                      SmEdgeuse * pEU      = pEdge->GetPrimaryEdgeuse();
                      SmExtent1d  sEdgeIvl = pEdge->GetInterval();
                      SmVector3d  sPt, sBiNorm;
                      pEU->EvaluateBinormal( sEdgeIvl.GetMid(), FALSE, sPt, sBiNorm );

                      // The back-face binormal is in same direction as the view vector.  
                      // Otherwise, the back-face is the Edgeuse->radial partner.
                      pThisStartFace =  (sBiNorm.Dot( sViewDir ) >= 0.0) 
                                       ? pEU->GetFace()
                                       : pEU->GetRadial()->GetFace() ;

                      // use any unprocessed pStartFace
                      if(pThisStartFace->IsMarked(eMarkType) == FALSE)
                        { pStartFace = pThisStartFace ;
                          break ; 
                        }

                    } // end iter every SilhouetteEdge seeking a connected unprocessed BackFace

                  // quit when no new unprocessed back-faces can be found
                  if(pStartFace == NULL)
                    { break ; }

                  // Collect faces connected to pStartFace on the same side of the (marked:[eMarkType]) silhouette edges.
                  SmTopologyTraverser sTT;
                  sTT.CollectFaces( pStartFace,        // in : Seed face (gets marked:[eMarkType])
                                    sTheseBackFaces,   // out: List of connected faces (Get marked:[eMarkType])
                                    eMarkType) ;       // in : specify mark for target objects (not incremented)
                  sBackFaces.Append(sTheseBackFaces) ;
#ifdef SM_DEBUG_CODE
                  if ( bDebugMe ) 
                    {
                      ULONG di ;

                      smgfx_Erase();
                      smgfx_SetLook(1,4, 0,0,1) ; if(pBrepToSweep) pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,4, 0,1,0) ; if(pSceneBrep) pSceneBrep->Draw(TRUE); sm_GraphicsLoop();
                      smgfx_SetLook(1,4, 0,1,1) ; for(di=0;di<sBackFaces.GetSize();di++) { sBackFaces[di]->DrawUV( 6,6); sm_GraphicsLoop();
                                                                                           sm_GraphicsLoop();
                                                                                         }
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                } // end while Finding unprocessed back-faces

              // create new temp pBackBrep to contain the back-faces being moved
              SmBrep *pBackBrep = new (*cpBrepContext) SmBrep();
              // gwc: removed next line - sets pBackBrep->Tol = *cpBrepContext::ZoneTol3d
              //    pBackBrep->SetTolerance( pBrepToSweep->GetTolerance() );
              SmObjDelete sCleanBackBrep(pBackBrep);

              // the Merge Object
              SmMerge sMergeObj2( *cpRegionContext, pBackBrep, pBrepToSweep,
                                   pBrepToSweep->GetTolerance(), 20.0*SM_PI/180.0 );

              // copy back-faces to pBackBrep
              pBackBrep->m_bEditingEnabled = TRUE;
              pBrepToSweep->CopyFaces( sBackFaces, pBackBrep, NULL, TRUE, &sMergeObj2 );

              // But -- if a face is Lamina, then there's no front and back.
              // In that case, we make a copy of the face and transform the copy.
              // Here, that just means that we don't delete Lamina faces.
              // Collect non-lamina (manifold) Faces in another array.
              SmTArray< SmFace* > sFacesToRemove;
              for ( ULONG jj=0; jj<sBackFaces.GetSize(); jj++ )
                {
                  SmFace *pBackFace = sBackFaces[jj];
                  if ( ! pBackFace->HasLaminaEdge() )
                    { sFacesToRemove.Add( pBackFace ); }
                }
              pBrepToSweep->RemoveFaces( sFacesToRemove );  // increments unlocked mark value

              // Transform the back-facing faces to the end of the sweep.
              SmAxis2Placement sSweepXFM;
              m_pGeometryCreation->GetSweepTransform( sSweepXFM );
              pBackBrep->Transform( sSweepXFM );

              // Step 5: Merge it back together.
              // Merge the temp Brep back into the main one.
              pBrepToSweep->MergeBrep( *pBackBrep );

            } // end else, sweep not closed and there are faces branch

          // arrive here after - pBrepToSweep internal faces have been deleted
          //                   - pBrepToSweep back faces have been moved to their swept positions
          //                   - pBrepToSweep silhouette edges have been swept into Faces
          //                   - pWireBrep contains copies of all the pBrepToSweep original WireEdges and ShellVertices

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              pBrepToSweep->Dump( _T("Brep after Sweep:") );
              SM_ASSERT_VALID( pBrepToSweep );

              smgfx_Erase() ;
              smgfx_SetLook(1,4, 0,0,1) ; pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,4, 0,0,1) ; pBrepToSweep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end sweep entire Brep branch

      else // sweep geometry passed in as arguments
        {
          // Special care needed for the edges of faces-high, and not to
          // forget, the loop-vertices of same.

          // make sure every edge and face input topology object has a tag
          SER(TagProfile(pOptFacesToSweepHigherArg,
                         pOptEdgesToSweepHigherArg,
                         pOptVerticesToSweepHigherArg,
                         pOptFacesToSweepSameArg,
                         pOptEdgesToSweepSameArg,
                         pOptVerticesToSweepSameArg));

          SmTArray<SmEdge*>      sEdgesHigher;
          SmTArray<SmVertex*>    sVerticesHigher;
          SmTArray<SmEdge*>      sEdges2; // aux arrays marked '2'
          SmTArray<SmVertex*>    sVertices2;
          SmTArray<SmVertexuse*> sVertexuses2;
          
          // save Build edges to higher list
          if (pOptEdgesToSweepHigherArg) 
            {
              sEdgesHigher.Append(*pOptEdgesToSweepHigherArg);
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 

            }

          // Sweep edges of higher faces 
          if (pOptFacesToSweepHigherArg) 
            {
              // for every face
              for (ULONG i=0; i<pOptFacesToSweepHigherArg->GetSize(); i++) 
                {
                  SmFace *pF = (*pOptFacesToSweepHigherArg)[i];

                  // place face->edges into EdgesHigher array
                  pF->GetEdges(sEdges2);
                  for (ULONG j=0; j<sEdges2.GetSize(); j++) 
                    {
                      sEdgesHigher.AddUnique(sEdges2[j]);
                    }

                  // place face->vertices into VerticesHigher array
                  pF->GetVertices(sVertices2);
                  for (ULONG j1=0; j1<sVertices2.GetSize(); j1++) 
                    {
                      SmVertex* pV = sVertices2[j1];
                      pV->GetVertexuses(sVertexuses2);
                      for (ULONG j2=0; j2<sVertexuses2.GetSize();j2++) 
                        {
                          SmVertexuse* pVU = sVertexuses2[j2];
                          if (pVU->IsLoopVertexuse()) 
                            {
                              sVerticesHigher.AddUnique(pV);
                            }
                        }
                    } // end iter every vertex building VerticesHigher array
                } // end iter every face
            } // end OptFacesToSweepHigher check
          
          // sweep all higher vertices into edges
          if ( sVerticesHigher.GetSize() > 0 ) 
            {
              SER(DoSweepVerticesHigher(pRegion, sVerticesHigher));
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
            } // end vertices exist to sweep check

          // sweep all higher faces into solids
          if (pOptFacesToSweepHigherArg) 
            {
              if (!bSweepIsClosed) 
                {  // edges are swept high anyway,
                   // only stitching remains
                  SER(DoSweepFacesHigher(pRegion, *pOptFacesToSweepHigherArg));
                  DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                        // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
                } // end sweep is open branch
              else // Sweep is closed
                {
                  // Set mapping between faces and themselves
                  for(ULONG ix=0;ix<pOptFacesToSweepHigherArg->GetSize();ix++) 
                    {
                      SmFace* pF = (*pOptFacesToSweepHigherArg)[ix];
                      SetSame(pF, pF); 
                    }
                } // end sweep is closed branch
            } // end faces exist to sweep check
          
          // sweep faces to faces
          if (pOptFacesToSweepSameArg) 
            {
              SER(DoSweepFacesSame(pRegion,  *pOptFacesToSweepSameArg));
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
            }
          
          // sweep edges to edges
          if ( pOptEdgesToSweepSameArg ) 
            {
              SER(DoSweepEdgesSame(pRegion,   *pOptEdgesToSweepSameArg));
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
            } 
          
          // sweep vertices to edges
          if (pOptVerticesToSweepHigherArg) 
            {
              SER(DoSweepVerticesHigher(pRegion, *pOptVerticesToSweepHigherArg));
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
            }
          
          // sweep vertices to vertices
          if (pOptVerticesToSweepSameArg) 
            {
              SER(DoSweepVerticesSame(pRegion, *pOptVerticesToSweepSameArg));
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
            }

          // sweep edges to faces
          if ( sEdgesHigher.GetSize() > 0 ) 
            {
              SER(DoSweepEdgesHigher(pRegion, sEdgesHigher));
              DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                    // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 
            }
        } // end sweep geometry passed in as arguments branch

      // arrive here after - m_bDoMerge == TRUE  ? pNewBrep contains sweep 
      //                     m_bDoMerge == FALSE ? pBrepToSweep contains sweep
      // whole Brep branch - pBrepToSweep internal faces have been deleted
      //                   - pBrepToSweep back faces have been moved to their swept positions
      //                   - pBrepToSweep silhouette edges have been swept into Faces
      //                   - pWireBrep contains copies of all the pBrepToSweep original WireEdges and ShellVertices
      // input tgts branch -
      //                   -
      
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          pBrepToSweep->Dump();
          if(pNewBrep) pNewBrep->Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,4,0,0,0) ; pBrepToSweep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,4, 0, 0, 0 ) ; if(pNewBrep) {pNewBrep->DrawUV(TRUE); } sm_GraphicsLoop() ;
          smgfx_SetLook(1,4, 1,0,0) ; if(pWireBrep) {pWireBrep->Draw(TRUE); } sm_GraphicsLoop() ;  
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // when asked, DoMerge else check m_bDoStitching
      if (m_bDoMerge) 
        {
          SmBrep *pMergeBrep = pBrepToSweepInto;

          pMergeBrep->m_bEditingEnabled = TRUE;

// Remove Composites
// #ifndef SM_NO_COMPOSITES   //cbi_CEdge: 21
//           pMergeBrep->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES
          
          double dTol      = pMergeBrep->GetTolerance();
          double dAngleTol = 20.0*SM_PI/180.0;
          //sCleanBrep.Clear();
          SmMerge sMerge(*cpRegionContext,pMergeBrep,pNewBrep,dTol,dAngleTol);
          SmBrep *pResult;

          //cbi: PiecewiseMerge() does only Faces: wire Edges/shell Verts are ignored.
          // Note PiecewiseMerge does not destroy the other brep
          SER(sMerge.PiecewiseMerge(NULL,TRUE,TRUE,pResult));
          DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 

        } // end DoMerge branch

      // when asked to stitch - only if not asked to merge (once merged no need to stitch)
      else if (m_bDoStitching) 
        {
          ULONG              lStitchedEdges, lLaminaEdges;
          double             dMaxVertexGap, dMaxEdgeGap;
          SmSweepStitchCallback sCallBack(this);
          SmStitch              sStitch(sCallBack, pBrepToSweep->GetTolerance());

          sStitch.m_bMakingManifoldSolid = FALSE; // RCLxx must be FALSE for primitive box

          // do stitching
          SER(sStitch.DoStitching(pBrepToSweep, // in : target Brep
                            NULL,            // in : only glue coincident vertices on this list,   NULL = do all vertices
                            NULL,            // in : only glue coincident edge pairs on this list, NULL = do all edges
                            lStitchedEdges,      // out: number of edges stitched
                            lLaminaEdges,    // out: number of lamina edges remaining after stitch
                            dMaxVertexGap,   // out: max gap found between coincident vertices considered for gluing
                            dMaxEdgeGap));   // out: max gap found between coincident edges    considered for gluing
                                             //      NOTE: some coincident vertex and edge pairs do not get glued due to
                                             //            a. gaps exceeding tolerances
                                             //            b. geometries not listed within optional candidate lists
                                             //            c. a failure within the glue edge function
                                             //            d. not being lamina when m_bMakingManifoldSolid == TRUE
                                             // out: min gap found between vertices that did not get glued, NULL to ignore.
                                             // out: min gap found between edges that did not get glued, NULL to ignore.
          DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                                // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 

         // orient the surfaces
         SmBoolean bNormalsOutward       = TRUE;
         SmBoolean bMaybeNotClosedSolid  = FALSE; 
         SmBoolean bJustUnifyNormals     = TRUE;
         pBrepToSweep->OrientTrimmedSurfaces(bNormalsOutward,        // in : 
                                                bMaybeNotClosedSolid,   // out: 
                                                bJustUnifyNormals);     // in : 
                                                                        // note: increments unlocked mark value
         DBG_VALIDATE_POINTERS // pRegion->GetBrep()->ValidatePointers()
                               // if(bDebugMe) pRegion->GetBrep()->ValidatePointers() 

        } // end DoStitch branch

      // And don't forget the wire edges and shell vertices (from the whole-Brep case).
      if ( pWireBrep != NULL )
        {
          SmSweepGeometryCreation * pSweepGeomCopy = m_pGeometryCreation->Copy();
          SmTopologySweep           sWireSweeper( *pSweepGeomCopy,
                                                   m_bDoStitching, FALSE, m_bDoSelfIntersectionCheck,
                                                   m_bDoMerge, m_bDoTagging, m_lTagID );
                                    
          SmTArray< SmEdge*   > sWireEdges;
          SmTArray< SmVertex* > sShellVerts;
          pWireBrep->GetWireEdges    ( sWireEdges );
          pWireBrep->GetShellVertices( sShellVerts );

          SER( sWireSweeper.DoSweep( pWireBrep,                      // in : target Brep to sweep, always required
                                     pWireBrep->GetInfiniteRegion(), // in : The SmBrep of the region can be the
                                                                     //      BrepToSweep or another SmBrep.
                                                                     //      If no region given, the result will
                                                                     //      be merged into BrepToSweepArg.
                                     NULL,                           // in : sweep these faces into solids,   NULL to ingore, default:[NULL]
                                     &sWireEdges,                    // in : sweep these edges into faces,    NULL to ignore, default:[NULL]
                                     &sShellVerts,                   // in : sweep these vertices into edges, NULL to ignore, default:[NULL]
                                     NULL,                           // in : copy these faces to their swept position,    NULL to ignore, default:[NULL]
                                     NULL,                           // in : copy these edges to their swept position,    NULL to ignore, default:[NULL]
                                     NULL ));                        // in : copy these vertices to their swept position, NULL to ignore, default:[NULL]
                                                                     // note: increments unlocked mark value
          // Done with this, we have to delete it:
          delete( pSweepGeomCopy ); pSweepGeomCopy = NULL;

          // Now the Union.
          SmBrep *pResult = NULL;
          SmMerge sMergeObj( *cpRegionContext, pBrepToSweep, pWireBrep,
                              pBrepToSweep->GetTolerance(), SM_DEG2RAD(20.0) );
          SER( sMergeObj.ManifoldBoolean( SM_BO_UNION, pResult )); // Note, this call deletes pWireBrep.

          SM_ASSERT( pResult == pBrepToSweep );

        } // end if there were wire edges/shell vertices to sweep

    } // end iter ii, every sweep m_pRepetition

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SM_DUMP_AND_ASSERT_VALID(pBrepToSweep) ; 

      smgfx_Erase();
      smgfx_SetLook(1,4) ; pBrepToSweep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4) ; pBrepToSweep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

// all done
  return SM_SUCCESS;

} // end SmTopologySweep::DoSweep

#undef DBG_VALIDATE_POINTERS 

/*******************************************************************//**
PURPOSE: Advanced Sweeping method which sweeps the 'visible' envelope
  of a whole Brep.

NOTES:  
   The whole-Brep sweep is meant to create the envelope
   of the visible topology of the input Brep.  The result should be a manifold
   Brep, unless the input Brep contains wire edges or shell vertices, which
   will be swept into lamina faces and wire edges, respectively.
   Since the result is only the envelope, any interior topology will
   actually disappear.  (If you have a body with an internal void,
   and you want that void swept as well, you should separate that void
   into another Brep, sweep them separately, and subtract the void from
   the outer Brep.)  (That functionality could possibly be implemented
   here, as an enhancement; contact Support if the need arises.)
   Also, manifold Edges between Faces that are in the interior of the
   visible region (i.e., not silhouettes), even if 'visible', will not be
   swept into internal Faces.

   Sweeps of nonmanifold bodies, other than wire edges and shell vertices
   mentioned above, should result in manifold sweeps.  This is specifically
   the case for lamina faces, whose outer, lamina edges are all visible
   from the sweep direction.  Outer lamina edges that are occluded in the
   sweep direction could produce unexpected results.  Moreover, sweeps of
   nonmanifold bodies that have faces that are curvy and have silhouette
   edges that are occluded in the sweep direction, can also cause problems.

   Edges internal to faces that are visible in the sweep direction -- manifold
   edges -- will not be swept into internal faces.
   If sweeping lamina faces, they will be copied into
   similar edges at the far end of the sweep.  The same is true for loop
   vertices in a face: they will not be swept into internal wire edges.

   Limitation: Mappings: for non-trivial whole-Brep sweeps, the mappings of
   topology from before to after have not been completely implemented. 
   If this becomes an issue, please contact Support.

   Also, multiple-repetition sweeps are not supported for whole-Brep sweeps.
   The envelope would be the same as for a single big sweep.

   METHOD, for whole-Brep sweep:
    1: Find the silhouette of the Brep in the direction of the start of the sweep.
    2: Imprint those silhouette curves as Edges on the Brep.
    3: Sweep those silhouette Edges into Faces.
    4: Translate the back-facing Faces from the start of the sweep to the end.
    5: Stitch it back together.
***********************************************************************/

SmStatus SmTopologySweep::DoAdvSweep( SmBrep * pBrepToSweep, SmRegion * pRegionToSweepInto)
{
  return DoSweep( pBrepToSweep, pRegionToSweepInto, NULL, NULL, NULL, NULL, NULL, NULL );

} // end DoAdvSweep


/*******************************************************************//**
PURPOSE: Sweep vertices to a same dimension - i.e. make vertices by
   sweeping the vertices to the given sweep destination.

NOTES:  
***********************************************************************/
SmStatus SmTopologySweep::DoSweepVerticesSame
  (SmRegion * pRegionArg,
   const SmTArray<SmVertex*> & crVerticesToSweepSameArg)
{
    
    ULONG lVertexCount = crVerticesToSweepSameArg.GetSize();
    for (ULONG k=0; k<lVertexCount; k++) 
      {
        SmVertex *pVertex = crVerticesToSweepSameArg[k];
        
        if (!pVertex) continue;
        
        SmVertex* pVImage = GetToSame(pVertex);
        if (pVImage) continue;

        SmVertex *pNewVertex;
        SER(SweepVertexSame(pRegionArg,pVertex,pNewVertex));
        
        SetSame(pVertex,pNewVertex);
      }
    
    return SM_SUCCESS;

} // end SmTopologySweep::DoSweepVerticesSame

/*******************************************************************//**
PURPOSE: Sweep vertices of an array to a higher dimension - i.e. make 
            edges by sweeping the vertices along a path.

NOTES: create a new vertex and a new edge in input region for
            every input vertex
***********************************************************************/
SmStatus SmTopologySweep::DoSweepVerticesHigher
  (SmRegion                  * pRegionArg,                   // in : 
   const SmTArray<SmVertex*> & crVerticesToSweepHigherArg)   // in : 
{
  // locals
  SmBrep          *pBrepToSweep = pRegionArg->GetBrep();
  const SmContext *pContext     = pBrepToSweep->GetContext();

  // for every vertex
  ULONG k ;
  for(k=0; k<crVerticesToSweepHigherArg.GetSize(); k++) 
    {
      SmVertex *pVertex = crVerticesToSweepHigherArg[k];
      
      // skip missing vertices
      if (!pVertex) continue;

      // skip completed vertices
      SmEdge* pVImage = GetToHigher(pVertex);
      if (pVImage) continue;
      
      // pass the create curve by sweeping a vertex request along
      SmPoint3d  sNewPt;
      double     dNewEdgeTol;
      SmCurve  * pSweepCurve = NULL;
      SER( m_pGeometryCreation->VertexSweepHigher(*pContext,
                                                   pVertex,
                                                   pSweepCurve,  
                                                   dNewEdgeTol));
      // handle degenerate curve case
      if (!pSweepCurve ) 
        {
          // diminishing sweep:
          SetSame(pVertex,pVertex); 
          continue;
        }
      
      // get end-Point
      SmPoint3d sNewVertexLoc;
      SER( pSweepCurve->EvaluatePoint(pSweepCurve->GetNaturalInterval().GetMax(), sNewVertexLoc));
      
      // If merge now, then no new vertex is needed, use old
      SmBoolean bNewVtxNeeded = TRUE;
      
      if ( !bNewVtxNeeded )  
        { // if 'use old vtx, if you can': merge now
          // the vertex into which we glue the new edge
          // must have relatives in our region. If not, have to
          // create a new vertex, so that the new edge is OK.
          SmShell* pDummyShell;
          SmStatus sStat = pVertex->GetShellInRegion(pRegionArg, pDummyShell);
          if (sStat == SM_SUCCESS ) 
            bNewVtxNeeded = TRUE; // use the existing: merged!
        }
      
      // vertex shells are different: even if a new 
      // vertex needs to be created, it does not make much
      // sense if the original vertex is just a shell-vertex
      SmVertex* pOrigVertex = pVertex;
      if (bNewVtxNeeded ) 
        {
          SmShell  * pNewShell = NULL;
          SmVertex * pCopiedVtx = NULL;
          SER(pBrepToSweep->MakeShellVertex( pRegionArg,
                                             pVertex->GetPoint(),
                                             pNewShell,
                                             pCopiedVtx));

          pVertex = pCopiedVtx;
        }
      
      // make an edge and a vertex connected to input vertex
      SmEdge   * pNewEdge = NULL;
      SmVertex * pNewVertex = NULL;
      SER( pBrepToSweep->MakeWireEdgeVertex(pRegionArg,
                                            pVertex,        // start vtx
                                            pSweepCurve, 
                                            pSweepCurve->GetNaturalInterval(),
                                            SM_OT_SAME,     // curve orientation st->new
                                            sNewVertexLoc,  // end point
                                            pNewEdge,       // returned edge
                                            pNewVertex));   // returned vtx
      
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE pNewEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
      SM_NEWTOL_LINE pNewVertex->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE pNewEdge->SetTolerance  (dNewEdgeTol);
      SM_OLDTOL_LINE pNewVertex->SetTolerance(dNewEdgeTol, 1); // update_only_if_larger
#endif // SM_USE_OLDTOL
      
      // save construction relationships
      SetSame  (pOrigVertex,pNewVertex);
      SetHigher(pOrigVertex,pNewEdge);
    }

  // all done
  return SM_SUCCESS;

} // end SmTopologySweep::DoSweepVerticesHigher

/*******************************************************************//**
PURPOSE: Sweep a vertex to the same dimension and create a new vertex.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::SweepVertexSame
 (SmRegion  * pRegionArg,
  SmVertex  * pVertexArg, 
  SmVertex *& pSweptVertexArg)
{
  SmPoint3d sSweptPt; 
  double dSweptVtxTol = 0.0;
  SER( m_pGeometryCreation->VertexSweepSame(pVertexArg,
                                            sSweptPt,
                                            dSweptVtxTol));

  SmShell* pNewShell = NULL;
  SER(pRegionArg->GetBrep()->MakeShellVertex(pRegionArg,
                                             sSweptPt,
                                             pNewShell,
                                             pSweptVertexArg));

#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE pSweptVertexArg->ClearLocalZoneTol3d( ) ; // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE pSweptVertexArg->SetTolerance(dSweptVtxTol, 1); // update_only_if_larger
#endif // SM_USE_OLDTOL

  return SM_SUCCESS;

} // end SmTopologySweep::SweepVertexSame

/*******************************************************************//**
PURPOSE: This reports, if the edge has already been swept, but 
  creates a copy MakeWireEdge anyway. 
 It assumes that the vertices are already swept.
 The From/ToSame maps are updated to know about the new edge/vertices (if)

NOTES: Returns the edgeuse, which is oriented same as euarg.
***********************************************************************/
SmStatus SmTopologySweep::SweepEdgeSame
  (SmRegion   * pRegionArg,
   SmEdgeuse  * pEUArg,
   SmEdgeuse *& rpSweptEdgeuseArg,
   SmEdge    *& rpExistingSweptEdgeArg) //
                                        // Will be set only if there already 
                                        // exists a swept edge for this
                                        // original edge (pEUArg->GetEdge()) 
                                        // -- otherwise will be set to NULL
{
  NER(pRegionArg);
  SmBrep *pBrep = pRegionArg->GetBrep();
  const SmContext *pContext = pBrep->GetContext();

  // what we need to record here (edge-same, face-same/high): 
  //             v->sweptv if not exist yet into MapToSame 
  //        if edge not swept yet:
  //             e->swepte into MapToSame
    
  SmEdge* pE = pEUArg->GetEdge();
    
  rpExistingSweptEdgeArg = GetToSame(pE);
    
  SmCurve* pSweptCurve;
  double dSweptEdgeTol;
  SER( m_pGeometryCreation->EdgeSweepSame(*pContext,         // in : context for new object construction
                                           pE,               // in : target edge to sweep
                                           pSweptCurve,      // out: new curve at swept position
                                           dSweptEdgeTol));  // out: new edge tolerance
    
  // Using the primeu's vertices for creating the new edge 
  // oriented SM_OT_SAME
  SmVertex *pV1 = pEUArg->GetVertexuse()->GetVertex();
  SmVertex *pV2 = pE->GetOtherVertex(pV1);
    
  // Swap vertices if the edgeuse is oriented opposite to the
  // edge. Now the vertices will correspond to that of the edge
  if (pEUArg->GetOrientation() == SM_OT_OPPOSITE) 
    {
      SmVertex *pTmpV = pV1;
      pV1 = pV2;
      pV2 = pTmpV;
    }
    
  SmVertex *pIV1 = GetToSame(pV1);
  SmVertex *pIV2 = GetToSame(pV2);
    
  if (!pIV1 ) 
    {
      SER( SweepVertexSame(pRegionArg,pV1,pIV1));
        
      SetSame(pV1,pIV1);
    }
    
  SmBoolean bType1Edge = pE->IsClosed();
    
  if (bType1Edge ) 
    {
      pIV2 = pIV1;
    }
  else  if (!pIV2 )
    {
      SER( SweepVertexSame(pRegionArg,pV2,pIV2));
      SetSame(pV2,pIV2);
    }
    
    
  if (!(pIV1 && pIV2)) { SER(SM_ERR); } //SweepFacesSame: vertices not swept
    
  SmEdge* pSweptEdge = NULL;       
  SER(pRegionArg->GetBrep()->MakeWireEdge(pRegionArg,
                                          pIV1,
                                          pIV2,
                                          pSweptCurve,
                                          pSweptCurve->GetNaturalInterval(),
                                          SM_OT_SAME,
                                          pSweptEdge));
    
  SmEdgeuse* pSweptPEU = pSweptEdge->GetPrimaryEdgeuse();
    
  if (pEUArg->GetOrientation() == SM_OT_SAME )
      rpSweptEdgeuseArg = pSweptPEU;
  else
      rpSweptEdgeuseArg = pSweptPEU->GetMate();
    
  // pImage is same as rpExistingSweptEdgeArg
  // if(!rpExistingSweptEdgeArg) { 
      SetSame(pE,pSweptEdge);
  //}
    
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE pSweptEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE pSweptEdge->SetTolerance(dSweptEdgeTol,TRUE);
#endif // SM_USE_OLDTOL
        
  return SM_SUCCESS;

} // end SmTopologySweep::SweepEdgeSame

/*******************************************************************//**
PURPOSE: Sweep the edges to the same dimension.  Creates a copy of the
   edge at the final sweep position.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::DoSweepEdgesSame
  (SmRegion                * pRegionArg,
   const SmTArray<SmEdge*> & crEdgesToSweepSameArg)
{
  NER(pRegionArg);

  ULONG lEdgeCount = crEdgesToSweepSameArg.GetSize();
  for (ULONG k=0; k<lEdgeCount; k++) 
    {
        
      SmEdge *pEdge = crEdgesToSweepSameArg[k];
        
      // may have been cleared by CleanList
      if (!pEdge) continue;
        
      SmEdge * pEImage = GetToSame(pEdge);
      if (pEImage) continue;
        
      SmEdge* pDummyAlreadySweptEdge;
      SmEdgeuse* pNewEU;
      SER( SweepEdgeSame(pRegionArg,
          pEdge->GetPrimaryEdgeuse(),
          pNewEU,
          pDummyAlreadySweptEdge));
        
      if (pDummyAlreadySweptEdge) { SER(SM_ERR); } // Must be not produced
        
      // what we needed to have recorded above (sweep-edge-same): 
      // NB the edge has not yet been swept!
      //                    v->newv into MapToSame,  if not exist yet
      //                    newv->v into MapFromSame if not exist yet
      //                    e->newe into MapToSame, 
      //                    newe->e into MapFromSame 
    }
  return SM_SUCCESS;

} // end SmTopologySweep::DoSweepEdgesSame

/*******************************************************************//**
PURPOSE: Create an Edge connected to a face with appropriate
         Edge/Vertex connections.

NOTES: 1. Assumes the input curve is coincident with the face's surface.
       2. Connects the new edge to existing vertices when existing 
            face->vertices are coincident with the input curve ends.
          Otherwise adds new Loop Vertices coincident with the curve
            ends to which the new edge is connected.
***********************************************************************/
static SmStatus sm_CreateEdgeForBase
  (SmFace         * pFaceArg,        // in : target Face
   SmCurve        * pCurveArg,       // in : geometry of the new edge in face
   SmBSplineCurve * pUVCurveArg,     // in : associated UVTrim Geometry of new edge, NULL to ignore
   SmVertex       * pVtx1Arg,        // out: NewEdge Start Vertex: existing or newly added face->Vertex coincident with Curve Start
   SmVertex       * pVtx2Arg,        // out: NewEdge End Vertex:   existing or newly added face->Vertex coincident with Curve End
   SmEdge        *& pNewEdgeArg)     // out: The new edge connected to Face
{
  // locals
  ULONG ii ;
  SmBrep             * pBrep = pFaceArg->GetBrep();
  double               dTol  = pBrep->GetTolerance();
  SmExtent1d           sCurveIvl(pCurveArg->GetNaturalInterval());
  SmPoint3d            sStartPt, sEndPt;
  SmTArray<SmVertex*>  sVertices;
  SmVertex           * pCoincStartV = NULL;
  SmVertex           * pCoincEndV   = NULL;

  // get Curve end positions
  SER(pCurveArg->EvaluatePoint(sCurveIvl.GetMin(), sStartPt));
  SER(pCurveArg->EvaluatePoint(sCurveIvl.GetMax(), sEndPt));

  // Get Face->Vertices
  pFaceArg->GetVertices(sVertices);

  // Find the vertices of the face, which correspond to the curve 
  // points. 0,1 or 2 can be found (0 if startv/endv on axis, full rot sweep,
  // and the edge is not a line). The finds must be unique.
  
  // for every vertex  
  for (ii=0; ii<sVertices.GetSize(); ii++) 
    {
      SmVertex* pV = sVertices[ii];

      double dStartDist = (pV->GetPoint() - sStartPt).Length();
      double dEndDist   = (pV->GetPoint() - sEndPt).Length();

      if(   dStartDist < dTol 
         && dEndDist   < dTol) { SER(SM_ERR); }

      if (dStartDist < dTol ) 
        {
          if (pCoincStartV != NULL ) { SER(SM_ERR); } // double coincidence
          pCoincStartV = pV;
        }
      if (dEndDist < dTol) 
        { 
          if (pCoincEndV != NULL ) { SER(SM_ERR); } // double coincidence
          pCoincEndV = pV;
        }
    } // end iter every face vertex looking for coincidence with the curve ends
    
  // This is possible as explained above.
  // if (!pCoincStartV && !pCoincEndV) { SER(SM_ERR); } // no coincidences

  pVtx1Arg = NULL;
  pVtx2Arg = NULL;

  // when sStartPt is not coincident with an existing Face->Vertex
  if (!pCoincStartV ) 
    {
      // make a loop vertex at curve start
      SmLoop* pNewL = NULL;
      SER(pBrep->MakeVertexLoop(pFaceArg, sStartPt, pNewL, pCoincStartV));
      pVtx1Arg = pCoincStartV;
    }

  // when sEndPt is not coincident with an existing Face->Vertex
  if (!pCoincEndV ) 
    {
      // make a loop vertex at curve end
      SmLoop* pNewL = NULL;
      SER(pBrep->MakeVertexLoop(pFaceArg, sEndPt, pNewL, pCoincEndV));
      pVtx2Arg = pCoincEndV;
    }

  SmEdge* pNewE = NULL; SmLoop* pNewL = NULL; SmFace* pNewF = NULL;
  SER(pBrep->MakeEdgeInFace(pFaceArg,     // in : Face where new edge goes
                            pCoincStartV, // in : Edge starts at this vertex
                            pCoincEndV,   // in : Edge ends at this vertex. StartVertex == EndVertex for closed curves.
                            pCurveArg,    // in : 3D Geometry of edge
                            pUVCurveArg,  // in : opt 2D Geometry of edge in UV of face, NULL to ignore
                            sCurveIvl,    // in : 3DCurve interval to use for edge.
                            SM_OT_SAME,   // in : one of: SM_OT_SAME, SM_OT_OPPOSITE                                     
                            dTol,         // in : largest of (Brep Tol, Curve Tol (if intersected), Curve/Face max dist)
                            pNewE,        // out: Newly created manifold edge
                            pNewL,        // out: New loop - created when vertices lie on existing loop and loop is split
                            pNewF));      // out: New face - created when vertices lie on existing loop and face is split

  // set output                            
  pNewEdgeArg = pNewE;

  // all done
  return SM_SUCCESS;

} // end sm_CreateEdgeForBase

/*******************************************************************//**
PURPOSE:  This creates a new face for an edge-sweep-high (1 loop, 4 edges).
   Returns 1 face, 2 edges, and 2 vertices.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::SweepEdgeHigh
  (SmRegion  * pRegionArg,            // in : region to contain new topology objects
   SmEdge    * pEdgeArg,              // in : Edge to sweep into a face
   SmFace   *& rpNewFaceArg,          // out: the few face connected to following boundary topology
   SmEdge   *& rpCopiedEdgeArg,       // out: the copy of the original edge
   SmEdge   *& rpNewStartEdgeArg,     // out: the new edge emanating from the start vtx 
   SmEdge   *& rpNewEndEdgeArg,       // out: the new edge emanating from the end vtx 
   SmEdge   *& rpNewEdgeArg,          // out: copy of original edge swept to final position
   SmVertex *& rpSweptStartVtxArg,    // out: copy of original start vertex swept to final position
   SmVertex *& rpSweptEndVtxArg)      // out: copy of original end vertex swept to final position
{
  // init outputs
  rpNewFaceArg       = NULL;
  rpCopiedEdgeArg    = NULL;
  rpNewStartEdgeArg  = NULL;
  rpNewEndEdgeArg    = NULL;
  rpNewEdgeArg       = NULL;
  rpSweptStartVtxArg = NULL;
  rpSweptEndVtxArg   = NULL;
  
  // check input
  NER(pRegionArg);

  // input locals
  SmBrep          * pBrepToSweep = pRegionArg->GetBrep(); NER(pBrepToSweep);
  const SmContext * pContext     = pBrepToSweep->GetContext();
  SmVertex        * pStartV      = pEdgeArg->GetStartVertex();
  SmVertex        * pEndV        = pEdgeArg->GetOtherVertex(pStartV);
  
  SmSurface * pNewFaceSurf      = NULL;
  SmCurve   * pOrigEdgeCurve    = NULL;
  SmCurve   * pNewEdgeCurve     = NULL;
  SmCurve   * pNewStartVtxCurve = NULL;
  SmCurve   * pNewEndVtxCurve   = NULL;

  double dNewFaceTol     = SM_BIG_DOUBLE ; 
  double dOrigEdgeTol    = SM_BIG_DOUBLE ;
  double dNewEdgeTol     = SM_BIG_DOUBLE ;
  double dNewStartVtxTol = SM_BIG_DOUBLE ; 
  double dNewEndVtxTol   = SM_BIG_DOUBLE ;
  
  SmVector3d sSurfNvAtStVtx;
  
  SmTArray<SmCurve*>        s3DCurves;
  SmTArray<SmBSplineCurve*> sUVCurves;
  SmTArray<SmOrientType>    sOrients;

  // sweep the edge and its vertices to create a face surface and its trimming boundaries
  SER( m_pGeometryCreation->EdgeSweepHigher
         (*pContext,           // in : context for new object construction
           pEdgeArg,           // in : Edge to be swept to a higher dimension
           pNewFaceSurf,       // out: Surface created by the sweep of the edge
           dNewFaceTol,        // out: Tolerance of the face - derived from edge tolerance.
           pOrigEdgeCurve,     // out: Curve Copy in orig position trimmed to edge ivl.
           dOrigEdgeTol,       // out: Tolerance of the new start curve. value:[pEdgeArg->Tol]
           pNewEdgeCurve,      // out: Curve copy moved to end sweep position.
           dNewEdgeTol,        // out: Tolerance of this new edge. value:[pEdgeArg->Tol]
           pNewStartVtxCurve,  // out: start vertex sweep curve.
           dNewStartVtxTol,    // out: start vertex sweep curve tolerance.
           pNewEndVtxCurve,    // out: end vertex sweep curve. 
           dNewEndVtxTol,      // out: end vertex sweep curve tolerance.
           s3DCurves,          // out: ordered new 3D TrimCurves (ptrs to previously output curves) 
           sUVCurves,          // out: associated new UV TrimCurves when easy, not built when expensive.
           sOrients,           // out: associated TrimCurve Orients for a valid outer loop. SM_OT_SAME, SM_OT_OPPOSITE
           sSurfNvAtStVtx  )); // out: The surface normal of the surface at the start vertex.

  // exit condition: if pNewFaceSurf is NULL then this edge sweep does not create a new face
  //                  It is probably a curve on the axis of revolution or something like that.
  if (!pNewFaceSurf) 
    {
      // map orig geometry to itself and return (GWC: why add the self mappings? maybe handle a boundary case?)
      SetSame(pEdgeArg, pEdgeArg); 
      SetSame(pStartV, pStartV);
      SetSame(pEndV, pEndV);
      return SM_SUCCESS;
    }
  
  // Sweep Higher output locals
  ULONG n3DCurves = s3DCurves.GetSize();
  ULONG nUVCurves = sUVCurves.GetSize();

  // there must be at least one curve (that of the edge)
  if (n3DCurves == 0) { SER(SM_ERR); }

  // GWC: The upcoming SmBrep::MakeFaceWithCurves() call assumes the 
  //      set of curves within s3DCurves marks out a simple closed loop
  //      although it does allow for coincident edges which makes sense for some circumstances.
  //      It looks like the previous EdgeSweepHigher() case can return a set of boundary edges which
  //      does not make a simple closed loop boundary.
  //     
  //      It looks like the following block was a shot at handling these cases,
  //      based on an undocumented and forgotten side effect of the EdgeSweepHigher() method
  //      but has been commented out (probably due to a failure.)
  //   
  //      1. I believe that a linear sweep always produces simple closed loop boundary result.
  //      2. I know that a rotational sweep that causes the start curve to 
  //         be coincident with the end curve (for example rotating a line
  //         180 degrees about a point on that line) produces a complex closed loop boundary result.
  //      3. rotating a curve thru itself (ex: rotating a line more than 180 degrees about an interior point)
  //         will a complex closed loop boundary result ( a bow tie ).
  //      
  //                                                               +      <-- 
  //                                                          +         +     \2 
  /*      Ex Problem case: Sweep line 180 degrees          +               +   \    */
  //                       about its center point        +                   +  |   
  //             OrigCurve                              +        1            + |   
  //           ------------>                           +     ----------->      +    
  //        ----------+---------       =SWEEP=>        +-----------+-----------+    
  //                   \ Sweep Axis                    +     ----------->      +   
  //         Sweep yields 4 curves that                 +       3              + 
  //         don't make a simple closed loop             +                   +  |  
  //          1. Orig Curve               SM_OT_SAME       +               +    |  
  //          2. end vertex sweep curve   SM_OT_SAME          +         +      /   
  //          3. copied curve             SM_OT_OPPOSITE           +          /4   
  //          4. start vertex sweep curve SM_OT_SAME                      <--  
  //         curves 1 and 3 are coincident                                             
  //
  // GWC: What I think needs to happen is this method has to identify the cases where
  //      the EdgeSweepHigher() return boundaries don't form a simple closed loop.
  //      I don't see all the rules right now, but I can see some.  These include
  //      1. Any sweep rotation greater than 360.
  //      2. for planar curves being rotated within their own plane many possible problems:
  //          2a. any bounary curve intersecting another within the curve interiors
  //                including cases of coincidence (as shown above).
  //          2b. any curve that will rotate thru itself.  This wll be true
  //              for any curve which is tangent to the rotation circles at any interior point.
  //              (The rotation circles are all the circles that can be created 
  //                  by rotating points about the rotation axis.) 
  //               
  // GWC: I'm going to leave this block commented out.  This means that 
  //      all cases that will fail in       
  //      
  // original comment: First, figuring out if MakeFaceWithCurves can cope. 
  //   It can not with a circle, from which an edge protrudes, 
  //   or with a strut on rotational surface.


  SmCurve* pStrutC = NULL;
  SmBSplineCurve* pStrutUV = NULL;   

#if 0
  if (0 && n3DCurves <= 2) 
    {
      // checking the assumption that the first curve is the orig
      // if fails here, just find it in the array
      if( pOrigEdgeCurve != s3DCurves[0]) { SER(SM_ERR); }

      pStrutC = s3DCurves[0];
      s3DCurves.RemoveAt(0);
      if (nUVCurves > 0)  
        { // uvcurves may not have been created
          pStrutUV = sUVCurves[0];
          sUVCurves.RemoveAt(0);
        }
    }  // end obsolete recovery from unexpected curve ordering within s3DCurves array
#endif

  // MakeFaceWithCurves locals
  // MakeFaceWithCurves rejects a degenerate surface before taking ownership of
  // the surface and trim curves; free them here instead of leaking them.
  if (pNewFaceSurf->IsDegenerateCurve() || pNewFaceSurf->IsDegeneratePoint())
    {
      SmObjsDelete<SmCurve*>        sClean3D(&s3DCurves);
      SmObjsDelete<SmBSplineCurve*> sCleanUV(&sUVCurves);
      SmObjDelete                   sCleanSurf(pNewFaceSurf);
      SER(SM_ERR_INVALID_INPUT);
    }

  SmTArray<ULONG> sCurveLoops;
  SmTArray<SmBSplineCurve*> *pUVCurves = (nUVCurves > 0) ? &sUVCurves : NULL;
  SmTArray<SmPoint3d> sLoopPoints;
  SmOrientType        eSurfOrientation = SM_OT_SAME;
  SmRegion          * pNewRegion = NULL;
  SmShell           * pNewShell = NULL;
  SmFace            * pNewFace = NULL;
  
  // set sCurveLoops array - There is always one loop to be created. 
  // Its size is the number of curves in the s3DCurves array
  sCurveLoops.Add(n3DCurves);     
  
  // Build the SweptFace from the swept geometry                                
  //    The geometry creator, EdgeSweepHigher(), creates the surface if it can, and returns
  // the bounding curves of the face (without the degenerate ones).
  // The face created must be as many sided as the number of 3DCurves.
  SER(pBrepToSweep->MakeFaceWithCurves
       (pRegionArg,                          // in : region to contain new topology objects 
        sCurveLoops,                         // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop 
        &s3DCurves,                          // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts 
        pUVCurves,                           // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts 
        sOrients,                            // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE 
        sLoopPoints,                         // in : Point positions to build SmVertex VertexLoops 
        pNewFaceSurf,                        // in : new face->Surface 
        pNewFaceSurf->GetNaturalUVDomain(),  // in : domain of Surface used by face 
        eSurfOrientation,                    // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE 
        pNewRegion,                          // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids. 
        pNewShell,                           // out: New shell if any.  Trimmed surfaces always create a new shell. 
        pNewFace));                          // out: the new face 
                                              
  // when 
  if (pStrutC) // GWC: a value is always NULL - the block that sets pStrutC was obsoleted
    {
      SmVertex* pVtx1=NULL;
      SmVertex* pVtx2=NULL;
      SmEdge* pNewE = NULL;
      // Create New Edge in Face
      SER(sm_CreateEdgeForBase(pNewFace, // in : target Face
                               pStrutC,  // in : geometry of the new edge in face
                               pStrutUV, // in : associated UVTrim Geometry of new edge, NULL to ignore
                               pVtx1,    // out: NewEdge Start Vertex: existing or newly added face->Vertex coincident with Curve Start
                               pVtx2,    // out: NewEdge End Vertex:   existing or newly added face->Vertex coincident with Curve End
                               pNewE));  // out: The new edge connected to Face
      
      //SetSame(pEdgeArg, pNewE);
    }

  // record EdgeArg <==> NewFace mapping
  SetHigher(pEdgeArg, pNewFace) ;
  
  //
  if (pOrigEdgeCurve) 
    {
      // Case where created a plane and center line curve goes away
      SmEdge* pEdge = SM_CAST_PTR(SmEdge,pOrigEdgeCurve->GetOwner());
      NER(pEdge); // The face creator must have created this edge
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE pEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE pEdge->SetTolerance(dOrigEdgeTol);
#endif // SM_USE_OLDTOL
    }
  
  // ---------the sweep of the base edge
  // Check to see whether it has been deleted.  That will happen when it's
  // coincident with another input curve: if it's swept 360 degrees and
  // becomes a seam.  [B317]
  if ( pNewEdgeCurve )
    {
      ULONG lIndex;
      if ( ! s3DCurves.FindElement( pNewEdgeCurve, lIndex ) )
        { pNewEdgeCurve = NULL; }  // It is stale.
    }

  if (pNewEdgeCurve) 
    { // need not exist, can coincide with the edge
      SmEdge * pEdge = SM_CAST_PTR(SmEdge,pNewEdgeCurve->GetOwner());
      NER(pEdge);
      // The image of the base edge    
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE pEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE pEdge->SetTolerance(dNewEdgeTol);
#endif // SM_USE_OLDTOL
      SetSame(pEdgeArg,pEdge);
      rpNewEdgeArg = pEdge;
    }
  else //
    {
      SetSame(pEdgeArg,pEdgeArg);
    }
  
  // pNewStartVtxCurve is the v-high curve of the original vertex. 
  // It may not exist in rot sweep, if the ax crosses the vtx.
  // The corresponding edge is oriented arbitrarily.
  ULONG lFoundIndex;
  if (pNewStartVtxCurve && s3DCurves.FindElement(pNewStartVtxCurve,lFoundIndex)) 
    {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          pEdgeArg->GetCurve()->Dump();
          if (pOrigEdgeCurve) pOrigEdgeCurve->Dump();
          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0); pNewStartVtxCurve->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0); pBrepToSweep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      SmEdge * pEdge = SM_CAST_PTR(SmEdge,pNewStartVtxCurve->GetOwner());
      NER(pEdge); // the curve is not degenerate, an edge must exist for it
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE pEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE pEdge->SetTolerance(dNewStartVtxTol);
#endif // SM_USE_OLDTOL
     
      //
      SmVertex * pNewStartV = pEdge->GetStartVertex();
      SmVertex * pNewEndV   = pEdge->GetOtherVertex(pNewStartV);

      double dStSt = pNewStartV->GetPoint().DistanceBetween( pStartV->GetPoint() );
      double dStE  = pNewEndV  ->GetPoint().DistanceBetween( pStartV->GetPoint() );

      //
      if (pNewStartV == pNewEndV ) 
        {
          SetSame( pStartV, pNewStartV );
          rpSweptStartVtxArg = pNewStartV;
        }
      else if (dStSt < dStE) 
        {
          // double checking: otherwise the selection makes no sense
          SM_ASSERT( dStSt < pBrepToSweep->GetTolerance());
          SM_ASSERT( dStE  > pBrepToSweep->GetTolerance());

          SetSame( pStartV, pNewEndV );
          rpSweptStartVtxArg = pNewEndV;
        }
      else //
        {
          // double checking: otherwise the selection makes no sense
          SM_ASSERT( dStE  < pBrepToSweep->GetTolerance());
          SM_ASSERT( dStSt > pBrepToSweep->GetTolerance());

          SetSame( pStartV, pNewStartV );
          rpSweptStartVtxArg = pNewStartV;
        }

      SetHigher( pStartV, pEdge ); // the start vertex is swept-high
      rpNewStartEdgeArg = pEdge;
    }
  else
    { // There is no pNewStartVtxCurve in s3DCurves.
      SetSame( pStartV, pStartV );
    }
  
  // Same as above, now for the endpoint
  if (pNewEndVtxCurve && s3DCurves.FindElement(pNewEndVtxCurve,lFoundIndex)) 
    {
      // For existence, pls see pNewStartVtxCurve

      SmEdge *pEdge = SM_CAST_PTR(SmEdge,pNewEndVtxCurve->GetOwner());
      NER(pEdge);
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE pEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE pEdge->SetTolerance(dNewEndVtxTol);
#endif // SM_USE_OLDTOL

      SmVertex * pNewStartV = pEdge->GetStartVertex();
      SmVertex * pNewEndV = pEdge->GetOtherVertex(pNewStartV);

      double dStE = (pNewStartV->GetPoint()-pEndV->GetPoint()).Length();
      double dEE  = (pNewEndV->GetPoint()-pEndV->GetPoint()).Length();

      //
      if (pNewStartV == pNewEndV ) 
        {
          SetSame(pEndV, pNewStartV); // coincident vertices
        }
      else if (dStE < dEE) 
        {
          // double checking: otherwise the selection makes no sense
          SM_ASSERT( dStE < pBrepToSweep->GetTolerance());
          SM_ASSERT( dEE  > pBrepToSweep->GetTolerance());
          SetSame(pEndV, pNewEndV);
          rpSweptEndVtxArg = pNewEndV;
        }
      else //
        {
          // double checking: otherwise the selection makes no sense
          SM_ASSERT( dEE  < pBrepToSweep->GetTolerance());
          SM_ASSERT( dStE > pBrepToSweep->GetTolerance());
          SetSame(pEndV, pNewStartV);
          rpSweptEndVtxArg = pNewStartV;
        }
      SetHigher(pEndV,pEdge); // the end vertex is swept high
      rpNewEndEdgeArg = pEdge;
    }
  else //
    { // There is no pNewEndVtxCurve in s3DCurves.
      SetSame(pEndV, pEndV);
    }

  // set output
  rpNewFaceArg = pNewFace;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
  if (bDebugMe2) 
    {
      pBrepToSweep->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,1,0); pStartV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); pNewStartVtxCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); pEndV->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pNewEndVtxCurve->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,1); pEdgeArg->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pNewFace->Draw(); pNewFace->DrawUV(8,8); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pNewFace->GetSurface()->DrawUV(12,12); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmTopologySweep::SweepEdgeHigh

/*******************************************************************//**
PURPOSE: Sweep the edges to a higher dimension.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::DoSweepEdgesHigher
  (SmRegion                * pRegionArg,               // in : 
   const SmTArray<SmEdge*> & crEdgesToSweepHigherArg)  // in : 
{
  // locals
  ULONG kk, lNumSweep = crEdgesToSweepHigherArg.GetSize();

  // for every edge
  for (kk=0; kk<lNumSweep; kk++) 
    {
      SmEdge *pEdge = crEdgesToSweepHigherArg[kk];
      
      // may have been cleared by CleanList
      if (!pEdge) { continue; }
      
      // skip already done edges
      SmFace* pEImage = GetToHigher(pEdge);
      if (pEImage) { continue;  }
      
      // Sweep the edge - create face, bounding edges, and bounding vertices
      SmVertex  *pSweptStartVtx, *pSweptEndVtx;
      SmEdge    *pCopiedEdge,    *pNewStartEdge, *pNewEndEdge, *pNewEdge;
      SmFace    *pNewF;
      SER( SweepEdgeHigh(pRegionArg,       // in : region to contain new topology objects
                         pEdge,            // in : Edge to sweep into a face
                         pNewF,            // out: the few face connected to following boundary topology
                         pCopiedEdge,      // out: the copy of the original edge
                         pNewStartEdge,    // out: the new edge emanating from the start vtx 
                         pNewEndEdge,      // out: the new edge emanating from the end vtx 
                         pNewEdge,         // out: copy of original edge swept to final position
                         pSweptStartVtx,   // out: copy of original start vertex swept to final position
                         pSweptEndVtx));   // out: copy of original end vertex swept to final position
      
      // when tagging - tag the new face and the copied edge
      if (m_bDoTagging) 
        {
          SmTagAttribute *pETag = (SmTagAttribute*)pEdge->FindAttribute(SM_AI_TAG);
          if (pNewEdge) 
            {
              NER(pETag);
              SmTagAttribute *pNewETag = (SmTagAttribute*)new(*pEdge->GetContext()) SmTagAttribute(*pETag);
              pNewETag->m_lPrimaryOrdinal += SM_TI_SWEEP_STEP_DELTA; // Each sweep step this goes up by 100k
              pNewEdge->AddAttribute(pNewETag);
            }
          if (pNewF) 
            {
              SmTagAttribute *pNewFTag = (SmTagAttribute*)new(*pEdge->GetContext()) SmTagAttribute(*pETag);
              pNewFTag->m_lPrimaryOrdinal += SM_TI_SWEEP_STEP_DELTA; // Each sweep step this goes up by 100k
              pNewF->AddAttribute(pNewFTag);
            }
        }
      // diminishing sweep returns success!
   
    } // end iter every edge

  // all done
  return SM_SUCCESS;

} // end SmTopologySweep::DoSweepEdgesHigher

/*******************************************************************//**
PURPOSE: Sweep faces to the same dimension by copying them to the new
    location.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::DoSweepFacesSame
  (SmRegion* pRegionArg,
   const SmTArray<SmFace*> & crFacesToSweepSameArg)
{
    ULONG lFaceCount = crFacesToSweepSameArg.GetSize();
      for (ULONG k=0; k<lFaceCount; k++) {

       SmFace *pF = crFacesToSweepSameArg[k];

       if (!pF)  continue;

       SER( SweepFaceSame(pRegionArg, pF) );
      }
    return SM_SUCCESS;

} // end SmTopologySweep::DoSweepFacesSame
 
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static void sm_MapVertexLoops
  (SmFace* pOrigFaceArg, SmFace* pNewFaceArg,
   SmTopologySweep& rSweepArg)
{
    // -hof- : I have little clue as to how to map the vertex 
    // loopuses of the 2 faces. 
    // I assume that the vertex loop uses are created in the same order
    // as they occur in the original face. 
    // Even if this is not TRUE, the function works for the case of 
    // 1 VertexLoop.

    SmLoopuse *pData1[256];
    SmLoopuse *pData2[256];
    SmVertexuse *pData3[32];
    SmVertexuse *pData4[32];

    SmTArray<SmLoopuse*> sLoopuses(256,pData1);
    SmTArray<SmLoopuse*> sNewLoopuses(256,pData2);

    SmTArray<SmVertexuse*> sVertexuses(32,pData3);
    SmTArray<SmVertexuse*> sNewVertexuses(32,pData4);

    int nNewFirstVLUIndex = -1;
    // the index of the first VLU in the new face

    SmFaceuse* pPFU = pOrigFaceArg->GetUpwardFaceuse();
    SmFaceuse* pNewPFU = pNewFaceArg->GetUpwardFaceuse();
    pNewPFU->GetLoopuses(sNewLoopuses);
    for (ULONG k=0; k<sNewLoopuses.GetSize(); k++) {

        SmLoopuse *pNewLU = sNewLoopuses[k];
        if (pNewLU->IsVertexLoopuse()) {
            nNewFirstVLUIndex = k;
            break;
        }
    }

    int nVLUIndex = -1;
    pPFU->GetLoopuses(sLoopuses);
    for (ULONG j=0; j<sLoopuses.GetSize(); j++) {

        SmLoopuse *pLU = sLoopuses[j];
        if (pLU->IsVertexLoopuse()) {
            nVLUIndex++; // the count of VLU's in the original face

            SmLoopuse* pNewLU = sNewLoopuses[nNewFirstVLUIndex + nVLUIndex];

            pLU->GetVertexuses(sVertexuses); 
            pNewLU->GetVertexuses(sNewVertexuses); 

            SmVertex *pV    = sVertexuses[0]->GetVertex();
            SmVertex *pNewV = sNewVertexuses[0]->GetVertex();

            rSweepArg.SetSame(pV, pNewV);
        }
    }

} // end sm_MapVertexLoops

/*******************************************************************//**
PURPOSE: Sweep a face to the same dimension.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::SweepFaceSame
  (SmRegion * pRegionArg,    // in : Region to contain new topology objects
   SmFace   * pFaceArg)      // in : Face to be swept
{
  // init return
  if (!pFaceArg)
    return SM_SUCCESS;

  // check inputs
  NER(pRegionArg);

  // locals
  ULONG ii, jj ;
  SmBrep          * pBrepToSweep = pRegionArg->GetBrep();
  const SmContext * pContext     = pBrepToSweep->GetContext();
  SmFace          * pImage       = GetToSame(pFaceArg); 
  SmFaceuse       * pPFU         = pFaceArg->GetUpwardFaceuse();

  // no work - face already swept
  if (pImage) 
    { SER(SM_ERR); }  // Face already swept
  
  // locals
  ULONG         pData4[ 32]; SmTArray<ULONG>         sLoopEUCounts( 32,pData4); // one entry per EdgeLoop, value = edgecount
  SmPoint3d     apData[256]; SmTArray<SmPoint3d>     sLoopPoints  (256,apData); // one entry per VertexLoop, value = Vertex loc
                                                     
  SmEdge *      pData2[256]; SmTArray<SmEdge *>      sOrigEdges   (256,pData2); // ordered list of orig edges 
  SmOrientType  aeData[256]; SmTArray<SmOrientType>  sOrients     (256,aeData); // associated orig edge orientations
  SmCurve *     pData8[256]; SmTArray<SmCurve *>     s3DCurves    (256,pData8); // associated copied curves swept to final position
  double        adData[256]; SmTArray<double>        sTolerances  (256,adData); // associated copied curve tolerances
                                                     
  SmLoopuse *   pData3[256]; SmTArray<SmLoopuse *>   sLoopuses    (256,pData3); // iter list of loopuses in UpwardFaceuse
  SmEdgeuse *   pData1[256]; SmTArray<SmEdgeuse *>   sEdgeuses    (256,pData1); // iter list of edgeuses in one Edgeloopuse
  SmVertexuse * pData5[ 32]; SmTArray<SmVertexuse *> sVertexuses  ( 32,pData5); // iter list of vertexuses in one VertexLoopuse
                                                     
  // for every pFaceArg->UpperFace->Loopuse
  pPFU->GetLoopuses(sLoopuses);
  for (ii=0; ii<sLoopuses.GetSize(); ii++) 
    {
      SmLoopuse *pLU = sLoopuses[ii];

      // get Loopuse->Edgeuses
      pLU->GetEdgeuses(sEdgeuses); // resets the array for this LU!

      // accumlate EdgeLoop edge counts
      if (!pLU->IsVertexLoopuse()) 
        {
          sLoopEUCounts.Add(sEdgeuses.GetSize()); // a new loop: edge count
        }

      // edge-loops: for every Loopuse->Edgeuse  
      //               Copy Curve to new position and build sOrigEdges, s3DCurves, sTolerances, and sOrients arrays
      for (jj=0; jj<sEdgeuses.GetSize(); jj++) 
        {
          SmEdgeuse * pEU     = sEdgeuses[jj];
          SmEdge    * pOrigE  = pEU->GetEdge();
          
#if 0
          SmEdge    * pSweptE = GetToSame( pOrigE );
          if (0 && pSweptE) 
            {
              // the problem with this version is that in rot sweep,
              // if the edge goes opposite to the axis, there is no
              // way I know of to keep the swept curve oriented same
              // as the original. NB this branch of the 'if' is not
              // much faster then the other one, which would work
              // for all cases, so lets just switch it off.
              SmCurve *pCopiedCurve;
              SER(pSweptE->GetCurve()->Copy(*pContext,pCopiedCurve));
              NER(pCopiedCurve);

              sOrigEdges.Add(pOrigE);
              s3DCurves.Add(pCopiedCurve);
              sTolerances.Add(pSweptE->GetTolerance());
              sOrients.Add(pEU->GetOrientation());
 
            } // end obsolete branch
          else // now the only branch
#endif
            {
              double    dSweptEdgeTol;
              SmCurve * pSweptCurve;
              SER( m_pGeometryCreation->EdgeSweepSame(*pContext,         // in : context for new object construction
                                                       pOrigE,           // in : target edge to sweep
                                                       pSweptCurve,      // out: new curve at swept position
                                                       dSweptEdgeTol));  // out: new edge tolerance
              // set ordered associated arrays of origEdges to copied edges
              sOrigEdges.Add(pOrigE);
              s3DCurves.Add(pSweptCurve);
              sTolerances.Add(dSweptEdgeTol);
              sOrients.Add(pEU->GetOrientation());
            } // end setting OrigE, SweptCurve, SweptEdgeTo, Orientation arrays
        } // end iter every Loopuse->Edgeuse 

      // vertex-loops: Copy Vertex to new position and build sLoopPoints array
      if ( pLU->IsVertexLoopuse()) 
        {
          // get Loopuse->Vertexuses
          pLU->GetVertexuses(sVertexuses); 

          // locals
          SmVertex * pV      = sVertexuses[0]->GetVertex();
          SmVertex * pVSwept = GetToSame(pV);
          if (pVSwept) 
            {
              sLoopPoints.Add(pVSwept->GetPoint());
            } // end already swept vertex branch
          else // vertex needs to be swept
            {
              SmPoint3d sSweptPt; 
              double dSweptVtxTol;
              SER( m_pGeometryCreation->VertexSweepSame(pV,
                                                        sSweptPt,
                                                        dSweptVtxTol));
              sLoopPoints.Add(sSweptPt);
            } // end need to sweep vertex branch
        } // end IsVertexLoopuse (need to build sLoopPoints array) check
    } // end every pFaceArg->UpperFace->Loopuse

  // Copy Face->Surface to new position
  SmSurface * pSweptSurf;
  double      dSweptFaceTol;
  SmStatus eStat = m_pGeometryCreation->FaceSweepSame(*pContext,         // in : context for new object construction
                                                       pFaceArg,         // in : target face to sweep
                                                       pSweptSurf,       // out: new surface in swept position
                                                       dSweptFaceTol );  // out: new face tolerance
  if ( eStat != SM_SUCCESS )
    {
      SER( eStat );
    }

  // build new face from Copied Surface, Copied TrimCurves, and Copied LoopVertex positions
  // note: The use of MakeFaceWithCurves in this context may be somewhat 
  //       problematic, unless we are sure the original brep contains no 
  //       stitchables with the given sweep tolerance.
  SmFace   * pNewFace = NULL;
  SmShell  * pNewShell = NULL;
  SmRegion * pNewRegion = NULL;
  SER(pBrepToSweep->MakeFaceWithCurves(pRegionArg,                        // in : region to contain new topology objects
                                       sLoopEUCounts,                     // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                                      &s3DCurves,                         // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
                                       NULL,                              // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
                                       sOrients,                          // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                                       sLoopPoints,                       // in : Point positions to build SmVertex VertexLoops
                                       pSweptSurf,                        // in : new face->Surface
                                       pSweptSurf->GetNaturalUVDomain(),  // in : domain of Surface used by face
                                       SM_OT_SAME,                        // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                                       pNewRegion,                        // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                                       pNewShell,                         // out: New shell if any.  Trimmed surfaces always create a new shell.
                                       pNewFace));                        // out: the new face                                                                          
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE pNewFace->ClearLocalZoneTol3d( ) ; 
#else // SM_USE_OLDTOL
  SM_OLDTOL_LINE pNewFace->SetTolerance(dSweptFaceTol); 
#endif // SM_USE_OLDTOL
  
  // for every 3dCurve - set ownerEdge tolerance and add OrigEdge/CopiedEdge pairs to mapping table                                 
  for (ii=0; ii<s3DCurves.GetSize(); ii++) 
    {                           
      SmCurve *pCurve = s3DCurves[ii];                                     
      if (pCurve == NULL) continue;    
      
      // get copiedCurve->Edge (and set its tolerance)                                   
      SmEdge *pEdge = SM_CAST_PTR(SmEdge,pCurve->GetOwner());             
      NER(pEdge); // -hof-: this is the case of the topologies of the orig
                  //        being different from that of the sweep  
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE pEdge->ClearLocalZoneTol3d( ) ;  // GWC:NewTolerance - what's going on with the MaxGaps here?
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE pEdge->SetTolerance(sTolerances[ii]); 
#endif // SM_USE_OLDTOL
      
      // add origEdge/CopiedEdge to mapping                               
      SmEdge *pOrigEdge = sOrigEdges[ii];                                  
      SetSame(pOrigEdge,pEdge);                                           
      
      // get Edge/CopiedEdge vertices                                                                   
      SmVertex * pOrigStartV = pOrigEdge->GetStartVertex();                
      SmVertex * pStartV     = pEdge->GetStartVertex(); 
                              
      SmVertex * pOrigEndV   = pOrigEdge->GetOtherVertex(pOrigStartV);     
      SmVertex * pEndV       = pEdge->GetOtherVertex(pStartV);                 
                                                                           
      // add OrigVertices/CopiedVertices to mapping (when not already mapped)                                  
      SetSame(pOrigStartV, pStartV);                                      
      if(pOrigEndV != pOrigStartV) { SetSame(pOrigEndV, pEndV); }
                                                                          
    } // end iter every s3DCurve updating the Edge/CopiedEdge and Vertex/CopiedVertex mappings
                                                                          
  // map orig/copied VertexLoops                                        
  sm_MapVertexLoops(pFaceArg, pNewFace, *this);                           
  
  // add OrigFace/NewFace mapping                                                                        
  SetSame(pFaceArg,pNewFace);

  // when tagging
  if (m_bDoTagging) 
    {
      // copy SM_AI_TAG by hand  (GWC? should we be using the Notify Mechanism so users can do the same?)
      SmTagAttribute * pFTag    = (SmTagAttribute*)pFaceArg->FindAttribute(SM_AI_TAG);
      NER(pFTag);
      SmTagAttribute * pNewFTag = new(*pFaceArg->GetContext()) SmTagAttribute(*pFTag);
      pNewFTag->m_lPrimaryOrdinal -= SM_TI_SWEEP_STEP_DELTA; // Each sweep step this goes down by 100k
      pNewFace->AddAttribute(pNewFTag);
    }  // end if m_bDoTagging check

  // all done
  return SM_SUCCESS;

} // end SmTopologySweep::SweepFaceSame

/*******************************************************************//**
PURPOSE: Sweep faces to a higher dimension.

NOTES: 
***********************************************************************/
SmStatus SmTopologySweep::DoSweepFacesHigher
  (SmRegion* pRegionArg,
   const SmTArray<SmFace*> & rFacesToSweepHigherArg)
{
    SER(DoSweepFacesSame(pRegionArg,rFacesToSweepHigherArg));

    return SM_SUCCESS;

} // end SmTopologySweep::DoSweepFacesHigher



